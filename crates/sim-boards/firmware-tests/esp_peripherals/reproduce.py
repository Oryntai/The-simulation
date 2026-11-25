"""Пересборка flash и повторяемая выборка UART, MMIO и PC в QEMU Espressif."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import select
import shutil
import signal
import subprocess
import sys
import tempfile
import time


PROJECT = Path(__file__).resolve().parent
DATA = PROJECT.parents[1] / "tests/data/esp_peripherals"
CHIP = "esp32c3"
IDF_VERSION = "ESP-IDF v6.1.1"
QEMU_VERSION = "QEMU emulator version 9.2.2 (esp_develop_9.2.2_20260417)"

# Бюджет стенда. Время хоста используется только для управления внешними процессами.
BUILD_TIMEOUT = 300
QEMU_TIMEOUT = 15
QEMU_LOG_LIMIT = 64 * 1024 * 1024
MAX_ARTIFACT_SIZE = 2_000_000  # Ограничение размера файлов задачи 2.21.
FRAME = bytes.fromhex("55 a6 00 ff")  # Кадр стенда в main/esp_peripherals.c.

# ESP-IDF components/soc/esp32c3/register/soc/{reg_base,gpio_reg,uart_reg,
# ledc_reg,i2c_reg,rtc_cntl_reg,system_reg}.h. Адреса абсолютные, поля по тем же файлам.
GPIO_SET, GPIO_CLEAR, GPIO_ENABLE = 0x60004008, 0x6000400C, 0x60004024
GPIO_MASK = 1 << 3  # GPIO_OUT из прошивки, GPIO_OUT_W1TS/W1TC и GPIO_ENABLE_W1TS.
UART_FIFO, UART_CLKDIV = 0x60010000, 0x60010014
UART_BYTE_MASK = 0xFF  # TRM гл. 26, UART_RXFIFO_RD_BYTE[7:0].
LEDC_TIMER, LEDC_DUTY = 0x600190A0, 0x60019008
I2C_CONTROL, I2C_FIFO = 0x60013004, 0x6001301C
I2C_START = 1 << 5  # I2C_TRANS_START.
RTC_TIMER_LOW, RTC_TIMER_HIGH = 0x60008004, 0x60008008
RTC_TIMER_ALARM = 1 << 16  # RTC_CNTL_MAIN_TIMER_ALARM_EN.
RTC_STATE, RTC_SLEEP_ENABLE = 0x60008018, 1 << 31  # RTC_CNTL_SLEEP_EN.
FASTMEM_CONTROL, CRC_START, CRC_FINISH = 0x600C0048, 1 << 8, 1 << 31

# QEMU system/memory.c и hw/trace-events. Указатели хоста и времена из лога не сохраняются.
BUS = re.compile(
    rb"memory_region_ops_(read|write).* addr (0x[0-9a-f]+) value (0x[0-9a-f]+) size ([0-9]+) name '([^']+)'"
)
BRANCH = re.compile(rb"^(0x[0-9a-f]+):.*\bbeqz\s+t0,")


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def run(args, cwd):
    # Отдельная группа нужна для остановки дочерних процессов сборки при ошибке.
    with subprocess.Popen(args, cwd=cwd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, text=True, start_new_session=True) as process:
        try:
            out, _ = process.communicate(timeout=BUILD_TIMEOUT)
        except BaseException:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
            raise
    require(process.returncode == 0, f"{args[0]}: код {process.returncode}\n{out[-6000:]}")
    return out.strip()


def esptool(args, cwd):
    return run([sys.executable, "-m", "esptool", "--chip", CHIP, *args], cwd)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def build(work, out):
    require(run(["idf.py", "--version"], work) == IDF_VERSION,
            f"NotAnalyzed. Нужен {IDF_VERSION}.")
    app = work / "project"
    app.mkdir()
    shutil.copytree(PROJECT / "main", app / "main")
    for name in ("CMakeLists.txt", "sdkconfig.defaults"):
        shutil.copyfile(PROJECT / name, app / name)

    log = run(["idf.py", "-DCCACHE_ENABLE=0", "set-target", CHIP, "build"], app)
    require("Project build complete." in log, "ESP-IDF не подтвердил завершение сборки")
    print("ESP-IDF: Project build complete.", flush=True)
    directory = app / "build"
    layout = json.loads((directory / "flasher_args.json").read_text())
    flash = out / "flash.bin"
    padded = work / "padded.bin"
    esptool(["merge-bin", "-o", flash, "@flash_args"], directory)
    esptool(["merge-bin", "-o", padded, "--pad-to-size", layout["flash_settings"]["flash_size"],
             "@flash_args"], directory)
    data = flash.read_bytes()
    padded_data = padded.read_bytes()
    # Стертое состояние flash = 0xff, как при дополнении raw-образа в esptool merge-bin.
    require(padded_data.startswith(data) and set(padded_data[len(data):]) == {0xFF},
            "Двоичные образы flash для QEMU и стенда разошлись")

    regions = []
    for name in ("bootloader", "partition-table", "app"):
        region = layout[name]
        offset = int(region["offset"], 0)
        length = (directory / region["file"]).stat().st_size
        image = data[offset:offset + length]
        require(len(image) == length, f"Образ flash обрезан в регионе {name}")
        if name != "partition-table":
            # Проверяем байты из готового flash, включая заголовок, обновлённый merge-bin.
            image_path = work / f"{name}.bin"
            image_path.write_bytes(image)
            info = esptool(["image-info", image_path], work)
            for label in ("Checksum:", "Validation hash:"):
                require(any(line.startswith(label) and line.endswith(" (valid)")
                            for line in info.splitlines()), f"{name}: esptool не подтвердил {label}\n{info}")
        regions.append({"name": name, "offset": region["offset"], "size": length, "sha256": sha(image)})
    print("esptool: XOR и SHA-256 загрузчика и приложения подтверждены.", flush=True)

    manifest = {"chip": CHIP, "esp_idf": IDF_VERSION, "qemu": QEMU_VERSION,
                "esptool": run([sys.executable, "-m", "esptool", "version"], work).splitlines()[0],
                "flash_settings": layout["flash_settings"], "flash_capacity_bytes": len(padded_data),
                "regions": regions}
    (out / "flash-layout.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return directory / "esp_peripherals.elf"


def capture(work, reply):
    # Профиль из ESP-IDF tools/idf_py_actions/qemu_ext.py. 2^3 нс на инструкцию
    # — масштаб QEMU icount, sleep=on позволяет доставить ответ UART во время idle.
    # QEMU 9.2 заменил -singlestep на TCG one-insn-per-tb (Options -accel).
    args = [
        "qemu-system-riscv32", "-M", CHIP, "-nographic", "-monitor", "none", "-nic", "none",
        "-serial", "file:console.raw", "-serial", "stdio", "-icount", "shift=3,sleep=on", "-no-reboot",
        "-drive", "file=flash.bin,if=mtd,format=raw",
        "-drive", "file=efuse.bin,if=none,format=raw,id=efuse",
        "-global", "driver=nvram.esp32c3.efuse,property=drive,value=efuse",
        "-global", "driver=timer.esp32c3.timg,property=wdt_disable,value=true",
        "-accel", "tcg,one-insn-per-tb=on", "-d", "in_asm",
        "-trace", "enable=memory_region_ops_*,file=bus.raw",
    ]
    watched = {
        GPIO_SET, GPIO_CLEAR, GPIO_ENABLE, UART_FIFO, UART_CLKDIV, LEDC_TIMER, LEDC_DUTY,
        I2C_CONTROL, I2C_FIFO, RTC_TIMER_LOW, RTC_TIMER_HIGH, RTC_STATE, FASTMEM_CONTROL,
    }
    console = work / "console.raw"
    tx = bytearray()
    events = []
    pc = None
    crc_polls = 0
    crc_started = False
    pending = b""
    cursor = 0
    trace = work / "bus.raw"
    deadline = time.monotonic() + QEMU_TIMEOUT
    with (work / "stderr.raw").open("wb") as err:
        with subprocess.Popen(args, cwd=work, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                              stderr=err) as child:
            try:
                while crc_polls < 2 and time.monotonic() < deadline:
                    if select.select([child.stdout], [], [], 0.01)[0]:
                        chunk = os.read(child.stdout.fileno(), 4096)
                        require(chunk, f"QEMU завершился до проверки, код {child.poll()}")
                        tx.extend(chunk)
                        require(FRAME.startswith(tx), f"Неверный вывод UART1: {tx.hex(' ')}")
                        if tx == FRAME and reply:
                            child.stdin.write(reply)
                            child.stdin.flush()

                    if not trace.exists():
                        continue
                    require(trace.stat().st_size <= QEMU_LOG_LIMIT, "Превышен лимит лога QEMU")
                    with trace.open("rb") as log:
                        log.seek(cursor)
                        # Читаем порциями, чтобы снова обслужить UART1 между выборками трассы.
                        chunk = log.read(1024 * 1024)
                        cursor = log.tell()
                    *lines, pending = (pending + chunk).split(b"\n")
                    for line in lines:
                        if crc_polls == 1 and (match := BRANCH.match(line)):
                            pc = match[1].decode()
                        match = BUS.match(line)
                        if not match:
                            continue
                        op, address, value, width, device = match.groups()
                        address, value, width = int(address, 16), int(value, 16), int(width)
                        if address not in watched:
                            continue
                        if op == b"write":
                            events.append((op, address, value, width, device))
                            if address == FASTMEM_CONTROL and value & CRC_START:
                                crc_started = True
                        elif address == UART_FIFO:
                            events.append((op, address, value, width, device))
                        elif address == FASTMEM_CONTROL and crc_started:
                            require(not value & CRC_FINISH, "QEMU выставил флаг CRC, эталон требует обновления")
                            crc_polls += 1
                            events.append((op, address, value, width, device))
                            if crc_polls == 2:
                                break
                require(crc_polls == 2 and pc, "Не наблюдалось двух чтений флага CRC и перехода PC между ними")
            finally:
                if child.poll() is None:
                    child.terminate()
                    try:
                        child.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        child.kill()

    require(tx == FRAME, "На UART1 не получен полный кадр")
    markers = [line for line in console.read_text(encoding="ascii").splitlines()
               if line.startswith("FIXTURE ")]
    return markers, events, pc


def describe(markers, events, pc, reply, elf, work):
    def writes(address):
        return [value for op, reg, value, _, _ in events if reg == address and op == b"write"]

    def uart_bytes(operation):
        return bytes(value & UART_BYTE_MASK for op, reg, value, _, _ in events
                     if reg == UART_FIFO and op == operation)

    require(uart_bytes(b"write") == FRAME and uart_bytes(b"read") == reply,
            "Байты хоста и обращения к UART1 FIFO разошлись")
    # TRM гл. 26, UART_CLKDIV_REG. XTAL = 40 МГц, предделитель SCLK = 1
    # при 115200 бод (ESP-IDF components/esp_hal_uart/esp32c3/include/hal/uart_ll.h).
    divisors = writes(UART_CLKDIV)
    require(divisors, "В MMIO отсутствует настройка делителя UART1")
    divider = divisors[-1]
    divisor_x16 = (divider & 0xFFF) * 16 + ((divider >> 20) & 0xF)
    require(divisor_x16 == (40_000_000 * 16) // 115_200, "Неверный делитель UART1")

    gpio = [(reg, value, width) for op, reg, value, width, _ in events
            if op == b"write" and reg in (GPIO_SET, GPIO_CLEAR, GPIO_ENABLE) and value & GPIO_MASK]
    require(gpio == [(reg, GPIO_MASK, 4) for reg in
                     (GPIO_CLEAR, GPIO_ENABLE, GPIO_CLEAR, GPIO_SET, GPIO_CLEAR)],
            "Последовательность GPIO3 на шине не совпала с командами прошивки")
    # TRM гл. 32: CLK_DIV_LSTIMER0[21:4] (8 дробных бит), DUTY_RES[3:0];
    # ESP-IDF components/esp_hal_ledc/esp32c3/include/hal/ledc_ll.h: 4 дробных бита DUTY.
    ledc = writes(LEDC_TIMER)
    require(((40_000_000 * 256 // (4000 * (1 << 10))) << 4) in ledc and 10 in ledc and
            writes(LEDC_DUTY) == [256 << 4], "В MMIO отсутствует конфигурация LEDC")
    # TRM гл. 28: первый байт адреса равен (7-битный адрес << 1) | R/W.
    require(writes(I2C_FIFO) == [0x50 << 1, 0x51 << 1] and
            sum(bool(value & I2C_START) for value in writes(I2C_CONTROL)) == 2,
            "На шине не видно запуска обеих проб I²C")
    require(RTC_TIMER_ALARM in writes(RTC_TIMER_HIGH) and bool(writes(RTC_TIMER_LOW)) and
            not any(value & RTC_SLEEP_ENABLE for value in writes(RTC_STATE)),
            "Изменился путь SDK от взвода таймера до барьера CRC")

    irq_result = "match" if reply == FRAME else "fail"
    uart_line = f"FIXTURE uart tx=4 rx={len(reply)}" + (
        f" {reply.hex(' ')}" if reply else ""
    ) + f" {irq_result}"
    # Без модели GPIO/LEDC/I²C остаются биты 0, 1, 3, с неверным UART также бит 2.
    failures = 0xB if reply == FRAME else 0xF
    require(markers == [
        "FIXTURE cold",
        "FIXTURE gpio out=0 in=0", "FIXTURE gpio out=1 in=0", "FIXTURE gpio out=0 in=0",
        uart_line, "FIXTURE ledc begin hz=4000 duty=256,768",
        "FIXTURE error ledc duty ESP_ERR_INVALID_ARG",
        "FIXTURE i2c begin hz=100000 address=0x50",
        "FIXTURE i2c probe 0x50 ESP_ERR_TIMEOUT", "FIXTURE i2c probe 0x51 ESP_ERR_TIMEOUT",
        f"FIXTURE sleep timer us=2000000 failures=0x{failures:x}",
    ], "Неожиданный журнал UART0\n" + "\n".join(markers))
    symbol = run(["riscv32-esp-elf-addr2line", "-f", "-e", elf, pc], work).splitlines()[0]
    require(symbol == "rtc_deep_sleep_start", f"Переход PC {pc} находится в {symbol}")

    lines = [*markers, f"UART1 host TX={tx_repr(reply)} RX={tx_repr(FRAME)}"]
    lines.extend(f"GPIO write {reg:#010x} {value:#x} {width}" for reg, value, width in gpio)
    lines.append(f"UART1 CLKDIV {divider:#x} divisor_x16={divisor_x16}")
    summaries = [
        ("LEDC timer", LEDC_TIMER), ("LEDC duty", LEDC_DUTY), ("I2C FIFO", I2C_FIFO),
        ("RTC timer low", RTC_TIMER_LOW), ("RTC timer high", RTC_TIMER_HIGH),
    ]
    for title, register in summaries:
        values = " ".join(f"{value:#x}" for value in writes(register))
        device = next(dev.decode() for op, reg, _, _, dev in events if reg == register and op == b"write")
        lines.append(f"{title} {register:#010x} {values} [{device}]")
    crc = [f"{op.decode()}={value:#x}" for op, reg, value, _, _ in events if reg == FASTMEM_CONTROL]
    lines.append(f"RTC FASTMEM CRC {FASTMEM_CONTROL:#010x} {' '.join(crc)}")
    lines.append(f"PC {pc} {symbol}: beqz между двумя чтениями CRC")
    return lines


def tx_repr(data):
    return data.hex(" ") if data else "(пусто)"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true", help="перезаписать проверенные эталоны")
    options = parser.parse_args()
    qemu = shutil.which("qemu-system-riscv32")
    require(qemu, "NotAnalyzed. Нужен установленный QEMU Espressif для RISC-V.")
    with tempfile.TemporaryDirectory(prefix="t221-esp-", dir="/tmp") as scratch:
        work = Path(scratch)
        tmp = work / "tmp"
        tmp.mkdir()
        os.environ.update(TMPDIR=str(tmp), TMP=str(tmp), TEMP=str(tmp))
        require(run([qemu, "--version"], work).splitlines()[0] == QEMU_VERSION,
                f"NotAnalyzed. Нужен {QEMU_VERSION}.")
        out = work / "output"
        out.mkdir()
        elf = build(work, out)
        sys.path.insert(0, str(Path(os.environ["IDF_PATH"]) / "tools"))
        from idf_py_actions.qemu_ext import QEMU_TARGETS

        efuse = out / "qemu-efuse.bin"
        efuse.write_bytes(QEMU_TARGETS[CHIP].default_efuse)
        trace = [f"# {QEMU_VERSION}", "# UART0, UART1, MMIO и PC до второго чтения CRC"]
        # Отрицательные контроли: испорченный последний байт, затем отсутствие ответа.
        for case, reply in (("echo", FRAME), ("corrupt", FRAME[:-1] + b"\xfe"), ("silent", b"")):
            instance = work / case
            instance.mkdir()
            shutil.copyfile(work / "padded.bin", instance / "flash.bin")
            shutil.copyfile(efuse, instance / "efuse.bin")
            markers, events, pc = capture(instance, reply)
            trace.extend(["", f"[{case}]", *describe(markers, events, pc, reply, elf, instance)])
            print(f"QEMU {case}: UART FIFO, GPIO MMIO и остановка на CRC сверены.", flush=True)
        (out / "qemu.trace").write_text("\n".join(trace) + "\n", encoding="utf-8")
        artifacts = sorted(out.iterdir())
        require(all(0 < file.stat().st_size < MAX_ARTIFACT_SIZE for file in artifacts),
                "Размер эталонного файла вышел за пределы задания")
        hashes = out / "SHA256SUMS"
        hashes.write_text("".join(f"{sha(file.read_bytes())}  {file.name}\n" for file in artifacts),
                          encoding="ascii")
        for artifact in [*artifacts, hashes]:
            destination = DATA / artifact.name
            if options.write:
                shutil.copyfile(artifact, destination)
            else:
                require(destination.read_bytes() == artifact.read_bytes(),
                        f"Расхождение с эталоном {artifact.name}")
        print("Т2.21: 5 файлов записаны." if options.write else "Т2.21: 5 файлов совпали побайтно.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
        sys.exit(str(error))
