"""Сборка штатных образов и съём UART/MMIO в локальном QEMU Espressif."""

import json
import os
from pathlib import Path
import re
import selectors
import shutil
import signal
import subprocess
import sys
import tempfile
import time


HERE = Path(__file__).resolve().parent
TARGET = "esp32c3"
IDF_VERSION = "ESP-IDF v6.1.1"
QEMU_VERSION = "QEMU emulator version 9.2.2 (esp_develop_9.2.2_20260417)"
ARDUINO_VERSION = "4.0.0"
# Arduino ESP32 4.0.0, boards.txt: UART0 для Serial, штатный DIO-загрузчик.
FQBN = "esp32:esp32:esp32c3:CDCOnBoot=default,FlashMode=dio"

# Границы работы инструментов и объём выборки опыта. Время хоста служит сторожем процесса.
BUILD_TIMEOUT_S = 300
QEMU_TIMEOUT_S = 30
UART_LIMIT = 64 * 1024
BLINK_EDGES = 4  # Два SET/CLEAR и три интервала между командами.
BLINK_MARKER = b"Blink GPIO "

# ESP-IDF v6.1.1, components/soc/esp32c3/register/soc/{reg_base,uart_reg,gpio_reg}.h.
UART_FIFO = 0x60000000
GPIO_SET = 0x60004008
GPIO_CLEAR = 0x6000400C
GPIO_ENABLE = 0x60004024
BLINK_MASK = 1 << 3  # Blink/Blink.ino, нумерация битов по TRM v1.4, регистры 5.3, 5.4 и 5.6.


def run(args, cwd, timeout=BUILD_TIMEOUT_S):
    with subprocess.Popen(
        args, cwd=cwd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, start_new_session=True,
    ) as process:
        try:
            result, _ = process.communicate(timeout=timeout)
        except BaseException:
            os.killpg(process.pid, signal.SIGKILL)
            raise
        if process.returncode:
            raise RuntimeError(f"{args[0]} завершился с кодом {process.returncode}\n{result[-6000:]}")
        return result


def check_tools(work, case):
    if run(["idf.py", "--version"], work).strip() != IDF_VERSION:
        raise RuntimeError(f"NotAnalyzed: требуется {IDF_VERSION}")
    if not shutil.which("qemu-system-riscv32"):
        raise RuntimeError("NotAnalyzed: требуется установленный QEMU Espressif для RISC-V")
    if run(["qemu-system-riscv32", "--version"], work).splitlines()[0] != QEMU_VERSION:
        raise RuntimeError(f"NotAnalyzed: требуется {QEMU_VERSION}")
    if case == "blink":
        if not shutil.which("arduino-cli"):
            raise RuntimeError("NotAnalyzed: требуется установленный arduino-cli")
        config = json.loads(run(["arduino-cli", "core", "list", "--json"], work))
        if not any(
            core["id"] == "esp32:esp32" and core.get("installed_version") == ARDUINO_VERSION
            for core in config.get("platforms", [])
        ):
            raise RuntimeError(f"NotAnalyzed: требуется Arduino ESP32 {ARDUINO_VERSION}")


def check_image(path, work):
    info = run([sys.executable, "-m", "esptool", "--chip", TARGET, "image-info", path], work)
    for label in ("Checksum:", "Validation hash:"):
        if not any(line.startswith(label) and line.endswith(" (valid)") for line in info.splitlines()):
            raise RuntimeError(f"esptool не подтвердил {label} в {path.name}\n{info}")


def build_hello(work):
    project = work / "hello_world"
    shutil.copytree(HERE / "hello_world", project)
    run(["idf.py", "-DCCACHE_ENABLE=0", "set-target", TARGET, "build"], project)
    build = project / "build"
    layout = json.loads((build / "flasher_args.json").read_text())
    for item in ("bootloader", "app"):
        check_image(build / layout[item]["file"], work)
    flash = work / "flash.bin"
    run([
        sys.executable, "-m", "esptool", "--chip", TARGET, "merge-bin", "-o", flash,
        "--pad-to-size", layout["flash_settings"]["flash_size"], "@flash_args",
    ], build)
    # Меняем один бит сигнатуры первой записи, адрес берём из flasher_args.json.
    # Формат записи: ESP-IDF components/partition_table/gen_esp32part.py, PartitionDefinition.
    damaged = bytearray(flash.read_bytes())
    damaged[int(layout["partition-table"]["offset"], 0)] ^= 1
    (work / "bad-partition.bin").write_bytes(damaged)


def build_blink(work):
    sketch = work / "Blink"
    shutil.copytree(HERE / "Blink", sketch)
    build = work / "arduino-build"
    # Четыре рабочих процесса для сборки на общей машине с восемью ядрами.
    run(["arduino-cli", "compile", "-j", "4", "--fqbn", FQBN, "--build-path", build, sketch], work)
    for output in ("Blink.ino.bootloader.bin", "Blink.ino.bin"):
        check_image(build / output, work)
    # Arduino ESP32 4.0.0, platform.txt, recipe.hooks.objcopy.postobjcopy.3.
    (build / "Blink.ino.merged.bin").rename(work / "flash.bin")


def capture_uart(work, flash, trace=False):
    # Профиль flash/eFuse и отключение TIMG WDT из ESP-IDF tools/idf_py_actions/qemu_ext.py.
    # shift=3 задаёт 8 нс на инструкцию (степень двойки, ближайшая к 1/160 МГц).
    args = [
        "qemu-system-riscv32", "-M", TARGET, "-nographic", "-monitor", "none",
        "-serial", "stdio", "-nic", "none", "-icount", "shift=3,sleep=off", "-no-reboot",
        "-drive", f"file={flash},if=mtd,format=raw",
        "-drive", "file=efuse.bin,if=none,format=raw,id=efuse",
        "-global", "driver=nvram.esp32c3.efuse,property=drive,value=efuse",
        "-global", "driver=timer.esp32c3.timg,property=wdt_disable,value=true",
    ]
    if trace:
        args += ["-trace", "enable=memory_region_ops_write,file=mmio.log"]

    # Пустой, но открытый stdin сохраняет подключение QEMU stdio на весь прогон.
    deadline = time.monotonic() + QEMU_TIMEOUT_S
    raw = bytearray()
    with (work / "qemu.stderr").open("w+b") as diagnostic, selectors.DefaultSelector() as selector:
        with subprocess.Popen(args, cwd=work, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                              stderr=diagnostic) as process:
            selector.register(process.stdout, selectors.EVENT_READ)
            try:
                while selector.select(max(0, deadline - time.monotonic())):
                    chunk = os.read(process.stdout.fileno(), 4096)
                    if not chunk:
                        if process.wait(timeout=max(0, deadline - time.monotonic())) or trace:
                            raise RuntimeError("QEMU завершился до контрольного события")
                        return bytes(raw)
                    raw.extend(chunk)
                    if len(raw) > UART_LIMIT:
                        raise RuntimeError("QEMU превысил лимит UART до контрольного события")
                    if trace and raw.count(BLINK_MARKER) >= BLINK_EDGES:
                        end = raw.index(BLINK_MARKER)
                        for _ in range(BLINK_EDGES - 1):
                            end = raw.index(BLINK_MARKER, end + len(BLINK_MARKER))
                        newline = raw.find(b"\n", end)
                        if newline != -1:
                            return bytes(raw[:newline + 1])
                    if time.monotonic() >= deadline:
                        break
                raise RuntimeError("QEMU не дошёл до контрольного события в пределах лимита")
            except (RuntimeError, subprocess.TimeoutExpired) as exc:
                diagnostic.seek(0)
                raise RuntimeError(f"{exc}\nUART:\n{raw.decode(errors='replace')}\n"
                                   f"{diagnostic.read().decode(errors='replace')}") from exc
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        process.kill()


def gpio_trace(path, raw_uart):
    # QEMU 9.2.2, system/memory.c, hw/trace-events: берём поля шины, без указателя региона.
    bus_write = re.compile(rb" addr (0x[0-9a-f]+) value (0x[0-9a-f]+) size ([0-9]+) ")
    uart_from_bus = bytearray()
    rows = []
    with path.open("rb") as trace:
        for line in trace:
            match = bus_write.search(line)
            if not match:
                continue
            address, value = (int(field, 16) for field in match.group(1, 2))
            if address == UART_FIFO:
                # ESP32-C3 TRM v1.4, гл. 26, UART_FIFO_REG: UART_RXFIFO_RD_BYTE, биты 7:0.
                uart_from_bus.append(value & 0xFF)
                if len(uart_from_bus) == len(raw_uart):
                    break
            elif address in (GPIO_ENABLE, GPIO_SET, GPIO_CLEAR) and value & BLINK_MASK:
                offset = len(uart_from_bus) - uart_from_bus.count(b"\r")
                rows.append(f"{offset} {address:#010x} {value:#x} {int(match[3])}")
    if uart_from_bus != raw_uart:
        raise RuntimeError("UART0 из MMIO расходится с байтами консоли QEMU")
    return rows


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ("hello_world", "blink"):
        raise RuntimeError("Укажите hello_world или blink")
    case = sys.argv[1]
    repo = HERE.parents[3]
    excluded = (repo, repo.with_name("sim"))
    parent = Path(os.path.abspath(os.environ["SIM_ESP_SDK_TMPDIR"]))
    if not any(parent.is_relative_to(p) for p in excluded):
        parent = parent.resolve(strict=True)
    if any(parent.is_relative_to(p) for p in excluded):
        raise RuntimeError("SIM_ESP_SDK_TMPDIR должен быть вне рабочих копий репозитория")
    if not parent.is_dir():
        raise RuntimeError("SIM_ESP_SDK_TMPDIR должен быть существующим каталогом")
    with tempfile.TemporaryDirectory(prefix="esp-sdk-", dir=parent) as directory:
        work = Path(directory)
        scratch = work / "tmp"
        scratch.mkdir()
        os.environ.update(TMPDIR=str(scratch), TMP=str(scratch), TEMP=str(scratch),
                          ARDUINO_BUILD_CACHE_PATH=str(work / "arduino-cache"),
                          ARDUINO_UPDATER_ENABLE_NOTIFICATION="false")
        check_tools(work, case)
        sys.path.insert(0, str(Path(os.environ["IDF_PATH"]) / "tools"))
        from idf_py_actions.qemu_ext import QEMU_TARGETS

        efuse = QEMU_TARGETS[TARGET].default_efuse
        (work / "efuse.bin").write_bytes(efuse)
        if case == "hello_world":
            build_hello(work)
            uart = capture_uart(work, "flash.bin")
            (work / "efuse.bin").write_bytes(efuse)
            rejected = capture_uart(work, "bad-partition.bin")
            if (b"Failed to verify partition table" not in rejected
                or b"Calling app_main()" in rejected or b"Hello world!" in rejected):
                raise RuntimeError(f"QEMU не подтвердил отказ по таблице разделов\n{rejected!r}")
            rows = []
        else:
            build_blink(work)
            uart = capture_uart(work, "flash.bin", trace=True)
            rows = gpio_trace(work / "mmio.log", uart)
        if not uart.endswith(b"\n"):
            raise RuntimeError(f"Неполная трасса UART {case}\n{uart!r}")
        # Смещения MMIO отсчитываются в каноническом UART с LF.
        sys.stdout.write(uart.decode("ascii").replace("\r\n", "\n"))
        if case == "blink":
            sys.stdout.write("--- GPIO MMIO: байты UART, адрес, значение, ширина ---\n")
            sys.stdout.writelines(f"{row}\n" for row in rows)


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
        sys.exit(str(error))
