"""Байты конфигурации и расписание датаграмм со стороны хоста.

Раскладка EXOC v4 и сообщения 0xC1 взяты из sim-mcu/src/exo.rs.
Уставки взяты из make_config исходного контура, расписание из Protocol::ble_default.
Скрипт не читает выходы прошивки и журналы движения.
"""
from pathlib import Path
import math
import struct
import zlib

root = Path(__file__).resolve().parent
tasks = root / "tasks"
reference = "Численный стенд: 50 мс отправитель, 15 мс событие BLE, общее время измерения 25 мкс."
step_ns = 25_000               # Шаг исходного контура из golden/exo_loop/manifest.txt.
slot_ns = 15_000_000           # Protocol::ble_default.
step_s = step_ns / 1_000_000_000
slot_s = slot_ns / 1_000_000_000
horizon_ns = 3_000_000_000      # Горизонт восьми режимов сверки из tasks/acceptance.toml.
fault_delay_ns = 60_000_000    # fault_sweep.packet_delay_ns.

# PacketChannel принимает в слот только пакеты с sent_at <= next_event,
# а хост выгружает доставленные пакеты на первом шаге с now >= next_event.
# Времена сравниваются после накопления f64, как в прежнем контуре. В рантайм
# передаются только целочисленные границы; пакеты и эталонные выходы не читаются.
rows = ["accepts_through_ns,delivers_at_ns"]
now = next_slot = 0.0
ready_through_ns = horizon_ns + fault_delay_ns
for tick in range((ready_through_ns + slot_ns) // step_ns + 2):
    while next_slot <= now:
        cutoff = tick if next_slot == now else tick - 1
        rows.append(f"{cutoff * step_ns},{tick * step_ns}")
        next_slot += slot_s
    now += step_s
(root / "telemetry-slots.csv").write_text("\n".join(rows) + "\n")

# Масса, плечо, тензор и расстояние из device.toml, параметры мотора из sim-em.
mass = 1.3425982370937886
com = 0.17537552376904827
hand = 0.3133606681578101
j = 0.025182383069949632 + mass * com**2 + 10 * hand**2 + 1.36825
gear_tpa = 0.231 * 50 * 0.85
wn = math.tau * 2
wc = math.tau * 500
kp = j * wn * wn
kd = 2 * 0.8 * wn * j
sc = math.pi / 180 / gear_tpa
speed_limit = min(40, math.degrees((0.75 * 2 * 140) / math.sqrt(75000 * j)))
for hang in (0, 100):
    block = struct.pack("<4sIII20fIII10f", b"EXOC", 4, 0x98, 0,
        0.231, 50, 0.85, 8.12, 12.18, 6, mass, com, hand,
        5, math.degrees(2.2)-5, 70, 90, kp * sc, 0.5 * kp * sc, kd * sc,
        0.00107 * wc, 0.844 * wc, 2000, 6 * 3.63,
        50, hang, 0,
        0.844, 25, 1.82, 54.3, 125, 1.5, 3, 0.2, speed_limit, 14)
    (root / f"config-{hang}.bin").write_bytes(block + struct.pack("<I", zlib.crc32(block)))

cases = {
    "assist": (1, 90, 10, ""),
    "wrong_payload": (1, 90, 0, ""),
    "cold": (1, 90, 10, ""),
    "hold_relaxed": (2, 60, 10, ""),
    "fault_hang": (2, 60, 10, """[[board_action]]
id = "перезапуск со сторожем"
board = "плата"
kind = "restart"
at_ns = 1000000000
config = "config-100.bin"
source = "Ввод конфигурации с зависанием через 100 мс, контракт эталона."
"""),
    "estop": (2, 60, 10, """[[board_action]]
id = "аварийная кнопка"
board = "плата"
kind = "gpio"
at_ns = 1000000000
pin = 0
high = false
source = "Активный низкий уровень из схемы прошивки, ввод через секунду."
"""),
    "sensor": (2, 60, 10, """[[fault]]
id = "обрыв входа"
target = "sensor:угол"
at_ns = 1000000000
kind = "drop"
source = "Обрыв цепи датчика, вход ADC переводится на 0 В."
"""),
    "radio": (2, 60, 10, """[[fault]]
id = "потеря связи"
target = "channel:радио"
at_ns = 1000000000
kind = "drop"
source = "Полная потеря команд после первой секунды."

[[fault]]
id = "потеря обратной связи"
target = "channel:ответ"
at_ns = 1000000000
kind = "drop"
source = "Оракул выставляет PER=1 для обоих направлений при потере связи."
"""),
    "open": (2, 60, 10, """[[fault]]
id = "разомкнутый"
target = "channel:радио"
at_ns = 0
kind = "drop"
source = "Сравнение с отсутствием команд хоста с первого такта."
"""),
}

# Расписание численного оракула: накопленное время хоста сравнивается с
# накопленными временами события BLE и телефона. Перевод результата в целые
# наносекунды нужен, чтобы сохранить выбор такта при совпадении границ.
for name, (mode, target, payload, actions) in cases.items():
    now = next_command = next_slot = 0.0
    pending = []
    serial = 0
    lines = [f"# {reference}\n", actions]
    for tick in range(horizon_ns // step_ns):
        if now >= next_command:
            serial += 1
            packet = struct.pack("<BBBHHB", 0xc1, mode, 70, target*10, payload*10, serial)
            pending.append((now, packet))
            next_command += 0.05
        while next_slot <= now:
            while pending and pending[0][0] <= next_slot:
                _, packet = pending.pop(0)
                lines.append(f"""\n[[transmission]]
at_ns = {tick * step_ns}
route = "радио"
bytes = {list(packet)}
source = "{reference}"
""")
            next_slot += slot_s
        now += step_s
    (tasks / f"{name}-wiring.toml").write_text("".join(lines))
