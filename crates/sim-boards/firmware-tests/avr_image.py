"""Вручную закодированная программа AVRe: DS40002198C, адреса 7810D."""

import struct
from pathlib import Path

words = [0xFFFF] * 52  # 26 векторов ATmega328P по два слова.
labels = {}
jumps = []


def emit(*opcodes):
    words.extend(opcodes)


def label(name):
    labels[name] = len(words)


def jump(name, op=0xC000):
    jumps.append((len(words), op, name))
    emit(0)


def ldi(r, k):
    emit(0xE000 | (k & 0xF0) << 4 | (r - 16) << 4 | (k & 15))


def reg(op, d, r):
    emit(op | (r & 16) << 5 | d << 4 | (r & 15))


def out(a, r):
    emit(0xB800 | (a & 0x30) << 5 | r << 4 | (a & 15))


def stores(a, r):
    emit(0x9200 | r << 4, a)


def loads(r, a):
    emit(0x9000 | r << 4, a)


def set_reg(a, k):
    ldi(16, k)
    stores(a, 16)


def send(r, name):
    label(name)
    loads(23, 0xC0)  # UCSR0A.UDRE0.
    emit(0xFE00 | 23 << 4 | 5)  # SBRS r23,5.
    jump(name)
    stores(0xC6, r)


label("reset")
set_reg(0xC4, 0)
set_reg(0xC1, 8)
ldi(16, 0x28)
out(4, 16)  # DDRB: D13 = PB5, D11 = PB3 / OC2A.
loads(16, 0x54)
emit(0xFE00 | 16 << 4 | 3)  # При WDRF пропускаем переход.
jump("normal")
set_reg(0x54, 0)
emit(0x95A8)  # WDR.
ldi(16, 0x18)
ldi(17, 0)
stores(0x60, 16)
stores(0x60, 17)  # Окно WDCE: две соседние STS по два такта.
ldi(21, ord("R"))
send(21, "send_reset")
ldi(16, 0x20)
out(5, 16)
jump("idle")

label("normal")
emit(0x9B00 | 9 << 3 | 7)  # SBIS PIND,7: вход стенда должен быть высоким.
emit(0x9598)  # BREAK.
ldi(16, 255)
ldi(17, 1)
ldi(18, 0)
reg(0x0C00, 16, 17)  # ADD: C=1, Z=1.
jump("bad", 0xF401)  # BRNE.
reg(0x1C00, 18, 18)  # ADC: 0+0+C => 1.
reg(0x9C00, 18, 17)  # MUL: r1:r0 = 1.
emit(0x920F | 0 << 4, 0x900F | 19 << 4)  # PUSH r0, POP r19.
emit(0x3000 | (19 - 16) << 4 | 1)  # CPI r19,1.
jump("bad", 0xF401)
ldi(16, 127)
emit(0x9403 | 16 << 4)  # INC: V=1 при 0x7f → 0x80.
jump("bad", 0xF403)  # BRVC.
ldi(19, 21)
ldi(20, 0)
ldi(17, 3)
label("divide")
reg(0x1400, 19, 17)  # CP, C=1 при заёме.
jump("quotient", 0xF000)  # BRCS.
reg(0x1800, 19, 17)
emit(0x9403 | 20 << 4)  # INC r20.
jump("divide")
label("quotient")
emit(0x3000 | (20 - 16) << 4 | 7)
jump("checked", 0xF001)  # BREQ.
label("bad")
emit(0x9598)  # BREAK, отрицательный исход арифметического самотеста.
label("checked")

set_reg(0xB3, 63)  # OCR2A: 64 высоких такта из 256.
set_reg(0xB0, 0x83)
set_reg(0xB1, 2)
set_reg(0x7C, 0x40)  # AVCC, ADC0.
set_reg(0x7A, 0xC7)  # ADEN | ADSC; делитель 128, ADC 125 кГц.
label("adc_wait")
loads(16, 0x7A)
emit(0xFC00 | 16 << 4 | 6)  # SBRC r16,6.
jump("adc_wait")
loads(21, 0x78)
loads(22, 0x79)
send(22, "send_high")
send(21, "send_low")

set_reg(0x47, 249)  # OCR0A: 16 МГц / 64 / 250 = 1 кГц.
set_reg(0x44, 2)  # CTC.
set_reg(0x6E, 2)  # TIMSK0.OCIE0A.
set_reg(0x45, 3)

emit(0x95A8)
ldi(16, 0x18)
ldi(17, 8)
stores(0x60, 16)
stores(0x60, 17)
emit(0x9478)  # SEI.
label("idle")
jump("idle")
label("timer0")
emit(0x920F | 24 << 4)
ldi(24, 0x20)
out(3, 24)  # Запись PINB переключает PORTB.5.
emit(0x900F | 24 << 4, 0x9518)  # POP, RETI.

jumps.extend([(0, 0xC000, "reset"), (0x38 // 2, 0xC000, "timer0")])
for pos, op, destination in jumps:
    relative = labels[destination] - pos - 1
    if op == 0xC000:
        assert -2048 <= relative <= 2047
        words[pos] = op | (relative & 4095)
    else:
        assert -64 <= relative <= 63
        words[pos] = op | (relative & 127) << 3

image = struct.pack(f"<{len(words)}H", *words)


def record(address, kind, data):
    fields = bytes((len(data),)) + address.to_bytes(2, "big") + bytes((kind,)) + data
    return ":" + (fields + bytes((-sum(fields) & 255,))).hex().upper() + "\n"


target = Path(__file__).resolve().parent.parent / "testdata"
target.joinpath("avr.hex").write_text(
    "".join(record(i, 0, image[i:i + 16]) for i in range(0, len(image), 16)) + record(0, 1, b""),
    encoding="ascii",
)
# System V ELF32: ET_EXEC, EM_AVR, EF_AVR_MACH_AVR5 (GNU binutils), один PT_LOAD.
ident = b"\x7fELF\x01\x01\x01" + bytes(9)
header = struct.pack("<16sHHIIIIIHHHHHH", ident, 2, 83, 1, 0, 52, 0, 5, 52, 32, 1, 0, 0, 0)
program = struct.pack("<8I", 1, 84, 0, 0, len(image), len(image), 5, 2)
target.joinpath("avr.elf").write_bytes(header + program + image)
