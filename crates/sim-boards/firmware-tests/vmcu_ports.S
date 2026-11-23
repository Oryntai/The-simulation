# Регистры VMCU-C3 из crates/sim-mcu/MEMORY_MAP.md.
# Конфигурация стенда: первый байт печатается, слово +4 задаёт частоту PWM.
.section .text.start,"ax"
.global _start
_start:
    li s0, 0x60000000
    li s1, 0x50000000
    li s2, 0x60004000
    lbu t0, 0(s1)
    sw t0, 0(s2)
    li t0, 0x60001000
    li t1, 8
    sw t1, 0x10(t0)
    lw t2, 0x14(t0)
    andi t2, t2, 1
    slli t2, t2, 3
    sw t2, 0(t0)
    li t0, 0x60003000
    lw t1, 0(t0)
    slli t1, t1, 4
    li t0, 0x60002000
    sw t1, 0(t0)
    lw t1, 4(s1)
    sw t1, 4(t0)
    li t1, 1
    sw t1, 8(t0)

    lw t1, 4(s0)
    li t2, 2
    beq t1, t2, restarted
    lbu t1, 1(s1)
    beqz t1, exchange
    li t0, 0x60006000
    li t1, 30                 # Заданный стендом watchdog 30 мкс.
    sw t1, 4(t0)
    li t1, 1
    sw t1, 0(t0)
1:
    wfi
    j 1b

restarted:
    li t1, 2
    sw t1, 0x18(s0)
2:
    j 2b

exchange:
    lw t1, 8(s2)
    andi t1, t1, 1
    beqz t1, exchange
    lw t1, 4(s2)
    sw t1, 0(s2)
    li t0, 0x60005000
3:
    lw t1, 8(t0)
    beqz t1, 3b
    lbu t1, 0x200(t0)
    sb t1, 0x100(t0)
    lw t1, 0xc(t0)
    sb t1, 0x101(t0)
    li t1, 2
    sw t1, 4(t0)
    sw t1, 0x10(t0)
    sw zero, 0x18(s0)
4:
    j 4b
