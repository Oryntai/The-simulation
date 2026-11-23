# ESP32-C3 TRM, главы GPIO Matrix, UART, I²C и SPI. Только реальные MMIO.
.section .text.start,"ax"
.global _start
_start:
    li t0, 0x60009058           # IO_MUX_GPIO21: функция GPIO.
    li t1, 0x1000
    sw t1, 0(t0)
    li t0, 0x600045a8           # GPIO_FUNC21_OUT_SEL: U0TXD = 6.
    li t1, 6
    sw t1, 0(t0)
    li t0, 0x60009004           # GPIO0: вход для отрицательного теста.
    li t1, 0x1200
    sw t1, 0(t0)
    li s0, 0x60013000           # I²C0: XTAL=40 МГц, SCL 100 кГц.
    li t1, 199
    sw t1, 0x00(s0)
    li t1, 200
    sw t1, 0x38(s0)
    li t1, 4
    sw t1, 0x40(s0)
    li t1, 6
    sw t1, 0x44(s0)
    li t1, 8
    sw t1, 0x48(s0)
    li t1, 10
    sw t1, 0x4c(s0)
    li t1, 0x200000
    sw t1, 0x54(s0)
    li t1, 0x6c                # ams DS000365: адрес 0x36, бит R/W=0.
    sw t1, 0x1c(s0)
    li t1, 0x0c                # RAW ANGLE, старший байт.
    sw t1, 0x1c(s0)
    li t1, 0x6d                # Тот же адрес на чтение.
    sw t1, 0x1c(s0)
    li t1, 0x3000              # RESTART, WRITE(ACK), RESTART, WRITE(ACK).
    sw t1, 0x58(s0)
    li t2, 0x0901
    sw t2, 0x5c(s0)            # Адрес и полезные данные отдельными WRITE.
    sw t2, 0x60(s0)
    sw t1, 0x64(s0)
    sw t2, 0x68(s0)
    li t1, 0x1801              # READ 1 с ACK.
    sw t1, 0x6c(s0)
    li t1, 0x1c01              # READ 1 с NACK.
    sw t1, 0x70(s0)
    li t1, 0x1000              # STOP.
    sw t1, 0x74(s0)
    li t1, 0x133               # Ведущий, CLK_EN, open drain, TRANS_START.
    sw t1, 4(s0)
1:
    lw t1, 0x20(s0)
    andi t2, t1, 0x80
    beqz t2, 1b
    andi t2, t1, 0x400
    bnez t2, nack
    lw a0, 0x1c(s0)
    jal send
    lw a0, 0x1c(s0)
    jal send

    li s1, 0x60024000          # SPI2: PLL80, pre=8, n=10, SCLK 1 МГц.
    li t1, 7
    sw t1, 0xe8(s1)
    li t1, 0x1c9109
    sw t1, 0x0c(s1)
    sw zero, 0x08(s1)          # Оба направления MSB-first.
    li t1, 0x3b                # Активен CS2, CPOL=0.
    sw t1, 0x20(s1)
    li t1, 0x180002c1          # SPI mode 1, полный дуплекс, CS setup/hold.
    sw t1, 0x10(s1)
    li t1, 0x00820000          # Setup 2.5, hold 3.5 периода.
    sw t1, 0x14(s1)
    li t1, 15                  # 16 бит, два байта.
    sw t1, 0x1c(s1)
    li t1, 0xffff              # ams DS000298: READ 0x3fff и чётная парность.
    jal spi
    li t1, 0                   # NOP читает ответ на предыдущий запрос.
    jal spi
    lw t1, 0x98(s1)
    andi a0, t1, 255
    jal send
    lw t1, 0x98(s1)
    srli a0, t1, 8
    andi a0, a0, 255
    jal send
4:
    wfi
    j 4b
nack:
    li a0, 0x4e                # ASCII N.
    jal send
    j 4b
send:
    li t0, 0x60000000
    sw a0, 0(t0)
    ret
spi:
    sw t1, 0x98(s1)
    li t1, 0x800000
    sw t1, 0(s1)               # UPDATE.
    li t1, 0x1000000
    sw t1, 0(s1)               # USR.
5:
    lw t1, 0(s1)
    bnez t1, 5b
    ret
