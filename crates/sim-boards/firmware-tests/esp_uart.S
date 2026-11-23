# ESP32-C3 TRM, главы UART и Interrupt Matrix. Приём с реальным WFI/IRQ.
.section .text.start,"ax"
.global _start
_start:
    li t0, 0x60009058
    li t1, 0x1000
    sw t1, 0(t0)
    li t0, 0x600045a8
    li t1, 6
    sw t1, 0(t0)
    li s0, 0x60000000
    li t1, 0x03500000          # PLL80, делитель источника 1, RX/TX включены.
    sw t1, 0x78(s0)
    li t1, 80                  # 1 Мбод, 800 тактов для 8N1.
    sw t1, 0x14(s0)
    sw zero, 0x24(s0)          # IRQ при одном или большем числе байт.
    li t1, 0x11
    sw t1, 0x0c(s0)
    li t0, 0x600c2000
    li t1, 5
    sw t1, 0x54(t0)            # UART0 -> CPU_INT5.
    li t1, 1
    sw t1, 0x128(t0)           # CPU_INT_PRI_5 = 1.
    sw t1, 0x194(t0)
    li t1, 32
    sw t1, 0x104(t0)
    la t0, irq
    csrw mtvec, t0
    csrsi mstatus, 8
1:
    wfi
    j 1b

.balign 4
irq:
    csrr t2, mcause
    li t1, 0x80000005
    bne t1, t2, bad
    lw t1, 8(s0)
    andi t2, t1, 16
    beqz t2, 2f
    sw t2, 0x10(s0)            # W1C переполнения.
    lw t1, 4(s0)
    andi t1, t1, 16
    bnez t1, bad
    li t1, 0x4f                # ASCII O перед эхо из полного FIFO.
    sw t1, 0(s0)
2:
    lw t1, 0x1c(s0)
    andi t1, t1, 1023
    beqz t1, 3f
    lw t1, 0(s0)
    sw t1, 0(s0)
    j 2b
3:
    lw t1, 8(s0)
    bnez t1, bad
    mret
bad:
    ebreak
