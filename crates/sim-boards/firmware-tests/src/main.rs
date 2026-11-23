#![no_std]
#![no_main]

use core::arch::{asm, global_asm};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    #[cfg(target_arch = "arm")]
    unsafe {
        asm!("bkpt #0", options(noreturn))
    }
    #[cfg(target_arch = "riscv32")]
    unsafe {
        asm!("ebreak", options(noreturn))
    }
}

#[inline(always)]
fn write(addr: usize, value: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, value) };
}

#[inline(always)]
fn read(addr: usize) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

#[inline(never)]
fn send_pair(port: usize, status: usize, busy_mask: u32, when: u32, result: u32) {
    for byte in [result >> 8, result & 255] {
        while read(status) & busy_mask != when {}
        write(port, byte);
    }
}

#[inline(always)]
fn wait() -> ! {
    loop {
        unsafe { asm!("wfi") };
    }
}

#[cfg(target_arch = "arm")]
fn arithmetic() {
    // DDI0419/PM0056: перенос из 0xffffffff + 1, ADC, MUL и чтение стека.
    let n: u32;
    unsafe {
        asm!(
            "movs r0, #0",
            "mvns r1, r0",
            "adds r1, #1",
            "adcs r1, r0",
            "movs r0, #21",
            "muls r1, r0",
            "push {{r1}}",
            "pop {{r0}}",
            "movs r2, #1",
            "lsls r2, #31",
            "subs r2, #1",
            "adds r2, #1",
            "bvs 2f",
            "bkpt #1",
            "2:",
            out("r0") n, out("r1") _, out("r2") _,
        );
    }
    if n != 21 {
        panic!("ошибка арифметики Thumb");
    }
    #[cfg(stm)]
    {
        let q: u32;
        unsafe {
            asm!("udiv {0}, {1}, {2}", out(reg) q, in(reg) n, in(reg) 3u32);
        }
        if q != 7 {
            panic!("ошибка UDIV");
        }
    }
    #[cfg(pico)]
    {
        // ARMv6-M не имеет UDIV. Компилятор использует программное деление.
        let dividend = core::hint::black_box(n);
        let divisor = core::hint::black_box(3u32);
        if dividend / divisor != 7 {
            panic!("ошибка программного деления");
        }
    }
}

#[cfg(pico)]
mod pico {
    use super::*;

    global_asm!(
        r#"
        .section .vectors,"a",%progbits
        .word 0x20042000
        .word start
        .space 56
        .word interrupt
    "#
    );

    const SIO: usize = 0xd0000000;
    const TIMER: usize = 0x40054000;
    const ADC: usize = 0x4004c000;
    const UART: usize = 0x40034000;
    const WDT: usize = 0x40058000;
    const GPIO_MUX: usize = 0x40014004;
    const PWM: usize = 0x40050000;

    #[unsafe(no_mangle)]
    extern "C" fn interrupt() {
        // RP2040 TIMER §4.6: ALARM0, INTR W1C, повтор каждые 1000 тиков по 1 мкс.
        write(TIMER + 0x34, 1);
        write(TIMER + 0x10, read(TIMER + 0x10).wrapping_add(1000));
        write(SIO + 0x1c, 1 << 25);
    }

    #[unsafe(no_mangle)]
    extern "C" fn start() -> ! {
        write(GPIO_MUX + 25 * 8, 5);
        write(SIO + 0x24, 1 << 25);
        write(GPIO_MUX, 2);
        // RP2040 §4.7 PL011 DR, FR, IBRD, LCR_H, CR; 8N1, стендовый делитель 8.
        write(UART + 0x24, 8);
        write(UART + 0x2c, 0x60);
        write(UART + 0x30, 0x101);
        if read(WDT + 8) & 1 != 0 {
            write(UART, b'R'.into());
            write(SIO + 0x14, 1 << 25);
            wait();
        }
        if read(SIO + 4) & (1 << 3) == 0 {
            panic!("на GP3 ожидается высокий уровень стенда");
        }
        arithmetic();
        // GPIO2 → PWM slice1 A; счётчик 0..99, первые 25 тиков высокий уровень.
        write(GPIO_MUX + 2 * 8, 4);
        write(PWM + 0x14 + 16, 99);
        write(PWM + 0x14 + 12, 25);
        write(PWM + 0x14, 1);
        // RP2040 PADS_BANK0: GPIO26 в аналоговом режиме, без входного буфера и подтяжек.
        write(0x4001c000 + 4 * 27, 0x80);
        write(ADC, 1);
        while read(ADC) & 0x100 == 0 {}
        write(ADC, 5); // EN + START_ONCE, AINSEL=0.
        while read(ADC) & 0x100 == 0 {}
        send_pair(UART, UART + 0x18, 8, 0, read(ADC + 4));
        write(TIMER + 0x10, read(TIMER + 0x28).wrapping_add(1000));
        write(TIMER + 0x38, 1);
        write(0xe000e100, 1); // NVIC, TIMER_IRQ_0.
        // RP2040-E1: 16 000 отсчётов со скоростью 2 за микросекунду.
        write(0x40010008, 0x1fffc); // PSM WDSEL, как watchdog_enable в pico-sdk.
        write(WDT + 4, 16000);
        write(WDT, 1 << 30);
        wait()
    }
}

#[cfg(stm)]
mod stm {
    use super::*;

    global_asm!(
        r#"
        .section .vectors,"a",%progbits
        .word 0x20005000
        .word start
        .space 52
        .word systick
        .space 112
        .word interrupt
    "#
    );

    const RCC: usize = 0x40021000;
    const A: usize = 0x40010800;
    const C: usize = 0x40011000;
    const TIM2: usize = 0x40000000;
    const TIM3: usize = 0x40000400;
    const ADC: usize = 0x40012400;
    const USART: usize = 0x40013800;
    const WD: usize = 0x40003000;

    #[unsafe(no_mangle)]
    extern "C" fn interrupt() {
        write(TIM2 + 0x10, 0);
        write(0xe000ed04, 1 << 26); // PENDSTSET. SysTick вытесняет TIM2.
        write(C + 0xc, read(C + 0xc) ^ (1 << 13));
    }

    #[unsafe(no_mangle)]
    extern "C" fn systick() {
        // PA5 отмечает SysTick, PA4 дополнительно отмечает вытеснение активного TIM2.
        let active = read(0xe000e300) & (1 << 28) != 0;
        write(A + 0x10, (1 << 5) | (u32::from(active) << 4));
        write(0xe000e010, 0);
    }

    #[unsafe(no_mangle)]
    extern "C" fn start() -> ! {
        // RM0008 RCC, GPIO, USART1 TX=PA9; PA0 аналоговый, PA5 GPIO, PA6 TIM3_CH1.
        write(RCC + 0x18, (1 << 14) | (1 << 9) | (1 << 4) | (1 << 2));
        write(RCC + 0x1c, 3);
        write(A, 0x4a22_4440);
        write(A + 4, 0x4444_44a4);
        write(C + 4, 0x4424_4444);
        write(USART + 8, 16);
        write(USART + 0xc, 0x2008);
        if read(RCC + 0x24) & (1 << 29) != 0 {
            write(USART + 4, b'R'.into());
            write(C + 0x10, 1 << 13);
            wait();
        }
        if read(A + 8) & (1 << 7) == 0 {
            panic!("на PA7 ожидается высокий уровень стенда");
        }
        arithmetic();
        write(TIM3 + 0x2c, 99);
        write(TIM3 + 0x34, 25);
        write(TIM3 + 0x18, 0x60);
        write(TIM3 + 0x20, 1);
        write(TIM3, 1);
        write(ADC + 8, 1);
        // Ожидание стабилизации ADON: запас 16 тактов NOP при HSI 8 МГц.
        unsafe {
            asm!(".rept 16", "nop", ".endr");
        }
        write(ADC + 8, 1);
        while read(ADC) & 2 == 0 {}
        send_pair(USART + 4, USART, 0x80, 0x80, read(ADC + 0x4c));
        // RM0008 TIM2, 8 МГц / 8 / 1000 → 1000 обновлений/с.
        write(TIM2 + 0x28, 7);
        write(TIM2 + 0x2c, 999);
        write(TIM2 + 0x14, 1);
        write(TIM2 + 0x10, 0);
        write(TIM2 + 0xc, 1);
        write(0xe000e41c, 0x80); // NVIC IPR7, TIM2 ниже SysTick.
        write(0xe000e100, 1 << 28);
        write(TIM2, 1);
        write(0xe000e014, 3999);
        write(0xe000e018, 0);
        write(0xe000e010, 7);
        // IWDG при типовом LSI 40 кГц: 320 * 4 / 40 000 = 32 мс.
        write(WD, 0x5555);
        write(WD + 4, 0);
        write(WD + 8, 319);
        write(WD, 0xcccc);
        write(WD, 0xaaaa);
        wait()
    }
}

#[cfg(target_arch = "riscv32")]
mod esp {
    use super::*;

    global_asm!(
        r#"
        .section .text.start,"ax",@progbits
        .globl _start
        _start:
          li sp, 0x3fcb0000
          j start
        .section .text.vectors,"ax",@progbits
        .balign 4
        .globl trap
        trap:
          addi sp,sp,-16
          sw ra,12(sp)
          sw t0,8(sp)
          sw t1,4(sp)
          sw t2,0(sp)
          li t0,0x6001f000
          li t1,1
          sw t1,0x7c(t0)
          lw t1,0(t0)
          ori t1,t1,1024
          sw t1,0(t0)
          li t0,0x60004000
          lw t1,4(t0)
          xori t1,t1,8
          sw t1,4(t0)
          lw t2,0(sp)
          lw t1,4(sp)
          lw t0,8(sp)
          lw ra,12(sp)
          addi sp,sp,16
          mret
    "#
    );

    const GPIO: usize = 0x60004000;
    const IO_MUX: usize = 0x60009004;
    const INTR: usize = 0x600c2000;
    const TIMER: usize = 0x6001f000;
    const LEDC: usize = 0x60019000;
    const ADC: usize = 0x60040000;
    const UART: usize = 0x60000000;
    const MARKER: usize = 0x3fcb1000; // SRAM, слово стенда переживает системный сброс модели.

    #[unsafe(no_mangle)]
    extern "C" fn start() -> ! {
        for pin in [3, 4, 21] {
            write(IO_MUX + 4 * pin, 1 << 12);
        }
        write(GPIO + 0x24, 1 << 3);
        write(GPIO + 0x554 + 4 * 21, 6); // U0TXD_OUT_IDX.
        write(UART + 0x78, 0x0350_0000); // UART_SCLK = 80 МГц, TX включён.
        write(UART + 0x14, 16);
        if read(MARKER) == 0x42 {
            write(UART, b'R'.into());
            write(GPIO + 8, 1 << 3);
            wait();
        }
        write(IO_MUX + 4 * 10, (1 << 12) | (1 << 9));
        if read(GPIO + 0x3c) & (1 << 10) == 0 {
            panic!("на GPIO10 ожидается высокий уровень стенда");
        }
        let q: u32;
        unsafe {
            asm!("mul {out}, {x}, {y}", "divu {out}, {out}, {y}",
                out = out(reg) q, x = in(reg) 21u32, y = in(reg) 3u32);
        }
        let carry: u32;
        unsafe {
            asm!("add {v}, {max}, {one}", "sltu {v}, {v}, {max}",
            v = out(reg) carry, max = in(reg) u32::MAX, one = in(reg) 1u32);
        }
        if q != 21 || carry != 1 {
            panic!("ошибка RV32IM");
        }
        write(LEDC + 0xd0, 1); // APB 80 МГц.
        write(LEDC + 0xa0, (1 << 25) | (256 << 4) | 8);
        write(LEDC + 8, 64 << 4);
        write(LEDC + 0xc, 1 << 31);
        write(LEDC, 0x14);
        write(GPIO + 0x554 + 4 * 4, 45); // LEDC_LS_SIG_OUT0_IDX.
        write(ADC + 0x20, (1 << 31) | (3 << 23));
        write(ADC + 0x20, (1 << 31) | (1 << 29) | (3 << 23));
        while read(ADC + 0x44) >> 31 == 0 {}
        send_pair(UART, UART + 0x6c, 0xf0, 0, read(ADC + 0x2c) & 4095);
        write(INTR + 0x80, 1);
        write(INTR + 0x118, 1);
        write(INTR + 0x194, 1);
        write(INTR + 0x104, 1 << 1);
        unsafe {
            asm!("la t0, trap", "csrw mtvec,t0", "csrsi mstatus,8", out("t0") _);
        }
        // TIMG0: 80 МГц / 80, сигнал сравнения после 1000 тиков.
        write(TIMER + 0x10, 1000);
        write(TIMER + 0x70, 1);
        write(TIMER, 0xe0000400 | (80 << 13));
        write(MARKER, 0x42);
        // MWDT: 80 000 отсчётов с делителем 8 → 8 мс.
        write(TIMER + 0x64, 0x50d8_3aa1);
        write(TIMER + 0x4c, 8 << 16);
        write(TIMER + 0x50, 80000);
        write(TIMER + 0x48, 0xe044_8000);
        write(TIMER + 0x64, 0);
        wait()
    }
}
