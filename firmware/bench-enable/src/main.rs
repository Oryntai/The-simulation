//! Прошивка стенда для RP2040: мотор получает ШИМ только при высоком уровне на входе разрешения.
//! GP3 — вход разрешения (аварийная кнопка размыкает его на землю), GP2 — ШИМ на драйвер.
//! Обрыв провода даёт низкий уровень, поэтому состояние по умолчанию — мотор выключен.

#![no_std]
#![no_main]

use core::arch::{asm, global_asm};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    unsafe { asm!("bkpt #0", options(noreturn)) }
}

global_asm!(
    r#"
    .section .vectors,"a",%progbits
    .word 0x20042000
    .word start
"#
);

const SIO_GPIO_IN: usize = 0xd000_0004;
const GPIO_MUX: usize = 0x4001_4004;
const PWM_SLICE1: usize = 0x4005_0014;
/// Функция PWM в IO_BANK0 для GP2 (RP2040 §2.19.2).
const FUNC_PWM: u32 = 4;
const ENABLE_PIN: u32 = 3;
/// Счётчик 0..99: заполнение задаётся в процентах.
const TOP: u32 = 99;
const RUN_DUTY: u32 = 25;

fn write(addr: usize, value: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, value) };
}

fn read(addr: usize) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

#[unsafe(no_mangle)]
extern "C" fn start() -> ! {
    // Сначала нулевое заполнение, и только потом вывод отдаётся ШИМ.
    write(PWM_SLICE1 + 16, TOP);
    write(PWM_SLICE1 + 12, 0);
    write(PWM_SLICE1, 1);
    write(GPIO_MUX + 2 * 8, FUNC_PWM);
    loop {
        let enabled = read(SIO_GPIO_IN) & (1 << ENABLE_PIN) != 0;
        write(PWM_SLICE1 + 12, if enabled { RUN_DUTY } else { 0 });
    }
}
