#![no_std]
#![no_main]

use core::arch::global_asm;

#[cfg(esp_bus)]
global_asm!(include_str!("esp_bus.S"));
#[cfg(esp_uart)]
global_asm!(include_str!("esp_uart.S"));
#[cfg(vmcu)]
global_asm!(include_str!("vmcu_ports.S"));

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    loop {}
}
