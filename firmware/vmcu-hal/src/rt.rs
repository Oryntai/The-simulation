//! Стартовый код и ловушки.
//!
//! `_start` (первая инструкция флеша): стек ← конец ОЗУ, копирование `.data` из
//! флеша, обнуление `.bss`, `mtvec` ← `_vmcu_trap_entry` (прямой режим), вызов
//! `_vmcu_main` (определяет макрос [`crate::entry!`]).
//!
//! `_vmcu_trap_entry` сохраняет 16 регистров, которые не сохраняет вызываемая
//! функция по соглашению ILP32 (ra, t0–t6, a0–a7), и вызывает диспетчер на Rust:
//! прерывания → зарегистрированные обработчики, исключения → обработчик
//! исключений (по умолчанию — сообщить хосту о сбое через SYSCTRL и остановиться).

use crate::sys;

core::arch::global_asm!(
    r#"
    .section .text.init, "ax"
    .globl _start
_start:
    .option push
    .option norelax
    la sp, _stack_top
    .option pop
    la t0, _sdata
    la t1, _edata
    la t2, _sidata
1:
    bgeu t0, t1, 2f
    lw t3, 0(t2)
    sw t3, 0(t0)
    addi t0, t0, 4
    addi t2, t2, 4
    j 1b
2:
    la t0, _sbss
    la t1, _ebss
3:
    bgeu t0, t1, 4f
    sw zero, 0(t0)
    addi t0, t0, 4
    j 3b
4:
    la t0, _vmcu_trap_entry
    csrw mtvec, t0
    call _vmcu_main
5:
    j 5b

    .section .text.trap, "ax"
    .balign 4
    .globl _vmcu_trap_entry
_vmcu_trap_entry:
    addi sp, sp, -64
    sw ra, 0(sp)
    sw t0, 4(sp)
    sw t1, 8(sp)
    sw t2, 12(sp)
    sw a0, 16(sp)
    sw a1, 20(sp)
    sw a2, 24(sp)
    sw a3, 28(sp)
    sw a4, 32(sp)
    sw a5, 36(sp)
    sw a6, 40(sp)
    sw a7, 44(sp)
    sw t3, 48(sp)
    sw t4, 52(sp)
    sw t5, 56(sp)
    sw t6, 60(sp)
    mv a0, sp
    call _vmcu_trap_rust
    lw ra, 0(sp)
    lw t0, 4(sp)
    lw t1, 8(sp)
    lw t2, 12(sp)
    lw a0, 16(sp)
    lw a1, 20(sp)
    lw a2, 24(sp)
    lw a3, 28(sp)
    lw a4, 32(sp)
    lw a5, 36(sp)
    lw a6, 40(sp)
    lw a7, 44(sp)
    lw t3, 48(sp)
    lw t4, 52(sp)
    lw t5, 56(sp)
    lw t6, 60(sp)
    addi sp, sp, 64
    mret
"#
);

/// Регистры, сохранённые при входе в ловушку (порядок — как в `_vmcu_trap_entry`).
/// Обработчик `ecall` читает аргументы из `a0..a7` и может вернуть результат в `a0`.
#[repr(C)]
#[derive(Debug)]
pub struct TrapFrame {
    pub ra: u32,
    pub t0: u32,
    pub t1: u32,
    pub t2: u32,
    pub a: [u32; 8],
    pub t3: u32,
    pub t4: u32,
    pub t5: u32,
    pub t6: u32,
}

/// Обработчик исключения: (кадр, mcause, mepc, mtval) → новый mepc.
pub type ExceptionHandler = fn(&mut TrapFrame, u32, u32, u32) -> u32;

static mut TIMER_HANDLER: Option<fn()> = None;
static mut EXTERNAL_HANDLER: Option<fn()> = None;
static mut SOFTWARE_HANDLER: Option<fn()> = None;
static mut EXCEPTION_HANDLER: Option<ExceptionHandler> = None;

/// Назначить обработчик прерывания таймера (вызывать до разрешения прерываний).
pub fn set_timer_handler(h: fn()) {
    crate::interrupt::free(|| {
        // SAFETY: запись при запрещённых прерываниях, ядро одно.
        unsafe { TIMER_HANDLER = Some(h) }
    });
}

/// Назначить обработчик внешних прерываний (INTC).
pub fn set_external_handler(h: fn()) {
    crate::interrupt::free(|| {
        // SAFETY: см. выше.
        unsafe { EXTERNAL_HANDLER = Some(h) }
    });
}

/// Назначить обработчик программного прерывания (CLINT.MSIP).
pub fn set_software_handler(h: fn()) {
    crate::interrupt::free(|| {
        // SAFETY: см. выше.
        unsafe { SOFTWARE_HANDLER = Some(h) }
    });
}

/// Назначить обработчик исключений (ecall, недопустимая инструкция, …).
pub fn set_exception_handler(h: ExceptionHandler) {
    crate::interrupt::free(|| {
        // SAFETY: см. выше.
        unsafe { EXCEPTION_HANDLER = Some(h) }
    });
}

/// Длина инструкции по адресу (2 — сжатая, 4 — обычная).
pub fn instruction_len(pc: u32) -> u32 {
    // SAFETY: pc указывает на исполнявшуюся инструкцию — память читаема.
    let lo = unsafe { core::ptr::read_volatile(pc as *const u16) };
    if lo & 3 == 3 { 4 } else { 2 }
}

/// Обработчик исключений по умолчанию: сообщить хосту (UART + SYSCTRL.FAULT_*).
pub fn default_exception(_frame: &mut TrapFrame, mcause: u32, mepc: u32, mtval: u32) -> u32 {
    crate::uart::write_bytes(b"FAULT mcause=");
    crate::uart::write_hex(mcause);
    crate::uart::write_bytes(b" mepc=");
    crate::uart::write_hex(mepc);
    crate::uart::write_bytes(b" mtval=");
    crate::uart::write_hex(mtval);
    crate::uart::write_byte(b'\n');
    sys::report_fault(mcause, mepc, mtval)
}

#[unsafe(no_mangle)]
extern "C" fn _vmcu_trap_rust(frame: &mut TrapFrame) {
    let mcause = crate::csr_read!("mcause");
    if mcause & 0x8000_0000 != 0 {
        // SAFETY: чтение указателей на функции по значению; меняются они только
        // в критической секции.
        let h = unsafe {
            match mcause & 0x7fff_ffff {
                7 => TIMER_HANDLER,
                11 => EXTERNAL_HANDLER,
                3 => SOFTWARE_HANDLER,
                _ => None,
            }
        };
        if let Some(h) = h {
            h();
        }
    } else {
        let mepc = crate::csr_read!("mepc");
        let mtval = crate::csr_read!("mtval");
        // SAFETY: см. выше.
        let h = unsafe { EXCEPTION_HANDLER }.unwrap_or(default_exception);
        let new_epc = h(frame, mcause, mepc, mtval);
        crate::csr_write!("mepc", new_epc);
    }
}

/// Объявить точку входа: `vmcu_hal::entry!(main);` где `fn main() -> !`.
#[macro_export]
macro_rules! entry {
    ($path:path) => {
        #[unsafe(export_name = "_vmcu_main")]
        pub extern "C" fn __vmcu_main() -> ! {
            let f: fn() -> ! = $path;
            f()
        }
    };
}

/// Паника: сообщение в UART, затем остановка с кодом 0xDEAD_0001.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    crate::interrupt::disable();
    crate::println!("PANIC: {}", info);
    sys::halt(PANIC_EXIT_CODE)
}

/// Код выхода при панике.
pub const PANIC_EXIT_CODE: u32 = 0xdead_0001;
