//! HAL виртуального микроконтроллера VMCU-C3 (эмулятор `sim-mcu`).
//!
//! * [`rt`] — стартовый код (стек, копирование `.data`, обнуление `.bss`, `mtvec`),
//!   вход в ловушку, диспетчер прерываний, обработчик паники;
//! * обёртки периферии: [`gpio`], [`pwm`], [`adc`], [`uart`], [`radio`], [`wdt`],
//!   [`timer`] (CLINT), [`intc`], [`sys`], [`config`];
//! * [`csr`] — доступ к CSR, [`interrupt`] — критические секции и ячейка [`interrupt::Global`].
//!
//! Адреса регистров — `crates/sim-mcu/MEMORY_MAP.md`.
#![no_std]

pub mod rt;

/// Чтение 32-битного регистра.
#[inline(always)]
pub fn reg_read(addr: usize) -> u32 {
    // SAFETY: адреса — регистры периферии VMCU-C3 из карты памяти.
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

/// Запись 32-битного регистра.
#[inline(always)]
pub fn reg_write(addr: usize, v: u32) {
    // SAFETY: адреса — регистры периферии VMCU-C3 из карты памяти.
    unsafe { core::ptr::write_volatile(addr as *mut u32, v) }
}

/// Базовые адреса блоков.
pub mod map {
    pub const FLASH: usize = 0x4200_0000;
    pub const RAM: usize = 0x3fc8_0000;
    pub const CONFIG: usize = 0x5000_0000;
    pub const CONFIG_SIZE: usize = 4096;
    pub const CLINT: usize = 0x0200_0000;
    pub const SYSCTRL: usize = 0x6000_0000;
    pub const GPIO: usize = 0x6000_1000;
    pub const PWM: usize = 0x6000_2000;
    pub const ADC: usize = 0x6000_3000;
    pub const UART0: usize = 0x6000_4000;
    pub const RADIO: usize = 0x6000_5000;
    pub const WDT: usize = 0x6000_6000;
    pub const INTC: usize = 0x6000_7000;
    /// Частоты.
    pub const CPU_HZ: u32 = 160_000_000;
    pub const MTIME_HZ: u32 = 16_000_000;
}

/// Доступ к CSR машинного режима.
pub mod csr {
    /// Прочитать CSR по имени (`csr_read!("mcause")`).
    #[macro_export]
    macro_rules! csr_read {
        ($name:literal) => {{
            let v: u32;
            // SAFETY: чтение CSR не нарушает память.
            unsafe { core::arch::asm!(concat!("csrr {0}, ", $name), out(reg) v) };
            v
        }};
    }

    /// Записать CSR по имени (`csr_write!("mepc", v)`).
    #[macro_export]
    macro_rules! csr_write {
        ($name:literal, $v:expr) => {{
            let v: u32 = $v;
            // SAFETY: запись CSR машинного режима — ответственность вызывающего.
            unsafe { core::arch::asm!(concat!("csrw ", $name, ", {0}"), in(reg) v) };
        }};
    }

    /// Счётчик тактов `mcycle` (64 бита, чтение hi–lo–hi).
    pub fn mcycle() -> u64 {
        loop {
            let hi = crate::csr_read!("mcycleh");
            let lo = crate::csr_read!("mcycle");
            if crate::csr_read!("mcycleh") == hi {
                return ((hi as u64) << 32) | lo as u64;
            }
        }
    }

    /// Счётчик выполненных инструкций `minstret`.
    pub fn minstret() -> u64 {
        loop {
            let hi = crate::csr_read!("minstreth");
            let lo = crate::csr_read!("minstret");
            if crate::csr_read!("minstreth") == hi {
                return ((hi as u64) << 32) | lo as u64;
            }
        }
    }

    pub const MIE_MSIE: u32 = 1 << 3;
    pub const MIE_MTIE: u32 = 1 << 7;
    pub const MIE_MEIE: u32 = 1 << 11;
}

/// Прерывания: глобальное разрешение, критические секции, общая ячейка данных.
pub mod interrupt {
    use core::cell::UnsafeCell;

    /// Разрешить прерывания глобально (`mstatus.MIE = 1`).
    #[inline(always)]
    pub fn enable() {
        // SAFETY: установка бита MIE.
        unsafe { core::arch::asm!("csrsi mstatus, 8") };
    }

    /// Запретить прерывания глобально.
    #[inline(always)]
    pub fn disable() {
        // SAFETY: сброс бита MIE.
        unsafe { core::arch::asm!("csrci mstatus, 8") };
    }

    /// Разрешить отдельные источники в `mie` (биты [`crate::csr`]::MIE_*).
    pub fn unmask(bits: u32) {
        // SAFETY: установка битов mie.
        unsafe { core::arch::asm!("csrs mie, {0}", in(reg) bits) };
    }

    /// Выполнить `f` с запрещёнными прерываниями (вложенные вызовы допустимы).
    #[inline(always)]
    pub fn free<R>(f: impl FnOnce() -> R) -> R {
        let prev: u32;
        // SAFETY: атомарный сброс MIE с сохранением прежнего состояния; asm без
        // `nomem` — барьер компилятора для обращений к памяти.
        unsafe { core::arch::asm!("csrrci {0}, mstatus, 8", out(reg) prev) };
        let r = f();
        if prev & 8 != 0 {
            enable();
        }
        r
    }

    /// Ждать прерывания (`wfi`).
    #[inline(always)]
    pub fn wait() {
        // SAFETY: wfi — подсказка ядру, на память не влияет.
        unsafe { core::arch::asm!("wfi") };
    }

    /// Данные, общие для основного цикла и обработчиков прерываний одного ядра.
    /// Доступ — только внутри критической секции ([`Global::with`]).
    pub struct Global<T>(UnsafeCell<T>);

    // SAFETY: ядро одно, доступ к содержимому — только при запрещённых прерываниях.
    unsafe impl<T: Send> Sync for Global<T> {}

    impl<T> Global<T> {
        pub const fn new(v: T) -> Self {
            Global(UnsafeCell::new(v))
        }

        /// Доступ в критической секции.
        #[inline(always)]
        pub fn with<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
            // SAFETY: прерывания запрещены — второй ссылки одновременно нет.
            free(|| f(unsafe { &mut *self.0.get() }))
        }
    }
}

/// CLINT: `mtime` (16 МГц) и `mtimecmp`.
pub mod timer {
    use crate::{map::CLINT, reg_read, reg_write};

    const MTIMECMP_LO: usize = CLINT + 0x4000;
    const MTIMECMP_HI: usize = CLINT + 0x4004;
    const MTIME_LO: usize = CLINT + 0xbff8;
    const MTIME_HI: usize = CLINT + 0xbffc;
    /// Тиков `mtime` в микросекунде.
    pub const TICKS_PER_US: u64 = 16;

    pub fn mtime() -> u64 {
        loop {
            let hi = reg_read(MTIME_HI);
            let lo = reg_read(MTIME_LO);
            if reg_read(MTIME_HI) == hi {
                return ((hi as u64) << 32) | lo as u64;
            }
        }
    }

    /// Установить `mtimecmp` без ложного срабатывания (Privileged ISA, 3.2.1).
    pub fn set_compare(v: u64) {
        reg_write(MTIMECMP_LO, u32::MAX);
        reg_write(MTIMECMP_HI, (v >> 32) as u32);
        reg_write(MTIMECMP_LO, v as u32);
    }

    /// Разрешить прерывание таймера в `mie`.
    pub fn enable_irq() {
        crate::interrupt::unmask(crate::csr::MIE_MTIE);
    }
}

/// GPIO.
pub mod gpio {
    use crate::{map::GPIO, reg_read, reg_write};

    pub fn set_output(pin: u32) {
        reg_write(GPIO + 0x10, reg_read(GPIO + 0x10) | (1 << pin));
    }

    pub fn set_high(pin: u32) {
        reg_write(GPIO + 0x04, 1 << pin);
    }

    pub fn set_low(pin: u32) {
        reg_write(GPIO + 0x08, 1 << pin);
    }

    pub fn write(pin: u32, high: bool) {
        if high { set_high(pin) } else { set_low(pin) }
    }

    pub fn toggle(pin: u32) {
        reg_write(GPIO + 0x0c, 1 << pin);
    }

    /// Уровни всех выводов.
    #[inline(always)]
    pub fn levels() -> u32 {
        reg_read(GPIO + 0x14)
    }

    pub fn is_high(pin: u32) -> bool {
        levels() >> pin & 1 != 0
    }
}

/// ШИМ: 4 канала, заполнение 0..=65535.
pub mod pwm {
    use crate::{map::PWM, reg_write};

    pub fn configure(ch: u32, freq_hz: u32) {
        reg_write(PWM + 0x10 * ch as usize + 4, freq_hz);
        reg_write(PWM + 0x10 * ch as usize + 8, 1);
    }

    #[inline(always)]
    pub fn set_duty(ch: u32, duty: u32) {
        reg_write(PWM + 0x10 * ch as usize, duty);
    }

    pub fn disable(ch: u32) {
        reg_write(PWM + 0x10 * ch as usize, 0);
        reg_write(PWM + 0x10 * ch as usize + 8, 0);
    }
}

/// АЦП: 8 каналов, 12 бит, 3.3 В.
pub mod adc {
    use crate::{map::ADC, reg_read};

    /// Преобразование канала (код 0..=4095).
    #[inline(always)]
    pub fn read(ch: u32) -> u32 {
        reg_read(ADC + 4 * ch as usize)
    }

    /// Код → вольты.
    pub fn volts(code: u32) -> f32 {
        code as f32 * (3.3 / 4096.0)
    }
}

/// UART0 и макросы печати.
pub mod uart {
    use crate::{map::UART0, reg_read, reg_write};

    pub fn write_byte(b: u8) {
        reg_write(UART0, b as u32);
    }

    pub fn write_bytes(s: &[u8]) {
        for &b in s {
            write_byte(b);
        }
    }

    /// Байт из приёмника, если есть.
    pub fn read_byte() -> Option<u8> {
        let v = reg_read(UART0 + 4);
        if v & 0x8000_0000 != 0 {
            None
        } else {
            Some(v as u8)
        }
    }

    /// Печать шестнадцатеричного числа (8 цифр) без `core::fmt`.
    pub fn write_hex(v: u32) {
        for k in (0..8).rev() {
            let d = (v >> (4 * k)) & 0xf;
            write_byte(if d < 10 {
                b'0' + d as u8
            } else {
                b'a' + d as u8 - 10
            });
        }
    }

    /// Писатель для `core::fmt`.
    pub struct Uart;

    impl core::fmt::Write for Uart {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            write_bytes(s.as_bytes());
            Ok(())
        }
    }

    #[macro_export]
    macro_rules! print {
        ($($t:tt)*) => {{
            use core::fmt::Write as _;
            let _ = write!($crate::uart::Uart, $($t)*);
        }};
    }

    #[macro_export]
    macro_rules! println {
        () => { $crate::uart::write_byte(b'\n') };
        ($($t:tt)*) => {{
            use core::fmt::Write as _;
            let _ = writeln!($crate::uart::Uart, $($t)*);
        }};
    }
}

/// Пакетное радио.
pub mod radio {
    use crate::{map::RADIO, reg_read, reg_write};

    pub const MAX_PACKET: usize = 251;
    const STATUS: usize = RADIO;
    const TX_LEN: usize = RADIO + 0x04;
    const RX_LEN: usize = RADIO + 0x08;
    const RX_RSSI: usize = RADIO + 0x0c;
    const RX_POP: usize = RADIO + 0x10;
    const TX_BUF: usize = RADIO + 0x100;
    const RX_BUF: usize = RADIO + 0x200;

    /// Отправить пакет (1..=251 байт). `false` — очередь передачи полна.
    pub fn send(data: &[u8]) -> bool {
        if data.is_empty() || data.len() > MAX_PACKET || reg_read(STATUS) & 2 != 0 {
            return false;
        }
        for (k, chunk) in data.chunks(4).enumerate() {
            let mut w = [0u8; 4];
            w[..chunk.len()].copy_from_slice(chunk);
            reg_write(TX_BUF + 4 * k, u32::from_le_bytes(w));
        }
        reg_write(TX_LEN, data.len() as u32);
        true
    }

    /// Есть принятый пакет.
    pub fn available() -> bool {
        reg_read(STATUS) & 1 != 0
    }

    /// Снять принятый пакет в `buf`: (длина пакета, RSSI дБм). Если пакет длиннее
    /// буфера, копируется начало, длина возвращается полная.
    pub fn recv(buf: &mut [u8]) -> Option<(usize, i8)> {
        let len = reg_read(RX_LEN) as usize;
        if len == 0 {
            return None;
        }
        let rssi = reg_read(RX_RSSI) as i32 as i8;
        let n = len.min(buf.len());
        for (k, chunk) in buf[..n].chunks_mut(4).enumerate() {
            let w = reg_read(RX_BUF + 4 * k).to_le_bytes();
            chunk.copy_from_slice(&w[..chunk.len()]);
        }
        reg_write(RX_POP, 1);
        Some((len, rssi))
    }
}

/// Аппаратный сторож.
pub mod wdt {
    use crate::{map::WDT, reg_write};

    pub const KICK_KEY: u32 = 0x0d06_f00d;

    /// Включить с тайм-аутом в микросекундах.
    pub fn start(timeout_us: u32) {
        reg_write(WDT + 0x04, timeout_us);
        reg_write(WDT, 1);
    }

    /// «Покормить» сторожа.
    #[inline(always)]
    pub fn kick() {
        reg_write(WDT + 0x08, KICK_KEY);
    }
}

/// Контроллер внешних прерываний.
pub mod intc {
    use crate::{map::INTC, reg_read, reg_write};

    pub const UART_RX: u32 = 1 << 0;
    pub const RADIO_RX: u32 = 1 << 1;

    pub fn pending() -> u32 {
        reg_read(INTC)
    }

    /// Разрешить источники в INTC и линию MEIP в `mie`.
    pub fn enable(sources: u32) {
        reg_write(INTC + 4, reg_read(INTC + 4) | sources);
        crate::interrupt::unmask(crate::csr::MIE_MEIE);
    }

    pub fn disable(sources: u32) {
        reg_write(INTC + 4, reg_read(INTC + 4) & !sources);
    }
}

/// Системные регистры.
pub mod sys {
    use crate::{map::SYSCTRL, reg_read, reg_write};

    pub const RESET_POWER_ON: u32 = 1;
    pub const RESET_WATCHDOG: u32 = 2;
    pub const RESET_SOFTWARE: u32 = 3;

    pub fn id() -> u32 {
        reg_read(SYSCTRL)
    }

    pub fn reset_reason() -> u32 {
        reg_read(SYSCTRL + 0x04)
    }

    pub fn watchdog_resets() -> u32 {
        reg_read(SYSCTRL + 0x08)
    }

    /// Время модели, мкс.
    pub fn time_us() -> u64 {
        loop {
            let hi = reg_read(SYSCTRL + 0x10);
            let lo = reg_read(SYSCTRL + 0x0c);
            if reg_read(SYSCTRL + 0x10) == hi {
                return ((hi as u64) << 32) | lo as u64;
            }
        }
    }

    /// Остановить ядро с кодом выхода (для тестов).
    pub fn halt(code: u32) -> ! {
        loop {
            reg_write(SYSCTRL + 0x18, code);
        }
    }

    /// Сообщить хосту о неустранимом исключении и остановиться.
    pub fn report_fault(cause: u32, pc: u32, tval: u32) -> ! {
        reg_write(SYSCTRL + 0x20, pc);
        reg_write(SYSCTRL + 0x24, tval);
        loop {
            reg_write(SYSCTRL + 0x28, cause);
        }
    }

    /// Программный сброс.
    pub fn software_reset() -> ! {
        loop {
            reg_write(SYSCTRL + 0x1c, 0x5257_5354);
        }
    }
}

/// Блок конфигурации (хост пишет до сброса; 4096 байт по 0x5000_0000).
pub mod config {
    use crate::map::{CONFIG, CONFIG_SIZE};

    /// Весь блок.
    pub fn bytes() -> &'static [u8] {
        // SAFETY: область только для чтения, всегда отображена.
        unsafe { core::slice::from_raw_parts(CONFIG as *const u8, CONFIG_SIZE) }
    }

    pub fn u32_at(off: usize) -> u32 {
        let b = bytes();
        u32::from_le_bytes([b[off], b[off + 1], b[off + 2], b[off + 3]])
    }

    pub fn f32_at(off: usize) -> f32 {
        f32::from_bits(u32_at(off))
    }
}

/// CRC-32 (IEEE 802.3, полином 0xEDB88320, побитовый) — проверка блока конфигурации.
pub fn crc32(data: &[u8]) -> u32 {
    let mut c = 0xffff_ffffu32;
    for &b in data {
        c ^= b as u32;
        for _ in 0..8 {
            c = if c & 1 != 0 {
                (c >> 1) ^ 0xedb8_8320
            } else {
                c >> 1
            };
        }
    }
    !c
}
