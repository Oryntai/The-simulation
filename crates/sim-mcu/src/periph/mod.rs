//! Периферия VMCU-C3: свои простые регистры (не ESP32-C3), описаны в
//! `crates/sim-mcu/MEMORY_MAP.md`.
//!
//! Блоки по 0x1000 байт от `PERIPH_BASE` (0x6000_0000):
//! 0 — SYSCTRL, 1 — GPIO, 2 — PWM, 3 — ADC, 4 — UART0, 5 — RADIO, 6 — WDT, 7 — INTC.
//! Регистры — 32-битные, доступ только словом (иначе ошибка доступа); буферы
//! радио допускают байты и полуслова. Обращение к несуществующему регистру —
//! ошибка доступа (ловит ошибки прошивки).

pub mod adc;
pub mod clint;
pub mod gpio;
pub mod intc;
pub mod pwm;
pub mod radio;
pub mod sysctrl;
pub mod uart;
pub mod wdt;

use crate::cpu::{MIP_MEIP, MIP_MSIP, MIP_MTIP};

/// Запрос периферии к машине (выполняется после текущей инструкции).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Request {
    /// SYSCTRL.HALT: остановить ядро с кодом выхода (для тестов).
    Halt(u32),
    /// SYSCTRL.SW_RESET: программный сброс.
    SoftwareReset,
    /// SYSCTRL.FAULT_CAUSE: прошивка сообщила о неустранимом исключении.
    FirmwareFault { cause: u32, pc: u32, tval: u32 },
}

/// Номера блоков.
pub const SYSCTRL: u32 = 0;
pub const GPIO: u32 = 1;
pub const PWM: u32 = 2;
pub const ADC: u32 = 3;
pub const UART0: u32 = 4;
pub const RADIO: u32 = 5;
pub const WDT: u32 = 6;
pub const INTC: u32 = 7;

/// Вся периферия.
#[derive(Clone, Debug, Default)]
pub struct Peripherals {
    pub sys: sysctrl::SysCtrl,
    pub clint: clint::Clint,
    pub gpio: gpio::Gpio,
    pub pwm: pwm::Pwm,
    pub adc: adc::Adc,
    pub uart: uart::Uart,
    pub radio: radio::Radio,
    pub wdt: wdt::Wdt,
    pub intc: intc::Intc,
    pub request: Option<Request>,
    pub(crate) host_timestamps: bool,
    pub(crate) adc_inputs: Option<u8>,
    pub(crate) uart_tx_cycles: Vec<u64>,
    pub(crate) radio_tx_cycles: std::collections::VecDeque<u64>,
}

impl Peripherals {
    /// Сброс микросхемы (по питанию, сторожу или программный): регистры — в
    /// исходное, внешний мир (входы GPIO, напряжения АЦП, уже переданные пакеты и
    /// принятый хостом вывод UART) — не трогаем.
    pub fn reset_chip(&mut self) {
        self.clint = clint::Clint::default();
        self.gpio.reset();
        self.pwm = pwm::Pwm::default();
        self.uart.reset();
        self.radio.reset();
        self.wdt = wdt::Wdt::default();
        self.intc = intc::Intc::default();
        self.request = None;
    }

    /// Уровни внешних прерываний по источникам (до маски INTC.ENABLE).
    pub fn irq_lines(&self) -> u32 {
        let mut v = 0;
        if self.uart.rx_pending() {
            v |= intc::IRQ_UART_RX;
        }
        if self.radio.rx_pending() {
            v |= intc::IRQ_RADIO_RX;
        }
        v
    }

    /// Регистр `mip` в момент `now`.
    pub fn mip(&self, now: u64) -> u32 {
        let mut mip = 0;
        if self.clint.msip {
            mip |= MIP_MSIP;
        }
        if self.clint.timer_pending(now) {
            mip |= MIP_MTIP;
        }
        if self.irq_lines() & self.intc.enable != 0 {
            mip |= MIP_MEIP;
        }
        mip
    }

    /// Ближайший будущий такт, на котором что-то меняется само по себе
    /// (срабатывание таймера, истечение сторожа). `u64::MAX` — ничего.
    pub fn next_event(&self, now: u64) -> u64 {
        let t = self.clint.next_event(now);
        let w = self.wdt.expiry().filter(|&e| e > now).unwrap_or(u64::MAX);
        t.min(w)
    }

    pub fn read(&mut self, off: u32, size: u32, now: u64) -> Option<u32> {
        let (block, reg) = (off >> 12, off & 0xfff);
        if block == RADIO {
            return self.radio.read(reg, size);
        }
        if size != 4 {
            return None;
        }
        match block {
            SYSCTRL => self.sys.read(reg, now),
            GPIO => self.gpio.read(reg),
            PWM => self.pwm.read(reg),
            ADC if self.adc_inputs.is_some_and(|mask| reg < 4 * adc::CHANNELS as u32
                && (mask & (1 << (reg / 4))) == 0) => None,
            ADC => self.adc.read(reg),
            UART0 => self.uart.read(reg),
            WDT => self.wdt.read(reg, now),
            INTC => self.intc.read(reg, self.irq_lines()),
            _ => None,
        }
    }

    pub fn write(&mut self, off: u32, size: u32, val: u32, now: u64) -> bool {
        let (block, reg) = (off >> 12, off & 0xfff);
        if block == RADIO {
            let before = self.radio.tx_fifo.len();
            let ok = self.radio.write(reg, size, val);
            if self.host_timestamps && self.radio.tx_fifo.len() > before {
                self.radio_tx_cycles.push_back(now);
            }
            return ok;
        }
        if size != 4 {
            return false;
        }
        match block {
            SYSCTRL => match self.sys.write(reg, val) {
                Some(req) => {
                    if let Some(r) = req {
                        self.request = Some(r);
                    }
                    true
                }
                None => false,
            },
            GPIO => self.gpio.write(reg, val),
            PWM => self.pwm.write(reg, val),
            ADC => false,
            UART0 => {
                let before = self.uart.tx.len();
                let ok = self.uart.write(reg, val);
                if self.host_timestamps && self.uart.tx.len() > before {
                    self.uart_tx_cycles.push(now);
                }
                ok
            }
            WDT => self.wdt.write(reg, val, now),
            INTC => self.intc.write(reg, val),
            _ => false,
        }
    }
}
