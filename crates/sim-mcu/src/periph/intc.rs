//! INTC — контроллер внешних прерываний (линия MEIP ядра).
//!
//! | смещение | регистр | доступ |
//! |---|---|---|
//! | 0x00 | PENDING — уровни источников: бит 0 UART RX, бит 1 RADIO RX | R |
//! | 0x04 | ENABLE — маска источников | RW |
//!
//! `MEIP = (PENDING & ENABLE) ≠ 0`. Источники — уровневые: прерывание держится,
//! пока FIFO не опустошён.

pub const IRQ_UART_RX: u32 = 1 << 0;
pub const IRQ_RADIO_RX: u32 = 1 << 1;

#[derive(Clone, Debug, Default)]
pub struct Intc {
    pub enable: u32,
}

impl Intc {
    pub fn read(&self, reg: u32, lines: u32) -> Option<u32> {
        match reg {
            0x00 => Some(lines),
            0x04 => Some(self.enable),
            _ => None,
        }
    }

    pub fn write(&mut self, reg: u32, v: u32) -> bool {
        match reg {
            0x04 => {
                self.enable = v & (IRQ_UART_RX | IRQ_RADIO_RX);
                true
            }
            _ => false,
        }
    }
}
