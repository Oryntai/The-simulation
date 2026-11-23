//! UART0: передача — в строковый буфер хоста, приём — FIFO от хоста.
//!
//! | смещение | регистр | доступ |
//! |---|---|---|
//! | 0x00 | TXDATA — младший байт уходит хосту | W |
//! | 0x04 | RXDATA — байт из FIFO; FIFO пуст → бит 31 = 1 | R (снимает байт) |
//! | 0x08 | STATUS — бит 0: есть байт; бит 1: TX готов (всегда); биты 16..=24: число байт в FIFO | R |
//! | 0x0C | RX_DROPPED — байты, потерянные из-за переполнения FIFO | R |
//!
//! Скорость линии не моделируется (байт передаётся мгновенно). FIFO приёма —
//! 256 байт. Прерывание INTC бит 0 — пока FIFO приёма не пуст.

use std::collections::VecDeque;

pub const RX_FIFO_DEPTH: usize = 256;
/// Предел накопленного вывода (защита памяти хоста, если его не забирают).
pub const TX_BUFFER_LIMIT: usize = 1 << 20;

#[derive(Clone, Debug, Default)]
pub struct Uart {
    pub tx: Vec<u8>,
    pub rx: VecDeque<u8>,
    pub rx_dropped: u32,
}

impl Uart {
    pub fn reset(&mut self) {
        self.rx.clear();
    }

    pub fn rx_pending(&self) -> bool {
        !self.rx.is_empty()
    }

    /// Хост → МК. Возвращает число принятых байт.
    pub fn host_send(&mut self, bytes: &[u8]) -> usize {
        let mut n = 0;
        for &b in bytes {
            if self.rx.len() < RX_FIFO_DEPTH {
                self.rx.push_back(b);
                n += 1;
            } else {
                self.rx_dropped = self.rx_dropped.wrapping_add(1);
            }
        }
        n
    }

    pub fn read(&mut self, reg: u32) -> Option<u32> {
        Some(match reg {
            0x00 => 0,
            0x04 => self.rx.pop_front().map_or(0x8000_0000, |b| b as u32),
            0x08 => (self.rx_pending() as u32) | 2 | ((self.rx.len() as u32) << 16),
            0x0c => self.rx_dropped,
            _ => return None,
        })
    }

    pub fn write(&mut self, reg: u32, v: u32) -> bool {
        match reg {
            0x00 => {
                if self.tx.len() < TX_BUFFER_LIMIT {
                    self.tx.push(v as u8);
                }
                true
            }
            _ => false,
        }
    }
}
