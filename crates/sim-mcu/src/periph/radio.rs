//! RADIO — обобщённое пакетное радио (в духе BLE/Wi-Fi): очереди пакетов до 251 байта.
//!
//! | смещение | регистр | доступ |
//! |---|---|---|
//! | 0x000 | STATUS — бит 0: есть принятый пакет; бит 1: очередь передачи полна; биты 8..=15: принятых в очереди; биты 16..=23: в очереди передачи | R |
//! | 0x004 | TX_LEN — запись длины 1..=251 отправляет TX_BUF[0..len] | W |
//! | 0x008 | RX_LEN — длина головного принятого пакета (0 — нет) | R |
//! | 0x00C | RX_RSSI — RSSI головного пакета, дБм (знаковое) | R |
//! | 0x010 | RX_POP — запись снимает головной пакет | W |
//! | 0x014 | TX_DROPPED — пакеты, отброшенные из-за полной очереди или неверной длины | R |
//! | 0x018 | RX_DROPPED — принятые пакеты, потерянные из-за полной очереди | R |
//! | 0x100..0x1FC | TX_BUF — буфер передачи (байт/полуслово/слово) | RW |
//! | 0x200..0x2FC | RX_BUF — данные головного принятого пакета | R |
//!
//! Глубина очередей — 8 пакетов. Эфир (задержка, потери, дальность) моделирует
//! хост (`sim-radio`): МК лишь кладёт пакеты в очередь и получает их из неё.
//! Прерывание INTC бит 1 — пока очередь приёма не пуста.

use std::collections::VecDeque;

pub const MAX_PACKET: usize = 251;
pub const FIFO_DEPTH: usize = 8;
const BUF_LEN: usize = 252;

#[derive(Clone, Debug)]
pub struct Radio {
    pub tx_buf: [u8; BUF_LEN],
    pub tx_fifo: VecDeque<Vec<u8>>,
    pub rx_fifo: VecDeque<(Vec<u8>, i8)>,
    pub tx_dropped: u32,
    pub rx_dropped: u32,
}

impl Default for Radio {
    fn default() -> Self {
        Radio {
            tx_buf: [0; BUF_LEN],
            tx_fifo: VecDeque::new(),
            rx_fifo: VecDeque::new(),
            tx_dropped: 0,
            rx_dropped: 0,
        }
    }
}

fn get_le(buf: &[u8], off: usize, size: u32) -> u32 {
    let mut v = 0u32;
    for k in 0..size as usize {
        v |= (*buf.get(off + k).unwrap_or(&0) as u32) << (8 * k);
    }
    v
}

impl Radio {
    pub fn reset(&mut self) {
        self.tx_buf = [0; BUF_LEN];
        self.rx_fifo.clear();
    }

    pub fn rx_pending(&self) -> bool {
        !self.rx_fifo.is_empty()
    }

    /// Хост → МК: пакет принят из эфира. `false` — отброшен (очередь полна или длина неверна).
    pub fn deliver(&mut self, data: &[u8], rssi_dbm: i8) -> bool {
        if data.is_empty() || data.len() > MAX_PACKET {
            return false;
        }
        if self.rx_fifo.len() >= FIFO_DEPTH {
            self.rx_dropped = self.rx_dropped.wrapping_add(1);
            return false;
        }
        self.rx_fifo.push_back((data.to_vec(), rssi_dbm));
        true
    }

    pub fn read(&mut self, reg: u32, size: u32) -> Option<u32> {
        let r = reg as usize;
        if (0x100..0x100 + BUF_LEN).contains(&r) {
            return Some(get_le(&self.tx_buf, r - 0x100, size));
        }
        if (0x200..0x200 + BUF_LEN).contains(&r) {
            let head = self.rx_fifo.front().map_or(&[][..], |(d, _)| d.as_slice());
            return Some(get_le(head, r - 0x200, size));
        }
        if size != 4 {
            return None;
        }
        Some(match reg {
            0x000 => {
                (self.rx_pending() as u32)
                    | (((self.tx_fifo.len() >= FIFO_DEPTH) as u32) << 1)
                    | ((self.rx_fifo.len() as u32) << 8)
                    | ((self.tx_fifo.len() as u32) << 16)
            }
            0x008 => self.rx_fifo.front().map_or(0, |(d, _)| d.len() as u32),
            0x00c => self.rx_fifo.front().map_or(0, |&(_, r)| r as i32 as u32),
            0x014 => self.tx_dropped,
            0x018 => self.rx_dropped,
            0x004 | 0x010 => 0,
            _ => return None,
        })
    }

    pub fn write(&mut self, reg: u32, size: u32, v: u32) -> bool {
        let r = reg as usize;
        if (0x100..0x100 + BUF_LEN).contains(&r) {
            for (k, byte) in v.to_le_bytes().into_iter().take(size as usize).enumerate() {
                if let Some(slot) = self.tx_buf.get_mut(r - 0x100 + k) {
                    *slot = byte;
                }
            }
            return true;
        }
        if size != 4 {
            return false;
        }
        match reg {
            0x004 => {
                let len = v as usize;
                if len == 0 || len > MAX_PACKET || self.tx_fifo.len() >= FIFO_DEPTH {
                    self.tx_dropped = self.tx_dropped.wrapping_add(1);
                } else {
                    self.tx_fifo.push_back(self.tx_buf[..len].to_vec());
                }
            }
            0x010 => {
                self.rx_fifo.pop_front();
            }
            _ => return false,
        }
        true
    }
}
