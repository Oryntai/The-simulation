//! WDT — аппаратный сторож.
//!
//! | смещение | регистр | доступ |
//! |---|---|---|
//! | 0x00 | CTRL — бит 0: включён (запись 1 запускает отсчёт заново) | RW |
//! | 0x04 | TIMEOUT_US — тайм-аут, мкс (по умолчанию 1 000 000) | RW |
//! | 0x08 | KICK — запись ключа 0x0D06_F00D перезапускает отсчёт; другие значения игнорируются | W |
//! | 0x0C | REMAINING_US — сколько осталось до сброса | R |
//!
//! По истечении тайм-аута — сброс всей микросхемы (ядро и периферия), причина
//! сброса «сторож», счётчик сбросов сторожем +1. Сброс выключает сторож.

use crate::bus::CPU_HZ;

pub const KICK_KEY: u32 = 0x0d06_f00d;
pub const DEFAULT_TIMEOUT_US: u32 = 1_000_000;

#[derive(Clone, Debug)]
pub struct Wdt {
    pub enabled: bool,
    pub timeout_us: u32,
    pub last_kick: u64,
}

impl Default for Wdt {
    fn default() -> Self {
        Wdt {
            enabled: false,
            timeout_us: DEFAULT_TIMEOUT_US,
            last_kick: 0,
        }
    }
}

impl Wdt {
    fn timeout_cycles(&self) -> u64 {
        self.timeout_us as u64 * (CPU_HZ / 1_000_000)
    }

    /// Такт истечения (если включён).
    pub fn expiry(&self) -> Option<u64> {
        self.enabled
            .then(|| self.last_kick.saturating_add(self.timeout_cycles()))
    }

    pub fn read(&self, reg: u32, now: u64) -> Option<u32> {
        Some(match reg {
            0x00 => self.enabled as u32,
            0x04 => self.timeout_us,
            0x08 => 0,
            0x0c => match self.expiry() {
                Some(e) => (e.saturating_sub(now) / (CPU_HZ / 1_000_000)) as u32,
                None => 0,
            },
            _ => return None,
        })
    }

    pub fn write(&mut self, reg: u32, v: u32, now: u64) -> bool {
        match reg {
            0x00 => {
                self.enabled = v & 1 != 0;
                self.last_kick = now;
            }
            0x04 => {
                self.timeout_us = v.max(1);
                self.last_kick = now;
            }
            0x08 => {
                if v == KICK_KEY {
                    self.last_kick = now;
                }
            }
            _ => return false,
        }
        true
    }
}
