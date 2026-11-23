//! SYSCTRL — системные регистры.
//!
//! | смещение | регистр | доступ |
//! |---|---|---|
//! | 0x00 | ID = 0x5643_3343 («VC3C») | R |
//! | 0x04 | RESET_REASON: 1 — питание, 2 — сторож, 3 — программный | R |
//! | 0x08 | WDT_RESETS — сбросов сторожем с включения питания | R |
//! | 0x0C / 0x10 | TIME_US младшее / старшее слово — время модели, мкс | R |
//! | 0x14 | CPU_HZ = 160 000 000 | R |
//! | 0x18 | HALT — запись останавливает ядро, значение = код выхода (для тестов) | W |
//! | 0x1C | SW_RESET — запись ключа 0x5257_5354 («TSWR») — программный сброс | W |
//! | 0x20 | FAULT_PC — адрес сбоя (пишет обработчик исключений прошивки) | RW |
//! | 0x24 | FAULT_TVAL — mtval сбоя | RW |
//! | 0x28 | FAULT_CAUSE — запись останавливает ядро как «сбой прошивки» с этой причиной | W |
//! | 0x2C | BOOT_COUNT — сбросов любого рода с включения питания | R |

use super::Request;
use crate::bus::CPU_HZ;

pub const ID: u32 = 0x5643_3343;
pub const SW_RESET_KEY: u32 = 0x5257_5354;

/// Причина последнего сброса.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[repr(u32)]
pub enum ResetReason {
    #[default]
    PowerOn = 1,
    Watchdog = 2,
    Software = 3,
}

#[derive(Clone, Debug, Default)]
pub struct SysCtrl {
    pub reset_reason: ResetReason,
    pub wdt_resets: u32,
    pub boot_count: u32,
    pub fault_pc: u32,
    pub fault_tval: u32,
}

impl SysCtrl {
    pub fn read(&self, reg: u32, now: u64) -> Option<u32> {
        let us = now / (CPU_HZ / 1_000_000);
        Some(match reg {
            0x00 => ID,
            0x04 => self.reset_reason as u32,
            0x08 => self.wdt_resets,
            0x0c => us as u32,
            0x10 => (us >> 32) as u32,
            0x14 => CPU_HZ as u32,
            0x18 | 0x1c | 0x28 => 0,
            0x20 => self.fault_pc,
            0x24 => self.fault_tval,
            0x2c => self.boot_count,
            _ => return None,
        })
    }

    /// `None` — нет такого регистра; `Some(req)` — запись принята (и, возможно, запрос).
    pub fn write(&mut self, reg: u32, v: u32) -> Option<Option<Request>> {
        Some(match reg {
            0x18 => Some(Request::Halt(v)),
            0x1c => (v == SW_RESET_KEY).then_some(Request::SoftwareReset),
            0x20 => {
                self.fault_pc = v;
                None
            }
            0x24 => {
                self.fault_tval = v;
                None
            }
            0x28 => Some(Request::FirmwareFault {
                cause: v,
                pc: self.fault_pc,
                tval: self.fault_tval,
            }),
            _ => return None,
        })
    }
}
