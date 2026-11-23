//! CLINT — таймер машинного режима (раскладка SiFive CLINT, Privileged ISA 3.2.1).
//!
//! | смещение | регистр | доступ |
//! |---|---|---|
//! | 0x0000 | MSIP (бит 0 — программное прерывание) | RW |
//! | 0x4000 / 0x4004 | MTIMECMP младшее / старшее слово | RW |
//! | 0xBFF8 / 0xBFFC | MTIME младшее / старшее слово, 16 МГц | R |
//!
//! MTIME — общее время модели (такты / 10), сбросом не обнуляется. Прерывание
//! таймера ожидает, пока `mtime ≥ mtimecmp`. После сброса `mtimecmp = u64::MAX`.

use crate::bus::CYCLES_PER_MTIME;

#[derive(Clone, Debug)]
pub struct Clint {
    pub msip: bool,
    pub mtimecmp: u64,
}

impl Default for Clint {
    fn default() -> Self {
        Clint {
            msip: false,
            mtimecmp: u64::MAX,
        }
    }
}

impl Clint {
    #[inline]
    pub fn timer_pending(&self, now: u64) -> bool {
        now / CYCLES_PER_MTIME >= self.mtimecmp
    }

    /// Такт, на котором прерывание таймера станет ожидающим (если ещё не стало).
    pub fn next_event(&self, now: u64) -> u64 {
        if self.timer_pending(now) {
            u64::MAX
        } else {
            self.mtimecmp.saturating_mul(CYCLES_PER_MTIME)
        }
    }

    pub fn read(&self, off: u32, now: u64) -> Option<u32> {
        let mtime = now / CYCLES_PER_MTIME;
        Some(match off {
            0x0000 => self.msip as u32,
            0x4000 => self.mtimecmp as u32,
            0x4004 => (self.mtimecmp >> 32) as u32,
            0xbff8 => mtime as u32,
            0xbffc => (mtime >> 32) as u32,
            _ => return None,
        })
    }

    pub fn write(&mut self, off: u32, val: u32) -> bool {
        match off {
            0x0000 => self.msip = val & 1 != 0,
            0x4000 => self.mtimecmp = (self.mtimecmp & !0xffff_ffff) | val as u64,
            0x4004 => self.mtimecmp = (self.mtimecmp & 0xffff_ffff) | ((val as u64) << 32),
            _ => return false, // MTIME только для чтения
        }
        true
    }
}
