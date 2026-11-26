//! Запись PC/SP и контроль стека ESP32-C3, TRM v1.4 §§17.3.2–17.4.2.

use super::Access;
use crate::{BoardFault, esp_clock::SYSTEM};

// ESP-IDF soc/esp32c3/register/soc/{reg_base,system_reg}.h.
const CPU_CLOCK: u32 = SYSTEM;
const CPU_RESET: u32 = SYSTEM + 4;
const ASSIST_GATE: u32 = 1 << 6;
const DEDICATED_GPIO: u32 = 1 << 7;
const BASE: u32 = 0x600c_e000;
const END: u32 = BASE + 0x1000;
const WORD: u8 = size_of::<u32>() as u8;

// ESP-IDF soc/esp32c3/register/soc/assist_debug_reg.h.
const INTR_ENA: u32 = BASE;
const INTR_RAW: u32 = BASE + 0x04;
const INTR_RLS: u32 = BASE + 0x08;
const INTR_CLR: u32 = BASE + 0x0c;
const SP_MIN: u32 = BASE + 0x38;
const SP_MAX: u32 = BASE + 0x3c;
const SP_PC: u32 = BASE + 0x40;
const RECORD_EN: u32 = BASE + 0x44;
const RECORD_PC: u32 = BASE + 0x48;
const RECORD_SP: u32 = BASE + 0x4c;
const DEBUG_MODE: u32 = BASE + 0x98;
const BELOW: u32 = 1 << 8;
const ABOVE: u32 = 1 << 9;
const SP_BITS: u32 = BELOW | ABOVE;
const RECORD: u32 = 1;
const PDEBUG: u32 = 1 << 1;

#[derive(Clone)]
pub(super) struct EspAssist {
    clock: bool,
    reset: bool,
    monitor: u32,
    release: u32,
    raw: u32,
    irq: u32,
    clear: u32,
    min: u32,
    max: u32,
    spill_pc: u32,
    recording: u32,
    record_pc: u32,
    record_sp: u32,
}

impl Default for EspAssist {
    fn default() -> Self {
        // Состояния сброса из system_reg.h и assist_debug_reg.h.
        Self {
            clock: false,
            reset: true,
            monitor: 0,
            release: 0,
            raw: 0,
            irq: 0,
            clear: 0,
            min: 0,
            max: u32::MAX,
            spill_pc: 0,
            recording: 0,
            record_pc: 0,
            record_sp: 0,
        }
    }
}

impl EspAssist {
    pub(super) fn interrupt(&self) -> bool {
        self.clock && !self.reset && self.irq != 0
    }

    pub(super) fn observe(&mut self, pc: u32, sp: u32) {
        if !self.clock || self.reset {
            return;
        }
        // HAL esp32c3/assist_debug_ll.h: RAW показывает текущее условие,
        // RLS маскирует уровень на выходе, CLR снимает выход до следующей выборки.
        self.raw = self.monitor
            & ((u32::from(sp < self.min) * BELOW) | (u32::from(sp > self.max) * ABOVE));
        self.irq = self.raw & self.release & !std::mem::take(&mut self.clear);
        if self.raw != 0 {
            self.spill_pc = pc;
        }
        // TRM §17.4.2, рег. 17.18–17.20: PDEBUG и RECORD вместе разрешают запись.
        if self.recording == RECORD | PDEBUG {
            self.record_pc = pc;
            self.record_sp = sp;
        }
    }

    pub(super) fn access(&mut self, pc: u32, addr: u32, size: u8, value: Option<u32>) -> Access {
        let control = matches!(addr & !3, CPU_CLOCK | CPU_RESET);
        if !control && !(BASE..END).contains(&addr) {
            return Ok(None);
        }
        let unsupported = || BoardFault::UnsupportedRegister {
            pc,
            addr,
            size,
            write: value.is_some(),
            block: "ASSIST_DEBUG ESP32-C3",
        };
        if size != WORD {
            return Err(unsupported());
        }
        crate::memory::check_data_alignment(pc, addr, size)?;
        if control {
            // Выделанный GPIO остаётся в документированном состоянии сброса.
            let other = if addr == CPU_RESET { DEDICATED_GPIO } else { 0 };
            if let Some(bits) = value {
                if (bits ^ other) & !ASSIST_GATE != 0 {
                    return Err(unsupported());
                }
                let active = bits & ASSIST_GATE != 0;
                if addr == CPU_CLOCK {
                    self.clock = active;
                } else if active {
                    *self = Self {
                        clock: self.clock,
                        ..Self::default()
                    };
                } else {
                    self.reset = false;
                }
                return Ok(Some(0));
            }
            let active = if addr == CPU_CLOCK {
                self.clock
            } else {
                self.reset
            };
            return Ok(Some(other | (u32::from(active) * ASSIST_GATE)));
        }
        if !self.clock || self.reset {
            return Err(unsupported());
        }
        let result = match (addr, value) {
            (INTR_ENA, None) => self.monitor,
            (INTR_RLS, None) => self.release,
            (INTR_RAW, None) => self.raw,
            (SP_MIN, None) => self.min,
            (SP_MAX, None) => self.max,
            (SP_PC, None) => self.spill_pc,
            (RECORD_EN, None) => self.recording,
            (RECORD_PC, None) => self.record_pc,
            (RECORD_SP, None) => self.record_sp,
            (DEBUG_MODE, None) => 0, // В профиле нет внешнего отладчика (TRM гл. 17).
            (INTR_ENA, Some(bits)) if bits & !SP_BITS == 0 => {
                self.monitor = bits;
                0
            }
            (INTR_RLS, Some(bits)) if bits & !SP_BITS == 0 => {
                self.release = bits;
                0
            }
            (INTR_CLR, Some(bits)) if bits & !SP_BITS == 0 => {
                self.clear = bits;
                0
            }
            (SP_MIN, Some(bits)) => {
                self.min = bits;
                0
            }
            (SP_MAX, Some(bits)) => {
                self.max = bits;
                0
            }
            (RECORD_EN, Some(bits)) if bits & !(RECORD | PDEBUG) == 0 => {
                self.recording = bits;
                0
            }
            _ => return Err(unsupported()),
        };
        Ok(Some(result))
    }
}
