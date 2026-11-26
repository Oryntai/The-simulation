//! MWDT в группе таймеров с тактовым деревом raw flash, TRM §§11.5 и 12.2.

use super::{Access, esp_rtc_cal::RtcCalibration, ro, rw};
use crate::{
    BoardFault,
    esp_clock::{EspClockRates, QUARTERS_PER_SECOND, XTAL_HZ},
};

// ESP-IDF soc/esp32c3/register/soc/{reg_base,timer_group_reg,system_reg}.h.
pub(super) const BASES: [u32; 2] = [0x6001_f000, 0x6002_0000];
pub(super) const GATES: [u32; 2] = [1 << 13, 1 << 15];
const CONFIG: u32 = 0x48;
const DIVIDER: u32 = 0x4c;
const HOLD: u32 = 0x50;
const FEED: u32 = 0x60;
const PROTECT: u32 = 0x64;
const INT_ENA: u32 = 0x70;
const INT_RAW: u32 = 0x74;
const INT_ST: u32 = 0x78;
const INT_CLR: u32 = 0x7c;
const REGCLK: u32 = 0xfc;
const WORD: u8 = size_of::<u32>() as u8;

// TRM рег. 11.10–11.16 и 11.21–11.26.
const CONFIG_RW: u32 = 0xffbf_f000;
const ENABLE: u32 = 1 << 31;
const UPDATE: u32 = 1 << 22;
const XTAL: u32 = 1 << 21;
const FLASHBOOT: u32 = 1 << 14;
const RESET_CONFIG: u32 = (1 << 18) | (1 << 15) | FLASHBOOT;
const DIVCNT_RESET: u32 = 1;
const DIV_MASK: u32 = 0xffff << 16;
const DIV_SHIFT: u32 = 16;
const KEY: u32 = 0x50d8_3aa1;
const WDT_INT: u32 = 1 << 1;
const WDT_CLOCK: u32 = 1 << 29;
const TIMER_CLOCK: u32 = 1 << 30;
const REG_CLOCK: u32 = 1 << 31;
const STG_SHIFTS: [u32; 4] = [29, 27, 25, 23];

#[derive(Clone)]
struct WdtSettings {
    config: u32,
    div: u32,
    hold: [u32; STG_SHIFTS.len()],
}

impl Default for WdtSettings {
    fn default() -> Self {
        Self {
            config: RESET_CONFIG,
            div: 1 << DIV_SHIFT,
            hold: [26_000_000, 134_217_727, 1_048_575, 1_048_575],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(in crate::peripheral) enum MwdtReset {
    Cpu,
    Core,
}

#[derive(Clone)]
pub(super) struct EspTimg {
    calibration: RtcCalibration,
    shadow: WdtSettings,
    active: WdtSettings,
    flash_boot: bool,
    key: u32,
    stage: usize,
    ticks: u64,
    phase: u128,
    clock: u32,
    int_ena: u32,
    int_raw: u32,
}

impl Default for EspTimg {
    fn default() -> Self {
        Self {
            calibration: RtcCalibration::default(),
            shadow: WdtSettings::default(),
            active: WdtSettings::default(),
            flash_boot: false,
            key: KEY, // TIMG_WDT_WKEY, TRM рег. 11.17.
            stage: 0,
            ticks: 0,
            phase: 0,
            clock: TIMER_CLOCK | WDT_CLOCK,
            int_ena: 0,
            int_raw: 0,
        }
    }
}

impl EspTimg {
    pub(super) fn enter_flash_boot(&mut self) {
        self.flash_boot = true;
        self.feed();
    }

    fn feed(&mut self) {
        self.stage = 0;
        self.ticks = 0;
    }

    fn boot_protection(&self) -> bool {
        self.flash_boot && self.active.config & FLASHBOOT != 0
    }

    fn running(&self) -> bool {
        self.boot_protection() || self.active.config & ENABLE != 0
    }

    fn config(&mut self, word: u32) -> Result<u32, BoardFault> {
        if word & !(CONFIG_RW | UPDATE) != 0 {
            return Err(BoardFault::Unsupported(
                "MWDT: зарезервированные поля WDTCONFIG0".into(),
            ));
        }
        let mut next = self.clone();
        next.shadow.config = word & CONFIG_RW;
        if word & UPDATE != 0 {
            next.active = next.shadow.clone();
        } else {
            // ESP-IDF esp32c3/hal/wdt_ll.h: enable/disable пишут EN без UPDATE.
            next.active.config = next.active.config & !ENABLE | word & ENABLE;
        }
        if next.running() && (next.active.div == 0 || next.active.hold.contains(&0)) {
            return Err(BoardFault::Unsupported(
                "MWDT: нулевой делитель или порог ступени".into(),
            ));
        }
        if !self.running() && next.running() {
            next.feed();
        }
        *self = next;
        Ok(0)
    }

    pub(super) fn access(
        &mut self,
        pc: u32,
        addr: u32,
        offset: u32,
        size: u8,
        value: Option<u32>,
        bus_enabled: bool,
    ) -> Access {
        let unknown = || BoardFault::UnsupportedRegister {
            pc,
            addr,
            size,
            write: value.is_some(),
            block: "TIMG raw flash",
        };
        if size != WORD || !bus_enabled {
            return Err(unknown());
        }
        if let Some(word) = self.calibration.access(offset, value)? {
            return Ok(Some(word));
        }
        if self.key != KEY && value.is_some() && (CONFIG..=FEED).contains(&offset) {
            return Ok(Some(0)); // TRM §12.2.2.3: защищённая запись игнорируется.
        }
        match offset {
            CONFIG => match value {
                Some(word) => self.config(word).map(Some),
                None => Ok(Some(self.shadow.config)),
            },
            DIVIDER => {
                if let Some(word) = value {
                    if word & !(DIV_MASK | DIVCNT_RESET) != 0 {
                        return Err(unknown());
                    }
                    if word & DIVCNT_RESET != 0 {
                        self.phase = 0;
                    }
                }
                rw(value, &mut self.shadow.div, DIV_MASK)
            }
            HOLD..=0x5c => rw(
                value,
                &mut self.shadow.hold[((offset - HOLD) / u32::from(WORD)) as usize],
                u32::MAX,
            ),
            FEED => value.map_or(Ok(Some(0)), |_| {
                self.feed();
                Ok(Some(0))
            }),
            PROTECT => rw(value, &mut self.key, u32::MAX),
            INT_ENA if value.is_none_or(|word| word & !WDT_INT == 0) => {
                rw(value, &mut self.int_ena, WDT_INT)
            }
            INT_RAW if value.is_none() => ro(value, self.int_raw),
            INT_ST if value.is_none() => ro(value, self.int_raw & self.int_ena),
            INT_CLR if value.is_none_or(|word| word & !WDT_INT == 0) => {
                if let Some(word) = value {
                    self.int_raw &= !word;
                }
                Ok(Some(0))
            }
            REGCLK
                if value.is_none_or(|word| word & !(REG_CLOCK | TIMER_CLOCK | WDT_CLOCK) == 0) =>
            {
                rw(value, &mut self.clock, REG_CLOCK | TIMER_CLOCK | WDT_CLOCK)
            }
            _ => Err(unknown()),
        }
    }

    pub(super) fn advance(
        &mut self,
        elapsed_quarters: u128,
        rates: EspClockRates,
    ) -> Option<MwdtReset> {
        self.calibration.advance(elapsed_quarters);
        if !self.running() || self.clock & WDT_CLOCK == 0 || elapsed_quarters == 0 {
            return None;
        }
        // TRM §12.2.2.1: prescale тактов выбранного XTAL или APB.
        let (hz, div) = if self.active.config & XTAL != 0 {
            (XTAL_HZ, 1)
        } else {
            rates.apb_hz()
        };
        let period = u128::from(QUARTERS_PER_SECOND / hz)
            * u128::from(div)
            * u128::from(self.active.div >> DIV_SHIFT);
        let phase = self.phase + elapsed_quarters;
        let mut remaining = phase / period;
        self.phase = phase % period;
        while remaining > 0 {
            let until = u64::from(self.active.hold[self.stage]).saturating_sub(self.ticks);
            if remaining < u128::from(until) {
                self.ticks += remaining as u64;
                break;
            }
            remaining -= u128::from(until);
            self.ticks = 0;
            let action = if self.boot_protection() && self.stage == 0 {
                3 // TRM §12.2.2.4: core reset при истечении защиты загрузки.
            } else {
                (self.active.config >> STG_SHIFTS[self.stage]) & 3
            };
            self.stage = (self.stage + 1) % STG_SHIFTS.len();
            match action {
                1 => self.int_raw |= WDT_INT,
                2 => return Some(MwdtReset::Cpu),
                3 => return Some(MwdtReset::Core),
                _ => (),
            }
        }
        None
    }
}
