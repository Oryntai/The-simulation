//! Номинальные цифровые частоты ESP32-C3, TRM v1.4 §§6.2.3–6.2.4.
//! Состояние питания и захват PLL принадлежат модели RTC.

use crate::{BoardFault, clock::CycleTime};

// ESP-IDF soc/esp32c3/register/soc/{reg_base,system_reg}.h.
pub(crate) const SYSTEM: u32 = 0x600c_0000;
pub(crate) const CPU_PER_CONF: u32 = SYSTEM + 0x008;
pub(crate) const SYSCLK_CONF: u32 = SYSTEM + 0x058;

// SYSTEM_SYSCLK_CONF, SYSTEM_CPU_PER_CONF, их поля и значения сброса.
const PRE_DIV: u32 = 0x3ff;
const SOC_CLK_SHIFT: u32 = 10;
const SOC_CLK: u32 = 3 << SOC_CLK_SHIFT;
const CPU_PERIOD: u32 = 3;
const PLL_480: u32 = 1 << 2;
const WAIT_FORCE_ON: u32 = 1 << 3;
const WAIT_DELAY_SHIFT: u32 = 4;
const WAIT_DELAY: u32 = 0xf << WAIT_DELAY_SHIFT;
const CPU_PER_MASK: u32 = CPU_PERIOD | PLL_480 | WAIT_FORCE_ON | WAIT_DELAY;
const SYSCLK_RESET: u32 = 1;
const CPU_PER_RESET: u32 = PLL_480 | WAIT_FORCE_ON;

// TRM §6.2.3 и таблицы 6.2-2/6.2-4. Полоса PLL, а не предел частоты CPU.
pub(crate) const XTAL_HZ: u32 = 40_000_000;
pub(crate) const PLL_320_HZ: u32 = 320_000_000;
pub(crate) const PLL_480_HZ: u32 = 480_000_000;
const APB_PLL_HZ: u32 = 80_000_000;
// Выбранная шкала точно представляет периоды 40/80/160 МГц и целые наносекунды.
pub(crate) const QUARTERS_PER_NS: u32 = 4;
pub(crate) const QUARTERS_PER_SECOND: u32 = 1_000_000_000 * QUARTERS_PER_NS;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EspClockSource {
    Xtal,
    Pll,
}

/// Частоты текущей конфигурации. Дробные частоты представлены отношением целых.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EspClockRates {
    source: EspClockSource,
    pll_selected_hz: u32,
    cpu: (u32, u16),
}

impl EspClockRates {
    pub fn source(self) -> EspClockSource {
        self.source
    }

    pub fn xtal_hz(self) -> u32 {
        XTAL_HZ
    }

    /// Настройка SYSTEM_PLL_FREQ_SEL. Доступность PLL определяется моделью RTC.
    pub fn pll_selected_hz(self) -> u32 {
        self.pll_selected_hz
    }

    /// Числитель и знаменатель частоты CPU в Гц.
    pub fn cpu_hz(self) -> (u32, u16) {
        self.cpu
    }

    /// Числитель и знаменатель частоты APB в Гц.
    pub fn apb_hz(self) -> (u32, u16) {
        match self.source {
            EspClockSource::Xtal => self.cpu,
            EspClockSource::Pll => (APB_PLL_HZ, 1),
        }
    }

    pub(crate) fn cpu_cycle_quarters(self) -> u32 {
        (QUARTERS_PER_SECOND / self.cpu.0) * u32::from(self.cpu.1)
    }
}

#[derive(Clone)]
pub(crate) struct EspClock {
    sysclk: u32,
    cpu_per: u32,
    rates: EspClockRates,
    pub(crate) time: CycleTime,
}

impl Default for EspClock {
    fn default() -> Self {
        Self {
            sysclk: SYSCLK_RESET,
            cpu_per: CPU_PER_RESET,
            rates: EspClockRates {
                source: EspClockSource::Xtal,
                pll_selected_hz: PLL_480_HZ,
                cpu: (XTAL_HZ, (SYSCLK_RESET + 1) as u16),
            },
            time: CycleTime::default(),
        }
    }
}

impl EspClock {
    pub(crate) fn rates(&self) -> EspClockRates {
        self.rates
    }

    pub(crate) fn wait_gate_cycles(&self) -> Option<u8> {
        (self.cpu_per & WAIT_FORCE_ON == 0)
            .then_some(((self.cpu_per & WAIT_DELAY) >> WAIT_DELAY_SHIFT) as u8)
    }

    pub(crate) fn access(&mut self, addr: u32, value: Option<u32>) -> Result<u32, BoardFault> {
        let Some(value) = value else {
            return Ok(if addr == SYSCLK_CONF {
                self.sysclk
            } else {
                self.cpu_per
            });
        };
        // Только R/W-поля. CLK_XTAL_FREQ и CLK_DIV_EN сброшены в 0 по system_reg.h.
        let (sysclk, cpu_per) = if addr == SYSCLK_CONF {
            (value & (SOC_CLK | PRE_DIV), self.cpu_per)
        } else {
            (self.sysclk, value & CPU_PER_MASK)
        };
        let pll_selected_hz = if cpu_per & PLL_480 != 0 {
            PLL_480_HZ
        } else {
            PLL_320_HZ
        };
        // TRM таблицы 6.2-1/6.2-2: 0 — XTAL, 1 — PLL, 2 — RC_FAST.
        let (source, cpu) = match (sysclk & SOC_CLK) >> SOC_CLK_SHIFT {
            0 => (
                EspClockSource::Xtal,
                (XTAL_HZ, ((sysclk & PRE_DIV) + 1) as u16),
            ),
            1 => {
                let cpu_hz = match cpu_per & CPU_PERIOD {
                    0 => APB_PLL_HZ,
                    1 => 2 * APB_PLL_HZ,
                    n => return Err(BoardFault::Unsupported(format!("SYSTEM_CPUPERIOD_SEL={n}"))),
                };
                (EspClockSource::Pll, (cpu_hz, 1))
            }
            2 => {
                return Err(BoardFault::Unsupported(
                    "CPU от RC_FAST требует модели настройки RTC".into(),
                ));
            }
            n => return Err(BoardFault::Unsupported(format!("SYSTEM_SOC_CLK_SEL={n}"))),
        };
        self.sysclk = sysclk;
        self.cpu_per = cpu_per;
        self.rates = EspClockRates {
            source,
            pll_selected_hz,
            cpu,
        };
        Ok(0)
    }
}
