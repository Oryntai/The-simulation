//! Цифровая часть RTC_CNTL ESP32-C3. TRM v1.4, §§6.1, 9.3, 9.7 и 12.2.

use super::{Access, Hardware};
use crate::{BoardFault, BoardOutput, OutputEvent, ResetCause};

// TRM §9.7; ESP-IDF v6.1.1, soc/esp32c3/register/soc/{reg_base,rtc_cntl_reg}.h.
const BASE: u32 = 0x6000_8000;
const DATE: u32 = 0x1fc;
const OPTIONS: u32 = 0x00;
const TIME_UPDATE: u32 = 0x0c;
const TIME_LOW: u32 = 0x10;
const TIME_HIGH: u32 = 0x14;
const TIMER1: u32 = 0x1c;
const TIMER6: u32 = 0x30;
const ANA_CONF: u32 = 0x34;
const RESET_STATE: u32 = 0x38;
const WAKEUP_STATE: u32 = 0x3c;
const INT_ENA: u32 = 0x40;
const INT_RAW: u32 = 0x44;
const INT_ST: u32 = 0x48;
const INT_CLR: u32 = 0x4c;
const STORE0: u32 = 0x50;
const STORE3: u32 = 0x5c;
const CLK_CONF: u32 = 0x70;
const SLOW_CLK_CONF: u32 = 0x74;
const REGULATOR: u32 = 0x80;
const RTC_PWC: u32 = 0x84;
const DIG_PWC: u32 = 0x88;
const DIG_ISO: u32 = 0x8c;
const WDT_CONFIG0: u32 = 0x90;
const WDT_CONFIG1: u32 = 0x94;
const WDT_CONFIG4: u32 = 0xa0;
const WDT_FEED: u32 = 0xa4;
const WDT_WPROTECT: u32 = 0xa8;
const SWD_CONF: u32 = 0xac;
const SWD_WPROTECT: u32 = 0xb0;
const STORE4: u32 = 0xb8;
const STORE7: u32 = 0xc4;
const RTC_PAD_HOLD: u32 = 0xd0;
const DIG_PAD_HOLD: u32 = 0xd4;
const INT_ENA_SET: u32 = 0x100;
const INT_ENA_CLEAR: u32 = 0x104;
const FIB_SEL: u32 = 0x10c;

const WORD: u8 = size_of::<u32>() as u8;

// TRM регистры 9.1, 9.4, 9.12 и §9.3.3. Резервный бит 12 RESET_STATE
// имеет единицу в таблице сброса, поля единственного CPU начинаются с бита 13.
const SW_CORE_RESET: u32 = 1 << 31;
const SW_CPU_RESET: u32 = 1 << 5;
const OPTIONS_RW: u32 = (0x7ff << 20) | (0x0fff << 6) | (0b11 << 2);
const OPTIONS_RESET: u32 = (0b111 << 26) | (2 << 14) | (1 << 13);
const RESET_CAUSE_MASK: u32 = 0x3f;
const STATIC_VECTOR: u32 = 1 << 13;
const RESET_FLAG: u32 = 1 << 14;
const CLEAR_RESET_FLAG: u32 = 1 << 16;
const RESET_STATE_UNSUPPORTED: u32 = (1 << 19) | (1 << 25);
const TIME_LATCH: u32 = 1 << 31;
const TIME_MASK: u64 = (1 << 48) - 1;

// TRM регистры 9.14–9.17, 9.31–9.39; rtc_cntl_reg.h для CHIP_RESET_EN.
const WDT_INT: u32 = 1 << 3;
const WDT_ENABLE: u32 = 1 << 31;
const FLASHBOOT_ENABLE: u32 = 1 << 12;
const PROCPU_RESET_EN: u32 = 1 << 11;
const PAUSE_IN_SLEEP: u32 = 1 << 9;
const CHIP_RESET_EN: u32 = 1 << 8;
const WDT_CONFIG0_RESET: u32 = (1 << 16) | (1 << 13) | FLASHBOOT_ENABLE | PAUSE_IN_SLEEP | 20;
const WDT_STAGE_SHIFTS: [u32; 4] = [28, 25, 22, 19];
const WDT_STAGE_MASK: u32 = 0b111;
const WDT_FEED_CMD: u32 = 1 << 31;
const WDT_KEY: u32 = 0x50d8_3aa1;
const SWD_KEY: u32 = 0x8f1d_312a;
const SWD_CONF_RW: u32 = (0b11 << 30) | (0x3ff << 18) | (1 << 17);
const SWD_FEED_CMD: u32 = 1 << 29;
const SWD_CONF_RESET: u32 = 300 << 18;

// TRM регистры 9.27–9.30 и rtc_cntl_reg.h. BT-поля [27, 12, 11]
// приведены в заголовке IDF для C3, в регистре 9.29 TRM они зарезервированы.
const REGULATOR_RW: u32 = (0xf << 28) | (0xff << 14) | (1 << 7);
const REGULATOR_RESET: u32 = (1 << 31) | (1 << 29);
const PAD_FORCE_HOLD: u32 = 1 << 21;
const DIG_PWC_RW: u32 = (0x1f << 27) | (0xfff << 11) | 0x1f;
const DIG_PWC_RESET: u32 =
    (1 << 22) | (1 << 20) | (1 << 18) | (1 << 16) | (1 << 14) | (1 << 12) | (1 << 4);
const DIG_ISO_RW: u32 = (0x3ff << 22) | (0x1f << 11) | (0b11 << 7);
const DIG_ISO_RESET: u32 = (0x2aa << 22) | (1 << 14) | (1 << 12) | (1 << 7);
const DIG_PAD_ISO_OR_HOLD: u32 = (1 << 15) | (1 << 13);

// rtc_cntl_reg.h: TIMER1–TIMER6, маски R/W и значения сброса полей.
const POWER_TIMERS: [(u32, u32); 6] = [
    (
        u32::MAX,
        (40 << 24) | (80 << 14) | (0x10 << 6) | (1 << 1) | 1,
    ),
    (0xff << 24, 1 << 24),
    (u32::MAX, (5 << 25) | (8 << 16) | (5 << 9) | 8),
    (u32::MAX, (8 << 25) | (0x20 << 16) | (5 << 9) | 8),
    (0xff << 8, 0x80 << 8),
    (0xffff << 16, (5 << 25) | (8 << 16)),
];
// RTC_CNTL_WAKEUP_ENA[31:15], reset 12 (TRM рег. 9.13).
const WAKEUP_STATE_MASK: u32 = 0x1ffff << 15;
const WAKEUP_STATE_RESET: u32 = 12 << 15;

// TRM §6.2.3, табл. 9.3-1 и даташит v2.5 §4.1.3.3: номинал RC_SLOW_CLK.
const RC_SLOW_HZ: u32 = 136_000;
// TRM регистр 9.25 и поля CLK_CONF в rtc_cntl_reg.h ESP-IDF.
const SLOW_SOURCE: u32 = 0b11 << 30;
const CLK_CONF_RW: u32 = (u32::MAX << 12) | (0x3ff << 1);
const CLK_CONF_RESET: u32 = (1 << 28) | (172 << 17) | (3 << 12) | (1 << 9) | (1 << 4) | (1 << 3);
// TRM регистр 9.26: NEXT_EDGE[31] и ANA_CLK_DIV_VLD[22].
const SLOW_CLK_CONF_RW: u32 = 0x3ff << 22;
const SLOW_CLK_CONF_RESET: u32 = 1 << 22;
const SLOW_CLK_CONF_DIV: u32 = 0xff << 23;
// ESP-IDF rtc_cntl_reg.h: ANA_CONF, PAD_HOLD, DIG_PAD_HOLD, FIB_SEL.
const ANA_CONF_RW: u32 = (0b11 << 30) | (0x7f << 22) | (0b111 << 18);
const ANA_CONF_RESET: u32 = (1 << 23) | (1 << 22) | (1 << 18);
const RTC_PAD_MASK: u32 = (1 << 6) - 1;
const DIG_PAD_MASK: u32 = (1 << 22) - 1;
const FIB_SEL_MASK: u32 = 0b111;

/// Причина из RTC_CNTL_RESET_CAUSE_PROCPU. TRM табл. 6.1-1 и
/// ESP-IDF soc/esp32c3/include/soc/reset_reasons.h.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum EspRtcReset {
    PowerOn = 0x01,
    SoftwareCore = 0x03,
    MainWatchdog0 = 0x07,
    MainWatchdog1 = 0x08,
    WatchdogCore = 0x09,
    SoftwareCpu = 0x0c,
    WatchdogCpu = 0x0d,
    WatchdogSystem = 0x10,
}

impl EspRtcReset {
    /// TRM §6.1.1. Внутренняя память сохраняется при CPU, Core и System Reset.
    pub fn retains_rtc_fast(self) -> bool {
        self != Self::PowerOn
    }

    fn cpu_only(self) -> bool {
        matches!(self, Self::SoftwareCpu | Self::WatchdogCpu)
    }

    fn clears_controller(self) -> bool {
        matches!(self, Self::PowerOn | Self::WatchdogSystem)
    }
}

/// Логические состояния доменов в активном режиме. Для RTC FAST отдельно
/// возвращается принудительная политика удержания в сне по TRM регистру 9.29.
/// None оставляет выбор автомату сна.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EspRtcDomains {
    pub cpu: bool,
    pub digital: bool,
    pub peripherals: bool,
    pub wifi: bool,
    pub bluetooth: bool,
    pub rtc_fast_sleep_retention: Option<bool>,
}

#[derive(Clone)]
struct Watchdog {
    config: u32,
    hold: [u32; WDT_STAGE_SHIFTS.len()],
    key: u32,
    flash_boot: bool,
    stage: usize,
    elapsed: u64,
}

impl Default for Watchdog {
    fn default() -> Self {
        Self {
            config: WDT_CONFIG0_RESET,
            hold: [200_000, 80_000, 0xfff, 0xfff], // TRM регистры 9.32–9.35.
            key: 0,                                // TRM регистр 9.37: заблокирован после сброса.
            flash_boot: false,
            stage: 0,
            elapsed: 0,
        }
    }
}

impl Watchdog {
    fn active(&self) -> bool {
        self.config & WDT_ENABLE != 0 || self.flash_boot && self.config & FLASHBOOT_ENABLE != 0
    }

    fn feed(&mut self) {
        self.stage = 0;
        self.elapsed = 0;
    }

    fn action(&self, stage: usize) -> u32 {
        // TRM §12.2.2.4: для защиты загрузки ступень 0 сбрасывает систему с RTC.
        if stage == 0 && self.flash_boot && self.config & FLASHBOOT_ENABLE != 0 {
            return 4;
        }
        (self.config >> WDT_STAGE_SHIFTS[stage]) & WDT_STAGE_MASK
    }

    fn timeout(&self, stage: usize, delay_sel: u8) -> u64 {
        let hold = u64::from(self.hold[stage]);
        // TRM §12.2.2.2: только STG0 масштабируется полем eFuse.
        if stage == 0 {
            hold << (delay_sel + 1)
        } else {
            hold
        }
    }

    fn validate(&self, delay_sel: Option<u8>) -> Result<(), BoardFault> {
        // TRM регистр 9.31: значения 5–7 не описаны.
        if WDT_STAGE_SHIFTS
            .iter()
            .any(|&shift| (self.config >> shift) & WDT_STAGE_MASK > 4)
        {
            return Err(unsupported("код действия ступени RWDT"));
        }
        if self.config & CHIP_RESET_EN != 0 {
            return Err(unsupported("аналоговый Chip Reset от RWDT"));
        }
        if self.active() {
            let delay = delay_sel
                .ok_or_else(|| unsupported("для RWDT нужен снимок EFUSE_WDT_DELAY_SEL"))?;
            if self.hold.contains(&0) || self.elapsed >= self.timeout(self.stage, delay) {
                return Err(unsupported("нулевой или уже пройденный порог RWDT"));
            }
        }
        Ok(())
    }

    fn reset_action(&self, stage: usize) -> Option<EspRtcReset> {
        match self.action(stage) {
            2 if self.config & PROCPU_RESET_EN != 0 => Some(EspRtcReset::WatchdogCpu),
            3 => Some(EspRtcReset::WatchdogCore),
            4 => Some(EspRtcReset::WatchdogSystem),
            _ => None,
        }
    }

    // Возвращает число реально пройденных тактов до первого сброса.
    fn advance(&mut self, mut ticks: u64, delay: u8, raw: &mut u32) -> (u64, Option<EspRtcReset>) {
        let total = ticks;
        while ticks > 0 {
            if self.stage == 0 && self.elapsed == 0 {
                let stages = 0..self.hold.len();
                if !stages
                    .clone()
                    .any(|stage| self.reset_action(stage).is_some())
                {
                    let period = stages
                        .clone()
                        .map(|stage| self.timeout(stage, delay))
                        .sum::<u64>();
                    if ticks >= period && stages.into_iter().any(|stage| self.action(stage) == 1) {
                        *raw |= WDT_INT;
                    }
                    ticks %= period;
                }
            }
            let until = self.timeout(self.stage, delay) - self.elapsed;
            if ticks < until {
                self.elapsed += ticks;
                break;
            }
            ticks -= until;
            let next_reset = self.reset_action(self.stage);
            if self.action(self.stage) == 1 {
                *raw |= WDT_INT;
            }
            self.stage = (self.stage + 1) % self.hold.len();
            self.elapsed = 0;
            if next_reset.is_some() {
                return (total - ticks, next_reset);
            }
        }
        (total, None)
    }
}

/// MMIO и временное состояние RTC_CNTL. Внешняя модель передаёт такты RTC_SLOW_CLK.
#[derive(Clone)]
pub struct EspRtc {
    options: u32,
    clk_conf: u32,
    slow_clk_conf: u32,
    ana_conf: u32,
    power_timers: [u32; POWER_TIMERS.len()],
    wakeup_state: u32,
    regulator: u32,
    rtc_power: u32,
    dig_power: u32,
    dig_iso: u32,
    rtc_pad_hold: u32,
    dig_pad_hold: u32,
    fib_sel: u32,
    store: [u32; 8], // TRM §9.3, STORE0–STORE7.
    reason: Option<EspRtcReset>,
    pending_reset: Option<EspRtcReset>,
    reset_flag: bool,
    int_raw: u32,
    int_enable: u32,
    watchdog: Watchdog,
    wdt_delay_sel: Option<u8>,
    swd: u32,
    swd_key: u32,
    slow_ticks: u64,
    latched_time: u64,
    cpu_phase: u64,
}

impl Default for EspRtc {
    fn default() -> Self {
        Self {
            options: OPTIONS_RESET,
            clk_conf: CLK_CONF_RESET,
            slow_clk_conf: SLOW_CLK_CONF_RESET,
            ana_conf: ANA_CONF_RESET,
            power_timers: POWER_TIMERS.map(|(_, reset)| reset),
            wakeup_state: WAKEUP_STATE_RESET,
            regulator: REGULATOR_RESET,
            rtc_power: 0,
            dig_power: DIG_PWC_RESET,
            dig_iso: DIG_ISO_RESET,
            rtc_pad_hold: 0,
            dig_pad_hold: 0,
            fib_sel: FIB_SEL_MASK,
            store: [0; 8],
            reason: Some(EspRtcReset::PowerOn),
            pending_reset: None,
            reset_flag: true,
            int_raw: 0,
            int_enable: 0,
            watchdog: Watchdog::default(),
            wdt_delay_sel: None,
            swd: SWD_CONF_RESET,
            swd_key: 0, // TRM регистр 9.39.
            slow_ticks: 0,
            latched_time: 0,
            cpu_phase: 0,
        }
    }
}

impl EspRtc {
    /// EFUSE_WDT_DELAY_SEL из EFUSE_RD_REPEAT_DATA1_REG[17:16], ESP-IDF efuse_reg.h.
    pub fn new(wdt_delay_sel: u8) -> Result<Self, BoardFault> {
        if wdt_delay_sel > 0b11 {
            return Err(BoardFault::Misconfigured(
                "EFUSE_WDT_DELAY_SEL вне двух битов".into(),
            ));
        }
        Ok(Self {
            wdt_delay_sel: Some(wdt_delay_sel),
            ..Self::default()
        })
    }

    /// Сигнал входа в загрузку из flash по TRM §12.2.2.4.
    pub fn enter_flash_boot(&mut self) -> Result<(), BoardFault> {
        let mut next = self.watchdog.clone();
        next.flash_boot = true;
        next.feed();
        next.validate(self.wdt_delay_sel)?;
        self.watchdog = next;
        Ok(())
    }

    pub fn reset_reason(&self) -> Option<EspRtcReset> {
        self.reason
    }

    pub fn pending_reset(&self) -> Option<EspRtcReset> {
        self.pending_reset
    }

    /// Применяет сброс. CPU/Core сохраняют RTC_CNTL, System сбрасывает его.
    /// RTC FAST хранится в шине памяти и живёт по правилу retains_rtc_fast.
    pub fn reset(&mut self, reason: EspRtcReset) {
        if reason.clears_controller() {
            *self = Self {
                wdt_delay_sel: self.wdt_delay_sel,
                ..Self::default()
            };
        }
        self.reason = Some(reason);
        self.reset_flag = true;
        self.pending_reset = None;
    }

    pub fn interrupt_pending(&self) -> bool {
        self.int_raw & self.int_enable != 0
    }

    pub fn domains(&self) -> EspRtcDomains {
        // RTC_CNTL_DIG_PWC_REG: FORCE_PD для CPU_TOP, DG_WRAP, DG_PERI, WIFI и BT.
        let powered = |force_pd: u32| (self.dig_power & (1 << force_pd)) == 0;
        let digital = powered(19);
        EspRtcDomains {
            cpu: digital && powered(21),
            digital,
            peripherals: digital && powered(13),
            wifi: digital && powered(17),
            bluetooth: digital && powered(11),
            rtc_fast_sleep_retention: match self.dig_power & (0b11 << 15) {
                0 => None,
                value => Some(value & (1 << 16) != 0),
            },
        }
    }

    /// Считает целые такты до конца интервала или запроса сброса.
    /// Оставшееся после сброса время передаётся следующим вызовом после reset.
    pub fn advance_slow(&mut self, ticks: u64, sleeping: bool) -> u64 {
        if self.pending_reset.is_some() {
            return 0;
        }
        let (elapsed, reset) = if self.watchdog.active()
            && !(sleeping && self.watchdog.config & PAUSE_IN_SLEEP != 0)
        {
            // Активация проверяет eFuse и ненулевые пороги атомарно.
            self.watchdog
                .advance(ticks, self.wdt_delay_sel.unwrap(), &mut self.int_raw)
        } else {
            (ticks, None)
        };
        self.pending_reset = reset;
        self.slow_ticks = self.slow_ticks.wrapping_add(elapsed) & TIME_MASK;
        elapsed
    }

    /// Словный доступ. Вне окна возвращает None, неподдержанный регистр даёт ошибку с PC.
    pub fn access(
        &mut self,
        pc: u32,
        addr: u32,
        size: u8,
        value: Option<u32>,
    ) -> Result<Option<u32>, BoardFault> {
        if !(BASE..BASE + DATE + u32::from(WORD)).contains(&addr) {
            return Ok(None);
        }
        let unknown = || BoardFault::UnsupportedRegister {
            pc,
            addr,
            size,
            write: value.is_some(),
            block: "RTC_CNTL ESP32-C3",
        };
        if size != WORD {
            return Err(unknown());
        }
        crate::memory::check_data_alignment(pc, addr, size)?;
        match self.register(addr - BASE, value)? {
            Some(word) => Ok(Some(word)),
            None => Err(unknown()),
        }
    }

    fn register(&mut self, offset: u32, value: Option<u32>) -> Access {
        let result = match offset {
            OPTIONS => self.options(value)?,
            TIME_UPDATE => {
                if value.is_some_and(|word| word & !TIME_LATCH != 0) {
                    return Err(unsupported("снимок RTC от события сна"));
                }
                if value.is_some_and(|word| word & TIME_LATCH != 0) {
                    self.latched_time = self.slow_ticks;
                }
                0
            }
            TIME_LOW if value.is_none() => self.latched_time as u32,
            TIME_HIGH if value.is_none() => (self.latched_time >> u32::BITS) as u32,
            TIMER1..=TIMER6 => {
                let index = ((offset - TIMER1) / u32::from(WORD)) as usize;
                rw(&mut self.power_timers[index], POWER_TIMERS[index].0, value)
            }
            RESET_STATE => self.reset_state(value)?,
            WAKEUP_STATE => rw(&mut self.wakeup_state, WAKEUP_STATE_MASK, value),
            INT_ENA | INT_ENA_SET | INT_ENA_CLEAR | INT_CLR => {
                self.interrupt_register(offset, value)?
            }
            INT_RAW if value.is_none() => self.int_raw,
            INT_ST if value.is_none() => self.int_raw & self.int_enable,
            STORE0..=STORE3 | STORE4..=STORE7 => {
                let index = if offset <= STORE3 {
                    (offset - STORE0) / u32::from(WORD)
                } else {
                    (offset - STORE4) / u32::from(WORD) + 4
                };
                rw(&mut self.store[index as usize], u32::MAX, value)
            }
            ANA_CONF => rw(&mut self.ana_conf, ANA_CONF_RW, value),
            CLK_CONF => {
                if value.is_some_and(|word| word & SLOW_SOURCE == SLOW_SOURCE) {
                    return Err(unsupported("зарезервированный источник RTC_SLOW_CLK"));
                }
                rw(&mut self.clk_conf, CLK_CONF_RW, value)
            }
            SLOW_CLK_CONF => rw(&mut self.slow_clk_conf, SLOW_CLK_CONF_RW, value),
            REGULATOR => rw(&mut self.regulator, REGULATOR_RW, value),
            RTC_PWC => rw(&mut self.rtc_power, PAD_FORCE_HOLD, value),
            DIG_PWC => {
                // RTC_CNTL_DIG_PWC: пары FORCE_PU/FORCE_PD и FASTMEM_FORCE_LPU/LPD.
                const DOWN: u32 = (1 << 21)
                    | (1 << 19)
                    | (1 << 17)
                    | (1 << 15)
                    | (1 << 13)
                    | (1 << 11)
                    | (1 << 3);
                if value.is_some_and(|word| word & (word >> 1) & DOWN != 0) {
                    return Err(unsupported("одновременно FORCE_PU и FORCE_PD в DIG_PWC"));
                }
                rw(&mut self.dig_power, DIG_PWC_RW, value)
            }
            DIG_ISO => {
                // RTC_CNTL_DIG_ISO: четыре пары FORCE_ISO/FORCE_NOISO, hold и изоляция GPIO.
                const DOMAIN_ISOLATION: u32 = 0x155 << 22;
                let overlap = |word: u32| {
                    word & (word >> 1) & DOMAIN_ISOLATION != 0
                        || word & (word << 1) & (DIG_PAD_ISO_OR_HOLD | (1 << 8)) != 0
                };
                if value.is_some_and(overlap) {
                    return Err(unsupported(
                        "конфликт принудительной изоляции или удержания DIG_ISO",
                    ));
                }
                rw(&mut self.dig_iso, DIG_ISO_RW, value)
            }
            RTC_PAD_HOLD => rw(&mut self.rtc_pad_hold, RTC_PAD_MASK, value),
            DIG_PAD_HOLD => rw(&mut self.dig_pad_hold, DIG_PAD_MASK, value),
            WDT_CONFIG0..=WDT_WPROTECT => self.watchdog_register(offset, value)?,
            SWD_CONF => {
                if value.is_some() && self.swd_key != SWD_KEY {
                    return Ok(Some(0));
                }
                if value.is_some_and(|word| word & SWD_FEED_CMD != 0) {
                    return Err(unsupported("ручной feed аналогового SWD"));
                }
                rw(&mut self.swd, SWD_CONF_RW, value)
            }
            SWD_WPROTECT => rw(&mut self.swd_key, u32::MAX, value),
            FIB_SEL => rw(&mut self.fib_sel, FIB_SEL_MASK, value),
            _ => return Ok(None),
        };
        Ok(Some(result))
    }

    fn options(&mut self, value: Option<u32>) -> Result<u32, BoardFault> {
        let Some(word) = value else {
            return Ok(self.options);
        };
        if word & SW_CORE_RESET != 0 {
            self.pending_reset = Some(EspRtcReset::SoftwareCore);
        } else if word & SW_CPU_RESET != 0 {
            self.pending_reset = Some(EspRtcReset::SoftwareCpu);
        }
        Ok(rw(&mut self.options, OPTIONS_RW, value))
    }

    fn reset_state(&mut self, value: Option<u32>) -> Result<u32, BoardFault> {
        if let Some(word) = value {
            if word & RESET_STATE_UNSUPPORTED != 0 || word & STATIC_VECTOR == 0 {
                return Err(unsupported(
                    "нестандартный вектор, отладочный останов или DRESET_MASK",
                ));
            }
            if word & CLEAR_RESET_FLAG != 0 {
                self.reset_flag = false;
            }
            return Ok(0);
        }
        let reason = self
            .reason
            .ok_or_else(|| unsupported("группа MWDT не передана блоком TIMG"))?;
        // TRM регистр 9.12: бит 12 сохраняет показанное в таблице значение сброса.
        Ok(STATIC_VECTOR
            | (1 << 12)
            | (u32::from(self.reset_flag) * RESET_FLAG)
            | (u32::from(reason as u8) & RESET_CAUSE_MASK))
    }

    fn interrupt_register(&mut self, offset: u32, value: Option<u32>) -> Result<u32, BoardFault> {
        let Some(word) = value else {
            return Ok(if offset == INT_ENA {
                self.int_enable
            } else {
                0
            });
        };
        if matches!(offset, INT_ENA | INT_ENA_SET) && word & !WDT_INT != 0 {
            return Err(unsupported("источник прерывания RTC кроме RWDT"));
        }
        match offset {
            INT_ENA => self.int_enable = word,
            INT_ENA_SET => self.int_enable |= word,
            INT_ENA_CLEAR => self.int_enable &= !word,
            INT_CLR => self.int_raw &= !word,
            _ => unreachable!(),
        }
        Ok(0)
    }

    fn watchdog_register(&mut self, offset: u32, value: Option<u32>) -> Result<u32, BoardFault> {
        if offset == WDT_WPROTECT {
            return Ok(rw(&mut self.watchdog.key, u32::MAX, value));
        }
        let stage = || ((offset - WDT_CONFIG1) / u32::from(WORD)) as usize;
        let Some(word) = value else {
            return Ok(match offset {
                WDT_CONFIG0 => self.watchdog.config,
                WDT_FEED => 0,
                _ => self.watchdog.hold[stage()],
            });
        };
        // TRM §12.2.2.3: аппаратно запись под защитой игнорируется.
        if self.watchdog.key != WDT_KEY {
            return Ok(0);
        }
        let mut next = self.watchdog.clone();
        match offset {
            WDT_FEED if word & WDT_FEED_CMD != 0 => next.feed(),
            WDT_FEED => {}
            WDT_CONFIG0 => {
                // RTC_CNTL_WDT_APPCPU_RESET_EN[10] относится к отсутствующему второму CPU.
                next.config = word & !(1 << 10);
                if !next.active() || !self.watchdog.active() {
                    next.feed();
                }
                if word & FLASHBOOT_ENABLE == 0 {
                    next.flash_boot = false;
                }
            }
            WDT_CONFIG1..=WDT_CONFIG4 => next.hold[stage()] = word,
            _ => unreachable!(),
        }
        next.validate(self.wdt_delay_sel)?;
        self.watchdog = next;
        Ok(0)
    }

    fn board_access(&mut self, addr: u32, size: u8, value: Option<u32>) -> Access {
        let Some(offset) = addr.checked_sub(BASE) else {
            return Ok(None);
        };
        if offset > DATE || size != WORD || !addr.is_multiple_of(u32::from(WORD)) {
            return Ok(None);
        }
        let Some(word) = value else {
            return self.register(offset, value);
        };
        // До подключения автомата питания CPU/шины останавливаемся при
        // попытке отключить или изолировать домен, который исполняет BoardSim.
        let blocks_board = match offset {
            DIG_PWC => word & ((1 << 21) | (1 << 19) | (1 << 13)) != 0,
            DIG_ISO => {
                word & ((1 << 30) | (1 << 26) | (1 << 24) | DIG_PAD_ISO_OR_HOLD | (1 << 8)) != 0
            }
            RTC_PWC => word & PAD_FORCE_HOLD != 0,
            RTC_PAD_HOLD => word & RTC_PAD_MASK != 0,
            DIG_PAD_HOLD => word & DIG_PAD_MASK != 0,
            CLK_CONF => word & SLOW_SOURCE != 0,
            SLOW_CLK_CONF => word & SLOW_CLK_CONF_DIV != 0,
            _ => false,
        };
        if blocks_board {
            return Err(unsupported(
                "активное отключение/изоляция домена или смена RTC_SLOW_CLK",
            ));
        }
        self.register(offset, value)
    }

    fn advance_board(&mut self, cpu_cycles: u64, cpu_hz: u64) {
        // Текущий контракт Hardware всё ещё считает в фиксированных тактах CPU.
        // Целая дробь переносится между вызовами, включая снимок и CPU/Core Reset.
        let scaled = u128::from(cpu_cycles) * u128::from(RC_SLOW_HZ) + u128::from(self.cpu_phase);
        self.cpu_phase = (scaled % u128::from(cpu_hz)) as u64;
        self.advance_slow((scaled / u128::from(cpu_hz)) as u64, false);
    }
}

fn rw(register: &mut u32, mask: u32, value: Option<u32>) -> u32 {
    if let Some(word) = value {
        *register = word & mask;
        0
    } else {
        *register
    }
}

fn unsupported(detail: &str) -> BoardFault {
    BoardFault::Unsupported(format!("RTC_CNTL ESP32-C3: {detail}"))
}

impl Hardware {
    pub(super) fn rtc_access(&mut self, addr: u32, size: u8, value: Option<u32>) -> Access {
        self.rtc
            .as_mut()
            .map_or(Ok(None), |rtc| rtc.board_access(addr, size, value))
    }

    pub(super) fn tick_rtc(&mut self, passed: u64) {
        if let Some(rtc) = &mut self.rtc {
            rtc.advance_board(passed, Self::frequency(self.id));
            if let Some(reason) = rtc.pending_reset() {
                self.reset_requested = true;
                self.nvic.watchdog_reset |=
                    !matches!(reason, EspRtcReset::SoftwareCpu | EspRtcReset::SoftwareCore);
            }
        }
    }

    pub(super) fn rtc_cpu_restart(&mut self, watchdog: bool) -> bool {
        let Some(rtc) = &mut self.rtc else {
            return false;
        };
        let Some(reason) = rtc.pending_reset().filter(|reason| reason.cpu_only()) else {
            return false;
        };
        rtc.reset(reason);
        self.reset_requested = false;
        self.nvic.watchdog_reset = false;
        if watchdog {
            self.watchdog_resets = self.watchdog_resets.saturating_add(1);
            self.reset_events.push(OutputEvent {
                cycle: self.cycles,
                output: BoardOutput::Reset(ResetCause::Watchdog),
            });
        }
        true
    }

    pub(super) fn rtc_finish_restart(&mut self, new: &mut Self, watchdog: bool) {
        let Some(mut rtc) = self.rtc.take() else {
            return;
        };
        if let Some(reason) = rtc.pending_reset() {
            rtc.reset(reason);
            new.rtc = Some(rtc);
        } else if watchdog {
            // Hardware::tick получает от обеих групп TIMG один общий bool.
            // RTC сохраняется, но номер группы нельзя приписать наугад.
            rtc.reason = None;
            rtc.reset_flag = true;
            new.rtc = Some(rtc);
        }
    }
}
