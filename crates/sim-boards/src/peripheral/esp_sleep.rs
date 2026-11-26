//! Сон ESP32-C3. TRM v1.4, §§9.3–9.7 и ESP-IDF v6.1.1, rtc_sleep.c.

use super::{Access, Chip, Hardware, esp_rtc::EspRtcReset};
use crate::{
    BoardFault, BoardOutput, EspResetValueKind, EspSleepMode, EspSleepStatus, EspWake, OutputEvent,
    ResetCause, esp_clock::QUARTERS_PER_SECOND, esp_memory::EspMemory,
};

// TRM рег. 9.2–9.17, 9.23–9.24, 9.29 и rtc_cntl_reg.h.
const RTC: u32 = 0x6000_8000;
const ALARM_LOW: u32 = RTC + 0x04;
const ALARM_HIGH: u32 = RTC + 0x08;
const STATE: u32 = RTC + 0x18;
const TIMER5: u32 = RTC + 0x2c;
const RESET_STATE: u32 = RTC + 0x38;
const WAKEUP_STATE: u32 = RTC + 0x3c;
const INT_ENA: u32 = RTC + 0x40;
const INT_RAW: u32 = RTC + 0x44;
const INT_ST: u32 = RTC + 0x48;
const INT_CLR: u32 = RTC + 0x4c;
const EXT_WAKEUP: u32 = RTC + 0x64;
const REJECT_CONF: u32 = RTC + 0x68;
const DIG_POWER: u32 = RTC + 0x88;
const STORE6: u32 = RTC + 0xc0;
const REJECT_CAUSE: u32 = RTC + 0xf0;
const WAKE_CAUSE: u32 = RTC + 0xf8;
const INT_SET: u32 = RTC + 0x100;
const INT_CLEAR: u32 = RTC + 0x104;
const GPIO_WAKEUP: u32 = RTC + 0x110;

const ALARM_ENABLE: u32 = 1 << 16;
const SLEEP_ENABLE: u32 = 1 << 31;
const STATE_REJECT: u32 = 1 << 30;
const STATE_WAKE: u32 = 1 << 29;
const REJECT_CAUSE_CLEAR: u32 = 1 << 1;
const MIN_SLEEP_SHIFT: u32 = 8; // RTC_CNTL_MIN_SLP_VAL[15:8], TRM рег. 9.10.
const WAKE_MASK_SHIFT: u32 = 15;
const REJECT_MASK_SHIFT: u32 = 12;
const REJECT_MASK: u32 = 0x3ffff;
const DEEP_REJECT_ENABLE: u32 = 1 << 31;
const LIGHT_REJECT_ENABLE: u32 = 1 << 30;
const DEEP_ENABLE: u32 = 1 << 31; // DG_WRAP_PD_EN, rtc_sleep_init.
const DIGITAL_FORCE_ON: u32 = 1 << 20;
const CPU_PD_ENABLE: u32 = 1 << 29;
const PERIPH_PD_ENABLE: u32 = 1 << 28;
const FASTMEM_LOSS: u32 = 1 << 15;
const LIGHT_RAM_LOSS: u32 = 1 << 3;
const GPIO_FILTER: u32 = 1 << 31;
const RTC_GPIO_CLOCK: u32 = 1 << 7;
const RTC_GPIO_CLEAR: u32 = 1 << 6;
const RTC_GPIO_MASK: u32 = (1 << RTC_GPIO_COUNT) - 1;
const INT_WAKE: u32 = 1;
const INT_REJECT: u32 = 1 << 1;
const INT_ALARM: u32 = 1 << 10;
const SLEEP_INTERRUPTS: u32 = INT_WAKE | INT_REJECT | INT_ALARM;
const RESET_CAUSE_MASK: u32 = 0x3f;
const DEEP_RESET: u32 = 0x05; // soc/esp32c3/include/soc/reset_reasons.h.

// TRM табл. 9.4-2 и ESP-IDF port/esp32c3/include/soc/rtc.h.
const GPIO_TRIGGER: u32 = 1 << 2;
const TIMER_TRIGGER: u32 = 1 << 3;
const TRIGGERS: u32 = TIMER_TRIGGER | GPIO_TRIGGER;

// TRM §9.3.3 и рег. 9.3: 48 бит счётчика, старшие 16 бит будильника.
// TRM §6.2.3, табл. 9.3-1 и даташит §4.1.3.3: номинал RC_SLOW_CLK.
const RTC_TICK_MASK: u64 = (1 << 48) - 1;
const ALARM_HIGH_MASK: u32 = u16::MAX as u32;
const SLOW_HZ: u128 = 136_000;
const PHASE_DEN: u128 = QUARTERS_PER_SECOND as u128;

// TRM рег. 9.60, gpio_ll.h и soc_caps.h C3: RTC GPIO0…5.
// ESP-IDF esp_sleep.h: для надёжной регистрации уровня нужны три периода RTC SLOW.
pub(super) const RTC_GPIO_COUNT: usize = 6;
const GPIO_COUNT: usize = 22; // TRM §5.2.
const GPIO_SAMPLES: u8 = 3;
const RTC_PIN0_ENABLE: u32 = 31;
const RTC_PIN0_TYPE: u32 = 23;
const GPIO_TYPE_WIDTH: u32 = 3;
const GPIO_TYPE_MASK: u32 = (1 << GPIO_TYPE_WIDTH) - 1;
pub(super) const GPIO_LOW: u32 = 4; // ESP-IDF gpio_types.h: GPIO_INTR_LOW_LEVEL.
pub(super) const GPIO_HIGH: u32 = 5; // GPIO_INTR_HIGH_LEVEL.

// TRM §9.6, §3.3.2 и рег. 16.11–16.12, ESP-IDF system_reg.h.
const FASTMEM_CONFIG: u32 = 0x600c_0048;
const FASTMEM_CRC: u32 = 0x600c_004c;
const CRC_START: u32 = 1 << 8;
const CRC_FINISH: u32 = 1 << 31;
const CRC_CONFIG_MASK: u32 = (0x7ff << 20) | (0x7ff << 9);
const CRC_CONFIG_RESET: u32 = 0x7ff << 20;
const CRC_RESULT_RESET: u32 = 0;
const RTC_FAST: std::ops::Range<u32> = 0x5000_0000..0x5000_2000;

// Espressif QEMU 9.2.2, esp32c3_clock_read и наблюдения qemu.trace:
// после START весь CONFIG читается как 0. Это состояние ожидания, а не результат CRC.
const CRC_PENDING_PROFILE: &str = "RTC_FASTMEM_CONFIG_CRC_PENDING";

pub(crate) fn supports_crc_profile(
    addr: u32,
    field: &str,
    value: u32,
    kind: EspResetValueKind,
) -> bool {
    addr == FASTMEM_CONFIG
        && field == CRC_PENDING_PROFILE
        && kind == EspResetValueKind::Qemu
        && value == 0
}

#[derive(Clone, Copy)]
struct Sleeping {
    mode: EspSleepMode,
    entered_at: u64,
    sources: u32,
    retain_fast: bool,
    low_pins: u32,
    high_pins: u32,
    rtc_pins: u32,
    minimum_left: u64,
    early_pulse: bool,
    samples: [u8; GPIO_COUNT],
}

impl Sleeping {
    fn sample(&mut self, active: u32, elapsed: u64) -> u32 {
        let mut triggered = 0;
        for (pin, count) in self.samples.iter_mut().enumerate() {
            let bit = 1 << pin;
            if active & bit == 0 {
                *count = 0;
            } else {
                *count = (*count + elapsed.min(u64::from(GPIO_SAMPLES)) as u8).min(GPIO_SAMPLES);
                if *count == GPIO_SAMPLES {
                    triggered |= bit;
                }
            }
        }
        triggered
    }

    fn ticks_to_pin(&self, active: u32) -> Option<u64> {
        self.samples
            .iter()
            .enumerate()
            .filter(|(pin, _)| active & (1 << pin) != 0)
            .map(|(_, count)| u64::from(GPIO_SAMPLES - count))
            .min()
    }
}

#[derive(Clone)]
pub(super) struct EspSleep {
    phase: u128,
    ticks: u64,
    alarm_value: u64,
    alarm_armed: bool,
    alarm_fired: bool,
    raw: u32,
    enabled: u32,
    reject_config: u32,
    reject_cause: u32,
    wake_cause: u32,
    ext_wakeup: u32,
    gpio_wakeup: u32,
    rtc_gpio_status: u32,
    crc_config: u32,
    deep_reset: bool,
    state: Option<Sleeping>,
    last_wake: Option<EspWake>,
}

impl Default for EspSleep {
    fn default() -> Self {
        Self {
            phase: 0,
            ticks: 0,
            alarm_value: 0,
            alarm_armed: false,
            alarm_fired: false,
            raw: 0,
            enabled: 0,
            reject_config: 0,
            reject_cause: 0,
            wake_cause: 0,
            ext_wakeup: 0,
            gpio_wakeup: 0,
            rtc_gpio_status: 0,
            crc_config: CRC_CONFIG_RESET,
            deep_reset: false,
            state: None,
            last_wake: None,
        }
    }
}

impl EspSleep {
    fn status(&self) -> EspSleepStatus {
        EspSleepStatus {
            // TRM §9.4.2, табл. 9.4-1: после сброса работает CPU, RF выключен.
            mode: self.state.map_or(EspSleepMode::Modem, |s| s.mode),
            entered_at: self.state.map(|s| s.entered_at),
            rtc_slow_ticks: self.ticks,
            last_wake: self.last_wake,
        }
    }

    fn alarm_in(&self) -> Option<u64> {
        self.alarm_armed
            .then(|| self.alarm_value.saturating_sub(self.ticks).max(1))
    }

    fn advance_clock(&mut self, cycles: u64, period: u32, elapsed: u64) {
        self.phase = self.scaled(cycles, period) % PHASE_DEN;
        if self
            .alarm_in()
            .is_some_and(|remaining| remaining <= elapsed)
        {
            self.alarm_armed = false;
            self.alarm_fired = true;
            self.raw |= INT_ALARM;
        }
        self.ticks = self.ticks.wrapping_add(elapsed) & RTC_TICK_MASK;
    }

    fn scaled(&self, cycles: u64, period: u32) -> u128 {
        self.phase + u128::from(cycles) * u128::from(period) * SLOW_HZ
    }

    fn cycles_until(&self, ticks: u64, period: u32) -> u64 {
        let target = u128::from(ticks) * PHASE_DEN;
        let cycles = target
            .saturating_sub(self.phase)
            .div_ceil(u128::from(period) * SLOW_HZ);
        u64::try_from(cycles).unwrap_or(u64::MAX)
    }

    // GPIO_WAKEUP: номера выводов идут от старшего бита вниз, статус — от младшего.
    fn rtc_pin_level(&self, pin: usize) -> Result<Option<bool>, BoardFault> {
        let pin = pin as u32;
        if self.gpio_wakeup & (1 << (RTC_PIN0_ENABLE - pin)) == 0 {
            return Ok(None);
        }
        if self.gpio_wakeup & RTC_GPIO_CLOCK == 0 || self.ext_wakeup & GPIO_FILTER == 0 {
            return Err(unsupported(
                "RTC GPIO требует тактирования и включённого фильтра",
            ));
        }
        match self.gpio_wakeup >> (RTC_PIN0_TYPE - pin * GPIO_TYPE_WIDTH) & GPIO_TYPE_MASK {
            GPIO_LOW => Ok(Some(false)),
            GPIO_HIGH => Ok(Some(true)),
            _ => Err(unsupported(
                "режим RTC GPIO, отличный от пробуждения по уровню",
            )),
        }
    }
}

impl Hardware {
    pub(crate) fn esp_sleep_status(&self) -> Option<EspSleepStatus> {
        self.esp_sleep.as_deref().map(EspSleep::status)
    }

    pub(crate) fn sleeping_mode(&self) -> Option<EspSleepMode> {
        self.esp_sleep.as_ref()?.state.map(|s| s.mode)
    }

    pub(super) fn sleep_pin_changed(&mut self, pin: u32) {
        if let (Some(sleep), Chip::Esp(chip)) = (
            self.esp_sleep.as_mut().and_then(|s| s.state.as_mut()),
            &self.chip,
        ) {
            let bit = 1 << pin;
            if chip.active_wake_pins(sleep.low_pins & bit, sleep.high_pins & bit, sleep.mode)
                != Ok(bit)
            {
                let samples = &mut sleep.samples[pin as usize];
                sleep.early_pulse |= *samples == GPIO_SAMPLES && sleep.minimum_left != 0;
                *samples = 0;
            }
        }
    }

    pub(super) fn sleep_interrupt(&self) -> bool {
        self.esp_sleep
            .as_ref()
            .is_some_and(|s| s.raw & s.enabled != 0)
    }

    pub(super) fn sleep_after_reset(&mut self, next: &mut Self) {
        let Some(mut sleep) = self.esp_sleep.take() else {
            return;
        };
        // Снимок RTC сохраняется у аппаратных сбросов, пока жив его домен.
        let reason = self.rtc.as_ref().and_then(|rtc| rtc.pending_reset());
        if reason.is_none() || reason == Some(EspRtcReset::WatchdogSystem) {
            *sleep = EspSleep::default();
        }
        // TRM табл. 6.1-1: Core Reset сбрасывает цифровой SYSREG, RTC сохраняется.
        sleep.crc_config = CRC_CONFIG_RESET;
        sleep.state = None;
        sleep.deep_reset = false;
        next.esp_sleep = Some(sleep);
    }

    pub(super) fn sleep_after_cpu_reset(&mut self) {
        if let Some(sleep) = &mut self.esp_sleep {
            sleep.state = None;
            sleep.deep_reset = false;
        }
    }

    pub(crate) fn finish_deep_wake(&mut self) {
        // TRM §9.4.1 и §9.6: цифровой домен стартует с нуля, RTC остаётся.
        let mut rtc = self.rtc.take().expect("RTC установлен вместе с flash");
        let mut sleep = self.esp_sleep.take().expect("пробуждение из deep sleep");
        self.restart(false);
        // Поднимаем штатный флаг нового сброса. Причину Deep-sleep публикует
        // RTC_CNTL_RESET_STATE через sleep_access до следующего сброса.
        rtc.reset(EspRtcReset::SoftwareCore);
        self.rtc = Some(rtc);
        sleep.deep_reset = true;
        sleep.crc_config = CRC_CONFIG_RESET;
        self.esp_sleep = Some(sleep);
        self.reset_events.push(OutputEvent {
            cycle: self.cycles,
            output: BoardOutput::Reset(ResetCause::DeepSleep),
        });
    }

    pub(super) fn sleep_access(&mut self, addr: u32, size: u8, value: Option<u32>) -> Access {
        if matches!(addr, FASTMEM_CONFIG | FASTMEM_CRC) && self.esp_sleep.is_some() {
            return self.rtc_fast_crc_access(addr, size, value);
        }
        if self.esp_sleep.is_none() || size as usize != size_of::<u32>() {
            return Ok(None);
        }
        if addr == RESET_STATE && value.is_none() {
            if self.esp_sleep.as_ref().unwrap().deep_reset {
                return Ok(self
                    .rtc_access(addr, size, None)?
                    .map(|word| (word & !RESET_CAUSE_MASK) | DEEP_RESET));
            }
            return Ok(None);
        }
        if matches!(
            addr,
            INT_ENA | INT_RAW | INT_ST | INT_CLR | INT_SET | INT_CLEAR
        ) {
            let result = self.rtc_access(addr, size, value.map(|v| v & !SLEEP_INTERRUPTS))?;
            let sleep = self.esp_sleep.as_mut().unwrap();
            if let Some(bits) = value.map(|v| v & SLEEP_INTERRUPTS) {
                match addr {
                    INT_ENA => sleep.enabled = bits,
                    INT_CLR => sleep.raw &= !bits,
                    INT_SET => sleep.enabled |= bits,
                    INT_CLEAR => sleep.enabled &= !bits,
                    _ => {}
                }
            }
            let bits = match addr {
                INT_ENA => sleep.enabled,
                INT_RAW => sleep.raw,
                INT_ST => sleep.raw & sleep.enabled,
                _ => 0,
            };
            return Ok(result.map(|word| word | if value.is_none() { bits } else { 0 }));
        }
        if addr == STATE && value.is_some_and(|v| v & SLEEP_ENABLE != 0) {
            return self.start_sleep(value.unwrap()).map(|()| Some(0));
        }
        let sleep = self.esp_sleep.as_mut().unwrap();
        match (addr, value) {
            (ALARM_LOW, None) => Ok(Some(sleep.alarm_value as u32)),
            (ALARM_HIGH, None) => Ok(Some((sleep.alarm_value >> u32::BITS) as u32)),
            (ALARM_LOW, Some(low)) => {
                sleep.alarm_value = (sleep.alarm_value & !(u32::MAX as u64)) | u64::from(low);
                Ok(Some(0))
            }
            (ALARM_HIGH, Some(high)) => {
                sleep.alarm_value = u64::from(sleep.alarm_value as u32)
                    | (u64::from(high & ALARM_HIGH_MASK) << u32::BITS);
                if high & ALARM_ENABLE != 0 {
                    sleep.alarm_armed = true;
                    sleep.alarm_fired = false;
                }
                Ok(Some(0))
            }
            (STATE, None) => Ok(Some(
                u32::from(sleep.raw & INT_WAKE != 0) * STATE_WAKE
                    | u32::from(sleep.raw & INT_REJECT != 0) * STATE_REJECT,
            )),
            (STATE, Some(bits)) => {
                clear_reject(sleep, bits)?;
                Ok(Some(0))
            }
            (REJECT_CONF, None) => Ok(Some(sleep.reject_config)),
            (REJECT_CONF, Some(bits)) => {
                sleep.reject_config = bits
                    & (DEEP_REJECT_ENABLE
                        | LIGHT_REJECT_ENABLE
                        | (REJECT_MASK << REJECT_MASK_SHIFT));
                Ok(Some(0))
            }
            (REJECT_CAUSE, None) => Ok(Some(sleep.reject_cause)),
            (WAKE_CAUSE, None) => Ok(Some(sleep.wake_cause)),
            (EXT_WAKEUP, None) => Ok(Some(sleep.ext_wakeup)),
            (EXT_WAKEUP, Some(bits)) => {
                sleep.ext_wakeup = bits & GPIO_FILTER;
                Ok(Some(0))
            }
            (GPIO_WAKEUP, None) => Ok(Some(sleep.gpio_wakeup | sleep.rtc_gpio_status)),
            (GPIO_WAKEUP, Some(bits)) => {
                if bits & RTC_GPIO_CLEAR != 0 {
                    sleep.rtc_gpio_status = 0;
                }
                sleep.gpio_wakeup = bits & !(RTC_GPIO_MASK | RTC_GPIO_CLEAR);
                Ok(Some(0))
            }
            _ => Ok(None),
        }
    }

    fn rtc_fast_crc_access(&mut self, addr: u32, size: u8, value: Option<u32>) -> Access {
        let boot = self
            .esp_boot
            .as_mut()
            .expect("CRC доступен вместе с raw flash");
        let sleep = self.esp_sleep.as_mut().unwrap();
        let pending = sleep.crc_config & CRC_START != 0;
        if size as usize != size_of::<u32>()
            || (addr == FASTMEM_CONFIG
                && value
                    .is_some_and(|word| word & !(CRC_CONFIG_MASK | CRC_START | CRC_FINISH) != 0))
        {
            return Err(BoardFault::UnsupportedRegister {
                pc: boot.pc,
                addr,
                size,
                write: value.is_some(),
                block: "CRC RTC FAST ESP32-C3",
            });
        }
        match (addr, value) {
            (FASTMEM_CRC, Some(_)) => Err(BoardFault::WriteToReadOnly { pc: boot.pc, addr }),
            (FASTMEM_CRC, None) if !pending => Ok(Some(CRC_RESULT_RESET)),
            (FASTMEM_CONFIG, None) if !pending => Ok(Some(sleep.crc_config)),
            (FASTMEM_CONFIG, None) => boot
                .reset_values
                .read(addr, CRC_PENDING_PROFILE)?
                .map(Some)
                .ok_or_else(|| {
                    unsupported("CRC RTC FAST: отсутствует профиль ожидания CRC_FINISH")
                }),
            (_, _) if pending => Err(unsupported(
                "CRC RTC FAST: QEMU не выставил CRC_FINISH; результата и отмены расчёта в профиле нет",
            )),
            (FASTMEM_CONFIG, Some(bits)) => {
                if bits & CRC_START != 0 && !boot.reset_values.contains(addr, CRC_PENDING_PROFILE) {
                    return Err(unsupported(
                        "CRC RTC FAST: SYSTEM_RTC_FASTMEM_CONFIG_REG 0x600c0048; TRM рег. 16.11–16.12 не задаёт алгоритм и время расчёта",
                    ));
                }
                // FINISH доступен только для чтения (TRM рег. 16.11).
                sleep.crc_config = bits & (CRC_CONFIG_MASK | CRC_START);
                Ok(Some(0))
            }
            _ => unreachable!("адрес проверен до обращения к CRC"),
        }
    }

    fn start_sleep(&mut self, state_word: u32) -> Result<(), BoardFault> {
        validate_state_write(state_word)?;
        let word = size_of::<u32>() as u8;
        let power = self.rtc_access(DIG_POWER, word, None)?.unwrap();
        let minimum = (self.rtc_access(TIMER5, word, None)?.unwrap() >> MIN_SLEEP_SHIFT) as u8;
        let enabled_sources =
            self.rtc_access(WAKEUP_STATE, word, None)?.unwrap() >> WAKE_MASK_SHIFT;
        let deep = power & DEEP_ENABLE != 0;
        if deep && self.esp_sleep.as_ref().unwrap().crc_config & CRC_START != 0 {
            return Err(unsupported("CRC RTC FAST: вход в Deep-sleep до CRC_FINISH"));
        }
        if deep && power & DIGITAL_FORCE_ON != 0 {
            return Err(unsupported(
                "Deep-sleep при принудительно включённом цифровом домене",
            ));
        }
        if !deep && power & (CPU_PD_ENABLE | PERIPH_PD_ENABLE | LIGHT_RAM_LOSS | FASTMEM_LOSS) != 0
        {
            return Err(unsupported(
                "Light-sleep с отключением CPU, периферии или retention памяти",
            ));
        }
        if deep && self.rtc_access(STORE6, word, None)?.unwrap() != 0 {
            return Err(unsupported(
                "RTC wake stub требует аппаратного CRC RTC FAST",
            ));
        }
        let (sleep, Chip::Esp(chip)) = (self.esp_sleep.as_mut().unwrap(), &self.chip) else {
            unreachable!("сон доступен только в профиле ESP32-C3");
        };
        if chip.buses.active() {
            return Err(unsupported("вход в сон во время обмена I²C/SPI"));
        }
        let reject_enable = if deep {
            DEEP_REJECT_ENABLE
        } else {
            LIGHT_REJECT_ENABLE
        };
        let reject_sources = if sleep.reject_config & reject_enable != 0 {
            (sleep.reject_config >> REJECT_MASK_SHIFT) & REJECT_MASK
        } else {
            0
        };
        let sources = enabled_sources | reject_sources;
        if sources & !TRIGGERS != 0 {
            return Err(unsupported("источник сна кроме GPIO и таймера RTC"));
        }
        let mut state = Sleeping {
            mode: if deep {
                EspSleepMode::Deep
            } else {
                EspSleepMode::Light
            },
            entered_at: self.cycles,
            sources: enabled_sources,
            retain_fast: power & FASTMEM_LOSS == 0,
            low_pins: 0,
            high_pins: 0,
            rtc_pins: 0,
            minimum_left: u64::from(minimum),
            early_pulse: false,
            samples: [0; GPIO_COUNT],
        };
        if sources & GPIO_TRIGGER != 0 {
            for pin in 0..GPIO_COUNT {
                let rtc = if pin < RTC_GPIO_COUNT {
                    sleep.rtc_pin_level(pin)?
                } else {
                    None
                };
                let digital = if deep {
                    None
                } else {
                    chip.digital_wakeup_level(pin)
                };
                if rtc.is_some() {
                    state.rtc_pins |= 1 << pin;
                }
                if rtc.is_some() && digital.is_some() && rtc != digital {
                    return Err(unsupported(
                        "противоположные уровни пробуждения на одном GPIO",
                    ));
                }
                match rtc.or(digital) {
                    Some(true) => state.high_pins |= 1 << pin,
                    Some(false) => state.low_pins |= 1 << pin,
                    None => {}
                }
            }
            if state.low_pins | state.high_pins == 0 {
                return Err(unsupported(
                    "GPIO выбран источником пробуждения без настроенного вывода",
                ));
            }
        }
        let mut reject = 0;
        if reject_sources & TIMER_TRIGGER != 0
            && (sleep.alarm_fired || sleep.alarm_armed && sleep.alarm_value <= sleep.ticks)
        {
            reject |= TIMER_TRIGGER;
        }
        if reject_sources & GPIO_TRIGGER != 0
            && chip.active_wake_pins(state.low_pins, state.high_pins, state.mode)? != 0
        {
            reject |= GPIO_TRIGGER;
        }
        clear_reject(sleep, state_word)?;
        if reject != 0 {
            sleep.reject_cause = reject;
            sleep.raw |= INT_REJECT;
        } else {
            sleep.state = Some(state);
        }
        Ok(())
    }

    pub(crate) fn sleep_commit_entry(&mut self) {
        if let Some(s) = self.esp_sleep.as_mut().and_then(|s| s.state.as_mut()) {
            s.entered_at = self.cycles;
        }
    }

    pub(super) fn advance_awake_rtc(&mut self, cycles: u64, period: u32) {
        let sleep = self
            .esp_sleep
            .as_mut()
            .expect("RTC установлен вместе с flash");
        let rtc = self.rtc.as_mut().expect("RTC установлен вместе с flash");
        let elapsed = rtc.advance_slow((sleep.scaled(cycles, period) / PHASE_DEN) as u64, false);
        sleep.advance_clock(cycles, period, elapsed);
        self.check_rtc_reset();
    }

    /// Продвигает плату до первого источника RTC или конца заданного интервала.
    /// Some(retain_fast) запрашивает холодный старт после Deep-sleep.
    pub(crate) fn advance_sleep(
        &mut self,
        cycles: u64,
        period: u32,
    ) -> Result<(u64, Option<bool>), BoardFault> {
        let sleep = self
            .esp_sleep
            .as_mut()
            .expect("RTC установлен вместе с flash");
        let mut state = sleep.state.expect("плата спит");
        if state.early_pulse {
            return Err(unsupported("GPIO отпущен до минимального времени сна RTC"));
        }
        let Chip::Esp(chip) = &self.chip else {
            unreachable!()
        };
        let gpio = if state.sources & GPIO_TRIGGER != 0 {
            chip.active_wake_pins(state.low_pins, state.high_pins, state.mode)?
        } else {
            0
        };
        let timer = (state.sources & TIMER_TRIGGER != 0)
            .then(|| sleep.alarm_in().or_else(|| sleep.alarm_fired.then_some(1)))
            .flatten();
        let limit = timer
            .into_iter()
            .chain(state.ticks_to_pin(gpio))
            .min()
            .map(|ticks| ticks.max(state.minimum_left));
        let mut spend = limit.map_or(cycles, |ticks| {
            cycles.min(sleep.cycles_until(ticks, period))
        });
        let planned = (sleep.scaled(spend, period) / PHASE_DEN) as u64;
        let rtc = self.rtc.as_mut().expect("RTC установлен вместе с flash");
        let elapsed = rtc.advance_slow(planned, true);
        if rtc.pending_reset().is_some() {
            spend = sleep.cycles_until(elapsed, period);
        }
        sleep.advance_clock(spend, period, elapsed);
        self.cycles += spend;
        if rtc.pending_reset().is_some() {
            self.check_rtc_reset();
            if state.mode == EspSleepMode::Deep {
                return Err(unsupported("сброс RTC WDT во время Deep-sleep"));
            }
            return Ok((spend, None));
        }

        let gpio = state.sample(gpio, elapsed);
        let timer = timer.is_some_and(|remaining| elapsed >= remaining);
        state.minimum_left = state.minimum_left.saturating_sub(elapsed);
        if state.minimum_left != 0 || !timer && gpio == 0 {
            sleep.state = Some(state);
            return Ok((spend, None));
        }
        sleep.last_wake = Some(EspWake {
            mode: state.mode,
            cycle: self.cycles,
            rtc_slow_tick: sleep.ticks,
            timer,
            gpio_mask: gpio,
        });
        sleep.wake_cause = u32::from(timer) * TIMER_TRIGGER | u32::from(gpio != 0) * GPIO_TRIGGER;
        sleep.rtc_gpio_status = gpio & state.rtc_pins;
        sleep.raw |= INT_WAKE;
        sleep.state = None;
        if timer {
            sleep.alarm_fired = false;
        }
        Ok((
            spend,
            (state.mode == EspSleepMode::Deep).then_some(state.retain_fast),
        ))
    }

    fn check_rtc_reset(&mut self) {
        if let Some(reason) = self.rtc.as_ref().and_then(|rtc| rtc.pending_reset()) {
            self.reset_requested = true;
            self.nvic.watchdog_reset |=
                !matches!(reason, EspRtcReset::SoftwareCpu | EspRtcReset::SoftwareCore);
        }
    }
}

fn validate_state_write(word: u32) -> Result<(), BoardFault> {
    if word & !(SLEEP_ENABLE | STATE_REJECT | STATE_WAKE | REJECT_CAUSE_CLEAR) != 0 {
        return Err(unsupported("неподдержанные команды RTC_CNTL_STATE0"));
    }
    Ok(())
}

fn clear_reject(sleep: &mut EspSleep, word: u32) -> Result<(), BoardFault> {
    validate_state_write(word)?;
    if word & REJECT_CAUSE_CLEAR != 0 {
        sleep.reject_cause = 0;
    }
    Ok(())
}

pub(crate) fn retain_rtc_fast(old: &mut EspMemory, new: &mut EspMemory) -> Result<(), BoardFault> {
    for addr in RTC_FAST {
        // Сохраняем и частично инициализированные слова. Неопределённый байт
        // в старом банке остаётся неопределённым в новом.
        match old.read(addr, addr, 1) {
            Ok(byte) => new.write(addr, addr, 1, byte)?,
            Err(BoardFault::Unsupported(_)) => {}
            Err(error) => return Err(error),
        }
    }
    Ok(())
}

fn unsupported(reason: &str) -> BoardFault {
    BoardFault::Unsupported(format!("сон ESP32-C3: {reason}"))
}
