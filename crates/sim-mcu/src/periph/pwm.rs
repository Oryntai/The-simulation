//! PWM: 4 канала, канал n — по смещению 0x10·n.
//!
//! | смещение | регистр | доступ |
//! |---|---|---|
//! | +0x0 | DUTY — 0..=65535 (65535 = 100 %), больше — насыщается | RW |
//! | +0x4 | FREQ_HZ — частота несущей, Гц (по умолчанию 20 000) | RW |
//! | +0x8 | CTRL — бит 0: включён | RW |
//!
//! Модель усреднённая: хост видит долю заполнения `DUTY/65535` включённого
//! канала (выключенный — 0); пульсации на несущей не моделируются.

pub const CHANNELS: usize = 4;
pub const DEFAULT_FREQ_HZ: u32 = 20_000;

#[derive(Clone, Copy, Debug)]
pub struct Channel {
    pub duty: u32,
    pub freq_hz: u32,
    pub enabled: bool,
}

#[derive(Clone, Debug)]
pub struct Pwm {
    pub ch: [Channel; CHANNELS],
}

impl Default for Pwm {
    fn default() -> Self {
        Pwm {
            ch: [Channel {
                duty: 0,
                freq_hz: DEFAULT_FREQ_HZ,
                enabled: false,
            }; CHANNELS],
        }
    }
}

impl Pwm {
    /// Доля заполнения 0..=1 (выключенный канал — 0).
    pub fn duty(&self, n: usize) -> f64 {
        match self.ch.get(n) {
            Some(c) if c.enabled => c.duty as f64 / 65535.0,
            _ => 0.0,
        }
    }

    pub fn read(&self, reg: u32) -> Option<u32> {
        let n = (reg >> 4) as usize;
        let c = self.ch.get(n)?;
        Some(match reg & 0xf {
            0x0 => c.duty,
            0x4 => c.freq_hz,
            0x8 => c.enabled as u32,
            _ => return None,
        })
    }

    pub fn write(&mut self, reg: u32, v: u32) -> bool {
        let n = (reg >> 4) as usize;
        let Some(c) = self.ch.get_mut(n) else {
            return false;
        };
        match reg & 0xf {
            0x0 => c.duty = v.min(65535),
            0x4 => c.freq_hz = v,
            0x8 => c.enabled = v & 1 != 0,
            _ => return false,
        }
        true
    }
}
