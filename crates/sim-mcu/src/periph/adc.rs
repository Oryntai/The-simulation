//! ADC: 8 каналов, 12 бит, опорное 3.3 В.
//!
//! | смещение | регистр | доступ |
//! |---|---|---|
//! | 0x00 + 4·n | DATA[n] — чтение выполняет преобразование напряжения канала n | R |
//! | 0x40 | VREF_MV = 3300 | R |
//! | 0x44 | BITS = 12 | R |
//!
//! Идеальный АЦП: `code = clamp(floor(V·4096/3.3), 0, 4095)`, преобразование
//! мгновенное, без шума и нелинейности (шум при желании добавляет хост).

pub const CHANNELS: usize = 8;
pub const VREF: f64 = 3.3;
pub const BITS: u32 = 12;
pub const FULL_SCALE: u32 = 1 << BITS;

#[derive(Clone, Debug, Default)]
pub struct Adc {
    pub volts: [f64; CHANNELS],
}

/// Код идеального АЦП для напряжения `v`.
pub fn code(v: f64) -> u32 {
    if !v.is_finite() || v <= 0.0 {
        return 0;
    }
    ((v * FULL_SCALE as f64 / VREF).floor() as u64).min((FULL_SCALE - 1) as u64) as u32
}

impl Adc {
    pub fn read(&self, reg: u32) -> Option<u32> {
        match reg {
            0x00..=0x1c if reg.is_multiple_of(4) => Some(code(self.volts[(reg / 4) as usize])),
            0x40 => Some((VREF * 1000.0) as u32),
            0x44 => Some(BITS),
            _ => None,
        }
    }
}
