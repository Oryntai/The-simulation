//! Входы моста и приёмник импульсной команды.

use sim_signal::{ChannelId, Time};

/// Значения канала `<усилие>-bridge`. При отсутствии команды выбран выбег.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum BridgeCommand {
    Coast = 0,
    Drive = 1,
    Short = 2,
}

impl BridgeCommand {
    pub const fn value(self) -> f64 {
        self as u8 as f64
    }
}

impl TryFrom<f64> for BridgeCommand {
    type Error = String;

    fn try_from(value: f64) -> Result<Self, Self::Error> {
        match value {
            0.0 => Ok(Self::Coast),
            1.0 => Ok(Self::Drive),
            2.0 => Ok(Self::Short),
            _ => Err("режим моста: требуется 0 (выбег), 1 (тяга) или 2 (замыкание)".into()),
        }
    }
}

/// Три входа одного кадра. Передающая сторона записывает их одним Command
/// после измерения полного импульса, включая повторяющиеся ширины и периоды.
#[derive(Clone, Copy, Debug)]
pub struct PulseInputs {
    pub period_s: ChannelId,
    pub width_s: ChannelId,
    /// Целый, строго возрастающий номер кадра, точно представимый в f64.
    pub sequence: ChannelId,
}

#[derive(Clone, Copy)]
pub(crate) struct PulseCalibration {
    pub period: [f64; 2],
    pub width: [f64; 2],
    pub position: [f64; 2],
    pub timeout: Time,
}

#[derive(Clone)]
pub(crate) struct PulseReceiver {
    calibration: PulseCalibration,
    last: Option<(f64, Time)>,
    target: Option<f64>,
}

impl PulseReceiver {
    pub(crate) fn new(calibration: PulseCalibration) -> Self {
        Self {
            calibration,
            last: None,
            target: None,
        }
    }

    /// Выход за калиброванный диапазон сбрасывает команду сразу, обрыв
    /// сбрасывает её после таймаута. Флаг указывает на сброс очереди команд.
    pub(crate) fn sample(
        &mut self,
        now: Time,
        period: Option<f64>,
        width: Option<f64>,
        sequence: Option<f64>,
    ) -> Result<(Option<f64>, bool), String> {
        let calibration = self.calibration;
        let expired = self
            .last
            .is_some_and(|(_, at)| now.0 - at.0 > calibration.timeout.0);
        if expired {
            self.target = None;
        }
        if let Some(sequence) = sequence {
            // В двоичном f64 соседние целые различаются до 2^53.
            let limit = (1_u64 << f64::MANTISSA_DIGITS) as f64;
            if !(0.0..limit).contains(&sequence) || sequence.fract() != 0.0 {
                return Err("номер импульса должен быть целым в диапазоне [0, 2^53)".into());
            }
            let (Some(period), Some(width)) = (period, width) else {
                return Err("номер импульса требует период и ширину в том же кадре".into());
            };
            if !period.is_finite() || !width.is_finite() || width <= 0.0 || width > period {
                return Err("требуется конечный импульс с 0 < ширина ≤ период".into());
            }
            if self.last.is_some_and(|(old, _)| sequence < old) {
                return Err("номер импульса уменьшился".into());
            }
            if self.last.is_none_or(|(old, _)| sequence > old) {
                self.last = Some((sequence, now));
                let [p_min, p_max] = calibration.period;
                let [w_min, w_max] = calibration.width;
                if !(p_min..=p_max).contains(&period) || !(w_min..=w_max).contains(&width) {
                    self.target = None;
                    return Ok((None, true));
                }
                let [a, b] = calibration.position;
                let alpha = (width - w_min) / (w_max - w_min);
                self.target = Some(if width == w_max {
                    b
                } else {
                    alpha.mul_add(b - a, a)
                });
            }
        }
        Ok((self.target, expired))
    }
}
