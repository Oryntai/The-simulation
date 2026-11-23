use std::ops::RangeInclusive;

use sim_signal::{Rng, Time};

use crate::settings::{Fault, PipeSettings};

#[derive(Clone, Debug, Default)]
pub(crate) struct Drift {
    random_walk: f64,
}

impl Drift {
    pub fn advance(
        &mut self,
        p: &PipeSettings,
        elapsed_s: f64,
        dt: f64,
        rng: &mut Rng,
    ) -> Result<f64, String> {
        // Броуновское блуждание: дисперсия приращения пропорциональна интервалу времени.
        if p.drift_rw_std != 0.0 && dt != 0.0 {
            self.random_walk += p.drift_rw_std * dt.sqrt() * rng.normal();
        }
        let drift = p.drift_per_s * elapsed_s + self.random_walk;
        if !drift.is_finite() {
            return Err("дрейф вышел за диапазон f64".into());
        }
        Ok(drift)
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum SampleMode {
    Fresh,
    Shifted,
    Stuck,
    Invalid,
}

pub(crate) fn mode(
    fault: Fault,
    acquisition: RangeInclusive<Time>,
    probability: f64,
    rng: &mut Rng,
) -> SampleMode {
    if event(probability, rng) {
        return SampleMode::Invalid;
    }
    match fault {
        Fault::Open(at) if *acquisition.end() >= at => SampleMode::Invalid,
        Fault::Stuck(at) if *acquisition.end() >= at => SampleMode::Stuck,
        Fault::Shift(at) if *acquisition.start() >= at => SampleMode::Shifted,
        _ => SampleMode::Fresh,
    }
}

pub(crate) fn event(probability: f64, rng: &mut Rng) -> bool {
    probability > 0.0 && rng.uniform() < probability
}

pub(crate) fn value(
    raw: f64,
    settings: &PipeSettings,
    drift: f64,
    shifted: bool,
    period: Option<f64>,
    rng: &mut Rng,
) -> Result<f64, String> {
    let noise = if settings.noise_std == 0.0 {
        0.0
    } else {
        settings.noise_std * rng.normal()
    };
    let shift = if shifted { settings.shift } else { 0.0 };
    let mut value = raw + settings.bias + drift + noise + shift;
    if !value.is_finite() {
        return Err("показание вышло за диапазон f64".into());
    }
    let wrap = |value: f64| {
        if let Some(period) = period {
            let phase = value.rem_euclid(period);
            // При отрицательном числе, близком к нулю, округление может дать period.
            if phase == period { 0.0 } else { phase }
        } else {
            value
        }
    };
    value = wrap(value);
    if settings.quantum != 0.0 {
        value = (value / settings.quantum).round() * settings.quantum;
        if !value.is_finite() {
            return Err("квантование вышло за диапазон f64".into());
        }
        value = wrap(value);
    }
    if let Some((min, max)) = settings.bounds {
        value = value.clamp(min, max);
    }
    Ok(value)
}
