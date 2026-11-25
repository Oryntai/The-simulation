use sim_process::{
    SourcedRange,
    schematic::{Parameter, ResetProfile, Scope, ns_bound},
};
use sim_signal::Time;
use sim_world::Provenance;

pub(super) fn exact(range: &SourcedRange, label: &str, scope: Scope) -> Result<f64, String> {
    range.validate().map_err(|e| e.to_string())?;
    let sourced = range.can_prove()
        || scope == Scope::Reference && matches!(range.provenance, Provenance::Derived(_));
    if !sourced || range.min != range.max {
        return Err(format!("{label}: нужен подтверждённый точечный параметр"));
    }
    Ok(range.nominal)
}

pub(super) fn parameter(value: &Parameter, scope: Scope) -> Result<f64, String> {
    exact(
        value
            .range
            .as_ref()
            .ok_or_else(|| value.measurement.clone())?,
        &value.measurement,
        scope,
    )
}

pub(super) fn duration(value: &Parameter, scope: Scope) -> Result<u64, String> {
    let value = parameter(value, scope)?;
    let lower = ns_bound(value, false).map_err(|e| e.to_string())?;
    let upper = ns_bound(value, true).map_err(|e| e.to_string())?;
    if lower != upper {
        return Err("выдержка не представима целыми наносекундами".into());
    }
    Ok(lower)
}

pub(super) fn after(at: Time, ns: u64) -> Result<Time, String> {
    at.0.checked_add(ns)
        .map(Time)
        .ok_or_else(|| "электрическое событие вне шкалы времени".into())
}

/// Детектор живого номинального прогона. Метка означает реакцию на сетке связи.
/// Непрерывную огибающую между отсчётами проверяет отдельная модель PinRail.
#[derive(Clone)]
pub(super) struct Rail {
    falling: f64,
    rising: f64,
    assert_ns: u64,
    release_ns: u64,
    pending: Option<Time>,
    reset: bool,
    boot: Time,
}

impl Rail {
    pub fn new(profile: &ResetProfile, reset: bool, scope: Scope) -> Result<Self, String> {
        profile.validate().map_err(|e| e.to_string())?;
        Ok(Self {
            falling: parameter(&profile.falling_v, scope)?,
            rising: parameter(&profile.rising_v, scope)?,
            assert_ns: duration(&profile.assert_delay_s, scope)?,
            release_ns: duration(&profile.release_delay_s, scope)?,
            pending: None,
            reset,
            boot: Time::ZERO,
        })
    }

    pub fn observe(&mut self, at: Time, voltage: f64) -> Result<Option<bool>, String> {
        if !voltage.is_finite() {
            return Err("неконечное напряжение питания платы".into());
        }
        let (qualifies, delay) = if self.reset {
            (voltage > self.rising, self.release_ns)
        } else if voltage <= 0.0 {
            // При нулевом или обратном питании цифровые буферы не работают.
            // Выдержка детектора относится к питаемой плате.
            self.pending = None;
            (true, 0)
        } else {
            (voltage < self.falling, self.assert_ns)
        };
        if !qualifies {
            self.pending = None;
            return Ok(None);
        }
        let deadline = match self.pending {
            Some(deadline) => deadline,
            None => after(at, delay)?,
        };
        if at < deadline {
            self.pending = Some(deadline);
            return Ok(None);
        }
        self.pending = None;
        self.reset = !self.reset;
        if !self.reset {
            self.boot = at;
        }
        Ok(Some(self.reset))
    }

    pub fn deadline(&self) -> Option<Time> {
        self.pending
    }
    pub fn reset(&self) -> bool {
        self.reset
    }
    pub fn boot(&self) -> Time {
        self.boot
    }

    pub fn restart(&mut self, at: Time) {
        if !self.reset {
            self.boot = self.boot.max(at);
        }
    }
}
