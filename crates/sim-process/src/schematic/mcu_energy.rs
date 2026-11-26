//! Расход заряда МК и проверка автономности на одном профиле состояний.

use std::collections::BTreeMap;

use sim_signal::Time;
use sim_world::Provenance;

use crate::{Analysis, Interval, ProcessError};

use super::sleep_drain::{SleepDrainEvidence, SleepDrainInput, assess_sleep_drain};
use super::{Finding, Scope, identifier, invalid, open, supports_source, validate_source};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum McuCurrentKind {
    Typical,
    /// Измеренный пик в условиях строки. Гарантированная верхняя граница не установлена.
    Peak,
}

#[derive(Clone, Debug, PartialEq)]
pub struct McuCurrent {
    pub current_a: f64,
    pub kind: McuCurrentKind,
    pub source: Provenance,
    pub conditions: String,
}

/// Каталожные точки тока на выводах самого МК. Ключ адресует точную
/// конфигурацию, включая частоту и параметры периферии или радио.
#[derive(Clone, Debug, PartialEq)]
pub struct McuPowerProfile {
    board: String,
    states: BTreeMap<String, McuCurrent>,
}

impl McuPowerProfile {
    pub fn new(
        board: String,
        states: impl IntoIterator<Item = (String, McuCurrent)>,
    ) -> Result<Self, ProcessError> {
        if !identifier(&board) {
            return Err(invalid("недопустимый ID платы в профиле потребления"));
        }
        let mut entries = BTreeMap::new();
        for (id, current) in states {
            if !identifier(&id)
                || !identifier(&current.conditions)
                || !current.current_a.is_finite()
                || current.current_a < 0.0
            {
                return Err(invalid(format!("недопустимая точка тока МК «{id}»")));
            }
            validate_source(&current.source)?;
            if entries.insert(id.clone(), current).is_some() {
                return Err(invalid(format!("повторное состояние МК «{id}»")));
            }
        }
        Ok(Self {
            board,
            states: entries,
        })
    }

    pub fn board(&self) -> &str {
        &self.board
    }

    pub fn current(&self, state: &str) -> Analysis<&McuCurrent> {
        match self.states.get(state) {
            Some(value) if supports_source(Scope::Assembly, &value.source) => Analysis::Analyzed {
                value,
                scope: value.conditions.clone(),
            },
            _ => open(format!("нет подтверждённой точки тока МК для «{state}»")),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct McuCharge {
    pub window: Interval,
    /// Интеграл тока на выводах МК при удержании значения строки до конца состояния, Кл.
    pub charge_c: f64,
    pub includes_peak_held: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct McuEnergyReport {
    pub chip_charge: Analysis<McuCharge>,
    pub sleep_drain: Finding<SleepDrainEvidence>,
}

/// ID состояний в `input.profile` адресуют строки каталога. Вызывающий код
/// подтверждает соответствие платы и условий каждой точки. Без требования
/// по времени интегрируется один проход профиля. При переданном требовании
/// интеграл относится к его окну, включая целые циклы и остаток.
///
/// Для sleep_drain используются имеющиеся огибающие на клеммах батареи.
/// Каталожные точки в эти огибающие автоматически не переносятся. Для
/// положительного вывода о сборке источник полного профиля должен быть замером.
pub fn assess_mcu_energy(
    power: Option<&McuPowerProfile>,
    input: &SleepDrainInput<'_>,
) -> Result<McuEnergyReport, ProcessError> {
    let sleep_drain = assess_sleep_drain(input)?;
    let chip_charge = integrate(power, input)?;
    Ok(McuEnergyReport {
        chip_charge,
        sleep_drain,
    })
}

fn integrate(
    power: Option<&McuPowerProfile>,
    input: &SleepDrainInput<'_>,
) -> Result<Analysis<McuCharge>, ProcessError> {
    let Some(profile) = input.profile else {
        return Ok(open("нет профиля состояний МК"));
    };
    let period = profile
        .states
        .iter()
        .try_fold(0u64, |sum, phase| sum.checked_add(phase.duration.0))
        .ok_or_else(|| invalid("длительность профиля МК вне шкалы времени"))?;
    let horizon = input
        .required
        .map_or(period, |required| required.duration.0);
    if period == 0 || (!profile.repeat && horizon > period) {
        return Ok(open("профиль состояний МК не покрывает окно расчёта"));
    }
    let Some(power) = power else {
        return Ok(open("нет каталога точек тока МК"));
    };
    let cycles = if profile.repeat { horizon / period } else { 0 };
    let mut tail = horizon - cycles * period;
    let mut charge_c = 0.0;
    let mut includes_peak_held = false;
    for phase in profile.states {
        let partial = phase.duration.0.min(tail);
        tail -= partial;
        let ns = phase.duration.0 * cycles + partial;
        if ns == 0 {
            continue;
        }
        let current = match power.current(phase.id) {
            Analysis::Analyzed { value, .. } => value,
            Analysis::NotAnalyzed { reason } => return Ok(open(reason)),
        };
        includes_peak_held |= current.kind == McuCurrentKind::Peak;
        charge_c += current.current_a * Time(ns).as_secs();
        if !charge_c.is_finite() {
            return Err(invalid("интеграл тока МК вне числового диапазона"));
        }
    }
    Ok(Analysis::Analyzed {
        value: McuCharge {
            window: Interval {
                start: Time::ZERO,
                end: Time(horizon),
            },
            charge_c,
            includes_peak_held,
        },
        scope: "расчётный заряд МК при удержании каталожных токов".into(),
    })
}
