//! Проверки входа и операции в системе координат мира.

use sim_signal::Time;
use sim_world::Provenance;

use crate::{Analysis, Interval, ProcessError, SourcedRange};

pub(super) type Vec3 = [f64; 3];

pub(super) fn invalid(reason: impl Into<String>) -> ProcessError {
    ProcessError::InvalidInput(reason.into())
}

pub(super) fn execution(reason: impl Into<String>) -> ProcessError {
    ProcessError::Execution(reason.into())
}

pub(super) fn unknown<T>(reason: &str) -> Analysis<T> {
    Analysis::NotAnalyzed {
        reason: reason.into(),
    }
}

pub(super) fn calculated<T>(value: T, scope: &str) -> Analysis<T> {
    Analysis::Analyzed {
        value,
        scope: scope.into(),
    }
}

pub(super) fn finite(values: &[f64]) -> Result<(), ProcessError> {
    if values.iter().all(|x| x.is_finite()) {
        Ok(())
    } else {
        Err(invalid("нечисловое или бесконечное значение"))
    }
}

pub(super) fn computed(values: &[f64]) -> Result<(), ProcessError> {
    finite(values).map_err(|_| execution("переполнение в вычислении процесса"))
}

pub(super) fn name(value: &str) -> Result<(), ProcessError> {
    if value.trim().is_empty() {
        Err(invalid("пустое имя или описание"))
    } else {
        Ok(())
    }
}

pub(super) fn source(provenance: &Provenance) -> Result<(), ProcessError> {
    let text = match provenance {
        Provenance::Measured(s)
        | Provenance::Paper(s)
        | Provenance::Datasheet(s)
        | Provenance::Estimate(s)
        | Provenance::Derived(s) => s,
    };
    name(text)
}

/// У Derived в общем контракте есть описание, но нет проверяемой цепочки исходных данных.
pub(super) fn direct(provenance: &Provenance) -> bool {
    matches!(
        provenance,
        Provenance::Measured(_) | Provenance::Paper(_) | Provenance::Datasheet(_)
    )
}

pub(super) fn range(
    value: &SourcedRange,
    label: &str,
    allowed: impl Fn(f64) -> bool,
) -> Result<f64, ProcessError> {
    source(&value.provenance)?;
    let bounds = [value.min, value.nominal, value.max];
    finite(&bounds)?;
    if value.min <= value.nominal && value.nominal <= value.max && bounds.into_iter().all(allowed) {
        Ok(value.nominal)
    } else {
        Err(invalid(format!(
            "некорректный диапазон параметра «{label}»"
        )))
    }
}

pub(super) fn proven_point(value: &SourcedRange) -> bool {
    direct(&value.provenance) && value.min == value.max
}

pub(super) fn duration(interval: Interval) -> Result<f64, ProcessError> {
    let ns = interval
        .end
        .0
        .checked_sub(interval.start.0)
        .filter(|ns| *ns > 0)
        .ok_or_else(|| invalid("интервал процесса должен иметь положительную длительность"))?;
    Ok(Time(ns).as_secs())
}

pub(super) fn continuous(last: Option<Time>, interval: Interval) -> Result<f64, ProcessError> {
    if last.is_some_and(|end| end != interval.start) {
        return Err(invalid("пропуск или наложение интервалов модели"));
    }
    duration(interval)
}

pub(super) fn dot(a: Vec3, b: Vec3) -> f64 {
    a.into_iter().zip(b).map(|(x, y)| x * y).sum()
}

pub(super) fn norm([x, y, z]: Vec3) -> f64 {
    x.hypot(y).hypot(z)
}

pub(super) fn add(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|i| a[i] + b[i])
}

pub(super) fn sub(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|i| a[i] - b[i])
}

pub(super) fn scale(a: Vec3, factor: f64) -> Vec3 {
    a.map(|x| x * factor)
}

pub(super) fn cross([a, b, c]: Vec3, [x, y, z]: Vec3) -> Vec3 {
    [b * z - c * y, c * x - a * z, a * y - b * x]
}

pub(super) fn direction(value: Vec3) -> Result<Vec3, ProcessError> {
    finite(&value)?;
    let magnitude = norm(value);
    if magnitude == 0.0 || !magnitude.is_finite() {
        return Err(invalid("направление вырождено"));
    }
    Ok(value.map(|x| x / magnitude))
}

pub(super) fn point(values: &[SourcedRange; 3]) -> Result<Vec3, ProcessError> {
    let component = |i: usize| range(&values[i], "компонента вектора", |_| true);
    Ok([component(0)?, component(1)?, component(2)?])
}
