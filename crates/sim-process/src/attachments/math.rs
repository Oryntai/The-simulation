use sim_world::Provenance;

use crate::{Analysis, BodyLoad, Interval, ProcessError, SourcedRange};

pub type Vector = [f64; 3];

pub fn invalid(reason: impl Into<String>) -> ProcessError {
    ProcessError::InvalidInput(reason.into())
}

pub fn unavailable<T>(reason: impl Into<String>) -> Analysis<T> {
    Analysis::NotAnalyzed {
        reason: reason.into(),
    }
}

pub fn analyzed<T>(value: T, scope: &str) -> Analysis<T> {
    Analysis::Analyzed {
        value,
        scope: scope.into(),
    }
}

pub fn finite(values: impl IntoIterator<Item = f64>) -> Result<(), ProcessError> {
    if values.into_iter().all(f64::is_finite) {
        Ok(())
    } else {
        Err(invalid(
            "неконечное значение в модели крепления или деформации",
        ))
    }
}

pub fn range(value: &SourcedRange, min: f64, max: f64) -> Result<(), ProcessError> {
    finite([value.min, value.nominal, value.max])?;
    let source = match &value.provenance {
        Provenance::Datasheet(s)
        | Provenance::Paper(s)
        | Provenance::Measured(s)
        | Provenance::Derived(s)
        | Provenance::Estimate(s) => s,
    };
    if source.trim().is_empty()
        || value.min < min
        || value.max > max
        || value.nominal < value.min
        || value.nominal > value.max
    {
        return Err(invalid("неверный диапазон или пустой источник параметра"));
    }
    Ok(())
}

pub fn positive(value: &SourcedRange) -> Result<(), ProcessError> {
    range(value, 0.0, f64::MAX)?;
    if value.min <= 0.0 {
        return Err(invalid("диапазон параметра должен быть положительным"));
    }
    Ok(())
}

pub fn check_interval(
    interval: Interval,
    previous: Option<sim_signal::Time>,
) -> Result<(), ProcessError> {
    if interval.start >= interval.end || previous.is_some_and(|time| time != interval.start) {
        return Err(invalid("пустой, повторный или непоследовательный интервал"));
    }
    Ok(())
}

pub fn add(a: Vector, b: Vector) -> Vector {
    std::array::from_fn(|i| a[i] + b[i])
}

pub fn sub(a: Vector, b: Vector) -> Vector {
    std::array::from_fn(|i| a[i] - b[i])
}

pub fn scale(a: Vector, k: f64) -> Vector {
    a.map(|v| v * k)
}

pub fn dot(a: Vector, b: Vector) -> f64 {
    a.into_iter().zip(b).map(|(x, y)| x * y).sum()
}

pub fn cross(a: Vector, b: Vector) -> Vector {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub fn norm(a: Vector) -> f64 {
    a[0].hypot(a[1]).hypot(a[2])
}

pub fn length_and_unit(a: Vector) -> Option<(f64, Vector)> {
    let length = norm(a);
    (length.is_finite() && length > 0.0).then(|| (length, a.map(|x| x / length)))
}

pub fn angle(a: Vector, b: Vector) -> f64 {
    norm(cross(a, b)).atan2(dot(a, b))
}

/// Приведение силы в точке и свободного момента к центру масс.
pub fn load(
    body: &str,
    point: Vector,
    com: Vector,
    force: Vector,
    couple: Vector,
) -> Result<BodyLoad, ProcessError> {
    let torque = add(cross(sub(point, com), force), couple);
    finite(force.into_iter().chain(torque))?;
    Ok(BodyLoad {
        body: body.into(),
        force_n: force,
        torque_nm: torque,
    })
}
