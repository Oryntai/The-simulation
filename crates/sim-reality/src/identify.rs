mod camera;
mod linear;

use std::collections::{BTreeMap, BTreeSet};

use nalgebra::{DMatrix, DVector};
use serde::{Deserialize, Serialize};
use sim_signal::Time;

use crate::{BuildId, Capture, Origin};

use linear::{Fit, solve};

/// Квантиль N(0,1) на 0,975 для индивидуального двустороннего интервала 95%.
const Z_95: f64 = 1.959_963_984_540_054;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Parameter {
    Coulomb,
    Viscous,
    Inertia,
    Backlash,
    Stiffness,
    Delay,
    Fx,
    Fy,
    Cx,
    Cy,
    K1,
    K2,
    K3,
    P1,
    P2,
}

const DYNAMICS: [Parameter; 3] = [Parameter::Coulomb, Parameter::Viscous, Parameter::Inertia];
const COUPLING: [Parameter; 2] = [Parameter::Backlash, Parameter::Stiffness];
const TIMING: [Parameter; 1] = [Parameter::Delay];
const OPTICS: [Parameter; 9] = [
    Parameter::Fx,
    Parameter::Fy,
    Parameter::Cx,
    Parameter::Cy,
    Parameter::K1,
    Parameter::K2,
    Parameter::K3,
    Parameter::P1,
    Parameter::P2,
];

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Axis {
    Angular,
    Linear,
}

impl Axis {
    fn units(self) -> [&'static str; 6] {
        // Положение, скорость, ускорение, усилие, вязкое трение, инерция.
        match self {
            Self::Angular => ["rad", "rad/s", "rad/s^2", "N*m", "N*m*s/rad", "kg*m^2"],
            Self::Linear => ["m", "m/s", "m/s^2", "N", "N*s/m", "kg"],
        }
    }

    fn stiffness_unit(self) -> &'static str {
        match self {
            Self::Angular => "N*m/rad",
            Self::Linear => "N/m",
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum ExperimentKind {
    Dynamics {
        axis: Axis,
        acceleration: String,
        velocity: String,
        /// Усилие после вычитания независимо измеренных внешних нагрузок.
        effort: String,
        sigma_effort: f64,
        /// Полоса исключения страгивания по калибровке стенда.
        min_abs_velocity: f64,
    },
    Coupling {
        axis: Axis,
        relative_position: String,
        effort: String,
        sigma_position: f64,
        /// Гарантирует нагруженную сторону. Берётся из калибровки стенда.
        min_abs_effort: f64,
    },
    Delay {
        sent: String,
        received: String,
        /// СКО разности двух меток в наносекундах, без систематической ошибки.
        sigma_ns: f64,
        /// Абсолютная граница ошибки синхронизации часов, наносекунды.
        clock_offset_bound_ns: u64,
    },
    Camera {
        x: String,
        y: String,
        z: String,
        u: String,
        v: String,
        width: u32,
        height: u32,
        sigma_px: f64,
        max_iterations: usize,
    },
}

impl ExperimentKind {
    fn parameters(&self) -> &'static [Parameter] {
        match self {
            Self::Dynamics { .. } => &DYNAMICS,
            Self::Coupling { .. } => &COUPLING,
            Self::Delay { .. } => &TIMING,
            Self::Camera { .. } => &OPTICS,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Criteria {
    pub max_condition: f64,
    pub max_reduced_chi_square: f64,
    /// Предельная полуширина интервала каждого параметра в его единицах.
    pub max_half_width: BTreeMap<Parameter, f64>,
}

impl Criteria {
    fn check(&self, params: &[Parameter]) -> Result<(), String> {
        if !self.max_condition.is_finite()
            || self.max_condition <= 1.0
            || !self.max_reduced_chi_square.is_finite()
            || self.max_reduced_chi_square < 1.0
            || self.max_half_width.len() != params.len()
            || params
                .iter()
                .any(|p| self.max_half_width.get(p).is_none_or(|&v| !positive(v)))
        {
            return Err("некорректный предел обусловленности, остатков или интервала".into());
        }
        Ok(())
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Experiment {
    pub model: String,
    pub input: ExperimentKind,
    pub criteria: Criteria,
}

#[derive(Clone, Debug, Serialize)]
pub struct Estimate {
    pub parameter: Parameter,
    pub unit: &'static str,
    pub outcome: Outcome,
    /// Условная чувствительность в СКО наблюдения на единицу параметра.
    pub sensitivity: Option<f64>,
    /// Множитель увеличения СКО при совместном оценивании относительно отдельного параметра.
    pub condition: Option<f64>,
    pub interval95: Option<[f64; 2]>,
}

#[derive(Clone, Debug, Serialize)]
#[serde(tag = "state", rename_all = "snake_case")]
pub enum Outcome {
    Identified { value: f64 },
    NotAnalyzed { reason: &'static str },
}

#[derive(Debug, Serialize)]
pub struct Identification {
    build: BuildId,
    origin: Origin,
    fingerprint: u64,
    model: String,
    samples: usize,
    excluded: usize,
    reduced_chi_square: Option<f64>,
    estimates: Vec<Estimate>,
}

impl Identification {
    pub fn build(&self) -> &BuildId {
        &self.build
    }
    pub fn origin(&self) -> Origin {
        self.origin
    }
    pub fn fingerprint(&self) -> u64 {
        self.fingerprint
    }
    pub fn model(&self) -> &str {
        &self.model
    }
    pub fn estimates(&self) -> &[Estimate] {
        &self.estimates
    }

    pub(crate) fn excluded_samples(&self) -> usize {
        self.excluded
    }
}

#[derive(Clone, Copy)]
enum Domain {
    Any,
    Nonnegative,
    Positive,
}

#[derive(Clone, Copy)]
enum Transform {
    Identity(Domain),
    Twice,
    Reciprocal,
    Seconds(u64),
}

struct Coefficient {
    value: f64,
    sigma: f64,
    sensitivity: f64,
    condition: f64,
}

impl Estimate {
    fn missing(parameter: Parameter, unit: &'static str, reason: &'static str) -> Self {
        Self {
            parameter,
            unit,
            outcome: Outcome::NotAnalyzed { reason },
            sensitivity: None,
            condition: None,
            interval95: None,
        }
    }

    fn resolved(
        parameter: Parameter,
        unit: &'static str,
        fit: &Fit,
        index: usize,
        transform: Transform,
        criteria: &Criteria,
    ) -> Result<Self, String> {
        let mut report = Self::missing(parameter, unit, fit.problem);
        let Some(c) = fit.coefficients.get(index).and_then(Option::as_ref) else {
            return Ok(report);
        };
        let radius = Z_95 * c.sigma;
        let bounds = [c.value - radius, c.value + radius];
        let (value, interval, derivative, domain) = match transform {
            Transform::Identity(domain) => (c.value, bounds, 1.0, domain),
            Transform::Twice => (
                2.0 * c.value,
                bounds.map(|v| 2.0 * v),
                2.0,
                Domain::Nonnegative,
            ),
            Transform::Seconds(clock) => {
                // Перевод наносекунд в секунды и сложение с предельной ошибкой часов.
                let clock = Time(clock).as_secs();
                (
                    c.value * 1e-9,
                    [bounds[0] * 1e-9 - clock, bounds[1] * 1e-9 + clock],
                    1e-9,
                    Domain::Nonnegative,
                )
            }
            Transform::Reciprocal if bounds[0] > 0.0 => {
                let k = c.value.recip();
                (
                    k,
                    [bounds[1].recip(), bounds[0].recip()],
                    k * k,
                    Domain::Positive,
                )
            }
            Transform::Reciprocal => {
                report.condition = Some(c.condition);
                report.outcome = Outcome::NotAnalyzed {
                    reason: "интервал податливости включает ноль или отрицательные значения",
                };
                return Ok(report);
            }
        };
        let sensitivity = c.sensitivity / derivative;
        let half_width = (value - interval[0]).max(interval[1] - value);
        if ![value, interval[0], interval[1], sensitivity, half_width]
            .iter()
            .all(|x| x.is_finite())
        {
            return Err("оценка или доверительный интервал вышли за диапазон f64".into());
        }
        report.sensitivity = Some(sensitivity);
        report.condition = Some(c.condition);
        report.interval95 = Some(interval);
        report.outcome = if fit
            .reduced_chi_square
            .is_some_and(|x| x > criteria.max_reduced_chi_square)
        {
            Outcome::NotAnalyzed {
                reason: "остатки превышают допустимые для модели опыта",
            }
        } else if sensitivity <= 0.0 || c.condition > criteria.max_condition {
            Outcome::NotAnalyzed {
                reason: "недостаточная условная чувствительность",
            }
        } else if matches!(domain, Domain::Positive) && interval[0] <= 0.0
            || matches!(domain, Domain::Nonnegative) && value < 0.0
        {
            Outcome::NotAnalyzed {
                reason: "оценка не подтверждает физический диапазон",
            }
        } else if half_width > criteria.max_half_width[&parameter] {
            Outcome::NotAnalyzed {
                reason: "доверительный интервал шире допуска",
            }
        } else {
            Outcome::Identified { value }
        };
        Ok(report)
    }
}

fn positive(x: f64) -> bool {
    x.is_finite() && x > 0.0
}

/// Объединяем только синхронные отсчёты. Никакого восстановления пропусков в идентификаторе.
fn synchronized<const N: usize>(
    capture: &Capture,
    channels: [(&str, &str); N],
) -> Result<(Vec<[f64; N]>, usize), String> {
    let mut distinct = BTreeSet::new();
    let mut series = Vec::with_capacity(N);
    for (name, unit) in channels {
        if !distinct.insert(name) {
            return Err(format!("канал «{name}» повторён в опыте"));
        }
        series.push(capture.series(name, unit)?);
    }
    let Some(first) = series.first() else {
        return Ok((Vec::new(), 0));
    };
    if series
        .iter()
        .any(|s| s.len() != first.len() || s.iter().zip(*first).any(|(a, b)| a.at != b.at))
    {
        return Err("регрессия требует одинаковых меток каналов опыта".into());
    }
    let rows: Vec<_> = (0..first.len())
        .filter(|&i| series.iter().all(|s| s[i].valid))
        .map(|i| std::array::from_fn(|j| series[j][i].value))
        .collect();
    let excluded = first.len() - rows.len();
    Ok((rows, excluded))
}

fn fit_rows<const P: usize>(
    rows: &[([f64; P], f64)],
    sigma: f64,
    criteria: &Criteria,
) -> Result<Fit, String> {
    let design = DMatrix::from_fn(rows.len(), P, |i, j| rows[i].0[j] / sigma);
    let response = DVector::from_iterator(rows.len(), rows.iter().map(|r| r.1 / sigma));
    solve(design, response, criteria.max_condition)
}

type FitResult = (Fit, usize, usize);

fn marker_delay(
    capture: &Capture,
    sent: &str,
    received: &str,
    sigma: f64,
    criteria: &Criteria,
) -> Result<FitResult, String> {
    if !positive(sigma) || sent == received {
        return Err("задержка требует двух отдельных каналов и положительного СКО".into());
    }
    let sent = capture.series(sent, "1")?;
    let received = capture.series(received, "1")?;
    // Целые идентификаторы до 2^53 включительно отличимы в f64.
    const MAX_ID: f64 = (1_u64 << f64::MANTISSA_DIGITS) as f64;
    let mut rows = Vec::new();
    let mut last = None;
    for (a, b) in sent.iter().zip(received) {
        for p in [a, b].into_iter().filter(|p| p.valid) {
            if !(0.0..=MAX_ID).contains(&p.value) || p.value.fract() != 0.0 {
                return Err("некорректный целый номер маркера задержки".into());
            }
        }
        if !a.valid || !b.valid || a.value != b.value {
            continue;
        }
        if last.is_some_and(|p| a.value <= p) || b.at < a.at {
            return Err("маркер повторён, нарушен порядок или приём раньше отправления".into());
        }
        last = Some(a.value);
        rows.push(([1.0], (b.at.0 - a.at.0) as f64));
    }
    let excluded = sent.len().max(received.len()) - rows.len();
    let fit = if excluded != 0 {
        Fit::missing("нет полного соответствия меток отправки и приёма")
    } else {
        fit_rows(&rows, sigma, criteria)?
    };
    Ok((fit, rows.len(), excluded))
}

pub fn identify(capture: &Capture, experiment: &Experiment) -> Result<Identification, String> {
    use Domain::{Any, Nonnegative, Positive};
    use Transform::{Identity, Reciprocal, Seconds, Twice};

    let criteria = &experiment.criteria;
    criteria.check(experiment.input.parameters())?;
    if experiment.model.trim().is_empty() {
        return Err("пустое имя модели опыта".into());
    }
    let (fit, samples, excluded, fields) = match &experiment.input {
        ExperimentKind::Dynamics {
            axis,
            acceleration,
            velocity,
            effort,
            sigma_effort,
            min_abs_velocity,
        } => {
            if !positive(*sigma_effort) || !positive(*min_abs_velocity) {
                return Err("требуются положительные СКО усилия и граница скорости".into());
            }
            let [_, du, ddu, effort_unit, viscous_unit, inertia_unit] = axis.units();
            let (rows, missing) = synchronized(
                capture,
                [(velocity, du), (acceleration, ddu), (effort, effort_unit)],
            )?;
            // Баланс усилий с неизвестным постоянным смещением, вне зоны страгивания.
            let design = rows
                .iter()
                .filter(|[v, _, _]| v.abs() >= *min_abs_velocity)
                .map(|&[v, a, effort]| ([v.signum(), v, a, 1.0], effort))
                .collect::<Vec<_>>();
            let fit = fit_rows(&design, *sigma_effort, criteria)?;
            (
                fit,
                design.len(),
                missing + rows.len() - design.len(),
                vec![
                    (Parameter::Coulomb, effort_unit, Identity(Nonnegative)),
                    (Parameter::Viscous, viscous_unit, Identity(Nonnegative)),
                    (Parameter::Inertia, inertia_unit, Identity(Positive)),
                ],
            )
        }
        ExperimentKind::Coupling {
            axis,
            relative_position,
            effort,
            sigma_position,
            min_abs_effort,
        } => {
            if !positive(*sigma_position) || !positive(*min_abs_effort) {
                return Err("требуются положительные СКО положения и граница нагружения".into());
            }
            let [q, _, _, effort_unit, _, _] = axis.units();
            let (rows, missing) =
                synchronized(capture, [(relative_position, q), (effort, effort_unit)])?;
            // q = sign(T)·половина_люфта + T/K + смещение нуля. Полный люфт равен удвоенной оценке.
            let design = rows
                .iter()
                .filter(|[_, effort]| effort.abs() >= *min_abs_effort)
                .map(|&[position, effort]| ([effort.signum(), effort, 1.0], position))
                .collect::<Vec<_>>();
            let fit = fit_rows(&design, *sigma_position, criteria)?;
            (
                fit,
                design.len(),
                missing + rows.len() - design.len(),
                vec![
                    (Parameter::Backlash, q, Twice),
                    (Parameter::Stiffness, axis.stiffness_unit(), Reciprocal),
                ],
            )
        }
        ExperimentKind::Delay {
            sent,
            received,
            sigma_ns,
            clock_offset_bound_ns,
        } => {
            let (fit, used, missed) = marker_delay(capture, sent, received, *sigma_ns, criteria)?;
            (
                fit,
                used,
                missed,
                vec![(Parameter::Delay, "s", Seconds(*clock_offset_bound_ns))],
            )
        }
        ExperimentKind::Camera {
            x,
            y,
            z,
            u,
            v,
            width,
            height,
            sigma_px,
            max_iterations,
        } => {
            let (rows, excluded) = synchronized(
                capture,
                [(x, "m"), (y, "m"), (z, "m"), (u, "px"), (v, "px")],
            )?;
            let fit = camera::fit(
                &rows,
                [*width, *height],
                *sigma_px,
                *max_iterations,
                criteria,
            )?;
            let fields = OPTICS
                .into_iter()
                .enumerate()
                .map(|(i, p)| {
                    // Первые два коэффициента — фокусные расстояния, следующие два — главная точка.
                    let unit = if i < 4 { "px" } else { "1" };
                    (p, unit, Identity(if i < 2 { Positive } else { Any }))
                })
                .collect();
            (fit, rows.len(), excluded, fields)
        }
    };
    let estimates = fields
        .into_iter()
        .enumerate()
        .map(|(i, (p, unit, transform))| Estimate::resolved(p, unit, &fit, i, transform, criteria))
        .collect::<Result<_, _>>()?;
    Ok(Identification {
        model: experiment.model.clone(),
        build: capture.build().clone(),
        origin: capture.origin(),
        fingerprint: capture.fingerprint(),
        samples,
        excluded,
        reduced_chi_square: fit.reduced_chi_square,
        estimates,
    })
}
