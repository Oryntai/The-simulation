//! Временные метрики и описательная статистика кампании.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use sim_runtime::{CommandSource, Log, LogEntry, LogKind, Status};
use sim_signal::SignalSpec;

use crate::{
    Error, Fault, Fingerprint, Limit, Outcome, Signal, Source,
    campaign::{
        CampaignPlan, Disposition, OperatingMode, PlanRow, RunKey, SampledInputs, Target, Window,
    },
    invalid,
    sampling::{AppliedValue, Independence, Sample, SampleKey, Seeds, name},
};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Interpolation {
    Linear,
    Hold,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SeriesKind {
    Truth,
    Observation,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Reference {
    Constant {
        value: f64,
        unit: String,
    },
    Series {
        signal: Signal,
        series: SeriesKind,
    },
    Step {
        stimulus: String,
        before: f64,
        after: f64,
        unit: String,
    },
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Operation {
    Absolute { at_ns: u64 },
    TrackingRms,
    Settling { band: f64, deadline_ns: Option<u64> },
    Overshoot,
    OvershootFraction,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Worse {
    Higher,
    Lower,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct MetricSpec {
    pub id: String,
    pub signal: Signal,
    pub series: SeriesKind,
    pub reference: Reference,
    pub window: String,
    pub interpolation: Interpolation,
    pub operation: Operation,
    pub worse: Worse,
    pub source: Source,
    /// Максимальное значение в единице результата. Для времени — deadline_ns.
    pub limit: Option<f64>,
}

impl MetricSpec {
    pub fn unit(&self) -> &str {
        match self.operation {
            Operation::Settling { .. } => "ns",
            Operation::OvershootFraction => "1",
            _ => &self.signal.unit,
        }
    }

    pub(crate) fn is_criterion(&self) -> bool {
        self.limit.is_some()
            || matches!(
                self.operation,
                Operation::Settling {
                    deadline_ns: Some(_),
                    ..
                }
            )
    }

    pub(crate) fn signals(&self) -> impl Iterator<Item = &Signal> {
        std::iter::once(&self.signal).chain(match &self.reference {
            Reference::Series { signal, .. } => Some(signal),
            _ => None,
        })
    }

    fn validate_window(&self, window: Window) -> Result<(), Error> {
        window.validate()?;
        name(&self.id)?;
        self.signal.validate()?;
        self.source.validate()?;
        let unit = match &self.reference {
            Reference::Constant { value, unit } => {
                finite(*value)?;
                unit
            }
            Reference::Series { signal, .. } => {
                signal.validate()?;
                &signal.unit
            }
            Reference::Step {
                stimulus,
                before,
                after,
                unit,
            } => {
                name(stimulus)?;
                finite(*before)?;
                finite(*after)?;
                unit
            }
        };
        if unit != &self.signal.unit || self.limit.is_some_and(|v| !v.is_finite() || v < 0.0) {
            return Err(invalid(
                "reference.unit или limit: неверная единица или предел",
            ));
        }
        match self.operation {
            Operation::Absolute { at_ns }
                if !(window.start_ns..=window.end_ns).contains(&at_ns) =>
            {
                return Err(invalid("operation.at_ns: точка вне окна"));
            }
            Operation::Settling { band, deadline_ns } => {
                if !band.is_finite() || band < 0.0 || deadline_ns == Some(0) || self.limit.is_some()
                {
                    return Err(invalid("operation: неверная полоса или срок установления"));
                }
            }
            _ => (),
        }
        if matches!(
            self.operation,
            Operation::Settling { .. } | Operation::Overshoot | Operation::OvershootFraction
        ) && !matches!(self.reference, Reference::Step { .. })
        {
            return Err(invalid(
                "reference: нужна ступень с фактическим моментом воздействия",
            ));
        }
        Ok(())
    }

    pub(crate) fn validate(&self, mode: &OperatingMode) -> Result<(), Error> {
        let window = *mode
            .windows
            .get(&self.window)
            .ok_or_else(|| invalid("window: неизвестное окно метрики"))?;
        self.validate_window(window)?;
        if let Reference::Step { stimulus, .. } = &self.reference {
            let step = mode
                .steps
                .iter()
                .find(|s| s.id == *stimulus)
                .ok_or_else(|| invalid("reference.stimulus: неизвестное воздействие"))?;
            let latest = step
                .latest_ns()
                .ok_or_else(|| invalid("reference.stimulus: переполнение времени"))?;
            if step.at_ns < window.start_ns || latest >= window.end_ns {
                return Err(invalid(
                    "reference.stimulus: воздействие вне рабочего окна метрики",
                ));
            }
            if let Operation::Settling {
                deadline_ns: Some(deadline),
                ..
            } = self.operation
            {
                latest
                    .checked_add(deadline)
                    .ok_or_else(|| invalid("operation.deadline_ns: переполнение времени"))?;
            }
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Point {
    pub at_ns: u64,
    pub value: f64,
    pub valid: bool,
}

#[derive(Clone, Debug)]
pub struct Series {
    pub signal: Signal,
    pub points: Vec<Point>,
}

impl Series {
    pub(crate) fn validate(&self) -> Result<(), Error> {
        self.signal.validate()?;
        for point in &self.points {
            finite(point.value)?;
        }
        if self.points.windows(2).any(|w| w[0].at_ns >= w[1].at_ns) {
            return Err(invalid(format!(
                "{}: неоднозначные или неупорядоченные моменты",
                self.signal.name
            )));
        }
        Ok(())
    }

    pub(crate) fn slice(&self, window: Window) -> Option<&[Point]> {
        let first = self
            .points
            .partition_point(|p| p.at_ns <= window.start_ns)
            .checked_sub(1)?;
        let last = self.points.partition_point(|p| p.at_ns < window.end_ns);
        let rows = self.points.get(first..=last)?;
        if rows.iter().any(|p| !p.valid)
            || rows
                .windows(2)
                .any(|w| w[1].at_ns - w[0].at_ns > window.max_gap_ns)
        {
            return None;
        }
        Some(rows)
    }

    /// Для журнала канал определяется парой вида записи и имени. Истина доступна оценщику.
    pub fn from_log(log: &Log, kind: SeriesKind, signal: &Signal) -> Result<Self, Error> {
        let specs = match kind {
            SeriesKind::Truth => &log.header().truth,
            SeriesKind::Observation => &log.header().observations,
        };
        Self::from_entries(log.entries(), specs, kind, signal)
    }

    pub(crate) fn from_entries(
        entries: &[LogEntry],
        specs: &[SignalSpec],
        kind: SeriesKind,
        signal: &Signal,
    ) -> Result<Self, Error> {
        let mut matching = specs
            .iter()
            .enumerate()
            .filter(|(_, spec)| spec.name == signal.name);
        let points = match matching.next() {
            Some((index, spec)) => {
                if spec.unit != signal.unit || matching.next().is_some() {
                    return Err(invalid(format!(
                        "{}: неверная единица или неоднозначный сигнал",
                        signal.name
                    )));
                }
                entries
                    .iter()
                    .filter(|entry| {
                        entry.id as usize == index
                            && entry.kind
                                == match kind {
                                    SeriesKind::Truth => LogKind::Truth,
                                    SeriesKind::Observation => LogKind::Observation,
                                }
                    })
                    .map(|entry| Point {
                        at_ns: entry.at.0,
                        value: entry.value,
                        valid: entry.valid,
                    })
                    .collect()
            }
            None => Vec::new(),
        };
        let series = Self {
            signal: signal.clone(),
            points,
        };
        series.validate()?;
        Ok(series)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum MetricValue {
    Value { value: f64 },
    Censored { lower_bound: f64 },
    NotAnalyzed { reason: String },
}

impl MetricValue {
    fn open(reason: impl Into<String>) -> Self {
        Self::NotAnalyzed {
            reason: reason.into(),
        }
    }

    fn checked(&self) -> Result<(), Error> {
        match self {
            Self::Value { value } | Self::Censored { lower_bound: value } => {
                crate::task::finite(*value)
            }
            Self::NotAnalyzed { reason } => name(reason),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct MetricEvaluation {
    pub value: MetricValue,
    pub outcome: Option<Outcome>,
}

fn finite(value: f64) -> Result<f64, Error> {
    crate::task::finite(value)?;
    Ok(value)
}

fn lerp(a: f64, b: f64, ratio: f64) -> f64 {
    a * (1.0 - ratio) + b * ratio
}

fn interpolate(series: &[Point], at: u64, policy: Interpolation, left: bool) -> f64 {
    let i = series.partition_point(|p| p.at_ns <= at);
    let a = series[i - 1];
    if policy == Interpolation::Hold {
        if left && a.at_ns == at && i >= 2 {
            return series[i - 2].value;
        }
        return a.value;
    }
    if a.at_ns == at {
        return a.value;
    }
    let b = series[i];
    lerp(
        a.value,
        b.value,
        (at - a.at_ns) as f64 / (b.at_ns - a.at_ns) as f64,
    )
}

enum EvaluatedReference<'a> {
    Constant(f64),
    Series(&'a [Point]),
    Step { at: u64, before: f64, after: f64 },
}

impl EvaluatedReference<'_> {
    fn value(&self, at_ns: u64, policy: Interpolation, left: bool) -> f64 {
        match self {
            Self::Constant(value) => *value,
            Self::Series(series) => interpolate(series, at_ns, policy, left),
            Self::Step { at, before, after } => {
                if at_ns < *at || left && at_ns == *at {
                    *before
                } else {
                    *after
                }
            }
        }
    }
}

fn metric_status(spec: &MetricSpec, value: &MetricValue) -> Option<Outcome> {
    let exceeded = match spec.operation {
        Operation::Settling { deadline_ns, .. } => {
            let deadline = u128::from(deadline_ns?);
            match *value {
                MetricValue::Value { value } => (value as u128) > deadline,
                MetricValue::Censored { lower_bound } => (lower_bound as u128) >= deadline,
                _ => false,
            }
        }
        _ => {
            let limit = spec.limit?;
            matches!(*value, MetricValue::Value { value } if value > limit)
        }
    };
    Some(if exceeded {
        Outcome::Fail
    } else if matches!(value, MetricValue::Value { .. }) && spec.source.can_prove() {
        Outcome::Pass
    } else {
        Outcome::NotAnalyzed
    })
}

/// Линейное правило интегрирует квадрат линейной ошибки точно на каждом промежутке.
/// Пропуски, отсутствие опоры и непокрытые границы оставляют метрику открытой.
pub fn evaluate_metric(
    spec: &MetricSpec,
    window: Window,
    measured: &Series,
    reference: Option<&Series>,
    applied_at_ns: Option<u64>,
) -> Result<MetricEvaluation, Error> {
    spec.validate_window(window)?;
    measured.validate()?;
    if let Some(series) = reference {
        series.validate()?;
    }
    if measured.signal != spec.signal {
        return Err(invalid("сигнал не соответствует метрике"));
    }
    let finish = |value: MetricValue| -> Result<MetricEvaluation, Error> {
        value.checked()?;
        let outcome = metric_status(spec, &value);
        Ok(MetricEvaluation { value, outcome })
    };
    let Some(samples) = measured.slice(window) else {
        return finish(MetricValue::open("неполное окно измерения"));
    };
    let reference = match &spec.reference {
        Reference::Constant { value, .. } => EvaluatedReference::Constant(*value),
        Reference::Series { signal, .. } => {
            if reference.is_some_and(|s| s.signal != *signal) {
                return Err(invalid("сигнал опоры не соответствует метрике"));
            }
            let Some(points) = reference.and_then(|s| s.slice(window)) else {
                return finish(MetricValue::open("нет полного окна опоры"));
            };
            EvaluatedReference::Series(points)
        }
        Reference::Step { before, after, .. } => {
            let Some(at) = applied_at_ns.filter(|at| (window.start_ns..window.end_ns).contains(at))
            else {
                return finish(MetricValue::open("нет фактического момента ступени в окне"));
            };
            EvaluatedReference::Step {
                at,
                before: *before,
                after: *after,
            }
        }
    };
    let policy = spec.interpolation;
    let error = |time, left| {
        finite(interpolate(samples, time, policy, left) - reference.value(time, policy, left))
    };
    let result = match spec.operation {
        Operation::Absolute { at_ns } => MetricValue::Value {
            value: error(at_ns, false)?.abs(),
        },
        Operation::TrackingRms => {
            let mut times: BTreeSet<_> = samples
                .iter()
                .map(|p| p.at_ns)
                .filter(|&t| t > window.start_ns && t < window.end_ns)
                .collect();
            match &reference {
                EvaluatedReference::Series(series) => times.extend(
                    series
                        .iter()
                        .map(|p| p.at_ns)
                        .filter(|&t| t > window.start_ns && t < window.end_ns),
                ),
                EvaluatedReference::Step { at, .. } => {
                    times.insert(*at);
                }
                _ => (),
            }
            times.extend([window.start_ns, window.end_ns]);
            let times: Vec<_> = times.into_iter().collect();
            // При общем масштабе интегрируем нормированные ошибки, чтобы избежать переполнения квадратов.
            let mut intervals = Vec::new();
            let mut scale = 0.0_f64;
            for t in times.windows(2) {
                let e0 = error(t[0], false)?;
                let e1 = error(t[1], true)?;
                scale = scale.max(e0.abs()).max(e1.abs());
                intervals.push((t[1] - t[0], e0, e1));
            }
            let mut sum = 0.0;
            if scale > 0.0 {
                for (dt, a, b) in intervals {
                    let (a, b) = (a / scale, b / scale);
                    // Интеграл квадрата линейной функции на единичном интервале.
                    sum += (dt as f64 / (window.end_ns - window.start_ns) as f64)
                        * (a * a + a * b + b * b)
                        / 3.0;
                }
            }
            MetricValue::Value {
                value: scale * sum.sqrt(),
            }
        }
        Operation::Settling { band, .. } => {
            let EvaluatedReference::Step { at, after, .. } = reference else {
                return Err(invalid("для установления нужна ступень"));
            };
            let mut last_time = at;
            let mut last_error = finite(interpolate(samples, at, policy, false) - after)?;
            let mut entering = at;
            for t in samples
                .iter()
                .map(|p| p.at_ns)
                .filter(|&t| at < t && t < window.end_ns)
                .chain(std::iter::once(window.end_ns))
            {
                let e = finite(interpolate(samples, t, policy, false) - after)?;
                if last_error.abs() > band && e.abs() <= band {
                    entering = if policy == Interpolation::Hold {
                        t
                    } else {
                        let crossing = band.copysign(last_error);
                        let scale = last_error.abs().max(e.abs());
                        let ratio = ((last_error / scale - crossing / scale)
                            / (last_error / scale - e / scale))
                            .clamp(0.0, 1.0);
                        last_time
                            + (((t - last_time) as f64 * ratio).ceil() as u64).min(t - last_time)
                    };
                }
                last_time = t;
                last_error = e;
            }
            let censored = last_error.abs() > band || entering >= window.end_ns;
            let duration = if censored {
                window.end_ns - at
            } else {
                entering - at
            };
            let value = duration as f64;
            if value as u128 != u128::from(duration) {
                MetricValue::open(
                    "длительность не представима в шкале отчёта с точностью до наносекунды",
                )
            } else if censored {
                MetricValue::Censored { lower_bound: value }
            } else {
                MetricValue::Value { value }
            }
        }
        Operation::Overshoot | Operation::OvershootFraction => {
            let EvaluatedReference::Step { at, before, after } = reference else {
                return Err(invalid("для перерегулирования нужна ступень"));
            };
            let amplitude = finite(after - before)?;
            if amplitude == 0.0 {
                return finish(MetricValue::open(
                    "нулевая ступень: направление превышения не определено",
                ));
            }
            let mut peak = 0.0_f64;
            for t in std::iter::once(at)
                .chain(
                    samples
                        .iter()
                        .map(|p| p.at_ns)
                        .filter(|&t| at < t && t < window.end_ns),
                )
                .chain(std::iter::once(window.end_ns))
            {
                peak = peak.max(finite(
                    (interpolate(samples, t, policy, false) - after) * amplitude.signum(),
                )?);
            }
            MetricValue::Value {
                value: if matches!(spec.operation, Operation::OvershootFraction) {
                    peak / amplitude.abs()
                } else {
                    peak
                },
            }
        }
    };
    finish(result)
}

/// Сборщик фиксирует реально установленные значения. Отпечаток плана берётся
/// из SampledInputs после проверки установленных моделей и начального состояния.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Installation {
    pub fingerprint: Fingerprint,
    pub run_seed: u64,
    pub physics_hz: u64,
    pub values: BTreeMap<String, f64>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FaultObservation {
    pub fault: Fault,
    pub channel: String,
    pub requested_ns: u64,
    pub applied_ns: u64,
    pub before_from_ns: u64,
    pub before_through_ns: u64,
    pub after_from_ns: u64,
    pub after_through_ns: u64,
    /// Наибольший фактический пробел до/после события.
    pub max_gap_ns: u64,
}

impl FaultObservation {
    /// Двоичная истина для инъекции с состоянием: 0 до факта, 1 после.
    /// Для отказов связи адаптер проверяет состояния отдельных доставок.
    pub fn from_state(
        planned: &crate::campaign::PlannedFault,
        log: &Log,
    ) -> Result<Option<Self>, Error> {
        let signal = Signal {
            name: planned.channel.clone(),
            unit: "1".into(),
        };
        let series = Series::from_log(log, SeriesKind::Truth, &signal)?;
        let window = planned.window;
        let Some(points) = series.slice(window) else {
            return Ok(None);
        };
        let Some(index) = points.iter().position(|p| p.value == 1.0) else {
            return Ok(None);
        };
        let Some(before) = index.checked_sub(1).map(|i| points[i]) else {
            return Ok(None);
        };
        let after = points[index];
        if points[..index].iter().any(|p| p.value != 0.0)
            || points[index..].iter().any(|p| p.value != 1.0)
        {
            return Ok(None);
        }
        Ok(Some(Self {
            fault: planned.schedule.fault.clone(),
            channel: planned.channel.clone(),
            requested_ns: planned.schedule.at_ns,
            applied_ns: after.at_ns,
            before_from_ns: points[0].at_ns,
            before_through_ns: before.at_ns,
            after_from_ns: after.at_ns,
            after_through_ns: points.last().expect("есть отсчёт после отказа").at_ns,
            max_gap_ns: points
                .windows(2)
                .map(|w| w[1].at_ns - w[0].at_ns)
                .max()
                .expect("есть отсчёты до и после отказа"),
        }))
    }
}

#[derive(Clone, Debug)]
pub struct StepObservation {
    pub id: String,
    pub target: Target,
    pub unit: String,
    pub value: f64,
    pub applied_ns: u64,
}

/// Вход описательной статистики. Живые доказательства для вердикта создают
/// отдельные оценщики и адаптеры, сводка не является доказательством безопасности.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RunSummary {
    pub key: RunKey,
    pub installation: Option<Installation>,
    pub observed_through_ns: u64,
    pub completed: bool,
    pub criteria_complete: bool,
    pub outcome: Outcome,
    pub faults: Vec<FaultObservation>,
    pub metrics: BTreeMap<String, MetricValue>,
}

fn criterion_evidence(
    log: &Log,
    criterion: &crate::campaign::Criterion,
    window: Window,
) -> Result<(Option<[f64; 2]>, Outcome, bool), Error> {
    crate::campaign::validate_limits(&criterion.limit)?;
    let signal = Series::from_log(log, SeriesKind::Truth, criterion.limit.signal())?;
    let covered = signal.slice(window).filter(|points| {
        // Журнал должен содержать истинное состояние на обеих границах критерия.
        points.first().is_some_and(|p| p.at_ns == window.start_ns)
            && points.last().is_some_and(|p| p.at_ns == window.end_ns)
    });
    let (source, invalid_data, violation) = match &criterion.limit {
        Limit::Range {
            min, max, source, ..
        } => (
            source,
            false,
            signal.points.iter().any(|p| {
                p.valid
                    && (window.start_ns..=window.end_ns).contains(&p.at_ns)
                    && (min.is_some_and(|min| p.value < min)
                        || max.is_some_and(|max| p.value > max))
            }),
        ),
        Limit::Forbidden { source, .. } => (
            source,
            covered
                .is_some_and(|points| points.iter().any(|p| p.value != 0.0 && p.value != 1.0)),
            signal.points.iter().any(|p| {
                p.valid
                    && (window.start_ns..=window.end_ns).contains(&p.at_ns)
                    && p.value == 1.0
            }),
        ),
    };
    let complete = log.end().time.0 >= window.end_ns && covered.is_some() && !invalid_data;
    let outcome = if violation {
        Outcome::Fail
    } else if complete && source.can_prove() {
        Outcome::Pass
    } else {
        Outcome::NotAnalyzed
    };
    let observed = covered.filter(|_| complete).and_then(|points| {
        points.iter().map(|p| [p.value, p.value]).reduce(|[lo, hi], [v, _]| [lo.min(v), hi.max(v)])
    });
    Ok((observed, outcome, complete))
}

/// Пределы и огибающие критериев из того же журнала, который оценивает сценарий.
pub fn scenario_margins(
    mode: &OperatingMode,
    log: &Log,
) -> Result<BTreeMap<String, (crate::campaign::depth_search::Criterion, crate::campaign::depth_search::MarginEvidence)>, Error> {
    use crate::campaign::depth_search::criterion_margin;

    mode.criteria.iter().map(|criterion| {
        let window = mode.windows[&criterion.window];
        let (observed, outcome, _) = criterion_evidence(log, criterion, window)?;
        Ok((criterion.id.clone(), criterion_margin(&criterion.limit, window, observed, outcome)))
    }).collect()
}

/// Оценка общих критериев по завершённому журналу. Для непустого расписания
/// операции и модельные порты подтверждает установщик, усилия сверяются с журналом.
pub fn evaluate_scenario(
    row: &PlanRow,
    inputs: &SampledInputs,
    log: &Log,
    installation: Installation,
    steps: &[StepObservation],
    faults: &[FaultObservation],
) -> Result<RunSummary, Error> {
    for entry in log.entries() {
        finite(entry.value)?;
    }
    for value in installation.values.values() {
        finite(*value)?;
    }
    for step in steps {
        finite(step.value)?;
    }
    if row.key != inputs.key || log.header().seed != installation.run_seed {
        return Err(invalid(
            "журнал, установка и ключ относятся к разным прогонам",
        ));
    }
    if inputs.experiment.as_ref().and_then(|e| e.kind).is_some() {
        return Err(invalid("для ссылочной постановки нужен её оценщик"));
    }
    let mode = inputs
        .mode
        .as_ref()
        .ok_or_else(|| invalid("у строки нет постановки"))?;
    let mut applied = BTreeMap::new();
    let mut sources = mode.source.can_prove()
        && mode.initial.iter().all(|i| i.source.can_prove())
        && mode.poses.iter().all(|i| i.source.can_prove());
    for step in &mode.steps {
        sources &= step.source.can_prove();
        let confirmed = match &step.target {
            Target::Effort { name } => log
                .header()
                .commands
                .iter()
                .position(|s| s.name == *name && s.unit == step.unit)
                .and_then(|index| {
                    let mut matching = log.entries().iter().filter(|e| {
                        e.id as usize == index
                            && step.accepts_time(e.at.0)
                            && matches!(
                                e.kind,
                                LogKind::Command(
                                    CommandSource::External | CommandSource::BeforeStart
                                )
                            )
                    });
                    let first = matching.next()?;
                    (first.valid
                        && first.value.to_bits() == step.value.to_bits()
                        && matching.next().is_none())
                    .then_some(first.at.0)
                }),
            _ => {
                let mut seen = steps.iter().filter(|s| s.id == step.id);
                seen.next()
                    .filter(|actual| {
                        actual.target == step.target
                            && actual.unit == step.unit
                            && actual.value.to_bits() == step.value.to_bits()
                            && step.accepts_time(actual.applied_ns)
                            && seen.next().is_none()
                    })
                    .map(|actual| actual.applied_ns)
            }
        };
        if let Some(at) = confirmed {
            applied.insert(step.id.as_str(), at);
        }
    }
    let end = log.end();
    let completed = end.started
        && end.steps > 0
        && end.time.0 >= mode.horizon_ns
        && end.status == Status::Running;
    let mut outcomes = Vec::new();
    let mut criteria_complete = true;
    for criterion in &mode.criteria {
        let window = mode.windows[&criterion.window];
        let (_, outcome, complete) = criterion_evidence(log, criterion, window)?;
        criteria_complete &= complete;
        outcomes.push(outcome);
    }
    let mut metrics = BTreeMap::new();
    for spec in &mode.metrics {
        let series = Series::from_log(log, spec.series, &spec.signal)?;
        let (at, reference) = match &spec.reference {
            Reference::Step { stimulus, .. } => (applied.get(stimulus.as_str()).copied(), None),
            Reference::Series { signal, series } => {
                (None, Some(Series::from_log(log, *series, signal)?))
            }
            _ => (None, None),
        };
        let measurement = evaluate_metric(
            spec,
            mode.windows[&spec.window],
            &series,
            reference.as_ref(),
            at,
        )?;
        if let Some(outcome) = measurement.outcome {
            criteria_complete &= outcome != Outcome::NotAnalyzed;
            outcomes.push(outcome);
        }
        metrics.insert(spec.id.clone(), measurement.value);
    }
    let unexpected_effort = log.entries().iter().any(|entry| {
        matches!(
            entry.kind,
            LogKind::Command(CommandSource::External | CommandSource::BeforeStart)
        ) && !log
            .header()
            .commands
            .get(entry.id as usize)
            .is_some_and(|command| {
                mode.steps.iter().any(|step| {
                    matches!(&step.target, Target::Effort { name } if *name == command.name)
                        && step.accepts_time(entry.at.0)
                        && step.unit == command.unit
                })
            })
    });
    if !sources || unexpected_effort || applied.len() != mode.steps.len() {
        outcomes.push(Outcome::NotAnalyzed);
    }
    let covered_faults = row.faults.len() == faults.len()
        && row
            .faults
            .iter()
            .all(|fault| matching_fault(fault, faults, end.time.0));
    let outcome = if !matches!(row.disposition, Disposition::Ready)
        || !covered_faults
        || !installation_matches(inputs, &installation, row.grid.hz)
    {
        Outcome::NotAnalyzed
    } else {
        let outcome = Outcome::aggregate(outcomes);
        if !completed && outcome == Outcome::Pass {
            Outcome::NotAnalyzed
        } else {
            outcome
        }
    };
    Ok(RunSummary {
        key: row.key.clone(),
        installation: Some(installation),
        observed_through_ns: end.time.0,
        completed,
        criteria_complete,
        outcome,
        faults: faults.to_vec(),
        metrics,
    })
}

pub(crate) fn matching_fault(
    expected: &crate::campaign::PlannedFault,
    facts: &[FaultObservation],
    through_ns: u64,
) -> bool {
    let mut found = facts.iter().filter(|f| f.fault == expected.schedule.fault);
    let Some(actual) = found.next() else {
        return false;
    };
    let start = expected.schedule.at_ns;
    let window = expected.window;
    found.next().is_none()
        && actual.channel == expected.channel
        && actual.requested_ns == start
        && (start..=start + expected.schedule.max_lag_ns).contains(&actual.applied_ns)
        && actual.before_from_ns <= window.start_ns
        && actual.before_from_ns <= actual.before_through_ns
        && window.start_ns <= actual.before_through_ns
        && actual.before_through_ns < actual.applied_ns
        && actual.after_from_ns == actual.applied_ns
        && actual.after_through_ns >= actual.after_from_ns
        && actual.after_through_ns >= window.end_ns
        && actual.after_through_ns <= through_ns
        && actual.applied_ns - actual.before_through_ns <= actual.max_gap_ns
        && (1..=window.max_gap_ns).contains(&actual.max_gap_ns)
}

#[derive(Clone, Debug, Serialize)]
pub struct RowStats {
    pub key: RunKey,
    pub completed: bool,
    pub outcome: Option<Outcome>,
    pub reasons: Vec<String>,
    pub sample: Sample,
    pub metrics: BTreeMap<String, MetricValue>,
}

pub(crate) fn installation_matches(
    expected: &SampledInputs,
    actual: &Installation,
    physics_hz: u64,
) -> bool {
    actual.fingerprint == expected.fingerprint
        && actual.run_seed == expected.sample.seeds.run
        && actual.physics_hz == physics_hz
        && actual.values.len() == expected.values().len()
        && expected.values().iter().all(|v| {
            actual
                .values
                .get(&v.id)
                .is_some_and(|actual| actual.is_finite() && actual.to_bits() == v.value.to_bits())
        })
}

fn classify(
    plan: &CampaignPlan,
    row: &PlanRow,
    result: Option<&RunSummary>,
) -> Result<RowStats, Error> {
    let mut stats = RowStats {
        key: row.key.clone(),
        completed: false,
        outcome: Some(Outcome::NotAnalyzed),
        reasons: Vec::new(),
        sample: plan.sample(row)?,
        metrics: BTreeMap::new(),
    };
    match &row.disposition {
        Disposition::Inapplicable { reasons, .. } => {
            stats.outcome = None;
            stats.reasons.clone_from(reasons);
            return Ok(stats);
        }
        Disposition::NotAnalyzed { reasons } => {
            stats.reasons.clone_from(reasons);
            return Ok(stats);
        }
        Disposition::Ready => (),
    }
    let Some(result) = result else {
        stats
            .reasons
            .push("нет результата прогона (в том числе при фильтрации)".into());
        return Ok(stats);
    };
    let mode = plan
        .mode(&row.key)
        .expect("в исполняемой строке есть режим");
    let expected = plan.materialize(&row.key)?;
    if !result
        .installation
        .as_ref()
        .is_some_and(|actual| installation_matches(&expected, actual, row.grid.hz))
    {
        stats
            .reasons
            .push("не подтверждены установленные входы, зерно или сетка".into());
        return Ok(stats);
    }
    stats.completed = result.completed && result.observed_through_ns >= mode.horizon_ns;
    if !stats.completed {
        stats
            .reasons
            .push("прогон не достиг обязательного горизонта".into());
    }
    let covered_faults = row.faults.len() == result.faults.len()
        && row
            .faults
            .iter()
            .all(|fault| matching_fault(fault, &result.faults, result.observed_through_ns));
    if !covered_faults {
        stats
            .reasons
            .push("инъекция или её наблюдение в окне до и после не подтверждены".into());
        return Ok(stats);
    }
    if stats.completed {
        for spec in &mode.metrics {
            let value = result
                .metrics
                .get(&spec.id)
                .cloned()
                .unwrap_or_else(|| MetricValue::open("нет результата метрики"));
            stats.metrics.insert(spec.id.clone(), value);
        }
    }
    let metric_outcomes = mode
        .metrics
        .iter()
        .filter_map(|spec| {
            let value = stats.metrics.get(&spec.id)?;
            metric_status(spec, value)
        })
        .collect::<Vec<_>>();
    let complete =
        result.criteria_complete && metric_outcomes.iter().all(|&o| o != Outcome::NotAnalyzed);
    if !complete {
        stats.reasons.push("неполное покрытие критериев".into());
    }
    let valid_sources = mode.source.can_prove()
        && mode.initial.iter().all(|v| v.source.can_prove())
        && mode.poses.iter().all(|v| v.source.can_prove())
        && mode.steps.iter().all(|v| v.source.can_prove())
        && mode
            .criteria
            .iter()
            .all(|criterion| match &criterion.limit {
                Limit::Range { source, .. } | Limit::Forbidden { source, .. } => source.can_prove(),
            })
        && row.profile_source.can_prove()
        && row.grid.source.can_prove()
        && row
            .combination_source
            .as_ref()
            .is_none_or(Source::can_prove)
        && row.faults.iter().all(|f| f.schedule.source.can_prove())
        && stats
            .sample
            .values
            .iter()
            .all(|value| value.source.can_prove());
    if !valid_sources {
        stats
            .reasons
            .push("часть источников является оценкой".into());
    }
    if result.outcome == Outcome::Fail || metric_outcomes.contains(&Outcome::Fail) {
        stats.outcome = Some(Outcome::Fail);
    } else if stats.completed && complete && valid_sources && result.outcome == Outcome::Pass {
        stats.outcome = Some(Outcome::Pass);
    } else if stats.reasons.is_empty() {
        stats
            .reasons
            .push("критерий оставлен открытым оценщиком".into());
    }
    Ok(stats)
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Population {
    Deterministic,
    Random,
    Batch { batch: u32 },
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct SeriesKey {
    pub station: String,
    pub mode: String,
    pub path: String,
    pub faults: Vec<Fault>,
    pub grid: String,
    pub population: Population,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct Interval {
    pub low: f64,
    pub high: f64,
}

#[derive(Clone, Debug, Serialize)]
pub struct Confidence {
    pub level: Option<f64>,
    pub interval: Option<Interval>,
    pub reason: Option<String>,
}

#[derive(Clone, Debug, Serialize)]
pub struct Worst {
    pub value: f64,
    pub key: RunKey,
    pub seeds: Seeds,
    pub parameters: Vec<AppliedValue>,
}

#[derive(Clone, Debug, Serialize)]
pub struct ScalarStats {
    pub unit: String,
    pub planned: usize,
    pub measured: usize,
    pub censored: usize,
    pub median: Option<f64>,
    pub p95: Option<f64>,
    pub worst: Option<Worst>,
    pub repeatability: MetricValue,
}

#[derive(Clone, Debug, Serialize)]
pub struct SeriesStats {
    pub key: SeriesKey,
    pub planned: usize,
    pub applicable: usize,
    pub inapplicable: usize,
    pub completed: usize,
    pub passed: usize,
    pub failed: usize,
    pub open: usize,
    pub classified: usize,
    pub classified_success_rate: Option<f64>,
    pub applicable_success_bounds: Option<Interval>,
    pub confidence: Confidence,
    pub metrics: BTreeMap<String, ScalarStats>,
}

#[derive(Clone, Debug, Serialize)]
pub struct CampaignStats {
    pub rows: Vec<RowStats>,
    pub series: Vec<SeriesStats>,
}

fn independent(
    probability: Option<&crate::sampling::Probability>,
    key: &SeriesKey,
) -> Result<(), &'static str> {
    let p = probability.ok_or("для допуска не задан вероятностный закон")?;
    if !p.sourced() {
        return Err("источник закона или независимости является оценкой");
    }
    match (&key.population, p.independence) {
        (Population::Random, Independence::Batches)
        | (Population::Batch { .. }, Independence::WithinBatch) => Ok(()),
        (Population::Deterministic, _) => Err("номинал и границы не образуют вероятностную серию"),
        (Population::Random, Independence::WithinBatch) => {
            Err("образцы зависимы через партию, общий интервал для них не обоснован")
        }
        _ => Err("не обоснованы независимые единицы испытаний"),
    }
}

fn same_conditions(a: &Sample, b: &Sample) -> bool {
    a.values.len() == b.values.len()
        && a.values.iter().zip(&b.values).all(|(a, b)| {
            a.id == b.id
                && a.parameter == b.parameter
                && a.unit == b.unit
                && a.value.to_bits() == b.value.to_bits()
        })
}

fn scalar_stats(
    id: &str,
    unit: &str,
    worse: Worse,
    absolute: bool,
    rows: &[&RowStats],
    independent: Result<(), &'static str>,
) -> Result<ScalarStats, Error> {
    let mut values = Vec::new();
    let mut censored = 0;
    for row in rows {
        match row.metrics.get(id) {
            Some(MetricValue::Value { value }) => values.push((*value, *row)),
            Some(MetricValue::Censored { .. }) => censored += 1,
            _ => (),
        }
    }
    let n = values.len();
    // Для равных значений худшую строку выбираем отдельно по порядку ключей.
    values.sort_by(|a, b| a.0.total_cmp(&b.0));
    let (median, p95, worst) = if n == 0 {
        (None, None, None)
    } else {
        let middle = n / 2;
        let median = if n % 2 == 0 {
            f64::midpoint(values[middle - 1].0, values[middle].0)
        } else {
            values[middle].0
        };
        // §4.9 и план A: ближайший верхний ранг 95/100, без округления f64.
        let p95 = values[n - n / 20 - 1].0;
        let extreme = match worse {
            Worse::Higher => values[n - 1].0,
            Worse::Lower => values[0].0,
        };
        let row = values
            .iter()
            .filter(|(value, _)| *value == extreme)
            .map(|(_, row)| *row)
            .min_by_key(|row| &row.key)
            .expect("крайнее значение есть");
        (
            Some(finite(median)?),
            Some(p95),
            Some(Worst {
                value: extreme,
                key: row.key.clone(),
                seeds: row.sample.seeds.clone(),
                parameters: row.sample.values.clone(),
            }),
        )
    };
    let repeatability = if !absolute {
        MetricValue::open("повторяемость определена для абсолютной ошибки")
    } else if let Err(reason) = independent {
        MetricValue::open(reason)
    } else if n < 2 || n != rows.len() {
        MetricValue::open("нужна полная серия из двух или более независимых измерений")
    } else if !rows
        .iter()
        .all(|row| same_conditions(&rows[0].sample, &row.sample))
    {
        MetricValue::open("изменились физические условия между измерениями")
    } else {
        let errors: Vec<_> = values.iter().map(|(value, _)| *value).collect();
        MetricValue::Value {
            value: repeatability_value(&errors)?,
        }
    };
    Ok(ScalarStats {
        unit: unit.into(),
        planned: rows.len(),
        measured: n,
        censored,
        median,
        p95,
        worst,
        repeatability,
    })
}

fn repeatability_value(errors: &[f64]) -> Result<f64, Error> {
    let n = errors.len();
    let scale = errors.iter().copied().fold(0.0_f64, f64::max);
    if scale == 0.0 {
        return Ok(0.0);
    }
    // Выборочная дисперсия (делитель n−1) и среднее + 3σ по §4.9 и плану A.
    let mean = errors.iter().map(|x| x / scale).sum::<f64>() / n as f64;
    let variance = errors
        .iter()
        .map(|x| (x / scale - mean).powi(2))
        .sum::<f64>()
        / (n - 1) as f64;
    finite(scale * (mean + 3.0 * variance.sqrt()))
}

fn reduce<'a>(
    probability: Option<&crate::sampling::Probability>,
    key: SeriesKey,
    rows: &[&RowStats],
    specs: impl Iterator<Item = (&'a str, &'a str, Worse, bool)>,
) -> Result<SeriesStats, Error> {
    let planned = rows.len();
    let mut active = Vec::new();
    let mut completed = 0;
    let mut passed = 0;
    let mut failed = 0;
    for row in rows {
        if let Some(outcome) = row.outcome {
            active.push(*row);
            completed += usize::from(row.completed);
            passed += usize::from(outcome == Outcome::Pass);
            failed += usize::from(outcome == Outcome::Fail);
        }
    }
    let applicable = active.len();
    let classified = passed + failed;
    let open = applicable - classified;
    let ratio = |n, d| n as f64 / d as f64;
    let classified_success_rate = (classified > 0).then(|| ratio(passed, classified));
    let applicable_success_bounds = (applicable > 0).then(|| Interval {
        low: ratio(passed, applicable),
        high: ratio(passed + open, applicable),
    });
    let independence = independent(probability, &key);
    let level = if key.population == Population::Deterministic {
        None
    } else {
        probability.map(|p| p.confidence)
    };
    let confidence = match independence.and_then(|()| {
        let expected = probability.and_then(|p| match key.population {
            Population::Random => p.count().ok(),
            Population::Batch { .. } => Some(p.specimens as usize),
            Population::Deterministic => None,
        });
        if applicable == 0
            || open > 0
            || completed != applicable
            || applicable != planned
            || expected != Some(planned)
        {
            Err("для интервала нужна полная однородная серия")
        } else {
            Ok(())
        }
    }) {
        Ok(()) => Confidence {
            level,
            interval: Some(clopper_pearson(
                passed,
                applicable,
                level.expect("закон проверен"),
            )?),
            reason: None,
        },
        Err(reason) => Confidence {
            level,
            interval: None,
            reason: Some(reason.into()),
        },
    };
    let mut metrics = BTreeMap::new();
    for (id, unit, worse, absolute) in specs {
        metrics.insert(
            id.to_owned(),
            scalar_stats(id, unit, worse, absolute, &active, independence)?,
        );
    }
    Ok(SeriesStats {
        key,
        planned,
        applicable,
        inapplicable: planned - applicable,
        completed,
        passed,
        failed,
        open,
        classified,
        classified_success_rate,
        applicable_success_bounds,
        confidence,
        metrics,
    })
}

/// Сохраняет полную матрицу до фильтра, в устойчивом порядке, и считает
/// статистику по одноимённому пути, режиму, сочетанию отказов и сетке.
pub fn summarize(plan: &CampaignPlan, runs: &[RunSummary]) -> Result<CampaignStats, Error> {
    let mut results = BTreeMap::new();
    for run in runs {
        if plan.row(&run.key).is_none() || results.insert(&run.key, run).is_some() {
            return Err(invalid(format!("повторный или чужой ключ: {:?}", run.key)));
        }
        for value in run.installation.iter().flat_map(|i| i.values.values()) {
            finite(*value)?;
        }
        for (id, value) in &run.metrics {
            let spec = plan
                .mode(&run.key)
                .and_then(|mode| mode.metrics.iter().find(|spec| spec.id == *id))
                .ok_or_else(|| invalid(format!("неизвестная метрика {id}")))?;
            value.checked()?;
            if matches!(value, MetricValue::Value { value } if *value < 0.0)
                || matches!(value, MetricValue::Censored { lower_bound } if *lower_bound < 0.0
                    || !matches!(spec.operation, Operation::Settling { .. }))
            {
                return Err(invalid(format!("метрика {id}: неверное значение")));
            }
        }
    }
    let rows: Vec<_> = plan
        .rows()
        .iter()
        .map(|row| classify(plan, row, results.get(&row.key).copied()))
        .collect::<Result<_, _>>()?;
    grouped(rows, plan.probability(), |row| {
        plan.mode(&row.key)
            .into_iter()
            .flat_map(|mode| &mode.metrics)
            .map(|spec| {
                (
                    spec.id.as_str(),
                    spec.unit(),
                    spec.worse,
                    matches!(spec.operation, Operation::Absolute { .. }),
                )
            })
    })
}

fn grouped<'a, I: Iterator<Item = (&'a str, &'a str, Worse, bool)>>(
    rows: Vec<RowStats>,
    probability: Option<&crate::sampling::Probability>,
    specs: impl Fn(&RowStats) -> I,
) -> Result<CampaignStats, Error> {
    let mut groups: BTreeMap<_, Vec<_>> = BTreeMap::new();
    for row in &rows {
        let sample = &row.key.sample;
        let key = SeriesKey {
            station: row.key.station.clone(),
            mode: row.key.mode.clone(),
            path: row.key.path.clone(),
            faults: row.key.faults.clone(),
            grid: row.key.grid.clone(),
            population: if matches!(sample, SampleKey::Random { .. }) {
                Population::Random
            } else {
                Population::Deterministic
            },
        };
        if let SampleKey::Random { batch, .. } = *sample
            && probability.is_some_and(|p| p.independence == Independence::WithinBatch)
        {
            groups
                .entry(SeriesKey {
                    population: Population::Batch { batch },
                    ..key.clone()
                })
                .or_default()
                .push(row);
        }
        groups.entry(key).or_default().push(row);
    }
    let series = groups
        .into_iter()
        .map(|(key, rows)| reduce(probability, key, &rows, specs(rows[0])))
        .collect::<Result<_, _>>()?;
    Ok(CampaignStats { rows, series })
}

/// Сводит однородные дочерние строки ссылочных профилей общим статистическим
/// расчётом. Идентификаторы ряда и единицы задаются оценщиком профиля.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LinkedMetric {
    pub unit: String,
    pub worse: Worse,
    pub absolute_error: bool,
}

pub fn summarize_linked(
    rows: Vec<RowStats>,
    sampling: Option<&crate::sampling::Sampling>,
    metrics: &BTreeMap<RunKey, BTreeMap<String, LinkedMetric>>,
) -> Result<CampaignStats, Error> {
    let allowed = sampling
        .map(|s| s.keys())
        .transpose()?
        .map(|keys| keys.into_iter().collect::<BTreeSet<_>>());
    let mut keys = BTreeSet::new();
    for row in &rows {
        if row.key.sample != row.sample.key
            || !keys.insert(&row.key)
            || allowed
                .as_ref()
                .is_some_and(|allowed| !allowed.contains(&row.key.sample))
        {
            return Err(invalid("несовместимый или повторный ключ ссылочной строки"));
        }
        let key = RunKey {
            sample: SampleKey::Nominal,
            ..row.key.clone()
        };
        for (id, value) in &row.metrics {
            let spec = metrics
                .get(&key)
                .and_then(|specs| specs.get(id))
                .ok_or_else(|| invalid(format!("нет описания метрики «{id}»")))?;
            name(&spec.unit)?;
            value.checked()?;
            if spec.absolute_error && matches!(value, MetricValue::Value { value } if *value < 0.0)
            {
                return Err(invalid(format!("абсолютная ошибка «{id}» отрицательна")));
            }
        }
    }
    grouped(rows, sampling.and_then(|s| s.probability.as_ref()), |row| {
        let mut key = row.key.clone();
        key.sample = SampleKey::Nominal;
        metrics.get(&key).into_iter().flatten().map(|(id, metric)| {
            (
                id.as_str(),
                metric.unit.as_str(),
                metric.worse,
                metric.absolute_error,
            )
        })
    })
}

/// Двусторонний точный интервал Клоппера–Пирсона из плана A.
/// Нижняя граница обращает P(X >= k), верхняя — P(X <= k).
pub fn clopper_pearson(
    successes: usize,
    trials: usize,
    confidence: f64,
) -> Result<Interval, Error> {
    if successes > trials || trials == 0 || !(0.0 < confidence && confidence < 1.0) {
        return Err(invalid("интервал: неверные счётчики или уровень доверия"));
    }
    let alpha = (1.0 - confidence) / 2.0;
    // При k=n хвост равен p^n, при k=0 — (1-p)^n.
    let low = match successes {
        0 => 0.0,
        k if k == trials => (alpha.ln() / trials as f64).exp(),
        k => invert_tail(k, trials, alpha, true),
    };
    let high = match successes {
        0 => -(alpha.ln() / trials as f64).exp_m1(),
        k if k == trials => 1.0,
        k => invert_tail(k, trials, alpha, false),
    };
    if !(0.0 <= low && low <= high && high <= 1.0) {
        return Err(invalid("интервал: численная ошибка обращения хвостов"));
    }
    Ok(Interval { low, high })
}

fn invert_tail(k: usize, n: usize, alpha: f64, upper_tail: bool) -> f64 {
    let (mut lo, mut hi) = (0.0, 1.0);
    loop {
        let p = f64::midpoint(lo, hi);
        if p == lo || p == hi {
            // Край округления выбирается наружу для обеих границ.
            return if upper_tail { lo } else { hi };
        }
        let probability = binomial_tail(k, n, p, upper_tail);
        if if upper_tail {
            probability < alpha
        } else {
            probability >= alpha
        } {
            lo = p;
        } else {
            hi = p;
        }
    }
}

fn binomial_tail(k: usize, n: usize, p: f64, upper: bool) -> f64 {
    let q = 1.0 - p;
    let mode = (((n as f64 + 1.0) * p).floor() as usize).min(n);
    let contains = |i| if upper { i >= k } else { i <= k };
    // Относительные веса: вес моды равен единице. От него движемся в обе
    // стороны отношениями соседних членов биномиального распределения.
    let mut total = 1.0;
    let mut tail = f64::from(contains(mode));
    let mut weight = 1.0;
    for i in (1..=mode).rev() {
        weight *= ((i as f64) / ((n - i + 1) as f64)) * (q / p);
        if weight == 0.0 {
            break;
        }
        total += weight;
        if contains(i - 1) {
            tail += weight;
        }
    }
    weight = 1.0;
    for i in mode..n {
        weight *= (((n - i) as f64) / ((i + 1) as f64)) * (p / q);
        if weight == 0.0 {
            break;
        }
        total += weight;
        if contains(i + 1) {
            tail += weight;
        }
    }
    tail / total
}
