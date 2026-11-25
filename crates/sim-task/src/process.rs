//! Численная приёмка живых прогонов. Сетка времени, варианты и пределы задаются сценарием.

use std::collections::{BTreeMap, BTreeSet};

use serde::Serialize;
use sim_process::{
    Analysis, Interval, SourcedRange,
    contact::{CreepConfiguration, CreepReport, LimitComparison, RigidPlaneSupport},
    work::HazardCheck,
};
use sim_runtime::{
    Runtime, Status,
    process::{PhysicsFrame, ProcessEvent, ProcessTick, SolverProfile},
};
use sim_signal::{Rate, Time};

use crate::campaign::RunKey;
use crate::verdict::{Finding, HazardClass, Level, LimitValue, RequirementKey};
use crate::{Error, Fingerprint, Outcome, invalid};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VariantKind {
    Baseline,
    Solver,
    Material,
}

#[derive(Clone, Debug)]
pub struct TrialVariant {
    pub name: String,
    pub kind: VariantKind,
    /// Номиналы изменяемых параметров, переданные компилятору этого варианта.
    pub materials: BTreeMap<String, f64>,
}

#[derive(Clone, Debug)]
pub enum Series {
    NormalForce([String; 2]),
    Penetration([String; 2]),
    Measurement(String),
    Position(usize),
    Velocity(usize),
    Displacement(usize),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Statistic {
    Minimum,
    Maximum,
    AbsolutePeak,
    Integral,
    End,
    /// Сравнение всей кривой на объединении границ. Пределы действуют во всём окне.
    Continuous,
}

#[derive(Clone, Debug)]
pub struct Criterion {
    pub name: String,
    pub unit: String,
    pub series: Series,
    pub statistic: Statistic,
    pub tolerance: SourcedRange,
    /// Остатки сверх наблюдаемых разностей, с обоснованием для рабочего окна.
    pub remainder: ErrorBounds,
    pub lower: Option<SourcedRange>,
    pub upper: Option<SourcedRange>,
}

#[derive(Clone, Debug)]
pub struct ErrorBounds {
    pub discretization: SourcedRange,
    pub solver: SourcedRange,
    pub materials: SourcedRange,
}

#[derive(Clone, Debug)]
pub struct EventExpectation {
    pub name: String,
    pub state: String,
    /// Для физического перехода без внешнего запроса оставляется неопределённым.
    pub requested: Option<Time>,
    pub applied_window: Interval,
    pub phase_tolerance_ns: SourcedRange,
}

#[derive(Clone, Debug)]
pub struct StateExpectation {
    pub name: String,
    /// Сжатая последовательность состояний в общем окне, включая начальное и конечное.
    pub sequence: Vec<String>,
}

#[derive(Clone, Debug)]
pub struct CreepCheck {
    pub metric: String,
    pub report: CreepReport,
    pub configuration: CreepConfiguration,
    pub support: RigidPlaneSupport,
    /// Отпечаток сцены, использованной для проверки применимости замера.
    pub fixture_fingerprint: u64,
    /// Мировая точка, относительно которой задан момент прямоугольной опоры.
    pub support_origin_m: [f64; 3],
    pub limit_m: SourcedRange,
}

#[derive(Clone, Debug)]
pub struct ConvergencePlan {
    pub base_hz: u64,
    pub window: Interval,
    pub window_source: String,
    pub variants: Vec<TrialVariant>,
    pub material_ranges: BTreeMap<String, SourcedRange>,
    pub criteria: Vec<Criterion>,
    pub events: Vec<EventExpectation>,
    pub states: Vec<StateExpectation>,
    pub creep: Vec<CreepCheck>,
}

#[derive(Debug, Serialize)]
pub struct TrialReport {
    pub variant: String,
    pub rate_hz: u64,
    pub journal: Option<u64>,
    pub steps: u64,
    pub completed: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub snapshot: Option<crate::verdict::SnapshotAudit>,
    pub solver: Option<String>,
    pub materials: BTreeMap<String, f64>,
    pub events: Vec<EventReport>,
    pub states: BTreeMap<String, Vec<String>>,
    pub missing: Vec<String>,
    pub violations: Vec<String>,
    pub hazards: Vec<HazardReport>,
    pub covered_hazards: BTreeSet<(String, HazardClass)>,
}

#[derive(Debug, Serialize)]
pub struct HazardReport {
    pub subject: String,
    pub class: HazardClass,
    pub at_ns: u64,
    pub outcome: Outcome,
    pub reason: String,
    pub sources: Vec<ParameterAudit>,
    pub sources_confirmed: bool,
}

#[derive(Debug, Serialize)]
pub struct EventReport {
    pub name: String,
    pub state: String,
    pub requested_ns: u64,
    pub applied_ns: u64,
    pub phase_ns: u64,
}

#[derive(Debug, PartialEq, Serialize)]
pub struct ParameterAudit {
    pub range: [f64; 3],
    pub source: String,
}

impl From<&SourcedRange> for ParameterAudit {
    fn from(parameter: &SourcedRange) -> Self {
        Self {
            range: [parameter.min, parameter.nominal, parameter.max],
            source: format!("{:?}", parameter.provenance),
        }
    }
}

#[derive(Debug, Serialize)]
pub struct MetricReport {
    pub name: String,
    pub unit: String,
    #[serde(skip)]
    series: Series,
    /// По варианту и затем в порядке h, h/2, h/4.
    pub values: Vec<Option<[f64; 2]>>,
    pub adjacent_differences: Vec<Option<[f64; 2]>>,
    /// Оценка log₂(Δ₁/Δ₂). При росте разностей порядок отрицателен.
    pub observed_orders: Vec<Option<f64>>,
    /// Время максимума ряда на каждой сетке, если критерий проверяет максимум.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub maxima_at_ns: Option<Vec<Option<u64>>>,
    #[serde(skip)]
    pub extrema_at_ns: Vec<Option<[u64; 2]>>,
    #[serde(skip)]
    pub statistic: Statistic,
    #[serde(skip)]
    pub extrema_values: Vec<Option<[f64; 2]>>,
    pub tolerance: ParameterAudit,
    pub input_bounds: BTreeMap<String, ParameterAudit>,
    pub discretization: Option<f64>,
    pub solver: Option<f64>,
    pub materials: Option<f64>,
    pub interval: Option<[f64; 2]>,
    pub creep_interval: Option<[f64; 2]>,
    pub outcome: Outcome,
    pub detail: String,
}

#[derive(Debug, Serialize)]
pub struct ConvergenceReport {
    pub outcome: Outcome,
    pub window_ns: [u64; 2],
    pub window_source: String,
    pub material_ranges: BTreeMap<String, ParameterAudit>,
    pub trials: Vec<TrialReport>,
    pub metrics: Vec<MetricReport>,
    pub missing: Vec<String>,
    #[serde(skip)]
    seal: Option<Fingerprint>,
    #[serde(skip)]
    profile: Fingerprint,
}

impl ConvergenceReport {
    fn digest(&self) -> Fingerprint {
        Fingerprint::of(
            format!(
                "{:?}{:?}{:?}{:?}{:?}{:?}{:?}",
                self.outcome,
                self.window_ns,
                self.window_source,
                self.material_ranges,
                self.trials,
                self.metrics,
                self.missing
            )
            .as_bytes(),
        )
    }

    pub(crate) fn profile(&self) -> Fingerprint {
        self.profile
    }

    pub(crate) fn is_live(&self) -> bool {
        self.seal == Some(self.digest())
    }

    /// Связывает численное нарушение на сходящихся сетках с типизированной
    /// опасностью того же объекта и окна. Оба факта должны прийти из прогона.
    pub fn hazard_witness(
        &self,
        run: &RunKey,
        class: HazardClass,
        subject: &str,
        criterion: &str,
    ) -> Option<Finding> {
        if !self.is_live()
            || self.trials.is_empty()
            || self.trials.iter().any(|trial| !trial.completed)
        {
            return None;
        }
        // Имена критериев задаются данными. Численный свидетель допустим
        // только для физического ряда того же объекта и класса опасности.
        let (quantity, unit) = match class {
            HazardClass::StoredEnergy => ("energy", "J"),
            HazardClass::SharpEdge => ("radius_margin", "m"),
            HazardClass::Overcurrent => ("current", "A"),
            HazardClass::ThreadFailure => ("elastic_error", "m"),
            HazardClass::Retention => ("holding_deficit", "N"),
            HazardClass::ConnectionFailure => ("wrench_utilization", "1"),
            HazardClass::Impact => ("normal_impulse", "N*s"),
            HazardClass::Pinch => ("opposed_load", "N"),
            HazardClass::ContactStructure => ("normal_load", "N"),
            HazardClass::DangerousMotion => ("closing_speed", "m/s"),
            HazardClass::Fatigue => ("reversed_stress", "Pa"),
            _ => return None,
        };
        let channel = format!("{subject}/{quantity}");
        let metric_report = self.metrics.iter().find(|metric| {
            metric.name == criterion
                && metric.unit == unit
                && matches!(&metric.series, Series::Measurement(name) if name == &channel)
        })?;
        let upper = if matches!(
            class,
            HazardClass::Retention
                | HazardClass::ConnectionFailure
                | HazardClass::Impact
                | HazardClass::Pinch
                | HazardClass::ContactStructure
                | HazardClass::DangerousMotion
                | HazardClass::Fatigue
        ) {
            let limit = metric_report.input_bounds.get("upper")?;
            if metric_report.input_bounds.contains_key("lower")
                || metric_report.interval?[0] <= limit.range[2]
            {
                return None;
            }
            Some(limit)
        } else {
            None
        };
        let mut findings = self.campaign_findings(run);
        let index = findings.iter().position(|f| {
            f.key.class == HazardClass::TaskCriterion
                && f.code == criterion
                && f.outcome == Outcome::Fail
                && f.measurement.is_some()
                && f.limit.is_some()
        })?;
        let mut metric = findings.swap_remove(index);
        let audit_for = |trial: &TrialReport| {
            trial.hazards.iter().position(|hazard| {
                hazard.class == class
                    && hazard.subject == subject
                    && hazard.outcome == Outcome::Fail
                    && hazard.sources_confirmed
                    && upper.is_none_or(|limit| hazard.sources.contains(limit))
                    && (self.window_ns[0]..=self.window_ns[1]).contains(&hazard.at_ns)
            })
        };
        if self.trials.iter().any(|trial| audit_for(trial).is_none()) {
            return None;
        }
        let (trial, modeled) = self.trials.iter().find_map(|trial| {
            (trial.journal.map(Fingerprint::from_fnv1a) == metric.journal)
                .then(|| audit_for(trial).map(|index| (trial, &trial.hazards[index])))
                .flatten()
        })?;
        metric.key.class = class;
        metric.key.object = subject.into();
        metric.group = class.spec().group;
        metric.event_ns = Some(modeled.at_ns);
        metric.journal = trial.journal.map(Fingerprint::from_fnv1a);
        metric.reason.clone_from(&modeled.reason);
        metric
            .measurement_sources
            .extend(modeled.sources.iter().map(|source| source.source.clone()));
        metric.verification = class.spec().independent_check.into();
        metric.confirmed_hazard = true;
        Some(metric)
    }

    pub fn campaign_findings(&self, run: &RunKey) -> Vec<Finding> {
        let key = |class, object: String| RequirementKey {
            class,
            object,
            extension: None,
            run: Some(run.clone()),
            window_ns: Some(self.window_ns),
            level: Level::Simulation,
        };
        let mut result = Vec::new();
        for metric in &self.metrics {
            let mut finding = Finding::open(
                &key(HazardClass::TaskCriterion, metric.name.clone()),
                metric.detail.clone(),
            );
            finding.code = metric.name.clone();
            finding.outcome = metric.outcome;
            if !metric.input_bounds.contains_key("lower")
                && !metric.input_bounds.contains_key("upper")
                && metric.creep_interval.is_none()
            {
                finding.outcome = finding.outcome.max(Outcome::NotAnalyzed);
            }
            finding.measurement = metric.interval;
            let range = |id| {
                metric
                    .input_bounds
                    .get(id)
                    .map(|b| [b.range[0], b.range[2]])
            };
            let bounds = LimitValue {
                lower: range("lower"),
                upper: range("upper"),
            };
            if bounds.lower.is_some() || bounds.upper.is_some() {
                finding.limit = Some(bounds);
            }
            finding.unit = Some(metric.unit.clone());
            finding.model_scope = Some(self.window_source.clone());
            finding.limit_sources.extend(
                metric
                    .input_bounds
                    .iter()
                    .filter(|(k, _)| *k == "lower" || *k == "upper")
                    .map(|(_, p)| p.source.clone()),
            );
            finding.measurement_sources.extend(
                metric
                    .input_bounds
                    .iter()
                    .filter(|(k, _)| *k != "lower" && *k != "upper")
                    .map(|(_, p)| p.source.clone()),
            );
            finding
                .measurement_sources
                .push(metric.tolerance.source.clone());
            finding.coverage_gaps.extend(self.missing.iter().cloned());
            for trial in &self.trials {
                finding.coverage_gaps.extend(trial.missing.iter().cloned());
            }
            finding.journal = self
                .trials
                .get(2)
                .and_then(|t| t.journal)
                .map(Fingerprint::from_fnv1a);
            finding.event_ns = metric
                .maxima_at_ns
                .as_ref()
                .and_then(|v| v.get(2))
                .copied()
                .flatten();
            result.push(finding);
        }
        for trial in &self.trials {
            for audit in &trial.hazards {
                let mut finding = Finding::open(
                    &key(audit.class, audit.subject.clone()),
                    audit.reason.clone(),
                );
                finding.outcome = audit.outcome;
                if audit.class == HazardClass::Ingress && finding.outcome == Outcome::Pass {
                    finding.outcome = Outcome::NotAnalyzed;
                    finding.missing_data.push("требуется заверенный замер этой ревизии".into());
                }
                if finding.outcome == Outcome::Pass
                    && !self.trials.iter().all(|trial| {
                        trial
                            .covered_hazards
                            .contains(&(audit.subject.clone(), audit.class))
                    })
                {
                    finding.outcome = Outcome::NotAnalyzed;
                    finding
                        .coverage_gaps
                        .push("класс не покрыт на всех завершённых интервалах и сетках".into());
                }
                finding.event_ns = Some(audit.at_ns);
                finding.journal = trial.journal.map(Fingerprint::from_fnv1a);
                finding.model_scope = Some(self.window_source.clone());
                finding
                    .limit_sources
                    .extend(audit.sources.iter().map(|p| p.source.clone()));
                finding
                    .coverage_gaps
                    .extend(self.missing.iter().chain(&trial.missing).cloned());
                result.push(finding);
            }
            for reason in &trial.violations {
                let mut finding = Finding::open(
                    &key(HazardClass::Execution, "выполнение".into()),
                    reason.clone(),
                );
                finding.outcome = Outcome::Fail;
                finding.journal = trial.journal.map(Fingerprint::from_fnv1a);
                finding.coverage_gaps.extend(trial.missing.iter().cloned());
                result.push(finding);
            }
        }
        result
    }
}

#[derive(Clone)]
struct Curve(Vec<(Time, f64)>);

struct Trial {
    report: TrialReport,
    curves: Vec<Option<Curve>>,
    stamp: Option<Stamp>,
}

struct Stamp {
    setup: u64,
    scene: u64,
    seed: u64,
    initial: Vec<u64>,
    solver: SolverProfile,
    contact_settings: String,
    hazards: Vec<String>,
    mode: BTreeMap<[String; 2], Vec<bool>>,
    modeled_contacts: bool,
    creep_supported: BTreeSet<String>,
}

fn named(text: &str) -> bool {
    !text.trim().is_empty() && !text.chars().any(char::is_control)
}

fn valid_range(range: &SourcedRange) -> Result<(), Error> {
    range.validate().map_err(|e| invalid(e.to_string()))
}

fn nonnegative(range: &SourcedRange) -> Result<(), Error> {
    valid_range(range)?;
    if range.min < 0.0 {
        return Err(invalid("отрицательная граница погрешности"));
    }
    Ok(())
}

impl ConvergencePlan {
    pub fn fingerprint(&self) -> Fingerprint {
        Fingerprint::of(format!("физические сетки/v1:{self:?}").as_bytes())
    }

    fn validate(&self) -> Result<[u64; 3], Error> {
        let rates = [
            Some(self.base_hz),
            self.base_hz.checked_mul(2),
            self.base_hz.checked_mul(4),
        ];
        let [Some(h), Some(half), Some(quarter)] = rates else {
            return Err(invalid("переполнение частоты физики"));
        };
        if Rate::hz(quarter).is_none()
            || h == 0
            || self.window.start >= self.window.end
            || !named(&self.window_source)
            || self.criteria.is_empty()
            || self
                .variants
                .first()
                .is_none_or(|v| v.kind != VariantKind::Baseline)
            || self
                .variants
                .iter()
                .skip(1)
                .any(|v| v.kind == VariantKind::Baseline)
        {
            return Err(invalid("неполная серия или неверное окно сходимости"));
        }
        let mut names = BTreeSet::new();
        for v in &self.variants {
            if !named(&v.name)
                || !names.insert(&v.name)
                || v.materials.keys().ne(self.material_ranges.keys())
                || v.materials.values().any(|value| !value.is_finite())
            {
                return Err(invalid("повтор варианта или неверные параметры материалов"));
            }
            for (name, range) in &self.material_ranges {
                valid_range(range)?;
                let p = v.materials[name];
                if !named(name)
                    || p < range.min
                    || p > range.max
                    || (v.kind != VariantKind::Material && p != range.nominal)
                {
                    return Err(invalid(
                        "параметры материала вне заданного диапазона или варианта",
                    ));
                }
            }
        }
        names.clear();
        for c in &self.criteria {
            if !named(&c.name) || !named(&c.unit) || !names.insert(&c.name) {
                return Err(invalid("повтор или неверное имя критерия"));
            }
            match &c.series {
                Series::NormalForce([a, b]) | Series::Penetration([a, b])
                    if !named(a) || !named(b) || a == b =>
                {
                    return Err(invalid("неверная пара геометрий критерия"));
                }
                Series::Measurement(name) if !named(name) => {
                    return Err(invalid("неверное имя измерения"));
                }
                _ => (),
            }
            for bound in [
                &c.tolerance,
                &c.remainder.discretization,
                &c.remainder.solver,
                &c.remainder.materials,
            ] {
                nonnegative(bound)?;
            }
            if let Some(limit) = &c.lower {
                valid_range(limit)?;
            }
            if let Some(limit) = &c.upper {
                valid_range(limit)?;
            }
            if let (Some(lower), Some(upper)) = (&c.lower, &c.upper)
                && lower.max > upper.min
            {
                return Err(invalid("нижний и верхний пределы перекрываются"));
            }
        }
        names.clear();
        for c in &self.creep {
            if !names.insert(&c.metric)
                || !c.support_origin_m.into_iter().all(f64::is_finite)
                || !self.criteria.iter().any(|m| {
                    m.name == c.metric
                        && matches!(m.series, Series::Displacement(_))
                        && matches!(m.statistic, Statistic::AbsolutePeak | Statistic::End)
                        && m.unit == "m"
                })
            {
                return Err(invalid(
                    "ползучесть требует отдельной метрики смещения в метрах",
                ));
            }
            nonnegative(&c.limit_m)?;
        }
        for e in &self.events {
            if !named(&e.name)
                || !named(&e.state)
                || e.applied_window.start > e.applied_window.end
                || e.applied_window.start < self.window.start
                || e.applied_window.end > self.window.end
            {
                return Err(invalid("недопустимое окно или имя события"));
            }
            nonnegative(&e.phase_tolerance_ns)?;
        }
        names.clear();
        for state in &self.states {
            if !named(&state.name)
                || !names.insert(&state.name)
                || state.sequence.is_empty()
                || state.sequence.iter().any(|value| !named(value))
            {
                return Err(invalid(
                    "недопустимое описание последовательности состояний",
                ));
            }
        }
        Ok([h, half, quarter])
    }
}

/// Выполняет все сетки на свежих экземплярах. Сборщик пересобирает сцену с
/// запрошенной частотой, используя один состав, начальное состояние и вводы.
pub fn assess_processes(
    plan: &ConvergencePlan,
    build: impl FnMut(Rate, &TrialVariant) -> Result<Runtime, String>,
) -> Result<ConvergenceReport, Error> {
    assess_processes_observed(plan, build, |_, _, _| {})
}

/// Предоставляет завершённый прогон для сверок с внешними рядами.
pub fn assess_processes_observed(
    plan: &ConvergencePlan,
    build: impl FnMut(Rate, &TrialVariant) -> Result<Runtime, String>,
    observe: impl FnMut(Rate, &TrialVariant, &Runtime),
) -> Result<ConvergenceReport, Error> {
    assess_processes_with_snapshot(plan, None, build, observe)
}

pub fn assess_processes_with_snapshot(
    plan: &ConvergencePlan,
    snapshot_at: Option<Time>,
    mut build: impl FnMut(Rate, &TrialVariant) -> Result<Runtime, String>,
    mut observe: impl FnMut(Rate, &TrialVariant, &Runtime),
) -> Result<ConvergenceReport, Error> {
    let rates = plan.validate()?;
    let mut trials = Vec::new();
    for variant in &plan.variants {
        for rate in rates {
            let frequency = Rate::hz(rate).expect("частота проверена");
            let trial = build(frequency, variant).and_then(|rt| {
                if rt.physics_rate() != frequency || !rt.process_run_is_fresh() {
                    return Err("сборщик вернул чужую сетку или уже исполненный прогон".into());
                }
                Ok(rt)
            });
            trials.push(collect(plan, variant, rate, trial, snapshot_at, |rt| {
                observe(frequency, variant, rt);
            })?);
        }
    }

    let mut missing = compare_trials(plan, &trials);
    let metrics: Vec<_> = plan
        .criteria
        .iter()
        .enumerate()
        .map(|(i, criterion)| compare_metric(plan, criterion, i, &trials))
        .collect::<Result<_, _>>()?;
    if !plan
        .criteria
        .iter()
        .any(|criterion| criterion.lower.is_some() || criterion.upper.is_some())
        && plan.creep.is_empty()
        && plan.events.is_empty()
        && plan.states.is_empty()
    {
        missing.push("нет условий успешности процесса".into());
    }
    let outcome = Outcome::aggregate(
        metrics
            .iter()
            .map(|m| m.outcome)
            .chain(trials.iter().map(|t| {
                if !t.report.violations.is_empty() {
                    Outcome::Fail
                } else if t.report.missing.is_empty() {
                    Outcome::Pass
                } else {
                    Outcome::NotAnalyzed
                }
            }))
            .chain((!missing.is_empty()).then_some(Outcome::NotAnalyzed)),
    );
    let mut report = ConvergenceReport {
        outcome,
        window_ns: [plan.window.start.0, plan.window.end.0],
        window_source: plan.window_source.clone(),
        material_ranges: plan
            .material_ranges
            .iter()
            .map(|(n, v)| (n.clone(), v.into()))
            .collect(),
        trials: trials.into_iter().map(|t| t.report).collect(),
        metrics,
        missing,
        seal: None,
        profile: plan.fingerprint(),
    };
    report.seal = Some(report.digest());
    Ok(report)
}

fn collect(
    plan: &ConvergencePlan,
    variant: &TrialVariant,
    rate_hz: u64,
    runtime: Result<Runtime, String>,
    snapshot_at: Option<Time>,
    observe: impl FnOnce(&Runtime),
) -> Result<Trial, Error> {
    let mut trial = Trial {
        report: TrialReport {
            variant: variant.name.clone(),
            rate_hz,
            journal: None,
            steps: 0,
            completed: false,
            snapshot: None,
            solver: None,
            materials: variant.materials.clone(),
            events: Vec::new(),
            states: BTreeMap::new(),
            missing: Vec::new(),
            violations: Vec::new(),
            hazards: Vec::new(),
            covered_hazards: BTreeSet::new(),
        },
        curves: vec![None; plan.criteria.len()],
        stamp: None,
    };
    let mut rt = match runtime {
        Ok(rt) => rt,
        Err(e) => {
            trial.report.missing.push(e);
            return Ok(trial);
        }
    };
    if let Some(at) = snapshot_at {
        let audit = rt.audit_snapshot(at, plan.window.end);
        let reason = audit.err();
        trial.report.missing.extend(reason.iter().cloned());
        trial.report.snapshot = Some(crate::verdict::SnapshotAudit {
            at_ns: at.0,
            matched: reason.is_none(),
            reason,
        });
    }
    let outcome = rt.run_until(plan.window.end);
    if outcome == Ok(Status::Running) && rt.log().end().time == plan.window.end {
        observe(&rt);
    }
    let end = rt.log().end();
    let journal = rt.process_journal();
    let report = &mut trial.report;
    report.journal = Some(rt.log().fingerprint());
    report.steps = end.steps;
    report.completed =
        outcome == Ok(Status::Running) && end.time == plan.window.end && end.steps != 0;
    report.solver = Some(format!("{:?}", rt.process_solver_profile()));
    if !report.completed {
        report.missing.push(format!(
            "физика не завершила расчёт: {:?} {:?}",
            outcome,
            rt.failure()
        ));
    }
    if rt.process_mass_is_unverified() {
        report
            .missing
            .push("нет проверяемого источника массы или инерции".into());
    }
    for limitation in rt
        .simplifications()
        .iter()
        .filter(|l| l.blocks_proof && !covered_contact_marker(l, plan, &rt))
    {
        report
            .missing
            .push(format!("{}: {}", limitation.subject, limitation.text));
    }
    if let Some(run) = sim_runtime::control::run(&rt) {
        if run.origin() != sim_runtime::control::RunOrigin::Live || run.has_external_processes() {
            report
                .missing
                .push("происхождение или окружение контроллера не подтверждено".into());
        }
        if run.events().iter().any(|e| matches!(&e.kind, sim_runtime::control::EventKind::BoardOutput { event, .. } if matches!(event.output, sim_runtime::control::BoardEvent::Reset(_)))) {
            report.missing.push("контроллер сброшен".into());
        }
    }

    let start = journal.iter().position(|row| row.at == plan.window.start);
    let finish = journal.iter().position(|row| row.at == plan.window.end);
    let window = start
        .zip(finish)
        .and_then(|(start, end)| journal.get(start..=end))
        .filter(|rows| rows.len() > 1);
    let requirements = rt.process_requirements();
    let Some(rows) = window.filter(|rows| complete(rows)) else {
        assess_hazards(requirements.unwrap_or(&[]), journal, &[], report);
        report.completed = false;
        report
            .missing
            .push("нет сплошного журнала физических интервалов в общем окне".into());
        return Ok(trial);
    };
    let initial = journal
        .first()
        .filter(|row| row.at == Time::ZERO)
        .ok_or_else(|| invalid("отсутствует начальное состояние серии"))?;
    if requirements.is_none() {
        report
            .missing
            .push("не зафиксирован состав и обязательные опасности".into());
    }
    let missing: BTreeSet<_> = journal
        .iter()
        .filter_map(|row| row.completed.as_ref())
        .flat_map(|frame| frame.contact_gaps.iter())
        .collect();
    report.missing.extend(missing.into_iter().cloned());
    let missing: BTreeSet<_> = journal
        .iter()
        .flat_map(|row| {
            row.missing_models
                .iter()
                .map(|name| format!("для «{name}» модель не установлена"))
                .chain(row.outputs.iter().flat_map(|(name, out)| {
                    out.blockers
                        .iter()
                        .map(move |reason| format!("«{name}»: {reason}"))
                }))
        })
        .collect();
    report.missing.extend(missing);
    assess_hazards(requirements.unwrap_or(&[]), journal, rows, report);
    assess_events(plan, rows, report);
    assess_states(plan, rows, report);
    for (i, criterion) in plan.criteria.iter().enumerate() {
        let known_pair = match &criterion.series {
            Series::NormalForce(pair) | Series::Penetration(pair) => rt
                .process_contact_pairs()
                .any(|p| ordered(p.pair.geometries.clone()) == ordered(pair.clone())),
            _ => false,
        };
        trial.curves[i] = make_curve(rows, criterion, known_pair)?;
        if trial.curves[i].is_none() {
            report
                .missing
                .push(format!("неполные измерения «{}»", criterion.name));
        }
    }
    trial.stamp = Some(Stamp {
        setup: rt.process_sampling_fingerprint(),
        scene: rt.process_scene_fingerprint(),
        seed: rt.seed(),
        initial: initial
            .qpos
            .iter()
            .chain(&initial.qvel)
            .map(|x| x.to_bits())
            .collect(),
        solver: rt.process_solver_profile(),
        contact_settings: rt
            .process_contact_pairs()
            .map(|p| format!("{:?}{:?}{:?}", p.materials, p.parameters, p.calibration))
            .collect(),
        hazards: requirements
            .unwrap_or(&[])
            .iter()
            .map(|h| format!("{}/{:?}", h.subject, h.kind))
            .collect(),
        mode: contact_modes(rows),
        modeled_contacts: rt.process_contact_pairs().next().is_some()
            || rows.iter().any(|row| {
                row.completed
                    .as_ref()
                    .is_some_and(|frame| !frame.reactions.contacts.is_empty())
            }),
        creep_supported: plan
            .creep
            .iter()
            .filter(|c| {
                c.fixture_fingerprint == rt.process_scene_fingerprint()
                    && rt.matches_creep_configuration(&c.configuration)
                    && plan.criteria.iter().any(|criterion| {
                        criterion.name == c.metric
                            && matches!(criterion.series, Series::Displacement(idx)
                                if rt.creep_displacement_geometry(idx, &c.configuration.pair.geometries)
                                    .is_some_and(|geom| creep_load_matches(c, rows, &geom)))
                    })
            })
            .map(|c| c.metric.clone())
            .collect(),
    });
    Ok(trial)
}

fn creep_load_matches(check: &CreepCheck, rows: &[ProcessTick], moving: &str) -> bool {
    let tolerances = check.report.tolerances();
    let expected_force = check.support.required_force_n;
    let expected_moment = check.support.required_moment_nm;
    let pair = &check.configuration.pair.geometries;
    rows[1..].iter().all(|row| {
        let reactions = &row.completed.as_ref().expect("сплошной журнал").reactions;
        let mut force = [0.0; 3];
        let mut moment = [0.0; 3];
        for c in reactions.contacts.iter().filter(|c| {
            c.separation_m <= 0.0
                && (c.geometries == *pair || c.geometries.iter().eq(pair.iter().rev()))
        }) {
            let sign = if c.geometries[1] == moving { 1.0 } else { -1.0 };
            // CreepReport относится к плоской опоре в XY с нормалью +Z.
            if c.normal[0] != 0.0 || c.normal[1] != 0.0 || sign * c.normal[2] <= 0.0 {
                return false;
            }
            let [fx, fy, fz] = c.force_n.map(|f| sign * f);
            let [x, y, z] = std::array::from_fn(|i| c.point_m[i] - check.support_origin_m[i]);
            for (i, cross) in [y * fz - z * fy, z * fx - x * fz, x * fy - y * fx]
                .into_iter()
                .enumerate()
            {
                force[i] += sign * c.force_n[i];
                moment[i] += cross + sign * c.torque_nm[i];
            }
        }
        let [dx, dy, dz] = std::array::from_fn(|i| force[i] - expected_force[i]);
        let [mx, my, mz] = std::array::from_fn::<_, 3, _>(|i| moment[i] - expected_moment[i]);
        force[2] > 0.0
            && dz.abs() <= tolerances.normal_balance_n.min
            && dx.hypot(dy) <= tolerances.tangent_balance_n.min
            && mx.hypot(my).hypot(mz) <= tolerances.moment_balance_nm.min
    })
}

fn covered_contact_marker(
    limitation: &sim_world::Simplification,
    plan: &ConvergencePlan,
    rt: &Runtime,
) -> bool {
    // Общая метка сборщика остаётся блокирующей в одиночном контроле.
    // Здесь допускается только при полном численном покрытии установленных пар.
    if limitation.subject != "contact"
        || limitation.text
            != "Явные пары MuJoCo: применимость, сходимость и погрешность ползучести требуют численного доказательства"
        || rt.process_contact_pairs().next().is_none()
    {
        return false;
    }
    rt.process_contact_pairs().all(|pair| {
        let key = ordered(pair.pair.geometries.clone());
        let statistic = |statistic| plan.criteria.iter().any(|criterion|
            matches!(&criterion.series, Series::NormalForce(id) if ordered(id.clone()) == key)
                && criterion.statistic == statistic);
        pair.supports_nominal_proof()
            && statistic(Statistic::Maximum)
            && statistic(Statistic::Integral)
            && plan.criteria.iter().any(|criterion|
                matches!(&criterion.series, Series::Penetration(id) if ordered(id.clone()) == key)
                    && criterion.statistic == Statistic::Maximum)
            // Ползучесть относится к конкретной нагрузке. Для нескольких
            // фрикционных пар единый замер не покрывает взаимодействие.
            && (pair.pair.settings.dim == 1 || (rt.process_contact_pairs().count() == 1 && !plan.creep.is_empty()))
    })
}

fn complete(rows: &[ProcessTick]) -> bool {
    rows.windows(2).all(|pair| {
        pair[1].completed.as_ref().is_some_and(|frame| {
            frame.interval.start == pair[0].at
                && frame.interval.end == pair[1].at
                && frame.reactions.at == pair[1].at
        })
    })
}

fn assess_hazards(
    required: &[sim_process::work::HazardEvidence],
    journal: &[ProcessTick],
    rows: &[ProcessTick],
    report: &mut TrialReport,
) {
    let mut gaps = BTreeSet::new();
    let mut violations = BTreeSet::new();
    let mut audits = BTreeMap::new();
    let is_in_window = |at| {
        rows.first()
            .zip(rows.last())
            .is_some_and(|(start, end)| (start.at..=end.at).contains(&at))
    };
    for row in journal {
        for hazard in row.outputs.values().flat_map(|out| &out.hazards) {
            let (outcome, reason) = match &hazard.check {
                HazardCheck::Violation { reason } => (Outcome::Fail, reason),
                HazardCheck::WithinLimit { scope }
                    if !hazard.sources.is_empty()
                        && hazard.sources.iter().all(SourcedRange::can_prove) =>
                {
                    (Outcome::Pass, scope)
                }
                HazardCheck::WithinLimit { scope } => (Outcome::NotAnalyzed, scope),
                HazardCheck::NotAnalyzed { reason } => (Outcome::NotAnalyzed, reason),
            };
            let class = HazardClass::from(hazard.kind);
            // Сохраняем первое наблюдение каждой причины. Числа оставляем в журнале.
            let id = (
                hazard.subject.as_str(),
                class,
                outcome,
                reason.as_str(),
                is_in_window(row.at),
            );
            audits.entry(id).or_insert_with(|| HazardReport {
                subject: hazard.subject.clone(),
                class,
                at_ns: row.at.0,
                outcome,
                reason: reason.clone(),
                sources: hazard.sources.iter().map(ParameterAudit::from).collect(),
                sources_confirmed: !hazard.sources.is_empty()
                    && hazard.sources.iter().all(SourcedRange::can_prove),
            });
            if let HazardCheck::Violation { reason } = &hazard.check {
                violations.insert(format!(
                    "{} / {:?}, {} нс: {reason}",
                    hazard.subject, hazard.kind, row.at.0
                ));
            }
        }
        for attachment in row
            .outputs
            .values()
            .flat_map(|out| out.attachment_records())
        {
            if attachment.failed
                || matches!(
                    attachment.cause,
                    Some(
                        sim_process::attachments::AttachmentCause::Overload
                            | sim_process::attachments::AttachmentCause::Stripped
                    )
                )
            {
                let reason = "подтверждено разрушение крепления";
                let id = (
                    attachment.name.as_str(),
                    HazardClass::ConnectionFailure,
                    Outcome::Fail,
                    reason,
                    is_in_window(row.at),
                );
                audits.entry(id).or_insert_with(|| HazardReport {
                    subject: attachment.name.clone(),
                    class: HazardClass::ConnectionFailure,
                    at_ns: row.at.0,
                    outcome: Outcome::Fail,
                    reason: reason.into(),
                    sources: vec![],
                    sources_confirmed: false,
                });
                violations.insert(format!(
                    "разрушение крепления «{}» в {} нс",
                    attachment.name, row.at.0
                ));
            }
            if attachment.event.as_ref().is_some_and(|event| {
                !matches!(
                    event.outcome,
                    sim_process::attachments::AttachmentOutcome::Transition { .. }
                )
            }) {
                gaps.insert(format!(
                    "крепление «{}»: запрос не подтверждён",
                    attachment.name
                ));
            }
        }
    }
    if rows.len() > 1 {
        let mut covered: BTreeSet<_> = required
            .iter()
            .map(|h| (h.subject.clone(), HazardClass::from(h.kind)))
            .collect();
        for row in rows.iter().skip(1) {
            let mut bounded: BTreeMap<(String, HazardClass), bool> = BTreeMap::new();
            for hazard in row.outputs.values().flat_map(|out| &out.hazards) {
                let within = matches!(hazard.check, HazardCheck::WithinLimit { .. })
                    && !hazard.sources.is_empty()
                    && hazard.sources.iter().all(SourcedRange::can_prove);
                *bounded
                    .entry((hazard.subject.clone(), hazard.kind.into()))
                    .or_insert(true) &= within;
            }
            covered.retain(|key| bounded.get(key) == Some(&true));
        }
        report.covered_hazards = covered;
    }
    // Левая граница содержит исходную позу. Опасности от завершённых
    // интервалов оцениваются на их правых границах.
    for row in rows.iter().skip(1) {
        for h in required {
            let checks: Vec<_> = row
                .outputs
                .values()
                .flat_map(|out| &out.hazards)
                .filter(|x| h.subject == x.subject && h.kind == x.kind)
                .collect();
            if checks.is_empty()
                || checks.iter().any(|check| {
                    matches!(check.check, HazardCheck::NotAnalyzed { .. })
                        || (matches!(check.check, HazardCheck::WithinLimit { .. })
                            && (check.sources.is_empty()
                                || check.sources.iter().any(|s| !s.can_prove())))
                })
            {
                gaps.insert(format!("неполное покрытие «{}» / {:?}", h.subject, h.kind));
            }
        }
    }
    report.hazards.extend(audits.into_values());
    report.missing.extend(gaps);
    report.violations.extend(violations);
}

fn assess_events(plan: &ConvergencePlan, rows: &[ProcessTick], report: &mut TrialReport) {
    let actual: Vec<&ProcessEvent> = rows
        .iter()
        .flat_map(|row| row.outputs.values())
        .flat_map(|out| &out.events)
        .collect();
    report.events.extend(actual.iter().map(|e| EventReport {
        name: e.name.clone(),
        state: e.state.clone(),
        requested_ns: e.requested.0,
        applied_ns: e.applied.0,
        phase_ns: e.applied.0 - e.requested.0,
    }));
    if actual.len() != plan.events.len() {
        report
            .missing
            .push("пропущено или добавлено событие".into());
    }
    for (actual, expected) in actual.iter().zip(&plan.events) {
        if actual.name != expected.name
            || actual.state != expected.state
            || expected
                .requested
                .is_some_and(|requested| requested != actual.requested)
            || actual.applied < expected.applied_window.start
            || actual.applied > expected.applied_window.end
            || (actual.applied.0 - actual.requested.0) as f64 > expected.phase_tolerance_ns.min
            || !expected.phase_tolerance_ns.can_prove()
        {
            report.missing.push(format!(
                "событие «{}»: иной порядок, состояние или фаза",
                expected.name
            ));
        }
    }
}

fn assess_states(plan: &ConvergencePlan, rows: &[ProcessTick], report: &mut TrialReport) {
    let mut count = BTreeMap::new();
    for row in rows {
        for (name, state) in row.outputs.values().flat_map(|out| &out.states) {
            let sequence = report.states.entry(name.clone()).or_default();
            if sequence.last() != Some(state) {
                sequence.push(state.clone());
            }
            *count.entry(name).or_insert(0) += 1;
        }
    }
    for (name, count) in count {
        if count != rows.len() {
            report
                .missing
                .push(format!("разрыв журнала состояний «{name}»"));
        }
    }
    for state in &plan.states {
        if report.states.get(&state.name) != Some(&state.sequence) {
            report
                .missing
                .push(format!("иная последовательность «{}»", state.name));
        }
    }
}

fn contact_modes(rows: &[ProcessTick]) -> BTreeMap<[String; 2], Vec<bool>> {
    let sets: Vec<BTreeSet<_>> = rows[1..]
        .iter()
        .map(|row| {
            row.completed
                .as_ref()
                .unwrap()
                .reactions
                .contacts
                .iter()
                .filter(|c| c.force_n != [0.0; 3])
                .map(|c| ordered(c.geometries.clone()))
                .collect()
        })
        .collect();
    let names: BTreeSet<_> = sets.iter().flatten().cloned().collect();
    names
        .into_iter()
        .map(|name| {
            let mut sequence: Vec<_> = sets.iter().map(|set| set.contains(&name)).collect();
            sequence.dedup();
            (name, sequence)
        })
        .collect()
}

fn ordered(mut pair: [String; 2]) -> [String; 2] {
    pair.sort();
    pair
}

fn make_curve(
    rows: &[ProcessTick],
    criterion: &Criterion,
    known_pair: bool,
) -> Result<Option<Curve>, Error> {
    let initial = &rows[0].qpos;
    let force = matches!(criterion.series, Series::NormalForce(_));
    let contact = matches!(
        criterion.series,
        Series::NormalForce(_) | Series::Penetration(_)
    );
    let series = match &criterion.series {
        Series::NormalForce(pair) | Series::Penetration(pair) => {
            let pair = ordered(pair.clone());
            let mut points = Vec::new();
            let mut seen = false;
            for row in &rows[1..] {
                let frame: &PhysicsFrame = row.completed.as_ref().expect("сплошной журнал");
                let mut total: f64 = 0.0;
                for item in frame.reactions.contacts.iter().filter(|c| {
                    c.geometries == pair
                        || (c.geometries[0] == pair[1] && c.geometries[1] == pair[0])
                }) {
                    seen = true;
                    if force {
                        let normal = item
                            .normal
                            .iter()
                            .zip(item.force_n)
                            .map(|(a, b)| a * b)
                            .sum::<f64>();
                        total += normal;
                    } else {
                        total = total.max((-item.separation_m).max(0.0));
                    }
                }
                points.push((frame.interval.start, total));
            }
            if !seen && !known_pair {
                return Ok(None);
            }
            points.push((rows.last().unwrap().at, points.last().unwrap().1));
            Some(points)
        }
        _ => rows
            .iter()
            .map(|row| {
                let value = match &criterion.series {
                    Series::Measurement(name) => {
                        let unit = if criterion.statistic == Statistic::Integral {
                            criterion.unit.strip_suffix("*s")?
                        } else {
                            &criterion.unit
                        };
                        let item = row
                            .outputs
                            .values()
                            .flat_map(|o| &o.measurements)
                            .find(|m| m.signal.name == *name && m.signal.unit == unit)?;
                        match item.result {
                            Analysis::Analyzed { value, .. } => value,
                            _ => return None,
                        }
                    }
                    Series::Position(i) => *row.qpos.get(*i)?,
                    Series::Velocity(i) => *row.qvel.get(*i)?,
                    Series::Displacement(i) => row.qpos.get(*i)? - initial.get(*i)?,
                    _ => unreachable!(),
                };
                Some((row.at, value))
            })
            .collect(),
    };
    if contact {
        let expected = match (force, criterion.statistic) {
            (true, Statistic::Integral) => "N*s",
            (true, _) => "N",
            (false, Statistic::Integral) => "m*s",
            (false, _) => "m",
        };
        if criterion.unit != expected {
            return Err(invalid("неверная единица контакта"));
        }
    }
    if series
        .as_ref()
        .is_some_and(|points| points.iter().any(|(_, v)| !v.is_finite()))
    {
        return Err(invalid("нечисловая метрика процесса"));
    }
    Ok(series.map(Curve))
}

fn compare_trials(plan: &ConvergencePlan, trials: &[Trial]) -> Vec<String> {
    let mut missing = BTreeSet::new();
    let Some(base) = trials[0].stamp.as_ref() else {
        return vec!["базовая серия не содержит завершённой сетки".into()];
    };
    let base_report = &trials[0].report;
    if base.mode.is_empty()
        && plan
            .criteria
            .iter()
            .any(|c| matches!(c.series, Series::NormalForce(_) | Series::Penetration(_)))
    {
        missing.insert("отсутствует режим с ненулевой реакцией контакта".into());
    }
    if base.modeled_contacts && !plan.variants.iter().any(|v| v.kind == VariantKind::Solver) {
        missing.insert("нет отдельной серии параметров решателя".into());
    }
    for (variant, group) in plan.variants.iter().zip(trials.as_chunks::<3>().0) {
        let Some(first) = group[0].stamp.as_ref() else {
            continue;
        };
        let solver_differs = first.solver != base.solver;
        let materials_differ = first.contact_settings != base.contact_settings;
        if (variant.kind == VariantKind::Solver) != solver_differs
            || (variant.kind != VariantKind::Material && materials_differ)
            || (variant.kind == VariantKind::Material && base.modeled_contacts && !materials_differ)
        {
            missing.insert(format!(
                "вариант «{}» не отделяет решатель от параметров контакта",
                variant.name
            ));
        }
        for trial in group {
            let Some(stamp) = &trial.stamp else {
                continue;
            };
            if stamp.setup != base.setup
                || stamp.scene != base.scene
                || stamp.seed != base.seed
                || stamp.initial != base.initial
            {
                missing.insert("начальные данные либо сетка ввода-вывода изменились".into());
            }
            if stamp.solver != first.solver || stamp.contact_settings != first.contact_settings {
                missing.insert(
                    "между шагами одной серии изменены параметры решателя или контакта".into(),
                );
            }
            if stamp.hazards != base.hazards {
                missing.insert("обязательные опасности отличаются между прогонами".into());
            }
            if stamp.mode != base.mode || trial.report.states != base_report.states {
                missing.insert("изменился порядок или исход режимов контакта и процессов".into());
            }
        }
        for adjacent in group.windows(2) {
            for ((coarse, fine), expected) in adjacent[0]
                .report
                .events
                .iter()
                .zip(&adjacent[1].report.events)
                .zip(&plan.events)
            {
                if coarse.applied_ns.abs_diff(fine.applied_ns) as f64
                    > expected.phase_tolerance_ns.min
                {
                    missing.insert(format!(
                        "фаза «{}» не устойчива между соседними сетками",
                        expected.name
                    ));
                }
            }
        }
        for ((current, nominal), expectation) in group[2]
            .report
            .events
            .iter()
            .zip(&trials[2].report.events)
            .zip(&plan.events)
        {
            if current.applied_ns.abs_diff(nominal.applied_ns) as f64
                > expectation.phase_tolerance_ns.min
            {
                missing.insert(format!(
                    "фаза «{}» не устойчива при изменении параметров",
                    expectation.name
                ));
            }
        }
    }
    for (name, range) in &plan.material_ranges {
        if !range.can_prove() {
            missing.insert(format!("неподтверждённый источник диапазона «{name}»"));
        }
        for endpoint in [range.min, range.max] {
            if endpoint != range.nominal
                && !plan
                    .variants
                    .iter()
                    .any(|v| v.kind == VariantKind::Material && v.materials[name] == endpoint)
            {
                missing.insert(format!(
                    "не исследована граница {endpoint} параметра «{name}»"
                ));
            }
        }
    }
    missing.into_iter().collect()
}

impl Curve {
    fn value(&self, statistic: Statistic) -> [f64; 2] {
        let min = self.0.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
        let max = self.0.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
        let scalar = match statistic {
            Statistic::Continuous => return [min, max],
            Statistic::Minimum => min,
            Statistic::Maximum => max,
            Statistic::AbsolutePeak => min.abs().max(max.abs()),
            Statistic::End => self.0.last().expect("непустая кривая").1,
            Statistic::Integral => self
                .0
                .windows(2)
                .map(|pair| pair[0].1 * Time(pair[1].0.0 - pair[0].0.0).as_secs())
                .sum(),
        };
        [scalar; 2]
    }

    fn delta(&self, other: &Self, statistic: Statistic) -> f64 {
        if statistic != Statistic::Continuous {
            return (self.value(statistic)[0] - other.value(statistic)[0]).abs();
        }
        // Обе ступенчатые кривые определены на общем окне. Максимум
        // разности ищется на всех границах обеих сеток.
        let mut index = [0, 0];
        let mut delta: f64 = 0.0;
        loop {
            let a = self.0[index[0]];
            let b = other.0[index[1]];
            delta = delta.max((a.1 - b.1).abs());
            let next_a = self.0.get(index[0] + 1).map(|p| p.0);
            let next_b = other.0.get(index[1] + 1).map(|p| p.0);
            let Some(next) = next_a.into_iter().chain(next_b).min() else {
                break;
            };
            index[0] += usize::from(next_a == Some(next));
            index[1] += usize::from(next_b == Some(next));
        }
        delta
    }
}

/// Снимает метрику процесса из полного окна уже исполненного рантайма.
pub fn sample_criterion(
    runtime: &Runtime,
    window: Interval,
    criterion: &Criterion,
) -> Result<Option<[f64; 2]>, Error> {
    let journal = runtime.process_journal();
    let Some(rows) = journal.iter().position(|row| row.at == window.start)
        .zip(journal.iter().position(|row| row.at == window.end))
        .and_then(|(first, last)| journal.get(first..=last))
        .filter(|rows| rows.len() > 1 && complete(rows))
    else {
        return Ok(None);
    };
    let known_pair = match &criterion.series {
        Series::NormalForce(pair) | Series::Penetration(pair) => runtime
            .process_contact_pairs()
            .any(|p| ordered(p.pair.geometries.clone()) == ordered(pair.clone())),
        _ => false,
    };
    let value = make_curve(rows, criterion, known_pair)?.map(|curve| curve.value(criterion.statistic));
    if value.is_some_and(|bounds| bounds.iter().any(|v| !v.is_finite())) {
        return Err(invalid("переполнение оценки метрики процесса"));
    }
    Ok(value)
}

fn compare_metric(
    plan: &ConvergencePlan,
    c: &Criterion,
    i: usize,
    trials: &[Trial],
) -> Result<MetricReport, Error> {
    let curves: Vec<_> = trials.iter().map(|t| t.curves[i].as_ref()).collect();
    let mut result = MetricReport {
        name: c.name.clone(),
        unit: c.unit.clone(),
        series: c.series.clone(),
        values: curves
            .iter()
            .map(|curve| curve.map(|p| p.value(c.statistic)))
            .collect(),
        tolerance: (&c.tolerance).into(),
        input_bounds: [
            ("discretization", Some(&c.remainder.discretization)),
            ("solver", Some(&c.remainder.solver)),
            ("materials", Some(&c.remainder.materials)),
            ("lower", c.lower.as_ref()),
            ("upper", c.upper.as_ref()),
        ]
        .into_iter()
        .filter_map(|(key, value)| value.map(|p| (key.into(), p.into())))
        .collect(),
        adjacent_differences: Vec::new(),
        observed_orders: Vec::new(),
        statistic: c.statistic,
        extrema_values: curves.iter().map(|curve| curve.map(|curve| curve.value(Statistic::Continuous))).collect(),
        extrema_at_ns: curves.iter().map(|curve| {
            let points = &curve.as_ref()?.0;
            let min = points.iter().min_by(|(_, a), (_, b)| a.total_cmp(b))?;
            let max = points.iter().rev().max_by(|(_, a), (_, b)| a.total_cmp(b))?;
            Some([min.0.0, max.0.0])
        }).collect(),
        maxima_at_ns: (c.statistic == Statistic::Maximum).then(|| {
            curves
                .iter()
                .map(|curve| {
                    curve.and_then(|curve| {
                        curve
                            .0
                            .iter()
                            .enumerate()
                            .max_by(|(ia, (_, a)), (ib, (_, b))| {
                                a.total_cmp(b).then_with(|| ib.cmp(ia))
                            })
                            .map(|(_, (at, _))| at.0)
                    })
                })
                .collect()
        }),
        discretization: None,
        solver: None,
        materials: None,
        interval: None,
        creep_interval: None,
        outcome: Outcome::NotAnalyzed,
        detail: String::new(),
    };
    for group in curves.as_chunks::<3>().0 {
        result.adjacent_differences.push(match group {
            [Some(a), Some(b), Some(d)] => Some([a.delta(b, c.statistic), b.delta(d, c.statistic)]),
            _ => None,
        });
    }
    if result
        .values
        .iter()
        .flatten()
        .flatten()
        .chain(result.adjacent_differences.iter().flatten().flatten())
        .any(|x| !x.is_finite())
    {
        return Err(invalid(format!("переполнение метрики «{}»", c.name)));
    }
    result.observed_orders = result
        .adjacent_differences
        .iter()
        .map(|d| {
            d.filter(|[a, b]| *a > 0.0 && *b > 0.0)
                .map(|[a, b]| a.log2() - b.log2())
        })
        .collect();
    let Some(all): Option<Vec<_>> = curves.into_iter().collect() else {
        result.detail = "неполное покрытие на трёх сетках".into();
        return Ok(result);
    };
    let outside_tolerance = result
        .adjacent_differences
        .iter()
        .flatten()
        .flatten()
        .any(|&d| d > c.tolerance.min);
    if [
        &c.tolerance,
        &c.remainder.discretization,
        &c.remainder.solver,
        &c.remainder.materials,
    ]
    .into_iter()
    .chain(c.lower.iter())
    .chain(c.upper.iter())
    .any(|r| !r.can_prove())
    {
        result.detail = if outside_tolerance {
            "OutsideTolerance: соседние сетки расходятся, источник допуска или остатка не подтверждён"
        } else {
            "источник допуска, порога или остаточной ошибки не подтверждён"
        }.into();
        return Ok(result);
    }
    let nominal = all[2];
    let disc = result
        .adjacent_differences
        .iter()
        .flatten()
        .map(|[a, b]| a + b)
        .fold(0.0, f64::max);
    let mut solver: f64 = 0.0;
    let mut materials: f64 = 0.0;
    for (v, group) in plan.variants.iter().zip(all.as_chunks::<3>().0) {
        match v.kind {
            VariantKind::Baseline => (),
            VariantKind::Solver => solver = solver.max(nominal.delta(group[2], c.statistic)),
            VariantKind::Material => {
                materials = materials.max(nominal.delta(group[2], c.statistic))
            }
        }
    }
    let disc = disc + c.remainder.discretization.max;
    let solver = solver + c.remainder.solver.max;
    let materials = materials + c.remainder.materials.max;
    let uncertainty = disc + solver + materials;
    let [lo, hi] = nominal.value(c.statistic);
    let mut interval = [lo - uncertainty, hi + uncertainty];
    if !interval.into_iter().all(f64::is_finite) {
        return Err(invalid(format!(
            "выход за численный диапазон метрики «{}»",
            c.name
        )));
    }
    result.discretization = Some(disc);
    result.solver = Some(solver);
    result.materials = Some(materials);
    result.interval = Some(interval);
    if outside_tolerance {
        result.detail = "OutsideTolerance: соседние сетки расходятся".into();
        return Ok(result);
    }

    let mut creep_outcome = Outcome::Pass;
    if let Some(check) = plan.creep.iter().find(|check| check.metric == c.name) {
        if check.report.hold != plan.window
            || trials[2]
                .stamp
                .as_ref()
                .is_none_or(|stamp| !stamp.creep_supported.contains(&c.name))
        {
            result.detail =
                "замер ползучести не подтверждён для окна, сцены, решателя, оси и фактической нагрузки".into();
            return Ok(result);
        }
        let absolute = if interval[0] <= 0.0 && interval[1] >= 0.0 {
            [0.0, interval[0].abs().max(interval[1].abs())]
        } else {
            [
                interval[0].abs().min(interval[1].abs()),
                interval[0].abs().max(interval[1].abs()),
            ]
        };
        if check.report.trials().any(|trial| {
            trial.variant() == sim_process::contact::CreepVariant::QuarterStep
                && trial.support() == &check.support
                && trial.configuration() == &check.configuration
        }) && let Analysis::Analyzed { value: error, .. } =
            check.report.numerical_uncertainty_m()
            && (absolute[1] + error).is_finite()
        {
            result.creep_interval = Some([(absolute[0] - error).max(0.0), absolute[1] + error]);
        }
        match check
            .report
            .compare_displacement_limit(
                &check.configuration,
                &check.support,
                absolute,
                &check.limit_m,
            )
            .map_err(|e| invalid(e.to_string()))?
        {
            Analysis::NotAnalyzed { reason } => {
                result.detail = reason;
                return Ok(result);
            }
            Analysis::Analyzed {
                value: (expanded, comparison),
                ..
            } => {
                result.creep_interval = Some(expanded);
                if comparison == LimitComparison::ExceedsLimit {
                    creep_outcome = Outcome::Fail;
                }
                interval = match c.statistic {
                    Statistic::AbsolutePeak => expanded,
                    _ => [-expanded[1], expanded[1]],
                };
            }
        }
    }

    let compare = |bound: Option<&SourcedRange>, value: f64, sign: f64| {
        let Some(bound) = bound else {
            return Outcome::Pass;
        };
        let near = if sign > 0.0 { bound.min } else { -bound.max };
        let far = if sign > 0.0 { bound.max } else { -bound.min };
        let value = sign * value;
        if value + uncertainty <= near {
            Outcome::Pass
        } else if value - uncertainty > far {
            Outcome::Fail
        } else {
            Outcome::NotAnalyzed
        }
    };
    let outcomes = if result.creep_interval.is_some() {
        [
            c.lower.as_ref().map_or(Outcome::Pass, |b| {
                if interval[0] >= b.max {
                    Outcome::Pass
                } else {
                    Outcome::NotAnalyzed
                }
            }),
            c.upper.as_ref().map_or(Outcome::Pass, |b| {
                if interval[1] <= b.min {
                    Outcome::Pass
                } else {
                    Outcome::NotAnalyzed
                }
            }),
        ]
    } else {
        [
            compare(c.lower.as_ref(), lo, -1.0),
            compare(c.upper.as_ref(), hi, 1.0),
        ]
    };
    result.outcome = Outcome::aggregate(outcomes.into_iter().chain([creep_outcome]));
    result.detail = match result.outcome {
        Outcome::Pass => "интервал в пределах критерия",
        Outcome::Fail => "нарушен предел",
        Outcome::NotAnalyzed => "интервал пересекает предел",
    }
    .into();
    Ok(result)
}
