//! Доказательство по журналам завершённых физических интервалов.

use std::collections::{BTreeMap, BTreeSet};

use serde::Serialize;
pub use sim_process::SourcedRange;
use sim_process::attachments::InterfaceState;
use sim_process::{Analysis, Interval};
use sim_runtime::{
    CommandSource, LogKind, Runtime, Status,
    control::{EventKind, FaultAction},
    process::interaction::{INTERACTION_VERSION, InteractionIdentity, PatchSample},
};
use sim_signal::Time;
use sim_world::Provenance;

use crate::campaign::RunKey;
use crate::verdict::{Finding, HazardClass, Level, LimitValue, RequirementKey};
use crate::{Error, Fingerprint, Outcome, invalid};

/// Единое правило свёртки для живого отчёта и конструктора.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum SafetyDecision {
    Rejected,
    NeedsWork,
    BenchOnly,
    Passed,
}

pub fn safety_decision(
    outcomes: impl IntoIterator<Item = Outcome>,
    construction_complete: bool,
) -> SafetyDecision {
    match Outcome::aggregate(outcomes) {
        Outcome::Fail => SafetyDecision::Rejected,
        _ if !construction_complete => SafetyDecision::NeedsWork,
        Outcome::NotAnalyzed => SafetyDecision::BenchOnly,
        Outcome::Pass => SafetyDecision::Passed,
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reach {
    OffParticipant,
    Reachable,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum EnergyPath {
    Electrical,
    Thermal,
    Mechanical,
}

impl EnergyPath {
    const ALL: [Self; 3] = [Self::Electrical, Self::Thermal, Self::Mechanical];
}

#[derive(Clone, Debug)]
pub struct EnergyBounds {
    /// Запас источника на пути при границах режима и отказа.
    pub stored_j: Option<SourcedRange>,
    /// Верхняя и нижняя границы энергии, доступной на этом пути при отказе.
    pub deliverable_j: Option<SourcedRange>,
    /// Граница опасной энергии с обоснованием пути и окна.
    pub hazardous_above_j: Option<SourcedRange>,
    /// Подтверждённое воздействие и допустимая граница для того же окна.
    /// Установленная сила защиты сама по себе такой парой не является.
    pub violation: Option<(SourcedRange, SourcedRange)>,
}

#[derive(Clone, Debug)]
pub struct PowerSource {
    pub reach: Reach,
    pub paths: BTreeMap<EnergyPath, EnergyBounds>,
}

#[derive(Clone, Debug)]
pub struct PowerInventory {
    pub reach: Reach,
    /// Независимое подтверждение полного инвентаря источников и связей.
    pub topology: Option<Provenance>,
    pub required_sources: BTreeSet<String>,
    pub sources: BTreeMap<String, PowerSource>,
}

#[derive(Clone, Debug, Serialize)]
pub struct PowerFinding {
    pub source: Option<String>,
    pub path: Option<String>,
    pub outcome: Outcome,
    pub confirmed_hazard: bool,
    pub explanation: String,
    pub audit: Option<EnergyAudit>,
}

#[derive(Clone, Debug, Serialize)]
pub struct EnergyAudit {
    pub stored_j: Option<[f64; 2]>,
    pub deliverable_j: Option<[f64; 2]>,
    pub hazardous_above_j: Option<[f64; 2]>,
    pub actual_j: Option<[f64; 2]>,
    pub allowed_j: Option<[f64; 2]>,
    pub input_sources: Vec<String>,
    pub limit_sources: Vec<String>,
}

impl EnergyAudit {
    fn read(bounds: &EnergyBounds) -> Self {
        let range = |r: &SourcedRange| r.validate().is_ok().then_some([r.min, r.max]);
        let sources = |ranges: &[Option<&SourcedRange>]| {
            ranges
                .iter()
                .flatten()
                .map(|r| format!("{:?}", r.provenance))
                .collect()
        };
        let exposure = bounds.violation.as_ref();
        Self {
            stored_j: bounds.stored_j.as_ref().and_then(range),
            deliverable_j: bounds.deliverable_j.as_ref().and_then(range),
            hazardous_above_j: bounds.hazardous_above_j.as_ref().and_then(range),
            actual_j: exposure.and_then(|(actual, _)| range(actual)),
            allowed_j: exposure.and_then(|(_, allowed)| range(allowed)),
            input_sources: sources(&[
                bounds.stored_j.as_ref(),
                bounds.deliverable_j.as_ref(),
                exposure.map(|(actual, _)| actual),
            ]),
            limit_sources: sources(&[
                bounds.hazardous_above_j.as_ref(),
                exposure.map(|(_, allowed)| allowed),
            ]),
        }
    }
}

/// Риск запаса/отдачи блокирует положительный вердикт независимо от остальных строк.
pub fn assess_power_inventory(inventory: &PowerInventory) -> Vec<PowerFinding> {
    let complete = source_is_proven(inventory.topology.as_ref())
        && inventory
            .sources
            .keys()
            .eq(inventory.required_sources.iter())
        && inventory.required_sources.iter().all(|name| named(name))
        && inventory.reach != Reach::Unknown
        && !(inventory.reach == Reach::OffParticipant
            && inventory
                .sources
                .values()
                .any(|source| source.reach != Reach::OffParticipant));
    let mut result = vec![PowerFinding {
        source: None,
        path: None,
        outcome: if complete {
            Outcome::Pass
        } else {
            Outcome::NotAnalyzed
        },
        confirmed_hazard: false,
        audit: None,
        explanation: if complete {
            "инвентарь питания покрыт"
        } else {
            "нет полного подтверждённого инвентаря питания и достижимых связей"
        }
        .into(),
    }];
    for (name, source) in &inventory.sources {
        if source.reach == Reach::OffParticipant && complete {
            continue;
        }
        for path in EnergyPath::ALL {
            let bounds = source.paths.get(&path);
            let (outcome, confirmed_hazard, explanation) = compare_energy(bounds);
            result.push(PowerFinding {
                source: Some(name.clone()),
                path: Some(format!("{path:?}")),
                outcome: if source.reach == Reach::Unknown && outcome == Outcome::Pass {
                    Outcome::NotAnalyzed
                } else {
                    outcome
                },
                confirmed_hazard,
                audit: bounds.map(EnergyAudit::read),
                explanation,
            });
        }
    }
    let reachable = inventory
        .sources
        .values()
        .filter(|s| s.reach != Reach::OffParticipant || !complete)
        .collect::<Vec<_>>();
    if reachable.len() > 1 {
        for path in EnergyPath::ALL {
            let total = reachable.iter().try_fold(
                ([0.0; 2], [0.0; 2], [f64::INFINITY; 2]),
                |(s, d, l), source| {
                    let bounds = source.paths.get(&path)?;
                    let stored = bounds.stored_j.as_ref().filter(|r| proved_nonnegative(r))?;
                    let deliverable = bounds
                        .deliverable_j
                        .as_ref()
                        .filter(|r| proved_nonnegative(r))?;
                    let limit = bounds
                        .hazardous_above_j
                        .as_ref()
                        .filter(|r| proved_nonnegative(r))?;
                    Some((
                        [s[0] + stored.min, s[1] + stored.max],
                        [d[0] + deliverable.min, d[1] + deliverable.max],
                        [l[0].min(limit.min), l[1].min(limit.max)],
                    ))
                },
            );
            let bounded = total.is_some_and(|(s, d, l)| s[1] <= l[0] && d[1] <= l[0]);
            let hazard = total.is_some_and(|(s, d, l)| {
                [s[0], d[0]].into_iter().any(|v| v.is_finite() && v > l[1])
            });
            result.push(PowerFinding {
                source: None,
                path: Some(format!("{path:?}")),
                outcome: if bounded {
                    Outcome::Pass
                } else {
                    Outcome::NotAnalyzed
                },
                confirmed_hazard: hazard,
                audit: total
                    .filter(|(s, d, l)| s.iter().chain(d).chain(l).all(|v| v.is_finite()))
                    .map(|(stored, deliverable, limit)| EnergyAudit {
                        stored_j: Some(stored),
                        deliverable_j: Some(deliverable),
                        hazardous_above_j: Some(limit),
                        actual_j: None,
                        allowed_j: None,
                        input_sources: reachable
                            .iter()
                            .flat_map(|s| s.paths.get(&path))
                            .flat_map(|b| EnergyAudit::read(b).input_sources)
                            .collect(),
                        limit_sources: reachable
                            .iter()
                            .flat_map(|s| s.paths.get(&path))
                            .flat_map(|b| EnergyAudit::read(b).limit_sources)
                            .collect(),
                    }),
                explanation: format!(
                    "сумма запасов и отдачи достижимых источников против строгой границы: {total:?}"
                ),
            });
        }
    }
    result
}

fn compare_energy(bounds: Option<&EnergyBounds>) -> (Outcome, bool, String) {
    let Some(bounds) = bounds else {
        return (Outcome::NotAnalyzed, false, "нет границ для пути".into());
    };
    let stored = bounds.stored_j.as_ref().filter(|x| proved_nonnegative(x));
    let delivered = bounds
        .deliverable_j
        .as_ref()
        .filter(|x| proved_nonnegative(x));
    let limit = bounds
        .hazardous_above_j
        .as_ref()
        .filter(|x| proved_nonnegative(x));
    let hazard = limit.is_some_and(|limit| {
        stored
            .into_iter()
            .chain(delivered)
            .any(|energy| energy.min > limit.max)
    });
    let violation = bounds.violation.as_ref().is_some_and(|(actual, allowed)| {
        proved_nonnegative(actual) && proved_nonnegative(allowed) && actual.min > allowed.max
    });
    let bounded = stored
        .zip(delivered)
        .zip(limit)
        .is_some_and(|((s, d), l)| s.max <= l.min && d.max <= l.min);
    let exposure_covered = bounds.violation.as_ref().is_none_or(|(actual, allowed)| {
        proved_nonnegative(actual) && proved_nonnegative(allowed) && actual.max <= allowed.min
    });
    let (outcome, message) = if violation {
        (
            Outcome::Fail,
            "подтверждено превышение допустимого воздействия",
        )
    } else if hazard {
        (Outcome::NotAnalyzed, "подтверждена опасная энергия питания")
    } else if bounded && exposure_covered {
        (Outcome::Pass, "запас и доступная отдача ограничены")
    } else {
        (
            Outcome::NotAnalyzed,
            "нет обоснованной безопасной границы энергии или воздействия",
        )
    };
    (
        outcome,
        hazard,
        format!(
            "{message}. Запас: {:?}. Отдача: {:?}. Предел: {:?}. Воздействие: {:?}",
            bounds.stored_j, bounds.deliverable_j, bounds.hazardous_above_j, bounds.violation
        ),
    )
}

#[derive(Clone, Debug)]
pub struct InteractionPlan {
    pub build: Fingerprint,
    pub legacy_design: Option<u64>,
    pub construction_complete: bool,
    pub grids: [InteractionIdentity; 3],
    pub variant: String,
    pub window: Interval,
    /// Полный перечень участков, ожидаемых в этой постановке.
    pub regions: BTreeSet<String>,
    pub patches: BTreeMap<String, PatchPlan>,
    pub tolerances: ContactTolerances,
    pub limits: Vec<LimitCriterion>,
    pub power: PowerInventory,
    pub power_loss: Option<PowerLossPlan>,
    pub discrepancies: Vec<ReferenceDiscrepancy>,
}

#[derive(Clone, Debug)]
pub struct PatchPlan {
    pub geometry: String,
    pub mode: String,
    pub population: String,
    pub repetitions: u32,
    /// Отсутствие нагружения допустимо только со ссылкой на проверку достижимости.
    pub absence_basis: Option<Provenance>,
}

#[derive(Clone, Debug)]
pub struct ContactTolerances {
    pub onset_s: SourcedRange,
    pub force_n: SourcedRange,
    pub impulse_ns: SourcedRange,
    pub duration_s: SourcedRange,
    pub drift_m: SourcedRange,
    pub stress_pa: SourcedRange,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub enum ContactMetric {
    NormalForce,
    TangentialForce,
    MeanPressure,
    MeanShear,
}

impl ContactMetric {
    pub fn unit(self) -> &'static str {
        match self {
            Self::NormalForce | Self::TangentialForce => "N",
            Self::MeanPressure | Self::MeanShear => "Pa",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub enum CriterionKind {
    Exposure,
    Design,
}

#[derive(Clone, Debug)]
pub struct LimitCriterion {
    pub patch: String,
    pub region: String,
    pub metric: ContactMetric,
    pub kind: CriterionKind,
    pub limit: SourcedRange,
    /// Отдельный источник применимости критерия к модели или замеру.
    pub validation: Option<Provenance>,
    pub geometry: String,
    pub mode: String,
    pub population: String,
    pub duration_s: SourcedRange,
    pub max_repetitions: u32,
    pub area_m2: Option<SourcedRange>,
}

#[derive(Clone, Debug)]
pub struct PowerLossPlan {
    pub id: String,
    pub target: String,
    pub at: Time,
    pub loaded_patch: String,
    pub witness: Option<PowerWitness>,
}

#[derive(Clone, Debug)]
pub struct PowerWitness {
    /// Истинный сигнал физической модели питания, зарегистрированный сборщиком.
    pub signal: String,
    pub unit: String,
    pub before: SourcedRange,
    pub after: SourcedRange,
}

#[derive(Clone, Debug, Serialize)]
pub struct ReferenceDiscrepancy {
    pub id: String,
    pub mode: String,
    pub simulated: f64,
    pub reference: f64,
    pub unit: String,
    pub source: String,
    pub tolerance: String,
    pub note: String,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct ContactMetrics {
    /// Правая граница первого нагруженного интервала в данном окне.
    pub onset_ns: Option<u64>,
    pub peak_normal_n: f64,
    /// Нижняя граница по результирующей и верхняя по сумме модулей.
    pub peak_shear_n: [f64; 2],
    pub peak_point_shear_n: f64,
    pub peak_mean_pressure_pa: Option<[f64; 2]>,
    pub peak_mean_shear_pa: Option<[f64; 2]>,
    pub impulse_ns: f64,
    pub duration_s: f64,
    pub peak_drift_m: Option<f64>,
}

#[derive(Clone, Debug, Serialize)]
pub struct GridSummary {
    pub world: Fingerprint,
    pub journal: Fingerprint,
    pub rate_hz: f64,
    pub patches: BTreeMap<String, ContactMetrics>,
    pub regions: BTreeMap<String, ContactMetrics>,
    pub routes: BTreeMap<String, ContactMetrics>,
    pub coverage_gaps: Vec<String>,
    pub power_loss_applied_ns: Option<u64>,
}

#[derive(Clone, Debug, Serialize)]
pub struct GridDifference {
    pub quantity: &'static str,
    pub unit: &'static str,
    pub differences: [Option<f64>; 2],
    pub tolerance: Option<f64>,
    pub outcome: Outcome,
}

#[derive(Clone, Debug, Serialize)]
pub struct ExposureFinding {
    pub patch: String,
    pub region: String,
    pub variant: String,
    pub window_ns: [u64; 2],
    pub metric: ContactMetric,
    pub kind: CriterionKind,
    pub measurement: Option<[f64; 2]>,
    pub limit: Option<[f64; 2]>,
    pub unit: &'static str,
    pub outcome: Outcome,
    pub source: String,
    pub applicability: String,
    pub coverage_gaps: Vec<String>,
    pub event_ns: Option<u64>,
    pub event_journal: Option<Fingerprint>,
}

#[derive(Debug, Serialize)]
pub struct InteractionEvidence {
    version: u32,
    plan: Fingerprint,
    build: Fingerprint,
    legacy_design: Option<u64>,
    profile: Fingerprint,
    variant: String,
    grids: Vec<GridSummary>,
    comparisons: BTreeMap<String, Vec<GridDifference>>,
    findings: Vec<ExposureFinding>,
    power: Vec<PowerFinding>,
    discrepancies: Vec<ReferenceDiscrepancy>,
    blockers: Vec<String>,
    outcome: Outcome,
    decision: SafetyDecision,
    #[serde(skip)]
    window_ns: [u64; 2],
}

impl InteractionEvidence {
    pub(crate) fn campaign_build(&self) -> Fingerprint {
        self.build
    }
    pub(crate) fn campaign_profile(&self) -> Fingerprint {
        self.plan
    }

    pub fn campaign_findings(&self, run: &RunKey) -> Vec<Finding> {
        let key = |class, object: String, window_ns| RequirementKey {
            class,
            object,
            extension: None,
            run: Some(run.clone()),
            window_ns: Some(window_ns),
            level: Level::Simulation,
        };
        let mut result = Vec::new();
        let journal = self.grids.last().map(|g| g.journal);
        for item in &self.findings {
            let class = match item.kind {
                CriterionKind::Design => HazardClass::ContactStructure,
                CriterionKind::Exposure => HazardClass::TissueLoad,
            };
            let mut finding = Finding::open(
                &key(class, item.patch.clone(), item.window_ns),
                item.applicability.clone(),
            );
            finding.code = format!("{}/{:?}", item.region, item.metric);
            finding.outcome = item.outcome;
            finding.measurement = item.measurement;
            finding.limit = item.limit.map(LimitValue::upper);
            finding.unit = Some(item.unit.into());
            finding.limit_sources.push(item.source.clone());
            finding.model_scope = Some(item.applicability.clone());
            finding.journal = item.event_journal.or(journal);
            finding.event_ns = item.event_ns;
            finding
                .coverage_gaps
                .extend(item.coverage_gaps.iter().cloned());
            finding.confirmed_hazard = item.outcome == Outcome::Fail;
            result.push(finding);
        }
        for item in &self.power {
            let object = item.source.as_deref().unwrap_or("совокупность");
            let mut finding = Finding::open(
                &key(HazardClass::StoredEnergy, object.into(), self.window_ns),
                item.explanation.clone(),
            );
            finding.code = item.path.clone().unwrap_or_else(|| "инвентарь".into());
            finding.outcome = item.outcome;
            finding.journal = journal;
            if let Some(audit) = &item.audit {
                finding.measurement = audit.actual_j.or(audit.deliverable_j);
                finding.limit = audit
                    .allowed_j
                    .or(audit.hazardous_above_j)
                    .map(LimitValue::upper);
                finding.unit = Some("J".into());
                finding.measurement_sources.clone_from(&audit.input_sources);
                finding.limit_sources.clone_from(&audit.limit_sources);
            }
            finding.confirmed_hazard = item.confirmed_hazard && item.outcome == Outcome::Fail;
            result.push(finding);
        }
        if !self.blockers.is_empty() {
            let mut finding = Finding::open(
                &key(
                    HazardClass::Execution,
                    "взаимодействие".into(),
                    self.window_ns,
                ),
                "ограничения доказательства взаимодействия",
            );
            finding.coverage_gaps.clone_from(&self.blockers);
            finding.journal = journal;
            result.push(finding);
        }
        result
    }

    pub fn matches(&self, expected: &InteractionPlan, legacy_design: u64) -> bool {
        self.version == INTERACTION_VERSION
            && self.plan == expected.fingerprint()
            && self.legacy_design == Some(legacy_design)
    }

    pub fn outcome(&self) -> Outcome {
        self.outcome
    }
    pub fn decision(&self) -> SafetyDecision {
        self.decision
    }
    pub fn grids(&self) -> &[GridSummary] {
        &self.grids
    }
    pub fn comparisons(&self) -> &BTreeMap<String, Vec<GridDifference>> {
        &self.comparisons
    }
    pub fn findings(&self) -> &[ExposureFinding] {
        &self.findings
    }
    pub fn power(&self) -> &[PowerFinding] {
        &self.power
    }
    pub fn blockers(&self) -> &[String] {
        &self.blockers
    }
    pub fn discrepancies(&self) -> &[ReferenceDiscrepancy] {
        &self.discrepancies
    }
}

impl InteractionPlan {
    pub fn fingerprint(&self) -> Fingerprint {
        Fingerprint::of(format!("{INTERACTION_VERSION}:{self:?}").as_bytes())
    }

    fn validate(&self) -> Result<(), Error> {
        if !named(&self.variant)
            || self.window.start >= self.window.end
            || self.regions.iter().any(|r| !named(r))
            || self.patches.iter().any(|(p, plan)| {
                !named(p)
                    || !named(&plan.geometry)
                    || !named(&plan.mode)
                    || !named(&plan.population)
                    || plan.repetitions == 0
            })
        {
            return Err(invalid(
                "недопустимое имя, описание или окно связанного опыта",
            ));
        }
        let mut seen = BTreeSet::new();
        for criterion in &self.limits {
            if !seen.insert((
                &criterion.patch,
                &criterion.region,
                criterion.metric,
                criterion.kind,
            )) {
                return Err(invalid("повторён критерий нагрузки"));
            }
        }
        let t = &self.tolerances;
        let tolerances = [
            &t.onset_s,
            &t.force_n,
            &t.impulse_ns,
            &t.duration_s,
            &t.drift_m,
            &t.stress_pa,
        ];
        let criteria = self.limits.iter().flat_map(|c| {
            [&c.limit, &c.duration_s]
                .into_iter()
                .chain(c.area_m2.as_ref())
        });
        let energies = self
            .power
            .sources
            .values()
            .flat_map(|s| s.paths.values())
            .flat_map(|e| {
                [&e.stored_j, &e.deliverable_j, &e.hazardous_above_j]
                    .into_iter()
                    .flatten()
                    .chain(
                        e.violation
                            .iter()
                            .flat_map(|(actual, limit)| [actual, limit]),
                    )
            });
        let witnesses = self
            .power_loss
            .iter()
            .filter_map(|f| f.witness.as_ref())
            .flat_map(|w| [&w.before, &w.after]);
        if tolerances
            .into_iter()
            .chain(criteria)
            .chain(energies)
            .chain(witnesses)
            .any(|r| [r.min, r.nominal, r.max].iter().any(|v| !v.is_finite()))
        {
            return Err(invalid(
                "нечисловой или бесконечный параметр доказательства",
            ));
        }
        for d in &self.discrepancies {
            if ![d.simulated, d.reference].into_iter().all(f64::is_finite)
                || !named(&d.tolerance)
                || !named(&d.id)
                || !named(&d.source)
            {
                return Err(invalid("невалидная эталонная сверка"));
            }
        }
        if let Some(fault) = &self.power_loss
            && (!named(&fault.id)
                || !named(&fault.target)
                || !named(&fault.loaded_patch)
                || fault.at <= self.window.start
                || fault.at >= self.window.end)
        {
            return Err(invalid("событие питания должно находиться внутри окна"));
        }
        Ok(())
    }
}

/// Односеточная диагностика. Для положительного доказательства нужны три сетки.
pub fn assess_interaction(
    rt: &Runtime,
    plan: &InteractionPlan,
) -> Result<InteractionEvidence, Error> {
    assess(&[rt], plan)
}

pub fn assess_interaction_grids(
    runs: [&Runtime; 3],
    plan: &InteractionPlan,
) -> Result<InteractionEvidence, Error> {
    assess(&runs, plan)
}

fn assess(runs: &[&Runtime], plan: &InteractionPlan) -> Result<InteractionEvidence, Error> {
    plan.validate()?;
    let mut blockers = BTreeSet::new();
    let mut audits = Vec::new();
    if !plan.construction_complete {
        blockers.insert("неполное описание конструкции".into());
    }
    for check in &plan.discrepancies {
        blockers.insert(format!(
            "{}: нет отдельного подтверждения для вывода «{}» при известном расхождении",
            check.id, check.mode
        ));
    }
    let mut equivalent = runs.len() == plan.grids.len();
    let primary = plan.grids[0];
    for (i, rt) in runs.iter().enumerate() {
        let expected = plan.grids[i];
        if expected.version != INTERACTION_VERSION
            || rt.interaction_identity() != Some(expected)
            || rt.interaction_is_replay()
        {
            blockers.insert(format!(
                "сетка {}: другая версия, сборка, профиль, зерно или воспроизведение",
                i + 1
            ));
            equivalent = false;
            audits.push(None);
            continue;
        }
        let first = runs[0];
        // Удвоение частоты в f64 точно сохраняет показатель у конечных частот Rate.
        if expected.profile != primary.profile
            || expected.variant != primary.variant
            || expected.seed != primary.seed
            || (i > 0 && expected.rate.as_hz() != 2.0 * plan.grids[i - 1].rate.as_hz())
            || rt.process_scene_fingerprint() != first.process_scene_fingerprint()
            || rt.process_solver_profile() != first.process_solver_profile()
            || rt.process_sampling_fingerprint() != first.process_sampling_fingerprint()
            || !external_commands(rt).eq(external_commands(first))
        {
            equivalent = false;
        }
        audits.push(Some(audit_grid(rt, plan)));
    }
    if !equivalent {
        blockers.insert("нет сопоставимых живых прогонов на трёх вложенных сетках".into());
    }
    for audit in audits.iter().flatten() {
        blockers.extend(audit.summary.coverage_gaps.iter().cloned());
    }
    if plan.power_loss.is_some()
        && (audits
            .iter()
            .flatten()
            .any(|a| a.summary.power_loss_applied_ns.is_none())
            || audits
                .iter()
                .flatten()
                .map(|a| a.summary.power_loss_applied_ns)
                .collect::<BTreeSet<_>>()
                .len()
                != 1)
    {
        blockers.insert("не подтверждено одинаковое отключение питания под нагрузкой".into());
    }
    let mut comparisons = BTreeMap::new();
    let keys = audits
        .iter()
        .flatten()
        .flat_map(|a| {
            a.summary
                .patches
                .keys()
                .map(|k| ("patch", k))
                .chain(a.summary.regions.keys().map(|k| ("region", k)))
                .chain(a.summary.routes.keys().map(|k| ("route", k)))
        })
        .collect::<BTreeSet<_>>();
    for (category, id) in keys {
        let metrics = |audit: &GridAudit| {
            let mapping = match category {
                "region" => &audit.summary.regions,
                "route" => &audit.summary.routes,
                _ => &audit.summary.patches,
            };
            mapping.get(id).cloned()
        };
        let values: [Option<ContactMetrics>; 3] =
            std::array::from_fn(|i| audits.get(i).and_then(Option::as_ref).and_then(metrics));
        let row = compare_contact(
            values.each_ref().map(Option::as_ref),
            &plan.tolerances,
            equivalent,
        );
        if row.iter().any(|c| c.outcome != Outcome::Pass) {
            blockers.insert(format!("не подтверждена сходимость: {id}"));
        }
        comparisons.insert(format!("{category}/{id}"), row);
    }
    let mut findings = Vec::new();
    for criterion in &plan.limits {
        findings.push(compare_limit(
            criterion,
            plan,
            &audits,
            &comparisons,
            equivalent,
        ));
    }
    // Отсутствующий тканевый критерий нельзя закрыть проектным пределом.
    if plan.regions.is_empty() && plan.power.reach != Reach::OffParticipant {
        blockers.insert("нет подтверждённого перечня участков".into());
    }
    for (patch, binding) in runs
        .iter()
        .find_map(|rt| rt.interaction_patches())
        .into_iter()
        .flatten()
    {
        if plan.patches.contains_key(patch) {
            for region in binding.iter().flatten() {
                for metric in [
                    ContactMetric::NormalForce,
                    ContactMetric::MeanPressure,
                    ContactMetric::MeanShear,
                ] {
                    if !findings.iter().any(|f| {
                        f.patch == *patch
                            && f.region == *region
                            && f.kind == CriterionKind::Exposure
                            && f.metric == metric
                    }) {
                        blockers.insert(format!(
                            "нет предела {metric:?} для пятна «{patch}», участок «{region}»"
                        ));
                    }
                }
            }
        }
    }
    let mut inventory = plan.power.clone();
    let mut missing_drives = Vec::new();
    if plan.power.reach != Reach::OffParticipant || !plan.regions.is_empty() {
        let drives: BTreeSet<_> = runs
            .iter()
            .flat_map(|rt| rt.interaction_drive_names())
            .collect();
        if !drives.is_empty() {
            blockers.insert(
                "нет подтверждённых границ рисков приводного механизма на участнике".into(),
            );
        }
        if inventory.reach == Reach::OffParticipant {
            inventory.reach = Reach::Unknown;
        }
        // В этом контракте нет подтверждения изоляции отдельного источника
        // от доступного участника. Текстовое объявление не исключает путь.
        for source in inventory.sources.values_mut() {
            if source.reach == Reach::OffParticipant {
                source.reach = Reach::Unknown;
            }
        }
        for drive in drives {
            if !inventory.required_sources.contains(drive) {
                missing_drives.push(PowerFinding {
                    source: Some(drive.clone()),
                    path: None,
                    outcome: Outcome::NotAnalyzed,
                    confirmed_hazard: false,
                    audit: None,
                    explanation: "привод из сборки пропущен в инвентаре".into(),
                });
            }
        }
    }
    let mut power = assess_power_inventory(&inventory);
    power.extend(missing_drives);
    let outcome = Outcome::aggregate(
        findings
            .iter()
            .map(|f| f.outcome)
            .chain(power.iter().map(|f| f.outcome))
            .chain((!blockers.is_empty()).then_some(Outcome::NotAnalyzed)),
    );
    Ok(InteractionEvidence {
        version: INTERACTION_VERSION,
        plan: plan.fingerprint(),
        build: plan.build,
        legacy_design: plan.legacy_design,
        profile: Fingerprint::from_fnv1a(primary.profile),
        variant: plan.variant.clone(),
        grids: audits.into_iter().flatten().map(|a| a.summary).collect(),
        comparisons,
        findings,
        power,
        discrepancies: plan.discrepancies.clone(),
        blockers: blockers.into_iter().collect(),
        outcome,
        decision: safety_decision([outcome], plan.construction_complete),
        window_ns: [plan.window.start.0, plan.window.end.0],
    })
}

struct GridAudit {
    summary: GridSummary,
    complete: bool,
    binding: BTreeMap<String, [Option<String>; 2]>,
    areas: BTreeMap<String, SourcedRange>,
    invalid_patches: BTreeSet<String>,
    peaks_at: BTreeMap<String, BTreeMap<ContactMetric, u64>>,
}

#[derive(Default)]
struct Accumulator {
    metrics: ContactMetrics,
    offset_start: Option<[f64; 2]>,
    missing_area: bool,
    peaks_at: BTreeMap<ContactMetric, u64>,
}

impl Accumulator {
    fn add(&mut self, interval: Interval, normal: f64, shear: [f64; 2], point_shear: f64) {
        let m = &mut self.metrics;
        if normal > m.peak_normal_n {
            self.peaks_at
                .insert(ContactMetric::NormalForce, interval.end.0);
        }
        if shear[0] > m.peak_shear_n[0] {
            self.peaks_at
                .insert(ContactMetric::TangentialForce, interval.end.0);
        }
        m.peak_normal_n = m.peak_normal_n.max(normal);
        m.peak_shear_n = max_pair(m.peak_shear_n, shear);
        m.peak_point_shear_n = m.peak_point_shear_n.max(point_shear);
        if normal > 0.0 || shear[1] > 0.0 {
            m.onset_ns.get_or_insert(interval.end.0);
            let dt = Time(interval.end.0 - interval.start.0).as_secs();
            m.impulse_ns += normal * dt;
            m.duration_s += dt;
        }
    }

    fn patch(&mut self, sample: &PatchSample, normal: f64, shear: [f64; 2], point_shear: f64) {
        self.add(sample.interval, normal, shear, point_shear);
        let [a, b] = *self.offset_start.get_or_insert(sample.offset_m);
        let drift = (sample.offset_m[0] - a).hypot(sample.offset_m[1] - b);
        let m = &mut self.metrics;
        m.peak_drift_m = Some(m.peak_drift_m.map_or(drift, |prior| prior.max(drift)));
        if normal == 0.0 && shear[1] == 0.0 {
            return;
        }
        if let Some(area) = sample
            .area_m2
            .as_ref()
            .filter(|a| proved_nonnegative(a) && a.min > 0.0)
        {
            let pressure = [normal / area.max, normal / area.min];
            let shear = [shear[0] / area.max, shear[1] / area.min];
            if m.peak_mean_pressure_pa
                .is_none_or(|previous| pressure[0] > previous[0])
            {
                self.peaks_at
                    .insert(ContactMetric::MeanPressure, sample.interval.end.0);
            }
            if m.peak_mean_shear_pa
                .is_none_or(|previous| shear[0] > previous[0])
            {
                self.peaks_at
                    .insert(ContactMetric::MeanShear, sample.interval.end.0);
            }
            m.peak_mean_pressure_pa = Some(max_pair(
                m.peak_mean_pressure_pa.unwrap_or([0.0; 2]),
                pressure,
            ));
            m.peak_mean_shear_pa = Some(max_pair(m.peak_mean_shear_pa.unwrap_or([0.0; 2]), shear));
        } else {
            self.missing_area = true;
        }
    }

    fn finite(&self) -> bool {
        let m = &self.metrics;
        [
            m.peak_normal_n,
            m.peak_point_shear_n,
            m.impulse_ns,
            m.duration_s,
        ]
        .into_iter()
        .chain(m.peak_shear_n)
        .chain(m.peak_mean_pressure_pa.into_iter().flatten())
        .chain(m.peak_mean_shear_pa.into_iter().flatten())
        .chain(m.peak_drift_m)
        .all(f64::is_finite)
    }
}

#[derive(Default)]
struct RegionLoad {
    normal: f64,
    shear: [f64; 3],
    shear_upper: f64,
    point_shear: f64,
    peak_drift_m: Option<f64>,
    pressure: Option<[f64; 2]>,
    shear_pa: Option<[f64; 2]>,
}

fn audit_grid(rt: &Runtime, plan: &InteractionPlan) -> GridAudit {
    let mut patch_metrics = BTreeMap::<_, Accumulator>::new();
    let mut region_metrics = BTreeMap::<_, Accumulator>::new();
    let mut regions_without_area = BTreeSet::new();
    let mut route_metrics = BTreeMap::<_, Accumulator>::new();
    let mut gaps = BTreeSet::new();
    let mut invalid_patches = BTreeSet::new();
    let mut areas = BTreeMap::new();
    let binding = rt.interaction_patches().cloned().unwrap_or_default();
    let covered: BTreeSet<_> = binding.values().flatten().flatten().cloned().collect();
    if rt.interaction_regions() != Some(&plan.regions) {
        gaps.insert("реестр областей сборки отличается от задания".into());
    }
    for region in plan.regions.symmetric_difference(&covered) {
        gaps.insert(format!("участок «{region}» расходится с покрытием сборки"));
    }
    if !binding.keys().eq(plan.patches.keys()) {
        gaps.insert("набор пятен сборки отличается от задания".into());
    }
    for item in rt.simplifications().iter().filter(|s| s.blocks_proof) {
        gaps.insert(format!("{}: {}", item.subject, item.text));
    }
    if rt.process_mass_is_unverified() {
        gaps.insert("масса или инерция не имеет подтверждённого источника".into());
    }
    let mut right = plan.window.start;
    let mut complete = true;
    if rt.status() != Status::Running {
        gaps.insert(format!("прогон остановлен: {:?}", rt.failure()));
    }
    let mut loaded = BTreeMap::new();
    let mut unloaded = BTreeSet::new();
    let mut response_completed = false;
    let mut response_interval = None;
    for tick in rt.process_journal() {
        let in_window = tick.at >= plan.window.start && tick.at <= plan.window.end;
        let interaction = tick.outputs.values().find(|out| out.interaction.is_some());
        let previous_response = response_interval.take();
        if let Some(out) = interaction.filter(|_| in_window) {
            gaps.extend(out.blockers.iter().cloned());
            let data = out.interaction.as_ref().unwrap();
            response_interval = data
                .axes
                .iter()
                .find(|axis| axis.response_effort_delta != 0.0)
                .map(|axis| axis.interval);
            gaps.extend(data.coverage_gaps.iter().cloned());
            for (name, step) in &data.interfaces {
                if step.interval.start < plan.window.end
                    && matches!(
                        step.state,
                        InterfaceState::Sliding | InterfaceState::Released | InterfaceState::Broken
                    )
                {
                    gaps.insert(format!(
                        "интерфейс «{name}»: {:?}, удержание требует отдельного подтверждения",
                        step.state
                    ));
                }
            }
        }
        let Some(completed) = &tick.completed else {
            continue;
        };
        let interval = completed.interval;
        if interval.end <= plan.window.start || interval.start >= plan.window.end {
            continue;
        }
        if interval.start != right || interval.end > plan.window.end || interval.end != tick.at {
            complete = false;
            continue;
        }
        right = interval.end;
        let Some(data) = interaction.and_then(|o| o.interaction.as_ref()) else {
            complete = false;
            continue;
        };
        // Подтверждаем изменение усилия по реакции в завершённой физике.
        response_completed |= previous_response == Some(interval);
        for (name, route) in &data.completed_routes {
            match &route.reaction {
                Analysis::Analyzed { value, .. }
                    if route.interval == interval && value.tension_n.is_finite() =>
                {
                    let acc = route_metrics.entry(name.clone()).or_default();
                    acc.add(interval, value.tension_n, [0.0; 2], 0.0);
                    if let Some(payout) = route.payout_m {
                        let base = *acc.offset_start.get_or_insert([payout, 0.0]);
                        let drift = (payout - base[0]).abs();
                        let old = acc.metrics.peak_drift_m;
                        acc.metrics.peak_drift_m = Some(old.map_or(drift, |d| d.max(drift)));
                    }
                }
                _ => {
                    gaps.insert(format!("трасса «{name}» потеряла завершённую реакцию"));
                }
            }
        }
        let mut regions = BTreeMap::<_, RegionLoad>::new();
        for (patch, expected) in &binding {
            let Some(sample) = data.patches.get(patch) else {
                invalid_patches.insert(patch.clone());
                continue;
            };
            if &sample.regions != expected || sample.interval != interval || !sample.calibrated {
                invalid_patches.insert(patch.clone());
            }
            let Analysis::Analyzed { value: v, .. } = &sample.reaction else {
                invalid_patches.insert(patch.clone());
                continue;
            };
            let shear = [norm(v.shear_n), v.shear_upper_n];
            if ![v.normal_n, shear[0], shear[1], v.peak_shear_n]
                .into_iter()
                .all(f64::is_finite)
            {
                invalid_patches.insert(patch.clone());
                continue;
            }
            let accumulator = patch_metrics.entry(patch.clone()).or_default();
            accumulator.patch(sample, v.normal_n, shear, v.peak_shear_n);
            if let Some(area) = sample
                .area_m2
                .as_ref()
                .filter(|_| v.normal_n > 0.0 || shear[1] > 0.0)
            {
                areas
                    .entry(patch.clone())
                    .and_modify(|prior: &mut SourcedRange| {
                        prior.min = prior.min.min(area.min);
                        prior.max = prior.max.max(area.max);
                    })
                    .or_insert_with(|| area.clone());
            }
            if v.normal_n > 0.0
                && plan
                    .power_loss
                    .as_ref()
                    .is_some_and(|fault| fault.loaded_patch == *patch && interval.end == fault.at)
            {
                loaded.insert(patch.clone(), interval.start);
            }
            if plan
                .power_loss
                .as_ref()
                .is_some_and(|fault| fault.loaded_patch == *patch && interval.start >= fault.at)
            {
                unloaded.insert(patch.clone());
            }
            for (side, region) in sample.regions.iter().enumerate() {
                let Some(region) = region else { continue };
                let r = regions.entry(region.clone()).or_default();
                r.normal += v.normal_n;
                r.shear_upper += shear[1];
                r.point_shear = r.point_shear.max(v.peak_shear_n);
                let metrics = &accumulator.metrics;
                r.peak_drift_m = maximum(r.peak_drift_m, metrics.peak_drift_m, f64::max);
                r.pressure = maximum(r.pressure, metrics.peak_mean_pressure_pa, max_pair);
                r.shear_pa = maximum(r.shear_pa, metrics.peak_mean_shear_pa, max_pair);
                if accumulator.missing_area {
                    regions_without_area.insert(region.clone());
                }
                let sign = if side == 0 { -1.0 } else { 1.0 };
                for (total, value) in r.shear.iter_mut().zip(v.shear_n) {
                    *total += sign * value;
                }
            }
        }
        for (region, r) in regions {
            let acc = region_metrics.entry(region).or_default();
            acc.add(
                interval,
                r.normal,
                [norm(r.shear), r.shear_upper],
                r.point_shear,
            );
            let m = &mut acc.metrics;
            m.peak_drift_m = maximum(m.peak_drift_m, r.peak_drift_m, f64::max);
            // Для области сохраняем максимум среднего напряжения по отдельным пятнам.
            m.peak_mean_pressure_pa = maximum(m.peak_mean_pressure_pa, r.pressure, max_pair);
            m.peak_mean_shear_pa = maximum(m.peak_mean_shear_pa, r.shear_pa, max_pair);
        }
    }
    for region in regions_without_area {
        if let Some(acc) = region_metrics.get_mut(&region) {
            acc.metrics.peak_mean_pressure_pa = None;
            acc.metrics.peak_mean_shear_pa = None;
        }
    }
    if region_metrics.values().any(|a| !a.finite()) {
        gaps.insert("переполнение метрик области".into());
    }
    complete &= right == plan.window.end;
    if !complete {
        gaps.insert("неполное окно завершённых физических интервалов или остановка прогона".into());
    }
    if !plan.regions.is_empty() || rt.interaction_expects_response() {
        if !rt.interaction_expects_response() {
            gaps.insert("профиль реакции участника на контакт отсутствует".into());
        } else if !response_completed {
            gaps.insert("реакция на контакт не подтверждена завершённым интервалом".into());
        }
    }
    if !plan.regions.is_empty()
        && !patch_metrics.iter().any(|(patch, acc)| {
            acc.metrics.onset_ns.is_some() && binding[patch].iter().any(Option::is_some)
        })
    {
        gaps.insert("в проверенном окне нет нагруженного контакта с участником".into());
    }
    for (patch, acc) in &mut patch_metrics {
        if acc.missing_area {
            acc.metrics.peak_mean_pressure_pa = None;
            acc.metrics.peak_mean_shear_pa = None;
            gaps.insert(format!(
                "у нагруженного пятна «{patch}» есть интервалы без подтверждённой площади"
            ));
        }
        if !acc.finite() {
            invalid_patches.insert(patch.clone());
        }
        if acc.metrics.onset_ns.is_none()
            && !plan
                .patches
                .get(patch)
                .is_some_and(|p| source_is_proven(p.absence_basis.as_ref()))
        {
            gaps.insert(format!("нет подтверждённого нагружения пятна «{patch}»"));
            invalid_patches.insert(patch.clone());
        }
    }
    for patch in &invalid_patches {
        gaps.insert(format!(
            "пятно «{patch}» имеет пропуск, неконечное значение или выход из применимости"
        ));
    }
    if route_metrics.values().any(|v| !v.finite()) {
        gaps.insert("переполнение метрик натяжения".into());
    }
    let power_loss_applied_ns = plan.power_loss.as_ref().and_then(|fault| {
        let previous = *loaded.get(&fault.loaded_patch)?;
        unloaded
            .contains(&fault.loaded_patch)
            .then(|| power_off_receipt(rt, fault, previous))
            .flatten()
    });
    let metrics = |a: BTreeMap<String, Accumulator>| {
        a.into_iter()
            .filter(|(_, v)| v.finite())
            .map(|(k, v)| (k, v.metrics))
            .collect()
    };
    let peaks_at = patch_metrics
        .iter()
        .map(|(patch, summary)| (patch.clone(), summary.peaks_at.clone()))
        .collect();
    let summary = GridSummary {
        world: Fingerprint::from_fnv1a(rt.interaction_identity().unwrap().world),
        journal: Fingerprint::from_fnv1a(rt.log().fingerprint()),
        rate_hz: rt.physics_rate().as_hz(),
        patches: metrics(patch_metrics),
        regions: metrics(region_metrics),
        routes: metrics(route_metrics),
        coverage_gaps: gaps.into_iter().collect(),
        power_loss_applied_ns,
    };
    GridAudit {
        summary,
        complete,
        binding,
        areas,
        invalid_patches,
        peaks_at,
    }
}

fn external_commands(rt: &Runtime) -> impl Iterator<Item = &sim_runtime::LogEntry> {
    rt.log().entries().iter().filter(|entry| {
        matches!(
            entry.kind,
            LogKind::Command(CommandSource::BeforeStart | CommandSource::External)
        )
    })
}

fn power_off_receipt(rt: &Runtime, fault: &PowerLossPlan, previous: Time) -> Option<u64> {
    let witness = fault.witness.as_ref()?;
    if !proved_nonnegative(&witness.before)
        || !proved_nonnegative(&witness.after)
        || witness.before.min <= witness.after.max
    {
        return None;
    }
    let id = rt.truth_id(&witness.signal)?;
    if rt.truth_spec(id)?.unit != witness.unit {
        return None;
    }
    let index = rt
        .log()
        .header()
        .truth
        .iter()
        .position(|s| s.name == witness.signal)?;
    let mut rows = rt
        .log()
        .entries()
        .iter()
        .filter(|e| e.kind == LogKind::Truth && e.id as usize == index && e.valid);
    let before = rows.clone().rfind(|e| e.at == previous)?;
    let after = rows.rfind(|e| e.at == fault.at)?;
    if !(witness.before.min..=witness.before.max).contains(&before.value)
        || !(witness.after.min..=witness.after.max).contains(&after.value)
    {
        return None;
    }
    let run = sim_runtime::control::run(rt)?;
    let request = run.events().iter().any(|e| {
        e.at == fault.at
            && matches!(&e.kind,
        EventKind::FaultRequested { id, target, action: FaultAction::PowerOff, active: true }
        if *id == fault.id && *target == fault.target)
    });
    let mut applied = run.events().iter().filter(|e| {
        matches!(&e.kind,
        EventKind::FaultApplied { id, target, active: true }
        if *id == fault.id && *target == fault.target)
    });
    let receipt = applied.next()?;
    (request && receipt.at == fault.at && applied.next().is_none()).then_some(receipt.at.0)
}

fn compare_contact(
    values: [Option<&ContactMetrics>; 3],
    t: &ContactTolerances,
    comparable: bool,
) -> Vec<GridDifference> {
    let mut results = Vec::new();
    let rows_present = values.iter().all(Option::is_some);
    let mut compare = |quantity, unit, tolerance: &SourcedRange, values: [Option<f64>; 3]| {
        let differences = std::array::from_fn(|i| {
            values[i]
                .zip(values[i + 1])
                .map(|(a, b)| (b - a).abs())
                .filter(|d| d.is_finite())
        });
        let valid = proved_nonnegative(tolerance);
        let outcome = if comparable
            && valid
            && (differences
                .iter()
                .all(|d| d.is_some_and(|v| v <= tolerance.min))
                || quantity == "onset" && values == [None; 3] && rows_present)
        {
            Outcome::Pass
        } else {
            Outcome::NotAnalyzed
        };
        results.push(GridDifference {
            quantity,
            unit,
            differences,
            tolerance: valid.then_some(tolerance.min),
            outcome,
        });
    };
    compare(
        "onset",
        "s",
        &t.onset_s,
        values.map(|m| m.and_then(|m| m.onset_ns.map(|n| Time(n).as_secs()))),
    );
    for (quantity, unit, tolerance, metric) in [
        (
            "normal",
            "N",
            &t.force_n,
            (|m: &ContactMetrics| m.peak_normal_n) as fn(&ContactMetrics) -> f64,
        ),
        ("shear_min", "N", &t.force_n, |m| m.peak_shear_n[0]),
        ("shear_max", "N", &t.force_n, |m| m.peak_shear_n[1]),
        ("point_shear", "N", &t.force_n, |m| m.peak_point_shear_n),
        ("impulse", "N*s", &t.impulse_ns, |m| m.impulse_ns),
        ("duration", "s", &t.duration_s, |m| m.duration_s),
    ] {
        compare(quantity, unit, tolerance, values.map(|m| m.map(metric)));
    }
    if values.iter().flatten().any(|m| m.peak_drift_m.is_some()) {
        compare(
            "drift",
            "m",
            &t.drift_m,
            values.map(|m| m.and_then(|m| m.peak_drift_m)),
        );
    }
    for (quantity, metric) in [
        ("pressure", ContactMetric::MeanPressure),
        ("mean_shear", ContactMetric::MeanShear),
    ] {
        let samples = values.map(|m| m.and_then(|m| metric_value(m, metric)));
        if samples.iter().any(Option::is_some) {
            for index in [0, 1] {
                compare(
                    quantity,
                    "Pa",
                    &t.stress_pa,
                    samples.map(|s| s.map(|s| s[index])),
                );
            }
        }
    }
    results
}

fn compare_limit(
    rule: &LimitCriterion,
    plan: &InteractionPlan,
    audits: &[Option<GridAudit>],
    comparisons: &BTreeMap<String, Vec<GridDifference>>,
    equivalent: bool,
) -> ExposureFinding {
    let patch = plan.patches.get(&rule.patch);
    let finest = audits.iter().rev().flatten().next();
    let value = finest
        .and_then(|a| a.summary.patches.get(&rule.patch))
        .and_then(|m| metric_value(m, rule.metric));
    let limit = proved_nonnegative(&rule.limit).then_some([rule.limit.min, rule.limit.max]);
    let mut finding = ExposureFinding {
        patch: rule.patch.clone(),
        region: rule.region.clone(),
        variant: plan.variant.clone(),
        window_ns: [plan.window.start.0, plan.window.end.0],
        metric: rule.metric,
        kind: rule.kind,
        measurement: value,
        limit,
        unit: rule.metric.unit(),
        outcome: Outcome::NotAnalyzed,
        source: format!("{:?}; {:?}", rule.limit.provenance, rule.validation),
        coverage_gaps: Vec::new(),
        event_ns: None,
        event_journal: None,
        applicability: format!(
            "геометрия «{}», режим «{}», группа «{}»",
            rule.geometry, rule.mode, rule.population
        ),
    };
    let linked = plan.regions.contains(&rule.region)
        && audits.iter().flatten().all(|a| {
            a.binding
                .get(&rule.patch)
                .is_some_and(|ids| ids.iter().flatten().any(|id| *id == rule.region))
        });
    let covered = equivalent
        && audits.iter().flatten().all(|a| {
            a.complete
                && !a.invalid_patches.contains(&rule.patch)
                && a.summary.patches.contains_key(&rule.patch)
        });
    let with_area = !matches!(
        rule.metric,
        ContactMetric::MeanPressure | ContactMetric::MeanShear
    ) || rule.area_m2.is_some();
    let context = with_area
        && patch.is_some_and(|p| {
            p.geometry == rule.geometry
                && p.mode == rule.mode
                && p.population == rule.population
                && p.repetitions <= rule.max_repetitions
        })
        && source_is_proven(rule.validation.as_ref())
        && proved_nonnegative(&rule.duration_s)
        && audits.iter().flatten().all(|a| {
            a.summary.patches.get(&rule.patch).is_some_and(|m| {
                (rule.duration_s.min..=rule.duration_s.max).contains(&m.duration_s)
            }) && rule.area_m2.as_ref().is_none_or(|r| {
                proved_nonnegative(r)
                    && a.areas.get(&rule.patch).is_some_and(|area| {
                        area.can_prove() && area.min >= r.min && area.max <= r.max
                    })
            })
        });
    let quantities: &[&str] = match rule.metric {
        ContactMetric::NormalForce => &["normal"],
        ContactMetric::TangentialForce => &["shear_min", "shear_max", "point_shear"],
        ContactMetric::MeanPressure => &["normal", "pressure"],
        ContactMetric::MeanShear => &["shear_min", "shear_max", "point_shear", "mean_shear"],
    };
    let checks = comparisons.get(&format!("patch/{}", rule.patch));
    let converged = checks.is_some_and(|checks| {
        quantities.iter().chain(&["onset", "duration"]).all(|q| {
            checks.iter().any(|c| c.quantity == *q)
                && checks
                    .iter()
                    .filter(|c| c.quantity == *q)
                    .all(|c| c.outcome == Outcome::Pass)
        })
    });
    let (Some([min, max]), Some([low, high])) = (value, limit) else {
        finding
            .coverage_gaps
            .push("нет конечного измерения, площади или обоснованного предела".into());
        finding
            .applicability
            .push_str(". Нет конечного измерения, площади или обоснованного предела");
        return finding;
    };
    if !linked || !covered || !context || !converged {
        for (valid, reason) in [
            (linked, "привязка участка"),
            (covered, "полнота окна"),
            (context, "область и происхождение"),
            (converged, "сходимость показателя"),
        ] {
            if !valid {
                finding.coverage_gaps.push(reason.into());
            }
        }
        finding.applicability.push_str(&format!(
            ". Привязка: {linked}. Покрытие: {covered}. Область: {context}. Сходимость: {converged}"
        ));
        return finding;
    }
    let med = audits
        .iter()
        .flatten()
        .filter_map(|a| {
            a.summary
                .patches
                .get(&rule.patch)
                .and_then(|m| metric_value(m, rule.metric))
        })
        .collect::<Vec<_>>();
    finding.outcome = if min > high || med.iter().any(|bounds| bounds[0] > high) {
        Outcome::Fail
    } else if max <= low && med.iter().all(|bounds| bounds[1] <= low) {
        Outcome::Pass
    } else {
        Outcome::NotAnalyzed
    };
    if finding.outcome == Outcome::Fail
        && let Some((at, journal)) = audits.iter().rev().flatten().find_map(|audit| {
            let measured = metric_value(audit.summary.patches.get(&rule.patch)?, rule.metric)?;
            if measured[0] <= high {
                return None;
            }
            let at = *audit.peaks_at.get(&rule.patch)?.get(&rule.metric)?;
            Some((at, audit.summary.journal))
        })
    {
        finding.event_ns = Some(at);
        finding.event_journal = Some(journal);
    }
    finding.measurement = Some(med.into_iter().fold([min, max], max_pair));
    finding
}

fn metric_value(m: &ContactMetrics, metric: ContactMetric) -> Option<[f64; 2]> {
    match metric {
        ContactMetric::NormalForce => Some([m.peak_normal_n; 2]),
        ContactMetric::TangentialForce => Some(m.peak_shear_n),
        ContactMetric::MeanPressure => m.peak_mean_pressure_pa,
        ContactMetric::MeanShear => m.peak_mean_shear_pa,
    }
}

fn maximum<T>(a: Option<T>, b: Option<T>, combine: impl FnOnce(T, T) -> T) -> Option<T> {
    match (a, b) {
        (Some(a), Some(b)) => Some(combine(a, b)),
        (a, b) => a.or(b),
    }
}

fn max_pair(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0].max(b[0]), a[1].max(b[1])]
}
fn norm(v: [f64; 3]) -> f64 {
    v[0].hypot(v[1]).hypot(v[2])
}
fn named(s: &str) -> bool {
    !s.trim().is_empty() && !s.chars().any(char::is_control)
}
fn proved_nonnegative(range: &SourcedRange) -> bool {
    range.can_prove() && range.min >= 0.0
}
fn source_is_proven(source: Option<&Provenance>) -> bool {
    matches!(source, Some(Provenance::Measured(s) | Provenance::Paper(s) | Provenance::Datasheet(s)) if named(s))
}
