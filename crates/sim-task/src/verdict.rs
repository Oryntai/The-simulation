//! Обязательства и вердикт кампании. Отчёт не является доказательством.

use serde::{Deserialize, Serialize};
use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum Group {
    Mechanics,
    Parts,
    Actuators,
    Contact,
    Human,
    Power,
    Battery,
    Wiring,
    Firmware,
    Measurements,
}

impl Group {
    pub const ALL: [Self; 10] = [
        Self::Mechanics,
        Self::Parts,
        Self::Actuators,
        Self::Contact,
        Self::Human,
        Self::Power,
        Self::Battery,
        Self::Wiring,
        Self::Firmware,
        Self::Measurements,
    ];
}

impl fmt::Display for Group {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Mechanics => "Механика",
            Self::Parts => "Детали",
            Self::Actuators => "Приводы",
            Self::Contact => "Контакт с телом",
            Self::Human => "Человек",
            Self::Power => "Питание",
            Self::Battery => "Батарея",
            Self::Wiring => "Плата и соединения",
            Self::Firmware => "Прошивка",
            Self::Measurements => "Замеры",
        })
    }
}

mod evidence;
mod registry;

use std::collections::{BTreeMap, BTreeSet};

use sim_process::work::{ProcessBinding, WorkSubject, required_hazards};
use sim_world::{JointType, Provenance, SectionAxis};

use crate::{
    Error, FaultKind, Fingerprint, Limit, Outcome, Signal, Source, SourceKind,
    campaign::{CampaignPlan, Disposition, RunKey, Window},
    interaction::{EnergyPath, Reach, safety_decision},
    invalid,
};

pub use crate::interaction::SafetyDecision as Readiness;
pub use evidence::{CampaignEvidence, Installed, MeasurementSample, SnapshotAudit};
pub use registry::{ClassSpec, HazardClass, Method, REGISTRY, REGISTRY_VERSION};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Domain {
    Parts,
    Geometry,
    Attachments,
    Catalog,
    Power,
    Control,
    Environment,
    Participant,
    Measurements,
}

impl Domain {
    const ALL: [Self; 9] = [
        Self::Parts,
        Self::Geometry,
        Self::Attachments,
        Self::Catalog,
        Self::Power,
        Self::Control,
        Self::Environment,
        Self::Participant,
        Self::Measurements,
    ];
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ComponentKind {
    Part,
    CyclicLoad,
    Retention,
    Connection,
    MotionLimit,
    FreeSupport,
    ImpactPath,
    PinchPath,
    SharpSurface,
    ParticipantContact,
    MotionAtParticipant,
    HeatPath,
    Drive,
    Wire,
    Supply,
    Battery,
    ExposedElectricalPath,
    EnergyPath,
    EnvironmentalPath,
    Sensor,
    Communication,
    Controller,
    Output,
    Protection,
    CriticalMeasurement,
    Program,
}

#[derive(Clone, Debug)]
pub struct Component {
    pub object: String,
    pub kind: ComponentKind,
    pub source: Source,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Level {
    Construction,
    Simulation,
    Bench,
}

#[derive(Clone, Debug)]
pub enum Verification {
    /// Изгиб однородного участка в одной плоскости под ограниченным моментом.
    Section {
        geometry: usize,
        axis: SectionAxis,
        moment_nm: [f64; 2],
        allowable_pa: [f64; 2],
        load: Source,
        limit: Source,
    },
    /// Измерение в журнале истины. Порог и область задаёт разрешённый профиль.
    Truth(Limit),
    /// Ключ метрики или события в расчёте нескольких физических сеток.
    Process {
        profile: Fingerprint,
        criterion: String,
    },
    /// Вывод по пятну, области и типу нагрузки либо по отдельному пути энергии.
    Interaction {
        profile: Fingerprint,
        quantity: String,
    },
    /// Прогон контура с проверкой установленного образа, пути и реакции.
    Control {
        criteria: Fingerprint,
        check: ControlCriterion,
    },
    /// Предел и требуемый измерительный канал для данной версии и режима.
    Measurement(Limit),
}

#[derive(Clone, Debug)]
pub enum ControlCriterion {
    Limit(Signal),
    Reaction(Signal),
    ProgramIdentity,
}

/// Любое заполненное поле ограничивает применимость проверки.
#[derive(Clone, Debug, Default)]
pub struct RunSelector {
    pub station: Option<String>,
    pub mode: Option<String>,
    pub path: Option<String>,
    pub faults: Option<Vec<crate::Fault>>,
}

impl RunSelector {
    fn matches(&self, key: &RunKey) -> bool {
        self.station.as_ref().is_none_or(|v| *v == key.station)
            && self.mode.as_ref().is_none_or(|v| *v == key.mode)
            && self.path.as_ref().is_none_or(|v| *v == key.path)
            && self.faults.as_ref().is_none_or(|v| *v == key.faults)
    }
}

#[derive(Clone, Debug)]
pub struct Check {
    pub class: HazardClass,
    pub object: String,
    pub selector: RunSelector,
    pub window: Window,
    pub applicability: Source,
    pub scope: String,
    /// Источники всех входов расчёта, включая порог и границы применимости.
    pub inputs: Vec<Source>,
    pub verification: Verification,
}

/// Полный факт разрешения домена или явная причина открытого покрытия.
/// Доказательство отсутствия компонента хранится вместе с доменом.
#[derive(Clone, Debug)]
pub enum Coverage {
    Resolved(Source),
    Unknown(String),
}

#[derive(Clone, Debug, Default)]
pub struct ResolvedInventory {
    pub build: Option<Fingerprint>,
    pub participant: Option<Reach>,
    /// Расширенный мир с разрешёнными фрагментами, например моделью участника.
    pub compiled_rows: BTreeMap<RunKey, Fingerprint>,
    /// Отпечатки полных критериев специализированных опытов.
    pub linked_profiles: BTreeMap<RunKey, Fingerprint>,
    pub coverage: BTreeMap<Domain, Coverage>,
    pub components: Vec<Component>,
    pub work_subjects: Vec<WorkSubject>,
    pub process_bindings: Vec<ProcessBinding>,
    pub checks: Vec<Check>,
    /// Ссылки SourceKind::Derived разрешаются рекурсивно по этой таблице.
    pub derivations: BTreeMap<String, Vec<Source>>,
    /// Положительная огибающая для аппаратного решения должна быть отдельным расчётом.
    /// Здесь хранятся открытые диапазоны, обнаруженные при разрешении профиля.
    pub unresolved_ranges: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct RequirementKey {
    pub class: HazardClass,
    pub object: String,
    pub extension: Option<String>,
    pub run: Option<RunKey>,
    pub window_ns: Option<[u64; 2]>,
    pub level: Level,
}

#[derive(Clone, Debug, Serialize)]
pub struct LimitValue {
    pub lower: Option<[f64; 2]>,
    pub upper: Option<[f64; 2]>,
}

impl LimitValue {
    pub(crate) fn upper(bounds: [f64; 2]) -> Self {
        Self {
            lower: None,
            upper: Some(bounds),
        }
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct Finding {
    pub key: RequirementKey,
    pub code: String,
    pub group: Group,
    pub outcome: Outcome,
    pub reason: String,
    pub measurement: Option<[f64; 2]>,
    pub limit: Option<LimitValue>,
    pub unit: Option<String>,
    pub measurement_sources: Vec<String>,
    pub limit_sources: Vec<String>,
    pub model_scope: Option<String>,
    pub coverage_gaps: Vec<String>,
    pub missing_data: Vec<String>,
    pub verification: String,
    pub event_ns: Option<u64>,
    pub journal: Option<Fingerprint>,
    /// Отдельное подтверждение для корпуса опасностей.
    pub confirmed_hazard: bool,
}

impl Finding {
    pub fn open(key: &RequirementKey, reason: impl Into<String>) -> Self {
        let spec = key.class.spec();
        let reason = reason.into();
        Self {
            key: key.clone(),
            code: spec.id.into(),
            group: spec.group,
            outcome: Outcome::NotAnalyzed,
            missing_data: vec![reason.clone()],
            reason,
            measurement: None,
            limit: None,
            unit: None,
            measurement_sources: Vec::new(),
            limit_sources: Vec::new(),
            model_scope: None,
            coverage_gaps: Vec::new(),
            verification: spec.independent_check.into(),
            event_ns: None,
            journal: None,
            confirmed_hazard: false,
        }
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct NotApplicable {
    pub class: HazardClass,
    pub reason: String,
    pub sources: Vec<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum SimulationVerdict {
    Rejected,
    NotProven,
    PassedSimulation,
}

#[derive(Debug, Serialize)]
pub struct VerdictReport {
    pub registry_version: u32,
    pub plan: Fingerprint,
    pub build: Option<Fingerprint>,
    pub simulation: SimulationVerdict,
    pub readiness: Readiness,
    pub findings: Vec<Finding>,
    pub diagnostics: Vec<Finding>,
    pub inapplicable: Vec<NotApplicable>,
    pub inapplicable_runs: Vec<RunKey>,
}

#[derive(Clone, Debug)]
struct Requirement {
    key: RequirementKey,
    checks: Vec<Check>,
    fixed: Option<Finding>,
    declared: Vec<Source>,
}

#[derive(Debug)]
pub struct RequirementSet {
    plan: Fingerprint,
    stamp: Fingerprint,
    build: Option<Fingerprint>,
    requirements: Vec<Requirement>,
    inapplicable: Vec<NotApplicable>,
    inapplicable_runs: Vec<RunKey>,
    derivations: BTreeMap<String, Vec<Source>>,
    compiled_rows: BTreeMap<RunKey, Fingerprint>,
    linked_profiles: BTreeMap<RunKey, Fingerprint>,
}

impl RequirementSet {
    pub fn keys(&self) -> impl Iterator<Item = &RequirementKey> {
        self.requirements.iter().map(|r| &r.key)
    }
}

fn derives(
    source: &Source,
    graph: &BTreeMap<String, Vec<Source>>,
    active: &mut BTreeSet<String>,
    trace: &mut BTreeSet<String>,
) -> bool {
    trace.insert(format!("{:?}: {}", source.kind, source.reference));
    match source.kind {
        SourceKind::Estimate => false,
        SourceKind::Derived => {
            if !active.insert(source.reference.clone()) {
                return false;
            }
            let proved = graph.get(&source.reference).is_some_and(|inputs| {
                !inputs.is_empty()
                    && inputs
                        .iter()
                        .map(|input| derives(input, graph, active, trace))
                        .fold(true, |a, b| a & b)
            });
            active.remove(&source.reference);
            proved
        }
        _ => true,
    }
}

fn provenance(source: &Provenance) -> Source {
    let (kind, reference) = match source {
        Provenance::Datasheet(s) => (SourceKind::Datasheet, s),
        Provenance::Paper(s) => (SourceKind::Paper, s),
        Provenance::Measured(s) => (SourceKind::Measured, s),
        Provenance::Derived(s) => (SourceKind::Derived, s),
        Provenance::Estimate(s) => (SourceKind::Estimate, s),
    };
    Source {
        kind,
        reference: reference.clone(),
    }
}

fn trace(sources: &[Source], graph: &BTreeMap<String, Vec<Source>>) -> (bool, Vec<String>) {
    let mut refs = BTreeSet::new();
    let proved = sources
        .iter()
        .map(|s| {
            let valid = s.validate().is_ok();
            derives(s, graph, &mut BTreeSet::new(), &mut refs) && valid
        })
        .fold(!sources.is_empty(), |a, b| a & b);
    (proved, refs.into_iter().collect())
}

fn checked_pair(values: [f64; 2]) -> Result<(), Error> {
    if values.into_iter().all(f64::is_finite) && values[0] <= values[1] {
        Ok(())
    } else {
        Err(invalid("неконечный или обратный интервал доказательства"))
    }
}

fn limit_source(limit: &Limit) -> &Source {
    match limit {
        Limit::Range { source, .. } | Limit::Forbidden { source, .. } => source,
    }
}

fn validate_limit(limit: &Limit) -> Result<(), Error> {
    limit.signal().validate()?;
    limit_source(limit).validate()?;
    if matches!(limit, Limit::Forbidden { signal, .. } if !matches!(signal.unit.as_str(), "1" | "bool"))
    {
        return Err(invalid("запретное событие должно быть двоичным"));
    }
    if let Limit::Range { min, max, .. } = limit {
        if min.is_none() && max.is_none()
            || min.iter().chain(max).any(|v| !v.is_finite())
            || min.zip(*max).is_some_and(|(a, b)| a > b)
        {
            return Err(invalid("недопустимый численный предел"));
        }
    }
    Ok(())
}

fn validate_check(check: &Check) -> Result<(), Error> {
    crate::task::check_name(&check.object)?;
    crate::task::check_name(&check.scope)?;
    check.window.validate()?;
    check.applicability.validate()?;
    for input in &check.inputs {
        input.validate()?;
    }
    let method = check.class.spec().method;
    match &check.verification {
        Verification::Section {
            moment_nm,
            allowable_pa,
            load,
            limit,
            ..
        } if method == Method::Section => {
            checked_pair(*moment_nm)?;
            checked_pair(*allowable_pa)?;
            if allowable_pa[0] <= 0.0 {
                return Err(invalid("предел напряжения должен быть положительным"));
            }
            load.validate()?;
            limit.validate()
        }
        Verification::Truth(limit) if method == Method::Dynamics => validate_limit(limit),
        Verification::Process { criterion, .. }
            if matches!(method, Method::Process | Method::Dynamics) =>
        {
            crate::task::check_name(criterion)
        }
        Verification::Interaction { quantity, .. }
            if method == Method::Interaction || check.class == HazardClass::StoredEnergy =>
        {
            crate::task::check_name(quantity)
        }
        Verification::Control { check: kind, .. } if method == Method::Control => match kind {
            ControlCriterion::ProgramIdentity if check.class == HazardClass::ProgramIdentity => {
                Ok(())
            }
            ControlCriterion::Limit(signal) | ControlCriterion::Reaction(signal)
                if check.class != HazardClass::ProgramIdentity =>
            {
                signal.validate()
            }
            _ => Err(invalid("критерий контура не соответствует классу")),
        },
        Verification::Measurement(limit) if method == Method::Measurement => validate_limit(limit),
        _ => Err(invalid("метод проверки не соответствует классу опасности")),
    }
}

fn level(class: HazardClass) -> Level {
    match class {
        HazardClass::Measurement => Level::Bench,
        HazardClass::StaticStrength | HazardClass::Inventory => Level::Construction,
        _ => Level::Simulation,
    }
}

fn requires_fault(class: HazardClass) -> bool {
    matches!(
        class,
        HazardClass::SensorFault
            | HazardClass::WireFault
            | HazardClass::LinkFault
            | HazardClass::ControllerHalt
            | HazardClass::OutputFault
    )
}

fn fault_class(kind: FaultKind) -> HazardClass {
    match kind {
        FaultKind::SensorUnavailable => HazardClass::SensorFault,
        FaultKind::WireOpen => HazardClass::WireFault,
        FaultKind::PacketLoss | FaultKind::PacketDelay => HazardClass::LinkFault,
        FaultKind::ControllerHalted => HazardClass::ControllerHalt,
        FaultKind::DriverDisabled | FaultKind::PowerCut | FaultKind::DriveDisabled => {
            HazardClass::OutputFault
        }
    }
}

/// Точная идентичность скомпилированного мира и параметров модели.
pub fn compiled_world_fingerprint(world: &sim_world::CompiledWorld) -> Fingerprint {
    Fingerprint::of(format!("{world:?}").as_bytes())
}

/// Состав из модели мира до запуска. Дополнительные процессы, проводка и
/// контакты участника разрешаются их установщиками.
pub fn world_components(world: &crate::campaign::WorldInputs) -> Vec<(ComponentKind, String)> {
    let mut components = Vec::new();
    let mut mark = |kind, object: &str| components.push((kind, object.to_owned()));
    for (placement, input) in &world.devices {
        for link in &input.device.links {
            let name = format!("{placement}/{}", link.name);
            mark(ComponentKind::Part, &name);
            if link.joint.kind != JointType::Fixed {
                let joint = format!(
                    "{placement}/{}",
                    link.joint.name.as_deref().unwrap_or(&link.name)
                );
                if link.joint.kind == JointType::Free {
                    mark(ComponentKind::FreeSupport, &joint);
                }
                if link.joint.limits_deg.is_some()
                        || link.joint.limits_m.is_some()
                        || link.joint.ball_limit_deg.is_some()
                        // Канал усилия задаёт управляемое движение. Без
                        // проверки нельзя вывести допустимый ход из отсутствия упора.
                        || (link.joint.kind != JointType::Free && !link.joint.efforts.is_empty())
                {
                    mark(ComponentKind::MotionLimit, &joint);
                }
            }
        }
        for constraint in &input.device.constraints {
            let name = match constraint {
                sim_world::Constraint::Connect { name, .. }
                | sim_world::Constraint::Weld { name, .. }
                | sim_world::Constraint::ConnectAtReference { name, .. }
                | sim_world::Constraint::WeldAtReference { name, .. } => name,
            };
            mark(ComponentKind::Connection, &format!("{placement}/{name}"));
        }
        for drive in &input.device.drives {
            mark(ComponentKind::Drive, &format!("{placement}/{}", drive.name));
        }
        for sensor in &input.device.sensors {
            mark(
                ComponentKind::Sensor,
                &format!("{placement}/{}", sensor.name),
            );
        }
    }
    for object in world
        .world
        .objects
        .iter()
        .chain(&world.contact.added_objects)
    {
        mark(ComponentKind::Part, &object.name);
        mark(ComponentKind::FreeSupport, &object.name);
    }
    for sensor in &world.world.sensors {
        mark(ComponentKind::Sensor, &sensor.name);
    }
    for connection in world
        .world
        .attachments
        .iter()
        .chain(&world.contact.added_attachments)
    {
        mark(ComponentKind::Connection, &connection.name);
        mark(ComponentKind::Retention, &connection.name);
    }
    components
}

pub fn derive_requirements(
    inventory: &ResolvedInventory,
    plan: &CampaignPlan,
) -> Result<RequirementSet, Error> {
    let mut builder = Builder::new(inventory, plan);
    builder.inventory()?;
    builder.rows()?;
    Ok(builder.finish())
}

struct Builder<'a> {
    inventory: &'a ResolvedInventory,
    plan: &'a CampaignPlan,
    requirements: BTreeMap<RequirementKey, Requirement>,
    applicable: BTreeSet<(HazardClass, String)>,
    absent: Vec<NotApplicable>,
    excluded: Vec<RunKey>,
    domain_gaps: bool,
}

impl<'a> Builder<'a> {
    fn new(inventory: &'a ResolvedInventory, plan: &'a CampaignPlan) -> Self {
        Self {
            inventory,
            plan,
            requirements: BTreeMap::new(),
            applicable: BTreeSet::new(),
            absent: Vec::new(),
            excluded: Vec::new(),
            domain_gaps: false,
        }
    }

    fn add(&mut self, key: RequirementKey) -> &mut Requirement {
        self.requirements
            .entry(key.clone())
            .or_insert_with(|| Requirement {
                key,
                checks: Vec::new(),
                fixed: None,
                declared: Vec::new(),
            })
    }

    fn gap(&mut self, object: String, reason: impl Into<String>, level: Level) {
        let key = RequirementKey {
            class: HazardClass::Inventory,
            object,
            extension: None,
            run: None,
            window_ns: None,
            level,
        };
        self.add(key.clone()).fixed = Some(Finding::open(&key, reason));
    }

    fn mark(&mut self, kind: ComponentKind, object: &str) {
        self.applicable.extend(
            REGISTRY
                .iter()
                .filter(|spec| spec.applies_to.contains(&kind))
                .map(|spec| (spec.class, object.to_owned())),
        );
    }

    fn inventory(&mut self) -> Result<(), Error> {
        let inv = self.inventory;
        for (reference, inputs) in &inv.derivations {
            crate::task::check_name(reference)?;
            for source in inputs {
                source.validate()?;
            }
        }
        for domain in Domain::ALL {
            let reason = match inv.coverage.get(&domain) {
                Some(Coverage::Resolved(source)) => {
                    source.validate()?;
                    if trace(std::slice::from_ref(source), &inv.derivations).0 {
                        continue;
                    }
                    format!("не доказано разрешение области {domain:?}")
                }
                Some(Coverage::Unknown(reason)) => {
                    crate::task::check_name(reason)?;
                    reason.clone()
                }
                None => format!("нет инвентаризации области {domain:?}"),
            };
            self.domain_gaps = true;
            self.gap(format!("состав/{domain:?}"), reason, Level::Construction);
        }
        if inv.build.is_none() {
            self.gap(
                "состав/версия".into(),
                "нет идентичности сборки",
                Level::Construction,
            );
        }
        for (i, reason) in inv.unresolved_ranges.iter().enumerate() {
            crate::task::check_name(reason)?;
            self.gap(format!("диапазон/{i}"), reason, Level::Bench);
        }
        if let Some(reason) = self.plan.sampling_gap()? {
            self.gap("разброс".into(), reason, Level::Simulation);
        }
        for axis in self.plan.varying_axes() {
            self.gap(
                format!("допуск/{axis}"),
                "не подтверждена непрерывная огибающая допуска",
                Level::Bench,
            );
        }
        for component in &inv.components {
            crate::task::check_name(&component.object)?;
            component.source.validate()?;
            self.mark(component.kind, &component.object);
            if !trace(std::slice::from_ref(&component.source), &inv.derivations).0 {
                self.gap(
                    format!("состав/{}/{:?}", component.object, component.kind),
                    "источник компонента неполон или оценочный",
                    Level::Construction,
                );
            }
        }
        for (kind, object) in world_components(self.plan.world_inputs()) {
            self.mark(kind, &object);
        }
        if inv
            .compiled_rows
            .keys()
            .chain(inv.linked_profiles.keys())
            .any(|key| self.plan.row(key).is_none())
        {
            return Err(invalid("расширенный мир назначен неизвестному ключу"));
        }
        for h in required_hazards(&inv.work_subjects, &inv.process_bindings)
            .map_err(|e| invalid(e.to_string()))?
        {
            self.applicable.insert((h.kind.into(), h.subject));
        }
        for check in &inv.checks {
            validate_check(check)?;
            self.applicable.insert((check.class, check.object.clone()));
        }
        let contact = self
            .applicable
            .iter()
            .any(|(class, _)| *class == HazardClass::TissueLoad);
        let participant_gap = match inv.participant {
            Some(Reach::Reachable) if contact => None,
            Some(Reach::OffParticipant) if !contact => None,
            Some(Reach::Reachable) => Some("участник достижим, но нет инвентаря контактных пятен"),
            Some(Reach::OffParticipant) => {
                Some("изоляция участника противоречит инвентарю контакта")
            }
            _ => Some("не подтверждена достижимость участника"),
        };
        if let Some(reason) = participant_gap {
            self.domain_gaps = true;
            self.gap("состав/достижимость".into(), reason, Level::Construction);
        }
        if self
            .applicable
            .iter()
            .any(|(class, _)| *class == HazardClass::StaticStrength)
            && !self
                .applicable
                .iter()
                .any(|(class, _)| *class == HazardClass::Measurement)
        {
            self.gap(
                "замеры/сборка".into(),
                "нет проверяемого замера для версии сборки",
                Level::Bench,
            );
        }
        if inv.participant == Some(Reach::Reachable) {
            for (placement, input) in &self.plan.world_inputs().devices {
                for drive in &input.device.drives {
                    self.mark(
                        ComponentKind::MotionAtParticipant,
                        &format!("{placement}/{}", drive.name),
                    );
                }
            }
            if self
                .applicable
                .iter()
                .filter(|(class, _)| *class == HazardClass::StoredEnergy)
                .count()
                > 1
            {
                self.applicable
                    .insert((HazardClass::StoredEnergy, "совокупность".into()));
            }
        }
        Ok(())
    }

    fn rows(&mut self) -> Result<(), Error> {
        let fault_rows: BTreeSet<_> = self
            .plan
            .rows()
            .iter()
            .filter(|row| {
                !matches!(&row.disposition, Disposition::Inapplicable { bases, .. }
                if trace(bases, &self.inventory.derivations).0)
            })
            .flat_map(|row| {
                row.key.faults.iter().map(|fault| {
                    (
                        &row.key.station,
                        &row.key.mode,
                        fault_class(fault.kind),
                        &fault.node,
                    )
                })
            })
            .collect();
        for row in self.plan.rows() {
            let graph = &self.inventory.derivations;
            if let Disposition::Inapplicable { bases, .. } = &row.disposition {
                if trace(bases, graph).0 {
                    self.excluded.push(row.key.clone());
                    continue;
                }
            }
            let mode = self.plan.mode(&row.key);
            let mut active = self.applicable.clone();
            active.extend(
                row.key
                    .faults
                    .iter()
                    .map(|fault| (fault_class(fault.kind), fault.node.clone())),
            );
            for (class, object) in active {
                if requires_fault(class)
                    && fault_rows.contains(&(&row.key.station, &row.key.mode, class, &object))
                    && !row
                        .key
                        .faults
                        .iter()
                        .any(|fault| fault.node == object && fault_class(fault.kind) == class)
                {
                    continue;
                }
                let mut checks: Vec<_> = self
                    .inventory
                    .checks
                    .iter()
                    .filter(|c| {
                        c.class == class && c.object == object && c.selector.matches(&row.key)
                    })
                    .cloned()
                    .collect();
                if class == HazardClass::StoredEnergy
                    && self.inventory.participant == Some(Reach::Reachable)
                {
                    for path in [
                        EnergyPath::Electrical,
                        EnergyPath::Thermal,
                        EnergyPath::Mechanical,
                    ] {
                        let quantity = format!("{path:?}");
                        let selected: Vec<_> = checks.iter().filter(|check| {
                            matches!(&check.verification, Verification::Interaction { quantity: q, .. } if q == &quantity)
                        }).cloned().collect();
                        let key = RequirementKey {
                            class,
                            object: object.clone(),
                            extension: Some(quantity),
                            run: Some(row.key.clone()),
                            window_ns: mode.map(|m| [0, m.horizon_ns]),
                            level: Level::Simulation,
                        };
                        if selected.is_empty() {
                            self.add(key);
                        } else {
                            for check in selected {
                                if mode.is_some_and(|m| check.window.end_ns > m.horizon_ns) {
                                    return Err(invalid(
                                        "окно оценки энергии выходит за горизонт режима",
                                    ));
                                }
                                self.add(RequirementKey {
                                    window_ns: Some([check.window.start_ns, check.window.end_ns]),
                                    ..key.clone()
                                })
                                .checks
                                .push(check);
                            }
                        }
                    }
                    continue;
                }
                checks.sort_by_key(|c| (c.window.start_ns, c.window.end_ns, c.scope.clone()));
                if checks.is_empty() {
                    self.add(RequirementKey {
                        class,
                        object,
                        extension: None,
                        run: Some(row.key.clone()),
                        window_ns: mode.map(|m| [0, m.horizon_ns]),
                        level: level(class),
                    });
                } else {
                    // Проверка истинного ряда в коротком окне не закрывает
                    // возможность того же нарушения в остальной части режима.
                    if let Some(mode) = mode {
                        let mut covered = BTreeMap::<_, u64>::new();
                        for check in &checks {
                            if let Verification::Truth(limit) = &check.verification {
                                let end = covered.entry(limit.signal()).or_default();
                                if check.window.start_ns <= *end {
                                    *end = (*end).max(check.window.end_ns);
                                }
                            }
                        }
                        for (signal, end) in covered {
                            if end < mode.horizon_ns {
                                let key = RequirementKey {
                                    class,
                                    object: object.clone(),
                                    extension: Some(signal.name.clone()),
                                    run: Some(row.key.clone()),
                                    window_ns: Some([end, mode.horizon_ns]),
                                    level: level(class),
                                };
                                self.add(key.clone()).fixed = Some(Finding::open(
                                    &key,
                                    "проверки сигнала не покрывают весь горизонт опасности",
                                ));
                            }
                        }
                    }
                    for check in checks {
                        if mode.is_some_and(|m| check.window.end_ns > m.horizon_ns) {
                            return Err(invalid(format!(
                                "окно «{}» выходит за горизонт режима {:?}",
                                check.scope, row.key
                            )));
                        }
                        self.add(RequirementKey {
                            class,
                            object: object.clone(),
                            extension: None,
                            run: Some(row.key.clone()),
                            window_ns: Some([check.window.start_ns, check.window.end_ns]),
                            level: level(class),
                        })
                        .checks
                        .push(check);
                    }
                }
            }
            if let Some(mode) = mode {
                for declared in &mode.hazards {
                    let class = REGISTRY
                        .iter()
                        .find(|spec| spec.id == declared.class)
                        .filter(|spec| {
                            !matches!(
                                spec.class,
                                HazardClass::Execution
                                    | HazardClass::Inventory
                                    | HazardClass::TaskCriterion
                            )
                        })
                        .map(|spec| spec.class);
                    let window = mode.windows[&declared.window];
                    let key = RequirementKey {
                        class: class.unwrap_or(HazardClass::Inventory),
                        extension: class.is_none().then(|| declared.class.clone()),
                        object: declared.object.clone(),
                        run: Some(row.key.clone()),
                        window_ns: Some([window.start_ns, window.end_ns]),
                        level: class.map_or(Level::Construction, level),
                    };
                    let requirement = self.add(key.clone());
                    requirement.declared.push(declared.source.clone());
                    if class.is_none() {
                        let mut finding =
                            Finding::open(&key, "класс из постановки отсутствует в реестре");
                        finding.code.clone_from(&declared.class);
                        requirement.fixed = Some(finding);
                    }
                }
                for criterion in &mode.criteria {
                    let window = mode.windows[&criterion.window];
                    let key = RequirementKey {
                        class: HazardClass::TaskCriterion,
                        object: criterion.id.clone(),
                        extension: None,
                        run: Some(row.key.clone()),
                        window_ns: Some([window.start_ns, window.end_ns]),
                        level: Level::Simulation,
                    };
                    self.add(key).checks.push(Check {
                        class: HazardClass::TaskCriterion,
                        object: criterion.id.clone(),
                        selector: RunSelector::default(),
                        window,
                        applicability: mode.source.clone(),
                        scope: "критерий постановки".into(),
                        inputs: vec![],
                        verification: Verification::Truth(criterion.limit.clone()),
                    });
                }
            }
            // Даже полностью пассивная постановка требует живого журнала.
            let execution = RequirementKey {
                class: HazardClass::Execution,
                object: "выполнение".into(),
                extension: None,
                run: Some(row.key.clone()),
                window_ns: mode.map(|m| [0, m.horizon_ns]),
                level: Level::Simulation,
            };
            let gate = self.add(execution.clone());
            match &row.disposition {
                Disposition::Ready => {}
                Disposition::NotAnalyzed { reasons }
                | Disposition::Inapplicable { reasons, .. } => {
                    let mut finding =
                        Finding::open(&execution, "строка не готова к живому исполнению");
                    finding.coverage_gaps.clone_from(reasons);
                    gate.fixed = Some(finding);
                }
            }
        }
        if self.plan.rows().is_empty() || self.excluded.len() == self.plan.rows().len() {
            self.gap(
                "матрица".into(),
                "нет применимых строк кампании",
                Level::Construction,
            );
        }
        if !self.domain_gaps {
            let observed: BTreeSet<_> = self.requirements.keys().map(|k| k.class).collect();
            let bases: Vec<_> = self
                .inventory
                .coverage
                .values()
                .filter_map(|c| match c {
                    Coverage::Resolved(s) => Some(s.clone()),
                    _ => None,
                })
                .collect();
            let sources = trace(&bases, &self.inventory.derivations).1;
            self.absent.extend(
                REGISTRY
                    .iter()
                    .filter(|r| !observed.contains(&r.class) && r.class != HazardClass::Inventory)
                    .map(|r| NotApplicable {
                        class: r.class,
                        reason: "в разрешённом составе и матрице нет применимости".into(),
                        sources: sources.clone(),
                    }),
            );
        }
        Ok(())
    }

    fn finish(self) -> RequirementSet {
        let stamp = Fingerprint::of(
            format!(
                "{REGISTRY_VERSION}:{:?}:{:?}:{:?}",
                self.plan.fingerprint(),
                self.inventory,
                self.requirements
            )
            .as_bytes(),
        );
        RequirementSet {
            plan: self.plan.fingerprint(),
            stamp,
            build: self.inventory.build,
            requirements: self.requirements.into_values().collect(),
            inapplicable: self.absent,
            inapplicable_runs: self.excluded,
            derivations: self.inventory.derivations.clone(),
            compiled_rows: self.inventory.compiled_rows.clone(),
            linked_profiles: self.inventory.linked_profiles.clone(),
        }
    }
}

pub fn assess_campaign(
    requirements: &RequirementSet,
    evidence: &CampaignEvidence,
) -> Result<VerdictReport, Error> {
    let matched = evidence.stamp == requirements.stamp;
    let mut findings = Vec::new();
    for requirement in &requirements.requirements {
        if let Some(finding) = &requirement.fixed {
            findings.push(finding.clone());
        } else if matched {
            evidence.findings(requirements, requirement, &mut findings)?;
        } else {
            findings.push(Finding::open(
                &requirement.key,
                "доказательство относится к другому набору требований",
            ));
        }
    }
    let diagnostics = if matched {
        evidence.diagnostics(requirements)
    } else {
        Vec::new()
    };
    findings.extend(
        diagnostics
            .iter()
            .filter(|f| f.outcome == Outcome::Fail)
            .cloned(),
    );
    let outcome = Outcome::aggregate(
        findings
            .iter()
            .filter(|f| f.key.level != Level::Bench || f.outcome == Outcome::Fail)
            .map(|f| f.outcome),
    );
    let simulation = match outcome {
        Outcome::Fail => SimulationVerdict::Rejected,
        Outcome::NotAnalyzed => SimulationVerdict::NotProven,
        Outcome::Pass => SimulationVerdict::PassedSimulation,
    };
    let readiness = safety_decision(
        findings.iter().map(|f| f.outcome),
        simulation == SimulationVerdict::PassedSimulation,
    );
    Ok(VerdictReport {
        registry_version: REGISTRY_VERSION,
        plan: requirements.plan,
        build: requirements.build,
        simulation,
        readiness,
        findings,
        diagnostics,
        inapplicable: requirements.inapplicable.clone(),
        inapplicable_runs: requirements.inapplicable_runs.clone(),
    })
}
