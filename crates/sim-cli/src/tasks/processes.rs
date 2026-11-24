mod models;

use std::{
    collections::{BTreeMap, BTreeSet},
    io::Write,
    path::{Path, PathBuf},
};

use serde::{Deserialize, Serialize, de::DeserializeOwned};
use sim_process::{
    Interval, SourcedRange,
    contact::{
        ContactFriction, ContactMaterialTable, FrictionReference, GeometryMaterial, NormalResponse,
        PairCalibration, PairParameters,
    },
    work::{ProcessBinding, WorkSubject},
};
use sim_runtime::Runtime;
use sim_signal::{Rate, Time};
use sim_task::{
    Fingerprint, Outcome,
    campaign::WorldInputs,
    process::{
        ConvergencePlan, ConvergenceReport, Criterion, ErrorBounds, EventExpectation, Series,
        StateExpectation, Statistic, TrialVariant, VariantKind, assess_processes_with_snapshot,
    },
    sampling::ModelSettings,
};
use sim_world::{ConstraintSolver, ContactBuildOptions, ContactSolver, FrictionCone, Provenance};

use super::campaign::installation::{LinkedInventory, LinkedSample, Receipt};
use super::{Models, Setup, frame, measurements, read, readouts, reference};

mod screen;
use crate::{Options, catalog_path, export};

#[derive(Default, Deserialize)]
#[serde(transparent)]
pub(super) struct ModelSet(BTreeMap<String, models::Spec>);

impl ModelSet {
    pub(super) fn install(
        &self,
        compiled: &sim_world::CompiledWorld,
        rt: &mut Runtime,
        world: &sim_world::World,
        path: &Path,
    ) -> Result<(), String> {
        for (name, model) in &self.0 {
            model.validate_links(&self.0, compiled)?;
            model.install(name, compiled, rt, world, path, name)?;
        }
        Ok(())
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct TaskFile {
    version: u32,
    process_profile: PathBuf,
}

impl TaskFile {
    pub(super) fn load(path: &Path) -> Result<Option<Self>, String> {
        let bytes = read(path)?;
        let table: toml::Table = parse(path, &bytes)?;
        if !table.contains_key("process_profile") {
            return Ok(None);
        }
        let task: Self = parse(path, &bytes)?;
        if task.version != 1 {
            return Err("неподдерживаемая версия задания процессов".into());
        }
        Ok(Some(task))
    }
}

pub(super) fn parse<T: DeserializeOwned>(path: &Path, bytes: &[u8]) -> Result<T, String> {
    let text = std::str::from_utf8(bytes).map_err(|e| format!("{}: {e}", path.display()))?;
    toml::from_str(text).map_err(|e| format!("{}: {e}", path.display()))
}

pub(super) fn ensure_finite(value: &toml::Value) -> Result<(), String> {
    match value {
        toml::Value::Float(value) if !value.is_finite() => {
            Err("в данных процессов допустимы только конечные числа".into())
        }
        toml::Value::Table(table) => table.values().try_for_each(ensure_finite),
        toml::Value::Array(array) => array.iter().try_for_each(ensure_finite),
        _ => Ok(()),
    }
}

#[derive(Clone, Default)]
pub(super) struct Inputs(
    BTreeMap<PathBuf, std::sync::Arc<[u8]>>,
    BTreeMap<PathBuf, PathBuf>,
);

#[derive(Clone, PartialEq, Eq, Serialize, Deserialize)]
pub(super) struct FileStamp {
    canonical: PathBuf,
    fingerprint: Fingerprint,
}

impl FileStamp {
    pub(super) fn matches(&self, path: &Path) -> bool {
        path.canonicalize()
            .is_ok_and(|current| current == self.canonical)
            && read(path).is_ok_and(|bytes| Fingerprint::of(&bytes) == self.fingerprint)
    }
}

impl Inputs {
    pub(super) fn capture(&mut self, path: &Path) -> Result<&[u8], String> {
        let key = path
            .canonicalize()
            .map_err(|e| format!("{}: {e}", path.display()))?;
        let requested = std::path::absolute(path).map_err(|e| e.to_string())?;
        match self.1.entry(requested) {
            std::collections::btree_map::Entry::Vacant(slot) => {
                slot.insert(key.clone());
            }
            std::collections::btree_map::Entry::Occupied(slot) if *slot.get() != key => {
                return Err(format!("изменилась цель ссылки {}", path.display()));
            }
            _ => (),
        }
        if let std::collections::btree_map::Entry::Vacant(entry) = self.0.entry(key.clone()) {
            entry.insert(read(&key)?.into());
        }
        Ok(&self.0[&key])
    }

    pub(super) fn toml<T: DeserializeOwned>(&mut self, path: &Path) -> Result<T, String> {
        let bytes = self.capture(path)?;
        ensure_finite(&parse(path, bytes)?)?;
        parse(path, bytes)
    }

    pub(super) fn verify(&self) -> Result<(), String> {
        for (request, canonical) in &self.1 {
            if request.canonicalize().ok().as_ref() != Some(canonical) {
                return Err(format!("изменилась цель ссылки {}", request.display()));
            }
        }
        for (path, bytes) in &self.0 {
            if read(path)? != bytes.as_ref() {
                return Err(format!(
                    "вход изменился во время расчёта: {}",
                    path.display()
                ));
            }
        }
        Ok(())
    }

    pub(super) fn stamps(&self) -> BTreeMap<PathBuf, FileStamp> {
        self.1
            .iter()
            .map(|(request, canonical)| {
                (
                    request.clone(),
                    FileStamp {
                        canonical: canonical.clone(),
                        fingerprint: Fingerprint::of(&self.0[canonical]),
                    },
                )
            })
            .collect()
    }

    pub(super) fn fingerprints(&self) -> BTreeMap<String, Fingerprint> {
        self.0
            .iter()
            .map(|(path, bytes)| (path.to_string_lossy().into_owned(), Fingerprint::of(bytes)))
            .collect()
    }

    fn merge(&mut self, other: Self) -> Result<(), String> {
        for (request, canonical) in other.1 {
            if self
                .1
                .insert(request.clone(), canonical.clone())
                .is_some_and(|old| old != canonical)
            {
                return Err(format!("изменилась цель ссылки {}", request.display()));
            }
        }
        for (path, bytes) in other.0 {
            match self.0.entry(path) {
                std::collections::btree_map::Entry::Vacant(entry) => {
                    entry.insert(bytes);
                }
                std::collections::btree_map::Entry::Occupied(entry) => {
                    if *entry.get() != bytes {
                        return Err(format!(
                            "вход изменился между прогонами: {}",
                            entry.key().display()
                        ));
                    }
                }
            }
        }
        Ok(())
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Profile {
    version: u32,
    source: String,
    cases: BTreeMap<String, CaseEntry>,
}

#[derive(Deserialize)]
#[serde(untagged)]
enum CaseEntry {
    File(PathBuf),
    Overlay(Overlay),
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Overlay {
    file: PathBuf,
    overlay: PathBuf,
    #[serde(default)]
    remove_models: BTreeSet<String>,
    #[serde(default)]
    remove_reference: bool,
}

impl CaseEntry {
    fn load(self, base: &Path, inputs: &mut Inputs) -> Result<(PathBuf, Case), String> {
        match self {
            Self::File(path) => {
                let file = base.join(path);
                let case = inputs.toml(&file)?;
                Ok((file, case))
            }
            Self::Overlay(entry) => {
                let file = base.join(entry.file);
                let mut table: toml::Table = inputs.toml(&file)?;
                let patch: toml::Table = inputs.toml(&base.join(entry.overlay))?;
                merge_tables(&mut table, patch);
                if entry.remove_reference {
                    if table.remove("reference").is_none() {
                        return Err("не найдено удаляемое сравнение с эталоном".into());
                    }
                    table.remove("compare_with");
                }
                for name in entry.remove_models {
                    if table
                        .get_mut("models")
                        .and_then(toml::Value::as_table_mut)
                        .and_then(|models| models.remove(&name))
                        .is_none()
                    {
                        return Err(format!("не найдена удаляемая модель «{name}»"));
                    }
                }
                let case = toml::Value::Table(table)
                    .try_into()
                    .map_err(|e| format!("{}: {e}", file.display()))?;
                Ok((file, case))
            }
        }
    }
}

pub(super) fn merge_tables(base: &mut toml::Table, patch: toml::Table) {
    for (name, value) in patch {
        if let (Some(toml::Value::Table(base)), toml::Value::Table(patch)) =
            (base.get_mut(&name), &value)
        {
            merge_tables(base, patch.clone());
        } else {
            base.insert(name, value);
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Case {
    window_ns: [u64; 2],
    snapshot_ns: Option<u64>,
    source: String,
    world: Option<PathBuf>,
    joint_limit: Option<JointLimit>,
    transmission_time_constant_s: Option<Scalar>,
    contacts: Option<ContactInput>,
    #[serde(default)]
    models: BTreeMap<String, models::Spec>,
    #[serde(default)]
    inventory: BTreeMap<String, Vec<models::Kind>>,
    #[serde(default)]
    model_mounts: BTreeMap<String, String>,
    control: Option<Control>,
    reference: Option<reference::Reference>,
    compare_with: Option<String>,
    criteria: Vec<Metric>,
    #[serde(default)]
    events: Vec<ExpectedEvent>,
    #[serde(default)]
    states: BTreeMap<String, Vec<String>>,
    #[serde(default)]
    checks: Vec<WindowCheck>,
    #[serde(default)]
    hazard_witnesses: Vec<HazardWitness>,
    screen: Option<screen::Screen>,
    measured_result: Option<PathBuf>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct HazardWitness {
    class: String,
    object: String,
    criterion: String,
}

fn conductor_finding(
    case: &Case,
    plan: &ConvergencePlan,
    report: &ConvergenceReport,
    run: &sim_task::campaign::RunKey,
    witness: &HazardWitness,
) -> Option<sim_task::verdict::Finding> {
    use sim_task::verdict::HazardClass;

    let models::Spec::Electrical { conductor_heat: Some(heat), .. } =
        case.models.get(&witness.object)?
    else {
        return None;
    };
    let criterion = plan.criteria.iter().find(|criterion| criterion.name == witness.criterion)?;
    let signal = format!("{}/conductor_k", witness.object);
    if case.model_mounts.contains_key(&witness.object)
        || !matches!(&criterion.series, Series::Measurement(name) if name == &signal)
        || criterion.statistic != Statistic::Maximum
        || criterion.unit != "K"
        || criterion.lower.is_some()
        || criterion.upper.as_ref() != Some(&SourcedRange::from(heat.max_k.clone()))
        || report.trials.is_empty()
        || report.trials.iter().any(|trial| !trial.completed || trial.steps == 0)
    {
        return None;
    }
    let mut finding = report.campaign_findings(run).into_iter().find(|finding| {
        finding.key.class == HazardClass::TaskCriterion && finding.code == witness.criterion
    })?;
    let required = finding.measurement?;
    if finding.outcome == Outcome::NotAnalyzed
        || finding.outcome == Outcome::Fail && required[0] <= heat.max_k.value
        || finding.outcome == Outcome::Pass && required[1] > heat.max_k.value
        || finding.event_ns.is_none()
        || finding.journal.is_none()
    {
        return None;
    }
    finding.key.class = HazardClass::WiringHeat;
    finding.key.object.clone_from(&witness.object);
    finding.group = finding.key.class.spec().group;
    finding.reason = "температура теплоизолированного проводника по диссипации I²R".into();
    finding.verification = finding.key.class.spec().independent_check.into();
    finding.missing_data.clear();
    finding.confirmed_hazard = finding.outcome == Outcome::Fail;
    Some(finding)
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct JointLimit {
    time_constant_s: Scalar,
    damping_ratio: Scalar,
    impedance: Option<[Scalar; 5]>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct WindowCheck {
    name: String,
    signal: String,
    unit: String,
    window_ns: [u64; 2],
    min: Option<Scalar>,
    max: Option<Scalar>,
    drift: Option<DriftWindow>,
    evidence: Option<WindowEvidence>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct WindowEvidence {
    signal: String,
    unit: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct DriftWindow {
    signal: String,
    window_ns: [u64; 2],
}

#[derive(Serialize)]
struct CheckReport {
    name: String,
    rate_hz: f64,
    variant: String,
    samples: usize,
    observed: Option<[f64; 2]>,
    drift_allowance_m: Option<f64>,
    checked: Option<[f64; 2]>,
    outcome: &'static str,
}

impl WindowCheck {
    fn sample(runtime: &Runtime, range: Interval, signal: &str, unit: &str) -> WindowSamples {
        let mut observed: Option<[f64; 2]> = None;
        let mut first = None;
        let mut samples = 0;
        let mut missing = false;
        let mut boundaries = [None, None];
        for tick in runtime
            .process_journal()
            .iter()
            .filter(|tick| tick.at >= range.start && tick.at <= range.end)
        {
            boundaries[0].get_or_insert(tick.at);
            boundaries[1] = Some(tick.at);
            let value = tick
                .outputs
                .values()
                .flat_map(|out| &out.measurements)
                .find(|sample| sample.signal.name == signal && sample.signal.unit == unit);
            if let Some(sim_process::Analysis::Analyzed { value, .. }) = value.map(|x| &x.result) {
                samples += 1;
                first.get_or_insert(*value);
                let [lo, hi] = observed.unwrap_or([*value; 2]);
                observed = Some([lo.min(*value), hi.max(*value)]);
            } else {
                missing = true;
            }
        }
        WindowSamples {
            observed,
            first,
            samples,
            complete: !missing
                && boundaries == [Some(range.start), Some(range.end)]
                && samples >= 2,
        }
    }

    fn inspect(&self, rate: Rate, variant: &TrialVariant, runtime: &Runtime) -> CheckReport {
        let data = Self::sample(runtime, interval(self.window_ns), &self.signal, &self.unit);
        let evidence = self.evidence.as_ref().map(|evidence| {
            Self::sample(
                runtime,
                interval(self.window_ns),
                &evidence.signal,
                &evidence.unit,
            )
        });
        let drift_allowance_m = self.drift.as_ref().and_then(|drift| {
            let evidence = self.evidence.as_ref()?;
            let load = Self::sample(
                runtime,
                interval(drift.window_ns),
                &evidence.signal,
                &evidence.unit,
            );
            if !load.complete || !load.observed.is_some_and(|[low, _]| low > 0.0) {
                return None;
            }
            let series = Self::sample(runtime, interval(drift.window_ns), &drift.signal, "m");
            if !series.complete {
                return None;
            }
            let ([lo, hi], first) = series.observed.zip(series.first)?;
            Some((first - lo).max(hi - first))
        });
        let checked = data.observed.map(|[lo, hi]| {
            let extra = drift_allowance_m.unwrap_or(0.0);
            [lo - extra, hi + extra]
        });
        let outside = |[lo, hi]: [f64; 2]| {
            self.min.as_ref().is_some_and(|x| lo < x.value)
                || self.max.as_ref().is_some_and(|x| hi > x.value)
        };
        let outcome = if data.observed.is_some_and(outside)
            || evidence
                .as_ref()
                .is_some_and(|e| e.observed.is_some_and(|[lo, _]| lo <= 0.0))
        {
            "violated"
        } else if !data.complete
            || evidence.as_ref().is_some_and(|e| !e.complete)
            || (self.drift.is_some() && drift_allowance_m.is_none())
            || checked.is_some_and(outside)
            || checked.is_some_and(|range| !range.into_iter().all(f64::is_finite))
        {
            // Наблюдение внутри порога, а граница с учётом измеренной
            // погрешности за порогом: положительный исход не подтверждён.
            "missing"
        } else {
            "satisfied"
        };
        CheckReport {
            name: self.name.clone(),
            rate_hz: rate.as_hz(),
            variant: variant.name.clone(),
            samples: data.samples,
            observed: data.observed,
            drift_allowance_m,
            checked,
            outcome,
        }
    }
}

struct WindowSamples {
    observed: Option<[f64; 2]>,
    first: Option<f64>,
    samples: usize,
    complete: bool,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Control {
    wiring: PathBuf,
    packet_wiring: Option<PathBuf>,
    overlay: Option<PathBuf>,
    #[serde(default)]
    pub(super) measurements: Vec<measurements::Measurement>,
    #[serde(default)]
    pub(super) readouts: Vec<readouts::Readout>,
    #[serde(default)]
    sampled_drives: BTreeMap<String, PathBuf>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct ContactInput {
    pub(super) table: PathBuf,
    pub(super) solver: Option<Solver>,
    alternative_solver: Option<Solver>,
    #[serde(default)]
    pub(super) exclusions: Vec<[String; 2]>,
    pub(super) assignments: BTreeMap<String, String>,
    pub(super) pairs: Vec<Calibration>,
}

#[derive(Clone, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Method {
    Newton,
    ConjugateGradient,
    ProjectedGaussSeidel,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Solver {
    method: Method,
    elliptic: bool,
    impratio: f64,
    tolerance: f64,
    iterations: u32,
    line_search_tolerance: f64,
    line_search_iterations: u32,
    noslip_tolerance: f64,
    noslip_iterations: u32,
    ccd_tolerance: f64,
    ccd_iterations: u32,
}

impl From<&Solver> for ContactSolver {
    fn from(value: &Solver) -> Self {
        Self {
            solver: match value.method {
                Method::Newton => ConstraintSolver::Newton,
                Method::ConjugateGradient => ConstraintSolver::ConjugateGradient,
                Method::ProjectedGaussSeidel => ConstraintSolver::ProjectedGaussSeidel,
            },
            cone: if value.elliptic {
                FrictionCone::Elliptic
            } else {
                FrictionCone::Pyramidal
            },
            impratio: value.impratio,
            tolerance: value.tolerance,
            iterations: value.iterations,
            line_search_tolerance: value.line_search_tolerance,
            line_search_iterations: value.line_search_iterations,
            noslip_tolerance: value.noslip_tolerance,
            noslip_iterations: value.noslip_iterations,
            ccd_tolerance: value.ccd_tolerance,
            ccd_iterations: value.ccd_iterations,
        }
    }
}

#[derive(Clone, Deserialize)]
#[serde(rename_all = "snake_case", remote = "Provenance")]
pub(super) enum Origin {
    Datasheet(String),
    Paper(String),
    Measured(String),
    Derived(String),
    Estimate(String),
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Scalar {
    pub(super) value: f64,
    range: Option<[f64; 2]>,
    #[serde(with = "Origin")]
    source: Provenance,
}

impl From<Scalar> for SourcedRange {
    fn from(s: Scalar) -> Self {
        let [min, max] = s.range.unwrap_or([s.value; 2]);
        Self {
            min,
            nominal: s.value,
            max,
            provenance: s.source,
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Calibration {
    geometries: [String; 2],
    solimp: [f64; 5],
    #[serde(with = "Origin")]
    source: Provenance,
    mass_kg: Option<Scalar>,
    friction_damping: Option<Scalar>,
}

impl From<&Calibration> for PairCalibration {
    fn from(c: &Calibration) -> Self {
        Self {
            geometries: c.geometries.clone(),
            solimp: c.solimp,
            impedance_source: c.source.clone(),
            effective_mass_kg: c.mass_kg.clone().map(Into::into),
            friction_reference: c
                .friction_damping
                .clone()
                .map_or(FrictionReference::UseNormal, |s| {
                    FrictionReference::DirectDamping(s.into())
                }),
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct PairTable {
    pub(super) version: u32,
    pub(super) entries: Vec<PairRow>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct PairRow {
    materials: [String; 2],
    sliding: Option<[Scalar; 2]>,
    torsion_m: Option<Scalar>,
    rolling_m: Option<[Scalar; 2]>,
    normal: Normal,
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Normal {
    Restitution {
        coefficient: Scalar,
        duration_s: Scalar,
    },
    SpringDamper {
        stiffness_n_m: Scalar,
        damping_ns_m: Scalar,
    },
    Direct {
        stiffness_per_s2: Scalar,
        damping_per_s: Scalar,
    },
}

impl PairRow {
    pub(super) fn parameters(self) -> Result<([String; 2], PairParameters), String> {
        let friction = match (self.sliding, self.torsion_m, self.rolling_m) {
            (None, None, None) => ContactFriction::NormalOnly,
            (Some(sliding), None, None) => ContactFriction::Sliding {
                sliding: sliding.map(Into::into),
            },
            (Some(sliding), Some(torsion_m), None) => ContactFriction::SlidingAndTorsion {
                sliding: sliding.map(Into::into),
                torsion_m: torsion_m.into(),
            },
            (Some(sliding), Some(torsion_m), Some(rolling_m)) => {
                ContactFriction::SlidingTorsionAndRolling {
                    sliding: sliding.map(Into::into),
                    torsion_m: torsion_m.into(),
                    rolling_m: rolling_m.map(Into::into),
                }
            }
            _ => return Err("несовместимые компоненты трения".into()),
        };
        let normal = match self.normal {
            Normal::Restitution {
                coefficient,
                duration_s,
            } => NormalResponse::Restitution {
                coefficient: coefficient.into(),
                duration_s: duration_s.into(),
            },
            Normal::SpringDamper {
                stiffness_n_m,
                damping_ns_m,
            } => NormalResponse::SpringDamper {
                stiffness_n_m: stiffness_n_m.into(),
                damping_ns_m: damping_ns_m.into(),
            },
            Normal::Direct {
                stiffness_per_s2,
                damping_per_s,
            } => NormalResponse::Direct {
                stiffness_per_s2: stiffness_per_s2.into(),
                damping_per_s: damping_per_s.into(),
            },
        };
        Ok((self.materials, PairParameters { friction, normal }))
    }
}

#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
enum Reducer {
    Minimum,
    Maximum,
    AbsolutePeak,
    Integral,
    End,
    Continuous,
}

#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
enum Sample {
    NormalForce([String; 2]),
    Penetration([String; 2]),
    Measurement(String),
    Position(usize),
    Velocity(usize),
    Displacement(usize),
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Metric {
    name: String,
    unit: String,
    series: Sample,
    statistic: Reducer,
    tolerance: Scalar,
    lower: Option<Scalar>,
    upper: Option<Scalar>,
    remainder: [Scalar; 3],
}

impl From<Metric> for Criterion {
    fn from(m: Metric) -> Self {
        let [discretization, solver, materials] = m.remainder.map(Into::into);
        Self {
            name: m.name,
            unit: m.unit,
            series: match m.series {
                Sample::NormalForce(pair) => Series::NormalForce(pair),
                Sample::Penetration(pair) => Series::Penetration(pair),
                Sample::Measurement(name) => Series::Measurement(name),
                Sample::Position(i) => Series::Position(i),
                Sample::Velocity(i) => Series::Velocity(i),
                Sample::Displacement(i) => Series::Displacement(i),
            },
            statistic: match m.statistic {
                Reducer::Minimum => Statistic::Minimum,
                Reducer::Maximum => Statistic::Maximum,
                Reducer::AbsolutePeak => Statistic::AbsolutePeak,
                Reducer::Integral => Statistic::Integral,
                Reducer::End => Statistic::End,
                Reducer::Continuous => Statistic::Continuous,
            },
            tolerance: m.tolerance.into(),
            lower: m.lower.map(Into::into),
            upper: m.upper.map(Into::into),
            remainder: ErrorBounds {
                discretization,
                solver,
                materials,
            },
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ExpectedEvent {
    name: String,
    state: String,
    requested_ns: Option<u64>,
    applied_ns: [u64; 2],
    phase_tolerance_ns: Scalar,
}

fn interval([start, end]: [u64; 2]) -> Interval {
    Interval {
        start: Time(start),
        end: Time(end),
    }
}

pub(super) fn dir(path: &Path) -> &Path {
    path.parent().unwrap_or_else(|| Path::new("."))
}

pub(super) struct Wiring<'a> {
    pub(super) control: &'a Control,
    pub(super) base: PathBuf,
    pub(super) text: String,
    pub(super) sampled: BTreeMap<String, sim_drive::sampled::Spec>,
    pub(super) models: Models,
    pub(super) assets: BTreeMap<String, u64>,
}

impl<'a> Wiring<'a> {
    pub(super) fn load(
        control: &'a Control,
        base: &Path,
        inputs: &mut Inputs,
        options: &Options,
    ) -> Result<Self, String> {
        let file = base.join(&control.wiring);
        let mut text = std::str::from_utf8(inputs.capture(&file)?)
            .map_err(|e| e.to_string())?
            .to_owned();
        for overlay in control.packet_wiring.iter().chain(&control.overlay) {
            text.push('\n');
            text.push_str(
                std::str::from_utf8(inputs.capture(&base.join(overlay))?)
                    .map_err(|e| e.to_string())?,
            );
        }
        let sampled = control
            .sampled_drives
            .iter()
            .map(|(name, file)| {
                inputs
                    .toml(&base.join(file))
                    .map(|spec| (name.clone(), spec))
            })
            .collect::<Result<_, _>>()?;
        let assets = sim_runtime::control::ControlConfig::parse(&text)?
            .assets()?
            .into_iter()
            .map(|name| {
                let bytes = inputs.capture(&dir(&file).join(&name))?;
                Ok((name, sim_signal::fnv1a(bytes.iter().copied())))
            })
            .collect::<Result<_, String>>()?;
        Ok(Self {
            control,
            base: dir(&file).to_owned(),
            text,
            sampled,
            models: Models::load(&catalog_path(options)?)?,
            assets,
        })
    }
}

#[derive(Serialize)]
struct CaseReport {
    outcome: Outcome,
    convergence: ConvergenceReport,
    reference: Option<reference::Comparison>,
    #[serde(skip_serializing_if = "Option::is_none")]
    model_sensitivity: Option<Sensitivity>,
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    packets: BTreeMap<String, PacketCounts>,
    diagnostics: Vec<String>,
    checks: Vec<CheckReport>,
    world: BTreeMap<u64, Fingerprint>,
    control_assets: BTreeMap<String, Fingerprint>,
    #[serde(skip)]
    hazard_witnesses: Vec<sim_task::verdict::Finding>,
    #[serde(skip)]
    seed: u64,
    #[serde(skip)]
    catalog: Option<Fingerprint>,
    #[serde(skip)]
    checkpointable: bool,
    #[serde(skip)]
    inventory: Option<LinkedInventory>,
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    installations: BTreeMap<String, Receipt>,
}

#[derive(Default, Serialize)]
struct PacketCounts {
    retries: u64,
    lost: u64,
    delivered: u64,
}

#[derive(Serialize)]
struct Sensitivity {
    against: String,
    curves: Vec<reference::ModelDifference>,
}

#[derive(Serialize)]
struct Report {
    status: &'static str,
    outcome: Outcome,
    inputs: BTreeMap<String, Fingerprint>,
    cases: BTreeMap<String, CaseReport>,
}

fn build_world(
    world: &WorldInputs,
    seed: Option<u64>,
    rate: Rate,
    case: &Case,
    table: Option<&ContactMaterialTable>,
    variant: VariantKind,
    sample: Option<LinkedSample<'_>>,
    wiring: Option<&Wiring<'_>>,
) -> Result<
    (
        sim_world::CompiledWorld,
        Vec<sim_process::contact::ResolvedContactPair>,
        WorldInputs,
        ModelSettings,
    ),
    String,
> {
    let mut build = ContactBuildOptions {
        physics: Some(rate),
        ..Default::default()
    };
    if let Some(time) = &case.transmission_time_constant_s {
        SourcedRange::from(time.clone())
            .validate()
            .map_err(|e| e.to_string())?;
        build.transmission_time_constant_s = Some(time.value);
    }
    if let Some(limit) = &case.joint_limit {
        for s in [&limit.time_constant_s, &limit.damping_ratio] {
            SourcedRange::from(s.clone())
                .validate()
                .map_err(|e| e.to_string())?;
        }
        build.joint_limit_response = Some([limit.time_constant_s.value, limit.damping_ratio.value]);
        if let Some(impedance) = &limit.impedance {
            for value in impedance {
                SourcedRange::from(value.clone())
                    .validate()
                    .map_err(|e| e.to_string())?;
            }
            build.joint_limit_impedance = Some(impedance.each_ref().map(|s| s.value));
        }
    }
    let contacts = case.contacts.as_ref();
    if let Some(c) = contacts {
        build.excluded_bodies.clone_from(&c.exclusions);
        build.solver = match variant {
            VariantKind::Solver => c.alternative_solver.as_ref(),
            _ => c.solver.as_ref(),
        }
        .map(Into::into);
        if build.solver.is_none() {
            return Err("для таблицы контактов нужен явный решатель".into());
        }
    }
    let mut inputs = world.clone();
    inputs.world.physics_hz = rate.as_hz() as u64;
    if let Some(seed) = seed {
        inputs.world.seed = seed;
    }
    inputs.contact = ContactBuildOptions {
        solver: None,
        ..build.clone()
    };
    let base = inputs.compile().map_err(|e| e.to_string())?;
    let mut pairs = if let (Some(c), Some(table), Some(solver)) = (contacts, table, build.solver) {
        let assignments = c
            .assignments
            .iter()
            .map(|(geometry, material)| GeometryMaterial {
                geometry: geometry.clone(),
                material: material.clone(),
            })
            .collect::<Vec<_>>();
        let calibrations = c.pairs.iter().map(Into::into).collect::<Vec<_>>();
        let pairs = table
            .resolve(&base, &assignments, &calibrations, solver.cone)
            .map_err(|e| e.to_string())?;
        build.pairs = pairs.iter().map(|entry| entry.pair.clone()).collect();
        pairs
    } else {
        Vec::new()
    };
    inputs.contact = build;
    let settings = if let Some(sample) = sample {
        let wires = wiring
            .map(|w| sim_runtime::control::ControlConfig::parse(&w.text)?.wired_delays())
            .transpose()?
            .unwrap_or_default();
        let settings = sample.overlay(&base, &mut inputs, wiring.map(|w| &w.models), &wires)?;
        sample.update_contact_pairs(&mut pairs)?;
        settings
    } else {
        ModelSettings::default()
    };
    let compiled =
        if sample.is_none() && inputs.contact.solver.is_none() && inputs.contact.pairs.is_empty() {
            base
        } else {
            inputs.compile().map_err(|e| e.to_string())?
        };
    Ok((compiled, pairs, inputs, settings))
}

fn run_case(
    mode: &str,
    mut case: Case,
    case_file: &Path,
    options: &Options,
    inputs: &mut Inputs,
    sample: Option<LinkedSample<'_>>,
    inventory_requested: bool,
) -> Result<CaseReport, String> {
    let base = dir(case_file);
    let world_path = case
        .world
        .as_ref()
        .map_or_else(|| options.world.clone(), |p| base.join(p));
    let mut world = capture_world(&world_path, inputs)?;
    if let Some(hz) = options.linked_grid_hz {
        if !world_path.canonicalize().is_ok_and(|path| path == options.world) {
            return Err("ссылочная постановка процессов подменила мир кампании".into());
        }
        world.physics_hz = hz;
    }
    let world_inputs = WorldInputs::load(&world_path).map_err(|e| e.to_string())?;
    let reference = case
        .reference
        .as_ref()
        .map(|reference| {
            reference.validate()?;
            let text = std::str::from_utf8(inputs.capture(&base.join(&reference.file))?)
                .map_err(|e| e.to_string())?;
            reference.validate_text(text)?;
            Ok::<_, String>(text.to_owned())
        })
        .transpose()?;
    let table = case
        .contacts
        .as_ref()
        .map(|contact| {
            let table: PairTable = inputs.toml(&base.join(&contact.table))?;
            if table.version != 1 {
                return Err("неподдерживаемая версия таблицы контактов".into());
            }
            let rows = table
                .entries
                .into_iter()
                .map(PairRow::parameters)
                .collect::<Result<Vec<_>, _>>()?;
            ContactMaterialTable::new(rows).map_err(|e| e.to_string())
        })
        .transpose()?;
    let wiring = case
        .control
        .as_ref()
        .map(|control| Wiring::load(control, base, inputs, options))
        .transpose()?;
    let window = interval(case.window_ns);
    let child = super::campaign::profile_key("", mode, mode, Vec::new());
    let measured = case
        .measured_result
        .as_ref()
        .map(|file| {
            let path = base.join(file);
            let journal = Fingerprint::of(inputs.capture(&path)?);
            let result: sim_task::measured::MeasuredResult = inputs.toml(&path)?;
            result
                .assess(&child, case.window_ns, journal)
                .map_err(|err| format!("{}: {err}", path.display()))
        })
        .transpose()?;
    if let Some(screen) = &case.screen {
        screen.validate(&case)?;
    }
    if case
        .snapshot_ns
        .is_some_and(|at| at == 0 || at >= window.end.0)
    {
        return Err("такт снимка должен быть внутри прогона процессов".into());
    }
    let mut names = BTreeSet::new();
    for check in &case.checks {
        let [start, end] = check.window_ns;
        if check.name.trim().is_empty()
            || !names.insert(&check.name)
            || check.signal.trim().is_empty()
            || check.unit.trim().is_empty()
            || start >= end
            || start < window.start.0
            || end > window.end.0
            || (check.min.is_none() && check.max.is_none())
            || check
                .min
                .as_ref()
                .zip(check.max.as_ref())
                .is_some_and(|(l, h)| l.value > h.value)
            || check
                .evidence
                .as_ref()
                .is_some_and(|e| e.signal.trim().is_empty() || e.unit.trim().is_empty())
            || check.drift.as_ref().is_some_and(|d| {
                let [d_start, d_end] = d.window_ns;
                check.unit != "m"
                    || d.signal.trim().is_empty()
                    || d_end.checked_sub(d_start) != Some(2_000_000_000) // Выдержка 2 с по § 4.6.
                    || d_start < start
                    || d_end > end
                    || check.evidence.is_none()
            })
        {
            return Err("неверное окно проверки или пороги ряда процессов".into());
        }
        for source in check.min.iter().chain(check.max.iter()) {
            SourcedRange::from(source.clone())
                .validate()
                .map_err(|e| e.to_string())?;
        }
    }
    let mut variants = vec![TrialVariant {
        name: "основной".into(),
        kind: VariantKind::Baseline,
        materials: BTreeMap::new(),
    }];
    if case
        .contacts
        .as_ref()
        .is_some_and(|c| c.alternative_solver.is_some())
    {
        variants.push(TrialVariant {
            name: "решатель".into(),
            kind: VariantKind::Solver,
            materials: BTreeMap::new(),
        });
    }
    let plan = ConvergencePlan {
        base_hz: world.physics_hz,
        window,
        window_source: std::mem::take(&mut case.source),
        variants,
        material_ranges: BTreeMap::new(),
        criteria: std::mem::take(&mut case.criteria)
            .into_iter()
            .map(Into::into)
            .collect(),
        events: std::mem::take(&mut case.events)
            .into_iter()
            .map(|e| EventExpectation {
                name: e.name,
                state: e.state,
                requested: e.requested_ns.map(Time),
                applied_window: interval(e.applied_ns),
                phase_tolerance_ns: e.phase_tolerance_ns.into(),
            })
            .collect(),
        states: std::mem::take(&mut case.states)
            .into_iter()
            .map(|(name, sequence)| StateExpectation { name, sequence })
            .collect(),
        creep: Vec::new(),
    };
    let subjects = case
        .inventory
        .iter()
        .map(|(name, kinds)| WorkSubject {
            name: name.clone(),
            properties: kinds.iter().map(|kind| kind.process_kind()).collect(),
        })
        .collect::<Vec<_>>();
    if world.attachments.iter().any(|a| {
        a.mode == sim_world::AttachmentMode::Runtime
            && case.inventory.get(&a.name).is_none_or(Vec::is_empty)
            && !case
                .models
                .values()
                .any(|model| model.uses_runtime_attachment(&a.name))
    }) || case
        .model_mounts
        .keys()
        .any(|m| case.models.get(m).and_then(models::Spec::kind).is_none())
    {
        return Err("неполный состав сменных насадок или неверная привязка процесса".into());
    }
    let bindings = case
        .models
        .iter()
        .filter_map(|(name, spec)| {
            spec.kind().map(|kind| ProcessBinding {
                name: name.clone(),
                kind,
                attachment: case.model_mounts.get(name).cloned(),
                ports: BTreeMap::new(),
            })
        })
        .collect::<Vec<_>>();
    let mut stores_by_subject = BTreeMap::new();
    for binding in &bindings {
        if binding.kind == sim_process::work::ProcessKind::StoredEnergy {
            let subject = binding.attachment.as_deref().unwrap_or(&binding.name);
            *stores_by_subject.entry(subject).or_insert(0usize) += 1;
        }
    }
    let mut screened = Vec::new();
    let mut configuration = Vec::new();
    for bytes in inputs.0.values() {
        frame(&mut configuration, bytes);
    }
    let mut worlds = BTreeMap::new();
    let mut assets = BTreeMap::new();
    let mut comparison = None;
    let mut diagnostics = Vec::new();
    let mut view_export = None;
    let mut checks = Vec::new();
    let mut packets = BTreeMap::<String, PacketCounts>::new();
    let mut installations = BTreeMap::new();
    let mut inventory = None;
    let mut checkpointable = true;
    let mut missing_installations = Vec::new();
    let mut convergence = assess_processes_with_snapshot(
        &plan,
        case.snapshot_ns.map(Time),
        |rate, variant| {
            let (compiled, pairs, resolved, settings) = build_world(
                &world_inputs,
                options.seed,
                rate,
                &case,
                table.as_ref(),
                variant.kind,
                sample,
                wiring.as_ref(),
            )?;
            if variant.kind == VariantKind::Baseline {
                worlds.insert(
                    rate.as_hz() as u64,
                    Fingerprint::of(compiled.mjcf.as_bytes()),
                );
            }
            let mut runtime = Runtime::load(compiled.clone())?;
            if rate.as_hz() == plan.base_hz as f64 && variant.kind == VariantKind::Baseline {
                options.record_view(&mut runtime)?;
            }
            if let Some(other) = &case.compare_with {
                runtime.add_simplification(sim_world::Simplification {
                    subject: "сравнение моделей процесса".into(),
                    text: format!(
                        "Нужна валидация чувствительности к модели относительно «{other}»."
                    ),
                    blocks_proof: true,
                })?;
            }
            runtime.record_process_contacts(pairs)?;
            runtime.set_process_inventory(&subjects, &bindings)?;
            for (&subject, _) in stores_by_subject.iter().filter(|(_, count)| **count > 1) {
                runtime.add_simplification(sim_world::Simplification {
                    subject: subject.into(),
                    text:
                        "совместный выброс энергии нескольких накопителей требует отдельной модели"
                            .into(),
                    blocks_proof: true,
                })?;
            }
            if let Some(w) = &wiring {
                let setup = Setup {
                    world: &compiled,
                    readouts: &w.control.readouts,
                    measurements: &w.control.measurements,
                    sampled: w.sampled.clone(),
                    sampled_modes: &BTreeMap::new(),
                    models: Models {
                        drives: w.models.drives.clone(),
                        sensors: w.models.sensors.clone(),
                        bytes: w.models.bytes.clone(),
                    },
                    task_bytes: configuration.clone(),
                    world_bytes: compiled.mjcf.as_bytes().to_vec(),
                    base: w.base.clone(),
                    control: w.text.clone(),
                    overlays: BTreeMap::new(),
                };
                setup.install(&mut runtime, mode, window.end.0, "")?;
                let run =
                    sim_runtime::control::run(&runtime).ok_or("нет установленного контура")?;
                if run.identity().assets.iter().cloned().collect::<BTreeMap<_, _>>() != w.assets {
                    return Err("ресурсы установленного контура изменились".into());
                }
                for (path, hash) in &run.identity().assets {
                    let fingerprint = Fingerprint::from_fnv1a(*hash);
                    if assets
                        .insert(path.clone(), fingerprint)
                        .is_some_and(|before| before != fingerprint)
                    {
                        return Err(format!("ресурс контура изменился: {path}"));
                    }
                }
            } else {
                runtime.record_joint_state("запись состояния")?;
            }
            for (name, model) in &case.models {
                model.validate_links(&case.models, &compiled)?;
                let subject = case
                    .model_mounts
                    .get(name)
                    .map_or(name.as_str(), String::as_str);
                model.install(name, &compiled, &mut runtime, &resolved.world, &world_path, subject)?;
            }
            if let Some(sample) = sample {
                let receipt = sample.install(&mut runtime, &compiled, &resolved, &settings)?;
                installations.insert(format!("{}/{}", variant.name, rate.as_hz()), receipt);
            }
            if inventory_requested && variant.kind == VariantKind::Baseline && rate.as_hz() as u64 == plan.base_hz {
                inventory = Some(super::campaign::installation::inventory(
                    &compiled, &resolved, &runtime,
                    serde_json::json!({
                        "subjects": subjects.iter().map(|s| (&s.name, format!("{:?}", s.properties))).collect::<BTreeMap<_, _>>(),
                        "models": bindings.iter().map(|b| (&b.name, (format!("{:?}", b.kind), &b.attachment))).collect::<BTreeMap<_, _>>(),
                    }),
                    case.models.iter().filter_map(|(name, model)| {
                        matches!(model, models::Spec::Electrical { conductor_heat: Some(_), .. })
                            .then(|| (sim_task::verdict::ComponentKind::Wire, name.clone()))
                    }),
                ));
            }
            Ok(runtime)
        },
        |rate, variant, runtime| {
            if sample.is_some_and(|s| {
                s.sample.values.iter().any(|v| matches!(&v.parameter,
                    sim_task::sampling::Parameter::ChannelDelay { channel }
                    if !runtime.channel_delay_observed(channel)))
            }) {
                missing_installations.push(format!("на сетке {} Гц нет доставки с выбранной задержкой", rate.as_hz()));
            }
            if sim_runtime::control::run(runtime).is_some_and(|r| r.has_external_processes()) {
                checkpointable = false;
            }
            checks.extend(
                case.checks
                    .iter()
                    .map(|check| check.inspect(rate, variant, runtime)),
            );
            if let Some(screen) = &case.screen {
                screened.push(screen.observe(&case, mode, runtime, reference.as_deref()));
            }
            if rate.as_hz() != plan.base_hz as f64 || variant.kind != VariantKind::Baseline {
                return;
            }
            if let Some(run) = sim_runtime::control::run(runtime) {
                use sim_runtime::control::{EventKind, PacketOutcome};
                for e in run.events() {
                    match &e.kind {
                        EventKind::Packet { route, event }
                            if event.outcome == PacketOutcome::Retry =>
                        {
                            packets.entry(route.clone()).or_default().retries += 1
                        }
                        EventKind::Packet { route, event }
                            if event.outcome == PacketOutcome::Lost =>
                        {
                            packets.entry(route.clone()).or_default().lost += 1
                        }
                        EventKind::PacketDelivered { route, .. } => {
                            packets.entry(route.clone()).or_default().delivered += 1
                        }
                        _ => (),
                    }
                }
            }
            if let Some((spec, data)) = case.reference.as_ref().zip(reference.as_ref()) {
                match spec.compare(data, runtime) {
                    Ok(result) => comparison = Some(result),
                    Err(error) => diagnostics.push(error),
                }
            }
            if options.view.is_some() {
                view_export = Some(crate::view::save(options, runtime, Some(mode), None));
            }
            let exported = (|| {
                if let Some(path) = &options.log {
                    export(path, |file| runtime.log().write_binary(file))?;
                }
                if let Some(path) = &options.csv {
                    export(path, |file| runtime.log().write_csv(file))?;
                }
                Ok::<_, String>(())
            })();
            if let Err(error) = exported {
                diagnostics.push(error);
            }
        },
    )
    .map_err(|e| e.to_string())?;
    if options.view.is_some() {
        view_export.ok_or("нет исходного прогона для выгрузки просмотра")??;
    }
    checkpointable &= convergence
        .trials
        .iter()
        .all(|trial| trial.snapshot.as_ref().is_none_or(|audit| audit.matched));
    let mut hazard_witnesses = Vec::new();
    if let Some(screen) = &case.screen {
        hazard_witnesses.extend(screen.findings(&case, &child, &convergence, screened));
    }
    if let Some(measured) = measured
        && !convergence.trials.is_empty()
        && convergence.trials.iter().all(|trial| trial.completed && trial.steps > 0)
    {
        hazard_witnesses.extend(measured);
    }
    for witness in &case.hazard_witnesses {
        let class = sim_task::verdict::REGISTRY
            .iter()
            .find(|spec| spec.id == witness.class)
            .ok_or_else(|| format!("неизвестный класс «{}»", witness.class))?
            .class;
        if !convergence
            .metrics
            .iter()
            .any(|m| m.name == witness.criterion)
        {
            return Err(format!(
                "свидетель ссылается на неизвестный критерий «{}»",
                witness.criterion
            ));
        }
        let finding = if class == sim_task::verdict::HazardClass::WiringHeat {
            conductor_finding(&case, &plan, &convergence, &child, witness)
        } else {
            convergence.hazard_witness(&child, class, &witness.object, &witness.criterion)
        };
        if let Some(finding) = finding {
            hazard_witnesses.push(finding);
        }
    }
    let unproven_checks = case.checks.iter().any(|check| {
        check.drift.is_some()
            || check.evidence.is_some()
            || check.window_ns != [window.start.0, window.end.0]
            || !plan.criteria.iter().any(|criterion| {
                matches!(&criterion.series, Series::Measurement(name) if name == &check.signal)
                    && criterion.unit == check.unit
                    && criterion.statistic == Statistic::Continuous
                    && criterion.lower == check.min.clone().map(Into::into)
                    && criterion.upper == check.max.clone().map(Into::into)
            })
    });
    if unproven_checks {
        convergence
            .missing
            .push("оконные проверки без валидации погрешности являются диагностическими".into());
        convergence.outcome = Outcome::aggregate([convergence.outcome, Outcome::NotAnalyzed]);
    }
    if case.reference.is_some() && comparison.is_none() {
        diagnostics.push("полная сверка с эталоном не выполнена".into());
    }
    let outcome = Outcome::aggregate(
        std::iter::once(convergence.outcome)
            .chain(hazard_witnesses.iter().map(|finding| finding.outcome))
            .chain(comparison.as_ref().filter(|r| !r.passed && !case.screen.as_ref().is_some_and(screen::Screen::measurement)).map(|_| {
                if case.reference.as_ref().is_some_and(|r| r.diagnostic_only)
                    && case.compare_with.is_some()
                {
                    Outcome::NotAnalyzed
                } else {
                    Outcome::Fail
                }
            }))
            .chain(checks.iter().filter_map(|check| match check.outcome {
                "violated" => Some(Outcome::Fail),
                "missing" => Some(Outcome::NotAnalyzed),
                _ => None,
            }))
            .chain(
                (!diagnostics.is_empty() || !missing_installations.is_empty())
                    .then_some(Outcome::NotAnalyzed),
            ),
    );
    diagnostics.extend(missing_installations);
    Ok(CaseReport {
        outcome,
        convergence,
        reference: comparison,
        model_sensitivity: None,
        packets,
        diagnostics,
        checks,
        world: worlds,
        control_assets: assets,
        hazard_witnesses,
        seed: sample.map_or_else(|| options.seed.unwrap_or(world.seed), |s| s.sample.seeds.run),
        catalog: wiring.as_ref().map(|w| Fingerprint::of(&w.models.bytes)),
        checkpointable,
        inventory,
        installations,
    })
}

pub(super) fn capture_world(path: &Path, inputs: &mut Inputs) -> Result<sim_world::World, String> {
    let text = std::str::from_utf8(inputs.capture(path)?).map_err(|e| e.to_string())?;
    let world = sim_world::parse_world(path, text)?;
    let mut mesh_paths = BTreeSet::new();
    let mut collect_meshes = |base: &Path, geoms: &[sim_world::Geometry]| {
        for geom in geoms {
            if let sim_world::Shape::Mesh { file, .. } = &geom.shape {
                mesh_paths.insert(base.join(file));
            }
        }
    };
    let world_dir = dir(path);
    collect_meshes(world_dir, &world.environment.collision);
    collect_meshes(world_dir, &world.environment.visual);
    for object in &world.objects {
        collect_meshes(world_dir, &object.collision);
        collect_meshes(world_dir, &object.visual);
    }
    for placement in &world.devices {
        let file = world_dir.join(&placement.file);
        inputs.capture(&file)?;
        let device = sim_world::load_device(&file, &sim_world::LegacyCatalog)?;
        for link in &device.links {
            collect_meshes(dir(&file), &link.collision);
            collect_meshes(dir(&file), &link.visual);
        }
    }
    for mesh in mesh_paths {
        inputs.capture(&mesh)?;
    }
    Ok(world)
}

pub(super) fn run(
    task: TaskFile,
    path: &Path,
    options: &Options,
    out: &mut impl Write,
) -> Result<u8, String> {
    run_observed(task, path, options, out, None, None)
}

pub(super) fn run_observed(
    task: TaskFile,
    path: &Path,
    options: &Options,
    out: &mut impl Write,
    mut audit: Option<&mut super::campaign::LinkedAudit>,
    sample: Option<LinkedSample<'_>>,
) -> Result<u8, String> {
    if options.matrix_full {
        return Err("серия процессов задаётся файлом профиля".into());
    }
    let mut inputs = Inputs::default();
    inputs.capture(path)?;
    let profile_path = dir(path).join(task.process_profile);
    let profile: Profile = inputs.toml(&profile_path)?;
    if profile.version != 1
        || profile.cases.is_empty()
        || profile.source.trim().is_empty()
        || profile
            .cases
            .keys()
            .any(|name| name.trim().is_empty() || name.chars().any(char::is_control))
    {
        return Err("нужен непустой профиль процессов с источником и версией 1".into());
    }
    if let Some(name) = options.case.as_ref().filter(|name| !profile.cases.contains_key(*name)) {
        let reason = format!("режим «{name}» отсутствует в профиле процессов");
        if let Some(audit) = audit.as_mut().filter(|_| options.linked_grid_hz.is_some()) {
            audit.record_unavailable(reason);
            return Ok(0);
        }
        return Err(reason);
    }
    if options.exports_run()
        && options.case.is_none()
        && profile.cases.len() > 1
    {
        return Err("для выгрузки журнала выберите режим через --case".into());
    }
    let mut cases = BTreeMap::new();
    let mut sensitivity = BTreeMap::new();
    let mut selected = Vec::new();
    for (mode, entry) in profile.cases {
        if options.case.as_ref().is_some_and(|name| *name != mode) {
            continue;
        }
        let mut case_inputs = inputs.clone();
        let (file, case) = entry.load(dir(&profile_path), &mut case_inputs)?;
        if let Some(other) = &case.compare_with {
            if other.trim().is_empty() || *other == mode || case.reference.is_none() {
                return Err("нужен другой режим и общая сетка для сравнения моделей".into());
            }
            sensitivity.insert(mode.clone(), other.clone());
        }
        selected.push((mode, file, case, case_inputs));
    }
    if sensitivity
        .values()
        .any(|name| options.case.is_none() && !selected.iter().any(|(mode, ..)| mode == name))
    {
        return Err("вариант для сравнения не найден в профиле процессов".into());
    }
    if audit
        .as_mut()
        .is_some_and(|audit| audit.omit_unselected(selected.len()))
    {
        inputs.verify()?;
        return Ok(0);
    }
    let jobs = options
        .jobs
        .map_or(1, std::num::NonZeroUsize::get)
        .min(selected.len());
    let inventory_requested = audit.is_some();
    let run_one = |(mode, file, case, mut files): (String, PathBuf, Case, Inputs)| {
        let report = run_case(
            &mode,
            case,
            &file,
            options,
            &mut files,
            sample,
            inventory_requested,
        );
        (mode, report, files)
    };
    let mut finished: Vec<_> = if jobs <= 1 {
        selected.into_iter().map(run_one).collect()
    } else {
        let pending = std::sync::Mutex::new(selected.into_iter());
        std::thread::scope(|scope| {
            let workers = (0..jobs)
                .map(|_| {
                    std::thread::Builder::new()
                        .spawn_scoped(scope, || {
                            let mut results = Vec::new();
                            loop {
                                let next = pending.lock().expect("очередь режимов").next();
                                let Some(next) = next else { break };
                                results.push(run_one(next));
                            }
                            results
                        })
                        .map_err(|e| format!("не удалось создать поток режимов процессов: {e}"))
                })
                .collect::<Result<Vec<_>, _>>()?;
            let mut results = Vec::new();
            for worker in workers {
                results.extend(
                    worker
                        .join()
                        .map_err(|_| "ошибка потока режимов процессов")?,
                );
            }
            Ok::<_, String>(results)
        })?
    };
    finished.sort_unstable_by(|a, b| a.0.cmp(&b.0));
    for (mode, report, files) in finished {
        cases.insert(mode, report?);
        inputs.merge(files)?;
    }
    for (mode, against) in sensitivity {
        if options.case.is_some() {
            continue;
        }
        let pair = cases
            .get(&mode)
            .and_then(|case| case.reference.as_ref())
            .zip(cases.get(&against).and_then(|case| case.reference.as_ref()));
        let report = pair
            .ok_or_else(|| "нет двух полных рядов для сравнения моделей".to_string())
            .and_then(|(a, b)| a.difference(b));
        let case = cases.get_mut(&mode).expect("режим уже выполнен");
        match report {
            Ok(curves) => case.model_sensitivity = Some(Sensitivity { against, curves }),
            Err(error) => case.diagnostics.push(error),
        }
    }
    inputs.verify()?;
    let outcome = Outcome::aggregate(cases.values().map(|case| case.outcome));
    let completed = cases.values().all(|c| {
        c.diagnostics.is_empty() && c.convergence.trials.iter().all(|trial| trial.completed)
    });
    let report = Report {
        status: if completed { "completed" } else { "incomplete" },
        outcome,
        inputs: inputs.fingerprints(),
        cases,
    };
    if let Some(audit) = audit.as_mut() {
        audit.record_files(&inputs)?;
        audit.record_inputs(&report.inputs)?;
        for (name, case) in &report.cases {
            if let Some(stamp) = case.catalog {
                audit.record_catalog(stamp)?;
            }
            if !case.checkpointable {
                audit.disable_checkpoint();
            }
            if let Some(inventory) = &case.inventory {
                audit.inventory(name.clone(), inventory.clone())?;
            }
            audit.record_inputs((name, &case.world, &case.control_assets))?;
            let key = audit.child(name, name, Vec::new());
            let mut findings = case.convergence.campaign_findings(&key);
            findings.extend(case.hazard_witnesses.iter().cloned().map(|mut finding| {
                finding.key.run = Some(key.clone());
                finding
            }));
            let violations = case
                .checks
                .iter()
                .filter(|check| check.outcome == "violated")
                .map(|check| (check.name.as_str(), check.checked, None));
            audit.observe(
                key,
                case.outcome,
                case.convergence
                    .trials
                    .iter()
                    .all(|t| t.completed && t.steps > 0),
                super::campaign::ExecutionStamp {
                    seed: case.seed,
                    hz: case.convergence.trials[0].rate_hz,
                    reached_ns: case.convergence.trials.iter().all(|t| t.completed)
                        .then_some(case.convergence.window_ns[1]),
                    requested_ns: None,
                    applied_ns: None,
                    fault_delay_ns: None,
                },
                findings,
                violations,
            )?;
            audit.record_comparison(case.reference.as_ref());
        }
    }
    if options.json {
        serde_json::to_writer(&mut *out, &report).map_err(|e| e.to_string())?;
        writeln!(out).map_err(|e| e.to_string())?;
    } else {
        writeln!(out, "Результат процессов {:?}", report.outcome).map_err(|e| e.to_string())?;
        for (name, case) in report.cases {
            writeln!(
                out,
                "{name} — {:?}, сеток {}",
                case.outcome,
                case.convergence.trials.len()
            )
            .map_err(|e| e.to_string())?;
        }
    }
    Ok(u8::from(!completed))
}
