//! Оркестрация кампаний. Предметные отчёты создают штатные оценщики.

pub(super) mod installation;
mod native;
mod bench;

use std::collections::{BTreeMap, BTreeSet};
use std::io::Write;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};
use sim_task::campaign::RunKey;
use sim_task::metrics::{LinkedMetric, MetricValue, RowStats, Worse, summarize_linked};
use sim_task::sampling::{Sample, SampleKey, Sampling, Seeds};
use sim_task::verdict::{
    Finding, HazardClass, Level, LimitValue, Readiness, RequirementKey, SimulationVerdict,
};
use sim_task::{Case, CaseReport, Fault, Fingerprint, Limit, Outcome};

use super::{control_witness, interaction, matrix::Checkpoint, processes, read, run_control};
use crate::Options;
use installation::LinkedSample;

const VERSION: u32 = 1;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Manifest {
    version: u32,
    assemblies: BTreeMap<String, Assembly>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Assembly {
    scope: String,
    experiments: BTreeMap<String, Experiment>,
    sampling: Option<Sampling>,
    seed: Option<u64>,
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Experiment {
    Task {
        world: PathBuf,
        task: PathBuf,
        catalog: Option<PathBuf>,
        seed: Option<u64>,
        jobs: Option<std::num::NonZeroUsize>,
        sampling: Option<Sampling>,
    },
    Scenario {
        campaign: PathBuf,
        installation: PathBuf,
    },
}

#[derive(Serialize)]
struct Report {
    version: u32,
    registry_version: u32,
    status: &'static str,
    scope: &'static str,
    repeat: Option<String>,
    worst_run_key: Option<String>,
    resumed: usize,
    assemblies: BTreeMap<String, AssemblyReport>,
}

#[derive(Serialize)]
struct AssemblyReport {
    scope: String,
    verdict: SimulationVerdict,
    readiness: Readiness,
    coverage: Coverage,
    worst_run_key: Option<String>,
    runs: BTreeMap<String, RowReport>,
    statistics: BTreeMap<String, sim_task::metrics::CampaignStats>,
}

#[derive(Default, Serialize)]
struct Coverage {
    required: usize,
    executed: usize,
    omitted: usize,
    open: usize,
}

#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct RowReport {
    input: Fingerprint,
    status: RunStatus,
    outcome: Outcome,
    required: usize,
    executed: usize,
    findings: Vec<serde_json::Value>,
    detail: serde_json::Value,
}

#[derive(Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
enum RunStatus {
    Completed,
    Incomplete,
    Open,
    Omitted,
}

impl RowReport {
    fn omitted(input: Fingerprint) -> Self {
        Self::omitted_rows(input, 1)
    }

    fn omitted_rows(input: Fingerprint, required: usize) -> Self {
        Self {
            input,
            status: RunStatus::Omitted,
            outcome: Outcome::NotAnalyzed,
            required,
            executed: 0,
            findings: Vec::new(),
            detail: serde_json::Value::Null,
        }
    }
}

#[derive(Serialize, Deserialize)]
struct Child {
    key: RunKey,
    outcome: Outcome,
    executed: bool,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    metrics: BTreeMap<String, Statistic>,
    execution: ExecutionStamp,
}

#[derive(Serialize, Deserialize)]
pub(super) struct ExecutionStamp {
    pub seed: u64,
    pub hz: u64,
    pub reached_ns: Option<u64>,
    pub requested_ns: Option<u64>,
    pub applied_ns: Option<u64>,
    pub fault_delay_ns: Option<u64>,
}

#[derive(Clone, Serialize, Deserialize)]
struct Statistic {
    unit: String,
    worse: Worse,
    value: f64,
    #[serde(default)]
    absolute_error: bool,
}

impl Statistic {
    fn definition(&self) -> LinkedMetric {
        LinkedMetric {
            unit: self.unit.clone(),
            worse: self.worse,
            absolute_error: self.absolute_error,
        }
    }
}

#[derive(Serialize)]
struct LinkedReport<'a> {
    children: &'a [Child],
    profile: serde_json::Value,
    #[serde(skip_serializing_if = "Option::is_none")]
    sample: Option<&'a Sample>,
    inventory: &'a BTreeMap<String, serde_json::Value>,
}

pub(super) struct LinkedAudit {
    station: String,
    selected: bool,
    omitted: Option<usize>,
    unavailable: Option<String>,
    children: Vec<Child>,
    findings: Vec<Finding>,
    installed: Vec<Fingerprint>,
    files: BTreeMap<PathBuf, processes::FileStamp>,
    catalog: Option<Fingerprint>,
    checkpointable: bool,
    sample_key: SampleKey,
    inventory: BTreeMap<String, installation::LinkedInventory>,
}

impl LinkedAudit {
    pub(super) fn record_unavailable(&mut self, reason: impl Into<String>) {
        self.unavailable = Some(reason.into());
    }

    pub(super) fn inventory(
        &mut self,
        id: String,
        entry: installation::LinkedInventory,
    ) -> Result<(), String> {
        if self.inventory.insert(id, entry).is_some() {
            return Err("повтор строки инвентаризации".into());
        }
        Ok(())
    }

    pub(super) fn record_files(&mut self, files: &processes::Inputs) -> Result<(), String> {
        for (path, stamp) in files.stamps() {
            if let Some(old) = self.files.insert(path.clone(), stamp.clone())
                && old != stamp
            {
                return Err(format!("вход изменился между опытами: {}", path.display()));
            }
        }
        Ok(())
    }

    pub(super) fn record_catalog(&mut self, stamp: Fingerprint) -> Result<(), String> {
        if self.catalog.replace(stamp).is_some_and(|old| old != stamp) {
            return Err("каталог изменился между опытами".into());
        }
        Ok(())
    }

    pub(super) fn disable_checkpoint(&mut self) {
        self.checkpointable = false;
    }

    pub(super) fn omit_unselected(&mut self, required: usize) -> bool {
        if !self.selected {
            self.omitted = Some(required);
        }
        !self.selected
    }

    pub(super) fn record_inputs(&mut self, inputs: impl Serialize) -> Result<(), String> {
        self.installed.push(Fingerprint::of(&encode(inputs)?));
        Ok(())
    }

    pub(super) fn child(&self, mode: &str, path: &str, faults: Vec<Fault>) -> RunKey {
        RunKey {
            sample: self.sample_key.clone(),
            ..profile_key(&self.station, mode, path, faults)
        }
    }

    pub(super) fn observe<'a>(
        &mut self,
        key: RunKey,
        outcome: Outcome,
        executed: bool,
        execution: ExecutionStamp,
        findings: Vec<Finding>,
        violations: impl IntoIterator<Item = (&'a str, Option<[f64; 2]>, Option<([f64; 2], &'a str)>)>,
    ) -> Result<(), String> {
        self.findings.extend(findings);
        for (name, measured, limit) in violations {
            let mut finding = criterion(&key, name, "нарушение критерия постановки");
            finding.measurement = measured;
            if let Some(([lo, hi], source)) = limit {
                finding.limit = Some(LimitValue {
                    lower: Some([lo; 2]),
                    upper: Some([hi; 2]),
                });
                finding.limit_sources.push(source.into());
            }
            self.findings.push(finding);
        }
        if outcome == Outcome::Fail
            && !self
                .findings
                .iter()
                .any(|f| f.key.run.as_ref() == Some(&key) && f.outcome == Outcome::Fail)
        {
            return Err(format!(
                "у отказа режима «{}» отсутствует находка",
                key.mode
            ));
        }
        let mut metrics = BTreeMap::new();
        if executed {
            for finding in self
                .findings
                .iter()
                .filter(|f| f.key.run.as_ref() == Some(&key))
            {
                let Some(([lo, hi], unit, limit)) = finding
                    .measurement
                    .zip(finding.unit.as_ref())
                    .zip(finding.limit.as_ref())
                    .map(|((range, unit), limit)| (range, unit, limit))
                else {
                    continue;
                };
                let (value, worse) = match (limit.lower, limit.upper) {
                    (Some(_), None) => (lo, Worse::Lower),
                    (None, Some(_)) => (hi, Worse::Higher),
                    _ => continue,
                };
                if !value.is_finite() {
                    return Err(format!("неконечная метрика «{}»", finding.code));
                }
                let id = format!(
                    "{}/{}/{}",
                    finding.key.class.spec().id,
                    finding.key.object,
                    finding.code
                );
                match metrics.entry(id) {
                    std::collections::btree_map::Entry::Vacant(entry) => {
                        entry.insert(Statistic {
                            unit: unit.clone(),
                            worse,
                            value,
                            absolute_error: false,
                        });
                    }
                    std::collections::btree_map::Entry::Occupied(mut entry) => {
                        let previous = entry.get_mut();
                        if previous.unit != *unit || previous.worse != worse {
                            return Err(
                                "несовместимые единицы или направление метрики ссылочной строки"
                                    .into(),
                            );
                        }
                        previous.value = match worse {
                            Worse::Higher => previous.value.max(value),
                            Worse::Lower => previous.value.min(value),
                        };
                    }
                }
            }
        }
        self.children.push(Child {
            key,
            outcome,
            executed,
            metrics,
            execution,
        });
        Ok(())
    }

    pub(super) fn control(
        &mut self,
        cases: &[CaseReport],
        outcome: Outcome,
        specs: &[control_witness::Spec],
        observations: &BTreeMap<Case, Vec<control_witness::Observation>>,
        comparisons: &BTreeMap<String, super::reference::Comparison>,
        assessor: &sim_task::Assessor,
        seed: u64,
        hz: u64,
    ) -> Result<(), String> {
        for case in cases {
            let key = self.child(
                &case.case.mode,
                &case.case.path,
                case.case.fault.iter().cloned().collect(),
            );
            let mut findings = Vec::new();
            let observed = observations.get(&case.case).map_or(&[][..], Vec::as_slice);
            for (index, spec) in specs
                .iter()
                .enumerate()
                .filter(|(_, spec)| spec.case == case.case)
            {
                let mut key = requirement(&key, spec.class, &spec.object);
                key.window_ns = Some(spec.window_ns);
                let mut finding =
                    Finding::open(&key, "нет подтверждённого ряда исполнительной модели");
                if let Some(observation) = observed.iter().find(|item| item.spec == index) {
                    let (lower, upper) = spec.bounds();
                    finding.reason = spec.reason().into();
                    finding.outcome = observation.outcome;
                    finding.measurement = Some(observation.measured);
                    finding.limit = Some(LimitValue {
                        lower: lower.map(|v| [v; 2]),
                        upper: upper.map(|v| [v; 2]),
                    });
                    finding.unit = Some(spec.unit().into());
                    finding.event_ns = Some(observation.at_ns);
                    finding.journal = case.journal.map(Fingerprint::from_fnv1a);
                    finding.measurement_sources.push(spec.scope.clone());
                    finding.limit_sources.push(spec.source.reference.clone());
                    finding.model_scope = Some(spec.scope.clone());
                    finding.missing_data.clear();
                    finding.confirmed_hazard = observation.outcome == Outcome::Fail;
                }
                findings.push(finding);
            }
            for violation in &case.limit_violations {
                for metric in case.metrics.iter().filter(|m| m.signal == violation.signal) {
                    let (limit, source) = match &metric.limit {
                        Limit::Range {
                            min, max, source, ..
                        } if min.is_some_and(|v| violation.value < v)
                            || max.is_some_and(|v| violation.value > v) =>
                        {
                            (
                                LimitValue {
                                    lower: min.map(|v| [v; 2]),
                                    upper: max.map(|v| [v; 2]),
                                },
                                source,
                            )
                        }
                        Limit::Forbidden { source, .. } if violation.value != 0.0 => (
                            LimitValue {
                                lower: None,
                                upper: Some([0.0; 2]),
                            },
                            source,
                        ),
                        _ => continue,
                    };
                    let mut finding = criterion(
                        &key,
                        &violation.signal.name,
                        "нарушение предела замкнутого опыта",
                    );
                    finding.measurement = Some([violation.value; 2]);
                    finding.unit = Some(violation.signal.unit.clone());
                    finding.limit = Some(limit);
                    finding.limit_sources.push(source.reference.clone());
                    finding.event_ns = Some(violation.at_ns);
                    finding.journal = case.journal.map(Fingerprint::from_fnv1a);
                    finding.key.window_ns = Some([metric.first_ns, metric.last_ns]);
                    findings.push(finding);
                }
            }
            for reason in &case.violations {
                let mut finding = criterion(&key, &case.case.path, reason);
                finding.journal = case.journal.map(Fingerprint::from_fnv1a);
                findings.push(finding);
            }
            for reason in &case.missing {
                let mut finding = Finding::open(
                    &requirement(&key, HazardClass::Execution, &case.case.path),
                    reason,
                );
                finding.journal = case.journal.map(Fingerprint::from_fnv1a);
                findings.push(finding);
            }
            self.observe(
                key,
                case.outcome,
                case.was_executed(),
                ExecutionStamp {
                    seed,
                    hz,
                    reached_ns: case.reached_ns,
                    requested_ns: case.requested_ns,
                    applied_ns: case.applied_ns,
                    fault_delay_ns: assessor
                        .plan(&case.case)
                        .and_then(|plan| plan.fault)
                        .and_then(|fault| fault.extra_delay_ns),
                },
                findings,
                [],
            )?;
            if case.was_executed() {
                let child = self
                    .children
                    .last_mut()
                    .expect("строка только что зарегистрирована");
                for metric in case.metrics.iter().filter(|metric| metric.samples > 0) {
                    let id = format!("контур/{}", metric.signal.name);
                    child.metrics.insert(
                        format!("{id}/максимум"),
                        Statistic {
                            unit: metric.signal.unit.clone(),
                            worse: Worse::Higher,
                            value: metric.max,
                            absolute_error: false,
                        },
                    );
                    child.metrics.insert(
                        format!("{id}/минимум"),
                        Statistic {
                            unit: metric.signal.unit.clone(),
                            worse: Worse::Lower,
                            value: metric.min,
                            absolute_error: false,
                        },
                    );
                }
                if let Some(reaction) = &case.reaction {
                    child.metrics.insert(
                        "контур/задержка реакции".into(),
                        Statistic {
                            unit: "ns".into(),
                            worse: Worse::Higher,
                            value: reaction.reaction_ns as f64,
                            absolute_error: false,
                        },
                    );
                }
                if case.case.fault.is_none() {
                    self.record_comparison(
                        comparisons
                            .get(&case.case.mode)
                            .filter(|comparison| comparison.journal == case.journal),
                    );
                }
            }
        }
        if outcome == Outcome::Fail && !self.children.iter().any(|c| c.outcome == Outcome::Fail) {
            return Err("отказ контура отсутствует в дочерних опытах".into());
        }
        Ok(())
    }

    pub(super) fn record_comparison(&mut self, comparison: Option<&super::reference::Comparison>) {
        let Some(child) = self.children.last_mut().filter(|child| child.executed) else {
            return;
        };
        for (name, unit, value) in comparison.into_iter().flat_map(|r| r.absolute_errors()) {
            child.metrics.insert(
                format!("сверка/{name}/абсолютная ошибка"),
                Statistic {
                    unit: unit.into(),
                    worse: Worse::Higher,
                    value,
                    absolute_error: true,
                },
            );
        }
    }
}

pub(super) fn profile_key(station: &str, mode: &str, path: &str, faults: Vec<Fault>) -> RunKey {
    RunKey {
        station: station.into(),
        mode: mode.into(),
        path: path.into(),
        faults,
        sample: SampleKey::Nominal,
        grid: "profile".into(),
    }
}

fn criterion(key: &RunKey, name: &str, reason: &str) -> Finding {
    let mut finding = Finding::open(&requirement(key, HazardClass::TaskCriterion, name), reason);
    finding.code = name.into();
    finding.missing_data.clear();
    finding.outcome = Outcome::Fail;
    finding
}

fn requirement(key: &RunKey, class: HazardClass, object: &str) -> RequirementKey {
    RequirementKey {
        class,
        object: object.into(),
        extension: None,
        run: Some(key.clone()),
        window_ns: None,
        level: Level::Simulation,
    }
}

fn value<T: Serialize>(thing: T) -> Result<serde_json::Value, String> {
    serde_json::to_value(thing).map_err(|e| e.to_string())
}

fn encode(thing: impl Serialize) -> Result<Vec<u8>, String> {
    serde_json::to_vec(&thing).map_err(|e| e.to_string())
}

fn linked(
    input: Fingerprint,
    world: &Path,
    task: &Path,
    key: &str,
    selected: bool,
    mut options: Options,
    sample: Option<LinkedSample<'_>>,
    sampling_gap: Option<&str>,
    checkpoint: Option<&Checkpoint>,
) -> Result<(RowReport, bool), String> {
    options.world = world.to_owned();
    options.json = true;
    let mut roots = processes::Inputs::default();
    roots.capture(task)?;
    let checkpoint_prefix = format!("linked-{:016x}-", sim_signal::fnv1a(encode((input, key))?));
    if let Some(checkpoint) = checkpoint.filter(|_| selected) {
        for name in checkpoint.rows_with_prefix(&checkpoint_prefix)? {
            let Some(saved) = checkpoint.load_row::<SavedLinked>(&name)? else {
                continue;
            };
            if saved.input != input
                || saved.key != key
                || saved.row.outcome != Outcome::Fail
                || saved.row.status != RunStatus::Completed
                || saved.row.executed != saved.row.required
                || saved.row.executed == 0
                || saved.row.findings.is_empty()
                || saved.files.is_empty()
            {
                return Err(format!("чужая или незавершённая точка опыта «{key}»"));
            }
            if !saved.files.iter().all(|(path, stamp)| stamp.matches(path)) {
                continue;
            }
            if let Some(stamp) = saved.catalog {
                let models = super::Models::load(&crate::catalog_path(&options)?)?;
                if Fingerprint::of(&models.bytes) != stamp {
                    continue;
                }
            }
            return Ok((saved.row, true));
        }
    }
    let task = task
        .canonicalize()
        .map_err(|e| format!("{}: {e}", task.display()))?;
    let mut audit = LinkedAudit {
        station: key.into(),
        selected,
        omitted: None,
        unavailable: None,
        children: Vec::new(),
        findings: Vec::new(),
        installed: Vec::new(),
        files: BTreeMap::new(),
        catalog: None,
        checkpointable: true,
        sample_key: sample.map_or(SampleKey::Nominal, |s| s.sample.key.clone()),
        inventory: BTreeMap::new(),
    };
    audit.record_files(&roots)?;
    let mut stdout = Vec::new();
    // Старый контур имеет собственные точки завершённых строк.
    let mut cache_profile = false;
    if let Some(file) = interaction::TaskFile::load(&task)? {
        cache_profile = true;
        options.checkpoint = None;
        interaction::run_observed(file, &task, &options, &mut stdout, Some(&mut audit), sample)?;
    } else if let Some(file) = processes::TaskFile::load(&task)? {
        cache_profile = true;
        options.checkpoint = None;
        processes::run_observed(file, &task, &options, &mut stdout, Some(&mut audit), sample)?;
    } else {
        options.matrix_full = true;
        let compiled = if let Some(hz) = options.linked_grid_hz {
            let mut inputs =
                sim_task::campaign::WorldInputs::load(world).map_err(|e| e.to_string())?;
            inputs.world.physics_hz = hz;
            inputs.compile().map_err(|e| e.to_string())?
        } else {
            sim_world::load_and_compile(world)?
        };
        run_control(
            compiled,
            &task,
            &options,
            &mut stdout,
            Some(&mut audit),
            sample,
        )?;
    }
    if let Some(reason) = audit.unavailable {
        return Ok((
            RowReport {
                status: RunStatus::Open,
                detail: value(reason)?,
                ..RowReport::omitted(input)
            },
            false,
        ));
    }
    if let Some(required) = audit.omitted {
        if required == 0 {
            return Err("у опыта отсутствуют обязательные дочерние строки".into());
        }
        return Ok((RowReport::omitted_rows(input, required), false));
    }
    let required = audit.children.len();
    if required == 0 && options.selected_control_case.is_some() {
        return Ok((
            RowReport {
                status: RunStatus::Open,
                detail: value("в проводке и профиле отсутствует указанный путь или отказ")?,
                ..RowReport::omitted(input)
            },
            false,
        ));
    }
    if required == 0 || audit.installed.is_empty() {
        return Err("у опыта отсутствуют обязательные дочерние строки".into());
    }
    let executed = audit.children.iter().filter(|c| c.executed).count();
    for child in &audit.children {
        if let Some(inventory) = audit.inventory.get(&child.key.mode) {
            for (class, object) in &inventory.hazards {
                if !audit.findings.iter().any(|finding| {
                    finding.key.run.as_ref() == Some(&child.key)
                        && finding.key.class == *class
                        && finding.key.object == *object
                }) {
                    audit.findings.push(Finding::open(
                        &requirement(&child.key, *class, object),
                        "для объекта из разрешённого состава нет применимой проверки в этой строке",
                    ));
                }
            }
        }
    }
    let outcome = Outcome::aggregate(
        audit
            .children
            .iter()
            .map(|c| c.outcome)
            .chain([Outcome::NotAnalyzed]),
    );
    audit.findings.push(Finding::open(
        &requirement(
            &audit.child("profile", key, Vec::new()),
            HazardClass::Inventory,
            key,
        ),
        "для разрешённого состава профиля нет полной матрицы применимых проверок сборки",
    ));
    if let Some(reason) = sampling_gap {
        audit.findings.push(Finding::open(
            &requirement(
                &audit.child("profile", key, Vec::new()),
                HazardClass::Inventory,
                "разброс",
            ),
            reason,
        ));
    }
    let inventory = audit
        .inventory
        .into_iter()
        .map(|(id, entry)| (id, entry.report))
        .collect();
    let row = RowReport {
        input: Fingerprint::of(&encode((
            input,
            &audit.installed,
            &audit.files,
            audit.catalog,
            &inventory,
        ))?),
        status: if executed == required {
            RunStatus::Completed
        } else {
            RunStatus::Incomplete
        },
        outcome,
        required,
        executed,
        findings: audit.findings.iter().map(value).collect::<Result<_, _>>()?,
        detail: value(LinkedReport {
            children: &audit.children,
            profile: serde_json::from_slice(&stdout).map_err(|e| e.to_string())?,
            sample: sample.map(|s| s.sample),
            inventory: &inventory,
        })?,
    };
    if let Some(checkpoint) = checkpoint
        && cache_profile
        && audit.checkpointable
        && row.status == RunStatus::Completed
        && row.outcome == Outcome::Fail
    {
        if !audit.files.iter().all(|(path, stamp)| stamp.matches(path)) {
            return Err("вход связанного опыта изменился перед записью контрольной точки".into());
        }
        let graph = Fingerprint::of(&encode((&audit.files, audit.catalog, row.input))?);
        let name = format!(
            "{checkpoint_prefix}{:016x}",
            sim_signal::fnv1a(encode(graph)?)
        );
        let saved = SavedLinked {
            input,
            key: key.to_owned(),
            row,
            files: audit.files,
            catalog: audit.catalog,
        };
        checkpoint.save_row(&name, &saved)?;
        return Ok((saved.row, false));
    }
    Ok((row, false))
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct SavedLinked {
    input: Fingerprint,
    key: String,
    row: RowReport,
    files: BTreeMap<PathBuf, processes::FileStamp>,
    catalog: Option<Fingerprint>,
}

fn selected(options: &Options, key: &str) -> bool {
    options.repeat.as_deref().is_none_or(|wanted| wanted == key)
}

fn slug(id: &str) -> bool {
    !id.is_empty()
        && id
            .bytes()
            .all(|ch| ch.is_ascii_lowercase() || ch.is_ascii_digit() || ch == b'-' || ch == b'_')
}

fn linked_statistics(
    station: &str,
    rows: &[(String, Vec<Child>, Option<Sample>)],
    sampling: Option<&Sampling>,
) -> Result<(sim_task::metrics::CampaignStats, BTreeSet<String>), String> {
    let nominal_key = |key: &RunKey| RunKey {
        station: station.into(),
        sample: SampleKey::Nominal,
        ..key.clone()
    };
    let mut definitions = BTreeMap::<_, BTreeMap<_, _>>::new();
    let mut addresses = BTreeSet::new();
    for (_, children, _) in rows {
        for child in children {
            let key = nominal_key(&child.key);
            addresses.insert(key.clone());
            let series = definitions.entry(key).or_default();
            for (id, metric) in &child.metrics {
                let definition = metric.definition();
                if series
                    .insert(id.clone(), definition.clone())
                    .is_some_and(|old| old != definition)
                {
                    return Err("в серии изменились единица или направление метрики".into());
                }
            }
        }
    }
    let mut all_rows = Vec::new();
    let mut replay = BTreeMap::new();
    for (run_key, children, sample) in rows {
        let indexed: BTreeMap<_, _> = children
            .iter()
            .map(|child| (nominal_key(&child.key), child))
            .collect();
        if indexed.len() != children.len() {
            return Err("повтор дочерней строки ссылочной серии".into());
        }
        for template in &addresses {
            let child = indexed.get(template).copied();
            let sample = sample.clone().or_else(|| {
                child.map(|child| {
                    let seed = child.execution.seed;
                    Sample {
                        key: SampleKey::Nominal,
                        seeds: Seeds {
                            root: seed,
                            batch: seed,
                            specimen: seed,
                            repeat: seed,
                            run: seed,
                        },
                        alternatives: BTreeMap::new(),
                        values: Vec::new(),
                    }
                })
            });
            let Some(sample) = sample else { continue };
            let key = RunKey {
                sample: sample.key.clone(),
                ..template.clone()
            };
            let completed = child.is_some_and(|row| row.executed);
            let outcome = child.map_or(Outcome::NotAnalyzed, |child| {
                if child.executed || child.outcome == Outcome::Fail {
                    child.outcome
                } else {
                    Outcome::NotAnalyzed
                }
            });
            let metrics = child
                .into_iter()
                .flat_map(|row| &row.metrics)
                .map(|(id, metric)| {
                    (
                        id.clone(),
                        MetricValue::Value {
                            value: metric.value,
                        },
                    )
                })
                .collect();
            let reasons = if outcome == Outcome::NotAnalyzed {
                vec![
                    if child.is_some() {
                        "профиль оставил строку открытой"
                    } else {
                        "нет исполнения для этой выборки и строки профиля"
                    }
                    .into(),
                ]
            } else {
                Vec::new()
            };
            replay.insert(key.clone(), run_key.clone());
            all_rows.push(RowStats {
                key,
                completed,
                outcome: Some(outcome),
                reasons,
                sample,
                metrics,
            });
        }
    }
    let statistics =
        summarize_linked(all_rows, sampling, &definitions).map_err(|e| e.to_string())?;
    let worst = statistics
        .series
        .iter()
        .flat_map(|series| series.metrics.values())
        .filter_map(|metric| metric.worst.as_ref())
        .map(|worst| replay[&worst.key].clone())
        .collect();
    Ok((statistics, worst))
}

pub(super) fn run(path: &Path, options: &Options, out: &mut impl Write) -> Result<u8, String> {
    let base = processes::dir(path);
    let mut files = processes::Inputs::default();
    let manifest: Manifest = files.toml(path)?;
    if manifest.version != VERSION || manifest.assemblies.is_empty() {
        return Err("нужен непустой манифест кампании версии 1".into());
    }
    for (id, assembly) in &manifest.assemblies {
        if !slug(id)
            || assembly.scope.trim().is_empty()
            || assembly.experiments.is_empty()
            || assembly.experiments.keys().any(|id| !slug(id))
        {
            return Err(format!(
                "неверный идентификатор, область или пустая кампания «{id}»"
            ));
        }
        if assembly.sampling.is_some() && assembly.seed.is_none() {
            return Err(format!(
                "для общего разброса сборки «{id}» нужно явное корневое зерно seed"
            ));
        }
    }

    let bench = options.measurements.as_deref()
        .map(|path| bench::load(path, &mut files))
        .transpose()?.unwrap_or_default();
    for record in &bench {
        let Some((assembly, experiment)) = record.experiment.split_once('/') else {
            return Err("заверенный ряд требует ключ «сборка/опыт»".into());
        };
        if !matches!(
            manifest.assemblies.get(assembly).and_then(|a| a.experiments.get(experiment)),
            Some(Experiment::Scenario { .. })
        ) {
            return Err(format!("«{}»: нет сценария для заверенного ряда", record.experiment));
        }
    }
    let engine = sim_runtime::Runtime::physics_engine_identity()?;
    let identity = if options.checkpoint.is_some() && options.repeat.is_none() {
        let binary = std::env::current_exe().map_err(|e| e.to_string())?;
        Some(encode((
            VERSION,
            sim_task::verdict::REGISTRY_VERSION,
            Fingerprint::of(&read(&binary)?),
            &engine,
            files.fingerprints(),
        ))?)
    } else {
        None
    };
    let checkpoint = options
        .checkpoint
        .as_deref()
        .zip(identity.as_deref())
        .map(|(root, identity)| Checkpoint::with_identity(root, identity))
        .transpose()?;
    let mut report = Report {
        version: VERSION,
        registry_version: sim_task::verdict::REGISTRY_VERSION,
        status: "completed",
        scope: "модели и постановки из манифеста исполнения",
        repeat: options.repeat.clone(),
        worst_run_key: None,
        resumed: 0,
        assemblies: BTreeMap::new(),
    };
    let mut all_keys = BTreeSet::new();
    let mut worst = None;
    let mut view = None;
    for (id, mut assembly) in manifest.assemblies {
        if let Some(plan) = &mut assembly.sampling {
            plan.validate().map_err(|e| e.to_string())?;
        }
        let mut runs = BTreeMap::new();
        let mut statistics = BTreeMap::new();
        let mut metric_worst = BTreeSet::new();
        let mut readiness = Readiness::Passed;
        for (name, experiment) in assembly.experiments {
            let prefix = format!("{id}/{name}");
            match experiment {
                Experiment::Task {
                    world,
                    task,
                    catalog,
                    seed,
                    jobs,
                    mut sampling,
                } => {
                    readiness = Readiness::NeedsWork;
                    let (world, task) = (base.join(world), base.join(task));
                    let stamp = Fingerprint::of(&encode((&engine, &prefix, &world, &task, seed))?);
                    let catalog = catalog.map(|path| base.join(path));
                    if let Some(plan) = &mut sampling {
                        plan.validate().map_err(|e| e.to_string())?;
                    }
                    let inherited = sampling.is_none() && assembly.sampling.is_some();
                    if inherited && seed.is_some() && seed != assembly.seed {
                        return Err(format!(
                            "«{prefix}»: зерно общего разброса отличается от зерна сборки"
                        ));
                    }
                    let sampling = sampling.as_ref().or(assembly.sampling.as_ref());
                    let seed = seed.or(assembly.seed);
                    let root_seed = match (sampling.is_some(), seed) {
                        (true, None) => {
                            sim_task::campaign::WorldInputs::load(&world)
                                .map_err(|e| e.to_string())?
                                .world
                                .seed
                        }
                        (_, seed) => seed.unwrap_or_default(),
                    };
                    let gap = if let Some(plan) = sampling {
                        plan.coverage_gap(root_seed).map_err(|e| e.to_string())?
                    } else {
                        Some("отсутствует обязательный раздел разброса sampling")
                    };
                    let samples = if let Some(plan) = sampling {
                        plan.keys()
                            .map_err(|e| e.to_string())?
                            .iter()
                            .map(|key| {
                                plan.sample(root_seed, key, &["campaign/v1", "linked", &prefix])
                                    .map(Some)
                                    .map_err(|e| e.to_string())
                            })
                            .collect::<Result<Vec<_>, _>>()?
                    } else {
                        vec![None]
                    };
                    let mut series_rows = Vec::with_capacity(samples.len());
                    for sample in &samples {
                        let input = Fingerprint::of(&encode((stamp, sample, &catalog, jobs))?);
                        let run_key = if sample.is_some() {
                            format!("{prefix}/{:016x}", sim_signal::fnv1a(encode(input)?))
                        } else {
                            prefix.clone()
                        };
                        let sample = sampling
                            .zip(sample.as_ref())
                            .map(|(sampling, sample)| LinkedSample { sampling, sample });
                        let wanted = options.repeat.as_deref();
                        let selected = wanted.is_none_or(|wanted| {
                            if sample.is_some() {
                                wanted
                                    .strip_prefix(&run_key)
                                    .is_some_and(|suffix| suffix.starts_with('@'))
                            } else {
                                wanted == run_key
                            }
                        });
                        let (row, resumed) = linked(
                            input,
                            &world,
                            &task,
                            &run_key,
                            selected,
                            Options {
                                seed,
                                jobs,
                                catalog: catalog.clone(),
                                checkpoint: options
                                    .checkpoint
                                    .clone()
                                    .filter(|_| options.repeat.is_none()),
                                ..Default::default()
                            },
                            sample,
                            gap,
                            checkpoint.as_ref(),
                        )?;
                        let run_key = if selected && sample.is_some() {
                            format!("{run_key}@{:016x}", sim_signal::fnv1a(encode(row.input)?))
                        } else {
                            run_key
                        };
                        report.resumed += usize::from(resumed);
                        let children = match row.detail.get("children") {
                            Some(children) => serde_json::from_value(children.clone())
                                .map_err(|e| e.to_string())?,
                            None => Vec::new(),
                        };
                        series_rows.push((
                            run_key.clone(),
                            children,
                            sample.map(|s| s.sample.clone()),
                        ));
                        if runs.insert(run_key, row).is_some() {
                            return Err("повтор ключа ссылочного опыта".into());
                        }
                    }
                    let (series, worst) = linked_statistics(&prefix, &series_rows, sampling)?;
                    statistics.insert(name, series);
                    metric_worst.extend(worst);
                }
                Experiment::Scenario {
                    campaign,
                    installation,
                } => {
                    let result = native::run(
                        &prefix,
                        &base.join(campaign),
                        &base.join(installation),
                        options,
                        checkpoint.as_ref(),
                        &bench,
                    )?;
                    if let Some(archive) = result.view {
                        view = Some(archive);
                    }
                    report.resumed += result.resumed;
                    readiness = match (readiness, result.readiness) {
                        (Readiness::Rejected, _) | (_, Readiness::Rejected) => Readiness::Rejected,
                        (Readiness::NeedsWork, _) | (_, Readiness::NeedsWork) => {
                            Readiness::NeedsWork
                        }
                        (Readiness::BenchOnly, _) | (_, Readiness::BenchOnly) => {
                            Readiness::BenchOnly
                        }
                        (Readiness::Passed, Readiness::Passed) => Readiness::Passed,
                    };
                    statistics.insert(name, result.statistics);
                    metric_worst.extend(result.metric_worst);
                    for (key, row) in result.rows {
                        if runs.insert(key, row).is_some() {
                            return Err("повтор ключа прогона".into());
                        }
                    }
                }
            }
        }
        let mut coverage = Coverage::default();
        let mut assembly_worst = None;
        for (key, row) in &runs {
            if !all_keys.insert(key.clone()) {
                return Err("повтор глобального ключа прогона".into());
            }
            coverage.required += row.required;
            coverage.executed += row.executed;
            if row.status == RunStatus::Omitted {
                coverage.omitted += row.required;
            } else {
                coverage.open += row.required - row.executed;
                // При одинаковом исходе сохраняем физический экстремум
                // однородной серии. Разные метрики численно не сравниваем.
                let rank = (row.outcome, metric_worst.contains(key));
                if assembly_worst
                    .as_ref()
                    .is_none_or(|(current, _)| rank > *current)
                {
                    assembly_worst = Some((rank, key.clone()));
                }
            }
        }
        if let Some((rank, key)) = &assembly_worst
            && worst.as_ref().is_none_or(|(current, _)| rank > current)
        {
            worst = Some((*rank, key.clone()));
        }
        let outcome = Outcome::aggregate(runs.values().map(|r| r.outcome));
        let verdict = match outcome {
            Outcome::Fail => SimulationVerdict::Rejected,
            Outcome::NotAnalyzed => SimulationVerdict::NotProven,
            Outcome::Pass => SimulationVerdict::PassedSimulation,
        };
        report.assemblies.insert(
            id,
            AssemblyReport {
                scope: assembly.scope,
                verdict,
                readiness: match outcome {
                    Outcome::Fail => Readiness::Rejected,
                    Outcome::NotAnalyzed => Readiness::NeedsWork,
                    Outcome::Pass => readiness,
                },
                coverage,
                worst_run_key: assembly_worst.map(|(_, key)| key),
                runs,
                statistics,
            },
        );
    }
    if options
        .repeat
        .as_ref()
        .is_some_and(|key| !all_keys.contains(key))
    {
        return Err("ключ повтора отсутствует в манифесте и разрешённой матрице".into());
    }
    files.verify()?;
    if sim_runtime::Runtime::physics_engine_identity()? != engine {
        return Err("образ MuJoCo изменился во время кампании".into());
    }
    if let (Some(path), Some(bytes)) = (&options.view, view) {
        crate::export(path, |file| file.write_all(&bytes))?;
    }
    report.worst_run_key = worst.map(|(_, key)| key);
    if options.json {
        serde_json::to_writer(&mut *out, &report).map_err(|e| e.to_string())?;
        writeln!(out).map_err(|e| e.to_string())?;
    } else {
        for (id, assembly) in report.assemblies {
            writeln!(
                out,
                "{id} — {:?}, исполнено {} из {}",
                assembly.verdict, assembly.coverage.executed, assembly.coverage.required
            )
            .map_err(|e| e.to_string())?;
        }
    }
    Ok(0)
}
