use std::collections::BTreeMap;
use std::io::{ErrorKind, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};

use serde::{Deserialize, Serialize};
use sim_signal::{Time, fnv1a};
use sim_task::{Case, CaseReport, Identity, Outcome};

use super::{Execution, InstalledInputs, Profile, ResolvedWire, mechanism, read, reference};
use super::control_witness::Observation;
use crate::Options;

#[derive(Serialize)]
pub(super) struct Coverage {
    pub executed: usize,
    pub required: usize,
    pass: usize,
    fail: usize,
    not_analyzed: usize,
}

impl Coverage {
    pub(super) fn from_cases(cases: &[CaseReport]) -> Self {
        Self {
            executed: cases.iter().filter(|case| case.was_executed()).count(),
            required: cases.len(),
            pass: cases
                .iter()
                .filter(|case| case.outcome == Outcome::Pass)
                .count(),
            fail: cases
                .iter()
                .filter(|case| case.outcome == Outcome::Fail)
                .count(),
            not_analyzed: cases
                .iter()
                .filter(|case| case.outcome == Outcome::NotAnalyzed)
                .count(),
        }
    }
}

/// Отчёт CLI. Возобновление не создаёт живых RunResult или Evidence из файлов.
#[derive(Serialize)]
pub(super) struct MatrixReport<'a> {
    identity: &'a Identity,
    pub cases: Vec<CaseReport>,
    uncovered_drives: Vec<String>,
    pub outcome: Outcome,
}

pub(super) struct Plan<'a> {
    runnable: Vec<(usize, &'a Case)>,
    skipped: Vec<(usize, Row)>,
}

type ResultRows<'a> = (
    MatrixReport<'a>,
    bool,
    BTreeMap<String, reference::Comparison>,
    Coverage,
    BTreeMap<Case, Vec<Observation>>,
);

impl<'a> Plan<'a> {
    pub(super) fn new(
        execution: &'a Execution<'_>,
        profile: &Profile,
        wires: &BTreeMap<String, Vec<ResolvedWire>>,
        selected: Option<&Case>,
    ) -> Result<Self, String> {
        let mut plan = Self {
            runnable: Vec::new(),
            skipped: Vec::new(),
        };
        let mut configs = BTreeMap::new();
        for (index, case) in execution.assessor.required_cases().iter().enumerate() {
            if selected.is_some_and(|wanted| wanted != case) {
                continue;
            }
            let reason = if let Some(reason) = profile.deferred_modes.get(&case.mode) {
                Some(reason.clone())
            } else if let Some(run) = execution.assessor.plan(case) {
                if let Some(injection) = run.fault {
                    let node = &injection.fault.node;
                    let supported = format!("{:?}", mechanism(injection.fault.kind).0);
                    let wire = wires[&case.mode]
                        .iter()
                        .find(|wire| wire.name == case.path)
                        .ok_or("обязательный путь пропал из разрешённой проводки")?;
                    if !wire
                        .faults
                        .get(node)
                        .is_some_and(|items| items.contains(&supported))
                    {
                        Some(format!(
                            "в проводке нет механизма {supported} для узла «{node}»"
                        ))
                    } else {
                        let config = match configs.entry(&case.mode) {
                            std::collections::btree_map::Entry::Occupied(entry) => entry.into_mut(),
                            std::collections::btree_map::Entry::Vacant(entry) => {
                                entry.insert(execution.setup.config(&case.mode, "")?)
                            }
                        };
                        if config.fault_overlaps(node, Time(injection.at_ns))?
                            && execution.setup.reuse_fault(&case.mode, injection).is_none()
                        {
                            Some(format!(
                                "воздействие на «{node}» пересекается с уже заданным отказом режима"
                            ))
                        } else {
                            None
                        }
                    }
                } else {
                    None
                }
            } else {
                Some("нет обязательного режима или расписания отказа".into())
            };
            if let Some(reason) = reason {
                plan.skipped.push((
                    index,
                    Row {
                        report: CaseReport::not_analyzed(case.clone(), &reason),
                        comparison: None,
                        observations: Vec::new(),
                    },
                ));
            } else {
                plan.runnable.push((index, case));
            }
        }
        Ok(plan)
    }

    pub(super) fn run(
        self,
        execution: &'a Execution<'_>,
        installed: &BTreeMap<String, InstalledInputs>,
        options: &Options,
    ) -> Result<ResultRows<'a>, String> {
        let checkpoint = options
            .checkpoint
            .as_deref()
            .map(|path| Checkpoint::open(path, execution, installed))
            .transpose()?;
        let jobs = options
            .jobs
            .or_else(|| std::thread::available_parallelism().ok())
            .map_or(1, std::num::NonZeroUsize::get)
            .min(self.runnable.len());
        let next = AtomicUsize::new(0);
        let mut rows = std::thread::scope(|scope| {
            let mut workers = Vec::new();
            let runnable = &self.runnable;
            let checkpoint = checkpoint.as_ref();
            let next = &next;
            for _ in 0..jobs {
                workers.push(
                    std::thread::Builder::new()
                        .spawn_scoped(scope, move || {
                            let mut rows = Vec::new();
                            while let Some(&(index, case)) =
                                runnable.get(next.fetch_add(1, Ordering::Relaxed))
                            {
                                rows.push((index, run_case(execution, checkpoint, index, case)));
                            }
                            rows
                        })
                        .map_err(|error| format!("не удалось создать поток матрицы: {error}"))?,
                );
            }
            let mut rows = Vec::new();
            for worker in workers {
                rows.extend(
                    worker
                        .join()
                        .map_err(|_| "необработанная ошибка потока матрицы")?,
                );
            }
            rows.sort_by_key(|(index, _)| *index);
            rows.into_iter()
                .map(|(index, row)| Ok((index, row?)))
                .collect::<Result<Vec<_>, String>>()
        })?;

        let complete = !rows.is_empty() && rows.iter().all(|(_, row)| row.report.completed);
        rows.extend(self.skipped);
        rows.sort_by_key(|(index, _)| *index);
        let mut cases = Vec::with_capacity(rows.len());
        let mut comparisons = BTreeMap::new();
        let mut observations = BTreeMap::new();
        for (_, row) in rows {
            if let Some(comparison) = row.comparison {
                comparisons.insert(row.report.case.mode.clone(), comparison);
            }
            if !row.observations.is_empty() {
                observations.insert(row.report.case.clone(), row.observations);
            }
            cases.push(row.report);
        }
        let coverage = Coverage::from_cases(&cases);
        let assessor = execution.assessor;
        let uncovered_drives = assessor.assess(&[]).uncovered_drives().to_vec();
        let mut outcome = Outcome::aggregate(
            cases
                .iter()
                .filter(|row| row.was_executed() || row.outcome == Outcome::Fail)
                .map(|row| row.outcome),
        );
        if coverage.executed != coverage.required || !uncovered_drives.is_empty() {
            outcome = outcome.max(Outcome::NotAnalyzed);
        }
        Ok((
            MatrixReport {
                identity: assessor.identity(),
                cases,
                uncovered_drives,
                outcome,
            },
            complete,
            comparisons,
            coverage,
            observations,
        ))
    }
}

#[derive(Serialize, Deserialize)]
struct Row {
    report: CaseReport,
    comparison: Option<reference::Comparison>,
    observations: Vec<Observation>,
}

fn run_case(
    execution: &Execution<'_>,
    checkpoint: Option<&Checkpoint>,
    index: usize,
    case: &Case,
) -> Result<Row, String> {
    if let Some(row) = checkpoint
        .map(|state| state.load(index, case))
        .transpose()?
        .flatten()
    {
        return Ok(row);
    }
    let (run, comparison, observations) = execution.run_case(case, |_| Ok(()))?;
    let mut report = run.report().clone();
    if observations.iter().any(|value| value.outcome == Outcome::Fail) {
        report.outcome = Outcome::Fail;
    } else if observations.len()
        != execution.witnesses.iter().filter(|spec| &spec.case == case).count()
    {
        report.outcome = report.outcome.max(Outcome::NotAnalyzed);
    }
    if !report.was_executed() && report.outcome != Outcome::Fail {
        report.outcome = Outcome::NotAnalyzed;
    }
    let row = Row { report, comparison, observations };
    // Положительное доказательство требует свежего исполнения. Незавершённый
    // опыт повторяется при возобновлении, включая ошибки запуска процесса.
    if let Some(checkpoint) = checkpoint
        && row.report.completed
        && row.report.outcome != Outcome::Pass
    {
        checkpoint.save(index, &row)?;
    }
    Ok(row)
}

pub(super) struct Checkpoint {
    directory: PathBuf,
}

#[derive(Serialize, Deserialize)]
struct Envelope {
    checksum: u64,
    payload: String,
}

impl Checkpoint {
    pub(super) fn rows_with_prefix(&self, prefix: &str) -> Result<Vec<String>, String> {
        let mut names = Vec::new();
        for entry in self.directory.read_dir().map_err(|e| e.to_string())? {
            let entry = entry.map_err(|e| e.to_string())?;
            if let Some(name) = entry.file_name().to_str()
                && let Some(name) = name.strip_suffix(".json")
                && name.starts_with(prefix)
            {
                names.push(name.to_owned());
            }
        }
        names.sort();
        Ok(names)
    }

    fn open(
        root: &Path,
        execution: &Execution<'_>,
        installed: &BTreeMap<String, InstalledInputs>,
    ) -> Result<Self, String> {
        // Отпечаток исполняемого кода меняет каталог даже между сборками одной версии.
        let exe = std::env::current_exe().map_err(|error| error.to_string())?;
        let binary = fnv1a(read(&exe)?);
        let references: BTreeMap<_, _> = execution
            .reference_texts
            .iter()
            .map(|(mode, contents)| (mode, fnv1a(contents.bytes())))
            .collect();
        let identity =
            serde_json::to_vec(&(binary, execution.assessor.identity(), installed, references))
                .map_err(|error| error.to_string())?;
        Self::with_identity(root, &identity)
    }

    pub(super) fn with_identity(root: &Path, identity: &[u8]) -> Result<Self, String> {
        let directory = root.join(format!("{:016x}", fnv1a(identity.iter().copied())));
        std::fs::create_dir_all(&directory)
            .map_err(|error| format!("{}: {error}", directory.display()))?;
        Ok(Self { directory })
    }

    fn load(&self, index: usize, case: &Case) -> Result<Option<Row>, String> {
        let name = index.to_string();
        let Some(row): Option<Row> = self.load_row(&name)? else {
            return Ok(None);
        };
        if row.report.case != *case || !row.report.completed || row.report.outcome == Outcome::Pass
        {
            return Err(format!(
                "{}: строка не подходит для возобновления",
                self.directory.join(format!("{name}.json")).display()
            ));
        }
        Ok(Some(row))
    }

    pub(super) fn load_row<T: serde::de::DeserializeOwned>(
        &self,
        name: &str,
    ) -> Result<Option<T>, String> {
        let path = self.directory.join(format!("{name}.json"));
        let bytes = match std::fs::read(&path) {
            Ok(bytes) => bytes,
            Err(error) if error.kind() == ErrorKind::NotFound => return Ok(None),
            Err(error) => return Err(format!("{}: {error}", path.display())),
        };
        let envelope: Envelope = serde_json::from_slice(&bytes)
            .map_err(|error| format!("{}: {error}", path.display()))?;
        if fnv1a(envelope.payload.bytes()) != envelope.checksum {
            return Err(format!(
                "{}: контрольная сумма строки не совпала",
                path.display()
            ));
        }
        serde_json::from_str(&envelope.payload)
            .map(Some)
            .map_err(|error| format!("{}: {error}", path.display()))
    }

    fn save(&self, index: usize, row: &Row) -> Result<(), String> {
        self.save_row(&index.to_string(), row)
    }

    pub(super) fn save_row(&self, name: &str, row: &impl Serialize) -> Result<(), String> {
        let path = self.directory.join(format!("{name}.json"));
        let partial = self
            .directory
            .join(format!("{name}.{}.part", std::process::id()));
        let payload = serde_json::to_string(row).map_err(|error| error.to_string())?;
        let bytes = serde_json::to_vec(&Envelope {
            checksum: fnv1a(payload.bytes()),
            payload,
        })
        .map_err(|error| error.to_string())?;
        let mut file = std::fs::File::create_new(&partial)
            .map_err(|error| format!("{}: {error}", partial.display()))?;
        let result = (|| {
            file.write_all(&bytes)?;
            file.sync_all()?;
            drop(file);
            match std::fs::hard_link(&partial, &path) {
                Err(error) if error.kind() == ErrorKind::AlreadyExists => {
                    if std::fs::read(&path)? == bytes {
                        Ok(())
                    } else {
                        Err(error)
                    }
                }
                result => result,
            }
        })();
        let cleanup = std::fs::remove_file(&partial);
        result
            .and(cleanup)
            .map_err(|error| format!("{}: {error}", path.display()))
    }
}
