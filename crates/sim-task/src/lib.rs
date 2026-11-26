//! Оценка замкнутого контура по фактически исполненным прогонам.
//! Формат задачи и подключение сборщика описаны в FORMAT.md.

pub mod campaign;
mod evaluate;
pub mod interaction;
mod live;
mod live_control;
pub mod measured;
pub mod metrics;
pub mod process;
pub mod sampling;
mod task;
mod topology;
pub mod verdict;

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use std::sync::Arc;

use serde::{Deserialize, Serialize};
use sim_runtime::{Log, Runtime, Status};
use sim_signal::{Time, fnv1a};
use sim_world::CompiledWorld;

pub use evaluate::{
    CaseReport, Comparison, ComparisonAudit, ControlActivity, LimitViolation, LoopTiming, Metric,
    Outcome, Reaction, TransferMetric,
};
pub use interaction::{
    InteractionEvidence, InteractionPlan, assess_interaction, assess_interaction_grids,
};
pub use live::{
    Cycle, Delivery, Event, EventSink, FaultReceipt, Sample, StimulusReceipt, Transfer,
};
pub use task::{
    Case, Criteria, Fault, FaultKind, Injection, Limit, Mode, Response, Signal, Source, SourceKind,
    Stimulus, Task,
};
pub use topology::{Direction, Hop, LegacyDrive, Node, NodeKind, Path, Topology};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Error(String);

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

impl std::error::Error for Error {}

pub(crate) fn invalid(message: impl Into<String>) -> Error {
    Error(message.into())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Fingerprint(u64);

impl Fingerprint {
    /// Стабильный отпечаток FNV-1a из sim-signal, для идентификации версии входа.
    pub fn of(bytes: &[u8]) -> Self {
        Self(fnv1a(bytes.iter().copied()))
    }

    /// Числовое представление без преобразования через f64.
    pub fn as_u64(self) -> u64 {
        self.0
    }

    /// Отпечаток FNV-1a, уже посчитанный при установке ресурса.
    pub fn from_fnv1a(value: u64) -> Self {
        Self(value)
    }
}

/// Отпечатки полных наборов исходных артефактов, переданных сборщиком.
/// В состав program входят все исполняемые образы, в configuration — все их настройки.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct Inputs {
    pub build: String,
    pub catalog: Fingerprint,
    pub wiring: Fingerprint,
    pub program: Fingerprint,
    pub configuration: Fingerprint,
    /// Для конструктора: содержимое непосредственно образа в поле firmware.
    pub firmware_image: Option<Fingerprint>,
    pub legacy_design: Option<u64>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct Identity {
    pub inputs: Inputs,
    pub world: Fingerprint,
    pub physics_engine: String,
    pub task_and_criteria: Fingerprint,
    pub topology: Fingerprint,
    pub seed: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RunKind {
    Live,
    Comparison,
}

pub struct RunPlan<'a> {
    pub case: &'a Case,
    pub path: &'a Path,
    pub mode: &'a Mode,
    pub fault: Option<&'a Injection>,
    kind: RunKind,
}

impl RunPlan<'_> {
    pub fn kind(&self) -> RunKind {
        self.kind
    }

    pub fn stimulus_value(&self, stimulus: &Stimulus) -> f64 {
        match self.kind {
            RunKind::Live => stimulus.value,
            RunKind::Comparison if stimulus.id == self.mode.excitation => {
                // Только режим с заданным отличающимся значением получает Comparison.
                stimulus
                    .comparison_value
                    .expect("значение парного опыта проверено")
            }
            RunKind::Comparison => stimulus.value,
        }
    }
}

struct LiveRun {
    runtime: Runtime,
    status: Result<Status, String>,
    journal: Vec<Event>,
}

/// Полный результат с областью применимости. Сериализация — только для отчёта.
#[derive(Debug, Serialize)]
pub struct Evidence {
    identity: Identity,
    cases: Vec<CaseReport>,
    uncovered_drives: Vec<String>,
    live_board_paths: BTreeSet<String>,
    outcome: Outcome,
}

impl Evidence {
    pub fn identity(&self) -> &Identity {
        &self.identity
    }

    pub fn cases(&self) -> &[CaseReport] {
        &self.cases
    }

    pub fn uncovered_drives(&self) -> &[String] {
        &self.uncovered_drives
    }

    pub fn outcome(&self) -> Outcome {
        self.outcome
    }

    pub fn outcome_for(&self, expected: &Assessor, path: &str) -> Outcome {
        if self.identity != *expected.identity {
            return Outcome::NotAnalyzed;
        }
        Outcome::aggregate(
            self.cases
                .iter()
                .filter(|row| row.case.path == path)
                .map(|row| row.outcome),
        )
    }

    /// Приёмка вывода платы требует образа и пина из установленного контура.
    pub fn board_outcome_for(&self, expected: &Assessor, path: &str) -> Outcome {
        match self.outcome_for(expected, path) {
            Outcome::Pass if !self.live_board_paths.contains(path) => Outcome::NotAnalyzed,
            outcome => outcome,
        }
    }
}

/// Оценка одного живого запуска. Создаётся только методом execute.
#[derive(Debug)]
pub struct RunResult {
    identity: Arc<Identity>,
    report: CaseReport,
    task: Fingerprint,
    live_board: bool,
}

impl RunResult {
    pub fn report(&self) -> &CaseReport {
        &self.report
    }

    /// Свидетель и причины недоступности физики режима, посчитанные исполнителем
    /// задания после прогона. Режим с физикой без свидетеля не завершён.
    pub fn attach_mode_physics(
        &mut self,
        metrics: BTreeMap<String, [f64; 2]>,
        blockers: Vec<String>,
        required: bool,
    ) {
        if required && metrics.is_empty() {
            self.report.completed = false;
        }
        self.report.process_metrics = metrics;
        self.report.missing.extend(blockers);
    }
}

pub(crate) fn validate_world_numbers(world: &CompiledWorld) -> Result<(), Error> {
    for &value in world
        .drives
        .iter()
        .flat_map(|drive| drive.params.values())
        .chain(world.sensors.iter().flat_map(|sensor| {
            sensor
                .params
                .values()
                .chain(&sensor.position_m)
                .chain(&sensor.quat_wxyz)
        }))
        .chain(world.bodies.iter().flat_map(|body| {
            std::iter::once(&body.mass_kg)
                .chain(&body.com_m)
                .chain(&body.inertia_kg_m2)
        }))
    {
        task::finite(value)?;
    }
    Ok(())
}

pub struct Assessor {
    task: Task,
    topology: Topology,
    world: CompiledWorld,
    mode_worlds: BTreeMap<String, CompiledWorld>,
    identity: Arc<Identity>,
    cases: Vec<Case>,
}

impl Assessor {
    pub fn new(
        task: Task,
        topology: Topology,
        world: CompiledWorld,
        inputs: Inputs,
    ) -> Result<Self, Error> {
        task::check_name(&inputs.build)?;
        topology.validate(&world)?;
        validate_world_numbers(&world)?;
        // Debug фиксирован версией сборки. В CompiledWorld и Topology только упорядоченные поля.
        let identity = Arc::new(Identity {
            inputs,
            world: verdict::compiled_world_fingerprint(&world),
            physics_engine: sim_mujoco::version(),
            task_and_criteria: task.assessment_fingerprint(),
            topology: Fingerprint::of(format!("{topology:?}").as_bytes()),
            seed: world.seed,
        });
        let cases = topology.cases();
        Ok(Self {
            task,
            topology,
            world,
            mode_worlds: BTreeMap::new(),
            identity,
            cases,
        })
    }

    pub fn identity(&self) -> &Identity {
        &self.identity
    }

    pub fn topology(&self) -> &Topology {
        &self.topology
    }

    pub fn world(&self) -> &CompiledWorld {
        &self.world
    }

    /// Привязывает миры отдельных режимов к идентичности оценщика.
    pub fn with_mode_worlds(
        mut self,
        worlds: BTreeMap<String, CompiledWorld>,
    ) -> Result<Self, Error> {
        let mut fingerprints = BTreeMap::new();
        for (mode, world) in &worlds {
            if self.task.mode(mode).is_none()
                || world.seed != self.world.seed
                || world.physics != self.world.physics
            {
                return Err(invalid("неизвестный режим или другая сетка или зерно мира режима"));
            }
            self.topology.validate(world)?;
            validate_world_numbers(world)?;
            fingerprints.insert(mode, verdict::compiled_world_fingerprint(world));
        }
        let base = verdict::compiled_world_fingerprint(&self.world);
        Arc::make_mut(&mut self.identity).world = if fingerprints.is_empty() {
            base
        } else {
            Fingerprint::of(format!("{base:?}{fingerprints:?}").as_bytes())
        };
        self.mode_worlds = worlds;
        Ok(self)
    }

    fn world_for(&self, mode: &str) -> &CompiledWorld {
        self.mode_worlds.get(mode).unwrap_or(&self.world)
    }

    pub fn required_cases(&self) -> &[Case] {
        &self.cases
    }

    pub fn plan(&self, case: &Case) -> Option<RunPlan<'_>> {
        let index = self.cases.binary_search(case).ok()?;
        let mode = self.task.mode(&case.mode)?;
        Some(RunPlan {
            case: &self.cases[index],
            path: self
                .topology
                .paths
                .iter()
                .find(|path| path.id == case.path)?,
            mode,
            kind: RunKind::Live,
            fault: match &case.fault {
                Some(fault) => Some(mode.injection(fault)?),
                None => None,
            },
        })
    }

    pub fn execute(
        &self,
        case: &Case,
        install: impl FnMut(&mut Runtime, &RunPlan<'_>, &EventSink) -> Result<Inputs, String>,
    ) -> Result<RunResult, Error> {
        self.execute_recorded(case, install, |_, _, _| Ok(()))
    }

    /// Архиватор получает журнал и события по завершении живого прогона.
    /// Записи пригодны для аудита и не могут быть поданы обратно как доказательство.
    pub fn execute_recorded(
        &self,
        case: &Case,
        install: impl FnMut(&mut Runtime, &RunPlan<'_>, &EventSink) -> Result<Inputs, String>,
        mut archive: impl FnMut(RunKind, &Log, &[Event]) -> Result<(), String>,
    ) -> Result<RunResult, Error> {
        self.execute_observed(case, install, |kind, runtime, events| {
            archive(kind, runtime.log(), events)
        })
    }

    /// Даёт оценщику доступ к журналам физики и контура одного живого прогона.
    pub fn execute_observed(
        &self,
        case: &Case,
        mut install: impl FnMut(&mut Runtime, &RunPlan<'_>, &EventSink) -> Result<Inputs, String>,
        mut archive: impl FnMut(RunKind, &Runtime, &[Event]) -> Result<(), String>,
    ) -> Result<RunResult, Error> {
        let mut live_board = false;
        let report = if let Some(plan) = self.plan(case) {
            match self.run_live(&plan, &mut install)? {
                Ok(live) => {
                    live_board = self.board_binding(&live.runtime, plan.path);
                    let mut report = evaluate::run(
                        &live.runtime,
                        live.status,
                        &plan,
                        self.task.criteria(),
                        &live.journal,
                        &self.topology,
                        self.world_for(&plan.mode.id),
                    )?;
                    if let Err(error) = archive(RunKind::Live, &live.runtime, &live.journal) {
                        report
                            .missing
                            .push(format!("журнал прогона не сохранён: {error}"));
                    }
                    if (report.outcome == Outcome::Pass && report.missing.is_empty())
                        || (report.completed && plan.mode.diagnostic_comparison)
                    {
                        let has_comparison = plan.mode.stimuli.iter().any(|stimulus| {
                            stimulus.id == plan.mode.excitation
                                && stimulus.comparison_value.is_some()
                        });
                        if has_comparison {
                            let comparison = RunPlan {
                                kind: RunKind::Comparison,
                                ..plan
                            };
                            match self.run_live(&comparison, &mut install)? {
                                Ok(reference) => {
                                    evaluate::compare(
                                        &mut report,
                                        &comparison,
                                        &live.runtime,
                                        &reference,
                                    )?;
                                    if let Err(error) = archive(
                                        RunKind::Comparison,
                                        &reference.runtime,
                                        &reference.journal,
                                    ) {
                                        report.missing.push(format!(
                                            "журнал парного прогона не сохранён: {error}"
                                        ));
                                    }
                                }
                                Err(error) => report
                                    .missing
                                    .push(format!("парный прогон не установлен: {error}")),
                            }
                        } else {
                            report.missing.push(
                                "нет парного возбуждения для проверки зависимости команды".into(),
                            );
                        }
                    }
                    report.finish()
                }
                Err(error) => {
                    let mut report =
                        CaseReport::not_analyzed(case.clone(), "свежий живой прогон не установлен");
                    report.runtime_error = Some(error);
                    report
                }
            }
        } else {
            CaseReport::not_analyzed(
                case.clone(),
                "нет обязательного режима или расписания отказа",
            )
        };
        Ok(RunResult {
            report,
            task: self.task.fingerprint(),
            identity: self.identity.clone(),
            live_board,
        })
    }

    fn board_binding(&self, runtime: &Runtime, path: &Path) -> bool {
        let (Some(binding), Some(run)) = (&path.legacy, sim_runtime::control::run(runtime)) else {
            return false;
        };
        run.origin() == sim_runtime::control::RunOrigin::Live
            && self.identity.inputs.firmware_image == Some(path.program)
            && run.paths().iter().any(|actual| {
                actual.name == path.id
                    && actual.producer == path.controller
                    && actual.target == path.command.name
                    && Fingerprint::from_fnv1a(actual.program_hash) == path.program
                    && actual.board_output_pin == Some(binding.pin)
            })
    }

    fn run_live(
        &self,
        plan: &RunPlan<'_>,
        install: &mut impl FnMut(&mut Runtime, &RunPlan<'_>, &EventSink) -> Result<Inputs, String>,
    ) -> Result<Result<LiveRun, String>, Error> {
        let events = EventSink::default();
        let mut runtime = Runtime::load(self.world_for(&plan.mode.id).clone()).map_err(invalid)?;
        let original = runtime.snapshot();
        let installation = install(&mut runtime, plan, &events).and_then(|installed| {
            if installed != self.identity.inputs {
                return Err("отпечатки установленного каталога, проводки, программы или её настроек отличаются".into());
            }
            runtime.register_evaluator("sim-task/fresh-run", runtime.physics_rate(), live::FreshRun)?;
            // Снимок привязан к экземпляру и выявляет подмену мира в установщике.
            runtime.verify_snapshot_owner(&original)
        });
        if let Err(error) = installation {
            return Ok(Err(error));
        }
        if let Some(run) = sim_runtime::control::run(&runtime) {
            let actual = run.identity();
            if Fingerprint::from_fnv1a(actual.task) != self.task.fingerprint()
                || Fingerprint::from_fnv1a(actual.criteria) != self.task.fingerprint()
                || Fingerprint::from_fnv1a(actual.catalog) != self.identity.inputs.catalog
                || actual.build != self.identity.inputs.build
                || actual.mode != plan.mode.id
                || actual.phase_horizon != Time(plan.mode.horizon_ns)
                || actual.seed != self.identity.seed
            {
                return Ok(Err(
                    "установленный контур относится к другому плану или сборке".into(),
                ));
            }
        }
        events.open();
        let status = runtime.run_until(Time(plan.mode.horizon_ns));
        let journal = events.close()?;
        let journal = match sim_runtime::control::run(&runtime) {
            Some(run) => live_control::events(&run, plan),
            None => journal,
        };
        Ok(Ok(LiveRun {
            runtime,
            status,
            journal,
        }))
    }

    pub fn assess(&self, results: &[RunResult]) -> Evidence {
        let mut by_case = BTreeMap::new();
        for result in results.iter().filter(|run| *run.identity == *self.identity) {
            let row = &result.report;
            let entry = by_case.entry(&row.case).or_insert(result);
            // Повторный прогон может выявить нарушение. Пробел также сохраняется.
            if row.outcome > entry.report.outcome
                || (row.outcome == entry.report.outcome && !result.live_board)
            {
                *entry = result;
            }
        }
        let cases: Vec<_> = self
            .cases
            .iter()
            .map(|case| {
                by_case
                    .get(case)
                    .map(|result| result.report.clone())
                    .unwrap_or_else(|| {
                        let cause = if self.plan(case).is_some() {
                            "нет подходящего живого прогона"
                        } else {
                            "нет обязательного режима или расписания отказа"
                        };
                        CaseReport::not_analyzed(case.clone(), cause)
                    })
            })
            .collect();
        let uncovered_drives: Vec<_> = self
            .world
            .drives
            .iter()
            .filter(|drive| {
                !self
                    .topology
                    .paths
                    .iter()
                    .any(|path| path.drive == drive.name)
            })
            .map(|drive| drive.name.clone())
            .collect();
        let mut outcome = Outcome::aggregate(cases.iter().map(|row| row.outcome));
        if !uncovered_drives.is_empty() {
            outcome = outcome.max(Outcome::NotAnalyzed);
        }
        let live_board_paths = self
            .topology
            .paths
            .iter()
            .filter(|path| {
                self.cases
                    .iter()
                    .filter(|case| case.path == path.id)
                    .all(|case| by_case.get(case).is_some_and(|result| result.live_board))
            })
            .map(|path| path.id.clone())
            .collect();
        Evidence {
            identity: (*self.identity).clone(),
            cases,
            uncovered_drives,
            live_board_paths,
            outcome,
        }
    }
}
