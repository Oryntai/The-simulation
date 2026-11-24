//! Живые наблюдения и адаптеры предметных оценщиков.

use std::collections::{BTreeMap, BTreeSet};

use sim_runtime::{Log, Runtime, Status};
use sim_signal::{Command, Time};
use sim_world::{StrengthSection, strength_section};

use crate::{
    Error, Fingerprint, Limit, Outcome, RunResult, Source, SourceKind,
    campaign::{
        CampaignPlan, Disposition, PlannedFault, ReferenceKind, RunKey, SampledInputs, Step,
        Target, Window,
    },
    interaction::InteractionEvidence,
    invalid,
    metrics::{
        self, FaultObservation, Installation, Point, RunSummary, Series, SeriesKind,
        StepObservation,
    },
    process::ConvergenceReport,
    sampling::{ModelSettings, Parameter, delay_ns},
};

use super::{
    Check, ControlCriterion, Finding, HazardClass, LimitValue, Requirement, RequirementKey,
    RequirementSet, Verification, checked_pair, compiled_world_fingerprint, fault_class,
    limit_source, provenance, requires_fault, trace,
};

/// Возвращает разрешитель установки в той же границе доверия, что и сборщик контура.
/// Параметры должны быть считаны из установленных моделей.
#[derive(Clone, Debug)]
pub struct Installed {
    pub installation: Installation,
    pub build: Fingerprint,
    pub settings: ModelSettings,
    pub operations: Vec<StepObservation>,
}

/// Линия стендового журнала, уже разрешённого владельцем формата.
#[derive(Clone, Debug)]
pub struct MeasurementSample {
    pub at_ns: u64,
    pub value: f64,
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
pub struct SnapshotAudit {
    pub at_ns: u64,
    pub matched: bool,
    pub reason: Option<String>,
}

#[derive(Debug)]
struct Measurement {
    signal: crate::Signal,
    source: Source,
    journal: Fingerprint,
    samples: Vec<Point>,
    uncertainty: Option<f64>,
}

#[derive(Debug)]
struct Run {
    sampled: SampledInputs,
    build: Option<Fingerprint>,
    log: Log,
    divergent_repeat: Option<Log>,
    journal: u64,
    sources: Vec<Source>,
    installation_confirmed: bool,
    missing: Vec<String>,
    scenario: Outcome,
    summary: Option<RunSummary>,
    snapshot: Option<SnapshotAudit>,
    faults_confirmed: bool,
    planned_faults: Vec<PlannedFault>,
    task: Option<Fingerprint>,
    process: Option<(Fingerprint, Outcome, Vec<Finding>)>,
    interaction: Option<(Fingerprint, Outcome, Vec<Finding>)>,
    control: Option<(Fingerprint, bool, crate::CaseReport)>,
    linked_profile: Option<Fingerprint>,
    profiles: BTreeMap<ReferenceKind, BTreeSet<Fingerprint>>,
    bench: Vec<Measurement>,
}

/// Неподменяемые входы оценщика. Дисковый вердикт и статистические строки сюда не загружаются.
#[derive(Debug)]
pub struct CampaignEvidence {
    pub(super) stamp: Fingerprint,
    runs: BTreeMap<RunKey, Run>,
    unavailable: BTreeMap<RunKey, Vec<String>>,
}

impl CampaignEvidence {
    pub fn new(requirements: &RequirementSet) -> Self {
        Self {
            stamp: requirements.stamp,
            runs: BTreeMap::new(),
            unavailable: BTreeMap::new(),
        }
    }

    /// Регистрирует причину незапущенной строки, например ошибку установки или фильтрацию.
    pub fn record_unavailable(
        &mut self,
        requirements: &RequirementSet,
        key: &RunKey,
        reason: String,
    ) -> Result<(), Error> {
        self.matches(requirements)?;
        crate::task::check_name(&reason)?;
        if self.runs.contains_key(key) || !requirements.keys().any(|k| k.run.as_ref() == Some(key))
        {
            return Err(invalid(
                "строка отсутствует в обязательствах или уже исполнена",
            ));
        }
        self.unavailable
            .entry(key.clone())
            .or_default()
            .push(reason);
        Ok(())
    }

    fn matches(&self, requirements: &RequirementSet) -> Result<(), Error> {
        if self.stamp == requirements.stamp {
            Ok(())
        } else {
            Err(invalid("другой набор требований кампании"))
        }
    }

    /// Дважды исполняет одну строку на свежих экземплярах. Инсталлятор регистрирует
    /// компоненты, подтверждает фактические установки и планирует действия режима.
    /// Сборщик не имеет права заменять экземпляр, загружать лог или шагать сам.
    pub fn execute(
        &mut self,
        requirements: &RequirementSet,
        plan: &CampaignPlan,
        key: &RunKey,
        install: impl FnMut(&mut Runtime, &SampledInputs) -> Result<Installed, String>,
    ) -> Result<(), Error> {
        self.execute_inner(
            requirements,
            plan,
            key,
            None,
            install,
            |_, _| Ok(None),
            false,
            None,
            |_| Ok(()),
        )
    }

    /// Оценщик сам исполняет шаги усилия. Прочие операции передаются
    /// адаптеру в заданное время, а их принятие сверяется с расписанием.
    pub fn execute_timed(
        &mut self,
        requirements: &RequirementSet,
        plan: &CampaignPlan,
        key: &RunKey,
        install: impl FnMut(&mut Runtime, &SampledInputs) -> Result<Installed, String>,
        operation: impl FnMut(&mut Runtime, &Step) -> Result<Option<StepObservation>, String>,
    ) -> Result<(), Error> {
        self.execute_inner(
            requirements,
            plan,
            key,
            None,
            install,
            operation,
            true,
            None,
            |_| Ok(()),
        )
    }

    /// Исполняет сценарий и проверяет продолжение со снимка в том же экземпляре.
    pub fn execute_with_snapshot(
        &mut self,
        requirements: &RequirementSet,
        plan: &CampaignPlan,
        key: &RunKey,
        at: Time,
        install: impl FnMut(&mut Runtime, &SampledInputs) -> Result<Installed, String>,
    ) -> Result<(), Error> {
        self.execute_scenario_observed(requirements, plan, key, Some(at), install, |_| Ok(()))
    }

    /// Передаёт наблюдателю живой экземпляр, журнал которого идёт в оценку.
    pub fn execute_scenario_observed(
        &mut self,
        requirements: &RequirementSet,
        plan: &CampaignPlan,
        key: &RunKey,
        snapshot_at: Option<Time>,
        install: impl FnMut(&mut Runtime, &SampledInputs) -> Result<Installed, String>,
        observe: impl FnOnce(&Runtime) -> Result<(), String>,
    ) -> Result<(), Error> {
        self.execute_inner(
            requirements,
            plan,
            key,
            None,
            install,
            |_, _| Ok(None),
            true,
            snapshot_at,
            observe,
        )
    }

    pub fn scenario_report(
        &self,
        key: &RunKey,
    ) -> Option<(&RunSummary, u64, u64, Option<&SnapshotAudit>)> {
        let run = self.runs.get(key)?;
        Some((
            run.summary.as_ref()?,
            run.journal,
            run.log.end().steps,
            run.snapshot.as_ref(),
        ))
    }

    /// Настройки, считанные из живого рантайма и сверенные с постановкой.
    pub fn confirmed_settings(&self, key: &RunKey) -> Option<&ModelSettings> {
        let run = self.runs.get(key)?;
        run.installation_confirmed.then_some(&run.sampled.settings)
    }

    /// Для связанного профиля допустим расширенный мир с точным отпечатком из инвентаря.
    pub fn execute_linked(
        &mut self,
        requirements: &RequirementSet,
        plan: &CampaignPlan,
        key: &RunKey,
        world: sim_world::CompiledWorld,
        install: impl FnMut(&mut Runtime, &SampledInputs) -> Result<Installed, String>,
    ) -> Result<(), Error> {
        self.execute_inner(
            requirements,
            plan,
            key,
            Some(world),
            install,
            |_, _| Ok(None),
            false,
            None,
            |_| Ok(()),
        )
    }

    fn execute_inner(
        &mut self,
        requirements: &RequirementSet,
        plan: &CampaignPlan,
        key: &RunKey,
        expanded: Option<sim_world::CompiledWorld>,
        mut install: impl FnMut(&mut Runtime, &SampledInputs) -> Result<Installed, String>,
        mut operation: impl FnMut(&mut Runtime, &Step) -> Result<Option<StepObservation>, String>,
        timed: bool,
        snapshot_at: Option<Time>,
        observe: impl FnOnce(&Runtime) -> Result<(), String>,
    ) -> Result<(), Error> {
        self.matches(requirements)?;
        if plan.fingerprint() != requirements.plan || self.runs.contains_key(key) {
            return Err(invalid("другой план или повторная регистрация ключа"));
        }
        let row = plan
            .row(key)
            .ok_or_else(|| invalid("ключ отсутствует в плане"))?;
        if !matches!(row.disposition, Disposition::Ready) {
            return Err(invalid("строка плана не готова к исполнению"));
        }
        let sampled = plan.materialize(key)?;
        let mode = sampled
            .mode
            .as_ref()
            .ok_or_else(|| invalid("нет обязательного режима"))?;
        if snapshot_at.is_some_and(|at| at.0 == 0 || at.0 >= mode.horizon_ns) {
            return Err(invalid("снимок должен находиться внутри горизонта"));
        }
        let compiled = match (expanded, requirements.compiled_rows.get(key)) {
            (Some(world), Some(expected))
                if compiled_world_fingerprint(&world) == *expected
                    && world.seed == sampled.sample.seeds.run
                    && world.physics.as_hz() == row.grid.hz as f64 =>
            {
                world
            }
            (None, None) => sampled.world.compile()?,
            _ => {
                return Err(invalid(
                    "расширенный мир не совпал с разрешённой установкой",
                ));
            }
        };
        crate::validate_world_numbers(&compiled)?;
        let mut profiles: BTreeMap<_, BTreeSet<_>> = BTreeMap::new();
        for check in requirements
            .requirements
            .iter()
            .filter(|r| r.key.run.as_ref() == Some(key))
            .flat_map(|r| &r.checks)
        {
            let (kind, fingerprint) = match check.verification {
                Verification::Process { profile, .. } => (ReferenceKind::ProcessTask, profile),
                Verification::Interaction { profile, .. } => {
                    (ReferenceKind::InteractionTask, profile)
                }
                Verification::Control { criteria, .. } => (ReferenceKind::ControlTask, criteria),
                _ => continue,
            };
            profiles.entry(kind).or_default().insert(fingerprint);
        }
        let mut observe = Some(observe);
        let mut run_one = |snapshot_at: Option<Time>| -> Result<_, Error> {
            let mut runtime = Runtime::load(compiled.clone()).map_err(invalid)?;
            let owner = runtime.snapshot();
            let mut receipt = install(&mut runtime, &sampled).map_err(invalid)?;
            runtime.verify_snapshot_owner(&owner).map_err(invalid)?;
            if !runtime.process_run_is_fresh() {
                return Err(invalid(
                    "инсталлятор вернул начатый или воспроизведённый прогон",
                ));
            }
            let installation_confirmed = Some(receipt.build) == requirements.build
                && metrics::installation_matches(&sampled, &receipt.installation, row.grid.hz)
                && same_settings(&sampled.settings, &receipt.settings)
                && settings_match_runtime(&sampled.settings, &runtime);
            let mut missing = Vec::new();
            if !installation_confirmed {
                missing
                    .push("не совпадают сборка, отпечатки, зерно или фактические установки".into());
            }
            if timed && sampled.experiment.as_ref().and_then(|e| e.kind).is_some() {
                return Err(invalid(
                    "расписание связанного опыта принадлежит его установщику",
                ));
            }
            let mut advance = |runtime: &mut Runtime,
                               steps: &[Step],
                               operations: &mut Vec<StepObservation>,
                               until: Time|
             -> Result<_, Error> {
                if !timed {
                    return Ok(runtime.run_until(until));
                }
                for step in steps {
                    if step.at_ns > runtime.time().0 {
                        let status = runtime.run_until(Time(step.at_ns));
                        if status != Ok(Status::Running) {
                            return Ok(status);
                        }
                    }
                    if let Target::Effort { name } = &step.target {
                        let id = runtime
                            .channel(name)
                            .ok_or_else(|| invalid(format!("нет входа усилия «{name}»")))?;
                        runtime
                            .act(&Command {
                                targets: vec![(id, step.value)],
                            })
                            .map_err(invalid)?;
                    } else if let Some(observed) = operation(runtime, step).map_err(invalid)? {
                        operations.push(observed);
                    }
                }
                Ok(runtime.run_until(until))
            };
            let horizon = Time(mode.horizon_ns);
            let mut snapshot = None;
            let status = if let Some(at) = snapshot_at {
                let (before, after) = mode
                    .steps
                    .split_at(mode.steps.partition_point(|s| s.at_ns < at.0));
                let prefix = advance(&mut runtime, before, &mut receipt.operations, at)?;
                let checkpoint = if prefix == Ok(Status::Running) && runtime.time() == at {
                    runtime.try_snapshot()
                } else {
                    Err("не достигнут заявленный такт снимка".into())
                };
                let prefix_operations = receipt.operations.clone();
                let status = advance(&mut runtime, after, &mut receipt.operations, horizon)?;
                let audit = checkpoint.and_then(|checkpoint| {
                    let foreign = Runtime::load(compiled.clone())?;
                    if foreign.verify_snapshot_owner(&checkpoint).is_ok() {
                        return Err("принят снимок другого экземпляра".into());
                    }
                    let mut uninterrupted = Vec::new();
                    runtime
                        .log()
                        .write_binary(&mut uninterrupted)
                        .map_err(|e| e.to_string())?;
                    runtime.restore(&checkpoint)?;
                    let mut continued_operations = prefix_operations;
                    let resumed_status =
                        advance(&mut runtime, after, &mut continued_operations, horizon)
                            .map_err(|e| e.to_string())?;
                    let mut resumed = Vec::new();
                    runtime
                        .log()
                        .write_binary(&mut resumed)
                        .map_err(|e| e.to_string())?;
                    if uninterrupted != resumed
                        || status != resumed_status
                        || format!("{:?}", receipt.operations)
                            != format!("{continued_operations:?}")
                    {
                        return Err("журнал или завершение изменились после снимка".into());
                    }
                    Ok(())
                });
                snapshot = Some(SnapshotAudit {
                    at_ns: at.0,
                    matched: audit.is_ok(),
                    reason: audit.err(),
                });
                status
            } else {
                advance(&mut runtime, &mode.steps, &mut receipt.operations, horizon)?
            };
            let log = runtime.log();
            if status != Ok(Status::Running)
                || log.end().time.0 != mode.horizon_ns
                || log.end().steps == 0
            {
                missing.push(format!(
                    "не достигнут горизонт: {status:?} {:?}",
                    runtime.failure()
                ));
            }
            for entry in log.entries() {
                crate::task::finite(entry.value)?;
            }
            for value in receipt
                .installation
                .values
                .values()
                .chain(receipt.operations.iter().map(|s| &s.value))
                .chain(
                    receipt
                        .settings
                        .initial_velocity
                        .values()
                        .flat_map(|v| v.values()),
                )
            {
                crate::task::finite(*value)?;
            }
            for limitation in runtime.simplifications().iter().filter(|s| s.blocks_proof) {
                missing.push(format!("{}: {}", limitation.subject, limitation.text));
            }
            for channel in sampled.settings.channel_delay_ns.keys() {
                if !runtime.channel_delay_observed(channel) {
                    missing.push(format!("нет живой доставки с задержкой «{channel}»"));
                }
            }
            for sample in sampled.values() {
                let Parameter::SensorLatency { sensor } = &sample.parameter else {
                    continue;
                };
                let expected = delay_ns(sample.value).expect("задержка проверена");
                let prefix = format!("{sensor}/");
                let observations = runtime.observe();
                let delivered = observations.samples.iter().any(|observation| {
                    if !observation.valid {
                        return false;
                    }
                    let Some(spec) = log.header().observations.get(observation.signal.0 as usize)
                    else {
                        return false;
                    };
                    if !spec.name.starts_with(&prefix) {
                        return false;
                    }
                    let extra = sampled
                        .settings
                        .channel_delay_ns
                        .get(&format!("observation:{}", spec.name))
                        .copied()
                        .unwrap_or(0);
                    sim_runtime::control::measurement_time(&runtime, observation.signal)
                        .is_some_and(|t| {
                            t.delivered_at.0.checked_sub(t.captured_at.0)
                                == expected.checked_add(extra)
                        })
                });
                if !delivered {
                    missing.push(format!(
                        "не подтверждена задержка захвата и доставки датчика «{sensor}»"
                    ));
                }
            }
            if let Some(control) = sim_runtime::control::run(&runtime)
                && (control.origin() != sim_runtime::control::RunOrigin::Live
                    || control.has_external_processes())
            {
                missing.push(
                    "не подтверждены происхождение или изоляция управляющего процесса".into(),
                );
            }
            let mut faults = Vec::new();
            for fault in &row.faults {
                if let Some(observation) = FaultObservation::from_state(fault, log)? {
                    faults.push(observation);
                }
            }
            let complete_faults = row.faults.len() == faults.len()
                && row
                    .faults
                    .iter()
                    .all(|fault| metrics::matching_fault(fault, &faults, log.end().time.0));
            let summary = if sampled.experiment.as_ref().and_then(|e| e.kind).is_none() {
                Some(metrics::evaluate_scenario(
                    row,
                    &sampled,
                    log,
                    receipt.installation,
                    &receipt.operations,
                    &faults,
                )?)
            } else {
                None
            };
            if let Some(observe) = observe.take() {
                observe(&runtime).map_err(invalid)?;
            }
            Ok((
                log.clone(),
                complete_faults,
                missing,
                summary,
                installation_confirmed,
                snapshot,
            ))
        };
        let (log, faults_confirmed, mut missing, summary, installation_confirmed, _) =
            run_one(None)?;
        let (repeat, repeat_faults, repeat_missing, repeat_summary, repeat_installed, mut snapshot) =
            run_one(snapshot_at)?;
        let mut scenario = summary.as_ref().map_or(Outcome::NotAnalyzed, |r| r.outcome);
        let repeat_scenario = repeat_summary
            .as_ref()
            .map_or(Outcome::NotAnalyzed, |r| r.outcome);
        let journal = log.fingerprint();
        let repeated_journal = repeat.fingerprint();
        if journal != repeated_journal || scenario != repeat_scenario || !repeat_installed {
            missing.push("повтор живого прогона не совпал".into());
            if let Some(audit) = &mut snapshot
                && audit.matched
            {
                audit.matched = false;
                audit.reason =
                    Some("результат отличается от отдельного непрерывного прогона".into());
            }
        }
        if let Some(reason) = snapshot.as_ref().and_then(|audit| audit.reason.as_ref()) {
            missing.push(reason.clone());
        }
        // Нарушение во втором живом повторе сохраняется при рассогласовании с первым.
        if repeat_installed {
            scenario = scenario.max(repeat_scenario);
        }
        missing.extend(repeat_missing);
        missing.sort();
        missing.dedup();
        let sources = std::iter::once(&mode.source)
            .chain(mode.initial.iter().map(|i| &i.source))
            .chain(mode.poses.iter().map(|i| &i.source))
            .chain(mode.steps.iter().map(|i| &i.source))
            .chain(std::iter::once(&row.profile_source))
            .chain(std::iter::once(&row.grid.source))
            .chain(&row.combination_source)
            .chain(row.faults.iter().map(|f| &f.schedule.source))
            .chain(mode.metrics.iter().map(|metric| &metric.source))
            .chain(sampled.values().iter().flat_map(|v| &v.sources))
            .cloned()
            .chain(compiled.bodies.iter().map(|b| provenance(&b.provenance)))
            .collect();
        self.unavailable.remove(key);
        self.runs.insert(
            key.clone(),
            Run {
                sampled,
                build: requirements.build,
                journal,
                log,
                divergent_repeat: (repeat_installed && journal != repeated_journal)
                    .then_some(repeat),
                sources,
                installation_confirmed,
                missing,
                scenario,
                summary,
                snapshot,
                faults_confirmed: faults_confirmed && repeat_faults,
                planned_faults: row.faults.clone(),
                task: plan.task_fingerprint(key),
                process: None,
                interaction: None,
                control: None,
                linked_profile: requirements.linked_profiles.get(key).copied(),
                profiles,
                bench: Vec::new(),
            },
        );
        Ok(())
    }

    fn run_mut(&mut self, key: &RunKey) -> Result<&mut Run, Error> {
        self.runs
            .get_mut(key)
            .filter(|r| r.installation_confirmed)
            .ok_or_else(|| invalid("нет связанной живой строки с подтверждённой установкой"))
    }

    /// Принимает результат только собственного живого оценщика с тем же журналом и областью.
    pub fn record_control(&mut self, key: &RunKey, result: &RunResult) -> Result<(), Error> {
        let run = self.run_mut(key)?;
        let report = result.report();
        let scheduled = key.faults.first();
        if run.control.is_some()
            || run.task != Some(result.task)
            || !run.accepts(
                ReferenceKind::ControlTask,
                result.identity.task_and_criteria,
            )
            || report.journal != Some(run.journal)
            || report.case.path != key.path
            || report.case.mode != key.mode
            || report.case.fault.as_ref() != scheduled
            || key.faults.len() > 1
            || result.identity.seed != run.sampled.sample.seeds.run
            || run.sampled.experiment.as_ref().and_then(|e| e.kind)
                != Some(crate::campaign::ReferenceKind::ControlTask)
        {
            return Err(invalid("другой журнал, контур, режим или отказ"));
        }
        if let [fault] = run.planned_faults.as_slice()
            && report.outcome == Outcome::Pass
            && report.requested_ns == Some(fault.schedule.at_ns)
            && report.applied_ns.is_some_and(|at| {
                (fault.schedule.at_ns..=fault.schedule.at_ns + fault.schedule.max_lag_ns)
                    .contains(&at)
                    && at > fault.window.start_ns
                    && at < fault.window.end_ns
            })
            && report
                .before_loop
                .as_ref()
                .is_some_and(|before| before.feedback_ns < report.applied_ns.unwrap())
            && report
                .reached_ns
                .is_some_and(|at| at >= fault.window.end_ns)
        {
            run.faults_confirmed = true;
        }
        run.control = Some((
            result.identity.task_and_criteria,
            result.live_board,
            report.clone(),
        ));
        Ok(())
    }

    /// Сохраняет метрики, события и опасности только от живой серии с совпадающим базовым журналом.
    pub fn record_process(
        &mut self,
        key: &RunKey,
        evidence: &ConvergenceReport,
    ) -> Result<(), Error> {
        let run = self.run_mut(key)?;
        if run.process.is_some()
            || !run.accepts(ReferenceKind::ProcessTask, evidence.profile())
            || !evidence.is_live()
            || !evidence
                .trials
                .first()
                .is_some_and(|base| base.journal == Some(run.journal))
        {
            return Err(invalid("другой или изменённый отчёт о физических сетках"));
        }
        run.process = Some((
            evidence.profile(),
            evidence.outcome,
            evidence.campaign_findings(key),
        ));
        Ok(())
    }

    /// Переносит раздельные выводы по конструкции, тканям и достижимым путям энергии.
    pub fn record_interaction(
        &mut self,
        key: &RunKey,
        evidence: &InteractionEvidence,
    ) -> Result<(), Error> {
        let build = evidence.campaign_build();
        let run = self.run_mut(key)?;
        if run.interaction.is_some()
            || !run.accepts(ReferenceKind::InteractionTask, evidence.campaign_profile())
            || Some(build) != run.build
            || !evidence
                .grids()
                .iter()
                .any(|g| g.journal == Fingerprint::from_fnv1a(run.journal))
        {
            return Err(invalid("другое взаимодействие, сборка или журнал"));
        }
        run.interaction = Some((
            evidence.campaign_profile(),
            evidence.outcome(),
            evidence.campaign_findings(key),
        ));
        Ok(())
    }

    /// Принимает стендовый ряд для этой сборки, полного ключа и канала.
    /// Источник должен идентифицировать замер, его калибровку и область.
    pub fn record_measurement(
        &mut self,
        requirements: &RequirementSet,
        key: &RunKey,
        build: Fingerprint,
        signal: crate::Signal,
        source: Source,
        uncertainty: Option<f64>,
        samples: &[MeasurementSample],
    ) -> Result<(), Error> {
        self.matches(requirements)?;
        if Some(build) != requirements.build || source.kind != SourceKind::Measured {
            return Err(invalid(
                "замер не принадлежит сборке или не имеет измеренного источника",
            ));
        }
        source.validate()?;
        signal.validate()?;
        let run = self.run_mut(key)?;
        if run.bench.iter().any(|m| m.signal == signal) {
            return Err(invalid("повторный стендовый канал"));
        }
        let points: Vec<_> = samples
            .iter()
            .map(|sample| Point {
                at_ns: sample.at_ns,
                value: sample.value,
                valid: true,
            })
            .collect();
        Series {
            signal: signal.clone(),
            points: points.clone(),
        }
        .validate()?;
        if uncertainty.is_some_and(|u| {
            !u.is_finite() || u < 0.0 || points.iter().any(|p| !(p.value.abs() + u).is_finite())
        }) {
            return Err(invalid("некорректная граница погрешности стендового канала"));
        }
        let journal = Fingerprint::of(
            format!("{build:?}{key:?}{signal:?}{source:?}{uncertainty:?}{points:?}").as_bytes(),
        );
        run.bench.push(Measurement {
            signal,
            source,
            journal,
            samples: points,
            uncertainty,
        });
        Ok(())
    }

    pub(super) fn diagnostics(&self, set: &RequirementSet) -> Vec<Finding> {
        self.runs
            .values()
            .filter(|run| run.installation_confirmed)
            .flat_map(|run| {
                run.details().cloned().map(|mut finding| {
                    run.finish(set, &mut finding);
                    finding
                })
            })
            .collect()
    }

    pub(super) fn findings(
        &self,
        set: &RequirementSet,
        requirement: &Requirement,
        output: &mut Vec<Finding>,
    ) -> Result<(), Error> {
        let key = &requirement.key;
        let Some(run) = key
            .run
            .as_ref()
            .and_then(|k| self.runs.get(k))
            .filter(|r| r.installation_confirmed)
        else {
            let mut finding = Finding::open(
                key,
                "нет подходящего живого прогона с подтверждённой установкой",
            );
            if let Some(run) = key.run.as_ref().and_then(|k| self.runs.get(k)) {
                finding.coverage_gaps.extend(run.missing.iter().cloned());
            }
            if let Some(reasons) = key.run.as_ref().and_then(|k| self.unavailable.get(k)) {
                finding.coverage_gaps.extend(reasons.iter().cloned());
            }
            output.push(finding);
            return Ok(());
        };
        if requirement.checks.is_empty() {
            if key.class == HazardClass::Execution {
                output.push(run.execution(set, key));
            } else {
                output.extend(
                    run.details()
                        .filter(|f| {
                            f.key.class == key.class
                                && f.key.object == key.object
                                && f.outcome == Outcome::NotAnalyzed
                        })
                        .cloned(),
                );
                output.push(Finding::open(
                    key,
                    "нет применимого метода, предела или модели",
                ));
            }
            return Ok(());
        }
        for check in &requirement.checks {
            output.extend(run.check(set, requirement, check)?);
        }
        Ok(())
    }
}

fn same_settings(a: &ModelSettings, b: &ModelSettings) -> bool {
    a.channel_delay_ns == b.channel_delay_ns
        && a.initial_velocity.len() == b.initial_velocity.len()
        && a.initial_velocity.iter().all(|(key, dofs)| {
            b.initial_velocity.get(key).is_some_and(|other| {
                dofs.len() == other.len()
                    && dofs.iter().all(|(dof, value)| {
                        other
                            .get(dof)
                            .is_some_and(|v| v.to_bits() == value.to_bits())
                    })
            })
        })
}

fn settings_match_runtime(settings: &ModelSettings, runtime: &Runtime) -> bool {
    settings.initial_velocity.iter().all(|(joint, dofs)| {
        dofs.iter().all(|(&dof, &expected)| {
            runtime
                .joint_velocity(joint, dof)
                .is_some_and(|actual| actual.to_bits() == expected.to_bits())
        })
    }) && settings
        .channel_delay_ns
        .iter()
        .all(|(channel, &expected)| runtime.channel_delay_ns(channel) == Some(expected))
}

impl Run {
    fn expected_kind(&self) -> Option<ReferenceKind> {
        self.sampled.experiment.as_ref().and_then(|e| e.kind)
    }

    fn accepts(&self, kind: ReferenceKind, fingerprint: Fingerprint) -> bool {
        if self.expected_kind() == Some(kind) {
            self.linked_profile == Some(fingerprint)
        } else {
            self.expected_kind().is_none()
                && self
                    .profiles
                    .get(&kind)
                    .is_some_and(|set| set.contains(&fingerprint))
        }
    }

    fn details(&self) -> impl Iterator<Item = &Finding> {
        self.process
            .iter()
            .chain(&self.interaction)
            .flat_map(|(_, _, findings)| findings)
    }

    fn journal(&self) -> Fingerprint {
        Fingerprint::from_fnv1a(self.journal)
    }

    fn execution(&self, set: &RequirementSet, key: &RequirementKey) -> Finding {
        let mut finding = Finding::open(key, "покрытие выполнения режима");
        finding.journal = Some(self.journal());
        finding.outcome = self.scenario;
        if let Some((_, _, control)) = &self.control {
            finding.outcome = control.outcome;
            finding
                .coverage_gaps
                .extend(control.missing.iter().cloned());
            if !control.violations.is_empty() {
                finding.reason = control.violations.join(" / ");
            }
        } else if let Some(kind) = self.expected_kind() {
            let result = match kind {
                ReferenceKind::ProcessTask => self.process.as_ref(),
                ReferenceKind::InteractionTask => self.interaction.as_ref(),
                _ => None,
            };
            finding.outcome = result.map_or(Outcome::NotAnalyzed, |(_, outcome, _)| *outcome);
            finding.coverage_gaps.extend(
                self.details()
                    .filter(|f| f.key.class == HazardClass::Execution)
                    .flat_map(|f| &f.coverage_gaps)
                    .cloned(),
            );
        }
        self.finish(set, &mut finding);
        finding
    }

    fn finish(&self, set: &RequirementSet, finding: &mut Finding) {
        let (proved, sources) = trace(&self.sources, &set.derivations);
        finding.measurement_sources.extend(sources);
        finding.measurement_sources.sort();
        finding.measurement_sources.dedup();
        finding.coverage_gaps.extend(self.missing.iter().cloned());
        if !self.faults_confirmed {
            finding
                .coverage_gaps
                .push("нет подтверждения всех отказов в окне до и после".into());
        }
        if !proved {
            finding
                .coverage_gaps
                .push("есть оценка или неполная цепочка происхождения входов".into());
        }
        if finding.outcome == Outcome::Pass && !finding.coverage_gaps.is_empty() {
            finding.outcome = Outcome::NotAnalyzed;
        }
        if finding.outcome != Outcome::NotAnalyzed {
            finding.missing_data.clear();
        }
        finding.confirmed_hazard &= proved
            && self.faults_confirmed
            && self.installation_confirmed
            && self.log.end().steps > 0
            && self
                .sampled
                .mode
                .as_ref()
                .is_some_and(|m| self.log.end().time.0 == m.horizon_ns);
    }

    fn check(
        &self,
        set: &RequirementSet,
        requirement: &Requirement,
        check: &Check,
    ) -> Result<Vec<Finding>, Error> {
        let key = &requirement.key;
        let mut finding = Finding::open(key, "неполное покрытие проверки");
        finding.model_scope = Some(check.scope.clone());
        finding.journal = Some(self.journal());
        let mut sources = check.inputs.clone();
        sources.extend_from_slice(&requirement.declared);
        sources.push(check.applicability.clone());
        let details = match &check.verification {
            Verification::Section {
                geometry,
                axis,
                moment_nm,
                allowable_pa,
                load,
                limit,
            } => {
                finding.limit = Some(LimitValue::upper(*allowable_pa));
                finding.unit = Some("Pa".into());
                sources.push(load.clone());
                finding.limit_sources = trace(std::slice::from_ref(limit), &set.derivations).1;
                let world = &self.sampled.world;
                let geometries = key
                    .object
                    .split_once('/')
                    .and_then(|(device, link)| {
                        world
                            .devices
                            .get(device)?
                            .device
                            .links
                            .iter()
                            .find(|l| l.name == link)
                            .map(|l| &l.collision)
                    })
                    .or_else(|| {
                        world
                            .world
                            .objects
                            .iter()
                            .find(|o| o.name == key.object)
                            .map(|o| &o.collision)
                    });
                if let Some(shape) = geometries.and_then(|g| g.get(*geometry)).map(|g| &g.shape) {
                    match strength_section(shape, *axis).map_err(invalid)? {
                        StrengthSection::Applicable { section, .. } => {
                            let abs = moment_nm.map(f64::abs);
                            let lower = if moment_nm[0] <= 0.0 && moment_nm[1] >= 0.0 {
                                0.0
                            } else {
                                abs[0].min(abs[1])
                            };
                            let measured =
                                [lower, abs[0].max(abs[1])].map(|m| m / section.section_modulus());
                            checked_pair(measured)?;
                            finding.measurement = Some(measured);
                            finding.outcome = upper_interval(measured, *allowable_pa);
                            finding.reason = "расчёт напряжения изгиба по модулю сечения".into();
                            // Нагрузка сечения задана для всего окна статической проверки.
                            finding.event_ns = Some(check.window.start_ns);
                            finding.confirmed_hazard = finding.outcome == Outcome::Fail
                                && trace(std::slice::from_ref(limit), &set.derivations).0;
                        }
                        StrengthSection::NotAnalyzed { reason } => {
                            finding.missing_data.push(reason)
                        }
                    }
                }
                if !trace(std::slice::from_ref(limit), &set.derivations).0 {
                    sources.push(limit.clone());
                }
                vec![finding]
            }
            Verification::Truth(limit) => {
                let series = Series::from_log(&self.log, SeriesKind::Truth, limit.signal())?;
                compare_truth(&mut finding, limit, check.window, &series, 0.0, set)?;
                if key.class != HazardClass::TaskCriterion
                    && !trace(std::slice::from_ref(limit_source(limit)), &set.derivations).0
                {
                    sources.push(limit_source(limit).clone());
                }
                if let Some(log) = &self.divergent_repeat {
                    let mut repeated = Finding::open(key, "критерий повторного прогона");
                    let series = Series::from_log(log, SeriesKind::Truth, limit.signal())?;
                    compare_truth(&mut repeated, limit, check.window, &series, 0.0, set)?;
                    if repeated.outcome == Outcome::Fail {
                        repeated.journal = Some(Fingerprint::from_fnv1a(log.fingerprint()));
                        vec![finding, repeated]
                    } else {
                        vec![finding]
                    }
                } else {
                    vec![finding]
                }
            }
            Verification::Measurement(limit) => {
                if let Some(bench) = self.bench.iter().find(|m| &m.signal == limit.signal()) {
                    if let Some(uncertainty) = bench.uncertainty {
                        compare_truth(
                            &mut finding,
                            limit,
                            check.window,
                            &Series {
                                signal: bench.signal.clone(),
                                points: bench.samples.clone(),
                            },
                            uncertainty,
                            set,
                        )?;
                    } else {
                        finding.reason = "не задана граница погрешности стендового канала".into();
                        finding.coverage_gaps.push(finding.reason.clone());
                    }
                    finding.journal = Some(bench.journal);
                    sources.push(bench.source.clone());
                    sources.push(limit_source(limit).clone());
                }
                vec![finding]
            }
            Verification::Process { profile, criterion } => {
                let observations: Vec<_> = self
                    .process
                    .iter()
                    .filter(|(actual, _, _)| actual == profile)
                    .flat_map(|(_, _, findings)| findings)
                    .filter(|f| f.key.window_ns == key.window_ns)
                    .collect();
                let modeled = observations.iter().any(|f| {
                    f.key.class == key.class
                        && f.key.object == key.object
                        && f.outcome == Outcome::Pass
                });
                let mut results = Vec::new();
                for source in observations
                    .iter()
                    .filter(|f| f.key.class == HazardClass::TaskCriterion && f.code == *criterion)
                {
                    let mut metric = (*source).clone();
                    metric.group = key.class.spec().group;
                    let observed_failure = observations.iter().any(|f| {
                        f.key.class == key.class
                            && f.key.object == key.object
                            && f.outcome == Outcome::Fail
                            && f.event_ns.is_some()
                    });
                    metric.confirmed_hazard = metric.outcome == Outcome::Fail
                        && metric.measurement.is_some()
                        && metric.limit.is_some()
                        && (key.class.spec().method != super::Method::Process || observed_failure);
                    if !modeled
                        && key.class.spec().method == super::Method::Process
                        && metric.outcome == Outcome::Pass
                    {
                        metric
                            .coverage_gaps
                            .push("нет типизированного покрытия класса моделью".into());
                    }
                    results.push(metric);
                }
                if results.is_empty() {
                    vec![finding]
                } else {
                    results
                }
            }
            Verification::Interaction { profile, quantity } => {
                let matches: Vec<_> = self
                    .interaction
                    .iter()
                    .filter(|(actual, _, _)| actual == profile)
                    .flat_map(|(_, _, findings)| findings)
                    .filter(|f| {
                        f.code == *quantity
                            && f.key.class == key.class
                            && f.key.object == key.object
                            && f.key.window_ns == key.window_ns
                    })
                    .cloned()
                    .collect();
                if matches.is_empty() {
                    vec![finding]
                } else {
                    matches
                }
            }
            Verification::Control {
                criteria,
                check: kind,
            } => {
                if let Some((actual, board, report)) = &self.control
                    && actual == criteria
                    && (!requires_fault(key.class)
                        || report.case.fault.as_ref().is_some_and(|fault| {
                            fault.node == key.object && fault_class(fault.kind) == key.class
                        }))
                {
                    finding.coverage_gaps.extend(report.missing.iter().cloned());
                    match kind {
                        ControlCriterion::ProgramIdentity if *board && report.was_executed() => {
                            finding.reason = "образ и вывод подтверждены в живом контуре".into();
                            finding.outcome = if report.outcome == Outcome::Pass {
                                Outcome::Pass
                            } else {
                                Outcome::NotAnalyzed
                            };
                        }
                        ControlCriterion::Limit(signal) => {
                            let series = Series::from_log(&self.log, SeriesKind::Truth, signal)?;
                            let mut results = Vec::new();
                            for metric in report
                                .metrics
                                .iter()
                                .filter(|metric| &metric.signal == signal)
                            {
                                let mut item = finding.clone();
                                compare_truth(
                                    &mut item,
                                    &metric.limit,
                                    check.window,
                                    &series,
                                    0.0,
                                    set,
                                )?;
                                // Успех требует подтверждённой причинной цепочки.
                                if item.outcome == Outcome::Pass && report.outcome != Outcome::Pass
                                {
                                    item.outcome = Outcome::NotAnalyzed;
                                }
                                results.push(item);
                            }
                            if !results.is_empty() {
                                finding = results.remove(0);
                            }
                            if !results.is_empty() {
                                // Оценка каждого предела остаётся видна, включая противоречащие.
                                results.insert(0, finding);
                                return Ok(
                                    self.checked_findings(set, check, key, &sources, results)
                                );
                            }
                        }
                        ControlCriterion::Reaction(signal) => {
                            if let Some(reaction) = report
                                .reaction
                                .as_ref()
                                .filter(|r| &r.criterion.signal == signal)
                                && let Some(event) =
                                    reaction.applied_ns.checked_add(reaction.reaction_ns)
                                && (check.window.start_ns..=check.window.end_ns)
                                    .contains(&reaction.applied_ns)
                                && event <= check.window.end_ns
                            {
                                let (proved, names) = trace(
                                    std::slice::from_ref(&reaction.criterion.source),
                                    &set.derivations,
                                );
                                finding.measurement = Some([reaction.reaction_ns as f64; 2]);
                                finding.limit = Some(LimitValue::upper(
                                    [reaction.criterion.within_ns as f64; 2],
                                ));
                                finding.limit_sources = names;
                                finding.event_ns = Some(event);
                                finding.unit = Some("ns".into());
                                finding.reason =
                                    "измеренная задержка физической реакции на воздействие".into();
                                if reaction.reaction_ns > reaction.criterion.within_ns {
                                    finding.outcome = Outcome::Fail;
                                    finding.confirmed_hazard = proved;
                                } else if proved && report.outcome == Outcome::Pass {
                                    finding.outcome = Outcome::Pass;
                                }
                            }
                        }
                        _ => {}
                    }
                }
                vec![finding]
            }
        };
        Ok(self.checked_findings(set, check, key, &sources, details))
    }

    fn checked_findings(
        &self,
        set: &RequirementSet,
        check: &Check,
        key: &RequirementKey,
        sources: &[Source],
        mut details: Vec<Finding>,
    ) -> Vec<Finding> {
        let (proved, source_names) = trace(sources, &set.derivations);
        for item in &mut details {
            item.key = key.clone();
            if item.model_scope.is_none() {
                item.model_scope = Some(check.scope.clone());
            }
            item.measurement_sources
                .extend(source_names.iter().cloned());
            if !proved {
                item.coverage_gaps
                    .push("не подтверждены источник входов, предел или применимость метода".into());
                item.confirmed_hazard = false;
                if item.outcome != Outcome::Fail || key.class != HazardClass::TaskCriterion {
                    item.outcome = Outcome::NotAnalyzed;
                }
            }
            self.finish(set, item);
        }
        details
    }
}

fn upper_interval(measured: [f64; 2], limit: [f64; 2]) -> Outcome {
    if measured[0] > limit[1] {
        Outcome::Fail
    } else if measured[1] <= limit[0] {
        Outcome::Pass
    } else {
        Outcome::NotAnalyzed
    }
}

fn compare_truth(
    finding: &mut Finding,
    limit: &Limit,
    window: Window,
    series: &Series,
    uncertainty: f64,
    set: &RequirementSet,
) -> Result<(), Error> {
    let points: Vec<_> = series
        .points
        .iter()
        .filter(|p| p.valid && (window.start_ns..=window.end_ns).contains(&p.at_ns))
        .collect();
    let (proved, refs) = trace(std::slice::from_ref(limit_source(limit)), &set.derivations);
    finding.limit_sources = refs;
    finding.unit = Some(limit.signal().unit.clone());
    finding.limit = Some(match limit {
        Limit::Range { min, max, .. } => LimitValue {
            lower: min.map(|v| [v; 2]),
            upper: max.map(|v| [v; 2]),
        },
        // По формату событие равно 1, отсутствие события равно 0.
        Limit::Forbidden { .. } => LimitValue::upper([0.0; 2]),
    });
    if let Some(first) = points.first() {
        let [lo, hi] = points.iter().fold([first.value; 2], |[lo, hi], p| {
            [lo.min(p.value), hi.max(p.value)]
        });
        finding.measurement = Some(if uncertainty == 0.0 {
            [lo, hi]
        } else {
            [lo - uncertainty, hi + uncertainty]
        });
    }
    let violation = points.iter().find(|point| match limit {
        Limit::Range { min, max, .. } => {
            min.is_some_and(|limit| point.value + uncertainty < limit)
                || max.is_some_and(|limit| point.value - uncertainty > limit)
        }
        Limit::Forbidden { .. } => uncertainty == 0.0 && point.value == 1.0,
    });
    let invalid_events = matches!(limit, Limit::Forbidden { .. })
        && (uncertainty != 0.0 || points.iter().any(|p| p.value != 0.0 && p.value != 1.0));
    let possibly_exceeded = match (limit, finding.measurement) {
        (Limit::Range { min, max, .. }, Some([lo, hi])) => {
            min.is_some_and(|limit| lo < limit) || max.is_some_and(|limit| hi > limit)
        }
        _ => false,
    };
    let complete = series.slice(window).is_some_and(|p| {
        p.first().is_some_and(|p| p.at_ns == window.start_ns)
            && p.last().is_some_and(|p| p.at_ns == window.end_ns)
    }) && !invalid_events;
    if !complete {
        finding
            .coverage_gaps
            .push("сигнал, границы или шаг окна не подтверждены".into());
    }
    if !proved {
        finding
            .coverage_gaps
            .push("источник предела не подтверждён".into());
    }
    finding.outcome = if let Some(point) = violation {
        finding.event_ns = Some(point.at_ns);
        finding.reason = "наблюдалось нарушение критерия".into();
        finding.confirmed_hazard = proved && finding.key.class != HazardClass::TaskCriterion;
        Outcome::Fail
    } else if possibly_exceeded {
        finding.reason = "граница погрешности пересекает предел".into();
        finding.coverage_gaps.push(finding.reason.clone());
        Outcome::NotAnalyzed
    } else if complete && proved {
        finding.reason = "в полном окне соблюдён критерий".into();
        Outcome::Pass
    } else {
        Outcome::NotAnalyzed
    };
    Ok(())
}
