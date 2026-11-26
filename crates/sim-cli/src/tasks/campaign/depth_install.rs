//! Установщик покрытия. Координатор оплачивает отдельный процесс на строку;
//! дочерняя команда повторно разрешает исходные файлы и проверяет адрес опыта.

use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    path::{Path, PathBuf},
    time::Duration,
};

use serde::Deserialize;
use serde_json::json;
use sim_runtime::{Runtime, control};
use sim_signal::{Command, Time};
use sim_task::{
    Fingerprint, Outcome,
    campaign::{
        Campaign, Disposition, PlanRow, ReferenceKind, ResolvedInputs, RunKey, SampledInputs,
        Target,
        depth_factors::{DepthFactors, DepthPlan, DepthRow, Selection},
    },
    metrics::{self, CampaignStats},
    verdict::Readiness,
};

use super::super::{Checkpoint, RowReport, RunStatus, bench, encode, playback, value};
use super::{ResultRows, Spec, linked_row, setup};
use crate::{
    Options,
    commands::{self, CoverageRow, Recording},
    electrical::{LiveExperiment, LivePath, report},
    tasks::{
        Models, processes,
        workers::depth_budget::{BudgetError, DepthBudget, DepthLimits},
    },
    view::PlaybackFormat,
};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Limits {
    runs: u32,
    seconds: u64,
}

type PathKey = (String, String, String);
type Profiles = BTreeMap<PathKey, PathBuf>;

pub(super) fn profiles(file: &Path, spec: &Spec) -> Result<Profiles, String> {
    let mut profiles = BTreeMap::new();
    for path in &spec.paths {
        let Some(profile) = &path.electrical_profile else {
            continue;
        };
        if spec.coverage.is_none() {
            return Err("electrical_profile в пути требует режим [coverage]".into());
        }
        let key = (path.station.clone(), path.mode.clone(), path.path.clone());
        if profiles
            .insert(key, processes::dir(file).join(profile))
            .is_some()
        {
            return Err("два электрических профиля у одного пути".into());
        }
    }
    Ok(profiles)
}

pub(super) struct Installation<'a> {
    pub prefix: &'a str,
    pub path: &'a Path,
    pub installation: &'a Path,
    pub transport: Option<&'a Path>,
    pub options: &'a Options,
    pub bench: &'a [bench::Bench],
    pub spec: &'a Spec,
    pub files: processes::Inputs,
    pub campaign: &'a Campaign,
    pub inputs: &'a ResolvedInputs,
    pub models: Option<&'a Models>,
    pub electrical_profiles: Profiles,
}

fn result(rows: BTreeMap<String, RowReport>, resumed: usize, view: Option<Vec<u8>>) -> ResultRows {
    let readiness = if rows.values().any(|row| row.outcome == Outcome::Fail) {
        Readiness::Rejected
    } else {
        Readiness::NeedsWork
    };
    ResultRows {
        rows,
        readiness,
        resumed,
        view,
        statistics: CampaignStats {
            rows: Vec::new(),
            series: Vec::new(),
        },
        metric_worst: BTreeSet::new(),
    }
}

fn open(input: Fingerprint, reason: impl serde::Serialize) -> Result<RowReport, String> {
    Ok(RowReport {
        status: RunStatus::Open,
        detail: value(reason)?,
        ..RowReport::omitted(input)
    })
}

fn annotate(
    row: &mut RowReport,
    cover: &Selection,
    key: &RunKey,
    input: Fingerprint,
    order: Option<usize>,
) {
    row.detail = json!({
        "coverage": { "selection": cover, "key": key, "input": input, "order": order },
        "scenario": std::mem::take(&mut row.detail),
    });
}

fn stamp(stamp: Fingerprint) -> String {
    format!("{:016x}", stamp.as_u64())
}

pub(super) fn run(
    mut driver: Installation<'_>,
    child: Option<&CoverageRow>,
) -> Result<ResultRows, String> {
    let spec = driver.spec;
    let limits = spec
        .coverage
        .as_ref()
        .ok_or("для depth_factors требуется [coverage] в установке")?;
    let factors = driver
        .campaign
        .depth_factors()
        .ok_or("для [coverage] требуется depth_factors в кампании")?;
    let limits = DepthLimits::new(limits.runs, Duration::from_secs(limits.seconds))
        .map_err(|e| e.to_string())?;
    if spec.electrical.is_some() {
        return Err("в режиме покрытия электрический профиль задаётся у конкретного пути".into());
    }
    if driver
        .bench
        .iter()
        .any(|entry| entry.experiment == driver.prefix)
    {
        return Err("заверенные ряды требуют привязки к плану покрытия".into());
    }

    let source_manifest = child
        .map(|request| request.manifest.as_path())
        .or(driver.options.campaign.as_deref())
        .ok_or("отсутствует манифест установки")?;
    let source_manifest = source_manifest.canonicalize().map_err(|e| e.to_string())?;
    let manifest: toml::Value = driver.files.toml(&source_manifest)?;
    let (assembly, experiment) = driver
        .prefix
        .split_once('/')
        .ok_or("неверный адрес установки")?;
    let assembly = manifest
        .get("assemblies")
        .and_then(|items| items.get(assembly))
        .ok_or("в исходном манифесте нет сборки")?;
    let entries = assembly
        .get("experiments")
        .and_then(toml::Value::as_table)
        .ok_or("нет опытов сборки")?;
    if entries.len() != 1 || assembly.get("sampling").is_some() || assembly.get("seed").is_some() {
        return Err(
            "сборке с покрытием нужен один сценарий; разброс и зерно задаются в его кампании"
                .into(),
        );
    }
    let entry = entries.get(experiment).ok_or("нет сценария установки")?;
    for (field, expected) in [
        ("campaign", driver.path),
        ("installation", driver.installation),
    ] {
        let declared = entry
            .get(field)
            .and_then(toml::Value::as_str)
            .ok_or("неполная ссылка сценария")?;
        if processes::dir(&source_manifest)
            .join(declared)
            .canonicalize()
            .map_err(|e| e.to_string())?
            != expected.canonicalize().map_err(|e| e.to_string())?
        {
            return Err("дочерняя ссылка расходится с исходным манифестом".into());
        }
    }
    let exe = commands::coverage_binary()?;
    let executable = Fingerprint::of(driver.files.capture(&exe)?);
    let engine = Runtime::physics_engine_identity()?;
    let mut electrical = BTreeMap::new();
    for (key, profile) in &driver.electrical_profiles {
        let install = report::Installation::load(
            driver.campaign.world_path(),
            driver.campaign.catalog_path(),
            Some(profile),
        )?;
        electrical.insert(key.clone(), install);
    }
    let electrical_files: Vec<_> = electrical
        .iter()
        .map(|(key, install)| (key, install.fingerprints()))
        .collect();
    // Версия протокола покрытия отделяет точки старой полной матрицы.
    let identity = encode((
        "coverage/v1",
        driver.prefix,
        executable,
        &engine,
        driver.files.fingerprints(),
        &electrical_files,
        driver
            .inputs
            .references
            .values()
            .map(|entry| entry.graph_fingerprint)
            .collect::<Vec<_>>(),
    ))?;
    let plan_id = stamp(Fingerprint::of(&identity));
    let budget_dir = driver.options.checkpoint.as_ref().map(|path| {
        path.join(format!(
            "coverage-v1-{}",
            stamp(Fingerprint::of(driver.prefix.as_bytes()))
        ))
    });
    let budget = if child.is_none() {
        let budget = match budget_dir.as_deref() {
            Some(dir)
                if dir
                    .join("depth-budget-v1.json")
                    .try_exists()
                    .map_err(|e| e.to_string())? =>
            {
                DepthBudget::resume(limits, &plan_id, dir)
            }
            dir => DepthBudget::start(limits, &plan_id, dir),
        }
        .map_err(|e| e.to_string())?;
        Some(budget)
    } else {
        None
    };

    let plan = driver
        .campaign
        .prepare_depth(driver.inputs)
        .map_err(|e| e.to_string())?;
    let build = Fingerprint::of(&encode((&identity, plan.fingerprint()))?);
    let sampling = super::super::installation::coverage_sampling(driver.campaign.sampling_axes())?;
    let work = Work {
        driver,
        plan,
        sampling,
        build,
        electrical,
    };

    if let Some(request) = child {
        let row = work.resolve(&request.selection)?;
        let (id, input) = work.address(&row)?;
        if request.input != stamp(input) {
            return Err("исходные файлы дочернего опыта изменились".into());
        }
        let DepthRow::Bound(row) = row else {
            return Err("дочерняя команда требует разрешённой строки".into());
        };
        if !matches!(row.disposition, Disposition::Ready) {
            return Err("дочерняя команда требует готовой строки".into());
        }
        let archive = request.recording.map(|format| match format {
            Recording::Log => PlaybackFormat::Log,
            Recording::View => PlaybackFormat::View,
        });
        let mode = work
            .plan
            .materialize(&request.selection)
            .map_err(|e| e.to_string())?;
        let (mut report, recording) = work.execute(&row, &mode, input, &id, archive)?;
        work.verify(&engine)?;
        if let Some(recording) = recording {
            use std::io::Write;
            let directory = work
                .driver
                .transport
                .ok_or("нет каталога дочерней команды")?;
            fs::File::create_new(directory.join("record"))
                .and_then(|mut file| file.write_all(&recording))
                .map_err(|e| e.to_string())?;
        }
        annotate(&mut report, &request.selection, &row.key, input, None);
        return Ok(result(BTreeMap::from([(id, report)]), 0, None));
    }

    let budget = budget.expect("координатор имеет бюджет");
    let planned = cover(factors)?;
    let workers = work
        .driver
        .options
        .workers
        .as_deref()
        .ok_or("не заданы исполнители покрытия")?;
    let checkpoint = budget_dir
        .as_ref()
        .map(|path| Checkpoint::with_identity(path, &identity))
        .transpose()?;
    let scratch = super::super::installation::Scratch::new()?;
    let format = PlaybackFormat::for_options(work.driver.options);
    let mut jobs = Vec::new();
    let mut rows = BTreeMap::new();
    let mut resumed = 0;
    let mut addresses = BTreeSet::new();
    let missing = planned.missing;
    for (index, selection) in planned.rows.into_iter().enumerate() {
        let row = work.resolve(&selection)?;
        let (id, input) = work.address(&row)?;
        if !addresses.insert(id.clone()) {
            return Err("коллизия ключа строки покрытия".into());
        }
        if work.driver.options.repeat.as_ref().is_some_and(|wanted| {
            wanted != &id
                && !wanted
                    .strip_prefix(&id)
                    .is_some_and(|suffix| suffix.starts_with('@'))
        }) {
            continue;
        }
        let name = stamp(input);
        let key = row.key().clone();
        let phase = match row {
            DepthRow::Excluded { .. } => {
                return Err("планировщик выбрал физически исключённую строку".into());
            }
            DepthRow::NotAnalyzed { reasons, .. } => Err(value(reasons)?),
            DepthRow::Bound(row) => match row.disposition {
                Disposition::Ready => Ok(row),
                _ => Err(value(row.disposition)?),
            },
        };
        let row = match phase {
            Ok(row) => row,
            Err(reason) => {
                let mut row = open(input, reason)?;
                annotate(&mut row, &selection, &key, input, Some(index));
                rows.insert(id, row);
                continue;
            }
        };
        let sampled = work
            .plan
            .materialize(&selection)
            .map_err(|e| e.to_string())?;
        let reason = work.unsupported(&row, &sampled, format)?;
        if let Some(reason) = reason {
            let mut row = open(input, reason)?;
            annotate(&mut row, &selection, &key, input, Some(index));
            rows.insert(id, row);
            continue;
        }

        let cache = checkpoint.as_ref().filter(|_| {
            work.driver.options.repeat.is_none()
                && work.electrical_profile(&key).is_none()
                && sampled
                    .experiment
                    .as_ref()
                    .is_some_and(|experiment| experiment.kind.is_none())
        });
        if let Some(cache) = cache
            && let Some(saved) = cache.load_row::<RowReport>(&name)?
        {
            validate_reply(&saved, input, &selection, &key)?;
            if saved.executed != 1 || saved.status != RunStatus::Completed {
                return Err("контрольная точка содержит незавершённую строку покрытия".into());
            }
            rows.insert(id, saved);
            resumed += 1;
            continue;
        }
        let dir = scratch.path().join(index.to_string());
        fs::create_dir(&dir).map_err(|e| e.to_string())?;
        jobs.push((id, input, name, key, selection, index, dir, cache));
    }
    let mut view = None;
    let campaign_path = work.driver.path.canonicalize().map_err(|e| e.to_string())?;
    let installation_path = work
        .driver
        .installation
        .canonicalize()
        .map_err(|e| e.to_string())?;
    let recording = format.map(|format| match format {
        PlaybackFormat::Log => Recording::Log,
        PlaybackFormat::View => Recording::View,
    });
    // Дескрипторы вывода открываются только на порцию по числу доступных ядер.
    for jobs in jobs.chunks(crate::tasks::workers::Workers::cpu_count().get()) {
        let commands = jobs
            .iter()
            .map(|(_, _, name, _, selection, _, dir, _)| {
                let request = CoverageRow {
                    installation: installation_path.clone(),
                    manifest: source_manifest.clone(),
                    input: name.clone(),
                    selection: selection.clone(),
                    recording,
                };
                commands::coverage_command(&exe, dir, work.driver.prefix, &campaign_path, &request)
            })
            .collect::<Result<Vec<_>, _>>()?;
        let replies = workers.map_budgeted(&budget, commands, None)?;
        for ((id, input, name, key, selection, index, dir, cache), status) in
            jobs.iter().zip(replies)
        {
            let mut row = match status {
                Ok(status) => {
                    let bytes = fs::read(dir.join("report.json")).map_err(|e| e.to_string())?;
                    let mut response: serde_json::Value = serde_json::from_slice(&bytes)
                        .map_err(|e| format!("дочерний опыт «{id}» ({status}): {e}"))?;
                    if !status.success() {
                        return Err(format!("дочерний опыт «{id}»: {}", response["detail"]));
                    }
                    let (assembly, _) = work
                        .driver
                        .prefix
                        .split_once('/')
                        .expect("адрес установки проверен");
                    let row: RowReport =
                        serde_json::from_value(response["assemblies"][assembly]["runs"][id].take())
                            .map_err(|e| e.to_string())?;
                    validate_reply(&row, *input, selection, key)?;
                    if recording.is_some() && row.playback.is_some() {
                        view = Some(fs::read(dir.join("record")).map_err(|e| e.to_string())?);
                    }
                    row
                }
                Err(
                    error @ (BudgetError::QuotaExhausted
                    | BudgetError::DeadlineExceeded
                    | BudgetError::Cancelled),
                ) => open(*input, error.to_string())?,
                Err(error) => return Err(error.to_string()),
            };
            // У дочернего процесса нет порядкового номера координатора.
            if row.detail.get("coverage").is_some() {
                row.detail["coverage"]["order"] = json!(index);
            } else {
                annotate(&mut row, selection, key, *input, Some(*index));
            }
            if let Some(cache) =
                cache.filter(|_| row.executed == 1 && row.status == RunStatus::Completed)
            {
                cache.save_row(name, &row)?;
            }
            let actual_id = if row.input != *input {
                format!("{id}@{}", stamp(row.input))
            } else if row.status == RunStatus::Open {
                work.driver.options.repeat.as_ref().unwrap_or(id).clone()
            } else {
                id.clone()
            };
            if rows.insert(actual_id, row).is_some() {
                return Err("коллизия ключа строки покрытия".into());
            }
        }
    }
    work.verify(&engine)?;
    if !spec.checks.is_empty() {
        for row in rows.values_mut() {
            row.detail["coverage"]["installation_checks"] =
                json!("для проверок установки нужна отдельная привязка к выполненным строкам");
        }
    }
    if work.driver.options.repeat.is_none() && missing != 0 {
        rows.insert(
            format!("{}/depth-unplanned", work.driver.prefix),
            open(
                build,
                json!({ "unplanned_interactions": missing,
                "reason": "число строк покрытия достигло предела из блока Е" }),
            )?,
        );
    }
    Ok(result(rows, resumed, view))
}

fn validate_reply(
    row: &RowReport,
    input: Fingerprint,
    selection: &Selection,
    key: &RunKey,
) -> Result<(), String> {
    let audit = &row.detail["coverage"];
    if audit["input"] != value(input)?
        || audit["selection"] != value(selection)?
        || audit["key"] != value(key)?
        || row.required != 1
        || row.executed > 1
        || row.outcome == Outcome::Pass
        || (row.executed == 1) != (row.status == RunStatus::Completed)
        || row
            .playback
            .as_ref()
            .is_some_and(|playback| playback.child != *key)
    {
        return Err("ответ не соответствует запрошенной строке покрытия".into());
    }
    Ok(())
}

struct Work<'a> {
    driver: Installation<'a>,
    plan: DepthPlan<'a>,
    sampling: sim_task::sampling::Sampling,
    build: Fingerprint,
    electrical: BTreeMap<PathKey, report::Installation>,
}

impl Work<'_> {
    fn resolve(&self, selection: &Selection) -> Result<DepthRow, String> {
        self.plan.row(selection).map_err(|e| e.to_string())
    }

    fn address(&self, row: &DepthRow) -> Result<(String, Fingerprint), String> {
        let input = Fingerprint::of(&encode((self.build, row.key()))?);
        Ok((
            format!("{}/depth-{}", self.driver.prefix, stamp(input)),
            input,
        ))
    }

    fn electrical_profile(&self, key: &RunKey) -> Option<&Path> {
        self.driver
            .electrical_profiles
            .get(&(key.station.clone(), key.mode.clone(), key.path.clone()))
            .map(PathBuf::as_path)
    }

    fn verify(&self, engine: &(String, u64)) -> Result<(), String> {
        self.driver.files.verify()?;
        for install in self.electrical.values() {
            install.verify()?;
        }
        if Runtime::physics_engine_identity()? != *engine {
            return Err("образ движка изменился во время покрытия".into());
        }
        Ok(())
    }

    fn unsupported(
        &self,
        row: &PlanRow,
        sampled: &SampledInputs,
        archive: Option<PlaybackFormat>,
    ) -> Result<Option<&'static str>, String> {
        let mode = sampled.mode.as_ref().ok_or("нет режима строки покрытия")?;
        let linked = sampled.experiment.as_ref().and_then(|e| e.kind);
        let reason = if self.driver.spec.snapshot_ns.is_some() {
            Some("аудит снимка требует отдельного бюджетируемого повтора")
        } else if self.electrical_profile(&row.key).is_some() {
            (linked.is_some() || !mode.steps.is_empty())
                .then_some("электрический путь использует расписание своего профиля")
        } else if let Some(kind) = linked {
            match kind {
                ReferenceKind::ControlTask if archive.is_none() => {
                    let task = sampled
                        .experiment
                        .as_ref()
                        .and_then(|e| e.task.as_ref())
                        .ok_or("отсутствует файл задания связанного опыта")?;
                    let text =
                        fs::read_to_string(task).map_err(|e| format!("{}: {e}", task.display()))?;
                    let task = sim_task::Task::parse(&text).map_err(|e| e.to_string())?;
                    let paired = task
                        .modes()
                        .iter()
                        .filter(|m| m.id == row.key.mode)
                        .any(|m| {
                            m.stimuli
                                .iter()
                                .any(|s| s.id == m.excitation && s.comparison_value.is_some())
                        });
                    paired.then_some(
                        "сравнительная ветвь связанного опыта требует отдельного резерва",
                    )
                }
                ReferenceKind::ControlTask => {
                    Some("запись связанного опыта требует однопроходного захвата")
                }
                _ => Some(
                    "связанный профиль запускает несколько проб; нужен установщик одного варианта на одной сетке",
                ),
            }
        } else {
            (!row.faults.is_empty()
                || !mode
                    .steps
                    .iter()
                    .all(|step| matches!(step.target, Target::Effort { .. })))
            .then_some("встроенному опыту нужен исполнитель инъекции или модельного порта")
        };
        Ok(reason)
    }

    fn execute(
        &self,
        row: &PlanRow,
        sampled: &SampledInputs,
        input: Fingerprint,
        id: &str,
        format: Option<PlaybackFormat>,
    ) -> Result<(RowReport, Option<Vec<u8>>), String> {
        if let Some(reason) = self.unsupported(row, sampled, format)? {
            return Ok((open(input, reason)?, None));
        }
        let mode = sampled.mode.as_ref().ok_or("нет режима строки покрытия")?;
        if let Some(profile) = self.electrical_profile(&row.key) {
            return self.execute_electrical(row, sampled, input, mode, profile, format);
        }
        if sampled
            .experiment
            .as_ref()
            .is_some_and(|e| e.kind.is_some())
        {
            let (mut report, _) = linked_row(
                self.driver.campaign,
                sampled,
                row,
                &self.sampling,
                input,
                id,
                None,
                None,
                self.driver.options,
            )?;
            report.outcome = report.outcome.max(Outcome::NotAnalyzed);
            return Ok((report, None));
        }
        let compiled = sampled.world.compile().map_err(|e| e.to_string())?;
        let mut runtime = Runtime::load(compiled.clone())?;
        setup(&mut runtime, &compiled, self.driver.models)?;
        let receipt =
            super::super::installation::coverage_receipt(&mut runtime, sampled, &compiled)?;
        if format == Some(PlaybackFormat::View) {
            runtime.record_view()?;
        }
        let until = Time(mode.horizon_ns);
        for step in &mode.steps {
            if step.at_ns > runtime.time().0
                && runtime.run_until(Time(step.at_ns))? != sim_runtime::Status::Running
            {
                break;
            }
            let Target::Effort { name } = &step.target else {
                unreachable!("проверен тип шага");
            };
            let channel = runtime
                .channel(name)
                .ok_or_else(|| format!("нет входа усилия «{name}»"))?;
            runtime.act(&Command {
                targets: vec![(channel, step.value)],
            })?;
        }
        runtime.run_until(until)?;
        let summary = metrics::evaluate_scenario(row, sampled, runtime.log(), receipt, &[], &[])
            .map_err(|e| e.to_string())?;
        let complete = summary.completed
            && sampled
                .settings
                .channel_delay_ns
                .keys()
                .all(|name| runtime.channel_delay_observed(name));
        let report = live_report(
            row,
            input,
            &runtime,
            complete,
            summary.outcome,
            json!({ "summary": summary }),
        )?;
        let archive = collect(format, &report, &runtime, &row.key.mode)?;
        Ok((report, archive))
    }

    fn execute_electrical(
        &self,
        row: &PlanRow,
        sampled: &SampledInputs,
        input: Fingerprint,
        mode: &sim_task::campaign::OperatingMode,
        profile: &Path,
        format: Option<PlaybackFormat>,
    ) -> Result<(RowReport, Option<Vec<u8>>), String> {
        let directory = self
            .driver
            .transport
            .ok_or("нет каталога дочерней команды")?;
        let scratch = directory.join("world");
        fs::create_dir(&scratch).map_err(|e| e.to_string())?;
        let expected = sampled.world.compile().map_err(|e| e.to_string())?;
        let path = super::super::installation::coverage_world(
            &sampled.world,
            self.driver.inputs.world.world.seed,
            &scratch,
        )?;
        let mut actual_world = sim_world::load_and_compile_with_schematics(&path)?.world;
        actual_world.seed = sampled.sample.seeds.run;
        if actual_world != expected {
            return Err("временный электрический мир расходится с разрешённой установкой".into());
        }
        let until = Time(mode.horizon_ns);
        let catalog = self.driver.campaign.catalog_path();
        let mut installation =
            report::Installation::load(self.driver.campaign.world_path(), catalog, Some(profile))?;
        let mut run = LiveExperiment::load(
            &path,
            profile,
            catalog,
            Some(sampled.sample.seeds.run),
            until,
            LivePath::Full,
        )?;
        installation.bind_live(&run.runtime, until)?;
        run.runtime.record_joint_state("запись состояния")?;
        let receipt =
            super::super::installation::coverage_receipt(&mut run.runtime, sampled, &expected)?;
        if format == Some(PlaybackFormat::View) {
            run.runtime.record_view()?;
        }
        run.run_until(until)?;
        let actual = control::run(&run.runtime).ok_or("не установлен электрический контур")?;
        let injections = electrical_injections(row, actual.events());
        let complete = actual.reached_horizon()
            && actual.end().steps > 0
            && injections
            && sampled
                .settings
                .channel_delay_ns
                .keys()
                .all(|name| run.runtime.channel_delay_observed(name));
        let evaluated = installation.evaluate(self.build, Some(&run.runtime), Some(until))?;
        let scenario = row
            .faults
            .is_empty()
            .then(|| {
                metrics::evaluate_scenario(
                    row,
                    sampled,
                    run.runtime.log(),
                    receipt.clone(),
                    &[],
                    &[],
                )
            })
            .transpose()
            .map_err(|e| e.to_string())?;
        let outcome = evaluated.report.outcome().max(
            scenario
                .as_ref()
                .map_or(Outcome::NotAnalyzed, |result| result.outcome),
        );
        let mut report = live_report(
            row,
            input,
            &run.runtime,
            complete,
            outcome,
            json!({
                "electrical": value(&evaluated.report)?, "installation": receipt, "summary": scenario,
                "injections_confirmed": injections,
                "reason": (!injections).then_some("нет совпадающего запроса и применения электрической инъекции"),
            }),
        )?;
        let identity = actual.identity();
        report.input = Fingerprint::of(&encode((
            input,
            identity.world,
            identity.catalog,
            identity.control,
            &identity.assets,
            sampled.fingerprint,
            until.0,
        ))?);
        report.findings = report.detail["electrical"]["findings"]
            .as_array()
            .cloned()
            .unwrap_or_default();
        let archive = collect(format, &report, &run.runtime, &row.key.mode)?;
        Ok((report, archive))
    }
}

fn electrical_injections(row: &PlanRow, events: &[control::Event]) -> bool {
    use control::{EventKind, FaultAction};
    use sim_task::FaultKind;

    let mut matched = BTreeSet::new();
    for fault in &row.faults {
        let expected_action = match fault.schedule.fault.kind {
            FaultKind::SensorUnavailable
            | FaultKind::WireOpen
            | FaultKind::PacketLoss
            | FaultKind::DriverDisabled
            | FaultKind::DriveDisabled => Some(FaultAction::Drop),
            FaultKind::PowerCut => Some(FaultAction::PowerOff),
            FaultKind::ControllerHalted => Some(FaultAction::Pause),
            FaultKind::PacketDelay => fault.schedule.extra_delay_ns.map(FaultAction::Delay),
        };
        let mut requests = events.iter().filter_map(|event| match &event.kind {
            EventKind::FaultRequested {
                id,
                target,
                action,
                active: true,
            } if *target == fault.channel
                && Some(*action) == expected_action
                && event.at.0 == fault.schedule.at_ns =>
            {
                Some(id)
            }
            _ => None,
        });
        let Some(request) = requests.next() else {
            return false;
        };
        if requests.next().is_some() || !matched.insert(request) {
            return false;
        }
        if !events.iter().any(|event| {
            matches!(&event.kind, EventKind::FaultApplied { id, target, active: true }
                if id == request && *target == fault.channel)
                && event
                    .at
                    .0
                    .checked_sub(fault.schedule.at_ns)
                    .is_some_and(|lag| lag <= fault.schedule.max_lag_ns)
                && (fault.window.start_ns..fault.window.end_ns).contains(&event.at.0)
        }) {
            return false;
        }
    }
    // Чужая активная инъекция меняет измеренный опыт и не закрывает выбранную строку.
    events.iter().all(|event| match &event.kind {
        EventKind::FaultRequested {
            id, active: true, ..
        } => matched.contains(id),
        EventKind::FaultRequested {
            id, active: false, ..
        } => !matched.contains(id),
        _ => true,
    })
}

fn live_report(
    row: &PlanRow,
    input: Fingerprint,
    runtime: &Runtime,
    complete: bool,
    outcome: Outcome,
    mut detail: serde_json::Value,
) -> Result<RowReport, String> {
    let log = runtime.log();
    if log.header().seed != runtime.seed()
        || runtime.physics_rate().as_hz() as u64 != row.grid.hz
        || log.entries().iter().any(|entry| !entry.value.is_finite())
    {
        return Err("журнал содержит чужую сетку, зерно или неконечное значение".into());
    }
    let journal = log.fingerprint();
    detail["journal"] = json!(journal);
    detail["steps"] = json!(runtime.steps());
    let playback = complete
        .then(|| {
            playback::select(
                &row.key,
                runtime.seed(),
                [playback::Grid {
                    variant: None,
                    hz: row.grid.hz,
                    journal,
                    end_ns: runtime.time().0,
                }],
                &[],
            )
        })
        .flatten();
    Ok(RowReport {
        input,
        required: 1,
        executed: usize::from(complete),
        status: if complete {
            RunStatus::Completed
        } else {
            RunStatus::Incomplete
        },
        outcome: outcome.max(Outcome::NotAnalyzed),
        findings: Vec::new(),
        playback,
        detail,
    })
}

fn collect(
    format: Option<PlaybackFormat>,
    row: &RowReport,
    runtime: &Runtime,
    mode: &str,
) -> Result<Option<Vec<u8>>, String> {
    format
        .filter(|_| row.playback.is_some())
        .map(|format| format.collect(runtime, mode, None))
        .transpose()
}

struct Cover {
    rows: Vec<Selection>,
    missing: usize,
}

type Interaction<'a> = Vec<(&'a str, &'a str)>;

fn extend(
    factors: &DepthFactors,
    levels: &[(&str, Vec<&str>)],
    selection: &mut Selection,
    at: usize,
) -> Result<bool, String> {
    if factors
        .exclusion(selection)
        .map_err(|e| e.to_string())?
        .is_some()
    {
        return Ok(false);
    }
    let Some((id, levels_at)) = levels.get(at) else {
        return Ok(true);
    };
    if selection.contains_key(*id) {
        return extend(factors, levels, selection, at + 1);
    }
    for level in levels_at {
        selection.insert((*id).into(), (*level).into());
        if extend(factors, levels, selection, at + 1)? {
            return Ok(true);
        }
    }
    selection.remove(*id);
    Ok(false)
}

fn cover(factors: &DepthFactors) -> Result<Cover, String> {
    // Блок Е. Отбор перебирает только пары и тройки, завершает частичную
    // строку с отсечением и останавливается на первом допустимом продолжении.
    const MAX_ROWS: usize = 1000;
    let levels: Vec<_> = factors
        .factors()
        .iter()
        .map(|(id, factor)| (id.as_str(), factor.levels.level_ids().collect()))
        .collect();
    let mut tuples: BTreeSet<Interaction<'_>> = BTreeSet::new();
    for (i, (a, aa)) in levels.iter().enumerate() {
        for (j, (b, bb)) in levels.iter().enumerate().skip(i + 1) {
            for &x in aa {
                for &y in bb {
                    tuples.insert(vec![(a, x), (b, y)]);
                    if factors.factors()[*a].safety && factors.factors()[*b].safety {
                        for (c, cc) in levels
                            .iter()
                            .skip(j + 1)
                            .filter(|(id, _)| factors.factors()[*id].safety)
                        {
                            for &z in cc {
                                tuples.insert(vec![(a, x), (b, y), (c, z)]);
                            }
                        }
                    }
                }
            }
        }
    }
    let mut tuples: Vec<_> = tuples.into_iter().collect();
    tuples.sort_by(|a, b| a.len().cmp(&b.len()).then_with(|| a.cmp(b)));
    let mut rows: Vec<Selection> = Vec::new();
    let mut missing = 0;
    for tuple in tuples {
        if rows
            .iter()
            .any(|row| tuple.iter().all(|(id, level)| row[*id] == *level))
        {
            continue;
        }
        let mut row: Selection = tuple
            .into_iter()
            .map(|(id, level)| (id.into(), level.into()))
            .collect();
        if extend(factors, &levels, &mut row, 0)? {
            if rows.len() < MAX_ROWS {
                rows.push(row);
            } else {
                missing += 1;
            }
        }
    }
    if rows.is_empty() {
        return Err("план покрытия не содержит допустимых сочетаний".into());
    }
    Ok(Cover { rows, missing })
}
