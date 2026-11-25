//! Разрешение установки для общего сценария и отчёт оценщика.

use crate::view::PlaybackFormat;
use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use serde::{Deserialize, Serialize};
use sim_runtime::Runtime;
use sim_signal::Time;
use sim_task::campaign::{
    Binding, Campaign, Disposition, FaultSupport, Grid, PlanRow, ReferenceKind, RequiredPath, RunKey,
    ResolvedInputs, ResolvedReference, SampledInputs, Target, Window, WorldInputs,
};
use sim_task::metrics::{
    CampaignStats, Installation, LinkedMetric, MetricValue, Operation, RunSummary, summarize,
    summarize_linked,
};
use sim_task::sampling::{AppliedValue, ModelSettings, Parameter, SampleKey};
use sim_task::verdict::{
    CampaignEvidence, Check, Component, ComponentKind, ContactDuration, Coverage, Domain, Installed, REGISTRY,
    Readiness, ResolvedInventory, RunSelector, SectionLoad, SimulationVerdict, Verification, assess_campaign,
    derive_requirements_with_electrical,
};
use sim_task::{Case, Fault, Fingerprint, Limit, Outcome, Signal, Source};
use sim_world::{CompiledWorld, SectionAxis};

use super::{
    Checkpoint, Child, LinkedSample, RowReport, RunStatus, encode, linked, selected, value,
};
use crate::{Options, tasks::processes};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Spec {
    version: u32,
    paths: Vec<PathSpec>,
    snapshot_ns: Option<u64>,
    electrical: Option<ElectricalExperiment>,
    measurement_build: Option<sim_reality::BuildId>,
    #[serde(default)]
    checks: Vec<CheckSpec>,
    #[serde(default)]
    inventory: InventorySpec,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ElectricalExperiment {
    profile: std::path::PathBuf,
    horizon_ns: u64,
}

#[derive(Default, Deserialize)]
#[serde(deny_unknown_fields)]
struct InventorySpec {
    off_participant: Option<Source>,
    #[serde(default)]
    coverage: BTreeMap<Domain, CoverageSpec>,
    #[serde(default)]
    components: Vec<ComponentSpec>,
    #[serde(default)]
    derivations: BTreeMap<String, Vec<Source>>,
    #[serde(default)]
    unresolved_ranges: Vec<String>,
}

#[derive(Deserialize)]
#[serde(tag = "state", rename_all = "snake_case", deny_unknown_fields)]
enum CoverageSpec {
    Resolved { source: Source },
    Unknown { reason: String },
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ComponentSpec {
    object: String,
    kind: ComponentKind,
    source: Source,
}

impl InventorySpec {
    fn resolve(self, build: Fingerprint, checks: Vec<Check>) -> ResolvedInventory {
        let mut coverage: BTreeMap<_, _> = self
            .coverage
            .into_iter()
            .map(|(domain, spec)| {
                (
                    domain,
                    match spec {
                        CoverageSpec::Resolved { source } => Coverage::Resolved(source),
                        CoverageSpec::Unknown { reason } => Coverage::Unknown(reason),
                    },
                )
            })
            .collect();
        let participant = self.off_participant.map(|source| {
            coverage.insert(Domain::Participant, Coverage::Resolved(source));
            sim_task::interaction::Reach::OffParticipant
        });
        ResolvedInventory {
            build: Some(build),
            checks,
            participant,
            coverage,
            components: self
                .components
                .into_iter()
                .map(|item| Component {
                    object: item.object,
                    kind: item.kind,
                    source: item.source,
                })
                .collect(),
            derivations: self.derivations,
            unresolved_ranges: self.unresolved_ranges,
            ..Default::default()
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PathSpec {
    station: String,
    mode: String,
    path: String,
    source: Source,
    grids: Vec<Grid>,
    #[serde(default)]
    faults: Vec<Fault>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CheckSpec {
    class: String,
    object: String,
    station: Option<String>,
    mode: Option<String>,
    path: Option<String>,
    window: Window,
    applicability: Source,
    scope: String,
    inputs: Vec<Source>,
    verification: CheckKind,
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum CheckKind {
    Measurement {
        limit: Limit,
        contact_duration: Option<ContactDuration>,
        energy_path: Option<sim_task::interaction::EnergyPath>,
    },
    Truth {
        limit: Limit,
    },
    Section {
        geometry: usize,
        axis: Axis,
        moment_nm: [f64; 2],
        live_load: Option<SectionLoad>,
        allowable_pa: [f64; 2],
        load: Source,
        limit: Source,
    },
}

#[derive(Deserialize)]
#[serde(rename_all = "lowercase")]
enum Axis {
    X,
    Y,
    Z,
}

impl CheckSpec {
    fn resolve(self) -> Result<Check, String> {
        let class = REGISTRY
            .iter()
            .find(|spec| spec.id == self.class)
            .ok_or_else(|| format!("неизвестный класс «{}»", self.class))?
            .class;
        let verification = match self.verification {
            CheckKind::Truth { limit } => Verification::Truth(limit),
            CheckKind::Measurement { limit, contact_duration, energy_path } => match (contact_duration, energy_path) {
                (None, None) => Verification::Measurement(limit),
                (Some(duration), None) => Verification::TissueMeasurement { limit, duration },
                (None, Some(path)) => Verification::EnergyMeasurement { limit, path },
                (Some(_), Some(_)) => return Err("укажите область длительности или путь энергии".into()),
            },
            CheckKind::Section {
                geometry,
                axis,
                moment_nm,
                live_load,
                allowable_pa,
                load,
                limit,
            } => Verification::Section {
                geometry,
                moment_nm,
                live_load,
                allowable_pa,
                load,
                limit,
                axis: match axis {
                    Axis::X => SectionAxis::X,
                    Axis::Y => SectionAxis::Y,
                    Axis::Z => SectionAxis::Z,
                },
            },
        };
        Ok(Check {
            class,
            verification,
            object: self.object,
            selector: RunSelector {
                station: self.station,
                mode: self.mode,
                path: self.path,
                faults: None,
            },
            window: self.window,
            applicability: self.applicability,
            scope: self.scope,
            inputs: self.inputs,
        })
    }
}

pub(super) struct ResultRows {
    pub rows: BTreeMap<String, RowReport>,
    pub statistics: CampaignStats,
    pub metric_worst: BTreeSet<String>,
    pub resumed: usize,
    pub readiness: Readiness,
    pub view: Option<Vec<u8>>,
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Saved {
    run_key: String,
    summary: RunSummary,
    report: RowReport,
}

enum RowExecution {
    Report(RowReport),
    Saved(Saved),
    Linked(RowReport, Option<Vec<u8>>),
    Live(CampaignEvidence, Option<Vec<u8>>),
}

pub(super) fn run(
    prefix: &str,
    path: &Path,
    installation: &Path,
    options: &Options,
    checkpoint: Option<&Checkpoint>,
    bench: &[super::bench::Bench],
) -> Result<ResultRows, String> {
    let mut files = processes::Inputs::default();
    files.capture(path)?;
    let spec: Spec = files.toml(installation)?;
    if spec.version != 1 {
        return Err("неизвестная версия схемы установки".into());
    }
    let campaign = Campaign::load(path).map_err(|e| e.to_string())?;
    let world = WorldInputs::load(campaign.world_path()).map_err(|e| e.to_string())?;
    processes::capture_world(&world.path, &mut files)?;
    let nominal = world.compile().map_err(|e| e.to_string())?;
    let models = campaign
        .catalog_path()
        .is_dir()
        .then(|| super::super::Models::load(campaign.catalog_path()))
        .transpose()?;
    let mut probe = Runtime::load(nominal.clone())?;
    if campaign.has_native_scenarios() {
        setup(&mut probe, &nominal, models.as_ref())?;
    } else {
        // Ссылочный профиль самостоятельно устанавливает каталог и проводку.
        probe.record_joint_state("запись состояния")?;
    }
    let header = probe.log().header();
    let targets = header
        .commands
        .iter()
        .map(|spec| {
            (
                Target::Effort {
                    name: spec.name.clone(),
                },
                Binding {
                    unit: spec.unit.clone(),
                    resource: spec.name.clone(),
                    unavailable: None,
                },
            )
        })
        .collect();
    let signals = header
        .truth
        .iter()
        .chain(&header.observations)
        .map(|spec| Signal {
            name: spec.name.clone(),
            unit: spec.unit.clone(),
        })
        .collect();
    let paths = spec
        .paths
        .into_iter()
        .map(|p| {
            let mut faults = BTreeMap::new();
            for fault in p.faults {
                let node = fault.node.clone();
                if faults
                    .insert(
                        fault,
                        FaultSupport::Available {
                            channel: node.clone(),
                            resource: node,
                        },
                    )
                    .is_some()
                {
                    return Err("повтор отказа в установке пути".to_owned());
                }
            }
            Ok(RequiredPath {
                station: p.station,
                mode: p.mode,
                path: p.path,
                source: p.source,
                grids: p.grids,
                faults,
                combinations: Vec::new(),
                unavailable: None,
            })
        })
        .collect::<Result<_, String>>()?;
    let mut references = BTreeMap::new();
    for reference in campaign.references() {
        let bytes = if reference.path.is_dir() {
            models
                .as_ref()
                .ok_or("каталог не разрешён")?
                .bytes
                .as_slice()
        } else {
            files.capture(&reference.path)?
        };
        let stamp = Fingerprint::of(bytes);
        if reference
            .file_fingerprint
            .is_some_and(|prior| prior != stamp)
        {
            return Err("источник кампании изменился при разрешении".into());
        }
        references.insert(
            reference.path.clone(),
            ResolvedReference {
                kind: reference.kind,
                file_fingerprint: reference.file_fingerprint,
                graph_fingerprint: stamp,
            },
        );
    }
    let plan = campaign
        .plan(&ResolvedInputs {
            world,
            references,
            paths,
            targets,
            signals,
            model_parameters: super::installation::model_parameters(
                campaign.sampling_axes(),
                &nominal,
                models.as_ref(),
                &BTreeMap::new(),
            )?,
            initial_velocity: true,
        })
        .map_err(|e| e.to_string())?;
    let electrical_spec = spec.electrical.as_ref().map(|spec| {
        let base = installation.parent().unwrap_or_else(|| Path::new("."));
        (base.join(&spec.profile), Time(spec.horizon_ns))
    });
    let mut electrical = crate::electrical::report::Installation::load(
        campaign.world_path(),
        campaign.catalog_path(),
        electrical_spec.as_ref().map(|(profile, _)| profile.as_path()),
    )?;
    let electrical_files = (!electrical.is_empty()).then(|| electrical.fingerprints());
    let build = match &electrical_files {
        Some(electrical) => Fingerprint::of(&encode((
            plan.fingerprint(), files.fingerprints(), header.world_hash, electrical,
        ))?),
        None => Fingerprint::of(&encode((plan.fingerprint(), files.fingerprints(), header.world_hash))?),
    };
    let electrical_result = if let Some((profile, end)) = electrical_spec {
        if end == Time::ZERO {
            return Err("нулевой горизонт электрического опыта кампании".into());
        }
        let mut run = crate::electrical::LiveExperiment::load(
            campaign.world_path(), &profile, campaign.catalog_path(), None, end,
            crate::electrical::LivePath::Full,
        )?;
        electrical.bind_live(&run.runtime, end)?;
        run.run_until(end)?;
        Some(electrical.evaluate(build, Some(&run.runtime), Some(end))?)
    } else if electrical_files.is_some() {
        Some(electrical.evaluate(build, None, None)?)
    } else {
        None
    };
    let checks = spec.checks.into_iter().map(CheckSpec::resolve).collect::<Result<Vec<_>, _>>()?;
    let bench: Vec<_> = bench.iter().filter(|record| record.experiment == prefix).collect();
    for record in &bench {
        if record.build != build || spec.measurement_build.as_ref() != Some(&record.revision) {
            return Err(format!("«{prefix}»: заверенный ряд относится к другой сборке или ревизии"));
        }
        if plan.row(&record.run).is_none_or(|r| !matches!(r.disposition, Disposition::Ready))
            || !checks.iter().any(|check| {
                check.verification.accepts_measurement(&record.signal)
                    && check.selector.station.as_ref().is_none_or(|s| s == &record.run.station)
                    && check.selector.mode.as_ref().is_none_or(|m| m == &record.run.mode)
                    && check.selector.path.as_ref().is_none_or(|p| p == &record.run.path)
            })
        {
            return Err(format!("«{prefix}»: нет разрешённой строки и проверки канала заверенного ряда"));
        }
    }
    let inventory = spec.inventory.resolve(build, checks);
    let empty = BTreeMap::new();
    let requirements = derive_requirements_with_electrical(
        &inventory, &plan, electrical_result.as_ref().map_or(&empty, |result| &result.evidence),
    ).map_err(|e| e.to_string())?;
    let mut evidence = CampaignEvidence::new(&requirements);
    let mut rows = BTreeMap::new();
    let mut cli_keys = BTreeMap::new();
    let mut live = BTreeMap::new();
    let mut summaries = Vec::new();
    let mut linked_rows = BTreeSet::new();
    let mut linked_criteria = BTreeSet::new();
    let mut linked_definitions = BTreeMap::new();
    let mut resumed = 0;
    let mut view = None;
    drop(probe);
    let workers = options
        .workers
        .as_deref()
        .ok_or("не задан лимит исполнителей кампании")?;
    let completed = workers.map(plan.rows().iter().collect(), None, |row| {
        let key = &row.key;
        let sampled = plan.materialize(key).map_err(|e| e.to_string())?;
        let is_linked = sampled
            .experiment
            .as_ref()
            .is_some_and(|experiment| experiment.kind.is_some());
        let linked_mode = sampled.mode.as_ref().filter(|_| is_linked);
        let linked = is_linked.then(|| {
            let has_criteria = linked_mode
                .is_some_and(|mode| !mode.criteria.is_empty() || !mode.metrics.is_empty());
            let metrics = linked_mode.map(|mode| {
                mode.metrics
                    .iter()
                    .map(|metric| {
                        (
                            metric.id.clone(),
                            LinkedMetric {
                                unit: metric.unit().into(),
                                worse: metric.worse,
                                absolute_error: matches!(
                                    metric.operation,
                                    Operation::Absolute { .. }
                                ),
                            },
                        )
                    })
                    .collect::<BTreeMap<_, _>>()
            });
            (has_criteria, metrics)
        });
        let input = Fingerprint::of(&encode((build, sampled.fingerprint))?);
        // Адрес повтора связывает логический ключ с фактическими входами.
        // Иначе старый ключ с новым зерном молча запускает другой опыт.
        let run_key = format!("{prefix}/{:016x}", sim_signal::fnv1a(encode((key, input))?));
        let wants_linked = is_linked
            && options.repeat.as_deref().is_some_and(|wanted| {
                wanted
                    .strip_prefix(&run_key)
                    .is_some_and(|suffix| suffix.starts_with('@'))
            });
        let finish = |result| Ok((key, run_key.clone(), input, linked, result));
        if !selected(options, &run_key) && !wants_linked {
            return finish(RowExecution::Report(RowReport::omitted(input)));
        }
        let measurements: Vec<_> = bench.iter().filter(|record| &record.run == key).collect();
        let checkpoint = checkpoint.filter(|_| measurements.is_empty());
        if let Some(checkpoint) = checkpoint {
            let filename = checkpoint_name(&run_key, input)?;
            if let Some(saved) = checkpoint.load_row::<Saved>(&filename)? {
                if saved.run_key != run_key
                    || saved.summary.key != *key
                    || saved.report.input != input
                    || saved.report.outcome != Outcome::Fail
                    || saved.report.status != RunStatus::Completed
                    || !saved.summary.completed
                    || saved.report.executed != 1
                    || saved.report.required != 1
                    || saved.report.findings.is_empty()
                {
                    return Err(format!(
                        "контрольная точка не соответствует строке «{run_key}»"
                    ));
                }
                return finish(RowExecution::Saved(saved));
            }
        }
        if !matches!(&row.disposition, Disposition::Ready) {
            return finish(RowExecution::Report(RowReport {
                input,
                status: RunStatus::Open,
                detail: value(&row.disposition)?,
                ..RowReport::omitted(input)
            }));
        }
        if is_linked {
            if !measurements.is_empty() {
                return Err("для заверенного ряда нужна живая строка штатного оценщика".into());
            }
            let (result, archive) = linked_row(
                &campaign,
                &sampled,
                row,
                input,
                &run_key,
                spec.snapshot_ns,
                PlaybackFormat::for_options(options)
                    .filter(|_| selected(options, &run_key) || wants_linked),
                options,
            )?;
            return finish(RowExecution::Linked(result, archive));
        }
        let output = PlaybackFormat::for_options(options).filter(|_| selected(options, &run_key));
        let install = |runtime: &mut Runtime, sampled: &SampledInputs| {
            let compiled = sampled.world.compile().map_err(|e| e.to_string())?;
            setup(runtime, &compiled, models.as_ref())?;
            let settings = super::installation::apply_settings(runtime, &sampled.settings)?;
            let values = installed_values(
                sampled.values(),
                &sampled.world,
                &compiled,
                &settings,
                runtime,
            )?;
            if output == Some(PlaybackFormat::View) {
                runtime.record_view()?;
            }
            Ok(Installed {
                installation: Installation {
                    fingerprint: sampled.fingerprint,
                    run_seed: runtime.seed(),
                    physics_hz: runtime.physics_rate().as_hz() as u64,
                    values,
                },
                build,
                settings,
                operations: Vec::new(),
            })
        };
        let mut row_evidence = CampaignEvidence::new(&requirements);
        let mut archive = None;
        let result = row_evidence.execute_scenario_observed(
            &requirements,
            &plan,
            key,
            spec.snapshot_ns.map(Time),
            install,
            |runtime| {
                if let Some(format) = output {
                    archive = Some(format.collect(runtime, &key.mode, None)?);
                }
                Ok(())
            },
        );
        result.map_err(|e| format!("{run_key}: {e}"))?;
        for record in measurements {
            let (summary, _, _, _) = row_evidence
                .scenario_report(key)
                .ok_or("нет живого ряда оценки")?;
            if !summary.completed || summary.observed_through_ns != record.horizon_ns {
                return Err(format!(
                    "«{run_key}»: горизонт замера и исполненного прогона расходятся"
                ));
            }
            row_evidence
                .record_measurement(
                    &requirements,
                    key,
                    record.build,
                    record.signal.clone(),
                    record.source.clone(),
                    record.uncertainty,
                    &record.samples,
                )
                .map_err(|e| format!("«{run_key}»: {e}"))?;
        }
        finish(RowExecution::Live(row_evidence, archive))
    })?;
    let mut addresses = BTreeSet::new();
    for (key, mut run_key, input, linked, result) in completed {
        if !addresses.insert(run_key.clone()) {
            return Err("коллизия ключа строки".into());
        }
        if let Some((has_criteria, metrics)) = linked {
            linked_rows.insert(key.clone());
            if has_criteria {
                linked_criteria.insert(key.clone());
            }
            if let Some(metrics) = metrics {
                linked_definitions
                    .entry(RunKey {
                        sample: SampleKey::Nominal,
                        ..key.clone()
                    })
                    .or_insert(metrics);
            }
        }
        let report = match result {
            RowExecution::Report(report) => Some(report),
            RowExecution::Saved(saved) => {
                summaries.push(saved.summary);
                resumed += 1;
                Some(saved.report)
            }
            RowExecution::Linked(mut report, archive) => {
                if let Some(archive) = archive {
                    view = Some(archive);
                }
                let reason = "обязательства установки требуют отдельной привязки критериев ссылочного профиля";
                evidence
                    .record_unavailable(&requirements, key, reason.into())
                    .map_err(|e| e.to_string())?;
                report.findings.push(value(sim_task::verdict::Finding::open(
                    &super::requirement(
                        key,
                        sim_task::verdict::HazardClass::Inventory,
                        &key.station,
                    ),
                    reason,
                ))?);
                if report.status == RunStatus::Completed {
                    run_key = format!(
                        "{run_key}@{:016x}",
                        sim_signal::fnv1a(encode(report.input)?)
                    );
                }
                Some(report)
            }
            RowExecution::Live(row_evidence, archive) => {
                if let Some(archive) = archive {
                    view = Some(archive);
                }
                evidence.merge(row_evidence).map_err(|e| e.to_string())?;
                live.insert(key.clone(), (run_key.clone(), input));
                None
            }
        };
        cli_keys.insert(key.clone(), run_key.clone());
        if let Some(report) = report {
            rows.insert(run_key, report);
        }
    }
    let verdict = assess_campaign(&requirements, &evidence).map_err(|e| e.to_string())?;
    let global = verdict
        .findings
        .iter()
        .filter(|f| f.key.run.is_none())
        .collect::<Vec<_>>();
    for (key, run_key) in &cli_keys {
        if let Some(row) = rows.get_mut(run_key).filter(|row| {
            row.status == RunStatus::Open
                || (linked_rows.contains(key) && row.status != RunStatus::Omitted)
        }) {
            if let Some(electrical) = &electrical_result {
                row.outcome = row.outcome.max(electrical.report.outcome());
            }
            row.findings.extend(
                verdict
                    .findings
                    .iter()
                    .filter(|f| f.key.run.as_ref() == Some(key))
                    .chain(global.iter().copied())
                    .map(value)
                    .collect::<Result<Vec<_>, _>>()?,
            );
        }
    }
    for (key, (run_key, input)) in live {
        let (summary, journal, steps, snapshot) = evidence
            .scenario_report(&key)
            .ok_or("оценщик не сохранил результат запущенного сценария")?;
        let findings = verdict
            .findings
            .iter()
            .filter(|f| f.key.run.as_ref() == Some(&key))
            .chain(global.iter().copied())
            .collect::<Vec<_>>();
        let outcome = if findings.iter().any(|f| f.outcome == Outcome::Fail) {
            Outcome::Fail
        } else if verdict.simulation == SimulationVerdict::PassedSimulation {
            Outcome::Pass
        } else {
            Outcome::NotAnalyzed
        };
        let playback = summary.installation.as_ref().filter(|_| summary.completed)
            .and_then(|installed| {
                super::playback::select(
                    &key, installed.run_seed,
                    [super::playback::Grid {
                        variant: None,
                        hz: installed.physics_hz,
                        journal,
                        end_ns: summary.observed_through_ns,
                    }],
                    &findings,
                )
            });
        let event_ns = playback.as_ref().and_then(|p| p.event.as_ref())
            .filter(|event| event.violation).map(|event| event.at_ns);
        let row = RowReport {
            input,
            outcome,
            required: 1,
            status: if summary.completed {
                RunStatus::Completed
            } else {
                RunStatus::Incomplete
            },
            executed: usize::from(summary.completed),
            findings: findings.into_iter().map(value).collect::<Result<_, _>>()?,
            playback,
            detail: value(ScenarioReport {
                summary,
                journal,
                event_ns,
                steps,
                snapshot,
                settings: evidence.confirmed_settings(&key),
                measurement_build: spec.measurement_build.as_ref(),
                build_fingerprint: spec.measurement_build.as_ref().map(|_| {
                    // Fingerprint сериализует точное u64. TOML хранит отпечаток строкой.
                    let stamp = serde_json::to_value(build).expect("u64 представим в JSON");
                    format!("{:016x}", stamp.as_u64().expect("отпечаток сборки u64"))
                }),
            })?,
        };
        // Только завершённое нарушение можно повторно показать с диска.
        if let Some(checkpoint) = checkpoint
            && row.status == RunStatus::Completed
            && row.outcome == Outcome::Fail
        {
            let filename = checkpoint_name(&run_key, input)?;
            let saved = Saved {
                run_key: run_key.clone(),
                summary: summary.clone(),
                report: row,
            };
            checkpoint.save_row(&filename, &saved)?;
            rows.insert(run_key, saved.report);
        } else {
            rows.insert(run_key, row);
        }
        summaries.push(summary.clone());
    }
    if let Some(electrical) = &electrical_result
        && let Some(row) = rows.values_mut().find(|row| row.status != RunStatus::Omitted)
    {
        let detail = std::mem::take(&mut row.detail);
        row.detail = match detail {
            serde_json::Value::Object(mut fields) => {
                fields.insert("electrical".into(), value(&electrical.report)?);
                serde_json::Value::Object(fields)
            }
            other => serde_json::json!({ "scenario": other, "electrical": value(&electrical.report)? }),
        };
    }
    electrical.verify()?;
    files.verify()?;
    let mut statistics = summarize(&plan, &summaries).map_err(|e| e.to_string())?;
    if !linked_rows.is_empty() {
        let mut observations = Vec::new();
        let mut definitions = linked_definitions;
        for mut stat in std::mem::take(&mut statistics.rows) {
            if !linked_rows.contains(&stat.key) {
                statistics.rows.push(stat);
                continue;
            }
            let report = &rows[&cli_keys[&stat.key]];
            if let Some(children) = report.detail.get("children") {
                let children: Vec<Child> =
                    serde_json::from_value(children.clone()).map_err(|e| e.to_string())?;
                let [child] = children.as_slice() else {
                    return Err("нет строки ссылочной статистики".into());
                };
                let series = definitions
                    .entry(sim_task::campaign::RunKey {
                        sample: SampleKey::Nominal,
                        ..stat.key.clone()
                    })
                    .or_default();
                stat.completed = child.executed;
                stat.outcome = Some(if linked_criteria.contains(&stat.key) {
                    child.outcome.max(Outcome::NotAnalyzed)
                } else {
                    child.outcome
                });
                stat.reasons.clear();
                if linked_criteria.contains(&stat.key) {
                    stat.reasons.push("критерии и метрики общей постановки требуют привязки к рядам ссылочного профиля".into());
                }
                if child.outcome == Outcome::NotAnalyzed {
                    stat.reasons.push("профиль оставил строку открытой".into());
                }
                stat.metrics.clear();
                for (name, metric) in &child.metrics {
                    let definition = metric.definition();
                    if series
                        .insert(name.clone(), definition.clone())
                        .is_some_and(|old| old != definition)
                    {
                        return Err("в ссылочном профиле изменился смысл метрики".into());
                    }
                    stat.metrics.insert(
                        name.clone(),
                        MetricValue::Value {
                            value: metric.value,
                        },
                    );
                }
            }
            observations.push(stat);
        }
        let linked = summarize_linked(observations, Some(campaign.sampling()), &definitions)
            .map_err(|e| e.to_string())?;
        let linked_stations: BTreeSet<_> = linked_rows.iter().map(|key| &key.station).collect();
        statistics
            .series
            .retain(|series| !linked_stations.contains(&series.key.station));
        statistics.series.extend(linked.series);
        statistics.series.sort_by(|a, b| a.key.cmp(&b.key));
        statistics.rows.extend(linked.rows);
        statistics.rows.sort_by(|a, b| a.key.cmp(&b.key));
    }
    let metric_worst = statistics
        .series
        .iter()
        .flat_map(|series| series.metrics.values())
        .filter_map(|metric| metric.worst.as_ref())
        .map(|worst| cli_keys[&worst.key].clone())
        .collect();
    Ok(ResultRows {
        rows,
        statistics,
        metric_worst,
        resumed,
        readiness: verdict.readiness,
        view,
    })
}

fn linked_row(
    campaign: &Campaign,
    sampled: &SampledInputs,
    row: &PlanRow,
    input: Fingerprint,
    run_key: &str,
    snapshot_ns: Option<u64>,
    format: Option<PlaybackFormat>,
    campaign_options: &Options,
) -> Result<(RowReport, Option<Vec<u8>>), String> {
    let experiment = sampled.experiment.as_ref().ok_or("нет ссылочного опыта")?;
    let kind = experiment.kind.ok_or("нет типа ссылочного опыта")?;
    let task = experiment
        .task
        .as_deref()
        .ok_or("нет файла ссылочного задания")?;
    let mode = sampled.mode.as_ref().ok_or("нет режима ссылочного опыта")?;
    let open = |reason: &str| -> Result<(RowReport, Option<Vec<u8>>), String> {
        Ok((RowReport {
            status: RunStatus::Open,
            detail: value(reason)?,
            ..RowReport::omitted(input)
        }, None))
    };
    if !mode.initial.is_empty() || !mode.poses.is_empty() || !mode.steps.is_empty() {
        return open(
            "положения и действия ссылочного опыта задаёт его профиль; перенос встроенного расписания недоступен",
        );
    }
    if snapshot_ns.is_some() {
        return open("снимок ссылочного опыта задаётся в его профиле");
    }
    let control = kind == ReferenceKind::ControlTask;
    if (!control && (row.key.path != row.key.mode || !row.faults.is_empty()))
        || row.faults.len() > 1
    {
        return open("путь или сочетание отказов отсутствуют в формате ссылочного профиля");
    }
    if let Some(expected) = &experiment.profile {
        let mut files = processes::Inputs::default();
        let document: toml::Table = files.toml(task)?;
        let field = match kind {
            ReferenceKind::ControlTask => "profile",
            ReferenceKind::ProcessTask => "process_profile",
            ReferenceKind::InteractionTask => "interaction_profile",
            _ => return Err("неподдерживаемый тип ссылки постановки".into()),
        };
        let actual = document
            .get(field)
            .and_then(toml::Value::as_str)
            .ok_or("нет профиля в ссылочном задании")?;
        if !processes::dir(task)
            .join(actual)
            .canonicalize()
            .is_ok_and(|path| path == *expected)
        {
            return open("stations.*.profile расходится с профилем задания");
        }
    }
    let options = Options {
        case: (!control).then(|| row.key.mode.clone()),
        selected_control_case: control.then(|| Case {
            mode: row.key.mode.clone(),
            path: row.key.path.clone(),
            fault: row.key.faults.first().cloned(),
        }),
        linked_grid_hz: Some(row.grid.hz),
        workers: campaign_options.workers.clone(),
        seed: Some(sampled.sample.seeds.run),
        catalog: campaign
            .catalog_path()
            .is_dir()
            .then(|| campaign.catalog_path().to_owned()),
        ..Options::default()
    };
    let (mut result, _, view) = linked(
        input,
        campaign.world_path(),
        task,
        run_key,
        true,
        options,
        Some(LinkedSample {
            sampling: campaign.sampling(),
            sample: &sampled.sample,
        }),
        Some("ссылочный профиль подтверждает результат своего испытания"),
        None,
        format,
    )?;
    if result.status == RunStatus::Open {
        return Ok((result, None));
    }
    let mut children: Vec<Child> =
        serde_json::from_value(result.detail["children"].clone()).map_err(|e| e.to_string())?;
    let [child] = children.as_mut_slice() else {
        return Err("ссылочный опыт должен выдать ровно одну строку".into());
    };
    let stamp = &child.execution;
    let schedule_matches = row.faults.iter().all(|fault| {
        stamp.requested_ns == Some(fault.schedule.at_ns)
            && stamp.fault_delay_ns == fault.schedule.extra_delay_ns
            && stamp.applied_ns.is_some_and(|at| {
                (fault.schedule.at_ns..=fault.schedule.at_ns + fault.schedule.max_lag_ns)
                    .contains(&at)
                    && (fault.window.start_ns..fault.window.end_ns).contains(&at)
            })
    });
    if !child.executed
        || stamp.hz != row.grid.hz
        || stamp.seed != sampled.sample.seeds.run
        || stamp.reached_ns != Some(mode.horizon_ns)
        || !schedule_matches
    {
        let (mut unconfirmed, _) = open(
            "нет подтверждённого горизонта, сетки, зерна или применения отказа ссылочной строки",
        )?;
        unconfirmed.detail = value((unconfirmed.detail, result.detail))?;
        return Ok((unconfirmed, None));
    }
    let old_key = value(&child.key)?;
    child.key = row.key.clone();
    let actual_key = value(&child.key)?;
    for finding in &mut result.findings {
        if finding["key"]["run"] == old_key {
            finding["key"]["run"] = actual_key.clone();
        }
    }
    if let Some(playback) = &mut result.playback {
        playback.child = child.key.clone();
    }
    result.detail["children"] = value(children)?;
    Ok((result, view))
}

#[derive(Serialize)]
struct ScenarioReport<'a> {
    summary: &'a RunSummary,
    journal: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    event_ns: Option<u64>,
    steps: u64,
    snapshot: Option<&'a sim_task::verdict::SnapshotAudit>,
    #[serde(skip_serializing_if = "Option::is_none")]
    settings: Option<&'a ModelSettings>,
    #[serde(skip_serializing_if = "Option::is_none")]
    measurement_build: Option<&'a sim_reality::BuildId>,
    #[serde(skip_serializing_if = "Option::is_none")]
    build_fingerprint: Option<String>,
}

fn checkpoint_name(key: &str, input: Fingerprint) -> Result<String, String> {
    Ok(format!("{:016x}", sim_signal::fnv1a(encode((key, input))?)))
}

fn setup(
    runtime: &mut Runtime,
    world: &CompiledWorld,
    models: Option<&super::super::Models>,
) -> Result<(), String> {
    if let Some(models) = models {
        sim_drive::install(runtime, world, &models.drives)?;
        sim_sense::install(runtime, world, &models.sensors)?;
    } else if !world.drives.is_empty() || !world.sensors.is_empty() {
        return Err("для приводов или датчиков нужен каталог моделей".into());
    }
    runtime.record_joint_state("запись состояния")
}

pub(super) fn installed_values(
    values: &[AppliedValue],
    world: &WorldInputs,
    compiled: &CompiledWorld,
    settings: &ModelSettings,
    runtime: &Runtime,
) -> Result<BTreeMap<String, f64>, String> {
    if values.is_empty() {
        return Ok(BTreeMap::new());
    }
    let sim = compiled.open_contact_sim()?;
    let joints = sim.joints();
    let bodies = sim.bodies();
    Ok(values
        .iter()
        .filter_map(|sample| {
            let actual = match &sample.parameter {
                Parameter::JointVelocity { joint, dof } => {
                    settings.initial_velocity.get(joint)?.get(dof).copied()
                }
                Parameter::ChannelDelay { channel } => settings
                    .channel_delay_ns
                    .get(channel)
                    .filter(|delay| Some(**delay) == sim_task::sampling::delay_ns(sample.value))
                    .map(|_| sample.value),
                Parameter::JointPosition { joint } => {
                    let joint = joints.iter().find(|j| j.name == *joint)?;
                    sim.qpos().get(joint.qpos_adr).copied()
                }
                Parameter::Mass { body } => {
                    let id = bodies.iter().position(|(name, _)| name == body)?;
                    Some(sim.body_mass(id))
                }
                Parameter::Gravity { axis } => sim.gravity().get(usize::from(*axis)).copied(),
                Parameter::Density => Some(sim.model().opt.density),
                Parameter::Viscosity => Some(sim.model().opt.viscosity),
                Parameter::Wind { axis } => sim.model().opt.wind.get(usize::from(*axis)).copied(),
                Parameter::Friction { pair, component } => compiled
                    .explicit_contact_settings([&pair[0], &pair[1]])
                    .ok()
                    .flatten()?
                    .friction
                    .get(usize::from(*component))
                    .copied(),
                other => {
                    if let Some((name, field)) = super::installation::sensor_field(other) {
                        let prefix = format!("{name}/");
                        runtime
                            .log()
                            .header()
                            .observations
                            .iter()
                            .any(|s| s.name.starts_with(&prefix))
                            .then_some(())?;
                        compiled
                            .sensors
                            .iter()
                            .find(|s| s.name == name)?
                            .params
                            .get(field)
                            .copied()
                    } else {
                        installed_offset(world, other)
                    }
                }
            }?;
            Some((sample.id.clone(), actual))
        })
        .collect())
}

fn installed_offset(inputs: &WorldInputs, parameter: &Parameter) -> Option<f64> {
    let world = &inputs.world;
    match parameter {
        Parameter::PlacementOffset { placement, axis } => world
            .devices
            .iter()
            .find(|p| p.name == *placement)?
            .pose
            .position_m
            .get(usize::from(*axis))
            .copied(),
        Parameter::LinkOffset {
            placement,
            link,
            axis,
        } => inputs
            .devices
            .get(placement)?
            .device
            .links
            .iter()
            .find(|l| l.name == *link)?
            .pose
            .position_m
            .get(usize::from(*axis))
            .copied(),
        Parameter::ObjectOffset { object, axis } => world
            .objects
            .iter()
            .chain(&inputs.contact.added_objects)
            .find(|o| o.name == *object)?
            .pose
            .position_m
            .get(usize::from(*axis))
            .copied(),
        _ => None,
    }
}
