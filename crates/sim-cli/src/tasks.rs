mod campaign;
mod control_witness;
mod interaction;
mod matrix;
mod measurements;
pub(crate) mod processes;
mod readouts;
mod reference;
pub(crate) mod workers;

use std::collections::{BTreeMap, BTreeSet};
use std::io::Write;
use std::path::{Path, PathBuf};

use campaign::installation::{LinkedSample, Receipt};
use serde::{Deserialize, Serialize};
use sim_runtime::Runtime;
use sim_runtime::control::{
    self, ControlConfig, ControlPath, FaultMechanism, FileAssets, ModelPort, RunSpec,
};
use sim_signal::Time;
use sim_task::{
    Assessor, Case, Direction, FaultKind, Fingerprint, Hop, Inputs, Node, NodeKind, RunKind,
    RunPlan, RunResult, Signal, Source, Task, Topology,
};
use sim_task::{campaign::WorldInputs, sampling::ModelSettings};
use sim_world::CompiledWorld;

use crate::{Options, catalog_path, export};

pub(super) fn run_campaign(
    path: &Path,
    options: &Options,
    out: &mut impl Write,
) -> Result<u8, String> {
    let mut files = processes::Inputs::default();
    let document: toml::Value = files.toml(path)?;
    if document.get("electrical_format").is_some() {
        let manifest: crate::electrical::report::Campaign = document.try_into()
            .map_err(|err: toml::de::Error| err.to_string())?;
        files.verify()?;
        return manifest.run(path, options, out);
    }
    campaign::run(path, options, out)
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Profile {
    version: u32,
    required_modes: Vec<String>,
    paths: BTreeMap<String, PathProfile>,
    #[serde(default)]
    mode_control: BTreeMap<String, String>,
    #[serde(default)]
    mode_physics: BTreeMap<String, ModePhysicsRef>,
    #[serde(default)]
    limitation_paths: BTreeMap<String, BTreeSet<String>>,
    #[serde(default)]
    readouts: Vec<readouts::Readout>,
    #[serde(default)]
    measurements: Vec<measurements::Measurement>,
    #[serde(default)]
    sampled_drives: BTreeMap<String, String>,
    #[serde(default)]
    sampled_modes: BTreeMap<String, SampledMode>,
    #[serde(default)]
    reference: BTreeMap<String, reference::Reference>,
    #[serde(default)]
    deferred_modes: BTreeMap<String, String>,
    #[serde(default)]
    hazard_witnesses: Vec<control_witness::Spec>,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct SampledMode {
    ambient_c: Option<f64>,
    fixture_enabled: Option<bool>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ModePhysicsRef {
    case: PathBuf,
    witness: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PathProfile {
    observation: Signal,
    physical_response: Vec<Signal>,
    command_delta: f64,
    max_observation_age_ns: u64,
    source: Source,
    #[serde(default)]
    hops: Vec<HopProfile>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct HopProfile {
    target: String,
    direction: HopDirection,
    delay_ns: u64,
    jitter_ns: u64,
    receiver_tick_ns: u64,
    source: Source,
}

#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
enum HopDirection {
    Input,
    Output,
}

#[derive(Serialize)]
struct Report<'a, E> {
    status: &'static str,
    topology: &'a Topology,
    resolved_wiring: Vec<ResolvedWire>,
    installed_inputs: BTreeMap<String, InstalledInputs>,
    closed_loop: sim_task::Outcome,
    coverage: matrix::Coverage,
    evidence: E,
    reference: BTreeMap<String, reference::Comparison>,
}

#[derive(Clone, Serialize)]
struct ResolvedWire {
    name: String,
    producer: String,
    target: Signal,
    accepted: Option<String>,
    program: Fingerprint,
    feedback: Vec<Signal>,
    faults: BTreeMap<String, Vec<String>>,
}

#[derive(Serialize)]
struct InstalledInputs {
    world: Fingerprint,
    wiring: Fingerprint,
    assets: BTreeMap<String, Fingerprint>,
    #[serde(skip_serializing_if = "Option::is_none")]
    installation: Option<Receipt>,
}

fn resolved_wiring(run: &control::ControlRun<'_>) -> Vec<ResolvedWire> {
    let header = run.log().header();
    run.paths()
        .iter()
        .map(|path| ResolvedWire {
            name: path.name.clone(),
            producer: path.producer.clone(),
            target: Signal {
                name: path.target.clone(),
                unit: path.unit.clone(),
            },
            accepted: path.accepted_truth.and_then(|ack| {
                // Имя сконфигурированного подтверждения приводится в топологии.
                run.truth_spec(ack).map(|spec| spec.name.clone())
            }),
            program: Fingerprint::from_fnv1a(path.program_hash),
            feedback: path
                .feedback
                .iter()
                .map(|id| {
                    let spec = &header.observations[id.0 as usize];
                    Signal {
                        name: spec.name.clone(),
                        unit: spec.unit.clone(),
                    }
                })
                .collect(),
            faults: path
                .fault_targets
                .iter()
                .map(|target| {
                    (
                        target.name.clone(),
                        target
                            .mechanisms
                            .iter()
                            .map(|mechanism| format!("{mechanism:?}"))
                            .collect(),
                    )
                })
                .collect(),
        })
        .collect()
}

pub(crate) struct Models {
    pub(crate) drives: sim_drive::DriveCatalog,
    pub(crate) sensors: sim_sense::SensorCatalog,
    pub(crate) bytes: Vec<u8>,
}

impl Models {
    pub(crate) fn load(base: &Path) -> Result<Self, String> {
        let mut bytes = Vec::new();
        for dirname in ["drives", "sensors", "boards"] {
            let dir = base.join(dirname);
            let mut entries = std::fs::read_dir(&dir)
                .map_err(|error| format!("{}: {error}", dir.display()))?
                .collect::<Result<Vec<_>, _>>()
                .map_err(|error| error.to_string())?;
            entries.sort_by_key(|e| e.file_name());
            for entry in entries {
                let name = entry.file_name();
                let path = entry.path();
                if path.extension().is_some_and(|ext| ext == "toml") {
                    frame(&mut bytes, dirname.as_bytes());
                    frame(&mut bytes, name.as_encoded_bytes());
                    frame(&mut bytes, &read(&path)?);
                }
            }
        }
        Ok(Self {
            drives: sim_drive::DriveCatalog::load(base.join("drives"))?,
            sensors: sim_sense::SensorCatalog::load(base.join("sensors"))?,
            bytes,
        })
    }
}

pub(crate) fn model_ports(handles: &[sim_drive::DriveHandle]) -> Vec<ModelPort> {
    handles
        .iter()
        .flat_map(|handle| {
            std::iter::once(ModelPort::Drive {
                channel: handle.command,
                accepted: handle.accepted_command,
            })
            .chain(handle.power.into_iter().chain(handle.bridge_mode).map(
                |channel| ModelPort::DrivePower {
                    channel,
                    drive: handle.command,
                },
            ))
        })
        .collect()
}

pub(crate) fn install_live_measurements(
    path: &Path,
    rt: &mut Runtime,
    world: &CompiledWorld,
    sampled: &BTreeMap<String, sim_drive::sampled::Spec>,
) -> Result<Vec<u8>, String> {
    #[derive(Deserialize)]
    struct Measurements {
        measurements: Vec<measurements::Measurement>,
    }
    let text = read_text(path)?;
    let data: Measurements = toml::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
    for measurement in data.measurements {
        measurement.install(rt, world, sampled)?;
    }
    Ok(text.into_bytes())
}

struct Setup<'a> {
    world: &'a CompiledWorld,
    readouts: &'a [readouts::Readout],
    measurements: &'a [measurements::Measurement],
    sampled: BTreeMap<String, sim_drive::sampled::Spec>,
    sampled_modes: &'a BTreeMap<String, SampledMode>,
    models: Models,
    task_bytes: Vec<u8>,
    world_bytes: Vec<u8>,
    base: PathBuf,
    control: String,
    overlays: BTreeMap<String, String>,
}

impl Setup<'_> {
    fn config(&self, mode: &str, schedule: &str) -> Result<ControlConfig, String> {
        let overlay = self.overlays.get(mode).map_or("", String::as_str);
        ControlConfig::parse(&format!("{}\n{overlay}\n{schedule}", self.control))
    }

    fn reuse_fault(&self, mode: &str, injection: &sim_task::Injection) -> Option<String> {
        let overlay = self.overlays.get(mode).map_or("", String::as_str);
        let mut config: toml::Table = toml::from_str(&format!("{}\n{overlay}", self.control)).ok()?;
        let faults = config.get_mut("fault")?.as_array_mut()?;
        if faults.iter().any(|value| value.get("id").and_then(toml::Value::as_str) == Some("task")) {
            return None;
        }
        let matches = |t: &toml::Value| {
            let get = |field: &str| t.get(field);
            get("target").and_then(toml::Value::as_str) == Some(injection.fault.node.as_str())
                && get("kind").and_then(toml::Value::as_str) == Some(mechanism(injection.fault.kind).1)
                && get("at_ns").and_then(toml::Value::as_integer).and_then(|v| u64::try_from(v).ok()) == Some(injection.at_ns)
                && get("delay_ns").and_then(toml::Value::as_integer).and_then(|v| u64::try_from(v).ok()) == injection.extra_delay_ns
                && get("until_ns").is_none()
        };
        let mut matching = faults.iter_mut().filter(|t| matches(t));
        let item = matching.next()?.as_table_mut()?;
        if matching.next().is_some() {
            return None;
        }
        // Тот же отказ с тем же моментом и длительностью. Отмечаем его как
        // выбранную инъекцию, чтобы оценщик потребовал фактическое применение.
        item.insert("id".into(), toml::Value::String("task".into()));
        toml::to_string(&config).ok()
    }

    fn install(
        &self,
        rt: &mut Runtime,
        mode: &str,
        horizon: u64,
        schedule: &str,
    ) -> Result<Vec<sim_sense::SensorHandle>, String> {
        let config = self.config(mode, schedule)?;
        self.install_config(rt, mode, horizon, &config)
    }

    fn install_config(
        &self,
        rt: &mut Runtime,
        mode: &str,
        horizon: u64,
        config: &ControlConfig,
    ) -> Result<Vec<sim_sense::SensorHandle>, String> {
        self.install_config_on(rt, mode, horizon, config, self.world)
    }

    fn install_config_on(
        &self,
        rt: &mut Runtime,
        mode: &str,
        horizon: u64,
        config: &ControlConfig,
        compiled: &CompiledWorld,
    ) -> Result<Vec<sim_sense::SensorHandle>, String> {
        let mut world = compiled.clone();
        world
            .drives
            .retain(|drive| !self.sampled.contains_key(&drive.name));
        let handles = sim_drive::install(rt, &world, &self.models.drives)?;
        let mut ports = model_ports(&handles);
        let mut sampled = self.sampled.clone();
        for (name, spec) in &mut sampled {
            if let Some(overrides) = self.sampled_modes.get(mode) {
                if let Some(ambient) = overrides.ambient_c {
                    spec.ambient_c = ambient;
                }
                if let Some(enabled) = overrides.fixture_enabled {
                    spec.fixture_enabled = enabled;
                }
            }
            ports.extend(sim_drive::sampled::install(
                rt,
                compiled,
                name,
                spec.clone(),
            )?);
        }
        let sensors = sim_sense::install(rt, compiled, &self.models.sensors)?;
        for measurement in self.measurements {
            measurement.install(rt, compiled, &sampled)?;
        }
        rt.record_joint_state("запись состояния")?;
        for readout in self.readouts {
            readout.install(rt)?;
        }
        control::install(
            rt,
            config,
            &FileAssets { base: &self.base },
            RunSpec {
                world: if std::ptr::eq(compiled, self.world) {
                    &self.world_bytes
                } else {
                    compiled.mjcf.as_bytes()
                },
                catalog: &self.models.bytes,
                task: &self.task_bytes,
                criteria: &self.task_bytes,
                mode,
                horizon: Time(horizon),
                model_ports: &ports,
            },
        )?;
        Ok(sensors)
    }
}

fn read(path: &Path) -> Result<Vec<u8>, String> {
    std::fs::read(path).map_err(|error| format!("{}: {error}", path.display()))
}

fn read_text(path: &Path) -> Result<String, String> {
    std::fs::read_to_string(path).map_err(|error| format!("{}: {error}", path.display()))
}

fn frame(bytes: &mut Vec<u8>, part: &[u8]) {
    bytes.extend_from_slice(&(part.len() as u64).to_le_bytes());
    bytes.extend_from_slice(part);
}

fn node_kind(target: &str, path: &ControlPath) -> Result<NodeKind, String> {
    let (kind, name) = target.split_once(':').ok_or("неизвестная цель отказа")?;
    Ok(match kind {
        "sensor" => NodeKind::Sensor,
        "wire" if name == path.name => NodeKind::Driver,
        "wire" => NodeKind::Wire,
        "board" | "controller" => NodeKind::Controller,
        "actuator" => NodeKind::Drive,
        "power" => NodeKind::Power,
        "transport" | "bus" => NodeKind::Bus,
        "channel" if path.feedback.is_empty() => NodeKind::Wire,
        "channel" => NodeKind::Radio,
        _ => return Err(format!("неизвестная роль цели отказа «{target}»")),
    })
}

fn topology(
    world: &CompiledWorld,
    installed: &control::ControlRun<'_>,
    profile: &Profile,
) -> Result<Topology, String> {
    let mut paths = Vec::new();
    for resolved in installed
        .paths()
        .iter()
        .filter(|path| path.accepted_truth.is_some())
    {
        let path = profile
            .paths
            .get(&resolved.name)
            .ok_or_else(|| format!("нет профиля пути «{}»", resolved.name))?;
        let header = installed.log().header();
        if !resolved.feedback.iter().any(|id| {
            let spec = &header.observations[id.0 as usize];
            spec.name == path.observation.name && spec.unit == path.observation.unit
        }) {
            return Err(format!(
                "измерение «{}» отсутствует на пути «{}»",
                path.observation.name, resolved.name
            ));
        }
        let drive = world
            .drives
            .iter()
            .find(|drive| drive.effort == resolved.target)
            .ok_or_else(|| format!("нет привода на пути «{}»", resolved.name))?;
        let ack = header
            .truth
            .iter()
            .find(|spec| {
                spec.name == format!("{}/accepted_command", drive.name)
                    && spec.unit == resolved.unit
            })
            .ok_or_else(|| format!("нет подтверждения пути «{}»", resolved.name))?;
        for signal in &path.physical_response {
            if !header
                .truth
                .iter()
                .any(|spec| spec.name == signal.name && spec.unit == signal.unit)
            {
                return Err(format!(
                    "нет ряда истины «{}» с единицей «{}»",
                    signal.name, signal.unit
                ));
            }
        }
        let mut nodes = BTreeMap::new();
        for target in &resolved.fault_targets {
            let kind = node_kind(&target.name, resolved)?;
            nodes.insert(
                target.name.clone(),
                Node {
                    id: target.name.clone(),
                    kind,
                    fault_state: None,
                },
            );
        }
        if !nodes.values().any(|node| node.kind == NodeKind::Power) {
            // Неподключённая силовая цепь остаётся обязательной, но недоказанной.
            let power = format!("power:{}", drive.name);
            nodes.insert(
                power.clone(),
                Node {
                    id: power,
                    kind: NodeKind::Power,
                    fault_state: None,
                },
            );
        }
        let hops = path
            .hops
            .iter()
            .map(|hop| Hop {
                id: hop.target.clone(),
                node: hop.target.clone(),
                direction: match hop.direction {
                    HopDirection::Input => Direction::Input,
                    HopDirection::Output => Direction::Output,
                },
                delay_ns: hop.delay_ns,
                jitter_ns: hop.jitter_ns,
                receiver_tick_ns: hop.receiver_tick_ns,
                source: hop.source.clone(),
            })
            .collect();
        paths.push(sim_task::Path {
            id: resolved.name.clone(),
            drive: drive.name.clone(),
            observation: path.observation.clone(),
            command: Signal {
                name: resolved.target.clone(),
                unit: resolved.unit.clone(),
            },
            accepted: Signal {
                name: ack.name.clone(),
                unit: ack.unit.clone(),
            },
            physical_response: path.physical_response.clone(),
            controller: resolved.producer.clone(),
            program: Fingerprint::from_fnv1a(resolved.program_hash),
            command_delta: path.command_delta,
            max_observation_age_ns: path.max_observation_age_ns,
            source: path.source.clone(),
            nodes: nodes.into_values().collect(),
            hops,
            legacy: None,
        });
    }
    if paths.len() != profile.paths.len() || paths.is_empty() {
        return Err("профиль должен описывать каждый установленный путь с подтверждением".into());
    }
    Ok(Topology {
        required_modes: profile.required_modes.clone(),
        paths,
        limitation_paths: profile.limitation_paths.clone(),
    })
}

fn quoted(value: &str) -> Result<String, String> {
    serde_json::to_string(value).map_err(|error| error.to_string())
}

fn mechanism(kind: FaultKind) -> (FaultMechanism, &'static str) {
    match kind {
        FaultKind::SensorUnavailable
        | FaultKind::WireOpen
        | FaultKind::DriverDisabled
        | FaultKind::DriveDisabled
        | FaultKind::PacketLoss => (FaultMechanism::Drop, "drop"),
        FaultKind::PacketDelay => (FaultMechanism::Delay, "delay"),
        FaultKind::ControllerHalted => (FaultMechanism::Pause, "pause"),
        FaultKind::PowerCut => (FaultMechanism::PowerOff, "power_off"),
    }
}

fn schedule(plan: &RunPlan<'_>, reuse_fault: bool) -> Result<String, String> {
    let mut text = String::new();
    for stimulus in &plan.mode.stimuli {
        text.push_str(&format!(
            "\n[[stimulus]]\nat_ns = {}\ntarget = {}\nunit = {}\nvalue = {}\nsource = {}\n",
            stimulus.at_ns,
            quoted(&stimulus.signal.name)?,
            quoted(&stimulus.signal.unit)?,
            plan.stimulus_value(stimulus),
            quoted(&stimulus.source.reference)?
        ));
    }
    if let Some(injection) = plan.fault.filter(|_| !reuse_fault) {
        let (_, kind) = mechanism(injection.fault.kind);
        text.push_str(&format!(
            "\n[[fault]]\nid = 'task'\ntarget = {}\nat_ns = {}\nkind = '{kind}'\nsource = {}\n",
            quoted(&injection.fault.node)?,
            injection.at_ns,
            quoted(&injection.source.reference)?
        ));
        if let Some(delay) = injection.extra_delay_ns {
            text.push_str(&format!("delay_ns = {delay}\n"));
        }
    }
    Ok(text)
}

struct Execution<'a> {
    setup: &'a Setup<'a>,
    assessor: &'a Assessor,
    inputs: &'a Inputs,
    assets: &'a BTreeMap<String, Vec<(String, u64)>>,
    references: &'a BTreeMap<String, reference::Reference>,
    reference_texts: &'a BTreeMap<String, String>,
    sample: Option<(LinkedSample<'a>, &'a WorldInputs, &'a ModelSettings)>,
    witnesses: &'a [control_witness::Spec],
    physics: &'a BTreeMap<String, processes::ModePhysics>,
    coverage_control: bool,
}

struct ExecutedCase {
    run: RunResult,
    comparison: Option<reference::Comparison>,
    observations: Vec<control_witness::Observation>,
    process_metrics: BTreeMap<String, [f64; 2]>,
}

impl Execution<'_> {
    fn run_case(
        &self,
        case: &Case,
        record_view: bool,
        mut export_live: impl FnMut(&Runtime) -> Result<(), String>,
    ) -> Result<ExecutedCase, String> {
        let mut comparison = None;
        let mut archive_error = None;
        let mut observations = Vec::new();
        let mut process_metrics = BTreeMap::new();
        let plan = self.assessor.plan(case).ok_or("нет плана опыта")?;
        let result = self
            .assessor
            .execute_observed_with_comparison(
                case,
                self.coverage_control,
                |rt, plan, _| {
                    let mode = &plan.mode.id;
                    let reuse = plan.fault.and_then(|cause| self.setup.reuse_fault(mode, cause));
                    let events = schedule(plan, reuse.is_some())?;
                    let config = match reuse {
                        Some(text) => ControlConfig::parse(&format!("{text}\n{events}"))?,
                        None => self.setup.config(mode, &events)?,
                    };
                    if let Some(physics) = self.physics.get(mode) {
                        self.setup.install_config_on(rt, mode, plan.mode.horizon_ns, &config, &physics.world)?;
                        physics.install(rt)?;
                    } else {
                        self.setup.install_config(rt, mode, plan.mode.horizon_ns, &config)?;
                    }
                    if let Some((sample, world, settings)) = self.sample {
                        if let Some(physics) = self.physics.get(mode) {
                            sample.install(rt, &physics.world, &physics.resolved, &physics.settings)?;
                        } else {
                            sample.install(rt, self.setup.world, world, settings)?;
                        }
                    }
                    let run = control::run(rt).ok_or("контур не установлен")?;
                    if run.identity().assets != self.assets[&plan.mode.id] {
                        return Err("ресурсы установленного режима изменились".into());
                    }
                    if record_view { rt.record_view()?; }
                    Ok(self.inputs.clone())
                },
                |kind, rt, _| {
                    if let Some((_, _, settings)) = self.sample {
                        let settings = self.physics.get(&case.mode)
                            .map_or(settings, |physics| &physics.settings);
                        for channel in settings.channel_delay_ns.keys() {
                            if !rt.channel_delay_observed(channel) {
                                return Err(format!("нет живой доставки с задержкой «{channel}»"));
                            }
                        }
                    }
                    if kind == RunKind::Live {
                        if let Some(physics) = self.physics.get(&case.mode) {
                            let (name, bounds) = physics.observe(rt)?;
                            process_metrics.insert(name.to_owned(), bounds);
                        }
                        match control_witness::collect(self.witnesses, &plan, rt) {
                            Ok(values) => observations = values,
                            Err(error) => {
                                archive_error = Some(error.clone());
                                return Err(error);
                            }
                        }
                    }
                    if kind != RunKind::Live {
                        return Ok(());
                    }
                    if let Err(error) = export_live(rt) {
                        archive_error = Some(error.clone());
                        return Err(error);
                    }
                    if case.fault.is_some() {
                        return Ok(());
                    }
                    if let Some(reference) = self.references.get(&case.mode) {
                        match reference.compare(&self.reference_texts[&case.mode], rt) {
                            Ok(result) => comparison = Some(result),
                            Err(error) => {
                                archive_error = Some(error.clone());
                                return Err(error);
                            }
                        }
                        if comparison.as_ref().is_some_and(|result| !result.passed) {
                            return Err("расчёт расходится с заявленным эталоном".into());
                        }
                    }
                    Ok(())
                },
            )
            .map_err(|error| error.to_string())?;
        if let Some(error) = archive_error {
            return Err(format!("выгрузка или эталон режима «{}»: {error}", case.mode));
        }
        observations.retain(|value| {
            control_witness::valid_report(&self.witnesses[value.spec], result.report())
        });
        Ok(ExecutedCase {
            run: result,
            comparison,
            observations,
            process_metrics,
        })
    }
}

fn write_task_report(
    report: &Report<'_, impl Serialize>,
    cases: &[sim_task::CaseReport],
    options: &Options,
    out: &mut impl Write,
) -> Result<u8, String> {
    if options.json {
        serde_json::to_writer(&mut *out, report).map_err(|error| error.to_string())?;
        writeln!(out).map_err(|error| error.to_string())?;
    } else {
        writeln!(
            out,
            "Результат замкнутого контура {:?}. Исполнено {} из {}",
            report.closed_loop, report.coverage.executed, report.coverage.required,
        )
        .map_err(|error| error.to_string())?;
        for case in cases {
            writeln!(
                out,
                "{} / {} / {:?} — {:?}",
                case.case.mode, case.case.path, case.case.fault, case.outcome
            )
            .map_err(|error| error.to_string())?;
            for reason in &case.missing {
                writeln!(out, "  {reason}").map_err(|error| error.to_string())?;
            }
        }
    }
    Ok(u8::from(report.status != "completed"))
}

pub(super) fn run(
    world: CompiledWorld,
    task_path: &Path,
    options: &Options,
    out: &mut impl Write,
) -> Result<u8, String> {
    if let Some(task) = interaction::TaskFile::load(task_path)? {
        return interaction::run(task, task_path, options, out);
    }
    if let Some(task) = processes::TaskFile::load(task_path)? {
        return processes::run(task, task_path, options, out);
    }
    run_control(world, task_path, options, out, None, None)
}

fn run_control(
    mut world: CompiledWorld,
    task_path: &Path,
    options: &Options,
    out: &mut impl Write,
    mut audit: Option<&mut campaign::LinkedAudit>,
    sample: Option<LinkedSample<'_>>,
) -> Result<u8, String> {
    if options.jobs.is_some() && !options.matrix_full {
        return Err("--jobs требует --matrix full или профиль процессов".into());
    }
    if let Some(seed) = options.seed {
        world.seed = seed;
    }
    let mut task = Task::load(task_path).map_err(|error| error.to_string())?;
    let task_bytes = read(task_path)?;
    if Fingerprint::of(&task_bytes) != task.fingerprint() {
        return Err("задача изменилась во время чтения".into());
    }
    let task_dir = task_path.parent().unwrap_or_else(|| Path::new("."));
    let profile_path = task_dir.join(
        task.profile()
            .ok_or("в задаче нужен отдельный профиль эксплуатации")?,
    );
    let profile_bytes = read(&profile_path)?;
    processes::ensure_finite(&processes::parse(&profile_path, &profile_bytes)?)?;
    let profile: Profile = processes::parse(&profile_path, &profile_bytes)?;
    if profile.version != 1 || profile.required_modes.is_empty() {
        return Err("требуется непустой профиль версии 1".into());
    }
    for (mode, overrides) in &profile.sampled_modes {
        if !profile.required_modes.contains(mode)
            || overrides.ambient_c.is_some_and(|v| !v.is_finite())
        {
            return Err(format!(
                "некорректная настройка дискретной модели для «{mode}»"
            ));
        }
    }
    for (mode, reason) in &profile.deferred_modes {
        if !profile.required_modes.contains(mode)
            || task.modes().iter().any(|item| &item.id == mode)
            || reason.trim().is_empty()
        {
            return Err(format!("некорректная причина отложенного режима «{mode}»"));
        }
    }
    let mut reference_texts = BTreeMap::new();
    for (mode, reference) in &profile.reference {
        if !profile.required_modes.contains(mode) {
            return Err(format!("эталон объявлен для неизвестного режима «{mode}»"));
        }
        reference.validate()?;
        let text = read_text(&task_dir.join(&reference.file))?;
        reference.validate_text(&text)?;
        reference_texts.insert(mode.clone(), text);
    }
    let control_path = task_dir.join(task.control());
    let mut setup = Setup {
        world: &world,
        readouts: &profile.readouts,
        measurements: &profile.measurements,
        sampled: BTreeMap::new(),
        sampled_modes: &profile.sampled_modes,
        models: Models::load(&catalog_path(options)?)?,
        task_bytes,
        world_bytes: read(&options.world)?,
        base: control_path
            .parent()
            .unwrap_or_else(|| Path::new("."))
            .to_owned(),
        control: read_text(&control_path)?,
        overlays: BTreeMap::new(),
    };
    for (mode, path) in &profile.mode_control {
        if !profile.required_modes.contains(mode) {
            return Err(format!(
                "проводка объявлена для неизвестного режима «{mode}»"
            ));
        }
        setup
            .overlays
            .insert(mode.clone(), read_text(&task_dir.join(path))?);
    }
    let mut wiring = profile_bytes;
    frame(&mut wiring, setup.control.as_bytes());
    for (drive, resource) in &profile.sampled_drives {
        let text = read_text(&task_dir.join(resource))?;
        let spec = toml::from_str(&text).map_err(|error| format!("{resource}: {error}"))?;
        setup.sampled.insert(drive.clone(), spec);
        frame(&mut wiring, drive.as_bytes());
        frame(&mut wiring, text.as_bytes());
    }
    let sampled_world = if let Some(sample) = sample {
        let mut resolved = WorldInputs::load(&options.world).map_err(|e| e.to_string())?;
        if let Some(hz) = options.linked_grid_hz {
            resolved.world.physics_hz = hz;
        }
        let mut nominal_wires = BTreeMap::new();
        for mode in task.modes() {
            let wires = setup.config(&mode.id, "")?.wired_delays()?;
            for axis in sample.sampling.axes() {
                let sim_task::sampling::Parameter::ChannelDelay { channel } = &axis.parameter
                else {
                    continue;
                };
                if channel.starts_with("observation:") {
                    continue;
                }
                let delay = *wires.get(channel).ok_or_else(|| {
                    format!(
                        "режим «{}»: неизвестная задержка канала «{channel}»",
                        mode.id
                    )
                })?;
                if nominal_wires
                    .insert(channel.clone(), delay)
                    .is_some_and(|old| old != delay)
                {
                    return Err(format!(
                        "номинальная задержка «{channel}» различается между режимами"
                    ));
                }
            }
        }
        let settings =
            sample.overlay(&world, &mut resolved, Some(&setup.models), &nominal_wires)?;
        let compiled = resolved.compile().map_err(|e| e.to_string())?;
        Some((sample, resolved, compiled, settings))
    } else {
        None
    };
    let world = sampled_world
        .as_ref()
        .map_or(&world, |(_, _, world, _)| world);
    setup.world = world;
    let installation = sampled_world
        .as_ref()
        .map(|(sample, world, _, settings)| (*sample, world, settings));
    let mut physics_files = processes::Inputs::default();
    let physics = profile.mode_physics.iter().map(|(id, reference)| {
        let mode = task.modes().iter().find(|mode| &mode.id == id)
            .ok_or_else(|| format!("физика объявлена для неизвестного режима «{id}»"))?;
        let mut value = processes::ModePhysics::load(
            &task_dir.join(&reference.case), &reference.witness,
            &options.world, world.seed, world.physics, mode.horizon_ns, &mut physics_files,
        )?;
        if let Some(sample) = sample {
            let wires = setup.config(id, "")?.wired_delays()?;
            value.apply_sample(sample, &setup.models, &wires)?;
        }
        Ok((id.clone(), value))
    }).collect::<Result<BTreeMap<_, _>, String>>()?;
    if !physics.is_empty() {
        frame(&mut wiring, format!("{:?}", physics_files.fingerprints()).as_bytes());
    }
    let mut resource_bytes = Vec::new();
    let mut expected = BTreeMap::new();
    let mut resolved_topology = None;
    let mut wires = Vec::new();
    let mut mode_wires = BTreeMap::new();
    let mut installed_inputs = BTreeMap::new();
    let mut build = None;
    let mut catalog = None;
    let inventory_world = if audit.is_some() {
        Some(
            sampled_world
                .as_ref()
                .map(|(_, resolved, _, _)| resolved.clone())
                .map_or_else(
                    || WorldInputs::load(&options.world).map_err(|e| e.to_string()),
                    Ok,
                )?,
        )
    } else {
        None
    };
    for mode in task.modes() {
        let mode_physics = physics.get(&mode.id);
        let mode_world = mode_physics.map_or(world, |p| &p.world);
        let mut probe = Runtime::load(mode_world.clone())?;
        if let Some(physics) = mode_physics {
            let config = setup.config(&mode.id, "")?;
            setup.install_config_on(&mut probe, &mode.id, mode.horizon_ns, &config, mode_world)?;
            physics.install(&mut probe)?;
        } else {
            setup.install(&mut probe, &mode.id, mode.horizon_ns, "")?;
        }
        let receipt = installation
            .map(|(sample, resolved, settings)| {
                let (resolved, settings) = mode_physics
                    .map_or((resolved, settings), |p| (&p.resolved, &p.settings));
                sample.install(&mut probe, mode_world, resolved, settings)
            })
            .transpose()?;
        let inventory_source = mode_physics.map(|p| &p.resolved).or(inventory_world.as_ref());
        if let (Some(audit), Some(source_world)) = (audit.as_mut(), inventory_source) {
            audit.inventory(
                mode.id.clone(),
                campaign::installation::inventory(
                    mode_world,
                    source_world,
                    &probe,
                    serde_json::Value::Null,
                    setup.sampled.keys().map(|name| {
                        (sim_task::verdict::ComponentKind::Battery, name.clone())
                    }),
                ),
            )?;
        }
        let control = control::run(&probe).ok_or("контур не установлен")?;
        let candidate = topology(mode_world, &control, &profile)?;
        let encoded = serde_json::to_vec(&candidate).map_err(|error| error.to_string())?;
        if resolved_topology
            .as_ref()
            .is_some_and(|(first, _)| *first != encoded)
        {
            return Err("топология различается между режимами".into());
        }
        resolved_topology = Some((encoded, candidate));
        wires = resolved_wiring(&control);
        if options.matrix_full {
            mode_wires.insert(mode.id.clone(), wires.clone());
        }
        let identity = control.identity();
        installed_inputs.insert(
            mode.id.clone(),
            InstalledInputs {
                world: Fingerprint::from_fnv1a(identity.world),
                wiring: Fingerprint::from_fnv1a(identity.control),
                assets: identity
                    .assets
                    .iter()
                    .map(|(name, fingerprint)| {
                        (name.clone(), Fingerprint::from_fnv1a(*fingerprint))
                    })
                    .collect(),
                installation: receipt,
            },
        );
        build = Some(identity.build.clone());
        catalog = Some(identity.catalog);
        frame(&mut wiring, mode.id.as_bytes());
        if let Some(overlay) = setup.overlays.get(&mode.id) {
            frame(&mut wiring, overlay.as_bytes());
        }
        for (path, fingerprint) in &identity.assets {
            frame(&mut resource_bytes, path.as_bytes());
            frame(&mut resource_bytes, &fingerprint.to_le_bytes());
        }
        expected.insert(mode.id.clone(), identity.assets.clone());
    }
    let (_, topology) = resolved_topology.ok_or("задача не содержит ни одного режима")?;
    if options.matrix_full {
        task.expand_faults(&topology)
            .map_err(|error| error.to_string())?;
    }
    let inputs = Inputs {
        build: build.expect("установлен режим"),
        catalog: Fingerprint::from_fnv1a(catalog.expect("установлен режим")),
        wiring: Fingerprint::of(&wiring),
        program: Fingerprint::of(&resource_bytes),
        configuration: Fingerprint::of(&wiring),
        firmware_image: None,
        legacy_design: None,
    };
    let assessor = Assessor::new(task, topology, world.clone(), inputs.clone())
        .and_then(|assessor| assessor.with_mode_worlds(
            physics.iter().map(|(mode, entry)| (mode.clone(), entry.world.clone())).collect()
        )).map_err(|error| error.to_string())?;
    if !profile.hazard_witnesses.is_empty() && !options.matrix_full {
        return Err("предметные свидетели контура требуют --matrix full".into());
    }
    for witness in &profile.hazard_witnesses {
        witness.validate(&assessor)?;
    }
    if options
        .case
        .as_ref()
        .is_some_and(|mode| !assessor.topology().required_modes.contains(mode))
    {
        return Err("режим отсутствует в профиле".into());
    }
    if options.exports_run() && options.case.is_none() {
        return Err("для выгрузки журнала задачи выберите режим через --case".into());
    }
    let execution = Execution {
        setup: &setup,
        assessor: &assessor,
        inputs: &inputs,
        assets: &expected,
        references: &profile.reference,
        reference_texts: &reference_texts,
        sample: installation,
        witnesses: &profile.hazard_witnesses,
        physics: &physics,
        coverage_control: options.coverage_control,
    };
    if options.matrix_full {
        let plan = matrix::Plan::new(&execution, &profile, &mode_wires, options.selected_control_case.as_ref())?;
        if audit
            .as_mut()
            .is_some_and(|audit| audit.omit_unselected(assessor.required_cases().len()))
        {
            return Ok(0);
        }
        let (evidence, complete, comparisons, coverage, observations) =
            plan.run(&execution, &installed_inputs, options)?;
        if let Some(audit) = audit.as_mut() {
            physics_files.verify()?;
            audit.record_files(&physics_files)?;
            audit.record_inputs((assessor.identity(), &installed_inputs, &reference_texts))?;
            audit.control(&evidence.cases, evidence.outcome, execution.witnesses, &observations, &comparisons, &assessor, world.seed, world.physics.as_hz() as u64)?;
        }
        return write_task_report(
            &Report {
                status: if complete { "completed" } else { "incomplete" },
                topology: assessor.topology(),
                resolved_wiring: wires,
                installed_inputs,
                closed_loop: evidence.outcome,
                coverage,
                evidence: &evidence,
                reference: comparisons,
            },
            &evidence.cases,
            options,
            out,
        );
    }

    let mut runs = Vec::new();
    let mut exported = false;
    let mut comparisons = BTreeMap::new();
    for case in assessor.required_cases().iter().filter(|case| {
        options.case.as_ref().is_none_or(|mode| mode == &case.mode) && assessor.plan(case).is_some()
    }) {
        let result = execution.run_case(case, options.view.is_some() && !exported, |runtime| {
            if !exported && case.fault.is_none() {
                crate::view::save(options, runtime, Some(&case.mode), Some(case))?;
                let log = runtime.log();
                if let Some(path) = &options.log {
                    export(path, |file| log.write_binary(file))?;
                }
                if let Some(path) = &options.csv {
                    export(path, |file| log.write_csv(file))?;
                }
                exported = true;
            }
            Ok(())
        })?;
        if let Some(comparison) = result.comparison {
            comparisons.insert(case.mode.clone(), comparison);
        }
        runs.push(result.run);
    }
    if options.view.is_some() && !exported {
        return Err("для просмотра нет завершившего наблюдение штатного прогона".into());
    }
    let evidence = assessor.assess(&runs);
    if let Some(audit) = audit.as_mut() {
        audit.record_inputs((assessor.identity(), &installed_inputs, &reference_texts))?;
        audit.control(evidence.cases(), evidence.outcome(), &[], &BTreeMap::new(), &comparisons, &assessor, world.seed, world.physics.as_hz() as u64)?;
    }
    let complete = !runs.is_empty() && runs.iter().all(|result| result.report().completed);
    write_task_report(
        &Report {
            status: if complete { "completed" } else { "incomplete" },
            topology: assessor.topology(),
            resolved_wiring: wires,
            installed_inputs,
            closed_loop: evidence.outcome(),
            coverage: matrix::Coverage::from_cases(evidence.cases()),
            evidence: &evidence,
            reference: comparisons,
        },
        evidence.cases(),
        options,
        out,
    )
}
