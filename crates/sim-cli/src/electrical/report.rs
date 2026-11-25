//! Связь разрешённой схемы, сборочного листа и записи опыта.

use std::{
    collections::{BTreeMap, BTreeSet},
    io::Write,
    path::{Path, PathBuf},
};

use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sim_process::{
    Analysis, Interval,
    schematic::{
        AssemblySheet, DepthEvidence, DepthFinding, Endpoint, Expected, Parameter, ResetProfile,
        Section, Stage, Status,
    },
};
use sim_runtime::{Runtime, control};
use sim_signal::Time;
use sim_task::{
    Fingerprint, Outcome,
    verdict::{ElectricalEvidence, Finding, SimulationVerdict},
};

use super::{
    LiveExperiment, LivePath, LiveSummary,
    input::{CatalogInput, ProfileInput, SchematicInput, parse},
    install::{BoundLoad, Wiring, bound_loads, validate_owners},
};
use crate::tasks::processes::{Inputs, capture_world};

pub(crate) struct Installation {
    files: Inputs,
    schemes: BTreeMap<String, SchematicInput>,
    depth: BTreeMap<String, BTreeMap<String, DepthFinding>>,
    missing: BTreeSet<String>,
    profile: Option<Fingerprint>,
    installed: Option<control::RunIdentity>,
    resets: BTreeMap<String, ResetProfile>,
    loads: BTreeMap<String, BTreeMap<String, BoundLoad>>,
}

#[derive(Serialize)]
pub(crate) struct ElectricalReport {
    version: u32,
    build: Fingerprint,
    sources: BTreeMap<String, Fingerprint>,
    live: Option<LiveAudit>,
    simulation: SimulationVerdict,
    findings: Vec<Finding>,
    assemblies: BTreeMap<String, AssemblyReport>,
}

impl ElectricalReport {
    pub(crate) fn outcome(&self) -> Outcome {
        match self.simulation {
            SimulationVerdict::Rejected => Outcome::Fail,
            SimulationVerdict::NotProven => Outcome::NotAnalyzed,
            SimulationVerdict::PassedSimulation => Outcome::Pass,
        }
    }
}

#[derive(Serialize)]
struct LiveAudit {
    engine_build: String,
    world: Fingerprint,
    installation: Fingerprint,
    control: Fingerprint,
    profile: Fingerprint,
    assets: Vec<(String, u64)>,
    seed: u64,
    required_ns: u64,
    observed_ns: u64,
    complete: bool,
    journal: Fingerprint,
    summary: LiveSummary,
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    loads: BTreeMap<String, BTreeMap<String, BoundLoad>>,
}

#[derive(Serialize)]
struct AssemblyReport {
    schematic: Fingerprint,
    build: Fingerprint,
    journal: Option<Fingerprint>,
    proof_scope: &'static str,
    sheet: Value,
    modes: Vec<Value>,
    absent_modes: BTreeMap<u8, String>,
    inapplicable: BTreeMap<&'static str, String>,
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    depth_checks: BTreeMap<String, Value>,
}

pub(crate) struct Evaluated {
    pub evidence: BTreeMap<String, ElectricalEvidence>,
    pub report: ElectricalReport,
}

impl Installation {
    pub(crate) fn load(
        world: &Path,
        catalog: &Path,
        profile: Option<&Path>,
    ) -> Result<Self, String> {
        let mut files = Inputs::default();
        let parsed = capture_world(world, &mut files)?;
        let base = dir(world);
        let mut paths = BTreeMap::new();
        let mut missing = BTreeSet::new();
        for instance in parsed.devices {
            let path = base.join(&instance.file);
            let device = sim_world::load_device(&path, &sim_world::LegacyCatalog)?;
            if let Some(scheme) = device.schematic {
                paths.insert(instance.name, dir(&path).join(scheme));
            } else if !device.drives.is_empty()
                || !device.sensors.is_empty()
                || device
                    .links
                    .iter()
                    .flat_map(|l| &l.mounts)
                    .any(|m| m.kind == sim_world::MountKind::Power)
            {
                missing.insert(instance.name);
            }
        }
        let inputs = profile
            .map(|path| ProfileInput::load(path, &mut files, paths.keys()))
            .transpose()?;
        let stamp = inputs
            .as_ref()
            .map(|profile| Fingerprint::of(&profile.bytes));
        let mut resets = BTreeMap::new();
        if let Some(profile) = &inputs {
            let (control_path, bytes) = profile.control(&mut files)?;
            let wiring: Wiring = parse(&control_path, &bytes)?;
            for cfg in profile.data.circuits.values() {
                validate_owners(cfg)?;
                for (name, supply) in &cfg.boards {
                    let path = wiring.board_profile(name, catalog)?;
                    if supply.reference_reset.is_none()
                        || path
                            .try_exists()
                            .map_err(|err| format!("{}: {err}", path.display()))?
                    {
                        let board =
                            CatalogInput::parse_board_profile(&path, files.capture(&path)?)?;
                        if board.board != wiring.board(name)? {
                            return Err("электрический профиль относится к другой плате".into());
                        }
                        if supply.reference_reset.is_none() {
                            resets.insert(name.clone(), board.reset);
                        }
                    }
                }
                if let Some(path) = cfg
                    .diagnostic_connections
                    .as_ref()
                    .and_then(|d| d.parameters.as_ref())
                {
                    files.capture(&profile.dir.join(path))?;
                }
            }
            for path in profile
                .data
                .measurements
                .iter()
                .chain(profile.data.sampled.values())
            {
                files.capture(&profile.dir.join(path))?;
            }
        }
        let mut schemes = BTreeMap::new();
        let mut loads = BTreeMap::new();
        let mut depth = BTreeMap::new();
        if !paths.is_empty() {
            let catalog = CatalogInput::for_profile(catalog, inputs.as_ref(), &mut files)?;
            let parts = &catalog.catalog;
            let ratings = catalog.depth_ratings()?;
            for (instance, path) in paths {
                let bytes = files.capture(&path)?.to_vec();
                let scheme = SchematicInput::parse(&path, bytes, parts)?;
                let evaluated = scheme.assess_depth_with(&ratings)?;
                if !evaluated.is_empty() {
                    depth.insert(instance.clone(), evaluated);
                }
                if let Some(profile) = &inputs {
                    let cfg = &profile.data.circuits[&instance];
                    if !cfg.loads.is_empty() {
                        let net = scheme.live_netlist(
                            parts,
                            cfg.diagnostic_connections.as_ref(),
                            &profile.dir,
                            &mut files,
                        )?;
                        let loads_at_instance =
                            bound_loads(&instance, &cfg.loads, &net, &scheme.definition()?, parts)
                                .map_err(|err| format!("{}: {err}", path.display()))?;
                        loads.insert(instance.clone(), loads_at_instance);
                    }
                }
                schemes.insert(instance, scheme);
            }
        }
        Ok(Self {
            files,
            schemes,
            depth,
            missing,
            profile: stamp,
            installed: None,
            resets,
            loads,
        })
    }

    pub(crate) fn bind_live(&mut self, runtime: &Runtime, horizon: Time) -> Result<(), String> {
        let run = control::run(runtime).ok_or("нет установленного электрического контура")?;
        let identity = run.identity();
        if self.installed.is_some()
            || run.end().started
            || self.profile.map(Fingerprint::as_u64) != Some(identity.task)
            || identity.task != identity.criteria
            || identity.phase_horizon != horizon
            || identity.seed != runtime.seed()
        {
            return Err(
                "контур должен быть связан с электрическими входами до первого шага".into(),
            );
        }
        for load in self.loads.values().flat_map(BTreeMap::values) {
            for (name, unit) in [(&load.voltage_channel, "V"), (&load.current_channel, "A")] {
                if runtime
                    .truth_id(name)
                    .and_then(|id| runtime.truth_spec(id))
                    .is_none_or(|spec| spec.unit != unit)
                {
                    return Err(format!("отсутствует фактическая проба «{name}»"));
                }
            }
        }
        self.files.verify()?;
        self.installed = Some(identity.clone());
        Ok(())
    }

    pub(crate) fn verify(&self) -> Result<(), String> {
        self.files.verify()
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.schemes.is_empty() && self.missing.is_empty()
    }

    pub(crate) fn fingerprints(&self) -> BTreeMap<String, Fingerprint> {
        self.files.fingerprints()
    }

    pub(crate) fn evaluate(
        &self,
        build: Fingerprint,
        runtime: Option<&Runtime>,
        horizon: Option<Time>,
    ) -> Result<Evaluated, String> {
        let live = match (runtime, self.profile, horizon) {
            (None, _, _) => None,
            (Some(rt), Some(profile), Some(horizon)) => {
                let run =
                    control::run(rt).ok_or("нет живого журнала управления электрического опыта")?;
                let id = run.identity();
                if self.installed.as_ref() != Some(id)
                    || id.task != profile.as_u64()
                    || id.criteria != profile.as_u64()
                    || id.phase_horizon != horizon
                    || id.seed != rt.log().header().seed
                {
                    return Err(
                        "живой журнал относится к другому профилю, горизонту или зерну".into(),
                    );
                }
                let end = run.end();
                Some(LiveAudit {
                    engine_build: id.build.clone(),
                    world: Fingerprint::from_fnv1a(id.world),
                    installation: Fingerprint::from_fnv1a(id.catalog),
                    control: Fingerprint::from_fnv1a(id.control),
                    profile,
                    assets: id.assets.clone(),
                    seed: id.seed,
                    required_ns: horizon.0,
                    observed_ns: end.time.0,
                    complete: run.reached_horizon() && end.steps > 0,
                    journal: Fingerprint::from_fnv1a(run.log().fingerprint()),
                    summary: LiveSummary::from(rt),
                    loads: self.loads.clone(),
                })
            }
            _ => return Err("журнал требует разрешённого профиля и горизонта".into()),
        };
        let mut evidence = BTreeMap::new();
        let mut assemblies = BTreeMap::new();
        let mut findings = Vec::new();
        for (instance, input) in &self.schemes {
            let mut result = ElectricalEvidence::evaluate(
                instance,
                &input.bytes,
                &input.compiled,
                &input.modes,
                build,
            )
            .map_err(|e| e.to_string())?;
            if let (Some(runtime), Some(live), Some(horizon)) = (runtime, &live, horizon) {
                result
                    .observe_runtime(runtime, live.profile, horizon, &self.resets)
                    .map_err(|err| err.to_string())?;
            }
            let assessment = result.assessment();
            findings.extend_from_slice(result.findings());
            assemblies.insert(
                instance.clone(),
                AssemblyReport {
                    schematic: Fingerprint::of(&input.bytes),
                    build,
                    journal: live.as_ref().map(|run| run.journal),
                    proof_scope: match assessment.scope {
                        sim_process::schematic::Scope::Assembly => "сборка",
                        sim_process::schematic::Scope::Reference => "эталонный стенд",
                    },
                    sheet: sheet(input.compiled.assembly_sheet(), &input.crimps),
                    modes: assessment
                        .modes
                        .iter()
                        .map(|mode| {
                            json!({
                                    "number": mode.mode.number(),
                                    "mode": format!("{:?}", mode.mode),
                                    "end_ns": mode.horizon.map(|t| t.0),
                                    "nominal": mode.nominal,
                                    "envelope": mode.envelope,
                            "journal": result.mode_journal(mode),
                                    "reason": mode.reason,
                                })
                        })
                        .collect(),
                    absent_modes: assessment.absent_modes.clone(),
                    inapplicable: assessment
                        .inapplicable
                        .iter()
                        .map(|(class, reason)| (class.code(), reason.clone()))
                        .collect(),
                    depth_checks: self
                        .depth
                        .get(instance)
                        .into_iter()
                        .flatten()
                        .map(|(id, finding)| {
                            (
                                id.clone(),
                                depth_report(finding, build, Fingerprint::of(&input.bytes)),
                            )
                        })
                        .collect(),
                },
            );
            evidence.insert(instance.clone(), result);
        }
        for instance in &self.missing {
            findings.extend(ElectricalEvidence::missing(
                instance,
                "нет схемы экземпляра",
            ));
        }
        let depth_outcomes =
            self.depth
                .values()
                .flat_map(|checks| checks.values())
                .map(|finding| match finding.status {
                    Status::Pass => Outcome::Pass,
                    Status::Fail => Outcome::Fail,
                    Status::NotAnalyzed => Outcome::NotAnalyzed,
                });
        let simulation = match Outcome::aggregate(
            findings
                .iter()
                .map(|finding| finding.outcome)
                .chain(depth_outcomes),
        ) {
            Outcome::Fail => SimulationVerdict::Rejected,
            Outcome::NotAnalyzed => SimulationVerdict::NotProven,
            Outcome::Pass => SimulationVerdict::PassedSimulation,
        };
        self.files.verify()?;
        Ok(Evaluated {
            evidence,
            report: ElectricalReport {
                version: 1,
                build,
                sources: self.fingerprints(),
                live,
                simulation,
                findings,
                assemblies,
            },
        })
    }
}

fn dir(path: &Path) -> &Path {
    path.parent().unwrap_or_else(|| Path::new("."))
}

fn endpoint(end: &Endpoint) -> Value {
    json!({ "component": end.component, "pin": end.pin })
}

fn parameter(value: &Parameter, unit: &str) -> Value {
    use sim_world::Provenance;
    json!({
        "unit": unit,
        "measurement": value.measurement,
        "range": value.range.as_ref().map(|range| {
            let (kind, reference) = match &range.provenance {
                Provenance::Datasheet(s) => ("datasheet", s),
                Provenance::Paper(s) => ("paper", s),
                Provenance::Measured(s) => ("measured", s),
                Provenance::Derived(s) => ("derived", s),
                Provenance::Estimate(s) => ("estimate", s),
            };
            json!({
                "min": range.min, "nominal": range.nominal, "max": range.max,
                "source": { "kind": kind, "reference": reference },
                "supports_proof": range.can_prove(),
            })
        }),
    })
}

fn sheet(sheet: AssemblySheet, crimps: &BTreeSet<String>) -> Value {
    json!({
        "parts": sheet.parts.into_iter().map(|part| json!({
            "component": part.component, "part": part.part,
            "kind": format!("{:?}", part.kind), "source": part.source,
            "pins": part.pins, "missing_parameters": part.missing_parameters,
            "model_gap": part.model_gap,
        })).collect::<Vec<_>>(),
        "operations": sheet.operations.into_iter().map(|op| json!({
            "id": op.id,
            "kind": if crimps.contains(&op.id) { "Crimp".to_owned() } else { format!("{:?}", op.kind) },
            "ends": op.ends.each_ref().map(endpoint),
            "after": op.after, "compatible": op.compatible,
        })).collect::<Vec<_>>(),
        "bindings": sheet.bindings.into_iter().map(|binding| json!({
            "target": format!("{:?}", binding.target),
            "ends": binding.ends.iter().map(endpoint).collect::<Vec<_>>(),
        })).collect::<Vec<_>>(),
        "inspections": sheet.inspections.into_iter().map(|inspection| {
            let (kind, limit) = match &inspection.expected {
                Expected::ResistanceOhm(x) => ("resistance", Some(parameter(x, "Ω"))),
                Expected::NoDirectBridge => ("no_direct_bridge", None),
                Expected::PositiveMark => ("positive_mark", None),
                Expected::ReverseVoltageV(x) => ("reverse_voltage", Some(parameter(x, "V"))),
                Expected::CurrentLimitA(x) => ("current_limit", Some(parameter(x, "A"))),
                Expected::Continuity => ("continuity", None),
                Expected::SignalVoltage => ("signal_voltage", None),
            };
            json!({
                "subject": inspection.subject, "test": kind, "limit": limit,
                "stage": match inspection.stage {
                    Stage::BeforePower => "before_power",
                    Stage::CurrentLimitedStart => "current_limited_start",
                },
                "ends": inspection.ends.each_ref().map(endpoint), "conditions": inspection.conditions,
            })
        }).collect::<Vec<_>>(),
    })
}

pub(crate) fn build(installation: &Installation) -> Result<Fingerprint, String> {
    let (engine, engine_hash) = Runtime::physics_engine_identity()?;
    let binary = std::env::current_exe().map_err(|e| e.to_string())?;
    let bytes = std::fs::read(&binary).map_err(|e| format!("{}: {e}", binary.display()))?;
    Ok(Fingerprint::of(
        format!(
            "{engine:?}{engine_hash:?}{:?}{:?}",
            Fingerprint::of(&bytes),
            installation.fingerprints()
        )
        .as_bytes(),
    ))
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Campaign {
    electrical_format: u32,
    catalog: PathBuf,
    assemblies: BTreeMap<String, Experiment>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Experiment {
    world: PathBuf,
    profile: Option<PathBuf>,
    horizon_ns: Option<u64>,
}

impl Campaign {
    pub(crate) fn run(
        self,
        path: &Path,
        options: &crate::Options,
        out: &mut impl Write,
    ) -> Result<u8, String> {
        if self.electrical_format != 1 || self.assemblies.is_empty() {
            return Err("нужен непустой электрический манифест версии 1".into());
        }
        if options.checkpoint.is_some()
            || options.repeat.is_some()
            || options.measurements.is_some()
        {
            return Err(
                "электрический манифест выполняет каждый заданный опыт из исходных данных".into(),
            );
        }
        let root = dir(path);
        let catalog = root.join(self.catalog);
        let mut reports = BTreeMap::new();
        for (id, experiment) in self.assemblies {
            if id.trim().is_empty() || id.chars().any(char::is_control) {
                return Err("нужно имя сборки".into());
            }
            let world = root.join(experiment.world);
            let profile = experiment.profile.as_ref().map(|p| root.join(p));
            let horizon = experiment.horizon_ns.map(Time);
            let mut installation = Installation::load(&world, &catalog, profile.as_deref())?;
            if installation.is_empty() {
                installation.missing.insert(id.clone());
            }
            let runtime = match (profile, horizon) {
                (Some(profile), Some(end)) if end > Time::ZERO => {
                    let mut run = LiveExperiment::load(
                        &world,
                        &profile,
                        &catalog,
                        None,
                        end,
                        LivePath::Full,
                    )?;
                    installation.bind_live(&run.runtime, end)?;
                    run.run_until(end)?;
                    Some(run.runtime)
                }
                (None, None) => None,
                _ => {
                    return Err(format!(
                        "«{id}»: для живого опыта нужны профиль и положительный горизонт"
                    ));
                }
            };
            let result = installation.evaluate(build(&installation)?, runtime.as_ref(), horizon)?;
            reports.insert(id, result.report);
        }
        if options.json {
            serde_json::to_writer(&mut *out, &json!({ "version": 1, "assemblies": reports }))
                .map_err(|err| err.to_string())?;
            writeln!(out).map_err(|e| e.to_string())?;
        } else {
            for (id, report) in reports {
                writeln!(out, "{id}: {:?}", report.simulation).map_err(|e| e.to_string())?;
            }
        }
        Ok(0)
    }
}

fn depth_report(finding: &DepthFinding, build: Fingerprint, input: Fingerprint) -> Value {
    json!({
        "code": finding.code,
        "status": depth_status(finding.status),
        "reason": finding.reason,
        "scope": match finding.scope {
            sim_process::schematic::Scope::Reference => "reference",
            sim_process::schematic::Scope::Assembly => "assembly",
        },
        "mode": format!("{:?}", finding.mode),
        "mode_number": finding.mode.number(),
        "branch": depth_branch(&finding.branch),
        "window": finding.window.map(depth_window),
        "event": finding.event.as_ref().map(|event| json!({
            "kind": event.kind.code(),
            "time_ns": event.at.0,
            "branch": depth_branch(&event.branch),
            "source": provenance(&event.source),
        })),
        "applicability": finding.applicability.as_ref().map(provenance),
        "applicability_gap": finding.applicability_gap,
        "diagnostic_only": finding.applicability_gap.is_some(),
        "passport": finding.passport,
        "build": build,
        "observation_input": input,
        "metrics": finding.evidence.as_ref().map(depth_metrics),
    })
}

fn depth_branch(branch: &Section) -> Value {
    match branch {
        Section::Connection(id) => json!({ "kind": "connection", "id": id }),
        Section::Element { component, element } => {
            json!({ "kind": "element", "component": component, "element": element })
        }
    }
}

fn depth_window(window: Interval) -> Value {
    json!({ "start_ns": window.start.0, "end_ns": window.end.0 })
}

fn depth_status(status: Status) -> &'static str {
    match status {
        Status::Pass => "pass",
        Status::Fail => "fail",
        Status::NotAnalyzed => "not_analyzed",
    }
}

fn provenance(source: &sim_world::Provenance) -> Value {
    use sim_world::Provenance::*;
    let (kind, reference) = match source {
        Datasheet(value) => ("datasheet", value),
        Paper(value) => ("paper", value),
        Measured(value) => ("measured", value),
        Derived(value) => ("derived", value),
        Estimate(value) => ("estimate", value),
    };
    json!({ "kind": kind, "reference": reference })
}

fn depth_analysis<T>(analysis: &Analysis<T>, value: impl FnOnce(&T) -> Value) -> Value {
    match analysis {
        Analysis::Analyzed {
            value: result,
            scope,
        } => {
            json!({ "value": value(result), "scope": scope })
        }
        Analysis::NotAnalyzed { reason } => json!({ "status": "not_analyzed", "reason": reason }),
    }
}

fn depth_metrics(evidence: &DepthEvidence) -> Value {
    use sim_process::schematic::{
        capacitor_stress::{CapacitorBounds, CapacitorMargin},
        sleep_drain::SleepDrainBounds,
        switch_thermal::{SwitchBounds, SwitchMargin, SwitchSoaTime},
    };
    match evidence {
        DepthEvidence::CapacitorStress(e) => {
            let bounds = |b: &CapacitorBounds| json!([b.min, b.max]);
            let margin = |m: &CapacitorMargin| {
                json!({
                    "observed": bounds(&m.observed), "limit": bounds(&m.limit), "margin": bounds(&m.margin),
                })
            };
            json!({
                "terminals": e.terminals.each_ref().map(endpoint),
                "voltage_peak_sample_ns": e.voltage_peak_sample.map(|t| t.0),
                "voltage_v": depth_analysis(&e.voltage_v, margin),
                "ripple_a": depth_analysis(&e.ripple_a, margin),
                "loss_w": depth_analysis(&e.loss_w, bounds),
                "hotspot_k": depth_analysis(&e.hotspot_k, margin),
            })
        }
        DepthEvidence::FuseCoordination(e) => json!({
            "wire": depth_branch(&e.wire_branch),
            "switch": depth_branch(&e.switch_branch),
            "interrupt_current": depth_status(e.interrupt_current),
            "interrupt_voltage": depth_status(e.interrupt_voltage),
            "clearing_time_s": depth_analysis(&e.clearing_time_s, |v| json!([v.min, v.max])),
            "wire_damage": depth_analysis(&e.wire, |v| json!({
                "window_i2t_a2s": [v.window_i2t_a2s.min, v.window_i2t_a2s.max],
                "damage_time_s": [Some(v.damage_time_s.earliest_s), v.damage_time_s.latest_s],
            })),
            "switch_damage_time_s": depth_analysis(&e.switch, |v| json!([Some(v.earliest_s), v.latest_s])),
            "wire_margin_s": e.wire_margin_s.map(|v| json!([Some(v.min), v.max])),
            "switch_margin_s": e.switch_margin_s.map(|v| json!([Some(v.min), v.max])),
        }),
        DepthEvidence::SwitchThermal(e) => {
            let bounds = |b: &SwitchBounds| json!([b.min, b.max]);
            let margin = |m: &SwitchMargin| {
                json!({
                    "observed": bounds(&m.observed), "limit": bounds(&m.limit), "margin": bounds(&m.margin),
                })
            };
            let soa_time = |t: SwitchSoaTime| match t {
                SwitchSoaTime::Seconds(value) => json!({ "seconds": value }),
                SwitchSoaTime::Continuous => json!({ "continuous": true }),
            };
            json!({
                "terminals": e.terminals.each_ref().map(endpoint),
                "sustained": e.sustained,
                "equilibrium_power_w": depth_analysis(&e.equilibrium_power_w, bounds),
                "steady_junction_k": depth_analysis(&e.steady_junction_k, margin),
                "foster_equilibrium_junction_k": depth_analysis(&e.foster_equilibrium_junction_k, margin),
                "foster_peak_junction_k": depth_analysis(&e.foster_peak_junction_k, margin),
                "agreement_k": depth_analysis(&e.agreement_k, margin),
                "linear_soa": depth_analysis(&e.linear_soa, |pulses| json!(pulses.iter().map(|p| json!({
                    "window": depth_window(p.window), "duration_s": bounds(&p.duration_s),
                    "closed": p.closed, "vds_v": bounds(&p.vds_v), "id_a": bounds(&p.id_a),
                    "case_k": bounds(&p.case_k), "allowed": depth_analysis(&p.allowed, |a| json!(a.map(soa_time))),
                    "margin_s": depth_analysis(&p.margin_s, bounds),
                    "status": depth_status(p.status), "reason": p.reason,
                })).collect::<Vec<_>>())),
            })
        }
        DepthEvidence::DeepDischarge(e) => json!({
            "cutoff": depth_analysis(&e.cutoff, |v| json!({
                "detected_at_ns": v.detected_at.0, "disconnected_at_ns": v.disconnected_at.0,
                "observed_delay_ns": v.observed_delay.0,
                "allowed_delay_s": [v.allowed_delay_s.min, v.allowed_delay_s.max],
            })),
            "limiting_cell": e.limiting_cell,
            "cells": e.cells.iter().map(|v| json!({
                "id": v.id,
                "minimum_voltage_v": depth_analysis(&v.minimum_voltage_v, |b| json!([b.min, b.max])),
                "margin_v": depth_analysis(&v.margin_v, |b| json!([b.min, b.max])),
                "violation": v.violation.map(|b| json!({
                    "time_ns": b.time.0, "voltage_v": [b.voltage_v.min, b.voltage_v.max],
                    "limit_v": [b.limit_v.min, b.limit_v.max],
                })),
                "final_model_voltage_v": depth_analysis(&v.final_model_voltage_v, |b| json!([b.min, b.max])),
                "removed_charge_ah": depth_analysis(&v.removed_charge_ah, |b| json!([b.min, b.max])),
            })).collect::<Vec<_>>(),
        }),
        DepthEvidence::SleepDrain(e) => {
            let bounds = |b: &SleepDrainBounds| json!([b.min, b.max]);
            json!({
                "consumption": depth_analysis(&e.consumption, |q| json!({
                    "controller_ah": bounds(&q.controller_ah), "other_loads_ah": bounds(&q.other_loads_ah),
                    "self_discharge_ah": bounds(&q.self_discharge_ah), "total_ah": bounds(&q.total_ah),
                })),
                "charge_margin_ah": depth_analysis(&e.charge_margin_ah, bounds),
                "cutoff_margin_v": depth_analysis(&e.cutoff_margin_v, bounds),
                "cutoff_violation": e.cutoff_violation.as_ref().map(|v| json!({
                    "state": v.state, "time_ns": v.time.0, "margin_v": bounds(&v.margin_v),
                })),
            })
        }
        DepthEvidence::CableFlex(e) => {
            use sim_process::work::cable_flex::{CablePeak, CableRadiusBounds};
            let peak = |p: &CablePeak| {
                json!({
                    "bounds": [p.bounds.min, p.bounds.max], "sample_at_ns": p.sample_at.0,
                })
            };
            // Пустая граница радиуса означает прямой участок, как в контракте модели.
            let radius = |r: &CableRadiusBounds| json!({ "min_m": r.min_m, "max_m": r.max_m });
            json!({
                "maximum_length_m": depth_analysis(&e.maximum_length_m, peak),
                "maximum_curvature_inv_m": depth_analysis(&e.maximum_curvature_inv_m, peak),
                "minimum_radius_m": depth_analysis(&e.minimum_radius_m, radius),
                "relief": e.relief.map(depth_status),
                "slack": { "status": depth_status(e.slack.status),
                    "result": depth_analysis(&e.slack.result, |r| json!([r.min, r.max])) },
                "bend": { "status": depth_status(e.bend.status),
                    "result": depth_analysis(&e.bend.result, radius) },
                "fatigue": { "status": depth_status(e.fatigue.status),
                    "result": depth_analysis(&e.fatigue.result, radius) },
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::electrical::load_catalog;

    // Окно 1 мс и плоское напряжение. Для заданного кусочно-линейного
    // периодического тока от −0,5 до +0,5 А, интеграл I² за период даёт RMS² = 1/12 А².
    // Паспорт из каталога RC-стенда даёт 9 В, 1 Ом и предел 303 К.
    const SCHEME: &str = r#"
format = 1
name = "Блок"
scope = "reference"
nets = ["a", "b", "d", "g", "s"]
[components."блок"]
part = "capacitor"
nets = { a = "a", b = "b" }
[components."контакт"]
part = "wire_plug_socket"
nets = { a = "a", b = "b" }
[components."ключ"]
part = "nmos"
nets = { d = "d", g = "g", s = "s" }
[components."защита"]
part = "fuse_series"
nets = { input = "d", output = "a" }
[[modes.cases]]
mode = { kind = "normal" }
source = { derived = "Аналитический стенд блока" }
probes = [{ kind = "current", branch = { kind = "element", component = "блок", element = "c" } }]
"#;

    const CAPACITOR: &str = r#"
[[depth_checks]]
id = "конденсатор"
mode = { kind = "normal" }
branch = { kind = "element", component = "блок", element = "c" }
window = { start_ns = 0, end_ns = 1000000 }
applicability = { derived = "Аналитическая постановка блока и условия стендового предела на всём окне" }
[depth_checks.check]
kind = "capacitor_stress"
terminals = ["блок/a", "блок/b"]
voltage_sense = "forward"
rating = "capacitor_rc_bench"
series = { time_ns = [0, 500000, 1000000], voltage_v = [8.0, 8.0, 8.0], current_a = [-0.5, 0.5, -0.5] }
voltage_error_v = { min = 0.0, nominal = 0.0, max = 0.0, source = { derived = "Напряжение константно по постановке" } }
current_error_a = { min = 0.0, nominal = 0.0, max = 0.0, source = { derived = "Ток задан точно кусочно-линейной функцией" } }
frequency_hz = { min = 1000.0, nominal = 1000.0, max = 1000.0, source = { derived = "Стендовый частотный эквивалент с постоянными ESR и пределом RMS" } }
temperatures = { ambient_k = { min = 300.0, nominal = 300.0, max = 300.0, source = { derived = "Ta стенда постоянна" } } }
thermal = { kind = "steady_state", thermal_resistance_k_per_w = { min = 2.0, nominal = 2.0, max = 2.0, source = { derived = "Стационарное θ RC-стенда" } }, peak_above_mean_k = { min = 0.0, nominal = 0.0, max = 0.0, source = { derived = "Тепловая постановка RC-стенда с усреднением по периоду" } } }
"#;

    const OPEN_CHECKS: &str = r#"
[[depth_checks]]
id = "защита"
mode = { kind = "normal" }
branch = { kind = "element", component = "защита", element = "closed" }
window = { start_ns = 0, end_ns = 1000000 }
applicability = { derived = "Стендовая привязка" }
[depth_checks.check]
kind = "fuse_coordination"
wire = { kind = "element", component = "контакт", element = "contact" }
switch = { kind = "element", component = "ключ", element = "m" }
rating = "fuse_series_pending"

[[depth_checks]]
id = "ключ"
mode = { kind = "normal" }
branch = { kind = "element", component = "ключ", element = "m" }
window = { start_ns = 0, end_ns = 1000000 }
applicability = { derived = "Стендовая привязка" }
[depth_checks.check]
kind = "switch_thermal"
terminals = ["ключ/d", "ключ/s", "ключ/g"]
rating = "nmos_pending"

[[depth_checks]]
id = "разряд"
mode = { kind = "normal" }
branch = { kind = "element", component = "контакт", element = "contact" }
window = { start_ns = 0, end_ns = 1000000 }
applicability = { derived = "Стендовая привязка" }
check = { kind = "deep_discharge" }

[[depth_checks]]
id = "сон"
mode = { kind = "normal" }
branch = { kind = "element", component = "контакт", element = "contact" }
applicability = { derived = "Стендовая привязка" }
check = { kind = "sleep_drain" }

[[depth_checks]]
id = "изгиб"
mode = { kind = "normal" }
branch = { kind = "element", component = "контакт", element = "contact" }
window = { start_ns = 0, end_ns = 1000000 }
applicability = { derived = "Стендовая привязка" }
check = { kind = "cable_flex" }
"#;

    fn catalog() -> CatalogInput {
        load_catalog(&Path::new(env!("CARGO_MANIFEST_DIR")).join("../../catalog/electrical"))
            .unwrap()
    }

    fn evaluate(source: &str, catalog: &CatalogInput) -> Result<Value, String> {
        let input = SchematicInput::parse(
            Path::new("блок.toml"),
            source.as_bytes().to_vec(),
            &catalog.catalog,
        )?;
        let depth = BTreeMap::from([("блок".into(), input.assess_depth(catalog)?)]);
        let installation = Installation {
            files: Inputs::default(),
            schemes: BTreeMap::from([("блок".into(), input)]),
            depth,
            missing: BTreeSet::new(),
            profile: None,
            installed: None,
            resets: BTreeMap::new(),
            loads: BTreeMap::new(),
        };
        let evaluated = installation.evaluate(build(&installation)?, None, None)?;
        serde_json::to_value(evaluated.report).map_err(|err| err.to_string())
    }

    fn includes(bounds: &Value, expected: f64) {
        let low = bounds[0].as_f64().unwrap();
        let high = bounds[1].as_f64().unwrap();
        // Округление наружу на величинах до 10 В перед вычитанием для запаса.
        let tolerance = 32.0 * f64::EPSILON * expected.abs().max(1.0);
        assert!(
            low <= expected + tolerance && high >= expected - tolerance,
            "{bounds} против {expected}"
        );
        assert!(
            (high - low) <= 2.0 * tolerance,
            "слишком широкая огибающая {bounds}"
        );
    }

    #[test]
    fn t37_отчёт_различает_шесть_причин_и_сохраняет_старый_формат() {
        let catalog = catalog();
        let legacy = evaluate(SCHEME, &catalog).unwrap();
        assert_eq!(legacy["version"], 1);
        assert!(legacy["assemblies"]["блок"].get("depth_checks").is_none());

        let source = format!("{SCHEME}{CAPACITOR}{OPEN_CHECKS}");
        let report = evaluate(&source, &catalog).unwrap();
        assert_eq!(report, evaluate(&source, &catalog).unwrap());
        let decisions = |report: &Value| {
            report["findings"]
                .as_array()
                .unwrap()
                .iter()
                .map(|finding| {
                    ["key", "code", "outcome", "reason"].map(|field| finding[field].clone())
                })
                .collect::<Vec<_>>()
        };
        assert_eq!(decisions(&legacy), decisions(&report));
        let checks = &report["assemblies"]["блок"]["depth_checks"];
        assert_eq!(checks.as_object().unwrap().len(), 6);

        let cap = &checks["конденсатор"];
        assert_eq!(cap["status"], "pass");
        assert_eq!(cap["mode_number"], 6);
        assert_eq!(cap["window"]["end_ns"], 1_000_000);
        assert_eq!(cap["branch"]["component"], "блок");
        assert_eq!(cap["passport"], "capacitor_rc_bench");
        assert_eq!(
            cap["observation_input"],
            report["assemblies"]["блок"]["schematic"]
        );
        let metrics = &cap["metrics"];
        includes(&metrics["voltage_v"]["value"]["margin"], 9.0 - 8.0);
        includes(
            &metrics["ripple_a"]["value"]["observed"],
            0.5 / 3.0_f64.sqrt(),
        );
        includes(&metrics["loss_w"]["value"], 1.0 / 12.0);
        includes(
            &metrics["hotspot_k"]["value"]["observed"],
            300.0 + 2.0 / 12.0,
        );

        for (id, code, metric) in [
            ("защита", "fuse_coordination", "clearing_time_s"),
            ("ключ", "switch_thermal", "foster_peak_junction_k"),
            ("разряд", "deep_discharge", "cutoff"),
            ("сон", "sleep_drain", "consumption"),
            ("изгиб", "cable_flex", "minimum_radius_m"),
        ] {
            assert_eq!(checks[id]["code"], code);
            assert_eq!(checks[id]["status"], "not_analyzed");
            let gap = &checks[id]["metrics"][metric];
            assert_eq!(gap["status"], "not_analyzed", "{id}");
            assert!(!gap["reason"].as_str().unwrap().is_empty());
        }
        assert!(
            checks["защита"]["applicability_gap"]
                .as_str()
                .unwrap()
                .contains("инъекция")
        );
        let with_event = source.replace(
            "id = \"защита\"",
            r#"id = "защита"
event = { kind = "fault_applied", at_ns = 0, branch = { kind = "element", component = "ключ", element = "m" }, source = { derived = "Включение нагрузки на первой отметке стенда" } }"#,
        );
        let event_report = evaluate(&with_event, &catalog).unwrap();
        let fuse = &event_report["assemblies"]["блок"]["depth_checks"]["защита"];
        assert!(fuse["applicability_gap"].is_null());
        assert_eq!(fuse["event"]["kind"], "fault_applied");
        assert_eq!(fuse["event"]["branch"]["component"], "ключ");
        assert_eq!(fuse["event"]["time_ns"], 0);
        assert_eq!(fuse["status"], "not_analyzed");

        let shifted = with_event.replace("at_ns = 0, branch", "at_ns = 1, branch");
        let report = evaluate(&shifted, &catalog).unwrap();
        assert!(
            report["assemblies"]["блок"]["depth_checks"]["защита"]["applicability_gap"]
                .as_str()
                .unwrap()
                .contains("инъекция")
        );
    }

    #[test]
    fn t37_нет_ложного_допуска_по_оценке_или_чужому_окну() {
        let mut catalog = catalog();
        let source = format!("{SCHEME}{CAPACITOR}");
        let cap = |input: &str, catalog: &CatalogInput| {
            evaluate(input, catalog).unwrap()["assemblies"]["блок"]["depth_checks"]["конденсатор"]
                .clone()
        };
        let failed = source.replace("8.0, 8.0, 8.0", "10.0, 10.0, 10.0");
        let dangerous = cap(&failed, &catalog);
        assert_eq!(dangerous["status"], "fail");
        includes(
            &dangerous["metrics"]["voltage_v"]["value"]["margin"],
            9.0 - 10.0,
        );
        let report = evaluate(&failed, &catalog).unwrap();
        assert_eq!(report["simulation"], "Rejected");

        let open = source.replace("end_ns = 1000000", "end_ns = 1000001");
        let open = cap(&open, &catalog);
        assert_eq!(open["status"], "not_analyzed");
        assert_eq!(open["metrics"]["voltage_v"]["status"], "not_analyzed");

        let estimated_context = source.replace(
            "derived = \"Аналитическая постановка",
            "estimate = \"Аналитическая постановка",
        );
        let unbound = cap(&estimated_context, &catalog);
        assert_eq!(unbound["status"], "not_analyzed");
        assert_eq!(unbound["diagnostic_only"], true);
        assert!(!unbound["applicability_gap"].as_str().unwrap().is_empty());

        let assembly = source
            .replace("scope = \"reference\"", "scope = \"assembly\"")
            .replace("derived =", "measured =");
        let reference = cap(&assembly, &catalog);
        assert_eq!(reference["status"], "not_analyzed");
        assert!(reference["metrics"].is_null());
        assert!(
            reference["reason"]
                .as_str()
                .unwrap()
                .contains("эталонного стенда")
        );

        let (_, bytes) = catalog
            .sources
            .iter_mut()
            .find(|(path, _)| path.ends_with("depth-ratings.toml"))
            .unwrap();
        let ratings = String::from_utf8(bytes.clone()).unwrap();
        *bytes = ratings
            .replace(
                ", source = { derived = \"Условный предел RC-стенда, 12 В перед derating\" }",
                "",
            )
            .into_bytes();
        let unsourced = cap(&source, &catalog);
        assert_eq!(unsourced["status"], "not_analyzed");
        assert!(
            unsourced["metrics"]["voltage_v"]["reason"]
                .as_str()
                .unwrap()
                .contains("паспортного")
        );

        for (input, message) in [
            (source.replace("8.0, 8.0, 8.0", "8.0, nan, 8.0"), "конечные"),
            (source.replace("8.0, 8.0, 8.0", "8.0, inf, 8.0"), "конечные"),
            (
                source.replace("блок/a\", \"блок/b", "блок/b\", \"блок/a"),
                "полярность",
            ),
            (
                source.replace("capacitor_rc_bench", "nmos_pending"),
                "другой каталожной",
            ),
            (format!("{source}{CAPACITOR}"), "указано дважды"),
        ] {
            assert!(
                evaluate(&input, &catalog).unwrap_err().contains(message),
                "{message}"
            );
        }
    }
}
