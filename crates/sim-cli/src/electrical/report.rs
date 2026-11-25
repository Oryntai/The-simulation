//! Связь разрешённой схемы, сборочного листа и записи опыта.

use std::{
    collections::{BTreeMap, BTreeSet},
    io::Write,
    path::{Path, PathBuf},
};

use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sim_process::schematic::{AssemblySheet, Endpoint, Expected, Parameter, ResetProfile, Stage};
use sim_runtime::{Runtime, control};
use sim_signal::Time;
use sim_task::{
    Fingerprint, Outcome,
    verdict::{ElectricalEvidence, Finding, SimulationVerdict},
};

use super::{CatalogInput, LiveExperiment, LivePath, LiveSummary, SchematicInput};
use crate::tasks::processes::{Inputs, capture_world};

pub(crate) struct Installation {
    files: Inputs,
    schemes: BTreeMap<String, SchematicInput>,
    missing: BTreeSet<String>,
    profile: Option<Fingerprint>,
    installed: Option<control::RunIdentity>,
    resets: BTreeMap<String, ResetProfile>,
}

#[derive(Deserialize)]
struct ProfileFiles {
    parts: Option<PathBuf>,
    control: PathBuf,
    stimuli: Option<PathBuf>,
    measurements: Option<PathBuf>,
    #[serde(default)]
    sampled: BTreeMap<String, PathBuf>,
    circuits: BTreeMap<String, CircuitFiles>,
}

#[derive(Deserialize)]
struct CircuitFiles {
    #[serde(default)]
    boards: BTreeMap<String, BoardSupplyFiles>,
    diagnostic_connections: Option<DiagnosticFiles>,
}

#[derive(Deserialize)]
struct DiagnosticFiles {
    parameters: Option<PathBuf>,
}

#[derive(Deserialize)]
struct BoardSupplyFiles {
    reference_reset: Option<toml::Value>,
}

#[derive(Deserialize)]
struct BoardReference {
    name: String,
    catalog: String,
}

#[derive(Deserialize)]
struct ControlFiles {
    #[serde(default)]
    board: Vec<BoardReference>,
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
        let mut stamp = None;
        let mut local_parts = None;
        let mut resets = BTreeMap::new();
        if let Some(path) = profile {
            stamp = Some(Fingerprint::of(files.capture(path)?));
            let inputs: ProfileFiles = files.toml(path)?;
            let dir = dir(path);
            let control: ControlFiles = files.toml(&dir.join(&inputs.control))?;
            for diagnostic in inputs
                .circuits
                .values()
                .filter_map(|c| c.diagnostic_connections.as_ref())
            {
                if let Some(path) = &diagnostic.parameters {
                    files.capture(&dir.join(path))?;
                }
            }
            let defaults = inputs
                .circuits
                .into_values()
                .flat_map(|circuit| circuit.boards)
                .filter(|(_, supply)| supply.reference_reset.is_none())
                .map(|(board, _)| board)
                .collect::<BTreeSet<_>>();
            for board in control
                .board
                .into_iter()
                .filter(|board| defaults.contains(&board.name))
            {
                if Path::new(&board.catalog)
                    .file_name()
                    .and_then(|name| name.to_str())
                    != Some(board.catalog.as_str())
                    || board.catalog.chars().any(char::is_control)
                {
                    return Err("ожидалось имя позиции каталога платы".into());
                }
                let path = catalog
                    .join("boards/electrical")
                    .join(format!("{}.toml", board.catalog));
                let profile = CatalogInput::parse_board_profile(&path, files.capture(&path)?)?;
                if profile.board != board.catalog {
                    return Err("электрический профиль относится к другой плате".into());
                }
                resets.insert(board.name, profile.reset);
            }
            for file in [inputs.stimuli, inputs.measurements]
                .into_iter()
                .flatten()
                .chain(inputs.sampled.into_values())
            {
                files.capture(&dir.join(file))?;
            }
            local_parts = inputs.parts.map(|file| dir.join(file));
        }
        let mut schemes = BTreeMap::new();
        if !paths.is_empty() {
            let parts = if let Some(path) = local_parts {
                super::parse_catalog(&path, files.capture(&path)?)?
            } else {
                let CatalogInput { catalog, sources } =
                    super::load_catalog(&catalog.join("electrical"))?;
                for (path, bytes) in sources {
                    if files.capture(&path)? != bytes {
                        return Err(format!("изменился каталог {}", path.display()));
                    }
                }
                catalog
            };
            for (instance, path) in paths {
                let bytes = files.capture(&path)?.to_vec();
                schemes.insert(instance, SchematicInput::parse(&path, bytes, &parts)?);
            }
        }
        Ok(Self {
            files,
            schemes,
            missing,
            profile: stamp,
            installed: None,
            resets,
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
        let simulation = match Outcome::aggregate(findings.iter().map(|finding| finding.outcome)) {
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
