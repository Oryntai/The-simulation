use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    path::{Path, PathBuf},
};

use serde::{Deserialize, Serialize};
use sim_drive::{DriveHandle, sampled};
use sim_process::{
    Analysis, SourcedRange,
    schematic::{
        BindingTarget, BoardContact, Endpoint, ExpandedNetlist, Mode, Parameter, PinFunction,
        PinPhase, ResetProfile, Scope, Section,
        sim_circuit::netlist::{Analysis as NetAnalysis, ElementKind, Netlist, Waveform},
    },
};
use sim_runtime::{
    ObservationOut, Runtime, Sensor, Status, Truth, TruthId,
    control::{self, ControlConfig, ControlRecording, FileAssets, RunSpec},
    electrical::{
        BoardConnection, BoardCurrents, ElectricalProbe, ElectricalSetup, GpioConnection, LivePin,
        LoadConnection, SourceStep, TransientCircuit, resolve_live_contact,
    },
};
use sim_signal::{Rate, Rng, SignalId, Time};
use sim_world::{CompiledWorld, Provenance, Simplification, WorldWithSchematics};

use super::{CatalogInput, SchematicInput};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LivePath {
    Legacy,
    Full,
}

pub struct LiveExperiment {
    pub runtime: Runtime,
}

#[derive(Serialize, Default)]
pub(crate) struct LiveSummary {
    reset_edges: Vec<ResetEdge>,
    board_min_v: BTreeMap<String, f64>,
}

#[derive(Serialize)]
struct ResetEdge {
    board: String,
    time_ns: u64,
    asserted: bool,
}

impl From<&Runtime> for LiveSummary {
    fn from(runtime: &Runtime) -> Self {
        let mut summary = Self::default();
        if let Some(run) = control::run(runtime) {
            for event in run.events() {
                match &event.kind {
                    control::EventKind::ElectricalReset { board, asserted } => {
                        summary.reset_edges.push(ResetEdge {
                            board: board.clone(),
                            time_ns: event.at.0,
                            asserted: *asserted,
                        });
                    }
                    control::EventKind::ElectricalRail {
                        board, voltage_v, ..
                    } => {
                        summary
                            .board_min_v
                            .entry(board.clone())
                            .and_modify(|value| *value = value.min(*voltage_v))
                            .or_insert(*voltage_v);
                    }
                    _ => {}
                }
            }
        }
        summary
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Profile {
    format: u32,
    step_ns: u64,
    source: String,
    control: PathBuf,
    stimuli: Option<PathBuf>,
    measurements: Option<PathBuf>,
    legacy_measurements: Option<PathBuf>,
    parts: Option<PathBuf>,
    #[serde(default)]
    sampled: BTreeMap<String, PathBuf>,
    circuits: BTreeMap<String, CircuitProfile>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CircuitProfile {
    #[serde(default)]
    drives: BTreeMap<String, String>,
    #[serde(default)]
    reference_drive_min_v: BTreeMap<String, f64>,
    diagnostic_connections: Option<DiagnosticConnections>,
    #[serde(default)]
    boards: BTreeMap<String, BoardSupply>,
    #[serde(default)]
    observed_boards: BTreeMap<String, String>,
    #[serde(default)]
    steps: Vec<Step>,
    #[serde(default)]
    probes: BTreeMap<String, Probe>,
    #[serde(default)]
    meters: BTreeMap<String, Meter>,
    #[serde(default)]
    voltage_observations: BTreeMap<String, VoltageObservation>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct DiagnosticConnections {
    ohm: f64,
    source: String,
    parameters: Option<PathBuf>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Step {
    branch: String,
    at_ns: u64,
    value: f64,
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Probe {
    Voltage { ends: [String; 2] },
    Current { branch: String },
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Meter {
    rate_hz: u64,
    minimum_v: f64,
    source: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct VoltageObservation {
    ends: [String; 2],
    meter: Meter,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct BoardSupply {
    rail: String,
    branch: String,
    currents_a: [f64; 4],
    boot_ns: u64,
    initially_in_reset: bool,
    reference_reset: Option<ReferenceReset>,
    phase_source: Option<ConfirmedSource>,
    #[serde(default)]
    live_pins: Vec<u32>,
}

#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
enum ConfirmedSource {
    Datasheet(String),
    Measured(String),
}

impl From<ConfirmedSource> for Provenance {
    fn from(source: ConfirmedSource) -> Self {
        match source {
            ConfirmedSource::Datasheet(value) => Self::Datasheet(value),
            ConfirmedSource::Measured(value) => Self::Measured(value),
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ReferenceReset {
    falling_v: f64,
    rising_v: f64,
    assert_ns: u64,
    release_ns: u64,
    source: String,
}

#[derive(Deserialize)]
struct Wiring {
    #[serde(default)]
    board: Vec<InstalledBoard>,
    #[serde(default)]
    board_input: Vec<InstalledInput>,
    #[serde(default)]
    driver: Vec<InstalledDriver>,
}

#[derive(Deserialize)]
struct InstalledBoard {
    name: String,
    catalog: String,
}

#[derive(Deserialize)]
struct InstalledInput {
    board: String,
    kind: String,
    port: u32,
}

#[derive(Deserialize)]
struct InstalledDriver {
    board: String,
    pin: u32,
    quantity: String,
    direction_pin: Option<u32>,
    enable_pin: Option<u32>,
}

impl Wiring {
    fn board(&self, name: &str) -> Result<&str, String> {
        self.board
            .iter()
            .find(|b| b.name == name)
            .map(|b| b.catalog.as_str())
            .ok_or_else(|| format!("нет установленной платы «{name}»"))
    }

    fn pins(&self, board: &str) -> Result<BTreeSet<BindingTarget>, String> {
        let catalog = self.board(board)?;
        let mut result = BTreeSet::new();
        let mut by_pin = BTreeMap::new();
        let mut define = |number, function| -> Result<(), String> {
            let pin = resolve_live_contact(catalog, number, function)?;
            if by_pin
                .insert(pin.clone(), function)
                .is_some_and(|old| old != function)
            {
                return Err(format!("«{board}/{pin}»: неоднозначная функция контакта"));
            }
            result.insert(BindingTarget::BoardPin {
                board: board.into(),
                pin,
                function,
            });
            Ok(())
        };
        for input in self.board_input.iter().filter(|wire| wire.board == board) {
            match input.kind.as_str() {
                "adc" => define(input.port, PinFunction::Adc)?,
                "gpio" => define(input.port, PinFunction::GpioInput)?,
                _ => {
                    return Err(format!(
                        "«{board}»: вход {} не задаёт физический контакт",
                        input.kind
                    ));
                }
            }
        }
        for output in self.driver.iter().filter(|wire| wire.board == board) {
            let function = if output.quantity == "logic" {
                PinFunction::GpioOutput
            } else {
                PinFunction::Pwm
            };
            define(output.pin, function)?;
            for number in output.direction_pin.into_iter().chain(output.enable_pin) {
                define(number, PinFunction::GpioOutput)?;
            }
        }
        Ok(result)
    }
}

#[derive(Deserialize)]
struct SchematicBindings {
    scope: ScopeName,
    #[serde(default)]
    bindings: Vec<DiskBinding>,
}

#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
enum ScopeName {
    Assembly,
    Reference,
}

#[derive(Deserialize)]
struct DiskBinding {
    ends: Vec<String>,
    target: DiskTarget,
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum DiskTarget {
    BoardPin {
        board: String,
        pin: String,
        function: Function,
    },
    BoardPower {
        board: String,
        rail: String,
    },
    DrivePower {
        drive: String,
    },
    SensorPower {
        sensor: String,
    },
}

#[derive(Deserialize)]
#[serde(remote = "PinFunction", rename_all = "snake_case")]
enum FunctionData {
    GpioInput,
    GpioOutput,
    Adc,
    Pwm,
    Uart,
    I2c,
    Spi,
}

#[derive(Deserialize)]
#[serde(transparent)]
struct Function(#[serde(with = "FunctionData")] PinFunction);

impl From<DiskTarget> for BindingTarget {
    fn from(item: DiskTarget) -> Self {
        match item {
            DiskTarget::BoardPin {
                board,
                pin,
                function,
            } => Self::BoardPin {
                board,
                pin,
                function: function.0,
            },
            DiskTarget::BoardPower { board, rail } => Self::BoardPower { board, rail },
            DiskTarget::DrivePower { drive } => Self::DrivePower { drive },
            DiskTarget::SensorPower { sensor } => Self::SensorPower { sensor },
        }
    }
}

fn parse<T: serde::de::DeserializeOwned>(path: &Path, bytes: &[u8]) -> Result<T, String> {
    let text = std::str::from_utf8(bytes).map_err(|e| format!("{}: {e}", path.display()))?;
    let table: toml::Table =
        toml::from_str(text).map_err(|e| format!("{}: {e}", path.display()))?;
    fn finite(value: &toml::Value) -> bool {
        match value {
            toml::Value::Float(value) => value.is_finite(),
            toml::Value::Array(values) => values.iter().all(finite),
            toml::Value::Table(values) => values.values().all(finite),
            _ => true,
        }
    }
    if !table.values().all(finite) {
        return Err(format!(
            "{}: обнаружено неконечное значение",
            path.display()
        ));
    }
    table
        .try_into()
        .map_err(|e| format!("{}: {e}", path.display()))
}

fn read(path: &Path) -> Result<Vec<u8>, String> {
    fs::read(path).map_err(|e| format!("{}: {e}", path.display()))
}

fn base(path: &Path) -> &Path {
    path.parent().unwrap_or_else(|| Path::new("."))
}

fn frame(bytes: &mut Vec<u8>, part: &[u8]) {
    bytes.extend_from_slice(&(part.len() as u64).to_le_bytes());
    bytes.extend_from_slice(part);
}

fn section(name: &str) -> Section {
    match name.split_once('/') {
        Some((component, element)) => Section::Element {
            component: component.into(),
            element: element.into(),
        },
        None => Section::Connection(name.into()),
    }
}

fn branch(net: &ExpandedNetlist, address: &str) -> Result<String, String> {
    let target = section(address);
    net.origins
        .iter()
        .find(|(_, origin)| **origin == target)
        .map(|(name, _)| name.clone())
        .ok_or_else(|| format!("не найдена ветвь «{address}»"))
}

fn endpoint(name: &str) -> Result<Endpoint, String> {
    let (component, pin) = name
        .split_once('/')
        .ok_or_else(|| format!("«{name}»: нужен компонент/вывод"))?;
    Ok(Endpoint {
        component: component.into(),
        pin: pin.into(),
    })
}

fn node(net: &ExpandedNetlist, address: &str) -> Result<String, String> {
    net.nodes
        .get(&endpoint(address)?)
        .cloned()
        .ok_or_else(|| format!("нет узла для «{address}»"))
}

fn pair(net: &ExpandedNetlist, ends: &[String]) -> Result<[String; 2], String> {
    let [plus, minus] = ends else {
        return Err("питанию нужна пара клемм".into());
    };
    Ok([node(net, plus)?, node(net, minus)?])
}

fn external(net: &mut ExpandedNetlist, name: &str) -> Result<(), String> {
    let element = net
        .netlist
        .elements
        .iter_mut()
        .find(|el| el.name == name)
        .ok_or_else(|| format!("не найдена внешняя ветвь «{name}»"))?;
    let source = match &mut element.kind {
        ElementKind::CurrentSource(s) | ElementKind::VoltageSource(s) => s,
        _ => return Err(format!("«{name}»: нужна ветвь независимого источника")),
    };
    if matches!(source.wave, Some(Waveform::Ext { .. })) {
        return Ok(());
    }
    if source.wave.is_some() {
        return Err(format!(
            "«{name}»: внешний владелец заменяет форму источника"
        ));
    }
    let initial = source
        .dc
        .ok_or_else(|| format!("«{name}»: нет начального значения источника"))?;
    source.wave = Some(Waveform::Ext { initial });
    Ok(())
}

fn rename(net: &mut ExpandedNetlist, index: usize) {
    let node = |name: &mut String| {
        // Единственное общее число задаёт математическую опору каждого независимого острова.
        if name != "0" {
            *name = format!("c{index}_{name}");
        }
    };
    let model = |name: &mut String| *name = format!("c{index}_{name}");
    let element = |name: &str| format!("{}c{index}_{name}", name.as_bytes()[0] as char);
    for name in net.nodes.values_mut() {
        node(name);
    }
    for el in &mut net.netlist.elements {
        el.name = element(&el.name);
        for name in &mut el.nodes {
            node(name);
        }
        match &mut el.kind {
            ElementKind::Diode { model: name, .. }
            | ElementKind::Mosfet { model: name, .. }
            | ElementKind::Switch {
                model: Some(name), ..
            } => model(name),
            _ => {}
        }
    }
    for card in &mut net.netlist.models {
        model(&mut card.name);
    }
    net.origins = std::mem::take(&mut net.origins)
        .into_iter()
        .map(|(name, origin)| (element(&name), origin))
        .collect();
}

fn point(value: f64, provenance: &Provenance) -> SourcedRange {
    SourcedRange {
        min: value,
        nominal: value,
        max: value,
        provenance: provenance.clone(),
    }
}

fn parameter(value: f64, provenance: &Provenance) -> Parameter {
    Parameter {
        range: Some(point(value, provenance)),
        measurement: "снять соответствующий ряд на физическом стенде".into(),
    }
}

struct Assembly {
    setup: ElectricalSetup,
    net: Netlist,
    pins: Vec<LivePin>,
    drives: BTreeMap<String, ([String; 2], String)>,
    reference_drive_min_v: BTreeMap<String, f64>,
    meters: BTreeMap<String, Meter>,
    notes: Vec<Simplification>,
    board_owners: BTreeSet<String>,
}

impl Assembly {
    fn new(step: Time) -> Self {
        Self {
            setup: ElectricalSetup {
                max_step: step,
                scope: Scope::Assembly,
                boards: Vec::new(),
                sources: Vec::new(),
                probes: BTreeMap::new(),
            },
            net: Netlist {
                title: "Живые цепи мира".into(),
                elements: Vec::new(),
                models: Vec::new(),
                analyses: vec![NetAnalysis::Op],
                warnings: Vec::new(),
            },
            pins: Vec::new(),
            drives: BTreeMap::new(),
            reference_drive_min_v: BTreeMap::new(),
            meters: BTreeMap::new(),
            notes: Vec::new(),
            board_owners: BTreeSet::new(),
        }
    }

    fn append(
        &mut self,
        instance: &str,
        index: usize,
        input: SchematicInput,
        cfg: CircuitProfile,
        world: &CompiledWorld,
        wiring: &Wiring,
        catalog: &Path,
        parts: &sim_process::schematic::Catalog,
        profile_dir: &Path,
        source: &str,
        identity: &mut Vec<u8>,
    ) -> Result<(), String> {
        let SchematicBindings { scope, bindings } = parse(&input.path, &input.bytes)?;
        let reference = matches!(scope, ScopeName::Reference);
        let diagnostic = cfg.diagnostic_connections.is_some();
        let mut installed = BTreeSet::new();
        let mut boards = BTreeMap::new();
        for (name, rail) in cfg.observed_boards {
            if !self.board_owners.insert(name.clone()) {
                return Err(format!("плата «{name}» питается из двух схем"));
            }
            installed.extend(wiring.pins(&name)?);
            self.notes.push(Simplification {
                subject: format!("{name}/{rail}"),
                text:
                    "Питание платы наблюдается. Реакция платы на напряжение остаётся NotAnalyzed."
                        .into(),
                blocks_proof: true,
            });
            installed.insert(BindingTarget::BoardPower { board: name, rail });
        }
        for (name, data) in cfg.boards {
            if !self.board_owners.insert(name.clone()) {
                return Err(format!("плата «{name}» питается из двух схем"));
            }
            let board_type = wiring.board(&name)?.to_owned();
            installed.extend(wiring.pins(&name)?);
            installed.insert(BindingTarget::BoardPower {
                board: name.clone(),
                rail: data.rail.clone(),
            });
            boards.insert(name, (board_type, data));
        }
        for drive in cfg.drives.keys() {
            let full = format!("{instance}/{drive}");
            if !world
                .drives
                .iter()
                .any(|d| d.name == full && d.owner == instance)
            {
                return Err(format!("отсутствует установленный привод «{full}»"));
            }
            installed.insert(BindingTarget::DrivePower {
                drive: drive.clone(),
            });
        }
        for (drive, minimum) in cfg.reference_drive_min_v {
            if !(reference || diagnostic) || !cfg.drives.contains_key(&drive) || minimum < 0.0 {
                return Err(format!(
                    "«{drive}»: порог модели допустим только у нагрузки эталонной схемы"
                ));
            }
            self.reference_drive_min_v
                .insert(format!("{instance}/{drive}"), minimum);
        }
        for sensor in cfg.meters.keys() {
            let full = format!("{instance}/{sensor}");
            if !world.sensors.iter().any(|s| s.name == full) {
                return Err(format!("отсутствует установленная модель датчика «{full}»"));
            }
            installed.insert(BindingTarget::SensorPower {
                sensor: sensor.clone(),
            });
        }
        input
            .compiled
            .validate_bindings(&installed)
            .map_err(|e| e.to_string())?;
        let mut ends = BTreeMap::new();
        for item in bindings {
            ends.insert(BindingTarget::from(item.target), item.ends);
        }
        let mut net = if let Some(assumption) = cfg.diagnostic_connections {
            let file = assumption.parameters.map(|path| profile_dir.join(path));
            let bytes = file.as_deref().map(read).transpose()?;
            if let Some(bytes) = &bytes {
                frame(identity, bytes);
            }
            input.diagnostic_netlist(
                parts,
                assumption.ohm,
                &assumption.source,
                file.as_deref().zip(bytes.as_deref()),
            )?
        } else {
            match input.compiled.netlist() {
                Analysis::Analyzed { value, .. } => value,
                Analysis::NotAnalyzed { reason } => return Err(reason),
            }
        };
        rename(&mut net, index);
        for (drive, address) in cfg.drives {
            let nodes = pair(
                &net,
                &ends[&BindingTarget::DrivePower {
                    drive: drive.clone(),
                }],
            )?;
            let name = branch(&net, &address)?;
            external(&mut net, &name)?;
            let full = format!("{instance}/{drive}");
            self.probe(
                format!("{full}/ток MNA"),
                ElectricalProbe::Current {
                    branch: name.clone(),
                },
            )?;
            self.drives.insert(full, (nodes, name));
        }
        for (name, (board_type, data)) in boards {
            let synthetic_reset = data.reference_reset.is_some();
            let scope = if reference || diagnostic && synthetic_reset {
                Scope::Reference
            } else {
                Scope::Assembly
            };
            if !self.setup.boards.is_empty() && self.setup.scope != scope {
                return Err("платы одного опыта требуют разных областей модели сброса".into());
            }
            self.setup.scope = scope;
            let [plus, minus] = pair(
                &net,
                &ends[&BindingTarget::BoardPower {
                    board: name.clone(),
                    rail: data.rail.clone(),
                }],
            )?;
            let current_source = branch(&net, &data.branch)?;
            external(&mut net, &current_source)?;
            if data.currents_a.iter().any(|&current| current < 0.0) {
                return Err("ожидался неотрицательный ток потребления платы".into());
            }
            let profile_path = catalog
                .join("boards/electrical")
                .join(format!("{board_type}.toml"));
            let catalog_board = match fs::read(&profile_path) {
                Ok(bytes) => {
                    let profile = CatalogInput::parse_board_profile(&profile_path, &bytes)?;
                    if profile.board != board_type {
                        return Err("профиль записан для другой платы".into());
                    }
                    frame(identity, &bytes);
                    Some(profile)
                }
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => None,
                Err(e) => return Err(format!("{}: {e}", profile_path.display())),
            };
            let reset = if let Some(reset) = data.reference_reset {
                if scope != Scope::Reference {
                    return Err(
                        "синтетический детектор требует явного диагностического опыта".into(),
                    );
                }
                if reset.source.trim().is_empty() {
                    return Err("нет источника порогов стенда".into());
                }
                let provenance = Provenance::Derived(reset.source);
                self.notes.push(Simplification {
                    subject: format!("{name}/сброс"),
                    text: "Срабатывание заданного стендового детектора. Пороги и выдержки физической платы остаются NotAnalyzed.".into(),
                    blocks_proof: true,
                });
                ResetProfile {
                    falling_v: parameter(reset.falling_v, &provenance),
                    rising_v: parameter(reset.rising_v, &provenance),
                    assert_delay_s: parameter(Time(reset.assert_ns).as_secs(), &provenance),
                    release_delay_s: parameter(Time(reset.release_ns).as_secs(), &provenance),
                }
            } else {
                catalog_board
                    .as_ref()
                    .ok_or("NotAnalyzed: нет электрического профиля платы")?
                    .reset
                    .clone()
            };
            let provenance = match data.phase_source {
                Some(source) => source.into(),
                None if scope == Scope::Reference => Provenance::Derived(source.into()),
                None => {
                    return Err(
                        "для токов фаз и загрузки платы нужен источник замера или паспорта".into(),
                    );
                }
            };
            let [unpowered_a, reset_a, boot_a, working_a] =
                data.currents_a.map(|v| point(v, &provenance));
            let mut pins = Vec::new();
            for pin in data.live_pins {
                let contact = BoardContact {
                    board: name.clone(),
                    pin: pin.to_string(),
                };
                let mut setup = input
                    .modes
                    .cases
                    .get(&Mode::Normal)
                    .and_then(|case| case.pins.get(&contact))
                    .cloned()
                    .ok_or_else(|| {
                        format!("для {name}/{pin} отсутствует модель вывода штатного режима")
                    })?;
                if setup.rail != data.rail {
                    return Err(format!("у {name}/{pin} другая опора питания"));
                }
                if let Some(board) = &catalog_board {
                    board.apply_reset_default(pin, &mut setup.profile);
                }
                let phase = if data.initially_in_reset {
                    PinPhase::Reset
                } else if data.boot_ns > 0 {
                    PinPhase::Boot {
                        elapsed: Time::ZERO,
                    }
                } else {
                    setup.phase
                };
                let initial = match setup
                    .profile
                    .state(&phase, scope)
                    .map_err(|e| e.to_string())?
                {
                    Analysis::Analyzed { value, .. } => value,
                    Analysis::NotAnalyzed { reason } => return Err(reason),
                };
                let [pad, _, _] = input
                    .compiled
                    .board_contact_ends(&contact, &data.rail)
                    .map_err(|e| e.to_string())?;
                let pad = net
                    .nodes
                    .get(&pad)
                    .cloned()
                    .ok_or("контакт не подключён к ветвям MNA")?;
                let id = format!("{instance}/{name}/{pin}");
                self.pins.push(LivePin {
                    contact: id.clone(),
                    nodes: [pad, plus.clone(), minus.clone()],
                    profile: setup.profile.clone(),
                    initial,
                });
                pins.push(GpioConnection {
                    pin,
                    contact: id,
                    profile: setup.profile,
                });
            }
            self.setup.boards.push(BoardConnection {
                board: name,
                plus,
                minus,
                current_source,
                reset,
                initially_in_reset: data.initially_in_reset,
                currents: BoardCurrents {
                    unpowered_a,
                    reset_a,
                    boot_a,
                    working_a,
                },
                boot_duration_s: parameter(Time(data.boot_ns).as_secs(), &provenance),
                pins,
            });
        }
        for (sensor, meter) in cfg.meters {
            let [plus, minus] = pair(
                &net,
                &ends[&BindingTarget::SensorPower {
                    sensor: sensor.clone(),
                }],
            )?;
            let name = format!("{instance}/{sensor}/питание");
            self.probe(name.clone(), ElectricalProbe::Voltage { plus, minus })?;
            self.meters.insert(name, meter);
        }
        for (name, observation) in cfg.voltage_observations {
            let [plus, minus] = pair(&net, &observation.ends)?;
            self.probe(name.clone(), ElectricalProbe::Voltage { plus, minus })?;
            self.meters.insert(name, observation.meter);
        }
        for step in cfg.steps {
            let name = branch(&net, &step.branch)?;
            external(&mut net, &name)?;
            self.setup.sources.push(SourceStep {
                at: Time(step.at_ns),
                source: name,
                value: step.value,
            });
        }
        for (name, probe) in cfg.probes {
            let probe = match probe {
                Probe::Voltage { ends } => {
                    let [plus, minus] = pair(&net, &ends)?;
                    ElectricalProbe::Voltage { plus, minus }
                }
                Probe::Current { branch: address } => ElectricalProbe::Current {
                    branch: branch(&net, &address)?,
                },
            };
            self.probe(format!("{instance}/{name}"), probe)?;
        }
        for warning in &net.netlist.warnings {
            self.notes.push(Simplification {
                subject: format!("{instance}/схема"),
                text: warning.clone(),
                blocks_proof: true,
            });
        }
        self.net.elements.extend(net.netlist.elements);
        self.net.models.extend(net.netlist.models);
        frame(identity, instance.as_bytes());
        frame(identity, &input.bytes);
        Ok(())
    }

    fn probe(&mut self, name: String, probe: ElectricalProbe) -> Result<(), String> {
        if self.setup.probes.insert(name.clone(), probe).is_some() {
            return Err(format!("повтор электрического наблюдения «{name}»"));
        }
        Ok(())
    }
}

#[derive(Clone)]
struct VoltageMeter {
    input: TruthId,
    output: SignalId,
    low: f64,
}

impl Sensor for VoltageMeter {
    fn sample(
        &mut self,
        truth: &Truth<'_>,
        _: &mut Rng,
        out: &mut ObservationOut,
    ) -> Result<(), String> {
        let v = truth
            .plant(self.input)
            .ok_or("нет напряжения на входе измерителя")?;
        out.write(self.output, v, v >= self.low);
        Ok(())
    }
}

impl LiveExperiment {
    /// Устанавливает опыт из файлов мира, схем и профиля. Обе постановки
    /// используют одну механику, календарь, контроллеры и входные стимулы.
    pub fn load(
        world_path: &Path,
        profile_path: &Path,
        catalog: &Path,
        seed: Option<u64>,
        until: Time,
        path: LivePath,
    ) -> Result<Self, String> {
        Self::assemble(world_path, profile_path, catalog, seed, until, path, None)
    }

    pub fn load_replay(
        world_path: &Path,
        profile_path: &Path,
        catalog: &Path,
        seed: Option<u64>,
        until: Time,
        recording: ControlRecording,
    ) -> Result<Self, String> {
        Self::assemble(
            world_path,
            profile_path,
            catalog,
            seed,
            until,
            LivePath::Full,
            Some(recording),
        )
    }

    fn assemble(
        world_path: &Path,
        profile_path: &Path,
        catalog: &Path,
        seed: Option<u64>,
        until: Time,
        path: LivePath,
        recording: Option<ControlRecording>,
    ) -> Result<Self, String> {
        let WorldWithSchematics {
            mut world,
            schematics,
        } = sim_world::load_and_compile_with_schematics(world_path)?;
        if let Some(seed) = seed {
            world.seed = seed;
        }
        let profile_bytes = read(profile_path)?;
        let profile: Profile = parse(profile_path, &profile_bytes)?;
        if profile.format != 1 || profile.step_ns == 0 || profile.source.trim().is_empty() {
            return Err("нужны версия 1, ненулевой шаг и источник постановки".into());
        }
        if schematics.is_empty() || schematics.keys().ne(profile.circuits.keys()) {
            return Err("профиль должен покрывать все подключённые схемы мира".into());
        }
        let dir = base(profile_path);
        let control_path = dir.join(&profile.control);
        let mut control_bytes = read(&control_path)?;
        if let Some(stimuli) = profile.stimuli {
            control_bytes.push(b'\n');
            control_bytes.extend(read(&dir.join(stimuli))?);
        }
        let config =
            ControlConfig::parse(std::str::from_utf8(&control_bytes).map_err(|e| e.to_string())?)?;
        let models = crate::tasks::Models::load(catalog)?;
        let mut rt = Runtime::load(world.clone())?;
        let mut identity = Vec::new();
        frame(&mut identity, &profile_bytes);
        frame(&mut identity, &models.bytes);
        frame(
            &mut identity,
            format!("{:?}{:?}", world.drives, world.sensors).as_bytes(),
        );
        let mut assembly = Assembly::new(Time(profile.step_ns));
        if path == LivePath::Full {
            let wiring: Wiring = parse(&control_path, &control_bytes)?;
            let parts = if let Some(parts) = profile.parts {
                let file = dir.join(parts);
                let bytes = read(&file)?;
                let parts = super::parse_catalog(&file, &bytes)?;
                frame(&mut identity, &bytes);
                parts
            } else {
                let CatalogInput { sources, catalog } =
                    super::load_catalog(&catalog.join("electrical"))?;
                for (path, bytes) in sources {
                    frame(
                        &mut identity,
                        path.file_name()
                            .ok_or("нет имени файла каталога")?
                            .as_encoded_bytes(),
                    );
                    frame(&mut identity, &bytes);
                }
                catalog
            };
            for (index, (instance, cfg)) in profile.circuits.into_iter().enumerate() {
                let scheme = &schematics[&instance];
                let input = super::load_schematic(scheme, &parts)?;
                assembly
                    .append(
                        &instance,
                        index,
                        input,
                        cfg,
                        &world,
                        &wiring,
                        catalog,
                        &parts,
                        dir,
                        &profile.source,
                        &mut identity,
                    )
                    .map_err(|e| format!("{}: {e}", scheme.display()))?;
            }
        }
        let mut sampled = BTreeMap::new();
        for (name, file) in profile.sampled {
            let file = dir.join(file);
            let bytes = read(&file)?;
            sampled.insert(name, parse::<sampled::Spec>(&file, &bytes)?);
            frame(&mut identity, &bytes);
        }
        let mut general_world = world.clone();
        general_world
            .drives
            .retain(|drive| !sampled.contains_key(&drive.name));
        for drive in &mut general_world.drives {
            if let Some(minimum) = assembly.reference_drive_min_v.remove(&drive.name) {
                drive.params.insert("voltage_min_v".into(), minimum);
            }
        }
        if !assembly.reference_drive_min_v.is_empty() {
            return Err("порог общей модели назначен дискретному приводу".into());
        }
        let coupled = assembly
            .drives
            .keys()
            .filter(|name| !sampled.contains_key(*name))
            .map(String::as_str)
            .collect::<Vec<_>>();
        let handles =
            DriveHandle::install_with_circuit(&mut rt, &general_world, &models.drives, &coupled)?;
        let mut ports = crate::tasks::model_ports(&handles);
        let mut bus = BTreeMap::new();
        for handle in &handles {
            if let Some(port) = handle.circuit_port() {
                bus.insert(handle.name.clone(), port);
            }
        }
        for (name, spec) in &sampled {
            if assembly.drives.contains_key(name) {
                let (model_ports, power) =
                    sampled::install_with_circuit(&mut rt, &world, name, spec.clone())?;
                ports.extend(model_ports);
                bus.insert(name.clone(), power);
            } else {
                ports.extend(sampled::install(&mut rt, &world, name, spec.clone())?);
            }
        }
        sim_sense::install(&mut rt, &world, &models.sensors)?;
        let measurements = if path == LivePath::Legacy {
            profile.legacy_measurements.or(profile.measurements)
        } else {
            profile.measurements
        };
        if let Some(measurements) = measurements {
            let bytes = crate::tasks::install_live_measurements(
                &dir.join(measurements),
                &mut rt,
                &world,
                &sampled,
            )?;
            frame(&mut identity, &bytes);
        }
        if path == LivePath::Full {
            let circuit =
                TransientCircuit::new(assembly.net, assembly.setup.max_step, assembly.pins)?;
            rt.install_electrical(assembly.setup, &identity, circuit)?;
            let loads = assembly
                .drives
                .into_iter()
                .map(|(drive, ([plus, minus], current_source))| {
                    let (voltage_input, next_current) = bus
                        .remove(&drive)
                        .ok_or_else(|| format!("у «{drive}» нет пары каналов шины"))?;
                    Ok(LoadConnection {
                        plus,
                        minus,
                        current_source,
                        voltage_input,
                        next_current,
                    })
                })
                .collect::<Result<Vec<_>, String>>()?;
            if !bus.is_empty() {
                return Err("вход напряжения привода остался без схемы".into());
            }
            rt.bind_electrical_loads(loads)?;
            for (name, meter) in assembly.meters {
                let rate = Rate::hz(meter.rate_hz).ok_or("неверная частота измерителя питания")?;
                if meter.minimum_v < 0.0 || meter.source.trim().is_empty() {
                    return Err(format!("«{name}»: некорректный порог измерителя"));
                }
                let input = rt
                    .truth_id(&name)
                    .ok_or("нет электрического входа измерителя")?;
                let output = rt.register_observation(&name, "V")?;
                rt.register_sensor(
                    &name,
                    rate,
                    VoltageMeter {
                        input,
                        output,
                        low: meter.minimum_v,
                    },
                )?;
                rt.add_simplification(Simplification {
                    subject: name,
                    text: meter.source,
                    blocks_proof: true,
                })?;
            }
            for note in assembly.notes {
                rt.add_simplification(note)?;
            }
        }
        rt.add_simplification(Simplification {
            subject: "электрический опыт".into(),
            text: profile.source,
            blocks_proof: true,
        })?;
        let assets = FileAssets {
            base: base(&control_path).into(),
        };
        let spec = RunSpec {
            world: world.mjcf.as_bytes(),
            catalog: &identity,
            task: &profile_bytes,
            criteria: &profile_bytes,
            mode: "электрический опыт",
            horizon: until,
            model_ports: &ports,
        };
        if let Some(recording) = recording {
            control::install_replay(&mut rt, &config, &assets, spec, recording)?;
        } else {
            control::install(&mut rt, &config, &assets, spec)?;
        }
        Ok(Self { runtime: rt })
    }

    pub fn run_until(&mut self, until: Time) -> Result<Status, String> {
        self.runtime.run_electrical_until(until)
    }
}
