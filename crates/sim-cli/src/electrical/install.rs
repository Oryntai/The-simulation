use std::{
    collections::{BTreeMap, BTreeSet},
    path::{Path, PathBuf},
};

use serde::{Deserialize, Serialize};
use sim_drive::{DriveHandle, sampled};
use sim_process::{
    Analysis, SourcedRange,
    schematic::{
        BindingTarget, BoardContact, Catalog, Endpoint, ExpandedNetlist, Mode, Parameter,
        PinFunction, PinPhase, PinRole, Primitive, PrimitiveKind, ResetProfile, Schematic, Scope,
        Section,
        emi_reset::EmiTrial,
        sim_circuit::netlist::{Analysis as NetAnalysis, Element, ElementKind, Netlist, Waveform},
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
use sim_task::Outcome;
use sim_world::{CompiledWorld, Provenance, Simplification, WorldWithSchematics};

use super::input::{
    CatalogInput, CircuitProfile, LoadEquivalent, LoadRole, Meter, ProfileInput, ProfileProbe,
    SchematicInput, parse,
};
use crate::tasks::processes::Inputs;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LivePath {
    Legacy,
    Full,
}

pub struct LiveExperiment {
    pub runtime: Runtime,
}

enum LiveControl<'a> {
    Ordinary,
    Replay(ControlRecording),
    EmiReset { trial: &'a EmiTrial, coupled: bool },
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
pub(super) struct Wiring {
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
    pub(super) fn board(&self, name: &str) -> Result<&str, String> {
        self.board
            .iter()
            .find(|b| b.name == name)
            .map(|b| b.catalog.as_str())
            .ok_or_else(|| format!("нет установленной платы «{name}»"))
    }

    pub(super) fn board_profile(&self, board: &str, catalog: &Path) -> Result<PathBuf, String> {
        let part = self.board(board)?;
        if Path::new(part).file_name().and_then(|name| name.to_str()) != Some(part)
            || part.chars().any(char::is_control)
        {
            return Err("ожидалось имя позиции каталога платы".into());
        }
        Ok(catalog
            .join("boards/electrical")
            .join(format!("{part}.toml")))
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

/// Проверяет полярность физической ветви и её соответствие раскрытию MNA.
fn power_branch<'a, 'b>(
    net: &'a ExpandedNetlist,
    definition: &Schematic,
    catalog: &'b Catalog,
    address: &str,
    role: LoadRole,
) -> Result<(&'a Element, [String; 2], &'b Primitive), String> {
    let id = branch(net, address)?;
    let Section::Element { component, element } = section(address) else {
        return Err(format!("«{address}»: ожидается ветвь компонента"));
    };
    let part = catalog
        .part(&definition.components[&component].part)
        .unwrap();
    let primitive = &part.elements[&element];
    let expected = match role {
        LoadRole::Resistive => PrimitiveKind::Resistor,
        LoadRole::Current => PrimitiveKind::Current,
    };
    let el = net
        .netlist
        .elements
        .iter()
        .find(|el| el.name == id)
        .ok_or_else(|| format!("нет раскрытой ветви «{address}»"))?;
    let right_kind = matches!(
        (role, &el.kind),
        (LoadRole::Resistive, ElementKind::Resistor { .. })
            | (LoadRole::Current, ElementKind::CurrentSource(_))
    );
    if primitive.kind != expected || primitive.continuity || !right_kind {
        return Err(format!(
            "«{address}»: тип ветви не соответствует роли нагрузки"
        ));
    }
    let [plus, minus] = primitive.nodes.as_slice() else {
        return Err(format!("«{address}»: ожидается двухполюсная ветвь"));
    };
    if !part
        .pins
        .get(plus)
        .is_some_and(|p| p.role == PinRole::Power)
        || !part
            .pins
            .get(minus)
            .is_some_and(|p| p.role == PinRole::Return)
    {
        return Err(format!(
            "«{address}»: нужна ветвь от вывода питания к возврату"
        ));
    }
    let ends = [
        format!("{component}/{plus}"),
        format!("{component}/{minus}"),
    ];
    let nodes = pair(net, &ends)?;
    if nodes[0] == nodes[1] || el.nodes != nodes {
        return Err(format!("«{address}»: клеммы не совпадают с ветвью MNA"));
    }
    Ok((el, ends, primitive))
}

fn current_supply(
    net: &ExpandedNetlist,
    definition: &Schematic,
    catalog: &Catalog,
    address: &str,
    bound_ends: &[String],
) -> Result<([String; 2], String), String> {
    let (element, ends, _) = power_branch(net, definition, catalog, address, LoadRole::Current)?;
    if ends != bound_ends {
        return Err(format!("«{address}»: питание привязано к другим клеммам"));
    }
    Ok((pair(net, &ends)?, element.name.clone()))
}

pub(super) fn validate_owners(cfg: &CircuitProfile) -> Result<(), String> {
    let mut claimed = BTreeSet::new();
    let steps: BTreeSet<_> = cfg.steps.iter().map(|step| &step.branch).collect();
    for branch in cfg
        .drives
        .values()
        .chain(cfg.boards.values().map(|board| &board.branch))
        .chain(cfg.loads.values().map(|load| &load.branch))
        .chain(steps)
    {
        if !claimed.insert(branch) {
            return Err(format!("у ветви «{branch}» два владельца питания"));
        }
    }
    for name in cfg.loads.keys() {
        if name.trim().is_empty()
            || name.trim() != name
            || name.contains('/')
            || name.chars().any(char::is_control)
        {
            return Err(format!(
                "некорректное имя нагрузочного эквивалента «{name}»"
            ));
        }
        if cfg.drives.contains_key(name)
            || cfg.boards.contains_key(name)
            || cfg.observed_boards.contains_key(name)
        {
            return Err(format!("у нагрузки «{name}» два владельца питания"));
        }
    }
    Ok(())
}

#[derive(Clone, Serialize)]
pub(super) struct BoundLoad {
    branch: String,
    role: LoadRole,
    voltage_ends: [String; 2],
    pub(super) voltage_channel: String,
    pub(super) current_channel: String,
    physical_outcome: Outcome,
    measurement: String,
    #[serde(skip)]
    nodes: [String; 2],
    #[serde(skip)]
    current_branch: String,
}

pub(super) fn bound_loads(
    instance: &str,
    loads: &BTreeMap<String, LoadEquivalent>,
    net: &ExpandedNetlist,
    definition: &Schematic,
    catalog: &Catalog,
) -> Result<BTreeMap<String, BoundLoad>, String> {
    loads
        .iter()
        .map(|(name, load)| {
            let (el, voltage_ends, primitive) =
                power_branch(net, definition, catalog, &load.branch, load.role)?;
            let key = match load.role {
                LoadRole::Resistive => "ohm",
                LoadRole::Current => "ampere",
            };
            let measurement = primitive.parameters[key].measurement.clone();
            let prefix = format!("{instance}/{name}");
            Ok((
                name.clone(),
                BoundLoad {
                    branch: load.branch.clone(),
                    role: load.role,
                    nodes: pair(net, &voltage_ends)?,
                    voltage_ends,
                    voltage_channel: format!("{prefix}/напряжение нагрузки"),
                    current_channel: format!("{prefix}/ток нагрузки"),
                    physical_outcome: Outcome::NotAnalyzed,
                    measurement,
                    current_branch: el.name.clone(),
                },
            ))
        })
        .collect()
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
        files: &mut Inputs,
        identity: &mut Vec<u8>,
    ) -> Result<(), String> {
        validate_owners(&cfg)?;
        let definition = input.definition()?;
        let reference = definition.scope == Scope::Reference;
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
        let ends = definition
            .bindings
            .iter()
            .map(|binding| {
                (
                    binding.target.clone(),
                    binding
                        .ends
                        .iter()
                        .map(ToString::to_string)
                        .collect::<Vec<_>>(),
                )
            })
            .collect::<BTreeMap<_, _>>();
        let mut net = input.live_netlist(
            parts,
            cfg.diagnostic_connections.as_ref(),
            profile_dir,
            files,
        )?;
        if let Some(path) = cfg
            .diagnostic_connections
            .as_ref()
            .and_then(|d| d.parameters.as_ref())
        {
            frame(identity, files.capture(&profile_dir.join(path))?);
        }
        rename(&mut net, index);
        for (name, load) in bound_loads(instance, &cfg.loads, &net, &definition, parts)? {
            let [plus, minus] = load.nodes;
            self.probe(
                load.voltage_channel,
                ElectricalProbe::Voltage { plus, minus },
            )?;
            self.probe(
                load.current_channel,
                ElectricalProbe::Current {
                    branch: load.current_branch,
                },
            )?;
            self.notes.push(Simplification {
                subject: format!("{instance}/{name}"),
                text: format!(
                    "Электрический эквивалент «{}». {}",
                    load.branch, load.measurement
                ),
                blocks_proof: true,
            });
        }
        for (drive, address) in cfg.drives {
            let (nodes, name) = current_supply(
                &net,
                &definition,
                parts,
                &address,
                &ends[&BindingTarget::DrivePower {
                    drive: drive.clone(),
                }],
            )?;
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
            let ([plus, minus], current_source) = current_supply(
                &net,
                &definition,
                parts,
                &data.branch,
                &ends[&BindingTarget::BoardPower {
                    board: name.clone(),
                    rail: data.rail.clone(),
                }],
            )?;
            external(&mut net, &current_source)?;
            if data.currents_a.iter().any(|&current| current < 0.0) {
                return Err("ожидался неотрицательный ток потребления платы".into());
            }
            let profile_path = wiring.board_profile(&name, catalog)?;
            let catalog_board = match profile_path.try_exists() {
                Ok(true) => {
                    let bytes = files.capture(&profile_path)?;
                    let profile = CatalogInput::parse_board_profile(&profile_path, bytes)?;
                    if profile.board != board_type {
                        return Err("профиль записан для другой платы".into());
                    }
                    frame(identity, bytes);
                    Some(profile)
                }
                Ok(false) => None,
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
                ProfileProbe::Voltage { ends } => {
                    let [plus, minus] = pair(&net, &ends)?;
                    ElectricalProbe::Voltage { plus, minus }
                }
                ProfileProbe::Current { branch: address } => ElectricalProbe::Current {
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
        Self::assemble(
            world_path,
            profile_path,
            catalog,
            seed,
            until,
            path,
            LiveControl::Ordinary,
        )
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
            LiveControl::Replay(recording),
        )
    }

    pub(super) fn load_emi(
        world_path: &Path,
        profile_path: &Path,
        catalog: &Path,
        until: Time,
        trial: &EmiTrial,
        coupled: bool,
    ) -> Result<Self, String> {
        Self::assemble(
            world_path,
            profile_path,
            catalog,
            None,
            until,
            LivePath::Full,
            LiveControl::EmiReset { trial, coupled },
        )
    }

    fn assemble(
        world_path: &Path,
        profile_path: &Path,
        catalog: &Path,
        seed: Option<u64>,
        until: Time,
        path: LivePath,
        control: LiveControl<'_>,
    ) -> Result<Self, String> {
        let WorldWithSchematics {
            mut world,
            schematics,
        } = sim_world::load_and_compile_with_schematics(world_path)?;
        if let Some(seed) = seed {
            world.seed = seed;
        }
        let mut files = Inputs::default();
        let inputs = ProfileInput::load(profile_path, &mut files, schematics.keys())?;
        let (control_path, control_bytes) = inputs.control(&mut files)?;
        let wiring: Wiring = parse(&control_path, &control_bytes)?;
        let parts = if path == LivePath::Full {
            Some(CatalogInput::for_profile(
                catalog,
                Some(&inputs),
                &mut files,
            )?)
        } else {
            None
        };
        let ProfileInput {
            data: profile,
            bytes: profile_bytes,
            dir,
        } = inputs;
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
        if let Some(CatalogInput {
            sources,
            catalog: parts,
        }) = parts
        {
            for (path, bytes) in sources {
                if profile.parts.is_none() {
                    frame(
                        &mut identity,
                        path.file_name()
                            .ok_or("нет имени файла каталога")?
                            .as_encoded_bytes(),
                    );
                }
                frame(&mut identity, &bytes);
            }
            for (index, (instance, cfg)) in profile.circuits.into_iter().enumerate() {
                let scheme = &schematics[&instance];
                let input = SchematicInput::parse(scheme, files.capture(scheme)?.to_vec(), &parts)?;
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
                        &dir,
                        &profile.source,
                        &mut files,
                        &mut identity,
                    )
                    .map_err(|e| format!("{}: {e}", scheme.display()))?;
            }
        }
        let mut sampled = BTreeMap::new();
        for (name, file) in profile.sampled {
            let file = dir.join(file);
            let bytes = files.capture(&file)?;
            sampled.insert(name, parse::<sampled::Spec>(&file, bytes)?);
            frame(&mut identity, bytes);
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
        if let LiveControl::EmiReset { trial, coupled } = &control {
            rt.bind_emi_reset((*trial).clone(), *coupled)?;
        }
        if let LiveControl::Replay(recording) = control {
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
