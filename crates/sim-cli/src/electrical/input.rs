use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    path::{Path, PathBuf},
};

use serde::{Deserialize, Serialize, de::DeserializeOwned};
use sim_process::{SourcedRange, schematic::*};
use sim_runtime::electrical::BoardElectricalProfile;
use sim_signal::Time;
use sim_world::{Provenance, WorldWithSchematics};

use crate::tasks::processes::Inputs;

// Версия первого дискового формата схемы и каталога, независимая от формата мира.
const FORMAT: u32 = 1;

#[derive(Clone, Debug)]
pub struct SchematicInput {
    pub path: PathBuf,
    pub bytes: Vec<u8>,
    pub compiled: CompiledSchematic,
    pub modes: ModePlan,
    pub(super) crimps: BTreeSet<String>,
}

#[derive(Clone, Debug)]
pub struct CatalogInput {
    pub sources: BTreeMap<PathBuf, Vec<u8>>,
    pub catalog: Catalog,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Profile {
    pub(super) format: u32,
    pub(super) step_ns: u64,
    pub(super) source: String,
    pub(super) control: PathBuf,
    pub(super) stimuli: Option<PathBuf>,
    pub(super) measurements: Option<PathBuf>,
    pub(super) legacy_measurements: Option<PathBuf>,
    pub(super) parts: Option<PathBuf>,
    #[serde(default)]
    pub(super) sampled: BTreeMap<String, PathBuf>,
    pub(super) circuits: BTreeMap<String, CircuitProfile>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct CircuitProfile {
    #[serde(default)]
    pub(super) loads: BTreeMap<String, LoadEquivalent>,
    #[serde(default)]
    pub(super) drives: BTreeMap<String, String>,
    #[serde(default)]
    pub(super) reference_drive_min_v: BTreeMap<String, f64>,
    pub(super) diagnostic_connections: Option<DiagnosticConnections>,
    #[serde(default)]
    pub(super) boards: BTreeMap<String, BoardSupply>,
    #[serde(default)]
    pub(super) observed_boards: BTreeMap<String, String>,
    #[serde(default)]
    pub(super) steps: Vec<Step>,
    #[serde(default)]
    pub(super) probes: BTreeMap<String, ProfileProbe>,
    #[serde(default)]
    pub(super) meters: BTreeMap<String, Meter>,
    #[serde(default)]
    pub(super) voltage_observations: BTreeMap<String, VoltageObservation>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct DiagnosticConnections {
    pub(super) ohm: f64,
    pub(super) source: String,
    pub(super) parameters: Option<PathBuf>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Step {
    pub(super) branch: String,
    pub(super) at_ns: u64,
    pub(super) value: f64,
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub(super) enum ProfileProbe {
    Voltage { ends: [String; 2] },
    Current { branch: String },
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Meter {
    pub(super) rate_hz: u64,
    pub(super) minimum_v: f64,
    pub(super) source: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct VoltageObservation {
    pub(super) ends: [String; 2],
    pub(super) meter: Meter,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct BoardSupply {
    pub(super) rail: String,
    pub(super) branch: String,
    pub(super) currents_a: [f64; 4],
    pub(super) boot_ns: u64,
    pub(super) initially_in_reset: bool,
    pub(super) reference_reset: Option<ReferenceReset>,
    pub(super) phase_source: Option<ConfirmedSource>,
    #[serde(default)]
    pub(super) live_pins: Vec<u32>,
}

#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
pub(super) enum ConfirmedSource {
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
pub(super) struct ReferenceReset {
    pub(super) falling_v: f64,
    pub(super) rising_v: f64,
    pub(super) assert_ns: u64,
    pub(super) release_ns: u64,
    pub(super) source: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct LoadEquivalent {
    pub(super) branch: String,
    pub(super) role: LoadRole,
}

#[derive(Clone, Copy, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub(super) enum LoadRole {
    Resistive,
    Current,
}

pub(super) struct ProfileInput {
    pub(super) data: Profile,
    pub(super) bytes: Vec<u8>,
    pub(super) dir: PathBuf,
}

impl ProfileInput {
    pub(super) fn load<'a>(
        path: &Path,
        files: &mut Inputs,
        instances: impl Iterator<Item = &'a String>,
    ) -> Result<Self, String> {
        let bytes = files.capture(path)?.to_vec();
        let data: Profile = parse(path, &bytes)?;
        check_version(path, data.format)?;
        if data.step_ns == 0 || data.source.trim().is_empty() {
            return Err(located(path, "нужны ненулевой шаг и источник постановки"));
        }
        if data.circuits.is_empty() || data.circuits.keys().ne(instances) {
            return Err(located(
                path,
                "профиль должен покрывать все подключённые схемы мира",
            ));
        }
        Ok(Self {
            data,
            bytes,
            dir: path.parent().unwrap_or_else(|| Path::new(".")).into(),
        })
    }

    pub(super) fn control(&self, files: &mut Inputs) -> Result<(PathBuf, Vec<u8>), String> {
        let path = self.dir.join(&self.data.control);
        let mut bytes = files.capture(&path)?.to_vec();
        if let Some(stimuli) = &self.data.stimuli {
            bytes.push(b'\n');
            bytes.extend(files.capture(&self.dir.join(stimuli))?);
        }
        Ok((path, bytes))
    }
}

impl CatalogInput {
    pub(super) fn for_profile(
        root: &Path,
        profile: Option<&ProfileInput>,
        files: &mut Inputs,
    ) -> Result<Self, String> {
        // Поле parts в формате 1 указывает самостоятельный каталог старого профиля.
        if let Some(path) = profile.and_then(|p| p.data.parts.as_ref().map(|f| p.dir.join(f))) {
            let bytes = files.capture(&path)?.to_vec();
            return Ok(Self {
                catalog: parse_catalog(&path, &bytes)?,
                sources: BTreeMap::from([(path, bytes)]),
            });
        }
        let input = load_catalog(&root.join("electrical"))?;
        for (path, bytes) in &input.sources {
            if files.capture(path)? != bytes {
                return Err(located(path, "изменился каталог"));
            }
        }
        Ok(input)
    }
}

pub fn parse_catalog(path: &Path, bytes: &[u8]) -> Result<Catalog, String> {
    let parts = read_parts(path, bytes)?;
    Catalog::new(parts).map_err(|error| located(path, error))
}

pub fn load_catalog(directory: &Path) -> Result<CatalogInput, String> {
    let files = fs::read_dir(directory)
        .map_err(|error| located(directory, error))?
        .map(|entry| {
            entry
                .map(|entry| entry.path())
                .map_err(|error| located(directory, error))
        })
        .collect::<Result<BTreeSet<_>, _>>()?;
    let mut parts = BTreeMap::new();
    let mut sources = BTreeMap::new();
    for path in files
        .into_iter()
        .filter(|path| path.extension().is_some_and(|ext| ext == "toml"))
    {
        let bytes = fs::read(&path).map_err(|error| located(&path, error))?;
        for (id, part) in read_parts(&path, &bytes)? {
            if parts.insert(id.clone(), part).is_some() {
                return Err(located(
                    &path,
                    format!("позиция «{id}» уже определена в каталоге"),
                ));
            }
        }
        sources.insert(path, bytes);
    }
    Ok(CatalogInput {
        sources,
        catalog: Catalog::new(parts).map_err(|error| located(directory, error))?,
    })
}

pub fn parse_schematic(
    path: &Path,
    bytes: &[u8],
    catalog: &Catalog,
) -> Result<CompiledSchematic, String> {
    parse_schematic_and_modes(path, bytes, catalog).map(|(compiled, _, _)| compiled)
}

fn parse_schematic_and_modes(
    path: &Path,
    bytes: &[u8],
    catalog: &Catalog,
) -> Result<(CompiledSchematic, ModePlan, BTreeSet<String>), String> {
    let mut data: SchematicData = parse(path, bytes)?;
    check_version(path, data.format)?;
    let modes = std::mem::take(&mut data.modes)
        .convert()
        .map_err(|error| located(path, error))?;
    let crimps = data
        .connections
        .iter()
        .filter(|(_, connection)| connection.crimp)
        .map(|(id, _)| id.clone())
        .collect();
    let schematic = data.convert().map_err(|error| located(path, error))?;
    let compiled = compile(schematic, catalog).map_err(|error| located(path, error))?;
    compiled
        .validate_mode_plan(&modes)
        .map_err(|error| located(path, error))?;
    Ok((compiled, modes, crimps))
}

impl SchematicInput {
    pub fn parse(path: &Path, bytes: Vec<u8>, catalog: &Catalog) -> Result<Self, String> {
        let (compiled, modes, crimps) = parse_schematic_and_modes(path, &bytes, catalog)?;
        Ok(Self {
            path: path.into(),
            bytes,
            compiled,
            modes,
            crimps,
        })
    }

    pub(super) fn definition(&self) -> Result<Schematic, String> {
        let data: SchematicData = parse(&self.path, &self.bytes)?;
        data.convert().map_err(|error| located(&self.path, error))
    }

    pub(super) fn live_netlist(
        &self,
        catalog: &Catalog,
        diagnostic: Option<&DiagnosticConnections>,
        dir: &Path,
        files: &mut Inputs,
    ) -> Result<ExpandedNetlist, String> {
        if let Some(assumption) = diagnostic {
            let file = assumption.parameters.as_ref().map(|path| dir.join(path));
            let bytes = file
                .as_deref()
                .map(|path| files.capture(path))
                .transpose()?;
            self.diagnostic_netlist(
                catalog,
                assumption.ohm,
                &assumption.source,
                file.as_deref().zip(bytes),
            )
        } else {
            match self.compiled.netlist() {
                sim_process::Analysis::Analyzed { value, .. } => Ok(value),
                sim_process::Analysis::NotAnalyzed { reason } => Err(reason),
            }
        }
    }

    /// Подставляет сопротивление численного опыта только в ветви без замера.
    /// Исходная схема, режимы доказательства и сборочный лист этого значения не получают.
    pub(super) fn diagnostic_netlist(
        &self,
        catalog: &Catalog,
        ohm: f64,
        source: &str,
        estimates: Option<(&Path, &[u8])>,
    ) -> Result<ExpandedNetlist, String> {
        if !ohm.is_finite() || ohm <= 0.0 || source.trim().is_empty() {
            return Err("нужны конечное положительное сопротивление и источник опыта".into());
        }
        let mut schematic = self.definition()?;
        if schematic.scope != Scope::Assembly {
            return Err("подстановка предназначена для монтажной схемы".into());
        }
        let mut applied = false;
        for joint in schematic.connections.values_mut() {
            if joint.resistance_ohm.range.is_none() {
                joint.resistance_ohm.range = Some(SourcedRange {
                    min: ohm,
                    nominal: ohm,
                    max: ohm,
                    provenance: Provenance::Estimate(source.into()),
                });
                applied = true;
            }
        }
        if !applied {
            return Err("в схеме нет стыков без замера сопротивления".into());
        }
        let estimates = estimates
            .map(|(path, bytes)| diagnostic_catalog(path, bytes, catalog))
            .transpose()?;
        match compile(schematic, estimates.as_ref().unwrap_or(catalog))
            .map_err(|e| e.to_string())?
            .netlist()
        {
            sim_process::Analysis::Analyzed { value, .. } => Ok(value),
            sim_process::Analysis::NotAnalyzed { reason } => Err(reason),
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct DiagnosticCatalog {
    format: u32,
    parts: BTreeMap<String, BTreeMap<String, BTreeMap<String, RangeData>>>,
}

fn diagnostic_catalog(path: &Path, bytes: &[u8], catalog: &Catalog) -> Result<Catalog, String> {
    let data: DiagnosticCatalog = parse(path, bytes)?;
    check_version(path, data.format)?;
    let mut parts = catalog.parts().clone();
    for (id, elements) in data.parts {
        let part = parts.get_mut(&id).ok_or_else(|| located(path, &id))?;
        for (name, parameters) in elements {
            let element = part
                .elements
                .get_mut(&name)
                .ok_or_else(|| located(path, format!("неизвестная ветвь {id}/{name}")))?;
            for (key, range) in parameters {
                let label = format!("{id}/{name}/{key}");
                let parameter = element
                    .parameters
                    .get_mut(&key)
                    .ok_or_else(|| located(path, format!("неизвестный параметр {label}")))?;
                if parameter.range.is_some() || !matches!(range.source, Provenance::Estimate(_)) {
                    return Err(located(
                        path,
                        format!("для {label} нужна оценка отсутствующего параметра"),
                    ));
                }
                parameter.range = Some(range.into());
            }
        }
    }
    Catalog::new(parts).map_err(|e| located(path, e))
}

pub fn load_schematic(path: &Path, catalog: &Catalog) -> Result<SchematicInput, String> {
    let bytes = fs::read(path).map_err(|error| located(path, error))?;
    SchematicInput::parse(path, bytes, catalog)
}

/// Разрешает схемы по экземплярам и сверяет привязки с уже разрешёнными функциями платы и моделей.
pub fn load_world_schematics(
    world: &WorldWithSchematics,
    catalog: &Catalog,
    installed: &BTreeMap<String, BTreeSet<BindingTarget>>,
) -> Result<BTreeMap<String, SchematicInput>, String> {
    let empty = BTreeSet::new();
    world
        .schematics
        .iter()
        .map(|(instance, path)| {
            let input = load_schematic(path, catalog)?;
            let bindings = installed.get(instance).unwrap_or(&empty);
            for target in bindings {
                let valid =
                    match target {
                        BindingTarget::DrivePower { drive } => world.world.drives.iter().any(|d| {
                            d.owner == *instance && d.name == format!("{instance}/{drive}")
                        }),
                        BindingTarget::SensorPower { sensor } => world
                            .world
                            .sensors
                            .iter()
                            .any(|s| s.name == format!("{instance}/{sensor}")),
                        _ => true,
                    };
                if !valid {
                    return Err(located(
                        path,
                        format!(
                            "экземпляр «{instance}»: неизвестная установленная модель {target:?}"
                        ),
                    ));
                }
            }
            input
                .compiled
                .validate_bindings(bindings)
                .map_err(|error| located(path, format!("экземпляр «{instance}»: {error}")))?;
            Ok((instance.clone(), input))
        })
        .collect()
}

fn located(path: &Path, error: impl std::fmt::Display) -> String {
    format!("{}: {error}", path.display())
}

pub(super) fn parse<T: DeserializeOwned>(path: &Path, bytes: &[u8]) -> Result<T, String> {
    let text = std::str::from_utf8(bytes).map_err(|e| located(path, e))?;
    let table: toml::Table = toml::from_str(text).map_err(|e| located(path, e))?;
    fn nonfinite(value: &toml::Value) -> Option<String> {
        match value {
            toml::Value::Float(value) if !value.is_finite() => Some(String::new()),
            toml::Value::Array(values) => values
                .iter()
                .enumerate()
                .find_map(|(i, value)| nonfinite(value).map(|suffix| format!("[{i}]{suffix}"))),
            toml::Value::Table(values) => values
                .iter()
                .find_map(|(key, value)| nonfinite(value).map(|suffix| format!(".{key}{suffix}"))),
            _ => None,
        }
    }
    if let Some(field) = table
        .iter()
        .find_map(|(key, value)| nonfinite(value).map(|suffix| format!("{key}{suffix}")))
    {
        return Err(located(
            path,
            format!("{field}: допустимы только конечные числа"),
        ));
    }
    table.try_into().map_err(|e| located(path, e))
}

fn check_version(path: &Path, format: u32) -> Result<(), String> {
    if format != FORMAT {
        return Err(located(
            path,
            format!("format: версия {format}, ожидается {FORMAT}"),
        ));
    }
    Ok(())
}

fn unique(values: Vec<String>, context: &str) -> Result<BTreeSet<String>, String> {
    let mut keys = BTreeSet::new();
    for value in values {
        if !keys.insert(value.clone()) {
            return Err(format!("{context}: «{value}» указано дважды"));
        }
    }
    Ok(keys)
}

fn read_parts(path: &Path, bytes: &[u8]) -> Result<BTreeMap<String, Part>, String> {
    let data: CatalogData = parse(path, bytes)?;
    check_version(path, data.format)?;
    let parts = data
        .parts
        .into_iter()
        .map(|(id, part)| {
            let part = part
                .convert()
                .map_err(|e| located(path, format!("позиция «{id}»: {e}")))?;
            Ok((id, part))
        })
        .collect::<Result<BTreeMap<_, _>, String>>()?;
    // Проверка здесь сохраняет адрес именно того файла, в котором определена позиция.
    Catalog::new(parts.clone()).map_err(|e| located(path, e))?;
    Ok(parts)
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CatalogData {
    format: u32,
    parts: BTreeMap<String, PartData>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PartData {
    #[serde(with = "PartKindData")]
    kind: PartKind,
    source: String,
    pins: BTreeMap<String, PinData>,
    #[serde(default)]
    internal_nodes: Vec<String>,
    #[serde(default)]
    elements: BTreeMap<String, PrimitiveData>,
    #[serde(default)]
    ratings: BTreeMap<String, RatingData>,
    model_gap: Option<String>,
    #[serde(default)]
    reference_only: bool,
}

impl PartData {
    fn convert(self) -> Result<Part, String> {
        Ok(Part {
            kind: self.kind,
            source: self.source,
            pins: self
                .pins
                .into_iter()
                .map(|(name, pin)| (name, pin.into()))
                .collect(),
            internal_nodes: unique(self.internal_nodes, "internal_nodes")?,
            elements: self
                .elements
                .into_iter()
                .map(|(name, element)| (name, element.into()))
                .collect(),
            ratings: self
                .ratings
                .into_iter()
                .map(|(name, rating)| (name, rating.into()))
                .collect(),
            model_gap: self.model_gap,
            reference_only: self.reference_only,
        })
    }
}

#[derive(Deserialize)]
#[serde(remote = "PartKind", rename_all = "snake_case")]
enum PartKindData {
    Resistor,
    PolarCapacitor,
    Capacitor,
    Diode,
    Nmos,
    Pmos,
    SwitchModule,
    LinearRegulator,
    SwitchingRegulator,
    ChargePowerModule,
    UsbSource,
    Battery,
    BenchSupply,
    Wire,
    ScrewTerminal,
    Header,
    SolderJoint,
    Other,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PinData {
    #[serde(with = "ContactKindData")]
    contact: ContactKind,
    family: Option<String>,
    key: Option<String>,
    #[serde(with = "PinRoleData")]
    role: PinRole,
}

impl From<PinData> for Pin {
    fn from(pin: PinData) -> Self {
        Self {
            contact: Contact {
                kind: pin.contact,
                family: pin.family,
                key: pin.key,
            },
            role: pin.role,
        }
    }
}

#[derive(Deserialize)]
#[serde(remote = "ContactKind", rename_all = "snake_case")]
enum ContactKindData {
    Plug,
    Socket,
    Terminal,
    Pad,
    Bare,
}

#[derive(Deserialize)]
#[serde(remote = "PinRole", rename_all = "snake_case")]
enum PinRoleData {
    Passive,
    Power,
    Return,
    Control,
    SignalOut(String),
    SignalIn(String),
    SignalIo(String),
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PrimitiveData {
    #[serde(with = "PrimitiveKindData")]
    kind: PrimitiveKind,
    nodes: Vec<String>,
    parameters: BTreeMap<String, ParameterData>,
    #[serde(default)]
    continuity: bool,
}

impl From<PrimitiveData> for Primitive {
    fn from(data: PrimitiveData) -> Self {
        Self {
            kind: data.kind,
            nodes: data.nodes,
            continuity: data.continuity,
            parameters: data
                .parameters
                .into_iter()
                .map(|(name, p)| (name, p.into()))
                .collect(),
        }
    }
}

#[derive(Deserialize)]
#[serde(remote = "PrimitiveKind", rename_all = "snake_case")]
enum PrimitiveKindData {
    Resistor,
    Capacitor,
    Inductor,
    Voltage,
    Current,
    Diode,
    Nmos,
    Pmos,
    Switch,
    Vcvs,
    Vccs,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RatingData {
    #[serde(with = "QuantityData")]
    quantity: Quantity,
    nodes: [String; 2],
    limit: ParameterData,
    conditions: String,
}

impl From<RatingData> for Rating {
    fn from(data: RatingData) -> Self {
        Self {
            quantity: data.quantity,
            nodes: data.nodes,
            limit: data.limit.into(),
            conditions: data.conditions,
        }
    }
}

#[derive(Deserialize)]
#[serde(remote = "Quantity", rename_all = "snake_case")]
enum QuantityData {
    CurrentA,
    VoltageV,
    ReverseVoltageV,
    PowerW,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ParameterData {
    range: Option<RangeData>,
    measurement: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RangeData {
    min: f64,
    nominal: f64,
    max: f64,
    #[serde(with = "ProvenanceData")]
    source: Provenance,
}

#[derive(Deserialize)]
#[serde(remote = "Provenance", rename_all = "snake_case")]
enum ProvenanceData {
    Datasheet(String),
    Paper(String),
    Measured(String),
    Derived(String),
    Estimate(String),
}

impl From<ParameterData> for Parameter {
    fn from(data: ParameterData) -> Self {
        Self {
            range: data.range.map(Into::into),
            measurement: data.measurement,
        }
    }
}

impl From<RangeData> for SourcedRange {
    fn from(range: RangeData) -> Self {
        Self {
            min: range.min,
            nominal: range.nominal,
            max: range.max,
            provenance: range.source,
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct SchematicData {
    format: u32,
    name: String,
    #[serde(with = "ScopeData")]
    scope: Scope,
    nets: Vec<String>,
    components: BTreeMap<String, ComponentData>,
    #[serde(default)]
    connections: BTreeMap<String, ConnectionData>,
    #[serde(default)]
    rails: BTreeMap<String, RailData>,
    #[serde(default)]
    bindings: Vec<BindingData>,
    #[serde(default)]
    modes: ModePlanData,
}

impl SchematicData {
    fn convert(self) -> Result<Schematic, String> {
        let connections = self
            .connections
            .into_iter()
            .map(|(id, data)| {
                let connection = data
                    .convert()
                    .map_err(|e| format!("connections.{id}: {e}"))?;
                Ok((id, connection))
            })
            .collect::<Result<_, String>>()?;
        let rails = self
            .rails
            .into_iter()
            .map(|(id, data)| {
                let ends = endpoints(data.ends).map_err(|e| format!("rails.{id}: {e}"))?;
                Ok((
                    id,
                    Rail {
                        ends,
                        reverse_limit_v: data.reverse_limit_v.into(),
                    },
                ))
            })
            .collect::<Result<_, String>>()?;
        let bindings = self
            .bindings
            .into_iter()
            .enumerate()
            .map(|(i, data)| {
                let ends = data
                    .ends
                    .into_iter()
                    .map(endpoint)
                    .collect::<Result<_, _>>()
                    .map_err(|e| format!("bindings[{i}]: {e}"))?;
                Ok(Binding {
                    target: data.target,
                    ends,
                })
            })
            .collect::<Result<_, String>>()?;
        Ok(Schematic {
            name: self.name,
            scope: self.scope,
            nets: unique(self.nets, "nets")?,
            connections,
            rails,
            bindings,
            components: self
                .components
                .into_iter()
                .map(|(id, data)| {
                    (
                        id,
                        Component {
                            part: data.part,
                            nets: data.nets,
                        },
                    )
                })
                .collect(),
        })
    }
}

#[derive(Deserialize)]
#[serde(remote = "Scope", rename_all = "snake_case")]
enum ScopeData {
    Assembly,
    Reference,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ComponentData {
    part: String,
    nets: BTreeMap<String, String>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ConnectionData {
    #[serde(with = "JointKindData")]
    kind: JointKind,
    ends: [String; 2],
    resistance_ohm: ParameterData,
    current_limit_a: ParameterData,
    #[serde(default)]
    crimp: bool,
    #[serde(default)]
    after: Vec<String>,
}

impl ConnectionData {
    fn convert(self) -> Result<Connection, String> {
        if self.crimp && self.kind != JointKind::Splice {
            return Err("обжим гильзой требует стыка двух зачищенных жил".into());
        }
        Ok(Connection {
            kind: self.kind,
            ends: endpoints(self.ends)?,
            resistance_ohm: self.resistance_ohm.into(),
            current_limit_a: self.current_limit_a.into(),
            after: unique(self.after, "after")?,
        })
    }
}

#[derive(Deserialize)]
#[serde(remote = "JointKind", rename_all = "snake_case")]
enum JointKindData {
    Plug,
    Terminal,
    Solder,
    Splice,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RailData {
    ends: [String; 2],
    reverse_limit_v: ParameterData,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct BindingData {
    ends: Vec<String>,
    #[serde(with = "BindingTargetData")]
    target: BindingTarget,
}

#[derive(Deserialize)]
#[serde(
    remote = "BindingTarget",
    tag = "kind",
    rename_all = "snake_case",
    deny_unknown_fields
)]
enum BindingTargetData {
    BoardPin {
        board: String,
        pin: String,
        #[serde(with = "PinFunctionData")]
        function: PinFunction,
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
enum PinFunctionData {
    GpioInput,
    GpioOutput,
    Adc,
    Pwm,
    Uart,
    I2c,
    Spi,
}

fn endpoint(value: String) -> Result<Endpoint, String> {
    let (component, pin) = value
        .split_once('/')
        .filter(|(a, b)| !a.is_empty() && !b.is_empty() && !b.contains('/'))
        .ok_or_else(|| format!("«{value}»: ожидается адрес компонент/вывод"))?;
    Ok(Endpoint {
        component: component.into(),
        pin: pin.into(),
    })
}

fn endpoints([a, b]: [String; 2]) -> Result<[Endpoint; 2], String> {
    Ok([endpoint(a)?, endpoint(b)?])
}

impl CatalogInput {
    pub fn parse_board_profile(
        path: &Path,
        bytes: &[u8],
    ) -> Result<BoardElectricalProfile, String> {
        let data: BoardProfileData = parse(path, bytes)?;
        check_version(path, data.format)?;
        let mut reset_pins = BTreeMap::new();
        for item in data.reset_pins {
            for number in item.pins {
                if reset_pins
                    .insert(number, item.state.clone().convert())
                    .is_some()
                {
                    return Err(located(path, format!("контакт {number} определён дважды")));
                }
            }
        }
        let profile = BoardElectricalProfile {
            board: data.board,
            reset: data.reset.into(),
            reset_pins,
        };
        profile.validate_builtin().map_err(|e| located(path, e))?;
        Ok(profile)
    }

    pub fn load_board_profiles(
        directory: &Path,
    ) -> Result<BTreeMap<String, (BoardElectricalProfile, PathBuf, Vec<u8>)>, String> {
        let files = fs::read_dir(directory)
            .map_err(|e| located(directory, e))?
            .map(|entry| {
                entry
                    .map(|entry| entry.path())
                    .map_err(|e| located(directory, e))
            })
            .collect::<Result<BTreeSet<_>, _>>()?;
        let mut entries = BTreeMap::new();
        for path in files
            .into_iter()
            .filter(|path| path.extension().is_some_and(|ext| ext == "toml"))
        {
            let bytes = fs::read(&path).map_err(|e| located(&path, e))?;
            let profile = Self::parse_board_profile(&path, &bytes)?;
            if entries.contains_key(&profile.board) {
                return Err(located(
                    &path,
                    format!("профиль «{}» определён дважды", profile.board),
                ));
            }
            entries.insert(profile.board.clone(), (profile, path, bytes));
        }
        Ok(entries)
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct BoardProfileData {
    format: u32,
    board: String,
    reset: ResetProfileData,
    #[serde(default)]
    reset_pins: Vec<ResetPinsData>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ResetPinsData {
    pins: Vec<u32>,
    state: SourcedPinStateData,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ResetProfileData {
    falling_v: ParameterData,
    rising_v: ParameterData,
    assert_delay_s: ParameterData,
    release_delay_s: ParameterData,
}

impl From<ResetProfileData> for ResetProfile {
    fn from(data: ResetProfileData) -> Self {
        Self {
            falling_v: data.falling_v.into(),
            rising_v: data.rising_v.into(),
            assert_delay_s: data.assert_delay_s.into(),
            release_delay_s: data.release_delay_s.into(),
        }
    }
}

#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct ModePlanData {
    #[serde(default)]
    source_disconnects: BTreeMap<String, String>,
    #[serde(default)]
    cases: Vec<CaseData>,
}

impl ModePlanData {
    fn convert(self) -> Result<ModePlan, String> {
        let mut cases = BTreeMap::new();
        for case in self.cases {
            let (mode, experiment) = case.convert()?;
            if cases.insert(mode.clone(), experiment).is_some() {
                return Err(format!("повтор опыта {mode:?}"));
            }
        }
        Ok(ModePlan {
            source_disconnects: self.source_disconnects,
            cases,
        })
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CaseData {
    mode: ModeData,
    #[serde(with = "ProvenanceData")]
    source: Provenance,
    window: Option<WindowData>,
    #[serde(default)]
    sources: Vec<TraceData>,
    #[serde(default)]
    pins: Vec<PinSetupData>,
    probes: Vec<ProbeData>,
    sampling: Option<SamplingData>,
}

impl CaseData {
    fn convert(self) -> Result<(Mode, ModeCase), String> {
        let mode = self.mode.into();
        let mut pins = BTreeMap::new();
        let mut sources = BTreeMap::new();
        let mut probes = BTreeSet::new();
        for setup in self.pins {
            let (contact, setup) = setup.convert();
            if pins.insert(contact, setup).is_some() {
                return Err("повтор контакта в опыте".into());
            }
        }
        for trace in self.sources {
            if sources
                .insert(
                    trace.branch.into(),
                    SourceTrace {
                        points: trace
                            .points
                            .into_iter()
                            .map(|(t, v)| (Time(t), v))
                            .collect(),
                        source: trace.source,
                    },
                )
                .is_some()
            {
                return Err("повтор внешнего источника в опыте".into());
            }
        }
        for probe in self.probes {
            if !probes.insert(probe.convert()?) {
                return Err("повтор наблюдения в опыте".into());
            }
        }
        Ok((
            mode,
            ModeCase {
                source: self.source,
                window: self.window.map(WindowData::convert).transpose()?,
                sources,
                pins,
                probes,
                sampling: self.sampling.map(Into::into),
            },
        ))
    }
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum ModeData {
    AllOff {},
    UsbOnly {},
    MainOnly {},
    Both {},
    Reset {},
    Boot { elapsed_ns: u64 },
    Normal {},
    Start {},
    Stop {},
    Stall {},
    LoadOff {},
    OpenWire { wire: String },
}

impl From<ModeData> for Mode {
    fn from(data: ModeData) -> Self {
        match data {
            ModeData::AllOff {} => Self::AllOff,
            ModeData::UsbOnly {} => Self::UsbOnly,
            ModeData::MainOnly {} => Self::MainOnly,
            ModeData::Both {} => Self::Both,
            ModeData::Reset {} => Self::Reset,
            ModeData::Boot { elapsed_ns } => Self::Boot(Time(elapsed_ns)),
            ModeData::Normal {} => Self::Normal,
            ModeData::Start {} => Self::Start,
            ModeData::Stop {} => Self::Stop,
            ModeData::Stall {} => Self::Stall,
            ModeData::LoadOff {} => Self::LoadOff,
            ModeData::OpenWire { wire } => Self::OpenWire(wire),
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct WindowData {
    step_ns: u64,
    end_ns: u64,
    /// Если задано, перечисляются все C и L со знаком по порядку узлов.
    reactive: Option<Vec<ReactiveData>>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ReactiveData {
    branch: SectionData,
    value: f64,
}

impl WindowData {
    fn convert(self) -> Result<Window, String> {
        let initial = if let Some(items) = self.reactive {
            let mut reactive = BTreeMap::new();
            for item in items {
                if reactive.insert(item.branch.into(), item.value).is_some() {
                    return Err("повтор исходного состояния C/L".into());
                }
            }
            InitialState::Reactive(reactive)
        } else {
            InitialState::OperatingPoint
        };
        Ok(Window {
            step: Time(self.step_ns),
            end: Time(self.end_ns),
            initial,
        })
    }
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum SectionData {
    Connection { id: String },
    Element { component: String, element: String },
}

impl From<SectionData> for Section {
    fn from(data: SectionData) -> Self {
        match data {
            SectionData::Connection { id } => Self::Connection(id),
            SectionData::Element { component, element } => Self::Element { component, element },
        }
    }
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum ProbeData {
    Voltage { ends: [String; 2] },
    Current { branch: SectionData },
    Power { branch: SectionData },
}

impl ProbeData {
    fn convert(self) -> Result<Probe, String> {
        Ok(match self {
            Self::Voltage { ends } => {
                let [a, b] = endpoints(ends)?;
                Probe::Voltage(a, b)
            }
            Self::Current { branch } => Probe::Current(branch.into()),
            Self::Power { branch } => Probe::Power(branch.into()),
        })
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct TraceData {
    branch: SectionData,
    points: Vec<(u64, f64)>,
    #[serde(with = "ProvenanceData")]
    source: Provenance,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct SamplingData {
    seed: u64,
    count: usize,
    connection_ohm: BTreeMap<String, RangeData>,
}

impl From<SamplingData> for Sampling {
    fn from(data: SamplingData) -> Self {
        Self {
            seed: data.seed,
            count: data.count,
            connection_ohm: data
                .connection_ohm
                .into_iter()
                .map(|(name, r)| {
                    (
                        name,
                        SourcedRange {
                            min: r.min,
                            nominal: r.nominal,
                            max: r.max,
                            provenance: r.source,
                        },
                    )
                })
                .collect(),
        }
    }
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct SourcedPinStateData {
    state: PinStateData,
    #[serde(with = "ProvenanceData")]
    source: Provenance,
}

impl SourcedPinStateData {
    fn convert(self) -> SourcedPinState {
        SourcedPinState {
            state: self.state.into(),
            source: self.source,
        }
    }
}

#[derive(Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum PinStateData {
    High,
    Low,
    Input,
    PullUp,
    PullDown,
    HighZ,
}

impl From<PinStateData> for PinState {
    fn from(data: PinStateData) -> Self {
        match data {
            PinStateData::High => Self::High,
            PinStateData::Low => Self::Low,
            PinStateData::Input => Self::Input,
            PinStateData::PullUp => Self::InputPull(Pull::Up),
            PinStateData::PullDown => Self::InputPull(Pull::Down),
            PinStateData::HighZ => Self::HighZ,
        }
    }
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum PinPhaseData {
    Unpowered {},
    Reset {},
    Boot { elapsed_ns: u64 },
    Working { pin: SourcedPinStateData },
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PinSetupData {
    board: String,
    pin: String,
    rail: String,
    phase: PinPhaseData,
    model: PinModelData,
    unpowered: Option<SourcedPinStateData>,
    reset: Option<SourcedPinStateData>,
    #[serde(default)]
    boot: Vec<BootStageData>,
}

impl PinSetupData {
    fn convert(self) -> (BoardContact, PinSetup) {
        let phase = match self.phase {
            PinPhaseData::Unpowered {} => PinPhase::Unpowered,
            PinPhaseData::Reset {} => PinPhase::Reset,
            PinPhaseData::Boot { elapsed_ns } => PinPhase::Boot {
                elapsed: Time(elapsed_ns),
            },
            PinPhaseData::Working { pin } => PinPhase::Working(pin.convert()),
        };
        let profile = PinProfile {
            model: self.model.into(),
            unpowered: self.unpowered.map(SourcedPinStateData::convert),
            reset: self.reset.map(SourcedPinStateData::convert),
            boot: self
                .boot
                .into_iter()
                .map(|s| BootStage {
                    duration_s: s.duration_s.into(),
                    pin: s.pin.convert(),
                })
                .collect(),
        };
        (
            BoardContact {
                board: self.board,
                pin: self.pin,
            },
            PinSetup {
                rail: self.rail,
                profile,
                phase,
            },
        )
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct BootStageData {
    duration_s: ParameterData,
    pin: SourcedPinStateData,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PinModelData {
    high_ohm: Option<ParameterData>,
    low_ohm: Option<ParameterData>,
    pull_up_ohm: Option<ParameterData>,
    pull_down_ohm: Option<ParameterData>,
    input_ohm: Option<ParameterData>,
    high_z_ohm: Option<ParameterData>,
    upper_clamp: Option<ClampData>,
    lower_clamp: Option<ClampData>,
}

impl From<PinModelData> for PinModel {
    fn from(data: PinModelData) -> Self {
        let resistance = |p: Option<ParameterData>, name| {
            p.map_or_else(
                || Parameter {
                    range: None,
                    measurement: format!(
                        "измерить сопротивление контакта ({name}) в условиях опыта"
                    ),
                },
                Into::into,
            )
        };
        Self {
            high_ohm: resistance(data.high_ohm, "выход высокий"),
            low_ohm: resistance(data.low_ohm, "выход низкий"),
            pull_up_ohm: resistance(data.pull_up_ohm, "подтяжка к VDD"),
            pull_down_ohm: resistance(data.pull_down_ohm, "подтяжка к VSS"),
            input_ohm: resistance(data.input_ohm, "вход"),
            high_z_ohm: resistance(data.high_z_ohm, "высокий импеданс"),
            upper_clamp: data.upper_clamp.map_or(ClampPath::Unknown, Into::into),
            lower_clamp: data.lower_clamp.map_or(ClampPath::Unknown, Into::into),
        }
    }
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum ClampData {
    Absent {
        #[serde(with = "ProvenanceData")]
        source: Provenance,
    },
    Diode {
        saturation_a: ParameterData,
        ideality: ParameterData,
        series_ohm: ParameterData,
        injection_limit_a: ParameterData,
    },
}

impl From<ClampData> for ClampPath {
    fn from(data: ClampData) -> Self {
        match data {
            ClampData::Absent { source } => Self::Absent(source),
            ClampData::Diode {
                saturation_a,
                ideality,
                series_ohm,
                injection_limit_a,
            } => Self::Diode(ClampDiode {
                saturation_a: saturation_a.into(),
                ideality: ideality.into(),
                series_ohm: series_ohm.into(),
                injection_limit_a: injection_limit_a.into(),
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_process::{Analysis, work::HazardCheck};

    const FIXTURE: &str =
        include_str!("../../../../devices/electrical/corpus/assembly_mismatch/fixed.toml");
    const PARTS: &[u8] = include_bytes!("../../../../catalog/electrical/reference.toml");

    fn end(component: &str, pin: &str) -> Endpoint {
        Endpoint {
            component: component.into(),
            pin: pin.into(),
        }
    }

    fn load(mode_data: &str) -> Result<SchematicInput, String> {
        let path = Path::new("блок.toml");
        let mut parts = parse_catalog(path, PARTS)?.parts().clone();
        // У испытательного источника выход имеет силовое назначение и одну подключённую клемму.
        parts
            .get_mut("reference_source")
            .unwrap()
            .pins
            .get_mut("plus")
            .unwrap()
            .role = PinRole::Power;
        let catalog = Catalog::new(parts).unwrap();
        SchematicInput::parse(
            path,
            format!("{FIXTURE}\n{mode_data}").into_bytes(),
            &catalog,
        )
    }

    fn value<T: std::fmt::Debug>(result: Analysis<T>) -> T {
        match result {
            Analysis::Analyzed { value, .. } => value,
            error => panic!("{error:?}"),
        }
    }

    fn near(received: f64, expected: f64) {
        // Для точных параметров линейной цепи 1 нВ/1 нА покрывает округление MNA.
        assert!(
            (received - expected).abs() < 1e-9,
            "{received} != {expected}"
        );
    }

    #[test]
    fn диагностические_оценки_дополняют_только_пробелы() {
        let path = Path::new("блок.toml");
        let catalog = parse_catalog(
            path,
            include_bytes!("../../../../catalog/electrical/components.toml"),
        )
        .unwrap();
        // Стык 0,1 Ом из схемы корпуса Э1, только как оценка опыта.
        let data = r#"
format = 1
[parts.resistor.r]
ohm = { min = 0.1, nominal = 0.1, max = 0.1, source = { estimate = "стык Э1" } }
"#;
        let temporary = diagnostic_catalog(path, data.as_bytes(), &catalog).unwrap();
        let get_parameter = |catalog: &Catalog| {
            catalog.part("resistor").unwrap().elements["r"].parameters["ohm"].clone()
        };
        let range = get_parameter(&temporary).range.unwrap();
        assert_eq!(range.nominal, 0.1);
        assert!(!range.can_prove() && get_parameter(&catalog).range.is_none());
        assert!(diagnostic_catalog(path, data.as_bytes(), &temporary).is_err());
        for invalid in [
            data.replace("nominal = 0.1", "nominal = nan"),
            data.replace("nominal = 0.1", "nominal = inf"),
            data.replace("estimate =", "datasheet ="),
            data.replace("resistor.r", "resistor.unknown"),
        ] {
            assert!(diagnostic_catalog(path, invalid.as_bytes(), &catalog).is_err());
        }
    }

    #[test]
    fn electrical_modes_dc_wire_and_strict_parse() {
        let data = r#"
[modes.source_disconnects]
supply = "source_end"

[[modes.cases]]
mode = { kind = "main_only" }
source = { derived = "фиксированные параметры стенда" }
probes = [{ kind = "voltage", ends = ["load/input", "load/return"] }, { kind = "current", branch = { kind = "connection", id = "source_end" } }]

[[modes.cases]]
mode = { kind = "all_off" }
source = { derived = "контакт подачи разомкнут до пуска" }
probes = [{ kind = "voltage", ends = ["load/input", "load/return"] }]

[[modes.cases]]
mode = { kind = "open_wire", wire = "return" }
source = { derived = "единственный обрыв обратного проводника" }
probes = [{ kind = "voltage", ends = ["load/input", "load/return"] }, { kind = "current", branch = { kind = "connection", id = "return" } }]
"#;
        let input = load(data).unwrap();
        let reports = input.compiled.run_modes(&input.modes).unwrap();
        assert_eq!(
            reports
                .iter()
                .map(|r| match r {
                    ModeCoverage::Inapplicable { number, .. } => *number,
                    ModeCoverage::Case(r) => r.mode.number(),
                })
                .collect::<BTreeSet<_>>(),
            (1..=9).collect()
        );
        let report = |mode: Mode| {
            reports
                .iter()
                .find_map(|row| match row {
                    ModeCoverage::Case(r) if r.mode == mode => Some(r),
                    _ => None,
                })
                .unwrap()
        };
        let probe = Probe::Voltage(end("load", "input"), end("load", "return"));
        let main = report(Mode::MainOnly);
        let envelope = value(main.envelope.clone());
        // Значения из исходного TOML: 5 В, 1 Ом, 100 Ом, 0,2 Ом и три стыка по 0,1 Ом.
        let expected_a = 5.0 / (1.0 + 100.0 + 0.2 + 3.0 * 0.1);
        near(envelope[&probe].max.value, expected_a * 100.0);
        near(
            value(report(Mode::AllOff).nominal.clone()).values[&probe][0],
            0.0,
        );
        let opened = value(report(Mode::OpenWire("return".into())).nominal.clone());
        near(
            opened.values[&Probe::Current(Section::Connection("return".into()))][0],
            0.0,
        );
        near(opened.values[&probe][0], 0.0);
        assert!(matches!(
            report(Mode::Normal).envelope,
            Analysis::NotAnalyzed { .. }
        ));
        let planned = value(
            input
                .compiled
                .prepare_mode(&input.modes, &Mode::MainOnly)
                .unwrap(),
        );
        let reference = planned
            .compare_ngspice(&BTreeMap::from([(
                probe.clone(),
                ReferenceCheck::DcVoltage,
            )]))
            .unwrap();
        assert!(reference[&probe].matches());
        near(reference[&probe].max.reference, expected_a * 100.0);
        let measured_limit = |a| SourcedRange {
            min: a,
            nominal: a,
            max: a,
            provenance: Provenance::Measured("порог синтетического стенда".into()),
        };
        assert!(matches!(
            planned
                .check_current(
                    &Section::Connection("source_end".into()),
                    CurrentDirection::Absolute,
                    Some(&measured_limit(expected_a / 2.0))
                )
                .unwrap(),
            HazardCheck::Violation { .. }
        ));
        assert!(matches!(
            planned
                .check_current(
                    &Section::Connection("source_end".into()),
                    CurrentDirection::Reverse,
                    Some(&measured_limit(expected_a / 2.0))
                )
                .unwrap(),
            HazardCheck::WithinLimit { .. }
        ));
        assert!(load(&data.replace("wire = \"return\"", "wire = \"absent\"")).is_err());
        assert!(
            load(&data.replace(
                "kind = \"main_only\"",
                "kind = \"main_only\", unexpected = true"
            ))
            .is_err()
        );
        assert!(load(&data.replace("kind = \"main_only\"", "kind = \"main_only\"").replace("source = { derived = \"фиксированные параметры стенда\" }", "source = { derived = \"испытание\" }\nsampling = { seed = 1, count = 1, connection_ohm = {} }")).is_err());
    }

    #[test]
    fn electrical_modes_two_inputs_retain_reverse_current() {
        let path = Path::new("блок.toml");
        let mut parts = parse_catalog(path, PARTS).unwrap().parts().clone();
        let mut usb = parts["reference_cell"].clone();
        usb.kind = PartKind::UsbSource;
        usb.source = "стенд: эквивалент USB 4 В с последовательным сопротивлением 1 Ом".into();
        let p = usb
            .elements
            .get_mut("emf")
            .unwrap()
            .parameters
            .get_mut("volt")
            .unwrap();
        p.range = Some(SourcedRange {
            min: 4.0,
            nominal: 4.0,
            max: 4.0,
            provenance: Provenance::Derived(usb.source.clone()),
        });
        parts.insert("fixture_usb".into(), usb);
        let bare = &mut parts
            .get_mut("reference_load")
            .unwrap()
            .pins
            .get_mut("input")
            .unwrap()
            .contact;
        bare.kind = ContactKind::Bare;
        bare.family = None;
        let catalog = Catalog::new(parts).unwrap();
        let mut data = String::from(
            r#"
format = 1
scope = "reference"
name = "Блок"
nets = ["plus", "return"]
[components.main]
part = "reference_cell"
nets = { plus = "plus", minus = "return" }
[components.usb]
part = "fixture_usb"
nets = { plus = "plus", minus = "return" }
[components.load]
part = "reference_load"
nets = { input = "plus", return = "return" }
[modes.source_disconnects]
main = "main_feed"
usb = "usb_feed"
"#,
        );
        // У каждого источника 1 Ом внутри и два стыка по 1 Ом до нагрузки 100 Ом.
        for (name, source, terminal, load) in [
            ("main_feed", "main", "plus", "input"),
            ("usb_feed", "usb", "plus", "input"),
            ("main_return", "main", "minus", "return"),
            ("usb_return", "usb", "minus", "return"),
        ] {
            data.push_str(&format!(r#"
[connections.{name}]
kind = "solder"
ends = ["{source}/{terminal}", "load/{load}"]
resistance_ohm = {{ measurement = "сопротивление стыка", range = {{ min = 1.0, nominal = 1.0, max = 1.0, source = {{ derived = "стенд: 1 Ом" }} }} }}
current_limit_a = {{ measurement = "предел стыка" }}
"#));
        }
        for kind in ["usb_only", "main_only", "both"] {
            data.push_str(&format!(r#"
[[modes.cases]]
mode = {{ kind = "{kind}" }}
source = {{ derived = "оба входа подключены через доступные для разрыва физические связи" }}
probes = [{{ kind = "voltage", ends = ["load/input", "load/return"] }}, {{ kind = "current", branch = {{ kind = "connection", id = "usb_feed" }} }}]
"#));
        }
        let input = SchematicInput::parse(path, data.into_bytes(), &catalog).unwrap();
        let voltage = Probe::Voltage(end("load", "input"), end("load", "return"));
        let usb_feed = Probe::Current(Section::Connection("usb_feed".into()));
        let v_both = (5.0 / 3.0 + 4.0 / 3.0) / (2.0 / 3.0 + 1.0 / 100.0);
        for (mode, v, amps) in [
            (Mode::UsbOnly, 400.0 / 103.0, 4.0 / 103.0),
            (Mode::MainOnly, 500.0 / 103.0, 0.0),
            (Mode::Both, v_both, (4.0 - v_both) / 3.0),
        ] {
            let prepared = value(input.compiled.prepare_mode(&input.modes, &mode).unwrap());
            let trace = value(prepared.solve());
            near(trace.values[&voltage][0], v);
            near(trace.values[&usb_feed][0], amps);
            assert_eq!(amps < 0.0, mode == Mode::Both);
        }
    }

    #[test]
    fn electrical_modes_pwl_front_and_signed_current_against_ngspice() {
        // Стенд: 1→5 В в момент 10 мкс через тот же делитель, затем плато до 30 мкс.
        let data = r#"
[[modes.cases]]
mode = { kind = "normal" }
source = { derived = "синтетическое окно линейного делителя" }
window = { step_ns = 1000, end_ns = 30000 }
probes = [{ kind = "voltage", ends = ["load/input", "load/return"] }, { kind = "current", branch = { kind = "connection", id = "source_end" } }]
sources = [{ branch = { kind = "element", component = "supply", element = "emf" }, points = [[0, 1.0], [10000, 5.0]], source = { derived = "линейный фронт стенда" } }]
"#;
        let input = load(data).unwrap();
        let planned = value(
            input
                .compiled
                .prepare_mode(&input.modes, &Mode::Normal)
                .unwrap(),
        );
        let voltage = Probe::Voltage(end("load", "input"), end("load", "return"));
        let current = Probe::Current(Section::Connection("source_end".into()));
        let scale = 1.0 / (1.0 + 100.0 + 0.2 + 3.0 * 0.1);
        let trace = value(planned.solve());
        let index = trace.time.iter().position(|&t| t == Time(10_000)).unwrap();
        near(trace.values[&current][index], 5.0 * scale);
        near(trace.values[&voltage][index / 2], 3.0 * scale * 100.0);
        let check = |factor| {
            ReferenceCheck::Transient(ComparisonWindow {
                start: Time::ZERO,
                end: Time(30_000),
                target: 5.0 * scale * factor,
                band: scale * factor,
            })
        };
        let reference = planned
            .compare_ngspice(&BTreeMap::from([
                (voltage.clone(), check(100.0)),
                (current.clone(), check(1.0)),
            ]))
            .unwrap();
        for (probe, result) in reference {
            assert!(result.matches(), "{probe:?}: {result:?}");
            // Последний вход в полосу (4 В, отнесённые к выходу) происходит через 7,5 мкс.
            near(result.settling_s.unwrap().reference, 7.5e-6);
            assert!(result.rows > 2);
        }
        assert!(
            planned
                .compare_ngspice(&BTreeMap::from([(
                    voltage,
                    ReferenceCheck::Transient(ComparisonWindow {
                        start: Time::ZERO,
                        end: Time(30_000),
                        target: 100.0,
                        band: 0.1,
                    })
                )]))
                .is_err()
        );
        assert!(load(&data.replace("10000, 5.0", "10000, nan")).is_err());
    }

    #[test]
    fn electrical_all_off_retains_capacitor_charge() {
        let path = Path::new("блок.toml");
        let mut parts = parse_catalog(path, PARTS).unwrap().parts().clone();
        let input = &mut parts
            .get_mut("reference_load")
            .unwrap()
            .pins
            .get_mut("input")
            .unwrap()
            .contact;
        input.kind = ContactKind::Bare;
        input.family = None;
        let catalog = Catalog::new(parts).unwrap();
        let scenario = r#"
format = 1
scope = "reference"
name = "Блок"
nets = ["plus", "return"]
[[modes.cases]]
mode = { kind = "all_off" }
source = { derived = "образец: конденсатор заряжен к началу отключения" }
window = { step_ns = 20000, end_ns = 40800000, reactive = [{ branch = { kind = "element", component = "cap", element = "cap" }, value = 5.0 }] }
probes = [{ kind = "voltage", ends = ["cap/plus", "cap/minus"] }]
[components.cap]
part = "reference_polar_capacitor"
nets = { plus = "plus", minus = "return" }
[components.load]
part = "reference_load"
nets = { input = "plus", return = "return" }
[connections.p]
kind = "solder"
ends = ["cap/plus", "load/input"]
resistance_ohm = { measurement = "параметр стенда", range = { min = 1.0, nominal = 1.0, max = 1.0, source = { derived = "1 Ом на стык" } } }
current_limit_a = { measurement = "измерить допустимый ток" }
[connections.n]
kind = "solder"
ends = ["cap/minus", "load/return"]
resistance_ohm = { measurement = "параметр стенда", range = { min = 1.0, nominal = 1.0, max = 1.0, source = { derived = "1 Ом на стык" } } }
current_limit_a = { measurement = "измерить допустимый ток" }
"#;
        let input = SchematicInput::parse(path, scenario.as_bytes().to_vec(), &catalog).unwrap();
        let prepared = value(
            input
                .compiled
                .prepare_mode(&input.modes, &Mode::AllOff)
                .unwrap(),
        );
        let trace = value(prepared.solve());
        let voltage = Probe::Voltage(end("cap", "plus"), end("cap", "minus"));
        // sim-circuit::init_uic согласует узлы неявным Эйлером на h₀ = 10⁻⁶ от шага.
        let h0 = Time(20_000).as_secs() * 1e-6;
        near(
            trace.values[&voltage][0],
            5.0 / (1.0 + h0 / Time(10_200_000).as_secs()),
        );
        // R = 100 + 2×1 Ом, C = 100 мкФ. Через RC = 10,2 мс осталось 5/e В.
        let at = trace
            .time
            .iter()
            .position(|&t| t == Time(10_200_000))
            .unwrap();
        let expected = 5.0 / std::f64::consts::E;
        // У сетки h/τ < 0,002, численная ошибка второго порядка намного меньше 1 мВ.
        assert!((trace.values[&voltage][at] - expected).abs() < 0.001);
        let report = prepared
            .compare_ngspice(&BTreeMap::from([(
                voltage.clone(),
                ReferenceCheck::Transient(ComparisonWindow {
                    start: Time(20_000),
                    end: Time(40_800_000),
                    target: 0.0,
                    band: expected,
                }),
            )]))
            .unwrap();
        assert!(report[&voltage].matches(), "{report:?}");
        assert!(
            (report[&voltage].settling_s.unwrap().reference - Time(10_180_000).as_secs()).abs()
                < Time(20_000).as_secs()
        );
        let gap = scenario.replace("reactive = [{ branch = { kind = \"element\", component = \"cap\", element = \"cap\" }, value = 5.0 }]", "reactive = []");
        let invalid = SchematicInput::parse(path, gap.into_bytes(), &catalog).unwrap();
        assert!(matches!(
            invalid
                .compiled
                .prepare_mode(&invalid.modes, &Mode::AllOff)
                .unwrap(),
            Analysis::NotAnalyzed { .. }
        ));
    }

    #[test]
    fn electrical_ranges_close_series_dc_and_keep_samples_separate() {
        let path = Path::new("блок.toml");
        let catalog = parse_catalog(path, PARTS).unwrap();
        // Каждый из трёх стыков имеет независимое сопротивление от 0,05 до 0,15 Ом.
        let ranged = FIXTURE.replace(
            "min = 0.1, nominal = 0.1, max = 0.1, source = { derived =",
            "min = 0.05, nominal = 0.1, max = 0.15, source = { measured =",
        );
        let modes = r#"
[[modes.cases]]
mode = { kind = "main_only" }
source = { derived = "огибающая резистивного контура" }
probes = [{ kind = "voltage", ends = ["load/input", "load/return"] }, { kind = "current", branch = { kind = "connection", id = "source_end" } }]

[[modes.cases]]
mode = { kind = "normal" }
source = { derived = "выборка стенда" }
probes = [{ kind = "current", branch = { kind = "connection", id = "source_end" } }]
sampling = { seed = 7, count = 4, connection_ohm = { return = { min = 0.05, nominal = 0.1, max = 0.15, source = { measured = "равномерная популяция стыка в постановке" } } } }
"#;
        let input =
            SchematicInput::parse(path, format!("{ranged}\n{modes}").into_bytes(), &catalog)
                .unwrap();
        let run = || input.compiled.run_modes(&input.modes).unwrap();
        let first = run();
        assert_eq!(first, run());
        let report = |wanted: Mode| {
            first
                .iter()
                .find_map(|entry| match entry {
                    ModeCoverage::Case(case) if case.mode == wanted => Some(case),
                    _ => None,
                })
                .unwrap()
        };
        let envelope = value(report(Mode::MainOnly).envelope.clone());
        let current = Probe::Current(Section::Connection("source_end".into()));
        let amps = &envelope[&current];
        let v = &envelope[&Probe::Voltage(end("load", "input"), end("load", "return"))];
        near(amps.min.value, 5.0 / (100.0 + 1.0 + 0.2 + 3.0 * 0.15));
        near(amps.max.value, 5.0 / (100.0 + 1.0 + 0.2 + 3.0 * 0.05));
        near(v.min.value, amps.min.value * 100.0);
        near(v.max.value, amps.max.value * 100.0);
        assert!(amps.min.value <= 5.0 / (100.0 + 1.0 + 0.2 + 3.0 * 0.15));
        assert!(amps.max.value >= 5.0 / (100.0 + 1.0 + 0.2 + 3.0 * 0.05));
        let samples = report(Mode::Normal);
        assert_eq!(samples.seed, Some(7));
        assert_eq!(samples.samples.len(), 4);
        assert!(matches!(samples.envelope, Analysis::NotAnalyzed { .. }));
        for sample in &samples.samples {
            let r = sample.connection_ohm["return"];
            assert!((0.05..=0.15).contains(&r));
            near(
                value(sample.trace.clone()).values[&current][0],
                5.0 / (100.0 + 1.0 + 0.2 + 2.0 * 0.1 + r),
            );
        }
    }

    #[test]
    fn electrical_pin_five_states_and_boot_use_same_physical_contact() {
        let path = Path::new("блок.toml");
        let mut parts = parse_catalog(path, PARTS).unwrap().parts().clone();
        let mut io = parts["reference_receiver"].clone();
        io.pins.get_mut("input").unwrap().role = PinRole::SignalIo("return".into());
        let mut power = parts["reference_cell"].pins["plus"].clone();
        power.contact.kind = ContactKind::Bare;
        io.pins.insert("supply".into(), power);
        parts.insert("fixture_io".into(), io);
        let schematic = r#"
format = 1
scope = "reference"
name = "Блок"
nets = ["feed", "return", "signal"]
[[bindings]]
target = { kind = "board_pin", board = "board", pin = "0", function = "gpio_output" }
ends = ["io/input"]
[[bindings]]
target = { kind = "board_power", board = "board", rail = "local" }
ends = ["io/supply", "io/return"]
[components.source]
part = "reference_cell"
nets = { plus = "feed", minus = "return" }
[components.io]
part = "fixture_io"
nets = { input = "signal", return = "return", supply = "feed" }
[connections.feed]
kind = "solder"
ends = ["source/plus", "io/supply"]
resistance_ohm = { measurement = "заданный стык", range = { min = 1.0, nominal = 1.0, max = 1.0, source = { derived = "стенд: 1 Ом" } } }
current_limit_a = { measurement = "предел стенда" }
[connections.return]
kind = "solder"
ends = ["io/return", "source/minus"]
resistance_ohm = { measurement = "заданный стык", range = { min = 1.0, nominal = 1.0, max = 1.0, source = { derived = "стенд: 1 Ом" } } }
current_limit_a = { measurement = "предел стенда" }
"#;
        let input = SchematicInput::parse(
            path,
            schematic.as_bytes().to_vec(),
            &Catalog::new(parts).unwrap(),
        )
        .unwrap();
        let point = |v| Parameter {
            range: Some(SourcedRange {
                min: v,
                nominal: v,
                max: v,
                provenance: Provenance::Derived("точные исходные данные стенда".into()),
            }),
            measurement: "значение стенда".into(),
        };
        let pin = |state| SourcedPinState {
            state,
            source: Provenance::Derived("порядок состояний стенда".into()),
        };
        let model = PinModel {
            high_ohm: point(10.0),
            low_ohm: point(20.0),
            pull_up_ohm: point(100.0),
            pull_down_ohm: point(200.0),
            input_ohm: point(1000.0),
            high_z_ohm: point(10_000.0),
            upper_clamp: ClampPath::Absent(Provenance::Derived(
                "синтетическая развязанная модель".into(),
            )),
            lower_clamp: ClampPath::Absent(Provenance::Derived(
                "синтетическая развязанная модель".into(),
            )),
        };
        let profile = PinProfile {
            model,
            unpowered: Some(pin(PinState::HighZ)),
            reset: Some(pin(PinState::Input)),
            boot: vec![
                BootStage {
                    duration_s: point(Time(10).as_secs()),
                    pin: pin(PinState::InputPull(Pull::Up)),
                },
                BootStage {
                    duration_s: point(Time(20).as_secs()),
                    pin: pin(PinState::Low),
                },
            ],
        };
        let probe = Probe::Voltage(end("io", "input"), end("io", "return"));
        let at = BoardContact {
            board: "board".into(),
            pin: "0".into(),
        };
        for (mode, expected, volts, added) in [
            // Источник 5 В, 1 Ом и два стыка по 1 Ом. На выводе нагрузка 1 кОм.
            (Mode::MainOnly, PinState::High, 5.0 * 1000.0 / 1013.0, 1),
            (Mode::Reset, PinState::Input, 0.0, 1),
            // При включении буфера 1 кОм || 1 кОм дают 500 Ом. Подтяжка 100 Ом.
            (
                Mode::Boot(Time::ZERO),
                PinState::InputPull(Pull::Up),
                5.0 * 500.0 / 603.0,
                2,
            ),
            (Mode::Boot(Time(10)), PinState::Low, 0.0, 1),
            (Mode::AllOff, PinState::HighZ, 0.0, 1),
        ] {
            let setup = PinSetup {
                rail: "local".into(),
                profile: profile.clone(),
                phase: PinPhase::Working(pin(PinState::High)),
            };
            let plan = ModePlan {
                source_disconnects: BTreeMap::from([("source".into(), "feed".into())]),
                cases: BTreeMap::from([(
                    mode.clone(),
                    ModeCase {
                        source: Provenance::Derived("постановка вывода".into()),
                        window: None,
                        sources: BTreeMap::new(),
                        pins: BTreeMap::from([(at.clone(), setup)]),
                        probes: BTreeSet::from([probe.clone()]),
                        sampling: None,
                    },
                )]),
            };
            let prepared = value(input.compiled.prepare_mode(&plan, &mode).unwrap());
            assert_eq!(prepared.pins[&at].state, expected);
            assert_eq!(
                prepared
                    .netlist()
                    .origins
                    .keys()
                    .filter(|name| name.starts_with("rpin"))
                    .count(),
                added
            );
            assert_eq!(
                expected.input_enabled(),
                matches!(expected, PinState::Input | PinState::InputPull(_))
            );
            near(value(prepared.solve()).values[&probe][0], volts);
        }
        assert!(matches!(
            profile
                .state(&PinPhase::Boot { elapsed: Time(30) }, Scope::Reference)
                .unwrap(),
            Analysis::NotAnalyzed { .. }
        ));
        let broken = PinProfile {
            boot: vec![BootStage {
                duration_s: point(f64::NAN),
                pin: pin(PinState::High),
            }],
            ..profile
        };
        assert!(broken.validate().is_err());
    }

    #[test]
    fn electrical_board_catalog_distinguishes_por_from_gpio_and_rejects_bad_input() {
        let text = include_str!("../../../../catalog/boards/electrical/stm32f103c8.toml");
        let path = Path::new("плата.toml");
        let profile = CatalogInput::parse_board_profile(path, text.as_bytes()).unwrap();
        let folder = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../catalog/boards/electrical");
        let all = CatalogInput::load_board_profiles(&folder).unwrap();
        assert_eq!(
            all["esp32-c3"].0.reset_pins[&9].state,
            PinState::InputPull(Pull::Up)
        );
        assert!(all["esp32-c3"].0.reset.falling_v.range.is_none());
        assert_eq!(profile.reset_pins[&0].state, PinState::Input);
        assert!(!profile.reset_pins.contains_key(&13));
        assert_eq!(profile.reset.falling_v.range.unwrap().nominal, 1.88);
        assert_eq!(profile.reset.rising_v.range.unwrap().nominal, 1.92);
        assert!(profile.reset.assert_delay_s.range.is_none());
        assert!(
            CatalogInput::parse_board_profile(path, text.replace("1.88", "inf").as_bytes())
                .is_err()
        );
        assert!(
            CatalogInput::parse_board_profile(path, text.replace("0, 1,", "0, 0,").as_bytes())
                .is_err()
        );
    }
}
