//! Версионированные задания, разрешённые входы и полная матрица прогона.

use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    path::{Path, PathBuf},
};

use serde::{Deserialize, Serialize};
use sim_signal::Rate;
use sim_world::{CompiledWorld, ContactBuildOptions, DeviceInput, Pose, World};

use crate::{
    Error, Fault, FaultKind, Fingerprint, Limit, Signal, Source, Topology, invalid,
    metrics::MetricSpec,
    sampling::{
        AppliedValue, Axis, ModelSettings, Parameter, Sample, SampleKey, Sampling, Scope, field,
        name,
    },
};

pub mod depth_factors;
pub mod depth_cover;
pub mod depth_search;

use depth_factors::{DepthFactors, DepthPlan};

/// Файлы старых форматов разрешает их владелец. Вся цепочка фиксируется до планирования.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReferenceKind {
    Catalog,
    ControlTask,
    ControlSpec,
    ProcessTask,
    InteractionTask,
    Profile,
}

#[derive(Clone, Debug)]
pub struct Reference {
    pub path: PathBuf,
    pub kind: ReferenceKind,
    pub file_fingerprint: Option<Fingerprint>,
}

#[derive(Clone, Debug)]
pub struct ResolvedReference {
    pub kind: ReferenceKind,
    pub file_fingerprint: Option<Fingerprint>,
    pub graph_fingerprint: Fingerprint,
}

#[derive(Clone, Debug)]
pub struct WorldInputs {
    pub path: PathBuf,
    pub world: World,
    pub devices: BTreeMap<String, DeviceInput>,
    pub contact: ContactBuildOptions,
}

impl WorldInputs {
    /// Загрузка исходного мира обычным разрешителем форматов.
    pub fn load(path: impl AsRef<Path>) -> Result<Self, Error> {
        let path = path
            .as_ref()
            .canonicalize()
            .map_err(|e| invalid(format!("{}: {e}", path.as_ref().display())))?;
        let world = sim_world::parse_world(&path, &read(&path)?).map_err(invalid)?;
        let mut devices = BTreeMap::new();
        for (i, placement) in world.devices.iter().enumerate() {
            let file = locate(&path, &format!("devices[{i}].file"), &placement.file)?;
            let device =
                sim_world::load_device(&file, &sim_world::LegacyCatalog).map_err(invalid)?;
            if devices
                .insert(placement.name.clone(), DeviceInput { path: file, device })
                .is_some()
            {
                return Err(invalid(format!(
                    "{}: devices[{i}].name: повтор размещения",
                    path.display()
                )));
            }
        }
        Ok(Self {
            path,
            world,
            devices,
            contact: ContactBuildOptions::default(),
        })
    }

    pub fn compile(&self) -> Result<CompiledWorld, Error> {
        sim_world::compile_world_with_contact_options(
            &self.path,
            &self.world,
            &self.devices,
            &self.contact,
        )
        .map_err(invalid)
    }

    fn open_powered_mounts(&self) -> impl Iterator<Item = String> + '_ {
        // Связь, которую можно снять в прогоне, не подтверждает изоляцию.
        let bound: BTreeSet<_> = self.world.attachments.iter()
            .chain(&self.contact.added_attachments)
            .filter(|attachment| attachment.mode == sim_world::AttachmentMode::Design)
            .flat_map(|attachment| [&attachment.first, &attachment.second])
            .collect();
        self.devices.iter()
            .filter(|(_, input)| input.device.links.iter().any(|link| !link.joint.efforts.is_empty()))
            .flat_map(|(placement, input)| input.device.links.iter().flat_map(move |link| {
                link.mounts.iter()
                    .filter(|mount| mount.kind == sim_world::MountKind::Mechanical)
                    .map(move |mount| format!("{placement}/{}/{}", link.name, mount.name))
            }))
            .filter(move |mount| !bound.contains(mount))
    }
}

#[derive(Clone, Debug)]
pub struct ModelParameter {
    pub unit: String,
    pub nominal: f64,
    /// Пределы установленной модели, в единице параметра.
    pub min: f64,
    pub max: f64,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Target {
    Effort { name: String },
    Port { name: String },
    Operation { name: String },
}

#[derive(Clone, Debug)]
pub struct Binding {
    pub unit: String,
    /// Один канонический ресурс для адресов, которые физически перезаписывают друг друга.
    pub resource: String,
    pub unavailable: Option<String>,
}

#[derive(Clone, Debug)]
pub enum FaultSupport {
    Available { channel: String, resource: String },
    Unsupported { reason: String },
    Inapplicable { reason: String, basis: Source },
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Grid {
    pub id: String,
    pub hz: u64,
    pub source: Source,
}

#[derive(Clone, Debug)]
pub struct RequiredPath {
    pub station: String,
    pub mode: String,
    pub path: String,
    pub source: Source,
    pub grids: Vec<Grid>,
    pub faults: BTreeMap<Fault, FaultSupport>,
    pub combinations: Vec<FaultCombination>,
    /// Для отсутствующего механизма, неподдерживаемого режима или связки.
    pub unavailable: Option<String>,
}

impl RequiredPath {
    /// Штатные строки и одиночные отказы из уже разрешённой топологии.
    /// О наличии конкретной инъекции сообщает адаптер установленной модели.
    pub fn from_topology(
        station: &str,
        topology: &Topology,
        grids: &[Grid],
        source: &Source,
        mut support: impl FnMut(&str, &Fault) -> FaultSupport,
    ) -> Vec<Self> {
        let mut paths = BTreeMap::new();
        for case in topology.cases() {
            let entry = paths
                .entry((case.mode.clone(), case.path.clone()))
                .or_insert_with(|| Self {
                    station: station.into(),
                    mode: case.mode,
                    path: case.path,
                    source: source.clone(),
                    grids: grids.to_vec(),
                    faults: BTreeMap::new(),
                    combinations: Vec::new(),
                    unavailable: None,
                });
            if let Some(fault) = case.fault {
                let ability = support(&entry.path, &fault);
                entry.faults.insert(fault, ability);
            }
        }
        paths.into_values().collect()
    }
}

/// Разрешитель инвентаризирует состав и эксплуатацию независимо от фильтра запуска.
#[derive(Clone, Debug)]
pub struct ResolvedInputs {
    pub world: WorldInputs,
    pub references: BTreeMap<PathBuf, ResolvedReference>,
    pub paths: Vec<RequiredPath>,
    pub targets: BTreeMap<Target, Binding>,
    pub signals: BTreeSet<Signal>,
    pub model_parameters: BTreeMap<Parameter, ModelParameter>,
    /// Поддерживаемые установщиком начальные скорости до первого такта.
    pub initial_velocity: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Window {
    pub start_ns: u64,
    pub end_ns: u64,
    pub max_gap_ns: u64,
}

impl Window {
    pub(crate) fn validate(self) -> Result<(), Error> {
        if self.start_ns >= self.end_ns || self.max_gap_ns == 0 {
            Err(invalid(
                "окно должно иметь положительную длину и положительный предел шага",
            ))
        } else {
            Ok(())
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Step {
    pub id: String,
    pub target: Target,
    pub at_ns: u64,
    /// Допустимое опоздание при фактическом подтверждении действия.
    pub max_lag_ns: Option<u64>,
    pub value: f64,
    pub unit: String,
    pub source: Source,
}

impl Step {
    pub(crate) fn latest_ns(&self) -> Option<u64> {
        self.at_ns.checked_add(self.max_lag_ns.unwrap_or(0))
    }

    pub(crate) fn accepts_time(&self, at: u64) -> bool {
        self.latest_ns()
            .is_some_and(|latest| (self.at_ns..=latest).contains(&at))
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ScheduledFault {
    pub fault: Fault,
    pub at_ns: u64,
    pub max_lag_ns: u64,
    pub extra_delay_ns: Option<u64>,
    pub window: String,
    pub source: Source,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct FaultCombination {
    pub path: String,
    pub faults: Vec<Fault>,
    pub source: Source,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct InitialValue {
    pub parameter: Parameter,
    pub unit: String,
    pub value: f64,
    pub source: Source,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum PoseTarget {
    Placement { placement: String },
    Link { placement: String, link: String },
    Object { object: String },
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct InitialPose {
    pub target: PoseTarget,
    pub pose: Pose,
    pub source: Source,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Criterion {
    pub id: String,
    pub window: String,
    pub limit: Limit,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct DeclaredRequirement {
    pub class: String,
    pub object: String,
    pub window: String,
    pub source: Source,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct OperatingMode {
    pub horizon_ns: u64,
    pub source: Source,
    #[serde(default)]
    pub windows: BTreeMap<String, Window>,
    #[serde(default)]
    pub initial: Vec<InitialValue>,
    #[serde(default)]
    pub poses: Vec<InitialPose>,
    #[serde(default)]
    pub steps: Vec<Step>,
    #[serde(default)]
    pub criteria: Vec<Criterion>,
    #[serde(default)]
    pub hazards: Vec<DeclaredRequirement>,
    #[serde(default)]
    pub metrics: Vec<MetricSpec>,
    #[serde(default)]
    pub faults: Vec<ScheduledFault>,
    #[serde(default)]
    pub combinations: Vec<FaultCombination>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum ExperimentSpec {
    Scenario,
    Control { task: PathBuf },
    Process { task: PathBuf },
    Interaction { task: PathBuf },
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct StationSpec {
    experiment: ExperimentSpec,
    profile: Option<PathBuf>,
    modes: BTreeMap<String, OperatingMode>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Document {
    version: u32,
    seed: Option<u64>,
    world: Option<PathBuf>,
    catalog: Option<PathBuf>,
    sampling: Option<Sampling>,
    depth_factors: Option<DepthFactors>,
    #[serde(default)]
    include: Vec<PathBuf>,
    #[serde(default)]
    stations: BTreeMap<String, StationSpec>,
}

#[derive(Clone, Debug, Serialize)]
pub struct Experiment {
    pub kind: Option<ReferenceKind>,
    pub task: Option<PathBuf>,
    pub profile: Option<PathBuf>,
}

#[derive(Clone, Debug)]
struct Station {
    file: PathBuf,
    experiment: Experiment,
    modes: BTreeMap<String, OperatingMode>,
}

#[derive(Clone, Debug)]
pub struct Campaign {
    file: PathBuf,
    world_path: PathBuf,
    world: World,
    catalog: PathBuf,
    seed: u64,
    stations: BTreeMap<String, Station>,
    sampling: Sampling,
    sampling_declared: bool,
    depth_factors: Option<DepthFactors>,
    references: BTreeMap<PathBuf, Reference>,
    fingerprint: Fingerprint,
}

fn read(path: &Path) -> Result<String, Error> {
    fs::read_to_string(path).map_err(|e| invalid(format!("{}: {e}", path.display())))
}

fn locate(parent: &Path, label: &str, referenced: &Path) -> Result<PathBuf, Error> {
    if referenced.as_os_str().is_empty() {
        return Err(invalid(format!(
            "{}: {label}: пустой путь",
            parent.display()
        )));
    }
    let absolute = parent.parent().unwrap_or(Path::new("")).join(referenced);
    absolute.canonicalize().map_err(|error| {
        invalid(format!(
            "{}: {label} ({}): {error}",
            parent.display(),
            absolute.display()
        ))
    })
}

fn finite_toml(value: &toml::Value, field: &str) -> Result<(), Error> {
    match value {
        toml::Value::Float(x) if !x.is_finite() => {
            Err(invalid(format!("{field}: требуется конечное число")))
        }
        toml::Value::Table(t) => t
            .iter()
            .try_for_each(|(key, value)| finite_toml(value, &format!("{field}.{key}"))),
        toml::Value::Array(a) => a
            .iter()
            .enumerate()
            .try_for_each(|(i, value)| finite_toml(value, &format!("{field}[{i}]"))),
        _ => Ok(()),
    }
}

fn checked_toml(path: &Path, text: &str) -> Result<(), Error> {
    let value: toml::Value =
        toml::from_str(text).map_err(|e| invalid(format!("{}: {e}", path.display())))?;
    finite_toml(&value, &path.display().to_string())
}

fn read_document(path: &Path) -> Result<(Document, String), Error> {
    let text = read(path)?;
    checked_toml(path, &text)?;
    let doc: Document =
        toml::from_str(&text).map_err(|e| invalid(format!("{}: {e}", path.display())))?;
    // Версия 1 определяет структуру ключей, интервалы в наносекундах и выборки.
    if doc.version != 1 {
        return Err(invalid(format!(
            "{}: version: неподдерживаемая версия",
            path.display()
        )));
    }
    Ok((doc, text))
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ProcessTaskFile {
    version: u32,
    process_profile: PathBuf,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct InteractionTaskFile {
    version: u32,
    interaction_profile: PathBuf,
}

fn register_reference(
    references: &mut BTreeMap<PathBuf, Reference>,
    path: &Path,
    kind: ReferenceKind,
) -> Result<(), Error> {
    if let Some(previous) = references.get(path) {
        if previous.kind != kind {
            return Err(invalid(format!(
                "{}: файл объявлен с несовместимыми типами",
                path.display()
            )));
        }
        return Ok(());
    }
    let mut children = Vec::new();
    let file_fingerprint = if path.is_dir() && kind == ReferenceKind::Catalog {
        None
    } else {
        let text = read(path)?;
        checked_toml(path, &text)?;
        let parse_error = |e: toml::de::Error| invalid(format!("{}: {e}", path.display()));
        let profile = match kind {
            ReferenceKind::ControlTask => {
                let task = crate::Task::parse(&text)
                    .map_err(|e| invalid(format!("{}: {e}", path.display())))?;
                children.push((
                    ReferenceKind::ControlSpec,
                    locate(path, "control", Path::new(task.control()))?,
                ));
                task.profile()
                    .map(|profile| (1, "profile", PathBuf::from(profile)))
            }
            ReferenceKind::ControlSpec => {
                sim_runtime::control::ControlConfig::parse(&text)
                    .map_err(|e| invalid(format!("{}: {e}", path.display())))?;
                None
            }
            ReferenceKind::ProcessTask => {
                let task: ProcessTaskFile = toml::from_str(&text).map_err(parse_error)?;
                Some((task.version, "process_profile", task.process_profile))
            }
            ReferenceKind::InteractionTask => {
                let task: InteractionTaskFile = toml::from_str(&text).map_err(parse_error)?;
                Some((
                    task.version,
                    "interaction_profile",
                    task.interaction_profile,
                ))
            }
            ReferenceKind::Profile | ReferenceKind::Catalog => None,
        };
        if let Some((version, field, profile)) = profile {
            if version != 1 {
                return Err(invalid(format!(
                    "{}: version: неподдерживаемая версия задания",
                    path.display()
                )));
            }
            children.push((ReferenceKind::Profile, locate(path, field, &profile)?));
        }
        Some(Fingerprint::of(text.as_bytes()))
    };
    references.insert(
        path.to_owned(),
        Reference {
            path: path.to_owned(),
            kind,
            file_fingerprint,
        },
    );
    for (kind, child) in children {
        register_reference(references, &child, kind)?;
    }
    Ok(())
}

#[derive(Default)]
struct Documents {
    active: BTreeSet<PathBuf>,
    all: BTreeMap<PathBuf, Fingerprint>,
    stations: BTreeMap<String, Station>,
    references: BTreeMap<PathBuf, Reference>,
}

impl Documents {
    fn resolve(&mut self, file: &Path, doc: Document, text: &str) -> Result<(), Error> {
        self.active.insert(file.to_owned());
        self.all
            .insert(file.to_owned(), Fingerprint::of(text.as_bytes()));
        for (i, include) in doc.include.iter().enumerate() {
            let path = locate(file, &format!("include[{i}]"), include)?;
            if self.active.contains(&path) {
                return Err(invalid(format!(
                    "{}: include[{i}]: цикл через {}",
                    file.display(),
                    path.display()
                )));
            }
            if self.all.contains_key(&path) {
                continue;
            }
            let (included, contents) = read_document(&path)?;
            if included.seed.is_some()
                || included.world.is_some()
                || included.catalog.is_some()
                || included.sampling.is_some()
                || included.depth_factors.is_some()
            {
                return Err(invalid(format!(
                    "{}: seed, world, catalog, sampling и depth_factors задаются в корне кампании",
                    path.display()
                )));
            }
            self.resolve(&path, included, &contents)?;
        }
        for (id, spec) in doc.stations {
            field(&format!("{}: stations.{id}", file.display()), name(&id))?;
            let context = format!("stations.{id}.experiment.task");
            let (kind, task) = match spec.experiment {
                ExperimentSpec::Scenario => (None, None),
                ExperimentSpec::Control { task } => (
                    Some(ReferenceKind::ControlTask),
                    Some(locate(file, &context, &task)?),
                ),
                ExperimentSpec::Process { task } => (
                    Some(ReferenceKind::ProcessTask),
                    Some(locate(file, &context, &task)?),
                ),
                ExperimentSpec::Interaction { task } => (
                    Some(ReferenceKind::InteractionTask),
                    Some(locate(file, &context, &task)?),
                ),
            };
            if let (Some(kind), Some(task)) = (kind, &task) {
                register_reference(&mut self.references, task, kind)?;
            }
            let profile = spec
                .profile
                .as_ref()
                .map(|p| locate(file, &format!("stations.{id}.profile"), p))
                .transpose()?;
            if let Some(profile) = &profile {
                register_reference(&mut self.references, profile, ReferenceKind::Profile)?;
            }
            if spec.modes.is_empty() {
                return Err(invalid(format!(
                    "{}: stations.{id}.modes: пустой список",
                    file.display()
                )));
            }
            for (mode_id, mode) in &spec.modes {
                let context = format!("{}: stations.{id}.modes.{mode_id}", file.display());
                field(
                    &context,
                    name(mode_id).and_then(|()| mode.validate(kind.is_none())),
                )?;
            }
            if self
                .stations
                .insert(
                    id.clone(),
                    Station {
                        file: file.to_owned(),
                        experiment: Experiment {
                            kind,
                            task,
                            profile,
                        },
                        modes: spec.modes,
                    },
                )
                .is_some()
            {
                return Err(invalid(format!(
                    "{}: stations.{id}: повтор постановки",
                    file.display()
                )));
            }
        }
        self.active.remove(file);
        Ok(())
    }
}

impl Campaign {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, Error> {
        let path = path.as_ref();
        let file = path
            .canonicalize()
            .map_err(|e| invalid(format!("{}: {e}", path.display())))?;
        let (mut doc, text) = read_document(&file)?;
        let missing =
            |field: &str| invalid(format!("{}: {field}: обязательное поле", file.display()));
        let world_path = locate(
            &file,
            "world",
            doc.world.as_deref().ok_or_else(|| missing("world"))?,
        )?;
        let world = sim_world::parse_world(&world_path, &read(&world_path)?).map_err(invalid)?;
        let catalog = locate(
            &file,
            "catalog",
            doc.catalog.as_deref().ok_or_else(|| missing("catalog"))?,
        )?;
        let seed = doc.seed.ok_or_else(|| missing("seed"))?;
        let sampling_declared = doc.sampling.is_some();
        let depth_factors = doc.depth_factors.take();
        if depth_factors.is_some() && sampling_declared {
            return Err(invalid(format!(
                "{}: уровни depth_factors и прежний sampling задают разные планы разброса",
                file.display()
            )));
        }
        let mut sampling = doc.sampling.take().unwrap_or_default();
        field(&file.display().to_string(), sampling.validate())?;
        let mut docs = Documents::default();
        register_reference(&mut docs.references, &catalog, ReferenceKind::Catalog)?;
        docs.resolve(&file, doc, &text)?;
        if docs.stations.is_empty() {
            return Err(invalid(format!(
                "{}: stations: пустой список постановок",
                file.display()
            )));
        }
        let fingerprint = Fingerprint::of(format!("{:?}", docs.all).as_bytes());
        Ok(Self {
            file,
            world_path,
            world,
            catalog,
            seed,
            stations: docs.stations,
            sampling,
            sampling_declared,
            depth_factors,
            references: docs.references,
            fingerprint,
        })
    }

    pub fn world_path(&self) -> &Path {
        &self.world_path
    }
    pub fn catalog_path(&self) -> &Path {
        &self.catalog
    }
    pub fn references(&self) -> impl Iterator<Item = &Reference> {
        self.references.values()
    }

    /// Оси активного плана для разрешения параметров в установщике.
    pub fn sampling_axes(&self) -> &[Axis] {
        self.depth_factors
            .as_ref()
            .map_or(&self.sampling.axes, DepthFactors::axes)
    }

    pub fn sampling(&self) -> &Sampling {
        &self.sampling
    }

    pub fn has_native_scenarios(&self) -> bool {
        self.stations.values().any(|station| station.experiment.kind.is_none())
    }

    /// Горизонт встроенной постановки, для которой установщик может открыть профиль.
    pub fn native_horizon(&self, station: &str, mode: &str) -> Option<sim_signal::Time> {
        let station = self.stations.get(station)?;
        if station.experiment.kind.is_some() {
            return None;
        }
        station.modes.get(mode).map(|mode| sim_signal::Time(mode.horizon_ns))
    }

    pub fn depth_factors(&self) -> Option<&DepthFactors> {
        self.depth_factors.as_ref()
    }

    /// Разрешает входы покрытия без построения полной матрицы.
    pub fn prepare_depth(&self, inventory: &ResolvedInputs) -> Result<DepthPlan<'_>, Error> {
        let factors = self.depth_factors.as_ref().ok_or_else(|| {
            invalid(format!(
                "{}: отсутствует раздел depth_factors",
                self.file.display()
            ))
        })?;
        let nominal = self
            .validate_plan_inputs(inventory, factors.axes())
            .map_err(|e| invalid(format!("{}: {e}", self.file.display())))?;
        let (inputs, base) = self.plan_identity(inventory, &nominal);
        let fingerprint =
            Fingerprint::of(format!("campaign/depth/v1|{base:?}|{factors:?}").as_bytes());
        Ok(DepthPlan::new(self, factors, inputs, nominal, fingerprint))
    }

    pub fn plan(&self, inventory: &ResolvedInputs) -> Result<CampaignPlan, Error> {
        if self.depth_factors.is_some() {
            return Err(invalid(format!(
                "{}: для depth_factors требуется план покрытия",
                self.file.display()
            )));
        }
        self.build_plan(inventory)
            .map_err(|e| invalid(format!("{}: {e}", self.file.display())))
    }

    fn validate_plan_inputs(
        &self,
        inventory: &ResolvedInputs,
        axes: &[Axis],
    ) -> Result<CompiledWorld, Error> {
        if inventory.world.path != self.world_path || inventory.world.world != self.world {
            return Err(invalid("world: разрешитель передал другой начальный мир"));
        }
        for reference in self.references.values() {
            let resolved = inventory.references.get(&reference.path).ok_or_else(|| {
                invalid(format!(
                    "{}: нет проверки графа входов",
                    reference.path.display()
                ))
            })?;
            if (resolved.kind, resolved.file_fingerprint)
                != (reference.kind, reference.file_fingerprint)
            {
                return Err(invalid(format!(
                    "{}: изменился файл или тип ссылки",
                    reference.path.display()
                )));
            }
        }
        for (i, placement) in self.world.devices.iter().enumerate() {
            let field = format!("devices[{i}].file");
            let path = locate(&self.world_path, &field, &placement.file)?;
            if !inventory
                .world
                .devices
                .get(&placement.name)
                .is_some_and(|input| input.path == path)
            {
                return Err(invalid(format!(
                    "{}: {field}: разрешитель передал другую модель",
                    self.world_path.display()
                )));
            }
        }
        inventory.validate()?;
        let nominal = inventory.world.compile()?;
        inventory.validate_efforts(&nominal)?;
        for axis in axes {
            for value in [axis.min, axis.nominal, axis.max] {
                let context = format!("sampling.axes.{}", axis.id);
                field(
                    &context,
                    apply_axis(
                        axis,
                        value,
                        &axis.source,
                        inventory,
                        &nominal,
                        &mut inventory.world.clone(),
                        &mut ModelSettings::default(),
                    ),
                )?;
            }
        }
        for (station, spec) in &self.stations {
            for (mode, description) in &spec.modes {
                let context = format!("{}: stations.{station}.modes.{mode}", spec.file.display());
                let paths: Vec<_> = inventory
                    .paths
                    .iter()
                    .filter(|p| p.station == *station && p.mode == *mode)
                    .collect();
                field(
                    &context,
                    Self::validate_mode(description, &paths, inventory, &nominal, axes),
                )?;
            }
        }

        Ok(nominal)
    }

    fn build_plan(&self, inventory: &ResolvedInputs) -> Result<CampaignPlan, Error> {
        let nominal = self.validate_plan_inputs(inventory, &self.sampling.axes)?;
        let samples = self.sampling.keys()?;
        let mut velocity = BTreeSet::new();
        if !inventory.initial_velocity
            && self
                .sampling
                .axes
                .iter()
                .any(|a| matches!(a.parameter, Parameter::JointVelocity { .. }))
        {
            for key in &samples {
                if self
                    .sampling
                    .sample(self.seed, key, &["campaign/v1", "sample"])
                    .map(|sample| {
                        sample.values.iter().any(|value| {
                            matches!(value.parameter, Parameter::JointVelocity { .. })
                                && value.value != 0.0
                        })
                    })?
                {
                    velocity.insert(key);
                }
            }
        }
        let mut cases = Vec::new();
        let mut count = 0usize;
        for path in &inventory.paths {
            let mode = self
                .stations
                .get(&path.station)
                .and_then(|s| s.modes.get(&path.mode));
            let mut sets = vec![(Vec::new(), None)];
            sets.extend(path.faults.keys().cloned().map(|fault| (vec![fault], None)));
            let mut combined = BTreeMap::new();
            for c in path.combinations.iter().chain(
                mode.into_iter()
                    .flat_map(|m| &m.combinations)
                    .filter(|c| c.path == path.path),
            ) {
                let mut faults = c.faults.clone();
                faults.sort();
                combined.entry(faults).or_insert(&c.source);
            }
            sets.extend(
                combined
                    .into_iter()
                    .map(|(faults, source)| (faults, Some(source))),
            );
            count = sets
                .len()
                .checked_mul(path.grids.len())
                .and_then(|n| n.checked_mul(samples.len()))
                .and_then(|n| count.checked_add(n))
                .ok_or_else(|| invalid("matrix: переполнение числа строк"))?;
            cases.push((path, mode, sets));
        }
        let mut rows = Vec::new();
        rows.try_reserve_exact(count)
            .map_err(|_| invalid("matrix: число строк не помещается в память"))?;
        for (path, mode, sets) in cases {
            for (faults, combination_source) in sets {
                for grid in &path.grids {
                    for sample in &samples {
                        let key = RunKey {
                            station: path.station.clone(),
                            mode: path.mode.clone(),
                            path: path.path.clone(),
                            faults: faults.clone(),
                            sample: sample.clone(),
                            grid: grid.id.clone(),
                        };
                        rows.push(self.row(
                            key,
                            path,
                            mode,
                            grid,
                            combination_source,
                            inventory,
                            velocity.contains(sample),
                        ));
                    }
                }
            }
        }
        rows.sort_by(|a, b| a.key.cmp(&b.key));
        let (inputs, fingerprint) = self.plan_identity(inventory, &nominal);
        let mut plan = CampaignPlan {
            rows,
            campaign: self.clone(),
            inputs,
            nominal,
            fingerprint,
            participant_candidates: BTreeSet::new(),
        };
        plan.validate_materialized_worlds()?;
        Ok(plan)
    }

    fn plan_identity(
        &self,
        inventory: &ResolvedInputs,
        nominal: &CompiledWorld,
    ) -> (ResolvedInputs, Fingerprint) {
        // Отпечаток предназначен для идентификации входов в пределах одной версии формата и кода.
        // Порядок разрешённых путей и сеток не влияет на ключи или отпечаток.
        let mut inputs = inventory.clone();
        inputs
            .paths
            .sort_by(|a, b| (&a.station, &a.mode, &a.path).cmp(&(&b.station, &b.mode, &b.path)));
        for path in &mut inputs.paths {
            path.grids.sort_by(|a, b| a.id.cmp(&b.id));
        }
        let fingerprint = Fingerprint::of(
            format!(
                "{:?}|{inputs:?}|{nominal:?}|{}",
                self.fingerprint,
                sim_mujoco::version()
            )
            .as_bytes(),
        );
        (inputs, fingerprint)
    }
}

pub(crate) fn validate_limits(limit: &Limit) -> Result<(), Error> {
    limit.signal().validate()?;
    match limit {
        Limit::Range {
            min, max, source, ..
        } => {
            source.validate()?;
            if min.is_none() && max.is_none()
                || min.iter().chain(max).any(|x| !x.is_finite())
                || min.zip(*max).is_some_and(|(a, b)| a > b)
            {
                return Err(invalid("range: неверные пределы"));
            }
        }
        Limit::Forbidden { signal, source } => {
            source.validate()?;
            if signal.unit != "1" {
                return Err(invalid("forbidden: нужен двоичный сигнал с единицей 1"));
            }
        }
    }
    Ok(())
}

fn validate_combination(combination: &FaultCombination) -> Result<Vec<Fault>, Error> {
    name(&combination.path)?;
    combination.source.validate()?;
    let mut faults = combination.faults.clone();
    faults.sort();
    if faults.len() < 2 || faults.windows(2).any(|w| w[0] == w[1]) {
        return Err(invalid("нужны минимум два различных отказа"));
    }
    for fault in &faults {
        name(&fault.node)?;
    }
    Ok(faults)
}

impl OperatingMode {
    fn validate(&self, native: bool) -> Result<(), Error> {
        self.source.validate()?;
        if self.horizon_ns == 0 {
            return Err(invalid("horizon_ns: нужен положительный горизонт"));
        }
        for (id, window) in &self.windows {
            field(
                &format!("windows.{id}"),
                name(id).and_then(|()| window.validate()),
            )?;
            if window.end_ns > self.horizon_ns {
                return Err(invalid(format!("windows.{id}: конец за горизонтом")));
            }
        }
        let mut ids = BTreeSet::new();
        let mut previous = 0;
        for (i, step) in self.steps.iter().enumerate() {
            let context = format!("steps[{i}]");
            field(
                &context,
                name(&step.id)
                    .and_then(|()| name(&step.unit))
                    .and_then(|()| step.source.validate()),
            )?;
            if !ids.insert(&step.id)
                || !step.value.is_finite()
                || step.at_ns < previous
                || step.latest_ns().is_none_or(|time| time >= self.horizon_ns)
            {
                return Err(invalid(format!(
                    "{context}: повтор, неконечное значение или неверное время"
                )));
            }
            previous = step.at_ns;
        }
        let mut faults = BTreeSet::new();
        for (i, injection) in self.faults.iter().enumerate() {
            let context = format!("faults[{i}]");
            field(
                &context,
                name(&injection.fault.node).and_then(|()| injection.source.validate()),
            )?;
            let window = self
                .windows
                .get(&injection.window)
                .ok_or_else(|| invalid(format!("{context}.window: неизвестное окно")))?;
            let latest = injection
                .at_ns
                .checked_add(injection.max_lag_ns)
                .ok_or_else(|| invalid(format!("{context}.max_lag_ns: переполнение времени")))?;
            if injection.at_ns <= window.start_ns
                || latest >= window.end_ns
                || !faults.insert(&injection.fault)
            {
                return Err(invalid(format!(
                    "{context}: повтор или отсутствует окно до/после отказа"
                )));
            }
            match (injection.fault.kind, injection.extra_delay_ns) {
                (FaultKind::PacketDelay, Some(delay))
                    if delay > 0 && self.horizon_ns.checked_add(delay).is_some() =>
                {
                    ()
                }
                (FaultKind::PacketDelay, _) | (_, Some(_)) => {
                    return Err(invalid(format!(
                        "{context}.extra_delay_ns: неверная задержка отказа"
                    )));
                }
                _ => (),
            }
        }
        let mut sets = BTreeSet::new();
        for (i, combination) in self.combinations.iter().enumerate() {
            let ordered = validate_combination(combination)
                .map_err(|e| invalid(format!("combinations[{i}]: {e}")))?;
            if !sets.insert((&combination.path, ordered)) {
                return Err(invalid(format!("combinations[{i}]: повтор набора отказов")));
            }
        }
        ids.clear();
        for (i, criterion) in self.criteria.iter().enumerate() {
            let context = format!("criteria[{i}]");
            field(
                &context,
                name(&criterion.id).and_then(|()| validate_limits(&criterion.limit)),
            )?;
            if !ids.insert(&criterion.id) || !self.windows.contains_key(&criterion.window) {
                return Err(invalid(format!(
                    "{context}: повтор критерия или неизвестное окно"
                )));
            }
        }
        let mut hazards = BTreeSet::new();
        for (i, hazard) in self.hazards.iter().enumerate() {
            let context = format!("hazards[{i}]");
            field(
                &context,
                name(&hazard.class)
                    .and_then(|()| name(&hazard.object))
                    .and_then(|()| hazard.source.validate()),
            )?;
            if !self.windows.contains_key(&hazard.window)
                || !hazards.insert((&hazard.class, &hazard.object, &hazard.window))
            {
                return Err(invalid(format!(
                    "{context}: повтор обязательства или неизвестное окно"
                )));
            }
        }
        let mut ids = BTreeSet::new();
        for (i, metric) in self.metrics.iter().enumerate() {
            field(&format!("metrics[{i}]"), metric.validate(self))?;
            if !ids.insert(metric.id.as_str()) {
                return Err(invalid(format!("metrics[{i}].id: повтор метрики")));
            }
        }
        if native && self.criteria.is_empty() && !self.metrics.iter().any(MetricSpec::is_criterion)
        {
            return Err(invalid("criteria: общей постановке нужен критерий успеха"));
        }
        Ok(())
    }
}

impl ResolvedInputs {
    fn validate(&self) -> Result<(), Error> {
        if self.paths.is_empty() {
            return Err(invalid("profile.paths: пустой список обязательных путей"));
        }
        let mut paths = BTreeSet::new();
        for path in &self.paths {
            let context = format!("profile.{}.{}.{}", path.station, path.mode, path.path);
            let mut validate = || {
                for id in [&path.station, &path.mode, &path.path] {
                    name(id)?;
                }
                path.source.validate()?;
                if !paths.insert((&path.station, &path.mode, &path.path)) || path.grids.is_empty() {
                    return Err(invalid("повтор пути или отсутствуют сетки"));
                }
                if let Some(reason) = &path.unavailable {
                    name(reason)?;
                }
                let mut grids = BTreeSet::new();
                let mut rates = BTreeSet::new();
                for grid in &path.grids {
                    name(&grid.id)?;
                    grid.source.validate()?;
                    if !grids.insert(&grid.id)
                        || !rates.insert(grid.hz)
                        || Rate::hz(grid.hz).is_none()
                    {
                        return Err(invalid(format!(
                            "grids.{}: повтор или неверная частота",
                            grid.id
                        )));
                    }
                }
                for (fault, support) in &path.faults {
                    name(&fault.node)?;
                    match support {
                        FaultSupport::Available { channel, resource } => {
                            name(channel)?;
                            name(resource)?;
                        }
                        FaultSupport::Unsupported { reason } => name(reason)?,
                        FaultSupport::Inapplicable { reason, basis } => {
                            name(reason)?;
                            basis.validate()?;
                        }
                    }
                }
                let mut sets = BTreeSet::new();
                for combination in &path.combinations {
                    let faults = validate_combination(combination)?;
                    if combination.path != path.path
                        || faults.iter().any(|fault| !path.faults.contains_key(fault))
                        || !sets.insert(faults)
                    {
                        return Err(invalid("combinations: повтор или отказ вне пути"));
                    }
                }
                Ok(())
            };
            field(&context, validate())?;
        }
        for (target, binding) in &self.targets {
            let target = match target {
                Target::Effort { name } | Target::Port { name } | Target::Operation { name } => {
                    name
                }
            };
            name(target)?;
            name(&binding.resource)?;
            name(&binding.unit)?;
            if let Some(reason) = &binding.unavailable {
                name(reason)?;
            }
        }
        for signal in &self.signals {
            signal.validate()?;
        }
        for (parameter, binding) in &self.model_parameters {
            name(&binding.unit)?;
            if !matches!(
                parameter,
                Parameter::SensorNoise { .. }
                    | Parameter::SensorBias { .. }
                    | Parameter::SensorLatency { .. }
                    | Parameter::ChannelDelay { .. }
            ) || ![binding.min, binding.nominal, binding.max]
                .into_iter()
                .all(f64::is_finite)
                || !(binding.min <= binding.nominal && binding.nominal <= binding.max)
            {
                return Err(invalid(format!(
                    "model_parameters.{parameter:?}: неверный адрес или диапазон"
                )));
            }
        }
        // Полный мировой документ проверяется при загрузке. Здесь проверяются также
        // типизированные модели, пришедшие от разрешителя, включая неиспользуемые поля.
        for (name, device) in &self.world.devices {
            let table = toml::Value::try_from(&device.device)
                .map_err(|e| invalid(format!("{}: devices.{name}: {e}", device.path.display())))?;
            finite_toml(
                &table,
                &format!("{}: devices.{name}", device.path.display()),
            )?;
        }
        for (i, object) in self.world.contact.added_objects.iter().enumerate() {
            let table = toml::Value::try_from(object)
                .map_err(|e| invalid(format!("contact.added_objects[{i}]: {e}")))?;
            finite_toml(&table, &format!("contact.added_objects[{i}]"))?;
        }
        for (i, attachment) in self.world.contact.added_attachments.iter().enumerate() {
            let table = toml::Value::try_from(attachment)
                .map_err(|e| invalid(format!("contact.added_attachments[{i}]: {e}")))?;
            finite_toml(&table, &format!("contact.added_attachments[{i}]"))?;
        }
        Ok(())
    }

    fn effort_unit(world: &CompiledWorld, name: &str) -> Option<&'static str> {
        let effort = world.efforts.iter().find(|e| e.name == name)?;
        let joint = world.joints.iter().find(|j| j.name == effort.joint)?;
        use sim_world::CompiledJointKind::*;
        Some(match joint.kind {
            Slide => "N",
            Free if effort.dof < 3 => "N",
            Hinge | Ball | Free => "N*m",
        })
    }

    fn validate_efforts(&self, world: &CompiledWorld) -> Result<(), Error> {
        for (target, binding) in &self.targets {
            if let Target::Effort { name } = target
                && Self::effort_unit(world, name) != Some(binding.unit.as_str())
            {
                return Err(invalid(format!(
                    "targets.{name}: неразрешённое усилие или неверная единица"
                )));
            }
        }
        Ok(())
    }
}

fn apply_axis(
    axis: &Axis,
    value: f64,
    source: &Source,
    inputs: &ResolvedInputs,
    nominal: &CompiledWorld,
    world: &mut WorldInputs,
    settings: &mut ModelSettings,
) -> Result<(), Error> {
    if let Some(binding) = inputs.model_parameters.get(&axis.parameter)
        && (axis.min < binding.min || axis.max > binding.max)
    {
        return Err(invalid("допуск выходит за область модели"));
    }
    axis.parameter.apply(
        Some(axis.nominal),
        &axis.unit,
        value,
        source,
        &inputs.model_parameters,
        nominal,
        world,
        settings,
    )
}

fn apply_initial(
    mode: &OperatingMode,
    inputs: &ResolvedInputs,
    nominal: &CompiledWorld,
    world: &mut WorldInputs,
    settings: &mut ModelSettings,
) -> Result<(), Error> {
    for (i, value) in mode.initial.iter().enumerate() {
        field(
            &format!("initial[{i}]"),
            value.parameter.apply(
                None,
                &value.unit,
                value.value,
                &value.source,
                &inputs.model_parameters,
                nominal,
                world,
                settings,
            ),
        )?;
    }
    for (i, initial) in mode.poses.iter().enumerate() {
        let absent = || {
            invalid(format!(
                "poses[{i}]: неразрешённая поза {:?}",
                initial.target
            ))
        };
        let pose = match &initial.target {
            PoseTarget::Placement { placement } => world
                .world
                .devices
                .iter_mut()
                .find(|d| d.name == *placement)
                .map(|d| &mut d.pose),
            PoseTarget::Link { placement, link } => world
                .devices
                .get_mut(placement)
                .and_then(|d| d.device.links.iter_mut().find(|l| l.name == *link))
                .map(|l| &mut l.pose),
            PoseTarget::Object { object } => world
                .world
                .objects
                .iter_mut()
                .chain(&mut world.contact.added_objects)
                .find(|o| o.name == *object)
                .map(|o| &mut o.pose),
        }
        .ok_or_else(absent)?;
        *pose = initial.pose.clone();
    }
    Ok(())
}

impl Campaign {
    fn validate_mode(
        mode: &OperatingMode,
        paths: &[&RequiredPath],
        inputs: &ResolvedInputs,
        nominal: &CompiledWorld,
        axes: &[Axis],
    ) -> Result<(), Error> {
        if paths.is_empty() {
            return Err(invalid(
                "profile.paths: режим отсутствует в разрешённом профиле",
            ));
        }
        let mut occupied: BTreeSet<_> = axes.iter().map(|a| a.parameter.clone()).collect();
        for (i, value) in mode.initial.iter().enumerate() {
            let context = format!("initial[{i}]");
            field(&context, value.source.validate())?;
            if !matches!(
                value.parameter,
                Parameter::PlacementOffset { .. }
                    | Parameter::LinkOffset { .. }
                    | Parameter::ObjectOffset { .. }
                    | Parameter::JointPosition { .. }
                    | Parameter::JointVelocity { .. }
                    | Parameter::Gravity { .. }
                    | Parameter::Wind { .. }
                    | Parameter::Density
                    | Parameter::Viscosity
            ) || !occupied.insert(value.parameter.clone())
            {
                return Err(invalid(format!(
                    "{context}: изменение вне начального состояния или конфликт наложений"
                )));
            }
            let bounded = Axis {
                id: context.clone(),
                parameter: value.parameter.clone(),
                unit: value.unit.clone(),
                nominal: value.value,
                min: value.value,
                max: value.value,
                values: None,
                source: value.source.clone(),
                scope: Scope::Run,
            };
            field(&context, value.parameter.check_domain(&bounded))?;
        }
        for (i, pose) in mode.poses.iter().enumerate() {
            let context = format!("poses[{i}]");
            field(&context, pose.source.validate())?;
            // Поза владеет всеми тремя координатами и ориентацией этого кадра.
            for axis in 0..3 {
                let parameter = match &pose.target {
                    PoseTarget::Placement { placement } => Parameter::PlacementOffset {
                        placement: placement.clone(),
                        axis,
                    },
                    PoseTarget::Link { placement, link } => Parameter::LinkOffset {
                        placement: placement.clone(),
                        link: link.clone(),
                        axis,
                    },
                    PoseTarget::Object { object } => Parameter::ObjectOffset {
                        object: object.clone(),
                        axis,
                    },
                };
                if !occupied.insert(parameter) {
                    return Err(invalid(format!(
                        "{context}: конфликт позы с другим наложением"
                    )));
                }
            }
        }
        let mut world = inputs.world.clone();
        apply_initial(
            mode,
            inputs,
            nominal,
            &mut world,
            &mut ModelSettings::default(),
        )?;
        if !mode.poses.is_empty()
            || mode
                .initial
                .iter()
                .any(|i| !matches!(i.parameter, Parameter::JointVelocity { .. }))
        {
            world.compile()?;
        }
        let mut occupied = BTreeMap::new();
        for (i, step) in mode.steps.iter().enumerate() {
            let context = format!("steps[{i}]");
            let binding = inputs.targets.get(&step.target).ok_or_else(|| {
                invalid(format!(
                    "{context}.target: неразрешённый адрес {:?}",
                    step.target
                ))
            })?;
            let latest = step
                .latest_ns()
                .ok_or_else(|| invalid(format!("{context}.max_lag_ns: переполнение времени")))?;
            if step.unit != binding.unit
                || occupied
                    .insert(&binding.resource, latest)
                    .is_some_and(|previous| previous >= step.at_ns)
            {
                return Err(invalid(format!(
                    "{context}: неверная единица или пересекаются интервалы действий на одном ресурсе"
                )));
            }
        }
        for (i, criterion) in mode.criteria.iter().enumerate() {
            if !inputs.signals.contains(criterion.limit.signal()) {
                return Err(invalid(format!(
                    "criteria[{i}].limit.signal: неразрешённый сигнал или единица"
                )));
            }
        }
        for (i, metric) in mode.metrics.iter().enumerate() {
            if metric
                .signals()
                .any(|signal| !inputs.signals.contains(signal))
            {
                return Err(invalid(format!(
                    "metrics[{i}]: неразрешённый сигнал или единица"
                )));
            }
        }
        for (i, injection) in mode.faults.iter().enumerate() {
            if !paths
                .iter()
                .any(|p| p.faults.contains_key(&injection.fault))
            {
                return Err(invalid(format!(
                    "faults[{i}].fault: отказ вне разрешённого состава"
                )));
            }
        }
        for (i, combination) in mode.combinations.iter().enumerate() {
            if !paths.iter().any(|p| {
                p.path == combination.path
                    && combination.faults.iter().all(|f| p.faults.contains_key(f))
            }) {
                return Err(invalid(format!(
                    "combinations[{i}]: набор отказов вне разрешённого пути"
                )));
            }
        }
        let max_setting_delay = axes
            .iter()
            .filter(|a| {
                matches!(
                    a.parameter,
                    Parameter::SensorLatency { .. } | Parameter::ChannelDelay { .. }
                )
            })
            .try_fold(0u64, |sum, a| {
                sum.checked_add(crate::sampling::delay_ns(a.max).expect("граница проверена"))
                    .ok_or_else(|| invalid("sampling: переполнение суммы задержек"))
            })?;
        let max_fault_delay = mode
            .faults
            .iter()
            .filter_map(|f| f.extra_delay_ns)
            .try_fold(0u64, |sum, delay| {
                sum.checked_add(delay)
                    .ok_or_else(|| invalid("faults: переполнение суммы задержек"))
            })?;
        mode.horizon_ns
            .checked_add(max_setting_delay)
            .and_then(|t| t.checked_add(max_fault_delay))
            .ok_or_else(|| invalid("horizon_ns: горизонт и задержки переполняют время"))?;
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct RunKey {
    pub station: String,
    pub mode: String,
    pub path: String,
    pub faults: Vec<Fault>,
    pub sample: SampleKey,
    pub grid: String,
}

#[derive(Clone, Debug, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Disposition {
    Ready,
    NotAnalyzed {
        reasons: Vec<String>,
    },
    Inapplicable {
        reasons: Vec<String>,
        bases: Vec<Source>,
    },
}

#[derive(Clone, Debug, Serialize)]
pub struct PlannedFault {
    pub schedule: ScheduledFault,
    pub channel: String,
    pub window: Window,
}

#[derive(Clone, Debug, Serialize)]
pub struct PlanRow {
    pub key: RunKey,
    pub grid: Grid,
    pub profile_source: Source,
    pub combination_source: Option<Source>,
    pub faults: Vec<PlannedFault>,
    pub disposition: Disposition,
}

impl Campaign {
    fn row(
        &self,
        key: RunKey,
        path: &RequiredPath,
        mode: Option<&OperatingMode>,
        grid: &Grid,
        combination_source: Option<&Source>,
        inputs: &ResolvedInputs,
        needs_velocity: bool,
    ) -> PlanRow {
        let mut reasons = Vec::new();
        let mut inapplicable = Vec::new();
        let mut bases = Vec::new();
        let mut faults = Vec::new();
        let mut resources = BTreeSet::new();
        if let Some(reason) = &path.unavailable {
            reasons.push(reason.clone());
        }
        if let Some(mode) = mode {
            if needs_velocity
                || !inputs.initial_velocity
                    && mode.initial.iter().any(|i| {
                        matches!(i.parameter, Parameter::JointVelocity { .. }) && i.value != 0.0
                    })
            {
                reasons.push("установщик не поддерживает ненулевую начальную скорость".into());
            }
            for step in &mode.steps {
                if let Some(reason) = &inputs.targets[&step.target].unavailable {
                    reasons.push(format!("{}: {reason}", step.id));
                }
            }
        } else {
            reasons.push("в кампании отсутствует обязательный режим или постановка".into());
        }
        for fault in &key.faults {
            match &path.faults[fault] {
                FaultSupport::Inapplicable { reason, basis } if basis.can_prove() => {
                    inapplicable.push(format!("{fault:?}: {reason}"));
                    bases.push(basis.clone());
                }
                FaultSupport::Unsupported { reason }
                | FaultSupport::Inapplicable { reason, .. } => {
                    reasons.push(format!("{fault:?}: {reason}"))
                }
                FaultSupport::Available { channel, resource } => {
                    if !resources.insert(resource) {
                        reasons.push(format!(
                            "{fault:?}: совместные отказы конфликтуют на ресурсе {resource}"
                        ));
                    }
                    if let Some((mode, schedule)) = mode
                        .and_then(|m| m.faults.iter().find(|i| i.fault == *fault).map(|i| (m, i)))
                    {
                        faults.push(PlannedFault {
                            channel: channel.clone(),
                            schedule: schedule.clone(),
                            window: mode.windows[&schedule.window],
                        });
                    } else {
                        reasons.push(format!("{fault:?}: отсутствует расписание инъекции"));
                    }
                }
            }
        }
        faults.sort_by(|a, b| {
            (a.schedule.at_ns, &a.schedule.fault).cmp(&(b.schedule.at_ns, &b.schedule.fault))
        });
        // У доказанно отсутствующего одиночного отказа нет расписания.
        // У комбинации конфликт и неполнота остальных отказов остаются видимыми.
        let disposition =
            if !inapplicable.is_empty() && (key.faults.len() == 1 || reasons.is_empty()) {
                Disposition::Inapplicable {
                    reasons: inapplicable,
                    bases,
                }
            } else if !reasons.is_empty() {
                reasons.extend(inapplicable);
                Disposition::NotAnalyzed { reasons }
            } else {
                Disposition::Ready
            };
        PlanRow {
            key,
            grid: grid.clone(),
            profile_source: path.source.clone(),
            combination_source: combination_source.cloned(),
            faults,
            disposition,
        }
    }
}

impl Campaign {
    fn materialize_row(
        &self,
        row: &PlanRow,
        sample: Sample,
        axes: &[Axis],
        inputs: &ResolvedInputs,
        nominal: &CompiledWorld,
        fingerprint: Fingerprint,
    ) -> Result<SampledInputs, Error> {
        let key = &row.key;
        let mut world = inputs.world.clone();
        world.world.seed = sample.seeds.run;
        world.world.physics_hz = row.grid.hz;
        world.contact.physics = Some(Rate::hz(row.grid.hz).expect("сетка проверена"));
        let mut settings = ModelSettings::default();
        let station = self.stations.get(&key.station);
        let mode = station.and_then(|station| station.modes.get(&key.mode));
        if let Some(mode) = mode {
            apply_initial(mode, inputs, nominal, &mut world, &mut settings)?;
        }
        for (axis, value) in axes.iter().zip(&sample.values) {
            field(
                &format!(
                    "{}: sampling.axes.{}",
                    self.file.display(),
                    axis.id
                ),
                apply_axis(
                    axis,
                    value.value,
                    &value.source,
                    inputs,
                    nominal,
                    &mut world,
                    &mut settings,
                ),
            )?;
        }
        let fingerprint = Fingerprint::of(
            format!("{:?}|{key:?}|{sample:?}|{settings:?}", fingerprint).as_bytes(),
        );
        Ok(SampledInputs {
            key: key.clone(),
            sample,
            world,
            settings,
            mode: mode.cloned(),
            experiment: station.map(|s| s.experiment.clone()),
            fingerprint,
        })
    }
}

#[derive(Clone, Debug)]
pub struct CampaignPlan {
    rows: Vec<PlanRow>,
    campaign: Campaign,
    inputs: ResolvedInputs,
    nominal: CompiledWorld,
    fingerprint: Fingerprint,
    participant_candidates: BTreeSet<String>,
}

#[derive(Clone, Debug)]
pub struct SampledInputs {
    pub key: RunKey,
    pub sample: Sample,
    pub world: WorldInputs,
    pub settings: ModelSettings,
    pub mode: Option<OperatingMode>,
    pub experiment: Option<Experiment>,
    /// Включает выбранные наложения, постановку, сетку и разрешённый граф.
    pub fingerprint: Fingerprint,
}

impl CampaignPlan {
    pub(crate) fn sampling_gap(&self) -> Result<Option<&'static str>, Error> {
        let sampling = &self.campaign.sampling;
        if !self.campaign.sampling_declared {
            return Ok(Some("отсутствует обязательный раздел разброса sampling"));
        }
        sampling.coverage_gap(self.campaign.seed)
    }

    pub(crate) fn world_inputs(&self) -> &WorldInputs {
        &self.inputs.world
    }

    pub(crate) fn varying_axes(&self) -> impl Iterator<Item = &str> {
        self.campaign.sampling.axes().iter().filter(|a| a.min < a.max).map(|a| a.id.as_str())
    }

    pub(crate) fn has_discrete_envelope(&self, sourced: impl Fn(&Source) -> bool) -> bool {
        self.campaign.sampling.discrete_envelope_with(sourced)
    }

    pub fn rows(&self) -> &[PlanRow] {
        &self.rows
    }

    pub fn row(&self, key: &RunKey) -> Option<&PlanRow> {
        self.rows
            .binary_search_by(|row| row.key.cmp(key))
            .ok()
            .map(|index| &self.rows[index])
    }

    pub fn fingerprint(&self) -> Fingerprint {
        self.fingerprint
    }

    pub(crate) fn participant_candidates(&self) -> &BTreeSet<String> {
        &self.participant_candidates
    }

    pub(crate) fn cyclic_efforts(&self) -> BTreeSet<&str> {
        use std::cmp::Ordering::{self, Equal};
        let mut cyclic = BTreeSet::new();
        for mode in self.campaign.stations.values().flat_map(|station| station.modes.values()) {
            let mut last = BTreeMap::<&str, (f64, Ordering)>::new();
            for step in &mode.steps {
                let Target::Effort { name } = &step.target else { continue };
                // До команды действует нулевой вход. Смена знака приращения
                // образует нагружение и разгрузку даже без реверса оси.
                let (previous, slope) = last.entry(name).or_insert((0.0, Equal));
                let change = step.value.partial_cmp(previous).expect("команда проверена");
                if change != Equal {
                    if *slope != Equal && *slope != change {
                        cyclic.insert(name.as_str());
                    }
                    *slope = change;
                }
                *previous = step.value;
            }
        }
        cyclic
    }

    fn validate_materialized_worlds(&mut self) -> Result<(), Error> {
        let mut checked = BTreeSet::new();
        for row in &self.rows {
            // Путь контура, состав отказов и зерно не меняют компиляцию мира.
            let setup = (
                &row.key.station,
                &row.key.mode,
                row.grid.hz,
                (!self.campaign.sampling.axes.is_empty()).then_some(&row.key.sample),
            );
            if checked.insert(setup) {
                let sampled = self.materialize(&row.key)?;
                let compiled = sampled.world.compile()
                    .map_err(|e| invalid(format!("matrix.{:?}: {e}", row.key)))?;
                self.participant_candidates.extend(
                    compiled.powered_contact_candidates().map_err(invalid)?,
                );
                self.participant_candidates.extend(sampled.world.open_powered_mounts());
            }
        }
        Ok(())
    }

    pub fn materialize(&self, key: &RunKey) -> Result<SampledInputs, Error> {
        let row = self
            .row(key)
            .ok_or_else(|| invalid(format!("matrix: неизвестный ключ {key:?}")))?;
        let sample = self.sample(row)?;
        self.campaign.materialize_row(
            row,
            sample,
            &self.campaign.sampling.axes,
            &self.inputs,
            &self.nominal,
            self.fingerprint,
        )
    }

    pub(crate) fn task_fingerprint(&self, key: &RunKey) -> Option<Fingerprint> {
        let task = self
            .campaign
            .stations
            .get(&key.station)?
            .experiment
            .task
            .as_ref()?;
        self.inputs.references.get(task)?.file_fingerprint
    }

    pub(crate) fn mode(&self, key: &RunKey) -> Option<&OperatingMode> {
        self.campaign
            .stations
            .get(&key.station)?
            .modes
            .get(&key.mode)
    }

    pub(crate) fn has_linked_stations(&self) -> bool {
        self.campaign.stations.values().any(|station| station.experiment.kind.is_some())
    }

    pub(crate) fn probability(&self) -> Option<&crate::sampling::Probability> {
        self.campaign.sampling.probability.as_ref()
    }

    pub(crate) fn sample(&self, row: &PlanRow) -> Result<Sample, Error> {
        let key = &row.key;
        // Сетка и ветвь парного опыта не входят в зерно модели.
        let mut fields = vec!["campaign/v1", "run", &key.station, &key.mode, &key.path];
        let faults: Vec<_> = key.faults.iter().map(|f| format!("{:?}", f.kind)).collect();
        for (fault, kind) in key.faults.iter().zip(&faults) {
            fields.extend([fault.node.as_str(), kind.as_str()]);
        }
        self.campaign
            .sampling
            .sample(self.campaign.seed, &key.sample, &fields)
    }
}

impl SampledInputs {
    pub fn values(&self) -> &[AppliedValue] {
        &self.sample.values
    }
}
