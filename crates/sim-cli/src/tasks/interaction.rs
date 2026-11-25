//! Сборка связанного опыта из файлов и численный аудит завершённых интервалов.

use std::{
    collections::{BTreeMap, BTreeSet},
    io::Write,
    path::{Path, PathBuf},
};

use super::campaign::{
    installation::{LinkedInventory, LinkedSample, Receipt},
    playback,
};
use serde::{Deserialize, Serialize};
use sim_human::{
    Sex,
    scene::{self, BaseVariant, ParticipantSpec, RegionId},
};
use sim_process::{
    Analysis, Interval, SourcedRange,
    attachments::{
        CompliantPad, ConfirmedEvent, ConfirmedInterfaceEvent, CurvePoint, ForceCurve,
        FreeLengthLaw, InterfaceState, PadCalibration, PadLaw, PadReactionOwner, PadSpec,
        PatchAreaPoint, TensionCalibration, TensionRoute, TensionSpec, TractionFriction,
    },
    contact::{ContactMaterialTable, GeometryMaterial},
};
use sim_runtime::{
    LogKind, Runtime, Status, control,
    process::interaction::{
        LocalMount, LocalPoint, Maneuver, MotionAxis, MotionProfile, PadBinding,
        ResolvedInteraction, Response, RouteBinding, RouteSurface, Trajectory,
    },
};
use sim_signal::{ChannelId, Time};
use sim_task::{
    Fingerprint, Outcome,
    interaction::{
        ContactMetric, ContactTolerances, CriterionKind, EnergyBounds, EnergyPath,
        InteractionEvidence, InteractionPlan, LimitCriterion, PatchPlan, PowerInventory,
        PowerLossPlan, PowerSource, PowerWitness, Reach, ReferenceDiscrepancy,
        assess_interaction_grids, safety_decision,
    },
};
use sim_task::{campaign::WorldInputs, sampling::ModelSettings};
use sim_world::{
    ContactBuildOptions, DeviceInput, FragmentBindings, FragmentEffort, FragmentInertia,
    FragmentPort, ModelFragment, MountKind, Pose, Provenance, ResolvedPort,
};

use super::{
    Models, SampledMode, Setup, frame,
    processes::{
        ContactInput, Control, Inputs, ModelSet, Origin, PairRow, PairTable, Scalar, Wiring,
        capture_world, dir, ensure_finite, merge_tables, parse,
    },
    read,
};
use crate::{Options, export};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct TaskFile {
    version: u32,
    interaction_profile: PathBuf,
}

impl TaskFile {
    pub(super) fn load(path: &Path) -> Result<Option<Self>, String> {
        let bytes = read(path)?;
        let table: toml::Table = parse(path, &bytes)?;
        if !table.contains_key("interaction_profile") {
            return Ok(None);
        }
        ensure_finite(&toml::Value::Table(table))?;
        let task: Self = parse(path, &bytes)?;
        if task.version != 1 {
            return Err("неподдерживаемая версия задания взаимодействия".into());
        }
        Ok(Some(task))
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Profile {
    version: u32,
    source: String,
    base: PathBuf,
    cases: Vec<Variant>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Variant {
    id: String,
    overlay: Option<PathBuf>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Case {
    source: String,
    horizon_ns: u64,
    snapshot_ns: Option<u64>,
    audit_from_ns: u64,
    participant: Participant,
    motion: Motion,
    #[serde(default)]
    pads: BTreeMap<String, Pad>,
    #[serde(default)]
    routes: BTreeMap<String, Route>,
    contacts: ContactInput,
    control: Option<Control>,
    sampled_mode: Option<SampledMode>,
    #[serde(default)]
    processes: ModelSet,
    #[serde(default)]
    placement_overrides: BTreeMap<String, Pose>,
    #[serde(default)]
    probes: BTreeMap<String, Probe>,
    camera: Option<CameraAudit>,
    power: Inventory,
    power_loss: Option<PowerLoss>,
    tolerances: Tolerances,
    checks: Vec<Check>,
    #[serde(default)]
    limits: Vec<DesignLimit>,
    construction_complete: bool,
}

#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
enum Population {
    Female,
    Male,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Participant {
    name: String,
    population: Population,
    stature_m: Scalar,
    mass_kg: Scalar,
    pose: Pose,
    #[serde(default)]
    local_dimensions: Vec<Dimensions>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Dimensions {
    region: String,
    metres: [f64; 3],
    source: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Motion {
    axes: BTreeMap<String, Axis>,
    maneuver: Option<TimedOffsets>,
    response: Option<ReactiveOffsets>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Axis {
    channel: String,
    initial_position: f64,
    initial_velocity: f64,
    reference: Track,
    stiffness: Scalar,
    damping: Scalar,
    effort_limit: Scalar,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Track {
    #[serde(with = "Origin")]
    source: Provenance,
    knots: Vec<(u64, f64)>,
}

impl From<Track> for Trajectory {
    fn from(track: Track) -> Self {
        Self {
            source: track.source,
            knots: track.knots.into_iter().map(|(t, q)| (Time(t), q)).collect(),
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct TimedOffsets {
    at_ns: u64,
    initiator: String,
    offsets: BTreeMap<String, Track>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ReactiveOffsets {
    enabled: bool,
    initiator: String,
    patch: String,
    threshold_n: Scalar,
    latency_ns: u64,
    #[serde(with = "Origin")]
    latency_source: Provenance,
    offsets: BTreeMap<String, Track>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Pad {
    mounts: [String; 2],
    geometries: [String; 2],
    /// Нормаль в локальной рамке порта, от первой стороны ко второй.
    normal: [[f64; 3]; 2],
    tangent: [[f64; 3]; 2],
    area_m2: Option<Scalar>,
    #[serde(default)]
    area_curve: Vec<AreaPoint>,
    calibration: PadWindow,
    law: Option<PadModel>,
    #[serde(default)]
    events: Vec<InterfaceEvent>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct AreaPoint {
    compression_m: f64,
    area_m2: Scalar,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct PadWindow {
    compression_m: Scalar,
    offset_m: Scalar,
    axis_error_rad: Scalar,
    relative_speed_m_s: Scalar,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct PadModel {
    free_gap_m: Scalar,
    compression: Curve,
    damping_ns_m: Scalar,
    shear_stiffness_n_m: Scalar,
    friction: Scalar,
    capacity_n: Scalar,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Curve {
    points: Vec<[f64; 2]>,
    #[serde(with = "Origin")]
    source: Provenance,
}

impl Curve {
    fn build(&self) -> Result<ForceCurve, String> {
        let sourced = |v| SourcedRange {
            min: v,
            nominal: v,
            max: v,
            provenance: self.source.clone(),
        };
        ForceCurve::new(
            self.points
                .iter()
                .map(|&[x, f]| CurvePoint {
                    displacement_m: sourced(x),
                    force_n: sourced(f),
                })
                .collect(),
        )
        .map_err(|e| e.to_string())
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Route {
    mounts: Vec<String>,
    length: Length,
    extension: Curve,
    damping_ns_m: Scalar,
    capacity_n: Scalar,
    calibration: RouteWindow,
    friction: Option<Traction>,
    surfaces: BTreeMap<String, Surface>,
    #[serde(default)]
    events: Vec<InterfaceEvent>,
}

#[derive(Clone, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Length {
    Fixed {
        metres: Scalar,
    },
    Affine {
        channel: String,
        at_zero_m: Scalar,
        slope: Scalar,
        coordinate: Scalar,
    },
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct RouteWindow {
    extension_m: Scalar,
    extension_speed_m_s: Scalar,
    payout_m: Scalar,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Traction {
    coefficient: Scalar,
    clamp_normal_n: Scalar,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Surface {
    geometry: String,
    point: usize,
    reference: usize,
    normal: [f64; 3],
    tangent: [f64; 3],
    area_m2: Option<Scalar>,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct InterfaceEvent {
    at_ns: u64,
    event: InterfaceEventKind,
}

#[derive(Clone, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum InterfaceEventKind {
    Release,
    Break,
    Slip { payout_m: f64 },
}

#[derive(Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Accessibility {
    Reachable,
    Unknown,
}

impl From<Accessibility> for Reach {
    fn from(value: Accessibility) -> Self {
        match value {
            Accessibility::Reachable => Reach::Reachable,
            Accessibility::Unknown => Reach::Unknown,
        }
    }
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Inventory {
    #[serde(with = "Origin")]
    topology: Provenance,
    required: BTreeSet<String>,
    sources: BTreeMap<String, Supply>,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Supply {
    reach: Accessibility,
    electrical: Option<Energy>,
    thermal: Option<Energy>,
    mechanical: Option<Energy>,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Energy {
    stored_j: Option<Scalar>,
    deliverable_j: Option<Scalar>,
    hazardous_above_j: Option<Scalar>,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct PowerLoss {
    id: String,
    target: String,
    at_ns: u64,
    loaded_patch: String,
    signal: String,
    unit: String,
    before: Scalar,
    after: Scalar,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Tolerances {
    onset_s: Scalar,
    force_n: Scalar,
    impulse_ns: Scalar,
    duration_s: Scalar,
    drift_m: Scalar,
    stress_pa: Scalar,
}

#[derive(Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Metric {
    NormalForce,
    TangentialForce,
    MeanPressure,
    MeanShear,
}

impl From<Metric> for ContactMetric {
    fn from(value: Metric) -> Self {
        match value {
            Metric::NormalForce => Self::NormalForce,
            Metric::TangentialForce => Self::TangentialForce,
            Metric::MeanPressure => Self::MeanPressure,
            Metric::MeanShear => Self::MeanShear,
        }
    }
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct DesignLimit {
    #[serde(default)]
    kind: LimitKind,
    patch: String,
    region: String,
    metric: Metric,
    limit: Scalar,
    duration_s: Scalar,
    area_m2: Option<Scalar>,
    #[serde(with = "Origin")]
    validation: Provenance,
}

#[derive(Clone, Copy, Default, Deserialize)]
#[serde(rename_all = "snake_case")]
enum LimitKind {
    #[default]
    Design,
    TissueScreen,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Check {
    id: String,
    bounds: [f64; 2],
    source: String,
    min_samples: usize,
    #[serde(default)]
    required_design: bool,
    window_ns: Option<[u64; 2]>,
    metric: Measure,
}

#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
enum Stat {
    Peak,
    Span,
    Last,
    Sum,
    Min,
}

#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
enum State {
    Open,
    Loaded,
    Sliding,
    Released,
    Broken,
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Measure {
    Normal {
        patch: String,
    },
    Shear {
        patch: String,
    },
    OpposedNormal {
        first: String,
        second: String,
    },
    Jerk {
        channel: String,
        sample_ns: u64,
    },
    Pressure {
        patch: String,
    },
    NominalPressure {
        patch: String,
    },
    Tension {
        route: String,
    },
    RoutePayout {
        route: String,
    },
    InterfaceState {
        interface: String,
        state: State,
    },
    ActionReactionResidual {
        interface: String,
    },
    VirtualWorkResidual {
        route: String,
    },
    LengthResidual {
        route: String,
    },
    PadLawResidual {
        patch: String,
    },
    Axis {
        channel: String,
        statistic: Stat,
        quantity: AxisQuantity,
    },
    Signal {
        name: String,
        unit: String,
        statistic: Stat,
        #[serde(default)]
        observation: bool,
    },
    Packet {
        route: String,
        quantity: PacketQuantity,
        statistic: Stat,
    },
    MotionEvents,
    PowerCut,
    Probe {
        name: String,
        unit: String,
        statistic: Stat,
    },
    CameraProjection,
    CameraShift,
}

#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
enum AxisQuantity {
    Position,
    Velocity,
    Effort,
}

#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
enum PacketQuantity {
    LatencyNs,
    Delivered,
    Lost,
    Sequence,
}

#[derive(Serialize)]
struct CheckReport<'a> {
    id: &'a str,
    rate_hz: f64,
    samples: usize,
    value: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    event_ns: Option<u64>,
    #[serde(skip)]
    window_ns: [u64; 2],
    bounds: [f64; 2],
    source: &'a str,
    outcome: Outcome,
    required_design: bool,
}

#[derive(Serialize)]
struct CaseReport<'a> {
    id: &'a str,
    source: &'a str,
    outcome: Outcome,
    numerical: Outcome,
    readiness: sim_task::interaction::SafetyDecision,
    seed: u64,
    coverage: Vec<Coverage>,
    #[serde(skip)]
    horizon_ns: u64,
    checks: Vec<CheckReport<'a>>,
    evidence: InteractionEvidence,
    convergence: BTreeMap<String, Vec<NumericalConvergence>>,
    tissue: Vec<TissueAudit>,
    safety_gaps: Vec<String>,
    #[serde(skip)]
    checkpointable: bool,
    #[serde(skip)]
    inventory: Option<LinkedInventory>,
    #[serde(skip)]
    resources: BTreeMap<PathBuf, u64>,
}

#[derive(Serialize)]
struct Coverage {
    rate_hz: f64,
    expected_steps: u64,
    completed_steps: u64,
    status: &'static str,
    binding: Fingerprint,
    control: Option<Fingerprint>,
    failure: Option<String>,
    camera: Option<CameraReport>,
    #[serde(skip_serializing_if = "Option::is_none")]
    snapshot: Option<sim_task::verdict::SnapshotAudit>,
    #[serde(skip_serializing_if = "Option::is_none")]
    installation: Option<Receipt>,
    motion: Vec<MotionReceipt>,
    control_assets: BTreeMap<String, Fingerprint>,
}

#[derive(Serialize)]
struct MotionReceipt {
    initiator: String,
    contact_ns: Option<u64>,
    due_ns: u64,
    applied_ns: [u64; 2],
    sources: Vec<String>,
}

#[derive(Serialize)]
struct Report<'a> {
    status: &'static str,
    outcome: Outcome,
    numerical: Outcome,
    worst_variant: &'a str,
    seed: u64,
    inputs: BTreeMap<String, Fingerprint>,
    cases: Vec<CaseReport<'a>>,
}

fn named(name: &str) -> bool {
    !name.trim().is_empty() && !name.chars().any(char::is_control)
}

fn sourced(scalar: &Scalar) -> Result<SourcedRange, String> {
    let value = SourcedRange::from(scalar.clone());
    value.validate().map_err(|e| e.to_string())?;
    Ok(value)
}

fn origin(s: scene::Source<'_>) -> Provenance {
    Provenance::Derived(format!("{} — {}", s.reference, s.locator))
}

impl Participant {
    fn build(&self, motion: &Motion) -> Result<(scene::ParticipantModel, ModelFragment), String> {
        let model = scene::build(&ParticipantSpec {
            base: BaseVariant::MsHuman700,
            population: match self.population {
                Population::Male => Sex::Male,
                Population::Female => Sex::Female,
            },
            stature_m: sourced(&self.stature_m)?.nominal,
            mass_kg: sourced(&self.mass_kg)?.nominal,
            proportions: self
                .local_dimensions
                .iter()
                .map(|d| scene::Proportion {
                    region: RegionId(d.region.clone()),
                    measure: scene::ProportionMeasure::LocalDimensions(d.metres),
                    source: d.source.clone(),
                })
                .collect(),
        })?;
        if motion.axes.is_empty() {
            return Err("профиль движения должен содержать физические оси".into());
        }
        for (joint, axis) in &motion.axes {
            let offsets = motion
                .maneuver
                .iter()
                .filter_map(|e| e.offsets.get(joint))
                .chain(motion.response.iter().filter_map(|e| e.offsets.get(joint)));
            let (min, max) = offsets.fold((0.0, 0.0), |(min, max), track| {
                (
                    min + track.knots.iter().map(|k| k.1).fold(0.0, f64::min),
                    max + track.knots.iter().map(|k| k.1).fold(0.0, f64::max),
                )
            });
            for value in std::iter::once(axis.initial_position).chain(
                axis.reference
                    .knots
                    .iter()
                    .flat_map(|k| [k.1 + min, k.1 + max]),
            ) {
                model.validate_pose(&BTreeMap::from([(joint.clone(), value)]))?;
            }
        }
        let fragment = ModelFragment {
            name: self.name.clone(),
            source: model.source.clone(),
            mjcf: Some(model.mjcf.clone()),
            pose: self.pose.clone(),
            inertia: model
                .exported_mass
                .segments
                .iter()
                .map(|seg| {
                    let [[xx, xy, xz], [_, yy, yz], [_, _, zz]] = seg.inertia_kg_m2;
                    (
                        seg.name.clone(),
                        FragmentInertia {
                            mass_kg: seg.mass_kg,
                            com_m: seg.com_m,
                            tensor_kg_m2: [xx, yy, zz, xy, xz, yz],
                            provenance: origin(seg.source),
                        },
                    )
                })
                .collect(),
            ports: model
                .ports
                .iter()
                .map(|port| {
                    (
                        port.site.clone(),
                        FragmentPort {
                            site: port.site.clone(),
                            kind: MountKind::Mechanical,
                            interface: "контактная поверхность".into(),
                        },
                    )
                })
                .collect(),
            surfaces: model
                .regions
                .iter()
                .map(|(id, region)| (id.0.clone(), region.geometry.clone()))
                .collect(),
            efforts: motion
                .axes
                .iter()
                .map(|(joint, axis)| FragmentEffort {
                    name: axis.channel.clone(),
                    joint: joint.clone(),
                    dof: 0,
                })
                .collect(),
            drives: Vec::new(),
        };
        Ok((model, fragment))
    }
}

impl Motion {
    fn resolve(&self, participant: &str, rt: &Runtime) -> Result<MotionProfile, String> {
        let axis_id = |joint: &str| {
            let axis = self
                .axes
                .get(joint)
                .ok_or_else(|| format!("не объявлена ось {joint}"))?;
            channel(rt, &format!("{participant}/{}", axis.channel))
        };
        let offsets =
            |tracks: &BTreeMap<String, Track>| -> Result<BTreeMap<ChannelId, Trajectory>, String> {
                tracks
                    .iter()
                    .map(|(joint, track)| Ok((axis_id(joint)?, track.clone().into())))
                    .collect()
            };
        let axes = self
            .axes
            .iter()
            .map(|(joint, axis)| {
                Ok(MotionAxis {
                    channel: axis_id(joint)?,
                    initial_position: axis.initial_position,
                    initial_velocity: axis.initial_velocity,
                    reference: axis.reference.clone().into(),
                    stiffness: sourced(&axis.stiffness)?,
                    damping: sourced(&axis.damping)?,
                    effort_limit: sourced(&axis.effort_limit)?,
                })
            })
            .collect::<Result<Vec<_>, String>>()?;
        let maneuver = self
            .maneuver
            .as_ref()
            .map(|m| -> Result<_, String> {
                Ok(Maneuver {
                    at: Time(m.at_ns),
                    initiator: m.initiator.clone(),
                    offsets: offsets(&m.offsets)?,
                })
            })
            .transpose()?;
        let response = self
            .response
            .as_ref()
            .map(|r| -> Result<_, String> {
                Ok(Box::new(Response {
                    enabled: r.enabled,
                    initiator: r.initiator.clone(),
                    patch: r.patch.clone(),
                    threshold_n: sourced(&r.threshold_n)?,
                    latency_ns: r.latency_ns,
                    latency_source: r.latency_source.clone(),
                    offsets: offsets(&r.offsets)?,
                }))
            })
            .transpose()?;
        Ok(MotionProfile::Dynamic {
            axes,
            maneuver,
            response,
        })
    }
}

fn channel(rt: &Runtime, name: &str) -> Result<ChannelId, String> {
    rt.channel(name)
        .ok_or_else(|| format!("нет канала усилия «{name}»"))
}

fn point(port: &ResolvedPort) -> LocalPoint {
    LocalPoint {
        body: port.body.clone(),
        position_m: port.local_pose.position_m,
    }
}

fn port_vector(port: &ResolvedPort, vector: [f64; 3]) -> Result<[f64; 3], String> {
    let [w, x, y, z] = port
        .local_pose
        .quat_wxyz
        .ok_or("нет разрешённой ориентации порта")?;
    let q = [x, y, z];
    // Поворот единичным кватернионом: v + 2w(q×v) + 2q×(q×v).
    let s = cross(q, vector);
    let t = cross(q, s);
    Ok(std::array::from_fn(|i| vector[i] + 2.0 * (w * s[i] + t[i])))
}

fn geometry(bindings: &FragmentBindings, name: &str) -> String {
    bindings
        .surfaces
        .get(name)
        .map_or_else(|| name.into(), Clone::clone)
}

fn events(name: &str, values: &[InterfaceEvent]) -> Vec<ConfirmedInterfaceEvent> {
    values
        .iter()
        .map(|v| ConfirmedInterfaceEvent {
            name: name.into(),
            at: Time(v.at_ns),
            kind: match v.event {
                InterfaceEventKind::Release => ConfirmedEvent::Release,
                InterfaceEventKind::Break => ConfirmedEvent::Break,
                InterfaceEventKind::Slip { payout_m } => ConfirmedEvent::Slip { payout_m },
            },
        })
        .collect()
}

impl Pad {
    fn resolve(&self, name: &str, bindings: &FragmentBindings) -> Result<PadBinding, String> {
        let mount = |i: usize| -> Result<LocalMount, String> {
            let port = bindings.mechanical_port(&self.mounts[i])?;
            Ok(LocalMount {
                point: point(port),
                normal: port_vector(port, self.normal[i])?,
                tangent: port_vector(port, self.tangent[i])?,
            })
        };
        let mounts = [mount(0)?, mount(1)?];
        let c = &self.calibration;
        let owner = if let Some(law) = &self.law {
            PadReactionOwner::Model(Box::new(PadLaw {
                free_gap_m: sourced(&law.free_gap_m)?,
                compression: law.compression.build()?,
                damping_ns_m: sourced(&law.damping_ns_m)?,
                shear_stiffness_n_m: sourced(&law.shear_stiffness_n_m)?,
                friction: sourced(&law.friction)?,
                capacity_n: sourced(&law.capacity_n)?,
            }))
        } else {
            PadReactionOwner::Engine
        };
        let mut model = CompliantPad::new(PadSpec {
            name: name.into(),
            bodies: mounts.each_ref().map(|m| m.point.body.clone()),
            geometries: self.geometries.each_ref().map(|g| geometry(bindings, g)),
            owner,
            area_m2: self.area_m2.as_ref().map(sourced).transpose()?,
            calibration: PadCalibration {
                compression_m: sourced(&c.compression_m)?,
                offset_m: sourced(&c.offset_m)?,
                axis_error_rad: sourced(&c.axis_error_rad)?,
                relative_speed_m_s: sourced(&c.relative_speed_m_s)?,
            },
        })
        .map_err(|e| e.to_string())?;
        if !self.area_curve.is_empty() {
            let points = self
                .area_curve
                .iter()
                .map(|point| {
                    Ok(PatchAreaPoint {
                        compression_m: point.compression_m,
                        area_m2: sourced(&point.area_m2)?,
                    })
                })
                .collect::<Result<_, String>>()?;
            model = model.with_area_curve(points).map_err(|e| e.to_string())?;
        }
        Ok(PadBinding {
            model,
            mounts,
            events: events(name, &self.events),
        })
    }
}

impl Route {
    fn resolve(
        &self,
        name: &str,
        bindings: &FragmentBindings,
        rt: &Runtime,
    ) -> Result<RouteBinding, String> {
        let ports = bindings.tension_path(&self.mounts)?;
        let c = &self.calibration;
        let model = TensionRoute::new(TensionSpec {
            name: name.into(),
            bodies: ports.iter().map(|p| p.body.clone()).collect(),
            length: match &self.length {
                Length::Fixed { metres } => FreeLengthLaw::Fixed(sourced(metres)?),
                Length::Affine {
                    channel: name,
                    at_zero_m,
                    slope,
                    coordinate,
                } => FreeLengthLaw::Affine {
                    channel: channel(rt, name)?,
                    at_zero_m: sourced(at_zero_m)?,
                    slope: sourced(slope)?,
                    coordinate: sourced(coordinate)?,
                },
            },
            extension: self.extension.build()?,
            damping_ns_m: sourced(&self.damping_ns_m)?,
            friction: self
                .friction
                .as_ref()
                .map(|f| -> Result<_, String> {
                    Ok(TractionFriction {
                        coefficient: sourced(&f.coefficient)?,
                        clamp_normal_n: sourced(&f.clamp_normal_n)?,
                    })
                })
                .transpose()?,
            capacity_n: sourced(&self.capacity_n)?,
            calibration: TensionCalibration {
                extension_m: sourced(&c.extension_m)?,
                extension_speed_m_s: sourced(&c.extension_speed_m_s)?,
                payout_m: sourced(&c.payout_m)?,
            },
        })
        .map_err(|e| e.to_string())?;
        let surfaces = self
            .surfaces
            .iter()
            .map(|(patch, s)| {
                Ok(RouteSurface {
                    patch: patch.clone(),
                    geometry: geometry(bindings, &s.geometry),
                    point: s.point,
                    reference: s.reference,
                    normal: port_vector(ports.get(s.point).ok_or("нет точки пятна")?, s.normal)?,
                    tangent: port_vector(ports[s.point], s.tangent)?,
                    area_m2: s.area_m2.as_ref().map(sourced).transpose()?,
                })
            })
            .collect::<Result<_, String>>()?;
        Ok(RouteBinding {
            model,
            points: ports.into_iter().map(point).collect(),
            surfaces,
            events: events(name, &self.events),
        })
    }
}

impl Inventory {
    fn plan(&self, world: &sim_world::CompiledWorld) -> Result<PowerInventory, String> {
        if self.required.is_empty()
            || !self.required.iter().all(|s| named(s))
            || self.required.iter().ne(self.sources.keys())
            || world
                .drives
                .iter()
                .any(|d| !self.required.contains(&d.name))
        {
            return Err(
                "запись питания должна перечислять каждый источник и установленный привод".into(),
            );
        }
        let sources = self
            .sources
            .iter()
            .map(|(id, source)| {
                let paths = [
                    (EnergyPath::Electrical, &source.electrical),
                    (EnergyPath::Thermal, &source.thermal),
                    (EnergyPath::Mechanical, &source.mechanical),
                ]
                .into_iter()
                .filter_map(|(id, entry)| entry.as_ref().map(|v| (id, v)))
                .map(|(path, e)| {
                    Ok((
                        path,
                        EnergyBounds {
                            stored_j: e.stored_j.as_ref().map(sourced).transpose()?,
                            deliverable_j: e.deliverable_j.as_ref().map(sourced).transpose()?,
                            hazardous_above_j: e
                                .hazardous_above_j
                                .as_ref()
                                .map(sourced)
                                .transpose()?,
                            violation: None,
                        },
                    ))
                })
                .collect::<Result<_, String>>()?;
                Ok((
                    id.clone(),
                    PowerSource {
                        reach: source.reach.into(),
                        paths,
                    },
                ))
            })
            .collect::<Result<_, String>>()?;
        Ok(PowerInventory {
            reach: Reach::Reachable,
            // Профиль заявляет состав. Формат сцены не связывает его с
            // независимой ведомостью питания, проводки и накопителей.
            topology: Some(Provenance::Estimate(format!(
                "заявленный инвентарь без сверки с составом сборки: {:?}",
                self.topology
            ))),
            required_sources: self.required.clone(),
            sources,
        })
    }
}

impl PowerLoss {
    fn plan(&self) -> Result<PowerLossPlan, String> {
        Ok(PowerLossPlan {
            id: self.id.clone(),
            target: self.target.clone(),
            at: Time(self.at_ns),
            loaded_patch: self.loaded_patch.clone(),
            witness: Some(PowerWitness {
                signal: self.signal.clone(),
                unit: self.unit.clone(),
                before: sourced(&self.before)?,
                after: sourced(&self.after)?,
            }),
        })
    }

    fn schedule(&self) -> Result<String, String> {
        let string = |v: &str| serde_json::to_string(v).map_err(|e| e.to_string());
        Ok(format!(
            "[[fault]]\nid = {}\ntarget = {}\nat_ns = {}\nkind = 'power_off'\nsource = {}\n",
            string(&self.id)?,
            string(&self.target)?,
            self.at_ns,
            string("Отказ из задания испытания")?,
        ))
    }
}

fn discrepancies() -> Vec<ReferenceDiscrepancy> {
    sim_human::known_discrepancies()
        .into_iter()
        .map(|check| {
            let tolerance = check.tolerance_text();
            ReferenceDiscrepancy {
                id: check.id.into(),
                mode: check.title,
                simulated: check.simulated,
                reference: check.reference,
                unit: check.unit.into(),
                source: check.reference_method,
                tolerance,
                note: check.note,
            }
        })
        .collect()
}

struct Experiment<'a> {
    case: &'a Case,
    mode: &'a str,
    world_file: &'a Path,
    participant: scene::ParticipantModel,
    world: sim_world::World,
    devices: Vec<DeviceInput>,
    fragments: [ModelFragment; 1],
    materials: ContactMaterialTable,
    wiring: Wiring<'a>,
    fingerprint: Fingerprint,
    discrepancies: &'a [ReferenceDiscrepancy],
    seed: u64,
    sample: Option<LinkedSample<'a>>,
    inventory_requested: bool,
}

struct Grid {
    runtime: Runtime,
    coverage: Coverage,
    assets: Vec<(String, u64)>,
    regions: BTreeMap<String, Vec<String>>,
    patches: BTreeMap<String, PatchPlan>,
    power: PowerInventory,
    inventory: Option<LinkedInventory>,
    resources: BTreeMap<PathBuf, u64>,
    missing_delays: Vec<String>,
}

impl Experiment<'_> {
    fn run_grid(&self, hz: u64, record_view: bool) -> Result<Grid, String> {
        let c = self.case;
        let mut world = self.world.clone();
        world.physics_hz = hz;
        world.seed = self.seed;
        let solver = c
            .contacts
            .solver
            .as_ref()
            .ok_or("контакт требует явных параметров решателя")?
            .into();
        let mut options = ContactBuildOptions {
            physics: sim_signal::Rate::hz(hz),
            solver: Some(solver),
            excluded_bodies: c.contacts.exclusions.clone(),
            ..Default::default()
        };
        let compile = |options: &_| {
            sim_world::compile_with_fragment_bindings(
                self.world_file,
                &world,
                &self.devices,
                &self.fragments,
                options,
            )
        };
        let table_has_entries = !c.contacts.assignments.is_empty() || !c.contacts.pairs.is_empty();
        let (compiled, bindings) = if table_has_entries {
            compile(&ContactBuildOptions {
                solver: None,
                ..options.clone()
            })?
        } else {
            compile(&options)?
        };
        let physical = compiled.contact_geometry_names()?;
        let resolve = |name: &str| -> Result<String, String> {
            let actual = geometry(&bindings, name);
            physical
                .contains(&actual)
                .then_some(actual)
                .ok_or_else(|| format!("неизвестная физическая поверхность «{name}»"))
        };
        for pad in c.pads.values() {
            for g in &pad.geometries {
                resolve(g)?;
            }
        }
        let mut assignments = c
            .contacts
            .assignments
            .iter()
            .map(|(g, material)| {
                Ok(GeometryMaterial {
                    geometry: resolve(g)?,
                    material: material.clone(),
                })
            })
            .collect::<Result<Vec<_>, String>>()?;
        assignments.sort_by(|a, b| a.geometry.cmp(&b.geometry));
        let calibrations = c
            .contacts
            .pairs
            .iter()
            .map(|entry| {
                let mut p: sim_process::contact::PairCalibration = entry.into();
                p.geometries = [resolve(&p.geometries[0])?, resolve(&p.geometries[1])?];
                Ok(p)
            })
            .collect::<Result<Vec<_>, String>>()?;
        let mut pairs = self
            .materials
            .resolve(&compiled, &assignments, &calibrations, solver.cone)
            .map_err(|e| e.to_string())?;
        options.pairs = pairs.iter().map(|entry| entry.pair.clone()).collect();
        let mut resolved = WorldInputs {
            path: self.world_file.to_owned(),
            world,
            devices: self
                .world
                .devices
                .iter()
                .zip(&self.devices)
                .map(|(placing, device)| (placing.name.clone(), device.clone()))
                .collect(),
            contact: options,
        };
        let (compiled, bindings, settings) = if let Some(sample) = self.sample {
            let wires =
                sim_runtime::control::ControlConfig::parse(&self.wiring.text)?.wired_delays()?;
            let settings =
                sample.overlay(&compiled, &mut resolved, Some(&self.wiring.models), &wires)?;
            sample.update_contact_pairs(&mut pairs)?;
            let devices = resolved
                .world
                .devices
                .iter()
                .map(|placing| resolved.devices[&placing.name].clone())
                .collect::<Vec<_>>();
            let (compiled, bindings) = sim_world::compile_with_fragment_bindings(
                self.world_file,
                &resolved.world,
                &devices,
                &self.fragments,
                &resolved.contact,
            )?;
            (compiled, bindings, settings)
        } else if table_has_entries {
            let (compiled, bindings) = sim_world::compile_with_fragment_bindings(
                self.world_file,
                &resolved.world,
                &self.devices,
                &self.fragments,
                &resolved.contact,
            )?;
            (compiled, bindings, ModelSettings::default())
        } else {
            (compiled, bindings, ModelSettings::default())
        };
        bindings.verify_resources()?;
        let power = c.power.plan(&compiled)?;
        let mut rt = Runtime::load(compiled.clone())?;
        if record_view { rt.record_view()?; }
        if !pairs.is_empty() {
            rt.record_process_contacts(pairs)?;
        }
        let regions: BTreeMap<_, _> = bindings
            .surfaces
            .iter()
            .map(|(k, v)| (k.clone(), vec![v.clone()]))
            .collect();
        let pads = c
            .pads
            .iter()
            .map(|(name, pad)| pad.resolve(name, &bindings))
            .collect::<Result<Vec<_>, _>>()?;
        let routes = c
            .routes
            .iter()
            .map(|(name, route)| route.resolve(name, &bindings, &rt))
            .collect::<Result<Vec<_>, _>>()?;
        let mut resources = Vec::new();
        frame(&mut resources, format!("{:?}", self.fingerprint).as_bytes());
        frame(&mut resources, &self.wiring.models.bytes);
        frame(
            &mut resources,
            self.fragments[0]
                .mjcf
                .as_deref()
                .ok_or("нет подготовленного механизма")?
                .as_bytes(),
        );
        for (path, hash) in &bindings.resources {
            frame(&mut resources, path.as_os_str().as_encoded_bytes());
            frame(&mut resources, &hash.to_le_bytes());
        }
        let motion = c.motion.resolve(&c.participant.name, &rt)?;
        let w = &self.wiring;
        let sampled_modes = c
            .sampled_mode
            .iter()
            .map(|mode| (self.mode.to_owned(), mode.clone()))
            .collect();
        let setup = Setup {
            world: &compiled,
            readouts: &w.control.readouts,
            measurements: &w.control.measurements,
            sampled: w.sampled.clone(),
            sampled_modes: &sampled_modes,
            models: Models {
                drives: w.models.drives.clone(),
                sensors: w.models.sensors.clone(),
                bytes: w.models.bytes.clone(),
            },
            task_bytes: resources.clone(),
            world_bytes: compiled.mjcf.as_bytes().to_vec(),
            base: w.base.clone(),
            control: w.text.clone(),
            overlays: BTreeMap::new(),
        };
        let schedule = c
            .power_loss
            .as_ref()
            .map(PowerLoss::schedule)
            .transpose()?
            .unwrap_or_default();
        let sensors = setup.install(&mut rt, self.mode, c.horizon_ns, &schedule)?;
        let control = control::run(&rt).ok_or("контур опыта не установлен")?;
        let assets = control.identity().assets.clone();
        if assets.iter().cloned().collect::<BTreeMap<_, _>>() != w.assets {
            return Err("ресурсы установленного контура изменились".into());
        }
        for (path, hash) in &assets {
            frame(&mut resources, path.as_bytes());
            frame(&mut resources, &hash.to_le_bytes());
        }
        sim_runtime::register_interaction(
            &mut rt,
            ResolvedInteraction {
                name: "взаимодействие участников".into(),
                resources: sim_signal::fnv1a(resources.iter().copied()),
                variant: sim_signal::fnv1a(self.mode.bytes()),
                regions: regions.clone(),
                pads,
                routes,
                motion,
            },
        )?;
        self.install_probes(&mut rt, &bindings, &compiled)?;
        c.processes
            .install(&compiled, &mut rt, &resolved.world, self.world_file)?;
        let installation = self
            .sample
            .map(|sample| sample.install(&mut rt, &compiled, &resolved, &settings))
            .transpose()?;
        let inventory = (self.inventory_requested && hz == self.world.physics_hz).then(|| {
            use sim_task::verdict::ComponentKind;
            let mut extra = Vec::new();
            if let Some(patches) = rt.interaction_patches() {
                extra.extend(
                    patches
                        .keys()
                        .map(|name| (ComponentKind::ParticipantContact, name.clone())),
                );
            }
            extra.extend(
                power
                    .sources
                    .keys()
                    .map(|name| (ComponentKind::EnergyPath, name.clone())),
            );
            extra.extend(setup.sampled.keys().map(|name| (ComponentKind::Battery, name.clone())));
            if power.reach == Reach::Reachable {
                extra.extend(
                    compiled
                        .drives
                        .iter()
                        .map(|drive| (ComponentKind::MotionAtParticipant, drive.name.clone())),
                );
            }
            super::campaign::installation::inventory(
                &compiled,
                &resolved,
                &rt,
                serde_json::json!({
                    "participant": c.participant.name,
                    "patches": rt.interaction_patches().map(|p| p.keys().collect::<Vec<_>>()),
                    "power_sources": power.sources.keys().collect::<Vec<_>>(),
                    "power_required": power.required_sources,
                    "reach": format!("{:?}", power.reach),
                }),
                extra,
            )
        });
        let end = Time(c.horizon_ns);
        let camera_setup = c
            .camera
            .as_ref()
            .map(|audit| audit.prepare(&mut rt, &sensors, end))
            .transpose()?;
        let snapshot = if let Some(at_ns) = c.snapshot_ns {
            // Для наблюдения кадров надо пройти их моменты на основной шкале.
            let initial = if camera_setup.is_some()
                && !control::run(&rt).is_some_and(|run| run.has_external_processes())
            {
                Some(rt.try_snapshot()?)
            } else {
                None
            };
            let reason = rt.audit_snapshot(Time(at_ns), end).err();
            if let Some(initial) = initial {
                rt.restore(&initial)?;
            }
            Some(sim_task::verdict::SnapshotAudit {
                at_ns,
                matched: reason.is_none(),
                reason,
            })
        } else {
            None
        };
        let camera = c
            .camera
            .as_ref()
            .zip(camera_setup)
            .map(|(audit, setup)| audit.run(&mut rt, setup))
            .transpose()?;
        rt.run_until(end)?;
        let missing_delays = settings
            .channel_delay_ns
            .keys()
            .filter(|name| !rt.channel_delay_observed(name))
            .cloned()
            .collect();
        let expected_steps = tick_number(hz, end.0)?;
        let status =
            if rt.status() == Status::Running && rt.time() == end && rt.steps() == expected_steps {
                "completed"
            } else {
                "incomplete"
            };
        bindings.verify_resources()?;
        let patches = rt
            .interaction_patches()
            .ok_or("нет привязок пятен")?
            .keys()
            .map(|id| {
                (
                    id.clone(),
                    PatchPlan {
                        geometry: id.clone(),
                        mode: "движение по заявленному профилю".into(),
                        population: "антропометрическая постановка".into(),
                        repetitions: 1,
                        absence_basis: None,
                    },
                )
            })
            .collect();
        Ok(Grid {
            coverage: Coverage {
                rate_hz: hz as f64,
                expected_steps,
                completed_steps: rt.steps(),
                status,
                binding: Fingerprint::from_fnv1a(bindings.fingerprint),
                control: control::run(&rt).map(|r| Fingerprint::from_fnv1a(r.fingerprint())),
                failure: rt.failure().map(str::to_owned),
                camera,
                snapshot,
                installation,
                motion: rt
                    .process_journal()
                    .iter()
                    .flat_map(|tick| tick.outputs.values())
                    .filter_map(|out| out.interaction.as_ref())
                    .flat_map(|out| &out.motion_events)
                    .map(|event| MotionReceipt {
                        initiator: event.initiator.clone(),
                        contact_ns: event.contact_at.map(|t| t.0),
                        due_ns: event.due.0,
                        applied_ns: [event.interval.start.0, event.interval.end.0],
                        sources: event.sources.iter().map(|s| format!("{s:?}")).collect(),
                    })
                    .collect(),
                control_assets: assets
                    .iter()
                    .map(|(p, h)| (p.clone(), Fingerprint::from_fnv1a(*h)))
                    .collect(),
            },
            runtime: rt,
            assets,
            regions,
            patches,
            power,
            inventory,
            resources: bindings.resources,
            missing_delays,
        })
    }

    fn run(&self, options: &Options) -> Result<CaseReport<'_>, String> {
        let c = self.case;
        let hz = self.world.physics_hz;
        tick_number(hz, c.horizon_ns)?;
        tick_number(hz, c.audit_from_ns)?;
        if let Some(at) = c.snapshot_ns {
            tick_number(hz, at)?;
        }
        for e in c
            .pads
            .values()
            .flat_map(|p| &p.events)
            .chain(c.routes.values().flat_map(|r| &r.events))
        {
            if e.at_ns >= c.horizon_ns {
                return Err("событие лежит вне опыта".into());
            }
            tick_number(hz, e.at_ns)?;
        }
        if let Some(p) = &c.power_loss {
            tick_number(hz, p.at_ns)?;
        }
        let faster = |factor| hz.checked_mul(factor).ok_or("переполнение частоты");
        let capture = options.campaign_capture.as_ref();
        let record = |rate| capture.is_some_and(|c| c.records_view() && c.grid(None, rate));
        let [base, middle, fine] = [
            self.run_grid(hz, record(hz))?,
            self.run_grid(faster(2)?, record(faster(2)?))?,
            self.run_grid(faster(4)?, options.view.is_some() || record(faster(4)?))?,
        ];
        if base.assets != middle.assets || base.assets != fine.assets {
            return Err("ресурсы контура изменились между сетками".into());
        }
        let plan_for = |id: &str| {
            base.patches
                .get(id)
                .ok_or_else(|| format!("нет пятна критерия «{id}»"))
        };
        let limits = c
            .limits
            .iter()
            .map(|l| {
                let patch = plan_for(&l.patch)?;
                if !base.regions.contains_key(&l.region) {
                    return Err("неизвестная область критерия".into());
                }
                Ok(LimitCriterion {
                    patch: l.patch.clone(),
                    region: l.region.clone(),
                    metric: l.metric.into(),
                    kind: match l.kind {
                        LimitKind::Design => CriterionKind::Design,
                        LimitKind::TissueScreen => CriterionKind::TissueScreen,
                    },
                    limit: sourced(&l.limit)?,
                    validation: Some(l.validation.clone()),
                    geometry: patch.geometry.clone(),
                    mode: patch.mode.clone(),
                    population: patch.population.clone(),
                    duration_s: sourced(&l.duration_s)?,
                    max_repetitions: 1,
                    area_m2: l.area_m2.as_ref().map(sourced).transpose()?,
                })
            })
            .collect::<Result<Vec<_>, String>>()?;
        let runs = [&base.runtime, &middle.runtime, &fine.runtime];
        if let Some(capture) = capture {
            let runtime = runs.iter().find(|rt| capture.grid(None, rt.physics_rate().as_hz() as u64))
                .ok_or("в опыте взаимодействия отсутствует сетка повтора")?;
            capture.save(runtime, None)?;
        }
        let ids = runs.each_ref().map(|rt| {
            rt.interaction_identity()
                .ok_or("нет идентичности взаимодействия")
        });
        let [a, b, d] = ids;
        let t = &c.tolerances;
        let checks = [&base, &middle, &fine]
            .into_iter()
            .flat_map(|grid| c.checks.iter().map(move |check| check.audit(grid, c)))
            .collect::<Vec<_>>();
        let plan = InteractionPlan {
            build: self.fingerprint,
            legacy_design: None,
            construction_complete: c.construction_complete,
            grids: [a?, b?, d?],
            variant: self.mode.into(),
            window: Interval {
                start: Time(c.audit_from_ns),
                end: Time(c.horizon_ns),
            },
            regions: base.regions.keys().cloned().collect(),
            patches: base.patches,
            limits,
            tolerances: ContactTolerances {
                onset_s: sourced(&t.onset_s)?,
                force_n: sourced(&t.force_n)?,
                impulse_ns: sourced(&t.impulse_ns)?,
                duration_s: sourced(&t.duration_s)?,
                drift_m: sourced(&t.drift_m)?,
                stress_pa: sourced(&t.stress_pa)?,
            },
            power: base.power,
            power_loss: c.power_loss.as_ref().map(PowerLoss::plan).transpose()?,
            discrepancies: self.discrepancies.to_vec(),
        };
        let evidence = assess_interaction_grids(runs, &plan).map_err(|e| e.to_string())?;
        let convergence = numeric_convergence(&evidence, &c.tolerances);
        let tissue = self.audit_tissue(&fine.runtime, &evidence, &convergence);
        let mut safety_gaps = Vec::new();
        let anthropometry = &self.participant.anthropometry;
        let marginals = scene::marginals(anthropometry.population);
        let stature = sourced(&c.participant.stature_m)?;
        let mass = sourced(&c.participant.mass_kg)?;
        if stature.min < marginals.stature_m[0]
            || stature.max > marginals.stature_m[2]
            || mass.min < marginals.mass_kg[0]
            || mass.max > marginals.mass_kg[2]
        {
            safety_gaps.push(format!(
                "антропометрия вне диапазона выборки: рост {}–{} м, масса {}–{} кг",
                stature.min, stature.max, mass.min, mass.max
            ));
        }
        if stature.min != stature.max || mass.min != mass.max {
            safety_gaps.push(
                "физика рассчитана для номинала, полный разброс роста и массы не покрыт".into(),
            );
        }
        let coverage = vec![base.coverage, middle.coverage, fine.coverage];
        for (grid, delays) in coverage.iter().zip([
            base.missing_delays,
            middle.missing_delays,
            fine.missing_delays,
        ]) {
            for channel in delays {
                safety_gaps.push(format!(
                    "на сетке {} Гц нет доставки с задержкой «{channel}»",
                    grid.rate_hz
                ));
            }
        }
        safety_gaps.extend(coverage.iter().filter_map(|grid| {
            let reason = grid.snapshot.as_ref()?.reason.as_ref()?;
            Some(format!("снимок на сетке {} Гц: {reason}", grid.rate_hz))
        }));
        let incomplete = coverage.iter().any(|g| {
            g.status != "completed"
                || g.camera
                    .as_ref()
                    .is_some_and(|c| c.frames.len() != c.expected)
        });
        let numerical = Outcome::aggregate(
            checks
                .iter()
                .filter(|check| !check.required_design)
                .map(|check| check.outcome)
                .chain(incomplete.then_some(Outcome::NotAnalyzed)),
        );
        let outcome = Outcome::aggregate(
            std::iter::once(evidence.outcome())
                .chain(tissue.iter().map(|finding| finding.outcome))
                .chain(
                    checks
                        .iter()
                        .filter(|check| check.required_design)
                        .map(|check| check.outcome),
                )
                .chain((!safety_gaps.is_empty()).then_some(Outcome::NotAnalyzed))
                .chain((numerical != Outcome::Pass).then_some(Outcome::NotAnalyzed)),
        );
        let readiness = safety_decision([outcome], c.construction_complete);
        let checkpointable = coverage
            .iter()
            .all(|g| g.snapshot.as_ref().is_none_or(|audit| audit.matched))
            && runs
                .iter()
                .all(|rt| !control::run(rt).is_some_and(|run| run.has_external_processes()));
        crate::view::save(options, &fine.runtime, Some(self.mode), None)?;
        if let Some(path) = &options.log {
            export(path, |out| fine.runtime.log().write_binary(out))?;
        }
        if let Some(path) = &options.csv {
            export(path, |out| fine.runtime.log().write_csv(out))?;
        }
        Ok(CaseReport {
            id: self.mode,
            source: &c.source,
            outcome,
            numerical,
            readiness,
            seed: self.seed,
            coverage,
            horizon_ns: self.case.horizon_ns,
            checks,
            evidence,
            convergence,
            tissue,
            safety_gaps,
            checkpointable,
            inventory: base.inventory,
            resources: base.resources,
        })
    }
}

/// Номер физической границы для целой частоты. Учитывает округление такта вниз.
fn tick_number(hz: u64, ns: u64) -> Result<u64, String> {
    // Длительность секунды в единицах времени sim-signal.
    let second = 1_000_000_000_u128;
    let k = (u128::from(ns) * u128::from(hz)).div_ceil(second);
    if hz == 0 || k * second / u128::from(hz) != u128::from(ns) {
        return Err(format!(
            "{ns} нс не лежит на исходной физической сетке {hz} Гц"
        ));
    }
    u64::try_from(k).map_err(|_| "переполнение номера такта".into())
}

pub(super) fn run(
    task: TaskFile,
    path: &Path,
    options: &Options,
    out: &mut impl Write,
) -> Result<u8, String> {
    run_observed(task, path, options, out, None, None)
}

pub(super) fn run_observed(
    task: TaskFile,
    path: &Path,
    options: &Options,
    out: &mut impl Write,
    mut audit: Option<&mut super::campaign::LinkedAudit>,
    sample: Option<LinkedSample<'_>>,
) -> Result<u8, String> {
    if options.matrix_full || options.jobs.is_some() {
        return Err(
            "для взаимодействия порядок вариантов задаётся профилем, число исполнителей — кампанией".into(),
        );
    }
    let mut inputs = Inputs::default();
    inputs.capture(path)?;
    let profile_path = dir(path).join(task.interaction_profile);
    let profile: Profile = inputs.toml(&profile_path)?;
    let names: BTreeSet<_> = profile.cases.iter().map(|v| v.id.as_str()).collect();
    if profile.version != 1
        || !named(&profile.source)
        || names.is_empty()
        || names.len() != profile.cases.len()
        || names.iter().any(|name| !named(name))
    {
        return Err("невалидная матрица взаимодействия".into());
    }
    if let Some(name) = options.case.as_ref().filter(|name| !names.contains(name.as_str())) {
        let reason = format!("вариант «{name}» отсутствует в профиле взаимодействия");
        if let Some(audit) = audit.as_mut().filter(|_| options.linked_grid_hz.is_some()) {
            audit.record_unavailable(reason);
            return Ok(0);
        }
        return Err(reason);
    }
    if options.exports_run() && options.case.is_none() && names.len() > 1
    {
        return Err("выгрузка журнала требует одного варианта через --case".into());
    }
    let case_path = dir(&profile_path).join(profile.base);
    let base: toml::Table = inputs.toml(&case_path)?;
    let mut world = capture_world(&options.world, &mut inputs)?;
    if let Some(hz) = options.linked_grid_hz {
        world.physics_hz = hz;
    }
    let devices = world
        .devices
        .iter()
        .map(|d| {
            let path = dir(&options.world).join(&d.file);
            let device = sim_world::load_device(&path, &sim_world::LegacyCatalog)?;
            Ok(DeviceInput { device, path })
        })
        .collect::<Result<Vec<_>, String>>()?;
    let seed = sample.map_or_else(
        || options.seed.unwrap_or(world.seed),
        |s| s.sample.seeds.run,
    );
    let selected = profile
        .cases
        .iter()
        .filter(|v| options.case.as_ref().is_none_or(|name| name == &v.id));
    let mut effective_cases = Vec::new();
    for v in selected {
        let mut table = base.clone();
        let mut files = inputs.clone();
        if let Some(file) = &v.overlay {
            merge_tables(&mut table, files.toml(&dir(&profile_path).join(file))?);
        }
        let case: Case = toml::Value::Table(table)
            .try_into()
            .map_err(|e| format!("{}: {e}", case_path.display()))?;
        case.validate()?;
        effective_cases.push((v, case, files));
    }
    if audit
        .as_mut()
        .is_some_and(|audit| audit.omit_unselected(effective_cases.len()))
    {
        inputs.verify()?;
        return Ok(0);
    }
    let mut experiments = Vec::new();
    let discrepancies = discrepancies();
    let empty_control: Control = toml::from_str("wiring = ''").map_err(|e| e.to_string())?;
    for (variant, case, files) in &mut effective_cases {
        let (model, fragment) = case.participant.build(&case.motion)?;
        for resource in &model.resources {
            files.capture(resource)?;
        }
        let table: PairTable = files.toml(&dir(&case_path).join(&case.contacts.table))?;
        if table.version != 1 {
            return Err("неподдерживаемая версия таблицы контактов".into());
        }
        let materials = ContactMaterialTable::new(
            table
                .entries
                .into_iter()
                .map(PairRow::parameters)
                .collect::<Result<Vec<_>, _>>()?,
        )
        .map_err(|e| e.to_string())?;
        let wiring = if let Some(control) = &case.control {
            Wiring::load(control, dir(&case_path), files, options)?
        } else {
            Wiring {
                control: &empty_control,
                base: dir(&case_path).into(),
                text: "version = 1".into(),
                sampled: BTreeMap::new(),
                models: Models::load(&crate::catalog_path(options)?)?,
                assets: BTreeMap::new(),
            }
        };
        let fingerprint =
            Fingerprint::of(format!("{:?}:{:?}", files.fingerprints(), variant.id).as_bytes());
        let mut variant_world = world.clone();
        for (name, pose) in &case.placement_overrides {
            variant_world
                .devices
                .iter_mut()
                .find(|d| d.name == *name)
                .ok_or_else(|| format!("неизвестное размещение «{name}»"))?
                .pose = pose.clone();
        }
        let experiment = Experiment {
            case,
            mode: &variant.id,
            world_file: &options.world,
            participant: model,
            world: variant_world,
            devices: devices.clone(),
            fragments: [fragment],
            materials,
            wiring,
            fingerprint,
            discrepancies: &discrepancies,
            seed,
            sample,
            inventory_requested: audit.is_some(),
        };
        experiments.push((experiment, &*files));
    }
    let mut reports = Vec::new();
    let mut fingerprints = BTreeMap::new();
    let local = super::workers::Workers::new(std::num::NonZeroUsize::MIN);
    let workers = options.workers.as_deref().unwrap_or(&local);
    let finished = workers.map(experiments.iter().collect(), None, |(experiment, files)| {
        Ok((experiment, files, experiment.run(options)?))
    })?;
    for (experiment, files, report) in finished {
        let mut files = (*files).clone();
        if let Some(audit) = audit.as_mut() {
            for (path, hash) in &report.resources {
                if sim_signal::fnv1a(files.capture(path)?.iter().copied()) != *hash {
                    return Err(format!("изменился ресурс участника {}", path.display()));
                }
            }
            audit.record_files(&files)?;
            audit.record_catalog(Fingerprint::of(&experiment.wiring.models.bytes))?;
            if !report.checkpointable {
                audit.disable_checkpoint();
            }
            if let Some(inventory) = &report.inventory {
                audit.inventory(report.id.to_owned(), inventory.clone())?;
            }
        }
        files.verify()?;
        for (path, stamp) in files.fingerprints() {
            if fingerprints
                .insert(path, stamp)
                .is_some_and(|old| old != stamp)
            {
                return Err("файл входных данных изменился между вариантами".into());
            }
        }
        reports.push(report);
    }
    // Равные варианты оставляем в заданном порядке.
    let worst = reports
        .iter()
        .reduce(|a, b| {
            if (b.outcome, b.numerical) > (a.outcome, a.numerical) {
                b
            } else {
                a
            }
        })
        .ok_or("не выбран ни один вариант")?;
    let outcome = worst.outcome;
    let numerical = Outcome::aggregate(reports.iter().map(|r| r.numerical));
    let status = if outcome == Outcome::Fail || numerical == Outcome::Fail {
        "failed"
    } else if outcome != Outcome::Pass || numerical != Outcome::Pass {
        "incomplete"
    } else {
        "completed"
    };
    let report = Report {
        status,
        outcome,
        numerical,
        worst_variant: worst.id,
        seed,
        inputs: fingerprints,
        cases: reports,
    };
    if let Some(audit) = audit.as_mut() {
        audit.record_inputs((&report.inputs, report.seed))?;
        for case in &report.cases {
            audit.record_inputs((
                case.id,
                case.coverage
                    .iter()
                    .map(|g| (g.binding, &g.control_assets))
                    .collect::<Vec<_>>(),
            ))?;
            let key = audit.child(case.id, case.id, Vec::new());
            let mut findings = case.evidence.campaign_findings(&key);
            findings.extend(case.checks.iter()
                .filter(|check| check.outcome == Outcome::Fail)
                .map(|check| {
                    let journal = case.evidence.grids().iter().find(|grid| grid.rate_hz == check.rate_hz)
                        .map(|grid| grid.journal);
                    super::campaign::check_violation(
                        &key, check.id, check.value.map(|value| [value; 2]),
                        Some((check.bounds, check.source)), check.window_ns,
                        check.event_ns.zip(journal),
                    )
                }));
            audit.observe(
                key,
                case.outcome,
                case.coverage
                    .iter()
                    .all(|g| g.expected_steps > 0 && g.expected_steps == g.completed_steps),
                super::campaign::ExecutionStamp {
                    seed: case.seed,
                    hz: case.coverage[0].rate_hz as u64,
                    reached_ns: case.coverage.iter().all(|g| g.expected_steps == g.completed_steps)
                        .then_some(case.horizon_ns),
                    requested_ns: None,
                    applied_ns: None,
                    fault_delay_ns: None,
                },
                findings,
            )?;
            audit.consider_playback(case.evidence.grids().iter().map(|grid| {
                playback::Grid {
                    variant: None,
                    hz: grid.rate_hz as u64,
                    journal: grid.journal.as_u64(),
                    end_ns: case.horizon_ns,
                }
            }));
        }
    }
    if options.json {
        serde_json::to_writer(&mut *out, &report).map_err(|e| e.to_string())?;
        writeln!(out).map_err(|e| e.to_string())?;
    } else {
        for case in &report.cases {
            writeln!(
                out,
                "{} — {:?}, готовность {:?} (численная сверка {:?})",
                case.id, case.outcome, case.readiness, case.numerical
            )
            .map_err(|e| e.to_string())?;
        }
    }
    Ok(u8::from(status != "completed"))
}

impl Case {
    fn validate(&self) -> Result<(), String> {
        let mut ids = BTreeSet::new();
        if !named(&self.source)
            || self.audit_from_ns >= self.horizon_ns
            || self
                .snapshot_ns
                .is_some_and(|at| at == 0 || at >= self.horizon_ns)
            || self.checks.is_empty()
            || self.pads.is_empty() && self.routes.is_empty()
            || self.checks.iter().any(|c| {
                !named(&c.id)
                    || !ids.insert(&c.id)
                    || !named(&c.source)
                    || c.min_samples == 0
                    || c.bounds[0] > c.bounds[1]
                    || c.window_ns
                        .is_some_and(|[a, b]| a >= b || b > self.horizon_ns)
            })
        {
            return Err(
                "нужны непустой опыт, конечное окно и уникальные проверки с источниками".into(),
            );
        }
        Ok(())
    }
}

fn norm(v: [f64; 3]) -> f64 {
    v[0].hypot(v[1]).hypot(v[2])
}

fn statistic(values: &[(Time, f64)], stat: &Stat) -> Option<(f64, Option<Time>)> {
    let first = values.first()?.1;
    let series = || values.iter().map(|&(_, value)| value);
    let min = || series().fold(first, f64::min);
    let max = || series().fold(first, f64::max);
    let result = match stat {
        Stat::Peak => {
            let value = series().map(f64::abs).fold(0.0, f64::max);
            (value, values.iter().find(|(_, v)| v.abs() == value).map(|(at, _)| *at))
        }
        Stat::Min => {
            let value = min();
            (value, values.iter().find(|(_, v)| *v == value).map(|(at, _)| *at))
        }
        Stat::Last => {
            let &(at, value) = values.last()?;
            (value, Some(at))
        }
        Stat::Span => (max() - min(), None),
        Stat::Sum => (series().sum(), None),
    };
    result.0.is_finite().then_some(result)
}

impl Check {
    fn audit(&self, grid: &Grid, c: &Case) -> CheckReport<'_> {
        let rt = &grid.runtime;
        let [start, end] = self.window_ns.unwrap_or([c.audit_from_ns, c.horizon_ns]);
        let inside = |t: Time| (start..=end).contains(&t.0);
        let interaction = rt
            .process_journal()
            .iter()
            .filter(|tick| inside(tick.at))
            .flat_map(|tick| {
                tick.outputs
                    .values()
                    .filter_map(|out| out.interaction.as_ref().map(|out| (tick.at, out)))
            });
        let reactions = |name: &str| {
            interaction
                .clone()
                .filter_map(move |(at, tick)| {
                    tick.patches
                        .get(name)
                        .and_then(|sample| match &sample.reaction {
                            Analysis::Analyzed { value, .. } => Some((at, value, sample)),
                            _ => None,
                        })
                })
                .collect::<Vec<_>>()
        };
        let steps = |id: &str| {
            interaction
                .clone()
                .filter_map(|(at, tick)| tick.interfaces.get(id).map(|step| (at, step)))
                .collect::<Vec<_>>()
        };
        let control = control::run(rt);
        let peak = Stat::Peak;
        let mut stat = &peak;
        let mut values: Vec<(Time, f64)> = match &self.metric {
            Measure::Normal { patch } => reactions(patch)
                .into_iter()
                .map(|(at, v, _)| (at, v.normal_n))
                .collect(),
            Measure::Shear { patch } => reactions(patch)
                .into_iter()
                .map(|(at, v, _)| (at, norm(v.shear_n)))
                .collect(),
            Measure::OpposedNormal { first, second } => {
                // Одновременный прижим ограничивается меньшей из встречных сил.
                interaction
                    .clone()
                    .filter_map(|(at, tick)| {
                        let reaction = |id| match &tick.patches.get(id)?.reaction {
                            Analysis::Analyzed { value, .. } => Some(value.normal_n),
                            _ => None,
                        };
                        Some((at, reaction(first)?.min(reaction(second)?)))
                    })
                    .collect()
            }
            Measure::Jerk { channel, sample_ns } => {
                let id = rt.channel(channel);
                if *sample_ns == 0 {
                    Vec::new()
                } else {
                    let dt = Time(*sample_ns).as_secs();
                    let rows: Vec<_> = interaction
                        .clone()
                        .flat_map(|(_, tick)| &tick.axes)
                        .filter(|axis| {
                            Some(axis.channel) == id && axis.interval.start.0 % sample_ns == 0
                        })
                        .map(|a| (a.interval.start.0, a.velocity))
                        .collect();
                    rows.windows(3)
                        .filter(|v| v[2].0 - v[1].0 == *sample_ns && v[1].0 - v[0].0 == *sample_ns)
                        .map(|v| (Time(v[2].0), (v[2].1 - 2.0 * v[1].1 + v[0].1) / dt.powi(2)))
                        .collect()
                }
            }
            Measure::Pressure { patch } => reactions(patch)
                .into_iter()
                .filter_map(|(at, v, s)| Some((at, v.normal_n / s.area_m2.as_ref()?.min)))
                .collect(),
            // Сверка проектного бюджета оснастки по её номинальной площади.
            Measure::NominalPressure { patch } => c
                .pads
                .get(patch)
                .and_then(|pad| pad.area_m2.as_ref())
                .and_then(|area| sourced(area).ok())
                .into_iter()
                .flat_map(|area| {
                    reactions(patch)
                        .into_iter()
                        .map(move |(at, v, _)| (at, v.normal_n / area.min))
                })
                .collect(),
            Measure::Tension { route } => interaction
                .clone()
                .filter_map(|(at, tick)| match &tick.completed_routes.get(route)?.reaction {
                    Analysis::Analyzed { value, .. } => Some((at, value.tension_n)),
                    _ => None,
                })
                .collect(),
            Measure::RoutePayout { route } => interaction
                .clone()
                .filter_map(|(at, tick)| Some((at, tick.completed_routes.get(route)?.payout_m?)))
                .collect(),
            Measure::InterfaceState { interface, state } => steps(interface)
                .into_iter()
                .map(|(at, s)| {
                    let expected = match state {
                        State::Open => InterfaceState::Open,
                        State::Loaded => InterfaceState::Loaded,
                        State::Sliding => InterfaceState::Sliding,
                        State::Released => InterfaceState::Released,
                        State::Broken => InterfaceState::Broken,
                    };
                    (at, f64::from(s.state == expected))
                })
                .collect(),
            Measure::ActionReactionResidual { interface } => steps(interface)
                .into_iter()
                .filter_map(|(at, s)| {
                    if s.body_loads.is_empty() {
                        return None;
                    }
                    let mut total = [0.0; 3];
                    for load in &s.body_loads {
                        for (a, b) in total.iter_mut().zip(load.force_n) {
                            *a += b;
                        }
                    }
                    Some((at, norm(total)))
                })
                .collect(),
            Measure::VirtualWorkResidual { route } => steps(route)
                .into_iter()
                .filter_map(|(at, s)| {
                    let Length::Affine { slope, .. } = &c.routes.get(route)?.length else {
                        return None;
                    };
                    let Analysis::Analyzed { value, .. } = &s.reaction else {
                        return None;
                    };
                    // Независимая производная e(q)=L−(L₀+s·q) по коэффициенту из данных.
                    Some((at, (s.drive_effort?.1 - value.tension_n * slope.value).abs()))
                })
                .collect(),
            Measure::LengthResidual { route } => steps(route)
                .into_iter()
                .filter_map(|(at, s)| {
                    Some((at, (s.path_length_m? - s.free_length_m? - s.deformation_m?).abs()))
                })
                .collect(),
            Measure::PadLawResidual { patch } => steps(patch)
                .into_iter()
                .filter_map(|(at, s)| {
                    let law = c.pads.get(patch)?.law.as_ref()?;
                    if law.damping_ns_m.value != 0.0 || s.state.is_terminal() {
                        return None;
                    }
                    let x = s.deformation_m?;
                    let [a, b] = law
                        .compression
                        .points
                        .windows(2)
                        .find(|w| w[0][0] <= x && x <= w[1][0])?
                    else {
                        return None;
                    };
                    let expected = a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0]);
                    let Analysis::Analyzed { value, .. } = &s.reaction else {
                        return None;
                    };
                    Some((at, (expected - value.normal_n).abs()))
                })
                .collect(),
            Measure::Axis {
                channel,
                statistic,
                quantity,
            } => {
                stat = statistic;
                rt.channel(channel)
                    .into_iter()
                    .flat_map(|id| {
                        interaction.clone().flat_map(move |(at, tick)| {
                            tick.axes.iter().filter(move |a| a.channel == id).map(
                                move |a| (at, match quantity {
                                    AxisQuantity::Position => a.position,
                                    AxisQuantity::Velocity => a.velocity,
                                    AxisQuantity::Effort => a.applied_effort,
                                }),
                            )
                        })
                    })
                    .collect()
            }
            Measure::Signal {
                name,
                unit,
                statistic,
                observation,
            } => {
                stat = statistic;
                let header = rt.log().header();
                let channels = if *observation {
                    &header.observations
                } else {
                    &header.truth
                };
                channels
                    .iter()
                    .position(|s| s.name == *name && s.unit == *unit)
                    .into_iter()
                    .flat_map(|id| {
                        rt.log()
                            .entries()
                            .iter()
                            .filter(move |entry| {
                                entry.id as usize == id
                                    && entry.valid
                                    && inside(entry.at)
                                    && entry.kind
                                        == if *observation {
                                            LogKind::Observation
                                        } else {
                                            LogKind::Truth
                                        }
                            })
                            .map(|entry| (entry.at, entry.value))
                    })
                    .collect()
            }
            Measure::Packet {
                route,
                quantity,
                statistic,
            } => {
                stat = statistic;
                control
                    .iter()
                    .flat_map(|run| run.events())
                    .filter(|event| inside(event.at))
                    .filter_map(|record| {
                        let value = match (&record.kind, quantity) {
                        (
                            control::EventKind::PacketDelivered {
                                route: r,
                                sent_at,
                                id,
                                ..
                            },
                            q,
                        ) if r == route => match q {
                            PacketQuantity::Delivered => Some(1.0),
                            PacketQuantity::LatencyNs => {
                                record.at.0.checked_sub(sent_at.0).map(|ns| ns as f64)
                            }
                            PacketQuantity::Sequence => Some(*id as f64),
                            PacketQuantity::Lost => None,
                        },
                        (control::EventKind::Packet { route: r, event }, PacketQuantity::Lost)
                            if r == route && event.outcome == control::PacketOutcome::Lost =>
                        {
                            Some(1.0)
                        }
                            _ => None,
                        }?;
                        Some((record.at, value))
                    })
                    .collect()
            }
            Measure::MotionEvents => {
                stat = &Stat::Sum;
                interaction
                    .clone()
                    .flat_map(|(at, tick)| tick.motion_events.iter().map(move |_| (at, 1.0)))
                    .collect()
            }
            Measure::Probe {
                name,
                unit,
                statistic,
            } => {
                stat = statistic;
                rt.process_journal()
                    .iter()
                    .filter(|t| inside(t.at))
                    .flat_map(|t| t.outputs.values())
                    .flat_map(|o| &o.measurements)
                    .filter_map(|measurement| match &measurement.result {
                        Analysis::Analyzed { value, .. }
                            if measurement.signal.name == *name
                                && measurement.signal.unit == *unit =>
                        {
                            Some((measurement.at, *value))
                        }
                        _ => None,
                    })
                    .collect()
            }
            Measure::CameraProjection => grid
                .coverage
                .camera
                .iter()
                .flat_map(|camera| &camera.frames)
                .filter(|f| inside(Time(f.delivered_ns)))
                .map(|f| (Time(f.delivered_ns), f.projection_error_px))
                .collect(),
            Measure::CameraShift => {
                stat = &Stat::Span;
                grid.coverage
                    .camera
                    .iter()
                    .flat_map(|camera| &camera.frames)
                    .filter(|f| inside(Time(f.delivered_ns)))
                    .map(|f| (Time(f.delivered_ns), f.centroid_px[0]))
                    .collect()
            }
            Measure::PowerCut => {
                stat = &Stat::Sum;
                control
                    .iter()
                    .flat_map(|run| run.events())
                    .filter(|event| inside(event.at))
                    .filter_map(|event| match &event.kind {
                        control::EventKind::FaultApplied {
                            id,
                            target,
                            active: true,
                        } if c
                            .power_loss
                            .as_ref()
                            .is_some_and(|p| p.id == *id && p.target == *target) =>
                        {
                            Some((event.at, 1.0))
                        }
                        _ => None,
                    })
                    .collect()
            }
        };
        if values.iter().any(|(_, v)| !v.is_finite()) {
            values.clear();
        }
        let result = statistic(&values, stat);
        let outcome = match result {
            Some((v, _)) if values.len() >= self.min_samples => {
                if (self.bounds[0]..=self.bounds[1]).contains(&v) {
                    Outcome::Pass
                } else {
                    Outcome::Fail
                }
            }
            _ => Outcome::NotAnalyzed,
        };
        CheckReport {
            id: &self.id,
            rate_hz: rt.physics_rate().as_hz(),
            samples: values.len(),
            value: result.map(|(value, _)| value),
            event_ns: result.and_then(|(_, at)| at.map(|at| at.0)),
            window_ns: [start, end],
            bounds: self.bounds,
            source: &self.source,
            outcome,
            required_design: self.required_design,
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Probe {
    Joint {
        joint: String,
        dof: usize,
        unit: String,
    },
    RouteLength {
        route: String,
    },
    PointSpeed {
        port: String,
    },
    Support {
        port: String,
        geometries: Vec<String>,
        quantity: SupportQuantity,
    },
    GyroResidual {
        port: String,
        sensor: String,
        axis: usize,
        max_age_ns: u64,
    },
}

#[derive(Clone, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum SupportQuantity {
    Vertical,
    StaticMoment,
}

#[derive(Clone, Debug)]
enum ProbeInput {
    Joint {
        joint: String,
        dof: usize,
    },
    RouteLength {
        points: Vec<LocalPoint>,
    },
    PointSpeed {
        point: LocalPoint,
    },
    Support {
        point: LocalPoint,
        geometries: BTreeSet<String>,
        gravity_force: [f64; 3],
        quantity: SupportQuantity,
    },
    GyroResidual {
        point: LocalPoint,
        axis: [f64; 3],
        signal: sim_signal::SignalId,
        max_age_ns: u64,
    },
}

#[derive(Clone, Debug)]
struct ProbeModel {
    name: String,
    unit: String,
    input: ProbeInput,
}

impl sim_runtime::process::ProcessModel for ProbeModel {
    fn update(
        &mut self,
        frame: &sim_runtime::process::ProcessFrame<'_>,
        _: &mut sim_signal::Rng,
    ) -> Result<sim_runtime::process::ProcessOut, sim_process::ProcessError> {
        let mut out = sim_runtime::process::ProcessOut::default();
        let at = frame.next.start;
        let value = match &self.input {
            ProbeInput::Joint { joint, dof } => frame
                .truth
                .joint(joint)
                .and_then(|(p, _)| p.get(*dof))
                .copied(),
            ProbeInput::RouteLength { points } => {
                let points = points
                    .iter()
                    .map(|p| frame.body_point(&p.body, p.position_m))
                    .collect::<Result<Vec<_>, _>>()?;
                Some(
                    points
                        .windows(2)
                        .map(|p| {
                            norm(std::array::from_fn(|i| {
                                p[1].position_m[i] - p[0].position_m[i]
                            }))
                        })
                        .sum(),
                )
            }
            ProbeInput::PointSpeed { point } => Some(norm(
                frame
                    .body_point(&point.body, point.position_m)?
                    .velocity_m_s,
            )),
            ProbeInput::Support {
                point,
                geometries,
                gravity_force,
                quantity,
            } => {
                let com = frame.body_point(&point.body, point.position_m)?.com_m;
                frame.completed.map(|f| {
                    let mut force = [0.0; 3];
                    let mut moment = [0.0; 3];
                    for c in &f.reactions.contacts {
                        let sign = match c.geometries.each_ref().map(|g| geometries.contains(g)) {
                            [true, false] => -1.0,
                            [false, true] => 1.0,
                            _ => continue,
                        };
                        let lever = std::array::from_fn(|i| c.point_m[i] - com[i]);
                        let torque = cross(lever, c.force_n);
                        for i in 0..3 {
                            force[i] += sign * c.force_n[i];
                            moment[i] += sign * (torque[i] + c.torque_nm[i]);
                        }
                    }
                    match quantity {
                        SupportQuantity::Vertical => force[2] + gravity_force[2],
                        SupportQuantity::StaticMoment => norm(moment),
                    }
                })
            }
            ProbeInput::GyroResidual {
                point,
                axis,
                signal,
                max_age_ns,
            } => {
                let pose = frame.body_point(&point.body, point.position_m)?;
                let axis_world = pose.rotation.map(|row| dot(row, *axis));
                frame
                    .observation(*signal)
                    .filter(|s| s.valid && at.0.saturating_sub(s.at.0) <= *max_age_ns)
                    .map(|s| s.value - dot(axis_world, pose.angular_velocity_rad_s))
            }
        };
        if let Some(value) = value {
            out.measure(
                at,
                &self.name,
                &self.unit,
                Analysis::Analyzed {
                    value,
                    scope: "кинематика по позам и якобиану движка".into(),
                },
            )?;
        }
        Ok(out)
    }

    fn state_fingerprint(&self) -> u64 {
        sim_signal::fnv1a(format!("{self:?}").bytes())
    }
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[(i + 1) % 3] * b[(i + 2) % 3] - a[(i + 2) % 3] * b[(i + 1) % 3])
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a.into_iter().zip(b).map(|(a, b)| a * b).sum()
}

impl Experiment<'_> {
    fn install_probes(
        &self,
        rt: &mut Runtime,
        bindings: &FragmentBindings,
        compiled: &sim_world::CompiledWorld,
    ) -> Result<(), String> {
        for (id, probe) in &self.case.probes {
            let (input, unit) = match probe {
                Probe::Joint { joint, dof, unit } => {
                    if !compiled.joints.iter().any(|j| j.name == *joint) {
                        return Err(format!("неизвестная ось «{joint}»"));
                    }
                    (
                        ProbeInput::Joint {
                            joint: joint.clone(),
                            dof: *dof,
                        },
                        unit.as_str(),
                    )
                }
                Probe::RouteLength { route } => {
                    let path = &self
                        .case
                        .routes
                        .get(route)
                        .ok_or("нет измеряемой трассы")?
                        .mounts;
                    let points = bindings
                        .tension_path(path)?
                        .into_iter()
                        .map(point)
                        .collect();
                    (ProbeInput::RouteLength { points }, "m")
                }
                Probe::PointSpeed { port } => (
                    ProbeInput::PointSpeed {
                        point: point(bindings.mechanical_port(port)?),
                    },
                    "m/s",
                ),
                Probe::Support {
                    port,
                    geometries,
                    quantity,
                } => {
                    let point = point(bindings.mechanical_port(port)?);
                    let body = compiled
                        .bodies
                        .iter()
                        .find(|b| b.name == point.body)
                        .ok_or("нет массы опоры")?;
                    let names = compiled.contact_geometry_names()?;
                    if geometries.is_empty() || geometries.iter().any(|g| !names.contains(g)) {
                        return Err("неизвестна геометрия при проверке веса".into());
                    }
                    let unit = match quantity {
                        SupportQuantity::Vertical => "N",
                        SupportQuantity::StaticMoment => "N*m",
                    };
                    (
                        ProbeInput::Support {
                            point,
                            geometries: geometries.iter().cloned().collect(),
                            gravity_force: self.world.gravity_m_s2.map(|g| g * body.mass_kg),
                            quantity: quantity.clone(),
                        },
                        unit,
                    )
                }
                Probe::GyroResidual {
                    port,
                    sensor,
                    axis,
                    max_age_ns,
                } => {
                    let p = bindings.mechanical_port(port)?;
                    let mut vector = [0.0; 3];
                    *vector
                        .get_mut(*axis)
                        .ok_or("номер оси гироскопа вне диапазона")? = 1.0;
                    (
                        ProbeInput::GyroResidual {
                            point: point(p),
                            axis: port_vector(p, vector)?,
                            max_age_ns: *max_age_ns,
                            signal: rt.observation_id(sensor).ok_or("нет показания гироскопа")?,
                        },
                        "rad/s",
                    )
                }
            };
            rt.register_process(
                id,
                ProbeModel {
                    name: id.clone(),
                    unit: unit.into(),
                    input,
                },
            )?;
        }
        Ok(())
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CameraAudit {
    sensor: String,
    marker_geometry: String,
    color_component: usize,
    /// Моменты доставки ожидаемых кадров. Сроки проверяются по меткам самих кадров.
    frames_ns: Vec<[u64; 2]>,
    min_weight: f64,
}

#[derive(Serialize)]
struct CameraReport {
    frames: Vec<CameraFrameReport>,
    expected: usize,
    gaps: Vec<String>,
}

#[derive(Serialize)]
struct CameraFrameReport {
    captured_ns: u64,
    delivered_ns: u64,
    image: Fingerprint,
    centroid_px: [f64; 2],
    projection_px: [f64; 2],
    projection_error_px: f64,
}

#[derive(Clone)]
struct Projection {
    reader: sim_sense::FrameTruthReader,
    target: String,
    lens: sim_sense::Intrinsics,
    coordinates: [sim_runtime::TruthId; 2],
}

impl sim_runtime::Evaluator for Projection {
    fn evaluate(
        &mut self,
        truth: &sim_runtime::Truth<'_>,
        _: &mut sim_signal::Rng,
        out: &mut sim_runtime::TruthOut,
    ) -> Result<(), String> {
        for frame in self.reader.advance(truth)? {
            let object = frame
                .objects
                .iter()
                .find(|o| o.name == self.target)
                .ok_or_else(|| format!("в кадре истины нет геометрии «{}»", self.target))?;
            let [w, x, y, z] = frame.camera_pose.quat_wxyz;
            let [a, b, c] = std::array::from_fn(|i| {
                object.pose.position_m[i] - frame.camera_pose.position_m[i]
            });
            // Умножение на транспонированную матрицу позы камеры.
            let u = (1.0 - 2.0 * (y * y + z * z)) * a
                + 2.0 * (x * y + w * z) * b
                + 2.0 * (x * z - w * y) * c;
            let v = 2.0 * (x * y - w * z) * a
                + (1.0 - 2.0 * (x * x + z * z)) * b
                + 2.0 * (y * z + w * x) * c;
            let depth = 2.0 * (x * z + w * y) * a
                + 2.0 * (y * z - w * x) * b
                + (1.0 - 2.0 * (x * x + y * y)) * c;
            if depth <= 0.0 {
                continue;
            }
            // Независимое уравнение pinhole, в том же кадре координат, что и датчик.
            for (id, value) in self.coordinates.into_iter().zip([
                self.lens.fx * u / depth + self.lens.cx,
                self.lens.fy * v / depth + self.lens.cy,
            ]) {
                out.write(id, value);
            }
        }
        Ok(())
    }
}

impl CameraAudit {
    fn prepare(
        &self,
        rt: &mut Runtime,
        sensors: &[sim_sense::SensorHandle],
        end: Time,
    ) -> Result<(sim_signal::SignalId, [Option<usize>; 2]), String> {
        if self.frames_ns.is_empty()
            || self.color_component >= 3
            || self.min_weight <= 0.0
            || self.frames_ns.windows(2).any(|w| w[0][1] >= w[1][0])
            || self.frames_ns.iter().any(|[a, b]| a > b || *b > end.0)
        {
            return Err("невалидная сетка сверки кадров".into());
        }
        let camera = sensors
            .iter()
            .find(|s| s.name() == self.sensor)
            .ok_or("нет датчика изображения")?;
        let id = camera.signal("image").ok_or("нет RGB-канала")?;
        let lens = *camera.intrinsics().ok_or("у датчика нет оптики")?;
        if lens.radial != [0.0; 3] || lens.tangential != [0.0; 2] {
            return Err("сверка pinhole требует калибровочный режим без дисторсии".into());
        }
        let coordinates = [
            rt.register_truth("проверка проекции/x", "px")?,
            rt.register_truth("проверка проекции/y", "px")?,
        ];
        rt.register_evaluator(
            "сверка проекции",
            rt.physics_rate(),
            Projection {
                reader: camera
                    .frame_truth_reader()
                    .ok_or("нет источника истины кадра")?,
                target: self.marker_geometry.clone(),
                lens,
                coordinates,
            },
        )?;
        let indices = ["проверка проекции/x", "проверка проекции/y"]
            .map(|name| rt.log().header().truth.iter().position(|s| s.name == name));
        Ok((id, indices))
    }

    fn run(
        &self,
        rt: &mut Runtime,
        (id, indices): (sim_signal::SignalId, [Option<usize>; 2]),
    ) -> Result<CameraReport, String> {
        let mut report = CameraReport {
            expected: self.frames_ns.len(),
            frames: Vec::new(),
            gaps: Vec::new(),
        };
        let mut last_capture = None;
        for &[capture, delivery] in &self.frames_ns {
            rt.run_until(Time(delivery))?;
            let obs = rt.observe();
            let valid = obs.frame(id).filter(|f| {
                f.valid
                    && f.is_consistent()
                    && f.captured == Time(capture)
                    && f.at == Time(delivery)
                    && f.format == sim_signal::PixelFormat::Rgb8
            });
            let projection = indices.map(|i| {
                rt.log()
                    .entries()
                    .iter()
                    .rev()
                    .find(|e| {
                        Some(e.id as usize) == i
                            && e.kind == LogKind::Truth
                            && e.valid
                            && (capture..=delivery).contains(&e.at.0)
                            && last_capture.is_none_or(|last| e.at.0 > last)
                    })
                    .map(|e| e.value)
            });
            last_capture = Some(delivery);
            let Some(frame) = valid else {
                report
                    .gaps
                    .push(format!("нет доставленного RGB-кадра {capture} нс"));
                continue;
            };
            let [Some(x), Some(y)] = projection else {
                report.gaps.push(format!("нет проекции кадра {capture} нс"));
                continue;
            };
            let [mut weight, mut sum_x, mut sum_y] = [0.0; 3];
            for (i, rgb) in frame.data.as_chunks::<3>().0.iter().enumerate() {
                let other = (0..3)
                    .filter(|&c| c != self.color_component)
                    .map(|c| rgb[c])
                    .max()
                    .unwrap();
                // Пиксельный центр и нормировка RGB8 из sim-sense::Intrinsics и PixelFormat.
                let w =
                    f64::from(rgb[self.color_component].saturating_sub(other)) / f64::from(u8::MAX);
                weight += w;
                sum_x += (i % frame.width as usize) as f64 * w;
                sum_y += (i / frame.width as usize) as f64 * w;
            }
            if weight < self.min_weight {
                report
                    .gaps
                    .push(format!("мишень кадра {capture} нс не видна"));
                continue;
            }
            let measured = [sum_x / weight + 0.5, sum_y / weight + 0.5];
            report.frames.push(CameraFrameReport {
                captured_ns: capture,
                delivered_ns: delivery,
                image: Fingerprint::of(&frame.data),
                centroid_px: measured,
                projection_px: [x, y],
                projection_error_px: (measured[0] - x).hypot(measured[1] - y),
            });
        }
        Ok(report)
    }
}

#[derive(Serialize)]
struct NumericalConvergence {
    quantity: &'static str,
    unit: &'static str,
    differences: [Option<f64>; 2],
    tolerance: f64,
    outcome: Outcome,
    source: String,
}

fn numeric_convergence(
    e: &InteractionEvidence,
    t: &Tolerances,
) -> BTreeMap<String, Vec<NumericalConvergence>> {
    e.comparisons()
        .iter()
        .map(|(patch, rows)| {
            let rows = rows
                .iter()
                .filter_map(|row| {
                    let tolerance = match row.quantity {
                        "onset" => &t.onset_s,
                        "impulse" => &t.impulse_ns,
                        "duration" => &t.duration_s,
                        "drift" => &t.drift_m,
                        "pressure" | "mean_shear" => &t.stress_pa,
                        "normal" | "shear_min" | "shear_max" | "point_shear" => &t.force_n,
                        _ => return None,
                    };
                    let range = SourcedRange::from(tolerance.clone());
                    Some(NumericalConvergence {
                        quantity: row.quantity,
                        unit: row.unit,
                        differences: row.differences,
                        tolerance: range.min,
                        outcome: if row
                            .differences
                            .iter()
                            .all(|d| d.is_some_and(|v| v <= range.min))
                        {
                            Outcome::Pass
                        } else {
                            Outcome::NotAnalyzed
                        },
                        source: format!("{:?}", range.provenance),
                    })
                })
                .collect();
            (patch.clone(), rows)
        })
        .collect()
}

#[derive(Serialize)]
struct TissueAudit {
    region: String,
    patch: String,
    label: &'static str,
    measurement: Option<[f64; 2]>,
    metric: &'static str,
    unit: &'static str,
    outcome: Outcome,
    applicability: String,
    orientations: Vec<TissueOrientation>,
}

#[derive(Serialize)]
struct TissueOrientation {
    source: &'static str,
    locator: &'static str,
    range: Option<[f64; 2]>,
    population: &'static str,
    geometry: &'static str,
}

impl Experiment<'_> {
    fn audit_tissue(
        &self,
        rt: &Runtime,
        evidence: &InteractionEvidence,
        convergence: &BTreeMap<String, Vec<NumericalConvergence>>,
    ) -> Vec<TissueAudit> {
        let Some(summary) = evidence.grids().last() else {
            return Vec::new();
        };
        let Some(patches) = rt.interaction_patches() else {
            return Vec::new();
        };
        let participant = &self.case.participant.name;
        let mut out = Vec::new();
        for (patch, regions) in patches {
            let Some(values) = summary.patches.get(patch) else {
                continue;
            };
            let key = format!("patch/{patch}");
            let converged = convergence.get(&key).is_some_and(|rows| {
                !rows.is_empty() && rows.iter().all(|row| row.outcome == Outcome::Pass)
            });
            for region in regions.iter().flatten() {
                let id = RegionId(
                    region
                        .strip_prefix(participant)
                        .and_then(|s| s.strip_prefix('/'))
                        .unwrap_or(region)
                        .into(),
                );
                let label = self
                    .participant
                    .regions
                    .get(&id)
                    .map_or("неизвестная область", |r| r.label);
                for (metric, kind, measurement, direction) in [
                    (
                        scene::Metric::NormalForce,
                        "normal_force",
                        Some([values.peak_normal_n; 2]),
                        scene::Direction::Normal,
                    ),
                    (
                        scene::Metric::MeanPressure,
                        "mean_pressure",
                        values.peak_mean_pressure_pa,
                        scene::Direction::Normal,
                    ),
                    (
                        scene::Metric::MeanShear,
                        "mean_shear",
                        values.peak_mean_shear_pa,
                        scene::Direction::Tangential,
                    ),
                ] {
                    let load = |force: [f64; 2]| scene::Interval {
                        min: force[0],
                        max: force[1],
                    };
                    let force = match direction {
                        scene::Direction::Tangential => values.peak_shear_n,
                        _ => [values.peak_normal_n; 2],
                    };
                    let exposure = scene::Exposure {
                        metric,
                        value: load(force),
                    };
                    let context = scene::ExposureContext {
                        direction,
                        mode: scene::LoadMode::Transient,
                        patch: scene::Patch::Distributed,
                        duration_s: values.duration_s,
                        repetitions: 1,
                        population: "численный профиль".into(),
                        geometry: patch.clone(),
                        area_m2: None,
                        converged,
                        complete: summary.coverage_gaps.is_empty(),
                    };
                    let assessed = scene::assess_exposure(&id, exposure, &context, None);
                    let orientations = scene::limits(&id)
                        .into_iter()
                        .flatten()
                        .filter(|v| {
                            v.metric == metric && v.status == scene::LimitStatus::Orientation
                        })
                        .map(|v| TissueOrientation {
                            source: v.source.reference,
                            locator: v.source.locator,
                            range: v.range.map(|r| [r.min, r.max]),
                            population: v.applicability.population,
                            geometry: v.applicability.geometry,
                        })
                        .collect();
                    out.push(TissueAudit {
                        region: region.clone(),
                        patch: patch.clone(),
                        label,
                        measurement,
                        metric: kind,
                        unit: metric.unit(),
                        applicability: format!("{:?}", assessed.tissue),
                        outcome: match assessed.tissue {
                            scene::ExposureOutcome::WithinLimit => Outcome::Pass,
                            scene::ExposureOutcome::Fail => Outcome::Fail,
                            scene::ExposureOutcome::NotAnalyzed(_) => Outcome::NotAnalyzed,
                        },
                        orientations,
                    });
                }
            }
        }
        out
    }
}
