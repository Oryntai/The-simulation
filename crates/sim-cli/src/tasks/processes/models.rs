mod balance;
mod constitutive;
mod docking;
mod mechanical_monitor;
mod rotary;
mod thread;

use std::{collections::BTreeMap, path::Path};

use serde::Deserialize;
use sim_process::{
    Analysis, ProcessError,
    work::{
        HazardKind, ProcessKind,
        electrical::{
            BounceModel, BounceSegment, ContactFault, ElectricalContact, ElectricalInput,
            ElectricalSpec,
        },
        heat::{HeatInput, HeatSpec, HeatTransfer, ThermalContact},
        immersion::{
            DragSpec, ImmersionInput, ImmersionRegion, ImmersionSpec, RadioFluid, RadioWater,
            WetHeatSpec,
        },
        mechanics::{Handedness, ThreadSpec},
        trace::{SurfaceTrace, TraceSpec, TraceStep},
    },
};
use sim_runtime::{
    Runtime,
    process::{
        ImmersionAdapter, ProcessEvent, ProcessFrame, ProcessHazard, ProcessModel, ProcessOut,
        RadioPath,
    },
};
use sim_signal::{ChannelId, Rng, Time};
use sim_world::{CompiledJointKind, CompiledWorld, Simplification};

use super::{Origin, Scalar};

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
pub(super) enum Kind {
    SurfaceMarking,
    HeatTransfer,
    ThreadEngagement,
    ElectricalContact,
    Immersion,
    ImpactTransmission,
    StoredEnergy,
    SharpEdge,
}

impl Kind {
    pub(super) fn process_kind(self) -> ProcessKind {
        match self {
            Self::SurfaceMarking => ProcessKind::SurfaceMarking,
            Self::HeatTransfer => ProcessKind::HeatTransfer,
            Self::ThreadEngagement => ProcessKind::ThreadEngagement,
            Self::ElectricalContact => ProcessKind::ElectricalContact,
            Self::Immersion => ProcessKind::Immersion,
            Self::ImpactTransmission => ProcessKind::ImpactTransmission,
            Self::StoredEnergy => ProcessKind::StoredEnergy,
            Self::SharpEdge => ProcessKind::SharpEdge,
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Point {
    body: String,
    local_m: [f64; 3],
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Command {
    at_ns: u64,
    pub(super) value: f64,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Threshold {
    above: f64,
    below: f64,
    minimum_contact_n: f64,
    positive_pair: [String; 2],
    negative_pair: [String; 2],
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub(super) enum Spec {
    Effort {
        channel: String,
        values: Vec<Command>,
        source: String,
    },
    Coordinate {
        joint: String,
    },
    TransmissionResidual {
        input: String,
        output: String,
        ratio: Scalar,
        unit: String,
    },
    RotaryBalance {
        parameters: balance::Parameters,
    },
    ScalarHold {
        joint: String,
        at_ns: u64,
        source: String,
    },
    AngularStop {
        joint: String,
        channel: String,
        source: String,
        limit_rad: Scalar,
        lever_m: Scalar,
        stiffness_n_m: Scalar,
        damping_ns_m: Scalar,
    },
    RotaryMedium {
        joint: String,
        channel: String,
        source: String,
        sine_moment_nm: Scalar,
        quadratic_drag_nm_s2: Scalar,
    },
    BodyPoint {
        point: Point,
    },
    Coupler {
        carrier: Point,
        stations: BTreeMap<String, docking::Dock>,
        commands: Vec<docking::DockCommand>,
        bounds: [f64; 4],
        source: String,
    },
    ElasticAxis {
        channel: String,
        joint: String,
        source: String,
        stiffness: f64,
        damping: f64,
        seats: [f64; 2],
        detent: Option<Threshold>,
        capture_at_ns: Option<u64>,
        rest: Option<Vec<Command>>,
    },
    Grip {
        item: Point,
        carrier: Point,
        pairs: [[String; 2]; 2],
        mu: Scalar,
        mass_kg: Scalar,
        gravity_m_s2: Scalar,
        upward_axis: usize,
    },
    Trace {
        pair: [String; 2],
        plane: [Scalar; 3],
        normal: [Scalar; 3],
        tolerance_m: Scalar,
        path: Vec<[Scalar; 3]>,
    },
    Heat {
        pair: [String; 2],
        #[serde(default)]
        nominal_only: bool,
        capacities_j_k: [Scalar; 2],
        initial_k: [Scalar; 2],
        area_m2: Scalar,
        coefficient_w_m2_k: Scalar,
        power_w: Vec<Command>,
        source: String,
    },
    Electrical {
        latch: Option<String>,
        initially_closed: bool,
        #[serde(with = "Origin")]
        source: sim_world::Provenance,
        closed_ohm: Scalar,
        open_ohm: Scalar,
        source_ohm: Scalar,
        load_ohm: Scalar,
        supply_v: Scalar,
        supply_override_v: Option<f64>,
        fault: Option<ElectricalFault>,
        bounce_duration_s: Scalar,
        current_limit_a: Option<Scalar>,
        conductor_heat: Option<ConductorHeat>,
    },
    Thread {
        pair: [String; 2],
        angular: String,
        axial: String,
        lead_m: Scalar,
        travel_tolerance_m: Scalar,
        efficiency: Scalar,
        torque_effort: String,
        source: String,
    },
    Immersion(Box<ImmersionConfig>),
    Attachment(Box<constitutive::Attachment>),
    SoftObject(Box<constitutive::SoftObject>),
    StoredEnergy(Box<constitutive::Energy>),
    MechanicalMonitor(Box<mechanical_monitor::Spec>),
    SharpEdge(Box<constitutive::Edge>),
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct ConductorHeat {
    initial_k: Scalar,
    capacity_j_k: Scalar,
    pub(super) max_k: Scalar,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct ImmersionConfig {
    center: Point,
    half_extent_m: [Scalar; 3],
    surface_point_m: [Scalar; 3],
    outward_normal: [Scalar; 3],
    density_kg_m3: Scalar,
    viscosity_pa_s: Scalar,
    flow_m_s: [Scalar; 3],
    temperature_k: Scalar,
    drag_coefficient: Option<Scalar>,
    reference_length_m: Option<Scalar>,
    reynolds: Option<Scalar>,
    heat_coefficient: Option<Scalar>,
    heat_speed_m_s: Option<Scalar>,
    heat_area_m2: Option<Scalar>,
    drive: Option<String>,
    housing_c: Option<String>,
    initial_housing_k: Option<Scalar>,
    radio: Option<Box<Wireless>>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Wireless {
    antenna: Point,
    sea_at_catalog_salinity: bool,
    #[serde(with = "Origin")]
    composition_source: sim_world::Provenance,
    calibrated_temperature_k: Scalar,
    route: Option<String>,
    #[serde(default)]
    routes: Vec<String>,
    frequency_hz: Option<Scalar>,
    sensitivity_dbm: Option<Scalar>,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
pub(super) enum ElectricalFault {
    OpenCircuit,
    StuckClosed,
}

fn as_error(error: impl std::fmt::Display) -> String {
    error.to_string()
}

fn invalid(message: impl Into<String>) -> ProcessError {
    ProcessError::InvalidInput(message.into())
}

fn measured(value: f64, scope: &str) -> Analysis<f64> {
    Analysis::Analyzed {
        value,
        scope: scope.into(),
    }
}

fn validate_source(rt: &mut Runtime, name: &str, source: &str) -> Result<(), String> {
    if source.trim().is_empty() || source.chars().any(char::is_control) {
        return Err(format!("для «{name}» нужен источник"));
    }
    rt.add_simplification(Simplification {
        subject: name.into(),
        text: source.into(),
        blocks_proof: true,
    })
}

fn schedule(commands: &[Command]) -> Result<BTreeMap<Time, f64>, String> {
    let mut values = BTreeMap::new();
    for c in commands {
        if !c.value.is_finite() || values.insert(Time(c.at_ns), c.value).is_some() {
            return Err("невалидная точка календаря усилия".into());
        }
    }
    if values
        .first_key_value()
        .is_none_or(|(&t, _)| t != Time::ZERO)
    {
        return Err("календарь должен содержать исходное значение".into());
    }
    Ok(values)
}

fn scheduled(values: &BTreeMap<Time, f64>, at: Time) -> f64 {
    *values
        .range(..=at)
        .next_back()
        .expect("проверено начальное значение")
        .1
}

fn joint(rt: &CompiledWorld, name: &str) -> Result<CompiledJointKind, String> {
    match rt.joints.iter().find(|j| j.name == name).map(|j| j.kind) {
        Some(kind @ (CompiledJointKind::Hinge | CompiledJointKind::Slide)) => Ok(kind),
        _ => Err(format!("«{name}» не является скалярным соединением")),
    }
}

fn axis(frame: &ProcessFrame<'_>, name: &str) -> Result<(f64, f64), ProcessError> {
    frame
        .truth
        .joint(name)
        .map(|(p, v)| (p[0], v[0]))
        .ok_or_else(|| invalid(format!("нет соединения «{name}»")))
}

fn validate_pair(world: &CompiledWorld, pair: &[String; 2]) -> Result<(), String> {
    let names = world.contact_geometry_names()?;
    if pair[0] == pair[1] || !pair.iter().all(|name| names.contains(name)) {
        return Err(format!("неверная пара геометрий {pair:?}"));
    }
    Ok(())
}

fn normal(frame: &ProcessFrame<'_>, pair: &[String; 2]) -> f64 {
    frame
        .completed
        .into_iter()
        .flat_map(|f| &f.reactions.contacts)
        .filter(|c| {
            c.separation_m <= 0.0
                && (c.geometries == *pair
                    || c.geometries[0] == pair[1] && c.geometries[1] == pair[0])
        })
        .map(|c| {
            c.force_n
                .iter()
                .zip(c.normal)
                .map(|(f, n)| f * n)
                .sum::<f64>()
        })
        .sum()
}

fn bound(rt: &Runtime, name: &str) -> Result<ChannelId, String> {
    rt.channel(name)
        .ok_or_else(|| format!("нет канала усилия «{name}»"))
}

fn position(world: &CompiledWorld, point: &Point) -> Result<(), String> {
    if world.bodies.iter().all(|body| body.name != point.body) {
        return Err(format!("нет тела «{}»", point.body));
    }
    Ok(())
}

fn emit(
    out: &mut ProcessOut,
    at: Time,
    name: &str,
    values: &[(&str, &str, f64)],
    scope: &str,
) -> Result<(), ProcessError> {
    for &(field, unit, value) in values {
        out.measure(at, format!("{name}/{field}"), unit, measured(value, scope))?;
    }
    Ok(())
}

#[derive(Clone)]
struct Stateless<F>(F);

impl<F> ProcessModel for Stateless<F>
where
    F: Fn(&ProcessFrame<'_>) -> Result<ProcessOut, ProcessError>,
{
    fn update(
        &mut self,
        frame: &ProcessFrame<'_>,
        _: &mut Rng,
    ) -> Result<ProcessOut, ProcessError> {
        (self.0)(frame)
    }

    fn state_fingerprint(&self) -> u64 {
        0
    } // Нет изменяемого состояния.
}

#[derive(Clone, Debug)]
struct Elastic {
    name: String,
    channel: ChannelId,
    joint: String,
    stiffness: f64,
    damping: f64,
    seats: [f64; 2],
    detent: Option<Threshold>,
    capture_at: Option<Time>,
    captured: Option<f64>,
    seated: bool,
    rest: Option<BTreeMap<Time, f64>>,
    rotational: bool,
}

impl ProcessModel for Elastic {
    fn update(
        &mut self,
        frame: &ProcessFrame<'_>,
        _: &mut Rng,
    ) -> Result<ProcessOut, ProcessError> {
        let (q, v) = axis(frame, &self.joint)?;
        let at = frame.next.start;
        let mut output = ProcessOut::default();
        if let Some(t) = &self.detent {
            let new_state = if self.seated {
                !(q <= t.below && normal(frame, &t.negative_pair) >= t.minimum_contact_n)
            } else {
                q >= t.above && normal(frame, &t.positive_pair) >= t.minimum_contact_n
            };
            if self.seated != new_state {
                self.seated = new_state;
                output.events.push(ProcessEvent {
                    name: self.name.clone(),
                    state: self.state().into(),
                    requested: at,
                    applied: at,
                });
            }
        }
        if self.captured.is_none() && self.capture_at.is_some_and(|time| at >= time) {
            self.captured = Some(q);
            output.events.push(ProcessEvent {
                name: self.name.clone(),
                state: "удержание".into(),
                requested: self.capture_at.expect("есть команда"),
                applied: at,
            });
        }
        let effort = if self.capture_at.is_some() && self.captured.is_none() {
            0.0 // До команды в опоре отсутствует связь.
        } else {
            let seat = self
                .captured
                .or_else(|| self.rest.as_ref().map(|r| scheduled(r, at)))
                .unwrap_or(self.seats[usize::from(self.seated)]);
            -self.stiffness * (q - seat) - self.damping * v
        };
        output.efforts.push((self.channel, effort));
        output.states.insert(self.name.clone(), self.state().into());
        emit(
            &mut output,
            at,
            &self.name,
            &[(
                "reaction",
                if self.rotational { "N*m" } else { "N" },
                effort,
            )],
            "восстанавливающая сила пружины с вязким демпфером",
        )?;
        Ok(output)
    }

    fn state_fingerprint(&self) -> u64 {
        sim_signal::fnv1a(
            [
                self.captured.map_or(0, f64::to_bits),
                self.captured.is_some() as u64,
                self.seated as u64,
            ]
            .into_iter()
            .flat_map(u64::to_le_bytes),
        )
    }
}

impl Elastic {
    fn state(&self) -> &'static str {
        match (self.capture_at, self.captured, self.seated) {
            (Some(_), Some(_), _) => "удержание",
            (_, _, true) => "зафиксировано",
            _ => "свободно",
        }
    }
}

impl Spec {
    pub(super) fn validate_links(
        &self,
        models: &BTreeMap<String, Self>,
        world: &CompiledWorld,
    ) -> Result<(), String> {
        match self {
            Self::Thread {
                torque_effort,
                angular,
                ..
            } => {
                let Some(Self::Effort { channel, .. }) = models.get(torque_effort) else {
                    return Err(format!("нет модели подведённого момента «{torque_effort}»"));
                };
                if !world
                    .efforts
                    .iter()
                    .any(|e| e.name == *channel && e.joint == *angular && e.dof == 0)
                {
                    return Err("момент должен быть приложен к оси зацепления".into());
                }
            }
            Self::Electrical {
                latch: None,
                fault: None,
                ..
            } => {
                return Err("для управляемого контакта нужен бистабильный механизм".into());
            }
            Self::Electrical {
                latch: Some(latch), ..
            } if !matches!(
                models.get(latch),
                Some(Self::ElasticAxis {
                    detent: Some(_),
                    ..
                })
            ) =>
            {
                return Err(format!("нет бистабильного механизма «{latch}»"));
            }
            _ => {}
        }
        Ok(())
    }

    pub(super) fn kind(&self) -> Option<ProcessKind> {
        match self {
            Self::Trace { .. } => Some(Kind::SurfaceMarking),
            Self::Heat { .. } => Some(Kind::HeatTransfer),
            Self::Electrical { .. } => Some(Kind::ElectricalContact),
            Self::Thread { .. } => Some(Kind::ThreadEngagement),
            Self::Immersion(_) => Some(Kind::Immersion),
            Self::StoredEnergy(_) => Some(Kind::StoredEnergy),
            Self::SharpEdge(_) => Some(Kind::SharpEdge),
            Self::Grip { .. } | Self::ElasticAxis { .. } => Some(Kind::ImpactTransmission),
            _ => None,
        }
        .map(Kind::process_kind)
    }

    pub(super) fn uses_runtime_attachment(&self, name: &str) -> bool {
        matches!(self, Self::Attachment(config) if config.uses_weld(name))
    }

    pub(super) fn install(
        &self,
        name: &str,
        world: &CompiledWorld,
        rt: &mut Runtime,
        scene: &sim_world::World,
        world_file: &Path,
        subject: &str,
    ) -> Result<(), String> {
        match self.clone() {
            Self::Attachment(config) => config.install(name, world, rt, scene, world_file),
            Self::SoftObject(config) => config.install(name, world, rt),
            Self::StoredEnergy(config) => config.install(name, subject, rt),
            Self::MechanicalMonitor(config) => config.install(name, world, rt),
            Self::SharpEdge(config) => config.install(name, subject, world, rt),
            Self::Coupler {
                carrier,
                stations,
                commands,
                bounds,
                source,
            } => {
                validate_source(rt, name, &source)?;
                docking::install(name, world, rt, carrier, stations, commands, bounds)
            }
            Self::Effort {
                channel,
                values,
                source,
            } => {
                validate_source(rt, name, &source)?;
                let effort = world
                    .efforts
                    .iter()
                    .find(|e| e.name == channel)
                    .ok_or("усилие отсутствует в сцене")?;
                let kind = world
                    .joints
                    .iter()
                    .find(|j| j.name == effort.joint)
                    .ok_or("соединение усилия отсутствует в сцене")?
                    .kind;
                let unit = match kind {
                    CompiledJointKind::Slide => "N",
                    CompiledJointKind::Free if effort.dof < 3 => "N",
                    _ => "N*m",
                };
                let channel = bound(rt, &channel)?;
                let values = schedule(&values)?;
                let signal = name.to_owned();
                rt.register_process(
                    name,
                    Stateless(move |frame: &ProcessFrame<'_>| {
                        let mut out = ProcessOut::default();
                        let at = frame.next.start;
                        let effort = scheduled(&values, at);
                        out.efforts.push((channel, effort));
                        emit(
                            &mut out,
                            at,
                            &signal,
                            &[("effort", unit, effort)],
                            "фактически заданное обобщённое усилие",
                        )?;
                        Ok(out)
                    }),
                )
            }
            Self::RotaryBalance { parameters } => balance::install(name, world, rt, parameters),
            Self::ScalarHold {
                joint,
                at_ns,
                source,
            } => {
                validate_source(rt, name, &source)?;
                rotary::hold(name, rt, joint, Time(at_ns))
            }
            Self::AngularStop {
                joint,
                channel,
                source,
                limit_rad,
                lever_m,
                stiffness_n_m,
                damping_ns_m,
            } => {
                validate_source(rt, name, &source)?;
                let model = sim_process::work::rotary::AngularStop::new(
                    rotary::value(limit_rad)?,
                    rotary::value(lever_m)?,
                    rotary::value(stiffness_n_m)?,
                    rotary::value(damping_ns_m)?,
                )
                .map_err(as_error)?;
                rotary::stop(name, world, rt, joint, &channel, model)
            }
            Self::RotaryMedium {
                joint,
                channel,
                source,
                sine_moment_nm,
                quadratic_drag_nm_s2,
            } => {
                validate_source(rt, name, &source)?;
                let model = sim_process::work::rotary::RotaryMedium::new(
                    rotary::value(sine_moment_nm)?,
                    rotary::value(quadratic_drag_nm_s2)?,
                )
                .map_err(as_error)?;
                rotary::medium(name, world, rt, joint, &channel, model)
            }
            Self::TransmissionResidual {
                input,
                output,
                ratio,
                unit,
            } => {
                joint(world, &input)?;
                joint(world, &output)?;
                sim_process::SourcedRange::from(ratio.clone())
                    .validate()
                    .map_err(as_error)?;
                let signal = name.to_owned();
                rt.register_process(name, Stateless(move |frame: &ProcessFrame<'_>| {
                    let (input, _) = axis(frame, &input)?;
                    let (output, _) = axis(frame, &output)?;
                    let mut out = ProcessOut::default();
                    emit(&mut out, frame.next.start, &signal, &[("error", &unit, output - ratio.value * input)],
                        "абсолютная невязка кинематического отношения независимых координат MuJoCo")?;
                    Ok(out)
                }))
            }
            Self::Coordinate {
                joint: name_of_joint,
            } => {
                let kind = joint(world, &name_of_joint)?;
                let signal = name.to_owned();
                rt.register_process(
                    name,
                    Stateless(move |frame: &ProcessFrame<'_>| {
                        let (q, v) = axis(frame, &name_of_joint)?;
                        let (q_unit, v_unit) = if kind == CompiledJointKind::Hinge {
                            ("rad", "rad/s")
                        } else {
                            ("m", "m/s")
                        };
                        let mut out = ProcessOut::default();
                        emit(
                            &mut out,
                            frame.next.start,
                            &signal,
                            &[("position", q_unit, q), ("velocity", v_unit, v)],
                            "координата и скорость соединения MuJoCo",
                        )?;
                        Ok(out)
                    }),
                )
            }
            Self::BodyPoint { point } => {
                position(world, &point)?;
                let signal = name.to_owned();
                rt.register_process(
                    name,
                    Stateless(move |frame: &ProcessFrame<'_>| {
                        let p = frame.body_point(&point.body, point.local_m)?;
                        let mut out = ProcessOut::default();
                        let [x, y, z] = p.position_m;
                        emit(
                            &mut out,
                            frame.next.start,
                            &signal,
                            &[("x", "m", x), ("y", "m", y), ("z", "m", z)],
                            "прямое преобразование точки тела в мировые координаты",
                        )?;
                        Ok(out)
                    }),
                )
            }
            Self::ElasticAxis {
                channel,
                joint: name_of_joint,
                source,
                stiffness,
                damping,
                seats,
                detent,
                capture_at_ns,
                rest,
            } => {
                validate_source(rt, name, &source)?;
                let rotational = joint(world, &name_of_joint)? == CompiledJointKind::Hinge;
                let effort = world
                    .efforts
                    .iter()
                    .find(|e| e.name == channel && e.joint == name_of_joint)
                    .ok_or("канал пружины должен приводить указанное соединение")?;
                let channel = bound(rt, &effort.name)?;
                if stiffness <= 0.0
                    || damping < 0.0
                    || usize::from(detent.is_some())
                        + usize::from(rest.is_some())
                        + usize::from(capture_at_ns.is_some())
                        > 1
                {
                    return Err("невалидная характеристика пружины".into());
                }
                if let Some(t) = &detent {
                    // Равная потенциальная энергия двух пружинных ветвей
                    // достигается в середине между их положениями покоя.
                    let equal_energy = seats[0].midpoint(seats[1]);
                    if t.above < equal_energy
                        || t.below > equal_energy
                        || t.above <= t.below
                        || t.minimum_contact_n <= 0.0
                        || seats[0] >= t.below
                        || seats[1] <= t.above
                    {
                        return Err("невалидная характеристика двух устойчивых положений".into());
                    }
                    validate_pair(world, &t.positive_pair)?;
                    validate_pair(world, &t.negative_pair)?;
                }
                rt.register_process(
                    name,
                    Elastic {
                        name: name.into(),
                        channel,
                        joint: name_of_joint,
                        stiffness,
                        damping,
                        seats,
                        detent,
                        capture_at: capture_at_ns.map(Time),
                        captured: None,
                        seated: false,
                        rest: rest.as_deref().map(schedule).transpose()?,
                        rotational,
                    },
                )
            }
            Self::Grip {
                item,
                carrier,
                pairs,
                mu,
                mass_kg,
                gravity_m_s2,
                upward_axis,
            } => {
                position(world, &item)?;
                position(world, &carrier)?;
                for pair in &pairs {
                    validate_pair(world, pair)?;
                }
                if upward_axis >= 3
                    || mu.value <= 0.0
                    || mass_kg.value <= 0.0
                    || gravity_m_s2.value <= 0.0
                {
                    return Err("невалидный опыт удержания трением".into());
                }
                let [mu, mass, g]: [sim_process::SourcedRange; 3] =
                    [mu, mass_kg, gravity_m_s2].map(Into::into);
                for s in [&mu, &mass, &g] {
                    s.validate().map_err(as_error)?;
                    if s.min <= 0.0 {
                        return Err("недопустимые параметры удержания".into());
                    }
                }
                if pairs[0] == pairs[1] || pairs[0].iter().eq(pairs[1].iter().rev()) {
                    return Err("повтор контактной пары двустороннего прижима".into());
                }
                let sim = world.open_contact_sim()?;
                let geoms = sim.geoms();
                let bodies = sim.bodies();
                let item_geoms = pairs.each_ref().map(|pair| {
                    let mut matching = pair.iter().filter(|name| {
                        geoms
                            .iter()
                            .any(|g| g.name == **name && bodies[g.body].0 == item.body)
                    });
                    let geom = matching
                        .next()
                        .ok_or("пара не касается удерживаемого тела")?;
                    if matching.next().is_some() {
                        return Err("пара контактов замкнута внутри удерживаемого тела");
                    }
                    Ok(geom.clone())
                });
                let [left, right] = item_geoms;
                let item_geoms = [left?, right?];
                let signal = name.to_owned();
                rt.register_process(name, Stateless(move |frame: &ProcessFrame<'_>| {
                    let p = frame.body_point(&item.body, item.local_m)?;
                    let holder = frame.body_point(&carrier.body, carrier.local_m)?;
                    let mut reactions = [0.0; 2];
                    let mut capacity = 0.0;
                    let mut friction_support = 0.0;
                    for (i, pair) in pairs.iter().enumerate() {
                        for c in frame.completed.into_iter().flat_map(|f| &f.engine_step.contacts)
                            .filter(|c| c.separation_m <= 0.0
                                && (c.geometries == *pair || c.geometries.iter().eq(pair.iter().rev())))
                        {
                            let n = c.force_n.iter().zip(c.frame[0])
                                .map(|(f, a)| f * a).sum::<f64>().max(0.0);
                            reactions[i] += n;
                            if c.dim >= 3 && n > 0.0 {
                                // На эллипсе меньшая полуось даёт нижнюю границу
                                // доступного кулоновского усилия в любом направлении.
                                capacity += n * mu.min.min(c.friction[0]).min(c.friction[1]);
                                let sign = if c.geometries[1] == item_geoms[i] { 1.0 } else { -1.0 };
                                friction_support += sign * (c.force_n[upward_axis]
                                    - n * c.frame[0][upward_axis]);
                            }
                        }
                    }
                    let [a, b] = reactions;
                    let weight = mass.max * g.max;
                    let margin = if a > 0.0 && b > 0.0 { capacity - weight } else { -weight };
                    let mut out = ProcessOut::default();
                    emit(&mut out, frame.next.start, &signal, &[("normal_left", "N", a), ("normal_right", "N", b),
                        ("static_margin", "N", margin), ("friction_support", "N", friction_support),
                        ("height", "m", p.position_m[upward_axis]),
                        ("relative_height", "m", p.position_m[upward_axis] - holder.position_m[upward_axis]),
                        ("x", "m", p.position_m[0]), ("y", "m", p.position_m[1])],
                        "статический баланс Кулона, измеренная касательная реакция на теле и координаты")?;
                    Ok(out)
                }))
            }
            Self::Trace {
                pair,
                plane,
                normal: direction,
                tolerance_m,
                path,
            } => {
                validate_pair(world, &pair)?;
                let trace = SurfaceTrace::new(TraceSpec {
                    pair,
                    plane_point_m: plane.map(Into::into),
                    plane_normal: direction.map(Into::into),
                    plane_tolerance_m: tolerance_m.into(),
                    reference_m: path.into_iter().map(|p| p.map(Into::into)).collect(),
                })
                .map_err(as_error)?;
                let name_of_trace = name.to_owned();
                let mut length = 0.0;
                let mut maximum_error: f64 = 0.0;
                rt.register_step_model(
                    name,
                    trace,
                    |frame| {
                        Ok(frame
                            .completed
                            .map(|f| (f.interval, Some(f.reactions.clone()))))
                    },
                    move |at, response| {
                        let mut out = ProcessOut::default();
                        match response {
                            Analysis::Analyzed {
                                value:
                                    TraceStep::Mark {
                                        path_error_m,
                                        sampled_length_m,
                                        ..
                                    },
                                ..
                            } => {
                                length = sampled_length_m;
                                maximum_error = maximum_error.max(path_error_m);
                            }
                            Analysis::Analyzed {
                                value: TraceStep::NoContact,
                                ..
                            } => {}
                            Analysis::NotAnalyzed { reason } => out.blockers.push(reason),
                        }
                        emit(
                            &mut out,
                            at,
                            &name_of_trace,
                            &[
                                ("length", "m", length),
                                ("maximum_error", "m", maximum_error),
                            ],
                            "геометрия следа нагруженной точки контакта",
                        )?;
                        Ok(out)
                    },
                )
            }
            Self::Heat {
                pair,
                nominal_only,
                capacities_j_k,
                initial_k,
                area_m2,
                coefficient_w_m2_k,
                power_w,
                source,
            } => {
                validate_source(rt, name, &source)?;
                validate_pair(world, &pair)?;
                let power = schedule(&power_w)?;
                let area = area_m2.into();
                let [source_capacity_j_k, target_capacity_j_k] = capacities_j_k.map(Into::into);
                let [source_initial_k, target_initial_k] = initial_k.map(Into::into);
                let spec = HeatSpec {
                    source_capacity_j_k,
                    target_capacity_j_k,
                    source_initial_k,
                    target_initial_k,
                    contact_coefficient_w_m2_k: coefficient_w_m2_k.into(),
                    calibrated_area_m2: area,
                };
                let model = if nominal_only {
                    rt.add_simplification(sim_world::Simplification {
                        subject: name.into(),
                        text: "Тепловой опыт задаёт номинальную площадь пятна. Физическая применимость остаётся открытой.".into(),
                        blocks_proof: true,
                    })?;
                    HeatTransfer::new_diagnostic(spec.clone())
                } else {
                    HeatTransfer::new(spec.clone())
                }.map_err(as_error)?;
                let name_of_heat = name.to_owned();
                let mut supplied = 0.0;
                rt.register_step_model(name, model, move |frame| {
                    Ok(frame.completed.map(|f| (f.interval, HeatInput {
                        power_into_source_w: scheduled(&power, f.interval.start),
                        contact: Some(if normal(frame, &pair) > 0.0 {
                            ThermalContact::Touching { area_m2: Some(spec.calibrated_area_m2.clone()) }
                        } else { ThermalContact::Separated }),
                    })))
                }, move |at, result| {
                    let mut out = ProcessOut::default();
                    match result {
                        Analysis::Analyzed { value, .. } => {
                            supplied += value.supplied_j;
                            let storage = spec.source_capacity_j_k.nominal * (value.source_k - spec.source_initial_k.nominal)
                                + spec.target_capacity_j_k.nominal * (value.target_k - spec.target_initial_k.nominal);
                            emit(&mut out, at, &name_of_heat, &[("source", "K", value.source_k), ("target", "K", value.target_k),
                                ("energy_residual", "J", storage - supplied), ("supplied", "J", supplied)],
                                "две теплоёмкости, проверка по сумме изменений внутренней энергии")?;
                        }
                        Analysis::NotAnalyzed { reason } => out.blockers.push(reason),
                    }
                    Ok(out)
                })
            }
            Self::Electrical {
                latch,
                initially_closed,
                source,
                closed_ohm,
                open_ohm,
                source_ohm,
                load_ohm,
                supply_v,
                supply_override_v,
                fault,
                bounce_duration_s,
                current_limit_a,
                conductor_heat,
            } => {
                // Адаптер состояния защёлки к нормированному ходу: 0 и 1 — концы шкалы.
                let scale = |value| sim_process::SourcedRange {
                    min: value,
                    nominal: value,
                    max: value,
                    provenance: source.clone(),
                };
                let supply = supply_override_v.unwrap_or(supply_v.value);
                let fault = match fault {
                    None => ContactFault::None,
                    Some(ElectricalFault::OpenCircuit) => ContactFault::OpenCircuit,
                    Some(ElectricalFault::StuckClosed) => ContactFault::StuckClosed,
                };
                let closed_resistance = closed_ohm.value;
                let open_resistance = open_ohm.value;
                let series_resistance = source_ohm.value + load_ohm.value;
                let steady = BounceSegment {
                    duration_s: bounce_duration_s.into(),
                    follows_target: true,
                };
                let spec = ElectricalSpec {
                    release_below_m: scale(0.25),
                    close_above_m: scale(0.75),
                    initially_closed,
                    initial_state_source: source,
                    bounce: Some(BounceModel::Profile {
                        closing: vec![steady.clone()],
                        opening: vec![steady],
                    }),
                    contact_closed_ohm: closed_ohm.into(),
                    contact_open_ohm: open_ohm.into(),
                    source_resistance_ohm: source_ohm.into(),
                    load_ohm: load_ohm.into(),
                    calibrated_supply_v: supply_v.into(),
                };
                let model = ElectricalContact::new(spec.clone()).map_err(as_error)?;
                if let Some(heat) = &conductor_heat {
                    for (value, origin) in [&heat.initial_k, &heat.capacity_j_k, &heat.max_k]
                        .into_iter().map(|s| (s.value, &s.source))
                    {
                        if !value.is_finite() || value <= 0.0
                            || matches!(origin, sim_world::Provenance::Estimate(_) | sim_world::Provenance::Derived(_))
                        {
                            return Err("нужны подтверждённые положительные параметры теплоизолированного проводника".into());
                        }
                    }
                    let inputs = [
                        &spec.contact_closed_ohm, &spec.contact_open_ohm, &spec.source_resistance_ohm,
                        &spec.load_ohm, &spec.calibrated_supply_v,
                    ];
                    if [&heat.initial_k, &heat.capacity_j_k, &heat.max_k].into_iter().any(|input| {
                        input.range.is_some_and(|bounds| bounds != [input.value; 2])
                    }) || inputs.into_iter().any(|input| {
                        input.min != input.max || matches!(input.provenance,
                            sim_world::Provenance::Estimate(_) | sim_world::Provenance::Derived(_))
                    }) || supply != spec.calibrated_supply_v.nominal {
                        return Err("тепловой баланс требует проверяемых точечных параметров цепи".into());
                    }
                }
                let assessor = model.clone();
                let current_limit_a = current_limit_a.map(sim_process::SourcedRange::from);
                if let Some(limit) = &current_limit_a {
                    limit.validate().map_err(as_error)?;
                    if limit.min < 0.0 {
                        return Err("предел тока должен быть неотрицательным".into());
                    }
                }
                let current_sources: Vec<_> = current_limit_a
                    .iter()
                    .cloned()
                    .chain([
                        spec.contact_closed_ohm,
                        spec.contact_open_ohm,
                        spec.source_resistance_ohm,
                        spec.load_ohm,
                        spec.calibrated_supply_v,
                    ])
                    .collect();
                let signal = name.to_owned();
                let circuit = subject.to_owned();
                let mut previous = None;
                let mut heat_energy = Some(0.0);
                rt.register_step_model(
                    name,
                    model,
                    move |frame| {
                        Ok(Some((
                            frame.next,
                            ElectricalInput {
                                travel_m: latch
                                    .as_ref()
                                    .and_then(|latch| frame.truth.process_state(latch))
                                    .map(|s| {
                                        if s == "зафиксировано" {
                                            1.0
                                        } else {
                                            0.0
                                        }
                                    })
                                    .or_else(|| {
                                        (frame.next.start == Time::ZERO)
                                            .then_some(f64::from(initially_closed))
                                    }),
                                supply_v: supply,
                                fault,
                            },
                        )))
                    },
                    move |_, result| {
                        let mut out = ProcessOut::default();
                        if let Some(limit) = &current_limit_a {
                            out.hazards.push(ProcessHazard {
                                subject: circuit.clone(),
                                kind: HazardKind::Overcurrent,
                                check: assessor.assess_overcurrent(&result, Some(limit))?,
                                sources: current_sources.clone(),
                            });
                        }
                        match result.segments {
                            Analysis::Analyzed { value, .. } => {
                                for step in value {
                                    let at = step.interval.start;
                                    let state = if step.closed {
                                        "замкнуто"
                                    } else {
                                        "разомкнуто"
                                    };
                                    if previous.is_some_and(|old| old != step.closed) {
                                        out.events.push(ProcessEvent {
                                            name: signal.clone(),
                                            state: state.into(),
                                            requested: at,
                                            applied: at,
                                        });
                                    }
                                    previous = Some(step.closed);
                                    out.states.insert(signal.clone(), state.into());
                                    let resistance = if step.closed {
                                        closed_resistance
                                    } else {
                                        open_resistance
                                    };
                                    emit(
                                        &mut out,
                                        at,
                                        &signal,
                                        &[
                                            ("current", "A", step.current_a),
                                            ("voltage", "V", step.load_v),
                                            (
                                                "ohm_residual",
                                                "A",
                                                step.current_a
                                                    - supply / (series_resistance + resistance),
                                            ),
                                        ],
                                        "ток нагрузки и независимая проверка по закону Ома",
                                    )?;
                                    if let (Some(spec), Some(joules)) = (&conductor_heat, &mut heat_energy) {
                                        // Для теплоизолированного участка вся диссипация
                                        // последовательного проводника меняет его внутреннюю энергию.
                                        let temperature = spec.initial_k.value + *joules / spec.capacity_j_k.value;
                                        emit(
                                            &mut out,
                                            at,
                                            &signal,
                                            &[("conductor_k", "K", temperature)],
                                            "адиабатический баланс I²R и теплоёмкости последовательного проводника",
                                        )?;
                                        *joules += step.contact_j;
                                    }
                                }
                            }
                            Analysis::NotAnalyzed { reason } => {
                                heat_energy = None;
                                out.blockers.push(reason);
                            }
                        }
                        Ok(out)
                    },
                )
            }
            Self::Thread {
                pair,
                angular,
                axial,
                lead_m,
                travel_tolerance_m,
                efficiency,
                torque_effort,
                source,
            } => {
                validate_source(rt, name, &source)?;
                validate_pair(world, &pair)?;
                if joint(world, &angular)? != CompiledJointKind::Hinge
                    || joint(world, &axial)? != CompiledJointKind::Slide
                {
                    return Err("резьбе нужны вращательная и поступательная оси".into());
                }
                let mut pitch: sim_process::SourcedRange = lead_m.into();
                pitch.validate().map_err(as_error)?;
                let direction = if pitch.min > 0.0 {
                    Handedness::Positive
                } else if pitch.max < 0.0 {
                    (pitch.min, pitch.nominal, pitch.max) =
                        (-pitch.max, -pitch.nominal, -pitch.min);
                    Handedness::Negative
                } else {
                    return Err("диапазон шага резьбы пересекает нуль".into());
                };
                let model = ThreadSpec {
                    lead_m_per_turn: pitch,
                    handedness: direction,
                    efficiency: efficiency.into(),
                    travel_tolerance_m: travel_tolerance_m.into(),
                    tensile_strength: None,
                };
                thread::install(name, world, rt, pair, angular, axial, model, torque_effort)
            }
            Self::Immersion(config) => {
                let ImmersionConfig {
                    center,
                    half_extent_m,
                    surface_point_m,
                    outward_normal,
                    density_kg_m3,
                    viscosity_pa_s,
                    flow_m_s,
                    temperature_k,
                    drag_coefficient,
                    reference_length_m,
                    reynolds,
                    heat_coefficient,
                    heat_speed_m_s,
                    heat_area_m2,
                    drive,
                    housing_c,
                    initial_housing_k,
                    radio,
                } = *config;
                position(world, &center)?;
                let drag = match (drag_coefficient, reference_length_m, reynolds) {
                    (Some(coefficient), Some(length), Some(reynolds)) => Some(DragSpec {
                        coefficient: coefficient.into(),
                        reference_length_m: length.into(),
                        calibrated_reynolds: reynolds.into(),
                    }),
                    (None, None, None) => None,
                    _ => return Err("неполное определение сопротивления жидкости".into()),
                };
                let heat = match (heat_coefficient, heat_speed_m_s, heat_area_m2) {
                    (Some(coefficient), Some(speed), Some(area)) => Some(WetHeatSpec {
                        coefficient_w_m2_k: coefficient.into(),
                        calibrated_speed_m_s: speed.into(),
                        calibrated_area_m2: area.into(),
                    }),
                    (None, None, None) => None,
                    _ => return Err("неполное определение мокрого теплоотвода".into()),
                };
                let drive = drive.map(|name| bound(rt, &name)).transpose()?;
                let housing = housing_c
                    .map(|name| {
                        rt.truth_id(&name)
                            .ok_or_else(|| format!("нет ряда температуры «{name}»"))
                    })
                    .transpose()?;
                if let Some(initial) = &initial_housing_k {
                    sim_process::SourcedRange::from(initial.clone())
                        .validate()
                        .map_err(as_error)?;
                }
                let (radio_model, antenna, routes) = if let Some(radio) = radio {
                    position(world, &radio.antenna)?;
                    let mut routes = radio.routes;
                    if let Some(route) = radio.route {
                        if !routes.is_empty() {
                            return Err("задайте либо route, либо routes".into());
                        }
                        routes.push(route);
                    }
                    match (routes.is_empty(), radio.frequency_hz, radio.sensitivity_dbm) {
                        (false, Some(frequency_hz), Some(sensitivity_dbm)) => {
                            let path = RadioPath {
                                frequency_hz: frequency_hz.into(),
                                sensitivity_dbm: sensitivity_dbm.into(),
                            };
                            for name in &routes {
                                rt.bind_process_radio(name, path.clone())?;
                            }
                        }
                        (true, None, None) => {}
                        _ => return Err("неполная настройка радиолинии".into()),
                    }
                    let water = if radio.sea_at_catalog_salinity {
                        RadioWater::SeaAtCatalogSalinity
                    } else {
                        RadioWater::Fresh
                    };
                    (
                        Some(RadioFluid {
                            water,
                            composition_source: radio.composition_source,
                            calibrated_temperature_k: radio.calibrated_temperature_k.into(),
                        }),
                        Some(radio.antenna),
                        routes,
                    )
                } else {
                    (None, None, Vec::new())
                };
                let model = ImmersionSpec {
                    body: center.body.clone(),
                    half_extent_m: half_extent_m.map(Into::into),
                    region: ImmersionRegion {
                        surface_point_m: surface_point_m.map(Into::into),
                        outward_normal: outward_normal.map(Into::into),
                        density_kg_m3: density_kg_m3.into(),
                        viscosity_pa_s: viscosity_pa_s.into(),
                        flow_m_s: flow_m_s.map(Into::into),
                        temperature_k: temperature_k.into(),
                    },
                    drag,
                    heat,
                    radio: radio_model,
                };
                rt.register_immersion(ImmersionAdapter {
                    name: name.into(),
                    model,
                    drive,
                    radio: routes,
                    input: move |frame: &ProcessFrame<'_>| {
                        let box_center = frame.body_point(&center.body, center.local_m)?;
                        let local_com = std::array::from_fn(|i| {
                            center.local_m[i]
                                + (0..3)
                                    .map(|j| {
                                        box_center.rotation[j][i]
                                            * (box_center.com_m[j] - box_center.position_m[j])
                                    })
                                    .sum::<f64>()
                        });
                        let com = frame.body_point(&center.body, local_com)?;
                        Ok(ImmersionInput {
                            box_center_m: box_center.position_m,
                            box_axes: std::array::from_fn(|i| {
                                std::array::from_fn(|j| box_center.rotation[j][i])
                            }),
                            body_com_m: com.position_m,
                            velocity_at_com_m_s: com.velocity_m_s,
                            angular_velocity_rad_s: com.angular_velocity_rad_s,
                            // Эти три поля подставляет адаптер из фактической сцены.
                            gravity_m_s2: [0.0; 3],
                            engine_density_kg_m3: 0.0,
                            engine_viscosity_pa_s: 0.0,
                            housing_temperature_k: housing
                                .and_then(|id| frame.truth.plant(id))
                                .map(|t| t + 273.15) // Определение шкалы Кельвина.
                                .or_else(|| {
                                    (frame.next.start == Time::ZERO)
                                        .then(|| initial_housing_k.as_ref().map(|t| t.value))
                                        .flatten()
                                }),
                            antenna_m: antenna
                                .as_ref()
                                .map(|point| {
                                    frame
                                        .body_point(&point.body, point.local_m)
                                        .map(|body| body.position_m)
                                })
                                .transpose()?,
                        })
                    },
                })
            }
        }
    }
}
