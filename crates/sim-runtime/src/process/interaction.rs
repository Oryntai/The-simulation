//! Связанные нагрузки и двигательные намерения на границах физики.

use std::collections::{BTreeMap, BTreeSet};

use sim_mujoco::{JointInfo, JointKind, ffi::mjtObj};
use sim_process::attachments::{
    CompliantPad, ConfirmedEvent, ConfirmedInterfaceEvent, DriveMotion, FreeLengthLaw,
    InterfaceInput, InterfaceReaction, InterfaceState, InterfaceStep, MountMotion, PadInput,
    PadReactionOwner, RoutePoint, TensionInput, TensionRoute,
};
use sim_signal::{Rate, fnv1a};

use super::*;

type Vector = [f64; 3];

/// Версия контракта журнала и доказательства связанного прогона.
pub const INTERACTION_VERSION: u32 = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InteractionIdentity {
    pub version: u32,
    pub world: u64,
    /// Отпечаток профиля, внешних ресурсов и всех привязок.
    pub profile: u64,
    pub variant: u64,
    pub seed: u64,
    pub rate: Rate,
}

#[derive(Clone, Debug)]
pub struct ResolvedInteraction {
    pub name: String,
    pub resources: u64,
    pub variant: u64,
    /// Участок → все геометрии, покрытие которых проверяется.
    pub regions: BTreeMap<String, Vec<String>>,
    pub pads: Vec<PadBinding>,
    pub routes: Vec<RouteBinding>,
    pub motion: MotionProfile,
}

#[derive(Clone, Debug, PartialEq)]
pub struct LocalPoint {
    pub body: String,
    pub position_m: Vector,
}

#[derive(Clone, Debug, PartialEq)]
pub struct LocalMount {
    pub point: LocalPoint,
    pub normal: Vector,
    pub tangent: Vector,
}

#[derive(Clone, Debug)]
pub struct PadBinding {
    pub model: CompliantPad,
    pub mounts: [LocalMount; 2],
    pub events: Vec<ConfirmedInterfaceEvent>,
}

#[derive(Clone, Debug)]
pub struct RouteBinding {
    pub model: TensionRoute,
    pub points: Vec<LocalPoint>,
    pub surfaces: Vec<RouteSurface>,
    pub events: Vec<ConfirmedInterfaceEvent>,
}

/// Точка передачи натяжения на подтверждённую поверхность.
#[derive(Clone, Debug)]
pub struct RouteSurface {
    pub patch: String,
    pub geometry: String,
    pub point: usize,
    /// Соседняя точка, относительно которой измеряется смещение.
    pub reference: usize,
    /// Нормаль, направленная внутрь поверхности, и касательная ось в локальном кадре.
    pub normal: Vector,
    pub tangent: Vector,
    pub area_m2: Option<SourcedRange>,
}

#[derive(Clone, Debug)]
pub enum MotionProfile {
    Dynamic {
        axes: Vec<MotionAxis>,
        maneuver: Option<Maneuver>,
        response: Option<Box<Response>>,
    },
    /// Испытательная постановка с заданной неподвижной координатой.
    FixedPose { joint: String, position: f64 },
}

#[derive(Clone, Debug)]
pub struct MotionAxis {
    pub channel: ChannelId,
    pub initial_position: f64,
    pub initial_velocity: f64,
    pub reference: Trajectory,
    pub stiffness: SourcedRange,
    pub damping: SourcedRange,
    /// Нижняя граница доступного усилия в обе стороны.
    pub effort_limit: SourcedRange,
}

#[derive(Clone, Debug)]
pub struct Trajectory {
    pub source: Provenance,
    pub knots: Vec<(Time, f64)>,
}

#[derive(Clone, Debug)]
pub struct Maneuver {
    pub at: Time,
    pub initiator: String,
    pub offsets: BTreeMap<ChannelId, Trajectory>,
}

#[derive(Clone, Debug)]
pub struct Response {
    pub enabled: bool,
    pub initiator: String,
    pub patch: String,
    pub threshold_n: SourcedRange,
    pub latency_ns: u64,
    pub latency_source: Provenance,
    pub offsets: BTreeMap<ChannelId, Trajectory>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PatchSample {
    pub regions: [Option<String>; 2],
    pub interval: Interval,
    pub owner: ReactionOwner,
    /// Знаки в порядке сторон привязки, касательный вектор сохранён на второй стороне.
    pub reaction: Analysis<InterfaceReaction>,
    pub calibrated: bool,
    pub area_m2: Option<SourcedRange>,
    /// Касательное смещение в локальном базисе первой стороны.
    pub offset_m: [f64; 2],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReactionOwner {
    Engine,
    Model,
}

#[derive(Clone, Debug, PartialEq)]
pub struct MotionRecord {
    pub initiator: String,
    pub sources: Vec<Provenance>,
    pub contact_at: Option<Time>,
    pub due: Time,
    pub interval: Interval,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AxisSample {
    pub channel: ChannelId,
    pub position: f64,
    pub velocity: f64,
    pub reference: f64,
    pub reference_velocity: f64,
    pub applied_effort: f64,
    /// Изменение ограниченного усилия после реакции при том же состоянии.
    pub response_effort_delta: f64,
    pub interval: Interval,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct InteractionTick {
    pub patches: BTreeMap<String, PatchSample>,
    /// Следующий интервал для закона, завершённый для измерения движка.
    pub interfaces: BTreeMap<String, InterfaceStep>,
    pub completed_routes: BTreeMap<String, InterfaceStep>,
    pub axes: Vec<AxisSample>,
    pub motion_events: Vec<MotionRecord>,
    pub coverage_gaps: Vec<String>,
}

impl Trajectory {
    fn check(&self) -> Result<(), String> {
        check_source(&self.source)?;
        if self.knots.first().is_none_or(|k| k.0 != Time::ZERO)
            || self.knots.windows(2).any(|w| w[0].0 >= w[1].0)
            || self.knots.iter().any(|k| !k.1.is_finite())
        {
            return Err("профиль требует конечные значения и возрастающие времена от нуля".into());
        }
        Ok(())
    }

    fn sample(&self, time: Time) -> (f64, f64) {
        let index = self.knots.partition_point(|k| k.0 <= time);
        let (start, from) = self.knots[index - 1];
        let Some(&(end, to)) = self.knots.get(index) else {
            return (from, 0.0);
        };
        let elapsed = Time(time.0 - start.0).as_secs();
        let duration = Time(end.0 - start.0).as_secs();
        let u = elapsed / duration;
        // Единственный полином степени 5 с заданными концами и нулевыми
        // скоростью и ускорением на них. Производная 30u²(1−u)².
        let blend = u.powi(3) * (10.0 + u * (-15.0 + 6.0 * u));
        let rate = 30.0 * u.powi(2) * (1.0 - u).powi(2) / duration;
        ((1.0 - blend) * from + blend * to, (to - from) * rate)
    }
}

#[derive(Clone)]
struct BoundInteraction {
    spec: ResolvedInteraction,
    axes: BTreeMap<ChannelId, JointInfo>,
    initial_joints: Vec<(JointInfo, f64, f64)>,
    regions: BTreeMap<String, String>,
    movable_bodies: BTreeSet<String>,
    contact_gaps: Vec<String>,
    blockers: Vec<String>,
    fingerprint: u64,
    previous: BTreeMap<String, (InterfaceStep, [f64; 2])>,
    previous_routes: BTreeMap<String, InterfaceStep>,
    previous_surfaces: BTreeMap<String, PatchSample>,
    contact_at: Option<Time>,
    response_at: Option<Time>,
    maneuver_at: Option<Time>,
}

#[derive(Clone)]
pub(super) struct RegisteredInteraction {
    pub identity: InteractionIdentity,
    pub patches: BTreeMap<String, [Option<String>; 2]>,
    pub regions: BTreeSet<String>,
    pub motion: String,
    expects_response: bool,
}

impl Runtime {
    pub fn interaction_identity(&self) -> Option<InteractionIdentity> {
        Some(self.state.work.interaction.as_ref()?.identity)
    }

    pub fn interaction_patches(&self) -> Option<&BTreeMap<String, [Option<String>; 2]>> {
        Some(&self.state.work.interaction.as_ref()?.patches)
    }

    pub fn interaction_drive_names(&self) -> &BTreeSet<String> {
        &self.compiled_drive_names
    }

    pub fn interaction_regions(&self) -> Option<&BTreeSet<String>> {
        Some(&self.state.work.interaction.as_ref()?.regions)
    }

    pub fn interaction_expects_response(&self) -> bool {
        self.state
            .work
            .interaction
            .as_ref()
            .is_some_and(|model| model.expects_response)
    }

    /// Запись воспроизведения пригодна для аудита, но не для нового доказательства.
    pub fn interaction_is_replay(&self) -> bool {
        self.state.replay.is_some()
            || crate::control::run(self)
                .is_some_and(|r| r.origin() == crate::control::RunOrigin::Replay)
    }
}

/// Имена физических величин, которые установщик может подключить к датчику.
pub fn patch_normal_signal(patch: &str) -> String {
    format!("interaction/{patch}/normal")
}

pub fn register_interaction(rt: &mut Runtime, model: ResolvedInteraction) -> Result<(), String> {
    rt.ensure_configurable()?;
    if rt.state.work.interaction.is_some() {
        return Err("взаимодействие уже установлено".into());
    }
    let bound = BoundInteraction::new(rt, model)?;
    let identity = InteractionIdentity {
        version: INTERACTION_VERSION,
        world: rt.world_model_hash,
        profile: bound.fingerprint,
        variant: bound.spec.variant,
        seed: rt.seed(),
        rate: rt.physics_rate(),
    };
    let patches = bound
        .spec
        .pads
        .iter()
        .map(|pad| {
            let spec = pad.model.spec();
            (
                spec.name.clone(),
                spec.geometries
                    .each_ref()
                    .map(|g| bound.regions.get(g).cloned()),
            )
        })
        .chain(
            bound
                .spec
                .routes
                .iter()
                .flat_map(|route| &route.surfaces)
                .map(|surface| {
                    (
                        surface.patch.clone(),
                        [None, bound.regions.get(&surface.geometry).cloned()],
                    )
                }),
        )
        .collect();
    let motion = format!("{:?}", bound.spec.motion);
    let expects_response = matches!(
        &bound.spec.motion,
        MotionProfile::Dynamic {
            response: Some(_),
            ..
        }
    );
    let regions = bound.spec.regions.keys().cloned().collect();
    let initial = bound.initial_joints.clone();
    let label = bound.spec.name.clone();
    rt.register_process(&label, bound)?;
    for (joint, position, velocity) in initial {
        rt.sim.qpos_mut()[joint.qpos_adr] = position;
        rt.sim.qvel_mut()[joint.dof_adr] = velocity;
    }
    rt.sim.forward();
    rt.state.work.interaction = Some(RegisteredInteraction {
        identity,
        patches,
        regions,
        motion,
        expects_response,
    });
    Ok(())
}

impl BoundInteraction {
    fn new(rt: &Runtime, spec: ResolvedInteraction) -> Result<Self, String> {
        let geometry_names: BTreeSet<_> =
            rt.sim.geoms().into_iter().map(|geom| geom.name).collect();
        let compiled = rt.sim.model();
        let movable_bodies = (1..rt.sim.nbody())
            .filter_map(|id| {
                // Жёсткий предок тела определяет, доступна ли ему виртуальная работа.
                let weld = unsafe { *compiled.body_weldid.add(id) } as usize;
                let count = unsafe { *compiled.body_dofnum.add(weld) };
                (weld != 0 && count != 0).then(|| rt.sim.name(mjtObj::mjOBJ_BODY, id))
            })
            .collect();
        let mut regions = BTreeMap::new();
        for (region, geometries) in &spec.regions {
            if !name(region) || geometries.is_empty() {
                return Err("участок требует имя и геометрии".into());
            }
            for geom in geometries {
                if !geometry_names.contains(geom)
                    || !name(geom)
                    || regions.insert(geom.clone(), region.clone()).is_some()
                {
                    return Err(format!(
                        "геометрия участка отсутствует или повторена: {geom}"
                    ));
                }
            }
        }
        let mut pairs = BTreeSet::new();
        let mut names = BTreeSet::new();
        for pad in &spec.pads {
            let p = pad.model.spec();
            if !names.insert(&p.name) || !pairs.insert(key(p.geometries.clone())) {
                return Err(format!("реакция пары назначена дважды: {:?}", p.geometries));
            }
            if !p.geometries.iter().any(|g| regions.contains_key(g)) {
                return Err(format!("у пары «{}» нет области воздействия", p.name));
            }
            for ((mount, body), geom) in pad.mounts.iter().zip(&p.bodies).zip(&p.geometries) {
                check_point(rt, &mount.point, body)?;
                let id = rt
                    .sim
                    .id(mjtObj::mjOBJ_GEOM, geom)
                    .ok_or("нет геометрии площадки")?;
                let body_id = rt
                    .sim
                    .id(mjtObj::mjOBJ_BODY, body)
                    .ok_or("нет тела площадки")?;
                // geom_bodyid индексируется действительным ID геометрии.
                if unsafe { *rt.sim.model().geom_bodyid.add(id) } as usize != body_id {
                    return Err(format!("геометрия «{geom}» принадлежит другому телу"));
                }
                check_basis(mount.normal, mount.tangent)?;
            }
            let geom = p.geometries.each_ref().map(String::as_str);
            let explicit = rt.sim.explicit_contact_settings(geom)?.is_some();
            let allowed = rt.sim.contact_pair_allowed(geom)?;
            match &p.owner {
                PadReactionOwner::Model(_) if explicit || allowed => {
                    return Err(format!("пара {geom:?} одновременно активна в движке"));
                }
                PadReactionOwner::Engine if !allowed => {
                    return Err(format!("пара {geom:?} исключена из контакта движка"));
                }
                PadReactionOwner::Engine if !pad.events.is_empty() => {
                    return Err(
                        "событие площадки движка требует физического переключения пары".into(),
                    );
                }
                _ => {}
            }
            check_events(&p.name, &pad.events, false)?;
        }
        let participant_bodies = regions
            .keys()
            .filter_map(|geom| {
                let id = rt.sim.id(mjtObj::mjOBJ_GEOM, geom)?;
                // geom_bodyid индексируется известной геометрией.
                Some(unsafe { *rt.sim.model().geom_bodyid.add(id) } as usize)
            })
            .collect::<BTreeSet<_>>();
        let mut axes = BTreeMap::new();
        let mut blockers = Vec::new();
        let mut surface_names = BTreeSet::new();
        for route in &spec.routes {
            let r = route.model.spec();
            if !names.insert(&r.name) || route.points.len() != r.bodies.len() {
                return Err(format!("повторное имя или неполная трасса «{}»", r.name));
            }
            for (point, body) in route.points.iter().zip(&r.bodies) {
                check_point(rt, point, body)?;
            }
            if let FreeLengthLaw::Affine { channel, .. } = r.length {
                axes.insert(channel, scalar_channel(rt, channel)?);
            }
            let mut covered_points = BTreeSet::new();
            for surface in &route.surfaces {
                if !name(&surface.patch)
                    || !surface_names.insert(&surface.patch)
                    || surface.point == surface.reference
                    || surface.reference >= route.points.len()
                    || !covered_points.insert(surface.point)
                    || !regions.contains_key(&surface.geometry)
                {
                    return Err("дублирующееся или неполное пятно трассы".into());
                }
                let point = route
                    .points
                    .get(surface.point)
                    .ok_or("нет точки пятна трассы")?;
                let geom = rt.sim.id(mjtObj::mjOBJ_GEOM, &surface.geometry).unwrap();
                // Связь геометрии с телом проверена по соответствующему ID.
                let body = unsafe { *rt.sim.model().geom_bodyid.add(geom) } as usize;
                if rt.sim.id(mjtObj::mjOBJ_BODY, &point.body) != Some(body) {
                    return Err("пятно трассы принадлежит другому телу".into());
                }
                check_basis(surface.normal, surface.tangent)?;
                if let Some(area) = &surface.area_m2 {
                    check_nonnegative(area)?;
                    if area.min <= 0.0 {
                        return Err("площадь пятна трассы должна быть положительной".into());
                    }
                }
            }
            for (index, point) in route.points.iter().enumerate() {
                let body = rt.sim.id(mjtObj::mjOBJ_BODY, &point.body).unwrap();
                if participant_bodies.contains(&body) && !covered_points.contains(&index) {
                    blockers.push(format!(
                        "у участка трассы «{}» нет модели распределения нагрузки",
                        r.name
                    ));
                }
            }
            check_events(&r.name, &route.events, true)?;
        }
        if surface_names.iter().any(|name| names.contains(name)) {
            return Err("пятно трассы конфликтует с именем другого интерфейса".into());
        }
        let mut initial_joints = Vec::new();
        match &spec.motion {
            MotionProfile::Dynamic {
                axes: motion,
                maneuver,
                response,
            } => {
                if motion.is_empty() {
                    blockers.push("нет двигательного профиля участника".into());
                }
                let mut channels = BTreeSet::new();
                let mut dofs = BTreeSet::new();
                for axis in motion {
                    let joint = scalar_channel(rt, axis.channel)?;
                    if !channels.insert(axis.channel) || !dofs.insert(joint.dof_adr) {
                        return Err("двигательное намерение повторно назначено на ось".into());
                    }
                    check_initial(&joint, axis.initial_position, axis.initial_velocity)?;
                    axis.reference.check()?;
                    check_trajectory_source(&axis.reference, &mut blockers);
                    for parameter in [&axis.stiffness, &axis.damping, &axis.effort_limit] {
                        check_nonnegative(parameter)?;
                        if !parameter.can_prove() || parameter.min != parameter.max {
                            blockers.push(format!(
                                "параметр движения не подтверждён во всём диапазоне: {:?}",
                                parameter.provenance
                            ));
                        }
                    }
                    initial_joints.push((
                        joint.clone(),
                        axis.initial_position,
                        axis.initial_velocity,
                    ));
                    axes.insert(axis.channel, joint);
                }
                if let Some(event) = maneuver {
                    check_offsets(&event.initiator, &event.offsets, &channels, &mut blockers)?;
                    for curve in event.offsets.values() {
                        due(event.at, curve.knots.last().unwrap().0.0)
                            .map_err(|e| e.to_string())?;
                    }
                }
                if let Some(resp) = response {
                    if !spec.pads.iter().any(|p| p.model.spec().name == resp.patch)
                        && !surface_names.contains(&resp.patch)
                    {
                        return Err("реакция ссылается на отсутствующее пятно".into());
                    }
                    check_offsets(&resp.initiator, &resp.offsets, &channels, &mut blockers)?;
                    check_nonnegative(&resp.threshold_n)?;
                    check_source(&resp.latency_source)?;
                    if !resp.threshold_n.can_prove()
                        || resp.threshold_n.min != resp.threshold_n.max
                        || !source_proves(&resp.latency_source)
                    {
                        blockers
                            .push("задержка или условие реакции имеют слабое обоснование".into());
                    }
                    if !resp.enabled {
                        blockers.push("обратная связь намеренно отключена".into());
                    }
                }
            }
            MotionProfile::FixedPose { joint, position } => {
                blockers.push("заданная поза не подтверждает двигательный отклик участника".into());
                rt.validate_scalar_hold(joint)?;
                let joint = rt.joints.iter().find(|j| j.name == *joint).unwrap();
                check_initial(joint, *position, 0.0)?;
                initial_joints.push((joint.clone(), *position, 0.0));
            }
        }
        let mut candidates = rt
            .sim
            .contact_candidates()?
            .into_iter()
            .map(key)
            .collect::<BTreeSet<_>>();
        // Явные пары MJCF могут обходить динамические маски и исключения.
        let compiled = rt.sim.model();
        for index in 0..compiled.npair as usize {
            // Оба массива содержат по одному действительному ID на явную пару.
            let ids = unsafe {
                [
                    *compiled.pair_geom1.add(index),
                    *compiled.pair_geom2.add(index),
                ]
            };
            candidates.insert(key(
                ids.map(|id| rt.sim.name(mjtObj::mjOBJ_GEOM, id as usize))
            ));
        }
        let contact_gaps = candidates
            .into_iter()
            .filter(|p| p.iter().any(|g| regions.contains_key(g)) && !pairs.contains(p))
            .map(|p| format!("нет владельца измерения пары {p:?}"))
            .chain(
                regions
                    .keys()
                    .filter(|geometry| {
                        !pairs.iter().any(|pair| pair.contains(geometry))
                            && !spec
                                .routes
                                .iter()
                                .flat_map(|route| &route.surfaces)
                                .any(|surface| surface.geometry == **geometry)
                    })
                    .map(|geometry| format!("нет измерения нагрузки поверхности «{geometry}»")),
            )
            .collect();
        Ok(Self {
            fingerprint: fnv1a(format!("{INTERACTION_VERSION}:{spec:?}").bytes()),
            spec,
            axes,
            initial_joints,
            regions,
            movable_bodies,
            contact_gaps,
            blockers,
            previous: BTreeMap::new(),
            previous_routes: BTreeMap::new(),
            previous_surfaces: BTreeMap::new(),
            contact_at: None,
            response_at: None,
            maneuver_at: None,
        })
    }

    fn pad(
        pad: &mut PadBinding,
        frame: &ProcessFrame<'_>,
        rng: &mut Rng,
    ) -> Result<(InterfaceStep, [f64; 2]), ProcessError> {
        let [first, second] = &pad.mounts;
        let a = mounted(frame, first)?;
        let b = mounted(frame, second)?;
        let tangent = rotate(a.rotation, first.tangent);
        let normal = rotate(a.rotation, first.normal);
        let bitangent = cross(normal, tangent);
        let offset = std::array::from_fn(|i| b.position_m[i] - a.position_m[i]);
        let drift = [dot(offset, tangent), dot(offset, bitangent)];
        finite(drift)?;
        let frame_for_pair = frame.completed.map(|finished| ContactFrame {
            at: finished.interval.end,
            contacts: finished
                .reactions
                .contacts
                .iter()
                .filter(|c| {
                    c.geometries == pad.model.spec().geometries
                        || c.geometries[0] == pad.model.spec().geometries[1]
                            && c.geometries[1] == pad.model.spec().geometries[0]
                })
                .cloned()
                .collect(),
        });
        let input = PadInput {
            mounts: [
                mount_motion(a, normal),
                mount_motion(b, rotate(b.rotation, second.normal)),
            ],
            tangent_a: tangent,
            completed: frame_for_pair,
            event: event_at(&pad.events, frame.next)?,
        };
        Ok((
            pad.model
                .advance(frame.next, &InterfaceInput::Pad(input), rng)?,
            drift,
        ))
    }

    fn route(
        axes: &BTreeMap<ChannelId, JointInfo>,
        route: &mut RouteBinding,
        frame: &ProcessFrame<'_>,
        rng: &mut Rng,
    ) -> Result<(InterfaceStep, Vec<BodyPoint>), ProcessError> {
        let points = route
            .points
            .iter()
            .map(|point| frame.body_point(&point.body, point.position_m))
            .collect::<Result<Vec<_>, _>>()?;
        let drive = match route.model.spec().length {
            FreeLengthLaw::Fixed(_) => None,
            FreeLengthLaw::Affine { channel, .. } => {
                let (coordinate, velocity) = axis_state(frame, &axes[&channel])?;
                Some(DriveMotion {
                    coordinate,
                    velocity,
                })
            }
        };
        let input = TensionInput {
            points: points
                .iter()
                .map(|p| RoutePoint {
                    point_m: p.position_m,
                    com_m: p.com_m,
                    velocity_m_s: p.velocity_m_s,
                })
                .collect(),
            drive,
            event: event_at(&route.events, frame.next)?,
        };
        let step = route
            .model
            .advance(frame.next, &InterfaceInput::Tension(input), rng)?;
        Ok((step, points))
    }

    fn motion(
        &mut self,
        frame: &ProcessFrame<'_>,
        tick: &mut InteractionTick,
        output: &mut ProcessOut,
    ) -> Result<(), ProcessError> {
        let MotionProfile::Dynamic {
            axes,
            maneuver,
            response,
        } = &self.spec.motion
        else {
            let MotionProfile::FixedPose { joint, .. } = &self.spec.motion else {
                unreachable!()
            };
            output.hold_scalar_axis(joint.clone());
            return Ok(());
        };
        let now = frame.next.start;
        if maneuver
            .as_ref()
            .is_some_and(|event| now < event.at && event.at < frame.next.end)
        {
            return Err(bad("манёвр сценария не совпадает с границей физики"));
        }
        if let Some(event) = maneuver
            && now >= event.at
            && self.maneuver_at.is_none()
        {
            self.maneuver_at = Some(now);
            tick.motion_events.push(motion_event(
                &event.initiator,
                &event.offsets,
                None,
                event.at,
                frame.next,
            ));
        }
        if let Some(resp) = response {
            if resp.enabled
                && self.contact_at.is_none()
                && let Some(sample) = tick.patches.get(&resp.patch)
                && let Analysis::Analyzed { value, .. } = &sample.reaction
                && value.normal_n > 0.0
                && value.normal_n >= resp.threshold_n.nominal
            {
                self.contact_at = Some(sample.interval.end);
            }
            if let Some(contact) = self.contact_at {
                let requested = due(contact, resp.latency_ns)?;
                if now >= requested && self.response_at.is_none() {
                    for curve in resp.offsets.values() {
                        due(now, curve.knots.last().unwrap().0.0)?;
                    }
                    self.response_at = Some(now);
                    let mut event = motion_event(
                        &resp.initiator,
                        &resp.offsets,
                        Some(contact),
                        requested,
                        frame.next,
                    );
                    event.sources.extend([
                        resp.latency_source.clone(),
                        resp.threshold_n.provenance.clone(),
                    ]);
                    tick.motion_events.push(event);
                }
            }
        }
        for axis in axes {
            let (mut target, mut rate) = axis.reference.sample(now);
            if let Some((event, at)) = maneuver.as_ref().zip(self.maneuver_at)
                && let Some(curve) = event.offsets.get(&axis.channel)
            {
                let (q, v) = curve.sample(Time(now.0 - at.0));
                target += q;
                rate += v;
            }
            let (q, v) = axis_state(frame, &self.axes[&axis.channel])?;
            let cap = axis.effort_limit.min;
            let effort_at = |target, rate| {
                let effort =
                    axis.stiffness.nominal * (target - q) + axis.damping.nominal * (rate - v);
                finite([target, rate, effort])?;
                Ok::<_, ProcessError>(effort.clamp(-cap, cap))
            };
            let without_response = effort_at(target, rate)?;
            let applied_effort = if let Some((event, at)) = response.as_ref().zip(self.response_at)
                && let Some(curve) = event.offsets.get(&axis.channel)
            {
                let (q, v) = curve.sample(Time(now.0 - at.0));
                target += q;
                rate += v;
                effort_at(target, rate)?
            } else {
                without_response
            };
            let response_effort_delta = applied_effort - without_response;
            finite([response_effort_delta])?;
            output.efforts.push((axis.channel, applied_effort));
            tick.axes.push(AxisSample {
                channel: axis.channel,
                position: q,
                velocity: v,
                reference: target,
                reference_velocity: rate,
                applied_effort,
                response_effort_delta,
                interval: frame.next,
            });
        }
        Ok(())
    }
}

impl ProcessModel for BoundInteraction {
    fn update(
        &mut self,
        frame: &ProcessFrame<'_>,
        rng: &mut Rng,
    ) -> Result<ProcessOut, ProcessError> {
        let mut output = ProcessOut::default();
        let mut tick = InteractionTick {
            coverage_gaps: self.contact_gaps.clone(),
            ..Default::default()
        };
        output.blockers.clone_from(&self.blockers);
        for pad in &mut self.spec.pads {
            let (step, drift) = Self::pad(pad, frame, rng)?;
            let spec = pad.model.spec();
            let owner = match spec.owner {
                PadReactionOwner::Engine => ReactionOwner::Engine,
                PadReactionOwner::Model(_) => ReactionOwner::Model,
            };
            if let Some(completed) = frame.completed {
                let (measured, offset_m) = match owner {
                    ReactionOwner::Engine => (&step, drift),
                    ReactionOwner::Model => {
                        let (previous, offset) = self
                            .previous
                            .get(&spec.name)
                            .ok_or_else(|| bad("нет закоммиченной нагрузки площадки"))?;
                        if previous.interval != completed.interval {
                            return Err(bad("нагрузка площадки относится к другому интервалу"));
                        }
                        (previous, *offset)
                    }
                };
                let reaction = measured.reaction.clone();
                let calibrated = matches!(measured.calibration, Analysis::Analyzed { .. })
                    && (owner == ReactionOwner::Model
                        || frame_has_calibration(frame, &spec.geometries));
                let sample = PatchSample {
                    regions: spec
                        .geometries
                        .each_ref()
                        .map(|g| self.regions.get(g).cloned()),
                    interval: completed.interval,
                    owner,
                    reaction,
                    calibrated,
                    area_m2: if calibrated {
                        measured.effective_area_m2.clone()
                    } else {
                        None
                    },
                    offset_m,
                };
                record_patch(&mut output, &mut tick, &spec.name, sample)?;
            }
            if owner == ReactionOwner::Model {
                self.previous
                    .insert(spec.name.clone(), (step.clone(), drift));
            }
            record_interface(&mut output, &mut tick, &spec.name, step);
        }
        for route in &mut self.spec.routes {
            let (step, points) = Self::route(&self.axes, route, frame, rng)?;
            let spec = route.model.spec();
            if let Some(completed) = frame.completed {
                let prior = self
                    .previous_routes
                    .get(&spec.name)
                    .ok_or_else(|| bad("нет закоммиченной нагрузки трассы"))?;
                if prior.interval != completed.interval {
                    return Err(bad("натяжение относится к другому интервалу"));
                }
                tick.completed_routes
                    .insert(spec.name.clone(), prior.clone());
                for surface in &route.surfaces {
                    let prior = self
                        .previous_surfaces
                        .get(&surface.patch)
                        .ok_or_else(|| bad("нет закоммиченного пятна трассы"))?;
                    record_patch(&mut output, &mut tick, &surface.patch, prior.clone())?;
                }
            }
            for surface in &route.surfaces {
                let sample = route_patch(surface, &self.regions, &route.points, &points, &step)?;
                self.previous_surfaces.insert(surface.patch.clone(), sample);
            }
            self.previous_routes.insert(spec.name.clone(), step.clone());
            record_interface(&mut output, &mut tick, &spec.name, step);
        }
        if let Err(error) = self.motion(frame, &mut tick, &mut output) {
            output.stop_after_record = Some(error.to_string());
        }
        if let Err(error) = combine_effects(&mut output, &self.movable_bodies) {
            output.stop_after_record = Some(error.to_string());
        }
        if output.stop_after_record.is_some() {
            output.body_loads.clear();
            output.efforts.clear();
        }
        output.interaction = Some(tick);
        Ok(output)
    }

    fn state_fingerprint(&self) -> u64 {
        fnv1a(
            format!(
                "{:016x}{:?}{:?}{:?}{:?}{:?}{:?}{:?}{:?}",
                self.fingerprint,
                self.spec.pads,
                self.spec.routes,
                self.previous,
                self.previous_routes,
                self.previous_surfaces,
                self.contact_at,
                self.response_at,
                self.maneuver_at
            )
            .bytes(),
        )
    }
}

fn combine_effects(
    out: &mut ProcessOut,
    movable_bodies: &BTreeSet<String>,
) -> Result<(), ProcessError> {
    let mut loads = BTreeMap::<String, BodyLoad>::new();
    for load in std::mem::take(&mut out.body_loads) {
        // Опора остаётся в InterfaceStep, её виртуальная работа равна нулю.
        if !movable_bodies.contains(&load.body) {
            continue;
        }
        let sum = loads.entry(load.body.clone()).or_insert_with(|| BodyLoad {
            body: load.body,
            force_n: [0.0; 3],
            torque_nm: [0.0; 3],
        });
        for (total, component) in sum
            .force_n
            .iter_mut()
            .chain(&mut sum.torque_nm)
            .zip(load.force_n.into_iter().chain(load.torque_nm))
        {
            *total += component;
        }
        finite(sum.force_n.into_iter().chain(sum.torque_nm))?;
    }
    let mut efforts = BTreeMap::<ChannelId, f64>::new();
    for (channel, delta) in std::mem::take(&mut out.efforts) {
        let effort = efforts.entry(channel).or_default();
        *effort += delta;
        finite([*effort])?;
    }
    out.body_loads = loads.into_values().collect();
    out.efforts = efforts.into_iter().collect();
    Ok(())
}

fn route_patch(
    surface: &RouteSurface,
    regions: &BTreeMap<String, String>,
    bindings: &[LocalPoint],
    points: &[BodyPoint],
    step: &InterfaceStep,
) -> Result<PatchSample, ProcessError> {
    let p = points[surface.point];
    let reference = points[surface.reference];
    let normal = rotate(p.rotation, surface.normal);
    let tangent = rotate(p.rotation, surface.tangent);
    let delta = std::array::from_fn(|i| p.position_m[i] - reference.position_m[i]);
    let offset_m = [dot(delta, tangent), dot(delta, cross(normal, tangent))];
    finite(offset_m)?;
    let mut compressed = true;
    let reaction = match &step.reaction {
        Analysis::Analyzed { scope, value } => {
            // Закон B возвращает массив только для ненулевого натяжения.
            let force = step
                .route_forces_n
                .get(surface.point)
                .copied()
                .unwrap_or([0.0; 3]);
            let signed = dot(force, normal);
            compressed = signed >= 0.0;
            let shear_n = std::array::from_fn(|i| force[i] - signed * normal[i]);
            let shear = shear_n[0].hypot(shear_n[1]).hypot(shear_n[2]);
            let torque_nm = cross(std::array::from_fn(|i| p.position_m[i] - p.com_m[i]), force);
            finite([signed, shear].into_iter().chain(torque_nm))?;
            Analysis::Analyzed {
                scope: scope.clone(),
                value: InterfaceReaction {
                    loads: vec![BodyLoad {
                        body: bindings[surface.point].body.clone(),
                        force_n: force,
                        torque_nm,
                    }],
                    normal_n: signed.max(0.0),
                    shear_n,
                    shear_upper_n: shear,
                    peak_shear_n: shear,
                    tension_n: value.tension_n,
                    contact_count: usize::from(signed > 0.0 || shear > 0.0),
                },
            }
        }
        Analysis::NotAnalyzed { reason } => Analysis::NotAnalyzed {
            reason: reason.clone(),
        },
    };
    Ok(PatchSample {
        regions: [None, regions.get(&surface.geometry).cloned()],
        interval: step.interval,
        owner: ReactionOwner::Model,
        reaction,
        calibrated: compressed && matches!(step.calibration, Analysis::Analyzed { .. }),
        // Точечный расчёт трассы не содержит площади её контакта.
        area_m2: None,
        offset_m,
    })
}

fn record_patch(
    out: &mut ProcessOut,
    tick: &mut InteractionTick,
    name: &str,
    sample: PatchSample,
) -> Result<(), ProcessError> {
    out.measure(
        sample.interval.end,
        patch_normal_signal(name),
        "N",
        match &sample.reaction {
            Analysis::Analyzed { value, scope } => Analysis::Analyzed {
                value: value.normal_n,
                scope: scope.clone(),
            },
            Analysis::NotAnalyzed { reason } => Analysis::NotAnalyzed {
                reason: reason.clone(),
            },
        },
    )?;
    tick.patches.insert(name.into(), sample);
    Ok(())
}

fn record_interface(
    out: &mut ProcessOut,
    tick: &mut InteractionTick,
    name: &str,
    step: InterfaceStep,
) {
    if matches!(step.state, InterfaceState::Unavailable) {
        out.stop_after_record = Some(format!(
            "утрачен закон реакции «{name}»: {:?}",
            step.reaction
        ));
    }
    if matches!(&step.work, Analysis::Analyzed { value, .. } if value.released_energy_j > 0.0) {
        out.blockers.push(format!(
            "распределение высвобождённой энергии «{name}» не рассчитано"
        ));
    }
    out.body_loads.extend(step.body_loads.iter().cloned());
    out.efforts.extend(step.drive_effort);
    tick.interfaces.insert(name.into(), step);
}

fn frame_has_calibration(frame: &ProcessFrame<'_>, geometries: &[String; 2]) -> bool {
    let pair = key(geometries.clone());
    frame
        .contact_calibrations
        .and_then(|pairs| pairs.get(&pair))
        .is_some_and(|entry| {
            matches!(entry.nominal_coverage, Analysis::Analyzed { .. })
                && frame.completed.is_some_and(|completed| {
                    completed
                        .engine_step
                        .contacts
                        .iter()
                        .filter(|c| key(c.geometries.clone()) == pair)
                        .all(|c| c.matches_settings(entry.pair.settings))
                })
        })
}

fn motion_event(
    initiator: &str,
    offsets: &BTreeMap<ChannelId, Trajectory>,
    contact_at: Option<Time>,
    due: Time,
    interval: Interval,
) -> MotionRecord {
    MotionRecord {
        initiator: initiator.into(),
        sources: offsets.values().map(|t| t.source.clone()).collect(),
        contact_at,
        due,
        interval,
    }
}

fn check_initial(joint: &JointInfo, position: f64, velocity: f64) -> Result<(), String> {
    finite([position, velocity]).map_err(|e| e.to_string())?;
    if joint.limited && !(joint.range[0]..=joint.range[1]).contains(&position) {
        return Err(format!(
            "начальная координата вне диапазона оси «{}»",
            joint.name
        ));
    }
    Ok(())
}

fn check_offsets(
    initiator: &str,
    offsets: &BTreeMap<ChannelId, Trajectory>,
    channels: &BTreeSet<ChannelId>,
    blockers: &mut Vec<String>,
) -> Result<(), String> {
    if !name(initiator) || offsets.is_empty() || offsets.keys().any(|c| !channels.contains(c)) {
        return Err("событие требует инициатора и привязку к осям движения".into());
    }
    for curve in offsets.values() {
        curve.check()?;
        if curve.knots[0].1 != 0.0 || curve.knots.len() < 2 {
            return Err("отклонение начинается с нуля и требует заданной длительности".into());
        }
        check_trajectory_source(curve, blockers);
    }
    Ok(())
}

fn check_trajectory_source(curve: &Trajectory, blockers: &mut Vec<String>) {
    if !source_proves(&curve.source) {
        blockers.push(format!(
            "траектория задана слабо обоснованным источником: {:?}",
            curve.source
        ));
    }
}

fn check_source(source: &Provenance) -> Result<(), String> {
    let description = match source {
        Provenance::Datasheet(s)
        | Provenance::Paper(s)
        | Provenance::Measured(s)
        | Provenance::Derived(s)
        | Provenance::Estimate(s) => s,
    };
    name(description)
        .then_some(())
        .ok_or_else(|| "нет источника профиля".into())
}

fn source_proves(source: &Provenance) -> bool {
    matches!(
        source,
        Provenance::Measured(_) | Provenance::Paper(_) | Provenance::Datasheet(_)
    )
}

fn check_nonnegative(range: &SourcedRange) -> Result<(), String> {
    range.validate().map_err(|e| e.to_string())?;
    (range.min >= 0.0)
        .then_some(())
        .ok_or_else(|| "отрицательный параметр движения".into())
}

fn check_events(name: &str, events: &[ConfirmedInterfaceEvent], slip: bool) -> Result<(), String> {
    if events.windows(2).any(|w| w[0].at >= w[1].at)
        || events.iter().any(|e| {
            e.name != name
                || match e.kind {
                    ConfirmedEvent::Slip { payout_m } => {
                        !slip || !payout_m.is_finite() || payout_m <= 0.0
                    }
                    _ => false,
                }
        })
    {
        return Err(format!(
            "неверные времена, привязки или значения событий «{name}»"
        ));
    }
    Ok(())
}

fn event_at(
    events: &[ConfirmedInterfaceEvent],
    next: Interval,
) -> Result<Option<ConfirmedInterfaceEvent>, ProcessError> {
    let index = events.partition_point(|e| e.at < next.start);
    let Some(event) = events.get(index) else {
        return Ok(None);
    };
    if event.at > next.start && event.at < next.end {
        return Err(bad("событие интерфейса не совпадает с границей физики"));
    }
    Ok((event.at == next.start).then(|| event.clone()))
}

fn check_point(rt: &Runtime, point: &LocalPoint, body: &str) -> Result<(), String> {
    if point.body != body || rt.sim.id(mjtObj::mjOBJ_BODY, body).is_none() {
        return Err(format!("точка не привязана к телу «{body}»"));
    }
    finite(point.position_m).map_err(|e| e.to_string())
}

fn check_basis(normal: Vector, tangent: Vector) -> Result<(), String> {
    finite(normal.into_iter().chain(tangent)).map_err(|e| e.to_string())?;
    // Погрешность скалярных произведений трёхкомпонентного базиса.
    let tolerance = 32.0 * f64::EPSILON;
    if (dot(normal, normal) - 1.0).abs() > tolerance
        || (dot(tangent, tangent) - 1.0).abs() > tolerance
        || dot(normal, tangent).abs() > tolerance
    {
        return Err("локальные оси точки не образуют ортонормированный базис".into());
    }
    Ok(())
}

fn scalar_channel(rt: &Runtime, channel: ChannelId) -> Result<JointInfo, String> {
    let dof = rt
        .effort_dofs
        .get(channel.0 as usize)
        .ok_or("нет канала усилия")?;
    rt.joints
        .iter()
        .find(|j| j.dof_adr == *dof && matches!(j.kind, JointKind::Slide | JointKind::Hinge))
        .cloned()
        .ok_or_else(|| "траектория или трасса требует скалярную ось".into())
}

fn axis_state(frame: &ProcessFrame<'_>, joint: &JointInfo) -> Result<(f64, f64), ProcessError> {
    let q = frame.truth.qpos()[joint.qpos_adr];
    let v = frame.truth.qvel()[joint.dof_adr];
    finite([q, v])?;
    Ok((q, v))
}

fn mounted(frame: &ProcessFrame<'_>, mount: &LocalMount) -> Result<BodyPoint, ProcessError> {
    frame.body_point(&mount.point.body, mount.point.position_m)
}

fn mount_motion(p: BodyPoint, axis: Vector) -> MountMotion {
    MountMotion {
        point_m: p.position_m,
        com_m: p.com_m,
        axis,
        velocity_m_s: p.velocity_m_s,
        angular_velocity_rad_s: p.angular_velocity_rad_s,
    }
}

fn rotate(r: [Vector; 3], v: Vector) -> Vector {
    r.map(|row| dot(row, v))
}

fn dot(a: Vector, b: Vector) -> f64 {
    a.into_iter().zip(b).map(|(x, y)| x * y).sum()
}

fn cross(a: Vector, b: Vector) -> Vector {
    std::array::from_fn(|i| a[(i + 1) % 3] * b[(i + 2) % 3] - a[(i + 2) % 3] * b[(i + 1) % 3])
}

fn finite(values: impl IntoIterator<Item = f64>) -> Result<(), ProcessError> {
    values
        .into_iter()
        .all(f64::is_finite)
        .then_some(())
        .ok_or_else(|| bad("неконечное состояние взаимодействия"))
}

fn due(at: Time, delay: u64) -> Result<Time, ProcessError> {
    at.0.checked_add(delay)
        .map(Time)
        .ok_or_else(|| bad("переполнение времени реакции"))
}

fn bad(reason: &str) -> ProcessError {
    ProcessError::Execution(reason.into())
}
