use std::{collections::BTreeMap, path::Path};

use serde::Deserialize;
use sim_process::{
    Analysis, BodyLoad, ProcessError, SourcedRange,
    attachments::{
        AdhesiveLaw, AttachmentAction, AttachmentBinding, AttachmentCause, AttachmentInput,
        AttachmentKind, AttachmentLaw, AttachmentModel, AttachmentRequest, AttachmentState,
        CaptureLimits, CureObservation, JointWrench, LatchLaw, MountMotion, MountPort, ThreadLaw,
        ThreadMotion, WeldResidual, WrenchCapacity,
    },
    work::{HazardCheck, HazardEvidence, HazardKind, mechanical_limits::upper_check},
};
use sim_runtime::{
    Runtime,
    process::{AttachmentAdapter, ProcessFrame, ProcessHazard, ProcessModel, ProcessOut},
};
use sim_signal::{Rng, Time};
use sim_world::{AttachmentMode, CompiledJointKind, CompiledWorld, Mount, World};

use super::{
    Knot, Scalar, Supported, cross, curve, dot, missing, nonnegative, norm, same_pair,
    sourced_config, unavailable, validated_range,
};
use crate::tasks::processes::models::{axis, invalid, joint, measured, validate_pair};

sourced_config!(Capacity => WrenchCapacity { force_n, torque_nm });
sourced_config!(Latch => LatchLaw {
    free_seat_m, engaged_seat_m, engagement_m, release_m, minimum_stroke_m, maximum_stroke_m,
    stiffness_n_m, damping_ns_m, forward_contact_n, reverse_contact_n,
});
sourced_config!(Thread => ThreadLaw {
    lead_m_per_turn, entry_m, depth_m, phase_error_rad, maximum_elastic_error_m,
    stiffness_n_m, damping_ns_m, torsional_drag_nms,
});
sourced_config!(Adhesive => AdhesiveLaw { cure_s, compression_n, temperature_k });

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Limits {
    gap_m: Option<Scalar>,
    lateral_m: Option<Scalar>,
    axis_error_rad: Option<Scalar>,
    speed_m_s: Option<Scalar>,
    angular_speed_rad_s: Option<Scalar>,
    required_travel_m: Option<Scalar>,
    release_force_n: Option<Scalar>,
    release_torque_nm: Option<Scalar>,
    capacity: Option<Capacity>,
}

impl Limits {
    fn build(self) -> Option<CaptureLimits> {
        Some(CaptureLimits {
            gap_m: self.gap_m?.into(),
            lateral_m: self.lateral_m?.into(),
            axis_error_rad: self.axis_error_rad?.into(),
            speed_m_s: self.speed_m_s?.into(),
            angular_speed_rad_s: self.angular_speed_rad_s?.into(),
            required_travel_m: self.required_travel_m?.into(),
            release_force_n: self.release_force_n?.into(),
            release_torque_nm: self.release_torque_nm?.into(),
            capacity: self.capacity?.build()?,
        })
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
enum Law {
    Magnetic {
        pull: Option<Vec<Knot>>,
    },
    Latch {
        parameters: Option<Latch>,
        drive_pair: Option<[String; 2]>,
    },
    Threaded {
        parameters: Option<Thread>,
        angular_joint: Option<String>,
        phase_at_entry_rad: Option<Scalar>,
    },
    Adhesive {
        parameters: Option<Adhesive>,
        weld: Option<String>,
        cure_observation: Option<Cure>,
    },
}

impl Law {
    fn kind(&self) -> AttachmentKind {
        match self {
            Self::Magnetic { .. } => AttachmentKind::Magnetic,
            Self::Latch { .. } => AttachmentKind::Latch,
            Self::Threaded { .. } => AttachmentKind::Threaded,
            Self::Adhesive { .. } => AttachmentKind::Adhesive,
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Cure {
    compression_n: Option<Scalar>,
    temperature_k: Option<Scalar>,
    gap_m: Option<Scalar>,
    lateral_m: Option<Scalar>,
    axis_error_rad: Option<Scalar>,
    speed_m_s: Option<Scalar>,
    angular_speed_rad_s: Option<Scalar>,
}

impl Cure {
    fn build(self) -> Result<Option<CureObservation>, ProcessError> {
        let [p, t, gap, lateral, angle, speed, angular] = [
            self.compression_n,
            self.temperature_k,
            self.gap_m,
            self.lateral_m,
            self.axis_error_rad,
            self.speed_m_s,
            self.angular_speed_rad_s,
        ]
        .map(|v| v.map(nonnegative).transpose());
        let (Some(p), Some(t), Some(gap), Some(lateral), Some(angle), Some(speed), Some(angular)) =
            (p?, t?, gap?, lateral?, angle?, speed?, angular?)
        else {
            return Ok(None);
        };
        if t.min <= 0.0 {
            return Err(invalid("температура отверждения должна быть положительной"));
        }
        Ok(Some(CureObservation {
            compression_n: [p.min, p.max],
            temperature_k: [t.min, t.max],
            gap_m: gap.max,
            lateral_m: lateral.max,
            axis_error_rad: angle.max,
            speed_m_s: speed.max,
            angular_speed_rad_s: angular.max,
        }))
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Load {
    at_ns: u64,
    force_n: [f64; 3],
    torque_nm: [f64; 3],
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Loading {
    source: String,
    steps: Vec<Load>,
}

impl Loading {
    fn schedule(self) -> Result<BTreeMap<Time, JointWrench>, String> {
        if self.source.trim().is_empty() || self.source.chars().any(char::is_control) {
            return Err("нужен источник расписания внешней нагрузки".into());
        }
        let mut values = BTreeMap::new();
        for item in self.steps {
            let value = JointWrench {
                force_n: item.force_n,
                torque_nm: item.torque_nm,
            };
            if !item
                .force_n
                .into_iter()
                .chain(item.torque_nm)
                .all(f64::is_finite)
                || values.insert(Time(item.at_ns), value).is_some()
            {
                return Err("невалидная точка расписания внешней нагрузки".into());
            }
        }
        if values
            .first_key_value()
            .is_none_or(|(at, _)| *at != Time::ZERO)
        {
            return Err("нужна исходная внешняя нагрузка".into());
        }
        Ok(values)
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Action {
    Attach,
    Release,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Request {
    at_ns: u64,
    action: Action,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(in crate::tasks::processes) struct Attachment {
    mounts: Option<[String; 2]>,
    limits: Option<Limits>,
    law: Option<Law>,
    available_travel_m: Option<Scalar>,
    loading: Option<Loading>,
    requests: Option<Vec<Request>>,
    witness_limit: Option<Scalar>,
}

#[derive(Clone)]
struct LocatedMount {
    port: MountPort,
    local_m: [f64; 3],
    rotation: [[f64; 3]; 3],
}

impl LocatedMount {
    fn new(body: String, mount: Mount) -> Self {
        let pose = mount.pose;
        // Соглашение Pose в sim-world: внешние повороты X, Y, Z или кватернион wxyz.
        let rotation = if let Some(q) = pose.quat_wxyz {
            let scale = q.into_iter().map(f64::abs).fold(0.0, f64::max);
            let q = q.map(|v| v / scale);
            let [w, x, y, z] = q.map(|v| v / q[0].hypot(q[1]).hypot(q[2]).hypot(q[3]));
            // Матрица нормированного кватерниона. Двойки следуют из q v q⁻¹.
            [
                [
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - w * z),
                    2.0 * (x * z + w * y),
                ],
                [
                    2.0 * (x * y + w * z),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - w * x),
                ],
                [
                    2.0 * (x * z - w * y),
                    2.0 * (y * z + w * x),
                    1.0 - 2.0 * (x * x + y * y),
                ],
            ]
        } else {
            let [(sx, cx), (sy, cy), (sz, cz)] = pose
                .rpy_deg
                .unwrap_or([0.0; 3])
                .map(|angle| angle.to_radians().sin_cos());
            [
                [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
                [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
                [-sy, cy * sx, cy * cx],
            ]
        };
        Self {
            port: MountPort {
                name: format!("{body}/{}", mount.name),
                body,
                kind: mount.kind,
                interface: mount.interface,
            },
            local_m: pose.position_m,
            rotation,
        }
    }

    fn observe(
        &self,
        frame: &ProcessFrame<'_>,
    ) -> Result<(MountMotion, [[f64; 3]; 3]), ProcessError> {
        let point = frame.body_point(&self.port.body, self.local_m)?;
        let rotation = std::array::from_fn(|i| {
            std::array::from_fn(|j| dot(point.rotation[i], self.rotation.map(|row| row[j])))
        });
        // Локальная +X определяет направление положительного хода стыка.
        let axial = rotation.map(|row| row[0]);
        let length = norm(axial);
        Ok((
            MountMotion {
                point_m: point.position_m,
                com_m: point.com_m,
                axis: axial.map(|x| x / length),
                velocity_m_s: point.velocity_m_s,
                angular_velocity_rad_s: point.angular_velocity_rad_s,
            },
            rotation,
        ))
    }
}

fn locate(name: &str, scene: &World, base: &Path) -> Result<LocatedMount, String> {
    let (body, port) = name.rsplit_once('/').ok_or("нужно полное имя стыка")?;
    let (instance, link) = body.split_once('/').ok_or("нужно полное имя тела стыка")?;
    let mount = if let Some(p) = scene.devices.iter().find(|p| p.name == instance) {
        let device = sim_world::load_device(&base.join(&p.file), &sim_world::LegacyCatalog)?;
        device
            .links
            .into_iter()
            .find(|l| l.name == link)
            .and_then(|l| l.mounts.into_iter().find(|m| m.name == port))
    } else {
        scene
            .objects
            .iter()
            .find(|o| o.name == instance && o.name == link)
            .and_then(|o| o.mounts.iter().find(|m| m.name == port))
            .cloned()
    };
    mount
        .map(|m| LocatedMount::new(body.to_owned(), m))
        .ok_or_else(|| format!("нет стыка «{name}»"))
}

#[derive(Clone)]
struct Inputs {
    mounts: [LocatedMount; 2],
    loads: BTreeMap<Time, JointWrench>,
    travel_m: f64,
    gravity_force_n: [f64; 3],
    drive_pair: Option<[String; 2]>,
    thread: Option<(String, usize, f64)>,
    cure: Option<CureObservation>,
    equality: Option<String>,
}

impl Inputs {
    fn prescribed(&self, at: Time) -> JointWrench {
        *self
            .loads
            .range(..=at)
            .next_back()
            .expect("есть начальная нагрузка")
            .1
    }

    fn observe(&self, frame: &ProcessFrame<'_>) -> Result<AttachmentInput, ProcessError> {
        let previous = frame
            .completed
            .ok_or_else(|| invalid("нет завершённого шага"))?;
        let [(a, ra), (b, rb)] = [
            self.mounts[0].observe(frame)?,
            self.mounts[1].observe(frame)?,
        ];
        let is_active = self
            .equality
            .as_ref()
            .map(|id| frame.truth.engine().equality_active(id))
            .transpose()
            .map_err(invalid)?
            .unwrap_or(false);
        let wrench = if is_active {
            self.equality
                .as_ref()
                .and_then(|id| previous.weld_reactions.get(id))
                .and_then(|r| *r)
                .map(|reaction| JointWrench {
                    force_n: reaction.on_second.force_n,
                    torque_nm: reaction.on_second.torque_nm,
                })
        } else if previous.contact_gaps.is_empty() {
            let mut external = self.prescribed(previous.interval.start);
            let gravity_moment = cross(difference(b.com_m, b.point_m), self.gravity_force_n);
            for i in 0..3 {
                external.force_n[i] += self.gravity_force_n[i];
                external.torque_nm[i] += gravity_moment[i];
            }
            for c in &previous.reactions.contacts {
                let body = &self.mounts[1].port.body;
                let on_body = |g: &str| {
                    g.split_once("/collision/")
                        .is_some_and(|(owner, _)| owner == body)
                };
                let sign = f64::from(u8::from(on_body(&c.geometries[1])))
                    - f64::from(u8::from(on_body(&c.geometries[0])));
                if sign != 0.0 {
                    let moment = cross(difference(c.point_m, b.point_m), c.force_n);
                    for i in 0..3 {
                        external.force_n[i] += sign * c.force_n[i];
                        external.torque_nm[i] += sign * (moment[i] + c.torque_nm[i]);
                    }
                }
            }
            Some(external)
        } else {
            None
        };
        let latch_contact_n = self
            .drive_pair
            .as_ref()
            .filter(|_| previous.contact_gaps.is_empty())
            .map(|pair| {
                let force = previous
                    .reactions
                    .contacts
                    .iter()
                    .filter(|c| c.separation_m <= 0.0 && same_pair(&c.geometries, pair))
                    .map(|c| {
                        // В ContactSample сила приложена ко второй геометрии; принимающая указана второй в паре привода.
                        let projected = dot(c.force_n, a.axis);
                        if c.geometries[1] == pair[1] {
                            projected
                        } else {
                            -projected
                        }
                    })
                    .sum();
                measured(
                    force,
                    "проекция реакции приводящей контактной пары на ось стыка",
                )
            })
            .unwrap_or_else(|| unavailable("нет подтверждённой приводящей пары"));
        let thread = if let Some((name, id, phase)) = &self.thread {
            let (q, v) = axis(frame, name)?;
            let alignment = dot(frame.truth.engine().joint_axis(*id), a.axis);
            let speed = v * alignment.signum();
            let observed = dot(
                difference(b.angular_velocity_rad_s, a.angular_velocity_rad_s),
                a.axis,
            );
            // Допуск округления для двух проекций трёхмерных векторов.
            let rounding = 32.0 * f64::EPSILON;
            ((alignment.abs() - 1.0).abs() <= rounding
                && (observed - speed).abs() <= rounding * observed.abs().max(speed.abs()).max(1.0))
            .then_some(ThreadMotion {
                angle_rad: (q - phase) * alignment.signum(),
                angular_speed_rad_s: speed,
            })
        } else {
            None
        };
        let weld_residual = self.equality.as_ref().map(|_| {
            let squared = ra
                .into_iter()
                .flatten()
                .zip(rb.into_iter().flatten())
                .map(|(x, y)| (x - y).powi(2))
                .sum::<f64>();
            // ||R₁−R₂||²_F = 8 sin²(θ/2) для двух матриц поворота.
            WeldResidual {
                translation_m: norm(difference(b.point_m, a.point_m)),
                rotation_rad: 2.0 * (squared / 8.0).sqrt().min(1.0).asin(),
            }
        });
        Ok(AttachmentInput {
            mounts: [a, b],
            available_travel_m: measured(self.travel_m, "нижняя граница доступного хода из данных"),
            measured_wrench: wrench
                .map(|value| Analysis::Analyzed {
                    value,
                    scope: "реакция weld или внешняя нагрузка стенда, вес и контакты".into(),
                })
                .unwrap_or_else(|| {
                    unavailable("нет подтверждённой внешней нагрузки или реакции weld")
                }),
            latch_contact_n,
            thread,
            cure: self
                .cure
                .map(|value| Analysis::Analyzed {
                    value,
                    scope: "заданные границы наблюдения за всем интервалом отверждения".into(),
                })
                .unwrap_or_else(|| unavailable("нет наблюдения за отверждением")),
            weld_residual,
            command: None,
        })
    }
}

fn difference(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] - b[i])
}

#[derive(Clone)]
struct Output<I> {
    adapter: AttachmentAdapter<I>,
    input: Inputs,
    thread: Option<ThreadLaw>,
    load_audit: Option<LoadAudit>,
}

#[derive(Clone)]
enum LoadKind {
    Holding,
    Capacity(WrenchCapacity),
}

#[derive(Clone)]
struct LoadAudit {
    kind: LoadKind,
    limit: SourcedRange,
    sources: Vec<SourcedRange>,
    peak: Option<f64>,
    transition: bool,
}

impl LoadAudit {
    fn new(law: &AttachmentLaw, capacity: &WrenchCapacity, upper: Scalar) -> Result<Self, String> {
        let limit = nonnegative(upper).map_err(|e| e.to_string())?;
        let (kind, mut sources, bound) = match law {
            AttachmentLaw::Magnetic { pull } => (
                LoadKind::Holding,
                pull.points()
                    .iter()
                    .flat_map(|point| [&point.displacement_m, &point.force_n])
                    .cloned()
                    .collect(),
                0.0, // Условие равновесия: тянущая сила минус сила удержания.
            ),
            AttachmentLaw::Adhesive(_) => (
                LoadKind::Capacity(capacity.clone()),
                vec![capacity.force_n.clone(), capacity.torque_nm.clone()],
                1.0, // Нормированная огибающая WrenchCapacity.
            ),
            _ => return Err("для закона стыка нет численного свидетеля нагрузки".into()),
        };
        if limit.min != bound || limit.max != bound {
            return Err("предел свидетеля стыка должен совпадать с границей его модели".into());
        }
        sources.push(limit.clone());
        Ok(Self {
            kind,
            limit,
            sources,
            peak: None,
            transition: false,
        })
    }

    fn channel(
        &self,
    ) -> (
        HazardKind,
        &'static str,
        &'static str,
        AttachmentCause,
        &'static str,
    ) {
        match self.kind {
            LoadKind::Holding => (
                HazardKind::Retention,
                "holding_deficit",
                "N",
                AttachmentCause::PullOff,
                "осевая нагрузка превысила удерживающую силу и произошёл отрыв",
            ),
            LoadKind::Capacity(_) => (
                HazardKind::ConnectionFailure,
                "wrench_utilization",
                "1",
                AttachmentCause::Overload,
                "реакция замкнутого стыка вышла за огибающую, связь разорвана",
            ),
        }
    }

    fn record(
        &mut self,
        out: &mut ProcessOut,
        at: Time,
        name: &str,
        reaction: &Analysis<sim_process::attachments::AttachmentReaction>,
        state: AttachmentState,
        cause: Option<AttachmentCause>,
        axis: [f64; 3],
    ) -> Result<(), ProcessError> {
        let (kind, channel, unit, failure, reason) = self.channel();
        let analyzed = if let Analysis::Analyzed { value, .. } = reaction
            && (state == AttachmentState::Attached || cause == Some(failure))
        {
            let current = match &self.kind {
                LoadKind::Holding => {
                    dot(value.measured.force_n, axis) + dot(value.applied.force_n, axis)
                }
                LoadKind::Capacity(capacity) => capacity.utilization(value.measured)?[0],
            };
            if !current.is_finite() {
                return Err(invalid("неконечное значение нагрузки стыка"));
            }
            self.peak = Some(self.peak.map_or(current, |previous| previous.max(current)));
            self.transition |= cause == Some(failure) && current > self.limit.max;
            true
        } else {
            self.transition
        };
        let scope = "нагрузка удерживаемого или замкнутого стыка до подтверждённого размыкания";
        let value = self.peak.filter(|_| analyzed);
        let check = match value {
            Some(value)
                if self
                    .sources
                    .iter()
                    .all(|source| source.can_prove() && source.min == source.max) =>
            {
                match upper_check(value, &self.limit, scope)? {
                    HazardCheck::Violation { .. } if self.transition => HazardCheck::Violation {
                        reason: reason.into(),
                    },
                    HazardCheck::Violation { .. } => HazardCheck::NotAnalyzed {
                        reason: "нет подтверждённого перехода после перегрузки".into(),
                    },
                    check => check,
                }
            }
            _ => HazardCheck::NotAnalyzed {
                reason: "нет применимой реакции или источников стыка".into(),
            },
        };
        out.measure(
            at,
            format!("{name}/{channel}"),
            unit,
            value.map_or_else(
                || unavailable("нет наблюдения замкнутого стыка"),
                |value| measured(value, scope),
            ),
        )?;
        out.hazards.push(ProcessHazard {
            subject: name.into(),
            kind,
            check,
            sources: self.sources.clone(),
        });
        Ok(())
    }
}

impl<I> ProcessModel for Output<I>
where
    I: FnMut(&ProcessFrame<'_>) -> Result<AttachmentInput, ProcessError>,
{
    fn update(
        &mut self,
        frame: &ProcessFrame<'_>,
        rng: &mut Rng,
    ) -> Result<ProcessOut, ProcessError> {
        let mut out = self.adapter.update(frame, rng)?;
        let binding = self.adapter.model.binding();
        let at = frame.next.start;
        let load = self.input.prescribed(at);
        let b = &self.input.mounts[1];
        let (b, _) = b.observe(frame)?;
        let shift = cross(difference(b.point_m, b.com_m), load.force_n);
        if load.force_n != [0.0; 3] || load.torque_nm != [0.0; 3] {
            let body = &self.input.mounts[1].port.body;
            let torque = std::array::from_fn(|i| load.torque_nm[i] + shift[i]);
            if let Some(current) = out.body_loads.iter_mut().find(|item| &item.body == body) {
                for i in 0..3 {
                    current.force_n[i] += load.force_n[i];
                    current.torque_nm[i] += torque[i];
                }
            } else {
                out.body_loads.push(BodyLoad {
                    body: body.clone(),
                    force_n: load.force_n,
                    torque_nm: torque,
                });
            }
        }
        let record = out
            .attachment_records()
            .first()
            .map(|r| (r.cause, r.state, r.reaction.clone()));
        let values = record.as_ref().map(|(_, _, r)| r);
        if let Some(values) = values {
            let project =
                |map: fn(&sim_process::attachments::AttachmentReaction) -> f64| match values {
                    Analysis::Analyzed { value, scope } => measured(map(value), scope),
                    Analysis::NotAnalyzed { reason } => unavailable(reason),
                };
            type Readout = fn(&sim_process::attachments::AttachmentReaction) -> f64;
            let readouts: [(_, _, Readout); 5] = [
                ("gap", "m", |r| r.gap_m),
                ("applied_force", "N", |r| norm(r.applied.force_n)),
                ("applied_torque", "N*m", |r| norm(r.applied.torque_nm)),
                ("measured_force", "N", |r| norm(r.measured.force_n)),
                ("measured_torque", "N*m", |r| norm(r.measured.torque_nm)),
            ];
            for (field, unit, f) in readouts {
                out.measure(at, format!("{}/{field}", binding.name), unit, project(f))?;
            }
            let energy = match values {
                Analysis::Analyzed { value, .. } => value.stored_energy_j.clone(),
                Analysis::NotAnalyzed { reason } => unavailable(reason),
            };
            out.measure(at, format!("{}/stored_energy", binding.name), "J", energy)?;
        }
        if let (Some(audit), Some((cause, state, reaction))) = (&mut self.load_audit, &record) {
            let (support, _) = self.input.mounts[0].observe(frame)?;
            audit.record(
                &mut out,
                at,
                &binding.name,
                reaction,
                *state,
                *cause,
                support.axis,
            )?;
        }
        if let (Some(law), Some((cause, _, Analysis::Analyzed { value, .. }))) =
            (&self.thread, record)
            && let Some(turn) = self.input.observe(frame)?.thread
        {
            let ideal_travel =
                law.lead_m_per_turn.nominal * (turn.angle_rad / std::f64::consts::TAU);
            let error = (value.gap_m - law.entry_m.nominal - ideal_travel).abs();
            out.measure(
                at,
                format!("{}/elastic_error", binding.name),
                "m",
                measured(error, "измеренный осевой ход относительно фазы резьбы"),
            )?;
            if cause == Some(sim_process::attachments::AttachmentCause::Stripped)
                && error > law.maximum_elastic_error_m.max
            {
                out.hazards.push(ProcessHazard {
                    subject: binding.name.clone(),
                    kind: HazardKind::ThreadFailure,
                    check: HazardCheck::Violation {
                        reason: "срыв при превышении допустимой упругой невязки резьбы".into(),
                    },
                    sources: vec![
                        law.lead_m_per_turn.clone(),
                        law.entry_m.clone(),
                        law.maximum_elastic_error_m.clone(),
                    ],
                });
            }
        }
        Ok(out)
    }

    fn state_fingerprint(&self) -> u64 {
        self.load_audit.as_ref().map_or_else(
            || self.adapter.state_fingerprint(),
            |audit| {
                sim_signal::fnv1a(
                    format!(
                        "{:?}{:?}{:?}",
                        self.adapter.state_fingerprint(),
                        audit.peak,
                        audit.transition
                    )
                    .bytes(),
                )
            },
        )
    }
}

impl Attachment {
    pub(in crate::tasks::processes::models) fn uses_weld(&self, name: &str) -> bool {
        matches!(&self.law, Some(Law::Adhesive { weld: Some(weld), .. }) if weld == name)
    }

    pub(in crate::tasks::processes::models) fn install(
        self,
        name: &str,
        world: &CompiledWorld,
        rt: &mut Runtime,
        scene: &World,
        file: &Path,
    ) -> Result<(), String> {
        // Сохраняем обязательные опасности даже при неполной модели крепления.
        let kind = self.law.as_ref().map(Law::kind);
        let mut hazards = vec![
            HazardKind::Pinch,
            HazardKind::Impact,
            HazardKind::StoredEnergy,
        ];
        if kind == Some(AttachmentKind::Threaded) {
            hazards.push(HazardKind::ThreadFailure);
        }
        let incomplete = |rt: &mut Runtime| {
            rt.require_process_hazards(
                &hazards
                    .iter()
                    .map(|&kind| HazardEvidence {
                        subject: name.into(),
                        kind,
                        coverage: unavailable("нужна оценка опасности крепления"),
                    })
                    .collect::<Vec<_>>(),
            )?;
            missing(
                rt,
                name,
                "нет данных модели, нагрузки или наблюдений крепления",
            )
        };
        let (Some(ids), Some(limits), Some(law), Some(travel), Some(loads), Some(requests)) = (
            self.mounts,
            self.limits.and_then(Limits::build),
            self.law,
            self.available_travel_m,
            self.loading,
            self.requests,
        ) else {
            return incomplete(rt);
        };
        let base = super::super::super::dir(file);
        let mounts = [locate(&ids[0], scene, base)?, locate(&ids[1], scene, base)?];
        let body = &mounts[1].port.body;
        let mass = world
            .bodies
            .iter()
            .find(|b| &b.name == body)
            .ok_or_else(|| format!("нет подвижного тела стыка «{body}»"))?
            .mass_kg;
        let mut input = Inputs {
            mounts,
            loads: loads.schedule()?,
            travel_m: nonnegative(travel).map_err(|e| e.to_string())?.min,
            gravity_force_n: scene.gravity_m_s2.map(|g| mass * g),
            drive_pair: None,
            thread: None,
            cure: None,
            equality: None,
        };
        let law = match law {
            Law::Magnetic { pull } => {
                let Some(pull) = curve(pull).map_err(|e| e.to_string())? else {
                    return incomplete(rt);
                };
                AttachmentLaw::Magnetic { pull }
            }
            Law::Latch {
                parameters,
                drive_pair,
            } => {
                let (Some(p), Some(pair)) = (parameters.and_then(Latch::build), drive_pair) else {
                    return incomplete(rt);
                };
                validate_pair(world, &pair)?;
                let second = &input.mounts[1].port.body;
                if pair[1]
                    .split_once("/collision/")
                    .is_none_or(|(body, _)| body != second)
                {
                    return Err(
                        "вторая геометрия приводящего контакта должна принадлежать второму стыку"
                            .into(),
                    );
                }
                if rt.process_contact_pairs().any(|p| {
                    same_pair(&pair, &p.pair.geometries)
                        && matches!(p.nominal_coverage, Analysis::Analyzed { .. })
                }) {
                    input.drive_pair = Some(pair);
                }
                AttachmentLaw::Latch(p)
            }
            Law::Threaded {
                parameters,
                angular_joint,
                phase_at_entry_rad,
            } => {
                let (Some(p), Some(angular), Some(phase)) = (
                    parameters.and_then(Thread::build),
                    angular_joint,
                    phase_at_entry_rad,
                ) else {
                    return incomplete(rt);
                };
                if joint(world, &angular)? != CompiledJointKind::Hinge {
                    return Err("непрерывная фаза резьбы требует вращательного соединения".into());
                }
                let phase = validated_range(phase).map_err(|e| e.to_string())?;
                let sim = world.open_contact_sim()?;
                let hinge = sim
                    .joints()
                    .into_iter()
                    .find(|j| j.name == angular)
                    .ok_or("ось резьбы отсутствует в движке")?;
                let bodies = sim.bodies();
                let follows = |mount: &LocatedMount| {
                    let mut id = bodies.iter().position(|(name, _)| *name == mount.port.body);
                    while let Some(body) = id.filter(|&body| body != 0) {
                        if body == hinge.body {
                            return true;
                        }
                        id = Some(sim.body_parent(body));
                    }
                    false
                };
                if !follows(&input.mounts[1]) || follows(&input.mounts[0]) {
                    return Err("ось резьбы должна изменять взаимный угол стыков".into());
                }
                if phase.min == phase.max {
                    input.thread = Some((angular, hinge.id, phase.nominal));
                }
                AttachmentLaw::Threaded(p)
            }
            Law::Adhesive {
                parameters,
                weld,
                cure_observation,
            } => {
                let (Some(p), Some(weld), Some(cure)) =
                    (parameters.and_then(Adhesive::build), weld, cure_observation)
                else {
                    return incomplete(rt);
                };
                if !scene.attachments.iter().any(|a| {
                    a.name == weld
                        && a.mode == AttachmentMode::Runtime
                        && a.first == ids[0]
                        && a.second == ids[1]
                }) {
                    return Err(
                        "клею нужен исходно выключенный weld тех же стыков в том же порядке".into(),
                    );
                }
                let equality = format!("attachment/{weld}");
                let bodies = rt.record_process_weld(&equality)?;
                if bodies.each_ref() != input.mounts.each_ref().map(|m| &m.port.body) {
                    return Err("тела ограничения не совпадают с телами стыков".into());
                }
                input.equality = Some(equality);
                input.cure = cure.build().map_err(|e| e.to_string())?;
                AttachmentLaw::Adhesive(p)
            }
        };
        let ports = input.mounts.each_ref().map(|m| m.port.clone());
        let binding = AttachmentBinding {
            name: name.into(),
            mounts: ids,
            kind: kind.expect("закон задан"),
        };
        let thread = match &law {
            AttachmentLaw::Threaded(law) => Some(law.clone()),
            _ => None,
        };
        let load_audit = self
            .witness_limit
            .map(|upper| LoadAudit::new(&law, &limits.capacity, upper))
            .transpose()?;
        if let Some(audit) = &load_audit {
            rt.require_process_hazards(&[HazardEvidence {
                subject: name.into(),
                kind: audit.channel().0,
                coverage: unavailable("нужна подтверждённая нагрузка стыка"),
            }])?;
        }
        let model = AttachmentModel::new(binding, ports, limits, law).map_err(|e| e.to_string())?;
        rt.require_process_hazards(&model.hazards())?;
        let mut commands = BTreeMap::new();
        for r in requests {
            let at = Time(r.at_ns);
            let action = match r.action {
                Action::Attach => AttachmentAction::Attach,
                Action::Release => AttachmentAction::Release,
            };
            if commands
                .insert(
                    at,
                    AttachmentRequest {
                        at,
                        name: name.into(),
                        action,
                    },
                )
                .is_some()
            {
                return Err("повтор команды крепления".into());
            }
        }
        let observer = input.clone();
        let adapter = AttachmentAdapter {
            model,
            equality: input.equality.clone(),
            requests: commands,
            input: move |frame: &ProcessFrame<'_>| observer.observe(frame),
        };
        // Пространственную реакцию weld регистрируем один раз вместе с его строками решателя.
        Supported::install(
            Output {
                adapter,
                input,
                thread,
                load_audit,
            },
            name,
            world,
            rt,
        )
    }
}
