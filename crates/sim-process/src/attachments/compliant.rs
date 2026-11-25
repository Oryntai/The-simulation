//! Односторонняя площадка с податливостью по нормали и кулоновским сдвигом.

use sim_signal::{Rng, Time};

use crate::contact::ContactFrame;
use crate::{Analysis, BodyLoad, Interval, ProcessError, SourcedRange, StepModel};

use super::math::{
    Vector, add, analyzed, angle, cross, dot, finite, invalid, length_and_unit, load, norm, scale,
    sub, unavailable,
};
use super::{
    ForceCurve, InterfaceInput, InterfaceReaction, InterfaceState, InterfaceStep, InterfaceWork,
    MountMotion, Status, Transition, curve_supported, elastic, name, range, scalar_supported,
    work_is_finite,
};

#[derive(Clone, Debug, PartialEq)]
pub struct PadLaw {
    /// Расстояние по нормали между точками при нулевом сжатии.
    /// Меньшее начальное расстояние даёт преднатяг по характеристике.
    pub free_gap_m: SourcedRange,
    pub compression: ForceCurve,
    pub damping_ns_m: SourcedRange,
    pub shear_stiffness_n_m: SourcedRange,
    pub friction: SourcedRange,
    /// Предел результирующей силы крепления. При превышении возможен обрыв.
    pub capacity_n: SourcedRange,
}

#[derive(Clone, Debug, PartialEq)]
pub enum PadReactionOwner {
    /// Для этой пары коллизия должна быть отключена в собранном мире.
    Model(Box<PadLaw>),
    /// Движок прикладывает всю реакцию пары. Модель только измеряет её.
    Engine,
}

/// Каждый диапазон задаёт включённые границы применимости. Номинал здесь
/// не заменяет границ. Скорость измеряется в системе, вращающейся с опорой А.
#[derive(Clone, Debug, PartialEq)]
pub struct PadCalibration {
    pub compression_m: SourcedRange,
    pub offset_m: SourcedRange,
    pub axis_error_rad: SourcedRange,
    pub relative_speed_m_s: SourcedRange,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PadSpec {
    pub name: String,
    pub bodies: [String; 2],
    pub geometries: [String; 2],
    pub owner: PadReactionOwner,
    /// Эффективная площадь пятна, подтверждённая для окна калибровки.
    pub area_m2: Option<SourcedRange>,
    pub calibration: PadCalibration,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PadInput {
    /// Обе оси сонаправлены при идеальной посадке. Ось А указывает от А к Б.
    pub mounts: [MountMotion; 2],
    /// Касательная ось А в мире, перпендикулярна нормали. Состояние сдвига
    /// хранится в этом вращающемся базисе.
    pub tangent_a: Vector,
    /// Правая граница предыдущего шага. Пусто только до первой физики
    /// или если наблюдение недоступно, что блокирует доказательство.
    pub completed: Option<ContactFrame>,
    pub event: Option<super::ConfirmedInterfaceEvent>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StressBounds {
    /// Границы среднего давления, Па.
    pub pressure_pa: [f64; 2],
    /// Нижняя граница по результирующей, верхняя по сумме модулей.
    pub shear_pa: [f64; 2],
}

#[derive(Clone, Debug, PartialEq)]
pub struct PatchAreaPoint {
    pub compression_m: f64,
    pub area_m2: SourcedRange,
}

#[derive(Clone, Debug)]
pub struct CompliantPad {
    spec: PadSpec,
    area_curve: Vec<PatchAreaPoint>,
    shear_m: [f64; 2],
    status: Status,
}

struct ObservedPad {
    reaction: InterfaceReaction,
    compression_m: [f64; 2],
}

impl CompliantPad {
    pub fn new(spec: PadSpec) -> Result<Self, ProcessError> {
        name(&spec.name)?;
        for names in [&spec.bodies, &spec.geometries] {
            names.iter().try_for_each(|id| name(id))?;
            if names[0] == names[1] {
                return Err(invalid("стороны площадки должны быть различимы"));
            }
        }

        let calibration = &spec.calibration;
        for r in [
            &calibration.compression_m,
            &calibration.offset_m,
            &calibration.relative_speed_m_s,
        ] {
            range(r, 0.0, f64::MAX)?;
        }
        range(&calibration.axis_error_rad, 0.0, std::f64::consts::PI)?;
        if let Some(area) = &spec.area_m2 {
            super::positive(area)?;
        }
        if let PadReactionOwner::Model(law) = &spec.owner {
            for r in [&law.free_gap_m, &law.damping_ns_m, &law.friction] {
                range(r, 0.0, f64::MAX)?;
            }
            super::positive(&law.shear_stiffness_n_m)?;
            super::positive(&law.capacity_n)?;
            super::validate_curve(&law.compression)?;
            if calibration.compression_m.max
                > law.compression.points().last().unwrap().displacement_m.min
            {
                return Err(invalid("окно сжатия шире характеристики площадки"));
            }
        }
        Ok(Self {
            spec,
            area_curve: Vec::new(),
            shear_m: [0.0; 2],
            status: Status::default(),
        })
    }

    pub fn spec(&self) -> &PadSpec {
        &self.spec
    }

    /// Узлы эффективной площади из замера пятна при заданной геометрии и
    /// окне калибровки. Между узлами берётся огибающая, вне узлов площади нет.
    pub fn with_area_curve(mut self, points: Vec<PatchAreaPoint>) -> Result<Self, ProcessError> {
        if points.len() < 2
            || points
                .windows(2)
                .any(|p| p[0].compression_m >= p[1].compression_m)
            || points.iter().any(|p| {
                !p.compression_m.is_finite()
                    || p.compression_m < 0.0
                    || p.area_m2.min < 0.0
                    || p.area_m2.validate().is_err()
                    || p.area_m2.provenance != points[0].area_m2.provenance
            })
        {
            return Err(invalid("невалидная характеристика площади пятна"));
        }
        self.area_curve = points;
        Ok(self)
    }

    fn effective_area(&self, [low, high]: [f64; 2]) -> Option<SourcedRange> {
        let points = &self.area_curve;
        if low <= 0.0 || low < points.first()?.compression_m || high > points.last()?.compression_m
        {
            return None;
        }
        let first = points.partition_point(|p| p.compression_m <= low) - 1;
        let last = points.partition_point(|p| p.compression_m < high);
        let mut area = points[first].area_m2.clone();
        for p in &points[first..=last] {
            area.min = area.min.min(p.area_m2.min);
            area.max = area.max.max(p.area_m2.max);
        }
        (area.min > 0.0 && area.can_prove()).then_some(area)
    }

    fn measured(
        &self,
        start: Time,
        frame: Option<&ContactFrame>,
        mounts: &[MountMotion; 2],
        axis: Vector,
    ) -> Result<Option<ObservedPad>, ProcessError> {
        let Some(frame) = frame else { return Ok(None) };
        if frame.at != start {
            return Err(invalid(
                "контактная реакция относится к другой границе шага",
            ));
        }
        let mut forces = [
            load(
                &self.spec.bodies[0],
                mounts[0].com_m,
                mounts[0].com_m,
                [0.0; 3],
                [0.0; 3],
            )?,
            load(
                &self.spec.bodies[1],
                mounts[1].com_m,
                mounts[1].com_m,
                [0.0; 3],
                [0.0; 3],
            )?,
        ];
        let mut pressure_force = 0.0;
        let mut shear = [0.0; 3];
        let mut shear_bound = 0.0;
        let mut peak_shear: f64 = 0.0;
        let mut contact_count = 0;
        let mut compression = [0.0_f64; 2];
        for c in &frame.contacts {
            finite(
                c.point_m
                    .into_iter()
                    .chain(c.normal)
                    .chain(c.force_n)
                    .chain(c.torque_nm)
                    .chain([c.separation_m]),
            )?;
            let sign = if c.geometries == self.spec.geometries {
                1.0
            } else if c.geometries[0] == self.spec.geometries[1]
                && c.geometries[1] == self.spec.geometries[0]
            {
                -1.0
            } else {
                continue;
            };
            if matches!(self.spec.owner, PadReactionOwner::Model(_)) {
                return Err(invalid(
                    "пара площадки одновременно обслуживается движком и моделью",
                ));
            }
            let penetration = (-c.separation_m).max(0.0);
            compression = if contact_count == 0 {
                [penetration; 2]
            } else {
                [
                    compression[0].min(penetration),
                    compression[1].max(penetration),
                ]
            };
            contact_count += 1;
            let normal = unit(c.normal)?;
            let n = dot(c.force_n, normal);
            // Сжимающие реакции разных пятен не погашают друг друга.
            pressure_force += n.max(0.0);
            let t = scale(sub(c.force_n, scale(normal, n)), sign);
            shear_bound += norm(t);
            peak_shear = peak_shear.max(norm(t));
            // Для результирующего сдвига используется общая плоскость площадки.
            shear = add(shear, sub(t, scale(axis, dot(t, axis))));
            for (i, force) in forces.iter_mut().enumerate() {
                let side = if i == 0 { -sign } else { sign };
                let wrench = load(
                    &self.spec.bodies[i],
                    c.point_m,
                    mounts[i].com_m,
                    scale(c.force_n, side),
                    scale(c.torque_nm, side),
                )?;
                force.force_n = add(force.force_n, wrench.force_n);
                force.torque_nm = add(force.torque_nm, wrench.torque_nm);
            }
        }
        finite(
            [pressure_force, shear_bound, peak_shear]
                .into_iter()
                .chain(shear)
                .chain(
                    forces
                        .iter()
                        .flat_map(|w| w.force_n.into_iter().chain(w.torque_nm)),
                ),
        )?;
        Ok(Some(ObservedPad {
            reaction: InterfaceReaction {
                loads: forces.into(),
                normal_n: pressure_force,
                shear_n: shear,
                shear_upper_n: shear_bound.max(norm(shear)),
                peak_shear_n: peak_shear,
                tension_n: 0.0,
                contact_count,
            },
            compression_m: compression,
        }))
    }

    fn model(
        &self,
        law: &PadLaw,
        geometry: &PadGeometry,
        mounts: &[MountMotion; 2],
        dt: f64,
        mut step: InterfaceStep,
        release: Option<Transition>,
    ) -> Result<(InterfaceStep, [f64; 2]), ProcessError> {
        let compression = law.free_gap_m.nominal - geometry.gap_m;
        step.deformation_m = Some(compression);
        let speed = -dot(geometry.relative_velocity, geometry.normal);
        let Some(spring) = elastic(
            &law.compression,
            law.damping_ns_m.nominal,
            compression,
            speed,
        )?
        else {
            step.state = self
                .status
                .with_transition(release, InterfaceState::Unavailable);
            step.transition = release;
            step.unavailable(&format!("сжатие {compression} м вне характеристики площадки"));
            return Ok((step, self.shear_m));
        };

        let k = law.shear_stiffness_n_m.nominal;
        let cap = law.friction.nominal * spring.force_n;

        // Неявное обновление сдвига. Проекция пробного смещения на круг
        // |k·s| ≤ μ·N отделяет упругое смещение от необратимого скольжения.
        let mut trial = [0.0; 2];
        for (i, component) in trial.iter_mut().enumerate() {
            *component =
                self.shear_m[i] + dt * dot(geometry.relative_velocity, geometry.tangent[i]);
        }
        let length = trial[0].hypot(trial[1]);
        let radius = cap / k;
        finite([cap, length, radius])?;
        let sliding = compression > 0.0 && length > radius;
        let shear = if length > radius {
            trial.map(|v| v / length * radius)
        } else {
            trial
        };
        let [u, v] = geometry.tangent;
        let shear_force = add(scale(u, -k * shear[0]), scale(v, -k * shear[1]));
        let force = add(scale(geometry.normal, spring.force_n), shear_force);
        let attempted = norm(force);
        finite([attempted])?;
        let transition = release.or_else(|| {
            (attempted > law.capacity_n.min).then_some(Transition::Overload {
                force_n: attempted,
                limit_n: law.capacity_n.min,
            })
        });
        let state = self.status.with_transition(
            transition,
            if sliding {
                InterfaceState::Sliding
            } else if compression > 0.0 {
                InterfaceState::Loaded
            } else {
                InterfaceState::Open
            },
        );
        step.state = state;
        step.transition = transition;
        let mut work = InterfaceWork::default();
        let mut resulting_shear = shear;
        let mut reaction = InterfaceReaction::zero();

        if state.is_terminal() {
            // Размыкание на текущей границе переносит оставшийся запас
            // энергии в отдельную строку. Её распределение неизвестно.
            work.released_energy_j = spring.energy_j + shear_energy(k, self.shear_m);
            resulting_shear = [0.0; 2];
        } else {
            reaction.normal_n = spring.force_n;
            reaction.shear_n = shear_force;
            reaction.shear_upper_n = norm(shear_force);
            reaction.peak_shear_n = reaction.shear_upper_n;
            reaction.contact_count = usize::from(compression > 0.0);
            reaction.loads = paired_loads(&self.spec.bodies, mounts, force)?;
            if attempted > 0.0 {
                step.body_loads = reaction.loads.clone();
            }
            work.stored_energy_j = spring.energy_j + shear_energy(k, shear);
            work.body_work_j = dot(force, geometry.relative_velocity) * dt;
            let slip = (length - radius).max(0.0);
            step.slip_m = Some(if compression > 0.0 { slip } else { 0.0 });
            work.loss_j = spring.loss_power_w * dt + if sliding { cap * slip } else { 0.0 };
            // Для неявной дискретизации остаток равен k·|s₁−s₀|²/2.
            // При раскрытии оставшаяся энергия сдвига списывается численно.
            let delta = std::array::from_fn(|i| shear[i] - self.shear_m[i]);
            work.numerical_loss_j = shear_energy(k, delta);
        }
        work_is_finite(work)?;
        step.reaction = analyzed(reaction, "расчёт площадки на левой границе");
        step.work = analyzed(work, "работа удержанной реакции по скоростям левой границы");

        let sources = [
            &law.free_gap_m,
            &law.damping_ns_m,
            &law.shear_stiffness_n_m,
            &law.friction,
            &law.capacity_n,
        ];
        self.calibrate(
            &mut step,
            geometry,
            [compression.max(0.0); 2],
            interval_sources(sources) && curve_supported(&law.compression),
        )?;
        Ok((step, resulting_shear))
    }

    fn calibrate(
        &self,
        step: &mut InterfaceStep,
        g: &PadGeometry,
        compression: [f64; 2],
        law_supported: bool,
    ) -> Result<(), ProcessError> {
        let c = &self.spec.calibration;
        let bounds = [
            (&c.compression_m, compression[0]),
            (&c.compression_m, compression[1]),
            (&c.offset_m, g.offset_m),
            (&c.axis_error_rad, g.axis_error_rad),
            (&c.relative_speed_m_s, norm(g.relative_velocity)),
        ];
        finite(bounds.iter().map(|(_, value)| *value))?;
        let reason = if bounds.iter().any(|(r, value)| !super::contains(r, *value)) {
            Some("положение или скорость вне области калибровки площадки")
        } else if bounds.iter().any(|(r, _)| !r.can_prove()) {
            Some("границы применимости площадки не подтверждены")
        } else if !law_supported {
            Some("номинальный закон площадки требует подтверждения параметров и разброса")
        } else if step.contact_frame_at.is_none() {
            Some("нет завершённого кадра для проверки источника реакции")
        } else {
            None
        };
        step.calibration = reason.map_or_else(
            || {
                analyzed(
                    (),
                    "окно калибровки площадки и подтверждённый кадр контакта",
                )
            },
            unavailable,
        );

        if let Some(reason) = reason {
            step.stress = unavailable(reason);
        } else {
            if let Analysis::Analyzed { value, .. } = &step.reaction {
                if value.normal_n > 0.0 || value.shear_upper_n > 0.0 {
                    step.effective_area_m2 = (value.normal_n > 0.0)
                        .then(|| self.effective_area(compression))
                        .flatten();
                    // Постоянная площадь сохранена для прежней диагностики модели.
                    let area = step
                        .effective_area_m2
                        .as_ref()
                        .or_else(|| self.spec.area_m2.as_ref().filter(|r| r.can_prove()));
                    let Some(area) = area else {
                        step.stress = unavailable("нет подтверждённой эффективной площади");
                        return Ok(());
                    };
                    let result = StressBounds {
                        pressure_pa: [value.normal_n / area.max, value.normal_n / area.min],
                        shear_pa: [
                            norm(value.shear_n) / area.max,
                            value.shear_upper_n / area.min,
                        ],
                    };
                    finite(result.pressure_pa.into_iter().chain(result.shear_pa))?;
                    step.stress =
                        analyzed(result, "средние напряжения по подтверждённой площади пятна");
                } else {
                    step.stress = unavailable("нет подтверждённого нагруженного пятна");
                }
            }
        }
        Ok(())
    }
}

impl StepModel for CompliantPad {
    type Input = InterfaceInput;
    type Output = InterfaceStep;

    fn advance(
        &mut self,
        interval: Interval,
        input: &InterfaceInput,
        _: &mut Rng,
    ) -> Result<InterfaceStep, ProcessError> {
        let InterfaceInput::Pad(input) = input else {
            return Err(invalid("площадка требует кинематику площадки"));
        };
        let (dt, event) = self
            .status
            .begin(interval, &self.spec.name, input.event.as_ref())?;
        if matches!(event, Some(Transition::ConfirmedSlip { .. })) {
            return Err(invalid(
                "проскальзывание площадки определяется её законом сдвига",
            ));
        }
        let g = PadGeometry::new(&input.mounts, input.tangent_a)?;
        let measured = self.measured(
            interval.start,
            input.completed.as_ref(),
            &input.mounts,
            g.normal,
        )?;
        let mut step = InterfaceStep::new(interval, self.status.state);
        step.contact_frame_at = input.completed.as_ref().map(|frame| frame.at);
        step.offset_m = Some(g.offset_m);
        step.axis_error_rad = Some(g.axis_error_rad);
        if self.status.state == InterfaceState::Unavailable {
            step.state = self
                .status
                .with_transition(event, InterfaceState::Unavailable);
            step.transition = event;
            step.unavailable("предыдущий шаг оставил состояние площадки неопределённым");
            self.status.commit(&step);
            return Ok(step);
        }
        if self.status.state.is_terminal() {
            if measured.is_some_and(|r| r.reaction.contact_count != 0) {
                return Err(invalid("реакция движка осталась после размыкания площадки"));
            }
            step.mark_inactive();
            self.status.commit(&step);
            return Ok(step);
        }

        let (step, shear) = match &self.spec.owner {
            PadReactionOwner::Model(law) => self.model(law, &g, &input.mounts, dt, step, event)?,
            PadReactionOwner::Engine => {
                let compression = measured.as_ref().map(|r| r.compression_m);
                step.deformation_m = compression.map(|bounds| bounds[1]);
                step.transition = event;
                step.state = self.status.with_transition(
                    event,
                    match &measured {
                        Some(r) if r.reaction.contact_count > 0 => InterfaceState::Loaded,
                        Some(_) => InterfaceState::Open,
                        None => InterfaceState::Unobserved,
                    },
                );
                step.reaction = measured.map_or_else(
                    || unavailable("нет завершённого кадра контакта движка"),
                    |r| analyzed(r.reaction, "реакции пары из завершённого интервала движка"),
                );
                // Событие на границе меняет следующий интервал. Реакция
                // завершённого интервала сохраняется даже при размыкании.
                if let Some(compression) = compression {
                    self.calibrate(&mut step, &g, compression, true)?;
                }
                (step, self.shear_m)
            }
        };
        self.shear_m = shear;
        self.status.commit(&step);
        Ok(step)
    }
}

struct PadGeometry {
    gap_m: f64,
    offset_m: f64,
    axis_error_rad: f64,
    normal: Vector,
    tangent: [Vector; 2],
    relative_velocity: Vector,
}

impl PadGeometry {
    fn new(mounts: &[MountMotion; 2], tangent: Vector) -> Result<Self, ProcessError> {
        for m in mounts {
            finite(
                m.point_m
                    .into_iter()
                    .chain(m.com_m)
                    .chain(m.axis)
                    .chain(m.velocity_m_s)
                    .chain(m.angular_velocity_rad_s),
            )?;
        }
        let [a, b] = mounts;
        let normal = unit(a.axis)?;
        let tangent = unit(tangent)?;
        // Допуск для трёхкомпонентного скалярного произведения осей,
        // прошедших поворот и нормирование, порядка машинного округления.
        if dot(normal, tangent).abs() > 32.0 * f64::EPSILON {
            return Err(invalid("касательная ось не перпендикулярна нормали"));
        }
        let second = unit(cross(normal, tangent))?;
        let tangent = [cross(second, normal), second];
        let offset = sub(b.point_m, a.point_m);
        let gap_m = dot(offset, normal);
        let offset_m = norm(sub(offset, scale(normal, gap_m)));
        let relative_velocity = sub(
            sub(b.velocity_m_s, a.velocity_m_s),
            cross(a.angular_velocity_rad_s, offset),
        );
        let axis_error_rad = angle(normal, unit(b.axis)?);
        finite(
            [gap_m, offset_m, axis_error_rad]
                .into_iter()
                .chain(relative_velocity),
        )?;
        Ok(Self {
            gap_m,
            offset_m,
            axis_error_rad,
            normal,
            tangent,
            relative_velocity,
        })
    }
}

fn paired_loads(
    names: &[String; 2],
    mounts: &[MountMotion; 2],
    force: Vector,
) -> Result<Vec<BodyLoad>, ProcessError> {
    // Общая точка действия на Б обеспечивает баланс пространственного
    // момента и сопрягает работы сил с относительной скоростью в системе А.
    let point = mounts[1].point_m;
    Ok(vec![
        load(
            &names[0],
            point,
            mounts[0].com_m,
            scale(force, -1.0),
            [0.0; 3],
        )?,
        load(&names[1], point, mounts[1].com_m, force, [0.0; 3])?,
    ])
}

fn unit(v: Vector) -> Result<Vector, ProcessError> {
    length_and_unit(v)
        .map(|(_, direction)| direction)
        .ok_or_else(|| invalid("вырожденная ось площадки"))
}

fn shear_energy(k: f64, displacement: [f64; 2]) -> f64 {
    let [u, v] = displacement;
    (0.5 * k * u) * u + (0.5 * k * v) * v
}

fn interval_sources<const N: usize>(sources: [&SourcedRange; N]) -> bool {
    sources.into_iter().all(scalar_supported)
}
