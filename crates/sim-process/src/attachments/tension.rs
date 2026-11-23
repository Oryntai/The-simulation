//! Натяжение растяжимой трассы и реакция на сопряжённой координате.

use std::collections::BTreeMap;

use sim_signal::{ChannelId, Rng};

use crate::{Interval, ProcessError, SourcedRange, StepModel};

use super::math::{
    Vector, add, analyzed, dot, finite, invalid, length_and_unit, load, scale, sub, unavailable,
};
use super::{
    ConfirmedInterfaceEvent, ForceCurve, InterfaceInput, InterfaceReaction, InterfaceState,
    InterfaceStep, InterfaceWork, Status, Transition, contains, curve_supported, elastic, name,
    positive, range, scalar_supported, validate_curve, work_is_finite,
};

#[derive(Clone, Debug, PartialEq)]
pub enum FreeLengthLaw {
    Fixed(SourcedRange),
    Affine {
        channel: ChannelId,
        /// Свободная длина при нулевой физической координате.
        at_zero_m: SourcedRange,
        /// Подписанное изменение длины на рад или м физической координаты.
        slope: SourcedRange,
        coordinate: SourcedRange,
    },
}

/// Кулоновское скольжение одного конца через прижим. Промежуточные
/// направляющие идеальны. Распределённое трение в трассе здесь отсутствует.
#[derive(Clone, Debug, PartialEq)]
pub struct TractionFriction {
    pub coefficient: SourcedRange,
    pub clamp_normal_n: SourcedRange,
}

#[derive(Clone, Debug, PartialEq)]
pub struct TensionCalibration {
    pub extension_m: SourcedRange,
    pub extension_speed_m_s: SourcedRange,
    pub payout_m: SourcedRange,
}

#[derive(Clone, Debug, PartialEq)]
pub struct TensionSpec {
    pub name: String,
    /// Имена тел в порядке точек трассы. Одно тело может нести несколько точек.
    pub bodies: Vec<String>,
    pub length: FreeLengthLaw,
    pub extension: ForceCurve,
    pub damping_ns_m: SourcedRange,
    pub friction: Option<TractionFriction>,
    pub capacity_n: SourcedRange,
    pub calibration: TensionCalibration,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RoutePoint {
    pub point_m: Vector,
    pub com_m: Vector,
    pub velocity_m_s: Vector,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DriveMotion {
    pub coordinate: f64,
    pub velocity: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct TensionInput {
    pub points: Vec<RoutePoint>,
    pub drive: Option<DriveMotion>,
    pub event: Option<ConfirmedInterfaceEvent>,
}

#[derive(Clone, Debug)]
pub struct TensionRoute {
    spec: TensionSpec,
    slip_cap_n: Option<f64>,
    slip_extension_m: Option<f64>,
    payout_m: f64,
    status: Status,
}

impl TensionRoute {
    pub fn new(spec: TensionSpec) -> Result<Self, ProcessError> {
        name(&spec.name)?;
        spec.bodies.iter().try_for_each(|id| name(id))?;
        if spec.bodies.len() < 2 {
            return Err(invalid("у трассы должно быть не меньше двух точек"));
        }
        validate_curve(&spec.extension)?;
        positive(&spec.capacity_n)?;
        for r in [
            &spec.damping_ns_m,
            &spec.calibration.extension_m,
            &spec.calibration.extension_speed_m_s,
            &spec.calibration.payout_m,
        ] {
            range(r, 0.0, f64::MAX)?;
        }
        if spec.calibration.extension_m.max
            > spec.extension.points().last().unwrap().displacement_m.min
        {
            return Err(invalid("окно растяжения шире характеристики трассы"));
        }
        match &spec.length {
            FreeLengthLaw::Fixed(rest) => positive(rest)?,
            FreeLengthLaw::Affine {
                at_zero_m,
                slope,
                coordinate,
                ..
            } => {
                range(at_zero_m, -f64::MAX, f64::MAX)?;
                range(slope, -f64::MAX, f64::MAX)?;
                range(coordinate, -f64::MAX, f64::MAX)?;
                if contains(slope, 0.0) {
                    return Err(invalid("знак производной свободной длины не определён"));
                }
                for q in [coordinate.min, coordinate.max] {
                    for k in [slope.min, slope.max] {
                        let offset = k * q;
                        let length = at_zero_m.min + offset;
                        finite([length, at_zero_m.max + offset])?;
                        if length <= 0.0 {
                            return Err(invalid(
                                "неположительная свободная длина в рабочем диапазоне",
                            ));
                        }
                    }
                }
            }
        }
        let slip_cap_n = spec
            .friction
            .as_ref()
            .map(|friction| {
                range(&friction.coefficient, 0.0, f64::MAX)?;
                positive(&friction.clamp_normal_n)?;
                let cap = friction.coefficient.nominal * friction.clamp_normal_n.nominal;
                finite([cap])?;
                Ok(cap)
            })
            .transpose()?;
        let slip_extension_m = slip_cap_n.and_then(|cap| extension_at_limit(&spec.extension, cap));
        Ok(Self {
            spec,
            slip_cap_n,
            slip_extension_m,
            payout_m: 0.0,
            status: Status::default(),
        })
    }

    pub fn spec(&self) -> &TensionSpec {
        &self.spec
    }

    fn drive(&self, sample: Option<DriveMotion>) -> Result<FreeMotion, ProcessError> {
        let motion = match (&self.spec.length, sample) {
            (FreeLengthLaw::Fixed(rest), None) => FreeMotion {
                length_m: rest.nominal,
                speed_m_s: 0.0,
                effort: None,
                supported: scalar_supported(rest),
            },
            (
                FreeLengthLaw::Affine {
                    channel,
                    at_zero_m,
                    slope,
                    coordinate,
                },
                Some(q),
            ) => {
                finite([q.coordinate, q.velocity])?;
                FreeMotion {
                    length_m: at_zero_m.nominal + slope.nominal * q.coordinate,
                    speed_m_s: slope.nominal * q.velocity,
                    effort: Some((*channel, slope.nominal)),
                    supported: coordinate.can_prove()
                        && contains(coordinate, q.coordinate)
                        && scalar_supported(at_zero_m)
                        && scalar_supported(slope),
                }
            }
            _ => {
                return Err(invalid(
                    "кинематика координаты не соответствует закону свободной длины",
                ));
            }
        };
        finite([motion.length_m, motion.speed_m_s])?;
        Ok(motion)
    }

    fn calibration(
        &self,
        e: f64,
        speed: f64,
        payout: f64,
        drive: &FreeMotion,
    ) -> crate::Analysis<()> {
        let c = &self.spec.calibration;
        let windows = [
            (&c.extension_m, e.max(0.0)),
            (&c.extension_speed_m_s, speed.abs()),
            (&c.payout_m, payout),
        ];
        if windows.iter().any(|(r, x)| !contains(r, *x)) {
            return unavailable("растяжение, скорость или смещение вне области калибровки трассы");
        }
        let friction_supported = self.spec.friction.as_ref().is_none_or(|f| {
            scalar_supported(&f.coefficient) && scalar_supported(&f.clamp_normal_n)
        });
        if !drive.supported
            || !friction_supported
            || !curve_supported(&self.spec.extension)
            || !scalar_supported(&self.spec.damping_ns_m)
            || !scalar_supported(&self.spec.capacity_n)
            || !windows.iter().all(|(r, _)| r.can_prove())
        {
            return unavailable("границы или параметры номинального закона трассы не подтверждены");
        }
        analyzed(
            (),
            "область калибровки одномерной трассы с идеальными направляющими",
        )
    }

    fn solve(
        &self,
        interval: Interval,
        dt: f64,
        input: &TensionInput,
        event: Option<Transition>,
    ) -> Result<(InterfaceStep, f64), ProcessError> {
        let route = Route::new(&self.spec.bodies, &input.points)?;
        let drive = self.drive(input.drive)?;
        let mut step = InterfaceStep::new(interval, self.status.state);
        step.path_length_m = Some(route.length_m);
        step.payout_m = Some(self.payout_m);
        if self.status.state.is_terminal() {
            step.mark_inactive();
            return Ok((step, self.payout_m));
        }
        if self.status.state == InterfaceState::Unavailable {
            step.state = self
                .status
                .with_transition(event, InterfaceState::Unavailable);
            step.transition = event;
            step.unavailable("предыдущий шаг оставил состояние трассы неопределённым");
            return Ok((step, self.payout_m));
        }

        step.transition = event;
        let mut payout = self.payout_m;
        let confirmed = match event {
            Some(Transition::ConfirmedSlip { payout_m }) => payout_m,
            _ => 0.0,
        };
        let rate = route.speed_m_s - drive.speed_m_s;
        let free_before = drive.length_m + payout;
        let e_before = route.length_m - free_before;
        finite([e_before, rate, free_before])?;
        payout += confirmed;
        let free = drive.length_m + payout;
        let e = route.length_m - free;
        finite([e, free, payout])?;
        step.deformation_m = Some(e);
        step.free_length_m = Some(free);

        if free_before <= 0.0 {
            step.state = self
                .status
                .with_transition(event, InterfaceState::Unavailable);
            step.unavailable("неположительная свободная длина вне области модели");
            return Ok((step, self.payout_m));
        }
        let curve = &self.spec.extension;
        let damping = self.spec.damping_ns_m.nominal;
        let Some(before) = elastic(curve, 0.0, e_before, 0.0)? else {
            step.state = self
                .status
                .with_transition(event, InterfaceState::Unavailable);
            step.unavailable("растяжение вне характеристики трассы");
            return Ok((step, self.payout_m));
        };
        // Устранение перерастяжения на границе шага. Эта часть потерь
        // учитывается отдельно от непрерывного кулоновского проскальзывания.
        let working = self.slip_extension_m.map_or(e, |limit| e.min(limit));
        let correction = e - working;
        let spring = elastic(curve, damping, working, rate)?
            .ok_or_else(|| invalid("растяжение после проскальзывания вне характеристики трассы"))?;
        let mut tension = spring.force_n;
        let mut sliding_rate = 0.0;
        if let Some(cap) = self.slip_cap_n {
            if working > 0.0 && damping > 0.0 && tension > cap {
                sliding_rate = (rate - ((cap - spring.elastic_n) / damping).max(0.0)).max(0.0);
            } else if working >= 0.0 && self.slip_extension_m.is_some_and(|x| working >= x) {
                sliding_rate = rate.max(0.0);
            }
            tension = tension.min(cap);
        }
        let transition = if matches!(
            event,
            Some(Transition::ConfirmedRelease | Transition::ConfirmedBreak)
        ) {
            event
        } else if tension > self.spec.capacity_n.min {
            Some(Transition::Overload {
                force_n: tension,
                limit_n: self.spec.capacity_n.min,
            })
        } else {
            event
        };
        let slipping = confirmed > 0.0 || correction > 0.0 || sliding_rate > 0.0;
        step.transition = transition;
        step.state = self.status.with_transition(
            transition,
            if slipping {
                InterfaceState::Sliding
            } else if working > 0.0 {
                InterfaceState::Loaded
            } else {
                InterfaceState::Open
            },
        );

        let mut reaction = InterfaceReaction::zero();
        let mut work = InterfaceWork::default();
        if step.state.is_terminal() {
            work.released_energy_j = before.energy_j;
        } else {
            payout += correction;
            let after_event = elastic(curve, 0.0, e, 0.0)?
                .ok_or_else(|| invalid("растяжение после события вне характеристики трассы"))?;
            work.released_energy_j = (before.energy_j - after_event.energy_j).max(0.0);
            work.numerical_loss_j = (after_event.energy_j - spring.energy_j).max(0.0);
            work.stored_energy_j = spring.energy_j;
            work.body_work_j = -tension * route.speed_m_s * dt;
            work.drive_work_j = tension * drive.speed_m_s * dt;
            let elastic_rate = rate - sliding_rate;
            let damping_power = if working > 0.0 {
                ((tension - spring.elastic_n) * elastic_rate).max(0.0)
            } else {
                0.0
            };
            work.loss_j = (damping_power + tension * sliding_rate) * dt;
            let slip = sliding_rate * dt;
            step.slip_m = Some(confirmed + correction + slip);
            step.deformation_m = Some(working);
            step.free_length_m = Some(drive.length_m + payout);
            payout += slip;
            if tension > 0.0 {
                step.route_forces_n = route.forces(tension);
                reaction.loads =
                    body_loads(&self.spec.bodies, &input.points, &step.route_forces_n)?;
                step.body_loads = reaction.loads.clone();
                step.drive_effort = drive.effort.map(|(id, slope)| (id, tension * slope));
            }
            reaction.tension_n = tension;
        }
        finite(
            [sliding_rate, payout]
                .into_iter()
                .chain(step.slip_m)
                .chain(step.free_length_m)
                .chain(step.drive_effort.map(|(_, effort)| effort)),
        )?;
        work_is_finite(work)?;
        step.payout_m = Some(payout);
        step.reaction = analyzed(reaction, "натяжение вдоль ломаной трассы на левой границе");
        step.work = analyzed(work, "работа натяжения на телах и сопряжённой координате");
        step.calibration = self.calibration(e_before, rate, payout, &drive);
        Ok((step, payout))
    }
}

impl StepModel for TensionRoute {
    type Input = InterfaceInput;
    type Output = InterfaceStep;

    fn advance(
        &mut self,
        interval: Interval,
        input: &InterfaceInput,
        _: &mut Rng,
    ) -> Result<InterfaceStep, ProcessError> {
        let InterfaceInput::Tension(input) = input else {
            return Err(invalid("натяжная связь требует кинематику трассы"));
        };
        let (dt, event) = self
            .status
            .begin(interval, &self.spec.name, input.event.as_ref())?;
        let (step, payout) = self.solve(interval, dt, input, event)?;
        self.payout_m = payout;
        self.status.commit(&step);
        Ok(step)
    }
}

struct FreeMotion {
    length_m: f64,
    speed_m_s: f64,
    effort: Option<(ChannelId, f64)>,
    supported: bool,
}

struct Route {
    length_m: f64,
    speed_m_s: f64,
    units: Vec<Vector>,
}

impl Route {
    fn new(bodies: &[String], points: &[RoutePoint]) -> Result<Self, ProcessError> {
        if points.len() != bodies.len() {
            return Err(invalid("число точек не соответствует описанию трассы"));
        }
        let mut centers = BTreeMap::new();
        for (p, body) in points.iter().zip(bodies) {
            finite(p.point_m.into_iter().chain(p.com_m).chain(p.velocity_m_s))?;
            if centers
                .insert(body, p.com_m)
                .is_some_and(|prev| prev != p.com_m)
            {
                return Err(invalid("точки одного тела используют разные центры масс"));
            }
        }
        let mut route = Self {
            length_m: 0.0,
            speed_m_s: 0.0,
            units: Vec::new(),
        };
        for pair in points.windows(2) {
            let [a, b] = [&pair[0], &pair[1]];
            let (l, n) = length_and_unit(sub(b.point_m, a.point_m))
                .ok_or_else(|| invalid("нулевой или неконечный отрезок трассы"))?;
            route.length_m += l;
            route.speed_m_s += dot(n, sub(b.velocity_m_s, a.velocity_m_s));
            route.units.push(n);
        }
        finite([route.length_m, route.speed_m_s])?;
        Ok(route)
    }

    fn forces(&self, tension: f64) -> Vec<Vector> {
        // Каждая ветвь тянет оба своих конца. Внутренняя точка получает
        // векторную сумму двух ветвей по минус градиенту длины.
        (0..=self.units.len())
            .map(|i| {
                let forward = self.units.get(i).copied().unwrap_or([0.0; 3]);
                let backward = if i == 0 { [0.0; 3] } else { self.units[i - 1] };
                scale(sub(forward, backward), tension)
            })
            .collect()
    }
}

fn body_loads(
    names: &[String],
    points: &[RoutePoint],
    forces: &[Vector],
) -> Result<Vec<crate::BodyLoad>, ProcessError> {
    use std::collections::btree_map::Entry;
    let mut loads: BTreeMap<&String, crate::BodyLoad> = BTreeMap::new();
    for ((body, point), &force) in names.iter().zip(points).zip(forces) {
        let wrench = load(body, point.point_m, point.com_m, force, [0.0; 3])?;
        match loads.entry(body) {
            Entry::Vacant(entry) => {
                entry.insert(wrench);
            }
            Entry::Occupied(mut entry) => {
                let total = entry.get_mut();
                total.force_n = add(total.force_n, wrench.force_n);
                total.torque_nm = add(total.torque_nm, wrench.torque_nm);
                finite(total.force_n.into_iter().chain(total.torque_nm))?;
            }
        }
    }
    Ok(loads.into_values().collect())
}

fn extension_at_limit(curve: &ForceCurve, cap: f64) -> Option<f64> {
    for pair in curve.points().windows(2) {
        let [a, b] = [&pair[0], &pair[1]];
        let left = a.force_n.nominal;
        let right = b.force_n.nominal;
        if cap < right {
            let x0 = a.displacement_m.nominal;
            let x1 = b.displacement_m.nominal;
            return Some(x0 + (x1 - x0) * ((cap - left) / (right - left)));
        }
    }
    None
}
