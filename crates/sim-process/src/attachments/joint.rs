use std::f64::consts::{PI, TAU};

use sim_signal::{Rng, Time};
use sim_world::MountKind;

use crate::work::{HazardEvidence, HazardKind};
use crate::{Analysis, BodyLoad, Interval, ProcessError, SourcedRange, StepModel};

use super::math::{
    analyzed, angle, check_interval, cross, dot, finite, invalid, load, norm, positive, range,
    scale, sub, unavailable,
};
use super::{
    AttachmentAction, AttachmentBinding, AttachmentEvent, AttachmentKind, AttachmentOutcome,
    AttachmentRequest, AttachmentState, ForceCurve,
};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MountPort {
    pub name: String,
    pub body: String,
    pub kind: MountKind,
    pub interface: String,
}

/// Все векторы в системе мира. Оси сонаправлены при правильной стыковке.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MountMotion {
    pub point_m: [f64; 3],
    pub com_m: [f64; 3],
    pub axis: [f64; 3],
    pub velocity_m_s: [f64; 3],
    pub angular_velocity_rad_s: [f64; 3],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointWrench {
    /// Реакция на второй стык в мировой системе.
    pub force_n: [f64; 3],
    /// Момент относительно второго стыка. Плечо до центра масс учитывается отдельно.
    pub torque_nm: [f64; 3],
}

impl JointWrench {
    fn validate(self) -> Result<(), ProcessError> {
        finite(self.force_n.into_iter().chain(self.torque_nm))
    }
}

/// Линейная огибающая взаимодействия силы и момента `|F|/Fmax + |M|/Mmax <= 1`.
/// Предельные значения должны относиться к совместной нагрузке.
#[derive(Clone, Debug, PartialEq)]
pub struct WrenchCapacity {
    pub force_n: SourcedRange,
    pub torque_nm: SourcedRange,
}

impl WrenchCapacity {
    /// Линейная огибающая совместной силы и момента. Порядок: наиболее
    /// прочное и наименее прочное соединение в заданных диапазонах.
    pub fn utilization(&self, wrench: JointWrench) -> Result<[f64; 2], ProcessError> {
        positive(&self.force_n)?;
        positive(&self.torque_nm)?;
        wrench.validate()?;
        let (force, torque) = (norm(wrench.force_n), norm(wrench.torque_nm));
        let values = [
            force / self.force_n.max + torque / self.torque_nm.max,
            force / self.force_n.min + torque / self.torque_nm.min,
        ];
        finite(values)?;
        Ok(values)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct CaptureLimits {
    pub gap_m: SourcedRange,
    pub lateral_m: SourcedRange,
    pub axis_error_rad: SourcedRange,
    pub speed_m_s: SourcedRange,
    pub angular_speed_rad_s: SourcedRange,
    pub required_travel_m: SourcedRange,
    pub release_force_n: SourcedRange,
    pub release_torque_nm: SourcedRange,
    pub capacity: WrenchCapacity,
}

#[derive(Clone, Debug, PartialEq)]
pub struct LatchLaw {
    pub free_seat_m: SourcedRange,
    pub engaged_seat_m: SourcedRange,
    pub engagement_m: SourcedRange,
    pub release_m: SourcedRange,
    pub minimum_stroke_m: SourcedRange,
    pub maximum_stroke_m: SourcedRange,
    pub stiffness_n_m: SourcedRange,
    pub damping_ns_m: SourcedRange,
    pub forward_contact_n: SourcedRange,
    pub reverse_contact_n: SourcedRange,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ThreadLaw {
    /// Подписанный ход за полный оборот, включая число заходов.
    pub lead_m_per_turn: SourcedRange,
    /// Положение точки Б при входе в зацепление и нулевом угле.
    pub entry_m: SourcedRange,
    pub depth_m: SourcedRange,
    pub phase_error_rad: SourcedRange,
    pub maximum_elastic_error_m: SourcedRange,
    pub stiffness_n_m: SourcedRange,
    pub damping_ns_m: SourcedRange,
    pub torsional_drag_nms: SourcedRange,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AdhesiveLaw {
    pub cure_s: SourcedRange,
    /// Минимум и максимум непрерывного осевого прижима для образования связи.
    pub compression_n: SourcedRange,
    /// Диапазон температур образования связи.
    pub temperature_k: SourcedRange,
}

#[derive(Clone, Debug, PartialEq)]
pub enum AttachmentLaw {
    /// Осевая измеренная характеристика с явной нулевой силой на границе поля.
    Magnetic {
        pull: ForceCurve,
    },
    Latch(LatchLaw),
    Threaded(ThreadLaw),
    Adhesive(AdhesiveLaw),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThreadMotion {
    /// Непрерывный относительный угол, от известной фазы входа, знак вокруг оси первого стыка.
    pub angle_rad: f64,
    /// Производная этого же угла.
    pub angular_speed_rad_s: f64,
}

/// Границы измерений за весь завершённый интервал, включая его начало.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CureObservation {
    pub compression_n: [f64; 2],
    pub temperature_k: [f64; 2],
    /// Верхние границы модуля зазора, радиального и углового рассогласования.
    pub gap_m: f64,
    pub lateral_m: f64,
    pub axis_error_rad: f64,
    pub speed_m_s: f64,
    pub angular_speed_rad_s: f64,
}

/// Рассогласование относительно фактической опорной позы ограничения, разрешённого сборщиком.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WeldResidual {
    pub translation_m: f64,
    pub rotation_rad: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AttachmentInput {
    pub mounts: [MountMotion; 2],
    pub available_travel_m: Analysis<f64>,
    /// Нагрузка за завершённый интервал. Для магнита это внешнее тяговое усилие
    /// на второй стык, без силы собственного поля. Для активного weld сюда
    /// передаётся фактическая реакция ограничения до следующего `forward`.
    pub measured_wrench: Analysis<JointWrench>,
    /// Подписанная проекция внешнего контакта, приводящего защёлку. Нужна и при нулевой силе.
    pub latch_contact_n: Analysis<f64>,
    pub thread: Option<ThreadMotion>,
    pub cure: Analysis<CureObservation>,
    pub weld_residual: Option<WeldResidual>,
    pub command: Option<AttachmentRequest>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinkEffect {
    None,
    Weld(bool),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AttachmentCause {
    Captured,
    CureStarted,
    CureInterrupted,
    Released,
    Overload,
    PullOff,
    Stripped,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AttachmentChange {
    pub request: AttachmentRequest,
    pub from: AttachmentState,
    pub to: AttachmentState,
    pub effect: LinkEffect,
    pub cause: AttachmentCause,
    /// Изменение запасённой энергии при смене режима, учитываемое как выделившаяся энергия.
    pub released_energy_j: Analysis<f64>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AttachmentReaction {
    pub gap_m: f64,
    pub lateral_m: f64,
    pub axis_error_rad: f64,
    pub measured: JointWrench,
    /// Сила модели на второй стык для следующего интервала.
    pub applied: JointWrench,
    pub stored_energy_j: Analysis<f64>,
    pub dissipated_power_w: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AttachmentStep {
    pub at: Time,
    pub state: AttachmentState,
    pub failed: bool,
    pub reaction: Analysis<AttachmentReaction>,
    /// Нагрузки для следующего интервала. При наличии перехода применять атомарно вместе с ним.
    pub loads: Vec<BodyLoad>,
    pub change: Option<AttachmentChange>,
    /// Отказ или недоказанный запрос, обнаруженный до обращения к движку.
    pub event: Option<AttachmentEvent>,
    pub hazards: Vec<HazardEvidence>,
}

#[derive(Clone, Debug)]
pub struct AttachmentModel {
    binding: AttachmentBinding,
    ports: [MountPort; 2],
    limits: CaptureLimits,
    law: AttachmentLaw,
    cure_duration: Option<Time>,
    state: AttachmentState,
    failed: bool,
    cured_ns: u64,
    previous: Option<Time>,
    pending: Option<(Time, AttachmentChange)>,
}

impl AttachmentModel {
    pub fn new(
        binding: AttachmentBinding,
        ports: [MountPort; 2],
        limits: CaptureLimits,
        law: AttachmentLaw,
    ) -> Result<Self, ProcessError> {
        if binding.name.trim().is_empty()
            || ports.iter().any(|p| {
                p.name.trim().is_empty()
                    || p.body.trim().is_empty()
                    || p.interface.trim().is_empty()
                    || p.kind != MountKind::Mechanical
            })
            || binding.mounts != [ports[0].name.clone(), ports[1].name.clone()]
            || ports[0].name == ports[1].name
            || ports[0].body == ports[1].body
            || ports[0].interface != ports[1].interface
        {
            return Err(invalid(
                "несовместимые или неразрешённые механические стыки",
            ));
        }
        limits.validate()?;
        let (kind, cure_duration) = law.validate(&limits)?;
        if kind != binding.kind {
            return Err(invalid("вид крепления и закон не согласованы"));
        }
        Ok(Self {
            binding,
            ports,
            limits,
            law,
            cure_duration,
            state: AttachmentState::Free,
            failed: false,
            cured_ns: 0,
            previous: None,
            pending: None,
        })
    }

    pub fn state(&self) -> AttachmentState {
        self.state
    }

    pub fn binding(&self) -> &AttachmentBinding {
        &self.binding
    }

    pub fn hazards(&self) -> Vec<HazardEvidence> {
        let mut kinds = vec![
            HazardKind::Pinch,
            HazardKind::Impact,
            HazardKind::StoredEnergy,
        ];
        if self.binding.kind == AttachmentKind::Threaded {
            kinds.push(HazardKind::ThreadFailure);
        }
        kinds
            .into_iter()
            .map(|kind| HazardEvidence {
                subject: self.binding.name.clone(),
                kind,
                coverage: unavailable(
                    "модель крепления сама по себе не устанавливает границу опасности",
                ),
            })
            .collect()
    }

    /// Подтверждает результат атомарного применения запроса, без подмены момента или состояния.
    pub fn confirm(&mut self, event: &AttachmentEvent) -> Result<(), ProcessError> {
        let Some((at, pending)) = &self.pending else {
            return Err(invalid("нет ожидающего применения перехода"));
        };
        if event.request != pending.request || event.evaluated_at != *at {
            return Err(invalid(
                "подтверждение относится к другому запросу или такту",
            ));
        }
        match &event.outcome {
            AttachmentOutcome::Transition { from, to }
                if *from == pending.from && *to == pending.to =>
            {
                self.state = *to;
                self.failed |= matches!(pending.cause, AttachmentCause::Stripped)
                    || (pending.cause == AttachmentCause::Overload
                        && self.binding.kind == AttachmentKind::Latch);
                self.cured_ns = 0;
            }
            AttachmentOutcome::Rejected { reason } | AttachmentOutcome::NotAnalyzed { reason }
                if !reason.trim().is_empty() =>
            {
                self.cured_ns = 0
            }
            _ => {
                return Err(invalid(
                    "подтверждение не соответствует предложенному переходу",
                ));
            }
        }
        self.pending = None;
        Ok(())
    }
}

impl CaptureLimits {
    fn validate(&self) -> Result<(), ProcessError> {
        for quantity in [
            &self.gap_m,
            &self.lateral_m,
            &self.speed_m_s,
            &self.angular_speed_rad_s,
            &self.required_travel_m,
            &self.release_force_n,
            &self.release_torque_nm,
        ] {
            range(quantity, 0.0, f64::MAX)?;
        }
        range(&self.axis_error_rad, 0.0, PI)?;
        positive(&self.capacity.force_n)?;
        positive(&self.capacity.torque_nm)?;
        if self.release_force_n.max > self.capacity.force_n.min
            || self.release_torque_nm.max > self.capacity.torque_nm.min
        {
            return Err(invalid(
                "порог штатного отпускания превышает прочность стыка",
            ));
        }
        Ok(())
    }
}

impl AttachmentLaw {
    fn validate(
        &self,
        limits: &CaptureLimits,
    ) -> Result<(AttachmentKind, Option<Time>), ProcessError> {
        match self {
            Self::Magnetic { pull } => {
                let points = pull.points();
                if points[0].force_n.min <= 0.0
                    || points.last().unwrap().force_n.max != 0.0
                    || points
                        .windows(2)
                        .any(|p| p[0].force_n.min < p[1].force_n.max)
                    || limits.gap_m.max >= points.last().unwrap().displacement_m.min
                    || pull.at(limits.gap_m.max)?.0 <= 0.0
                {
                    return Err(invalid(
                        "магнитная характеристика должна убывать к нулю за границей захвата",
                    ));
                }
                Ok((AttachmentKind::Magnetic, None))
            }
            Self::Latch(law) => {
                for position in [
                    &law.free_seat_m,
                    &law.engaged_seat_m,
                    &law.engagement_m,
                    &law.release_m,
                    &law.minimum_stroke_m,
                    &law.maximum_stroke_m,
                ] {
                    range(position, -f64::MAX, f64::MAX)?;
                }
                for quantity in [
                    &law.stiffness_n_m,
                    &law.forward_contact_n,
                    &law.reverse_contact_n,
                ] {
                    positive(quantity)?;
                }
                range(&law.damping_ns_m, 0.0, f64::MAX)?;
                let mid_min = law.free_seat_m.min * 0.5 + law.engaged_seat_m.min * 0.5;
                let mid_max = law.free_seat_m.max * 0.5 + law.engaged_seat_m.max * 0.5;
                if !(law.minimum_stroke_m.max <= law.free_seat_m.min
                    && law.free_seat_m.max < law.release_m.min
                    && law.release_m.max <= mid_min
                    && mid_max <= law.engagement_m.min
                    && law.engagement_m.max < law.engaged_seat_m.min
                    && law.engaged_seat_m.max <= law.maximum_stroke_m.min)
                {
                    return Err(invalid(
                        "пороги и упоры защёлки нарушают порядок или создают энергию",
                    ));
                }
                Ok((AttachmentKind::Latch, None))
            }
            Self::Threaded(law) => {
                range(&law.lead_m_per_turn, -f64::MAX, f64::MAX)?;
                range(&law.entry_m, -f64::MAX, f64::MAX)?;
                range(&law.phase_error_rad, 0.0, PI)?;
                for quantity in [
                    &law.depth_m,
                    &law.maximum_elastic_error_m,
                    &law.stiffness_n_m,
                ] {
                    positive(quantity)?;
                }
                for quantity in [&law.damping_ns_m, &law.torsional_drag_nms] {
                    range(quantity, 0.0, f64::MAX)?;
                }
                if (law.lead_m_per_turn.min <= 0.0 && law.lead_m_per_turn.max >= 0.0)
                    || law.lead_m_per_turn.nominal / TAU == 0.0
                    || law.maximum_elastic_error_m.max >= law.depth_m.min
                {
                    return Err(invalid(
                        "ход резьбы или допустимое упругое смещение не определены",
                    ));
                }
                Ok((AttachmentKind::Threaded, None))
            }
            Self::Adhesive(law) => {
                positive(&law.cure_s)?;
                positive(&law.compression_n)?;
                positive(&law.temperature_k)?;
                let duration = Time::from_secs(law.cure_s.max)
                    .filter(|t| *t > Time::ZERO)
                    .ok_or_else(|| invalid("время отверждения вне шкалы времени"))?;
                // Округление шкалы не должно сократить требуемую длительность.
                let duration = if duration.as_secs() < law.cure_s.max {
                    Time(
                        duration
                            .0
                            .checked_add(1)
                            .ok_or_else(|| invalid("переполнение времени отверждения"))?,
                    )
                } else {
                    duration
                };
                Ok((AttachmentKind::Adhesive, Some(duration)))
            }
        }
    }
}

#[derive(Clone, Copy)]
struct Geometry {
    gap: f64,
    radial: f64,
    tilt: f64,
    speed: f64,
    spin: f64,
    axial_speed: f64,
}

impl Geometry {
    fn from_mounts(mounts: [MountMotion; 2]) -> Result<Self, ProcessError> {
        for site in mounts {
            finite(
                site.point_m
                    .into_iter()
                    .chain(site.com_m)
                    .chain(site.velocity_m_s)
                    .chain(site.angular_velocity_rad_s)
                    .chain(site.axis),
            )?;
            // Запас на округление трёх компонент единичного вектора после преобразования позы.
            if (norm(site.axis) - 1.0).abs() > 8.0 * f64::EPSILON {
                return Err(invalid("ось стыка должна быть единичной"));
            }
        }
        let [a, b] = mounts;
        let delta = sub(b.point_m, a.point_m);
        let velocity = sub(b.velocity_m_s, a.velocity_m_s);
        let gap = dot(delta, a.axis);
        let result = Self {
            gap,
            radial: norm(sub(delta, scale(a.axis, gap))),
            tilt: angle(a.axis, b.axis),
            speed: norm(velocity),
            spin: norm(sub(b.angular_velocity_rad_s, a.angular_velocity_rad_s)),
            axial_speed: dot(velocity, a.axis)
                + dot(delta, cross(a.angular_velocity_rad_s, a.axis)),
        };
        finite([
            result.gap,
            result.radial,
            result.tilt,
            result.speed,
            result.spin,
            result.axial_speed,
        ])?;
        Ok(result)
    }

    fn aligned(self, limits: &CaptureLimits) -> bool {
        self.radial <= limits.lateral_m.min && self.tilt <= limits.axis_error_rad.min
    }

    fn capturable(self, limits: &CaptureLimits, gap: f64) -> bool {
        self.aligned(limits)
            && gap.abs() <= limits.gap_m.min
            && self.speed <= limits.speed_m_s.min
            && self.spin <= limits.angular_speed_rad_s.min
    }
}

#[derive(Clone, Copy)]
struct Decision {
    to: AttachmentState,
    effect: LinkEffect,
    cause: AttachmentCause,
}

fn decision(to: AttachmentState, cause: AttachmentCause) -> Decision {
    Decision {
        to,
        effect: LinkEffect::None,
        cause,
    }
}

#[derive(Clone, Copy)]
struct LawForce {
    axial: f64,
    torque: f64,
    energy: Option<f64>,
    loss_w: f64,
}

impl LawForce {
    // Связь без податливости не даёт данных о запасённой в ней энергии.
    const RIGID: Self = Self {
        axial: 0.0,
        torque: 0.0,
        energy: None,
        loss_w: 0.0,
    };
    const NONE: Self = Self {
        energy: Some(0.0),
        ..Self::RIGID
    };
}

type Evaluation = (LawForce, Option<Decision>);

struct Update {
    evaluation: Analysis<Evaluation>,
    refusal: Option<&'static str>,
    cured_ns: u64,
}

impl Update {
    fn measured(force: LawForce, transition: Option<Decision>) -> Self {
        Self {
            evaluation: analyzed((force, transition), "номинальный закон стыка"),
            refusal: None,
            cured_ns: 0,
        }
    }

    fn refused(force: LawForce, reason: &'static str) -> Self {
        Self {
            refusal: Some(reason),
            ..Self::measured(force, None)
        }
    }

    fn unavailable(reason: &str) -> Self {
        Self {
            evaluation: unavailable(reason),
            refusal: None,
            cured_ns: 0,
        }
    }
}

impl AttachmentInput {
    fn validate(&self) -> Result<(), ProcessError> {
        if let Analysis::Analyzed { value, .. } = self.available_travel_m {
            finite([value])?;
            if value < 0.0 {
                return Err(invalid("отрицательный доступный ход"));
            }
        }
        if let Analysis::Analyzed { value, .. } = self.measured_wrench {
            value.validate()?;
        }
        if let Analysis::Analyzed { value, .. } = self.latch_contact_n {
            finite([value])?;
        }
        if let Some(t) = self.thread {
            finite([t.angle_rad, t.angular_speed_rad_s])?;
        }
        if let Analysis::Analyzed { value: c, .. } = self.cure {
            finite(c.compression_n.into_iter().chain(c.temperature_k).chain([
                c.gap_m,
                c.lateral_m,
                c.axis_error_rad,
                c.speed_m_s,
                c.angular_speed_rad_s,
            ]))?;
            if c.compression_n[0] < 0.0
                || c.compression_n[0] > c.compression_n[1]
                || c.temperature_k[0] <= 0.0
                || c.temperature_k[0] > c.temperature_k[1]
                || [
                    c.gap_m,
                    c.lateral_m,
                    c.axis_error_rad,
                    c.speed_m_s,
                    c.angular_speed_rad_s,
                ]
                .iter()
                .any(|v| *v < 0.0)
            {
                return Err(invalid("неверные границы наблюдения за отверждением"));
            }
        }
        if let Some(w) = self.weld_residual {
            finite([w.translation_m, w.rotation_rad])?;
            if w.translation_m < 0.0 || !(0.0..=PI).contains(&w.rotation_rad) {
                return Err(invalid("неверное рассогласование опорной позы ограничения"));
            }
        }
        Ok(())
    }
}

impl AttachmentModel {
    fn evaluate(
        &self,
        dt_ns: u64,
        input: &AttachmentInput,
        g: Geometry,
        wrench: JointWrench,
    ) -> Result<Update, ProcessError> {
        let command = input.command.as_ref().map(|c| c.action);
        let attached = self.state == AttachmentState::Attached;
        let magnitude = (norm(wrench.force_n), norm(wrench.torque_nm));
        finite([magnitude.0, magnitude.1])?;
        if self.failed {
            return Ok(if command.is_some() {
                Update::refused(LawForce::NONE, "стык разрушен")
            } else {
                Update::measured(LawForce::NONE, None)
            });
        }

        let [robust, worst] = self.limits.capacity.utilization(wrench)?;
        // Единица — граница нормированной линейной огибающей.
        let definitely_over = robust > 1.0;
        let possibly_over = worst > 1.0;
        if attached && definitely_over && !matches!(self.law, AttachmentLaw::Magnetic { .. }) {
            let cause = if self.binding.kind == AttachmentKind::Threaded {
                AttachmentCause::Stripped
            } else {
                AttachmentCause::Overload
            };
            let mut change = decision(AttachmentState::Free, cause);
            if self.binding.kind == AttachmentKind::Adhesive {
                change.effect = LinkEffect::Weld(false);
            }
            return Ok(Update::measured(LawForce::NONE, Some(change)));
        }
        if possibly_over && !definitely_over {
            return Ok(Update::unavailable(
                "нагрузка пересекает диапазон прочности стыка",
            ));
        }
        let release = magnitude.0 <= self.limits.release_force_n.min
            && magnitude.1 <= self.limits.release_torque_nm.min;
        let capture = match input.available_travel_m {
            Analysis::Analyzed { value, .. } => {
                Some(value >= self.limits.required_travel_m.max && !possibly_over)
            }
            Analysis::NotAnalyzed { .. } => None,
        };
        match &self.law {
            AttachmentLaw::Magnetic { pull } => {
                self.magnetic(pull, input, g, wrench, capture, release, definitely_over)
            }
            AttachmentLaw::Latch(law) => self.latch(law, input, g, capture, release),
            AttachmentLaw::Threaded(law) => self.threaded(law, input, g, capture, release),
            AttachmentLaw::Adhesive(law) => self.adhesive(law, input, g, dt_ns, capture, release),
        }
    }

    #[expect(
        clippy::too_many_arguments,
        reason = "закон получает наблюдение, геометрию и отдельные механические допуски"
    )]
    fn magnetic(
        &self,
        pull: &ForceCurve,
        input: &AttachmentInput,
        g: Geometry,
        wrench: JointWrench,
        capture: Option<bool>,
        release: bool,
        overload: bool,
    ) -> Result<Update, ProcessError> {
        let command = input.command.as_ref().map(|c| c.action);
        let attached = self.state == AttachmentState::Attached;
        let on = self.limits.gap_m.min;
        let cutoff = pull.maximum_m();
        if g.gap < 0.0 || (g.gap < cutoff && !g.aligned(&self.limits)) {
            return Ok(Update::unavailable(
                "геометрия вне калибровки магнитного поля",
            ));
        }
        if attached && !g.aligned(&self.limits) {
            return Ok(Update::unavailable(
                "магнитный отрыв со смещением вне калибровки",
            ));
        }
        // Вне поля действует заданная в данных нулевая концевая сила.
        let (force, energy) = pull.at(g.gap.min(cutoff))?;
        let model = LawForce {
            axial: -force,
            energy: Some(energy),
            ..LawForce::NONE
        };
        let tensile = dot(wrench.force_n, input.mounts[0].axis);
        finite([tensile])?;
        let shear = norm(cross(wrench.force_n, input.mounts[0].axis));
        finite([shear])?;
        if (attached || command == Some(AttachmentAction::Attach))
            && (shear != 0.0 || norm(wrench.torque_nm) != 0.0)
        {
            return Ok(Update::unavailable(
                "осевая магнитная модель требует отдельной опоры для сдвига и момента",
            ));
        }
        if attached && (overload || g.gap >= cutoff || tensile > force) {
            return Ok(Update::measured(
                model,
                Some(decision(AttachmentState::Free, AttachmentCause::PullOff)),
            ));
        }
        if let Some(action) = command {
            return Ok(match (action, attached) {
                (AttachmentAction::Attach, false) if capture.is_none() => {
                    Update::unavailable("нет наблюдения доступного хода")
                }
                (AttachmentAction::Attach, false)
                    if capture == Some(true)
                        && g.capturable(&self.limits, g.gap)
                        && tensile <= force =>
                {
                    Update::measured(
                        model,
                        Some(decision(
                            AttachmentState::Attached,
                            AttachmentCause::Captured,
                        )),
                    )
                }
                (AttachmentAction::Release, true)
                    if release && g.gap >= on && g.axial_speed > 0.0 =>
                {
                    Update::measured(
                        model,
                        Some(decision(AttachmentState::Free, AttachmentCause::Released)),
                    )
                }
                _ => Update::refused(
                    model,
                    "условия захвата или отпускания магнитного стыка не выполнены",
                ),
            });
        }
        Ok(Update::measured(model, None))
    }

    fn latch(
        &self,
        law: &LatchLaw,
        input: &AttachmentInput,
        g: Geometry,
        capture: Option<bool>,
        release: bool,
    ) -> Result<Update, ProcessError> {
        if !g.aligned(&self.limits)
            || g.gap < law.minimum_stroke_m.max
            || g.gap > law.maximum_stroke_m.min
        {
            return Ok(Update::unavailable(
                "ход защёлки или геометрия вне области калибровки",
            ));
        }
        let Analysis::Analyzed { value: contact, .. } = input.latch_contact_n else {
            return Ok(Update::unavailable(
                "нет измерения приводящего контакта защёлки",
            ));
        };
        let attached = self.state == AttachmentState::Attached;
        let command = input.command.as_ref().map(|c| c.action);
        let current = self.latch_force(law, g, attached);
        let forward = !attached
            && g.gap >= law.engagement_m.max
            && contact >= law.forward_contact_n.max
            && command != Some(AttachmentAction::Release);
        let reverse = attached
            && g.gap <= law.release_m.min
            && contact <= -law.reverse_contact_n.max
            && command != Some(AttachmentAction::Attach);
        if forward && capture.is_none() {
            return Ok(Update::unavailable("нет наблюдения доступного хода"));
        }
        if forward && g.capturable(&self.limits, g.gap) && capture == Some(true) {
            return Ok(Update::measured(
                self.latch_force(law, g, true),
                Some(decision(
                    AttachmentState::Attached,
                    AttachmentCause::Captured,
                )),
            ));
        }
        if reverse && release {
            return Ok(Update::measured(
                self.latch_force(law, g, false),
                Some(decision(AttachmentState::Free, AttachmentCause::Released)),
            ));
        }
        if command.is_some() || forward || reverse {
            return Ok(Update::refused(
                current,
                "нужны реальный контакт, допустимый ход и нагрузка отпускания",
            ));
        }
        Ok(Update::measured(current, None))
    }

    fn latch_force(&self, law: &LatchLaw, g: Geometry, attached: bool) -> LawForce {
        let seat = if attached {
            law.engaged_seat_m.nominal
        } else {
            law.free_seat_m.nominal
        };
        let dx = g.gap - seat;
        let elastic = law.stiffness_n_m.nominal * dx;
        let drag = law.damping_ns_m.nominal * g.axial_speed;
        LawForce {
            axial: -elastic - drag,
            energy: Some(0.5 * elastic * dx),
            loss_w: drag * g.axial_speed,
            ..LawForce::NONE
        }
    }

    fn threaded(
        &self,
        law: &ThreadLaw,
        input: &AttachmentInput,
        g: Geometry,
        capture: Option<bool>,
        release: bool,
    ) -> Result<Update, ProcessError> {
        let command = input.command.as_ref().map(|c| c.action);
        let attached = self.state == AttachmentState::Attached;
        if !attached && command.is_none() {
            return Ok(Update::measured(LawForce::NONE, None));
        }
        let Some(turn) = input.thread else {
            return Ok(Update::unavailable("нет непрерывной фазы резьбы"));
        };
        let per_rad = law.lead_m_per_turn.nominal / TAU;
        let depth = g.gap - law.entry_m.nominal;
        let nominal_depth = per_rad * turn.angle_rad;
        let error = depth - nominal_depth;
        finite([depth, nominal_depth, error])?;
        if !attached {
            if capture.is_none() {
                return Ok(Update::unavailable("нет наблюдения доступного хода"));
            }
            return Ok(
                if command == Some(AttachmentAction::Attach)
                    && capture == Some(true)
                    && g.capturable(&self.limits, depth)
                    && depth >= 0.0
                    && depth <= law.depth_m.min
                    && turn.angle_rad.abs() <= law.phase_error_rad.min
                    && error == 0.0
                    && g.axial_speed == per_rad * turn.angular_speed_rad_s
                {
                    Update::measured(
                        LawForce::NONE,
                        Some(decision(
                            AttachmentState::Attached,
                            AttachmentCause::Captured,
                        )),
                    )
                } else {
                    Update::refused(
                        LawForce::NONE,
                        "резьба не совпала по фазе, ходу или относительному движению",
                    )
                },
            );
        }
        let release_window = law.phase_error_rad.min * per_rad.abs();
        if !g.aligned(&self.limits) || depth.max(nominal_depth) > law.depth_m.min {
            return Ok(Update::unavailable(
                "зацепление вне калибровки оси или глубины",
            ));
        }
        if error.abs() > law.maximum_elastic_error_m.max {
            return Ok(Update::measured(
                LawForce::NONE,
                Some(decision(AttachmentState::Free, AttachmentCause::Stripped)),
            ));
        }
        if error.abs() > law.maximum_elastic_error_m.min {
            return Ok(Update::unavailable(
                "упругая невязка пересекает диапазон срыва резьбы",
            ));
        }
        let backed_out = depth <= 0.0
            && nominal_depth <= 0.0
            && depth >= -release_window
            && nominal_depth >= -release_window;
        if backed_out && release && turn.angular_speed_rad_s * per_rad < 0.0 {
            return Ok(Update::measured(
                LawForce::NONE,
                Some(decision(AttachmentState::Free, AttachmentCause::Released)),
            ));
        }
        if depth < 0.0 || nominal_depth < 0.0 {
            return Ok(Update::unavailable(
                "выход из зацепления за пределами штатного отпускания",
            ));
        }
        let error_speed = g.axial_speed - per_rad * turn.angular_speed_rad_s;
        let elastic = law.stiffness_n_m.nominal * error;
        let damping = law.damping_ns_m.nominal * error_speed;
        let axial = -elastic - damping;
        let drag = law.torsional_drag_nms.nominal * turn.angular_speed_rad_s;
        // Градиент энергии от z - a·φ даёт Fz и Mφ = -a·Fz.
        let model = LawForce {
            axial,
            torque: -per_rad * axial - drag,
            energy: Some(0.5 * elastic * error),
            loss_w: damping * error_speed + drag * turn.angular_speed_rad_s,
        };
        Ok(if command.is_some() {
            Update::refused(
                model,
                "резьба отпускается обратным ходом до выхода без нагрузки",
            )
        } else {
            Update::measured(model, None)
        })
    }

    fn adhesive(
        &self,
        law: &AdhesiveLaw,
        input: &AttachmentInput,
        g: Geometry,
        dt_ns: u64,
        capture: Option<bool>,
        release: bool,
    ) -> Result<Update, ProcessError> {
        let command = input.command.as_ref().map(|c| c.action);
        if self.state == AttachmentState::Attached {
            return Ok(if command.is_some() {
                Update::refused(
                    LawForce::RIGID,
                    "отверждённое соединение размыкается при разрушении",
                )
            } else {
                Update::measured(LawForce::RIGID, None)
            });
        }
        let curing = self.state == AttachmentState::Engaging;
        if command == Some(AttachmentAction::Release) {
            return Ok(if curing && release {
                Update::measured(
                    LawForce::NONE,
                    Some(decision(
                        AttachmentState::Free,
                        AttachmentCause::CureInterrupted,
                    )),
                )
            } else {
                Update::refused(LawForce::NONE, "отмена отверждения под нагрузкой запрещена")
            });
        }
        if !curing && command.is_none() {
            return Ok(Update::measured(LawForce::NONE, None));
        }
        let Analysis::Analyzed { value: cure, .. } = input.cure else {
            return Ok(Update::unavailable(
                "нет границ наблюдений за отверждением на всём интервале",
            ));
        };
        if capture.is_none() {
            return Ok(Update::unavailable("нет наблюдения доступного хода"));
        }
        let qualified = capture == Some(true)
            && g.capturable(&self.limits, g.gap)
            && cure.compression_n[0] >= law.compression_n.min
            && cure.compression_n[1] <= law.compression_n.max
            && cure.temperature_k[0] >= law.temperature_k.min
            && cure.temperature_k[1] <= law.temperature_k.max
            && cure.gap_m <= self.limits.gap_m.min
            && cure.lateral_m <= self.limits.lateral_m.min
            && cure.axis_error_rad <= self.limits.axis_error_rad.min
            && cure.speed_m_s <= self.limits.speed_m_s.min
            && cure.angular_speed_rad_s <= self.limits.angular_speed_rad_s.min;
        if !qualified {
            return Ok(if curing {
                Update::measured(
                    LawForce::NONE,
                    Some(decision(
                        AttachmentState::Free,
                        AttachmentCause::CureInterrupted,
                    )),
                )
            } else {
                Update::refused(
                    LawForce::NONE,
                    "условия образования клеевой связи не выполнены",
                )
            });
        }
        if !curing {
            return Ok(Update::measured(
                LawForce::NONE,
                Some(decision(
                    AttachmentState::Engaging,
                    AttachmentCause::CureStarted,
                )),
            ));
        }
        let elapsed = self
            .cured_ns
            .checked_add(dt_ns)
            .ok_or_else(|| invalid("переполнение длительности отверждения"))?;
        if elapsed < self.cure_duration.unwrap().0 {
            return Ok(Update {
                cured_ns: elapsed,
                ..Update::measured(LawForce::NONE, None)
            });
        }
        let Some(residual) = input.weld_residual else {
            return Ok(Update::unavailable("поза опоры weld неизвестна"));
        };
        // Идеальная связь допускается только без изменения позы и относительной кинетической энергии.
        if residual.translation_m != 0.0
            || residual.rotation_rad != 0.0
            || g.speed != 0.0
            || g.spin != 0.0
        {
            return Ok(Update::refused(
                LawForce::NONE,
                "включение weld привело бы к скачку состояния",
            ));
        }
        let change = Decision {
            effect: LinkEffect::Weld(true),
            ..decision(AttachmentState::Attached, AttachmentCause::Captured)
        };
        Ok(Update::measured(LawForce::RIGID, Some(change)))
    }
}

impl StepModel for AttachmentModel {
    type Input = AttachmentInput;
    type Output = AttachmentStep;

    fn advance(
        &mut self,
        interval: Interval,
        input: &AttachmentInput,
        _: &mut Rng,
    ) -> Result<AttachmentStep, ProcessError> {
        check_interval(interval, self.previous)?;
        input.validate()?;
        if self.pending.is_some() {
            return Err(invalid("предыдущий переход ещё не подтверждён рантаймом"));
        }
        let at = interval.end;
        if input
            .command
            .as_ref()
            .is_some_and(|request| request.name != self.binding.name || request.at > at)
        {
            return Err(invalid(
                "запрос относится к другому креплению или будущему такту",
            ));
        }
        let g = Geometry::from_mounts(input.mounts)?;
        let mut step = AttachmentStep {
            at,
            state: self.state,
            failed: self.failed,
            reaction: unavailable("нет реакции стыка"),
            loads: Vec::new(),
            change: None,
            event: None,
            hazards: self.hazards(),
        };
        let Analysis::Analyzed {
            value: measured, ..
        } = input.measured_wrench
        else {
            step.reaction = unavailable("нет измеренной нагрузки стыка");
            step.event = self.unavailable_event(at, input, "нет измеренной нагрузки стыка");
            self.previous = Some(at);
            self.cured_ns = 0;
            return Ok(step);
        };
        let dt = interval.end.0 - interval.start.0;
        let update = self.evaluate(dt, input, g, measured)?;
        match update.evaluation {
            Analysis::NotAnalyzed { reason } => {
                step.event = self.unavailable_event(at, input, &reason);
                step.reaction = unavailable(reason);
            }
            Analysis::Analyzed {
                value: (force, transition),
                ..
            } => {
                let [a, b] = input.mounts;
                finite(
                    [force.axial, force.torque, force.loss_w]
                        .into_iter()
                        .chain(force.energy),
                )?;
                let applied = JointWrench {
                    force_n: scale(a.axis, force.axial),
                    torque_nm: scale(a.axis, force.torque),
                };
                let scope = "номинальный закон крепления в калиброванном диапазоне";
                step.reaction = analyzed(
                    AttachmentReaction {
                        gap_m: g.gap,
                        lateral_m: g.radial,
                        axis_error_rad: g.tilt,
                        measured,
                        applied,
                        dissipated_power_w: force.loss_w,
                        stored_energy_j: force.energy.map_or_else(
                            || unavailable("энергия деформации идеальной связи неизвестна"),
                            |energy| analyzed(energy, scope),
                        ),
                    },
                    scope,
                );
                if force.axial != 0.0 || force.torque != 0.0 {
                    // Общая точка Б для обеих сил учитывает реактивный момент вращения оси А.
                    step.loads = vec![
                        load(
                            &self.ports[0].body,
                            b.point_m,
                            a.com_m,
                            scale(applied.force_n, -1.0),
                            scale(applied.torque_nm, -1.0),
                        )?,
                        load(
                            &self.ports[1].body,
                            b.point_m,
                            b.com_m,
                            applied.force_n,
                            applied.torque_nm,
                        )?,
                    ];
                }
                if let Some(change) = transition {
                    let request_action = if matches!(
                        change.cause,
                        AttachmentCause::Captured | AttachmentCause::CureStarted
                    ) {
                        AttachmentAction::Attach
                    } else {
                        AttachmentAction::Release
                    };
                    let released_energy_j =
                        self.released_energy(g, input.thread, change.cause, force)?;
                    step.change = Some(AttachmentChange {
                        request: self.request(at, input, request_action),
                        from: self.state,
                        to: change.to,
                        effect: change.effect,
                        cause: change.cause,
                        released_energy_j,
                    });
                } else if let Some(reason) = update.refusal {
                    let action = if self.state == AttachmentState::Attached {
                        AttachmentAction::Release
                    } else {
                        AttachmentAction::Attach
                    };
                    let request = input
                        .command
                        .clone()
                        .unwrap_or_else(|| self.request(at, input, action));
                    step.event = Some(AttachmentEvent {
                        request,
                        evaluated_at: at,
                        outcome: AttachmentOutcome::Rejected {
                            reason: reason.into(),
                        },
                    });
                }
            }
        }
        self.pending = step.change.as_ref().map(|c| (at, c.clone()));
        self.previous = Some(at);
        self.cured_ns = update.cured_ns;
        Ok(step)
    }
}

impl AttachmentModel {
    fn request(
        &self,
        at: Time,
        input: &AttachmentInput,
        action: AttachmentAction,
    ) -> AttachmentRequest {
        input
            .command
            .as_ref()
            .filter(|request| request.action == action)
            .cloned()
            .unwrap_or_else(|| AttachmentRequest {
                at,
                name: self.binding.name.clone(),
                action,
            })
    }

    fn unavailable_event(
        &self,
        at: Time,
        input: &AttachmentInput,
        reason: &str,
    ) -> Option<AttachmentEvent> {
        input.command.as_ref().map(|request| AttachmentEvent {
            request: request.clone(),
            evaluated_at: at,
            outcome: AttachmentOutcome::NotAnalyzed {
                reason: reason.into(),
            },
        })
    }

    fn released_energy(
        &self,
        g: Geometry,
        thread: Option<ThreadMotion>,
        cause: AttachmentCause,
        next: LawForce,
    ) -> Result<Analysis<f64>, ProcessError> {
        let jump = match (&self.law, cause) {
            (AttachmentLaw::Latch(law), AttachmentCause::Captured | AttachmentCause::Released) => {
                let previous = self.latch_force(law, g, self.state == AttachmentState::Attached);
                previous.energy.unwrap() - next.energy.unwrap()
            }
            (AttachmentLaw::Threaded(law), AttachmentCause::Released) => {
                let angle = thread
                    .ok_or_else(|| invalid("нет фазы для работы отпускания резьбы"))?
                    .angle_rad;
                let error = g.gap - law.entry_m.nominal - law.lead_m_per_turn.nominal / TAU * angle;
                0.5 * law.stiffness_n_m.nominal * error.powi(2)
            }
            (_, AttachmentCause::Overload | AttachmentCause::Stripped) => {
                return Ok(unavailable("энергия разрушения не задана"));
            }
            // Смена статуса магнита не переключает поле. При захвате резьбы
            // и образовании клеевой связи предварительная упругая нагрузка равна нулю.
            _ => 0.0,
        };
        finite([jump])?;
        if jump < 0.0 {
            return Err(invalid("переход добавляет запасённую энергию"));
        }
        Ok(analyzed(jump, "работа на смене модели при неизменной позе"))
    }
}
