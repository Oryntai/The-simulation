//! Крепления в ходе прогона и классы деформируемых предметов.

use sim_signal::Time;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AttachmentKind {
    Magnetic,
    Latch,
    Threaded,
    Adhesive,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AttachmentBinding {
    pub name: String,
    pub mounts: [String; 2],
    pub kind: AttachmentKind,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AttachmentAction {
    Attach,
    Release,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AttachmentState {
    Free,
    Engaging,
    Attached,
    Releasing,
}

/// Запрос модели на границе шага. Состояние связи меняется по итогам
/// физической проверки и применения эффекта рантаймом.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AttachmentRequest {
    pub at: Time,
    pub name: String,
    pub action: AttachmentAction,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AttachmentOutcome {
    Transition {
        from: AttachmentState,
        to: AttachmentState,
    },
    Rejected {
        reason: String,
    },
    NotAnalyzed {
        reason: String,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AttachmentEvent {
    pub request: AttachmentRequest,
    pub evaluated_at: Time,
    pub outcome: AttachmentOutcome,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SoftObjectKind {
    Cable,
    Rubber,
    Foam,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SoftObjectMeasure {
    pub name: String,
    pub deformation_m: crate::Analysis<f64>,
    pub reaction_n: crate::Analysis<f64>,
    pub stored_energy_j: crate::Analysis<f64>,
}

mod curve;
mod joint;
mod math;
mod soft;

pub use curve::{CurvePoint, ForceCurve};
pub use joint::*;
pub use soft::*;

mod compliant;
mod tension;

pub use compliant::{
    CompliantPad, PadCalibration, PadInput, PadLaw, PadReactionOwner, PadSpec, PatchAreaPoint,
    StressBounds,
};
pub use tension::{
    DriveMotion, FreeLengthLaw, RoutePoint, TensionCalibration, TensionInput, TensionRoute,
    TensionSpec, TractionFriction,
};

use sim_signal::ChannelId;

use crate::{Analysis, BodyLoad, Interval, ProcessError, SourcedRange};

use self::math::{analyzed, check_interval, finite, invalid, unavailable};

#[derive(Clone, Debug, PartialEq)]
#[allow(
    clippy::large_enum_variant,
    reason = "вход передаётся по ссылке без аллокации каждый такт"
)]
pub enum InterfaceInput {
    Pad(PadInput),
    Tension(TensionInput),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InterfaceState {
    Unobserved,
    /// Характеристика вышла за область расчёта. Историю связи больше
    /// нельзя продолжить без восстановления из последнего валидного снимка.
    Unavailable,
    Open,
    Loaded,
    Sliding,
    Released,
    Broken,
}

impl InterfaceState {
    pub fn is_terminal(self) -> bool {
        matches!(self, Self::Released | Self::Broken)
    }
}

/// Уже подтверждённое рантаймом событие на левой границе. Положительный
/// ход проскальзывания удлиняет свободный участок натяжной связи.
#[derive(Clone, Debug, PartialEq)]
pub struct ConfirmedInterfaceEvent {
    pub name: String,
    pub at: Time,
    pub kind: ConfirmedEvent,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ConfirmedEvent {
    Release,
    Break,
    Slip { payout_m: f64 },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Transition {
    ConfirmedRelease,
    ConfirmedBreak,
    ConfirmedSlip {
        payout_m: f64,
    },
    /// Сработала нижняя граница прочности крепления по результирующей силе.
    /// Это событие модели, применимость предела проверяется отдельно.
    Overload {
        force_n: f64,
        limit_n: f64,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct InterfaceReaction {
    /// Реакции на телах, включая измеренные движком, для журнала.
    pub loads: Vec<BodyLoad>,
    pub normal_n: f64,
    /// Результирующий сдвиг в мировой системе на стороне Б.
    pub shear_n: [f64; 3],
    pub shear_upper_n: f64,
    pub peak_shear_n: f64,
    pub tension_n: f64,
    pub contact_count: usize,
}

impl InterfaceReaction {
    fn zero() -> Self {
        Self {
            loads: Vec::new(),
            normal_n: 0.0,
            shear_n: [0.0; 3],
            shear_upper_n: 0.0,
            peak_shear_n: 0.0,
            tension_n: 0.0,
            contact_count: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct InterfaceWork {
    /// Запас упругой энергии текущего закона. Включает начальный преднатяг.
    pub stored_energy_j: f64,
    /// Оценка работы сил по скоростям в начале удержания.
    pub body_work_j: f64,
    pub drive_work_j: f64,
    pub loss_j: f64,
    pub numerical_loss_j: f64,
    /// Энергия, вышедшая из модели при мгновенном событии. Её дальнейшее
    /// распределение неизвестно и требует отдельного учёта.
    pub released_energy_j: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct InterfaceStep {
    pub interval: Interval,
    pub state: InterfaceState,
    pub transition: Option<Transition>,
    /// Только нагрузки, которые требуется применить. Измеренные силы
    /// движка остаются в reaction и повторно в физику не подаются.
    pub body_loads: Vec<BodyLoad>,
    pub drive_effort: Option<(ChannelId, f64)>,
    /// Если закон нагрузки недоступен, пустой список нагрузок означает
    /// потерю расчёта, и рантайм не может продолжать связанный прогон.
    pub reaction: Analysis<InterfaceReaction>,
    pub stress: Analysis<StressBounds>,
    /// Площадь из характеристики пятна при сжатии в этом интервале.
    pub effective_area_m2: Option<SourcedRange>,
    pub work: Analysis<InterfaceWork>,
    /// Проверка окна текущей модели. Для движка применимость его закона
    /// и наличие требуемой пары подтверждаются отдельно по собранной сцене.
    pub calibration: Analysis<()>,
    pub contact_frame_at: Option<Time>,
    /// Сжатие площадки или подписанное растяжение трассы.
    pub deformation_m: Option<f64>,
    pub offset_m: Option<f64>,
    pub axis_error_rad: Option<f64>,
    pub path_length_m: Option<f64>,
    /// Силы в порядке точек трассы до суммирования нагрузок одного тела.
    pub route_forces_n: Vec<[f64; 3]>,
    /// Свободная длина с уже учтённым смещением крепления.
    pub free_length_m: Option<f64>,
    /// Необратимое смещение за этот шаг.
    pub slip_m: Option<f64>,
    pub payout_m: Option<f64>,
}

impl InterfaceStep {
    fn new(interval: Interval, state: InterfaceState) -> Self {
        Self {
            interval,
            state,
            transition: None,
            body_loads: Vec::new(),
            drive_effort: None,
            reaction: unavailable("нет подтверждённой реакции"),
            stress: unavailable("нет применимого расчёта среднего напряжения"),
            effective_area_m2: None,
            work: unavailable("нет модели работы для источника реакции"),
            calibration: unavailable("нет подтверждённой области применимости"),
            contact_frame_at: None,
            deformation_m: None,
            offset_m: None,
            axis_error_rad: None,
            path_length_m: None,
            route_forces_n: Vec::new(),
            free_length_m: None,
            slip_m: None,
            payout_m: None,
        }
    }

    fn unavailable(&mut self, reason: &str) {
        self.reaction = unavailable(reason);
        self.calibration = unavailable(reason);
        self.stress = unavailable(reason);
        self.work = unavailable(reason);
    }

    fn mark_inactive(&mut self) {
        self.reaction = analyzed(InterfaceReaction::zero(), "связь разомкнута");
        self.calibration = unavailable("передача нагрузки через разомкнутую связь не оценивается");
    }
}

#[derive(Clone, Copy, Debug)]
struct Status {
    last: Option<Time>,
    state: InterfaceState,
}

impl Default for Status {
    fn default() -> Self {
        Self {
            last: None,
            state: InterfaceState::Unobserved,
        }
    }
}

impl Status {
    fn begin(
        self,
        interval: Interval,
        id: &str,
        event: Option<&ConfirmedInterfaceEvent>,
    ) -> Result<(f64, Option<Transition>), ProcessError> {
        check_interval(interval, self.last)?;
        let transition = match event {
            Some(e) => {
                if e.name != id || e.at != interval.start || self.state.is_terminal() {
                    return Err(invalid("событие не соответствует шагу или активной связи"));
                }
                Some(match e.kind {
                    ConfirmedEvent::Release => Transition::ConfirmedRelease,
                    ConfirmedEvent::Break => Transition::ConfirmedBreak,
                    ConfirmedEvent::Slip { payout_m } => {
                        finite([payout_m])?;
                        if payout_m <= 0.0 {
                            return Err(invalid(
                                "подтверждённое проскальзывание должно удлинять связь",
                            ));
                        }
                        Transition::ConfirmedSlip { payout_m }
                    }
                })
            }
            None => None,
        };
        Ok((
            Time(interval.end.0 - interval.start.0).as_secs(),
            transition,
        ))
    }

    fn with_transition(
        self,
        transition: Option<Transition>,
        otherwise: InterfaceState,
    ) -> InterfaceState {
        if self.state.is_terminal() {
            return self.state;
        }
        match transition {
            Some(Transition::ConfirmedRelease) => InterfaceState::Released,
            Some(Transition::ConfirmedBreak | Transition::Overload { .. }) => {
                InterfaceState::Broken
            }
            Some(Transition::ConfirmedSlip { .. }) => InterfaceState::Sliding,
            None => otherwise,
        }
    }

    fn commit(&mut self, step: &InterfaceStep) {
        self.last = Some(step.interval.end);
        self.state = step.state;
    }
}

fn name(value: &str) -> Result<(), ProcessError> {
    if value.trim().is_empty() || value.chars().any(char::is_control) {
        return Err(invalid("пустое имя или управляющий символ в имени связи"));
    }
    Ok(())
}

fn range(value: &SourcedRange, min: f64, max: f64) -> Result<(), ProcessError> {
    value.validate()?;
    math::range(value, min, max)
}

fn positive(value: &SourcedRange) -> Result<(), ProcessError> {
    value.validate()?;
    math::positive(value)
}

fn contains(value: &SourcedRange, x: f64) -> bool {
    (value.min..=value.max).contains(&x)
}

fn scalar_supported(value: &SourcedRange) -> bool {
    value.min == value.max && value.can_prove()
}

fn validate_curve(curve: &ForceCurve) -> Result<(), ProcessError> {
    curve.compression()?;
    for p in curve.points() {
        p.displacement_m.validate()?;
        p.force_n.validate()?;
    }
    Ok(())
}

fn curve_supported(curve: &ForceCurve) -> bool {
    curve
        .points()
        .iter()
        .all(|p| scalar_supported(&p.displacement_m) && scalar_supported(&p.force_n))
}

struct ElasticReaction {
    elastic_n: f64,
    force_n: f64,
    energy_j: f64,
    loss_power_w: f64,
}

fn elastic(
    curve: &ForceCurve,
    damping: f64,
    x: f64,
    speed: f64,
) -> Result<Option<ElasticReaction>, ProcessError> {
    finite([x, speed])?;
    if x <= 0.0 {
        return Ok(Some(ElasticReaction {
            elastic_n: 0.0,
            force_n: 0.0,
            energy_j: 0.0,
            loss_power_w: 0.0,
        }));
    }
    let Some((elastic_n, energy_j)) = curve.sample(x)? else {
        return Ok(None);
    };
    let raw = elastic_n + damping * speed;
    finite([raw])?;
    let force_n = raw.max(0.0);
    // При быстром разгружении ограничитель силы также пассивен:
    // (F - F_упр) ẋ ≥ 0, в том числе при F = 0 и ẋ < 0.
    let loss_power_w = (force_n - elastic_n) * speed;
    finite([loss_power_w])?;
    Ok(Some(ElasticReaction {
        elastic_n,
        force_n,
        energy_j,
        loss_power_w,
    }))
}

fn work_is_finite(work: InterfaceWork) -> Result<(), ProcessError> {
    finite([
        work.stored_energy_j,
        work.body_work_j,
        work.drive_work_j,
        work.loss_j,
        work.numerical_loss_j,
        work.released_energy_j,
    ])
}
