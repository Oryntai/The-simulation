//! Окно процесса для заданного интервала контакта и калиброванной геометрии.

use std::num::NonZeroU32;

use sim_signal::{Rng, Time};

use crate::{Analysis, Interval, ProcessError, SourcedRange, StepModel};

use super::heat::solder_network::{
    SolderContact, SolderNetwork, SolderNetworkInput, SolderNetworkSpec,
};
use super::math::{calculated, computed, finite, invalid, unknown};

const SCOPE: &str = "окно процесса для заданного контакта, тепловой модели и геометрии";

/// Верхний предел дозы применим на всём подтверждённом диапазоне шагов.
/// Калибровка относится к той же геометрии площадок, сплаву и процессу.
#[derive(Clone, Debug, PartialEq)]
pub struct SolderBridgeLimit {
    pub pitch_m: SourcedRange,
    pub max_volume_m3: SourcedRange,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct SolderWindowLimits {
    pub wetting_k: Option<SourcedRange>,
    pub max_time_to_wet_s: Option<SourcedRange>,
    pub liquidus_k: Option<SourcedRange>,
    pub min_above_liquidus_s: Option<SourcedRange>,
    pub max_above_liquidus_s: Option<SourcedRange>,
    pub max_pad_k: Option<SourcedRange>,
    /// Предел относится к температуре вывода в четырёхузловой модели.
    pub max_component_k: Option<SourcedRange>,
    pub min_volume_m3: Option<SourcedRange>,
    pub bridge: Option<SolderBridgeLimit>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SolderWindowInput {
    pub network: SolderNetworkSpec,
    /// Постоянные тепловая мощность и контакт за всё окно.
    pub thermal: SolderNetworkInput,
    /// Усилие, при котором подтверждена заданная проводимость контакта.
    pub force_n: Option<SourcedRange>,
    pub duration_s: Option<SourcedRange>,
    pub pitch_m: Option<SourcedRange>,
    pub dose_m3: Option<SourcedRange>,
    /// Источники пределов должны относиться к данной постановке.
    pub limits: SolderWindowLimits,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SolderBounds {
    pub lower: f64,
    pub upper: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SolderWettingTime {
    /// Первое достижение площадкой заданного маркера смачивания.
    Reached(SolderBounds),
    NotReached,
    Unresolved {
        earliest_s: f64,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct SolderThermalMetrics {
    pub time_to_wet_s: Analysis<SolderWettingTime>,
    /// Суммарное время строго выше ликвидуса на интервале расчёта.
    pub above_liquidus_s: Analysis<SolderBounds>,
    pub peak_pad_k: SolderBounds,
    pub peak_component_k: SolderBounds,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SolderCriterion {
    MaxTimeToWet,
    MinAboveLiquidus,
    MaxAboveLiquidus,
    MaxPadTemperature,
    MaxComponentTemperature,
    Bridge,
    MinVolume,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SolderWindowStatus {
    /// Подтверждено выполнение критерия или окна процесса в его области.
    Pass,
    Fail,
    NotAnalyzed {
        reason: String,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct SolderWindowCheck {
    pub criterion: SolderCriterion,
    pub status: SolderWindowStatus,
    /// Запас положителен на допустимой стороне. Единицы задаёт критерий.
    pub margin: Option<SolderBounds>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SolderWindowReport {
    pub thermal: Analysis<SolderThermalMetrics>,
    checks: [SolderWindowCheck; 7],
}

impl SolderWindowReport {
    pub fn status(&self) -> SolderWindowStatus {
        if self
            .checks
            .iter()
            .any(|check| check.status == SolderWindowStatus::Fail)
        {
            return SolderWindowStatus::Fail;
        }
        self.checks
            .iter()
            .find(|check| matches!(check.status, SolderWindowStatus::NotAnalyzed { .. }))
            .map_or(SolderWindowStatus::Pass, |check| check.status.clone())
    }

    pub fn checks(&self) -> &[SolderWindowCheck] {
        &self.checks
    }

    pub fn check(&self, criterion: SolderCriterion) -> &SolderWindowCheck {
        self.checks
            .iter()
            .find(|check| check.criterion == criterion)
            .unwrap()
    }
}

/// Считает проверяемое окно на равномерной сетке целых наносекунд. Огибающая
/// включает всё время между узлами сетки. Если знак запаса не определён,
/// критерий остаётся открытым. Плотность сетки задаёт вызывающий код.
pub fn assess_solder_window(
    input: &SolderWindowInput,
    steps: NonZeroU32,
) -> Result<SolderWindowReport, ProcessError> {
    use SolderCriterion::*;

    let limits = &input.limits;
    let get = |value, label, positive| confirmed(value, label, positive);
    let wetting = get(
        limits.wetting_k.as_ref(),
        "температура маркера смачивания",
        true,
    )?;
    let deadline = get(limits.max_time_to_wet_s.as_ref(), "срок смачивания", false)?;
    let liquidus = get(limits.liquidus_k.as_ref(), "ликвидус сплава", true)?;
    let min_time = get(
        limits.min_above_liquidus_s.as_ref(),
        "минимум времени над ликвидусом",
        false,
    )?;
    let max_time = get(
        limits.max_above_liquidus_s.as_ref(),
        "максимум времени над ликвидусом",
        false,
    )?;
    let max_pad = get(
        limits.max_pad_k.as_ref(),
        "предел температуры площадки",
        true,
    )?;
    let max_component = get(
        limits.max_component_k.as_ref(),
        "предел температуры вывода",
        true,
    )?;
    let min_volume = get(
        limits.min_volume_m3.as_ref(),
        "минимальный объём припоя",
        false,
    )?;
    let dose = get(input.dose_m3.as_ref(), "доза припоя", false)?;
    let pitch = get(input.pitch_m.as_ref(), "шаг площадок", true)?;
    let calibrated_pitch = get(
        limits.bridge.as_ref().map(|limit| &limit.pitch_m),
        "шаг калибровки перемычки",
        true,
    )?;
    let bridge = get(
        limits.bridge.as_ref().map(|limit| &limit.max_volume_m3),
        "предел дозы для перемычки",
        false,
    )?;
    let bridge = match (calibrated_pitch, pitch, bridge) {
        (Ok(cal), Ok(pitch), Ok(limit)) if cal.min <= pitch.min && pitch.max <= cal.max => {
            Ok(limit)
        }
        (Ok(_), Ok(_), Ok(_)) => {
            Err("шаг площадок вне диапазона калибровки риска перемычки".into())
        }
        (Err(reason), _, _) | (_, Err(reason), _) | (_, _, Err(reason)) => Err(reason),
    };
    ordered_limits(&min_time, &max_time, "время над ликвидусом")?;
    ordered_limits(&min_volume, &bridge, "объём припоя")?;

    let (thermal, cold_end) = match prepare_thermal(input)? {
        Ok((law, initial, duration)) => {
            simulate(input, law, initial, duration, steps, wetting, liquidus)?
        }
        Err(reason) => (unknown(&reason), false),
    };
    let (wetting_time, above, pad, component) = match &thermal {
        Analysis::Analyzed { value, .. } => (
            analysis_value(&value.time_to_wet_s),
            analysis_value(&value.above_liquidus_s),
            Ok(value.peak_pad_k),
            Ok(value.peak_component_k),
        ),
        Analysis::NotAnalyzed { reason } => (
            Err(reason.clone()),
            Err(reason.clone()),
            Err(reason.clone()),
            Err(reason.clone()),
        ),
    };
    let minimum = LimitSide::Minimum;
    let maximum = LimitSide::Maximum;
    let mut checks = [
        check_wetting(wetting_time, deadline)?,
        minimum.check(MinAboveLiquidus, above.clone(), min_time)?,
        maximum.check(MaxAboveLiquidus, above, max_time)?,
        maximum.check(MaxPadTemperature, pad, max_pad)?,
        maximum.check(MaxComponentTemperature, component, max_component)?,
        maximum.check(Bridge, dose.clone().map(SolderBounds::from), bridge)?,
        minimum.check(MinVolume, dose.map(SolderBounds::from), min_volume)?,
    ];
    for check in &mut checks {
        if matches!(check.criterion, MaxAboveLiquidus | MaxComponentTemperature)
            && check.status == SolderWindowStatus::Pass
            && !cold_end
        {
            *check = SolderWindowCheck::open(
                check.criterion,
                "нет фазы остывания: не подтверждено, что площадка и вывод холоднее ликвидуса при снятии жала".into(),
                None,
            );
        }
    }
    Ok(SolderWindowReport { thermal, checks })
}

type Evidence<T> = Result<T, String>;

fn confirmed<'a>(
    value: Option<&'a SourcedRange>,
    label: &str,
    positive: bool,
) -> Result<Evidence<&'a SourcedRange>, ProcessError> {
    let Some(value) = value else {
        return Ok(Err(format!("{label}: нет значения")));
    };
    finite(&[value.min, value.nominal, value.max])?;
    if value.min < 0.0
        || (positive && value.min == 0.0)
        || value.min > value.nominal
        || value.nominal > value.max
    {
        return Err(invalid(format!("некорректный диапазон: {label}")));
    }
    Ok(if value.can_prove() {
        Ok(value)
    } else {
        Err(format!("{label}: нет подтверждённого источника"))
    })
}

fn ordered_limits(
    minimum: &Evidence<&SourcedRange>,
    maximum: &Evidence<&SourcedRange>,
    label: &str,
) -> Result<(), ProcessError> {
    if let (Ok(minimum), Ok(maximum)) = (minimum, maximum)
        && minimum.min > maximum.max
    {
        return Err(invalid(format!("{label}: нижний предел выше верхнего")));
    }
    Ok(())
}

fn analysis_value<T: Copy>(analysis: &Analysis<T>) -> Evidence<T> {
    match analysis {
        Analysis::Analyzed { value, .. } => Ok(*value),
        Analysis::NotAnalyzed { reason } => Err(reason.clone()),
    }
}

impl SolderWindowCheck {
    fn open(criterion: SolderCriterion, reason: String, margin: Option<SolderBounds>) -> Self {
        Self {
            criterion,
            status: SolderWindowStatus::NotAnalyzed { reason },
            margin,
        }
    }
}

#[derive(Clone, Copy)]
enum LimitSide {
    Minimum,
    Maximum,
}

impl LimitSide {
    fn check(
        self,
        criterion: SolderCriterion,
        actual: Evidence<SolderBounds>,
        limit: Evidence<&SourcedRange>,
    ) -> Result<SolderWindowCheck, ProcessError> {
        let (actual, limit) = match (actual, limit) {
            (Ok(actual), Ok(limit)) => (actual, SolderBounds::from(limit)),
            (Err(reason), _) | (_, Err(reason)) => {
                return Ok(SolderWindowCheck::open(criterion, reason, None));
            }
        };
        let (high, low) = match self {
            Self::Minimum => (actual, limit),
            Self::Maximum => (limit, actual),
        };
        let margin = high.sub(low).checked()?;
        // Сравнение исходных границ сохраняет включённый порог при точном равенстве.
        let status = if high.lower >= low.upper {
            SolderWindowStatus::Pass
        } else if high.upper < low.lower {
            SolderWindowStatus::Fail
        } else {
            SolderWindowStatus::NotAnalyzed {
                reason: "огибающая пересекает предел".into(),
            }
        };
        Ok(SolderWindowCheck {
            criterion,
            status,
            margin: Some(margin),
        })
    }
}

fn check_wetting(
    time: Evidence<SolderWettingTime>,
    deadline: Evidence<&SourcedRange>,
) -> Result<SolderWindowCheck, ProcessError> {
    let criterion = SolderCriterion::MaxTimeToWet;
    match (time, deadline) {
        (Ok(SolderWettingTime::Reached(time)), deadline) => {
            LimitSide::Maximum.check(criterion, Ok(time), deadline)
        }
        (Ok(SolderWettingTime::NotReached), Ok(_)) => Ok(SolderWindowCheck {
            criterion,
            status: SolderWindowStatus::Fail,
            margin: None,
        }),
        (Ok(SolderWettingTime::Unresolved { earliest_s }), Ok(deadline))
            if earliest_s > deadline.max =>
        {
            Ok(SolderWindowCheck {
                criterion,
                status: SolderWindowStatus::Fail,
                margin: None,
            })
        }
        (Ok(SolderWettingTime::Unresolved { .. }), Ok(_)) => Ok(SolderWindowCheck::open(
            criterion,
            "не установлено достижение маркера смачивания".into(),
            None,
        )),
        (Err(reason), _) | (_, Err(reason)) => Ok(SolderWindowCheck::open(criterion, reason, None)),
    }
}

/// Постоянная тепловая задача. Порядок узлов совпадает с SolderNetwork.
struct ThermalLaw {
    capacities: [f64; 4],
    conductances: [f64; 4],
    polygon_k: f64,
    power_w: f64,
}

fn prepare_thermal(
    input: &SolderWindowInput,
) -> Result<Evidence<(ThermalLaw, [f64; 4], Time)>, ProcessError> {
    let spec = &input.network;
    let mut unresolved = None;
    let mut point = |value: &Option<SourcedRange>,
                     label: &str,
                     positive|
     -> Result<Option<f64>, ProcessError> {
        let result = match confirmed(value.as_ref(), label, positive)? {
            Ok(range) if range.min == range.max => return Ok(Some(range.nominal)),
            Ok(_) => format!("{label}: тепловая огибающая диапазона не рассчитана"),
            Err(reason) => reason,
        };
        unresolved.get_or_insert(result);
        Ok(None)
    };
    let capacities = [
        point(&spec.capacity_j_k.tip, "теплоёмкость жала", true)?,
        point(&spec.capacity_j_k.pad, "теплоёмкость площадки", true)?,
        point(&spec.capacity_j_k.lead, "теплоёмкость вывода", true)?,
        point(&spec.capacity_j_k.copper, "теплоёмкость меди", true)?,
    ];
    let initial = [
        point(&spec.initial_k.tip, "начальная температура жала", true)?,
        point(&spec.initial_k.pad, "начальная температура площадки", true)?,
        point(&spec.initial_k.lead, "начальная температура вывода", true)?,
        point(&spec.initial_k.copper, "начальная температура меди", true)?,
    ];
    let contact = match &input.thermal.contact {
        Some(SolderContact::Touching { conductance_w_k }) => {
            point(conductance_w_k, "проводимость контакта жала", true)?
        }
        _ => point(&None, "подтверждённый тепловой контакт жала", true)?,
    };
    let conductances = [
        contact,
        point(&spec.pad_lead_w_k, "проводимость площадка–вывод", false)?,
        point(&spec.pad_copper_w_k, "проводимость площадка–медь", false)?,
        point(&spec.copper_polygon_w_k, "проводимость медь–полигон", false)?,
    ];
    let polygon = point(&spec.polygon_k, "температура полигона", true)?;
    let power_limit = point(&spec.max_power_w, "предел мощности жала", false)?;
    let request = point(
        &input.thermal.requested_power_w,
        "запрос мощности жала",
        false,
    )?;
    point(&input.force_n, "усилие калибровки контакта", true)?;
    let duration = point(&input.duration_s, "длительность контакта", true)?;
    if let Some(reason) = unresolved {
        return Ok(Err(reason));
    }
    let seconds = duration.unwrap();
    let Some(duration) = Time::from_secs(seconds).filter(|t| t.0 > 0 && t.as_secs() == seconds)
    else {
        return Ok(Err(
            "длительность контакта не представима на наносекундной сетке".into(),
        ));
    };
    Ok(Ok((
        ThermalLaw {
            capacities: capacities.map(Option::unwrap),
            conductances: conductances.map(Option::unwrap),
            polygon_k: polygon.unwrap(),
            power_w: request.unwrap().min(power_limit.unwrap()),
        },
        initial.map(Option::unwrap),
        duration,
    )))
}

impl ThermalLaw {
    fn rate(&self, [tip, pad, lead, copper]: [SolderBounds; 4]) -> [SolderBounds; 4] {
        let [tip_pad, pad_lead, pad_copper, copper_polygon] =
            self.conductances.map(SolderBounds::point);
        let tip_pad = tip_pad.mul(tip.sub(pad));
        let pad_lead = pad_lead.mul(pad.sub(lead));
        let pad_copper = pad_copper.mul(pad.sub(copper));
        let copper_polygon = copper_polygon.mul(copper.sub(SolderBounds::point(self.polygon_k)));
        let flows = [
            SolderBounds::point(self.power_w).sub(tip_pad),
            tip_pad.sub(pad_lead).sub(pad_copper),
            pad_lead,
            pad_copper.sub(copper_polygon),
        ];
        std::array::from_fn(|i| flows[i].div_positive(self.capacities[i]))
    }
}

struct StepEnvelope {
    pad: SolderBounds,
    pad_end: SolderBounds,
    component: SolderBounds,
    component_end: SolderBounds,
}

/// Кубический полином Эрмита через расчётные концы шага. Коэффициенты Бернштейна
/// ограничивают значение полинома и его невязку на всём шаге. Из принципа
/// максимума для пассивной тепловой сети ошибка в норме максимума растёт
/// не быстрее интеграла абсолютной невязки, даже если концы шага неточны.
fn enclose_step(
    law: &ThermalLaw,
    start: [f64; 4],
    end: [f64; 4],
    dt: SolderBounds,
    error_k: &mut f64,
) -> Result<StepEnvelope, ProcessError> {
    let a = start.map(SolderBounds::point);
    let b = end.map(SolderBounds::point);
    let da = law.rate(a);
    let db = law.rate(b);
    // c = [a, a + dt·a'/3, b − dt·b'/3, b]. Разности для производной
    // считаются через смещения, чтобы сохранять точность на малых шагах.
    let l = da.map(|slope| dt.mul(slope).div_positive(3.0));
    let r = db.map(|slope| dt.mul(slope).div_positive(3.0));
    let c = [
        a,
        std::array::from_fn(|i| a[i].add(l[i])),
        std::array::from_fn(|i| b[i].sub(r[i])),
        b,
    ];
    let middle: [_; 4] = std::array::from_fn(|i| b[i].sub(a[i]).sub(l[i]).sub(r[i]));
    let two = SolderBounds::point(2.0);
    let derivatives = [
        da.map(|slope| dt.mul(slope)),
        std::array::from_fn(|i| l[i].add(two.mul(middle[i]))),
        std::array::from_fn(|i| two.mul(middle[i]).add(r[i])),
        db.map(|slope| dt.mul(slope)),
    ];
    let mut residual: f64 = 0.0;
    for (control, slope) in c.into_iter().zip(derivatives) {
        for (actual, fitted) in law.rate(control).into_iter().zip(slope) {
            let difference = dt.mul(actual).sub(fitted).checked()?;
            residual = residual.max(difference.lower.abs().max(difference.upper.abs()));
        }
    }
    *error_k = (*error_k + residual).next_up();
    computed(&[*error_k])?;
    let error = SolderBounds {
        lower: -*error_k,
        upper: *error_k,
    };
    let bounds = |index: usize| {
        c.iter()
            .map(|row| row[index])
            .reduce(SolderBounds::hull)
            .unwrap()
            .add(error)
            .checked()
    };
    // Индексы площадки и вывода в порядке узлов тепловой сети.
    Ok(StepEnvelope {
        pad: bounds(1)?,
        pad_end: b[1].add(error).checked()?,
        component: bounds(2)?,
        component_end: b[2].add(error).checked()?,
    })
}

struct ThermalRun {
    peak_pad: SolderBounds,
    peak_component: SolderBounds,
    possible_wet_at: Option<u64>,
    definite_wet_by: Option<u64>,
    above_liquidus_ns: [u64; 2],
}

impl ThermalRun {
    fn new(initial: [f64; 4], wetting: Option<&SourcedRange>) -> Self {
        Self {
            peak_pad: SolderBounds::point(initial[1]),
            peak_component: SolderBounds::point(initial[2]),
            possible_wet_at: wetting.filter(|w| initial[1] >= w.min).map(|_| 0),
            definite_wet_by: wetting.filter(|w| initial[1] >= w.max).map(|_| 0),
            above_liquidus_ns: [0; 2],
        }
    }

    fn observe(
        &mut self,
        interval: Interval,
        step: StepEnvelope,
        wetting: Option<&SourcedRange>,
        liquidus: Option<&SourcedRange>,
    ) {
        self.peak_pad.lower = self.peak_pad.lower.max(step.pad_end.lower);
        self.peak_pad.upper = self.peak_pad.upper.max(step.pad.upper);
        self.peak_component.lower = self.peak_component.lower.max(step.component_end.lower);
        self.peak_component.upper = self.peak_component.upper.max(step.component.upper);
        if let Some(threshold) = wetting {
            if step.pad.upper >= threshold.min {
                self.possible_wet_at.get_or_insert(interval.start.0);
            }
            if step.pad_end.lower >= threshold.max {
                self.definite_wet_by.get_or_insert(interval.end.0);
            }
        }
        if let Some(threshold) = liquidus {
            let ns = interval.end.0 - interval.start.0;
            // Сумма непересекающихся шагов не превосходит горизонта расчёта.
            if step.pad.lower > threshold.max {
                self.above_liquidus_ns[0] += ns;
            }
            if step.pad.upper > threshold.min {
                self.above_liquidus_ns[1] += ns;
            }
        }
    }

    fn metrics(
        self,
        wetting: Evidence<&SourcedRange>,
        liquidus: Evidence<&SourcedRange>,
    ) -> SolderThermalMetrics {
        let time_to_wet_s = match wetting {
            Ok(_) => {
                let time = match (self.possible_wet_at, self.definite_wet_by) {
                    (Some(first), Some(last)) => {
                        SolderWettingTime::Reached(time_span([first, last]))
                    }
                    (Some(first), None) => SolderWettingTime::Unresolved {
                        earliest_s: seconds(first).lower,
                    },
                    (None, _) => SolderWettingTime::NotReached,
                };
                calculated(time, SCOPE)
            }
            Err(reason) => unknown(&reason),
        };
        let above_liquidus_s = match liquidus {
            Ok(_) => calculated(time_span(self.above_liquidus_ns), SCOPE),
            Err(reason) => unknown(&reason),
        };
        SolderThermalMetrics {
            time_to_wet_s,
            above_liquidus_s,
            peak_pad_k: self.peak_pad,
            peak_component_k: self.peak_component,
        }
    }
}

fn simulate(
    input: &SolderWindowInput,
    law: ThermalLaw,
    initial: [f64; 4],
    duration: Time,
    steps: NonZeroU32,
    wetting: Evidence<&SourcedRange>,
    liquidus: Evidence<&SourcedRange>,
) -> Result<(Analysis<SolderThermalMetrics>, bool), ProcessError> {
    let mut network = SolderNetwork::new(input.network.clone())?;
    let mut previous = initial;
    let mut error_k = 0.0;
    let mut start = Time::ZERO;
    let n = duration.0.min(u64::from(steps.get()));
    let mut run = ThermalRun::new(initial, wetting.as_ref().ok().copied());
    let mut end_k = [initial[1], initial[2]].map(SolderBounds::point);
    // Тепловая сеть детерминирована, генератор требуется контрактом StepModel.
    let mut rng = Rng::stream(0, "тепловое окно пайки");
    for k in 1..=n {
        let end = Time((u128::from(duration.0) * u128::from(k) / u128::from(n)) as u64);
        let interval = Interval { start, end };
        let result = network.advance(interval, &input.thermal, &mut rng)?;
        let value = match result {
            Analysis::Analyzed { value, .. } => value,
            Analysis::NotAnalyzed { reason } => return Ok((unknown(&reason), false)),
        };
        let node = value.end_k;
        let next = [node.tip, node.pad, node.lead, node.copper];
        let envelope = enclose_step(&law, previous, next, seconds(end.0 - start.0), &mut error_k)?;
        end_k = [envelope.pad_end, envelope.component_end];
        run.observe(
            interval,
            envelope,
            wetting.as_ref().ok().copied(),
            liquidus.as_ref().ok().copied(),
        );
        previous = next;
        start = end;
    }
    let cold_end = liquidus.as_ref().is_ok_and(|threshold| {
        end_k.iter().all(|bounds| bounds.upper < threshold.min)
    });
    Ok((calculated(run.metrics(wetting, liquidus), SCOPE), cold_end))
}

fn time_span([start, end]: [u64; 2]) -> SolderBounds {
    SolderBounds {
        lower: seconds(start).lower,
        upper: seconds(end).upper,
    }
}

/// Округление наружу при переводе целой шкалы sim-signal в секунды.
fn seconds(ns: u64) -> SolderBounds {
    if ns == 0 {
        return SolderBounds::point(0.0);
    }
    let n = ns as f64;
    let bounds = match (n as u128).cmp(&u128::from(ns)) {
        std::cmp::Ordering::Less => SolderBounds {
            lower: n,
            upper: n.next_up(),
        },
        std::cmp::Ordering::Greater => SolderBounds {
            lower: n.next_down(),
            upper: n,
        },
        std::cmp::Ordering::Equal => SolderBounds::point(n),
    };
    bounds.div_positive(1e9)
}

impl From<&SourcedRange> for SolderBounds {
    fn from(value: &SourcedRange) -> Self {
        Self {
            lower: value.min,
            upper: value.max,
        }
    }
}

/// Каждая операция расширяет интервал на один соседний float при округлении.
impl SolderBounds {
    fn point(x: f64) -> Self {
        Self { lower: x, upper: x }
    }

    fn checked(self) -> Result<Self, ProcessError> {
        computed(&[self.lower, self.upper])?;
        Ok(self)
    }

    fn add(self, other: Self) -> Self {
        Self {
            lower: (self.lower + other.lower).next_down(),
            upper: (self.upper + other.upper).next_up(),
        }
    }

    fn sub(self, other: Self) -> Self {
        Self {
            lower: (self.lower - other.upper).next_down(),
            upper: (self.upper - other.lower).next_up(),
        }
    }

    fn mul(self, other: Self) -> Self {
        // В модели умножитель слева всегда положителен или нулевой.
        // Интервал справа может пересекать ноль.
        let lower_factor = if other.lower < 0.0 {
            self.upper
        } else {
            self.lower
        };
        let upper_factor = if other.upper > 0.0 {
            self.upper
        } else {
            self.lower
        };
        Self {
            lower: (lower_factor * other.lower).next_down(),
            upper: (upper_factor * other.upper).next_up(),
        }
    }

    fn div_positive(self, divisor: f64) -> Self {
        Self {
            lower: (self.lower / divisor).next_down(),
            upper: (self.upper / divisor).next_up(),
        }
    }

    fn hull(self, other: Self) -> Self {
        Self {
            lower: self.lower.min(other.lower),
            upper: self.upper.max(other.upper),
        }
    }
}
