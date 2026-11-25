//! Расход заряда и запас до отсечки на явном профиле состояний.

use std::collections::BTreeSet;

use sim_signal::Time;
use sim_world::Provenance;

use crate::{Analysis, Interval, ProcessError, SourcedRange};

use super::sim_circuit::battery::{CellParams, Table};
use super::{
    Finding, Mode, Scope, Section, Status, identifier, invalid, open, supports_source,
    validate_source,
};

pub const SLEEP_DRAIN_CODE: &str = "sleep_drain";
const NS_PER_HOUR: f64 = 3_600_000_000_000.0; // Наносекунд в часе.
const ABSOLUTE_ZERO_C: f64 = -273.15; // Ноль шкалы Кельвина в градусах Цельсия.
const WINDOW_SCOPE: &str = "ветвь питания и заданное окно автономности";

#[derive(Clone, Copy, Debug)]
pub struct SleepDrainLoad<'a> {
    pub branch: &'a Section,
    /// Вклад этой ветви в ток на клеммах батареи во всём состоянии. Для DC-DC
    /// источник покрывает преобразование по входному напряжению, нагрузке и КПД.
    /// Потери и ток покоя учитываются ровно один раз. Для тока покоя регулятора,
    /// делителей, подтяжек и модулей передаются отдельные вклады.
    pub battery_current_a: Option<&'a SourcedRange>,
}

#[derive(Clone, Copy, Debug)]
pub struct SleepDrainState<'a> {
    pub id: &'a str,
    pub duration: Time,
    /// Вклад МК, приведённый к клеммам батареи. Любая огибающая тока
    /// включает переходы и пики между тактами. Номинал и отдельный отсчёт
    /// такой гарантии не дают. При неизвестной огибающей передаётся None.
    pub controller_a: Option<&'a SourcedRange>,
    /// Полный состав прочих нагрузок. Some(&[]) означает подтверждённое отсутствие.
    pub loads: Option<&'a [SleepDrainLoad<'a>]>,
}

#[derive(Clone, Copy, Debug)]
pub struct SleepDrainProfile<'a> {
    /// Последовательные состояния с нулевой отметки. Фронт можно задать отдельным состоянием.
    pub states: &'a [SleepDrainState<'a>],
    pub repeat: bool,
    /// Подтверждает порядок, длительности, охват пиков, состав путей тока
    /// и применимость всех состояний и повторов на заявленном горизонте.
    pub source: &'a Provenance,
}

#[derive(Clone, Copy, Debug)]
pub struct SleepDrainRequirement<'a> {
    pub duration: Time,
    pub source: &'a Provenance,
}

#[derive(Clone, Copy, Debug)]
pub struct SleepDrainBattery<'a> {
    /// Эквивалент источника для тока и напряжения именно в проверяемой ветви.
    pub params: &'a CellParams,
    /// Подтверждает модель и интерполяцию по всему диапазону нагрузки, SOC и T,
    /// включая разброс батареи и применимость эквивалента. Для многозвенной
    /// батареи требуется подтверждение, что отсечка учитывает её слабые звенья.
    pub applicability: &'a Provenance,
    /// Границы фактической ёмкости при опорных 25 °C. Номинал модели их не заменяет.
    pub capacity_ah: Option<&'a SourcedRange>,
    /// Доля начального заряда по шкале опорной ёмкости, от 0 до 1.
    pub initial_soc: Option<&'a SourcedRange>,
    /// Начальные напряжения звеньев в порядке модели. Отдых задаётся явными нулями.
    pub initial_polarization_v: Option<&'a [SourcedRange]>,
    /// Постоянная температура либо огибающая температуры на всём окне.
    pub temperature_c: Option<&'a SourcedRange>,
    /// Внутренний саморазряд, эквивалентный по заряду. Через клеммы и RC он не проходит.
    pub self_discharge_a: Option<&'a SourcedRange>,
    /// Порог прекращения штатной работы при достижении напряжения клемм.
    /// Если отсечка зависит от выдержки, требуется применимый порог для этого профиля.
    pub cutoff_v: Option<&'a SourcedRange>,
    /// Абсолютная погрешность напряжения в применимой области, включая точность вычислений.
    pub voltage_error_v: Option<&'a SourcedRange>,
}

#[derive(Clone, Copy, Debug)]
pub struct SleepDrainInput<'a> {
    pub scope: Scope,
    pub mode: &'a Mode,
    pub branch: &'a Section,
    pub required: Option<SleepDrainRequirement<'a>>,
    pub profile: Option<SleepDrainProfile<'a>>,
    pub battery: Option<SleepDrainBattery<'a>>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SleepDrainBounds {
    pub min: f64,
    pub max: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SleepDrainConsumption {
    pub controller_ah: SleepDrainBounds,
    pub other_loads_ah: SleepDrainBounds,
    pub self_discharge_ah: SleepDrainBounds,
    pub total_ah: SleepDrainBounds,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SleepDrainCutoffViolation {
    pub state: String,
    /// Подтверждающая отметка на границе состояния, с током этого состояния.
    /// Фактическое прекращение работы могло произойти раньше.
    pub time: Time,
    pub margin_v: SleepDrainBounds,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SleepDrainEvidence {
    pub code: &'static str,
    pub scope: Scope,
    pub mode: Mode,
    pub branch: Section,
    pub window: Option<Interval>,
    pub consumption: Analysis<SleepDrainConsumption>,
    pub charge_margin_ah: Analysis<SleepDrainBounds>,
    /// Минимальное напряжение в окне минус порог. Нижняя граница учитывает
    /// всё окно, верхняя подтверждается на границах состояний.
    pub cutoff_margin_v: Analysis<SleepDrainBounds>,
    pub cutoff_violation: Option<SleepDrainCutoffViolation>,
}

pub fn assess_sleep_drain(
    input: &SleepDrainInput<'_>,
) -> Result<Finding<SleepDrainEvidence>, ProcessError> {
    let subject = validate(input)?;
    let mut result = Finding {
        subject,
        status: Status::NotAnalyzed,
        reason: "нет подтверждённого срока автономности".into(),
        evidence: SleepDrainEvidence {
            code: SLEEP_DRAIN_CODE,
            scope: input.scope,
            mode: input.mode.clone(),
            branch: input.branch.clone(),
            window: input.required.map(|r| Interval {
                start: Time::ZERO,
                end: r.duration,
            }),
            consumption: open("нет расчёта расхода заряда"),
            charge_margin_ah: open("нет расчёта зарядового запаса"),
            cutoff_margin_v: open("нет расчёта запаса до отсечки"),
            cutoff_violation: None,
        },
    };
    let scope = input.scope;
    let Some(required) = input.required.filter(|r| supports_source(scope, r.source)) else {
        return Ok(result);
    };
    let Some(profile) = input.profile.filter(|p| supports_source(scope, p.source)) else {
        result.reason = "нет подтверждённого профиля сна, пробуждений и нагрузок".into();
        return Ok(result);
    };
    let Some(schedule) = schedule(scope, profile, required.duration)? else {
        result.reason =
            "профиль не покрывает срок или не подтверждены огибающие всех токов с пиками".into();
        return Ok(result);
    };
    let Some(battery) = input.battery else {
        result.reason = "нет модели батареи и применимой отсечки".into();
        return Ok(result);
    };
    let Some(self_discharge) = proven(scope, battery.self_discharge_a) else {
        result.reason = "нет подтверждённой огибающей саморазряда".into();
        return Ok(result);
    };
    let [controller, others] = schedule.charge;
    let internal = self_discharge.mul_nonnegative(hours(required.duration.0));
    let total = controller.add(others).add(internal);
    total.check()?;
    result.evidence.consumption = analyzed(SleepDrainConsumption {
        controller_ah: controller,
        other_loads_ah: others,
        self_discharge_ah: internal,
        total_ah: total,
    });
    let Some(model) = BatteryModel::new(scope, battery)? else {
        result.reason =
            "нет применимой модели, условий батареи или подтверждённого порога отсечки".into();
        return Ok(result);
    };
    let reserve = model.available_ah.sub(total).check()?;
    result.evidence.charge_margin_ah = analyzed(reserve);
    if reserve.max < 0.0 {
        result.status = Status::Fail;
        result.reason = "батарея исчерпывает доступный заряд раньше заявленного срока".into();
        return Ok(result);
    }
    let Some((margin, violation)) = cutoff(&model, &schedule, self_discharge, required.duration)?
    else {
        result.reason =
            "SOC выходит за область подтверждённых кривых или доступный заряд может исчерпаться"
                .into();
        return Ok(result);
    };
    result.evidence.cutoff_margin_v = analyzed(margin);
    result.evidence.cutoff_violation = violation;
    (result.status, result.reason) = if result.evidence.cutoff_violation.is_some() {
        (
            Status::Fail,
            "под нагрузкой отсечка достигается раньше заявленного срока".into(),
        )
    } else if reserve.min >= 0.0 && margin.min > 0.0 {
        (
            Status::Pass,
            "заряда хватает и отсечка не достигается во всём заявленном окне".into(),
        )
    } else {
        (
            Status::NotAnalyzed,
            "огибающая запаса пересекает границу отсечки или исчерпания заряда".into(),
        )
    };
    Ok(result)
}

type Bounds = SleepDrainBounds;

impl Bounds {
    const ZERO: Self = Self { min: 0.0, max: 0.0 };
    const ONE: Self = Self { min: 1.0, max: 1.0 };

    fn check(self) -> Result<Self, ProcessError> {
        if self.min.is_finite() && self.max.is_finite() && self.min <= self.max {
            Ok(self)
        } else {
            Err(invalid(
                "неконечные или неверно упорядоченные границы sleep_drain",
            ))
        }
    }

    fn add(self, rhs: Self) -> Self {
        if rhs == Self::ZERO {
            return self;
        }
        if self == Self::ZERO {
            return rhs;
        }
        let sum = Self::outer(self.min + rhs.min, self.max + rhs.max);
        if self.min >= 0.0 && rhs.min >= 0.0 {
            sum.nonnegative()
        } else {
            sum
        }
    }

    fn sub(self, rhs: Self) -> Self {
        if rhs == Self::ZERO {
            return self;
        }
        Self::outer(self.min - rhs.max, self.max - rhs.min)
    }

    fn mul_nonnegative(self, rhs: Self) -> Self {
        if self == Self::ZERO || rhs == Self::ZERO {
            return Self::ZERO;
        }
        Self::outer(self.min * rhs.min, self.max * rhs.max).nonnegative()
    }

    fn div_positive(self, rhs: Self) -> Self {
        if self == Self::ZERO {
            return Self::ZERO;
        }
        Self::outer(self.min / rhs.max, self.max / rhs.min).nonnegative()
    }

    fn nonnegative(self) -> Self {
        Self {
            min: self.min.max(0.0),
            max: self.max.max(0.0),
        }
    }

    // Округление наружу на каждом шаге зарядового баланса. Неконечные
    // результаты сохраняются для явной проверки, а не насыщаются.
    fn outer(min: f64, max: f64) -> Self {
        Self {
            min: if min.is_finite() {
                min.next_down()
            } else {
                min
            },
            max: if max.is_finite() { max.next_up() } else { max },
        }
    }

    fn hull(self, rhs: Self) -> Self {
        Self {
            min: self.min.min(rhs.min),
            max: self.max.max(rhs.max),
        }
    }
}

fn analyzed<T>(value: T) -> Analysis<T> {
    Analysis::Analyzed {
        value,
        scope: WINDOW_SCOPE.into(),
    }
}

fn proven(scope: Scope, range: Option<&SourcedRange>) -> Option<Bounds> {
    range
        .filter(|r| supports_source(scope, &r.provenance))
        .map(|r| Bounds {
            min: r.min,
            max: r.max,
        })
}

fn count(n: u64) -> Bounds {
    let float = n as f64;
    // Целые до 2^53 включительно представимы точно в IEEE 754 binary64.
    if n <= 1 << f64::MANTISSA_DIGITS {
        Bounds {
            min: float,
            max: float,
        }
    } else {
        Bounds::outer(float, float)
    }
}

fn hours(ns: u64) -> Bounds {
    count(ns).div_positive(Bounds {
        min: NS_PER_HOUR,
        max: NS_PER_HOUR,
    })
}

fn section_name(section: &Section) -> Result<String, ProcessError> {
    match section {
        Section::Element { component, element } if identifier(component) && identifier(element) => {
            Ok(format!("{component}/{element}"))
        }
        Section::Connection(id) if identifier(id) => Ok(id.clone()),
        _ => Err(invalid("недопустимая ветвь проверки автономности")),
    }
}

fn validate_range(
    range: Option<&SourcedRange>,
    valid: impl Fn(f64) -> bool,
) -> Result<(), ProcessError> {
    if let Some(range) = range {
        range.validate()?;
        if !valid(range.min) || !valid(range.max) {
            return Err(invalid("вход sleep_drain вне физической области"));
        }
    }
    Ok(())
}

fn validate(input: &SleepDrainInput<'_>) -> Result<String, ProcessError> {
    let subject = section_name(input.branch)?;
    if let Some(required) = input.required {
        validate_source(required.source)?;
        if required.duration == Time::ZERO {
            return Err(invalid("нулевой срок автономности"));
        }
    }
    if let Some(profile) = input.profile {
        validate_source(profile.source)?;
        for state in profile.states {
            if !identifier(state.id) || state.duration == Time::ZERO {
                return Err(invalid("пустой ID или нулевая длительность состояния"));
            }
            validate_range(state.controller_a, |x| x >= 0.0)?;
            let mut branches = BTreeSet::new();
            for load in state.loads.unwrap_or_default() {
                section_name(load.branch)?;
                if load.branch == input.branch || !branches.insert(load.branch) {
                    return Err(invalid(
                        "ток ветви питания или повторный вклад ветви нагрузки",
                    ));
                }
                validate_range(load.battery_current_a, |x| x >= 0.0)?;
            }
        }
    }
    if let Some(battery) = input.battery {
        validate_source(battery.applicability)?;
        validate_range(battery.capacity_ah, |x| x > 0.0)?;
        validate_range(battery.initial_soc, |x| (0.0..=1.0).contains(&x))?;
        validate_range(battery.temperature_c, |x| x > ABSOLUTE_ZERO_C)?;
        validate_range(battery.self_discharge_a, |x| x >= 0.0)?;
        validate_range(battery.cutoff_v, |x| x > 0.0)?;
        validate_range(battery.voltage_error_v, |x| x >= 0.0)?;
        if let Some(initial) = battery.initial_polarization_v {
            if initial.len() != battery.params.rc.len() {
                return Err(invalid(
                    "начальные напряжения не соответствуют числу звеньев RC",
                ));
            }
            for v in initial {
                v.validate()?;
            }
        }
        validate_model(battery.params)?;
    }
    Ok(subject)
}

fn validate_model(p: &CellParams) -> Result<(), ProcessError> {
    if ![
        p.capacity_ah,
        p.nominal_v,
        p.v_min,
        p.v_max,
        p.ea_r0,
        p.ea_rc,
        p.mass_kg,
        p.heat_capacity_j_k,
        p.charge_temp_c.0,
        p.charge_temp_c.1,
        p.discharge_temp_c.0,
        p.discharge_temp_c.1,
    ]
    .into_iter()
    .all(f64::is_finite)
        || p.discharge_temp_c.0 > p.discharge_temp_c.1
        || p.max_charge_a
            .iter()
            .chain(&p.max_discharge_a)
            .any(|&(t, i)| !t.is_finite() || !i.is_finite())
    {
        return Err(invalid("недопустимые параметры модели батареи"));
    }
    let soc = |z| (0.0..=1.0).contains(&z);
    validate_table(&p.ocv, soc, |v| v > 0.0)?;
    validate_table(&p.entropic, soc, |_| true)?;
    validate_table(&p.r0, soc, |r| r >= 0.0)?;
    validate_table(&p.capacity_factor, |t| t > ABSOLUTE_ZERO_C, |f| f > 0.0)?;
    for pole in &p.rc {
        if !pole.c.is_finite() || pole.c <= 0.0 {
            return Err(invalid("недопустимая ёмкость звена RC батареи"));
        }
        validate_table(&pole.r, soc, |r| r > 0.0)?;
    }
    Ok(())
}

fn validate_table(
    table: &Table,
    axis: impl Fn(f64) -> bool,
    value: impl Fn(f64) -> bool,
) -> Result<(), ProcessError> {
    if table
        .0
        .iter()
        .any(|&(x, y)| !x.is_finite() || !y.is_finite() || !axis(x) || !value(y))
        || table.0.windows(2).any(|p| p[0].0 >= p[1].0)
    {
        return Err(invalid("недопустимая таблица модели батареи"));
    }
    Ok(())
}

struct Phase<'a> {
    id: &'a str,
    offset: u64,
    duration: u64,
    currents: [Bounds; 2],
    prefix_ah: Bounds,
}

impl Phase<'_> {
    fn current(&self) -> Bounds {
        self.currents[0].add(self.currents[1])
    }
}

struct Schedule<'a> {
    phases: Vec<Phase<'a>>,
    period: u64,
    full_cycles: u64,
    tail: u64,
    cycle_ah: Bounds,
    charge: [Bounds; 2],
}

fn schedule<'a>(
    scope: Scope,
    profile: SleepDrainProfile<'a>,
    horizon: Time,
) -> Result<Option<Schedule<'a>>, ProcessError> {
    let period = profile
        .states
        .iter()
        .try_fold(0u64, |sum, s| sum.checked_add(s.duration.0))
        .ok_or_else(|| invalid("длительность профиля вне шкалы времени"))?;
    if period == 0 || (!profile.repeat && period < horizon.0) {
        return Ok(None);
    }
    let full_cycles = horizon.0 / period;
    let tail = horizon.0 % period;
    let mut offset = 0;
    let mut phases = Vec::new();
    let mut cycle = [Bounds::ZERO; 2];
    let mut partial = [Bounds::ZERO; 2];
    for state in profile.states {
        let duration = state.duration.0.min(horizon.0 - offset);
        let Some(controller) = proven(scope, state.controller_a) else {
            return Ok(None);
        };
        let Some(loads) = state.loads else {
            return Ok(None);
        };
        let mut others = Bounds::ZERO;
        for load in loads {
            let Some(current) = proven(scope, load.battery_current_a) else {
                return Ok(None);
            };
            others = others.add(current);
        }
        let currents = [controller, others.check()?];
        phases.push(Phase {
            id: state.id,
            offset,
            duration,
            currents,
            prefix_ah: cycle[0].add(cycle[1]),
        });
        let tail_duration = tail.saturating_sub(offset).min(duration);
        for (i, current) in currents.into_iter().enumerate() {
            cycle[i] = cycle[i]
                .add(current.mul_nonnegative(hours(duration)))
                .check()?;
            partial[i] = partial[i]
                .add(current.mul_nonnegative(hours(tail_duration)))
                .check()?;
        }
        offset += duration;
        if offset == horizon.0 {
            break;
        }
    }
    let charge =
        std::array::from_fn(|i| cycle[i].mul_nonnegative(count(full_cycles)).add(partial[i]));
    Ok(Some(Schedule {
        phases,
        period,
        full_cycles,
        tail,
        cycle_ah: cycle[0].add(cycle[1]).check()?,
        charge,
    }))
}

fn covers(table: &Table, axis: Bounds) -> bool {
    table.0.first().is_some_and(|p| p.0 <= axis.min)
        && table.0.last().is_some_and(|p| axis.max <= p.0)
}

fn electrical_tables(p: &CellParams) -> impl Iterator<Item = &Table> {
    [&p.ocv, &p.entropic, &p.r0]
        .into_iter()
        .chain(p.rc.iter().map(|rc| &rc.r))
}

fn extrema<'a>(
    tables: impl IntoIterator<Item = &'a Table>,
    axis: Bounds,
    temp: Bounds,
    eval: impl Fn(f64, f64) -> f64,
) -> Result<Bounds, ProcessError> {
    let first = eval(axis.min, temp.min);
    let mut bounds = Bounds {
        min: first,
        max: first,
    }
    .check()?;
    let knots = [axis.min, axis.max]
        .into_iter()
        .chain(tables.into_iter().flat_map(|table| {
            let start = table.0.partition_point(|p| p.0 <= axis.min);
            table.0[start..]
                .iter()
                .map(|p| p.0)
                .take_while(move |&x| x < axis.max)
        }));
    for x in knots {
        for t in [temp.min, temp.max] {
            let value = eval(x, t);
            bounds = bounds.hull(
                Bounds {
                    min: value,
                    max: value,
                }
                .check()?,
            );
        }
    }
    Ok(bounds)
}

struct BatteryModel<'a> {
    params: &'a CellParams,
    temperature: Bounds,
    capacity: Bounds,
    factor: Bounds,
    initial_soc: Bounds,
    initial_polarization: Vec<Bounds>,
    available_ah: Bounds,
    cutoff: Bounds,
    error_v: f64,
}

impl<'a> BatteryModel<'a> {
    fn new(scope: Scope, battery: SleepDrainBattery<'a>) -> Result<Option<Self>, ProcessError> {
        let get = |r| proven(scope, r);
        let (
            Some(temperature),
            Some(capacity),
            Some(initial_soc),
            Some(initial),
            Some(cutoff),
            Some(error),
        ) = (
            get(battery.temperature_c),
            get(battery.capacity_ah),
            get(battery.initial_soc),
            battery.initial_polarization_v,
            get(battery.cutoff_v),
            get(battery.voltage_error_v),
        )
        else {
            return Ok(None);
        };
        let p = battery.params;
        if !supports_source(scope, battery.applicability)
            || temperature.min < p.discharge_temp_c.0
            || temperature.max > p.discharge_temp_c.1
            || !covers(&p.capacity_factor, temperature)
            || electrical_tables(p).any(|table| table.0.is_empty())
        {
            return Ok(None);
        }
        let Some(initial_polarization) = initial.iter().map(|r| get(Some(r))).collect() else {
            return Ok(None);
        };
        let factor = extrema([&p.capacity_factor], temperature, temperature, |t, _| {
            p.capacity_factor.eval(t)
        })?;
        // При изменении T шкала эффективного SOC сжимается от полного заряда.
        let deficit = Bounds::ONE.sub(initial_soc).nonnegative();
        let available_ah = factor
            .sub(deficit)
            .check()?
            .nonnegative()
            .mul_nonnegative(capacity)
            .check()?;
        Ok(Some(Self {
            params: p,
            temperature,
            capacity,
            factor,
            initial_soc,
            initial_polarization,
            available_ah,
            cutoff,
            error_v: error.max,
        }))
    }

    fn soc(&self, charge: Bounds) -> Result<Bounds, ProcessError> {
        let removed = Bounds::ONE
            .sub(self.initial_soc)
            .nonnegative()
            .add(charge.div_positive(self.capacity));
        let soc = Bounds::ONE.sub(removed.div_positive(self.factor)).check()?;
        // Заряд не возрастает, так что выход выше 1 возможен лишь от округления.
        Ok(Bounds {
            min: soc.min,
            max: soc.max.min(1.0),
        })
    }

    fn margin(&self, soc: Bounds, i: Bounds, polarization: Bounds) -> Result<Bounds, ProcessError> {
        let p = self.params;
        // При фиксированной T сумма OCV и омического падения кусочно-линейна по SOC.
        // Для диапазона T отдельно ограничиваем линейную OCV и монотонный множитель R0.
        let loaded = if self.temperature.min == self.temperature.max {
            let tables = [&p.ocv, &p.entropic, &p.r0];
            let min = extrema(tables, soc, self.temperature, |z, t| {
                p.ocv_at(z, t) - i.max * p.r0_at(z, t)
            })?
            .min;
            let max = extrema(tables, soc, self.temperature, |z, t| {
                p.ocv_at(z, t) - i.min * p.r0_at(z, t)
            })?
            .max;
            Bounds { min, max }
        } else {
            let ocv = extrema([&p.ocv, &p.entropic], soc, self.temperature, |z, t| {
                p.ocv_at(z, t)
            })?;
            let r0 = extrema([&p.r0], soc, self.temperature, |z, t| p.r0_at(z, t))?;
            ocv.sub(Bounds {
                min: r0.min * i.min,
                max: r0.max * i.max,
            })
        };
        loaded
            .sub(polarization)
            .add(Bounds {
                min: -self.error_v,
                max: self.error_v,
            })
            .sub(self.cutoff)
            .check()
    }
}

/// Постоянные R и крайние токи задают решения сравнения для Vp ≥ 0:
/// Vp' = I/C − Vp/(RC) возрастает по I и R. Начальную отрицательную
/// нижнюю границу можно удерживать постоянной при любом разрядном токе.
struct RcBound {
    resistance: f64,
    tau: f64,
    fixed: bool,
    state: [f64; 3],
}

impl RcBound {
    fn new(
        initial: f64,
        r: f64,
        c: f64,
        upper: bool,
        schedule: &Schedule<'_>,
    ) -> Result<Self, ProcessError> {
        let tau = r * c;
        if !tau.is_finite() || tau <= 0.0 {
            return Err(invalid("недопустимая постоянная времени звена RC"));
        }
        let start = if upper { initial.max(0.0) } else { initial };
        let mut bound = Self {
            resistance: r,
            tau,
            fixed: start < 0.0,
            state: [start; 3],
        };
        if !bound.fixed && schedule.full_cycles > 0 {
            let mut increment = 0.0;
            for phase in &schedule.phases {
                let current = phase.current();
                increment = bound.advance(
                    increment,
                    if upper { current.max } else { current.min },
                    phase.duration,
                );
            }
            // За полный период динамика аффинна. Сумма геометрической прогрессии
            // через exp_m1 сохраняет точность для периода, короткого относительно RC.
            let x = -Time(schedule.period).as_secs() / tau;
            for (track, cycles) in [(1, schedule.full_cycles - 1), (2, schedule.full_cycles)] {
                if cycles > 0 {
                    let nx = x * cycles as f64;
                    bound.state[track] = start * nx.exp() + increment * (nx.exp_m1() / x.exp_m1());
                }
            }
        }
        if bound.state.iter().any(|v| !v.is_finite()) {
            return Err(invalid(
                "неконечное начальное состояние расчёта поляризации",
            ));
        }
        Ok(bound)
    }

    fn advance(&self, initial: f64, current: f64, duration: u64) -> f64 {
        if self.fixed {
            return initial;
        }
        let x = -Time(duration).as_secs() / self.tau;
        initial * x.exp() - (current * self.resistance) * x.exp_m1()
    }
}

fn cutoff(
    model: &BatteryModel<'_>,
    schedule: &Schedule<'_>,
    self_discharge: Bounds,
    horizon: Time,
) -> Result<Option<(Bounds, Option<SleepDrainCutoffViolation>)>, ProcessError> {
    let p = model.params;
    let [controller, others] = schedule.charge;
    let total = controller
        .add(others)
        .add(self_discharge.mul_nonnegative(hours(horizon.0)));
    let span = model.soc(total)?.hull(model.soc(Bounds::ZERO)?);
    if span.min < 0.0 || !electrical_tables(p).all(|table| covers(table, span)) {
        return Ok(None);
    }
    let mut poles = Vec::new();
    for (k, rc) in p.rc.iter().enumerate() {
        let r = extrema([&rc.r], span, model.temperature, |z, t| p.rc_r_at(k, z, t))?;
        let initial = model.initial_polarization[k];
        poles.push((
            RcBound::new(initial.min, r.min, rc.c, false, schedule)?,
            RcBound::new(initial.max, r.max, rc.c, true, schedule)?,
        ));
    }
    let mut margin: Option<Bounds> = None;
    let mut violation: Option<SleepDrainCutoffViolation> = None;
    for phase in &schedule.phases {
        let tail = schedule
            .tail
            .saturating_sub(phase.offset)
            .min(phase.duration);
        let full = schedule.full_cycles;
        // Начала одноимённых фаз промежуточных циклов лежат между началом
        // первого и последнего полного цикла для каждого решения сравнения.
        let occurrences = [
            (0, phase.duration),
            (
                full.saturating_sub(1),
                if full > 1 { phase.duration } else { 0 },
            ),
            (full, if full > 0 { tail } else { 0 }),
        ];
        let mut during_rc: Option<Bounds> = None;
        let mut during_soc: Option<Bounds> = None;
        let mut upper: Option<f64> = None;
        let i = phase.current().check()?;
        for (track, (cycle, dt)) in occurrences
            .into_iter()
            .enumerate()
            .filter(|(_, (_, dt))| *dt > 0)
        {
            let start = cycle * schedule.period + phase.offset;
            let end = start + dt;
            let terminal_before = schedule
                .cycle_ah
                .mul_nonnegative(count(cycle))
                .add(phase.prefix_ah);
            let q_start = terminal_before.add(self_discharge.mul_nonnegative(hours(start)));
            let q_end = terminal_before
                .add(i.mul_nonnegative(hours(dt)))
                .add(self_discharge.mul_nonnegative(hours(end)));
            let soc_start = model.soc(q_start)?;
            let soc_end = model.soc(q_end)?;
            let soc = soc_start.hull(soc_end);
            during_soc = Some(during_soc.map_or(soc, |span| span.hull(soc)));

            let mut before = Bounds::ZERO;
            let mut after = Bounds::ZERO;
            let mut rc_span = Bounds::ZERO;
            for (low, high) in &mut poles {
                let initial = Bounds {
                    min: low.state[track],
                    max: high.state[track],
                }
                .check()?;
                low.state[track] = low.advance(initial.min, i.min, dt);
                high.state[track] = high.advance(initial.max, i.max, dt);
                let final_v = Bounds {
                    min: low.state[track],
                    max: high.state[track],
                }
                .check()?;
                before = before.add(initial);
                after = after.add(final_v);
                rc_span = rc_span.add(initial.hull(final_v));
            }
            during_rc = Some(during_rc.map_or(rc_span, |span| span.hull(rc_span)));
            for (time, soc, rc) in [(start, soc_start, before), (end, soc_end, after)] {
                let at = model.margin(soc, i, rc)?;
                upper = Some(upper.map_or(at.max, |bound| bound.min(at.max)));
                // Строгое нарушение на правой границе относится к левому пределу.
                // Ровно порог в конце окна оставляет результат открытым.
                if (at.max < 0.0 || (time < horizon.0 && at.max <= 0.0))
                    && violation
                        .as_ref()
                        .is_none_or(|previous| Time(time) < previous.time)
                {
                    violation = Some(SleepDrainCutoffViolation {
                        state: phase.id.into(),
                        time: Time(time),
                        margin_v: at,
                    });
                }
            }
        }
        if let (Some(soc), Some(rc), Some(upper)) = (during_soc, during_rc, upper) {
            if !electrical_tables(p).all(|table| covers(table, soc)) {
                return Ok(None);
            }
            let lower = model.margin(soc, i, rc)?.min;
            margin = Some(margin.map_or(
                Bounds {
                    min: lower,
                    max: upper,
                },
                |m| Bounds {
                    min: m.min.min(lower),
                    max: m.max.min(upper),
                },
            ));
        }
    }
    margin.map(|m| Ok((m.check()?, violation))).transpose()
}
