//! Запас конденсатора по напряжению, пульсациям и перегреву в одном окне режима.

use std::ops::Range;

use sim_signal::Time;
use sim_world::Provenance;

use crate::schematic::{ElectricalSeries, Endpoint, Finding, Mode, Probe, Scope, Section, Status};
use crate::{Analysis, Interval, ProcessError, SourcedRange};

pub const CAPACITOR_STRESS_CODE: &str = "capacitor_stress";
const WINDOW_SCOPE: &str = "ветвь и окно указанного режима";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CapacitorVoltageSense {
    /// V(+) − V(−). Обратное включение проверяет `reverse_polarity`.
    Forward,
    Absolute,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CapacitorTemperatureBasis {
    Ambient,
    Case,
    Hotspot,
}

/// Огибающая, гарантированная источником внутри каждой ячейки f/T.
/// Ось из одной точки применима только в этой точке. На внутренней границе
/// действует правая ячейка. Крайняя правая граница включена.
/// Ячейки перечислены строками температур, частота меняется быстрее.
/// Пропуск ячейки оставляет соответствующую часть огибающей открытой.
#[derive(Clone, Copy, Debug)]
pub struct CapacitorCurve<'a> {
    /// `None` допустим для derating, когда источник подтверждает его независимость от частоты.
    pub frequencies_hz: Option<&'a [f64]>,
    pub temperatures_k: &'a [f64],
    pub temperature_basis: CapacitorTemperatureBasis,
    pub cells: &'a [Option<SourcedRange>],
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CapacitorRatings<'a> {
    pub rated_voltage_v: Option<&'a SourcedRange>,
    pub voltage_derating: Option<CapacitorCurve<'a>>,
    /// Предел полного RMS для состава спектра и длительности заявленного опыта.
    pub ripple_rms_a: Option<CapacitorCurve<'a>>,
    /// ESR для оценки средних потерь во всей полосе тока в этом окне.
    pub esr_ohm: Option<CapacitorCurve<'a>>,
    pub max_hotspot_k: Option<&'a SourcedRange>,
}

/// Огибающие физических температур на всём окне. Температуру для кривой
/// подбирают по месту измерения, указанному в её источнике.
#[derive(Clone, Copy, Debug, Default)]
pub struct CapacitorTemperatures<'a> {
    pub ambient_k: Option<&'a SourcedRange>,
    pub case_k: Option<&'a SourcedRange>,
    pub hotspot_k: Option<&'a SourcedRange>,
}

#[derive(Clone, Copy, Debug)]
pub enum CapacitorThermal<'a> {
    /// Границы максимума температуры горячей точки на всём окне.
    MeasuredPeak(&'a SourcedRange),
    /// Для установившегося опыта. Источники должны покрывать монтаж, охлаждение,
    /// окружающий нагрев, период усреднения и разность пика и средней температуры.
    SteadyState {
        thermal_resistance_k_per_w: &'a SourcedRange,
        peak_above_mean_k: &'a SourcedRange,
    },
}

#[derive(Clone, Copy, Debug)]
pub struct CapacitorStressInput<'a> {
    pub scope: Scope,
    pub mode: &'a Mode,
    pub branch: &'a Section,
    pub terminals: &'a [Endpoint; 2],
    pub voltage_sense: CapacitorVoltageSense,
    pub window: Interval,
    /// Каналы терминального напряжения и тока ветви должны покрывать обе границы окна.
    pub series: Option<&'a ElectricalSeries>,
    /// Обоснованные абсолютные погрешности кусочно-линейных рядов во всём окне.
    /// Сюда входят погрешности модели или замера, событий и экстремумов между отсчётами.
    pub voltage_error_v: Option<&'a SourcedRange>,
    pub current_error_a: Option<&'a SourcedRange>,
    /// Границы всего спектра тока, относящегося к пульсациям и потерям в окне.
    /// Шаг записи, частота ШИМ и пик спектра сами по себе такую полосу не задают.
    pub frequency_hz: Option<&'a SourcedRange>,
    pub temperatures: CapacitorTemperatures<'a>,
    pub thermal: Option<CapacitorThermal<'a>>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CapacitorBounds {
    pub min: f64,
    pub max: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CapacitorMargin {
    pub observed: CapacitorBounds,
    pub limit: CapacitorBounds,
    /// Границы разности «допустимое − наблюдаемое» в единицах показателя.
    pub margin: CapacitorBounds,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CapacitorStressEvidence {
    pub code: &'static str,
    pub scope: Scope,
    pub mode: Mode,
    pub branch: Section,
    pub terminals: [Endpoint; 2],
    pub window: Interval,
    pub voltage_peak_sample: Option<Time>,
    pub voltage_v: Analysis<CapacitorMargin>,
    pub ripple_a: Analysis<CapacitorMargin>,
    pub loss_w: Analysis<CapacitorBounds>,
    pub hotspot_k: Analysis<CapacitorMargin>,
}

/// Проверяет только нагрузку указанного конденсатора при применимости источников в этом окне.
pub fn assess_capacitor_stress(
    input: &CapacitorStressInput<'_>,
    ratings: &CapacitorRatings<'_>,
) -> Result<Finding<CapacitorStressEvidence>, ProcessError> {
    let subject = validate(input, ratings)?;
    let bounds = |range| proven_bounds(input.scope, range);
    let curve = |value: Option<CapacitorCurve<'_>>| value.and_then(|curve| curve.bounds(input));
    let Observation {
        voltage,
        current,
        voltage_peak_sample,
    } = observation(input)?;
    let voltage_limit = product(
        bounds(ratings.rated_voltage_v),
        curve(ratings.voltage_derating),
    )?;
    let frequency_curve = |value: Option<CapacitorCurve<'_>>| {
        curve(value.filter(|curve| curve.frequencies_hz.is_some()))
    };
    let ripple_limit = frequency_curve(ratings.ripple_rms_a);
    // Средние потери за окно I_rms²·ESR с огибающей ESR по всей полосе и температуре.
    let loss = product(product(current, frequency_curve(ratings.esr_ohm))?, current)?;
    let temperature = match input.thermal {
        Some(CapacitorThermal::MeasuredPeak(peak)) => bounds(Some(peak)),
        Some(CapacitorThermal::SteadyState {
            thermal_resistance_k_per_w,
            peak_above_mean_k,
        }) => {
            let heating = product(loss, bounds(Some(thermal_resistance_k_per_w)))?;
            sum(
                sum(bounds(input.temperatures.ambient_k), heating)?,
                bounds(Some(peak_above_mean_k)),
            )?
        }
        None => None,
    };
    if let Some((peak, range)) = temperature.zip(bounds(input.temperatures.hotspot_k))
        && (peak.min > range.max || peak.max < range.min)
    {
        return Err(invalid("пик температуры противоречит её огибающей в окне"));
    }
    let evidence = CapacitorStressEvidence {
        code: CAPACITOR_STRESS_CODE,
        scope: input.scope,
        mode: input.mode.clone(),
        branch: input.branch.clone(),
        terminals: input.terminals.clone(),
        window: input.window,
        voltage_peak_sample,
        voltage_v: margin(
            voltage,
            voltage_limit,
            "нет подтверждённой огибающей напряжения между отсчётами во всём окне",
            "нет применимого паспортного напряжения или derating",
        )?,
        ripple_a: margin(
            current,
            ripple_limit,
            "нет подтверждённого ряда тока во всём окне",
            "нет допустимого RMS пульсаций для всей полосы и температуры",
        )?,
        loss_w: analyzed(
            loss,
            "нет ряда тока или подтверждённой огибающей ESR по частоте и температуре",
        ),
        hotspot_k: margin(
            temperature,
            bounds(ratings.max_hotspot_k),
            "нет подтверждённого пика температуры или параметров установившегося нагрева",
            "нет допустимой температуры горячей точки",
        )?,
    };
    let (status, reason) = summarize(&evidence);
    Ok(Finding {
        subject,
        status,
        reason,
        evidence,
    })
}

fn validate(
    input: &CapacitorStressInput<'_>,
    ratings: &CapacitorRatings<'_>,
) -> Result<String, ProcessError> {
    let Section::Element { component, element } = input.branch else {
        return Err(invalid("нужна ветвь элемента конденсатора"));
    };
    if input.window.start >= input.window.end
        || input.terminals[0] == input.terminals[1]
        || input
            .terminals
            .iter()
            .any(|end| &end.component != component)
        || [
            component,
            element,
            &input.terminals[0].pin,
            &input.terminals[1].pin,
        ]
        .into_iter()
        .any(|name| name.trim().is_empty() || name.chars().any(char::is_control))
    {
        return Err(invalid(
            "недопустимые ветвь, выводы или окно проверки конденсатора",
        ));
    }
    for range in [
        input.voltage_error_v,
        input.current_error_a,
        input.frequency_hz,
    ] {
        validate_range(range, |v| v >= 0.0)?;
    }
    for range in [
        input.temperatures.ambient_k,
        input.temperatures.case_k,
        input.temperatures.hotspot_k,
        ratings.rated_voltage_v,
        ratings.max_hotspot_k,
    ] {
        validate_range(range, |v| v > 0.0)?;
    }
    match input.thermal {
        Some(CapacitorThermal::MeasuredPeak(peak)) => validate_range(Some(peak), |v| v > 0.0)?,
        Some(CapacitorThermal::SteadyState {
            thermal_resistance_k_per_w,
            peak_above_mean_k,
        }) => {
            validate_range(Some(thermal_resistance_k_per_w), |v| v >= 0.0)?;
            validate_range(Some(peak_above_mean_k), |v| v >= 0.0)?;
        }
        None => {}
    }
    if let Some(curve) = ratings.voltage_derating {
        curve.validate(|v| (0.0..=1.0).contains(&v))?;
    }
    for curve in [ratings.ripple_rms_a, ratings.esr_ohm]
        .into_iter()
        .flatten()
    {
        curve.validate(|v| v >= 0.0)?;
    }
    if let Some(series) = input.series
        && (series.time.windows(2).any(|w| w[0] >= w[1])
            || series
                .values
                .values()
                .any(|v| v.len() != series.time.len() || v.iter().any(|x| !x.is_finite())))
    {
        return Err(invalid(
            "электрический ряд неполный, неконечный или время немонотонно",
        ));
    }
    Ok(format!("{component}/{element}"))
}

fn invalid(reason: &str) -> ProcessError {
    ProcessError::InvalidInput(reason.into())
}

fn validate_range(
    range: Option<&SourcedRange>,
    accepts: impl Fn(f64) -> bool,
) -> Result<(), ProcessError> {
    if let Some(range) = range {
        range.validate()?;
        if !accepts(range.min) || !accepts(range.max) {
            return Err(invalid("величина конденсатора вне физических границ"));
        }
    }
    Ok(())
}

fn proven_bounds(scope: Scope, range: Option<&SourcedRange>) -> Option<CapacitorBounds> {
    let range = range?;
    // Расчётные характеристики стенда остаются в области Reference.
    (range.can_prove()
        || (scope == Scope::Reference && matches!(&range.provenance, Provenance::Derived(_))))
    .then_some(CapacitorBounds {
        min: range.min,
        max: range.max,
    })
}

impl CapacitorCurve<'_> {
    fn validate(self, accepts: impl Fn(f64) -> bool) -> Result<(), ProcessError> {
        let valid_axis = |axis: &[f64], valid: fn(f64) -> bool| {
            !axis.is_empty()
                && axis.iter().all(|&x| x.is_finite() && valid(x))
                && axis.windows(2).all(|w| w[0] < w[1])
        };
        let columns = self.frequencies_hz.map_or(1, cell_count);
        if self
            .frequencies_hz
            .is_some_and(|f| !valid_axis(f, |x| x >= 0.0))
            || !valid_axis(self.temperatures_k, |x| x > 0.0)
            || columns.checked_mul(cell_count(self.temperatures_k)) != Some(self.cells.len())
        {
            return Err(invalid(
                "недопустимая сетка паспортной огибающей конденсатора",
            ));
        }
        for cell in self.cells {
            validate_range(cell.as_ref(), &accepts)?;
        }
        Ok(())
    }

    fn bounds(self, input: &CapacitorStressInput<'_>) -> Option<CapacitorBounds> {
        let range = match self.temperature_basis {
            CapacitorTemperatureBasis::Ambient => input.temperatures.ambient_k,
            CapacitorTemperatureBasis::Case => input.temperatures.case_k,
            CapacitorTemperatureBasis::Hotspot => input.temperatures.hotspot_k,
        };
        let rows = covering_cells(self.temperatures_k, proven_bounds(input.scope, range)?)?;
        let cols = match self.frequencies_hz {
            Some(axis) => covering_cells(axis, proven_bounds(input.scope, input.frequency_hz)?)?,
            None => 0..1,
        };
        let columns = self.frequencies_hz.map_or(1, cell_count);
        let mut cells = rows.flat_map(|row| cols.clone().map(move |col| row * columns + col));
        let cell = |index: usize| proven_bounds(input.scope, self.cells[index].as_ref());
        let first = cell(cells.next()?)?;
        cells.try_fold(first, |bounds, index| {
            let value = cell(index)?;
            Some(CapacitorBounds {
                min: bounds.min.min(value.min),
                max: bounds.max.max(value.max),
            })
        })
    }
}

fn cell_count(axis: &[f64]) -> usize {
    axis.len().saturating_sub(1).max(1)
}

fn covering_cells(axis: &[f64], bounds: CapacitorBounds) -> Option<Range<usize>> {
    let last = *axis.last()?;
    if bounds.min < axis[0] || bounds.max > last {
        return None;
    }
    let index = |x| {
        axis.partition_point(|&breakpoint| breakpoint <= x)
            .saturating_sub(1)
            .min(cell_count(axis) - 1)
    };
    Some(index(bounds.min)..index(bounds.max) + 1)
}

struct Observation {
    voltage: Option<CapacitorBounds>,
    current: Option<CapacitorBounds>,
    voltage_peak_sample: Option<Time>,
}

fn observation(input: &CapacitorStressInput<'_>) -> Result<Observation, ProcessError> {
    let mut result = Observation {
        voltage: None,
        current: None,
        voltage_peak_sample: None,
    };
    let Some(series) = input.series else {
        return Ok(result);
    };
    let (Ok(first), Ok(last)) = (
        series.time.binary_search(&input.window.start),
        series.time.binary_search(&input.window.end),
    ) else {
        return Ok(result);
    };
    let time = &series.time[first..=last];
    let [positive, negative] = input.terminals;
    let probe = Probe::Voltage(positive.clone(), negative.clone());
    if let Some(values) = series.values.get(&probe) {
        let want = |x: f64| match input.voltage_sense {
            CapacitorVoltageSense::Forward => x.max(0.0),
            CapacitorVoltageSense::Absolute => x.abs(),
        };
        let mut peak = want(values[first]);
        let mut at = time[0];
        for (&t, &v) in time.iter().zip(&values[first..=last]) {
            if want(v) > peak {
                peak = want(v);
                at = t;
            }
        }
        result.voltage = observation_bounds(
            CapacitorBounds {
                min: peak,
                max: peak,
            },
            proven_bounds(input.scope, input.voltage_error_v),
        )?;
        result.voltage_peak_sample = Some(at);
    }
    if let Some((values, error)) = series
        .values
        .get(&Probe::Current(input.branch.clone()))
        .zip(proven_bounds(input.scope, input.current_error_a))
    {
        let rms = rms(time, &values[first..=last])?;
        result.current = observation_bounds(rms, Some(error))?;
    }
    Ok(result)
}

fn rms(time: &[Time], values: &[f64]) -> Result<CapacitorBounds, ProcessError> {
    let scale = values.iter().fold(0.0_f64, |peak, i| peak.max(i.abs()));
    let zero = CapacitorBounds { min: 0.0, max: 0.0 };
    if scale == 0.0 {
        return Ok(zero);
    }
    let duration = (time[time.len() - 1].0 - time[0].0) as f64;
    let mut sum = zero;
    for (t, i) in time.windows(2).zip(values.windows(2)) {
        let a = i[0] / scale;
        let b = i[1] / scale;
        let a = outward(a, a)?;
        let b = outward(b, b)?;
        // Средний квадрат линейного отрезка (a² + b² + (a+b)²)/6.
        // Все операции округляются наружу, в том числе перевод тактов в f64.
        let joined = square(outward(a.min + b.min, a.max + b.max)?)?;
        let a = square(a)?;
        let b = square(b)?;
        let endpoints = nonnegative(a.min + b.min, a.max + b.max)?;
        let numerator = nonnegative(endpoints.min + joined.min, endpoints.max + joined.max)?;
        let average = nonnegative(numerator.min / 6.0, numerator.max / 6.0)?;
        let dt = (t[1].0 - t[0].0) as f64;
        let weight = nonnegative(
            dt.next_down() / duration.next_up(),
            dt.next_up() / duration.next_down(),
        )?;
        let contribution = nonnegative(weight.min * average.min, weight.max * average.max)?;
        sum = nonnegative(sum.min + contribution.min, sum.max + contribution.max)?;
    }
    // Средний квадрат нормированного ряда математически не выше единицы.
    let root = nonnegative(sum.min.sqrt(), sum.max.min(1.0).sqrt())?;
    nonnegative(scale * root.min, scale * root.max.min(1.0))
}

fn square(value: CapacitorBounds) -> Result<CapacitorBounds, ProcessError> {
    let near_zero = value.min.max(0.0).max(-value.max);
    let far_from_zero = value.min.abs().max(value.max.abs());
    nonnegative(near_zero * near_zero, far_from_zero * far_from_zero)
}

fn observation_bounds(
    value: CapacitorBounds,
    error: Option<CapacitorBounds>,
) -> Result<Option<CapacitorBounds>, ProcessError> {
    let Some(error) = error else {
        return Ok(None);
    };
    // Неравенство треугольника для максимума и нормы RMS с абсолютной ошибкой ряда.
    nonnegative(value.min - error.max, value.max + error.max).map(Some)
}

fn nonnegative(min: f64, max: f64) -> Result<CapacitorBounds, ProcessError> {
    let mut bounds = outward(min, max)?;
    bounds.min = bounds.min.max(0.0);
    Ok(bounds)
}

fn outward(min: f64, max: f64) -> Result<CapacitorBounds, ProcessError> {
    let (min, max) = (min.next_down(), max.next_up());
    if !min.is_finite() || !max.is_finite() {
        return Err(ProcessError::Execution(
            "переполнение расчёта нагрузки конденсатора".into(),
        ));
    }
    Ok(CapacitorBounds { min, max })
}

fn product(
    a: Option<CapacitorBounds>,
    b: Option<CapacitorBounds>,
) -> Result<Option<CapacitorBounds>, ProcessError> {
    a.zip(b)
        .map(|(a, b)| nonnegative(a.min * b.min, a.max * b.max))
        .transpose()
}

fn sum(
    a: Option<CapacitorBounds>,
    b: Option<CapacitorBounds>,
) -> Result<Option<CapacitorBounds>, ProcessError> {
    a.zip(b)
        .map(|(a, b)| nonnegative(a.min + b.min, a.max + b.max))
        .transpose()
}

fn analyzed<T>(value: Option<T>, reason: &str) -> Analysis<T> {
    match value {
        Some(value) => Analysis::Analyzed {
            value,
            scope: WINDOW_SCOPE.into(),
        },
        None => Analysis::NotAnalyzed {
            reason: reason.into(),
        },
    }
}

fn margin(
    observed: Option<CapacitorBounds>,
    limit: Option<CapacitorBounds>,
    missing_observed: &str,
    missing_limit: &str,
) -> Result<Analysis<CapacitorMargin>, ProcessError> {
    let value = observed
        .zip(limit)
        .map(|(observed, limit)| {
            let margin = outward(limit.min - observed.max, limit.max - observed.min)?;
            Ok(CapacitorMargin {
                observed,
                limit,
                margin,
            })
        })
        .transpose()?;
    let reason = match (observed, limit) {
        (None, None) => format!("{missing_observed}, {missing_limit}"),
        (None, _) => missing_observed.into(),
        _ => missing_limit.into(),
    };
    Ok(analyzed(value, &reason))
}

fn summarize(e: &CapacitorStressEvidence) -> (Status, String) {
    let mut violations = Vec::new();
    let mut gaps = Vec::new();
    for (label, metric) in [
        ("пиковое напряжение", &e.voltage_v),
        ("RMS тока", &e.ripple_a),
        ("температура", &e.hotspot_k),
    ] {
        match metric {
            Analysis::Analyzed { value, .. } if value.margin.max < 0.0 => violations.push(label),
            Analysis::Analyzed { value, .. } if value.margin.min > 0.0 => {}
            Analysis::Analyzed { .. } => {
                gaps.push(format!("{label} в границах неопределённости предела"))
            }
            Analysis::NotAnalyzed { reason } => gaps.push(reason.clone()),
        }
    }
    if let Analysis::NotAnalyzed { reason } = &e.loss_w {
        gaps.push(reason.clone());
    }
    if !violations.is_empty() {
        (
            Status::Fail,
            format!("превышен предел ({})", violations.join(", ")),
        )
    } else if !gaps.is_empty() {
        (Status::NotAnalyzed, gaps.join(". "))
    } else {
        (
            Status::Pass,
            "напряжение, пульсации и температура в пределах для этой ветви и окна".into(),
        )
    }
}
