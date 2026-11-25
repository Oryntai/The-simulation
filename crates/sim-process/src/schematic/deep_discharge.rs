//! Переразряд отдельных ячеек последовательной цепочки в заданном окне.

use std::collections::BTreeSet;

use sim_signal::Time;
use sim_world::Provenance;

use crate::{Analysis, Interval, ProcessError, SourcedRange};

use super::sim_circuit::battery::{CellParams, Table};
use super::{
    Finding, Mode, Scope, Section, Status, identifier, invalid, ns_bound, open, supports_source,
    validate_source,
};

pub const DEEP_DISCHARGE_CODE: &str = "deep_discharge";
const WINDOW_SCOPE: &str = "ячейка, ветвь и окно указанного режима";
const SECONDS_PER_HOUR: f64 = 3600.0; // По определению часа.
const ABSOLUTE_ZERO_C: f64 = -273.15; // Ноль шкалы Кельвина в градусах Цельсия.

#[derive(Clone, Copy, Debug)]
pub struct DeepDischargeModel<'a> {
    pub params: &'a CellParams,
    /// Подтверждает применимость используемых таблиц к этой ячейке, температуре,
    /// току, состоянию и условиям опыта, включая интерполяцию между узлами.
    pub applicability: &'a Provenance,
    /// Абсолютная граница ошибки напряжения на всём окне, включая ошибку модели
    /// и вычислений. Оценочные участки кривых не подтверждают применимость.
    pub voltage_error_v: Option<&'a SourcedRange>,
}

#[derive(Clone, Copy, Debug)]
pub struct DeepDischargeCell<'a> {
    pub id: &'a str,
    pub model: Option<DeepDischargeModel<'a>>,
    /// Индивидуальная ёмкость при опорных 25 °C. Номинал модели её не заменяет.
    pub capacity_ah: Option<&'a SourcedRange>,
    /// Индивидуальный начальный заряд в долях этой ёмкости при 25 °C.
    pub initial_soc: Option<&'a SourcedRange>,
    /// Постоянная в окне температура ячейки или диапазон её возможных значений.
    pub temperature_c: Option<&'a SourcedRange>,
    /// Напряжения в звеньях RC в порядке модели. Отдых задаётся явными нулями.
    pub initial_polarization_v: Option<&'a [SourcedRange]>,
    /// Эквивалентный внутренний саморазряд во всём окне. Уменьшает заряд
    /// и не проходит через клеммы, R0 и поляризационные звенья.
    pub self_discharge_a: Option<&'a SourcedRange>,
    /// Применимый нижний предел на клеммах этой ячейки.
    pub min_voltage_v: Option<&'a SourcedRange>,
}

/// Огибающие токов на всём шаге в направлении разряда, включая пики между
/// отсчётами. Значение в отдельном отсчёте такой огибающей не задаёт.
/// На стыке шагов учитываются обе стороны скачка тока.
#[derive(Clone, Copy, Debug)]
pub struct DeepDischargeSegment<'a> {
    pub window: Interval,
    pub load_a: Option<&'a SourcedRange>,
    /// Ток питания защиты через проверяемую цепочку, в том числе после отключения.
    pub protection_a: Option<&'a SourcedRange>,
}

#[derive(Clone, Copy, Debug)]
pub struct DeepDischargeCutoff<'a> {
    pub detected_at: Time,
    pub disconnected_at: Time,
    /// Применимый интервал задержки от обнаружения до физического размыкания.
    /// Положительный результат относится к наблюдаемой задержке этого опыта.
    pub delay_s: Option<&'a SourcedRange>,
    /// Подтверждает наблюдение обоих событий и привязку к этой ветви.
    /// Состояние нагрузки после события дополнительно сверяется по току.
    pub source: &'a Provenance,
}

#[derive(Clone, Copy, Debug)]
pub struct DeepDischargeInput<'a> {
    pub scope: Scope,
    pub mode: &'a Mode,
    pub branch: &'a Section,
    /// Конечное окно опыта, включающее требуемый период после отсечки.
    pub window: Interval,
    pub cells: Option<&'a [DeepDischargeCell<'a>]>,
    /// Подтверждает полный состав последовательной цепочки и общий ток через
    /// каждую ячейку. Неохваченные отводы и неизвестное деление тока оставляют анализ открытым.
    pub series_applicability: Option<&'a Provenance>,
    pub segments: Option<&'a [DeepDischargeSegment<'a>]>,
    pub cutoff: Option<DeepDischargeCutoff<'a>>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DeepDischargeBounds {
    pub min: f64,
    pub max: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DeepDischargeCellEvidence {
    pub id: String,
    /// Нижняя граница охватывает всё окно. Верхняя подтверждена на границе шага.
    pub minimum_voltage_v: Analysis<DeepDischargeBounds>,
    /// Минимальное напряжение минус применимый предел.
    pub margin_v: Analysis<DeepDischargeBounds>,
    /// Достоверное нарушение, в том числе при незакрытом полном окне.
    pub violation: Option<DeepDischargeViolation>,
    /// Напряжение модели на правой границе последнего шага. Модель ограничивает
    /// эффективный SOC нулём при исчерпании заряда, поэтому такая точка
    /// после исчерпания описывает только численную модель.
    pub final_model_voltage_v: Analysis<DeepDischargeBounds>,
    /// Отданный в этом окне заряд, включая саморазряд, без начального дефицита SOC.
    pub removed_charge_ah: Analysis<DeepDischargeBounds>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DeepDischargeViolation {
    pub time: Time,
    pub voltage_v: DeepDischargeBounds,
    pub limit_v: DeepDischargeBounds,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DeepDischargeCutoffEvidence {
    pub detected_at: Time,
    pub disconnected_at: Time,
    pub observed_delay: Time,
    pub allowed_delay_s: DeepDischargeBounds,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DeepDischargeEvidence {
    pub code: &'static str,
    pub scope: Scope,
    pub mode: Mode,
    pub branch: Section,
    pub window: Interval,
    pub cutoff: Analysis<DeepDischargeCutoffEvidence>,
    pub cells: Vec<DeepDischargeCellEvidence>,
    /// Ячейка с наименьшей нижней границей среди вычисленных запасов.
    pub limiting_cell: Option<String>,
}

pub fn assess_deep_discharge(
    input: &DeepDischargeInput<'_>,
) -> Result<Finding<DeepDischargeEvidence>, ProcessError> {
    let subject = validate(input)?;
    let segments = load_trace(input)?;
    let cutoff = cutoff_evidence(input, &segments)?;
    let series_confirmed = confirmed(input.scope, input.series_applicability);
    let cells = input
        .cells
        .unwrap_or_default()
        .iter()
        .map(|cell| {
            if series_confirmed {
                assess_cell(input, cell, &segments)
            } else {
                Ok(open_cell(
                    cell.id,
                    "нет подтверждения состава или тока цепочки",
                ))
            }
        })
        .collect::<Result<Vec<_>, _>>()?;
    let limiting_cell = cells
        .iter()
        .filter_map(|cell| value(&cell.margin_v).map(|margin| (cell, margin.min)))
        .min_by(|(_, a), (_, b)| a.total_cmp(b))
        .map(|(cell, _)| cell.id.clone());
    let (status, reason) = if let Some(cell) = cells.iter().find(|cell| cell.violation.is_some()) {
        (
            Status::Fail,
            format!("ячейка {} опустилась ниже допустимого напряжения", cell.id),
        )
    } else if cells.is_empty() {
        (Status::NotAnalyzed, "не заданы отдельные ячейки".into())
    } else if let Some(reason) = cells.iter().find_map(|cell| match &cell.margin_v {
        Analysis::NotAnalyzed { reason } => Some(format!("ячейка {}: {reason}", cell.id)),
        Analysis::Analyzed { value, .. } if value.min < 0.0 => Some(format!(
            "огибающая ячейки {} не исключает провал между отсчётами или в пределах погрешности",
            cell.id
        )),
        _ => None,
    }) {
        (Status::NotAnalyzed, reason)
    } else if let Analysis::NotAnalyzed { reason } = &cutoff {
        (Status::NotAnalyzed, reason.clone())
    } else {
        (
            Status::Pass,
            "подтверждены отсечка и безопасное напряжение каждой ячейки во всём окне".into(),
        )
    };
    Ok(Finding {
        subject,
        status,
        reason,
        evidence: DeepDischargeEvidence {
            code: DEEP_DISCHARGE_CODE,
            scope: input.scope,
            mode: input.mode.clone(),
            branch: input.branch.clone(),
            window: input.window,
            cutoff,
            cells,
            limiting_cell,
        },
    })
}

type Bounds = DeepDischargeBounds;

impl Bounds {
    const ZERO: Self = Self { min: 0.0, max: 0.0 };

    fn new(min: f64, max: f64) -> Result<Self, ProcessError> {
        if !min.is_finite() || !max.is_finite() || min > max {
            return Err(invalid("некорректная численная огибающая переразряда"));
        }
        Ok(Self { min, max })
    }

    fn add(self, other: Self) -> Result<Self, ProcessError> {
        Self::new(self.min + other.min, self.max + other.max)
    }

    fn sub(self, other: Self) -> Result<Self, ProcessError> {
        Self::new(self.min - other.max, self.max - other.min)
    }

    fn hull(self, other: Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    fn product_nonnegative(self, other: Self) -> Result<Self, ProcessError> {
        Self::new(self.min * other.min, self.max * other.max)
    }

    fn clipped_soc(self) -> Self {
        Self {
            min: self.min.clamp(0.0, 1.0),
            max: self.max.clamp(0.0, 1.0),
        }
    }
}

impl From<&SourcedRange> for DeepDischargeBounds {
    fn from(range: &SourcedRange) -> Self {
        Self {
            min: range.min,
            max: range.max,
        }
    }
}

fn value<T>(analysis: &Analysis<T>) -> Option<&T> {
    match analysis {
        Analysis::Analyzed { value, .. } => Some(value),
        Analysis::NotAnalyzed { .. } => None,
    }
}

fn analyzed<T>(value: T) -> Analysis<T> {
    Analysis::Analyzed {
        value,
        scope: WINDOW_SCOPE.into(),
    }
}

fn confirmed(scope: Scope, source: Option<&Provenance>) -> bool {
    source.is_some_and(|source| supports_source(scope, source))
}

fn proven(scope: Scope, range: Option<&SourcedRange>) -> Option<Bounds> {
    range
        .filter(|range| supports_source(scope, &range.provenance))
        .map(Bounds::from)
}

fn validate_range(
    range: Option<&SourcedRange>,
    valid: impl Fn(f64) -> bool,
) -> Result<(), ProcessError> {
    if let Some(range) = range {
        range.validate()?;
        if !valid(range.min) || !valid(range.max) {
            return Err(invalid("вход переразряда вне физической области"));
        }
    }
    Ok(())
}

fn validate(input: &DeepDischargeInput<'_>) -> Result<String, ProcessError> {
    let subject = match input.branch {
        Section::Element { component, element } if identifier(component) && identifier(element) => {
            format!("{component}/{element}")
        }
        Section::Connection(id) if identifier(id) => id.clone(),
        _ => return Err(invalid("недопустимая ветвь проверки переразряда")),
    };
    if input.window.start >= input.window.end {
        return Err(invalid("пустое или обратное окно проверки переразряда"));
    }
    if let Some(source) = input.series_applicability {
        validate_source(source)?;
    }
    if let Some(cutoff) = input.cutoff {
        validate_source(cutoff.source)?;
        validate_range(cutoff.delay_s, |x| x >= 0.0 && Time::from_secs(x).is_some())?;
        if cutoff.detected_at > cutoff.disconnected_at {
            return Err(invalid("размыкание раньше обнаружения разряда"));
        }
    }
    let mut last_end = input.window.start;
    for segment in input.segments.unwrap_or_default() {
        if segment.window.start < last_end
            || segment.window.start >= segment.window.end
            || segment.window.end > input.window.end
        {
            return Err(invalid("перекрывающиеся или недопустимые участки тока"));
        }
        validate_range(segment.load_a, |x| x >= 0.0)?;
        validate_range(segment.protection_a, |x| x >= 0.0)?;
        last_end = segment.window.end;
    }
    let mut ids = BTreeSet::new();
    for cell in input.cells.unwrap_or_default() {
        if !identifier(cell.id) || !ids.insert(cell.id) {
            return Err(invalid("пустой или повторяющийся адрес ячейки"));
        }
        validate_range(cell.capacity_ah, |x| x > 0.0)?;
        validate_range(cell.initial_soc, |x| (0.0..=1.0).contains(&x))?;
        validate_range(cell.temperature_c, |x| x > ABSOLUTE_ZERO_C)?;
        validate_range(cell.self_discharge_a, |x| x >= 0.0)?;
        validate_range(cell.min_voltage_v, |x| x > 0.0)?;
        if let Some(model) = cell.model {
            validate_model(model)?;
            if cell
                .initial_polarization_v
                .is_some_and(|initial| initial.len() != model.params.rc.len())
            {
                return Err(invalid(
                    "число начальных напряжений не совпадает с числом звеньев RC",
                ));
            }
        }
        for initial in cell.initial_polarization_v.unwrap_or_default() {
            initial.validate()?;
        }
    }
    Ok(subject)
}

fn validate_model(model: DeepDischargeModel<'_>) -> Result<(), ProcessError> {
    validate_source(model.applicability)?;
    validate_range(model.voltage_error_v, |x| x >= 0.0)?;
    let p = model.params;
    // Проверяются и метаданные CellParams, которые не участвуют в расчёте напряжения.
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
        return Err(invalid("недопустимые параметры модели ячейки"));
    }
    let soc = |z| (0.0..=1.0).contains(&z);
    validate_table(&p.ocv, soc, |v| v > 0.0)?;
    validate_table(&p.entropic, soc, |_| true)?;
    validate_table(&p.r0, soc, |r| r >= 0.0)?;
    validate_table(&p.capacity_factor, |t| t > ABSOLUTE_ZERO_C, |f| f > 0.0)?;
    for pole in &p.rc {
        if !pole.c.is_finite() || pole.c <= 0.0 {
            return Err(invalid("недопустимая ёмкость звена RC ячейки"));
        }
        validate_table(&pole.r, soc, |r| r > 0.0)?;
    }
    Ok(())
}

fn validate_table(
    table: &Table,
    valid_axis: impl Fn(f64) -> bool,
    valid_value: impl Fn(f64) -> bool,
) -> Result<(), ProcessError> {
    if table
        .0
        .iter()
        .any(|&(x, y)| !x.is_finite() || !y.is_finite() || !valid_axis(x) || !valid_value(y))
        || table.0.windows(2).any(|pair| pair[0].0 >= pair[1].0)
    {
        return Err(invalid("недопустимая таблица модели ячейки"));
    }
    Ok(())
}

#[derive(Clone, Copy)]
struct LoadSegment {
    window: Interval,
    load: Bounds,
    terminal_current: Bounds,
}

/// Связный префикс с известным током, пригодный для поиска подтверждённого
/// нарушения даже при пропуске позже в том же окне.
fn load_trace(input: &DeepDischargeInput<'_>) -> Result<Vec<LoadSegment>, ProcessError> {
    let mut result = Vec::new();
    let mut last_end = input.window.start;
    for segment in input.segments.unwrap_or_default() {
        let (Some(load), Some(idle)) = (
            proven(input.scope, segment.load_a),
            proven(input.scope, segment.protection_a),
        ) else {
            break;
        };
        if segment.window.start != last_end {
            break;
        }
        result.push(LoadSegment {
            window: segment.window,
            load,
            terminal_current: load.add(idle)?,
        });
        last_end = segment.window.end;
    }
    Ok(result)
}

fn cutoff_evidence(
    input: &DeepDischargeInput<'_>,
    segments: &[LoadSegment],
) -> Result<Analysis<DeepDischargeCutoffEvidence>, ProcessError> {
    let Some(cutoff) = input.cutoff else {
        return Ok(open("нет подтверждения отсечки нагрузки в заданном окне"));
    };
    let Some(delay) = proven(input.scope, cutoff.delay_s) else {
        return Ok(open("нет применимого интервала задержки BMS"));
    };
    let off = cutoff.disconnected_at;
    if !supports_source(input.scope, cutoff.source)
        || cutoff.detected_at < input.window.start
        || off >= input.window.end
    {
        return Ok(open(
            "нет подтверждённых событий отсечки и окна после размыкания",
        ));
    }
    let elapsed = off.0 - cutoff.detected_at.0;
    if elapsed < ns_bound(delay.min, true)? || elapsed > ns_bound(delay.max, false)? {
        return Ok(open(
            "наблюдаемая задержка не укладывается в заявленный интервал BMS",
        ));
    }
    if !segments
        .iter()
        .any(|step| step.window.end == off && step.load.min > 0.0)
        || segments
            .iter()
            .filter(|step| step.window.start >= off)
            .any(|step| step.load.max != 0.0)
        || !segments
            .last()
            .is_some_and(|step| step.window.end == input.window.end)
    {
        return Ok(open(
            "ряд тока не подтверждает снятие нагрузки и её отсутствие после отсечки",
        ));
    }
    Ok(analyzed(DeepDischargeCutoffEvidence {
        detected_at: cutoff.detected_at,
        disconnected_at: off,
        observed_delay: Time(elapsed),
        allowed_delay_s: delay,
    }))
}

fn open_cell(id: &str, reason: &str) -> DeepDischargeCellEvidence {
    DeepDischargeCellEvidence {
        id: id.into(),
        minimum_voltage_v: open(reason),
        margin_v: open(reason),
        violation: None,
        final_model_voltage_v: open(reason),
        removed_charge_ah: open(reason),
    }
}

struct CellModel<'a> {
    params: &'a CellParams,
    capacity: Bounds,
    initial_soc: Bounds,
    temperature: Bounds,
    capacity_factor: Bounds,
    self_discharge: Bounds,
    error: Bounds,
}

impl CellModel<'_> {
    fn soc(&self, removed_ah: Bounds) -> Result<Bounds, ProcessError> {
        // Та же шкала, что в Cell::effective_soc, с индивидуальной ёмкостью.
        Bounds::new(
            1.0 - ((1.0 - self.initial_soc.min) + removed_ah.max / self.capacity.min)
                / self.capacity_factor.min,
            1.0 - ((1.0 - self.initial_soc.max) + removed_ah.min / self.capacity.max)
                / self.capacity_factor.max,
        )
    }

    fn electrical_tables(&self) -> impl Iterator<Item = &Table> {
        [&self.params.ocv, &self.params.entropic, &self.params.r0]
            .into_iter()
            .chain(self.params.rc.iter().map(|pole| &pole.r))
    }

    fn voltage(
        &self,
        soc: Bounds,
        current: Bounds,
        polarization: Bounds,
    ) -> Result<Bounds, ProcessError> {
        // OCV и энтропийная поправка линейны между узлами обеих таблиц
        // при фиксированной T. Для фиксированного SOC результирующая OCV
        // линейна по T. Масштаб сопротивления монотонен по T.
        let p = self.params;
        let ocv = extrema([&p.ocv, &p.entropic], soc, self.temperature, |z, t| {
            p.ocv_at(z, t)
        })?;
        let r0 = extrema([&p.r0], soc, self.temperature, |z, t| p.r0_at(z, t))?;
        ocv.sub(current.product_nonnegative(r0)?)?
            .sub(polarization)?
            .add(self.error)
    }
}

fn covers(table: &Table, axis: Bounds) -> bool {
    table.0.first().is_some_and(|p| p.0 <= axis.min)
        && table.0.last().is_some_and(|p| axis.max <= p.0)
}

fn extrema<'a>(
    tables: impl IntoIterator<Item = &'a Table>,
    axis: Bounds,
    temperature: Bounds,
    eval: impl Fn(f64, f64) -> f64,
) -> Result<Bounds, ProcessError> {
    let first = eval(axis.min, temperature.min);
    let mut result = Bounds::new(first, first)?;
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
        for temp in [temperature.min, temperature.max] {
            let y = eval(x, temp);
            result = result.hull(Bounds::new(y, y)?);
        }
    }
    Ok(result)
}

fn assess_cell(
    input: &DeepDischargeInput<'_>,
    cell: &DeepDischargeCell<'_>,
    segments: &[LoadSegment],
) -> Result<DeepDischargeCellEvidence, ProcessError> {
    let proven = |r| proven(input.scope, r);
    let Some(model) = cell
        .model
        .filter(|m| supports_source(input.scope, m.applicability))
    else {
        return Ok(open_cell(cell.id, "нет применимой модели ячейки"));
    };
    let p = model.params;
    let (
        Some(capacity),
        Some(initial_soc),
        Some(temperature),
        Some(self_discharge),
        Some(error),
        Some(initial),
    ) = (
        proven(cell.capacity_ah),
        proven(cell.initial_soc),
        proven(cell.temperature_c),
        proven(cell.self_discharge_a),
        proven(model.voltage_error_v),
        cell.initial_polarization_v,
    )
    else {
        return Ok(open_cell(
            cell.id,
            "нет подтверждённой ёмкости, SOC, температуры, саморазряда, поляризации или точности модели",
        ));
    };
    let Some(mut polarization) = initial
        .iter()
        .map(|r| proven(Some(r)))
        .collect::<Option<Vec<_>>>()
    else {
        return Ok(open_cell(
            cell.id,
            "нет подтверждённых начальных напряжений звеньев RC",
        ));
    };
    if !covers(&p.capacity_factor, temperature)
        || temperature.min < p.discharge_temp_c.0
        || temperature.max > p.discharge_temp_c.1
    {
        return Ok(open_cell(
            cell.id,
            "температура вне применимой области модели ячейки",
        ));
    }
    let capacity_factor = extrema([&p.capacity_factor], temperature, temperature, |t, _| {
        p.capacity_factor.eval(t)
    })?;
    let model = CellModel {
        params: p,
        capacity,
        initial_soc,
        temperature,
        capacity_factor,
        self_discharge,
        error: Bounds::new(-error.max, error.max)?,
    };
    let limit = proven(cell.min_voltage_v);
    let mut removed = Bounds::ZERO;
    let mut minimum: Option<Bounds> = None;
    let mut final_voltage = None;
    let mut violation = None;
    let mut exhausted = false;
    let mut voltage_covered = true;
    for step in segments {
        let dt = Time(step.window.end.0 - step.window.start.0).as_secs();
        let elapsed = dt / SECONDS_PER_HOUR;
        let next_removed = removed.add(
            step.terminal_current
                .add(model.self_discharge)?
                .product_nonnegative(Bounds::new(elapsed, elapsed)?)?,
        )?;
        let initial_soc = model.soc(removed)?;
        let final_soc = model.soc(next_removed)?;
        removed = next_removed;
        if !voltage_covered {
            continue;
        }
        let span = initial_soc.hull(final_soc).clipped_soc();
        // Перед потерей покрытия сохраняем достоверный результат на левой
        // границе, включая скачок I·R0 от включения нового участка.
        let start_soc = initial_soc.clipped_soc();
        let start =
            if initial_soc.min >= 0.0 && model.electrical_tables().all(|t| covers(t, start_soc)) {
                let start = model.voltage(start_soc, step.terminal_current, sum(&polarization)?)?;
                record_violation(&mut violation, limit, start, step.window.start);
                Some(start)
            } else {
                None
            };
        if !model.electrical_tables().all(|table| covers(table, span)) {
            voltage_covered = false;
            continue;
        }
        let mut during = Bounds::ZERO;
        for (k, state) in polarization.iter_mut().enumerate() {
            let r = extrema([&p.rc[k].r], span, temperature, |z, t| p.rc_r_at(k, z, t))?;
            let next = rc_end(*state, step.terminal_current, r, p.rc[k].c, dt)?;
            during = during.add(state.hull(next))?;
            *state = next;
        }
        let end = model.voltage(
            final_soc.clipped_soc(),
            step.terminal_current,
            sum(&polarization)?,
        )?;
        final_voltage = Some(end);
        if final_soc.min < 0.0 {
            exhausted = true;
            continue;
        }
        record_violation(&mut violation, limit, end, step.window.end);
        let lower = model.voltage(span, step.terminal_current, during)?.min;
        let upper = start.map_or(end.max, |start| start.max.min(end.max));
        minimum = Some(match minimum {
            Some(previous) => Bounds::new(previous.min.min(lower), previous.max.min(upper))?,
            None => Bounds::new(lower, upper)?,
        });
    }
    let complete = segments
        .last()
        .is_some_and(|step| step.window.end == input.window.end);
    let gap = if !complete {
        Some("нет подтверждённой огибающей тока и тока защиты во всём окне")
    } else if !voltage_covered {
        Some("SOC вышел за область применимых таблиц")
    } else if exhausted {
        Some("доступный заряд мог исчерпаться, напряжение после исчерпания не доказано")
    } else {
        None
    };
    let minimum_voltage_v = match (gap, minimum) {
        (None, Some(minimum)) => analyzed(minimum),
        (Some(reason), _) => open(reason),
        _ => open("не рассчитана огибающая напряжения"),
    };
    let margin_v = match (&minimum_voltage_v, limit) {
        (Analysis::Analyzed { value, .. }, Some(limit)) => analyzed(value.sub(limit)?),
        (Analysis::NotAnalyzed { reason }, _) => open(reason.clone()),
        _ => open("нет применимого нижнего предела напряжения ячейки"),
    };
    Ok(DeepDischargeCellEvidence {
        id: cell.id.into(),
        minimum_voltage_v,
        margin_v,
        violation,
        final_model_voltage_v: match final_voltage {
            Some(v)
                if complete
                    && voltage_covered
                    && (!exhausted || input.scope == Scope::Reference) =>
            {
                if exhausted {
                    Analysis::Analyzed {
                        value: v,
                        scope: "граничная точка численной модели при SOC = 0".into(),
                    }
                } else {
                    analyzed(v)
                }
            }
            _ => open("нет расчёта напряжения на правой границе окна"),
        },
        removed_charge_ah: if complete {
            analyzed(removed)
        } else {
            open("ряд тока не охватывает окно")
        },
    })
}

fn record_violation(
    event: &mut Option<DeepDischargeViolation>,
    limit: Option<Bounds>,
    voltage: Bounds,
    time: Time,
) {
    if event.is_none()
        && let Some(limit) = limit
        && voltage.max < limit.min
    {
        *event = Some(DeepDischargeViolation {
            time,
            voltage_v: voltage,
            limit_v: limit,
        });
    }
}

fn sum(values: &[Bounds]) -> Result<Bounds, ProcessError> {
    values
        .iter()
        .try_fold(Bounds::ZERO, |sum, &pole| sum.add(pole))
}

/// Для Vp' = I/C − Vp/(RC) на неотрицательной полуоси больший R
/// замедляет спад, на отрицательной ускоряет его. Нижнее и верхнее
/// решения сравнения используют соответствующие крайние R по обе
/// стороны нуля и монотонны на всём шаге. Их конечные точки вместе
/// с начальными ограничивают напряжение RC между тактами.
fn rc_end(v: Bounds, i: Bounds, r: Bounds, c: f64, dt: f64) -> Result<Bounds, ProcessError> {
    Bounds::new(
        rc_comparison(v.min, i.min, r.min, r.max, c, dt)?,
        rc_comparison(v.max, i.max, r.max, r.min, c, dt)?,
    )
}

fn rc_comparison(
    start: f64,
    current: f64,
    r_positive: f64,
    r_negative: f64,
    c: f64,
    dt: f64,
) -> Result<f64, ProcessError> {
    let evolve = |v: f64, r: f64, duration: f64| {
        let tau = r * c;
        let equilibrium = r * current;
        if !tau.is_finite() || tau <= 0.0 || !equilibrium.is_finite() {
            return Err(invalid(
                "недопустимая постоянная времени или поляризация ячейки",
            ));
        }
        let x = -duration / tau;
        Ok(v * x.exp() - equilibrium * x.exp_m1())
    };
    if start >= 0.0 {
        return evolve(start, r_positive, dt);
    }
    let end = evolve(start, r_negative, dt)?;
    if end <= 0.0 {
        return Ok(end);
    }
    // Точное время достижения Vp = 0 в ветви с постоянными I, R и C.
    let crossing = r_negative * c * (-start / (current * r_negative)).ln_1p();
    if !crossing.is_finite() {
        return Err(invalid(
            "не удалось ограничить пересечение нуля поляризации",
        ));
    }
    evolve(0.0, r_positive, (dt - crossing).max(0.0))
}
