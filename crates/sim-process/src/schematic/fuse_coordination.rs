//! Согласование плавкого предохранителя с проводом и силовым ключом при одном отказе.

use sim_signal::Time;
use sim_world::Provenance;

use crate::schematic::{Finding, Mode, Scope, Section, Status};
use crate::{Analysis, Interval, ProcessError, SourcedRange};

pub const FUSE_COORDINATION_CODE: &str = "fuse_coordination";

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FuseBounds {
    pub min: f64,
    pub max: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FuseWaveform<'a> {
    /// Прямоугольная ступень постоянного тока, для SOA также постоянное напряжение.
    DcStep,
    /// Идентификатор охарактеризованного профиля. Источник должен покрывать его форму,
    /// полярность, начальное состояние и разброс, а для ключа также дугу и выбросы.
    Characterized(&'a str),
}

#[derive(Clone, Copy, Debug)]
pub struct FuseDcConditions<'a> {
    pub temperature_k: FuseBounds,
    /// L/R отключаемой цепи в области действия источника, в секундах.
    pub time_constant_s: FuseBounds,
    pub waveform: FuseWaveform<'a>,
}

#[derive(Clone, Copy, Debug)]
pub struct FuseInterruptRating<'a> {
    pub conditions: FuseDcConditions<'a>,
    /// Паспортная пара тока и напряжения для одного испытательного контура.
    pub max_prospective_current_a: &'a SourcedRange,
    pub max_recovery_voltage_v: &'a SourcedRange,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FuseTimeStage {
    TotalClearing,
    PreArcing,
}

#[derive(Clone, Copy, Debug)]
pub struct FuseTccBand<'a> {
    /// Замкнутый интервал ожидаемого тока КЗ, покрытый указанными границами времени.
    /// Строка паспорта в одной точке покрывает только эту точку.
    pub prospective_current_a: FuseBounds,
    pub time_s: Option<&'a SourcedRange>,
}

#[derive(Clone, Copy, Debug)]
pub struct FuseTcc<'a> {
    pub conditions: FuseDcConditions<'a>,
    pub recovery_voltage_v: FuseBounds,
    pub stage: FuseTimeStage,
    /// Границы, гарантированные во всей области каждой полосы источником.
    /// Для TotalClearing время включает плавление и гашение дуги.
    pub bands: &'a [FuseTccBand<'a>],
}

#[derive(Clone, Copy, Debug)]
pub struct FuseWireLimit<'a> {
    pub start_temperature_k: FuseBounds,
    /// Границы достижения предела проводника или изоляции по I²t.
    /// Источник покрывает сечение, материал, монтаж, охлаждение и длительность
    /// данной формы импульса. Адиабатическая нижняя оценка сама по себе
    /// не задаёт верхней границы времени реального повреждения.
    pub damage_i2t_a2s: &'a SourcedRange,
}

#[derive(Clone, Copy, Debug)]
pub struct FuseSwitchSoa<'a> {
    pub start_temperature_k: FuseBounds,
    pub current_a: FuseBounds,
    pub voltage_v: FuseBounds,
    pub waveform: FuseWaveform<'a>,
    /// Интервал допустимой длительности импульса во всей области I/V/T.
    /// Источник покрывает полярность, рабочий режим, начальное состояние,
    /// монтаж, охлаждение и накопленное тепло. Типовая кривая сама по себе
    /// не задаёт границ предела. Превышение означает выход из SOA.
    pub damage_time_s: &'a SourcedRange,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct FuseCoordinationRatings<'a> {
    pub interrupt: Option<FuseInterruptRating<'a>>,
    pub tcc: Option<FuseTcc<'a>>,
    pub wire: Option<FuseWireLimit<'a>>,
    pub switch: Option<FuseSwitchSoa<'a>>,
}

#[derive(Clone, Copy, Debug)]
pub struct FuseFaultStep<'a> {
    pub interval: Interval,
    /// Мгновенные границы модуля тока на всём интервале, в том числе между
    /// тактами и при дуге, пока цепь ещё проводит. Отсчёт или средний ток
    /// такой огибающей не задаёт. Если граница неизвестна, передаётся None.
    pub wire_current_a: Option<&'a SourcedRange>,
    pub switch_current_a: Option<&'a SourcedRange>,
    pub switch_voltage_v: Option<&'a SourcedRange>,
}

#[derive(Clone, Copy, Debug)]
pub struct FuseCoordinationInput<'a> {
    pub scope: Scope,
    pub mode: &'a Mode,
    pub fuse: &'a Section,
    pub wire: &'a Section,
    pub switch: &'a Section,
    /// Начало окна совпадает с приложением отказа, времена пределов отсчитываются от него.
    pub window: Interval,
    /// Свидетельство, что отключение этой ветви прекращает повреждающую нагрузку
    /// на проводе и ключе при этом отказе, с учётом альтернативных путей и переходов.
    pub protection_path: Option<&'a Provenance>,
    /// Ограничивает нагрузку, пока предохранитель проводит, включая разброс момента дуги.
    pub trace: &'a [FuseFaultStep<'a>],
    /// Ток КЗ в точке предохранителя без его токоограничения. Его максимум
    /// включает возможный пик между отсчётами.
    pub prospective_current_a: Option<&'a SourcedRange>,
    /// Границы максимального восстанавливающегося DC-напряжения при отключении.
    pub recovery_voltage_v: Option<&'a SourcedRange>,
    pub dc_time_constant_s: Option<&'a SourcedRange>,
    pub fuse_temperature_k: Option<&'a SourcedRange>,
    pub wire_start_temperature_k: Option<&'a SourcedRange>,
    pub switch_start_temperature_k: Option<&'a SourcedRange>,
    pub prospective_waveform: Option<FuseWaveform<'a>>,
    pub switch_waveform: Option<FuseWaveform<'a>>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FuseDamageTime {
    pub earliest_s: f64,
    /// None, если верхняя граница не установлена в доступном окне.
    pub latest_s: Option<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FuseTimeMargin {
    /// Запас «время повреждения − время полного отключения» в секундах.
    pub min: f64,
    pub max: Option<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FuseWireEvidence {
    /// Интеграл огибающей при продолжающемся токе на всём окне опыта.
    pub window_i2t_a2s: FuseBounds,
    pub damage_time_s: FuseDamageTime,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FuseCoordinationEvidence {
    pub code: &'static str,
    pub scope: Scope,
    pub mode: Mode,
    pub fuse: Section,
    pub wire_branch: Section,
    pub switch_branch: Section,
    pub window: Interval,
    pub interrupt_current: Status,
    pub interrupt_voltage: Status,
    pub clearing_time_s: Analysis<FuseBounds>,
    pub wire: Analysis<FuseWireEvidence>,
    pub switch: Analysis<FuseDamageTime>,
    pub wire_margin_s: Option<FuseTimeMargin>,
    pub switch_margin_s: Option<FuseTimeMargin>,
}

/// Проверяет согласование в указанном режиме. Открытые входы и наложение
/// диапазонов оставляют критерий открытым, некорректный ввод возвращает ошибку.
pub fn assess_fuse_coordination(
    input: &FuseCoordinationInput<'_>,
    ratings: &FuseCoordinationRatings<'_>,
) -> Result<Finding<FuseCoordinationEvidence>, ProcessError> {
    validate(input, ratings)?;
    let path = input
        .protection_path
        .is_some_and(|s| supports(input.scope, s));
    let complete_trace = input.trace.first().map(|s| s.interval.start) == Some(input.window.start)
        && input.trace.last().map(|s| s.interval.end) == Some(input.window.end)
        && input
            .trace
            .windows(2)
            .all(|w| w[0].interval.end == w[1].interval.start);
    let horizon = elapsed(input.window.start, input.window.end);
    let (interrupt_current, interrupt_voltage) = interrupt(input, ratings.interrupt);
    let clear = clearing(input, ratings.tcc);
    let wire = if complete_trace {
        wire_damage(input, ratings.wire, horizon)?
    } else {
        None
    };
    let switch = if complete_trace {
        switch_damage(input, ratings.switch, horizon)
    } else {
        None
    };
    let wire_margin_s = wire
        .zip(clear)
        .map(|(wire, clear)| margin(wire.damage_time_s, clear))
        .transpose()?;
    let switch_margin_s = switch
        .zip(clear)
        .map(|(damage, clear)| margin(damage, clear))
        .transpose()?;
    let evidence = FuseCoordinationEvidence {
        code: FUSE_COORDINATION_CODE,
        scope: input.scope,
        mode: input.mode.clone(),
        fuse: input.fuse.clone(),
        wire_branch: input.wire.clone(),
        switch_branch: input.switch.clone(),
        window: input.window,
        interrupt_current,
        interrupt_voltage,
        clearing_time_s: analyzed(
            clear,
            "нет применимой полной TCC с обеими границами времени",
        ),
        wire: analyzed(
            wire,
            "нет непрерывной огибающей тока провода или применимого предела I²t",
        ),
        switch: analyzed(
            switch,
            "нет непрерывной огибающей напряжения и тока ключа или применимой SOA",
        ),
        wire_margin_s,
        switch_margin_s,
    };
    let (status, reason) = summarize(path, clear, horizon, &evidence);
    let subject = match input.fuse {
        Section::Element { component, element } => format!("{component}/{element}"),
        Section::Connection(name) => name.clone(),
    };
    Ok(Finding {
        subject,
        status,
        reason,
        evidence,
    })
}

fn validate(
    input: &FuseCoordinationInput<'_>,
    ratings: &FuseCoordinationRatings<'_>,
) -> Result<(), ProcessError> {
    let name_ok =
        |name: &str| !name.is_empty() && name.trim() == name && !name.chars().any(char::is_control);
    let section_ok = |section: &Section| match section {
        Section::Connection(name) => name_ok(name),
        Section::Element { component, element } => name_ok(component) && name_ok(element),
    };
    if input.window.start >= input.window.end
        || ![input.fuse, input.wire, input.switch]
            .into_iter()
            .all(section_ok)
        || input.fuse == input.wire
        || input.fuse == input.switch
        || input.wire == input.switch
    {
        return Err(invalid("недопустимые ветви или окно согласования защиты"));
    }
    if let Some(source) = input.protection_path {
        let description = match source {
            Provenance::Datasheet(s)
            | Provenance::Measured(s)
            | Provenance::Paper(s)
            | Provenance::Derived(s)
            | Provenance::Estimate(s) => s,
        };
        if !name_ok(description) {
            return Err(invalid("недопустимый источник пути отключения"));
        }
    }
    let nonnegative = |x| x >= 0.0;
    let positive = |x| x > 0.0;
    for value in [
        input.prospective_current_a,
        input.recovery_voltage_v,
        input.dc_time_constant_s,
    ] {
        validate_range(value, nonnegative)?;
    }
    for value in [
        input.fuse_temperature_k,
        input.wire_start_temperature_k,
        input.switch_start_temperature_k,
    ] {
        validate_range(value, positive)?;
    }
    for shape in [input.prospective_waveform, input.switch_waveform]
        .into_iter()
        .flatten()
    {
        validate_waveform(shape, name_ok)?;
    }
    let mut end = input.window.start;
    for step in input.trace {
        if step.interval.start < end
            || step.interval.start >= step.interval.end
            || step.interval.end > input.window.end
        {
            return Err(invalid(
                "недопустимые или пересекающиеся интервалы тока отказа",
            ));
        }
        end = step.interval.end;
        for value in [
            step.wire_current_a,
            step.switch_current_a,
            step.switch_voltage_v,
        ] {
            validate_range(value, nonnegative)?;
        }
    }
    for conditions in [
        ratings.interrupt.map(|r| r.conditions),
        ratings.tcc.map(|r| r.conditions),
    ]
    .into_iter()
    .flatten()
    {
        validate_bounds(conditions.temperature_k, positive)?;
        validate_bounds(conditions.time_constant_s, nonnegative)?;
        validate_waveform(conditions.waveform, name_ok)?;
    }
    if let Some(r) = ratings.interrupt {
        validate_range(Some(r.max_prospective_current_a), positive)?;
        validate_range(Some(r.max_recovery_voltage_v), positive)?;
    }
    if let Some(r) = ratings.tcc {
        validate_bounds(r.recovery_voltage_v, nonnegative)?;
        for band in r.bands {
            validate_bounds(band.prospective_current_a, positive)?;
            validate_range(band.time_s, nonnegative)?;
        }
    }
    if let Some(r) = ratings.wire {
        validate_bounds(r.start_temperature_k, positive)?;
        validate_range(Some(r.damage_i2t_a2s), nonnegative)?;
    }
    if let Some(r) = ratings.switch {
        validate_bounds(r.start_temperature_k, positive)?;
        validate_bounds(r.current_a, nonnegative)?;
        validate_bounds(r.voltage_v, nonnegative)?;
        validate_waveform(r.waveform, name_ok)?;
        validate_range(Some(r.damage_time_s), nonnegative)?;
    }
    Ok(())
}

fn validate_bounds(
    bounds: FuseBounds,
    condition: impl Fn(f64) -> bool,
) -> Result<(), ProcessError> {
    if bounds.min.is_finite()
        && bounds.max.is_finite()
        && bounds.min <= bounds.max
        && condition(bounds.min)
        && condition(bounds.max)
    {
        Ok(())
    } else {
        Err(invalid("неконечная или недопустимая граница защиты"))
    }
}

fn validate_range(
    range: Option<&SourcedRange>,
    condition: impl Fn(f64) -> bool,
) -> Result<(), ProcessError> {
    if let Some(r) = range {
        r.validate()?;
        validate_bounds(FuseBounds::from(r), condition)?;
    }
    Ok(())
}

fn validate_waveform(
    shape: FuseWaveform<'_>,
    name_ok: impl Fn(&str) -> bool,
) -> Result<(), ProcessError> {
    if matches!(shape, FuseWaveform::Characterized(id) if !name_ok(id)) {
        return Err(invalid("недопустимый идентификатор формы импульса"));
    }
    Ok(())
}

fn invalid(reason: &str) -> ProcessError {
    ProcessError::InvalidInput(reason.into())
}

fn supports(scope: Scope, source: &Provenance) -> bool {
    matches!(
        source,
        Provenance::Datasheet(_) | Provenance::Measured(_) | Provenance::Paper(_)
    ) || (scope == Scope::Reference && matches!(source, Provenance::Derived(_)))
}

fn proven(scope: Scope, range: Option<&SourcedRange>) -> Option<FuseBounds> {
    let range = range?;
    supports(scope, &range.provenance).then(|| FuseBounds::from(range))
}

impl From<&SourcedRange> for FuseBounds {
    fn from(r: &SourcedRange) -> Self {
        Self {
            min: r.min,
            max: r.max,
        }
    }
}

impl FuseBounds {
    const ZERO: Self = Self { min: 0.0, max: 0.0 };

    fn contains(self, other: Self) -> bool {
        self.min <= other.min && self.max >= other.max
    }

    fn hull(self, other: Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }
}

fn compatible(input: &FuseCoordinationInput<'_>, band: FuseDcConditions<'_>) -> Option<()> {
    let observed = |r| proven(input.scope, r);
    (Some(band.waveform) == input.prospective_waveform
        && band
            .temperature_k
            .contains(observed(input.fuse_temperature_k)?)
        && band
            .time_constant_s
            .contains(observed(input.dc_time_constant_s)?))
    .then_some(())
}

fn interrupt(
    input: &FuseCoordinationInput<'_>,
    rating: Option<FuseInterruptRating<'_>>,
) -> (Status, Status) {
    let values = |rating: FuseInterruptRating<'_>| {
        compatible(input, rating.conditions)?;
        Some((
            interrupt_status(
                input.scope,
                input.prospective_current_a,
                rating.max_prospective_current_a,
            ),
            interrupt_status(
                input.scope,
                input.recovery_voltage_v,
                rating.max_recovery_voltage_v,
            ),
        ))
    };
    rating
        .and_then(values)
        .unwrap_or((Status::NotAnalyzed, Status::NotAnalyzed))
}

fn interrupt_status(scope: Scope, actual: Option<&SourcedRange>, limit: &SourcedRange) -> Status {
    match proven(scope, actual).zip(proven(scope, Some(limit))) {
        Some((actual, limit)) if actual.min > limit.max => Status::Fail,
        Some((actual, limit)) if actual.max <= limit.min => Status::Pass,
        _ => Status::NotAnalyzed,
    }
}

fn clearing(input: &FuseCoordinationInput<'_>, curve: Option<FuseTcc<'_>>) -> Option<FuseBounds> {
    let curve = curve?;
    compatible(input, curve.conditions)?;
    if curve.stage != FuseTimeStage::TotalClearing
        || !curve
            .recovery_voltage_v
            .contains(proven(input.scope, input.recovery_voltage_v)?)
    {
        return None;
    }
    let current = proven(input.scope, input.prospective_current_a)?;
    let mut bands: Vec<_> = curve
        .bands
        .iter()
        .filter_map(|band| {
            let domain = band.prospective_current_a;
            let time = proven(input.scope, band.time_s)?;
            (domain.max >= current.min && domain.min <= current.max).then_some((domain, time))
        })
        .collect();
    bands.sort_unstable_by(|(a, _), (b, _)| a.min.total_cmp(&b.min));

    // Объединяем весь покрытый диапазон. Промежуток между точками таблицы
    // без подтверждённой полосы нельзя закрыть интерполяцией.
    let mut frontier = current.min;
    let mut time: Option<FuseBounds> = None;
    for (domain, bounds) in bands {
        if domain.min > frontier {
            return None;
        }
        frontier = frontier.max(domain.max);
        time = Some(time.map_or(bounds, |value| value.hull(bounds)));
    }
    if frontier < current.max {
        return None;
    }
    time
}

fn elapsed(origin: Time, at: Time) -> FuseBounds {
    if origin == at {
        return FuseBounds::ZERO;
    }
    // 10⁹ нс в секунде. Округление наружу после преобразования целого и деления.
    let ns = (at.0 - origin.0) as f64;
    FuseBounds {
        min: (ns.next_down() / 1_000_000_000.0).next_down(),
        max: (ns.next_up() / 1_000_000_000.0).next_up(),
    }
}

fn rounded(min: f64, max: f64) -> Result<FuseBounds, ProcessError> {
    let bounds = FuseBounds {
        min: min.next_down(),
        max: max.next_up(),
    };
    if bounds.min.is_finite() && bounds.max.is_finite() {
        Ok(bounds)
    } else {
        Err(ProcessError::Execution(
            "переполнение расчёта согласования защиты".into(),
        ))
    }
}

fn wire_damage(
    input: &FuseCoordinationInput<'_>,
    rating: Option<FuseWireLimit<'_>>,
    horizon: FuseBounds,
) -> Result<Option<FuseWireEvidence>, ProcessError> {
    let bound = |x| proven(input.scope, x);
    let Some(limit) = rating
        .filter(|r| {
            bound(input.wire_start_temperature_k).is_some_and(|t| r.start_temperature_k.contains(t))
        })
        .and_then(|r| bound(Some(r.damage_i2t_a2s)))
    else {
        return Ok(None);
    };
    let mut dose = FuseBounds::ZERO;
    let mut earliest = (limit.min == 0.0).then_some(0.0);
    let mut latest = (limit.max == 0.0).then_some(0.0);
    for step in input.trace {
        let Some(current) = bound(step.wire_current_a) else {
            return Ok(None);
        };
        if current.max == 0.0 {
            continue;
        }
        let rate = rounded(current.min * current.min, current.max * current.max)?;
        let dt = elapsed(step.interval.start, step.interval.end);
        let increment = rounded(rate.min.max(0.0) * dt.min, rate.max * dt.max)?;
        let after = rounded(dose.min + increment.min.max(0.0), dose.max + increment.max)?;
        let start = elapsed(input.window.start, step.interval.start);
        let end = elapsed(input.window.start, step.interval.end);
        // I²t монотонен. Верхняя огибающая тока даёт первое возможное
        // пересечение нижнего предела, нижняя — последнее пересечение верхнего.
        if earliest.is_none() && after.max >= limit.min {
            let remaining = (limit.min - dose.max).next_down().max(0.0);
            let offset = (remaining / rate.max).next_down().max(0.0);
            earliest = Some((start.min + offset).next_down().max(0.0).min(end.min));
        }
        if latest.is_none() && after.min >= limit.max {
            let remaining = (limit.max - dose.min).next_up();
            let offset = if rate.min > 0.0 {
                (remaining / rate.min).next_up()
            } else {
                dt.max
            };
            latest = Some((start.max + offset).next_up().min(end.max));
        }
        dose = FuseBounds {
            min: after.min.max(0.0),
            max: after.max,
        };
    }
    Ok(Some(FuseWireEvidence {
        window_i2t_a2s: dose,
        damage_time_s: FuseDamageTime {
            earliest_s: earliest.unwrap_or(horizon.min),
            latest_s: latest,
        },
    }))
}

fn switch_damage(
    input: &FuseCoordinationInput<'_>,
    rating: Option<FuseSwitchSoa<'_>>,
    horizon: FuseBounds,
) -> Option<FuseDamageTime> {
    let rating = rating?;
    let observe = |r| proven(input.scope, r);
    if Some(rating.waveform) != input.switch_waveform
        || !rating
            .start_temperature_k
            .contains(observe(input.switch_start_temperature_k)?)
    {
        return None;
    }
    let read = |step: &FuseFaultStep<'_>| {
        proven(input.scope, step.switch_current_a).zip(proven(input.scope, step.switch_voltage_v))
    };
    let baseline = read(input.trace.first()?)?;
    for step in input.trace {
        let (current, voltage) = read(step)?;
        if !rating.current_a.contains(current)
            || !rating.voltage_v.contains(voltage)
            || (rating.waveform == FuseWaveform::DcStep && (current, voltage) != baseline)
        {
            return None;
        }
    }
    let limit = observe(Some(rating.damage_time_s))?;
    Some(FuseDamageTime {
        earliest_s: limit.min.min(horizon.min),
        latest_s: (limit.max <= horizon.min).then_some(limit.max),
    })
}

fn margin(damage: FuseDamageTime, clear: FuseBounds) -> Result<FuseTimeMargin, ProcessError> {
    let min = (damage.earliest_s - clear.max).next_down();
    let max = damage.latest_s.map(|limit| (limit - clear.min).next_up());
    if !min.is_finite() || max.is_some_and(|x| !x.is_finite()) {
        return Err(ProcessError::Execution(
            "переполнение запаса по времени отключения".into(),
        ));
    }
    Ok(FuseTimeMargin { min, max })
}

fn analyzed<T>(value: Option<T>, reason: &str) -> Analysis<T> {
    match value {
        Some(value) => Analysis::Analyzed {
            value,
            scope: "ветви и окно указанного отказа".into(),
        },
        None => Analysis::NotAnalyzed {
            reason: reason.into(),
        },
    }
}

fn summarize(
    path: bool,
    clear: Option<FuseBounds>,
    horizon: FuseBounds,
    e: &FuseCoordinationEvidence,
) -> (Status, String) {
    if !path {
        return (
            Status::NotAnalyzed,
            "не подтверждён путь отключения нагрузки этим предохранителем".into(),
        );
    }
    let mut violations = Vec::new();
    let mut gaps = Vec::new();
    for (status, label) in [
        (e.interrupt_current, "отключающий ток"),
        (e.interrupt_voltage, "восстанавливающееся напряжение"),
    ] {
        match status {
            Status::Fail => violations.push(format!("превышен предел: {label}")),
            Status::NotAnalyzed => gaps.push(format!("не подтверждён предел: {label}")),
            Status::Pass => {}
        }
    }
    for (label, time) in [("провод", e.wire_margin_s), ("ключ", e.switch_margin_s)] {
        if let Some(time) = time {
            if time.max.is_some_and(|max| max < 0.0) {
                violations.push(format!("{label} достигает предела до полного отключения"));
            } else if time.min <= 0.0 {
                gaps.push(format!(
                    "{label}: интервалы отключения и предела пересекаются или запас не установлен"
                ));
            }
        }
    }
    if !violations.is_empty() {
        return (Status::Fail, violations.join(". "));
    }
    for absent in [
        missing_reason(&e.clearing_time_s),
        missing_reason(&e.wire),
        missing_reason(&e.switch),
    ]
    .into_iter()
    .flatten()
    {
        gaps.push(absent.to_owned());
    }
    if clear.is_some_and(|time| time.max > horizon.min) {
        gaps.push("окно заканчивается раньше верхней границы полного отключения".into());
    }
    if gaps.is_empty() {
        (
            Status::Pass,
            "полное отключение раньше обоих пределов в указанном окне".into(),
        )
    } else {
        (Status::NotAnalyzed, gaps.join(". "))
    }
}

fn missing_reason<T>(analysis: &Analysis<T>) -> Option<&str> {
    match analysis {
        Analysis::Analyzed { .. } => None,
        Analysis::NotAnalyzed { reason } => Some(reason),
    }
}
