//! Адресная оценка одиночного контактного разряда на резистивном стенде.
//! Область ограничена заявленными монтажом, полосой и погрешностью клеммной модели.

use std::fmt::Write as _;
use std::path::Path;

use sim_circuit::ngspice::{self, NgAnalysis};
use sim_signal::Time;
use sim_world::Provenance;

use crate::schematic::{CompiledSchematic, Endpoint, Finding, Scope, Status};
use crate::{Analysis, ProcessError, SourcedRange};

pub const ESD_EVENT_CODE: &str = "esd_event";
const BENCH_SCOPE: &str = "одиночный контактный импульс на указанных клеммах и монтаже";
const SPICE_SCOPE: &str = "независимый переходный расчёт эквивалента этого стенда";
// Скорость света в вакууме по определению СИ. Для клеммного тракта
// проверяем скорость распространения возмущения, как в LumpedDomain.
const LIGHT_SPEED_M_S: f64 = 299_792_458.0;
// То же отсечение λ/10 для сосредоточенной модели, что в LumpedDomain.
// MIT 6.013, §14.4: https://web.mit.edu/6.013_book/www/chapter14/14.html
const MAX_WAVELENGTH_FRACTION: f64 = 0.1;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EsdPort {
    pub exposed: Endpoint,
    pub protected: Endpoint,
    /// Общий физический возврат воздействия, защиты и защищаемого вывода.
    pub return_: Endpoint,
}

/// Уровни прямого контактного разряда IEC 61000-4-2:2008.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EsdLevel {
    Kv2,
    Kv4,
    Kv6,
    Kv8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EsdPolarity {
    Positive,
    Negative,
}

impl EsdPolarity {
    fn sign(self) -> f64 {
        match self {
            Self::Positive => 1.0,
            Self::Negative => -1.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EsdSample {
    /// Секунды от начала контакта. Дробные наносекунды сохраняются.
    pub offset_s: f64,
    pub current_abs_a: f64,
}

/// Кусочно-линейная форма от нуля до конца импульса. Значения заданы по модулю.
/// Источник подтверждает редакцию, уровень, полярность и применимость всей
/// формы, включая хвост и погрешность оцифровки, к заявленной нагрузке.
/// Ток калибровочной мишени требует подтверждения переноса на эту клемму.
#[derive(Clone, Copy, Debug)]
pub struct EsdPulse<'a> {
    pub level: EsdLevel,
    pub polarity: EsdPolarity,
    pub samples: &'a [EsdSample],
    /// Верхняя значимая частота из данных стенда. Остаток вне полосы
    /// учитывается в подтверждённых границах погрешности эквивалента.
    pub bandwidth_hz: f64,
    pub source: Option<&'a Provenance>,
}

/// Граница применимости эквивалента для данного монтажа и формы разряда.
/// Источник связывает этот эквивалент с ревизией и указанными адресами,
/// подтверждает начальное состояние, параллельные пути, возврат и
/// допустимость замены нагруженного генератора источником тока.
#[derive(Clone, Copy, Debug)]
pub struct EsdDomain<'a> {
    pub bandwidth_hz: f64,
    /// Наибольшая существенная длина пути сигнала или возврата.
    pub length_m: f64,
    /// Нижняя граница скорости распространения на этих путях.
    pub wave_speed_m_s: f64,
    /// Допустимая электрическая длина l/λ в полосе модели.
    pub max_length_wavelengths: f64,
    /// Модуль напряжения клеммы, до которого подтверждён эквивалент.
    pub max_terminal_abs_v: f64,
    /// Двусторонняя абсолютная погрешность U/I вывода на всём импульсе.
    /// Включает монтаж, выбросы L·di/dt, паразитные ёмкости, пределы ВАХ,
    /// погрешность входной формы и численное округление. Нули допустимы для
    /// точного синтетического стенда.
    pub voltage_error_v: f64,
    pub current_error_a: f64,
    pub source: Option<&'a Provenance>,
}

/// Модель TVS без памяти. Iтvs = max(0, (|Vклеммы| − V₀)/Rᴅ)
/// для одной заявленной полярности. Паспорт и подтверждение эквивалента
/// должны покрывать эту аппроксимацию на всём импульсе и при данном монтаже.
/// DC, 8/20 мкс или одна точка TLP сами по себе фронт ESD не определяют.
#[derive(Clone, Copy, Debug)]
pub struct EsdTvs<'a> {
    pub polarity: EsdPolarity,
    pub knee_v: f64,
    pub dynamic_ohm: f64,
    /// Верхняя граница применимого пика источника. В модели с пассивным
    /// делением пиковый ток TVS ограничен сверху током источника.
    pub max_injected_peak_a: f64,
    pub min_rise_s: f64,
    pub max_pulse_s: f64,
    pub datasheet: Option<&'a Provenance>,
}

#[derive(Clone, Copy, Debug)]
pub enum EsdProtection<'a> {
    /// Схема или обмер подтверждает отсутствие значимой ветви защиты.
    Absent(&'a Provenance),
    /// Шунт между открытой клеммой и локальным возвратом.
    Tvs(EsdTvs<'a>),
}

/// Открытая клемма — Rseries — защищаемый вывод — Rpin — физический возврат.
/// Ток вывода направлен от него в нагрузку. Модель без рабочего смещения.
#[derive(Clone, Copy, Debug)]
pub struct EsdNetwork<'a> {
    pub series_ohm: f64,
    pub pin_ohm: f64,
    pub domain: EsdDomain<'a>,
    pub protection: Option<EsdProtection<'a>>,
}

/// Границы мгновенных модулей U/I на этом выводе для указанной полярности,
/// формы и длительности. Источник подтверждает применимость к одному воздействию.
#[derive(Clone, Debug, PartialEq)]
pub struct EsdPinLimits {
    pub voltage_abs_v: SourcedRange,
    pub current_abs_a: SourcedRange,
}

#[derive(Clone, Copy, Debug)]
pub struct EsdInput<'a> {
    pub port: &'a EsdPort,
    pub start: Time,
    /// Подтверждённый прямой контакт зонда с открытой клеммой в этой сборке.
    pub exposure: Option<&'a Provenance>,
    pub pulse: Option<EsdPulse<'a>>,
    pub network: Option<EsdNetwork<'a>>,
    pub limits: Option<&'a EsdPinLimits>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EsdPulseMetrics {
    pub first_peak_a: f64,
    pub rise_s: f64,
    /// Отсчёты от первого достижения 10% тока первого пика.
    pub at_30ns_a: f64,
    pub at_60ns_a: f64,
    pub duration_s: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EsdValues {
    pub terminal_v: f64,
    pub pin_v: f64,
    pub pin_a: f64,
    pub tvs_a: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EsdResponse {
    pub pulse: EsdPulseMetrics,
    pub electrical_length_wavelengths: f64,
    pub peak_offset_s: f64,
    /// Знаковые уровни в момент максимума модулей.
    pub peak: EsdValues,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EsdMargin {
    /// Предел минус нагрузка, с учётом обеих огибающих.
    pub lower: f64,
    pub nominal: f64,
    pub upper: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EsdMargins {
    pub voltage_v: EsdMargin,
    pub current_a: EsdMargin,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EsdCriterion {
    Exceeded,
    WithinBench,
    Overlap,
}

#[derive(Clone, Debug, PartialEq)]
pub struct EsdEvidence {
    pub code: &'static str,
    pub scope: Scope,
    pub port: EsdPort,
    pub start: Time,
    pub level: Option<EsdLevel>,
    pub polarity: Option<EsdPolarity>,
    pub pulse_source: Option<Provenance>,
    pub response: Analysis<EsdResponse>,
    pub margins: Analysis<EsdMargins>,
    /// Результат сравнения с пределами данного стенда.
    pub criterion: Option<EsdCriterion>,
}

#[derive(Clone, Copy, Debug)]
pub struct EsdSpiceOptions {
    pub max_step_s: f64,
    pub voltage_tolerance_v: f64,
    pub current_tolerance_a: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EsdSpiceComparison {
    pub rows: usize,
    pub pin_voltage_peak_abs_v: f64,
    pub pin_current_peak_abs_a: f64,
    /// Максимум ошибки по шкале ngspice и отдельному сравнению пиков.
    pub voltage_error_v: f64,
    pub current_error_a: f64,
    pub within_tolerance: bool,
}

/// Превышение даёт esd_event / Fail. Запас в области стенда сохраняется
/// численно, а итог оценки остаётся открытым до измерений на установке.
pub fn assess_esd_event(
    schematic: &CompiledSchematic,
    input: &EsdInput<'_>,
) -> Result<Finding<EsdEvidence>, ProcessError> {
    let scope = schematic.depth_scope();
    let mut evidence = EsdEvidence {
        code: ESD_EVENT_CODE,
        scope,
        port: input.port.clone(),
        start: input.start,
        level: input.pulse.map(|p| p.level),
        polarity: input.pulse.map(|p| p.polarity),
        pulse_source: input.pulse.and_then(|p| p.source).cloned(),
        response: open("нет применимого расчёта стенда"),
        margins: open("нет применимых U/I и пределов вывода"),
        criterion: None,
    };
    let mut status = Status::NotAnalyzed;
    let reason = match prepare(schematic, input)? {
        Analysis::NotAnalyzed { reason } => {
            evidence.response = open(reason.clone());
            reason
        }
        Analysis::Analyzed {
            value: prepared, ..
        } => {
            evidence.response = analyzed(prepared.response, BENCH_SCOPE);
            match input.limits.filter(|limits| {
                confirmed(scope, Some(&limits.voltage_abs_v.provenance))
                    && confirmed(scope, Some(&limits.current_abs_a.provenance))
            }) {
                None => {
                    let reason = "нужны применимые импульсные пределы напряжения и тока вывода";
                    evidence.margins = open(reason);
                    reason.into()
                }
                Some(limits) => {
                    let peak = prepared.response.peak;
                    let domain = prepared.network.domain;
                    let margins = EsdMargins {
                        voltage_v: margin(
                            peak.pin_v.abs(),
                            domain.voltage_error_v,
                            &limits.voltage_abs_v,
                        )?,
                        current_a: margin(
                            peak.pin_a.abs(),
                            domain.current_error_a,
                            &limits.current_abs_a,
                        )?,
                    };
                    let criterion =
                        if margins.voltage_v.upper < 0.0 || margins.current_a.upper < 0.0 {
                            status = Status::Fail;
                            EsdCriterion::Exceeded
                        } else if margins.voltage_v.lower >= 0.0 && margins.current_a.lower >= 0.0 {
                            EsdCriterion::WithinBench
                        } else {
                            EsdCriterion::Overlap
                        };
                    evidence.criterion = Some(criterion);
                    evidence.margins = analyzed(margins, BENCH_SCOPE);
                    match criterion {
                        EsdCriterion::Exceeded => "остаточное напряжение или ток вывода превышает импульсный предел",
                        EsdCriterion::WithinBench => "есть запас по U/I в области модели; нужны стендовые измерения на выводе",
                        EsdCriterion::Overlap => "огибающая остаточных U/I пересекает предел; нужны стендовые измерения на выводе",
                    }
                    .into()
                }
            }
        }
    };
    Ok(Finding {
        subject: input.port.exposed.to_string(),
        status,
        reason,
        evidence,
    })
}

/// Сверяет остаточные U/I в каждой точке ngspice и по пикам.
/// Внешняя программа получает только сформированный числовой нетлист.
pub fn cross_check_esd_event(
    schematic: &CompiledSchematic,
    input: &EsdInput<'_>,
    executable: &Path,
    options: EsdSpiceOptions,
) -> Result<Analysis<EsdSpiceComparison>, ProcessError> {
    positive("шаг ngspice", options.max_step_s)?;
    nonnegative("допуск напряжения для сверки", options.voltage_tolerance_v)?;
    nonnegative("допуск тока для сверки", options.current_tolerance_a)?;
    let prepared = match prepare(schematic, input)? {
        Analysis::Analyzed { value, .. } => value,
        Analysis::NotAnalyzed { reason } => return Ok(open(reason)),
    };
    let response = prepared.response;
    let step = options.max_step_s;
    if step >= response.pulse.rise_s || !(response.pulse.duration_s / step).is_finite() {
        return Ok(open(
            "шаг ngspice не разрешает фронт при заданной длительности",
        ));
    }
    let deck = prepared.deck(options);
    let table = match ngspice::run(
        executable,
        &deck,
        NgAnalysis::Tran {
            tstep: step,
            tstop: response.pulse.duration_s,
            tstart: 0.0,
            tmax: step,
            uic: false,
        },
        &["v(protected)", "i(vprobe)"],
    ) {
        Ok(table) => table,
        Err(error) => return Ok(open(format!("нет полной сверки ngspice: {error}"))),
    };

    let mut comparison = EsdSpiceComparison {
        rows: table.rows.len(),
        pin_voltage_peak_abs_v: 0.0,
        pin_current_peak_abs_a: 0.0,
        voltage_error_v: 0.0,
        current_error_a: 0.0,
        within_tolerance: false,
    };
    for row in &table.rows {
        let expected = prepared.values(current_at(prepared.pulse.samples, row[0]))?;
        let voltage = row[1];
        let current = row[2];
        comparison.pin_voltage_peak_abs_v = comparison.pin_voltage_peak_abs_v.max(voltage.abs());
        comparison.pin_current_peak_abs_a = comparison.pin_current_peak_abs_a.max(current.abs());
        comparison.voltage_error_v = comparison
            .voltage_error_v
            .max((voltage - expected.pin_v).abs());
        comparison.current_error_a = comparison
            .current_error_a
            .max((current - expected.pin_a).abs());
    }
    comparison.voltage_error_v = comparison
        .voltage_error_v
        .max((comparison.pin_voltage_peak_abs_v - response.peak.pin_v.abs()).abs());
    comparison.current_error_a = comparison
        .current_error_a
        .max((comparison.pin_current_peak_abs_a - response.peak.pin_a.abs()).abs());
    if !comparison.voltage_error_v.is_finite() || !comparison.current_error_a.is_finite() {
        return Ok(open("невязка ngspice вышла за числовой диапазон"));
    }
    comparison.within_tolerance = comparison.voltage_error_v <= options.voltage_tolerance_v
        && comparison.current_error_a <= options.current_tolerance_a;
    Ok(analyzed(comparison, SPICE_SCOPE))
}

#[derive(Clone, Copy)]
struct Prepared<'a> {
    pulse: EsdPulse<'a>,
    network: EsdNetwork<'a>,
    tvs: Option<EsdTvs<'a>>,
    response: EsdResponse,
}

fn prepare<'a>(
    schematic: &CompiledSchematic,
    input: &EsdInput<'a>,
) -> Result<Analysis<Prepared<'a>>, ProcessError> {
    validate(schematic, input)?;
    let scope = schematic.depth_scope();
    if !confirmed(scope, input.exposure) {
        return Ok(open(
            "нужен подтверждённый доступ зонда к указанной открытой клемме",
        ));
    }
    let Some(pulse) = input.pulse else {
        return Ok(open("не задан контактный импульс IEC 61000-4-2:2008"));
    };
    if !confirmed(scope, pulse.source) {
        return Ok(open(
            "нужен применимый источник импульса для этой клеммы и нагрузки",
        ));
    }
    let Some((metrics, peak)) = pulse_metrics(pulse) else {
        return Ok(open(
            "форма не покрывает параметры контактного импульса IEC 61000-4-2:2008",
        ));
    };
    let Some(network) = input.network else {
        return Ok(open("не задан эквивалент тракта от клеммы до вывода"));
    };
    let domain = network.domain;
    if !confirmed(scope, domain.source) {
        return Ok(open(
            "не подтверждены область эквивалента и огибающая ошибки U/I",
        ));
    }
    if scope == Scope::Assembly && (domain.voltage_error_v == 0.0 || domain.current_error_a == 0.0)
    {
        return Ok(open(
            "нужна ненулевая граница погрешности U/I для физического монтажа",
        ));
    }
    if pulse.bandwidth_hz > domain.bandwidth_hz {
        return Ok(open("полоса импульса выходит за область эквивалента"));
    }
    if domain.wave_speed_m_s > LIGHT_SPEED_M_S {
        return Ok(open(
            "для эквивалента нужна физическая скорость распространения импульса",
        ));
    }
    let wavelength = positive(
        "длина волны в полосе стенда",
        domain.wave_speed_m_s / pulse.bandwidth_hz,
    )?;
    let electrical_length = nonnegative("электрическая длина", domain.length_m / wavelength)?;
    if electrical_length > domain.max_length_wavelengths.min(MAX_WAVELENGTH_FRACTION) {
        return Ok(open(
            "электрическая длина тракта выходит за область эквивалента",
        ));
    }
    let tvs = match network.protection {
        Some(EsdProtection::Absent(source)) if confirmed(scope, Some(source)) => None,
        Some(EsdProtection::Tvs(tvs)) => {
            let datasheet = tvs.datasheet.is_some_and(|source| {
                matches!(source, Provenance::Datasheet(_))
                    || (scope == Scope::Reference && matches!(source, Provenance::Derived(_)))
            }) && confirmed(scope, tvs.datasheet);
            if !datasheet {
                return Ok(open("нужен паспорт применимой импульсной ВАХ TVS"));
            }
            if tvs.polarity != pulse.polarity
                || metrics.rise_s < tvs.min_rise_s
                || metrics.duration_s > tvs.max_pulse_s
                || peak.current_abs_a > tvs.max_injected_peak_a
            {
                return Ok(open("импульс вне паспортной области ВАХ TVS"));
            }
            Some(tvs)
        }
        _ => return Ok(open("состояние защиты на открытой клемме не подтверждено")),
    };
    let values = values(network, tvs, pulse.polarity, peak.current_abs_a)?;
    if values.terminal_v.abs() > domain.max_terminal_abs_v {
        return Ok(open(
            "напряжение клеммы выходит за область эквивалента источника",
        ));
    }
    Ok(analyzed(
        Prepared {
            pulse,
            network,
            tvs,
            response: EsdResponse {
                pulse: metrics,
                electrical_length_wavelengths: electrical_length,
                peak_offset_s: peak.offset_s,
                peak: values,
            },
        },
        BENCH_SCOPE,
    ))
}

fn validate(schematic: &CompiledSchematic, input: &EsdInput<'_>) -> Result<(), ProcessError> {
    let port = input.port;
    if port.exposed == port.protected
        || port.exposed == port.return_
        || port.protected == port.return_
    {
        return Err(invalid("для ESD требуются три разных вывода"));
    }
    for end in [&port.exposed, &port.protected, &port.return_] {
        if !schematic.pins().any(|pin| pin == end) {
            return Err(invalid(format!("неизвестный вывод ESD «{end}»")));
        }
    }
    if let Some(pulse) = input.pulse {
        positive("полоса импульса", pulse.bandwidth_hz)?;
        for sample in pulse.samples {
            nonnegative("время импульса", sample.offset_s)?;
            nonnegative("модуль тока импульса", sample.current_abs_a)?;
        }
        if pulse
            .samples
            .windows(2)
            .any(|pair| pair[0].offset_s >= pair[1].offset_s)
        {
            return Err(invalid("время импульса должно строго возрастать"));
        }
    }
    if let Some(network) = input.network {
        if network.series_ohm != 0.0 {
            resistance("последовательное сопротивление", network.series_ohm)?;
        }
        resistance("сопротивление вывода", network.pin_ohm)?;
        positive(
            "сумма сопротивлений тракта",
            network.series_ohm + network.pin_ohm,
        )?;
        let domain = network.domain;
        positive("полоса эквивалента", domain.bandwidth_hz)?;
        nonnegative("длина тракта", domain.length_m)?;
        positive("скорость распространения", domain.wave_speed_m_s)?;
        positive(
            "предельная электрическая длина",
            domain.max_length_wavelengths,
        )?;
        positive("предельное напряжение клеммы", domain.max_terminal_abs_v)?;
        nonnegative("ошибка напряжения", domain.voltage_error_v)?;
        nonnegative("ошибка тока", domain.current_error_a)?;
        if let Some(EsdProtection::Tvs(tvs)) = network.protection {
            nonnegative("напряжение начала ограничения TVS", tvs.knee_v)?;
            resistance("динамическое сопротивление TVS", tvs.dynamic_ohm)?;
            positive("предельный пик для TVS", tvs.max_injected_peak_a)?;
            positive("минимальный фронт для TVS", tvs.min_rise_s)?;
            positive("предельная длительность для TVS", tvs.max_pulse_s)?;
        }
    }
    if let Some(limits) = input.limits {
        for (label, range) in [
            ("предел напряжения вывода", &limits.voltage_abs_v),
            ("предел тока вывода", &limits.current_abs_a),
        ] {
            for value in [range.min, range.nominal, range.max] {
                nonnegative(label, value)?;
            }
            if range.min > range.nominal || range.nominal > range.max {
                return Err(invalid(format!("{label}: нарушен порядок границ")));
            }
        }
    }
    Ok(())
}

fn pulse_metrics(pulse: EsdPulse<'_>) -> Option<(EsdPulseMetrics, EsdSample)> {
    let samples = pulse.samples;
    let start = samples.first()?;
    let end = samples.last()?;
    if start.offset_s != 0.0 || start.current_abs_a != 0.0 || end.current_abs_a != 0.0 {
        return None;
    }
    // Первая вершина может быть плоской. Первый спад завершает её,
    // глобальный максимум ищется и на остальной части записи.
    let first_index = samples
        .windows(2)
        .position(|p| p[1].current_abs_a < p[0].current_abs_a)?;
    let first_peak = samples[first_index].current_abs_a;

    // IEC 61000-4-2:2008, §6.2.2, таблица 3 и рисунок 2.
    // https://preview.sist.si/sist-preview/15211/3d21bb73620b41d9a383aa4c15fecb28/IEC-61000-4-2-2008.pdf
    // 10–90%, 0,8 нс ±25%; Ipeak ±15%; I30 и I60 ±30% от уровня.
    let (reference_peak, reference_30, reference_60) = match pulse.level {
        EsdLevel::Kv2 => (7.5, 4.0, 2.0),
        EsdLevel::Kv4 => (15.0, 8.0, 4.0),
        EsdLevel::Kv6 => (22.5, 12.0, 6.0),
        EsdLevel::Kv8 => (30.0, 16.0, 8.0),
    };
    let crossing = |fraction| {
        let threshold = first_peak * fraction;
        let pair = samples[..=first_index]
            .windows(2)
            .find(|w| w[1].current_abs_a >= threshold)?;
        let ratio =
            (threshold - pair[0].current_abs_a) / (pair[1].current_abs_a - pair[0].current_abs_a);
        Some(pair[0].offset_s + ratio * (pair[1].offset_s - pair[0].offset_s))
    };
    let t10 = crossing(0.1)?;
    let rise = crossing(0.9)? - t10;
    let t30 = t10 + 30e-9;
    let t60 = t10 + 60e-9;
    if end.offset_s < t60 || samples[first_index].offset_s >= t30 {
        return None;
    }
    let i30 = current_at(samples, t30);
    let i60 = current_at(samples, t60);
    let in_band = |value, nominal, tolerance| {
        ((nominal * (1.0 - tolerance))..=(nominal * (1.0 + tolerance))).contains(&value)
    };
    if !in_band(first_peak, reference_peak, 0.15)
        || !(0.6e-9..=1.0e-9).contains(&rise)
        || !in_band(i30, reference_30, 0.30)
        || !in_band(i60, reference_60, 0.30)
    {
        return None;
    }
    let peak = samples.iter().copied().fold(*start, |peak, sample| {
        if sample.current_abs_a > peak.current_abs_a {
            sample
        } else {
            peak
        }
    });
    Some((
        EsdPulseMetrics {
            first_peak_a: first_peak,
            rise_s: rise,
            at_30ns_a: i30,
            at_60ns_a: i60,
            duration_s: end.offset_s,
        },
        peak,
    ))
}

fn current_at(samples: &[EsdSample], time: f64) -> f64 {
    let next = samples.partition_point(|sample| sample.offset_s <= time);
    if next == 0 || next == samples.len() {
        return 0.0; // Обе границы применимой записи имеют нулевой ток.
    }
    let left = samples[next - 1];
    let right = samples[next];
    let ratio = (time - left.offset_s) / (right.offset_s - left.offset_s);
    left.current_abs_a + (right.current_abs_a - left.current_abs_a) * ratio
}

fn values(
    network: EsdNetwork<'_>,
    tvs: Option<EsdTvs<'_>>,
    polarity: EsdPolarity,
    input_a: f64,
) -> Result<EsdValues, ProcessError> {
    let path = network.series_ohm + network.pin_ohm;
    // KCL: I = V/Rpath + max(0, (V−V₀)/Rᴅ). При открытом TVS
    // приращение V равно (I−V₀/Rpath)·(Rpath || Rᴅ).
    // Обе ветви монотонны, поэтому максимумы U/I находятся в вершинах PWL.
    let terminal = if let Some(tvs) = tvs.filter(|tvs| input_a > tvs.knee_v / path) {
        let smaller = path.min(tvs.dynamic_ohm);
        let larger = path.max(tvs.dynamic_ohm);
        let parallel = smaller / (1.0 + smaller / larger);
        tvs.knee_v + (input_a - tvs.knee_v / path) * parallel
    } else {
        input_a * path
    };
    let pin_a = terminal / path;
    let pin_v = pin_a * network.pin_ohm;
    let sign = polarity.sign();
    let tvs_a = if tvs.is_some() {
        (input_a - pin_a).max(0.0)
    } else {
        0.0
    };
    for value in [terminal, pin_a, pin_v, tvs_a] {
        nonnegative("расчётное значение ESD", value)?;
    }
    Ok(EsdValues {
        terminal_v: sign * terminal,
        pin_v: sign * pin_v,
        pin_a: sign * pin_a,
        tvs_a: sign * tvs_a,
    })
}

impl Prepared<'_> {
    fn values(self, input_a: f64) -> Result<EsdValues, ProcessError> {
        values(self.network, self.tvs, self.pulse.polarity, input_a)
    }

    fn deck(self, options: EsdSpiceOptions) -> String {
        let mut deck = String::from("Стенд ESD\nIstrike 0 strike PWL(");
        let sign = self.pulse.polarity.sign();
        for sample in self.pulse.samples {
            write!(
                deck,
                "\n+ {:.16e} {:.16e}",
                sample.offset_s,
                sign * sample.current_abs_a
            )
            .unwrap();
        }
        deck.push_str(")\nVprobe protected load 0\n");
        if self.network.series_ohm == 0.0 {
            deck.push_str("Vpath strike protected 0\n");
        } else {
            writeln!(
                deck,
                "Rpath strike protected {:.16e}",
                self.network.series_ohm
            )
            .unwrap();
        }
        writeln!(deck, "Rload load 0 {:.16e}", self.network.pin_ohm).unwrap();
        if let Some(tvs) = self.tvs {
            // Независимый узловой расчёт исходной ВАХ, без обратной формулы V(I).
            writeln!(
                deck,
                "Bclamp strike 0 I={{({sign})*max(0, (({sign})*v(strike)-{:.16e})/{:.16e})}}",
                tvs.knee_v, tvs.dynamic_ohm,
            )
            .unwrap();
        }
        // По руководству ngspice, §1.4, критерии сходимости складывают
        // относительный и абсолютный допуски. Делим заявленный бюджет
        // пополам на эти слагаемые и ограничиваем снизу точностью f64.
        // Фактические ошибки ряда и пика проверяются отдельно.
        let peak = self.response.peak;
        let voltage_scale = peak.terminal_v.abs();
        let current_scale = peak.pin_a.abs() + peak.tvs_a.abs();
        let vntol = (options.voltage_tolerance_v / 2.0).max(voltage_scale * f64::EPSILON);
        let abstol = (options.current_tolerance_a / 2.0).max(current_scale * f64::EPSILON);
        let reltol = (vntol / voltage_scale).min(abstol / current_scale);
        writeln!(
            deck,
            ".options vntol={vntol:.16e} abstol={abstol:.16e} reltol={reltol:.16e}"
        )
        .unwrap();
        deck
    }
}

fn margin(peak: f64, error: f64, limit: &SourcedRange) -> Result<EsdMargin, ProcessError> {
    let high = nonnegative("верхняя огибающая остаточного воздействия", peak + error)?;
    Ok(EsdMargin {
        lower: limit.min - high,
        nominal: limit.nominal - peak,
        upper: limit.max - (peak - error).max(0.0),
    })
}

fn confirmed(scope: Scope, source: Option<&Provenance>) -> bool {
    source.is_some_and(|source| {
        let text = match source {
            Provenance::Datasheet(text) | Provenance::Measured(text) | Provenance::Paper(text) => {
                text
            }
            Provenance::Derived(text) if scope == Scope::Reference => text,
            _ => return false,
        };
        !text.trim().is_empty() && !text.chars().any(char::is_control)
    })
}

fn invalid(reason: impl Into<String>) -> ProcessError {
    ProcessError::InvalidInput(reason.into())
}

fn nonnegative(label: &str, value: f64) -> Result<f64, ProcessError> {
    if value.is_finite() && value >= 0.0 {
        Ok(value)
    } else {
        Err(invalid(format!(
            "{label}: требуется конечное неотрицательное число"
        )))
    }
}

fn positive(label: &str, value: f64) -> Result<f64, ProcessError> {
    if value.is_finite() && value > 0.0 {
        Ok(value)
    } else {
        Err(invalid(format!(
            "{label}: требуется конечное положительное число"
        )))
    }
}

fn resistance(label: &str, value: f64) -> Result<(), ProcessError> {
    if positive(label, value)?.recip().is_finite() {
        Ok(())
    } else {
        Err(invalid(format!(
            "{label}: проводимость вне числового диапазона"
        )))
    }
}

fn open<T>(reason: impl Into<String>) -> Analysis<T> {
    Analysis::NotAnalyzed {
        reason: reason.into(),
    }
}

fn analyzed<T>(value: T, scope: &str) -> Analysis<T> {
    Analysis::Analyzed {
        value,
        scope: scope.into(),
    }
}
