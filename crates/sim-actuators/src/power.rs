//! Сводка питания устройства: суммарный ток и мощность приводов, датчиков и плат,
//! хватает ли стабилизаторов и батареи.
//!
//! Допущения в опасную сторону:
//! * пики всех потребителей считаются совпавшими;
//! * рекуперация не засчитывается;
//! * ток батареи считается при напряжении разряженной батареи (постоянная мощность);
//! * нагрев линейного стабилизатора считается при заряженной батарее.
//!
//! Чего нет в данных, то `NotAnalyzed`: общий итог в этом случае не может быть `Pass`.

use crate::actuator::Actuator;
use crate::assess::{DutyCycle, Outcome, electrical_load_for_budget, resolve_driver};
use crate::driver::Driver;
use crate::regulator::{Regulator, RegulatorKind};
use crate::sensor::Sensor;
use crate::spec::Spec;

/// Батарея. Простые поля для формы.
#[derive(Clone, Debug, PartialEq)]
pub struct Battery {
    pub nominal_v: f64,
    /// Напряжение заряженной батареи.
    pub full_v: f64,
    /// Напряжение батареи без нагрузки к концу используемого диапазона разряда.
    pub empty_v: f64,
    /// Доступная ёмкость для рассматриваемых температуры и нагрузки, А·ч.
    pub capacity_ah: f64,
    pub max_continuous_a: Option<f64>,
    pub max_peak_a: Option<f64>,
    pub internal_resistance_ohm: Option<f64>,
}

/// Шина от стабилизатора, питается от батареи.
#[derive(Clone, Debug)]
pub struct RailInput<'a> {
    pub name: String,
    pub regulator: &'a Regulator,
    pub output_v: f64,
}

/// Приводы одного типа. `rail`: индекс шины, `None` — прямо от батареи.
#[derive(Clone, Debug)]
pub struct ActuatorLoad<'a> {
    pub actuator: &'a Actuator,
    pub driver: Option<&'a Driver>,
    pub duty: &'a DutyCycle,
    pub count: u32,
    pub rail: Option<usize>,
}

/// Датчики одного типа.
#[derive(Clone, Debug)]
pub struct SensorLoad<'a> {
    pub sensor: &'a Sensor,
    pub count: u32,
    pub rail: Option<usize>,
    /// Подтверждённый верхний ток одного датчика с обвязкой, А.
    pub current_override_a: Option<f64>,
}

/// Плата или другой потребитель с известным током.
#[derive(Clone, Debug, PartialEq)]
pub struct BoardLoad {
    pub name: String,
    /// Напряжение, для которого заданы токи. Для платы прямо на батарее используется
    /// модель постоянной мощности, допуск входа платы из этих данных неизвестен.
    pub voltage_v: f64,
    pub current_avg_a: f64,
    pub current_peak_a: f64,
    pub rail: Option<usize>,
}

/// Вход сводки питания.
#[derive(Clone, Debug)]
pub struct PowerInput<'a> {
    pub battery: Battery,
    pub rails: Vec<RailInput<'a>>,
    pub actuators: Vec<ActuatorLoad<'a>>,
    pub sensors: Vec<SensorLoad<'a>>,
    pub boards: Vec<BoardLoad>,
    pub ambient_c: f64,
    /// Требуемое время работы, с.
    pub required_runtime_s: Option<f64>,
}

/// Что проверяет сводка питания.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PowerCheck {
    InvalidInput,
    DataCompleteness,
    ConsumerVoltage,
    RailInputVoltage,
    RailOutputVoltage,
    RailCurrent,
    RailMinLoad,
    RailTemperature,
    BatteryContinuousCurrent,
    BatteryPeakCurrent,
    BatterySag,
    Runtime,
}

impl PowerCheck {
    pub fn title(self) -> &'static str {
        match self {
            PowerCheck::InvalidInput => "Корректность данных",
            PowerCheck::DataCompleteness => "Полнота данных",
            PowerCheck::ConsumerVoltage => "Напряжение потребителя",
            PowerCheck::RailInputVoltage => "Вход стабилизатора",
            PowerCheck::RailOutputVoltage => "Выход стабилизатора",
            PowerCheck::RailCurrent => "Ток стабилизатора",
            PowerCheck::RailMinLoad => "Минимальная нагрузка стабилизатора",
            PowerCheck::RailTemperature => "Нагрев стабилизатора",
            PowerCheck::BatteryContinuousCurrent => "Длительный ток батареи",
            PowerCheck::BatteryPeakCurrent => "Пиковый ток батареи",
            PowerCheck::BatterySag => "Просадка батареи",
            PowerCheck::Runtime => "Время работы",
        }
    }
}

/// Результат одной проверки питания.
#[derive(Clone, Debug, PartialEq)]
pub struct PowerFinding {
    pub kind: PowerCheck,
    pub outcome: Outcome,
    pub value: f64,
    pub limit: f64,
    pub unit: &'static str,
    /// К чему относится: имя шины, потребителя или «батарея».
    pub subject: String,
    pub detail: String,
}

/// Строка потребителя в сводке (суммарно на `count` штук).
#[derive(Clone, Debug, PartialEq)]
pub struct PowerLine {
    pub name: String,
    pub voltage_v: f64,
    pub avg_a: f64,
    pub peak_a: f64,
    pub avg_w: f64,
    pub peak_w: f64,
    /// Число включено в сумму, в том числе расчётное. Полноту данных показывает
    /// `PowerCheck::DataCompleteness`.
    pub known: bool,
    pub rail: Option<usize>,
}

/// Режим шины в сводке.
#[derive(Clone, Debug, PartialEq)]
pub struct RailSummary {
    pub name: String,
    pub output_v: f64,
    pub load_avg_a: f64,
    pub load_peak_a: f64,
    /// Мощность из батареи при разряженной батарее, Вт.
    pub input_avg_w: f64,
    pub input_peak_w: f64,
    /// Потери в стабилизаторе при средней нагрузке и худшем входе, Вт.
    pub loss_avg_w: f64,
}

/// Итог сводки питания.
#[derive(Clone, Debug, PartialEq)]
pub struct PowerBudget {
    pub lines: Vec<PowerLine>,
    pub rails: Vec<RailSummary>,
    /// Средний и пиковый ток батареи с учётом рассчитанной просадки, А.
    pub total_avg_a: f64,
    pub total_peak_a: f64,
    pub total_avg_w: f64,
    pub total_peak_w: f64,
    /// Время работы по энергии батареи, с.
    pub runtime_s: Option<f64>,
    pub findings: Vec<PowerFinding>,
    pub outcome: Outcome,
}

fn power_finding(
    kind: PowerCheck,
    outcome: Outcome,
    value: f64,
    limit: f64,
    unit: &'static str,
    subject: &str,
    detail: impl Into<String>,
) -> PowerFinding {
    let finite = value.is_finite() && limit.is_finite();
    PowerFinding {
        kind,
        outcome: if finite { outcome } else { Outcome::Fail },
        value: if value.is_finite() { value } else { 0.0 },
        limit: if limit.is_finite() { limit } else { 0.0 },
        unit,
        subject: subject.to_string(),
        detail: detail.into(),
    }
}

/// «value ≤ limit»: превышение — `Fail`, недоказанные данные — `NotAnalyzed`.
#[allow(clippy::too_many_arguments)]
fn at_most(
    kind: PowerCheck,
    value: f64,
    limit: f64,
    unit: &'static str,
    subject: &str,
    proof: bool,
    detail: String,
) -> PowerFinding {
    let outcome = if value > limit {
        Outcome::Fail
    } else if proof {
        Outcome::Pass
    } else {
        Outcome::NotAnalyzed
    };
    power_finding(kind, outcome, value, limit, unit, subject, detail)
}

/// «value ≥ limit».
#[allow(clippy::too_many_arguments)]
fn at_least(
    kind: PowerCheck,
    value: f64,
    limit: f64,
    unit: &'static str,
    subject: &str,
    proof: bool,
    detail: String,
) -> PowerFinding {
    let outcome = if value < limit {
        Outcome::Fail
    } else if proof {
        Outcome::Pass
    } else {
        Outcome::NotAnalyzed
    };
    power_finding(kind, outcome, value, limit, unit, subject, detail)
}

fn finite_pos(x: f64) -> bool {
    x.is_finite() && x > 0.0
}

fn finite_nonneg(x: f64) -> bool {
    x.is_finite() && x >= 0.0
}

fn validate(p: &PowerInput) -> Result<(), String> {
    let b = &p.battery;
    if !(finite_pos(b.nominal_v) && finite_pos(b.full_v) && finite_pos(b.empty_v)) {
        return Err("напряжения батареи должны быть положительными и конечными".into());
    }
    if !(b.empty_v <= b.nominal_v && b.nominal_v <= b.full_v) {
        return Err("у батареи должно быть: отсечка ≤ номинал ≤ заряд".into());
    }
    if !finite_pos(b.capacity_ah) {
        return Err("ёмкость батареи должна быть положительной и конечной".into());
    }
    for x in [b.max_continuous_a, b.max_peak_a, b.internal_resistance_ohm]
        .into_iter()
        .flatten()
    {
        if !finite_nonneg(x) {
            return Err("пределы батареи должны быть конечными и не меньше 0".into());
        }
    }
    if !(p.ambient_c.is_finite() && p.ambient_c > -273.15) {
        return Err("температура воздуха должна быть конечной и выше абсолютного нуля".into());
    }
    if let Some(t) = p.required_runtime_s
        && !finite_nonneg(t)
    {
        return Err("требуемое время работы должно быть конечным и не меньше 0".into());
    }
    let n = p.rails.len();
    let bad_rail = |r: Option<usize>| r.is_some_and(|i| i >= n);
    for r in &p.rails {
        if !finite_pos(r.output_v) {
            return Err(format!("шина {}: неверное выходное напряжение", r.name));
        }
        for (name, s) in r.regulator.numeric_specs() {
            if s.is_broken() || s.value.is_some_and(|v| v < 0.0 && !name.starts_with("t_")) {
                return Err(format!(
                    "{}: неверное число в поле {name}",
                    r.regulator.name
                ));
            }
        }
        let reg = r.regulator;
        if reg.max_duty.value.is_some_and(|f| !(0.0 < f && f <= 1.0))
            || reg
                .efficiency_fallback
                .value
                .is_some_and(|e| !(0.0 < e && e <= 1.0))
            || reg
                .efficiency_points
                .iter()
                .any(|&(v, e)| !finite_pos(v) || !(0.0 < e && e <= 1.0))
            || reg.efficiency_points.windows(2).any(|w| w[0].0 >= w[1].0)
        {
            return Err(format!("{}: неверные данные КПД или скважности", reg.name));
        }
    }
    for a in &p.actuators {
        if bad_rail(a.rail) {
            return Err(format!("{}: нет такой шины", a.actuator.name));
        }
        crate::assess::validate(a.actuator, a.driver, a.duty)?;
    }
    for s in &p.sensors {
        if bad_rail(s.rail) {
            return Err(format!("{}: нет такой шины", s.sensor.name));
        }
        if let Some(i) = s.current_override_a
            && !finite_nonneg(i)
        {
            return Err(format!("{}: неверный ток", s.sensor.name));
        }
        let sensor = s.sensor;
        if [sensor.current_a, sensor.current_peak_a]
            .into_iter()
            .any(|x| x.value.is_some_and(|i| !finite_nonneg(i)))
            || [
                sensor.range_min,
                sensor.range_max,
                sensor.resolution,
                sensor.accuracy,
            ]
            .into_iter()
            .any(Spec::is_broken)
            || sensor
                .supply_ranges
                .iter()
                .any(|&(lo, hi)| !finite_pos(lo) || !finite_pos(hi) || hi < lo)
        {
            return Err(format!("{}: некорректные данные датчика", sensor.name));
        }
    }
    for c in &p.boards {
        if bad_rail(c.rail) {
            return Err(format!("{}: нет такой шины", c.name));
        }
        if !(finite_pos(c.voltage_v)
            && finite_nonneg(c.current_avg_a)
            && finite_nonneg(c.current_peak_a))
        {
            return Err(format!("{}: неверные напряжение или ток", c.name));
        }
        if c.current_peak_a < c.current_avg_a {
            return Err(format!("{}: пиковый ток меньше среднего", c.name));
        }
    }
    Ok(())
}

/// Напряжение потребителя на шине или батарее против его допусков.
fn check_consumer_range(
    subject: &str,
    rail: Option<&RailInput>,
    battery: &Battery,
    min: Spec,
    max: Spec,
) -> PowerFinding {
    let (lo, hi) = match rail {
        Some(r) => (r.output_v, r.output_v),
        None => (battery.empty_v, battery.full_v),
    };
    if let Some(m) = max.get()
        && hi > m
    {
        return power_finding(
            PowerCheck::ConsumerVoltage,
            Outcome::Fail,
            hi,
            m,
            "В",
            subject,
            format!(
                "Питание до {hi:.2} В выше предела {m:.2} В ({}).",
                max.cite()
            ),
        );
    }
    if let Some(m) = min.get()
        && lo < m
    {
        return power_finding(
            PowerCheck::ConsumerVoltage,
            Outcome::Fail,
            lo,
            m,
            "В",
            subject,
            format!(
                "Питание от {lo:.2} В ниже минимума {m:.2} В ({}).",
                min.cite()
            ),
        );
    }
    match max.get() {
        Some(m) => at_most(
            PowerCheck::ConsumerVoltage,
            hi,
            m,
            "В",
            subject,
            max.is_proof() && min.is_proof(),
            format!("Питание {lo:.2}–{hi:.2} В в пределах."),
        ),
        None => power_finding(
            PowerCheck::ConsumerVoltage,
            Outcome::NotAnalyzed,
            hi,
            0.0,
            "В",
            subject,
            "Верхний предел напряжения не опубликован.",
        ),
    }
}

fn invalid(why: String) -> PowerBudget {
    PowerBudget {
        lines: Vec::new(),
        rails: Vec::new(),
        total_avg_a: 0.0,
        total_peak_a: 0.0,
        total_avg_w: 0.0,
        total_peak_w: 0.0,
        runtime_s: None,
        findings: vec![power_finding(
            PowerCheck::InvalidInput,
            Outcome::Fail,
            0.0,
            0.0,
            "",
            "вход",
            format!("Некорректный вход: {why}."),
        )],
        outcome: Outcome::Fail,
    }
}

#[derive(Clone, Copy, Default)]
struct Sum {
    avg_a: f64,
    peak_a: f64,
    avg_w: f64,
    peak_w: f64,
}

impl Sum {
    fn add(&mut self, l: &PowerLine) {
        self.avg_a += l.avg_a;
        self.peak_a += l.peak_a;
        self.avg_w += l.avg_w;
        self.peak_w += l.peak_w;
    }
}

/// Собрать сводку питания устройства.
pub fn power_budget(input: &PowerInput) -> PowerBudget {
    if let Err(why) = validate(input) {
        return invalid(why);
    }
    let b = &input.battery;
    let mut findings = Vec::new();
    let mut lines: Vec<PowerLine> = Vec::new();
    let mut complete = true;
    let mut required_v: f64 = 0.0;

    // --- приводы ---
    for load in &input.actuators {
        let a = load.actuator;
        let n = load.count as f64;
        let rail = load.rail.map(|i| &input.rails[i]);
        let v_duty = load.duty.supply_voltage_v;
        let subject = a.name;
        if !finite_pos(v_duty) {
            return invalid(format!("{}: неверное напряжение цикла", a.name));
        }
        // Цикл должен быть посчитан при том напряжении, которое реально на шине.
        let (lo, hi) = match rail {
            Some(r) => (r.output_v, r.output_v),
            None => (b.empty_v, b.full_v),
        };
        if v_duty < lo || v_duty > hi {
            findings.push(power_finding(
                PowerCheck::ConsumerVoltage,
                Outcome::Fail,
                v_duty,
                if v_duty < lo { lo } else { hi },
                "В",
                subject,
                format!("Цикл привода задан при {v_duty:.2} В, а на его шине {lo:.2}–{hi:.2} В."),
            ));
        }
        findings.push(check_consumer_range(
            subject,
            rail,
            b,
            a.voltage_min_v,
            a.voltage_max_v,
        ));
        let (drv, driver_problem) = resolve_driver(a, load.driver);
        if let Some(problem) = driver_problem {
            findings.push(power_finding(
                PowerCheck::DataCompleteness,
                Outcome::Fail,
                0.0,
                0.0,
                "",
                subject,
                problem,
            ));
        }
        if let Some(d) = drv {
            findings.push(check_consumer_range(
                d.name,
                rail,
                b,
                d.voltage_min_v,
                d.voltage_max_v,
            ));
        }
        let mut electrical = match electrical_load_for_budget(a, load.driver, load.duty) {
            Ok(load) => load,
            Err(why) => return invalid(format!("{subject}: {why}")),
        };
        // На прямой линии ток и мощность могут измениться по мере разряда.
        // Проверяем оба края отдельно и берём наибольшие результаты.
        if rail.is_none() {
            let mut duty = load.duty.clone();
            for volts in [lo, hi] {
                if volts == v_duty {
                    continue;
                }
                duty.supply_voltage_v = volts;
                match (
                    electrical,
                    electrical_load_for_budget(a, load.driver, &duty),
                ) {
                    (Some((mut e, _)), Ok(Some((edge, _)))) => {
                        e.power_avg_w = e.power_avg_w.max(edge.power_avg_w);
                        e.power_peak_w = e.power_peak_w.max(edge.power_peak_w);
                        e.bus_current_avg_a = e.bus_current_avg_a.max(edge.bus_current_avg_a);
                        e.bus_current_peak_a = e.bus_current_peak_a.max(edge.bus_current_peak_a);
                        electrical = Some((e, false));
                    }
                    (_, Err(why)) => return invalid(format!("{subject}: {why}")),
                    _ => electrical = None,
                }
            }
        }
        if load.count > 0 && !electrical.is_some_and(|(_, proven)| proven) {
            complete = false;
            findings.push(power_finding(
                PowerCheck::DataCompleteness,
                Outcome::NotAnalyzed,
                0.0,
                0.0,
                "А",
                subject,
                "Верхняя граница потребления привода и драйвера на шине не подтверждена.",
            ));
        }
        match electrical {
            Some((e, _)) => lines.push(PowerLine {
                name: format!("{} × {}", a.name, load.count),
                voltage_v: v_duty,
                avg_a: n * e.bus_current_avg_a,
                peak_a: n * e.bus_current_peak_a,
                avg_w: n * e.power_avg_w,
                peak_w: n * e.power_peak_w,
                known: true,
                rail: load.rail,
            }),
            None => lines.push(PowerLine {
                name: format!("{} × {}", a.name, load.count),
                voltage_v: v_duty,
                avg_a: 0.0,
                peak_a: 0.0,
                avg_w: 0.0,
                peak_w: 0.0,
                known: false,
                rail: load.rail,
            }),
        }
    }

    // --- датчики ---
    for load in &input.sensors {
        let s = load.sensor;
        let n = load.count as f64;
        let rail = load.rail.map(|i| &input.rails[i]);
        let (lo, hi) = match rail {
            Some(r) => (r.output_v, r.output_v),
            None => (b.empty_v, b.full_v),
        };
        if s.supply_ranges.is_empty() {
            findings.push(power_finding(
                PowerCheck::ConsumerVoltage,
                Outcome::NotAnalyzed,
                hi,
                0.0,
                "В",
                s.name,
                "Диапазон питания не нормирован производителем.",
            ));
        } else if let Some(&(min, _)) = s.supply_ranges.iter().find(|&&(a, c)| lo >= a && hi <= c) {
            if rail.is_none() {
                required_v = required_v.max(min);
            }
            findings.push(power_finding(
                PowerCheck::ConsumerVoltage,
                Outcome::Pass,
                hi,
                hi,
                "В",
                s.name,
                format!("Питание {lo:.2}–{hi:.2} В в допустимом диапазоне."),
            ));
        } else {
            findings.push(power_finding(
                PowerCheck::ConsumerVoltage,
                Outcome::Fail,
                hi,
                s.supply_ranges.iter().map(|r| r.1).fold(0.0, f64::max),
                "В",
                s.name,
                format!(
                    "Питание {lo:.2}–{hi:.2} В не попадает в допустимые диапазоны {:?} В.",
                    s.supply_ranges
                ),
            ));
        }
        let peak_spec = s.current_peak_a;
        let observed = s.worst_current_a();
        let peak = match load.current_override_a {
            Some(i) => Some(observed.map_or(i, |published| published.max(i))),
            None => observed,
        };
        let avg = load.current_override_a.or(s.current_a.get()).or(peak);
        if load.count > 0 && load.current_override_a.is_none() && !peak_spec.is_proof() {
            complete = false;
            findings.push(power_finding(
                PowerCheck::DataCompleteness,
                Outcome::NotAnalyzed,
                0.0,
                0.0,
                "А",
                s.name,
                "Верхний ток датчика с обвязкой не указан.",
            ));
        }
        let v = hi;
        match (avg, peak) {
            (Some(i_avg), Some(i_peak)) => lines.push(PowerLine {
                name: format!("{} × {}", s.name, load.count),
                voltage_v: v,
                avg_a: n * i_avg,
                peak_a: n * i_peak.max(i_avg),
                avg_w: n * i_avg * v,
                peak_w: n * i_peak.max(i_avg) * v,
                known: true,
                rail: load.rail,
            }),
            _ => lines.push(PowerLine {
                name: format!("{} × {}", s.name, load.count),
                voltage_v: v,
                avg_a: 0.0,
                peak_a: 0.0,
                avg_w: 0.0,
                peak_w: 0.0,
                known: false,
                rail: load.rail,
            }),
        }
    }

    // --- платы ---
    for c in &input.boards {
        let matches = match c.rail {
            Some(i) => c.voltage_v == input.rails[i].output_v,
            None => c.voltage_v >= b.empty_v && c.voltage_v <= b.full_v,
        };
        let proven =
            c.rail.is_some() || (b.empty_v == b.full_v && b.internal_resistance_ohm == Some(0.0));
        findings.push(power_finding(
            PowerCheck::ConsumerVoltage,
            if !matches {
                Outcome::Fail
            } else if proven {
                Outcome::Pass
            } else {
                Outcome::NotAnalyzed
            },
            c.voltage_v,
            c.rail.map_or(b.full_v, |i| input.rails[i].output_v),
            "В",
            &c.name,
            if matches && !proven {
                "Для платы на батарее нужен допустимый диапазон её входного напряжения.".to_string()
            } else if matches {
                "Напряжение платы совпадает с шиной.".to_string()
            } else {
                format!(
                    "Плата рассчитана на {:.2} В, а шина даёт другое.",
                    c.voltage_v
                )
            },
        ));
        lines.push(PowerLine {
            name: c.name.clone(),
            voltage_v: c.voltage_v,
            avg_a: c.current_avg_a,
            peak_a: c.current_peak_a,
            avg_w: c.current_avg_a * c.voltage_v,
            peak_w: c.current_peak_a * c.voltage_v,
            known: true,
            rail: c.rail,
        });
    }

    // --- шины ---
    let mut direct = Sum::default();
    for l in lines.iter().filter(|l| l.rail.is_none()) {
        direct.add(l);
    }
    let mut total_avg_w = direct.avg_w;
    let mut total_peak_w = direct.peak_w;
    let mut proven = complete;
    let mut runtime_power_w = direct.avg_w;
    let mut rails = Vec::new();

    for (j, r) in input.rails.iter().enumerate() {
        let reg = r.regulator;
        let subject = r.name.as_str();
        let mut sum = Sum::default();
        for l in lines.iter().filter(|l| l.rail == Some(j)) {
            sum.add(l);
        }
        let need = reg.required_input_v(r.output_v);
        if !need.is_finite() {
            return invalid(format!("{subject}: требуемое напряжение шины не конечное"));
        }
        required_v = required_v.max(need);
        if let Some(m) = reg.input_abs_max_v.get()
            && b.full_v > m
        {
            findings.push(power_finding(
                PowerCheck::RailInputVoltage,
                Outcome::Fail,
                b.full_v,
                m,
                "В",
                subject,
                format!("{}: батарея выше абсолютного максимума входа.", reg.name),
            ));
        }
        if let Some(m) = reg.input_max_v.get() {
            findings.push(at_most(
                PowerCheck::RailInputVoltage,
                b.full_v,
                m,
                "В",
                subject,
                reg.input_max_v.is_proof(),
                format!(
                    "{}: заряженная батарея {:.2} В при пределе входа {m:.2} В.",
                    reg.name, b.full_v
                ),
            ));
        } else {
            findings.push(power_finding(
                PowerCheck::RailInputVoltage,
                Outcome::NotAnalyzed,
                b.full_v,
                0.0,
                "В",
                subject,
                "Рабочий верхний предел входа стабилизатора не указан.",
            ));
        }
        let headroom_proven = reg.input_min_v.is_proof()
            && match reg.kind {
                RegulatorKind::Linear => reg.dropout_v.is_proof(),
                RegulatorKind::Buck => reg.dropout_v.is_proof() && reg.max_duty.is_proof(),
            };
        findings.push(at_least(
            PowerCheck::RailInputVoltage,
            b.empty_v,
            need,
            "В",
            subject,
            headroom_proven,
            format!(
                "{}: разряженная батарея {:.2} В, для {:.2} В на выходе нужно не меньше {need:.2} В.",
                reg.name, b.empty_v, r.output_v
            ),
        ));
        let range_ok = reg.output_min_v.get().is_none_or(|m| r.output_v >= m)
            && reg.output_max_v.get().is_none_or(|m| r.output_v <= m);
        let output_proven = reg.output_min_v.is_proof() && reg.output_max_v.is_proof();
        findings.push(power_finding(
            PowerCheck::RailOutputVoltage,
            if !range_ok {
                Outcome::Fail
            } else if output_proven {
                Outcome::Pass
            } else {
                Outcome::NotAnalyzed
            },
            r.output_v,
            reg.output_max_v.get().unwrap_or(r.output_v),
            "В",
            subject,
            if range_ok && !output_proven {
                "Допустимый диапазон выходного напряжения подтверждён не полностью.".into()
            } else if range_ok {
                format!("{} может выдать {:.2} В.", reg.name, r.output_v)
            } else {
                format!("{} не выдаёт {:.2} В.", reg.name, r.output_v)
            },
        ));
        match reg.output_current_a.get() {
            Some(m) => findings.push(at_most(
                PowerCheck::RailCurrent,
                sum.peak_a,
                m,
                "А",
                subject,
                reg.output_current_a.is_proof() && complete,
                format!(
                    "{}: пиковая нагрузка {:.3} А при пределе {m:.3} А.",
                    reg.name, sum.peak_a
                ),
            )),
            None => findings.push(power_finding(
                PowerCheck::RailCurrent,
                Outcome::NotAnalyzed,
                sum.peak_a,
                0.0,
                "А",
                subject,
                "Выходной ток не опубликован.",
            )),
        }
        if let Some(m) = reg.min_load_a.get() {
            let outcome = if !complete || (sum.avg_a >= m && sum.avg_a != sum.peak_a) {
                Outcome::NotAnalyzed
            } else if sum.avg_a < m {
                Outcome::Fail
            } else if reg.min_load_a.is_proof() {
                Outcome::Pass
            } else {
                Outcome::NotAnalyzed
            };
            findings.push(power_finding(
                PowerCheck::RailMinLoad,
                outcome,
                sum.avg_a,
                m,
                "А",
                subject,
                format!(
                    "{}: средняя нагрузка {:.4} А, для стабилизации нужен минимум {m:.4} А во всех режимах.",
                    reg.name, sum.avg_a
                ),
            ));
        }
        // Ток из батареи — при разряженной, нагрев линейного — при заряженной.
        let in_avg = reg.operating(b.empty_v, r.output_v, sum.avg_a);
        let in_peak = reg.operating(b.empty_v, r.output_v, sum.peak_a);
        let v_hot = if reg.kind == RegulatorKind::Linear {
            b.full_v
        } else {
            b.empty_v
        };
        let hot_avg = reg.operating(v_hot, r.output_v, sum.avg_a);
        let hot_peak = reg.operating(v_hot, r.output_v, sum.peak_a);
        let (mut input_avg_w, mut input_peak_w, mut loss_avg_w) = (sum.avg_w, sum.peak_w, 0.0);
        if let (Some(a), Some(p)) = (in_avg, in_peak) {
            input_avg_w = a.input_power_w;
            input_peak_w = p.input_power_w;
            proven &= a.proven && p.proven;
        } else {
            proven = false;
        }
        total_avg_w += input_avg_w;
        total_peak_w += input_peak_w;
        runtime_power_w += hot_avg.map_or(input_avg_w, |op| input_avg_w.max(op.input_power_w));
        match (
            hot_avg,
            hot_peak,
            reg.r_theta_ja_k_per_w.get(),
            reg.junction_limit_c(),
        ) {
            (Some(a), Some(p), Some(theta), Some(lim)) => {
                loss_avg_w = a.loss_w;
                let t_mean = input.ambient_c + theta * a.loss_w;
                let t_worst = input.ambient_c + theta * p.loss_w;
                let proof = a.proven
                    && p.proven
                    && complete
                    && reg.r_theta_ja_k_per_w.is_proof()
                    && reg.t_j_max_c.is_proof();
                let f = if t_mean > lim {
                    power_finding(
                        PowerCheck::RailTemperature,
                        Outcome::Fail,
                        t_mean,
                        lim,
                        "°C",
                        subject,
                        format!(
                            "{}: кристалл нагреется до {t_mean:.0} °C при пределе {lim:.0} °C (потери {:.2} Вт).",
                            reg.name, a.loss_w
                        ),
                    )
                } else if t_worst <= lim {
                    at_most(
                        PowerCheck::RailTemperature,
                        t_worst,
                        lim,
                        "°C",
                        subject,
                        proof,
                        format!(
                            "{}: кристалл до {t_worst:.0} °C при пределе {lim:.0} °C.",
                            reg.name
                        ),
                    )
                } else {
                    power_finding(
                        PowerCheck::RailTemperature,
                        Outcome::NotAnalyzed,
                        t_worst,
                        lim,
                        "°C",
                        subject,
                        format!(
                            "{}: в среднем {t_mean:.0} °C, на пике до {t_worst:.0} °C при пределе {lim:.0} °C; тепловая инерция корпуса не опубликована.",
                            reg.name
                        ),
                    )
                };
                findings.push(f);
            }
            _ => findings.push(power_finding(
                PowerCheck::RailTemperature,
                Outcome::NotAnalyzed,
                0.0,
                0.0,
                "°C",
                subject,
                "Нет данных для расчёта нагрева стабилизатора.",
            )),
        }
        rails.push(RailSummary {
            name: r.name.clone(),
            output_v: r.output_v,
            load_avg_a: sum.avg_a,
            load_peak_a: sum.peak_a,
            input_avg_w,
            input_peak_w,
            loss_avg_w,
        });
    }

    // Минимальное напряжение прямых потребителей — для проверки просадки.
    for load in input.actuators.iter().filter(|l| l.rail.is_none()) {
        if let Some(m) = load.actuator.voltage_min_v.get() {
            required_v = required_v.max(m);
        }
        if let (Some(d), _) = resolve_driver(load.actuator, load.driver)
            && let Some(m) = d.voltage_min_v.get()
        {
            required_v = required_v.max(m);
        }
    }

    // --- батарея ---
    let peak_a_at_empty = total_peak_w / b.empty_v;
    let subject = "батарея";
    let loaded_v = if let Some(r) = b.internal_resistance_ohm {
        // Для постоянной мощности P = U·(U_OC − U)/R, устойчивая ветвь
        // U = U_OC/2·(1 + √(1 − 4PR/U_OC²)). При отрицательном дискриминанте
        // источник не может выдать требуемую мощность даже при идеальной обвязке.
        let ratio = peak_a_at_empty * (r / b.empty_v);
        if !ratio.is_finite() {
            return invalid("расчёт просадки батареи не конечен".into());
        }
        let discriminant = 1.0 - 4.0 * ratio;
        if !discriminant.is_finite() {
            return invalid("дискриминант просадки батареи не конечен".into());
        }
        if discriminant <= 0.0 {
            findings.push(power_finding(
                PowerCheck::BatterySag,
                Outcome::Fail,
                total_peak_w,
                (b.empty_v / r) * (b.empty_v / 4.0),
                "Вт",
                subject,
                "Под пиком батарея не имеет устойчивой рабочей точки.",
            ));
            None
        } else {
            let loaded = b.empty_v * (0.5 * (1.0 + discriminant.sqrt()));
            let known_direct_board_range = input
                .boards
                .iter()
                .all(|c| c.rail.is_some() || (loaded == c.voltage_v && b.full_v == c.voltage_v));
            findings.push(at_least(
                PowerCheck::BatterySag,
                loaded,
                required_v,
                "В",
                subject,
                proven && known_direct_board_range,
                format!("Под пиком {loaded:.2} В, потребителям нужно не меньше {required_v:.2} В."),
            ));
            Some(loaded)
        }
    } else {
        findings.push(power_finding(
            PowerCheck::BatterySag,
            Outcome::NotAnalyzed,
            b.empty_v,
            required_v,
            "В",
            subject,
            "Внутреннее сопротивление батареи не задано: просадка не проверена.",
        ));
        None
    };
    let sizing_voltage = loaded_v.unwrap_or(b.empty_v);
    let total_avg_a = total_avg_w / sizing_voltage;
    let total_peak_a = total_peak_w / sizing_voltage;
    match b.max_continuous_a {
        Some(m) => findings.push(at_most(
            PowerCheck::BatteryContinuousCurrent,
            total_avg_a,
            m,
            "А",
            subject,
            proven && loaded_v.is_some() && total_peak_a <= m,
                format!("Средний ток {total_avg_a:.3} А, пик {total_peak_a:.3} А, длительный предел {m:.3} А."),
        )),
        None => findings.push(power_finding(
            PowerCheck::BatteryContinuousCurrent,
            Outcome::NotAnalyzed,
            total_avg_a,
            0.0,
            "А",
            subject,
            "Длительный ток батареи не задан.",
        )),
    }
    match b.max_peak_a {
        Some(m) => findings.push(at_most(
            PowerCheck::BatteryPeakCurrent,
            total_peak_a,
            m,
            "А",
            subject,
            proven && loaded_v.is_some(),
            format!("Пиковый ток {total_peak_a:.3} А (все пики совпали) при пределе {m:.3} А."),
        )),
        None => findings.push(power_finding(
            PowerCheck::BatteryPeakCurrent,
            Outcome::NotAnalyzed,
            total_peak_a,
            0.0,
            "А",
            subject,
            "Пиковый ток батареи не задан.",
        )),
    }
    // Энергия снизу по напряжению конца разряда под нагрузкой, потребление
    // сверху по потерям стабилизаторов. 3600 с в часе.
    let energy_j = b.capacity_ah * 3600.0 * loaded_v.unwrap_or(b.empty_v);
    let runtime_s = (runtime_power_w > 0.0).then(|| energy_j / runtime_power_w);
    if [
        total_avg_a,
        total_peak_a,
        total_avg_w,
        total_peak_w,
        runtime_power_w,
        energy_j,
    ]
    .into_iter()
    .any(|x| !x.is_finite())
        || runtime_s.is_some_and(|t| !t.is_finite())
        || lines.iter().any(|l| {
            ![l.avg_a, l.peak_a, l.avg_w, l.peak_w]
                .into_iter()
                .all(f64::is_finite)
        })
        || rails.iter().any(|r| {
            ![r.load_avg_a, r.load_peak_a, r.loss_avg_w]
                .into_iter()
                .all(f64::is_finite)
        })
    {
        return invalid("сумма токов, мощностей или энергии не конечна".into());
    }
    if let Some(need) = input.required_runtime_s {
        match runtime_s {
            Some(t) => findings.push(at_least(
                PowerCheck::Runtime,
                t,
                need,
                "с",
                subject,
                proven && loaded_v.is_some(),
                format!("Энергии хватит на {t:.0} с, нужно {need:.0} с."),
            )),
            None => findings.push(power_finding(
                PowerCheck::Runtime,
                Outcome::NotAnalyzed,
                0.0,
                need,
                "с",
                subject,
                "Нагрузка не определена: время работы не посчитано.",
            )),
        }
    }

    let outcome = if findings.iter().any(|f| f.outcome == Outcome::Fail) {
        Outcome::Fail
    } else if findings.iter().any(|f| f.outcome == Outcome::NotAnalyzed) || findings.is_empty() {
        Outcome::NotAnalyzed
    } else {
        Outcome::Pass
    };
    PowerBudget {
        lines,
        rails,
        total_avg_a,
        total_peak_a,
        total_avg_w,
        total_peak_w,
        runtime_s,
        findings,
        outcome,
    }
}
