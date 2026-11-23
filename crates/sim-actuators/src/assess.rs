//! Оценка привода для вердикта: выдержит ли деталь заданный цикл работы.
//!
//! Правила, от которых зависит цена ошибки:
//! * NaN и бесконечность в любом входе или результате — `Fail`;
//! * нет данных — `NotAnalyzed`, и он никогда не превращается в `Pass`;
//! * значение-«оценка» может доказать `Fail`, но не `Pass`;
//! * рабочая точка считается по прямой «момент–скорость» при фактическом напряжении на
//!   зажимах: питание минус падение в драйвере, обмотка горячая.

use crate::actuator::{Actuator, ActuatorKind, Backdrive, Brake, MotorLine, MotorType};
use crate::driver::{Driver, DriverKind};
use crate::spec::{Spec, T_REF_C, UNKNOWN_HOT_WINDING_C};
use crate::thermal::{HeatSegment, ThermalNet, periodic_peak_winding_c, steady_winding_c};

/// Итог одной проверки.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    Pass,
    Fail,
    NotAnalyzed,
}

impl Outcome {
    pub fn title(self) -> &'static str {
        match self {
            Outcome::Pass => "годится",
            Outcome::Fail => "не годится",
            Outcome::NotAnalyzed => "не проверено",
        }
    }
}

/// Что проверяем.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ActuatorCheck {
    PeakTorque,
    ContinuousTorque,
    Speed,
    Voltage,
    Current,
    DriverCompatibility,
    DriverCurrent,
    DriverVoltage,
    DriverTemperature,
    WindingTemperature,
    AmbientTemperature,
    HoldWhenUnpowered,
}

impl ActuatorCheck {
    /// Все проверки в порядке вывода.
    pub const ALL: [ActuatorCheck; 12] = [
        ActuatorCheck::PeakTorque,
        ActuatorCheck::ContinuousTorque,
        ActuatorCheck::Speed,
        ActuatorCheck::Voltage,
        ActuatorCheck::Current,
        ActuatorCheck::DriverCompatibility,
        ActuatorCheck::DriverCurrent,
        ActuatorCheck::DriverVoltage,
        ActuatorCheck::DriverTemperature,
        ActuatorCheck::WindingTemperature,
        ActuatorCheck::AmbientTemperature,
        ActuatorCheck::HoldWhenUnpowered,
    ];

    pub fn title(self) -> &'static str {
        match self {
            ActuatorCheck::PeakTorque => "Пиковый момент",
            ActuatorCheck::ContinuousTorque => "Длительный момент",
            ActuatorCheck::Speed => "Скорость при нагрузке",
            ActuatorCheck::Voltage => "Напряжение привода",
            ActuatorCheck::Current => "Ток привода",
            ActuatorCheck::DriverCompatibility => "Совместимость драйвера",
            ActuatorCheck::DriverCurrent => "Ток драйвера",
            ActuatorCheck::DriverVoltage => "Напряжение драйвера",
            ActuatorCheck::DriverTemperature => "Нагрев драйвера",
            ActuatorCheck::WindingTemperature => "Температура обмотки",
            ActuatorCheck::AmbientTemperature => "Температура воздуха",
            ActuatorCheck::HoldWhenUnpowered => "Удержание без питания",
        }
    }
}

/// Результат одной проверки. При `NotAnalyzed` без данных `value` и `limit` равны 0:
/// смысл несёт `outcome` и `detail`.
#[derive(Clone, Debug, PartialEq)]
pub struct ActuatorFinding {
    pub kind: ActuatorCheck,
    pub outcome: Outcome,
    pub value: f64,
    pub limit: f64,
    pub unit: &'static str,
    pub detail: String,
}

/// Участок цикла: момент и скорость на выходном валу, длительность.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DutySegment {
    pub torque_nm: f64,
    pub speed_rad_s: f64,
    pub duration_s: f64,
}

/// Цикл работы привода. Простые поля — заполняется прямо из формы.
#[derive(Clone, Debug, PartialEq)]
pub struct DutyCycle {
    pub segments: Vec<DutySegment>,
    pub ambient_c: f64,
    pub supply_voltage_v: f64,
    /// Момент, который надо держать без питания: `None` — не указано,
    /// `Some(0.0)` — держать не нужно.
    pub unpowered_hold_torque_nm: Option<f64>,
}

impl DutyCycle {
    pub fn new(supply_voltage_v: f64, ambient_c: f64) -> Self {
        Self {
            segments: Vec::new(),
            ambient_c,
            supply_voltage_v,
            unpowered_hold_torque_nm: None,
        }
    }

    /// Добавить участок.
    pub fn segment(mut self, torque_nm: f64, speed_rad_s: f64, duration_s: f64) -> Self {
        self.segments.push(DutySegment {
            torque_nm,
            speed_rad_s,
            duration_s,
        });
        self
    }

    /// Указать момент удержания без питания (0 — удерживать не нужно).
    pub fn hold_unpowered(mut self, torque_nm: f64) -> Self {
        self.unpowered_hold_torque_nm = Some(torque_nm);
        self
    }

    pub fn total_time_s(&self) -> f64 {
        self.segments.iter().map(|s| s.duration_s).sum()
    }

    /// Среднеквадратичный момент цикла, Н·м.
    pub fn rms_torque_nm(&self) -> f64 {
        let t = self.total_time_s();
        let sum: f64 = self
            .segments
            .iter()
            .map(|s| s.torque_nm * s.torque_nm * s.duration_s)
            .sum();
        (sum / t).sqrt()
    }

    pub fn peak_torque_nm(&self) -> f64 {
        self.segments
            .iter()
            .map(|s| s.torque_nm.abs())
            .fold(0.0, f64::max)
    }

    pub fn peak_speed_rad_s(&self) -> f64 {
        self.segments
            .iter()
            .map(|s| s.speed_rad_s.abs())
            .fold(0.0, f64::max)
    }
}

/// Общий итог: `Fail`, если есть хоть один; иначе `NotAnalyzed`, если что-то не проверено
/// или проверок нет; `Pass` — только когда пройдены все.
pub fn overall(findings: &[ActuatorFinding]) -> Outcome {
    if findings.is_empty() {
        return Outcome::NotAnalyzed;
    }
    if findings
        .iter()
        .any(|f| f.outcome == Outcome::Fail || !f.value.is_finite() || !f.limit.is_finite())
    {
        return Outcome::Fail;
    }
    if findings.iter().any(|f| f.outcome == Outcome::NotAnalyzed)
        || ActuatorCheck::ALL
            .iter()
            .any(|k| !findings.iter().any(|f| f.kind == *k))
    {
        return Outcome::NotAnalyzed;
    }
    Outcome::Pass
}

/// Электрическая нагрузка привода на шину питания за цикл.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ElectricalLoad {
    /// Среднеквадратичный ток обмотки, А.
    pub current_rms_a: f64,
    /// Пиковый ток обмотки, А.
    pub current_peak_a: f64,
    /// Средняя мощность из шины, Вт (рекуперация не уменьшает потребление других участков).
    pub power_avg_w: f64,
    /// Пиковая мощность из шины, Вт.
    pub power_peak_w: f64,
    /// Средний и пиковый ток из шины при напряжении цикла, А.
    pub bus_current_avg_a: f64,
    pub bus_current_peak_a: f64,
}

const NO_DRIVER: &str = "драйвер не выбран";

fn finding(
    kind: ActuatorCheck,
    outcome: Outcome,
    value: f64,
    limit: f64,
    unit: &'static str,
    detail: impl Into<String>,
) -> ActuatorFinding {
    let detail = detail.into();
    if !value.is_finite() || !limit.is_finite() {
        return ActuatorFinding {
            kind,
            outcome: Outcome::Fail,
            value: if value.is_finite() { value } else { 0.0 },
            limit: if limit.is_finite() { limit } else { 0.0 },
            unit,
            detail: format!("Результат расчёта не является конечным числом. {detail}"),
        };
    }
    ActuatorFinding {
        kind,
        outcome,
        value,
        limit,
        unit,
        detail,
    }
}

fn not_analyzed(
    kind: ActuatorCheck,
    value: f64,
    unit: &'static str,
    detail: impl Into<String>,
) -> ActuatorFinding {
    finding(kind, Outcome::NotAnalyzed, value, 0.0, unit, detail)
}

fn check_spec_list(
    who: &str,
    specs: &[(&'static str, Spec)],
    temperatures: &[(&'static str, Spec)],
) -> Result<(), String> {
    for (name, s) in specs {
        if s.is_broken() {
            return Err(format!("{who}: поле {name} — не конечное число"));
        }
        if s.value.is_some_and(|v| v < 0.0) {
            return Err(format!("{who}: поле {name} отрицательное"));
        }
    }
    for (name, s) in temperatures {
        if s.is_broken() {
            return Err(format!("{who}: поле {name} — не конечное число"));
        }
    }
    Ok(())
}

pub(crate) fn validate(
    actuator: &Actuator,
    driver: Option<&Driver>,
    duty: &DutyCycle,
) -> Result<(), String> {
    if !(duty.supply_voltage_v.is_finite() && duty.supply_voltage_v > 0.0) {
        return Err("напряжение питания должно быть положительным конечным числом".into());
    }
    if !(duty.ambient_c.is_finite() && duty.ambient_c > -273.15) {
        return Err("температура воздуха должна быть конечной и выше абсолютного нуля".into());
    }
    if duty.segments.is_empty() {
        return Err("в цикле нет ни одного участка".into());
    }
    for (j, s) in duty.segments.iter().enumerate() {
        if !(s.torque_nm.is_finite() && s.speed_rad_s.is_finite() && s.duration_s.is_finite()) {
            return Err(format!("участок {}: не конечное число", j + 1));
        }
        if s.duration_s <= 0.0 {
            return Err(format!(
                "участок {}: длительность должна быть больше 0",
                j + 1
            ));
        }
        if !(s.torque_nm * s.speed_rad_s).is_finite() {
            return Err(format!(
                "участок {}: механическая мощность не конечна",
                j + 1
            ));
        }
    }
    if !duty.total_time_s().is_finite() || !duty.rms_torque_nm().is_finite() {
        return Err("цикл слишком велик: сумма по участкам не конечна".into());
    }
    if let Some(t) = duty.unpowered_hold_torque_nm
        && !(t.is_finite() && t >= 0.0)
    {
        return Err("момент удержания без питания должен быть конечным и не меньше 0".into());
    }
    check_spec_list(
        actuator.name,
        &actuator.numeric_specs(),
        &actuator.temperature_specs(),
    )?;
    if !(actuator.heated_phases.is_finite() && actuator.heated_phases > 0.0) {
        return Err(format!("{}: неверное число греющихся фаз", actuator.name));
    }
    if actuator
        .stepper_pole_pairs
        .is_some_and(|n| !n.is_finite() || n <= 0.0)
        || actuator
            .gear_efficiency
            .get()
            .is_some_and(|e| e <= 0.0 || e > 1.0)
        || actuator.numeric_specs().iter().any(|(name, spec)| {
            matches!(
                *name,
                "resistance_ohm"
                    | "kt_motor_nm_per_a"
                    | "ke_motor_v_s"
                    | "kv_rpm_per_v"
                    | "gear_ratio"
                    | "rated_current_a"
                    | "tau_winding_s"
                    | "tau_housing_s"
                    | "r_th_winding_housing_k_per_w"
                    | "r_th_housing_ambient_k_per_w"
            ) && spec.get() == Some(0.0)
        })
    {
        return Err(format!(
            "{}: неверные параметры модели двигателя",
            actuator.name
        ));
    }
    for p in actuator.voltage_points {
        let ok = [p.voltage_v, p.stall_torque_nm, p.no_load_rpm]
            .iter()
            .all(|x| x.is_finite() && *x > 0.0)
            && p.stall_current_a.is_none_or(|i| i.is_finite() && i > 0.0);
        if !ok {
            return Err(format!("{}: неверная паспортная точка", actuator.name));
        }
    }
    if actuator
        .voltage_points
        .windows(2)
        .any(|w| w[0].voltage_v >= w[1].voltage_v)
    {
        return Err(format!(
            "{}: напряжения паспортных точек должны возрастать",
            actuator.name
        ));
    }
    let (effective, _) = resolve_driver(actuator, driver);
    for d in [driver, effective].into_iter().flatten() {
        check_spec_list(d.name, &d.numeric_specs(), &d.temperature_specs())?;
        if d.drop_table
            .iter()
            .any(|(i, v)| !(i.is_finite() && v.is_finite() && *i > 0.0 && *v >= 0.0))
            || d.drop_table
                .windows(2)
                .any(|w| w[0].0 >= w[1].0 || w[0].1 > w[1].1)
            || d.low_voltage_current
                .is_some_and(|(v, i)| !(v.is_finite() && v > 0.0 && i.is_finite() && i >= 0.0))
            || d.output_fraction.get().is_some_and(|f| f <= 0.0 || f > 1.0)
        {
            return Err(format!("{}: неверная таблица падения напряжения", d.name));
        }
    }
    Ok(())
}

/// Определить реально работающий драйвер и проблему совместимости.
pub(crate) fn resolve_driver<'a>(
    actuator: &Actuator,
    given: Option<&'a Driver>,
) -> (Option<&'a Driver>, Option<String>) {
    if let Some(id) = actuator.builtin_driver {
        let builtin = crate::driver_by_id(id);
        return match given {
            Some(d)
                if d.id != id
                    || d.kind != DriverKind::Integrated
                    || !d.supports.contains(&actuator.kind.motor_type()) =>
            {
                (
                    builtin,
                    Some(format!(
                        "{} имеет встроенный драйвер, внешний {} к нему не подключается",
                        actuator.name, d.name
                    )),
                )
            }
            Some(d) => (Some(d), None),
            None => (builtin, None),
        };
    }
    match given {
        None => (None, None),
        Some(d) if d.kind == DriverKind::Integrated => (
            None,
            Some(format!(
                "{} встроен в другой привод и отдельно не подключается",
                d.name
            )),
        ),
        Some(d) if !d.supports.contains(&actuator.kind.motor_type()) => (
            None,
            Some(format!(
                "{} не управляет двигателем типа «{}»",
                d.name,
                actuator.kind.motor_type().title()
            )),
        ),
        Some(d) => (Some(d), None),
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum WindingThermal {
    NoData,
    Runaway,
    /// `peak_c` — наибольшая оценка, `mean_c` — по средним потерям.
    Known {
        peak_c: f64,
        mean_c: f64,
        dynamic: bool,
    },
}

fn has_two_node(a: &Actuator) -> bool {
    a.r_th_winding_housing_k_per_w.get().is_some()
        && a.r_th_housing_ambient_k_per_w.get().is_some()
        && a.tau_winding_s.get().is_some()
        && a.tau_housing_s.get().is_some()
}

fn thermal_is_proof(a: &Actuator) -> bool {
    let limit = a.max_winding_temp_c.is_proof();
    if has_two_node(a) {
        limit
            && a.r_th_winding_housing_k_per_w.is_proof()
            && a.r_th_housing_ambient_k_per_w.is_proof()
            && a.tau_winding_s.is_proof()
            && a.tau_housing_s.is_proof()
    } else {
        limit && a.r_th_total_k_per_w.is_proof()
    }
}

fn winding_thermal(
    a: &Actuator,
    line: &MotorLine,
    duty: &DutyCycle,
    currents: &[f64],
) -> WindingThermal {
    let Some(r0) = a.resistance_ohm.get() else {
        return WindingThermal::NoData;
    };
    let heat = a.heated_phases;
    if has_two_node(a) {
        let net = ThermalNet::from_datasheet(
            a.r_th_winding_housing_k_per_w.get().unwrap_or(1.0),
            a.r_th_housing_ambient_k_per_w.get().unwrap_or(1.0),
            a.tau_winding_s.get().unwrap_or(1.0),
            a.tau_housing_s.get().unwrap_or(1.0),
        );
        let segs: Vec<HeatSegment> = duty
            .segments
            .iter()
            .zip(currents)
            .map(|(s, &i)| {
                let p_nl = line.i0_a * line.k_e_v_s * s.speed_rad_s.abs();
                HeatSegment {
                    duration_s: s.duration_s,
                    copper_w_at_ref: heat * i * i * r0,
                    // При неизвестном распределении потерь холостого хода весь нагрев
                    // относим к обмотке, ближайшему к пределу узлу.
                    other_winding_w: p_nl,
                    other_housing_w: 0.0,
                }
            })
            .collect();
        return match periodic_peak_winding_c(&net, &segs, duty.ambient_c, T_REF_C) {
            Some(t) => WindingThermal::Known {
                peak_c: t,
                mean_c: t,
                dynamic: true,
            },
            None => WindingThermal::Runaway,
        };
    }
    if let Some(r_th) = a.r_th_total_k_per_w.get() {
        let total = duty.total_time_s();
        let mean_i2: f64 = duty
            .segments
            .iter()
            .zip(currents)
            .map(|(s, &i)| i * i * s.duration_s)
            .sum::<f64>()
            / total;
        let worst_i = currents.iter().copied().fold(0.0, f64::max);
        let worst = steady_winding_c(heat * worst_i * worst_i * r0, r_th, duty.ambient_c, T_REF_C);
        let mean = steady_winding_c(heat * mean_i2 * r0, r_th, duty.ambient_c, T_REF_C);
        return match (worst, mean) {
            (Some(p), Some(m)) => WindingThermal::Known {
                peak_c: p,
                mean_c: m,
                dynamic: false,
            },
            (None, Some(m)) => WindingThermal::Known {
                peak_c: f64::MAX,
                mean_c: m,
                dynamic: false,
            },
            _ => WindingThermal::Runaway,
        };
    }
    WindingThermal::NoData
}

fn segment_current(line: &MotorLine, s: &DutySegment) -> Option<f64> {
    if s.torque_nm == 0.0 && s.speed_rad_s == 0.0 {
        return Some(0.0);
    }
    line.current_for_torque(s.torque_nm.abs())
}

struct Solved<'a> {
    driver: Option<&'a Driver>,
    driver_problem: Option<String>,
    line: Option<MotorLine>,
    winding_c: f64,
    currents: Option<Vec<f64>>,
    i_rms: f64,
    i_peak: f64,
    thermal: WindingThermal,
    thermal_bound_proven: bool,
}

impl Solved<'_> {
    /// Напряжение на зажимах при токе `i`.
    fn v_terminal(&self, v: f64, i: f64) -> f64 {
        self.driver.map_or(v, |d| d.output_voltage(v, i))
    }

    fn current_model_has_data(a: &Actuator) -> bool {
        a.voltage_points.is_empty()
            && a.kind != ActuatorKind::Stepper
            && a.resistance_ohm.is_proof()
            && a.kt_motor_nm_per_a.is_proof()
            && a.gear_ratio.is_proof()
            && a.gear_efficiency.is_proof()
            && a.no_load_current_a.is_proof()
    }

    fn curve_is_proof(&self, a: &Actuator) -> bool {
        self.thermal_bound_proven
            && Self::current_model_has_data(a)
            && self.driver.is_some_and(|d| {
                d.output_fraction.is_proof()
                    || d.r_on_hot_ohm.is_proof()
                    || !d.drop_table.is_empty()
            })
    }
}

fn solve<'a>(
    actuator: &Actuator,
    driver: Option<&'a Driver>,
    duty: &DutyCycle,
) -> Result<Solved<'a>, String> {
    let (effective, driver_problem) = resolve_driver(actuator, driver);
    let ta = duty.ambient_c;
    let v = duty.supply_voltage_v;
    let has_thermal = has_two_node(actuator) || actuator.r_th_total_k_per_w.get().is_some();
    let fallback = actuator
        .max_winding_temp_c
        .get()
        .or(actuator.max_ambient_c.get())
        .unwrap_or(UNKNOWN_HOT_WINDING_C)
        .max(ta);
    let cap = actuator
        .max_winding_temp_c
        .get()
        .unwrap_or(UNKNOWN_HOT_WINDING_C)
        .max(ta);
    let model_at = |t| -> Result<_, String> {
        let line = actuator.motor_line(v, t);
        if line.as_ref().is_some_and(|l| !l.is_finite()) {
            return Err(
                "параметры кривой момент–скорость не конечны или вне области модели".into(),
            );
        }
        let currents = line.as_ref().and_then(|l| {
            duty.segments
                .iter()
                .map(|s| segment_current(l, s))
                .collect::<Option<Vec<_>>>()
        });
        if currents.iter().flatten().any(|i| !i.is_finite()) {
            return Err("ток обмотки не конечен".into());
        }
        Ok((line, currents))
    };
    let mut t = if has_thermal {
        ta.max(T_REF_C)
    } else {
        fallback
    };
    let mut thermal = WindingThermal::NoData;
    // Итерации дают оценку температуры. Для положительного вывода отдельно проверяем
    // верхнюю границу, поэтому результат не зависит от числа итераций.
    for _ in 0..12 {
        if !has_thermal {
            break;
        }
        let (line, currents) = model_at(t)?;
        let (Some(l), Some(c)) = (line.as_ref(), currents.as_ref()) else {
            break;
        };
        thermal = winding_thermal(actuator, l, duty, c);
        let next = match thermal {
            WindingThermal::Known { peak_c, .. } => peak_c.min(cap).max(ta),
            WindingThermal::Runaway => cap,
            WindingThermal::NoData => t,
        };
        let done = (next - t).abs() < 0.01;
        t = next;
        if done {
            break;
        }
    }
    // Ток при паспортном пределе температуры ограничивает ток всех более холодных
    // состояний. При известном пределе сравниваем нагрев от этой верхней нагрузки с ним.
    let mut thermal_bound_proven = false;
    if thermal_is_proof(actuator) && ta <= cap {
        let (line, currents) = model_at(cap)?;
        if let (Some(l), Some(c)) = (line.as_ref(), currents.as_ref())
            && let bound @ WindingThermal::Known { peak_c, .. } =
                winding_thermal(actuator, l, duty, c)
            && peak_c <= cap
        {
            thermal_bound_proven = true;
            thermal = bound;
            t = cap;
        }
    }
    let (line, currents) = model_at(t)?;
    let total = duty.total_time_s();
    let (i_rms, i_peak) = match &currents {
        Some(c) => (
            (duty
                .segments
                .iter()
                .zip(c)
                .map(|(s, i)| i * i * s.duration_s)
                .sum::<f64>()
                / total)
                .sqrt(),
            c.iter().copied().fold(0.0, f64::max),
        ),
        None => (0.0, 0.0),
    };
    if !(i_rms.is_finite() && i_peak.is_finite()) {
        return Err("среднеквадратичный или пиковый ток не конечен".into());
    }
    Ok(Solved {
        driver: effective,
        driver_problem,
        line,
        winding_c: t,
        currents,
        i_rms,
        i_peak,
        thermal,
        thermal_bound_proven,
    })
}

fn check_peak(a: &Actuator, ctx: &Solved, duty: &DutyCycle) -> ActuatorFinding {
    let kind = ActuatorCheck::PeakTorque;
    let tau = duty.peak_torque_nm();
    let curve = ctx.line.as_ref().map(|l| {
        let i = l.current_for_torque(tau).unwrap_or(0.0);
        l.available_torque(0.0, ctx.v_terminal(duty.supply_voltage_v, i))
    });
    if let Some(c) = curve.filter(|c| !c.is_finite()) {
        return finding(
            kind,
            Outcome::Fail,
            tau,
            c,
            "Н·м",
            "Расчёт пикового момента.",
        );
    }
    let spec = a.peak_torque_nm.get();
    let (limit, proof, why) = match (spec, curve) {
        (Some(s), Some(c)) if c < s => (
            c,
            a.peak_torque_nm.is_proof() && ctx.curve_is_proof(a),
            format!(
                "предел по кривой при {:.1} В и обмотке {:.0} °C (паспортный пик {s:.3} Н·м)",
                duty.supply_voltage_v, ctx.winding_c
            ),
        ),
        (Some(s), _) => (s, a.peak_torque_nm.is_proof(), a.peak_torque_nm.cite()),
        (None, Some(c)) => (
            c,
            false,
            format!("предел по кривой при {:.1} В", duty.supply_voltage_v),
        ),
        (None, None) => {
            return not_analyzed(kind, tau, "Н·м", "Пиковый момент не опубликован.");
        }
    };
    let peak_is_operating_limit = a.voltage_points.is_empty() || a.rated_torque_nm.is_proof();
    judge(
        kind,
        tau,
        limit,
        "Н·м",
        proof && peak_is_operating_limit,
        format!("Нужно {tau:.3} Н·м, предел модели {limit:.3} Н·м: {why}."),
    )
}

/// Сравнение «значение ≤ предел»: превышение — `Fail`, недоказанный предел — `NotAnalyzed`.
fn judge(
    kind: ActuatorCheck,
    value: f64,
    limit: f64,
    unit: &'static str,
    proof: bool,
    detail: String,
) -> ActuatorFinding {
    let outcome = if value > limit {
        Outcome::Fail
    } else if proof {
        Outcome::Pass
    } else {
        Outcome::NotAnalyzed
    };
    let detail = if outcome == Outcome::NotAnalyzed {
        format!("{detail} Для подтверждения запаса недостаточно данных.")
    } else {
        detail
    };
    finding(kind, outcome, value, limit, unit, detail)
}

fn check_continuous(a: &Actuator, duty: &DutyCycle) -> ActuatorFinding {
    let kind = ActuatorCheck::ContinuousTorque;
    let rms = duty.rms_torque_nm();
    let Some(rated) = a.rated_torque_nm.get() else {
        return not_analyzed(
            kind,
            rms,
            "Н·м",
            format!(
                "Длительный момент производитель не публикует; среднеквадратичный момент цикла {rms:.3} Н·м."
            ),
        );
    };
    // Снижение по температуре воздуха: I(T_a) = I_N·√((T_max − T_a)/(T_max − 25)).
    let mut limit = rated;
    let mut note = String::new();
    let mut proof = a.rated_torque_nm.is_proof();
    if let Some(t_max) = a.max_winding_temp_c.get()
        && duty.ambient_c > T_REF_C
    {
        let f = if t_max <= duty.ambient_c {
            0.0
        } else {
            ((t_max - duty.ambient_c) / (t_max - T_REF_C)).sqrt()
        };
        limit = rated * f;
        proof &= a.max_winding_temp_c.is_proof();
        note = format!(" С учётом воздуха {:.0} °C: ×{f:.3}.", duty.ambient_c);
    }
    judge(
        kind,
        rms,
        limit,
        "Н·м",
        proof,
        format!(
            "Среднеквадратичный момент {rms:.3} Н·м, длительный предел {limit:.3} Н·м ({}).{note}",
            a.rated_torque_nm.cite()
        ),
    )
}

fn check_speed(a: &Actuator, ctx: &Solved, duty: &DutyCycle) -> ActuatorFinding {
    let kind = ActuatorCheck::Speed;
    let Some(line) = ctx.line.as_ref() else {
        return not_analyzed(
            kind,
            duty.peak_speed_rad_s(),
            "рад/с",
            "Нет данных для кривой момент–скорость.",
        );
    };
    let v = duty.supply_voltage_v;
    let mut worst: Option<(f64, f64, f64, f64)> = None; // (запас, ω, ω_доступн, τ)
    let mut infeasible = false;
    let mut braking = false;
    for s in &duty.segments {
        if s.torque_nm == 0.0 && s.speed_rad_s == 0.0 {
            continue;
        }
        let tau = s.torque_nm.abs();
        let w = s.speed_rad_s.abs();
        let i = line.current_for_torque(tau).unwrap_or(0.0);
        let va = ctx.v_terminal(v, i);
        let generating = s.torque_nm * s.speed_rad_s < 0.0;
        braking |= generating;
        let (w_avail, ok) = match (generating, line.r_ohm) {
            (true, Some(r)) if line.stepper.is_none() => {
                let i_lo = (i - 2.0 * line.i0_a).max(0.0);
                let wa = (va.min(line.v_cap) + r * i_lo) / line.k_e_v_s;
                (wa, w <= wa)
            }
            _ => {
                let wa = line.available_speed(tau, va);
                let ok = w <= wa && tau <= line.available_torque(w, va) * (1.0 + 1e-9);
                (wa, ok)
            }
        };
        if !w_avail.is_finite()
            || !va.is_finite()
            || (!generating && !line.available_torque(w, va).is_finite())
        {
            return finding(
                kind,
                Outcome::Fail,
                w,
                f64::NAN,
                "рад/с",
                "Расчёт рабочей точки.",
            );
        }
        infeasible |= !ok;
        let margin = if ok {
            w_avail - w
        } else {
            -1.0 - (w - w_avail).max(0.0)
        };
        if worst.is_none_or(|(m, ..)| margin < m) {
            worst = Some((margin, w, w_avail, tau));
        }
    }
    let Some((_, w, w_avail, tau)) = worst else {
        return finding(
            kind,
            Outcome::Pass,
            0.0,
            0.0,
            "рад/с",
            "В цикле нет движения и нагрузки.",
        );
    };
    let shown_limit = if w_avail >= f64::MAX / 2.0 {
        0.0
    } else {
        w_avail
    };
    if infeasible {
        return finding(
            kind,
            Outcome::Fail,
            w,
            shown_limit,
            "рад/с",
            format!(
                "При {tau:.3} Н·м и {v:.1} В на питании доступно {shown_limit:.2} рад/с, нужно {w:.2} рад/с."
            ),
        );
    }
    if a.kind == ActuatorKind::Stepper {
        return finding(
            kind,
            Outcome::NotAnalyzed,
            w,
            shown_limit,
            "рад/с",
            "Без подтверждённого режима шагов и фазного тока верхняя граница по импедансу не доказывает запас момента.",
        );
    }
    if braking {
        return not_analyzed(
            kind,
            w,
            "рад/с",
            "При движении под действием нагрузки нужны данные о торможении и приёме энергии на шине.",
        );
    }
    finding(
        kind,
        if ctx.curve_is_proof(a) {
            Outcome::Pass
        } else {
            Outcome::NotAnalyzed
        },
        w,
        shown_limit,
        "рад/с",
        format!(
            "Худший участок: {tau:.3} Н·м, {w:.2} рад/с при расчётном пределе {shown_limit:.2} рад/с."
        ),
    )
}

fn check_range(kind: ActuatorCheck, who: &str, v: f64, min: Spec, max: Spec) -> ActuatorFinding {
    if let Some(m) = max.get()
        && v > m
    {
        return finding(
            kind,
            Outcome::Fail,
            v,
            m,
            "В",
            format!("{who}: {v:.2} В выше предела {m:.2} В ({}).", max.cite()),
        );
    }
    if let Some(m) = min.get()
        && v < m
    {
        return finding(
            kind,
            Outcome::Fail,
            v,
            m,
            "В",
            format!("{who}: {v:.2} В ниже минимума {m:.2} В ({}).", min.cite()),
        );
    }
    let Some(m) = max.get() else {
        return not_analyzed(
            kind,
            v,
            "В",
            format!("{who}: верхний предел напряжения не опубликован."),
        );
    };
    let proof = max.is_proof() && min.is_proof();
    judge(
        kind,
        v,
        m,
        "В",
        proof,
        format!("{who}: {v:.2} В при пределе {m:.2} В."),
    )
}

fn check_current(a: &Actuator, ctx: &Solved, duty: &DutyCycle) -> ActuatorFinding {
    let kind = ActuatorCheck::Current;
    if ctx.currents.is_none() {
        return not_analyzed(kind, 0.0, "А", "Связь момента с током не опубликована.");
    }
    let (rms, peak) = (ctx.i_rms, ctx.i_peak);
    // Предел пика: паспортный пиковый ток, иначе стоповый при этом напряжении.
    let stall_here = ctx
        .line
        .as_ref()
        .and_then(|l| l.r_ohm.map(|r| duty.supply_voltage_v.min(l.v_cap) / r));
    let peak_spec = if a.peak_current_a.get().is_some() {
        a.peak_current_a
    } else {
        a.stall_current_a
    };
    let peak_limit = match (peak_spec.get(), stall_here) {
        (Some(p), _) => Some(p),
        (None, Some(s)) if a.kind != ActuatorKind::Stepper => Some(s),
        _ => None,
    };
    let rated = a.rated_current_a;
    if let Some(p) = peak_limit
        && peak > p
    {
        return finding(
            kind,
            Outcome::Fail,
            peak,
            p,
            "А",
            format!(
                "Пиковый ток {peak:.3} А выше предела {p:.3} А ({}).",
                peak_spec.cite()
            ),
        );
    }
    if let Some(r) = rated.get()
        && rms > r
    {
        return finding(
            kind,
            Outcome::Fail,
            rms,
            r,
            "А",
            format!(
                "Среднеквадратичный ток {rms:.3} А выше длительного {r:.3} А ({}).",
                rated.cite()
            ),
        );
    }
    let Some(r) = rated.get() else {
        return not_analyzed(
            kind,
            peak,
            "А",
            format!("Длительный ток не опубликован; пиковый ток цикла {peak:.3} А."),
        );
    };
    // Стоповый ток — аварийная точка, она не разрешает работу сверх длительного тока.
    let peak_proven = peak <= r && rated.is_proof();
    if rated.is_proof()
        && peak_proven
        && Solved::current_model_has_data(a)
        && ctx.thermal_bound_proven
    {
        finding(
            kind,
            Outcome::Pass,
            rms,
            r,
            "А",
            format!("Ток: {rms:.3} А rms при пределе {r:.3} А, пик {peak:.3} А."),
        )
    } else {
        finding(
            kind,
            Outcome::NotAnalyzed,
            peak,
            peak_limit.unwrap_or(0.0),
            "А",
            format!(
                "Расчётный пик {peak:.3} А. Данных о рабочем токе или допустимости импульсов недостаточно."
            ),
        )
    }
}

fn check_driver_compat(a: &Actuator, ctx: &Solved) -> ActuatorFinding {
    let kind = ActuatorCheck::DriverCompatibility;
    if let Some(p) = &ctx.driver_problem {
        return finding(kind, Outcome::Fail, 0.0, 0.0, "", format!("{p}."));
    }
    match ctx.driver {
        Some(d) => finding(
            kind,
            Outcome::Pass,
            0.0,
            0.0,
            "",
            format!("{} подходит для привода {}.", d.name, a.name),
        ),
        None => not_analyzed(
            kind,
            0.0,
            "",
            format!("{NO_DRIVER}: совместимость не проверена."),
        ),
    }
}

fn check_driver_current(a: &Actuator, ctx: &Solved, duty: &DutyCycle) -> ActuatorFinding {
    let kind = ActuatorCheck::DriverCurrent;
    let Some(d) = ctx.driver else {
        return not_analyzed(kind, ctx.i_peak, "А", format!("{NO_DRIVER}."));
    };
    let Some(c) = ctx.currents.as_ref() else {
        return not_analyzed(
            kind,
            0.0,
            "А",
            format!("{}: ток привода неизвестен.", a.name),
        );
    };
    let (rms, peak) = (ctx.i_rms, ctx.i_peak);
    let cont_spec = d.continuous_current_at(duty.supply_voltage_v);
    let cont = cont_spec.get();
    let peak_lim = d.current_peak_a.get();
    if let Some(p) = peak_lim
        && peak > p
    {
        return finding(
            kind,
            Outcome::Fail,
            peak,
            p,
            "А",
            format!(
                "{}: пиковый ток {peak:.3} А выше предела {p:.3} А ({}).",
                d.name,
                d.current_peak_a.cite()
            ),
        );
    }
    if let Some(l) = cont
        && rms > l
    {
        return finding(
            kind,
            Outcome::Fail,
            rms,
            l,
            "А",
            format!(
                "{}: длительный ток {rms:.3} А выше предела {l:.3} А ({}).",
                d.name,
                cont_spec.cite()
            ),
        );
    }
    // Смежные участки образуют один импульс, в том числе через границу циклов.
    if let (Some(l), Some(t_max)) = (cont, d.peak_max_duration_s.get()) {
        let start = c.iter().position(|&i| i <= l).unwrap_or(0);
        let mut duration = 0.0;
        for j in (start..c.len()).chain(0..start) {
            duration = if c[j] > l {
                duration + duty.segments[j].duration_s
            } else {
                0.0
            };
            if duration > t_max {
                return finding(
                    kind,
                    Outcome::Fail,
                    duration,
                    t_max,
                    "с",
                    format!(
                        "{}: ток выше {l:.3} А держится {duration:.3} с подряд при пределе {t_max:.3} с.",
                        d.name
                    ),
                );
            }
        }
    }
    let Some(l) = cont else {
        return not_analyzed(
            kind,
            peak,
            "А",
            format!(
                "{}: длительный ток не нормирован; пик цикла {peak:.3} А.",
                d.name
            ),
        );
    };
    let over_continuous = peak > l;
    // Одной длительности недостаточно: для повторяющегося импульса нужны условия паузы.
    let peak_ok = !over_continuous;
    if cont_spec.is_proof() && peak_ok {
        finding(
            kind,
            Outcome::Pass,
            peak.max(rms),
            if over_continuous {
                peak_lim.unwrap_or(l)
            } else {
                l
            },
            "А",
            format!("{}: {rms:.3} А rms, пик {peak:.3} А в пределах.", d.name),
        )
    } else {
        finding(
            kind,
            Outcome::NotAnalyzed,
            peak,
            peak_lim.unwrap_or(l),
            "А",
            format!(
                "{}: пик {peak:.3} А. Для подтверждения режима нужны рабочий предел и условия повторения импульсов.",
                d.name
            ),
        )
    }
}

fn check_driver_voltage(ctx: &Solved, duty: &DutyCycle) -> ActuatorFinding {
    let kind = ActuatorCheck::DriverVoltage;
    let v = duty.supply_voltage_v;
    let Some(d) = ctx.driver else {
        return not_analyzed(kind, v, "В", format!("{NO_DRIVER}."));
    };
    if let Some(m) = d.voltage_abs_max_v.get()
        && v > m
    {
        return finding(
            kind,
            Outcome::Fail,
            v,
            m,
            "В",
            format!("{}: {v:.2} В выше абсолютного предела {m:.2} В.", d.name),
        );
    }
    check_range(kind, d.name, v, d.voltage_min_v, d.voltage_max_v)
}

fn check_driver_temperature(a: &Actuator, ctx: &Solved, duty: &DutyCycle) -> ActuatorFinding {
    let kind = ActuatorCheck::DriverTemperature;
    let ta = duty.ambient_c;
    let Some(d) = ctx.driver else {
        return not_analyzed(kind, ta, "°C", format!("{NO_DRIVER}."));
    };
    if let Some(m) = d.max_ambient_c.get()
        && ta > m
    {
        return finding(
            kind,
            Outcome::Fail,
            ta,
            m,
            "°C",
            format!("{}: воздух {ta:.1} °C выше допустимых {m:.1} °C.", d.name),
        );
    }
    let Some(c) = ctx.currents.as_ref() else {
        return not_analyzed(
            kind,
            ta,
            "°C",
            format!("{}: ток привода неизвестен.", d.name),
        );
    };
    if d.rated_module {
        let cont = d.continuous_current_at(duty.supply_voltage_v);
        return match (cont.get(), d.max_ambient_c.get()) {
            (Some(l), Some(m))
                if ctx.i_peak <= l && cont.is_proof() && d.max_ambient_c.is_proof() =>
            {
                finding(
                    kind,
                    Outcome::Pass,
                    ta,
                    m,
                    "°C",
                    format!(
                        "{}: длительный ток нормирован производителем до {m:.0} °C воздуха.",
                        d.name
                    ),
                )
            }
            (Some(l), _) if ctx.i_rms > l => finding(
                kind,
                Outcome::Fail,
                ctx.i_rms,
                l,
                "А",
                format!("{}: ток выше нормированного для модуля.", d.name),
            ),
            _ => not_analyzed(
                kind,
                ta,
                "°C",
                format!("{}: тепловые данные модуля неполны.", d.name),
            ),
        };
    }
    let (Some(theta), Some(t_lim)) = (d.r_theta_ja_k_per_w.get(), d.junction_limit_c()) else {
        return not_analyzed(
            kind,
            ta,
            "°C",
            format!(
                "{}: тепловое сопротивление или предел кристалла не опубликованы.",
                d.name
            ),
        );
    };
    // У шагового работают два моста, у остальных один.
    let bridges = if a.kind.motor_type() == MotorType::BipolarStepper {
        2.0
    } else {
        1.0
    };
    let q = d.quiescent_current_a.get().unwrap_or(0.0) * duty.supply_voltage_v;
    let total = duty.total_time_s();
    let mut mean = 0.0;
    let mut worst: f64 = 0.0;
    for (s, &i) in duty.segments.iter().zip(c) {
        let Some(p) = d.conduction_loss_w(i) else {
            return not_analyzed(
                kind,
                ta,
                "°C",
                format!("{}: падение на ключах не опубликовано.", d.name),
            );
        };
        let p = bridges * p;
        mean += p * s.duration_s / total;
        worst = worst.max(p);
    }
    let t_mean = ta + theta * (mean + q);
    let t_worst = ta + theta * (worst + q);
    if t_mean > t_lim {
        return finding(
            kind,
            Outcome::Fail,
            t_mean,
            t_lim,
            "°C",
            format!(
                "{}: кристалл нагреется до {t_mean:.0} °C при пределе {t_lim:.0} °C (R_θ = {theta:.1} К/Вт, потери {:.2} Вт).",
                d.name,
                mean + q
            ),
        );
    }
    if t_worst <= t_lim {
        return judge(
            kind,
            t_worst,
            t_lim,
            "°C",
            d.r_theta_ja_k_per_w.is_proof()
                && d.t_j_max_c.is_proof()
                && d.quiescent_current_a.is_proof()
                && (d.r_on_hot_ohm.is_proof() || !d.drop_table.is_empty()),
            format!(
                "{}: кристалл до {t_worst:.0} °C при пределе {t_lim:.0} °C.",
                d.name
            ),
        );
    }
    finding(
        kind,
        Outcome::NotAnalyzed,
        t_worst,
        t_lim,
        "°C",
        format!(
            "{}: по средним потерям {t_mean:.0} °C, на самом тяжёлом участке до {t_worst:.0} °C при пределе {t_lim:.0} °C; тепловая инерция корпуса не опубликована.",
            d.name
        ),
    )
}

fn check_winding(a: &Actuator, ctx: &Solved) -> ActuatorFinding {
    let kind = ActuatorCheck::WindingTemperature;
    let limit = a.max_winding_temp_c;
    let proof = ctx.thermal_bound_proven && Solved::current_model_has_data(a);
    if let Some(t_max) = limit.get()
        && ctx.thermal == WindingThermal::NoData
        && ctx.winding_c > t_max
    {
        return finding(
            kind,
            Outcome::Fail,
            ctx.winding_c,
            t_max,
            "°C",
            "Температура воздуха уже выше предела обмотки.",
        );
    }
    match ctx.thermal {
        WindingThermal::NoData => not_analyzed(
            kind,
            0.0,
            "°C",
            "Тепловые сопротивления обмотки не опубликованы: нагрев не проверен.",
        ),
        WindingThermal::Runaway => finding(
            kind,
            Outcome::Fail,
            limit.get().unwrap_or(0.0),
            limit.get().unwrap_or(0.0),
            "°C",
            "Теплового равновесия нет: сопротивление растёт с нагревом быстрее, чем уходит тепло.",
        ),
        WindingThermal::Known {
            peak_c,
            mean_c,
            dynamic,
        } => {
            let Some(t_max) = limit.get() else {
                return not_analyzed(
                    kind,
                    mean_c,
                    "°C",
                    "Предел температуры обмотки не опубликован.",
                );
            };
            if dynamic {
                return judge(
                    kind,
                    peak_c,
                    t_max,
                    "°C",
                    proof,
                    format!(
                        "Установившийся цикл: обмотка до {peak_c:.1} °C при пределе {t_max:.0} °C."
                    ),
                );
            }
            if mean_c > t_max {
                return finding(
                    kind,
                    Outcome::Fail,
                    mean_c,
                    t_max,
                    "°C",
                    format!(
                        "По средним потерям обмотка {mean_c:.1} °C выше предела {t_max:.0} °C."
                    ),
                );
            }
            if peak_c <= t_max {
                return judge(
                    kind,
                    peak_c,
                    t_max,
                    "°C",
                    proof,
                    format!(
                        "Даже на самом тяжёлом участке обмотка {peak_c:.1} °C при пределе {t_max:.0} °C."
                    ),
                );
            }
            finding(
                kind,
                Outcome::NotAnalyzed,
                mean_c,
                t_max,
                "°C",
                format!(
                    "По средним потерям {mean_c:.1} °C, но самый тяжёлый участок в установившемся режиме превысил бы {t_max:.0} °C; постоянных времени в паспорте нет."
                ),
            )
        }
    }
}

fn check_ambient(a: &Actuator, duty: &DutyCycle) -> ActuatorFinding {
    let kind = ActuatorCheck::AmbientTemperature;
    let ta = duty.ambient_c;
    if let Some(m) = a.min_ambient_c.get()
        && ta < m
    {
        return finding(
            kind,
            Outcome::Fail,
            ta,
            m,
            "°C",
            format!(
                "Воздух {ta:.1} °C ниже рабочего минимума {m:.1} °C ({}).",
                a.min_ambient_c.cite()
            ),
        );
    }
    match a.max_ambient_c.get() {
        Some(m) => judge(
            kind,
            ta,
            m,
            "°C",
            a.max_ambient_c.is_proof() && a.min_ambient_c.is_proof(),
            format!(
                "Воздух {ta:.1} °C, допустимо до {m:.1} °C ({}).",
                a.max_ambient_c.cite()
            ),
        ),
        None => not_analyzed(
            kind,
            ta,
            "°C",
            "Допустимая температура воздуха не опубликована.",
        ),
    }
}

fn check_hold(a: &Actuator, duty: &DutyCycle) -> ActuatorFinding {
    let kind = ActuatorCheck::HoldWhenUnpowered;
    let Some(t) = duty.unpowered_hold_torque_nm else {
        return not_analyzed(
            kind,
            0.0,
            "Н·м",
            "Не указано, должен ли привод держать нагрузку без питания.",
        );
    };
    if t == 0.0 {
        return finding(
            kind,
            Outcome::Pass,
            0.0,
            0.0,
            "Н·м",
            "Удержание без питания не требуется.",
        );
    }
    if let Brake::Holding { torque_nm } = a.brake {
        return match torque_nm.get() {
            Some(l) => judge(
                kind,
                t,
                l,
                "Н·м",
                torque_nm.is_proof(),
                format!(
                    "Тормоз держит {l:.3} Н·м, нужно {t:.3} Н·м ({}).",
                    torque_nm.cite()
                ),
            ),
            None => not_analyzed(kind, t, "Н·м", "Тормоз есть, но его момент не опубликован."),
        };
    }
    match a.backdrive {
        Backdrive::Reversible { torque_nm } => {
            let friction = torque_nm.get().unwrap_or(0.0);
            finding(
                kind,
                Outcome::Fail,
                t,
                0.0,
                "Н·м",
                format!(
                    "Привод обратим и без тормоза: при пропадании питания нагрузка {t:.3} Н·м не удерживается. Трение проворота {friction:.3} Н·м удержанием не считается: оно не гарантировано и падает от вибрации и износа."
                ),
            )
        }
        Backdrive::SelfLocking { source } => not_analyzed(
            kind,
            t,
            "Н·м",
            format!(
                "Самоторможение заявлено ({source}). Нужен допустимый момент удержания без питания."
            ),
        ),
        Backdrive::Unknown => not_analyzed(
            kind,
            t,
            "Н·м",
            "Производитель не сообщает, держит ли привод нагрузку без питания: считать удержанием нельзя.",
        ),
    }
}

/// Оценить привод на заданном цикле. Встроенный драйвер подставляется сам; `driver` — для
/// приводов с внешним драйвером. Возвращает все проверки из [`ActuatorCheck::ALL`].
pub fn assess_actuator(
    actuator: &Actuator,
    driver: Option<&Driver>,
    duty: &DutyCycle,
) -> Vec<ActuatorFinding> {
    let solved = validate(actuator, driver, duty).and_then(|()| solve(actuator, driver, duty));
    let ctx = match solved {
        Ok(ctx) => ctx,
        Err(why) => {
            return ActuatorCheck::ALL
                .iter()
                .map(|&kind| ActuatorFinding {
                    kind,
                    outcome: Outcome::Fail,
                    value: 0.0,
                    limit: 0.0,
                    unit: "",
                    detail: format!("Некорректные данные для расчёта: {why}."),
                })
                .collect();
        }
    };
    vec![
        check_peak(actuator, &ctx, duty),
        check_continuous(actuator, duty),
        check_speed(actuator, &ctx, duty),
        check_range(
            ActuatorCheck::Voltage,
            actuator.name,
            duty.supply_voltage_v,
            actuator.voltage_min_v,
            actuator.voltage_max_v,
        ),
        check_current(actuator, &ctx, duty),
        check_driver_compat(actuator, &ctx),
        check_driver_current(actuator, &ctx, duty),
        check_driver_voltage(&ctx, duty),
        check_driver_temperature(actuator, &ctx, duty),
        check_winding(actuator, &ctx),
        check_ambient(actuator, duty),
        check_hold(actuator, duty),
    ]
}

/// Электрическая нагрузка на шину за цикл. `None` — вход некорректен или связь момента
/// с током не опубликована.
pub fn electrical_load(
    actuator: &Actuator,
    driver: Option<&Driver>,
    duty: &DutyCycle,
) -> Option<ElectricalLoad> {
    electrical_load_for_budget(actuator, driver, duty)
        .ok()
        .flatten()
        .map(|(load, _)| load)
}

/// Признак рядом с нагрузкой указывает, подтверждены ли её границы исходными данными.
pub(crate) fn electrical_load_for_budget(
    actuator: &Actuator,
    driver: Option<&Driver>,
    duty: &DutyCycle,
) -> Result<Option<(ElectricalLoad, bool)>, String> {
    validate(actuator, driver, duty)?;
    let ctx = solve(actuator, driver, duty)?;
    let (Some(line), Some(currents)) = (ctx.line.as_ref(), ctx.currents.as_ref()) else {
        return Ok(None);
    };
    let Some(r) = line.r_ohm else { return Ok(None) };
    let v = duty.supply_voltage_v;
    let stepper = actuator.kind == ActuatorKind::Stepper;
    let bridges = if stepper { 2.0 } else { 1.0 };
    let q = ctx
        .driver
        .and_then(|d| d.quiescent_current_a.get())
        .unwrap_or(0.0)
        + actuator.standby_current_a.get().unwrap_or(0.0);
    let total = duty.total_time_s();
    let (mut energy, mut peak) = (0.0, 0.0f64);
    let mut proven = check_speed(actuator, &ctx, duty).outcome == Outcome::Pass
        && check_current(actuator, &ctx, duty).outcome == Outcome::Pass
        && ctx.driver_problem.is_none()
        && actuator.kind == ActuatorKind::BrushedGearmotor
        && ctx
            .driver
            .is_some_and(|d| d.kind == DriverKind::HBridge && d.quiescent_current_a.is_proof());
    for (s, &i) in duty.segments.iter().zip(currents) {
        let mech = s.torque_nm * s.speed_rad_s;
        proven &= mech >= 0.0;
        let copper = actuator.heated_phases * r * i * i;
        let p_motor = if stepper {
            copper + mech.max(0.0)
        } else {
            // Даже на торможении ток греет медь. Без схемы торможения
            // не вычитаем генерируемую энергию из потребления.
            let converted = line.k_e_v_s * s.speed_rad_s.abs() * i;
            if !converted.is_finite() {
                return Err("мощность противо-ЭДС не конечна".into());
            }
            copper
                + if mech >= 0.0 {
                    converted.max(mech)
                } else {
                    0.0
                }
        };
        let p_driver = ctx
            .driver
            .and_then(|d| d.conduction_loss_w(i))
            .unwrap_or(0.0)
            * bridges;
        let p = p_motor + p_driver + v * q;
        // Мгновенный ток источника во включённой части ШИМ больше среднего
        // тока за период. Пики источника считаем без скидки на скважность.
        let on = v * (bridges * i + q);
        if !(p.is_finite() && on.is_finite()) {
            return Err("мощность на шине не конечна".into());
        }
        energy += p * (s.duration_s / total);
        peak = peak.max(p).max(on);
    }
    if matches!(
        actuator.kind,
        ActuatorKind::HobbyServo | ActuatorKind::SmartServo
    ) {
        // При пуске или заклинивании возможен паспортный стоповый ток.
        // По нижней точке напряжения верхний ток шины не доказывается.
        let stalled = actuator.point_at(v).and_then(|p| p.stall_current_a);
        for i in [
            stalled,
            actuator.stall_current_a.get(),
            actuator.peak_current_a.get(),
        ]
        .into_iter()
        .flatten()
        {
            peak = peak.max(v * (i + q));
        }
    }
    let avg = energy;
    let out = ElectricalLoad {
        current_rms_a: ctx.i_rms,
        current_peak_a: ctx.i_peak,
        power_avg_w: avg,
        power_peak_w: peak,
        bus_current_avg_a: avg / v,
        bus_current_peak_a: peak / v,
    };
    if ![
        out.current_rms_a,
        out.current_peak_a,
        out.power_avg_w,
        out.power_peak_w,
        out.bus_current_avg_a,
        out.bus_current_peak_a,
    ]
    .iter()
    .all(|x| x.is_finite())
    {
        return Err("итоговая нагрузка на шину не конечна".into());
    }
    Ok(Some((out, proven)))
}
