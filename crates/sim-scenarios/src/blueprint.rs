//! Чертёж и единый вердикт по билду.
//!
//! Правило: «прошёл симуляцию» можно получить, только когда каждая обязательная опасность
//! посчитана и выдержана с запасом. Что не посчитано, то «не доказано», а не «нормально».
//! Даже высший вердикт не заменяет стенд: он значит только, что симуляция не нашла причин
//! не переходить к стендовым испытаниям.

use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::sync::Mutex;

use sim_core::Vec3;
use sim_mcu::exo::fault;

use crate::beam::BeamStrut;
use crate::elbow_plant::Obstacle;
use crate::exo_anatomy::ExoAnatomy;
use crate::exo_loop::{ContactLog, ExoLoop, CUFF_M};
use crate::exo_arm::SAFETY_FACTOR;
use crate::human_limits as lim;
use crate::pcb_board::PcbBoard;
use crate::scenario::*;

pub const H_FUNCTION: &str = "Поднимает заявленный груз";
pub const H_STRUCT: &str = "Прочность штанги (статика)";
pub const H_FATIGUE: &str = "Усталость и концентраторы";
pub const H_TRACES: &str = "Перегрев дорожек платы";
pub const H_PARTS: &str = "Запас по температуре деталей";
pub const H_CUFF: &str = "Давление манжеты на кожу";
pub const H_DROP: &str = "Отказ привода с грузом";
pub const H_ESTOP: &str = "Аварийная кнопка";
pub const H_SENSOR: &str = "Обрыв датчика угла";
pub const H_COMM: &str = "Потеря связи";
pub const H_HANG: &str = "Зависание прошивки";
pub const H_JAM: &str = "Заклинивание сустава";
pub const H_IMPACT: &str = "Удар рукой о препятствие";
pub const H_CLAMP: &str = "Зажим руки приводом";
pub const H_SHEAR: &str = "Ножницы в шарнире и острые кромки";
pub const H_BATTERY: &str = "Батарея: КЗ и перегрев";
pub const H_PRINT: &str = "Печатные детали";
pub const H_REAL_MCU: &str = "Прошивка на настоящем чипе";

/// Без доказательства по каждому пункту вердикта «прошёл симуляцию» быть не может.
pub const REQUIRED_HAZARDS: [&str; 18] = [
    H_FUNCTION, H_STRUCT, H_FATIGUE, H_TRACES, H_PARTS, H_CUFF, H_DROP, H_ESTOP, H_SENSOR, H_COMM,
    H_HANG, H_JAM, H_IMPACT, H_CLAMP, H_SHEAR, H_BATTERY, H_PRINT, H_REAL_MCU,
];
/// Предел обмотки maxon EC 90 flat, °C.
const WINDING_MAX_C: f64 = 125.0;
/// Сколько модельного времени ждём реакции прошивки на заклинивание, с.
const JAM_HORIZON_S: usize = 150;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BuildVerdict {
    /// Найдена конкретная опасность: строить в таком виде нельзя.
    Rejected,
    /// Явных нарушений нет, но не всё посчитано.
    NotProven,
    /// Все обязательные опасности посчитаны и выдержаны. Дальше стенд.
    PassedSimulation,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    Pass,
    Fail,
    NotAnalyzed,
}

#[derive(Clone, Debug)]
pub struct Finding {
    pub hazard: &'static str,
    pub outcome: Outcome,
    pub value: f64,
    pub limit: f64,
    pub unit: &'static str,
    pub detail: String,
}

impl Finding {
    fn judged(hazard: &'static str, ok: bool, value: f64, limit: f64, unit: &'static str, detail: impl Into<String>) -> Self {
        let finite = value.is_finite() && limit.is_finite();
        Self {
            hazard,
            outcome: if ok && finite { Outcome::Pass } else { Outcome::Fail },
            value,
            limit,
            unit,
            detail: detail.into(),
        }
    }

    /// Требование «значение не больше предела». NaN и ∞ — провал.
    pub fn at_most(hazard: &'static str, value: f64, limit: f64, unit: &'static str, detail: impl Into<String>) -> Self {
        Self::judged(hazard, value <= limit, value, limit, unit, detail)
    }

    /// Требование «значение не меньше предела». NaN и ∞ — провал.
    pub fn at_least(hazard: &'static str, value: f64, limit: f64, unit: &'static str, detail: impl Into<String>) -> Self {
        Self::judged(hazard, value >= limit, value, limit, unit, detail)
    }

    pub fn failed(hazard: &'static str, detail: impl Into<String>) -> Self {
        Self {
            hazard,
            outcome: Outcome::Fail,
            value: f64::NAN,
            limit: f64::NAN,
            unit: "",
            detail: detail.into(),
        }
    }

    pub fn passed(hazard: &'static str, detail: impl Into<String>) -> Self {
        Self {
            hazard,
            outcome: Outcome::Pass,
            value: f64::NAN,
            limit: f64::NAN,
            unit: "",
            detail: detail.into(),
        }
    }

    pub fn not_analyzed(hazard: &'static str, why: impl Into<String>) -> Self {
        Self {
            hazard,
            outcome: Outcome::NotAnalyzed,
            value: f64::NAN,
            limit: f64::NAN,
            unit: "",
            detail: why.into(),
        }
    }
}

/// Вердикт по списку находок. По умолчанию «не доказано»: чтобы пройти, каждая обязательная
/// опасность должна иметь положительное доказательство.
pub fn verdict(findings: &[Finding]) -> BuildVerdict {
    if findings.iter().any(|f| f.outcome == Outcome::Fail) {
        return BuildVerdict::Rejected;
    }
    let proven = REQUIRED_HAZARDS.iter().all(|h| {
        let mine: Vec<_> = findings.iter().filter(|f| f.hazard == *h).collect();
        !mine.is_empty() && mine.iter().all(|f| f.outcome == Outcome::Pass)
    });
    if proven {
        BuildVerdict::PassedSimulation
    } else {
        BuildVerdict::NotProven
    }
}

/// Чертёж экзоскелета локтя: из чего собран и в каких условиях работает.
#[derive(Clone, Debug)]
pub struct ExoBuild {
    pub name: &'static str,
    pub payload_kg: f64,
    pub strut_b_mm: f64,
    pub strut_h_mm: f64,
    pub trace_mm: f64,
    pub copper_2oz: bool,
    pub motor_current_a: f64,
    pub ambient_c: f64,
    pub assist_pct: f64,
    pub cuff_area_cm2: f64,
    /// Долгая носка: предел давления на кожу намного строже.
    pub long_wear: bool,
    /// Правки конфигурации прошивки, строки «имя = значение».
    pub firmware_edit: &'static str,
}

impl ExoBuild {
    /// Аккуратный билд: на нём проверяем, что вердикт не поднимает ложных тревог.
    pub fn reference() -> Self {
        Self {
            name: "Аккуратный билд: 5 кг, штанга 12×30, дорожки 3 мм",
            payload_kg: 5.0,
            strut_b_mm: 12.0,
            strut_h_mm: 30.0,
            trace_mm: 3.0,
            copper_2oz: true,
            motor_current_a: 8.0,
            ambient_c: 25.0,
            assist_pct: 70.0,
            cuff_area_cm2: 150.0,
            long_wear: false,
            firmware_edit: "",
        }
    }
}

/// Заведомо опасные билды и опасность, на которой каждый обязан провалиться.
pub fn dangerous_builds() -> Vec<(ExoBuild, &'static str)> {
    let r = ExoBuild::reference;
    vec![
        (
            ExoBuild { name: "Опасно: тонкая штанга 20×10 под 50 кг", payload_kg: 50.0, strut_b_mm: 20.0, strut_h_mm: 10.0, ..r() },
            H_STRUCT,
        ),
        (
            ExoBuild { name: "Опасно: дорожки 1 мм, 1 oz под 8 А", trace_mm: 1.0, copper_2oz: false, ..r() },
            H_TRACES,
        ),
        (
            ExoBuild { name: "Опасно: манжета 10 см²", cuff_area_cm2: 10.0, ..r() },
            H_CUFF,
        ),
        (
            ExoBuild { name: "Опасно: 20 кг без тормоза", payload_kg: 20.0, ..r() },
            H_DROP,
        ),
        (
            ExoBuild {
                name: "Опасно: прошивка без защиты от заклинивания",
                firmware_edit: "stall_time_s = 100000\nt_winding_max_c = 1000\ncontact_torque_nm = 0\n",
                ..r()
            },
            H_JAM,
        ),
        (
            ExoBuild {
                name: "Опасно: сторожевой таймер фактически выключен",
                firmware_edit: "wdt_timeout_ms = 100000\n",
                ..r()
            },
            H_HANG,
        ),
        (
            ExoBuild {
                name: "Опасно: привод слишком слабый для груза",
                firmware_edit: "i_max_a = 1.5\ni_trip_a = 3\n",
                ..r()
            },
            H_FUNCTION,
        ),
        (
            ExoBuild {
                name: "Опасно: нет реакции на контакт, рука зажата",
                firmware_edit: "contact_torque_nm = 0\n",
                ..r()
            },
            H_CLAMP,
        ),
        (
            ExoBuild {
                name: "Опасно: нет предела скорости, удар по руке",
                firmware_edit: "speed_limit_dps = 0\ncontact_torque_nm = 0\n",
                ..r()
            },
            H_IMPACT,
        ),
        (
            ExoBuild {
                name: "Опасно: прошивка разрешает 30 А, плата на 8 А",
                trace_mm: 1.5,
                copper_2oz: false,
                firmware_edit: "i_max_a = 30\ni_trip_a = 35\n",
                ..r()
            },
            H_TRACES,
        ),
    ]
}

fn run(s: &mut ExoLoop, seconds: f64) {
    for _ in 0..(seconds / 0.01).round() as usize {
        s.advance(0.01);
    }
}

/// Стенд с прошивкой под этот билд. Опечатка в конфигурации — ошибка, а не молчаливый пропуск.
fn rig(b: &ExoBuild) -> Result<ExoLoop, String> {
    let mut s = ExoLoop::default();
    s.set_param("payload", b.payload_kg);
    s.set_param("phone_payload", b.payload_kg);
    s.set_param("assist", b.assist_pct);
    s.set_param("ambient", b.ambient_c);
    // Без сброса мотор и батарея остались бы при температуре по умолчанию, а не билда.
    s.reset();
    if !b.firmware_edit.is_empty() {
        s.set_text("config", b.firmware_edit.to_string());
        if let Some(e) = s.console().iter().find(|l| l.contains("не принята")) {
            return Err(e.clone());
        }
    }
    match s.firmware_error() {
        Some(e) => Err(format!("прошивка не запустилась: {e}")),
        None => Ok(s),
    }
}

fn check_structure(b: &ExoBuild) -> Finding {
    let mut s = BeamStrut::default();
    s.set_param("b", b.strut_b_mm);
    s.set_param("h", b.strut_h_mm);
    s.set_param("load", b.payload_kg);
    Finding::at_least(H_STRUCT, s.static_safety(), SAFETY_FACTOR.0, "×", "запас по текучести σт/σ, весь груз на штанге")
}

fn check_board(b: &ExoBuild) -> Vec<Finding> {
    // Плата должна выдержать то, что реально разрешает прошивка, а не только заявленный ток.
    let fw_limit = match rig(b) {
        Ok(s) => s.firmware_config().i_max_a as f64,
        Err(e) => return vec![Finding::failed(H_TRACES, e.clone()), Finding::failed(H_PARTS, e)],
    };
    let mut p = PcbBoard::default();
    p.set_param("i", b.motor_current_a.max(fw_limit));
    p.set_param("w", b.trace_mm);
    p.set_param("amb", b.ambient_c);
    p.set_choice("cu", b.copper_2oz as usize);
    let rep = p.report();
    let rise = rep.traces.iter().map(|t| t.delta_t).fold(f64::NEG_INFINITY, f64::max);
    let margin = rep.parts.iter().map(|t| t.margin_c).fold(f64::INFINITY, f64::min);
    vec![
        Finding::at_most(H_TRACES, rise, 20.0, "К", "самая горячая дорожка, IPC-2221"),
        Finding::at_least(H_PARTS, margin, 20.0, "°C", "запас до предела детали: взаимный нагрев не считается, поэтому нужно от 20 °C"),
    ]
}

fn check_body(b: &ExoBuild) -> Vec<Finding> {
    let mut a = ExoAnatomy::default();
    a.set_param("payload", b.payload_kg);
    a.set_param("assist", b.assist_pct);
    a.set_param("area", b.cuff_area_cm2);
    // Предплечье горизонтально: худший угол для момента от груза.
    a.set_param("angle", 90.0);
    let Some(n) = a.summary_numbers() else {
        let why = "модель руки не загрузилась, считать нечем";
        return vec![Finding::not_analyzed(H_CUFF, why), Finding::not_analyzed(H_DROP, why)];
    };
    let limit = if b.long_wear { lim::CUFF_PRESSURE_LONG_WEAR_KPA } else { lim::CUFF_PRESSURE_DISCOMFORT_KPA };
    let women = lim::FLEXION_MEASURED[3].2;
    vec![
        Finding::at_most(H_CUFF, n.cuff_pressure_kpa, limit, "кПа", if b.long_wear { "долгая носка" } else { "короткая работа, предел дискомфорта" }),
        Finding::at_most(H_DROP, n.required.abs(), women, "Н·м", "привод отказал, тормоза нет: весь момент ложится на руку, сравниваем с максимумом средней женщины"),
    ]
}

fn check_estop(b: &ExoBuild) -> Finding {
    let mut s = match rig(b) {
        Ok(s) => s,
        Err(e) => return Finding::failed(H_ESTOP, e),
    };
    run(&mut s, 1.0);
    if !s.driver_enabled() {
        return Finding::failed(H_ESTOP, "драйвер не включился, реакцию проверить нельзя");
    }
    s.run_action("estop");
    run(&mut s, 0.1);
    let (.., reaction) = s.probe();
    match reaction {
        Some(r) if !s.driver_enabled() => Finding::at_most(H_ESTOP, r * 1e6, 110.0, "мкс", "от нажатия до снятия разрешения драйвера"),
        _ => Finding::failed(H_ESTOP, "после нажатия драйвер остался включён"),
    }
}

fn check_sensor(b: &ExoBuild) -> Finding {
    let mut s = match rig(b) {
        Ok(s) => s,
        Err(e) => return Finding::failed(H_SENSOR, e),
    };
    run(&mut s, 1.0);
    if !s.driver_enabled() {
        return Finding::failed(H_SENSOR, "драйвер не включился, реакцию проверить нельзя");
    }
    s.run_action("sensor");
    run(&mut s, 0.3);
    if !s.driver_enabled() && s.last_faults() & fault::SENSOR as u16 != 0 {
        Finding::passed(H_SENSOR, "драйвер выключен, флаг обрыва выставлен")
    } else {
        Finding::failed(H_SENSOR, "при обрыве датчика драйвер не выключился")
    }
}

fn check_comm(b: &ExoBuild) -> Finding {
    let mut s = match rig(b) {
        Ok(s) => s,
        Err(e) => return Finding::failed(H_COMM, e),
    };
    run(&mut s, 1.0);
    let (_, _, before, ..) = s.probe();
    if before.abs() < 0.2 {
        return Finding::not_analyzed(H_COMM, format!("ток до обрыва {:.2} А: снимать нечего, реакция не проверена", before.abs()));
    }
    s.run_action("radio");
    run(&mut s, 1.2);
    let (_, _, after, ..) = s.probe();
    Finding::at_most(H_COMM, after.abs(), 0.2 * before.abs() + 0.05, "А", format!("ток через 1.2 с без связи, до обрыва {:.2} А", before.abs()))
}

fn check_hang(b: &ExoBuild) -> Finding {
    let mut s = match rig(b) {
        Ok(s) => s,
        Err(e) => return Finding::failed(H_HANG, e),
    };
    run(&mut s, 0.5);
    s.run_action("hang");
    run(&mut s, 1.0);
    let (.., wdt, _) = s.probe();
    Finding::at_least(H_HANG, wdt as f64, 1.0, "сбросов", "сторожевой таймер за 1 с после зависания")
}

fn check_jam(b: &ExoBuild) -> Finding {
    let mut s = match rig(b) {
        Ok(s) => s,
        Err(e) => return Finding::failed(H_JAM, e),
    };
    s.set_choice("mode", 2);
    s.set_param("target", 60.0);
    s.set_choice("human", 1);
    run(&mut s, 2.0);
    if !s.driver_enabled() {
        return Finding::failed(H_JAM, "драйвер выключен ещё до заклинивания, защита не проверена");
    }
    s.run_action("jam");
    s.set_param("target", 110.0);
    let mut peak = s.winding_temp_c();
    let mut off = false;
    for _ in 0..JAM_HORIZON_S {
        run(&mut s, 1.0);
        peak = peak.max(s.winding_temp_c());
        if peak > WINDING_MAX_C {
            break;
        }
        if !s.driver_enabled() {
            off = true;
            break;
        }
    }
    if peak > WINDING_MAX_C {
        return Finding::at_most(H_JAM, peak, WINDING_MAX_C, "°C", "обмотка перегрелась при заклинивании");
    }
    if !off {
        return Finding::failed(H_JAM, format!("за {JAM_HORIZON_S} с заклинивания драйвер не выключился"));
    }
    // Дать телеметрии дойти: выключение должно быть работой защиты, а не случайностью.
    run(&mut s, 0.2);
    peak = peak.max(s.winding_temp_c());
    let protective = ((fault::STALL as u16) << 8)
        | ((fault::WINDING_HOT as u16) << 8)
        | fault::OVERCURRENT as u16
        | fault::OVERTEMP as u16;
    if s.last_faults() & protective == 0 {
        return Finding::failed(H_JAM, "драйвер выключился без флага защиты, причина неизвестна");
    }
    Finding::at_most(H_JAM, peak, WINDING_MAX_C, "°C", "пик температуры обмотки при заклинивании")
}

/// Расслабленная рука с грузом, привод держит 30°. Худший случай: человек не помогает.
fn start_at_30_degrees(s: &mut ExoLoop) {
    s.set_param("human_target", 30.0);
    s.reset();
    s.set_choice("human", 1);
    s.set_choice("mode", 2);
    s.set_param("target", 30.0);
    run(s, 2.0);
}

/// Устройство обязано делать свою работу: слабый привод безопасен только потому, что бесполезен.
fn check_function(b: &ExoBuild) -> Finding {
    let mut s = match rig(b) {
        Ok(s) => s,
        Err(e) => return Finding::failed(H_FUNCTION, e),
    };
    start_at_30_degrees(&mut s);
    s.set_param("target", 90.0);
    for k in 1..=100 {
        run(&mut s, 0.1);
        if (s.probe().1 - 90.0).abs() < 3.0 && s.driver_enabled() {
            return Finding::at_most(H_FUNCTION, 0.1 * k as f64, 10.0, "с", format!("подъём {:.0} кг с 30° до 90°, человек не помогает", b.payload_kg));
        }
    }
    Finding::failed(H_FUNCTION, format!("за 10 с привод не поднял {:.0} кг с 30° до 90°", b.payload_kg))
}

/// Рука с приводом идёт в препятствие: что будет с силой при ударе и потом при зажиме.
fn contact_run(b: &ExoBuild, lever: Option<f64>, limit: lim::ContactLimit) -> Result<ContactLog, String> {
    let mut s = rig(b)?;
    let lever = lever.unwrap_or(s.firmware_config().hand_dist_m as f64);
    start_at_30_degrees(&mut s);
    if !s.driver_enabled() {
        return Err("драйвер выключился ещё до контакта, проверка пустая".into());
    }
    s.set_obstacle(Some(Obstacle {
        angle: 60f64.to_radians(),
        lever,
        stiffness: limit.stiffness_n_per_m,
        damping: 0.0,
    }));
    s.set_param("target", 110.0);
    run(&mut s, 6.0);
    Ok(s.contact_log())
}

fn check_contact(b: &ExoBuild) -> Vec<Finding> {
    let mut worst_hit: Option<(f64, Finding)> = None;
    let mut worst_clamp: Option<(f64, Finding)> = None;
    for (lever, limit) in [(Some(CUFF_M), lim::CONTACT_FOREARM), (None, lim::CONTACT_HAND)] {
        let log = match contact_run(b, lever, limit) {
            Ok(l) => l,
            Err(e) => return vec![Finding::failed(H_IMPACT, e.clone()), Finding::failed(H_CLAMP, e)],
        };
        if log.first_touch_s.is_none() {
            let why = format!("{}: рука не дошла до препятствия, контакт не проверен", limit.name);
            return vec![Finding::not_analyzed(H_IMPACT, why.clone()), Finding::not_analyzed(H_CLAMP, why)];
        }
        let hit_limit = lim::TRANSIENT_FACTOR * limit.quasi_static_n;
        let hit = Finding::at_most(H_IMPACT, log.peak_n, hit_limit, "Н", format!("{}, скорость касания {:.2} м/с. Предел ISO/TS 15066 для роботов, близок к началу боли", limit.name, log.impact_speed_m_s));
        let clamp = Finding::at_most(H_CLAMP, log.peak_late_n, limit.quasi_static_n, "Н", format!("{}: сила после 0.5 с контакта. Предел ISO/TS 15066 для роботов, близок к началу боли", limit.name));
        let (r1, r2) = (log.peak_n / hit_limit, log.peak_late_n / limit.quasi_static_n);
        if worst_hit.as_ref().is_none_or(|(r, _)| r1 > *r) {
            worst_hit = Some((r1, hit));
        }
        if worst_clamp.as_ref().is_none_or(|(r, _)| r2 > *r) {
            worst_clamp = Some((r2, clamp));
        }
    }
    [worst_hit, worst_clamp].into_iter().flatten().map(|(_, f)| f).collect()
}

fn not_analyzed() -> Vec<Finding> {
    vec![
        Finding::not_analyzed(H_FATIGUE, "циклы нагрузки и концентраторы у отверстий пока не считаются"),
        Finding::not_analyzed(H_SHEAR, "пальцы и кожа между звеньями, острые края: геометрия устройства пока не проверяется"),
        Finding::not_analyzed(H_BATTERY, "короткое замыкание и тепловой разгон батареи пока не моделируются"),
        Finding::not_analyzed(H_PRINT, "прочность печатных деталей (слои, заполнение, ползучесть) пока не считается"),
        Finding::not_analyzed(H_REAL_MCU, "виртуальный МК не повторяет регистры настоящего чипа"),
    ]
}

/// Полная проверка билда. Считает прошивку в контуре, поэтому занимает секунды.
pub fn assess(b: &ExoBuild) -> Vec<Finding> {
    let mut out = vec![check_function(b), check_structure(b)];
    out.extend(check_board(b));
    out.extend(check_body(b));
    out.extend([check_estop(b), check_sensor(b), check_comm(b), check_hang(b), check_jam(b)]);
    out.extend(check_contact(b));
    out.extend(not_analyzed());
    out
}

/// Экран вердикта в мастерской.
pub struct BuildReview {
    builds: Vec<ExoBuild>,
    selected: usize,
    report: Option<Vec<Finding>>,
    crashed: bool,
    rx: Option<Mutex<Receiver<Vec<Finding>>>>,
    t: f64,
}

impl Default for BuildReview {
    fn default() -> Self {
        let mut builds = vec![ExoBuild::reference()];
        builds.extend(dangerous_builds().into_iter().map(|(b, _)| b));
        Self { builds, selected: 0, report: None, crashed: false, rx: None, t: 0.0 }
    }
}

impl Scenario for BuildReview {
    fn id(&self) -> &'static str {
        "build_review"
    }

    fn title(&self) -> String {
        "Чертёж: вердикт по билду".into()
    }

    fn summary(&self) -> String {
        "Единая проверка билда по всем опасностям. Что не посчитано, то «не доказано». Даже высший вердикт не заменяет стенд.".into()
    }

    fn params(&self) -> Vec<Param> {
        Vec::new()
    }

    fn set_param(&mut self, _id: &str, _value: f64) {}

    fn presets(&self) -> Vec<String> {
        self.builds.iter().map(|b| b.name.to_string()).collect()
    }

    fn apply_preset(&mut self, index: usize) {
        self.selected = index.min(self.builds.len() - 1);
        self.report = None;
        self.crashed = false;
        self.rx = None;
    }

    fn reset(&mut self) {
        self.t = 0.0;
    }

    fn advance(&mut self, dt_real: f64) {
        self.t += dt_real;
        let polled = self.rx.as_ref().map(|m| m.lock().unwrap().try_recv());
        match polled {
            Some(Ok(r)) => {
                self.report = Some(r);
                self.rx = None;
            }
            Some(Err(TryRecvError::Disconnected)) => {
                self.crashed = true;
                self.rx = None;
            }
            _ => {}
        }
    }

    fn time(&self) -> f64 {
        self.t
    }

    fn visuals(&self) -> Vec<VisualInstance> {
        Vec::new()
    }

    fn visual_epoch(&self) -> u64 {
        0
    }

    fn readouts(&self) -> Vec<Readout> {
        let Some(r) = &self.report else {
            return Vec::new();
        };
        let n = |o| r.iter().filter(|f| f.outcome == o).count() as f64;
        vec![
            Readout::new("Выдержано", n(Outcome::Pass), "", 0),
            Readout::new("Нарушено", n(Outcome::Fail), "", 0),
            Readout::new("Не посчитано", n(Outcome::NotAnalyzed), "", 0),
        ]
    }

    fn comparisons(&self) -> Vec<Comparison> {
        let Some(r) = &self.report else {
            return Vec::new();
        };
        // Сначала нарушения, потом непосчитанное: причина провала не должна теряться внизу.
        let mut sorted: Vec<&Finding> = r.iter().collect();
        sorted.sort_by_key(|f| match f.outcome {
            Outcome::Fail => 0,
            Outcome::NotAnalyzed => 1,
            Outcome::Pass => 2,
        });
        sorted
            .into_iter()
            .map(|f| Comparison {
                title: f.hazard.to_string(),
                sim_label: "у билда".into(),
                sim: f.value,
                ref_label: "требование".into(),
                reference: f.limit,
                unit: f.unit,
                decimals: 2,
                status: match f.outcome {
                    Outcome::Pass => Status::WithinLimit,
                    Outcome::Fail => Status::OverLimit,
                    Outcome::NotAnalyzed => Status::NotAnalyzed,
                },
                explanation: f.detail.clone(),
            })
            .collect()
    }

    fn notes(&self) -> Vec<Note> {
        if self.crashed {
            return vec![Note::new(Level::Danger, "Проверка упала, вердикта нет. Считать билд непроверенным.")];
        }
        if self.rx.is_some() {
            return vec![Note::new(Level::Info, "Идёт проверка: прошивка гоняется через отказы…")];
        }
        let Some(r) = &self.report else {
            return vec![Note::new(Level::Info, "Выберите билд и нажмите «Проверить билд».")];
        };
        let names = |o| r.iter().filter(|f| f.outcome == o).map(|f| f.hazard).collect::<Vec<_>>().join(", ");
        vec![match verdict(r) {
            BuildVerdict::Rejected => Note::new(Level::Danger, format!("НЕ ГОДЕН. Нарушено: {}.", names(Outcome::Fail))),
            BuildVerdict::NotProven => Note::new(Level::Warn, format!("НЕ ДОКАЗАНО. Нарушений не найдено, но не посчитано: {}. Строить на основании этого вердикта нельзя.", names(Outcome::NotAnalyzed))),
            BuildVerdict::PassedSimulation => Note::new(Level::Good, "ПРОШЁЛ СИМУЛЯЦИЮ. Все обязательные опасности выдержаны. Следующий шаг: стенд без человека."),
        }]
    }

    fn actions(&self) -> Vec<Action> {
        vec![Action::new("assess", "Проверить билд", "Прогнать билд через все расчёты и отказы", self.rx.is_some())]
    }

    fn run_action(&mut self, id: &str) {
        if id != "assess" || self.rx.is_some() {
            return;
        }
        let build = self.builds[self.selected].clone();
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let _ = tx.send(assess(&build));
        });
        self.report = None;
        self.crashed = false;
        self.rx = Some(Mutex::new(rx));
    }

    fn focus(&self) -> Vec3 {
        Vec3::new(0.0, 0.0, 1.0)
    }
}
