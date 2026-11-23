//! Единая оценка сборки для вердикта.
//!
//! Правила, которые нельзя ослаблять:
//! * `Pass` выдаётся только при конечных значении и пределе и выполненном неравенстве;
//! * NaN, бесконечность или физически невозможный вход дают `Fail` по всем видам проверок:
//!   расчёт на таком входе не выполняется вовсе, потому что `f64::max` молча проглатывает NaN;
//! * нет данных — `NotAnalyzed`, и это не `Pass`: общий итог [`overall`] тоже не будет `Pass`;
//! * провод обязан выдержать КЗ при отключении **одним предохранителем**: MOSFET BMS может
//!   отказать в короткозамкнутом состоянии, поэтому на него как на единственную защиту
//!   рассчитывать нельзя. BMS проверяется отдельно.

use sim_env::contact::{ContactClass, burn_threshold_c};

use crate::cells::{CellFormat, CellSpec};
use crate::fault::{FaultCurrents, FaultPoint, fault_table};
use crate::pack::{ChargeProfile, LoadProfile, LoadStep, PackDesign};
use crate::protection::{BmsSpec, FuseSpec};
use crate::thermal::{ThermalResult, simulate_temperature};
use crate::wire::{CU_MELT_C, Conductor, WireRun, adiabatic_time_s};

/// Итог проверки.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Предел выполнен.
    Pass,
    /// Предел нарушен или вход некорректен.
    Fail,
    /// Данных для вывода нет.
    NotAnalyzed,
}

/// Вид проверки.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BatteryCheck {
    /// Ток КЗ против отключающей способности в каждой точке.
    ShortCircuit,
    /// Нагрев провода: рабочий, при перегрузке и при КЗ.
    WireHeating,
    /// Предохранитель и BMS.
    Protection,
    /// Ток ячейки.
    CellCurrent,
    /// Температура ячейки против окна даташита.
    CellTemperature,
    /// Запас до начала саморазогрева по химии.
    ThermalRunaway,
    /// Окно заряда.
    ChargeWindow,
    /// Температура поверхности у кожи.
    SkinContact,
    /// Диапазон напряжений.
    VoltageRange,
}

/// Все виды проверок: каждый присутствует в результате [`assess_pack`].
pub const ALL_CHECKS: [BatteryCheck; 9] = [
    BatteryCheck::ShortCircuit,
    BatteryCheck::WireHeating,
    BatteryCheck::Protection,
    BatteryCheck::CellCurrent,
    BatteryCheck::CellTemperature,
    BatteryCheck::ThermalRunaway,
    BatteryCheck::ChargeWindow,
    BatteryCheck::SkinContact,
    BatteryCheck::VoltageRange,
];

impl BatteryCheck {
    /// Название для интерфейса.
    pub fn name(self) -> &'static str {
        match self {
            BatteryCheck::ShortCircuit => "короткое замыкание",
            BatteryCheck::WireHeating => "нагрев провода",
            BatteryCheck::Protection => "защита",
            BatteryCheck::CellCurrent => "ток ячейки",
            BatteryCheck::CellTemperature => "температура ячейки",
            BatteryCheck::ThermalRunaway => "тепловой разгон",
            BatteryCheck::ChargeWindow => "окно заряда",
            BatteryCheck::SkinContact => "контакт с кожей",
            BatteryCheck::VoltageRange => "диапазон напряжений",
        }
    }
}

/// Результат одной проверки.
#[derive(Clone, Debug, PartialEq)]
pub struct BatteryFinding {
    /// Вид проверки.
    pub kind: BatteryCheck,
    /// Итог.
    pub outcome: Outcome,
    /// Расчётное значение (при `NotAnalyzed` — 0).
    pub value: f64,
    /// Предел (при `NotAnalyzed` — 0).
    pub limit: f64,
    /// Единица.
    pub unit: &'static str,
    /// Пояснение и источник предела.
    pub detail: String,
}

/// Общий итог: любой `Fail` → `Fail`; иначе любой `NotAnalyzed` или пустой список →
/// `NotAnalyzed`; `Pass` — только если пройдено всё.
pub fn overall(findings: &[BatteryFinding]) -> Outcome {
    if findings.iter().any(|f| f.outcome == Outcome::Fail) {
        Outcome::Fail
    } else if findings.is_empty() || findings.iter().any(|f| f.outcome == Outcome::NotAnalyzed) {
        Outcome::NotAnalyzed
    } else {
        Outcome::Pass
    }
}

fn finding(
    kind: BatteryCheck,
    ok: bool,
    value: f64,
    limit: f64,
    unit: &'static str,
    detail: String,
) -> BatteryFinding {
    let finite = value.is_finite() && limit.is_finite();
    BatteryFinding {
        kind,
        outcome: if ok && finite {
            Outcome::Pass
        } else {
            Outcome::Fail
        },
        value,
        limit,
        unit,
        detail,
    }
}

/// `Pass`, если `value ≤ limit`.
fn le(k: BatteryCheck, value: f64, limit: f64, unit: &'static str, d: String) -> BatteryFinding {
    finding(k, value <= limit, value, limit, unit, d)
}

/// `Pass`, если `value ≥ limit`.
fn ge(k: BatteryCheck, value: f64, limit: f64, unit: &'static str, d: String) -> BatteryFinding {
    finding(k, value >= limit, value, limit, unit, d)
}

fn fail(k: BatteryCheck, value: f64, limit: f64, unit: &'static str, d: String) -> BatteryFinding {
    finding(k, false, value, limit, unit, d)
}

fn na(kind: BatteryCheck, unit: &'static str, detail: impl Into<String>) -> BatteryFinding {
    BatteryFinding {
        kind,
        outcome: Outcome::NotAnalyzed,
        value: 0.0,
        limit: 0.0,
        unit,
        detail: detail.into(),
    }
}

// ---------------------------------------------------------------------------
// Проверка входа
// ---------------------------------------------------------------------------

struct Input(Vec<String>);

impl Input {
    fn positive(&mut self, name: &str, v: f64) {
        if !(v.is_finite() && v > 0.0) {
            self.0.push(format!("{name} = {v} (нужно конечное > 0)"));
        }
    }

    fn non_negative(&mut self, name: &str, v: f64) {
        if !(v.is_finite() && v >= 0.0) {
            self.0.push(format!("{name} = {v} (нужно конечное ≥ 0)"));
        }
    }

    fn range(&mut self, name: &str, v: f64, lo: f64, hi: f64) {
        if !(v.is_finite() && v >= lo && v <= hi) {
            self.0.push(format!("{name} = {v} (нужно {lo}…{hi})"));
        }
    }

    fn opt_positive(&mut self, name: &str, v: Option<f64>) {
        if let Some(v) = v {
            self.positive(name, v);
        }
    }

    fn ordered(&mut self, what: &str, lo: Option<f64>, hi: Option<f64>) {
        if let (Some(a), Some(b)) = (lo, hi)
            && a >= b
        {
            self.0
                .push(format!("{what}: нижняя граница {a} не меньше верхней {b}"));
        }
    }

    fn not_inverted(&mut self, lo: f64, hi: f64) {
        if lo > hi {
            self.0.push(format!(
                "наименьшая температура {lo} больше наибольшей {hi}"
            ));
        }
    }
}

fn validate_cell(c: &CellSpec, e: &mut Input) {
    for (name, p) in c.params() {
        let Some(v) = p.value else { continue };
        if name.contains("°C") || name == "граница холода" {
            e.range(name, v, -100.0, 200.0);
        } else {
            e.positive(name, v);
        }
    }
    e.ordered(
        "конечное и номинальное напряжение",
        c.v_cutoff.value,
        c.v_nominal.value,
    );
    e.ordered(
        "номинальное и зарядное напряжение",
        c.v_nominal.value,
        c.v_charge_max.value,
    );
    e.ordered(
        "окно заряда",
        c.t_charge_min_c.value,
        c.t_charge_max_c.value,
    );
    e.ordered(
        "окно разряда",
        c.t_discharge_min_c.value,
        c.t_discharge_max_c.value,
    );
    if let (Some(lo), Some(hi)) = (c.r_low_ohm.value, c.r_dc_high_ohm.value)
        && lo > hi
    {
        e.0.push(format!("нижнее сопротивление {lo} больше верхнего {hi}"));
    }
    let mut last = f64::NEG_INFINITY;
    for l in &c.charge_limits {
        e.range("предел заряда, °C", l.up_to_c, -100.0, 200.0);
        e.positive("предел тока заряда", l.current_a);
        if l.up_to_c <= last {
            e.0.push("таблица пределов заряда не упорядочена".into());
        }
        last = l.up_to_c;
    }
    match c.format {
        CellFormat::Cylindrical {
            diameter_mm,
            height_mm,
        } => {
            e.positive("диаметр ячейки", diameter_mm);
            e.positive("высота ячейки", height_mm);
        }
        CellFormat::Pouch {
            length_mm,
            width_mm,
            thickness_mm,
        } => {
            e.opt_positive("длина пачки", length_mm);
            e.opt_positive("ширина пачки", width_mm);
            e.opt_positive("толщина пачки", thickness_mm);
        }
    }
}

fn validate_wire(w: &WireRun, e: &mut Input) {
    match w.conductor {
        Conductor::Awg(n) if n > 40 => e.0.push(format!("калибр AWG {n} вне диапазона 0…40")),
        Conductor::Awg(_) => {}
        Conductor::Mm2(s) => e.positive("сечение провода", s),
    }
    e.positive("длина провода", w.length_m);
    e.opt_positive("наружный диаметр провода", w.outer_diameter_mm);
}

fn validate_fuse(f: &FuseSpec, wire_length_m: f64, e: &mut Input) {
    e.positive("номинал предохранителя", f.rating_a);
    e.opt_positive("напряжение предохранителя", f.voltage_dc_v);
    e.opt_positive("отключающая способность", f.breaking_a);
    e.opt_positive("I²t предохранителя", f.melt_i2t_a2s);
    if let Some(r) = f.cold_resistance_ohm {
        e.non_negative("сопротивление предохранителя", r);
    }
    e.non_negative("положение предохранителя", f.position_m);
    if f.position_m > wire_length_m {
        e.0.push(format!(
            "предохранитель на {} м дальше конца провода {} м",
            f.position_m, wire_length_m
        ));
    }
    let mut last = 0.0;
    for p in &f.tcc {
        e.positive("кратность тока характеристики", p.multiple);
        e.opt_positive("наименьшее время характеристики", p.t_min_s);
        e.opt_positive("наибольшее время характеристики", p.t_max_s);
        if p.multiple <= last {
            e.0.push("время-токовая характеристика не упорядочена".into());
        }
        last = p.multiple;
    }
}

fn validate_bms(b: &BmsSpec, e: &mut Input) {
    e.positive("порог КЗ, наименьший", b.scd_trip_min_a);
    e.positive("порог КЗ, наибольший", b.scd_trip_max_a);
    e.positive("задержка КЗ", b.scd_delay_max_s);
    e.positive("порог перегрузки, наименьший", b.ocd_trip_min_a);
    e.positive("порог перегрузки, наибольший", b.ocd_trip_max_a);
    e.positive("задержка перегрузки, наименьшая", b.ocd_delay_min_s);
    e.positive("задержка перегрузки, наибольшая", b.ocd_delay_max_s);
    e.opt_positive("порог перезаряда", b.cell_ovp_v);
    e.opt_positive("порог переразряда", b.cell_uvp_v);
    e.positive("Vds ключа", b.mosfet.vds_max_v);
    e.positive("импульсный ток ключа", b.mosfet.id_pulse_a);
    e.positive("длительность импульса ключа", b.mosfet.id_pulse_width_s);
    e.non_negative("сопротивление ключа", b.mosfet.rds_on_ohm);
    e.opt_positive("лавинная энергия ключа", b.mosfet.avalanche_energy_j);
    if b.scd_trip_min_a > b.scd_trip_max_a || b.ocd_trip_min_a > b.ocd_trip_max_a {
        e.0.push("наименьший порог BMS больше наибольшего".into());
    }
    if b.ocd_delay_min_s > b.ocd_delay_max_s {
        e.0.push("наименьшая задержка BMS больше наибольшей".into());
    }
}

/// Список ошибок входа; пустой — вход корректен.
pub fn validate(pack: &PackDesign, load: &LoadProfile) -> Vec<String> {
    let mut e = Input(Vec::new());
    if pack.series == 0 || pack.series > 1000 {
        e.0.push(format!("последовательно {} (нужно 1…1000)", pack.series));
    }
    if pack.parallel == 0 || pack.parallel > 1000 {
        e.0.push(format!("параллельно {} (нужно 1…1000)", pack.parallel));
    }
    validate_cell(&pack.cell, &mut e);
    e.range("разброс DCIR", pack.cell_resistance_spread, 0.0, 0.9);
    e.non_negative("разбаланс групп", pack.group_voltage_spread_v);
    e.non_negative("сопротивление перемычек", pack.interconnect_r_ohm);
    e.non_negative("сопротивление контактов", pack.contact_r_ohm);
    validate_wire(&pack.wire, &mut e);
    if let Some(f) = &pack.fuse {
        validate_fuse(f, pack.wire.length_m, &mut e);
    }
    if let Some(b) = &pack.bms {
        validate_bms(b, &mut e);
    }
    if let Some(x) = &pack.enclosure {
        e.positive("площадь корпуса", x.outer_area_m2);
        e.positive("высота корпуса", x.height_m);
        e.non_negative("толщина стенки", x.wall_thickness_m);
        e.positive("теплопроводность стенки", x.wall_k_w_mk);
        e.positive("внутренняя теплоотдача", x.h_inner_w_m2k);
        e.range("излучательная способность", x.emissivity, 0.0, 1.0);
        e.range("доля площади у тела", x.body_side_fraction, 0.0, 0.99);
    }
    if let Some(x) = &pack.skin {
        e.non_negative("толщина прокладки", x.spacer_thickness_m);
        e.positive("теплопроводность прокладки", x.spacer_k_w_mk);
        e.positive("время контакта", x.contact_time_s);
        e.range("доля поверхности тела", x.contact_body_fraction, 0.0, 1.0);
    }

    e.range(
        "наименьшая температура среды",
        load.ambient_min_c,
        -60.0,
        120.0,
    );
    e.range(
        "наибольшая температура среды",
        load.ambient_max_c,
        -60.0,
        120.0,
    );
    e.not_inverted(load.ambient_min_c, load.ambient_max_c);
    if load.cycles == 0 {
        e.0.push("число циклов 0".into());
    }
    for s in &load.steps {
        e.non_negative("ток шага", s.current_a);
        e.positive("длительность шага", s.duration_s);
    }
    e.opt_positive("наименьшее напряжение нагрузки", load.load_v_min);
    e.opt_positive("наибольшее напряжение нагрузки", load.load_v_max);
    if let Some(c) = &load.charge {
        e.positive("ток заряда", c.current_a);
        e.positive("напряжение заряда", c.voltage_v);
        e.range("температура заряда, минимум", c.ambient_min_c, -60.0, 120.0);
        e.range(
            "температура заряда, максимум",
            c.ambient_max_c,
            -60.0,
            120.0,
        );
        e.not_inverted(c.ambient_min_c, c.ambient_max_c);
    }
    e.0
}

// ---------------------------------------------------------------------------
// Расчётный контекст
// ---------------------------------------------------------------------------

struct Ctx<'a> {
    pack: &'a PackDesign,
    load: &'a LoadProfile,
    faults: Vec<FaultCurrents>,
    v_max: Option<f64>,
    /// Температура жилы в рабочем режиме перед КЗ, °C.
    theta0_c: f64,
    thermal: Option<ThermalResult>,
}

/// Время, после которого считается, что неотключённый ток течёт бесконечно, с.
const NO_CLEAR_S: f64 = 3600.0;
const SCAN_POINTS: usize = 48;
/// Допуск напряжения заряда: ±0.05 В в даташитах 30Q и VTC6.
const CHARGE_V_TOLERANCE: f64 = 0.05;

struct WireScan {
    theta_c: f64,
    current_a: f64,
    time_s: f64,
    all_cleared: bool,
}

/// Худшая температура провода на всём диапазоне токов КЗ: у защиты с обратнозависимой
/// характеристикой опасным может оказаться не наибольший ток, а средний.
///
/// `knots` — токи, где характеристика ломается или скачет (строки таблицы, пороги BMS): сетка
/// могла бы перешагнуть точку сразу под порогом, где отключение самое долгое.
fn scan_wire(
    w: &WireRun,
    i_min: f64,
    i_max: f64,
    theta0: f64,
    t_amb: f64,
    knots: &[f64],
    clear: &dyn Fn(f64) -> Option<f64>,
) -> WireScan {
    let mut out = WireScan {
        theta_c: f64::NEG_INFINITY,
        current_a: i_max,
        time_s: 0.0,
        all_cleared: true,
    };
    let ratio = if i_max > i_min { i_max / i_min } else { 1.0 };
    let grid = (0..SCAN_POINTS).map(|k| {
        if k + 1 == SCAN_POINTS {
            i_max
        } else {
            i_min * ratio.powf(k as f64 / (SCAN_POINTS - 1) as f64)
        }
    });
    let inside = knots
        .iter()
        .flat_map(|&x| [x, x * (1.0 - 1e-9)])
        .filter(|&x| x >= i_min && x <= i_max);
    for i in grid.chain(inside) {
        let t = match clear(i) {
            Some(t) => t,
            None => {
                out.all_cleared = false;
                NO_CLEAR_S
            }
        };
        let theta = w.temperature_after_c(i, t, theta0, t_amb);
        if theta > out.theta_c {
            out.theta_c = theta;
            out.current_a = i;
            out.time_s = t;
        }
    }
    out
}

/// Время адиабатического нагрева до `target`; если жила уже горячее — ноль.
fn time_to_temp(i: f64, s_mm2: f64, theta0: f64, target: f64) -> f64 {
    if theta0 >= target {
        0.0
    } else {
        adiabatic_time_s(i, s_mm2, theta0, target)
    }
}

fn min_of(values: impl Iterator<Item = f64>) -> f64 {
    values.fold(f64::INFINITY, |m, x| if x < m { x } else { m })
}

fn max_of(values: impl Iterator<Item = f64>) -> f64 {
    values.fold(0.0, |m, x| if x > m { x } else { m })
}

// ---------------------------------------------------------------------------
// Проверки
// ---------------------------------------------------------------------------

fn check_short_circuit(c: &Ctx, out: &mut Vec<BatteryFinding>) {
    use BatteryCheck::ShortCircuit as K;
    if c.faults.is_empty() {
        out.push(na(
            K,
            "А",
            "нет напряжения или сопротивления ячейки: ток КЗ не посчитан",
        ));
        return;
    }
    for f in &c.faults {
        let place = f.point.name();
        match &c.pack.fuse {
            None => out.push(fail(
                K,
                f.i_max_a,
                0.0,
                "А",
                format!(
                    "КЗ {place}: ток до {:.0} А, предохранителя нет — отключать нечем",
                    f.i_max_a
                ),
            )),
            Some(_) if !f.fuse_in_path => out.push(fail(
                K,
                f.i_max_a,
                0.0,
                "А",
                format!(
                    "КЗ {place}: ток до {:.0} А идёт мимо предохранителя — он стоит дальше по \
                     проводу; поставьте его у клеммы батареи",
                    f.i_max_a
                ),
            )),
            Some(fuse) => match fuse.breaking_a {
                Some(b) => out.push(le(
                    K,
                    f.i_max_a,
                    b,
                    "А",
                    format!(
                        "КЗ {place}: наибольший ток {:.0} А, наименьший {:.0} А; отключающая \
                         способность {} — {b:.0} А ({})",
                        f.i_max_a, f.i_min_a, fuse.name, fuse.source
                    ),
                )),
                None => out.push(na(
                    K,
                    "А",
                    format!(
                        "КЗ {place}: ток до {:.0} А, отключающая способность {} не задана",
                        f.i_max_a, fuse.name
                    ),
                )),
            },
        }
    }
}

fn check_wire(c: &Ctx, out: &mut Vec<BatteryFinding>) {
    use BatteryCheck::WireHeating as K;
    let w = &c.pack.wire;
    let t_amb = c.load.ambient_max_c;
    let ins = w.insulation;

    if c.load.steps.is_empty() {
        out.push(na(
            K,
            "°C",
            "профиль нагрузки пуст: рабочий нагрев провода не посчитан",
        ));
    } else {
        out.push(le(
            K,
            c.theta0_c,
            ins.continuous_c(),
            "°C",
            format!(
                "рабочий режим: жила {:.1} °C, длительный предел изоляции {} — {:.0} °C ({})",
                c.theta0_c,
                ins.name(),
                ins.continuous_c(),
                ins.source()
            ),
        ));
    }

    let ampacity = w.ampacity_a(t_amb);
    match &c.pack.fuse {
        None => out.push(fail(
            K,
            f64::MAX,
            ampacity,
            "А",
            format!(
                "перегрузка: предохранителя нет, ток не ограничен; провод держит {ampacity:.1} А"
            ),
        )),
        Some(fuse) => match fuse.guaranteed_open_a() {
            Some(open) => out.push(le(
                K,
                open,
                ampacity,
                "А",
                format!(
                    "перегрузка: {} гарантированно сгорает только с {open:.1} А, провод \
                     длительно держит {ampacity:.1} А при {t_amb:.0} °C",
                    fuse.name
                ),
            )),
            None => out.push(na(
                K,
                "А",
                format!(
                    "перегрузка: у {} нет времени срабатывания в данных",
                    fuse.name
                ),
            )),
        },
    }

    if c.faults.is_empty() {
        out.push(na(
            K,
            "°C",
            "ток КЗ не посчитан: нагрев провода при КЗ не оценён",
        ));
        return;
    }
    let limit = ins.short_circuit_c();
    for f in c.faults.iter().filter(|f| f.wire_loop_m > 0.0) {
        let fuse = c.pack.fuse.as_ref().filter(|_| f.fuse_in_path);
        let clear = |i: f64| fuse.and_then(|x| x.max_clear_time_s(i));
        let knots: Vec<f64> = fuse.map_or(Vec::new(), |x| {
            x.tcc.iter().map(|p| p.multiple * x.rating_a).collect()
        });
        let scan = scan_wire(w, f.i_min_a, f.i_max_a, c.theta0_c, t_amb, &knots, &clear);
        let t_ins = time_to_temp(scan.current_a, w.area_mm2(), c.theta0_c, limit);
        let t_melt = time_to_temp(scan.current_a, w.area_mm2(), c.theta0_c, CU_MELT_C);
        let verdict = if scan.theta_c >= CU_MELT_C {
            "жила расплавится раньше, чем сработает защита"
        } else if scan.theta_c > limit {
            "изоляция разрушится раньше, чем сработает защита"
        } else if !scan.all_cleared {
            "защита отключает не весь диапазон токов, провод выдерживает"
        } else {
            "провод выдерживает до отключения"
        };
        out.push(le(
            K,
            scan.theta_c,
            limit,
            "°C",
            format!(
                "КЗ {}: худший ток {:.0} А, отключение за {:.3} с, жила {:.0} °C при пределе \
                 {} {limit:.0} °C; предел изоляции за {t_ins:.3} с, плавление за {t_melt:.3} с \
                 (адиабата IEC 60949); {verdict}",
                f.point.name(),
                scan.current_a,
                scan.time_s,
                scan.theta_c,
                ins.name(),
            ),
        ));
    }
}

fn check_fuse(c: &Ctx, fuse: &FuseSpec, out: &mut Vec<BatteryFinding>) {
    use BatteryCheck::Protection as K;
    match (fuse.voltage_dc_v, c.v_max) {
        (None, _) => out.push(fail(
            K,
            c.v_max.unwrap_or(0.0),
            0.0,
            "В",
            format!(
                "{}: нет номинала по постоянному току; дуга постоянного тока может не погаснуть",
                fuse.name
            ),
        )),
        (Some(_), None) => out.push(na(K, "В", "напряжение батареи неизвестно")),
        (Some(rated), Some(v)) => out.push(le(
            K,
            v,
            rated,
            "В",
            format!(
                "{}: напряжение полного заряда {v:.1} В, номинал {rated:.0} В DC",
                fuse.name
            ),
        )),
    }

    let in_path: Vec<&FaultCurrents> = c.faults.iter().filter(|f| f.fuse_in_path).collect();
    match fuse.guaranteed_open_a() {
        None => out.push(na(
            K,
            "А",
            format!("{}: нет времени срабатывания", fuse.name),
        )),
        Some(_) if in_path.is_empty() => {
            out.push(na(K, "А", "ток КЗ через предохранитель не посчитан"));
        }
        Some(open) => {
            let i_min = min_of(in_path.iter().map(|f| f.i_min_a));
            out.push(ge(
                K,
                i_min,
                open,
                "А",
                format!(
                    "{}: наименьший ток КЗ (холод, конец разряда, старая ячейка) {i_min:.1} А, \
                     гарантированное срабатывание с {open:.1} А",
                    fuse.name
                ),
            ));
        }
    }

    if c.load.steps.is_empty() {
        return;
    }
    let mut worst: Option<(f64, f64, f64)> = None;
    for s in &c.load.steps {
        if let Some(t_melt) = fuse.min_melt_time_s(s.current_a) {
            let ratio = s.duration_s / t_melt;
            if worst.is_none_or(|w| ratio > w.0) {
                worst = Some((ratio, s.current_a, t_melt));
            }
        }
    }
    match worst {
        Some((ratio, i, t)) => out.push(le(
            K,
            ratio,
            1.0,
            "доля",
            format!(
                "{}: шаг нагрузки {i:.1} А длится {ratio:.2} от наименьшего времени плавления \
                 {t:.3} с",
                fuse.name
            ),
        )),
        None => {
            let floor = fuse.tcc.first().map_or(1.0, |p| p.multiple) * fuse.rating_a;
            out.push(le(
                K,
                c.load.peak_a(),
                floor,
                "А",
                format!(
                    "{}: пик нагрузки {:.1} А ниже области плавления, ложного срабатывания нет",
                    fuse.name,
                    c.load.peak_a()
                ),
            ));
        }
    }
}

fn check_bms_fault(c: &Ctx, bms: &BmsSpec, out: &mut Vec<BatteryFinding>) {
    use BatteryCheck::Protection as K;
    let m = &bms.mosfet;
    let i_min = min_of(c.faults.iter().map(|f| f.i_min_a));
    let i_max = max_of(c.faults.iter().map(|f| f.i_max_a));
    out.push(ge(
        K,
        i_min,
        bms.ocd_trip_max_a,
        "А",
        format!(
            "{}: наименьший ток КЗ {i_min:.1} А, порог перегрузки с допуском {:.1} А, порог \
             КЗ {:.1} А",
            bms.name, bms.ocd_trip_max_a, bms.scd_trip_max_a
        ),
    ));
    out.push(le(
        K,
        i_max,
        m.id_pulse_a,
        "А",
        format!(
            "ключ BMS: ток КЗ до {i_max:.0} А, импульсный предел {:.0} А",
            m.id_pulse_a
        ),
    ));
    out.push(le(
        K,
        bms.scd_delay_max_s,
        m.id_pulse_width_s,
        "с",
        format!(
            "ключ BMS: задержка КЗ до {:.0} мкс, импульсный ток задан на {:.0} мкс",
            bms.scd_delay_max_s * 1e6,
            m.id_pulse_width_s * 1e6
        ),
    ));
    if i_min < bms.scd_trip_max_a {
        out.push(na(
            K,
            "А",
            format!(
                "ключ BMS: токи КЗ от {i_min:.1} до {:.1} А могут отключаться только по перегрузке, \
                 до {:.3} с; область безопасной работы ключа для такого режима не задана",
                bms.scd_trip_max_a, bms.ocd_delay_max_s
            ),
        ));
    }
    let w = &c.pack.wire;
    let e = 0.5 * w.loop_inductance_h(w.length_m) * i_max * i_max;
    match m.avalanche_energy_j {
        Some(limit) => out.push(le(
            K,
            e,
            limit,
            "Дж",
            format!(
                "ключ BMS: энергия индуктивности провода при отключении {e:.4} Дж, лавинный \
                 предел {limit:.4} Дж (зазор пары 10 мм — оценка)"
            ),
        )),
        None => out.push(na(
            K,
            "Дж",
            format!("ключ BMS: лавинная энергия не задана, выброс {e:.4} Дж не с чем сравнить"),
        )),
    }
    if let Some(f) = c.faults.iter().find(|f| f.point == FaultPoint::AtLoad) {
        let clear = |i: f64| bms.max_clear_time_s(i);
        let scan = scan_wire(
            w,
            f.i_min_a,
            f.i_max_a,
            c.theta0_c,
            c.load.ambient_max_c,
            &[bms.ocd_trip_max_a, bms.scd_trip_max_a],
            &clear,
        );
        let limit = w.insulation.short_circuit_c();
        out.push(le(
            K,
            scan.theta_c,
            limit,
            "°C",
            format!(
                "BMS: при КЗ на нагрузке жила дойдёт до {:.0} °C за {:.4} с (ток {:.0} А), \
                 предел {limit:.0} °C",
                scan.theta_c, scan.time_s, scan.current_a
            ),
        ));
    }
}

fn check_bms(c: &Ctx, bms: &BmsSpec, out: &mut Vec<BatteryFinding>) {
    use BatteryCheck::Protection as K;
    let m = &bms.mosfet;
    if c.faults.is_empty() {
        out.push(na(
            K,
            "А",
            "ток КЗ не посчитан: срабатывание BMS не оценено",
        ));
    } else {
        check_bms_fault(c, bms, out);
    }
    match c.v_max {
        Some(v) => out.push(le(
            K,
            v,
            m.vds_max_v,
            "В",
            format!(
                "ключ BMS: {v:.1} В на батарее, предел сток-исток {:.0} В",
                m.vds_max_v
            ),
        )),
        None => out.push(na(K, "В", "напряжение батареи неизвестно")),
    }
    if c.load.steps.is_empty() {
        return;
    }
    let false_trip = c.load.steps.iter().find(|s| {
        s.current_a >= bms.scd_trip_min_a
            || (s.current_a >= bms.ocd_trip_min_a && s.duration_s >= bms.ocd_delay_min_s)
    });
    let peak = c.load.peak_a();
    match false_trip {
        Some(s) => out.push(fail(
            K,
            s.current_a,
            bms.ocd_trip_min_a,
            "А",
            format!(
                "{}: рабочий ток {:.1} А за {:.3} с вызовет отключение (порог от {:.1} А)",
                bms.name, s.current_a, s.duration_s, bms.ocd_trip_min_a
            ),
        )),
        None => out.push(finding(
            K,
            true,
            peak,
            bms.scd_trip_min_a,
            "А",
            format!(
                "{}: пик нагрузки {peak:.1} А не вызывает ложного отключения",
                bms.name
            ),
        )),
    }
}

fn check_protection(c: &Ctx, out: &mut Vec<BatteryFinding>) {
    use BatteryCheck::Protection as K;
    match &c.pack.fuse {
        Some(f) => check_fuse(c, f, out),
        None => out.push(fail(
            K,
            0.0,
            1.0,
            "шт",
            "нет плавкого предохранителя: ключ BMS может отказать в замкнутом состоянии, и тогда \
             ток КЗ ничем не ограничен. На теле такая сборка недопустима"
                .into(),
        )),
    }
    match &c.pack.bms {
        Some(b) => check_bms(c, b, out),
        None => out.push(fail(
            K,
            0.0,
            1.0,
            "шт",
            "нет BMS: перезаряд, переразряд и перегрузка групп никем не контролируются".into(),
        )),
    }
}

fn check_cell_current(c: &Ctx, out: &mut Vec<BatteryFinding>) {
    use BatteryCheck::CellCurrent as K;
    if c.load.steps.is_empty() {
        out.push(na(K, "А", "профиль нагрузки пуст"));
        return;
    }
    let cell = &c.pack.cell;
    let t = c.load.ambient_min_c;
    let Some(limit) = cell.discharge_limit_a(t) else {
        out.push(na(K, "А", "длительный ток ячейки не задан"));
        return;
    };
    let rms = c.pack.worst_cell_current_a(c.load.rms_a());
    out.push(le(
        K,
        rms,
        limit,
        "А",
        format!(
            "действующий ток худшей ячейки {rms:.2} А (разбаланс ×{:.2}), длительный предел \
             {limit:.2} А при {t:.0} °C — {}",
            c.pack.worst_cell_share(),
            cell.name
        ),
    ));

    // Каждый шаг выше длительного предела обязан уложиться в импульсный: проверка только
    // наибольшего тока пропустила бы чуть меньший, но слишком долгий шаг.
    let over: Vec<(f64, f64)> = c
        .load
        .steps
        .iter()
        .map(|s| (c.pack.worst_cell_current_a(s.current_a), s.duration_s))
        .filter(|&(i, _)| i > limit)
        .collect();
    if over.is_empty() {
        let peak = c.pack.worst_cell_current_a(c.load.peak_a());
        out.push(le(
            K,
            peak,
            limit,
            "А",
            format!("пиковый ток худшей ячейки {peak:.2} А в пределах длительного {limit:.2} А"),
        ));
        return;
    }
    let (Some(ip), Some(ts)) = (cell.i_pulse_a.value, cell.pulse_s.value) else {
        let peak = max_of(over.iter().map(|x| x.0));
        out.push(fail(
            K,
            peak,
            limit,
            "А",
            format!(
                "ток худшей ячейки {peak:.2} А выше длительного предела {limit:.2} А, импульсного \
                 предела в даташите нет"
            ),
        ));
        return;
    };
    for (i, d) in over {
        if d > ts {
            out.push(fail(
                K,
                d,
                ts,
                "с",
                format!(
                    "ток {i:.2} А выше длительного {limit:.2} А и длится {d:.1} с, а импульс \
                     разрешён только на {ts:.0} с"
                ),
            ));
        } else {
            out.push(le(
                K,
                i,
                ip,
                "А",
                format!("импульс {i:.2} А на {d:.1} с, предел {ip:.1} А на {ts:.0} с"),
            ));
        }
    }
}

/// Адиабатический расчёт — только верхняя граница: превышение предела без данных о корпусе
/// означает «вывода нет», а не «опасно». Нечисловой результат остаётся `Fail`.
fn bounded(
    th: &ThermalResult,
    kind: BatteryCheck,
    value: f64,
    limit: f64,
    detail: String,
) -> BatteryFinding {
    if th.adiabatic && value.is_finite() && limit.is_finite() && value > limit {
        na(
            kind,
            "°C",
            format!("{detail}; корпус не задан, а без теплоотвода предел превышен — вывода нет"),
        )
    } else {
        le(kind, value, limit, "°C", detail)
    }
}

fn check_thermal(c: &Ctx, out: &mut Vec<BatteryFinding>) {
    let cell = &c.pack.cell;
    match cell.t_discharge_min_c.value {
        Some(lim) => out.push(ge(
            BatteryCheck::CellTemperature,
            c.load.ambient_min_c,
            lim,
            "°C",
            format!(
                "наименьшая температура работы {:.1} °C, нижняя граница разряда {lim:.0} °C",
                c.load.ambient_min_c
            ),
        )),
        None => out.push(na(
            BatteryCheck::CellTemperature,
            "°C",
            "нижняя граница разряда не задана",
        )),
    }
    let Some(th) = c.thermal else {
        let why = "нет профиля нагрузки, сопротивления, массы или теплоёмкости ячейки";
        out.push(na(BatteryCheck::CellTemperature, "°C", why));
        out.push(na(BatteryCheck::ThermalRunaway, "°C", why));
        return;
    };
    let mode = if th.adiabatic {
        "без теплоотвода (корпус не задан, верхняя граница)"
    } else {
        "с корпусом"
    };
    match cell.t_discharge_max_c.value {
        Some(lim) => out.push(bounded(
            &th,
            BatteryCheck::CellTemperature,
            th.t_max_c,
            lim,
            format!(
                "ячейка нагреется до {:.1} °C {mode}, тепловыделение до {:.1} Вт; верхняя граница \
                 разряда {lim:.0} °C ({})",
                th.t_max_c, th.heat_peak_w, cell.t_discharge_max_c.source
            ),
        )),
        None => out.push(na(
            BatteryCheck::CellTemperature,
            "°C",
            "верхняя граница разряда не задана",
        )),
    }
    let tr = cell.chemistry.runaway();
    let t2 = tr
        .t2_trigger_c
        .map_or("нет данных".to_string(), |x| format!("{x:.0} °C"));
    out.push(bounded(
        &th,
        BatteryCheck::ThermalRunaway,
        th.t_max_c,
        tr.t1_self_heating_c,
        format!(
            "{}: ячейка {:.1} °C {mode}, начало саморазогрева T1 = {:.1} °C, запуск разгона T2: \
             {t2} ({})",
            cell.chemistry.name(),
            th.t_max_c,
            tr.t1_self_heating_c,
            tr.source
        ),
    ));
}

fn check_skin(c: &Ctx, out: &mut Vec<BatteryFinding>) {
    use BatteryCheck::SkinContact as K;
    if !c.pack.worn_on_body {
        out.push(na(K, "°C", "батарея не отмечена как носимая на теле"));
        return;
    }
    let Some(skin) = &c.pack.skin else {
        out.push(na(K, "°C", "не задано прилегание к коже"));
        return;
    };
    if skin.contact_body_fraction >= 0.10 {
        out.push(na(
            K,
            "°C",
            "батарея закрывает 10 % тела и больше: порог ISO 13732-1 для длительного контакта \
             (43 °C) действует только для меньшей площади",
        ));
        return;
    }
    let Some(th) = c.thermal else {
        out.push(na(K, "°C", "температура ячейки не посчитана"));
        return;
    };
    let limit = burn_threshold_c(skin.surface_class, skin.contact_time_s);
    let surface = skin.surface_temp_c(th.t_max_c);
    out.push(bounded(
        &th,
        K,
        surface,
        limit,
        format!(
            "поверхность у кожи {surface:.1} °C при ячейке {:.1} °C, порог ожога ISO 13732-1 за \
             {:.0} с — {limit:.1} °C",
            th.t_max_c, skin.contact_time_s
        ),
    ));
    if !c.load.steps.is_empty() {
        let wire_limit = burn_threshold_c(ContactClass::Plastics, skin.contact_time_s);
        out.push(le(
            K,
            c.theta0_c,
            wire_limit,
            "°C",
            format!(
                "провод может лечь на кожу: жила {:.1} °C (изоляция не вычитается), порог ISO \
                 13732-1 для пластмасс {wire_limit:.1} °C",
                c.theta0_c
            ),
        ));
    }
}

/// Самонагрев при заряде: окно даташита относится к ячейке, а не к воздуху вокруг. Считается
/// заряд от нуля постоянным током: спад тока на участке CV в запас не берётся.
fn check_charge_heating(c: &Ctx, ch: &ChargeProfile, out: &mut Vec<BatteryFinding>) {
    use BatteryCheck::ChargeWindow as K;
    let cell = &c.pack.cell;
    let (Some(q_ah), Some(lim)) = (cell.capacity_typ_ah.value, cell.t_charge_max_c.value) else {
        out.push(na(
            K,
            "°C",
            "нагрев при заряде не посчитан: нет ёмкости или верхней границы заряда",
        ));
        return;
    };
    let duration_s = q_ah * c.pack.parallel as f64 / ch.current_a * 3600.0;
    let profile = LoadProfile {
        steps: vec![LoadStep {
            current_a: ch.current_a,
            duration_s,
        }],
        cycles: 1,
        ambient_min_c: ch.ambient_min_c,
        ambient_max_c: ch.ambient_max_c,
        load_v_min: None,
        load_v_max: None,
        charge: None,
    };
    let Some(th) = simulate_temperature(c.pack, &profile) else {
        out.push(na(
            K,
            "°C",
            "нагрев при заряде не посчитан: нет сопротивления, массы или теплоёмкости ячейки",
        ));
        return;
    };
    out.push(bounded(
        &th,
        K,
        th.t_max_c,
        lim,
        format!(
            "при заряде {:.1} А ячейка нагреется до {:.1} °C, верхняя граница заряда {lim:.0} °C",
            ch.current_a, th.t_max_c
        ),
    ));
}

fn check_charge(c: &Ctx, out: &mut Vec<BatteryFinding>) {
    use BatteryCheck::ChargeWindow as K;
    let Some(ch) = &c.load.charge else {
        out.push(na(K, "°C", "режим заряда не задан"));
        return;
    };
    let cell = &c.pack.cell;
    match cell.t_charge_min_c.value {
        Some(lim) => out.push(ge(
            K,
            ch.ambient_min_c,
            lim,
            "°C",
            format!(
                "заряд при {:.1} °C, нижняя граница {lim:.0} °C: на холоде на аноде осаждается \
                 металлический литий (Waldmann et al., J. Power Sources 384:107, 2018), это \
                 необратимо и ведёт к внутреннему КЗ",
                ch.ambient_min_c
            ),
        )),
        None => out.push(na(K, "°C", "нижняя граница заряда не задана")),
    }
    match cell.t_charge_max_c.value {
        Some(lim) => out.push(le(
            K,
            ch.ambient_max_c,
            lim,
            "°C",
            format!(
                "заряд при {:.1} °C, верхняя граница {lim:.0} °C",
                ch.ambient_max_c
            ),
        )),
        None => out.push(na(K, "°C", "верхняя граница заряда не задана")),
    }
    check_charge_heating(c, ch, out);
    if cell.charge_limits.is_empty() {
        out.push(na(K, "А", "предел тока заряда не задан"));
        return;
    }
    let i_cell = c.pack.worst_cell_current_a(ch.current_a);
    // Таблица ступенчатая: наименьший предел может лежать внутри диапазона, поэтому смотрим
    // обе границы и каждую ступень, попавшую в диапазон.
    let mut limits = vec![
        cell.charge_limit_a(ch.ambient_min_c),
        cell.charge_limit_a(ch.ambient_max_c),
    ];
    limits.extend(
        cell.charge_limits
            .iter()
            .filter(|l| l.up_to_c >= ch.ambient_min_c && l.up_to_c <= ch.ambient_max_c)
            .map(|l| Some(l.current_a)),
    );
    if limits.iter().any(Option::is_none) {
        out.push(fail(
            K,
            i_cell,
            0.0,
            "А",
            format!(
                "для {:.0}…{:.0} °C в таблице даташита нет допустимого тока заряда",
                ch.ambient_min_c, ch.ambient_max_c
            ),
        ));
        return;
    }
    let lim = min_of(limits.into_iter().flatten());
    out.push(le(
        K,
        i_cell,
        lim,
        "А",
        format!(
            "ток заряда худшей ячейки {i_cell:.2} А, предел {lim:.2} А в диапазоне {:.0}…{:.0} °C",
            ch.ambient_min_c, ch.ambient_max_c
        ),
    ));
}

fn check_sag(c: &Ctx, v_end: f64, out: &mut Vec<BatteryFinding>) {
    use BatteryCheck::VoltageRange as K;
    let cell = &c.pack.cell;
    let s = c.pack.series as f64;
    let (Some(v_nom), Some(r_cell)) = (
        cell.v_nominal.value,
        c.pack.cell_r_worst_ohm(c.load.ambient_min_c),
    ) else {
        out.push(na(
            K,
            "В",
            "номинальное напряжение или сопротивление не заданы",
        ));
        return;
    };
    let w = &c.pack.wire;
    let r_path = r_cell * c.pack.s_over_p()
        + c.pack.interconnect_r_ohm
        + c.pack.contact_r_ohm
        + w.loop_resistance_ohm(2.0 * w.length_m, c.theta0_c)
        + c.pack
            .fuse
            .as_ref()
            .and_then(|f| f.cold_resistance_ohm)
            .unwrap_or(0.0)
        + c.pack.bms.as_ref().map_or(0.0, |b| b.mosfet.rds_on_ohm);
    let peak = c.load.peak_a();
    let sag = v_nom * s - peak * r_path;
    let floor = match c.load.load_v_min {
        Some(v) if v > v_end => v,
        _ => v_end,
    };
    out.push(ge(
        K,
        sag,
        floor,
        "В",
        format!(
            "просадка при {peak:.1} А (холод, старая ячейка; на проводе {:.3} В): {sag:.2} В на \
             нагрузке, нужно не ниже {floor:.2} В",
            w.voltage_drop_v(peak, c.theta0_c)
        ),
    ));
}

fn check_voltage(c: &Ctx, out: &mut Vec<BatteryFinding>) {
    use BatteryCheck::VoltageRange as K;
    let cell = &c.pack.cell;
    let s = c.pack.series as f64;
    let (Some(v_full), Some(v_cut)) = (cell.v_charge_max.value, cell.v_cutoff.value) else {
        out.push(na(K, "В", "напряжения ячейки не заданы"));
        return;
    };
    let v_max = v_full * s;
    let v_end = v_cut * s;
    match c.load.load_v_max {
        Some(lim) => out.push(le(
            K,
            v_max,
            lim,
            "В",
            format!("полный заряд {v_max:.2} В, нагрузка допускает до {lim:.2} В"),
        )),
        None => out.push(na(K, "В", "наибольшее напряжение нагрузки не задано")),
    }
    if !c.load.steps.is_empty() {
        check_sag(c, v_end, out);
    }

    let ovp_limit = v_full + CHARGE_V_TOLERANCE;
    if let Some(ch) = &c.load.charge {
        // 1 нВ — только от ошибки округления произведения 4.2·S
        out.push(le(
            K,
            ch.voltage_v,
            v_max + 1e-9,
            "В",
            format!(
                "зарядное устройство {:.2} В, предел сборки {v_max:.2} В",
                ch.voltage_v
            ),
        ));
        let group = ch.voltage_v / s + c.pack.group_voltage_spread_v * (s - 1.0) / s;
        out.push(le(
            K,
            group,
            ovp_limit,
            "В",
            format!(
                "при разбалансе {:.3} В самая заряженная группа дойдёт до {group:.3} В, предел \
                 {ovp_limit:.2} В",
                c.pack.group_voltage_spread_v
            ),
        ));
    }
    match &c.pack.bms {
        None => out.push(fail(
            K,
            0.0,
            ovp_limit,
            "В",
            "нет BMS: напряжение групп не контролируется".into(),
        )),
        Some(b) => {
            match b.cell_ovp_v {
                Some(v) => out.push(le(
                    K,
                    v,
                    ovp_limit,
                    "В",
                    format!(
                        "порог перезаряда BMS {v:.3} В, предел {ovp_limit:.2} В ({v_full:.2} В + \
                         допуск 0.05 В)"
                    ),
                )),
                None => out.push(na(K, "В", "порог перезаряда BMS не задан")),
            }
            match b.cell_uvp_v {
                Some(v) => out.push(ge(
                    K,
                    v,
                    v_cut,
                    "В",
                    format!("порог переразряда BMS {v:.3} В, конечное напряжение {v_cut:.2} В"),
                )),
                None => out.push(na(K, "В", "порог переразряда BMS не задан")),
            }
        }
    }
}

/// Оценка сборки при заданном профиле. Каждый вид из [`ALL_CHECKS`] присутствует в результате
/// хотя бы один раз. Для общего вывода используйте [`overall`].
pub fn assess_pack(pack: &PackDesign, load: &LoadProfile) -> Vec<BatteryFinding> {
    let problems = validate(pack, load);
    if !problems.is_empty() {
        let detail = format!(
            "расчёт не выполнен, вход некорректен: {}",
            problems.join("; ")
        );
        return ALL_CHECKS
            .iter()
            .map(|&kind| BatteryFinding {
                kind,
                outcome: Outcome::Fail,
                value: 0.0,
                limit: 0.0,
                unit: "",
                detail: detail.clone(),
            })
            .collect();
    }

    let theta0_c = if load.steps.is_empty() {
        load.ambient_max_c
    } else {
        pack.wire
            .profile_max_temp_c(&load.step_pairs(), load.cycles, load.ambient_max_c)
    };
    let ctx = Ctx {
        pack,
        load,
        faults: fault_table(pack, load),
        v_max: pack.summary().v_max,
        theta0_c,
        thermal: simulate_temperature(pack, load),
    };

    let mut out = Vec::new();
    check_short_circuit(&ctx, &mut out);
    check_wire(&ctx, &mut out);
    check_protection(&ctx, &mut out);
    check_cell_current(&ctx, &mut out);
    check_thermal(&ctx, &mut out);
    check_charge(&ctx, &mut out);
    check_skin(&ctx, &mut out);
    check_voltage(&ctx, &mut out);

    for &kind in &ALL_CHECKS {
        if !out.iter().any(|f| f.kind == kind) {
            out.push(na(kind, "", "проверка не выполнена: нет данных"));
        }
    }
    for f in &mut out {
        if f.outcome == Outcome::Pass && !(f.value.is_finite() && f.limit.is_finite()) {
            f.outcome = Outcome::Fail;
            f.detail
                .push_str("; результат расчёта не является конечным числом");
        }
    }
    out
}
