//! Эталонные сверки блока вторым независимым способом: ручным расчётом, таблицами стандартов
//! и производителей, численным интегрированием. Расхождения не подгоняются: они записаны в
//! допуске и пояснении.

use sim_env::contact::{ContactClass, burn_threshold_c};
use sim_verify::{Check, Tolerance};

use crate::cells::CellSpec;
use crate::fault::{FaultPoint, short_circuit};
use crate::pack::{EOL_RESISTANCE_FACTOR, LoadProfile, LoadStep, PackDesign};
use crate::protection::{BmsSpec, Bq769x0Config, FuseSpec, MosfetSpec};
use crate::thermal::{EnclosureDesign, H_INNER_DEFAULT_W_M2K, simulate_temperature};
use crate::wire::{
    CU_BETA_C, CU_QC_J_K_M3, CU_RHO20_OHM_M, Conductor, Insulation, WireCooling, WireRun,
    adiabatic_final_temp_c, adiabatic_k, awg_area_mm2, awg_diameter_mm, copper_k_from_constants,
    onderdonk_current_a,
};

#[allow(clippy::too_many_arguments)]
fn chk(
    id: &'static str,
    title: impl Into<String>,
    simulated: f64,
    reference: f64,
    unit: &'static str,
    method: impl Into<String>,
    reference_method: impl Into<String>,
    tolerance: Tolerance,
    note: impl Into<String>,
) -> Check {
    Check {
        id,
        title: title.into(),
        simulated,
        reference,
        unit,
        method: method.into(),
        reference_method: reference_method.into(),
        tolerance,
        note: note.into(),
    }
}

fn wire(awg: u8) -> WireRun {
    WireRun {
        conductor: Conductor::Awg(awg),
        insulation: Insulation::Silicone,
        length_m: 1.0,
        cooling: WireCooling::FreeAir,
        outer_diameter_mm: None,
    }
}

fn bare_pack() -> PackDesign {
    PackDesign {
        cell: CellSpec::lg_m50(),
        series: 4,
        parallel: 2,
        cell_resistance_spread: 0.0,
        group_voltage_spread_v: 0.0,
        interconnect_r_ohm: 0.0,
        contact_r_ohm: 0.0,
        wire: wire(14),
        fuse: None,
        bms: None,
        enclosure: None,
        skin: None,
        worn_on_body: true,
    }
}

fn still_load(current_a: f64, duration_s: f64) -> LoadProfile {
    LoadProfile {
        steps: vec![LoadStep {
            current_a,
            duration_s,
        }],
        cycles: 1,
        ambient_min_c: 25.0,
        ambient_max_c: 25.0,
        load_v_min: None,
        load_v_max: None,
        charge: None,
    }
}

/// Интегрирование нагрева жилы методом Рунге–Кутты 4-го порядка из физических свойств меди,
/// без постоянной `K` и без замкнутой формулы.
fn rk4_wire(i: f64, s_mm2: f64, theta0: f64, time: f64, cooling_w_m_k: f64, t_amb: f64) -> f64 {
    let s = s_mm2 * 1e-6;
    let f = |th: f64| {
        let rho = CU_RHO20_OHM_M * (CU_BETA_C + th) / (CU_BETA_C + 20.0);
        (i * i * rho / s - cooling_w_m_k * (th - t_amb)) / (CU_QC_J_K_M3 * s)
    };
    let n = 20_000;
    let h = time / n as f64;
    let mut th = theta0;
    for _ in 0..n {
        let k1 = f(th);
        let k2 = f(th + 0.5 * h * k1);
        let k3 = f(th + 0.5 * h * k2);
        let k4 = f(th + h * k3);
        th += h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }
    th
}

fn conductor_checks() -> Vec<Check> {
    let iec =
        "IEC 60364-5-54:2011, табл. A.54 (значения приведены также на mycableengineering.com)";
    let rel = Tolerance::Relative(0.01);
    let mut v = vec![chk(
        "battery.k_copper_constants",
        "Постоянная меди K из физических свойств",
        copper_k_from_constants(),
        226.0,
        "А·с^½/мм²",
        "√(Qc·(β + 20)/ρ20), Qc = 3.45 мДж/(К·мм³), β = 234.5 °C, ρ20 = 17.241 нОм·м",
        "IEC 60949 / IEC 60364-5-54: K = 226",
        rel,
        "стандарт округляет до целого",
    )];
    for (id, title, ti, tf, k) in [
        (
            "battery.k_pvc_160",
            "k: ПВХ 70 → 160 °C",
            70.0,
            160.0,
            115.0,
        ),
        (
            "battery.k_pvc_140",
            "k: ПВХ 70 → 140 °C",
            70.0,
            140.0,
            103.0,
        ),
        (
            "battery.k_xlpe",
            "k: сшитый полиэтилен 90 → 250 °C",
            90.0,
            250.0,
            143.0,
        ),
        (
            "battery.k_silicone",
            "k: силикон 180 → 350 °C",
            180.0,
            350.0,
            132.0,
        ),
    ] {
        v.push(chk(
            id,
            title,
            adiabatic_k(ti, tf),
            k,
            "А·с^½/мм²",
            "226·√ln(1 + (θf − θi)/(234.5 + θi))",
            iec,
            rel,
            "табличные значения округлены до целого",
        ));
    }

    let s = awg_area_mm2(14);
    let k_melt = adiabatic_k(25.0, 1083.0);
    v.push(chk(
        "battery.onderdonk_vs_iec",
        "Ток плавления AWG 14 за 1 с: Ондердонк против IEC",
        onderdonk_current_a(s, 1.0, 25.0),
        k_melt * s,
        "А",
        "формула Ондердонка в круговых милах, log₁₀, 234, константа 33",
        "адиабата IEC 60949: I = k·S/√t при θ 25 → 1083 °C",
        Tolerance::Relative(0.01),
        "две записи одной адиабаты с разными округлениями; теплота плавления не учтена в обеих",
    ));

    let th_closed = adiabatic_final_temp_c(600.0, 0.05, s, 40.0);
    v.push(chk(
        "battery.adiabatic_vs_rk4",
        "Нагрев жилы при КЗ: формула против численного интегрирования",
        th_closed,
        rk4_wire(600.0, s, 40.0, 0.05, 0.0, 25.0),
        "°C",
        "(θi + β)·exp(I²t/(K²S²)) − β",
        "РК4 по ρ(θ) и Qc без постоянной K",
        Tolerance::Relative(0.01),
        "разница — округление K до 226 в стандарте",
    ));

    let w = wire(14);
    let cool = w.cooling_w_per_m_k(25.0);
    v.push(chk(
        "battery.cooling_vs_rk4",
        "Нагрев жилы с охлаждением: точное решение против численного",
        w.temperature_after_c(40.0, 120.0, 30.0, 25.0),
        rk4_wire(40.0, w.area_mm2(), 30.0, 120.0, cool, 25.0),
        "°C",
        "точное решение линейного уравнения dθ/dt = (θ + β)/τa − (θ − Ta)/τc",
        "РК4 по ρ(θ), Qc и теплоотдаче",
        Tolerance::Relative(0.01),
        "разница — округление K до 226",
    ));

    for (id, awg, d) in [
        ("battery.awg10_d", 10u8, 2.588),
        ("battery.awg14_d", 14, 1.628),
        ("battery.awg18_d", 18, 1.024),
        ("battery.awg22_d", 22, 0.644),
    ] {
        v.push(chk(
            id,
            format!("Диаметр жилы AWG {awg}"),
            awg_diameter_mm(awg),
            d,
            "мм",
            "0.127·92^((36 − n)/39)",
            "таблица ASTM B258",
            Tolerance::Relative(0.002),
            "",
        ));
    }

    v.push(chk(
        "battery.awg14_resistance",
        "Сопротивление AWG 14 при 20 °C",
        wire(14).resistance_ohm_per_m(20.0) * 1e3,
        2.525 * 3.280_84,
        "мОм/м",
        "ρ20/S по сечению ASTM B258",
        "NBS Handbook 100: 2.525 Ом на 1000 футов",
        Tolerance::Relative(0.005),
        "",
    ));

    let w14 = wire(14);
    let model = (w14.cooling_w_per_m_k(30.0) * 60.0 / w14.resistance_ohm_per_m(90.0)).sqrt();
    v.push(chk(
        "battery.ampacity_vs_nec",
        "Допустимый ток AWG 14, свободный воздух 30 °C, жила 90 °C",
        model,
        35.0,
        "А",
        "баланс I²R' = h·π·D·ΔT по голой жиле при наименьшем h (перегрев 10 К)",
        "NEC табл. 310.15(B)(17), колонка 90 °C",
        Tolerance::Relative(0.40),
        "модель намеренно ниже таблицы: не учтена площадь изоляции; завышения быть не должно \
         (проверяется отдельным тестом на калибрах 18…10)",
    ));
    v
}

fn protection_checks() -> Vec<Check> {
    let mut v = Vec::new();
    let ato = "Littelfuse 257 Series (ATO), каталог 2010, лист 0257030";
    if let Some(f) = FuseSpec::ato(10) {
        for (id, mult, t) in [
            ("battery.ato_135", 1.35, 600.0),
            ("battery.ato_200", 2.0, 5.0),
            ("battery.ato_350", 3.5, 0.5),
            ("battery.ato_600", 6.0, 0.1),
        ] {
            v.push(chk(
                id,
                format!("ATO 10 А: наибольшее время при {:.0} %", mult * 100.0),
                f.max_clear_time_s(10.0 * mult).unwrap_or(f64::NAN),
                t,
                "с",
                "характеристика модуля в узле таблицы",
                ato,
                Tolerance::Relative(1e-9),
                "",
            ));
        }
        v.push(chk(
            "battery.ato_constant_i2t",
            "ATO 10 А: время при 1200 % по постоянному I²t",
            f.max_clear_time_s(120.0).unwrap_or(f64::NAN),
            0.1 * (60.0_f64 / 120.0).powi(2),
            "с",
            "продолжение характеристики выше последней строки",
            "ручной расчёт: t = 0.1 с · (600 %/1200 %)²",
            Tolerance::Relative(1e-9),
            "верхняя граница: реальный предохранитель открывается быстрее",
        ));
    }
    let cfg = Bq769x0Config {
        r_sense_ohm: 0.002,
        scd_mv: 44.0,
        scd_delay_us: 100,
        ocd_mv: 20.0,
        ocd_delay_ms: 160.0,
        cell_ovp_v: Some(4.25),
        cell_uvp_v: Some(2.8),
        mosfet: MosfetSpec {
            vds_max_v: 40.0,
            id_pulse_a: 500.0,
            id_pulse_width_s: 2e-4,
            rds_on_ohm: 0.002,
            avalanche_energy_j: None,
        },
    };
    if let Ok(b) = BmsSpec::bq769x0(&cfg) {
        v.push(chk(
            "battery.bq_scd_trip_max",
            "BQ769x0: наибольший порог КЗ при 44 мВ на шунте 2 мОм",
            b.scd_trip_max_a,
            25.45,
            "А",
            "порог с допусками модуля",
            "ручной расчёт: (44 мВ·1.1 + 2.5 мВ)/2 мОм; допуски TI SLUSBK2I",
            Tolerance::Relative(1e-9),
            "",
        ));
        v.push(chk(
            "battery.bq_scd_delay_max",
            "BQ769x0: наибольшая задержка КЗ при настройке 100 мкс",
            b.scd_delay_max_s * 1e6,
            150.0,
            "мкс",
            "таблица задержек модуля",
            "TI SLUSBK2I, SCD DELAY: 50 / 100 / 150 мкс",
            Tolerance::Relative(1e-9),
            "",
        ));
    }
    v
}

fn pack_checks() -> Vec<Check> {
    let mut v = Vec::new();
    let p = bare_pack();
    let load = still_load(4.0, 1800.0);
    if let Some(f) = short_circuit(&p, &load, FaultPoint::Terminals) {
        v.push(chk(
            "battery.sc_hand",
            "Наибольший ток КЗ 4S2P M50 на клеммах",
            f.i_max_a,
            350.0,
            "А",
            "short_circuit, точка «на клеммах»",
            "ручной расчёт: 4·4.2 В / (4·24 мОм / 2)",
            Tolerance::Relative(1e-9),
            "без перемычек, предохранителя и ключа",
        ));
        v.push(chk(
            "battery.sc_min_hand",
            "Наименьший ток КЗ 4S2P M50 на клеммах при 25 °C",
            f.i_min_a,
            10.0 / (2.0 * 0.036 * EOL_RESISTANCE_FACTOR),
            "А",
            "short_circuit, конец разряда и конец службы",
            "ручной расчёт: 4·2.5 В / (2·36 мОм·2)",
            Tolerance::Relative(1e-9),
            "множитель конца службы 2 — оценка",
        ));
    }
    let s = p.summary();
    v.push(chk(
        "battery.energy_hand",
        "Энергия 4S2P M50",
        s.energy_wh.unwrap_or(f64::NAN),
        8.0 * 4.9 * 3.63,
        "Вт·ч",
        "PackDesign::summary",
        "ручной расчёт: 8 ячеек · 4.9 А·ч · 3.63 В",
        Tolerance::Relative(1e-12),
        "",
    ));
    let mut split = bare_pack();
    split.series = 1;
    split.cell_resistance_spread = 0.2;
    v.push(chk(
        "battery.current_share",
        "Ток худшей ячейки 1S2P при разбросе ±20 %, 10 А",
        split.worst_cell_current_a(10.0),
        6.0,
        "А",
        "формула делителя в PackDesign",
        "ручной расчёт: 10 А · (1/0.8)/(1/0.8 + 1/1.2)",
        Tolerance::Relative(1e-12),
        "",
    ));

    let enc = EnclosureDesign {
        outer_area_m2: 0.06,
        height_m: 0.1,
        wall_thickness_m: 0.002,
        wall_k_w_mk: 0.18,
        h_inner_w_m2k: H_INNER_DEFAULT_W_M2K,
        emissivity: 0.85,
        body_side_fraction: 0.3,
    };
    let mut hot = bare_pack();
    hot.enclosure = Some(enc);
    let rt = enc.r_th_k_w(25.0);
    let c = 8.0 * 0.0685 * 887.0;
    let q = 8.0 * 2.0 * 2.0 * (0.036 * EOL_RESISTANCE_FACTOR);
    v.push(chk(
        "battery.lumped_step",
        "Нагрев батареи за 30 мин: шаговый расчёт против точного",
        simulate_temperature(&hot, &load).map_or(f64::NAN, |r| r.t_end_c),
        25.0 + q * rt * (1.0 - (-1800.0 / (c * rt)).exp()),
        "°C",
        "simulate_temperature",
        "точное решение T = Ta + Q·R·(1 − exp(−t/(R·C)))",
        Tolerance::Absolute(1e-6),
        "сопротивление постоянно: температура не ниже 25 °C",
    ));

    for (id, title, class, time, reference) in [
        (
            "battery.burn_8h",
            "Порог ожога за 8 ч",
            ContactClass::Plastics,
            28_800.0,
            43.0,
        ),
        (
            "battery.burn_plastic_1min",
            "Порог ожога пластмассы за 1 мин",
            ContactClass::Plastics,
            60.0,
            60.0,
        ),
        (
            "battery.burn_metal_1min",
            "Порог ожога голого металла за 1 мин",
            ContactClass::UncoatedMetal,
            60.0,
            51.0,
        ),
    ] {
        v.push(chk(
            id,
            title,
            burn_threshold_c(class, time),
            reference,
            "°C",
            "sim_env::contact::burn_threshold_c",
            "ISO 13732-1:2006, табл. 1",
            Tolerance::Absolute(1e-9),
            "порог общий с sim-env: сверка защищает от его незаметного изменения",
        ));
    }
    v
}

/// Все сверки блока.
pub fn all() -> Vec<Check> {
    let mut v = conductor_checks();
    v.extend(protection_checks());
    v.extend(pack_checks());
    v
}
