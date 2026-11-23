//! Сверки моделей `sim-env` со вторым независимым способом: другой формулой, точным
//! решением, другим численным методом, опубликованной таблицей или тестовыми значениями
//! внешней эталонной реализации. Каждая сверка — оба числа, допуск, источник; расхождения
//! объясняются в `note`, а не подгоняются.

use sim_verify::{Check, Tolerance};

use crate::contact::{self, Material};
use crate::convection::{self, Medium};
use crate::effects;
use crate::immersion;
use crate::indices;
use crate::two_node::{self, TwoNodeInputs, WettednessCap};
use crate::{air, water};

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

/// Все сверки.
pub fn all() -> Vec<Check> {
    let mut v = Vec::new();
    v.extend(air_checks());
    v.extend(water_checks());
    v.extend(convection_checks());
    v.extend(index_checks());
    v.extend(two_node_checks());
    v.extend(immersion_checks());
    v.extend(contact_checks());
    v.extend(solar_checks());
    v
}

/// Точка наибольшего относительного расхождения f/g − 1 на сетке.
fn worst_rel(
    ts: impl Iterator<Item = f64>,
    f: impl Fn(f64) -> f64,
    g: impl Fn(f64) -> f64,
) -> (f64, f64, f64) {
    let mut best = (0.0, f64::NAN, f64::NAN);
    let mut worst = -1.0;
    for t in ts {
        let (a, b) = (f(t), g(t));
        let e = (a / b - 1.0).abs();
        if e > worst {
            worst = e;
            best = (t, a, b);
        }
    }
    best
}

fn grid(from: f64, to: f64, step: f64) -> impl Iterator<Item = f64> {
    let n = ((to - from) / step).round() as usize;
    (0..=n).map(move |i| from + i as f64 * step)
}

// ---------------------------------------------------------------------------
// Воздух
// ---------------------------------------------------------------------------

/// Сверки свойств воздуха.
pub fn air_checks() -> Vec<Check> {
    let mut v = Vec::new();
    let (t, a, b) = worst_rel(
        grid(0.0, 50.0, 0.5),
        air::p_sat_magnus_water,
        air::p_sat_iapws_water,
    );
    v.push(chk(
        "env.psat.magnus",
        format!(
            "Давление насыщения над водой: Магнус против IAPWS, худшая точка 0…50 °C ({t:.1} °C)"
        ),
        a,
        b,
        "Па",
        "Магнус, коэффициенты Alduchov & Eskridge 1996",
        "Wagner & Pruß 1993 (IAPWS SR1-86), doi:10.1063/1.555926",
        Tolerance::Relative(0.004),
        "допуск — заявленная авторами точность AERK (< 0.384 % в −40…+50 °C)",
    ));
    let (t, a, b) = worst_rel(
        grid(0.0, 50.0, 0.5),
        air::p_sat_buck_water,
        air::p_sat_iapws_water,
    );
    v.push(chk(
        "env.psat.buck",
        format!("Давление насыщения над водой: Бак против IAPWS, худшая точка 0…50 °C ({t:.1} °C)"),
        a,
        b,
        "Па",
        "Бак (1981/1996): 611.21·exp((18.678 − t/234.5)·t/(257.14 + t))",
        "Wagner & Pruß 1993 (IAPWS SR1-86)",
        Tolerance::Relative(5e-4),
        "допуск 0.05 % — заявленная точность формулы Бака",
    ));
    for (id, name, f) in [
        (
            "env.psat.supercooled_magnus",
            "Магнус",
            air::p_sat_magnus_water as fn(f64) -> f64,
        ),
        ("env.psat.supercooled_buck", "Бак", air::p_sat_buck_water),
        (
            "env.psat.supercooled_iapws",
            "IAPWS (экстраполяция ниже 0 °C)",
            air::p_sat_iapws_water,
        ),
    ] {
        let (t, a, b) = worst_rel(grid(-40.0, 0.0, 0.5), f, air::p_sat_murphy_koop_water);
        v.push(chk(
            id,
            format!(
                "Переохлаждённая вода −40…0 °C: {name} против Murphy–Koop, худшая точка ({t:.1} °C)"
            ),
            a,
            b,
            "Па",
            name,
            "Murphy & Koop 2005, doi:10.1256/qj.04.94 (по данным для переохлаждённой воды)",
            Tolerance::Relative(0.01),
            "ниже 0 °C данные о воде скудны: формулы расходятся до 0.7 % при −40 °C (IAPWS там \
             экстраполирован). Поэтому в крейте p_sat < 0 °C берётся по Murphy–Koop",
        ));
    }
    for (id, name, f, tol) in [
        (
            "env.psat.ice_magnus",
            "Магнус (AERKi)",
            air::p_sat_magnus_ice as fn(f64) -> f64,
            0.002,
        ),
        ("env.psat.ice_buck", "Бак (лёд)", air::p_sat_buck_ice, 0.002),
        (
            "env.psat.ice_mk",
            "Murphy–Koop (лёд)",
            air::p_sat_murphy_koop_ice,
            0.001,
        ),
    ] {
        let (t, a, b) = worst_rel(grid(-80.0, 0.0, 0.5), f, air::p_sat_iapws_ice);
        v.push(chk(
            id,
            format!(
                "Сублимация льда −80…0 °C: {name} против IAPWS R14-08, худшая точка ({t:.1} °C)"
            ),
            a,
            b,
            "Па",
            name,
            "IAPWS R14-08 (Wagner et al. 2011), doi:10.1063/1.3657937",
            Tolerance::Relative(tol),
            "допуск — порядок точности, заявленный авторами формул",
        ));
    }
    v.push(chk(
        "env.psat.boiling",
        "IAPWS: давление насыщения в нормальной точке кипения 99.974 °C (МТШ-90)",
        air::p_sat_iapws_water(99.974),
        air::P_STANDARD,
        "Па",
        "уравнение Вагнера–Прусса",
        "определение: кипение при 101 325 Па — 373.124 К по МТШ-90",
        Tolerance::Relative(2e-5),
        "проверка реализации коэффициентов",
    ));
    let mut worst: (f64, f64, f64, f64, f64) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for t in grid(-20.0, 40.0, 5.0) {
        for rh in [10.0, 30.0, 50.0, 70.0, 90.0] {
            let (a, b) = (air::dew_point_magnus(t, rh), air::dew_point_iapws(t, rh));
            if (a - b).abs() > (worst.2 - worst.3).abs() {
                worst = (t, rh, a, b, 0.0);
            }
        }
    }
    v.push(chk(
        "env.dewpoint",
        format!("Точка росы: обращение Магнуса против численного обращения эталона, худшая точка ({:.0} °C, {:.0} %)", worst.0, worst.1),
        worst.2,
        worst.3,
        "°C",
        "аналитическое обращение Магнуса",
        "бисекция по p_sat (IAPWS / Murphy–Koop)",
        Tolerance::Absolute(0.1),
        "−20…40 °C, RH 10–90 %",
    ));
    v.push(chk(
        "env.air.density_cipm",
        "Плотность влажного воздуха 20 °C, 50 %, 101 325 Па: идеальная смесь против CIPM-2007",
        air::density_moist(20.0, 50.0, air::P_STANDARD),
        air::density_cipm2007(20.0, 50.0, air::P_STANDARD),
        "кг/м³",
        "ρ = (p − e)/(R_d·T) + e/(R_v·T)",
        "CIPM-2007 (Picard et al. 2008, doi:10.1088/0026-1394/45/2/004), с Z и f",
        Tolerance::Relative(1e-3),
        "разница — неидеальность (Z ≈ 0.9996), ≈ 0.04 %",
    ));
    // Таблица Incropera A.4 (свойства воздуха при 1 бар = 100 кПа)
    let table = [
        (250.0, 1.3947, 1006.0, 159.6e-7, 22.3e-3, 0.720),
        (300.0, 1.1614, 1007.0, 184.6e-7, 26.3e-3, 0.707),
        (350.0, 0.9950, 1009.0, 208.2e-7, 30.0e-3, 0.700),
    ];
    let props = |tk: f64| air::air_props(tk - air::T0, 0.0, 100_000.0);
    type Getter = fn(&air::AirProps) -> f64;
    let items: [(
        &'static str,
        &'static str,
        &'static str,
        usize,
        Getter,
        f64,
        &str,
    ); 5] = [
        (
            "env.air.rho_table",
            "плотность (1 бар)",
            "кг/м³",
            1,
            |p| p.rho,
            2e-3,
            "идеальный газ",
        ),
        (
            "env.air.cp_table",
            "теплоёмкость",
            "Дж/(кг·К)",
            2,
            |p| p.cp,
            5e-3,
            "наша аппроксимация c_p(T) («оценка»)",
        ),
        (
            "env.air.mu_table",
            "вязкость",
            "Па·с",
            3,
            |p| p.mu,
            0.01,
            "Сазерленд, S = 110.4 К",
        ),
        (
            "env.air.k_table",
            "теплопроводность",
            "Вт/(м·К)",
            4,
            |p| p.k,
            0.01,
            "Сазерленд для k, S = 194 К",
        ),
        (
            "env.air.pr_table",
            "число Прандтля",
            "",
            5,
            |p| p.pr,
            0.01,
            "μ·c_p/k",
        ),
    ];
    for (id, name, unit, col, get, tol, method) in items {
        let mut w: (f64, f64, f64) = (0.0, 0.0, 0.0);
        for row in &table {
            let r = [row.0, row.1, row.2, row.3, row.4, row.5][col];
            let s = get(&props(row.0));
            if (s / r - 1.0).abs() >= (w.1 / w.2 - 1.0).abs() || w.2 == 0.0 {
                w = (row.0, s, r);
            }
        }
        v.push(chk(
            id,
            format!(
                "Сухой воздух: {name}, худшая точка из 250/300/350 К ({:.0} К)",
                w.0
            ),
            w.1,
            w.2,
            unit,
            method,
            "Incropera et al., Fundamentals of Heat and Mass Transfer, 6th ed., табл. A.4",
            Tolerance::Relative(tol),
            "таблица дана при 1 бар (100 кПа), поэтому плотность считается при 100 кПа",
        ));
    }
    v
}

// ---------------------------------------------------------------------------
// Вода
// ---------------------------------------------------------------------------

/// Сверки свойств воды.
pub fn water_checks() -> Vec<Check> {
    let mut v = Vec::new();
    v.push(chk(
        "env.water.eos80",
        "EOS-80: плотность морской воды S = 35, 5 °C, p = 0 — контрольное значение ЮНЕСКО",
        water::density_eos80(5.0, 35.0),
        1_027.675_47,
        "кг/м³",
        "Millero & Poisson 1981 (наша реализация)",
        "UNESCO Tech. Pap. Mar. Sci. 36/44 (контрольное значение)",
        Tolerance::Absolute(1e-4),
        "проверка реализации второго способа",
    ));
    let (t, a, b) = worst_rel(
        grid(0.0, 40.0, 1.0),
        |t| water::density(t, 35.0),
        |t| water::density_eos80(t, 35.0),
    );
    v.push(chk(
        "env.water.sea_density",
        format!("Плотность морской воды S = 35: Sharqawy 2010 против EOS-80, худшая точка 0…40 °C ({t:.0} °C)"),
        a,
        b,
        "кг/м³",
        "Sharqawy et al. 2010, ур. 8 (doi:10.5004/dwt.2010.1079)",
        "UNESCO EOS-80 (Millero & Poisson 1981)",
        Tolerance::Relative(1e-3),
        "допуск — заявленная точность Sharqawy ±0.1 %; солёность Sharqawy — эталонная (г/кг), \
         EOS-80 — практическая (на 0.47 % меньше), вклад ≈ 0.01 %",
    ));
    // NIST WebBook (IAPWS-95, вязкость IAPWS 2008, теплопроводность IAPWS 2011), 0.101325 МПа
    let nist = [
        (5.0, 999.967, 4205.04, 1.518_17e-3, 0.567_794),
        (25.0, 997.048, 4181.31, 8.900_22e-4, 0.606_516),
    ];
    let src =
        "NIST Chemistry WebBook: IAPWS-95 (ρ, c_p), IAPWS 2008 (μ), IAPWS 2011 (k), 0.101325 МПа";
    for (t, rho, cp, mu, k) in nist {
        v.push(chk(
            if t < 10.0 {
                "env.water.rho_5"
            } else {
                "env.water.rho_25"
            },
            format!("Пресная вода {t:.0} °C: плотность"),
            water::density(t, 0.0),
            rho,
            "кг/м³",
            "Sharqawy 2010 при S = 0",
            src,
            Tolerance::Relative(5e-4),
            "",
        ));
        v.push(chk(
            if t < 10.0 {
                "env.water.mu_5"
            } else {
                "env.water.mu_25"
            },
            format!("Пресная вода {t:.0} °C: вязкость"),
            water::viscosity(t, 0.0),
            mu,
            "Па·с",
            "Sharqawy 2010, ур. 23",
            src,
            Tolerance::Relative(2e-3),
            "",
        ));
        v.push(chk(
            if t < 10.0 {
                "env.water.k_5"
            } else {
                "env.water.k_25"
            },
            format!("Пресная вода {t:.0} °C: теплопроводность"),
            water::conductivity(t, 0.0),
            k,
            "Вт/(м·К)",
            "Sharqawy 2010, ур. 13 (Jamieson & Tudhope 1970)",
            src,
            Tolerance::Relative(0.03),
            "допуск — заявленная точность корреляции ±3 %",
        ));
        v.push(chk(
            if t < 10.0 {
                "env.water.cp_5"
            } else {
                "env.water.cp_25"
            },
            format!("Пресная вода {t:.0} °C: теплоёмкость"),
            water::heat_capacity(t, 0.0),
            cp,
            "Дж/(кг·К)",
            "Sharqawy 2010, ур. 9 (Jamieson et al. 1969)",
            src,
            Tolerance::Relative(3e-3),
            "допуск — заявленная точность ±0.28 %",
        ));
    }
    v.push(chk(
        "env.water.pss78",
        "Электропроводность морской воды S = 35 при 15 °C (обращение PSS-78)",
        water::seawater_electrical_conductivity(15.0, 35.0),
        water::SEAWATER_C35_15,
        "См/м",
        "численное обращение S(R_t, t) PSS-78",
        "определение PSS-78: C(35, 15, 0) = 42.914 мСм/см",
        Tolerance::Relative(1e-4),
        "при 25 °C модель даёт 5.31 См/м; пресная вода — 0.005–0.15 См/м",
    ));
    v.push(chk(
        "env.water.freezing",
        "Температура замерзания морской воды S = 40, 500 дбар — контрольное значение ЮНЕСКО",
        water::freezing_point_at(40.0, 500.0),
        -2.588_567,
        "°C",
        "Fofonoff & Millard 1983 (наша реализация)",
        "UNESCO Tech. Pap. Mar. Sci. 44 (контрольное значение)",
        Tolerance::Absolute(1e-5),
        "при S = 35 у поверхности: −1.92 °C",
    ));
    v
}

// ---------------------------------------------------------------------------
// Конвекция
// ---------------------------------------------------------------------------

/// Сверки корреляций теплоотдачи.
pub fn convection_checks() -> Vec<Check> {
    let mut v = Vec::new();
    // Учебный пример: цилиндр D = 12.7 мм, L = 94 мм, воздух 10 м/с, 26.2 °C, поверхность
    // 128.4 °C, мощность 46 Вт, 15 % теряется излучением и через торцы ⇒
    // h = 0.85·46/(π·D·L·ΔT) = 102 Вт/(м²·К) (Incropera, 6th ed., Example 7.4).
    let (d, len, vel, tinf, ts) = (0.0127, 0.094, 10.0, 26.2, 128.4);
    let h_exp = 0.85 * 46.0 / (std::f64::consts::PI * d * len * (ts - tinf));
    let dry = Medium::Air {
        rh_percent: 0.0,
        p_pa: 100_000.0,
    };
    let pf = dry.props(0.5 * (ts + tinf));
    let re = vel * d / pf.nu();
    let h_cb = convection::nu_churchill_bernstein(re, pf.pr) * pf.k / d;
    let h_hil = convection::nu_hilpert(re, pf.pr) * pf.k / d;
    let (pi, ps) = (dry.props(tinf), dry.props(ts));
    let h_zk = convection::nu_zukauskas(vel * d / pi.nu(), pi.pr, ps.pr) * pi.k / d;
    let ex_src = "опыт: Incropera et al., 6th ed., Example 7.4 (h = 0.85·46 Вт/(π·D·L·ΔT))";
    for (id, name, h) in [
        ("env.conv.example_cb", "Черчилль–Бернштейн", h_cb),
        ("env.conv.example_hilpert", "Хилперт", h_hil),
        ("env.conv.example_zukauskas", "Жукаускас", h_zk),
    ] {
        v.push(chk(
            id,
            format!("Цилиндр 12.7 мм в потоке 10 м/с: {name} против измерения"),
            h,
            h_exp,
            "Вт/(м²·К)",
            format!("{name}, свойства воздуха по нашим формулам"),
            ex_src,
            Tolerance::Relative(0.20),
            "допуск ±20 % — типичная погрешность корреляций (Incropera §7.4); учебник даёт \
             88 (Хилперт), 96 (Ч–Б), 105 (Жукаускас)",
        ));
    }
    // тот же расчёт, но против пересчёта со свойствами из таблицы учебника (T_f = 350 К:
    // ν = 20.92e-6, k = 0.030, Pr = 0.700) — отделяет ошибку свойств от ошибки корреляции
    let re_tab: f64 = 10.0 * 0.0127 / 20.92e-6;
    let h_cb_tab = convection::nu_churchill_bernstein(re_tab, 0.700) * 0.030 / 0.0127;
    v.push(chk(
        "env.conv.example_cb_props",
        "Тот же пример, Черчилль–Бернштейн: наши свойства воздуха против таблицы учебника",
        h_cb,
        h_cb_tab,
        "Вт/(м²·К)",
        "свойства по Сазерленду при T_f = 350 К",
        "свойства из Incropera табл. A.4 (ν = 20.92·10⁻⁶, k = 0.030, Pr = 0.700)",
        Tolerance::Relative(0.03),
        "разница — только в свойствах воздуха (ν на 1.7 % меньше табличной при 1 атм)",
    ));
    // Предплечье 8 см: разброс корреляций в воздухе и воде
    for (id, med, vel, tf, label) in [
        (
            "env.conv.forearm_air",
            Medium::air(50.0),
            5.0,
            0.0,
            "воздух 0 °C, 5 м/с",
        ),
        (
            "env.conv.forearm_water",
            Medium::FreshWater,
            0.5,
            10.0,
            "пресная вода 10 °C, 0.5 м/с",
        ),
        (
            "env.conv.forearm_sea",
            Medium::sea(),
            0.5,
            10.0,
            "морская вода 10 °C, 0.5 м/с",
        ),
    ] {
        let ts = 30.0;
        let c = convection::cylinder_crossflow(&med, 0.08, vel, ts, tf);
        let p = med.props(0.5 * (ts + tf));
        let h_h = convection::nu_hilpert(c.re, p.pr) * p.k / 0.08;
        let (pinf, psur) = (med.props(tf), med.props(ts));
        let h_z =
            convection::nu_zukauskas(vel * 0.08 / pinf.nu(), pinf.pr, psur.pr) * pinf.k / 0.08;
        v.push(chk(
            if id == "env.conv.forearm_air" {
                "env.conv.forearm_air_hilpert"
            } else if id == "env.conv.forearm_water" {
                "env.conv.forearm_water_hilpert"
            } else {
                "env.conv.forearm_sea_hilpert"
            },
            format!(
                "Предплечье Ø8 см, {label}: Черчилль–Бернштейн против Хилперта (Re = {:.0})",
                c.re
            ),
            c.h,
            h_h,
            "Вт/(м²·К)",
            "Черчилль–Бернштейн",
            "Хилперт (Incropera табл. 7.2)",
            Tolerance::Relative(0.20),
            "ожидаемый разброс корреляций ±20 %",
        ));
        v.push(chk(
            if id == "env.conv.forearm_air" {
                "env.conv.forearm_air_zukauskas"
            } else if id == "env.conv.forearm_water" {
                "env.conv.forearm_water_zukauskas"
            } else {
                "env.conv.forearm_sea_zukauskas"
            },
            format!("Предплечье Ø8 см, {label}: Черчилль–Бернштейн против Жукаускаса"),
            c.h,
            h_z,
            "Вт/(м²·К)",
            "Черчилль–Бернштейн (свойства при T_f)",
            "Жукаускас (свойства при T_∞, поправка (Pr/Pr_s)^¼)",
            Tolerance::Relative(0.20),
            "ожидаемый разброс корреляций ±20 %",
        ));
    }
    // Свободная конвекция горизонтального цилиндра: воздух и вода против Моргана
    let air = Medium::air(50.0);
    let c = convection::horizontal_cylinder_natural(&air, 0.08, 30.0, 20.0);
    v.push(chk(
        "env.conv.natural_air",
        format!("Свободная конвекция, цилиндр Ø8 см в воздухе, ΔT = 10 К (Ra = {:.2e}): Черчилль–Чу против Моргана", c.ra),
        c.nu,
        convection::nu_morgan_horizontal_cylinder(c.ra),
        "",
        "Черчилль–Чу 1975",
        "Морган 1975 (Incropera табл. 9.1)",
        Tolerance::Relative(0.10),
        "в воздухе корреляции согласуются в пределах 10 %",
    ));
    let cw = convection::horizontal_cylinder_natural(&Medium::FreshWater, 0.08, 30.0, 20.0);
    v.push(chk(
        "env.conv.natural_water",
        format!("Свободная конвекция, цилиндр Ø8 см в воде, ΔT = 10 К (Ra = {:.2e}): Черчилль–Чу против Моргана", cw.ra),
        cw.nu,
        convection::nu_morgan_horizontal_cylinder(cw.ra),
        "",
        "Черчилль–Чу 1975 (с поправкой на Pr)",
        "Морган 1975",
        Tolerance::Relative(0.25),
        "РАСХОЖДЕНИЕ ≈ 20 %: корреляция Моргана не зависит от Pr и подогнана в основном \
         к газам; при Pr ≈ 7 и Ra > 10⁷ она даёт меньше. Для воды верим Черчиллю–Чу, \
         разброс h в воде — ±25 %",
    ));
    let vp = convection::vertical_surface_natural(&air, 0.3, 30.0, 20.0);
    v.push(chk(
        "env.conv.vertical_plate",
        format!("Вертикальная поверхность 0.3 м в воздухе, ΔT = 10 К (Ra = {:.2e}): Черчилль–Чу против решения подобия", vp.ra),
        vp.nu,
        convection::nu_lefevre_ede_vertical_cylinder(vp.ra, vp.pr, 0.3, 1e12),
        "",
        "Черчилль–Чу (вертикальная пластина)",
        "Ле Февр–Эде при D → ∞ (ламинарное решение подобия)",
        Tolerance::Relative(0.10),
        "ламинарный режим Ra < 10⁹",
    ));
    // Пластина: корреляция Польгаузена против численного решения Блазиуса–Польгаузена
    let (fpp0, _) = convection::blasius_pohlhausen(1.0);
    v.push(chk(
        "env.conv.blasius",
        "Пограничный слой Блазиуса: трение f''(0) — наш численный расчёт (РК4 + стрельба)",
        fpp0,
        0.332_057,
        "",
        "РК4, η ∈ [0, 12], 12 000 шагов, стрельба бисекцией",
        "классическое значение f''(0) = 0.332057 (Howarth 1938)",
        Tolerance::Relative(1e-5),
        "",
    ));
    for (id, pr) in [
        ("env.conv.pohlhausen_pr07", 0.7),
        ("env.conv.pohlhausen_pr7", 7.0),
    ] {
        let (_, exact) = convection::blasius_pohlhausen(pr);
        v.push(chk(
            id,
            format!(
                "Ламинарная пластина, Pr = {pr}: Nu_x/Re_x^½ — корреляция против точного решения"
            ),
            convection::nu_flat_plate_local_laminar(1.0, pr),
            exact,
            "",
            "Польгаузен: 0.332·Pr^⅓",
            "численное решение уравнений Блазиуса и энергии (наш РК4)",
            Tolerance::Relative(0.02),
            "известная точность Pr^⅓-аппроксимации ±2 % при Pr ≥ 0.6",
        ));
    }
    v
}

// ---------------------------------------------------------------------------
// Индексы
// ---------------------------------------------------------------------------

/// Таблица NWS Wind Chill Chart (01.11.2001): строки T = 40…−45 °F шагом 5, столбцы
/// V = 5…60 миль/ч шагом 5. Источник: https://www.weather.gov/media/safety/windchillchart3.pdf
const NWS_WIND_CHILL: [[f64; 12]; 18] = [
    [36., 34., 32., 30., 29., 28., 28., 27., 26., 26., 25., 25.],
    [31., 27., 25., 24., 23., 22., 21., 20., 19., 19., 18., 17.],
    [25., 21., 19., 17., 16., 15., 14., 13., 12., 12., 11., 10.],
    [19., 15., 13., 11., 9., 8., 7., 6., 5., 4., 4., 3.],
    [13., 9., 6., 4., 3., 1., 0., -1., -2., -3., -3., -4.],
    [7., 3., 0., -2., -4., -5., -7., -8., -9., -10., -11., -11.],
    [
        1., -4., -7., -9., -11., -12., -14., -15., -16., -17., -18., -19.,
    ],
    [
        -5., -10., -13., -15., -17., -19., -21., -22., -23., -24., -25., -26.,
    ],
    [
        -11., -16., -19., -22., -24., -26., -27., -29., -30., -31., -32., -33.,
    ],
    [
        -16., -22., -26., -29., -31., -33., -34., -36., -37., -38., -39., -40.,
    ],
    [
        -22., -28., -32., -35., -37., -39., -41., -43., -44., -45., -46., -48.,
    ],
    [
        -28., -35., -39., -42., -44., -46., -48., -50., -51., -52., -54., -55.,
    ],
    [
        -34., -41., -45., -48., -51., -53., -55., -57., -58., -60., -61., -62.,
    ],
    [
        -40., -47., -51., -55., -58., -60., -62., -64., -65., -67., -68., -69.,
    ],
    [
        -46., -53., -58., -61., -64., -67., -69., -71., -72., -74., -75., -76.,
    ],
    [
        -52., -59., -64., -68., -71., -73., -76., -78., -79., -81., -82., -84.,
    ],
    [
        -57., -66., -71., -74., -78., -80., -82., -84., -86., -88., -89., -91.,
    ],
    [
        -63., -72., -77., -81., -84., -87., -89., -91., -93., -95., -97., -98.,
    ],
];

/// Сверки погодных индексов.
pub fn index_checks() -> Vec<Check> {
    let mut v = Vec::new();
    let mut w: (f64, f64, f64, f64) = (0.0, 0.0, 0.0, 0.0);
    for (i, row) in NWS_WIND_CHILL.iter().enumerate() {
        let t = 40.0 - 5.0 * i as f64;
        for (j, &chart) in row.iter().enumerate() {
            let vel = 5.0 + 5.0 * j as f64;
            let ours = indices::wind_chill_f(t, vel);
            if (ours - chart).abs() > (w.2 - w.3).abs() {
                w = (t, vel, ours, chart);
            }
        }
    }
    v.push(chk(
        "env.windchill.chart",
        format!(
            "Ветро-холодовой индекс: худшая из 216 клеток таблицы NWS ({:.0} °F, {:.0} миль/ч)",
            w.0, w.1
        ),
        w.2,
        w.3,
        "°F",
        "формула NWS 2001 (Osczevski & Bluestein 2005)",
        "NWS Wind Chill Chart (windchillchart3.pdf), значения округлены до 1 °F",
        Tolerance::Absolute(0.5),
        "допуск — округление таблицы",
    ));
    let mut wm: (f64, f64, f64, f64) = (0.0, 0.0, 0.0, 0.0);
    for t in grid(-45.0, 40.0, 5.0) {
        for vel in grid(5.0, 60.0, 5.0) {
            let a = indices::wind_chill_f(t, vel);
            let b = indices::c_to_f(indices::wind_chill_c(indices::f_to_c(t), vel * 1.609_344));
            if (a - b).abs() > (wm.2 - wm.3).abs() {
                wm = (t, vel, b, a);
            }
        }
    }
    v.push(chk(
        "env.windchill.metric",
        format!("Ветро-холодовой индекс: метрическая формула EC против формулы NWS в °F, худшая точка ({:.0} °F, {:.0} миль/ч)", wm.0, wm.1),
        wm.2,
        wm.3,
        "°F",
        "13.12 + 0.6215·T − 11.37·V^0.16 + 0.3965·T·V^0.16 (°C, км/ч), пересчёт в °F",
        "35.74 + 0.6215·T − 35.75·V^0.16 + 0.4275·T·V^0.16 (°F, миль/ч)",
        Tolerance::Absolute(0.2),
        "две независимые подгонки одной модели в разных единицах",
    ));
    // время до обморожения: регрессия против таблицы EC
    let mut rel = Vec::new();
    let mut worst: (f64, f64, f64, f64) = (0.0, 0.0, 0.0, 0.0);
    for (r, vel) in indices::FROSTBITE_TABLE_V_KMH.iter().enumerate() {
        for (c, t) in indices::FROSTBITE_TABLE_T_C.iter().enumerate() {
            if let (Some(tab), Some(reg)) = (
                indices::FROSTBITE_TABLE_MIN[r][c],
                indices::frostbite_minutes_regression(*t, *vel),
            ) && tab > 2.0
            {
                let e = reg / tab - 1.0;
                rel.push(e.abs());
                if e.abs() > worst.2.abs() {
                    worst = (*t, *vel, e, tab);
                }
            }
        }
    }
    rel.sort_by(|a, b| a.total_cmp(b));
    let median = rel[rel.len() / 2];
    v.push(chk(
        "env.frostbite.regression",
        format!("Время до обморожения: регрессия EC против таблицы EC, медиана |отклонения| по {} клеткам", rel.len()),
        median,
        0.0,
        "доля",
        "t_f = [−24.5·(0.667·V + 4.8) + 2111]·(−4.8 − T)^−1.668",
        "таблица Environment Canada «Minutes to Frostbite» (клетки > 2 мин в области 20–80 км/ч)",
        Tolerance::Absolute(0.15),
        format!(
            "худшая клетка ({:.0} °C, {:.0} км/ч): регрессия на {:.0} % от табличных {:.0} мин; \
             вблизи порога обморожения время растёт неограниченно, а степенной закон этого не \
             повторяет. Таблица — основная (консервативный поиск), регрессия — только сверка",
            worst.0,
            worst.1,
            worst.2 * 100.0,
            worst.3
        ),
    ));
    // индекс жары против таблицы NWS
    let mut sq = 0.0;
    let mut n = 0.0;
    let mut wo: (f64, f64, f64, f64) = (0.0, 0.0, 0.0, 0.0);
    for (i, row) in indices::HEAT_INDEX_TABLE.iter().enumerate() {
        for (j, &val) in row.iter().enumerate() {
            let t = indices::HEAT_INDEX_TABLE_T_F[i];
            let rh = indices::HEAT_INDEX_TABLE_RH[j];
            let ours = indices::heat_index_f(t, rh);
            sq += (ours - val).powi(2);
            n += 1.0;
            let in_adj = rh > 85.0 && (80.0..=87.0).contains(&t);
            if !in_adj && (ours - val).abs() > (wo.2 - wo.3).abs() {
                wo = (t, rh, ours, val);
            }
        }
    }
    v.push(chk(
        "env.heatindex.rms",
        format!("Индекс жары: СКО алгоритма NWS от таблицы NWS по {n:.0} клеткам"),
        (sq / n).sqrt(),
        0.0,
        "°F",
        "алгоритм WPC/NWS (Стедман + Ротфус + поправки)",
        "таблица NWS Heat Index (L. Rothfusz, NWS)",
        Tolerance::Absolute(1.3),
        "допуск — заявленная NWS ошибка регрессии ±1.3 °F",
    ));
    v.push(chk(
        "env.heatindex.worst",
        format!(
            "Индекс жары: худшая клетка вне зоны поправки на влажность ({:.0} °F, {:.0} %)",
            wo.0, wo.1
        ),
        wo.2,
        wo.3,
        "°F",
        "алгоритм NWS",
        "таблица NWS Heat Index",
        Tolerance::Absolute(1.8),
        "допуск 1.3 °F (точность регрессии) + 0.5 °F (округление таблицы)",
    ));
    let t = 80.0;
    let rh = 100.0;
    let with_adj = indices::heat_index_f(t, rh);
    let adj = (rh - 85.0) / 10.0 * ((87.0 - t) / 5.0);
    v.push(chk(
        "env.heatindex.humid_corner",
        "Индекс жары 80 °F, 100 %: регрессия Ротфуса без поправки на влажность против таблицы",
        with_adj - adj,
        87.0,
        "°F",
        "регрессия Ротфуса без поправки «RH > 85 %»",
        "таблица NWS Heat Index",
        Tolerance::Absolute(1.3),
        format!(
            "РАСХОЖДЕНИЕ показано: официальный алгоритм NWS с поправкой даёт {with_adj:.1} °F \
             (+{:.1} °F к таблице) — таблица построена без этой поправки. Приложение использует \
             официальный алгоритм (выше — консервативнее)",
            with_adj - 87.0
        ),
    ));
    v
}

// ---------------------------------------------------------------------------
// Двухузловая модель
// ---------------------------------------------------------------------------

/// Таблица проверки SET из ANSI/ASHRAE 55-2020 (Normative Appendix D): t_a, t_r, v, RH,
/// мет, кло, SET. Машиночитаемая копия: validation-data-comfort-models v1.1.0, ts_set.json
/// (MIT, © 2021 F. Tartarini), https://github.com/FedericoTartarini/validation-data-comfort-models
pub const ASHRAE55_SET_TABLE: [[f64; 7]; 22] = [
    [25., 25., 0.15, 50., 1., 0.5, 23.8],
    [0., 25., 0.15, 50., 1., 0.5, 12.1],
    [10., 25., 0.15, 50., 1., 0.5, 16.8],
    [15., 25., 0.15, 50., 1., 0.5, 19.2],
    [20., 25., 0.15, 50., 1., 0.5, 21.5],
    [30., 25., 0.15, 50., 1., 0.5, 26.4],
    [40., 25., 0.15, 50., 1., 0.5, 34.3],
    [25., 25., 0.15, 10., 1., 0.5, 23.3],
    [25., 25., 0.15, 90., 1., 0.5, 24.9],
    [25., 25., 0.1, 50., 1., 0.5, 24.0],
    [25., 25., 0.6, 50., 1., 0.5, 21.3],
    [25., 25., 1.1, 50., 1., 0.5, 20.2],
    [25., 25., 3.0, 50., 1., 0.5, 18.6],
    [25., 10., 0.15, 50., 1., 0.5, 15.3],
    [25., 40., 0.15, 50., 1., 0.5, 31.6],
    [25., 25., 0.15, 50., 1., 0.1, 20.7],
    [25., 25., 0.15, 50., 1., 1.0, 27.2],
    [25., 25., 0.15, 50., 1., 2.0, 32.4],
    [25., 25., 0.15, 50., 1., 4.0, 37.6],
    [25., 25., 0.15, 50., 0.8, 0.5, 23.3],
    [25., 25., 0.15, 50., 2.0, 0.5, 25.9],
    [25., 25., 0.15, 50., 4.0, 0.5, 30.5],
];

const SET_IDS: [&str; 22] = [
    "env.set.01",
    "env.set.02",
    "env.set.03",
    "env.set.04",
    "env.set.05",
    "env.set.06",
    "env.set.07",
    "env.set.08",
    "env.set.09",
    "env.set.10",
    "env.set.11",
    "env.set.12",
    "env.set.13",
    "env.set.14",
    "env.set.15",
    "env.set.16",
    "env.set.17",
    "env.set.18",
    "env.set.19",
    "env.set.20",
    "env.set.21",
    "env.set.22",
];

/// Строка тестовых значений pythermalcomfort `two_nodes_gagge`.
struct PtcRow {
    id: &'static str,
    /// t_a, t_r, v, RH, мет, кло.
    inputs: [f64; 6],
    w_max: Option<f64>,
    max_sweating: Option<f64>,
    max_blood_flow: Option<f64>,
    what: &'static str,
    pick: fn(&two_node::TwoNodeOutput) -> f64,
    reference: f64,
    tol: f64,
    unit: &'static str,
}

/// Выборка из ts_two_nodes_gagge.json (validation-data-comfort-models v1.1.0, MIT).
const PTC_TWO_NODE_ROWS: [PtcRow; 8] = [
    PtcRow {
        id: "env.twonode.core_40",
        inputs: [40., 40., 1.1, 50., 2., 0.5],
        w_max: None,
        max_sweating: None,
        max_blood_flow: None,
        what: "T ядра",
        pick: |o| o.t_core_c,
        reference: 38.1,
        tol: 0.1,
        unit: "°C",
    },
    PtcRow {
        id: "env.twonode.skin_40",
        inputs: [40., 40., 1.1, 50., 2., 0.5],
        w_max: None,
        max_sweating: None,
        max_blood_flow: None,
        what: "T кожи",
        pick: |o| o.t_skin_c,
        reference: 37.4,
        tol: 0.1,
        unit: "°C",
    },
    PtcRow {
        id: "env.twonode.sweat_40",
        inputs: [40., 40., 1.1, 50., 2., 0.5],
        w_max: None,
        max_sweating: None,
        max_blood_flow: None,
        what: "потоотделение",
        pick: |o| o.sweat_g_h_m2,
        reference: 133.6,
        tol: 6.0,
        unit: "г/(ч·м²)",
    },
    PtcRow {
        id: "env.twonode.core_wmax",
        inputs: [40., 40., 1.1, 50., 2., 0.5],
        w_max: Some(0.2),
        max_sweating: None,
        max_blood_flow: None,
        what: "T ядра при w_max = 0.2",
        pick: |o| o.t_core_c,
        reference: 39.2,
        tol: 0.1,
        unit: "°C",
    },
    PtcRow {
        id: "env.twonode.sweat_cap",
        inputs: [45., 45., 1.1, 20., 3., 0.2],
        w_max: None,
        max_sweating: Some(300.0),
        max_blood_flow: None,
        what: "потоотделение при пределе 300",
        pick: |o| o.sweat_g_h_m2,
        reference: 300.0,
        tol: 6.0,
        unit: "г/(ч·м²)",
    },
    PtcRow {
        id: "env.twonode.core_bloodcap",
        inputs: [45., 45., 1.1, 20., 3., 0.2],
        w_max: None,
        max_sweating: None,
        max_blood_flow: Some(60.0),
        what: "T ядра при кровотоке ≤ 60",
        pick: |o| o.t_core_c,
        reference: 38.4,
        tol: 0.1,
        unit: "°C",
    },
    PtcRow {
        id: "env.twonode.esk_office",
        inputs: [25., 25., 0.1, 50., 1.2, 0.5],
        w_max: None,
        max_sweating: None,
        max_blood_flow: None,
        what: "испарение с кожи",
        pick: |o| o.e_skin,
        reference: 18.1,
        tol: 4.0,
        unit: "Вт/м²",
    },
    PtcRow {
        id: "env.twonode.w_warm",
        inputs: [30., 30., 0.1, 50., 1.2, 0.5],
        w_max: None,
        max_sweating: None,
        max_blood_flow: None,
        what: "влажность кожи",
        pick: |o| o.wettedness,
        reference: 0.3,
        tol: 0.02,
        unit: "",
    },
];

/// Сверки двухузловой модели.
pub fn two_node_checks() -> Vec<Check> {
    let mut v = Vec::new();
    for (row, id) in ASHRAE55_SET_TABLE.iter().zip(SET_IDS) {
        let mut inp = TwoNodeInputs::ashrae55(row[0], row[1], row[2], row[3], row[4], row[5]);
        inp.wettedness_cap = WettednessCap::Ashrae55Code;
        let out = two_node::two_node(&inp);
        let mut note = String::from(
            "таблица округлена до 0.1 К; расчёт с ограничением влажности кожи как в коде ASHRAE",
        );
        let mut default = inp.clone();
        default.wettedness_cap = WettednessCap::Gagge1986;
        let alt = two_node::two_node(&default).set_c;
        if (alt - out.set_c).abs() > 0.01 {
            note = format!(
                "кожа упёрлась в w_max: таблица ASHRAE получена кодом с p_rsw = w_max/0.94; \
                 вариант Gagge 1986 (по умолчанию в крейте и в pythermalcomfort ≥ 4.7.0, \
                 issue #489) даёт SET = {alt:.2} °C — на {:+.2} К от таблицы",
                alt - row[6]
            );
        }
        v.push(chk(
            id,
            format!(
                "SET: t_a = {} °C, t_r = {} °C, v = {} м/с, RH = {} %, {} мет, {} кло",
                row[0], row[1], row[2], row[3], row[4], row[5]
            ),
            out.set_c,
            row[6],
            "°C",
            "двухузловая модель Гагге (порт pythermalcomfort/ASHRAE), 59 шагов по 1 мин",
            "ANSI/ASHRAE 55-2020, Appendix D, таблица проверки SET",
            Tolerance::Absolute(0.1),
            note,
        ));
    }
    // тестовые значения pythermalcomfort two_nodes_gagge (ts_two_nodes_gagge.json, MIT)
    let src =
        "pythermalcomfort two_nodes_gagge: тестовые значения ts_two_nodes_gagge.json (v1.1.0)";
    for r in PTC_TWO_NODE_ROWS {
        let i = r.inputs;
        let mut inp = TwoNodeInputs::ashrae55(i[0], i[1], i[2], i[3], i[4], i[5]);
        inp.w_max = r.w_max;
        if let Some(s) = r.max_sweating {
            inp.max_sweating = s;
        }
        if let Some(b) = r.max_blood_flow {
            inp.max_skin_blood_flow = b;
        }
        let out = two_node::two_node(&inp);
        v.push(chk(
            r.id,
            format!(
                "Двухузловая модель, {}: {} °C, {} м/с, {} %, {} мет, {} кло",
                r.what, i[0], i[2], i[3], i[4], i[5]
            ),
            (r.pick)(&out),
            r.reference,
            r.unit,
            "наш порт, 59 шагов по 1 мин",
            src,
            Tolerance::Absolute(r.tol),
            "допуск — из файла тестовых значений pythermalcomfort (значения округлены)",
        ));
    }
    // Второй численный способ: непрерывная постановка + РК4 мелким шагом
    let hot = TwoNodeInputs::ashrae55(35.0, 35.0, 0.5, 50.0, 3.0, 0.5);
    let cold = TwoNodeInputs::ashrae55(5.0, 5.0, 1.0, 50.0, 1.5, 1.0);
    let euler_hot = two_node::two_node(&hot);
    let euler_cold = two_node::two_node(&cold);
    let (rk_hot_core, _) = two_node::two_node_rk4(&hot, 0.05);
    let (_, rk_cold_skin) = two_node::two_node_rk4(&cold, 0.05);
    v.push(chk(
        "env.twonode.rk4_hot",
        "Работа в жаре 35 °C, 3 мет, 59 мин: T ядра — схема ASHRAE (Эйлер, 1 мин) против РК4 (0.05 мин)",
        euler_hot.t_core_c,
        rk_hot_core,
        "°C",
        "явная схема эталонного кода, шаг 1 мин",
        "та же модель в непрерывной постановке, РК4, шаг 3 с",
        Tolerance::Absolute(0.05),
        "разница — ошибка дискретизации эталонного кода (≈ 0.01 К)",
    ));
    v.push(chk(
        "env.twonode.rk4_cold",
        "Холод 5 °C, ветер 1 м/с, 1 кло, 59 мин: T кожи — схема ASHRAE против РК4",
        euler_cold.t_skin_c,
        rk_cold_skin,
        "°C",
        "явная схема эталонного кода, шаг 1 мин",
        "РК4, шаг 3 с",
        Tolerance::Absolute(0.1),
        "кожа на холоде остывает быстро — ошибка шага 1 мин ≈ 0.05 К; вне области ASHRAE 55 \
         (t < 10 °C) — модель выдаёт предупреждение",
    ));
    let fine = two_node::two_node(&hot.clone().with_exposure(59.0, 0.05));
    v.push(chk(
        "env.twonode.euler_converges",
        "Сходимость: явная схема с шагом 0.05 мин против РК4 (жара, 3 мет), T ядра",
        fine.t_core_c,
        rk_hot_core,
        "°C",
        "явная схема, шаг 3 с",
        "РК4, шаг 3 с",
        Tolerance::Absolute(0.005),
        "при уменьшении шага схема ASHRAE сходится к решению РК4",
    ));
    let e = euler_hot.energy;
    v.push(chk(
        "env.twonode.energy",
        "Баланс энергии за 59 мин (жара, 3 мет): накопление = метаболизм − работа − потери",
        e.storage_j,
        e.metabolic_j - e.work_j - e.dry_j - e.evaporative_j - e.respiratory_j,
        "Дж",
        "c·m·[α·ΔT_кожи + (1 − α)·ΔT_ядра] по шагам",
        "∫(M − W − C − R − E_кожи − дыхание)dt; сухие потери пересчитаны по поверхности одежды",
        Tolerance::Relative(1e-6),
        "проверяет согласованность сети сопротивлений одежды и воздуха с балансом узлов",
    ));
    let long = two_node::two_node(
        &TwoNodeInputs::ashrae55(25.0, 25.0, 0.15, 50.0, 1.0, 0.5).with_exposure(480.0, 1.0),
    );
    v.push(chk(
        "env.twonode.steady",
        "Нейтральная среда, 8 ч: установившийся режим — потери равны теплопродукции",
        long.q_sensible + long.e_skin + long.q_respiration,
        long.state.metabolic_w_m2,
        "Вт/м²",
        "сухие + испарение + дыхание в конце расчёта",
        "метаболизм (работа 0)",
        Tolerance::Relative(0.005),
        "",
    ));
    v
}

// ---------------------------------------------------------------------------
// Погружение
// ---------------------------------------------------------------------------

/// Сверки модели погружения.
pub fn immersion_checks() -> Vec<Check> {
    let mut v = Vec::new();
    for (i, (id, t)) in [
        ("env.immersion.5c", 5.0),
        ("env.immersion.10c", 10.0),
        ("env.immersion.15c", 15.0),
    ]
    .into_iter()
    .enumerate()
    {
        let (lo, hi) = immersion::survival_models_range_h(i);
        let hay = immersion::hayward_time_to_core_30_min(t).unwrap_or(f64::NAN) / 60.0;
        v.push(chk(
            id,
            format!("Вода {t:.0} °C: время охлаждения ядра до 30 °C (Hayward 1975) против разброса 5 моделей выживания"),
            hay,
            0.5 * (lo + hi),
            "ч",
            "Hayward et al. 1975: 15 + 7.2/(0.0785 − 0.0034·T) мин",
            format!("Oakley & Pethybridge 1997 через TP 13822: {lo}–{hi} ч (середина ± половина разброса)"),
            Tolerance::Absolute(0.5 * (hi - lo)),
            "Hayward — время до потери сознания (30 °C), модели — до гибели; оба для лёгкой \
             одежды и жилета. Разброс между людьми больше разброса моделей",
        ));
    }
    v
}

// ---------------------------------------------------------------------------
// Контакт кожи с материалом
// ---------------------------------------------------------------------------

/// Сверки модели контакта.
pub fn contact_checks() -> Vec<Check> {
    let mut v = Vec::new();
    let skin = Material::Skin.props();
    for (id, mat, tm) in [
        ("env.contact.fd_pla_hot", Material::Pla, 60.0),
        (
            "env.contact.fd_steel_cold",
            Material::StainlessSteel304,
            -10.0,
        ),
    ] {
        let mp = mat.props();
        let mut p = contact::semi_infinite_problem(&mp, tm, &skin, 33.0, 10.0, 100);
        p.material[0].cells = 300; // разные шаги по сторонам — восстановление на границе не «точное»
        let fd = p.contact_temperatures(&[10.0], 0.005)[0];
        v.push(chk(
            id,
            format!(
                "Касание: {} при {tm} °C, кожа 33 °C, 10 с — численно против формулы через e",
                mp.name
            ),
            fd,
            contact::contact_temperature(skin.effusivity(), 33.0, mp.effusivity(), tm),
            "°C",
            "1D конечные объёмы, Кранк–Николсон, 300 + 100 ячеек, dt = 5 мс",
            "T_c = (e_s·T_s + e_m·T_m)/(e_s + e_m), полубесконечные тела",
            Tolerance::Absolute(1e-3),
            "кожа — IT'IS v4.1 (k 0.37, ρ 1109, c 3391); без контактного сопротивления",
        ));
    }
    // порядок сходимости по полному профилю
    let pla = Material::Pla.props();
    let err = |cells: usize| {
        let p = contact::semi_infinite_problem(&pla, 60.0, &skin, 33.0, 1.0, cells);
        let (_, prof) = p.run(&[1.0], 1.0 / cells as f64);
        prof.iter()
            .map(|&(x, t)| {
                (t - contact::semi_infinite_exact(&pla, 60.0, &skin, 33.0, x, 1.0)).abs()
            })
            .fold(0.0, f64::max)
    };
    let (e1, e2) = (err(100), err(200));
    v.push(chk(
        "env.contact.fd_order",
        format!("Сходимость численной модели контакта: порядок по max-норме профиля (100→200 ячеек, ошибки {e1:.2e} → {e2:.2e} К)"),
        (e1 / e2).log2(),
        2.0,
        "",
        "log₂(e_h/e_{h/2}), dt ∝ h",
        "теоретический второй порядок (центральные разности + Кранк–Николсон)",
        Tolerance::Absolute(0.2),
        "точное решение — erf-профили двух полубесконечных тел",
    ));
    // слой PLA 1 мм на алюминии: численно против точного решения
    let al = Material::Aluminium6061.props();
    let problem = contact::ContactProblem {
        material: vec![
            contact::Layer {
                props: al,
                thickness: 0.6,
                t_init_c: -20.0,
                cells: 600,
                perfusion: None,
            },
            contact::Layer {
                props: pla,
                thickness: 1e-3,
                t_init_c: -20.0,
                cells: 40,
                perfusion: None,
            },
        ],
        tissue: vec![contact::Layer {
            props: skin,
            thickness: 0.025,
            t_init_c: 30.0,
            cells: 1000,
            perfusion: None,
        }],
        back: contact::Boundary::Adiabatic,
        deep: contact::Boundary::Adiabatic,
    };
    let times = [5.0, 30.0];
    let fd = problem.contact_temperatures(&times, 0.005);
    let breakthrough = contact::layer_breakthrough_time_s(&pla, 1e-3, &al, 30.0, -20.0);
    for (k, (id, t)) in [
        ("env.contact.layer_5s", times[0]),
        ("env.contact.layer_30s", times[1]),
    ]
    .into_iter()
    .enumerate()
    {
        v.push(chk(
            id,
            format!("Палец на PLA 1 мм поверх алюминия −20 °C, {t:.0} с: численно против точного решения"),
            fd[k],
            contact::layered_contact_temperature(&skin, 30.0, &pla, 1e-3, &al, -20.0, t),
            "°C",
            "конечные объёмы: алюминий 600 + PLA 40 + кожа 1000 ячеек",
            "точное решение «слой на подложке» (Лаплас, ряд по отражениям)",
            Tolerance::Absolute(5e-3),
            format!(
                "толщина слоя меняет ответ: сразу контакт как с PLA ({:.1} °C), через {breakthrough:.0} с \
                 сдвиг на 10 % к алюминию, в пределе — как с голым алюминием ({:.1} °C)",
                contact::contact_temperature(skin.effusivity(), 30.0, pla.effusivity(), -20.0),
                contact::contact_temperature(skin.effusivity(), 30.0, al.effusivity(), -20.0),
            ),
        ));
    }
    v
}

// ---------------------------------------------------------------------------
// Солнце
// ---------------------------------------------------------------------------

/// Сверки нагрева поверхностей солнцем.
pub fn solar_checks() -> Vec<Check> {
    let mut v = Vec::new();
    for (id, idx) in [
        ("env.solar.black_anodised", 0usize),
        ("env.solar.white_paint", 3),
    ] {
        let s = &effects::SURFACES[idx].0;
        let h_o = 17.0;
        let t_sol = effects::sol_air_temperature(s, 1000.0, 35.0, true, h_o);
        // лучистая часть h_o — линеаризация при средней температуре
        let h_r = 4.0
            * s.emissivity
            * convection::STEFAN_BOLTZMANN
            * (air::T0 + 0.5 * (t_sol + 35.0)).powi(3);
        let t_nl = effects::solar_surface_temperature(s, 1000.0, 35.0, h_o - h_r, 1.0, 35.0);
        v.push(chk(
            id,
            format!("{} на солнце 1000 Вт/м², воздух 35 °C, горизонтально: баланс против sol-air ASHRAE", s.name),
            t_nl,
            t_sol,
            "°C",
            "нелинейный баланс: α·G = h_c·ΔT + ε·σ·(T⁴ − T_неба⁴), небо по Swinbank 1963",
            "T_sol-air = T_a + α·G/h_o − ε·ΔR/h_o, h_o = 17, ΔR = 63 Вт/м² (ASHRAE Fundamentals, гл. 18)",
            Tolerance::Absolute(1.0),
            "h_c = h_o − h_r(лин.); расхождение — линеаризация излучения и разные модели неба",
        ));
    }
    v
}
