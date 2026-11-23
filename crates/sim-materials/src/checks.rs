//! Эталонные сверки блока: каждая сравнивает код с независимым вторым способом — численным
//! интегрированием, численным решением задачи на собственные значения, примером из учебника,
//! таблицей стандарта, чтением графика или ручным расчётом.
//!
//! Показания графиков Shigley (табл. A-15, 9-е изд., с. 1026–1027) сняты глазами с точностью
//! около ±0.03, поэтому допуски на них шире.

use std::f64::consts::PI;

use sim_verify::{Check, Tolerance};

use crate::assess::{
    Fastening, FindingKind, HoleOrientation, LoadCase, PartFinding, PartSpec, PrintSpec,
    StressRaiser, assess_part,
};
use crate::catalog::{AL_6061_T6, AL6061_YIELD_VS_T, MPA, MaterialId, temperature_factor};
use crate::fasteners::{
    BoltGrade, BoltSize, FastenerSpec, MemberStrength, ThreadEngagement, check_fasteners,
    thread_strip_area,
};
use crate::fatigue::{
    Reliability, SnCurve, StressBlock, Surface, fatigue_notch_factor, goodman_factor,
    kt_fillet_bending, kt_fillet_tension, kt_hole_bending_in_plane, kt_hole_bending_out_of_plane,
    kt_hole_tension, kt_hole_tension_alt, miner_damage, notch_sensitivity_steel, size_factor,
    surface_factor_10e,
};
use crate::printability::PrintGeometry;
use crate::printed::PrintProcess;
use crate::section::{
    EndCondition, Section, bending_stress, critical_load, rect_lateral_buckling, rect_torsion_alpha,
};

const KPSI: f64 = 6.894_757e6;

#[allow(clippy::too_many_arguments)]
fn chk(
    id: &'static str,
    title: &str,
    simulated: f64,
    reference: f64,
    unit: &'static str,
    method: &str,
    reference_method: &str,
    tolerance: Tolerance,
    note: &str,
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

/// Численный интеграл методом средних прямоугольников.
fn midpoint(a: f64, b: f64, n: usize, f: impl Fn(f64) -> f64) -> f64 {
    let dx = (b - a) / n as f64;
    (0..n).map(|i| f(a + (i as f64 + 0.5) * dx) * dx).sum()
}

/// Наименьшее собственное значение задачи −y″ = λ·y, y(0) = y(L) = 0, обратными итерациями
/// на сетке (метод прогонки). Для шарнирного стержня Pcr = λ·E·I.
fn smallest_column_eigenvalue(length: f64, cells: usize) -> f64 {
    let n = cells - 1;
    let h = length / cells as f64;
    let (diag, off) = (2.0 / (h * h), -1.0 / (h * h));
    let mut y: Vec<f64> = (0..n)
        .map(|i| ((i + 1) as f64 / cells as f64 * PI).sin() + 0.1)
        .collect();
    let mut lambda = 0.0;
    for _ in 0..60 {
        // Прогонка для трёхдиагональной матрицы.
        let mut c = vec![0.0; n];
        let mut d = vec![0.0; n];
        c[0] = off / diag;
        d[0] = y[0] / diag;
        for i in 1..n {
            let m = diag - off * c[i - 1];
            c[i] = off / m;
            d[i] = (y[i] - off * d[i - 1]) / m;
        }
        let mut x = vec![0.0; n];
        x[n - 1] = d[n - 1];
        for i in (0..n - 1).rev() {
            x[i] = d[i] - c[i] * x[i + 1];
        }
        let num: f64 = y.iter().zip(&x).map(|(a, b)| a * b).sum();
        let den: f64 = x.iter().map(|a| a * a).sum();
        lambda = num / den;
        let norm = den.sqrt();
        y = x.iter().map(|a| a / norm).collect();
    }
    lambda
}

/// Постоянная кручения прямоугольника a ≥ c рядом Сен-Венана (Timoshenko & Goodier).
fn rect_torsion_constant(a: f64, c: f64) -> f64 {
    let r = a / c;
    let sum: f64 = (0..41)
        .map(|i| {
            let n = (2 * i + 1) as f64;
            (n * PI * r / 2.0).tanh() / n.powi(5)
        })
        .sum();
    (1.0 - 192.0 / (PI.powi(5) * r) * sum) / 3.0 * a * c.powi(3)
}

fn value_of(findings: &[PartFinding], kind: FindingKind) -> f64 {
    findings
        .iter()
        .find(|f| f.kind == kind)
        .map_or(f64::NAN, |f| f.value)
}

fn section_checks(out: &mut Vec<Check>) {
    let (b, h) = (0.02, 0.01);
    let rect = Section::Rect { b, h };
    out.push(chk(
        "materials.section.rect_inertia",
        "Момент инерции прямоугольника 20×10 мм",
        rect.inertia() * 1e12,
        midpoint(-h / 2.0, h / 2.0, 4000, |y| y * y * b) * 1e12,
        "мм⁴",
        "Section::inertia: b·h³/12",
        "численный интеграл ∫y²·b dy",
        Tolerance::Relative(1e-6),
        "",
    ));

    let (od, wall) = (0.03, 0.002);
    let tube = Section::Tube { od, wall };
    let (ro, ri) = (od / 2.0, od / 2.0 - wall);
    let tube_i = midpoint(ri, ro, 600, |r| {
        midpoint(0.0, 2.0 * PI, 600, |t| (r * t.sin()).powi(2) * r)
    });
    out.push(chk(
        "materials.section.tube_inertia",
        "Момент инерции трубы Ø30×2 мм",
        tube.inertia() * 1e12,
        tube_i * 1e12,
        "мм⁴",
        "Section::inertia: π(D⁴ − d⁴)/64",
        "двойной численный интеграл в полярных координатах",
        Tolerance::Relative(1e-4),
        "",
    ));

    out.push(chk(
        "materials.section.rect_bending",
        "Изгиб полосы 20×10 мм моментом 10 Н·м",
        bending_stress(10.0, rect.section_modulus()) / MPA,
        30.0,
        "МПа",
        "M/W через Section",
        "ручной расчёт: 6·10/(0.02·0.01²) = 30 МПа",
        Tolerance::Relative(1e-9),
        "",
    ));

    // Касательные напряжения: τ = V·Q/(I·t), статический момент — численно.
    let v = 1000.0;
    let q_rect = midpoint(0.0, h / 2.0, 4000, |y| y * b);
    out.push(chk(
        "materials.section.rect_shear",
        "Сдвиг прямоугольника, максимум на нейтральной оси",
        rect.shear_stress(v) / MPA,
        v * q_rect / (rect.inertia() * b) / MPA,
        "МПа",
        "1.5·V/A",
        "V·Q/(I·b), Q — численный интеграл",
        Tolerance::Relative(1e-5),
        "",
    ));
    let d = 0.012;
    let round = Section::Round { d };
    let q_round = midpoint(0.0, d / 2.0, 20000, |y| {
        y * 2.0 * ((d / 2.0).powi(2) - y * y).sqrt()
    });
    out.push(chk(
        "materials.section.round_shear",
        "Сдвиг круга, максимум на нейтральной оси",
        round.shear_stress(v) / MPA,
        v * q_round / (round.inertia() * d) / MPA,
        "МПа",
        "4·V/(3·A)",
        "V·Q/(I·d), Q — численный интеграл",
        Tolerance::Relative(1e-4),
        "",
    ));
    let q_tube = midpoint(0.0, ro, 40000, |y| {
        let outer = 2.0 * (ro * ro - y * y).max(0.0).sqrt();
        let inner = if y < ri {
            2.0 * (ri * ri - y * y).sqrt()
        } else {
            0.0
        };
        y * (outer - inner)
    });
    out.push(chk(
        "materials.section.tube_shear",
        "Сдвиг трубы, максимум на нейтральной оси",
        tube.shear_stress(v) / MPA,
        v * q_tube / (tube.inertia() * 2.0 * wall) / MPA,
        "МПа",
        "V·Q/(I·2t), Q = ⅔(R³ − r³)",
        "то же с численным интегралом Q",
        Tolerance::Relative(1e-3),
        "",
    ));

    for (id, ratio, table) in [
        ("materials.section.torsion_alpha_1", 1.0, 0.208),
        ("materials.section.torsion_alpha_2", 2.0, 0.246),
        ("materials.section.torsion_alpha_4", 4.0, 0.282),
        ("materials.section.torsion_alpha_10", 10.0, 0.312),
    ] {
        out.push(chk(
            id,
            &format!("Кручение прямоугольника a/c = {ratio}: коэффициент α"),
            rect_torsion_alpha(ratio),
            table,
            "",
            "ряд Сен-Венана",
            "Shigley, табл. 3-1 (Timoshenko)",
            Tolerance::Absolute(0.0015),
            "таблица дана с тремя знаками",
        ));
    }

    let thin = Section::Tube {
        od: 0.03,
        wall: 0.0005,
    };
    let rm = (0.03 - 0.0005) / 2.0;
    out.push(chk(
        "materials.section.tube_torsion_bredt",
        "Кручение тонкой трубы Ø30×0.5 мм",
        thin.torsion_stress(10.0) / MPA,
        10.0 / (2.0 * PI * rm * rm * 0.0005) / MPA,
        "МПа",
        "T·r/J",
        "формула Бредта для тонкостенного контура",
        Tolerance::Relative(0.03),
        "Бредт даёт среднее по стенке, T·r/J — на наружной поверхности",
    ));
}

fn buckling_checks(out: &mut Vec<Check>) {
    let s = Section::Round { d: 0.01 };
    let (e, sy, l) = (200e9, 350e6, 1.0);
    let b = critical_load(&s, e, sy, l, EndCondition::PinnedPinned);
    out.push(chk(
        "materials.buckling.euler_eigenvalue",
        "Эйлер: шарнирный стержень Ø10×1000 мм",
        b.critical_load,
        smallest_column_eigenvalue(l, 800) * e * s.inertia(),
        "Н",
        "π²EA/λ²",
        "численная задача на собственные значения EI·y″ + P·y = 0",
        Tolerance::Relative(1e-4),
        "",
    ));

    let s = Section::Round { d: 0.02 };
    let (e, sy) = (207e9, 350e6);
    let b = critical_load(&s, e, sy, 0.3, EndCondition::PinnedPinned);
    // λ = 0.3/(0.02/4) = 60.
    let manual = PI * 0.02f64.powi(2) / 4.0 * sy * (1.0 - sy * 3600.0 / (4.0 * PI * PI * e));
    out.push(chk(
        "materials.buckling.johnson_manual",
        "Джонсон: короткий стержень Ø20×300 мм, λ = 60",
        b.critical_load,
        manual,
        "Н",
        "critical_load",
        "ручной расчёт по параболе Джонсона (Shigley, §4-14)",
        Tolerance::Relative(1e-9),
        "",
    ));

    // Боковое выпучивание полосы 40×2 мм, L = 1 м: упругая зона F11 против точного решения.
    let (depth, width, length, e_ltb, fy): (f64, f64, f64, f64, f64) =
        (0.04, 0.002, 1.0, 200e9, 250e6);
    let g = e_ltb / 2.6;
    let iy = depth * width.powi(3) / 12.0;
    let j = rect_torsion_constant(depth, width);
    out.push(chk(
        "materials.buckling.lateral_timoshenko",
        "Боковое выпучивание полосы 40×2×1000 мм",
        rect_lateral_buckling(depth, width, e_ltb, fy, length).unwrap_or(f64::NAN),
        PI / length * (e_ltb * iy * g * j).sqrt(),
        "Н·м",
        "AISC 360-16, F11-3/F11-4",
        "Тимошенко: Mcr = (π/L)·√(E·Iy·G·J), J — ряд Сен-Венана",
        Tolerance::Relative(0.04),
        "формула AISC ниже точной на 2–3%: сторона запаса",
    ));

    let r = s.radius_of_gyration_min();
    let at_transition = critical_load(&s, e, sy, b.transition * r, EndCondition::PinnedPinned);
    out.push(chk(
        "materials.buckling.transition",
        "Сшивка Эйлера и Джонсона на граничной гибкости",
        at_transition.critical_load,
        s.area() * sy / 2.0,
        "Н",
        "critical_load при λ = λт",
        "точное значение A·Sy/2",
        Tolerance::Relative(1e-6),
        "",
    ));
}

fn concentration_checks(out: &mut Vec<Check>) {
    out.push(chk(
        "materials.kt.kirsch_limit",
        "Малое отверстие в растянутой пластине",
        kt_hole_tension(0.0),
        3.0,
        "",
        "аппроксимация Pilkey при d/w = 0",
        "точное решение Кирша (1898): Kt = 3",
        Tolerance::Absolute(1e-9),
        "",
    ));
    for (id, x) in [
        ("materials.kt.hole_two_fits_025", 0.25),
        ("materials.kt.hole_two_fits_050", 0.5),
    ] {
        out.push(chk(
            id,
            &format!("Отверстие при растяжении, d/w = {x}: две аппроксимации"),
            kt_hole_tension(x),
            kt_hole_tension_alt(x),
            "",
            "Pilkey: 3 − 3.14x + 3.667x² − 1.527x³",
            "Peterson, диаграмма 4.1: 2 + 0.284y − 0.6y² + 1.32y³, y = 1 − x",
            Tolerance::Relative(0.01),
            "",
        ));
    }
    for (id, x, chart) in [
        ("materials.kt.hole_chart_02", 0.2, 2.50),
        ("materials.kt.hole_chart_05", 0.5, 2.18),
        ("materials.kt.hole_chart_08", 0.8, 2.07),
    ] {
        out.push(chk(
            id,
            &format!("Отверстие при растяжении, d/w = {x}: график"),
            kt_hole_tension(x),
            chart,
            "",
            "kt_hole_tension",
            "Shigley, рис. A-15-1 (чтение графика)",
            Tolerance::Relative(0.02),
            "",
        ));
    }

    // Изгиб из плоскости: формула не должна быть ниже графика. Сверяется невыгодная разность.
    for (id, dw, dh, chart) in [
        ("materials.kt.bend_oop_d1_w01", 0.1, 1.0, 2.05),
        ("materials.kt.bend_oop_d1_w03", 0.3, 1.0, 1.76),
        ("materials.kt.bend_oop_d2_w01", 0.1, 2.0, 1.87),
        ("materials.kt.bend_oop_d2_w03", 0.3, 2.0, 1.58),
    ] {
        out.push(chk(
            id,
            &format!("Изгиб из плоскости, d/w = {dw}, d/h = {dh}: не ниже графика"),
            (chart - kt_hole_bending_out_of_plane(dw, dh)).max(0.0),
            0.0,
            "",
            "max(0, Kt графика − Kt формулы)",
            "Shigley, рис. A-15-2 (чтение графика)",
            Tolerance::Absolute(0.03),
            "формула Pilkey выше графика, это сторона запаса",
        ));
    }

    out.push(chk(
        "materials.kt.bend_in_plane_edge",
        "Изгиб в плоскости: большое отверстие, d/h = 0.7",
        kt_hole_bending_in_plane(0.7),
        2.0 * 0.7,
        "",
        "max(1, 2d/D) к волокну нетто-сечения",
        "Pilkey: Kt = 2 у края отверстия при σ = 6Md/((D³ − d³)t), пересчёт к волокну: 2d/D",
        Tolerance::Absolute(1e-12),
        "",
    ));

    // Огибающая построена по этим же графикам, поэтому сверка подтверждает две вещи: итоговый
    // Kt не ниже показания и не уходит от него дальше допуска. Независимой она не является.
    for (id, big, r, chart) in [
        ("materials.kt.fillet_t_150_r10", 1.5, 0.10, 2.10),
        ("materials.kt.fillet_t_150_r20", 1.5, 0.20, 1.74),
        ("materials.kt.fillet_t_110_r05", 1.1, 0.05, 2.05),
        ("materials.kt.fillet_t_110_r10", 1.1, 0.10, 1.65),
        ("materials.kt.fillet_t_102_r10", 1.02, 0.10, 1.35),
    ] {
        let kt = kt_fillet_tension(big, 1.0, r).unwrap_or(f64::NAN);
        out.push(chk(
            id,
            &format!("Галтель, растяжение: D/d = {big}, r/d = {r}"),
            if kt >= chart { kt } else { f64::NAN },
            chart,
            "",
            "большее из аппроксимации Pilkey и графика; ниже графика — NaN (провал)",
            "Shigley, рис. A-15-5 (чтение графика)",
            Tolerance::Relative(0.08),
            "превышение над графиком — сторона запаса",
        ));
    }
    for (id, big, r, chart) in [
        ("materials.kt.fillet_b_300_r10", 3.0, 0.10, 1.89),
        ("materials.kt.fillet_b_130_r10", 1.3, 0.10, 1.74),
        ("materials.kt.fillet_b_110_r05", 1.1, 0.05, 1.94),
        ("materials.kt.fillet_b_110_r10", 1.1, 0.10, 1.60),
        ("materials.kt.fillet_b_102_r10", 1.02, 0.10, 1.41),
    ] {
        let kt = kt_fillet_bending(big, 1.0, r).unwrap_or(f64::NAN);
        out.push(chk(
            id,
            &format!("Галтель, изгиб: D/d = {big}, r/d = {r}"),
            if kt >= chart { kt } else { f64::NAN },
            chart,
            "",
            "большее из аппроксимации Pilkey и графика; ниже графика — NaN (провал)",
            "Shigley, рис. A-15-6 (чтение графика)",
            Tolerance::Relative(0.08),
            "превышение над графиком — сторона запаса",
        ));
    }

    let q = notch_sensitivity_steel(100.0 * KPSI, 3e-3, false);
    out.push(chk(
        "materials.fatigue.notch_shigley",
        "Kf стали: Sut = 100 kpsi, r = 3 мм, Kt = 1.65",
        fatigue_notch_factor(1.65, q),
        1.55,
        "",
        "Нойбер с полиномом √a (Shigley, ур. 6-35a)",
        "Shigley, пример 6-6: Kf = 1.55",
        Tolerance::Absolute(0.01),
        "",
    ));
}

fn fatigue_checks(out: &mut Vec<Check>) {
    // Shigley, пример 6-2: сталь 1050 HR, Sut = 90 kpsi, S'e = 45 kpsi.
    let curve = SnCurve::from_sut(90.0 * KPSI, 45.0 * KPSI, 1e6, true);
    out.push(chk(
        "materials.fatigue.basquin_strength",
        "Баскин: прочность на 10⁴ циклов, сталь 1050 HR",
        curve.strength_at(1e4) / KPSI,
        64.6,
        "kpsi",
        "SnCurve::from_sut",
        "Shigley, пример 6-2: Sf = 64.6 kpsi",
        Tolerance::Relative(0.01),
        "в учебнике f округлено до 0.86, у нас 0.864 по формуле",
    ));
    out.push(chk(
        "materials.fatigue.basquin_life",
        "Баскин: долговечность при 55 kpsi",
        curve.life(55.0 * KPSI),
        77_500.0,
        "циклов",
        "SnCurve::life",
        "Shigley, пример 6-2: N = 77 500",
        Tolerance::Relative(0.05),
        "долговечность чувствительна к округлению f",
    ));

    out.push(chk(
        "materials.fatigue.goodman_manual",
        "Модифицированный Гудман",
        goodman_factor(50e6, 100e6, 200e6, 400e6),
        2.0,
        "×",
        "goodman_factor",
        "ручной расчёт: 1/(50/200 + 100/400) = 2",
        Tolerance::Absolute(1e-9),
        "",
    ));

    let c = SnCurve::through(1e3, 300e6, 1e6, 150e6, false);
    let blocks = [
        StressBlock {
            sigma_a: 200e6,
            sigma_m: 0.0,
            cycles: 1e4,
        },
        StressBlock {
            sigma_a: 150e6,
            sigma_m: 0.0,
            cycles: 2e5,
        },
    ];
    // N(200) = 10³·1.5^(3/lg 2), N(150) = 10⁶ по построению кривой.
    let n200 = 1e3 * 1.5f64.powf(3.0 / 2f64.log10());
    out.push(chk(
        "materials.fatigue.miner_manual",
        "Правило Майнера, два блока",
        miner_damage(&c, 400e6, &blocks),
        1e4 / n200 + 2e5 / 1e6,
        "",
        "miner_damage",
        "ручной расчёт: Σn/N по двум точкам кривой",
        Tolerance::Relative(1e-9),
        "",
    ));

    out.push(chk(
        "materials.fatigue.marin_ka",
        "Коэффициент поверхности: обработка, Sut = 520 МПа",
        surface_factor_10e(Surface::Machined, 520.0),
        0.860,
        "",
        "surface_factor_10e",
        "Shigley, пример 6-3: ka = 0.860",
        Tolerance::Absolute(0.002),
        "",
    ));
    out.push(chk(
        "materials.fatigue.marin_kb",
        "Коэффициент размера, Ø32 мм",
        size_factor(0.032),
        0.858,
        "",
        "size_factor: 1.24·d^−0.107",
        "Shigley: (32/7.62)^−0.107 = 0.858",
        Tolerance::Relative(0.01),
        "две формы одной аппроксимации",
    ));
}

fn fastener_checks(out: &mut Vec<Check>) {
    let worst = BoltSize::ALL
        .iter()
        .map(|s| {
            let f = PI / 4.0 * (s.d() - 0.9382 * s.pitch()).powi(2);
            (f - s.stress_area()).abs() / s.stress_area()
        })
        .fold(0.0, f64::max);
    out.push(chk(
        "materials.fasteners.stress_area",
        "Площадь резьбы M2–M8: таблица против формулы",
        worst,
        0.0,
        "отн.",
        "наибольшее расхождение таблицы ISO 898-1",
        "A = π/4·(d − 0.9382·P)²",
        Tolerance::Absolute(0.01),
        "таблица округлена до трёх значащих цифр",
    ));

    let (d, p, l) = (4e-3, 0.7e-3, 6e-3);
    let turns = l / p;
    let mach = PI * turns * d * (p / 2.0 + 0.57735 * 0.6495 * p);
    out.push(chk(
        "materials.fasteners.strip_area",
        "Площадь среза витков M4, зацепление 6 мм",
        thread_strip_area(d, l) * 1e6,
        mach * 1e6,
        "мм²",
        "0.875·π·d·L",
        "Machinery's Handbook: π·n·Le·D·(1/(2n) + 0.57735·(D − E))",
        Tolerance::Relative(1e-3),
        "",
    ));

    let strong = MemberStrength {
        bearing: 240e6,
        shear: 138e6,
        thread_factor: 1.0,
        model_applies: true,
    };
    let spec = FastenerSpec {
        size: BoltSize::M6,
        grade: BoltGrade::G8_8,
        count: 1,
        engagement: ThreadEngagement::ThroughBolt,
        plate_thickness: 6e-3,
        edge_distance: 12e-3,
    };
    let r = check_fasteners(&spec, &strong, 1000.0, 0.0, 0.0, 0.0);
    let by_name = |n: &str| {
        r.items
            .iter()
            .find(|i| i.name == n)
            .map_or(f64::NAN, |i| i.factor)
    };
    let d3 = 6.0 - 1.226_869;
    out.push(chk(
        "materials.fasteners.bolt_shear",
        "Срез M6 8.8 силой 1 кН по резьбе",
        by_name("срез болта"),
        0.577 * 580.0 * (PI / 4.0 * d3 * d3) / 1000.0,
        "×",
        "check_fasteners",
        "ручной расчёт: 0.577·Sp·π/4·d₃²/F",
        Tolerance::Relative(1e-6),
        "",
    ));
    out.push(chk(
        "materials.fasteners.bearing",
        "Смятие под M6, лист 6 мм, Sy = 240 МПа",
        by_name("смятие под болтом"),
        240.0 * 6.0 * 6.0 / 1000.0,
        "×",
        "check_fasteners",
        "ручной расчёт: Sy·d·t/F",
        Tolerance::Relative(1e-9),
        "",
    ));
}

#[allow(clippy::approx_constant)] // 3.14 — коэффициент аппроксимации Pilkey, а не π
fn assess_checks(out: &mut Vec<Check>) {
    out.push(chk(
        "materials.temperature.al6061",
        "6061-T6 при 149 °C: доля предела текучести",
        temperature_factor(&AL_6061_T6, 149.0),
        AL6061_YIELD_VS_T[2].1 / AL6061_YIELD_VS_T[0].1,
        "",
        "temperature_factor",
        "MatWeb/AA: 214 МПа при 149 °C против 276 при 24 °C",
        Tolerance::Absolute(1e-12),
        "",
    ));

    let mut part = PartSpec::new(
        "полоса 20×10",
        MaterialId::Al6061T6,
        Section::Rect { b: 0.02, h: 0.01 },
        0.2,
    );
    part.fastening = Fastening::NoFasteners;
    let load = LoadCase {
        bending: 10.0,
        temperature_c: 23.0,
        ..LoadCase::default()
    };
    out.push(chk(
        "materials.assess.static_manual",
        "assess_part: статика полосы 6061 при 10 Н·м",
        value_of(&assess_part(&part, &load), FindingKind::Static),
        240.0 / 30.0,
        "×",
        "assess_part",
        "ручной расчёт: σ = 30 МПа, Sy = 240 МПа",
        Tolerance::Relative(1e-9),
        "",
    ));

    let mut bar = PartSpec::new(
        "вал",
        MaterialId::SteelS235,
        Section::Round { d: 0.01 },
        0.1,
    );
    bar.reliability = Reliability::R99;
    bar.fastening = Fastening::NoFasteners;
    let load = LoadCase {
        bending: 1.0,
        temperature_c: 23.0,
        cycles: 1e7,
        load_ratio: -1.0,
        ..LoadCase::default()
    };
    // Вся цепочка вручную: S'e = 0.5·360, ka = 3.04·360^−0.217, kb = 1.24·10^−0.107, ke = 0.814.
    let se = 180.0 * (3.04 * 360f64.powf(-0.217)) * (1.24 * 10f64.powf(-0.107)) * 0.814;
    let sigma = 32.0 * 1.0 / (PI * 0.01f64.powi(3)) / 1e6;
    out.push(chk(
        "materials.assess.fatigue_manual",
        "assess_part: усталость вала S235, симметричный изгиб",
        value_of(&assess_part(&bar, &load), FindingKind::Fatigue),
        se / sigma,
        "×",
        "assess_part",
        "ручная цепочка Марина: Se/σ",
        Tolerance::Relative(1e-6),
        "",
    ));

    // Отверстие: S235, полоса 30×5, Ø10 по оси, растяжение R = −1.
    let mut strip = PartSpec::new(
        "полоса с отверстием",
        MaterialId::SteelS235,
        Section::Rect { b: 0.03, h: 0.005 },
        0.1,
    );
    strip.raiser = StressRaiser::Hole {
        d: 0.01,
        orientation: HoleOrientation::OutOfPlane,
        centered: true,
    };
    strip.fastening = Fastening::NoFasteners;
    let load = LoadCase {
        axial: 1000.0,
        temperature_c: 23.0,
        cycles: 1e7,
        load_ratio: -1.0,
        ..LoadCase::default()
    };
    let x = 1.0 / 3.0;
    let kt = 3.0 - 3.14 * x + 3.667 * x * x - 1.527 * x * x * x;
    let s_kpsi: f64 = 360.0 / 6.894_757;
    let sqrt_a = 0.246 - 3.08e-3 * s_kpsi + 1.51e-5 * s_kpsi.powi(2) - 2.67e-8 * s_kpsi.powi(3);
    let q = 1.0 / (1.0 + sqrt_a / (5.0 / 25.4f64).sqrt());
    let kf = 1.0 + q * (kt - 1.0);
    let se = 180.0 * (3.04 * 360f64.powf(-0.217)) * (1.24 * 30f64.powf(-0.107)) * 0.814;
    let sigma_net = 1000.0 / (20.0 * 5.0); // МПа
    out.push(chk(
        "materials.assess.hole_fatigue_manual",
        "assess_part: усталость полосы с отверстием при растяжении",
        value_of(&assess_part(&strip, &load), FindingKind::Fatigue),
        se / (kf * sigma_net / 0.85),
        "×",
        "assess_part",
        "ручной расчёт: Se/(Kf·σнетто/kc), Kf из Kt и q",
        Tolerance::Relative(1e-3),
        "",
    ));

    // Печатная деталь поперёк слоёв: 28 МПа · 0.7 = 19.6 МПа, σ = 196 Н / 100 мм² = 1.96 МПа.
    let mut printed = PartSpec::new(
        "кубик PLA",
        MaterialId::Pla,
        Section::Rect { b: 0.01, h: 0.01 },
        0.02,
    );
    printed.fastening = Fastening::NoFasteners;
    printed.print = Some(PrintSpec {
        process: PrintProcess {
            infill: 1.0,
            load_angle_deg: 90.0,
            ..PrintProcess::default()
        },
        geometry: PrintGeometry {
            bbox: [0.01, 0.01, 0.02],
            build_volume: [0.25, 0.25, 0.25],
            min_wall: 0.01,
            max_overhang_deg: 0.0,
            max_bridge: 0.0,
            min_hole: 0.0,
            supports: false,
            enclosed: false,
        },
    });
    let load = LoadCase {
        axial: 196.0,
        temperature_c: 23.0,
        ..LoadCase::default()
    };
    out.push(chk(
        "materials.assess.printed_across",
        "assess_part: PLA поперёк слоёв, 196 Н на 10×10 мм",
        value_of(&assess_part(&printed, &load), FindingKind::Static),
        19.6 / 1.96,
        "×",
        "assess_part",
        "ручной расчёт: 28 МПа · 0.7 / 1.96 МПа = 10",
        Tolerance::Relative(1e-9),
        "",
    ));
}

/// Все эталонные сверки блока.
pub fn all() -> Vec<Check> {
    let mut out = Vec::new();
    section_checks(&mut out);
    buckling_checks(&mut out);
    concentration_checks(&mut out);
    fatigue_checks(&mut out);
    fastener_checks(&mut out);
    assess_checks(&mut out);
    out
}
