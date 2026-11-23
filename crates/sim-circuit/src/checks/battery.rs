//! Модель ячейки LG INR21700-M50 против аналитики, даташита и опубликованных измерений.

use sim_verify::{Check, Tolerance};

use super::mk;
use crate::battery::{Cell, CellParams, discharge_to_cutoff, lg_m50_ocv_chen2020};

pub fn all() -> Vec<Check> {
    let mut v = vec![
        coulomb_counting(),
        rest_ocv(),
        energy_balance(),
        dcir_datasheet(),
    ];
    v.extend(capacity_c2());
    v.extend(ocv_vs_independent());
    v
}

/// Профиль тока для сверки кулоновского счёта: I(t) = 2 + 1.5·sin(2πt/600) А.
fn profile(t: f64) -> f64 {
    2.0 + 1.5 * (std::f64::consts::TAU * t / 600.0).sin()
}

const PROFILE_T: f64 = 3900.0;

fn run_profile(dt: f64) -> Cell {
    let mut cell = Cell::new(CellParams::lg_m50());
    let n = (PROFILE_T / dt).round() as usize;
    for k in 0..n {
        cell.step(profile((k as f64 + 0.5) * dt), dt, 25.0);
    }
    cell
}

pub fn coulomb_counting() -> Check {
    let cell = run_profile(1.0);
    let q = cell.params().capacity_ah;
    // ∫I dt = 2·T + 1.5·600/(2π)·(1 − cos(2πT/600)), Кл.
    let w = std::f64::consts::TAU / 600.0;
    let charge_c = 2.0 * PROFILE_T + 1.5 / w * (1.0 - (w * PROFILE_T).cos());
    mk(
        "battery.coulomb_counting",
        "Ячейка: SOC после переменного разряда (кулоновский счёт)",
        cell.soc(),
        1.0 - charge_c / 3600.0 / q,
        "",
        "шаг 1 с, ток в середине шага, 3900 с",
        "SOC₀ − ∫I dt/(3600·Q), интеграл синусоидального профиля аналитически",
        Tolerance::Absolute(1e-6),
        "I(t) = 2 + 1.5·sin(2πt/600) А, Q = 4.90 А·ч; правило средних точек даёт ошибку O(dt²)",
    )
}

pub fn rest_ocv() -> Check {
    let mut cell = run_profile(1.0);
    for _ in 0..(2 * 3600) {
        cell.step(0.0, 1.0, 25.0);
    }
    mk(
        "battery.rest_ocv",
        "Ячейка: напряжение после 2 ч отдыха = OCV(SOC)",
        cell.terminal_voltage(),
        lg_m50_ocv_chen2020(cell.soc()),
        "В",
        "модель: табличная OCV, релаксация звена RC (τ ≤ 75 с)",
        "OCV по формулам электродов Chen et al. 2020 при сосчитанном SOC",
        Tolerance::Absolute(2e-3),
        "проверяет затухание поляризации и интерполяцию таблицы OCV (шаг 1% SOC)",
    )
}

pub fn energy_balance() -> Check {
    // Импульсный разряд/заряд: 10 с по 10 А, 20 с отдых, 10 с заряд 2 А — 40 циклов.
    let mut cell = Cell::with_soc(CellParams::lg_m50(), 0.8);
    let dt = 0.25;
    let (mut e_chem, mut e_out, mut e_heat) = (0.0, 0.0, 0.0);
    let e_rc0 = cell.rc_energy_j();
    for k in 0..(40 * 160) {
        let ph = (k % 160) as f64 * dt;
        let i = if ph < 10.0 {
            10.0
        } else if ph < 30.0 {
            0.0
        } else if ph < 40.0 {
            -2.0
        } else {
            0.0
        };
        let ocv0 = cell.ocv();
        let v0 = cell.terminal_voltage_at(i);
        let q0 = {
            let mut c = cell.clone();
            c.step(i, 0.0, 25.0);
            c.irreversible_heat_w()
        };
        cell.step(i, dt, 25.0);
        e_chem += i * dt * 0.5 * (ocv0 + cell.ocv());
        e_out += i * dt * 0.5 * (v0 + cell.terminal_voltage());
        e_heat += dt * 0.5 * (q0 + cell.irreversible_heat_w());
    }
    let d_rc = cell.rc_energy_j() - e_rc0;
    mk(
        "battery.energy_balance",
        "Ячейка: баланс энергии (химическая = отданная + тепло + в RC)",
        e_out + e_heat + d_rc,
        e_chem,
        "Дж",
        "∫V·I dt + ∫(I²R0 + V₁²/R₁) dt + Δ(½C₁V₁²) по шагам 0.25 с",
        "∫OCV·I dt — энергия, отданная электрохимической ЭДС",
        Tolerance::Relative(1e-4),
        "40 циклов: 10 с разряд 10 А, 20 с отдых, 10 с заряд 2 А, 20 с отдых; обратимое \
         (энтропийное) тепло — обмен с окружением, в баланс ЭДС не входит",
    )
}

pub fn dcir_datasheet() -> Check {
    let mut cell = Cell::with_soc(CellParams::lg_m50(), 0.5);
    let v0 = cell.terminal_voltage_at(0.0);
    let i = 0.5 * 4.85;
    for _ in 0..300 {
        cell.step(i, 0.1, 25.0);
    }
    mk(
        "battery.dcir_datasheet",
        "Ячейка: внутреннее сопротивление DCIR (50% SOC, 0.5C, 30 с)",
        (v0 - cell.terminal_voltage()) / i,
        0.030,
        "Ом",
        "модель: R0 и R1, C1 по HPPC (Lone et al. 2025), ΔV/I через 30 с",
        "даташит LG INR21700-M50, §4.2.2: DCIR = 30 ± 6 мОм",
        Tolerance::Absolute(0.006),
        "в ΔV входит и сдвиг OCV за 30 с разряда (≈3.5 мВ) — так же, как при измерении",
    )
}

/// Ёмкость при C/2 (2.5 А) до 2.5 В при 25 и 0 °C против измерений Brosa Planella et al. 2021.
pub fn capacity_c2() -> Vec<Check> {
    let p = CellParams::lg_m50();
    let mut out = Vec::new();
    for (id, temp, measured) in [
        ("battery.capacity_c2_25c", 25.0, 4.776),
        ("battery.capacity_c2_0c", 0.0, 4.274),
    ] {
        let (q, _) = discharge_to_cutoff(&p, 2.5, temp, 1.0);
        out.push(mk(
            id,
            &format!("Ячейка: ёмкость разряда C/2 до 2.5 В при {temp:.0} °C"),
            q,
            measured,
            "А·ч",
            "модель: f(T) по ёмкостям C/10, R(T) по Аррениусу; разряд 2.5 А шагом 1 с",
            "измерение: Brosa Planella et al. 2021 (Electrochim. Acta 388:138524), открытые \
             данные, 2-й цикл, среднее по ячейкам",
            Tolerance::Relative(0.03),
            "данные C/2 при подборе параметров не использовались (только C/10 и скачки \
             напряжения); в опыте ячейка саморазогревается, у нас температура постоянна; \
             одно звено RC не описывает диффузионные потери у дна разряда — отсюда допуск 3%",
        ));
    }
    out
}

/// OCV (Chen 2020, ячейка LG M50) против независимо измеренного полинома OCV (LG M50T,
/// Lone et al. 2025, табл. I) в средней части диапазона.
pub fn ocv_vs_independent() -> Vec<Check> {
    let p = CellParams::lg_m50();
    [
        ("battery.ocv_vs_lone_20", 0.2, 3.472),
        ("battery.ocv_vs_lone_50", 0.5, 3.707),
        ("battery.ocv_vs_lone_80", 0.8, 4.011),
    ]
    .into_iter()
    .map(|(id, soc, reference)| {
        mk(
            id,
            &format!(
                "OCV ячейки при SOC {:.0}%: Chen 2020 против Lone 2025",
                soc * 100.0
            ),
            p.ocv.eval(soc),
            reference,
            "В",
            "таблица модели (электродные OCP по GITT, Chen et al. 2020)",
            "полином полной ячейки, измеренный на другом образце (M50T), arXiv:2508.14454",
            Tolerance::Absolute(0.03),
            "разные образцы, методы и ветви (разряд/заряд); препринт без рецензии",
        )
    })
    .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn energy_at_0c_not_below_datasheet_minimum() {
        // Даташит §4.3.2: при 0 °C, 0.2C — не менее 80% номинальной энергии (18.2 Вт·ч).
        let p = CellParams::lg_m50();
        let (_, e) = discharge_to_cutoff(&p, 0.2 * 4.85, 0.0, 2.0);
        assert!(
            e >= 0.8 * 18.2,
            "энергия при 0 °C {e:.2} Вт·ч < 80% от 18.2"
        );
        let (_, e) = discharge_to_cutoff(&p, 0.2 * 4.85, -10.0, 2.0);
        assert!(
            e >= 0.7 * 18.2,
            "энергия при −10 °C {e:.2} Вт·ч < 70% от 18.2"
        );
    }
}
