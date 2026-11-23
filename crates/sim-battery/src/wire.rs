//! Провода: сечения AWG и мм², сопротивление, допустимый ток, нагрев при коротком замыкании.
//!
//! Нагрев при КЗ — адиабатическая формула IEC 60949 / IEC 60364-5-54 (приложение A):
//! `I²·t = k²·S²`, `k = K·√ln(1 + (θf − θi)/(β + θi))`, для меди `K = 226 А·с^½/мм²`,
//! `β = 234.5 °C`. Обратная форма: `θf = (θi + β)·exp(I²t/(K²S²)) − β`.
//!
//! Для долгих токов адиабата слишком пессимистична, поэтому есть решение с охлаждением:
//! `dθ/dt = (θ + β)/τa − (θ − Ta)/τc`, `1/τa = I²/(K²S²)`, `1/τc = h·π·D/(Qc·S)`. Уравнение
//! линейно по θ и решается точно: при `τc → ∞` это формула IEC, при `t → ∞` — установившийся
//! режим. Если `1/τa ≥ 1/τc`, установившегося режима нет: сопротивление растёт быстрее
//! теплоотдачи, и провод греется до разрушения.
//!
//! Теплоотдача берётся из `sim-env` (Morgan, излучение) при малом перегреве 10 К: там `h`
//! наименьший, то есть это опасная сторона. Без заданного наружного диаметра считается по
//! голой жиле: меньше площадь — меньше охлаждение.

use std::f64::consts::PI;

use sim_env::convection::{Medium, h_horizontal_cylinder_natural, h_radiation};

/// Удельное сопротивление отожжённой меди при 20 °C, Ом·м (IACS, IEC 60028).
pub const CU_RHO20_OHM_M: f64 = 1.7241e-8;
/// Обратный температурный коэффициент меди при 0 °C, °C (IEC 60949, IEC 60364-5-54).
pub const CU_BETA_C: f64 = 234.5;
/// Объёмная теплоёмкость меди, Дж/(К·м³) (IEC 60364-5-54: 3.45·10⁻³ Дж/(К·мм³)).
pub const CU_QC_J_K_M3: f64 = 3.45e6;
/// Постоянная меди в формуле IEC, А·с^½/мм².
pub const CU_K_IEC: f64 = 226.0;
/// Температура плавления меди в формуле Ондердонка, °C.
pub const CU_MELT_C: f64 = 1083.0;
/// Круговых милов в 1 мм² (1 cmil = π/4 mil²).
pub const CMIL_PER_MM2: f64 = 1973.525;
/// Излучательная способность изоляции: оценка, нижняя граница для ПВХ, силикона и ПТФЭ.
const WIRE_EMISSIVITY: f64 = 0.8;

/// Изоляция провода.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Insulation {
    /// ПВХ, 70 °C.
    Pvc,
    /// Силиконовая резина.
    Silicone,
    /// ПТФЭ (фторопласт).
    Ptfe,
}

impl Insulation {
    /// Название.
    pub fn name(self) -> &'static str {
        match self {
            Insulation::Pvc => "ПВХ",
            Insulation::Silicone => "силикон",
            Insulation::Ptfe => "ПТФЭ",
        }
    }

    /// Предельная длительная температура жилы, °C.
    pub fn continuous_c(self) -> f64 {
        match self {
            Insulation::Pvc => 70.0,
            Insulation::Silicone => 180.0,
            // оценка: класс 200 °C луженых и серебрёных жил, общий предел ПТФЭ — 260 °C
            Insulation::Ptfe => 200.0,
        }
    }

    /// Предельная температура жилы при КЗ, °C.
    pub fn short_circuit_c(self) -> f64 {
        match self {
            Insulation::Pvc => 160.0,
            Insulation::Silicone => 350.0,
            // оценка: в таблицах IEC ПТФЭ нет; взят его верхний рабочий предел без запаса на КЗ
            Insulation::Ptfe => 260.0,
        }
    }

    /// Источник пределов.
    pub fn source(self) -> &'static str {
        match self {
            Insulation::Pvc => {
                "IEC 60364-5-54 табл. A.54: ПВХ 70 °C, конечная 160 °C (S ≤ 300 мм²)"
            }
            Insulation::Silicone => {
                "IEC 60364-5-54 табл. A.54: силикон, начальная 180 °C, конечная 350 °C"
            }
            Insulation::Ptfe => {
                "оценка: в IEC 60364-5-54 ПТФЭ нет, предел 260 °C — верх рабочего диапазона"
            }
        }
    }
}

/// Условия охлаждения провода.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WireCooling {
    /// Одиночный провод в свободном воздухе.
    FreeAir,
    /// Жгут 4–6 проводов: ток × 0.8 (NEC табл. 310.15(C)(1)), теплоотдача × 0.64.
    Bundled,
    /// В закрытом корпусе: как жгут 7–9 проводов, ток × 0.7, теплоотдача × 0.49 (оценка).
    Enclosed,
}

impl WireCooling {
    /// Множитель теплоотдачи.
    pub fn factor(self) -> f64 {
        match self {
            WireCooling::FreeAir => 1.0,
            WireCooling::Bundled => 0.64,
            WireCooling::Enclosed => 0.49,
        }
    }
}

/// Сечение жилы.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Conductor {
    /// Калибр AWG.
    Awg(u8),
    /// Сечение, мм².
    Mm2(f64),
}

/// Диаметр жилы по AWG, мм (ASTM B258): `d = 0.127·92^((36 − n)/39)`.
pub fn awg_diameter_mm(awg: u8) -> f64 {
    0.127 * 92f64.powf((36.0 - awg as f64) / 39.0)
}

/// Сечение жилы по AWG, мм².
pub fn awg_area_mm2(awg: u8) -> f64 {
    let d = awg_diameter_mm(awg);
    PI / 4.0 * d * d
}

impl Conductor {
    /// Сечение, мм².
    pub fn area_mm2(self) -> f64 {
        match self {
            Conductor::Awg(n) => awg_area_mm2(n),
            Conductor::Mm2(s) => s,
        }
    }

    /// Диаметр эквивалентной сплошной жилы, мм.
    pub fn diameter_mm(self) -> f64 {
        (4.0 * self.area_mm2() / PI).sqrt()
    }
}

/// Постоянная `k` формулы `I²t = k²S²` для перехода от θi к θf, А·с^½/мм².
pub fn adiabatic_k(theta_i_c: f64, theta_f_c: f64) -> f64 {
    CU_K_IEC
        * (1.0 + (theta_f_c - theta_i_c) / (CU_BETA_C + theta_i_c))
            .ln()
            .sqrt()
}

/// Постоянная `K` меди из физических свойств: `√(Qc·(β + 20)/ρ20)`, А·с^½/мм².
pub fn copper_k_from_constants() -> f64 {
    let qc_mm3 = CU_QC_J_K_M3 * 1e-9; // Дж/(К·мм³)
    let rho_mm = CU_RHO20_OHM_M * 1e3; // Ом·мм
    (qc_mm3 * (CU_BETA_C + 20.0) / rho_mm).sqrt()
}

/// Конечная температура жилы по адиабатической формуле, °C.
pub fn adiabatic_final_temp_c(current_a: f64, time_s: f64, area_mm2: f64, theta_i_c: f64) -> f64 {
    let x = current_a * current_a * time_s / (CU_K_IEC * CU_K_IEC * area_mm2 * area_mm2);
    (theta_i_c + CU_BETA_C) * x.exp() - CU_BETA_C
}

/// Время нагрева жилы от θi до θf при токе `current_a`, с.
pub fn adiabatic_time_s(current_a: f64, area_mm2: f64, theta_i_c: f64, theta_f_c: f64) -> f64 {
    let k = adiabatic_k(theta_i_c, theta_f_c);
    (k * area_mm2 / current_a).powi(2)
}

/// Ток плавления меди за время `time_s` по формуле Ондердонка, А:
/// `I = A·√(log₁₀(1 + (Tm − Ta)/(234 + Ta))/(33·t))`, A — в круговых милах.
pub fn onderdonk_current_a(area_mm2: f64, time_s: f64, t_ambient_c: f64) -> f64 {
    let a_cmil = area_mm2 * CMIL_PER_MM2;
    let x = 1.0 + (CU_MELT_C - t_ambient_c) / (234.0 + t_ambient_c);
    a_cmil * (x.log10() / (33.0 * time_s)).sqrt()
}

/// Проводка от батареи до нагрузки.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WireRun {
    /// Жила.
    pub conductor: Conductor,
    /// Изоляция.
    pub insulation: Insulation,
    /// Длина в один конец, м (контур тока вдвое длиннее).
    pub length_m: f64,
    /// Охлаждение.
    pub cooling: WireCooling,
    /// Наружный диаметр по изоляции, мм; `None` — считать по голой жиле (опасная сторона).
    pub outer_diameter_mm: Option<f64>,
}

impl WireRun {
    /// Сечение, мм².
    pub fn area_mm2(&self) -> f64 {
        self.conductor.area_mm2()
    }

    /// Сопротивление на метр жилы при температуре, Ом/м.
    pub fn resistance_ohm_per_m(&self, temp_c: f64) -> f64 {
        CU_RHO20_OHM_M * (CU_BETA_C + temp_c) / (CU_BETA_C + 20.0) / (self.area_mm2() * 1e-6)
    }

    /// Сопротивление участка контура длиной `loop_m`, Ом.
    pub fn loop_resistance_ohm(&self, loop_m: f64, temp_c: f64) -> f64 {
        self.resistance_ohm_per_m(temp_c) * loop_m
    }

    /// Падение напряжения на всём контуре при токе, В.
    pub fn voltage_drop_v(&self, current_a: f64, temp_c: f64) -> f64 {
        current_a * self.loop_resistance_ohm(2.0 * self.length_m, temp_c)
    }

    fn outer_diameter_m(&self) -> f64 {
        self.outer_diameter_mm
            .unwrap_or_else(|| self.conductor.diameter_mm())
            * 1e-3
    }

    /// Теплоотдача с метра длины на кельвин, Вт/(м·К).
    pub fn cooling_w_per_m_k(&self, t_ambient_c: f64) -> f64 {
        let d = self.outer_diameter_m();
        let ts = t_ambient_c + 10.0;
        let h = h_horizontal_cylinder_natural(&Medium::air(50.0), d, ts, t_ambient_c)
            + h_radiation(WIRE_EMISSIVITY, ts, t_ambient_c);
        h * PI * d * self.cooling.factor()
    }

    /// Допустимый длительный ток, А: `I²·R'(θ) = h·π·D·(θ − Ta)` при θ = пределу изоляции.
    pub fn ampacity_a(&self, t_ambient_c: f64) -> f64 {
        let theta = self.insulation.continuous_c();
        let dt = theta - t_ambient_c;
        if dt <= 0.0 {
            return 0.0;
        }
        (self.cooling_w_per_m_k(t_ambient_c) * dt / self.resistance_ohm_per_m(theta)).sqrt()
    }

    /// Установившаяся температура жилы при токе, °C. `None` — равновесия нет.
    pub fn steady_temp_c(&self, current_a: f64, t_ambient_c: f64) -> Option<f64> {
        let (a, c) = self.rates(current_a, t_ambient_c);
        if a >= c {
            return None;
        }
        Some((a * CU_BETA_C + c * t_ambient_c) / (c - a))
    }

    /// Скорости `1/τa` и `1/τc`, 1/с.
    fn rates(&self, current_a: f64, t_ambient_c: f64) -> (f64, f64) {
        let s = self.area_mm2();
        let a = current_a * current_a / (CU_K_IEC * CU_K_IEC * s * s);
        let c = self.cooling_w_per_m_k(t_ambient_c) / (CU_QC_J_K_M3 * s * 1e-6);
        (a, c)
    }

    /// Температура жилы через `time_s` при токе `current_a` с учётом охлаждения, °C.
    /// Точное решение; результат ограничен температурой плавления меди.
    pub fn temperature_after_c(
        &self,
        current_a: f64,
        time_s: f64,
        theta0_c: f64,
        t_ambient_c: f64,
    ) -> f64 {
        let (a, c) = self.rates(current_a, t_ambient_c);
        let lambda = a - c;
        let b = a * CU_BETA_C + c * t_ambient_c;
        let theta = if lambda.abs() < 1e-12 {
            theta0_c + b * time_s
        } else {
            (theta0_c + b / lambda) * (lambda * time_s).exp() - b / lambda
        };
        if theta.is_finite() {
            theta.min(CU_MELT_C)
        } else {
            CU_MELT_C
        }
    }

    /// Наибольшая температура жилы за профиль нагрузки `(ток, с)`, повторённый `cycles` раз.
    pub fn profile_max_temp_c(&self, steps: &[(f64, f64)], cycles: u32, t_ambient_c: f64) -> f64 {
        let mut theta = t_ambient_c;
        let mut worst = theta;
        for _ in 0..cycles.clamp(1, 100_000) {
            let start = theta;
            for &(i, d) in steps {
                theta = self.temperature_after_c(i, d, theta, t_ambient_c);
                if theta > worst {
                    worst = theta;
                }
            }
            if (theta - start).abs() < 1e-6 {
                break;
            }
        }
        worst
    }

    /// Индуктивность контура из пары проводов, Гн: `L = μ₀/π·l·(ln(2a/d) + ¼)`.
    /// Зазор между проводами `a` = 10 мм — оценка для незавитой пары.
    pub fn loop_inductance_h(&self, one_way_m: f64) -> f64 {
        const MU0: f64 = 1.256_637_06e-6;
        const SPACING_M: f64 = 0.010;
        let d = self.conductor.diameter_mm() * 1e-3;
        MU0 / PI * one_way_m * ((2.0 * SPACING_M / d).ln().max(0.0) + 0.25)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wire(awg: u8) -> WireRun {
        WireRun {
            conductor: Conductor::Awg(awg),
            insulation: Insulation::Silicone,
            length_m: 1.0,
            cooling: WireCooling::FreeAir,
            outer_diameter_mm: None,
        }
    }

    #[test]
    fn adiabatic_forward_and_inverse_agree() {
        let t = adiabatic_time_s(500.0, 2.08, 70.0, 160.0);
        let theta = adiabatic_final_temp_c(500.0, t, 2.08, 70.0);
        // формула k округляет K до 226 в обе стороны: совпадение точное
        assert!((theta - 160.0).abs() < 1e-9, "{theta}");
    }

    #[test]
    fn cooling_solution_tends_to_adiabatic_for_short_pulse() {
        let w = wire(14);
        let cooled = w.temperature_after_c(800.0, 0.01, 40.0, 25.0);
        let adiabatic = adiabatic_final_temp_c(800.0, 0.01, w.area_mm2(), 40.0);
        assert!((cooled - adiabatic).abs() < 0.05, "{cooled} {adiabatic}");
    }

    #[test]
    fn cooling_solution_tends_to_steady_state() {
        let w = wire(14);
        let steady = w.steady_temp_c(15.0, 25.0).expect("равновесие есть");
        let late = w.temperature_after_c(15.0, 1e5, 25.0, 25.0);
        assert!((late - steady).abs() < 1e-6, "{late} {steady}");
    }

    #[test]
    fn huge_current_has_no_steady_state_and_caps_at_melting() {
        let w = wire(30);
        assert_eq!(w.steady_temp_c(500.0, 25.0), None);
        assert_eq!(w.temperature_after_c(500.0, 100.0, 25.0, 25.0), CU_MELT_C);
    }

    #[test]
    fn thinner_wire_carries_less() {
        let mut last = f64::INFINITY;
        for awg in [10, 12, 14, 16, 18, 20, 22, 24] {
            let i = wire(awg).ampacity_a(25.0);
            assert!(i > 0.0 && i < last, "AWG {awg}: {i}");
            last = i;
        }
    }

    #[test]
    fn enclosed_wire_carries_less_than_free() {
        let mut w = wire(16);
        let free = w.ampacity_a(25.0);
        w.cooling = WireCooling::Enclosed;
        assert!((w.ampacity_a(25.0) / free - 0.7).abs() < 1e-9);
    }
}
