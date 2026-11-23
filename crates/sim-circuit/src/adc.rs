//! Вход АЦП с выборкой-хранением: делитель и допустимое сопротивление источника.
//!
//! За время выборки `T_s` конденсатор выборки `C_s` заряжается через сопротивление
//! источника `R_src` и ключа `R_adc`; чтобы ошибка не превысила ¼ младшего разряда N-битного
//! АЦП, нужно e^(−T_s/((R_src+R_adc)·C_s)) ≤ 2^−(N+2), откуда
//! `R_src < T_s / (C_s · ln 2^(N+2)) − R_adc`.
//! Источник формулы: STMicroelectronics AN2834 «How to get the best ADC accuracy in STM32
//! microcontrollers» (раздел о сопротивлении источника аналогового входа),
//! <https://www.st.com/resource/en/application_note/an2834-how-to-get-the-best-adc-accuracy-in-stm32-microcontrollers-stmicroelectronics.pdf>;
//! та же формула приводится в даташитах STM32 (R_AIN max).
//! Внешний конденсатор у входа АЦП (≫ C_s) снимает ограничение для медленных сигналов: он
//! отдаёт заряд выборки, а делитель лишь подзаряжает его между выборками.

/// Делитель R1 (сверху) / R2 (снизу): (коэффициент передачи, сопротивление Тевенина, Ом).
pub fn divider_thevenin(r1: f64, r2: f64) -> (f64, f64) {
    (r2 / (r1 + r2), r1 * r2 / (r1 + r2))
}

/// Наибольшее сопротивление источника, Ом, при котором ошибка выборки ≤ ¼ LSB.
pub fn max_source_resistance(t_sample: f64, c_sample: f64, r_switch: f64, bits: u32) -> f64 {
    t_sample / (c_sample * (bits as f64 + 2.0) * std::f64::consts::LN_2) - r_switch
}

/// Относительная ошибка недозаряда конденсатора выборки за время выборки.
pub fn sampling_error(t_sample: f64, c_sample: f64, r_total: f64) -> f64 {
    (-t_sample / (r_total * c_sample)).exp()
}

/// Итог проверки входа АЦП.
#[derive(Clone, Debug, PartialEq)]
pub struct AdcInputCheck {
    pub ratio: f64,
    pub r_source: f64,
    pub r_max: f64,
    /// Есть ли внешний конденсатор, который снимает ограничение.
    pub has_buffer_cap: bool,
    pub ok: bool,
    pub message: String,
}

/// Проверить делитель на входе АЦП. `c_ext` — конденсатор у входа (0 — нет).
pub fn check_divider_input(
    r1: f64,
    r2: f64,
    c_ext: f64,
    t_sample: f64,
    c_sample: f64,
    r_switch: f64,
    bits: u32,
) -> AdcInputCheck {
    let (ratio, r_source) = divider_thevenin(r1, r2);
    let r_max = max_source_resistance(t_sample, c_sample, r_switch, bits);
    // Буферный конденсатор: падение при выборке ≈ V·C_s/C_ext должно быть < ¼ LSB.
    let has_buffer_cap = c_ext > c_sample * 2f64.powi(bits as i32 + 2);
    let ok = r_source <= r_max || has_buffer_cap;
    let message = if r_source <= r_max {
        format!(
            "сопротивление источника {:.0} Ом ≤ допустимых {:.0} Ом",
            r_source, r_max
        )
    } else if has_buffer_cap {
        format!(
            "сопротивление источника {:.0} Ом > {:.0} Ом, но конденсатор {:.0} нФ у входа \
             отдаёт заряд выборки (годится для медленных сигналов: τ = {:.2} мс)",
            r_source,
            r_max,
            c_ext * 1e9,
            r_source * c_ext * 1e3
        )
    } else {
        format!(
            "сопротивление источника {:.0} Ом > допустимых {:.0} Ом: увеличьте время выборки, \
             уменьшите делитель или поставьте конденсатор у входа АЦП",
            r_source, r_max
        )
    };
    AdcInputCheck {
        ratio,
        r_source,
        r_max,
        has_buffer_cap,
        ok,
        message,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn divider_and_limit() {
        let (k, r) = divider_thevenin(100e3, 27e3);
        assert!((k - 27.0 / 127.0).abs() < 1e-15);
        assert!((r - 100e3 * 27e3 / 127e3).abs() < 1e-9);
        let rmax = max_source_resistance(1e-6, 5e-12, 1e3, 12);
        // На пределе ошибка ровно ¼ LSB.
        let err = sampling_error(1e-6, 5e-12, rmax + 1e3);
        assert!((err - 2f64.powi(-14)).abs() < 1e-12);
        let c = check_divider_input(100e3, 27e3, 0.0, 1e-6, 5e-12, 1e3, 12);
        assert!(!c.ok);
        let c = check_divider_input(100e3, 27e3, 100e-9, 1e-6, 5e-12, 1e3, 12);
        assert!(c.ok && c.has_buffer_cap);
    }
}
