//! Пресная и морская вода: плотность, вязкость, теплопроводность, теплоёмкость, Pr,
//! коэффициент объёмного расширения, электропроводность, точка замерзания.
//!
//! Основные формулы — корреляции обзора Sharqawy, Lienhard, Zubair (2010), справедливые
//! для 0–180 °C (вязкость, плотность) и солёности 0–150 г/кг; при S = 0 они дают пресную воду
//! (подогнаны к IAPWS). Второй способ:
//! * плотность морской воды — уравнение состояния ЮНЕСКО EOS-80 при p = 0
//!   (Millero & Poisson 1981) с опубликованными контрольными значениями;
//! * свойства пресной воды — табличные значения IAPWS-95/IAPWS-2008/IAPWS-2011 при 0.1 МПа.
//!
//! Допущения: давление атмосферное (погружение на глубину до десятков метров меняет
//! плотность < 0.5 %); солёность Sharqawy — «эталонная» (г/кг), практическая солёность
//! PSS-78 меньше на 0.47 % — разница в плотности ~0.01 %, ею пренебрегаем.
//!
//! Источник: Sharqawy M.H., Lienhard J.H., Zubair S.M. (2010) Desalination and Water
//! Treatment 16:354–380, doi:10.5004/dwt.2010.1079.

/// Солёность стандартной морской воды, г/кг.
pub const SEAWATER_SALINITY: f64 = 35.0;

/// Плотность чистой воды (Sharqawy 2010, ур. 8, S = 0), кг/м³. Точность 0.01 %, 0–180 °C.
fn rho_pure_sharqawy(t: f64) -> f64 {
    9.999e2 + 2.034e-2 * t - 6.162e-3 * t * t + 2.261e-5 * t.powi(3) - 4.657e-8 * t.powi(4)
}

/// Плотность морской воды, кг/м³ (Sharqawy 2010, ур. 8; S в г/кг). Точность ±0.1 %,
/// 0–180 °C, 0–160 г/кг.
pub fn density(t_c: f64, salinity_g_kg: f64) -> f64 {
    let s = salinity_g_kg / 1000.0;
    let t = t_c;
    rho_pure_sharqawy(t)
        + s * (8.020e2 - 2.001 * t + 1.677e-2 * t * t - 3.060e-5 * t.powi(3) - 1.613e-5 * s * t * t)
}

/// Плотность морской воды по уравнению ЮНЕСКО EOS-80 при атмосферном давлении
/// (Millero & Poisson 1981), кг/м³; S — практическая солёность (PSS-78), t — °C.
/// Применимость: 0–40 °C, S 0.5–43. Контрольные значения ЮНЕСКО:
/// ρ(0, 5) = 999.966 75, ρ(35, 5) = 1027.675 47, ρ(35, 25) = 1023.343 06.
///
/// Источники: Millero F.J., Poisson A. (1981) Deep-Sea Res. 28A:625–629,
/// doi:10.1016/0198-0149(81)90122-9; UNESCO Tech. Pap. Mar. Sci. 36 (1981);
/// Fofonoff N.P., Millard R.C. (1983) UNESCO Tech. Pap. Mar. Sci. 44.
pub fn density_eos80(t_c: f64, salinity_psu: f64) -> f64 {
    let t = t_c;
    let s = salinity_psu;
    let rho_w = 999.842_594 + 6.793_952e-2 * t - 9.095_290e-3 * t * t + 1.001_685e-4 * t.powi(3)
        - 1.120_083e-6 * t.powi(4)
        + 6.536_332e-9 * t.powi(5);
    let a = 8.244_93e-1 - 4.0899e-3 * t + 7.6438e-5 * t * t - 8.2467e-7 * t.powi(3)
        + 5.3875e-9 * t.powi(4);
    let b = -5.724_66e-3 + 1.0227e-4 * t - 1.6546e-6 * t * t;
    let c = 4.8314e-4;
    rho_w + a * s + b * s.powf(1.5) + c * s * s
}

/// Динамическая вязкость, Па·с (Sharqawy 2010, ур. 22–23). Чистая вода — подгонка к
/// IAPWS 2008 (±0.05 %), поправка на солёность ±0.4 %; 0–180 °C, 0–150 г/кг.
pub fn viscosity(t_c: f64, salinity_g_kg: f64) -> f64 {
    let t = t_c;
    let s = salinity_g_kg / 1000.0;
    let mu_w = 4.2844e-5 + 1.0 / (0.157 * (t + 64.993).powi(2) - 91.296);
    let a = 1.541 + 1.998e-2 * t - 9.52e-5 * t * t;
    let b = 7.974 - 7.561e-2 * t + 4.724e-4 * t * t;
    mu_w * (1.0 + a * s + b * s * s)
}

/// Теплопроводность, Вт/(м·К) (Sharqawy 2010, ур. 13, по Jamieson & Tudhope 1970).
/// Точность ±3 %; 0–180 °C, 0–160 г/кг.
pub fn conductivity(t_c: f64, salinity_g_kg: f64) -> f64 {
    let s = salinity_g_kg;
    let t = t_c + 273.15;
    let log_k = (240.0 + 0.0002 * s).log10()
        + 0.434 * (2.3 - (343.5 + 0.037 * s) / t) * (1.0 - t / (647.0 + 0.03 * s)).powf(0.333);
    10f64.powf(log_k) * 1e-3
}

/// Удельная теплоёмкость, Дж/(кг·К) (Sharqawy 2010, ур. 9, по Jamieson et al. 1969).
/// Точность ±0.28 %; 0–180 °C, 0–180 г/кг.
pub fn heat_capacity(t_c: f64, salinity_g_kg: f64) -> f64 {
    let s = salinity_g_kg;
    let t = t_c + 273.15;
    let a = 5.328 - 9.76e-2 * s + 4.04e-4 * s * s;
    let b = -6.913e-3 + 7.351e-4 * s - 3.15e-6 * s * s;
    let c = 9.6e-6 - 1.927e-6 * s + 8.23e-9 * s * s;
    let d = 2.5e-9 + 1.666e-9 * s - 7.125e-12 * s * s;
    (a + b * t + c * t * t + d * t.powi(3)) * 1e3
}

/// Число Прандтля.
pub fn prandtl(t_c: f64, salinity_g_kg: f64) -> f64 {
    viscosity(t_c, salinity_g_kg) * heat_capacity(t_c, salinity_g_kg)
        / conductivity(t_c, salinity_g_kg)
}

/// Коэффициент объёмного расширения β = −(1/ρ)·∂ρ/∂T, 1/К (центральная разность по
/// [`density`]). У пресной воды β < 0 ниже ≈ 4 °C (аномалия плотности).
pub fn expansion_coefficient(t_c: f64, salinity_g_kg: f64) -> f64 {
    let h = 0.01;
    let rho = density(t_c, salinity_g_kg);
    -(density(t_c + h, salinity_g_kg) - density(t_c - h, salinity_g_kg)) / (2.0 * h) / rho
}

/// Температура замерзания морской воды, °C, при избыточном давлении `p_dbar` (дбар; 0 — у
/// поверхности): t_f = −0.0575·S + 1.710523·10⁻³·S^1.5 − 2.154996·10⁻⁴·S² − 7.53·10⁻⁴·p.
/// Контрольное значение ЮНЕСКО: t_f(S = 40, p = 500 дбар) = −2.588567 °C.
/// Источник: Fofonoff N.P., Millard R.C. (1983) UNESCO Tech. Pap. Mar. Sci. 44.
pub fn freezing_point_at(salinity_psu: f64, p_dbar: f64) -> f64 {
    let s = salinity_psu;
    -0.0575 * s + 1.710_523e-3 * s.powf(1.5) - 2.154_996e-4 * s * s - 7.53e-4 * p_dbar
}

/// Температура замерзания морской воды у поверхности, °C (S = 35 → −1.92 °C).
pub fn freezing_point(salinity_psu: f64) -> f64 {
    freezing_point_at(salinity_psu, 0.0)
}

/// Свойства воды в одной точке.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WaterProps {
    /// Плотность, кг/м³.
    pub rho: f64,
    /// Динамическая вязкость, Па·с.
    pub mu: f64,
    /// Теплопроводность, Вт/(м·К).
    pub k: f64,
    /// Удельная теплоёмкость, Дж/(кг·К).
    pub cp: f64,
    /// Число Прандтля.
    pub pr: f64,
    /// Коэффициент объёмного расширения, 1/К.
    pub beta: f64,
}

/// Свойства воды солёностью `salinity_g_kg` (0 — пресная) при `t_c`.
pub fn water_props(t_c: f64, salinity_g_kg: f64) -> WaterProps {
    let mu = viscosity(t_c, salinity_g_kg);
    let k = conductivity(t_c, salinity_g_kg);
    let cp = heat_capacity(t_c, salinity_g_kg);
    WaterProps {
        rho: density(t_c, salinity_g_kg),
        mu,
        k,
        cp,
        pr: mu * cp / k,
        beta: expansion_coefficient(t_c, salinity_g_kg),
    }
}

// ---------------------------------------------------------------------------
// Электропроводность
// ---------------------------------------------------------------------------

/// Электропроводность стандартной морской воды S = 35 при 15 °C (определение шкалы PSS-78:
/// отношение к раствору KCl 32.4356 г/кг равно 1), См/м.
pub const SEAWATER_C35_15: f64 = 4.2914;

/// Отношение r_t = C(35, t)/C(35, 15) по PSS-78 (Lewis & Perkin 1981; UNESCO 1981).
pub fn pss78_rt(t_c: f64) -> f64 {
    let t = t_c;
    0.676_609_7 + 2.005_64e-2 * t + 1.104_259e-4 * t * t - 6.9698e-7 * t.powi(3)
        + 1.0031e-9 * t.powi(4)
}

/// Практическая солёность по PSS-78 из отношения R_t = C(S, t)/C(35, t) при p = 0.
/// Применимость 2 < S < 42, −2…35 °C.
pub fn pss78_salinity(rt: f64, t_c: f64) -> f64 {
    const A: [f64; 6] = [0.0080, -0.1692, 25.3851, 14.0941, -7.0261, 2.7081];
    const B: [f64; 6] = [0.0005, -0.0056, -0.0066, -0.0375, 0.0636, -0.0144];
    const K: f64 = 0.0162;
    let r = rt.max(0.0).sqrt();
    let mut s = 0.0;
    let mut ds = 0.0;
    let mut rp = 1.0;
    for i in 0..6 {
        s += A[i] * rp;
        ds += B[i] * rp;
        rp *= r;
    }
    let dt = t_c - 15.0;
    s + dt / (1.0 + K * dt) * ds
}

/// Электропроводность морской воды, См/м, при солёности `salinity_psu` и `t_c`
/// (обращение PSS-78 по Ньютону/бисекции). Точность самой шкалы ~0.002 по солёности.
///
/// Источник: UNESCO (1981) Tech. Pap. Mar. Sci. 37; Lewis E.L., Perkin R.G. (1981)
/// Deep-Sea Res. 28A:307–328, doi:10.1016/0198-0149(81)90002-9.
pub fn seawater_electrical_conductivity(t_c: f64, salinity_psu: f64) -> f64 {
    let rt = crate::air::bisect(|r| pss78_salinity(r, t_c) - salinity_psu, 1e-4, 2.0, 1e-12);
    rt * pss78_rt(t_c) * SEAWATER_C35_15
}

/// Электропроводность чистой (сверхчистой) воды при 25 °C, См/м (собственная диссоциация):
/// 5.5·10⁻⁶ См/м (0.055 мкСм/см). Источник: Light T.S. et al. (2005) Electrochem.
/// Solid-State Lett. 8:E16, doi:10.1149/1.1836121.
pub const PURE_WATER_CONDUCTIVITY: f64 = 5.5e-6;

/// Типичный диапазон электропроводности пресной природной и водопроводной воды, См/м:
/// ≈ 50–1500 мкСм/см = 0.005–0.15 См/м («ориентир»: USGS Water Science School,
/// «Specific Conductance»; https://www.usgs.gov/special-topics/water-science-school).
/// Морская вода (~5 См/м) проводит в 30–1000 раз лучше — главный фактор для утечек
/// тока и электрокоррозии экзоскелета в море.
pub const FRESH_WATER_CONDUCTIVITY_RANGE: (f64, f64) = (0.005, 0.15);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eos80_check_values() {
        assert!((density_eos80(5.0, 0.0) - 999.966_75).abs() < 1e-5);
        assert!((density_eos80(5.0, 35.0) - 1_027.675_47).abs() < 1e-5);
        assert!((density_eos80(25.0, 35.0) - 1_023.343_06).abs() < 1e-5);
    }

    #[test]
    fn pss78_definition() {
        assert!((pss78_rt(15.0) - 1.0).abs() < 2e-5);
        assert!((pss78_salinity(1.0, 15.0) - 35.0).abs() < 1e-4);
        let c = seawater_electrical_conductivity(15.0, 35.0);
        assert!((c - SEAWATER_C35_15).abs() < 1e-3);
    }

    #[test]
    fn seawater_freezes_below_zero() {
        assert!((freezing_point(35.0) + 1.922).abs() < 0.01);
    }
}
