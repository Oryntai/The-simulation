//! Диэлектрические свойства сред на радиочастотах: вода (пресная и морская), ткани тела.
//! Из комплексной проницаемости ε = ε' − j·ε'' получаем затухание плоской волны и потери
//! на границе сред — так считаем, почему Bluetooth не проходит сквозь воду и тело.
//!
//! Две независимые модели воды (второй способ проверки):
//! * ITU-R P.840 (двойной Дебай по Liebe и др. 1991) — пресная вода;
//! * Klein & Swift 1977 (одиночный Дебай + солёность + ионная проводимость) —
//!   IEEE Trans. Antennas Propag. 25(1):104–111, doi:10.1109/TAP.1977.1141539.

use crate::{C, EPS0};
use std::f64::consts::PI;

/// Относительная комплексная проницаемость ε = re − j·im (im ≥ 0 — потери).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Permittivity {
    pub re: f64,
    pub im: f64,
}

impl Permittivity {
    pub const VACUUM: Self = Self { re: 1.0, im: 0.0 };

    /// Из ε' и полной проводимости σ (См/м) на частоте `f_hz`.
    pub fn from_conductivity(eps_r: f64, sigma: f64, f_hz: f64) -> Self {
        Self {
            re: eps_r,
            im: sigma / (2.0 * PI * f_hz * EPS0),
        }
    }

    /// Эквивалентная проводимость (все потери), См/м.
    pub fn conductivity(&self, f_hz: f64) -> f64 {
        self.im * 2.0 * PI * f_hz * EPS0
    }

    pub fn loss_tangent(&self) -> f64 {
        self.im / self.re
    }

    /// Коэффициент затухания поля α, Нп/м: α = (ω/c)·√(ε'/2·(√(1+tg²δ) − 1)), μ = 1.
    pub fn attenuation_np_per_m(&self, f_hz: f64) -> f64 {
        let w = 2.0 * PI * f_hz;
        let t = self.loss_tangent();
        w / C * (self.re / 2.0 * ((1.0 + t * t).sqrt() - 1.0)).sqrt()
    }

    /// Затухание мощности, дБ/м (20·log10(e) ≈ 8.686 дБ на непер поля).
    pub fn attenuation_db_per_m(&self, f_hz: f64) -> f64 {
        20.0 * std::f64::consts::LOG10_E * self.attenuation_np_per_m(f_hz)
    }

    /// Глубина проникновения: поле падает в e раз (мощность — в e² ≈ 7.4 раза), м.
    pub fn penetration_depth_m(&self, f_hz: f64) -> f64 {
        1.0 / self.attenuation_np_per_m(f_hz)
    }

    /// Волновое сопротивление относительно вакуума: η/η0 = 1/√ε (комплексное, (re, im)).
    fn rel_impedance(&self) -> (f64, f64) {
        // √(re − j·im) в главной ветви.
        let (a, b) = (self.re, -self.im);
        let r = (a * a + b * b).sqrt();
        let sr = ((r + a) / 2.0).sqrt();
        let si = ((r - a) / 2.0).sqrt() * b.signum();
        // 1/(sr + j·si)
        let d = sr * sr + si * si;
        (sr / d, -si / d)
    }
}

/// Потери на прохождение границы при нормальном падении, дБ (≥ 0): 1 − |Γ|²,
/// Γ = (η2 − η1)/(η2 + η1). Для границы воздух → вода ≈ 2.5–3.5 дБ.
pub fn interface_loss_db(from: Permittivity, to: Permittivity) -> f64 {
    let (r1, i1) = from.rel_impedance();
    let (r2, i2) = to.rel_impedance();
    let (nr, ni) = (r2 - r1, i2 - i1);
    let (dr, di) = (r2 + r1, i2 + i1);
    let g2 = (nr * nr + ni * ni) / (dr * dr + di * di);
    -10.0 * (1.0 - g2).log10()
}

/// Пресная вода по ITU-R P.840 (двойной Дебай, модель Liebe 1991), t в °C.
/// ITU-R P.840-8, раздел 2 (формулы для ε' и ε'' жидкой воды).
pub fn water_itu_p840(t_c: f64, f_hz: f64) -> Permittivity {
    let theta = 300.0 / (t_c + 273.15);
    let e0 = 77.66 + 103.3 * (theta - 1.0);
    let e1 = 5.48;
    let e2 = 3.51;
    let fp = 20.09 - 142.0 * (theta - 1.0) + 294.0 * (theta - 1.0).powi(2); // ГГц
    let fs = 590.0 - 1500.0 * (theta - 1.0); // ГГц
    let f = f_hz * 1e-9;
    let re = (e0 - e1) / (1.0 + (f / fp).powi(2)) + (e1 - e2) / (1.0 + (f / fs).powi(2)) + e2;
    let im = f * (e0 - e1) / (fp * (1.0 + (f / fp).powi(2)))
        + f * (e1 - e2) / (fs * (1.0 + (f / fs).powi(2)));
    Permittivity { re, im }
}

/// Проводимость морской воды по Klein & Swift 1977, См/м (t в °C, солёность в ‰/psu).
pub fn seawater_conductivity_klein_swift(t_c: f64, s: f64) -> f64 {
    let s25 = s * (0.18252 - 1.4619e-3 * s + 2.093e-5 * s * s - 1.282e-7 * s.powi(3));
    let d = 25.0 - t_c;
    let beta = 2.033e-2 + 1.266e-4 * d + 2.464e-6 * d * d
        - s * (1.849e-5 - 2.551e-7 * d + 2.551e-8 * d * d);
    s25 * (-d * beta).exp()
}

/// Вода по Klein & Swift 1977: одиночный Дебай с поправками на солёность `s` (‰) и ионной
/// проводимостью. `extra_sigma` — добавочная проводимость пресной воды (водопровод/озеро
/// ~0.005–0.05 См/м; на 2.4 ГГц почти не влияет), См/м.
pub fn water_klein_swift(t_c: f64, s: f64, extra_sigma: f64, f_hz: f64) -> Permittivity {
    let t = t_c;
    let eps_s0 = 87.134 - 1.949e-1 * t - 1.276e-2 * t * t + 2.491e-4 * t.powi(3);
    let a = 1.0 + 1.613e-5 * t * s - 3.656e-3 * s + 3.210e-5 * s * s - 4.232e-7 * s.powi(3);
    let tau0 = 1.768e-11 - 6.086e-13 * t + 1.104e-14 * t * t - 8.111e-17 * t.powi(3);
    let b = 1.0 + 2.282e-5 * t * s - 7.638e-4 * s - 7.760e-6 * s * s + 1.105e-8 * s.powi(3);
    let eps_s = eps_s0 * a;
    let tau = tau0 * b;
    let eps_inf = 4.9;
    let wt = 2.0 * PI * f_hz * tau;
    let sigma = if s > 0.0 {
        seawater_conductivity_klein_swift(t_c, s)
    } else {
        0.0
    } + extra_sigma;
    let re = eps_inf + (eps_s - eps_inf) / (1.0 + wt * wt);
    let im = (eps_s - eps_inf) * wt / (1.0 + wt * wt) + sigma / (2.0 * PI * f_hz * EPS0);
    Permittivity { re, im }
}

/// Солёность океанской воды по умолчанию, ‰.
pub const OCEAN_SALINITY: f64 = 35.0;

/// Среда вокруг устройства.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Medium {
    Air,
    /// Пресная вода (t °C).
    FreshWater {
        t_c: f64,
    },
    /// Морская вода 35 ‰ (t °C).
    SeaWater {
        t_c: f64,
    },
}

impl Medium {
    pub fn permittivity(&self, f_hz: f64) -> Permittivity {
        match *self {
            Medium::Air => Permittivity::VACUUM,
            Medium::FreshWater { t_c } => water_klein_swift(t_c, 0.0, 0.01, f_hz),
            Medium::SeaWater { t_c } => water_klein_swift(t_c, OCEAN_SALINITY, 0.0, f_hz),
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Medium::Air => "воздух",
            Medium::FreshWater { .. } => "пресная вода",
            Medium::SeaWater { .. } => "морская вода",
        }
    }
}

/// Ткани тела на 2.45 ГГц: (ε', σ См/м) по базе IT'IS v4.1 (Hasgall и др., на основе
/// параметрической модели Gabriel 1996, doi:10.1088/0031-9155/41/11/003).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tissue {
    Muscle,
    SkinDry,
    Fat,
    Blood,
    BoneCortical,
}

impl Tissue {
    pub const ALL: [Tissue; 5] = [
        Tissue::SkinDry,
        Tissue::Fat,
        Tissue::Muscle,
        Tissue::Blood,
        Tissue::BoneCortical,
    ];

    /// (ε', σ) на 2.45 ГГц.
    pub fn at_2g45(&self) -> (f64, f64) {
        match self {
            Tissue::Muscle => (52.7, 1.74),
            Tissue::SkinDry => (38.0, 1.46),
            Tissue::Fat => (5.28, 0.105),
            Tissue::Blood => (58.3, 2.54),
            Tissue::BoneCortical => (11.4, 0.394),
        }
    }

    pub fn permittivity_2g45(&self) -> Permittivity {
        let (e, s) = self.at_2g45();
        Permittivity::from_conductivity(e, s, 2.45e9)
    }

    pub fn name(&self) -> &'static str {
        match self {
            Tissue::Muscle => "мышца",
            Tissue::SkinDry => "кожа",
            Tissue::Fat => "жир",
            Tissue::Blood => "кровь",
            Tissue::BoneCortical => "кость",
        }
    }
}

/// Мышечная ткань на 2450 МГц по Johnson & Guy 1972 (Proc. IEEE 60(6):692,
/// doi:10.1109/PROC.1972.8728): ε' = 47, σ = 2.21 См/м, глубина проникновения 1.70 см.
/// Используется как независимая сверка формулы затухания.
pub const JOHNSON_GUY_MUSCLE_2450: (f64, f64, f64) = (47.0, 2.21, 0.0170);
