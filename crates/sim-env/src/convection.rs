//! Конвективный и лучистый теплообмен: цилиндр в поперечном потоке, пластина, свободная
//! конвекция горизонтального и вертикального цилиндра; в воздухе, пресной и морской воде.
//!
//! Применение: предплечье (цилиндр ≈ 8 см), корпус мотора экзоскелета, батарейный блок
//! на ветру или в воде.
//!
//! Допущения:
//! * свойства среды берутся при плёночной температуре (T_s + T_∞)/2 (кроме Жукаускаса —
//!   там свойства при T_∞ и поправка (Pr/Pr_s)^¼, как требует корреляция);
//! * поток однородный, без турбулентности набегающего потока (на улице порывы повышают h
//!   на 10–30 % — не учтено);
//! * смешанная конвекция — по Черчиллю: Nu³ = Nu_вын³ + Nu_своб³ (поперечное
//!   и попутное направления, Incropera §9.9);
//! * корреляции эмпирические, разброс между ними ±20 % (Incropera §7.4) — это и есть
//!   честная погрешность h.
//!
//! Источники корреляций:
//! * Churchill S.W., Bernstein M. (1977) J. Heat Transfer 99:300–306, doi:10.1115/1.3450685;
//! * Hilpert R. (1933) Forsch. Ing.-Wes. 4:215; таблица C, m — Incropera et al.,
//!   Fundamentals of Heat and Mass Transfer, 6th ed., табл. 7.2 (с поправкой Pr^⅓);
//! * Zukauskas A. (1972) Adv. Heat Transfer 8:93–160, doi:10.1016/S0065-2717(08)70038-8;
//! * Churchill S.W., Chu H.H.S. (1975) Int. J. Heat Mass Transfer 18:1049–1053 (горизонтальный
//!   цилиндр, doi:10.1016/0017-9310(75)90222-7) и 18:1323–1329 (вертикальная пластина,
//!   doi:10.1016/0017-9310(75)90243-4);
//! * Morgan V.T. (1975) Adv. Heat Transfer 11:199–264, doi:10.1016/S0065-2717(08)70075-3;
//! * Le Fevre E.J., Ede A.J. (1956) Proc. 9th Int. Congr. Appl. Mech. 4:175 (вертикальный цилиндр);
//! * Churchill S.W., Ozoe H. (1973) J. Heat Transfer 95:416, doi:10.1115/1.3450078 (пластина).

use crate::air;
use crate::water;

/// Ускорение свободного падения, м/с².
pub const G: f64 = 9.806_65;
/// Постоянная Стефана–Больцмана, Вт/(м²·К⁴).
pub const STEFAN_BOLTZMANN: f64 = 5.670_374_419e-8;

/// Среда, омывающая тело.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Medium {
    /// Влажный воздух: относительная влажность, %, и давление, Па.
    Air { rh_percent: f64, p_pa: f64 },
    /// Пресная вода.
    FreshWater,
    /// Морская вода с солёностью, г/кг.
    SeaWater { salinity_g_kg: f64 },
}

impl Medium {
    /// Воздух при нормальном давлении и заданной влажности.
    pub fn air(rh_percent: f64) -> Self {
        Medium::Air {
            rh_percent,
            p_pa: air::P_STANDARD,
        }
    }

    /// Стандартная морская вода, S = 35 г/кг.
    pub fn sea() -> Self {
        Medium::SeaWater {
            salinity_g_kg: water::SEAWATER_SALINITY,
        }
    }

    /// Название среды.
    pub fn name(&self) -> &'static str {
        match self {
            Medium::Air { .. } => "воздух",
            Medium::FreshWater => "пресная вода",
            Medium::SeaWater { .. } => "морская вода",
        }
    }

    /// Свойства среды при температуре `t_c`.
    pub fn props(&self, t_c: f64) -> FluidProps {
        match *self {
            Medium::Air { rh_percent, p_pa } => {
                let a = air::air_props(t_c, rh_percent, p_pa);
                FluidProps {
                    rho: a.rho,
                    mu: a.mu,
                    k: a.k,
                    cp: a.cp,
                    pr: a.pr,
                    beta: 1.0 / (t_c + air::T0),
                }
            }
            Medium::FreshWater => from_water(water::water_props(t_c, 0.0)),
            Medium::SeaWater { salinity_g_kg } => {
                from_water(water::water_props(t_c, salinity_g_kg))
            }
        }
    }
}

fn from_water(w: water::WaterProps) -> FluidProps {
    FluidProps {
        rho: w.rho,
        mu: w.mu,
        k: w.k,
        cp: w.cp,
        pr: w.pr,
        beta: w.beta,
    }
}

/// Теплофизические свойства среды.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FluidProps {
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

impl FluidProps {
    /// Кинематическая вязкость, м²/с.
    pub fn nu(&self) -> f64 {
        self.mu / self.rho
    }
    /// Температуропроводность, м²/с.
    pub fn alpha(&self) -> f64 {
        self.k / (self.rho * self.cp)
    }
}

/// Подробный результат расчёта теплоотдачи.
#[derive(Clone, Debug, PartialEq)]
pub struct Convection {
    /// Коэффициент теплоотдачи, Вт/(м²·К).
    pub h: f64,
    /// Число Нуссельта по характерному размеру.
    pub nu: f64,
    /// Число Рейнольдса (0 для свободной конвекции).
    pub re: f64,
    /// Число Рэлея (0, если свободная конвекция не считалась).
    pub ra: f64,
    /// Число Прандтля при температуре определения.
    pub pr: f64,
    /// Какой корреляцией получено.
    pub correlation: &'static str,
    /// Все числа внутри области применимости корреляции.
    pub in_range: bool,
}

// ---------------------------------------------------------------------------
// Корреляции в безразмерном виде
// ---------------------------------------------------------------------------

/// Черчилль–Бернштейн, цилиндр в поперечном потоке, средний Nu. Применимость Re·Pr > 0.2.
pub fn nu_churchill_bernstein(re: f64, pr: f64) -> f64 {
    0.3 + 0.62 * re.sqrt() * pr.cbrt() / (1.0 + (0.4 / pr).powf(2.0 / 3.0)).powf(0.25)
        * (1.0 + (re / 282_000.0).powf(5.0 / 8.0)).powf(4.0 / 5.0)
}

/// Хилперт, цилиндр в поперечном потоке: Nu = C·Re^m·Pr^⅓ (Incropera, табл. 7.2).
/// Применимость 0.4 < Re < 4·10⁵, Pr ≥ 0.7.
pub fn nu_hilpert(re: f64, pr: f64) -> f64 {
    let (c, m) = if re < 4.0 {
        (0.989, 0.330)
    } else if re < 40.0 {
        (0.911, 0.385)
    } else if re < 4000.0 {
        (0.683, 0.466)
    } else if re < 40_000.0 {
        (0.193, 0.618)
    } else {
        (0.027, 0.805)
    };
    c * re.powf(m) * pr.cbrt()
}

/// Жукаускас, цилиндр в поперечном потоке: Nu = C·Re^m·Pr^n·(Pr/Pr_s)^¼, свойства при T_∞.
/// Применимость 1 < Re < 10⁶, 0.7 < Pr < 500.
pub fn nu_zukauskas(re: f64, pr: f64, pr_s: f64) -> f64 {
    let (c, m) = if re < 40.0 {
        (0.75, 0.4)
    } else if re < 1000.0 {
        (0.51, 0.5)
    } else if re < 2.0e5 {
        (0.26, 0.6)
    } else {
        (0.076, 0.7)
    };
    let n = if pr <= 10.0 { 0.37 } else { 0.36 };
    c * re.powf(m) * pr.powf(n) * (pr / pr_s).powf(0.25)
}

/// Черчилль–Чу, свободная конвекция горизонтального цилиндра, средний Nu_D.
/// Применимость Ra_D ≤ 10¹².
pub fn nu_churchill_chu_horizontal_cylinder(ra: f64, pr: f64) -> f64 {
    let x =
        0.60 + 0.387 * ra.powf(1.0 / 6.0) / (1.0 + (0.559 / pr).powf(9.0 / 16.0)).powf(8.0 / 27.0);
    x * x
}

/// Морган, свободная конвекция горизонтального цилиндра: Nu = C·Ra^n (табл. в Incropera 9.1).
/// Второй способ для [`nu_churchill_chu_horizontal_cylinder`].
pub fn nu_morgan_horizontal_cylinder(ra: f64) -> f64 {
    let (c, n) = if ra < 1e-2 {
        (0.675, 0.058)
    } else if ra < 1e2 {
        (1.02, 0.148)
    } else if ra < 1e4 {
        (0.850, 0.188)
    } else if ra < 1e7 {
        (0.480, 0.250)
    } else {
        (0.125, 0.333)
    };
    c * ra.powf(n)
}

/// Черчилль–Чу, вертикальная пластина, средний Nu_L, весь диапазон Ra_L.
pub fn nu_churchill_chu_vertical_plate(ra: f64, pr: f64) -> f64 {
    let x =
        0.825 + 0.387 * ra.powf(1.0 / 6.0) / (1.0 + (0.492 / pr).powf(9.0 / 16.0)).powf(8.0 / 27.0);
    x * x
}

/// Ле Февр–Эде, ламинарная свободная конвекция вертикального цилиндра (учитывает
/// кривизну), средний Nu_L. Применимость: ламинарный режим Ra_L < 10⁹.
pub fn nu_lefevre_ede_vertical_cylinder(ra: f64, pr: f64, length: f64, diameter: f64) -> f64 {
    4.0 / 3.0 * (7.0 * ra * pr / (5.0 * (20.0 + 21.0 * pr))).powf(0.25)
        + 4.0 * (272.0 + 315.0 * pr) * length / (35.0 * (64.0 + 63.0 * pr) * diameter)
}

/// Пластина вдоль потока, средний Nu_L: ламинарный участок + турбулентный (переход при
/// Re_x,c = 5·10⁵): Nu = (0.037·Re^0.8 − 871)·Pr^⅓ при Re > 5·10⁵, иначе 0.664·Re^½·Pr^⅓.
pub fn nu_flat_plate_average(re: f64, pr: f64) -> f64 {
    if re <= 5.0e5 {
        0.664 * re.sqrt() * pr.cbrt()
    } else {
        (0.037 * re.powf(0.8) - 871.0) * pr.cbrt()
    }
}

/// Локальный Nu_x ламинарного слоя по Польгаузену: 0.332·Re_x^½·Pr^⅓ (Pr ≥ 0.6).
pub fn nu_flat_plate_local_laminar(re_x: f64, pr: f64) -> f64 {
    0.332 * re_x.sqrt() * pr.cbrt()
}

/// Локальный Nu_x ламинарного слоя по Черчиллю–Озое (все Pr).
pub fn nu_churchill_ozoe_local(re_x: f64, pr: f64) -> f64 {
    0.3387 * re_x.sqrt() * pr.cbrt() / (1.0 + (0.0468 / pr).powf(2.0 / 3.0)).powf(0.25)
}

/// Точное автомодельное решение Блазиуса–Польгаузена (численно, RK4 + стрельба):
/// возвращает (f''(0), Nu_x/Re_x^½) для числа Прандтля `pr`. Второй, независимый от
/// корреляций способ для ламинарной пластины.
///
/// Уравнения: f''' + ½·f·f'' = 0, f(0) = f'(0) = 0, f'(∞) = 1;
/// θ'' + ½·Pr·f·θ' = 0 ⇒ θ'(0) = 1/∫₀^∞ exp(−½·Pr·∫₀^η f dξ) dη.
pub fn blasius_pohlhausen(pr: f64) -> (f64, f64) {
    let eta_max = 12.0;
    let n = 12_000;
    let h = eta_max / n as f64;
    // y = [f, f', f'', ∫f]
    let rhs = |y: [f64; 4]| [y[1], y[2], -0.5 * y[0] * y[2], y[0]];
    let integrate = |fpp0: f64, mut visit: Option<&mut dyn FnMut(f64, [f64; 4])>| -> [f64; 4] {
        let mut y = [0.0, 0.0, fpp0, 0.0];
        for i in 0..n {
            if let Some(v) = visit.as_mut() {
                v(i as f64 * h, y);
            }
            let k1 = rhs(y);
            let k2 = rhs(add(y, k1, 0.5 * h));
            let k3 = rhs(add(y, k2, 0.5 * h));
            let k4 = rhs(add(y, k3, h));
            for j in 0..4 {
                y[j] += h / 6.0 * (k1[j] + 2.0 * k2[j] + 2.0 * k3[j] + k4[j]);
            }
        }
        if let Some(v) = visit.as_mut() {
            v(eta_max, y);
        }
        y
    };
    let fpp0 = air::bisect(|s| integrate(s, None)[1] - 1.0, 0.1, 1.0, 1e-13);
    // ∫ exp(−½·Pr·F(η)) dη методом Симпсона по узлам RK4
    let mut vals = Vec::with_capacity(n + 1);
    let mut collect = |_eta: f64, y: [f64; 4]| vals.push((-0.5 * pr * y[3]).exp());
    integrate(fpp0, Some(&mut collect));
    let mut integral = vals[0] + vals[n];
    for (i, v) in vals.iter().enumerate().take(n).skip(1) {
        integral += if i % 2 == 1 { 4.0 * v } else { 2.0 * v };
    }
    integral *= h / 3.0;
    (fpp0, 1.0 / integral)
}

fn add(y: [f64; 4], k: [f64; 4], s: f64) -> [f64; 4] {
    [
        y[0] + s * k[0],
        y[1] + s * k[1],
        y[2] + s * k[2],
        y[3] + s * k[3],
    ]
}

// ---------------------------------------------------------------------------
// Размерные функции
// ---------------------------------------------------------------------------

fn film(t_surface_c: f64, t_fluid_c: f64) -> f64 {
    0.5 * (t_surface_c + t_fluid_c)
}

/// Число Рэлея по размеру `length` при свойствах `p` и перепаде `dt`.
pub fn rayleigh(p: &FluidProps, length: f64, dt: f64) -> f64 {
    G * (p.beta * dt).abs() * length.powi(3) / (p.nu() * p.alpha())
}

/// Вынужденная конвекция цилиндра в поперечном потоке (Черчилль–Бернштейн), свойства при
/// плёночной температуре.
pub fn cylinder_crossflow(
    medium: &Medium,
    diameter: f64,
    speed: f64,
    t_surface_c: f64,
    t_fluid_c: f64,
) -> Convection {
    let p = medium.props(film(t_surface_c, t_fluid_c));
    let re = speed.abs() * diameter / p.nu();
    let nu = nu_churchill_bernstein(re, p.pr);
    Convection {
        h: nu * p.k / diameter,
        nu,
        re,
        ra: 0.0,
        pr: p.pr,
        correlation: "Черчилль–Бернштейн (1977)",
        in_range: re * p.pr > 0.2,
    }
}

/// Свободная конвекция горизонтального цилиндра (Черчилль–Чу), свойства при плёночной
/// температуре.
pub fn horizontal_cylinder_natural(
    medium: &Medium,
    diameter: f64,
    t_surface_c: f64,
    t_fluid_c: f64,
) -> Convection {
    let p = medium.props(film(t_surface_c, t_fluid_c));
    let ra = rayleigh(&p, diameter, t_surface_c - t_fluid_c);
    let nu = nu_churchill_chu_horizontal_cylinder(ra, p.pr);
    Convection {
        h: nu * p.k / diameter,
        nu,
        re: 0.0,
        ra,
        pr: p.pr,
        correlation: "Черчилль–Чу, горизонтальный цилиндр (1975)",
        in_range: ra <= 1e12,
    }
}

/// Свободная конвекция вертикальной поверхности высотой `height` (Черчилль–Чу, пластина).
/// Для вертикального цилиндра применима, если D ≥ 35·L/Gr_L^¼ (иначе см.
/// [`vertical_cylinder_natural`]).
pub fn vertical_surface_natural(
    medium: &Medium,
    height: f64,
    t_surface_c: f64,
    t_fluid_c: f64,
) -> Convection {
    let p = medium.props(film(t_surface_c, t_fluid_c));
    let ra = rayleigh(&p, height, t_surface_c - t_fluid_c);
    let nu = nu_churchill_chu_vertical_plate(ra, p.pr);
    Convection {
        h: nu * p.k / height,
        nu,
        re: 0.0,
        ra,
        pr: p.pr,
        correlation: "Черчилль–Чу, вертикальная пластина (1975)",
        in_range: true,
    }
}

/// Свободная конвекция вертикального цилиндра: пластина, если цилиндр «толстый», иначе
/// поправка на кривизну Ле Февра–Эде (ламинарный режим).
pub fn vertical_cylinder_natural(
    medium: &Medium,
    diameter: f64,
    length: f64,
    t_surface_c: f64,
    t_fluid_c: f64,
) -> Convection {
    let p = medium.props(film(t_surface_c, t_fluid_c));
    let ra = rayleigh(&p, length, t_surface_c - t_fluid_c);
    let gr = ra / p.pr;
    if diameter >= 35.0 * length / gr.powf(0.25) || ra >= 1e9 {
        return vertical_surface_natural(medium, length, t_surface_c, t_fluid_c);
    }
    let nu = nu_lefevre_ede_vertical_cylinder(ra, p.pr, length, diameter);
    Convection {
        h: nu * p.k / length,
        nu,
        re: 0.0,
        ra,
        pr: p.pr,
        correlation: "Ле Февр–Эде, тонкий вертикальный цилиндр (1956)",
        in_range: ra < 1e9,
    }
}

/// Пластина длиной `length` вдоль потока, средний h (ламинарный + турбулентный участок).
pub fn flat_plate(
    medium: &Medium,
    length: f64,
    speed: f64,
    t_surface_c: f64,
    t_fluid_c: f64,
) -> Convection {
    let p = medium.props(film(t_surface_c, t_fluid_c));
    let re = speed.abs() * length / p.nu();
    let nu = nu_flat_plate_average(re, p.pr);
    Convection {
        h: nu * p.k / length,
        nu,
        re,
        ra: 0.0,
        pr: p.pr,
        correlation: "пластина: 0.664·Re^½·Pr^⅓ / (0.037·Re^0.8 − 871)·Pr^⅓",
        in_range: p.pr >= 0.6 && re < 1e8,
    }
}

/// Горизонтальный цилиндр в поперечном потоке с учётом свободной конвекции
/// (смешанная, Nu³ = Nu_F³ + Nu_N³). Подробная версия [`h_cylinder`].
pub fn cylinder_mixed(
    medium: &Medium,
    diameter: f64,
    speed: f64,
    t_surface_c: f64,
    t_fluid_c: f64,
) -> Convection {
    let forced = cylinder_crossflow(medium, diameter, speed, t_surface_c, t_fluid_c);
    let natural = horizontal_cylinder_natural(medium, diameter, t_surface_c, t_fluid_c);
    let nu = (forced.nu.powi(3) + natural.nu.powi(3)).cbrt();
    let p = medium.props(film(t_surface_c, t_fluid_c));
    Convection {
        h: nu * p.k / diameter,
        nu,
        re: forced.re,
        ra: natural.ra,
        pr: p.pr,
        correlation: "смешанная: (Nu_ЧБ³ + Nu_ЧЧ³)^⅓",
        in_range: forced.in_range && natural.in_range,
    }
}

/// Коэффициент теплоотдачи цилиндра (рука, корпус мотора, батарея) диаметром `diameter`, м,
/// в потоке скоростью `speed`, м/с, Вт/(м²·К). Вынужденная + свободная конвекция.
pub fn h_cylinder(
    medium: &Medium,
    diameter: f64,
    speed: f64,
    t_surface_c: f64,
    t_fluid_c: f64,
) -> f64 {
    cylinder_mixed(medium, diameter, speed, t_surface_c, t_fluid_c).h
}

/// Коэффициент теплоотдачи пластины длиной `length` вдоль потока, Вт/(м²·К).
pub fn h_flat_plate(
    medium: &Medium,
    length: f64,
    speed: f64,
    t_surface_c: f64,
    t_fluid_c: f64,
) -> f64 {
    flat_plate(medium, length, speed, t_surface_c, t_fluid_c).h
}

/// Коэффициент свободной конвекции горизонтального цилиндра, Вт/(м²·К).
pub fn h_horizontal_cylinder_natural(
    medium: &Medium,
    diameter: f64,
    t_surface_c: f64,
    t_fluid_c: f64,
) -> f64 {
    horizontal_cylinder_natural(medium, diameter, t_surface_c, t_fluid_c).h
}

/// Коэффициент свободной конвекции вертикальной поверхности высотой `height`, Вт/(м²·К).
pub fn h_vertical_natural(medium: &Medium, height: f64, t_surface_c: f64, t_fluid_c: f64) -> f64 {
    vertical_surface_natural(medium, height, t_surface_c, t_fluid_c).h
}

/// Линеаризованный коэффициент лучистого теплообмена серой поверхности с окружением
/// (окружение много больше тела), Вт/(м²·К): h_r = ε·σ·(T_s + T_sur)·(T_s² + T_sur²).
/// Точно: q = h_r·(T_s − T_sur) совпадает с ε·σ·(T_s⁴ − T_sur⁴).
pub fn h_radiation(emissivity: f64, t_surface_c: f64, t_surround_c: f64) -> f64 {
    let ts = t_surface_c + air::T0;
    let tr = t_surround_c + air::T0;
    emissivity * STEFAN_BOLTZMANN * (ts + tr) * (ts * ts + tr * tr)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blasius_wall_shear() {
        let (fpp0, nu1) = blasius_pohlhausen(1.0);
        assert!((fpp0 - 0.332_057).abs() < 1e-5, "{fpp0}");
        // при Pr = 1 Nu_x/Re_x^½ = f''(0) (аналогия Рейнольдса)
        assert!((nu1 - fpp0).abs() < 1e-4, "{nu1}");
    }

    #[test]
    fn forearm_in_wind_is_reasonable() {
        // предплечье 8 см на ветру 5 м/с, воздух 0 °C, кожа 30 °C: h ≈ 20–30 Вт/(м²·К)
        let h = h_cylinder(&Medium::air(50.0), 0.08, 5.0, 30.0, 0.0);
        assert!((15.0..40.0).contains(&h), "{h}");
        // в воде в 20–100 раз больше
        let hw = h_cylinder(&Medium::FreshWater, 0.08, 0.5, 30.0, 10.0);
        assert!(hw > 20.0 * h, "{hw}");
    }
}
