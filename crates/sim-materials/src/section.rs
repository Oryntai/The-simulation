//! Сечения стержня: прямоугольник, круг, труба. Геометрические характеристики, напряжения
//! от растяжения, изгиба, сдвига и кручения, потеря устойчивости (Эйлер и Джонсон).
//!
//! Единицы — СИ. Изгиб считается вокруг оси, параллельной стороне `b` прямоугольника: высота
//! сечения в плоскости изгиба — `h`.

use std::f64::consts::PI;

/// Поперечное сечение.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Section {
    /// Прямоугольник: `b` — ширина (вдоль оси изгиба), `h` — высота (в плоскости изгиба), м.
    Rect { b: f64, h: f64 },
    /// Сплошной круг диаметром `d`, м.
    Round { d: f64 },
    /// Труба: наружный диаметр `od`, толщина стенки `wall`, м.
    Tube { od: f64, wall: f64 },
}

impl Section {
    /// Геометрия задана корректно: все размеры конечны, положительны, стенка меньше радиуса.
    pub fn validate(&self) -> Result<(), String> {
        let ok = |v: f64| v.is_finite() && v > 0.0;
        match *self {
            Section::Rect { b, h } => {
                if ok(b) && ok(h) {
                    Ok(())
                } else {
                    Err(format!(
                        "размеры прямоугольника некорректны: b = {b}, h = {h}"
                    ))
                }
            }
            Section::Round { d } => {
                if ok(d) {
                    Ok(())
                } else {
                    Err(format!("диаметр круга некорректен: {d}"))
                }
            }
            Section::Tube { od, wall } => {
                if ok(od) && ok(wall) && 2.0 * wall < od {
                    Ok(())
                } else {
                    Err(format!(
                        "размеры трубы некорректны: наружный {od}, стенка {wall} \
                         (стенка должна быть меньше радиуса)"
                    ))
                }
            }
        }
    }

    /// Площадь, м².
    pub fn area(&self) -> f64 {
        match *self {
            Section::Rect { b, h } => b * h,
            Section::Round { d } => PI * d * d / 4.0,
            Section::Tube { od, wall } => {
                let id = od - 2.0 * wall;
                PI * (od * od - id * id) / 4.0
            }
        }
    }

    /// Момент инерции относительно оси изгиба, м⁴.
    pub fn inertia(&self) -> f64 {
        match *self {
            Section::Rect { b, h } => b * h.powi(3) / 12.0,
            Section::Round { d } => PI * d.powi(4) / 64.0,
            Section::Tube { od, wall } => {
                let id = od - 2.0 * wall;
                PI * (od.powi(4) - id.powi(4)) / 64.0
            }
        }
    }

    /// Наименьший момент инерции (для потери устойчивости), м⁴.
    pub fn inertia_min(&self) -> f64 {
        match *self {
            Section::Rect { b, h } => b * h * b.min(h).powi(2) / 12.0,
            _ => self.inertia(),
        }
    }

    /// Расстояние от нейтральной оси до крайнего волокна, м.
    pub fn c(&self) -> f64 {
        match *self {
            Section::Rect { h, .. } => h / 2.0,
            Section::Round { d } => d / 2.0,
            Section::Tube { od, .. } => od / 2.0,
        }
    }

    /// Момент сопротивления изгибу, м³.
    pub fn section_modulus(&self) -> f64 {
        self.inertia() / self.c()
    }

    /// Наименьший радиус инерции, м.
    pub fn radius_of_gyration_min(&self) -> f64 {
        (self.inertia_min() / self.area()).sqrt()
    }

    /// Наибольший размер сечения, м (для коэффициента размера в усталости).
    pub fn max_dimension(&self) -> f64 {
        match *self {
            Section::Rect { b, h } => b.max(h),
            Section::Round { d } => d,
            Section::Tube { od, .. } => od,
        }
    }

    /// Наибольшее касательное напряжение от поперечной силы `v`, Па: τ = V·Q/(I·t) на
    /// нейтральной оси (Shigley, §3-11).
    pub fn shear_stress(&self, v: f64) -> f64 {
        let v = v.abs();
        match *self {
            Section::Rect { .. } => 1.5 * v / self.area(),
            Section::Round { .. } => 4.0 * v / (3.0 * self.area()),
            Section::Tube { od, wall } => {
                let (ro, ri) = (od / 2.0, od / 2.0 - wall);
                let q = 2.0 / 3.0 * (ro.powi(3) - ri.powi(3));
                v * q / (self.inertia() * 2.0 * wall)
            }
        }
    }

    /// Наибольшее касательное напряжение от крутящего момента `t`, Па. Круг и труба — T·r/J;
    /// прямоугольник — точное решение Сен-Венана рядом (Timoshenko & Goodier, «Theory of
    /// Elasticity», гл. 10).
    pub fn torsion_stress(&self, t: f64) -> f64 {
        let t = t.abs();
        match *self {
            Section::Rect { b, h } => {
                t / (rect_torsion_alpha(b.max(h) / b.min(h)) * b.max(h) * b.min(h).powi(2))
            }
            Section::Round { d } => 16.0 * t / (PI * d.powi(3)),
            Section::Tube { od, wall } => {
                let id = od - 2.0 * wall;
                let j = PI * (od.powi(4) - id.powi(4)) / 32.0;
                t * (od / 2.0) / j
            }
        }
    }
}

/// Коэффициент α в формуле τ = T/(α·a·c²) для прямоугольника a ≥ c, `ratio` = a/c.
/// Считается рядом Сен-Венана, сверяется с таблицей Shigley 3-1.
pub fn rect_torsion_alpha(ratio: f64) -> f64 {
    let r = ratio.max(1.0);
    let mut sum_k = 0.0;
    let mut sum_tau = 0.0;
    // Ряд по нечётным n сходится быстро; 41 член заведомо хватает для двойной точности.
    for i in 0..41 {
        let n = (2 * i + 1) as f64;
        let arg = n * PI * r / 2.0;
        sum_k += arg.tanh() / n.powi(5);
        // При большом аргументе cosh → ∞, слагаемое → 0: без переполнения в NaN.
        sum_tau += if arg > 300.0 {
            0.0
        } else {
            1.0 / (n * n * arg.cosh())
        };
    }
    let k1 = (1.0 - 192.0 / (PI.powi(5) * r) * sum_k) / 3.0;
    let shape = 1.0 - 8.0 / (PI * PI) * sum_tau;
    k1 / shape
}

/// Напряжение растяжения-сжатия, Па.
pub fn axial_stress(force: f64, area: f64) -> f64 {
    force / area
}

/// Напряжение в крайнем волокне при изгибе, Па.
pub fn bending_stress(moment: f64, section_modulus: f64) -> f64 {
    moment / section_modulus
}

/// Эквивалентное напряжение по Мизесу для σ и τ в одной точке, Па.
pub fn von_mises(sigma: f64, tau: f64) -> f64 {
    (sigma * sigma + 3.0 * tau * tau).sqrt()
}

/// Закрепление концов стержня при расчёте на устойчивость.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EndCondition {
    /// Шарнир — шарнир.
    PinnedPinned,
    /// Заделка — заделка.
    FixedFixed,
    /// Заделка — шарнир.
    FixedPinned,
    /// Заделка — свободный конец.
    FixedFree,
}

impl EndCondition {
    /// Коэффициент приведённой длины K: **рекомендованные** значения AISC 360, комментарий к
    /// приложению 7, табл. C-A-7.1. Они выше теоретических (0.5 / 0.7 / 2.0), потому что
    /// настоящая заделка не абсолютно жёсткая.
    pub fn k(self) -> f64 {
        match self {
            EndCondition::PinnedPinned => 1.0,
            EndCondition::FixedFixed => 0.65,
            EndCondition::FixedPinned => 0.80,
            EndCondition::FixedFree => 2.10,
        }
    }
}

/// Какая формула дала критическую силу.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BucklingRegime {
    /// Длинный стержень: упругая потеря устойчивости.
    Euler,
    /// Средняя гибкость: неупругая потеря устойчивости.
    Johnson,
}

/// Результат расчёта на устойчивость.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Buckling {
    /// Критическая сила, Н.
    pub critical_load: f64,
    /// Гибкость K·L/r.
    pub slenderness: f64,
    /// Граничная гибкость √(2π²E/Sy).
    pub transition: f64,
    pub regime: BucklingRegime,
}

/// Критическая сила стержня: Эйлер при λ ≥ λт, парабола Джонсона при λ < λт (Shigley, §4-14):
/// P = A·Sy·(1 − Sy·λ²/(4π²E)). На границе обе формулы дают A·Sy/2.
pub fn critical_load(
    section: &Section,
    e: f64,
    sy: f64,
    length: f64,
    end: EndCondition,
) -> Buckling {
    let area = section.area();
    let r = section.radius_of_gyration_min();
    let lambda = end.k() * length / r;
    let transition = (2.0 * PI * PI * e / sy).sqrt();
    if lambda >= transition {
        Buckling {
            critical_load: PI * PI * e * area / (lambda * lambda),
            slenderness: lambda,
            transition,
            regime: BucklingRegime::Euler,
        }
    } else {
        Buckling {
            critical_load: area * sy * (1.0 - sy * lambda * lambda / (4.0 * PI * PI * e)),
            slenderness: lambda,
            transition,
            regime: BucklingRegime::Johnson,
        }
    }
}

/// Предел отношения D/t для круглой трубы при сжатии, выше которого формула стержня не
/// применима из-за местной потери устойчивости: 0.11·E/Fy (AISC 360-16, табл. B4.1a, случай 9).
pub fn tube_local_buckling_limit(e: f64, sy: f64) -> f64 {
    0.11 * e / sy
}

/// Предел D/t компактной круглой трубы при изгибе: 0.07·E/Fy (AISC 360-16, табл. B4.1b,
/// случай 20). Выше него стенка может смяться раньше, чем сечение выйдет на расчётный момент.
pub fn tube_flexure_compact_limit(e: f64, sy: f64) -> f64 {
    0.07 * e / sy
}

/// Предельный момент высокого прямоугольного бруса по боковому выпучиванию (AISC 360-16, F11,
/// Cb = 1 — постоянный момент, худший случай). `depth` — высота в плоскости изгиба, `width` —
/// ширина, `unbraced` — длина без боковых раскреплений.
///
/// `None` — форма не грозит: изгиб вокруг слабой оси или L·d/t² ≤ 0.08·E/Fy.
pub fn rect_lateral_buckling(
    depth: f64,
    width: f64,
    e: f64,
    fy: f64,
    unbraced: f64,
) -> Option<f64> {
    if depth <= width {
        return None;
    }
    let lambda = unbraced * depth / (width * width);
    if lambda <= 0.08 * e / fy {
        return None;
    }
    let s = width * depth * depth / 6.0;
    let my = fy * s;
    let mp = fy * width * depth * depth / 4.0;
    let mn = if lambda <= 1.9 * e / fy {
        (1.52 - 0.274 * lambda * fy / e) * my // F11-2
    } else {
        1.9 * e / lambda * s // F11-3, F11-4
    };
    Some(mn.min(mp))
}

#[cfg(test)]
mod tests {
    use super::*;

    const R: Section = Section::Rect { b: 0.02, h: 0.01 };

    #[test]
    fn lateral_buckling_only_for_tall_slender_bars() {
        let (e, fy) = (200e9, 250e6);
        // Изгиб вокруг слабой оси и короткий брус — формы нет.
        assert!(rect_lateral_buckling(0.005, 0.03, e, fy, 1.0).is_none());
        assert!(rect_lateral_buckling(0.02, 0.01, e, fy, 0.05).is_none());
        // Длиннее — момент падает монотонно и не превышает пластический.
        let mp = fy * 0.002 * 0.04f64.powi(2) / 4.0;
        let mut prev = f64::INFINITY;
        for i in 1..200 {
            let l = i as f64 * 0.02;
            if let Some(m) = rect_lateral_buckling(0.04, 0.002, e, fy, l) {
                assert!(
                    m <= mp * (1.0 + 1e-12) && m <= prev * (1.0 + 1e-9),
                    "L = {l}"
                );
                prev = m;
            }
        }
        assert!(prev < mp / 2.0);
    }

    #[test]
    fn invalid_geometry_rejected() {
        for s in [
            Section::Rect {
                b: f64::NAN,
                h: 0.01,
            },
            Section::Rect { b: 0.01, h: -1.0 },
            Section::Round { d: f64::INFINITY },
            Section::Round { d: 0.0 },
            Section::Tube {
                od: 0.01,
                wall: 0.005,
            },
            Section::Tube {
                od: 0.01,
                wall: 0.006,
            },
            Section::Tube {
                od: f64::NAN,
                wall: 0.001,
            },
        ] {
            assert!(s.validate().is_err(), "{s:?}");
        }
        assert!(R.validate().is_ok());
    }

    #[test]
    fn rect_weak_axis_for_buckling() {
        assert!((R.inertia() - 0.02 * 1e-6 / 12.0).abs() < 1e-18);
        // Стержень 20×10 теряет устойчивость вокруг слабой оси, какой бы стороной ни лёг.
        let flipped = Section::Rect { b: 0.01, h: 0.02 };
        assert!((R.inertia_min() - flipped.inertia_min()).abs() < 1e-18);
        assert!(flipped.inertia() > flipped.inertia_min());
    }

    #[test]
    fn euler_and_johnson_meet() {
        let (e, sy) = (200e9, 400e6);
        let probe = critical_load(
            &Section::Round { d: 0.02 },
            e,
            sy,
            1.0,
            EndCondition::PinnedPinned,
        );
        // Длина, при которой гибкость ровно на границе.
        let r = Section::Round { d: 0.02 }.radius_of_gyration_min();
        let l = probe.transition * r;
        let above = critical_load(
            &Section::Round { d: 0.02 },
            e,
            sy,
            l * 1.0001,
            EndCondition::PinnedPinned,
        );
        let below = critical_load(
            &Section::Round { d: 0.02 },
            e,
            sy,
            l * 0.9999,
            EndCondition::PinnedPinned,
        );
        assert_eq!(above.regime, BucklingRegime::Euler);
        assert_eq!(below.regime, BucklingRegime::Johnson);
        let half = Section::Round { d: 0.02 }.area() * sy / 2.0;
        assert!((above.critical_load - half).abs() / half < 1e-3);
        assert!((below.critical_load - half).abs() / half < 1e-3);
    }

    #[test]
    fn johnson_never_exceeds_squash_load() {
        let s = Section::Round { d: 0.02 };
        for l in [0.001, 0.01, 0.05, 0.1, 0.3] {
            let b = critical_load(&s, 200e9, 400e6, l, EndCondition::PinnedPinned);
            assert!(b.critical_load <= s.area() * 400e6);
        }
    }

    #[test]
    fn longer_column_is_weaker() {
        let s = Section::Tube {
            od: 0.02,
            wall: 0.002,
        };
        let mut prev = f64::INFINITY;
        for i in 1..50 {
            let b = critical_load(&s, 70e9, 240e6, i as f64 * 0.05, EndCondition::FixedFree);
            assert!(b.critical_load < prev);
            prev = b.critical_load;
        }
    }

    #[test]
    fn torsion_alpha_long_strip_tends_to_third() {
        assert!((rect_torsion_alpha(1000.0) - 1.0 / 3.0).abs() < 1e-3);
        assert!(rect_torsion_alpha(1000.0).is_finite());
    }
}
