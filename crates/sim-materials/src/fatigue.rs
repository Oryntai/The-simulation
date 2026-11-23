//! Усталость: кривые S-N (Баскин), среднее напряжение (модифицированный Гудман), концентраторы
//! по Петерсону, чувствительность к надрезу, правило Майнера.
//!
//! Основа — Budynas & Nisbett, «Shigley's Mechanical Engineering Design» (9–11-е изд., гл. 6);
//! коэффициенты концентрации — Pilkey & Pilkey, «Peterson's Stress Concentration Factors» и
//! Pilkey, «Formulas for Stress, Strain, and Structural Matrices», 2-е изд., 2005 (кубические
//! аппроксимации в выписке <https://amesweb.info/StressConcentrationFactor/>).
//!
//! Напряжения — в Па; формулы Шигли, заданные в МПа или kpsi, пересчитываются внутри.

/// МПа в одном kpsi.
const MPA_PER_KPSI: f64 = 6.894_757;

/// Обработка поверхности (для коэффициента ka Марина).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Surface {
    /// Шлифовка.
    Ground,
    /// Механическая обработка или холодная протяжка.
    Machined,
    /// Горячий прокат.
    HotRolled,
    /// После ковки.
    AsForged,
}

/// ka = a·Sut^b по таблице 6-2 Шигли (10-е изд.), Sut в МПа; результат не больше 1.
pub fn surface_factor_10e(surface: Surface, sut_mpa: f64) -> f64 {
    let (a, b) = match surface {
        Surface::Ground => (1.58, -0.085),
        Surface::Machined => (4.51, -0.265),
        Surface::HotRolled => (57.7, -0.718),
        Surface::AsForged => (272.0, -0.995),
    };
    (a * sut_mpa.powf(b)).min(1.0)
}

/// Расчётный ka. Для механической обработки — меньшее из 10-го (4.51, −0.265) и 11-го
/// (3.04, −0.217) изданий: 10-е завышает до ~8%. Формулы выведены на сталях; для алюминия и
/// титана это **оценка**.
pub fn surface_factor(surface: Surface, sut_mpa: f64) -> f64 {
    let k10 = surface_factor_10e(surface, sut_mpa);
    match surface {
        Surface::Machined => k10.min(3.04 * sut_mpa.powf(-0.217)),
        _ => k10,
    }
}

/// Коэффициент размера kb при изгибе и кручении по диаметру, м (Shigley, ур. 6-20):
/// 1.24·d^−0.107 при 2.79–51 мм, 1.51·d^−0.157 при 51–254 мм. Меньше 2.79 мм — 1.
/// Для некруглых сечений вызывающая сторона передаёт наибольший размер: это в запас.
pub fn size_factor(d: f64) -> f64 {
    let mm = d * 1e3;
    if mm <= 2.79 {
        1.0
    } else if mm <= 51.0 {
        1.24 * mm.powf(-0.107)
    } else {
        1.51 * mm.min(254.0).powf(-0.157)
    }
}

/// Коэффициент нагрузки kc для осевой нагрузки (Shigley, ур. 6-26): 0.85. Для изгиба — 1;
/// кручение входит через эквивалент Мизеса.
pub const KC_AXIAL: f64 = 0.85;

/// Надёжность (коэффициент ke, Shigley, табл. 6-5).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Reliability {
    R50,
    R90,
    R95,
    /// 99% — значение по умолчанию для всего, что носят на теле.
    #[default]
    R99,
    R999,
    R9999,
}

impl Reliability {
    /// Коэффициент ke.
    pub fn factor(self) -> f64 {
        match self {
            Reliability::R50 => 1.0,
            Reliability::R90 => 0.897,
            Reliability::R95 => 0.868,
            Reliability::R99 => 0.814,
            Reliability::R999 => 0.753,
            Reliability::R9999 => 0.702,
        }
    }
}

/// Доля f от Sut, которую выдерживают 10³ циклов (Shigley, рис. 6-18): 0.9 при Sut ≤ 70 kpsi,
/// дальше f = 1.06 − 2.8·10⁻³·S + 6.9·10⁻⁶·S² (S в kpsi, до 200 kpsi).
pub fn fatigue_fraction(sut_pa: f64) -> f64 {
    let s = sut_pa / 1e6 / MPA_PER_KPSI;
    if s <= 70.0 {
        0.9
    } else {
        let s = s.min(200.0);
        1.06 - 2.8e-3 * s + 6.9e-6 * s * s
    }
}

/// Кривая S-N в форме Баскина: σ = a·N^b.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SnCurve {
    /// Коэффициент, Па.
    pub a: f64,
    /// Показатель (отрицательный).
    pub b: f64,
    /// Число циклов на конце наклонного участка.
    pub knee_cycles: f64,
    /// Прочность на конце наклонного участка, Па.
    pub knee_strength: f64,
    /// Есть ли настоящий предел выносливости (ниже колена долговечность бесконечна).
    pub endurance_limit: bool,
}

/// Начало многоцикловой области.
pub const HCF_START: f64 = 1e3;

impl SnCurve {
    /// Прямая в логарифмических координатах через две точки.
    pub fn through(n1: f64, s1: f64, n2: f64, s2: f64, endurance_limit: bool) -> Self {
        let b = (s2 / s1).log10() / (n2 / n1).log10();
        let a = s1 / n1.powf(b);
        Self {
            a,
            b,
            knee_cycles: n2,
            knee_strength: s2,
            endurance_limit,
        }
    }

    /// Кривая по правилу Шигли: от f·Sut на 10³ циклов до `sf` на `n_ref` циклов.
    pub fn from_sut(sut: f64, sf: f64, n_ref: f64, endurance_limit: bool) -> Self {
        // Сильно сниженный предел всё равно не может быть выше прочности на 10³ циклов.
        let s_start = (fatigue_fraction(sut) * sut).max(sf * (1.0 + 1e-9));
        Self::through(HCF_START, s_start, n_ref, sf, endurance_limit)
    }

    /// Усталостная прочность на `n` циклов, Па. Меньше 10³ циклов кривая не поднимается: в
    /// малоцикловой области формула Баскина неприменима, берём значение на 10³ в запас.
    pub fn strength_at(&self, n: f64) -> f64 {
        let n = n.max(HCF_START);
        if self.endurance_limit && n >= self.knee_cycles {
            self.knee_strength
        } else {
            self.a * n.powf(self.b)
        }
    }

    /// Долговечность при полностью обратном напряжении, циклов. Ниже предела выносливости —
    /// бесконечность (только при `endurance_limit`).
    pub fn life(&self, sigma: f64) -> f64 {
        if self.endurance_limit && sigma <= self.knee_strength {
            f64::INFINITY
        } else {
            self.life_no_cutoff(sigma)
        }
    }

    /// Долговечность по наклонной прямой без отсечки: для спектра нагрузки. Большие циклы
    /// «расшатывают» предел выносливости, поэтому при суммировании повреждений отсечка опасна.
    pub fn life_no_cutoff(&self, sigma: f64) -> f64 {
        if sigma <= 0.0 {
            f64::INFINITY
        } else {
            (sigma / self.a).powf(1.0 / self.b)
        }
    }
}

/// Запас по модифицированному Гудману: 1/n = σa/Sf + σm/Sut. Сжимающее среднее (σm < 0)
/// запаса не добавляет и считается нулевым.
pub fn goodman_factor(sigma_a: f64, sigma_m: f64, sf: f64, sut: f64) -> f64 {
    1.0 / (sigma_a / sf + sigma_m.max(0.0) / sut)
}

/// Эквивалентное полностью обратное напряжение по Гудману: σar = σa/(1 − σm/Sut).
/// При σm ≥ Sut деталь разрушится за первый цикл: возвращает бесконечность.
pub fn equivalent_reversed(sigma_a: f64, sigma_m: f64, sut: f64) -> f64 {
    let m = sigma_m.max(0.0);
    if m >= sut {
        f64::INFINITY
    } else {
        sigma_a / (1.0 - m / sut)
    }
}

/// Блок спектра нагрузки.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StressBlock {
    /// Амплитуда, Па.
    pub sigma_a: f64,
    /// Среднее, Па.
    pub sigma_m: f64,
    /// Число циклов в блоке.
    pub cycles: f64,
}

/// Повреждение по Пальмгрену–Майнеру: D = Σ nᵢ/Nᵢ. Отсечка по пределу выносливости не
/// применяется (см. [`SnCurve::life_no_cutoff`]). D ≥ 1 — разрушение.
pub fn miner_damage(curve: &SnCurve, sut: f64, blocks: &[StressBlock]) -> f64 {
    blocks
        .iter()
        .map(|b| {
            if b.cycles <= 0.0 {
                return 0.0;
            }
            let s = equivalent_reversed(b.sigma_a, b.sigma_m, sut);
            if !s.is_finite() {
                return f64::INFINITY;
            }
            b.cycles / curve.life_no_cutoff(s)
        })
        .sum()
}

/// Запас по напряжениям для спектра: во сколько раз надо увеличить все напряжения, чтобы
/// повреждение стало равно 1. Логарифмическая бисекция, результат в 10⁻⁶..10⁶.
pub fn miner_stress_factor(curve: &SnCurve, sut: f64, blocks: &[StressBlock]) -> f64 {
    let damage = |s: f64| {
        let scaled: Vec<StressBlock> = blocks
            .iter()
            .map(|b| StressBlock {
                sigma_a: b.sigma_a * s,
                sigma_m: b.sigma_m * s,
                cycles: b.cycles,
            })
            .collect();
        miner_damage(curve, sut, &scaled)
    };
    let (mut lo, mut hi) = (1e-6_f64, 1e6_f64);
    if damage(lo) >= 1.0 {
        return lo;
    }
    if damage(hi) < 1.0 {
        return hi;
    }
    for _ in 0..200 {
        let mid = (lo * hi).sqrt();
        if damage(mid) < 1.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo
}

// --- Концентраторы -------------------------------------------------------------------------

/// Kt отверстия в полосе при растяжении, по **нетто**-сечению σ = F/((D − d)·t).
/// Pilkey (2005): 3.000 − 3.140x + 3.667x² − 1.527x³, x = d/D, 0 ≤ x ≤ 1.
#[allow(clippy::approx_constant)] // 3.140 — коэффициент аппроксимации, а не π
pub fn kt_hole_tension(d_over_w: f64) -> f64 {
    let x = d_over_w.clamp(0.0, 1.0);
    3.000 - 3.140 * x + 3.667 * x * x - 1.527 * x * x * x
}

/// Вторая независимая аппроксимация той же кривой (Peterson, диаграмма 4.1):
/// Ktn = 2 + 0.284(1 − x) − 0.600(1 − x)² + 1.32(1 − x)³. Только для сверки.
pub fn kt_hole_tension_alt(d_over_w: f64) -> f64 {
    let y = 1.0 - d_over_w.clamp(0.0, 1.0);
    2.0 + 0.284 * y - 0.600 * y * y + 1.32 * y * y * y
}

/// Kt отверстия на оси полосы при изгибе **в плоскости** полосы, относительно напряжения в
/// крайнем волокне нетто-сечения σ = 6M·D/((D³ − d³)·t).
///
/// Pilkey (2005): у края отверстия Kt = 2 относительно 6M·d/((D³ − d³)·t), то есть 2d/D
/// относительно волокна. Край полосы — 1. Берётся наибольшее из двух.
pub fn kt_hole_bending_in_plane(d_over_h: f64) -> f64 {
    (2.0 * d_over_h.clamp(0.0, 1.0)).max(1.0)
}

/// Kt отверстия на оси при изгибе **из плоскости** полосы, σ = 6M/((D − d)·t²).
///
/// Pilkey (2005), 0 ≤ d/D ≤ 0.3, 1 ≤ d/t ≤ 7:
/// (1.793 + 0.131/u + 2.052/u² − 1.019/u³)·(1 − 1.04x + 1.22x²), u = d/t, x = d/D.
/// В этой области формула не ниже кривых Шигли A-15-2 (при d/t = 1 заметно выше).
///
/// Вне области — кривая растяжения: она совпадает с кривой d/h → 0 на рис. A-15-2 и
/// ограничивает все остальные сверху (завышение для тонких пластин до ×1.6, сторона безопасная).
pub fn kt_hole_bending_out_of_plane(d_over_w: f64, d_over_t: f64) -> f64 {
    if (0.0..=0.3).contains(&d_over_w) && (1.0..=7.0).contains(&d_over_t) {
        let u = d_over_t;
        let x = d_over_w;
        (1.793 + 0.131 / u + 2.052 / (u * u) - 1.019 / (u * u * u))
            * (1.0 - 1.04 * x + 1.22 * x * x)
    } else {
        kt_hole_tension(d_over_w)
    }
}

/// Kt отверстия при чистом сдвиге: σmax = 4τ (бесконечная пластина, решение Кирша).
/// В эквивалент Мизеса τ входит как √3·τ, поэтому множитель к τ равен 4/√3.
pub const KTS_HOLE: f64 = 2.309_401_076_758_503;

/// Коэффициенты кубической формы Kt = C1 + C2·s + C3·s² + C4·s³, s = 2h/D, h = (D − d)/2.
fn fillet_poly(c: [[f64; 3]; 4], ratio: f64, s: f64) -> f64 {
    let root = ratio.sqrt();
    let v: Vec<f64> = c
        .iter()
        .map(|k| k[0] + k[1] * root + k[2] * ratio)
        .collect();
    v[0] + v[1] * s + v[2] * s * s + v[3] * s * s * s
}

/// Подготовка параметров галтели: `(h/r, 2h/D)`. Вне применимости по h/r > 20 — `None`;
/// при h/r < 0.1 берётся 0.1: уступ ещё меньше, реальный Kt ниже, оценка в безопасную сторону.
fn fillet_params(big: f64, small: f64, r: f64) -> Option<(f64, f64)> {
    if !(big.is_finite() && small.is_finite() && r.is_finite())
        || r <= 0.0
        || small <= 0.0
        || big <= small
    {
        return None;
    }
    let h = (big - small) / 2.0;
    let ratio = h / r;
    if ratio > 20.0 {
        return None;
    }
    Some((ratio.max(0.1), 2.0 * h / big))
}

/// Узлы r/d графиков Шигли A-15-5 и A-15-6. Острее первого узла графиков нет: Kt «не посчитан».
const FILLET_RD: [f64; 6] = [0.025, 0.05, 0.10, 0.15, 0.20, 0.30];
/// Узлы D/d графика растяжения.
const FILLET_TENSION_DD: [f64; 4] = [1.02, 1.05, 1.10, 1.50];
/// Показания графика A-15-5 (растяжение). При D/d = 1.5, r/d = 0.025 кривая уходит за край
/// графика (> 3): стоит нижняя граница 3.0, а значение даёт аппроксимация.
const FILLET_TENSION_KT: [[f64; 6]; 4] = [
    [1.79, 1.54, 1.35, 1.27, 1.23, 1.19],
    [2.16, 1.73, 1.48, 1.38, 1.34, 1.28],
    [2.46, 2.05, 1.65, 1.51, 1.42, 1.35],
    [3.00, 2.72, 2.10, 1.88, 1.74, 1.56],
];
/// Узлы D/d графика изгиба.
const FILLET_BENDING_DD: [f64; 5] = [1.02, 1.05, 1.10, 1.30, 3.00];
/// Показания графика A-15-6 (изгиб).
const FILLET_BENDING_KT: [[f64; 6]; 5] = [
    [1.87, 1.63, 1.41, 1.33, 1.30, 1.27],
    [2.12, 1.78, 1.56, 1.44, 1.39, 1.32],
    [2.23, 1.94, 1.60, 1.47, 1.40, 1.34],
    [2.56, 2.15, 1.74, 1.57, 1.47, 1.37],
    [2.98, 2.44, 1.89, 1.64, 1.51, 1.40],
];
/// Запас на точность чтения графика глазами.
const CHART_READ_MARGIN: f64 = 0.03;

/// Кусочно-линейная интерполяция с насыщением на краях.
fn interp(xs: &[f64], ys: &[f64], x: f64) -> f64 {
    if x <= xs[0] {
        return ys[0];
    }
    for i in 1..xs.len() {
        if x <= xs[i] {
            return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
        }
    }
    ys[ys.len() - 1]
}

/// Kt по графику: билинейно по r/d и D/d, плюс запас на чтение.
fn chart_kt(rows: &[f64], table: &[[f64; 6]], d_ratio: f64, r_ratio: f64) -> f64 {
    let at_r: Vec<f64> = table
        .iter()
        .map(|row| interp(&FILLET_RD, row, r_ratio))
        .collect();
    interp(rows, &at_r, d_ratio) + CHART_READ_MARGIN
}

/// Kt галтели ступенчатой полосы при растяжении, σ = F/(d·t).
///
/// Берётся **большее** из аппроксимации Pilkey (2005, 0.1 ≤ h/r ≤ 20) и графика Шигли A-15-5:
/// при малых уступах и острых радиусах аппроксимация оказалась ниже графика до 9%, а занижать
/// концентратор нельзя. При r/d < 0.025 данных нет — `None`.
pub fn kt_fillet_tension(big: f64, small: f64, r: f64) -> Option<f64> {
    let (ratio, s) = fillet_params(big, small, r)?;
    if r / small < FILLET_RD[0] {
        return None;
    }
    let chart = chart_kt(
        &FILLET_TENSION_DD,
        &FILLET_TENSION_KT,
        big / small,
        r / small,
    );
    let c = if ratio <= 2.0 {
        [
            [1.006, 1.008, -0.044],
            [-0.115, -0.584, 0.315],
            [0.245, -1.006, -0.257],
            [-0.135, 0.582, -0.017],
        ]
    } else {
        [
            [1.020, 1.009, -0.048],
            [-0.065, -0.165, -0.007],
            [-3.459, 1.266, -0.016],
            [3.505, -2.109, 0.069],
        ]
    };
    Some(fillet_poly(c, ratio, s).max(chart).max(1.0))
}

/// Kt галтели ступенчатой полосы при изгибе, σ = 6M/(t·d²): большее из аппроксимации Pilkey
/// (2005, 0.1 ≤ h/r ≤ 20) и графика Шигли A-15-6. При r/d < 0.025 данных нет — `None`.
pub fn kt_fillet_bending(big: f64, small: f64, r: f64) -> Option<f64> {
    let (ratio, s) = fillet_params(big, small, r)?;
    if r / small < FILLET_RD[0] {
        return None;
    }
    let chart = chart_kt(
        &FILLET_BENDING_DD,
        &FILLET_BENDING_KT,
        big / small,
        r / small,
    );
    let c = if ratio <= 2.0 {
        [
            [1.006, 0.967, 0.013],
            [-0.270, -2.372, 0.708],
            [0.662, 1.157, -0.908],
            [-0.405, 0.249, -0.200],
        ]
    } else {
        [
            [1.058, 1.002, -0.038],
            [-3.652, 1.639, -0.436],
            [6.170, -5.687, 1.175],
            [-2.558, 3.046, -0.701],
        ]
    };
    Some(fillet_poly(c, ratio, s).max(chart).max(1.0))
}

/// Чувствительность к надрезу стали по Нойберу: q = 1/(1 + √a/√r), √a — полином Шигли
/// (ур. 6-35a/b), Sut в kpsi, r в дюймах. Применимо при 50 ≤ Sut ≤ 250 kpsi; вне диапазона и
/// при нулевом радиусе возвращает 1 (полная чувствительность — в запас).
pub fn notch_sensitivity_steel(sut_pa: f64, r: f64, torsion: bool) -> f64 {
    let s = sut_pa / 1e6 / MPA_PER_KPSI;
    if !(50.0..=250.0).contains(&s) || !(r.is_finite() && r > 0.0) {
        return 1.0;
    }
    let sqrt_a = if torsion {
        0.190 - 2.51e-3 * s + 1.35e-5 * s * s - 2.67e-8 * s * s * s
    } else {
        0.246 - 3.08e-3 * s + 1.51e-5 * s * s - 2.67e-8 * s * s * s
    };
    let r_in = r / 0.0254;
    (1.0 / (1.0 + sqrt_a / r_in.sqrt())).clamp(0.0, 1.0)
}

/// Эффективный коэффициент концентрации: Kf = 1 + q·(Kt − 1).
pub fn fatigue_notch_factor(kt: f64, q: f64) -> f64 {
    1.0 + q * (kt - 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hole_tension_limits() {
        assert!((kt_hole_tension(0.0) - 3.0).abs() < 1e-12);
        // Две независимые аппроксимации одной кривой сходятся.
        for i in 0..=8 {
            let x = i as f64 * 0.1;
            let (a, b) = (kt_hole_tension(x), kt_hole_tension_alt(x));
            assert!((a - b).abs() < 0.03, "d/w = {x}: {a} против {b}");
        }
    }

    #[test]
    fn out_of_plane_never_above_tension_envelope_outside_range() {
        assert_eq!(kt_hole_bending_out_of_plane(0.5, 2.0), kt_hole_tension(0.5));
        assert_eq!(kt_hole_bending_out_of_plane(0.1, 0.5), kt_hole_tension(0.1));
        assert_eq!(kt_hole_bending_out_of_plane(0.1, 9.0), kt_hole_tension(0.1));
    }

    #[test]
    fn in_plane_small_hole_does_not_raise_outer_fibre() {
        assert_eq!(kt_hole_bending_in_plane(0.2), 1.0);
        assert!((kt_hole_bending_in_plane(0.7) - 1.4).abs() < 1e-12);
    }

    #[test]
    fn sharper_fillet_is_worse() {
        let mut prev = 0.0;
        for r in [5.0e-3, 3.0e-3, 2.0e-3, 1.0e-3, 0.6e-3] {
            let k = kt_fillet_tension(0.03, 0.02, r).unwrap();
            assert!(k > prev, "r = {r}: {k}");
            prev = k;
            assert!(kt_fillet_bending(0.03, 0.02, r).unwrap() > 1.0);
        }
    }

    #[test]
    fn fillet_ranges_join_without_jump() {
        // На границе h/r = 2 две половины аппроксимации не должны расходиться заметно.
        let (d, big) = (0.02, 0.03);
        let h = (big - d) / 2.0;
        let r = h / 2.0;
        let a = kt_fillet_tension(big, d, r * 1.0001).unwrap();
        let b = kt_fillet_tension(big, d, r * 0.9999).unwrap();
        assert!((a - b).abs() < 0.03, "{a} / {b}");
        let a = kt_fillet_bending(big, d, r * 1.0001).unwrap();
        let b = kt_fillet_bending(big, d, r * 0.9999).unwrap();
        assert!((a - b).abs() < 0.03, "{a} / {b}");
    }

    #[test]
    fn fillet_outside_data_is_none() {
        assert!(kt_fillet_tension(0.05, 0.01, 0.0001).is_none()); // h/r = 200
        assert!(kt_fillet_tension(0.01, 0.02, 0.001).is_none()); // ступень наоборот
        assert!(kt_fillet_tension(f64::NAN, 0.02, 0.001).is_none());
        assert!(kt_fillet_bending(0.03, 0.02, 0.0).is_none());
    }

    #[test]
    fn notch_sensitivity_bounds() {
        let (sut, r) = (400e6, 2e-3);
        let q = notch_sensitivity_steel(sut, r, false);
        assert!(q > 0.0 && q < 1.0);
        // Острый надрез чувствует меньше, но поведение монотонно по радиусу.
        assert!(notch_sensitivity_steel(sut, 4e-3, false) > q);
        // Вне диапазона — полная чувствительность.
        assert_eq!(notch_sensitivity_steel(100e6, r, false), 1.0);
        assert_eq!(notch_sensitivity_steel(sut, 0.0, false), 1.0);
        assert_eq!(notch_sensitivity_steel(sut, f64::NAN, false), 1.0);
    }

    #[test]
    fn basquin_passes_through_its_points() {
        let c = SnCurve::through(1e3, 300e6, 1e6, 150e6, true);
        assert!((c.strength_at(1e3) - 300e6).abs() < 1.0);
        assert!((c.strength_at(1e6) - 150e6).abs() < 1.0);
        assert_eq!(c.strength_at(1e9), 150e6);
        assert_eq!(c.life(100e6), f64::INFINITY);
        assert!(c.life_no_cutoff(100e6).is_finite());
        let n = c.life(200e6);
        assert!((c.strength_at(n) - 200e6).abs() < 1.0);
    }

    #[test]
    fn no_endurance_limit_keeps_falling() {
        let c = SnCurve::through(1e3, 250e6, 5e8, 96e6, false);
        assert!(c.strength_at(1e10) < 96e6);
        assert!(c.life(50e6).is_finite());
    }

    #[test]
    fn goodman_compression_gives_no_credit() {
        let n0 = goodman_factor(100e6, 0.0, 200e6, 400e6);
        let nc = goodman_factor(100e6, -150e6, 200e6, 400e6);
        assert_eq!(n0, nc);
        assert!((goodman_factor(50e6, 100e6, 200e6, 400e6) - 2.0).abs() < 1e-12);
    }

    #[test]
    fn miner_ignores_cutoff_and_detects_failure() {
        let c = SnCurve::through(1e3, 300e6, 1e6, 150e6, true);
        // Блок ниже предела выносливости всё равно даёт повреждение.
        let low = [StressBlock {
            sigma_a: 140e6,
            sigma_m: 0.0,
            cycles: 1e6,
        }];
        assert!(miner_damage(&c, 400e6, &low) > 0.0);
        // Среднее больше предела прочности — разрушение.
        let broken = [StressBlock {
            sigma_a: 1e6,
            sigma_m: 500e6,
            cycles: 1.0,
        }];
        assert_eq!(miner_damage(&c, 400e6, &broken), f64::INFINITY);
    }

    #[test]
    fn miner_factor_matches_goodman_for_one_block() {
        let c = SnCurve::through(1e3, 300e6, 1e6, 150e6, false);
        let block = [StressBlock {
            sigma_a: 50e6,
            sigma_m: 40e6,
            cycles: 1e5,
        }];
        let s = miner_stress_factor(&c, 400e6, &block);
        let n = goodman_factor(50e6, 40e6, c.strength_at(1e5), 400e6);
        assert!((s - n).abs() / n < 1e-6, "{s} против {n}");
    }

    #[test]
    fn marin_factors_are_not_bonuses() {
        for sut in [100.0, 260.0, 510.0, 895.0, 1500.0] {
            for s in [
                Surface::Ground,
                Surface::Machined,
                Surface::HotRolled,
                Surface::AsForged,
            ] {
                let k = surface_factor(s, sut);
                assert!(k > 0.0 && k <= 1.0, "{s:?} при {sut}: {k}");
            }
        }
        assert_eq!(size_factor(2e-3), 1.0);
        assert!(size_factor(0.03) < 1.0);
        assert!(size_factor(0.2) < size_factor(0.03));
    }
}
