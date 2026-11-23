//! Усталость мышц: трёхкамерная модель (3CC) и кривые выносливости.
//!
//! Камеры: MA — работающие волокна, MF — уставшие, MR — отдыхающие, MA + MF + MR = 1.
//! Xia & Frey-Law, 2008, J Biomech 41:3046; уравнения и параметры по суставам — Frey-Law, Looft,
//! Heitsman, 2012, J Biomech 45:1803, приложение и табл. 1:
//!   dMR/dt = −C + R·MF,  dMA/dt = C − F·MA,  dMF/dt = F·MA − R·MF,
//!   C = L·(TL − MA), если MA ≥ TL или хватает резерва; иначе C = L·MR.
//! Время удержания — момент, когда MA + MR < TL.

use crate::data::*;

/// Сустав для параметров усталости.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FatigueJoint {
    Shoulder,
    Elbow,
    Trunk,
}

impl FatigueJoint {
    pub const ALL: [Self; 3] = [Self::Shoulder, Self::Elbow, Self::Trunk];

    pub fn params(self) -> ThreeCcParams {
        match self {
            Self::Shoulder => TCC_SHOULDER,
            Self::Elbow => TCC_ELBOW,
            Self::Trunk => TCC_TRUNK,
        }
    }

    pub fn endurance_fit(self) -> EnduranceFit {
        match self {
            Self::Shoulder => ET_SHOULDER,
            Self::Elbow => ET_ELBOW,
            Self::Trunk => ET_TRUNK,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Shoulder => "плечо",
            Self::Elbow => "локоть",
            Self::Trunk => "поясница",
        }
    }
}

/// Состояние трёх камер (доли от полной силы).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThreeCcState {
    pub ma: f64,
    pub mf: f64,
    pub mr: f64,
}

impl Default for ThreeCcState {
    fn default() -> Self {
        Self {
            ma: 0.0,
            mf: 0.0,
            mr: 1.0,
        }
    }
}

fn derivative(s: ThreeCcState, target: f64, p: ThreeCcParams) -> ThreeCcState {
    let l = TCC_TRACKING_L;
    let c = l * (target - s.ma).min(s.mr);
    ThreeCcState {
        ma: c - p.f * s.ma,
        mf: p.f * s.ma - p.r * s.mf,
        mr: -c + p.r * s.mf,
    }
}

/// Шаг Рунге–Кутты 4-го порядка.
pub fn step(s: ThreeCcState, target: f64, p: ThreeCcParams, dt: f64) -> ThreeCcState {
    let add = |a: ThreeCcState, b: ThreeCcState, k: f64| ThreeCcState {
        ma: a.ma + k * b.ma,
        mf: a.mf + k * b.mf,
        mr: a.mr + k * b.mr,
    };
    let k1 = derivative(s, target, p);
    let k2 = derivative(add(s, k1, 0.5 * dt), target, p);
    let k3 = derivative(add(s, k2, 0.5 * dt), target, p);
    let k4 = derivative(add(s, k3, dt), target, p);
    ThreeCcState {
        ma: s.ma + dt / 6.0 * (k1.ma + 2.0 * k2.ma + 2.0 * k3.ma + k4.ma),
        mf: s.mf + dt / 6.0 * (k1.mf + 2.0 * k2.mf + 2.0 * k3.mf + k4.mf),
        mr: s.mr + dt / 6.0 * (k1.mr + 2.0 * k2.mr + 2.0 * k3.mr + k4.mr),
    }
}

/// Уровень, ниже которого усталость в 3CC не исчерпывает резерв при постоянном TL.
pub fn sustainable_intensity(p: ThreeCcParams) -> f64 {
    let tracking_ratio = TCC_TRACKING_L / (TCC_TRACKING_L + p.f);
    p.r / (p.r + p.f * tracking_ratio)
}

/// Решение трёх уравнений до первого пересечения MA + MR = TL.
/// До пересечения C = L(TL − MA); MA = (L·TL/k)(1 − e^(−kt)), k = L + F.
/// Решение dMF/dt + R·MF = F·MA подставляется в условие исчерпания резерва.
pub fn endurance_time_3cc(target: f64, p: ThreeCcParams) -> Option<f64> {
    if !target.is_finite() || target >= 1.0 {
        return Some(0.0);
    }
    if target <= 0.0 || target <= sustainable_intensity(p) {
        return None;
    }
    let k = TCC_TRACKING_L + p.f;
    let active_steady = TCC_TRACKING_L * target / k;
    let fatigue = |t: f64| {
        let build = -(-p.r * t).exp_m1() / p.r;
        let tracking = (-p.r * t).exp() * -(-(k - p.r) * t).exp_m1() / (k - p.r);
        p.f * active_steady * (build - tracking)
    };
    let mut lo = 0.0;
    // Численная скобка: за 64 постоянные восстановления экспонента ниже точности f64.
    let mut hi = 64.0 / p.r;
    let limit = 1.0 - target;
    if fatigue(hi) < limit {
        return None;
    }
    // Численный допуск по времени менее одного микросекундного шага.
    while hi - lo > 1e-6 {
        let mid = 0.5 * (lo + hi);
        if fatigue(mid) >= limit {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    Some(lo)
}

/// Независимый прогон Рунге–Кутты для эталонных сверок на опубликованных интенсивностях.
pub fn endurance_time_rk4(target: f64, p: ThreeCcParams) -> Option<f64> {
    // Численные шаг и бюджет сверки. Шаг удовлетворяет L·dt < 1.
    let dt = 0.02;
    let mut state = ThreeCcState::default();
    for i in 1..=100_000 {
        state = step(state, target, p, dt);
        if state.ma + state.mr <= target {
            return Some(i as f64 * dt);
        }
    }
    None
}

/// Степенная кривая выносливости Frey-Law & Avin 2010, с.
pub fn endurance_time_power(target: f64, fit: EnduranceFit) -> f64 {
    if !target.is_finite() || target >= 1.0 {
        return 0.0;
    }
    if target <= 0.0 {
        return f64::INFINITY;
    }
    fit.b0 * target.powf(fit.b1)
}

/// Кривая Rohmert (1960), мин → с: T = −1.5 + 2.1/f − 0.6/f² + 0.1/f³.
/// Формула по учебному пособию USF (цитирует Rohmert через Müller; там же множитель 0.9 для
/// 40–100% MVC назван произвольным, поэтому здесь его нет; используем только этот диапазон).
pub fn endurance_time_rohmert(target: f64) -> Option<f64> {
    if !target.is_finite() || !(0.40..=1.0).contains(&target) {
        return None;
    }
    let f = target;
    // Та же формула, с вынесенным (1−f) для устойчивости вблизи MVC.
    Some(60.0 * (1.0 - f) * (1.5 * f * f - 0.6 * f + 0.1) / f.powi(3))
}

/// Вывод по усталости одного сустава при удержании.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EnduranceVerdict {
    pub joint: FatigueJoint,
    pub intensity: f64,
    /// Наименьшая из оценок времени удержания, с (∞ — предела не найдено).
    pub limit_s: f64,
    pub time_3cc_s: Option<f64>,
    pub time_power_s: f64,
    pub time_rohmert_s: Option<f64>,
    /// Интенсивность ниже диапазона подбора — кривая экстраполирована.
    pub extrapolated: bool,
}

/// Оценка времени удержания сустава: меньшее из 3CC и кривой выносливости.
pub fn endurance(joint: FatigueJoint, intensity: f64) -> EnduranceVerdict {
    let time_3cc_s = endurance_time_3cc(intensity, joint.params());
    let time_power_s = endurance_time_power(intensity, joint.endurance_fit());
    let time_rohmert_s = endurance_time_rohmert(intensity);
    let limit_s = [time_3cc_s, time_rohmert_s]
        .into_iter()
        .flatten()
        .fold(time_power_s, f64::min);
    EnduranceVerdict {
        joint,
        intensity,
        limit_s,
        time_3cc_s,
        time_power_s,
        time_rohmert_s,
        extrapolated: intensity < ET_FIT_MIN_INTENSITY,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compartments_sum_to_one() {
        let p = TCC_SHOULDER;
        let mut s = ThreeCcState::default();
        for _ in 0..5000 {
            s = step(s, 0.4, p, 0.02);
            assert!((s.ma + s.mf + s.mr - 1.0).abs() < 1e-9);
        }
    }

    #[test]
    fn numeric_time_matches_closed_form() {
        for joint in FatigueJoint::ALL {
            for f in [0.2, 0.4, 0.6, 0.8] {
                let a = endurance_time_3cc(f, joint.params()).unwrap();
                let n = endurance_time_rk4(f, joint.params()).unwrap();
                assert!((a - n).abs() < 0.02, "{joint:?} {f}: {a} / {n}");
            }
        }
    }

    #[test]
    fn below_sustainable_level_never_fails_in_3cc() {
        let p = TCC_ELBOW;
        assert!(endurance_time_3cc(sustainable_intensity(p) * 0.9, p).is_none());
    }

    #[test]
    fn broken_input_gives_zero_time() {
        assert_eq!(endurance_time_power(f64::NAN, ET_ELBOW), 0.0);
        assert_eq!(endurance_time_3cc(f64::INFINITY, TCC_ELBOW), Some(0.0));
    }

    #[test]
    fn rohmert_applicability() {
        assert!(endurance_time_rohmert(0.1).is_none());
        assert!(endurance_time_rohmert(0.5).unwrap() > 0.0);
        assert!(endurance_time_rohmert(f64::NAN).is_none());
    }
}
