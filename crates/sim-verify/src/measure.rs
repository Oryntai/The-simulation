//! Измерения на выборке из симуляции (как осциллограф на стенде).

/// Детектор пересечений нуля снизу вверх. Момент пересечения уточняется кубическим
/// эрмитовым интерполянтом по значению и производной на концах шага, поэтому ошибка
/// измерения периода — O(dt⁴), а не O(dt), как при поиске «ближайшего отсчёта».
#[derive(Clone, Debug, Default)]
pub struct CrossingDetector {
    prev: Option<(f64, f64, f64)>,
    crossings: Vec<f64>,
}

impl CrossingDetector {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, t: f64, x: f64, xd: f64) {
        if let Some((t0, x0, xd0)) = self.prev
            && x0 < 0.0
            && x >= 0.0
        {
            let h = t - t0;
            let hermite = |s: f64| {
                let s2 = s * s;
                let s3 = s2 * s;
                (2.0 * s3 - 3.0 * s2 + 1.0) * x0
                    + (s3 - 2.0 * s2 + s) * h * xd0
                    + (-2.0 * s3 + 3.0 * s2) * x
                    + (s3 - s2) * h * xd
            };
            let (mut lo, mut hi) = (0.0, 1.0);
            for _ in 0..80 {
                let mid = 0.5 * (lo + hi);
                if hermite(mid) < 0.0 {
                    lo = mid
                } else {
                    hi = mid
                }
            }
            self.crossings.push(t0 + 0.5 * (lo + hi) * h);
        }
        self.prev = Some((t, x, xd));
    }

    pub fn crossings(&self) -> &[f64] {
        &self.crossings
    }

    /// Средний период по всем полным циклам.
    pub fn mean_period(&self) -> Option<f64> {
        let c = &self.crossings;
        if c.len() < 2 {
            return None;
        }
        Some((c[c.len() - 1] - c[0]) / (c.len() - 1) as f64)
    }

    /// Период по последним `n` циклам.
    pub fn recent_period(&self, n: usize) -> Option<f64> {
        let c = &self.crossings;
        if c.len() < 2 {
            return None;
        }
        let k = n.min(c.len() - 1);
        Some((c[c.len() - 1] - c[c.len() - 1 - k]) / k as f64)
    }

    pub fn clear(&mut self) {
        self.prev = None;
        self.crossings.clear();
    }
}

/// Момент, когда сигнал впервые опустился до уровня `level` (сверху вниз), с эрмитовой интерполяцией.
pub fn hermite_level_crossing(
    t0: f64,
    x0: f64,
    xd0: f64,
    t1: f64,
    x1: f64,
    xd1: f64,
    level: f64,
) -> f64 {
    let mut det = CrossingDetector::new();
    // Переворачиваем: ищем пересечение (level − x) снизу вверх.
    det.push(t0, level - x0, -xd0);
    det.push(t1, level - x1, -xd1);
    det.crossings().first().copied().unwrap_or(t1)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sine_period_is_measured_precisely() {
        let w = 2.0 * std::f64::consts::PI / 1.7;
        let mut d = CrossingDetector::new();
        let dt = 0.01;
        for i in 0..2000 {
            let t = i as f64 * dt;
            d.push(t, (w * t + 0.3).sin(), w * (w * t + 0.3).cos());
        }
        assert!((d.mean_period().unwrap() - 1.7).abs() < 1e-8);
    }
}
