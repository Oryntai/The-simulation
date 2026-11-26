//! Что контроллер знает о воде в бутылке: форма полости известна (в железе — по камере),
//! объём — по весам в руке. По ним считаются уровень, напор над кромкой горла, скорость
//! струи и куда она упадёт. Та же статика, что в процессе «жидкость» симулятора, но
//! коэффициент расхода — табличная оценка, а предел «бульканья» контроллеру неизвестен.

use serde::Deserialize;

use crate::geometry::V3;

const TILTS: usize = 180;
const BINS: usize = 1000;
const STEP_M: f64 = 0.0005;
const MOUTH: usize = 64;

#[derive(Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct Spec {
    /// Внутренний профиль от внутреннего дна: (высота, радиус), последняя точка — срез горла.
    pub profile: Vec<[f64; 2]>,
    /// Высота бутылки снаружи, радиус корпуса, от среза горла до центра захвата по оси.
    pub height: f64,
    pub radius: f64,
    pub mouth_above_grasp: f64,
    pub empty_kg: f64,
    pub density: f64,
    pub discharge: f64,
}

struct Row {
    s0: f64,
    ds: f64,
    cum: Vec<f64>,
}

pub struct Bottle {
    rows: Vec<Row>,
    height: f64,
    mouth: f64,
    discharge: f64,
}

pub struct Jet {
    pub speed: f64,
    pub rate: f64,
}

fn radius(profile: &[[f64; 2]], z: f64) -> f64 {
    let i = profile.partition_point(|p| p[0] <= z).clamp(1, profile.len() - 1);
    let ([z0, r0], [z1, r1]) = (profile[i - 1], profile[i]);
    r0 + (r1 - r0) * ((z - z0) / (z1 - z0)).clamp(0.0, 1.0)
}

impl Bottle {
    pub fn new(spec: &Spec) -> Self {
        let p = &spec.profile;
        let height = p.last().expect("профиль не пуст")[0];
        let nz = (height / STEP_M).ceil() as usize;
        let dz = height / nz as f64;
        let mut samples = Vec::new();
        for i in 0..nz {
            let z = (i as f64 + 0.5) * dz;
            let r = radius(p, z);
            let nx = ((2.0 * r / STEP_M).ceil() as usize).max(16);
            let dx = 2.0 * r / nx as f64;
            for j in 0..nx {
                let x = -r + (j as f64 + 0.5) * dx;
                samples.push((x, z, 2.0 * (r * r - x * x).max(0.0).sqrt() * dx * dz, dx));
            }
        }
        let rows = (0..=TILTS)
            .map(|k| {
                let (sin, cos) = (std::f64::consts::PI * k as f64 / TILTS as f64).sin_cos();
                let span = |&(x, z, _, dx): &(f64, f64, f64, f64)| {
                    let half = 0.5 * dx * sin.abs() + 0.5 * dz * cos.abs();
                    (x * sin + z * cos - half, x * sin + z * cos + half)
                };
                let (min, max) = samples.iter().map(span).fold((f64::MAX, f64::MIN), |(a, b), (lo, hi)| (a.min(lo), b.max(hi)));
                let ds = (max - min) / BINS as f64;
                let mut bins = vec![0.0; BINS];
                for sample in &samples {
                    let (lo, hi) = span(sample);
                    let (a, b) = ((lo - min) / ds, (hi - min) / ds);
                    let first = (a.floor().max(0.0) as usize).min(BINS - 1);
                    let last = (b.ceil() as usize).clamp(first + 1, BINS);
                    for (k, bin) in bins.iter_mut().enumerate().take(last).skip(first) {
                        *bin += sample.2 * (b.min(k as f64 + 1.0) - a.max(k as f64)).max(0.0) / (b - a);
                    }
                }
                let mut cum = vec![0.0];
                let mut acc = 0.0;
                for v in bins {
                    acc += v;
                    cum.push(acc);
                }
                Row { s0: min, ds, cum }
            })
            .collect();
        Self { rows, height, mouth: p.last().expect("профиль не пуст")[1], discharge: spec.discharge }
    }

    fn surface(&self, theta: f64, volume: f64) -> f64 {
        let t = (theta / std::f64::consts::PI * TILTS as f64).clamp(0.0, TILTS as f64);
        let k = (t.floor() as usize).min(TILTS - 1);
        let f = t - k as f64;
        let level = |row: &Row| {
            let full = row.cum[BINS];
            if volume >= full {
                return row.s0 + BINS as f64 * row.ds + (volume - full) / (std::f64::consts::PI * self.mouth * self.mouth);
            }
            let i = (row.cum.partition_point(|c| *c <= volume.max(0.0)) - 1).min(BINS - 1);
            let (lo, hi) = (row.cum[i], row.cum[i + 1]);
            let g = if hi > lo { (volume - lo) / (hi - lo) } else { 0.0 };
            row.s0 + (i as f64 + g) * row.ds
        };
        let (a, b) = (level(&self.rows[k]), level(&self.rows[k + 1]));
        a + f * (b - a)
    }

    /// Напор над нижней точкой кромки горла, м.
    pub fn head(&self, theta: f64, volume: f64) -> f64 {
        self.surface(theta, volume) - (self.height * theta.cos() - self.mouth * theta.sin())
    }

    /// Наклон оси от вертикали, при котором напор равен `head`.
    pub fn theta_for_head(&self, volume: f64, head: f64) -> f64 {
        let (mut lo, mut hi) = (0.0, 175f64.to_radians());
        if self.head(lo, volume) >= head {
            return lo;
        }
        for _ in 0..50 {
            let mid = 0.5 * (lo + hi);
            if self.head(mid, volume) < head {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    }

    /// Расход и средняя скорость струи через срез при наклоне `theta`.
    pub fn jet(&self, theta: f64, volume: f64, g: f64) -> Jet {
        let surface = self.surface(theta, volume);
        let (sin, cos) = theta.sin_cos();
        let r = self.mouth;
        let dx = 2.0 * r / MOUTH as f64;
        let (mut rate, mut area) = (0.0, 0.0);
        for j in 0..MOUTH {
            let x = -r + (j as f64 + 0.5) * dx;
            let head = surface - (x * sin + self.height * cos);
            if head > 0.0 {
                let da = 2.0 * (r * r - x * x).max(0.0).sqrt() * dx;
                rate += self.discharge * (2.0 * g * head).sqrt() * da;
                area += da;
            }
        }
        Jet { speed: if area > 0.0 { rate / (self.discharge * area) } else { 0.0 }, rate }
    }
}

/// Смещение по горизонтали точки падения струи со скоростью `v` с высоты `height` над плоскостью.
pub fn landing(v: V3, height: f64, g: f64) -> V3 {
    let t = (v[2] + (v[2] * v[2] + 2.0 * g * height.max(0.0)).sqrt()) / g;
    [v[0] * t, v[1] * t, 0.0]
}
