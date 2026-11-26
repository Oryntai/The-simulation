//! Кинематика кронштейна: прямая задача и обратная для сферического запястья.

use serde::Deserialize;

pub type V3 = [f64; 3];
/// Поворот: строки матрицы; столбцы — оси подвижной системы в мире.
pub type Mat = [[f64; 3]; 3];

pub fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
pub fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub fn scale(a: V3, k: f64) -> V3 {
    [a[0] * k, a[1] * k, a[2] * k]
}
pub fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
pub fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}
pub fn unit(a: V3) -> V3 {
    scale(a, 1.0 / norm(a))
}
pub fn matmul(a: &Mat, b: &Mat) -> Mat {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| a[i][k] * b[k][j]).sum()))
}
pub fn transpose(a: &Mat) -> Mat {
    std::array::from_fn(|i| std::array::from_fn(|j| a[j][i]))
}
pub fn apply(r: &Mat, v: V3) -> V3 {
    std::array::from_fn(|i| dot(r[i], v))
}
fn rz(t: f64) -> Mat {
    let (s, c) = t.sin_cos();
    [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
}
fn ry(t: f64) -> Mat {
    let (s, c) = t.sin_cos();
    [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]
}
/// Поворот на угол `t` вокруг единичной оси `k` (Родриг).
pub fn rot(k: V3, t: f64) -> Mat {
    let (s, c) = t.sin_cos();
    let v = 1.0 - c;
    let [x, y, z] = k;
    [
        [c + x * x * v, x * y * v - z * s, x * z * v + y * s],
        [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
        [z * x * v - y * s, z * y * v + x * s, c + z * z * v],
    ]
}
/// Ось и угол поворота.
pub fn angle_axis(r: &Mat) -> (V3, f64) {
    let c = ((r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0).clamp(-1.0, 1.0);
    let angle = c.acos();
    let v = [r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]];
    if norm(v) < 1e-12 {
        if angle < 1e-6 {
            return ([0.0, 0.0, 1.0], 0.0);
        }
        // Поворот на π: ось — столбец (R + I) с наибольшей нормой.
        let cols: Vec<V3> = (0..3).map(|j| std::array::from_fn(|i| r[i][j] + if i == j { 1.0 } else { 0.0 })).collect();
        let best = cols.into_iter().max_by(|a, b| norm(*a).total_cmp(&norm(*b))).expect("три столбца");
        return (unit(best), angle);
    }
    (unit(v), angle)
}
/// Поворот ладони по оси подхода (z) и сжатия (x); y = z × x.
pub fn frame(approach: V3, closing: V3) -> Mat {
    let z = unit(approach);
    let x = unit(sub(closing, scale(z, dot(closing, z))));
    let y = cross(z, x);
    std::array::from_fn(|i| [x[i], y[i], z[i]])
}
pub fn min_jerk(s: f64) -> f64 {
    let s = s.clamp(0.0, 1.0);
    s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)
}

#[derive(Deserialize, Clone, Copy)]
#[serde(deny_unknown_fields)]
pub struct Geometry {
    pub h0: f64,
    pub l1: f64,
    pub e_roll: f64,
    pub l2b: f64,
    pub tcp: f64,
    pub limits_deg: [[f64; 2]; 6],
}

impl Geometry {
    pub fn forward(&self, q: &[f64; 6]) -> (V3, Mat) {
        let mut r = rz(q[0]);
        let mut p = [0.0, 0.0, self.h0];
        r = matmul(&r, &ry(q[1]));
        p = add(p, apply(&r, [0.0, 0.0, self.l1]));
        r = matmul(&r, &ry(q[2]));
        p = add(p, apply(&r, [0.0, 0.0, self.e_roll]));
        r = matmul(&r, &rz(q[3]));
        p = add(p, apply(&r, [0.0, 0.0, self.l2b]));
        r = matmul(&r, &ry(q[4]));
        r = matmul(&r, &rz(q[5]));
        p = add(p, apply(&r, [0.0, 0.0, self.tcp]));
        (p, r)
    }

    fn within(&self, q: &[f64; 6]) -> bool {
        q.iter().zip(self.limits_deg).all(|(v, [lo, hi])| (lo.to_radians() - 1e-9..=hi.to_radians() + 1e-9).contains(v))
    }

    /// Углы для центра захвата `p` и поворота ладони `r`; ветвь запястья ближе к `near`.
    pub fn inverse(&self, p: V3, r: &Mat, near: &[f64; 6]) -> Result<[f64; 6], String> {
        let z = [r[0][2], r[1][2], r[2][2]];
        let w = sub(p, scale(z, self.tcp));
        let q1 = w[1].atan2(w[0]);
        let rr = w[0].hypot(w[1]);
        let zz = w[2] - self.h0;
        let l2 = self.e_roll + self.l2b;
        let c3 = (rr * rr + zz * zz - self.l1 * self.l1 - l2 * l2) / (2.0 * self.l1 * l2);
        if c3.abs() > 1.0 {
            return Err(format!("центр запястья {:?} вне досягаемости", w.map(|c| (c * 1000.0).round())));
        }
        let q3 = c3.acos();
        let q2 = rr.atan2(zz) - (l2 * q3.sin()).atan2(self.l1 + l2 * q3.cos());
        let r03 = matmul(&rz(q1), &ry(q2 + q3));
        let m = matmul(&transpose(&r03), r);
        let s5 = m[0][2].hypot(m[1][2]);
        let mut best: Option<([f64; 6], f64)> = None;
        for sign in [1.0, -1.0] {
            let q5 = (sign * s5).atan2(m[2][2]);
            let (q4, q6) = if s5 < 1e-9 {
                (near[3], m[1][0].atan2(m[0][0]) - near[3])
            } else {
                ((sign * m[1][2]).atan2(sign * m[0][2]), (sign * m[2][1]).atan2(-sign * m[2][0]))
            };
            // Угол вращения выбирается среди равных по модулю 2π ближе к прошлому.
            let wrap = |v: f64, near: f64| v + ((near - v) / std::f64::consts::TAU).round() * std::f64::consts::TAU;
            let q = [q1, q2, q3, wrap(q4, near[3]), q5, wrap(q6, near[5])];
            if !self.within(&q) {
                continue;
            }
            let cost: f64 = q.iter().zip(near).map(|(a, b)| (a - b).powi(2)).sum();
            if best.is_none_or(|(_, c)| cost < c) {
                best = Some((q, cost));
            }
        }
        best.map(|(q, _)| q).ok_or_else(|| format!("поза TCP {:?} вне пределов суставов", p.map(|c| (c * 1000.0).round())))
    }
}

/// Звено цепи от поворота до губы: начало в родителе, ось сустава (если есть), масса и центр.
#[derive(Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct Link {
    pub pos: V3,
    pub axis: Option<String>,
    pub mass: f64,
    pub com: V3,
}

/// Массы звеньев и податливость суставов: упреждение просадки под весом.
#[derive(Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct ArmModel {
    pub chain: Vec<Link>,
    pub stiffness: [f64; 6],
    pub half_gap: [f64; 6],
}

impl ArmModel {
    /// Моменты тяжести на суставы при углах `q` и грузе в центре захвата.
    pub fn gravity(&self, q: &[f64; 6], payload: f64, geometry: &Geometry) -> [f64; 6] {
        let g = [0.0, 0.0, -9.80665];
        let mut p = [0.0; 3];
        let mut r: Mat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut axes = Vec::new();
        let mut bodies = Vec::new();
        for link in &self.chain {
            p = add(p, apply(&r, link.pos));
            if let Some(axis) = &link.axis {
                let k = axes.len();
                let (local, turn) = if axis == "z" { ([0.0, 0.0, 1.0], rz(q[k])) } else { ([0.0, 1.0, 0.0], ry(q[k])) };
                axes.push((p, apply(&r, local)));
                r = matmul(&r, &turn);
            }
            bodies.push((axes.len().saturating_sub(1), link.mass, add(p, apply(&r, link.com))));
        }
        let (tcp, _) = geometry.forward(q);
        bodies.push((5, payload, tcp));
        std::array::from_fn(|k| {
            let (pj, axis) = axes[k];
            bodies
                .iter()
                .filter(|(owner, _, _)| *owner >= k)
                .map(|&(_, m, c)| dot(cross(sub(c, pj), scale(g, m)), axis))
                .sum()
        })
    }

    /// Задание мотору, при котором выход под весом приходит в `q`: минус скручивание и плавный полу-люфт.
    pub fn compensate(&self, q: &[f64; 6], payload: f64, geometry: &Geometry) -> [f64; 6] {
        let tau = self.gravity(q, payload, geometry);
        std::array::from_fn(|i| q[i] - tau[i] / self.stiffness[i] - self.half_gap[i] * (tau[i] / 0.05).tanh())
    }
}
