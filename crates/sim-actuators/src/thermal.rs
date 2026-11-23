//! Периодический тепловой режим двухузловой сети «обмотка → корпус → среда».
//!
//! На каждом участке цикла ток постоянен, а потери в меди I²·R₀·(1 + α(T − T₀)) линейны
//! по температуре обмотки. Значит, система на участке линейная и решается точно через
//! матричную экспоненту, а установившийся цикл — неподвижная точка x = Φ·x + γ.
//! Многократное интегрирование по циклам не нужно, и длинная постоянная времени корпуса
//! не мешает.

use crate::spec::ALPHA_CU;

type M3 = [[f64; 3]; 3];

fn mul(a: &M3, b: &M3) -> M3 {
    let mut r = [[0.0; 3]; 3];
    for (i, row) in r.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            *cell = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    r
}

/// Матричная экспонента: масштабирование, ряд Тейлора, возведение в квадрат.
fn expm3(m: &M3) -> M3 {
    if !m.iter().flatten().all(|v| v.is_finite()) {
        return [[f64::NAN; 3]; 3];
    }
    let norm = m
        .iter()
        .map(|r| r.iter().map(|x| x.abs()).sum::<f64>())
        .fold(0.0, f64::max);
    if !norm.is_finite() {
        return [[f64::NAN; 3]; 3];
    }
    // Масштабирование ряда около нуля. Логарифм разности степеней
    // избегает переполнения при делении большого norm на 0.25.
    let s = if norm > 0.25 {
        (norm.log2() + 2.0).ceil() as u32
    } else {
        0
    };
    let scale = 0.5f64.powi(s as i32);
    let mut a = *m;
    for row in &mut a {
        for x in row {
            *x *= scale;
        }
    }
    let mut result: M3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut term = result;
    for k in 1..=16 {
        term = mul(&term, &a);
        for row in &mut term {
            for x in row {
                *x /= k as f64;
            }
        }
        for (rr, tr) in result.iter_mut().zip(term.iter()) {
            for (x, t) in rr.iter_mut().zip(tr.iter()) {
                *x += t;
            }
        }
    }
    for _ in 0..s {
        result = mul(&result, &result);
    }
    result
}

/// Тепловая сеть: сопротивления К/Вт, ёмкости Дж/К.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThermalNet {
    pub r_wh: f64,
    pub r_ha: f64,
    pub c_w: f64,
    pub c_h: f64,
}

impl ThermalNet {
    /// По постоянным времени паспорта C = τ/R (трактовка maxon, как в `sim-em`).
    pub fn from_datasheet(r_wh: f64, r_ha: f64, tau_w: f64, tau_h: f64) -> Self {
        Self {
            r_wh,
            r_ha,
            c_w: tau_w / r_wh,
            c_h: tau_h / r_ha,
        }
    }
}

/// Участок цикла с точки зрения нагрева.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeatSegment {
    pub duration_s: f64,
    /// I²·R при опорной температуре, Вт (с учётом числа греющихся фаз).
    pub copper_w_at_ref: f64,
    /// Потери холостого хода в обмотке, Вт.
    pub other_winding_w: f64,
    /// Потери холостого хода в корпусе, Вт.
    pub other_housing_w: f64,
}

fn segment_matrix(net: &ThermalNet, s: &HeatSegment, t_amb: f64, t_ref: f64, dt: f64) -> M3 {
    let a_ww = (-1.0 / net.r_wh + s.copper_w_at_ref * ALPHA_CU) / net.c_w;
    let a_wh = 1.0 / (net.r_wh * net.c_w);
    let a_hw = 1.0 / (net.r_wh * net.c_h);
    let a_hh = -(1.0 / net.r_wh + 1.0 / net.r_ha) / net.c_h;
    let b_w = (s.copper_w_at_ref * (1.0 + ALPHA_CU * (t_amb - t_ref).max(0.0)) + s.other_winding_w)
        / net.c_w;
    let b_h = s.other_housing_w / net.c_h;
    expm3(&[
        [a_ww * dt, a_wh * dt, b_w * dt],
        [a_hw * dt, a_hh * dt, b_h * dt],
        [0.0, 0.0, 0.0],
    ])
}

fn state_at(
    net: &ThermalNet,
    s: &HeatSegment,
    t_amb: f64,
    t_ref: f64,
    dt: f64,
    x: [f64; 3],
) -> Option<[f64; 3]> {
    let m = segment_matrix(net, s, t_amb, t_ref, dt);
    let result = m.map(|row| row.iter().zip(x).map(|(a, b)| a * b).sum::<f64>());
    result.into_iter().all(f64::is_finite).then_some(result)
}

/// Наибольшая температура обмотки в установившемся цикле, °C.
/// `None` — теплового равновесия нет (тепловой разгон) или вход некорректен.
pub fn periodic_peak_winding_c(
    net: &ThermalNet,
    segments: &[HeatSegment],
    t_amb: f64,
    t_ref: f64,
) -> Option<f64> {
    if segments.is_empty()
        || ![net.r_wh, net.r_ha, net.c_w, net.c_h]
            .into_iter()
            .all(|v| v.is_finite() && v > 0.0)
        || !t_amb.is_finite()
        || !t_ref.is_finite()
        || segments.iter().any(|s| {
            ![
                s.duration_s,
                s.copper_w_at_ref,
                s.other_winding_w,
                s.other_housing_w,
            ]
            .into_iter()
            .all(|v| v.is_finite() && v >= 0.0)
        })
    {
        return None;
    }
    // Отображение за цикл: x_конец = Φ·x_начало + γ.
    let mut cycle: M3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for s in segments {
        if s.duration_s > 0.0 {
            cycle = mul(&segment_matrix(net, s, t_amb, t_ref, s.duration_s), &cycle);
        }
    }
    let (a, b, c, d) = (cycle[0][0], cycle[0][1], cycle[1][0], cycle[1][1]);
    let (tr, det) = (a + d, a * d - b * c);
    // Условие Джури: собственные числа Φ внутри единичного круга.
    if !(det < 1.0 && tr.abs() < 1.0 + det) {
        return None;
    }
    let (m11, m12, m21, m22) = (1.0 - a, -b, -c, 1.0 - d);
    let den = m11 * m22 - m12 * m21;
    if !(den.is_finite() && den > 0.0) {
        return None;
    }
    let (g1, g2) = (cycle[0][2], cycle[1][2]);
    let mut x = [
        (m22 * g1 - m12 * g2) / den,
        (m11 * g2 - m21 * g1) / den,
        1.0,
    ];
    if !x.into_iter().all(f64::is_finite) {
        return None;
    }
    let mut peak = x[0];
    for s in segments {
        if s.duration_s <= 0.0 {
            continue;
        }
        let end = state_at(net, s, t_amb, t_ref, s.duration_s, x)?;
        let slope = |[w, h, _]: [f64; 3]| {
            s.copper_w_at_ref * (1.0 + ALPHA_CU * ((t_amb - t_ref).max(0.0) + w))
                + s.other_winding_w
                - (w - h) / net.r_wh
        };
        let (left, right) = (slope(x), slope(end));
        if !(left.is_finite() && right.is_finite()) {
            return None;
        }
        // Производная является суммой двух действительных экспонент:
        // у неё не более одного изолированного нуля. Ищем максимум внутри участка.
        if left > 0.0 && right < 0.0 {
            let (mut lo, mut hi) = (0.0, s.duration_s);
            // До точности мантиссы f64 в относительных координатах участка.
            for _ in 0..f64::MANTISSA_DIGITS {
                let mid = lo + (hi - lo) * 0.5;
                let at = state_at(net, s, t_amb, t_ref, mid, x)?;
                let derivative = slope(at);
                if !derivative.is_finite() {
                    return None;
                }
                peak = peak.max(at[0]);
                if derivative > 0.0 {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
        }
        peak = peak.max(end[0]);
        x = end;
    }
    let t = t_amb + peak;
    t.is_finite().then_some(t)
}

/// Установившаяся температура обмотки при постоянных потерях с учётом R(T), одна ступень:
/// T = (T_a + k·(1 − α·T₀))/(1 − k·α), k = P₀·R_th. `None` — тепловой разгон.
pub fn steady_winding_c(copper_w_at_ref: f64, r_th: f64, t_amb: f64, t_ref: f64) -> Option<f64> {
    let k = copper_w_at_ref * r_th;
    let den = 1.0 - k * ALPHA_CU;
    if den <= 0.0 {
        return None;
    }
    let t = t_amb + k * (1.0 + ALPHA_CU * (t_amb - t_ref).max(0.0)) / den;
    t.is_finite().then_some(t)
}
