//! Зрение кронштейна: камера глубины D405 на ладони.
//!
//! Рука встаёт над столом камерой вниз и копит несколько кадров. Каждый пиксель — луч из оптического
//! центра; поза камеры — прямая кинематика по энкодерам и номинальное крепление на ладони. Ошибка
//! крепления экземпляра контроллеру неизвестна, смещение глубины — тоже: его снимает стол, высота
//! которого известна (z = 0). Дальше точки над столом делятся на связные куски кадра. Бутылку
//! узнаём по высоте среза горла, кружку — по высоте края. Центр бутылки — середина среза горла,
//! центр и радиус кружки — окружность по точкам края.

use serde::Deserialize;

use crate::geometry::{Mat, V3, add, apply, scale, sub, transpose};

/// Внутренние параметры и крепление камеры, как их знает контроллер (номинал, не экземпляр).
#[derive(Deserialize, Clone, Copy)]
#[serde(deny_unknown_fields)]
pub struct Camera {
    pub width: usize,
    pub height: usize,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    /// Оптический центр в осях ладони относительно центра захвата, м. Оси кадра — оси ладони.
    pub mount: V3,
    /// Паспортная ближняя дальность: ближе глубины нет.
    pub min_m: f64,
    pub max_m: f64,
}

/// Что известно о предметах заранее: высота среза горла бутылки и высота кружки в пределах.
#[derive(Deserialize, Clone, Copy)]
#[serde(deny_unknown_fields)]
pub struct Expect {
    pub bottle_top: f64,
    pub mug_top: [f64; 2],
}

#[derive(Clone, Copy, Debug)]
pub struct Found {
    /// Ось бутылки на столе.
    pub bottle: [f64; 2],
    pub bottle_top: f64,
    pub mug_center: [f64; 2],
    pub mug_rim: f64,
    /// Внутренний радиус кружки по краю.
    pub mug_radius: f64,
    pub mug_outer: f64,
    /// Смещение глубины, снятое по столу, м.
    pub bias: f64,
    pub table_points: usize,
}

/// Среднее нескольких кадров глубины по пикселям; дыры (ноль) не считаются.
pub struct Stack {
    sum: Vec<f64>,
    count: Vec<u16>,
    pub frames: usize,
    pub last_captured: u64,
}

impl Stack {
    pub fn new(cam: &Camera) -> Self {
        Self { sum: vec![0.0; cam.width * cam.height], count: vec![0; cam.width * cam.height], frames: 0, last_captured: 0 }
    }

    /// Добавить кадр DepthMm16 (u16 little-endian, мм; ноль — дыра).
    pub fn add(&mut self, data: &[u8], captured: u64) -> Result<(), String> {
        if data.len() != 2 * self.sum.len() {
            return Err(format!("кадр {} байт, ожидалось {}", data.len(), 2 * self.sum.len()));
        }
        for (i, px) in data.as_chunks::<2>().0.iter().enumerate() {
            let mm = u16::from_le_bytes(*px);
            if mm != 0 {
                self.sum[i] += f64::from(mm) / 1000.0;
                self.count[i] += 1;
            }
        }
        self.frames += 1;
        self.last_captured = captured;
        Ok(())
    }

    /// Средняя глубина как картинка PGM: ближе — светлее, дыры чёрные. Для отладки.
    pub fn pgm(&self, cam: &Camera) -> Vec<u8> {
        let mut out = format!("P5\n{} {}\n255\n", cam.width, cam.height).into_bytes();
        out.extend((0..self.sum.len()).map(|i| {
            self.depth(i).map_or(0, |d| (255.0 * (1.0 - (d - 0.1) / 0.5)).clamp(1.0, 255.0) as u8)
        }));
        out
    }

    fn depth(&self, i: usize) -> Option<f64> {
        // Пиксель годен, если глубина была хотя бы в половине кадров.
        (2 * usize::from(self.count[i]) >= self.frames.max(1)).then(|| self.sum[i] / f64::from(self.count[i]))
    }
}

fn percentile(values: &mut [f64], q: f64) -> f64 {
    values.sort_by(f64::total_cmp);
    let i = ((values.len() - 1) as f64 * q).round() as usize;
    values[i]
}

/// Окружность по точкам (алгебраическая, Каса): центр и радиус.
fn circle(points: &[[f64; 2]]) -> Option<([f64; 2], f64)> {
    // Центрирование для устойчивости.
    let n = points.len() as f64;
    let (mx, my) = points.iter().fold((0.0, 0.0), |(a, b), p| (a + p[0] / n, b + p[1] / n));
    let mut m = [[0.0; 3]; 3];
    let mut v = [0.0; 3];
    for p in points {
        let (x, y) = (p[0] - mx, p[1] - my);
        let row = [x, y, 1.0];
        let rhs = -(x * x + y * y);
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] += row[i] * row[j];
            }
            v[i] += row[i] * rhs;
        }
    }
    let s = solve3(m, v)?;
    let (cx, cy) = (-s[0] / 2.0, -s[1] / 2.0);
    let r2 = cx * cx + cy * cy - s[2];
    (r2 > 0.0).then(|| ([cx + mx, cy + my], r2.sqrt()))
}

fn solve3(mut a: [[f64; 3]; 3], mut b: [f64; 3]) -> Option<[f64; 3]> {
    for c in 0..3 {
        let p = (c..3).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if a[p][c].abs() < 1e-18 {
            return None;
        }
        a.swap(c, p);
        b.swap(c, p);
        for r in c + 1..3 {
            let k = a[r][c] / a[c][c];
            for j in c..3 {
                a[r][j] -= k * a[c][j];
            }
            b[r] -= k * b[c];
        }
    }
    let mut x = [0.0; 3];
    for r in (0..3).rev() {
        x[r] = (b[r] - (r + 1..3).map(|j| a[r][j] * x[j]).sum::<f64>()) / a[r][r];
    }
    Some(x)
}

/// Найти бутылку и кружку по накопленным кадрам. `tcp`, `r` — поза центра захвата по энкодерам.
pub fn find(stack: &Stack, cam: &Camera, expect: &Expect, tcp: V3, r: &Mat) -> Result<Found, String> {
    let origin = add(tcp, apply(r, cam.mount));
    let (w, h) = (cam.width, cam.height);
    let ray = |i: usize| -> V3 {
        let (u, v) = ((i % w) as f64 + 0.5, (i / w) as f64 + 0.5);
        apply(r, [(u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy, 1.0])
    };
    // Смещение глубины по столу: на луче стола глубина известна, медиана разниц — смещение.
    let mut residuals = Vec::new();
    for i in 0..w * h {
        let Some(d) = stack.depth(i) else { continue };
        let dir = ray(i);
        if dir[2] < -0.05 {
            let table = -origin[2] / dir[2];
            let e = d - table;
            if e.abs() < 0.012 {
                residuals.push(e);
            }
        }
    }
    if residuals.len() < 2000 {
        return Err(format!("стол не виден: {} точек", residuals.len()));
    }
    let table_points = residuals.len();
    let bias = percentile(&mut residuals, 0.5);
    // Точки над столом, без самой руки (губы под камерой) и ближе паспортных 7 см.
    let rt = transpose(r);
    let mut points: Vec<Option<V3>> = vec![None; w * h];
    let mut depth = vec![0.0; w * h];
    for i in 0..w * h {
        let Some(d) = stack.depth(i) else { continue };
        if d < cam.min_m || d > cam.max_m {
            continue;
        }
        let d = d - bias;
        let p = add(origin, scale(ray(i), d));
        let q = apply(&rt, sub(p, tcp));
        let own = q[2] < 0.05 && q[0].abs() < 0.12 && q[1].abs() < 0.10;
        if !own && p[2] > 0.012 {
            points[i] = Some(p);
            depth[i] = d;
        }
    }
    // Связные куски: соседи по кадру с глубиной ближе 15 мм.
    let mut label = vec![usize::MAX; w * h];
    let mut pieces: Vec<Vec<usize>> = Vec::new();
    for start in 0..w * h {
        if points[start].is_none() || label[start] != usize::MAX {
            continue;
        }
        let id = pieces.len();
        let mut piece = vec![start];
        label[start] = id;
        let mut k = 0;
        while k < piece.len() {
            let i = piece[k];
            k += 1;
            let (x, y) = (i % w, i / w);
            let mut near = Vec::with_capacity(4);
            if x > 0 { near.push(i - 1) }
            if x + 1 < w { near.push(i + 1) }
            if y > 0 { near.push(i - w) }
            if y + 1 < h { near.push(i + w) }
            for j in near {
                if points[j].is_some() && label[j] == usize::MAX && (depth[j] - depth[i]).abs() < 0.015 {
                    label[j] = id;
                    piece.push(j);
                }
            }
        }
        pieces.push(piece);
    }
    let top = |piece: &[usize]| {
        let mut z: Vec<f64> = piece.iter().map(|&i| points[i].expect("точка куска")[2]).collect();
        percentile(&mut z, 0.99)
    };
    let mut bottle: Option<(usize, f64)> = None;
    let mut mug: Option<(usize, f64)> = None;
    for (id, piece) in pieces.iter().enumerate() {
        if piece.len() < 40 {
            continue;
        }
        let t = top(piece);
        if (t - expect.bottle_top).abs() < 0.02 && bottle.is_none_or(|(b, _)| pieces[b].len() < piece.len()) {
            bottle = Some((id, t));
        }
        if (expect.mug_top[0]..=expect.mug_top[1]).contains(&t) && mug.is_none_or(|(m, _)| pieces[m].len() < piece.len()) {
            mug = Some((id, t));
        }
    }
    let (b, bottle_top) = bottle.ok_or("бутылка не найдена: нет куска со срезом горла на нужной высоте")?;
    let (m, mug_rim) = mug.ok_or("кружка не найдена: нет куска с краем на нужной высоте")?;
    // Срез горла: точки в 3 мм от верха, середина.
    let cut: Vec<[f64; 2]> = pieces[b]
        .iter()
        .filter_map(|&i| points[i].filter(|p| p[2] > bottle_top - 0.003).map(|p| [p[0], p[1]]))
        .collect();
    if cut.len() < 20 {
        return Err(format!("мало точек среза горла: {}", cut.len()));
    }
    let n = cut.len() as f64;
    let center = cut.iter().fold([0.0; 2], |a, p| [a[0] + p[0] / n, a[1] + p[1] / n]);
    // Край кружки: точки в 4 мм от верха, окружность.
    let rim: Vec<[f64; 2]> = pieces[m]
        .iter()
        .filter_map(|&i| points[i].filter(|p| p[2] > mug_rim - 0.004).map(|p| [p[0], p[1]]))
        .collect();
    if rim.len() < 30 {
        return Err(format!("мало точек края кружки: {}", rim.len()));
    }
    let (mug_center, _) = circle(&rim).ok_or("окружность края кружки не сошлась")?;
    let mut radii: Vec<f64> = rim.iter().map(|p| (p[0] - mug_center[0]).hypot(p[1] - mug_center[1])).collect();
    let mug_radius = percentile(&mut radii, 0.05);
    let mug_outer = percentile(&mut radii, 0.95);
    Ok(Found { bottle: center, bottle_top, mug_center, mug_rim, mug_radius, mug_outer, bias, table_points })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn окружность_по_точкам_края() {
        let points: Vec<[f64; 2]> = (0..40)
            .map(|k| {
                let a = k as f64 * 0.157;
                [0.45 + 0.039 * a.cos(), 0.1 + 0.039 * a.sin()]
            })
            .collect();
        let (c, r) = circle(&points).expect("окружность");
        assert!((c[0] - 0.45).abs() < 1e-9 && (c[1] - 0.1).abs() < 1e-9 && (r - 0.039).abs() < 1e-9);
    }
}
