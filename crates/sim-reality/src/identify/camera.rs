use nalgebra::{DMatrix, DVector};
use sim_sense::Intrinsics;

use super::{Criteria, Fit, fit_rows, positive, solve};

const N: usize = super::OPTICS.len();

struct Point {
    xyz: [f64; 3],
    xy: [f64; 2],
    pixel: [f64; 2],
}

fn lens(a: &DVector<f64>, [width, height]: [u32; 2]) -> Intrinsics {
    Intrinsics {
        width,
        height,
        fx: a[0],
        fy: a[1],
        cx: a[2],
        cy: a[3],
        radial: [a[4], a[5], a[6]],
        tangential: [a[7], a[8]],
    }
}

fn linearize(
    points: &[Point],
    lens: &Intrinsics,
    sigma: f64,
) -> Result<(DMatrix<f64>, DVector<f64>), String> {
    let mut jac = DMatrix::zeros(points.len() * 2, N);
    let mut residual = DVector::zeros(points.len() * 2);
    for (i, point) in points.iter().enumerate() {
        let [x, y] = point.xy;
        let r2 = x * x + y * y;
        let r4 = r2 * r2;
        let r6 = r4 * r2;
        let [u, v] = lens
            .project(point.xyz)?
            .ok_or("мишень позади оптической плоскости")?;
        residual[2 * i] = (point.pixel[0] - u) / sigma;
        residual[2 * i + 1] = (point.pixel[1] - v) / sigma;
        // Производные проекции Брауна — Конради из sim-sense по девяти параметрам.
        let [fx, fy] = [lens.fx, lens.fy];
        let [du_dfx, dv_dfy] = [(u - lens.cx) / fx, (v - lens.cy) / fy];
        let u_row = [
            du_dfx,
            0.0,
            1.0,
            0.0,
            fx * x * r2,
            fx * x * r4,
            fx * x * r6,
            fx * 2.0 * x * y,
            fx * (r2 + 2.0 * x * x),
        ];
        let v_row = [
            0.0,
            dv_dfy,
            0.0,
            1.0,
            fy * y * r2,
            fy * y * r4,
            fy * y * r6,
            fy * (r2 + 2.0 * y * y),
            fy * 2.0 * x * y,
        ];
        for (j, (&u, &v)) in u_row.iter().zip(&v_row).enumerate() {
            jac[(2 * i, j)] = u / sigma;
            jac[(2 * i + 1, j)] = v / sigma;
        }
    }
    Ok((jac, residual))
}

pub(super) fn fit(
    rows: &[[f64; 5]],
    size: [u32; 2],
    sigma: f64,
    iterations: usize,
    criteria: &Criteria,
) -> Result<Fit, String> {
    if size.contains(&0) || !positive(sigma) || iterations == 0 {
        return Err("требуются размеры кадра, положительное СКО и предел итераций".into());
    }
    let points = rows
        .iter()
        .map(|&[x, y, z, u, v]| {
            if !positive(z)
                || !(0.0..f64::from(size[0])).contains(&u)
                || !(0.0..f64::from(size[1])).contains(&v)
            {
                return Err("мишень должна быть перед камерой, а наблюдение внутри кадра".into());
            }
            let xy = [x / z, y / z];
            if !xy.iter().all(|v| v.is_finite()) {
                return Err("нормированная координата мишени вышла за диапазон f64".into());
            }
            Ok(Point {
                xyz: [x, y, z],
                xy,
                pixel: [u, v],
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    if points.len().saturating_mul(2) <= N {
        return Ok(Fit::missing("недостаточно точек мишени"));
    }

    // Начальная оценка из вложенной модели без дисторсии. Нули и стартовые
    // коэффициенты остаются внутренним состоянием поиска.
    let mut a = DVector::zeros(N);
    for d in 0..2 {
        let rows = points
            .iter()
            .map(|p| ([p.xy[d], 1.0], p.pixel[d]))
            .collect::<Vec<_>>();
        let start = fit_rows(&rows, sigma, criteria)?;
        if start.coefficients.first().is_none_or(Option::is_none) || !positive(start.values[0]) {
            return Ok(Fit::missing(
                "нет независимой проекции мишени по обеим осям",
            ));
        }
        a[d] = start.values[0];
        a[d + 2] = start.values[1];
    }

    // Критерий остановки по линейному изменению пикселей: √eps от масштаба
    // исходного кадра. Допуск определяется точностью вычислений, не истиной опыта.
    let tolerance = f64::EPSILON.sqrt() * f64::from(size[0].max(size[1])) / sigma;
    for _ in 0..iterations {
        let (jac, residual) = linearize(&points, &lens(&a, size), sigma)?;
        let mut step = solve(jac.clone(), residual.clone(), criteria.max_condition)?;
        if step.values.len() != N {
            return Ok(step);
        }
        let change = (&jac * &step.values).amax();
        if change <= tolerance {
            // Ковариация в текущей точке; values от solve здесь являются шагом, а не оценкой.
            for (c, &value) in step.coefficients.iter_mut().zip(a.iter()) {
                if let Some(c) = c {
                    c.value = value;
                }
            }
            return Ok(step);
        }
        let cost = residual.norm();
        let mut rate = 1.0;
        loop {
            let candidate = &a + rate * &step.values;
            let optics = lens(&candidate, size);
            if optics.validate().is_ok()
                && linearize(&points, &optics, sigma).is_ok_and(|(_, r)| r.norm() < cost)
            {
                a = candidate;
                break;
            }
            // Деление пополам до масштаба округления проекции.
            rate *= 0.5;
            if rate <= f64::EPSILON.sqrt() {
                return Ok(Fit::missing(
                    "поиск параметров камеры не уменьшил ошибку проекции",
                ));
            }
        }
    }
    Ok(Fit::missing(
        "оценка параметров камеры не сошлась за заданное число итераций",
    ))
}
