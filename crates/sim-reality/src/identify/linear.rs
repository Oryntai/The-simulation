use nalgebra::{DMatrix, DVector, linalg::SVD};

use super::Coefficient;

pub(super) struct Fit {
    pub values: DVector<f64>,
    pub coefficients: Vec<Option<Coefficient>>,
    pub reduced_chi_square: Option<f64>,
    pub problem: &'static str,
}

impl Fit {
    pub(super) fn missing(problem: &'static str) -> Self {
        Self {
            values: DVector::zeros(0),
            coefficients: Vec::new(),
            reduced_chi_square: None,
            problem,
        }
    }
}

/// SVD взвешенной матрицы со столбцами единичной нормы. Параметр остаётся
/// неопределимым, если его ось имеет проекцию на отброшенные направления.
pub(super) fn solve(
    design: DMatrix<f64>,
    response: DVector<f64>,
    condition_limit: f64,
) -> Result<Fit, String> {
    let (rows, cols) = design.shape();
    if !design.iter().chain(response.iter()).all(|x| x.is_finite()) {
        return Err("неконечная матрица наблюдений или взвешенные остатки".into());
    }
    if rows <= cols {
        return Ok(Fit::missing("недостаточно наблюдений для оценки остатков"));
    }
    let norms = DVector::from_iterator(
        cols,
        design
            .column_iter()
            .map(|c| c.iter().fold(0.0_f64, |norm, &x| norm.hypot(x))),
    );
    if norms.iter().any(|x| !x.is_finite()) {
        return Err("масштаб столбца вышел за диапазон f64".into());
    }
    let scaled = DMatrix::from_fn(rows, cols, |i, j| {
        if norms[j] > 0.0 {
            design[(i, j)] / norms[j]
        } else {
            0.0
        }
    });
    // Ограничение итераций защищает от зависания при сбое сходимости SVD.
    // 128 шагов для матриц с максимум 9 столбцами. Неуспех возвращаем как ошибку.
    const SVD_STEPS: usize = 128;
    let svd = SVD::try_new(scaled, true, true, f64::EPSILON, SVD_STEPS).ok_or("SVD не сошлось")?;
    let s = svd.singular_values;
    let u = svd.u.ok_or("SVD не вернуло U")?;
    let vt = svd.v_t.ok_or("SVD не вернуло Vᵀ")?;
    let ceiling = s.iter().copied().fold(0.0, f64::max);
    // Стандартная оценка численного ранга max(m,n)·eps·s_max плюс порог опыта.
    let cutoff = ceiling * (1.0 / condition_limit).max(rows.max(cols) as f64 * f64::EPSILON);
    let rank = s.iter().filter(|&&x| x > cutoff).count();
    if rank == 0 {
        return Ok(Fit::missing("нулевая чувствительность наблюдений"));
    }
    let mut projection = u.transpose() * &response;
    for i in 0..cols {
        projection[i] = if s[i] > cutoff {
            projection[i] / s[i]
        } else {
            0.0
        };
    }
    let normalized = vt.transpose() * projection;
    let values = DVector::from_iterator(
        cols,
        (0..cols).map(|i| {
            if norms[i] > 0.0 {
                normalized[i] / norms[i]
            } else {
                0.0
            }
        }),
    );
    let residual = response - design * &values;
    let chi_square = residual.norm_squared() / (rows - rank) as f64;
    if !chi_square.is_finite() || values.iter().any(|x| !x.is_finite()) {
        return Err("решение регрессии или остатки вышли за диапазон f64".into());
    }
    let inflation = chi_square.sqrt().max(1.0);
    let mut coefficients = Vec::with_capacity(cols);
    for i in 0..cols {
        let mut discarded: f64 = 0.0;
        let mut condition: f64 = 0.0;
        for j in 0..cols {
            if s[j] > cutoff {
                condition = condition.hypot(vt[(j, i)] / s[j]);
            } else {
                discarded = discarded.hypot(vt[(j, i)]);
            }
        }
        // Допуск проекции √eps оставляет только ошибки округления ортогональной базы.
        // Отсечённое направление с заметным вкладом делает интервал неограниченным.
        if norms[i] == 0.0 || discarded > f64::EPSILON.sqrt() || condition <= 0.0 {
            coefficients.push(None);
            continue;
        }
        let sensitivity = norms[i] / condition;
        let sigma = inflation / sensitivity;
        if !sigma.is_finite() || sigma <= 0.0 || !sensitivity.is_finite() {
            return Err("неопределённость регрессии вышла за диапазон f64".into());
        }
        coefficients.push(Some(Coefficient {
            value: values[i],
            sigma,
            sensitivity,
            condition,
        }));
    }
    Ok(Fit {
        values,
        coefficients,
        reduced_chi_square: Some(chi_square),
        problem: "параметр связан с отброшенным направлением или не возбуждён",
    })
}
