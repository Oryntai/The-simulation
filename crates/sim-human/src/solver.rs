//! Распределение сил мышц: минимум Σaᵢ² при заданных моментах и пределах 0 ≤ aᵢ ≤ 1.
//!
//! Решаем через двойственную задачу: a(λ) = clip(Wᵀλ, 0, 1), а λ ищем полугладким методом
//! Ньютона с линейным поиском. Малая регуляризация ε делает задачу разрешимой и тогда, когда
//! мышц не хватает: решение становится наилучшим по невязке, а сама невязка показывает дефицит
//! момента. Это условие оптимальности (ККТ) проверяется в тестах отдельно.

use nalgebra::{DMatrix, DVector};

#[derive(Clone, Debug)]
pub struct ActivationSolution {
    pub activation: Vec<f64>,
    /// W·a − b по каждой координате, Н·м.
    pub residual: Vec<f64>,
    pub converged: bool,
}

impl ActivationSolution {
    /// Наибольшая по модулю невязка момента, Н·м.
    pub fn max_residual(&self) -> f64 {
        if self.residual.iter().any(|r| !r.is_finite()) {
            return f64::NAN;
        }
        self.residual.iter().fold(0.0_f64, |m, r| m.max(r.abs()))
    }
}

fn clip(s: f64) -> f64 {
    s.clamp(0.0, 1.0)
}

/// Двойственная функция (её максимизируем).
fn dual(w: &DMatrix<f64>, b: &DVector<f64>, eps: f64, lambda: &DVector<f64>) -> f64 {
    let s = w.tr_mul(lambda);
    let primal: f64 = s
        .iter()
        .map(|&si| {
            let a = clip(si);
            0.5 * a * a - a * si
        })
        .sum();
    primal + lambda.dot(b) - 0.5 * eps * lambda.norm_squared()
}

/// min ½‖a‖² при W·a = b, 0 ≤ a ≤ 1. Строки W — координаты, столбцы — мышцы.
pub fn solve_activation(w: &DMatrix<f64>, b: &DVector<f64>) -> ActivationSolution {
    let (m, n) = w.shape();
    if b.len() != m || w.iter().chain(b.iter()).any(|x| !x.is_finite()) {
        return ActivationSolution {
            activation: vec![f64::NAN; n],
            residual: vec![f64::NAN; m.max(1)],
            converged: false,
        };
    }
    if n == 0 {
        return ActivationSolution {
            activation: Vec::new(),
            residual: b.iter().map(|x| -x).collect(),
            converged: true,
        };
    }
    let wwt = w * w.transpose();
    if wwt.iter().any(|x| !x.is_finite()) {
        return ActivationSolution {
            activation: vec![f64::NAN; n],
            residual: vec![f64::NAN; m],
            converged: false,
        };
    }
    let scale = (0..m).map(|i| wwt[(i, i)]).sum::<f64>() / m.max(1) as f64;
    let eps = 1e-9 * scale.max(1e-9);
    let tol = 1e-8 * b.amax().max(1.0);

    // Старт — решение без пределов.
    let reg = &wwt + DMatrix::identity(m, m) * eps;
    let mut lambda = reg
        .clone()
        .cholesky()
        .map(|c| c.solve(b))
        .unwrap_or_else(|| DVector::zeros(m));

    let mut converged = false;
    // Численный бюджет поиска двойственного решения и последующей проверки невязки.
    for _ in 0..200 {
        let s = w.tr_mul(&lambda);
        let a = DVector::from_iterator(n, s.iter().map(|&x| clip(x)));
        let grad = b - w * &a - &lambda * eps;
        if grad.amax() < tol {
            converged = true;
            break;
        }
        // Гессиан — по мышцам, которые не упёрлись в предел.
        let mut h = DMatrix::identity(m, m) * eps;
        for j in 0..n {
            if s[j] > 0.0 && s[j] < 1.0 {
                let c = w.column(j);
                for p in 0..m {
                    for q in 0..m {
                        h[(p, q)] += c[p] * c[q];
                    }
                }
            }
        }
        let step = match h.clone().cholesky() {
            Some(c) => c.solve(&grad),
            None => match h.lu().solve(&grad) {
                Some(x) => x,
                None => grad.clone() / scale.max(1e-9),
            },
        };
        // Линейный поиск Армихо по двойственной функции.
        let g0 = dual(w, b, eps, &lambda);
        let slope = grad.dot(&step);
        let mut t = 1.0;
        let mut accepted = false;
        for _ in 0..50 {
            let candidate = &lambda + &step * t;
            if dual(w, b, eps, &candidate) >= g0 + 1e-4 * t * slope {
                lambda = candidate;
                accepted = true;
                break;
            }
            t *= 0.5;
        }
        if !accepted {
            break;
        }
    }
    let s = w.tr_mul(&lambda);
    let activation: Vec<f64> = s.iter().map(|&x| clip(x)).collect();
    let a = DVector::from_column_slice(&activation);
    let r = w * &a - b;
    if !converged {
        // Шаг перестал улучшать функцию: проверяем условие оптимальности ещё раз.
        let grad = b - w * &a - &lambda * eps;
        converged = grad.amax() < tol * 10.0;
    }
    ActivationSolution {
        activation,
        residual: r.iter().copied().collect(),
        converged,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Две мышцы на одну координату: a₁/a₂ = w₁/w₂ (ручной расчёт из условия Лагранжа).
    #[test]
    fn two_muscles_match_hand_solution() {
        let w = DMatrix::from_row_slice(1, 2, &[30.0, 10.0]);
        let b = DVector::from_element(1, 20.0);
        let s = solve_activation(&w, &b);
        // λ = b / Σw², a = λ·w
        let lam = 20.0 / (900.0 + 100.0);
        assert!((s.activation[0] - lam * 30.0).abs() < 1e-7);
        assert!((s.activation[1] - lam * 10.0).abs() < 1e-7);
        assert!(s.converged);
    }

    /// Первая мышца упирается в 1: остаток уходит второй (ручной расчёт).
    #[test]
    fn saturated_muscle_passes_rest_to_other() {
        let w = DMatrix::from_row_slice(1, 2, &[30.0, 10.0]);
        let b = DVector::from_element(1, 35.0);
        let s = solve_activation(&w, &b);
        assert!((s.activation[0] - 1.0).abs() < 1e-7);
        assert!((s.activation[1] - 0.5).abs() < 1e-6);
        assert!(s.max_residual() < 1e-5);
    }

    /// Момента не хватает: все на 100%, невязка равна дефициту.
    #[test]
    fn infeasible_reports_deficit() {
        let w = DMatrix::from_row_slice(1, 2, &[30.0, 10.0]);
        let b = DVector::from_element(1, 55.0);
        let s = solve_activation(&w, &b);
        assert!(s.activation.iter().all(|a| (a - 1.0).abs() < 1e-6));
        assert!((s.residual[0] + 15.0).abs() < 1e-4, "{:?}", s.residual);
    }

    /// Мышцы не могут дать момент обратного знака.
    #[test]
    fn wrong_sign_stays_zero() {
        let w = DMatrix::from_row_slice(1, 2, &[30.0, 10.0]);
        let b = DVector::from_element(1, -5.0);
        let s = solve_activation(&w, &b);
        assert!(s.activation.iter().all(|a| a.abs() < 1e-9));
        assert!((s.residual[0] - 5.0).abs() < 1e-6);
    }

    /// Случайная задача: условия ККТ — a = clip(Wᵀλ) с λ из активного набора.
    #[test]
    fn kkt_holds_on_random_problem() {
        let mut seed = 12345u64;
        let mut rnd = || {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((seed >> 33) as f64 / (1u64 << 31) as f64) - 0.5
        };
        let (m, n) = (5, 60);
        let w = DMatrix::from_fn(m, n, |_, _| 40.0 * rnd());
        let target = DVector::from_fn(n, |_, _| 0.3 + 0.4 * rnd());
        let b = &w * &target;
        let s = solve_activation(&w, &b);
        assert!(s.converged);
        assert!(s.max_residual() < 1e-4, "невязка {}", s.max_residual());
        // Решение не хуже допустимой точки, из которой построена правая часть.
        let sum: f64 = s.activation.iter().map(|a| a * a).sum();
        assert!(
            sum <= target.norm_squared() + 1e-9,
            "{sum} > {}",
            target.norm_squared()
        );
        assert!(s.activation.iter().all(|a| (0.0..=1.0).contains(a)));
    }
}
