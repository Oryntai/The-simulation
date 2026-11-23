//! `sim-fem` — первый кирпич прочностного блока: балка Эйлера–Бернулли методом конечных
//! элементов (эрмитовы элементы, 2 степени свободы на узел: прогиб w и поворот θ).
//!
//! Это та же постановка, что элементы B31/B23 в CalculiX/Abaqus и SEG2 «POU_D_E» в Code_Aster —
//! на ней мы потом сверяемся с этими пакетами на одинаковых кейсах.
//!
//! Допущения модели (важно для доверия к числам):
//! * малые прогибы (линейная геометрия), материал линейно-упругий;
//! * сдвиг не учитывается (короткие толстые балки L/h < 10 — нужен Тимошенко);
//! * сечение постоянное по длине.

use nalgebra::{DMatrix, DVector};

/// Сечения и материалы.
pub mod section {
    /// Момент инерции прямоугольника `b × h` при изгибе поперёк `h`, м⁴.
    pub fn rect_i(b: f64, h: f64) -> f64 {
        b * h.powi(3) / 12.0
    }

    /// Момент инерции трубы: наружный диаметр `d_out`, внутренний `d_in`, м⁴.
    pub fn tube_i(d_out: f64, d_in: f64) -> f64 {
        std::f64::consts::PI * (d_out.powi(4) - d_in.powi(4)) / 64.0
    }

    #[derive(Clone, Copy, Debug)]
    pub struct Material {
        pub name: &'static str,
        /// Модуль Юнга, Па.
        pub e: f64,
        /// Предел текучести, Па.
        pub yield_strength: f64,
        /// Плотность, кг/м³.
        pub density: f64,
    }

    /// Алюминий 6061-T6 (ASM / MatWeb, типовые значения).
    pub const AL_6061_T6: Material = Material {
        name: "Al 6061-T6",
        e: 68.9e9,
        yield_strength: 276e6,
        density: 2700.0,
    };
    /// Сталь конструкционная (S235-класс).
    pub const STEEL_S235: Material = Material {
        name: "сталь S235",
        e: 210e9,
        yield_strength: 235e6,
        density: 7850.0,
    };
}

/// Консольная балка: заделка в x = 0, свободный конец в x = L.
#[derive(Clone, Copy, Debug)]
pub struct Cantilever {
    pub length: f64,
    /// Изгибная жёсткость E·I, Н·м².
    pub ei: f64,
    pub elements: usize,
}

/// Нагрузки (положительное направление — направление прогиба).
#[derive(Clone, Copy, Debug, Default)]
pub struct Loads {
    /// Сосредоточенная сила на конце, Н.
    pub tip_force: f64,
    /// Равномерная погонная нагрузка, Н/м.
    pub distributed: f64,
}

#[derive(Clone, Debug)]
pub struct BeamSolution {
    pub x: Vec<f64>,
    /// Прогиб в узлах, м.
    pub w: Vec<f64>,
    /// Угол поворота в узлах, рад.
    pub theta: Vec<f64>,
}

impl BeamSolution {
    pub fn tip_deflection(&self) -> f64 {
        *self.w.last().unwrap()
    }

    pub fn tip_slope(&self) -> f64 {
        *self.theta.last().unwrap()
    }
}

/// Матрица жёсткости элемента длины `l` (порядок dof: w₁, θ₁, w₂, θ₂).
pub fn element_stiffness(ei: f64, l: f64) -> [[f64; 4]; 4] {
    let k = ei / l.powi(3);
    let l2 = l * l;
    [
        [12.0 * k, 6.0 * l * k, -12.0 * k, 6.0 * l * k],
        [6.0 * l * k, 4.0 * l2 * k, -6.0 * l * k, 2.0 * l2 * k],
        [-12.0 * k, -6.0 * l * k, 12.0 * k, -6.0 * l * k],
        [6.0 * l * k, 2.0 * l2 * k, -6.0 * l * k, 4.0 * l2 * k],
    ]
}

/// Согласованный вектор нагрузки от равномерной `q` на элементе длины `l`.
pub fn element_udl(q: f64, l: f64) -> [f64; 4] {
    [
        q * l / 2.0,
        q * l * l / 12.0,
        q * l / 2.0,
        -q * l * l / 12.0,
    ]
}

/// Решить консоль: сборка K, заделка (w₀ = θ₀ = 0), разложение Холецкого.
pub fn solve_cantilever(beam: &Cantilever, loads: &Loads) -> BeamSolution {
    let n = beam.elements.max(1);
    let l = beam.length / n as f64;
    let ndof = 2 * (n + 1);
    let mut k = DMatrix::<f64>::zeros(ndof, ndof);
    let mut f = DVector::<f64>::zeros(ndof);
    let ke = element_stiffness(beam.ei, l);
    let fe = element_udl(loads.distributed, l);
    for e in 0..n {
        let dofs = [2 * e, 2 * e + 1, 2 * e + 2, 2 * e + 3];
        for a in 0..4 {
            f[dofs[a]] += fe[a];
            for b in 0..4 {
                k[(dofs[a], dofs[b])] += ke[a][b];
            }
        }
    }
    f[ndof - 2] += loads.tip_force;
    // Заделка: убираем dof 0 и 1.
    let kf = k.view((2, 2), (ndof - 2, ndof - 2)).clone_owned();
    let ff = f.rows(2, ndof - 2).clone_owned();
    let u = kf
        .cholesky()
        .expect("матрица жёсткости должна быть положительно определённой")
        .solve(&ff);
    let mut w = vec![0.0; n + 1];
    let mut theta = vec![0.0; n + 1];
    for i in 1..=n {
        w[i] = u[2 * (i - 1)];
        theta[i] = u[2 * (i - 1) + 1];
    }
    let x = (0..=n).map(|i| i as f64 * l).collect();
    BeamSolution { x, w, theta }
}

/// Изгибающий момент в заделке консоли, Н·м.
pub fn root_moment(beam: &Cantilever, loads: &Loads) -> f64 {
    loads.tip_force * beam.length + loads.distributed * beam.length * beam.length / 2.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn single_element_tip_load_is_exact() {
        let b = Cantilever {
            length: 2.0,
            ei: 1000.0,
            elements: 1,
        };
        let s = solve_cantilever(
            &b,
            &Loads {
                tip_force: 10.0,
                distributed: 0.0,
            },
        );
        let exact = 10.0 * 8.0 / (3.0 * 1000.0);
        assert!((s.tip_deflection() - exact).abs() < 1e-12);
    }
}
