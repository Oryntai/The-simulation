//! Шаг по времени. Управление и внешние силы держатся постоянными в течение шага
//! (zero-order hold) — так же работает цифровой регулятор мотора на реальном железе.

use nalgebra::DVector;

use crate::dynamics::{
    aba, normalize_q, passive_forces, point_force_on_body, q_derivative, world_to_body,
};
use crate::model::Model;
use crate::spatial::{SVec, Vec3, assert_finite};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Integrator {
    /// Явный Эйлер: 1-й порядок, энергия растёт. Держим как «плохой» пример для сравнения.
    ExplicitEuler,
    /// Полунеявный (симплектический) Эйлер: 1-й порядок, энергия не уплывает систематически.
    /// Так шагают MuJoCo (по умолчанию), Bullet, PhysX.
    SemiImplicitEuler,
    /// Классический Рунге–Кутта 4-го порядка.
    Rk4,
}

impl Integrator {
    pub const ALL: [Integrator; 3] = [
        Integrator::Rk4,
        Integrator::SemiImplicitEuler,
        Integrator::ExplicitEuler,
    ];

    pub fn label(&self) -> &'static str {
        match self {
            Integrator::ExplicitEuler => "явный Эйлер (1-й порядок)",
            Integrator::SemiImplicitEuler => "полунеявный Эйлер (симплектический)",
            Integrator::Rk4 => "Рунге–Кутта 4",
        }
    }
}

/// Сила, заданная в мире и приложенная к точке тела.
#[derive(Clone, Copy, Debug)]
pub struct PointForce {
    pub body: usize,
    pub point_body: Vec3,
    pub force_world: Vec3,
}

/// Входы на шаг: моменты приводов и внешние силы.
#[derive(Clone, Debug)]
pub struct Inputs {
    pub tau: DVector<f64>,
    pub forces: Vec<PointForce>,
}

impl Inputs {
    pub fn zero(model: &Model) -> Self {
        Self {
            tau: DVector::zeros(model.nv()),
            forces: Vec::new(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct State {
    pub q: DVector<f64>,
    pub v: DVector<f64>,
    pub t: f64,
}

impl State {
    pub fn rest(model: &Model) -> Self {
        Self {
            q: model.neutral_q(),
            v: DVector::zeros(model.nv()),
            t: 0.0,
        }
    }
}

/// Внешние силы → пространственные силы в осях тел для текущей позы.
pub fn external_forces(model: &Model, q: &DVector<f64>, forces: &[PointForce]) -> Vec<SVec> {
    let mut f = model.zero_forces();
    if forces.is_empty() {
        return f;
    }
    let xw = world_to_body(model, q);
    for pf in forces {
        assert_finite(pf.point_body.as_slice(), "точка приложения внешней силы");
        assert_finite(pf.force_world.as_slice(), "внешняя сила");
        f[pf.body] += point_force_on_body(&xw[pf.body], &pf.point_body, &pf.force_world);
    }
    f
}

/// Ускорения при данных (q, v) с учётом приводов, пружин/трения в шарнирах и внешних сил.
pub fn acceleration(
    model: &Model,
    q: &DVector<f64>,
    v: &DVector<f64>,
    inputs: &Inputs,
) -> DVector<f64> {
    let tau = &inputs.tau + passive_forces(model, q, v);
    let f = external_forces(model, q, &inputs.forces);
    aba(model, q, v, &tau, &f)
}

/// Один шаг интегрирования длиной `dt`.
/// Неконечные входы, неконечный результат и недопустимые кватернионы приводят к панике.
pub fn step(model: &Model, s: &mut State, inputs: &Inputs, dt: f64, method: Integrator) {
    assert_finite(&[s.t, dt], "время и шаг интегрирования");
    let t = s.t + dt;
    assert_finite(&[t], "новое время");
    let (mut q, v) = match method {
        Integrator::ExplicitEuler => {
            let a = acceleration(model, &s.q, &s.v, inputs);
            let qd = q_derivative(model, &s.q, &s.v);
            (&s.q + qd * dt, &s.v + a * dt)
        }
        Integrator::SemiImplicitEuler => {
            let a = acceleration(model, &s.q, &s.v, inputs);
            let v = &s.v + a * dt;
            let qd = q_derivative(model, &s.q, &v);
            (&s.q + qd * dt, v)
        }
        Integrator::Rk4 => {
            let (q0, v0) = (s.q.clone(), s.v.clone());
            let k1q = q_derivative(model, &q0, &v0);
            let k1v = acceleration(model, &q0, &v0, inputs);

            let q2 = &q0 + &k1q * (0.5 * dt);
            let v2 = &v0 + &k1v * (0.5 * dt);
            let k2q = q_derivative(model, &q2, &v2);
            let k2v = acceleration(model, &q2, &v2, inputs);

            let q3 = &q0 + &k2q * (0.5 * dt);
            let v3 = &v0 + &k2v * (0.5 * dt);
            let k3q = q_derivative(model, &q3, &v3);
            let k3v = acceleration(model, &q3, &v3, inputs);

            let q4 = &q0 + &k3q * dt;
            let v4 = &v0 + &k3v * dt;
            let k4q = q_derivative(model, &q4, &v4);
            let k4v = acceleration(model, &q4, &v4, inputs);

            (
                q0 + (k1q + k2q * 2.0 + k3q * 2.0 + k4q) * (dt / 6.0),
                v0 + (k1v + k2v * 2.0 + k3v * 2.0 + k4v) * (dt / 6.0),
            )
        }
    };
    assert_finite(v.as_slice(), "новые скорости");
    normalize_q(model, &mut q);
    *s = State { q, v, t };
}
