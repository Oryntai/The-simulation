use std::f64::consts::TAU;

use nalgebra::{DVector, Quaternion, UnitQuaternion};
use sim_core::dynamics::{
    aba, body_coms_world, kinetic_energy, mass_matrix, potential_energy, rnea,
};
use sim_core::{Body, G_STANDARD, Inertia, Inputs, Integrator, Joint, Model, SVec, State, Vec3};
use sim_core::{Xform, step};

// Заданные размеры и массы идеальных тел для проверок, СИ.
const LENGTH: f64 = 0.8;
const MASS: f64 = 1.4;
const RADIUS: f64 = 0.1;
// Допуск алгебраических сверок тот же, что в тестах динамики ядра.
const ALGEBRA_TOL: f64 = 1e-9;

fn add_pendulum(model: &mut Model, parent: Option<usize>) -> usize {
    let offset = if parent.is_some() {
        -LENGTH * Vec3::z()
    } else {
        Vec3::zeros()
    };
    model.add_body(Body::new(
        "маятник",
        parent,
        Joint::Ball,
        Xform::translation(offset),
        Inertia::solid_sphere(MASS, RADIUS, -LENGTH * Vec3::z()),
    ))
}

fn write_orientation(q: &mut [f64], r: &UnitQuaternion<f64>) {
    q.copy_from_slice(&[r.w, r.i, r.j, r.k]);
}

fn orientation(q: &[f64]) -> UnitQuaternion<f64> {
    UnitQuaternion::new_unchecked(Quaternion::new(q[0], q[1], q[2], q[3]))
}

fn pendulums(double: bool) -> (Model, State) {
    let mut model = Model::earth();
    let first = add_pendulum(&mut model, None);
    if double {
        add_pendulum(&mut model, Some(first));
    }
    let mut state = State::rest(&model);
    // Разные наклоны и скорости возбуждают движение по всем трём осям.
    let fixtures = [
        ((0.35, -0.5, 0.2), [0.4, -0.2, 0.6]),
        ((-0.45, 0.15, 0.3), [-0.2, 0.55, 0.3]),
    ];
    for (i, &((roll, pitch, yaw), rate)) in fixtures.iter().take(model.bodies.len()).enumerate() {
        write_orientation(
            &mut state.q.as_mut_slice()[model.q_range(i)],
            &UnitQuaternion::from_euler_angles(roll, pitch, yaw),
        );
        state.v.as_mut_slice()[model.v_range(i)].copy_from_slice(&rate);
    }
    (model, state)
}

#[test]
fn ball_dynamics_agree_with_inverse_dynamics_and_energy() {
    let (mut model, state) = pendulums(true);
    // Заданная добавочная инерция на каждой оси одного соединения, кг·м².
    model.bodies[1].armature = 0.02;
    let q = &state.q;
    let v = &state.v;
    let acceleration = DVector::from_row_slice(&[0.3, -0.1, 0.8, 0.5, 0.2, -0.7]);
    let mut forces = model.zero_forces();
    forces[1] = SVec::new(0.2, -0.3, 0.4, -0.7, 0.2, 1.2);
    let tau = rnea(&model, q, v, &acceleration, &forces);
    let recovered = aba(&model, q, v, &tau, &forces);
    assert!((&recovered - acceleration).norm() < ALGEBRA_TOL);
    assert!((rnea(&model, q, v, &recovered, &forces) - &tau).norm() < ALGEBRA_TOL);

    let m = mass_matrix(&model, q);
    assert!((&m - m.transpose()).norm() < ALGEBRA_TOL);
    let cholesky = m.clone().cholesky().expect("M положительно определена");
    let bias = rnea(&model, q, v, &DVector::zeros(model.nv()), &forces);
    assert!((cholesky.solve(&(tau - bias)) - recovered).norm() < ALGEBRA_TOL);
    assert!((0.5 * v.dot(&(m * v)) - kinetic_energy(&model, q, v)).abs() < ALGEBRA_TOL);
}

#[test]
fn spherical_and_double_pendulums_conserve_energy_with_rk4() {
    for double in [false, true] {
        let (model, mut state) = pendulums(double);
        let inputs = Inputs::zero(&model);
        let ke = kinetic_energy(&model, &state.q, &state.v);
        let pe = potential_energy(&model, &state.q);
        let initial = ke + pe;
        let scale = ke.abs() + pe.abs();
        let mut max_drift = 0.0_f64;
        // Четыре секунды при 500 Гц покрывают несколько периодов простого маятника.
        for _ in 0..2000 {
            step(&model, &mut state, &inputs, 0.002, Integrator::Rk4);
            let e = kinetic_energy(&model, &state.q, &state.v) + potential_energy(&model, &state.q);
            assert!(e.is_finite());
            max_drift = max_drift.max((e - initial).abs() / scale);
        }
        println!("двойной маятник={double}: дрейф энергии {max_drift:.3e}");
        // Порог относительного дрейфа RK4 в этой проверке, 0,001 % энергии.
        assert!(max_drift < 1e-5, "двойной={double}, дрейф {max_drift:.3e}");
    }
}

#[test]
fn spherical_pendulum_precession_period_matches_formula() {
    let mut model = Model::earth();
    add_pendulum(&mut model, None);
    let tilt = 0.6_f64; // Заданный наклон конуса к вертикали, рад.
    let initial_rotation = UnitQuaternion::from_axis_angle(&Vec3::y_axis(), tilt);
    // Для шара на невесомом стержне J_поп − J_ось = mℓ². Для ω = Ω R⁻¹ e_z
    // уравнение Эйлера даёт mℓ² Ω² sin(θ) cos(θ) = mgℓ sin(θ).
    let omega = (G_STANDARD / (LENGTH * tilt.cos())).sqrt();
    let expected = TAU / omega;
    let dt = 0.002; // Шаг 500 Гц, пересечение ищем между отсчётами.
    let mut state = State::rest(&model);
    write_orientation(state.q.as_mut_slice(), &initial_rotation);
    state
        .v
        .copy_from(&(initial_rotation.inverse() * (omega * Vec3::z())));

    let mut prev_y = 0.0;
    let mut period = None;
    let inputs = Inputs::zero(&model);
    let mut max_height_error = 0.0_f64;
    // Двух расчётных оборотов достаточно для поиска первого пересечения.
    for _ in 0..(2.0 * expected / dt).ceil() as usize {
        step(&model, &mut state, &inputs, dt, Integrator::Rk4);
        let p = body_coms_world(&model, &state.q)[0];
        max_height_error = max_height_error.max((p.z + LENGTH * tilt.cos()).abs());
        if prev_y > 0.0 && p.y <= 0.0 && p.x < 0.0 {
            period = Some(state.t - dt * p.y / (p.y - prev_y));
            break;
        }
        prev_y = p.y;
    }
    let measured = period.expect("маятник должен завершить полный оборот");
    let relative_error = (measured / expected - 1.0).abs();
    println!("прецессия: {measured:.10} с, формула {expected:.10} с, ошибка {relative_error:.3e}");
    // Допуск 0,0001 % учитывает интерполяцию пересечения и ошибку RK4.
    assert!(relative_error < 1e-6);
    assert!(max_height_error < LENGTH * 1e-6);
}

#[test]
fn all_integrators_rotate_in_body_axes_and_keep_unit_quaternions() {
    // Шар с ЦМ в точке крепления. При постоянном соосном моменте ω(t)=ω₀+αt,
    // угол φ(t)=ω₀t+αt²/2. Ненулевая начальная поза проверяет порядок умножения.
    let inertia = Inertia::solid_sphere(MASS, RADIUS, Vec3::zeros());
    let mut model = Model::new(Vec3::zeros());
    model.add_body(Body::new(
        "блок",
        None,
        Joint::Ball,
        Xform::IDENTITY,
        inertia,
    ));
    let start = UnitQuaternion::from_euler_angles(0.4, -0.3, 0.5);
    let axis = Vec3::new(1.0, -2.0, 3.0).normalize();
    // Заданные начальная угловая скорость, ускорение и горизонт проверки, СИ.
    let omega = 0.8;
    let alpha = 1.2;
    let duration = 1.0;
    let mut inputs = Inputs::zero(&model);
    inputs.tau.copy_from(&(inertia.i_com * axis * alpha));
    let expected = start
        * UnitQuaternion::from_scaled_axis(
            axis * (omega * duration + 0.5 * alpha * duration * duration),
        );
    for method in Integrator::ALL {
        let run = |steps: usize| {
            let mut state = State::rest(&model);
            assert_eq!(state.q.as_slice(), &[1.0, 0.0, 0.0, 0.0]);
            assert_eq!(state.v.len(), 3);
            write_orientation(state.q.as_mut_slice(), &start);
            state.v.copy_from(&(axis * omega));
            for _ in 0..steps {
                step(&model, &mut state, &inputs, duration / steps as f64, method);
                assert!((state.q.norm() - 1.0).abs() <= 4.0 * f64::EPSILON);
            }
            assert!((state.v - axis * (omega + alpha * duration)).norm() < ALGEBRA_TOL);
            orientation(state.q.as_slice()).angle_to(&expected)
        };
        // Две сетки для проверки сходимости: у Эйлера порядок 1, у RK4 порядок 4.
        let coarse = run(8);
        let fine = run(16);
        let minimum_ratio = match method {
            Integrator::Rk4 => 8.0,
            _ => 1.5,
        };
        assert!(
            fine > 0.0 && coarse / fine > minimum_ratio,
            "{method:?}: {coarse} → {fine}"
        );
    }
}

#[test]
fn ball_rejects_invalid_pose_velocity_and_time() {
    let (model, state) = pendulums(false);
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut q = state.q.clone();
        q[2] = invalid;
        assert!(std::panic::catch_unwind(|| mass_matrix(&model, &q)).is_err());
        let mut v = state.v.clone();
        v[1] = invalid;
        assert!(std::panic::catch_unwind(|| kinetic_energy(&model, &state.q, &v)).is_err());
        let mut invalid_state = state.clone();
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            step(
                &model,
                &mut invalid_state,
                &Inputs::zero(&model),
                invalid,
                Integrator::Rk4,
            )
        }));
        assert!(result.is_err());
        assert_eq!(invalid_state.q, state.q);
        assert_eq!(invalid_state.v, state.v);
        assert_eq!(invalid_state.t, state.t);
    }
    let zero_quat = DVector::zeros(Joint::Ball.nq());
    assert!(std::panic::catch_unwind(|| mass_matrix(&model, &zero_quat)).is_err());
}
