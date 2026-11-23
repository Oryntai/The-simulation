//! Привод: API ко-симуляции, тепловая модель, установившиеся режимы.
use sim_em::motor::*;

#[test]
fn cosim_api_round_trip() {
    let mut m = Motor::new(MotorParams::maxon_ec90flat_260w_hd_csg25_50());
    let dt = 1e-3;
    let mut t = 0.0;
    // сустав вращается со скоростью 1 рад/с (двигатель 50 рад/с, противо-ЭДС 11.6 В);
    // на зажимах 20 В — ток положительный, двигатель отдаёт мощность
    for _ in 0..200 {
        let omega_joint = 1.0;
        m.step_electrical(20.0, omega_joint, dt);
        m.step_thermal(dt, 25.0);
        t += dt;
    }
    assert!(t > 0.19);
    let tau = m.joint_torque();
    assert!(tau > 0.0);
    assert!((m.back_emf_v() - 0.231 * 50.0).abs() < 1e-12);
    assert!(m.current() > 0.0);
    // при 10 В двигатель тормозит (генераторный режим): ток и момент отрицательны
    let mut g = Motor::new(MotorParams::maxon_ec90flat_260w_hd_csg25_50());
    for _ in 0..50 {
        g.step_electrical(10.0, 1.0, dt);
    }
    assert!(g.current() < 0.0 && g.joint_torque() < 0.0);
    assert!(m.copper_loss_w() > 0.0);
    assert!(
        m.reflected_inertia() > 1.0,
        "50² · (J_ротора + J_вг) ≈ 1.4 кг·м²"
    );
    assert!(m.winding_temp_c() > 25.0 && m.housing_temp_c() >= 25.0);
    m.set_housing_to_ambient_k_per_w(0.4);
    assert_eq!(m.housing_to_ambient_k_per_w(), 0.4);
}

#[test]
fn backdriving_needs_more_torque_than_forward() {
    let p = MotorParams::cubemars_ak80_64();
    let fwd = p.gear_output_torque(1.0, 1.0);
    let back = p.gear_output_torque(-1.0, 1.0);
    assert!(back.abs() > fwd.abs());
}

#[test]
fn torque_speed_curve_is_monotone() {
    let p = MotorParams::maxon_ec90flat_260w_hd_csg25_50();
    let c = p.torque_speed_curve_joint(48.0, 30);
    assert_eq!(c.len(), 30);
    for w in c.windows(2) {
        assert!(w[1].0 > w[0].0 && w[1].1 <= w[0].1);
    }
    // упор при полном напряжении — сотни Н·м, далеко за пределом редуктора:
    // реальный момент ограничивает драйвер (max_current_a)
    assert!(c[0].1 > p.gear.momentary_peak_output_torque_nm);
    assert!(c.last().unwrap().1.abs() < 1e-9);
}

#[test]
fn hot_winding_derating() {
    let p = MotorParams::maxon_ec90flat_260w_hd_csg25_50();
    let r = p.thermal.r_housing_ambient_k_per_w;
    let i25 = p.continuous_current_a(25.0, 0.0, r);
    let i40 = p.continuous_current_a(40.0, 0.0, r);
    assert!((i40 - derated_current_maxon(i25, 125.0, 40.0)).abs() < 1e-12);
    // с обдувом (меньшее R корпус→среда) длительный ток выше
    assert!(p.continuous_current_a(25.0, 0.0, 0.5) > i25);
}

#[test]
fn motor_checks_pass() {
    for c in sim_em::motor::checks() {
        c.assert_pass();
    }
}
