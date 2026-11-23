//! Поля токов: API для приложения (источники, суперпозиция, силовые линии).
use sim_em::field::*;
use std::f64::consts::PI;

#[test]
fn scene_superposition_and_field_lines_for_drawing() {
    // катушка Гельмгольца: два витка на расстоянии радиуса — поле в центре почти однородно
    let a = 0.05;
    let mut scene = FieldScene::new();
    scene.push(CircularLoop::new(
        Vec3::new(0.0, 0.0, -0.5 * a),
        Vec3::z(),
        a,
        1.0,
    ));
    scene.push(CircularLoop::new(
        Vec3::new(0.0, 0.0, 0.5 * a),
        Vec3::z(),
        a,
        1.0,
    ));
    let b0 = scene.b_at(&Vec3::zeros()).z;
    // аналитика: B = (4/5)^{3/2}·μ₀I/a
    let expected = (0.8f64).powf(1.5) * sim_em::MU0 / a;
    assert!((b0 - expected).abs() < 1e-12 * expected);
    let b_off = scene.b_at(&Vec3::new(0.0, 0.0, 0.05 * a)).z;
    assert!((b_off / b0 - 1.0).abs() < 1e-4, "однородность Гельмгольца");

    let opt = TraceOptions {
        step: 0.01 * a,
        max_steps: 4000,
        bounds_radius: 10.0 * a,
        ..Default::default()
    };
    let seeds = seeds_on_circle(Vec3::zeros(), Vec3::z(), 0.5 * a, 6);
    let lines = trace_field_lines(&scene, &seeds, &opt);
    assert_eq!(lines.len(), 6);
    for l in &lines {
        assert!(l.len() > 20);
        assert!(l.iter().all(|p| p.iter().all(|c| c.is_finite())));
    }
}

#[test]
fn polyline_rectangle_matches_center_formula() {
    // квадратная рамка a×a: B в центре = 2√2·μ₀I/(πa)
    let a = 0.1;
    let r = Polyline::rectangle(Vec3::zeros(), Vec3::x(), Vec3::y(), a, a, 2.0);
    let b = r.b_at(&Vec3::zeros()).z;
    let exact = 2.0 * 2f64.sqrt() * sim_em::MU0 * 2.0 / (PI * a);
    assert!((b - exact).abs() < 1e-12 * exact);
}

#[test]
fn solenoid_off_axis_field_is_continuous_across_sheet_axially() {
    let s = Solenoid::new(Vec3::zeros(), Vec3::z(), 0.02, 0.1, 200.0, 1.0);
    // B_ρ непрерывна при переходе через слой (скачок только у B_z)
    let inside = s.b_at(&Vec3::new(0.02 * (1.0 - 1e-7), 0.0, 0.03));
    let outside = s.b_at(&Vec3::new(0.02 * (1.0 + 1e-7), 0.0, 0.03));
    assert!((inside.x - outside.x).abs() < 1e-5 * inside.norm());
    // скачок B_z = μ₀·n·I (граничное условие)
    let jump = inside.z - outside.z;
    assert!((jump - s.infinite_limit()).abs() < 1e-4 * s.infinite_limit());
}

#[test]
fn field_checks_pass() {
    for c in sim_em::field::checks() {
        c.assert_pass();
    }
}
