//! Экзоскелет руки: момент в локте из прямой динамики против ручного расчёта.
use sim_scenarios::exo_arm::{ExoArmConfig, HUMAN_ELBOW_LIMIT, simulate_hold};

fn check(payload: f64, expected_simple: f64) {
    let cfg = ExoArmConfig {
        payload,
        ..ExoArmConfig::default()
    };
    let r = simulate_hold(cfg).expect("рука должна установиться за 20 с");
    let h = r.hand;
    println!(
        "{payload} кг: симуляция {:.3} Н·м | m·g·L {:.3} | Σ mᵢgxᵢ {:.3} | RNEA {:.3} | плечо {:.3} | угол {:.4}° | установилось к {:.2} с",
        r.sim_elbow,
        h.payload,
        h.full,
        r.rnea_elbow,
        r.sim_shoulder,
        r.elbow_angle_deg,
        r.settled_at
    );
    // Ручной расчёт из постановки задачи: ~70 Н·м для 20 кг, ~170 Н·м для 50 кг (L ≈ 0.35 м).
    assert!(
        (h.payload - expected_simple).abs() / expected_simple < 0.03,
        "m·g·L = {} вместо ~{}",
        h.payload,
        expected_simple
    );
    // Второй способ (все массы) — совпадение в 1%.
    assert!(
        (r.sim_elbow - h.full).abs() / h.full < 0.01,
        "симуляция {} vs Σ {}",
        r.sim_elbow,
        h.full
    );
    // Обратная динамика — совпадение в 0.1%.
    assert!(
        (r.rnea_elbow - h.full).abs() / h.full < 1e-3,
        "RNEA {} vs Σ {}",
        r.rnea_elbow,
        h.full
    );
    // Плечо держит тот же момент (плечо вертикально).
    assert!(
        (r.sim_shoulder - h.full).abs() / h.full < 0.01,
        "плечо {} vs Σ {}",
        r.sim_shoulder,
        h.full
    );
    // Угол действительно 90°.
    assert!((r.elbow_angle_deg - 90.0).abs() < 0.1);
    // Формула m·g·L занижает: разница = вес руки и ортеза, и она положительна.
    assert!(r.sim_elbow > h.payload);
}

#[test]
fn elbow_torque_20kg() {
    check(20.0, 70.0);
}

#[test]
fn elbow_torque_50kg() {
    check(50.0, 170.0);
}

#[test]
fn verdict_vs_human_limit() {
    let r20 = simulate_hold(ExoArmConfig {
        payload: 20.0,
        ..Default::default()
    })
    .unwrap();
    let r50 = simulate_hold(ExoArmConfig {
        payload: 50.0,
        ..Default::default()
    })
    .unwrap();
    // 20 кг — около/в пределах человеческого коридора; 50 кг — далеко за ним.
    assert!(r20.sim_elbow > HUMAN_ELBOW_LIMIT.0 && r20.sim_elbow < HUMAN_ELBOW_LIMIT.1 + 5.0);
    assert!(r50.sim_elbow > 2.0 * HUMAN_ELBOW_LIMIT.1);
}
