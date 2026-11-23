//! Шаг 2b: наш движок (Featherstone, `sim-core`) против MuJoCo на одних и тех же моделях тела.
use sim_mujoco::skeleton::{Skeleton, cross_check};
use sim_mujoco::{Sim, compose_myoarm, models};

fn check(name: &str, mut sim: Sim) {
    let sk = Skeleton::from_mujoco(&sim).expect("перенос модели");
    let r = cross_check(&mut sim, &sk, 200, 7);
    println!(
        "{name}: наших тел {} | поз {} | положение ≤ {:.2e} м | ориентация ≤ {:.2e} рад | M ≤ {:.2e} отн. | C+G ≤ {:.2e} (масштаб {:.1})",
        sk.model.bodies.len(),
        r.samples,
        r.max_pos_err,
        r.max_rot_err,
        r.max_mass_rel,
        r.max_bias_abs,
        r.bias_scale
    );
    assert!(r.max_pos_err < 1e-9, "{name}: положения расходятся");
    assert!(r.max_rot_err < 1e-9, "{name}: ориентации расходятся");
    assert!(r.max_mass_rel < 1e-9, "{name}: матрица масс расходится");
    assert!(
        r.max_bias_abs < 1e-8 * r.bias_scale.max(1.0),
        "{name}: силы C+G расходятся"
    );
}

#[test]
fn full_body_matches_mujoco() {
    check("MS-Human-700", Sim::load(&models::ms_human_700()).unwrap());
}

#[test]
fn myoarm_matches_mujoco() {
    check("MyoArm", compose_myoarm().unwrap());
}

#[test]
fn human_arm_matches_mujoco() {
    check(
        "MS-Human-700 рука",
        Sim::load(&models::ms_human_700_arm()).unwrap(),
    );
}
