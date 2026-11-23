//! Рука в воде и на ветру: MuJoCo против ручного расчёта.

use sim_mujoco::elbow::ElbowRig;
use sim_mujoco::fluid::*;
use sim_mujoco::{Sim, models};

fn arm_at_90() -> (Sim, ElbowRig) {
    let mut sim = Sim::load(&models::ms_human_700_arm()).unwrap();
    let rig = ElbowRig::find(&sim).unwrap();
    let q0 = sim.qpos0().to_vec();
    sim.qpos_mut().copy_from_slice(&q0);
    sim.qpos_mut()[rig.qadr] = 90f64.to_radians();
    let c = sim.joint_couplings();
    sim.apply_joint_couplings(&c);
    sim.forward();
    (sim, rig)
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[test]
fn gravity_torque_matches_mujoco_bias() {
    let (sim, rig) = arm_at_90();
    let ours = gravity_elbow_torque(&sim, &rig);
    let mj = -sim.bias_forces()[rig.dof];
    println!("тяжесть: наш Σ {ours:.6} Н·м, MuJoCo −bias {mj:.6} Н·м");
    assert!((ours - mj).abs() < 1e-9);
}

#[test]
fn mujoco_has_no_buoyancy_so_we_add_archimedes() {
    let (mut sim, rig) = arm_at_90();
    let at_rest = mujoco_fluid_elbow_torque(&mut sim, &rig, 1000.0, 1e-3, [0.0; 3], 0.0);
    assert!(
        at_rest.abs() < 1e-9,
        "MuJoCo вдруг даёт силу в покое: {at_rest}"
    );
    let g = gravity_elbow_torque(&sim, &rig);
    let b = buoyancy_elbow_torque(&sim, &rig, 1000.0, 1.0);
    let frac = -b / g;
    println!("тяжесть {g:.3} Н·м, Архимед {b:.3} Н·м, доля {frac:.3}");
    // Плотности 1.13–1.16 г/см³ → вода снимает 86–88 % момента веса.
    assert!(frac > 0.85 && frac < 0.89);
}

#[test]
fn rotation_drag_in_water_two_ways() {
    let (mut sim, rig) = arm_at_90();
    let l = forearm_hand_length(&sim, &rig);
    for omega in [1.0, 3.0] {
        let mj = -mujoco_fluid_elbow_torque(&mut sim, &rig, 1000.0, 1e-3, [0.0; 3], omega);
        let hand = cylinder_rotation_drag(1000.0, 1.0, 0.07, l, omega);
        println!("ω {omega}: MuJoCo {mj:.3} Н·м, цилиндр D 7 см L {l:.3} м Cd 1 → {hand:.3} Н·м");
        assert!(mj > 0.0, "сопротивление должно тормозить");
        let r = mj / hand;
        assert!(
            r > 0.6 && r < 1.6,
            "модели расходятся сильнее разумного: {r}"
        );
    }
}

#[test]
fn wind_on_forearm_two_ways() {
    let (mut sim, rig) = arm_at_90();
    let l = forearm_hand_length(&sim, &rig);
    let (_, axis) = rig.axis(&sim);
    let fd = rig.forearm_dir(&sim);
    // Ветер поперёк предплечья в плоскости сгибания.
    let w = cross(axis, fd);
    let v = 20.0;
    let mj = mujoco_fluid_elbow_torque(
        &mut sim,
        &rig,
        1.204,
        1.81e-5,
        [v * w[0], v * w[1], v * w[2]],
        0.0,
    );
    let hand = cylinder_crossflow_torque(1.204, 1.0, 0.07, l, v);
    println!("ветер 20 м/с: MuJoCo {mj:.3} Н·м, цилиндр {hand:.3} Н·м");
    let r = mj.abs() / hand;
    assert!(r > 0.6 && r < 1.6, "{r}");
}
