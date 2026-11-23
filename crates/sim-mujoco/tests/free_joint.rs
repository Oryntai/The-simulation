//! Свободное основание: второй движок обязан совпасть с MuJoCo, несмотря на разные кадры скоростей.

use sim_mujoco::Sim;
use sim_mujoco::skeleton::{Skeleton, cross_check};

// Параметры синтетической сцены заданы здесь: наклоны и смещения выявляют ошибку кадра.
const SCENE: &str = r#"
<mujoco model="блок">
  <compiler angle="degree"/>
  <option gravity="0.7 -1.2 -9.4"/>
  <worldbody>
    <body name="основание" pos="0.3 -0.2 0.9" euler="11 -23 37">
      <freejoint name="свободное"/>
      <inertial pos="0.04 -0.03 0.06" euler="9 14 -21" mass="2.4" diaginertia="0.05 0.07 0.09"/>
      <body name="стержень" pos="0.12 0.08 -0.05" euler="-13 27 8">
        <joint name="ось" type="hinge" axis="1 -2 3" pos="0.02 0.01 -0.03" armature="0.03"/>
        <inertial pos="0.03 0.05 -0.12" mass="0.7" diaginertia="0.02 0.025 0.03"/>
        <body name="блок" pos="0.05 -0.07 -0.2">
          <joint name="шар" type="ball" pos="0.01 0.02 0.03"/>
          <inertial pos="0.02 -0.01 -0.05" mass="0.3" diaginertia="0.004 0.005 0.006"/>
        </body>
      </body>
    </body>
    <body name="отдельный" pos="-0.5 0.4 1.1">
      <freejoint name="второе"/>
      <inertial pos="0.01 0 -0.02" mass="1.1" diaginertia="0.01 0.012 0.015"/>
    </body>
  </worldbody>
</mujoco>
"#;

#[test]
fn free_base_matches_mujoco() {
    let mut sim = Sim::from_xml(SCENE).unwrap();
    let sk = Skeleton::from_mujoco(&sim).unwrap();
    assert_eq!((sk.model.nq(), sk.model.nv()), (19, 16));
    let r = cross_check(&mut sim, &sk, 200, 11);
    println!("{r:?}");
    // Те же допуски, что в engine_vs_mujoco.
    assert!(r.max_pos_err < 1e-9, "{r:?}");
    assert!(r.max_rot_err < 1e-9, "{r:?}");
    assert!(r.max_mass_rel < 1e-9, "{r:?}");
    assert!(r.max_bias_abs < 1e-6 * r.bias_scale.max(1.0), "{r:?}");
    assert!(r.bias_scale > 1.0, "сверка прошла вхолостую: {r:?}");
}

#[test]
fn wrong_inertia_is_detected_on_free_base() {
    let wrong = SCENE.replace(r#"mass="2.4" diaginertia="0.05 0.07 0.09""#, r#"mass="2.4" diaginertia="0.10 0.14 0.18""#);
    let mut truth = Sim::from_xml(SCENE).unwrap();
    let wrong = Sim::from_xml(&wrong).unwrap();
    let sk = Skeleton::from_mujoco(&wrong).unwrap();
    let r = cross_check(&mut truth, &sk, 50, 3);
    assert!(r.max_mass_rel > 1e-3, "{r:?}");
}

#[test]
fn armature_on_free_joint_is_refused() {
    let xml = SCENE.replace(r#"<freejoint name="второе"/>"#, r#"<joint name="второе" type="free" armature="0.1"/>"#);
    let sim = Sim::from_xml(&xml).unwrap();
    let e = Skeleton::from_mujoco(&sim).err().expect("должна быть ошибка");
    assert!(e.contains("свободного"), "{e}");
}

#[test]
#[should_panic(expected = "velocity_map")]
fn scalar_velocity_shortcut_refuses_free_joint() {
    let sim = Sim::from_xml(SCENE).unwrap();
    let sk = Skeleton::from_mujoco(&sim).unwrap();
    let _ = sk.v_from(sim.qvel());
}
