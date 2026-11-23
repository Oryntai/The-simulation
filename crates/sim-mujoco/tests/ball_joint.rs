use nalgebra::DMatrix;
use sim_mujoco::Sim;
use sim_mujoco::skeleton::{Skeleton, cross_check};

// Параметры синтетической сцены заданы здесь. Неосевые крепления и наклоны
// выявляют перепутанные системы координат, ref отличает qpos от координаты ядра.
const CHAIN: &str = r#"
<mujoco model="маятник">
  <compiler angle="degree"/>
  <option gravity="1.5 -2.5 -9"/>
  <worldbody>
    <body name="маятник" pos="0.2 -0.1 0.8" euler="19 -28 13">
      <joint name="шар" type="ball" pos="0.11 -0.03 0.05" armature="0.04"/>
      <inertial pos="-0.02 0.07 -0.3" euler="16 25 -9" mass="1.8" diaginertia="0.08 0.09 0.12"/>
      <body name="стержень" pos="-0.1 0.05 -0.4" euler="12 3 -31">
        <joint name="ось" type="hinge" axis="1 2 -3" pos="0.04 0.02 -0.03"
               ref="20" range="-50 70" armature="0.02"/>
        <inertial pos="0.05 -0.04 -0.2" mass="0.9" diaginertia="0.04 0.05 0.06"/>
        <body name="каретка" pos="0.05 0.18 -0.1" euler="-10 25 17">
          <joint name="ход" type="slide" axis="-1 3 2" pos="0.03 0.02 0.01"
                 ref="0.015" range="-0.08 0.12" armature="0.07"/>
          <inertial pos="0.02 0.07 -0.03" mass="0.4" diaginertia="0.01 0.017 0.02"/>
          <body name="блок" pos="0.13 0.07 -0.09" euler="16 15 -9">
            <inertial pos="0.02 0 0.03" mass="0.2" diaginertia="0.003 0.004 0.005"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"#;

#[test]
fn ball_chain_matches_mujoco() {
    // Число поз и допуски те же, что в engine_vs_mujoco.
    let samples = 200;
    let mut sim = Sim::from_xml(CHAIN).unwrap();
    let sk = Skeleton::from_mujoco(&sim).unwrap();
    assert_eq!((sk.model.nq(), sk.model.nv()), (6, 5));
    let (ball, hinge, slide) = (&sk.joints[0], &sk.joints[1], &sk.joints[2]);
    assert_eq!(
        (&ball.qpos, &ball.dofs, &ball.q, &ball.v),
        (&(0..4), &(0..3), &(0..4), &(0..3))
    );
    assert_eq!(
        (&hinge.qpos, &hinge.dofs, &hinge.q, &hinge.v),
        (&(4..5), &(3..4), &(4..5), &(3..4))
    );
    assert_eq!(
        (&slide.qpos, &slide.dofs, &slide.q, &slide.v),
        (&(5..6), &(4..5), &(5..6), &(4..5))
    );
    assert_eq!(sk.q_from(sim.qpos0()), sk.model.neutral_q());
    // Разные значения вдоль всех осей проверяют порядок компонент, включая следующие соединения.
    let q0 = sim.qpos0();
    let q = sk.q_from(&[0.5, -0.5, 0.5, 0.5, q0[4] + 0.25, q0[5] - 0.03125]);
    assert_eq!(&q.as_slice()[..4], &[0.5, -0.5, 0.5, 0.5]);
    // При сложении с ref и обратном вычитании допускаем округление f64.
    assert!((q[4] - 0.25).abs() <= f64::EPSILON);
    assert!((q[5] + 0.03125).abs() <= f64::EPSILON);
    let velocity = [-0.25, 0.5, 0.25, 1.0, -1.0];
    let v = sk.v_from(&velocity);
    assert_eq!(v.as_slice(), &velocity);
    assert_eq!(sk.to_mj_dofs(&v, sim.nv()), velocity);
    // Несимметричный пример отличает построчную вёрстку от транспонирования.
    let data: Vec<f64> = (0..sim.nv().pow(2)).map(|i| i as f64).collect();
    let matrix = DMatrix::from_row_slice(sim.nv(), sim.nv(), &data);
    assert_eq!(sk.to_mj_matrix(&matrix, sim.nv()), data);

    let report = cross_check(&mut sim, &sk, samples, 7);
    println!("маятник, стержень, каретка: {report:?}");
    assert_eq!(report.samples, samples);
    assert!(report.max_pos_err < 1e-9);
    assert!(report.max_rot_err < 1e-9);
    assert!(report.max_mass_rel < 1e-9);
    assert!(report.bias_scale > 0.0);
    assert!(report.max_bias_abs < 1e-8 * report.bias_scale.max(1.0));
    let repeat = cross_check(&mut sim, &sk, samples, 7);
    assert_eq!(report.max_mass_rel.to_bits(), repeat.max_mass_rel.to_bits());
    assert_eq!(report.max_bias_abs.to_bits(), repeat.max_bias_abs.to_bits());
}

#[test]
fn limited_ball_and_slide_share_one_body() {
    // MJCF разрешает поступательное соединение после шарового в одном теле.
    let mut sim = Sim::from_xml(
        r#"
<mujoco model="каретка">
  <compiler angle="degree"/>
  <worldbody>
    <body name="каретка" pos="0.2 0.05 0.4" euler="8 -22 13">
      <joint name="шар" type="ball" pos="0.1 -0.03 0.05" range="0 40"/>
      <joint name="ход" type="slide" axis="2 3 -1" pos="-0.03 0.02 0.04" range="-0.1 0.2"/>
      <inertial pos="-0.05 0.13 -0.3" mass="1.2" diaginertia="0.06 0.09 0.1"/>
    </body>
  </worldbody>
</mujoco>
"#,
    )
    .unwrap();
    let sk = Skeleton::from_mujoco(&sim).unwrap();
    assert_eq!(sk.model.bodies.len(), 2);
    let report = cross_check(&mut sim, &sk, 200, 7);
    println!("каретка с двумя соединениями: {report:?}");
    assert!(report.max_pos_err < 1e-9);
    assert!(report.max_rot_err < 1e-9);
    assert!(report.max_mass_rel < 1e-9);
    assert!(report.max_bias_abs < 1e-8 * report.bias_scale.max(1.0));
}

#[test]
fn comparison_detects_inertia_error_and_rejects_invalid_input() {
    let mut sim = Sim::from_xml(CHAIN).unwrap();
    let mut sk = Skeleton::from_mujoco(&sim).unwrap();
    assert!(
        std::panic::catch_unwind(|| { sk.q_from(&[f64::NAN, 0.0, 0.0, 0.0, 0.0, 0.0]) }).is_err()
    );
    assert!(std::panic::catch_unwind(|| sk.q_from(&[0.0; 6])).is_err());
    assert!(std::panic::catch_unwind(|| sk.v_from(&[f64::INFINITY; 5])).is_err());
    assert!(
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            cross_check(&mut sim, &sk, 0, 7)
        }))
        .is_err()
    );
    // Удвоенный тензор заведомо портит матрицу масс и силы инерции.
    sk.model.bodies[0].inertia.i_com *= 2.0;
    let report = cross_check(&mut sim, &sk, 200, 7);
    assert!(report.max_mass_rel >= 1e-9);
    assert!(report.max_bias_abs >= 1e-8 * report.bias_scale.max(1.0));
}
