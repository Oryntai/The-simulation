use sim_mujoco::{Sim, ffi, state::EqualitySwitch};

// Блок и стержень со свободными координатами. Повороты и смещения
// выбраны, чтобы проявились три компоненты силы и три компоненты момента.
const SCENE: &str = r#"
<mujoco><option timestep="0.001" gravity="0 0 0" jacobian="sparse"/>
  <worldbody>
    <body name="блок" pos="0 0 1" quat="0.8 0.6 0 0">
      <freejoint/><geom type="box" size=".2 .1 .15" mass="2" contype="0" conaffinity="0"/>
      <site name="точка-а" pos="0.12 0.1 0.04" quat="0.6 0 0.8 0"/>
    </body>
    <body name="стержень" pos="0.4 0 1" quat="0.6 0 0 0.8">
      <freejoint/><geom type="box" size=".3 .08 .07" mass="3" contype="0" conaffinity="0"/>
      <site name="точка-б" pos="-0.2 0.04 0.09" quat="0.8 0 0.6 0"/>
    </body>
  </worldbody>
  <equality>WELD</equality>
</mujoco>"#;

#[test]
fn реакции_в_точках_сверены_проекцией_в_обобщённые_силы() {
    for equality in [
        r#"<weld name="стык" body1="блок" body2="стержень" anchor="-0.16 0.08 0.07" torquescale=".37"/>"#,
        r#"<weld name="стык" site1="точка-а" site2="точка-б" torquescale=".61"/>"#,
    ] {
        let mut sim = Sim::from_xml(&SCENE.replace("WELD", equality)).unwrap();
        // Смещаем координату и задаём разность скоростей, чтобы включить мягкую связь.
        sim.qpos_mut()[0] += 0.012;
        sim.qvel_mut()[4] = 0.7;
        sim.qfrc_applied_mut()[9] = -4.0;
        sim.step();
        let r = sim.weld_reaction("стык").unwrap().unwrap();
        assert_eq!(r.on_first.force_n, r.on_second.force_n.map(|x| -x));
        assert_eq!(r.on_first.torque_nm, r.on_second.torque_nm.map(|x| -x));
        assert!(r.on_first.force_n.iter().any(|x| x.abs() > 1.0));
        assert!(r.on_first.torque_nm.iter().any(|x| x.abs() > 0.01));
        // Независимый путь: перенести обе реакции в заданную точку, а затем
        // спроецировать их через геометрические якобианы тел.
        let mut projected = vec![0.0; sim.nv()];
        for side in [r.on_first, r.on_second] {
            let shifted = side.at_world_point([0.22, -0.17, 0.82]).unwrap();
            assert!(side.at_world_point([f64::INFINITY, 0.0, 0.0]).is_err());
            let mut linear = vec![0.0; 3 * sim.nv()];
            let mut angular = linear.clone();
            unsafe {
                ffi::mj_jac(
                    sim.model(),
                    sim.data(),
                    linear.as_mut_ptr(),
                    angular.as_mut_ptr(),
                    shifted.point_m.as_ptr(),
                    shifted.body as i32,
                );
            }
            for (axis, (lin, ang)) in linear
                .chunks_exact(sim.nv())
                .zip(angular.chunks_exact(sim.nv()))
                .enumerate()
            {
                for (value, (&jv, &jw)) in projected.iter_mut().zip(lin.iter().zip(ang)) {
                    *value += shifted.force_n[axis] * jv + shifted.torque_nm[axis] * jw;
                }
            }
        }
        let expected = sim.equality_reaction("стык").unwrap().unwrap();
        for (a, b) in projected.into_iter().zip(expected) {
            // Округление координат и проекции f64.
            assert!((a - b).abs() < 1e-10 * b.abs().max(1.0), "{a} против {b}");
        }
        sim.apply_step_effects(
            &[],
            &[EqualitySwitch {
                name: "стык",
                active: false,
            }],
        )
        .unwrap();
        sim.step();
        assert!(sim.weld_reaction("стык").unwrap().is_none());
        assert!(sim.weld_reaction("отсутствует").is_err());
    }
}
