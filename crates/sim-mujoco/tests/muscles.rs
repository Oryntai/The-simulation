//! Шаг 2c–2d: мышцы и локоть на анатомических моделях.
use nalgebra::DVector;
use sim_core::dynamics::rnea;
use sim_core::integrate::{PointForce, external_forces};
use sim_mujoco::elbow::{
    ElbowRig, ExoLoad, capacity, joint_reaction, muscles_at_pose, required_torque, static_opt,
    torque_angle_curve,
};
use sim_mujoco::skeleton::Skeleton;
use sim_mujoco::{Sim, compose_myoarm, models};

fn arms() -> Vec<(&'static str, Sim)> {
    vec![
        ("MyoArm", compose_myoarm().unwrap()),
        (
            "MS-Human-700 рука",
            Sim::load(&models::ms_human_700_arm()).unwrap(),
        ),
    ]
}

/// Наша модель мышцы (Rust) против самого MuJoCo на всех мышцах и сетке длин и скоростей.
#[test]
fn hill_model_rewrite_matches_mujoco() {
    let mut worst: f64 = 0.0;
    let mut n = 0;
    for (_, sim) in arms()
        .into_iter()
        .chain([("тело", Sim::load(&models::ms_human_700()).unwrap())])
    {
        for m in sim.muscles() {
            let mm = sim.muscle_model(m.actuator);
            let (lo, hi) = (mm.lengthrange[0], mm.lengthrange[1]);
            for i in 0..=40 {
                let len = lo - 0.2 * (hi - lo) + 1.4 * (hi - lo) * i as f64 / 40.0;
                for k in 0..=10 {
                    let vel = (k as f64 / 5.0 - 1.0) * 2.0 * mm.l0() * mm.prm[6];
                    let (a, b) = (mm.gain(len, vel), mm.gain_mujoco(len, vel));
                    worst = worst.max((a - b).abs() / mm.f0());
                    n += 1;
                }
                worst = worst.max((mm.bias(len) - mm.bias_mujoco(len)).abs() / mm.f0());
            }
        }
    }
    println!("мышца на Rust ↔ MuJoCo: {n} точек, max |Δ|/F₀ = {worst:.2e}");
    assert!(worst < 1e-12);
}

/// Плечо мышцы двумя способами: якобиан MuJoCo и −dℓ/dθ конечными разностями.
#[test]
fn moment_arms_two_ways_agree() {
    for (name, mut sim) in arms() {
        let rig = ElbowRig::find(&sim).unwrap();
        let mut worst: f64 = 0.0;
        let mut count = 0;
        for deg in (5..=125).step_by(10) {
            sim.qpos_mut()[rig.qadr] = (deg as f64).to_radians();
            sim.forward();
            for m in muscles_at_pose(&mut sim, &rig) {
                worst = worst.max((m.r - m.r_fd).abs());
                count += 1;
            }
        }
        println!("{name}: {count} пар «мышца × угол», max |r − r_fd| = {worst:.2e} м");
        assert!(worst < 1e-5, "{name}: плечи расходятся на {worst}");
    }
}

/// Момент, который нужно держать в локте: MuJoCo ↔ наш движок ↔ сумма по центрам масс.
#[test]
fn required_torque_three_ways() {
    for (name, mut sim) in arms() {
        let rig = ElbowRig::find(&sim).unwrap();
        sim.qpos_mut()[rig.qadr] = 90f64.to_radians();
        sim.forward();
        let payload = 20.0;
        let tau_mj = required_torque(&sim, &rig, payload);

        // Наш движок: RNEA с силой груза в центре масс кисти.
        let sk = Skeleton::from_mujoco(&sim).unwrap();
        let q = sk.q_from(sim.qpos());
        let (hb, off) = sk.body_frame[rig.hand_body].unwrap();
        let xw = sim_core::dynamics::world_to_body(&sk.model, &q);
        let com_w = sim.body_com(rig.hand_body);
        let com_local = xw[hb].point_to_child(&sim_core::Vec3::new(com_w[0], com_w[1], com_w[2]));
        let _ = off;
        let g = sim.gravity();
        let pf = PointForce {
            body: hb,
            point_body: com_local,
            force_world: sim_core::Vec3::new(g[0], g[1], g[2]) * payload,
        };
        let fext = external_forces(&sk.model, &q, &[pf]);
        let tau_ours = rnea(
            &sk.model,
            &q,
            &DVector::zeros(sk.model.nv()),
            &DVector::zeros(sk.model.nv()),
            &fext,
        );
        let tau_ours = sk.to_mj_dofs(&tau_ours, sim.nv())[rig.dof];

        // Ручная сумма: Σ mᵢ·(a · ((cᵢ − o) × (−g))) по поддереву предплечья + груз.
        let (o, a) = rig.axis(&sim);
        let lever = |p: [f64; 3], m: f64| {
            let r = [p[0] - o[0], p[1] - o[1], p[2] - o[2]];
            let f = [-g[0] * m, -g[1] * m, -g[2] * m];
            let c = [
                r[1] * f[2] - r[2] * f[1],
                r[2] * f[0] - r[0] * f[2],
                r[0] * f[1] - r[1] * f[0],
            ];
            a[0] * c[0] + a[1] * c[1] + a[2] * c[2]
        };
        let mut tau_hand = lever(com_w, payload);
        for (b, (_, m)) in sim.bodies().iter().enumerate() {
            if rig.subtree[b] {
                tau_hand += lever(sim.body_com(b), *m);
            }
        }
        println!(
            "{name}: момент в локте при 20 кг, 90°: MuJoCo {tau_mj:.6} | наш {tau_ours:.6} | сумма по ЦМ {tau_hand:.6} Н·м"
        );
        assert!((tau_mj - tau_ours).abs() < 1e-9 * tau_mj.abs().max(1.0));
        assert!((tau_mj - tau_hand).abs() < 1e-9 * tau_mj.abs().max(1.0));
    }
}

/// Сила человека по углу, распределение сил мышц, сжатие в локте с экзоскелетом и без.
#[test]
fn strength_muscle_forces_and_joint_load() {
    for (name, mut sim) in arms() {
        let rig = ElbowRig::find(&sim).unwrap();
        let curve = torque_angle_curve(&mut sim, &rig, 0.0, 130.0, 10.0);
        let line: Vec<String> = curve
            .iter()
            .map(|p| {
                format!(
                    "{:.0}°:{:.0}/{:.0}",
                    p.angle_deg, p.capacity.flexion, p.capacity.extension
                )
            })
            .collect();
        println!(
            "{name}: сила сгибания/разгибания по углу, Н·м: {}",
            line.join(" ")
        );
        let peak = curve.iter().map(|p| p.capacity.flexion).fold(0.0, f64::max);
        assert!(
            peak > 20.0 && peak < 250.0,
            "{name}: пик силы сгибания {peak}"
        );

        sim.qpos_mut()[rig.qadr] = 90f64.to_radians();
        sim.forward();
        let ms = muscles_at_pose(&mut sim, &rig);
        let cap = capacity(&ms);
        for payload in [0.0, 5.0, 20.0, 50.0] {
            let tau = required_torque(&sim, &rig, payload);
            for assist in [0.0, 1.0] {
                let exo = ExoLoad {
                    torque: assist * tau,
                    cuff_distance: 0.15,
                };
                let opt = static_opt(&ms, tau - exo.torque);
                let a = if opt.feasible {
                    opt.activation.clone()
                } else {
                    vec![0.0; ms.len()]
                };
                let rc = joint_reaction(&sim, &rig, &ms, &a, payload, Some(exo));
                println!(
                    "{name}: {payload:>4} кг, экзо {:>3.0}%: нужно {tau:6.1} Н·м, мышцам {:6.1} (сила 90° = {:.1}), {} макс. активация {:5.1}%, сжатие локтя {:7.0} Н, сдвиг {:5.0} Н, манжета {:5.0} Н",
                    assist * 100.0,
                    tau - exo.torque,
                    cap.flexion,
                    if opt.feasible {
                        "хватает,"
                    } else {
                        "НЕ ХВАТАЕТ,"
                    },
                    opt.max_activation * 100.0,
                    rc.compression,
                    rc.shear,
                    rc.cuff_force
                );
                if assist == 1.0 {
                    assert!(
                        opt.max_activation < 0.05,
                        "с полным экзо мышцы почти не работают"
                    );
                }
            }
        }
    }
}

/// Плечо и поясница: груз в руке нагружает их, экзоскелет локтя — нет.
#[test]
fn shoulder_and_spine_loads() {
    use sim_mujoco::loads::{NIOSH_ACTION_LIMIT_N, proximal_loads};
    for (name, mut sim) in arms() {
        let rig = ElbowRig::find(&sim).unwrap();
        sim.qpos_mut()[rig.qadr] = 90f64.to_radians();
        sim.forward();
        let mut prev = 0.0;
        for payload in [0.0, 20.0, 50.0] {
            let p = proximal_loads(&sim, rig.hand_body, payload).expect("в модели есть плечо и L5");
            println!(
                "{name}: {payload:>4} кг — плечо {:.1} Н·м, L5/S1 {:.1} Н·м, выше L5 {:.1} кг, сжатие {:.0} Н",
                p.shoulder_moment, p.l5s1_moment, p.upper_body_mass, p.l5s1_compression
            );
            assert!(p.l5s1_compression > prev, "сжатие должно расти с грузом");
            prev = p.l5s1_compression;
            if payload == 0.0 {
                // Без груза сжатие — порядка веса верхней части тела (корпус стоит почти ровно).
                let w = p.upper_body_mass * 9.80665;
                assert!(
                    p.l5s1_compression < 2.5 * w,
                    "без груза сжатие {} при весе выше L5 {w}",
                    p.l5s1_compression
                );
            }
            if payload == 50.0 {
                assert!(
                    p.l5s1_compression > NIOSH_ACTION_LIMIT_N * 0.8,
                    "50 кг на вытянутой руке — у предела NIOSH и выше"
                );
            }
        }
    }
}
