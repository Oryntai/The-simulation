//! Локоть с прошивкой упирается в препятствие. Сначала сверяем саму модель удара с формулой
//! и с MuJoCo, потом смотрим, какую силу реально даёт прошивка.

use sim_mujoco::Sim;
use sim_scenarios::elbow_plant::{ElbowLoad, ElbowParams, ElbowPlant, Obstacle};
use sim_scenarios::exo_loop::{ContactLog, ExoLoop};
use sim_scenarios::human_limits::{ContactLimit, CONTACT_FOREARM, CONTACT_HAND};
use sim_scenarios::Scenario;

/// Свободный удар локтя о пружину: (пик силы, скорость отскока).
fn plant_impact(extra_inertia: f64, omega: f64, lever: f64, k: f64) -> (f64, f64, f64) {
    let mut p = ElbowParams::winter(75.0, 1.75);
    p.damping = 0.0;
    let mut e = ElbowPlant::new(p, 0.5);
    // Вес руки снят, чтобы сравнивать только удар.
    e.load = ElbowLoad { arm_buoyancy: 1.0, ..ElbowLoad::default() };
    e.omega = omega;
    e.obstacle = Some(Obstacle { angle: 0.55, lever, stiffness: k, damping: 0.0 });
    let j = e.inertia(extra_inertia);
    let mut peak = 0.0_f64;
    for _ in 0..200_000 {
        e.step(5e-6, 0.0, extra_inertia);
        peak = peak.max(e.contact_force);
        // Контакт закончился: дальше рука долетела бы до нижнего упора сустава.
        if peak > 0.0 && e.contact_force == 0.0 {
            break;
        }
    }
    (peak, e.omega, j)
}

#[test]
fn impact_force_matches_spring_formula() {
    for (extra, omega, lever, k) in [(1.37, 2.0, 0.15, 40_000.0), (1.37, 1.0, 0.33, 75_000.0), (0.1, 4.0, 0.25, 40_000.0)] {
        let (peak, after, j) = plant_impact(extra, omega, lever, k);
        let want = omega * (k * j).sqrt();
        println!("J {j:.3} кг·м², ω {omega} рад/с, плечо {lever} м: пик {peak:.1} Н, формула ω·√(k·J) {want:.1} Н");
        assert!((peak - want).abs() < 0.005 * want, "{peak} против {want}");
        assert!((after + omega).abs() < 0.01 * omega, "без демпфирования отскок равен удару: {after}");
    }
}

/// Тот же удар в MuJoCo: звено на шарнире с инерцией привода бьёт в пластину на пружине.
fn mujoco_impact(j: f64, omega: f64, lever: f64, k: f64) -> f64 {
    let tip_mass = 0.01;
    let armature = j - tip_mass * lever * lever;
    let mut s = Sim::from_xml(&format!(
        r#"<mujoco><option timestep="0.00005" gravity="0 0 0" integrator="RK4"/><worldbody>
        <body pos="0 0 1"><joint name="elbow" type="hinge" axis="0 1 0" armature="{armature}"/>
          <geom type="sphere" size="0.01" pos="{lever} 0 0" mass="{tip_mass}" solref="0.0004 1" solimp="0.99 0.99 0.001 0.5 2"/>
        </body>
        <body pos="{lever} 0 {z}"><joint name="plate" type="slide" axis="0 0 1" stiffness="{k}"/>
          <geom type="box" size="0.05 0.05 0.002" mass="0.02" solref="0.0004 1" solimp="0.99 0.99 0.001 0.5 2"/>
        </body>
        </worldbody></mujoco>"#,
        z = 1.0 - 0.012 - 0.002,
    ))
    .unwrap();
    s.qvel_mut()[0] = omega;
    let mut peak = 0.0_f64;
    while s.time() < 0.25 {
        s.step();
        peak = peak.max(k * s.qpos()[1].abs());
    }
    assert!(s.numerically_ok());
    peak
}

#[test]
fn impact_force_matches_mujoco() {
    for (extra, omega, lever, k) in [(1.37, 2.0, 0.15, 40_000.0), (1.37, 1.0, 0.33, 75_000.0)] {
        let (ours, _, j) = plant_impact(extra, omega, lever, k);
        let theirs = mujoco_impact(j, omega, lever, k);
        println!("удар {omega} рад/с, плечо {lever} м: наша модель {ours:.1} Н, MuJoCo {theirs:.1} Н");
        assert!((ours - theirs).abs() < 0.03 * ours, "{ours} против {theirs}");
    }
}

fn run(s: &mut ExoLoop, seconds: f64) {
    for _ in 0..(seconds / 0.01).round() as usize {
        s.advance(0.01);
    }
}

fn clamp_with_firmware(lever: f64, limit: ContactLimit) -> (ContactLog, bool, f64) {
    let mut s = ExoLoop::default();
    s.set_param("payload", 5.0);
    s.set_param("phone_payload", 5.0);
    s.set_choice("human", 1);
    s.set_choice("mode", 2);
    s.set_param("target", 30.0);
    run(&mut s, 3.0);
    s.set_obstacle(Some(Obstacle {
        angle: 60f64.to_radians(),
        lever,
        stiffness: limit.stiffness_n_per_m,
        damping: 0.0,
    }));
    s.set_param("target", 110.0);
    run(&mut s, 6.0);
    (s.contact_log(), s.driver_enabled(), s.firmware_config().i_max_a as f64)
}

#[test]
fn firmware_clamp_is_measured() {
    for (lever, limit) in [(0.15, CONTACT_FOREARM), (0.33, CONTACT_HAND)] {
        let (log, driver_on, i_max) = clamp_with_firmware(lever, limit);
        println!(
            "{}: плечо {lever} м, скорость касания {:.2} м/с, пик {:.0} Н (предел удара {:.0}), зажим {:.0} Н (предел {:.0}), драйвер {}, i_max {i_max:.1} А",
            limit.name,
            log.impact_speed_m_s,
            log.peak_n,
            2.0 * limit.quasi_static_n,
            log.peak_late_n,
            limit.quasi_static_n,
            if driver_on { "включён" } else { "выключен" },
        );
        assert!(log.first_touch_s.is_some(), "рука должна дойти до препятствия");
        assert!(log.peak_n <= 2.0 * limit.quasi_static_n, "{}: удар {:.0} Н", limit.name, log.peak_n);
        assert!(log.peak_late_n <= limit.quasi_static_n, "{}: зажим {:.0} Н", limit.name, log.peak_late_n);
    }
}
