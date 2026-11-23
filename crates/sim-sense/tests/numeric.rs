mod common;

use std::f64::consts::FRAC_PI_2;

use sim_runtime::{Forces, Object, Runtime, Truth, TruthId};
use sim_sense::{SensorKind, install};
use sim_signal::{Command, Rate, Rng, Time};
use sim_world::CompiledJointKind;

use common::{CatalogDir, SLIDE, effort, near, observations, run, sensor, world};

#[test]
fn noise_drop_delay_and_linear_drift_match_the_statistical_model() {
    let dir = CatalogDir::new("статистика");
    let (sigma, bias, slope, drop_probability) = (0.4, 1.2, 0.05, 0.21);
    // 100 квыборок и сдвиг на 7.5 такта позволяют отдельно оценить моменты и очередь.
    dir.add(
        "noise",
        SensorKind::Encoder,
        &[
            ("rate_hz", 1000.0),
            ("noise_std", sigma),
            ("bias", bias),
            ("drift_per_s", slope),
            ("drop_probability", drop_probability),
            ("latency_s", 0.0075),
        ],
    );
    let mut s = sensor("отсчёт", "noise");
    s.joint = Some("ось".into());
    let scene = world(SLIDE, "0 0 0", vec![s]);
    let mut runtime = Runtime::load(scene.clone()).unwrap();
    let id = install(&mut runtime, &scene, &dir.catalog()).unwrap()[0]
        .signal("value")
        .unwrap();
    run(&mut runtime, 0.007499999);
    assert!(runtime.observe().samples.is_empty());
    run(&mut runtime, 100.0075);

    let samples = observations(&runtime, id);
    assert_eq!(samples.len(), 100_001);
    let mut valid = Vec::new();
    for (i, &(at, value, good)) in samples.iter().enumerate() {
        assert_eq!(at, Time(7_500_000 + i as u64 * 1_000_000));
        if good {
            valid.push((i as f64 / 1000.0, value));
        }
    }
    let n = valid.len() as f64;
    let total = samples.len() as f64;
    assert!(n > 0.0 && n < total);
    // Стандартные ошибки оценки среднего, СКО гауссовского шума и биномиальной доли.
    let m = valid.iter().map(|(t, v)| v - t * slope).sum::<f64>() / n;
    let var = valid
        .iter()
        .map(|(t, v)| (v - t * slope - m).powi(2))
        .sum::<f64>()
        / (n - 1.0);
    near(m, bias, 5.0 * sigma / n.sqrt());
    near(var.sqrt(), sigma, 5.0 * sigma / (2.0 * (n - 1.0)).sqrt());
    near(
        1.0 - n / total,
        drop_probability,
        5.0 * (drop_probability * (1.0 - drop_probability) / total).sqrt(),
    );
    let center = valid.iter().map(|(t, _)| t).sum::<f64>() / n;
    let sxx = valid.iter().map(|(t, _)| (t - center).powi(2)).sum::<f64>();
    let fitted_slope = valid.iter().map(|(t, v)| (t - center) * v).sum::<f64>() / sxx;
    near(fitted_slope, slope, 5.0 * sigma / sxx.sqrt());
}

#[test]
fn drift_random_walk_has_the_configured_increment_variance() {
    let dir = CatalogDir::new("блуждание");
    let coefficient = 0.7;
    dir.add(
        "walk",
        SensorKind::Encoder,
        &[("drift_rw_std", coefficient)],
    );
    let mut s = sensor("отсчёт", "walk");
    s.joint = Some("ось".into());
    let world = world(SLIDE, "0 0 0", vec![s]);
    let mut rt = Runtime::load(world.clone()).unwrap();
    let id = install(&mut rt, &world, &dir.catalog()).unwrap()[0]
        .signal("value")
        .unwrap();
    run(&mut rt, 60.0);
    let samples = observations(&rt, id);
    assert_eq!(samples.len(), 6001);
    assert!(samples.iter().all(|&(_, _, ok)| ok));
    let deltas: Vec<_> = samples.windows(2).map(|p| p[1].1 - p[0].1).collect();
    let n = deltas.len() as f64;
    let mean = deltas.iter().sum::<f64>() / n;
    let measured = (deltas.iter().map(|d| (d - mean).powi(2)).sum::<f64>() / (n - 1.0)).sqrt();
    let expected = coefficient * 0.01_f64.sqrt(); // 100 Гц, σ приращения = α·√Δt.
    near(mean, 0.0, 5.0 * expected / n.sqrt());
    near(
        measured,
        expected,
        5.0 * expected / (2.0 * (n - 1.0)).sqrt(),
    );
}

#[test]
fn fault_modes_quantization_and_saturation_follow_a_moving_reference() {
    let dir = CatalogDir::new("отказы");
    for (name, values) in [
        ("raw", vec![]),
        (
            "shift",
            vec![("fault", 3.0), ("fault_at_s", 0.02), ("fault_offset", 0.25)],
        ),
        ("open", vec![("fault", 2.0), ("fault_at_s", 0.02)]),
        ("stuck", vec![("fault", 1.0), ("fault_at_s", 0.02)]),
        ("early", vec![("fault", 1.0), ("fault_at_s", 0.0)]),
        (
            "scaled",
            vec![
                ("bias", 0.13),
                ("quantum", 0.10),
                ("saturation", 1.0),
                ("min", -0.15),
                ("max", 0.15),
            ],
        ),
        (
            "wrapped",
            vec![
                ("encoder_mode", 1.0),
                ("encoder_period", 0.4),
                ("encoder_zero", 0.1),
            ],
        ),
    ] {
        dir.add(name, SensorKind::Encoder, &values);
    }
    let names = [
        "raw", "shift", "open", "stuck", "early", "scaled", "wrapped",
    ];
    let sensors = names
        .iter()
        .map(|&n| {
            let mut s = sensor(n, n);
            s.joint = Some("ось".into());
            s
        })
        .collect();
    let mut scene = world(SLIDE, "0 0 0", sensors);
    effort(&mut scene, CompiledJointKind::Slide);
    let mut runtime = Runtime::load(scene.clone()).unwrap();
    let handles = install(&mut runtime, &scene, &dir.catalog()).unwrap();
    let channel = runtime.channel("вход").unwrap();
    runtime
        .act(&Command {
            targets: vec![(channel, 100.0)],
        })
        .unwrap();
    run(&mut runtime, 0.06);
    let logs: Vec<_> = handles
        .iter()
        .map(|h| observations(&runtime, h.signal("value").unwrap()))
        .collect();
    assert!(logs.iter().all(|log| log.len() == 7));
    for (i, &(time, q, valid)) in logs[0].iter().enumerate() {
        assert!(valid);
        let steps = time.0 as f64 / 1_000_000.0;
        // Эйлер для 1 кг, 100 Н и Δt=1 мс.
        near(
            q,
            100.0 * 0.001_f64.powi(2) * steps * (steps + 1.0) / 2.0,
            1e-12,
        );
        let failed = i >= 2;
        near(logs[1][i].1, q + if failed { 0.25 } else { 0.0 }, 1e-12);
        assert!(logs[1][i].2);
        assert_eq!(logs[2][i].2, !failed);
        assert_eq!(logs[3][i].2, !failed);
        near(logs[3][i].1, if failed { logs[0][1].1 } else { q }, 1e-12);
        assert!(!logs[4][i].2);
        near(
            logs[5][i].1,
            (((q + 0.13) / 0.1).round() * 0.1).clamp(-0.15, 0.15),
            1e-12,
        );
        near(logs[6][i].1, if q < 0.1 { q + 0.3 } else { q - 0.1 }, 1e-12);
    }
}

#[test]
fn imu_rotating_at_constant_speed_measures_gravity_omega_and_centripetal_acceleration() {
    let dir = CatalogDir::new("инерция");
    dir.add("rest", SensorKind::Imu, &[("rate_hz", 1000.0)]);
    dir.add(
        "spin",
        SensorKind::Imu,
        &[
            ("rate_hz", 1000.0),
            ("mount_dx_m", 0.4),
            ("mount_yaw_rad", FRAC_PI_2),
        ],
    );
    let mut spin = sensor("вращение", "spin");
    spin.body = Some("стержень".into());
    let mut scene = world(
        r#"<body name="стержень">
        <joint name="ось" type="hinge" axis="0 0 1"/>
        <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/></body>"#,
        "0 0 -9.81",
        vec![sensor("покой", "rest"), spin],
    );
    effort(&mut scene, CompiledJointKind::Hinge);
    let mut runtime = Runtime::load(scene.clone()).unwrap();
    let handles = install(&mut runtime, &scene, &dir.catalog()).unwrap();
    let channel = runtime.channel("вход").unwrap();
    // I=1 кг·м², импульс 2000 Н·м за 1 мс задаёт ω=2 рад/с.
    runtime
        .act(&Command {
            targets: vec![(channel, 2000.0)],
        })
        .unwrap();
    run(&mut runtime, 0.001);
    runtime
        .act(&Command {
            targets: vec![(channel, 0.0)],
        })
        .unwrap();
    run(&mut runtime, 0.13);
    for (h, expected) in handles.iter().zip([
        [0.0, 0.0, 9.81, 0.0, 0.0, 0.0],
        [0.0, 2.0_f64.powi(2) * 0.4, 9.81, 0.0, 0.0, 2.0],
    ]) {
        for (axis, expected) in ["x", "y", "z", "gx", "gy", "gz"].into_iter().zip(expected) {
            let samples = observations(&runtime, h.signal(axis).unwrap());
            assert_eq!(samples.len(), 131);
            for &(_, value, valid) in &samples[2..] {
                assert!(valid);
                near(value, expected, 1e-9);
            }
        }
    }
}

#[derive(Clone)]
struct Plant([(TruthId, f64); 3]);

impl Object for Plant {
    fn update(
        &mut self,
        _: &Truth<'_>,
        _: &Command,
        _: &mut Rng,
        out: &mut Forces,
    ) -> Result<(), String> {
        for (channel, value) in self.0 {
            out.publish(channel, value);
        }
        Ok(())
    }
}

#[test]
fn plant_channels_blanking_on_start_and_ray_distance() {
    let dir = CatalogDir::new("каналы");
    // Дальность и диапазон задаёт исходная синтетическая сцена: сфера на оси в 2.5 м, r=0.5 м.
    dir.add(
        "range",
        SensorKind::Rangefinder,
        &[("range_min_m", 0.1), ("range_max_m", 5.0)],
    );
    for (name, kind) in [
        ("current", SensorKind::Current),
        ("temperature", SensorKind::Temperature),
        ("bridge", SensorKind::BridgeAdc),
    ] {
        dir.add(name, kind, &[]);
    }
    let mut sensors = vec![sensor("луч", "range"), sensor("мимо", "range")];
    sensors[1].position_m[0] = 2.0;
    sensors.extend(["current", "temperature", "bridge"].map(|name| {
        let mut s = sensor(name, name);
        s.drive = Some("модель".into());
        s
    }));
    let world = world(
        r#"<geom type="sphere" size="0.5" pos="0 0 2.5"/>"#,
        "0 0 0",
        sensors,
    );
    let cat = dir.catalog();
    let mut rt = Runtime::load(world.clone()).unwrap();
    assert!(
        install(&mut rt, &world, &cat)
            .err()
            .unwrap()
            .contains("не зарегистрирован")
    );
    let a = rt.register_truth("модель/current_a", "A").unwrap();
    let c = rt.register_truth("модель/winding_c", "degC").unwrap();
    let v = rt.register_truth("модель/bridge_mv", "mV").unwrap();
    // Заведомо разные знаки и единицы исключают пересечение выходов.
    rt.register_object(
        "модель",
        Rate::hz(1000).unwrap(),
        &[],
        Plant([(a, -1.75), (c, 45.5), (v, 0.8)]),
    )
    .unwrap();
    let handles = install(&mut rt, &world, &cat).unwrap();
    run(&mut rt, 0.01);
    for (i, value) in [Some(2.0), None, Some(-1.75), Some(45.5), Some(0.8)]
        .into_iter()
        .enumerate()
    {
        let id = handles[i].signal("value").unwrap();
        let log = observations(&rt, id);
        assert_eq!(log.len(), 2);
        assert_eq!(rt.observe().latest(id), value);
        assert_eq!(log[0].2, i == 0);
    }
}

#[test]
fn reaction_and_contact_patch_match_weight_and_lever_arm() {
    let dir = CatalogDir::new("нагрузка");
    for (id, kind, fields) in [
        ("force", SensorKind::Force, vec![]),
        ("torque", SensorKind::Torque, vec![("mount_dx_m", 0.25)]),
        (
            "pressure",
            SensorKind::Pressure,
            vec![("patch_radius_m", 0.25), ("patch_area_m2", 0.04)],
        ),
        (
            "touch",
            SensorKind::Touch,
            vec![("patch_radius_m", 0.25), ("touch_threshold_n", 18.0)],
        ),
        (
            "miss",
            SensorKind::Touch,
            vec![("patch_radius_m", 0.25), ("touch_threshold_n", 21.0)],
        ),
    ] {
        dir.add(id, kind, &fields);
    }
    let sensors = ["force", "torque", "pressure", "touch", "miss"]
        .into_iter()
        .map(|id| {
            let mut s = sensor(id, id);
            s.body = Some(
                if matches!(id, "force" | "torque") {
                    "стержень"
                } else {
                    "блок"
                }
                .into(),
            );
            if !matches!(id, "force" | "torque") {
                s.position_m[2] = -0.1;
            }
            s
        })
        .collect();
    let scene = world(
        r#"
        <geom name="плоскость" type="plane" size="2 2 0.1"/>
        <body name="стержень" pos="1 0 1"><geom size="0.1" mass="2"/></body>
        <body name="блок" pos="0 0 0.1"><freejoint/>
          <geom type="box" size="0.1 0.1 0.1" mass="2"/>
        </body>"#,
        "0 0 -9.81",
        sensors,
    );
    let mut rt = Runtime::load(scene.clone()).unwrap();
    let h = install(&mut rt, &scene, &dir.catalog()).unwrap();
    run(&mut rt, 0.5);
    let obs = rt.observe();
    let value = |i: usize, name| obs.latest(h[i].signal(name).unwrap()).unwrap();
    let weight = 2.0 * 9.81;
    near(value(0, "z"), weight, 1e-12);
    near(value(0, "x"), 0.0, 1e-12);
    near(value(1, "y"), weight * 0.25, 1e-12);
    near(value(1, "z"), 0.0, 1e-12);
    near(value(2, "value"), weight / 0.04, 0.01);
    assert_eq!(value(3, "value"), 1.0);
    assert_eq!(value(4, "value"), 0.0);
}
