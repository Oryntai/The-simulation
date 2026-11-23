mod common;

use std::f64::consts::{FRAC_PI_2, TAU};

use sim_runtime::{Evaluator, LogKind, Runtime, Truth, TruthOut};
use sim_sense::{
    FrameTruth, FrameTruthReader, ObjectPose, Pose, SensorKind, compare_detections, install,
};
use sim_signal::{Command, ImageFrame, PixelFormat, Rate, Rng, SignalId, Time};
use sim_world::{CompiledJointKind, CompiledSensor};

use common::{CatalogDir, SLIDE, effort, near, observations, run, sensor, world};

const ROTOR: &str = r#"<body name="стержень" pos="0 0 2">
  <joint name="ось" axis="0 0 1"/>
  <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
  <geom name="блок" type="sphere" pos="1 0 0" size=".05" rgba="1 1 1 1"/>
</body>"#;

fn encoder(name: &str, part: &str) -> CompiledSensor {
    let mut sensor = sensor(name, part);
    sensor.joint = Some("ось".into());
    sensor
}

fn drive(rt: &mut Runtime, value: f64) {
    rt.act(&Command {
        targets: vec![(rt.channel("вход").unwrap(), value)],
    })
    .unwrap();
}

fn depth_mm(frame: &ImageFrame) -> Vec<u16> {
    assert_eq!(frame.format, PixelFormat::DepthMm16);
    assert!(frame.is_consistent());
    let (pixels, remainder) = frame.data.as_chunks::<2>();
    assert!(remainder.is_empty());
    pixels.iter().copied().map(u16::from_le_bytes).collect()
}

#[test]
fn stuck_scalar_never_marks_a_repeated_measurement_as_fresh() {
    let dir = CatalogDir::new("залипание-отсчёта");
    dir.add(
        "stuck",
        SensorKind::Encoder,
        &[
            ("fault", 1.0),
            ("fault_at_s", 0.01),
            ("latency_s", 0.002),
            ("bias", 1.0),
        ],
    );
    let mut scene = world(SLIDE, "0 0 0", vec![encoder("отсчёт", "stuck")]);
    effort(&mut scene, CompiledJointKind::Slide);
    let mut rt = Runtime::load(scene.clone()).unwrap();
    let id = install(&mut rt, &scene, &dir.catalog()).unwrap()[0]
        .signal("value")
        .unwrap();
    drive(&mut rt, 100.0);
    run(&mut rt, 0.001999999);
    assert!(rt.observe().samples.is_empty());
    run(&mut rt, 0.011999999);
    let first = rt.observe().samples;
    assert_eq!(
        (first[0].at, first[0].value, first[0].valid),
        (Time(2_000_000), 1.0, true)
    );
    run(&mut rt, 0.012);
    let obs = rt.observe();
    let last = obs.samples.last().unwrap();
    assert_eq!(last.signal, id);
    assert_eq!(
        (last.at, last.value, last.valid),
        (Time(12_000_000), 1.0, false)
    );
    assert_eq!(obs.samples.len(), 2);
    assert_eq!(obs.samples[0], first[0]);
}

#[test]
fn positive_subnanosecond_delay_cannot_deliver_at_zero() {
    let dir = CatalogDir::new("меньше-наносекунды");
    // Одна наносекунда — первый представимый момент после заданных 0.4 нс.
    dir.add("short", SensorKind::Encoder, &[("latency_s", 0.4e-9)]);
    let scene = world(SLIDE, "0 0 0", vec![encoder("отсчёт", "short")]);
    let mut rt = Runtime::load(scene.clone()).unwrap();
    let id = install(&mut rt, &scene, &dir.catalog()).unwrap()[0]
        .signal("value")
        .unwrap();
    run(&mut rt, 0.0);
    assert_eq!(observations(&rt, id), vec![]);
    run(&mut rt, 1e-9);
    assert_eq!(observations(&rt, id), [(Time(1), 0.0, true)]);
}

#[test]
fn slower_off_grid_samples_cannot_support_a_proof() {
    let dir = CatalogDir::new("между-тактами");
    dir.add(
        "phase",
        SensorKind::Encoder,
        &[("rate_hz", 333.0), ("latency_s", 0.002)],
    );
    let mut scene = world(SLIDE, "0 0 0", vec![encoder("отсчёт", "phase")]);
    effort(&mut scene, CompiledJointKind::Slide);
    let mut rt = Runtime::load(scene.clone()).unwrap();
    let id = install(&mut rt, &scene, &dir.catalog()).unwrap()[0]
        .signal("value")
        .unwrap();
    drive(&mut rt, 100.0);
    run(&mut rt, 0.005003003);
    let samples = observations(&rt, id);
    assert_eq!(samples.len(), 2);
    assert_eq!(samples[1].0, Time(5_003_003));
    // 333 Гц: начало в 3 003 003 нс, а каретка известна только в 3 000 000 нс.
    near(
        samples[1].1,
        100.0 * 0.001_f64.powi(2) * (1.0 + 2.0 + 3.0),
        1e-12,
    );
    assert!(
        rt.simplifications()
            .iter()
            .any(|s| s.subject == "отсчёт" && s.blocks_proof),
        "показание {:?} использует позу на 3003 нс старше",
        samples[1]
    );
}

#[test]
fn encoder_crosses_counter_boundary_in_both_directions_over_multiple_turns() {
    let dir = CatalogDir::new("перенос");
    dir.add(
        "wrapped",
        SensorKind::Encoder,
        &[
            ("encoder_mode", 1.0),
            ("encoder_period", TAU),
            ("quantum", FRAC_PI_2),
        ],
    );
    dir.add("direct", SensorKind::Encoder, &[]);
    dir.add(
        "relative",
        SensorKind::Encoder,
        &[("encoder_mode", 2.0), ("encoder_zero", 0.25)],
    );
    let mut scene = world(
        ROTOR,
        "0 0 0",
        vec![
            encoder("цикл", "wrapped"),
            encoder("ось", "direct"),
            encoder("относительный", "relative"),
        ],
    );
    effort(&mut scene, CompiledJointKind::Hinge);
    let catalog = dir.catalog();
    for direction in [1.0, -1.0] {
        let mut rt = Runtime::load(scene.clone()).unwrap();
        let handles = install(&mut rt, &scene, &catalog).unwrap();
        // Импульс на I=1 кг·м² за 1 мс даёт 100 рад/с, то есть 1 рад на отсчёт.
        drive(&mut rt, direction * 100_000.0);
        run(&mut rt, 0.001);
        drive(&mut rt, 0.0);
        run(&mut rt, 0.2);
        let logs: Vec<_> = handles
            .iter()
            .map(|h| observations(&rt, h.signal("value").unwrap()))
            .collect();
        assert!(logs.iter().all(|log| log.len() == 21));
        for i in 0..=20 {
            let q = direction * i as f64;
            near(logs[1][i].1, q, 1e-11);
            near(logs[2][i].1, q - 0.25, 1e-11);
            let expected = (q / FRAC_PI_2).round().rem_euclid(4.0) * FRAC_PI_2;
            near(logs[0][i].1, expected, 1e-11);
            assert!((0.0..TAU).contains(&logs[0][i].1));
        }
    }
}

#[test]
fn fault_during_exposure_invalidates_the_frame_before_delayed_delivery() {
    let dir = CatalogDir::new("обрыв-в-экспозиции");
    for (part, fault) in [("open", 2.0), ("stuck", 1.0)] {
        dir.add(
            part,
            SensorKind::ColorCamera,
            &[
                ("width", 1.0),
                ("height", 1.0),
                ("cx", 0.5),
                ("cy", 0.5),
                ("exposure_s", 0.004),
                ("latency_s", 0.003),
                ("fault", fault),
                ("fault_at_s", 0.012),
            ],
        );
    }
    let scene = world(
        r#"<geom type="sphere" size=".1" pos="0 0 2" rgba="1 0 0 1"/>"#,
        "0 0 0",
        vec![sensor("обрыв", "open"), sensor("залипание", "stuck")],
    );
    let mut rt = Runtime::load(scene.clone()).unwrap();
    let h = install(&mut rt, &scene, &dir.catalog()).unwrap();
    run(&mut rt, 0.006999999);
    assert!(rt.observe().frames.is_empty());
    run(&mut rt, 0.016999999);
    let first = rt.observe().frames;
    assert_eq!(first.len(), 2);
    assert!(first.iter().all(|f| f.valid
        && f.captured == Time::ZERO
        && f.at == Time(7_000_000)
        && f.data.as_ref() == [255, 0, 0]));
    run(&mut rt, 0.017);
    let obs = rt.observe();
    for (i, expected_payload) in [[0, 0, 0], [255, 0, 0]].into_iter().enumerate() {
        let f = obs.frame(h[i].signal("image").unwrap()).unwrap();
        assert_eq!(
            (f.at, f.valid, f.data.as_ref()),
            (Time(17_000_000), false, expected_payload.as_slice())
        );
        assert_eq!(
            f.captured,
            if i == 0 { Time(10_000_000) } else { Time::ZERO }
        );
    }
}

#[derive(Clone)]
struct WithTruth<F>(FrameTruthReader, F);

impl<F: Fn(&FrameTruth) + Clone> Evaluator for WithTruth<F> {
    fn evaluate(&mut self, truth: &Truth<'_>, _: &mut Rng, _: &mut TruthOut) -> Result<(), String> {
        for frame in self.0.advance(truth)? {
            self.1(&frame);
        }
        Ok(())
    }
}

#[test]
fn rolling_rows_keep_rotating_geometry_rigid_between_physics_ticks() {
    let dir = CatalogDir::new("дуга-стержня");
    dir.add(
        "rolling",
        SensorKind::ColorCamera,
        &[
            ("width", 64.0),
            ("height", 3.0),
            ("fx", 40.0),
            ("fy", 40.0),
            ("cx", 32.0),
            ("cy", 1.5 - 20.0 * 0.5_f64.sin()),
            ("readout_s", 0.001),
        ],
    );
    let mut scene = world(ROTOR, "0 0 0", vec![sensor("кадр", "rolling")]);
    effort(&mut scene, CompiledJointKind::Hinge);
    let mut rt = Runtime::load(scene.clone()).unwrap();
    let h = &install(&mut rt, &scene, &dir.catalog()).unwrap()[0];
    let check = |f: &FrameTruth| {
        assert_eq!(
            f.row_times.as_ref(),
            [Time(0), Time(500_000), Time(1_000_000)]
        );
        assert_eq!(f.reference_at, Time(500_000));
        let [x, y, z] = f
            .objects
            .iter()
            .find(|o| o.name == "блок")
            .unwrap()
            .pose
            .position_m;
        // За шаг стержень поворачивается на 1 рад; в средней строке угол 0.5 рад.
        near(x, 0.5_f64.cos(), 1e-12);
        near(y, 0.5_f64.sin(), 1e-12);
        near(z, 2.0, 1e-12);
        near(x.hypot(y), 1.0, 1e-12);
    };
    rt.register_evaluator(
        "сверка дуги",
        scene.physics,
        WithTruth(h.frame_truth_reader().unwrap(), check),
    )
    .unwrap();
    drive(&mut rt, 1_000_000.0);
    run(&mut rt, 0.001);
    let obs = rt.observe();
    let frame = obs.frame(h.signal("image").unwrap()).unwrap();
    assert!(frame.valid);
    // Луч через центр пикселя 49 средней строки проходит через сферу на дуге.
    assert_eq!(frame.data[(64 + 49) * 3], 255);
}

#[test]
fn malformed_truth_cannot_turn_one_visible_pixel_into_zero_missed_targets() {
    let pose = Pose {
        position_m: [0.0, 0.0, 2.0],
        quat_wxyz: [1.0, 0.0, 0.0, 0.0],
    };
    let frame = FrameTruth {
        signal: SignalId(0),
        captured: Time::ZERO,
        available: Time::ZERO,
        width: 1,
        height: 1,
        depth_m: vec![Some(2.0)].into(),
        object_ids: vec![Some(7)].into(),
        row_times: vec![Time::ZERO].into(),
        reference_at: Time::ZERO,
        camera_pose: pose,
        objects: vec![ObjectPose {
            id: 7,
            name: "блок".into(),
            body: 0,
            body_name: String::new(),
            pose,
        }]
        .into(),
    };
    assert_eq!(
        compare_detections(&frame, &[7], &[], 1).unwrap().missed,
        [7]
    );
    for corrupt in [
        FrameTruth {
            object_ids: vec![].into(),
            ..frame.clone()
        },
        FrameTruth {
            object_ids: vec![Some(8)].into(),
            ..frame.clone()
        },
        FrameTruth {
            depth_m: vec![Some(f64::NAN)].into(),
            ..frame.clone()
        },
        FrameTruth {
            depth_m: vec![None].into(),
            ..frame.clone()
        },
        FrameTruth {
            camera_pose: Pose {
                position_m: [f64::INFINITY; 3],
                ..pose
            },
            ..frame.clone()
        },
    ] {
        let score = compare_detections(&corrupt, &[7], &[], 1);
        assert!(score.is_err(), "некорректная истина дала {score:?}");
    }
}

#[test]
fn controller_gets_only_sensor_payload_and_requested_errors() {
    let dir = CatalogDir::new("наблюдение");
    let tiny = [("width", 1.0), ("height", 1.0), ("cx", 0.5), ("cy", 0.5)];
    dir.add("rgb", SensorKind::ColorCamera, &tiny);
    let depth = [tiny.as_slice(), &[("bias", 0.125)]].concat();
    dir.add("depth", SensorKind::DepthCamera, &depth);
    dir.add("biased", SensorKind::Encoder, &[("bias", 0.25)]);
    let scene = world(
        &format!("{SLIDE}<geom type='plane' size='2 2 .1' pos='0 0 2' rgba='1 0 0 1'/>"),
        "0 0 0",
        vec![
            sensor("цвет", "rgb"),
            sensor("глубина", "depth"),
            encoder("отсчёт", "biased"),
        ],
    );
    let mut rt = Runtime::load(scene.clone()).unwrap();
    rt.record_joint_state("истинные координаты").unwrap();
    let h = install(&mut rt, &scene, &dir.catalog()).unwrap();
    let rgb = h[0].signal("image").unwrap();
    let depth = h[1].signal("depth").unwrap();
    let value = h[2].signal("value").unwrap();
    rt.register_evaluator(
        "истина глубины",
        scene.physics,
        WithTruth(h[1].frame_truth_reader().unwrap(), |frame: &FrameTruth| {
            assert_eq!(frame.depth_m.as_ref(), [Some(2.0)]);
            assert!(frame.object_ids[0].is_some());
        }),
    )
    .unwrap();
    rt.register_controller("контроль", Rate::hz(100).unwrap(), move |obs| {
        assert_eq!(obs.frames.len(), 2);
        assert_eq!(obs.samples.len(), 1);
        assert_eq!(obs.samples[0].signal, value);
        assert_eq!(obs.latest(value), Some(0.25));
        let color = obs.frame(rgb).unwrap();
        assert_eq!(
            (color.format, color.data.as_ref()),
            (PixelFormat::Rgb8, [255, 0, 0].as_slice())
        );
        let measured = obs.frame(depth).unwrap();
        assert_eq!(
            (measured.format, measured.data.as_ref()),
            (PixelFormat::DepthMm16, 2125_u16.to_le_bytes().as_slice())
        );
        assert!(color.valid && measured.valid);
        Command::default()
    })
    .unwrap();
    run(&mut rt, 0.0);
    assert_eq!(
        rt.log()
            .entries()
            .iter()
            .filter(|e| e.kind == LogKind::Truth)
            .count(),
        2
    );
}

#[test]
fn frame_rendering_and_call_batching_do_not_change_contact_trajectory() {
    let dir = CatalogDir::new("контакт-и-кадр");
    dir.add(
        "rgb",
        SensorKind::ColorCamera,
        &[
            ("rate_hz", 100.0),
            ("exposure_s", 0.003),
            ("exposure_samples", 3.0),
        ],
    );
    let scene = world(
        r#"
        <geom type="plane" size="2 2 .1" pos="0 0 3" rgba="1 0 0 1"/>
        <body name="блок" pos=".1 0 3.11" euler="12 20 0">
          <freejoint/><geom type="box" size=".1 .08 .05" mass="1"/>
        </body>"#,
        "0 0 -9.81",
        vec![sensor("кадр", "rgb")],
    );
    let catalog = dir.catalog();
    let mut no_camera = Runtime::load(scene.clone()).unwrap();
    let mut bulk = Runtime::load(scene.clone()).unwrap();
    let mut incremental = Runtime::load(scene.clone()).unwrap();
    let first = install(&mut bulk, &scene, &catalog).unwrap()[0]
        .signal("image")
        .unwrap();
    let second = install(&mut incremental, &scene, &catalog).unwrap()[0]
        .signal("image")
        .unwrap();
    let initial = no_camera.log().end().state_hash;
    for i in 1..=200 {
        let t = f64::from(i) / 1000.0;
        run(&mut no_camera, t);
        run(&mut incremental, t);
        assert_eq!(
            no_camera.log().end().state_hash,
            incremental.log().end().state_hash,
            "физика на такте {i}"
        );
    }
    assert_ne!(initial, incremental.log().end().state_hash);
    run(&mut bulk, 0.2);
    assert_eq!(
        bulk.log().end().state_hash,
        no_camera.log().end().state_hash
    );
    assert_eq!(bulk.log().fingerprint(), incremental.log().fingerprint());
    assert_eq!(
        bulk.observe().frame(first),
        incremental.observe().frame(second)
    );
    let frames = observations(&bulk, first);
    assert_eq!(frames.len(), 20);
    assert!(frames.iter().all(|&(_, _, valid)| valid));
    assert!(
        frames
            .windows(2)
            .any(|w| w[0].1.to_bits() != w[1].1.to_bits())
    );
}

#[test]
fn off_axis_depth_clips_near_far_and_behind_before_resolving_occlusion() {
    let dir = CatalogDir::new("отсечение");
    dir.add(
        "depth",
        SensorKind::DepthCamera,
        &[
            ("width", 3.0),
            ("height", 1.0),
            ("cx", 1.5),
            ("cy", 0.5),
            ("fx", 2.0),
            ("fy", 2.0),
            ("near_m", 0.5),
            ("far_m", 4.0),
            ("k1", 0.2),
        ],
    );
    // Два дальних блока на том же луче, узкая ближняя сфера за near и плоскости вне диапазона.
    let scene = world(
        r#"
        <geom type="plane" size="3 3 .1" pos="0 0 -1"/>
        <geom type="plane" size="3 3 .1" pos="0 0 5"/>
        <geom type="sphere" size=".08" pos="0 0 .2"/>
        <geom type="box" size=".14 .14 .1" pos="0 0 2.1"/>
        <geom type="box" size=".25 .2 .1" pos="0 0 3.1"/>
        <geom type="box" size=".02 .14 .1" pos=".9563 0 2.1"/>"#,
        "0 0 0",
        vec![sensor("глубина", "depth")],
    );
    let mut rt = Runtime::load(scene.clone()).unwrap();
    let h = install(&mut rt, &scene, &dir.catalog()).unwrap();
    let lens = h[0].intrinsics().unwrap();
    assert_eq!(lens.project([0.0, 0.0, -1.0]).unwrap(), None);
    // x/z = 0.47815: Браун r_d = r + 0.2r³, положение у правого края.
    let r = 0.9563_f64 / 2.0;
    near(
        lens.project([0.9563, 0.0, 2.0]).unwrap().unwrap()[0],
        1.5 + 2.0 * (r + 0.2 * r.powi(3)),
        1e-12,
    );
    near(1.5 + 2.0 * (r + 0.2 * r.powi(3)), 2.5, 0.001);
    run(&mut rt, 0.0);
    let obs = rt.observe();
    let frame = obs.frame(h[0].signal("depth").unwrap()).unwrap();
    assert!(frame.valid);
    assert_eq!(depth_mm(frame), [0, 2000, 2000]);
    assert!((2.0 * r.hypot(1.0) - 2.0) > 0.2); // При дальности вдоль луча вышло бы около 2217 мм.
}

#[test]
fn white_noise_streams_are_independent_and_walk_scales_at_both_rates() {
    let dir = CatalogDir::new("частоты-и-потоки");
    for (part, hz) in [("slow", 100.0), ("fast", 1000.0)] {
        dir.add(
            part,
            SensorKind::Encoder,
            &[("drift_rw_std", 0.7), ("rate_hz", hz)],
        );
    }
    dir.add("noise", SensorKind::Encoder, &[("noise_std", 0.4)]);
    let scene = world(
        SLIDE,
        "0 0 0",
        vec![
            encoder("медленный", "slow"),
            encoder("быстрый", "fast"),
            encoder("первый", "noise"),
            encoder("второй", "noise"),
        ],
    );
    let catalog = dir.catalog();
    let measure = |names: &[&str]| {
        let mut scene = scene.clone();
        scene.sensors.retain(|s| names.contains(&s.name.as_str()));
        let mut rt = Runtime::load(scene.clone()).unwrap();
        let h = install(&mut rt, &scene, &catalog).unwrap();
        run(&mut rt, 40.0);
        h.iter()
            .map(|h| observations(&rt, h.signal("value").unwrap()))
            .collect::<Vec<_>>()
    };
    let simultaneous = measure(&["медленный", "быстрый", "первый", "второй"]);
    let bits =
        |samples: &[(Time, f64, bool)]| samples.iter().map(|s| s.1.to_bits()).collect::<Vec<_>>();
    assert_eq!(bits(&simultaneous[2]), bits(&measure(&["первый"])[0]));
    assert_ne!(bits(&simultaneous[2]), bits(&simultaneous[3]));

    for (samples, stride) in simultaneous.iter().zip([10_usize, 100]) {
        // Непересекающиеся окна 0.1 с, ожидаем E[Δ²] = 0.7² × 0.1 при обеих частотах.
        let steps: Vec<_> = samples.iter().step_by(stride).collect();
        assert_eq!(steps.len(), 401);
        assert!(steps.iter().all(|s| s.2));
        let n = (steps.len() - 1) as f64;
        let moment = steps
            .windows(2)
            .map(|p| (p[1].1 - p[0].1).powi(2))
            .sum::<f64>()
            / n;
        let expected = 0.7_f64.powi(2) * 0.1;
        near(moment, expected, 5.0 * expected * (2.0 / n).sqrt());
    }
    let a = &simultaneous[2];
    let b = &simultaneous[3];
    let n = a.len() as f64;
    let covariance = a.iter().zip(b).map(|(a, b)| a.1 * b.1).sum::<f64>() / n;
    near(covariance, 0.0, 5.0 * 0.4_f64.powi(2) / n.sqrt());
}

#[test]
fn arbitrary_imu_frame_sees_specific_force_at_rest_and_zero_in_free_fall() {
    let dir = CatalogDir::new("свободное-падение");
    dir.add("imu", SensorKind::Imu, &[("rate_hz", 1000.0)]);
    let mut fixed = sensor("покой", "imu");
    fixed.quat_wxyz = [1.0, 2.0, 3.0, 4.0];
    let mut falling = fixed.clone();
    falling.name = "падение".into();
    falling.body = Some("блок".into());
    let scene = world(
        r#"<body name="блок"><freejoint/>
           <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
           </body>"#,
        "2 -3 -6",
        vec![fixed, falling],
    );
    let mut rt = Runtime::load(scene.clone()).unwrap();
    let h = install(&mut rt, &scene, &dir.catalog()).unwrap();
    run(&mut rt, 0.005);
    // При q = (1,2,3,4)/√30 столбцы R равны
    // (-20,20,10), (4,-10,28), (22,20,4) / 30. Скалярные произведения с -g.
    let fixed_expected = [160.0 / 30.0, 130.0 / 30.0, 40.0 / 30.0, 0.0, 0.0, 0.0];
    for (handle, expected) in h.iter().zip([fixed_expected, [0.0; 6]]) {
        for ((_, id), expected) in handle.signals().iter().zip(expected) {
            let samples = observations(&rt, *id);
            assert_eq!(samples.len(), 6);
            for (_, actual, valid) in samples {
                assert!(valid);
                near(actual, expected, 1e-12);
            }
        }
    }
}

#[test]
fn rotating_camera_measures_each_row_along_its_own_optical_axis() {
    let dir = CatalogDir::new("вращение-оптики");
    dir.add(
        "depth",
        SensorKind::DepthCamera,
        &[
            ("width", 1.0),
            ("height", 3.0),
            ("cx", 0.5),
            ("cy", 1.5),
            ("readout_s", 0.001),
            ("latency_s", 0.002),
        ],
    );
    let mut camera = sensor("глубина", "depth");
    camera.body = Some("стержень".into());
    let mut scene = world(
        r#"
        <body name="стержень">
          <joint name="ось" axis="0 1 0"/>
          <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
        </body>
        <geom type="plane" size="3 3 .1" pos="0 0 2"/>"#,
        "0 0 0",
        vec![camera],
    );
    effort(&mut scene, CompiledJointKind::Hinge);
    let mut rt = Runtime::load(scene.clone()).unwrap();
    let h = &install(&mut rt, &scene, &dir.catalog()).unwrap()[0];
    let id = h.signal("depth").unwrap();
    let expected = [0.0_f64, 0.3, 0.6].map(|angle| 2.0 / angle.cos());
    rt.register_evaluator(
        "поворот",
        scene.physics,
        WithTruth(h.frame_truth_reader().unwrap(), move |f: &FrameTruth| {
            assert_eq!(
                f.row_times.as_ref(),
                [Time(0), Time(500_000), Time(1_000_000)]
            );
            near(f.camera_pose.quat_wxyz[2], 0.15_f64.sin(), 1e-12);
            for (depth, expected) in f.depth_m.iter().zip(expected) {
                near(depth.unwrap(), expected, 1e-12);
            }
        }),
    )
    .unwrap();
    // 600 000 Н·м за 1 мс, I=1 кг·м²: поворот 0.6 рад к последней строке.
    drive(&mut rt, 600_000.0);
    run(&mut rt, 0.002999999);
    assert!(rt.observe().frame(id).is_none());
    run(&mut rt, 0.003);
    let obs = rt.observe();
    let f = obs.frame(id).unwrap();
    assert_eq!(
        (f.at, f.captured, f.valid),
        (Time(3_000_000), Time::ZERO, true)
    );
    assert_eq!(
        depth_mm(f),
        expected.map(|depth| (depth * 1000.0).round() as u16)
    );
}

#[test]
fn known_estimate_and_instance_override_each_block_proof() {
    let dir = CatalogDir::new("источник-числа");
    dir.add("reference", SensorKind::Encoder, &[("bias", 0.3)]);
    let mut estimate = dir.catalog().get("reference").unwrap().clone();
    estimate.id = "estimate".into();
    let sim_sense::Parameter::Known(bias) = estimate.model.get_mut("bias").unwrap() else {
        panic!("в стенде задано смещение 0.3 м");
    };
    bias.provenance = sim_sense::SourceKind::Estimate;
    dir.write("estimate", &toml::to_string(&estimate).unwrap());
    let mut instance = encoder("экземпляр", "reference");
    instance.params.insert("bias".into(), 0.4);
    let scene = world(
        SLIDE,
        "0 0 0",
        vec![
            encoder("измеренный", "reference"),
            encoder("оценка", "estimate"),
            instance,
        ],
    );
    let mut rt = Runtime::load(scene.clone()).unwrap();
    let h = install(&mut rt, &scene, &dir.catalog()).unwrap();
    run(&mut rt, 0.0);
    for (handle, expected) in h.iter().zip([0.3, 0.3, 0.4]) {
        assert_eq!(
            rt.observe().latest(handle.signal("value").unwrap()),
            Some(expected)
        );
    }
    let blocked: Vec<_> = rt
        .simplifications()
        .iter()
        .filter(|s| s.blocks_proof)
        .map(|s| s.subject.as_str())
        .collect();
    assert_eq!(blocked, ["оценка", "экземпляр"]);
}
