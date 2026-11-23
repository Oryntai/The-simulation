mod common;

use std::time::Instant;

use sim_runtime::{Evaluator, LogKind, Runtime, Truth, TruthId, TruthOut};
use sim_sense::{FrameTruthReader, SensorKind, install};
use sim_signal::{Command, ImageFrame, Rng, Time};
use sim_world::{CompiledJointKind, CompiledSensor, CompiledWorld};

use common::{CatalogDir, SLIDE, effort, near, run, sensor, world};

fn scene(geometry: &str, sensors: Vec<CompiledSensor>) -> CompiledWorld {
    let mut scene = world(&format!("{SLIDE}{geometry}"), "0 0 0", sensors);
    effort(&mut scene, CompiledJointKind::Slide);
    scene
}

fn moving_camera(name: &str, part: &str) -> CompiledSensor {
    let mut s = sensor(name, part);
    s.body = Some("каретка".into());
    s
}

fn kick(rt: &mut Runtime, velocity_m_s: f64) {
    // Импульс на 1 кг за 1 мс, после которого каретка идёт с постоянной скоростью.
    let channel = rt.channel("вход").unwrap();
    rt.act(&Command {
        targets: vec![(channel, velocity_m_s / 0.001)],
    })
    .unwrap();
    run(rt, 0.001);
    rt.act(&Command {
        targets: vec![(channel, 0.0)],
    })
    .unwrap();
}

fn red(frame: &ImageFrame, x: usize, y: usize) -> f64 {
    f64::from(frame.data[(y * frame.width as usize + x) * 3]) / f64::from(u8::MAX)
}

fn first_edge(frame: &ImageFrame, row: usize) -> f64 {
    // Белая кромка заполняет строку вправо. Интеграл покрытия равен width-x.
    let width = frame.width as usize;
    assert_eq!(red(frame, 0, row), 0.0);
    assert_eq!(red(frame, width - 1, row), 1.0);
    width as f64 - (0..width).map(|x| red(frame, x, row)).sum::<f64>()
}

#[derive(Clone)]
struct MotionTruth {
    reader: FrameTruthReader,
    checked: TruthId,
}

impl Evaluator for MotionTruth {
    fn evaluate(
        &mut self,
        truth: &Truth<'_>,
        _: &mut Rng,
        out: &mut TruthOut,
    ) -> Result<(), String> {
        for f in self.reader.advance(truth)? {
            assert_eq!(f.row_times[0], f.captured);
            assert_eq!(
                *f.row_times.last().unwrap(),
                Time(f.captured.0 + 40_000_000)
            );
            assert_eq!(f.reference_at, Time(f.captured.0 + 20_000_000));
            assert_eq!(f.available, Time(f.captured.0 + 47_500_000));
            near(
                f.camera_pose.position_m[0],
                0.025 + 2.0 * f.reference_at.as_secs(),
                1e-12,
            );
            let point = f.objects.iter().find(|g| g.name == "кромка").unwrap();
            near(point.pose.position_m[2], 2.1, 1e-12);
            out.write(self.checked, f.camera_pose.position_m[0]);
        }
        Ok(())
    }
}

#[test]
fn moving_mount_and_rolling_shutter_tilt_a_vertical_edge_by_the_calculated_distance() {
    let dir = CatalogDir::new("затвор");
    let base = [("rate_hz", 20.0), ("fx", 100.0), ("spatial_samples", 8.0)];
    dir.add("static", SensorKind::ColorCamera, &base);
    let mut rolling = base.to_vec();
    rolling.extend([
        ("readout_s", 0.04),
        ("latency_s", 0.0075),
        ("mount_dx_m", 0.025),
    ]);
    dir.add("rolling", SensorKind::ColorCamera, &rolling);
    let world = scene(
        r#"<geom name="кромка" type="box" size=".7 3 .1" pos=".7 0 2.1" rgba="1 1 1 1"/>"#,
        vec![
            sensor("неподвижная", "static"),
            moving_camera("подвижная", "rolling"),
        ],
    );
    let mut rt = Runtime::load(world.clone()).unwrap();
    let h = install(&mut rt, &world, &dir.catalog()).unwrap();
    let static_id = h[0].signal("image").unwrap();
    let rolling_id = h[1].signal("image").unwrap();
    let checked = rt.register_truth("проверено", "m").unwrap();
    rt.register_evaluator(
        "истина движения",
        world.physics,
        MotionTruth {
            reader: h[1].frame_truth_reader().unwrap(),
            checked,
        },
    )
    .unwrap();
    kick(&mut rt, 2.0);
    run(&mut rt, 0.097499999);
    assert_eq!(rt.observe().frame(rolling_id).unwrap().captured, Time::ZERO);
    run(&mut rt, 0.0975);
    let obs = rt.observe();
    let still = obs.frame(static_id).unwrap();
    let moved = obs.frame(rolling_id).unwrap();
    assert!(still.valid && moved.valid);
    assert_eq!(moved.at, Time(97_500_000));
    assert_eq!(moved.captured, Time(50_000_000));
    let top = first_edge(moved, 0);
    let bottom = first_edge(moved, moved.height as usize - 1);
    // z=2 м, vx=2 м/с, fx=100 px. Первую строку смещают ход и поправка установки.
    near(first_edge(still, 0), 32.0, 0.1);
    near(first_edge(still, still.height as usize - 1), 32.0, 0.1);
    near(top, 32.0 - 100.0 * (0.025 + 2.0 * 0.05) / 2.0, 0.1);
    near(bottom - top, -100.0 * 2.0 * 0.04 / 2.0, 0.1);
    assert_eq!(
        rt.log()
            .entries()
            .iter()
            .filter(|e| e.kind == LogKind::Truth && e.id == 0)
            .count(),
        2
    );
}

#[test]
fn exposure_smears_a_one_pixel_target_over_eight_pixels_of_motion() {
    let dir = CatalogDir::new("смаз");
    let base = [
        ("rate_hz", 20.0),
        ("fx", 100.0),
        ("cx", 32.5),
        ("spatial_samples", 4.0),
    ];
    dir.add("sharp", SensorKind::ColorCamera, &base);
    let mut blur = base.to_vec();
    blur.extend([("exposure_s", 0.04), ("exposure_samples", 40.0)]);
    dir.add("blur", SensorKind::ColorCamera, &blur);
    let world = scene(
        r#"<geom name="точка" type="box" size=".01 .15 .0001" pos="0 0 2.0001" rgba="1 1 1 1"/>"#,
        vec![
            moving_camera("короткая", "sharp"),
            moving_camera("длинная", "blur"),
        ],
    );
    let mut rt = Runtime::load(world.clone()).unwrap();
    let h = install(&mut rt, &world, &dir.catalog()).unwrap();
    kick(&mut rt, 4.0);
    run(&mut rt, 0.09);
    let obs = rt.observe();
    let a = obs.frame(h[0].signal("image").unwrap()).unwrap();
    let b = obs.frame(h[1].signal("image").unwrap()).unwrap();
    assert!(a.valid && b.valid);
    assert_eq!(
        (a.captured, b.captured),
        (Time(50_000_000), Time(50_000_000))
    );
    assert_eq!(b.at, Time(90_000_000));
    // z=2 м, fx=100 px и vx=4 м/с: центр идёт от 22.5 до 14.5 px.
    // Свёртка прямоугольника ширины 1 с равномерной экспозицией длины 8
    // даёт края по 1/16, внутренние пиксели по 1/8, площадь остаётся равной 1.
    for x in 0..a.width as usize {
        near(red(a, x, 24), if x == 22 { 1.0 } else { 0.0 }, 1.0 / 255.0);
        let expected = match x {
            14 | 22 => 1.0 / 16.0,
            15..=21 => 1.0 / 8.0,
            _ => 0.0,
        };
        near(red(b, x, 24), expected, 0.012);
    }
    near(
        (0..b.width as usize).map(|x| red(b, x, 24)).sum(),
        1.0,
        0.025,
    );
}

fn binary(rt: &Runtime) -> Vec<u8> {
    let mut bytes = Vec::new();
    rt.log().write_binary(&mut bytes).unwrap();
    bytes
}

#[test]
fn multi_camera_logs_payloads_and_mid_exposure_snapshot_are_bitwise_repeatable() {
    let dir = CatalogDir::new("снимок");
    dir.add(
        "rgb",
        SensorKind::ColorCamera,
        &[
            ("rate_hz", 20.0),
            ("width", 32.0),
            ("height", 20.0),
            ("k1", 0.1),
            ("readout_s", 0.01),
            ("exposure_s", 0.03),
            ("exposure_samples", 3.0),
            ("latency_s", 0.065),
            ("noise_std", 0.04),
            ("drop_probability", 0.15),
            ("drift_rw_std", 0.13),
        ],
    );
    dir.add(
        "depth",
        SensorKind::DepthCamera,
        &[
            ("rate_hz", 25.0),
            ("width", 47.0),
            ("height", 23.0),
            ("readout_s", 0.005),
            ("exposure_s", 0.017),
            ("exposure_samples", 3.0),
            ("latency_s", 0.0205),
            ("hole_probability", 0.32),
            ("noise_std", 0.006),
        ],
    );
    let world = scene(
        r#"<geom name="блок" type="box" size=".5 .5 .2" pos="0 0 3" rgba="0.5 0.3 1 1"/>
                         <geom type="plane" size="10 10 .1" pos="0 0 5"/>"#,
        vec![moving_camera("цвет", "rgb"), sensor("глубина", "depth")],
    );
    let cat = dir.catalog();
    let configured = || {
        let mut rt = Runtime::load(world.clone()).unwrap();
        install(&mut rt, &world, &cat).unwrap();
        rt
    };
    let load = || {
        let mut rt = configured();
        kick(&mut rt, 2.0);
        rt
    };
    let mut a = load();
    let mut b = load();
    run(&mut a, 0.7);
    run(&mut b, 0.078); // В очереди доставка кадров, а следующая экспозиция не закрыта.
    let snapshot = b.snapshot();
    let mut branch = b.fork().unwrap();
    run(&mut b, 0.7);
    run(&mut branch, 0.7);
    let expected = binary(&a);
    assert_eq!(expected, binary(&b));
    assert_eq!(expected, binary(&branch));
    assert_eq!(a.observe(), b.observe());
    assert_eq!(a.observe(), branch.observe());

    b.restore(&snapshot).unwrap();
    run(&mut b, 0.7);
    assert_eq!(binary(&b), expected);
    let obs = a.observe();
    assert_eq!(obs, b.observe());
    assert_eq!(obs.frames.len(), 2);
    for name in ["цвет/image", "глубина/depth"] {
        let signal = a.observation_id(name).unwrap();
        assert!(
            a.log()
                .entries()
                .iter()
                .filter(|e| e.id == signal.0 && e.kind == LogKind::Observation && e.valid)
                .count()
                > 5
        );
        assert!(obs.frame(signal).unwrap().data.iter().any(|&b| b > 0));
    }
    let mut replay = configured();
    replay.replay(a.log().clone()).unwrap();
    run(&mut replay, 0.7);
    replay.verify_replay().unwrap();
    assert_eq!(expected, binary(&replay));
    assert_eq!(obs, replay.observe());
}

#[test]
fn frame_failures_preserve_payload_or_invalidate_it_at_the_configured_time() {
    let dir = CatalogDir::new("отказы-кадра");
    dir.add(
        "stuck",
        SensorKind::ColorCamera,
        &[("fault", 1.0), ("fault_at_s", 0.01)],
    );
    dir.add(
        "open",
        SensorKind::ColorCamera,
        &[("fault", 2.0), ("fault_at_s", 0.01)],
    );
    dir.add(
        "shift",
        SensorKind::ColorCamera,
        &[("fault", 3.0), ("fault_at_s", 0.01), ("fault_offset", 0.2)],
    );
    let world = scene(
        r#"<geom type="sphere" size=".13" pos="0 0 2" rgba="0.5 0.5 0.5 1"/>"#,
        vec![
            moving_camera("залипание", "stuck"),
            moving_camera("обрыв", "open"),
            moving_camera("смещение", "shift"),
        ],
    );
    let mut rt = Runtime::load(world.clone()).unwrap();
    install(&mut rt, &world, &dir.catalog()).unwrap();
    kick(&mut rt, 20.0);
    let before = rt.observe().frames;
    run(&mut rt, 0.01);
    let after = rt.observe().frames;
    assert_eq!(before.len(), 3);
    assert_eq!(after.len(), 3);
    assert!(
        before
            .iter()
            .all(|frame| frame.valid && frame.data.iter().any(|&byte| byte != 0))
    );
    assert!(after.iter().all(|frame| frame.at == Time(10_000_000)));
    assert!(
        after[1..]
            .iter()
            .all(|frame| frame.captured == Time(10_000_000))
    );
    assert_eq!(after[0].captured, before[0].captured);
    assert!(!after[0].valid && before[0].data == after[0].data);
    assert!(!after[1].valid && after[1].data.iter().all(|&b| b == 0));
    assert!(after[2].valid && after[2].data != before[2].data);
    assert_eq!(after[2].data[0], 51); // 0.2 белой шкалы RGB8 вне объекта.
}

#[test]
fn nonintegral_camera_period_keeps_its_phase_and_delivery_within_a_physics_tick() {
    let dir = CatalogDir::new("фаза");
    dir.add(
        "camera",
        SensorKind::DepthCamera,
        &[
            ("rate_hz", 300.0),
            ("latency_s", 0.0001),
            ("width", 7.0),
            ("height", 5.0),
            ("cx", 3.5),
            ("cy", 2.5),
        ],
    );
    let world = world(
        r#"<geom type="plane" size="1 1 .1" pos="0 0 3"/>"#,
        "0 0 0",
        vec![sensor("кадр", "camera")],
    );
    let mut rt = Runtime::load(world.clone()).unwrap();
    let id = install(&mut rt, &world, &dir.catalog()).unwrap()[0]
        .signal("depth")
        .unwrap();
    // Каждый захват рассчитан как k/300 секунды, а готовность наступает
    // на ближайшем шаге физики, если паспортный момент доставки раньше него.
    for k in 0..=9_000 {
        let captured = Time(k * 1_000_000_000 / 300);
        let due = Time(captured.0 + 100_000);
        let available = due.max(Time(captured.0.div_ceil(1_000_000) * 1_000_000));
        run(&mut rt, available.as_secs());
        let obs = rt.observe();
        let frame = obs.frame(id).unwrap();
        assert_eq!(frame.captured, captured);
        assert_eq!(frame.at, available);
        assert!(frame.valid && available.0 - due.0 < 1_000_000);
    }
}

#[test]
fn rasterizes_simple_320_by_240_scene_in_tens_of_milliseconds() {
    let dir = CatalogDir::new("скорость");
    dir.add(
        "camera",
        SensorKind::ColorCamera,
        &[
            ("width", 320.0),
            ("height", 240.0),
            ("fx", 280.0),
            ("fy", 280.0),
            ("cx", 160.0),
            ("cy", 120.0),
        ],
    );
    let world = world(
        r#"<geom type="plane" size="5 5 .1" pos="0 0 4" rgba=".2 .2 .2 1"/>
                         <geom type="sphere" size=".45" pos="0 0 3" rgba="1 1 1 1"/>
                         <geom type="box" size=".3 .2 .4" pos="1 0 3" rgba="1 0 0 1"/>"#,
        "0 0 0",
        vec![sensor("кадр", "camera")],
    );
    let mut rt = Runtime::load(world.clone()).unwrap();
    let id = install(&mut rt, &world, &dir.catalog()).unwrap()[0]
        .signal("image")
        .unwrap();
    run(&mut rt, 0.0);
    let mut times = Vec::new();
    for i in 1..=9 {
        // Часы измеряют длительность в тесте снаружи прогона.
        let before = Instant::now();
        run(&mut rt, i as f64 * 0.01);
        times.push(before.elapsed());
        let obs = rt.observe();
        let frame = obs.frame(id).unwrap();
        assert_eq!(frame.captured, Time(i * 10_000_000));
        assert!(frame.valid);
        assert!(red(frame, 160, 120) > red(frame, 0, 0));
    }
    times.sort_unstable();
    let median = times[times.len() / 2];
    eprintln!(
        "320×240, медиана девяти кадров: {:.3} мс",
        median.as_secs_f64() * 1000.0
    );
    assert!(median.as_millis() < 90);
}
