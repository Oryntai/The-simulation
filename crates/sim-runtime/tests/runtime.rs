use sim_runtime::{
    CommandSource, Evaluator, Forces, Link, Log, LogKind, Object, ObservationOut, Runtime, Sensor,
    Status, Truth, TruthId, TruthOut,
};
use sim_signal::{ChannelId, Command, Observation, Rate, Rng, SignalId, Time, fnv1a};
use sim_world::{
    CompiledBody, CompiledJoint, CompiledJointKind, CompiledWorld, EffortChannel, Provenance,
};

// Каретка массой 2 кг в вакууме. Схема Эйлера проверяется формулой
// v[k+1] = v[k] + F[k] Δt/m, x[k+1] = x[k] + v[k+1] Δt.
fn scene() -> CompiledWorld {
    CompiledWorld {
        name: "каретка".into(),
        seed: 41,
        physics: Rate::hz(1000).unwrap(),
        mjcf: r#"<mujoco model="каретка">
            <option timestep="0.001" gravity="0 0 0" integrator="Euler"/>
            <worldbody><body name="блок">
                <joint name="ход" type="slide" axis="1 0 0"/>
                <geom type="box" size="0.1 0.1 0.1" mass="2"/>
            </body></worldbody>
        </mujoco>"#
            .into(),
        bodies: vec![],
        joints: vec![CompiledJoint {
            name: "ход".into(),
            owner: "каретка".into(),
            kind: CompiledJointKind::Slide,
        }],
        efforts: vec![EffortChannel {
            name: "сила".into(),
            joint: "ход".into(),
            dof: 0,
        }],
        drives: vec![],
        sensors: vec![],
        simplifications: vec![],
    }
}

fn command(value: f64) -> Command {
    Command {
        targets: vec![(ChannelId(0), value)],
    }
}

fn bytes(log: &Log) -> Vec<u8> {
    let mut encoded = Vec::new();
    log.write_binary(&mut encoded).unwrap();
    encoded
}

#[derive(Clone)]
struct Position {
    signal: SignalId,
    bias: f64,
}

impl Sensor for Position {
    fn sample(
        &mut self,
        truth: &Truth<'_>,
        _: &mut Rng,
        out: &mut ObservationOut,
    ) -> Result<(), String> {
        out.write(self.signal, truth.qpos()[0] + self.bias, true);
        Ok(())
    }
}

#[derive(Clone)]
struct NoisyPosition {
    signal: SignalId,
    count: u32,
}

impl Sensor for NoisyPosition {
    fn sample(
        &mut self,
        truth: &Truth<'_>,
        rng: &mut Rng,
        out: &mut ObservationOut,
    ) -> Result<(), String> {
        self.count += 1;
        // Искусственный датчик для проверки пропусков и сохранения кеша пары Бокса — Мюллера.
        out.write(
            self.signal,
            truth.qpos()[0] + rng.normal(),
            !self.count.is_multiple_of(3),
        );
        Ok(())
    }
}

#[derive(Clone)]
struct Transfer(SignalId, SignalId);

impl Link for Transfer {
    fn transfer(
        &mut self,
        input: &Observation,
        _: &mut Rng,
        out: &mut ObservationOut,
    ) -> Result<(), String> {
        if let Some(value) = input.latest(self.0) {
            out.write(self.1, value, true);
        }
        Ok(())
    }
}

#[derive(Clone)]
struct DoubleForce;

impl Object for DoubleForce {
    fn update(
        &mut self,
        _: &Truth<'_>,
        command: &Command,
        _: &mut Rng,
        out: &mut Forces,
    ) -> Result<(), String> {
        for &(channel, value) in &command.targets {
            // Искусственный коэффициент передачи для проверки обхода прямого входа.
            out.set(channel, 2.0 * value);
        }
        Ok(())
    }
}

#[derive(Clone)]
struct Velocity(TruthId);

impl Evaluator for Velocity {
    fn evaluate(
        &mut self,
        truth: &Truth<'_>,
        _: &mut Rng,
        out: &mut TruthOut,
    ) -> Result<(), String> {
        let (_, velocities) = truth.joint("ход").ok_or("нет соединения")?;
        assert_eq!(velocities, truth.qvel());
        assert!(truth.joint("неизвестное").is_none());
        out.write(self.0, velocities[0]);
        Ok(())
    }
}

fn multirate(playback: bool) -> Runtime {
    let mut r = Runtime::load(scene()).unwrap();
    let sensor = r.register_observation("положение", "m").unwrap();
    let received = r.register_observation("принято", "m").unwrap();
    r.register_sensor(
        "шумный датчик",
        Rate::hz(3000).unwrap(),
        NoisyPosition {
            signal: sensor,
            count: 0,
        },
    )
    .unwrap();
    r.register_link(
        "канал связи",
        Rate::hz(2200).unwrap(),
        Transfer(sensor, received),
    )
    .unwrap();
    let mut rng = Rng::stream(r.seed(), "управление");
    r.register_controller(
        "управление",
        Rate::hz(700).unwrap(),
        move |obs: &Observation| {
            assert!(!playback, "при воспроизведении контроллер вызван");
            command(obs.latest(received).unwrap() + rng.normal())
        },
    )
    .unwrap();
    r.register_object(
        "преобразователь",
        Rate::hz(1100).unwrap(),
        &[ChannelId(0)],
        DoubleForce,
    )
    .unwrap();
    r.record_joint_state("проверка движения").unwrap();
    r
}

#[test]
fn two_runs_snapshot_and_branch_are_bitwise_deterministic() {
    let mut straight = multirate(false);
    let mut split = multirate(false);
    let end = Time(23_408_414);
    let middle = Time(7_001_111);
    straight.run_until(end).unwrap();
    split.run_until(middle).unwrap();
    let checkpoint = split.snapshot();
    let before = bytes(split.log());
    let mut branch = split.fork().unwrap();
    assert!(branch.restore(&checkpoint).is_err());
    branch.run_until(Time(8_000_000)).unwrap();
    branch.act(&command(150.0)).unwrap();
    branch.run_until(end).unwrap();
    assert_ne!(
        branch.log().end().state_hash,
        straight.log().end().state_hash
    );
    assert_eq!(bytes(split.log()), before);
    assert_eq!(split.log().end(), checkpoint.end());
    assert_eq!(split.log().entries().len(), checkpoint.log_len());

    split.run_until(end).unwrap();
    let later = split.snapshot();
    let expected = bytes(straight.log());
    assert_eq!(straight.steps(), 23);
    assert!(straight.log().entries().len() > 100);
    assert_eq!(bytes(split.log()), expected);
    split.restore(&checkpoint).unwrap();
    assert_eq!(bytes(split.log()), before);
    split.run_until(end).unwrap();
    assert_eq!(bytes(split.log()), expected);
    split.restore(&checkpoint).unwrap();
    split.restore(&later).unwrap();
    assert_eq!(bytes(split.log()), expected);
}

#[test]
fn начальная_скорость_и_задержка_меняют_движение_и_момент_доставки() {
    use std::collections::BTreeMap;
    let mut rt = Runtime::load(scene()).unwrap();
    let id = rt.register_observation("положение", "m").unwrap();
    rt.register_sensor(
        "датчик положения",
        Rate::hz(1000).unwrap(),
        Position {
            signal: id,
            bias: 0.0,
        },
    )
    .unwrap();
    rt.record_joint_state("запись движения").unwrap();

    let zero = rt.joint_velocity("ход", 0).unwrap();
    assert_eq!(zero, 0.0);
    for invalid in [f64::NAN, f64::INFINITY] {
        assert!(
            rt.set_initial_velocities(&BTreeMap::from([(
                "ход".into(),
                BTreeMap::from([(0, invalid)])
            ),]))
                .is_err()
        );
    }
    assert!(
        rt.set_initial_velocities(&BTreeMap::from([(
            "ход".into(),
            BTreeMap::from([(0, 0.2), (1, 0.1)])
        ),]))
            .is_err()
    );
    assert_eq!(rt.joint_velocity("ход", 0), Some(zero));
    assert_eq!(rt.joint_velocity("ход", 1), None);

    // Исходные условия опыта: v₀ = 0,2 м/с, F = 2 Н, передача занимает 1,5 мс.
    rt.set_initial_velocities(&BTreeMap::from([(
        "ход".into(),
        BTreeMap::from([(0, 0.2)]),
    )]))
    .unwrap();
    let channel = "observation:положение";
    assert!(
        rt.set_channel_delays(&BTreeMap::from([(channel.into(), 1), ("zzz".into(), 1)]))
            .is_err()
    );
    assert_eq!(rt.channel_delay_ns(channel), Some(0));
    rt.set_channel_delays(&BTreeMap::from([(channel.into(), 1_500_000)]))
        .unwrap();
    assert_eq!(rt.channel_delay_ns(channel), Some(1_500_000));
    rt.act(&command(2.0)).unwrap();
    rt.run_until(Time(1_499_999)).unwrap();
    assert!(!rt.channel_delay_observed(channel));
    assert!(rt.observe().latest(id).is_none());
    rt.run_until(Time(1_500_000)).unwrap();
    assert!(rt.channel_delay_observed(channel));
    assert_eq!(rt.observe().latest(id), Some(0.0));
    rt.run_until(Time(4_000_000)).unwrap();
    // x(2 мс) = 2·v₀·Δt + (1+2)·(F/m)·Δt² = 403 мкм.
    assert!((rt.observe().latest(id).unwrap() - 0.000403).abs() < f64::EPSILON);
    assert!((rt.joint_velocity("ход", 0).unwrap() - 0.204).abs() < f64::EPSILON);
    assert!(rt.set_initial_velocities(&BTreeMap::new()).is_err());
    assert!(rt.set_channel_delays(&BTreeMap::new()).is_err());
}

#[test]
fn exact_fractional_deadlines_and_physics_ticks() {
    let mut world = scene();
    world.physics = Rate::hz(3000).unwrap();
    world.mjcf = world.mjcf.replace("0.001", "0.0003333333333333333");
    let mut r = Runtime::load(world).unwrap();
    let fast = r.register_observation("быстрый", "m").unwrap();
    let rational = r.register_observation("несинхронный", "m").unwrap();
    r.register_sensor(
        "частота 44100",
        Rate::hz(44_100).unwrap(),
        Position {
            signal: fast,
            bias: 0.0,
        },
    )
    .unwrap();
    r.register_sensor(
        "дробная частота",
        Rate::ratio(7000, 3).unwrap(),
        Position {
            signal: rational,
            bias: 0.0,
        },
    )
    .unwrap();
    r.record_joint_state("истина").unwrap();
    let end = Time(10_000_000);
    r.run_until(end).unwrap();
    let times = |kind, id| {
        r.log()
            .entries()
            .iter()
            .filter(|e| e.kind == kind && e.id == id)
            .map(|e| e.at.0)
            .collect::<Vec<_>>()
    };
    // Независимая проверка точными дробями времени и числа событий за 10 мс.
    assert_eq!(
        times(LogKind::Observation, fast.0),
        (0..=441).map(|k| k * 10_000_000 / 441).collect::<Vec<_>>()
    );
    assert_eq!(
        times(LogKind::Observation, rational.0),
        (0..=23).map(|k| k * 3_000_000 / 7).collect::<Vec<_>>()
    );
    assert_eq!(
        times(LogKind::Truth, 0),
        (0..=30).map(|k| k * 1_000_000 / 3).collect::<Vec<_>>()
    );
    assert_eq!(r.steps(), 30);
}

#[test]
fn phases_follow_contract_and_command_affects_next_step() {
    let mut r = Runtime::load(scene()).unwrap();
    let measured = r.register_observation("показание", "m").unwrap();
    let linked = r.register_observation("передано", "m").unwrap();
    let velocity = r.register_truth("скорость", "m/s").unwrap();
    let rate = r.physics_rate();
    // Регистрация в обратном порядке фаз. Второй датчик пишет после первого.
    r.register_evaluator("оценка", rate, Velocity(velocity))
        .unwrap();
    r.register_object("модель", rate, &[ChannelId(0)], DoubleForce)
        .unwrap();
    r.register_controller("решение", rate, move |obs: &Observation| {
        let position = obs.latest(linked).unwrap();
        // При начальном положении 0 второй датчик даёт 4, модель прикладывает 8 Н.
        command(position)
    })
    .unwrap();
    r.register_link("связь", rate, Transfer(measured, linked))
        .unwrap();
    r.register_sensor(
        "первый",
        rate,
        Position {
            signal: measured,
            bias: 9.0,
        },
    )
    .unwrap();
    r.register_sensor(
        "второй",
        rate,
        Position {
            signal: measured,
            bias: 4.0,
        },
    )
    .unwrap();

    r.run_until(Time::ZERO).unwrap();
    let zero = r.log().entries();
    assert_eq!(
        zero.iter()
            .map(|e| (e.kind, e.id, e.value))
            .collect::<Vec<_>>(),
        vec![
            (LogKind::Observation, measured.0, 9.0),
            (LogKind::Observation, measured.0, 4.0),
            (LogKind::Observation, linked.0, 4.0),
            (LogKind::Command(CommandSource::Controller(2)), 0, 4.0),
            (LogKind::Truth, 0, 0.0),
        ]
    );
    assert_eq!(r.steps(), 0);
    r.step().unwrap();
    assert_eq!(r.time(), Time(1_000_000));
    let velocity = r.log().entries().last().unwrap();
    assert_eq!(velocity.kind, LogKind::Truth);
    assert_eq!(velocity.value, (8.0 / 2.0) * 0.001);
    let second_sensor = r.log().entries()[6];
    assert_eq!(second_sensor.value, 4.0 + velocity.value * 0.001);
}

#[test]
fn replay_reinjects_all_command_sources_and_compares_encoded_log() {
    let mut original = multirate(false);
    original.act(&command(-0.0)).unwrap();
    original.run_until(Time::ZERO).unwrap();
    original.act(&command(19.0)).unwrap();
    original.run_until(Time(4_199_820)).unwrap();
    original.act(&command(-17.0)).unwrap();
    original.act(&command(-12.0)).unwrap();
    let end = Time(12_003_300);
    original.run_until(end).unwrap();

    let log = Log::read_binary(bytes(original.log()).as_slice()).unwrap();
    let mut r = multirate(true);
    r.replay(log).unwrap();
    assert!(r.verify_replay().is_err());
    assert!(r.act(&command(30.0)).is_err());
    r.run_until(Time(4_199_820)).unwrap();
    let checkpoint = r.snapshot();
    r.run_until(end).unwrap();
    r.verify_replay().unwrap();
    assert_eq!(bytes(r.log()), bytes(original.log()));
    r.restore(&checkpoint).unwrap();
    r.run_until(end).unwrap();
    r.verify_replay().unwrap();

    let mut wrong = multirate(true);
    let mut different_world = scene();
    different_world.seed += 1;
    let different_world = Runtime::load(different_world).unwrap();
    assert!(wrong.replay(different_world.log().clone()).is_err());
}

#[test]
fn binary_roundtrip_csv_and_corruption_checks() {
    let mut r = Runtime::load(scene()).unwrap();
    let id = r.register_observation("координата,\"X\"", "m").unwrap();
    r.register_sensor(
        "датчик",
        r.physics_rate(),
        Position {
            signal: id,
            bias: 0.0,
        },
    )
    .unwrap();
    r.record_joint_state("истина").unwrap();
    r.act(&command(-0.0)).unwrap();
    r.step().unwrap();
    let encoded = bytes(r.log());
    let decoded = Log::read_binary(encoded.as_slice()).unwrap();
    assert_eq!(bytes(&decoded), encoded);
    assert_eq!(decoded.entries()[0].value.to_bits(), (-0.0f64).to_bits());
    let trailer = encoded.len() - size_of::<u64>();
    assert_eq!(
        fnv1a(encoded[..trailer].iter().copied()),
        decoded.fingerprint()
    );
    let mut csv = Vec::new();
    decoded.write_csv(&mut csv).unwrap();
    let csv = String::from_utf8(csv).unwrap();
    assert!(csv.contains("0,command,before_start,\"сила\",\"N\",true,-0\n"));
    assert!(csv.contains("1000000,observation,,\"координата,\"\"X\"\"\",\"m\",true,0\n"));
    assert!(csv.contains("1000000,truth,,\"qvel[0]\",\"m/s\",true,0\n"));

    let mut corrupted = encoded.clone();
    corrupted[0] ^= 1;
    assert!(Log::read_binary(corrupted.as_slice()).is_err());
    assert!(Log::read_binary(&encoded[..trailer]).is_err());
    let mut wrong_version = encoded.clone();
    wrong_version[8] = 255;
    let hash = fnv1a(wrong_version[..trailer].iter().copied());
    wrong_version[trailer..].copy_from_slice(&hash.to_le_bytes());
    assert!(
        Log::read_binary(wrong_version.as_slice())
            .unwrap_err()
            .to_string()
            .contains("версия")
    );

    // Контрольная сумма здесь правильная. Проверяется отказ при NaN в последней записи.
    let mut nonfinite = encoded;
    let value = trailer - (4 * size_of::<u64>() + 2 * size_of::<u8>()) - size_of::<u64>();
    nonfinite[value..value + size_of::<u64>()].copy_from_slice(&f64::NAN.to_bits().to_le_bytes());
    let hash = fnv1a(nonfinite[..trailer].iter().copied());
    nonfinite[trailer..].copy_from_slice(&hash.to_le_bytes());
    assert!(
        Log::read_binary(nonfinite.as_slice())
            .unwrap_err()
            .to_string()
            .contains("значение")
    );
}

#[test]
fn invalid_input_is_rejected_before_it_changes_the_run() {
    let mut wrong = scene();
    wrong.physics = Rate::hz(60).unwrap();
    assert!(Runtime::load(wrong).is_err());
    let mut wrong = scene();
    wrong.efforts[0].dof = 1;
    assert!(Runtime::load(wrong).is_err());
    let mut wrong = scene();
    wrong.bodies.push(CompiledBody {
        name: "блок".into(),
        owner: "каретка".into(),
        mass_kg: f64::NAN,
        com_m: [0.0; 3],
        inertia_kg_m2: [0.0; 3],
        provenance: Provenance::Estimate("проверка отказа".into()),
    });
    assert!(Runtime::load(wrong).is_err());

    let mut r = Runtime::load(scene()).unwrap();
    let before = bytes(r.log());
    assert!(r.act(&command(f64::NAN)).is_err());
    assert!(r.act(&command(f64::INFINITY)).is_err());
    assert!(
        r.act(&Command {
            targets: vec![(ChannelId(0), 1.0), (ChannelId(1), 2.0)]
        })
        .is_err()
    );
    assert!(
        r.act(&Command {
            targets: vec![(ChannelId(0), 1.0), (ChannelId(0), 2.0)]
        })
        .is_err()
    );
    assert_eq!(bytes(r.log()), before);
    let id = r.register_observation("датчик", "m").unwrap();
    assert!(r.register_observation("датчик", "m").is_err());
    r.register_sensor(
        "модель",
        r.physics_rate(),
        Position {
            signal: id,
            bias: f64::INFINITY,
        },
    )
    .unwrap();
    assert_eq!(r.step().unwrap(), Status::ComponentFailure);
    assert_eq!(r.time(), Time::ZERO);
    assert!(r.log().entries().is_empty());
    assert_eq!(r.steps(), 0);
    assert_eq!(r.step().unwrap(), Status::ComponentFailure);
}

#[test]
fn distant_clock_can_end_outside_the_time_scale() {
    let mut r = Runtime::load(scene()).unwrap();
    let id = r.register_observation("показание", "m").unwrap();
    r.register_sensor(
        "редкий датчик",
        Rate::ratio(1, u64::MAX).unwrap(),
        Position {
            signal: id,
            bias: 0.0,
        },
    )
    .unwrap();
    r.run_until(Time(2_000_000)).unwrap();
    assert_eq!(r.steps(), 2);
    assert_eq!(r.status(), Status::Running);
    assert_eq!(r.log().entries().len(), 1);
    assert_eq!(r.log().entries()[0].at, Time::ZERO);
}

#[test]
fn mujoco_divergence_stops_the_run() {
    const MARKER: &str = "SIM_RUNTIME_TEST_DIVERGENCE_CHILD";
    if std::env::var_os(MARKER).is_none() {
        // MuJoCo может записать свой диагностический файл в рабочую папку.
        let tmp =
            std::env::temp_dir().join(format!("sim-runtime-divergence-{}", std::process::id()));
        std::fs::create_dir(&tmp).unwrap();
        let child = std::process::Command::new(std::env::current_exe().unwrap())
            .args(["--exact", "mujoco_divergence_stops_the_run", "--nocapture"])
            .env(MARKER, "1")
            .current_dir(&tmp)
            .output();
        std::fs::remove_dir_all(&tmp).unwrap();
        let child = child.unwrap();
        assert!(
            child.status.success(),
            "{}\n{}",
            String::from_utf8_lossy(&child.stdout),
            String::from_utf8_lossy(&child.stderr)
        );
        return;
    }

    let world = scene();
    let mut reference = sim_mujoco::Sim::from_xml(&world.mjcf).unwrap();
    reference.qfrc_applied_mut()[0] = f64::MAX;
    reference.step();
    assert!(!reference.numerically_ok());

    let mut r = Runtime::load(world).unwrap();
    r.record_joint_state("истина").unwrap();
    r.act(&command(f64::MAX)).unwrap();
    assert_eq!(
        r.run_until(Time(5_000_000)).unwrap(),
        Status::NumericalFailure
    );
    assert_eq!(r.time(), Time(1_000_000));
    assert_eq!(r.steps(), 0);
    assert!(r.failure().unwrap().contains("MuJoCo"));
    assert!(r.log().entries().iter().all(|e| e.at == Time::ZERO));
    assert!(r.next_event_time().is_none());
    assert_eq!(r.step().unwrap(), Status::NumericalFailure);
}

#[test]
fn replay_preserves_unstarted_world_and_controller_failure() {
    let configure = |playback: bool| {
        let mut r = Runtime::load(scene()).unwrap();
        r.record_joint_state("истина").unwrap();
        r.register_controller(
            "управление",
            r.physics_rate(),
            move |obs: &Observation| {
                assert!(!playback);
                // Первый ход допустим. Неизвестный канал на следующем такте должен остановить прогон.
                if obs.time == Time::ZERO {
                    command(4.0)
                } else {
                    Command {
                        targets: vec![(ChannelId(1), 4.0)],
                    }
                }
            },
        )
        .unwrap();
        r
    };

    let mut src = configure(false);
    src.act(&command(-0.0)).unwrap();
    let mut dst = configure(true);
    dst.replay(Log::read_binary(bytes(src.log()).as_slice()).unwrap())
        .unwrap();
    dst.run_until(Time::ZERO).unwrap();
    dst.verify_replay().unwrap();
    assert_eq!(bytes(src.log()), bytes(dst.log()));
    assert!(!dst.log().end().started);

    assert_eq!(
        src.run_until(Time(8_000_000)).unwrap(),
        Status::ComponentFailure
    );
    assert_eq!(src.time(), Time(1_000_000));
    assert_eq!(src.steps(), 1);
    let mut dst = configure(true);
    dst.replay(Log::read_binary(bytes(src.log()).as_slice()).unwrap())
        .unwrap();
    assert_eq!(dst.run_until(src.time()).unwrap(), Status::ComponentFailure);
    dst.verify_replay().unwrap();
    assert_eq!(bytes(src.log()), bytes(dst.log()));
}

#[test]
fn actuators_sum_to_same_dof_and_unclaimed_input_applies_directly() {
    let mut world = scene();
    world.efforts.push(EffortChannel {
        name: "вторая сила".into(),
        joint: "ход".into(),
        dof: 0,
    });
    let mut r = Runtime::load(world).unwrap();
    let a = r.channel("сила").unwrap();
    let b = r.channel("вторая сила").unwrap();
    let id = r.register_truth("скорость", "m/s").unwrap();
    r.register_evaluator("скорость", r.physics_rate(), Velocity(id))
        .unwrap();
    r.act(&Command {
        targets: vec![(a, 2.0), (b, 6.0)],
    })
    .unwrap();
    r.step().unwrap();
    assert_eq!(
        r.log().entries().last().unwrap().value,
        ((2.0 + 6.0) / 2.0) * 0.001
    );
}

mod platform_contract {
    use std::sync::Arc;

    use sim_runtime::{Forces, Object, ObservationOut, Runtime, Sensor, Status, Truth, TruthId};
    use sim_signal::{ChannelId, Command, ImageFrame, PixelFormat, Rate, Rng, SignalId, Time};
    use sim_world::{
        CompiledBody, CompiledJoint, CompiledJointKind, CompiledWorld, EffortChannel, Provenance,
    };

    const SCENE: &str = r#"<mujoco><option timestep="0.001"/><worldbody>
        <geom type="plane" size="1 1 0.1"/>
        <body name="блок" pos="0 0 0.5"><joint name="ход" type="slide" axis="0 0 1"/>
            <geom type="box" size="0.05 0.05 0.05" mass="1"/></body>
        <body name="шар" pos="0.4 0 0.2"><freejoint/><geom type="sphere" size="0.05" mass="0.2"/></body>
    </worldbody></mujoco>"#;

    fn world() -> CompiledWorld {
        CompiledWorld {
            name: "сцена".into(),
            seed: 5,
            physics: Rate::hz(1000).unwrap(),
            mjcf: SCENE.into(),
            bodies: vec![CompiledBody {
                name: "блок".into(),
                owner: "сцена".into(),
                mass_kg: 1.0,
                com_m: [0.0; 3],
                inertia_kg_m2: [0.001; 3],
                provenance: Provenance::Derived("геометрия сцены".into()),
            }],
            joints: vec![CompiledJoint {
                name: "ход".into(),
                owner: "сцена".into(),
                kind: CompiledJointKind::Slide,
            }],
            efforts: vec![EffortChannel {
                name: "сила".into(),
                joint: "ход".into(),
                dof: 0,
            }],
            drives: Vec::new(),
            sensors: Vec::new(),
            simplifications: Vec::new(),
        }
    }

    /// Датчик с чистой задержкой: пишет положение, доступное через `delay`.
    #[derive(Clone)]
    struct DelayedPosition {
        signal: SignalId,
        delay: Time,
    }

    impl Sensor for DelayedPosition {
        fn sample(
            &mut self,
            truth: &Truth<'_>,
            _: &mut Rng,
            out: &mut ObservationOut,
        ) -> Result<(), String> {
            let value = truth.time().0 as f64;
            out.write_at(
                self.signal,
                value,
                true,
                Time(truth.time().0 + self.delay.0),
            );
            Ok(())
        }
    }

    #[test]
    fn delayed_sample_arrives_exactly_when_declared() {
        let mut rt = Runtime::load(world()).unwrap();
        let signal = rt.register_observation("датчик", "нс").unwrap();
        // 3.5 мс: не кратно ни такту датчика 1 кГц, ни шагу физики.
        let delay = Time(3_500_000);
        rt.register_sensor(
            "датчик",
            Rate::hz(1000).unwrap(),
            DelayedPosition { signal, delay },
        )
        .unwrap();
        rt.run_until(Time(3_499_999)).unwrap();
        assert_eq!(
            rt.observe().latest(signal),
            None,
            "показание пришло раньше задержки"
        );
        rt.run_until(Time(3_500_000)).unwrap();
        assert_eq!(rt.observe().latest(signal), Some(0.0));
        rt.run_until(Time(10_000_000)).unwrap();
        // В 10 мс доступен отсчёт, снятый не позже 6.5 мс.
        assert_eq!(rt.observe().latest(signal), Some(6_000_000.0));
    }

    /// Высота тела из движка должна соответствовать текущему qpos, а не прошлому шагу.
    #[derive(Clone)]
    struct BodyHeight {
        signal: SignalId,
        body: usize,
    }

    impl Sensor for BodyHeight {
        fn sample(
            &mut self,
            truth: &Truth<'_>,
            _: &mut Rng,
            out: &mut ObservationOut,
        ) -> Result<(), String> {
            let (pos, _) = truth.engine().body_pose(self.body);
            out.write(self.signal, pos[2] - 0.5 - truth.qpos()[0], true);
            Ok(())
        }
    }

    #[test]
    fn sensors_see_current_kinematics_and_do_not_change_trajectory() {
        let hash = |with_sensor: bool| {
            let mut rt = Runtime::load(world()).unwrap();
            let signal = rt.register_observation("высота", "м").unwrap();
            if with_sensor {
                rt.register_sensor(
                    "высота",
                    Rate::hz(1000).unwrap(),
                    BodyHeight { signal, body: 1 },
                )
                .unwrap();
            }
            let force = rt.channel("сила").unwrap();
            rt.act(&Command {
                targets: vec![(force, 3.0)],
            })
            .unwrap();
            assert_eq!(rt.run_until(Time(400_000_000)).unwrap(), Status::Running);
            if with_sensor {
                let error = rt.observe().latest(signal).unwrap();
                assert!(error.abs() < 1e-12, "датчик видит прошлый шаг: {error}");
            }
            rt.log().end().state_hash
        };
        assert_eq!(hash(true), hash(false), "датчик изменил траекторию");
    }

    /// Модель мира публикует внутреннее состояние, датчик его читает.
    #[derive(Clone)]
    struct Heater {
        state: TruthId,
        channel: ChannelId,
        value: f64,
    }

    impl Object for Heater {
        fn update(
            &mut self,
            _: &Truth<'_>,
            command: &Command,
            _: &mut Rng,
            out: &mut Forces,
        ) -> Result<(), String> {
            let cmd = command
                .targets
                .iter()
                .find(|(c, _)| *c == self.channel)
                .map_or(0.0, |x| x.1);
            self.value += cmd;
            out.publish(self.state, self.value);
            out.set(self.channel, 0.0);
            Ok(())
        }
    }

    #[derive(Clone)]
    struct PlantReader {
        state: TruthId,
        signal: SignalId,
    }

    impl Sensor for PlantReader {
        fn sample(
            &mut self,
            truth: &Truth<'_>,
            _: &mut Rng,
            out: &mut ObservationOut,
        ) -> Result<(), String> {
            if let Some(v) = truth.plant(self.state) {
                out.write(self.signal, v, true);
            }
            Ok(())
        }
    }

    #[test]
    fn plant_state_reaches_sensors_and_log() {
        let mut rt = Runtime::load(world()).unwrap();
        let state = rt.register_truth("нагрев", "К").unwrap();
        let signal = rt.register_observation("термометр", "К").unwrap();
        let channel = rt.channel("сила").unwrap();
        let rate = Rate::hz(1000).unwrap();
        rt.register_object(
            "нагреватель",
            rate,
            &[channel],
            Heater {
                state,
                channel,
                value: 0.0,
            },
        )
        .unwrap();
        rt.register_sensor("термометр", rate, PlantReader { state, signal })
            .unwrap();
        rt.act(&Command {
            targets: vec![(channel, 2.0)],
        })
        .unwrap();
        rt.run_until(Time(5_000_000)).unwrap();
        // Такты 0–5 мс: модель работает в фазе после датчика, поэтому датчик отстаёт на такт.
        assert_eq!(rt.observe().latest(signal), Some(10.0));
        assert!(rt.log().entries().iter().any(|e| e.value == 12.0));
    }

    #[derive(Clone)]
    struct Camera {
        signal: SignalId,
    }

    impl Sensor for Camera {
        fn sample(
            &mut self,
            truth: &Truth<'_>,
            _: &mut Rng,
            out: &mut ObservationOut,
        ) -> Result<(), String> {
            let shade = (truth.time().0 / 1_000_000) as u8;
            out.write_frame(ImageFrame {
                signal: self.signal,
                captured: truth.time(),
                at: Time(truth.time().0 + 2_000_000),
                width: 2,
                height: 1,
                format: PixelFormat::Rgb8,
                data: Arc::from(vec![shade; 6]),
                valid: true,
            });
            Ok(())
        }
    }

    #[test]
    fn frames_arrive_late_are_logged_and_snapshots_keep_them() {
        let mut rt = Runtime::load(world()).unwrap();
        let signal = rt.register_observation("камера", "кадр").unwrap();
        rt.register_sensor("камера", Rate::hz(100).unwrap(), Camera { signal })
            .unwrap();
        rt.run_until(Time(1_999_999)).unwrap();
        assert!(rt.observe().frame(signal).is_none());
        rt.run_until(Time(2_000_000)).unwrap();
        assert_eq!(rt.observe().frame(signal).unwrap().captured, Time(0));
        let snapshot = rt.snapshot();
        rt.run_until(Time(50_000_000)).unwrap();
        let end = rt.log().end().state_hash;
        let frames = rt.log().entries().len();
        rt.restore(&snapshot).unwrap();
        rt.run_until(Time(50_000_000)).unwrap();
        assert_eq!(rt.log().end().state_hash, end);
        assert_eq!(rt.log().entries().len(), frames);
    }

    #[test]
    fn inconsistent_frame_is_a_failure() {
        #[derive(Clone)]
        struct Broken(SignalId);
        impl Sensor for Broken {
            fn sample(
                &mut self,
                truth: &Truth<'_>,
                _: &mut Rng,
                out: &mut ObservationOut,
            ) -> Result<(), String> {
                out.write_frame(ImageFrame {
                    signal: self.0,
                    captured: truth.time(),
                    at: truth.time(),
                    width: 10,
                    height: 10,
                    format: PixelFormat::Rgb8,
                    data: Arc::from(vec![0; 3]),
                    valid: true,
                });
                Ok(())
            }
        }
        let mut rt = Runtime::load(world()).unwrap();
        let signal = rt.register_observation("камера", "кадр").unwrap();
        rt.register_sensor("камера", Rate::hz(100).unwrap(), Broken(signal))
            .unwrap();
        assert_eq!(
            rt.run_until(Time(1_000_000)).unwrap(),
            Status::ComponentFailure
        );
    }
}
