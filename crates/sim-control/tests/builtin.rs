use std::sync::Arc;

use sim_control::builtin::*;
use sim_control::{
    ControlContract, ControlError, Controller, InputChannel, InputKind, OutputChannel,
};
use sim_signal::{
    ChannelId, Command, ImageFrame, Observation, PixelFormat, Rng, Sample, SignalId, Time,
};

const NS: u64 = 1_000_000_000; // Наносекунд в секунде.
const OUTPUT: ChannelId = ChannelId(5);

fn scalar(id: u32, name: &str, max_age_ns: u64) -> InputChannel {
    InputChannel {
        id: SignalId(id),
        name: name.into(),
        unit: "1".into(),
        kind: InputKind::Scalar { max_age_ns },
    }
}

fn image() -> InputChannel {
    InputChannel {
        id: SignalId(0),
        name: "кадр".into(),
        unit: "rgb".into(),
        kind: InputKind::Image {
            format: PixelFormat::Rgb8,
            max_capture_age_ns: 10,
            max_delivery_age_ns: 2,
        },
    }
}

fn contract(inputs: Vec<InputChannel>) -> ControlContract {
    ControlContract::new(
        inputs,
        vec![OutputChannel {
            id: OUTPUT,
            name: "выход".into(),
            unit: "1".into(),
            max_age_ns: 3,
        }],
    )
    .unwrap()
}

fn controller(
    contract: ControlContract,
    blocks: Vec<Block>,
    missing: MissingOutput,
) -> BuiltinController {
    let node = NodeId(blocks.len() - 1);
    BuiltinController::new(
        contract,
        BuiltinConfig {
            blocks,
            outputs: vec![OutputBinding {
                channel: OUTPUT,
                node,
                missing,
            }],
        },
    )
    .unwrap()
}

fn sample(signal: u32, value: f64, time: u64, valid: bool) -> Sample {
    Sample {
        signal: SignalId(signal),
        value,
        at: Time(time),
        valid,
    }
}

fn observed(time: u64, samples: Vec<Sample>) -> Observation {
    Observation {
        time: Time(time),
        samples,
        frames: Vec::new(),
    }
}

fn close(value: f64, expected: f64) {
    assert!((value - expected).abs() < 1e-12, "{value} != {expected}");
}

#[test]
fn pid_saturation_and_anti_windup_match_manual_recurrence() {
    let graph = vec![
        Block::Scalar(SignalId(0)),
        Block::Pid {
            error: NodeId(0),
            enable: None,
            config: PidConfig {
                kp: 2.,
                ki: 1.,
                kd: 0.5,
                integral_limits: [-2., 2.],
                output_limits: [-8., 8.],
            },
        },
    ];
    let mut controller = controller(
        contract(vec![scalar(0, "ошибка", 0)]),
        graph,
        MissingOutput::Error,
    );
    let mut rng = Rng::stream(12, "регулятор");
    // При 0,5 с: 4+1+1=6. При 1 с насыщение удерживает интеграл 1.
    // При 1,5 с: 2+1,5−9=−5,5. При 2,5 с интеграл ограничен значением 2.
    for (time, error, expected) in [
        (0, 1., 2.),
        (NS / 2, 2., 6.),
        (NS, 10., 8.),
        (NS + NS / 2, 1., -5.5),
        (2 * NS, 1., 4.),
        (2 * NS + NS / 2, 1., 4.),
    ] {
        let got = controller
            .step(
                &observed(time, vec![sample(0, error, time, true)]),
                &mut rng,
            )
            .unwrap();
        assert_eq!(got.targets, [(OUTPUT, expected)]);
    }
}

#[test]
fn upper_goal_stop_and_feedback_use_freshest_values_and_reset_after_loss() {
    let graph = vec![
        Block::Scalar(SignalId(0)),
        Block::Scalar(SignalId(1)),
        Block::Scalar(SignalId(2)),
        Block::Affine {
            terms: vec![(NodeId(1), 1.), (NodeId(0), -1.)],
            offset: 0.,
        },
        Block::InRange {
            input: NodeId(2),
            bounds: [1., 1.],
        },
        Block::Pid {
            error: NodeId(3),
            enable: Some(NodeId(4)),
            config: PidConfig {
                kp: 1.,
                ki: 1.,
                kd: 0.,
                integral_limits: [-4., 4.],
                output_limits: [-10., 10.],
            },
        },
    ];
    let c = contract(vec![
        scalar(0, "измерение", 0),
        scalar(1, "цель", NS),
        scalar(2, "разрешение", 0),
    ]);
    let mut controller = controller(c, graph, MissingOutput::Value(0.));
    let mut rng = Rng::stream(12, "контур");
    let step = |controller: &mut BuiltinController, rng: &mut Rng, t, goal: &[Sample], enable| {
        let mut data = vec![sample(0, 1., t, true), sample(2, enable, t, true)];
        data.extend(goal.iter().copied());
        controller.step(&observed(t, data), rng).unwrap().targets[0].1
    };
    let upper = sample(1, 3., 0, true);
    assert_eq!(step(&mut controller, &mut rng, 0, &[upper], 1.), 2.);
    assert_eq!(step(&mut controller, &mut rng, NS / 2, &[upper], 1.), 3.);
    // Новый недействительный отсчёт стоит перед старым в массиве.
    let missing = sample(1, 20., NS, false);
    assert_eq!(
        step(&mut controller, &mut rng, NS, &[missing, upper], 1.),
        0.
    );
    let next = sample(1, 3., 2 * NS, true);
    assert_eq!(step(&mut controller, &mut rng, 2 * NS, &[next], 1.), 2.);
    assert_eq!(step(&mut controller, &mut rng, 3 * NS, &[next], 0.), 0.);
    assert_eq!(step(&mut controller, &mut rng, 3 * NS + 1, &[next], 1.), 0.);
}

#[test]
fn invalid_observation_is_an_error_even_if_its_value_would_be_discarded() {
    let c = contract(vec![scalar(0, "положение", 0)]);
    let mut controller = controller(
        c.clone(),
        vec![Block::Scalar(SignalId(0))],
        MissingOutput::Omit,
    );
    let mut rng = Rng::stream(0, "неверный вход");
    for samples in [
        vec![sample(0, 1., 1, true), sample(0, f64::NAN, 0, false)],
        vec![sample(0, f64::INFINITY, 0, false)],
        vec![sample(0, 1., 2, true)],
        vec![sample(1, 1., 0, false)],
        vec![sample(0, 1., 0, true), sample(0, 1., 0, false)],
    ] {
        assert!(matches!(
            controller.step(&observed(1, samples), &mut rng),
            Err(ControlError::InvalidObservation(_))
        ));
    }
    assert_eq!(
        controller
            .step(&observed(1, vec![sample(0, 1., 1, false)]), &mut rng)
            .unwrap(),
        Command::default()
    );
    assert!(controller.step(&observed(1, Vec::new()), &mut rng).is_err());
    assert!(ControlContract::new(vec![scalar(0, "а", 0), scalar(0, "б", 0)], Vec::new()).is_err());
    assert!(ControlContract::new(vec![scalar(0, "а", 0), scalar(1, "а", 0)], Vec::new()).is_err());
}

#[test]
fn configuration_and_numeric_overflow_fail_closed() {
    let c = contract(vec![scalar(0, "положение", 3)]);
    let build = |blocks, missing| {
        BuiltinController::new(
            c.clone(),
            BuiltinConfig {
                blocks,
                outputs: vec![OutputBinding {
                    channel: OUTPUT,
                    node: NodeId(0),
                    missing,
                }],
            },
        )
    };
    assert!(build(vec![Block::Scalar(SignalId(42))], MissingOutput::Omit).is_err());
    assert!(
        build(
            vec![Block::Affine {
                terms: vec![(NodeId(0), 1.)],
                offset: 0.
            }],
            MissingOutput::Omit
        )
        .is_err()
    );
    assert!(build(vec![Block::Constant(f64::NAN)], MissingOutput::Omit).is_err());
    assert!(
        build(
            vec![Block::Constant(1.)],
            MissingOutput::Value(f64::INFINITY)
        )
        .is_err()
    );
    assert!(
        BuiltinController::new(
            c.clone(),
            BuiltinConfig {
                blocks: vec![],
                outputs: vec![]
            }
        )
        .is_err()
    );
    let mut controller = controller(
        c,
        vec![
            Block::Scalar(SignalId(0)),
            Block::Affine {
                terms: vec![(NodeId(0), 2.)],
                offset: 0.,
            },
        ],
        MissingOutput::Error,
    );
    let mut rng = Rng::stream(0, "арифметика");
    assert!(matches!(
        controller.step(&observed(0, vec![sample(0, f64::MAX, 0, true)]), &mut rng),
        Err(ControlError::Execution(_))
    ));
    assert_eq!(
        controller
            .step(&observed(0, vec![sample(0, 2., 0, true)]), &mut rng)
            .unwrap()
            .targets,
        [(OUTPUT, 4.)]
    );
    assert!(matches!(
        controller.step(&observed(1, vec![]), &mut rng),
        Err(ControlError::Execution(_))
    ));
}

#[test]
fn imu_angle_matches_filter_recurrence_rejects_acceleration_and_reinitializes() {
    let c = contract(vec![
        scalar(0, "синус", 0),
        scalar(1, "косинус", 0),
        scalar(2, "угловая скорость", 0),
    ]);
    let mut controller = controller(
        c,
        vec![
            Block::Scalar(SignalId(0)),
            Block::Scalar(SignalId(1)),
            Block::Scalar(SignalId(2)),
            Block::PlanarTilt {
                sin: NodeId(0),
                cos: NodeId(1),
                rate: NodeId(2),
                config: TiltConfig {
                    acceleration_range: [0.8, 1.2],
                    time_constant_s: 1.,
                },
            },
        ],
        MissingOutput::Omit,
    );
    let mut rng = Rng::stream(0, "наклон");
    let mut step = |time, sin, cos, rate| {
        controller
            .step(
                &observed(
                    time,
                    vec![
                        sample(0, sin, time, true),
                        sample(1, cos, time, true),
                        sample(2, rate, time, true),
                    ],
                ),
                &mut rng,
            )
            .unwrap()
    };
    close(step(0, 0., 1., 1.).targets[0].1, 0.);
    // Прогноз 1 рад за секунду, измерение 0, остаток exp(−1).
    close(step(NS, 0., 1., 1.).targets[0].1, (-1_f64).exp());
    assert!(step(2 * NS, 0., 2., 1.).targets.is_empty());
    let near_pi: f64 = 3.1;
    close(
        step(3 * NS, near_pi.sin(), near_pi.cos(), 0.).targets[0].1,
        near_pi,
    );
    let across = -near_pi;
    let expected = near_pi + (1. - (-1_f64).exp()) * (std::f64::consts::TAU - 2. * near_pi)
        - std::f64::consts::TAU;
    close(
        step(4 * NS, across.sin(), across.cos(), 0.).targets[0].1,
        expected,
    );
}

fn frame(captured: u64, at: u64, count: usize) -> ImageFrame {
    let mut data = [0; 24]; // RGB8: 4 × 2 × 3 байта.
    for index in [1, 3, 5].into_iter().take(count) {
        data[index * 3] = 255;
    }
    ImageFrame {
        signal: SignalId(0),
        captured: Time(captured),
        at: Time(at),
        width: 4,
        height: 2,
        format: PixelFormat::Rgb8,
        data: Arc::from(data),
        valid: true,
    }
}

#[test]
fn rgb_centroid_gate_and_two_independent_frame_ages() {
    let target = RgbTarget {
        min: [255, 0, 0],
        max: [255, 0, 0],
        min_pixels: 3,
        axis: ImageAxis::X,
        projection: ImageProjection::Bearing {
            center_px: 1.,
            focal_px: 2.,
        },
    };
    let mut controller = controller(
        contract(vec![image()]),
        vec![
            Block::Rgb {
                image: SignalId(0),
                target,
            },
            Block::Constant(1.),
            Block::Gate {
                input: NodeId(0),
                condition: NodeId(1),
            },
        ],
        MissingOutput::Omit,
    );
    let mut rng = Rng::stream(0, "цвет");
    let mut step = |time, data| {
        controller.step(
            &Observation {
                time: Time(time),
                samples: Vec::new(),
                frames: data,
            },
            &mut rng,
        )
    };
    let good = step(10, vec![frame(0, 8, 3)]).unwrap();
    // Координаты x совпавших пикселей 1, 3, 1. Угол atan((5/3−1)/2).
    close(good.targets[0].1, (1_f64 / 3.).atan());
    assert!(step(11, vec![frame(0, 11, 3)]).unwrap().targets.is_empty()); // Старое время съёмки.
    assert!(step(12, vec![frame(8, 9, 3)]).unwrap().targets.is_empty()); // Старая доставка.
    assert!(step(13, vec![frame(11, 13, 2)]).unwrap().targets.is_empty()); // Мало пикселей.
    let mut down = frame(13, 14, 3);
    down.valid = false;
    assert!(
        step(14, vec![down, frame(12, 13, 3)])
            .unwrap()
            .targets
            .is_empty()
    );
    let mut wrong = frame(15, 15, 3);
    wrong.width = 5;
    assert!(step(15, vec![wrong]).is_err());
    assert!(step(15, vec![frame(16, 15, 3)]).is_err());
    assert!(step(15, vec![frame(15, 16, 3)]).is_err());

    let mut stopped = super_gate();
    assert!(
        stopped
            .step(&observed(0, vec![]), &mut rng)
            .unwrap()
            .targets
            .is_empty()
    );
}

fn super_gate() -> BuiltinController {
    controller(
        contract(Vec::new()),
        vec![
            Block::Constant(9.),
            Block::Constant(0.),
            Block::Gate {
                input: NodeId(0),
                condition: NodeId(1),
            },
        ],
        MissingOutput::Omit,
    )
}

#[test]
fn accepting_side_rejects_unrecognized_duplicate_nonfinite_and_expired_output() {
    let contract = contract(Vec::new());
    let command = |id, value| Command {
        targets: vec![(id, value)],
    };
    let good = command(OUTPUT, 1.);
    assert!(contract.validate_command(&good, Time(10), Time(13)).is_ok());
    assert!(
        contract
            .validate_command(&good, Time(10), Time(14))
            .is_err()
    );
    assert!(contract.validate_command(&good, Time(10), Time(9)).is_err());
    assert!(
        contract
            .validate_command(&command(ChannelId(99), 1.), Time(10), Time(10))
            .is_err()
    );
    assert!(
        contract
            .validate_command(&command(OUTPUT, f64::NAN), Time(10), Time(10))
            .is_err()
    );
    assert!(
        contract
            .validate_command(
                &Command {
                    targets: vec![(OUTPUT, 1.), (OUTPUT, 1.)]
                },
                Time(10),
                Time(10)
            )
            .is_err()
    );
}

#[test]
fn two_rate_closed_cycle_matches_analytic_carriage_and_stops_on_signal_loss() {
    // Идеальная каретка для сверки блоков. При dt=1/4 с и Kp=2 ошибка за такт делится на два.
    let dt = NS / 4;
    let mut high = controller(
        contract(vec![scalar(0, "задание", 0)]),
        vec![Block::Scalar(SignalId(0))],
        MissingOutput::Omit,
    );
    let mut low = controller(
        contract(vec![scalar(0, "положение", 0), scalar(1, "цель", 3 * dt)]),
        vec![
            Block::Scalar(SignalId(0)),
            Block::Scalar(SignalId(1)),
            Block::Affine {
                terms: vec![(NodeId(1), 1.), (NodeId(0), -1.)],
                offset: 0.,
            },
            Block::Pid {
                error: NodeId(2),
                enable: None,
                config: PidConfig {
                    kp: 2.,
                    ki: 0.,
                    kd: 0.,
                    integral_limits: [0., 0.],
                    output_limits: [-2., 2.],
                },
            },
        ],
        MissingOutput::Value(0.),
    );
    let mut high_rng = Rng::stream(4, "верх");
    let mut low_rng = Rng::stream(4, "низ");
    let mut position = 0.;
    let mut goal = Vec::new();
    for tick in 0..=8_u64 {
        let time = tick * dt;
        if tick.is_multiple_of(4) {
            let valid = tick != 4;
            let command = high
                .step(
                    &observed(time, vec![sample(0, 1., time, valid)]),
                    &mut high_rng,
                )
                .unwrap();
            assert_eq!(!command.targets.is_empty(), valid);
            goal.push(match command.targets.first() {
                Some(&(_, value)) => sample(1, value, time, true),
                None => sample(1, 0., time, false),
            });
        }
        let inputs = [vec![sample(0, position, time, true)], goal.clone()].concat();
        let velocity = low
            .step(&observed(time, inputs), &mut low_rng)
            .unwrap()
            .targets[0]
            .1;
        position += velocity * Time(dt).as_secs();
        let active_steps = (tick + 1).min(4) + u64::from(tick == 8);
        let expected = 1. - 0.5_f64.powi(active_steps as i32);
        assert_eq!(position, expected);
        if (4..8).contains(&tick) {
            assert_eq!(velocity, 0.);
        }
    }
    assert_eq!(position, 31. / 32.);
}
