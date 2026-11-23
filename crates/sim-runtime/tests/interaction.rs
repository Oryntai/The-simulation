use sim_process::{
    Analysis, SourcedRange,
    attachments::{
        CompliantPad, ConfirmedEvent, ConfirmedInterfaceEvent, CurvePoint, ForceCurve,
        FreeLengthLaw, PadCalibration, PadLaw, PadReactionOwner, PadSpec, TensionCalibration,
        TensionRoute, TensionSpec,
    },
};
use sim_runtime::{
    Forces, Log, LogKind, Object, ObservationOut, Runtime, Sensor, Status, Truth, TruthId,
    control::{self, ControlAssets, ControlConfig, EventKind, ModelPort, RunSpec},
    process::interaction::*,
};
use sim_signal::{ChannelId, Command, Rate, Rng, SignalId, Time};
use sim_world::{CompiledJoint, CompiledJointKind, CompiledWorld, EffortChannel, Provenance};

const MS: u64 = 1_000_000;
const DT: f64 = 0.001;
const MASS: f64 = 2.0;

fn source() -> Provenance {
    Provenance::Measured("аналитический стенд, заданные параметры".into())
}
fn band(min: f64, max: f64) -> SourcedRange {
    SourcedRange {
        min,
        nominal: min,
        max,
        provenance: source(),
    }
}
fn n(x: f64) -> SourcedRange {
    band(x, x)
}
fn trajectory(knots: &[(u64, f64)]) -> Trajectory {
    Trajectory {
        source: source(),
        knots: knots.iter().map(|&(ms, q)| (Time(ms * MS), q)).collect(),
    }
}

fn world(collides: bool) -> CompiledWorld {
    // Каретка 2 кг без трения и тяжести, интегратор Euler с шагом 1 мс.
    CompiledWorld {
        name: "каретка".into(),
        seed: 13,
        physics: Rate::hz(1000).unwrap(),
        mjcf: format!(
            r#"<mujoco><option timestep="{DT}" gravity="0 0 0" integrator="Euler"/>
          <worldbody>
            <body name="основание"><geom name="а" type="sphere" size=".02" contype="{}" conaffinity="{}"/></body>
            <body name="блок"><joint name="ось" type="slide" axis="0 0 1"/>
               <geom name="б" type="sphere" size=".02" mass="{MASS}" contype="1" conaffinity="{}"/>
            </body>
          </worldbody></mujoco>"#,
            usize::from(collides),
            usize::from(collides),
            usize::from(collides)
        ),
        bodies: vec![],
        joints: vec![CompiledJoint {
            name: "ось".into(),
            owner: "блок".into(),
            kind: CompiledJointKind::Slide,
        }],
        efforts: ["намерение", "сила", "питание"]
            .map(|name| EffortChannel {
                name: name.into(),
                joint: "ось".into(),
                dof: 0,
            })
            .into(),
        drives: vec![],
        sensors: vec![],
        simplifications: vec![],
    }
}

fn pad(owner: PadReactionOwner) -> PadBinding {
    // N=1000*(0,04−q)+. Площадь 20–40 см² задана независимо от геометрии решателя.
    let spec = PadSpec {
        name: "пятно".into(),
        bodies: ["основание".into(), "блок".into()],
        geometries: ["а".into(), "б".into()],
        owner,
        area_m2: Some(band(0.002, 0.004)),
        calibration: PadCalibration {
            compression_m: band(0.0, 0.04),
            offset_m: band(0.0, 0.02),
            axis_error_rad: band(0.0, 0.05),
            relative_speed_m_s: band(0.0, 5.0),
        },
    };
    let mounts = spec.bodies.clone().map(|body| LocalMount {
        point: LocalPoint {
            body,
            position_m: [0.0; 3],
        },
        normal: [0.0, 0.0, 1.0],
        tangent: [1.0, 0.0, 0.0],
    });
    PadBinding {
        model: CompliantPad::new(spec).unwrap(),
        mounts,
        events: vec![],
    }
}

fn model_pad() -> PadBinding {
    pad(PadReactionOwner::Model(Box::new(PadLaw {
        free_gap_m: n(0.04),
        compression: ForceCurve::new(
            [0.0, 0.04]
                .map(|x| CurvePoint {
                    displacement_m: n(x),
                    force_n: n(1000.0 * x),
                })
                .into(),
        )
        .unwrap(),
        damping_ns_m: n(0.0),
        shear_stiffness_n_m: n(100.0),
        friction: n(0.5),
        capacity_n: n(100.0),
    })))
}

fn profile(initial: f64, pad: Option<PadBinding>, feedback: Option<bool>) -> ResolvedInteraction {
    let regions = pad
        .as_ref()
        .map(|_| [("участок".into(), vec!["б".into()])].into())
        .unwrap_or_default();
    ResolvedInteraction {
        name: "связь".into(),
        resources: 8,
        variant: 0,
        regions,
        pads: pad.into_iter().collect(),
        routes: vec![],
        motion: MotionProfile::Dynamic {
            axes: vec![MotionAxis {
                channel: ChannelId(0),
                initial_position: initial,
                initial_velocity: 0.0,
                reference: trajectory(&[(0, initial)]),
                stiffness: n(500.0),
                damping: n(2.0),
                effort_limit: n(4.0),
            }],
            maneuver: None,
            response: feedback.map(|enabled| {
                Box::new(Response {
                    enabled,
                    initiator: "ответ".into(),
                    patch: "пятно".into(),
                    threshold_n: n(1.0),
                    latency_ns: 2 * MS,
                    latency_source: source(),
                    offsets: [(ChannelId(0), trajectory(&[(0, 0.0), (4, 0.02)]))].into(),
                })
            }),
        },
    }
}

fn runtime(world: CompiledWorld, profile: ResolvedInteraction) -> Runtime {
    let mut rt = Runtime::load(world).unwrap();
    register_interaction(&mut rt, profile).unwrap();
    rt
}

fn run(rt: &mut Runtime, stop_ms: u64) {
    assert_eq!(
        rt.run_until(Time(stop_ms * MS)).unwrap(),
        Status::Running,
        "{:?}",
        rt.failure()
    );
}

fn near(actual: f64, expected: f64) {
    // Сложение короткой сетки и решение диагональной массы с округлением f64.
    assert!((actual - expected).abs() < 1e-12, "{actual} != {expected}");
}

fn reactions(rt: &Runtime) -> Vec<MotionRecord> {
    rt.process_journal()
        .iter()
        .flat_map(|tick| {
            tick.outputs["связь"]
                .interaction
                .as_ref()
                .unwrap()
                .motion_events
                .iter()
                .cloned()
        })
        .collect()
}

#[test]
fn free_motion_and_scheduled_maneuver_integrate_with_effort_cap() {
    let mut setup = profile(0.0, None, None);
    let MotionProfile::Dynamic { axes, maneuver, .. } = &mut setup.motion else {
        panic!()
    };
    axes[0].reference = trajectory(&[(0, 0.02)]);
    axes[0].damping = n(0.0);
    *maneuver = Some(Maneuver {
        at: Time(5 * MS),
        initiator: "сценарий".into(),
        offsets: [(ChannelId(0), trajectory(&[(0, 0.0), (4, -0.04)]))].into(),
    });
    let mut rt = runtime(world(false), setup);
    run(&mut rt, 4);
    for (k, t) in rt.process_journal().iter().enumerate() {
        let k = k as f64;
        near(t.qvel[0], 4.0 * k * DT / MASS);
        near(t.qpos[0], 4.0 * k * (k + 1.0) * DT.powi(2) / (2.0 * MASS));
        near(t.outputs["связь"].efforts[0].1, 4.0);
    }
    run(&mut rt, 12);
    let events = reactions(&rt);
    assert_eq!(events.len(), 1);
    assert_eq!(events[0].initiator, "сценарий");
    assert_eq!(events[0].contact_at, None);
    assert_eq!(events[0].due, Time(5 * MS));
    assert_eq!(events[0].interval.end, Time(6 * MS));
    near(rt.process_journal()[10].outputs["связь"].efforts[0].1, -4.0);
}

#[test]
fn completed_contact_triggers_once_and_impulse_matches_physics() {
    let setup = profile(0.03, Some(model_pad()), Some(true));
    let mut rt = runtime(world(false), setup.clone());
    let mut control = runtime(world(false), profile(0.03, Some(model_pad()), Some(false)));
    let mut miss = runtime(world(false), profile(0.05, Some(model_pad()), Some(true)));
    for r in [&mut rt, &mut control, &mut miss] {
        run(r, 12);
    }
    let events = reactions(&rt);
    assert_eq!(events.len(), 1);
    let first = &events[0];
    assert_eq!(first.contact_at, Some(Time(MS)));
    assert_eq!(first.due, Time(3 * MS));
    assert_eq!(first.interval.start, first.due);
    assert!(first.sources.len() >= 3);
    assert!(reactions(&control).is_empty() && reactions(&miss).is_empty());
    assert_eq!(miss.process_journal().last().unwrap().qvel, [0.0]);
    let final_q = |rt: &Runtime| rt.process_journal().last().unwrap().qpos[0];
    assert!(final_q(&rt) > final_q(&control));
    for pair in rt.process_journal().windows(2) {
        let [a, b] = pair else { unreachable!() };
        let expected = 1000.0 * (0.04 - a.qpos[0]).max(0.0);
        let out = &b.outputs["связь"];
        let patch = &out.interaction.as_ref().unwrap().patches["пятно"];
        let Analysis::Analyzed { value, .. } = &patch.reaction else {
            panic!()
        };
        near(value.normal_n, expected);
        assert_eq!(patch.interval, b.completed.as_ref().unwrap().interval);
        near(value.loads[0].force_n[2], -expected);
        near(value.loads[1].force_n[2], expected);
        near(
            MASS * (b.qvel[0] - a.qvel[0]) / DT,
            expected + a.outputs["связь"].efforts[0].1,
        );
    }
}

#[test]
fn estimate_runs_with_explicit_blocker_and_fixed_pose_is_diagnostic() {
    let mut guess = profile(0.03, Some(model_pad()), Some(true));
    let MotionProfile::Dynamic {
        response: Some(response),
        ..
    } = &mut guess.motion
    else {
        panic!()
    };
    response.latency_source = Provenance::Estimate("пробный профиль".into());
    let mut rt = runtime(world(false), guess);
    run(&mut rt, 5);
    assert_eq!(reactions(&rt).len(), 1);
    assert!(
        rt.process_journal()[3].outputs["связь"]
            .blockers
            .iter()
            .any(|s| s.contains("слабое обоснование"))
    );

    let mut fixed = profile(0.0, None, None);
    fixed.motion = MotionProfile::FixedPose {
        joint: "ось".into(),
        position: 0.03,
    };
    let mut rt = runtime(world(false), fixed);
    // Та же проверочная сила, что в свободном опыте.
    rt.act(&Command {
        targets: vec![(ChannelId(1), 4.0)],
    })
    .unwrap();
    run(&mut rt, 5);
    assert_eq!(rt.process_journal().last().unwrap().qpos, [0.03]);
    assert_eq!(rt.process_journal().last().unwrap().qvel, [0.0]);
    assert!(
        rt.process_journal()[1].outputs["связь"]
            .blockers
            .iter()
            .any(|s| s.contains("идеальное удержание"))
    );
}

#[test]
fn only_one_contact_owner_and_engine_effect_is_not_applied_twice() {
    let mut rt = Runtime::load(world(true)).unwrap();
    assert!(
        register_interaction(&mut rt, profile(0.03, Some(model_pad()), None))
            .unwrap_err()
            .contains("одновременно")
    );
    let mut masked = Runtime::load(world(false)).unwrap();
    assert!(
        register_interaction(
            &mut masked,
            profile(0.03, Some(pad(PadReactionOwner::Engine)), None)
        )
        .unwrap_err()
        .contains("исключена")
    );
    register_interaction(
        &mut rt,
        profile(0.03, Some(pad(PadReactionOwner::Engine)), Some(true)),
    )
    .unwrap();
    run(&mut rt, 5);
    let steps = rt.process_journal();
    let out = &steps[1].outputs["связь"];
    assert!(out.body_loads.is_empty());
    let Analysis::Analyzed { value, .. } =
        &out.interaction.as_ref().unwrap().patches["пятно"].reaction
    else {
        panic!()
    };
    assert!(value.normal_n > 1.0);
    near(MASS * steps[1].qvel[0] / DT, value.normal_n);
    assert_eq!(reactions(&rt)[0].contact_at, Some(Time(MS)));
}

fn binary(rt: &Runtime) -> Vec<u8> {
    let mut result = Vec::new();
    rt.log().write_binary(&mut result).unwrap();
    result
}

#[test]
fn snapshots_fork_and_binary_replay_match_through_reaction() {
    let setup = profile(0.03, Some(model_pad()), Some(true));
    let mut rt = runtime(world(false), setup.clone());
    run(&mut rt, 0);
    let before = rt.snapshot();
    run(&mut rt, 5);
    let during = rt.snapshot();
    let mut branch = rt.fork().unwrap();
    run(&mut branch, 12);
    assert_eq!(rt.time(), Time(5 * MS));
    run(&mut rt, 12);
    let original = binary(&rt);
    assert_eq!(binary(&branch), original);
    let mut audited = runtime(world(false), setup.clone());
    audited.audit_snapshot(Time(5 * MS), Time(12 * MS)).unwrap();
    assert_eq!(binary(&audited), original);
    for checkpoint in [before, during] {
        rt.restore(&checkpoint).unwrap();
        run(&mut rt, 12);
        assert_eq!(binary(&rt), original);
    }
    let mut replay = runtime(world(false), setup);
    replay
        .replay(Log::read_binary(original.as_slice()).unwrap())
        .unwrap();
    run(&mut replay, 12);
    replay.verify_replay().unwrap();
    assert_eq!(binary(&replay), original);
    assert!(replay.interaction_is_replay());
}

#[test]
fn affine_route_loads_match_virtual_work_and_release_preserves_past_reaction() {
    let mut setup = profile(0.03, None, None);
    setup.regions.insert("участок".into(), vec!["б".into()]);
    // При q=30 мм геометрическая длина q, свободная 35−0,5q мм.
    // При k=1000 Н/м получается 10 Н, а −dU/dq=−1,5T.
    let points = ["основание", "блок"].map(|body| LocalPoint {
        body: body.into(),
        position_m: [0.0; 3],
    });
    let route = TensionRoute::new(TensionSpec {
        name: "трасса".into(),
        bodies: points.each_ref().map(|p| p.body.clone()).into(),
        length: FreeLengthLaw::Affine {
            channel: ChannelId(0),
            at_zero_m: n(0.035),
            slope: n(-0.5),
            coordinate: band(0.0, 0.05),
        },
        extension: ForceCurve::new(
            [0.0, 0.04]
                .map(|x| CurvePoint {
                    displacement_m: n(x),
                    force_n: n(1000.0 * x),
                })
                .into(),
        )
        .unwrap(),
        damping_ns_m: n(0.0),
        friction: None,
        capacity_n: n(100.0),
        calibration: TensionCalibration {
            extension_m: band(0.0, 0.04),
            extension_speed_m_s: band(0.0, 5.0),
            payout_m: n(0.0),
        },
    })
    .unwrap();
    setup.routes.push(RouteBinding {
        model: route,
        points: points.into(),
        surfaces: vec![RouteSurface {
            patch: "прижим".into(),
            geometry: "б".into(),
            point: 1,
            reference: 0,
            normal: [0.0, 0.0, -1.0],
            tangent: [1.0, 0.0, 0.0],
            area_m2: Some(band(0.002, 0.004)),
        }],
        events: vec![ConfirmedInterfaceEvent {
            name: "трасса".into(),
            at: Time(2 * MS),
            kind: ConfirmedEvent::Release,
        }],
    });
    let mut rt = runtime(world(false), setup);
    run(&mut rt, 3);
    let steps = rt.process_journal();
    let get = |i: usize| steps[i].outputs["связь"].interaction.as_ref().unwrap();
    let first = &get(1).completed_routes["трасса"];
    near(first.drive_effort.unwrap().1, -5.0);
    near(MASS * steps[1].qvel[0] / DT, -15.0);
    let Analysis::Analyzed { value, .. } = &get(1).patches["прижим"].reaction else {
        panic!()
    };
    near(value.normal_n, 10.0);
    assert!(get(2).completed_routes["трасса"].drive_effort.is_some());
    assert!(get(2).interfaces["трасса"].drive_effort.is_none());
    let Analysis::Analyzed { value, .. } = &get(3).patches["прижим"].reaction else {
        panic!()
    };
    near(value.normal_n, 0.0);
    assert!(
        steps[2].outputs["связь"]
            .blockers
            .iter()
            .any(|s| s.contains("высвобождённой энергии"))
    );
}

#[derive(Clone)]
struct Gauge(TruthId, SignalId);
impl Sensor for Gauge {
    fn sample(
        &mut self,
        truth: &Truth<'_>,
        _: &mut Rng,
        out: &mut ObservationOut,
    ) -> Result<(), String> {
        if let Some(force) = truth.plant(self.0) {
            out.write(self.1, force, true);
        }
        Ok(())
    }
}

#[test]
fn controller_gets_force_only_from_an_installed_sensor() {
    for with_sensor in [false, true] {
        let mut rt = runtime(world(false), profile(0.03, Some(model_pad()), None));
        let truth = rt
            .publish_process_measurement(&patch_normal_signal("пятно"), "N")
            .unwrap();
        let id = rt.register_observation("измерение площадки", "N").unwrap();
        if with_sensor {
            rt.register_sensor("датчик", rt.physics_rate(), Gauge(truth, id))
                .unwrap();
        }
        rt.register_controller("контроллер", rt.physics_rate(), move |obs| {
            Command {
                targets: obs
                    .samples
                    .iter()
                    .filter(|s| s.signal == id && s.valid)
                    .map(|s| (ChannelId(1), s.value))
                    .collect(),
            }
        })
        .unwrap();
        run(&mut rt, 4);
        let first = rt
            .log()
            .entries()
            .iter()
            .find(|e| matches!(e.kind, LogKind::Command(_)));
        if with_sensor {
            let first = first.expect("после публикации датчик доставляет силу");
            assert_eq!(first.at, Time(2 * MS));
            near(first.value, 10.0);
        } else {
            assert!(first.is_none());
            assert!(
                rt.log()
                    .entries()
                    .iter()
                    .any(|e| e.kind == LogKind::Truth && e.value > 0.0)
            );
        }
    }
}

#[test]
fn invalid_profile_and_response_time_overflow_are_errors() {
    for bad in [f64::NAN, f64::INFINITY] {
        let mut rt = Runtime::load(world(false)).unwrap();
        assert!(register_interaction(&mut rt, profile(bad, None, None)).is_err());
        assert!(rt.interaction_identity().is_none());
    }
    let mut setup = profile(0.03, Some(model_pad()), Some(true));
    let MotionProfile::Dynamic {
        response: Some(response),
        ..
    } = &mut setup.motion
    else {
        panic!()
    };
    response.latency_ns = u64::MAX;
    let mut rt = runtime(world(false), setup);
    assert_eq!(
        rt.run_until(Time(2 * MS)).unwrap(),
        Status::ComponentFailure
    );
    assert!(rt.failure().unwrap().contains("переполнение"));
    assert!(reactions(&rt).is_empty());
}

#[derive(Clone)]
struct Powered;
impl Object for Powered {
    fn update(
        &mut self,
        _: &Truth<'_>,
        command: &Command,
        _: &mut Rng,
        out: &mut Forces,
    ) -> Result<(), String> {
        if command.targets.contains(&(ChannelId(2), 1.0)) {
            // Независимая нагрузка 6 Н, прекращающаяся при снятии питания.
            out.set(ChannelId(1), -6.0);
        }
        Ok(())
    }
}

struct Assets;
impl ControlAssets for Assets {
    fn read(&self, _: &str) -> Result<Vec<u8>, String> {
        Ok("version=1\n[[block]]\nkind='constant'\nvalue=1.0\nsource='стенд'\n[[bind]]\nnode=0\noutput='выход'\nmissing='omit'\nsource='стенд'".as_bytes().to_vec())
    }
    fn executable(&self, _: &str) -> Result<std::path::PathBuf, String> {
        Err("на стенде только встроенный контроллер".into())
    }
}

#[test]
fn power_off_while_loaded_removes_only_powered_effort() {
    let mut rt = runtime(world(false), profile(0.03, Some(model_pad()), None));
    rt.register_object("привод", rt.physics_rate(), &[ChannelId(2)], Powered)
        .unwrap();
    let config = ControlConfig::parse(
        r#"
version=1
[[controller]]
name='подача'
program='стенд'
rate_hz=1000
kind='builtin'
source='синтетический стенд'
[[output]]
name='выход'
controller='подача'
target='питание'
target_kind='power'
unit='bool'
max_age_ns=4000000
fallback=0.0
source='синтетический стенд'
[[fault]]
id='обрыв'
target='power:питание'
at_ns=4000000
until_ns=7000000
kind='power_off'
source='синтетический стенд'
"#,
    )
    .unwrap();
    control::install(
        &mut rt,
        &config,
        &Assets,
        RunSpec {
            world: "каретка".as_bytes(),
            catalog: "стенд".as_bytes(),
            task: "возмущение".as_bytes(),
            criteria: "импульс".as_bytes(),
            mode: "отказ",
            horizon: Time(8 * MS),
            model_ports: &[ModelPort::Power {
                channel: ChannelId(2),
            }],
        },
    )
    .unwrap();
    run(&mut rt, 8);
    let view = control::run(&rt).unwrap();
    assert!(view.events().iter().any(|e| e.at == Time(4 * MS)
        && matches!(&e.kind, EventKind::FaultApplied { id, active: true, .. } if id == "обрыв")));
    for (i, pair) in rt.process_journal().windows(2).enumerate() {
        let [a, b] = pair else { panic!() };
        let patch = &b.outputs["связь"].interaction.as_ref().unwrap().patches["пятно"];
        let Analysis::Analyzed { value, .. } = &patch.reaction else {
            panic!()
        };
        assert!(value.normal_n > 0.0);
        let powered = if (4..7).contains(&i) { 0.0 } else { -6.0 };
        let motion = a.outputs["связь"].efforts[0].1;
        near(
            MASS * (b.qvel[0] - a.qvel[0]) / DT,
            value.normal_n + motion + powered,
        );
    }
}
