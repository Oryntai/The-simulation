use sim_process::attachments::*;
use sim_process::work::HazardKind;
use sim_process::{Analysis, Interval, SourcedRange, StepModel};
use sim_signal::{Rate, Rng, Time};
use sim_world::{CompiledWorld, MountKind, Provenance, set_attachment_active};
use std::f64::consts::TAU;

fn sourced(value: f64) -> SourcedRange {
    band(value, value, value)
}

fn band(min: f64, nominal: f64, max: f64) -> SourcedRange {
    SourcedRange {
        min,
        nominal,
        max,
        provenance: Provenance::Measured("синтетический стенд с заданной характеристикой".into()),
    }
}

fn known<T>(value: T) -> Analysis<T> {
    Analysis::Analyzed {
        value,
        scope: "синтетический стенд".into(),
    }
}

fn unknown<T>() -> Analysis<T> {
    Analysis::NotAnalyzed {
        reason: "стенд не предоставил отсчёт".into(),
    }
}

fn value<T>(analysis: &Analysis<T>) -> &T {
    match analysis {
        Analysis::Analyzed { value, .. } => value,
        Analysis::NotAnalyzed { reason } => panic!("требовался расчёт: {reason}"),
    }
}

fn near(actual: f64, expected: f64) {
    // Допуск на ошибки округления в арифметике синтетического стенда.
    assert!((actual - expected).abs() <= 1e-12, "{actual} != {expected}");
}

fn limits() -> CaptureLimits {
    CaptureLimits {
        gap_m: sourced(0.016),
        lateral_m: sourced(0.001),
        axis_error_rad: sourced(0.01),
        speed_m_s: sourced(0.02),
        angular_speed_rad_s: sourced(0.1),
        required_travel_m: sourced(0.03),
        release_force_n: sourced(2.0),
        release_torque_nm: sourced(0.1),
        capacity: WrenchCapacity {
            force_n: sourced(100.0),
            torque_nm: sourced(10.0),
        },
    }
}

fn ports() -> [MountPort; 2] {
    ["a", "b"].map(|name| MountPort {
        name: format!("{name}/стык"),
        body: name.into(),
        kind: MountKind::Mechanical,
        interface: "посадка".into(),
    })
}

fn binding(kind: AttachmentKind) -> AttachmentBinding {
    AttachmentBinding {
        name: "связь".into(),
        mounts: ports().map(|p| p.name),
        kind,
    }
}

fn magnetic() -> AttachmentLaw {
    let points = [(0.0, 40.0), (0.04, 0.0)]
        .into_iter()
        .map(|(x, f)| CurvePoint {
            displacement_m: sourced(x),
            force_n: sourced(f),
        })
        .collect();
    AttachmentLaw::Magnetic {
        pull: ForceCurve::new(points).unwrap(),
    }
}

fn latch() -> AttachmentLaw {
    AttachmentLaw::Latch(LatchLaw {
        free_seat_m: sourced(0.0),
        engaged_seat_m: sourced(0.020),
        engagement_m: sourced(0.012),
        release_m: sourced(0.008),
        minimum_stroke_m: sourced(0.0),
        maximum_stroke_m: sourced(0.022),
        stiffness_n_m: sourced(1000.0),
        damping_ns_m: sourced(0.0),
        forward_contact_n: sourced(5.0),
        reverse_contact_n: sourced(5.0),
    })
}

fn thread() -> AttachmentLaw {
    AttachmentLaw::Threaded(ThreadLaw {
        lead_m_per_turn: sourced(0.008),
        entry_m: sourced(0.0),
        depth_m: sourced(0.04),
        phase_error_rad: sourced(0.07),
        maximum_elastic_error_m: sourced(0.003),
        stiffness_n_m: sourced(5000.0),
        damping_ns_m: sourced(8.0),
        torsional_drag_nms: sourced(0.0),
    })
}

fn adhesive() -> AttachmentLaw {
    AttachmentLaw::Adhesive(AdhesiveLaw {
        cure_s: sourced(0.2),
        compression_n: band(3.0, 5.0, 7.0),
        temperature_k: band(290.0, 295.0, 300.0),
    })
}

fn wrench(axial_n: f64) -> JointWrench {
    JointWrench {
        force_n: [axial_n, 0.0, 0.0],
        torque_nm: [0.0; 3],
    }
}

fn inlet(gap: f64) -> AttachmentInput {
    let base = MountMotion {
        point_m: [0.0; 3],
        com_m: [0.0, 0.1, 0.0],
        axis: [1.0, 0.0, 0.0],
        velocity_m_s: [0.0; 3],
        angular_velocity_rad_s: [0.0; 3],
    };
    let other = MountMotion {
        point_m: [gap, 0.0, 0.0],
        com_m: [gap, 0.1, 0.0],
        ..base
    };
    AttachmentInput {
        mounts: [base, other],
        available_travel_m: known(0.05),
        measured_wrench: known(wrench(0.0)),
        latch_contact_n: known(0.0),
        thread: None,
        cure: unknown(),
        weld_residual: None,
        command: None,
    }
}

fn request(action: AttachmentAction) -> Option<AttachmentRequest> {
    Some(AttachmentRequest {
        at: Time::ZERO,
        name: "связь".into(),
        action,
    })
}

fn tick(model: &mut AttachmentModel, k: u64, input: &AttachmentInput) -> AttachmentStep {
    let clock = Rate::hz(10).unwrap(); // 0,1 с за такт стенда.
    model
        .advance(
            Interval {
                start: clock.tick(k),
                end: clock.tick(k + 1),
            },
            input,
            &mut Rng::stream(7, "крепление"),
        )
        .unwrap()
}

fn event(step: &AttachmentStep) -> AttachmentEvent {
    let change = step.change.as_ref().expect("ожидался переход");
    AttachmentEvent {
        request: change.request.clone(),
        evaluated_at: step.at,
        outcome: AttachmentOutcome::Transition {
            from: change.from,
            to: change.to,
        },
    }
}

fn confirm(model: &mut AttachmentModel, step: &AttachmentStep, cause: AttachmentCause) {
    assert_eq!(step.change.as_ref().unwrap().cause, cause);
    model.confirm(&event(step)).unwrap();
}

fn rejected(step: &AttachmentStep) {
    assert!(step.change.is_none());
    assert!(matches!(
        step.event.as_ref().unwrap().outcome,
        AttachmentOutcome::Rejected { .. }
    ));
}

#[test]
fn magnetic_gap_force_work_hold_and_release() {
    let mut model = AttachmentModel::new(
        binding(AttachmentKind::Magnetic),
        ports(),
        limits(),
        magnetic(),
    )
    .unwrap();
    let mut input = inlet(0.01);
    input.command = request(AttachmentAction::Attach);
    let step = tick(&mut model, 0, &input);
    assert_eq!(model.state(), AttachmentState::Free);
    assert_eq!(step.change.as_ref().unwrap().effect, LinkEffect::None);
    // 40 Н на нулевом зазоре, линейный спад до нуля на 40 мм.
    let r = value(&step.reaction);
    near(r.applied.force_n[0], -30.0);
    near(*value(&r.stored_energy_j), (40.0 + 30.0) * 0.01 / 2.0);
    near(step.loads[0].force_n[0] + step.loads[1].force_n[0], 0.0);
    near(step.loads[1].torque_nm[2], -3.0);
    near(step.loads[0].torque_nm[2], 3.0);
    let mut wrong = event(&step);
    wrong.evaluated_at = Time::ZERO;
    assert!(model.confirm(&wrong).is_err());
    assert_eq!(model.state(), AttachmentState::Free);
    confirm(&mut model, &step, AttachmentCause::Captured);

    input.measured_wrench = known(wrench(20.0));
    input.command = request(AttachmentAction::Release);
    rejected(&tick(&mut model, 1, &input));
    assert_eq!(model.state(), AttachmentState::Attached);

    input = inlet(0.018);
    input.mounts[1].velocity_m_s[0] = 0.01;
    let hold = tick(&mut model, 2, &input);
    input.command = request(AttachmentAction::Release);
    let release = tick(&mut model, 3, &input);
    assert_eq!(hold.loads, release.loads);
    near(
        *value(&release.change.as_ref().unwrap().released_energy_j),
        0.0,
    );
    confirm(&mut model, &release, AttachmentCause::Released);
    assert_eq!(model.state(), AttachmentState::Free);

    input = inlet(0.01);
    input.command = request(AttachmentAction::Attach);
    let capture = tick(&mut model, 4, &input);
    confirm(&mut model, &capture, AttachmentCause::Captured);
    input.command = None;
    input.measured_wrench = known(wrench(31.0));
    let tear = tick(&mut model, 5, &input);
    confirm(&mut model, &tear, AttachmentCause::PullOff);
    near(value(&tear.reaction).applied.force_n[0], -30.0);
    let far = tick(&mut model, 6, &inlet(0.05));
    near(value(&far.reaction).applied.force_n[0], 0.0);
}

#[test]
fn invalid_mount_geometry_speed_load_and_missing_travel_cannot_attach() {
    let config = binding(AttachmentKind::Magnetic);
    let [a, mut b] = ports();
    b.interface = "другая посадка".into();
    assert!(AttachmentModel::new(config.clone(), [a.clone(), b], limits(), magnetic()).is_err());
    let mut b = ports()[1].clone();
    b.kind = MountKind::Power;
    assert!(AttachmentModel::new(config.clone(), [a, b], limits(), magnetic()).is_err());
    let mut model = AttachmentModel::new(config, ports(), limits(), magnetic()).unwrap();
    for (k, mut input) in [inlet(0.02), inlet(0.01), inlet(0.01), inlet(0.01)]
        .into_iter()
        .enumerate()
    {
        input.command = request(AttachmentAction::Attach);
        match k {
            1 => input.mounts[1].velocity_m_s[0] = 0.021,
            2 => input.available_travel_m = known(0.001),
            3 => input.measured_wrench = known(wrench(1000.0)),
            _ => {}
        }
        rejected(&tick(&mut model, k as u64, &input));
    }
    let mut input = inlet(0.01);
    input.command = request(AttachmentAction::Attach);
    input.available_travel_m = unknown();
    let step = tick(&mut model, 4, &input);
    assert!(matches!(
        step.event.unwrap().outcome,
        AttachmentOutcome::NotAnalyzed { .. }
    ));
    assert!(
        step.hazards
            .iter()
            .all(|h| matches!(h.coverage, Analysis::NotAnalyzed { .. }))
    );
    assert!(step.hazards.iter().any(|h| h.kind == HazardKind::Pinch));
    input.mounts[1].point_m[0] = f64::NAN;
    let interval = Interval {
        start: Time(500_000_000),
        end: Time(600_000_000),
    };
    assert!(
        model
            .advance(interval, &input, &mut Rng::stream(7, "крепление"))
            .is_err()
    );
    input.mounts[1].point_m[0] = 0.01;
    input.thread = Some(ThreadMotion {
        angle_rad: 0.0,
        angular_speed_rad_s: f64::INFINITY,
    });
    assert!(
        model
            .advance(interval, &input, &mut Rng::stream(7, "крепление"))
            .is_err()
    );
    input.thread = None;
    input.mounts[1].point_m[1] = 0.005;
    assert!(matches!(
        tick(&mut model, 5, &input).reaction,
        Analysis::NotAnalyzed { .. }
    ));
    assert_eq!(model.state(), AttachmentState::Free);
}

#[test]
fn latch_requires_contact_preserves_state_and_returns_with_hysteresis() {
    let mut model =
        AttachmentModel::new(binding(AttachmentKind::Latch), ports(), limits(), latch()).unwrap();
    let mut input = inlet(0.014);
    input.command = request(AttachmentAction::Attach);
    let step = tick(&mut model, 0, &input);
    rejected(&step);
    near(value(&step.reaction).applied.force_n[0], -14.0);

    input.command = None;
    input.latch_contact_n = known(6.0);
    let captured = tick(&mut model, 1, &input);
    near(value(&captured.reaction).applied.force_n[0], 6.0);
    // 0,5 k (0,014² − 0,006²), работа при переходе на другой минимум.
    near(
        *value(&captured.change.as_ref().unwrap().released_energy_j),
        0.08,
    );
    confirm(&mut model, &captured, AttachmentCause::Captured);

    let equilibrium = tick(&mut model, 2, &inlet(0.020));
    assert_eq!(equilibrium.state, AttachmentState::Attached);
    near(value(&equilibrium.reaction).applied.force_n[0], 0.0);
    let mut input = inlet(0.006);
    input.command = request(AttachmentAction::Release);
    input.latch_contact_n = known(-6.0);
    input.measured_wrench = known(wrench(5.0));
    rejected(&tick(&mut model, 3, &input));
    input.measured_wrench = known(wrench(0.0));
    let returned = tick(&mut model, 4, &input);
    // При возврате упругая характеристика направляет точку к исходному минимуму.
    near(value(&returned.reaction).applied.force_n[0], -6.0);
    near(
        *value(&returned.change.as_ref().unwrap().released_energy_j),
        0.08,
    );
    confirm(&mut model, &returned, AttachmentCause::Released);
    assert_eq!(model.state(), AttachmentState::Free);
    near(
        value(&tick(&mut model, 5, &inlet(0.0)).reaction)
            .applied
            .force_n[0],
        0.0,
    );
}

#[test]
fn threaded_link_obeys_lead_and_virtual_work_then_reverses_or_strips() {
    let mut model = AttachmentModel::new(
        binding(AttachmentKind::Threaded),
        ports(),
        limits(),
        thread(),
    )
    .unwrap();
    let mut input = inlet(0.0);
    input.command = request(AttachmentAction::Attach);
    input.thread = Some(ThreadMotion {
        angle_rad: 0.2,
        angular_speed_rad_s: 0.0,
    });
    rejected(&tick(&mut model, 0, &input));
    input.thread.as_mut().unwrap().angle_rad = 0.0;
    let capture = tick(&mut model, 1, &input);
    confirm(&mut model, &capture, AttachmentCause::Captured);

    let mut input = inlet(0.008);
    input.thread = Some(ThreadMotion {
        angle_rad: TAU,
        angular_speed_rad_s: 0.0,
    });
    let turn = tick(&mut model, 2, &input);
    near(value(&turn.reaction).applied.force_n[0], 0.0);
    input.mounts[1].point_m[0] = 0.010;
    let loaded = tick(&mut model, 3, &input);
    let r = value(&loaded.reaction);
    near(r.applied.force_n[0], -10.0);
    near(
        r.applied.torque_nm[0] * TAU + r.applied.force_n[0] * 0.008,
        0.0,
    );
    near(*value(&r.stored_energy_j), 0.010);
    input.command = request(AttachmentAction::Release);
    rejected(&tick(&mut model, 4, &input));

    let mut input = inlet(0.0);
    input.thread = Some(ThreadMotion {
        angle_rad: 0.0,
        angular_speed_rad_s: -TAU,
    });
    input.mounts[1].velocity_m_s[0] = -0.008;
    // Остаточная упругая невязка 10 мкм отделяется без импульса,
    // а её энергия остаётся в событии отпускания.
    input.mounts[1].point_m[0] = -1e-5;
    let backed_out = tick(&mut model, 5, &input);
    confirm(&mut model, &backed_out, AttachmentCause::Released);
    near(
        *value(&backed_out.change.unwrap().released_energy_j),
        2.5e-7,
    );

    input.command = request(AttachmentAction::Attach);
    input.thread.as_mut().unwrap().angular_speed_rad_s = 0.0;
    input.mounts[1].point_m[0] = 0.0;
    input.mounts[1].velocity_m_s[0] = 0.0;
    let capture = tick(&mut model, 6, &input);
    confirm(&mut model, &capture, AttachmentCause::Captured);
    input.command = None;
    input.mounts[1].point_m[0] = 0.005;
    let stripped = tick(&mut model, 7, &input);
    assert!(matches!(
        stripped.change.as_ref().unwrap().released_energy_j,
        Analysis::NotAnalyzed { .. }
    ));
    confirm(&mut model, &stripped, AttachmentCause::Stripped);
    assert!(tick(&mut model, 8, &input).failed);
}

fn cure_sample() -> AttachmentInput {
    let mut input = inlet(0.0);
    input.cure = known(CureObservation {
        compression_n: [4.0, 6.0],
        temperature_k: [295.0, 296.0],
        gap_m: 0.0,
        lateral_m: 0.0,
        axis_error_rad: 0.0,
        speed_m_s: 0.0,
        angular_speed_rad_s: 0.0,
    });
    input.weld_residual = Some(WeldResidual {
        translation_m: 0.0,
        rotation_rad: 0.0,
    });
    input
}

#[test]
fn adhesive_requires_uninterrupted_cure_and_matching_weld_then_breaks_under_load() {
    let mut model = AttachmentModel::new(
        binding(AttachmentKind::Adhesive),
        ports(),
        limits(),
        adhesive(),
    )
    .unwrap();
    let mut input = cure_sample();
    input.command = request(AttachmentAction::Attach);
    let started = tick(&mut model, 0, &input);
    confirm(&mut model, &started, AttachmentCause::CureStarted);
    input.command = None;
    assert!(tick(&mut model, 1, &input).change.is_none());
    let mut interrupted = input.clone();
    interrupted.cure = unknown();
    assert!(matches!(
        tick(&mut model, 2, &interrupted).reaction,
        Analysis::NotAnalyzed { .. }
    ));
    assert!(tick(&mut model, 3, &input).change.is_none());
    input.weld_residual.as_mut().unwrap().translation_m = 0.001;
    rejected(&tick(&mut model, 4, &input));
    input.weld_residual.as_mut().unwrap().translation_m = 0.0;
    assert!(tick(&mut model, 5, &input).change.is_none());
    let finished = tick(&mut model, 6, &input);
    assert_eq!(
        finished.change.as_ref().unwrap().effect,
        LinkEffect::Weld(true)
    );
    confirm(&mut model, &finished, AttachmentCause::Captured);
    input.command = request(AttachmentAction::Release);
    rejected(&tick(&mut model, 7, &input));
    input.command = None;
    input.measured_wrench = known(wrench(120.0));
    let torn = tick(&mut model, 8, &input);
    assert_eq!(
        torn.change.as_ref().unwrap().effect,
        LinkEffect::Weld(false)
    );
    confirm(&mut model, &torn, AttachmentCause::Overload);
    assert_eq!(model.state(), AttachmentState::Free);
}

#[test]
fn mujoco_weld_switch_preserves_common_motion_and_exposes_real_reaction() {
    // Два свободных блока по 2 кг, совместная скорость 0,3 м/с и нулевая угловая скорость.
    let world = CompiledWorld {
        name: "блоки".into(),
        seed: 7,
        physics: Rate::hz(1000).unwrap(),
        mjcf: r#"<mujoco><option gravity="0 0 0" timestep="0.001"/>
          <worldbody>
            <body name="a"><freejoint/><inertial pos="0 0 0" mass="2" diaginertia="1 1 1"/></body>
            <body name="b"><freejoint/><inertial pos="0 0 0" mass="2" diaginertia="1 1 1"/></body>
          </worldbody>
          <equality><weld name="attachment/test" body1="a" body2="b" active="false"/></equality>
        </mujoco>"#
            .into(),
        bodies: vec![],
        joints: vec![],
        efforts: vec![],
        drives: vec![],
        sensors: vec![],
        simplifications: vec![],
    };
    let mut sim = world.open_contact_sim().unwrap();
    sim.qvel_mut()[0] = 0.3;
    sim.qvel_mut()[6] = 0.3;
    let before = (sim.qpos().to_vec(), sim.qvel().to_vec());
    assert!(!sim.equality_active("attachment/test").unwrap());
    for active in [true, false, true] {
        set_attachment_active(&mut sim, "test", active).unwrap();
        assert_eq!(sim.equality_active("attachment/test").unwrap(), active);
        assert_eq!(before.0, sim.qpos());
        assert_eq!(before.1, sim.qvel());
        let kinetic = sim.qvel()[0].powi(2) + sim.qvel()[6].powi(2);
        near(kinetic, 0.18);
    }
    sim.step();
    near(sim.qvel()[0], 0.3);
    near(sim.qvel()[6], 0.3);
    sim.qfrc_applied_mut()[6] = 20.0;
    for _ in 0..100 {
        // 100 мс позволяют реакции решателя установиться.
        sim.step();
    }
    let reaction = sim.equality_reaction("attachment/test").unwrap().unwrap();
    near(reaction[0] + reaction[6], 0.0);
    // Половина внешних 20 Н ускоряет второй блок с той же скоростью.
    assert!((reaction[0] - 10.0).abs() < 0.1, "{reaction:?}");
    assert!((reaction[6] + 10.0).abs() < 0.1, "{reaction:?}");
}
