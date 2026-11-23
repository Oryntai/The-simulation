use std::f64::consts::FRAC_PI_2;

use sim_process::attachments::*;
use sim_process::contact::{ContactFrame, ContactSample};
use sim_process::{Analysis, Interval, ProcessError, SourcedRange, StepModel};
use sim_signal::{ChannelId, Rate, Rng, Time};
use sim_world::{CompiledWorld, Provenance};

fn band(min: f64, nominal: f64, max: f64) -> SourcedRange {
    SourcedRange {
        min,
        nominal,
        max,
        provenance: Provenance::Measured("параметр синтетического стенда".into()),
    }
}

fn exact(x: f64) -> SourcedRange {
    band(x, x, x)
}

fn curve(k: f64, max: f64) -> ForceCurve {
    ForceCurve::new(
        [0.0, max / 2.0, max]
            .map(|x| CurvePoint {
                displacement_m: exact(x),
                force_n: exact(k * x),
            })
            .into(),
    )
    .unwrap()
}

fn value<T>(result: &Analysis<T>) -> &T {
    match result {
        Analysis::Analyzed { value, .. } => value,
        Analysis::NotAnalyzed { reason } => panic!("требовалось значение: {reason}"),
    }
}

fn unavailable<T>(result: &Analysis<T>) -> bool {
    matches!(result, Analysis::NotAnalyzed { .. })
}

fn close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} != {expected}, допуск {tolerance}"
    );
}

fn near(actual: f64, expected: f64) {
    // Для ручных подстановок без интегрирования запас на округление.
    close(actual, expected, 1e-10);
}

fn xyz(actual: [f64; 3], expected: [f64; 3]) {
    for (a, b) in actual.into_iter().zip(expected) {
        near(a, b);
    }
}

fn interval(index: u64) -> Interval {
    // Квазистатические пробы и история сдвига на шагах 10 мс.
    let rate = Rate::hz(100).unwrap();
    Interval {
        start: rate.tick(index),
        end: rate.tick(index + 1),
    }
}

fn advance(
    model: &mut impl StepModel<Input = InterfaceInput, Output = InterfaceStep>,
    i: u64,
    input: InterfaceInput,
) -> Result<InterfaceStep, ProcessError> {
    model.advance(interval(i), &input, &mut Rng::stream(7, "стенд"))
}

fn event(name: &str, i: u64, kind: ConfirmedEvent) -> ConfirmedInterfaceEvent {
    ConfirmedInterfaceEvent {
        name: name.into(),
        at: interval(i).start,
        kind,
    }
}

fn pad_spec() -> PadSpec {
    // Плоский стенд: зазор 40 мм в ненагруженной геометрии,
    // законы N = 1000*x + 20*ẋ и F_t = -100*s до |F_t| = 0,5*N.
    PadSpec {
        name: "площадка".into(),
        bodies: ["a".into(), "b".into()],
        geometries: ["ga".into(), "gb".into()],
        owner: PadReactionOwner::Model(Box::new(PadLaw {
            free_gap_m: exact(0.04),
            compression: curve(1000.0, 0.04),
            damping_ns_m: exact(20.0),
            shear_stiffness_n_m: exact(100.0),
            friction: exact(0.5),
            capacity_n: exact(100.0),
        })),
        area_m2: Some(band(0.002, 0.003, 0.004)),
        calibration: PadCalibration {
            compression_m: band(0.0, 0.01, 0.035),
            offset_m: band(0.0, 0.01, 0.05),
            axis_error_rad: band(0.0, 0.025, 0.05),
            relative_speed_m_s: band(0.0, 1.0, 5.0),
        },
    }
}

fn pad_input(at: Time, gap: f64, offset: f64, vx: f64, vz: f64) -> PadInput {
    let a = MountMotion {
        point_m: [0.0; 3],
        com_m: [0.0; 3],
        axis: [0.0, 0.0, 1.0],
        velocity_m_s: [0.0; 3],
        angular_velocity_rad_s: [0.0; 3],
    };
    PadInput {
        mounts: [
            a,
            MountMotion {
                point_m: [offset, 0.0, gap],
                com_m: [offset, 0.0, gap],
                velocity_m_s: [vx, 0.0, vz],
                ..a
            },
        ],
        tangent_a: [1.0, 0.0, 0.0],
        completed: Some(ContactFrame {
            at,
            contacts: Vec::new(),
        }),
        event: None,
    }
}

#[test]
fn calibrated_patch_area_follows_compression_and_does_not_extrapolate() {
    // Синтетическая характеристика пятна: A = 0,2·x до 20 мм.
    let points = [0.0, 0.01, 0.02].map(|compression_m| PatchAreaPoint {
        compression_m,
        area_m2: exact(0.2 * compression_m),
    });
    let mut spec = pad_spec();
    spec.area_m2 = None;
    let mut pad = CompliantPad::new(spec)
        .unwrap()
        .with_area_curve(points.to_vec())
        .unwrap();
    for (i, gap, expected) in [
        (0, 0.03, Some([0.002, 0.004])),
        (1, 0.024, Some([0.002, 0.004])),
        (2, 0.015, None),
        (3, 0.045, None),
    ] {
        let input = pad_input(interval(i).start, gap, 0.0, 0.5, 0.0);
        let step = advance(&mut pad, i, InterfaceInput::Pad(input)).unwrap();
        if let Some([low, high]) = expected {
            let area = step.effective_area_m2.unwrap();
            near(area.min, low);
            near(area.max, high);
            let normal = 1000.0 * (0.04 - gap);
            let shear = 100.0 * 0.01 * 0.5 * (i + 1) as f64;
            let stress = value(&step.stress);
            near(stress.pressure_pa[1], normal / low);
            near(stress.shear_pa[1], shear / low);
        } else {
            assert!(step.effective_area_m2.is_none());
            assert!(unavailable(&step.stress));
        }
    }
    assert!(pad.with_area_curve(vec![points[0].clone(); 2]).is_err());
}

#[test]
fn pad_compression_preload_unload_and_misalignment() {
    let mut pad = CompliantPad::new(pad_spec()).unwrap();
    for (i, vz, normal, work, loss) in [
        (0, 0.0, 10.0, 0.0, 0.0),
        (1, -0.2, 14.0, -0.028, 0.008),
        (2, 0.2, 6.0, 0.012, 0.008),
        (3, 1.0, 0.0, 0.0, 0.1),
    ] {
        let input = pad_input(interval(i).start, 0.03, 0.003, 0.0, vz);
        let step = advance(&mut pad, i, InterfaceInput::Pad(input)).unwrap();
        let reaction = value(&step.reaction);
        let energy = value(&step.work);
        // За исходные 10 мм kx²/2=0,05 Дж. При быстром раскрытии
        // 20 Н вязкой силы отсечены до нулевой полной реакции.
        near(reaction.normal_n, normal);
        near(energy.stored_energy_j, 0.05);
        near(energy.body_work_j, work);
        near(energy.loss_j, loss);
        xyz(reaction.loads[0].torque_nm, [0.0, normal * 0.003, 0.0]);
        xyz(reaction.loads[1].torque_nm, [0.0; 3]);
        if normal > 0.0 {
            let area = value(&step.stress);
            near(area.pressure_pa[0], normal / 0.004);
            near(area.pressure_pa[1], normal / 0.002);
        } else {
            assert!(step.body_loads.is_empty() && unavailable(&step.stress));
        }
    }

    let mut input = pad_input(interval(4).start, 0.03, 0.003, 0.0, 0.0);
    input.mounts[1].axis = [1.0, 0.0, 0.0];
    let skew = advance(&mut pad, 4, InterfaceInput::Pad(input)).unwrap();
    near(skew.axis_error_rad.unwrap(), FRAC_PI_2);
    near(skew.offset_m.unwrap(), 0.003);
    near(value(&skew.reaction).normal_n, 10.0);
    assert!(unavailable(&skew.calibration) && unavailable(&skew.stress));

    let opening = pad_input(interval(5).start, 0.041, 0.003, 0.0, -1.0);
    let separated = advance(&mut pad, 5, InterfaceInput::Pad(opening)).unwrap();
    assert_eq!(separated.state, InterfaceState::Open);
    assert!(separated.body_loads.is_empty());
    near(value(&separated.reaction).normal_n, 0.0);
    near(value(&separated.work).stored_energy_j, 0.0);
}

#[test]
fn pad_shear_reversal_coulomb_work_and_independent_clone() {
    let mut pad = CompliantPad::new(pad_spec()).unwrap();
    let mut old_shear_energy = 0.0;
    for (i, vx, shear, slip, loss) in [
        (0, 2.0, -2.0, 0.0, 0.0),
        (1, 4.0, -5.0, 0.01, 0.05),
        (2, -4.0, -1.0, 0.0, 0.0),
        (3, -4.0, 3.0, 0.0, 0.0),
    ] {
        let input = pad_input(interval(i).start, 0.03, 0.0, vx, 0.0);
        let step = advance(&mut pad, i, InterfaceInput::Pad(input)).unwrap();
        let r = value(&step.reaction);
        near(r.shear_n[0], shear);
        near(step.slip_m.unwrap(), slip);
        near(value(&step.stress).shear_pa[1], shear.abs() / 0.002);
        assert_eq!(
            step.state,
            if slip > 0.0 {
                InterfaceState::Sliding
            } else {
                InterfaceState::Loaded
            }
        );
        let work = value(&step.work);
        near(work.loss_j, loss);
        let shear_energy = shear * shear / 200.0;
        near(work.stored_energy_j, 0.05 + shear_energy);
        near(
            -work.body_work_j - (shear_energy - old_shear_energy),
            work.loss_j + work.numerical_loss_j,
        );
        assert!(work.loss_j >= 0.0 && work.numerical_loss_j >= 0.0);
        old_shear_energy = shear_energy;
    }
    let mut branch = pad.clone();
    let mut control = pad.clone();
    let input = pad_input(interval(4).start, 0.03, 0.0, 0.0, 0.0);
    let mut release = input.clone();
    release.event = Some(event("площадка", 4, ConfirmedEvent::Release));
    let off = advance(&mut branch, 4, InterfaceInput::Pad(release)).unwrap();
    assert_eq!(off.state, InterfaceState::Released);
    assert_eq!(off.transition, Some(Transition::ConfirmedRelease));
    assert!(off.body_loads.is_empty());
    near(value(&off.work).released_energy_j, 0.05 + 9.0 / 200.0);

    let original = advance(&mut pad, 4, InterfaceInput::Pad(input.clone())).unwrap();
    let copy = advance(&mut control, 4, InterfaceInput::Pad(input)).unwrap();
    assert_eq!(original, copy);
    assert_eq!(
        value(&original.work).stored_energy_j.to_bits(),
        value(&copy.work).stored_energy_j.to_bits()
    );
    near(value(&original.reaction).shear_n[0], 3.0);
    let still_off = advance(
        &mut branch,
        5,
        InterfaceInput::Pad(pad_input(interval(5).start, 0.03, 0.0, 0.0, 0.0)),
    )
    .unwrap();
    assert_eq!(still_off.state, InterfaceState::Released);
    assert!(still_off.body_loads.is_empty());
}

#[test]
fn rotating_pad_uses_contact_point_moment_and_objective_shear() {
    let mut pad = CompliantPad::new(pad_spec()).unwrap();
    let mut input = pad_input(Time::ZERO, 0.03, 0.02, 0.0, 0.0);
    // Жёсткий общий поворот на 2 рад/с вокруг Y даёт ω×r=(0,06, 0, −0,04).
    input.mounts[0].angular_velocity_rad_s = [0.0, 2.0, 0.0];
    input.mounts[1].velocity_m_s = [0.06, 0.0, -0.04];
    let step = advance(&mut pad, 0, InterfaceInput::Pad(input)).unwrap();
    near(value(&step.reaction).normal_n, 10.0);
    xyz(value(&step.reaction).shear_n, [0.0; 3]);
    near(step.body_loads[0].torque_nm[1], 0.2);
    // Мощность пары M_A*ω_A + F_B*v_B = 0,4 − 0,4 = 0 Вт.
    near(value(&step.work).body_work_j, 0.0);
}

fn contact(first: &str, second: &str, n: [f64; 3], force: [f64; 3]) -> ContactSample {
    ContactSample {
        geometries: [first.into(), second.into()],
        normal: n,
        force_n: force,
        separation_m: -0.001,
        point_m: [0.02, 0.0, 0.0],
        torque_nm: [0.0, 0.0, 0.1],
    }
}

#[test]
fn engine_reactions_count_once_with_reversed_geometries_and_conservative_area() {
    let mut spec = pad_spec();
    spec.owner = PadReactionOwner::Engine;
    let mut engine = CompliantPad::new(spec).unwrap();
    let mut input = pad_input(Time::ZERO, -0.001, 0.0, 0.0, 0.0);
    let frame = input.completed.as_mut().unwrap();
    frame.contacts = vec![
        contact("ga", "gb", [0.0, 0.0, 1.0], [3.0, 0.0, 8.0]),
        contact("gb", "ga", [0.0, 0.0, -1.0], [2.0, 0.0, -4.0]),
        contact("other", "gb", [0.0, 1.0, 0.0], [0.0, 20.0, 0.0]),
    ];
    let observed = advance(&mut engine, 0, InterfaceInput::Pad(input.clone())).unwrap();
    assert!(observed.body_loads.is_empty());
    assert!(unavailable(&observed.work));
    let r = value(&observed.reaction);
    assert_eq!(r.contact_count, 2);
    near(r.normal_n, 12.0);
    xyz(r.shear_n, [1.0, 0.0, 0.0]);
    near(r.shear_upper_n, 5.0);
    near(r.peak_shear_n, 3.0);
    xyz(r.loads[1].force_n, [1.0, 0.0, 12.0]);
    xyz(r.loads[1].torque_nm, [0.0, -0.239, 0.0]);
    let stress = value(&observed.stress);
    near(stress.pressure_pa[0], 3000.0);
    near(stress.pressure_pa[1], 6000.0);
    near(stress.shear_pa[0], 250.0);
    near(stress.shear_pa[1], 2500.0);
    let mut deep = input.clone();
    let frame = deep.completed.as_mut().unwrap();
    frame.at = interval(1).start;
    frame.contacts[1].separation_m = -0.04;
    deep.event = Some(event("площадка", 1, ConfirmedEvent::Release));
    let released = advance(&mut engine, 1, InterfaceInput::Pad(deep.clone())).unwrap();
    assert_eq!(released.state, InterfaceState::Released);
    near(released.deformation_m.unwrap(), 0.04);
    near(value(&released.reaction).normal_n, 12.0);
    assert!(unavailable(&released.stress) && released.body_loads.is_empty());
    deep.event = None;
    deep.completed.as_mut().unwrap().at = interval(2).start;
    assert!(advance(&mut engine, 2, InterfaceInput::Pad(deep)).is_err());
    let mut model = CompliantPad::new(pad_spec()).unwrap();
    assert!(advance(&mut model, 0, InterfaceInput::Pad(input.clone())).is_err());
    // После ошибки тот же интервал можно повторить с проверенным входом.
    input.completed.as_mut().unwrap().contacts.clear();
    input.mounts[1].point_m[2] = 0.03;
    near(
        value(
            &advance(&mut model, 0, InterfaceInput::Pad(input))
                .unwrap()
                .reaction,
        )
        .normal_n,
        10.0,
    );
}

#[test]
fn pad_refuses_unknown_area_estimates_overload_and_invalid_state() {
    let specimen = pad_input(Time::ZERO, 0.03, 0.0, 0.0, 0.0);
    let mut spec = pad_spec();
    spec.area_m2 = None;
    let mut pad = CompliantPad::new(spec.clone()).unwrap();
    let step = advance(&mut pad, 0, InterfaceInput::Pad(specimen.clone())).unwrap();
    assert!(unavailable(&step.stress));
    near(value(&step.reaction).normal_n, 10.0);

    spec.area_m2 = Some(SourcedRange {
        provenance: Provenance::Estimate("оценка".into()),
        ..exact(0.002)
    });
    let PadReactionOwner::Model(law) = &mut spec.owner else {
        unreachable!()
    };
    law.capacity_n = band(9.0, 11.0, 12.0);
    let overloaded = advance(
        &mut CompliantPad::new(spec.clone()).unwrap(),
        0,
        InterfaceInput::Pad(specimen.clone()),
    )
    .unwrap();
    let Some(Transition::Overload { force_n, limit_n }) = overloaded.transition else {
        panic!("превышение нижней границы прочности не обнаружено");
    };
    near(force_n, 10.0);
    near(limit_n, 9.0);
    assert_eq!(overloaded.state, InterfaceState::Broken);
    assert!(overloaded.body_loads.is_empty() && unavailable(&overloaded.calibration));
    near(value(&overloaded.work).released_energy_j, 0.05);

    let mut estimate = pad_spec();
    estimate.area_m2 = spec.area_m2.clone();
    let estimated = advance(
        &mut CompliantPad::new(estimate).unwrap(),
        0,
        InterfaceInput::Pad(specimen.clone()),
    )
    .unwrap();
    assert!(unavailable(&estimated.stress));
    near(value(&estimated.reaction).normal_n, 10.0);

    let mut invalid_spec = pad_spec();
    invalid_spec.area_m2 = Some(exact(0.0));
    assert!(CompliantPad::new(invalid_spec).is_err());
    let mut invalid_spec = pad_spec();
    invalid_spec.calibration.offset_m.provenance = Provenance::Measured(" ".into());
    assert!(CompliantPad::new(invalid_spec).is_err());
    let mut model = CompliantPad::new(pad_spec()).unwrap();
    for bad in [f64::NAN, f64::INFINITY] {
        let mut bad_input = specimen.clone();
        bad_input.mounts[1].velocity_m_s[2] = bad;
        assert!(advance(&mut model, 0, InterfaceInput::Pad(bad_input)).is_err());
    }
    let mut bad_input = specimen.clone();
    bad_input.tangent_a = [1.0, 0.0, 1.0];
    assert!(advance(&mut model, 0, InterfaceInput::Pad(bad_input)).is_err());
    let mut bad_input = specimen.clone();
    bad_input.completed.as_mut().unwrap().at = interval(0).end;
    assert!(advance(&mut model, 0, InterfaceInput::Pad(bad_input)).is_err());
    let mut no_frame = specimen;
    no_frame.completed = None;
    let step = advance(&mut model, 0, InterfaceInput::Pad(no_frame)).unwrap();
    assert!(unavailable(&step.stress) && unavailable(&step.calibration));
    near(value(&step.reaction).normal_n, 10.0);
    let beyond = pad_input(interval(1).start, -0.01, 0.0, 0.0, 0.0);
    let step = advance(&mut model, 1, InterfaceInput::Pad(beyond)).unwrap();
    assert_eq!(step.state, InterfaceState::Unavailable);
    assert!(unavailable(&step.reaction));
    let returned = pad_input(interval(2).start, 0.03, 0.0, 0.0, 0.0);
    assert!(unavailable(
        &advance(&mut model, 2, InterfaceInput::Pad(returned))
            .unwrap()
            .reaction
    ));
}

fn tension_spec() -> TensionSpec {
    // Каретка меняет длину покоя 0,65 − 0,05 q м, осевая жёсткость 200 Н/м.
    TensionSpec {
        name: "трасса".into(),
        bodies: ["a".into(), "b".into(), "c".into()].into(),
        length: FreeLengthLaw::Affine {
            channel: ChannelId(2),
            at_zero_m: exact(0.65),
            slope: exact(-0.05),
            coordinate: band(-3.0, 0.0, 3.0),
        },
        extension: curve(200.0, 0.25),
        damping_ns_m: exact(4.0),
        friction: None,
        capacity_n: exact(40.0),
        calibration: TensionCalibration {
            extension_m: band(0.0, 0.05, 0.2),
            extension_speed_m_s: band(0.0, 0.5, 2.0),
            payout_m: band(0.0, 0.02, 0.1),
        },
    }
}

fn route_input(q: f64) -> TensionInput {
    TensionInput {
        points: [[0.0, 0.0, 0.0], [0.3, 0.0, 0.0], [0.3, 0.4, 0.0]]
            .map(|point_m| RoutePoint {
                point_m,
                com_m: [point_m[0], point_m[1], -0.02],
                velocity_m_s: [0.0; 3],
            })
            .into(),
        drive: Some(DriveMotion {
            coordinate: q,
            velocity: 0.0,
        }),
        event: None,
    }
}

#[test]
fn route_takes_up_slack_and_matches_geometric_gradient_and_drive_work() {
    let mut route = TensionRoute::new(tension_spec()).unwrap();
    let mut input = route_input(-2.0);
    input.points[2].velocity_m_s[1] = 0.5;
    let slack = advance(&mut route, 0, InterfaceInput::Tension(input)).unwrap();
    assert_eq!(slack.state, InterfaceState::Open);
    near(slack.deformation_m.unwrap(), -0.05);
    near(value(&slack.reaction).tension_n, 0.0);
    assert!(slack.body_loads.is_empty() && slack.drive_effort.is_none());

    let step = advance(&mut route, 1, InterfaceInput::Tension(route_input(0.0))).unwrap();
    let r = value(&step.reaction);
    near(step.path_length_m.unwrap(), 0.7);
    near(step.free_length_m.unwrap(), 0.65);
    near(r.tension_n, 10.0);
    near(value(&step.work).stored_energy_j, 0.25);
    xyz(step.route_forces_n[0], [10.0, 0.0, 0.0]);
    xyz(step.route_forces_n[1], [-10.0, 10.0, 0.0]);
    xyz(step.route_forces_n[2], [0.0, -10.0, 0.0]);
    xyz(step.body_loads[1].torque_nm, [-0.2, -0.2, 0.0]);
    let (channel, effort) = step.drive_effort.unwrap();
    assert_eq!(channel, ChannelId(2));

    // Независимая энергия геометрии по Пифагору.
    let energy = |q: f64, y: f64| 100.0 * (0.3f64.hypot(y) + (0.4 - y) - (0.65 - 0.05 * q)).powi(2);
    let h = 1e-6; // Центральная разность, ошибка O(h²) при ветви 0,3 м.
    close(
        effort,
        -(energy(h, 0.0) - energy(-h, 0.0)) / (2.0 * h),
        1e-8,
    );
    close(
        step.route_forces_n[1][1],
        -(energy(0.0, h) - energy(0.0, -h)) / (2.0 * h),
        1e-8,
    );
    near(effort, -0.5);

    let mut input = route_input(0.0);
    input.points[2].velocity_m_s[1] = 0.2;
    input.drive.as_mut().unwrap().velocity = 0.5;
    let moving = advance(&mut route, 2, InterfaceInput::Tension(input)).unwrap();
    let work = value(&moving.work);
    // ė = 0,2 − (−0,05·0,5) = 0,225 м/с, T=10 + 4·0,225.
    near(value(&moving.reaction).tension_n, 10.9);
    near(work.body_work_j, -10.9 * 0.2 * 0.01);
    near(work.drive_work_j, -10.9 * 0.025 * 0.01);
    near(work.loss_j, 4.0 * 0.225f64.powi(2) * 0.01);
    near(
        -work.body_work_j - work.drive_work_j - work.loss_j,
        10.0 * 0.225 * 0.01,
    );
}

#[test]
fn route_slip_break_invalid_inputs_and_clone() {
    let mut spec = tension_spec();
    spec.friction = Some(TractionFriction {
        coefficient: exact(0.3),
        clamp_normal_n: exact(20.0),
    });
    let mut route = TensionRoute::new(spec.clone()).unwrap();
    let mut input = route_input(0.0);
    input.points[2].velocity_m_s[1] = 0.2;
    let step = advance(&mut route, 0, InterfaceInput::Tension(input.clone())).unwrap();
    assert_eq!(step.state, InterfaceState::Sliding);
    near(step.payout_m.unwrap(), 0.022);
    near(value(&step.reaction).tension_n, 6.0);
    near(value(&step.work).loss_j, 0.012);
    near(value(&step.work).numerical_loss_j, 0.16);
    near(value(&step.work).stored_energy_j, 0.09);

    let mut branch = route.clone();
    let mut copy = route.clone();
    input.points[2].point_m[1] += 0.002;
    input.points[2].velocity_m_s[1] = 0.0;
    let no_slip = input.clone();
    input.event = Some(event("трасса", 1, ConfirmedEvent::Slip { payout_m: 0.04 }));
    let slipped = advance(&mut branch, 1, InterfaceInput::Tension(input)).unwrap();
    assert_eq!(
        slipped.transition,
        Some(Transition::ConfirmedSlip { payout_m: 0.04 })
    );
    near(slipped.payout_m.unwrap(), 0.062);
    assert!(slipped.body_loads.is_empty());
    near(value(&slipped.work).released_energy_j, 0.09);
    let original = advance(&mut route, 1, InterfaceInput::Tension(no_slip.clone())).unwrap();
    let copied = advance(&mut copy, 1, InterfaceInput::Tension(no_slip)).unwrap();
    assert_eq!(original, copied);
    assert_eq!(
        original.payout_m.unwrap().to_bits(),
        copied.payout_m.unwrap().to_bits()
    );
    near(value(&original.reaction).tension_n, 6.0);

    let mut short = route_input(0.0);
    short.points[2].point_m[1] = 0.3775;
    short.points[2].velocity_m_s[1] = 0.5;
    let damped = advance(
        &mut TensionRoute::new(spec.clone()).unwrap(),
        0,
        InterfaceInput::Tension(short),
    )
    .unwrap();
    // До упругого предела ещё 0,0025 м. Вязкость удерживает ė=0,125 м/с,
    // остаток 0,375 м/с уходит в скольжение под силой 6 Н.
    near(damped.payout_m.unwrap(), 0.00375);
    near(value(&damped.work).loss_j, 0.023125);

    spec.friction = None;
    spec.capacity_n = exact(8.0);
    let mut weak = TensionRoute::new(spec).unwrap();
    let broken = advance(&mut weak, 0, InterfaceInput::Tension(route_input(0.0))).unwrap();
    assert_eq!(broken.state, InterfaceState::Broken);
    assert!(broken.body_loads.is_empty() && broken.drive_effort.is_none());
    let Some(Transition::Overload { force_n, limit_n }) = broken.transition else {
        panic!("нет обрыва")
    };
    near(force_n, 10.0);
    near(limit_n, 8.0);
    near(value(&broken.work).released_energy_j, 0.25);

    let mut malformed = route_input(0.0);
    malformed.event = Some(event("трасса", 1, ConfirmedEvent::Release));
    let mut fresh = TensionRoute::new(tension_spec()).unwrap();
    assert!(advance(&mut fresh, 0, InterfaceInput::Tension(malformed.clone())).is_err());
    malformed.event = Some(event(
        "трасса",
        0,
        ConfirmedEvent::Slip { payout_m: f64::NAN },
    ));
    assert!(advance(&mut fresh, 0, InterfaceInput::Tension(malformed.clone())).is_err());
    malformed.event = None;
    malformed.drive.as_mut().unwrap().velocity = f64::INFINITY;
    assert!(advance(&mut fresh, 0, InterfaceInput::Tension(malformed)).is_err());
    let mut degenerate = route_input(0.0);
    degenerate.points[2] = degenerate.points[1];
    assert!(advance(&mut fresh, 0, InterfaceInput::Tension(degenerate)).is_err());
    let mut invalid_spec = tension_spec();
    invalid_spec.length = FreeLengthLaw::Fixed(exact(-0.1));
    assert!(TensionRoute::new(invalid_spec).is_err());
    let mut outside = route_input(0.0);
    outside.points[2].point_m[1] = 1.0;
    let unavailable_step = advance(&mut fresh, 0, InterfaceInput::Tension(outside)).unwrap();
    assert!(unavailable(&unavailable_step.reaction));
    assert!(unavailable(
        &advance(&mut fresh, 1, InterfaceInput::Tension(route_input(0.0)))
            .unwrap()
            .reaction
    ));

    let mut extended_spec = tension_spec();
    extended_spec.capacity_n = exact(60.0);
    let mut extrapolated = TensionRoute::new(extended_spec).unwrap();
    let diagnostic = advance(
        &mut extrapolated,
        0,
        InterfaceInput::Tension(route_input(3.5)),
    )
    .unwrap();
    near(value(&diagnostic.reaction).tension_n, 45.0);
    assert!(unavailable(&diagnostic.calibration));
}

#[test]
fn route_merges_loads_at_a_common_center_and_keeps_local_forces() {
    let mut spec = tension_spec();
    spec.bodies[2] = "a".into();
    let mut input = route_input(0.0);
    input.points[2].com_m = input.points[0].com_m;
    let step = advance(
        &mut TensionRoute::new(spec.clone()).unwrap(),
        0,
        InterfaceInput::Tension(input.clone()),
    )
    .unwrap();
    assert_eq!(step.body_loads.len(), 2);
    assert_eq!(step.route_forces_n.len(), 3);
    xyz(step.body_loads[0].force_n, [10.0, -10.0, 0.0]);
    xyz(step.body_loads[0].torque_nm, [0.2, 0.2, -3.0]);
    input.points[2].com_m[1] = 0.1;
    assert!(
        advance(
            &mut TensionRoute::new(spec).unwrap(),
            0,
            InterfaceInput::Tension(input)
        )
        .is_err()
    );
}

#[test]
fn nonlinear_curve_returns_to_traction_limit_and_releases_preload() {
    let mut spec = tension_spec();
    spec.length = FreeLengthLaw::Fixed(exact(0.65));
    spec.calibration.extension_m = band(0.0, 0.05, 0.1);
    spec.extension = ForceCurve::new(
        [(0.0, 0.0), (0.025, 5.0), (0.05, 15.0), (0.1, 40.0)]
            .map(|(x, f)| CurvePoint {
                displacement_m: exact(x),
                force_n: exact(f),
            })
            .into(),
    )
    .unwrap();
    // Предел 8 Н расположен на втором отрезке. x = 25 + 25·3/10 мм,
    // энергия по трапециям 5·0,025/2 + (5+8)·0,0075/2 Дж.
    spec.friction = Some(TractionFriction {
        coefficient: exact(0.4),
        clamp_normal_n: exact(20.0),
    });
    let mut route = TensionRoute::new(spec).unwrap();
    let mut input = route_input(0.0);
    input.drive = None;
    let step = advance(&mut route, 0, InterfaceInput::Tension(input.clone())).unwrap();
    near(value(&step.reaction).tension_n, 8.0);
    near(step.payout_m.unwrap(), 0.0175);
    near(value(&step.work).stored_energy_j, 0.11125);
    near(value(&step.work).numerical_loss_j, 0.20125);
    assert!(step.drive_effort.is_none());
    input.event = Some(event("другая трасса", 1, ConfirmedEvent::Break));
    assert!(advance(&mut route, 1, InterfaceInput::Tension(input.clone())).is_err());
    input.event = Some(event("трасса", 1, ConfirmedEvent::Break));
    let broken = advance(&mut route, 1, InterfaceInput::Tension(input)).unwrap();
    assert_eq!(broken.transition, Some(Transition::ConfirmedBreak));
    assert_eq!(broken.state, InterfaceState::Broken);
    assert!(broken.body_loads.is_empty());
    near(value(&broken.work).released_energy_j, 0.11125);
}

fn world(rate: Rate, scene: &str) -> CompiledWorld {
    let dt = rate.as_hz().recip();
    CompiledWorld {
        name: "независимый стенд".into(),
        seed: 7,
        physics: rate,
        mjcf: format!(
            r#"<mujoco><option gravity="0 0 0" timestep="{dt}" integrator="Euler"/>{scene}</mujoco>"#
        ),
        bodies: vec![],
        joints: vec![],
        efforts: vec![],
        drives: vec![],
        sensors: vec![],
        simplifications: vec![],
    }
}

fn oscillator(x0: f64, mass: f64, k: f64, c: f64, t: f64) -> f64 {
    // m*x''+c*x'+k*x=0, x(0)=x0, x'(0)=0. Подкритический случай.
    let decay = c / (2.0 * mass);
    let omega = (k / mass - decay * decay).sqrt();
    x0 * (-decay * t).exp() * ((omega * t).cos() + decay / omega * (omega * t).sin())
}

fn pad_hold(hz: u64) -> [f64; 6] {
    // Блок 2 кг на двух идеальных осях. Начальные 10 мм преднатяга.
    let rate = Rate::hz(hz).unwrap();
    let world = world(
        rate,
        r#"<worldbody>
      <body name="a"/>
      <body name="b" pos="0 0 0.03">
        <joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 0 1"/>
        <inertial pos="0 0 0" mass="2" diaginertia="1 1 1"/>
      </body></worldbody>"#,
    );
    let mut sim = world.open_contact_sim().unwrap();
    let mut pad = CompliantPad::new(pad_spec()).unwrap();
    let mut rng = Rng::stream(7, "каретка");
    let mut impulse = [0.0; 2];
    let mut loss = 0.0;
    let mut peak_n: f64 = 0.0;
    let mut normal_impulse = 0.0;
    let mut held_work = 0.0;
    let mut displacement_work = 0.0;
    let dt = rate.as_hz().recip();
    // Выдержка 0,4 с: постоянная касательная сила 2 Н ниже предела
    // 5 Н. Для нормали 12 Н дают равновесие при сжатии 12 мм.
    for i in 0..(hz * 2 / 5) {
        let at = rate.tick(i);
        let input = InterfaceInput::Pad(pad_input(
            at,
            0.03 + sim.qpos()[1],
            sim.qpos()[0],
            sim.qvel()[0],
            sim.qvel()[1],
        ));
        let step = pad
            .advance(
                Interval {
                    start: at,
                    end: rate.tick(i + 1),
                },
                &input,
                &mut rng,
            )
            .unwrap();
        assert_eq!(step.state, InterfaceState::Loaded);
        value(&step.calibration);
        let contact = value(&step.reaction);
        let f = contact.loads[1].force_n;
        let net = [2.0 + f[0], -12.0 + f[2]];
        let [x0, z0]: [f64; 2] = sim.qpos().try_into().unwrap();
        sim.qfrc_applied_mut().copy_from_slice(&net);
        sim.step();
        for j in 0..2 {
            impulse[j] += net[j] * dt;
        }
        let work = value(&step.work);
        assert!(work.loss_j >= 0.0 && work.numerical_loss_j >= 0.0);
        loss += work.loss_j + work.numerical_loss_j;
        peak_n = peak_n.max(contact.normal_n);
        normal_impulse += contact.normal_n * dt;
        held_work += work.body_work_j;
        displacement_work += f[0] * (sim.qpos()[0] - x0) + f[2] * (sim.qpos()[1] - z0);
    }
    for (v, j) in sim.qvel().iter().zip(impulse) {
        near(2.0 * v, j);
    }
    let [x, z]: [f64; 2] = sim.qpos().try_into().unwrap();
    let kinetic = sim.qvel().iter().map(|v| v * v).sum::<f64>();
    let elastic = 500.0 * (0.01 - z).powi(2) + 50.0 * x * x;
    let external = 2.0 * x - 12.0 * z;
    let residual = kinetic + elastic - 0.05 - external + loss;
    [
        x,
        z,
        residual,
        peak_n,
        normal_impulse,
        held_work - displacement_work,
    ]
}

#[test]
fn pad_block_impulse_energy_and_drift_converge_across_three_grids() {
    let samples = [500, 1000, 2000].map(pad_hold);
    let reference_x = 0.02 - oscillator(0.02, 2.0, 100.0, 0.0, 0.4);
    let reference_z = -0.002 - oscillator(-0.002, 2.0, 1000.0, 20.0, 0.4);
    let x_errors = samples.map(|s| (s[0] - reference_x).abs());
    let z_errors = samples.map(|s| (s[1] - reference_z).abs());
    let energy = samples.map(|s| s[2].abs());
    let work = samples.map(|s| s[5].abs());
    for errors in [x_errors, z_errors, energy, work] {
        assert!(errors[1] < errors[0] && errors[2] < errors[1], "{errors:?}");
    }
    // Пределы для стенда: 1% от 20 мм статического сдвига,
    // 0,5% от 2 мм шага равновесия и 2% от начальных 0,05 Дж.
    assert!(x_errors[2] < 0.0002 && z_errors[2] < 0.00001 && energy[2] < 0.001);
    // N(t)=12+exp(−5t)(−2*cos(ωt)+10/ω*sin(ωt)),
    // где ω² = 1000/2 − 5². Пик определяется первым нулём N'.
    let omega = 475.0_f64.sqrt();
    let peak_time = (std::f64::consts::PI - (20.0 / (2.0 * omega - 50.0 / omega)).atan()) / omega;
    let peak_ref = 12.0
        + (-5.0 * peak_time).exp()
            * (-2.0 * (omega * peak_time).cos() + (10.0 / omega) * (omega * peak_time).sin());
    let impulse_ref = 12.0 * 0.4 - 2.0 * (-5.0_f64 * 0.4).exp() * (omega * 0.4).sin() / omega;
    // Сетки сравниваются раздельно в Н и Н·с. Допуски стенда
    // равны 0,1% от удерживаемых 12 Н и от 4,8 Н·с соответственно.
    for (index, reference, tolerance) in [(3, peak_ref, 0.012), (4, impulse_ref, 0.0048)] {
        let values = samples.map(|s| s[index]);
        close(values[2], reference, tolerance);
        close(values[0], values[1], tolerance);
        close(values[1], values[2], tolerance);
        assert!((values[2] - values[1]).abs() < (values[1] - values[0]).abs());
    }
    eprintln!(
        "Площадка, [сдвиг м, ход м, невязка Дж, пик Н, импульс Н·с, ошибка работы Дж]: {samples:?}; аналитика {reference_x}, {reference_z}, {peak_ref}, {impulse_ref}"
    );
}

fn tension_hold(hz: u64) -> [f64; 4] {
    let rate = Rate::hz(hz).unwrap();
    let world = world(
        rate,
        r#"<worldbody>
       <body name="a"><joint type="slide" axis="1 0 0"/>
         <inertial pos="0 0 0" mass="2" diaginertia="1 1 1"/>
       </body>
       <body name="b" pos="0.7 0 0"><joint type="slide" axis="1 0 0"/>
         <inertial pos="0 0 0" mass="2" diaginertia="1 1 1"/>
       </body>
       <body name="c"><joint type="hinge" axis="0 0 1"/>
         <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
       </body>
    </worldbody>"#,
    );
    let mut sim = world.open_contact_sim().unwrap();
    let mut spec = tension_spec();
    spec.bodies.pop();
    let mut model = TensionRoute::new(spec).unwrap();
    let dt = rate.as_hz().recip();
    let mut rng = Rng::stream(7, "два блока");
    let mut impulse = 0.0;
    let mut loss = 0.0;
    // 80 мс, до первого провиса аналитической траектории.
    for i in 0..(hz * 2 / 25) {
        let [a, b, q]: [f64; 3] = sim.qpos().try_into().unwrap();
        let [va, vb, vq]: [f64; 3] = sim.qvel().try_into().unwrap();
        let input = InterfaceInput::Tension(TensionInput {
            points: [(a, va), (0.7 + b, vb)]
                .map(|(x, v)| RoutePoint {
                    point_m: [x, 0.0, 0.0],
                    com_m: [x, 0.0, 0.0],
                    velocity_m_s: [v, 0.0, 0.0],
                })
                .into(),
            drive: Some(DriveMotion {
                coordinate: q,
                velocity: vq,
            }),
            event: None,
        });
        let step = model
            .advance(
                Interval {
                    start: rate.tick(i),
                    end: rate.tick(i + 1),
                },
                &input,
                &mut rng,
            )
            .unwrap();
        assert_eq!(step.state, InterfaceState::Loaded);
        value(&step.calibration);
        let force = value(&step.reaction).tension_n;
        let work = value(&step.work);
        assert!(work.loss_j >= 0.0);
        loss += work.loss_j;
        impulse += force * dt;
        sim.qfrc_applied_mut().copy_from_slice(&[
            step.body_loads[0].force_n[0],
            step.body_loads[1].force_n[0],
            step.drive_effort.unwrap().1,
        ]);
        sim.step();
    }
    let [va, vb, vq]: [f64; 3] = sim.qvel().try_into().unwrap();
    near(2.0 * va, impulse);
    near(2.0 * vb, -impulse);
    near(vq, -0.05 * impulse);
    let [a, b, q]: [f64; 3] = sim.qpos().try_into().unwrap();
    let extension = 0.05 + b - a + 0.05 * q;
    let mechanical = va * va + vb * vb + 0.5 * vq * vq + 100.0 * extension * extension;
    [extension, mechanical - 0.25 + loss, impulse, q]
}

#[test]
fn coupled_blocks_and_drive_close_impulse_and_refine_energy() {
    let samples = [500, 1000, 2000].map(tension_hold);
    // ∂e/∂[x_A,x_B,q]=[-1,1,0,05]. Обратная приведённая масса 1,0025 кг⁻¹.
    let reference = oscillator(0.05, 1.0 / 1.0025, 200.0, 4.0, 0.08);
    let errors = samples.map(|s| (s[0] - reference).abs());
    let residuals = samples.map(|s| s[1].abs());
    for seq in [errors, residuals] {
        assert!(seq[2] < seq[1] && seq[1] < seq[0], "{seq:?}");
    }
    // Для стенда 1% от 50 мм преднатяга и 1% от 0,25 Дж исходного запаса.
    assert!(errors[2] < 0.0005 && residuals[2] < 0.0025);
    eprintln!(
        "Трасса, [растяжение м, невязка Дж, импульс Н·с, угол рад]: {samples:?}; аналитика {reference}"
    );
}

#[test]
fn engine_owned_pad_reads_live_support_impulse_without_applying_it_again() {
    // Вес блока 2 кг в поле 9,81 м/с² равен 19,62 Н.
    let rate = Rate::hz(1000).unwrap();
    let mut support = world(
        rate,
        r#"<worldbody>
       <body name="a"><geom name="ga" type="plane" size="1 1 0.1"/></body>
       <body name="b" pos="0 0 0.1"><freejoint/>
         <geom name="gb" type="box" size="0.1 0.1 0.1" mass="2"/>
       </body></worldbody>"#,
    );
    support.mjcf = support
        .mjcf
        .replace("gravity=\"0 0 0\"", "gravity=\"0 0 -9.81\"");
    let mut sim = support.open_contact_sim().unwrap();
    for _ in 0..1000 {
        sim.step();
    } // Выдержка 1 с до установившегося равновесия.
    let v0 = sim.qvel()[2];
    let raw = sim.step_with_contacts().unwrap();
    let prior = Interval {
        start: rate.tick(1000),
        end: rate.tick(1001),
    };
    let frame = ContactFrame::from_engine_step(prior, &raw).unwrap();
    let mut spec = pad_spec();
    spec.owner = PadReactionOwner::Engine;
    let mut model = CompliantPad::new(spec).unwrap();
    let mut input = pad_input(prior.end, sim.qpos()[2] - 0.1, 0.0, 0.0, sim.qvel()[2]);
    input.completed = Some(frame);
    let out = model
        .advance(
            Interval {
                start: prior.end,
                end: rate.tick(1002),
            },
            &InterfaceInput::Pad(input),
            &mut Rng::stream(7, "блок на плоскости"),
        )
        .unwrap();
    let normal = value(&out.reaction).normal_n;
    // Учёт импульса по независимой скорости движка. Допуск 0,01 Н для
    // мягкого решателя и остаточного перехода после выдержки.
    close(normal, 19.62, 0.01);
    // Допуск 10⁻⁷ Н на остаток численного решателя.
    close(normal, 2.0 * (9.81 + (sim.qvel()[2] - v0) * 1000.0), 1e-7);
    assert!(out.body_loads.is_empty());
    eprintln!("Движок: {normal:.9} Н против 19,62 Н веса");
}
