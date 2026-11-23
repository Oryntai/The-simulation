//! Синтетические стенды. Числа задают входы с известным ручным решением.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::{PI, TAU};

use sim_env::contact::ContactClass;
use sim_materials::fasteners::{BoltGrade, BoltSize, FastenerSpec, ThreadEngagement};
use sim_process::contact::{ContactFrame, ContactSample};
use sim_process::work::electrical::*;
use sim_process::work::heat::*;
use sim_process::work::mechanics::*;
use sim_process::work::trace::*;
use sim_process::work::{
    HazardCheck, HazardKind, ProcessBinding, ProcessKind, ProcessMeasurement, WorkSubject,
    required_hazards,
};
use sim_process::{Analysis, Interval, SourcedRange, StepModel};
use sim_signal::{Rng, SignalSpec, Time};
use sim_world::Provenance;

fn measured() -> Provenance {
    Provenance::Measured("синтетический стенд с аналитическим эталоном".into())
}
fn r(x: f64) -> SourcedRange {
    band(x, x, x)
}
fn band(min: f64, nominal: f64, max: f64) -> SourcedRange {
    SourcedRange {
        min,
        nominal,
        max,
        provenance: measured(),
    }
}
fn point(p: [f64; 3]) -> [SourcedRange; 3] {
    p.map(r)
}
fn time(s: f64) -> Time {
    Time::from_secs(s).unwrap()
}
fn step(start: f64, end: f64) -> Interval {
    Interval {
        start: time(start),
        end: time(end),
    }
}
fn value<T: std::fmt::Debug>(analysis: Analysis<T>) -> T {
    match analysis {
        Analysis::Analyzed { value, .. } => value,
        missing => panic!("ожидался расчёт: {missing:?}"),
    }
}
fn is_unknown<T>(analysis: &Analysis<T>) -> bool {
    matches!(analysis, Analysis::NotAnalyzed { .. })
}
fn unresolved(check: &HazardCheck) -> bool {
    matches!(check, HazardCheck::NotAnalyzed { .. })
}
fn close(actual: f64, expected: f64) {
    // Зазор на округление двойной точности, включая решение линейной цепи.
    assert!(
        (actual - expected).abs() <= 2e-12 * expected.abs().max(1.0),
        "{actual} ≠ {expected}"
    );
}

fn sample(
    names: [&str; 2],
    point_m: [f64; 3],
    normal: [f64; 3],
    force_n: [f64; 3],
) -> ContactSample {
    ContactSample {
        geometries: names.map(str::to_string),
        point_m,
        normal,
        separation_m: 0.0,
        force_n,
        torque_nm: [0.0; 3],
    }
}

#[test]
fn trace_requires_force_and_does_not_bridge_gaps() {
    let pair = ["плоскость", "блок"];
    let mut trace = SurfaceTrace::new(TraceSpec {
        pair: pair.map(str::to_string),
        plane_point_m: point([1.0, 0.0, 0.0]),
        plane_normal: point([0.0, 0.0, 1.0]),
        plane_tolerance_m: r(0.01),
        reference_m: vec![point([1.0, 0.0, 0.0]), point([4.0, 0.0, 0.0])],
    })
    .unwrap();
    let mut rng = Rng::stream(7, "след");
    let mut take = |start, contacts| {
        let i = step(start, start + 1.0);
        trace
            .advance(
                i,
                &Some(ContactFrame {
                    at: i.end,
                    contacts,
                }),
                &mut rng,
            )
            .unwrap()
    };
    let mark = |x, y, force| sample(pair, [x, y, -0.001], [0.0, 0.0, 1.0], [0.0, 0.0, force]);
    assert_eq!(
        value(take(0.0, vec![mark(1.0, 0.0, 0.0)])),
        TraceStep::NoContact
    );
    let mut separated = mark(2.0, 0.0, 1.0);
    separated.separation_m = 0.001;
    assert_eq!(value(take(1.0, vec![separated])), TraceStep::NoContact);
    assert!(matches!(
        value(take(2.0, vec![mark(1.0, 0.0, 2.0)])),
        TraceStep::Mark {
            sampled_length_m: 0.0,
            ..
        }
    ));
    let result = value(take(3.0, vec![mark(1.3, 0.4, 2.0)]));
    let TraceStep::Mark {
        point_m,
        path_error_m,
        plane_error_m,
        sampled_length_m,
    } = result
    else {
        panic!("{result:?}")
    };
    assert_eq!(point_m, [1.3, 0.4, 0.0]);
    close(plane_error_m, 0.001);
    close(path_error_m, 0.4);
    // Прямоугольный треугольник 0,3–0,4–0,5 м.
    close(sampled_length_m, 0.5);
    assert!(is_unknown(&take(
        4.0,
        vec![mark(1.3, 0.4, 2.0), mark(1.4, 0.4, 2.0)]
    )));
    assert_eq!(value(take(5.0, vec![])), TraceStep::NoContact);
    let result = value(take(6.0, vec![mark(5.0, 0.0, 2.0)]));
    assert!(
        matches!(result, TraceStep::Mark { sampled_length_m, path_error_m: 1.0, .. } if (sampled_length_m - 0.5).abs() < 1e-12)
    );
    let interval = step(7.0, 8.0);
    assert!(
        trace
            .advance(
                interval,
                &Some(ContactFrame {
                    at: interval.start,
                    contacts: vec![]
                }),
                &mut rng
            )
            .is_err()
    );
    assert!(is_unknown(
        &trace.advance(interval, &None, &mut rng).unwrap()
    ));
}

fn heat_spec() -> HeatSpec {
    HeatSpec {
        source_capacity_j_k: r(10.0),
        target_capacity_j_k: r(30.0),
        source_initial_k: r(350.0),
        target_initial_k: r(300.0),
        contact_coefficient_w_m2_k: r(400.0),
        calibrated_area_m2: band(0.009, 0.01, 0.011),
    }
}

#[test]
fn heat_matches_closed_form_and_conserves_energy_across_steps() {
    let mut model = HeatTransfer::new(heat_spec()).unwrap();
    let mut rng = Rng::stream(7, "тепло");
    let touching = HeatInput {
        power_into_source_w: 100.0,
        contact: Some(ThermalContact::Touching {
            area_m2: Some(r(0.01)),
        }),
    };
    let initial = model.clone();
    let dt = 2.0;
    let result = value(model.advance(step(0.0, dt), &touching, &mut rng).unwrap());
    // Независимое решение через средневзвешенную температуру и разность T.
    let lambda = 4.0 * (1.0 / 10.0 + 1.0 / 30.0);
    let equilibrium_diff = (100.0 / 10.0) / lambda;
    let diff = equilibrium_diff + (50.0 - equilibrium_diff) * (-lambda * dt).exp();
    let mean = (350.0 * 10.0 + 300.0 * 30.0 + 100.0 * dt) / 40.0;
    close(result.source_k, mean + 0.75 * diff);
    close(result.target_k, mean - 0.25 * diff);
    close(
        10.0 * (result.source_k - 350.0) + 30.0 * (result.target_k - 300.0),
        200.0,
    );
    close(result.transferred_j, 30.0 * (mean - 0.25 * diff - 300.0));
    let mut split = initial.clone();
    split.advance(step(0.0, 1.0), &touching, &mut rng).unwrap();
    let split_out = value(split.advance(step(1.0, 2.0), &touching, &mut rng).unwrap());
    close(split_out.source_k, result.source_k);
    close(split_out.target_k, result.target_k);
    let mut separated = initial.clone();
    let dry = value(
        separated
            .advance(
                step(0.0, 2.0),
                &HeatInput {
                    contact: Some(ThermalContact::Separated),
                    ..touching.clone()
                },
                &mut rng,
            )
            .unwrap(),
    );
    assert_eq!(
        (dry.source_k, dry.target_k, dry.transferred_j),
        (370.0, 300.0, 0.0)
    );
    for area_m2 in [
        None,
        Some(r(0.02)),
        Some(SourcedRange {
            provenance: Provenance::Estimate("без замера".into()),
            ..r(0.01)
        }),
    ] {
        let mut branch = initial.clone();
        assert!(is_unknown(
            &branch
                .advance(
                    step(0.0, 1.0),
                    &HeatInput {
                        contact: Some(ThermalContact::Touching { area_m2 }),
                        ..touching.clone()
                    },
                    &mut rng
                )
                .unwrap()
        ));
        assert!(is_unknown(
            &branch.advance(step(1.0, 2.0), &touching, &mut rng).unwrap()
        ));
    }
    assert!(
        model
            .advance(
                step(2.0, 3.0),
                &HeatInput {
                    power_into_source_w: f64::NAN,
                    ..touching.clone()
                },
                &mut rng
            )
            .is_err()
    );
    assert!(model.advance(step(2.0, 3.0), &touching, &mut rng).is_ok());
}

#[test]
fn burn_uses_material_duration_and_tissue_scope() {
    let mut test = BurnExposure {
        surface_k: r(350.0),
        contact_time: time(10.0),
        area_m2: Some(r(0.01)),
        surface_class: Some((ContactClass::UncoatedMetal, measured())),
        dry_skin: Some(SkinApplicability {
            body_region: "предплечье".into(),
            validated_area_m2: band(0.005, 0.01, 0.02),
            body_area_fraction: r(0.01),
            dry_skin_source: measured(),
        }),
    };
    // Таблица EN 13202 в sim-env: 55 °C для металла и 70 °C для пластмасс при 10 с.
    let hot = assess_burn(&test).unwrap();
    close(value(hot.threshold_k), 328.15);
    assert!(matches!(hot.check, HazardCheck::Violation { .. }));
    test.surface_k = band(330.0, 333.0, 340.0);
    test.surface_class = Some((ContactClass::Plastics, measured()));
    let plastic = assess_burn(&test).unwrap();
    close(value(plastic.threshold_k), 343.15);
    assert!(matches!(plastic.check, HazardCheck::WithinLimit { .. }));
    test.surface_k = band(340.0, 343.0, 345.0);
    assert!(unresolved(&assess_burn(&test).unwrap().check));
    test.surface_k = SourcedRange {
        provenance: Provenance::Derived("непроверенное происхождение".into()),
        ..r(300.0)
    };
    assert!(unresolved(&assess_burn(&test).unwrap().check));
    test.contact_time = time(0.5);
    assert!(is_unknown(&assess_burn(&test).unwrap().threshold_k));
    test.contact_time = time(10.0);
    test.dry_skin = None;
    assert!(unresolved(&assess_burn(&test).unwrap().check));
    test.area_m2 = Some(r(f64::INFINITY));
    assert!(assess_burn(&test).is_err());
}

#[test]
fn impact_uses_reversed_pair_and_independent_momentum_balance() {
    let path = ImpactPath {
        input_pair: ["ударник".into(), "стержень".into()],
        output_pair: ["стержень".into(), "блок".into()],
        carrier_mass_kg: r(2.0),
        momentum_tolerance_ns: r(1e-9),
        path_source: measured(),
    };
    let i = step(0.0, 0.1);
    let frame = ContactFrame {
        at: i.end,
        contacts: vec![
            sample(
                ["стержень", "ударник"],
                [0.0; 3],
                [-1.0, 0.0, 0.0],
                [-30.0, 0.0, 0.0],
            ),
            sample(
                ["стержень", "блок"],
                [0.0; 3],
                [1.0, 0.0, 0.0],
                [10.0, 0.0, 0.0],
            ),
        ],
    };
    let input = ImpactInput {
        carrier_velocity_start_m_s: [0.0; 3],
        carrier_velocity_end_m_s: [0.75, 0.0, 0.0],
        other_impulse_ns: Some([-0.5, 0.0, 0.0]),
    };
    let result = value(path.evaluate(i, Some(&frame), &input).unwrap());
    close(result.input_impulse_ns[0], 3.0);
    close(result.output_impulse_ns[0], 1.0);
    close(result.reaction_on_carrier_ns[0], -1.0);
    close(result.carrier_delta_momentum_ns[0], 1.5);
    close(result.residual_ns[0], 0.0);
    assert!(is_unknown(
        &path
            .evaluate(
                i,
                Some(&frame),
                &ImpactInput {
                    carrier_velocity_end_m_s: [0.5, 0.0, 0.0],
                    ..input
                }
            )
            .unwrap()
    ));
    assert!(is_unknown(
        &path
            .evaluate(
                i,
                Some(&frame),
                &ImpactInput {
                    other_impulse_ns: None,
                    ..input
                }
            )
            .unwrap()
    ));
    assert!(is_unknown(&path.evaluate(i, None, &input).unwrap()));
    let mut unverified = path.clone();
    unverified.path_source = Provenance::Estimate("связь неизвестна".into());
    assert!(is_unknown(
        &unverified.evaluate(i, Some(&frame), &input).unwrap()
    ));
    let mut bad = frame.clone();
    bad.contacts[0].torque_nm[0] = f64::NAN;
    assert!(path.evaluate(i, Some(&bad), &input).is_err());
}

#[test]
fn screw_kinematics_and_strength_have_separate_failure_checks() {
    let spec = ThreadSpec {
        lead_m_per_turn: r(0.001),
        handedness: Handedness::Positive,
        efficiency: r(0.5),
        travel_tolerance_m: r(1e-8),
        tensile_strength: Some(ThreadStrength {
            fastener: FastenerSpec {
                size: BoltSize::M6,
                grade: BoltGrade::G8_8,
                count: 1,
                engagement: ThreadEngagement::Tapped { length: 0.004 },
                plate_thickness: 0.005,
                edge_distance: 0.015,
            },
            fastener_source: measured(),
            member_bearing_pa: r(40e6),
            member_shear_pa: r(10e6),
            thread_factor: r(1.0),
            member_model_source: Some(measured()),
        }),
    };
    let input = ThreadInput {
        engagement: Some(EngagementState::Engaged),
        angle_delta_rad: TAU,
        axial_delta_m: 0.001,
        drive_torque_nm: 0.1,
        measured_tension_n: Some(400.0),
    };
    let result = spec.evaluate(&input).unwrap();
    let motion = value(result.motion);
    close(motion.expected_travel_m, 0.001);
    close(motion.axial_force_n, 100.0 * PI);
    close(motion.input_work_j, 0.2 * PI);
    close(motion.axial_work_j, motion.axial_force_n * 0.001);
    close(motion.input_work_j, motion.axial_work_j + motion.loss_j);
    // Срез витков 0,875·π·6 мм·4 мм при 10 МПа даёт 659,73 Н.
    let strength_n = 210.0 * PI;
    let HazardCheck::WithinLimit { scope } = result.strength else {
        panic!("{:?}", result.strength)
    };
    assert!(scope.contains(&format!("{:.6}", strength_n / 400.0)));
    assert!(matches!(
        spec.evaluate(&ThreadInput {
            measured_tension_n: Some(720.0),
            ..input
        })
        .unwrap()
        .strength,
        HazardCheck::Violation { .. }
    ));
    let slipping = spec
        .evaluate(&ThreadInput {
            axial_delta_m: 0.0,
            ..input
        })
        .unwrap();
    assert!(is_unknown(&slipping.motion));
    assert!(matches!(slipping.alignment, HazardCheck::Violation { .. }));
    let stripped = spec
        .evaluate(&ThreadInput {
            engagement: Some(EngagementState::Stripped),
            ..input
        })
        .unwrap();
    assert!(is_unknown(&stripped.motion));
    assert!(matches!(stripped.alignment, HazardCheck::Violation { .. }));
    let reversed = ThreadSpec {
        handedness: Handedness::Negative,
        ..spec.clone()
    };
    close(
        value(
            reversed
                .evaluate(&ThreadInput {
                    axial_delta_m: -0.001,
                    ..input
                })
                .unwrap()
                .motion,
        )
        .axial_force_n,
        -100.0 * PI,
    );
    let mut ranged = spec.clone();
    ranged.tensile_strength.as_mut().unwrap().member_shear_pa = band(8e6, 10e6, 12e6);
    assert!(unresolved(
        &ranged
            .evaluate(&ThreadInput {
                measured_tension_n: Some(650.0),
                ..input
            })
            .unwrap()
            .strength
    ));
    let mut missing = spec.clone();
    missing
        .tensile_strength
        .as_mut()
        .unwrap()
        .member_model_source = None;
    assert!(unresolved(&missing.evaluate(&input).unwrap().strength));
    assert!(
        spec.evaluate(&ThreadInput {
            drive_torque_nm: f64::INFINITY,
            ..input
        })
        .is_err()
    );
}

fn electrical_spec(bounce: Option<BounceModel>) -> ElectricalSpec {
    ElectricalSpec {
        release_below_m: r(0.001),
        close_above_m: r(0.003),
        initially_closed: false,
        initial_state_source: measured(),
        bounce,
        contact_closed_ohm: r(1.0),
        contact_open_ohm: r(1000.0),
        source_resistance_ohm: r(1.0),
        load_ohm: r(4.0),
        calibrated_supply_v: band(-12.0, 12.0, 12.0),
    }
}

#[test]
fn electrical_uses_hysteresis_and_detects_open_and_stuck_contacts() {
    let mut model = ElectricalContact::new(electrical_spec(Some(BounceModel::VerifiedAbsent {
        source: measured(),
    })))
    .unwrap();
    let mut rng = Rng::stream(7, "контакт");
    let mut run = |start, travel, supply, fault| {
        model
            .advance(
                step(start, start + 0.5),
                &ElectricalInput {
                    travel_m: Some(travel),
                    supply_v: supply,
                    fault,
                },
                &mut rng,
            )
            .unwrap()
    };
    let open = run(0.0, 0.002, 12.0, ContactFault::None);
    assert_eq!(open.requested_closed, Some(false));
    let low = value(open.segments)[0];
    assert!(!low.closed);
    close(low.current_a, 12.0 / 1005.0);
    let closed = run(0.5, 0.004, 12.0, ContactFault::None);
    assert_eq!(closed.requested_closed, Some(true));
    let high = value(closed.segments.clone())[0];
    assert!(high.closed);
    // 12 В при 1 + 1 + 4 Ом дают 2 А и 8 В на нагрузке.
    close(high.current_a, 2.0);
    close(high.load_v, 8.0);
    close(high.contact_v, 2.0);
    close(high.load_j, 8.0);
    close(
        high.source_j,
        high.load_j + high.contact_j + high.source_loss_j,
    );
    let held = value(run(1.0, 0.002, 6.0, ContactFault::None).segments)[0];
    assert!(held.closed);
    close(held.current_a, 1.0);
    close(held.load_j, 2.0);
    let broken = run(1.5, 0.004, 12.0, ContactFault::OpenCircuit);
    assert_eq!(broken.requested_closed, Some(true));
    let broken = value(broken.segments)[0];
    assert!(!broken.closed);
    close(broken.load_v, low.load_v);
    let stuck = run(2.0, 0.0, -12.0, ContactFault::StuckClosed);
    assert_eq!(stuck.requested_closed, Some(false));
    let stuck = value(stuck.segments)[0];
    assert!(stuck.closed);
    close(stuck.current_a, -2.0);
    close(stuck.load_j, 8.0);
    let released = value(run(2.5, 0.0, 12.0, ContactFault::None).segments)[0];
    assert!(!released.closed);
    assert!(matches!(
        model.assess_overcurrent(&closed, Some(&r(1.9))).unwrap(),
        HazardCheck::Violation { .. }
    ));
    assert!(matches!(
        model.assess_overcurrent(&closed, Some(&r(2.1))).unwrap(),
        HazardCheck::WithinLimit { .. }
    ));
    assert!(unresolved(
        &model
            .assess_overcurrent(&closed, Some(&band(1.9, 2.0, 2.1)))
            .unwrap()
    ));
}

#[test]
fn electrical_slices_bounce_on_integer_time_and_preserves_missing_history() {
    let pulse = |duration, follows_target| BounceSegment {
        duration_s: r(duration),
        follows_target,
    };
    let config = electrical_spec(Some(BounceModel::Profile {
        closing: vec![pulse(0.001, false), pulse(0.001, true), pulse(0.002, false)],
        opening: vec![pulse(0.001, false)],
    }));
    let mut model = ElectricalContact::new(config).unwrap();
    let mut rng = Rng::stream(7, "дребезг");
    let input = ElectricalInput {
        travel_m: Some(0.004),
        supply_v: 12.0,
        fault: ContactFault::None,
    };
    let first = model.advance(step(0.0, 0.003), &input, &mut rng).unwrap();
    let first = value(first.segments);
    assert_eq!(
        first
            .iter()
            .map(|x| (x.interval.end, x.closed))
            .collect::<Vec<_>>(),
        [
            (time(0.001), false),
            (time(0.002), true),
            (time(0.003), false)
        ]
    );
    let mut branch = model.clone();
    let second = model.advance(step(0.003, 0.005), &input, &mut rng).unwrap();
    assert_eq!(
        second,
        branch
            .advance(step(0.003, 0.005), &input, &mut rng)
            .unwrap()
    );
    assert_eq!(value(second.segments)[0].interval.end, time(0.004));
    // 2 мс при 16 Вт и 3 мс с утечкой через 1005 Ом.
    close(
        value(second.total_load_j),
        0.032 + 0.003 * 4.0 * (12.0_f64 / 1005.0).powi(2),
    );
    let open = model
        .advance(
            step(0.005, 0.007),
            &ElectricalInput {
                travel_m: Some(0.0),
                ..input
            },
            &mut rng,
        )
        .unwrap();
    let open = value(open.segments);
    assert_eq!(
        open.iter()
            .map(|x| (x.interval.end, x.closed))
            .collect::<Vec<_>>(),
        [(time(0.006), true), (time(0.007), false)]
    );
    assert!(
        model
            .advance(
                step(0.007, 0.008),
                &ElectricalInput {
                    supply_v: f64::NAN,
                    ..input
                },
                &mut rng
            )
            .is_err()
    );
    let mut no_bounce = ElectricalContact::new(electrical_spec(None)).unwrap();
    assert!(is_unknown(
        &no_bounce
            .advance(step(0.0, 1.0), &input, &mut rng)
            .unwrap()
            .segments
    ));
    let fault = no_bounce
        .advance(
            step(1.0, 2.0),
            &ElectricalInput {
                fault: ContactFault::StuckClosed,
                ..input
            },
            &mut rng,
        )
        .unwrap();
    assert!(value(fault.segments)[0].closed);
    assert!(is_unknown(&fault.total_load_j));
    let mut invalid = electrical_spec(None);
    invalid.release_below_m = r(0.003);
    assert!(ElectricalContact::new(invalid).is_err());
}

#[test]
fn stored_energy_uses_signed_coordinate_extrema_and_source_quality() {
    let store = EnergyStore::Spring {
        stiffness_n_m: band(100.0, 200.0, 300.0),
        deflection_m: band(-0.2, -0.1, 0.1),
    };
    let result = assess_energy(Some(&store), Some(&r(3.0))).unwrap();
    let bounds = value(result.energy);
    // Интеграл силы kx: 0 Дж в нуле, 1 Дж в номинале и 6 Дж на худшей границе.
    close(bounds.min_j, 0.0);
    close(bounds.nominal_j, 1.0);
    close(bounds.max_j, 6.0);
    assert!(unresolved(&result.check));
    assert!(matches!(
        assess_energy(Some(&store), Some(&r(7.0))).unwrap().check,
        HazardCheck::WithinLimit { .. }
    ));
    let charged = EnergyStore::Capacitor {
        capacitance_f: r(2.0),
        voltage_v: r(3.0),
    };
    let result = assess_energy(Some(&charged), Some(&r(8.0))).unwrap();
    close(value(result.energy).nominal_j, 9.0);
    assert!(matches!(result.check, HazardCheck::Violation { .. }));
    assert!(unresolved(
        &assess_energy(
            Some(&charged),
            Some(&SourcedRange {
                provenance: Provenance::Estimate("предел без испытания".into()),
                ..r(10.0)
            })
        )
        .unwrap()
        .check
    ));
    assert!(is_unknown(&assess_energy(None, None).unwrap().energy));
    assert!(assess_energy(Some(&charged), Some(&r(f64::NAN))).is_err());
}

#[test]
fn missing_edge_does_not_remove_required_hazards_or_create_evidence() {
    let subjects = [WorkSubject {
        name: "стержень".into(),
        properties: BTreeSet::from([ProcessKind::SharpEdge, ProcessKind::StoredEnergy]),
    }];
    let binding = ProcessBinding {
        name: "след".into(),
        kind: ProcessKind::SurfaceMarking,
        attachment: Some("стержень".into()),
        ports: BTreeMap::from([("поверхность".into(), "блок".into())]),
    };
    let mandatory = required_hazards(&subjects, std::slice::from_ref(&binding)).unwrap();
    assert_eq!(mandatory.len(), 3);
    assert!(mandatory.iter().all(|h| is_unknown(&h.coverage)));
    let without = required_hazards(&subjects, &[]).unwrap();
    assert_eq!(
        without.iter().map(|h| h.kind).collect::<Vec<_>>(),
        [HazardKind::StoredEnergy, HazardKind::SharpEdge]
    );
    assert!(required_hazards(&[], &[binding]).is_err());
    let mut edge = EdgeInput {
        measured_radius_m: None,
        minimum_radius_m: Some(r(0.001)),
        contact_force_n: Some(r(10.0)),
        calibrated_application: Some(EdgeApplicability {
            contact_force_n: band(1.0, 10.0, 20.0),
            source: measured(),
        }),
        scope: "контакт под калиброванной нагрузкой".into(),
    };
    assert!(unresolved(&assess_edge(&edge).unwrap()));
    edge.measured_radius_m = Some(band(0.0002, 0.0004, 0.0008));
    assert!(matches!(
        assess_edge(&edge).unwrap(),
        HazardCheck::Violation { .. }
    ));
    edge.measured_radius_m = Some(band(0.0007, 0.001, 0.0015));
    assert!(unresolved(&assess_edge(&edge).unwrap()));
    edge.measured_radius_m = Some(band(0.0012, 0.0014, 0.0016));
    assert!(matches!(
        assess_edge(&edge).unwrap(),
        HazardCheck::WithinLimit { .. }
    ));
    edge.contact_force_n = Some(r(25.0));
    assert!(unresolved(&assess_edge(&edge).unwrap()));
    let signal = SignalSpec {
        name: "радиус".into(),
        unit: "м".into(),
    };
    assert!(
        ProcessMeasurement::new(
            time(1.0),
            signal.clone(),
            Analysis::Analyzed {
                value: f64::INFINITY,
                scope: edge.scope.clone()
            }
        )
        .is_err()
    );
    let sample = ProcessMeasurement::new(
        time(1.0),
        signal,
        Analysis::Analyzed {
            value: 0.0014,
            scope: edge.scope,
        },
    )
    .unwrap();
    assert_eq!(sample.at, time(1.0));
    close(value(sample.result), 0.0014);
}
