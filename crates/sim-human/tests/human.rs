use sim_human::*;
use sim_verify::Verdict;

// Испытательные параметры формы. Нагрузки выбраны для проверки отказов, не как рекомендации.
fn scenario() -> HumanLoadCase {
    HumanLoadCase {
        payload_kg: 2.0,
        lever_m: 0.0,
        pose: BodyPose {
            elbow_flexion: 90_f64.to_radians(),
            ..BodyPose::default()
        },
        assist_nm: [0.0; NC],
        person: Person {
            sex: Sex::Female,
            strength_percentile: 5.0,
            age_years: 40.0,
            stature_m: 1.498,
            mass_kg: 49.8,
        },
        work_time_s: 0.0,
        lift: None,
        assist_loss_time_s: None,
        cycles_per_day: Some(1),
    }
}

fn get(report: &HumanReport, kind: HumanCheck) -> &HumanFinding {
    report
        .findings
        .iter()
        .find(|f| f.kind == kind)
        .expect("проверка присутствует")
}

#[test]
fn weak_person_overload_fails_on_measured_capacity() {
    let report = analyze_human(&HumanLoadCase {
        payload_kg: 20.0,
        ..scenario()
    });
    let held = report
        .holding
        .as_ref()
        .expect("статика фактически посчитана");
    let f = get(&report, HumanCheck::MuscleCapacity);
    assert_eq!(f.outcome, Outcome::Fail, "{f:?}");
    assert!(
        f.value > f.limit && f.limit > 0.0 && f.detail.contains("слабого человека"),
        "{f:?}"
    );
    assert!(held.max_activation > 0.0 && held.certified_deficit_nm > 0.0);
    assert!(
        held.muscles
            .iter()
            .all(|m| (0.0..=1.0).contains(&m.activation))
    );
}

#[test]
fn compression_over_niosh_has_own_failure() {
    let mut case = HumanLoadCase {
        payload_kg: 50.0,
        lever_m: 0.25,
        ..scenario()
    };
    case.pose.trunk_inclination = 35_f64.to_radians();
    case.pose.lumbar_flexion = 20_f64.to_radians();
    let report = analyze_human(&case);
    let held = report
        .holding
        .as_ref()
        .expect("статика фактически посчитана");
    let f = get(&report, HumanCheck::SpineCompression);
    assert_eq!(f.outcome, Outcome::Fail, "{f:?}");
    assert_eq!(f.limit, 3400.0); // NIOSH 81-122, округление силы в ньютонах.
    assert!(
        f.value > f.limit && held.chaffin_compression_n > f.limit && f.detail.contains("6400"),
        "{f:?}"
    );
}

#[test]
fn heavy_payload_fails_when_drive_loses_assistance_mid_lift() {
    let mut case = HumanLoadCase {
        payload_kg: 12.0,
        assist_loss_time_s: Some(0.03),
        ..scenario()
    };
    case.pose.shoulder_plane = 90_f64.to_radians();
    case.pose.shoulder_elevation = 30_f64.to_radians();
    case.lift = Some(LiftInput {
        start_pose: BodyPose {
            elbow_flexion: 60_f64.to_radians(),
            shoulder_elevation: 5_f64.to_radians(),
            ..case.pose
        },
        peak_speed_rad_s: 1.0,
    });
    case.assist_nm[C_ELBOW] = 33.0;
    case.assist_nm[C_ELEV] = 40.0;
    case.assist_nm[C_L5_FE] = 42.0;
    let report = analyze_human(&case);
    let loss = report
        .failure
        .as_ref()
        .unwrap_or_else(|| panic!("нет расчёта отказа: {:?}", report.findings));
    let motion = loss.motion.as_ref().expect("отказ проинтегрирован");
    let f = get(&report, HumanCheck::DriveFailure);
    assert_eq!(f.outcome, Outcome::Fail, "{f:?}");
    assert!(
        f.detail.contains("После полной потери помощи") && f.value > f.limit,
        "{f:?}"
    );
    assert!(
        motion.peak_speed_deg_s > 0.0 && motion.fell_before_reaction,
        "{motion:?}"
    );
    assert!(
        loss.unassisted_hold.required_nm[C_ELBOW]
            > loss.unassisted_hold.weak_strength.elbow_flexion.nm
    );
}

#[test]
fn long_hold_exceeds_endurance_below_maximum_torque() {
    let case = HumanLoadCase {
        work_time_s: 1800.0,
        ..scenario()
    };
    let report = analyze_human(&case);
    let f = get(&report, HumanCheck::Fatigue);
    assert_eq!(f.outcome, Outcome::Fail, "{f:?}");
    assert!(
        f.value > f.limit && f.limit > 0.0 && f.detail.contains("3CC"),
        "{f:?}"
    );
    assert!(
        report
            .endurance
            .iter()
            .any(|e| (0.1..1.0).contains(&e.intensity) && e.time_3cc_s.is_some())
    );
}

#[test]
fn accelerates_against_gravity_and_consumes_activation_reserve() {
    let mut case = scenario();
    case.lift = Some(LiftInput {
        start_pose: BodyPose {
            elbow_flexion: 30_f64.to_radians(),
            ..case.pose
        },
        peak_speed_rad_s: 4.0,
    });
    let report = analyze_human(&case);
    let lift = report.lift.as_ref().expect("обратная динамика выполнена");
    assert!(lift.samples > 2);
    assert!(
        lift.peak_required_nm[C_ELBOW] > lift.peak_static_nm[C_ELBOW],
        "{lift:?}"
    );
    assert!(lift.max_activation >= lift.max_static_activation);
    assert_eq!(get(&report, HumanCheck::DynamicLift).outcome, Outcome::Fail);
}

#[test]
fn missing_thresholds_and_failure_input_cannot_mark_safe() {
    let report = analyze_human(&scenario());
    assert!(report.holding.is_some(), "{:?}", report.findings);
    for check in [
        HumanCheck::ShoulderLoad,
        HumanCheck::ElbowLoad,
        HumanCheck::DriveFailure,
    ] {
        assert_eq!(get(&report, check).outcome, Outcome::NotAnalyzed);
    }
    assert_ne!(report.outcome(), Outcome::Pass);
    assert_eq!(assess_human(&scenario()), report.findings);
}

#[test]
fn non_finite_fields_or_overflow_fail_closed() {
    let mut cases = [
        HumanLoadCase {
            payload_kg: f64::NAN,
            ..scenario()
        },
        HumanLoadCase {
            assist_loss_time_s: Some(f64::INFINITY),
            ..scenario()
        },
        HumanLoadCase {
            payload_kg: f64::MAX,
            lever_m: f64::MAX,
            ..scenario()
        },
        HumanLoadCase {
            work_time_s: f64::INFINITY,
            ..scenario()
        },
        HumanLoadCase {
            person: Person {
                stature_m: f64::INFINITY,
                ..scenario().person
            },
            ..scenario()
        },
    ];
    cases[1].pose.shoulder_plane = f64::NEG_INFINITY;
    for case in &cases {
        let findings = assess_human(case);
        assert_eq!(findings.len(), HumanCheck::ALL.len());
        assert!(
            findings.iter().all(|f| f.outcome == Outcome::Fail),
            "{findings:?}"
        );
    }
}

#[test]
fn independent_checks_pass_and_measurement_mismatch_is_visible() {
    let checks = reference_checks();
    for check in &checks {
        println!("{check}");
    }
    for id in [
        "human_elbow_gravity",
        "human_lumbar_gravity",
        "human_elbow_acceleration",
        "human_lift_speed",
        "human_3cc_shoulder",
        "human_3cc_elbow",
        "human_3cc_spine",
    ] {
        checks
            .iter()
            .find(|c| c.id == id)
            .expect("независимая сверка присутствует")
            .assert_pass();
    }
    let known = known_discrepancies();
    assert_eq!(known.len(), 5, "расхождения не должны исчезать из отчёта молча");
    for id in [
        "human_verdict_endurance_shoulder",
        "human_verdict_endurance_elbow",
        "human_verdict_endurance_spine",
        "human_verdict_strength_weak",
        "human_verdict_strength_median",
    ] {
        checks
            .iter()
            .find(|c| c.id == id)
            .expect("защитная сверка присутствует")
            .assert_pass();
    }
    for id in ["human_mvc_men", "human_mvc_women"] {
        let mvc = known
            .iter()
            .find(|c| c.id == id)
            .expect("сверка силы присутствует");
        assert!(mvc.simulated.is_finite() && mvc.simulated > mvc.reference);
        assert_eq!(mvc.verdict(), Verdict::Fail);
    }
    let shoulder = known
        .iter()
        .find(|c| c.id == "human_shoulder_invivo")
        .expect("сверка плеча присутствует");
    assert!(
        shoulder.simulated.is_finite() && shoulder.simulated > 0.0,
        "{shoulder}"
    );
}

#[test]
fn release_reports_joint_reactions_on_balanced_motion_samples() {
    let mut case = HumanLoadCase {
        payload_kg: 0.0,
        assist_loss_time_s: Some(0.10),
        ..scenario()
    };
    case.pose.shoulder_elevation = 20_f64.to_radians();
    case.pose.elbow_flexion = 65_f64.to_radians();
    case.assist_nm[C_ELBOW] = 2.0;
    let report = analyze_human(&case);
    let loss = report
        .failure
        .as_ref()
        .expect("отказ действительно посчитан");
    assert!(loss.braking_samples > 0, "{loss:?}");
    let peaks = loss.braking_reactions.unwrap_or_else(|| {
        panic!(
            "нет равновесной реакции: {:?}",
            get(&report, HumanCheck::DriveFailure)
        )
    });
    assert!(peaks.elbow_n > 0.0 && peaks.shoulder_n > 0.0 && peaks.spine_compression_n > 0.0);
}
