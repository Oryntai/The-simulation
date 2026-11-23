mod support;

use sim_runtime::{Log, Runtime};
use sim_signal::Time;

use sim_task::*;
use support::*;

// Допуск округления f64 для 12 шагов без контактов, позиция порядка 10⁻⁴ м.
const ROUND_OFF: f64 = 1e-14;
// Верхний порог стенда 1 мм и срок 2 такта. При отдельной проверке порога он меняется.
const SAFE_TRAVEL: f64 = 0.001;
const DEADLINE: u64 = 2 * TICK;

#[test]
fn full_matrix_and_independent_motion_agree() {
    for radio in [false, true] {
        let assessor = assessor(SAFE_TRAVEL, DEADLINE, radio);
        let cases = assessor.required_cases();
        assert_eq!(cases.len(), if radio { 9 } else { 7 });
        let results: Vec<_> = cases
            .iter()
            .map(|case| execute(&assessor, case, Trial::Normal))
            .collect();
        let report = assessor.assess(&results);
        for row in report.cases() {
            assert_eq!(row.outcome, Outcome::Pass, "{row:#?}");
            assert_eq!(row.reaction.as_ref().unwrap().reaction_ns, TICK);
        }
        assert_eq!(report.outcome(), Outcome::Pass);
        assert_eq!(report.outcome_for(&assessor, PATH), Outcome::Pass);
        let row = &report.cases()[0];
        let metric = &row.metrics[0];
        let manual = reference_nominal();
        assert_eq!(metric.min, 0.0);
        assert_eq!(metric.samples, manual.len());
        assert!((metric.max - manual.last().unwrap()).abs() < ROUND_OFF);
        assert_eq!(row.reaction.as_ref().unwrap().change, manual[5]);
        let again = execute(&assessor, &cases[0], Trial::Normal);
        assert_eq!(row.journal, again.report().journal);
        assert_eq!(row.events, again.report().events);
    }
}

#[test]
fn an_unrun_or_unconfigured_case_stays_in_the_matrix() {
    let assessor = assessor(SAFE_TRAVEL, DEADLINE, false);
    let nominal = execute(&assessor, &assessor.required_cases()[0], Trial::Normal);
    assert_eq!(nominal.report().outcome, Outcome::Pass);
    let partial = assessor.assess(&[nominal]);
    assert_eq!(partial.cases().len(), 7);
    assert_eq!(partial.outcome(), Outcome::NotAnalyzed);
    assert_eq!(partial.cases()[1].outcome, Outcome::NotAnalyzed);
    let mut additional_mode = topology(false);
    additional_mode.required_modes.push("непрогнанный".into());
    let text = document(SAFE_TRAVEL, DEADLINE, false);
    let expected = Assessor::new(
        Task::parse(&text).unwrap(),
        additional_mode,
        world(PART),
        inputs(),
    )
    .unwrap();
    assert_eq!(expected.required_cases().len(), 14);
    assert!(
        expected
            .required_cases()
            .iter()
            .any(|case| case.mode == "непрогнанный" && expected.plan(case).is_none())
    );

    let omitted = text.replace("at_ns=8000000\nsource=", "at_ns=7000000\nsource=")
        .replace("[[modes.faults]]\nfault={node='датчик',kind='sensor_unavailable'}\nat_ns=7000000\n",
                 "[[modes.faults]]\nfault={node='другой датчик',kind='sensor_unavailable'}\nat_ns=7000000\n");
    let required = Assessor::new(
        Task::parse(&omitted).unwrap(),
        topology(false),
        world(PART),
        inputs(),
    )
    .unwrap();
    let case = required
        .required_cases()
        .iter()
        .find(|case| {
            case.fault
                .as_ref()
                .is_some_and(|f| f.kind == FaultKind::SensorUnavailable)
        })
        .unwrap();
    assert!(required.plan(case).is_none());
    assert_eq!(
        execute(&required, case, Trial::Normal).report().outcome,
        Outcome::NotAnalyzed
    );

    let empty = Assessor::new(
        Task::parse("version=1\ncontrol='control.toml'\nmodes=[]\ncriteria=[]").unwrap(),
        Topology {
            required_modes: vec![],
            paths: vec![],
            limitation_paths: Default::default(),
        },
        world(PART),
        inputs(),
    )
    .unwrap();
    let empty_report = empty.assess(&[]);
    assert_eq!(empty_report.outcome(), Outcome::NotAnalyzed);
    assert_eq!(empty_report.uncovered_drives(), &[DRIVE]);
}

#[test]
fn positive_result_cannot_move_to_a_different_identity() {
    let original = assessor(SAFE_TRAVEL, DEADLINE, false);
    let results: Vec<_> = original
        .required_cases()
        .iter()
        .map(|case| execute(&original, case, Trial::Normal))
        .collect();
    let proof = original.assess(&results);
    assert_eq!(proof.outcome(), Outcome::Pass);

    for modified in [
        Inputs {
            program: Fingerprint::of(b"other image"),
            ..inputs()
        },
        Inputs {
            wiring: Fingerprint::of(b"other connection"),
            ..inputs()
        },
        Inputs {
            catalog: Fingerprint::of(b"other catalog"),
            ..inputs()
        },
        Inputs {
            configuration: Fingerprint::of(b"other configuration"),
            ..inputs()
        },
        Inputs {
            build: "other build".into(),
            ..inputs()
        },
    ] {
        let expected = Assessor::new(
            Task::parse(&document(SAFE_TRAVEL, DEADLINE, false)).unwrap(),
            topology(false),
            world(PART),
            modified,
        )
        .unwrap();
        assert_eq!(expected.assess(&results).outcome(), Outcome::NotAnalyzed);
        assert_eq!(proof.outcome_for(&expected, PATH), Outcome::NotAnalyzed);
    }
    let mut seeded = world(PART);
    seeded.seed += 1;
    let expected = Assessor::new(
        Task::parse(&document(SAFE_TRAVEL, DEADLINE, false)).unwrap(),
        topology(false),
        seeded,
        inputs(),
    )
    .unwrap();
    assert_eq!(proof.outcome_for(&expected, PATH), Outcome::NotAnalyzed);

    let mut wrong_channel = topology(false);
    wrong_channel.paths[0].command.name = "стенд/чужая команда".into();
    assert!(
        Assessor::new(
            Task::parse(&document(SAFE_TRAVEL, DEADLINE, false)).unwrap(),
            wrong_channel,
            world(PART),
            inputs()
        )
        .is_err()
    );
    let other_threshold = assessor(SAFE_TRAVEL * 2.0, DEADLINE, false);
    assert_eq!(
        proof.outcome_for(&other_threshold, PATH),
        Outcome::NotAnalyzed
    );
    assert_eq!(
        proof.outcome_for(&original, "другой путь"),
        Outcome::NotAnalyzed
    );
}

#[test]
fn sealed_replay_empty_run_and_substituted_runtime_do_not_prove() {
    let assessor = assessor(SAFE_TRAVEL, DEADLINE, false);
    let case = &assessor.required_cases()[0];
    let empty = assessor.execute(case, |_, _, _| Ok(inputs())).unwrap();
    assert!(empty.report().completed);
    assert_eq!(empty.report().outcome, Outcome::NotAnalyzed);

    let mut prior = Runtime::load(world(PART)).unwrap();
    install(
        &mut prior,
        &assessor.plan(case).unwrap(),
        &EventSink::default(),
        Trial::Normal,
    )
    .unwrap();
    let mut bytes = Vec::new();
    // В двоичном журнале с корректной контрольной суммой есть начатый прогон.
    // Прямой запуск останавливается, когда пытается писать вне живого сеанса оценщика.
    prior.run_until(Time(HORIZON)).unwrap();
    assert!(prior.log().end().started);
    assert!(!prior.log().entries().is_empty());
    prior.log().write_binary(&mut bytes).unwrap();
    let recorded = Log::read_binary(bytes.as_slice()).unwrap();
    let attempt = assessor
        .execute(case, |runtime, plan, sink| {
            install(runtime, plan, sink, Trial::Normal)?;
            runtime.replay(recorded.clone())?;
            Ok(inputs())
        })
        .unwrap();
    assert_eq!(attempt.report().outcome, Outcome::NotAnalyzed);
    assert!(
        attempt
            .report()
            .runtime_error
            .as_deref()
            .unwrap()
            .contains("зафиксирована")
    );

    let replaced = assessor
        .execute(case, |runtime, plan, sink| {
            *runtime = Runtime::load(world(PART))?;
            install(runtime, plan, sink, Trial::Normal)?;
            Ok(inputs())
        })
        .unwrap();
    assert_eq!(replaced.report().outcome, Outcome::NotAnalyzed);
    assert!(
        replaced
            .report()
            .runtime_error
            .as_deref()
            .unwrap()
            .contains("другого экземпляра")
    );
}

#[test]
fn archive_uses_live_journal_and_its_failure_preserves_observed_violation() {
    let assessor = assessor(SAFE_TRAVEL, DEADLINE, false);
    let case = &assessor.required_cases()[0];
    let mut logs = Vec::new();
    let result = assessor
        .execute_recorded(
            case,
            |runtime, plan, sink| install(runtime, plan, sink, Trial::Normal).map(|()| inputs()),
            |kind, journal, events| {
                assert!(!events.is_empty());
                logs.push((kind, journal.fingerprint()));
                Ok(())
            },
        )
        .unwrap();
    assert_eq!(result.report().outcome, Outcome::Pass);
    assert_eq!(
        logs,
        vec![
            (RunKind::Live, result.report().journal.unwrap()),
            (
                RunKind::Comparison,
                result.report().comparison.as_ref().unwrap().journal
            ),
        ]
    );
    for (trial, expected) in [
        (Trial::Normal, Outcome::NotAnalyzed),
        (Trial::Forbidden, Outcome::Fail),
    ] {
        let result = assessor
            .execute_recorded(
                case,
                |runtime, plan, sink| install(runtime, plan, sink, trial).map(|()| inputs()),
                |_, _, _| Err("ошибка записи".into()),
            )
            .unwrap();
        assert_eq!(result.report().outcome, expected);
        assert!(
            result
                .report()
                .missing
                .iter()
                .any(|reason| reason.contains("журнал прогона не сохранён"))
        );
    }
}

#[test]
fn chain_requires_fresh_actual_measurement_command_application_and_feedback() {
    let assessor = assessor(SAFE_TRAVEL, DEADLINE, false);
    let case = &assessor.required_cases()[0];
    for trial in [
        Trial::Constant,
        Trial::NoCommandAcceptance,
        Trial::InvalidSample,
        Trial::WrongController,
        Trial::WrongFeedback,
        Trial::StaleCapture,
        Trial::External,
        Trial::Blocked,
        Trial::Incomplete,
    ] {
        let report = execute(&assessor, case, trial);
        assert_eq!(
            report.report().outcome,
            Outcome::NotAnalyzed,
            "{trial:?}: {:#?}",
            report.report()
        );
        assert!(!report.report().missing.is_empty());
        if trial == Trial::Constant {
            assert!(
                report.report().reaction.is_some(),
                "постоянный выход изменял физическое состояние"
            );
            assert!(
                report
                    .report()
                    .missing
                    .iter()
                    .any(|reason| reason.contains("не прослежена реакция"))
            );
        }
    }
}

#[test]
fn prerecorded_schedule_fails_the_comparison_even_if_it_moves_the_world() {
    let assessor = assessor(SAFE_TRAVEL, DEADLINE, false);
    let case = &assessor.required_cases()[0];
    for case in [case, &assessor.required_cases()[1]] {
        let row = execute(&assessor, case, Trial::Scripted);
        let row = row.report();
        assert_eq!(row.outcome, Outcome::NotAnalyzed);
        assert!(row.after_loop.is_some() && row.reaction.is_some());
        let comparison = row.comparison.as_ref().unwrap();
        assert_eq!(comparison.original_command, 2.0);
        assert_eq!(comparison.comparison_command, 2.0);
    }
    let partial = execute(&assessor, case, Trial::ScriptedUntilExcitation);
    let row = partial.report();
    assert!(!row.comparison_audit.as_ref().unwrap().command_stream_equal);
    assert_eq!(row.comparison.as_ref().unwrap().comparison_command, 2.0);
    assert_eq!(row.outcome, Outcome::NotAnalyzed);
}

#[test]
fn injection_requires_acknowledgement_and_observable_effect() {
    let assessor = assessor(SAFE_TRAVEL, DEADLINE, false);
    let case = assessor
        .required_cases()
        .iter()
        .find(|case| {
            case.fault
                .as_ref()
                .is_some_and(|f| f.kind == FaultKind::PowerCut)
        })
        .unwrap();
    for trial in [Trial::MissingAck, Trial::NoFaultEffect] {
        let result = execute(&assessor, case, trial);
        assert_eq!(result.report().outcome, Outcome::NotAnalyzed);
        assert!(result.report().reaction.is_none());
    }
}

#[test]
fn threshold_forbidden_event_and_late_measured_reaction_dominate_missing_data() {
    // 1 мкм ниже аналитического смещения 2 мкм уже на первом шаге.
    let strict = assessor(0.000_001, DEADLINE, false);
    let data = execute(&strict, &strict.required_cases()[0], Trial::Incomplete);
    assert_eq!(data.report().outcome, Outcome::Fail);
    assert!(!data.report().missing.is_empty());
    assert_eq!(strict.assess(&[data]).outcome(), Outcome::Fail);

    let normal = assessor(SAFE_TRAVEL, DEADLINE, false);
    let case = &normal.required_cases()[0];
    let seen = execute(&normal, case, Trial::Forbidden);
    assert_eq!(seen.report().outcome, Outcome::Fail);
    assert_eq!(seen.report().metrics[1].max, 1.0);
    assert!(
        seen.report()
            .violations
            .iter()
            .any(|reason| reason.contains("5000000 нс"))
    );
    let late = assessor(SAFE_TRAVEL, TICK - 1, false);
    let result = execute(&late, &late.required_cases()[0], Trial::Normal);
    assert_eq!(result.report().outcome, Outcome::Fail);
    assert_eq!(result.report().reaction.as_ref().unwrap().reaction_ns, TICK);
}

#[test]
fn network_fault_needs_traffic_and_delivery_is_measured() {
    let assessor = assessor(SAFE_TRAVEL, DEADLINE, true);
    let case = assessor
        .required_cases()
        .iter()
        .find(|case| {
            case.fault
                .as_ref()
                .is_some_and(|fault| fault.kind == FaultKind::PacketLoss)
        })
        .unwrap();
    let no_traffic = execute(&assessor, case, Trial::ClaimedRadioWithoutTraffic);
    assert_eq!(no_traffic.report().outcome, Outcome::NotAnalyzed);
    assert!(
        no_traffic
            .report()
            .missing
            .iter()
            .any(|reason| reason.contains("не наблюдалась потеря"))
    );
    let late = execute(&assessor, &assessor.required_cases()[0], Trial::LateRadio);
    assert_eq!(late.report().outcome, Outcome::Fail);
    assert!(
        late.report()
            .violations
            .iter()
            .any(|reason| reason.contains("2000000 нс"))
    );
}

#[test]
fn schema_and_nonfinite_inputs_have_explicit_diagnostics() {
    let text = document(SAFE_TRAVEL, DEADLINE, false);
    for (from, to) in [
        ("version=1", "version=2"),
        ("version=1", "version=1\nunknown=true"),
        ("kind='derived'", "kind='derived',typo=true"),
        ("value=1.0", "value=nan"),
        ("max=0.001", "max=inf"),
        ("min_change=1e-6", "min_change=-inf"),
        ("max_gap_ns=1000000", "max_gap_ns=0"),
        ("at_ns=8000000", "at_ns=9000000"),
        ("unit='1'", "unit='V'"),
    ] {
        assert!(text.contains(from), "{from}");
        assert!(
            Task::parse(&text.replace(from, to)).is_err(),
            "{from} → {to}"
        );
    }
    let mut bad_world = world(PART);
    bad_world.drives[0]
        .params
        .insert("масса".into(), f64::INFINITY);
    assert!(
        Assessor::new(
            Task::parse(&text).unwrap(),
            topology(false),
            bad_world,
            inputs()
        )
        .is_err()
    );

    let assessor = assessor(SAFE_TRAVEL, DEADLINE, false);
    let result = execute(&assessor, &assessor.required_cases()[0], Trial::NonFinite);
    assert_eq!(result.report().outcome, Outcome::NotAnalyzed);
    assert!(
        result
            .report()
            .runtime_error
            .as_deref()
            .unwrap()
            .contains("нечисловое")
    );
}

#[test]
fn no_safety_checks_and_estimated_thresholds_preclude_positive_evidence() {
    let text = document(SAFE_TRAVEL, DEADLINE, false);
    let empty = text
        .lines()
        .filter(|line| !line.starts_with("limits=") && !line.starts_with("{kind='forbidden'"))
        .collect::<Vec<_>>()
        .join("\n")
        .replace("response=", "limits=[]\nresponse=");
    for text in [empty, text.replace("kind='derived'", "kind='estimate'")] {
        let assessor = Assessor::new(
            Task::parse(&text).unwrap(),
            topology(false),
            world(PART),
            inputs(),
        )
        .unwrap();
        assert_eq!(
            execute(&assessor, &assessor.required_cases()[0], Trial::Normal)
                .report()
                .outcome,
            Outcome::NotAnalyzed
        );
    }
}
