use std::{collections::BTreeMap, path::Path, process::Command as Process};

use sim_reality::*;
use sim_signal::{Rate, Time};
use sim_world::{CompiledJoint, CompiledJointKind, CompiledSensor, CompiledWorld, EffortChannel};

fn channel(name: &str, unit: &str, role: Role, sampling: Sampling) -> Channel {
    Channel {
        name: name.into(),
        unit: unit.into(),
        role,
        sampling,
        allow_invalid: false,
    }
}

#[test]
fn import_checks_units_time_numbers_final_count_and_loss() {
    let cmd = channel("тяга", "N", Role::Command, Sampling::Event);
    let mut pos = channel(
        "сдвиг",
        "m",
        Role::Observation,
        Sampling::Periodic {
            first_ns: 0,
            period_ns: 10,
        },
    );
    pos.allow_invalid = true;
    let schema = [cmd, pos];
    let csv = concat!(
        "sim-reality,1,recorded\nbuild,блок,ревизия,20\ntime_ns,signal,unit,seq,value,valid\n",
        "0,тяга,N,0,4,1\n0,сдвиг,m,0,0,1\n10,сдвиг,m,1,,0\n20,сдвиг,m,2,2,1\nend,4\n"
    );
    let imported = Capture::parse(csv, &schema).unwrap();
    let id = imported.signals().id("сдвиг").unwrap();
    assert_eq!(imported.horizon(), Time(20));
    assert_eq!(imported.commands()[0].1.targets[0].1, 4.0);
    assert!(!imported.samples(id).unwrap()[1].valid);
    assert_eq!(imported.samples(id).unwrap()[2].value, 2.0);
    for (from, to) in [
        ("end,4", "end,3"),
        ("end,4\n", ""),
        (",N,", ",kg,"),
        (",4,1", ",NaN,1"),
        (",4,1", ",inf,1"),
        (",1,,0", ",1,NaN,0"),
        ("10,сдвиг", "9,сдвиг"),
        ("10,сдвиг", "20,сдвиг"),
        ("20,сдвиг", "0,сдвиг"),
        ("20,сдвиг", "21,сдвиг"),
        (",m,1,", ",m,2,"),
        ("20,сдвиг,m,2,2,1\nend,4", "end,3"),
        ("0,тяга", "1,тяга"),
        ("sim-reality,1,recorded", "sim-reality,2,recorded"),
    ] {
        assert!(
            Capture::parse(&csv.replacen(from, to, 1), &schema).is_err(),
            "{from} → {to}"
        );
    }
    let schema = [
        channel("тяга", "bool", Role::Command, Sampling::Event),
        schema[1].clone(),
    ];
    assert!(Capture::parse(&csv.replace(",N,", ",bool,"), &schema).is_err());
}

fn block() -> CompiledWorld {
    CompiledWorld {
        name: "блок".into(),
        seed: 17,
        physics: Rate::hz(100).unwrap(),
        // Вход 4 Н, масса 2 кг, шаг 10 мс, без тяжести и пассивных сил.
        mjcf: r#"<mujoco><option timestep="0.01" gravity="0 0 0"/><worldbody>
            <body name="блок"><joint name="ось" type="slide" axis="1 0 0"/>
              <geom type="sphere" size="0.1" mass="2"/>
            </body></worldbody></mujoco>"#
            .into(),
        joints: vec![CompiledJoint {
            name: "ось".into(),
            owner: "блок".into(),
            kind: CompiledJointKind::Slide,
        }],
        efforts: vec![EffortChannel {
            name: "вход".into(),
            joint: "ось".into(),
            dof: 0,
        }],
        bodies: Vec::new(),
        drives: Vec::new(),
        sensors: Vec::new(),
        simplifications: Vec::new(),
    }
}

#[test]
fn replay_matches_impulse_balance_and_exposes_missing_coverage() {
    let mut position = channel("позиция", "m", Role::Observation, Sampling::Event);
    position.allow_invalid = true;
    let schema = [
        channel("тяга", "N", Role::Command, Sampling::Event),
        position.clone(),
    ];
    // Euler: после второго такта x=0,0006 м, v=0,04 м/с. В 25 мс снимаем силу.
    // На третьем такте x=0,0010 м. В лог намеренно добавлены 0,0001 м и один пропуск.
    let csv = concat!(
        "sim-reality,1,synthetic\nbuild,блок,ревизия,40000000\ntime_ns,signal,unit,seq,value,valid\n",
        "0,тяга,N,0,4,1\n0,позиция,m,0,0,1\n25000000,тяга,N,1,0,1\n",
        "25000000,позиция,m,1,0.0008,1\n30000000,позиция,m,2,0.0011,1\n40000000,позиция,m,3,,0\nend,6\n"
    );
    let imported = Capture::parse(csv, &schema).unwrap();
    let binding = [InputBinding {
        recorded: "тяга".into(),
        simulated: "вход".into(),
    }];
    let mut curve = CurveBinding {
        model: "блок".into(),
        recorded: "позиция".into(),
        simulated: "qpos[0]".into(),
        source: CurveSource::Truth,
        alignment: Alignment::Linear {
            max_gap_ns: 10_000_000,
        },
        uncertainty: Some(0.00002), // Заданный бюджет неопределённости тестового отсчёта, 20 мкм.
    };
    let run = |curve: &CurveBinding| {
        replay(
            &imported,
            ReplayRuntime::load(block(), None).unwrap(),
            &binding,
            std::slice::from_ref(curve),
        )
        .unwrap()
    };
    let report = run(&curve);
    assert_eq!(report.physics_steps, 4);
    assert!(report.unmapped_commands.is_empty());
    let series = &report.curves[0];
    assert_eq!(series.state, "partial");
    assert!(series.points[3].recorded.is_none());
    assert!((series.points[1].simulated.unwrap() - 0.0008).abs() < 1e-15);
    assert!((series.points[3].simulated.unwrap() - 0.0014).abs() < 1e-15);
    let gap = series.gap.as_ref().unwrap();
    assert_eq!(gap.compared, 3);
    assert!((gap.bias + 0.0001 / 3.0).abs() < 1e-15);
    assert!((gap.rms - 0.0001 / 3_f64.sqrt()).abs() < 1e-15);
    assert!((gap.upper_max_abs_at_samples.unwrap() - 0.00012).abs() < 1e-15);
    assert_eq!(
        serde_json::to_vec(&report).unwrap(),
        serde_json::to_vec(&run(&curve)).unwrap()
    );
    curve.alignment = Alignment::Linear {
        max_gap_ns: 9_000_000,
    };
    assert!(run(&curve).curves[0].points[1].simulated.is_none());
    curve.alignment = Alignment::Hold {
        max_age_ns: 5_000_000,
    };
    assert!((run(&curve).curves[0].points[1].simulated.unwrap() - 0.0006).abs() < 1e-15);
    let wrong = [
        channel("тяга", "N*m", Role::Command, Sampling::Event),
        position,
    ];
    let bad = Capture::parse(&csv.replace(",N,", ",N*m,"), &wrong).unwrap();
    assert!(
        replay(
            &bad,
            ReplayRuntime::load(block(), None).unwrap(),
            &binding,
            &[curve]
        )
        .is_err()
    );
}

#[test]
fn calibration_updates_only_resolved_parameter_for_this_revision() {
    let schema = [
        channel("ушло", "1", Role::Observation, Sampling::Event),
        channel("пришло", "1", Role::Observation, Sampling::Event),
    ];
    // Фикстура записанного источника с 10 нс задержки. Реальных данных здесь нет.
    let csv = concat!(
        "sim-reality,1,recorded\nbuild,блок,ревизия,100\ntime_ns,signal,unit,seq,value,valid\n",
        "0,ушло,1,0,0,1\n10,пришло,1,0,0,1\n20,ушло,1,1,1,1\n30,пришло,1,1,1,1\nend,4\n"
    );
    let imported = Capture::parse(csv, &schema).unwrap();
    let build = imported.build();
    let receipt = Receipt {
        build: build.clone(),
        log_fingerprint: format!("{:016x}", imported.fingerprint()),
        capture_id: "фикстура".into(),
        calibration: "фикстура".into(),
        scope: "фикстура".into(),
    };
    let model = "датчик";
    let identified = identify(
        &imported,
        &Experiment {
            model: model.into(),
            input: ExperimentKind::Delay {
                sent: "ушло".into(),
                received: "пришло".into(),
                sigma_ns: 1.0,
                clock_offset_bound_ns: 0,
            },
            criteria: Criteria {
                max_condition: 2.0,
                max_reduced_chi_square: 2.0,
                max_half_width: BTreeMap::from([(Parameter::Delay, 0.000001)]),
            },
        },
    )
    .unwrap();
    let measured = MeasuredCapture::verify(&imported, build, &receipt).unwrap();
    let calibration = Calibration::from_identification(&identified, &measured).unwrap();
    let mut world = block();
    world.sensors.push(CompiledSensor {
        name: model.into(),
        part: "условный".into(),
        body: None,
        position_m: [0.0; 3],
        quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        joint: None,
        drive: None,
        params: BTreeMap::new(),
    });
    let before = world.clone();
    let other = BuildId {
        revision: "другая".into(),
        ..build.clone()
    };
    assert!(
        calibration
            .apply(&other, &mut world, Target::SensorDelay)
            .is_err()
    );
    assert!(
        calibration
            .apply(build, &mut world, Target::CameraOptics)
            .is_err()
    );
    assert_eq!(world, before);
    calibration
        .apply(build, &mut world, Target::SensorDelay)
        .unwrap();
    assert!((world.sensors[0].params["latency_s"] - 10e-9).abs() < 1e-22);
    assert!(world.simplifications[0].blocks_proof);
}

fn delay_profile() -> Profile {
    toml::from_str(
        r#"
        models = ["блок"]
        [build]
        design = "блок"
        revision = "ревизия"
        [[channels]]
        name = "ушло"
        unit = "1"
        role = "observation"
        sampling = { kind = "event" }
        [[channels]]
        name = "пришло"
        unit = "1"
        role = "observation"
        sampling = { kind = "event" }
        [[channels]]
        name = "положение"
        unit = "m"
        role = "observation"
        sampling = { kind = "periodic", first_ns = 0, period_ns = 100 }
        allow_invalid = true
        [[experiments]]
        model = "блок"
        input = { kind = "delay", sent = "ушло", received = "пришло", sigma_ns = 1, clock_offset_bound_ns = 0 }
        criteria = { max_condition = 2, max_reduced_chi_square = 2, max_half_width = { delay = 0.000001 } }
        "#,
    )
    .unwrap()
}

// Стендовая фикстура: три парных импульса с задержкой 10 нс и два положения.
const DELAY_LOG: &str = concat!(
    "sim-reality,1,recorded\nbuild,блок,ревизия,100\ntime_ns,signal,unit,seq,value,valid\n",
    "0,ушло,1,0,0,1\n0,положение,m,0,0,1\n10,пришло,1,0,0,1\n",
    "20,ушло,1,1,1,1\n30,пришло,1,1,1,1\n40,ушло,1,2,2,1\n50,пришло,1,2,2,1\n",
    "100,положение,m,1,0,1\nend,8\n"
);

fn fixture_receipt(build: &BuildId, data: &str) -> Receipt {
    Receipt {
        build: build.clone(),
        log_fingerprint: format!("{:016x}", sim_signal::fnv1a(data.bytes())),
        capture_id: "фикстура".into(),
        calibration: "фикстура".into(),
        scope: "фикстура".into(),
    }
}

#[test]
fn bad_or_incomplete_logs_cannot_be_measured() {
    let profile = delay_profile();
    let run = |data: &str| {
        analyze(
            &profile,
            Path::new("."),
            data,
            Some(&fixture_receipt(&profile.build, data)),
        )
    };
    let good = run(DELAY_LOG).unwrap();
    assert_eq!(good.calibrations.len(), 1);
    assert_eq!(good.safety_evidence, "not_analyzed");
    for (from, to) in [
        ("1,recorded", "1,synthetic"),
        ("блок,ревизия,", "блок,другая-ревизия,"),
        ("m,1,0,1", "m,1,NaN,1"),
        ("m,1,0,1", "m,1,inf,1"),
        ("m,1,0,1", "cm,1,0,1"),
        ("100,положение,m,1,0,1\nend,8", "end,7"),
        ("m,1,0,1", "m,1,,0"),
    ] {
        assert!(
            run(&DELAY_LOG.replacen(from, to, 1)).is_err(),
            "{from} → {to}"
        );
    }
    let incomplete = DELAY_LOG.replace("m,1,0,1", "m,1,,0");
    let diagnostic = analyze(&profile, Path::new("."), &incomplete, None).unwrap();
    assert_eq!(diagnostic.calibrations.len(), 0);
    assert_eq!(diagnostic.safety_evidence, "not_analyzed");
    assert!(matches!(
        diagnostic.identifications[0].estimates()[0].outcome,
        Outcome::Identified { .. }
    ));

    // Даже если неверную единицу внесли одновременно в CSV и профиль,
    // модель задержки требует безразмерные номера маркеров.
    let mut bad_schema = delay_profile();
    bad_schema.channels[0].unit = "s".into();
    let wrong_unit = DELAY_LOG.replace(",ушло,1,", ",ушло,s,");
    assert!(Capture::parse(&wrong_unit, &bad_schema.channels).is_ok());
    let receipt = fixture_receipt(&bad_schema.build, &wrong_unit);
    assert!(analyze(&bad_schema, Path::new("."), &wrong_unit, Some(&receipt)).is_err());
}

#[test]
fn unidentified_parameters_cannot_be_measured() {
    let mut profile = delay_profile();
    let ExperimentKind::Delay {
        clock_offset_bound_ns,
        ..
    } = &mut profile.experiments[0].input
    else {
        panic!("в опыте должны быть парные метки");
    };
    // Парный журнал полный, но граница ошибки часов 2 мкс шире допуска 1 мкс.
    *clock_offset_bound_ns = 2_000;
    let cap = Capture::parse(DELAY_LOG, &profile.channels).unwrap();
    let receipt = fixture_receipt(&profile.build, DELAY_LOG);
    let measured = MeasuredCapture::verify(&cap, &profile.build, &receipt).unwrap();
    let result = identify(&cap, &profile.experiments[0]).unwrap();
    assert!(matches!(
        result.estimates()[0].outcome,
        Outcome::NotAnalyzed { .. }
    ));
    assert!(Calibration::from_identification(&result, &measured).is_err());
    let report = analyze(&profile, Path::new("."), DELAY_LOG, Some(&receipt)).unwrap();
    assert!(report.calibrations.is_empty());
    assert_eq!(report.safety_evidence, "not_analyzed");
}

#[test]
fn lost_marker_in_one_experiment_blocks_provenance_for_the_capture() {
    let mut profile = delay_profile();
    profile.models.push("стержень".into());
    profile
        .channels
        .push(channel("пришло-б", "1", Role::Observation, Sampling::Event));
    let mut second = delay_profile().experiments.remove(0);
    second.model = "стержень".into();
    let ExperimentKind::Delay { received, .. } = &mut second.input else {
        panic!("в опыте должны быть парные метки");
    };
    *received = "пришло-б".into();
    profile.experiments.push(second);
    // Другой тракт получает первые два маркера, третий теряется при передаче.
    let csv = DELAY_LOG
        .replace(
            "10,пришло,1,0,0,1\n",
            "10,пришло,1,0,0,1\n10,пришло-б,1,0,0,1\n",
        )
        .replace(
            "30,пришло,1,1,1,1\n",
            "30,пришло,1,1,1,1\n30,пришло-б,1,1,1,1\n",
        )
        .replace("end,8", "end,10");
    let diagnostic = analyze(&profile, Path::new("."), &csv, None).unwrap();
    assert!(matches!(
        diagnostic.identifications[0].estimates()[0].outcome,
        Outcome::Identified { .. }
    ));
    assert!(matches!(
        diagnostic.identifications[1].estimates()[0].outcome,
        Outcome::NotAnalyzed { .. }
    ));
    let receipt = fixture_receipt(&profile.build, &csv);
    assert!(analyze(&profile, Path::new("."), &csv, Some(&receipt)).is_err());
}

#[test]
fn owner_profiles_load_and_bridge_curves_run_headless() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../devices/stage7");
    let mut bridge: Profile =
        toml::from_str(include_str!("../../../devices/stage7/04-bridge.toml")).unwrap();
    let mut pulse: Profile =
        toml::from_str(include_str!("../../../devices/stage7/04-capture.toml")).unwrap();
    // Возбуждение теста — только нейтральная нулевая команда стенда, не снятые показания.
    bridge.build.revision = "проверка-схемы".into();
    pulse.build.revision = bridge.build.revision.clone();
    let csv = concat!(
        "sim-reality,1,synthetic\nbuild,стенд-04,проверка-схемы,3000000\ntime_ns,signal,unit,seq,value,valid\n",
        "0,скважность-моста,1,0,0,1\n0,питание-моста,bool,0,1,1\n",
        "2000000,угол-рычага,rad,0,0,1\n2000000,угол-клавиши,rad,0,0,1\n2000000,ток-обмотки,A,0,0,1\nend,5\n"
    );
    let live = analyze(&bridge, &root, csv, None).unwrap();
    assert_eq!(live.replay.as_ref().unwrap().physics_steps, 3000);
    assert!(live.models.values().all(|m| m.comparison == "compared"));
    assert_eq!(live.safety_evidence, "not_analyzed");
    assert_eq!(live.physical_check, "не проверено на реальном устройстве");
    assert!(live.calibrations.is_empty());
    let csv = concat!(
        "sim-reality,1,synthetic\nbuild,стенд-04,проверка-схемы,20000000\ntime_ns,signal,unit,seq,value,valid\n",
        "0,угол-рычага,rad,0,0,1\n0,угол-клавиши,rad,0,0,1\n0,ток-шины,A,0,0,1\n0,напряжение-шины,V,0,0,1\n",
        "20000000,период,s,0,0.02,1\n20000000,ширина,s,0,0.001,1\n20000000,номер-кадра,1,0,0,1\nend,7\n"
    );
    assert!(
        analyze(
            &pulse,
            &root,
            &csv.replace("20000000,период", "19999999,период"),
            None
        )
        .is_err()
    );
    let imported = analyze(&pulse, &root, csv, None).unwrap();
    assert!(imported.replay.is_none());
    assert!(
        imported
            .models
            .values()
            .all(|m| m.comparison == "not_analyzed")
    );
    assert_eq!(
        imported.physical_check,
        "не проверено на реальном устройстве"
    );
}

#[test]
fn data_only_world_runs_through_cli_and_each_model_tracks_its_data() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../devices/stage7/acceptance");
    let output = Process::new(env!("CARGO_BIN_EXE_sim-reality"))
        .arg("run")
        .args([root.join("profile.toml"), root.join("capture.csv")])
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let json: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(json["replay"]["physics_steps"], 4);
    assert_eq!(json["models"]["каретка"]["comparison"], "compared");
    assert_eq!(json["models"]["блок"]["comparison"], "compared");
    assert_eq!(json["models"]["отсчёт"]["comparison"], "not_analyzed");
    assert_eq!(json["models"]["стержень"]["comparison"], "not_analyzed");
    assert!(json["models"]["стержень"]["reason"].is_string());
    assert_eq!(json["safety_evidence"], "not_analyzed");
    assert_eq!(json["calibrations"], serde_json::json!([]));
    assert_eq!(
        json["identifications"][0]["estimates"][0]["parameter"],
        "delay"
    );
    // Среднее четырёх целочисленных разностей меток равно 174 мкс.
    let lag = json["identifications"][0]["estimates"][0]["outcome"]["value"]
        .as_f64()
        .unwrap();
    assert!((lag - 174e-6).abs() < 1e-15);
    let curves = json["replay"]["curves"].as_array().unwrap();
    assert_eq!(curves.len(), 2);
    // Независимый полудискретный шаг: v += F*h/m, x += v*h. Команды
    // меняются после второго такта. Записанный сдвиг после нуля +2 и −1 мкм.
    for (curve, (model, positions_um, bias_um)) in curves.iter().zip([
        ("каретка", [0.0, 3.0, 9.0, 15.0, 21.0], -2.0_f64),
        ("блок", [0.0, -2.0, -6.0, -8.0, -8.0], 1.0_f64),
    ]) {
        assert_eq!(curve["model"], model);
        assert_eq!(curve["gap"]["compared"], 5);
        assert!((curve["gap"]["bias"].as_f64().unwrap() - bias_um * 0.8e-6).abs() < 1e-15);
        assert!((curve["gap"]["max_abs"].as_f64().unwrap() - bias_um.abs() * 1e-6).abs() < 1e-15);
        for (point, um) in curve["points"].as_array().unwrap().iter().zip(positions_um) {
            assert!((point["simulated"].as_f64().unwrap() - um * 1e-6).abs() < 1e-15);
        }
    }

    let profile = toml::from_str(include_str!(
        "../../../devices/stage7/acceptance/profile.toml"
    ))
    .unwrap();
    let csv = include_str!("../../../devices/stage7/acceptance/capture.csv");
    let rerun = analyze(&profile, &root, csv, None).unwrap();
    assert_eq!(serde_json::to_vec_pretty(&rerun).unwrap(), output.stdout);
    let changed = csv
        .replace("0.000023,1", "0.000029,1")
        .replace("ход-б,m,4,-0.000009", "ход-б,m,4,-0.000013");
    let changed = analyze(&profile, &root, &changed, None).unwrap();
    let changed = changed.replay.unwrap();
    assert_eq!(changed.journal, rerun.replay.unwrap().journal);
    for (curve, expected_um) in changed.curves.iter().zip([8.0, 5.0]) {
        let gap = curve.gap.as_ref().unwrap();
        assert!((gap.max_abs - expected_um * 1e-6).abs() < 1e-15);
    }
}

#[test]
fn attested_delay_is_applied_after_an_unresolved_experiment() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../devices/stage7/acceptance");
    let settings = include_str!("../../../devices/stage7/acceptance/profile.toml");
    let mut profile: Profile = toml::from_str(settings).unwrap();
    let mut resolved = toml::from_str::<Profile>(settings)
        .unwrap()
        .experiments
        .remove(0);
    resolved.model = "отсчёт".into();
    let ExperimentKind::Delay {
        clock_offset_bound_ns,
        ..
    } = &mut profile.experiments[0].input
    else {
        panic!("в опыте должны быть парные метки");
    };
    // Ошибка синхронизации 50 мкс превышает общий допуск полуширины 1 мкс.
    *clock_offset_bound_ns = 50_000;
    profile.experiments.push(resolved);
    profile
        .simulation
        .as_mut()
        .unwrap()
        .curves
        .push(CurveBinding {
            model: "отсчёт".into(),
            recorded: "ход-а".into(),
            simulated: "отсчёт/value".into(),
            source: CurveSource::Observation,
            alignment: Alignment::Hold {
                max_age_ns: 1_000_000,
            }, // Период датчика из каталога стенда.
            uncertainty: None,
        });
    // Имитация удостоверенного источника для проверки установки. Физического журнала нет.
    let synthetic = include_str!("../../../devices/stage7/acceptance/capture.csv");
    let csv = synthetic.replacen("1,synthetic", "1,recorded", 1);
    let receipt = fixture_receipt(&profile.build, &csv);
    let run = || analyze(&profile, &root, &csv, Some(&receipt));
    let original = run().unwrap();
    assert_eq!(original.calibrations.len(), 1);
    assert!(matches!(
        original.identifications[0].estimates()[0].outcome,
        Outcome::NotAnalyzed { .. }
    ));

    profile
        .simulation
        .as_mut()
        .unwrap()
        .apply
        .push(Installation {
            model: "отсчёт".into(),
            target: Target::SensorDelay,
        });
    let calibrated = analyze(&profile, &root, &csv, Some(&receipt)).unwrap();
    assert!(analyze(&profile, &root, synthetic, None).is_err());
    assert_eq!(calibrated.calibrations.len(), 1);
    assert_eq!(calibrated.safety_evidence, "not_analyzed");
    assert_eq!(calibrated.physical_check, "сравнение по журналу калибровки");
    let before = original.replay.unwrap();
    let after = calibrated.replay.unwrap();
    assert!(
        after.limitations["отсчёт"]
            .iter()
            .any(|text| text.contains("Источник «замер»"))
    );
    // В 3 мс без задержки доступно 15 мкм. С задержкой 174 мкс ещё доступен
    // отсчёт с 2 мс, 9 мкм. Числа получены шагом v += Fh/m, x += vh.
    let point = |report: &ReplayReport| report.curves[2].points[3].simulated.unwrap();
    assert!((point(&before) - 15e-6).abs() < 1e-15);
    assert!((point(&after) - 9e-6).abs() < 1e-15);
}

#[test]
fn binary_outputs_json_for_csv_and_rejects_missing_inputs() {
    struct Temp(std::path::PathBuf);
    impl Drop for Temp {
        fn drop(&mut self) {
            std::fs::remove_dir_all(&self.0).unwrap();
        }
    }
    let path = std::env::temp_dir().join(format!("sim-reality-cli-{}", std::process::id()));
    std::fs::create_dir(&path).unwrap();
    let dir = Temp(path);
    let profile = dir.0.join("блок.toml");
    let log = dir.0.join("блок.csv");
    std::fs::write(
        &profile,
        r#"
        models = ["блок"]
        [build]
        design = "блок"
        revision = "фикстура"
        [[channels]]
        name = "положение"
        unit = "m"
        role = "observation"
        sampling = { kind = "event" }
    "#,
    )
    .unwrap();
    std::fs::write(
        &log,
        concat!(
            "sim-reality,1,synthetic\nbuild,блок,фикстура,1\ntime_ns,signal,unit,seq,value,valid\n",
            "0,положение,m,0,0,1\nend,1\n"
        ),
    )
    .unwrap();
    let binary = env!("CARGO_BIN_EXE_sim-reality");
    let output = Process::new(binary)
        .arg("run")
        .args([&profile, &log])
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let report: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(report["origin"], "synthetic");
    assert_eq!(report["models"]["блок"]["comparison"], "not_analyzed");
    assert_eq!(report["safety_evidence"], "not_analyzed");
    let failed = Process::new(binary)
        .arg("run")
        .args([&profile, &dir.0.join("нет.csv")])
        .output()
        .unwrap();
    assert!(!failed.status.success());
    assert!(failed.stdout.is_empty());
}
