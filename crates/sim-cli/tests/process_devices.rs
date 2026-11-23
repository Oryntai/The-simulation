use std::{collections::BTreeMap, ffi::OsString, path::PathBuf};

use serde::Deserialize;
use serde_json::Value;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Manifest {
    version: u32,
    source: String,
    runs: Vec<Run>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Run {
    world: PathBuf,
    task: PathBuf,
    exit: u8,
    error: Option<String>,
    jobs: Option<usize>,
    #[serde(default)]
    cases: BTreeMap<String, Expected>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Expected {
    outcome: String,
    trials: usize,
    events: usize,
    satisfied: Option<bool>,
    violated: Option<String>,
    missing_check: Option<String>,
    rejected: Option<String>,
    metric: Option<String>,
    range: Option<[f64; 2]>,
    reference_samples: Option<u64>,
    reference_passed: Option<bool>,
    sensitivity: Option<Sensitivity>,
    nonconvergent_peak: Option<String>,
    wet_routes: Option<Vec<String>>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Sensitivity {
    against: String,
    column: String,
    minimum: f64,
}

fn rows(value: &Value) -> &[Value] {
    value.as_array().expect("массив результата")
}

fn check(case: &Value, label: &str, expected: &Expected) {
    assert_eq!(case["outcome"], expected.outcome.as_str(), "{label}");
    let trials = rows(&case["convergence"]["trials"]);
    assert_eq!(trials.len(), expected.trials, "{label}");
    assert!(trials.len() >= 3 && trials.len().is_multiple_of(3));
    for group in trials.as_chunks::<3>().0 {
        let base = group[0]["rate_hz"].as_u64().expect("базовая частота");
        for (i, trial) in group.iter().enumerate() {
            assert_eq!(trial["rate_hz"], base * (1 << i), "{label}");
            assert_eq!(rows(&trial["events"]).len(), expected.events, "{label}");
            if let Some(reason) = &expected.rejected {
                assert_eq!(trial["completed"], false, "{label}");
                assert!(
                    rows(&trial["missing"])
                        .iter()
                        .any(|value| value.as_str().unwrap().contains(reason)),
                    "{label}: {}",
                    trial["missing"]
                );
            } else {
                assert_eq!(trial["completed"], true, "{label}: {}", trial["missing"]);
                assert!(
                    trial["steps"].as_u64().is_some_and(|steps| steps > 0),
                    "{label}"
                );
            }
        }
    }
    let checks = rows(&case["checks"]);
    if expected.satisfied == Some(true) {
        assert!(!checks.is_empty(), "{label}: нет проверок по окнам");
        for row in checks {
            assert_eq!(row["outcome"], "satisfied", "{label}: {row}");
            assert!(
                row["samples"].as_u64().is_some_and(|steps| steps >= 2),
                "{label}"
            );
        }
    }
    for (name, state) in [
        (&expected.violated, "violated"),
        (&expected.missing_check, "missing"),
    ] {
        if let Some(name) = name {
            let selected: Vec<_> = checks
                .iter()
                .filter(|row| row["name"] == name.as_str())
                .collect();
            assert_eq!(selected.len(), trials.len(), "{label}: {name}");
            assert!(
                selected.iter().all(|row| row["outcome"] == state),
                "{label}: {selected:?}"
            );
        }
    }
    if let Some((name, [lo, hi])) = expected.metric.as_ref().zip(expected.range) {
        let metric = rows(&case["convergence"]["metrics"])
            .iter()
            .find(|m| m["name"] == name.as_str())
            .expect("независимый критерий");
        assert_eq!(rows(&metric["values"]).len(), trials.len());
        for row in rows(&metric["values"]) {
            for v in rows(row) {
                let v = v.as_f64().expect("измеренное конечное значение");
                assert!(
                    (lo..=hi).contains(&v),
                    "{label}: {name} = {v} вне [{lo}; {hi}]"
                );
            }
        }
    }
    if let Some(samples) = expected.reference_samples {
        let errors = rows(&case["reference"]["errors"]);
        assert!(!errors.is_empty(), "{label}: нет сверки рядов");
        assert_eq!(
            case["reference"]["passed"],
            expected.reference_passed.unwrap(),
            "{label}"
        );
        assert!(
            errors.iter().all(|row| row["samples"] == samples),
            "{label}: неполная сетка"
        );
        if expected.reference_passed == Some(true) {
            assert_eq!(errors.len(), 14, "{label}: полная схема эталонного опыта");
            assert!(
                errors.iter().all(|row| row["passed"] == true),
                "{label}: {errors:?}"
            );
        }
        for error in errors {
            eprintln!(
                "{label} {}: abs={}, отн={}, отсчётов={samples}",
                error["column"], error["max_abs_error"], error["max_relative_error"],
            );
        }
    }
    if let Some(sensitivity) = &expected.sensitivity {
        let report = &case["model_sensitivity"];
        assert_eq!(report["against"], sensitivity.against, "{label}");
        let differences = rows(&report["curves"]);
        assert_eq!(differences.len(), rows(&case["reference"]["errors"]).len());
        let diff = differences
            .iter()
            .find(|d| d["column"] == sensitivity.column)
            .unwrap();
        assert_eq!(diff["samples"], expected.reference_samples.unwrap());
        assert!(
            diff["relative_to_reference_peak"].as_f64().unwrap() >= sensitivity.minimum,
            "{label}: {diff}"
        );
        for diff in differences {
            eprintln!(
                "{label} {}: разность моделей abs={}, отн={}",
                diff["column"], diff["max_abs"], diff["relative_to_reference_peak"],
            );
        }
    }
    if let Some(peak) = &expected.nonconvergent_peak {
        let metrics = rows(&case["convergence"]["metrics"]);
        let peak = metrics.iter().find(|m| m["name"] == *peak).unwrap();
        assert_eq!(peak["outcome"], "not_analyzed");
        assert!(
            rows(&peak["observed_orders"])
                .iter()
                .all(|p| p.as_f64().is_some_and(|p| p < 0.0))
        );
        for metric in metrics {
            eprintln!("{label}: {metric}");
        }
    }
    if let Some(routes) = &expected.wet_routes {
        for route in routes {
            let counts = &case["packets"][route];
            let lost = counts["lost"].as_u64().unwrap();
            assert!(lost > 0, "{label}: {counts}");
            assert!(
                counts["retries"].as_u64().unwrap() >= 2 * lost,
                "{label}: {counts}"
            );
            assert_eq!(counts["delivered"], 0, "{label}: {counts}");
            eprintln!("{label}/{route}: {counts}");
        }
    }
}

#[test]
fn живые_прогоны_по_манифесту() {
    let base = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../devices");
    let data = std::fs::read_to_string(base.join("stage4.toml")).expect("манифест");
    let manifest: Manifest = toml::from_str(&data).expect("строгая схема манифеста");
    assert_eq!(manifest.version, 1);
    assert!(!manifest.source.trim().is_empty() && !manifest.runs.is_empty());
    std::thread::scope(|scope| {
        for run in manifest.runs {
            let base = &base;
            scope.spawn(move || check_run(base, run));
        }
    });
}

fn check_run(base: &std::path::Path, run: Run) {
    let mut out = Vec::new();
    let mut err = Vec::new();
    let mut args = vec![
        base.join(&run.world).into_os_string(),
        "--task".into(),
        base.join(&run.task).into_os_string(),
        OsString::from("--json"),
    ];
    if let Some(jobs) = run.jobs {
        args.extend(["--jobs".into(), jobs.to_string().into()]);
    }
    let code = sim_cli::run(args, std::io::empty(), &mut out, &mut err);
    let result: Value = serde_json::from_slice(&out).expect("отчёт JSON");
    assert_eq!(
        code,
        run.exit,
        "{}: {}",
        run.task.display(),
        result["detail"]
    );
    if let Some(error) = run.error {
        assert!(
            result["detail"].as_str().unwrap().contains(&error),
            "{result}"
        );
        return;
    }
    let cases = result["cases"].as_object().expect("живые прогоны");
    assert!(!run.cases.is_empty() && cases.len() == run.cases.len());
    assert!(!result["inputs"].as_object().unwrap().is_empty());
    for (name, expected) in run.cases {
        check(&cases[&name], &name, &expected);
    }
}

#[test]
fn независимые_сборки_различают_координату_контакт_и_полезную_работу() {
    let base = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../devices/stage4-audit");
    let mut out = Vec::new();
    let mut err = Vec::new();
    let code = sim_cli::run(
        vec![
            base.join("world.toml").into_os_string(),
            "--task".into(),
            base.join("acceptance.toml").into_os_string(),
            "--json".into(),
        ],
        std::io::empty(),
        &mut out,
        &mut err,
    );
    let report: Value = serde_json::from_slice(&out).expect("отчёт контрпримеров");
    assert_eq!(code, 0, "{report}");
    let cases = &report["cases"];
    let check_state = |case: &str, expected: &str, numbers: &[f64]| {
        let v = &cases[case];
        assert_eq!(rows(&v["convergence"]["trials"]).len(), 3);
        assert!(
            rows(&v["convergence"]["trials"])
                .iter()
                .all(|row| row["completed"] == true)
        );
        for group in rows(&v["checks"]).chunks_exact(numbers.len()) {
            for (check, value) in group.iter().zip(numbers) {
                let observed = rows(&check["observed"]);
                assert_eq!(check["outcome"], expected, "{check}");
                assert!(
                    (observed[0].as_f64().unwrap() - value).abs() <= 0.000002,
                    "{check}"
                );
                assert!(
                    (observed[1].as_f64().unwrap() - value).abs() <= 0.000002,
                    "{check}"
                );
            }
        }
        assert_eq!(rows(&v["checks"]).len(), 3 * numbers.len());
    };
    // Из данных стенда: нулевая осевая реакция, начальные 300 К без
    // теплового пути, длина следа ноль. Геометрия лотка даёт 56 мм,
    // до хода начальная поза 25 мм. Два мкм покрывают мягкость опоры.
    check_state("поперечная опора", "violated", &[0.0, 300.0, 0.0]);
    check_state("упор до столкновения", "violated", &[0.025]);
    check_state("рычаг и лоток", "satisfied", &[0.056]);
    check_state("координата без удержания", "violated", &[0.056, -1.0]);
    for label in [
        "поперечная опора",
        "упор до столкновения",
        "координата без удержания",
    ] {
        assert_eq!(cases[label]["outcome"], "fail");
    }

    let impulses = |case: &str, i: usize| {
        rows(&cases[case]["convergence"]["metrics"][i]["values"])
            .iter()
            .map(|row| row[0].as_f64().unwrap())
            .collect::<Vec<_>>()
    };
    let lever = "рычаг и лоток";
    assert_eq!(cases[lever]["outcome"], "not_analyzed");
    for (drive, stop) in impulses(lever, 0).into_iter().zip(impulses(lever, 1)) {
        // Интеграл вычитания реакций равен cΔy из уравнения импульса
        // блока: c=1 Н·с/м, v(0)=v(T)=0, Δy≈31 мм. Окно 2 мкН·с
        // соответствует тем же 2 мкм в контроле конечной координаты.
        assert!(drive > stop && ((drive - stop) - 0.031).abs() <= 0.000002);
    }
    assert!(
        impulses("упор до столкновения", 0)
            .into_iter()
            .all(|x| x == 0.0)
    );
    assert!(
        impulses("упор до столкновения", 1)
            .into_iter()
            .all(|x| x == 0.0)
    );
    assert!(impulses("поперечная опора", 0).into_iter().all(|x| x > 0.0));
}
