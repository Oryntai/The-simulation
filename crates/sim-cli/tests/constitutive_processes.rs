use std::{collections::BTreeMap, path::Path, process::Command};

use serde::Deserialize;
use serde_json::Value;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Manifest {
    version: u32,
    world: String,
    task: String,
    source: String,
    cases: BTreeMap<String, Expected>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Expected {
    outcome: String,
    states: Option<Vec<String>>,
    missing: Option<String>,
    violation: Option<String>,
    #[serde(default)]
    metrics: Vec<Metric>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Metric {
    name: String,
    value: f64,
    absolute_tolerance: f64,
    outcome: String,
    source: String,
}

fn base() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../devices/stage4-processes")
}

fn run(world: &str, task: &str, selected: Option<&str>) -> std::process::Output {
    let mut run = Command::new(env!("CARGO_BIN_EXE_sim-run"));
    run.current_dir(base())
        .args([world, "--task", task, "--json"]);
    if let Some(case) = selected {
        run.args(["--case", case]);
    }
    run.output().expect("запуск sim-run")
}

fn rows(value: &Value) -> &[Value] {
    value.as_array().expect("массив полей отчёта")
}

fn has_reason(rows: &[Value], expected: &str) -> bool {
    rows.iter()
        .any(|row| row.as_str().is_some_and(|item| item.contains(expected)))
}

#[test]
fn прогоны_моделей_и_отказов_из_данных() {
    let manifest: Manifest = toml::from_str(
        &std::fs::read_to_string(base().join("manifest.toml")).expect("манифест опытов"),
    )
    .expect("строгий формат манифеста");
    assert_eq!(manifest.version, 1);
    assert!(!manifest.source.trim().is_empty());
    let first = run(&manifest.world, &manifest.task, None);
    assert!(
        first.status.success(),
        "{}",
        String::from_utf8_lossy(&first.stderr)
    );
    let repeated = run(&manifest.world, &manifest.task, None);
    assert!(repeated.status.success());
    assert_eq!(
        first.stdout, repeated.stdout,
        "побитовая воспроизводимость отчёта"
    );
    let report: Value = serde_json::from_slice(&first.stdout).expect("отчёт sim-run");
    assert_eq!(report["status"], "completed");
    assert_eq!(report["outcome"], "fail");
    let cases = report["cases"].as_object().expect("режимы опыта");
    assert_eq!(cases.len(), manifest.cases.len());
    // Десять пар обязательных режимов, остальные проверяют пропуски и отпускание.
    assert!(cases.len() >= 20);
    for (name, expected) in manifest.cases {
        let case = &cases[&name];
        assert_eq!(case["outcome"], expected.outcome, "{name}: {case}");
        let convergence = &case["convergence"];
        let trials = rows(&convergence["trials"]);
        for (trial, hz) in trials.iter().zip([1000, 2000, 4000]) {
            assert_eq!(trial["rate_hz"], hz, "{name}");
            assert_eq!(trial["completed"], true, "{name}: {trial}");
            assert!(
                trial["steps"].as_u64().is_some_and(|steps| steps > 0),
                "{name}"
            );
            assert!(trial["journal"].as_u64().is_some(), "{name}");
            if let Some(states) = &expected.states {
                assert_eq!(trial["states"]["стык"], serde_json::json!(states), "{name}");
                let events: Vec<_> = rows(&trial["events"])
                    .iter()
                    .map(|event| event["state"].as_str().expect("состояние события"))
                    .collect();
                // При начале окна на первом событии оно входит и в последовательность.
                let expected_after_start = if states[0] == "Free" {
                    &states[1..]
                } else {
                    states
                };
                assert_eq!(events, expected_after_start, "{name}");
            }
            let missing = rows(&trial["missing"]);
            if let Some(reason) = &expected.missing {
                assert!(has_reason(missing, reason), "{name}: {trial}");
            } else if expected.outcome != "fail" {
                assert!(
                    !has_reason(missing, "пропущено или добавлено событие"),
                    "{name}"
                );
                assert!(
                    !has_reason(missing, "событие «стык»: иной порядок"),
                    "{name}"
                );
            }
            let violations = rows(&trial["violations"]);
            if let Some(reason) = &expected.violation {
                assert!(has_reason(violations, reason), "{name}: {trial}");
            } else {
                assert!(violations.is_empty(), "{name}: {trial}");
            }
        }
        assert_eq!(trials.len(), 3, "{name}");
        for metric in expected.metrics {
            assert!(!metric.source.trim().is_empty());
            let line = rows(&convergence["metrics"])
                .iter()
                .find(|line| line["name"] == metric.name)
                .expect("численная сверка");
            assert_eq!(line["outcome"], metric.outcome, "{name}: {line}");
            let values = rows(&line["values"]);
            assert_eq!(values.len(), trials.len(), "{name}");
            for v in values {
                for observed in rows(v) {
                    let observed = observed.as_f64().expect("численный отсчёт");
                    assert!(
                        (observed - metric.value).abs() <= metric.absolute_tolerance,
                        "{name}: {observed} против {} ({})",
                        metric.value,
                        metric.source
                    );
                }
            }
        }
    }
}

#[test]
fn неконечные_числа_и_лишние_поля_явно_отклоняются() {
    for (case, reason) in [
        ("nan", "только конечные числа"),
        ("inf", "только конечные числа"),
        ("unknown-key", "unused"),
    ] {
        let output = run("world-block.toml", "invalid.toml", Some(case));
        assert_eq!(output.status.code(), Some(2), "{case}");
        let report: Value = serde_json::from_slice(&output.stdout).expect("отчёт ошибки");
        assert!(
            report["detail"]
                .as_str()
                .is_some_and(|e| e.contains(reason)),
            "{report}"
        );
    }
}
