use std::collections::BTreeSet;
use std::path::Path;
use std::process::Command;
use std::time::Instant;

use serde::Deserialize;
use serde_json::Value;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Manifest {
    version: u32,
    runs: Vec<Run>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Run {
    id: u32,
    world: String,
    task: String,
    outcome: String,
    #[serde(default)]
    reference_modes: Vec<String>,
    #[serde(default)]
    deferred_modes: Vec<String>,
    cases: Vec<Case>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Case {
    path: String,
    mode: String,
    fault: Option<Fault>,
    read: Option<bool>,
    accepted: Option<bool>,
    changes: Option<bool>,
    expired: Option<bool>,
    watchdog: Option<bool>,
    unchanged_pair: Option<bool>,
    action: Option<String>,
    applied: Option<String>,
    applied_at_ns: Option<u64>,
    last_packet_before: Option<String>,
    comparison: Option<Comparison>,
    #[serde(default)]
    metrics: Vec<Metric>,
    #[serde(default)]
    delays: Vec<Delay>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Fault {
    node: String,
    kind: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Delay {
    hop: String,
    min_ns: u64,
    max_ns: u64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Metric {
    signal: String,
    field: String,
    min: Option<f64>,
    max: Option<f64>,
    gt: Option<f64>,
    equals: Option<f64>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Comparison {
    mode: String,
    signal: String,
    field: String,
}

fn row<'a>(report: &'a Value, path: &str, mode: &str, fault: Option<&Fault>) -> &'a Value {
    report["evidence"]["cases"]
        .as_array()
        .unwrap()
        .iter()
        .find(|row| {
            let case = &row["case"];
            case["path"] == path
                && case["mode"] == mode
                && match fault {
                    Some(fault) => {
                        case["fault"]["node"] == fault.node && case["fault"]["kind"] == fault.kind
                    }
                    None => case["fault"].is_null(),
                }
        })
        .unwrap_or_else(|| panic!("нет строки {path} / {mode}"))
}

fn value(row: &Value, signal: &str, field: &str) -> f64 {
    row["metrics"]
        .as_array()
        .unwrap()
        .iter()
        .find(|item| item["signal"]["name"] == signal)
        .and_then(|item| item[field].as_f64())
        .unwrap_or_else(|| panic!("нет метрики {signal} / {field} в {}", row["case"]))
}

fn check(report: &Value, case: &Case) {
    let row = row(report, &case.path, &case.mode, case.fault.as_ref());
    assert_eq!(row["completed"], true, "{row}");
    let control = &row["control"];
    for (field, expected) in [
        ("valid_reads", case.read),
        ("model_acceptances", case.accepted),
        ("command_changes", case.changes),
        ("expired_outputs", case.expired),
    ] {
        if let Some(expected) = expected {
            assert_eq!(
                control[field].as_u64().unwrap() > 0,
                expected,
                "{field}: {row}"
            );
        }
    }
    if case.watchdog == Some(true) {
        assert!(
            control["resets"]
                .as_array()
                .unwrap()
                .iter()
                .any(|reset| reset[1] == "Watchdog"),
            "{control}"
        );
    }
    if let Some(action) = &case.action {
        assert!(control["actions"][action].is_u64(), "{control}");
    }
    if let Some(unchanged) = case.unchanged_pair {
        let audit = &row["comparison_audit"];
        assert!(audit["changed_observations"].as_u64().unwrap() > 0, "{row}");
        assert_eq!(audit["command_stream_equal"], unchanged, "{row}");
        if unchanged {
            assert!(
                row["missing"].as_array().unwrap().iter().any(|entry| {
                    entry
                        .as_str()
                        .unwrap()
                        .contains("изменение доставленных измерений не изменило команды")
                }),
                "{row}"
            );
        }
    }
    if let Some(target) = &case.applied {
        assert_eq!(
            control["applied_faults"][target].as_u64(),
            case.applied_at_ns,
            "{control}"
        );
    }
    if let Some(route) = &case.last_packet_before {
        assert!(control["last_packet_ns"][route].as_u64().unwrap() < case.applied_at_ns.unwrap());
    }
    for metric in &case.metrics {
        let v = value(row, &metric.signal, &metric.field);
        assert!(
            metric.min.is_none_or(|min| v >= min)
                && metric.max.is_none_or(|max| v <= max)
                && metric.gt.is_none_or(|min| v > min)
                && metric.equals.is_none_or(|equal| v == equal),
            "{} / {}: {v}, {}",
            metric.signal,
            metric.field,
            row["case"]
        );
    }
    for delay in &case.delays {
        assert!(
            row["deliveries"].as_array().unwrap().iter().any(|item| {
                item["hop"] == delay.hop
                    && item["delivered"].as_u64().unwrap() > 0
                    && item["measured_min_ns"].as_u64().unwrap() >= delay.min_ns
                    && item["measured_max_ns"].as_u64().unwrap() <= delay.max_ns
            }),
            "{row}"
        );
    }
    if let Some(compare) = &case.comparison {
        let baseline = self::row(report, &case.path, &compare.mode, None);
        assert!(
            value(row, &compare.signal, &compare.field)
                > value(baseline, &compare.signal, &compare.field),
            "{row}"
        );
    }
}

#[test]
fn исполняемые_сборки_сходятся_с_манифестом() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let data = root.join("devices");
    let manifest: Manifest =
        toml::from_str(&std::fs::read_to_string(data.join("stage3.toml")).unwrap()).unwrap();
    assert_eq!(manifest.version, 1);
    assert!(!manifest.runs.is_empty());
    for run in &manifest.runs {
        let start = Instant::now();
        let output = Command::new(env!("CARGO_BIN_EXE_sim-run"))
            .arg(data.join(&run.world))
            .arg("--task")
            .arg(data.join(&run.task))
            .arg("--json")
            .output()
            .unwrap();
        let report: Value = serde_json::from_slice(&output.stdout).unwrap_or_else(|error| {
            panic!(
                "{}: {error}; {}",
                run.id,
                String::from_utf8_lossy(&output.stderr)
            )
        });
        println!("Сборка {}: {:.2} с", run.id, start.elapsed().as_secs_f64());
        assert!(output.status.success(), "{}: {report}", run.id);
        assert_eq!(report["status"], "completed");
        assert_eq!(report["closed_loop"], run.outcome);
        assert!(!run.cases.is_empty());
        for case in &run.cases {
            check(&report, case);
        }
        let measured_modes = report["reference"].as_object().unwrap();
        assert_eq!(
            measured_modes.keys().collect::<BTreeSet<_>>(),
            run.reference_modes.iter().collect::<BTreeSet<_>>(),
        );
        for mode in &run.reference_modes {
            let reference = &report["reference"][mode];
            let errors = reference["errors"].as_array().unwrap();
            // Полная сетка 5 мс на горизонте 3 с для каждой колонки CSV.
            assert_eq!(errors.len(), 14);
            assert_eq!(
                errors
                    .iter()
                    .filter(|item| item["column"] == "reported_faults")
                    .count(),
                1,
                "{mode}: {reference}"
            );
            for item in errors {
                assert_eq!(item["samples"], 600);
                match item["column"].as_str().unwrap() {
                    "reported_faults" | "contact_n" | "driver_on" => {
                        assert_eq!(item["different_samples"], 0, "{mode}: {item}");
                        assert_eq!(item["max_abs_error"].as_f64(), Some(0.0), "{mode}: {item}");
                    }
                    _ => assert_eq!(item["passed"], true, "{mode}: {item}"),
                }
            }
            assert_eq!(reference["passed"], true, "{mode}: {reference}");
        }
        for mode in &run.deferred_modes {
            let rows: Vec<_> = report["evidence"]["cases"]
                .as_array()
                .unwrap()
                .iter()
                .filter(|row| row["case"]["mode"] == *mode)
                .collect();
            assert!(!rows.is_empty());
            assert!(rows.iter().all(|row| row["outcome"] == "not_analyzed"));
        }
    }
}
