//! Приёмка связанного прогона через обычную командную строку.

use std::{
    collections::BTreeMap,
    fs,
    path::{Path, PathBuf},
    process::Command,
};

use serde::Deserialize;
use serde_json::Value;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Manifest {
    version: u32,
    source: String,
    experiments: Vec<Experiment>,
    comparisons: Vec<Comparison>,
    checkpoints: Vec<Checkpoint>,
    equivalence: Equivalence,
    invalid_inputs: InvalidInputs,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Experiment {
    id: String,
    world: PathBuf,
    task: PathBuf,
    exit: i32,
    base_hz: u64,
    minimum_steps: u64,
    cases: Vec<Case>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Case {
    id: String,
    outcome: String,
    minimum_checks: usize,
    failure: Option<String>,
    minimum_frames: Option<usize>,
    power_cut_ns: Option<u64>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Comparison {
    experiment: String,
    first: String,
    second: String,
    check: String,
    min_delta: f64,
    source: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Checkpoint {
    experiment: String,
    case: String,
    check: String,
    bounds: [f64; 2],
    source: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Equivalence {
    first_experiment: String,
    first_case: String,
    repeated_experiment: String,
    repeated_case: String,
    metrics: Vec<String>,
    absolute_tolerance: f64,
    source: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct InvalidInputs {
    experiment: String,
    disabled_check: String,
}

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .unwrap()
}

fn invoke(root: &Path, world: &Path, task: &Path, case: Option<&str>) -> (i32, Value) {
    let mut command = Command::new(env!("CARGO_BIN_EXE_sim-run"));
    command
        .current_dir(root)
        .arg(world)
        .arg("--task")
        .arg(task)
        .arg("--catalog")
        .arg(root.join("catalog"))
        .arg("--json");
    if let Some(id) = case {
        command.args(["--case", id]);
    }
    let output = command.output().unwrap();
    assert!(
        output.status.code().is_some(),
        "процесс завершён сигналом: {:?}",
        output.status
    );
    let report = serde_json::from_slice(&output.stdout).unwrap_or_else(|error| {
        panic!(
            "{error}, stderr: {}",
            String::from_utf8_lossy(&output.stderr)
        )
    });
    (output.status.code().unwrap(), report)
}

fn items(value: &Value) -> &[Value] {
    value.as_array().expect("ожидался массив в отчёте")
}

fn find<'a>(values: &'a Value, id: &str) -> &'a Value {
    items(values)
        .iter()
        .find(|v| v["id"] == id)
        .unwrap_or_else(|| panic!("нет записи {id}"))
}

fn metric(case: &Value, id: &str) -> f64 {
    items(&case["checks"])
        .iter()
        .rev()
        .find(|x| x["id"] == id)
        .and_then(|x| x["value"].as_f64())
        .unwrap_or_else(|| panic!("нет измерения {id}"))
}

#[test]
fn матрица_связанных_прогонов() {
    let root = root();
    let data = root.join("devices");
    let manifest: Manifest =
        toml::from_str(&fs::read_to_string(data.join("stage5.toml")).unwrap()).unwrap();
    assert_eq!(manifest.version, 1);
    assert!(!manifest.source.is_empty());
    assert_eq!(manifest.experiments.len(), 4);
    let mut reports = BTreeMap::new();
    for exp in &manifest.experiments {
        let world = data.join(&exp.world);
        let task = data.join(&exp.task);
        let (exit, report) = invoke(&root, &world, &task, None);
        assert_eq!(
            exit,
            exp.exit,
            "{}: {}",
            exp.id,
            report.get("detail").unwrap_or(&Value::Null)
        );
        let cases = items(&report["cases"]);
        assert_eq!(cases.len(), exp.cases.len());
        for (actual, expected) in cases.iter().zip(&exp.cases) {
            assert_eq!(actual["id"], expected.id, "порядок матрицы");
            assert_eq!(
                actual["outcome"], expected.outcome,
                "{}/{}",
                exp.id, expected.id
            );
            assert_ne!(actual["readiness"], "Passed");
            let grids = items(&actual["coverage"]);
            let ev = &actual["evidence"];
            assert_eq!(grids.len(), 3);
            assert_eq!(items(&ev["grids"]).len(), 3);
            assert_eq!(items(&ev["discrepancies"]).len(), 5);
            for discrepancy in items(&ev["discrepancies"]) {
                let id = discrepancy["id"].as_str().unwrap();
                assert!(
                    items(&ev["blockers"])
                        .iter()
                        .any(|entry| entry.as_str().unwrap().starts_with(&format!("{id}:"))),
                    "расхождение «{id}» не участвует в вердикте"
                );
                assert!(
                    discrepancy["simulated"]
                        .as_f64()
                        .is_some_and(f64::is_finite)
                );
                assert!(
                    discrepancy["reference"]
                        .as_f64()
                        .is_some_and(f64::is_finite)
                );
                assert_ne!(discrepancy["simulated"], discrepancy["reference"]);
                for field in ["mode", "note", "source", "tolerance"] {
                    assert!(
                        discrepancy[field]
                            .as_str()
                            .is_some_and(|s| !s.trim().is_empty())
                    );
                }
            }
            assert!(!items(&ev["power"]).is_empty());
            assert!(
                items(&ev["power"])
                    .iter()
                    .any(|r| r["outcome"] == "not_analyzed")
            );
            assert!(
                actual["convergence"]
                    .as_object()
                    .is_some_and(|o| !o.is_empty())
            );
            let exposure = items(&actual["tissue"]);
            assert!(!exposure.is_empty());
            assert!(exposure.iter().all(|row| row["outcome"] == "not_analyzed"));
            let mut journals = std::collections::BTreeSet::new();
            for (i, grid) in grids.iter().enumerate() {
                let factor = 1 << i;
                assert_eq!(
                    grid["rate_hz"].as_f64(),
                    Some((exp.base_hz * factor) as f64)
                );
                assert_eq!(
                    grid["status"], "completed",
                    "{}: {}",
                    expected.id, grid["failure"]
                );
                assert_eq!(grid["expected_steps"], grid["completed_steps"]);
                assert!(grid["completed_steps"].as_u64().unwrap() >= exp.minimum_steps * factor);
                assert!(journals.insert(ev["grids"][i]["journal"].as_u64().unwrap()));
                if let Some(time) = expected.power_cut_ns {
                    assert_eq!(ev["grids"][i]["power_loss_applied_ns"], time);
                }
                let checks = items(&actual["checks"])
                    .iter()
                    .filter(|check| check["rate_hz"] == grid["rate_hz"])
                    .collect::<Vec<_>>();
                assert!(checks.len() >= expected.minimum_checks);
                assert!(checks.iter().any(|r| r["samples"].as_u64().unwrap() > 1));
                if let Some(failure) = &expected.failure {
                    let finding = checks.iter().find(|r| r["id"] == *failure).unwrap();
                    assert_eq!(finding["outcome"], "fail", "{failure}: {finding}");
                    assert_eq!(finding["required_design"], true);
                    assert_eq!(actual["readiness"], "Rejected");
                    if factor == 4 {
                        eprintln!(
                            "{} / {failure}: {} против {}",
                            expected.id, finding["value"], finding["bounds"]
                        );
                    }
                } else {
                    assert_eq!(
                        actual["numerical"],
                        "pass",
                        "{}: {:?}",
                        expected.id,
                        checks
                            .iter()
                            .filter(|r| r["outcome"] != "pass")
                            .collect::<Vec<_>>()
                    );
                }
                if let Some(minimum) = expected.minimum_frames {
                    let frames = items(&grid["camera"]["frames"]);
                    assert!(frames.len() >= minimum);
                    assert!(
                        frames
                            .iter()
                            .all(|f| f["projection_error_px"].as_f64().unwrap() <= 0.1)
                    );
                    assert!(frames.windows(2).any(|p| p[0]["image"] != p[1]["image"]));
                }
            }
        }
        assert!(reports.insert(exp.id.as_str(), report).is_none());
    }
    for c in &manifest.checkpoints {
        assert!(!c.source.is_empty());
        let value = metric(
            find(&reports[c.experiment.as_str()]["cases"], &c.case),
            &c.check,
        );
        assert!(
            (c.bounds[0]..=c.bounds[1]).contains(&value),
            "{}: {value}",
            c.check
        );
    }
    for c in &manifest.comparisons {
        assert!(!c.source.is_empty());
        let cases = &reports[c.experiment.as_str()]["cases"];
        let a = metric(find(cases, &c.first), &c.check);
        let b = metric(find(cases, &c.second), &c.check);
        assert!(
            (a - b).abs() >= c.min_delta,
            "{}/{}, {}: {a} и {b}",
            c.first,
            c.second,
            c.check
        );
    }
    let eq = &manifest.equivalence;
    assert!(!eq.source.is_empty());
    assert!(!eq.metrics.is_empty());
    let exp = manifest
        .experiments
        .iter()
        .find(|e| e.id == eq.repeated_experiment)
        .unwrap();
    let (status, repeated) = invoke(
        &root,
        &data.join(&exp.world),
        &data.join(&exp.task),
        Some(&eq.repeated_case),
    );
    assert_eq!(status, exp.exit);
    assert_eq!(
        repeated["cases"],
        reports[eq.repeated_experiment.as_str()]["cases"]
    );
    let alternate = find(&repeated["cases"], &eq.repeated_case);
    let nominal = find(
        &reports[eq.first_experiment.as_str()]["cases"],
        &eq.first_case,
    );
    assert_ne!(alternate["evidence"]["build"], nominal["evidence"]["build"]);
    for id in &eq.metrics {
        let a = metric(nominal, id);
        let b = metric(alternate, id);
        assert!((a - b).abs() <= eq.absolute_tolerance, "{id}: {a} и {b}");
    }
}

struct Scratch(PathBuf);

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

#[test]
fn повреждённые_входы_блокируют_вердикт() {
    let root = root();
    let data = root.join("devices");
    let manifest: Manifest =
        toml::from_str(&fs::read_to_string(data.join("stage5.toml")).unwrap()).unwrap();
    let exp = manifest
        .experiments
        .iter()
        .find(|e| e.id == manifest.invalid_inputs.experiment)
        .unwrap();
    let task_dir = data.join(&exp.task).parent().unwrap().to_owned();
    let tmp = Scratch(std::env::temp_dir().join(format!("sim-interaction-{}", std::process::id())));
    fs::create_dir(&tmp.0).unwrap();
    let taskfile = tmp.0.join("acceptance.toml");
    fs::write(
        &taskfile,
        "version = 1\ninteraction_profile = 'profile.toml'\n",
    )
    .unwrap();
    fs::write(tmp.0.join("profile.toml"), "version = 1\nsource = 'Отрицательные проверки'\nbase = 'setup.toml'\n[[cases]]\nid = 'audit'\n").unwrap();
    let mut base: toml::Table =
        toml::from_str(&fs::read_to_string(task_dir.join("setup.toml")).unwrap()).unwrap();
    for (table, key) in [("contacts", "table"), ("control", "wiring")] {
        let field = base[table].as_table_mut().unwrap().get_mut(key).unwrap();
        let path = task_dir
            .join(field.as_str().unwrap())
            .canonicalize()
            .unwrap();
        *field = toml::Value::String(path.to_str().unwrap().into());
    }
    let audit = |mutated: &toml::Table, expected: &str| {
        fs::write(tmp.0.join("setup.toml"), toml::to_string(mutated).unwrap()).unwrap();
        let (status, result) = invoke(&root, &data.join(&exp.world), &taskfile, None);
        assert_eq!(status, 2);
        assert_eq!(result["proof_status"], "not_analyzed");
        assert!(
            result["detail"].as_str().unwrap().contains(expected),
            "{result}"
        );
    };
    for key in ["motion", "power"] {
        let mut missing = base.clone();
        missing.remove(key);
        audit(&missing, key);
    }
    let mutate_pad = |case: &mut toml::Table, field: &str, value: toml::Value| {
        let pad = case["pads"]
            .as_table_mut()
            .unwrap()
            .iter_mut()
            .map(|(_, value)| value)
            .next()
            .unwrap();
        pad.as_table_mut().unwrap().insert(field.into(), value);
    };
    let mut visual = base.clone();
    let pad = base["pads"].as_table().unwrap().values().next().unwrap();
    let mut geometries = pad["geometries"].as_array().unwrap().clone();
    geometries[0] = geometries[0]
        .as_str()
        .unwrap()
        .replace("/collision/", "/visual/")
        .into();
    mutate_pad(&mut visual, "geometries", toml::Value::Array(geometries));
    audit(&visual, "физическая поверхность");
    let mut nonfinite = base.clone();
    mutate_pad(&mut nonfinite, "normal", toml::Value::Float(f64::NAN));
    audit(&nonfinite, "конечные числа");

    let mut disabled = base.clone();
    for pad in disabled["pads"]
        .as_table_mut()
        .unwrap()
        .iter_mut()
        .map(|(_, value)| value)
    {
        let law = pad["law"].as_table_mut().unwrap();
        for point in law["compression"]["points"].as_array_mut().unwrap() {
            let force = &mut point.as_array_mut().unwrap()[1];
            // Почти нулевая, но строго возрастающая характеристика доходит до живого прогона.
            *force = (force.as_float().unwrap() * 1e-9).into();
        }
    }
    fs::write(
        tmp.0.join("setup.toml"),
        toml::to_string(&disabled).unwrap(),
    )
    .unwrap();
    let (status, report) = invoke(&root, &data.join(&exp.world), &taskfile, None);
    assert_eq!(
        status,
        1,
        "{}",
        report.get("detail").unwrap_or(&Value::Null)
    );
    let case = find(&report["cases"], "audit");
    assert_eq!(case["numerical"], "fail", "{:?}", case["checks"]);
    assert!(
        items(&case["checks"])
            .iter()
            .any(|c| c["id"] == manifest.invalid_inputs.disabled_check && c["outcome"] == "fail")
    );
}

#[test]
fn независимая_сборка_проверяет_контакт_реакцию_и_учёт_энергии() {
    let root = root();
    let data = root.join("devices/stage5-audit");
    let text = |p: &str| fs::read_to_string(data.join(p)).unwrap();
    let tmp = Scratch(std::env::temp_dir().join(format!("sim-block-{}", std::process::id())));
    fs::create_dir(&tmp.0).unwrap();
    for path in ["acceptance.toml", "profile.toml", "block.toml"] {
        fs::copy(data.join(path), tmp.0.join(path)).unwrap();
    }
    let mut setup: toml::Table = toml::from_str(&text("setup.toml")).unwrap();
    let contacts = setup["contacts"].as_table_mut().unwrap();
    contacts.insert(
        "table".into(),
        root.join("catalog/contact/stage5.toml")
            .to_str()
            .unwrap()
            .into(),
    );
    let world = text("world.toml");
    let run = |name: &str, setup: &toml::Table, world: &str| {
        fs::write(tmp.0.join("setup.toml"), toml::to_string(setup).unwrap()).unwrap();
        fs::write(tmp.0.join("world.toml"), world).unwrap();
        let (status, report) = invoke(
            &root,
            &tmp.0.join("world.toml"),
            &tmp.0.join("acceptance.toml"),
            None,
        );
        assert!(report.get("detail").is_none(), "{name}: {report}");
        assert_eq!(status, 1, "{name}");
        let row = find(&report["cases"], "контроль");
        assert_eq!(row["outcome"], "not_analyzed", "{name}");
        assert_eq!(row["readiness"], "BenchOnly", "{name}");
        assert_eq!(items(&row["coverage"]).len(), 3);
        assert!(
            items(&row["coverage"])
                .iter()
                .all(|g| g["status"] == "completed")
        );
        eprintln!(
            "{name}: {:?}, реакция {} Н",
            row["readiness"], row["evidence"]["grids"][2]["patches"]["опора"]["peak_normal_n"]
        );
        row.clone()
    };
    let blocked = |row: &Value, phrase: &str| {
        assert!(
            items(&row["evidence"]["blockers"])
                .iter()
                .chain(items(&row["safety_gaps"]))
                .any(|s| s.as_str().unwrap().contains(phrase)),
            "нет блокировки «{phrase}»"
        );
    };
    let power = |row: &Value| items(&row["evidence"]["power"]).to_vec();
    let base = run("одна сфера", &setup, &world);
    assert_eq!(base["numerical"], "pass");
    assert!(power(&base).iter().any(|r| r["confirmed_hazard"] == true));
    blocked(&base, "без подтверждённой площади");
    blocked(&base, "нет предела MeanShear");
    blocked(&base, "реакции участника");

    let mut moved: toml::Table = toml::from_str(&world).unwrap();
    let placement = &mut moved["devices"].as_array_mut().unwrap()[0];
    // Перемещение на 1 м по X против радиуса сферы 10 мм.
    let x = &mut placement["pose"]["position_m"].as_array_mut().unwrap()[0];
    *x = (x.as_float().unwrap() + 1.0).into();
    let miss = run("промах", &setup, &toml::to_string(&moved).unwrap());
    blocked(&miss, "нет подтверждённого нагружения");
    assert_eq!(
        miss["evidence"]["grids"][2]["patches"]["опора"]["peak_normal_n"],
        0.0
    );

    let mut outside = setup.clone();
    // Вне 5–95 перцентилей роста в исходной таблице, масса остаётся исходной.
    outside["participant"]["stature_m"]["value"] = 2.2.into();
    let row = run("рост вне выборки", &outside, &world);
    blocked(&row, "антропометрия вне диапазона");

    let mut untriggered = setup.clone();
    let response = toml::from_str::<toml::Table>(
        r#"
enabled = true
initiator = "контрольный порог вне нагрузки"
patch = "опора"
# Реакция оснастки ограничена 2 Н, порог 3 Н заведомо не достигнут.
threshold_n = { value = 3.0, source = { measured = "Синтетический порог 2 Н + 1 Н запаса." } }
latency_ns = 0
latency_source = { measured = "Нулевая задержка фикстуры для проверки промаха порога." }
[offsets.j0038]
knots = [[0, 0.0], [180000000, 0.045]]
source = { measured = "Опорный поворот контрольного стенда." }
"#,
    )
    .unwrap();
    untriggered["motion"]
        .as_table_mut()
        .unwrap()
        .insert("response".into(), response.into());
    let no_response = run("порог не достигнут", &untriggered, &world);
    blocked(&no_response, "реакция на контакт не подтверждена");

    let mut renamed = setup.clone();
    let sources = renamed["power"]["sources"].as_table_mut().unwrap();
    let energy = sources.remove("блок питания").unwrap();
    sources.insert("крепёж".into(), energy.clone());
    renamed["power"]["required"] = toml::Value::Array(vec!["крепёж".into()]);
    let renamed_row = run("переименование источника", &renamed, &world);
    assert!(
        power(&renamed_row)
            .iter()
            .any(|r| r["source"] == "крепёж" && r["confirmed_hazard"] == true)
    );
    renamed["power"]["sources"]["крепёж"]["electrical"]["stored_j"]["value"] = 0.0.into();
    let zero = run("ноль в файле", &renamed, &world);
    assert!(
        power(&zero)
            .iter()
            .any(|r| r["source"].is_null() && r["outcome"] == "not_analyzed")
    );

    let mut split = setup;
    let sources = split["power"]["sources"].as_table_mut().unwrap();
    sources.clear();
    for name in ["регистратор", "крышка"] {
        let mut part = energy.clone();
        // Два источника 0,6 Дж по отдельности ниже границы 1 Дж.
        part["electrical"]["stored_j"]["value"] = 0.6.into();
        sources.insert(name.into(), part);
    }
    split["power"]["required"] = toml::Value::Array(vec!["регистратор".into(), "крышка".into()]);
    fs::write(
        tmp.0.join("cover.toml"),
        text("block.toml").replace("[1, 0]", "[0, 0]"),
    )
    .unwrap();
    let separate = world
        + r#"
[[devices]]
name = "крышка"
file = "cover.toml"
# Отдельное размещение на расстоянии 1 м от начала сцены.
pose = { position_m = [1.0, 0.0, 0.0] }
"#;
    let split = run("два размещения по 0,6 Дж", &split, &separate);
    let findings = power(&split);
    assert!(findings.iter().any(|r| r["source"].is_null()
        && r["path"] == "Electrical"
        && r["confirmed_hazard"] == true));
    assert_eq!(
        findings
            .iter()
            .filter(|r| r["source"].is_string()
                && r["path"] == "Electrical"
                && r["outcome"] == "pass")
            .count(),
        2
    );
}
