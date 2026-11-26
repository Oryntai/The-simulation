use std::{collections::BTreeSet, fs, path::PathBuf, process::Command};

use serde_json::Value;
use sim_process::schematic::{Catalog, ElectricalClass, Expected, Stage, StaticClass, Status};
use sim_task::Fingerprint;

#[path = "fixtures/electrical_verdict.rs"]
mod electrical_verdict;
use electrical_verdict::ElectricalStand;

#[test]
fn сборочные_листы_и_открытые_обязательства_совпадают_с_исходными_схемами() {
    let manifest =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../devices/electrical/acceptance.toml");
    let content: toml::Value = toml::from_str(&fs::read_to_string(&manifest).unwrap()).unwrap();
    let run = || {
        let output = Command::new(env!("CARGO_BIN_EXE_sim-run"))
            .args(["--campaign", manifest.to_str().unwrap(), "--json"])
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        output.stdout
    };
    let bytes = run();
    assert_eq!(bytes, run(), "журналы и сборочные листы повторного опыта");
    let report: Value = serde_json::from_slice(&bytes).unwrap();
    let configs = content["assemblies"].as_table().unwrap();
    let ids = ["01", "02", "03", "04", "05", "06", "07"];
    assert_eq!(configs.keys().map(String::as_str).collect::<Vec<_>>(), ids);
    assert_eq!(
        report["assemblies"]
            .as_object()
            .unwrap()
            .keys()
            .map(String::as_str)
            .collect::<Vec<_>>(),
        ids
    );
    let mut resets = 0;
    let mut sheets = 0;
    for (id, config) in configs {
        let assembly = &report["assemblies"][id];
        let world_path = manifest
            .parent()
            .unwrap()
            .join(config["world"].as_str().unwrap());
        let world = sim_world::load_and_compile_with_schematics(&world_path).unwrap();
        let findings = assembly["findings"].as_array().unwrap();
        assert_eq!(assembly["simulation"], "NotProven", "{id}: {findings:?}");
        assert!(findings.iter().any(|f| f["outcome"] == "not_analyzed"));
        assert_eq!(
            world.schematics.len(),
            if id == "06" { 4 } else { 1 },
            "{id}"
        );
        assert!(config["profile"].is_str(), "{id}");
        let live = &assembly["live"];
        assert_eq!(live["complete"], true);
        assert_eq!(
            live["observed_ns"],
            config["horizon_ns"].as_integer().unwrap()
        );
        assert_eq!(live["required_ns"], live["observed_ns"]);
        assert!(live["journal"].as_u64().is_some(), "{id}");
        let edges = live["summary"]["reset_edges"].as_array().unwrap();
        if !edges.is_empty() {
            resets += 1;
            assert!(edges.iter().any(|edge| edge["asserted"] == true));
            assert!(edges.iter().any(|edge| edge["asserted"] == false));
            assert!(findings.iter().any(|f| f["code"] == "rail_sag"));
            assert!(
                findings
                    .iter()
                    .filter(|f| f["code"] == "rail_sag")
                    .all(|f| f["outcome"] == "not_analyzed")
            );
        }
        assert_eq!(
            assembly["assemblies"].as_object().unwrap().len(),
            world.schematics.len()
        );
        let path = manifest
            .parent()
            .unwrap()
            .join(format!("assembly-{id}.json"));
        let published: Value = serde_json::from_slice(&fs::read(&path).unwrap()).unwrap();
        assert_eq!(published.as_object().unwrap().len(), world.schematics.len());
        for (instance, path) in world.schematics {
            sheets += 1;
            let bytes = fs::read(path).unwrap();
            let raw: toml::Value = toml::from_str(std::str::from_utf8(&bytes).unwrap()).unwrap();
            let record = &assembly["assemblies"][&instance];
            assert_eq!(
                record["schematic"].as_u64(),
                Some(Fingerprint::of(&bytes).as_u64())
            );
            assert_eq!(record["build"], assembly["build"]);
            assert_eq!(record["journal"], live["journal"]);
            let sheet = &record["sheet"];
            assert_eq!(raw["scope"].as_str(), Some("assembly"));
            assert_eq!(record["proof_scope"], "сборка");
            assert_eq!(
                published[&instance],
                serde_json::json!({
                    "schematic": record["schematic"],
                    "proof_scope": record["proof_scope"],
                    "sheet": sheet,
                }),
                "опубликованный лист {id}/{instance}",
            );
            let source_parts = raw["components"].as_table().unwrap();
            assert!(source_parts.len() > 1);
            assert_eq!(sheet["parts"].as_array().unwrap().len(), source_parts.len());
            let mut wires = BTreeSet::new();
            let mut seen_parts = BTreeSet::new();
            let mut stand_supplies = 0;
            for item in sheet["parts"].as_array().unwrap() {
                let id = item["component"].as_str().unwrap();
                assert!(seen_parts.insert(id), "{id}");
                assert_eq!(item["part"], source_parts[id]["part"].as_str().unwrap());
                assert_eq!(
                    item["pins"],
                    serde_json::to_value(&source_parts[id]["nets"]).unwrap()
                );
                if item["kind"] == "BenchSupply" {
                    stand_supplies += 1;
                    assert!(!item["missing_parameters"].as_object().unwrap().is_empty());
                    assert!(!item["model_gap"].as_str().unwrap().is_empty());
                }
                if item["kind"] == "Wire" {
                    wires.extend(
                        item["pins"]
                            .as_object()
                            .unwrap()
                            .keys()
                            .map(|pin| format!("{id}/{pin}")),
                    );
                    assert!(!item["missing_parameters"].as_object().unwrap().is_empty());
                }
            }
            // Две условные шины живого стенда №05 описаны в каталоге как открытые позиции.
            assert_eq!(stand_supplies, if id == "05" { 2 } else { 0 });
            assert!(!wires.is_empty());
            let raw_operations = raw["connections"].as_table().unwrap();
            let operations = sheet["operations"].as_array().unwrap();
            assert_eq!(operations.len(), raw_operations.len());
            let inspections = sheet["inspections"].as_array().unwrap();
            let mut performed = BTreeSet::new();
            for op in operations {
                let id = op["id"].as_str().unwrap();
                let raw = &raw_operations[id];
                verify_ends(&op["ends"], &raw["ends"]);
                let after = raw.get("after").map(raw_ends).unwrap_or_default();
                let published_after = op["after"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|item| item.as_str().unwrap())
                    .collect::<BTreeSet<_>>();
                assert_eq!(published_after, after.iter().map(String::as_str).collect());
                assert!(after.iter().all(|pred| performed.contains(pred.as_str())));
                assert!(performed.insert(id));
                assert_eq!(op["compatible"], true);
                let kind = match (
                    raw.get("crimp").and_then(toml::Value::as_bool),
                    raw["kind"].as_str().unwrap(),
                ) {
                    (Some(true), _) => "Crimp",
                    (_, "solder") => "Solder",
                    (_, "terminal") => "Terminal",
                    (_, "splice") => "Splice",
                    (_, "plug") => "Plug",
                    (_, kind) => panic!("неизвестный вид операции: {kind}"),
                };
                assert_eq!(op["kind"], kind);
                for end in ends(&op["ends"]) {
                    wires.remove(&end);
                }
                for (test, stage) in [
                    ("resistance", "before_power"),
                    ("current_limit", "current_limited_start"),
                ] {
                    let inspection = inspections
                        .iter()
                        .find(|item| item["subject"] == id && item["test"] == test)
                        .unwrap();
                    assert_eq!(inspection["ends"], op["ends"]);
                    assert_eq!(inspection["stage"], stage);
                    assert!(inspection["limit"]["range"].is_null());
                }
            }
            assert!(wires.is_empty(), "{wires:?}");
            assert!(
                inspections
                    .iter()
                    .all(|item| !item["conditions"].as_str().unwrap().is_empty())
            );
            assert!(inspections.is_sorted_by_key(|item| item["stage"] == "current_limited_start"));
            // В том числе адреса прозвонки, развязки и измерения сигналов.
            for inspection in inspections {
                for end in inspection["ends"].as_array().unwrap() {
                    let component = end["component"].as_str().unwrap();
                    let pin = end["pin"].as_str().unwrap();
                    assert!(
                        source_parts[component]["nets"].get(pin).is_some(),
                        "{id}/{instance}: {end}"
                    );
                }
            }
            let bindings = raw["bindings"].as_array().unwrap();
            assert_eq!(sheet["bindings"].as_array().unwrap().len(), bindings.len());
            for (binding, raw) in sheet["bindings"].as_array().unwrap().iter().zip(bindings) {
                verify_ends(&binding["ends"], &raw["ends"]);
            }
            for (rail, data) in raw["rails"].as_table().unwrap() {
                for (test, stage) in [
                    ("positive_mark", "before_power"),
                    ("reverse_voltage", "current_limited_start"),
                ] {
                    assert!(
                        inspections.iter().any(|i| {
                            i["test"] == test
                                && i["stage"] == stage
                                && ends(&i["ends"]) == raw_ends(&data["ends"])
                        }),
                        "{id}/{instance}/{rail}: {test}"
                    );
                }
            }
        }
    }
    // Четыре листа одной установки и по одному у остальных. Сброс наблюдается в опыте Э3.
    assert_eq!((sheets, resets), (10, 1));
}

#[test]
fn удаление_провода_убирает_его_из_монтажа_и_прозвонки() {
    let (corpus, catalog) = корпус_с_каталогом();
    let path = corpus.join("missing_ground/fixed.toml");
    let original = fs::read_to_string(&path).unwrap();
    let compile = |source: &str| {
        sim_cli::electrical::parse_schematic(&path, source.as_bytes(), &catalog).unwrap()
    };
    let mut raw: toml::Value = toml::from_str(&original).unwrap();
    let before = compile(&original).assembly_sheet();
    let connections = raw["connections"].as_table_mut().unwrap();
    let return_path = connections.remove("return").unwrap();
    let signal = connections
        .get_mut("signal")
        .unwrap()
        .as_table_mut()
        .unwrap();
    assert_eq!(raw_ends(&signal.remove("after").unwrap()), ["return"]);
    let after = compile(&toml::to_string(&raw).unwrap());
    assert_eq!(
        after.check_static().status(StaticClass::MissingGround),
        Status::Fail
    );
    let after = after.assembly_sheet();

    // Исходная проводка корпуса состоит из возврата, затем сигнала.
    let points = |operation: &sim_process::schematic::AssemblyOperation| {
        operation.ends.each_ref().map(ToString::to_string)
    };
    let [return_op, signal_op] = before.operations.as_slice() else {
        panic!("нужны две монтажные операции");
    };
    assert_eq!(return_op.id, "return");
    assert_eq!(points(return_op).to_vec(), raw_ends(&return_path["ends"]));
    assert_eq!(signal_op.id, "signal");
    assert!(signal_op.after.contains("return"));
    let [remaining] = after.operations.as_slice() else {
        panic!("должен остаться сигнальный провод");
    };
    assert_eq!(remaining.id, signal_op.id);
    assert_eq!(remaining.ends, signal_op.ends);
    assert!(remaining.after.is_empty());

    for op in &before.operations {
        assert!(op.compatible);
        let staged = before
            .inspections
            .iter()
            .filter(|item| item.subject == op.id)
            .collect::<Vec<_>>();
        let [resistance, current] = staged.as_slice() else {
            panic!("у {} нужны замеры сопротивления и тока", op.id);
        };
        assert_eq!(resistance.ends, op.ends);
        assert_eq!(resistance.stage, Stage::BeforePower);
        assert!(matches!(resistance.expected, Expected::ResistanceOhm(_)));
        assert_eq!(current.ends, op.ends);
        assert_eq!(current.stage, Stage::CurrentLimitedStart);
        assert!(matches!(current.expected, Expected::CurrentLimitA(_)));
    }
    assert!(before.inspections.iter().any(|item| {
        matches!(item.expected, Expected::Continuity) && item.ends == return_op.ends
    }));
    assert!(after.inspections.iter().all(|item| {
        item.subject != return_op.id && !matches!(item.expected, Expected::Continuity)
    }));
}

fn ends(value: &Value) -> Vec<String> {
    value
        .as_array()
        .unwrap()
        .iter()
        .map(|point| {
            format!(
                "{}/{}",
                point["component"].as_str().unwrap(),
                point["pin"].as_str().unwrap()
            )
        })
        .collect()
}

fn raw_ends(value: &toml::Value) -> Vec<String> {
    value
        .as_array()
        .unwrap()
        .iter()
        .map(|point| point.as_str().unwrap().into())
        .collect()
}

fn verify_ends(points: &Value, raw: &toml::Value) {
    assert_eq!(ends(points), raw_ends(raw));
}

fn корпус_с_каталогом() -> (PathBuf, Catalog) {
    use sim_cli::electrical::load_catalog;

    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let corpus = root.join("devices/electrical/corpus");
    let base = load_catalog(&root.join("catalog/electrical")).unwrap();
    let fixture = load_catalog(&corpus.join("catalog")).unwrap();
    let mut parts = base.catalog.parts().clone();
    for (id, part) in fixture.catalog.parts() {
        assert!(parts.insert(id.clone(), part.clone()).is_none(), "{id}");
    }
    (corpus, Catalog::new(parts).unwrap())
}

#[test]
fn опасный_корпус_не_может_получить_положительный_вердикт() {
    use sim_task::{Outcome, verdict::SimulationVerdict};

    let (root, catalog) = корпус_с_каталогом();
    let manifest: toml::Value =
        toml::from_str(&fs::read_to_string(root.join("manifest.toml")).unwrap()).unwrap();
    let pairs = manifest["pairs"].as_table().unwrap();
    assert_eq!(pairs.len(), ElectricalClass::ALL.len());
    let stand = ElectricalStand::new("приёмка");
    for class in ElectricalClass::ALL {
        let code = class.code();
        let path = root.join(pairs[code][0].as_str().unwrap());
        let input = sim_cli::electrical::load_schematic(&path, &catalog).unwrap();
        let verdict = stand.assess(&input);
        let witness = verdict.findings.iter().find(|f| {
            f.code == code
                && f.outcome == Outcome::Fail
                && f.confirmed_hazard
                && f.electrical.is_some()
                && f.journal.is_some()
                && !f.reason.is_empty()
        });
        assert!(
            witness.is_some(),
            "{code}: нет подтверждённого нарушения своего класса"
        );
        assert!(
            !verdict
                .inapplicable
                .iter()
                .any(|item| item.code.as_deref() == Some(code))
        );
        assert_eq!(verdict.simulation, SimulationVerdict::Rejected, "{code}");
    }
}

#[test]
fn неприменимость_выводится_из_состава_и_записывается_в_вердикт() {
    use sim_cli::electrical::{SchematicInput, load_schematic};

    let (root, catalog) = корпус_с_каталогом();
    let stand = ElectricalStand::new("применимость");
    let no_load = load_schematic(&root.join("assembly_mismatch/fixed.toml"), &catalog).unwrap();
    let verdict = stand.assess(&no_load);
    for code in ["rail_sag", "startup_actuation"] {
        let item = verdict
            .inapplicable
            .iter()
            .find(|item| item.code.as_deref() == Some(code))
            .unwrap();
        assert_eq!(item.object.as_deref(), Some("блок"));
        assert!(item.reason.contains("в разрешённой схеме нет"));
        assert!(!item.sources.is_empty());
        assert!(!verdict.findings.iter().any(|finding| finding.code == code));
    }
    let powered =
        load_schematic(&root.join("ground_switch_backfeed/unsafe.toml"), &catalog).unwrap();
    let verdict = stand.assess(&powered);
    // Без привязки к плате нельзя подтвердить сброс и пуск, если в схеме есть управляемый ключ.
    assert!(powered.compiled.bindings().is_empty());
    for code in ["rail_sag", "startup_actuation"] {
        assert!(
            !verdict
                .inapplicable
                .iter()
                .any(|item| item.code.as_deref() == Some(code))
        );
        assert!(verdict.findings.iter().any(|finding| finding.code == code
            && finding.outcome == sim_task::Outcome::NotAnalyzed
            && !finding.reason.is_empty()));
    }
    let source = std::str::from_utf8(&powered.bytes).unwrap();
    let declared = format!("inapplicable = [\"rail_sag\"]\n{source}");
    let err = SchematicInput::parse(&powered.path, declared.into_bytes(), &catalog).unwrap_err();
    assert!(err.contains("inapplicable"), "{err}");
}

#[test]
fn прямое_соединение_источников_даёт_свидетельство_в_вердикте() {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../devices/electrical/corpus");
    let catalog = sim_cli::electrical::load_catalog(&root.join("catalog")).unwrap();
    let evaluate = |name: &str| {
        let path = root.join("source_conflict").join(name);
        let input = sim_cli::electrical::load_schematic(&path, &catalog.catalog).unwrap();
        sim_task::verdict::ElectricalEvidence::evaluate(
            "блок",
            &input.bytes,
            &input.compiled,
            &input.modes,
            Fingerprint::of(b"build"),
        )
        .unwrap()
    };
    let unsafe_case = evaluate("unsafe.toml");
    let conflict = unsafe_case
        .findings()
        .iter()
        .find(|finding| {
            finding.code == "source_conflict" && finding.outcome == sim_task::Outcome::Fail
        })
        .unwrap();
    // Независимый расчёт Э2: разность 5 − 4 В, два сопротивления по 1 Ом и два стыка по 0,1 Ом.
    let expected = (5.0 - 4.0) / (1.0 + 1.0 + 0.1 + 0.1);
    let tolerance = 1e-9; // Допуск округления линейного баланса.
    assert!((conflict.measurement.unwrap()[0] - expected).abs() < tolerance);
    assert!(conflict.journal.is_some());
    let fixed = evaluate("fixed.toml");
    assert!(
        fixed
            .findings()
            .iter()
            .any(|f| f.code == "source_conflict" && f.measurement.is_some())
    );
    assert!(
        fixed
            .findings()
            .iter()
            .all(|f| f.code != "source_conflict" || f.outcome != sim_task::Outcome::Fail)
    );
}

#[test]
fn незавершённый_живой_журнал_сохраняет_открытый_долг() {
    let manifest =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../devices/electrical/acceptance.toml");
    let settings: toml::Value = toml::from_str(&fs::read_to_string(&manifest).unwrap()).unwrap();
    let input = settings["assemblies"]
        .as_table()
        .unwrap()
        .values()
        .find(|case| case.get("profile").is_some())
        .unwrap();
    let base = manifest.parent().unwrap();
    let output = Command::new(env!("CARGO_BIN_EXE_sim-run"))
        .arg(base.join(input["world"].as_str().unwrap()))
        .arg("--electrical")
        .arg(base.join(input["profile"].as_str().unwrap()))
        .arg("--catalog")
        .arg(base.join(settings["catalog"].as_str().unwrap()))
        .arg("--duration")
        .arg(
            sim_signal::Time(input["horizon_ns"].as_integer().unwrap() as u64)
                .as_secs()
                .to_string(),
        )
        .args(["--mode", "step", "--json"])
        .output()
        .unwrap();
    assert_eq!(
        output.status.code(),
        Some(1),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let report: Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(report["status"], "interrupted");
    let electrical = &report["electrical_verdict"];
    assert_eq!(electrical["live"]["complete"], false);
    assert_eq!(electrical["simulation"], "NotProven");
    assert!(
        electrical["findings"]
            .as_array()
            .unwrap()
            .iter()
            .any(|finding| {
                finding["code"] == "execution"
                    && finding["outcome"] == "not_analyzed"
                    && finding["journal"] == electrical["live"]["journal"]
            })
    );
}

#[test]
fn ток_соединения_и_стабилизатора_сводится_в_один_класс() {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../devices/electrical/corpus");
    let catalog = sim_cli::electrical::load_catalog(&root.join("catalog")).unwrap();
    for (cause, source_ohm, load_ohm) in [
        ("connection_overcurrent", 1.0, 2.0),
        ("regulator_overload", 0.0, 3.0),
    ] {
        // Стенды Э2: источник 5 В с внутренним сопротивлением 1 Ом
        // или идеальный регулятор 5 В, по 0,1 Ом на каждом конце нагрузки.
        let expected = 5.0 / (source_ohm + load_ohm + 0.1 + 0.1);
        for (case, expected_failure) in [("unsafe.toml", true), ("fixed.toml", false)] {
            let path = root.join(cause).join(case);
            let input = sim_cli::electrical::load_schematic(&path, &catalog.catalog).unwrap();
            let report = sim_task::verdict::ElectricalEvidence::evaluate(
                "блок",
                &input.bytes,
                &input.compiled,
                &input.modes,
                Fingerprint::of(b"build"),
            )
            .unwrap();
            let rows: Vec<_> = report
                .findings()
                .iter()
                .filter(|f| f.code == cause && f.measurement.is_some())
                .collect();
            assert!(!rows.is_empty(), "{cause}/{case}");
            assert!(
                rows.iter()
                    .all(|f| f.key.class == sim_task::verdict::HazardClass::Overcurrent)
            );
            assert_eq!(
                rows.iter().any(|f| f.outcome == sim_task::Outcome::Fail),
                expected_failure,
                "{cause}/{case}"
            );
            if expected_failure {
                // Запас округления линейной схемы МНА.
                assert!(
                    rows.iter().any(|row| row.outcome == sim_task::Outcome::Fail
                        && (row.measurement.unwrap()[0] - expected).abs() < 1e-9
                        && row.journal.is_some()),
                    "{rows:?}"
                );
            }
        }
    }
}
