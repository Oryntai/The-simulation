use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::process::Command;

use serde::Deserialize;
use serde_json::Value;
use sim_task::verdict::{HazardClass, REGISTRY};

#[path = "stage6/thresholds.rs"]
mod thresholds;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Acceptance {
    version: u32,
    source: String,
    assemblies: BTreeMap<String, Assembly>,
    witnesses: Vec<Witness>,
    corpus: Corpus,
    reference: Vec<Reference>,
    named_failure: Vec<NamedFailure>,
    independent: Independent,
    restored_fault: Vec<RestoredFault>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RestoredFault {
    run: String,
    mode: String,
    fault: sim_task::Fault,
    at_ns: u64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Assembly {
    verdict: String,
    executed: u64,
    experiments: BTreeMap<String, u64>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    run: String,
    mode: String,
    samples: u64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct NamedFailure {
    run: String,
    mode: String,
    code: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Independent {
    run: String,
    mode: String,
    repeat: String,
    repeat_mode: String,
    measurements: Vec<String>,
    tolerance: f64,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Witness {
    assembly: String,
    class: String,
    object: String,
    mode: String,
    expected: Option<f64>,
    tolerance: f64,
    pair_assembly: String,
    pair_mode: Option<String>,
    pair_metric: Option<String>,
    pair_expected: Option<f64>,
    pair_tolerance: Option<f64>,
    reason_contains: Option<String>,
    mechanical: Option<MechanicalWitness>,
    paired_finding: Option<PairedFinding>,
    #[serde(default)]
    reject_only: bool,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct PairedFinding {
    bound: usize,
    reason_contains: String,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct MechanicalWitness {
    criterion: String,
    upper: f64,
    reason_contains: String,
    grid_reference: Option<[[f64; 3]; 2]>,
    momentum: Option<MomentumBalance>,
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct MomentumBalance {
    final_speed: String,
    incoming_impulses: [f64; 2],
    mass_kg: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Corpus {
    open: BTreeSet<String>,
    gaps: BTreeMap<String, Gap>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Gap {
    source: String,
    applicability: String,
    measurement: String,
    orientation_limit: Option<f64>,
}

fn data() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../devices")
}

fn acceptance() -> Acceptance {
    let expected: Acceptance =
        toml::from_str(&std::fs::read_to_string(data().join("stage6/acceptance.toml")).unwrap())
            .unwrap();
    assert_eq!(expected.version, 1);
    assert!(!expected.source.is_empty());
    expected
}

fn cli(file: &Path, args: &[&str], expected_exit: i32) -> Value {
    cli_at(
        Path::new(env!("CARGO_BIN_EXE_sim-run")),
        file,
        args,
        expected_exit,
    )
}

fn cli_at(binary: &Path, file: &Path, args: &[&str], expected_exit: i32) -> Value {
    serde_json::from_slice(&cli_output(binary, file, args, expected_exit)).unwrap()
}

fn cli_output(binary: &Path, file: &Path, args: &[&str], expected_exit: i32) -> Vec<u8> {
    let process = Command::new(binary)
        .arg("--campaign")
        .arg(file)
        .arg("--json")
        .args(args)
        .output()
        .expect("запуск штатного CLI");
    assert_eq!(
        process.status.code(),
        Some(expected_exit),
        "{} {}",
        String::from_utf8_lossy(&process.stdout),
        String::from_utf8_lossy(&process.stderr)
    );
    process.stdout
}

fn rows(v: &Value) -> &[Value] {
    v.as_array().expect("массив отчёта")
}

fn map(v: &Value) -> &serde_json::Map<String, Value> {
    v.as_object().expect("таблица отчёта")
}

fn all_findings(assembly: &Value) -> impl Iterator<Item = &Value> {
    map(&assembly["runs"])
        .values()
        .flat_map(|run| rows(&run["findings"]))
}

fn metric<'a>(case: &'a Value, name: &str) -> &'a Value {
    rows(&case["convergence"]["metrics"])
        .iter()
        .find(|row| row["name"] == name)
        .unwrap_or_else(|| panic!("нет метрики «{name}»"))
}

fn process_case<'a>(assembly: &'a Value, mode: &str) -> Result<&'a Value, String> {
    map(&assembly["runs"])
        .values()
        .find_map(|run| run["detail"]["profile"]["cases"].get(mode))
        .ok_or_else(|| format!("нет процессного опыта «{mode}»"))
}

fn grid_values(case: &Value, criterion: &str) -> Result<[f64; 3], String> {
    let values: Vec<_> = rows(&metric(case, criterion)["values"])
        .iter()
        .map(|v| {
            let bounds = v.as_array()?;
            let [low, high] = bounds.as_slice() else {
                return None;
            };
            let value = low.as_f64()?;
            (value.is_finite() && low == high).then_some(value)
        })
        .collect::<Option<_>>()
        .ok_or_else(|| format!("нет численных отсчётов «{criterion}»"))?;
    values
        .try_into()
        .map_err(|_| format!("нет трёх сеток «{criterion}»"))
}

fn verify_mechanical(
    subject: &Value,
    pair: &Value,
    proof: &Witness,
    witness: &MechanicalWitness,
) -> Result<(), String> {
    if pair["verdict"] == "Rejected" {
        return Err(format!("парная сборка отклонена для {}", proof.class));
    }
    let cases = [
        process_case(subject, &proof.mode)?,
        process_case(pair, &proof.mode)?,
    ];
    for (index, case) in cases.into_iter().enumerate() {
        let adverse = index == 0;
        let trials = rows(&case["convergence"]["trials"]);
        let proven = trials.len() == 3
            && trials.iter().all(|trial| {
                trial["completed"] == true
                    && trial["steps"].as_u64().is_some_and(|steps| steps > 0)
                    && trial["journal"].as_u64().is_some_and(|stamp| stamp != 0)
            });
        let limit_outcome = if adverse { "fail" } else { "pass" };
        if !proven
            || (case["outcome"] == "fail") != adverse
            || metric(case, &witness.criterion)["outcome"] != limit_outcome
        {
            return Err(format!(
                "нет живого исхода {limit_outcome} для {}",
                proof.class
            ));
        }
        if !adverse
            && all_findings(pair).any(|f| {
                f["key"]["class"] == proof.class
                    && f["key"]["object"] == proof.object
                    && f["key"]["run"]["mode"] == proof.mode
                    && f["outcome"] == "fail"
            })
        {
            return Err(format!("свидетель сработал на паре для {}", proof.class));
        }
        let actual = grid_values(case, &witness.criterion)?;
        let reference = if let Some(balance) = &witness.momentum {
            // Баланс поступательного импульса по скорости тела из другого канала.
            // Направление +X от опоры, входящий импульс приложен вдоль −X.
            grid_values(case, &balance.final_speed)?
                .map(|v| balance.incoming_impulses[index] + balance.mass_kg * v)
        } else if let Some(reference) = witness.grid_reference {
            reference[index]
        } else {
            let expected = if adverse {
                proof.expected
            } else {
                proof.pair_expected
            };
            [expected.ok_or("нет независимого ожидаемого значения")?; 3]
        };
        if actual.into_iter().zip(reference).any(|(a, b)| {
            !b.is_finite() || (a - b).abs() > proof.tolerance || (a > witness.upper) != adverse
        }) {
            return Err(format!(
                "сверка {} / {limit_outcome}: {actual:?} и {reference:?}",
                proof.class
            ));
        }
    }
    Ok(())
}

fn verify_witness(report: &Value, proof: &Witness) -> Result<(), String> {
    let subject = &report["assemblies"][&proof.assembly];
    if subject["verdict"] != "Rejected" || subject["coverage"]["executed"] == 0 {
        return Err(format!("сборка не дала живой отказ для {}", proof.class));
    }
    let find = |f: &&Value| {
        f["key"]["class"] == proof.class
            && f["key"]["object"] == proof.object
            && f["key"]["run"]["mode"] == proof.mode
    };
    let numerical = |f: &Value| {
        let Some(bounds) = f["measurement"].as_array() else {
            return false;
        };
        let Some(event) = f["event_ns"].as_u64() else {
            return false;
        };
        if !f["key"]["window_ns"]
            .as_array()
            .is_some_and(|w| w[0].as_u64().unwrap() <= event && event <= w[1].as_u64().unwrap())
            || !f["journal"].as_u64().is_some_and(|stamp| stamp != 0)
            || rows(&f["limit_sources"]).is_empty()
            || rows(&f["measurement_sources"]).is_empty()
        {
            return false;
        }
        let lo = bounds[0].as_f64().unwrap();
        let hi = bounds[1].as_f64().unwrap();
        if !lo.is_finite()
            || !hi.is_finite()
            || lo > hi
            || proof.reason_contains.as_ref().is_some_and(|part| {
                !f["reason"]
                    .as_str()
                    .is_some_and(|reason| reason.contains(part))
            })
        {
            return false;
        }
        if proof.reject_only
            && !f["limit"]["upper"][1]
                .as_f64()
                .is_some_and(|limit| lo > limit)
        {
            return false;
        }
        if let Some(paired) = &proof.paired_finding {
            return paired.bound < bounds.len()
                && f["reason"]
                    .as_str()
                    .is_some_and(|reason| reason.contains(&paired.reason_contains))
                && proof.expected.is_some_and(|expected| {
                    (bounds[paired.bound].as_f64().unwrap() - expected).abs() <= proof.tolerance
                })
                && (f["limit"]["upper"][1].as_f64().is_some_and(|max| hi > max)
                    || f["limit"]["lower"][0].as_f64().is_some_and(|min| lo < min));
        }
        if let Some(witness) = &proof.mechanical {
            return lo.is_finite()
                && hi.is_finite()
                && lo > witness.upper
                && f["limit"]["upper"] == serde_json::json!([witness.upper, witness.upper])
                && f["code"] == witness.criterion
                && f["reason"]
                    .as_str()
                    .is_some_and(|reason| reason.contains(&witness.reason_contains));
        }
        let exceeded = f["limit"]["upper"][1].as_f64().is_some_and(|max| hi > max)
            || f["limit"]["lower"][0].as_f64().is_some_and(|min| lo < min);
        exceeded
            && proof.expected.is_some_and(|expected| {
                [lo, hi]
                    .iter()
                    .any(|m| (m - expected).abs() <= proof.tolerance)
            })
    };
    let Some(adverse) = all_findings(subject)
        .filter(find)
        .find(|f| f["outcome"] == "fail" && f["confirmed_hazard"] == true && numerical(f))
    else {
        return Err(format!("нет численного свидетеля {}", proof.class));
    };
    let within_limit = |measurement: &Value| {
        let Some([lo, hi]) = measurement.as_array().map(Vec::as_slice) else {
            return false;
        };
        lo.as_f64().zip(hi.as_f64()).is_some_and(|(lo, hi)| {
            lo.is_finite()
                && hi.is_finite()
                && lo <= hi
                && adverse["limit"]["lower"][1]
                    .as_f64()
                    .is_none_or(|min| lo >= min)
                && adverse["limit"]["upper"][0]
                    .as_f64()
                    .is_none_or(|max| hi <= max)
        })
    };
    let pair = &report["assemblies"][&proof.pair_assembly];
    if proof.reject_only && pair["verdict"] != "NotProven" {
        return Err(format!("парный опыт закрыл готовность для {}", proof.class));
    }
    let pair_tolerance = proof.pair_tolerance.unwrap_or(proof.tolerance);
    if let Some(witness) = &proof.mechanical {
        return verify_mechanical(subject, pair, proof, witness);
    }
    if let Some(paired) = &proof.paired_finding {
        let paired_mode = proof.pair_mode.as_deref().unwrap_or(&proof.mode);
        let find_pair = |f: &&Value| {
            f["key"]["class"] == proof.class
                && f["key"]["object"] == proof.object
                && f["key"]["run"]["mode"] == paired_mode
        };
        let found = all_findings(pair).filter(find_pair).any(|f| {
            f["outcome"]
                == if proof.reject_only {
                    "not_analyzed"
                } else {
                    "pass"
                }
                && f["confirmed_hazard"] == false
                && within_limit(&f["measurement"])
                && (!proof.reject_only
                    || f["measurement"][1]
                        .as_f64()
                        .zip(f["limit"]["upper"][0].as_f64())
                        // Нулевой предел запрещает событие. Точный нуль
                        // в паре проверяет его отсутствие.
                        .is_some_and(|(hi, limit)| hi < limit || (hi == 0.0 && limit == 0.0)))
                && f["reason"]
                    .as_str()
                    .is_some_and(|reason| reason.contains(&paired.reason_contains))
                && f["journal"].as_u64().is_some_and(|stamp| stamp != 0)
                && proof.pair_expected.is_some_and(|expected| {
                    f["measurement"][paired.bound]
                        .as_f64()
                        .is_some_and(|m| (m - expected).abs() <= pair_tolerance)
                })
        });
        if found
            && !all_findings(pair)
                .filter(find_pair)
                .any(|f| f["outcome"] == "fail")
        {
            return Ok(());
        }
        return Err(format!(
            "нет парного численного подтверждения {}",
            proof.class
        ));
    }
    if let Some((mode, name, expected)) = proof
        .pair_mode
        .as_ref()
        .zip(proof.pair_metric.as_ref())
        .zip(proof.pair_expected)
        .map(|((m, n), e)| (m, n, e))
    {
        let run = map(&pair["runs"])
            .values()
            .find(|r| !r["detail"]["profile"]["cases"][mode].is_null())
            .ok_or("нет парного опыта")?;
        let case = &run["detail"]["profile"]["cases"][mode];
        let metric = metric(case, name);
        let values = rows(&metric["values"]);
        let trials = rows(&case["convergence"]["trials"]);
        if metric["outcome"] != "pass"
            || metric["unit"] != adverse["unit"]
            || !within_limit(&metric["interval"])
            || values.len() != trials.len()
            || trials.len() < 3
            || !trials.iter().all(|trial| {
                trial["completed"] == true
                    && trial["journal"]
                        .as_u64()
                        .is_some_and(|journal| journal != 0)
            })
            || !values.iter().all(|range| {
                within_limit(range)
                    && rows(range).iter().all(|v| {
                        v.as_f64()
                            .is_some_and(|v| (v - expected).abs() <= pair_tolerance)
                    })
            })
        {
            return Err(format!("не сходится парный опыт для {}", proof.class));
        }
    } else if !all_findings(pair)
        .filter(find)
        .any(|f| f["outcome"] == "pass" && within_limit(&f["measurement"]))
    {
        return Err(format!(
            "нет соблюдения предела в парном опыте {}",
            proof.class
        ));
    }
    Ok(())
}

fn required() -> BTreeSet<String> {
    REGISTRY
        .iter()
        .filter(|spec| {
            !matches!(
                spec.class,
                HazardClass::TaskCriterion | HazardClass::Execution | HazardClass::Inventory
            )
        })
        .map(|spec| spec.id.into())
        .collect()
}

fn covered(report: &Value, witnesses: &[Witness]) -> Result<BTreeSet<String>, String> {
    let mut classes = BTreeSet::new();
    for proof in witnesses {
        if !required().contains(&proof.class) || !classes.insert(proof.class.clone()) {
            return Err(format!("повтор или неизвестный класс {}", proof.class));
        }
        verify_witness(report, proof)?;
    }
    Ok(classes)
}

fn check_corpus(
    report: &Value,
    witnesses: &[Witness],
    documented_gaps: &BTreeSet<String>,
) -> Result<usize, String> {
    let confirmed = covered(report, witnesses)?;
    let missing: BTreeSet<_> = required().difference(&confirmed).cloned().collect();
    if &missing != documented_gaps {
        return Err(format!("покрытие корпуса изменилось: открыты {missing:?}"));
    }
    Ok(confirmed.len())
}

struct Temp(PathBuf);
impl Drop for Temp {
    fn drop(&mut self) {
        std::fs::remove_dir_all(&self.0).expect("удаление контрольных точек");
    }
}

#[test]
fn корпус_и_воспроизведение() {
    let expected = acceptance();
    assert_eq!(
        expected
            .corpus
            .gaps
            .keys()
            .cloned()
            .collect::<BTreeSet<_>>(),
        expected.corpus.open
    );
    for gap in expected.corpus.gaps.values() {
        assert!(
            gap.source.starts_with("https://")
                && !gap.applicability.is_empty()
                && !gap.measurement.is_empty()
        );
        assert!(
            gap.orientation_limit
                .is_none_or(|v| v.is_finite() && v > 0.0)
        );
    }
    let manifest = data().join("stage6/corpus.toml");
    let temp = Temp(std::env::temp_dir().join(format!("campaign-stage6-{}", std::process::id())));
    std::fs::create_dir(&temp.0).unwrap();
    // Другая команда Cargo может перелинковать штатный путь между двумя запусками.
    let binary = temp.0.join("sim-run");
    std::fs::copy(env!("CARGO_BIN_EXE_sim-run"), &binary).unwrap();
    let checkpoint = temp.0.join("checkpoint");
    let run = |args: &[&str], code| cli_at(&binary, &manifest, args, code);
    let checkpoint_arg = checkpoint.to_str().unwrap();
    let first = run(&["--checkpoint", checkpoint_arg], 0);
    assert_eq!(first["status"], "completed");
    assert_eq!(first["resumed"], 0);
    assert_eq!(
        first["assemblies"]["under-travel"]["verdict"],
        "PassedSimulation"
    );
    assert_eq!(
        first["assemblies"]["under-travel"]["readiness"],
        "BenchOnly"
    );
    let second = run(&["--checkpoint", checkpoint_arg], 0);
    assert!(second["resumed"].as_u64().unwrap() > 0);
    assert_eq!(first["assemblies"], second["assemblies"]);
    assert_eq!(first["worst_run_key"], second["worst_run_key"]);
    let key = first["worst_run_key"].as_str().unwrap();
    let repeat = run(&["--repeat", key], 0);
    assert_eq!(repeat["worst_run_key"], key);
    let id = key.split('/').next().unwrap();
    assert_eq!(
        first["assemblies"][id]["runs"][key],
        repeat["assemblies"][id]["runs"][key]
    );
    for (id, assembly) in map(&first["assemblies"]) {
        assert_eq!(
            assembly["coverage"]["required"], repeat["assemblies"][id]["coverage"]["required"],
            "{id}: адресный повтор изменил размер матрицы"
        );
    }
    assert!(
        repeat["assemblies"][id]["coverage"]["omitted"]
            .as_u64()
            .unwrap()
            > 0
            || map(&repeat["assemblies"])
                .values()
                .any(|a| a["coverage"]["omitted"].as_u64().unwrap() > 0)
    );

    // Численно проверяем, что изменился начальный параметр и изменился физический результат.
    let runs = map(&first["assemblies"]["over-travel"]["runs"]);
    assert_eq!(runs.len(), 2);
    let mut end: Vec<_> = runs
        .values()
        .map(|run| {
            let detail = &run["detail"];
            assert_eq!(detail["snapshot"]["matched"], true);
            assert_eq!(detail["steps"], 4);
            assert_eq!(detail["summary"]["observed_through_ns"], 4_000_000);
            let applied = detail["summary"]["installation"]["values"]["начальная координата"]
                .as_f64()
                .unwrap();
            let measured = detail["summary"]["metrics"]["конечный ход"]["value"]
                .as_f64()
                .unwrap();
            (applied, measured)
        })
        .collect();
    end.sort_by(|a, b| a.0.total_cmp(&b.0));
    assert_eq!([end[0].0, end[1].0], [0.0, 0.000001]);
    assert!(((end[1].1 - end[0].1) - 0.000001).abs() < 1e-14);
    let assembly = &first["assemblies"]["over-travel"];
    let worst = assembly["worst_run_key"].as_str().unwrap();
    assert_eq!(
        assembly["runs"][worst]["detail"]["summary"]["metrics"]["конечный ход"]["value"],
        serde_json::json!(end[1].1)
    );

    let supported = check_corpus(&first, &expected.witnesses, &expected.corpus.open).unwrap();
    assert_eq!(supported, required().len());
    let imported: BTreeSet<_> = expected.witnesses.iter().filter_map(|w| {
        all_findings(&first["assemblies"][&w.assembly]).any(|f| {
            f["key"]["class"] == w.class && f["confirmed_hazard"] == true
                && rows(&f["measurement_sources"]).iter().any(|source| {
                    source.as_str().is_some_and(|s| s.starts_with("Синтетический ряд:"))
                })
        }).then_some(w.class.as_str())
    }).collect();
    assert_eq!(imported, BTreeSet::from(["tissue_load", "ingress"]));
    println!(
        "Корпус: требуется {}, численных пар {}, из них {} по синтетическому входному ряду: {:?}",
        required().len(),
        supported,
        imported.len(),
        imported
    );
    let mut absent = expected.witnesses.clone();
    absent.remove(0);
    assert!(check_corpus(&first, &absent, &expected.corpus.open).is_err());
    let mut wrong = expected.witnesses.clone();
    wrong[0].class = "fatigue".into();
    assert!(covered(&first, &wrong).is_err());
    let mut unproven = expected.witnesses.clone();
    unproven[0].assembly = "estimate".into();
    assert!(covered(&first, &unproven).is_err());
    for proof in &expected.witnesses {
        let mut masked = first.clone();
        for run in masked["assemblies"][&proof.assembly]["runs"]
            .as_object_mut()
            .unwrap()
            .values_mut()
        {
            run["findings"].as_array_mut().unwrap().retain(|finding| {
                finding["key"]["class"] != proof.class || finding["confirmed_hazard"] != true
            });
        }
        assert!(
            verify_witness(&masked, proof).is_err(),
            "выключенная проверка {} не замечена",
            proof.class
        );
    }
    let dir = std::fs::read_dir(&checkpoint)
        .unwrap()
        .next()
        .unwrap()
        .unwrap()
        .path();
    let checkpoint_file = std::fs::read_dir(dir)
        .unwrap()
        .next()
        .unwrap()
        .unwrap()
        .path();
    std::fs::write(checkpoint_file, b"{").unwrap();
    assert_eq!(run(&["--checkpoint", checkpoint_arg], 2)["status"], "error");
    verify_unconfirmed_inputs(&first, &expected.witnesses, &binary, &temp.0);
    verify_disabled_controls(&first, &expected.witnesses, &binary, &temp.0);
    verify_disabled_conductor(&first, &expected.witnesses, &binary, &temp.0);
    verify_disabled_screens(&expected.witnesses, &binary, &temp.0);
    verify_dc_limits(&first, &expected.witnesses, &binary, &temp.0);
    verify_measured_results(&first, &expected.witnesses, &binary, &temp.0);
}

fn verify_disabled_screens(witnesses: &[Witness], binary: &Path, temp: &Path) {
    let fixtures = data().join("stage6/corpus/screen");
    for proof in witnesses.iter().filter(|proof| {
        proof.reject_only
            && matches!(
                proof.class.as_str(),
                "burns" | "electric_shock" | "measurement"
            )
    }) {
        let source_file = if proof.class == "measurement" {
            "curve"
        } else {
            &proof.mode
        };
        let mut case: toml::Value = toml::from_str(
            &std::fs::read_to_string(fixtures.join(format!("{source_file}.toml"))).unwrap(),
        )
        .unwrap();
        let absolute = |value: &mut toml::Value| {
            *value = toml::Value::String(
                fixtures
                    .join(value.as_str().unwrap())
                    .canonicalize()
                    .unwrap()
                    .to_str()
                    .unwrap()
                    .into(),
            );
        };
        absolute(&mut case["world"]);
        if let Some(contact) = case.get_mut("contacts") {
            absolute(&mut contact["table"]);
        }
        if let Some(reference) = case.get_mut("reference") {
            absolute(&mut reference["file"]);
        }
        let write = |name: &str, text: &str| std::fs::write(temp.join(name), text).unwrap();
        write(
            "muted-screen-task.toml",
            "version = 1\nprocess_profile = 'muted-screen-profile.toml'\n",
        );
        write(
            "muted-screen-profile.toml",
            &format!(
                "version = 1\nsource = 'Отключение применимости'\n[cases]\n{} = 'muted-screen-case.toml'\n",
                proof.mode
            ),
        );
        write(
            "muted-screen-manifest.toml",
            &format!(
                "version = 1\n[assemblies.{}]\nscope = 'Отключение применимости'\n[assemblies.{}.experiments.test]\nkind = 'task'\nworld = {}\ntask = 'muted-screen-task.toml'\n",
                proof.assembly, proof.assembly, case["world"]
            ),
        );
        match proof.class.as_str() {
            "burns" => case["screen"]["condition"]["dry_bare_skin"] = false.into(),
            "electric_shock" => case["screen"]["condition"]["ripple_free"] = false.into(),
            "measurement" => case["screen"]["condition"]["synthetic"] = false.into(),
            _ => panic!("неизвестное условие применимости"),
        }
        write("muted-screen-case.toml", &toml::to_string(&case).unwrap());
        let report = cli_at(binary, &temp.join("muted-screen-manifest.toml"), &[], 0);
        let assembly = &report["assemblies"][&proof.assembly];
        assert_eq!(assembly["coverage"]["executed"], 1);
        assert_eq!(assembly["verdict"], "NotProven");
        assert!(all_findings(assembly).any(|finding| {
            finding["key"]["class"] == proof.class
                && finding["outcome"] == "not_analyzed"
                && finding["measurement"].is_null()
                && finding["confirmed_hazard"] == false
        }));
        assert!(verify_witness(&report, proof).is_err());
        if proof.class == "measurement" {
            write(
                "nonfinite.csv",
                "seconds,height\n0,0.02\n0.001,NaN\n0.002,0.02\n0.003,0.02\n",
            );
            case["reference"]["file"] = "nonfinite.csv".into();
            write("muted-screen-case.toml", &toml::to_string(&case).unwrap());
            let invalid = cli_at(binary, &temp.join("muted-screen-manifest.toml"), &[], 2);
            assert_eq!(invalid["status"], "error");
            assert!(invalid.to_string().contains("конечными отсчётами"));
        }
    }
}

fn add_process_specimen(
    temp: &Path,
    manifest: &mut String,
    assembly: &str,
    mode: &str,
    case: &toml::Value,
) {
    let write = |suffix: &str, text: &str| {
        std::fs::write(temp.join(format!("{assembly}-{suffix}.toml")), text).unwrap();
    };
    write("case", &toml::to_string(case).unwrap());
    write(
        "task",
        &format!("version = 1\nprocess_profile = '{assembly}-profile.toml'\n"),
    );
    write(
        "profile",
        &format!(
            "version = 1\nsource = 'Проверка применимости'\n[cases]\n{mode} = '{assembly}-case.toml'\n"
        ),
    );
    manifest.push_str(&format!(
        "\n[assemblies.{assembly}]\nscope = 'Проверка применимости'\n[assemblies.{assembly}.experiments.test]\nkind = 'task'\nworld = {}\ntask = '{assembly}-task.toml'\n",
        case["world"]
    ));
}

fn verify_dc_limits(first: &Value, witnesses: &[Witness], binary: &Path, temp: &Path) {
    let mut proof = witnesses
        .iter()
        .find(|w| w.class == "electric_shock")
        .unwrap()
        .clone();
    proof.assembly = "dc-border".into();
    proof.mode = "dc-border".into();
    proof.expected = Some(2.005002 / 1002.0);
    verify_witness(first, &proof).unwrap();

    let fixtures = data().join("stage6/corpus/screen");
    let mut base: toml::Value =
        toml::from_str(&std::fs::read_to_string(fixtures.join("dc.toml")).unwrap()).unwrap();
    base["world"] = fixtures
        .join(base["world"].as_str().unwrap())
        .canonicalize()
        .unwrap()
        .to_str()
        .unwrap()
        .into();
    let mut manifest = "version = 1\n".to_string();
    // 35 В через 100 кОм отделяет превышение напряжения от превышения тока.
    // −12 В меняет направление при прежнем пути, 4 мс меняют только выдержку.
    let specimens = [
        (
            "dc-dry",
            35.0,
            100000.0,
            "dry_small_area",
            false,
            true,
            false,
            false,
            60.0,
        ),
        (
            "dc-wet", 35.0, 100000.0, "wet", false, true, false, true, 30.0,
        ),
        (
            "dc-body",
            35.0,
            100000.0,
            "body_worn",
            false,
            true,
            false,
            true,
            30.0,
        ),
        (
            "dc-elv",
            120.0,
            100000.0,
            "dry_small_area",
            false,
            true,
            false,
            true,
            60.0,
        ),
        (
            "dc-12v",
            12.0,
            1000.0,
            "dry_small_area",
            false,
            false,
            true,
            false,
            60.0,
        ),
        (
            "dc-inverse",
            -12.0,
            1000.0,
            "dry_small_area",
            false,
            false,
            false,
            false,
            60.0,
        ),
        (
            "dc-rebound",
            -12.0,
            1000.0,
            "dry_small_area",
            true,
            false,
            true,
            false,
            60.0,
        ),
        (
            "dc-short",
            12.0,
            1000.0,
            "dry_small_area",
            false,
            true,
            false,
            false,
            60.0,
        ),
    ];
    for (id, supply, load, exposure, reverse, short, _, _, _) in specimens {
        let mut case = base.clone();
        let model = &mut case["models"]["контакт"];
        model["supply_v"]["value"] = supply.into();
        model["load_ohm"]["value"] = load.into();
        case["screen"]["condition"]["exposure"] = exposure.into();
        if reverse {
            case["screen"]["condition"]["path_when_positive"]
                .as_array_mut()
                .unwrap()
                .reverse();
        }
        if short {
            // При физике 1 кГц окно занимает четыре целых шага после первого такта.
            case["window_ns"][1] = 5_000_000_i64.into();
        }
        add_process_specimen(temp, &mut manifest, id, id, &case);
    }
    let file = temp.join("dc-conditions.toml");
    std::fs::write(&file, manifest).unwrap();
    let report = cli_at(binary, &file, &[], 0);
    for (id, supply, load, _, _, _, current_fail, voltage_fail, voltage_limit) in specimens {
        let assembly = &report["assemblies"][id];
        let findings: Vec<_> = all_findings(assembly)
            .filter(|f| f["key"]["class"] == "electric_shock")
            .collect();
        assert_eq!(
            assembly["verdict"],
            if current_fail || voltage_fail {
                "Rejected"
            } else {
                "NotProven"
            }
        );
        for (unit, expected, limit, fail) in [
            ("A", f64::abs(supply) / (load + 2.0), 0.002, current_fail),
            ("V", f64::abs(supply), voltage_limit, voltage_fail),
        ] {
            let f = findings.iter().find(|f| f["unit"] == unit).unwrap();
            assert!(
                (f["measurement"][0].as_f64().unwrap() - expected).abs() < 1e-10,
                "{id}"
            );
            assert_eq!(f["limit"]["upper"][0].as_f64().unwrap(), limit, "{id}");
            assert_eq!(f["confirmed_hazard"], fail, "{id}");
            assert_eq!(
                f["outcome"],
                if fail { "fail" } else { "not_analyzed" },
                "{id}"
            );
        }
    }
}

fn verify_measured_results(first: &Value, witnesses: &[Witness], binary: &Path, temp: &Path) {
    let fixtures = data().join("stage6/corpus/screen");
    let mut case: toml::Value =
        toml::from_str(&std::fs::read_to_string(fixtures.join("measured.toml")).unwrap()).unwrap();
    case["world"] = fixtures
        .join(case["world"].as_str().unwrap())
        .canonicalize()
        .unwrap()
        .to_str()
        .unwrap()
        .into();
    case.as_table_mut()
        .unwrap()
        .remove("measured_result")
        .unwrap();
    let mut manifest = "version = 1\n".to_string();
    let mut measured = Vec::new();
    let mut borders = Vec::new();
    for (class, secondary, value, event) in [
        ("tissue_load", "пик контактной силы", 6.0, 2_000_000),
        ("ingress", "отказ изоляции", 1.0, 4_000_000),
    ] {
        let proof = witnesses.iter().find(|w| w.class == class).unwrap();
        let assembly = &first["assemblies"][&proof.assembly];
        let secondary = all_findings(assembly)
            .find(|f| f["code"] == secondary)
            .unwrap();
        assert_eq!(secondary["measurement"], serde_json::json!([value, value]));
        assert_eq!(secondary["confirmed_hazard"], class == "ingress");
        assert!(all_findings(assembly).any(|f| {
            f["key"]["class"] == class
                && f["outcome"] == "fail"
                && f["event_ns"] == event
                && rows(&f["measurement_sources"])
                    .iter()
                    .any(|s| s.as_str().unwrap().starts_with("Синтетический ряд:"))
        }));
        let id = format!("{}-disabled", proof.assembly);
        add_process_specimen(temp, &mut manifest, &id, &proof.mode, &case);
        measured.push((id, proof));

        let mut border = proof.clone();
        for outside in [true, false] {
            let label = if outside { "outside" } else { "inside" };
            let id = format!("{}-{label}", proof.assembly);
            let mut row: toml::Value = toml::from_str(&std::fs::read_to_string(
                fixtures.join(format!("{}-row.toml", proof.assembly)),
            ).unwrap()).unwrap();
            row["mode"] = id.clone().into();
            let sample = &mut row["check"]["samples"][1];
            let value = if class == "tissue_load" {
                // Разрешение синтетической карты 1 Па. Интеграл давления даёт показание весов.
                let pressure = if outside { 80_001.0 } else { 79_999.0 };
                let cells = &mut sample["pressure_map"];
                cells[0]["pressure_pa"]["value"] = 0.0.into();
                cells[1]["pressure_pa"]["value"] = pressure.into();
                sample["force_n"]["value"] = (pressure * 0.00005).into();
                pressure
            } else {
                // При нулевом пределе минимальная проба стенда 1 мкл = 10⁻⁹ м³.
                // При той же воде и исправной изоляции меняется только зона.
                let protected = if outside { 1e-9 } else { 0.0 };
                sample["locations"][0]["volume_m3"]["value"] = protected.into();
                sample["locations"][1]["volume_m3"]["value"] = (1e-9 - protected).into();
                sample["insulation"] = "intact".into();
                protected
            };
            row["origin"]["reference"] = "Пограничная пара с известной нагрузкой и объёмом".into();
            let name = format!("{id}-row.toml");
            std::fs::write(temp.join(&name), toml::to_string(&row).unwrap()).unwrap();
            let mut specimen = case.clone();
            specimen.as_table_mut().unwrap().insert("measured_result".into(), name.into());
            add_process_specimen(temp, &mut manifest, &id, &id, &specimen);
            if outside {
                border.assembly = id.clone();
                border.mode = id;
                border.expected = Some(value);
            } else {
                border.pair_assembly = id.clone();
                border.pair_mode = Some(id);
                border.pair_expected = Some(value);
            }
        }
        borders.push(border);
    }
    let file = temp.join("measured-disabled.toml");
    std::fs::write(&file, manifest).unwrap();
    let report = cli_at(binary, &file, &[], 0);
    for proof in &borders {
        verify_witness(&report, proof).unwrap_or_else(|e| panic!("{e}"));
        println!("{}: {:?} / {:?}", proof.class, proof.expected, proof.pair_expected);
    }
    for (id, proof) in measured {
        let disabled = &report["assemblies"][&id];
        assert_eq!(disabled["coverage"]["executed"], 1);
        assert_eq!(disabled["verdict"], "NotProven");
        assert!(
            !all_findings(disabled)
                .any(|f| f["key"]["class"] == proof.class && f["confirmed_hazard"] == true)
        );
        let mut masked = first.clone();
        masked["assemblies"][&proof.assembly] = disabled.clone();
        assert!(verify_witness(&masked, proof).is_err());
    }
}

fn verify_disabled_conductor(report: &Value, witnesses: &[Witness], binary: &Path, temp: &Path) {
    let data = data();
    let proof = witnesses.iter().find(|w| w.class == "wiring_heat").unwrap();
    let literal = |path: &Path| toml::Value::String(path.to_str().unwrap().into());
    let mut specimen: toml::Table = toml::from_str(
        &std::fs::read_to_string(data.join("stage6/corpus/conductor-hot.toml")).unwrap(),
    )
    .unwrap();
    specimen.remove("hazard_witnesses").unwrap();
    let world = literal(&data.join("stage4-processes/world-energy.toml"));
    specimen.insert("world".into(), world.clone());
    let write = |name: &str, text: &str| std::fs::write(temp.join(name), text).unwrap();
    write("conductor-muted.toml", &toml::to_string(&specimen).unwrap());
    write(
        "conductor-muted-task.toml",
        "version = 1\nprocess_profile = 'conductor-muted-profile.toml'\n",
    );
    write(
        "conductor-muted-profile.toml",
        "version = 1\nsource = 'Парный опыт отключения свидетельства'\n[cases]\nconductor-hot = 'conductor-muted.toml'\n",
    );
    write(
        "conductor-muted-campaign.toml",
        &format!(
            "version = 1\n[assemblies.processes]\nscope = 'Тепловой опыт с выключенным свидетелем'\n\
         [assemblies.processes.experiments.heat]\nkind = 'task'\nworld = {world}\ntask = 'conductor-muted-task.toml'\n"
        ),
    );
    let muted = cli_at(binary, &temp.join("conductor-muted-campaign.toml"), &[], 0);
    let assembly = &muted["assemblies"]["processes"];
    assert_eq!(assembly["verdict"], "Rejected");
    assert_eq!(assembly["coverage"]["executed"], 1);
    let case = |r: &Value| {
        let case = process_case(&r["assemblies"]["processes"], &proof.mode).unwrap();
        grid_values(case, proof.pair_metric.as_deref().unwrap()).unwrap()
    };
    assert_eq!(case(report), case(&muted));
    assert_eq!(
        verify_witness(&muted, proof).unwrap_err(),
        "нет численного свидетеля wiring_heat"
    );
}

fn verify_disabled_controls(report: &Value, witnesses: &[Witness], binary: &Path, temp: &Path) {
    let fixtures = data().join("stage6/corpus/control");
    let read = |path: &Path| -> toml::Value {
        toml::from_str(&std::fs::read_to_string(path).unwrap()).unwrap()
    };
    let literal = |path: &Path| toml::Value::String(path.to_str().unwrap().into());
    let save = |name: &str, value: &toml::Value| {
        std::fs::write(temp.join(name), toml::to_string(value).unwrap()).unwrap();
    };
    let manifest = temp.join("control-muted.toml");
    for (id, task_name) in [
        ("control-unsafe", "unsafe-task.toml"),
        ("battery-depleted", "battery-depleted-task.toml"),
    ] {
        let mut task = read(&fixtures.join(task_name));
        let mut profile = read(&fixtures.join(task["profile"].as_str().unwrap()));
        let selected: Vec<_> = witnesses.iter().filter(|w| w.assembly == id).collect();
        let disabled = profile
            .as_table_mut()
            .unwrap()
            .remove("hazard_witnesses")
            .unwrap();
        assert_eq!(disabled.as_array().unwrap().len(), selected.len());
        assert!(!selected.is_empty());
        for (_, value) in profile["sampled_drives"].as_table_mut().unwrap().iter_mut() {
            *value = literal(&fixtures.join(value.as_str().unwrap()));
        }
        save("control-muted-profile.toml", &profile);
        task["profile"] = literal(&temp.join("control-muted-profile.toml"));
        task["control"] = literal(&fixtures.join(task["control"].as_str().unwrap()));
        save("control-muted-task.toml", &task);
        std::fs::write(&manifest, format!(
            "version = 1\n[assemblies.{id}]\nscope = 'Контроль с выключенными свидетелями'\n\
             [assemblies.{id}.experiments.contour]\nkind = 'task'\nworld = {}\ntask = 'control-muted-task.toml'\njobs = 1\n",
            literal(&fixtures.join("world.toml")),
        )).unwrap();
        let muted = cli_at(binary, &manifest, &[], 0);
        assert_eq!(
            muted["assemblies"][id]["coverage"]["executed"],
            report["assemblies"][id]["coverage"]["executed"]
        );
        for proof in selected {
            assert!(
                verify_witness(&muted, proof).is_err(),
                "выключен {}",
                proof.class
            );
        }
    }
}

fn verify_unconfirmed_inputs(report: &Value, witnesses: &[Witness], binary: &Path, temp: &Path) {
    let fixtures = data().join("stage6/corpus/mechanical");
    let read = |path: &Path| -> toml::Value {
        toml::from_str(&std::fs::read_to_string(path).unwrap()).unwrap()
    };
    let save = |name: &str, value: &toml::Value| {
        let path = temp.join(name);
        std::fs::write(&path, toml::to_string(value).unwrap()).unwrap();
        path
    };
    let absolute = |base: &Path, name: &toml::Value| {
        toml::Value::String(base.join(name.as_str().unwrap()).to_str().unwrap().into())
    };
    let mut profile = read(&fixtures.join("unsafe-profile.toml"));
    let sources = profile["cases"].as_table_mut().unwrap();
    let proofs: Vec<_> = witnesses
        .iter()
        .filter(|w| w.mechanical.is_some())
        .collect();
    assert!(!proofs.is_empty());
    assert_eq!(proofs.len(), sources.len());
    for proof in &proofs {
        let selected = &mut sources[&proof.mode];
        let original = selected
            .as_str()
            .unwrap_or_else(|| selected["file"].as_str().unwrap());
        let mut case = read(&fixtures.join(original));
        case["world"] = absolute(&fixtures, &case["world"]);
        if let Some(contacts) = case.get_mut("contacts") {
            contacts["table"] = absolute(&fixtures, &contacts["table"]);
        }
        let model = &mut case["models"][&proof.object];
        let field = if model["kind"].as_str() == Some("attachment") {
            "witness_limit"
        } else {
            "upper"
        };
        // Отключаем подтверждение в самой модели. Значение и критерий,
        // подтверждённый источник которого отдельный, оставляем как были.
        model[field]["source"] =
            toml::from_str("estimate = 'Источник свидетеля выключен для отрицательного опыта'")
                .unwrap();
        let file = save(&format!("unconfirmed-{}.toml", proof.mode), &case);
        let new_path = toml::Value::String(file.to_str().unwrap().into());
        if selected.is_str() {
            *selected = new_path;
        } else {
            selected["file"] = new_path;
            selected["overlay"] = absolute(&fixtures, &selected["overlay"]);
        }
    }
    save("unconfirmed-profile.toml", &profile);
    let task = temp.join("unconfirmed-task.toml");
    std::fs::write(
        &task,
        "version = 1\nprocess_profile = 'unconfirmed-profile.toml'\n",
    )
    .unwrap();
    let manifest = temp.join("unconfirmed-campaign.toml");
    let assembly = &proofs[0].assembly;
    assert!(proofs.iter().all(|proof| proof.assembly == *assembly));
    std::fs::write(
        &manifest,
        format!(
            "version = 1\n[assemblies.{assembly}]\nscope = 'Численные опыты без подтверждения предела модели'\n\
             [assemblies.{assembly}.experiments.loads]\nkind = 'task'\nworld = {}\ntask = 'unconfirmed-task.toml'\n",
            toml::Value::String(fixtures.join("world-block.toml").to_str().unwrap().into())
        ),
    ).unwrap();
    let muted = cli_at(binary, &manifest, &[], 0);
    let normal = &report["assemblies"][assembly];
    let subject = &muted["assemblies"][assembly];
    assert_eq!(subject["verdict"], "Rejected");
    assert_eq!(
        subject["coverage"]["executed"],
        normal["coverage"]["executed"]
    );
    for proof in proofs {
        let criterion = &proof.mechanical.as_ref().unwrap().criterion;
        let saved = process_case(normal, &proof.mode).unwrap();
        let rerun = process_case(subject, &proof.mode).unwrap();
        assert_eq!(
            grid_values(saved, criterion),
            grid_values(rerun, criterion),
            "{}",
            proof.class
        );
        assert_eq!(
            metric(rerun, criterion)["outcome"],
            "fail",
            "{}",
            proof.class
        );
        let failure = verify_witness(&muted, proof).unwrap_err();
        assert_eq!(failure, format!("нет численного свидетеля {}", proof.class));
    }
}

#[test]
fn численный_свидетель_должен_измерять_свою_опасность() {
    let temp = Temp(std::env::temp_dir().join(format!("campaign-signal-{}", std::process::id())));
    std::fs::create_dir(&temp.0).unwrap();
    let data = data();
    let file = |name: &str| temp.0.join(name);
    let quoted = |path: PathBuf| toml::Value::String(path.to_str().unwrap().into());
    let overlay = std::fs::read_to_string(data.join("stage6/corpus/edge-sharp.toml")).unwrap();
    // Сила находится около 10 Н. В чужом критерии выставляем нижнюю
    // границу 11 Н из огибающей стенда, чтобы оба критерия дали отказ.
    let overlay = overlay
        .replace("value = 9.8,", "value = 11.0,")
        .replace("value = 10.2,", "value = 11.0,");
    std::fs::write(file("reference.toml"), &overlay).unwrap();
    std::fs::write(
        file("substitution.toml"),
        overlay.replace(
            "criterion = \"ограничение радиуса\"",
            "criterion = \"опорная сила\"",
        ),
    )
    .unwrap();
    std::fs::write(
        file("task.toml"),
        "version = 1\nprocess_profile = 'profile.toml'",
    )
    .unwrap();
    std::fs::write(
        file("profile.toml"),
        format!(
            r#"
version = 1
source = "Сравнение исходного и подменённого численного свидетеля"
[cases]
reference = {{ file = {base}, overlay = "reference.toml" }}
substitution = {{ file = {base}, overlay = "substitution.toml" }}
"#,
            base = quoted(data.join("stage4-processes/edge.toml"))
        ),
    )
    .unwrap();
    std::fs::write(
        file("manifest.toml"),
        format!(
            r#"
version = 1
[assemblies.block]
scope = "Каретка и кромка"
[assemblies.block.experiments.contact]
kind = "task"
world = {world}
task = "task.toml"
"#,
            world = quoted(data.join("stage4-processes/world-block.toml"))
        ),
    )
    .unwrap();
    let report = cli(&file("manifest.toml"), &[], 0);
    let assembly = &report["assemblies"]["block"];
    assert_eq!(assembly["verdict"], "Rejected");
    for (mode, code) in [
        ("reference", "ограничение радиуса"),
        ("substitution", "опорная сила"),
    ] {
        assert!(
            all_findings(assembly).any(|f| f["key"]["run"]["mode"] == mode
                && f["key"]["class"] == "task_criterion"
                && f["code"] == code
                && f["outcome"] == "fail")
        );
    }
    let modes: Vec<_> = all_findings(assembly)
        .filter(|f| f["key"]["class"] == "sharp_edge" && f["confirmed_hazard"] == true)
        .map(|f| f["key"]["run"]["mode"].as_str().unwrap())
        .collect();
    assert_eq!(modes, ["reference"]);
}

#[test]
fn новая_сборка_проверяет_допуск_зерно_и_чужую_точку() {
    let temp = Temp(std::env::temp_dir().join(format!("campaign-sample-{}", std::process::id())));
    std::fs::create_dir(&temp.0).unwrap();
    let base = data().join("stage6/corpus");
    let file = |name: &str| temp.0.join(name);
    let write = |name, contents: &str| std::fs::write(file(name), contents).unwrap();
    let fixture = |name| {
        std::fs::read_to_string(base.join(name))
            .unwrap()
            .replace("каретка", "блок")
    };
    for name in ["world.toml", "catalog.toml"] {
        write(name, &fixture(name));
    }
    // Новый брусок 20×50 мм той же массы и длины. Моменты инерции
    // пересчитаны по I = m(b² + h²)/12 и перестановкам осей.
    let rod = fixture("rod.toml")
        .replace("20×40", "20×50")
        .replace("[1.0, 0.02, 0.04]", "[1.0, 0.02, 0.05]")
        .replace("0.0003333333333333333", "0.0004833333333333334")
        .replace("0.16693333333333333", "0.16708333333333333");
    write("rod.toml", &rod);
    let safe = fixture("under-travel.toml");
    let install = fixture("under-travel-install.toml");
    write("install.toml", &install);
    write(
        "manifest.toml",
        r#"
version = 1
[assemblies.block]
scope = "Стержень на поступательной оси"
[assemblies.block.experiments.axis]
kind = "scenario"
campaign = "input.toml"
installation = "install.toml"
"#,
    );
    // x = x₀ + (F/2 кг)·(1+2+3+4)·(1 мс)².
    // С допуском 1 мкм сила 1,4002 Н даёт 8,001 мкм при пределе 8 мкм.
    let near = safe.replace("value = 1.0, unit = \"N\"", "value = 1.4002, unit = \"N\"");
    write("input.toml", &near);
    let manifest = file("manifest.toml");
    let checkpoint = file("checkpoint");
    let args = ["--checkpoint", checkpoint.to_str().unwrap()];
    let first = cli(&manifest, &args, 0);
    let assembly = &first["assemblies"]["block"];
    assert_eq!(assembly["verdict"], "Rejected");
    let old_key = assembly["worst_run_key"].as_str().unwrap();
    let witness = all_findings(assembly)
        .find(|f| f["key"]["class"] == "travel_jam" && f["confirmed_hazard"] == true)
        .expect("нет подтверждённого превышения хода");
    assert!((witness["measurement"][1].as_f64().unwrap() - 8.001e-6).abs() < 1e-14);
    assert_eq!(witness["limit"]["upper"], serde_json::json!([8e-6, 8e-6]));
    assert_eq!(witness["event_ns"], 4_000_000);
    assert_eq!(cli(&manifest, &args, 0)["resumed"], 1);

    let nominal = map(&assembly["runs"])
        .iter()
        .find(|(_, row)| row["detail"]["summary"]["key"]["sample"]["kind"] == "nominal")
        .unwrap()
        .0;
    let partial = cli(&manifest, &["--repeat", nominal], 0);
    let partial = &partial["assemblies"]["block"];
    assert_eq!(partial["verdict"], "NotProven");
    assert_eq!(partial["coverage"]["required"], 2);
    assert_eq!(partial["coverage"]["omitted"], 1);

    // Удваиваем массу и каждый момент инерции при неизменной внешней силе.
    // На новом образце максимальный ход 1 + 7,001/2 мкм.
    let heavier = rod
        .replace("масса 2 кг", "масса 4 кг")
        .replace("mass_kg = 2.0", "mass_kg = 4.0")
        .replace("0.0004833333333333334", "0.0009666666666666668")
        .replace("0.16708333333333333", "0.33416666666666667")
        .replace("0.16673333333333332", "0.33346666666666664");
    write("rod.toml", &heavier);
    let changed = cli(&manifest, &args, 0);
    assert_eq!(changed["resumed"], 0);
    assert_eq!(
        changed["assemblies"]["block"]["verdict"],
        "PassedSimulation"
    );
    assert_eq!(changed["assemblies"]["block"]["readiness"], "BenchOnly");
    let stress = all_findings(&changed["assemblies"]["block"])
        .find(|f| f["key"]["class"] == "static_strength")
        .unwrap();
    let section = 0.02 * 0.05_f64.powi(2) / 6.0;
    assert!((stress["measurement"][1].as_f64().unwrap() - 20.0 / section).abs() < 1e-8);
    assert_eq!(cli(&manifest, &["--repeat", old_key], 2)["status"], "error");

    let main = safe.split_once("[stations.stand]").unwrap();
    // Две партии нужны для наблюдения смены случайного потока. Это
    // синтетический закон и объём для проверки воспроизводимости.
    let probability = r#"
[sampling.probability]
batches = 2
specimens = 1
repeats = 1
confidence = 0.95
independence = "batches"
source = { kind = "datasheet", reference = "Протокол синтетической проверки воспроизводимости" }
batch = { source = { kind = "datasheet", reference = "Одна ветвь для проверки потока повторов" }, alternatives = [{ id = "only", weight = 1.0, values = {} }] }
specimen = [{ batch = "only", law = { source = { kind = "datasheet", reference = "Один образец для проверки потока повторов" }, alternatives = [{ id = "only", weight = 1.0, values = {} }] } }]
run = [{ batch = "only", specimen = "only", law = { source = { kind = "datasheet", reference = "Равномерное смещение от 0 до 1 мкм в синтетическом опыте" }, alternatives = [{ id = "only", weight = 1.0, values = { "начальная координата" = { kind = "uniform", min = 0.0, max = 0.000001, factor = "offset" } } }] } }]
"#;
    let sampled = format!("{}{probability}[stations.stand]{}", main.0, main.1);
    write("input.toml", &sampled);
    let binary = Path::new(env!("CARGO_BIN_EXE_sim-run"));
    let bytes = cli_output(binary, &manifest, &[], 0);
    assert_eq!(bytes, cli_output(binary, &manifest, &[], 0));
    let same = serde_json::from_slice(&bytes).unwrap();
    let selected = |report: &Value| {
        map(&report["assemblies"]["block"]["runs"])
            .iter()
            .filter_map(|(key, row)| {
                let summary = &row["detail"]["summary"];
                (summary["key"]["sample"]["kind"] == "random").then(|| {
                    (
                        key.clone(),
                        summary["installation"]["values"]["начальная координата"]
                            .as_f64()
                            .unwrap()
                            .to_bits(),
                    )
                })
            })
            .collect::<BTreeMap<_, _>>()
    };
    let before = selected(&same);
    assert_eq!(before.len(), 2);
    assert_eq!(same["assemblies"]["block"]["verdict"], "PassedSimulation");

    // У редкой партии положительный вес 1/1000 по протоколу этого опыта.
    // При зерне 7 обе партии попадают в проверенную ветвь, однако допуск
    // относится ко всему заявленному закону, включая оценочную ветвь.
    let mut unsupported: toml::Value = toml::from_str(&sampled).unwrap();
    let probability = &mut unsupported["sampling"]["probability"];
    let alternatives = probability["batch"]["alternatives"].as_array_mut().unwrap();
    alternatives.push(toml::from_str("id = 'rare'\nweight = 0.001\nvalues = {}").unwrap());
    for level in ["specimen", "run"] {
        let laws = probability[level].as_array_mut().unwrap();
        let mut rare = laws[0].clone();
        rare["batch"] = "rare".into();
        rare["law"]["source"]["kind"] = "estimate".into();
        laws.push(rare);
    }
    write("input.toml", &toml::to_string(&unsupported).unwrap());
    let unsupported = cli(&manifest, &[], 0);
    assert_eq!(
        selected(&unsupported).values().collect::<BTreeSet<_>>(),
        before.values().collect::<BTreeSet<_>>()
    );
    let sample_rows = rows(&unsupported["assemblies"]["block"]["statistics"]["axis"]["rows"]);
    let random: Vec<_> = sample_rows
        .iter()
        .filter(|r| r["sample"]["key"]["kind"] == "random")
        .collect();
    assert_eq!(random.len(), 2);
    assert!(
        random
            .iter()
            .all(|r| r["sample"]["alternatives"]["batch"] == "only")
    );
    assert_eq!(unsupported["assemblies"]["block"]["verdict"], "NotProven");
    assert!(
        all_findings(&unsupported["assemblies"]["block"]).any(|finding| {
            finding["key"]["object"] == "разброс"
                && finding["reason"].as_str().unwrap().contains("источник")
        })
    );

    write("input.toml", &sampled.replace("seed = 7", "seed = 8"));
    let after = selected(&cli(&manifest, &[], 0));
    assert_eq!(after.len(), 2);
    assert!(
        before
            .values()
            .all(|value| !after.values().any(|other| value == other))
    );
    let saved_key = before.first_key_value().unwrap().0;
    assert_eq!(
        cli(&manifest, &["--repeat", saved_key], 2)["status"],
        "error"
    );

    write("input.toml", &sampled.replace("batches = 2", "batches = 1"));
    let single = cli(&manifest, &[], 0);
    let assembly = &single["assemblies"]["block"];
    assert_eq!(assembly["verdict"], "NotProven");
    assert_eq!(assembly["readiness"], "NeedsWork");
    assert!(all_findings(assembly).any(|finding| {
        finding["key"]["object"] == "разброс"
            && finding["reason"]
                .as_str()
                .unwrap()
                .contains("меньше двух различимых выборок")
    }));
    let series = rows(&assembly["statistics"]["axis"]["series"])
        .iter()
        .find(|series| series["key"]["population"]["kind"] == "random")
        .unwrap();
    assert_eq!(series["planned"], 1);
    assert_eq!(
        series["metrics"]["конечный ход"]["repeatability"]["kind"],
        "not_analyzed"
    );

    let empty = safe
        .lines()
        .filter(|line| !line.starts_with("boundaries ="))
        .collect::<Vec<_>>()
        .join("\n");
    write("input.toml", &empty);
    assert_eq!(cli(&manifest, &[], 2)["status"], "error");
    for number in ["nan", "inf"] {
        write(
            "input.toml",
            &safe.replace(
                "value = 1.0, unit = \"N\"",
                &format!("value = {number}, unit = \"N\""),
            ),
        );
        assert_eq!(cli(&manifest, &[], 2)["status"], "error");
    }
}

#[test]
fn связанные_опыты_повторяют_выборку_снимок_и_дисковую_точку() {
    let temp = Temp(std::env::temp_dir().join(format!("campaign-linked-{}", std::process::id())));
    std::fs::create_dir(&temp.0).unwrap();
    let file = |name: &str| temp.0.join(name);
    let write = |name, text: &str| std::fs::write(file(name), text).unwrap();
    let devices = data().canonicalize().unwrap();
    let catalog = devices.parent().unwrap().join("catalog");
    let passive = devices.join("stage4-processes/world-energy.toml");
    let corpus = devices.join("stage6/corpus");
    let process_case = std::fs::read_to_string(corpus.join("circuit.toml")).unwrap();
    let process_case = process_case.replace(
        "../../stage4-processes/world-energy.toml",
        passive.to_str().unwrap(),
    );
    let process_case = format!("snapshot_ns = 2000000\n{process_case}");
    write("process.toml", &process_case);
    write(
        "process-task.toml",
        "version = 1\nprocess_profile = 'process-profile.toml'\n",
    );
    write(
        "process-profile.toml",
        "version = 1\nsource = 'Стенд цепи'\n[cases]\nblock = 'process.toml'\n",
    );

    let participant = devices.join("stage5-audit");
    for name in ["acceptance.toml", "profile.toml"] {
        std::fs::copy(participant.join(name), file(name)).unwrap();
    }
    let setup = std::fs::read_to_string(participant.join("setup.toml")).unwrap();
    // Пробный верхний предел 0,01 Н ниже диапазона исходной проверки 0,02–2 Н.
    let setup = setup
        .replace(
            "../../catalog/contact/stage5.toml",
            catalog.join("contact/stage5.toml").to_str().unwrap(),
        )
        .replace(
            "bounds = [0.02, 2.0]",
            "bounds = [0.0, 0.01]\nrequired_design = true",
        );
    write("setup.toml", &format!("snapshot_ns = 4000000\n{setup}"));

    // Два совместных состояния ускорения по X/Y по протоколу синтетической серии.
    let spread = r#"
axes = [
  { id = "x", parameter = { kind = "gravity", axis = 0 }, scope = "run", unit = "m/s^2", nominal = 0.0, min = 0.0, max = 0.01, source = { kind = "measured", reference = "Два состояния ускорения стенда." } },
  { id = "y", parameter = { kind = "gravity", axis = 1 }, scope = "run", unit = "m/s^2", nominal = 0.0, min = 0.0, max = 0.02, source = { kind = "measured", reference = "Два состояния ускорения стенда." } },
]
boundaries = [{ id = "край", values = { x = 0.01, y = 0.02 }, source = { kind = "measured", reference = "Обе компоненты меняются вместе." } }]
"#;
    write(
        "manifest.toml",
        &format!(
            r#"
version = 1
[assemblies.block]
scope = "Два опыта блока"
seed = 1
[assemblies.block.sampling]
{spread}
[assemblies.block.experiments.process]
kind = "task"
world = '{}'
task = 'process-task.toml'
[assemblies.block.experiments.participant]
kind = "task"
world = '{}'
task = 'acceptance.toml'
catalog = '{}'
"#,
            passive.display(),
            participant.join("world.toml").display(),
            catalog.display()
        ),
    );
    let manifest = file("manifest.toml");
    let checkpoint = file("checkpoint");
    let args = ["--checkpoint", checkpoint.to_str().unwrap()];
    let first = cli(&manifest, &args, 0);
    assert_eq!(first["resumed"], 0);
    let assembly = &first["assemblies"]["block"];
    assert_eq!(assembly["verdict"], "Rejected");
    let series = rows(&assembly["statistics"]["process"]["series"]);
    assert_eq!(series.len(), 1);
    assert_eq!(series[0]["planned"], 2);
    assert_eq!(series[0]["completed"], 2);
    assert_eq!(series[0]["failed"], 2);
    assert!(series[0]["confidence"]["interval"].is_null());
    let current = &series[0]["metrics"]["task_criterion/пиковый ток/пиковый ток"];
    assert_eq!(current["measured"], 2);
    // Две одинаковые нагрузки: 6 В / 12 Ом на каждой строке выборки.
    for field in ["median", "p95"] {
        assert!((current[field].as_f64().unwrap() - 0.5).abs() < 1e-10);
    }
    assert_eq!(current["worst"]["key"]["station"], "block/process");
    let runs = map(&assembly["runs"]);
    assert_eq!(runs.len(), 4);
    let mut seeds = BTreeMap::<String, Vec<u64>>::new();
    for (key, run) in runs {
        let detail = &run["detail"];
        let sample = &detail["sample"];
        let source = rows(&sample["values"]);
        let actual = source
            .iter()
            .map(|v| (v["id"].as_str().unwrap(), v["value"].as_f64().unwrap()))
            .collect::<BTreeMap<_, _>>();
        let kind = sample["key"]["kind"].as_str().unwrap();
        assert_eq!(
            [actual["x"], actual["y"]],
            if kind == "nominal" {
                [0.0, 0.0]
            } else {
                [0.01, 0.02]
            }
        );
        seeds
            .entry(kind.into())
            .or_default()
            .push(sample["seeds"]["specimen"].as_u64().unwrap());
        assert!(!map(&detail["inventory"]).is_empty());
        let profile = &detail["profile"];
        if key.contains("/process/") {
            let block = &profile["cases"]["block"];
            let trials = rows(&block["convergence"]["trials"]);
            assert_eq!(trials.len(), 3);
            assert!(
                trials
                    .iter()
                    .all(|trial| trial["snapshot"]["matched"] == true)
            );
            let peak = &rows(&block["convergence"]["metrics"])[0]["values"];
            // Закон Ома: 6 В / (1 + 1 + 10) Ом = 0,5 А.
            assert!(rows(peak).iter().all(|value| {
                rows(value).len() == 2
                    && rows(value)
                        .iter()
                        .all(|bound| (bound.as_f64().unwrap() - 0.5).abs() < 1e-10)
            }));
            for installation in map(&block["installations"]).values() {
                assert_eq!(installation["values"], serde_json::json!(actual));
            }
        } else {
            let scene = &rows(&profile["cases"])[0];
            assert_eq!(rows(&scene["coverage"]).len(), 3);
            for grid in rows(&scene["coverage"]) {
                assert_eq!(grid["snapshot"]["matched"], true);
                assert_eq!(grid["installation"]["values"], serde_json::json!(actual));
            }
        }
    }
    assert_eq!(seeds.len(), 2);
    assert!(
        seeds
            .values()
            .all(|seeds| seeds.len() == 2 && seeds[0] == seeds[1])
    );
    let second = cli(&manifest, &args, 0);
    assert_eq!(second["resumed"], 4);
    assert_eq!(first["assemblies"], second["assemblies"]);
    assert_eq!(first["worst_run_key"], second["worst_run_key"]);
    let worst = first["worst_run_key"].as_str().unwrap();
    let repeat = cli(&manifest, &["--repeat", worst], 0);
    assert_eq!(repeat["assemblies"]["block"]["runs"][worst], runs[worst]);

    let process_key = runs.keys().find(|key| key.contains("/process/")).unwrap();
    // Новая уставка питания даёт ток 2,4/12 = 0,2 А, ниже 0,25 А.
    write(
        "process.toml",
        &process_case
            .replace("value = 6,", "value = 2.4,")
            .replace("источник 6 В", "источник 2,4 В"),
    );
    let changed = cli(&manifest, &args, 0);
    assert_eq!(changed["resumed"], 2);
    assert!(
        map(&changed["assemblies"]["block"]["runs"])
            .iter()
            .filter(|(key, _)| key.contains("/process/"))
            .all(|(key, row)| !runs.contains_key(key) && row["outcome"] == "not_analyzed")
    );
    assert_eq!(
        cli(&manifest, &["--repeat", process_key], 2)["status"],
        "error"
    );
    assert_eq!(cli(&manifest, &args, 0)["resumed"], 2);
}

fn verify_children(run: &Value, key: &str) {
    let profile = &run["detail"]["profile"];
    let children = rows(&run["detail"]["children"]);
    assert_eq!(
        children.len() as u64,
        run["required"].as_u64().unwrap(),
        "{key}"
    );
    for child in children {
        let address = &child["key"];
        let mode = address["mode"].as_str().unwrap();
        if !profile["evidence"].is_null() {
            let cases = rows(&profile["evidence"]["cases"]);
            let case = cases
                .iter()
                .find(|case| {
                    case["case"]["mode"] == mode
                        && case["case"]["path"] == address["path"]
                        && address["faults"]
                            .as_array()
                            .unwrap()
                            .first()
                            .unwrap_or(&Value::Null)
                            == &case["case"]["fault"]
                })
                .unwrap_or_else(|| panic!("нет строки контура: {key} / {address}"));
            assert_eq!(case["outcome"], child["outcome"], "{key}");
            if child["executed"] == true {
                assert!(
                    case["journal"].as_u64().is_some_and(|journal| journal != 0),
                    "{key} / {mode}"
                );
                assert_eq!(case["completed"], true, "{key} / {mode}");
                assert!(
                    case["reached_ns"].as_u64().is_some_and(|time| time > 0),
                    "{key} / {mode}"
                );
                if !case["case"]["fault"].is_null() {
                    assert!(
                        case["requested_ns"].is_u64() && case["applied_ns"].is_u64(),
                        "{key} / {mode}"
                    );
                }
            } else {
                let reasons = rows(&case["missing"]);
                assert!(
                    reasons
                        .iter()
                        .any(|r| r.as_str().is_some_and(|text| !text.trim().is_empty())),
                    "{key} / {address}"
                );
            }
        } else if let Some(cases) = profile["cases"].as_object() {
            let case = &cases[mode];
            assert_eq!(case["outcome"], child["outcome"], "{key} / {mode}");
            let trials = rows(&case["convergence"]["trials"]);
            assert!(trials.len() >= 3 && trials.len().is_multiple_of(3));
            if child["executed"] == true {
                for trial in trials {
                    let steps = trial["steps"].as_u64().unwrap();
                    let hz = trial["rate_hz"].as_u64().unwrap();
                    let end = case["convergence"]["window_ns"][1].as_u64().unwrap();
                    assert_eq!(trial["completed"], true, "{key} / {mode}");
                    assert!(
                        trial["journal"]
                            .as_u64()
                            .is_some_and(|journal| journal != 0)
                    );
                    assert!(
                        steps > 0
                            && u128::from(steps) * 1_000_000_000
                                >= u128::from(end) * u128::from(hz)
                    );
                }
                assert!(
                    rows(&case["convergence"]["metrics"])
                        .iter()
                        .any(|m| rows(&m["values"]).iter().any(|v| !v.is_null())),
                    "{key} / {mode}"
                );
            } else {
                assert!(
                    trials.iter().any(|trial| {
                        trial["completed"] == false
                            && rows(&trial["missing"]).iter().any(|value| {
                                value
                                    .as_str()
                                    .is_some_and(|reason| !reason.trim().is_empty())
                            })
                    }),
                    "{key} / {mode}"
                );
            }
        } else {
            let cases = rows(&profile["cases"]);
            let case = cases
                .iter()
                .find(|c| c["id"] == mode)
                .expect("вариант взаимодействия");
            assert_eq!(case["outcome"], child["outcome"], "{key} / {mode}");
            assert_eq!(rows(&case["coverage"]).len(), 3, "{key} / {mode}");
            if child["executed"] == true {
                for grid in rows(&case["coverage"]) {
                    assert_eq!(
                        grid["completed_steps"], grid["expected_steps"],
                        "{key} / {mode}"
                    );
                    assert!(grid["completed_steps"].as_u64().unwrap() > 0);
                }
                assert!(
                    rows(&case["evidence"]["grids"])
                        .iter()
                        .all(|grid| grid["journal"].as_u64().is_some_and(|journal| journal != 0)),
                    "{key} / {mode}"
                );
                assert!(
                    rows(&case["checks"])
                        .iter()
                        .any(|check| check["value"].is_number()),
                    "{key} / {mode}"
                );
            }
        }
    }
}

#[test]
fn постановка_исполняет_прямую_ссылку_на_профиль() {
    let fixtures = data().join("stage6").canonicalize().unwrap();
    let temp = Temp(std::env::temp_dir().join(format!("station-stage6-{}", std::process::id())));
    std::fs::create_dir(&temp.0).unwrap();
    let save = |name: &str, text: &str| std::fs::write(temp.0.join(name), text).unwrap();
    let path = |file: &str| {
        toml::Value::String(
            fixtures
                .join(file)
                .canonicalize()
                .unwrap()
                .to_str()
                .unwrap()
                .into(),
        )
    };
    let campaign = format!(
        r#"version = 1
seed = 1 # Зерно синтетического блока из описания мира.
world = {}
catalog = {}

[stations.block]
experiment = {{ kind = "process", task = {} }}
profile = {}

[stations.block.modes.curve-offset]
horizon_ns = 4000000
source = {{ kind = "datasheet", reference = "Горизонт синтетического блока, четыре миллисекунды." }}
"#,
        path("corpus/free-world.toml"),
        path("corpus/catalog.toml"),
        path("corpus/screen/curve-offset-task.toml"),
        path("corpus/screen/curve-offset-profile.toml")
    );
    save("station.toml", &campaign);
    save(
        "installation.toml",
        r#"version = 1
[[paths]]
station = "block"
mode = "curve-offset"
path = "curve-offset"
source = { kind = "datasheet", reference = "Один процесс на постановку." }
grids = [{ id = "base", hz = 1000, source = { kind = "datasheet", reference = "Базовая сетка 1 мс, профиль проверяет её на вложенных сетках." } }]
"#,
    );
    save(
        "manifest.toml",
        r#"version = 1
[assemblies.block]
scope = "Прямая ссылка на падение блока"
[assemblies.block.experiments.fixture]
kind = "scenario"
campaign = "station.toml"
installation = "installation.toml"
"#,
    );
    let run = || cli(&temp.0.join("manifest.toml"), &[], 0);
    let report = run();
    let assembly = &report["assemblies"]["block"];
    assert_eq!(assembly["coverage"]["executed"], 1);
    assert_eq!(assembly["verdict"], "Rejected");
    let found = all_findings(assembly)
        .find(|f| f["key"]["class"] == "measurement" && f["confirmed_hazard"] == true)
        .unwrap();
    assert_eq!(found["key"]["run"]["station"], "block");
    assert_eq!(found["key"]["run"]["grid"], "base");
    assert!((found["measurement"][1].as_f64().unwrap() - 0.001025).abs() < 1e-12);
    let stats = &assembly["statistics"]["fixture"]["series"][0];
    assert_eq!(stats["failed"], 1);
    assert_eq!(
        stats["metrics"]["measurement/отметка блока/measurement"]["measured"],
        1
    );
    let key = assembly["worst_run_key"].as_str().unwrap();
    let replay = cli(&temp.0.join("manifest.toml"), &["--repeat", key], 0);
    assert_eq!(
        replay["assemblies"]["block"]["runs"][key],
        assembly["runs"][key]
    );
    save(
        "station.toml",
        &campaign.replace("horizon_ns = 4000000", "horizon_ns = 5000000"),
    );
    let wrong = run();
    assert_eq!(wrong["assemblies"]["block"]["coverage"]["executed"], 0);
    assert_eq!(wrong["assemblies"]["block"]["verdict"], "NotProven");
    assert!(
        map(&wrong["assemblies"]["block"]["runs"])
            .values()
            .any(|row| {
                row["detail"][0]
                    .as_str()
                    .is_some_and(|reason| reason.contains("горизонта"))
            })
    );

    let control = format!(
        r#"version = 1
seed = 17 # Зерно синтетического стержня из описания мира.
world = {}
catalog = {}

[stations.block]
experiment = {{ kind = "control", task = {} }}

[stations.block.modes."контур"]
horizon_ns = 12000000
source = {{ kind = "datasheet", reference = "Горизонт синтетического стержня из задания контура." }}
faults = [{{ fault = {{ node = "power:стенд/допуск", kind = "power_cut" }}, at_ns = 5000000, max_lag_ns = 0, window = "cut", source = {{ kind = "datasheet", reference = "Пятый такт ключа питания в задании." }} }}]

[stations.block.modes."контур".windows.cut]
start_ns = 3000000
end_ns = 10000000
max_gap_ns = 1000000
"#,
        path("corpus/control/world.toml"),
        path("../../catalog"),
        path("corpus/control/unsafe-task.toml")
    );
    save("station.toml", &control);
    save(
        "installation.toml",
        r#"version = 1
[[paths]]
station = "block"
mode = "контур"
path = "напряжение"
faults = [{ node = "power:стенд/допуск", kind = "power_cut" }]
source = { kind = "datasheet", reference = "Исполнительный путь и ключ питания из проводки." }
grids = [{ id = "base", hz = 1000, source = { kind = "datasheet", reference = "Базовая сетка 1 мс." } }]
"#,
    );
    let linked_control = run();
    let assembly = &linked_control["assemblies"]["block"];
    assert_eq!(assembly["coverage"]["executed"], 2);
    let source = all_findings(assembly)
        .find(|f| f["key"]["class"] == "emergency_stop" && f["confirmed_hazard"] == true)
        .unwrap();
    assert_eq!(source["key"]["run"]["faults"][0]["kind"], "power_cut");
    assert!((source["measurement"][1].as_f64().unwrap() - 0.522335524313519).abs() <= 0.012);
    save(
        "station.toml",
        &control.replace("at_ns = 5000000", "at_ns = 6000000"),
    );
    assert_eq!(run()["assemblies"]["block"]["coverage"]["executed"], 1);
}

#[test]
fn семь_сборок_через_cli() {
    let expected = acceptance();
    let manifest = data().join("stage6.toml");
    let binary = Path::new(env!("CARGO_BIN_EXE_sim-run"));
    let first = cli_output(binary, &manifest, &[], 0);
    let second = cli_output(binary, &manifest, &[], 0);
    assert!(first == second, "полные живые отчёты отличаются побайтно");
    println!(
        "Два полных живых отчёта: {} байт, FNV-1a {:016x}",
        first.len(),
        sim_signal::fnv1a(first.iter().copied())
    );
    let report: Value = serde_json::from_slice(&first).unwrap();
    assert_eq!(report["resumed"], 0);
    assert_eq!(report["version"], expected.version);
    assert_eq!(report["status"], "completed");
    let assemblies = map(&report["assemblies"]);
    assert_eq!(assemblies.len(), 7);
    assert_eq!(
        assemblies.keys().collect::<Vec<_>>(),
        expected.assemblies.keys().collect::<Vec<_>>()
    );
    for (id, a) in assemblies {
        let spec = &expected.assemblies[id];
        assert_eq!(a["verdict"], spec.verdict, "{id}: {}", a["coverage"]);
        let runs = map(&a["runs"]);
        assert_eq!(
            runs.keys()
                .map(|key| key.split_once('/').unwrap().1.to_owned())
                .collect::<BTreeSet<_>>(),
            spec.experiments.keys().cloned().collect::<BTreeSet<_>>()
        );
        assert_eq!(a["coverage"]["executed"], spec.executed, "{id}");
        for (key, run) in runs {
            assert_eq!(
                run["required"],
                spec.experiments[key.split_once('/').unwrap().1],
                "{key}"
            );
            verify_children(run, key);
            assert!(!rows(&run["detail"]["children"]).is_empty(), "{key}");
            assert!(!rows(&run["findings"]).is_empty(), "{key}");
            let child_exists = rows(&run["detail"]["children"])
                .iter()
                .any(|c| c["executed"] == true);
            assert!(child_exists, "{key}");
        }
        if spec.verdict == "Rejected" {
            assert!(
                all_findings(a).any(|f| f["outcome"] == "fail" && f["code"].is_string()),
                "{id}"
            );
        }
        println!("{id} {:?} {}", a["verdict"], a["coverage"]);
    }
    let run = |key: &str| &assemblies[key.split_once('/').unwrap().0]["runs"][key];
    for proof in &expected.restored_fault {
        let cases = rows(&run(&proof.run)["detail"]["profile"]["evidence"]["cases"]);
        let fault = serde_json::to_value(&proof.fault).unwrap();
        let case = cases
            .iter()
            .find(|case| case["case"]["mode"] == proof.mode && case["case"]["fault"] == fault)
            .unwrap_or_else(|| panic!("нет инъекции {} / {}: {fault}", proof.run, proof.mode));
        assert_eq!(case["completed"], true);
        assert_eq!(case["requested_ns"], proof.at_ns);
        assert_eq!(case["applied_ns"], proof.at_ns);
    }
    for check in expected.reference {
        let comparison = &run(&check.run)["detail"]["profile"]["cases"][&check.mode]["reference"];
        assert_eq!(comparison["passed"], true, "{} / {}", check.run, check.mode);
        let errors = rows(&comparison["errors"]);
        assert!(!errors.is_empty());
        assert!(
            errors
                .iter()
                .all(|e| e["samples"] == check.samples && e["passed"] == true)
        );
    }
    for failure in expected.named_failure {
        assert!(
            rows(&run(&failure.run)["findings"]).iter().any(|f| {
                f["key"]["run"]["mode"] == failure.mode
                    && f["code"] == failure.code
                    && f["outcome"] == "fail"
            }),
            "{} / {}: {}",
            failure.run,
            failure.mode,
            failure.code
        );
    }
    let samples = |key: &str, mode: &str, measurement: &str| -> Vec<f64> {
        let cases = rows(&run(key)["detail"]["profile"]["cases"]);
        let case = cases
            .iter()
            .find(|c| c["id"] == mode)
            .expect("опыт сравнения");
        rows(&case["checks"])
            .iter()
            .filter(|c| c["id"] == measurement)
            .map(|c| c["value"].as_f64().expect("измеренное значение"))
            .collect()
    };
    let comparison = expected.independent;
    for measurement in &comparison.measurements {
        let first = samples(&comparison.run, &comparison.mode, measurement);
        let second = samples(&comparison.repeat, &comparison.repeat_mode, measurement);
        assert_eq!(first.len(), 3);
        assert_eq!(first.len(), second.len());
        assert!(
            first
                .iter()
                .zip(second)
                .all(|(a, b)| (a - b).abs() <= comparison.tolerance),
            "{measurement}"
        );
    }
}
