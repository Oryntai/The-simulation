use super::*;
use toml::Value as Toml;

fn fixture(path: &Path) -> Toml {
    let mut case: Toml = toml::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    let absolute = |file: &mut Toml| {
        *file = path
            .parent()
            .unwrap()
            .join(file.as_str().unwrap())
            .canonicalize()
            .unwrap()
            .to_str()
            .unwrap()
            .into();
    };
    if let Some(world) = case.get_mut("world") {
        absolute(world);
    }
    if let Some(contacts) = case.get_mut("contacts") {
        absolute(&mut contacts["table"]);
    }
    case
}

fn save(path: &Path, contents: &Toml) {
    std::fs::write(path, toml::to_string(contents).unwrap()).unwrap();
}

// Полунеявный Euler, масса 1 кг, исходный зазор 20 мм.
// Сила магнита на первом шаге ещё не приложена, далее P = 20 − 500x.
fn retention(load: f64, hz: u64) -> f64 {
    let h = 1.0 / hz as f64;
    let (mut x, mut v) = (0.02, 0.0);
    let mut peak = f64::NEG_INFINITY;
    for step in 0..(8 * hz / 1000) {
        let force = if step < 5 * hz / 1000 { 9.0 } else { load };
        let pull = if step == 0 { 0.0 } else { 20.0 - 500.0 * x };
        v += h * (force - pull);
        x += h * v;
        let deficit = force - (20.0 - 500.0 * x);
        if step + 1 >= 3 * hz / 1000 {
            peak = peak.max(deficit);
            if deficit > 0.0 {
                break;
            }
        }
    }
    peak
}

struct Specimen {
    proof: Witness,
    cases: [Toml; 2],
    reference: [[f64; 3]; 2],
    criterion: String,
}

fn specimen(proof: &Witness) -> Specimen {
    let corpus = data().join("stage6/corpus");
    let mechanical = corpus.join("mechanical");
    let old = data().join("stage4-processes");
    let both = |file: PathBuf| {
        let input = fixture(&file);
        [input.clone(), input]
    };
    let mut cases = match proof.class.as_str() {
        "sharp_edge" | "stored_energy" => {
            let base = if proof.class == "sharp_edge" {
                "edge"
            } else {
                "spring"
            };
            let mut case = fixture(&old.join(format!("{base}.toml")));
            let overlay = fixture(&corpus.join(format!("{}.toml", proof.mode)));
            case["criteria"] = overlay["criteria"].clone();
            case.as_table_mut().unwrap().insert(
                "hazard_witnesses".into(),
                overlay["hazard_witnesses"].clone(),
            );
            [case.clone(), case]
        }
        "overcurrent" => both(corpus.join("circuit.toml")),
        "wiring_heat" => both(corpus.join("conductor-hot.toml")),
        _ => both(mechanical.join(format!("{}.toml", proof.mode))),
    };
    let criterion = cases[0]["hazard_witnesses"][0]["criterion"]
        .as_str()
        .unwrap()
        .to_owned();
    let mut reference = [[0.0; 3]; 2];
    for (i, case) in cases.iter_mut().enumerate() {
        let sign = if i == 0 { 1.0 } else { -1.0 };
        let model = &mut case["models"][&proof.object];
        let expected = match proof.class.as_str() {
            "fatigue" => {
                // Полпроцента по обе стороны предела. M = σπd³/32, d = 10 мм.
                let stress = proof.mechanical.as_ref().unwrap().upper * (1.0 + sign * 0.005);
                let moment = stress * std::f64::consts::PI * 0.01_f64.powi(3) / 32.0;
                for (step, value) in case["models"]["нагрузка"]["values"]
                    .as_array_mut()
                    .unwrap()
                    .iter_mut()
                    .zip([moment, -moment, moment])
                {
                    step["value"] = value.into();
                }
                stress
            }
            "pinch" | "contact_structure" => {
                // Запас в десять остатков 1 мН. В равновесии N = |F|.
                let limit = proof.mechanical.as_ref().unwrap().upper;
                let force = limit + sign * 0.01;
                // Набор силы за 100 мс с шагом базовой сетки 1 мс,
                // затем ещё 100 мс до окна. Ступень на старте даёт всплеск.
                case["models"]["нагрузка"]["values"] = Toml::Array(
                    (0..=100)
                        .map(|ms| {
                            Toml::Table(toml::Table::from_iter([
                                ("at_ns".into(), (ms * 1_000_000_i64).into()),
                                ("value".into(), (-force * ms as f64 / 100.0).into()),
                            ]))
                        })
                        .collect(),
                );
                force
            }
            "dangerous_motion" => {
                // За 10 мс: v = |F|t / 1 кг. Шаг нагрузки 0,1 Н даёт 1 мм/с.
                let force = 50.0 + sign * 0.1;
                case["models"]["нагрузка"]["values"][0]["value"] = (-force).into();
                force * 0.01
            }
            "connection_failure" => {
                if i == 0 {
                    // До движения реакция weld = d(0)F = 0,9·1 Н.
                    // Огибающая выбирается так, чтобы первое измерение было 1,01.
                    let overlay = fixture(&mechanical.join("connection-weak.toml"));
                    model["limits"]["capacity"]["force_n"] =
                        overlay["models"][&proof.object]["limits"]["capacity"]["force_n"].clone();
                    model["limits"]["capacity"]["force_n"]["value"] = (0.9 / 1.01).into();
                    model["limits"]["capacity"]["force_n"]["source"]["datasheet"] =
                        "Расчётная проверка 0,9 Н / 1,01 по импедансу weld в исходной позе.".into();
                    for field in ["states", "events"] {
                        case.as_table_mut()
                            .unwrap()
                            .insert(field.into(), overlay[field].clone());
                    }
                    1.01
                } else {
                    0.5 // Штатная пара: 1 Н / 2 Н после установления.
                }
            }
            "retention" => {
                // 50 мН по обе стороны исходной силы удержания 10 Н.
                // Шаг в 2,5 раза больше допуска сходимости 20 мН.
                let load = 10.0 + sign * 0.05;
                model["loading"]["steps"][1]["force_n"][0] = load.into();
                if i == 1 {
                    case.as_table_mut().unwrap().remove("events");
                    case["states"][&proof.object]
                        .as_array_mut()
                        .unwrap()
                        .truncate(1);
                }
                reference[i] = [1000, 2000, 4000].map(|hz| retention(load, hz));
                continue;
            }
            "stored_energy" => {
                // 1 мДж по обе стороны 0,1 Дж, x = √(2E/k).
                let energy = 0.1 + sign * 0.001;
                model["store"]["deflection_m"]["value"] = (2.0 * energy / 200.0).sqrt().into();
                energy
            }
            "sharp_edge" => {
                // Радиус отличается от 1 мм на 1 мкм. Запас = r − r_min.
                let radius = 0.001 - sign * 0.000001;
                model["measured_radius_m"]["value"] = radius.into();
                radius - 0.001
            }
            "overcurrent" => {
                // 1 мА по обе стороны 0,25 А. U = I·(1+1+10) Ом.
                let current = 0.25 + sign * 0.001;
                model["supply_v"]["value"] = (current * 12.0).into();
                current
            }
            "wiring_heat" => {
                // 5 мК по обе стороны 300,5 К. I = √(CΔT/Rt), U = 12I.
                let rise = 0.5 + sign * 0.005;
                model["supply_v"]["value"] = (12.0 * (0.001 * rise / 0.004).sqrt()).into();
                300.0 + rise
            }
            _ => panic!("не задан независимый расчёт для {}", proof.class),
        };
        reference[i] = [expected; 3];
        // Старый контроль точного номинала накопителя заменён независимым
        // расчётом выше. Сам предел энергии остаётся тем же.
        if proof.class == "stored_energy" {
            case["criteria"]
                .as_array_mut()
                .unwrap()
                .retain(|c| c["name"].as_str() == Some(&criterion));
        }
    }
    Specimen {
        proof: proof.clone(),
        cases,
        reference,
        criterion,
    }
}

#[test]
fn корпус_замечает_отключение_десяти_пограничных_расчётов() {
    let expected = acceptance();
    let temp =
        Temp(std::env::temp_dir().join(format!("campaign-threshold-{}", std::process::id())));
    std::fs::create_dir(&temp.0).unwrap();
    let manifest = temp.0.join("manifest.toml");
    let mut manifest_text = String::from("version = 1\n");
    let mut specimens = Vec::new();
    for class in [
        "fatigue",
        "pinch",
        "contact_structure",
        "dangerous_motion",
        "connection_failure",
        "retention",
        "stored_energy",
        "sharp_edge",
        "overcurrent",
        "wiring_heat",
    ] {
        let mut specimen = specimen(
            expected
                .witnesses
                .iter()
                .find(|w| w.class == class)
                .unwrap(),
        );
        let mode = &specimen.proof.mode;
        let mut disabled = specimen.cases[0].clone();
        if matches!(class, "wiring_heat" | "connection_failure" | "retention") {
            // Физическая цепь/стык остаётся в сцене. Поле включает отдельный
            // численный расчёт: тепловой интеграл или наблюдение нагрузки.
            let field = if class == "wiring_heat" {
                "conductor_heat"
            } else {
                "witness_limit"
            };
            disabled["models"][&specimen.proof.object]
                .as_table_mut()
                .unwrap()
                .remove(field)
                .unwrap();
        } else {
            disabled["models"]
                .as_table_mut()
                .unwrap()
                .remove(&specimen.proof.object)
                .unwrap();
        }
        for (label, case) in ["outside", "inside", "disabled"]
            .into_iter()
            .zip(specimen.cases.iter().chain([&disabled]))
        {
            let id = format!("{class}-{label}").replace('_', "-");
            save(&temp.0.join(format!("{id}-case.toml")), case);
            std::fs::write(
                temp.0.join(format!("{id}-profile.toml")),
                format!(
                    "version = 1\nsource = 'Пограничный опыт'\n[cases]\n{mode} = '{id}-case.toml'\n"
                ),
            )
            .unwrap();
            std::fs::write(
                temp.0.join(format!("{id}-task.toml")),
                format!("version = 1\nprocess_profile = '{id}-profile.toml'\n"),
            )
            .unwrap();
            manifest_text.push_str(&format!(
                "[assemblies.{id}]\nscope = 'Пограничный опыт'\n[assemblies.{id}.experiments.test]\nkind = 'task'\nworld = {}\ntask = '{id}-task.toml'\n",
                case["world"]
            ));
            match label {
                "outside" => specimen.proof.assembly = id,
                "inside" => specimen.proof.pair_assembly = id,
                _ => {}
            }
        }
        specimens.push(specimen);
    }
    std::fs::write(&manifest, manifest_text).unwrap();
    let report = cli(&manifest, &[], 0);
    for mut specimen in specimens {
        let proof = &mut specimen.proof;
        if let Some(mechanical) = &mut proof.mechanical {
            mechanical.grid_reference = Some(specimen.reference);
        } else {
            proof.expected = Some(specimen.reference[0][2]);
            proof.pair_mode = Some(proof.mode.clone());
            proof.pair_metric = Some(specimen.criterion.clone());
            proof.pair_expected = Some(specimen.reference[1][2]);
        }
        verify_witness(&report, proof).unwrap_or_else(|e| panic!("{e}"));
        let id = proof.assembly.replace("-outside", "-disabled");
        let disabled = &report["assemblies"][&id];
        let case = process_case(disabled, &proof.mode).unwrap();
        let trials = rows(&case["convergence"]["trials"]);
        assert_eq!(trials.len(), 3, "{}", proof.class);
        assert!(
            trials.iter().all(|trial| trial["completed"] == true),
            "{}",
            proof.class
        );
        let missing = metric(case, &specimen.criterion);
        assert_eq!(missing["outcome"], "not_analyzed", "{}", proof.class);
        assert!(
            rows(&missing["values"]).iter().all(Value::is_null),
            "{}",
            proof.class
        );
        // Разрушение стыка остаётся самостоятельным отказом. Удалённое
        // численное измерение не может подтвердить класс даже при Rejected.
        assert!(
            !all_findings(disabled)
                .any(|f| { f["key"]["class"] == proof.class && f["confirmed_hazard"] == true }),
            "{}",
            proof.class
        );
        assert!(matches!(
            disabled["verdict"].as_str(),
            Some("NotProven" | "Rejected")
        ));
        let mut disabled_proof = proof.clone();
        disabled_proof.assembly = id;
        assert!(
            verify_witness(&report, &disabled_proof).is_err(),
            "{}",
            proof.class
        );
        println!(
            "{}: {:?} / {:?}, без расчёта {}",
            proof.class, specimen.reference[0], specimen.reference[1], disabled["verdict"]
        );
    }
}
