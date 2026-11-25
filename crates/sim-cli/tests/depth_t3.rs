use std::{
    f64::consts::FRAC_PI_2,
    fs,
    path::{Path, PathBuf},
    process::{Command, Output},
};

use serde_json::{Value, json};
use sim_cli::electrical::{load_catalog, load_schematic};
use sim_process::{
    Analysis,
    schematic::{ElectricalSeries, Endpoint, Mode, Probe, Section},
};
use sim_signal::Time;
use sim_task::Fingerprint;
use toml::{Table, Value as Toml};

const SECOND: u64 = 1_000_000_000; // Наносекунд в секунде.
const COULOMBS_PER_AH: f64 = 3600.0;
const ROUNDING: f64 = 1e-12; // Запас на направленное округление конечных сумм стенда.

struct Case {
    code: &'static str,
    branch: [&'static str; 2],
    mode: &'static str,
    window: [u64; 2],
}

const CASES: [Case; 6] = [
    Case {
        code: "capacitor_stress",
        branch: ["cap", "c"],
        mode: "Normal",
        window: [0, SECOND / 1000],
    },
    Case {
        code: "fuse_coordination",
        branch: ["fault", "fuse"],
        mode: "Stall",
        window: [SECOND, SECOND + 35_000_000],
    },
    Case {
        code: "switch_thermal",
        branch: ["switch", "channel"],
        mode: "Normal",
        window: [0, 10 * SECOND],
    },
    Case {
        code: "deep_discharge",
        branch: ["battery", "emf"],
        mode: "Normal",
        window: [0, 3 * SECOND],
    },
    Case {
        code: "sleep_drain",
        branch: ["battery", "emf"],
        mode: "Normal",
        window: [0, 1000 * SECOND],
    },
    Case {
        code: "cable_flex",
        branch: ["lead", ""],
        mode: "Normal",
        window: [0, SECOND],
    },
];

fn corpus() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../devices/electrical/depth_t3")
}

fn launch(manifest: &Path) -> (Output, Value) {
    let output = Command::new(env!("CARGO_BIN_EXE_sim-run"))
        .arg("--campaign")
        .arg(manifest)
        .arg("--json")
        .output()
        .unwrap();
    let report = serde_json::from_slice(&output.stdout).unwrap_or_else(|err| {
        panic!(
            "{}: {err}\n{}\n{}",
            manifest.display(),
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        )
    });
    (output, report)
}

fn campaign(manifest: &Path) -> Value {
    let (output, report) = launch(manifest);
    assert_eq!(output.status.code(), Some(0), "{report}");
    assert_eq!(report["version"], 1);
    assert!(report["assemblies"].is_object());
    report
}

fn window([start, end]: [u64; 2]) -> Value {
    json!({ "start_ns": start, "end_ns": end })
}

fn element(component: &str, name: &str) -> Value {
    json!({ "kind": "element", "component": component, "element": name })
}

impl Case {
    fn id(&self, variant: &str) -> String {
        format!("{}_{variant}", self.code)
    }

    fn input(&self, variant: &str) -> PathBuf {
        corpus().join(self.code).join(variant)
    }

    fn check<'a>(&self, report: &'a Value, expected_status: &str, scope: &str) -> &'a Value {
        assert_eq!(report["version"], 1);
        let assembly = &report["assemblies"]["block"];
        let findings = assembly["depth_checks"].as_object().unwrap();
        assert_eq!(findings.len(), 1, "{}", self.code);
        let hit = &findings[self.code];
        assert_eq!(hit["code"], self.code, "{hit}");
        assert_eq!(hit["status"], expected_status, "{hit}");
        assert!(!hit["reason"].as_str().unwrap().trim().is_empty());
        assert_eq!(hit["scope"], scope, "{hit}");
        let [component, section] = self.branch;
        let branch = if self.code == "cable_flex" {
            json!({ "kind": "connection", "id": component })
        } else {
            element(component, section)
        };
        assert_eq!(hit["branch"], branch, "{hit}");
        assert_eq!(hit["window"], window(self.window), "{hit}");
        assert_eq!(hit["mode"], self.mode, "{hit}");
        let mode = assembly["modes"]
            .as_array()
            .unwrap()
            .iter()
            .find(|row| row["mode"] == self.mode)
            .unwrap();
        assert_eq!(hit["mode_number"], mode["number"], "{hit}");
        assert_eq!(hit["build"], report["build"], "{hit}");
        assert_eq!(hit["observation_input"], assembly["schematic"], "{hit}");
        assert!(
            report["findings"]
                .as_array()
                .unwrap()
                .iter()
                .all(|row| row["outcome"] != "fail"),
            "другая причина закрыла исход для {}: {}",
            self.code,
            report["findings"]
        );
        assert_eq!(
            report["simulation"],
            if expected_status == "fail" {
                "Rejected"
            } else {
                "NotProven"
            }
        );
        hit
    }
}

fn bounds(actual: &Value, expected: [f64; 2], tolerance: f64) {
    let actual = actual.as_array().unwrap();
    assert_eq!(actual.len(), expected.len());
    for (observed, expected) in actual.iter().zip(expected) {
        let observed = observed.as_f64().unwrap();
        assert!(
            (observed - expected).abs() <= tolerance,
            "{observed} вместо {expected}, допуск {tolerance}"
        );
    }
}

fn event(hit: &Value, kind: &str, branch: &Value, at: u64) {
    let event = &hit["event"];
    assert_eq!(event["kind"], kind, "{hit}");
    assert_eq!(&event["branch"], branch, "{hit}");
    assert_eq!(event["time_ns"], at, "{hit}");
    assert_eq!(event["source"]["kind"], "derived", "{hit}");
    assert!(!event["source"]["reference"].as_str().unwrap().is_empty());
}

fn capacitor(hit: &Value, fixed: bool) {
    let metrics = &hit["metrics"];
    let source_v = if fixed { 8.0 } else { 10.0 };
    let limit = 12.0 * 0.75;
    let voltage = &metrics["voltage_v"]["value"];
    bounds(&voltage["observed"], [source_v; 2], ROUNDING);
    bounds(&voltage["limit"], [limit; 2], ROUNDING);
    bounds(&voltage["margin"], [limit - source_v; 2], ROUNDING);
    // DC при постоянной ЭДС: dU/dt = 0, значит нет пульсаций и потерь I²·ESR.
    bounds(
        &metrics["ripple_a"]["value"]["observed"],
        [0.0; 2],
        ROUNDING,
    );
    bounds(&metrics["loss_w"]["value"], [0.0; 2], ROUNDING);
    bounds(
        &metrics["hotspot_k"]["value"]["observed"],
        [300.0; 2],
        ROUNDING,
    );
}

fn fuse(hit: &Value, fixed: bool) {
    let metrics = &hit["metrics"];
    event(hit, "fault_applied", &element("fault", "channel"), SECOND);
    assert_eq!(metrics["wire"], element("fault", "wire"));
    assert_eq!(metrics["switch"], hit["event"]["branch"]);
    assert_eq!(metrics["interrupt_current"], "pass");
    assert_eq!(metrics["interrupt_voltage"], "pass");
    let clearing = if fixed {
        [0.015, 0.017]
    } else {
        [0.028, 0.030]
    };
    let i_sq = (12.0_f64 / (0.1 + 0.4 + 0.1)).powi(2);
    let wire = [8.0 / i_sq, 9.2 / i_sq];
    let switch = [0.025, 0.027];
    let margin = |[earliest, latest]: [f64; 2]| [earliest - clearing[1], latest - clearing[0]];
    let dose = &metrics["wire_damage"]["value"];
    bounds(&dose["window_i2t_a2s"], [i_sq * 0.035; 2], ROUNDING);
    bounds(&dose["damage_time_s"], wire, ROUNDING);
    bounds(&metrics["clearing_time_s"]["value"], clearing, ROUNDING);
    bounds(&metrics["switch_damage_time_s"]["value"], switch, ROUNDING);
    bounds(&metrics["wire_margin_s"], margin(wire), ROUNDING);
    bounds(&metrics["switch_margin_s"], margin(switch), ROUNDING);
}

fn thermal(hit: &Value, fixed: bool) {
    let metrics = &hit["metrics"];
    let (off, t) = if fixed {
        (SECOND / 2, 0.5_f64)
    } else {
        (2 * SECOND, 2.0)
    };
    event(hit, "mode_entered", &hit["branch"], off);
    let pulse = &metrics["linear_soa"]["value"];
    assert_eq!(pulse.as_array().unwrap().len(), 1);
    assert_eq!(pulse[0]["window"], window([0, off]));
    assert_eq!(pulse[0]["status"], "pass");
    bounds(&pulse[0]["vds_v"], [4.0; 2], ROUNDING);
    bounds(&pulse[0]["id_a"], [2.0; 2], ROUNDING);

    // Точный отклик двух тепловых RC на ступень 4 В × 2 А.
    let expected = 300.0 + 8.0 * ((1.0 - (-t).exp()) + 2.0 * (1.0 - (-t / 2.0).exp()));
    let peak = &metrics["foster_peak_junction_k"]["value"];
    bounds(&peak["observed"], [expected; 2], ROUNDING);
    bounds(&peak["margin"], [310.0 - expected; 2], ROUNDING);
}

fn discharge(hit: &Value, fixed: bool) {
    let metrics = &hit["metrics"];
    let (off, t) = if fixed {
        (SECOND + SECOND / 4, 1.25_f64)
    } else {
        (2 * SECOND, 2.0)
    };
    event(hit, "mode_entered", &hit["branch"], off);
    let cutoff = &metrics["cutoff"]["value"];
    assert_eq!(cutoff["detected_at_ns"], SECOND);
    assert_eq!(cutoff["disconnected_at_ns"], off);
    assert_eq!(cutoff["observed_delay_ns"], off - SECOND);
    assert_eq!(metrics["limiting_cell"], "weak");
    let cells = metrics["cells"].as_array().unwrap();
    assert_eq!(cells.len(), 2);
    for (cell, (id, initial_soc, capacity_c, self_discharge_a)) in cells
        .iter()
        .zip([("weak", 0.5, 10.0, 0.02), ("strong", 0.8, 20.0, 0.01)])
    {
        assert_eq!(cell["id"], id);
        // Ток саморазряда входит в зарядовый баланс, поляризация зависит от тока клемм.
        let loaded_a = 1.0 + self_discharge_a;
        let removed_c = loaded_a * t + (0.01 + self_discharge_a) * (3.0 - t);
        let soc = initial_soc - loaded_a * t / capacity_c;
        let minimum = 2.5 + 2.0 * soc - 0.1 - 0.2 * (1.0 - (-t).exp());
        // Концы огибающей с заданной в паспорте погрешностью модели 0,1 нВ.
        let error = 1e-10;
        bounds(
            &cell["minimum_voltage_v"]["value"],
            [minimum - error, minimum + error],
            ROUNDING,
        );
        bounds(
            &cell["margin_v"]["value"],
            [minimum - 3.0 - error, minimum - 3.0 + error],
            ROUNDING,
        );
        bounds(
            &cell["removed_charge_ah"]["value"],
            [removed_c / COULOMBS_PER_AH; 2],
            ROUNDING,
        );
        if id == "weak" && !fixed {
            assert_eq!(cell["violation"]["time_ns"], off);
        } else {
            assert!(cell["violation"].is_null(), "{cell}");
        }
    }
}

fn sleep(hit: &Value, fixed: bool) {
    let metrics = &hit["metrics"];
    let load = if fixed { 0.025 } else { 0.070 };
    let runtime = 1000.0;
    let controller_c = 900.0 * 0.005 + 100.0 * 0.105;
    let total_c = controller_c + runtime * (load + 0.002);
    let consum = &metrics["consumption"]["value"];
    bounds(
        &consum["controller_ah"],
        [controller_c / COULOMBS_PER_AH; 2],
        ROUNDING,
    );
    bounds(
        &consum["other_loads_ah"],
        [runtime * load / COULOMBS_PER_AH; 2],
        ROUNDING,
    );
    bounds(
        &consum["total_ah"],
        [total_c / COULOMBS_PER_AH; 2],
        ROUNDING,
    );
    // В конце последнего пробуждения SOC минимален, а I·R максимально.
    let v_margin = 3.0 + (1.0 - total_c / 100.0) - 0.5 * (0.105 + load) - 3.2;
    let error = 1e-9; // Погрешность линейной модели из входных данных.
    bounds(
        &metrics["cutoff_margin_v"]["value"],
        [v_margin - error, v_margin + error],
        ROUNDING,
    );
    if fixed {
        assert!(metrics["cutoff_violation"].is_null());
    } else {
        assert_eq!(metrics["cutoff_violation"]["state"], "sleep");
        assert_eq!(metrics["cutoff_violation"]["time_ns"], 900 * SECOND);
    }
}

fn cable(hit: &Value, fixed: bool) {
    let metrics = &hit["metrics"];
    let radius = if fixed { 0.011 } else { 0.009 };
    let length = 0.080 + 0.120 - 2.0 * radius + FRAC_PI_2 * radius;
    let actual_length = &metrics["maximum_length_m"]["value"];
    assert_eq!(actual_length["sample_at_ns"], SECOND);
    bounds(&actual_length["bounds"], [length; 2], ROUNDING);
    let actual_radius = &metrics["minimum_radius_m"]["value"];
    bounds(
        &json!([actual_radius["min_m"], actual_radius["max_m"]]),
        [radius; 2],
        ROUNDING,
    );
    bounds(
        &metrics["slack"]["result"]["value"],
        [0.197 - length; 2],
        ROUNDING,
    );
    let bend = &metrics["bend"]["result"]["value"];
    bounds(
        &json!([bend["min_m"], bend["max_m"]]),
        [radius - 0.010; 2],
        ROUNDING,
    );
    assert_eq!(metrics["relief"], json!(["pass", "pass"]));
    assert_eq!(
        metrics["bend"]["status"],
        if fixed { "pass" } else { "fail" }
    );
    if fixed {
        assert_eq!(metrics["fatigue"]["status"], "pass");
    }
}

fn current(component: &str, element: &str) -> Probe {
    Probe::Current(Section::Element {
        component: component.into(),
        element: element.into(),
    })
}

fn sample(series: &ElectricalSeries, probe: &Probe, at: u64, expected: f64) {
    let index = series.time.binary_search(&Time(at)).unwrap();
    let actual = series.values[probe][index];
    // Допуск MNA для резистивных ветвей и установившегося конденсатора.
    assert!(
        (actual - expected).abs() <= 1e-9,
        "{probe:?} в {at} нс: {actual} вместо {expected}"
    );
}

fn electrical_samples(
    case: &Case,
    path: &Path,
    catalog: &sim_process::schematic::Catalog,
    report: &Value,
    hit: &Value,
    fixed: bool,
) {
    let mode = match case.code {
        "fuse_coordination" => Mode::Stall,
        "capacitor_stress" | "switch_thermal" | "deep_discharge" => Mode::Normal,
        // Для автономности и геометрии источником являются профиль и траектория.
        _ => return,
    };
    let mode_report = report["assemblies"]["block"]["modes"]
        .as_array()
        .unwrap()
        .iter()
        .find(|row| row["mode"] == case.mode)
        .unwrap();
    assert_eq!(mode_report["nominal"], true);
    assert_eq!(mode_report["end_ns"], case.window[1]);
    assert!(mode_report["journal"].is_u64());

    let input = load_schematic(path, catalog).unwrap();
    let prepared = input.compiled.prepare_mode(&input.modes, &mode).unwrap();
    let Analysis::Analyzed {
        value: prepared, ..
    } = prepared
    else {
        panic!("{prepared:?}");
    };
    let Analysis::Analyzed { value: trace, .. } = prepared.solve() else {
        panic!("нет ряда расчёта для {}", case.code);
    };
    match case.code {
        "capacitor_stress" => {
            let terminal = |pin: &str| Endpoint {
                component: "cap".into(),
                pin: pin.into(),
            };
            let voltage = Probe::Voltage(terminal("a"), terminal("b"));
            for at in &trace.time {
                sample(&trace, &voltage, at.0, if fixed { 8.0 } else { 10.0 });
                sample(&trace, &current("cap", "c"), at.0, 0.0);
            }
        }
        "fuse_coordination" => {
            let start = hit["event"]["time_ns"].as_u64().unwrap();
            let fuse = current("fault", "fuse");
            sample(&trace, &fuse, start - 1, 0.0);
            sample(&trace, &fuse, start, 20.0);
            sample(&trace, &fuse, case.window[1], 20.0);
        }
        "switch_thermal" => {
            let off = hit["event"]["time_ns"].as_u64().unwrap();
            let channel = current("switch", "channel");
            sample(&trace, &channel, off - 1, 2.0);
            sample(&trace, &channel, off, 0.0);
        }
        "deep_discharge" => {
            let off = hit["event"]["time_ns"].as_u64().unwrap();
            let load = current("battery", "load");
            let protection = current("battery", "protection");
            sample(&trace, &load, off - 1, 1.0);
            sample(&trace, &load, off, 0.0);
            sample(&trace, &protection, off - 1, 0.0);
            sample(&trace, &protection, off, 0.01);
        }
        _ => unreachable!(),
    }
}

#[test]
fn шесть_пар_через_электрический_вердикт_cli() {
    let manifest = corpus().join("manifest.toml");
    let catalog = load_catalog(&corpus().join("catalog/electrical")).unwrap();
    let report = campaign(&manifest);
    let cases = report["assemblies"].as_object().unwrap();
    assert_eq!(cases.len(), 2 * CASES.len());
    for (case, compare) in CASES
        .iter()
        .zip([capacitor, fuse, thermal, discharge, sleep, cable] as [fn(&Value, bool); 6])
    {
        for (variant, status) in [("unsafe", "fail"), ("fixed", "pass")] {
            let result = &cases[&case.id(variant)];
            let hit = case.check(result, status, "reference");
            assert_eq!(hit["diagnostic_only"], false);
            let path = case.input(variant).join("scheme.toml");
            let bytes = fs::read(&path).unwrap();
            assert_eq!(hit["observation_input"], json!(Fingerprint::of(&bytes)));
            compare(hit, variant == "fixed");
            electrical_samples(
                case,
                &path,
                &catalog.catalog,
                result,
                hit,
                variant == "fixed",
            );
        }
    }
    assert_eq!(report, campaign(&manifest));
}

struct Scratch(PathBuf);

impl Scratch {
    fn new(label: &str) -> Self {
        let path =
            std::env::temp_dir().join(format!("sim-depth-t3-{}-{label}", std::process::id()));
        fs::create_dir(&path).unwrap();
        Self(path)
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

fn read(path: &Path) -> Toml {
    toml::from_str(&fs::read_to_string(path).unwrap()).unwrap()
}

fn remove(value: &mut Toml, key: &str) -> Toml {
    value.as_table_mut().unwrap().remove(key).unwrap()
}

fn save(path: &Path, value: &impl serde::Serialize) {
    fs::write(path, toml::to_string(value).unwrap()).unwrap();
}

fn edited(
    label: &str,
    variant: &str,
    cases: &[Case],
    edit: impl Fn(&Case, &mut Toml, &mut Toml),
) -> (Scratch, PathBuf) {
    let scratch = Scratch::new(label);
    let dest = &scratch.0;
    let catalog = Path::new("catalog/electrical");
    fs::create_dir_all(dest.join(catalog)).unwrap();
    let mut parts = read(&corpus().join(catalog).join("parts.toml"));
    if label == "assembly" {
        // Разрешаем загрузку эквивалентов, чтобы проверить именно барьер паспорта или отметки.
        for (_, part) in parts["parts"].as_table_mut().unwrap().iter_mut() {
            remove(part, "reference_only");
        }
    }
    save(&dest.join(catalog).join("parts.toml"), &parts);
    let mut ratings = read(&corpus().join(catalog).join("ratings.toml"));
    let mut assemblies = Table::new();
    for case in cases {
        let src = case.input(variant);
        let dir = dest.join(case.code);
        fs::create_dir(&dir).unwrap();
        for name in ["world.toml", "device.toml"] {
            fs::copy(src.join(name), dir.join(name)).unwrap();
        }
        let mut scheme = read(&src.join("scheme.toml"));
        edit(case, &mut scheme, &mut ratings);
        save(&dir.join("scheme.toml"), &scheme);
        let entry = Table::from_iter([(
            "world".into(),
            Toml::String(format!("{}/world.toml", case.code)),
        )]);
        assemblies.insert(case.code.into(), Toml::Table(entry));
    }
    save(&dest.join(catalog).join("ratings.toml"), &ratings);
    let manifest = dest.join("manifest.toml");
    save(
        &manifest,
        &Table::from_iter([
            ("electrical_format".into(), Toml::Integer(1)),
            ("catalog".into(), Toml::String("catalog".into())),
            ("assemblies".into(), Toml::Table(assemblies)),
        ]),
    );
    (scratch, manifest)
}

#[test]
fn потеря_входа_оставляет_каждую_опасность_открытой() {
    let (_scratch, manifest) = edited("missing", "unsafe", &CASES, |case, scheme, _| {
        let check = &mut scheme["depth_checks"][0];
        if case.code == "fuse_coordination" {
            remove(check, "event");
            return;
        }
        let input = &mut check["check"];
        match case.code {
            "capacitor_stress" => remove(&mut input["series"], "voltage_v"),
            "switch_thermal" => remove(&mut input["segments"][0], "id_a"),
            "deep_discharge" => remove(&mut input["cells"][0], "min_voltage_v"),
            "sleep_drain" => remove(
                &mut input["profile"]["states"][0]["loads"][0],
                "battery_current_a",
            ),
            "cable_flex" => remove(input, "motion"),
            _ => unreachable!(),
        };
    });
    let report = campaign(&manifest);
    for case in &CASES {
        let hit = case.check(
            &report["assemblies"][case.code],
            "not_analyzed",
            "reference",
        );
        let metrics = &hit["metrics"];
        let metric = match case.code {
            "capacitor_stress" => &metrics["voltage_v"],
            "switch_thermal" => &metrics["foster_peak_junction_k"],
            "deep_discharge" => &metrics["cells"][0]["margin_v"],
            "sleep_drain" => &metrics["consumption"],
            "cable_flex" => &metrics["minimum_radius_m"],
            "fuse_coordination" => {
                assert!(hit["event"].is_null());
                assert!(
                    hit["applicability_gap"]
                        .as_str()
                        .unwrap()
                        .contains("инъекция")
                );
                assert!(metrics["clearing_time_s"]["value"].is_array());
                continue;
            }
            _ => unreachable!(),
        };
        assert_eq!(metric["status"], "not_analyzed", "{hit}");
        assert!(!metric["reason"].as_str().unwrap().trim().is_empty());
    }
}

#[test]
fn пределы_без_источника_и_стендовая_область_блокируют_допуск() {
    let (_scratch, manifest) = edited("untrusted", "fixed", &CASES, |case, scheme, ratings| {
        let input = &mut scheme["depth_checks"][0]["check"];
        let limit = match case.code {
            "capacitor_stress" => {
                &mut ratings["depth_ratings"]["capacitor_bench_8"]["data"]["rated_voltage_v"]
            }
            "fuse_coordination" => {
                &mut ratings["depth_ratings"]["fuse_fast"]["data"]["tcc"]["bands"][0]["time_s"]
            }
            "switch_thermal" => {
                &mut ratings["depth_ratings"]["switch_bench"]["data"]["max_junction_k"]
            }
            "deep_discharge" => &mut input["cells"][0]["min_voltage_v"],
            "sleep_drain" => &mut input["battery"]["cutoff_v"],
            "cable_flex" => {
                &mut ratings["depth_ratings"]["cable_bench"]["data"]["min_bend_radius_m"]
            }
            _ => unreachable!(),
        };
        remove(limit, "source");
    });
    let report = campaign(&manifest);
    for case in &CASES {
        case.check(
            &report["assemblies"][case.code],
            "not_analyzed",
            "reference",
        );
    }

    let (_assembly, manifest) = edited("assembly", "fixed", &CASES, |case, scheme, _| {
        scheme["scope"] = Toml::String("assembly".into());
        if !matches!(case.code, "deep_discharge" | "sleep_drain") {
            // Для проверок с каталогом ограничение задано самим паспортом.
            remove(&mut scheme["depth_checks"][0], "reference_only");
        }
    });
    let report = campaign(&manifest);
    for case in &CASES {
        let hit = case.check(&report["assemblies"][case.code], "not_analyzed", "assembly");
        assert!(
            hit["applicability_gap"]
                .as_str()
                .unwrap()
                .contains("эталонного стенда")
        );
        assert!(hit["metrics"].is_null(), "{hit}");
    }
}

fn legacy_signature(report: &Value) -> Vec<[Value; 4]> {
    report["findings"]
        .as_array()
        .unwrap()
        .iter()
        .map(|row| ["key", "code", "outcome", "reason"].map(|field| row[field].clone()))
        .collect()
}

#[test]
fn прежние_находки_совпадают_без_новых_полей() {
    let full = campaign(&corpus().join("manifest.toml"));
    let (_scratch, manifest) = edited("legacy", "fixed", &CASES, |_, scheme, _| {
        remove(scheme, "depth_checks");
    });
    let legacy = campaign(&manifest);
    for case in &CASES {
        let old = &legacy["assemblies"][case.code];
        let new = &full["assemblies"][case.id("fixed")];
        assert!(old["assemblies"]["block"].get("depth_checks").is_none());
        assert!(!old["findings"].as_array().unwrap().is_empty());
        assert_eq!(
            legacy_signature(old),
            legacy_signature(new),
            "{}",
            case.code
        );
        assert_eq!(old["simulation"], new["simulation"], "{}", case.code);
    }
}

#[test]
fn неконечное_наблюдение_отклоняется_cli() {
    for (label, value) in [("nan", f64::NAN), ("infinite", f64::INFINITY)] {
        let (_scratch, manifest) = edited(label, "fixed", &CASES[..1], |_, scheme, _| {
            scheme["depth_checks"][0]["check"]["series"]["voltage_v"][0] = Toml::Float(value);
        });
        let (output, report) = launch(&manifest);
        assert_eq!(output.status.code(), Some(2), "{report}");
        assert_eq!(report["status"], "error", "{report}");
        assert_eq!(report["blocks_proof"], true, "{report}");
        let reason = report["detail"].as_str().unwrap();
        assert!(
            reason.contains("depth_checks[0].check.series.voltage_v[0]"),
            "{reason}"
        );
        assert!(reason.contains("конечные числа"), "{reason}");
    }
}
