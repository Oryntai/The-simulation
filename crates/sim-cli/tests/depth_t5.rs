//! Штатный электрический отчёт для синтетического блока. Оракул — expected.md.

use std::{
    fs,
    path::{Path, PathBuf},
    process::{Command, Output},
};

use serde::Deserialize;
use serde_json::Value;
use sim_cli::electrical::solder_input::{ResolvedSolderProfile, SolderCatalog, SolderProfile};
use sim_cli::electrical::{load_catalog, load_schematic};
use sim_process::{Analysis, Interval, StepModel, work::heat::solder_network::SolderNetwork};
use sim_signal::{Rng, Time};
use sim_task::{
    Fingerprint, Outcome,
    interaction::safety_decision,
    verdict::{Finding, HazardClass, Readiness, SimulationVerdict},
};
use toml::Value as Toml;

#[path = "fixtures/electrical_verdict.rs"]
mod electrical_verdict;

const JOINT: &str = "стык-т5-эталон";
const STEPS: u32 = 1024; // Сетка теста solder_window.rs, шаг до 1,8 мс для корпуса.

#[derive(Deserialize)]
struct Corpus {
    scope: String,
    revision: String,
    scheme: PathBuf,
    component_catalog: PathBuf,
    solder_catalog: PathBuf,
    finding: String,
    pairs: Vec<Pair>,
    negative: Vec<Negative>,
}

#[derive(Deserialize)]
struct Pair {
    id: String,
    criterion: String,
    #[serde(rename = "unsafe")]
    dangerous: PathBuf,
    fixed: PathBuf,
    margin: Margin,
}

#[derive(Deserialize)]
struct Margin {
    #[serde(rename = "unsafe")]
    dangerous: f64,
    fixed: f64,
    unit: String,
}

#[derive(Deserialize)]
struct Negative {
    id: String,
    profile: PathBuf,
    path: String,
    remove: Option<bool>,
    replace_toml: Option<String>,
    expected: String,
}

struct Block {
    dir: PathBuf,
    assemblies: String,
}

impl Drop for Block {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.dir).expect("удаление временного стенда");
    }
}

impl Block {
    fn new(label: &str) -> (Self, Corpus) {
        let dir = std::env::temp_dir().join(format!("sim-depth-t5-{}-{label}", std::process::id()));
        fs::create_dir(&dir).unwrap();
        let block = Self {
            dir,
            assemblies: String::new(),
        };
        // Оба прогона сравнения читают один исполняемый файл и его отпечаток.
        fs::copy(env!("CARGO_BIN_EXE_sim-run"), block.dir.join("sim-run")).unwrap();
        let corpus: Corpus = toml::from_str(&block.read_corpus("manifest.toml")).unwrap();
        fs::create_dir_all(block.dir.join("catalog/electrical")).unwrap();
        for (src, dest) in [
            (corpus.scheme.as_path(), "scheme.toml"),
            (
                corpus.component_catalog.as_path(),
                "catalog/electrical/parts.toml",
            ),
            (corpus.solder_catalog.as_path(), "solder.toml"),
        ] {
            block.write(dest, block.read_corpus(src));
        }
        block.write("device.toml", r#"
format = 3
name = "Блок"
schematic = "scheme.toml"
[[links]]
name = "корпус"
joint = { kind = "slide", axis = [1.0, 0.0, 0.0] }
# Куб 1 м, 6 кг, I = m·a²/6 = 1 кг·м².
mass = { kind = "explicit", mass_kg = 6.0, com_m = [0.0, 0.0, 0.0], inertia_kg_m2 = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0], provenance = { derived = "Формула инерции куба" } }
"#);
        block.write(
            "world.toml",
            r#"
format = 3
name = "Блок"
seed = 1 # Фиксированное зерно стенда без разброса.
physics_hz = 1000 # Сетка носителя из тестового стенда electrical_verdict.
gravity_m_s2 = [0.0, 0.0, 0.0] # Внешние силы в постановке отсутствуют.
[environment]
density_kg_m3 = 0.0
viscosity_pa_s = 0.0
wind_m_s = [0.0, 0.0, 0.0]
[[devices]]
name = "блок"
file = "device.toml"
"#,
        );
        (block, corpus)
    }

    fn read_corpus(&self, path: impl AsRef<Path>) -> String {
        fs::read_to_string(
            Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../devices/electrical/depth_t5")
                .join(path),
        )
        .unwrap()
    }

    fn write(&self, path: &str, text: impl AsRef<[u8]>) {
        fs::write(self.dir.join(path), text).unwrap();
    }

    fn case(&mut self, name: &str, profile: &str, series: Option<&str>) {
        self.write(&format!("{name}-profile.toml"), profile);
        let series = series.map_or_else(String::new, |series| format!("series = \"{series}\"\n"));
        self.write(
            &format!("{name}-setup.toml"),
            format!(
                r#"
format = 1
catalog = "solder.toml"
steps = {STEPS}
[instances."блок"."{JOINT}"]
profile = "{name}-profile.toml"
{series}"#
            ),
        );
        self.assemblies.push_str(&format!(
            r#"
[assemblies.{name}]
world = "world.toml"
solder = "{name}-setup.toml"
"#
        ));
    }

    fn launch(&self) -> Output {
        self.write(
            "campaign.toml",
            format!(
                "electrical_format = 1\ncatalog = \"catalog\"\n{}",
                self.assemblies
            ),
        );
        Command::new(self.dir.join("sim-run"))
            .arg("--campaign")
            .arg(self.dir.join("campaign.toml"))
            .arg("--json")
            .output()
            .unwrap()
    }

    fn report(&self) -> Value {
        let output = self.launch();
        assert!(output.status.success(), "{output:?}");
        serde_json::from_slice(&output.stdout).unwrap()
    }

    fn assert_captured(&self, report: &Value, file: &str) {
        let path = self.dir.join(file).canonicalize().unwrap();
        assert_eq!(
            report["sources"][path.to_str().unwrap()].as_u64(),
            Some(Fingerprint::of(&fs::read(path).unwrap()).as_u64())
        );
    }
}

fn joint(report: &Value) -> &Value {
    &report["assemblies"]["блок"]["solder_checks"][JOINT]
}

fn solder_finding(report: &Value) -> &Value {
    let findings = report["findings"].as_array().unwrap();
    let mut solder = findings
        .iter()
        .filter(|item| item["code"] == "solder_defect");
    let finding = solder.next().unwrap();
    assert!(solder.next().is_none());
    assert_eq!(finding["key"]["class"], "solder_defect");
    assert_eq!(finding["key"]["object"], format!("блок/{JOINT}"));
    assert_eq!(finding["electrical"]["build"], report["build"]);
    finding
}

fn bounds(value: &Value) -> [f64; 2] {
    serde_json::from_value(value.clone()).unwrap()
}

fn includes(bounds: [f64; 2], expected: f64, tolerance: f64) {
    assert!(
        bounds[0] <= expected + tolerance && bounds[1] >= expected - tolerance,
        "{bounds:?}, ожидается {expected}, допуск {tolerance}"
    );
}

#[test]
fn пять_пар_проверяют_окно_и_вердикт_через_cli() {
    let (mut block, corpus) = Block::new("pairs");
    assert_eq!(corpus.pairs.len(), 5); // Пять классов постановки по Т5.
    for pair in &corpus.pairs {
        for (variant, source) in [("unsafe", &pair.dangerous), ("fixed", &pair.fixed)] {
            block.case(
                &format!("{}_{variant}", pair.id),
                &block.read_corpus(source),
                None,
            );
        }
    }
    let first = block.launch();
    assert!(first.status.success(), "{first:?}");
    assert!(first.stdout == block.launch().stdout, "отчёты одного образа отличаются побайтно");
    let report: Value = serde_json::from_slice(&first.stdout).unwrap();
    let solder_catalog = SolderCatalog::load(&block.dir.join("solder.toml"))
        .unwrap()
        .data;
    let mut max_peak_error: f64 = 0.0;
    let mut max_energy_error: f64 = 0.0;
    for pair in &corpus.pairs {
        for (variant, is_dangerous, expected, source) in [
            ("unsafe", true, pair.margin.dangerous, &pair.dangerous),
            ("fixed", false, pair.margin.fixed, &pair.fixed),
        ] {
            let case = format!("{}_{variant}", pair.id);
            let report = &report["assemblies"][&case];
            let joint = joint(report);
            assert_eq!(joint["address"]["scope"], corpus.scope);
            assert_eq!(joint["address"]["connection"], JOINT);
            assert_eq!(joint["revision"], corpus.revision);
            assert_eq!(joint["code"], corpus.finding);
            // tfix1 №1: у стендов корпуса нет фазы остывания. Исправление
            // своего критерия оставляет обоим верхним пределам открытое окно.
            let status = if is_dangerous { "fail" } else { "not_analyzed" };
            assert_eq!(joint["window_status"], status, "{case}: {joint}");
            let window = &joint["window"]["value"];
            let checks = window["checks"].as_object().unwrap();
            assert_eq!(checks.len(), 7); // Семь критериев SolderCriterion.
            for (criterion, check) in checks {
                assert_eq!(
                    check["status"],
                    if criterion == &pair.criterion && is_dangerous {
                        "fail"
                    } else if matches!(criterion.as_str(), "max_above_liquidus_s" | "max_component_k") {
                        assert!(check["reason"].as_str().unwrap().contains("нет фазы остывания"));
                        "not_analyzed"
                    } else {
                        "pass"
                    },
                    "{case}/{criterion}: {check}"
                );
            }
            let target = &checks[&pair.criterion];
            assert_eq!(target["unit"], pair.margin.unit);
            assert!(target["limit"]["source"]["reference"].is_string());
            let range = bounds(&target["margin"]);
            // Таблица округляет время и температуры до 9 знаков. Для объёма
            // учитываем округление произведения и вычитания в единицах СИ.
            includes(
                range,
                expected,
                if pair.margin.unit == "m3" {
                    1e-25
                } else {
                    5e-10
                },
            );
            assert!(
                if is_dangerous {
                    range[1] < 0.0
                } else {
                    range[0] > 0.0
                },
                "{range:?}"
            );
            max_peak_error = max_peak_error.max(analytical_check(
                &block.read_corpus(source),
                &window["thermal"]["value"],
                checks,
                &pair.criterion,
                is_dangerous,
            ));
            let profile = SolderProfile::load(&block.dir.join(format!("{case}-profile.toml")))
                .unwrap()
                .data;
            max_energy_error = max_energy_error.max(energy_check(&profile, &solder_catalog));
            println!(
                "{case}. Ручной запас {expected:+.9e} {}, огибающая {range:?}",
                pair.margin.unit
            );

            let finding = solder_finding(report);
            assert_eq!(finding["outcome"], status);
            assert_eq!(finding["confirmed_hazard"], is_dangerous);
            for other in report["findings"]
                .as_array()
                .unwrap()
                .iter()
                .filter(|f| f["code"] != "solder_defect")
            {
                assert_ne!(other["outcome"], "fail", "{case}: {other}");
            }
            assert_eq!(
                report["simulation"],
                if is_dangerous {
                    "Rejected"
                } else {
                    "NotProven"
                }
            );
            assert_eq!(joint["series"]["resistance_ohm"]["status"], "not_analyzed");
            assert_eq!(joint["series"]["strength_n"]["status"], "not_analyzed");
            for file in [
                "solder.toml".into(),
                format!("{case}-setup.toml"),
                format!("{case}-profile.toml"),
            ] {
                block.assert_captured(report, &file);
            }
        }
    }
    println!(
        "Наибольшее отклонение границ по приросту T {:.3e}%. Невязка энергии {:.3e} Дж.",
        max_peak_error * 100.0,
        max_energy_error
    );
}

fn analytical_check(
    text: &str,
    thermal: &Value,
    checks: &serde_json::Map<String, Value>,
    criterion: &str,
    dangerous: bool,
) -> f64 {
    let profile: Toml = toml::from_str(text).unwrap();
    let mode = &profile["mode"];
    let limits = &profile["limits"];
    let nominal = |value: &Toml| value["nominal"].as_float().unwrap();
    let duration = nominal(&mode["duration_s"]);
    // expected.md: 528 К в стационаре, компоненты моды 2 и 3, τ = 1 с.
    let amplitude = (528.0 - nominal(&mode["pad_initial_k"])) / 2.0;
    let crossing = |threshold: f64| (2.0 * amplitude / (528.0 - threshold)).ln();
    let wetting = crossing(nominal(&limits["wetting_k"]));
    let above = duration - crossing(493.15); // Ликвидус Indalloy 256, expected.md.
    let wet = &thermal["time_to_wet_s"]["value"];
    assert_eq!(wet["state"], "reached");
    includes(bounds(&wet["bounds"]), wetting, 0.0);
    includes(bounds(&thermal["above_liquidus_s"]["value"]), above, 0.0);
    let decay = (-duration).exp();
    let mut max_error: f64 = 0.0;
    for (field, factor) in [("peak_pad_k", 2.0), ("peak_component_k", 3.0)] {
        let peak = 528.0 - factor * amplitude * decay;
        let rise = factor * amplitude * (1.0 - decay);
        let range = bounds(&thermal[field]);
        includes(range, peak, 0.0);
        // ТЗ В: вся огибающая в пределах 5% аналитического прироста.
        let relative = (range[0] - peak).abs().max((range[1] - peak).abs()) / rise;
        assert!(relative <= 0.05);
        max_error = max_error.max(relative);
    }
    // expected.md: мерная заготовка (0,5 мм)², пределы длины 1 и 2 мм,
    // отступ вариантов на 0,1 мм и базовый отрезок 1,5 мм.
    let length_mm = match (criterion, dangerous) {
        ("bridge_volume_m3", true) => 2.1,
        ("bridge_volume_m3", false) => 1.9,
        ("min_volume_m3", true) => 0.9,
        ("min_volume_m3", false) => 1.1,
        _ => 1.5,
    };
    let volume = |length_mm: f64| (0.5e-3_f64).powi(2) * (length_mm * 1e-3);
    let dose = volume(length_mm);
    let limit = |key| nominal(&limits[key]);
    for (key, expected) in [
        ("max_time_to_wet_s", limit("max_time_to_wet_s") - wetting),
        (
            "min_above_liquidus_s",
            above - limit("min_above_liquidus_s"),
        ),
        (
            "max_above_liquidus_s",
            limit("max_above_liquidus_s") - above,
        ),
        (
            "max_pad_k",
            limit("max_pad_k") - (528.0 - 2.0 * amplitude * decay),
        ),
        (
            "max_component_k",
            limit("max_component_k") - (528.0 - 3.0 * amplitude * decay),
        ),
        ("bridge_volume_m3", volume(2.0) - dose),
        ("min_volume_m3", dose - volume(1.0)),
    ] {
        let tolerance = if key.ends_with("_m3") { 1e-25 } else { 0.0 };
        if matches!(key, "max_above_liquidus_s" | "max_component_k") {
            assert!(checks[key]["margin"].is_null() && expected > 0.0);
        } else {
            includes(bounds(&checks[key]["margin"]), expected, tolerance);
        }
    }
    max_error
}

fn energy_check(profile: &SolderProfile, catalog: &SolderCatalog) -> f64 {
    let resolved = ResolvedSolderProfile {
        profile,
        alloy: catalog.alloys.get(profile.alloy.as_ref().unwrap()),
        tool: catalog.tools.get(profile.tool.as_ref().unwrap()),
    };
    let (spec, input) = resolved.thermal_inputs();
    let duration = profile.mode.duration_s.as_ref().unwrap().bounds()[1];
    let mut network = SolderNetwork::new(spec).unwrap();
    let Analysis::Analyzed { value, .. } = network
        .advance(
            Interval {
                start: Time::ZERO,
                end: Time::from_secs(duration).unwrap(),
            },
            &input,
            &mut Rng::stream(0, "баланс блока без случайности"),
        )
        .unwrap()
    else {
        panic!("ожидался тепловой расчёт стенда")
    };
    let (start, end) = (value.start_k, value.end_k);
    // expected.md: C в порядке жало, площадка, вывод, медь. Вход 21,6 Вт.
    let stored = 0.04 * (end.tip - start.tip)
        + 0.015 * (end.pad - start.pad)
        + 0.01 * (end.lead - start.lead)
        + 0.02 * (end.copper - start.copper);
    let supplied = 21.6 * duration;
    // Независимый интеграл потока 0,20 Вт/К × (Tмеди(t) − 300 К).
    let amplitude = 408.0 - start.copper;
    let polygon = supplied - 0.20 * amplitude * (1.0 - (-duration).exp());
    let error = (value.supplied_j - supplied)
        .abs()
        .max((value.to_polygon_j - polygon).abs())
        .max((stored + value.to_polygon_j - supplied).abs());
    // Бюджет округления из solder_network.rs, проверка первого закона.
    assert!(
        error <= 4096.0 * f64::EPSILON * supplied,
        "невязка {error} Дж"
    );
    error
}

#[test]
fn пропуски_оставляют_окно_открытым_неконечные_входы_дают_ошибку() {
    let (mut block, corpus) = Block::new("negative");
    assert_eq!(corpus.negative.len(), 6); // Шесть отрицательных вариантов manifest.toml.
    for case in corpus.negative {
        block.assemblies.clear();
        let mut profile: Toml = toml::from_str(&block.read_corpus(case.profile)).unwrap();
        let mut path = case.path.rsplitn(2, '.');
        let key = path.next().unwrap();
        let parent = path
            .next()
            .unwrap()
            .split('.')
            .fold(&mut profile, |node, field| &mut node[field]);
        if case.remove == Some(true) {
            assert!(parent.as_table_mut().unwrap().remove(key).is_some());
        } else {
            let update: toml::Table =
                toml::from_str(&format!("value = {}", case.replace_toml.unwrap())).unwrap();
            parent[key] = update["value"].clone();
        }
        block.case(&case.id, &toml::to_string(&profile).unwrap(), None);
        if case.expected == "Error" {
            let output = block.launch();
            assert!(!output.status.success(), "{}", case.id);
            let diagnostic = format!(
                "{}{}",
                String::from_utf8_lossy(&output.stdout),
                String::from_utf8_lossy(&output.stderr)
            );
            assert!(
                diagnostic.contains(&case.path) && diagnostic.contains("конечные"),
                "{diagnostic}"
            );
        } else {
            assert_eq!(case.expected, "NotAnalyzed");
            let report = block.report();
            let report = &report["assemblies"][&case.id];
            assert_eq!(
                joint(report)["window_status"],
                "not_analyzed",
                "{}",
                case.id
            );
            assert_eq!(solder_finding(report)["outcome"], "not_analyzed");
            assert_eq!(report["simulation"], "NotProven");
        }
    }

    block.assemblies.clear();
    let profile = block.read_corpus(&corpus.pairs[0].fixed);
    block.case("address", &profile.replace(JOINT, "другой стык"), None);
    let output = block.launch();
    assert!(!output.status.success());
    let diagnostic = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(
        diagnostic.contains("профиль адресован другому стыку"),
        "{diagnostic}"
    );
}

#[test]
fn старый_формат_и_выполненное_окно_без_серии_не_доказывают_сборку() {
    let (mut block, corpus) = Block::new("hardware");
    let scheme = block
        .read_corpus(&corpus.scheme)
        .replace("scope = \"reference\"", "scope = \"assembly\"");
    let parts = block
        .read_corpus(&corpus.component_catalog)
        .replace("reference_only = true\n", "");
    block.write("scheme.toml", scheme);
    block.write("catalog/electrical/parts.toml", parts);
    block
        .assemblies
        .push_str("[assemblies.legacy]\nworld = \"world.toml\"\n");
    let profile = cooled_profile(&block, &corpus);
    block.case("window", &profile, None);
    let report = block.report();
    for (name, expected) in [("legacy", "not_analyzed"), ("window", "pass")] {
        let report = &report["assemblies"][name];
        assert_eq!(joint(report)["window_status"], expected);
        let finding = solder_finding(report);
        assert_eq!(finding["outcome"], "not_analyzed");
        assert_eq!(joint(report)["status"], finding["outcome"]);
        assert_eq!(joint(report)["scope"], "assembly");
        assert_eq!(joint(report)["series"]["id"], Value::Null);
        assert_eq!(report["simulation"], "NotProven", "{report}");
        let finding: Finding = serde_json::from_value(finding.clone()).unwrap();
        assert_eq!(finding.key.class, HazardClass::SolderDefect);
        // Проверяем барьер даже при выполненных остальных обязательствах.
        let outcomes = [Outcome::Pass, finding.outcome];
        assert_eq!(Outcome::aggregate(outcomes), Outcome::NotAnalyzed);
        assert_eq!(safety_decision(outcomes, true), Readiness::BenchOnly);
    }
    let catalog = load_catalog(&block.dir.join("catalog/electrical")).unwrap();
    let scheme = load_schematic(&block.dir.join("scheme.toml"), &catalog.catalog).unwrap();
    let stand = electrical_verdict::ElectricalStand::new("solder-window");
    let verdict = stand.assess(&scheme);
    let finding = verdict
        .findings
        .iter()
        .find(|f| f.key.class == HazardClass::SolderDefect)
        .unwrap();
    assert_eq!(finding.outcome, Outcome::NotAnalyzed);
    assert_eq!(verdict.simulation, SimulationVerdict::NotProven);
    assert_ne!(verdict.readiness, Readiness::Passed);
}

fn cooled_profile(block: &Block, corpus: &Corpus) -> String {
    let mut profile: Toml = toml::from_str(&block.read_corpus(&corpus.pairs[0].fixed)).unwrap();
    let mut catalog: Toml =
        toml::from_str(&fs::read_to_string(block.dir.join("solder.toml")).unwrap()).unwrap();
    let set = |value: &mut Toml, number: f64| {
        for bound in ["min", "nominal", "max"] {
            value[bound] = Toml::Float(number);
        }
        value["source"]["datasheet"] = Toml::String("Аналитическая мода охлаждения теста".into());
    };
    // tfix1 №1: используем точную моду охлаждения из sim-process/tests/solder_joint.rs.
    // T = 460 + 20 × [3, 2, 3, 1] × exp(−t/1 с); при t = 1 с оба узла
    // холоднее 493,15 К, выдержка ln(40 / 33,15) > 0,1 с.
    set(
        &mut catalog["tools"]["bench-t5-r1"]["tip_capacity_j_k"],
        1.0 / 3.0,
    );
    for (field, number) in [
        ("pad_capacity_j_k", 0.5),
        ("lead_capacity_j_k", 1.0 / 3.0),
        ("copper_capacity_j_k", 1.0),
        ("pad_lead_w_k", 1.0),
        ("pad_copper_w_k", 3.0),
        ("copper_polygon_w_k", 4.0),
        ("polygon_k", 460.0),
    ] {
        set(&mut profile["geometry"][field], number);
    }
    for (field, number) in [
        ("tip_initial_k", 520.0),
        ("pad_initial_k", 500.0),
        ("lead_initial_k", 520.0),
        ("copper_initial_k", 480.0),
        ("requested_thermal_power_w", 0.0),
        ("duration_s", 1.0),
    ] {
        set(&mut profile["mode"][field], number);
    }
    set(&mut profile["mode"]["contact"]["conductance_w_k"], 1.0);
    set(&mut profile["limits"]["min_above_liquidus_s"], 0.1);
    // Предел на 10 К выше начальных 520 К, как на стенде переноса серии.
    set(&mut profile["limits"]["max_component_k"], 530.0);
    block.write("solder.toml", toml::to_string(&catalog).unwrap());
    toml::to_string(&profile).unwrap()
}

#[test]
fn серия_переносит_замеры_только_для_своего_стыка_ревизии_и_режима() {
    let (mut block, corpus) = Block::new("series");
    let profile = cooled_profile(&block, &corpus);
    block.write("calibration.toml", &profile);
    // Синтетические чтения
    // задают эталон переноса: падение 1, 2, 3 мВ при 1 А и разрушение
    // в трёх опытах при F = 2 Н/мм × (3, 3,5, 4 мм).
    let series = format!(
        r#"
format = 1
id = "контроль блока"
revision = "{}"
schematic = "Блок"
scope = "reference"
connection = "{JOINT}"
ends = ["площадка/p", "вывод/p"]
[applicability]
calibration_profile = "calibration.toml"
force_n = [1.0, 1.0]
duration_s = [1.0, 1.0]
pitch_m = [0.001, 0.001]
dose_m3 = [3.75e-10, 3.75e-10]
source.measured = "Синтетический протокол покрытия точки стенда"
[four_wire_resistance_ohm]
range = {{ min = 0.001, nominal = 0.002, max = 0.003, source.measured = "Синтетические падения напряжения при 1 А" }}
conditions = "Четыре провода, три условные пробы"
[destructive_strength_n]
range = {{ min = 6.0, nominal = 7.0, max = 8.0, source.measured = "Синтетические нагрузки при разрушении, по оснастке" }}
conditions = "Одноосное растяжение, три условные пробы"
"#,
        corpus.revision
    );
    block.write("valid.toml", &series);
    block.case("valid", &profile, Some("valid.toml"));
    block.case(
        "revision",
        &profile.replace(
            &format!("revision = \"{}\"", corpus.revision),
            "revision = \"другая ревизия\"",
        ),
        Some("valid.toml"),
    );
    for (name, text) in [
        (
            "ends",
            series.replace("\"площадка/p\", \"вывод/p\"", "\"вывод/p\", \"площадка/p\""),
        ),
        ("domain", series.replace("[1.0, 1.0]\npitch_m", "[1.25, 1.25]\npitch_m")),
        (
            "source",
            series.replace(
                "source.measured = \"Синтетический протокол",
                "source.datasheet = \"Синтетический протокол",
            ),
        ),
        (
            "resistance",
            series.replace(
                "source.measured = \"Синтетические падения",
                "source.estimate = \"Синтетические падения",
            ),
        ),
    ] {
        let file = format!("{name}.toml");
        block.write(&file, text);
        block.case(name, &profile, Some(&file));
    }
    let report = block.report();
    let valid = &report["assemblies"]["valid"];
    let series = &joint(valid)["series"];
    assert_eq!(series["id"], "контроль блока");
    assert_eq!(series["revision"], corpus.revision);
    assert!(series["applicability"]["value"].is_object(), "{series}");
    for (record, expected) in [
        (
            &series["resistance_ohm"],
            [1e-3 / 1.0, 2e-3 / 1.0, 3e-3 / 1.0],
        ),
        (&series["strength_n"], [2.0 * 3.0, 2.0 * 3.5, 2.0 * 4.0]),
    ] {
        let range = &record["value"]["range"];
        assert_eq!(
            [
                range["min"].as_f64().unwrap(),
                range["nominal"].as_f64().unwrap(),
                range["max"].as_f64().unwrap()
            ],
            expected
        );
        assert_eq!(range["source"]["kind"], "measured");
        assert!(record["value"]["conditions"].is_string());
    }
    for file in ["valid.toml", "calibration.toml"] {
        block.assert_captured(valid, file);
    }
    for (name, reason) in [
        ("revision", "ревизия"),
        ("ends", "другому стыку"),
        ("domain", "вне области"),
        ("source", "источника совместной огибающей"),
    ] {
        let report = &report["assemblies"][name];
        let joint = joint(report);
        assert_eq!(joint["window_status"], "pass", "{name}: {joint}");
        let series = &joint["series"];
        assert!(
            series["applicability"]["reason"]
                .as_str()
                .unwrap()
                .to_lowercase()
                .contains(reason),
            "{series}"
        );
        for measurement in ["resistance_ohm", "strength_n"] {
            assert_eq!(series[measurement]["status"], "not_analyzed");
        }
        assert_eq!(report["simulation"], "NotProven");
    }
    let series = &joint(&report["assemblies"]["resistance"])["series"];
    assert_eq!(series["resistance_ohm"]["status"], "not_analyzed");
    assert!(series["strength_n"]["value"].is_object());
    assert_eq!(valid["simulation"], "NotProven");
}
