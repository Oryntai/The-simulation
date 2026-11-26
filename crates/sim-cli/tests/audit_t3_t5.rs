//! Независимые возмущения штатных данных Т3 и Т5 через sim-run.

use std::{
    fs,
    path::{Path, PathBuf},
    process::{Command, Output},
};

use serde_json::Value;
use toml::Value as Toml;

struct Stand(PathBuf);

impl Stand {
    fn new(label: &str) -> Self {
        let path = std::env::temp_dir().join(format!("audit-t3-t5-{}-{label}", std::process::id()));
        fs::create_dir(&path).unwrap();
        let stand = Self(path);
        for directory in ["electrical", "drives", "sensors", "boards"] {
            fs::create_dir_all(stand.0.join("catalog").join(directory)).unwrap();
        }
        stand
    }

    fn write(&self, path: &str, text: &str) {
        fs::write(self.0.join(path), text).unwrap();
    }

    fn save(&self, path: &str, value: &Toml) {
        self.write(path, &toml::to_string(value).unwrap());
    }

    fn parse(&self, path: &str) -> Toml {
        parse(&fs::read_to_string(self.0.join(path)).unwrap())
    }

    fn electrical(&self, extra: &str) -> Value {
        self.write("electrical.toml", &format!(
            "electrical_format = 1\ncatalog = 'catalog'\n[assemblies.audit]\nworld = 'world.toml'\n{extra}"
        ));
        self.run("electrical.toml")["assemblies"]["audit"].clone()
    }

    fn launch(&self, manifest: &str) -> Output {
        Command::new(env!("CARGO_BIN_EXE_sim-run"))
            .args(["--campaign", manifest, "--json", "--jobs", "1"])
            .current_dir(&self.0)
            .output()
            .unwrap()
    }

    fn run(&self, manifest: &str) -> Value {
        let output = self.launch(manifest);
        assert!(output.status.success(), "{output:?}");
        serde_json::from_slice(&output.stdout).unwrap_or_else(|error| {
            panic!(
                "{error}: {}, {}",
                output.status,
                String::from_utf8_lossy(&output.stderr)
            )
        })
    }

    fn t3(label: &str, code: &str, variant: &str) -> Self {
        let stand = Self::new(label);
        for name in ["world.toml", "device.toml", "scheme.toml"] {
            stand.write(name, &corpus(&format!("depth_t3/{code}/{variant}/{name}")));
        }
        for name in ["parts.toml", "ratings.toml"] {
            let path = format!("catalog/electrical/{name}");
            stand.write(&path, &corpus(&format!("depth_t3/{path}")));
        }
        stand
    }
}

impl Drop for Stand {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

fn corpus(path: &str) -> String {
    fs::read_to_string(
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../devices/electrical")
            .join(path),
    )
    .unwrap()
}

fn parse(text: &str) -> Toml {
    toml::from_str(text).unwrap()
}

fn remove(value: &mut Toml, key: &str) {
    assert!(
        value.as_table_mut().unwrap().remove(key).is_some(),
        "нет поля {key}"
    );
}

fn hit<'a>(report: &'a Value, code: &str, status: &str) -> &'a Value {
    let check = &report["assemblies"]["block"]["depth_checks"][code];
    assert_eq!(check["code"], code, "{report}");
    assert_eq!(check["status"], status, "{check}");
    assert_eq!(
        report["simulation"],
        if status == "fail" {
            "Rejected"
        } else {
            "NotProven"
        },
        "{report}"
    );
    &check["metrics"]
}

fn contains(bounds: &Value, expected: f64, tolerance: f64) {
    let lo = bounds[0].as_f64().unwrap();
    let hi = bounds[1].as_f64().unwrap();
    assert!(
        lo <= expected + tolerance && hi >= expected - tolerance,
        "{bounds} против {expected}"
    );
}

#[test]
fn fuse_overlapping_tcc() {
    let code = "fuse_coordination";
    let stand = Stand::t3("tcc", code, "fixed");
    let baseline = stand.electrical("");
    let baseline = hit(&baseline, code, "pass");
    assert_eq!(
        baseline["clearing_time_s"]["value"],
        serde_json::json!([0.015, 0.017])
    );
    // При 20 А провод с пределом 8–9,2 А²·с достигает его через 20–23 мс.
    let bounds = &baseline["wire_damage"]["value"]["damage_time_s"];
    assert!((bounds[0].as_f64().unwrap() - 8.0 / 20.0_f64.powi(2)).abs() < 1e-12);
    assert!((bounds[1].as_f64().unwrap() - 9.2 / 20.0_f64.powi(2)).abs() < 1e-12);
    let file = "catalog/electrical/ratings.toml";
    let mut ratings = stand.parse(file);
    let mut overlap = ratings["depth_ratings"]["fuse_slow"]["data"]["tcc"]["bands"][0].clone();
    let bands = |ratings: &mut Toml, band| {
        let bands = ratings["depth_ratings"]["fuse_fast"]["data"]["tcc"]["bands"]
            .as_array_mut()
            .unwrap();
        bands.truncate(1);
        bands.push(band);
    };
    bands(&mut ratings, overlap.clone());
    stand.save(file, &ratings);
    let known = stand.electrical("");
    let known = hit(&known, code, "not_analyzed");
    assert_eq!(
        known["clearing_time_s"]["value"],
        serde_json::json!([0.015, 0.03])
    );

    // Полоса 28–30 мс покрывает те же 20 А. Потеря её источника не уменьшает
    // неопределённость до 15–17 мс. Оценочный источник даёт тот же исход.
    for source in [None, Some("{ estimate = 'оценка времени выбега' }")] {
        remove(&mut overlap["time_s"], "source");
        if let Some(source) = source {
            overlap["time_s"].as_table_mut().unwrap().insert(
                "source".into(),
                parse(&format!("source = {source}"))["source"].clone(),
            );
        }
        bands(&mut ratings, overlap.clone());
        stand.save(file, &ratings);
        let unknown = stand.electrical("");
        let unknown = hit(&unknown, code, "not_analyzed");
        assert_eq!(unknown["clearing_time_s"]["status"], "not_analyzed");
        // Возвращаем подтверждённую запись для следующего отрицательного опыта.
        overlap = ratings["depth_ratings"]["fuse_slow"]["data"]["tcc"]["bands"][0].clone();
    }
    // Ближайшая целая отметка выше 20 А вне диапазона этого отказа.
    overlap["prospective_current_a"] = parse("x = [21.0, 21.0]")["x"].clone();
    remove(&mut overlap["time_s"], "source");
    bands(&mut ratings, overlap);
    stand.save(file, &ratings);
    hit(&stand.electrical(""), code, "pass");
}

#[test]
fn capacitor_rms_and_intersample_peak() {
    let code = "capacitor_stress";
    let stand = Stand::t3("capacitor", code, "fixed");
    hit(&stand.electrical(""), code, "pass");
    let mut scheme = stand.parse("scheme.toml");
    let check = &mut scheme["depth_checks"][0]["check"];
    // Линейный отрезок от −2 до +2 А имеет средний ток 0 и RMS = 2/√3.
    check["series"]["current_a"] = parse("x = [-2.0, 2.0]")["x"].clone();
    stand.save("scheme.toml", &scheme);
    let ripple = stand.electrical("");
    let ripple = hit(&ripple, code, "fail");
    contains(
        &ripple["ripple_a"]["value"]["observed"],
        2.0 / 3.0_f64.sqrt(),
        1e-12,
    );

    let mut scheme = stand.parse("scheme.toml");
    let check = &mut scheme["depth_checks"][0]["check"];
    check["series"]["current_a"] = parse("x = [0.0, 0.0]")["x"].clone();
    // Зафиксировано 8 В, допустимо 9 В. Огибающая допускает пик до 10 В.
    check["voltage_error_v"]["max"] = Toml::Float(2.0);
    stand.save("scheme.toml", &scheme);
    let peak = stand.electrical("");
    let peak = hit(&peak, code, "not_analyzed");
    contains(&peak["voltage_v"]["value"]["observed"], 10.0, 1e-12);
    assert!(peak["voltage_v"]["value"]["margin"][0].as_f64().unwrap() < 0.0);
    assert!(peak["voltage_v"]["value"]["margin"][1].as_f64().unwrap() > 0.0);

    // Между узлами сетки 0 и 500 мкс задаём вершину 10 В на 125 мкс.
    // Напряжение параллельного C равно известной ЭДС на этой отметке.
    scheme["depth_checks"][0]["check"]["voltage_error_v"]["max"] = Toml::Float(0.0);
    scheme["modes"]["cases"][0].as_table_mut().unwrap().insert(
        "sources".into(),
        parse(r#"sources = [{ branch = { kind = "element", component = "cap", element = "emf" }, points = [[0, 8.0], [125000, 10.0], [250000, 8.0]], source = { derived = "Вершина треугольного входа на четверти шага 500 мкс" } }]"#)["sources"].clone(),
    );
    stand.save("scheme.toml", &scheme);
    let report = stand.electrical("");
    hit(&report, code, "not_analyzed");
    assert!(
        report["assemblies"]["block"]["depth_checks"][code]["applicability_gap"]
            .as_str()
            .unwrap()
            .contains("не согласуется"),
        "{report}"
    );
}

#[test]
fn switch_soa_and_instantaneous_power() {
    let code = "switch_thermal";
    let stand = Stand::t3("switch", code, "fixed");
    let baseline = stand.electrical("");
    let metrics = hit(&baseline, code, "pass");
    // Два RC-звена при 4 В · 2 А в течение 0,5 с. Средняя мощность за 10 с — 0,4 Вт.
    let rise = |power| power * ((1.0 - (-0.5_f64).exp()) + 2.0 * (1.0 - (-0.25_f64).exp()));
    contains(
        &metrics["foster_peak_junction_k"]["value"]["observed"],
        300.0 + rise(8.0),
        1e-10,
    );
    let path = "catalog/electrical/ratings.toml";
    let mut ratings = stand.parse(path);
    // Импульс длится не менее 0,5 с даже при незакрытой левой границе.
    ratings["depth_ratings"]["switch_bench"]["data"]["soa"]["limits"] = parse(
        "x = [{ kind = 'pulse', duration_s = { min = 0.25, nominal = 0.25, max = 0.25, source = { derived = 'Порог синтетического импульса: половина наблюдаемой длительности' } } }]"
    )["x"].clone();
    stand.save(path, &ratings);
    let report = stand.electrical("");
    let metrics = hit(&report, code, "fail");
    assert_eq!(metrics["linear_soa"]["value"][0]["status"], "fail");
}

#[test]
fn weakest_cell_and_missing_leakage() {
    let discharge = "deep_discharge";
    let stand = Stand::t3("weak-cell", discharge, "unsafe");
    let report = stand.electrical("");
    let metrics = hit(&report, discharge, "fail");
    // Перед отсечкой t = 2 с. Формула слабой ячейки из постановки Т3.
    let v = 2.5 + 2.0 * (0.5 - 1.02 * 2.0 / 10.0) - 0.1 - 0.2 * (1.0 - (-2.0_f64).exp());
    assert!(v < 3.0);
    let weak = &metrics["cells"][0];
    assert_eq!(metrics["limiting_cell"], weak["id"]);
    contains(&weak["violation"]["voltage_v"], v, 1e-10);
    assert!(
        metrics["cells"][1]["margin_v"]["value"][0]
            .as_f64()
            .unwrap()
            > 0.0
    );

    let sleep = "sleep_drain";
    let stand = Stand::t3("leakage", sleep, "fixed");
    let mut scheme = stand.parse("scheme.toml");
    let baseline = stand.electrical("");
    let metrics = hit(&baseline, sleep, "pass");
    // Десять периодов с суммой 15 Кл на контроллер, 25 Кл на ветвь, 2 Кл саморазряд.
    contains(
        &metrics["consumption"]["value"]["total_ah"],
        (15.0 + 25.0 + 2.0) / 3600.0,
        1e-12,
    );
    let load = &mut scheme["depth_checks"][0]["check"]["profile"]["states"][0]["loads"][0];
    remove(load, "battery_current_a");
    stand.save("scheme.toml", &scheme);
    let report = stand.electrical("");
    let metrics = hit(&report, sleep, "not_analyzed");
    assert_eq!(metrics["consumption"]["status"], "not_analyzed");
}

#[test]
fn cable_one_stroke_does_not_prove_cycle_duty() {
    let code = "cable_flex";
    let stand = Stand::t3("flex", code, "fixed");
    let baseline = stand.electrical("");
    hit(&baseline, code, "pass");
    let mut ratings = stand.parse("catalog/electrical/ratings.toml");
    // Сохраняем подтверждённые геометрию и фиксацию. Статические пределы
    // и свидетельство монтажа не заменяют кривую повторных изгибов.
    remove(
        &mut ratings["depth_ratings"]["cable_bench"]["data"],
        "curve",
    );
    stand.save("catalog/electrical/ratings.toml", &ratings);
    let report = stand.electrical("");
    let metrics = hit(&report, code, "not_analyzed");
    assert_eq!(metrics["bend"]["status"], "pass");
    assert_eq!(metrics["relief"], serde_json::json!(["pass", "pass"]));
    assert_eq!(metrics["fatigue"]["status"], "not_analyzed");
}

#[test]
fn solder_window_and_unresolved_series() {
    let stand = Stand::t3("solder", "capacitor_stress", "fixed");
    stand.write(
        "scheme.toml",
        &corpus("depth_t5/scheme.toml").replace("scope = \"reference\"", "scope = \"assembly\""),
    );
    stand.write(
        "catalog/electrical/parts.toml",
        &corpus("depth_t5/parts.toml").replace("reference_only = true\n", ""),
    );
    fs::remove_file(stand.0.join("catalog/electrical/ratings.toml")).unwrap();
    stand.write("solder.toml", &corpus("depth_t5/solder.toml"));
    stand.write("profile.toml", &corpus("depth_t5/bridge/fixed.toml"));
    // Сетка 1024 повторяет приёмку четырёхузлового стенда Т5.
    let setup = "format = 1\ncatalog = 'solder.toml'\nsteps = 1024\n[instances.block.'стык-т5-эталон']\nprofile = 'profile.toml'\n";
    stand.write("setup.toml", setup);
    let extra = "solder = 'setup.toml'\n";
    let report = stand.electrical(extra);
    let joint = &report["assemblies"]["block"]["solder_checks"]["стык-т5-эталон"];
    assert_eq!(joint["window_status"], "pass", "{joint}");
    assert_eq!(joint["series"]["applicability"]["status"], "not_analyzed");
    // Из expected.md для стенда Т5, Tп = 528 − 70·exp(−t/1 с).
    contains(
        &joint["window"]["value"]["thermal"]["value"]["peak_pad_k"],
        528.0 - 70.0 * (-1.45_f64).exp(),
        1e-8,
    );
    let finding = report["findings"]
        .as_array()
        .unwrap()
        .iter()
        .find(|f| f["code"] == "solder_defect")
        .unwrap();
    assert_eq!(finding["outcome"], "not_analyzed");
    assert_eq!(report["simulation"], "NotProven");

    let file = "profile.toml";
    let mut profile = stand.parse(file);
    profile["limits"]["max_pad_k"]["source"] =
        parse("source.estimate = 'Оценочная температура площадки'")["source"].clone();
    stand.save(file, &profile);
    let estimated = stand.electrical(extra);
    assert_eq!(
        estimated["assemblies"]["block"]["solder_checks"]["стык-т5-эталон"]["window_status"],
        "not_analyzed"
    );
    stand.write(file, &corpus("depth_t5/bridge/fixed.toml"));

    // Невалидный вход серии обязан отклоняться и без ссылки на калибровку.
    stand.write("setup.toml", &format!("{setup}series = 'series.toml'\n"));
    for number in ["nan", "inf"] {
        stand.write(
            "series.toml",
            &format!(
                r#"
format = 1
id = "серия"
revision = "bench-t5-r1"
schematic = "Блок"
scope = "assembly"
connection = "стык-т5-эталон"
ends = ["площадка/p", "вывод/p"]
[applicability]
force_n = [1.0, {number}]
"#
            ),
        );
        stand.write("electrical.toml",
            "electrical_format = 1\ncatalog = 'catalog'\n[assemblies.audit]\nworld = 'world.toml'\nsolder = 'setup.toml'\n");
        let result = stand.launch("electrical.toml");
        let diagnostic = format!(
            "{}{}",
            String::from_utf8_lossy(&result.stdout),
            String::from_utf8_lossy(&result.stderr)
        );
        assert!(
            !result.status.success() && diagnostic.contains("конечные"),
            "{diagnostic}"
        );
    }
}

#[test]
fn depth_window_must_fit_declared_mode() {
    let code = "capacitor_stress";
    let stand = Stand::t3("mode-window", code, "fixed");
    hit(&stand.electrical(""), code, "pass");
    let mut scheme = stand.parse("scheme.toml");
    // Электрический режим заканчивается после одного шага 500 мкс.
    // Окно свидетельства остаётся прежним и выходит за этот режим.
    scheme["modes"]["cases"][0]["window"]["end_ns"] = Toml::Integer(500_000);
    stand.save("scheme.toml", &scheme);
    hit(&stand.electrical(""), code, "not_analyzed");
}

#[test]
fn fuse_fault_must_match_the_assessed_path() {
    let code = "fuse_coordination";
    let stand = Stand::t3("fault-branch", code, "fixed");
    hit(&stand.electrical(""), code, "pass");
    let mut scheme = stand.parse("scheme.toml");
    // Метка на постоянном источнике затвора. В опыте на нём нет события на 1 с.
    scheme["depth_checks"][0]["event"]["branch"]["element"] = Toml::String("gate".into());
    stand.save("scheme.toml", &scheme);
    hit(&stand.electrical(""), code, "not_analyzed");
}

#[test]
fn reference_rating_cannot_prove_an_assembly() {
    let code = "capacitor_stress";
    let stand = Stand::t3("reference-rating", code, "fixed");
    for path in ["scheme.toml", "catalog/electrical/parts.toml"] {
        // Маркируем числа синтетической фикстуры как измеренные только для
        // изоляции барьера области паспорта, сохраняем его scope = reference.
        stand.write(
            path,
            &fs::read_to_string(stand.0.join(path))
                .unwrap()
                .replace("reference_only = true\n", "")
                .replace("scope = \"reference\"", "scope = \"assembly\"")
                .replace("derived =", "measured ="),
        );
    }
    let report = stand.electrical("");
    let row = &report["assemblies"]["block"]["depth_checks"][code];
    assert_eq!(row["status"], "not_analyzed", "{row}");
    assert!(
        row["applicability_gap"]
            .as_str()
            .unwrap()
            .contains("эталонного стенда")
    );
    assert_eq!(row["metrics"], Value::Null);
    assert_eq!(report["simulation"], "NotProven");
}

#[test]
fn contradictory_capacitor_voltage_blocks_assembly_verdict() {
    let code = "capacitor_stress";
    let stand = Stand::t3("voltage-conflict", code, "unsafe");
    for path in [
        "scheme.toml",
        "catalog/electrical/parts.toml",
        "catalog/electrical/ratings.toml",
    ] {
        // Типы источников заданы фикстурой, чтобы проверить путь допуска сборки.
        stand.write(
            path,
            &fs::read_to_string(stand.0.join(path))
                .unwrap()
                .replace("reference_only = true\n", "")
                .replace("scope = \"reference\"", "scope = \"assembly\"")
                .replace("derived =", "measured ="),
        );
    }
    let report = stand.electrical("");
    hit(&report, code, "fail");
    let mut scheme = stand.parse("scheme.toml");
    // В цепи C параллелен идеальному источнику 10 В. Подмена ряда на 8 В
    // оставляла положительный запас до 12 В · 0,75 = 9 В и PassedSimulation.
    scheme["depth_checks"][0]["check"]["series"]["voltage_v"] =
        parse("x = [8.0, 8.0]")["x"].clone();
    for declared_probe in [true, false] {
        if !declared_probe {
            scheme["modes"]["cases"][0]["probes"]
                .as_array_mut()
                .unwrap()
                .retain(|probe| probe["kind"].as_str() != Some("voltage"));
        }
        stand.save("scheme.toml", &scheme);
        let report = stand.electrical("");
        hit(&report, code, "not_analyzed");
        let reason = &report["assemblies"]["block"]["depth_checks"][code]["applicability_gap"];
        assert!(reason.as_str().unwrap().contains("не согласуется"));
    }
    // Увеличиваем верхнюю границу с 12 до 14 В. После derating режимные
    // 10 В попадают внутрь диапазона предела [9; 10,5] В.
    let path = "catalog/electrical/ratings.toml";
    let mut ratings = stand.parse(path);
    ratings["depth_ratings"]["capacitor_bench_10"]["data"]["rated_voltage_v"]["max"] =
        Toml::Float(14.0);
    stand.save(path, &ratings);
    hit(&stand.electrical(""), code, "not_analyzed");
}

#[test]
fn common_campaign_counts_declared_depth_failure() {
    let code = "capacitor_stress";
    let stand = Stand::t3("common-campaign", code, "unsafe");
    let electrical = stand.electrical("");
    hit(&electrical, code, "fail");
    stand.write("plan.toml", r#"
version = 1
world = "world.toml"
catalog = "catalog"
seed = 1 # Зерно исходной постановки блока.

[stations.block]
experiment = { kind = "scenario" }
[stations.block.modes.rest]
# Один такт неподвижного носителя на частоте 1 кГц.
horizon_ns = 1000000
source = { kind = "measured", reference = "Неподвижный блок без внешних сил" }
criteria = [{ id = "покой", window = "whole", limit = { kind = "range", signal = { name = "qpos[0]", unit = "m" }, min = 0.0, max = 0.0, source = { kind = "measured", reference = "Ось неподвижна" } } }]
[stations.block.modes.rest.windows.whole]
start_ns = 0
end_ns = 1000000
max_gap_ns = 1000000
"#);
    stand.write(
        "installation.toml",
        r#"
version = 1
[[paths]]
station = "block"
mode = "rest"
path = "axis"
source = { kind = "measured", reference = "Ось неподвижна" }
grids = [{ id = "base", hz = 1000, source = { kind = "measured", reference = "Сетка носителя" } }]
"#,
    );
    stand.write(
        "campaign.toml",
        r#"
version = 1
[assemblies.block]
scope = "Неподвижный блок"
[assemblies.block.experiments.rest]
kind = "scenario"
campaign = "plan.toml"
installation = "installation.toml"
"#,
    );
    let report = stand.run("campaign.toml");
    assert_eq!(
        report["assemblies"]["block"]["verdict"], "Rejected",
        "{report}"
    );
    let runs = report["assemblies"]["block"]["runs"].as_object().unwrap();
    let finding = runs
        .values()
        .flat_map(|row| row["findings"].as_array().unwrap())
        .find(|finding| finding["code"] == code)
        .expect("обязательство Т3 вошло в общую кампанию");
    assert_eq!(finding["outcome"], "fail", "{finding}");
}
