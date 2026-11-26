//! Приёмка данных Т7. Для последовательной проверки запускать с --test-threads=1.

use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    path::{Path, PathBuf},
    process::{Command, Output},
    time::{Duration, Instant},
};

use serde_json::{Value, json};
use sim_cli::view::{RunView, campaign_bindings};
use sim_runtime::{Log, LogKind};
use sim_task::{
    campaign::{
        RunKey,
        depth_factors::{DepthFactors, Selection},
    },
    sampling::SampleKey,
};

#[path = "../../sim-task/tests/fixtures/depth_pair_oracle.rs"]
mod depth_pair_oracle;
use depth_pair_oracle::required_interactions;

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

struct Scratch(PathBuf);

impl Scratch {
    fn new(case: &str) -> Self {
        let path =
            std::env::temp_dir().join(format!("depth-campaign-{}-{case}", std::process::id()));
        fs::create_dir(&path).unwrap();
        let scratch = Self(path);
        // Идентичность исполняемого файла входит в контрольную точку.
        // Соседняя сборка Cargo может заменить общий бинарник между запусками.
        fs::copy(env!("CARGO_BIN_EXE_sim-run"), scratch.0.join("sim-run")).unwrap();
        scratch
    }

    fn write(&self, name: &str, text: &str) {
        fs::write(self.0.join(name), text).unwrap();
    }

    fn cli(&self, manifest: &Path, options: &[&str], code: i32) -> Output {
        let mut command = Command::new(self.0.join("sim-run"));
        command.arg("--campaign").arg(manifest).arg("--json");
        if !options.contains(&"--jobs") {
            command.args(["--jobs", "1"]);
        }
        let output = command.args(options).output().unwrap();
        assert_eq!(
            output.status.code(),
            Some(code),
            "{}\n{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
        output
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

fn report(output: &Output) -> Value {
    serde_json::from_slice(&output.stdout).unwrap()
}

fn counts(required: usize, planned: usize, executed: usize) -> Value {
    json!({"required": required, "planned": planned, "executed": executed})
}

fn reserved(checkpoint: &Path) -> BTreeMap<String, u64> {
    fs::read_dir(checkpoint)
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .map(|dir| dir.join("depth-budget-v1.json"))
        .filter(|file| file.is_file())
        .map(|file| {
            let envelope: Value = serde_json::from_slice(&fs::read(&file).unwrap()).unwrap();
            let payload = envelope["payload"].as_str().unwrap();
            assert_eq!(envelope["checksum"], sim_signal::fnv1a(payload.bytes()));
            let state: Value = serde_json::from_str(payload).unwrap();
            let name = file
                .parent()
                .unwrap()
                .file_name()
                .unwrap()
                .to_str()
                .unwrap();
            (name.to_owned(), state["reserved"].as_u64().unwrap())
        })
        .collect()
}

// Стенд: сила 6 Н, массы 6 и 12 кг. Полунеявный Euler даёт
// x = F/m · h² · n(n+1)/2. На сетке 1 кГц получаем 3 и 1,5 мкм.
// При пределе 2,75 мкм ожидаем запасы −0,25 и +1,25 мкм.
// Допустимая уточнённая сетка для лёгкой каретки даёт 2,5 мкм и +0,25 мкм.
const LIMIT: f64 = 2.75e-6;
const TRAVEL: [(&str, &str, f64, f64, &str); 3] = [
    ("base", "light", 6.0, -0.25e-6, "fail"),
    ("base", "heavy", 12.0, 1.25e-6, "pass"),
    ("fine", "light", 6.0, 0.25e-6, "pass"),
];

const CAMPAIGN: &str = r#"
version = 1
world = "world.toml"
catalog = "catalog.toml"
# Зерно нейтрального стержня из tests/fixtures/rotary/world.toml.
seed = 17
[stations.block]
experiment = { kind = "scenario" }
[stations.block.modes.motion]
horizon_ns = 2000000
source = { kind = "derived", reference = "Два такта основной сетки стенда" }
steps = [{ id = "force", target = { kind = "effort", name = "блок/сила" }, at_ns = 0, value = 6.0, unit = "N", source = { kind = "derived", reference = "Испытательная сила 6 Н: ускорения 1 и 0,5 м/с² для двух образцов" } }]
criteria = [{ id = "travel", window = "whole", limit = { kind = "range", signal = { name = "qpos[0]", unit = "m" }, max = 0.00000275, source = { kind = "datasheet", reference = "Порог оснастки 2,75 мкм между ручными значениями лёгкого образца на двух сетках" } } }]
[stations.block.modes.motion.windows.whole]
start_ns = 0
end_ns = 2000000
max_gap_ns = 1000000
[depth_factors.factors.grid]
safety = true
source = { kind = "derived", reference = "Шаги оснастки 1 и 0,5 мс" }
levels = { kind = "grid", values = { base = "base", fine = "fine" } }
[depth_factors.factors.mass]
safety = false
source = { kind = "datasheet", reference = "Единичные кубы плотности 6 и 12 кг/м³" }
levels = { kind = "axis", parameter = { kind = "mass", body = "блок/каретка" }, unit = "kg", nominal = 6.0, values = { light = 6.0, heavy = 12.0 } }
[depth_factors.factors.mode]
safety = true
source = { kind = "derived", reference = "Постоянное усилие на оснастке" }
levels = { kind = "mode", values = { motion = { station = "block", mode = "motion" } } }
[depth_factors.factors.path]
safety = true
source = { kind = "derived", reference = "Одна продольная направляющая оснастки" }
levels = { kind = "path", values = { line = "line" } }
[depth_factors.constraints.heavy_grid]
when = { mass = "heavy", grid = "fine" }
reason = "У тяжёлого образца оснастка выдаёт отсчёты только с шагом 1 мс"
source = { kind = "datasheet", reference = "Ограничение двух образцов синтетического стенда" }
"#;

const INSTALL: &str = r#"
version = 1
[coverage]
# Три разрешённых опыта и по одному повтору для сверки каждого журнала.
runs = 6
# Дедлайн координатора из блока Е ТЗ.
seconds = 1800
[[paths]]
station = "block"
mode = "motion"
path = "line"
source = { kind = "derived", reference = "Путь синтетической каретки" }
grids = [
    { id = "base", hz = 1000, source = { kind = "derived", reference = "Шаг 1 мс" } },
    { id = "fine", hz = 2000, source = { kind = "derived", reference = "Шаг 0,5 мс" } },
]
"#;

fn stand(case: &str) -> Scratch {
    let s = Scratch::new(case);
    s.write(
        "manifest.toml",
        r#"
version = 1
[assemblies.block]
scope = "Стенд каретки"
[assemblies.block.experiments.cover]
kind = "scenario"
campaign = "campaign.toml"
installation = "install.toml"
"#,
    );
    s.write("campaign.toml", CAMPAIGN);
    s.write("install.toml", INSTALL);
    s.write("catalog.toml", "version = 1");
    s.write(
        "world.toml",
        r#"
format = 3
name = "Блок"
seed = 17
physics_hz = 1000
# Изолированный горизонтальный стенд. Единственное усилие задано в кампании.
gravity_m_s2 = [0.0, 0.0, 0.0]
[environment]
density_kg_m3 = 0.0
viscosity_pa_s = 0.0
wind_m_s = [0.0, 0.0, 0.0]
[[devices]]
name = "блок"
file = "block.toml"
"#,
    );
    s.write("block.toml", r#"
format = 3
name = "Блок"
[[links]]
name = "каретка"
joint = { kind = "slide", axis = [1.0, 0.0, 0.0], efforts = [{ name = "сила", dof = 0 }] }
# Единичный куб. I = ma²/6, центр масс в нуле.
mass = { kind = "explicit", mass_kg = 6.0, com_m = [0.0, 0.0, 0.0], inertia_kg_m2 = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0], provenance = { datasheet = "Куб оснастки" } }
"#);
    s
}

fn close(actual: f64, expected: f64) {
    // 1 пм меньше на пять порядков минимального запаса 0,25 мкм.
    assert!(
        (actual - expected).abs() <= 1e-12,
        "{actual} против {expected}"
    );
}

fn peak(log: &Log) -> f64 {
    log.entries()
        .iter()
        .filter(|e| e.kind == LogKind::Truth && log.header().truth[e.id as usize].name == "qpos[0]")
        .map(|e| e.value)
        .reduce(f64::max)
        .expect("нет измеренного хода")
}

#[test]
fn адресная_строка_исполняется_и_повторяется_по_квитанции() {
    let s = stand("refinement");
    let manifest = s.0.join("manifest.toml");
    let checkpoint = s.0.join("checkpoint");
    let cp = checkpoint.to_str().unwrap();
    let campaign = CAMPAIGN.split("[depth_factors.constraints").next().unwrap()
        .replace("base = \"base\", fine = \"fine\"", "base = \"base\"");
    // На каретку действуют 6 Н и продольная добавка до 6 Н / 12 кг.
    // Поперечная добавка той же величины гасится связью. При m=6 кг
    // и a=1,5 м/с² ручное решение для двух тактов даёт 4,5 мкм.
    s.write("campaign.toml", &format!(r#"{campaign}
[depth_factors.factors.gravity_x]
safety = false
source = {{ kind = "derived", reference = "Добавка 6 Н / 12 кг вдоль направляющей стенда" }}
levels = {{ kind = "axis", parameter = {{ kind = "gravity", axis = 0 }}, unit = "m/s^2", nominal = 0.0, values = {{ off = 0.0, on = 0.5 }} }}
[depth_factors.factors.gravity_y]
safety = false
source = {{ kind = "derived", reference = "Та же пробная добавка поперёк направляющей стенда" }}
levels = {{ kind = "axis", parameter = {{ kind = "gravity", axis = 1 }}, unit = "m/s^2", nominal = 0.0, values = {{ off = 0.0, on = 0.5 }} }}
"#));
    // Общий потолок блока Е, включая автоматический и адресный повторы.
    s.write("install.toml", &INSTALL.replace("runs = 6", "runs = 1000\nrepeat_worst = true"));

    let first = s.cli(&manifest, &["--checkpoint", cp], 0);
    let mut first_report = report(&first);
    let a = &first_report["assemblies"]["block"];
    let depth = &a["depth_coverage"];
    let rows = a["runs"].as_object().unwrap();
    assert_eq!(depth["acceptance"]["repeat"]["status"], "verified");
    assert_eq!(depth["refinement"]["step"]["status"], "no_untried_neighbors");
    assert_eq!(depth["worst"]["ranking"], "normalized_margin");
    close(depth["worst"]["margin"]["normalized"].as_f64().unwrap(), -7.0 / 11.0);
    let (key, refined) = rows.iter()
        .find(|(_, row)| row["detail"]["coverage"]["phase"] == "refinement")
        .expect("план покрытия должен оставить хотя бы одного соседа для адресного поиска");
    assert_eq!(refined["executed"], 1);
    let view = s.0.join("refined.simview");
    let replay = report(&s.cli(&manifest, &["--checkpoint", cp, "--repeat", key,
        "--view", view.to_str().unwrap()], 0));
    assert_eq!(replay["depth_repeat"]["status"], "verified");
    let view = RunView::read(&fs::read(view).unwrap()).unwrap();
    assert_eq!(view.journal.fingerprint(), refined["playback"]["journal"].as_u64().unwrap());
    let values = &refined["detail"]["scenario"]["summary"]["installation"]["values"];
    let expected = (6.0 / values["mass"].as_f64().unwrap() + values["gravity_x"].as_f64().unwrap()) * 3e-6;
    close(peak(&view.journal), expected);

    let mut second = report(&s.cli(&manifest, &["--jobs", "2"], 0));
    // Внешний замер отделён от детерминированных результатов физики.
    for report in [&mut first_report, &mut second] {
        report["assemblies"]["block"]["depth_coverage"]["acceptance"]["elapsed_ns"] = Value::Null;
    }
    assert_eq!(first_report, second);
}

#[test]
fn при_нехватке_факторов_отчёт_указывает_неприменимость_троек() {
    let s = stand("no-triples");
    let manifest = s.0.join("manifest.toml");
    let mut campaign: toml::Value = toml::from_str(CAMPAIGN).unwrap();
    for (count, marked) in [(0, &[][..]), (1, &["mode"][..]), (2, &["mode", "path"][..])] {
        for (id, factor) in campaign["depth_factors"]["factors"].as_table_mut().unwrap() {
            factor["safety"] = toml::Value::Boolean(marked.contains(&id.as_str()));
        }
        s.write("campaign.toml", &toml::to_string(&campaign).unwrap());
        let reason = format!("Тройки не требуются: факторов безопасности {count}");
        let json = report(&s.cli(&manifest, &[], 0));
        let depth = &json["assemblies"]["block"]["depth_coverage"];
        // Состав допустимых пар из теста оснастки от пометки не зависит.
        assert_eq!(depth["pairs"], counts(12, 12, 12));
        assert_eq!(depth["safety_triples"], counts(0, 0, 0));
        assert_eq!(depth["safety_triples_not_required"], reason);
        let output = Command::new(s.0.join("sim-run"))
            .arg("--campaign")
            .arg(&manifest)
            .args(["--jobs", "1"])
            .output()
            .unwrap();
        assert!(output.status.success());
        let output = String::from_utf8(output.stdout).unwrap();
        assert!(output.contains(&reason), "{output}");
        assert!(!output.contains("Тройки безопасности · требуется"));
    }
}

#[test]
fn две_установки_дают_расчётные_запасы_и_тот_же_худший_опыт() {
    let s = stand("samples");
    let manifest = s.0.join("manifest.toml");
    let checkpoint = s.0.join("checkpoint");
    let cp = checkpoint.to_str().unwrap();
    let first = s.cli(&manifest, &["--checkpoint", cp], 0);
    let second = s.cli(&manifest, &["--jobs", "2"], 0);
    assert_eq!(first.stdout, second.stdout, "повторный независимый отчёт");
    let json = report(&first);
    let a = &json["assemblies"]["block"];
    let depth = &a["depth_coverage"];
    // grid×mass: три допустимые пары. Четыре пары переменного фактора
    // с постоянным имеют по два уровня. mode×path: одна пара. Тройки:
    // mode×path×grid, по одной на сетку (mass не помечена безопасностью).
    assert_eq!(depth["pairs"], counts(12, 12, 12));
    assert_eq!(depth["safety_triples"], counts(2, 2, 2));
    assert_eq!(a["verdict"], "Rejected");
    let rows = a["runs"].as_object().unwrap();
    assert_eq!(rows.len(), TRAVEL.len());
    let mut seeds = BTreeMap::new();
    let mut samples = BTreeSet::new();
    let worst = json["worst_run_key"].as_str().unwrap();
    assert_eq!(depth["worst"]["run_key"], worst);
    let bindings = campaign_bindings(std::str::from_utf8(&first.stdout).unwrap()).unwrap();
    for (grid, mass, kg, reserve, outcome) in TRAVEL {
        let (key, row) = rows
            .iter()
            .find(|(_, row)| {
                let sel = &row["detail"]["coverage"]["selection"];
                sel["grid"] == grid && sel["mass"] == mass
            })
            .unwrap();
        let summary = &row["detail"]["scenario"]["summary"];
        assert_eq!(summary["outcome"], outcome, "{key}: {summary}");
        assert_eq!(summary["criteria_complete"], true);
        assert_eq!(summary["installation"]["values"]["mass"], kg);
        assert_eq!(
            row["outcome"],
            if reserve < 0.0 {
                "fail"
            } else {
                "not_analyzed"
            }
        );
        assert_eq!(key == worst, reserve < 0.0);
        let seed = row["playback"]["seed"].as_u64().unwrap();
        assert_eq!(*seeds.entry(mass).or_insert(seed), seed);
        samples.insert(summary["key"]["sample"]["id"].as_str().unwrap());

        let archive = s.0.join(format!("{grid}-{mass}.simview"));
        let repeat = report(&s.cli(
            &manifest,
            &[
                "--checkpoint",
                cp,
                "--repeat",
                key,
                "--view",
                archive.to_str().unwrap(),
            ],
            0,
        ));
        assert_eq!(repeat["depth_repeat"]["status"], "verified");
        let view = RunView::read(&fs::read(archive).unwrap()).unwrap();
        assert_eq!(view.metadata.campaign.as_ref().unwrap(), &bindings[key]);
        assert_eq!(
            view.journal.fingerprint(),
            row["playback"]["journal"].as_u64().unwrap()
        );
        assert_eq!(view.journal.header().seed, seed);
        close(LIMIT - peak(&view.journal), reserve);
    }
    assert_eq!(samples.len(), 2);
    assert_ne!(seeds["light"], seeds["heavy"]);
    assert_eq!(reserved(&checkpoint).into_values().collect::<Vec<_>>(), [6]);
    let archive = s.0.join("exhausted.simview");
    let repeat = report(&s.cli(
        &manifest,
        &[
            "--checkpoint",
            cp,
            "--repeat",
            worst,
            "--view",
            archive.to_str().unwrap(),
        ],
        0,
    ));
    assert_eq!(repeat["depth_repeat"]["status"], "not_analyzed");
    assert!(
        repeat["view_unavailable"]
            .as_str()
            .unwrap()
            .contains("квот")
    );
    assert!(!archive.exists());
    assert_eq!(reserved(&checkpoint).into_values().collect::<Vec<_>>(), [6]);
}

#[test]
fn дедлайн_отсутствующая_инъекция_и_непроверенный_предел_держат_строки_открытыми() {
    let s = stand("negative");
    let manifest = s.0.join("manifest.toml");
    let cp = s.0.join("deadline");
    s.write(
        "install.toml",
        &INSTALL.replace("seconds = 1800", "seconds = 0"),
    );
    let stopped = report(&s.cli(&manifest, &["--checkpoint", cp.to_str().unwrap()], 0));
    let assembly = &stopped["assemblies"]["block"];
    let depth = &assembly["depth_coverage"];
    assert_eq!(assembly["verdict"], "NotProven");
    assert_eq!(depth["pairs"], counts(12, 12, 0));
    assert_eq!(depth["safety_triples"], counts(2, 2, 0));
    assert_eq!(depth["open_rows"].as_array().unwrap().len(), TRAVEL.len());
    for row in depth["open_rows"].as_array().unwrap() {
        assert!(row["reason"].as_str().unwrap().contains("дедлайн"));
    }
    assert_eq!(reserved(&cp).into_values().collect::<Vec<_>>(), [0]);

    let cp = s.0.join("quota");
    s.write("install.toml", &INSTALL.replace("runs = 6", "runs = 1"));
    let stopped = report(&s.cli(&manifest, &["--checkpoint", cp.to_str().unwrap()], 0));
    let assembly = &stopped["assemblies"]["block"];
    let depth = &assembly["depth_coverage"];
    // Единственная строка проецируется на C(4, 2) пар и одну тройку.
    assert_eq!(depth["pairs"], counts(12, 12, 6));
    assert_eq!(depth["safety_triples"], counts(2, 2, 1));
    assert_eq!(depth["open_rows"].as_array().unwrap().len(), 2);
    assert_ne!(assembly["verdict"], "PassedSimulation");
    for row in depth["open_rows"].as_array().unwrap() {
        assert!(row["reason"].as_str().unwrap().contains("квот"));
    }
    assert_eq!(reserved(&cp).into_values().collect::<Vec<_>>(), [1]);

    s.write("install.toml", INSTALL);
    s.write("campaign.toml", &format!(r#"{}
[depth_factors.factors.interruption]
safety = false
source = {{ kind = "datasheet", reference = "Разрыв контрольной ветви стенда" }}
levels = {{ kind = "faults", values = {{ nominal = [], open = [{{ node = "wire:line", kind = "wire_open" }}] }} }}
"#, CAMPAIGN.replace("kind = \"datasheet\", reference = \"Порог", "kind = \"estimate\", reference = \"Порог")));
    let checked = report(&s.cli(&manifest, &[], 0));
    let assembly = &checked["assemblies"]["block"];
    let depth = &assembly["depth_coverage"];
    // К 12 парам добавляются по четыре с массой и сеткой и по две с режимом и путём.
    assert_eq!(depth["pairs"], counts(24, 24, 18));
    // Две тройки без отказа. С отказом ещё 4 + 4 + 2, из них при
    // исполнении доступен лишь номинальный уровень (2 + 2 + 1).
    assert_eq!(depth["safety_triples"], counts(12, 12, 7));
    assert!(depth.get("safety_triples_not_required").is_none());
    for row in depth["open_rows"].as_array().unwrap() {
        assert_eq!(row["key"]["faults"][0]["node"], "wire:line");
        assert!(
            row["reason"]
                .as_str()
                .unwrap()
                .contains("отказ отсутствует")
        );
    }
    let rows = assembly["runs"].as_object().unwrap();
    let mut checked_safe = 0;
    for row in rows.values().filter(|row| row["executed"] == 1) {
        let selection = &row["detail"]["coverage"]["selection"];
        if selection["mass"] == "heavy" || selection["grid"] == "fine" {
            assert_eq!(
                row["detail"]["scenario"]["summary"]["outcome"],
                "not_analyzed"
            );
            checked_safe += 1;
        }
    }
    assert_eq!(checked_safe, 2);

    for non_finite in ["nan", "inf", "-inf"] {
        s.write(
            "campaign.toml",
            &CAMPAIGN.replace("heavy = 12.0", &format!("heavy = {non_finite}")),
        );
        assert_eq!(report(&s.cli(&manifest, &[], 2))["status"], "error");
    }
}

fn required_keys(
    id: &str,
    historical: &[toml::Value],
    placements: &toml::Value,
) -> BTreeSet<RunKey> {
    let make = |station: &str, mode: &str, path: &str, faults| RunKey {
        station: station.into(),
        mode: mode.into(),
        path: path.into(),
        faults,
        sample: SampleKey::Nominal,
        grid: "native".into(),
    };
    let native = make("electrical", "full", "electric", Vec::new());
    let mut keys = BTreeSet::from([
        native.clone(),
        RunKey {
            grid: "refined".into(),
            ..native
        },
    ]);
    for row in historical
        .iter()
        .filter(|row| row.get("replacement").and_then(toml::Value::as_str) == Some(id))
    {
        let original: RunKey = row["key"].clone().try_into().unwrap();
        let mode = placements[id]["mode"].as_str().unwrap();
        keys.insert(make("linked", mode, &original.path, Vec::new()));
        keys.insert(make("linked", mode, &original.path, original.faults));
    }
    keys
}

// Оракул проецирует разрешённые строки исходных заданий, без доступа к
// планировщику и без перебора недопустимых комбинаций.
fn projections(keys: &BTreeSet<RunKey>, with_fault: bool) -> (usize, usize) {
    let mut pairs = BTreeSet::new();
    let mut triples = BTreeSet::new();
    for key in keys {
        let mut levels = vec![
            format!("{}/{}", key.station, key.mode),
            key.path.clone(),
            key.grid.clone(),
        ];
        if with_fault {
            levels.push(format!("{:?}", key.faults));
        }
        for (i, a) in levels.iter().enumerate() {
            for (j, b) in levels.iter().enumerate().skip(i + 1) {
                pairs.insert((i, a.clone(), j, b.clone()));
                for (k, c) in levels.iter().enumerate().skip(j + 1) {
                    triples.insert((i, a.clone(), j, b.clone(), k, c.clone()));
                }
            }
        }
    }
    (pairs.len(), triples.len())
}

fn toml_file(path: PathBuf) -> toml::Value {
    toml::from_str(&fs::read_to_string(path).unwrap()).unwrap()
}

// Берём знаменатель из независимого реестра, а зачёт — из квитанций строк.
fn oracle_counts<const N: usize>(
    factors: &DepthFactors,
    safety_only: bool,
    rows: &[(Selection, bool)],
) -> Value {
    let registry = required_interactions::<N>(factors, safety_only);
    let covered = |executed_only: bool| {
        registry
            .iter()
            .filter(|tuple| {
                rows.iter().any(|(selection, executed)| {
                    (!executed_only || *executed)
                        && tuple.iter().all(|&(id, level)| selection[id] == level)
                })
            })
            .count()
    };
    counts(registry.len(), covered(false), covered(true))
}

#[test]
fn семь_манифестов_сохраняют_ключи_и_повторяют_журнал_в_квоте() {
    let devices = root().join("devices");
    let manifest = devices.join("depth-campaign.toml");
    let manifest_data = toml_file(manifest.clone());
    let electrical = toml_file(devices.join("electrical/acceptance.toml"));
    let history = toml_file(devices.join("stage6/depth-open.toml"));
    let historical = history["rows"].as_array().unwrap();
    let s = Scratch::new("assemblies");
    let checkpoint = s.0.join("checkpoint");
    let cp = checkpoint.to_str().unwrap();
    let started = Instant::now();
    let first = s.cli(&manifest, &["--checkpoint", cp, "--jobs", "2"], 0);
    let elapsed = started.elapsed();
    let first_report = report(&first);
    let assemblies = first_report["assemblies"].as_object().unwrap();
    assert_eq!(
        assemblies.keys().map(String::as_str).collect::<Vec<_>>(),
        electrical["assemblies"]
            .as_table()
            .unwrap()
            .keys()
            .map(String::as_str)
            .collect::<Vec<_>>()
    );
    assert_eq!(assemblies.len(), 7); // Семь установок из блока Е и строки 7.9.
    let historical_report = first_report["historical_open_rows"]["rows"]
        .as_array()
        .unwrap();
    // Девять конфликтов, двенадцать ненаблюдаемых путей и один отказ установки по §7.7.
    assert_eq!(historical_report.len(), 9 + 12 + 1);
    let original_keys: BTreeSet<RunKey> = historical
        .iter()
        .map(|r| r["key"].clone().try_into().unwrap())
        .collect();
    let actual_keys = historical_report
        .iter()
        .map(|r| {
            assert_eq!(r["outcome"], "not_analyzed");
            assert!(!r["reason"].as_str().unwrap().is_empty());
            serde_json::from_value(r["key"].clone()).unwrap()
        })
        .collect::<BTreeSet<RunKey>>();
    assert_eq!(actual_keys, original_keys);
    let bindings = campaign_bindings(std::str::from_utf8(&first.stdout).unwrap()).unwrap();
    let reservations = reserved(&checkpoint);
    assert_eq!(reservations.len(), assemblies.len());

    for (id, a) in assemblies {
        let required = required_keys(id, historical, &history["placements"]);
        let with_fault = required.iter().any(|key| !key.faults.is_empty());
        let (pairs, triples) = projections(&required, with_fault);
        let rows = a["runs"].as_object().unwrap();
        let mut planned = BTreeSet::new();
        let mut executed = BTreeSet::new();
        let mut selections = Vec::new();
        let reference = &electrical["assemblies"][id];
        let source_world = toml_file(
            devices
                .join("electrical")
                .join(reference["world"].as_str().unwrap()),
        );
        let hz = source_world["physics_hz"].as_integer().unwrap() as u64;
        let full = reference["horizon_ns"].as_integer().unwrap() as u64;
        let mut open = 0;
        let mut launched = 0;
        for (run, row) in rows {
            let receipt = &row["detail"]["coverage"];
            let key: RunKey = serde_json::from_value(receipt["key"].clone()).unwrap();
            let selection = serde_json::from_value(receipt["selection"].clone()).unwrap();
            let completed = row["status"] == "completed";
            launched += receipt["launches"].as_u64().unwrap_or(0);
            selections.push((selection, completed));
            assert!(planned.insert(key.clone()), "{id}: повтор ключа {key:?}");
            if completed {
                assert_eq!(row["executed"], 1);
                if key.station == "electrical" {
                    let rate = if key.grid == "native" { hz } else { 2 * hz };
                    let result = &row["detail"]["scenario"];
                    let live = &result["electrical"]["live"];
                    assert_eq!(live["required_ns"], full, "{id}, {run}");
                    assert_eq!(live["observed_ns"], full);
                    assert_eq!(live["complete"], true);
                    assert_eq!(result["injections_confirmed"], true);
                    assert_eq!(result["steps"], full * rate / 1_000_000_000);
                    assert_eq!(bindings[run].playback.physics_hz, rate);
                    assert_eq!(bindings[run].playback.end_ns, full);
                    assert_eq!(bindings[run].playback.seed, live["seed"].as_u64().unwrap());
                } else {
                    let case = &row["detail"]["scenario"]["profile"]["evidence"]["cases"][0];
                    assert_eq!(receipt["launches"], 2, "{id}: {run}");
                    assert_eq!(case["completed"], true);
                    assert_eq!(case["comparison_execution"]["complete"], true);
                    assert_eq!(case["comparison_execution"]["reached_ns"], case["reached_ns"]);
                    assert_eq!(case["journal"], row["playback"]["journal"]);
                    if !key.faults.is_empty() {
                        assert!(case["applied_ns"].is_u64());
                        assert_eq!(row["detail"]["scenario"]["injections_confirmed"], true);
                    }
                }
                executed.insert(key);
            } else {
                assert_eq!(key.station, "linked", "{id}: {row}");
                assert_eq!(row["executed"], 0);
                assert!(row["playback"].is_null());
                open += 1;
            }
        }
        assert_eq!(planned, required, "{id}");
        let (ran_pairs, ran_triples) = projections(&executed, with_fault);
        let depth = &a["depth_coverage"];
        let open_rows = depth["open_rows"].as_array().unwrap();
        assert_eq!(open_rows.len(), open);
        for row in open_rows {
            let run = row["run_key"].as_str().unwrap();
            let reason = row["reason"].as_str().unwrap();
            assert!(!reason.is_empty(), "{id}: {run}");
            let case = &rows[run]["detail"]["scenario"]["profile"]["evidence"]["cases"][0];
            eprintln!("{id} открыто {run}: {reason} · {} · {}", case["runtime_error"], case["missing"]);
        }
        assert_eq!((ran_pairs, ran_triples), (pairs, triples), "{id}: {open_rows:?}");
        assert_eq!(depth["pairs"], counts(pairs, pairs, ran_pairs));
        assert_eq!(
            depth["safety_triples"],
            counts(triples, triples, ran_triples)
        );
        let input = &manifest_data["assemblies"][id]["experiments"]["coverage"]["campaign"];
        let campaign = toml_file(devices.join(input.as_str().unwrap()));
        let factors: DepthFactors = campaign["depth_factors"].clone().try_into().unwrap();
        assert_eq!(
            depth["pairs"],
            oracle_counts::<2>(&factors, false, &selections),
            "{id}"
        );
        assert_eq!(
            depth["safety_triples"],
            oracle_counts::<3>(&factors, true, &selections),
            "{id}"
        );
        assert_ne!(a["verdict"], "PassedSimulation");
        let dir = format!(
            "coverage-v1-{:016x}",
            sim_signal::fnv1a(format!("{id}/coverage").bytes())
        );
        let acceptance = &depth["acceptance"];
        let reserved = acceptance["reserved_runs"].as_u64().unwrap();
        let confirmed = acceptance["confirmed_launches"].as_u64().unwrap();
        let ns = acceptance["elapsed_ns"].as_u64().unwrap();
        let time = Duration::from_nanos(ns);
        let worst = depth["worst"]["run_key"].as_str().unwrap();
        assert_eq!(a["worst_run_key"], worst);
        assert_eq!(acceptance["repeat"]["status"], "verified", "{id}: {acceptance}");
        assert_eq!(reservations[&dir], reserved);
        let worst_launches = rows[worst]["detail"]["coverage"]["launches"].as_u64().unwrap();
        assert_eq!(confirmed, launched + worst_launches);
        assert!((confirmed..=1000).contains(&reserved));
        assert!(Duration::ZERO < time && time <= Duration::from_secs(30 * 60));
        assert!(time < elapsed);
        eprintln!(
            "{id} | {pairs}/{pairs}/{ran_pairs} | {triples}/{triples}/{ran_triples} | {confirmed} ({reserved}) | {:.3} с | {worst} | {} | {open}",
            time.as_secs_f64(), a["verdict"].as_str().unwrap()
        );
    }
    assert!(elapsed < Duration::from_secs(30 * 60));
    eprintln!("Кампания по внешнему таймеру за {elapsed:.2?}");

    let normalize = |mut value: Value| {
        for assembly in value["assemblies"].as_object_mut().unwrap().values_mut() {
            assembly["depth_coverage"]["acceptance"]["elapsed_ns"] = Value::Null;
        }
        value
    };
    let second = report(&s.cli(&manifest, &["--jobs", "3"], 0));
    assert!(normalize(first_report) == normalize(second), "изменился отчёт при повторной установке");
}
