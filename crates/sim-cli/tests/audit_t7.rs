//! Проверка отчёта Т7 через штатный CLI и малые независимые реестры.

use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    path::{Path, PathBuf},
    process::{Command, Stdio},
    thread,
    time::Duration,
};

use serde_json::{Value, json};
use sim_cli::view::{RunView, campaign_bindings};

#[cfg(unix)]
#[allow(dead_code, reason = "аудит вызывает только координатор отмены")]
#[path = "../src/tasks/campaign/depth_budget.rs"]
mod depth_budget;

const MANIFEST: &str = r#"
version = 1
[assemblies.block]
scope = "Каретка"
[assemblies.block.experiments.cover]
kind = "scenario"
campaign = "campaign.toml"
installation = "install.toml"
"#;

const CAMPAIGN: &str = r#"
version = 1
world = "world.toml"
catalog = "catalog.toml"
seed = 17 # Зерно стенда из fixtures/rotary/world.toml.
[stations.block]
experiment = { kind = "scenario" }
[stations.block.modes.slide]
horizon_ns = 2000000
source = { kind = "derived", reference = "Два такта по 1 мс" }
[depth_factors.factors.mode]
safety = true
source = { kind = "derived", reference = "Свободный ход блока" }
levels = { kind = "mode", values = { slide = { station = "block", mode = "slide" } } }
[depth_factors.factors.path]
safety = true
source = { kind = "derived", reference = "Продольная ось блока" }
levels = { kind = "path", values = { line = "line" } }
[depth_factors.factors.grid]
safety = true
source = { kind = "derived", reference = "Шаги 1 и 0,5 мс" }
levels = { kind = "grid", values = { base = "base", fine = "fine" } }
"#;

const INSTALL: &str = r#"
version = 1
[coverage]
runs = 2 # Два шага сетки и один запуск на каждый.
seconds = 30 # Верхняя граница для короткой синтетической пробы.
[[paths]]
station = "block"
mode = "slide"
path = "line"
source = { kind = "derived", reference = "Продольная ось блока" }
grids = [
    { id = "base", hz = 1000, source = { kind = "derived", reference = "Шаг 1 мс" } },
    { id = "fine", hz = 2000, source = { kind = "derived", reference = "Шаг 0,5 мс" } },
]
"#;

const LIMIT: &str = r#"
# В изолированном стенде без усилий x = 0. Порог 1 м равен ребру куба.
criteria = [{ id = "ход", window = "whole", limit = { kind = "range", signal = { name = "qpos[0]", unit = "m" }, max = 1.0, source = { kind = "measured", reference = "Ребро куба стенда" } } }]
[stations.block.modes.slide.windows.whole]
start_ns = 0
end_ns = 2000000
max_gap_ns = 1000000
"#;

struct Stand(PathBuf);

impl Stand {
    fn new(name: &str) -> Self {
        let path = std::env::temp_dir().join(format!("audit-t7-{}-{name}", std::process::id()));
        fs::create_dir(&path).unwrap();
        let stand = Self(path);
        // Точка и повтор используют один исполняемый файл даже во время соседней сборки.
        fs::copy(env!("CARGO_BIN_EXE_sim-run"), stand.0.join("sim-run")).unwrap();
        stand
    }

    fn write(&self, name: &str, data: &str) {
        fs::write(self.0.join(name), data).unwrap();
    }

    fn run(&self, args: &[&str], code: i32) -> Value {
        let mut command = Command::new(self.0.join("sim-run"));
        command.arg("--campaign")
            .arg(self.0.join("manifest.toml"))
            .arg("--json");
        if !args.contains(&"--jobs") {
            command.args(["--jobs", "1"]);
        }
        let output = command.args(args).output().unwrap();
        assert_eq!(
            output.status.code(),
            Some(code),
            "{}\n{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
        serde_json::from_slice(&output.stdout).unwrap()
    }

    fn block(name: &str, install: &str) -> Self {
        let stand = Self::new(name);
        stand.write("manifest.toml", MANIFEST);
        stand.write(
            "campaign.toml",
            &CAMPAIGN.replace(
                "[depth_factors.factors.mode]",
                &format!("{LIMIT}\n[depth_factors.factors.mode]"),
            ),
        );
        stand.write("install.toml", install);
        stand.write("catalog.toml", "version = 1");
        stand.write(
            "world.toml",
            r#"
format = 3
name = "Каретка"
seed = 17
physics_hz = 1000
gravity_m_s2 = [0, 0, 0] # Горизонтальная стендовая направляющая.
[environment]
density_kg_m3 = 0
viscosity_pa_s = 0
wind_m_s = [0, 0, 0]
[[devices]]
name = "блок"
file = "block.toml"
"#,
        );
        stand.write("block.toml", r#"
format = 3
name = "Блок"
[[links]]
name = "каретка"
joint = { kind = "slide", axis = [1, 0, 0] }
# Единичный куб массы 6 кг, I = ma²/6.
mass = { kind = "explicit", mass_kg = 6, com_m = [0,0,0], inertia_kg_m2 = [1,1,1,0,0,0], provenance = { measured = "Единичный куб" } }
"#);
        stand
    }

    fn state(&self) -> PathBuf {
        self.0.join("state")
    }

    fn budget_file(&self) -> PathBuf {
        self.state().join(format!(
            "coverage-v1-{:016x}/depth-budget-v1.json",
            sim_signal::fnv1a("block/cover".bytes())
        ))
    }
}

impl Drop for Stand {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

fn assembly(value: &Value) -> &Value {
    &value["assemblies"]["block"]
}

fn count(required: usize, planned: usize, executed: usize) -> Value {
    json!({ "required": required, "planned": planned, "executed": executed })
}

#[test]
fn квота_и_дедлайн_не_закрывают_проекции_и_вердикт() {
    // Из одной строки получается C(3,2)=3 пары и одна тройка.
    // По двум сеткам пары с сеткой дают 4, пара без сетки даёт 1.
    for (name, installation, pairs, triples, reason) in [
        (
            "quota",
            INSTALL.replace("runs = 2", "runs = 1"),
            3,
            1,
            "квот",
        ),
        (
            "deadline",
            INSTALL.replace("seconds = 30", "seconds = 0"),
            0,
            0,
            "дедлайн",
        ),
    ] {
        let stand = Stand::block(name, &installation);
        let report = stand.run(&[], 0);
        let a = assembly(&report);
        let depth = &a["depth_coverage"];
        assert_eq!(depth["status"], "incomplete");
        assert_eq!(depth["pairs"], count(5, 5, pairs));
        assert_eq!(depth["safety_triples"], count(2, 2, triples));
        assert_eq!(a["verdict"], "NotProven");
        if pairs != 0 {
            let covered = a["runs"]
                .as_object()
                .unwrap()
                .values()
                .find(|row| row["executed"] == 1)
                .unwrap();
            assert_eq!(covered["detail"]["scenario"]["summary"]["outcome"], "pass");
        }
        let open = depth["open_rows"].as_array().unwrap();
        assert_eq!(open.len(), 2 - triples);
        assert!(
            open.iter()
                .all(|row| row["reason"].as_str().unwrap().contains(reason))
        );
    }
}

#[test]
fn пропавший_бюджет_не_заменяется_новым_при_продолжении() {
    let stand = Stand::block("missing-budget", &INSTALL.replace("runs = 2", "runs = 1"));
    let cp = stand.state();
    let args = ["--checkpoint", cp.to_str().unwrap()];
    let first = stand.run(&args, 0);
    assert_eq!(assembly(&first)["coverage"]["executed"], 1);
    let state = stand.budget_file();
    let envelope: Value = serde_json::from_slice(&fs::read(&state).unwrap()).unwrap();
    let saved: Value = serde_json::from_str(envelope["payload"].as_str().unwrap()).unwrap();
    assert_eq!(saved["reserved"], 1);
    fs::remove_file(&state).unwrap();
    let rejected = stand.run(&args, 2);
    assert!(
        rejected["detail"]
            .as_str()
            .unwrap()
            .contains("состояние бюджета")
    );
    assert!(!state.exists());
}

#[test]
fn квитанция_без_выгруженной_записи_не_подтверждает_повтор() {
    // Два исходных запуска и два повтора той же строки.
    let stand = Stand::block("recording", &INSTALL.replace("runs = 2", "runs = 4"));
    let cp = stand.state();
    let cp = cp.to_str().unwrap();
    let first = stand.run(&["--checkpoint", cp], 0);
    let key = first["worst_run_key"].as_str().unwrap();
    let repeat = stand.run(&["--checkpoint", cp, "--repeat", key], 0);
    assert_eq!(repeat["depth_repeat"]["status"], "not_analyzed");
    let file = stand.0.join("row.simview");
    let repeat = stand.run(
        &[
            "--checkpoint",
            cp,
            "--repeat",
            key,
            "--view",
            file.to_str().unwrap(),
        ],
        0,
    );
    assert_eq!(repeat["depth_repeat"]["status"], "verified");
    let view = RunView::read(&fs::read(&file).unwrap()).unwrap();
    let bindings = campaign_bindings(&first.to_string()).unwrap();
    assert_eq!(view.metadata.campaign.as_ref(), Some(&bindings[key]));
    assert_eq!(view.journal.fingerprint(), bindings[key].playback.journal);
}

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .unwrap()
}

fn linked(stand: &Stand) -> String {
    let fixture = root().join("crates/sim-cli/tests/fixtures/rotary");
    for file in fs::read_dir(fixture).unwrap() {
        let file = file.unwrap();
        fs::copy(file.path(), stand.0.join(file.file_name())).unwrap();
    }
    let task = fs::read_to_string(stand.0.join("task.toml")).unwrap();
    stand.write("task.toml", &task.replace(", comparison_value = 0", ""));
    stand.write("manifest.toml", MANIFEST);
    stand.write("catalog.toml", "version = 1");
    stand.write(
        "install.toml",
        &INSTALL
            .replace("station = \"block\"", "station = \"rod\"")
            .replace("mode = \"slide\"", "mode = \"измерение\"")
            .replace("path = \"line\"", "path = \"напряжение\""),
    );
    CAMPAIGN
        .replace(
            "kind = \"scenario\"",
            "kind = \"control\", task = \"task.toml\"",
        )
        .replace("stations.block", "stations.rod")
        .replace(
            "[stations.rod.modes.slide]",
            "[stations.rod.modes.\"измерение\"]",
        )
        .replace(
            "station = \"block\", mode = \"slide\"",
            "station = \"rod\", mode = \"измерение\"",
        )
        .replace("line = \"line\"", "line = \"напряжение\"")
        .replace("2000000", "8000000")
        .replace("Два такта по 1 мс", "Горизонт из fixtures/rotary/task.toml")
        .replace("base = \"base\", fine = \"fine\"", "base = \"base\"")
}

#[cfg(unix)]
#[test]
fn связанный_опыт_с_другим_каталогом_остаётся_открытым() {
    let stand = Stand::new("catalog");
    let campaign = linked(&stand);
    std::os::unix::fs::symlink(root().join("catalog"), stand.0.join("catalog")).unwrap();
    stand.write(
        "campaign.toml",
        &campaign.replace("catalog.toml", "catalog"),
    );
    let rated = stand.run(&[], 0);
    assert_eq!(assembly(&rated)["coverage"]["executed"], 1);
    assert_eq!(
        assembly(&rated)["depth_coverage"]["safety_triples"],
        count(1, 1, 1)
    );
    // Рядом лежит рабочая папка каталога, а в плане выбран другой вход.
    stand.write("campaign.toml", &campaign);
    let wrong = stand.run(&[], 0);
    let a = assembly(&wrong);
    assert_eq!(a["verdict"], "NotProven");
    assert_eq!(a["depth_coverage"]["pairs"], count(3, 3, 0));
    assert_eq!(a["depth_coverage"]["safety_triples"], count(1, 1, 0));
    assert!(
        a["depth_coverage"]["open_rows"][0]["reason"]
            .as_str()
            .unwrap()
            .contains("каталог")
    );
}

#[test]
fn связанный_режим_с_подменой_мира_не_закрывает_строку() {
    let stand = Stand::new("changed-world");
    let campaign = linked(&stand).replace("catalog.toml", root().join("catalog").to_str().unwrap());
    stand.write("campaign.toml", &campaign);
    let world = fs::read_to_string(stand.0.join("world.toml")).unwrap();
    let device = fs::read_to_string(stand.0.join("device.toml")).unwrap();
    stand.write(
        "other-world.toml",
        &world.replace("device.toml", "other-device.toml"),
    );
    stand.write("other-device.toml", &device);
    // При 60° единичное усиление graph.toml даёт команду π/3 > 1.
    // Подмена на исходные 30° укладывает её в диапазон скважности.
    stand.write(
        "device.toml",
        &device.replace("initial_deg = 30", "initial_deg = 60"),
    );
    let base = stand.run(&[], 0);
    assert_eq!(assembly(&base)["coverage"]["executed"], 0);
    assert!(
        base.to_string()
            .contains("скважность должна быть в диапазоне")
    );
    let profile = fs::read_to_string(stand.0.join("profile.toml")).unwrap();
    stand.write(
        "profile.toml",
        &format!(
            r#"{profile}
[mode_physics."измерение"]
case = "mode-physics.toml"
witness = "угол"
"#
        ),
    );
    stand.write("mode-physics.toml", r#"
source = "Положение оси и два мира из теста стержня"
window_ns = [0, 8000000]
world = "other-world.toml"
[models.angle]
kind = "coordinate"
joint = "стенд/поворот"
[[criteria]]
name = "угол"
unit = "rad"
series = { position = 0 }
statistic = "maximum"
upper = { value = 1, source = { derived = "Верхняя граница из task.toml" } }
tolerance = { value = 0.000001, source = { derived = "Масштаб смещения от единичного пробного момента на шаге 1 мс" } }
remainder = [
    { value = 0, source = { derived = "Заданное отсчётное число для теста подмены" } },
    { value = 0, source = { derived = "Нет контактов у стенда" } },
    { value = 0, source = { derived = "Один точный тензор из синтетической постановки" } },
]
"#);
    let wrong = stand.run(&[], 0);
    let a = assembly(&wrong);
    assert_eq!(a["depth_coverage"]["pairs"], count(3, 3, 0));
    assert_eq!(a["depth_coverage"]["safety_triples"], count(1, 1, 0));
    assert_eq!(a["verdict"], "NotProven");
    assert!(
        a["depth_coverage"]["open_rows"][0]["reason"]
            .as_str()
            .unwrap()
            .contains("физика связанного режима")
    );
}

#[test]
fn упавший_дочерний_расчёт_не_даёт_покрытия() {
    let stand = Stand::new("failed-child");
    let campaign = linked(&stand).replace("catalog.toml", root().join("catalog").to_str().unwrap());
    stand.write("campaign.toml", &campaign);
    let profile = fs::read_to_string(stand.0.join("profile.toml")).unwrap();
    stand.write(
        "profile.toml",
        &profile.replace("drive.toml", "missing-drive.toml"),
    );
    let cp = stand.state();
    let error = stand.run(&["--checkpoint", cp.to_str().unwrap()], 2);
    assert_eq!(error["status"], "error");
    assert!(error["detail"].as_str().unwrap().contains("дочерний опыт"));
    assert!(
        error["detail"]
            .as_str()
            .unwrap()
            .contains("missing-drive.toml")
    );
    assert!(error.get("assemblies").is_none());
    let envelope: Value = serde_json::from_slice(&fs::read(stand.budget_file()).unwrap()).unwrap();
    let budget: Value = serde_json::from_str(envelope["payload"].as_str().unwrap()).unwrap();
    assert_eq!(budget["reserved"], 1);
}

#[cfg(unix)]
#[test]
fn отменённый_дочерний_процесс_не_возвращает_успех() {
    use depth_budget::{BudgetError, DepthBudget, DepthLimits};
    let stand = Stand::new("cancelled-child");
    let marker = stand.0.join("started");
    let budget = DepthBudget::start(
        DepthLimits::new(1, Duration::from_secs(30)).unwrap(),
        "синтетический дочерний опыт",
        None,
    )
    .unwrap();
    let permit = budget.reserve().unwrap();
    thread::scope(|scope| {
        let mut command = Command::new("/bin/sh");
        command
            .args([
                "-c",
                "printf '%s\\n' \"$$\" > \"$1\"; exec /bin/sleep 30",
                "stand",
            ])
            .arg(&marker)
            .stdin(Stdio::null());
        let result = scope.spawn(|| permit.run(command));
        for _ in 0..100 {
            if fs::read_to_string(&marker).is_ok_and(|p| p.ends_with('\n')) {
                break;
            }
            thread::sleep(Duration::from_millis(10));
        }
        budget.cancel_and_wait().unwrap();
        assert!(marker.exists(), "процесс должен успеть начать работу");
        assert!(matches!(
            result.join().unwrap(),
            Err(BudgetError::Cancelled)
        ));
        assert_eq!(budget.usage().unwrap().reserved, 1);
        let pid = fs::read_to_string(marker).unwrap();
        assert!(
            !Command::new("/bin/kill")
                .args(["-0", pid.trim()])
                .stderr(Stdio::null())
                .status()
                .unwrap()
                .success()
        );
    });
}

type Selection = BTreeMap<String, String>;

fn toml_file(path: &Path) -> toml::Value {
    toml::from_str(&fs::read_to_string(path).unwrap()).unwrap()
}

// Для двух малых манифестов перечисляем контексты независимо от планировщика.
fn allowed(factors: &toml::Value) -> BTreeSet<Selection> {
    let mut rows = vec![Selection::new()];
    for (id, factor) in factors["factors"].as_table().unwrap() {
        let levels = factor["levels"]["values"].as_table().unwrap();
        rows = rows
            .into_iter()
            .flat_map(|row| {
                levels.keys().map(move |level| {
                    let mut row = row.clone();
                    row.insert(id.clone(), level.clone());
                    row
                })
            })
            .collect();
    }
    rows.into_iter()
        .filter(|row| {
            !factors
                .get("constraints")
                .and_then(toml::Value::as_table)
                .into_iter()
                .flatten()
                .any(|(_, rule)| {
                    rule["when"]
                        .as_table()
                        .unwrap()
                        .iter()
                        .all(|(id, level)| row[id] == level.as_str().unwrap())
                })
        })
        .collect()
}

fn project(rows: &BTreeSet<Selection>, safety: &BTreeSet<&str>) -> (usize, usize) {
    let mut pairs = BTreeSet::new();
    let mut triples = BTreeSet::new();
    for row in rows {
        let fields: Vec<_> = row.iter().collect();
        for (i, &a) in fields.iter().enumerate() {
            for (j, &b) in fields.iter().enumerate().skip(i + 1) {
                pairs.insert([a, b]);
                for &c in fields.iter().skip(j + 1) {
                    if [a, b, c].iter().all(|(id, _)| safety.contains(id.as_str())) {
                        triples.insert([a, b, c]);
                    }
                }
            }
        }
    }
    (pairs.len(), triples.len())
}

#[test]
fn выборка_режимов_отказов_и_масс_не_перемножается() {
    let (limits, path) = INSTALL.split_once("[[paths]]").unwrap();
    let installation = format!(
        "{}[[paths]]{path}[[paths]]{}",
        limits.replace("runs = 2", "runs = 0"),
        path.replace("mode = \"slide\"", "mode = \"park\""),
    );
    let stand = Stand::block("product", &installation);
    let input = fs::read_to_string(stand.0.join("campaign.toml")).unwrap();
    let input = input
        .replace(
            "[depth_factors.factors.mode]",
            &format!("[stations.block.modes.park]\nhorizon_ns = 2000000\nsource = {{ kind = 'derived', reference = 'Два такта тех же граничных условий' }}\n{}\n[depth_factors.factors.mode]", LIMIT.replace(".slide.", ".park.")),
        )
        .replace(r#"mode = "slide" } }"#, r#"mode = "slide" }, park = { station = "block", mode = "park" } }"#);
    stand.write("campaign.toml", &format!(r#"{input}
[depth_factors.factors.mass]
safety = false
source = {{ kind = "measured", reference = "Куб 6 кг и куб удвоенной плотности" }}
levels = {{ kind = "axis", parameter = {{ kind = "mass", body = "блок/каретка" }}, unit = "kg", nominal = 6.0, values = {{ light = 6.0, heavy = 12.0 }} }}
[depth_factors.factors.fault]
safety = false
source = {{ kind = "measured", reference = "Два состояния линии стенда" }}
levels = {{ kind = "faults", values = {{ intact = [], open = [{{ node = "line", kind = "wire_open" }}] }} }}
[depth_factors.constraints.incompatible]
when = {{ fault = "open", grid = "fine", mass = "heavy" }}
reason = "Регистратор тяжёлого образца при обрыве оставляет базовый шаг"
source = {{ kind = "measured", reference = "Таблица режимов синтетического регистратора" }}
"#));
    let spec = toml_file(&stand.0.join("campaign.toml"));
    let admissible = allowed(&spec["depth_factors"]);
    // 2⁴ сочетания и два режима для исключённой тройки уровней.
    assert_eq!(admissible.len(), 16 - 2);
    let report = stand.run(&[], 0);
    let a = assembly(&report);
    let planned: BTreeSet<Selection> = a["runs"]
        .as_object()
        .unwrap()
        .values()
        .map(|row| {
            assert_eq!(row["executed"], 0);
            serde_json::from_value(row["detail"]["coverage"]["selection"].clone()).unwrap()
        })
        .collect();
    assert!(planned.is_subset(&admissible));
    assert!(planned.len() < admissible.len());
    let safety = BTreeSet::from(["fault", "grid", "mode", "path"]);
    // Пары четырёх двоичных факторов и их пары с постоянным путём.
    // Тройки: одна без пути и три с постоянным путём. Запрет по массе
    // не исключает проекцию, потому что остаётся лёгкий образец.
    let expected = (6 * 4 + 4 * 2, 8 + 3 * 4);
    assert_eq!(project(&admissible, &safety), expected);
    assert_eq!(project(&planned, &safety), expected);
    assert_eq!(
        a["depth_coverage"]["pairs"],
        count(expected.0, expected.0, 0)
    );
    assert_eq!(
        a["depth_coverage"]["safety_triples"],
        count(expected.1, expected.1, 0)
    );
}

#[test]
fn две_сборки_совпадают_с_независимым_оракулом() {
    let stand = Stand::new("oracle");
    let devices = root().join("devices");
    let mut manifest = toml_file(&devices.join("depth-campaign.toml"));
    let retained = ["03", "07"];
    let assemblies = manifest["assemblies"].as_table_mut().unwrap();
    assemblies.retain(|id, _| retained.contains(&id));
    for (_, entry) in assemblies.iter_mut() {
        let cover = &mut entry["experiments"]["coverage"];
        for field in ["campaign", "installation"] {
            cover[field] = toml::Value::String(
                devices
                    .join(cover[field].as_str().unwrap())
                    .display()
                    .to_string(),
            );
        }
    }
    manifest["open_rows"] = toml::Value::String(
        devices
            .join(manifest["open_rows"].as_str().unwrap())
            .display()
            .to_string(),
    );
    stand.write("manifest.toml", &toml::to_string(&manifest).unwrap());
    let report = stand.run(&["--jobs", "3"], 0);
    for (id, expected) in [("03", (5, 2)), ("07", (41, 44))] {
        let a = &report["assemblies"][id];
        let spec = &manifest["assemblies"][id]["experiments"]["coverage"];
        let file = toml_file(Path::new(spec["campaign"].as_str().unwrap()));
        let factors = &file["depth_factors"];
        let safe = factors["factors"]
            .as_table()
            .unwrap()
            .iter()
            .filter(|(_, data)| data["safety"].as_bool().unwrap())
            .map(|(id, _)| id.as_str())
            .collect();
        let admissible = allowed(factors);
        let mut planned = BTreeSet::new();
        let mut executed = BTreeSet::new();
        for row in a["runs"].as_object().unwrap().values() {
            let selection: Selection =
                serde_json::from_value(row["detail"]["coverage"]["selection"].clone()).unwrap();
            assert!(admissible.contains(&selection));
            assert!(planned.insert(selection.clone()));
            if row["status"] == "completed" {
                let scenario = &row["detail"]["scenario"];
                let playback = &row["playback"];
                if selection["path"] == "electric" {
                    let live = &scenario["electrical"]["live"];
                    let end = live["required_ns"].as_u64().unwrap();
                    let hz = playback["physics_hz"].as_u64().unwrap();
                    assert_eq!(live["observed_ns"], end);
                    assert_eq!(scenario["steps"], end * hz / 1_000_000_000);
                    assert_eq!(playback["journal"], scenario["journal"]);
                } else {
                    let case = &scenario["profile"]["evidence"]["cases"][0];
                    assert_eq!(case["completed"], true);
                    assert_eq!(case["comparison_execution"]["complete"], true);
                    assert_eq!(case["journal"], playback["journal"]);
                    assert_eq!(case["reached_ns"], playback["end_ns"]);
                }
                assert_eq!(scenario["injections_confirmed"], true);
                assert_eq!(row["executed"], 1);
                executed.insert(selection);
            } else {
                assert_eq!(row["executed"], 0);
                assert!(row["playback"].is_null());
            }
        }
        assert_eq!(executed, admissible);
        let (required, planned, executed) = (
            project(&admissible, &safe),
            project(&planned, &safe),
            project(&executed, &safe),
        );
        assert_eq!(required, expected);
        assert_eq!(executed, expected);
        assert_eq!(required, planned);
        assert_eq!(
            a["depth_coverage"]["pairs"],
            count(required.0, planned.0, executed.0)
        );
        assert_eq!(
            a["depth_coverage"]["safety_triples"],
            count(required.1, planned.1, executed.1)
        );
        assert_ne!(a["verdict"], "PassedSimulation");
        assert_eq!(a["depth_coverage"]["worst"]["ranking"], "outcome");
        assert_eq!(
            a["depth_coverage"]["worst"]["margin"]["status"],
            "not_analyzed"
        );
        eprintln!(
            "{id}: пары {}/{}/{}, тройки {}/{}/{}",
            required.0, planned.0, executed.0, required.1, planned.1, executed.1
        );
    }
}

#[test]
fn установки_01_и_07_совпадают_с_проекциями_допустимых_строк() {
    let stand = Stand::new("oracle-01-07");
    let devices = root().join("devices");
    let mut manifest = toml_file(&devices.join("depth-campaign.toml"));
    let assemblies = manifest["assemblies"].as_table_mut().unwrap();
    assemblies.retain(|id, _| matches!(id, "01" | "07"));
    for (_, entry) in assemblies.iter_mut() {
        let cover = &mut entry["experiments"]["coverage"];
        for field in ["campaign", "installation"] {
            let path = devices.join(cover[field].as_str().unwrap());
            cover[field] = path.to_str().unwrap().into();
        }
    }
    let history = devices.join(manifest["open_rows"].as_str().unwrap());
    manifest["open_rows"] = history.to_str().unwrap().into();
    stand.write("manifest.toml", &toml::to_string(&manifest).unwrap());
    let report = stand.run(&["--jobs", "2"], 0);

    for (id, row_count, expected) in [("01", 6, (23, 20)), ("07", 16, (41, 44))] {
        let a = &report["assemblies"][id];
        let cover = &manifest["assemblies"][id]["experiments"]["coverage"];
        let path = Path::new(cover["campaign"].as_str().unwrap());
        let spec = toml_file(path);
        let included: Vec<_> = spec
            .get("include")
            .and_then(toml::Value::as_array)
            .into_iter()
            .flatten()
            .map(|file| toml_file(&path.parent().unwrap().join(file.as_str().unwrap())))
            .collect();
        let factors = &spec["depth_factors"];
        let admissible = allowed(factors);
        let safety = factors["factors"]
            .as_table()
            .unwrap()
            .iter()
            .filter(|(_, factor)| {
                factor["levels"]["kind"] == toml::Value::from("faults")
                    || factor["safety"].as_bool().unwrap()
            })
            .map(|(id, _)| id.as_str())
            .collect();
        assert_eq!(admissible.len(), row_count);
        assert_eq!(project(&admissible, &safety), expected);
        let mut planned = BTreeSet::new();
        let mut executed = BTreeSet::new();
        for row in a["runs"].as_object().unwrap().values() {
            let selection: Selection =
                serde_json::from_value(row["detail"]["coverage"]["selection"].clone()).unwrap();
            assert!(admissible.contains(&selection));
            assert!(planned.insert(selection.clone()));
            let live = &row["detail"]["scenario"];
            let playback = &row["playback"];
            let duration = playback["end_ns"].as_u64().unwrap();
            let hz = playback["physics_hz"].as_u64().unwrap();
            let mode = &factors["factors"]["mode"]["levels"]["values"][&selection["mode"]];
            let station = mode["station"].as_str().unwrap();
            let mode = mode["mode"].as_str().unwrap();
            let horizon = std::iter::once(&spec)
                .chain(&included)
                .find_map(|document| {
                    document
                        .get("stations")?
                        .get(station)?
                        .get("modes")?
                        .get(mode)
                })
                .unwrap()["horizon_ns"]
                .as_integer()
                .unwrap() as u64;
            assert_eq!(duration, horizon);
            if selection["path"] == "electric" {
                assert_eq!(live["electrical"]["live"]["observed_ns"], horizon);
                assert_eq!(live["steps"], horizon * hz / 1_000_000_000); // Наносекунд в секунде.
                assert_eq!(live["journal"], playback["journal"]);
            } else {
                let case = &live["profile"]["evidence"]["cases"][0];
                assert_eq!(case["reached_ns"], horizon);
                assert_eq!(case["journal"], playback["journal"]);
                assert_eq!(case["comparison_execution"]["complete"], true);
                assert_eq!(case["comparison_execution"]["reached_ns"], horizon);
                assert_eq!(case["comparison_execution"]["seed"], playback["seed"]);
            }
            assert_eq!(live["injections_confirmed"], true);
            assert_eq!(row["executed"], 1);
            assert_eq!(row["status"], "completed");
            assert!(executed.insert(selection));
        }
        let required = project(&admissible, &safety);
        let planned = project(&planned, &safety);
        let executed = project(&executed, &safety);
        let depth = &a["depth_coverage"];
        assert_eq!(depth["pairs"], count(required.0, planned.0, executed.0));
        assert_eq!(
            depth["safety_triples"],
            count(required.1, planned.1, executed.1)
        );
        assert_eq!(executed, expected);
        assert_eq!(depth["acceptance"]["repeat"]["status"], "verified");
        assert_ne!(a["verdict"], "PassedSimulation");
        eprintln!(
            "{id}: пары {}/{}/{}, тройки {}/{}/{}",
            required.0, planned.0, executed.0, required.1, planned.1, executed.1
        );
    }

    // Отдельная постановка даже с журналом не заменяет исходный RunKey.
    let original = toml_file(&history);
    let expected = original["rows"].as_array().unwrap();
    let actual = report["historical_open_rows"]["rows"].as_array().unwrap();
    assert_eq!(actual.len(), expected.len());
    for row in expected {
        let key: sim_task::campaign::RunKey = row["key"].clone().try_into().unwrap();
        let key = serde_json::to_value(key).unwrap();
        let actual = actual.iter().find(|r| r["key"] == key).unwrap();
        assert_eq!(actual["outcome"], "not_analyzed");
        assert_eq!(actual["reason"], row["reason"].as_str().unwrap());
        assert!(!actual["reason"].as_str().unwrap().trim().is_empty());
        assert_eq!(actual["source"], row["source"].as_str().unwrap());
    }
}

#[test]
fn сохранённая_строка_без_квитанции_журнала_не_засчитывается() {
    let stand = Stand::block("missing-witness", INSTALL);
    let cp = stand.state();
    let args = ["--checkpoint", cp.to_str().unwrap()];
    let original = stand.run(&args, 0);
    assert_eq!(
        assembly(&original)["depth_coverage"]["pairs"],
        count(5, 5, 5)
    );
    let completed = assembly(&original)["runs"]
        .as_object()
        .unwrap()
        .values()
        .next()
        .unwrap();
    assert_eq!(completed["status"], "completed");
    assert!(completed["playback"]["journal"].is_u64());
    let input = completed["detail"]["coverage"]["input"].as_u64().unwrap();
    let file = format!("{input:016x}.json");
    let dir = stand.budget_file();
    let receipt = fs::read_dir(dir.parent().unwrap())
        .unwrap()
        .map(|entry| entry.unwrap().path().join(&file))
        .find(|path| path.is_file())
        .unwrap();
    rewrite_receipt(&receipt, |row| row["playback"] = Value::Null);

    let error = stand.run(&args, 2);
    assert_eq!(error["status"], "error");
    assert!(
        error["detail"]
            .as_str()
            .unwrap()
            .contains("строке покрытия")
    );
    assert!(error.get("assemblies").is_none());
}

fn rewrite_receipt(path: &Path, change: impl FnOnce(&mut Value)) {
    let mut envelope: Value = serde_json::from_slice(&fs::read(path).unwrap()).unwrap();
    let mut payload: Value = serde_json::from_str(envelope["payload"].as_str().unwrap()).unwrap();
    change(&mut payload);
    let payload = serde_json::to_string(&payload).unwrap();
    envelope["checksum"] = sim_signal::fnv1a(payload.bytes()).into();
    envelope["payload"] = payload.into();
    fs::write(path, serde_json::to_vec(&envelope).unwrap()).unwrap();
}

#[test]
fn парный_опыт_и_повтор_оплачивают_обе_ветви() {
    let stand = Stand::new("paired");
    let campaign = linked(&stand).replace("catalog.toml", root().join("catalog").to_str().unwrap());
    stand.write("campaign.toml", &campaign);
    let task = include_str!("fixtures/rotary/task.toml");
    stand.write("task.toml", task);
    let install = fs::read_to_string(stand.0.join("install.toml")).unwrap();
    // Две ветви исходного опыта и ещё две для повтора.
    stand.write("install.toml", &install.replace("runs = 2", "runs = 1"));
    let short = stand.run(&[], 0);
    let a = assembly(&short);
    assert_eq!(a["depth_coverage"]["pairs"], count(3, 3, 0));
    assert_eq!(a["depth_coverage"]["safety_triples"], count(1, 1, 0));
    assert_eq!(a["verdict"], "NotProven");
    assert!(
        a["depth_coverage"]["open_rows"][0]["reason"]
            .as_str()
            .unwrap()
            .contains("квот")
    );

    stand.write("install.toml", &install.replace("runs = 2", "runs = 6"));
    let cp = stand.state();
    let state = cp.to_str().unwrap();
    let first = stand.run(&["--checkpoint", state], 0);
    let a = assembly(&first);
    assert_eq!(a["depth_coverage"]["pairs"], count(3, 3, 3));
    let key = first["worst_run_key"].as_str().unwrap();
    let row = &a["runs"][key];
    assert_eq!(row["detail"]["coverage"]["launches"], 2);
    let case = &row["detail"]["scenario"]["profile"]["evidence"]["cases"][0];
    let comparison = &case["comparison_execution"];
    assert_eq!(comparison["complete"], true);
    assert_ne!(comparison["journal"], case["journal"]);
    let archive = stand.0.join("pair.simview");
    let args = [
        "--checkpoint",
        state,
        "--repeat",
        key,
        "--view",
        archive.to_str().unwrap(),
    ];
    let repeat = stand.run(&args, 0);
    assert_eq!(repeat["depth_repeat"]["status"], "verified");
    let view = RunView::read(&fs::read(&archive).unwrap()).unwrap();
    assert_eq!(json!(view.journal.fingerprint()), case["journal"]);
    fs::remove_file(&archive).unwrap();

    let filename = format!(
        "depth-report-v1-{:016x}.json",
        sim_signal::fnv1a(key.bytes())
    );
    let receipt = fs::read_dir(&cp)
        .unwrap()
        .map(|entry| entry.unwrap().path().join(&filename))
        .find(|path| path.is_file())
        .unwrap();
    // Меняем отпечаток исходной оценки, оставляя привязку основного журнала.
    rewrite_receipt(&receipt, |saved| {
        saved["report"] = json!(saved["report"].as_u64().unwrap() ^ 1);
    });
    let error = stand.run(&args, 2);
    assert!(
        error["detail"]
            .as_str()
            .unwrap()
            .contains("повтор покрытия расходится")
    );
    assert!(!archive.exists());
}

#[test]
fn два_фактора_безопасности_и_старый_флаг_отказа_различаются() {
    let stand = Stand::block("triple-flags", INSTALL);
    let input = fs::read_to_string(stand.0.join("campaign.toml")).unwrap();
    let (before, grid) = input.split_once("[depth_factors.factors.grid]").unwrap();
    let two = format!(
        "{before}[depth_factors.factors.grid]{}",
        grid.replace("safety = true", "safety = false")
    );
    // При полном горизонте оценочный предел сохраняет NotAnalyzed.
    let two = two.replace(
        "kind = \"measured\", reference = \"Ребро",
        "kind = \"estimate\", reference = \"Ребро",
    );
    stand.write("campaign.toml", &two);
    let two_report = stand.run(&[], 0);
    let a = assembly(&two_report);
    assert_eq!(a["depth_coverage"]["pairs"], count(5, 5, 5));
    assert_eq!(a["depth_coverage"]["safety_triples"], count(0, 0, 0));
    assert!(
        a["depth_coverage"]["safety_triples_not_required"]
            .as_str()
            .unwrap()
            .ends_with('2')
    );
    assert_eq!(a["verdict"], "NotProven");
    for row in a["runs"].as_object().unwrap().values() {
        assert_eq!(
            row["detail"]["scenario"]["summary"]["outcome"],
            "not_analyzed"
        );
    }

    stand.write("campaign.toml", &format!(r#"{two}
[depth_factors.factors.fault]
safety = false
source = {{ kind = "measured", reference = "Режим разрыва синтетической линии" }}
levels = {{ kind = "faults", values = {{ intact = [], open = [{{ node = "wire:line", kind = "wire_open" }}] }} }}
[depth_factors.constraints.excluded]
when = {{ fault = "open", grid = "fine" }}
source = {{ kind = "measured", reference = "Синтетическая линия допускает разрыв на базовом шаге" }}
reason = "Ограничение оснастки"
"#));
    let fault_report = stand.run(&[], 0);
    let a = assembly(&fault_report);
    let depth = &a["depth_coverage"];
    // 5 исходных пар + 2 с режимом + 2 с путём + 3 с сеткой.
    assert_eq!(depth["pairs"], count(12, 12, 9));
    assert_eq!(depth["safety_triples"], count(2, 2, 1));
    assert!(depth.get("safety_triples_not_required").is_none());
    assert_eq!(depth["open_rows"].as_array().unwrap().len(), 1);
    assert_eq!(depth["open_rows"][0]["key"]["grid"], "base");
    assert!(
        depth["open_rows"][0]["reason"]
            .as_str()
            .unwrap()
            .contains("отказ отсутствует")
    );
    assert_eq!(a["verdict"], "NotProven");
}

#[test]
fn электрический_опыт_с_инъекцией_отклоняет_отрицательный_запас() {
    let install = format!(
        r#"{}electrical_profile = "electrical.toml"
faults = [{{ node = "controller:регулятор", kind = "controller_halted" }}]
"#,
        INSTALL.replace("seconds = 30", "seconds = 30\nrepeat_worst = true")
    );
    let stand = Stand::block("electric-fault-limit", &install);
    let scheme = root().join("devices/electrical/corpus/reverse_polarity/fixed.toml");
    fs::copy(scheme, stand.0.join("scheme.toml")).unwrap();
    let block = fs::read_to_string(stand.0.join("block.toml")).unwrap();
    stand.write(
        "block.toml",
        &block
            .replace(
                "name = \"Блок\"",
                "name = \"Блок\"\nschematic = \"scheme.toml\"",
            )
            .replace(
                "axis = [1, 0, 0]",
                "axis = [1, 0, 0], efforts = [{ name = \"усилие\", dof = 0 }]",
            ),
    );
    let world = fs::read_to_string(stand.0.join("world.toml")).unwrap();
    stand.write(
        "world.toml",
        &world.replace("gravity_m_s2 = [0, 0, 0]", "gravity_m_s2 = [1, 0, 0]"),
    );
    // Ускорение 1 м/с² и нулевое усилие на двух шагах сетки 1 мс.
    // По Euler x = a·h²·(1+2) = 3 мкм, предел 11/6 мкм.
    let upper = 11e-6 / 6.0;
    let campaign = fs::read_to_string(stand.0.join("campaign.toml")).unwrap()
        .replace("catalog.toml", root().join("catalog").to_str().unwrap())
        .replace("base = \"base\", fine = \"fine\"", "base = \"base\"")
        .replace("max = 1.0", &format!("max = {upper}"))
        .replace("reference = \"Ребро куба стенда\"", "reference = \"Предел стенда 11/6 мкм\"")
        .replace("criteria = [", r#"faults = [{ fault = { node = "controller:регулятор", kind = "controller_halted" }, at_ns = 1000000, max_lag_ns = 0, window = "whole", source = { kind = "measured", reference = "Останов в середине двухтактового опыта" } }]
criteria = ["#);
    stand.write("campaign.toml", &format!(r#"{campaign}
[depth_factors.factors.interruption]
safety = false
source = {{ kind = "measured", reference = "Останов синтетического контроллера" }}
levels = {{ kind = "faults", values = {{ stopped = [{{ node = "controller:регулятор", kind = "controller_halted" }}] }} }}
"#));
    stand.write(
        "electrical.toml",
        r#"
format = 1
step_ns = 500000
source = "Половина шага механики 1 мс"
control = "control.toml"
[circuits."блок".probes.cap]
kind = "voltage"
ends = ["capacitor/plus", "capacitor/minus"]
"#,
    );
    stand.write(
        "control.toml",
        r#"
version = 1
[[controller]]
name = "регулятор"
kind = "builtin"
rate_hz = 1000
program = "graph.toml"
source = "Шаг стенда 1 мс"
[[output]]
name = "усилие"
controller = "регулятор"
target = "блок/усилие"
target_kind = "effort"
unit = "N"
max_age_ns = 1000000
fallback = 0
source = "Нулевое усилие и после останова"
[[fault]]
id = "останов"
target = "controller:регулятор"
at_ns = 1000000
kind = "pause"
source = "Останов после первого такта"
"#,
    );
    stand.write(
        "graph.toml",
        r#"
version = 1
[[block]]
kind = "constant"
value = 0
source = "Движение задаёт только постоянное ускорение"
[[bind]]
output = "усилие"
node = 0
missing = "omit"
source = "Нулевое усилие на всех тактах"
"#,
    );
    let report = stand.run(&[], 0);
    let a = assembly(&report);
    let row = a["runs"].as_object().unwrap().values().next().unwrap();
    let scenario = &row["detail"]["scenario"];
    assert_eq!(scenario["injections_confirmed"], true);
    assert_eq!(row["executed"], 1);
    assert_eq!(a["depth_coverage"]["pairs"], count(6, 6, 6)); // C(4,2).
    assert_eq!(a["depth_coverage"]["safety_triples"], count(4, 4, 4)); // C(4,3).
    let criterion = &scenario["margins"]["ход"][1];
    assert_eq!(criterion["outcome"], "fail");
    let observed = criterion["observed"][1].as_f64().unwrap();
    let tolerance = 16.0 * f64::EPSILON; // Округление двух шагов Euler и нормирования.
    assert!((observed / 3e-6 - 1.0).abs() < tolerance);
    assert!(((upper - observed) / upper + 7.0 / 11.0).abs() < tolerance);
    assert_eq!(row["outcome"], "fail");
    assert_eq!(a["verdict"], "Rejected");
    let depth = &a["depth_coverage"];
    assert_eq!(depth["acceptance"]["repeat"]["status"], "verified");
    assert_eq!(depth["worst"]["ranking"], "normalized_margin");
    assert_eq!(depth["worst"]["margin"]["outcome"], "fail");
    assert!(
        (depth["worst"]["margin"]["normalized"].as_f64().unwrap() + 7.0 / 11.0).abs() < tolerance
    );
}
