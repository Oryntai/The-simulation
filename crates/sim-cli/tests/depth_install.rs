use std::{
    collections::BTreeSet,
    fs,
    path::{Path, PathBuf},
    process::Command,
};

use serde_json::Value;

struct Stand(PathBuf);

impl Stand {
    fn new(id: &str, runs: u32, extra: &str) -> Self {
        let path = std::env::temp_dir().join(format!("depth-install-{}-{id}", std::process::id()));
        fs::create_dir(&path).unwrap();
        let stand = Self(path);
        stand.write(
            "world.toml",
            r#"
format = 3
name = "Блок"
seed = 1
physics_hz = 1000
gravity_m_s2 = [1.0, 0.0, 0.0]
[environment]
density_kg_m3 = 0.0
viscosity_pa_s = 0.0
wind_m_s = [0.0, 0.0, 0.0]
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
joint = { kind = "slide", axis = [1.0, 0.0, 0.0] }
# Единичный куб: I = m·a²/6.
mass = { kind = "explicit", mass_kg = 6.0, com_m = [0.0, 0.0, 0.0], inertia_kg_m2 = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0], provenance = { measured = "единичный куб синтетического стенда" } }
"#);
        stand.write("catalog.toml", "version = 1");
        stand.write(
            "manifest.toml",
            r#"
version = 1
[assemblies.block]
scope = "Каретка"
[assemblies.block.experiments.cover]
kind = "scenario"
campaign = "campaign.toml"
installation = "install.toml"
"#,
        );
        stand.write(
            "install.toml",
            &format!(
                r#"
version = 1
[coverage]
runs = {runs}
seconds = 30

[[paths]]
station = "block"
mode = "motion"
path = "line"
source = {{ kind = "measured", reference = "оснастка каретки" }}
grids = [
    {{ id = "base", hz = 1000, source = {{ kind = "measured", reference = "шаг стенда 1 мс" }} }},
    {{ id = "fine", hz = 2000, source = {{ kind = "measured", reference = "шаг стенда 0,5 мс" }} }},
]
{extra}
"#
            ),
        );
        stand.write("campaign.toml", r#"
version = 1
world = "world.toml"
catalog = "catalog.toml"
seed = 1
[stations.block]
experiment = { kind = "scenario" }
[stations.block.modes.motion]
horizon_ns = 2000000
source = { kind = "measured", reference = "двухмиллисекундный стенд" }
# Полунеявный Euler: x = a·h²·n(n+1)/2. Получаем 3 и 2,5 мкм.
# Верхний предел 2,75 мкм, ручной запас −0,25 и +0,25 мкм.
criteria = [{ id = "ход", window = "whole", limit = { kind = "range", signal = { name = "qpos[0]", unit = "m" }, min = 0.0, max = 0.00000275, source = { kind = "measured", reference = "порог синтетической оснастки" } } }]
[stations.block.modes.motion.windows.whole]
start_ns = 0
end_ns = 2000000
max_gap_ns = 1000000
[depth_factors.factors.operation]
safety = true
source = { kind = "measured", reference = "режим стенда" }
levels = { kind = "mode", values = { move = { station = "block", mode = "motion" } } }
[depth_factors.factors.route]
safety = true
source = { kind = "measured", reference = "путь стенда" }
levels = { kind = "path", values = { line = "line" } }
[depth_factors.factors.clock]
safety = true
source = { kind = "measured", reference = "две сетки из установки" }
levels = { kind = "grid", values = { base = "base", fine = "fine" } }
"#);
        stand
    }

    fn write(&self, name: &str, data: &str) {
        fs::write(self.0.join(name), data).unwrap();
    }

    fn run(&self, args: &[&str], code: i32) -> Value {
        let output = Command::new(env!("CARGO_BIN_EXE_sim-run"))
            .arg("--campaign")
            .arg(self.0.join("manifest.toml"))
            .arg("--json")
            .args(args)
            .output()
            .unwrap();
        assert_eq!(
            output.status.code(),
            Some(code),
            "{}",
            String::from_utf8_lossy(&output.stdout)
        );
        serde_json::from_slice(&output.stdout).unwrap()
    }
}

impl Drop for Stand {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

fn log(path: &Path) -> sim_runtime::Log {
    sim_runtime::Log::read_binary(fs::File::open(path).unwrap()).unwrap()
}

#[test]
fn две_сетки_выполняются_и_дают_ручной_запас_по_обе_стороны_порога() {
    let stand = Stand::new("grids", 2, "");
    let report = stand.run(&["--jobs", "1"], 0);
    assert_eq!(report, stand.run(&["--jobs", "2"], 0));
    let assembly = &report["assemblies"]["block"];
    assert_eq!(assembly["coverage"]["executed"], 2);
    assert_eq!(assembly["verdict"], "Rejected");
    let mut pairs = BTreeSet::new();
    let mut triples = BTreeSet::new();
    let rows = assembly["runs"].as_object().unwrap();
    assert_eq!(rows.len(), 2);
    for (key, row) in rows {
        let detail = &row["detail"];
        let selected = &detail["coverage"]["selection"];
        let grid = selected["clock"].as_str().unwrap();
        let operation = selected["operation"].as_str().unwrap();
        let route = selected["route"].as_str().unwrap();
        triples.insert((grid, operation, route));
        pairs.extend([
            ("clock", grid, "operation", operation),
            ("clock", grid, "route", route),
            ("operation", operation, "route", route),
        ]);
        let (expected, state) = match grid {
            "base" => (3.0e-6, "fail"),
            "fine" => (2.5e-6, "pass"),
            _ => panic!("неизвестная сетка {grid}"),
        };
        assert_eq!(detail["scenario"]["summary"]["outcome"], state);
        if state == "pass" {
            assert_eq!(row["outcome"], "not_analyzed");
        }
        let archive = stand.0.join("row.simlog");
        stand.run(&["--repeat", key, "--log", archive.to_str().unwrap()], 0);
        let log = log(&archive);
        fs::remove_file(&archive).unwrap();
        assert_eq!(
            log.fingerprint(),
            detail["scenario"]["journal"].as_u64().unwrap()
        );
        let observed = log
            .entries()
            .iter()
            .rev()
            .find(|entry| {
                entry.kind == sim_runtime::LogKind::Truth
                    && log.header().truth[entry.id as usize].name == "qpos[0]"
            })
            .unwrap();
        assert!((observed.value - expected).abs() < 1e-12);
    }
    assert_eq!(pairs.len(), 5);
    assert_eq!(
        triples,
        BTreeSet::from([("base", "move", "line"), ("fine", "move", "line")])
    );
}

#[test]
fn нулевая_квота_и_отсутствующая_сетка_оставляют_открытые_строки() {
    let stand = Stand::new("quota", 0, "");
    let report = stand.run(&[], 0);
    let assembly = &report["assemblies"]["block"];
    assert_eq!(assembly["verdict"], "NotProven");
    assert_eq!(assembly["coverage"]["executed"], 0);
    assert!(assembly["coverage"]["open"].as_u64().unwrap() > 0);

    let stand = Stand::new("grid-gap", 2, "");
    let path = stand.0.join("install.toml");
    let input = fs::read_to_string(&path).unwrap();
    stand.write(
        "install.toml",
        &input.replace("id = \"fine\"", "id = \"other\""),
    );
    let report = stand.run(&[], 0);
    let assembly = &report["assemblies"]["block"];
    assert_eq!(assembly["coverage"]["executed"], 1);
    assert_eq!(assembly["coverage"]["open"], 1);

    let path = stand.0.join("campaign.toml");
    let campaign = fs::read_to_string(path).unwrap();
    stand.write(
        "campaign.toml",
        &format!(
            r#"{campaign}
[depth_factors.constraints.grid_limit]
when = {{ clock = "fine" }}
reason = "стенд поддерживает только шаг 1 мс"
source = {{ kind = "measured", reference = "профиль синтетической оснастки" }}
"#
        ),
    );
    let report = stand.run(&[], 0);
    let assembly = &report["assemblies"]["block"];
    assert_eq!(assembly["coverage"]["required"], 1);
    assert_eq!(assembly["coverage"]["executed"], 1);
    assert_eq!(assembly["coverage"]["open"], 0);
}

#[test]
fn неконечный_вход_отклоняется_до_запуска() {
    let stand = Stand::new("infinity", 2, "");
    let path = stand.0.join("world.toml");
    let input = fs::read_to_string(&path).unwrap();
    for invalid in ["nan", "inf", "-inf"] {
        stand.write(
            "world.toml",
            &input.replace("[1.0, 0.0, 0.0]", &format!("[{invalid}, 0.0, 0.0]")),
        );
        assert_eq!(stand.run(&[], 2)["status"], "error");
    }
}

#[test]
fn электрический_путь_сохраняет_знак_напряжения_и_исход_стендовой_пары() {
    let stand = Stand::new("electrical", 2, "electrical_profile = 'electrical.toml'");
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .unwrap();
    let campaign = fs::read_to_string(stand.0.join("campaign.toml"))
        .unwrap()
        .replace(
            "catalog = \"catalog.toml\"",
            &format!("catalog = '{}'", root.join("catalog").display()),
        )
        // Порог 4 мкм лежит выше ручного хода 3 мкм на сетке 1 кГц.
        .replace("max = 0.00000275", "max = 0.000004")
        .replace(
            "values = { base = \"base\", fine = \"fine\" }",
            "values = { base = 'base' }",
        );
    stand.write("campaign.toml", &format!(r#"{campaign}
[depth_factors.factors.mass]
safety = false
source = {{ kind = "measured", reference = "единичный куб из block.toml и куб удвоенной плотности" }}
levels = {{ kind = "axis", parameter = {{ kind = "mass", body = "блок/каретка" }}, unit = "kg", nominal = 6.0, values = {{ base = 6.0, heavy = 12.0 }} }}
"#));
    let device = fs::read_to_string(stand.0.join("block.toml")).unwrap();
    stand.write(
        "block.toml",
        &device.replace(
            "name = \"Блок\"",
            "name = \"Блок\"\nschematic = \"scheme.toml\"",
        ),
    );
    stand.write("control.toml", "version = 1");
    stand.write(
        "electrical.toml",
        r#"
format = 1
step_ns = 500000
source = "сетка 0,5 мс стенда"
control = "control.toml"
[circuits."блок".probes.cap]
kind = "voltage"
ends = ["capacitor/plus", "capacitor/minus"]
"#,
    );

    for (fixture, sign, outcome) in [("unsafe", -1.0, "fail"), ("fixed", 1.0, "pass")] {
        let scheme = root
            .join("devices/electrical/corpus/reverse_polarity")
            .join(format!("{fixture}.toml"));
        fs::copy(scheme, stand.0.join("scheme.toml")).unwrap();
        let report = stand.run(&["--jobs", "1"], 0);
        let assembly = &report["assemblies"]["block"];
        assert_eq!(assembly["coverage"]["executed"], 2);
        assert_eq!(
            assembly["verdict"],
            if sign < 0.0 { "Rejected" } else { "NotProven" }
        );
        for (key, row) in assembly["runs"].as_object().unwrap() {
            let result = &row["detail"]["scenario"];
            let mass = match row["detail"]["coverage"]["selection"]["mass"].as_str() {
                Some("base") => 6.0,
                Some("heavy") => 12.0,
                selection => panic!("неизвестный уровень массы {selection:?}"),
            };
            assert_eq!(result["installation"]["values"]["mass"], mass);
            assert_eq!(result["electrical"]["live"]["required_ns"], 2_000_000);
            assert_eq!(
                result["electrical"]["live"]["seed"],
                row["playback"]["seed"]
            );
            assert!(
                row["findings"].as_array().unwrap().iter().any(|finding| {
                    finding["code"] == "reverse_polarity" && finding["outcome"] == outcome
                }),
                "{result}"
            );
            let archive = stand.0.join("electric.simlog");
            stand.run(&["--repeat", key, "--log", archive.to_str().unwrap()], 0);
            let log = log(&archive);
            fs::remove_file(&archive).unwrap();
            assert_eq!(Some(log.fingerprint()), result["journal"].as_u64());
            let actual = log
                .entries()
                .iter()
                .rev()
                .find(|entry| {
                    entry.kind == sim_runtime::LogKind::Truth
                        && log.header().truth[entry.id as usize].name.ends_with("/cap")
                })
                .unwrap();
            // Начало из DC: идеальная ёмкость разомкнута, I·R = 0.
            // Паспорт reference_cell в catalog/electrical/reference.toml задаёт 5 В.
            let expected = sign * 5.0;
            assert!(
                (actual.value - expected).abs() < 1e-9,
                "{} против {expected}",
                actual.value
            );
        }
    }
    let fault = r#"{ node = "wire:line", kind = "wire_open" }"#;
    let modified = campaign
        .replace("criteria = [", &format!(
            "faults = [{{ fault = {fault}, at_ns = 1000000, max_lag_ns = 0, window = 'whole', source = {{ kind = 'measured', reference = 'разрыв на середине опыта' }} }}]\ncriteria = ["
        ));
    stand.write(
        "campaign.toml",
        &format!(
            r#"{modified}

[depth_factors.factors.interruption]
safety = true
source = {{ kind = "measured", reference = "синтетическая инъекция" }}
levels = {{ kind = "faults", values = {{ nominal = [], open = [{fault}] }} }}
"#
        ),
    );
    let installation = fs::read_to_string(stand.0.join("install.toml")).unwrap();
    stand.write(
        "install.toml",
        &format!("{installation}\nfaults = [{fault}]\n"),
    );
    let results = stand.run(&[], 0);
    let assembly = &results["assemblies"]["block"];
    assert_eq!(assembly["coverage"]["executed"], 1);
    assert_eq!(assembly["coverage"]["open"], 1);
    let missed = assembly["runs"]
        .as_object()
        .unwrap()
        .values()
        .find(|row| row["detail"]["coverage"]["selection"]["interruption"] == "open")
        .unwrap();
    assert_eq!(missed["detail"]["scenario"]["injections_confirmed"], false);
    assert_eq!(missed["outcome"], "not_analyzed");
}

#[test]
fn продолжение_сохраняет_квоту_и_завершённую_строку() {
    let stand = Stand::new("resume", 1, "");
    let checkpoint = stand.0.join("state");
    let args = ["--checkpoint", checkpoint.to_str().unwrap()];
    let a = stand.run(&args, 0);
    let b = stand.run(&args, 0);
    assert_eq!(a["resumed"], 0);
    assert_eq!(b["resumed"], 1);
    assert_eq!(a["assemblies"], b["assemblies"]);
    assert_eq!(b["assemblies"]["block"]["coverage"]["executed"], 1);
    assert_eq!(b["assemblies"]["block"]["coverage"]["open"], 1);
    let manifest = fs::read_to_string(stand.0.join("manifest.toml")).unwrap();
    stand.write("manifest.toml", &manifest.replace("Каретка", "Блок"));
    let error = stand.run(&args, 2);
    assert!(error["detail"].as_str().unwrap().contains("точка бюджета"));
}

#[test]
fn связанный_контур_подтверждает_свою_сетку_зерно_и_горизонт() {
    let stand = Stand::new("linked", 1, "");
    let catalog = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../catalog")
        .canonicalize()
        .unwrap();
    let fixture = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/rotary");
    for entry in fs::read_dir(fixture).unwrap() {
        let entry = entry.unwrap();
        fs::copy(entry.path(), stand.0.join(entry.file_name())).unwrap();
    }
    let task = fs::read_to_string(stand.0.join("task.toml")).unwrap();
    stand.write("task.toml", &task.replace(", comparison_value = 0", ""));
    stand.write(
        "campaign.toml",
        &format!(
            r#"
version = 1
world = "world.toml"
catalog = '{}'
seed = 17 # Зерно синтетического стержня в world.toml.
[stations.rod]
experiment = {{ kind = "control", task = "task.toml" }}
[stations.rod.modes."измерение"]
horizon_ns = 8000000 # Восемь тактов по task.toml.
source = {{ kind = "measured", reference = "стенд стержня" }}
[depth_factors.factors.operation]
safety = true
source = {{ kind = "measured", reference = "режим стенда" }}
levels = {{ kind = "mode", values = {{ turn = {{ station = "rod", mode = "измерение" }} }} }}
[depth_factors.factors.route]
safety = true
source = {{ kind = "measured", reference = "выход регулятора" }}
levels = {{ kind = "path", values = {{ line = "напряжение" }} }}
[depth_factors.factors.clock]
safety = true
source = {{ kind = "measured", reference = "шаг стенда 1 мс" }}
levels = {{ kind = "grid", values = {{ base = "base" }} }}
"#,
            catalog.display()
        ),
    );
    stand.write(
        "install.toml",
        r#"
version = 1
[coverage]
runs = 1
seconds = 30
[[paths]]
station = "rod"
mode = "измерение"
path = "напряжение"
source = { kind = "measured", reference = "оснастка стержня" }
grids = [{ id = "base", hz = 1000, source = { kind = "measured", reference = "шаг стенда 1 мс" } }]
"#,
    );
    let report = stand.run(&[], 0);
    let runs = report["assemblies"]["block"]["runs"].as_object().unwrap();
    let row = runs.values().next().unwrap();
    assert_eq!(row["executed"], 1, "{row}");
    let linked = &row["detail"]["scenario"];
    assert_eq!(linked["children"][0]["execution"]["reached_ns"], 8_000_000);
    assert_eq!(linked["children"][0]["execution"]["hz"], 1000);
    assert_eq!(
        linked["children"][0]["key"],
        row["detail"]["coverage"]["key"]
    );
    let seed = linked["children"][0]["execution"]["seed"].as_u64().unwrap();
    let seed = seed.to_string();
    let direct = Command::new(env!("CARGO_BIN_EXE_sim-run"))
        .arg(stand.0.join("world.toml"))
        .arg("--task")
        .arg(stand.0.join("task.toml"))
        .arg("--catalog")
        .arg(&catalog)
        .args(["--matrix", "full", "--seed", &seed, "--json"])
        .output()
        .unwrap();
    assert!(
        direct.status.success(),
        "{}",
        String::from_utf8_lossy(&direct.stdout)
    );
    let direct: Value = serde_json::from_slice(&direct.stdout).unwrap();
    let expected = &direct["evidence"]["cases"][0]["journal"];
    assert!(expected.is_u64());
    assert_eq!(
        linked["profile"]["evidence"]["cases"][0]["journal"],
        *expected
    );

    stand.write("task.toml", &task);
    let pending = stand.run(&[], 0);
    assert_eq!(pending["assemblies"]["block"]["coverage"]["executed"], 0);
    assert_eq!(pending["assemblies"]["block"]["coverage"]["open"], 1);
}
