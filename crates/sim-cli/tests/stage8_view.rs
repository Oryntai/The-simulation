use std::io::Cursor;
use std::path::{Path, PathBuf};

use serde_json::Value;
use sim_cli::view::{RunView, campaign_bindings};
use sim_runtime::LogKind;

struct Temp(PathBuf);

impl Temp {
    fn new() -> Self {
        for index in 0.. {
            let path =
                std::env::temp_dir().join(format!("sim-stage8-cli-{}-{index}", std::process::id()));
            match std::fs::create_dir(&path) {
                Ok(()) => return Self(path),
                Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
                Err(error) => panic!("{error}"),
            }
        }
        unreachable!()
    }
}

impl Drop for Temp {
    fn drop(&mut self) {
        std::fs::remove_dir_all(&self.0).unwrap();
    }
}

fn campaign(path: &Path, args: &[&str]) -> (i32, String) {
    let mut report = Vec::new();
    let mut diagnostics = Vec::new();
    let args = ["--json", "--campaign", path.to_str().unwrap()]
        .into_iter()
        .chain(args.iter().copied())
        .map(Into::into);
    let code = sim_cli::run(args, Cursor::new([]), &mut report, &mut diagnostics);
    assert!(
        diagnostics.is_empty(),
        "{}",
        String::from_utf8_lossy(&diagnostics)
    );
    (i32::from(code), String::from_utf8(report).unwrap())
}

#[test]
fn пакет_и_журнал_повторяют_ту_же_строку_а_открытая_строка_объясняется() {
    let data = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../devices/stage8/acceptance")
        .canonicalize()
        .unwrap();
    let temp = Temp::new();
    let manifest = data.join("campaign.toml");
    let (code, raw) = campaign(&manifest, &[]);
    assert_eq!(code, 0, "{raw}");
    let report: Value = serde_json::from_str(&raw).unwrap();
    let worst = report["assemblies"]["block"]["worst_run_key"]
        .as_str()
        .unwrap();
    let bindings = campaign_bindings(&raw).unwrap();
    let expected = &bindings[worst];
    let mut tampered = report.clone();
    // Подмена курсора на соседнюю наносекунду не имеет свидетеля оценщика.
    tampered["assemblies"]["block"]["runs"][worst]["playback"]["event"]["at_ns"] =
        serde_json::json!(expected.playback.event.as_ref().unwrap().at_ns - 1);
    assert!(campaign_bindings(&tampered.to_string()).unwrap_err().contains("отсутствует в строке"));

    let archive = temp.0.join("block.simview");
    let journal = temp.0.join("block.simlog");
    let (code, raw) = campaign(
        &manifest,
        &["--repeat", worst, "--view", archive.to_str().unwrap()],
    );
    assert_eq!(code, 0, "{raw}");
    let (code, raw_log) = campaign(
        &manifest,
        &["--repeat", worst, "--log", journal.to_str().unwrap()],
    );
    assert_eq!(code, 0, "{raw_log}");
    assert_eq!(raw, raw_log);
    let bytes = std::fs::read(archive).unwrap();
    let view = RunView::read(&bytes).unwrap();
    let repeated = campaign_bindings(&raw).unwrap();
    let actual = &repeated[worst];
    assert_eq!(view.metadata.campaign.as_ref(), Some(actual));
    assert_eq!(expected.input, actual.input);
    assert_eq!(expected.playback, actual.playback);
    assert_eq!(expected.verdict, actual.verdict);
    assert_eq!(actual.coverage.executed, 1);
    assert_eq!(actual.coverage.omitted, 1);
    assert_eq!(view.binary, std::fs::read(journal).unwrap());
    let limit = 8e-6; // Геометрический зазор из установки.
    let id = view
        .journal
        .header()
        .truth
        .iter()
        .position(|s| s.name == "qpos[0]")
        .unwrap();
    let first = view
        .journal
        .entries()
        .iter()
        .find(|row| {
            row.kind == LogKind::Truth && row.id as usize == id && row.valid && row.value > limit
        })
        .unwrap();
    let event = expected.playback.event.as_ref().unwrap();
    assert!(event.violation);
    assert_eq!(event.at_ns, first.at.0);
    // Сумма четырёх шагов Euler из задачи и граничного смещения на 1 мкм.
    assert!((first.value - (1.0 + 1.0 + 2.0 + 4.0 + 6.0) * 1e-6).abs() < 1e-14);
    let mut broken = bytes;
    broken[24] ^= 1; // Первый байт заголовка после magic, размера и суммы.
    assert!(RunView::read(&broken).is_err());
    assert_eq!(
        campaign(
            &manifest,
            &[
                "--repeat",
                "block/чужой",
                "--view",
                temp.0.join("other").to_str().unwrap()
            ]
        )
        .0,
        2
    );

    // Установка запрашивает отсутствующий режим. План должен объяснить, почему журнал не создан.
    let mut installation = std::fs::read_to_string(data.join("installation.toml")).unwrap();
    let start = installation.find("[[paths]]").unwrap();
    let end = installation.find("[[checks]]").unwrap();
    let unavailable_path =
        installation[start..end].replace("mode = \"loaded\"", "mode = \"missing\"");
    installation.push_str(&unavailable_path);
    std::fs::write(temp.0.join("open.toml"), installation).unwrap();
    let manifest = temp.0.join("campaign.toml");
    std::fs::write(&manifest, format!(
        "version = 1\n[assemblies.block]\nscope = 'Синтетический блок'\n[assemblies.block.experiments.open]\nkind = 'scenario'\ncampaign = {:?}\ninstallation = 'open.toml'\n",
        data.join("task.toml").to_str().unwrap(),
    )).unwrap();
    let (code, raw) = campaign(&manifest, &[]);
    assert_eq!(code, 0, "{raw}");
    let report: Value = serde_json::from_str(&raw).unwrap();
    let open = report["assemblies"]["block"]["runs"]
        .as_object()
        .unwrap()
        .iter()
        .find(|(_, row)| row["status"] == "open")
        .unwrap()
        .0;
    let absent = temp.0.join("absent.simview");
    let (code, raw) = campaign(
        &manifest,
        &["--repeat", open, "--view", absent.to_str().unwrap()],
    );
    assert_eq!(code, 0, "{raw}");
    let report: Value = serde_json::from_str(&raw).unwrap();
    assert_eq!(report["assemblies"]["block"]["verdict"], "NotProven");
    assert!(
        report["view_unavailable"]
            .as_str()
            .unwrap()
            .contains("отсутствует обязательный режим")
    );
    assert!(!absent.exists());
}

#[test]
fn составная_строка_выбирает_сетку_и_временную_метку_из_опыта() {
    let manifest = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../devices/stage8/acceptance/rod-campaign.toml");
    let (code, raw) = campaign(&manifest, &[]);
    assert_eq!(code, 0, "{raw}");
    let alias = manifest
        .parent()
        .unwrap()
        .join(".")
        .join(manifest.file_name().unwrap());
    let (alias_code, alias_report) = campaign(&alias, &[]);
    assert_eq!(alias_code, 0, "{alias_report}");
    assert_eq!(raw, alias_report);
    let bindings = campaign_bindings(&raw).unwrap();
    let binding = &bindings["rod/travel"];
    let p = &binding.playback;
    assert_eq!(p.child.mode, "длинный");
    // Базовые 1 кГц и два вложенных деления шага из плана сходимости.
    assert_eq!(p.physics_hz, 4000);
    assert!(p.event.is_none(), "диагностический максимум не является событием: {:?}", p.event);
    let temp = Temp::new();
    let out = temp.0.join("rod.simview");
    let (code, raw) = campaign(
        &manifest,
        &["--repeat", &binding.key, "--view", out.to_str().unwrap()],
    );
    assert_eq!(code, 0, "{raw}");
    let view = RunView::read(&std::fs::read(out).unwrap()).unwrap();
    assert_eq!(view.metadata.campaign.as_ref(), Some(binding));
    let qpos = view
        .journal
        .header()
        .truth
        .iter()
        .position(|s| s.name == "qpos[0]")
        .unwrap();
    let at = view
        .journal
        .entries()
        .iter()
        .find(|e| e.kind == LogKind::Truth && e.id as usize == qpos && e.at.0 == 50_000_000)
        .unwrap();
    // 200 шагов по 250 мкс под заданным ускорением 10 м/с².
    let expected = 10.0 * 0.00025_f64.powi(2) * 200.0 * 201.0 / 2.0;
    assert!((at.value - expected).abs() < 1e-13);
}

fn scenario(dir: &Path, world: &str, run: &str, checks: &str) -> Value {
    let fixtures = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../devices/stage6/corpus");
    std::fs::copy(fixtures.join("catalog.toml"), dir.join("catalog.toml")).unwrap();
    for (name, content) in [
        ("world.toml", world),
        ("task.toml", run),
        ("installation.toml", checks),
        ("manifest.toml", r#"
version = 1
[assemblies.stand]
scope = "Нейтральный стенд"
[assemblies.stand.experiments.motion]
kind = "scenario"
campaign = "task.toml"
installation = "installation.toml"
"#),
    ] {
        std::fs::write(dir.join(name), content).unwrap();
    }
    let (code, raw) = campaign(&dir.join("manifest.toml"), &[]);
    assert_eq!(code, 0, "{raw}");
    serde_json::from_str::<Value>(&raw).unwrap()["assemblies"]["stand"].clone()
}

fn rod_fixture(file: &str) -> String {
    std::fs::read_to_string(
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../devices/stage6/corpus").join(file),
    ).unwrap()
}

fn findings(report: &Value) -> impl Iterator<Item = &Value> {
    report["runs"].as_object().unwrap().values()
        .flat_map(|run| run["findings"].as_array().unwrap())
}

#[test]
fn границы_угла_не_доказывают_огибающую_маятника() {
    let dir = Temp::new();
    std::fs::write(dir.0.join("rod.toml"), r#"
format = 3
name = "Маятник"
[[links]]
name = "стержень"
[links.joint]
kind = "hinge"
axis = [0.0, 1.0, 0.0]
damping = 0.0
friction_loss = 0.0
armature = 0.0
[links.mass]
kind = "explicit"
mass_kg = 2.0
com_m = [0.0, 0.0, -1.0]
# I=2mr²/5 для груза радиусом 1 см на плече 1 м.
inertia_kg_m2 = [0.00008, 0.00008, 0.00008, 0.0, 0.0, 0.0]
provenance = { datasheet = "Численный маятник 2 кг, 1 м и груз радиусом 1 см." }
[[links.collision]]
shape = { kind = "box", size_m = [1.0, 0.02, 0.04] }
contact_mask = [0, 0]
friction = [0.0, 0.0, 0.0]
"#).unwrap();
    let world = rod_fixture("world.toml").replace("gravity_m_s2 = [0.0, 0.0, 0.0]", "gravity_m_s2 = [0.0, 0.0, -9.81]");
    let extent = std::f64::consts::PI;
    let midpoint = std::f64::consts::FRAC_PI_2;
    let task = |discrete: &str, interior: &str| format!(r#"
version = 1
world = "world.toml"
catalog = "catalog.toml"
seed = 7 # Номер серии численного стенда.
[sampling]
{discrete}
axes = [{{ id = "угол", parameter = {{ kind = "joint_position", joint = "каретка/стержень" }}, scope = "run", unit = "rad", nominal = 0.0, min = 0.0, max = {extent}, source = {{ kind = "datasheet", reference = "Угол подвеса от 0 до π." }} }}]
boundaries = [
  {{ id = "граница", values = {{ "угол" = {extent} }}, source = {{ kind = "datasheet", reference = "Верхнее положение маятника." }} }},
  {interior}
]
[stations.stand]
experiment = {{ kind = "scenario" }}
[stations.stand.modes.loaded]
horizon_ns = 495000000 # Независимая сверка первых 495 шагов по 1 мс.
source = {{ kind = "datasheet", reference = "Свободный маятник с контролем скорости." }}
criteria = [{{ id = "скорость", window = "whole", limit = {{ kind = "range", signal = {{ name = "qvel[0]", unit = "rad/s" }}, min = -0.5, max = 0.5, source = {{ kind = "datasheet", reference = "Граница скорости синтетической постановки: 0,5 рад/с." }} }} }}]
[stations.stand.modes.loaded.windows.whole]
start_ns = 0
end_ns = 495000000
max_gap_ns = 1000000
"#);
    let installation = rod_fixture("under-travel-install.toml");
    let prefix = &installation[..installation.find("[[checks]]").unwrap()];
    let coverage = &installation[installation.find("[inventory]").unwrap()..];
    // Старая версия закрывала независимую прочность одним заявленным числом.
    let section = &installation[installation.find("class = \"static_strength\"").unwrap()..installation.find("[inventory]").unwrap()];
    let checks = format!("{prefix}[[checks]]\n{section}{coverage}")
        .replace("end_ns = 4000000", "end_ns = 495000000");
    for declaration in ["", r#"discrete = { kind = "datasheet", reference = "Допуск угла от 0 до π" }"#] {
        let report = scenario(&dir.0, &world, &task(declaration, ""), &checks);
        assert_eq!(report["verdict"], "NotProven");
        assert!(findings(&report).any(|f| f["key"]["object"] == "допуск/угол"
            && f["key"]["level"] == "simulation"));
        let criteria = findings(&report).filter(|f| f["key"]["object"] == "скорость").collect::<Vec<_>>();
        assert_eq!(criteria.len(), 2);
        assert!(criteria.iter().all(|f| f["outcome"] == "pass"
            && f["measurement"][0].as_f64().unwrap().abs() < 1e-12
            && f["measurement"][1].as_f64().unwrap().abs() < 1e-12));
    }
    let interior = format!(r#"{{ id = "внутри", values = {{ "угол" = {midpoint} }}, source = {{ kind = "datasheet", reference = "Средний угол диапазона." }} }}"#);
    let report = scenario(&dir.0, &world, &task("", &interior), &checks);
    assert_eq!(report["verdict"], "Rejected");
    let fail = findings(&report).find(|f| f["key"]["object"] == "скорость" && f["outcome"] == "fail").unwrap();
    // Независимая рекурсия Euler: I·dω/dt = −mgL·sin(θ), I=2,00008 кг·м².
    let (mut angle, mut speed) = (midpoint, 0.0);
    for _ in 0..495 {
        speed -= (2.0 * 9.81 / 2.00008) * angle.sin() * 0.001;
        angle += speed * 0.001;
    }
    assert!((speed.abs() - 4.23).abs() < 0.01);
    assert!((fail["measurement"][0].as_f64().unwrap() - speed).abs() < 1e-10);
}

#[test]
fn разрешённая_установка_не_скрывает_передачу_из_мира() {
    let dir = Temp::new();
    let mut rod = rod_fixture("rod.toml");
    rod.push_str(r#"
[[links]]
name = "винт"
parent = "стержень"
[links.joint]
kind = "hinge"
axis = [1.0, 0.0, 0.0]
damping = 0.0
friction_loss = 0.0
armature = 0.0
[links.mass]
kind = "explicit"
mass_kg = 2.0
com_m = [0.0, 0.0, 0.0]
inertia_kg_m2 = [0.01, 0.02, 0.025, 0.0, 0.0, 0.0]
provenance = { datasheet = "Синтетическое звено 2 кг, инерция из стендовой постановки." }
[[transmissions]]
kind = "screw"
name = "передача"
input = "винт"
output = "стержень"
# Численный стенд передачи с ходом 1 см за оборот.
lead_m_per_turn = 0.01
"#);
    std::fs::write(dir.0.join("rod.toml"), rod).unwrap();
    let report = scenario(&dir.0, &rod_fixture("world.toml"),
        &rod_fixture("under-travel.toml"), &rod_fixture("under-travel-install.toml"));
    let missing: Vec<_> = findings(&report)
        .filter(|f| f["key"]["object"] == "каретка/передача").collect();
    for class in ["thread_failure", "connection_failure"] {
        assert!(missing.iter().any(|f| f["key"]["class"] == class
            && f["outcome"] == "not_analyzed"), "{report}");
    }
}

#[test]
fn каретка_проходит_сквозь_отключённый_упор_без_положительного_вердикта() {
    let dir = Temp::new();
    std::fs::write(dir.0.join("rod.toml"), rod_fixture("rod.toml")).unwrap();
    let world = rod_fixture("world.toml").replace("[[devices]]", r#"
[[environment.collision]]
# Упор того же размера стоит в 2 мкм от границы метрового стержня.
shape = { kind = "box", size_m = [1.0, 0.02, 0.04] }
pose = { position_m = [1.000002, 0.0, 0.0] }
contact_mask = [0, 0]
friction = [0.0, 0.0, 0.0]

[[devices]]
"#);
    let task = rod_fixture("under-travel.toml").replace("[sampling]", r#"[sampling]
discrete = { source = { kind = "datasheet", reference = "Две посадки жёсткой проставки: без сдвига или со сдвигом 1 мкм." }, boundary_ids = ["смещение"] }"#);
    let report = scenario(&dir.0, &world, &task, &rod_fixture("under-travel-install.toml"));
    assert_eq!(report["verdict"], "NotProven");
    assert!(findings(&report).any(|f| f["coverage_gaps"].as_array().unwrap().iter().any(|gap|
        gap.as_str().is_some_and(|gap| gap.contains("contact: Маски отключают контакт")))));
    // Ручной ход F/m·h²·(1+2+3+4) плюс 0/1 мкм. Обе пробы пересекают упор.
    let mut end: Vec<_> = report["runs"].as_object().unwrap().values().map(|run|
        run["detail"]["summary"]["metrics"]["конечный ход"]["value"].as_f64().unwrap()
    ).collect();
    end.sort_by(f64::total_cmp);
    assert_eq!(end.len(), 2);
    for (actual, expected) in end.into_iter().zip([5e-6, 6e-6]) {
        assert!((actual - expected).abs() < 1e-14 && actual > 2e-6);
    }
}
