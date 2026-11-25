use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Command, Output, Stdio};

use serde_json::Value;
use sim_runtime::{Log, LogKind};
use sim_signal::Time;

// Однородный шар массой 1 кг и радиусом 0,1 м: I = 2mr²/5 = 0,004 кг·м².
// Гравитация −10 м/с² выбрана для независимой проверки скорости v = gt.
const WORLD: &str = r#"
format = 3
name = "блок"
seed = 28
physics_hz = 1000
gravity_m_s2 = [0.0, 0.0, -10.0]

[environment]
density_kg_m3 = 0.0
viscosity_pa_s = 0.0
wind_m_s = [0.0, 0.0, 0.0]

[[objects]]
name = "блок"

[objects.mass]
kind = "explicit"
mass_kg = 1.0
com_m = [0.0, 0.0, 0.0]
inertia_kg_m2 = [0.004, 0.004, 0.004, 0.0, 0.0, 0.0]
provenance = { derived = "2mr²/5, однородный шар" }

[[objects.collision]]
shape = { kind = "sphere", radius_m = 0.1 }
"#;

struct Fixture(PathBuf);

impl Fixture {
    fn new(name: &str) -> Self {
        let path = std::env::temp_dir().join(format!("sim-run-test-{}-{name}", std::process::id()));
        fs::create_dir(&path).unwrap();
        let fixture = Self(path);
        fs::write(fixture.0.join("world.toml"), WORLD).unwrap();
        fixture
    }

    fn path(&self, name: &str) -> PathBuf {
        self.0.join(name)
    }

    fn run(&self, world: &str, args: &[&str], input: &[u8]) -> Output {
        let mut child = Command::new(env!("CARGO_BIN_EXE_sim-run"))
            .current_dir(&self.0)
            .arg(world)
            .args(args)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap();
        child.stdin.take().unwrap().write_all(input).unwrap();
        child.wait_with_output().unwrap()
    }
}

impl Drop for Fixture {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

fn json(output: &Output, code: i32) -> Value {
    assert_eq!(
        output.status.code(),
        Some(code),
        "stdout: {}\nstderr: {}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );
    serde_json::from_slice(&output.stdout).unwrap()
}

fn log(path: &Path) -> Log {
    Log::read_binary(fs::File::open(path).unwrap()).unwrap()
}

#[test]
fn command_times_match_in_all_drivers_and_replay() {
    let f = Fixture::new("commands");
    // Каретка массой 1 кг из условного куба, шаг 1 мс. На трёх шагах
    // действуют силы 1, 2, −1 Н, поэтому v по ним: 0.001, 0.003, 0.002 м/с.
    fs::write(
        f.path("world.toml"),
        r#"
format = 3
name = "каретка"
seed = 19
physics_hz = 1000
gravity_m_s2 = [0.0, 0.0, 0.0]
[environment]
density_kg_m3 = 0.0
viscosity_pa_s = 0.0
wind_m_s = [0.0, 0.0, 0.0]
[[devices]]
name = "каретка"
file = "block.toml"
"#,
    )
    .unwrap();
    fs::write(f.path("block.toml"), r#"
format = 3
name = "блок"
[[links]]
name = "блок"
joint = { kind = "slide", axis = [1.0, 0.0, 0.0], armature = 0.0, damping = 0.0, friction_loss = 0.0, efforts = [{ name = 'сила,"ось"', dof = 0 }] }
mass = { kind = "explicit", mass_kg = 1.0, com_m = [0.0, 0.0, 0.0], inertia_kg_m2 = [0.16666666666666666, 0.16666666666666666, 0.16666666666666666, 0.0, 0.0, 0.0], provenance = { derived = "I = ma²/6, куб 1 кг и 1 м" } }
"#).unwrap();
    let channel = r#"каретка/сила,"ось""#;
    let csv_channel = r#""каретка/сила,""ось""""#;
    let csv = |rows: &[(&str, &str)]| {
        let mut text = String::from("time_s,channel,value\r\n");
        for (time, value) in rows {
            text.push_str(&format!("{time},{csv_channel},{value}\r\n"));
        }
        text
    };
    let inputs = csv(&[("0", "1"), ("0.0015", "2"), ("0.002", "-1")]);
    fs::write(f.path("commands.csv"), &inputs).unwrap();
    let mut previous = None;
    for (mode, input) in [
        ("fast", b"".as_slice()),
        ("realtime", b"".as_slice()),
        ("step", b"\n\n\n".as_slice()),
    ] {
        let path = format!("{mode}.bin");
        let output = f.run(
            "world.toml",
            &[
                "--duration",
                "0.003",
                "--commands",
                "commands.csv",
                "--mode",
                mode,
                "--snapshot-at",
                "0.0015",
                "--log",
                &path,
                "--json",
            ],
            input,
        );
        let report = json(&output, 0);
        assert_eq!(report["snapshot"]["time_ns"], 1_500_000);
        assert_eq!(report["channels"]["commands"][channel]["value"], -1.0);
        assert_eq!(
            report["channels"]["commands"][channel]["time_ns"],
            2_000_000
        );
        assert_eq!(report["channels"]["truth"]["qvel[0]"]["value"], 0.002);
        let bytes = fs::read(f.path(&path)).unwrap();
        if let Some(old) = &previous {
            assert_eq!(old, &bytes);
        }
        previous = Some(bytes);
    }
    let replay = f.run(
        "world.toml",
        &[
            "--duration",
            "0.003",
            "--replay",
            "fast.bin",
            "--log",
            "replay.bin",
            "--json",
        ],
        b"",
    );
    assert_eq!(json(&replay, 0)["status"], "completed");
    assert_eq!(fs::read(f.path("replay.bin")).unwrap(), previous.unwrap());

    let invalid = [
        (csv(&[("NaN", "1")]), "time_s"),
        (csv(&[("0", "NaN")]), "value"),
        (csv(&[("0", "inf")]), "value"),
        (csv(&[("0.004", "1")]), "выходит за --duration"),
        (csv(&[("0.002", "1"), ("0.001", "2")]), "по возрастанию"),
        (
            csv(&[("0", "1"), ("0.0000000002", "2")]),
            "той же наносекунде",
        ),
        (
            inputs.replace(csv_channel, "неизвестный"),
            "неизвестный канал",
        ),
        (inputs.replace(csv_channel, channel), "кавычек CSV"),
    ];
    for (contents, detail) in invalid {
        fs::write(f.path("commands.csv"), contents).unwrap();
        let output = f.run(
            "world.toml",
            &[
                "--duration",
                "0.003",
                "--commands",
                "commands.csv",
                "--json",
            ],
            b"",
        );
        let report = json(&output, 2);
        assert!(
            report["detail"].as_str().unwrap().contains(detail),
            "{report}"
        );
        assert_eq!(report["blocks_proof"], true);
    }
    let mixed = f.run(
        "world.toml",
        &[
            "--duration",
            "0.003",
            "--commands",
            "commands.csv",
            "--replay",
            "fast.bin",
            "--json",
        ],
        b"",
    );
    assert!(
        json(&mixed, 2)["detail"]
            .as_str()
            .unwrap()
            .contains("одновременно")
    );
}

#[test]
fn process_runs_all_modes_and_replays_same_binary_log() {
    let f = Fixture::new("modes");
    let fast = f.run(
        "world.toml",
        &[
            "--duration",
            "0.004",
            "--seed",
            "613",
            "--snapshot-at",
            "0.0015",
            "--log",
            "fast.bin",
            "--csv",
            "fast.csv",
            "--json",
        ],
        b"",
    );
    let fast = json(&fast, 0);
    assert_eq!(fast["status"], "completed");
    assert_eq!(fast["time_ns"], 4_000_000);
    assert_eq!(fast["steps"], 4);
    assert_eq!(fast["snapshot"]["time_ns"], 1_500_000);
    assert_eq!(fast["snapshot"]["steps"], 1);
    assert!(fast["simplifications"].is_array());
    let original = log(&f.path("fast.bin"));
    assert_eq!(original.header().seed, 613);
    assert_eq!(
        fast["log_fingerprint"],
        format!("{:016x}", original.fingerprint())
    );
    let z = original
        .header()
        .truth
        .iter()
        .position(|s| s.name == "qvel[2]")
        .unwrap();
    let last_vel_z = original
        .entries()
        .iter()
        .find(|entry| {
            entry.kind == LogKind::Truth && entry.at == Time(4_000_000) && entry.id as usize == z
        })
        .unwrap();
    assert!((last_vel_z.value - (-10.0 * 0.004)).abs() < f64::EPSILON);
    assert!(
        fs::read_to_string(f.path("fast.csv"))
            .unwrap()
            .contains("4000000,truth,,\"qvel[2]\",\"m/s\",true,")
    );

    for (mode, input) in [
        ("realtime", b"".as_slice()),
        ("step", b"\n\n\n\n".as_slice()),
    ] {
        let binary = format!("{mode}.bin");
        let result = f.run(
            "world.toml",
            &[
                "--duration",
                "0.004",
                "--seed",
                "613",
                "--mode",
                mode,
                "--log",
                &binary,
                "--json",
            ],
            input,
        );
        let result = json(&result, 0);
        assert_eq!(result["log_fingerprint"], fast["log_fingerprint"]);
        assert_eq!(
            fs::read(f.path(&binary)).unwrap(),
            fs::read(f.path("fast.bin")).unwrap()
        );
    }
    let replay = f.run(
        "world.toml",
        &[
            "--duration",
            "0.004",
            "--replay",
            "fast.bin",
            "--log",
            "replay.bin",
            "--json",
        ],
        b"",
    );
    assert_eq!(json(&replay, 0)["log_fingerprint"], fast["log_fingerprint"]);
    assert_eq!(
        fs::read(f.path("replay.bin")).unwrap(),
        fs::read(f.path("fast.bin")).unwrap()
    );
}

#[test]
fn process_reports_bad_inputs_and_interrupted_step() {
    let f = Fixture::new("errors");
    let missing = f.run("missing.toml", &["--duration", "0.002", "--json"], b"");
    let missing = json(&missing, 2);
    assert_eq!(missing["status"], "error");
    assert!(missing["detail"].as_str().unwrap().contains("missing.toml"));
    let invalid = f.run("world.toml", &["--duration", "NaN", "--json"], b"");
    assert!(
        json(&invalid, 2)["detail"]
            .as_str()
            .unwrap()
            .contains("--duration")
    );

    fs::write(f.path("nonfinite.toml"), WORLD.replace("-10.0]", "nan]")).unwrap();
    let invalid = f.run("nonfinite.toml", &["--duration", "0.002", "--json"], b"");
    assert!(
        json(&invalid, 2)["detail"]
            .as_str()
            .unwrap()
            .contains("gravity_m_s2")
    );

    fs::write(f.path("bad.bin"), b"broken").unwrap();
    let invalid = f.run(
        "world.toml",
        &["--duration", "0.002", "--replay", "bad.bin", "--json"],
        b"",
    );
    assert!(
        json(&invalid, 2)["detail"]
            .as_str()
            .unwrap()
            .contains("bad.bin")
    );
    let interrupted = f.run(
        "world.toml",
        &["--duration", "0.002", "--mode", "step", "--json"],
        b"\n",
    );
    let interrupted = json(&interrupted, 1);
    assert_eq!(interrupted["status"], "interrupted");
    assert_eq!(interrupted["steps"], 1);
    assert_eq!(interrupted["time_ns"], 1_000_000);

    fs::write(
        f.path("estimate.toml"),
        WORLD.replace("derived =", "estimate ="),
    )
    .unwrap();
    let estimate = f.run("estimate.toml", &["--duration", "0.002", "--json"], b"");
    let estimate = json(&estimate, 0);
    assert!(
        estimate["simplifications"]
            .as_array()
            .unwrap()
            .iter()
            .any(|entry| entry["blocks_proof"] == true)
    );
}

fn rotary(name: &str) -> Fixture {
    let f = Fixture::new(name);
    for (name, text) in [
        ("world.toml", include_str!("fixtures/rotary/world.toml")),
        ("device.toml", include_str!("fixtures/rotary/device.toml")),
        ("drive.toml", include_str!("fixtures/rotary/drive.toml")),
        ("graph.toml", include_str!("fixtures/rotary/graph.toml")),
        ("control.toml", include_str!("fixtures/rotary/control.toml")),
        ("profile.toml", include_str!("fixtures/rotary/profile.toml")),
        ("task.toml", include_str!("fixtures/rotary/task.toml")),
    ] {
        fs::write(f.path(name), text).unwrap();
    }
    f
}

fn rotary_report(f: &Fixture, code: i32) -> Value {
    let output_log = f.path("rotary.bin");
    if output_log.exists() {
        fs::remove_file(output_log).unwrap();
    }
    let catalog = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../catalog");
    let output = f.run(
        "world.toml",
        &[
            "--catalog",
            catalog.to_str().unwrap(),
            "--task",
            "task.toml",
            "--case",
            "измерение",
            "--log",
            "rotary.bin",
            "--json",
        ],
        b"",
    );
    json(&output, code)
}

#[test]
fn полная_матрица_воспроизводится_по_потокам_и_после_частичной_записи() {
    let f = rotary("rotary-matrix");
    let task = fs::read_to_string(f.path("task.toml")).unwrap().replace(
        "[[modes]]",
        r#"[fault_sweep]
at_ns = 4000000
invariant_limits = true
source = { kind = "derived", reference = "После ступени 3 мс с полным окном 4 мс." }

[[modes]]"#,
    );
    fs::write(f.path("task.toml"), &task).unwrap();
    let profile = fs::read_to_string(f.path("profile.toml")).unwrap().replace(
        "required_modes = [\"измерение\"]",
        "required_modes = [\"измерение\", \"отложен\"]",
    );
    fs::write(
        f.path("profile.toml"),
        format!("{profile}\n[deferred_modes]\n'отложен' = 'Нет модели препятствия.'\n"),
    )
    .unwrap();
    let catalog = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../catalog");
    let run = |jobs: &str, checkpoint: &str| {
        f.run(
            "world.toml",
            &[
                "--catalog",
                catalog.to_str().unwrap(),
                "--task",
                "task.toml",
                "--matrix",
                "full",
                "--jobs",
                jobs,
                "--checkpoint",
                checkpoint,
                "--json",
            ],
            b"",
        )
    };
    let one = run("1", "один");
    let report = json(&one, 0);
    let many = run("3", "несколько");
    json(&many, 0);
    assert_eq!(one.stdout, many.stdout);
    let rows = report["evidence"]["cases"].as_array().unwrap();
    // Два режима. На путь приходят регулятор, датчик, провод команды,
    // привод, провод разрешения, питание и штатный опыт.
    assert_eq!(report["coverage"]["required"], 14);
    assert_eq!(report["coverage"]["executed"], 7);
    assert_eq!(report["coverage"]["pass"], 0);
    assert!(rows.iter().any(|row| row["case"]["mode"] == "отложен"
        && row["completed"] == false
        && row["missing"][0] == "Нет модели препятствия."));
    assert!(
        rows.iter()
            .any(|row| row["case"]["fault"]["node"] == "power:стенд/допуск"
                && row["applied_ns"] == 4_000_000
                && row["metrics"][0]["signal"]["name"] == "qpos[0]")
    );

    let directory = fs::read_dir(f.path("несколько"))
        .unwrap()
        .next()
        .unwrap()
        .unwrap()
        .path();
    let mut files: Vec<_> = fs::read_dir(directory)
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .collect();
    files.sort();
    assert_eq!(files.len(), 7);
    let saved = fs::read(&files[0]).unwrap();
    fs::remove_file(&files[0]).unwrap();
    assert_eq!(run("2", "несколько").stdout, one.stdout);
    assert_eq!(fs::read(&files[0]).unwrap(), saved);
    let mut corrupted: Value = serde_json::from_slice(&saved).unwrap();
    corrupted["payload"] = Value::String("{}".into());
    fs::write(&files[0], serde_json::to_vec(&corrupted).unwrap()).unwrap();
    let rejected = json(&run("2", "несколько"), 2);
    assert!(
        rejected["detail"]
            .as_str()
            .unwrap()
            .contains("контрольная сумма")
    );

    fs::write(
        f.path("task.toml"),
        task.replace("at_ns = 4000000", "at_ns = 5000000"),
    )
    .unwrap();
    let rejected = json(&run("2", "новый"), 2);
    assert!(
        rejected["detail"]
            .as_str()
            .unwrap()
            .contains("не помещается")
    );
}

#[test]
fn стержень_с_другой_проводкой_и_регулятором_сходится_с_rl() {
    let f = rotary("rotary-drive");
    let result = rotary_report(&f, 0);
    assert_eq!(result["status"], "completed", "{result}");
    assert_eq!(result["closed_loop"], "not_analyzed");
    let case = &result["evidence"]["cases"][0];
    assert!(case["control"]["model_acceptances"].as_u64().unwrap() > 0);
    assert!(case["control"]["verified_cycles"].as_u64().unwrap() > 0);
    assert!(
        result["topology"]["paths"][0]["nodes"]
            .as_array()
            .unwrap()
            .iter()
            .any(|node| node["id"] == "power:стенд/допуск")
    );
    let data = log(&f.path("rotary.bin"));
    let at = |name: &str, time_ns| {
        let id = data
            .header()
            .truth
            .iter()
            .position(|item| item.name == name)
            .unwrap();
        data.entries()
            .iter()
            .find(|entry| {
                entry.kind == LogKind::Truth
                    && entry.id as usize == id
                    && entry.at == Time(time_ns)
                    && entry.valid
            })
            .unwrap()
            .value
    };
    // Стержень из fixtures/rotary: R=1, τ=L/R=10 мс, такт 1 мс,
    // J=1,5. На старте ω=0 и I=0. Проверка не использует sim-em.
    let angle = std::f64::consts::FRAC_PI_6;
    let voltage = angle * at("стенд/источник/bus_voltage_v", 0);
    let current = voltage * -(-0.1_f64).exp_m1();
    let angle_after_step = angle + current * 1e-6 / 1.5;
    let close = |actual: f64, expected: f64| {
        assert!(
            (actual - expected).abs() < 8.0 * f64::EPSILON,
            "{actual} != {expected}"
        );
    };
    close(at("стенд/источник/voltage_v", 0), voltage);
    close(at("стенд/источник/current_a", 0), current);
    close(at("стенд/источник/joint_torque_nm", 0), current);
    close(at("qpos[0]", 1_000_000), angle_after_step);

    let profile = fs::read_to_string(f.path("profile.toml")).unwrap();
    let reference = r#"
[reference."измерение"]
file = "reference.csv"
time_column = "t"
interval_ns = 1000000
horizon_ns = 8000000
tolerance = 0
columns = [{ reference = "q", signal = "qpos[0]", offset_ns = 0 }]
"#;
    fs::write(f.path("profile.toml"), format!("{profile}{reference}")).unwrap();
    // Заведомо неверный эталон нулевого хода на каждом такте задачи.
    let mut csv = String::from("t,q\n");
    for i in 0..8 {
        csv.push_str(&format!("{},0\n", Time(i * 1_000_000).as_secs()));
    }
    fs::write(f.path("reference.csv"), csv).unwrap();
    let compared = rotary_report(&f, 0);
    assert_eq!(compared["reference"]["измерение"]["passed"], false);
    assert_eq!(log(&f.path("rotary.bin")).fingerprint(), data.fingerprint());
    assert!(
        compared["evidence"]["cases"][0]["missing"]
            .as_array()
            .unwrap()
            .iter()
            .any(|item| item
                .as_str()
                .unwrap()
                .contains("расходится с заявленным эталоном"))
    );
    fs::write(f.path("profile.toml"), &profile).unwrap();

    let control = fs::read_to_string(f.path("control.toml")).unwrap();
    fs::write(f.path("control.toml"), format!("{control}\n[[fault]]\nid='task'\ntarget='power:стенд/допуск'\nat_ns=2000000\nkind='power_off'\nsource='Отключение на втором такте вне задачи.'\n")).unwrap();
    let unplanned = rotary_report(&f, 0);
    let row = &unplanned["evidence"]["cases"][0];
    assert_eq!(
        row["control"]["applied_faults"]["power:стенд/допуск"],
        2_000_000
    );
    assert!(row["missing"].as_array().unwrap().iter().any(|item| {
        item.as_str()
            .unwrap()
            .contains("отдельные диагностические отказы")
    }));
    assert_eq!(row["outcome"], "not_analyzed");

    fs::write(
        f.path("control.toml"),
        control.replace("signal = \"поворот\"", "signal = \"служебный ряд\""),
    )
    .unwrap();
    let readout = r#"
[[readouts]]
signal = { name = "служебный ряд", unit = "rad" }
rate_hz = 1000
source = { kind = "derived", reference = "Координата для оценщика." }
formula = { kind = "position", index = 0 }
"#;
    fs::write(f.path("profile.toml"), format!("{profile}{readout}")).unwrap();
    let denied = rotary_report(&f, 2);
    assert!(
        denied["detail"].as_str().unwrap().contains("служебный ряд"),
        "{denied}"
    );
    fs::write(
        f.path("profile.toml"),
        format!("{profile}{readout}observation = true\n"),
    )
    .unwrap();
    let denied = rotary_report(&f, 2);
    assert!(
        denied["detail"].as_str().unwrap().contains("observation"),
        "{denied}"
    );
}

#[test]
fn attested_series_reaches_campaign_verdict() {
    let fixture = Fixture::new("bench-series");
    let corpus = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../devices/stage6/corpus");
    let existing = fs::read_to_string(corpus.join("under-travel-install.toml")).unwrap();
    let install = existing.replacen("version = 1", r#"
version = 1
measurement_build = { design = "стержень", revision = "r1" }
"#, 1);
    fs::write(fixture.path("install.toml"), format!(r#"{install}

[[inventory.components]]
object = "ход"
kind = "critical_measurement"
source = {{ kind = "datasheet", reference = "Проверка источника и шага ряда в испытании интерфейса." }}

[[checks]]
class = "measurement"
object = "ход"
window = {{ start_ns = 0, end_ns = 4000000, max_gap_ns = 1000000 }}
applicability = {{ kind = "datasheet", reference = "Интерфейсный опыт длительностью 4 мс из профиля." }}
scope = "Опыт допуска ряда с измерителем хода."
inputs = [{{ kind = "datasheet", reference = "Заявленная оснастка опыта и предел хода." }}]
verification = {{ kind = "measurement", limit = {{ kind = "range", signal = {{ name = "ход", unit = "m" }}, max = 0.000012, source = {{ kind = "datasheet", reference = "Предел испытательной оснастки: расчётные 10 мкм и запас 2 мкм." }} }} }}
"#)).unwrap();
    fs::write(fixture.path("campaign.toml"), format!(r#"
version = 1
[assemblies.block]
scope = "Изолированный стержень"
[assemblies.block.experiments.test]
kind = "scenario"
campaign = {}
installation = "install.toml"
"#, toml::Value::String(corpus.join("under-travel.toml").to_str().unwrap().into()))).unwrap();
    fs::write(fixture.path("profile.toml"), r#"
build = { design = "стержень", revision = "r1" }
models = ["движение"]
[[channels]]
name = "ход"
unit = "m"
role = "observation"
sampling = { kind = "periodic", first_ns = 0, period_ns = 1000000 }
"#).unwrap();

    let run = |measurements: bool, status| {
        let mut args = vec!["campaign.toml", "--json"];
        if measurements { args.extend(["--measurements", "records.toml"]); }
        json(&fixture.run("--campaign", &args, b""), status)
    };
    let before = run(false, 0);
    let assembly = &before["assemblies"]["block"];
    assert_eq!(assembly["verdict"], "NotProven", "обязательный замер отсутствует");
    assert_eq!(assembly["readiness"], "NeedsWork");
    let rows = assembly["runs"].as_object().unwrap();
    let (run_key, row) = rows.iter().find(|(_, row)| {
        row["detail"]["summary"]["key"]["sample"]["kind"] == "nominal"
    }).expect("номинальная строка");
    let detail = &row["detail"];
    assert_eq!(detail["measurement_build"]["revision"], "r1");
    let run_table: sim_task::campaign::RunKey = serde_json::from_value(detail["summary"]["key"].clone()).unwrap();
    let run_table = toml::Value::try_from(run_table).unwrap();
    let build = detail["build_fingerprint"].as_str().unwrap();

    let register = |origin, revision, receipt_revision, fingerprint, last| {
        // Только тест протокола. Синтетические координаты равны сумме шагов Euler с a = 1 м/с².
        let raw = format!(
            "sim-reality,1,{origin}\nbuild,стержень,{revision},4000000\ntime_ns,signal,unit,seq,value,valid\n0,ход,m,0,0,1\n1000000,ход,m,1,0.000001,1\n2000000,ход,m,2,0.000003,1\n3000000,ход,m,3,0.000006,1\n4000000,ход,m,4,{last},1\nend,5\n"
        );
        fs::write(fixture.path("bench.csv"), &raw).unwrap();
        let log_fingerprint = format!("{:016x}", sim_signal::fnv1a(raw.bytes()));
        fs::write(fixture.path("records.toml"), format!(r#"
version = 1
[[records]]
experiment = "block/test"
run = {run_table}
build_fingerprint = "{fingerprint}"
signal = {{ name = "ход", unit = "m" }}
# Точные значения тестового регистратора получены рекурсией Euler без шума.
uncertainty = 0.0
profile = "profile.toml"
capture = "bench.csv"
[records.receipt]
build = {{ design = "стержень", revision = "{receipt_revision}" }}
log_fingerprint = "{log_fingerprint}"
capture_id = "Имитация допуска в интеграционном тесте"
calibration = "Условная калибровка шкалы интеграционного теста"
scope = "Изолированная ось на полном окне 4 мс"
"#)).unwrap();
    };
    let finding = |report: &Value| {
        report["assemblies"]["block"]["runs"][run_key]["findings"].as_array().unwrap()
            .iter().find(|f| f["key"]["class"] == "measurement").unwrap().clone()
    };
    assert_eq!(finding(&before)["outcome"], "not_analyzed");
    let install_path = fixture.path("install.toml");
    let original_install = fs::read_to_string(&install_path).unwrap();
    let (without_check, _) = original_install.split_once("[[checks]]\nclass = \"measurement\"").unwrap();
    fs::write(&install_path, without_check).unwrap();
    let unchecked = run(false, 0);
    assert_eq!(unchecked["assemblies"]["block"]["verdict"], "NotProven");
    // Ключ строки включает отпечаток установки и меняется при удалении проверки.
    assert!(unchecked["assemblies"]["block"]["runs"].as_object().unwrap().values().all(|row| {
        row["findings"].as_array().unwrap().iter().any(|f| {
            f["key"]["class"] == "measurement" && f["outcome"] == "not_analyzed"
        })
    }));
    fs::write(&install_path, original_install).unwrap();
    register("recorded", "r1", "r1", build, "0.000010");
    let partial = run(true, 0);
    assert_eq!(partial["assemblies"]["block"]["verdict"], "NotProven");
    let accepted = finding(&partial);
    assert_eq!(accepted["outcome"], "pass");
    assert_eq!(accepted["measurement"], serde_json::json!([0.0, 1e-5]));
    assert!(accepted["measurement_sources"].as_array().unwrap().iter().any(|s| {
        s.as_str().is_some_and(|s| s.contains("Условная калибровка"))
    }));

    let path = fixture.path("records.toml");
    let exact = fs::read_to_string(&path).unwrap();
    // Второй замер привязан к граничному образцу. Его начальный сдвиг 1 мкм
    // даёт ряд 1, 2, 4, 7, 11 мкм по той же рекурсии.
    let boundary_key = rows.values().find_map(|row| {
        let key = &row["detail"]["summary"]["key"];
        (key["sample"]["kind"] == "boundary").then_some(key)
    }).unwrap();
    let boundary_key: sim_task::campaign::RunKey = serde_json::from_value(boundary_key.clone()).unwrap();
    let boundary_log = "sim-reality,1,recorded\nbuild,стержень,r1,4000000\ntime_ns,signal,unit,seq,value,valid\n0,ход,m,0,0.000001,1\n1000000,ход,m,1,0.000002,1\n2000000,ход,m,2,0.000004,1\n3000000,ход,m,3,0.000007,1\n4000000,ход,m,4,0.000011,1\nend,5\n";
    fs::write(fixture.path("boundary.csv"), boundary_log).unwrap();
    let mut both: toml::Value = toml::from_str(&exact).unwrap();
    let records = both["records"].as_array_mut().unwrap();
    let mut boundary = records[0].clone();
    boundary["run"] = toml::Value::try_from(boundary_key).unwrap();
    boundary["capture"] = "boundary.csv".into();
    boundary["receipt"]["capture_id"] = "Граничный образец интеграционного теста".into();
    boundary["receipt"]["log_fingerprint"] = format!("{:016x}", sim_signal::fnv1a(boundary_log.bytes())).into();
    records.push(boundary);
    fs::write(&path, toml::to_string(&both).unwrap()).unwrap();
    let both_accepted = run(true, 0);
    assert_eq!(both_accepted["assemblies"]["block"]["verdict"], "NotProven");
    // Измерены две точки допуска. Непрерывная огибающая остаётся открытой.
    assert_eq!(both_accepted["assemblies"]["block"]["readiness"], "NeedsWork");
    assert_eq!(both_accepted["assemblies"]["block"]["coverage"]["executed"], 2);
    assert!(both_accepted["assemblies"]["block"]["runs"].as_object().unwrap().values().all(|row| {
        row["findings"].as_array().unwrap().iter().any(|f| {
            f["key"]["class"] == "measurement" && f["outcome"] == "pass"
        })
    }));

    // Ошибка 3 мкм даёт верхнюю границу 13 мкм при пределе 12 мкм.
    fs::write(&path, exact.replace("uncertainty = 0.0", "uncertainty = 0.000003")).unwrap();
    let uncertain = run(true, 0);
    let open = finding(&uncertain);
    assert_eq!(open["outcome"], "not_analyzed");
    assert_eq!(uncertain["assemblies"]["block"]["verdict"], "NotProven");
    assert_eq!(uncertain["assemblies"]["block"]["readiness"], "NeedsWork");
    assert!((open["measurement"][1].as_f64().unwrap() - 0.000013).abs() < 1e-15);
    fs::write(&path, exact.replace("uncertainty = 0.0\n", "")).unwrap();
    assert_eq!(finding(&run(true, 0))["outcome"], "not_analyzed");
    for invalid in ["nan", "inf", "-0.000003"] {
        fs::write(&path, exact.replace("uncertainty = 0.0", &format!("uncertainty = {invalid}"))).unwrap();
        assert_eq!(run(true, 2)["proof_status"], "not_analyzed");
    }

    register("recorded", "r1", "r1", build, "0.000015");
    let rejected = run(true, 0);
    assert_eq!(rejected["assemblies"]["block"]["verdict"], "Rejected");
    assert_eq!(finding(&rejected)["event_ns"], 4_000_000);
    for (origin, revision, receipt_revision, fingerprint, reason) in [
        ("synthetic", "r1", "r1", build, "синтетический"),
        ("recorded", "r2", "r1", build, "ревизии"),
        ("recorded", "r1", "r2", build, "ревизии"),
        ("recorded", "r1", "r1", "0000000000000000", "другой сборке"),
    ] {
        register(origin, revision, receipt_revision, fingerprint, "0.000010");
        let result = run(true, 2);
        assert_eq!(result["proof_status"], "not_analyzed");
        assert!(result["detail"].as_str().unwrap().contains(reason), "{result}");
        assert!(result.get("assemblies").is_none());
    }
    let profile_path = fixture.path("profile.toml");
    let profile = fs::read_to_string(&profile_path).unwrap();
    fs::write(&profile_path, profile.replace("r1", "r2")).unwrap();
    register("recorded", "r2", "r2", build, "0.000010");
    let foreign = run(true, 2);
    assert_eq!(foreign["proof_status"], "not_analyzed");
    assert!(foreign["detail"].as_str().unwrap().contains("другой сборке или ревизии"));
    assert!(foreign.get("assemblies").is_none());
    fs::write(&profile_path, profile).unwrap();

    register("recorded", "r1", "r1", build, "0.000010");
    let path = fixture.path("records.toml");
    let record = fs::read_to_string(&path).unwrap();
    let (unattested, _) = record.split_once("\n[records.receipt]").unwrap();
    fs::write(&path, unattested).unwrap();
    let denied = run(true, 2);
    assert_eq!(denied["proof_status"], "not_analyzed");
    assert!(denied.get("assemblies").is_none());

    let capture_path = fixture.path("bench.csv");
    let raw = fs::read_to_string(&capture_path).unwrap();
    let hash = |csv: &str| format!("{:016x}", sim_signal::fnv1a(csv.bytes()));
    // Четыре отсчёта до 3 мс образуют корректный, но слишком короткий замер.
    let short = raw.replace(",r1,4000000\n", ",r1,3000000\n")
        .replace("4000000,ход,m,4,0.000010,1\nend,5", "end,4");
    fs::write(&capture_path, &short).unwrap();
    fs::write(&path, record.replace(&hash(&raw), &hash(&short))).unwrap();
    let denied = run(true, 2);
    assert!(denied["detail"].as_str().unwrap().contains("горизонт замера"));
    assert!(denied.get("assemblies").is_none());

    let incomplete = raw.replace("2000000,ход,m,2,0.000003,1", "2000000,ход,m,2,0,0");
    fs::write(capture_path, &incomplete).unwrap();
    fs::write(path, record.replace(&hash(&raw), &hash(&incomplete))).unwrap();
    let denied = run(true, 2);
    assert_eq!(denied["proof_status"], "not_analyzed");
    assert!(denied["detail"].as_str().unwrap().contains("пропуск"));
    assert!(denied.get("assemblies").is_none());
}

#[test]
fn прочность_принимает_только_заверенный_стенд_своей_ревизии() {
    let fixture = Fixture::new("strength-attestation");
    let corpus = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../devices/stage6/corpus");
    let installation = fs::read_to_string(corpus.join("under-travel-install.toml")).unwrap();
    let installation = installation.replacen("version = 1", r#"
version = 1
measurement_build = { design = "стержень", revision = "r1" }
"#, 1);
    fs::write(fixture.path("install.toml"), format!(r#"{installation}

[[checks]]
class = "static_strength"
object = "каретка/стержень"
window = {{ start_ns = 0, end_ns = 4000000, max_gap_ns = 1000000 }}
scope = "Полное стендовое окно изгиба"
applicability = {{ kind = "datasheet", reference = "Постановка испытания стержня на изгиб." }}
inputs = []
verification = {{ kind = "measurement", limit = {{ kind = "range", signal = {{ name = "напряжение", unit = "Pa" }}, max = 5000000.0, source = {{ kind = "datasheet", reference = "Предел стенда 5 МПа из установки парного опыта." }} }} }}
"#)).unwrap();
    let campaign = toml::Value::String(corpus.join("under-travel.toml").to_str().unwrap().into());
    fs::write(fixture.path("campaign.toml"), format!(r#"
version = 1
[assemblies.block]
scope = "Стержень"
[assemblies.block.experiments.test]
kind = "scenario"
campaign = {campaign}
installation = "install.toml"
"#)).unwrap();
    fs::write(fixture.path("profile.toml"), r#"
build = { design = "стержень", revision = "r1" }
models = ["изгиб"]
[[channels]]
name = "напряжение"
unit = "Pa"
role = "observation"
sampling = { kind = "periodic", first_ns = 0, period_ns = 1000000 }
"#).unwrap();
    let before = json(&fixture.run("--campaign", &["campaign.toml", "--json"], b""), 0);
    let (key, row) = before["assemblies"]["block"]["runs"].as_object().unwrap().iter()
        .find(|(_, row)| row["detail"]["summary"]["key"]["sample"]["kind"] == "nominal").unwrap();
    let strength = |report: &Value| report["assemblies"]["block"]["runs"][key]["findings"]
        .as_array().unwrap().iter().filter(|f| f["key"]["class"] == "static_strength")
        .cloned().collect::<Vec<_>>();
    assert!(!strength(&before).is_empty());
    assert!(strength(&before).iter().all(|f| f["outcome"] == "not_analyzed"));
    let run: sim_task::campaign::RunKey = serde_json::from_value(row["detail"]["summary"]["key"].clone()).unwrap();
    let run = toml::Value::try_from(run).unwrap();
    let build = row["detail"]["build_fingerprint"].as_str().unwrap();
    for (origin, revision, status) in [("recorded", "r1", 0), ("synthetic", "r1", 2), ("recorded", "r2", 2)] {
        let mut capture = format!("sim-reality,1,{origin}\nbuild,стержень,{revision},4000000\ntime_ns,signal,unit,seq,value,valid\n");
        // Ряд протокольного теста. Ручная формула для 2 Н·м и сечения 40×20 мм
        // даёт 6·2/(0,04·0,02²) = 750 кПа.
        for tick in 0..=4 {
            capture.push_str(&format!("{},напряжение,Pa,{tick},750000,1\n", tick * 1_000_000));
        }
        capture.push_str("end,5\n");
        let stamp = sim_signal::fnv1a(capture.bytes());
        fs::write(fixture.path("strength.csv"), &capture).unwrap();
        fs::write(fixture.path("records.toml"), format!(r#"
version = 1
[[records]]
experiment = "block/test"
run = {run}
build_fingerprint = "{build}"
signal = {{ name = "напряжение", unit = "Pa" }}
uncertainty = 0.0
profile = "profile.toml"
capture = "strength.csv"
[records.receipt]
build = {{ design = "стержень", revision = "{revision}" }}
log_fingerprint = "{stamp:016x}"
capture_id = "Проверка переноса записи владельца"
calibration = "Точная шкала протокольного теста"
scope = "Изгиб по всему окну стенда"
"#)).unwrap();
        let report = json(&fixture.run("--campaign",
            &["campaign.toml", "--json", "--measurements", "records.toml"], b""), status);
        if status == 0 {
            let findings = strength(&report);
            assert_eq!(findings.len(), 1);
            assert_eq!(findings[0]["outcome"], "pass");
            assert_eq!(findings[0]["measurement"], serde_json::json!([750000.0, 750000.0]));
        } else {
            assert_eq!(report["proof_status"], "not_analyzed");
            let reason = if origin == "synthetic" { "синтетический" } else { "ревизии" };
            assert!(report["detail"].as_str().unwrap().contains(reason), "{report}");
        }
    }
}

#[test]
fn view_keeps_journal_and_rejects_corruption() {
    let fixture = Fixture::new("view");
    let run = fixture.run("world.toml", &[
        "--duration", "0.1", "--view", "run.simview", "--log", "run.bin", "--json",
    ], b"");
    json(&run, 0);
    let bytes = fs::read(fixture.path("run.simview")).unwrap();
    let view = sim_cli::view::RunView::read(&bytes).unwrap();
    assert_eq!(view.binary, fs::read(fixture.path("run.bin")).unwrap());
    assert_eq!(view.metadata.scene.world_hash(), view.journal.header().world_hash);
    // Свободное падение в сцене без поверхности: нет контактов и события для перехода.
    assert!(view.metadata.peak.is_none());
    assert!(view.metadata.reactions.is_empty());
    assert!(view.frames.is_empty());
    assert!(sim_cli::view::RunView::read(&bytes[..bytes.len() - 1]).is_err());

    let mut corrupted = bytes;
    let index = corrupted.windows("блок".len()).position(|s| s == "блок".as_bytes()).unwrap();
    // Изменяем байт имени модели так, чтобы строка осталась корректным UTF-8.
    corrupted[index + 1] ^= 1;
    assert!(sim_cli::view::RunView::read(&corrupted).is_err());

    let run = fixture.run("world.toml", &[
        "--duration", "0.1", "--view", "missing.simview", "--peak", "отсутствует", "--json",
    ], b"");
    assert_eq!(json(&run, 2)["proof_status"], "not_analyzed");
    assert!(!fixture.path("missing.simview").exists());
}
