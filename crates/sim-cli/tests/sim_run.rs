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
