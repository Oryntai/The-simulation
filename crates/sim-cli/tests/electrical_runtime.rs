use std::{
    collections::BTreeMap,
    fs,
    path::{Path, PathBuf},
    process::Command,
    time::Instant,
};

use serde::Deserialize;
use sim_cli::electrical::{LiveExperiment, LivePath};
use sim_process::schematic::sim_circuit::netlist;
use sim_runtime::{
    Log, LogKind, Runtime, Status,
    control::{self, EventKind},
    electrical::{CircuitModel, TransientCircuit},
};
use sim_signal::Time;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Fixture {
    world: PathBuf,
    horizon_ns: u64,
    profile: PathBuf,
    phase_ns: u64,
    fixed: Option<PathBuf>,
    drive: Option<Drive>,
    meter: Option<Meter>,
    board: Option<Board>,
}

#[derive(Deserialize)]
struct Drive {
    id: String,
    rail: String,
    minimum_peak_a: f64,
    voltage_of_completed_interval: bool,
    ohm: Option<Ohm>,
}

#[derive(Deserialize)]
struct Ohm {
    emf_v: f64,
    series_ohm: f64,
    rest_a: f64,
    polarization_ohm: f64,
    polarization_probe: String,
    battery_voltage: String,
    battery_current: String,
    common_harness_ohm: f64,
    drive_harness_ohm: f64,
}

#[derive(Deserialize)]
struct Meter {
    voltage: String,
    current: String,
    emf_v: f64,
    sensor_ohm: f64,
    source_ohm: f64,
    load_leads_ohm: f64,
    cap_f: f64,
    drop: [u64; 2],
}

#[derive(Deserialize)]
struct Board {
    vdd: String,
    pin: String,
    key: String,
    command: String,
    falling_v: f64,
    rising_v: f64,
    assert_ns: u64,
    release_ns: u64,
    drop: [u64; 2],
    plateau_v: f64,
    working_v: [f64; 2],
}

fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn cases() -> Vec<(PathBuf, Fixture)> {
    let mut paths = fs::read_dir(root().join("devices"))
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.join("electrical-check.toml").is_file())
        .collect::<Vec<_>>();
    paths.sort();
    // Строка Э3 требует три живые сборки и парный опыт просадки.
    assert_eq!(paths.len(), 3);
    paths
        .into_iter()
        .map(|path| {
            let data = fs::read_to_string(path.join("electrical-check.toml")).unwrap();
            (path, toml::from_str(&data).unwrap())
        })
        .collect()
}

fn load(dir: &Path, case: &Fixture, profile: &Path, path: LivePath) -> LiveExperiment {
    LiveExperiment::load(
        &dir.join(&case.world),
        &dir.join(profile),
        &root().join("catalog"),
        None,
        Time(case.horizon_ns),
        path,
    )
    .unwrap_or_else(|e| panic!("{}: {e}", dir.display()))
}

fn finish(run: &mut LiveExperiment, end: u64) {
    assert_eq!(
        run.run_until(Time(end)).unwrap(),
        Status::Running,
        "{:?}",
        run.runtime.failure()
    );
    assert_eq!(run.runtime.time(), Time(end));
}

fn trace(log: &Log, name: &str, kind: LogKind) -> BTreeMap<u64, (f64, bool)> {
    let table = match kind {
        LogKind::Truth => &log.header().truth,
        LogKind::Observation => &log.header().observations,
        LogKind::Command(_) => &log.header().commands,
    };
    let id = table
        .iter()
        .position(|item| item.name == name)
        .unwrap_or_else(|| panic!("{name}")) as u32;
    let values = log
        .entries()
        .iter()
        .filter(|entry| entry.kind == kind && entry.id == id)
        .map(|entry| {
            assert!(entry.value.is_finite());
            (entry.at.0, (entry.value, entry.valid))
        })
        .collect::<BTreeMap<_, _>>();
    assert!(!values.is_empty(), "{name}");
    values
}

fn value(row: &BTreeMap<u64, (f64, bool)>, time: u64) -> f64 {
    let &(value, valid) = &row[&time];
    assert!(valid);
    value
}

fn approx(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual:e} против {expected:e}, допуск {tolerance:e}"
    );
}

fn check_drive(run: &Runtime, drive: &Drive) {
    let log = run.log();
    let port = |leaf| trace(log, &format!("{}/{leaf}", drive.id), LogKind::Truth);
    let current = port("ток MNA");
    let next = port("next_bus_current_a");
    let drive_voltage = port("bus_voltage_v");
    let rail = trace(log, &drive.rail, LogKind::Truth);
    let charge = port("bus_charge_c");
    let mut peak: f64 = 0.0;
    let mut integrated = 0.0;
    let mut previous = None;
    for (&at, &(next_current, valid)) in &next {
        assert!(valid);
        let current = value(&current, at);
        if let Some((past, expected)) = previous {
            assert_eq!(current.to_bits(), f64::to_bits(expected), "{at}");
            integrated += current * Time(at - past).as_secs();
            let applied = if drive.voltage_of_completed_interval {
                past
            } else {
                at
            };
            assert_eq!(
                value(&drive_voltage, at).to_bits(),
                value(&rail, applied).to_bits()
            );
        }
        previous = Some((at, next_current));
        peak = peak.max(current.abs());
    }
    assert!(peak > drive.minimum_peak_a);
    // Независимая квадратура удержанного тока в нс, допуск на суммирование f64.
    approx(charge.last_key_value().unwrap().1.0, integrated, 1e-10);
    let at = *next.last_key_value().unwrap().0;
    println!(
        "нагрузка: V({at} нс)={:.9} В, I={:.9} А, |I|max={peak:.9} А, Q={integrated:.9} Кл",
        value(&rail, at),
        value(&current, at)
    );
    if let Some(ohm) = &drive.ohm {
        let polarization = trace(log, &ohm.polarization_probe, LogKind::Truth);
        let battery = trace(log, &ohm.battery_voltage, LogKind::Truth);
        let supply = trace(log, &ohm.battery_current, LogKind::Truth);
        let supply_current = value(&supply, at);
        let drive_current = value(&current, at);
        approx(supply_current, drive_current + ohm.rest_a, 1e-9);
        let expected = ohm.emf_v
            - supply_current * ohm.series_ohm
            - value(&polarization, at) * ohm.polarization_ohm;
        approx(value(&battery, at), expected, 1e-9);
        let after_harness = expected
            - supply_current * ohm.common_harness_ohm
            - drive_current * ohm.drive_harness_ohm;
        approx(value(&rail, at), after_harness, 1e-9);
        println!(
            "баланс источника и жгута: {:.3e} и {:.3e} В",
            value(&battery, at) - expected,
            value(&rail, at) - after_harness
        );
    }
}

fn check_meter(run: &Runtime, test: &Meter) {
    let observation = trace(run.log(), &test.voltage, LogKind::Observation);
    let current = trace(run.log(), &test.current, LogKind::Truth);
    // Контрольное RC-решение оставляет только основную нагрузку.
    // Токи сигнальных входов в численном опыте протекают через мегаомные пути.
    let load = test.sensor_ohm + test.load_leads_ohm;
    let dc = test.emf_v * test.sensor_ohm / (load + test.source_ohm);
    let tau = test.cap_f / (test.source_ohm.recip() + load.recip());
    let mut saw_drop = false;
    let mut saw_recovery = false;
    let mut worst: f64 = 0.0;
    for (&at, &(acquired, valid)) in &observation {
        let [drop, release] = test.drop;
        let expected = if at <= drop {
            dc
        } else if at <= release {
            dc * (-Time(at - drop).as_secs() / tau).exp()
        } else {
            let at_release = dc * (-Time(release - drop).as_secs() / tau).exp();
            dc + (at_release - dc) * (-Time(at - release).as_secs() / tau).exp()
        };
        approx(acquired, expected, 0.010);
        approx(value(&current, at), acquired / test.sensor_ohm, 1e-12);
        worst = worst.max((expected - acquired).abs());
        saw_drop |= at > drop && at <= release && !valid;
        saw_recovery |= at > release && valid;
    }
    assert!(saw_drop && saw_recovery && observation[&0].1);
    let minimum = observation.values().map(|x| x.0).reduce(f64::min).unwrap();
    println!(
        "измеритель: {dc:.9} В → {minimum:.9} В → {:.9} В, невязка RC {worst:.3e} В",
        observation.last_key_value().unwrap().1.0
    );
}

fn check_board(rt: &Runtime, board: &Board, fixed: bool) {
    let run = control::run(rt).unwrap();
    let edges = run
        .events()
        .iter()
        .filter_map(|event| match &event.kind {
            EventKind::ElectricalReset { asserted, .. } => Some((event.at.0, *asserted)),
            _ => None,
        })
        .collect::<Vec<_>>();
    let rail = trace(rt.log(), &board.vdd, LogKind::Truth);
    let pin = trace(rt.log(), &board.pin, LogKind::Truth);
    let key = trace(rt.log(), &board.key, LogKind::Truth);
    let min = rail.values().map(|x| x.0).reduce(f64::min).unwrap();
    let live = |after: u64, until: u64| {
        run.events().iter().any(|event| {
            event.at.0 >= after
                && event.at.0 < until
                && matches!(
                    &event.kind, EventKind::CommandApplied { channel, command, .. }
                        if Some(*channel) == rt.channel(&board.command) && command.value > 0.0
                )
        })
    };
    if fixed {
        let [lower, upper] = board.working_v;
        assert!(edges.is_empty() && min > board.rising_v);
        assert!(
            rail.values()
                .all(|&(v, valid)| valid && (lower..=upper).contains(&v))
        );
        assert!(live(board.drop[0], board.drop[1]));
    } else {
        let [(down, true), (up, false)] = edges.as_slice() else {
            panic!("{edges:?}");
        };
        assert!(board.drop[0] < *down && *down < board.drop[1] && *up > board.drop[1]);
        let first_below = rail
            .range(board.drop[0]..)
            .find(|(_, x)| x.0 < board.falling_v)
            .unwrap()
            .0;
        let first_above = rail
            .range(board.drop[1]..)
            .find(|(_, x)| x.0 > board.rising_v)
            .unwrap()
            .0;
        assert_eq!(*down - first_below, board.assert_ns);
        assert_eq!(*up - first_above, board.release_ns);
        assert!(live(0, *down) && !live(*down, *up) && live(*up, rt.time().0));
        // На установившемся отрезке независимо проверяются KVL и отпущенный ключ.
        let midpoint = (board.drop[0] + board.drop[1]) / 2;
        approx(value(&rail, midpoint), board.plateau_v, 0.010);
        // Остаток на выключенном выводе определяется разрядом конденсатора.
        assert!(value(&pin, midpoint).abs() < 0.010 && value(&key, midpoint).abs() < 1e-6);
    }
    assert!(key.values().any(|x| x.0 > 0.1));
    println!("плата: минимум {min:.9} В, фронты {edges:?}");
}

#[test]
fn живые_опыты_по_данным() {
    for (dir, case) in cases() {
        let mut run = load(&dir, &case, &case.profile, LivePath::Full);
        finish(&mut run, case.horizon_ns);
        assert!(
            run.runtime
                .simplifications()
                .iter()
                .any(|item| item.blocks_proof)
        );
        if let Some(drive) = &case.drive {
            check_drive(&run.runtime, drive);
        }
        if let Some(meter) = &case.meter {
            check_meter(&run.runtime, meter);
        }
        if let Some(board) = &case.board {
            check_board(&run.runtime, board, false);
            let mut fixed = load(&dir, &case, case.fixed.as_ref().unwrap(), LivePath::Full);
            finish(&mut fixed, case.horizon_ns);
            check_board(&fixed.runtime, board, true);
        }
    }
}

#[test]
fn снимок_ветвление_и_типизированный_повтор() {
    for (dir, case) in cases() {
        let mut run = load(&dir, &case, &case.profile, LivePath::Full);
        finish(&mut run, case.phase_ns);
        let snap = run.runtime.try_snapshot().unwrap();
        let mut branch = run.runtime.fork().unwrap();
        assert!(branch.verify_snapshot_owner(&snap).is_err());
        finish(&mut run, case.horizon_ns);
        let continuous = control::run(&run.runtime).unwrap();
        let identity = continuous.fingerprint();
        let recording = continuous.recording();
        assert_eq!(
            branch.run_electrical_until(Time(case.horizon_ns)).unwrap(),
            Status::Running
        );
        assert_eq!(control::run(&branch).unwrap().fingerprint(), identity);
        run.runtime.restore(&snap).unwrap();
        finish(&mut run, case.horizon_ns);
        assert_eq!(control::run(&run.runtime).unwrap().fingerprint(), identity);
        let mut replay = LiveExperiment::load_replay(
            &dir.join(&case.world),
            &dir.join(&case.profile),
            &root().join("catalog"),
            None,
            Time(case.horizon_ns),
            recording,
        )
        .unwrap();
        finish(&mut replay, case.horizon_ns);
        control::verify_replay(&replay.runtime).unwrap();
    }
}

#[test]
fn команда_возвращает_диагностику_без_положительного_вердикта() {
    for (dir, case) in cases() {
        let output = Command::new(env!("CARGO_BIN_EXE_sim-run"))
            .arg(dir.join(&case.world))
            .args([
                "--duration",
                &Time(case.horizon_ns).as_secs().to_string(),
                "--electrical",
            ])
            .arg(dir.join(&case.profile))
            .arg("--json")
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stdout)
        );
        let json: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
        assert_eq!(json["status"], "completed");
        assert_eq!(json["proof_status"], "not_analyzed");
        assert_eq!(json["blocks_proof"], true);
        assert_eq!(json["time_ns"], case.horizon_ns);
        let verdict = &json["electrical_verdict"];
        assert_eq!(verdict["simulation"], "NotProven");
        if case.board.is_some() {
            let findings = verdict["findings"].as_array().unwrap();
            assert!(findings.iter().any(|f| f["code"] == "rail_sag"));
            assert!(
                findings
                    .iter()
                    .filter(|f| f["code"] == "rail_sag")
                    .all(|f| f["outcome"] == "not_analyzed")
            );
        }
        assert_eq!(
            json["electrical"]["reset_edges"].as_array().unwrap().len(),
            if case.board.is_some() { 2 } else { 0 }
        );
    }
}

#[test]
fn переход_rlc_и_ошибки_ориентации() {
    // Критическое затухание: L=1 мГн, C=10 мкФ, R=2√(L/C)=20 Ом.
    // Состояние покоя и единичная ступень позволяют сверить решение в замкнутой форме.
    let net = netlist::parse(
        "Блок\nVE src 0 EXT 0\nR1 src p 20\nL1 p bus 1m\nC1 bus 0 10u\nI1 bus 0 EXT 0\n.end\n",
    )
    .unwrap();
    let mut model = TransientCircuit::new(net, Time(500), Vec::new()).unwrap();
    assert!(model.validate_current_source("bus", "0", "I1").is_ok());
    assert!(model.validate_current_source("0", "bus", "I1").is_err());
    assert!(model.validate_current_source("src", "0", "VE").is_err());
    assert!(model.set_source_step("VE", f64::NAN).is_err());
    assert!(model.set_source_step("I1", f64::INFINITY).is_err());
    model.set_source_step("VE", 1.0).unwrap();
    assert_eq!(model.voltage("bus", "0").unwrap(), 0.0);
    model.advance_to(Time(100_000)).unwrap();
    let mut branch = model.clone();
    model.advance_to(Time(200_000)).unwrap();
    // t / √(LC) = 2 в контрольный момент.
    approx(
        model.voltage("bus", "0").unwrap(),
        1.0 - 3.0 * (-2.0_f64).exp(),
        0.001,
    );
    branch.advance_to(Time(200_000)).unwrap();
    assert_eq!(
        model.voltage("bus", "0").unwrap().to_bits(),
        branch.voltage("bus", "0").unwrap().to_bits()
    );
    assert!(branch.advance_to(Time(100_000)).is_err());
    branch.set_source_step("I1", -0.01).unwrap();
    branch.advance_to(Time(201_000)).unwrap();
    assert_eq!(branch.current("I1").unwrap(), -0.01);
}

#[test]
#[ignore = "Внешнее измерение времени на реальных входах. Запускается отдельно в release."]
fn compare_electrical_runtime_cost() {
    const TRIALS: usize = 3; // Нечётная выборка из плана сравнения живых прогонов.
    let mut slow = Vec::new();
    for (dir, case) in cases() {
        for profile in std::iter::once(&case.profile).chain(&case.fixed) {
            let mut legacy = Vec::new();
            let mut full = Vec::new();
            for _ in 0..TRIALS {
                for (path, times) in [(LivePath::Legacy, &mut legacy), (LivePath::Full, &mut full)]
                {
                    let mut run = load(&dir, &case, profile, path);
                    let start = Instant::now();
                    finish(&mut run, case.horizon_ns);
                    times.push(start.elapsed());
                    assert!(run.runtime.steps() > 0);
                    if let LivePath::Full = path {
                        if let Some(drive) = &case.drive {
                            check_drive(&run.runtime, drive);
                        }
                        if let Some(meter) = &case.meter {
                            check_meter(&run.runtime, meter);
                        }
                        if let Some(board) = &case.board {
                            check_board(&run.runtime, board, case.fixed.as_ref() == Some(profile));
                        }
                    }
                }
            }
            legacy.sort();
            full.sort();
            let (base, circuit) = (legacy[TRIALS / 2], full[TRIALS / 2]);
            let ratio = circuit.as_secs_f64() / base.as_secs_f64();
            println!(
                "{} / {}: {base:?} → {circuit:?}, {ratio:.3}×",
                dir.file_name().unwrap().to_string_lossy(),
                profile.display()
            );
            if ratio > 2.0 {
                slow.push((dir.clone(), profile.clone(), ratio));
            }
        }
    }
    assert!(slow.is_empty(), "граница §3.6 ELECTRICAL_SPEC: {slow:?}");
}
