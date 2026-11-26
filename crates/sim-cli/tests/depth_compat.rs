use std::fs;
use std::io::Cursor;
use std::path::{Path, PathBuf};

use serde::Deserialize;
use serde_json::Value;
use sim_cli::electrical::{CatalogInput, LiveExperiment, LivePath, load_catalog, parse_schematic};
use sim_runtime::control::{self, BoardEvent, ControlConfig, EventKind, FileAssets, RunSpec};
use sim_runtime::{Log, LogKind, Runtime, Status};
use sim_signal::Time;
use sim_task::Outcome;
use sim_task::verdict::{REGISTRY_VERSION, SimulationVerdict, VerdictReport};

fn fixtures() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/depth_compat")
}

fn run_cli(path: &Path, args: &[&str]) -> Value {
    let mut report = Vec::new();
    let mut diagnostics = Vec::new();
    let args = ["--json", "--campaign", path.to_str().unwrap()]
        .into_iter()
        .chain(args.iter().copied())
        .map(Into::into);
    let code = sim_cli::run(args, Cursor::new([]), &mut report, &mut diagnostics);
    assert_eq!(
        code,
        0,
        "{}\n{}",
        String::from_utf8_lossy(&report),
        String::from_utf8_lossy(&diagnostics)
    );
    serde_json::from_slice(&report).unwrap()
}

fn near(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual:.12e} против {expected:.12e}, допуск {tolerance:.3e}"
    );
}

#[derive(Deserialize)]
struct ElectricalReference {
    horizon_ns: u64,
    phase_ns: u64,
    meter: Meter,
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

fn channel(log: &Log, kind: LogKind, name: &str) -> u32 {
    let specs = match kind {
        LogKind::Observation => &log.header().observations,
        LogKind::Truth => &log.header().truth,
        LogKind::Command(_) => &log.header().commands,
    };
    specs.iter().position(|spec| spec.name == name).unwrap() as u32
}

#[test]
fn прежний_электрический_профиль_даёт_тот_же_rc_переход_и_повтор() {
    let base = fixtures().join("electric");
    let input = base.join("devices/block");
    let world = input.join("world.toml");
    let profile = input.join("electrical-profile.toml");
    let catalog = base.join("catalog");
    let reference: ElectricalReference =
        toml::from_str(&fs::read_to_string(input.join("electrical-check.toml")).unwrap()).unwrap();
    let horizon = Time(reference.horizon_ns);
    let mut trial =
        LiveExperiment::load(&world, &profile, &catalog, None, horizon, LivePath::Full).unwrap();
    trial
        .runtime
        .audit_snapshot(Time(reference.phase_ns), horizon)
        .unwrap();
    assert_eq!(trial.run_until(horizon).unwrap(), Status::Running);
    assert_eq!(trial.runtime.time(), horizon);

    let log = trial.runtime.log();
    let meter = reference.meter;
    let id = channel(log, LogKind::Observation, &meter.voltage);
    let current = channel(log, LogKind::Truth, &meter.current);
    let rows: Vec<_> = log
        .entries()
        .iter()
        .filter(|row| row.kind == LogKind::Observation && row.id == id)
        .collect();
    let [drop, restore] = meter.drop;
    // Стационарный делитель и аналитическое RC-решение из electrical-check.toml.
    let load = meter.sensor_ohm + meter.load_leads_ohm;
    let steady = meter.emf_v * meter.sensor_ohm / (load + meter.source_ohm);
    let decay = |elapsed| {
        let tau = meter.cap_f / (meter.source_ohm.recip() + load.recip());
        (-Time(elapsed).as_secs() / tau).exp()
    };
    let at_restore = steady * decay(restore - drop);
    for row in &rows {
        let at = row.at.0;
        let expected = if at <= drop {
            steady
        } else if at <= restore {
            steady * decay(at - drop)
        } else {
            steady + (at_restore - steady) * decay(at - restore)
        };
        // ARCHITECTURE §8.3: невязка модели к RC 0,2989 мВ, округляем вверх до 0,3 мВ.
        near(row.value, expected, 0.0003);
        let measured = log
            .entries()
            .iter()
            .find(|item| item.kind == LogKind::Truth && item.id == current && item.at == row.at)
            .unwrap();
        // Допуск 1 пА из приёмки электрического слоя для закона Ома на нагрузке.
        near(measured.value, row.value / meter.sensor_ohm, 1e-12);
    }
    assert!(rows.first().unwrap().valid);
    assert!(
        rows.iter()
            .any(|row| row.at.0 > drop && row.at.0 <= restore && !row.valid)
    );
    assert_eq!(rows.last().unwrap().at, horizon);
    assert!(rows.last().unwrap().valid);

    let min = rows
        .iter()
        .map(|entry| entry.value)
        .reduce(f64::min)
        .unwrap();
    // Приёмка Э3 печатает расчётное DC, минимум и последнее наблюдение.
    // Числа из ARCHITECTURE §8.3 и половина единицы последнего разряда.
    near(steady, 3.299041, 0.5e-6);
    near(min, -4.657e-6, 0.0005e-6);
    near(rows.last().unwrap().value, 3.299034, 0.5e-6);
    println!(
        "Питание: начальный отсчёт {:.6} В, расчётное DC {steady:.6} В, минимум {:.3} мкВ, конец {:.6} В",
        rows[0].value,
        min * 1e6,
        rows.last().unwrap().value
    );

    let mut replay = LiveExperiment::load_replay(
        &world,
        &profile,
        &catalog,
        None,
        horizon,
        control::run(&trial.runtime).unwrap().recording(),
    )
    .unwrap();
    assert_eq!(replay.run_until(horizon).unwrap(), Status::Running);
    control::verify_replay(&replay.runtime).unwrap();
    let mut expected = Vec::new();
    let mut actual = Vec::new();
    log.write_binary(&mut expected).unwrap();
    replay.runtime.log().write_binary(&mut actual).unwrap();
    assert_eq!(actual, expected);
}

#[test]
fn прежний_электрический_манифест_и_опубликованный_лист_сохранили_исход() {
    let base = fixtures().join("electric");
    let saved: Value =
        serde_json::from_slice(&fs::read(base.join("assembly.json")).unwrap()).unwrap();
    let report = run_cli(&base.join("selected.toml"), &[]);
    let assembly = &report["assemblies"]["block"];
    assert_eq!(assembly["simulation"], "NotProven");
    assert_eq!(assembly["live"]["complete"], true);
    let old = saved.as_object().unwrap();
    let new = assembly["assemblies"].as_object().unwrap();
    assert_eq!(old.len(), 1);
    assert_eq!(
        old.keys().collect::<Vec<_>>(),
        new.keys().collect::<Vec<_>>()
    );
    let (instance, old) = old.iter().next().unwrap();
    for field in ["schematic", "proof_scope", "sheet"] {
        assert_eq!(new[instance][field], old[field], "{field}");
    }
    // Опубликованные размеры сборочного листа, ARCHITECTURE §8.3.
    assert_eq!(old["sheet"]["parts"].as_array().unwrap().len(), 21);
    assert_eq!(old["sheet"]["operations"].as_array().unwrap().len(), 36);
    assert!(
        assembly["findings"]
            .as_array()
            .unwrap()
            .iter()
            .any(|f| f["outcome"] == "not_analyzed")
    );
}

#[test]
fn прежние_схема_и_профиль_платы_сохраняют_строгий_разбор() {
    let base = fixtures().join("electric");
    let catalog = load_catalog(&base.join("catalog/electrical"))
        .unwrap()
        .catalog;
    let path = base.join("devices/block/electrical-scheme.toml");
    let source = fs::read_to_string(&path).unwrap();
    let old = parse_schematic(&path, source.as_bytes(), &catalog).unwrap();
    let empty_checks = source.replacen("format = 1", "format = 1\ndepth_checks = []", 1);
    assert_ne!(source, empty_checks);
    let explicit = parse_schematic(&path, empty_checks.as_bytes(), &catalog).unwrap();
    assert_eq!(old.assembly_sheet(), explicit.assembly_sheet());
    for invalid in [
        source.replacen("format = 1", "format = 999", 1),
        source.replacen("format = 1", "format = 1\nнеизвестное_поле = true", 1),
        source.replacen("format = 1", "format = 1\ndepth_checks = false", 1),
    ] {
        assert_ne!(source, invalid);
        assert!(parse_schematic(&path, invalid.as_bytes(), &catalog).is_err());
    }
    let path = base.join("catalog/boards/electrical/stm32f103c8.toml");
    let source = fs::read_to_string(&path).unwrap();
    let board = CatalogInput::parse_board_profile(&path, source.as_bytes()).unwrap();
    // Прежний паспорт платы, нижний порог из falling_v.range в TOML.
    assert_eq!(board.reset.falling_v.range.unwrap().nominal, 1.88);
    let invalid = source.replacen("nominal = 1.88", "nominal = nan", 1);
    assert_ne!(source, invalid);
    assert!(CatalogInput::parse_board_profile(&path, invalid.as_bytes()).is_err());
}

#[test]
fn холодный_запуск_прежних_elf_и_intel_hex_достигает_штатного_останова() {
    let base = fixtures();
    let boards = base.join("board");
    let world = base.join("stage6/corpus/world.toml");
    let world_bytes = fs::read(&world).unwrap();
    let catalog = fs::read(base.join("stage6/corpus/catalog.toml")).unwrap();
    let criteria = fs::read(base.join("stage6/corpus/over-travel-install.toml")).unwrap();
    let horizon = Time(1_000_000); // Один интервал старого стенда загрузки.
    for (wiring, image) in [
        ("elf-control.toml", "esp.elf"),
        ("hex-control.toml", "avr.hex"),
    ] {
        let source = fs::read_to_string(boards.join(wiring)).unwrap();
        let config = ControlConfig::parse(&source).unwrap();
        let mut runtime = Runtime::load(sim_world::load_and_compile(&world).unwrap()).unwrap();
        let spec = || RunSpec {
            world: &world_bytes,
            catalog: &catalog,
            task: source.as_bytes(),
            criteria: &criteria,
            mode: "блок",
            horizon,
            model_ports: &[],
        };
        let assets = FileAssets { base: &boards };
        control::install(&mut runtime, &config, &assets, spec()).unwrap();
        let expected = sim_signal::fnv1a(fs::read(boards.join(image)).unwrap());
        assert_eq!(
            runtime.run_until(horizon).unwrap(),
            Status::ComponentFailure
        );
        let run = control::run(&runtime).unwrap();
        assert_eq!(run.identity().assets, [(image.into(), expected)]);
        // Прежний тест sim-boards/tests/boards.rs проверяет Breakpoint при низком входе.
        assert!(
            run.events().iter().any(|event| matches!(
                &event.kind,
                EventKind::BoardOutput { event, .. }
                    if matches!(&event.output, BoardEvent::Fault(fault)
                        if fault.to_string().starts_with("точка останова по адресу"))
            )),
            "{image}: {:?}",
            run.events()
        );

        let invalid = source.replacen(image, wiring, 1);
        let config = ControlConfig::parse(&invalid).unwrap();
        let mut runtime = Runtime::load(sim_world::load_and_compile(&world).unwrap()).unwrap();
        assert!(control::install(&mut runtime, &config, &assets, spec()).is_err());
        assert!(control::run(&runtime).is_none());
    }
}

#[test]
fn прежние_журналы_читаются_и_пишутся_побайтно() {
    let base = fixtures();
    for (path, process) in [
        ("observation-v1.simlog", false),
        ("process-v2.simlog", true),
    ] {
        let bytes = fs::read(base.join(path)).unwrap();
        let log = Log::read_binary(bytes.as_slice()).unwrap();
        assert_eq!(log.header().observations[0].name, "x");
        assert_eq!(log.header().observations[0].unit, "m");
        assert_eq!(log.entries().len(), 1);
        let sample = &log.entries()[0];
        assert_eq!(sample.kind, LogKind::Observation);
        assert_eq!(sample.value, 1.25);
        assert_eq!(sample.at, Time(1_000_000));
        assert!(sample.valid);
        assert_eq!(log.end().time, sample.at);
        assert_eq!(log.end().steps, 1);
        if process {
            assert_eq!(
                log.process_entries(),
                [(sample.at, "блок: контрольная запись процесса".into())]
            );
        } else {
            assert!(log.process_entries().is_empty());
        }
        let mut output = Vec::new();
        log.write_binary(&mut output).unwrap();
        assert_eq!(output, bytes);

        let mut damaged = bytes.clone();
        damaged[0] ^= 1;
        let error = Log::read_binary(damaged.as_slice()).unwrap_err();
        assert!(error.to_string().contains("контрольная сумма"));

        let update_checksum = |packet: &mut [u8]| {
            let end = packet.len() - 8;
            let checksum = sim_signal::fnv1a(packet[..end].iter().copied());
            packet[end..].copy_from_slice(&checksum.to_le_bytes());
        };
        // Пересчитанная контрольная сумма не делает неизвестную версию допустимой.
        damaged.copy_from_slice(&bytes);
        let version = b"SIMLOG\r\n".len();
        damaged[version..version + 2].copy_from_slice(&u16::MAX.to_le_bytes());
        update_checksum(&mut damaged);
        let error = Log::read_binary(damaged.as_slice()).unwrap_err();
        assert!(error.to_string().contains("неизвестная версия журнала"));

        damaged.copy_from_slice(&bytes);
        let value = sample.value.to_le_bytes();
        let offset = bytes
            .windows(value.len())
            .position(|window| window == value)
            .unwrap();
        damaged[offset..offset + value.len()].copy_from_slice(&f64::NAN.to_le_bytes());
        update_checksum(&mut damaged);
        let error = Log::read_binary(damaged.as_slice()).unwrap_err();
        assert!(error.to_string().contains("неверное значение"));
    }
}

#[test]
fn отчёт_реестра_v2_сохраняет_коды_и_открытые_находки() {
    let source = include_str!("fixtures/depth_compat/verdict-v2.toml");
    let report: VerdictReport = toml::from_str(source).unwrap();
    assert_eq!(report.registry_version, 2);
    assert_eq!(REGISTRY_VERSION, 3);
    assert_eq!(report.simulation, SimulationVerdict::NotProven);
    let codes = [
        "overcurrent",
        "battery",
        "wire_fault",
        "switch_underdrive",
        "fatigue",
        "connection_failure",
    ];
    assert_eq!(report.findings.len(), codes.len());
    for (finding, code) in report.findings.iter().zip(codes) {
        assert_eq!(finding.code, code);
        assert_eq!(finding.key.class.spec().id, code);
        assert_eq!(finding.outcome, Outcome::NotAnalyzed);
        assert!(!finding.confirmed_hazard);
        assert!(finding.depth.is_none());
    }
    let json = serde_json::to_string(&report).unwrap();
    let decoded: VerdictReport = serde_json::from_str(&json).unwrap();
    assert_eq!(serde_json::to_string(&decoded).unwrap(), json);
    let finite: VerdictReport =
        toml::from_str(&format!("{source}\nmeasurement = [0.0, 0.0]")).unwrap();
    assert_eq!(
        finite.findings.last().unwrap().measurement,
        Some([0.0, 0.0])
    );
    assert!(
        toml::from_str::<VerdictReport>(&format!("{source}\nmeasurement = [nan, inf]")).is_err()
    );
}

#[test]
fn прежний_парный_опыт_сохраняет_число_и_не_превращает_нарушение_в_проход() {
    let acceptance: toml::Value = toml::from_str(
        &fs::read_to_string(
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../devices/stage6/acceptance.toml"),
        )
        .unwrap(),
    )
    .unwrap();
    let reference = acceptance["witnesses"]
        .as_array()
        .unwrap()
        .iter()
        .find(|w| w["assembly"].as_str() == Some("over-travel"))
        .unwrap();
    let report = run_cli(&fixtures().join("stage6/selected.toml"), &["--jobs", "1"]);
    assert_eq!(report["version"], 1);
    assert_eq!(report["status"], "completed");
    let bad = &report["assemblies"]["over-travel"];
    let good = &report["assemblies"]["under-travel"];
    assert_eq!(bad["verdict"], "Rejected");
    assert_eq!(good["verdict"], "NotProven");
    fn findings(assembly: &Value) -> impl Iterator<Item = &Value> {
        assembly["runs"]
            .as_object()
            .unwrap()
            .values()
            .flat_map(|row| row["findings"].as_array().unwrap())
            .filter(|finding| finding["code"] == "travel_jam")
    }
    let witness = findings(bad)
        .filter(|finding| {
            finding["outcome"] == "fail"
                && finding["confirmed_hazard"] == true
                && finding["reason"]
                    .as_str()
                    .unwrap()
                    .contains(reference["reason_contains"].as_str().unwrap())
        })
        .max_by(|a, b| {
            a["measurement"][1]
                .as_f64()
                .unwrap()
                .total_cmp(&b["measurement"][1].as_f64().unwrap())
        })
        .unwrap();
    let upper = witness["measurement"][1].as_f64().unwrap();
    let expected = reference["expected"].as_float().unwrap();
    let tolerance = reference["tolerance"].as_float().unwrap();
    near(upper, expected, tolerance);
    // Независимая сумма шагов Euler для F=[2,2,4,4] Н, m=2 кг, dt=1 мс,
    // начальное смещение 1 мкм из over-travel.toml.
    near(upper, (1.0 + 1.0 + 2.0 + 4.0 + 6.0) * 1e-6, tolerance);
    assert!(findings(good).all(|f| f["outcome"] != "fail"));
    let paired = findings(good)
        .filter(|f| f["outcome"] == "pass")
        .map(|f| f["measurement"][1].as_f64().unwrap())
        .reduce(f64::max)
        .unwrap();
    // Постоянное усилие 1 Н в парном опыте, та же масса и смещение.
    near(paired, (1.0 + 0.5 + 1.0 + 1.5 + 2.0) * 1e-6, tolerance);
    println!(
        "Ход каретки: {:.3} / {:.3} мкм, порог 8 мкм, опасный опыт Rejected",
        upper * 1e6,
        paired * 1e6,
    );
}
