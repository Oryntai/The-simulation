use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use sim_cli::electrical::{CatalogInput, mcu_input};
use sim_process::Analysis;
use sim_process::schematic::{Status as FindingStatus, mcu_energy::McuCurrentKind};
use sim_runtime::control::external_board::{Stop, UartProbe};
use sim_runtime::control::{self, ControlAssets, ControlConfig, FileAssets, RunSpec};
use sim_runtime::{Runtime, Status};
use sim_signal::{Rate, Time, fnv1a};
use sim_world::CompiledWorld;

const POWER: &str = include_str!("../../../catalog/boards/electrical/esp32-c3-power.toml");
const SCHEDULE: &str = r#"
format = 1
repeat = true
source = "График стенда: 1 мс работы, 2 мс глубокого сна"
[[states]]
id = "modem_sleep_80_mhz_running_clocks_off"
duration_ns = 1000000
[[states]]
id = "deep_sleep"
duration_ns = 2000000
"#;
const EXTERNAL_WIRING: &str = r#"
version = 1
[[external_board]]
name = "блок"
catalog = "esp32-s3-n16r8"
profile = "external.toml"
source = "Диагностика UART0"
"#;
const EXTERNAL: &str = r#"
format = 1
board = "esp32-s3-n16r8"
machine = "esp32s3"
qemu = "/несуществующий/qemu-system-xtensa"
boot = "uart_download"
clock = "icount"
# QEMU -icount: 2^3 виртуальных наносекунд на инструкцию.
icount_shift = 3
seed = 0
# Бюджеты диагностического стенда. Darwin включает область dyld в RLIMIT_AS.
wall_time_ms = 5000
address_space_bytes = 515396075520
uart_bytes = 16384
stderr_bytes = 4096
required_blocks = ["uart", "gpio_strap"]
required_ports = ["uart0"]
source = "QEMU Espressif Supported features, ESP32-S3"
"#;

#[derive(Default)]
struct Assets(BTreeMap<&'static str, Vec<u8>>);

impl Assets {
    fn put(&mut self, name: &'static str, bytes: impl AsRef<[u8]>) {
        self.0.insert(name, bytes.as_ref().to_vec());
    }
}

impl ControlAssets for Assets {
    fn read(&self, name: &str) -> Result<Vec<u8>, String> {
        self.0
            .get(name)
            .cloned()
            .ok_or_else(|| format!("нет ресурса «{name}»"))
    }

    fn executable(&self, _: &str) -> Result<PathBuf, String> {
        panic!("штатный установщик не должен запускать диагностику QEMU")
    }
}

fn runtime() -> Runtime {
    Runtime::load(CompiledWorld {
        name: "блок".into(),
        seed: 0,
        physics: Rate::hz(1000).unwrap(),
        // Шаг стенда 1 мс, масса блока 1 кг. Механика здесь нужна для расписания.
        mjcf: r#"<mujoco><option timestep="0.001"/><worldbody><body name="block">
          <freejoint/><geom type="box" size="0.1 0.1 0.1" mass="1"/>
          </body></worldbody></mujoco>"#
            .into(),
        bodies: vec![],
        joints: vec![],
        efforts: vec![],
        drives: vec![],
        sensors: vec![],
        simplifications: vec![],
    })
    .unwrap()
}

fn run_spec(horizon: Time) -> RunSpec<'static> {
    RunSpec {
        world: b"block",
        catalog: b"catalog",
        task: b"task",
        criteria: b"criteria",
        mode: "блок",
        horizon,
        model_ports: &[],
    }
}

fn wiring(catalog: &str, fields: &str) -> ControlConfig {
    ControlConfig::parse(&format!(
        r#"version = 1
[[board]]
name = "блок"
catalog = "{catalog}"
image = "image"
rate_hz = 1000
source = "Плата опрашивается через 1 мс"
{fields}
"#
    ))
    .unwrap()
}

fn refuses(config: &ControlConfig, assets: &Assets, reason: &str) {
    let mut rt = runtime();
    let spec = run_spec(Time(1_000_000)); // Интервал стенда 1 мс.
    let error = control::install(&mut rt, config, assets, spec).unwrap_err();
    assert!(error.contains(reason), "{error}");
    assert!(control::run(&rt).is_none());
    assert!(rt.simplifications().is_empty());
}

#[test]
fn старый_каталог_сброса_и_строгий_каталог_потребления() {
    let folder = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../catalog/boards/electrical");
    let old = CatalogInput::load_board_profiles(&folder).unwrap();
    let (_, path, bytes) = &old["stm32f103c8"];
    assert_eq!(
        CatalogInput::parse_board_profile(path, bytes)
            .unwrap()
            .reset
            .falling_v
            .range
            .unwrap()
            .nominal,
        1.88
    );
    let power = folder.join("esp32-c3-power.toml");
    let profile = mcu_input::load_mcu_power_profile(&power).unwrap();
    assert_eq!(profile.board(), "esp32-c3");
    // ESP32-C3 Datasheet v2.5, §5.6, таблицы 5-7, 5-8 и 5-9. Независимо от TOML.
    for (id, amperes, kind) in [
        ("wifi_tx_b_1_mbps_21_dbm", 335e-3, McuCurrentKind::Peak),
        (
            "modem_sleep_80_mhz_running_clocks_off",
            17e-3,
            McuCurrentKind::Typical,
        ),
        ("deep_sleep", 5e-6, McuCurrentKind::Typical),
    ] {
        let Analysis::Analyzed { value, .. } = profile.current(id) else {
            panic!("нет точки {id}");
        };
        assert_eq!((value.current_a, value.kind), (amperes, kind));
    }
    for bad in [
        POWER.replace("0.017", "nan"),
        POWER.replace("0.017", "1e999"),
        POWER.replace("0.017", "-0.017"),
        POWER.replacen("typical", "guaranteed", 1),
        POWER.replacen("section =", "лишнее =", 1),
        POWER.replacen("format = 1", "format = 2", 1),
        format!(
            "{POWER}\n{}",
            POWER
                .split_once("[[states]]")
                .unwrap()
                .1
                .split("[[states]]")
                .next()
                .map(|row| format!("[[states]]{row}"))
                .unwrap()
        ),
    ] {
        assert!(
            mcu_input::parse_mcu_power_profile(&power, bad.as_bytes()).is_err(),
            "{bad}"
        );
    }
    assert!(mcu_input::parse_mcu_power_profile(path, bytes).is_err());
}

#[test]
fn установщик_связывает_каталог_интеграл_и_байты_снимка() {
    let mut assets = Assets::default();
    assets.put(
        "image",
        include_bytes!("../../sim-boards/testdata/esp_uart.elf"),
    );
    assets.put("mcu-power.toml", POWER);
    assets.put("states.toml", SCHEDULE);
    assets.put(
        "boards.toml",
        r#"[[board]]
id = "bench-board"
profile = "esp32-c3"
name = "Плата стенда"
source = "ESP32-C3 Datasheet v2.5"
"#,
    );
    let config = wiring(
        "bench-board",
        "catalog_asset='boards.toml'\npower_profile='mcu-power.toml'\npower_schedule='states.toml'",
    );
    // Два периода по 3 мс и хвост 1 мс. Сохранение заряда Q = Σ IΔt.
    let horizon = Time(7_000_000);
    let report = config.mcu_energy(&assets, "блок", horizon).unwrap();
    let Analysis::Analyzed { value, .. } = report.chip_charge else {
        panic!("нет интеграла заряда");
    };
    let expected = 17e-3 * 3e-3 + 5e-6 * 4e-3;
    assert!((value.charge_c - expected).abs() <= f64::EPSILON * expected);
    assert_eq!(value.window.end, horizon);
    assert!(!value.includes_peak_held);
    assert_eq!(report.sleep_drain.status, FindingStatus::NotAnalyzed);
    let mut rt = runtime();
    control::install(&mut rt, &config, &assets, run_spec(horizon)).unwrap();
    let run = control::run(&rt).unwrap();
    assert!(
        run.simplifications()
            .iter()
            .any(|s| s.blocks_proof && s.text.contains("Кл за 7000000 нс"))
    );
    let captured: BTreeMap<_, _> = run.identity().assets.iter().cloned().collect();
    assert_eq!(
        config.assets().unwrap().into_iter().collect::<Vec<_>>(),
        captured.keys().cloned().collect::<Vec<_>>()
    );
    for (name, bytes) in &assets.0 {
        assert_eq!(captured[*name], fnv1a(bytes.iter().copied()));
    }
    rt.audit_snapshot(Time(2_000_000), horizon).unwrap();
    let recording = control::run(&rt).unwrap().recording();
    assets.put(
        "states.toml",
        format!("{SCHEDULE}\n# Другие байты источника"),
    );
    let error = control::install_replay(
        &mut runtime(),
        &config,
        &assets,
        run_spec(horizon),
        recording,
    )
    .unwrap_err();
    assert!(error.contains("отличаются от записи"), "{error}");
    assets.put("states.toml", SCHEDULE.replace("deep_sleep", "unknown"));
    refuses(&config, &assets, "unknown");
    assets.put("states.toml", SCHEDULE);
    assets.put("mcu-power.toml", POWER.replace("esp32-c3\"", "rp2040\""));
    refuses(&config, &assets, "несовместим");
}

#[test]
fn отсутствующий_или_короткий_график_оставляет_заряд_открытым() {
    let mut assets = Assets::default();
    assets.put("mcu-power.toml", POWER);
    assets.put(
        "states.toml",
        SCHEDULE.replace("repeat = true", "repeat = false"),
    );
    for field in ["", "\npower_schedule='states.toml'"] {
        let config = wiring(
            "esp32-c3",
            &format!("power_profile='mcu-power.toml'{field}"),
        );
        // 4 мс больше одиночного графика на 3 мс.
        let report = config.mcu_energy(&assets, "блок", Time(4_000_000)).unwrap();
        assert!(matches!(report.chip_charge, Analysis::NotAnalyzed { .. }));
        assert_eq!(report.sleep_drain.status, FindingStatus::NotAnalyzed);
    }
}

#[test]
fn штатный_установщик_принимает_старые_elf_и_hex() {
    for (id, bytes) in [
        (
            "esp32-c3",
            &include_bytes!("../../sim-boards/testdata/esp.elf")[..],
        ),
        (
            "atmega328p",
            &include_bytes!("../../sim-boards/testdata/avr.hex")[..],
        ),
    ] {
        let mut assets = Assets::default();
        assets.put("image", bytes);
        let config = wiring(id, "");
        let mut rt = runtime();
        control::install(&mut rt, &config, &assets, run_spec(Time(1_000_000))).unwrap();
        assert_eq!(
            control::run(&rt).unwrap().identity().assets,
            [("image".into(), fnv1a(bytes.iter().copied()))]
        );
        assert_eq!(
            rt.run_until(Time(1_000_000)).unwrap(),
            Status::ComponentFailure
        );
        let events = format!("{:?}", control::run(&rt).unwrap().events());
        assert!(events.contains("Breakpoint"), "{events}");
    }
}

#[test]
fn старая_запись_simlog_v1_читается_побайтно() {
    // Прежняя раскладка SIMLOG v1 из sim-runtime/src/log.rs. Фиксированный
    // образ содержит один отсчёт координаты 1,25 м на 1 мс и контрольную сумму FNV-1a.
    let hex = concat!(
        "53494d4c4f470d0a010007000000000000000900000000000000010000000100",
        "000078010000006d000000000000000001000000000000000000010000000000",
        "00000040420f0000000000000000000000f43f40420f00000000000100000000",
        "0000000b00000000000000000100000000000000004e9d61b401b74ecf",
    );
    let bytes = (0..hex.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap())
        .collect::<Vec<_>>();
    let log = sim_runtime::Log::read_binary(bytes.as_slice()).unwrap();
    assert_eq!(log.header().observations[0].unit, "m");
    assert_eq!(log.entries().len(), 1);
    assert_eq!(log.entries()[0].value, 1.25);
    assert_eq!(log.entries()[0].at, Time(1_000_000));
    let mut output = Vec::new();
    log.write_binary(&mut output).unwrap();
    assert_eq!(output, bytes);
}

#[test]
fn raw_flash_читает_явный_профиль_и_проверяет_полноту_входов() {
    let mut assets = Assets::default();
    assets.put(
        "image",
        include_bytes!("../../sim-boards/tests/data/esp_peripherals/flash.bin"),
    );
    // TRM §3.3.2: ROM0+ROM1 по IBUS 384 КиБ. Вектор 0x40000000.
    // Контрольный ROM содержит RISC-V Unprivileged ISA, EBREAK в векторе сброса.
    let mut rom = vec![0; 384 * 1024];
    rom[..4].copy_from_slice(&0x0010_0073_u32.to_le_bytes());
    assets.put("rom", rom);
    // ESP-IDF efuse_reg.h: 0x2c..0x17c, последнее слово перед регистром ошибок.
    assets.put("efuse", [0; 0x17c - 0x2c]);
    // flash-layout.json для штатной сборки, TRM §§5.8 и 10.3 для strap и фазы.
    let profile = r#"format = 1
board = "esp32-c3"
partition_table_offset = 0x8000
gpio2 = true
gpio8 = true
gpio9 = true
systimer_first_xtal_cycles = 2
source = "Образ esp_peripherals, синтетические входы ROM"
"#;
    assets.put("flash.toml", profile);
    let config = wiring(
        "esp32-c3",
        "flash_profile='flash.toml'\nrom='rom'\nefuse='efuse'",
    );
    let mut rt = runtime();
    control::install(&mut rt, &config, &assets, run_spec(Time(1_000_000))).unwrap();
    assert_eq!(
        rt.run_until(Time(1_000_000)).unwrap(),
        Status::ComponentFailure
    );
    let events = format!("{:?}", control::run(&rt).unwrap().events());
    assert!(events.contains("Breakpoint { pc: 1073741824 }"), "{events}");
    assets.put("flash.toml", profile.replace("0x8000", "0x7fff"));
    refuses(&config, &assets, "смещение таблицы разделов");
    assets.put("flash.toml", profile.replace("cycles = 2", "cycles = 4"));
    refuses(&config, &assets, "фаза SYSTIMER");
    assets.put("flash.toml", profile);
    assets.put("efuse", [0; 1]);
    refuses(&config, &assets, "слово eFuse");
    assets.put("efuse", [0; 4]);
    refuses(&config, &assets, "eFuse");
}

#[test]
fn внешняя_плата_требует_совместимых_блоков_портов_и_образов() {
    let mut assets = Assets::default();
    let config = ControlConfig::parse(EXTERNAL_WIRING).unwrap();
    for (profile, error) in [
        (
            EXTERNAL.replace("gpio_strap", "i2c"),
            "несовместимый блок «i2c»",
        ),
        (EXTERNAL.replace("uart0", "uart1"), "порт «uart1»"),
        (
            EXTERNAL.replace("gpio_strap", "psram_opi"),
            "настройками PSRAM",
        ),
        (EXTERNAL.replace("esp32-s3-n16r8", "другая"), "несовместим"),
        (
            EXTERNAL.replace("uart_download", "flash"),
            "нужен образ flash",
        ),
        (
            EXTERNAL.replace("icount_shift = 3", "icount_shift = 11"),
            "icount_shift",
        ),
    ] {
        assets.put("external.toml", profile);
        refuses(&config, &assets, error);
    }
    assets.put("external.toml", EXTERNAL);
    let mut rt = runtime();
    let horizon = Time(1_000_000); // Один шаг блока.
    control::install(&mut rt, &config, &assets, run_spec(horizon)).unwrap();
    assert!(rt.simplifications().iter().any(|s| s.blocks_proof
        && s.text.contains("NotAnalyzed")
        && s.text.contains("снимка QEMU")));
    // Даже GPIO-действие без остальной проводки требует порта общего времени.
    let gpio = ControlConfig::parse(&format!("{EXTERNAL_WIRING}\n[[board_action]]\nid='g'\nat_ns=0\nboard='блок'\nkind='gpio'\npin=0\nhigh=true\nsource='стенд'\n")).unwrap();
    refuses(&gpio, &assets, "виртуальными метками времени");
    let recording = control::run(&rt).unwrap().recording();
    assert!(
        control::install_replay(
            &mut runtime(),
            &config,
            &assets,
            run_spec(horizon),
            recording
        )
        .unwrap_err()
        .contains("снимка QEMU")
    );
    let bad_images = ControlConfig::parse(&format!(
        "{EXTERNAL_WIRING}\nimage='image'\nefuse='efuse'\n"
    ))
    .unwrap();
    assets.put("image", [0]);
    refuses(&bad_images, &assets, "размер образа flash");
    // Руководство Espressif QEMU S3: допустим flash 2 МиБ, eFuse 1 КиБ.
    assets.put("image", vec![0; 2 << 20]);
    assets.put("efuse", [0]);
    refuses(&bad_images, &assets, "размер образа eFuse");
}

fn diagnostic(
    profile: &str,
    case: &str,
) -> (control::external_board::Report, sim_world::Simplification) {
    struct Temp(PathBuf);
    impl Drop for Temp {
        fn drop(&mut self) {
            std::fs::remove_dir_all(&self.0).unwrap();
        }
    }
    let root = std::env::temp_dir().join(format!("mcu-catalog-{}-{case}", std::process::id()));
    std::fs::create_dir(&root).unwrap();
    let directory = Temp(root);
    std::fs::write(directory.0.join("external.toml"), profile).unwrap();
    ControlConfig::parse(EXTERNAL_WIRING)
        .unwrap()
        .probe_external_board(
            &FileAssets { base: &directory.0 },
            "блок",
            UartProbe {
                port: "uart0",
                transmit: None,
                until: b"wait uart0 download\r\n",
            },
        )
        .unwrap()
}

#[test]
fn недоступный_qemu_остаётся_открытым() {
    let (report, gap) = diagnostic(EXTERNAL, "missing");
    let Stop::Unavailable(reason) = report.stop else {
        panic!("{report:?}");
    };
    assert!(gap.blocks_proof && gap.text.contains(&reason));
}

#[test]
#[ignore = "нужен SIM_EXTERNAL_BOARD_QEMU из поставки Espressif"]
fn qemu_s3_подтверждает_uart0_через_профиль() {
    let qemu = std::env::var("SIM_EXTERNAL_BOARD_QEMU").unwrap();
    let profile = EXTERNAL.replace("/несуществующий/qemu-system-xtensa", &qemu);
    let (report, gap) = diagnostic(&profile, "qemu");
    assert_eq!(report.stop, Stop::Marker, "{report:?}");
    assert!(report.uart.starts_with(b"ESP-ROM:esp32s3-"));
    assert!(report.uart.ends_with(b"wait uart0 download\r\n"));
    assert!(gap.blocks_proof && gap.text.contains("виртуальному времени"));
    println!("{} байт UART0, ROM ESP32-S3", report.uart.len());
}
