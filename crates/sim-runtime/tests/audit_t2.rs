use std::collections::BTreeMap;
use std::path::PathBuf;

use sim_boards::{BoardOutput, ResetCause};
use sim_process::Analysis;
use sim_process::schematic::Status as FindingStatus;
use sim_radio::timed::{
    AttemptLink, EspNowChannel, EspNowConfirmation, EspNowError, EspNowEventKind, EspNowInterface,
    EspNowLink, EspNowPeer, EspNowProfile, EspNowVersion,
};
use sim_runtime::control::external_board::{BlockCoverage, Report, Stop, esp32s3_coverage};
use sim_runtime::control::{self, ControlAssets, ControlConfig, EventKind, RunSpec};
use sim_runtime::{Runtime, Status};
use sim_signal::{Rate, Rng, Time};
use sim_world::CompiledWorld;

const MS: u64 = 1_000_000; // Наносекунд в миллисекунде.
const END: Time = Time(2 * MS); // Два шага стенда, снимок ровно посередине.
const EXTERNAL: &str = "version=1\n[[external_board]]\nname='блок'\ncatalog='esp32-s3-n16r8'\nprofile='qemu'\nsource='Стенд UART0'\n";
// Лимиты условного диагностического стенда. Установщик QEMU здесь не запускает.
const QEMU: &str = r#"
format=1
board='esp32-s3-n16r8'
qemu='qemu-system-xtensa'
machine='esp32s3'
boot='uart_download'
clock='icount'
icount_shift=3
seed=0
wall_time_ms=1000
address_space_bytes=515396075520
uart_bytes=1024
stderr_bytes=1024
required_blocks=['uart']
required_ports=['uart0']
source='QEMU Espressif Supported features, ESP32-S3'
"#;

struct Assets(BTreeMap<&'static str, Vec<u8>>);

impl ControlAssets for Assets {
    fn read(&self, name: &str) -> Result<Vec<u8>, String> {
        self.0
            .get(name)
            .cloned()
            .ok_or_else(|| format!("нет ресурса {name}"))
    }

    fn executable(&self, _: &str) -> Result<PathBuf, String> {
        panic!("диагностика вне установки контура")
    }
}

fn runtime() -> Runtime {
    // Нейтральный блок массой 1 кг, размер 0,2 м, шаг 1 мс.
    Runtime::load(CompiledWorld {
        name: "блок".into(),
        seed: 0,
        physics: Rate::hz(1000).unwrap(),
        mjcf: "<mujoco><option timestep='0.001'/><worldbody><body><freejoint/><geom type='box' size='.1 .1 .1' mass='1'/></body></worldbody></mujoco>".into(),
        bodies: vec![],
        joints: vec![],
        efforts: vec![],
        drives: vec![],
        sensors: vec![],
        simplifications: vec![],
    }).unwrap()
}

fn spec(horizon: Time) -> RunSpec<'static> {
    RunSpec {
        world: b"block",
        catalog: b"bench",
        task: b"audit_t2",
        criteria: b"audit_t2",
        mode: "блок",
        horizon,
        model_ports: &[],
    }
}

#[test]
fn маркер_uart_не_подтверждает_повторяемость_внешней_платы() {
    let marker = b"ready";
    let report = Report {
        stop: Stop::Marker,
        uart: marker.to_vec(),
        stderr: vec![],
        input_written: 0,
    };
    assert_eq!(report.uart, marker);
    assert!(
        matches!(report.repeatability(), Analysis::NotAnalyzed { reason } if reason.contains("снимка QEMU"))
    );

    let assets = Assets(BTreeMap::from([("qemu", QEMU.as_bytes().to_vec())]));
    let config = ControlConfig::parse(EXTERNAL).unwrap();
    let mut rt = runtime();
    control::install(&mut rt, &config, &assets, spec(END)).unwrap();
    assert!(
        rt.simplifications()
            .iter()
            .any(|s| s.blocks_proof && s.text.contains("снимка QEMU"))
    );
    assert_eq!(rt.run_until(Time(MS)).unwrap(), Status::Running);
    assert_eq!(rt.log().end().time, Time(MS));
    let recording = control::run(&rt).unwrap().recording();
    assert!(
        control::install_replay(&mut runtime(), &config, &assets, spec(END), recording)
            .unwrap_err()
            .contains("снимка QEMU")
    );
    assert!(matches!(rt.try_snapshot(), Err(error) if error.contains("снимка QEMU")));
    assert!(matches!(rt.fork(), Err(error) if error.contains("снимка QEMU")));
    let snapshot = rt.snapshot();
    assert!(rt.restore(&snapshot).unwrap_err().contains("снимка QEMU"));

    let mut rt = runtime();
    control::install(&mut rt, &config, &assets, spec(END)).unwrap();
    assert!(
        rt.audit_snapshot(Time(MS), END)
            .unwrap_err()
            .contains("снимка QEMU")
    );
    assert_eq!(rt.log().end().time, Time::ZERO);
    // Такой же блок с пустой проводкой имеет полноценный снимок.
    let mut rt = runtime();
    control::install(
        &mut rt,
        &ControlConfig::parse("version=1").unwrap(),
        &assets,
        spec(END),
    )
    .unwrap();
    rt.audit_snapshot(Time(MS), END).unwrap();
    assert_eq!(rt.log().end().time, END);
}

#[test]
fn неподдержанные_блоки_порты_и_профили_qemu_отклоняются() {
    assert_eq!(esp32s3_coverage("uart"), BlockCoverage::Documented);
    for block in ["i2c", "adc_calibration", "deep_sleep", "неизвестный блок"] {
        assert!(
            matches!(esp32s3_coverage(block), BlockCoverage::NotAnalyzed { reason } if !reason.is_empty())
        );
    }
    let config = ControlConfig::parse(EXTERNAL).unwrap();
    for (profile, reason) in [
        (
            QEMU.replace("['uart']", "['i2c']"),
            "несовместимый блок «i2c»",
        ),
        (
            QEMU.replace("['uart']", "['psram_opi']"),
            "настройками PSRAM",
        ),
        (QEMU.replace("['uart0']", "['uart1']"), "порт «uart1»"),
        (QEMU.replace("esp32-s3-n16r8", "esp32-c3"), "несовместим"),
        (
            QEMU.replace("machine='esp32s3'", "machine='esp32c3'"),
            "несовместимая машина",
        ),
    ] {
        let mut rt = runtime();
        let assets = Assets(BTreeMap::from([("qemu", profile.into_bytes())]));
        let error = control::install(&mut rt, &config, &assets, spec(END)).unwrap_err();
        assert!(error.contains(reason), "{error}");
        assert!(control::run(&rt).is_none());
        assert!(rt.simplifications().is_empty());
    }
}

#[test]
fn паспортный_заряд_мк_не_заменяет_профиль_батареи() {
    const POWER: &str = include_str!("../../../catalog/boards/electrical/esp32-c3-power.toml");
    let profile = ControlConfig::parse_mcu_power_profile(POWER).unwrap();
    // ESP32-C3 Datasheet v2.5, §5.6.2, табл. 5-9, Deep-sleep, 5 мкА.
    let Analysis::Analyzed { value, .. } = profile.current("deep_sleep") else {
        panic!("нет паспортного тока")
    };
    assert_eq!(value.current_a, 5e-6);
    assert!(
        matches!(&value.source, sim_world::Provenance::Datasheet(s) if s.contains("§5.6.2, табл. 5-9"))
    );
    for bad in [
        POWER.replace("section = \"5.6.2\"", "section = \"\""),
        POWER.replace("0.017", "NaN"),
        POWER.replace("0.017", "inf"),
    ] {
        assert!(ControlConfig::parse_mcu_power_profile(&bad).is_err());
    }
    let assets = Assets(BTreeMap::from([
        ("power", POWER.as_bytes().to_vec()),
        ("schedule", "format=1\nrepeat=true\nsource='Один шаг сна блока'\n[[states]]\nid='deep_sleep'\nduration_ns=1000000\n".as_bytes().to_vec()),
    ]));
    let config = ControlConfig::parse("version=1\n[[board]]\nname='блок'\ncatalog='esp32-c3'\nimage='образ'\nrate_hz=1000\npower_profile='power'\npower_schedule='schedule'\nsource='Сетка стенда 1 мс'\n").unwrap();
    let report = config.mcu_energy(&assets, "блок", END).unwrap();
    let Analysis::Analyzed { value, .. } = report.chip_charge else {
        panic!("нет расчёта заряда")
    };
    assert_eq!(value.window.end, END);
    assert_eq!(value.charge_c, 5e-6 * 2e-3); // Два шага по 1 мс, Q=I·t.
    assert_eq!(report.sleep_drain.status, FindingStatus::NotAnalyzed);
}

#[test]
fn esp_now_доставка_подтверждение_и_отсутствующие_данные_различимы() {
    // Синтетический маршрут: одна попытка за 1 мс, тайм-аут ответа через ещё 1 мс.
    let sender = EspNowProfile {
        version: EspNowVersion::V2,
        wifi_channel: 6,
        station_enabled: true,
        soft_ap_enabled: false,
        queue_capacity: Some(1),
        attempt_ns: Some(MS),
        mac_attempts: Some(1),
    };
    let rng = || Rng::stream(0, "аудит ESP-NOW");
    let no_timing = EspNowProfile {
        attempt_ns: None,
        ..sender
    };
    assert!(matches!(
        EspNowChannel::new(no_timing, rng()),
        Err(EspNowError::NotAnalyzed(_))
    ));
    let address = [2, 0, 0, 0, 0, 1]; // Локальный unicast адрес стенда, RFC 9542 §2.1.1.
    let peer = EspNowPeer {
        address,
        interface: EspNowInterface::Station,
        channel: 6,
        encrypted: false,
    };
    let link = EspNowLink {
        frame: AttemptLink {
            loss: 0.0,
            rssi_dbm: -43,
        }, // Тестовый сигнал, без потерь кадра.
        receiver_channel: Some(6),
        receiver_version: Some(EspNowVersion::V2),
        mac_ack_loss: Some(0.0),
    };
    let app = EspNowConfirmation::Application {
        timeout_ns: Some(MS),
        transmissions: Some(1),
    };
    let mut channel = EspNowChannel::new(sender, rng()).unwrap();
    channel.add_peer(peer).unwrap();
    let id = channel
        .send(Time::ZERO, address, b"x".to_vec(), app)
        .unwrap();
    assert_eq!(
        channel.acknowledge(Time(MS - 1), address, id),
        Err(EspNowError::UnexpectedAck)
    );
    for bad_ack in [None, Some(f64::NAN), Some(f64::INFINITY)] {
        let attempted = channel.advance_with_link(Time(MS), |_| {
            Ok(EspNowLink {
                mac_ack_loss: bad_ack,
                ..link
            })
        });
        if bad_ack.is_none() {
            assert!(matches!(attempted, Err(EspNowError::NotAnalyzed(_))));
        } else {
            assert!(matches!(attempted, Err(EspNowError::Invalid(_))));
        }
        assert_eq!(channel.next_event(), Some(Time(MS)));
    }
    let events = channel.advance_with_link(Time(MS), |_| Ok(link)).unwrap();
    assert_eq!(
        events.iter().map(|e| (&e.kind, e.at)).collect::<Vec<_>>(),
        [
            (
                &EspNowEventKind::Received {
                    bytes: b"x".to_vec(),
                    rssi_dbm: -43
                },
                Time(MS)
            ),
            (&EspNowEventKind::MacConfirmed, Time(MS)),
        ]
    );
    let mut confirmed = channel.clone();
    assert_eq!(
        confirmed
            .acknowledge(Time(MS + 1), address, id)
            .unwrap()
            .kind,
        EspNowEventKind::ApplicationConfirmed
    );
    assert_eq!(confirmed.next_event(), None);
    assert_eq!(
        channel.acknowledge(END, address, id),
        Err(EspNowError::UnexpectedAck)
    );
    let events = channel
        .advance_with_link(END, |_| panic!("повторов в стенде нет"))
        .unwrap();
    assert_eq!(
        events.iter().map(|e| (&e.kind, e.at)).collect::<Vec<_>>(),
        [(&EspNowEventKind::ApplicationTimedOut, END)]
    );
}

#[test]
fn виртуальный_wdt_и_доставка_не_закрывают_доказательство_платы_и_радио() {
    let mut rt = runtime();
    rt.register_observation("вход", "m").unwrap();
    // Код vmcu_ports.elf: байт 1 включает WDT на 30 мкс, следующие 4 байта задают PWM 1000 Гц.
    let mut watchdog = vec![b'S', 1, 0, 0];
    watchdog.extend(1000_u32.to_le_bytes());
    let assets = Assets(BTreeMap::from([
        (
            "elf",
            include_bytes!("../../sim-boards/testdata/vmcu_ports.elf").to_vec(),
        ),
        ("wdt", watchdog),
    ]));
    // Стенд опрашивает плату каждые 5 мкс, задержка пакетной попытки 1 мс.
    let config = ControlConfig::parse(
        r#"
version=1
[[board]]
name='блок'
catalog='vmcu-c3'
image='elf'
config='wdt'
rate_hz=200000
source='Сетка стенда 5 мкс'
[[board_input]]
name='ADC'
board='блок'
signal='вход'
unit='m'
kind='adc'
port=0
max_age_ns=0
scale=0
offset=1.65
fallback=1.65
source='Половина опорных 3,3 В стенда'
[[fault]]
id='пауза'
target='board:блок'
at_ns=20000
until_ns=100000
kind='pause'
source='Пауза длиннее настроенных 30 мкс WDT'
[[packet]]
name='радио'
kind='wifi'
to_host=true
attempt_ns=1000000
loss=0
rssi_dbm=-43
attempts=1
capacity=1
max_bytes=1
max_age_ns=2000000
source='Идеальная линия стенда, 1 мс на пакет'
[[transmission]]
route='радио'
at_ns=0
bytes=[1]
source='Контрольный байт стенда'
"#,
    )
    .unwrap();
    control::install(&mut rt, &config, &assets, spec(END)).unwrap();
    assert_eq!(
        rt.run_until(END).unwrap(),
        Status::Running,
        "{:?}",
        rt.failure()
    );
    let run = control::run(&rt).unwrap();
    assert!(run.reached_horizon());
    assert!(run.events().iter().any(
        |e| matches!(&e.kind, EventKind::BoardOutput { board, produced_at, event }
        if board == "блок" && matches!(event.output, BoardOutput::Reset(ResetCause::Watchdog))
            && (Time(20_000)..Time(100_000)).contains(produced_at))
    ));
    assert!(run.events().iter().any(
        |e| matches!(&e.kind, EventKind::PacketDelivered { route, bytes, .. }
        if route == "радио" && bytes == &[1])
            && e.at == Time(MS)
    ));
    for subject in ["board:блок", "channel:радио"] {
        assert!(
            run.simplifications()
                .iter()
                .any(|s| s.subject == subject && s.blocks_proof)
        );
    }
}
