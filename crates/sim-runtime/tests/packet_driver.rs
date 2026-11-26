use std::path::PathBuf;

use sim_boards::{
    BoardFault, BoardId, BoardInput, BoardOutput, EspRadioAbi as Abi, EspRadioPort, InputEvent,
    InputOutcome, Packet, RadioStatus, new_board,
};
use sim_runtime::control::{
    self, ControlAssets, ControlConfig, EventKind, PacketOutcome, Rejection, RunSpec,
};
use sim_runtime::{Runtime, Status};
use sim_signal::{Rate, Time};
use sim_world::CompiledWorld;

const MS: u64 = 1_000_000;
const ATTEMPT: u64 = MS / 4;
const END: Time = Time(10 * MS);
const PAYLOAD: &[u8] = b"A\\";
const RSSI: i8 = -43;

// TRM §3.3.2, табл. 3.3-1: разные участки SRAM1 через IBUS (код) и DBUS (буфер).
const TEXT: u32 = 0x4038_0000;
const MAILBOX: u32 = 0x3fc8_1000;

// Минимальный стенд RV32I. Кодировки сверяются с Unprivileged ISA, гл. 2,
// и ассемблером riscv32-esp-elf. x5 — база команды, x6 — данные, x7 — условие.
struct Firmware(Vec<u32>);

impl Firmware {
    fn new() -> Self {
        Self(vec![MAILBOX | (5 << 7) | 0x37]) // lui t0, %hi(mailbox)
    }

    fn store(&mut self, offset: usize, value: u8, byte: bool) {
        self.0.push((u32::from(value) << 20) | (6 << 7) | 0x13); // addi t1, zero, value
        self.store_register(offset, 6, byte);
    }

    fn store_register(&mut self, offset: usize, register: u32, byte: bool) {
        let offset = offset as u32;
        let funct3 = if byte { 0 } else { 2 }; // SB / SW
        self.0.push(
            ((offset >> 5) << 25)
                | (register << 20)
                | (5 << 15)
                | (funct3 << 12)
                | ((offset & 31) << 7)
                | 0x23,
        );
    }

    fn load(&mut self, offset: usize) {
        self.0
            .push((offset as u32) << 20 | (5 << 15) | (2 << 12) | (7 << 7) | 0x03); // lw t2, offset(t0)
    }

    fn call(&mut self, op: u32) {
        self.store(Abi::FUNCTION, u8::try_from(op).unwrap(), false);
        self.0.extend([Abi::INSTRUCTION, MAILBOX]);
    }

    fn open(&mut self, port: EspRadioPort) {
        self.store(Abi::PORT, port as u8, false);
        self.call(Abi::OPEN);
    }

    fn send(&mut self, bytes: &[u8]) {
        self.store(Abi::LENGTH, bytes.len() as u8, false);
        for (i, &byte) in bytes.iter().enumerate() {
            self.store(Abi::PAYLOAD_OFFSET + i, byte, true);
        }
        self.call(Abi::SEND);
    }

    fn poll(&mut self, op: u32, repeated_status: u32) {
        let start = self.0.len();
        if op == Abi::RECEIVE {
            // При пустой очереди гость получает длину 0 и выставляет ёмкость снова.
            self.store(Abi::LENGTH, PAYLOAD.len() as u8, false);
        }
        self.call(op);
        self.load(Abi::STATUS);
        self.0.push((repeated_status << 20) | (6 << 7) | 0x13); // addi t1, zero, status
        // beq t2,t1,start. B-immediate распределён по битам слова.
        let rel = (-((self.0.len() - start) as i32) * 4) as u32;
        self.0.push(
            (((rel >> 12) & 1) << 31)
                | (((rel >> 5) & 63) << 25)
                | (6 << 20)
                | (7 << 15)
                | (((rel >> 1) & 15) << 8)
                | (((rel >> 11) & 1) << 7)
                | 0x63,
        );
    }

    fn finish(mut self) -> Vec<u8> {
        self.0.push(0x0000_006f); // jal zero, 0
        elf(&self.0)
    }
}

fn elf(code: &[u32]) -> Vec<u8> {
    // ELF32 little endian, System V ABI, §4–5. Один исполняемый PT_LOAD.
    const EH: usize = 52;
    const PH: usize = 32;
    let mut bytes = vec![0; EH + PH];
    bytes[..7].copy_from_slice(b"\x7fELF\x01\x01\x01");
    let write16 = |b: &mut [u8], at, n: u16| b[at..at + 2].copy_from_slice(&n.to_le_bytes());
    let write32 = |b: &mut [u8], at, n: u32| b[at..at + 4].copy_from_slice(&n.to_le_bytes());
    write16(&mut bytes, 16, 2); // ET_EXEC
    write16(&mut bytes, 18, 243); // EM_RISCV, RISC-V psABI
    write32(&mut bytes, 20, 1); // EV_CURRENT
    write32(&mut bytes, 24, TEXT);
    write32(&mut bytes, 28, EH as u32);
    write16(&mut bytes, 40, EH as u16);
    write16(&mut bytes, 42, PH as u16);
    write16(&mut bytes, 44, 1);
    let size = size_of_val(code) as u32;
    for (offset, value) in [
        (0, 1), // PT_LOAD
        (4, (EH + PH) as u32),
        (8, TEXT),
        (12, TEXT),
        (16, size),
        (20, size),
        (24, 5), // PF_R | PF_X
        (28, 4), // Выравнивание 32-битной инструкции.
    ] {
        write32(&mut bytes, EH + offset, value);
    }
    bytes.extend(code.iter().flat_map(|n| n.to_le_bytes()));
    bytes
}

fn producer(port: EspRadioPort, report_status: bool) -> Vec<u8> {
    let mut code = Firmware::new();
    code.open(port);
    code.send(PAYLOAD);
    if report_status {
        code.poll(Abi::TX_STATUS, Abi::PENDING);
        code.open(EspRadioPort::Wifi);
        code.store(Abi::LENGTH, 1, false);
        code.store_register(Abi::PAYLOAD_OFFSET, 7, true);
        code.call(Abi::SEND);
    }
    code.finish()
}

fn consumer(port: EspRadioPort, echoes: bool, responds: bool) -> Vec<u8> {
    let mut code = Firmware::new();
    code.open(port);
    code.poll(Abi::RECEIVE, Abi::EMPTY);
    if responds {
        code.call(Abi::ACK);
    }
    if echoes {
        code.load(Abi::RSSI);
        code.store_register(Abi::PAYLOAD_OFFSET + PAYLOAD.len(), 7, true);
        code.store(Abi::LENGTH, (PAYLOAD.len() + 1) as u8, false);
        code.call(Abi::SEND);
    }
    code.finish()
}

struct Assets {
    tx: Vec<u8>,
    rx: Vec<u8>,
    profile: String,
}

impl ControlAssets for Assets {
    fn read(&self, name: &str) -> Result<Vec<u8>, String> {
        match name {
            "tx" => Ok(self.tx.clone()),
            "rx" => Ok(self.rx.clone()),
            "profile" => Ok(self.profile.as_bytes().to_vec()),
            _ => Err(format!("неизвестный ресурс {name}")),
        }
    }

    fn executable(&self, _: &str) -> Result<PathBuf, String> {
        Err("у стенда нет внешнего процесса".into())
    }
}

fn base() -> &'static str {
    r#"version=1
[[board]]
name='передатчик'
catalog='esp32-c3'
image='tx'
rate_hz=1000
source='сетка стенда 1 мс'
[[board]]
name='приёмник'
catalog='esp32-c3'
image='rx'
rate_hz=1000
source='сетка стенда 1 мс'
"#
}

fn route(name: &str, kind: &str, from: &str, to: Option<&str>, port: EspRadioPort) -> String {
    let port = port as u32;
    let destination = to.map_or("to_host=true".into(), |name| {
        format!("to_board='{name}'\nto_port={port}")
    });
    let params = match kind {
        "esp_now" => "profile_asset='profile'".into(),
        "ble" => format!("interval_ns={ATTEMPT}\nloss=0\nrssi_dbm={RSSI}\nattempts=2"),
        "wifi" => format!("attempt_ns={ATTEMPT}\nloss=0\nrssi_dbm={RSSI}\nattempts=2"),
        _ => unreachable!(),
    };
    format!(
        r#"
[[packet]]
name='{name}'
kind='{kind}'
from_board='{from}'
from_port={port}
{destination}
{params}
capacity=2
max_bytes=32
max_age_ns={END_NS}
source='синтетические параметры стенда'
"#,
        END_NS = END.0
    )
}

fn install(config: &str, assets: &Assets) -> Result<Runtime, String> {
    let mut rt = Runtime::load(CompiledWorld {
        name: "каретка".into(),
        seed: 71,
        physics: Rate::hz(1000).unwrap(),
        mjcf: "<mujoco><option timestep='0.001'/><worldbody><geom type='plane' size='1 1 1'/></worldbody></mujoco>".into(),
        bodies: vec![],
        joints: vec![],
        efforts: vec![],
        drives: vec![],
        sensors: vec![],
        simplifications: vec![],
    })?;
    control::install(
        &mut rt,
        &ControlConfig::parse(config)?,
        assets,
        RunSpec {
            world: "каретка".as_bytes(),
            catalog: "стенд".as_bytes(),
            task: b"packet_driver",
            criteria: b"packet_driver",
            mode: "стенд",
            horizon: END,
            model_ports: &[],
        },
    )?;
    Ok(rt)
}

fn advance(rt: &mut Runtime, until: Time) {
    assert_eq!(
        rt.run_until(until).unwrap(),
        Status::Running,
        "{:?}",
        rt.failure()
    );
}

fn terminal(rt: &Runtime, port: EspRadioPort) -> Vec<(Time, RadioStatus)> {
    control::run(rt)
        .unwrap()
        .events()
        .iter()
        .filter_map(|e| match &e.kind {
            EventKind::BoardOutput { board, event, .. } if board == "передатчик" => {
                match event.output {
                    BoardOutput::RadioStatus {
                        port: p, status, ..
                    } if p == port as u32 && status != RadioStatus::Pending => Some((e.at, status)),
                    _ => None,
                }
            }
            _ => None,
        })
        .collect()
}

fn delivered(rt: &Runtime, name: &str) -> Vec<(Time, Vec<u8>)> {
    control::run(rt)
        .unwrap()
        .events()
        .iter()
        .filter_map(|e| match &e.kind {
            EventKind::PacketDelivered { route, bytes, .. } if route == name => {
                Some((e.at, bytes.clone()))
            }
            _ => None,
        })
        .collect()
}

fn profile(mac_loss: &str, confirmation: &str) -> String {
    format!(
        r#"format=1
kind='esp_now'
sender_version=2
wifi_channel=6
station_enabled=true
soft_ap_enabled=false
queue_capacity=2
attempt_ns={ATTEMPT}
mac_attempts=2
destination=[2,0,0,0,0,1]
interface='station'
peer_channel=0
encrypted=false
loss=0
rssi_dbm={RSSI}
receiver_channel=6
receiver_version=2
mac_ack_loss={mac_loss}
{confirmation}
source='параметры стенда, ESP-IDF esp_now.h для формата'
"#
    )
}

fn esp_now(assets: &Assets) -> Runtime {
    let config = format!(
        "{}{}{}",
        base(),
        route(
            "прямой",
            "esp_now",
            "передатчик",
            Some("приёмник"),
            EspRadioPort::EspNow
        ),
        route("статус", "wifi", "передатчик", None, EspRadioPort::Wifi)
    );
    install(&config, assets).unwrap()
}

#[test]
fn wifi_и_ble_гость_читает_и_возвращает_байты_и_rssi() {
    for (kind, port, delivered_at) in [
        ("wifi", EspRadioPort::Wifi, Time(3 * MS + ATTEMPT)),
        ("ble", EspRadioPort::Ble, Time(3 * MS + ATTEMPT)),
    ] {
        let assets = Assets {
            tx: producer(port, false),
            rx: consumer(port, true, false),
            profile: String::new(),
        };
        let config = format!(
            "{}{}{}",
            base(),
            route("прямой", kind, "передатчик", Some("приёмник"), port),
            route("обратный", kind, "приёмник", None, port)
        );
        let mut rt = install(&config, &assets).unwrap();
        advance(&mut rt, END);
        assert_eq!(delivered(&rt, "прямой"), [(Time(2 * MS), PAYLOAD.to_vec())]);
        let mut echo = PAYLOAD.to_vec();
        echo.push(RSSI as u8);
        assert_eq!(delivered(&rt, "обратный"), [(delivered_at, echo)]);
        assert!(matches!(
            terminal(&rt, port).as_slice(),
            [(Time(MS), RadioStatus::NotAnalyzed(_))]
        ));
    }
}

#[test]
fn esp_now_потеря_mac_ack_отличается_от_приёма_пакета() {
    let port = EspRadioPort::EspNow;
    let mut assets = Assets {
        tx: producer(port, true),
        rx: consumer(port, false, false),
        profile: profile("0", "confirmation='driver'"),
    };
    let mut nominal = esp_now(&assets);
    advance(&mut nominal, END);
    assert_eq!(
        terminal(&nominal, port),
        [(Time(2 * MS), RadioStatus::MacConfirmed)]
    );
    assert_eq!(
        delivered(&nominal, "прямой"),
        [(Time(2 * MS), PAYLOAD.to_vec())]
    );
    assert_eq!(
        delivered(&nominal, "статус"),
        [(Time(3 * MS + ATTEMPT), vec![Abi::MAC_CONFIRMED as u8])]
    );

    assets.profile = profile("1", "confirmation='driver'");
    let mut lost = esp_now(&assets);
    advance(&mut lost, END);
    assert_eq!(terminal(&lost, port), [(Time(2 * MS), RadioStatus::Failed)]);
    assert_eq!(
        delivered(&lost, "прямой"),
        [(Time(2 * MS), PAYLOAD.to_vec())]
    );
    let attempts: Vec<_> = control::run(&lost)
        .unwrap()
        .events()
        .iter()
        .filter_map(|e| match &e.kind {
            EventKind::Packet { route, event }
                if route == "прямой"
                    && matches!(event.outcome, PacketOutcome::Retry | PacketOutcome::Lost) =>
            {
                Some((event.at, event.attempt, event.outcome.clone()))
            }
            _ => None,
        })
        .collect();
    assert_eq!(
        attempts,
        [
            (Time(MS + ATTEMPT), 1, PacketOutcome::Retry),
            (Time(MS + 2 * ATTEMPT), 2, PacketOutcome::Lost),
        ]
    );

    assets.profile =
        profile("0", "confirmation='driver'").replace("receiver_channel=6", "receiver_channel=11");
    let mut mismatch = esp_now(&assets);
    advance(&mut mismatch, END);
    assert!(delivered(&mismatch, "прямой").is_empty());
    assert_eq!(
        terminal(&mismatch, port),
        [(Time(2 * MS), RadioStatus::Failed)]
    );
}

#[test]
fn ответ_приложения_имеет_обратную_линию_срок_и_снимок() {
    let port = EspRadioPort::EspNow;
    // MAC завершается в 1,25 мс, дедлайн приложения в 4,25 мс.
    // Гость вызывает ACK на границе 3 мс, обратный пакет приходит через reply_attempt_ns.
    let application = |reply_ns, reply_loss| {
        format!(
            "confirmation='application'\ntimeout_ns={}\ntransmissions=1\nreply_attempt_ns={reply_ns}\nreply_loss={reply_loss}\nreply_rssi_dbm={RSSI}",
            3 * MS
        )
    };
    let mut assets = Assets {
        tx: producer(port, true),
        rx: consumer(port, false, true),
        profile: profile("0", &application(ATTEMPT, 0)),
    };
    let mut rt = esp_now(&assets);
    advance(&mut rt, Time(3 * MS));
    assert!(terminal(&rt, port).is_empty());
    let mut branch = rt.fork().unwrap();
    advance(&mut rt, END);
    advance(&mut branch, END);
    assert_eq!(
        control::run(&rt).unwrap().events(),
        control::run(&branch).unwrap().events()
    );
    assert_eq!(
        terminal(&rt, port),
        [(Time(4 * MS), RadioStatus::ApplicationConfirmed)]
    );
    assert_eq!(
        delivered(&rt, "статус"),
        [(
            Time(5 * MS + ATTEMPT),
            vec![Abi::APPLICATION_CONFIRMED as u8]
        )]
    );

    for (reply_ns, loss) in [(MS + ATTEMPT, 0), (ATTEMPT, 1)] {
        assets.profile = profile("0", &application(reply_ns, loss));
        let mut rt = esp_now(&assets);
        advance(&mut rt, END);
        assert_eq!(delivered(&rt, "прямой"), [(Time(2 * MS), PAYLOAD.to_vec())]);
        assert_eq!(
            terminal(&rt, port),
            [(Time(5 * MS), RadioStatus::ApplicationTimedOut)]
        );
        assert!(control::run(&rt).unwrap().events().iter().any(|e| matches!(
            &e.kind,
            EventKind::Rejected {
                reason: Rejection::Expired | Rejection::Disconnected,
                ..
            }
        )));
    }
}

#[test]
fn неизвестный_вызов_ошибка_и_сброс_отменяет_ожидание() {
    let port = EspRadioPort::EspNow;
    let mut code = Firmware::new();
    code.open(port);
    code.call(Abi::ACK + 1); // Следующая, не объявленная в ABI функция.
    let mut board = new_board(BoardId::Esp32C3);
    board.load(&code.finish()).unwrap();
    assert!(
        matches!(board.run_cycles(100), Err(BoardFault::Unsupported(s)) if s.contains("неизвестная функция"))
    );
    assert!(
        board
            .take_events()
            .iter()
            .any(|e| matches!(e.output, BoardOutput::Fault(_)))
    );

    board.load(&producer(port, false)).unwrap();
    assert!(board.run_cycles(100).unwrap().bus_request_pending);
    let sent_cycle = board
        .take_events()
        .into_iter()
        .find(|e| matches!(e.output, BoardOutput::Radio { .. }))
        .unwrap()
        .cycle;
    board.reset();
    let cycle = board.cycles();
    assert!(!board.esp_radio_listening(port as u32));
    assert_eq!(
        board
            .apply(InputEvent {
                cycle,
                input: BoardInput::RadioResult {
                    port: port as u32,
                    sent_cycle,
                    status: RadioStatus::MacConfirmed
                },
            })
            .unwrap(),
        InputOutcome::Unchanged
    );
    assert_eq!(
        board
            .apply(InputEvent {
                cycle,
                input: BoardInput::Radio {
                    port: port as u32,
                    packet: Packet {
                        bytes: PAYLOAD.to_vec(),
                        rssi_dbm: Some(RSSI)
                    }
                }
            })
            .unwrap(),
        InputOutcome::PacketDropped
    );
    assert!(board.take_events().iter().any(|e| matches!(
        e.output,
        BoardOutput::RadioStatus {
            status: RadioStatus::NotAnalyzed(_),
            ..
        }
    )));
}

#[test]
fn некорректные_данные_маршрута_отклоняются() {
    let port = EspRadioPort::EspNow;
    let mut assets = Assets {
        tx: producer(port, true),
        rx: consumer(port, false, false),
        profile: profile("NaN", "confirmation='driver'"),
    };
    let config = format!(
        "{}{}",
        base(),
        route("прямой", "esp_now", "передатчик", Some("приёмник"), port)
    );
    let error = install(&config, &assets).err().unwrap();
    assert!(error.contains("mac_ack_loss"), "{error}");
    assets.profile = profile("1e999", "confirmation='driver'");
    let error = install(&config, &assets).err().unwrap();
    assert!(error.contains("неконечное число"), "{error}");
    assets.profile = profile("0", "confirmation='driver'");
    assert!(ControlConfig::parse(&config.replace("from_port=3", "from_port=2")).is_err());
    assets.profile = assets.profile.replace("mac_ack_loss=0", "");
    let mut rt = install(&config, &assets).unwrap();
    assert_eq!(rt.run_until(END).unwrap(), Status::ComponentFailure);
    assert!(
        rt.failure()
            .unwrap()
            .contains("нет потерь обратного MAC ACK")
    );
    assert!(terminal(&rt, port).is_empty());
}

#[test]
fn broadcast_завершает_отправку_без_подтверждения_и_приёма() {
    let port = EspRadioPort::EspNow;
    let assets = Assets {
        tx: producer(port, true),
        rx: Firmware::new().finish(), // Приёмник не открыл логический порт.
        profile: profile("0", "confirmation='driver'").replace(
            "destination=[2,0,0,0,0,1]",
            "destination=[255,255,255,255,255,255]",
        ),
    };
    let mut rt = esp_now(&assets);
    advance(&mut rt, END);
    assert_eq!(
        terminal(&rt, port),
        [(Time(2 * MS), RadioStatus::BroadcastSent)]
    );
    assert!(delivered(&rt, "прямой").is_empty());
    assert_eq!(
        delivered(&rt, "статус"),
        [(Time(3 * MS + ATTEMPT), vec![Abi::BROADCAST_SENT as u8])]
    );
}

#[test]
fn первая_mac_попытка_видит_приёмник_между_тактами_рантайма() {
    let port = EspRadioPort::EspNow;
    let assets = Assets {
        tx: Firmware::new().finish(),
        rx: consumer(port, false, false),
        profile: profile("0", "confirmation='driver'"),
    };
    let config = format!(
        "{}{}{}",
        base(),
        route("прямой", "esp_now", "передатчик", Some("приёмник"), port)
            .replace("from_board='передатчик'\nfrom_port=3\n", ""),
        "[[transmission]]\nroute='прямой'\nat_ns=0\nbytes=[65,92]\nsource='пакет стенда'\n"
    );
    let mut rt = install(&config, &assets).unwrap();
    advance(&mut rt, Time(MS));
    assert_eq!(delivered(&rt, "прямой"), [(Time(MS), PAYLOAD.to_vec())]);
    assert!(control::run(&rt).unwrap().events().iter().any(|e| matches!(
        &e.kind, EventKind::Packet { route, event } if route == "прямой"
            && event.at == Time(ATTEMPT) && event.attempt == 1
            && matches!(event.outcome, PacketOutcome::Delivered { rssi_dbm: RSSI })
    )));
}
