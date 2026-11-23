use std::path::PathBuf;

use sim_boards::{BoardInput, BoardOutput, TransferStatus};
use sim_runtime::control::{
    self, CommandOrigin, ControlAssets, ControlConfig, EventKind, ModelPort, Rejection, RunOrigin,
    RunSpec,
};
use sim_runtime::{
    Forces, LogKind, Object, ObservationOut, Runtime, Sensor, Status, Truth, TruthId,
};
use sim_signal::{ChannelId, Command, Rate, Rng, SignalId, Time};
use sim_world::{CompiledJoint, CompiledJointKind, CompiledWorld, EffortChannel};

const MS: u64 = 1_000_000;
const SOURCE: &str = "source='синтетический стенд'";

fn scene() -> CompiledWorld {
    // Каретка 2 кг, Euler 1 мс, без трения и тяжести. v[k+1]=v[k]+F[k]·dt/m.
    CompiledWorld {
        name: "каретка".into(),
        seed: 71,
        physics: Rate::hz(1000).unwrap(),
        mjcf: r#"<mujoco><option timestep="0.001" gravity="0 0 0" integrator="Euler"/>
            <worldbody><body name="блок"><joint name="ход" type="slide" axis="1 0 0"/>
            <geom type="box" size="0.1 0.1 0.1" mass="2"/></body></worldbody></mujoco>"#
            .into(),
        bodies: vec![],
        joints: vec![CompiledJoint {
            name: "ход".into(),
            owner: "каретка".into(),
            kind: CompiledJointKind::Slide,
        }],
        efforts: vec![
            EffortChannel {
                name: "сила".into(),
                joint: "ход".into(),
                dof: 0,
            },
            EffortChannel {
                name: "питание".into(),
                joint: "ход".into(),
                dof: 0,
            },
        ],
        drives: vec![],
        sensors: vec![],
        simplifications: vec![],
    }
}

#[derive(Clone)]
struct Scripted {
    id: SignalId,
    rows: Vec<(Time, f64, bool, Time)>,
}

impl Sensor for Scripted {
    fn sample(
        &mut self,
        truth: &Truth<'_>,
        _: &mut Rng,
        out: &mut ObservationOut,
    ) -> Result<(), String> {
        for &(sampled, value, valid, delivered) in &self.rows {
            if truth.time() == sampled {
                out.write_measured(self.id, value, valid, sampled, delivered);
            }
        }
        Ok(())
    }
}

fn runtime(rows: Vec<(Time, f64, bool, Time)>) -> Runtime {
    let mut rt = Runtime::load(scene()).unwrap();
    let id = rt.register_observation("измерение", "m").unwrap();
    rt.register_sensor("датчик", Rate::hz(2000).unwrap(), Scripted { id, rows })
        .unwrap();
    rt.record_joint_state("измерение физики").unwrap();
    rt
}

struct Assets;

impl ControlAssets for Assets {
    fn read(&self, name: &str) -> Result<Vec<u8>, String> {
        Ok(match name {
            "копия" => copy_program("измерение", "выход").into_bytes(),
            "низ" => copy_program("цель", "низ").into_bytes(),
            "мощность" => constant_program("подача", 1.0).into_bytes(),
            "работник" => {
                std::fs::read(self.executable(name)?).map_err(|e| e.to_string())?
            }
            "плата" => include_bytes!("../../sim-boards/testdata/vmcu_ports.elf").to_vec(),
            "uart" => include_bytes!("../../sim-boards/testdata/esp_uart.elf").to_vec(),
            "шины" => include_bytes!("support/registers.elf").to_vec(),
            "i2c" => include_bytes!("../../../catalog/bus_devices/as5600.toml").to_vec(),
            "spi" => include_bytes!("../../../catalog/bus_devices/as5048a.toml").to_vec(),
            "байты" => {
                // Стенд UART/PWM платы: ASCII S, watchdog выключен, PWM 1000 Гц.
                let mut bytes = vec![b'S', 0, 0, 0];
                bytes.extend(1000_u32.to_le_bytes());
                bytes
            }
            "watchdog" => {
                // Ассемблер стенда задаёт таймаут 30 мкс при байте 1, PWM 1000 Гц.
                let mut bytes = vec![b'S', 1, 0, 0];
                bytes.extend(1000_u32.to_le_bytes());
                bytes
            }
            "слоты" => {
                b"accepts_through_ns,delivers_at_ns\n250000,500000\n1000000,1000000\n".to_vec()
            }
            "ошибочные слоты" => {
                b"accepts_through_ns,delivers_at_ns\n250000,500000\n250000,1000000\n".to_vec()
            }
            _ => return Err(format!("нет ресурса «{name}»")),
        })
    }

    fn executable(&self, name: &str) -> Result<PathBuf, String> {
        if name == "работник" {
            Ok(env!("CARGO_BIN_EXE_sim-runtime-control-fixture").into())
        } else {
            Err("ресурс не исполняется".into())
        }
    }
}

fn copy_program(input: &str, output: &str) -> String {
    format!(
        "version=1\n[[block]]\nkind='scalar'\nsignal='{input}'\n{SOURCE}\n[[bind]]\nnode=0\noutput='{output}'\nmissing='omit'\n{SOURCE}"
    )
}

fn constant_program(output: &str, value: f64) -> String {
    format!(
        "version=1\n[[block]]\nkind='constant'\nvalue={value}\n{SOURCE}\n[[bind]]\nnode=0\noutput='{output}'\nmissing='omit'\n{SOURCE}"
    )
}

fn spec(horizon: Time, model_ports: &[ModelPort]) -> RunSpec<'_> {
    RunSpec {
        world: b"world",
        catalog: b"catalog",
        task: b"task",
        criteria: b"criteria",
        mode: "каретка",
        horizon,
        model_ports,
    }
}

fn output(
    name: &str,
    owner: &str,
    target: &str,
    kind: &str,
    unit: &str,
    max_age_ns: u64,
) -> String {
    format!(
        "\n[[output]]\nname='{name}'\ncontroller='{owner}'\ntarget='{target}'\ntarget_kind='{kind}'\nunit='{unit}'\nmax_age_ns={max_age_ns}\n{SOURCE}\n"
    )
}

fn input(owner: &str, signal: &str, max_age_ns: u64) -> String {
    format!(
        "\n[[input]]\ncontroller='{owner}'\nsignal='{signal}'\nunit='m'\nkind='scalar'\nmax_age_ns={max_age_ns}\n{SOURCE}\n"
    )
}

fn node(name: &str, program: &str, rate: u64) -> String {
    format!(
        "\n[[controller]]\nname='{name}'\nprogram='{program}'\nrate_hz={rate}\nkind='builtin'\n{SOURCE}\n"
    )
}

fn feedback(age: u64, kind: &str) -> String {
    let mut text = "version=1\n".to_owned();
    text.push_str(&node("верхний", "копия", 2000));
    text.push_str(&input("верхний", "измерение", age));
    text.push_str(&output("выход", "верхний", "сила", kind, "N", 4 * MS));
    text
}

fn add_fault(text: &mut String, id: &str, target: &str, start: u64, end: u64, action: &str) {
    text.push_str(&format!("\n[[fault]]\nid='{id}'\ntarget='{target}'\nat_ns={start}\nuntil_ns={end}\n{action}\n{SOURCE}\n"));
}

fn velocity(rt: &Runtime, at: Time) -> f64 {
    let id = rt
        .log()
        .header()
        .truth
        .iter()
        .position(|spec| spec.name == "qvel[0]")
        .unwrap();
    rt.log()
        .entries()
        .iter()
        .find(|e| e.kind == LogKind::Truth && e.at == at && e.id as usize == id)
        .unwrap()
        .value
}

fn install(rt: &mut Runtime, text: &str, horizon: Time, ports: &[ModelPort]) {
    control::install(
        rt,
        &ControlConfig::parse(text).unwrap(),
        &Assets,
        spec(horizon, ports),
    )
    .unwrap();
}

#[test]
fn middle_of_step_changes_only_the_next_physics_interval() {
    let mut rt = runtime(vec![(Time(MS / 2), 4.0, true, Time(MS / 2))]);
    let cfg = feedback(10 * MS, "effort");
    install(&mut rt, &cfg, Time(3 * MS), &[]);
    assert!(
        rt.act(&Command {
            targets: vec![(ChannelId(0), 9.0)]
        })
        .is_err()
    );
    assert_eq!(
        rt.run_until(Time(3 * MS)).unwrap(),
        Status::Running,
        "{:?}",
        rt.failure()
    );
    assert_eq!(velocity(&rt, Time(MS)), 0.0);
    assert!((velocity(&rt, Time(2 * MS)) - 0.002).abs() < 1e-14);
    assert!((velocity(&rt, Time(3 * MS)) - 0.004).abs() < 1e-14);
    let run = control::run(&rt).unwrap();
    assert!(run.reached_horizon());
    assert!(run.events().iter().any(
        |e| matches!(&e.kind, EventKind::CommandApplied { command, effective_at, .. }
        if e.at == Time(2 * MS) && command.value == 4.0 && *effective_at == Time(MS))
    ));
    assert!(
        !run.events()
            .iter()
            .any(|e| matches!(&e.kind, EventKind::CommandApplied { .. } if e.at <= Time(MS)))
    );
}

#[test]
fn late_measurement_and_invalid_newer_sample_never_reanimate_command() {
    let mut rt = runtime(vec![
        (Time::ZERO, 8.0, true, Time(2 * MS)),
        (Time(MS / 2), 4.0, true, Time(MS)),
        (Time(3 * MS / 2), 9.0, false, Time(3 * MS / 2)),
        (Time(2 * MS), 40.0, true, Time(4 * MS)),
    ]);
    install(&mut rt, &feedback(MS, "effort"), Time(5 * MS), &[]);
    rt.run_until(Time(2 * MS)).unwrap();
    let last = rt.observe();
    assert_eq!(last.latest(SignalId(0)), None);
    assert_eq!(
        control::measurement_time(&rt, SignalId(0))
            .unwrap()
            .captured_at,
        Time(3 * MS / 2)
    );
    assert!(!last.samples[0].valid);
    rt.run_until(Time(5 * MS)).unwrap();
    assert_eq!(velocity(&rt, Time(2 * MS)), 0.002);
    assert_eq!(velocity(&rt, Time(5 * MS)), 0.002);
    let run = control::run(&rt).unwrap();
    assert!(run.events().iter().any(|e| matches!(&e.kind, EventKind::OutputInvalidated { reason: Rejection::Missing, .. } if e.at == Time(3 * MS / 2))));
    assert!(run.events().iter().any(
        |e| matches!(&e.kind, EventKind::ControllerInput { time, valid: false, value: 40.0, .. }
        if time.captured_at == Time(2 * MS) && time.delivered_at == Time(4 * MS))
    ));
}

#[derive(Clone)]
struct PoweredForce {
    ack: TruthId,
}

impl Object for PoweredForce {
    fn update(
        &mut self,
        _: &Truth<'_>,
        command: &Command,
        _: &mut Rng,
        out: &mut Forces,
    ) -> Result<(), String> {
        let find = |id| {
            command
                .targets
                .iter()
                .find(|&&(key, _)| key == ChannelId(id))
                .map(|&(_, value)| value)
        };
        if find(1) == Some(1.0)
            && let Some(value) = find(0)
        {
            out.set(ChannelId(0), value);
            out.publish(self.ack, value);
        }
        Ok(())
    }
}

#[test]
fn faults_are_distinct_from_effect_in_model_and_trajectory() {
    let mut rt = runtime(vec![(Time::ZERO, 4.0, true, Time::ZERO)]);
    let ack = rt.register_truth("принято", "N").unwrap();
    rt.register_object(
        "модель",
        Rate::hz(1000).unwrap(),
        &[ChannelId(0), ChannelId(1)],
        PoweredForce { ack },
    )
    .unwrap();
    let mut cfg = feedback(20 * MS, "drive");
    cfg.push_str("ack='принято'\nfallback=0.0\n");
    cfg.push_str(&node("энергия", "мощность", 1000));
    cfg.push_str(&output(
        "подача",
        "энергия",
        "питание",
        "power",
        "bool",
        4 * MS,
    ));
    cfg.push_str("fallback=0.0\n");
    add_fault(
        &mut cfg,
        "обрыв",
        "wire:выход",
        2 * MS,
        3 * MS,
        "kind='drop'",
    );
    add_fault(
        &mut cfg,
        "залип",
        "actuator:сила",
        3 * MS,
        4 * MS,
        "kind='stuck'\nvalue=2.0",
    );
    add_fault(
        &mut cfg,
        "питание снято",
        "power:питание",
        9 * MS / 2,
        6 * MS,
        "kind='power_off'",
    );
    add_fault(
        &mut cfg,
        "пауза",
        "controller:верхний",
        6 * MS,
        7 * MS,
        "kind='pause'",
    );
    install(
        &mut rt,
        &cfg,
        Time(8 * MS),
        &[
            ModelPort::Drive {
                channel: ChannelId(0),
                accepted: ack,
            },
            ModelPort::Power {
                channel: ChannelId(1),
            },
        ],
    );
    assert_eq!(
        rt.run_until(Time(8 * MS)).unwrap(),
        Status::Running,
        "{:?}",
        rt.failure()
    );
    // На интервалах [0,1)..[7,8) силы 4,4,0,2,4,0,0,4 Н.
    let velocities = [0.0, 0.002, 0.004, 0.004, 0.005, 0.007, 0.007, 0.007, 0.009];
    for (i, expected) in velocities.into_iter().enumerate() {
        assert!(
            (velocity(&rt, Time(i as u64 * MS)) - expected).abs() < 1e-14,
            "такт {i}"
        );
    }
    let run = control::run(&rt).unwrap();
    assert!(run.events().iter().any(|e| matches!(&e.kind, EventKind::FaultRequested { id, active: true, .. } if id == "питание снято" && e.at == Time(9 * MS / 2))));
    assert!(run.events().iter().any(|e| matches!(&e.kind, EventKind::FaultApplied { id, active: true, .. } if id == "питание снято" && e.at == Time(5 * MS))));
    assert!(run.events().iter().any(|e| matches!(&e.kind, EventKind::CommandApplied { command, .. } if command.origin == CommandOrigin::Fault && command.value == 2.0 && e.at == Time(3 * MS))));
}

#[test]
fn invalid_config_cannot_leave_partial_installation_or_numeric_nan() {
    let mut rt = runtime(vec![(Time::ZERO, f64::NAN, false, Time::ZERO)]);
    let cfg = feedback(MS, "effort");
    assert!(ControlConfig::parse(&format!("{cfg}\nspelling_error=1")).is_err());
    assert!(ControlConfig::parse(&cfg.replace("max_age_ns=4000000", "max_age_ns=NaN")).is_err());
    let mut faulty = cfg.clone();
    add_fault(
        &mut faulty,
        "ошибка",
        "sensor:измерение",
        MS,
        2 * MS,
        "kind='pause'",
    );
    let duplicate = format!(
        "{cfg}{}",
        output("другой", "верхний", "сила", "effort", "N", 1000)
    );
    for bad in [
        faulty,
        duplicate,
        cfg.replace("unit='N'", "unit='V'"),
        cfg.replace("сила", "неизвестно"),
    ] {
        let document = ControlConfig::parse(&bad).unwrap();
        assert!(control::install(&mut rt, &document, &Assets, spec(Time(3 * MS), &[])).is_err());
        assert!(control::run(&rt).is_none());
    }
    install(&mut rt, &cfg, Time(3 * MS), &[]);
    assert_eq!(
        rt.run_until(Time(3 * MS)).unwrap(),
        Status::ComponentFailure
    );
    assert!(rt.failure().unwrap().contains("нечисловое"));
    assert!(!control::run(&rt).unwrap().reached_horizon());
}

fn external(
    input_delay: u64,
    output_delay: u64,
    jitter: u64,
    deadline: u64,
    history: bool,
) -> String {
    // Верхний уровень 250 Гц, принимающая сторона 4 кГц, нижний уровень 1 кГц.
    let mut text = format!(
        r#"version=1
[[controller]]
name='верхний'
kind='external'
program='работник'
args=['{deadline}', '$input:измерение', '$output:верх']
rate_hz=250
input_latency_ns={input_delay}
input_jitter_ns=0
output_latency_ns={output_delay}
output_jitter_ns={jitter}
request_hz=4000
request_max_age_ns=2000000000
queue_capacity=4096
max_message_bytes=4096
{SOURCE}
"#
    );
    if history {
        text.push_str("max_history_bytes=8388608\n");
    }
    text.push_str(&input("верхний", "измерение", 2_000_000_000));
    text.push_str(&output("верх", "верхний", "цель", "goal", "m", deadline));
    text.push_str(&node("нижний", "низ", 1000));
    text.push_str(&input("нижний", "цель", 20_000_000_000));
    text.push_str(&output("низ", "нижний", "сила", "effort", "N", 2 * MS));
    text
}

#[test]
fn full_run_snapshot_fork_and_replay_keep_pending_response_packet_and_fault() {
    let horizon = Time(12 * MS);
    let rows = vec![(Time::ZERO, 3.0, true, Time::ZERO)];
    let mut cfg = external(1_100_000, 2_100_000, 100_000, 7 * MS, true);
    cfg.push_str(&board("пакеты", "vmcu-c3", "плата", Some("байты"), 4000));
    cfg.push_str(&board_adc("пакеты", 0.0));
    cfg.push_str(&packet("эфир", "ble", "to_board='пакеты'\nto_port=0\ninterval_ns=7500000\nloss=0\nrssi_dbm=-64\nattempts=3\nmax_bytes=20", 20 * MS));
    cfg.push_str(&transmission("эфир", MS, "[0xd3]"));
    add_fault(
        &mut cfg,
        "разрыв",
        "channel:эфир",
        5 * MS,
        8 * MS,
        "kind='drop'",
    );
    add_fault(
        &mut cfg,
        "задержка",
        "transport:верхний/response",
        5 * MS,
        8 * MS,
        "kind='delay'\ndelay_ns=8000000",
    );
    let document = ControlConfig::parse(&cfg).unwrap();
    let mut split = runtime(rows.clone());
    install(&mut split, &cfg, horizon, &[]);
    split.run_until(Time(2 * MS)).unwrap();
    let interim = control::run(&split).unwrap();
    assert!(interim.events().iter().any(
        |event| matches!(&event.kind, EventKind::ResponseSent { due, .. } if *due == Time(4 * MS))
    ));
    assert!(interim.events().iter().any(
        |event| matches!(&event.kind, EventKind::PacketSent { due, .. } if *due == Time(7_500_000))
    ));
    assert!(!interim.events().iter().any(|event| matches!(
        &event.kind,
        EventKind::ResponseDelivered { .. } | EventKind::PacketDelivered { .. }
    )));
    let snapshot = split.try_snapshot().unwrap();
    let mut branch = split.fork().unwrap();
    assert_ne!(
        control::worker_ids(&split).next(),
        control::worker_ids(&branch).next()
    );
    assert_eq!(
        split.run_until(horizon).unwrap(),
        Status::Running,
        "{:?}",
        split.failure()
    );
    assert_eq!(branch.run_until(horizon).unwrap(), Status::Running);
    let finished = control::run(&split).unwrap();
    let fingerprint = finished.fingerprint();
    assert_eq!(control::run(&branch).unwrap().fingerprint(), fingerprint);
    assert!(finished.events().iter().any(
        |event| matches!(&event.kind, EventKind::Packet { event: packet, .. }
        if packet.outcome == sim_radio::timed::Outcome::Disconnected && event.at == Time(5 * MS))
    ));
    assert!(finished.events().iter().any(|event| matches!(&event.kind, EventKind::ResponseSent { id: 1, due, .. } if *due == Time(16 * MS))));
    assert_eq!(velocity(&split, Time(5 * MS)), 0.0);
    assert!((velocity(&split, Time(6 * MS)) - 0.003).abs() < 1e-14);
    let recording = finished.recording();
    split.restore(&snapshot).unwrap();
    split.run_until(horizon).unwrap();
    assert_eq!(control::run(&split).unwrap().fingerprint(), fingerprint);
    let mut repeated = runtime(rows.clone());
    install(&mut repeated, &cfg, horizon, &[]);
    repeated.run_until(horizon).unwrap();
    assert_eq!(control::run(&repeated).unwrap().fingerprint(), fingerprint);
    let mut replay = runtime(rows.clone());
    control::install_replay(
        &mut replay,
        &document,
        &Assets,
        spec(horizon, &[]),
        recording.clone(),
    )
    .unwrap();
    assert_eq!(control::worker_ids(&replay).count(), 0);
    replay.run_until(horizon).unwrap();
    control::verify_replay(&replay).unwrap();
    assert_eq!(control::run(&replay).unwrap().origin(), RunOrigin::Replay);
    let mut wrong = runtime(rows);
    let wrong_spec = RunSpec {
        criteria: b"modified",
        ..spec(horizon, &[])
    };
    assert!(
        control::install_replay(&mut wrong, &document, &Assets, wrong_spec, recording).is_err()
    );
}

#[test]
fn external_delays_checked_at_delivery_for_ms_and_tens_of_seconds() {
    // Просроченный ответ за 4 мс с лимитом 3 мс никогда не появится на нижнем уровне.
    let mut late = runtime(vec![(Time::ZERO, 2.0, true, Time::ZERO)]);
    install(
        &mut late,
        &external(1_100_000, 2_100_000, 100_000, 3 * MS, false),
        Time(9 * MS),
        &[],
    );
    assert!(late.try_snapshot().is_err());
    assert!(late.fork().is_err());
    late.run_until(Time(9 * MS)).unwrap();
    let run = control::run(&late).unwrap();
    assert!(run.events().iter().any(|e| matches!(&e.kind, EventKind::OutputInvalidated { path, reason: Rejection::Expired } if path == "верх" && e.at == Time(4 * MS))));
    assert!(
        !run.events()
            .iter()
            .any(|e| matches!(&e.kind, EventKind::CommandApplied { .. }))
    );
    assert_eq!(velocity(&late, Time(9 * MS)), 0.0);

    // 12 с ± 1 мс, выход квантуется сеткой верхнего уровня с периодом 4 мс.
    let mut long = runtime(vec![(Time::ZERO, 2.0, true, Time::ZERO)]);
    install(
        &mut long,
        &external(1_100_000, 12_000 * MS, MS, 14_000 * MS, true),
        Time(12_009 * MS),
        &[],
    );
    assert_eq!(
        long.run_until(Time(12_009 * MS)).unwrap(),
        Status::Running,
        "{:?}",
        long.failure()
    );
    let run = control::run(&long).unwrap();
    let exchange_at = run
        .events()
        .iter()
        .find_map(|event| match event.kind {
            EventKind::RequestDelivered { id: 0, .. } => Some(event.at),
            _ => None,
        })
        .unwrap();
    assert_eq!(exchange_at, Time(1_250_000));
    let delivered_at = run
        .events()
        .iter()
        .find_map(|event| match event.kind {
            EventKind::ResponseDelivered { id: 0, .. } => Some(event.at),
            _ => None,
        })
        .unwrap();
    let delay = delivered_at.0 - exchange_at.0;
    assert!((11_999 * MS..12_005 * MS).contains(&delay));
    assert_eq!(delivered_at.0 % (4 * MS), 0);
    assert_eq!(velocity(&long, Time(delivered_at.0 + MS)), 0.0);
    assert!((velocity(&long, Time(delivered_at.0 + 2 * MS)) - 0.002).abs() < 1e-14);
}

#[test]
fn failed_external_exchange_cannot_finish_a_good_run() {
    let mut rt = runtime(vec![(Time::ZERO, -1.0, true, Time::ZERO)]);
    install(&mut rt, &external(0, 0, 0, 3 * MS, true), Time(9 * MS), &[]);
    assert_eq!(
        rt.run_until(Time(9 * MS)).unwrap(),
        Status::ComponentFailure
    );
    let run = control::run(&rt).unwrap();
    assert!(!run.reached_horizon());
    assert!(
        run.events()
            .iter()
            .any(|e| matches!(&e.kind, EventKind::Failure { .. }))
    );
    assert!(
        !run.events()
            .iter()
            .any(|e| matches!(&e.kind, EventKind::CommandApplied { .. }))
    );
}

fn board(name: &str, catalog: &str, image: &str, config: Option<&str>, rate: u64) -> String {
    let config = config
        .map(|name| format!("config='{name}'\n"))
        .unwrap_or_default();
    format!(
        "\n[[board]]\nname='{name}'\ncatalog='{catalog}'\nimage='{image}'\nrate_hz={rate}\n{config}{SOURCE}\n"
    )
}

fn packet(name: &str, kind: &str, params: &str, age: u64) -> String {
    format!(
        "\n[[packet]]\nname='{name}'\nkind='{kind}'\ncapacity=16\nmax_age_ns={age}\n{params}\n{SOURCE}\n"
    )
}

fn transmission(route: &str, at: u64, payload: &str) -> String {
    format!("\n[[transmission]]\nroute='{route}'\nat_ns={at}\nbytes={payload}\n{SOURCE}\n")
}

#[test]
fn задержка_провода_измеряется_до_такта_приёмника() {
    use std::collections::BTreeMap;
    let mut config = "version=1\n".to_owned();
    config.push_str(&board("плата", "vmcu-c3", "плата", Some("байты"), 4000));
    // Тот же вход АЦП, который использует прошивка в опыте UART/PWM.
    config.push_str(&format!(
        r#"
[[board_input]]
name='ADC'
board='плата'
signal='измерение'
unit='m'
kind='adc'
port=0
max_age_ns=2000000
scale=1
offset=0
fallback=0
{SOURCE}
"#
    ));
    config.push_str(&packet(
        "провод",
        "uart",
        "to_board='плата'\nto_port=0\ndelay_ns=125000",
        2 * MS,
    ));
    config.push_str(&transmission("провод", MS / 2, "[0x6a]"));
    let channel = "channel:провод";
    let mut rt = runtime(vec![(Time::ZERO, 1.65, true, Time::ZERO)]);
    install(&mut rt, &config, Time(2 * MS), &[]);
    assert_eq!(rt.channel_delay_ns(channel), Some(125_000));
    // Отправка на 0,5 мс, замена задержки на 0,375 мс, приёмник тикает через 0,25 мс.
    rt.set_channel_delays(&BTreeMap::from([(channel.into(), 375_000)]))
        .unwrap();
    assert_eq!(rt.channel_delay_ns(channel), Some(375_000));
    assert_eq!(
        rt.run_until(Time(7 * MS / 8)).unwrap(),
        Status::Running,
        "{:?}",
        rt.failure()
    );
    assert!(!rt.channel_delay_observed(channel));
    assert_eq!(
        rt.run_until(Time(MS)).unwrap(),
        Status::Running,
        "{:?}",
        rt.failure()
    );
    let run = control::run(&rt).unwrap();
    assert!(
        rt.channel_delay_observed(channel),
        "{:?}, {:?}",
        rt.failure(),
        run.events()
    );
    assert!(run.events().iter().any(|event| {
        matches!(&event.kind, EventKind::Packet { route, event: packet }
            if route == "провод" && packet.at == Time(7 * MS / 8))
    }));
    assert!(run.events().iter().any(|event| {
        event.at == Time(MS)
            && matches!(&event.kind, EventKind::PacketDelivered { route, .. } if route == "провод")
    }));
}

#[test]
fn scheduled_radio_delivers_at_configured_slots() {
    let mut cfg = String::from("version=1\n");
    cfg.push_str(&board("плата", "vmcu-c3", "плата", Some("байты"), 4000));
    cfg.push_str(&format!(
        r#"
[[board_input]]
name='ADC'
board='плата'
signal='измерение'
unit='m'
kind='adc'
port=0
max_age_ns=2000000
scale=1
offset=0
fallback=0
{SOURCE}
"#
    ));
    cfg.push_str(&packet(
        "вход",
        "scheduled_radio",
        "to_board='плата'\nto_port=0\nrssi_dbm=-60\nmax_bytes=20\nslots_asset='слоты'",
        MS,
    ));
    assert!(
        ControlConfig::parse(
            &cfg.replace("slots_asset='слоты'", "slots_asset='слоты'\ndelay_ns=0")
        )
        .is_err()
    );
    let invalid = ControlConfig::parse(&cfg.replace("'слоты'", "'ошибочные слоты'")).unwrap();
    let mut rt = runtime(vec![]);
    let error = control::install(&mut rt, &invalid, &Assets, spec(Time(2 * MS), &[])).unwrap_err();
    assert!(error.contains("нарушен порядок слотов"), "{error}");

    // Пакет ровно на границе попадает в первый слот, на 1 нс позже — во второй.
    cfg.push_str(&transmission("вход", MS / 4, "[1]"));
    cfg.push_str(&transmission("вход", MS / 4 + 1, "[2]"));
    cfg.push_str(&transmission("вход", MS + 1, "[3]"));
    install(&mut rt, &cfg, Time(2 * MS), &[]);
    assert_eq!(
        rt.run_until(Time(MS)).unwrap(),
        Status::Running,
        "{:?}",
        rt.failure()
    );
    let run = control::run(&rt).unwrap();
    let actual: Vec<_> = run
        .events()
        .iter()
        .filter_map(|event| match &event.kind {
            EventKind::PacketDelivered {
                route,
                bytes,
                sent_at,
                ..
            } if route == "вход" => Some((bytes.as_slice(), sent_at.0, event.at.0)),
            _ => None,
        })
        .collect();
    assert_eq!(
        actual,
        [(&[1][..], MS / 4, MS / 2), (&[2][..], MS / 4 + 1, MS)]
    );
    assert_eq!(
        rt.run_until(Time(2 * MS)).unwrap(),
        Status::ComponentFailure
    );
    assert!(rt.failure().unwrap().contains("не содержит слота"));
}

#[test]
fn uart_radio_adc_and_pwm_close_the_board_to_physics_path() {
    let mut cfg = String::from("version=1\n");
    cfg.push_str(&board("плата", "vmcu-c3", "плата", Some("байты"), 4000));
    cfg.push_str(&packet(
        "вход UART",
        "uart",
        "to_board='плата'\nto_port=0\ndelay_ns=125000",
        20 * MS,
    ));
    cfg.push_str(&packet(
        "выход UART",
        "uart",
        "from_board='плата'\nfrom_port=0\ndelay_ns=125000",
        20 * MS,
    ));
    cfg.push_str(&packet("вход радио", "ble", "to_board='плата'\nto_port=0\ninterval_ns=7500000\nloss=0\nrssi_dbm=-64\nattempts=3\nmax_bytes=20", 20 * MS));
    cfg.push_str(&packet("выход радио", "wifi", "from_board='плата'\nfrom_port=0\nattempt_ns=1000000\nloss=0\nrssi_dbm=-55\nattempts=3\nmax_bytes=20", 20 * MS));
    cfg.push_str(&transmission("вход UART", 500_000, "[0x6a]"));
    cfg.push_str(&transmission("вход радио", 4 * MS, "[0xd3]"));
    cfg.push_str(&format!(
        r#"
[[board_input]]
name='ADC'
board='плата'
signal='измерение'
unit='m'
kind='adc'
port=0
max_age_ns=20000000
scale=1
offset=0
fallback=0
{SOURCE}

[[driver]]
name='мост'
board='плата'
target='сила'
target_kind='effort'
unit='N'
quantity='duty'
pin=0
scale=4
offset=0
max_age_ns=1000000
{SOURCE}
"#
    ));
    let mut rt = runtime(vec![(Time::ZERO, 1.65, true, Time::ZERO)]);
    install(&mut rt, &cfg, Time(12 * MS), &[]);
    assert_eq!(
        rt.run_until(Time(12 * MS)).unwrap(),
        Status::Running,
        "{:?}",
        rt.failure()
    );
    let run = control::run(&rt).unwrap();
    assert!(run.events().iter().any(|event| matches!(&event.kind, EventKind::BoardInput { input: BoardInput::Adc { volts: 1.65, .. }, .. } if event.at == Time::ZERO)));
    let delivered = run
        .events()
        .iter()
        .filter_map(|event| match &event.kind {
            EventKind::PacketDelivered {
                route,
                bytes,
                sent_at,
                ..
            } => Some((route.as_str(), bytes.as_slice(), *sent_at, event.at)),
            _ => None,
        })
        .collect::<Vec<_>>();
    assert!(delivered.contains(&("вход UART", &[0x6a], Time(500_000), Time(750_000))));
    assert!(delivered.contains(&("вход радио", &[0xd3], Time(4 * MS), Time(7_500_000))));
    assert!(
        delivered
            .iter()
            .any(|(route, bytes, sent, received)| *route == "выход UART"
                && *bytes == b"j"
                && received.0 - sent.0 == 125_000)
    );
    assert!(
        delivered
            .iter()
            .any(|(route, bytes, sent, received)| *route == "выход радио"
                && *bytes == [0xd3, 192]
                && received.0 - sent.0 == MS)
    );
    // 1.65 В -> 2048 через 12-битную шкалу 3.3 В. ELF сдвигает код
    // влево на 4, PWM делит на 65535, преобразование задаёт 4 Н на единицу.
    let force = 4.0 * 32768.0 / 65535.0;
    assert!((velocity(&rt, Time(2 * MS)) - force * 0.001 / 2.0).abs() < 1e-14);
}

#[test]
fn modelled_registers_match_independent_protocol_words_and_timing() {
    let mut cfg = String::from("version=1\n");
    cfg.push_str(&board("плата", "esp32-c3", "шины", None, 40_000));
    for (name, model, bus, cs) in [
        ("первая", "i2c", 0, ""),
        ("вторая", "spi", 2, "chip_select=2"),
    ] {
        cfg.push_str(&format!("\n[[bus]]\nname='{name}'\nboard='плата'\nsignal='фаза'\nmodel='{model}'\nbus={bus}\n{cs}\nmax_age_ns=3000000\n{SOURCE}\n"));
    }
    let mut rt = Runtime::load(scene()).unwrap();
    let phase = rt.register_observation("фаза", "rad").unwrap();
    rt.register_sensor(
        "угол",
        Rate::hz(1000).unwrap(),
        Scripted {
            id: phase,
            rows: vec![(Time::ZERO, std::f64::consts::FRAC_PI_2, true, Time::ZERO)],
        },
    )
    .unwrap();
    install(&mut rt, &cfg, Time(3 * MS), &[]);
    assert_eq!(
        rt.run_until(Time(3 * MS)).unwrap(),
        Status::Running,
        "{:?}",
        rt.failure()
    );
    let run = control::run(&rt).unwrap();
    let mut i2c = None;
    let mut spi = Vec::new();
    let mut received = Vec::new();
    for event in run.events() {
        let EventKind::BoardOutput { event, .. } = &event.kind else {
            continue;
        };
        match &event.output {
            BoardOutput::I2c(request) => {
                assert_eq!(
                    (request.address, &request.write[..], request.repeated_start),
                    (0x36, &[0x0c][..], true)
                );
                i2c = Some(event.cycle);
            }
            BoardOutput::I2cDone {
                status: TransferStatus::Complete,
                bytes,
                ..
            } => {
                // Четверть оборота 4096/4 = 1024 и 45 бит по 800 тактов + условия START/STOP.
                assert_eq!(bytes, &[0x04, 0x00]);
                assert_eq!(event.cycle - i2c.unwrap(), 10 + 45 * 800 + 24 + 40);
            }
            BoardOutput::Spi(request) => spi.push((request.id, event.cycle)),
            BoardOutput::SpiDone {
                id,
                status: TransferStatus::Complete,
                bytes,
                ..
            } => {
                let start = spi
                    .iter()
                    .find(|&&(start_id, _)| start_id == *id)
                    .unwrap()
                    .1;
                assert_eq!(event.cycle - start, (31 + 5 + 7) * 40);
                received.push(bytes.clone());
            }
            _ => (),
        }
    }
    // Пайплайн: первый обмен отдаёт начальное слово, второй — 16384/4 = 0x1000
    // с установленным старшим битом чётности.
    assert!(i2c.is_some());
    assert_eq!(received, [vec![0x00, 0x00], vec![0x90, 0x00]]);
}

#[test]
fn hardware_watchdog_keeps_ticking_during_scheduled_cpu_pause() {
    let mut cfg = String::from("version=1\n");
    cfg.push_str(&board(
        "плата",
        "vmcu-c3",
        "плата",
        Some("watchdog"),
        200_000,
    ));
    cfg.push_str(&board_adc("плата", 1.65));
    add_fault(
        &mut cfg,
        "остановка CPU",
        "board:плата",
        20_000,
        100_000,
        "kind='pause'",
    );
    let mut rt = runtime(vec![]);
    install(&mut rt, &cfg, Time(125_000), &[]);
    assert_eq!(
        rt.run_until(Time(125_000)).unwrap(),
        Status::Running,
        "{:?}",
        rt.failure()
    );
    let run = control::run(&rt).unwrap();
    let events = run.events();
    assert!(events.iter().any(|e| matches!(&e.kind, EventKind::BoardInput { input: BoardInput::Execution { enabled: false }, .. } if e.at == Time(20_000))));
    assert!(events.iter().any(
        |e| matches!(&e.kind, EventKind::BoardOutput { produced_at, event, .. }
        if matches!(event.output, BoardOutput::Reset(sim_boards::ResetCause::Watchdog))
            && Time(20_000) < *produced_at && *produced_at < Time(100_000))
    ));
}

fn board_adc(board: &str, volts: f64) -> String {
    format!(
        "\n[[board_input]]\nname='ADC'\nboard='{board}'\nsignal='измерение'\nunit='m'\nkind='adc'\nport=0\nmax_age_ns=0\nscale=0\noffset={volts}\nfallback={volts}\n{SOURCE}\n"
    )
}

#[test]
fn custom_controller_only_sees_bound_input_and_cannot_submit_nan_or_other_output() {
    #[derive(Clone)]
    struct Custom(ChannelId, f64);
    impl sim_control::Controller for Custom {
        fn step(
            &mut self,
            input: &sim_signal::Observation,
            _: &mut Rng,
        ) -> Result<Command, sim_control::ControlError> {
            assert_eq!(input.samples.len(), 1);
            assert_eq!(input.latest(SignalId(0)), Some(2.0));
            assert!(input.frames.is_empty());
            Ok(Command {
                targets: vec![(self.0, self.1)],
            })
        }
    }

    let document = ControlConfig::parse(&feedback(MS, "effort")).unwrap();
    for (id, number, expected) in [
        (ChannelId(2), f64::NAN, "неконечный"),
        (ChannelId(1), 8.0, "неизвестный"),
    ] {
        let mut rt = runtime(vec![(Time::ZERO, 2.0, true, Time::ZERO)]);
        let secret = rt.register_observation("закрытый", "m").unwrap();
        rt.register_sensor(
            "другой датчик",
            Rate::hz(1000).unwrap(),
            Scripted {
                id: secret,
                rows: vec![(Time::ZERO, 5.0, true, Time::ZERO)],
            },
        )
        .unwrap();
        let mut bindings = control::ControllerBindings::default();
        bindings.bind("верхний", Custom(id, number)).unwrap();
        control::install_with_controllers(
            &mut rt,
            &document,
            &Assets,
            spec(Time(MS), &[]),
            bindings,
        )
        .unwrap();
        assert_eq!(rt.run_until(Time(MS)).unwrap(), Status::ComponentFailure);
        assert!(rt.failure().unwrap().contains(expected));
        let run = control::run(&rt).unwrap();
        assert_eq!(run.origin(), RunOrigin::ProgrammaticController);
        assert!(
            run.events()
                .iter()
                .all(|e| !matches!(&e.kind, EventKind::CommandApplied { .. }))
        );
    }
}

#[test]
fn delayed_model_report_after_expiry_is_an_unattributed_observation() {
    #[derive(Clone)]
    struct DelayedModel {
        ack: TruthId,
        initial: Option<f64>,
    }
    impl Object for DelayedModel {
        fn update(
            &mut self,
            truth: &Truth<'_>,
            command: &Command,
            _: &mut Rng,
            out: &mut Forces,
        ) -> Result<(), String> {
            if truth.time() == Time::ZERO {
                self.initial = command.targets.first().map(|&(_, value)| value);
            } else if truth.time() == Time(2 * MS) {
                out.publish(
                    self.ack,
                    self.initial.expect("получена команда в начальный момент"),
                );
            }
            Ok(())
        }
    }
    let mut rt = runtime(vec![(Time::ZERO, 4.0, true, Time::ZERO)]);
    let ack = rt.register_truth("принято", "N").unwrap();
    rt.register_object(
        "модель",
        Rate::hz(1000).unwrap(),
        &[ChannelId(0)],
        DelayedModel { ack, initial: None },
    )
    .unwrap();
    let mut cfg = feedback(0, "drive").replace("max_age_ns=4000000", "max_age_ns=500000");
    cfg.push_str("ack='принято'\nfallback=0\n");
    install(
        &mut rt,
        &cfg,
        Time(3 * MS),
        &[ModelPort::Drive {
            channel: ChannelId(0),
            accepted: ack,
        }],
    );
    rt.run_until(Time(3 * MS)).unwrap();
    let run = control::run(&rt).unwrap();
    assert!(run.events().iter().any(
        |e| matches!(&e.kind, EventKind::CommandPresented { command, .. } if command.value == 4.0)
    ));
    assert!(run.events().iter().any(|e| matches!(&e.kind, EventKind::ModelAcceptanceUnattributed { value: 4.0, .. } if e.at == Time(2 * MS))));
    assert!(
        !run.events()
            .iter()
            .any(|e| matches!(&e.kind, EventKind::CommandApplied { .. }))
    );
}

#[test]
fn assert_uart_serialization_matches_8n1_on_actual_mcu() {
    let mut cfg = String::from("version=1\n");
    cfg.push_str(&board("плата", "esp32-c3", "uart", None, 40_000));
    cfg.push_str(&packet(
        "приём",
        "uart",
        "to_board='плата'\nto_port=0\ndelay_ns=15000",
        3 * MS,
    ));
    cfg.push_str(&packet(
        "передача",
        "uart",
        "from_board='плата'\nfrom_port=0\ndelay_ns=0",
        3 * MS,
    ));
    cfg.push_str(&transmission("приём", 100_000, "[0x37, 0x52]"));
    let mut rt = runtime(vec![]);
    install(&mut rt, &cfg, Time(MS), &[]);
    assert_eq!(
        rt.run_until(Time(MS)).unwrap(),
        Status::Running,
        "{:?}",
        rt.failure()
    );
    let run = control::run(&rt).unwrap();
    let uart_input = run
        .events()
        .iter()
        .find_map(|e| match &e.kind {
            EventKind::BoardInput {
                cycle,
                input: BoardInput::Uart { bytes, .. },
                ..
            } => Some((e.at, *cycle, bytes.as_slice())),
            _ => None,
        })
        .unwrap();
    assert_eq!(uart_input, (Time(125_000), 10_000, &[0x37, 0x52][..]));
    let uart_output: Vec<_> = run
        .events()
        .iter()
        .filter_map(|e| match &e.kind {
            EventKind::BoardOutput {
                event, produced_at, ..
            } => match event.output {
                BoardOutput::Uart { port: 0, byte } => Some((*produced_at, event.cycle, byte)),
                _ => None,
            },
            _ => None,
        })
        .collect();
    assert_eq!(
        uart_output
            .iter()
            .map(|&(_, _, byte)| byte)
            .collect::<Vec<_>>(),
        [0x37, 0x52]
    );
    // Из сборки стенда: 1 Мбод, 8N1. От приёма до эха не меньше двух кадров.
    assert!(uart_output[0].0.0 >= 125_000 + 20_000);
    assert_eq!(uart_output[1].1 - uart_output[0].1, 800);
}

#[test]
fn scheduled_direct_stimulus_is_quantized_and_marked_separately() {
    let mut rt = runtime(vec![]);
    let document = format!(
        "version=1\n[[stimulus]]\nat_ns=500000\ntarget='сила'\nunit='N'\nvalue=4\n{SOURCE}"
    );
    install(&mut rt, &document, Time(2 * MS), &[]);
    rt.run_until(Time(2 * MS)).unwrap();
    let run = control::run(&rt).unwrap();
    assert!(run.paths().is_empty());
    assert_eq!(velocity(&rt, Time(MS)), 0.0);
    assert_eq!(velocity(&rt, Time(2 * MS)), 0.002);
    assert!(run.events().iter().any(|e| matches!(&e.kind, EventKind::CommandApplied { command, effective_at, .. } if command.origin == CommandOrigin::Stimulus && *effective_at == Time(MS))));
}

#[test]
fn gpio_and_reset_configuration_are_applied_on_board_exchange_boundary() {
    let mut cfg = String::from("version=1\n");
    cfg.push_str(&board("плата", "vmcu-c3", "плата", Some("байты"), 200_000));
    cfg.push_str(&board_adc("плата", 1.65));
    cfg.push_str(&format!(
        r#"
[[board_action]]
id='подача уровня'
board='плата'
at_ns=1001
kind='gpio'
pin=0
high=true
{SOURCE}
[[board_action]]
id='новая конфигурация'
board='плата'
at_ns=31001
kind='reset'
config='watchdog'
{SOURCE}
"#
    ));
    let mut rt = runtime(vec![]);
    install(&mut rt, &cfg, Time(75_000), &[]);
    assert_eq!(
        rt.run_until(Time(75_000)).unwrap(),
        Status::Running,
        "{:?}",
        rt.failure()
    );
    let run = control::run(&rt).unwrap();
    let actions: Vec<_> = run
        .events()
        .iter()
        .filter_map(|event| match &event.kind {
            EventKind::BoardActionApplied { id, cycle, .. } => {
                Some((id.as_str(), event.at, *cycle))
            }
            _ => None,
        })
        .collect();
    // Плата 160 МГц, обмен раз в 5 мкс. Сброс в 35 мкс = 5600 тактов.
    assert_eq!(
        actions,
        [
            ("подача уровня", Time(5_000), 800),
            ("новая конфигурация", Time(35_000), 5600)
        ]
    );
    assert!(run.events().iter().any(|entry| matches!(&entry.kind, EventKind::BoardOutput { event, produced_at, .. }
        if event.output == BoardOutput::Reset(sim_boards::ResetCause::Watchdog) && Time(65_000) < *produced_at)));
}

#[test]
fn bus_open_fault_returns_nack_to_the_running_image() {
    let mut cfg = String::from("version=1\n");
    cfg.push_str(&board("плата", "esp32-c3", "шины", None, 40_000));
    cfg.push_str(&format!("\n[[bus]]\nname='первая'\nboard='плата'\nsignal='фаза'\nmodel='i2c'\nbus=0\nmax_age_ns=3000000\n{SOURCE}\n"));
    add_fault(&mut cfg, "обрыв шины", "bus:первая", 0, MS, "kind='drop'");
    let mut rt = Runtime::load(scene()).unwrap();
    let phase = rt.register_observation("фаза", "rad").unwrap();
    rt.register_sensor(
        "угол",
        Rate::hz(1000).unwrap(),
        Scripted {
            id: phase,
            rows: vec![(Time::ZERO, std::f64::consts::FRAC_PI_2, true, Time::ZERO)],
        },
    )
    .unwrap();
    install(&mut rt, &cfg, Time(2 * MS), &[]);
    assert_eq!(
        rt.run_until(Time(2 * MS)).unwrap(),
        Status::Running,
        "{:?}",
        rt.failure()
    );
    let run = control::run(&rt).unwrap();
    assert!(run.events().iter().any(|entry| matches!(&entry.kind, EventKind::BoardOutput { event, .. }
        if matches!(&event.output, BoardOutput::I2cDone { bytes, status: TransferStatus::Nack, .. } if bytes.is_empty()))));
    assert!(run.events().iter().any(
        |entry| matches!(&entry.kind, EventKind::BoardOutput { event, .. }
        if matches!(event.output, BoardOutput::Uart { port: 0, byte: b'N' }))
    ));
    assert!(!run.events().iter().any(
        |entry| matches!(&entry.kind, EventKind::BoardOutput { event, .. }
        if matches!(event.output, BoardOutput::Spi(_)))
    ));
}
