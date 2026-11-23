use std::ffi::OsString;
use std::process::{Child, Command, Stdio};
use std::sync::Arc;

use sim_control::transport::{
    ExternalController, ProcessSpec, ProcessTransport, SessionOptions, StreamTransport,
};
use sim_control::{
    ControlContract, ControlError, ControlRequest, ControlTransport, InputChannel, InputKind,
    OutputChannel, RequestId,
};
use sim_signal::{ChannelId, ImageFrame, Observation, PixelFormat, Sample, SignalId, Time};

const NS: u64 = 1_000_000_000; // Наносекунд в секунде.
const FIRST: RequestId = RequestId(7);
const INPUT: SignalId = SignalId(8);
const OUTPUT: ChannelId = ChannelId(9);
const OPTIONS: SessionOptions = SessionOptions {
    first_request: FIRST,
    seed: 19,
    max_message_bytes: 512,
};

fn contract(kind: InputKind, name: &str, unit: &str) -> ControlContract {
    ControlContract::new(
        vec![InputChannel {
            id: INPUT,
            name: name.into(),
            unit: unit.into(),
            kind,
        }],
        vec![OutputChannel {
            id: OUTPUT,
            name: "цель".into(),
            unit: "рад".into(),
            max_age_ns: 5,
        }],
    )
    .unwrap()
}

fn spec(args: &[&str]) -> ProcessSpec {
    ProcessSpec {
        program: env!("CARGO_BIN_EXE_sim-control-worker").into(),
        args: args.iter().map(OsString::from).collect(),
    }
}

fn pid_contract() -> ControlContract {
    contract(InputKind::Scalar { max_age_ns: 0 }, "ошибка", "рад")
}

fn pid_spec() -> ProcessSpec {
    // Kp=1, Ki=2. При ошибке 1 управляющее воздействие растёт на 2 в секунду.
    spec(&[
        "pid",
        "512",
        "8",
        "ошибка",
        "рад",
        "9",
        "цель",
        "рад",
        "5",
        "1",
        "0",
        "omit",
        "0",
        "1",
        "2",
        "0",
        "-10",
        "10",
        "-20",
        "20",
    ])
}

fn pid_request(id: u64, time: u64, value: f64, valid: bool) -> ControlRequest {
    ControlRequest {
        id: RequestId(id),
        observation: Observation {
            time: Time(time),
            samples: vec![Sample {
                signal: INPUT,
                value,
                at: Time(time),
                valid,
            }],
            frames: vec![],
        },
    }
}

#[test]
fn new_process_rebuilds_state_and_forks_independently() {
    let contract = pid_contract();
    let mut spawned = None;
    let transport = ProcessTransport::spawn_supervised(
        pid_spec(),
        contract.clone(),
        OPTIONS,
        Some(4096),
        |pid| spawned = Some(pid),
    )
    .unwrap();
    assert_eq!(spawned, Some(transport.worker_id()));
    let mut controller = ExternalController::new(transport, contract, FIRST);
    let first = controller
        .request(&pid_request(7, 0, 1., true).observation)
        .unwrap();
    assert_eq!(first.0.id, FIRST);
    assert_eq!(first.1.command.targets, [(OUTPUT, 1.)]);
    let second = controller
        .request(&pid_request(8, NS, 1., true).observation)
        .unwrap();
    assert_eq!(second.1.command.targets, [(OUTPUT, 3.)]);
    let mut other_pid = None;
    let mut fork = controller
        .fork_supervised(|pid| other_pid = Some(pid))
        .unwrap();
    assert!(other_pid.is_some() && other_pid != spawned);
    let observation = pid_request(9, 2 * NS, 1., true).observation;
    let left = controller.request(&observation).unwrap();
    let right = fork.request(&observation).unwrap();
    assert_eq!(left, right);
    assert_eq!(right.1.command.targets, [(OUTPUT, 5.)]);
    let divergent = fork
        .request(&pid_request(10, 3 * NS, 2., true).observation)
        .unwrap();
    assert_eq!(divergent.1.command.targets, [(OUTPUT, 10.)]);
    let original = controller
        .request(&pid_request(10, 3 * NS, 1., true).observation)
        .unwrap();
    assert_eq!(original.1.command.targets, [(OUTPUT, 7.)]);
}

#[test]
fn process_history_limit_and_worker_schema_mismatch_are_explicit() {
    let contract = pid_contract();
    let mut live = ProcessTransport::spawn(pid_spec(), contract.clone(), OPTIONS, Some(1)).unwrap();
    assert!(live.snapshot().is_ok());
    assert_eq!(
        live.exchange(&pid_request(7, 0, 1., true))
            .unwrap()
            .command
            .targets,
        [(OUTPUT, 1.)]
    );
    assert!(live.snapshot().is_err());
    assert_eq!(
        live.exchange(&pid_request(8, NS, 1., true))
            .unwrap()
            .command
            .targets,
        [(OUTPUT, 3.)]
    );
    let disabled = ProcessTransport::spawn(pid_spec(), contract, OPTIONS, None).unwrap();
    assert!(disabled.snapshot().is_err());
    let mismatched = self::contract(InputKind::Scalar { max_age_ns: 0 }, "ошибка", "градус");
    assert!(ProcessTransport::spawn(pid_spec(), mismatched, OPTIONS, None).is_err());
}

struct BenchWorker(Child);

impl Drop for BenchWorker {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

#[test]
fn same_worker_accepts_same_messages_from_simulation_and_bench() {
    let kind = InputKind::Image {
        format: PixelFormat::Rgb8,
        max_capture_age_ns: 10,
        max_delivery_age_ns: 2,
    };
    let contract = contract(kind, "кадр", "rgb");
    let spec = spec(&[
        "rgb", "512", "8", "кадр", "rgb", "9", "цель", "рад", "5", "1", "0", "omit", "10", "2",
        "x", "255", "0", "0", "255", "0", "0", "2", "bearing", "1", "2",
    ]);
    let live = ProcessTransport::spawn(spec.clone(), contract.clone(), OPTIONS, None).unwrap();
    let mut simulation = ExternalController::new(live, contract.clone(), FIRST);
    let child = Command::new(spec.program)
        .args(spec.args)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .unwrap();
    let mut child = BenchWorker(child);
    let mut bench = StreamTransport::open(
        child.0.stdout.take().unwrap(),
        child.0.stdin.take().unwrap(),
        contract.clone(),
        OPTIONS,
    )
    .unwrap();
    // Два красных пикселя x=2,3. Средняя координата 2,5, угол atan((2,5−1)/2).
    let frame = ImageFrame {
        signal: INPUT,
        captured: Time(0),
        at: Time(8),
        width: 4,
        height: 1,
        format: PixelFormat::Rgb8,
        data: Arc::from([0, 0, 0, 0, 0, 0, 255, 0, 0, 255, 0, 0]),
        valid: true,
    };
    let observation = Observation {
        time: Time(10),
        samples: vec![],
        frames: vec![frame.clone()],
    };
    let request = ControlRequest {
        id: FIRST,
        observation: observation.clone(),
    };
    let (sim_request, sim_response) = simulation.request(&observation).unwrap();
    assert_eq!(sim_request, request);
    let bench_response = bench.exchange(&request).unwrap();
    assert_eq!(sim_response, bench_response);
    assert!((sim_response.command.targets[0].1 - 0.75_f64.atan()).abs() < 1e-12);
    assert!(
        contract
            .validate_response(&request, &bench_response, Time(15))
            .is_ok()
    );
    assert!(
        contract
            .validate_response(&request, &bench_response, Time(16))
            .is_err()
    );

    let stale = Observation {
        time: Time(11),
        ..observation
    };
    let request = ControlRequest {
        id: RequestId(8),
        observation: stale.clone(),
    };
    let (_, sim_response) = simulation.request(&stale).unwrap();
    assert_eq!(sim_response, bench.exchange(&request).unwrap());
    assert!(sim_response.command.targets.is_empty()); // Данные о цели истекли.
    drop(bench);
    assert!(child.0.wait().unwrap().success());
}

#[test]
fn local_input_checks_precede_writes_and_worker_reports_execution_error() {
    let mut transport =
        ProcessTransport::spawn(pid_spec(), pid_contract(), OPTIONS, Some(4096)).unwrap();
    assert!(transport.exchange(&pid_request(8, 0, 1., true)).is_err());
    assert!(matches!(
        transport.exchange(&pid_request(7, 0, f64::NAN, false)),
        Err(ControlError::InvalidObservation(_))
    ));
    assert_eq!(
        transport
            .exchange(&pid_request(7, 0, 1., true))
            .unwrap()
            .command
            .targets,
        [(OUTPUT, 1.)]
    );
    assert!(matches!(
        transport.exchange(&pid_request(8, 0, 1., true)),
        Err(ControlError::Execution(_))
    ));
    assert!(transport.exchange(&pid_request(8, 1, 1., true)).is_err());
    assert!(transport.snapshot().is_err());
}

#[cfg(unix)]
#[test]
fn изолированное_окружение_запуска_работника() {
    // У переменной искусственное содержимое. Вложенный тест передаёт его
    // только родительскому процессу транспорта, не меняя глобальное окружение.
    const PROBE: &str = "SIM_TEST_PARENT_ONLY";
    if std::env::var_os(PROBE).is_none() {
        let output = std::process::Command::new(std::env::current_exe().unwrap())
            .args(["--exact", "изолированное_окружение_запуска_работника"])
            .env(PROBE, "проверочное значение")
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stdout)
        );
        return;
    }

    let worker = pid_spec();
    let mut args: Vec<OsString> = [
        "-c",
        r#"test "${SIM_TEST_PARENT_ONLY+x}" != x && exec "$@""#,
        "проверка",
    ]
    .into_iter()
    .map(Into::into)
    .collect();
    args.push(worker.program.into_os_string());
    args.extend(worker.args);
    let mut process = ProcessTransport::spawn(
        ProcessSpec {
            program: "/bin/sh".into(),
            args,
        },
        pid_contract(),
        OPTIONS,
        None,
    )
    .unwrap();
    let response = process
        .exchange(&pid_request(OPTIONS.first_request.0, 0, 1.0, true))
        .unwrap();
    assert_eq!(response.command.targets, vec![(OUTPUT, 1.0)]);
}
