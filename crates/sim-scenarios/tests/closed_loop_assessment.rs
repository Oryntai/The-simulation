#[path = "../../sim-task/tests/support/mod.rs"]
mod bench;
mod support;

use sim_boards::{BoardId, Connection, new_board};
use sim_device::{Device, FirmwareSettings, Wiring};
use sim_scenarios::blueprint::BuildVerdict;
use sim_scenarios::device_assess::{
    ClosedLoopInput, DeviceReport, Outcome, Readiness, assess_device_in,
    assess_device_with_closed_loop, measurement_stamp,
};
use sim_task::{Assessor, Evidence, Fingerprint, LegacyDrive, RunResult, Task};

// Граница стенда из fixtures задачи: поворот до 1 мрад, реакция за два такта.
const SAFE_TRAVEL: f64 = 0.001;
const DEADLINE: u64 = 2 * bench::TICK;

/// DS40002198C и AVR Instruction Set Manual: I/O DDRD=0x0a, PIND=0x09, PORTD=0x0b.
/// 16-разрядные команды AVRe настраивают D2 как выход и копируют на него D3.
fn firmware_bytes() -> Vec<u8> {
    let code: [u16; 7] = [
        0x9a00 | 0x0a << 3 | 2, // SBI DDRD,2
        0x9900 | 0x09 << 3 | 3, // SBIC PIND,3
        0xc002,                 // RJMP высокий уровень, слово 5
        0x9800 | 0x0b << 3 | 2, // CBI PORTD,2
        0xcffc,                 // RJMP на слово 1
        0x9a00 | 0x0b << 3 | 2, // SBI PORTD,2
        0xcffa,                 // RJMP на слово 1
    ];
    let data: Vec<_> = code.into_iter().flat_map(u16::to_le_bytes).collect();
    // Запись Intel HEX по адресу 0 и отдельный конец файла.
    let mut line = format!(":{:02X}000000", data.len());
    let mut checksum = data.len() as u8;
    for byte in data {
        use std::fmt::Write;
        write!(line, "{byte:02X}").unwrap();
        checksum = checksum.wrapping_add(byte);
    }
    line.push_str(&format!("{:02X}\n:00000001FF\n", checksum.wrapping_neg()));
    line.into_bytes()
}

fn neutral_device(dir: &support::TempDir, image: &[u8]) -> Device {
    let mut device = support::device();
    device.name = "Каретка на стенде".into();
    device.links[0].joint = support::moving().links[0].joint.clone();
    let electronics = &mut device.electronics;
    electronics.board = "atmega328p".into();
    electronics.firmware = "bench.hex".into();
    electronics.connections.push(Wiring(Connection::Motor {
        pin: 2,
        direct: false,
    }));
    electronics.firmware_cases = [true, false]
        .into_iter()
        .map(|level| FirmwareSettings {
            boot: level.then_some(true),
            input_pin: Some(3),
            input_high: Some(level),
            output_pin: Some(2),
            output_high: Some(level),
            reason: Some("Выход повторяет вход стенда".into()),
            ..Default::default()
        })
        .collect();
    std::fs::write(dir.0.join(&electronics.firmware), image).unwrap();
    assert!(device.validate().is_empty(), "{:?}", device.validate());
    device
}

fn expected(device: &Device, dir: &support::TempDir, image: &[u8], max: f64, pin: u32) -> Assessor {
    let mut inputs = bench::inputs();
    inputs.program = Fingerprint::of(image);
    inputs.firmware_image = Some(inputs.program);
    inputs.legacy_design = Some(measurement_stamp(device, &dir.0));
    let mut topology = bench::rotary_topology();
    topology.paths[0].legacy = Some(LegacyDrive { pin, link: 0 });
    topology.paths[0].program = inputs.program;
    Assessor::new(
        Task::parse(&bench::rotary_document(max, DEADLINE)).unwrap(),
        topology,
        bench::rotary_world(&device.links[0].joint.actuator),
        inputs,
    )
    .unwrap()
}

fn run(assessor: &Assessor, image: &[u8], cases: usize) -> Vec<RunResult> {
    assessor
        .required_cases()
        .iter()
        .take(cases)
        .map(|case| {
            assessor
                .execute(case, |runtime, plan, sink| {
                    let mut board = new_board(BoardId::Atmega328p);
                    board.load(image).map_err(|error| error.to_string())?;
                    let program = Fingerprint::of(image);
                    bench::install_with(
                        runtime,
                        plan,
                        sink,
                        bench::Trial::Normal,
                        program,
                        move |measurement| {
                            // На стенде положительная ошибка выдаёт уровень 1.
                            // Исполнительная модель переводит его в момент 2 Н·м.
                            board.set_pin_input(3, measurement > 0.0);
                            board
                                .run_for(sim_signal::Time(bench::TICK).as_secs())
                                .unwrap();
                            if board.pin_output(2).unwrap() {
                                2.0
                            } else {
                                0.0
                            }
                        },
                    )?;
                    let mut installed = assessor.identity().inputs.clone();
                    installed.program = program;
                    installed.firmware_image = Some(program);
                    Ok(installed)
                })
                .unwrap()
        })
        .collect()
}

fn assess(
    device: &Device,
    dir: &support::TempDir,
    expected: &Assessor,
    evidence: &Evidence,
) -> DeviceReport {
    assess_device_with_closed_loop(device, &dir.0, ClosedLoopInput { expected, evidence })
}

fn closed(report: &DeviceReport) -> Outcome {
    let found: Vec<_> = report
        .findings
        .iter()
        .filter(|finding| finding.code == "closed_loop")
        .collect();
    assert_eq!(found.len(), 1, "{found:#?}");
    found[0].outcome
}

#[test]
fn execution_and_pin_checks_remain_separate_from_live_proof() {
    let dir = support::TempDir::new();
    let image = firmware_bytes();
    let device = neutral_device(&dir, &image);
    let expected = expected(&device, &dir, &image, SAFE_TRAVEL, 2);
    let old = assess_device_in(&device, &dir.0);
    assert_eq!(closed(&old), Outcome::NotAnalyzed);
    let separate: Vec<_> = old
        .findings
        .iter()
        .filter(|f| matches!(f.code, "firmware_execution" | "firmware_behavior"))
        .collect();
    assert_eq!(separate.len(), 3, "{separate:#?}");
    assert!(
        separate.iter().all(|f| f.outcome == Outcome::Pass),
        "{separate:#?}"
    );

    let partial = expected.assess(&run(&expected, &image, 1));
    assert_eq!(partial.cases()[0].outcome, sim_task::Outcome::Pass);
    assert_eq!(
        closed(&assess(&device, &dir, &expected, &partial)),
        Outcome::NotAnalyzed
    );

    let complete = expected.assess(&run(&expected, &image, expected.required_cases().len()));
    for row in complete.cases() {
        assert_eq!(row.outcome, sim_task::Outcome::Pass, "{row:#?}");
    }
    // При 2 Н·м, 1 кг·м² и 1 мс первый угол 2 мкрад, за 8 шагов
    // q = dt²·2·(1+...+8), то есть 72 мкрад.
    let metric = &complete.cases()[0].metrics[0];
    assert!((metric.max - 0.000_072).abs() < 1e-14);
    let actual = assess(&device, &dir, &expected, &complete);
    // Исполнение через пользовательский замыкатель не связывает вывод
    // установленной платы с приводом, даже при полном наборе стендовых случаев.
    assert_eq!(closed(&actual), Outcome::NotAnalyzed);
    assert_eq!(
        actual
            .findings
            .iter()
            .filter(|f| matches!(f.code, "firmware_execution" | "firmware_behavior"))
            .collect::<Vec<_>>(),
        separate
    );
}

#[test]
fn binding_rejects_other_image_design_and_pin() {
    let dir = support::TempDir::new();
    let image = firmware_bytes();
    let mut device = neutral_device(&dir, &image);
    let first = expected(&device, &dir, &image, SAFE_TRAVEL, 2);
    let complete = first.assess(&run(&first, &image, first.required_cases().len()));
    assert_eq!(complete.outcome(), sim_task::Outcome::Pass);
    assert_eq!(
        closed(&assess(&device, &dir, &first, &complete)),
        Outcome::NotAnalyzed
    );

    let other_pin = expected(&device, &dir, &image, SAFE_TRAVEL, 3);
    assert_eq!(
        closed(&assess(&device, &dir, &other_pin, &complete)),
        Outcome::NotAnalyzed
    );
    device.links[0].length_mm += 1.0; // Изменили геометрию стенда на 1 мм.
    assert_eq!(
        closed(&assess(&device, &dir, &first, &complete)),
        Outcome::NotAnalyzed
    );
    device.links[0].length_mm -= 1.0;

    let mut changed = image.clone();
    changed.push(b'\n'); // Исполняемые команды те же, байты образа другие.
    std::fs::write(dir.0.join(&device.electronics.firmware), &changed).unwrap();
    assert_eq!(
        closed(&assess(&device, &dir, &first, &complete)),
        Outcome::NotAnalyzed
    );
}

#[test]
fn measured_violation_rejects_even_with_other_cases_missing() {
    let dir = support::TempDir::new();
    let image = firmware_bytes();
    let device = neutral_device(&dir, &image);
    let expected = expected(&device, &dir, &image, 0.000_001, 2); // Ограничение ниже 2 мкрад на первом шаге.
    let proof = expected.assess(&run(&expected, &image, 1));
    assert_eq!(proof.outcome(), sim_task::Outcome::Fail);
    assert!(
        proof
            .cases()
            .iter()
            .any(|row| row.outcome == sim_task::Outcome::NotAnalyzed)
    );
    let report = assess(&device, &dir, &expected, &proof);
    assert_eq!(closed(&report), Outcome::Fail);
    assert_eq!(report.verdict(), BuildVerdict::Rejected);
    assert_eq!(report.readiness(), Readiness::Rejected);
}
