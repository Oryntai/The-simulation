//! Эталон поведения контура «прошивка ↔ физика» до переноса на общую платформу.
//!
//! Текущий контур обязан повторять его побитово. Новый рантайм сравнивается с редкими
//! отсчётами этих файлов (допуск 1 %, см. docs/PLATFORM_SPEC.md, правило 10).
//! Перезаписать эталон можно только явно: `UPDATE_GOLDEN=1 cargo test --test golden_exo_loop`.

use std::fmt::Write as _;
use std::path::PathBuf;

use sim_scenarios::Scenario;
use sim_scenarios::exo_loop::{ExoLoop, LoopTrace, exo_ctrl_elf};

/// Шагов контура 25 мкс на редкий отсчёт: 5 мс.
const EVERY: u64 = 200;
const FRAME_S: f64 = 0.01;
const COLUMNS: &str = "t,theta,omega,current_a,motor_v,duty,joint_torque,human_torque,stop_torque,contact_n,battery_v,winding_c,housing_c,driver_on,reported_faults";

struct Case {
    name: &'static str,
    preset: usize,
    /// Какую кнопку отказа нажать и на какой секунде.
    action: Option<(f64, &'static str)>,
    seconds: f64,
}

const CASES: [Case; 11] = [
    Case { name: "assist", preset: 0, action: None, seconds: 3.0 },
    Case { name: "wrong_payload", preset: 1, action: None, seconds: 3.0 },
    Case { name: "hold_relaxed", preset: 2, action: None, seconds: 3.0 },
    Case { name: "cold", preset: 3, action: None, seconds: 3.0 },
    Case { name: "water", preset: 4, action: None, seconds: 3.0 },
    Case { name: "obstacle", preset: 5, action: None, seconds: 4.0 },
    Case { name: "fault_estop", preset: 2, action: Some((1.0, "estop")), seconds: 3.0 },
    Case { name: "fault_sensor", preset: 2, action: Some((1.0, "sensor")), seconds: 3.0 },
    Case { name: "fault_jam", preset: 2, action: Some((1.0, "jam")), seconds: 4.0 },
    Case { name: "fault_radio", preset: 2, action: Some((1.0, "radio")), seconds: 3.0 },
    Case { name: "fault_hang", preset: 2, action: Some((1.0, "hang")), seconds: 3.0 },
];

fn run(case: &Case) -> LoopTrace {
    let mut sc = ExoLoop::default();
    sc.apply_preset(case.preset);
    assert!(sc.firmware_error().is_none(), "{:?}", sc.firmware_error());
    sc.start_trace(EVERY);
    let frames = (case.seconds / FRAME_S).round() as usize;
    let at = case.action.map(|(t, _)| (t / FRAME_S).round() as usize);
    for frame in 0..frames {
        if Some(frame) == at {
            sc.run_action(case.action.expect("кнопка отказа").1);
        }
        sc.advance(FRAME_S);
    }
    sc.take_trace().expect("запись включена")
}

fn dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../golden/exo_loop")
}

fn fnv(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf2_9ce4_8422_2325, |h, b| {
        (h ^ u64::from(*b)).wrapping_mul(0x0000_0100_0000_01b3)
    })
}

fn csv(trace: &LoopTrace) -> String {
    let mut out = String::from(COLUMNS);
    out.push('\n');
    for sample in &trace.samples {
        let row: Vec<String> = sample.values().iter().map(|v| format!("{v:e}")).collect();
        let _ = writeln!(out, "{}", row.join(","));
    }
    out
}

#[test]
fn loop_repeats_reference_bit_for_bit() {
    let firmware = fnv(&std::fs::read(exo_ctrl_elf()).expect("образ прошивки"));
    let mut manifest = format!("firmware_fnv = {firmware:016x}\nstep_s = 25e-6\n");
    let traces: Vec<_> = CASES.iter().map(|c| (c, run(c))).collect();
    for (case, trace) in &traces {
        let _ = writeln!(manifest, "{} steps={} hash={:016x}", case.name, trace.steps, trace.hash);
    }
    if std::env::var_os("UPDATE_GOLDEN").is_some() {
        std::fs::create_dir_all(dir()).unwrap();
        for (case, trace) in &traces {
            std::fs::write(dir().join(format!("{}.csv", case.name)), csv(trace)).unwrap();
        }
        std::fs::write(dir().join("manifest.txt"), &manifest).unwrap();
        return;
    }
    let stored = std::fs::read_to_string(dir().join("manifest.txt")).expect("эталон снят");
    assert_eq!(stored, manifest, "поведение контура изменилось");
    for (case, trace) in &traces {
        let file = std::fs::read_to_string(dir().join(format!("{}.csv", case.name))).unwrap();
        assert_eq!(file, csv(trace), "{}", case.name);
    }
}

#[test]
fn every_reference_case_exercises_what_it_claims() {
    let by_name = |name: &str| run(CASES.iter().find(|c| c.name == name).unwrap());
    let moved = |t: &LoopTrace| {
        let first = t.samples.first().unwrap().theta;
        t.samples.iter().map(|s| (s.theta - first).abs()).fold(0.0, f64::max)
    };
    assert!(moved(&by_name("hold_relaxed")) > 0.2, "привод не двигал сустав");
    assert!(by_name("obstacle").samples.iter().any(|s| s.contact_n > 1.0), "нет контакта");
    let off_after = |t: &LoopTrace| {
        t.samples.iter().filter(|s| s.t > 1.5).all(|s| !s.driver_on)
    };
    assert!(off_after(&by_name("fault_estop")), "аварийная кнопка не сняла драйвер");
    assert!(off_after(&by_name("fault_sensor")), "обрыв датчика не снял драйвер");
    let flags = |t: &LoopTrace| t.samples.last().unwrap().reported_faults;
    assert_ne!(flags(&by_name("fault_estop")), 0, "прошивка не сообщила об отказе");
    assert_ne!(
        flags(&by_name("fault_estop")),
        flags(&by_name("fault_sensor")),
        "разные отказы неотличимы в логе"
    );
    let jam = by_name("fault_jam");
    let held = jam.samples.iter().filter(|s| s.t > 1.05).map(|s| s.omega.abs()).fold(0.0, f64::max);
    assert_eq!(held, 0.0, "заклинивший сустав двигался");
}
