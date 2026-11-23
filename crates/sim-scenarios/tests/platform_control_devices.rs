//! Приёмка этапа 1: контрольные устройства идут через общий рантайм только из файлов данных.

use std::path::PathBuf;

use sim_runtime::{LogKind, Runtime, Status};
use sim_scenarios::Scenario;
use sim_scenarios::exo_loop::{ExoLoop, LoopSample};
use sim_signal::{Command, Time};
use sim_world::{CheckStatus, load_and_compile, verify_compiled};

fn devices(path: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../devices")
        .join(path)
}

/// Эталон: момент привода и человека на каждом шаге 25 мкс и что получилось.
fn oracle(preset: usize, fault: Option<(f64, &str)>, seconds: f64) -> Vec<LoopSample> {
    let mut sc = ExoLoop::default();
    sc.apply_preset(preset);
    sc.start_trace(1);
    let frames = (seconds / 0.01).round() as usize;
    let at = fault.map(|(t, _)| (t / 0.01).round() as usize);
    for frame in 0..frames {
        if Some(frame) == at {
            sc.run_action(fault.expect("отказ").1);
        }
        sc.advance(0.01);
    }
    sc.take_trace().expect("запись").samples
}

/// Те же моменты подаются в мир из данных. Возвращает наибольшие ошибки угла и скорости.
fn replay(samples: &[LoopSample]) -> (f64, f64) {
    let world = load_and_compile(&devices("01-elbow-assist/world.toml")).unwrap();
    assert_eq!(world.physics.tick(1), Time(25_000), "шаг эталона 25 мкс");
    let mut rt = Runtime::load(world).unwrap();
    rt.record_joint_state("состояние").unwrap();
    let channel = rt.channel("устройство/момент").expect("канал момента");
    for s in samples {
        rt.act(&Command {
            targets: vec![(channel, s.joint_torque + s.human_torque)],
        })
        .unwrap();
        assert_eq!(rt.step().unwrap(), Status::Running, "{:?}", rt.failure());
    }
    let header = rt.log().header();
    let q = header.truth.iter().position(|c| c.name == "qpos[0]").unwrap() as u32;
    let v = header.truth.iter().position(|c| c.name == "qvel[0]").unwrap() as u32;
    let series = |id: u32| -> Vec<f64> {
        rt.log()
            .entries()
            .iter()
            .filter(|e| e.kind == LogKind::Truth && e.id == id)
            .map(|e| e.value)
            .collect()
    };
    let (angles, speeds) = (series(q), series(v));
    // В логе идёт одно значение на такт. Сдвиг берём по совпадению числа записей с шагами.
    assert!(angles.len() >= samples.len(), "{} {}", angles.len(), samples.len());
    let shift = angles.len() - samples.len();
    let (mut angle_error, mut speed_error) = (0.0_f64, 0.0_f64);
    for (k, s) in samples.iter().enumerate() {
        angle_error = angle_error.max((angles[k + shift] - s.theta).abs());
        speed_error = speed_error.max((speeds[k + shift] - s.omega).abs());
    }
    (angle_error, speed_error)
}

#[test]
fn elbow_assist_matches_reference_within_one_percent() {
    // Режимы без упора, заклинивания, воды и препятствия: эта физика появляется на этапах 2–4.
    for (name, preset, fault) in [
        ("assist", 0, None),
        ("wrong_payload", 1, None),
        ("hold_relaxed", 2, None),
        ("cold", 3, None),
        ("fault_hang", 2, Some((1.0, "hang"))),
    ] {
        let samples = oracle(preset, fault, 3.0);
        assert!(samples.iter().all(|s| s.stop_torque == 0.0 && s.contact_n == 0.0), "{name}");
        let (angle, speed) = replay(&samples);
        let span = samples.iter().map(|s| s.theta.abs()).fold(0.0, f64::max);
        let peak = samples.iter().map(|s| s.omega.abs()).fold(0.0, f64::max);
        println!(
            "{name}: угол {:.5} % от {span:.3} рад, скорость {:.5} % от {peak:.4} рад/с",
            100.0 * angle / span,
            100.0 * speed / peak.max(1e-9)
        );
        assert!(angle <= 0.01 * span, "{name}: угол разошёлся на {angle} рад");
        assert!(speed <= 0.01 * peak.max(1e-3), "{name}: скорость разошлась на {speed}");
    }
}

#[test]
fn elbow_assist_world_is_verified_by_second_engine() {
    let world = load_and_compile(&devices("01-elbow-assist/world.toml")).unwrap();
    let check = verify_compiled(&world);
    assert_eq!(check.inertial, CheckStatus::Verified, "{check:?}");
    assert_eq!(check.dynamics, CheckStatus::Verified, "{check:?}");
    let engine = check.engine.expect("сверка движков");
    assert!(engine.max_mass_rel < 1e-9 && engine.max_bias_abs < 1e-6, "{engine:?}");
}

#[test]
fn balance_cart_is_built_from_data_and_falls_without_control() {
    let path = devices("07-balance-cart/world.toml");
    let world = load_and_compile(&path).unwrap();
    assert_eq!(world.efforts.len(), 2);
    let check = verify_compiled(&world);
    assert_eq!(check.inertial, CheckStatus::Verified, "{check:?}");
    assert!(
        world.simplifications.iter().any(|s| s.blocks_proof),
        "оценочные массы не должны давать доказательство"
    );
    let mut rt = Runtime::load(world).unwrap();
    rt.record_joint_state("состояние").unwrap();
    assert_eq!(
        rt.run_until(Time::from_secs(3.0).unwrap()).unwrap(),
        Status::Running,
        "{:?}",
        rt.failure()
    );
    let header = rt.log().header();
    let id = |name: &str| header.truth.iter().position(|c| c.name == name).unwrap() as u32;
    let last = |channel: u32| {
        rt.log()
            .entries()
            .iter()
            .rev()
            .find(|e| e.kind == LogKind::Truth && e.id == channel)
            .unwrap()
            .value
    };
    let (w, y) = (last(id("qpos[3]")), last(id("qpos[5]")));
    // Наклон вокруг поперечной оси по кватерниону: без управления неустойчивый корпус падает.
    let tilt = 2.0 * y.atan2(w).to_degrees();
    println!("наклон через 3 с: {tilt:.1}°, высота оси {:.4} м", last(id("qpos[2]")));
    assert!(tilt.abs() > 45.0, "тележка не упала: {tilt}°");
    assert!(last(id("qpos[2]")) > 0.0, "провалилась сквозь пол");
}
