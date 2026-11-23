//! Контракт exo-ctrl: конфигурация, протокол, поведение прошивки, скорость эмуляции.

use sim_mcu::bench::{Bench, ExoPlant, PlantParams};
use sim_mcu::checks::exo_bench;
use sim_mcu::exo::{
    CONFIG_SIZE, ExoCommand, ExoConfig, ExoMode, ExoTelemetry, fault, pins, sensors, state,
};
use sim_mcu::{Mcu, firmware};

#[test]
fn config_bytes_round_trip_and_layout() {
    let c = ExoConfig {
        i_max_a: 6.5,
        test_hang_ms: 123,
        ..ExoConfig::default()
    };
    let b = c.to_bytes();
    assert_eq!(b.len(), CONFIG_SIZE);
    assert_eq!(&b[0..4], b"EXOC");
    assert_eq!(u32::from_le_bytes(b[4..8].try_into().unwrap()), 4);
    assert_eq!(f32::from_le_bytes(b[0x1c..0x20].try_into().unwrap()), 6.5);
    assert_eq!(u32::from_le_bytes(b[0x64..0x68].try_into().unwrap()), 123);
    assert_eq!(f32::from_le_bytes(b[0x6c..0x70].try_into().unwrap()), 0.844);
    assert_eq!(f32::from_le_bytes(b[0x80..0x84].try_into().unwrap()), 1.5);
    assert_eq!(f32::from_le_bytes(b[0x84..0x88].try_into().unwrap()), 3.0);
    assert_eq!(f32::from_le_bytes(b[0x88..0x8c].try_into().unwrap()), 0.2);
    assert_eq!(f32::from_le_bytes(b[0x8c..0x90].try_into().unwrap()), 40.0);
    assert_eq!(f32::from_le_bytes(b[0x90..0x94].try_into().unwrap()), 14.0);
    assert_eq!(
        u32::from_le_bytes(b[0x94..0x98].try_into().unwrap()),
        sim_mcu::exo::crc32(&b[..0x94])
    );
    assert_eq!(ExoConfig::from_bytes(&b), Some(c));
    let mut bad = b.clone();
    bad[0x20] ^= 1;
    assert_eq!(ExoConfig::from_bytes(&bad), None, "CRC должен ловить порчу");
}

#[test]
fn protocol_round_trip() {
    let c = ExoCommand {
        mode: ExoMode::PositionTrack,
        assist_pct: 70,
        target_deg: 95.3,
        payload_kg: 1.2,
        seq: 9,
    };
    let e = c.encode();
    assert_eq!(e, [0xc1, 2, 70, 0xb9, 0x03, 12, 0, 9]);
    assert_eq!(ExoCommand::decode(&e), Some(c));
    assert_eq!(ExoCommand::decode(&e[..7]), None);
    let t = ExoTelemetry {
        state: 0x45,
        angle_deg: 90.1,
        current_a: -2.35,
        temp_c: -5.5,
        vbat_v: 24.07,
        fault_flags: fault::ESTOP | fault::SENSOR,
        seq: 200,
        last_cmd_seq: 9,
        fault_flags2: fault::STALL | fault::WINDING_HOT,
    };
    let e = t.encode();
    assert_eq!(e.len(), 14);
    assert_eq!(e[13], 0x03, "второй байт флагов — в конце пакета");
    assert_eq!(ExoTelemetry::decode(&e), Some(t));
    assert_eq!(
        ExoTelemetry::decode(&e[..13]),
        None,
        "старый 13-байтный формат не принимается"
    );
    assert_eq!(
        fault::names(t.all_faults()),
        vec![
            "датчик угла",
            "аварийная кнопка",
            "заклинивание",
            "обмотка горячая"
        ]
    );
}

#[test]
fn sensor_scales_match_contract() {
    assert!((sensors::angle_volts(0.0) - 0.3).abs() < 1e-12);
    assert!((sensors::angle_volts(150.0) - 3.0).abs() < 1e-12);
    assert!((sensors::current_volts(-16.5) - 0.0).abs() < 1e-12);
    assert!((sensors::temp_volts(25.0) - 0.75).abs() < 1e-12);
    assert!((sensors::battery_volts(22.0) - 2.0).abs() < 1e-12);
    assert!((sensors::angle_from_volts(sensors::angle_volts(42.0)) - 42.0).abs() < 1e-12);
}

#[test]
fn firmware_without_config_uses_defaults_and_reports_it() {
    let mut m = Mcu::from_elf(firmware::exo_ctrl_elf()).unwrap();
    sim_mcu::exo::write_sensors(&mut m, 90.0, 0.0, 30.0, 24.0);
    m.reset();
    m.run_for(0.05).unwrap();
    let tl: Vec<ExoTelemetry> = m
        .radio_take_tx()
        .iter()
        .filter_map(|p| ExoTelemetry::decode(p))
        .collect();
    assert!(tl.len() >= 2);
    let last = tl.last().unwrap();
    assert!(last.state & state::DEFAULT_CONFIG != 0);
    assert_eq!(last.mode(), 0);
    assert!(!last.driver_on());
    assert!(
        !m.gpio_output(pins::GPIO_ENABLE),
        "без команды драйвер выключен"
    );
    assert!(
        m.gpio_output(pins::GPIO_LED),
        "индикатор горит — неисправностей нет"
    );
    assert!((last.angle_deg - 90.0).abs() < 0.2);
    assert!((last.vbat_v - 24.0).abs() < 0.05);
}

#[test]
fn led_blinks_at_5_hz_on_fault() {
    let cfg = ExoConfig::default();
    let mut b = exo_bench(&cfg, 90.0, true, 0.0).unwrap();
    b.mcu.set_gpio_input(pins::GPIO_ESTOP, false);
    b.run(0.05);
    let mut toggles = 0;
    let mut last = b.mcu.gpio_output(pins::GPIO_LED);
    let t0 = b.time();
    while b.time() - t0 < 1.0 {
        b.step();
        let now = b.mcu.gpio_output(pins::GPIO_LED);
        if now != last {
            toggles += 1;
            last = now;
        }
    }
    assert!(
        (9..=11).contains(&toggles),
        "5 Гц = 10 переключений за 1 с, а было {toggles}"
    );
}

#[test]
fn closed_loop_gravity_assist_follows_human_motion() {
    // Человек ведёт руку от 20° до 120°; поддержка 100 % должна давать ток по формуле.
    let cfg = ExoConfig::default();
    let mut b = exo_bench(&cfg, 20.0, true, 1.0).unwrap();
    let c = ExoCommand {
        mode: ExoMode::GravityAssist,
        assist_pct: 100,
        target_deg: 0.0,
        payload_kg: 1.0,
        seq: 0,
    };
    b.send(c);
    b.run(0.1);
    let mut worst: f64 = 0.0;
    for k in 0..100 {
        let th = 20.0 + k as f64;
        b.plant.hold_deg = Some(th);
        b.run(0.01);
        if k % 20 == 0 {
            b.send(c);
        }
        let want = cfg.gravity_assist_current(100.0, 1.0, th);
        worst = worst.max((b.plant.current - want).abs());
    }
    assert!(worst < 0.05, "наибольшее отклонение тока {worst} А");
}

#[test]
fn emulation_is_fast_enough() {
    // Требование: exo-ctrl не медленнее 20× реального времени в профиле тестов.
    // Берём лучшую из трёх попыток (тесты идут параллельно и делят ядра хоста).
    let mut best_mcu = 0.0f64;
    let mut best_loop = 0.0f64;
    let mut mips = 0.0f64;
    for _ in 0..3 {
        let mut m = Mcu::from_elf(firmware::exo_ctrl_elf()).unwrap();
        sim_mcu::exo::write_sensors(&mut m, 90.0, 2.36, 30.0, 24.0);
        m.reset();
        let cmd = ExoCommand {
            mode: ExoMode::GravityAssist,
            assist_pct: 100,
            target_deg: 0.0,
            payload_kg: 2.0,
            seq: 1,
        };
        m.radio_deliver(&cmd.encode(), -50);
        let w = std::time::Instant::now();
        let i0 = m.instret();
        for _ in 0..50_000 {
            m.run_for(20e-6).unwrap();
        }
        let s = w.elapsed().as_secs_f64();
        best_mcu = best_mcu.max(1.0 / s);
        mips = mips.max((m.instret() - i0) as f64 / s / 1e6);

        let mut plant = ExoPlant::new(PlantParams::default(), 30.0);
        plant.payload_kg = 0.0;
        let mut bench = Bench::new(firmware::exo_ctrl_elf(), &ExoConfig::default(), plant).unwrap();
        bench.send(ExoCommand {
            mode: ExoMode::PositionTrack,
            assist_pct: 0,
            target_deg: 90.0,
            payload_kg: 0.0,
            seq: 0,
        });
        let w = std::time::Instant::now();
        bench.run(0.4);
        best_loop = best_loop.max(0.4 / w.elapsed().as_secs_f64());
    }
    println!(
        "exo-ctrl: только МК — {best_mcu:.1}× реального времени ({mips:.0} MIPS), \
         замкнутый контур со стендом — {best_loop:.1}×"
    );
    assert!(best_mcu >= 20.0, "эмуляция exo-ctrl {best_mcu:.1}× < 20×");
}
