//! Две независимые записи контракта прошивки (стенд — sim-scenarios/exo_io.rs, хост МК —
//! sim-mcu/exo.rs) должны совпадать байт в байт и вольт в вольт.

use sim_mcu::exo::{ExoCommand, ExoMode, ExoTelemetry, pins, sensors};
use sim_scenarios::exo_io::*;

#[test]
fn command_bytes_match() {
    for (mode, m2) in [
        (Mode::Off, ExoMode::Off),
        (Mode::GravityAssist, ExoMode::GravityAssist),
        (Mode::Position, ExoMode::PositionTrack),
    ] {
        for (a, t, p, s) in [
            (0u8, 0.0, 0.0, 0u8),
            (70, 90.0, 10.0, 17),
            (100, 135.5, 49.9, 255),
        ] {
            let ours = encode_command(mode, a, t, p, s);
            let theirs = ExoCommand {
                mode: m2,
                assist_pct: a,
                target_deg: t,
                payload_kg: p,
                seq: s,
            }
            .encode();
            assert_eq!(
                ours.as_slice(),
                theirs.as_slice(),
                "{mode:?} {a} {t} {p} {s}"
            );
        }
    }
}

#[test]
fn telemetry_decodes_the_same() {
    let tel = ExoTelemetry {
        state: 0x45,
        angle_deg: 87.3,
        current_a: -2.37,
        temp_c: 41.6,
        vbat_v: 22.84,
        fault_flags: 0x81,
        seq: 9,
        last_cmd_seq: 200,
        fault_flags2: 0x03,
    };
    let bytes = tel.encode();
    let ours = decode_telemetry(&bytes).expect("разбор");
    assert_eq!(ours.state, tel.state);
    assert!((ours.angle_deg - tel.angle_deg).abs() < 0.051);
    assert!((ours.current_a - tel.current_a).abs() < 0.0051);
    assert!((ours.temp_c - tel.temp_c).abs() < 0.051);
    assert!((ours.vbat - tel.vbat_v).abs() < 0.0051);
    assert_eq!(ours.faults, tel.fault_flags);
    assert_eq!((ours.seq, ours.last_cmd_seq), (tel.seq, tel.last_cmd_seq));
    assert_eq!(ours.faults2, tel.fault_flags2);
}

#[test]
fn sensor_scales_and_pins_match() {
    for th in [0.0, 45.0, 90.0, 150.0] {
        assert!((angle_sensor_volts(th) - sensors::angle_volts(th)).abs() < 1e-12);
    }
    for i in [-10.0, 0.0, 2.5, 15.0] {
        assert!((current_sensor_volts(i) - sensors::current_volts(i)).abs() < 1e-12);
    }
    for t in [-20.0, 25.0, 90.0] {
        assert!((temp_sensor_volts(t) - sensors::temp_volts(t)).abs() < 1e-12);
    }
    assert!((battery_sense_volts(22.2) - sensors::battery_volts(22.2)).abs() < 1e-12);
    assert_eq!(
        (ADC_ANGLE, ADC_CURRENT, ADC_TEMP, ADC_VBAT),
        (
            pins::ADC_ANGLE,
            pins::ADC_CURRENT,
            pins::ADC_TEMP,
            pins::ADC_VBAT
        )
    );
    assert_eq!(
        (GPIO_ESTOP, GPIO_DIR, GPIO_ENABLE, GPIO_LED, PWM_BRIDGE),
        (
            pins::GPIO_ESTOP,
            pins::GPIO_DIR,
            pins::GPIO_ENABLE,
            pins::GPIO_LED,
            pins::PWM_MOTOR
        )
    );
}
