//! Периферия VMCU-C3 напрямую (по карте регистров MEMORY_MAP.md).

use sim_mcu::periph::{adc, radio::Radio, uart::Uart, wdt};

#[test]
fn adc_is_ideal_12_bit_with_3v3_reference() {
    assert_eq!(adc::code(0.0), 0);
    assert_eq!(adc::code(-1.0), 0);
    assert_eq!(adc::code(f64::NAN), 0);
    assert_eq!(adc::code(1.65), 2048);
    assert_eq!(adc::code(3.3), 4095);
    assert_eq!(adc::code(5.0), 4095);
    assert_eq!(adc::code(3.3 / 4096.0 * 100.5), 100);
}

#[test]
fn uart_fifo_and_overflow() {
    let mut u = Uart::default();
    assert_eq!(u.read(0x04), Some(0x8000_0000), "пустой FIFO — бит 31");
    assert_eq!(u.host_send(&[1, 2, 3]), 3);
    assert_eq!(u.read(0x08), Some(1 | 2 | (3 << 16)));
    assert_eq!(u.read(0x04), Some(1));
    let big = vec![0u8; 300];
    let n = u.host_send(&big);
    assert_eq!(n, 256 - 2);
    assert_eq!(u.read(0x0c), Some(300 - (256 - 2)));
    assert!(u.write(0x00, 0x41));
    assert_eq!(u.tx, b"A");
    assert!(!u.write(0x30, 0), "несуществующий регистр");
}

#[test]
fn radio_queues_packets_both_ways() {
    let mut r = Radio::default();
    for (k, b) in [0xc1u32, 0x02, 0x03, 0x04, 0x05].iter().enumerate() {
        assert!(r.write(0x100 + k as u32, 1, *b));
    }
    assert!(r.write(0x004, 4, 5));
    assert_eq!(r.tx_fifo.pop_front().unwrap(), vec![0xc1, 2, 3, 4, 5]);
    assert!(
        r.write(0x004, 4, 0),
        "длина 0 — пакет отброшен, но запись принята"
    );
    assert_eq!(r.read(0x014, 4), Some(1));

    assert!(r.deliver(&[9, 8, 7], -70));
    assert!(!r.deliver(&[0; 252], -70), "длиннее 251 байта");
    assert_eq!(r.read(0x000, 4), Some(1 | (1 << 8)));
    assert_eq!(r.read(0x008, 4), Some(3));
    assert_eq!(r.read(0x00c, 4), Some((-70i32) as u32));
    assert_eq!(r.read(0x200, 4), Some(0x0007_0809));
    assert!(r.write(0x010, 4, 1));
    assert_eq!(r.read(0x008, 4), Some(0));
    for _ in 0..8 {
        assert!(r.deliver(&[1], 0));
    }
    assert!(!r.deliver(&[1], 0), "очередь из 8 полна");
    assert_eq!(r.read(0x018, 4), Some(1));
}

#[test]
fn watchdog_kick_requires_key() {
    let mut w = wdt::Wdt::default();
    assert!(w.write(0x04, 1000, 0));
    assert!(w.write(0x00, 1, 0));
    assert_eq!(w.expiry(), Some(160_000));
    assert!(w.write(0x08, 0x1234, 50_000));
    assert_eq!(w.expiry(), Some(160_000), "неверный ключ не кормит");
    assert!(w.write(0x08, wdt::KICK_KEY, 50_000));
    assert_eq!(w.expiry(), Some(210_000));
    assert_eq!(w.read(0x0c, 60_000), Some(937));
}

#[test]
fn gpio_pwm_through_mcu_api() {
    // Через прошивку exo-ctrl: после команды режима 1 драйвер включается, ШИМ ненулевой.
    let mut b =
        sim_mcu::checks::exo_bench(&sim_mcu::exo::ExoConfig::default(), 90.0, true, 2.0).unwrap();
    assert_eq!(b.mcu.pwm_freq_hz(0), 20_000);
    b.send(sim_mcu::exo::ExoCommand {
        mode: sim_mcu::exo::ExoMode::GravityAssist,
        assist_pct: 100,
        target_deg: 0.0,
        payload_kg: 2.0,
        seq: 0,
    });
    b.run(0.05);
    let (duty, flex, en) = b.drive();
    assert!(en && flex);
    assert!(duty > 0.01 && duty < 0.95, "заполнение {duty}");
    assert!(b.mcu.gpio_levels() & (1 << 2) != 0);
}
