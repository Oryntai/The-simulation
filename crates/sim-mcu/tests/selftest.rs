//! Самопроверка: прошивка `selftest` в эмуляторе против тех же функций
//! `selftest-core`, собранных нативно (dev-зависимость) — каждое значение.

use sim_mcu::checks::{SELFTEST_RX, run_selftest, selftest_sys_expectations};

#[test]
fn every_selftest_value_matches_native_build() {
    let r = run_selftest().expect("прогон selftest");
    assert_eq!(
        r.exit_code,
        Some(0),
        "selftest не завершился штатно; сбой: {:?}\n{}",
        r.fault,
        r.uart
    );
    let mut native = Vec::new();
    selftest_core::run_isa(&selftest_core::RefIsa, &mut |k, v| native.push((k, v)));
    selftest_core::run_algo(&mut |k, v| native.push((k, v)));
    assert_eq!(native.len(), selftest_core::key_count());
    assert_eq!(
        r.end_count,
        Some(native.len() as u32),
        "прошивка сообщила иное число ключей"
    );
    assert_eq!(r.st.len(), native.len(), "лишние или потерянные ключи");
    for (k, v) in native {
        assert_eq!(
            r.st.get(k).copied(),
            Some(v),
            "{k}: эмулятор {:08x?} ≠ натив {v:08x}",
            r.st.get(k)
        );
    }
}

#[test]
fn selftest_system_values_match_specification() {
    let r = run_selftest().expect("прогон selftest");
    for (k, want, what) in selftest_sys_expectations() {
        assert_eq!(r.sys.get(k).copied(), Some(want), "{what} ({k})");
    }
    assert_eq!(
        r.sys.get("rx.crc").copied(),
        Some(selftest_core::crc32(SELFTEST_RX))
    );
    // Период таймера: ровно 1600 тиков = 16 000 тактов, без дрожания (ядро ждёт в wfi).
    assert_eq!(r.sys.get("timer.period_min").copied(), Some(16_000));
    assert_eq!(r.sys.get("timer.period_max").copied(), Some(16_000));
    assert_eq!(r.sys.get("timer.period_sum20").copied(), Some(320_000));
}

#[test]
fn known_answer_values() {
    // Значения из внешних источников, а не только «натив = эмулятор».
    assert_eq!(selftest_core::crc32(b"123456789"), 0xcbf4_3926); // CRC-32/ISO-HDLC
    assert_eq!(selftest_core::fnv1a32(b""), 0x811c_9dc5); // смещение FNV-1a
    assert_eq!(selftest_core::fnv1a32(b"a"), 0xe40c_292c);
    assert_eq!(selftest_core::fib_u64(90), 2_880_067_194_370_816_120);
    let r = run_selftest().expect("прогон selftest");
    assert_eq!(r.st.get("crc32.check").copied(), Some(0xcbf4_3926));
    assert_eq!(
        r.st.get("fib.90_lo").copied(),
        Some(2_880_067_194_370_816_120u64 as u32)
    );
}
