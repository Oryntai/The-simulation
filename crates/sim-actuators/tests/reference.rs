//! Все эталонные сверки должны проходить; каждая печатается целиком:
//! `cargo test --offline -p sim-actuators --test reference -- --nocapture`.

use sim_actuators::{datasheet_discrepancies, reference_checks};

#[test]
fn every_reference_check_passes() {
    let checks = reference_checks();
    assert!(checks.len() >= 40, "сверок всего {}", checks.len());
    let mut failed = Vec::new();
    for c in &checks {
        println!("{c}");
        if !c.passed() {
            failed.push(c.id);
        }
    }
    println!("сверок: {}, не пройдено: {}", checks.len(), failed.len());
    assert!(failed.is_empty(), "не пройдены: {failed:?}");
}

#[test]
fn check_ids_are_unique() {
    let mut ids: Vec<_> = reference_checks().iter().map(|c| c.id).collect();
    ids.extend(datasheet_discrepancies().iter().map(|c| c.id));
    let n = ids.len();
    ids.sort_unstable();
    ids.dedup();
    assert_eq!(n, ids.len(), "повторяющиеся id сверок");
}

#[test]
fn discrepancies_are_reported_not_hidden() {
    // Это находки о данных производителей: печатаем, не требуем «пройдено».
    let list = datasheet_discrepancies();
    assert!(list.len() >= 20);
    let mut outside = 0;
    for c in &list {
        println!("{c}");
        assert!(
            c.simulated.is_finite() && c.reference.is_finite(),
            "{}",
            c.id
        );
        assert!(
            !c.note.is_empty(),
            "{}: не сказано, какое число взято",
            c.id
        );
        if !c.passed() {
            outside += 1;
        }
    }
    println!("расхождений: {}, вне допуска 5 %: {outside}", list.len());
    // Если все стали «проходить», значит расхождения перестали быть видны.
    assert!(outside >= 10, "вне допуска только {outside}");
}
