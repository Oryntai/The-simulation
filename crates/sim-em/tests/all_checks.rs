//! Все сверки `sim-em` с эталоном должны проходить; каждая печатается целиком
//! (оба числа, ошибка, допуск, допущения) — `cargo test -p sim-em -- --nocapture`.

#[test]
fn every_reference_check_passes() {
    let checks = sim_em::reference_checks();
    assert!(!checks.is_empty());
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
    let checks = sim_em::reference_checks();
    let mut ids: Vec<_> = checks.iter().map(|c| c.id).collect();
    ids.sort_unstable();
    let n = ids.len();
    ids.dedup();
    assert_eq!(n, ids.len(), "повторяющиеся id сверок");
}

#[test]
fn datasheet_discrepancies_are_reported() {
    // Это находки о данных производителей, не о модели: печатаем, не требуем «пройдено».
    for c in sim_em::datasheet_discrepancies() {
        println!("{c}");
        assert!(c.simulated.is_finite() && c.reference.is_finite());
    }
}
