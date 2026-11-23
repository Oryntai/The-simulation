//! Все сверки `sim-env` со вторым независимым способом должны проходить.

#[test]
fn all_reference_checks_pass() {
    let checks = sim_env::reference_checks();
    assert!(checks.len() >= 60, "сверок слишком мало: {}", checks.len());
    let mut failed = Vec::new();
    for c in &checks {
        println!("{c}");
        if !c.passed() {
            failed.push(c.id);
        }
    }
    assert!(failed.is_empty(), "не прошли: {failed:?}");
}

#[test]
fn check_ids_are_unique() {
    let checks = sim_env::reference_checks();
    let mut ids: Vec<_> = checks.iter().map(|c| c.id).collect();
    ids.sort_unstable();
    let n = ids.len();
    ids.dedup();
    assert_eq!(ids.len(), n, "повторяющиеся id сверок");
}
