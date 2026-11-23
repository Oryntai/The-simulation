//! Все сверки блока со вторым независимым способом должны проходить.

#[test]
fn all_reference_checks_pass() {
    let checks = sim_mcu::reference_checks();
    assert!(checks.len() >= 25, "сверок {}", checks.len());
    let mut ids = std::collections::BTreeSet::new();
    for c in &checks {
        println!("{c}");
        assert!(ids.insert(c.id), "повтор id {}", c.id);
        assert!(c.id.starts_with("mcu."), "{}", c.id);
    }
    for c in &checks {
        c.assert_pass();
    }
}
