use sim_scenarios::{Level, Scenario, Status};

#[test]
fn presets_produce_sane_physiology_and_device_warnings() {
    let mut sc = sim_scenarios::environment::Environment::default();
    for p in 0..5 {
        sc.apply_preset(p);
        println!("=== пресет {p}");
        for r in sc.readouts() {
            println!("  {:<42} {:>9.2} {}", r.label, r.value, r.unit);
            assert!(!r.value.is_nan(), "{}", r.label);
        }
        for c in sc.comparisons() {
            println!(
                "  [{:?}] {}: {:.3} / {:.3}",
                c.status, c.title, c.sim, c.reference
            );
            assert_ne!(c.status, Status::Mismatch, "{}", c.title);
        }
        for n in sc.notes() {
            println!("  <{:?}> {}", n.level, n.text);
        }
    }
    // Мороз: рама из алюминия без подкладки — опасно; с неопреном — нет.
    sc.apply_preset(0);
    assert!(
        sc.notes()
            .iter()
            .any(|n| n.level == Level::Danger && n.text.contains("Рама"))
    );
    sc.set_choice("liner", 1);
    assert!(
        !sc.notes()
            .iter()
            .any(|n| n.level == Level::Danger && n.text.contains("Рама"))
    );
    // Жара на солнце: чёрный корпус горячий.
    sc.apply_preset(1);
    let frame = sc
        .readouts()
        .iter()
        .find(|r| r.label == "Рама экзо")
        .unwrap()
        .value;
    assert!(frame > 55.0, "{frame}");
}
