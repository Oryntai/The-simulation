use sim_scenarios::{Scenario, Status};
use std::time::Instant;

#[test]
fn every_mode_computes_and_checks_agree() {
    let mut sc = sim_scenarios::magnets::Magnets::default();
    for preset in 0..4 {
        sc.apply_preset(preset);
        let t = Instant::now();
        let plot = sc.plot().unwrap();
        let cmp = sc.comparisons();
        let dt = t.elapsed().as_secs_f64() * 1000.0;
        println!(
            "пресет {preset}: график+сверки за {dt:.1} мс, точек {}",
            plot.series[0].points.len()
        );
        for c in &cmp {
            println!(
                "  {:<55} {:>10.4} / {:>10.4} {} {:?}",
                c.title, c.sim, c.reference, c.unit, c.status
            );
            assert_ne!(c.status, Status::Mismatch, "{}", c.title);
        }
        for r in sc.readouts() {
            assert!(!r.value.is_nan(), "{}", r.label);
        }
        assert!(!sc.visuals().is_empty());
        let lines = sc.overlays().len();
        println!("  линий поля (отрезков): {lines}");
    }
}

#[test]
fn electromagnet_saturates() {
    let mut sc = sim_scenarios::magnets::Magnets::default();
    sc.apply_preset(2);
    let b = |sc: &sim_scenarios::magnets::Magnets| {
        sc.readouts()
            .iter()
            .find(|r| r.label == "Индукция в зазоре")
            .unwrap()
            .value
    };
    sc.set_param("i", 1.0);
    let b1 = b(&sc);
    sc.set_param("i", 10.0);
    let b10 = b(&sc);
    println!("B(1 А) = {b1:.3} Тл, B(10 А) = {b10:.3} Тл");
    assert!(b10 < 5.0 * b1, "насыщение должно срезать рост");
}
