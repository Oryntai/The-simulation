use sim_scenarios::{Level, Scenario, Status};
use std::time::{Duration, Instant};

fn wait<F: Fn(&dyn Scenario) -> bool>(sc: &mut dyn Scenario, done: F, secs: f64) -> bool {
    let t0 = Instant::now();
    while t0.elapsed().as_secs_f64() < secs {
        sc.advance(0.01);
        if done(sc) {
            return true;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    false
}

#[test]
fn every_example_runs_and_matches_ngspice() {
    let mut sc = sim_scenarios::circuit_lab::CircuitLab::default();
    let n = sc.choices()[0].options.len();
    for k in 0..n {
        sc.set_choice("example", k);
        let title = sc.choices()[0].options[k].clone();
        assert!(
            wait(
                &mut sc,
                |s| !s.readouts().is_empty() || s.notes().iter().any(|n| n.level == Level::Danger),
                60.0
            ),
            "{title}: расчёт не закончился"
        );
        assert!(
            !sc.notes().iter().any(|n| n.level == Level::Danger),
            "{title}: {:?}",
            sc.console()
        );
        sc.run_action("ngspice");
        assert!(
            wait(&mut sc, |s| !s.comparisons().is_empty(), 60.0),
            "{title}: ngspice молчит"
        );
        for c in sc.comparisons() {
            println!("{title}: {} — {:?} ({})", c.title, c.status, c.explanation);
            assert_ne!(c.status, Status::Mismatch, "{title}: {}", c.title);
        }
    }
}

#[test]
fn user_edit_with_error_shows_line() {
    let mut sc = sim_scenarios::circuit_lab::CircuitLab::default();
    sc.set_text(
        "netlist",
        "Сломанная схема\nR1 a 0 1k\nQ7 a b c\n.tran 1u 1m\n.end\n".into(),
    );
    assert!(wait(
        &mut sc,
        |s| s.notes().iter().any(|n| n.level == Level::Danger),
        10.0
    ));
    let msg = sc
        .notes()
        .into_iter()
        .find(|n| n.level == Level::Danger)
        .unwrap()
        .text;
    println!("{msg}");
    assert!(
        msg.contains('3'),
        "сообщение должно указать строку 3: {msg}"
    );
}

#[test]
fn pcb_thin_traces_overheat_and_fix_works() {
    let mut sc = sim_scenarios::pcb_board::PcbBoard::default();
    sc.apply_preset(0);
    let bad = sc.comparisons();
    for c in &bad {
        println!(
            "{}: {:.2} / {:.2} {:?}",
            c.title, c.sim, c.reference, c.status
        );
        assert_ne!(c.status, Status::Mismatch);
    }
    assert_eq!(bad[0].status, Status::OverLimit, "1 мм под 8 А — тонко");
    let rise = |sc: &sim_scenarios::pcb_board::PcbBoard| {
        sc.readouts()
            .iter()
            .filter(|r| r.label.ends_with("перегрев"))
            .map(|r| r.value)
            .fold(0.0, f64::max)
    };
    let hot = rise(&sc);
    sc.apply_preset(1);
    let ok = rise(&sc);
    println!("перегрев дорожек: 1 мм/1 oz {hot:.0} К → 3 мм/2 oz {ok:.1} К");
    assert!(hot > 20.0 && ok < 20.0);
    assert_eq!(sc.comparisons()[0].status, Status::WithinLimit);
    for n in sc.notes() {
        println!("  <{:?}> {}", n.level, n.text);
    }
}

#[test]
fn lab_skips_source_driven_nodes() {
    let mut sc = sim_scenarios::circuit_lab::CircuitLab::default();
    sc.set_choice("example", 6);
    assert!(wait(&mut sc, |s| s.plot().is_some(), 60.0));
    let names: Vec<String> = sc
        .plot()
        .unwrap()
        .series
        .iter()
        .map(|s| s.name.clone())
        .collect();
    println!("на графике: {names:?}");
    assert!(!names.iter().any(|n| n == "v(vbus)" || n == "v(gh)"));
}
