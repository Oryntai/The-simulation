//! Экзоскелет на анатомической руке: три способа посчитать момент сходятся, экзоскелет
//! разгружает мышцы и локоть, манжета нагружается ровно на момент / плечо.
use sim_scenarios::exo_anatomy::ExoAnatomy;
use sim_scenarios::{Scenario, Status};

fn run(model: usize) {
    let mut s = ExoAnatomy::default();
    s.set_choice("model", model);
    for preset in 0..4 {
        s.apply_preset(preset);
        let n = s.summary_numbers().unwrap();
        let (tau, exo, j_none, j_exo, cuff, p, act, ok) = (
            n.required,
            n.exo,
            n.joint_force_unassisted,
            n.joint_force,
            n.cuff_force,
            n.cuff_pressure_kpa,
            n.max_activation,
            n.feasible,
        );
        println!(
            "модель {model}, вариант {preset}: нужно {tau:.1} Н·м, экзо {exo:.1}, локоть {j_none:.0} → {j_exo:.0} Н, манжета {cuff:.0} Н ({p:.0} кПа), макс. активация {:.0}%, мышцам {}",
            act * 100.0,
            if ok {
                "хватает"
            } else {
                "НЕ хватает"
            }
        );
        let cmp = s.comparisons();
        for c in cmp.iter().take(3) {
            assert_eq!(c.status, Status::Ok, "{}: {}", c.title, c.explanation);
        }
        // Ни одна сверка не должна быть «необъяснимо расходится».
        for c in &cmp {
            println!(
                "  {:?} {} — {:.2} ↔ {:.2} {}: {}",
                c.status, c.title, c.sim, c.reference, c.unit, c.explanation
            );
            assert_ne!(c.status, Status::Mismatch, "{}", c.title);
        }
        for n in s.notes() {
            println!("  [{:?}] {}", n.level, n.text);
        }
        // Манжета: сила = момент привода / плечо манжеты — с точностью до проекции.
        let arm = if preset == 3 { 0.25 } else { 0.15 };
        if exo > 0.0 {
            assert!(
                (cuff - exo / arm).abs() / (exo / arm) < 0.05,
                "манжета {cuff} vs {}",
                exo / arm
            );
        }
        match preset {
            0 => {
                assert!(ok, "20 кг без экзо: средний мужчина держит при 90°");
                assert!(j_none > 900.0 && j_none < 2500.0, "сила в локте {j_none}");
            }
            1 => assert!(
                j_exo < 0.6 * j_none,
                "экзо 70% должен заметно разгрузить локоть"
            ),
            2 => {
                assert!(act < 0.05, "50 кг с экзо 100%: мышцы почти не работают");
                assert!(
                    p > 40.0,
                    "давление манжеты при 50 кг должно быть опасным: {p}"
                );
            }
            _ => assert!(
                p < 20.0,
                "широкая дальняя манжета должна быть ниже порога боли: {p}"
            ),
        }
    }
    assert!(!s.visuals().is_empty());
    assert!(s.plot().is_some());
}

#[test]
fn ms_human_arm() {
    run(0);
}

#[test]
fn myoarm() {
    run(1);
}

#[test]
fn water_makes_arm_light_and_gravity_assist_overshoots() {
    use sim_scenarios::Scenario;
    let mut sc = sim_scenarios::exo_anatomy::ExoAnatomy::default();
    sc.apply_preset(1); // 20 кг, экзо 70 %
    let dry = sc.summary_numbers().unwrap();
    sc.set_choice("medium", 1);
    let [hold, moving, help, drag_mj, drag_hand, _, _] = sc.env_numbers().unwrap();
    println!(
        "сухо: нужно {:.1} Н·м, экзо {:.1}; вода: держать {hold:.1}, двигать {moving:.1}, \
         помощь {help:.1}, сопротивление MuJoCo {drag_mj:.2} / цилиндр {drag_hand:.2}",
        dry.required, dry.exo
    );
    // Архимед: рука ~88 % и чугун ~14 % веса.
    assert!(help > 0.1 * dry.required && help < 0.3 * dry.required);
    assert!((hold - (dry.required - help)).abs() < 1e-9);
    assert!(sc.notes().iter().any(|n| n.text.contains("режим «вода»")));
    // Движение в воде дороже, чем удержание.
    assert!(moving > hold);
    let ratio = drag_mj / drag_hand;
    assert!(ratio > 0.5 && ratio < 2.0, "{ratio}");
    // Без груза экзо, настроенное на воздух, перебивает почти невесомую руку.
    sc.set_param("payload", 0.0);
    let empty = sc.summary_numbers().unwrap();
    let [hold0, _, help0, _, _, _, _] = sc.env_numbers().unwrap();
    println!(
        "без груза: нужно в воздухе {:.2}, в воде {hold0:.2}, экзо {:.2}",
        empty.required, empty.exo
    );
    assert!(help0 > 0.85 * empty.required);
    assert!(empty.exo > hold0);
    assert!(
        sc.notes()
            .iter()
            .any(|n| n.text.contains("само поднимет руку"))
    );
}

#[test]
fn wind_on_elbow_is_small_but_whole_body_is_pushed() {
    use sim_scenarios::Scenario;
    let mut sc = sim_scenarios::exo_anatomy::ExoAnatomy::default();
    sc.set_choice("medium", 2);
    sc.set_param("angle", 30.0);
    let [_, _, _, _, _, wind_mj, wind_hand] = sc.env_numbers().unwrap();
    println!("ветер 20 м/с при 30°: MuJoCo {wind_mj:.3} Н·м, цилиндр {wind_hand:.3} Н·м");
    assert!(wind_mj.abs() < 3.0);
    let r = (wind_mj / wind_hand).abs();
    assert!(r > 0.5 && r < 2.0, "{r}");
}
