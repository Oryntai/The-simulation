//! Модели тела в MuJoCo: открываются, поза ставится, сверки посчитаны и правдоподобны.
use sim_scenarios::Scenario;
use sim_scenarios::anatomy::{Anatomy, WINTER_ARM_FRACTION, summaries};

#[test]
fn all_three_models_open_and_agree_roughly() {
    let s = summaries();
    let names = ["MS-Human-700 всё тело", "MyoArm", "MS-Human-700 рука"];
    for (n, r) in names.iter().zip(&s) {
        let m = r.as_ref().unwrap_or_else(|e| panic!("{n}: {e}"));
        println!(
            "{n}: рука {:.3} кг, ΣF₀ сгибателей {:.0} Н, тело {:?}",
            m.arm_mass, m.flexor_f0, m.body_mass
        );
        assert!(
            m.arm_mass > 2.0 && m.arm_mass < 6.0,
            "{n}: масса руки {}",
            m.arm_mass
        );
        assert!(
            m.flexor_f0 > 1000.0 && m.flexor_f0 < 5000.0,
            "{n}: ΣF₀ {}",
            m.flexor_f0
        );
        if let Some(b) = m.body_mass {
            // Модель «среднего» человека против таблиц: в пределах ±50% (иначе — ошибка загрузки).
            let rel = m.arm_mass / (WINTER_ARM_FRACTION * b) - 1.0;
            assert!(
                rel.abs() < 0.5,
                "{n}: масса руки расходится с таблицами на {:.0}%",
                rel * 100.0
            );
        }
    }
}

#[test]
fn pose_is_applied_with_shoulder_rhythm() {
    let mut a = Anatomy::default();
    // Модель 1 — MyoArm: у него плечевой пояс связан с подъёмом плеча.
    a.set_choice("model", 1);
    a.apply_preset(1); // рука вперёд 90°
    let r = a.readouts();
    let elbow = r
        .iter()
        .find(|x| x.label.starts_with("Угол в локте"))
        .unwrap()
        .value;
    assert!((elbow - 10.0).abs() < 0.5, "локоть {elbow}");
    assert!(!a.visuals().is_empty());
    let cmp = a.comparisons();
    for c in &cmp {
        println!(
            "{}: {:.3} ↔ {:.3} ({:?})",
            c.title, c.sim, c.reference, c.status
        );
    }
    assert!(!cmp.is_empty());
}
