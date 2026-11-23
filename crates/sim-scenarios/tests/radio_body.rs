use sim_scenarios::{Level, Scenario, Status};

#[test]
fn presets_give_expected_verdicts() {
    let mut sc = sim_scenarios::radio_body::RadioBody::default();
    // 0: телефон в 3 м, видно — связь устойчива, пакеты идут.
    sc.apply_preset(0);
    sc.advance(0.1);
    for _ in 0..100 {
        sc.advance(0.05);
    }
    let margin = sc
        .readouts()
        .iter()
        .find(|r| r.label == "Запас линии")
        .unwrap()
        .value;
    println!("видно: запас {margin:.1} дБ");
    assert!(margin > 20.0);
    assert!(sc.notes().iter().any(|n| n.level == Level::Good));
    let lat = sc
        .comparisons()
        .into_iter()
        .find(|c| c.title.contains("задержка"))
        .unwrap();
    println!(
        "задержка: канал {:.2} мс, формула {:.2} мс, {:?}",
        lat.sim, lat.reference, lat.status
    );
    assert_eq!(lat.status, Status::Ok);

    // 3: море, 5 см — связи нет (≈10 дБ на сантиметр).
    sc.apply_preset(3);
    for _ in 0..40 {
        sc.advance(0.05);
    }
    assert!(
        sc.notes()
            .iter()
            .any(|n| n.level == Level::Danger && n.text.contains("Под водой"))
    );
    let lost = sc
        .readouts()
        .iter()
        .find(|r| r.label == "Потеряно")
        .unwrap()
        .value;
    println!("море 5 см: потеряно {lost:.1} %");
    assert!(lost > 50.0);
}
