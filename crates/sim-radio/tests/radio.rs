use sim_radio::body::*;
use sim_radio::channel::*;
use sim_radio::dielectric::*;
use sim_radio::pathloss::Surroundings;
use sim_radio::phy::Phy;
use sim_radio::wearable::*;

#[test]
fn reference_checks_pass() {
    let checks = sim_radio::reference_checks();
    assert!(checks.len() >= 12);
    for c in &checks {
        println!(
            "{:<22} sim {:>12.5} ref {:>12.5} {} err {}",
            c.id,
            c.simulated,
            c.reference,
            c.unit,
            c.error_text()
        );
        assert!(c.passed(), "сверка {} не прошла: {:?}", c.id, c);
    }
}

#[test]
fn water_kills_2g4_within_centimetres() {
    let f = F_2G4;
    let fresh = Medium::FreshWater { t_c: 15.0 }.permittivity(f);
    let sea = Medium::SeaWater { t_c: 15.0 }.permittivity(f);
    let fresh_db_cm = fresh.attenuation_db_per_m(f) / 100.0;
    let sea_db_cm = sea.attenuation_db_per_m(f) / 100.0;
    println!("пресная {fresh_db_cm:.2} дБ/см, морская {sea_db_cm:.2} дБ/см");
    assert!(fresh_db_cm > 1.5 && fresh_db_cm < 4.0);
    assert!(sea_db_cm > 3.0 * fresh_db_cm);
    let link = WearableLink {
        phy: Phy::Ble1M,
        tx_dbm: 9.0,
        distance_m: 2.0,
        surroundings: Surroundings::OpenLos,
        body_blocks: false,
        medium: Medium::FreshWater { t_c: 15.0 },
        depth_m: 0.1,
        wet_detuning_db: 0.0,
    };
    let depth = max_depth_m(&link);
    println!(
        "предельная глубина пресной воды для BLE: {:.1} см",
        depth * 100.0
    );
    assert!(depth > 0.02 && depth < 0.3);
    let mut dry = link.clone();
    dry.depth_m = 0.0;
    let lost = dry.budget().margin_db() - link.budget().margin_db();
    println!("10 см пресной воды съели {lost:.1} дБ");
    assert!(lost > 30.0);
}

#[test]
fn interface_loss_air_water_is_a_few_db() {
    let w = water_itu_p840(20.0, 2.45e9);
    let l = interface_loss_db(Permittivity::VACUUM, w);
    let back = interface_loss_db(w, Permittivity::VACUUM);
    println!("граница воздух→вода {l:.2} дБ, обратно {back:.2} дБ");
    // Γ = (1 − √ε)/(1 + √ε) ≈ −0.8 → 1 − |Γ|² ≈ 0.36 → ≈ 4.4 дБ.
    assert!(l > 3.0 && l < 6.0);
    assert!((l - back).abs() < 1e-9);
}

#[test]
fn body_goes_around_not_through() {
    // Сквозь 25 см мышечной ткани ≈ 100 дБ; вокруг тела меньше: ползущая волна
    // и отражения от комнаты (пол модели −71.3 дБ).
    let through = Tissue::Muscle
        .permittivity_2g45()
        .attenuation_db_per_m(2.45e9)
        * 0.25;
    let (around, _) = around_body_db(40.0);
    println!("сквозь тело {through:.0} дБ, вокруг {around:.1} дБ");
    assert!(through > around + 15.0);
    assert!(around < 75.0);
    let shadow = off_body_shadowing_mean_db();
    println!(
        "тело закрывает: в среднем +{shadow:.1} дБ, максимум +{:.1}",
        off_body_shadowing_max_db()
    );
    assert!(shadow > 5.0 && shadow < 15.0);
}

#[test]
fn cm3_values_match_published_formula() {
    let (pl, s) = on_body_cm3_db(300.0, OnBodyRoom::Hospital);
    assert!((pl - (6.6 * 300f64.log10() + 36.1)).abs() < 1e-12);
    assert_eq!(s, 3.80);
    let (pl0, _) = around_body_db(0.0);
    assert!((pl0 - 25.8).abs() < 0.01);
}

#[test]
fn per_is_anchored_at_sensitivity() {
    for phy in Phy::ALL {
        let per = sim_radio::link::packet_error_rate(phy, 0.0, phy.test_packet_bits());
        assert!((per - phy.per_at_sensitivity()).abs() < 1e-9, "{phy:?}");
        let per_hi = sim_radio::link::packet_error_rate(phy, 6.0, phy.test_packet_bits());
        assert!(per_hi < 0.01, "{phy:?} {per_hi}");
    }
}

#[test]
fn ble_disconnects_when_link_dies_and_reconnects() {
    let mut ch = PacketChannel::new(Protocol::ble_default(), 3);
    ch.set_link(0.0, -60.0);
    ch.advance(1.0);
    assert!(ch.connected());
    ch.set_link(1.0, -110.0);
    ch.advance(2.0);
    assert!(!ch.connected());
    assert_eq!(ch.stats.disconnects, 1);
    ch.set_link(0.05, -70.0);
    ch.advance(3.0);
    assert!(ch.connected());
}

#[test]
fn wifi_drops_after_retry_limit() {
    let mut ch = PacketChannel::new(Protocol::Wifi, 5);
    ch.set_link(1.0, -100.0);
    for i in 0..100 {
        ch.send(i as f64 * 0.01, Dir::PhoneToDevice, &[0; 10]);
    }
    let got = ch.advance(10.0);
    assert!(got.is_empty());
    assert_eq!(ch.stats.dropped, 100);
}
