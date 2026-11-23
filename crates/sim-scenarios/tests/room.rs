//! Комната: вес доходит до пола, тяжёлое влияет на лёгкое, столик опрокидывается по расчёту.

use sim_scenarios::room::{tip_mass, Room};
use sim_scenarios::{Scenario, Status};

fn room(preset: usize) -> Room {
    let mut r = Room::default();
    assert!(r.error().is_none(), "{:?}", r.error());
    r.apply_preset(preset);
    r
}

#[test]
fn floor_carries_the_whole_room_at_rest() {
    let mut r = room(0);
    r.run_for(3.0);
    assert!(r.numerically_ok());
    println!("пол {:.2} Н, вес {:.2} Н, кинетическая {:.6} Дж", r.floor_force(), r.total_weight(), r.kinetic());
    assert!(r.kinetic() < 1e-3, "комната должна успокоиться");
    assert!((r.floor_force() - r.total_weight()).abs() < 0.01 * r.total_weight());
    assert_eq!(r.comparisons()[0].status, Status::Ok);
}

#[test]
fn falling_weight_hits_at_free_fall_speed_and_shakes_the_table() {
    let mut r = room(1);
    r.set_param("drop", 0.8);
    r.set_param("mass", 16.0);
    r.run_for(2.0);
    assert!(r.numerically_ok());
    let v = r.impact_speed().expect("груз должен удариться");
    let want = (2.0 * 9.81 * 0.8_f64).sqrt();
    println!("удар {v:.3} м/с, расчёт {want:.3}; пик силы на пол {:.0} Н при весе {:.0} Н", r.peak_floor_force(), r.total_weight());
    assert!((v - want).abs() < 0.02 * want);
    assert!(r.energy() <= r.initial_energy() + 0.05, "энергия выросла");
    assert!(r.peak_floor_force() > 1.5 * r.total_weight(), "удар должен дойти через стол до пола");
}

#[test]
fn heavy_impact_loads_the_floor_far_more_than_light_one() {
    let extra = |mass: f64| {
        let mut r = room(1);
        r.set_param("mass", mass);
        r.run_for(2.0);
        assert!(r.numerically_ok());
        r.peak_floor_force() - r.total_weight()
    };
    let (light, heavy) = (extra(0.5), extra(40.0));
    println!("добавка к силе на пол от удара: 0.5 кг {light:.0} Н, 40 кг {heavy:.0} Н");
    assert!(heavy > 10.0 * light.max(1.0), "тяжёлый груз должен бить по опоре намного сильнее лёгкого");
}

/// Классика: шар, пущенный скользить, переходит в качение со скоростью 5/7 от начальной.
#[test]
fn sliding_sphere_settles_into_rolling_at_five_sevenths() {
    let mut r = room(2);
    r.set_param("mass", 16.0);
    r.run_for(0.5);
    assert!(r.numerically_ok());
    let (speed, want) = (r.weight_speed(), 3.0 * 5.0 / 7.0);
    println!("скорость качения {speed:.4} м/с, расчёт 5/7·v₀ = {want:.4}");
    assert!((speed - want).abs() < 0.01 * want, "{speed} против {want}");
}

#[test]
fn heavy_rolling_weight_pushes_light_box_and_barely_slows() {
    let mut r = room(2);
    r.set_param("mass", 16.0);
    r.run_for(1.2);
    assert!(r.numerically_ok());
    let (moved, speed) = (r.displacement("box"), r.weight_speed());
    println!("коробка сдвинулась на {:.1} см, гиря едет {speed:.2} м/с", moved * 100.0);
    assert!(moved > 0.1, "гиря 16 кг обязана сдвинуть коробку 0.8 кг");
    assert!(speed > 0.5, "лёгкая коробка не должна остановить гирю");
}

#[test]
fn light_rolling_weight_cannot_push_box_as_far() {
    let moved = |mass: f64| {
        let mut r = room(2);
        r.set_param("mass", mass);
        r.run_for(2.5);
        assert!(r.numerically_ok());
        r.displacement("box")
    };
    let (light, heavy) = (moved(0.5), moved(16.0));
    println!("коробку сдвинул груз 0.5 кг на {:.1} см, 16 кг на {:.1} см", light * 100.0, heavy * 100.0);
    assert!(heavy > 2.0 * light, "масса должна менять исход");
}

#[test]
fn side_table_tips_exactly_where_hand_calculation_says() {
    let crit = tip_mass();
    println!("порог опрокидывания по расчёту: {crit:.3} кг");
    for (factor, should_tip) in [(0.5, false), (0.8, false), (0.9, false), (1.1, true), (1.25, true), (2.0, true), (8.0, true)] {
        let mut r = room(3);
        r.set_param("mass", crit * factor);
        r.run_for(3.0);
        assert!(r.numerically_ok());
        let tilt = r.tilt_deg("side_table");
        println!("  груз {:.2} кг ({factor}× порога): наклон {tilt:.1}°", crit * factor);
        assert_eq!(tilt > 10.0, should_tip, "при {factor}× порога наклон {tilt:.1}°");
    }
}
