//! Постоянные магниты: марки, поле, силы, датчик угла.
use sim_em::field::{FieldSource, Vec3};
use sim_em::magnet::*;

#[test]
fn grades_and_temperature() {
    let g = ndfeb_grade("N52").unwrap();
    assert!(g.br_min_t < g.br_typical_t() && g.br_typical_t() < g.br_max_t);
    assert_eq!(NDFEB_GRADES.len(), 8);
    // при 80 °C Br падает на 7.2 % (−0.12 %/К от 20 °C)
    let b80 = br_at_temperature(1.3, 80.0);
    assert!((b80 / 1.3 - 0.928).abs() < 1e-12);
    assert_eq!(temp_class_of("N38UH").unwrap().max_operating_c, 180.0);
    assert!(temp_class_of("N42XX").is_none());
}

#[test]
fn pull_force_decreases_with_gap() {
    let (br, r, l) = (1.3, 5e-3, 5e-3);
    let f0 = pull_force_ideal_steel(br, r, l, 0.0);
    let f1 = pull_force_ideal_steel(br, r, l, 0.5e-3);
    let f2 = pull_force_ideal_steel(br, r, l, 2e-3);
    assert!(f0 > f1 && f1 > f2 && f2 > 0.0, "{f0} {f1} {f2}");
    assert!(f0 < contact_pull_upper_bound(br, r));
}

#[test]
fn magnet_field_symmetry() {
    let m = AxialCylinderMagnet::new(Vec3::new(0.1, 0.0, 0.0), Vec3::y(), 4e-3, 6e-3, 1.3);
    // поле на оси над северным торцом направлено по оси
    let b = m.b_at(&Vec3::new(0.1, 6e-3, 0.0));
    assert!(b.y > 0.0 && b.x.abs() < 1e-12 && b.z.abs() < 1e-12);
    // зеркальная симметрия относительно середины
    let b_up = m.b_at(&Vec3::new(0.105, 4e-3, 0.001));
    let b_dn = m.b_at(&Vec3::new(0.105, -4e-3, 0.001));
    assert!((b_up.y - b_dn.y).abs() < 1e-12 && (b_up.x + b_dn.x).abs() < 1e-12);
}

#[test]
fn as5600_magnet_and_gap_window() {
    // магнит Ø6×2.5 N35 диаметральный — окно зазоров для 30–90 мТ
    let (lo, hi) = sensor_gap_window(3e-3, 2.5e-3, 1.19, &AS5600).expect("поля хватает");
    assert!(lo < hi);
    // окно пересекается с рекомендацией даташита 0.5–3 мм
    assert!(lo < 3e-3 && hi > 0.5e-3, "{lo} {hi}");
    let b_mid = sensor_bz_amplitude(3e-3, 2.5e-3, 1.19, 0.5 * (lo + hi), AS5600.hall_radius_m);
    assert!(b_mid > AS5600.bz_min_t && b_mid < AS5600.bz_max_t);
}

#[test]
fn magnet_checks_pass() {
    for c in sim_em::magnet::checks() {
        c.assert_pass();
    }
}
