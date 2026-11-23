//! Модель устройства: файл проекта, механика против ручного расчёта и независимого RNEA,
//! удар и зажим, зазоры.

use nalgebra::DVector;
use sim_core::Vec3;
use sim_core::dynamics::rnea;
use sim_device::file::{from_text, to_text};
use sim_device::measure::{MeasureStatus, Measurement, Quantity, compare};
use sim_device::mechanics::{ActuatorData, Catalog, analyze, build};
use sim_device::model::*;
use sim_device::safety::{contact_results, crush_gaps, impact_force, safe_speed};

const G: f64 = 9.80665;

struct TestCatalog;

impl Catalog for TestCatalog {
    fn density_kg_m3(&self, material: &str) -> Option<f64> {
        match material {
            "al" => Some(2700.0),
            "pla" => Some(1240.0),
            _ => None,
        }
    }

    fn actuator(&self, id: &str) -> Option<ActuatorData> {
        (id == "drive").then_some(ActuatorData {
            mass_kg: 0.5,
            reflected_inertia_kg_m2: Some(1.4),
            peak_torque_nm: Some(100.0),
        })
    }
}

fn link(name: &str, parent: Option<usize>, length_mm: f64, end_mass_kg: f64) -> LinkSpec {
    LinkSpec {
        name: name.into(),
        parent,
        offset_mm: [0.0; 3],
        direction: Direction::Forward,
        length_mm,
        section: Section::Rect {
            b_mm: 10.0,
            h_mm: 20.0,
        },
        material: "al".into(),
        print: None,
        hole_mm: 0.0,
        end_mass_kg,
        near_body: BodyRegion::None,
        contact_lever_mm: 0.0,
        settings: Default::default(),
        joint: JointSpec {
            kind: JointKind::Hinge,
            axis: Axis::Y,
            min_deg: -90.0,
            max_deg: 90.0,
            actuator: "drive".into(),
            speed_limit_dps: 40.0,
            torque_limit_nm: 0.0,
            pinch_gap_mm: 150.0,
            guarded: false,
        },
    }
}

fn device(links: Vec<LinkSpec>) -> Device {
    Device {
        name: "Проверочное".into(),
        template: "test".into(),
        mount: BodyRegion::Torso,
        links,
        electronics: Electronics {
            board: "rp2040".into(),
            series: 6,
            parallel: 2,
            wire_mm2: 1.5,
            wire_m: 0.5,
            bms: true,
            fuse_a: 15.0,
            ..Electronics::default()
        },
        usage: UseProfile::default(),
        settings: Default::default(),
        measurements: Vec::new(),
    }
}

fn two_links() -> Device {
    let mut second = link("второе", Some(0), 200.0, 1.0);
    second.offset_mm = [300.0, 0.0, 0.0];
    second.print = Some(PrintSpec {
        layer_mm: 0.2,
        walls: 4,
        infill_pct: 40.0,
        across_layers: true,
    });
    second.material = "pla".into();
    second.near_body = BodyRegion::Hand;
    second.contact_lever_mm = 180.0;
    second.hole_mm = 5.0;
    device(vec![link("первое", None, 300.0, 0.0), second])
}

#[test]
fn project_file_round_trips() {
    let mut d = two_links();
    d.measurements = vec![
        Measurement {
            quantity: Quantity::TotalMass,
            link: None,
            value: 1.25,
            note: "кухонные весы".into(),
            design: Some(0x1234),
        },
        Measurement {
            quantity: Quantity::ClampForce,
            link: Some(1),
            value: 118.0,
            note: "динамометр".into(),
            design: None,
        },
    ];
    let text = to_text(&d);
    assert_eq!(from_text(&text).unwrap(), d);
}

fn m(quantity: Quantity, value: f64) -> Measurement {
    Measurement {
        quantity,
        link: quantity.per_link().then_some(0),
        value,
        note: String::new(),
        design: None,
    }
}

#[test]
fn measurement_worse_than_simulation_is_flagged() {
    // Сила: больше расчёта — опасно.
    assert_eq!(
        compare(&m(Quantity::ClampForce, 100.0), Some(100.0)).status,
        MeasureStatus::Agrees
    );
    assert_eq!(
        compare(&m(Quantity::ClampForce, 109.0), Some(100.0)).status,
        MeasureStatus::Agrees
    );
    assert_eq!(
        compare(&m(Quantity::ClampForce, 111.0), Some(100.0)).status,
        MeasureStatus::RealIsWorse
    );
    assert_eq!(
        compare(&m(Quantity::ClampForce, 80.0), Some(100.0)).status,
        MeasureStatus::RealIsBetter
    );
    // Момент привода: меньше паспорта — опасно.
    assert_eq!(
        compare(&m(Quantity::ActuatorPeakTorque, 80.0), Some(100.0)).status,
        MeasureStatus::RealIsWorse
    );
    assert_eq!(
        compare(&m(Quantity::ActuatorPeakTorque, 130.0), Some(100.0)).status,
        MeasureStatus::RealIsBetter
    );
    // Температура у кожи: допуск абсолютный, 2 °C.
    assert_eq!(
        compare(&m(Quantity::SkinSurfaceTemperature, 41.5), Some(40.0)).status,
        MeasureStatus::Agrees
    );
    assert_eq!(
        compare(&m(Quantity::SkinSurfaceTemperature, 42.5), Some(40.0)).status,
        MeasureStatus::RealIsWorse
    );
    // Сломанные числа безопасными не считаются.
    assert_eq!(
        compare(&m(Quantity::ClampForce, f64::NAN), Some(100.0)).status,
        MeasureStatus::RealIsWorse
    );
    assert_eq!(
        compare(&m(Quantity::ClampForce, 100.0), None).status,
        MeasureStatus::NoPrediction
    );
    assert_eq!(
        compare(&m(Quantity::ClampForce, 100.0), Some(f64::NAN)).status,
        MeasureStatus::RealIsWorse
    );
}

#[test]
fn measurement_must_point_to_a_real_link() {
    let mut d = two_links();
    d.measurements = vec![Measurement {
        quantity: Quantity::ClampForce,
        link: None,
        value: 1.0,
        note: String::new(),
        design: None,
    }];
    assert!(d.validate().iter().any(|e| e.contains("укажите звено")));
    d.measurements[0].link = Some(7);
    assert!(d.validate().iter().any(|e| e.contains("нет такого звена")));
}

#[test]
fn project_file_reports_mistakes_with_a_line() {
    let good = to_text(&two_links());
    for (from, to, expect) in [
        ("length_mm = 300", "length_mm = abc", "должно быть числом"),
        ("length_mm = 300", "length_mm = NaN", "должно быть числом"),
        ("hole_mm = 5", "hole_mn = 5", "hole"),
        ("direction = forward", "direction = sideways", "направление"),
        ("parent = 1", "parent = 9", "родителем"),
        ("guarded = false", "guarded = maybe", "true или false"),
        ("length_mm = 300", "length_mm = -5", "длина"),
    ] {
        let bad = good.replacen(from, to, 1);
        assert_ne!(bad, good, "замена «{from}» не сработала");
        let e = from_text(&bad).expect_err(&format!("«{to}» должно быть ошибкой"));
        assert!(e.contains(expect), "«{to}»: {e}");
    }
    let duplicated = good.replacen("hole_mm = 0", "hole_mm = 0\nhole_mm = 1", 1);
    assert!(from_text(&duplicated).unwrap_err().contains("дважды"));
}

#[test]
fn one_link_matches_hand_calculation() {
    let d = device(vec![link("штанга", None, 300.0, 2.0)]);
    let a = analyze(&d, &TestCatalog).unwrap();
    let rod = 2700.0 * 10e-3 * 20e-3 * 0.3;
    let want_gravity = (rod * 0.15 + 2.0 * 0.3) * G;
    let want_inertia = 1.4 + rod * 0.3 * 0.3 / 3.0 + 2.0 * 0.3 * 0.3;
    let j = a.joints[0];
    println!(
        "момент {:.4} (расчёт {want_gravity:.4}), инерция {:.4} (расчёт {want_inertia:.4})",
        j.gravity_nm, j.inertia_kg_m2
    );
    assert!((a.total_mass_kg - (rod + 2.0 + 0.5)).abs() < 1e-9);
    assert!((j.gravity_nm - want_gravity).abs() < 1e-9);
    assert!((j.inertia_kg_m2 - want_inertia).abs() < 2e-3 * want_inertia);
    let inertial = want_inertia * 40f64.to_radians() / 0.25;
    assert!((j.inertial_nm - inertial).abs() < 2e-3 * inertial);
}

/// Независимая проверка: RNEA с силой тяжести, направленной в найденную худшую сторону,
/// обязан дать тот же момент, а ни одно другое направление — больше.
#[test]
fn worst_gravity_matches_rnea() {
    let d = two_links();
    let a = analyze(&d, &TestCatalog).unwrap();
    let (mut built, _) = build(&d, &TestCatalog).unwrap();
    let zero = DVector::zeros(2);
    let mut best = 0.0_f64;
    for k in 0..=72 {
        let t = k as f64 * 5f64.to_radians();
        built.model.gravity = Vec3::new(t.cos(), 0.0, t.sin()) * G;
        for s in [-90.0_f64, -45.0, 0.0, 45.0, 90.0] {
            let q = DVector::from_vec(vec![0.0, s.to_radians()]);
            best = best.max(rnea(&built.model, &q, &zero, &zero, &[])[0].abs());
        }
    }
    let ours = a.joints[0].gravity_nm;
    println!("худший момент: наш {ours:.4}, перебор RNEA {best:.4}");
    assert!(ours >= best - 1e-6, "наш расчёт занизил момент");
    assert!(ours.is_finite());
}

#[test]
fn branches_do_not_load_each_other() {
    let mut l = link("правая", Some(0), 200.0, 1.0);
    l.offset_mm = [100.0, 0.0, 0.0];
    let mut r = link("левая", Some(0), 200.0, 5.0);
    r.offset_mm = [100.0, 50.0, 0.0];
    let mut base = link("база", None, 100.0, 0.0);
    base.joint = JointSpec::fixed();
    let d = device(vec![base, l, r]);
    let a = analyze(&d, &TestCatalog).unwrap();
    let (left, right) = (a.joints[0], a.joints[1]);
    assert_eq!((left.link, right.link), (1, 2));
    let rod = 2700.0 * 10e-3 * 20e-3 * 0.2;
    assert!(
        (left.gravity_nm - (rod * 0.1 + 0.2) * G).abs() < 1e-9,
        "{}",
        left.gravity_nm
    );
    assert!(
        (right.gravity_nm - (rod * 0.1 + 1.0) * G).abs() < 1e-9,
        "{}",
        right.gravity_nm
    );
}

#[test]
fn conservative_bounds_cover_large_tree() {
    let mut links = vec![link("0", None, 100.0, 0.1)];
    for i in 1..9 {
        let mut l = link(&i.to_string(), Some(i - 1), 100.0, 0.1);
        l.offset_mm = [100.0, 0.0, 0.0];
        links.push(l);
    }
    let a = analyze(&device(links), &TestCatalog).unwrap();
    assert_eq!(a.joints.len(), 9);
    assert!(a.joints[0].gravity_nm > a.joints[8].gravity_nm);
    assert!(
        a.joints
            .iter()
            .all(|j| j.required_nm.is_finite() && j.required_nm > 0.0)
    );
}

#[test]
fn impact_and_clamp_match_formulas() {
    let d = two_links();
    let a = analyze(&d, &TestCatalog).unwrap();
    let r = contact_results(&d, &a, &TestCatalog);
    assert_eq!(r.len(), 2, "оба шарнира ведут звено рядом с кистью");
    for c in &r {
        let j = a.joints.iter().find(|j| j.link == c.joint_link).unwrap();
        let want = 40f64.to_radians() * (75_000.0 * j.inertia_kg_m2).sqrt();
        assert!((c.impact_n.unwrap() - want).abs() < 1e-9);
        assert_eq!(c.impact_limit_n, 280.0);
        assert!((c.clamp_n.unwrap() - (100.0 + j.gravity_nm) / 0.18).abs() < 1e-9);
        assert_eq!(c.clamp_limit_n, 140.0);
    }
    let w = safe_speed(280.0, 75_000.0, 1.44);
    assert!((impact_force(w, 75_000.0, 1.44) - 280.0).abs() < 1e-9);
}

#[test]
fn firmware_torque_limit_reduces_clamp() {
    let mut d = two_links();
    d.links[1].joint.torque_limit_nm = 20.0;
    let a = analyze(&d, &TestCatalog).unwrap();
    let r = contact_results(&d, &a, &TestCatalog);
    let c = r.iter().find(|c| c.joint_link == 1).unwrap();
    assert!((c.clamp_n.unwrap() - (20.0 + a.joints[1].gravity_nm) / 0.18).abs() < 1e-9);
}

#[test]
fn crush_gaps_follow_iso_13854() {
    let mut d = two_links();
    d.links[1].joint.pinch_gap_mm = 60.0;
    let g = crush_gaps(&d);
    assert_eq!(g.len(), 1);
    assert_eq!(g[0].required_mm, 100.0);
    assert!(!g[0].ok(), "60 мм меньше 100 мм для кисти");
    d.links[1].joint.pinch_gap_mm = 100.0;
    assert!(crush_gaps(&d)[0].ok());
    d.links[1].joint.pinch_gap_mm = f64::NAN;
    assert!(!crush_gaps(&d)[0].ok(), "NaN — не зазор");
    d.links[1].joint.pinch_gap_mm = 10.0;
    d.links[1].joint.guarded = true;
    assert!(crush_gaps(&d)[0].ok());
}

#[test]
fn unknown_parts_and_nonsense_are_errors() {
    let mut d = two_links();
    d.links[0].material = "unobtainium".into();
    assert!(
        analyze(&d, &TestCatalog)
            .unwrap_err()
            .iter()
            .any(|e| e.contains("материала"))
    );
    let mut d = two_links();
    d.links[1].joint.actuator = "warp".into();
    assert!(
        analyze(&d, &TestCatalog)
            .unwrap_err()
            .iter()
            .any(|e| e.contains("привода"))
    );
    let mut d = two_links();
    d.links[0].parent = Some(1);
    assert!(!d.validate().is_empty());
    let mut d = two_links();
    d.links[1].joint.speed_limit_dps = 0.0;
    assert!(d.validate().iter().any(|e| e.contains("предел скорости")));
    let mut d = two_links();
    d.links[0].section = Section::Tube {
        od_mm: 10.0,
        id_mm: 12.0,
    };
    assert!(d.validate().iter().any(|e| e.contains("сечение")));
}
