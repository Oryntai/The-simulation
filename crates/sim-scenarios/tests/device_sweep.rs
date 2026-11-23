//! Перебор полей от полностью проходящего контроля: что остаётся «прошёл» при порче значения.

use sim_device::measure::Measurement;
use sim_device::*;
use sim_scenarios::blueprint::BuildVerdict;
use sim_scenarios::device_assess::{
    Group, Outcome, Readiness, assess_device, assess_device_in, measurement_stamp,
};
use sim_scenarios::device_templates::{template, templates};

mod support;
use support::*;

fn passing(dir: &std::path::Path) -> Device {
    let mut d = device();
    firmware(&mut d, dir);
    let first = assess_device_in(&d, dir);
    let stamp = measurement_stamp(&d, dir);
    for p in first.predictions.iter().filter(|p| p.applicable) {
        d.measurements.push(Measurement {
            quantity: p.quantity,
            link: p.link,
            value: p.value.expect("предсказание"),
            note: "Эталонный замер контрольного стенда".into(),
            design: Some(stamp),
        });
    }
    assert_eq!(assess_device_in(&d, dir).verdict(), BuildVerdict::PassedSimulation);
    d
}

/// Порча, после которой контроль законно остаётся годным: значение реально и безопасно.
const LEGITIMATE: [&str; 7] = [
    "electric.fuse_position_m = 0",      // предохранитель прямо у клеммы
    "electric.enclosure_body_fraction = 0", // устройство не на теле
    "link.hole = 0",                     // отверстия нет
    // Расчёт — верхняя граница, реальная величина вдвое ниже неё допустима.
    "замер link_deflection × 0.5",
    "замер board_temperature × 0.5",
    "замер battery_temperature × 0.5",
    "замер battery_current × 0.5",
];

#[test]
fn no_corrupted_value_keeps_the_pass() {
    let dir = TempDir::new();
    let base = passing(&dir.0);
    let values = ["0", "-1", "0.000001", "1000000000"];
    let mut alive = Vec::new();
    macro_rules! sweep {
        ($name:literal, $ty:ty, $get:expr) => {
            for f in <$ty>::FIELDS.iter().filter(|f| f.numeric) {
                for v in values {
                    let mut d = base.clone();
                    let get: fn(&mut Device) -> &mut $ty = $get;
                    get(&mut d).set(f.key, v).unwrap();
                    if assess_device_in(&d, &dir.0).verdict() == BuildVerdict::PassedSimulation {
                        alive.push(format!("{}.{} = {v}", $name, f.key));
                    }
                }
            }
        };
    }
    sweep!("device", DeviceSettings, |d| &mut d.settings);
    sweep!("link", LinkSettings, |d| &mut d.links[0].settings);
    sweep!("electric", ElectricalSettings, |d| &mut d.electronics.settings);
    sweep!("firmware", FirmwareSettings, |d| &mut d.electronics.firmware_cases[0]);
    let edits: Vec<(&str, fn(&mut Device, f64))> = vec![
        ("usage.ambient_c", |d, x| d.usage.ambient_c = x),
        ("usage.cycles", |d, x| d.usage.cycles = x),
        ("usage.hours", |d, x| d.usage.hours_per_day = x),
        ("link.length", |d, x| d.links[0].length_mm = x),
        ("link.hole", |d, x| d.links[0].hole_mm = x),
        ("link.end_mass", |d, x| d.links[0].end_mass_kg = x),
        ("link.lever", |d, x| d.links[0].contact_lever_mm = x),
        ("el.fuse", |d, x| d.electronics.fuse_a = x),
        ("el.wire_mm2", |d, x| d.electronics.wire_mm2 = x),
        ("el.wire_m", |d, x| d.electronics.wire_m = x),
    ];
    for (name, edit) in edits {
        for x in [0.0, -1.0, 1e-6, 1e9] {
            let mut d = base.clone();
            edit(&mut d, x);
            if assess_device_in(&d, &dir.0).verdict() == BuildVerdict::PassedSimulation {
                alive.push(format!("{name} = {x}"));
            }
        }
    }
    for m in 0..base.measurements.len() {
        for factor in [0.0, 0.5] {
            let mut d = base.clone();
            d.measurements[m].value *= factor;
            if assess_device_in(&d, &dir.0).verdict() == BuildVerdict::PassedSimulation {
                alive.push(format!("замер {} × {factor}", d.measurements[m].quantity.key()));
            }
        }
    }
    let mut heavier = base.clone();
    heavier.links[0].end_mass_kg *= 3.0;
    if assess_device_in(&heavier, &dir.0).verdict() == BuildVerdict::PassedSimulation {
        alive.push("старые замеры после утяжеления ×3".into());
    }
    alive.retain(|a| !LEGITIMATE.contains(&a.as_str()));
    assert!(alive.is_empty(), "Порча не сняла «прошёл»:\n{}", alive.join("\n"));
}

fn has(r: &sim_scenarios::device_assess::DeviceReport, code: &str, outcome: Outcome) -> bool {
    r.findings.iter().any(|f| f.code == code && f.outcome == outcome)
}

#[test]
fn stale_measurement_proves_nothing() {
    let dir = TempDir::new();
    let mut d = passing(&dir.0);
    d.links[0].end_mass_kg *= 1.5;
    d.settings.accessory_mass_kg = Some(0.2);
    let r = assess_device_in(&d, &dir.0);
    assert_ne!(r.verdict(), BuildVerdict::PassedSimulation);
    assert!(r.measures.iter().any(|m| m.detail.contains("другой версии")));
}

#[test]
fn replaced_firmware_invalidates_measurements() {
    let dir = TempDir::new();
    let d = passing(&dir.0);
    let path = dir.0.join("pico.elf");
    let mut image = std::fs::read(&path).unwrap();
    *image.last_mut().unwrap() ^= 0xff;
    std::fs::write(&path, image).unwrap();
    assert_ne!(
        assess_device_in(&d, &dir.0).verdict(),
        BuildVerdict::PassedSimulation
    );
}

#[test]
fn stall_torque_loads_the_part() {
    // Лёгкое звено из PLA: вес ему не страшен, а пик привода ломает.
    let mut d = printed();
    d.links[0].section = Section::Rect { b_mm: 6.0, h_mm: 6.0 };
    d.links[0].end_mass_kg = 0.2;
    d.links[0].settings.bbox_y_mm = Some(6.0);
    d.links[0].settings.bbox_z_mm = Some(6.0);
    let calm = assess_device_in(&d, &std::env::temp_dir());
    assert!(!has(&calm, "static_strength", Outcome::Fail));
    let drive = template(0).links.remove(0).joint;
    d.links[0].joint = drive;
    d.links[0].joint.speed_limit_dps = 1.0;
    d.links[0].joint.torque_limit_nm = 0.5;
    d.links[0].settings.motor_part_c = Some(d.usage.ambient_c);
    let r = assess_device_in(&d, &std::env::temp_dir());
    assert!(has(&r, "static_strength", Outcome::Fail), "{:#?}", r.findings);
}

#[test]
fn plastic_near_motor_needs_its_temperature() {
    let mut d = printed();
    d.links[0].section = Section::Rect { b_mm: 20.0, h_mm: 20.0 };
    d.links[0].end_mass_kg = 0.2;
    d.links[0].joint = template(0).links.remove(0).joint;
    d.links[0].joint.speed_limit_dps = 1.0;
    let dir = std::env::temp_dir();
    assert!(has(&assess_device_in(&d, &dir), "motor_heat", Outcome::NotAnalyzed));
    let softening = sim_materials::MaterialId::Pla.material().softening_c.unwrap();
    d.links[0].settings.motor_part_c = Some(softening);
    assert!(has(
        &assess_device_in(&d, &dir),
        "material_temperature",
        Outcome::Fail
    ));
}

#[test]
fn impossible_sn_curve_is_rejected() {
    let mut d = printed();
    d.links[0].settings.sn_a_mpa = Some(2500.0);
    d.links[0].settings.sn_b = Some(-0.05);
    d.links[0].settings.sn_cycles = Some(1e7);
    d.links[0].settings.sn_source = Some("Протокол".into());
    let r = assess_device_in(&d, &std::env::temp_dir());
    assert!(has(&r, "sn_plausibility", Outcome::Fail));
}

#[test]
fn contact_point_cannot_leave_the_link() {
    let mut d = moving();
    d.links[0].contact_lever_mm = d.links[0].length_mm * 10.0;
    assert!(d.validate().iter().any(|e| e.contains("дальше конца")));
    let mut d = device();
    d.links[0].settings.bolt_lever_mm = Some(d.links[0].length_mm * 10.0);
    assert!(d.validate().iter().any(|e| e.contains("болтовой")));
}

#[test]
fn hinge_upstream_of_the_body_needs_a_zone() {
    let mut d = moving();
    d.mount = BodyRegion::None;
    d.links[0].near_body = BodyRegion::None;
    let mut child = d.links[0].clone();
    child.name = "Кисть".into();
    child.parent = Some(0);
    child.offset_mm = [0.0, 0.0, -d.links[0].length_mm];
    child.joint = JointSpec::fixed();
    child.near_body = BodyRegion::Hand;
    child.contact_lever_mm = child.length_mm;
    d.links.push(child);
    let r = assess_device_in(&d, &std::env::temp_dir());
    assert!(
        r.findings.iter().any(|f| f.code == "contact_coverage"
            && f.link == Some(0)
            && f.outcome == Outcome::NotAnalyzed),
        "{:#?}",
        r.findings.iter().filter(|f| f.group == Group::Contact).collect::<Vec<_>>()
    );
}

#[test]
fn idle_firmware_is_not_a_response() {
    let dir = TempDir::new();
    let mut d = device();
    firmware(&mut d, &dir.0);
    let case = &mut d.electronics.firmware_cases[0];
    case.pwm_min = Some(0.0);
    case.pwm_max = Some(1.0);
    let r = assess_device_in(&d, &dir.0);
    assert!(has(&r, "firmware_response", Outcome::NotAnalyzed));
}

#[test]
fn bench_stage_never_hides_a_defect() {
    // Ступень «на стенд» не появляется при нарушении и при дырке в самой конструкции.
    let dir = TempDir::new();
    let mut d = device();
    firmware(&mut d, &dir.0);
    assert_eq!(assess_device_in(&d, &dir.0).readiness(), Readiness::BenchOnly);
    let mut broken = d.clone();
    broken.electronics.fuse_a = 0.0;
    assert_eq!(assess_device_in(&broken, &dir.0).readiness(), Readiness::Rejected);
    let mut unknown = d.clone();
    unknown.electronics.settings.board_peak_a = None;
    assert_eq!(assess_device_in(&unknown, &dir.0).readiness(), Readiness::NeedsWork);
    let mut no_firmware = d;
    no_firmware.electronics.firmware.clear();
    assert_eq!(assess_device_in(&no_firmware, &dir.0).readiness(), Readiness::NeedsWork);
}

/// Путь пользователя: готовая схема без нарушений, но «на стенд» только после своих протоколов.
#[test]
fn working_template_reaches_the_bench_only_with_evidence() {
    let index = templates().iter().position(|t| t.id == "elbow-24v").unwrap();
    let mut d = template(index);
    let r = assess_device(&d);
    let failures: Vec<_> = r.findings.iter().filter(|f| f.outcome == Outcome::Fail).collect();
    assert!(failures.is_empty(), "{failures:#?}");
    assert_eq!(r.readiness(), Readiness::NeedsWork);
    d.settings.inventory_source = Some("Ведомость масс".into());
    d.links[1].settings.inertia_source = Some("Маятник".into());
    d.electronics.settings.board_source = Some("Амперметр и термопара".into());
    d.electronics.settings.sequencing_source = Some("Осциллограммы пуска".into());
    d.electronics.settings.sensors_declared = Some(true);
    let r = assess_device(&d);
    let left: Vec<_> = r
        .findings
        .iter()
        .filter(|f| f.outcome == Outcome::NotAnalyzed && f.group != Group::Measurements)
        .map(|f| format!("{} {} {}", f.group, f.code, f.title))
        .collect();
    assert_eq!(r.readiness(), Readiness::BenchOnly, "{left:#?}");
    // Стенд не равен допуску на человека.
    assert_eq!(r.verdict(), BuildVerdict::NotProven);
    let mut hot = d.clone();
    hot.links[1].joint.speed_limit_dps = 2000.0;
    assert_eq!(assess_device(&hot).readiness(), Readiness::Rejected);
}
