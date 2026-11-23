//! Приёмка конструктора через публичное API. Числа здесь — входы стендовых случаев,
//! параметры эталонной прошивки плат и геометрия для ручного расчёта.

use std::time::{Duration, Instant};

use sim_boards::Connection;
use sim_device::file::{from_text, to_text};
use sim_device::measure::{Measurement, Quantity};
use sim_device::*;
use sim_scenarios::blueprint::BuildVerdict;
use sim_scenarios::constructor::Constructor;
use sim_scenarios::device_assess::{
    DeviceReport, Group, Outcome, assess_device, assess_device_in, measurement_stamp,
    nominal_bending_pa, verdict,
};
use sim_scenarios::device_templates::{template, templates};
use sim_scenarios::{FormValue, Scenario};

mod support;
use support::*;

fn no_failure(r: &DeviceReport, group: Group, code: &str) {
    let evidence: Vec<_> = r
        .findings
        .iter()
        .filter(|f| f.group == group && f.code == code)
        .collect();
    assert!(
        !evidence.is_empty(),
        "В контрольной сборке нет проверки {code}"
    );
    assert!(
        evidence.iter().all(|f| f.outcome != Outcome::Fail),
        "{code}: {evidence:#?}"
    );
}

fn reject<F: FnOnce(&mut Device)>(base: Device, group: Group, code: &str, defect: F) {
    no_failure(&assess_device(&base), group, code);
    let mut bad = base;
    defect(&mut bad);
    let r = assess_device(&bad);
    assert_eq!(
        r.verdict(),
        BuildVerdict::Rejected,
        "{:#?}",
        r.findings
            .iter()
            .filter(|f| f.code == code)
            .collect::<Vec<_>>()
    );
    let reason: Vec<_> = r
        .findings
        .iter()
        .filter(|f| f.group == group && f.code == code)
        .collect();
    assert!(
        reason.iter().any(|f| f.outcome == Outcome::Fail),
        "{code}: {reason:#?}"
    );
}

#[test]
fn pla_across_layers() {
    reject(printed(), Group::Parts, "static_strength", |d| {
        d.links[0].print.as_mut().unwrap().across_layers = true
    });
}

#[test]
fn thin_link() {
    reject(device(), Group::Parts, "static_strength", |d| {
        d.links[0].section = Section::Rect {
            b_mm: 0.5,
            h_mm: 30.0,
        };
    });
}

#[test]
fn inadequate_torque_limit() {
    reject(moving(), Group::Actuators, "torque_limit", |d| {
        d.links[0].joint.torque_limit_nm = 0.1
    });
}

#[test]
fn sg90_on_arm() {
    reject(moving(), Group::Actuators, "peak_torque", |d| {
        d.links[0].joint.actuator = "sg90".into()
    });
}

#[test]
fn driver_current() {
    let mut d = moving();
    d.electronics.series = 3;
    d.electronics.driver = "bts7960".into();
    d.links[0].joint.actuator = "jga25-371-34-12v".into();
    // Из sim-actuators/tests/assess.rs: ток 2.2 А выше длительного предела L298N 2 А.
    let motor = sim_actuators::actuator_by_id(&d.links[0].joint.actuator).unwrap();
    let target = motor
        .motor_line(12.0, 80.0)
        .unwrap()
        .torque_for_current(2.2);
    let l = &mut d.links[0];
    let length_m = l.length_mm * sim_materials::catalog::MM;
    let rod_kg =
        sim_materials::MaterialId::Al6061T6.material().density * l.section.area_m2() * length_m;
    l.end_mass_kg = target / (sim_core::G_STANDARD * length_m) - rod_kg / 2.0;
    l.joint.speed_limit_dps = 0.01;
    reject(d, Group::Actuators, "driver_current", |d| {
        d.electronics.driver = "l298n".into()
    });
}

#[test]
fn violent_impact() {
    reject(moving(), Group::Contact, "impact", |d| {
        d.links[0].joint.speed_limit_dps = 800.0
    });
}

#[test]
fn excessive_clamp_force() {
    reject(moving(), Group::Contact, "clamp", |d| {
        d.links[0].contact_lever_mm = 10.0
    });
}

#[test]
fn gap_40_mm_at_hand() {
    reject(moving(), Group::Contact, "crush_gap", |d| {
        d.links[0].joint.pinch_gap_mm = 40.0
    });
}

#[test]
fn missing_fuse() {
    let good = device();
    no_failure(&assess_device(&good), Group::Battery, "short_circuit");
    let mut bad = good;
    bad.electronics.fuse_a = 0.0;
    let r = assess_device(&bad);
    assert_eq!(r.verdict(), BuildVerdict::Rejected);
    assert!(
        r.findings
            .iter()
            .any(|f| f.code == "no_fuse" && f.outcome == Outcome::Fail)
    );
}

#[test]
fn thin_wire() {
    reject(device(), Group::Battery, "wire_heating", |d| {
        d.electronics.settings.awg = Some(30)
    });
}

#[test]
fn five_volts_on_3v3_input() {
    let mut d = device();
    let good = assess_device(&d);
    assert!(
        !good
            .findings
            .iter()
            .any(|f| f.code == "overvoltage" && f.outcome == Outcome::Fail)
    );
    d.electronics.connections[2] = Wiring(Connection::DigitalInput {
        pin: 3,
        low_v: 0.0,
        high_v: 5.0,
    });
    let r = assess_device(&d);
    assert_eq!(r.verdict(), BuildVerdict::Rejected);
    assert!(
        r.findings
            .iter()
            .any(|f| f.code == "overvoltage" && f.outcome == Outcome::Fail)
    );
}

#[test]
fn motor_from_gpio() {
    reject(moving(), Group::Wiring, "direct_motor", |d| {
        *d.electronics.connections.last_mut().unwrap() = Wiring(Connection::Motor {
            pin: 2,
            direct: true,
        });
    });
}

#[test]
fn invalid_firmware_image() {
    let root = TempDir::new();
    let mut d = device();
    firmware(&mut d, &root.0);
    let good = assess_device_in(&d, &root.0);
    assert!(
        good.findings
            .iter()
            .any(|f| f.code == "firmware_behavior" && f.outcome == Outcome::Pass),
        "{:?}",
        good.findings
            .iter()
            .filter(|f| f.group == Group::Firmware)
            .collect::<Vec<_>>()
    );
    assert!(!good.uart.is_empty());
    // Machинный образ другого контроллера. Загрузка должна отклонить архитектуру.
    std::fs::write(
        root.0.join("pico.elf"),
        include_bytes!("../../sim-boards/testdata/avr.elf"),
    )
    .unwrap();
    let bad = assess_device_in(&d, &root.0);
    assert_eq!(bad.verdict(), BuildVerdict::Rejected);
    assert!(
        bad.findings
            .iter()
            .any(|f| f.code == "firmware_fault" && f.outcome == Outcome::Fail)
    );
}

#[test]
fn worse_measurement() {
    let mut d = device();
    let mass = assess_device(&d).analysis.unwrap().total_mass_kg;
    d.measurements.push(Measurement {
        quantity: Quantity::TotalMass,
        link: None,
        value: mass,
        note: "Весы стенда".into(),
        design: None,
    });
    reject(d, Group::Measurements, "measurement", |d| {
        d.measurements[0].value *= 2.0
    });
}

#[test]
fn hot_pla() {
    reject(printed(), Group::Parts, "material_temperature", |d| {
        d.usage.ambient_c = sim_materials::MaterialId::Pla
            .material()
            .softening_c
            .unwrap();
    });
}

#[test]
fn loose_bolt_near_edge() {
    reject(device(), Group::Parts, "fastener", |d| {
        d.links[0].settings.edge_mm = Some(1.0)
    });
}

#[test]
fn manual_millimeters_to_pascal() {
    let d = device();
    let rho = sim_materials::MaterialId::Al6061T6.material().density;
    let rod = rho * 0.012 * 0.030 * 0.300;
    let moment = (rod * 0.150 + 0.2 * 0.300) * sim_core::G_STANDARD;
    let expected_pa = 6.0 * moment / (0.030 * 0.012_f64.powi(2));
    let r = assess_device(&d);
    let finding = r
        .findings
        .iter()
        .find(|f| f.code == "bending_stress")
        .unwrap();
    assert!((r.analysis.unwrap().links[0].bending_nm - moment).abs() < 1e-12);
    assert!((nominal_bending_pa(&d.links[0], moment) - expected_pa).abs() < 1e-8);
    assert!((finding.value * 1e6 - expected_pa).abs() < 1e-8);
}

#[test]
fn unknown_inertia_blocks_impact() {
    let mut d = moving();
    let l = &mut d.links[0];
    l.settings.inertia_kg_m2 = None;
    let r = assess_device(&d);
    assert!(
        r.findings
            .iter()
            .any(|f| f.code == "impact" && f.outcome == Outcome::NotAnalyzed)
    );
    assert!(
        r.predictions
            .iter()
            .find(|p| p.quantity == Quantity::ImpactForce)
            .unwrap()
            .value
            .is_none()
    );
}

#[test]
fn templates_are_counted_saved_and_loaded() {
    assert_eq!(sim_scenarios::catalog()[0].id(), "constructor");
    let dir = TempDir::new();
    let mut c = Constructor::default();
    c.set_project_dir(dir.0.clone());
    assert_eq!(templates().len(), 7);
    for (i, meta) in templates().iter().enumerate() {
        let d = template(i);
        let r = assess_device(&d);
        assert!(r.analysis.is_some(), "{}", meta.title);
        assert_ne!(r.verdict(), BuildVerdict::PassedSimulation);
        for group in Group::ALL {
            assert!(
                r.findings.iter().any(|f| f.group == group),
                "{}: {group}",
                meta.title
            );
        }
        println!("{} {:?}", meta.title, r.verdict());
        for f in r.findings.iter().filter(|f| f.outcome != Outcome::Pass) {
            println!(
                "{} {} {:?} {}: {}",
                f.group, f.code, f.outcome, f.title, f.detail
            );
        }
        c.set_device(d.clone());
        let path = c.save().unwrap();
        c.load(path.file_stem().unwrap().to_str().unwrap()).unwrap();
        assert_eq!(c.device(), &d);
    }
}

fn wait(c: &mut Constructor) {
    let end = Instant::now() + Duration::from_secs(10);
    while c.is_checking() {
        assert!(Instant::now() < end, "Фоновая проверка не завершилась");
        std::thread::yield_now();
        c.poll_background();
    }
}

#[test]
fn editing_drops_completed_and_stale_background_report() {
    let mut c = Constructor::default();
    c.set_device(device());
    c.start_check();
    wait(&mut c);
    assert!(c.report().is_some());
    c.set_form("design.cold_c", FormValue::Text("".into()));
    assert!(c.report().is_none());
    c.start_check();
    assert!(c.is_checking());
    c.set_form(
        "project.name",
        FormValue::Text("Правка после старта".into()),
    );
    c.poll_background();
    assert!(!c.is_checking());
    assert!(c.report().is_none());
    c.start_check();
    wait(&mut c);
    assert!(
        c.report()
            .unwrap()
            .findings
            .iter()
            .any(|f| f.code == "cold" && f.outcome == Outcome::NotAnalyzed)
    );
    c.set_form("design.cold_c", FormValue::Text("не число".into()));
    c.start_check();
    assert!(!c.is_checking());
    assert!(c.report().is_none());
}

#[test]
fn projects_stay_inside_chosen_directory() {
    let root = TempDir::new();
    let mut c = Constructor::default();
    c.set_project_dir(root.0.join("projects"));
    let mut d = device();
    d.name = "../проект/\\опасное имя".into();
    c.set_device(d.clone());
    let p = c.save().unwrap();
    assert_eq!(p.parent(), Some(root.0.join("projects").as_path()));
    assert!(c.load("../elsewhere").is_err());
    assert!(c.load("/absolute").is_err());
    c.load(p.file_stem().unwrap().to_str().unwrap()).unwrap();
    assert_eq!(c.device(), &d);
    #[cfg(unix)]
    {
        use std::os::unix::fs::symlink;
        let outside = root.0.join("elsewhere.device");
        std::fs::write(&outside, to_text(&d)).unwrap();
        std::fs::remove_file(&p).unwrap();
        symlink(&outside, &p).unwrap();
        assert!(c.load(p.file_stem().unwrap().to_str().unwrap()).is_err());
        assert!(c.save().is_err());
        assert_eq!(std::fs::read_to_string(outside).unwrap(), to_text(&d));
    }
}

#[test]
fn extended_file_roundtrip_and_legacy() {
    let dir = TempDir::new();
    let mut d = printed();
    firmware(&mut d, &dir.0);
    d.links[0].settings.sn_a_mpa = Some(25.0);
    d.links[0].settings.sn_b = Some(-0.1);
    d.links[0].settings.sn_cycles = Some(1e5);
    d.links[0].settings.sn_source = Some("Стенд\nПротокол".into());
    d.electronics.sensors.push(SensorSettings {
        id: Some("as5600".into()),
        count: Some(2),
        on_regulator: Some(false),
        current_a: Some(0.02),
    });
    let text = to_text(&d);
    assert_eq!(from_text(&text).unwrap(), d);
    let mut old = d;
    old.settings = DeviceSettings::default();
    old.links[0].settings = LinkSettings::default();
    old.electronics.settings = ElectricalSettings::default();
    old.electronics.connections.clear();
    old.electronics.sensors.clear();
    old.electronics.firmware_cases.clear();
    old.electronics.firmware.clear();
    let legacy = to_text(&old).replace("# Формат sim-device 2\n", "");
    assert!(!legacy.contains("[design]"));
    let loaded = from_text(&legacy).unwrap();
    assert_eq!(loaded, old);
    let old_special_name =
        legacy.replacen("name = Стенд конструктора", "name = @:ручная запись", 1);
    assert_eq!(
        from_text(&old_special_name).unwrap().name,
        "@:ручная запись"
    );
    let r = assess_device(&loaded);
    assert!(
        r.findings
            .iter()
            .any(|f| f.code == "fatigue" && f.outcome == Outcome::NotAnalyzed)
    );
    assert!(
        r.findings
            .iter()
            .any(|f| f.code == "missing_firmware" && f.outcome == Outcome::NotAnalyzed)
    );
}

#[test]
fn nonfinite_input_is_rejected_everywhere() {
    let mut base = moving();
    base.measurements.push(Measurement {
        quantity: Quantity::TotalMass,
        link: None,
        value: 1.0,
        note: "Весы".into(),
        design: None,
    });
    base.electronics.sensors.push(SensorSettings::default());
    base.electronics
        .firmware_cases
        .push(FirmwareSettings::default());
    // Для типизированных секций перебираем все редактируемые поля, куда принимается f64.
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let token = bad.to_string();
        macro_rules! sweep {
            ($type:ty, $object:expr) => {
                for f in <$type>::FIELDS {
                    let mut d = base.clone();
                    let field_of: fn(&mut Device) -> &mut $type = $object;
                    let section = field_of(&mut d);
                    if f.numeric {
                        section.set(f.key, &token).unwrap();
                        assert_eq!(
                            assess_device(&d).verdict(),
                            BuildVerdict::Rejected,
                            "поле {}",
                            f.key
                        );
                    }
                }
            };
        }
        sweep!(DeviceSettings, |d: &mut Device| &mut d.settings);
        sweep!(LinkSettings, |d: &mut Device| &mut d.links[0].settings);
        sweep!(ElectricalSettings, |d: &mut Device| &mut d
            .electronics
            .settings);
        sweep!(SensorSettings, |d: &mut Device| &mut d.electronics.sensors
            [0]);
        sweep!(FirmwareSettings, |d: &mut Device| &mut d
            .electronics
            .firmware_cases[0]);
        let edits: &[fn(&mut Device, f64)] = &[
            |d, x| d.usage.ambient_c = x,
            |d, x| d.usage.cycles = x,
            |d, x| d.usage.hours_per_day = x,
            |d, x| d.links[0].length_mm = x,
            |d, x| d.links[0].hole_mm = x,
            |d, x| d.links[0].end_mass_kg = x,
            |d, x| d.links[0].contact_lever_mm = x,
            |d, x| d.links[0].offset_mm = [x; 3],
            |d, x| {
                d.links[0].section = Section::Rect {
                    b_mm: x,
                    h_mm: 30.0,
                }
            },
            |d, x| {
                d.links[0].section = Section::Rect {
                    b_mm: 12.0,
                    h_mm: x,
                }
            },
            |d, x| d.links[0].section = Section::Round { d_mm: x },
            |d, x| {
                d.links[0].section = Section::Tube {
                    od_mm: x,
                    id_mm: 1.0,
                }
            },
            |d, x| {
                d.links[0].section = Section::Tube {
                    od_mm: 2.0,
                    id_mm: x,
                }
            },
            |d, x| {
                d.links[0].print = Some(PrintSpec {
                    layer_mm: x,
                    walls: 4,
                    infill_pct: 20.0,
                    across_layers: false,
                })
            },
            |d, x| {
                d.links[0].print = Some(PrintSpec {
                    layer_mm: 0.2,
                    walls: 4,
                    infill_pct: x,
                    across_layers: false,
                })
            },
            |d, x| d.links[0].joint.min_deg = x,
            |d, x| d.links[0].joint.max_deg = x,
            |d, x| d.links[0].joint.speed_limit_dps = x,
            |d, x| d.links[0].joint.torque_limit_nm = x,
            |d, x| d.links[0].joint.pinch_gap_mm = x,
            |d, x| {
                d.links[0].joint = JointSpec::fixed();
                d.links[0].joint.speed_limit_dps = x;
            },
            |d, x| d.electronics.fuse_a = x,
            |d, x| d.electronics.wire_mm2 = x,
            |d, x| d.electronics.wire_m = x,
            |d, x| d.measurements[0].value = x,
        ];
        for (i, edit) in edits.iter().enumerate() {
            let mut d = base.clone();
            edit(&mut d, bad);
            assert_eq!(
                assess_device(&d).verdict(),
                BuildVerdict::Rejected,
                "поле {i}"
            );
        }
        for spec in [
            format!("supply mcu {token}"),
            format!("digital 3 {token} 3.3"),
            format!("digital 3 0 {token}"),
            format!("analog 26 {token} 3.3"),
            format!("analog 26 1 {token}"),
            format!("load 3 {token} false"),
            format!("led 3 {token} 1 3 false"),
            format!("led 3 300 {token} 3 false"),
            format!("led 3 300 1 {token} false"),
        ] {
            let mut d = base.clone();
            d.electronics
                .connections
                .push(Wiring::from_line(&spec).unwrap());
            assert_eq!(
                assess_device(&d).verdict(),
                BuildVerdict::Rejected,
                "{spec}"
            );
        }
    }
}

#[test]
fn every_group_and_real_input_is_mandatory() {
    let dir = TempDir::new();
    let mut d = device();
    firmware(&mut d, &dir.0);
    let first = assess_device_in(&d, &dir.0);
    let stamp = measurement_stamp(&d, &dir.0);
    for pred in first.predictions.iter().filter(|p| p.applicable) {
        d.measurements.push(Measurement {
            quantity: pred.quantity,
            link: pred.link,
            value: pred.value.expect("Для стенда есть предсказание"),
            note: "Эталонный замер контрольного стенда".into(),
            design: Some(stamp),
        });
    }
    let r = assess_device_in(&d, &dir.0);
    let blocked: Vec<_> = r
        .findings
        .iter()
        .filter(|f| f.outcome != Outcome::Pass)
        .collect();
    assert_eq!(r.verdict(), BuildVerdict::PassedSimulation, "{blocked:#?}");
    for g in Group::ALL {
        let filtered: Vec<_> = r
            .findings
            .iter()
            .filter(|f| f.group != g)
            .cloned()
            .collect();
        assert_eq!(verdict(&filtered), BuildVerdict::NotProven, "{g}");
    }
    let remove: &[(Group, fn(&mut Device))] = &[
        (Group::Mechanics, |d| d.settings.inventory_source = None),
        (Group::Parts, |d| d.links[0].settings.bolt_size = None),
        (Group::Actuators, |d| {
            d.links[0].joint = template(0).links.remove(0).joint;
        }),
        (Group::Contact, |d| d.settings.contact_source = None),
        (Group::Human, |d| d.mount = BodyRegion::Neck),
        (Group::Power, |d| d.electronics.settings.board_peak_a = None),
        (Group::Battery, |d| {
            d.electronics.settings.charge_max_c = None
        }),
        (Group::Wiring, |d| {
            d.electronics.connections.remove(1);
        }),
        (Group::Firmware, |d| d.electronics.firmware_cases.clear()),
        (Group::Measurements, |d| d.measurements.clear()),
    ];
    for (group, edit) in remove {
        let mut broken = d.clone();
        edit(&mut broken);
        let result = assess_device_in(&broken, &dir.0);
        assert_ne!(result.verdict(), BuildVerdict::PassedSimulation, "{group}");
        assert!(
            result
                .findings
                .iter()
                .any(|f| f.group == *group && f.outcome != Outcome::Pass),
            "{group}"
        );
    }
}

#[test]
fn printed_fatigue_requires_matching_sn_and_editing_invalidates_it() {
    let mut d = printed();
    d.links[0].material = "petg".into();
    d.links[0].end_mass_kg = 0.2;
    let check_fatigue = |d: &Device, expected| {
        let report = assess_device(d);
        let f = report
            .findings
            .iter()
            .find(|f| f.code == "fatigue")
            .unwrap();
        assert_eq!(f.outcome, expected, "{f:?}");
    };
    check_fatigue(&d, Outcome::NotAnalyzed);
    let s = &mut d.links[0].settings;
    // Эталонная кривая из sim-materials/tests/assess.rs. Она нужна для проверки передачи МПа.
    s.sn_a_mpa = Some(60.0);
    s.sn_b = Some(-0.08);
    s.sn_cycles = Some(1e6);
    s.sn_source = Some("Стендовый протокол PETG".into());
    check_fatigue(&d, Outcome::Pass);
    let mut c = Constructor::default();
    c.set_device(d);
    c.set_form("link.layer", FormValue::Number(0.25));
    check_fatigue(c.device(), Outcome::NotAnalyzed);
    c.set_form("sn.remove", FormValue::Pressed);
    assert_eq!(c.device().links[0].settings.sn_a_mpa, None);
    assert_eq!(c.device().links[0].settings.sn_cycles, None);
}

#[test]
fn assisted_arm_runs_lift_and_drive_loss() {
    let mut d = moving();
    let l = &mut d.links[0];
    l.settings.arm_action = Some("elbow".into());
    l.settings.assist_nm = Some(8.0);
    l.end_mass_kg = 12.0;
    l.joint.torque_limit_nm = 0.0;
    l.joint.speed_limit_dps = 1.0_f64.to_degrees();
    // Рабочие позы и антропометрия контрольного случая sim-human/tests/human.rs.
    d.settings.sex = Some("female".into());
    d.settings.strength_pct = Some(5.0);
    d.settings.height_mm = Some(1498.0);
    d.settings.mass_kg = Some(49.8);
    d.settings.age_years = Some(40.0);
    d.settings.shoulder_plane_deg = Some(90.0);
    d.settings.shoulder_elevation_deg = Some(30.0);
    d.settings.shoulder_rotation_deg = Some(0.0);
    d.settings.elbow_deg = Some(90.0);
    d.settings.forearm_deg = Some(0.0);
    d.settings.trunk_deg = Some(0.0);
    d.settings.lumbar_deg = Some(0.0);
    d.settings.lateral_deg = Some(0.0);
    d.settings.lumbar_rotation_deg = Some(0.0);
    d.settings.start_shoulder_deg = Some(5.0);
    d.settings.start_elbow_deg = Some(60.0);
    d.settings.hold_s = Some(0.0);
    d.settings.arm_lever_mm = Some(0.0);
    d.settings.loss_s = Some(0.03);
    d.settings.cycles_day = Some(1);
    let r = assess_device(&d);
    let h = r
        .human
        .as_ref()
        .expect("Модель человека фактически выполнена");
    let holding = h.holding.as_ref().expect("Рассчитано удержание");
    assert!(h.lift.is_some(), "{:?}", h.findings);
    let loss = h
        .failure
        .as_ref()
        .expect("Смоделирована полная потеря помощи");
    assert!(loss.motion.is_some(), "{:?}", h.findings);
    assert!(
        holding.muscle_nm[sim_human::C_ELBOW] < loss.unassisted_hold.muscle_nm[sim_human::C_ELBOW]
    );
    assert!(
        r.findings
            .iter()
            .any(|f| f.code == "drive_failure" && f.outcome == Outcome::Fail)
    );
}

#[test]
fn connections_edit_and_delete_without_kept_drafts() {
    let mut c = Constructor::default();
    c.set_device(device());
    let index = c.device().electronics.connections.len();
    c.set_form("connection.add", FormValue::Pressed);
    c.set_form(
        &format!("connection.{index}.spec"),
        FormValue::Text("неполная строка".into()),
    );
    c.start_check();
    assert!(!c.is_checking());
    c.set_form(
        &format!("connection.{index}.spec"),
        FormValue::Text("digital 4 0 5".into()),
    );
    c.start_check();
    wait(&mut c);
    assert!(
        c.report()
            .unwrap()
            .findings
            .iter()
            .any(|f| f.code == "overvoltage" && f.outcome == Outcome::Fail)
    );
    c.set_form(&format!("connection.{index}.delete"), FormValue::Pressed);
    assert_eq!(c.device().electronics.connections.len(), index);
    assert!(c.report().is_none());
}
