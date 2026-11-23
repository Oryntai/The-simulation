//! Каталог: полнота, источники у каждого числа, согласованность с паспортами.

use std::collections::HashSet;

use sim_actuators::{
    Backdrive, DriverKind, KGF_CM, Provenance, Spec, actuator_by_id, actuators, driver_by_id,
    drivers, external_drivers, regulator_by_id, regulators, sensor_by_id, sensors,
};

fn assert_sourced(part: &str, field: &str, s: Spec) {
    match s.value {
        Some(v) => {
            assert!(v.is_finite(), "{part}.{field}: не конечное");
            assert_ne!(s.provenance, Provenance::Missing, "{part}.{field}");
            assert!(!s.source.trim().is_empty(), "{part}.{field}: нет источника");
        }
        None => assert_eq!(s.provenance, Provenance::Missing, "{part}.{field}"),
    }
}

fn no_ru_domain(text: &str) {
    let lower = text.to_lowercase();
    for token in lower.split(|c: char| c.is_whitespace() || c == ',' || c == ';') {
        if let Some(rest) = token.split("://").nth(1) {
            let host = rest.split('/').next().unwrap_or("");
            assert!(!host.ends_with(".ru"), "ссылка в зоне .ru: {token}");
        }
    }
}

#[test]
fn required_parts_are_present() {
    for id in [
        "maxon-ec90-hd-csg25-50",
        "cubemars-ak80-64",
        "cubemars-ak60-6",
        "cubemars-ak70-10",
        "dynamixel-xl330-m288",
        "dynamixel-xl430-w250",
        "dynamixel-xm430-w350",
        "feetech-sts3215",
        "sg90",
        "mg90s",
        "mg996r",
        "ds3218",
        "n20-100-6v-hp",
        "n20-298-6v-hp",
        "jga25-371-34-12v",
        "nema17-17hs19-2004s1",
    ] {
        assert!(actuator_by_id(id).is_some(), "нет привода {id}");
    }
    for id in [
        "drv8833",
        "drv8833-pw",
        "tb6612fng",
        "l298n",
        "bts7960",
        "drv8825",
        "a4988",
        "escon-50-5",
        "builtin-dynamixel-xl330",
        "builtin-dynamixel-xl430",
        "builtin-dynamixel-xm430",
        "builtin-feetech-sts3215",
        "builtin-hobby-servo",
        "builtin-cubemars-ak80",
        "builtin-cubemars-ak70",
        "builtin-cubemars-ak60",
    ] {
        assert!(driver_by_id(id).is_some(), "нет драйвера {id}");
    }
    for id in [
        "ams1117-3.3",
        "ams1117-5.0",
        "lm2596-adj",
        "mp1584",
        "tps5430",
    ] {
        assert!(regulator_by_id(id).is_some(), "нет стабилизатора {id}");
    }
    for id in [
        "as5600",
        "as5048a",
        "amt102-v",
        "mpu6050",
        "bmi270",
        "vl53l0x",
        "hx711",
        "tal220b-5kg",
        "fsr402",
        "ina219",
    ] {
        assert!(sensor_by_id(id).is_some(), "нет датчика {id}");
    }
    assert!(actuator_by_id("нет такого").is_none());
    assert!(driver_by_id("нет такого").is_none());
    assert!(regulator_by_id("нет такого").is_none());
    assert!(sensor_by_id("нет такого").is_none());
}

#[test]
fn ids_are_unique_and_texts_filled() {
    let mut seen = HashSet::new();
    for a in actuators() {
        assert!(seen.insert(a.id), "повтор {}", a.id);
        assert!(!a.name.is_empty() && !a.sources.is_empty(), "{}", a.id);
        assert!(
            a.what_is.chars().any(|c| ('а'..='я').contains(&c)),
            "{}",
            a.id
        );
    }
    for d in drivers() {
        assert!(seen.insert(d.id), "повтор {}", d.id);
        assert!(!d.sources.is_empty(), "{}", d.id);
        assert!(
            d.what_is.chars().any(|c| ('а'..='я').contains(&c)),
            "{}",
            d.id
        );
    }
    for r in regulators() {
        assert!(seen.insert(r.id), "повтор {}", r.id);
        assert!(!r.sources.is_empty(), "{}", r.id);
        assert!(
            r.what_is.chars().any(|c| ('а'..='я').contains(&c)),
            "{}",
            r.id
        );
    }
    for s in sensors() {
        assert!(seen.insert(s.id), "повтор {}", s.id);
        assert!(!s.sources.is_empty(), "{}", s.id);
        assert!(
            s.what_is.chars().any(|c| ('а'..='я').contains(&c)),
            "{}",
            s.id
        );
    }
}

#[test]
fn every_number_has_a_source() {
    for a in actuators() {
        for (f, s) in a.numeric_specs() {
            assert_sourced(a.id, f, s);
            assert!(s.value.is_none_or(|v| v >= 0.0), "{}.{f}", a.id);
        }
        for (f, s) in a.temperature_specs() {
            assert_sourced(a.id, f, s);
        }
    }
    for d in drivers() {
        for (f, s) in d.numeric_specs() {
            assert_sourced(d.id, f, s);
        }
        for (f, s) in d.temperature_specs() {
            assert_sourced(d.id, f, s);
        }
    }
    for r in regulators() {
        for (f, s) in r.numeric_specs() {
            assert_sourced(r.id, f, s);
        }
    }
    for s in sensors() {
        for (f, x) in [
            ("current_a", s.current_a),
            ("current_peak_a", s.current_peak_a),
            ("range_min", s.range_min),
            ("range_max", s.range_max),
            ("resolution", s.resolution),
            ("accuracy", s.accuracy),
        ] {
            assert_sourced(s.id, f, x);
        }
    }
}

#[test]
fn no_source_points_to_ru_zone() {
    for a in actuators() {
        a.sources.iter().for_each(|s| no_ru_domain(s));
        a.numeric_specs()
            .iter()
            .for_each(|(_, s)| no_ru_domain(s.source));
    }
    for d in drivers() {
        d.sources.iter().for_each(|s| no_ru_domain(s));
        d.numeric_specs()
            .iter()
            .for_each(|(_, s)| no_ru_domain(s.source));
    }
    for r in regulators() {
        r.sources.iter().for_each(|s| no_ru_domain(s));
    }
    for s in sensors() {
        s.sources.iter().for_each(|x| no_ru_domain(x));
    }
}

#[test]
fn unpublished_values_stay_none() {
    // Эти числа производители не публикуют: подставлять догадку нельзя.
    let xl = actuator_by_id("dynamixel-xl430-w250").unwrap();
    assert!(xl.rated_torque_nm.value.is_none());
    assert!(xl.resistance_ohm.value.is_none());
    assert!(xl.max_winding_temp_c.value.is_none());
    let ds = actuator_by_id("ds3218").unwrap();
    assert!(ds.rated_current_a.value.is_none());
    assert_eq!(ds.stall_current_a.value, Some(2.2));
    let nema = actuator_by_id("nema17-17hs19-2004s1").unwrap();
    assert_eq!(nema.mass_kg.value, Some(0.390));
    assert!(nema.rated_torque_nm.value.is_none());
    assert!(matches!(
        nema.backdrive,
        Backdrive::Reversible { torque_nm } if torque_nm.value.is_none()
    ));
    let ak70 = actuator_by_id("cubemars-ak70-10").unwrap();
    assert!(ak70.r_th_housing_ambient_k_per_w.value.is_none());
    let amt = sensor_by_id("amt102-v").unwrap();
    assert_eq!(amt.current_a.value, Some(0.006));
    assert!(amt.current_peak_a.value.is_none());
}

#[test]
fn numbers_agree_with_sim_em_presets() {
    // Числа из crates/sim-em/src/motor.rs, сверены чтением.
    let m = actuator_by_id("maxon-ec90-hd-csg25-50").unwrap();
    assert_eq!(m.resistance_ohm.get(), Some(0.844));
    assert_eq!(m.inductance_h.get(), Some(1.07e-3));
    assert_eq!(m.kt_motor_nm_per_a.get(), Some(0.231));
    assert_eq!(m.rotor_inertia_kg_m2.get(), Some(5060e-7));
    assert_eq!(m.gear_ratio.get(), Some(50.0));
    assert_eq!(m.peak_torque_nm.get(), Some(127.0));
    assert_eq!(m.r_th_winding_housing_k_per_w.get(), Some(1.82));
    assert_eq!(m.r_th_housing_ambient_k_per_w.get(), Some(1.78));
    assert_eq!(m.tau_winding_s.get(), Some(54.3));
    assert_eq!(m.tau_housing_s.get(), Some(265.0));
    assert_eq!(m.max_winding_temp_c.get(), Some(125.0));
    // КПД здесь намеренно ниже, чем в sim-em (0.85): нижний край диапазона 70–85 %.
    assert_eq!(m.gear_efficiency.get(), Some(0.70));
    assert_eq!(m.gear_efficiency.provenance, Provenance::Estimate);

    let a = actuator_by_id("cubemars-ak80-64").unwrap();
    assert_eq!(a.resistance_ohm.get(), Some(0.22));
    assert_eq!(a.inductance_h.get(), Some(133.5e-6));
    assert_eq!(a.kt_motor_nm_per_a.get(), Some(0.136));
    assert_eq!(a.rotor_inertia_kg_m2.get(), Some(564.5e-7));
    assert_eq!(a.gear_ratio.get(), Some(64.0));
    assert_eq!(a.rated_torque_nm.get(), Some(48.0));
    assert_eq!(a.peak_torque_nm.get(), Some(120.0));
    assert_eq!(a.rated_current_a.get(), Some(7.0));
    assert_eq!(a.peak_current_a.get(), Some(19.0));
    // В sim-em эти тепловые параметры помечены оценкой, в паспорте их нет.
    assert_eq!(a.r_th_winding_housing_k_per_w.get(), None);
    assert_eq!(a.r_th_housing_ambient_k_per_w.get(), None);
    assert_eq!(a.tau_winding_s.get(), None);
    assert_eq!(a.tau_housing_s.get(), None);
    assert!(matches!(
        a.backdrive,
        Backdrive::Reversible { torque_nm } if torque_nm.get() == Some(4.7)
    ));
}

#[test]
fn cheap_servos_use_lower_torque_and_are_labelled_vendor() {
    let sg = actuator_by_id("sg90").unwrap();
    assert_eq!(sg.peak_torque_nm.provenance, Provenance::Vendor);
    assert!((sg.peak_torque_nm.get().unwrap() - 1.2 * KGF_CM).abs() < 1e-12);
    assert!(sg.caveats.iter().any(|c| c.contains("1.2")));
    for id in ["mg90s", "mg996r", "ds3218"] {
        let a = actuator_by_id(id).unwrap();
        assert_eq!(a.peak_torque_nm.provenance, Provenance::Vendor, "{id}");
        assert!(!a.caveats.is_empty(), "{id}");
    }
}

#[test]
fn safe_side_is_taken_where_sheets_disagree() {
    let tb = driver_by_id("tb6612fng").unwrap();
    assert_eq!(tb.current_continuous_a.get(), Some(1.0));
    let ams = regulator_by_id("ams1117-3.3").unwrap();
    assert_eq!(ams.output_current_a.get(), Some(0.8));
    let bts = driver_by_id("bts7960").unwrap();
    assert_eq!(bts.current_peak_a.get(), Some(33.0));
    let n = actuator_by_id("n20-298-6v-hp").unwrap();
    assert!((n.peak_torque_nm.get().unwrap() - 2.0 * KGF_CM).abs() < 1e-12);
    let nema = actuator_by_id("nema17-17hs19-2004s1").unwrap();
    assert_eq!(nema.resistance_ohm.get(), Some(1.6));
    assert_eq!(
        actuator_by_id("cubemars-ak60-6").unwrap().mass_kg.get(),
        Some(0.368)
    );
}

#[test]
fn builtin_links_resolve() {
    for a in actuators() {
        if let Some(id) = a.builtin_driver {
            let d = driver_by_id(id).unwrap_or_else(|| panic!("{}: нет драйвера {id}", a.id));
            assert_eq!(d.kind, DriverKind::Integrated, "{}", a.id);
            assert!(d.supports.contains(&a.kind.motor_type()), "{}", a.id);
        }
    }
    assert!(
        external_drivers()
            .iter()
            .all(|d| d.kind != DriverKind::Integrated)
    );
    assert!(external_drivers().len() >= 7);
}

#[test]
fn voltage_points_are_ordered_and_consistent() {
    for a in actuators() {
        for w in a.voltage_points.windows(2) {
            assert!(w[0].voltage_v < w[1].voltage_v, "{}", a.id);
            assert!(w[0].stall_torque_nm <= w[1].stall_torque_nm, "{}", a.id);
            assert!(w[0].no_load_rpm <= w[1].no_load_rpm, "{}", a.id);
        }
        if let (Some(lo), Some(first)) = (a.voltage_min_v.get(), a.voltage_points.first()) {
            assert!(first.voltage_v >= lo - 1e-9, "{}", a.id);
        }
        if let (Some(hi), Some(last)) = (a.voltage_max_v.get(), a.voltage_points.last()) {
            assert!(last.voltage_v <= hi + 1e-9, "{}", a.id);
        }
    }
}

#[test]
fn reflected_inertia_where_published() {
    let a = actuator_by_id("cubemars-ak70-10").unwrap();
    let j = a.reflected_inertia_kg_m2().unwrap();
    assert!((j - 753.4788e-7 * 100.0).abs() < 1e-12);
    assert!(
        actuator_by_id("sg90")
            .unwrap()
            .reflected_inertia_kg_m2()
            .is_none()
    );
}

#[test]
fn torque_speed_curve_for_ui_is_monotonic() {
    for id in [
        "maxon-ec90-hd-csg25-50",
        "dynamixel-xm430-w350",
        "n20-100-6v-hp",
        "nema17-17hs19-2004s1",
    ] {
        let a = actuator_by_id(id).unwrap();
        let v = a.voltage_nominal_v.get().unwrap().max(12.0).min(48.0);
        let c = a.torque_speed_curve(v, 40.0, 50);
        assert_eq!(c.len(), 50, "{id}");
        for w in c.windows(2) {
            assert!(w[1].0 >= w[0].0 && w[1].1 <= w[0].1 + 1e-9, "{id}: {w:?}");
            assert!(w[0].1.is_finite() && w[0].0.is_finite());
        }
    }
}
