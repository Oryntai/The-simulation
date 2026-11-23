mod common;

use std::path::Path;

use sim_actuators::{Provenance, SensorKind as OldKind, Spec};
use sim_runtime::{ObservationOut, Runtime, Sensor, Truth};
use sim_sense::{
    Parameter, PassportKind, SensorCatalog, SensorKind, SensorSpec, SourceKind, install,
};
use sim_signal::{Rate, Rng};

use common::{CatalogDir, run, sensor, world};

fn compare(new: &Parameter, old: Spec) {
    assert_eq!(new.value().map(f64::to_bits), old.value.map(f64::to_bits));
    match new {
        Parameter::Unknown(_) => assert_eq!(old.provenance, Provenance::Missing),
        Parameter::Known(n) => {
            let provenance = match old.provenance {
                Provenance::Datasheet => SourceKind::Datasheet,
                Provenance::Vendor => SourceKind::Vendor,
                Provenance::Measured => SourceKind::Measured,
                Provenance::Derived => SourceKind::Derived,
                Provenance::Estimate => SourceKind::Estimate,
                Provenance::Missing => panic!("числа в исходном паспорте нет"),
            };
            assert_eq!(n.provenance, provenance);
            assert_eq!(n.source, old.source);
        }
    }
}

#[test]
fn migrated_passports_match_source_bit_for_bit() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../catalog/sensors");
    let catalog = SensorCatalog::load(path).unwrap();
    let old = sim_actuators::sensors();
    assert!(!old.is_empty());
    for s in old {
        let entry = catalog.get(s.id).unwrap();
        let p = entry.passport.as_ref().unwrap();
        let (passport, model) = match s.kind {
            OldKind::Angle => (PassportKind::Angle, SensorKind::Encoder),
            OldKind::Encoder => (PassportKind::Encoder, SensorKind::Encoder),
            OldKind::Imu => (PassportKind::Imu, SensorKind::Imu),
            OldKind::Distance => (PassportKind::Distance, SensorKind::Rangefinder),
            OldKind::ForceAdc => (PassportKind::ForceAdc, SensorKind::BridgeAdc),
            OldKind::LoadCell => (PassportKind::LoadCell, SensorKind::Force),
            OldKind::ForceResistor => (PassportKind::ForceResistor, SensorKind::Touch),
            OldKind::PowerMonitor => (PassportKind::PowerMonitor, SensorKind::Current),
        };
        assert_eq!((p.kind, entry.kind), (passport, model));
        assert_eq!(entry.name, s.name);
        assert_eq!(p.what_is, s.what_is);
        assert_eq!(p.interface, s.interface);
        assert_eq!(p.sources, s.sources);
        assert_eq!(p.caveats, s.caveats);
        assert_eq!(
            [p.range_unit.as_str(), &p.resolution_unit, &p.accuracy_unit],
            [s.range_unit, s.resolution_unit, s.accuracy_unit]
        );
        assert_eq!(p.supply_ranges.len(), s.supply_ranges.len());
        for (range, &(a, b)) in p.supply_ranges.iter().zip(s.supply_ranges) {
            assert_eq!(range.min.value.to_bits(), a.to_bits());
            assert_eq!(range.max.value.to_bits(), b.to_bits());
            assert!(!range.min.source.is_empty() && !range.max.source.is_empty());
        }
        for (new, old) in [
            (&p.current_a, s.current_a),
            (&p.current_peak_a, s.current_peak_a),
            (&p.range_min, s.range_min),
            (&p.range_max, s.range_max),
            (&p.resolution, s.resolution),
            (&p.accuracy, s.accuracy),
        ] {
            compare(new, old);
        }
        assert_eq!(entry.model["rate_hz"].value(), None);
    }
    assert_eq!(
        catalog.iter().filter(|p| p.passport.is_some()).count(),
        old.len()
    );
}

#[test]
fn installing_an_added_toml_requires_no_rust_registration() {
    let dir = CatalogDir::new("позиция");
    let mut reading = sensor("контроль", "position-file-only");
    reading.joint = Some("ось".into());
    let scene = world(
        r#"<body name="каретка"><joint name="ось" type="slide" ref="0.35"/>
                      <geom size="0.1" mass="1"/></body>"#,
        "0 0 0",
        vec![reading],
    );
    let mut runtime = Runtime::load(scene.clone()).unwrap();
    let error = install(&mut runtime, &scene, &dir.catalog()).err().unwrap();
    assert!(error.contains("неизвестная позиция"));
    assert!(runtime.observation_id("контроль/value").is_none());

    // Тот же тип и та же сцена. Теперь в каталоге появился один файл.
    dir.add("position-file-only", SensorKind::Encoder, &[]);
    let handles = install(&mut runtime, &scene, &dir.catalog()).unwrap();
    let id = handles[0].signal("value").unwrap();
    run(&mut runtime, 0.0);
    assert_eq!(runtime.observe().latest(id), Some(0.35));
}

#[test]
fn invalid_catalog_data_are_refused() {
    let base = "id = 'позиция'\nname = 'Датчик'\nkind = 'depth_camera'\n[model]\n";
    let base = base.replace("позиция", "position");
    for (field, message) in [
        ("typo = 'unknown'", "неизвестный параметр"),
        (
            "fx = {value=nan, provenance='measured', source='стенд'}",
            "конечным",
        ),
        (
            "fy = {value=inf, provenance='measured', source='стенд'}",
            "конечным",
        ),
        (
            "fx = {value=1.0, provenance='measured', source=''}",
            "источник",
        ),
        (
            "fx = {value=0.0, provenance='measured', source='стенд'}",
            "недопустимое значение",
        ),
        (
            "width = {value=0.5, provenance='measured', source='стенд'}",
            "недопустимое значение",
        ),
        (
            "fx = {value=1.0, provenance='measured', source='стенд', typo=true}",
            "разбор",
        ),
        ("fx = 'undocumented'", "разбор"),
    ] {
        let err = SensorSpec::parse(&format!("{base}{field}")).unwrap_err();
        assert!(err.contains(message), "{field}: {err}");
    }
    let err = SensorSpec::parse(&base.replace("[model]", "typo = true\n[model]")).unwrap_err();
    assert!(err.contains("unknown field"));
}

#[test]
fn missing_invalid_and_unproven_instance_parameters() {
    let dir = CatalogDir::new("проверка");
    dir.write(
        "unspecified",
        "id='unspecified'\nname='Датчик'\nkind='imu'\n[model]\nrate_hz='unknown'\n",
    );
    dir.add("camera", SensorKind::ColorCamera, &[]);
    dir.add("imu", SensorKind::Imu, &[]);

    let mut scene = world("", "0 0 0", vec![sensor("контроль", "unspecified")]);
    let mut runtime = Runtime::load(scene.clone()).unwrap();
    let catalog = dir.catalog();
    assert!(
        install(&mut runtime, &scene, &catalog)
            .err()
            .unwrap()
            .contains("обязателен")
    );
    for (key, value, message) in [
        ("bias", f64::NAN, "недопустимое"),
        ("latency_s", f64::INFINITY, "недопустимое"),
        ("typo", 0.0, "неизвестный параметр"),
        ("far_m", 0.001, "near_m"),
        ("k1", -100.0, "необратима"),
        ("width", f64::from(u32::MAX), "бюджет"),
    ] {
        scene.sensors[0] = sensor("контроль", "camera");
        scene.sensors[0].params.insert(key.into(), value);
        let err = install(&mut runtime, &scene, &catalog).err().unwrap();
        assert!(err.contains(message), "{key}: {err}");
        assert!(runtime.observation_id("контроль/image").is_none());
        assert!(runtime.simplifications().is_empty());
    }
    scene.sensors[0] = sensor("контроль", "imu");
    scene.sensors[0].params.insert("bias".into(), 0.1);
    assert!(install(&mut runtime, &scene, &catalog).is_ok());
    assert_eq!(runtime.simplifications().len(), 1);
    assert!(runtime.simplifications()[0].blocks_proof);
}

#[derive(Clone)]
struct Silent;

impl Sensor for Silent {
    fn sample(&mut self, _: &Truth<'_>, _: &mut Rng, _: &mut ObservationOut) -> Result<(), String> {
        Ok(())
    }
}

#[test]
fn installation_rolls_back_registrations_on_conflict_and_checks_world_identity() {
    let dir = CatalogDir::new("откат");
    dir.add("camera", SensorKind::ColorCamera, &[]);
    let scene = world(
        "",
        "0 0 0",
        vec![sensor("кадр", "camera"), sensor("повтор", "camera")],
    );
    let mut runtime = Runtime::load(scene.clone()).unwrap();
    runtime
        .register_sensor("датчик/повтор", Rate::hz(100).unwrap(), Silent)
        .unwrap();
    let fp = runtime.log().fingerprint();
    let error = install(&mut runtime, &scene, &dir.catalog()).err().unwrap();
    assert!(error.contains("повторное имя"));
    assert_eq!(runtime.log().fingerprint(), fp);
    assert!(runtime.simplifications().is_empty());
    assert!(runtime.observation_id("кадр/image").is_none());
    assert!(runtime.observation_id("повтор/image").is_none());

    let other = world(
        r#"<geom type="sphere" size="0.1" pos="0 0 2"/>"#,
        "0 0 0",
        scene.sensors.clone(),
    );
    let err = install(&mut runtime, &other, &dir.catalog()).err().unwrap();
    assert!(err.contains("другого мира"));
}
