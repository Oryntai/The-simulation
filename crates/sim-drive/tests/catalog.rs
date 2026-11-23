use std::path::Path;

use sim_actuators::{ActuatorKind, Backdrive, Brake, Provenance, Spec, actuators};
use sim_drive::{
    BackdriveMode, BrakeMode, DriveCatalog, DriveKind, LegacyClass, Magnet, Parameter, Quantity,
    Source,
};

fn kinds(kind: ActuatorKind) -> (DriveKind, LegacyClass, &'static str, &'static str) {
    use ActuatorKind::*;
    match kind {
        GearedBldc => (
            DriveKind::Bldc,
            LegacyClass::GearedBldc,
            "bldc",
            "geared_bldc",
        ),
        Qdd => (DriveKind::Bldc, LegacyClass::Qdd, "bldc", "qdd"),
        SmartServo => (
            DriveKind::BusServo,
            LegacyClass::SmartServo,
            "bus_servo",
            "smart_servo",
        ),
        HobbyServo => (
            DriveKind::HobbyServo,
            LegacyClass::HobbyServo,
            "hobby_servo",
            "hobby_servo",
        ),
        BrushedGearmotor => (
            DriveKind::Dc,
            LegacyClass::BrushedGearmotor,
            "dc",
            "brushed_gearmotor",
        ),
        Stepper => (
            DriveKind::Stepper,
            LegacyClass::Stepper,
            "stepper",
            "stepper",
        ),
    }
}

fn mapped(old: Provenance) -> Option<(Source, &'static str)> {
    Some(match old {
        Provenance::Datasheet => (Source::Datasheet, "datasheet"),
        Provenance::Derived => (Source::Derived, "derived"),
        Provenance::Measured => (Source::Measured, "measured"),
        Provenance::Vendor | Provenance::Estimate => (Source::Estimate, "estimate"),
        Provenance::Missing => return None,
    })
}

#[test]
fn migrated_catalog_preserves_every_passport_field() {
    let catalog =
        DriveCatalog::load(Path::new(env!("CARGO_MANIFEST_DIR")).join("../../catalog/drives"))
            .unwrap();
    assert_eq!(
        catalog.iter().filter(|p| p.legacy_class.is_some()).count(),
        actuators().len()
    );
    for act in actuators() {
        let entry = catalog.get(act.id).unwrap();
        let (kind, class, _, _) = kinds(act.kind);
        assert_eq!(
            (
                entry.id.as_str(),
                entry.name.as_str(),
                entry.what_is.as_str()
            ),
            (act.id, act.name, act.what_is)
        );
        assert_eq!((entry.kind, entry.legacy_class), (kind, Some(class)));
        assert_eq!(entry.builtin_driver.as_deref(), act.builtin_driver);
        assert_eq!(entry.sources, act.sources);
        assert_eq!(entry.caveats, act.caveats);
        assert_eq!(
            entry.magnet,
            Some(match act.magnet {
                sim_actuators::Magnet::NdFeB => Magnet::NdFeB,
                sim_actuators::Magnet::Ferrite => Magnet::Ferrite,
                sim_actuators::Magnet::Unknown => Magnet::Unknown,
            })
        );
        let field = |key, reference, parameter| {
            compare(act.id, key, &entry.parameters[&parameter], reference)
        };
        field(
            "voltage_nominal_v",
            act.voltage_nominal_v,
            Parameter::VoltageNominal,
        );
        field("voltage_min_v", act.voltage_min_v, Parameter::VoltageMin);
        field("voltage_max_v", act.voltage_max_v, Parameter::VoltageMax);
        field(
            "rated_torque_nm",
            act.rated_torque_nm,
            Parameter::RatedTorque,
        );
        field("peak_torque_nm", act.peak_torque_nm, Parameter::PeakTorque);
        field(
            "no_load_speed_rpm",
            act.no_load_speed_rpm,
            Parameter::NoLoadSpeed,
        );
        field(
            "rated_speed_rpm",
            act.rated_speed_rpm,
            Parameter::RatedSpeed,
        );
        field(
            "no_load_current_a",
            act.no_load_current_a,
            Parameter::NoLoadCurrent,
        );
        field(
            "stall_current_a",
            act.stall_current_a,
            Parameter::StallCurrent,
        );
        field(
            "rated_current_a",
            act.rated_current_a,
            Parameter::RatedCurrent,
        );
        field("peak_current_a", act.peak_current_a, Parameter::PeakCurrent);
        field(
            "standby_current_a",
            act.standby_current_a,
            Parameter::StandbyCurrent,
        );
        field("resistance_ohm", act.resistance_ohm, Parameter::Resistance);
        field("inductance_h", act.inductance_h, Parameter::Inductance);
        field("kt_motor_nm_per_a", act.kt_motor_nm_per_a, Parameter::Kt);
        field("ke_motor_v_s", act.ke_motor_v_s, Parameter::Ke);
        field("kv_rpm_per_v", act.kv_rpm_per_v, Parameter::Kv);
        field("gear_ratio", act.gear_ratio, Parameter::Ratio);
        field(
            "gear_efficiency",
            act.gear_efficiency,
            Parameter::ForwardEfficiency,
        );
        field(
            "rotor_inertia_kg_m2",
            act.rotor_inertia_kg_m2,
            Parameter::RotorInertia,
        );
        field(
            "gear_input_inertia_kg_m2",
            act.gear_input_inertia_kg_m2,
            Parameter::GearInertia,
        );
        field("mass_kg", act.mass_kg, Parameter::Mass);
        field(
            "r_th_winding_housing_k_per_w",
            act.r_th_winding_housing_k_per_w,
            Parameter::ThermalWh,
        );
        field(
            "r_th_housing_ambient_k_per_w",
            act.r_th_housing_ambient_k_per_w,
            Parameter::ThermalHa,
        );
        field("tau_winding_s", act.tau_winding_s, Parameter::TauWinding);
        field("tau_housing_s", act.tau_housing_s, Parameter::TauHousing);
        field(
            "r_th_total_k_per_w",
            act.r_th_total_k_per_w,
            Parameter::ThermalTotal,
        );
        field(
            "max_winding_temp_c",
            act.max_winding_temp_c,
            Parameter::MaxWinding,
        );
        field("min_ambient_c", act.min_ambient_c, Parameter::MinAmbient);
        field("max_ambient_c", act.max_ambient_c, Parameter::MaxAmbient);
        assert_eq!(
            entry.parameters[&Parameter::HeatedPhases].value(),
            Some(act.heated_phases)
        );
        assert_eq!(
            entry.parameters[&Parameter::PolePairs].value(),
            act.stepper_pole_pairs
        );
        match act.backdrive {
            Backdrive::Unknown => assert_eq!(entry.backdrive, BackdriveMode::Unknown),
            Backdrive::SelfLocking { source } => {
                assert_eq!(entry.backdrive, BackdriveMode::SelfLocking);
                assert_eq!(entry.backdrive_source.as_deref(), Some(source));
            }
            Backdrive::Reversible { torque_nm } => {
                assert_eq!(entry.backdrive, BackdriveMode::Reversible);
                compare(
                    act.id,
                    "backdrive_torque_nm",
                    entry.backdrive_torque_nm.as_ref().unwrap(),
                    torque_nm,
                );
            }
        }
        match act.brake {
            // В прежней схеме None означал в том числе отсутствие сведений.
            Brake::None => {
                assert_eq!(entry.brake, BrakeMode::Unknown);
                assert!(entry.brake_torque_nm.is_none());
            }
            Brake::Holding { torque_nm } => {
                assert_eq!(entry.brake, BrakeMode::Holding);
                compare(
                    act.id,
                    "brake_torque_nm",
                    entry.brake_torque_nm.as_ref().unwrap(),
                    torque_nm,
                );
            }
        }
        assert_eq!(entry.voltage_points.len(), act.voltage_points.len());
        for (actual, old) in entry.voltage_points.iter().zip(act.voltage_points) {
            assert_eq!(
                actual.voltage_v.value().map(f64::to_bits),
                Some(old.voltage_v.to_bits()),
                "{}",
                act.id
            );
            assert_eq!(
                actual.no_load_rpm.value().map(f64::to_bits),
                Some(old.no_load_rpm.to_bits()),
                "{}",
                act.id
            );
            assert_eq!(
                actual.stall_torque_nm.value().map(f64::to_bits),
                Some(old.stall_torque_nm.to_bits()),
                "{}",
                act.id
            );
            assert_eq!(
                actual.stall_current_a.value().map(f64::to_bits),
                old.stall_current_a.map(f64::to_bits),
                "{}",
                act.id
            );
        }
    }
}

fn compare(id: &str, key: &str, actual: &Quantity, old: Spec) {
    assert_eq!(
        actual.value().map(f64::to_bits),
        old.value.map(f64::to_bits),
        "{id}/{key}"
    );
    match actual {
        Quantity::Known(s) => {
            assert_eq!(
                (s.source, s.reference.as_str()),
                (mapped(old.provenance).unwrap().0, old.source),
                "{id}/{key}"
            );
        }
        Quantity::Unknown(u) => assert!(!u.bench.is_empty(), "{id}/{key}"),
    }
}
