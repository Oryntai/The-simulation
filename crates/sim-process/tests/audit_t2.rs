use sim_circuit::battery::{CellParams, Table};
use sim_process::schematic::mcu_energy::assess_mcu_energy;
use sim_process::schematic::sleep_drain::{
    SleepDrainBattery, SleepDrainInput, SleepDrainProfile, SleepDrainRequirement, SleepDrainState,
    assess_sleep_drain,
};
use sim_process::schematic::{Mode, Scope, Section, Status};
use sim_process::{Analysis, SourcedRange};
use sim_signal::Time;
use sim_world::Provenance;

#[test]
fn автономность_сборки_требует_замеренного_профиля_и_применимой_батареи() {
    let measured = Provenance::Measured("Синтетическая запись стенда блока".into());
    let paper = Provenance::Paper("Синтетическое описание расчётного режима".into());
    let estimate = Provenance::Estimate("Заявленный номинал из ТЗ: 1000 мА·ч".into());
    let range = |value: f64, source: &Provenance| SourcedRange {
        min: value,
        nominal: value,
        max: value,
        provenance: source.clone(),
    };
    // Стенд: за 1 ч при 100 мА расходуется 0,1 А·ч. При 1 А·ч, SOC=1,
    // OCV=3+SOC, R0=1 Ом и без RC: U в конце 3,8 В, запас до 3 В равен 0,8 В.
    let mut params = CellParams::lg_m50();
    params.capacity_ah = 1.0;
    params.ocv = Table(vec![(0.0, 3.0), (1.0, 4.0)]);
    params.entropic = Table(vec![(0.0, 0.0), (1.0, 0.0)]);
    params.r0 = Table(vec![(0.0, 1.0), (1.0, 1.0)]);
    params.rc.clear();
    // Опорная температура модели батареи 25 °C.
    params.capacity_factor = Table(vec![(25.0, 1.0)]);
    let capacity = range(params.capacity_ah, &measured);
    let nominal = range(params.capacity_ah, &estimate);
    let cutoff = range(3.0, &measured);
    let unsafe_cutoff = range(3.85, &measured); // Между 3,9 В на пуске и 3,8 В в конце.
    let full = range(1.0, &measured);
    let zero = range(0.0, &measured);
    let temperature = range(25.0, &measured);
    let current = range(0.1, &measured);
    let duration = Time::from_secs(3600.0).unwrap(); // Секунд в часе.
    let branch = Section::Connection("питание блока".into());
    let states = [SleepDrainState {
        id: "работа",
        duration,
        controller_a: Some(&current),
        loads: Some(&[]),
    }];
    let profile = SleepDrainProfile {
        states: &states,
        repeat: true,
        source: &measured,
    };
    let battery = SleepDrainBattery {
        params: &params,
        applicability: &measured,
        capacity_ah: Some(&capacity),
        initial_soc: Some(&full),
        initial_polarization_v: Some(&[]),
        temperature_c: Some(&temperature),
        self_discharge_a: Some(&zero),
        cutoff_v: Some(&cutoff),
        voltage_error_v: Some(&zero),
    };
    let input = SleepDrainInput {
        scope: Scope::Assembly,
        mode: &Mode::Normal,
        branch: &branch,
        required: Some(SleepDrainRequirement {
            duration,
            source: &measured,
        }),
        profile: Some(profile),
        battery: Some(battery),
    };
    let calculate = |input| {
        let direct = assess_sleep_drain(&input).unwrap();
        let via_mcu = assess_mcu_energy(None, &input).unwrap();
        assert_eq!(via_mcu.sleep_drain, direct);
        assert!(matches!(via_mcu.chip_charge, Analysis::NotAnalyzed { .. }));
        direct
    };
    let complete = calculate(input);
    assert_eq!(complete.status, Status::Pass);
    for (value, analytical) in [
        (complete.evidence.charge_margin_ah, 0.9),
        (complete.evidence.cutoff_margin_v, 0.8),
    ] {
        let Analysis::Analyzed { value, .. } = value else {
            panic!("нет запаса")
        };
        // Допуск округления для десятичных данных стенда.
        assert!((value.min - analytical).abs() < 1e-12);
        assert!((value.max - analytical).abs() < 1e-12);
    }

    let analytic_profile = SleepDrainProfile {
        source: &paper,
        ..profile
    };
    let analytic_input = SleepDrainInput {
        profile: Some(analytic_profile),
        ..input
    };
    for incomplete in [
        SleepDrainInput {
            profile: None,
            ..input
        },
        SleepDrainInput {
            required: None,
            ..input
        },
        SleepDrainInput {
            battery: Some(SleepDrainBattery {
                cutoff_v: None,
                ..battery
            }),
            ..input
        },
        SleepDrainInput {
            battery: Some(SleepDrainBattery {
                capacity_ah: None,
                ..battery
            }),
            ..input
        },
        SleepDrainInput {
            battery: Some(SleepDrainBattery {
                capacity_ah: Some(&nominal),
                ..battery
            }),
            ..input
        },
        SleepDrainInput {
            battery: Some(SleepDrainBattery {
                temperature_c: None,
                ..battery
            }),
            ..input
        },
        analytic_input,
    ] {
        assert_eq!(calculate(incomplete).status, Status::NotAnalyzed);
    }
    let fails = calculate(SleepDrainInput {
        battery: Some(SleepDrainBattery {
            cutoff_v: Some(&unsafe_cutoff),
            ..battery
        }),
        ..analytic_input
    });
    assert_eq!(fails.status, Status::Fail);
    assert_eq!(fails.evidence.cutoff_violation.unwrap().time, duration);

    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let bad_capacity = range(invalid, &measured);
        let invalid_input = SleepDrainInput {
            profile: None,
            battery: Some(SleepDrainBattery {
                capacity_ah: Some(&bad_capacity),
                ..battery
            }),
            ..input
        };
        assert!(assess_sleep_drain(&invalid_input).is_err());
        assert!(assess_mcu_energy(None, &invalid_input).is_err());
    }
}
