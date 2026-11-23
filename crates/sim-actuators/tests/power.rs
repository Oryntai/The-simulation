//! Сводка питания: батарея, стабилизаторы, потребители.

use sim_actuators::{
    ActuatorLoad, Battery, BoardLoad, DutyCycle, Outcome, PowerBudget, PowerCheck as P, PowerInput,
    RailInput, SensorLoad, Spec, actuator_by_id, power_budget, regulator_by_id, sensor_by_id,
};

fn battery_2s() -> Battery {
    Battery {
        nominal_v: 7.4,
        full_v: 8.4,
        empty_v: 6.4,
        capacity_ah: 2.0,
        max_continuous_a: Some(20.0),
        max_peak_a: Some(40.0),
        internal_resistance_ohm: Some(0.03),
    }
}

fn battery_3s() -> Battery {
    Battery {
        nominal_v: 11.1,
        full_v: 12.6,
        empty_v: 9.6,
        capacity_ah: 2.2,
        max_continuous_a: Some(20.0),
        max_peak_a: Some(40.0),
        internal_resistance_ohm: Some(0.03),
    }
}

fn empty_input(battery: Battery) -> PowerInput<'static> {
    PowerInput {
        battery,
        rails: Vec::new(),
        actuators: Vec::new(),
        sensors: Vec::new(),
        boards: Vec::new(),
        ambient_c: 25.0,
        required_runtime_s: None,
    }
}

fn rail(name: &str, id: &str, v: f64) -> RailInput<'static> {
    RailInput {
        name: name.into(),
        regulator: regulator_by_id(id).unwrap(),
        output_v: v,
    }
}

fn board(name: &str, v: f64, avg: f64, peak: f64, rail: Option<usize>) -> BoardLoad {
    BoardLoad {
        name: name.into(),
        voltage_v: v,
        current_avg_a: avg,
        current_peak_a: peak,
        rail,
    }
}

fn outcomes(b: &PowerBudget, kind: P) -> Vec<Outcome> {
    b.findings
        .iter()
        .filter(|f| f.kind == kind)
        .map(|f| f.outcome)
        .collect()
}

fn assert_has_fail(b: &PowerBudget, kind: P) {
    assert!(
        outcomes(b, kind).contains(&Outcome::Fail),
        "{kind:?}: {:#?}",
        b.findings
    );
    assert_eq!(b.outcome, Outcome::Fail);
}

#[test]
fn healthy_device_passes() {
    let mut p = empty_input(battery_2s());
    p.rails.push(rail("3.3 В", "ams1117-3.3", 3.3));
    p.boards.push(board("плата", 3.3, 0.05, 0.05, Some(0)));
    p.required_runtime_s = Some(3600.0);
    let b = power_budget(&p);
    for f in &b.findings {
        assert_eq!(f.outcome, Outcome::Pass, "{f:?}");
    }
    assert_eq!(b.outcome, Outcome::Pass);
    assert!(b.runtime_s.unwrap() > 3600.0);
}

#[test]
fn full_battery_above_servo_limit_fails() {
    // XL430 рассчитан до 12 В, заряженная 3S даёт 12.6 В.
    let a = actuator_by_id("dynamixel-xl430-w250").unwrap();
    let duty = DutyCycle::new(11.1, 25.0)
        .segment(0.3, 0.5, 1.0)
        .hold_unpowered(0.0);
    let mut p = empty_input(battery_3s());
    p.actuators.push(ActuatorLoad {
        actuator: a,
        driver: None,
        duty: &duty,
        count: 2,
        rail: None,
    });
    assert_has_fail(&power_budget(&p), P::ConsumerVoltage);
}

#[test]
fn duty_voltage_must_match_its_rail() {
    let a = actuator_by_id("mg996r").unwrap();
    let duty = DutyCycle::new(6.0, 25.0)
        .segment(0.2, 0.5, 1.0)
        .hold_unpowered(0.0);
    let mut p = empty_input(battery_2s());
    p.rails.push(rail("5 В", "lm2596-adj", 5.0));
    p.actuators.push(ActuatorLoad {
        actuator: a,
        driver: None,
        duty: &duty,
        count: 1,
        rail: Some(0),
    });
    assert_has_fail(&power_budget(&p), P::ConsumerVoltage);
}

#[test]
fn servo_stall_overloads_small_regulator() {
    let a = actuator_by_id("mg996r").unwrap();
    // Шесть серв при заметной нагрузке на одном модуле на 3 А.
    let duty = DutyCycle::new(5.0, 25.0)
        .segment(0.5, 0.0, 1.0)
        .hold_unpowered(0.0);
    let mut p = empty_input(battery_2s());
    p.rails.push(rail("5 В", "lm2596-adj", 5.0));
    p.actuators.push(ActuatorLoad {
        actuator: a,
        driver: None,
        duty: &duty,
        count: 6,
        rail: Some(0),
    });
    let b = power_budget(&p);
    assert_has_fail(&b, P::RailCurrent);
    assert!(b.rails[0].load_peak_a > 3.0);
}

#[test]
fn linear_regulator_from_high_voltage_overheats() {
    let mut p = empty_input(Battery {
        nominal_v: 11.1,
        full_v: 11.1,
        empty_v: 11.1,
        capacity_ah: 2.0,
        max_continuous_a: Some(10.0),
        max_peak_a: Some(20.0),
        internal_resistance_ohm: Some(0.03),
    });
    p.rails.push(rail("3.3 В", "ams1117-3.3", 3.3));
    p.boards.push(board("плата", 3.3, 0.5, 0.5, Some(0)));
    let b = power_budget(&p);
    assert_has_fail(&b, P::RailTemperature);
    assert!(b.rails[0].loss_avg_w > 3.5);
}

#[test]
fn ams1117_input_above_rating_fails() {
    let mut p = empty_input(battery_3s());
    p.rails.push(rail("3.3 В", "ams1117-3.3", 3.3));
    p.boards.push(board("плата", 3.3, 0.02, 0.02, Some(0)));
    assert_has_fail(&power_budget(&p), P::RailInputVoltage);
}

#[test]
fn regulator_dropout_fails_on_low_battery() {
    // 1S: 3.0–4.2 В, для 3.3 В нужно не меньше 4.6 В.
    let mut p = empty_input(Battery {
        nominal_v: 3.7,
        full_v: 4.2,
        empty_v: 3.0,
        capacity_ah: 1.0,
        max_continuous_a: Some(5.0),
        max_peak_a: Some(10.0),
        internal_resistance_ohm: Some(0.05),
    });
    p.rails.push(rail("3.3 В", "ams1117-3.3", 3.3));
    p.boards.push(board("плата", 3.3, 0.05, 0.05, Some(0)));
    assert_has_fail(&power_budget(&p), P::RailInputVoltage);
}

#[test]
fn ldo_below_minimum_load_fails() {
    let mut p = empty_input(battery_2s());
    p.rails.push(rail("3.3 В", "ams1117-3.3", 3.3));
    p.sensors.push(SensorLoad {
        sensor: sensor_by_id("bmi270").unwrap(),
        count: 1,
        rail: Some(0),
        current_override_a: Some(0.001), // Верхняя граница тока в тестовой схеме.
    });
    assert_has_fail(&power_budget(&p), P::RailMinLoad);
}

#[test]
fn sensor_on_wrong_voltage_fails() {
    // Голый MPU-6050 на 5 В: предел чипа 3.46 В.
    let mut p = empty_input(battery_2s());
    p.rails.push(rail("5 В", "lm2596-adj", 5.0));
    p.sensors.push(SensorLoad {
        sensor: sensor_by_id("mpu6050").unwrap(),
        count: 1,
        rail: Some(0),
        current_override_a: None,
    });
    assert_has_fail(&power_budget(&p), P::ConsumerVoltage);
}

#[test]
fn as5600_accepts_both_supply_ranges_but_not_between() {
    for (v, ok) in [(3.3, true), (5.0, true), (4.0, false)] {
        assert_eq!(sensor_by_id("as5600").unwrap().supply_ok(v), ok, "{v} В");
    }
    assert!(!sensor_by_id("as5600").unwrap().supply_ok(f64::NAN));
}

#[test]
fn unknown_current_is_not_a_pass() {
    let mut p = empty_input(battery_3s());
    p.rails.push(rail("5 В", "ams1117-5.0", 5.0));
    // Источник с границами, входящими в рабочий диапазон AMS1117.
    p.battery.full_v = 12.0;
    p.boards.push(board("плата", 5.0, 0.02, 0.02, Some(0)));
    p.sensors.push(SensorLoad {
        sensor: sensor_by_id("amt102-v").unwrap(),
        count: 1,
        rail: Some(0),
        current_override_a: None,
    });
    let b = power_budget(&p);
    assert!(outcomes(&b, P::DataCompleteness).contains(&Outcome::NotAnalyzed));
    assert_eq!(b.outcome, Outcome::NotAnalyzed);
    assert!(
        b.lines
            .iter()
            .any(|l| l.name.contains("AMT102") && l.avg_a == 0.006)
    );

    // С заданным вручную током данные полны.
    p.sensors[0].current_override_a = Some(0.02);
    let b = power_budget(&p);
    assert!(outcomes(&b, P::DataCompleteness).is_empty());
    assert_eq!(b.outcome, Outcome::Pass, "{:#?}", b.findings);
}

#[test]
fn actuator_without_bus_current_bound_is_not_a_pass() {
    let a = actuator_by_id("ds3218").unwrap();
    let duty = DutyCycle::new(6.0, 25.0)
        .segment(0.5, 0.5, 1.0)
        .hold_unpowered(0.0);
    let mut p = empty_input(battery_3s());
    p.rails.push(rail("6 В", "lm2596-adj", 6.0));
    p.actuators.push(ActuatorLoad {
        actuator: a,
        driver: None,
        duty: &duty,
        count: 1,
        rail: Some(0),
    });
    let b = power_budget(&p);
    assert_eq!(b.outcome, Outcome::NotAnalyzed, "{:#?}", b.findings);
    assert!(outcomes(&b, P::DataCompleteness).contains(&Outcome::NotAnalyzed));
}

#[test]
fn estimated_efficiency_cannot_prove_pass() {
    let mut p = empty_input(battery_2s());
    p.rails.push(rail("5 В", "mp1584", 5.0));
    p.boards.push(board("плата", 5.0, 0.2, 0.4, Some(0)));
    let b = power_budget(&p);
    assert!(outcomes(&b, P::RailTemperature).contains(&Outcome::NotAnalyzed));
    assert_ne!(b.outcome, Outcome::Pass);
}

#[test]
fn battery_current_limits_fail() {
    let mut p = empty_input(Battery {
        max_continuous_a: Some(1.0),
        max_peak_a: Some(2.0),
        ..battery_2s()
    });
    p.boards.push(board("нагрузка", 7.4, 1.5, 3.0, None));
    let b = power_budget(&p);
    assert_has_fail(&b, P::BatteryContinuousCurrent);
    assert_has_fail(&b, P::BatteryPeakCurrent);
}

#[test]
fn battery_sag_fails_with_high_resistance() {
    let mut p = empty_input(Battery {
        internal_resistance_ohm: Some(0.5),
        ..battery_2s()
    });
    p.boards.push(board("нагрузка", 7.4, 1.0, 5.0, None));
    assert_has_fail(&power_budget(&p), P::BatterySag);
}

#[test]
fn missing_battery_limits_are_not_analyzed() {
    let mut p = empty_input(Battery {
        max_continuous_a: None,
        max_peak_a: None,
        internal_resistance_ohm: None,
        ..battery_2s()
    });
    p.boards.push(board("нагрузка", 7.4, 0.5, 1.0, None));
    let b = power_budget(&p);
    for k in [
        P::BatteryContinuousCurrent,
        P::BatteryPeakCurrent,
        P::BatterySag,
    ] {
        assert_eq!(outcomes(&b, k), vec![Outcome::NotAnalyzed], "{k:?}");
    }
    assert_eq!(b.outcome, Outcome::NotAnalyzed);
}

#[test]
fn runtime_requirement_fails_when_energy_is_short() {
    let mut p = empty_input(battery_2s());
    p.boards.push(board("нагрузка", 7.4, 2.0, 2.0, None));
    p.required_runtime_s = Some(4.0 * 3600.0);
    let b = power_budget(&p);
    assert_has_fail(&b, P::Runtime);
    let power = 2.0 * 7.4;
    let expected = 2.0 * 3600.0 * loaded_voltage(&p.battery, power) / power;
    assert!((b.runtime_s.unwrap() - expected).abs() < 1e-6);
}

#[test]
fn nan_and_infinity_are_fail() {
    for bad in [f64::NAN, f64::INFINITY] {
        let inputs: Vec<PowerInput> = vec![
            empty_input(Battery {
                nominal_v: bad,
                ..battery_2s()
            }),
            empty_input(Battery {
                full_v: bad,
                ..battery_2s()
            }),
            empty_input(Battery {
                empty_v: bad,
                ..battery_2s()
            }),
            empty_input(Battery {
                capacity_ah: bad,
                ..battery_2s()
            }),
            empty_input(Battery {
                max_continuous_a: Some(bad),
                ..battery_2s()
            }),
            empty_input(Battery {
                max_peak_a: Some(bad),
                ..battery_2s()
            }),
            empty_input(Battery {
                internal_resistance_ohm: Some(bad),
                ..battery_2s()
            }),
            {
                let mut p = empty_input(battery_2s());
                p.ambient_c = bad;
                p
            },
            {
                let mut p = empty_input(battery_2s());
                p.required_runtime_s = Some(bad);
                p
            },
            {
                let mut p = empty_input(battery_2s());
                p.boards.push(board("плата", 7.4, bad, 1.0, None));
                p
            },
            {
                let mut p = empty_input(battery_2s());
                p.boards.push(board("плата", bad, 0.1, 1.0, None));
                p
            },
            {
                let mut p = empty_input(battery_2s());
                p.rails.push(rail("шина", "lm2596-adj", bad));
                p
            },
            {
                let mut p = empty_input(battery_2s());
                p.sensors.push(SensorLoad {
                    sensor: sensor_by_id("fsr402").unwrap(),
                    count: 1,
                    rail: None,
                    current_override_a: Some(bad),
                });
                p
            },
        ];
        for p in &inputs {
            let b = power_budget(p);
            assert_eq!(b.outcome, Outcome::Fail, "{p:?}");
            assert!(outcomes(&b, P::InvalidInput).contains(&Outcome::Fail));
        }
    }
}

#[test]
fn nonsense_battery_and_rail_index_are_fail() {
    let p = empty_input(Battery {
        empty_v: 9.0,
        ..battery_2s()
    });
    assert_eq!(power_budget(&p).outcome, Outcome::Fail);
    let mut p = empty_input(battery_2s());
    p.boards.push(board("плата", 5.0, 0.1, 0.2, Some(3)));
    assert_eq!(power_budget(&p).outcome, Outcome::Fail);
    let mut p = empty_input(battery_2s());
    p.boards.push(board("плата", 7.4, 0.5, 0.2, None));
    assert_eq!(power_budget(&p).outcome, Outcome::Fail);
}

#[test]
fn nan_duty_in_actuator_load_fails() {
    let a = actuator_by_id("mg996r").unwrap();
    let duty = DutyCycle::new(6.0, 25.0).segment(f64::NAN, 0.5, 1.0);
    let mut p = empty_input(battery_2s());
    p.rails.push(rail("6 В", "lm2596-adj", 6.0));
    p.actuators.push(ActuatorLoad {
        actuator: a,
        driver: None,
        duty: &duty,
        count: 1,
        rail: Some(0),
    });
    assert_has_fail(&power_budget(&p), P::InvalidInput);
}

// Независимая запись корня U² − U_OC·U + P·R = 0 для тестовых напряжений.
fn loaded_voltage(b: &Battery, power: f64) -> f64 {
    let half = b.empty_v * 0.5;
    half + (half * half - b.internal_resistance_ohm.unwrap() * power).sqrt()
}

#[test]
fn totals_are_sums_of_lines() {
    let mut p = empty_input(battery_2s());
    p.boards.push(board("А", 7.4, 0.5, 1.0, None));
    p.boards.push(board("Б", 7.4, 0.25, 0.5, None));
    let b = power_budget(&p);
    assert!((b.total_avg_w - 0.75 * 7.4).abs() < 1e-9);
    assert!((b.total_peak_w - 1.5 * 7.4).abs() < 1e-9);
    let loaded = loaded_voltage(&p.battery, 1.5 * 7.4);
    assert!((b.total_avg_a - 0.75 * 7.4 / loaded).abs() < 1e-9);
    assert_eq!(b.lines.len(), 2);
}

#[test]
fn lm2596_cannot_prove_five_volts_from_6_4_volts() {
    let mut p = empty_input(battery_2s());
    p.rails.push(rail("5 В", "lm2596-adj", 5.0));
    p.boards.push(board("плата", 5.0, 0.2, 0.4, Some(0)));
    let b = power_budget(&p);
    assert_has_fail(&b, P::RailInputVoltage);
    let failure = b
        .findings
        .iter()
        .find(|f| f.kind == P::RailInputVoltage && f.outcome == Outcome::Fail)
        .unwrap();
    assert_eq!((failure.value, failure.limit), (6.4, 6.5));
}

#[test]
fn sag_checks_end_of_discharge_for_direct_sensor() {
    let mut p = empty_input(Battery {
        empty_v: 3.6,
        nominal_v: 3.7,
        full_v: 4.2, // Нижняя граница работы AMT102-V.
        internal_resistance_ohm: Some(0.2),
        ..battery_2s()
    });
    p.sensors.push(SensorLoad {
        sensor: sensor_by_id("amt102-v").unwrap(),
        count: 1,
        rail: None,
        current_override_a: Some(0.02), // Верхний ток тестового экземпляра с обвязкой.
    });
    let b = power_budget(&p);
    assert_eq!(outcomes(&b, P::ConsumerVoltage), vec![Outcome::Pass]);
    assert_has_fail(&b, P::BatterySag);
}

#[test]
fn sensor_override_preserves_published_peak() {
    let mut p = empty_input(battery_2s());
    p.rails.push(rail("3.3 В", "ams1117-3.3", 3.3));
    let s = sensor_by_id("vl53l0x").unwrap();
    p.sensors.push(SensorLoad {
        sensor: s,
        count: 1,
        rail: Some(0),
        current_override_a: s.current_a.get(),
    });
    let b = power_budget(&p);
    assert_eq!(b.lines[0].peak_a, s.current_peak_a.get().unwrap());
}

#[test]
fn overflow_in_unused_runtime_and_nan_efficiency_are_fail() {
    let mut p = empty_input(Battery {
        capacity_ah: f64::MAX,
        ..battery_2s()
    });
    p.boards.push(board("нагрузка", 7.4, 0.1, 0.1, None));
    assert_has_fail(&power_budget(&p), P::InvalidInput);

    let mut r = regulator_by_id("lm2596-adj").unwrap().clone();
    r.efficiency_fallback = Spec::est(f64::NAN, "испорченные входные данные");
    let mut p = empty_input(battery_3s());
    p.rails.push(RailInput {
        name: "5 В".into(),
        regulator: &r,
        output_v: 5.0,
    });
    assert_has_fail(&power_budget(&p), P::InvalidInput);
}
