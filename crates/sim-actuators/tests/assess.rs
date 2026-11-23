//! Вердикт по приводу: негативные сценарии обязаны падать по своей причине,
//! а заведомо нормальный привод — проходить. Тесты проверяют конкретный вид проверки,
//! а не просто наличие какого-то `Fail`.

use sim_actuators::{
    Actuator, ActuatorCheck as C, ActuatorFinding, Backdrive, Brake, Driver, DutyCycle, Outcome,
    RPM, Spec, actuator_by_id, actuators, assess_actuator, driver_by_id, drivers, electrical_load,
    overall,
};

fn act(id: &str) -> &'static Actuator {
    actuator_by_id(id).unwrap_or_else(|| panic!("нет привода {id}"))
}

fn drv(id: &str) -> &'static Driver {
    driver_by_id(id).unwrap_or_else(|| panic!("нет драйвера {id}"))
}

fn get(f: &[ActuatorFinding], kind: C) -> &ActuatorFinding {
    f.iter()
        .find(|x| x.kind == kind)
        .unwrap_or_else(|| panic!("нет проверки {kind:?}"))
}

fn outcome(f: &[ActuatorFinding], kind: C) -> Outcome {
    get(f, kind).outcome
}

fn assert_fail(f: &[ActuatorFinding], kind: C) {
    let x = get(f, kind);
    assert_eq!(x.outcome, Outcome::Fail, "{kind:?}: {x:?}");
    assert_eq!(overall(f), Outcome::Fail);
}

/// Нормальный цикл maxon + ESCON: умеренный момент, запас по всем пределам.
fn good_maxon_duty() -> DutyCycle {
    DutyCycle::new(48.0, 25.0)
        .segment(20.0, 1.0, 2.0)
        .segment(5.0, 1.5, 3.0)
        .segment(0.0, 0.0, 2.0)
        .hold_unpowered(0.0)
}

// ------------------------------------------------------------------------------------
// Обязательные негативные сценарии
// ------------------------------------------------------------------------------------

#[test]
fn sg90_under_arm_load_fails_on_peak_torque() {
    // Предплечье с кистью: около 1.5 кг на плече 15 см, 2.2 Н·м.
    let tau = 1.5 * 9.80665 * 0.15;
    let duty = DutyCycle::new(5.0, 25.0)
        .segment(tau, 0.0, 5.0)
        .hold_unpowered(0.0);
    let f = assess_actuator(act("sg90"), None, &duty);
    assert_fail(&f, C::PeakTorque);
    let p = get(&f, C::PeakTorque);
    assert!((p.value - tau).abs() < 1e-9);
    assert!(p.limit <= 1.2 * sim_actuators::KGF_CM, "предел {}", p.limit);
}

#[test]
fn sg90_uses_low_vendor_torque_not_marketing_figure() {
    // 1.5 кгс·см укладывается в рекламные 1.8, но не в нижнюю оценку 1.2.
    let tau = 1.5 * sim_actuators::KGF_CM;
    let duty = DutyCycle::new(4.8, 25.0)
        .segment(tau, 0.0, 0.2)
        .hold_unpowered(0.0);
    let f = assess_actuator(act("sg90"), None, &duty);
    assert_fail(&f, C::PeakTorque);
}

#[test]
fn l298_above_current_limit_fails_on_driver_current() {
    let a = act("jga25-371-34-12v");
    let line = a.motor_line(12.0, 80.0).unwrap();
    // Момент, которому соответствует ток выше 2.5 А, у этого мотора недостижим:
    // проверяем предел драйвера на искусственно сильном моторе той же схемы.
    let mut strong = a.clone();
    strong.peak_torque_nm = Spec::ds(50.0, "тест");
    let tau = line.torque_for_current(3.0);
    let duty = DutyCycle::new(12.0, 25.0)
        .segment(tau, 0.0, 1.0)
        .hold_unpowered(0.0);
    let f = assess_actuator(&strong, Some(drv("l298n")), &duty);
    let d = get(&f, C::DriverCurrent);
    assert_eq!(d.outcome, Outcome::Fail, "{d:?}");
    assert!(d.value > 2.5, "ток {}", d.value);
}

#[test]
fn l298_current_above_dc_rating_longer_than_pulse_fails() {
    // 2.2 А: ниже повторяющегося пика 2.5 А, выше DC 2 А, держится дольше 10 мс.
    let a = act("jga25-371-34-12v");
    let line = a.motor_line(12.0, 80.0).unwrap();
    let tau = line.torque_for_current(2.2);
    let duty = DutyCycle::new(12.0, 25.0)
        .segment(tau, 0.0, 0.5)
        .segment(0.0, 0.0, 5.0)
        .hold_unpowered(0.0);
    let f = assess_actuator(a, Some(drv("l298n")), &duty);
    assert_fail(&f, C::DriverCurrent);
}

#[test]
fn voltage_above_datasheet_fails() {
    for (id, v) in [
        ("dynamixel-xl330-m288", 7.4),
        ("dynamixel-xl430-w250", 12.6),
        ("sg90", 7.4),
        ("cubemars-ak60-6", 48.0),
        ("n20-100-6v-hp", 12.0),
    ] {
        let duty = DutyCycle::new(v, 25.0)
            .segment(0.01, 0.1, 1.0)
            .hold_unpowered(0.0);
        let f = assess_actuator(act(id), None, &duty);
        assert_eq!(outcome(&f, C::Voltage), Outcome::Fail, "{id} при {v} В");
        assert_eq!(overall(&f), Outcome::Fail, "{id}");
    }
}

#[test]
fn voltage_below_minimum_fails() {
    let duty = DutyCycle::new(9.0, 25.0)
        .segment(0.1, 0.1, 1.0)
        .hold_unpowered(0.0);
    let f = assess_actuator(act("dynamixel-xm430-w350"), None, &duty);
    assert_fail(&f, C::Voltage);
}

#[test]
fn continuous_over_rated_with_allowed_peak_fails_only_on_continuous() {
    // AK80-64: 70 Н·м ниже пика 120, но длительно это выше номинала 48.
    let duty = DutyCycle::new(48.0, 25.0)
        .segment(70.0, 0.2, 60.0)
        .hold_unpowered(0.0);
    let f = assess_actuator(act("cubemars-ak80-64"), None, &duty);
    assert_eq!(
        outcome(&f, C::PeakTorque),
        Outcome::Pass,
        "{:?}",
        get(&f, C::PeakTorque)
    );
    assert_fail(&f, C::ContinuousTorque);
}

#[test]
fn short_peak_above_rated_is_allowed_by_rms() {
    // Тот же момент, но коротко: среднеквадратичный ниже номинала.
    let duty = DutyCycle::new(48.0, 25.0)
        .segment(70.0, 0.2, 0.5)
        .segment(5.0, 0.2, 9.5)
        .hold_unpowered(0.0);
    let f = assess_actuator(act("cubemars-ak80-64"), None, &duty);
    assert_eq!(outcome(&f, C::ContinuousTorque), Outcome::Pass);
    assert_eq!(outcome(&f, C::PeakTorque), Outcome::Pass);
}

#[test]
fn backdrivable_without_brake_holding_load_fails() {
    for id in [
        "cubemars-ak80-64",
        "cubemars-ak70-10",
        "cubemars-ak60-6",
        "maxon-ec90-hd-csg25-50",
        "nema17-17hs19-2004s1",
    ] {
        let v = act(id).voltage_nominal_v.get().unwrap().max(12.0).min(48.0);
        let v = if id == "cubemars-ak60-6" { 24.0 } else { v };
        // Даже нагрузка меньше момента проворота не считается удержанной.
        let duty = DutyCycle::new(v, 25.0)
            .segment(0.1, 0.0, 1.0)
            .hold_unpowered(0.1);
        let f = assess_actuator(act(id), None, &duty);
        assert_fail(&f, C::HoldWhenUnpowered);
    }
}

#[test]
fn brake_with_enough_torque_holds_and_weak_brake_fails() {
    let mut a = act("cubemars-ak80-64").clone();
    a.brake = Brake::Holding {
        torque_nm: Spec::ds(10.0, "тестовый тормоз"),
    };
    let ok = DutyCycle::new(48.0, 25.0)
        .segment(5.0, 0.0, 1.0)
        .hold_unpowered(8.0);
    assert_eq!(
        outcome(&assess_actuator(&a, None, &ok), C::HoldWhenUnpowered),
        Outcome::Pass
    );
    let weak = ok.clone().hold_unpowered(12.0);
    assert_eq!(
        outcome(&assess_actuator(&a, None, &weak), C::HoldWhenUnpowered),
        Outcome::Fail
    );
}

#[test]
fn unknown_hold_data_is_not_a_pass() {
    let duty = DutyCycle::new(5.0, 25.0).segment(0.05, 0.0, 1.0);
    let f = assess_actuator(act("dynamixel-xl330-m288"), None, &duty);
    assert_eq!(outcome(&f, C::HoldWhenUnpowered), Outcome::NotAnalyzed);
    let holding = duty.clone().hold_unpowered(0.05);
    let f = assess_actuator(act("dynamixel-xl330-m288"), None, &holding);
    assert_eq!(outcome(&f, C::HoldWhenUnpowered), Outcome::NotAnalyzed);
    assert_ne!(overall(&f), Outcome::Pass);
}

// ------------------------------------------------------------------------------------
// NaN и бесконечность
// ------------------------------------------------------------------------------------

#[test]
fn nan_and_infinity_in_duty_are_fail_everywhere() {
    let base = good_maxon_duty();
    let a = act("maxon-ec90-hd-csg25-50");
    let d = drv("escon-50-5");
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut variants = Vec::new();
        for field in 0..6 {
            let mut x = base.clone();
            match field {
                0 => x.segments[0].torque_nm = bad,
                1 => x.segments[0].speed_rad_s = bad,
                2 => x.segments[0].duration_s = bad,
                3 => x.ambient_c = bad,
                4 => x.supply_voltage_v = bad,
                _ => x.unpowered_hold_torque_nm = Some(bad),
            }
            variants.push(x);
        }
        for x in variants {
            let f = assess_actuator(a, Some(d), &x);
            assert_eq!(f.len(), C::ALL.len());
            assert!(
                f.iter().all(|r| r.outcome == Outcome::Fail),
                "вход {x:?}: {f:?}"
            );
            assert!(f.iter().all(|r| r.value.is_finite() && r.limit.is_finite()));
            assert_eq!(overall(&f), Outcome::Fail);
        }
    }
}

#[test]
fn nan_in_part_data_is_fail() {
    let duty = good_maxon_duty();
    let mut a = act("maxon-ec90-hd-csg25-50").clone();
    a.resistance_ohm = Spec::ds(f64::NAN, "порча");
    assert_eq!(
        overall(&assess_actuator(&a, Some(drv("escon-50-5")), &duty)),
        Outcome::Fail
    );

    let mut a = act("maxon-ec90-hd-csg25-50").clone();
    a.max_winding_temp_c = Spec::ds(f64::INFINITY, "порча");
    assert_eq!(
        overall(&assess_actuator(&a, Some(drv("escon-50-5")), &duty)),
        Outcome::Fail
    );

    let mut d = drv("escon-50-5").clone();
    d.current_peak_a = Spec::ds(f64::NAN, "порча");
    assert_eq!(
        overall(&assess_actuator(
            act("maxon-ec90-hd-csg25-50"),
            Some(&d),
            &duty
        )),
        Outcome::Fail
    );
}

#[test]
fn invalid_duty_shapes_are_fail() {
    let a = act("maxon-ec90-hd-csg25-50");
    let empty = DutyCycle::new(48.0, 25.0);
    assert_eq!(overall(&assess_actuator(a, None, &empty)), Outcome::Fail);
    let zero = DutyCycle::new(48.0, 25.0).segment(1.0, 1.0, 0.0);
    assert_eq!(overall(&assess_actuator(a, None, &zero)), Outcome::Fail);
    let negative = DutyCycle::new(-48.0, 25.0).segment(1.0, 1.0, 1.0);
    assert_eq!(overall(&assess_actuator(a, None, &negative)), Outcome::Fail);
    let hold = DutyCycle::new(48.0, 25.0)
        .segment(1.0, 1.0, 1.0)
        .hold_unpowered(-1.0);
    assert_eq!(overall(&assess_actuator(a, None, &hold)), Outcome::Fail);
    let cold = DutyCycle::new(48.0, -300.0).segment(1.0, 1.0, 1.0);
    assert_eq!(overall(&assess_actuator(a, None, &cold)), Outcome::Fail);
}

#[test]
fn no_catalog_part_ever_passes_on_garbage() {
    for a in actuators() {
        for bad in [f64::NAN, f64::INFINITY] {
            let duty = DutyCycle::new(bad, 25.0)
                .segment(0.01, 0.01, 1.0)
                .hold_unpowered(0.0);
            let f = assess_actuator(a, None, &duty);
            assert!(f.iter().all(|x| x.outcome == Outcome::Fail), "{}", a.id);
        }
    }
}

// ------------------------------------------------------------------------------------
// Кривая момент–скорость
// ------------------------------------------------------------------------------------

#[test]
fn working_point_uses_torque_speed_curve_not_nameplate() {
    // XM430 при 12 В: 1.5 Н·м ниже стопового и 4.3 рад/с ниже холостого хода (4.82),
    // но вместе эта точка лежит выше прямой.
    let a = act("dynamixel-xm430-w350");
    let duty = DutyCycle::new(12.0, 25.0)
        .segment(1.5, 4.3, 0.5)
        .hold_unpowered(0.0);
    assert!(duty.peak_speed_rad_s() < 46.0 * RPM);
    let f = assess_actuator(a, None, &duty);
    assert_eq!(
        outcome(&f, C::PeakTorque),
        Outcome::NotAnalyzed,
        "{:?}",
        get(&f, C::PeakTorque)
    );
    assert_fail(&f, C::Speed);
    // Более лёгкая точка внутри модели. Для подтверждения у ROBOTIS нужна кривая N-T.
    let light = DutyCycle::new(12.0, 25.0)
        .segment(0.05, 3.5, 0.5)
        .hold_unpowered(0.0);
    let f = assess_actuator(a, None, &light);
    assert_eq!(outcome(&f, C::Speed), Outcome::NotAnalyzed);
    assert!(get(&f, C::Speed).limit > light.peak_speed_rad_s());
}

#[test]
fn lower_voltage_means_less_speed_and_torque() {
    let a = act("dynamixel-xm430-w350");
    let s = |v: f64| a.available_torque_nm(v, 0.0, 25.0).unwrap();
    assert!(s(11.1) < s(12.0) && s(12.0) < s(14.8));
    // Заявленная при 14.8 В скорость при 12 В недостижима.
    let duty = DutyCycle::new(12.0, 25.0)
        .segment(0.05, 55.0 * RPM, 0.5)
        .hold_unpowered(0.0);
    assert_fail(&assess_actuator(a, None, &duty), C::Speed);
}

#[test]
fn hot_winding_reduces_capability() {
    let a = act("n20-100-6v-hp");
    let cold = a.available_torque_nm(6.0, 0.0, 25.0).unwrap();
    let hot = a.available_torque_nm(6.0, 0.0, 100.0).unwrap();
    assert!(hot < 0.8 * cold, "{hot} против {cold}");
    // Ниже 25 °C выигрыш не засчитывается.
    assert_eq!(a.available_torque_nm(6.0, 0.0, -20.0).unwrap(), cold);
}

#[test]
fn l298_voltage_drop_costs_speed_compared_to_tb6612() {
    // JGA25 от 12 В: падение L298 до 3.2+ В отнимает скорость, у TB6612 оно 0.2 В.
    let a = act("jga25-371-34-12v");
    let duty = DutyCycle::new(12.0, 25.0)
        .segment(0.05, 9.5, 1.0)
        .hold_unpowered(0.0);
    let with_tb = assess_actuator(a, Some(drv("tb6612fng")), &duty);
    let with_l298 = assess_actuator(a, Some(drv("l298n")), &duty);
    assert_eq!(outcome(&with_tb, C::Speed), Outcome::NotAnalyzed);
    assert!(get(&with_tb, C::Speed).limit > duty.peak_speed_rad_s());
    assert_fail(&with_l298, C::Speed);
}

#[test]
fn overvoltage_gives_no_extra_speed_credit() {
    // MG996R разрешён до 7.2 В, но данных выше 6 В нет: скорость считается как при 6 В.
    let a = act("mg996r");
    let at_6 = a.motor_line(6.0, 25.0).unwrap().available_speed(0.0, 6.0);
    let at_7 = a.motor_line(7.2, 25.0).unwrap().available_speed(0.0, 7.2);
    assert!((at_6 - at_7).abs() < 1e-9, "{at_6} / {at_7}");
}

#[test]
fn stepper_speed_can_only_be_disproved() {
    let a = act("nema17-17hs19-2004s1");
    let d = drv("drv8825");
    // Умеренная точка: верхняя граница не нарушена, но это не доказательство.
    let duty = DutyCycle::new(24.0, 25.0)
        .segment(0.2, 5.0, 1.0)
        .hold_unpowered(0.0);
    let f = assess_actuator(a, Some(d), &duty);
    assert_eq!(
        outcome(&f, C::Speed),
        Outcome::NotAnalyzed,
        "{:?}",
        get(&f, C::Speed)
    );
    // Полный момент на большой скорости физически недостижим: индуктивность не даст тока.
    let fast = DutyCycle::new(24.0, 25.0)
        .segment(0.5, 100.0, 1.0)
        .hold_unpowered(0.0);
    assert_fail(&assess_actuator(a, Some(d), &fast), C::Speed);
}

// ------------------------------------------------------------------------------------
// Ток, драйвер, нагрев
// ------------------------------------------------------------------------------------

#[test]
fn motor_current_over_rated_fails() {
    let a = act("n20-100-6v-hp");
    // Pololu: длительно не выше 25 % стопового тока (0.4 А).
    let duty = DutyCycle::new(6.0, 25.0)
        .segment(0.08, 0.0, 10.0)
        .hold_unpowered(0.0);
    let f = assess_actuator(a, Some(drv("drv8833")), &duty);
    assert_fail(&f, C::Current);
}

#[test]
fn missing_driver_is_never_pass() {
    let a = act("jga25-371-34-12v");
    let duty = DutyCycle::new(12.0, 25.0)
        .segment(0.05, 1.0, 1.0)
        .hold_unpowered(0.0);
    let f = assess_actuator(a, None, &duty);
    for k in [
        C::DriverCompatibility,
        C::DriverCurrent,
        C::DriverVoltage,
        C::DriverTemperature,
    ] {
        assert_eq!(outcome(&f, k), Outcome::NotAnalyzed, "{k:?}");
    }
    assert_eq!(overall(&f), Outcome::NotAnalyzed);
}

#[test]
fn incompatible_drivers_fail() {
    let duty = DutyCycle::new(24.0, 25.0)
        .segment(0.1, 0.1, 1.0)
        .hold_unpowered(0.0);
    // Шаговый драйвер на коллекторный мотор.
    let f = assess_actuator(act("jga25-371-34-12v"), Some(drv("a4988")), &duty);
    assert_fail(&f, C::DriverCompatibility);
    // H-мост на BLDC.
    let f = assess_actuator(act("maxon-ec90-hd-csg25-50"), Some(drv("l298n")), &duty);
    assert_fail(&f, C::DriverCompatibility);
    // Внешний драйвер на привод со встроенным.
    let f = assess_actuator(act("cubemars-ak80-64"), Some(drv("escon-50-5")), &duty);
    assert_fail(&f, C::DriverCompatibility);
    // Встроенный драйвер чужого привода.
    let f = assess_actuator(
        act("jga25-371-34-12v"),
        Some(drv("builtin-dynamixel-xm430")),
        &duty,
    );
    assert_fail(&f, C::DriverCompatibility);
}

#[test]
fn builtin_driver_is_used_automatically() {
    let duty = DutyCycle::new(12.0, 25.0)
        .segment(0.5, 0.5, 1.0)
        .hold_unpowered(0.0);
    let f = assess_actuator(act("dynamixel-xm430-w350"), None, &duty);
    assert_eq!(outcome(&f, C::DriverCompatibility), Outcome::Pass);
    assert_eq!(outcome(&f, C::DriverVoltage), Outcome::Pass);
}

#[test]
fn driver_voltage_limits_fail() {
    let a = act("jga25-371-34-12v");
    // DRV8833 — до 10.8 В; мотор на 12 В допустим, драйвер нет.
    let duty = DutyCycle::new(12.0, 25.0)
        .segment(0.02, 1.0, 1.0)
        .hold_unpowered(0.0);
    let f = assess_actuator(a, Some(drv("drv8833")), &duty);
    assert_eq!(outcome(&f, C::Voltage), Outcome::NotAnalyzed);
    assert_fail(&f, C::DriverVoltage);
    // A4988 и DRV8825 не запускаются ниже 8 В.
    let n = act("nema17-17hs19-2004s1");
    let low = DutyCycle::new(5.0, 25.0)
        .segment(0.05, 0.0, 1.0)
        .hold_unpowered(0.0);
    assert_fail(
        &assess_actuator(n, Some(drv("a4988")), &low),
        C::DriverVoltage,
    );
}

#[test]
fn tb6612_low_voltage_current_limit_applies() {
    let a = act("n20-100-6v-hp");
    let line = a.motor_line(3.3, 80.0).unwrap();
    let tau = line.torque_for_current(0.6);
    let duty = DutyCycle::new(3.3, 25.0)
        .segment(tau, 0.0, 5.0)
        .hold_unpowered(0.0);
    let f = assess_actuator(a, Some(drv("tb6612fng")), &duty);
    let d = get(&f, C::DriverCurrent);
    assert_eq!(d.outcome, Outcome::Fail, "{d:?}");
    assert!((d.limit - 0.4).abs() < 1e-9, "{d:?}");
}

#[test]
fn l298_without_heatsink_overheats_and_heatsink_helps() {
    let a = act("jga25-371-34-12v");
    let line = a.motor_line(12.0, 80.0).unwrap();
    let tau = line.torque_for_current(0.95);
    let duty = DutyCycle::new(12.0, 25.0)
        .segment(tau, 0.0, 600.0)
        .hold_unpowered(0.0);
    let bare = drv("l298n");
    let f = assess_actuator(a, Some(bare), &duty);
    assert_fail(&f, C::DriverTemperature);
    let cooled = bare.with_heatsink(5.0).unwrap();
    let f = assess_actuator(a, Some(&cooled), &duty);
    assert_eq!(
        outcome(&f, C::DriverTemperature),
        Outcome::Pass,
        "{:?}",
        get(&f, C::DriverTemperature)
    );
    assert!(bare.with_heatsink(f64::NAN).is_none());
}

#[test]
fn bts7960_headline_current_fails_without_heatsink() {
    // Искусственно сильный коллекторный мотор: 25 А длительно — ниже заголовочных 43 А.
    let mut a = act("jga25-371-34-12v").clone();
    a.voltage_points = &[];
    a.resistance_ohm = Spec::ds(0.1, "тест");
    a.kt_motor_nm_per_a = Spec::ds(0.05, "тест");
    a.gear_ratio = Spec::ds(10.0, "тест");
    a.gear_efficiency = Spec::ds(1.0, "тест");
    a.no_load_current_a = Spec::NONE;
    a.peak_torque_nm = Spec::ds(100.0, "тест");
    let duty = DutyCycle::new(12.0, 25.0)
        .segment(12.5, 0.0, 120.0)
        .hold_unpowered(0.0);
    let f = assess_actuator(&a, Some(drv("bts7960")), &duty);
    assert_eq!(
        outcome(&f, C::DriverCurrent),
        Outcome::Pass,
        "{:?}",
        get(&f, C::DriverCurrent)
    );
    assert_fail(&f, C::DriverTemperature);
}

#[test]
fn winding_overheats_on_long_overload() {
    let a = act("maxon-ec90-hd-csg25-50");
    // 8 А длительно: вдвое выше номинала.
    let line = a.motor_line(48.0, 100.0).unwrap();
    let tau = line.torque_for_current(8.0);
    let duty = DutyCycle::new(48.0, 25.0)
        .segment(tau, 0.5, 3600.0)
        .hold_unpowered(0.0);
    let f = assess_actuator(a, Some(drv("escon-50-5")), &duty);
    let w = get(&f, C::WindingTemperature);
    assert_eq!(w.outcome, Outcome::Fail, "{w:?}");
    assert!(w.value > 125.0);
}

#[test]
fn hot_air_shrinks_continuous_torque() {
    let a = act("maxon-ec90-hd-csg25-50");
    let rated = a.rated_torque_nm.get().unwrap();
    let tau = 0.95 * rated;
    let cool = DutyCycle::new(48.0, 25.0)
        .segment(tau, 0.5, 100.0)
        .hold_unpowered(0.0);
    let hot = DutyCycle::new(48.0, 60.0)
        .segment(tau, 0.5, 100.0)
        .hold_unpowered(0.0);
    assert_eq!(
        outcome(
            &assess_actuator(a, Some(drv("escon-50-5")), &cool),
            C::ContinuousTorque
        ),
        Outcome::NotAnalyzed
    );
    assert_fail(
        &assess_actuator(a, Some(drv("escon-50-5")), &hot),
        C::ContinuousTorque,
    );
}

#[test]
fn ambient_limit_fails() {
    let duty = DutyCycle::new(5.0, 75.0)
        .segment(0.05, 0.1, 1.0)
        .hold_unpowered(0.0);
    let f = assess_actuator(act("dynamixel-xl330-m288"), None, &duty);
    assert_fail(&f, C::AmbientTemperature);
    let mut cold = duty;
    cold.ambient_c = -10.0;
    let f = assess_actuator(act("dynamixel-xl330-m288"), None, &cold);
    assert_fail(&f, C::AmbientTemperature);
    assert_eq!(get(&f, C::AmbientTemperature).limit, -5.0);
}

#[test]
fn escon_ambient_limit_fails() {
    let mut duty = good_maxon_duty();
    duty.ambient_c = 50.0;
    let f = assess_actuator(
        act("maxon-ec90-hd-csg25-50"),
        Some(drv("escon-50-5")),
        &duty,
    );
    assert_fail(&f, C::DriverTemperature);
}

// ------------------------------------------------------------------------------------
// «Оценка» не доказывает «годится»
// ------------------------------------------------------------------------------------

#[test]
fn estimated_thermal_can_fail_but_never_pass() {
    let mut a = act("cubemars-ak80-64").clone();
    // Проверяем оценки из тепловой модели sim-em отдельно от каталога.
    a.r_th_winding_housing_k_per_w = Spec::est(1.5, "тепловая модель sim-em");
    a.r_th_housing_ambient_k_per_w = Spec::est(3.87, "тепловая модель sim-em");
    a.tau_winding_s = Spec::est(60.0, "тепловая модель sim-em");
    a.tau_housing_s = Spec::est(1600.0, "тепловая модель sim-em");
    a.max_winding_temp_c = Spec::est(100.0, "тепловая модель sim-em");
    let light = DutyCycle::new(48.0, 25.0)
        .segment(5.0, 0.1, 100.0)
        .hold_unpowered(0.0);
    let f = assess_actuator(&a, None, &light);
    assert_eq!(outcome(&f, C::WindingTemperature), Outcome::NotAnalyzed);
    assert_ne!(overall(&f), Outcome::Pass);
    let heavy = DutyCycle::new(48.0, 25.0)
        .segment(100.0, 0.1, 3600.0)
        .hold_unpowered(0.0);
    let f = assess_actuator(&a, None, &heavy);
    assert_fail(&f, C::WindingTemperature);
}

#[test]
fn missing_thermal_data_is_not_analyzed() {
    let duty = DutyCycle::new(24.0, 25.0)
        .segment(1.0, 1.0, 10.0)
        .hold_unpowered(0.0);
    let f = assess_actuator(act("cubemars-ak60-6"), None, &duty);
    assert_eq!(outcome(&f, C::WindingTemperature), Outcome::NotAnalyzed);
    assert_ne!(overall(&f), Outcome::Pass);
}

#[test]
fn servo_without_rated_torque_is_not_a_continuous_pass() {
    let duty = DutyCycle::new(12.0, 25.0)
        .segment(0.5, 0.5, 100.0)
        .hold_unpowered(0.0);
    let f = assess_actuator(act("dynamixel-xm430-w350"), None, &duty);
    assert_eq!(outcome(&f, C::ContinuousTorque), Outcome::NotAnalyzed);
    assert_ne!(overall(&f), Outcome::Pass);
}

#[test]
fn ds3218_stall_current_does_not_prove_continuous_current() {
    let duty = DutyCycle::new(6.0, 25.0)
        .segment(0.5, 0.5, 1.0)
        .hold_unpowered(0.0);
    let f = assess_actuator(act("ds3218"), None, &duty);
    assert_eq!(outcome(&f, C::Current), Outcome::NotAnalyzed);
    assert!(
        electrical_load(act("ds3218"), None, &duty)
            .unwrap()
            .bus_current_peak_a
            >= 2.6
    );
}

// ------------------------------------------------------------------------------------
// Положительные сценарии: проверка не должна быть «всегда Fail»
// ------------------------------------------------------------------------------------

#[test]
fn good_design_gets_full_pass() {
    let mut a = act("maxon-ec90-hd-csg25-50").clone();
    let d = drv("escon-50-5");
    let duty = good_maxon_duty();
    let catalog_findings = assess_actuator(&a, Some(d), &duty);
    assert_eq!(overall(&catalog_findings), Outcome::NotAnalyzed);
    assert_eq!(
        outcome(&catalog_findings, C::ContinuousTorque),
        Outcome::NotAnalyzed
    );

    // Синтетический вход с подтверждёнными паспортными пробелами. Значения взяты
    // из исходных оценок, чтобы проверить влияние происхождения независимо от числа.
    a.voltage_min_v = Spec::measured(duty.supply_voltage_v, "синтетическая калибровка питания");
    a.gear_efficiency = Spec::measured(
        a.gear_efficiency.get().unwrap(),
        "синтетическая калибровка КПД",
    );
    a.rated_torque_nm = Spec::measured(
        a.rated_torque_nm.get().unwrap(),
        "синтетический длительный замер",
    );
    let f = assess_actuator(&a, Some(d), &duty);
    for x in &f {
        assert_eq!(x.outcome, Outcome::Pass, "{x:?}");
    }
    assert_eq!(overall(&f), Outcome::Pass);
}

#[test]
fn every_check_is_reported_once_in_order() {
    let f = assess_actuator(
        act("sg90"),
        None,
        &DutyCycle::new(5.0, 25.0).segment(0.01, 0.1, 1.0),
    );
    let kinds: Vec<C> = f.iter().map(|x| x.kind).collect();
    assert_eq!(kinds, C::ALL.to_vec());
}

#[test]
fn all_findings_are_finite_for_all_parts_and_drivers() {
    for a in actuators() {
        let v = a.voltage_nominal_v.get().unwrap_or(12.0);
        let duty = DutyCycle::new(v, 30.0)
            .segment(0.05, 0.2, 1.0)
            .segment(-0.05, 0.2, 1.0)
            .segment(0.0, 0.0, 1.0)
            .hold_unpowered(0.0);
        let mut options: Vec<Option<&Driver>> = vec![None];
        options.extend(drivers().iter().map(Some));
        for d in options {
            let f = assess_actuator(a, d, &duty);
            assert_eq!(f.len(), C::ALL.len());
            for x in &f {
                assert!(
                    x.value.is_finite() && x.limit.is_finite(),
                    "{} {:?}",
                    a.id,
                    x
                );
                assert!(!x.detail.is_empty(), "{} {:?}", a.id, x.kind);
            }
        }
    }
}

#[test]
fn overall_rules() {
    let pass = ActuatorFinding {
        kind: C::Voltage,
        outcome: Outcome::Pass,
        value: 1.0,
        limit: 2.0,
        unit: "В",
        detail: "x".into(),
    };
    let na = ActuatorFinding {
        outcome: Outcome::NotAnalyzed,
        ..pass.clone()
    };
    let fail = ActuatorFinding {
        outcome: Outcome::Fail,
        ..pass.clone()
    };
    let nan = ActuatorFinding {
        value: f64::NAN,
        ..pass.clone()
    };
    assert_eq!(overall(&[]), Outcome::NotAnalyzed);
    assert_eq!(overall(std::slice::from_ref(&pass)), Outcome::NotAnalyzed);
    assert_eq!(
        overall(&C::ALL.map(|kind| ActuatorFinding {
            kind,
            ..pass.clone()
        })),
        Outcome::Pass
    );
    assert_eq!(overall(&[pass.clone(), na.clone()]), Outcome::NotAnalyzed);
    assert_eq!(overall(&[pass.clone(), na, fail]), Outcome::Fail);
    assert_eq!(overall(&[pass, nan]), Outcome::Fail);
}

#[test]
fn braking_does_not_subtract_energy_or_hide_copper_loss() {
    let a = act("maxon-ec90-hd-csg25-50");
    let motoring = DutyCycle::new(48.0, 25.0).segment(10.0, 2.0, 1.0);
    let braking = DutyCycle::new(48.0, 25.0).segment(-10.0, 2.0, 1.0);
    let m = electrical_load(a, None, &motoring).unwrap();
    let b = electrical_load(a, None, &braking).unwrap();
    assert!(m.power_avg_w > 20.0, "{m:?}");
    assert!(
        0.0 < b.power_avg_w && b.power_avg_w < m.power_avg_w,
        "{b:?}"
    );
    assert_eq!(
        outcome(
            &assess_actuator(a, Some(drv("escon-50-5")), &braking),
            C::Speed
        ),
        Outcome::NotAnalyzed
    );
}

#[test]
fn driver_pulse_is_continuous_across_cycle_boundary() {
    let a = act("jga25-371-34-12v");
    let d = drv("l298n");
    let t = d.peak_max_duration_s.get().unwrap();
    let i = (d.current_continuous_a.get().unwrap() + d.current_peak_a.get().unwrap()) * 0.5;
    let torque = a.motor_line(12.0, 80.0).unwrap().torque_for_current(i);
    let part = 0.75 * t;
    let duty = DutyCycle::new(12.0, 25.0)
        .segment(torque, 0.0, part)
        .segment(0.0, 0.0, 1.0)
        .segment(torque, 0.0, part);
    let f = assess_actuator(a, Some(d), &duty);
    assert_fail(&f, C::DriverCurrent);
    let f = get(&f, C::DriverCurrent);
    assert_eq!(f.unit, "с");
    assert_eq!(f.value, 2.0 * part);
    assert_eq!(f.limit, t);
}

#[test]
fn invalid_secondary_parameters_fail() {
    let mut stepper = act("nema17-17hs19-2004s1").clone();
    stepper.stepper_pole_pairs = Some(f64::NAN);
    assert_eq!(
        overall(&assess_actuator(
            &stepper,
            Some(drv("drv8825")),
            &good_maxon_duty()
        )),
        Outcome::Fail
    );

    let mut driver = drv("escon-50-5").clone();
    driver.low_voltage_current = Some((f64::INFINITY, 1.0));
    assert_eq!(
        overall(&assess_actuator(
            act("maxon-ec90-hd-csg25-50"),
            Some(&driver),
            &good_maxon_duty()
        )),
        Outcome::Fail
    );
}

#[test]
fn drv8833_rated_current_requires_its_rated_voltage() {
    let a = act("n20-100-6v-hp");
    let d = drv("drv8833");
    let rated_v = d.current_rating_min_voltage_v.get().unwrap();
    let low_v = d.voltage_min_v.get().unwrap();
    let torque = a
        .motor_line(rated_v, 80.0)
        .unwrap()
        .torque_for_current(a.rated_current_a.get().unwrap());
    let duty = |volts| DutyCycle::new(volts, 25.0).segment(torque, 0.0, 1.0);
    assert_eq!(
        outcome(
            &assess_actuator(a, Some(d), &duty(rated_v)),
            C::DriverCurrent
        ),
        Outcome::Pass
    );
    assert_eq!(
        outcome(&assess_actuator(a, Some(d), &duty(low_v)), C::DriverCurrent),
        Outcome::NotAnalyzed
    );
}

#[test]
fn self_locking_does_not_make_motor_peak_a_holding_rating() {
    let mut a = act("maxon-ec90-hd-csg25-50").clone();
    a.backdrive = Backdrive::SelfLocking {
        source: "проверка синтетической детали",
    };
    let f = assess_actuator(
        &a,
        Some(drv("escon-50-5")),
        &good_maxon_duty().hold_unpowered(1.0),
    );
    assert_eq!(outcome(&f, C::HoldWhenUnpowered), Outcome::NotAnalyzed);
}
