use std::f64::consts::TAU;

use sim_em::motor::{Motor, MotorParams};
use sim_signal::Time;

use crate::config::{
    Cadence, Config, Coupling, Electrical, Friction, Hold, Mechanics, Mode, Stop, Thermal,
};
use crate::electric::{Bridge, winding};
use crate::model::{Model, Motion};

fn fixture() -> Config {
    // Синтетический стенд: R = 2 Ом, L = 0.5 Гн дают τ = 0.25 с.
    // Единичная передача и коэффициент двигателя упрощают ручную сверку.
    Config {
        electrical: Electrical {
            supply: 8.0,
            limit: 100.0,
            resistance: 2.0,
            inductance: 0.5,
            k: 1.0,
        },
        mechanical: Mechanics {
            ratio: 1.0,
            forward: 1.0,
            backward: 1.0,
            friction: Friction {
                coulomb: 0.0,
                viscous: 0.0,
                stiction: 0.0,
                stribeck: 1.0,
                band: 1.0,
            },
            coupling: None,
            stop: None,
            lock: None,
            brake: None,
        },
        thermal: None,
        mode: Mode::Pwm,
        motor_feedback: false,
        control: Cadence::new(1000.0).unwrap(),
        delay: Time::ZERO,
        bus_range: None,
        pulse: None,
    }
}

fn check(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual:.15e} против {expected:.15e}, допуск {tolerance:.2e}"
    );
}

#[test]
fn locked_current_matches_rl_and_reverse_emf_can_exceed_limit() {
    let config = fixture();
    let mut drive = Model::new(config);
    for i in 0..=1000 {
        let t = f64::from(i) / 1000.0;
        drive
            .update(Time::from_secs(t).unwrap(), Motion::default(), Some(1.0))
            .unwrap();
        // Решение L di/dt + R i = V с неподвижным ротором.
        check(drive.current[0], 4.0 * (1.0 - (-4.0 * t).exp()), 2e-13);
        assert_eq!(drive.current[1], 0.0);
    }
    let p = Electrical {
        limit: 1.0,
        ..config.electrical
    };
    let limited = winding(p, 2.0, 0.0, 0.0, Bridge::Duty(1.0), 0, 1.0).unwrap();
    check(limited.current, 1.0, 1e-14);
    assert!(limited.voltage < 8.0 && !limited.limit_exceeded);
    // Генератор вращается с ЭДС 12 В. Даже при +8 В на мосту i∞ = −2 А.
    let regen = winding(p, 2.0, 0.0, 12.0, Bridge::Current([0.0; 2]), 0, 1.0).unwrap();
    check(regen.current, -2.0 * (1.0 - (-4.0_f64).exp()), 1e-14);
    assert_eq!(regen.voltage, 8.0);
    assert!(regen.limit_exceeded && regen.electrical_j < 0.0);
}

#[test]
fn reversal_crosses_the_full_backlash_before_opposite_torque() {
    // Выход движется по стенду при удержанном роторе: полный зазор 0.4 рад,
    // жёсткость 100 Н·м/рад. Смена стороны контакта идёт через свободный ход.
    let c = Coupling {
        inertia_motor: 1.0,
        half_gap: 0.2,
        stiffness: 100.0,
        damping: 0.0,
    };
    let output_positions = [-0.3, -0.2, -0.1, 0.0, 0.1, 0.199, 0.2, 0.3];
    let reference = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -10.0];
    for (position, expected) in output_positions.into_iter().zip(reference) {
        check(c.force(-position, -1.0).0, expected, 1e-13);
    }
    // Геометрия одного зазора без натяжения противоположной стороны на отрыве.
    let damped = Coupling { damping: 10.0, ..c };
    let (effort, dissipated) = damped.force(0.21, -1.0);
    assert_eq!(effort, 0.0);
    assert!(dissipated > 0.0);
}

#[test]
fn self_lock_brake_and_stop_oppose_motion_with_finite_limits() {
    let base = fixture();
    let mut free = Model::new(base);
    let brake = Hold {
        effort: 3.0,
        stiffness: 100.0,
    };
    let lock = Hold {
        effort: 4.0,
        stiffness: 200.0,
    };
    let mut held = Model::new(Config {
        mechanical: Mechanics {
            lock: Some(lock),
            ..base.mechanical
        },
        ..base
    });
    let mut braked = Model::new(Config {
        mechanical: Mechanics {
            brake: Some(brake),
            ..base.mechanical
        },
        ..base
    });
    for model in [&mut free, &mut held, &mut braked] {
        model.update(Time::ZERO, Motion::default(), None).unwrap();
        model
            .update(Time(1_000_000), Motion { q: 0.01, v: 10.0 }, None)
            .unwrap();
    }
    // Независимое равенство F = kΔq и проверка проворота при перегрузке.
    check(free.effort, 0.0, 1e-13);
    check(held.effort, -2.0, 1e-13);
    check(braked.effort, -1.0, 1e-13);
    held.update(Time(2_000_000), Motion { q: 0.1, v: 0.0 }, None)
        .unwrap();
    braked
        .update(Time(2_000_000), Motion { q: 0.1, v: 0.0 }, None)
        .unwrap();
    check(held.effort, -4.0, 1e-13);
    check(braked.effort, -3.0, 1e-13);
    braked
        .update(Time(3_000_000), Motion { q: 0.1, v: 0.0 }, Some(0.0))
        .unwrap();
    check(braked.effort, 0.0, 1e-13);

    // Упор −1..1 рад, 20 Н·м/рад и 2 Н·м·с/рад.
    let end = Stop {
        min: -1.0,
        max: 1.0,
        stiffness: 20.0,
        damping: 2.0,
    };
    check(end.force(1.1, 0.5), -3.0, 1e-13);
    check(end.force(-1.1, -0.5), 3.0, 1e-13);
    assert_eq!(end.force(1.1, -2.0), 0.0);
    assert_eq!(end.force(0.0, 5.0), 0.0);
}

#[test]
fn two_phase_stepper_slips_under_lock_but_not_when_tracking() {
    let mut config = fixture();
    // Стендовый драйвер: полный шаг, 2 электрических периода за оборот,
    // 8 шагов в секунду. V и R из синтетического стенда.
    config.mode = Mode::StepRate {
        limit: 8.0,
        current: 1.0,
        poles: 2.0,
        steps_per_period: 4.0,
    };
    config.electrical.inductance = 0.005;
    let mut locked = Model::new(config);
    let mut following = Model::new(config);
    let mut seen_torque = false;
    for ms in 0..=1100 {
        let t = f64::from(ms) / 1000.0;
        let speed = TAU;
        locked
            .update(Time::from_secs(t).unwrap(), Motion::default(), Some(8.0))
            .unwrap();
        following
            .update(
                Time::from_secs(t).unwrap(),
                Motion {
                    q: t * speed,
                    v: speed,
                },
                Some(8.0),
            )
            .unwrap();
        seen_torque |= locked.effort.abs() > 0.5;
    }
    // 8 командуемых полношаговых положений, ноль фактически пройдено на стопоре:
    // два полных электрических периода, по 4 шага каждый.
    assert_eq!(locked.control.steps, 8.0);
    assert_eq!(locked.lost_steps, 8.0);
    assert_eq!(following.lost_steps, 0.0);
    assert!(seen_torque);
    check(
        locked.work.residual(locked.stored(Motion::default())),
        0.0,
        2e-12,
    );
}

#[test]
fn stall_against_independently_listed_motor_passport() {
    let reference = MotorParams::maxon_ec90flat_260w_hd_csg25_50();
    let mut config = fixture();
    config.electrical = Electrical {
        supply: reference.nominal_voltage_v,
        resistance: reference.resistance_ohm,
        inductance: reference.inductance_h,
        k: reference.torque_constant_nm_per_a,
        ..config.electrical
    };
    let mut motor = Model::new(config);
    // Неподвижный вал двигателя до редуктора, холодная обмотка. 40τ снимает
    // переходный вклад. Момент из паспорта не входил в модель.
    let duration = 40.0 * config.electrical.inductance / config.electrical.resistance;
    motor
        .update(Time::ZERO, Motion::default(), Some(1.0))
        .unwrap();
    motor
        .update(
            Time::from_secs(duration).unwrap(),
            Motion::default(),
            Some(1.0),
        )
        .unwrap();
    let old = reference.datasheet.stall_torque_nm.unwrap();
    // Точность округления 13.1 Н·м в таблице 2022: ±0.05 Н·м.
    check(motor.effort, old, 0.05);
    let new = 9.41; // Страница 500267, противоречие записано в sim-em и sim-actuators.
    assert!(motor.effort - new > 3.0);
    println!(
        "Стоповый момент: модель {:.6} Н·м, паспорт 2022 {old}, другая редакция {new}",
        motor.effort
    );
}

#[test]
fn heat_and_energy_against_sim_em_on_the_same_reversing_cycle() {
    let reference = MotorParams::maxon_ec90flat_260w_hd_csg25_50();
    let heat = &reference.thermal;
    let electrical = Electrical {
        supply: reference.nominal_voltage_v,
        limit: reference.max_current_a,
        resistance: reference.resistance_ohm,
        inductance: reference.inductance_h,
        k: reference.torque_constant_nm_per_a,
    };
    let mechanics = Mechanics {
        ratio: reference.gear.ratio,
        forward: reference.gear.efficiency_forward,
        backward: reference.gear.efficiency_backward,
        friction: Friction {
            coulomb: reference.coulomb_friction_nm + reference.gear.input_coulomb_nm,
            viscous: reference.viscous_friction_nm_s_per_rad,
            stiction: reference.coulomb_friction_nm + reference.gear.input_coulomb_nm,
            stribeck: 1.0,
            band: reference.coulomb_band_rad_s,
        },
        ..fixture().mechanical
    };
    let thermal = Thermal {
        ambient: reference.resistance_ref_c,
        initial: [reference.resistance_ref_c; 2],
        reference: reference.resistance_ref_c,
        alpha: sim_actuators::spec::ALPHA_CU,
        r_wh: heat.r_winding_housing_k_per_w,
        r_ha: heat.r_housing_ambient_k_per_w,
        c_w: heat.tau_winding_s / heat.r_winding_housing_k_per_w,
        c_h: heat.tau_housing_s / heat.r_housing_ambient_k_per_w,
        winding_fraction: heat.no_load_loss_to_winding,
        gear_fraction: 0.0,
        maximum: Some(heat.max_winding_temp_c),
    };
    // Для одинаковой тепловой границы отключаем потери входа редуктора в обоих
    // расчётах. КПД передачи остаётся и даёт потери, уходящие мимо корпуса.
    let mechanics = Mechanics {
        friction: Friction {
            coulomb: reference.coulomb_friction_nm,
            stiction: reference.coulomb_friction_nm,
            ..mechanics.friction
        },
        ..mechanics
    };
    let mut reference = reference;
    reference.gear.input_coulomb_nm = 0.0;
    let mut oracle = Motor::new(reference);
    let mut drive = Model::new(Config {
        electrical,
        mechanical: mechanics,
        thermal: Some(thermal),
        ..fixture()
    });
    // 40 периодов по 2 с, совместный цикл разгона и рекуперации. 1 мс —
    // передаточная сетка стенда. Обе реализации получают одинаковую траекторию.
    let dt = 0.001;
    let duty = |t: f64| 0.2 * (TAU * t / 2.0).cos();
    let speed = |t: f64| 0.5 * (TAU * t / 2.0).sin();
    let mut joint = Motion::default();
    drive.update(Time::ZERO, joint, Some(duty(0.0))).unwrap();
    let mut max_error = [0.0_f64; 2];
    let mut regenerated = false;
    for n in 1..=80_000 {
        let t = f64::from(n) * dt;
        joint.v = speed(t);
        let supplied = duty(t - dt) * electrical.supply;
        let v = (speed(t - dt) + joint.v) * 0.5;
        let old_electrical = oracle.energy().electrical_in_j;
        oracle.step_electrical(supplied, v, dt);
        oracle.step_thermal(dt, thermal.ambient);
        regenerated |= oracle.energy().electrical_in_j < old_electrical;
        drive
            .update(Time::from_secs(t).unwrap(), joint, Some(duty(t)))
            .unwrap();
        let temperatures = drive.temperatures.unwrap();
        max_error[0] = max_error[0].max((temperatures[0] - oracle.winding_temp_c()).abs());
        max_error[1] = max_error[1].max((temperatures[1] - oracle.housing_temp_c()).abs());
    }
    // Погрешность численных тепловых схем на сетке и округления длинного цикла.
    assert!(max_error[0] < 2e-7 && max_error[1] < 2e-7, "{max_error:?}");
    assert!(oracle.winding_temp_c() - thermal.ambient > 0.5);
    let energy = oracle.energy();
    check(drive.work.electrical, energy.electrical_in_j, 1e-5);
    check(drive.work.output, energy.mechanical_out_j, 1e-5);
    check(drive.work.gear, energy.gear_j, 1e-5);
    check(drive.work.copper, energy.copper_j, 1e-5);
    assert!(regenerated && energy.gear_j > 0.0 && energy.copper_j > 1.0);
    let residual = drive.work.residual(drive.stored(joint));
    check(residual, 0.0, 1e-7);
    println!(
        "Нагрев: max ΔT обмотки {:.9e} °C, корпуса {:.9e} °C, невязка энергии {:.9e} Дж",
        max_error[0], max_error[1], residual
    );
}

#[test]
fn cadence_uses_absolute_ticks_with_fractional_hertz() {
    // 1000.5 Гц = 2001/2. Каждые 2001 такт проходят ровно две секунды.
    let c = Cadence::new(1000.5).unwrap();
    assert_eq!(c.tick(1), Some(Time(2_000_000_000 / 2001)));
    assert_eq!(c.tick(2001), Some(Time(2_000_000_000)));
    assert_eq!(c.tick(2001 * 10_000), Some(Time(2_000_000_000 * 10_000)));
    assert_eq!(c.tick(u64::MAX), None);
}

#[test]
fn asymmetric_gear_loss_is_nonnegative_in_both_directions() {
    // N=3, прямой КПД 75 %, обратный 50 %. При |ω|=6 рад/с и
    // |τ|=2 Н·м мощность со стороны ротора 12 Вт по модулю.
    let m = Mechanics {
        ratio: 3.0,
        forward: 0.75,
        backward: 0.5,
        ..fixture().mechanical
    };
    for sign in [-1.0, 1.0] {
        let speed = 6.0 * sign;
        let from_motor = m.rigid_effort(2.0 * sign, speed);
        let to_motor = m.rigid_effort(-2.0 * sign, speed);
        check(from_motor, 4.5 * sign, 0.0);
        check(to_motor, -12.0 * sign, 0.0);
        check(12.0 - from_motor * speed / m.ratio, 3.0, 0.0);
        check(-12.0 - to_motor * speed / m.ratio, 12.0, 0.0);
        check(m.load_on_motor(from_motor, speed), 2.0 * sign, 0.0);
        check(m.load_on_motor(to_motor, speed), -2.0 * sign, 0.0);
    }
}

#[test]
fn diode_coast_splits_at_zero_and_returns_energy_in_both_directions() {
    // R=2 Ом, L=1/2 Гн, U=8 В, i₀=2 А. Сверяем с интегралом
    // закона Кирхгофа до нуля: R∫i dt = Li₀ + (U_м−ЭДС)t.
    let p = fixture().electrical;
    let dt = p.inductance / p.resistance;
    let initial = 2.0;
    let t_zero = dt * (p.resistance * initial / p.supply).ln_1p();
    let charge = (p.inductance * initial - p.supply * t_zero) / p.resistance;
    let returned = p.supply * charge;
    for sign in [-1.0, 1.0] {
        let coast = winding(p, p.resistance, initial * sign, 0.0, Bridge::Coast, 0, dt).unwrap();
        assert_eq!(coast.current, 0.0);
        check(coast.mean * dt, charge * sign, 1e-15);
        check(coast.electrical_j, -returned, 1e-15);
        check(
            coast.copper_j + returned,
            0.5 * p.inductance * initial * initial,
            1e-14,
        );

        // ЭДС в полтора раза выше шины. После нуля проводит другая пара диодов.
        let emf = 1.5 * p.supply;
        let t_zero = dt * (p.resistance * initial / (p.supply + emf)).ln_1p();
        let coast = winding(
            p,
            p.resistance,
            initial * sign,
            emf * sign,
            Bridge::Coast,
            0,
            dt,
        )
        .unwrap();
        let steady = (p.supply - emf) / p.resistance;
        check(
            coast.current,
            sign * steady * (1.0 - (t_zero / dt - 1.0).exp()),
            1e-14,
        );
        assert!(coast.electrical_j < 0.0 && coast.copper_j > 0.0);
        check(
            0.5 * p.inductance * (coast.current.powi(2) - initial.powi(2))
                + coast.copper_j
                + emf * sign * coast.mean * dt,
            coast.electrical_j,
            1e-14,
        );
    }
    println!("Выбег при нулевой ЭДС: возврат {returned:.9} Дж из 1 Дж поля");
}

#[test]
fn short_dissipates_external_work_in_copper_even_above_active_current_limit() {
    // Из нулевого тока при ЭДС 12 В. Предел активного моста 1 А,
    // но идеальное замыкание не может поднять напряжение для ограничения.
    let p = Electrical {
        limit: 1.0,
        ..fixture().electrical
    };
    let emf = 12.0;
    let dt = p.inductance / p.resistance;
    let short = winding(p, p.resistance, 0.0, emf, Bridge::Short, 0, dt).unwrap();
    let expected = -emf / p.resistance * (1.0 - (-1.0_f64).exp());
    check(short.current, expected, 1e-14);
    assert!(short.limit_exceeded);
    assert_eq!(short.voltage, 0.0);
    assert_eq!(short.electrical_j, 0.0);
    let mechanical_in = -emf * short.mean * dt;
    assert!(mechanical_in > short.copper_j && short.copper_j > 0.0);
    check(
        mechanical_in,
        short.copper_j + 0.5 * p.inductance * expected * expected,
        1e-14,
    );
    let no_inductance = Electrical {
        inductance: 0.0,
        ..p
    };
    let short = winding(no_inductance, p.resistance, 0.0, emf, Bridge::Short, 0, dt).unwrap();
    check(short.current, -emf / p.resistance, 0.0);
    check(short.copper_j, -emf * short.mean * dt, 0.0);
    println!("Замыкание: ток {expected:.9} А, механическая энергия {mechanical_in:.9} Дж");
}
