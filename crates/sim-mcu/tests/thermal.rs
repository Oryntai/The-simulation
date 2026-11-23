//! Заклинивание и тепловая модель обмотки: сверка с моделью двигателя `sim-em`
//! (независимый код: точная экспонента тока, RK4 тепловой сети) и сценарий
//! интегрированной симуляции координатора (сустав заклинен, 25 °C).

use sim_em::motor::{Motor, MotorParams};
use sim_mcu::checks::{exo_jam_restarts, maxon_bench};
use sim_mcu::exo::{ExoCommand, ExoConfig, ExoDiag, ExoMode, fault, pins};

fn cmd(mode: ExoMode, assist: u8, target: f64, payload: f64) -> ExoCommand {
    ExoCommand {
        mode,
        assist_pct: assist,
        target_deg: target,
        payload_kg: payload,
        seq: 0,
    }
}

#[test]
fn bench_and_firmware_thermal_match_sim_em_motor() {
    // Удержание 16 кг при 90° (≈5.9 А) 20 с; ток стенда записываем на каждом шаге
    // и проигрываем в sim-em: Motor::step_current (идеальный токовый контур) + step_thermal.
    let cfg = ExoConfig::maxon_ec90flat_hd50();
    let mut b = maxon_bench(&cfg, 90.0, true, 16.0, 25.0).unwrap();
    let dt = b.dt;
    let mut motor = Motor::new(MotorParams::maxon_ec90flat_260w_hd_csg25_50());
    let c = cmd(ExoMode::GravityAssist, 100, 0.0, 16.0);
    let steps = (20.0 / dt).round() as usize;
    for k in 0..steps {
        if k % 4000 == 0 {
            b.send(c);
        }
        b.step();
        motor.step_current(b.plant.current, 0.0, dt, 48.0);
        motor.step_thermal(dt, 25.0);
    }
    let d = ExoDiag::read(&b.mcu).unwrap();
    println!(
        "20 с при {:.2} А: стенд обмотка {:.3} / корпус {:.3} °C; sim-em {:.3} / {:.3} °C; прошивка {:.3} °C",
        b.plant.current,
        b.plant.t_winding_c,
        b.plant.t_housing_c,
        motor.winding_temp_c(),
        motor.housing_temp_c(),
        d.winding_est_c
    );
    assert!(
        motor.winding_temp_c() > 35.0,
        "сценарий должен заметно греть обмотку"
    );
    assert!((b.plant.t_winding_c - motor.winding_temp_c()).abs() < 0.05);
    assert!((b.plant.t_housing_c - motor.housing_temp_c()).abs() < 0.05);
    assert!((d.winding_est_c - motor.winding_temp_c()).abs() < 0.5);
}

#[test]
fn winding_estimate_survives_watchdog_reset() {
    // Основной цикл «зависает» на 8-й секунде тяжёлого удержания; после сброса
    // сторожем оценка обмотки должна продолжиться из ОЗУ (.noinit), а не начаться
    // заново с температуры корпуса.
    let cfg = ExoConfig {
        test_hang_ms: 8000,
        ..ExoConfig::maxon_ec90flat_hd50()
    };
    let mut b = maxon_bench(&cfg, 90.0, true, 16.0, 25.0).unwrap();
    let c = cmd(ExoMode::GravityAssist, 100, 0.0, 16.0);
    let mut k = 0;
    while b.mcu.watchdog_resets() == 0 {
        assert!(b.time() < 9.0, "сброса сторожем не было");
        if k % 4000 == 0 {
            b.send(c);
        }
        b.step();
        k += 1;
    }
    b.send(c);
    b.run(0.1);
    let d = ExoDiag::read(&b.mcu).unwrap();
    println!(
        "после сброса сторожем: оценка {:.2} °C, стенд {:.2} °C, корпус {:.2} °C",
        d.winding_est_c, b.plant.t_winding_c, b.plant.t_housing_c
    );
    assert!(d.winding_retained);
    assert!(b.plant.t_winding_c - b.plant.t_housing_c > 5.0);
    assert!((d.winding_est_c - b.plant.t_winding_c).abs() < 0.5);
}

#[test]
fn saturated_assist_hold_never_drops_the_load() {
    // Случай координатора: режим 1, человек держит предплечье неподвижно при 90° с
    // грузом 10 кг, поддержка 70 % хочет больше, чем разрешённые i_max = 1.5 А.
    // Ток должен стоять на пределе, драйвер не выключаться.
    let cfg = ExoConfig {
        i_max_a: 1.5,
        ..ExoConfig::maxon_ec90flat_hd50()
    };
    let mut b = maxon_bench(&cfg, 90.0, true, 10.0, 25.0).unwrap();
    let c = cmd(ExoMode::GravityAssist, 70, 0.0, 10.0);
    let need = ExoConfig {
        i_max_a: f32::MAX,
        ..cfg
    }
    .gravity_assist_current(70.0, 10.0, 90.0);
    assert!(need > 1.5, "задание {need} А должно быть выше предела");
    let mut off = 0;
    for k in 0..(5.0 / b.dt) as usize {
        if k % 4000 == 0 {
            b.send(c);
        }
        b.step();
        if !b.mcu.gpio_output(pins::GPIO_ENABLE) && b.time() > 0.01 {
            off += 1;
        }
    }
    let flags2 = b.telemetry.iter().fold(0, |a, (_, t)| a | t.fault_flags2);
    println!(
        "поддержка 70 % у предела 1.5 А 5 с: ток {:.3} А, шагов с выключенным драйвером {off}, флаги2 {flags2:#x}",
        b.plant.current
    );
    assert_eq!(off, 0);
    assert_eq!(flags2 & fault::STALL, 0);
    assert!((b.plant.current - 1.5).abs() < 0.02);
}

#[test]
fn position_hold_near_target_at_limit_is_not_a_stall() {
    // Режим 2, цель в 1° от удерживаемого угла (|e| < 3°), груз 25 кг: ток у предела,
    // сустав стоит — движение не требуется, заклинивания нет.
    let cfg = ExoConfig::maxon_ec90flat_hd50();
    let mut b = maxon_bench(&cfg, 90.0, true, 25.0, 25.0).unwrap();
    let c = cmd(ExoMode::PositionTrack, 0, 91.0, 25.0);
    for k in 0..(3.0 / b.dt) as usize {
        if k % 4000 == 0 {
            b.send(c);
        }
        b.step();
    }
    let flags2 = b.telemetry.iter().fold(0, |a, (_, t)| a | t.fault_flags2);
    assert!(
        b.plant.current > 0.85 * cfg.i_max_a as f64,
        "ток {}",
        b.plant.current
    );
    assert!(b.mcu.gpio_output(pins::GPIO_ENABLE));
    assert_eq!(flags2 & fault::STALL, 0);
}

#[test]
fn coordinator_jam_case_from_25c() {
    // Позиционный режим, сустав заклинен, 25 °C. Без перезапусков защита от
    // заклинивания выключает привод через 1.5 с.
    let cfg = ExoConfig::maxon_ec90flat_hd50();
    let mut b = maxon_bench(&cfg, 60.0, true, 0.0, 25.0).unwrap();
    let c = cmd(ExoMode::PositionTrack, 0, 120.0, 0.0);
    let mut peak: f64 = 0.0;
    let mut trip = None;
    for k in 0..(10.0 / b.dt) as usize {
        if k % 4000 == 0 {
            b.send(c);
        }
        b.step();
        peak = peak.max(b.plant.t_winding_c);
        if trip.is_none() && b.time() > 0.1 && !b.mcu.gpio_output(pins::GPIO_ENABLE) {
            trip = Some(b.time());
        }
    }
    let f2 = b.last_telemetry().unwrap().fault_flags2;
    println!(
        "заклинивание без перезапусков: выключение в t = {:.3} с, пик обмотки {peak:.2} °C, флаги {:?}",
        trip.unwrap(),
        fault::names((f2 as u16) << 8)
    );
    assert!(f2 & fault::STALL != 0);
    assert!(peak < 30.0);

    // Оператор упорно перезапускает (режим 0 → 2 каждые 100 мс после срабатывания):
    // обмотку бережёт только тепловая модель.
    let r = exo_jam_restarts(25.0, 150.0).unwrap();
    println!(
        "заклинивание с перезапусками 150 с от 25 °C: пик оценки {:.1} °C, пик стенда {:.1} °C, корпус {:.1} °C, \
         перезапусков {}, драйвер включён {:.0} %",
        r.est_peak_enabled_c,
        r.true_peak_c,
        r.housing_end_c,
        r.restarts,
        100.0 * r.duty
    );
    let t_max = cfg.t_winding_max_c as f64;
    assert!(r.est_peak_enabled_c <= t_max + 5.0);
    assert!(r.true_peak_c <= t_max + 5.0);
}
