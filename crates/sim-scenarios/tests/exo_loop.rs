use sim_scenarios::Scenario;
use sim_scenarios::exo_loop::ExoLoop;
use std::time::Instant;

fn run(sc: &mut ExoLoop, seconds: f64) {
    let n = (seconds / 0.01).round() as usize;
    for _ in 0..n {
        sc.advance(0.01);
    }
}

#[test]
fn gravity_assist_current_matches_hand_calc() {
    let mut sc = ExoLoop::default();
    assert!(sc.firmware_error().is_none(), "{:?}", sc.firmware_error());
    let t0 = Instant::now();
    run(&mut sc, 4.0);
    let wall = t0.elapsed().as_secs_f64();
    let (t, th, i, i_hand, tw, human, wdt, _) = sc.probe();
    println!(
        "t {t:.2} с (за {wall:.2} с реального времени, ×{:.1}), угол {th:.1}°, ток {i:.3} А, ручной {i_hand:.3} А, обмотка {tw:.2} °C, человек {human:.1} Н·м, wdt {wdt}",
        t / wall
    );
    for l in sc.console().iter().take(30) {
        println!("  {l}");
    }
    for c in sc.comparisons() {
        println!(
            "  [{:?}] {} : {:.3} / {:.3} {}",
            c.status, c.title, c.sim, c.reference, c.unit
        );
    }
    for n in sc.notes() {
        println!("  <{:?}> {}", n.level, n.text);
    }
    assert!((th - 90.0).abs() < 5.0, "человек держит 90°");
    assert!(
        (i - i_hand).abs() < 0.05 + 0.05 * i_hand.abs(),
        "ток {i} против {i_hand}"
    );
}

#[test]
fn estop_disables_driver_within_a_tick() {
    let mut sc = ExoLoop::default();
    run(&mut sc, 1.0);
    assert!(sc.driver_enabled());
    sc.run_action("estop");
    run(&mut sc, 0.1);
    let (.., reaction) = sc.probe();
    println!(
        "реакция на аварийную кнопку: {:?}",
        reaction.map(|r| r * 1000.0)
    );
    assert!(!sc.driver_enabled());
    assert!(reaction.unwrap() <= 1.1e-4);
}

#[test]
fn faults_are_caught_by_firmware() {
    use sim_mcu::exo::fault;
    // Обрыв датчика угла → драйвер выключен, флаг SENSOR.
    let mut sc = ExoLoop::default();
    run(&mut sc, 1.0);
    sc.run_action("sensor");
    run(&mut sc, 0.3);
    println!(
        "обрыв датчика: флаги {:?}, драйвер {}",
        fault::names(sc.last_faults()),
        sc.driver_enabled()
    );
    assert!(!sc.driver_enabled());
    assert!(sc.last_faults() & fault::SENSOR as u16 != 0);

    // Пропала связь → через 0.5 с флаг CMD_TIMEOUT, ток снимается плавно.
    let mut sc = ExoLoop::default();
    run(&mut sc, 1.0);
    let (_, _, i_before, ..) = sc.probe();
    sc.run_action("radio");
    run(&mut sc, 1.2);
    let (_, _, i_after, ..) = sc.probe();
    println!("связь пропала: ток {i_before:.2} → {i_after:.2} А");
    assert!(i_after.abs() < 0.2 * i_before.abs() + 0.05);

    // Зависание основного цикла → сброс сторожевым таймером.
    let mut sc = ExoLoop::default();
    run(&mut sc, 0.5);
    sc.run_action("hang");
    run(&mut sc, 1.0);
    let (.., wdt, _) = sc.probe();
    println!("зависание: сбросов сторожевым таймером {wdt}");
    assert!(wdt >= 1);
}

#[test]
fn jam_trips_overcurrent_or_limits_current() {
    use sim_mcu::exo::fault;
    let mut sc = ExoLoop::default();
    sc.set_choice("mode", 2);
    sc.set_param("target", 60.0);
    sc.set_choice("human", 1);
    run(&mut sc, 2.0);
    sc.run_action("jam");
    sc.set_param("target", 110.0);
    run(&mut sc, 2.0);
    let (_, th, i, ..) = sc.probe();
    println!(
        "заклинило: угол {th:.1}°, ток {i:.2} А, флаги {:?}, драйвер {}",
        fault::names(sc.last_faults()),
        sc.driver_enabled()
    );
    // Прошивка не должна гнать ток выше предела и должна распознать заклинивание.
    assert!(i.abs() < 2.0 * 4.1 + 0.5);
    assert!(
        !sc.driver_enabled(),
        "после заклинивания драйвер должен быть выключен"
    );
    assert!(sc.last_faults() & ((fault::STALL as u16) << 8) != 0);
}

#[test]
#[ignore = "долгий: 4 минуты модельного времени"]
fn jam_overheats_winding_before_housing_sensor() {
    let mut sc = ExoLoop::default();
    sc.set_choice("mode", 2);
    sc.set_param("target", 60.0);
    sc.set_choice("human", 1);
    run(&mut sc, 2.0);
    sc.run_action("jam");
    sc.set_param("target", 110.0);
    for k in 0..24 {
        run(&mut sc, 10.0);
        let (t, _, i, _, tw, ..) = sc.probe();
        let housing = sc
            .readouts()
            .iter()
            .find(|r| r.label == "Корпус мотора")
            .unwrap()
            .value;
        println!(
            "t {t:6.1} с: ток {i:5.2} А, обмотка {tw:6.1} °C, корпус {housing:5.1} °C, флаги {:#04x}",
            sc.last_faults()
        );
        let _ = k;
    }
}

#[test]
fn config_editor_round_trip_and_errors() {
    let mut sc = ExoLoop::default();
    let text = sc.text_fields()[0].text.clone();
    assert!(text.contains("stall_time_s"));
    // Правка: предел тока 3 А — ток поддержки должен упереться в него.
    let edited = text.replace(
        &text
            .lines()
            .find(|l| l.starts_with("i_max_a"))
            .unwrap()
            .to_string(),
        "i_max_a = 1.5",
    );
    sc.set_text("config", edited);
    assert!(sc.text_fields()[0].text.contains("i_max_a = 1.5"));
    run(&mut sc, 3.0);
    let (_, _, i, ..) = sc.probe();
    println!(
        "ток при пределе 1.5 А: {i:.3} А, флаги {:?}",
        sim_mcu::exo::fault::names(sc.last_faults())
    );
    for l in sc.console().iter().rev().take(5) {
        println!("  {l}");
    }
    assert!(
        i.abs() <= 1.55 && i.abs() > 1.4,
        "ток должен упереться в предел, а не пропасть"
    );
    assert!(
        sc.driver_enabled(),
        "удержание груза на пределе тока — не заклинивание"
    );
    // Ошибка в тексте не ломает МК и попадает в журнал.
    sc.set_text("config", "i_max_a = много".into());
    assert!(sc.console().iter().any(|l| l.contains("не принята")));
}

#[test]
fn debug_readouts() {
    let mut sc = ExoLoop::default();
    run(&mut sc, 3.0);
    for r in sc.readouts() {
        println!("{:<45} {:.3} {}", r.label, r.value, r.unit);
    }
}
