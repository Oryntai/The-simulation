//! Пошаговый API для совместного моделирования: H-мост с ШИМ 20 кГц, шаг 1 мкс, 1 с.

use std::time::Instant;

use sim_circuit::{Circuit, SimError, Transient, examples};

#[test]
fn hbridge_with_diodes_one_second_at_one_microsecond() {
    // Пример «H-мост для прошивки»: ключи из кода, мёртвое время 1 мкс, встроенные диоды.
    let ckt = Circuit::parse(examples::HBRIDGE_EXT.netlist).unwrap();
    let mut tr = Transient::new(&ckt, 1e-6).unwrap();
    let (k, j, b, tl) = (0.05, 2e-5, 1e-5, 0.01);
    let mut omega = 0.0f64;
    let mut omega_avg = 0.0;
    let start = Instant::now();
    tr.set_switch("S4", true).unwrap();
    let n = 1_000_000usize;
    for step in 0..n {
        let ph = step % 50;
        tr.set_switch("S1", (1..30).contains(&ph)).unwrap();
        tr.set_switch("S2", ph >= 31).unwrap();
        tr.step().unwrap();
        let i = tr.current("Lm").unwrap();
        omega += 1e-6 / j * (k * i - b * omega - tl);
        tr.set_source("Vbemf", k * omega).unwrap();
        if step >= n - 1000 {
            omega_avg += omega / 1000.0;
        }
    }
    let el = start.elapsed();
    println!(
        "1 с модельного времени (10⁶ шагов, нелинейная схема): {el:.2?}, ω = {omega_avg:.3} рад/с, {:?}",
        tr.stats()
    );
    assert!((tr.time() - 1.0).abs() < 1e-9);
    assert_eq!(tr.stats().rejected, 0);
    // Усреднённая оценка: ключ сверху открыт 29 мкс из 50, в мёртвое время (2 мкс)
    // ток идёт через нижний диод (≈ −0.8 В); R = 1 + 2·0.01 Ом.
    let v_avg = 12.0 * 29.0 / 50.0 - 0.8 * 2.0 / 50.0;
    let r = 1.02;
    let expected = (k * v_avg - r * tl) / (r * b + k * k);
    assert!(
        (omega_avg - expected).abs() / expected < 0.02,
        "ω = {omega_avg}, оценка {expected}"
    );
}

#[test]
fn tellegen_power_balance() {
    // Сумма мгновенных мощностей всех элементов равна нулю (теорема Теллегена) — независимая
    // проверка токов и мощностей, которые отдаёт API.
    let ckt = Circuit::parse(examples::MOTOR_LOWSIDE_PWM.netlist).unwrap();
    let names: Vec<String> = ckt.element_names().map(String::from).collect();
    let mut tr = Transient::new(&ckt, 1e-6).unwrap();
    for step in 0..3000 {
        tr.step().unwrap();
        if step % 97 == 0 {
            let p: Vec<f64> = names.iter().map(|n| tr.power(n).unwrap()).collect();
            let sum: f64 = p.iter().sum();
            let scale = p.iter().map(|x| x.abs()).fold(0.0, f64::max);
            assert!(
                sum.abs() <= 1e-6 * scale + 1e-9,
                "t = {}: Σp = {sum}, {p:?}",
                tr.time()
            );
        }
    }
    // Источник питания отдаёт энергию: его поглощаемая мощность отрицательна.
    assert!(tr.power("Vbat").unwrap() < 0.0);
}

#[test]
fn api_errors_are_explicit() {
    let ckt = Circuit::parse(examples::MOTOR_LOWSIDE_PWM.netlist).unwrap();
    let mut tr = Transient::new(&ckt, 1e-6).unwrap();
    assert!(matches!(
        tr.set_source("Vbat", 1.0),
        Err(SimError::WrongElement { .. })
    ));
    assert!(matches!(
        tr.set_source("Nope", 1.0),
        Err(SimError::UnknownElement(_))
    ));
    assert!(matches!(
        tr.set_switch("Rm", true),
        Err(SimError::WrongElement { .. })
    ));
    assert!(matches!(
        tr.voltage("nowhere"),
        Err(SimError::UnknownNode(_))
    ));
    assert!(
        tr.set_source("vbemf", 3.0).is_ok(),
        "имена без учёта регистра"
    );
    let e = tr.set_source("Vbat", 1.0).unwrap_err().to_string();
    assert!(e.contains("EXT"), "{e}");
    assert!(Transient::new(&ckt, 0.0).is_err());
    assert!(Transient::new(&ckt, -1.0).is_err());
}

#[test]
fn ext_source_is_interpolated_within_step() {
    // RC, вход — EXT, растёт линейно: трапеции точны для кусочно-линейного входа, поэтому
    // ответ на рампу совпадает с аналитикой v = a·(t − τ(1 − e^(−t/τ))).
    let ckt = Circuit::parse("рампа\nV1 in 0 EXT 0\nR1 in out 1k\nC1 out 0 1u\n").unwrap();
    let dt = 10e-6;
    let mut tr = Transient::new(&ckt, dt).unwrap();
    let a = 1000.0; // В/с
    for k in 1..=300 {
        tr.set_source("V1", a * k as f64 * dt).unwrap();
        tr.step().unwrap();
    }
    let t = tr.time();
    let tau = 1e-3;
    let exact = a * (t - tau * (1.0 - (-t / tau).exp()));
    let got = tr.voltage("out").unwrap();
    assert!((got - exact).abs() < 1e-4 * exact, "{got} vs {exact}");
}

#[test]
fn types_can_live_in_app_resources() {
    // Приложение (Bevy) хранит схему, переходный процесс, ячейку и плату в ресурсах.
    fn assert_send_sync<T: Send + Sync + 'static>() {}
    assert_send_sync::<Circuit>();
    assert_send_sync::<Transient>();
    assert_send_sync::<sim_circuit::battery::Pack>();
    assert_send_sync::<sim_circuit::pcb::Board>();
    assert_send_sync::<sim_circuit::Waveforms>();
}
