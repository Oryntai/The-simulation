//! Каждая схема из библиотеки примеров разбирается и считается; ключевые величины разумны.

use sim_circuit::{Circuit, examples, operating_point, run_tran};

#[test]
fn every_example_runs() {
    for ex in examples::all() {
        let ckt =
            Circuit::parse(ex.netlist).unwrap_or_else(|e| panic!("{}: ошибка разбора: {e}", ex.id));
        assert!(!ex.description.is_empty() && !ex.title.is_empty());
        if ckt.tran_spec().is_some() {
            let wf = run_tran(&ckt).unwrap_or_else(|e| panic!("{}: {e}", ex.id));
            assert!(wf.len() > 10, "{}: мало точек", ex.id);
            for (name, s) in &wf.signals {
                assert!(
                    s.iter().all(|v| v.is_finite()),
                    "{}: {name} не конечен",
                    ex.id
                );
            }
        } else {
            operating_point(&ckt).unwrap_or_else(|e| panic!("{}: {e}", ex.id));
        }
        assert_eq!(examples::by_id(ex.id).map(|e| e.id), Some(ex.id));
    }
}

#[test]
fn example_values_are_sane() {
    let op = operating_point(&Circuit::parse(examples::LED_RESISTOR.netlist).unwrap()).unwrap();
    let i = -op.current("V1").unwrap();
    assert!((0.018..0.022).contains(&i), "ток светодиода {i}");
    let op = operating_point(&Circuit::parse(examples::NTC_DIVIDER.netlist).unwrap()).unwrap();
    assert!((op.voltage("out").unwrap() - 1.65).abs() < 1e-6);

    let wf = run_tran(&Circuit::parse(examples::RC_LOWPASS.netlist).unwrap()).unwrap();
    // Ступенька в 0.5 мс, τ = 1 мс: при 1.5 мс выход ≈ 5·(1 − e^−1).
    let v = wf.at("v(out)", 1.5e-3).unwrap();
    assert!((v - 5.0 * (1.0 - (-1.0f64).exp())).abs() < 0.02, "{v}");

    let wf = run_tran(&Circuit::parse(examples::BUCK_SYNC.netlist).unwrap()).unwrap();
    let v = wf.get("v(out)").unwrap();
    let tail = &v[v.len() - 250..];
    let avg = tail.iter().sum::<f64>() / tail.len() as f64;
    assert!((avg - 5.0).abs() < 0.1, "buck: Vout = {avg}");

    let wf = run_tran(&Circuit::parse(examples::HALF_WAVE_RECTIFIER.netlist).unwrap()).unwrap();
    let v = wf.get("v(out)").unwrap();
    let max = v.iter().cloned().fold(0.0, f64::max);
    assert!((15.0..16.8).contains(&max), "пик выпрямителя {max}");
}

#[test]
fn ntc_divider_is_monotonic() {
    let r0 = examples::ntc_resistance(0.0);
    let r25 = examples::ntc_resistance(25.0);
    let r85 = examples::ntc_resistance(85.0);
    assert!((r25 - 10e3).abs() < 1e-9);
    assert!(r0 > r25 && r25 > r85);
    // Определение B25/85: ln(R25/R85) = B·(1/298.15 − 1/358.15).
    let b = (r25 / r85).ln() / (1.0 / 298.15 - 1.0 / 358.15);
    assert!((b - 3977.0).abs() < 1e-6);
    let mut prev = f64::INFINITY;
    for t in [-20.0, 0.0, 25.0, 50.0, 100.0] {
        let op = operating_point(&Circuit::parse(&examples::ntc_divider(t)).unwrap()).unwrap();
        let v = op.voltage("out").unwrap();
        assert!(
            v < prev,
            "напряжение делителя должно падать с ростом температуры"
        );
        prev = v;
    }
}
