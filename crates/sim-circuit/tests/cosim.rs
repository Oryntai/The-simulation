//! Пошаговый API для совместного моделирования: H-мост с ШИМ 20 кГц, шаг 1 мкс, 1 с.

use std::time::Instant;

use sim_circuit::ngspice::{
    NgAnalysis, NgEvent, NgEventValue, find_ngspice, prepare_netlist_with_events,
};
use sim_circuit::transient::{TranOptions, run_tran_with};
use sim_circuit::{Circuit, SimError, SimOptions, Transient, examples, operating_point, parse};

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

#[test]
fn snapshot_repeats_reactive_history_and_pending_commands_bitwise() {
    // Синтетическая RLC-цепь с обратным диодом и конечным сопротивлением выключенного ключа.
    let circuit = Circuit::parse(
        "блок\nV1 p 0 EXT 4\nS1 p a EXT ON RON=1 ROFF=1meg\nL1 a b 1m\n\
         R1 b c 10\nC1 c 0 2u\nR2 c 0 100\nD1 0 a clamp\n.model clamp D(IS=1e-12 RS=0.1)\n",
    )
    .unwrap();
    let mut tr = Transient::new(&circuit, 2e-6).unwrap();
    tr.set_source_step("v1", 8.0).unwrap();
    tr.advance_to(67e-6).unwrap();
    assert!(tr.current("c1").unwrap().abs() > 1e-3);
    assert!(tr.current("l1").unwrap().abs() > 1e-3);
    tr.set_switch("s1", false).unwrap();
    tr.set_source("v1", -2.0).unwrap();
    let snapshot = tr.snapshot();
    let mut branch = tr.clone();
    branch.set_switch("s1", true).unwrap();

    let run = |tr: &mut Transient| {
        let mut trace = Vec::new();
        for k in 1..=47 {
            tr.advance_to(67e-6 + f64::from(k) * 2e-6).unwrap();
            let mut signals = Vec::new();
            tr.signal_values(&mut signals);
            signals.extend(
                circuit
                    .element_names()
                    .flat_map(|e| [tr.current(e).unwrap(), tr.power(e).unwrap()]),
            );
            signals.push(tr.time());
            trace.push((
                signals.into_iter().map(f64::to_bits).collect::<Vec<_>>(),
                tr.switch_state("s1").unwrap(),
                tr.stats(),
            ));
        }
        trace
    };
    let original = run(&mut tr);
    assert_ne!(original, run(&mut branch));
    assert!(!tr.switch_state("s1").unwrap());
    tr.restore(&snapshot);
    assert!(tr.switch_state("s1").unwrap());
    assert_eq!(original, run(&mut tr));
}

#[test]
fn exact_source_edge_and_accepted_currents() {
    // Синтетические 1 кОм, 1 мкФ и 2 мА: конечное напряжение 2 В, τ = 1 мс.
    let circuit = Circuit::parse("блок\nI1 0 n EXT 0\nR1 n 0 1k\nC1 n 0 1u\n").unwrap();
    let mut tr = Transient::new(&circuit, 10e-6).unwrap();
    let edge = 13.7e-6;
    tr.advance_to(edge).unwrap();
    assert_eq!(tr.time().to_bits(), edge.to_bits());
    tr.set_source_step("i1", 2e-3).unwrap();
    assert_eq!(tr.current("i1").unwrap(), 0.0);
    assert_eq!(tr.voltage("n").unwrap(), 0.0);
    tr.advance_to(edge).unwrap();
    // Интервал 1 фс меньше прежней отсечки в 10 фс, но содержит отдельное решение.
    let right = edge + 1e-15;
    tr.advance_to(right).unwrap();
    assert_eq!(tr.time().to_bits(), right.to_bits());
    assert_eq!(tr.terminal_currents("i1").unwrap(), vec![2e-3, -2e-3]);
    assert!(tr.voltage("n").unwrap() > 0.0);
    let end = edge + 1e-3;
    tr.advance_to(end).unwrap();
    let v = tr.voltage("n").unwrap();
    let exact = 2.0 * (1.0 - (-1.0_f64).exp());
    // 0,01% на сетке τ/100 относительно аналитической RC-кривой.
    assert!((v - exact).abs() < 1e-4 * exact, "{v} против {exact}");
    let kcl = tr.current("r1").unwrap() + tr.current("c1").unwrap() - tr.current("i1").unwrap();
    // Максимум 2 пА от штатного gshunt, запас на округление токов.
    assert!(kcl.abs() < 1e-11, "невязка KCL {kcl:e} А");
    tr.set_source("i1", -1e-3).unwrap();
    assert_eq!(tr.current("i1").unwrap(), 2e-3);
    assert_eq!(tr.power("i1").unwrap(), -2e-3 * v);

    // Скачок с последующей рампой: от −2 мА до −1 мА за следующую 1 мс.
    tr.set_source_step("i1", -2e-3).unwrap();
    tr.set_source("i1", -1e-3).unwrap();
    tr.advance_to(end + 1e-3).unwrap();
    // При t = τ линейный член a(t−τ) равен нулю, остаётся u0 + (v−u0+aτ)e⁻¹.
    let expected = -2.0 + (v + 3.0) * (-1.0_f64).exp();
    assert!((tr.voltage("n").unwrap() - expected).abs() < 1e-4);
    assert_eq!(tr.current("i1").unwrap(), -1e-3);
}

#[test]
fn inconsistent_switch_feedback_does_not_return_operating_point() {
    // При Roff узел выше порога 2 В, при Ron ниже него. Стационарного состояния нет.
    let ckt = Circuit::parse(
        "блок\nV1 p 0 5\nR1 p a 100\nS1 a 0 a 0 sm\n.model sm SW(RON=1 ROFF=1g VT=2)\n",
    )
    .unwrap();
    assert!(matches!(
        operating_point(&ckt),
        Err(SimError::NoConvergence { .. })
    ));
    assert!(Transient::new(&ckt, 1e-5).is_err());
}

#[test]
fn terminal_currents_follow_mna_polarity_and_closing_kcl() {
    // Токи выводов сверяются с токами отдельных идеальных источников и законами Кирхгофа.
    let ckt = Circuit::parse(
        "блок\nVd d 0 2\nVg g 0 3\nVs s 0 0\nVb b 0 0.4\n\
         M1 d g s b nm\nD1 b 0 dm\n.model nm NMOS(VTO=1 KP=1m IS=1n)\n.model dm D(IS=1n RS=10)\n",
    )
    .unwrap();
    let op = operating_point(&ckt).unwrap();
    let i = op.terminal_currents("M1").unwrap();
    assert_eq!(i.len(), 4);
    assert!(i[0] > 0.0 && i[2] < 0.0 && i[3] > 0.0, "{i:?}");
    assert_eq!(i[1], 0.0);
    assert_eq!(i[0], op.current("m1").unwrap());
    assert!(i.iter().sum::<f64>().abs() < 1e-15);
    let v = [2.0, 3.0, 0.0, 0.4];
    for ((j, branch), v) in i.iter().zip(["vd", "vg", "vs", "vb"]).zip(v) {
        let diode = if branch == "vb" {
            op.current("d1").unwrap()
        } else {
            0.0
        };
        // Численный шунт узла учитывается отдельно. Запас — округление LU.
        let residual = j + diode + op.current(branch).unwrap() + SimOptions::default().gshunt * v;
        assert!(residual.abs() < 1e-13, "{branch}: {residual:e} А");
    }
    assert_eq!(op.terminal_currents("d1").unwrap().len(), 2);
    assert!(op.terminal_currents("неизвестный").is_err());
}

#[test]
fn nonfinite_and_failed_advance_keep_last_accepted_state() {
    let ckt =
        Circuit::parse("блок\nV1 a 0 EXT -2\nS1 a b EXT RON=1 ROFF=1meg\nR1 b 0 1k\n").unwrap();
    let mut tr = Transient::new(&ckt, 1e-6).unwrap();
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(tr.set_source("v1", value).is_err());
        assert!(tr.set_source_step("v1", value).is_err());
        assert!(tr.advance_to(value).is_err());
        let opts = TranOptions {
            max_step: Some(value),
            ..TranOptions::default()
        };
        assert!(Transient::with_options(&ckt, 1e-6, opts).is_err());
        let opts = TranOptions {
            sim: SimOptions {
                gshunt: value,
                ..SimOptions::default()
            },
            ..TranOptions::default()
        };
        assert!(Transient::with_options(&ckt, 1e-6, opts).is_err());
        let spec = sim_circuit::netlist::TranSpec {
            tstep: 1e-6,
            tstop: value,
            tstart: 0.0,
            tmax: None,
            uic: false,
        };
        assert!(run_tran_with(&ckt, &spec, SimOptions::default()).is_err());
    }
    assert!(tr.advance_to(-1.0).is_err());
    let net = Circuit::parse("блок\nI1 a 0 EXT 1e999\nR1 a 0 1k\n").unwrap();
    assert!(Transient::new(&net, 1e-6).is_err());
    assert!(operating_point(&net).is_err());
    let before = (
        tr.time(),
        tr.stats(),
        tr.current("v1").unwrap(),
        tr.voltage("b").unwrap(),
    );
    tr.set_switch("s1", true).unwrap();
    // Первый подшаг проходит, последний превышает предел MNA для конечного решения (1e30).
    tr.set_source("v1", 1.5e30).unwrap();
    assert!(tr.advance_to(2e-6).is_err());
    assert!(!tr.switch_state("s1").unwrap());
    assert_eq!(
        before,
        (
            tr.time(),
            tr.stats(),
            tr.current("v1").unwrap(),
            tr.voltage("b").unwrap()
        )
    );
    tr.set_source("v1", 2.0).unwrap();
    tr.advance_to(2e-6).unwrap();
    assert!(tr.switch_state("s1").unwrap());
    assert_eq!(tr.voltage("a").unwrap(), 2.0);
}

#[test]
fn ngspice_preserves_events_dc_and_current_directions() {
    let ng = find_ngspice().expect("для проверки событий нужен ngspice");
    // Синтетический делитель 1 кОм / 1 кОм и 1 мкФ: цель 2 В, τ = 0,5 мс.
    let netlist = parse(
        "блок\nV1 p 0 EXT -2\nS1 p n EXT OFF RON=10 ROFF=1g\nR1 n q 990\n\
         R2 q 0 1k\nC1 q 0 1u\nI1 0 load EXT 1m\nR3 load 0 1k\nV2 aux 0 EXT -1\n",
    )
    .unwrap();
    let ckt = Circuit::from_netlist(&netlist).unwrap();
    // Фронты намеренно не привязаны к сетке в 13 мкс.
    let on = 37e-6;
    let off = 3.371e-3;
    let stop = 6e-3;
    let event = |time, element, value| NgEvent {
        time,
        element,
        value,
    };
    let events = [
        event(on, "I1", NgEventValue::SourceStep(-2e-3)),
        event(on, "S1", NgEventValue::Switch(true)),
        event(on, "V1", NgEventValue::SourceStep(4.0)),
        event(on, "V2", NgEventValue::SourceRamp(2.0)),
        event(off, "V1", NgEventValue::SourceStep(-2.0)),
        event(off, "S1", NgEventValue::Switch(false)),
    ];
    let deck = prepare_netlist_with_events(&netlist, &events).unwrap();
    let current_v = deck.source_current("v1").unwrap();
    let current_i = deck.source_current("i1").unwrap();
    let signals = ["v(q)", "v(p)", "v(load)", "v(aux)", current_v, current_i];
    let op = operating_point(&ckt).unwrap();
    let dc = deck.run(&ng, NgAnalysis::Op, &signals).unwrap();
    let dc_values = [
        op.voltage("q").unwrap(),
        op.voltage("p").unwrap(),
        op.voltage("load").unwrap(),
        op.voltage("aux").unwrap(),
        op.current("v1").unwrap(),
        op.current("i1").unwrap(),
    ];
    let dc_max = dc.rows[0][..4]
        .iter()
        .zip(&dc_values)
        .map(|(a, b)| (a - b).abs())
        .fold(0.0, f64::max);
    for (&reference, value) in dc.rows[0][..4].iter().zip(dc_values) {
        assert!((value - reference).abs() <= 0.01_f64.max(0.01 * reference.abs()));
    }
    assert!(dc.rows[0][4] > 0.0 && dc.rows[0][5] > 0.0);
    let analysis = NgAnalysis::Tran {
        tstep: 1e-6,
        tmax: 1e-6,
        tstop: stop,
        tstart: 0.0,
        uic: false,
    };
    let reference = deck.run(&ng, analysis, &signals).unwrap();
    let t = reference.column("time").unwrap();
    let q = reference.column("v(q)").unwrap();
    let p = reference.column("v(p)").unwrap();
    let i_v = reference.column(current_v).unwrap();
    let i_i = reference.column(current_i).unwrap();
    let i_on = t.partition_point(|&x| x < on);
    let i_off = t.partition_point(|&x| x < off);
    assert!((t[i_on] - on).abs() <= 2.0 * f64::EPSILON * on);
    assert!((t[i_off] - off).abs() <= 2.0 * f64::EPSILON * off);
    assert_eq!(
        (p[i_on], p[i_on + 1], p[i_off], p[i_off + 1]),
        (-2.0, 4.0, 4.0, -2.0)
    );
    assert!(i_v[i_on] > 0.0 && i_v[i_on + 1] < 0.0 && i_v[i_off + 1] > 0.0);
    assert!((i_i[i_on] - 1e-3).abs() < 1e-12 && (i_i[i_on + 1] + 2e-3).abs() < 1e-12);
    let aux = reference.column("v(aux)").unwrap();
    let k = i_on / 2;
    assert!(t[k] > 0.0 && t[k] < on);
    assert!((aux[k] - (-1.0 + 3.0 * t[k] / on)).abs() < 1e-10);

    // Окно включения [on, off], полоса ±2% вокруг 2 В. None означает отсутствие установления.
    let settle = |t: &[f64], v: &[f64]| {
        let last_outside = v.iter().rposition(|&x| (x - 2.0).abs() > 0.04)?;
        let next = last_outside + 1;
        (next < v.len()).then(|| {
            let frac = ((2.0 - 0.04) - v[last_outside]) / (v[next] - v[last_outside]);
            t[last_outside] + (t[next] - t[last_outside]) * frac - on
        })
    };
    let peak = |v: &[f64]| v.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let reference_peak = peak(&q[i_on..=i_off]);
    let reference_settle = settle(&t[i_on..=i_off], &q[i_on..=i_off]).unwrap();
    let analytical = 0.5e-3 * ((2.0 - dc_values[0]) / 0.04).ln();
    assert!((reference_settle - analytical).abs() < 0.01 * analytical);

    let evaluate = |dt: f64| {
        let mut tr = Transient::new(&ckt, dt).unwrap();
        tr.set_source("v2", 2.0).unwrap();
        tr.advance_to(on).unwrap();
        assert_eq!(tr.time().to_bits(), on.to_bits());
        assert_eq!(tr.voltage("p").unwrap(), -2.0);
        tr.set_source_step("v1", 4.0).unwrap();
        tr.set_source_step("i1", -2e-3).unwrap();
        tr.set_switch("s1", true).unwrap();
        let mut time = vec![on];
        let mut v = vec![tr.voltage("q").unwrap()];
        while tr.time() < off {
            tr.advance_to((tr.time() + dt).min(off)).unwrap();
            time.push(tr.time());
            v.push(tr.voltage("q").unwrap());
            assert!(tr.current("v1").unwrap() < 0.0 && tr.current("i1").unwrap() < 0.0);
        }
        tr.set_source_step("v1", -2.0).unwrap();
        tr.set_switch("s1", false).unwrap();
        tr.advance_to(stop).unwrap();
        assert!(tr.current("v1").unwrap() > 0.0);
        (peak(&v), settle(&time, &v).unwrap())
    };
    let (peak_coarse, settle_coarse) = evaluate(13e-6);
    let (peak_fine, settle_fine) = evaluate(6.5e-6);
    for (p, s) in [(peak_coarse, settle_coarse), (peak_fine, settle_fine)] {
        assert!((p - reference_peak).abs() < 0.05 * reference_peak);
        assert!((s - reference_settle).abs() < 0.05 * reference_settle);
    }
    assert!((peak_fine - peak_coarse).abs() < 0.01 * reference_peak);
    assert!((settle_fine - settle_coarse).abs() < 0.01 * reference_settle);
    println!(
        "DC |ΔV|max={dc_max:.6e} В; пик v(q) MNA/ngspice {peak_coarse:.9}/{reference_peak:.9} В; установление MNA/ngspice/RC {settle_coarse:.9e}/{reference_settle:.9e}/{analytical:.9e} с; фронты {on:e}/{off:e} с"
    );
    assert!(deck.run(&ng, analysis, &["v(неизвестный)"]).is_err());
    let invalid = [
        event(f64::NAN, "V1", NgEventValue::SourceStep(0.0)),
        event(on, "V1", NgEventValue::SourceStep(f64::INFINITY)),
        event(on, "S1", NgEventValue::SourceStep(0.0)),
        event(on, "R2", NgEventValue::Switch(true)),
        event(on, "нет", NgEventValue::Switch(true)),
    ];
    for e in invalid {
        assert!(prepare_netlist_with_events(&netlist, &[e]).is_err());
    }
    assert!(prepare_netlist_with_events(&netlist, &[events[0], events[0]]).is_err());
    let short = NgAnalysis::Tran {
        tstep: 1e-6,
        tmax: 1e-6,
        tstop: 2e-3,
        tstart: 0.0,
        uic: false,
    };
    assert!(deck.run(&ng, short, &signals).is_err());
}

#[test]
fn ngspice_preserves_signed_reactive_initial_conditions() {
    let ng = find_ngspice().expect("для проверки IC нужен ngspice");
    // Знак IC на C1 задан от земли к c. τC = 1 мс, τL = 0,1 мс.
    let netlist = parse(
        "блок\nC1 0 c 1u IC=0.5\nR1 c 0 1k\nL1 a 0 1m IC=0.2\nR2 a 0 10\n\
         .tran 1u 200u 0 1u uic\n",
    )
    .unwrap();
    let deck = prepare_netlist_with_events(&netlist, &[]).unwrap();
    let stop = 200e-6;
    let analysis = NgAnalysis::Tran {
        tstep: 1e-6,
        tstop: stop,
        tstart: 0.0,
        tmax: 1e-6,
        uic: true,
    };
    let signals = ["v(c)", "v(a)", "i(l1)"];
    let table = deck.run(&ng, analysis, &signals).unwrap();
    let ckt = Circuit::from_netlist(&netlist).unwrap();
    let mut tr = Transient::new(&ckt, 1e-6).unwrap();
    assert!(tr.voltage("c").unwrap() < 0.0 && tr.current("l1").unwrap() > 0.0);
    tr.advance_to(stop).unwrap();
    let exact = [
        -0.5 * (-0.2_f64).exp(),
        -2.0 * (-2.0_f64).exp(),
        0.2 * (-2.0_f64).exp(),
    ];
    let actual = [
        tr.voltage("c").unwrap(),
        tr.voltage("a").unwrap(),
        tr.current("l1").unwrap(),
    ];
    for ((actual, &reference), exact) in actual
        .into_iter()
        .zip(&table.rows.last().unwrap()[1..])
        .zip(exact)
    {
        // При сетке τL/100 допускаем 0,1% у обоих численных решателей.
        assert!(
            (actual - exact).abs() < 0.001 * exact.abs(),
            "MNA {actual:e}, ngspice {reference:e}, аналитика {exact:e}"
        );
        assert!(
            (reference - exact).abs() < 0.001 * exact.abs(),
            "MNA {actual:e}, ngspice {reference:e}, аналитика {exact:e}"
        );
    }
    println!(
        "UIC при {stop:e} с: v(c) MNA/ngspice {:.9}/{:.9} В; i(L1) {:.9}/{:.9} А",
        actual[0],
        table.rows.last().unwrap()[1],
        actual[2],
        table.rows.last().unwrap()[3]
    );
}
