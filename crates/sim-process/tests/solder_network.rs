//! Синтетические стенды. Числа заданы уравнениями теплоёмкостей и балансом энергии.

use sim_process::work::heat::solder_network::{
    SolderContact, SolderNetwork, SolderNetworkInput, SolderNetworkSpec, SolderNetworkStep,
    SolderNodes,
};
use sim_process::{Analysis, Interval, ProcessError, SourcedRange, StepModel};
use sim_signal::{Rng, Time};
use sim_world::Provenance;

fn point(x: f64) -> Option<SourcedRange> {
    Some(SourcedRange {
        min: x,
        nominal: x,
        max: x,
        provenance: Provenance::Paper("аналитические стенды теста тепловой сети".into()),
    })
}

fn nodes<T>([tip, pad, lead, copper]: [T; 4]) -> SolderNodes<T> {
    SolderNodes {
        tip,
        pad,
        lead,
        copper,
    }
}

fn interval(start: f64, end: f64) -> Interval {
    Interval {
        start: Time::from_secs(start).unwrap(),
        end: Time::from_secs(end).unwrap(),
    }
}

fn run(
    network: &mut SolderNetwork,
    start: f64,
    end: f64,
    input: &SolderNetworkInput,
) -> Result<Analysis<SolderNetworkStep>, ProcessError> {
    network.advance(
        interval(start, end),
        input,
        &mut Rng::stream(0, "тепловая сеть"),
    )
}

fn analyzed(result: Result<Analysis<SolderNetworkStep>, ProcessError>) -> SolderNetworkStep {
    let Analysis::Analyzed { value, .. } = result.unwrap() else {
        panic!("ожидался численный расчёт стенда");
    };
    value
}

fn unknown(result: Result<Analysis<SolderNetworkStep>, ProcessError>) {
    assert!(matches!(result, Ok(Analysis::NotAnalyzed { reason }) if !reason.is_empty()));
}

fn close(actual: f64, expected: f64) {
    // Бюджет округления для малых стендов. 4096 ε оставляет запас на матричные
    // произведения и вычитание абсолютных температур при проверке баланса.
    assert!(
        (actual - expected).abs() <= 4096.0 * f64::EPSILON * expected.abs().max(1.0),
        "{actual:.16e} вместо {expected:.16e}"
    );
}

fn two_nodes(max_power_w: f64) -> SolderNetworkSpec {
    // Две единичные ёмкости связаны проводимостью 1 Вт/К, остальные узлы изолированы.
    SolderNetworkSpec {
        capacity_j_k: nodes([1.0; 4].map(point)),
        initial_k: nodes([304.0, 300.0, 300.0, 300.0].map(point)),
        pad_lead_w_k: point(0.0),
        pad_copper_w_k: point(0.0),
        copper_polygon_w_k: point(0.0),
        polygon_k: point(300.0),
        max_power_w: point(max_power_w),
    }
}

fn touching(conductance_w_k: f64, requested_power_w: f64) -> SolderNetworkInput {
    SolderNetworkInput {
        contact: Some(SolderContact::Touching {
            conductance_w_k: point(conductance_w_k),
        }),
        requested_power_w: point(requested_power_w),
    }
}

#[test]
fn power_limit_changes_the_side_of_the_reference_threshold() {
    // При Cж = Cп = G = 1 разность Tж − Tп = P/2 + (4 − P/2)e^(−2t).
    // Среднее = 302 + Pt/2. За 0,5 с запас площадки до порога стенда 301,5 К
    // равен −0,051819161757 К при лимите 2 Вт и +0,132120558829 К при 4 Вт.
    let threshold_k = 301.5;
    let input = touching(1.0, 8.0);
    for (limit, expected_margin) in [(2.0, -0.051819161757), (4.0, 0.132120558829)] {
        let mut network = SolderNetwork::new(two_nodes(limit)).unwrap();
        let step = analyzed(run(&mut network, 0.0, 0.5, &input));
        let margin = step.end_k.pad - threshold_k;
        close(margin, expected_margin);
        assert_eq!(
            margin.is_sign_positive(),
            expected_margin.is_sign_positive()
        );
        let difference = limit / 2.0 + (4.0 - limit / 2.0) * (-1.0_f64).exp();
        close(step.end_k.tip - step.end_k.pad, difference);
        close(step.applied_power_w, limit);
        close(step.supplied_j, limit / 2.0);
        close(step.end_k.tip + step.end_k.pad - 604.0, step.supplied_j);
        assert_eq!(step.start_k, nodes([304.0, 300.0, 300.0, 300.0]));
        close(step.end_k.lead, 300.0);
        close(step.end_k.copper, 300.0);
        assert_eq!(step.to_polygon_j, 0.0);
    }
}

#[test]
fn four_active_nodes_and_polygon_match_an_eigenmode_and_first_law() {
    // Стенд: C = 1 / 2 / 3 / 4 Дж/К, G = 4 / 12 / 11 / 26 Вт/К
    // по ветвям жало–площадка, площадка–вывод, площадка–медь, медь–полигон.
    // При 26 Вт стационарные превышения над 300 К идут от стока через P/G.
    // Для v = [4, 3, 4, 1] К подстановка в четыре баланса даёт dv/dt = −v.
    let capacity_j_k = [1.0, 2.0, 3.0, 4.0];
    let steady = [217.0 / 22.0, 37.0 / 11.0, 37.0 / 11.0, 1.0];
    let mode = [4.0, 3.0, 4.0, 1.0];
    let initial: [f64; 4] = std::array::from_fn(|i| 300.0 + steady[i] + mode[i]);
    let mut spec = two_nodes(26.0);
    spec.capacity_j_k = nodes(capacity_j_k.map(point));
    spec.initial_k = nodes(initial.map(point));
    spec.pad_lead_w_k = point(12.0);
    spec.pad_copper_w_k = point(11.0);
    spec.copper_polygon_w_k = point(26.0);
    let input = touching(4.0, 52.0);
    let model = SolderNetwork::new(spec).unwrap();
    let mut partitioned = model.clone();
    let mut previous = 0.0;
    let mut removed_j = 0.0;
    let mut max_relative_error: f64 = 0.0;
    let mut max_balance_j: f64 = 0.0;
    // Пробы быстрого перехода, режима порядка постоянной времени и установления.
    for t in [0.125_f64, 0.5, 1.0, 8.0, 10_000.0] {
        let whole = analyzed(run(&mut model.clone(), 0.0, t, &input));
        let split = analyzed(run(&mut partitioned, previous, t, &input));
        assert_eq!(whole, analyzed(run(&mut model.clone(), 0.0, t, &input)));
        let [tip, pad, lead, copper] = [
            whole.end_k.tip,
            whole.end_k.pad,
            whole.end_k.lead,
            whole.end_k.copper,
        ];
        let actual = [tip, pad, lead, copper];
        for i in 0..4 {
            let expected_rise_k = steady[i] + mode[i] * (-t).exp();
            let error = (actual[i] - 300.0 - expected_rise_k).abs() / expected_rise_k;
            max_relative_error = max_relative_error.max(error);
            // Допуск 5% по приросту температуры задан в приёмке Т5.
            assert!(error <= 0.05);
            close(actual[i] - 300.0, expected_rise_k);
        }
        close(split.end_k.tip, tip);
        close(split.end_k.pad, pad);
        close(split.end_k.lead, lead);
        close(split.end_k.copper, copper);
        close(whole.supplied_j, 26.0 * t);
        close(whole.applied_power_w, 26.0);
        // q = 26(1 + e^−t) Вт по температуре меди. Отдельная сверка по изменению ΣC·T.
        close(whole.to_polygon_j, 26.0 * (t - (-t).exp_m1()));
        let stored_j = (0..4)
            .map(|i| capacity_j_k[i] * (actual[i] - initial[i]))
            .sum::<f64>();
        let balance_j = whole.supplied_j - whole.to_polygon_j - stored_j;
        max_balance_j = max_balance_j.max(balance_j.abs());
        close(whole.to_polygon_j + stored_j, whole.supplied_j);
        removed_j += split.to_polygon_j;
        close(removed_j, whole.to_polygon_j);
        previous = t;
    }
    println!(
        "Сверка четырёх узлов: относительная ошибка {max_relative_error:.3e}, невязка {max_balance_j:.3e} Дж"
    );
}

#[test]
fn separation_opens_the_contact_and_polygon_can_supply_heat() {
    // Cм = 1 Дж/К, Gмп = 2 Вт/К, полигон 302 К. Медь прогревается от 300 К
    // по Tм = 302 − 2e^(−2t). Разомкнутое жало получает запрошенные 2 Вт.
    let mut spec = two_nodes(4.0);
    spec.polygon_k = point(302.0);
    spec.copper_polygon_w_k = point(2.0);
    let mut network = SolderNetwork::new(spec).unwrap();
    let input = SolderNetworkInput {
        contact: Some(SolderContact::Separated),
        requested_power_w: point(2.0),
    };
    let step = analyzed(run(&mut network, 0.0, 0.5, &input));
    let copper_rise = -2.0 * (-1.0_f64).exp_m1();
    close(step.end_k.copper - 300.0, copper_rise);
    close(step.to_polygon_j, -copper_rise);
    close(step.end_k.tip, 305.0);
    close(step.applied_power_w, 2.0);
    close(step.supplied_j, 1.0);
    close(step.end_k.pad, 300.0);
    close(step.end_k.lead, 300.0);
    // При повторном касании двух единичных ёмкостей их среднее равно 302,5 К,
    // разность после следующих 0,5 с без нагрева равна 5e^(−1) К.
    let rejoined = analyzed(run(&mut network, 0.5, 1.0, &touching(1.0, 0.0)));
    assert_eq!(rejoined.start_k, step.end_k);
    close(rejoined.end_k.tip, 302.5 + 2.5 * (-1.0_f64).exp());
    close(rejoined.end_k.pad, 302.5 - 2.5 * (-1.0_f64).exp());
    close(rejoined.end_k.copper, 302.0 - 2.0 * (-2.0_f64).exp());
    close(
        rejoined.to_polygon_j,
        step.end_k.copper - rejoined.end_k.copper,
    );
}

#[test]
fn missing_contact_power_source_or_envelope_leaves_analysis_open() {
    let spec = two_nodes(4.0);
    let good = touching(1.0, 8.0);
    for bad in [
        SolderNetworkInput {
            contact: None,
            ..good.clone()
        },
        SolderNetworkInput {
            contact: Some(SolderContact::Touching {
                conductance_w_k: None,
            }),
            ..good.clone()
        },
        SolderNetworkInput {
            requested_power_w: None,
            ..good.clone()
        },
    ] {
        let mut network = SolderNetwork::new(spec.clone()).unwrap();
        unknown(run(&mut network, 0.0, 0.5, &bad));
        unknown(run(&mut network, 0.5, 1.0, &good));
    }
    let mut no_capacity = spec.clone();
    no_capacity.capacity_j_k.lead = None;
    let mut estimate = spec.clone();
    estimate.max_power_w.as_mut().unwrap().provenance = Provenance::Estimate("оценка".into());
    let mut blank_source = spec.clone();
    blank_source.max_power_w.as_mut().unwrap().provenance = Provenance::Paper(String::new());
    let mut open_envelope = spec;
    open_envelope.max_power_w.as_mut().unwrap().min = 2.0;
    for incomplete in [no_capacity, estimate, blank_source, open_envelope] {
        let mut network = SolderNetwork::new(incomplete).unwrap();
        unknown(run(&mut network, 0.0, 0.5, &good));
    }
}

#[test]
fn invalid_inputs_are_rejected_even_with_missing_data_and_errors_are_atomic() {
    let good = touching(1.0, 8.0);
    let spec = two_nodes(4.0);
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1.0] {
        let mut broken = spec.clone();
        broken.polygon_k = None;
        broken.capacity_j_k.tip = point(invalid);
        assert!(matches!(
            SolderNetwork::new(broken),
            Err(ProcessError::InvalidInput(_))
        ));
        let mut network = SolderNetwork::new(spec.clone()).unwrap();
        let bad = SolderNetworkInput {
            contact: None,
            requested_power_w: point(invalid),
        };
        assert!(matches!(
            run(&mut network, 0.0, 0.5, &bad),
            Err(ProcessError::InvalidInput(_))
        ));
        let bad = SolderNetworkInput {
            requested_power_w: None,
            ..touching(invalid, 8.0)
        };
        assert!(matches!(
            run(&mut network, 0.0, 0.5, &bad),
            Err(ProcessError::InvalidInput(_))
        ));
        let first = analyzed(run(&mut network, 0.0, 0.5, &good));
        assert!(matches!(
            run(&mut network, 0.75, 1.0, &good),
            Err(ProcessError::InvalidInput(_))
        ));
        assert!(matches!(
            run(&mut network, 0.5, 0.5, &good),
            Err(ProcessError::InvalidInput(_))
        ));
        assert_eq!(
            analyzed(run(&mut network, 0.5, 1.0, &good)).start_k,
            first.end_k
        );
    }
    let mut network = SolderNetwork::new(two_nodes(f64::MAX)).unwrap();
    assert!(matches!(
        run(&mut network, 0.0, 2.0, &touching(1.0, f64::MAX)),
        Err(ProcessError::Execution(_))
    ));
    close(
        analyzed(run(&mut network, 0.0, 0.5, &good)).applied_power_w,
        8.0,
    );
}
