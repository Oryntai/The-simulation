use sim_process::{
    Analysis, SourcedRange,
    schematic::{
        Endpoint, Parameter, ResetProfile, Scope,
        emi_reset::{
            EMI_RESET_CODE, EmiBand, EmiCriterion, EmiInput, EmiInterval, EmiOutcome,
            EmiResetObservation, EmiReturn, EmiSourceKind, EmiTransfer, assess_emi_reset,
            prepare_emi_reset,
        },
        sim_circuit::{
            Circuit, SimOptions,
            emc::loop_model::{LumpedDomain, SeriesRl},
            operating_point_with, parse, run_tran,
        },
    },
};
use sim_signal::Time;
use sim_world::Provenance;

// Синтетический стенд: 3 В покоя, порог 2 В, выдержка 8 мкс.
// Для 0,6 Ом, 1 мкГн, 2 А и dI/dt = 0 запас −0,2 В; для 0,4 Ом +0,2 В.
// Участок 11 мкс даёт −3 мкс запаса по выдержке при постоянном нарушении.
const US: u64 = 1_000; // Перевод мкс в нс.
const START: Time = Time(11 * US);
const END: Time = Time(22 * US);
const HIT: Time = Time(19 * US);

fn source() -> Provenance {
    Provenance::Derived("синтетический стенд, известная схема и форма тока".into())
}

fn parameter(nominal: f64) -> Parameter {
    Parameter {
        range: Some(SourcedRange {
            min: nominal,
            nominal,
            max: nominal,
            provenance: source(),
        }),
        measurement: "снять синхронный ряд на испытательной установке".into(),
    }
}

fn profile() -> ResetProfile {
    ResetProfile {
        falling_v: parameter(2.0),
        rising_v: parameter(2.5),
        assert_delay_s: parameter(Time(8 * US).as_secs()),
        release_delay_s: parameter(Time(3 * US).as_secs()),
    }
}

fn input() -> EmiInput {
    EmiInput {
        scope: Scope::Reference,
        source: "ветвь ключа".into(),
        source_kind: EmiSourceKind::Switch,
        board: "блок".into(),
        input: Endpoint {
            component: "блок".into(),
            pin: "сброс".into(),
        },
        reference: Endpoint {
            component: "блок".into(),
            pin: "опора".into(),
        },
        binding: Some(source()),
        start: START,
        end: END,
        bias_v: parameter(3.0),
        detector: Some(profile()),
        transfer: Some(EmiTransfer::SharedReturn(EmiReturn {
            geometry: "общий прямой участок возврата, остальной путь развязан".into(),
            // Геометрия и полоса синтетического участка. Допустимо
            // f <= min(2 МГц; 0,01 × 2e8 м/с / 0,1 м) = 2 МГц.
            domain: LumpedDomain {
                max_frequency_hz: 2e6,
                longest_path_m: 0.1,
                min_wave_speed_m_s: 2e8,
                max_wavelength_fraction: 0.01,
            },
            rl: SeriesRl {
                resistance_ohm: 0.6,
                inductance_h: 1e-6,
            },
            highest_frequency_hz: 1e6,
            model_source: Some(source()),
            remainder_v: parameter(0.0),
            intervals: vec![EmiInterval {
                end: END,
                value: (EmiBand::point(2.0), EmiBand::point(0.0)),
            }],
        })),
    }
}

fn observed(reset: Option<Time>) -> EmiResetObservation {
    EmiResetObservation {
        observed_until: END,
        complete: true,
        reset_at: reset,
        external_asserted_at: reset,
        baseline_reset_at: None,
    }
}

fn close(actual: f64, reference: f64) {
    // Допуск округления короткой цепочки операций f64 на стенде до 3 В.
    assert!(
        (actual - reference).abs() <= 4.0 * f64::EPSILON * 3.0,
        "{actual} ≠ {reference}"
    );
}

#[test]
fn общий_возврат_даёт_запас_по_обе_стороны_порога() {
    for (resistance, expected, reset) in [(0.6, -0.2, Some(HIT)), (0.4, 0.2, None)] {
        let mut case = input();
        let Some(EmiTransfer::SharedReturn(path)) = &mut case.transfer else {
            unreachable!()
        };
        path.rl.resistance_ohm = resistance;
        let Analysis::Analyzed { value: trial, .. } = prepare_emi_reset(case).unwrap() else {
            panic!("нет огибающей стенда");
        };
        let evidence = assess_emi_reset(&trial, Some(observed(reset)));
        assert_eq!(evidence.code, EMI_RESET_CODE);
        close(evidence.margins.voltage_v.nominal, expected);
        // Независимый MNA: источник тока поднимает потенциал локальной опоры.
        let net = parse(&format!(
            "Блок\nVbias in 0 3\nIpath 0 g 2\nRpath g mid {resistance}\nLpath mid 0 1u\n.end"
        ))
        .unwrap();
        // Все узлы уже имеют возврат. Убираем численный шунт MNA к земле.
        let dc = operating_point_with(
            &Circuit::from_netlist(&net).unwrap(),
            SimOptions {
                gshunt: 0.0,
                ..SimOptions::default()
            },
        )
        .unwrap();
        close(
            trial.voltage_at(START, true),
            dc.voltage("in").unwrap() - dc.voltage("g").unwrap(),
        );
        if let Some(at) = reset {
            assert_eq!(evidence.outcome, EmiOutcome::Fail { at });
            assert_eq!(evidence.margins.dwell_ns, [-3_000; 3]);
        } else {
            assert!(matches!(evidence.outcome, EmiOutcome::NotAnalyzed { .. }));
            assert_eq!(evidence.criterion, Some(EmiCriterion::WithinBench));
            assert_eq!(evidence.margins.dwell_ns, [8_000; 3]);
        }
    }
}

#[test]
fn пропуски_полоса_и_неконечные_входы() {
    for missing in 0..5 {
        let mut case = input();
        match missing {
            0 => case.detector = None,
            1 => case.binding = None,
            2 => case.transfer = None,
            3 => case.bias_v.range = None,
            _ => {
                let Some(EmiTransfer::SharedReturn(path)) = &mut case.transfer else {
                    unreachable!()
                };
                path.highest_frequency_hz = 2_000_001.0;
            }
        }
        assert!(matches!(
            prepare_emi_reset(case).unwrap(),
            Analysis::NotAnalyzed { .. }
        ));
    }
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut case = input();
        case.binding = None;
        let Some(EmiTransfer::SharedReturn(path)) = &mut case.transfer else {
            unreachable!()
        };
        path.intervals[0].value.1 = EmiBand::point(bad);
        assert!(prepare_emi_reset(case).is_err());
    }
}

#[test]
fn передний_фронт_неопределённость_и_отсутствие_события() {
    let mut case = input();
    let Some(EmiTransfer::SharedReturn(path)) = &mut case.transfer else {
        unreachable!()
    };
    // 0,6 Ом × 2 А = 1,2 В. Остаток ±0,3 В расширяет запас
    // −0,2 В до [−0,5; +0,1] В.
    path.remainder_v = parameter(0.3);
    let Analysis::Analyzed { value: trial, .. } = prepare_emi_reset(case).unwrap() else {
        panic!("нет огибающей");
    };
    let evidence = assess_emi_reset(&trial, Some(observed(Some(HIT))));
    close(evidence.margins.voltage_v.lower, -0.5);
    close(evidence.margins.voltage_v.upper, 0.1);
    assert!(matches!(evidence.outcome, EmiOutcome::NotAnalyzed { .. }));
    assert_eq!(evidence.criterion, None);
    assert_eq!(assess_emi_reset(&trial, None).criterion, None);
}

#[test]
fn индуктивный_фронт_сверен_отдельным_переходным_расчётом() {
    let mut case = input();
    case.start = Time::ZERO;
    case.end = Time(4 * US);
    let Some(EmiTransfer::SharedReturn(path)) = &mut case.transfer else {
        unreachable!()
    };
    // На 2 мкс линейного фронта I = 1 А, dI/dt = 300 кА/с.
    // R·I = 0,6 В, L·dI/dt = 0,3 В, разность клемм 2,1 В.
    path.intervals = vec![EmiInterval {
        end: case.end,
        value: (
            EmiBand {
                lower: 0.4,
                nominal: 1.0,
                upper: 1.6,
            },
            EmiBand::point(300_000.0),
        ),
    }];
    let Analysis::Analyzed { value: trial, .. } = prepare_emi_reset(case).unwrap() else {
        panic!("нет огибающей фронта");
    };
    let net = parse("Блок\nVbias in 0 3\nIpath 0 g PWL(0 0.4 4u 1.6)\nRpath g mid 0.6\nLpath mid 0 1u\n.tran 20n 4u\n.end").unwrap();
    let trace = run_tran(&Circuit::from_netlist(&net).unwrap()).unwrap();
    let at = Time(2 * US);
    let actual = trace.at("v(in)", at.as_secs()).unwrap() - trace.at("v(g)", at.as_secs()).unwrap();
    close(trial.voltage_at(at, true), 2.1);
    // Бюджет стенда 1 мкВ для численного интегрирования линейного фронта.
    assert!((actual - 2.1).abs() < 1e-6, "MNA = {actual}");
}

#[test]
fn выдержка_не_переносится_через_разрыв_и_проверяет_границу() {
    for (duration, violated) in [(7_999, false), (8_000, false), (8_001, true)] {
        let mut case = input();
        // Два низких участка разделяет один нс при высоком уровне. Допуск
        // требует низкого уровня в момент окончания выдержки.
        case.transfer = Some(EmiTransfer::Measured {
            intervals: vec![
                EmiInterval {
                    end: Time(START.0 + duration),
                    value: EmiBand::point(-1.2),
                },
                EmiInterval {
                    end: Time(START.0 + duration + 1),
                    value: EmiBand::point(0.0),
                },
                EmiInterval {
                    end: END,
                    value: EmiBand::point(-1.2),
                },
            ],
            source: Some(source()),
        });
        let Analysis::Analyzed { value: trial, .. } = prepare_emi_reset(case).unwrap() else {
            panic!("нет огибающей");
        };
        let evidence = assess_emi_reset(&trial, Some(observed(violated.then_some(HIT))));
        assert_eq!(evidence.margins.dwell_ns, [8_000 - duration as i128; 3]);
        assert_eq!(
            evidence.criterion,
            match duration.cmp(&(8 * US)) {
                std::cmp::Ordering::Less => Some(EmiCriterion::WithinBench),
                std::cmp::Ordering::Equal => None,
                std::cmp::Ordering::Greater => Some(EmiCriterion::Violated),
            }
        );
    }
}
