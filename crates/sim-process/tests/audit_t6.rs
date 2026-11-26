#[allow(dead_code)]
#[path = "../src/schematic/esd_event.rs"]
mod esd_event;

use esd_event::*;
use sim_process::{
    Analysis, ProcessError, SourcedRange, schematic,
    schematic::{
        Catalog, CompiledSchematic, Component, Contact, ContactKind, Endpoint, Parameter, Part,
        PartKind, Pin, PinRole, ResetProfile, Schematic, Scope, Status,
        emi_reset::{
            EmiBand, EmiCriterion, EmiInput, EmiInterval, EmiOutcome, EmiResetObservation,
            EmiSourceKind, EmiTransfer, assess_emi_reset, prepare_emi_reset,
        },
    },
};
use sim_signal::Time;
use sim_world::Provenance;

fn point(value: f64, source: &Provenance) -> SourcedRange {
    SourcedRange {
        min: value,
        nominal: value,
        max: value,
        provenance: source.clone(),
    }
}

fn endpoint(pin: &str) -> Endpoint {
    Endpoint {
        component: "блок".into(),
        pin: pin.into(),
    }
}

fn block() -> CompiledSchematic {
    let pins = [
        ("exposed", ContactKind::Terminal, PinRole::Passive),
        (
            "input",
            ContactKind::Pad,
            PinRole::SignalIn("return".into()),
        ),
        ("return", ContactKind::Pad, PinRole::Return),
    ];
    let catalog = Catalog::new(
        [(
            "block".into(),
            Part {
                kind: PartKind::Other,
                source: "Три клеммы синтетического блока Т6".into(),
                reference_only: true,
                model_gap: Some("Импульсный эквивалент задан стендом".into()),
                pins: pins
                    .map(|(name, kind, role)| {
                        (
                            name.into(),
                            Pin {
                                contact: Contact {
                                    kind,
                                    family: None,
                                    key: None,
                                },
                                role,
                            },
                        )
                    })
                    .into(),
                internal_nodes: Default::default(),
                elements: Default::default(),
                ratings: Default::default(),
            },
        )]
        .into(),
    )
    .unwrap();
    let names = ["exposed", "input", "return"];
    schematic::compile(
        Schematic {
            name: "Блок".into(),
            scope: Scope::Reference,
            nets: names.map(str::to_owned).into(),
            components: [(
                "блок".into(),
                Component {
                    part: "block".into(),
                    nets: names.map(|pin| (pin.into(), pin.into())).into(),
                },
            )]
            .into(),
            connections: Default::default(),
            rails: Default::default(),
            bindings: Default::default(),
        },
        &catalog,
    )
    .unwrap()
}

#[test]
fn esd_требует_паспорт_пределы_и_сосредоточенный_стенд() {
    // Т6.6, IEC 61000-4-2:2008, таблица 3: уровень 2 кВ, пик 7,5 А,
    // фронт 10–90% 0,8 нс, отсчёты 4 и 2 А через 30 и 60 нс от t10.
    // Для Rпути=5 Ом, V₀=6,5 В и R_TVS=0,5 Ом: из KCL
    // Iвывода = (7,5 + 6,5/0,5)/(1 + 5/0,5) = 41/22 А.
    let samples = [
        [0.0, 0.0],
        [1.0, 7.5],
        [5.0, 2.0],
        [30.1, 4.0],
        [60.1, 2.0],
        [100.0, 0.0],
    ]
    .map(|[ns, current_abs_a]| EsdSample {
        offset_s: ns * 1e-9,
        current_abs_a,
    });
    let source = Provenance::Derived("Импульс и параметры резистивного стенда Т6.6".into());
    let port = EsdPort {
        exposed: endpoint("exposed"),
        protected: endpoint("input"),
        return_: endpoint("return"),
    };
    let limits = EsdPinLimits {
        voltage_abs_v: point(4.0, &source),
        current_abs_a: point(3.0, &source),
    };
    let tvs = EsdTvs {
        polarity: EsdPolarity::Positive,
        knee_v: 6.5,
        dynamic_ohm: 0.5,
        max_injected_peak_a: 8.0,
        min_rise_s: 0.6e-9,
        max_pulse_s: 100.0 * 1e-9,
        datasheet: Some(&source),
    };
    let domain = EsdDomain {
        bandwidth_hz: 2e9,
        length_m: 0.001,
        wave_speed_m_s: 2e8,
        max_length_wavelengths: 0.1,
        max_terminal_abs_v: 40.0,
        voltage_error_v: 0.0,
        current_error_a: 0.0,
        source: Some(&source),
    };
    let network = EsdNetwork {
        series_ohm: 3.0,
        pin_ohm: 2.0,
        domain,
        protection: Some(EsdProtection::Tvs(tvs)),
    };
    let input = EsdInput {
        port: &port,
        start: Time(100), // Метка контакта из стенда Т6.6.
        exposure: Some(&source),
        pulse: Some(EsdPulse {
            level: EsdLevel::Kv2,
            polarity: EsdPolarity::Positive,
            samples: &samples,
            bandwidth_hz: domain.bandwidth_hz,
            source: Some(&source),
        }),
        network: Some(network),
        limits: Some(&limits),
    };
    let scheme = block();
    let assessed = assess_esd_event(&scheme, &input).unwrap();
    assert_eq!(assessed.status, Status::NotAnalyzed);
    assert_eq!(assessed.evidence.criterion, Some(EsdCriterion::WithinBench));
    let Analysis::Analyzed { value, .. } = assessed.evidence.response else {
        panic!("отсутствует контрольный ответ резистивного блока");
    };
    assert!((value.peak.pin_a - 41.0 / 22.0).abs() <= 8.0 * f64::EPSILON);
    let bare = EsdNetwork {
        protection: Some(EsdProtection::Absent(&source)),
        ..network
    };
    assert_eq!(
        assess_esd_event(
            &scheme,
            &EsdInput {
                network: Some(bare),
                ..input
            }
        )
        .unwrap()
        .status,
        Status::Fail
    );
    let no_sheet = EsdNetwork {
        protection: Some(EsdProtection::Tvs(EsdTvs {
            datasheet: None,
            ..tvs
        })),
        ..network
    };
    // При λ=0,1 м ошибочно заявленная граница λ разрешала длинный путь λ/2.
    // Завышение скорости до 2c также не может служить доказательством применимости.
    let half_wave = EsdDomain {
        length_m: 0.05,
        max_length_wavelengths: 1.0,
        ..domain
    };
    let superluminal = EsdDomain {
        wave_speed_m_s: 2.0 * 299_792_458.0,
        ..domain
    };
    let short_band = EsdDomain {
        bandwidth_hz: domain.bandwidth_hz.next_down(),
        ..domain
    };
    for candidate in [
        EsdInput {
            limits: None,
            ..input
        },
        EsdInput {
            exposure: None,
            ..input
        },
        EsdInput {
            network: Some(no_sheet),
            ..input
        },
        EsdInput {
            network: Some(EsdNetwork {
                protection: None,
                ..network
            }),
            ..input
        },
        EsdInput {
            network: Some(EsdNetwork {
                domain: short_band,
                ..network
            }),
            ..input
        },
        EsdInput {
            network: Some(EsdNetwork {
                domain: half_wave,
                ..network
            }),
            ..input
        },
        EsdInput {
            network: Some(EsdNetwork {
                domain: superluminal,
                ..network
            }),
            ..input
        },
    ] {
        let got = assess_esd_event(&scheme, &candidate).unwrap();
        assert_eq!(got.status, Status::NotAnalyzed, "{got:?}");
        assert_eq!(got.evidence.criterion, None, "{got:?}");
        assert!(matches!(got.evidence.margins, Analysis::NotAnalyzed { .. }));
        if candidate.limits.is_some() {
            assert!(matches!(
                got.evidence.response,
                Analysis::NotAnalyzed { .. }
            ));
        }
    }
    let invalid = EsdDomain {
        length_m: f64::NAN,
        ..domain
    };
    assert!(
        assess_esd_event(
            &scheme,
            &EsdInput {
                network: Some(EsdNetwork {
                    domain: invalid,
                    ..network
                }),
                exposure: None,
                ..input
            }
        )
        .is_err()
    );
}

#[test]
fn emi_порог_и_выдержка_не_заменяют_событие_платы() {
    // Постановка Т6.7: 3 В покоя, пороги 2/2,5 В, 8/3 мкс,
    // 1,8 В на [11,22) мкс. Минимальный запас −0,2 В,
    // расчётный срок истекает в 19 мкс, на 3 мкс раньше конца плато.
    let source = Provenance::Derived("Огибающая и детектор стенда Т6.7".into());
    let param = |value| Parameter {
        range: Some(point(value, &source)),
        measurement: "стендовый ряд".into(),
    };
    let at = Time(19_000);
    let end = Time(22_000);
    let input = EmiInput {
        scope: Scope::Reference,
        source: "ключ".into(),
        source_kind: EmiSourceKind::Switch,
        board: "блок".into(),
        input: endpoint("input"),
        reference: endpoint("return"),
        binding: Some(source.clone()),
        start: Time(11_000),
        end,
        bias_v: param(3.0),
        detector: Some(ResetProfile {
            falling_v: param(2.0),
            rising_v: param(2.5),
            assert_delay_s: param(Time(8_000).as_secs()),
            release_delay_s: param(Time(3_000).as_secs()),
        }),
        transfer: Some(EmiTransfer::Measured {
            source: Some(source.clone()),
            intervals: vec![EmiInterval {
                end,
                value: EmiBand::point(-1.2),
            }],
        }),
    };
    let Analysis::Analyzed { value: trial, .. } = prepare_emi_reset(input.clone()).unwrap() else {
        panic!("нет контрольной огибающей");
    };
    let observed = EmiResetObservation {
        complete: true,
        observed_until: end,
        reset_at: Some(at),
        external_asserted_at: Some(at),
        baseline_reset_at: None,
    };
    let confirmed = assess_emi_reset(&trial, Some(observed));
    assert_eq!(confirmed.outcome, EmiOutcome::Fail { at });
    assert_eq!(confirmed.margins.dwell_ns, [-3_000; 3]);
    assert!((confirmed.margins.voltage_v.nominal + 0.2).abs() <= f64::EPSILON);
    for observed in [
        None,
        Some(EmiResetObservation {
            external_asserted_at: None,
            ..observed
        }),
        Some(EmiResetObservation {
            reset_at: None,
            ..observed
        }),
        Some(EmiResetObservation {
            baseline_reset_at: Some(at),
            ..observed
        }),
        Some(EmiResetObservation {
            observed_until: Time(at.0 - 1),
            ..observed
        }),
    ] {
        let evidence = assess_emi_reset(&trial, observed);
        assert!(matches!(evidence.outcome, EmiOutcome::NotAnalyzed { .. }));
        assert_eq!(evidence.criterion, None);
        assert!(evidence.packet.is_none());
    }
    let fixed = EmiInput {
        transfer: Some(EmiTransfer::Measured {
            source: Some(source),
            intervals: vec![EmiInterval {
                end,
                value: EmiBand::point(-0.8),
            }],
        }),
        ..input
    };
    let Analysis::Analyzed { value: trial, .. } = prepare_emi_reset(fixed).unwrap() else {
        panic!("нет контрольной огибающей");
    };
    let quiet = EmiResetObservation {
        reset_at: None,
        external_asserted_at: None,
        ..observed
    };
    let evidence = assess_emi_reset(&trial, Some(quiet));
    assert_eq!(evidence.criterion, Some(EmiCriterion::WithinBench));
    assert!(matches!(evidence.outcome, EmiOutcome::NotAnalyzed { .. }));
    assert_eq!(assess_emi_reset(&trial, None).criterion, None);
}
