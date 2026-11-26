// Подключение к общему модулю схемы выполняется при публикации Т6.
#[path = "../src/schematic/esd_event.rs"]
mod esd_event;

use std::path::Path;

use esd_event::*;
use sim_circuit::ngspice;
use sim_process::schematic::{
    Catalog, CompiledSchematic, Component, Contact, ContactKind, Endpoint, Part, PartKind, Pin,
    PinRole, Schematic, Scope, Status, compile,
};
use sim_process::{Analysis, ProcessError, SourcedRange, schematic};
use sim_signal::Time;
use sim_world::Provenance;

// Импульс стенда «блок»: IEC 61000-4-2:2008, табл. 3 и рис. 2, уровень 1.
// 7,5 А на 1 нс, 10% и 90% на 0,1 и 0,9 нс; 4 и 2 А через 30 и 60 нс от 10%.
// Провал 2 А на 5 нс и линейный хвост до 0 А на 100 нс задаёт сам стенд.
// Ветка вывода 3 + 2 = 5 Ом. Без TVS при 7,5 А имеем 37,5 В на клемме
// и 15 В / 7,5 А на выводе. Запас относительно 4 В / 3 А: −11 В / −4,5 А.
// TVS стенда: V₀ = 6,5 В, Rᴅ = 0,5 Ом. Точки 1 А / 7 В и 5 А / 9 В
// взяты из TI ESD224, табл. 6.6, https://www.ti.com/lit/ds/symlink/esd224.pdf.
// Линейный эквивалент без динамических эффектов определён только для стенда.
// KCL: I = V/5 + (V−6,5)/0,5, откуда Vклеммы = 205/22 В,
// Iвывода = 41/22 А, Vвывода = 41/11 В, Itvs = 62/11 А.
// Запас на выводе равен 3/11 В и 25/22 А. На спаде при 2 А TVS ещё открыт,
// при 0 А закрыт. ngspice сверяет обе области, включая переход через излом.
const NS: f64 = 1e-9; // Наносекунда в СИ.
const PULSE: [EsdSample; 6] = [
    EsdSample {
        offset_s: 0.0,
        current_abs_a: 0.0,
    },
    EsdSample {
        offset_s: NS,
        current_abs_a: 7.5,
    },
    EsdSample {
        offset_s: 5.0 * NS,
        current_abs_a: 2.0,
    },
    EsdSample {
        offset_s: 30.1 * NS,
        current_abs_a: 4.0,
    },
    EsdSample {
        offset_s: 60.1 * NS,
        current_abs_a: 2.0,
    },
    EsdSample {
        offset_s: 100.0 * NS,
        current_abs_a: 0.0,
    },
];
const VOLTAGE_LIMIT: f64 = 4.0;
const CURRENT_LIMIT: f64 = 3.0;
const ANALYTIC_TOL: f64 = 1e-12; // Допуск стенда на округление в ручной и узловой формулах.
const SPICE_TOL: f64 = 1e-6; // Заявленный допуск стенда на абсолютную невязку U/I ngspice.

fn bench_source() -> Provenance {
    Provenance::Derived("Определение синтетического стенда блока".into())
}

fn exact(value: f64) -> SourcedRange {
    SourcedRange {
        min: value,
        nominal: value,
        max: value,
        provenance: bench_source(),
    }
}

fn ends() -> EsdPort {
    let endpoint = |pin: &str| Endpoint {
        component: "блок".into(),
        pin: pin.into(),
    };
    EsdPort {
        exposed: endpoint("terminal"),
        protected: endpoint("input"),
        return_: endpoint("return"),
    }
}

fn schematic(scope: Scope) -> CompiledSchematic {
    let catalog = Catalog::new(
        [(
            "block".into(),
            Part {
                kind: PartKind::Other,
                source: "Описание стенда блока".into(),
                pins: [
                    ("terminal", ContactKind::Terminal, PinRole::Passive),
                    (
                        "input",
                        ContactKind::Pad,
                        PinRole::SignalIn("return".into()),
                    ),
                    ("return", ContactKind::Pad, PinRole::Return),
                ]
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
                model_gap: Some("Переходная модель блока передаётся отдельно".into()),
                reference_only: scope == Scope::Reference,
            },
        )]
        .into(),
    )
    .unwrap();
    compile(
        Schematic {
            name: "Блок".into(),
            scope,
            nets: ["G", "EXPOSED", "INPUT"].map(str::to_owned).into(),
            components: [(
                "блок".into(),
                Component {
                    part: "block".into(),
                    nets: [("terminal", "EXPOSED"), ("input", "INPUT"), ("return", "G")]
                        .map(|(pin, net)| (pin.into(), net.into()))
                        .into(),
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

struct Bench {
    source: Provenance,
    limits: EsdPinLimits,
    port: EsdPort,
}

impl Bench {
    fn new() -> Self {
        Self {
            source: bench_source(),
            limits: EsdPinLimits {
                voltage_abs_v: exact(VOLTAGE_LIMIT),
                current_abs_a: exact(CURRENT_LIMIT),
            },
            port: ends(),
        }
    }

    fn input(&self, protected: bool, polarity: EsdPolarity) -> EsdInput<'_> {
        EsdInput {
            port: &self.port,
            start: Time(100), // Начало условного контакта через 100 нс от старта стенда.
            exposure: Some(&self.source),
            pulse: Some(EsdPulse {
                level: EsdLevel::Kv2,
                polarity,
                samples: &PULSE,
                bandwidth_hz: 2e9, // Полоса применимости синтетического воздействия, 2 ГГц.
                source: Some(&self.source),
            }),
            network: Some(EsdNetwork {
                series_ohm: 3.0,
                pin_ohm: 2.0,
                domain: EsdDomain {
                    bandwidth_hz: 2e9,
                    // Геометрия стенда: 1 мм, v=2·10⁸ м/с, l/λ=0,01 при 2 ГГц.
                    length_m: 1e-3,
                    wave_speed_m_s: 2e8,
                    max_length_wavelengths: 0.1, // Объявленная граница стенда.
                    max_terminal_abs_v: 40.0,    // Заявленная общая область обоих вариантов.
                    // Стенд определяется точной резистивной схемой.
                    voltage_error_v: 0.0,
                    current_error_a: 0.0,
                    source: Some(&self.source),
                },
                protection: Some(if protected {
                    EsdProtection::Tvs(EsdTvs {
                        polarity,
                        knee_v: 6.5,
                        dynamic_ohm: 0.5,
                        max_injected_peak_a: 8.0, // Границы синтетического паспорта стенда.
                        min_rise_s: 0.6 * NS,
                        max_pulse_s: 100.0 * NS,
                        datasheet: Some(&self.source),
                    })
                } else {
                    EsdProtection::Absent(&self.source)
                }),
            }),
            limits: Some(&self.limits),
        }
    }
}

fn calculated<T>(analysis: Analysis<T>) -> T {
    match analysis {
        Analysis::Analyzed { value, .. } => value,
        Analysis::NotAnalyzed { reason } => panic!("{reason}"),
    }
}

fn close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual:.16e} != {expected:.16e}"
    );
}

#[test]
fn wall_of_limits_and_tvs_for_both_polarities() {
    let bench = Bench::new();
    let scheme = schematic(Scope::Reference);
    for (polarity, sign) in [(EsdPolarity::Positive, 1.0), (EsdPolarity::Negative, -1.0)] {
        for protected in [false, true] {
            let input = bench.input(protected, polarity);
            let finding = assess_esd_event(&scheme, &input).unwrap();
            assert_eq!(finding.evidence.code, "esd_event");
            assert_eq!(finding.evidence.port, bench.port);
            assert_eq!(finding.evidence.start, input.start);
            assert_eq!(finding.evidence.polarity, Some(polarity));
            let expect = if protected {
                assert_eq!(finding.status, Status::NotAnalyzed);
                assert!(finding.reason.contains("измер"));
                assert_eq!(finding.evidence.criterion, Some(EsdCriterion::WithinBench));
                [
                    205.0 / 22.0,
                    41.0 / 11.0,
                    41.0 / 22.0,
                    62.0 / 11.0,
                    3.0 / 11.0,
                    25.0 / 22.0,
                ]
            } else {
                assert_eq!(finding.status, Status::Fail);
                assert_eq!(finding.evidence.criterion, Some(EsdCriterion::Exceeded));
                [37.5, 15.0, 7.5, 0.0, -11.0, -4.5]
            };
            let response = calculated(finding.evidence.response);
            close(response.peak_offset_s, NS, 1e-21);
            close(response.peak.pin_v, sign * expect[1], ANALYTIC_TOL);
            close(response.peak.pin_a, sign * expect[2], ANALYTIC_TOL);
            close(response.peak.terminal_v, sign * expect[0], ANALYTIC_TOL);
            close(response.peak.tvs_a, sign * expect[3], ANALYTIC_TOL);
            close(response.pulse.first_peak_a, 7.5, ANALYTIC_TOL);
            close(response.pulse.rise_s, 0.8 * NS, 1e-21); // Допуск округления на шкале нс.
            close(response.pulse.at_30ns_a, 4.0, ANALYTIC_TOL);
            close(response.pulse.at_60ns_a, 2.0, ANALYTIC_TOL);
            close(response.electrical_length_wavelengths, 0.01, ANALYTIC_TOL);
            let margins = calculated(finding.evidence.margins);
            close(margins.voltage_v.lower, expect[4], ANALYTIC_TOL);
            close(margins.current_a.upper, expect[5], ANALYTIC_TOL);
        }
    }
}

#[test]
fn threshold_boundaries_and_uncertain_overlap() {
    let mut bench = Bench::new();
    let scheme = schematic(Scope::Reference);
    // Из ручного расчёта незащищённого стенда: 15 В и 7,5 А.
    // Шаг проверки 1 мВ / 1 мА, включая точное равенство предела.
    for (voltage, current, expected) in [
        (14.999, 8.0, EsdCriterion::Exceeded),
        (16.0, 7.499, EsdCriterion::Exceeded),
        (15.000, 7.500, EsdCriterion::WithinBench),
        (15.001, 7.501, EsdCriterion::WithinBench),
    ] {
        bench.limits = EsdPinLimits {
            voltage_abs_v: exact(voltage),
            current_abs_a: exact(current),
        };
        let input = bench.input(false, EsdPolarity::Positive);
        let finding = assess_esd_event(&scheme, &input).unwrap();
        assert_eq!(finding.evidence.criterion, Some(expected));
        assert_eq!(
            finding.status,
            if expected == EsdCriterion::Exceeded {
                Status::Fail
            } else {
                Status::NotAnalyzed
            }
        );
    }
    // Огибающая из условия стенда: 15 ±0,002 В вокруг порога 15,001 В.
    let mut input = bench.input(false, EsdPolarity::Positive);
    input.network.as_mut().unwrap().domain.voltage_error_v = 0.002;
    let finding = assess_esd_event(&scheme, &input).unwrap();
    assert_eq!(finding.evidence.criterion, Some(EsdCriterion::Overlap));
    assert_eq!(finding.status, Status::NotAnalyzed);
    let margins = calculated(finding.evidence.margins);
    close(margins.voltage_v.lower, -0.001, ANALYTIC_TOL);
    close(margins.voltage_v.upper, 0.003, ANALYTIC_TOL);
}

#[test]
fn later_peak_is_seen_and_tvs_range_uses_it() {
    let bench = Bench::new();
    let scheme = schematic(Scope::Reference);
    // Поздний максимум 8,5 А между провалом и точкой 30 нс.
    // В открытом эквиваленте Vвывода = I·2 Ом, максимум 17 В на 8 нс.
    let mut pulse = PULSE.to_vec();
    pulse.insert(
        3,
        EsdSample {
            offset_s: 8.0 * NS,
            current_abs_a: 8.5,
        },
    );
    let mut input = bench.input(false, EsdPolarity::Positive);
    input.pulse.as_mut().unwrap().samples = &pulse;
    // 8,5 А на 5 Ом дают 42,5 В. Для этой формы стенд допускает 45 В.
    input.network.as_mut().unwrap().domain.max_terminal_abs_v = 45.0;
    let finding = assess_esd_event(&scheme, &input).unwrap();
    let response = calculated(finding.evidence.response);
    close(response.pulse.first_peak_a, 7.5, ANALYTIC_TOL);
    close(response.peak.pin_v, 17.0, ANALYTIC_TOL);
    close(response.peak_offset_s, 8.0 * NS, 1e-21);
    let mut protected = bench.input(true, EsdPolarity::Positive);
    protected.pulse.as_mut().unwrap().samples = &pulse;
    let finding = assess_esd_event(&scheme, &protected).unwrap();
    assert_eq!(finding.status, Status::NotAnalyzed);
    assert!(finding.reason.contains("области ВАХ"));
}

#[test]
fn contact_levels_scale_against_the_iec_table() {
    let bench = Bench::new();
    let scheme = schematic(Scope::Reference);
    // Таблица 3 IEC 61000-4-2:2008. Для других уровней токовые отметки
    // масштабируются относительно 2 кВ, временные отметки сохраняются.
    for (level, factor, first, i30, i60) in [
        (EsdLevel::Kv4, 2.0, 15.0, 8.0, 4.0),
        (EsdLevel::Kv6, 3.0, 22.5, 12.0, 6.0),
        (EsdLevel::Kv8, 4.0, 30.0, 16.0, 8.0),
    ] {
        let samples = PULSE.map(|sample| EsdSample {
            current_abs_a: sample.current_abs_a * factor,
            ..sample
        });
        let mut input = bench.input(false, EsdPolarity::Positive);
        input.pulse.as_mut().unwrap().level = level;
        input.pulse.as_mut().unwrap().samples = &samples;
        // 30 А на 5 Ом дают до 150 В в обобщённом синтетическом стенде.
        input.network.as_mut().unwrap().domain.max_terminal_abs_v = 150.0;
        let finding = assess_esd_event(&scheme, &input).unwrap();
        assert_eq!(finding.status, Status::Fail);
        let response = calculated(finding.evidence.response);
        close(response.pulse.first_peak_a, first, ANALYTIC_TOL);
        close(response.pulse.at_30ns_a, i30, ANALYTIC_TOL);
        close(response.pulse.at_60ns_a, i60, ANALYTIC_TOL);
        close(response.peak.pin_v, 2.0 * first, ANALYTIC_TOL); // Vpin = I · 2 Ом.
    }
}

#[test]
fn gaps_and_out_of_domain_never_close_the_criterion() {
    let mut bench = Bench::new();
    let scheme = schematic(Scope::Reference);
    let estimated = Provenance::Estimate("Неподтверждённые данные стенда".into());
    let mut wrong_tail = PULSE;
    wrong_tail[4].current_abs_a = 0.5; // Ниже IEC 2 А −30%.
    let mut wrong_front = PULSE;
    wrong_front[1].offset_s = 0.5 * NS; // Фронт 0,4 нс, ниже 0,6 нс.
    let mut wrong_peak = PULSE;
    wrong_peak[1].current_abs_a = 10.0; // Выше IEC 7,5 А +15%.
    let mut truncated = PULSE;
    truncated[4].current_abs_a = 0.0;
    let mut bad_origin = PULSE;
    bad_origin[0].current_abs_a = 1.0;
    type Edit = fn(&mut EsdInput<'_>);
    let edits: [(Edit, &str); 8] = [
        (|i| i.exposure = None, "доступ"),
        (|i| i.pulse = None, "импульс"),
        (|i| i.limits = None, "предел"),
        (|i| i.network = None, "эквивалент"),
        (|i| i.network.as_mut().unwrap().protection = None, "защит"),
        (
            |i| i.network.as_mut().unwrap().domain.length_m *= 20.0,
            "электрическая длина",
        ),
        (|i| i.pulse.as_mut().unwrap().bandwidth_hz *= 2.0, "полоса"),
        (
            |i| i.network.as_mut().unwrap().domain.max_terminal_abs_v = 5.0,
            "напряжение клеммы",
        ),
    ];
    for (edit, reason) in edits {
        let mut input = bench.input(true, EsdPolarity::Positive);
        edit(&mut input);
        assert_open(&scheme, &input, reason);
    }
    for samples in [
        &wrong_tail[..],
        &wrong_front[..],
        &wrong_peak[..],
        &truncated[..5],
        &bad_origin[..],
    ] {
        let mut input = bench.input(true, EsdPolarity::Positive);
        input.pulse.as_mut().unwrap().samples = samples;
        assert_open(&scheme, &input, "IEC");
    }
    for (passport, polarity) in [
        (None, EsdPolarity::Positive),
        (Some(&estimated), EsdPolarity::Positive),
        (Some(&bench.source), EsdPolarity::Negative),
    ] {
        let mut input = bench.input(true, EsdPolarity::Positive);
        let Some(EsdProtection::Tvs(tvs)) = &mut input.network.as_mut().unwrap().protection else {
            panic!("нет модели TVS");
        };
        tvs.datasheet = passport;
        tvs.polarity = polarity;
        assert_open(&scheme, &input, "TVS");
    }
    let mut input = bench.input(true, EsdPolarity::Positive);
    input.pulse.as_mut().unwrap().source = Some(&estimated);
    assert_open(&scheme, &input, "источник импульса");
    assert_open(
        &schematic(Scope::Assembly),
        &bench.input(true, EsdPolarity::Positive),
        "доступ",
    );

    bench.limits.voltage_abs_v.provenance = estimated;
    assert_open(
        &scheme,
        &bench.input(false, EsdPolarity::Positive),
        "предел",
    );
}

fn assert_open(scheme: &CompiledSchematic, input: &EsdInput<'_>, reason: &str) {
    let result = assess_esd_event(scheme, input).unwrap();
    assert_eq!(result.status, Status::NotAnalyzed);
    assert_eq!(result.evidence.criterion, None);
    assert!(result.reason.contains(reason), "{}", result.reason);
}

#[test]
fn assembly_requires_errors_and_a_tvs_passport() {
    let mut bench = Bench::new();
    let scheme = schematic(Scope::Assembly);
    // Фиктивные записи метаданных для проверки допуска источников, без натурных данных.
    let measurement = Provenance::Measured("Имитация записи измерения для теста".into());
    let passport = Provenance::Datasheet("Имитация импульсного паспорта для теста".into());
    bench.limits.voltage_abs_v.provenance = passport.clone();
    bench.limits.current_abs_a.provenance = passport.clone();
    let mut input = bench.input(true, EsdPolarity::Positive);
    input.exposure = Some(&measurement);
    input.pulse.as_mut().unwrap().source = Some(&measurement);
    input.network.as_mut().unwrap().domain.source = Some(&measurement);
    assert_open(&scheme, &input, "погрешности");

    // Сдвиг нижней границы запаса на 0,1 В и 0,2 А по условиям этого теста.
    let network = input.network.as_mut().unwrap();
    network.domain.voltage_error_v = 0.1;
    network.domain.current_error_a = 0.2;
    let Some(EsdProtection::Tvs(tvs)) = network.protection.as_mut() else {
        panic!("нет модели TVS");
    };
    tvs.datasheet = Some(&measurement);
    assert_open(&scheme, &input, "паспорт");

    let Some(EsdProtection::Tvs(tvs)) = input.network.as_mut().unwrap().protection.as_mut() else {
        panic!("нет модели TVS");
    };
    tvs.datasheet = Some(&passport);
    let result = assess_esd_event(&scheme, &input).unwrap();
    assert_eq!(result.status, Status::NotAnalyzed);
    assert_eq!(result.evidence.criterion, Some(EsdCriterion::WithinBench));
    let margins = calculated(result.evidence.margins);
    close(margins.voltage_v.lower, 3.0 / 11.0 - 0.1, ANALYTIC_TOL);
    close(margins.current_a.lower, 25.0 / 22.0 - 0.2, ANALYTIC_TOL);
}

#[test]
fn invalid_input_is_an_error_even_when_other_data_are_missing() {
    let bench = Bench::new();
    let scheme = schematic(Scope::Reference);
    let mut port = ends();
    port.exposed.pin = "missing".into();
    let mut input = bench.input(true, EsdPolarity::Positive);
    input.port = &port;
    assert!(matches!(
        assess_esd_event(&scheme, &input),
        Err(ProcessError::InvalidInput(_))
    ));
    let mut coincident = ends();
    coincident.return_ = coincident.protected.clone();
    input.port = &coincident;
    assert!(assess_esd_event(&scheme, &input).is_err());

    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut samples = PULSE;
        samples[3].current_abs_a = value;
        let mut input = bench.input(false, EsdPolarity::Positive);
        input.exposure = None;
        input.pulse.as_mut().unwrap().samples = &samples;
        assert!(assess_esd_event(&scheme, &input).is_err());

        let mut limits = bench.limits.clone();
        limits.voltage_abs_v.max = value;
        input.pulse = None;
        input.limits = Some(&limits);
        assert!(assess_esd_event(&scheme, &input).is_err());

        input.limits = None;
        input.network.as_mut().unwrap().domain.voltage_error_v = value;
        assert!(assess_esd_event(&scheme, &input).is_err());

        let mut input = bench.input(true, EsdPolarity::Positive);
        let Some(EsdProtection::Tvs(tvs)) = &mut input.network.as_mut().unwrap().protection else {
            panic!("нет модели TVS");
        };
        tvs.dynamic_ohm = value;
        tvs.datasheet = None;
        assert!(assess_esd_event(&scheme, &input).is_err());
    }

    let mut samples = PULSE;
    samples[2].offset_s = samples[1].offset_s;
    let mut input = bench.input(false, EsdPolarity::Positive);
    input.pulse.as_mut().unwrap().samples = &samples;
    assert!(assess_esd_event(&scheme, &input).is_err());
}

fn spice_options() -> EsdSpiceOptions {
    EsdSpiceOptions {
        max_step_s: 0.05 * NS, // 16 шагов на фронт 0,8 нс для независимой сверки.
        voltage_tolerance_v: SPICE_TOL,
        current_tolerance_a: SPICE_TOL,
    }
}

#[test]
fn ngspice_checks_the_whole_trace_and_peaks() {
    let bench = Bench::new();
    let scheme = schematic(Scope::Reference);
    let ng = ngspice::find_ngspice().expect("для приёмки стенда нужен установленный ngspice");
    for polarity in [EsdPolarity::Positive, EsdPolarity::Negative] {
        for protected in [false, true] {
            let mut options = spice_options();
            for divide in [1.0, 2.0] {
                options.max_step_s = spice_options().max_step_s / divide;
                let checked = calculated(
                    cross_check_esd_event(&scheme, &bench.input(protected, polarity), &ng, options)
                        .unwrap(),
                );
                assert!(checked.within_tolerance, "{checked:?}");
                assert!(checked.rows >= (100.0 * NS / options.max_step_s).floor() as usize);
                let (voltage, current) = if protected {
                    (41.0 / 11.0, 41.0 / 22.0)
                } else {
                    (15.0, 7.5)
                };
                close(checked.pin_voltage_peak_abs_v, voltage, SPICE_TOL);
                close(checked.pin_current_peak_abs_a, current, SPICE_TOL);
                println!(
                    "{polarity:?}, TVS {protected}, шаг {:.3e} с: {checked:?}",
                    options.max_step_s
                );
            }
        }
    }
    let mut input = bench.input(false, EsdPolarity::Positive);
    input.network.as_mut().unwrap().series_ohm = 0.0;
    let checked = calculated(cross_check_esd_event(&scheme, &input, &ng, spice_options()).unwrap());
    assert!(checked.within_tolerance, "{checked:?}");
    close(checked.pin_voltage_peak_abs_v, 15.0, SPICE_TOL);
}

#[test]
fn spice_missing_or_underresolved_is_open() {
    let bench = Bench::new();
    let scheme = schematic(Scope::Reference);
    let input = bench.input(false, EsdPolarity::Positive);
    let missing = Path::new("/неустановленный/ngspice");
    let absent = cross_check_esd_event(&scheme, &input, missing, spice_options()).unwrap();
    assert!(matches!(absent, Analysis::NotAnalyzed { reason } if reason.contains("ngspice")));

    let mut options = spice_options();
    options.max_step_s = NS;
    let coarse = cross_check_esd_event(&scheme, &input, missing, options).unwrap();
    assert!(matches!(coarse, Analysis::NotAnalyzed { reason } if reason.contains("шаг")));

    options.current_tolerance_a = f64::INFINITY;
    assert!(cross_check_esd_event(&scheme, &input, missing, options).is_err());
}
