use sim_circuit::{Circuit, SimOptions, operating_point_with, parse};
use sim_process::schematic::switch_thermal::*;
use sim_process::schematic::{Endpoint, Mode, Scope, Section, Status};
use sim_process::{Analysis, Interval, ProcessError, SourcedRange};
use sim_signal::Time;
use sim_world::Provenance;

// Стенд «блок». В каждой фазе идеальный источник питает эквивалент
// канала через 1 Ом. В линейной фазе 6 В / (1 + 2) Ом = 2 А,
// Vds = 4 В, Vgs = 2 В, P = 8 Вт. В паузах ток равен нулю.
// Ta = Tc = 300 К, θja = 3 К/Вт. Два звена Фостера (R, τ):
// (1 К/Вт, 1 с) и (2 К/Вт, 2 с). Начало в холодном равновесии.
// Tj(2 с) = 300 + 8[(1 − exp(−2)) + 2(1 − exp(−1))] = 317,031246675 К.
// Tj(0,5 с) = 306,686942193 К. При пределе 322 К ручной запас
// +4,968753325 / +15,313057807 К. Линия SOA 1 с при 4 В, 2 А, 300 К,
// запас 1 − 2 = −1 с / 1 − 0,5 = +0,5 с. За окно 10 с средние
// токи 0,4 / 0,1 А. Они не отражают 2 А во время линейного нагрева.
// Длительный опыт: Vgs = 5 В, Vds = 1 В, Id = 8 / 2 А.
// Tj∞ = 300 + (8 / 2)·3 = 324 / 306 К. При 318 К запас −6 / +12 К.
const TOL_K: f64 = 0.000_001; // Заявленный численный допуск стенда, сверяется с RK4.
const SECOND: u64 = 1_000_000_000; // Наносекунд в секунде.

fn window(start: u64, end: u64) -> Interval {
    Interval {
        start: Time(start),
        end: Time(end),
    }
}

fn source() -> Provenance {
    Provenance::Derived("Условия и аналитические огибающие стенда блока".into())
}

fn exact(value: f64) -> SourcedRange {
    SourcedRange {
        min: value,
        nominal: value,
        max: value,
        provenance: source(),
    }
}

struct Bench {
    branch: Section,
    terminals: [Endpoint; 3],
    ta: SourcedRange,
    tc: SourcedRange,
    full_gate: SourcedRange,
    theta: SourcedRange,
    r: [SourcedRange; 2],
    tau: [SourcedRange; 2],
    rises: [SourcedRange; 2],
    fit: SourcedRange,
    tolerance: SourcedRange,
    limit: SourcedRange,
    cool_line: SourcedRange,
    warm_line: SourcedRange,
    applicability: Provenance,
}

impl Bench {
    fn new(limit: f64) -> Self {
        Self {
            branch: Section::Element {
                component: "блок".into(),
                element: "channel".into(),
            },
            terminals: ["d", "s", "g"].map(|pin| Endpoint {
                component: "блок".into(),
                pin: pin.into(),
            }),
            ta: exact(300.0),
            tc: exact(300.0),
            full_gate: exact(5.0),
            theta: exact(3.0),
            r: [exact(1.0), exact(2.0)],
            tau: [exact(1.0), exact(2.0)],
            rises: [exact(0.0), exact(0.0)],
            fit: exact(0.0), // Тепловая модель определяет сам синтетический стенд.
            tolerance: exact(TOL_K),
            limit: exact(limit),
            cool_line: exact(1.0),
            warm_line: exact(0.25), // Отдельная синтетическая линия для горячего корпуса.
            applicability: source(),
        }
    }

    fn input<'a>(&'a self, segments: &'a [SwitchSegment<'a>]) -> SwitchThermalInput<'a> {
        SwitchThermalInput {
            scope: Scope::Reference,
            mode: &Mode::Normal,
            branch: &self.branch,
            terminals: &self.terminals,
            window: window(0, 10 * SECOND),
            segments: Some(segments),
            ambient_k: Some(&self.ta),
            case_k: Some(&self.tc),
            initial_rises_k: Some(&self.rises),
            thermal_applicability: Some(&self.applicability),
            exposure: SwitchExposure::Window,
        }
    }

    fn poles(&self) -> [SwitchFosterPole<'_>; 2] {
        std::array::from_fn(|i| SwitchFosterPole {
            resistance_k_per_w: &self.r[i],
            time_constant_s: &self.tau[i],
        })
    }

    fn limits(&self) -> [Option<SwitchSoaLimit<'_>>; 2] {
        [
            Some(SwitchSoaLimit::Pulse(&self.cool_line)),
            Some(SwitchSoaLimit::Pulse(&self.warm_line)),
        ]
    }

    fn ratings<'a>(
        &'a self,
        poles: &'a [SwitchFosterPole<'a>],
        limits: &'a [Option<SwitchSoaLimit<'a>>],
    ) -> SwitchThermalRatings<'a> {
        SwitchThermalRatings {
            max_junction_k: Some(&self.limit),
            theta_ja_k_per_w: Some(&self.theta),
            foster: Some(SwitchFoster {
                poles,
                fit_error_k: &self.fit,
            }),
            agreement_tolerance_k: Some(&self.tolerance),
            fully_on_vgs_v: Some(&self.full_gate),
            soa: Some(SwitchSoa {
                vds_edges_v: &[4.0],
                id_edges_a: &[2.0],
                case_edges_k: &[300.0, 310.0, 320.0], // Граница горячей линии 310 К.
                limits,
                applicability: Some(&self.applicability),
                duty: SwitchSoaDuty::SinglePulse,
            }),
        }
    }
}

fn operating_values(vds: f64, id: f64, vgs: f64, resistance: f64) -> [SourcedRange; 3] {
    let supply = vds + id; // 1 Ом внешней нагрузки.
    let circuit = Circuit::from_netlist(&parse(&format!(
        "Блок\nVsrc supply 0 {supply}\nRload supply d 1\nRchannel d 0 {resistance}\nVgate g 0 {vgs}\n.end\n"
    )).unwrap()).unwrap();
    let op = operating_point_with(
        &circuit,
        SimOptions {
            // В аналитической схеме стенда отсутствуют шунты узлов на землю.
            gshunt: 0.0,
            ..SimOptions::default()
        },
    )
    .unwrap();
    let values = [
        op.voltage("d").unwrap(),
        op.current("rchannel").unwrap(),
        op.voltage("g").unwrap(),
    ];
    assert_eq!(values, [vds, id, vgs]); // Точные точки схемы из ручного расчёта выше.
    values.map(exact)
}

fn segment(window: Interval, [vds, id, vgs]: &[SourcedRange; 3]) -> SwitchSegment<'_> {
    SwitchSegment {
        window,
        vds_v: Some(vds),
        id_a: Some(id),
        vgs_v: Some(vgs),
    }
}

fn pulse<'a>(
    on: &'a [SourcedRange; 3],
    off: &'a [SourcedRange; 3],
    duration_ns: u64,
) -> [SwitchSegment<'a>; 3] {
    [
        segment(window(0, SECOND), off),
        segment(window(SECOND, SECOND + duration_ns), on),
        segment(window(SECOND + duration_ns, 10 * SECOND), off),
    ]
}

fn checked<T>(analysis: &Analysis<T>) -> &T {
    match analysis {
        Analysis::Analyzed { value, .. } => value,
        Analysis::NotAnalyzed { reason } => panic!("{reason}"),
    }
}

fn encloses(bounds: SwitchBounds, expected: f64, tolerance: f64) {
    assert!(
        bounds.min <= expected + tolerance && bounds.max >= expected - tolerance,
        "{bounds:?}, ожидалось {expected}"
    );
    assert!(bounds.max - bounds.min < tolerance, "{bounds:?}");
}

// Независимое интегрирование дифференциальных уравнений тепловых звеньев
// классическим RK4. Модуль рассчитывает аналитическое решение через expm1.
fn integrate(states: &mut [f64; 2], power: f64, duration: f64, steps: u32) {
    let dt = duration / f64::from(steps);
    for (i, x) in states.iter_mut().enumerate() {
        let r = [1.0, 2.0][i];
        let tau = [1.0, 2.0][i];
        let slope = |rise| (r * power - rise) / tau;
        for _ in 0..steps {
            let k1 = slope(*x);
            let k2 = slope(*x + dt * k1 / 2.0);
            let k3 = slope(*x + dt * k2 / 2.0);
            let k4 = slope(*x + dt * k3);
            *x += dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
        }
    }
}

#[test]
fn sustained_temperature_has_the_expected_reserve_on_both_sides() {
    let bench = Bench::new(318.0);
    let poles = bench.poles();
    let limits = bench.limits();
    let ratings = bench.ratings(&poles, &limits);
    for (current, peak, reserve, status) in [
        (8.0, 324.0, -6.0, Status::Fail),
        (2.0, 306.0, 12.0, Status::Pass),
    ] {
        let on = operating_values(1.0, current, 5.0, 1.0 / current);
        let segments = [segment(window(0, 10 * SECOND), &on)];
        let input = SwitchThermalInput {
            exposure: SwitchExposure::Sustained(&bench.applicability),
            ..bench.input(&segments)
        };
        let finding = assess_switch_thermal(&input, &ratings).unwrap();
        assert_eq!(finding.status, status, "{}", finding.reason);
        let e = finding.evidence;
        assert_eq!(e.code, "switch_thermal");
        assert_eq!(e.branch, bench.branch);
        assert_eq!(e.mode, Mode::Normal);
        assert_eq!(e.scope, Scope::Reference);
        assert_eq!(e.window, input.window);
        assert!(e.sustained);
        let steady = checked(&e.steady_junction_k);
        let rc = checked(&e.foster_equilibrium_junction_k);
        encloses(steady.observed, peak, TOL_K);
        encloses(steady.margin, reserve, TOL_K);
        encloses(rc.observed, peak, TOL_K);
        encloses(checked(&e.foster_peak_junction_k).observed, peak, TOL_K);
        assert!(checked(&e.agreement_k).margin.min > 0.0);
        assert!(checked(&e.linear_soa).is_empty());
    }
}

#[test]
fn linear_pulse_uses_branch_power_case_temperature_and_full_duration() {
    let mut bench = Bench::new(322.0);
    let on = operating_values(4.0, 2.0, 2.0, 2.0);
    let off = operating_values(0.0, 0.0, 0.0, 2.0);
    let cases = [
        (
            2 * SECOND,
            317.031_246_675,
            4.968_753_325,
            -1.0,
            Status::Fail,
        ),
        (
            SECOND / 2,
            306.686_942_193,
            15.313_057_807,
            0.5,
            Status::Pass,
        ),
    ];
    for (ticks, expected_k, reserve_k, reserve_s, status) in cases {
        let segments = pulse(&on, &off, ticks);
        let input = bench.input(&segments);
        let finding =
            assess_switch_thermal(&input, &bench.ratings(&bench.poles(), &bench.limits())).unwrap();
        assert_eq!(finding.status, status, "{}", finding.reason);
        let e = finding.evidence;
        assert_eq!(e.code, "switch_thermal");
        encloses(checked(&e.steady_junction_k).observed, 324.0, TOL_K);
        let tj = checked(&e.foster_peak_junction_k);
        encloses(tj.observed, expected_k, TOL_K);
        encloses(tj.margin, reserve_k, TOL_K);
        let mut rises = [0.0; 2];
        // Сходимость RK4 независимо проверяется на 256 и 512 шагах за импульс.
        let mut refined = rises;
        let duration = Time(ticks).as_secs();
        integrate(&mut rises, 8.0, duration, 256);
        integrate(&mut refined, 8.0, duration, 512);
        let numerical_peak = 300.0 + refined.iter().sum::<f64>();
        assert!((rises.iter().sum::<f64>() - refined.iter().sum::<f64>()).abs() < TOL_K);
        encloses(tj.observed, numerical_peak, TOL_K);
        let soa = checked(&e.linear_soa);
        assert_eq!(soa.len(), 1);
        assert_eq!(soa[0].window, segments[1].window);
        assert_eq!(soa[0].status, status);
        encloses(*checked(&soa[0].margin_s), reserve_s, TOL_K);
        assert!(2.0 * duration / input.window.end.as_secs() < 0.5);
        eprintln!(
            "t={duration} с, Tj={numerical_peak:.9} К, запас {reserve_k:+.9} К, SOA {reserve_s:+.3} с"
        );
    }

    bench.tc = exact(310.0); // 0,5 с − 0,25 с теперь за горячей линией.
    let segments = pulse(&on, &off, SECOND / 2);
    let result = assess_switch_thermal(
        &bench.input(&segments),
        &bench.ratings(&bench.poles(), &bench.limits()),
    )
    .unwrap();
    assert_eq!(result.status, Status::Fail);
    encloses(
        *checked(&checked(&result.evidence.linear_soa)[0].margin_s),
        -0.25,
        TOL_K,
    );
    let case_crossing = SourcedRange {
        min: 300.0,
        ..bench.tc.clone()
    };
    let input = SwitchThermalInput {
        case_k: Some(&case_crossing),
        ..bench.input(&segments)
    };
    let result =
        assess_switch_thermal(&input, &bench.ratings(&bench.poles(), &bench.limits())).unwrap();
    assert_eq!(result.status, Status::NotAnalyzed);
    let soa = checked(&result.evidence.linear_soa);
    assert_eq!(
        *checked(&soa[0].allowed),
        [SwitchSoaTime::Seconds(0.25), SwitchSoaTime::Seconds(1.0)]
    );
}

#[test]
fn transient_temperature_crosses_its_limit_within_continuous_soa() {
    // Те же две ручные температуры из начала файла. Предел 310 К даёт
    // запас −7,031246675 / +3,313057807 К при независимом разрешении SOA.
    let bench = Bench::new(310.0);
    let on = operating_values(4.0, 2.0, 2.0, 2.0);
    let off = operating_values(0.0, 0.0, 0.0, 2.0);
    let poles = bench.poles();
    let continuous = [Some(SwitchSoaLimit::Continuous(&bench.applicability)); 2];
    let ratings = bench.ratings(&poles, &continuous);
    for (ticks, reserve, status) in [
        (2 * SECOND, -7.031_246_675, Status::Fail),
        (SECOND / 2, 3.313_057_807, Status::Pass),
    ] {
        let segments = pulse(&on, &off, ticks);
        let result = assess_switch_thermal(&bench.input(&segments), &ratings).unwrap();
        assert_eq!(result.status, status, "{}", result.reason);
        assert_eq!(result.evidence.code, "switch_thermal");
        let soa = checked(&result.evidence.linear_soa);
        assert_eq!(soa[0].status, Status::Pass);
        assert_eq!(*checked(&soa[0].allowed), [SwitchSoaTime::Continuous; 2]);
        encloses(
            checked(&result.evidence.foster_peak_junction_k).margin,
            reserve,
            TOL_K,
        );
    }
}

#[test]
fn missing_estimated_and_unresolved_inputs_cannot_pass() {
    let bench = Bench::new(322.0);
    let on = operating_values(4.0, 2.0, 2.0, 2.0);
    let off = operating_values(0.0, 0.0, 0.0, 2.0);
    let segments = pulse(&on, &off, SECOND / 2);
    let input = bench.input(&segments);
    let poles = bench.poles();
    let limits = bench.limits();
    let ratings = bench.ratings(&poles, &limits);
    let soa = ratings.soa.unwrap();
    let estimate = Provenance::Estimate("Оценка без источника".into());
    let estimated_range = SourcedRange {
        provenance: estimate.clone(),
        ..exact(1.0)
    };
    let estimated_poles = [
        SwitchFosterPole {
            time_constant_s: &estimated_range,
            ..bench.poles()[0]
        },
        bench.poles()[1],
    ];
    let estimated_lines = [
        Some(SwitchSoaLimit::Pulse(&estimated_range)),
        bench.limits()[1],
    ];
    let gap_lines = [None, bench.limits()[1]];
    let gaps = [segments[0], segments[2]]; // Точки до и после импульса с током 2 А.
    let mut uncertain_gate = segments;
    let gate_bound = SourcedRange {
        min: 2.0,
        nominal: 2.0,
        max: 5.0,
        provenance: source(),
    };
    uncertain_gate[1].vgs_v = Some(&gate_bound);
    let mut missing_current = segments;
    missing_current[1].id_a = None;
    let off_axis = exact(299.0);
    let assert_open = |input: &SwitchThermalInput<'_>, ratings: &SwitchThermalRatings<'_>| {
        let result = assess_switch_thermal(input, ratings).unwrap();
        assert_eq!(result.status, Status::NotAnalyzed, "{}", result.reason);
        assert!(!result.reason.is_empty());
    };
    for input in [
        SwitchThermalInput {
            segments: None,
            ..input
        },
        SwitchThermalInput {
            segments: Some(&gaps),
            ..input
        },
        SwitchThermalInput {
            segments: Some(&missing_current),
            ..input
        },
        SwitchThermalInput {
            segments: Some(&uncertain_gate),
            ..input
        },
        SwitchThermalInput {
            case_k: None,
            ..input
        },
        SwitchThermalInput {
            case_k: Some(&off_axis),
            ..input
        },
        SwitchThermalInput {
            ambient_k: None,
            ..input
        },
        SwitchThermalInput {
            initial_rises_k: None,
            ..input
        },
        SwitchThermalInput {
            thermal_applicability: None,
            ..input
        },
        SwitchThermalInput {
            thermal_applicability: Some(&estimate),
            ..input
        },
        SwitchThermalInput {
            scope: Scope::Assembly,
            ..input
        },
        SwitchThermalInput {
            exposure: SwitchExposure::Sustained(&estimate),
            ..input
        },
        SwitchThermalInput {
            window: window(SECOND, 10 * SECOND),
            segments: Some(&segments[1..]),
            ..input
        },
    ] {
        assert_open(&input, &ratings);
    }
    for missing in [
        SwitchThermalRatings {
            foster: None,
            ..ratings
        },
        SwitchThermalRatings {
            max_junction_k: None,
            ..ratings
        },
        SwitchThermalRatings {
            theta_ja_k_per_w: None,
            ..ratings
        },
        SwitchThermalRatings {
            fully_on_vgs_v: None,
            ..ratings
        },
        SwitchThermalRatings {
            agreement_tolerance_k: None,
            ..ratings
        },
        SwitchThermalRatings {
            soa: None,
            ..ratings
        },
        SwitchThermalRatings {
            soa: Some(SwitchSoa {
                applicability: Some(&estimate),
                ..soa
            }),
            ..ratings
        },
        SwitchThermalRatings {
            soa: Some(SwitchSoa {
                limits: &estimated_lines,
                ..soa
            }),
            ..ratings
        },
        SwitchThermalRatings {
            soa: Some(SwitchSoa {
                limits: &gap_lines,
                ..soa
            }),
            ..ratings
        },
        SwitchThermalRatings {
            foster: Some(SwitchFoster {
                poles: &estimated_poles,
                ..ratings.foster.unwrap()
            }),
            ..ratings
        },
    ] {
        assert_open(&input, &missing);
    }
    // На 1 К/Вт расходятся независимые сопротивления, на 8 К температуры.
    // Последний интервал имеет P = 0, расхождение сохраняется на нагруженной фазе.
    let changed_theta = exact(4.0);
    let inconsistent = SwitchThermalRatings {
        theta_ja_k_per_w: Some(&changed_theta),
        ..ratings
    };
    let sustained = SwitchThermalInput {
        exposure: SwitchExposure::Sustained(&bench.applicability),
        ..input
    };
    let result = assess_switch_thermal(&sustained, &inconsistent).unwrap();
    assert_eq!(result.status, Status::NotAnalyzed);
    encloses(checked(&result.evidence.agreement_k).observed, 8.0, TOL_K);
}

#[test]
fn adjacent_intervals_keep_duration_and_single_pulse_does_not_cover_a_train() {
    let bench = Bench::new(322.0);
    let on = operating_values(4.0, 2.0, 2.0, 2.0);
    let off = operating_values(0.0, 0.0, 0.0, 2.0);
    // Два участка по 0,75 с. Ни один сам по себе не нарушает SOA 1 с.
    let segments = [
        segment(window(0, SECOND), &off),
        segment(window(SECOND, 7 * SECOND / 4), &on),
        segment(window(7 * SECOND / 4, 5 * SECOND / 2), &on),
        segment(window(5 * SECOND / 2, 10 * SECOND), &off),
    ];
    let poles = bench.poles();
    let limits = bench.limits();
    let ratings = bench.ratings(&poles, &limits);
    let result = assess_switch_thermal(&bench.input(&segments), &ratings).unwrap();
    assert_eq!(result.status, Status::Fail);
    let soa = checked(&result.evidence.linear_soa);
    assert_eq!(soa.len(), 1);
    encloses(*checked(&soa[0].margin_s), -0.5, TOL_K);

    let train = [
        segment(window(0, SECOND), &off),
        segment(window(SECOND, 3 * SECOND / 2), &on),
        segment(window(3 * SECOND / 2, 8 * SECOND), &off),
        segment(window(8 * SECOND, 17 * SECOND / 2), &on),
        segment(window(17 * SECOND / 2, 10 * SECOND), &off),
    ];
    let input = bench.input(&train);
    let result = assess_switch_thermal(&input, &ratings).unwrap();
    assert_eq!(result.status, Status::NotAnalyzed);
    let train_rating = SwitchThermalRatings {
        soa: Some(SwitchSoa {
            duty: SwitchSoaDuty::PulseTrain,
            ..ratings.soa.unwrap()
        }),
        ..ratings
    };
    let result = assess_switch_thermal(&input, &train_rating).unwrap();
    assert_eq!(result.status, Status::Pass, "{}", result.reason);
    assert_eq!(checked(&result.evidence.linear_soa).len(), 2);
    let mut f = [0.0; 2];
    integrate(&mut f, 8.0, 0.5, 512);
    integrate(&mut f, 0.0, 6.5, 512);
    integrate(&mut f, 8.0, 0.5, 512);
    encloses(
        checked(&result.evidence.foster_peak_junction_k).observed,
        300.0 + f.iter().sum::<f64>(),
        TOL_K,
    );
}

#[test]
fn unresolved_thermal_peak_between_ticks_blocks_pass() {
    // Быстрое звено растёт, медленное спадает. При P = 4 Вт
    // x₁=4(1−exp(−t)), x₂=8+2exp(−t/2). Из 4exp(−t)=exp(−t/2)
    // следует максимум в t=2 ln 4: 312,25 К. Конец окна даёт 312,0133 К.
    let mut bench = Bench::new(312.2);
    bench.rises = [exact(0.0), exact(10.0)];
    let on = operating_values(1.0, 4.0, 5.0, 0.25);
    let segments = [segment(window(0, 10 * SECOND), &on)];
    let result = assess_switch_thermal(
        &bench.input(&segments),
        &bench.ratings(&bench.poles(), &bench.limits()),
    )
    .unwrap();
    let peak = checked(&result.evidence.foster_peak_junction_k);
    assert!(peak.observed.min < 312.2 && peak.observed.max >= 312.25);
    assert_eq!(result.status, Status::NotAnalyzed, "{}", result.reason);
}

#[test]
fn nonfinite_values_are_errors_even_in_unused_fields() {
    let bench = Bench::new(322.0);
    let on = operating_values(4.0, 2.0, 2.0, 2.0);
    let off = operating_values(0.0, 0.0, 0.0, 2.0);
    let segments = pulse(&on, &off, SECOND / 2);
    let input = bench.input(&segments);
    let poles = bench.poles();
    let limits = bench.limits();
    let ratings = bench.ratings(&poles, &limits);
    let error = |input: &SwitchThermalInput<'_>, ratings: &SwitchThermalRatings<'_>| {
        assert!(matches!(
            assess_switch_thermal(input, ratings),
            Err(ProcessError::InvalidInput(_))
        ));
    };
    for n in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let bad = SourcedRange {
            nominal: n,
            ..exact(1.0)
        };
        let missing = SwitchThermalInput {
            segments: None,
            ..input
        };
        error(
            &SwitchThermalInput {
                case_k: Some(&bad),
                ..missing
            },
            &ratings,
        );
        let mut trace = segments;
        trace[0].vgs_v = Some(&bad);
        error(
            &SwitchThermalInput {
                segments: Some(&trace),
                ..input
            },
            &SwitchThermalRatings::default(),
        );
        let mut poles = bench.poles();
        poles[0].resistance_k_per_w = &bad;
        error(
            &missing,
            &SwitchThermalRatings {
                foster: Some(SwitchFoster {
                    poles: &poles,
                    fit_error_k: &bench.fit,
                }),
                ..ratings
            },
        );
        let soa = ratings.soa.unwrap();
        let edges = [n];
        error(
            &missing,
            &SwitchThermalRatings {
                soa: Some(SwitchSoa {
                    id_edges_a: &edges,
                    ..soa
                }),
                ..ratings
            },
        );
        let lines = [Some(SwitchSoaLimit::Pulse(&bad)), bench.limits()[1]];
        error(
            &missing,
            &SwitchThermalRatings {
                soa: Some(SwitchSoa {
                    limits: &lines,
                    ..soa
                }),
                ..ratings
            },
        );
    }
    let overlapping = [segments[0], segments[0], segments[1], segments[2]];
    error(
        &SwitchThermalInput {
            segments: Some(&overlapping),
            ..input
        },
        &ratings,
    );
}
