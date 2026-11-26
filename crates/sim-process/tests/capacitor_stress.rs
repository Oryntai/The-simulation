// Подключение к общему модулю схемы выполняет задача 3.7.
#[path = "../src/schematic/capacitor_stress.rs"]
mod capacitor_stress;

use std::{collections::BTreeMap, f64::consts::TAU};

use capacitor_stress::*;
use sim_circuit::{Circuit, TranOptions, Transient, ngspice, parse};
use sim_process::schematic::{ElectricalSeries, Endpoint, Mode, Probe, Scope, Section, Status};
use sim_process::{Analysis, Interval, ProcessError, SourcedRange, schematic};
use sim_signal::Time;
use sim_world::Provenance;

// Стенд «блок»: источник 6 В + A·sin(ωt), 1 Ом внешнего R, 1 Ом ESR,
// C = 1/(ω·2 Ом), f = 1 кГц. Суммарный импеданс 2 − j2 Ом.
// A = 4√2 / 2√2 В даёт Ipk = 2 / 1 А, Irms = √2 / 1/√2 А.
// На выводах C с ESR Upeak = 6 + Ipk·√5 = 10,472135955 / 8,236067978 В.
// За период P = Irms²·ESR = 2 / 0,5 Вт. Стенд задаёт Ta = 300 К,
// θ = 2 К/Вт и нулевую тепловую пульсацию. Tmax = 304 / 301 К.
// При 12 В × 0,75, 1 А и 303 К запас «предел − нагрузка» равен
// −1,472135955 / +0,763932022 В, −0,414213562 / +0,292893219 А, −1 / +2 К.
const FREQUENCY: f64 = 1_000.0;
const PERIOD: Time = Time(1_000_000); // 1/f на шкале наносекунд.
const STEP: Time = Time(1_000); // 1000 отсчётов на период для сверки двух решателей.
// Допуски стенда для абсолютной ошибки ряда. Проверяются против точного решения.
// Допуск интеграла тока учитывает ошибку ряда и квадратуру на сетке ngspice.
const V_ERROR: f64 = 0.001;
const I_ERROR: f64 = 0.0005;
const RMS_REFERENCE_ERROR: f64 = 2.0 * I_ERROR;

const WINDOW: Interval = Interval {
    start: PERIOD,
    end: Time(2 * PERIOD.0),
};

fn exact(value: f64) -> SourcedRange {
    SourcedRange {
        min: value,
        nominal: value,
        max: value,
        provenance: Provenance::Derived("Расчётная постановка RC-стенда блока".into()),
    }
}

fn netlist(peak_a: f64) -> String {
    let amplitude = peak_a * 2.0 * 2.0_f64.sqrt();
    let c = 1.0 / (TAU * FREQUENCY * 2.0);
    let initial_v = 6.0 - amplitude / 2.0;
    format!(
        "RC блок\nVsrc supply 0 SIN(6 {amplitude} {FREQUENCY})\n\
         Rsource supply plus 1\nVprobe plus terminal 0\nResr terminal internal 1\n\
         Cblock internal 0 {c} IC={initial_v}\n.end\n"
    )
}

fn analytic_current(t: f64, peak_a: f64) -> f64 {
    let phase = TAU * FREQUENCY * t;
    peak_a * (phase.sin() + phase.cos()) / 2.0_f64.sqrt()
}

fn analytic_voltage(t: f64, peak_a: f64) -> f64 {
    let phase = TAU * FREQUENCY * t;
    // Из фазора I·(1 − j2), или из суммы Vc и падения на ESR.
    6.0 + peak_a * (3.0 * phase.sin() - phase.cos()) / 2.0_f64.sqrt()
}

struct Bench {
    branch: Section,
    terminals: [Endpoint; 2],
    series: ElectricalSeries,
    frequency: SourcedRange,
    ambient: SourcedRange,
    theta: SourcedRange,
    thermal_ripple: SourcedRange,
    voltage_error: SourcedRange,
    current_error: SourcedRange,
}

impl Bench {
    fn run(peak_a: f64) -> Self {
        let branch = Section::Element {
            component: "блок".into(),
            element: "c".into(),
        };
        let terminals = ["p", "n"].map(|pin| Endpoint {
            component: "блок".into(),
            pin: pin.into(),
        });
        let circuit = Circuit::from_netlist(&parse(&netlist(peak_a)).unwrap()).unwrap();
        let mut sim = Transient::with_options(
            &circuit,
            STEP.as_secs(),
            TranOptions {
                uic: true,
                ..TranOptions::default()
            },
        )
        .unwrap();
        let mut time = Vec::new();
        let mut current = Vec::new();
        let mut voltage = Vec::new();
        for ns in (WINDOW.start.0..=WINDOW.end.0).step_by(STEP.0 as usize) {
            let t = Time(ns);
            sim.advance_to(t.as_secs()).unwrap();
            let i = sim.current("cblock").unwrap();
            let v = sim.voltage("terminal").unwrap();
            // Половина бюджета на ошибку точек. Ошибка линейного отрезка
            // ≤ h²·max|x″|/8 и проверяется отдельно в тесте.
            assert!((i - analytic_current(t.as_secs(), peak_a)).abs() < I_ERROR / 2.0);
            assert!((v - analytic_voltage(t.as_secs(), peak_a)).abs() < V_ERROR / 2.0);
            time.push(t);
            current.push(i);
            voltage.push(v);
        }
        let interpolation_factor = (STEP.as_secs() * TAU * FREQUENCY).powi(2) / 8.0;
        assert!(peak_a * interpolation_factor < I_ERROR / 2.0);
        assert!(peak_a * 5.0_f64.sqrt() * interpolation_factor < V_ERROR / 2.0);
        let values = BTreeMap::from([
            (Probe::Current(branch.clone()), current),
            (
                Probe::Voltage(terminals[0].clone(), terminals[1].clone()),
                voltage,
            ),
        ]);
        Self {
            branch,
            terminals,
            series: ElectricalSeries { time, values },
            frequency: exact(FREQUENCY),
            ambient: exact(300.0),
            theta: exact(2.0),
            thermal_ripple: exact(0.0),
            voltage_error: exact(V_ERROR),
            current_error: exact(I_ERROR),
        }
    }

    fn input(&self) -> CapacitorStressInput<'_> {
        CapacitorStressInput {
            scope: Scope::Reference,
            mode: &Mode::Normal,
            branch: &self.branch,
            terminals: &self.terminals,
            voltage_sense: CapacitorVoltageSense::Forward,
            window: WINDOW,
            series: Some(&self.series),
            voltage_error_v: Some(&self.voltage_error),
            current_error_a: Some(&self.current_error),
            frequency_hz: Some(&self.frequency),
            temperatures: CapacitorTemperatures {
                ambient_k: Some(&self.ambient),
                case_k: None,
                hotspot_k: None,
            },
            thermal: Some(CapacitorThermal::SteadyState {
                thermal_resistance_k_per_w: &self.theta,
                peak_above_mean_k: &self.thermal_ripple,
            }),
        }
    }
}

fn curve(cells: &[Option<SourcedRange>]) -> CapacitorCurve<'_> {
    CapacitorCurve {
        // Синтетическая огибающая стенда только в точке 1 кГц и 300 К.
        frequencies_hz: Some(&[FREQUENCY]),
        temperatures_k: &[300.0],
        temperature_basis: CapacitorTemperatureBasis::Ambient,
        cells,
    }
}

struct Ratings {
    voltage: SourcedRange,
    derating: [Option<SourcedRange>; 1],
    ripple: [Option<SourcedRange>; 1],
    esr: [Option<SourcedRange>; 1],
    temperature: SourcedRange,
}

impl Ratings {
    fn new(voltage: f64, ripple: f64, temperature: f64) -> Self {
        Self {
            voltage: exact(voltage),
            derating: [Some(exact(0.75))],
            ripple: [Some(exact(ripple))],
            esr: [Some(exact(1.0))],
            temperature: exact(temperature),
        }
    }

    fn spec(&self) -> CapacitorRatings<'_> {
        CapacitorRatings {
            rated_voltage_v: Some(&self.voltage),
            voltage_derating: Some(CapacitorCurve {
                frequencies_hz: None,
                ..curve(&self.derating)
            }),
            ripple_rms_a: Some(curve(&self.ripple)),
            esr_ohm: Some(curve(&self.esr)),
            max_hotspot_k: Some(&self.temperature),
        }
    }
}

fn contains(value: CapacitorBounds, expected: f64) {
    assert!(
        value.min <= expected && expected <= value.max,
        "{value:?}, ожидалось {expected}"
    );
}

fn analyzed<T>(value: &Analysis<T>) -> &T {
    let Analysis::Analyzed { value, .. } = value else {
        panic!("нет расчёта запаса")
    };
    value
}

#[test]
fn rc_block_violates_each_limit_and_remains_within_after_reduction() {
    let unsafe_bench = Bench::run(2.0);
    let corrected_bench = Bench::run(1.0);
    // В каждой строке только один критерий превышен. Широкие пределы
    // остальных критериев взяты выше ручных значений опасного варианта.
    for (selected, limits) in [
        Ratings::new(12.0, 2.0, 310.0),
        Ratings::new(16.0, 1.0, 310.0),
        Ratings::new(16.0, 2.0, 303.0),
    ]
    .iter()
    .enumerate()
    {
        for (bench, status, margins, power) in [
            (
                &unsafe_bench,
                Status::Fail,
                [-1.472135955, -0.414213562, -1.0],
                2.0,
            ),
            (
                &corrected_bench,
                Status::Pass,
                [0.763932022, 0.292893219, 2.0],
                0.5,
            ),
        ] {
            let result = assess_capacitor_stress(&bench.input(), &limits.spec()).unwrap();
            assert_eq!(result.status, status, "{}", result.reason);
            let e = &result.evidence;
            assert_eq!(e.code, "capacitor_stress");
            assert_eq!(e.branch, bench.branch);
            assert_eq!(e.mode, Mode::Normal);
            assert_eq!(e.window, WINDOW);
            assert_eq!(e.scope, Scope::Reference);
            for (index, limit) in [&e.voltage_v, &e.ripple_a, &e.hotspot_k]
                .into_iter()
                .enumerate()
            {
                let margin = analyzed(limit).margin;
                if index == selected {
                    contains(margin, margins[index]);
                    assert_eq!(margin.max < 0.0, status == Status::Fail);
                } else {
                    assert!(margin.min > 0.0);
                }
            }
            contains(*analyzed(&e.loss_w), power);
        }
    }
}

#[test]
fn ngspice_checks_branch_current_on_both_sides() {
    let ng = ngspice::find_ngspice().expect("для независимой сверки тока нужен ngspice");
    for peak_a in [2.0, 1.0] {
        let table = ngspice::run(
            &ng,
            &netlist(peak_a),
            ngspice::NgAnalysis::Tran {
                tstep: STEP.as_secs(),
                tstop: WINDOW.end.as_secs(),
                tstart: 0.0,
                tmax: STEP.as_secs(),
                uic: true,
            },
            &["i(vprobe)"],
        )
        .unwrap();
        let time = table.column("time").unwrap();
        let current = table.column("i(vprobe)").unwrap();
        let start = time.partition_point(|&t| t < WINDOW.start.as_secs());
        assert!(start > 0 && start < time.len() - 1);
        let mut peak_error = 0.0_f64;
        for (&t, &i) in time[start..].iter().zip(&current[start..]) {
            peak_error = peak_error.max((i - analytic_current(t, peak_a)).abs());
        }
        assert!(
            peak_error < I_ERROR,
            "максимальная ошибка тока ngspice {peak_error:e} А"
        );
        // Независимая квадратура трапециями для i² по адаптивной сетке ngspice.
        let first = WINDOW.start.as_secs();
        let last = WINDOW.end.as_secs();
        let t0 = time[start - 1];
        let t1 = time[start];
        let i0 = current[start - 1];
        let i1 = current[start];
        let at_start = i0 + (i1 - i0) * (first - t0) / (t1 - t0);
        let mut integral = (at_start.powi(2) + i1.powi(2)) * (t1 - first) / 2.0;
        for (t, i) in time[start..].windows(2).zip(current[start..].windows(2)) {
            integral += (i[0].powi(2) + i[1].powi(2)) * (t[1] - t[0]) / 2.0;
        }
        let ng_rms = (integral / (last - first)).sqrt();
        assert!((ng_rms - peak_a / 2.0_f64.sqrt()).abs() < RMS_REFERENCE_ERROR);
        let bench = Bench::run(peak_a);
        let limits = Ratings::new(12.0, 1.0, 303.0);
        let result = assess_capacitor_stress(&bench.input(), &limits.spec()).unwrap();
        let ours = analyzed(&result.evidence.ripple_a).observed;
        assert!((ng_rms - ours.min.midpoint(ours.max)).abs() < RMS_REFERENCE_ERROR);
        eprintln!(
            "Ipk={peak_a} А, ngspice RMS={ng_rms:.9} А, sim-process RMS={:.9} А, ошибка тока ngspice до {peak_error:.3e} А",
            ours.min.midpoint(ours.max)
        );
    }
}

#[test]
fn missing_or_estimated_data_does_not_prove_a_pass() {
    let bench = Bench::run(1.0);
    let ratings = Ratings::new(12.0, 1.0, 303.0);
    let input = bench.input();
    let spec = ratings.spec();
    let mut estimate = exact(12.0);
    estimate.provenance = Provenance::Estimate("Оценка без паспорта".into());
    let estimated_cells = [Some(estimate.clone())];
    let mut no_current = bench.series.clone();
    no_current
        .values
        .remove(&Probe::Current(bench.branch.clone()))
        .unwrap();
    for incomplete in [
        CapacitorStressInput {
            series: None,
            ..input
        },
        CapacitorStressInput {
            series: Some(&no_current),
            ..input
        },
        CapacitorStressInput {
            window: Interval {
                end: Time(WINDOW.end.0 + STEP.0),
                ..WINDOW
            },
            ..input
        },
        CapacitorStressInput {
            current_error_a: None,
            ..input
        },
        CapacitorStressInput {
            voltage_error_v: None,
            ..input
        },
        CapacitorStressInput {
            frequency_hz: None,
            ..input
        },
        CapacitorStressInput {
            thermal: None,
            ..input
        },
        CapacitorStressInput {
            temperatures: CapacitorTemperatures::default(),
            ..input
        },
        CapacitorStressInput {
            scope: Scope::Assembly,
            ..input
        },
    ] {
        let result = assess_capacitor_stress(&incomplete, &spec).unwrap();
        assert_eq!(result.status, Status::NotAnalyzed, "{}", result.reason);
        assert_eq!(result.evidence.code, "capacitor_stress");
    }
    for incomplete in [
        CapacitorRatings {
            rated_voltage_v: None,
            ..spec
        },
        CapacitorRatings {
            voltage_derating: None,
            ..spec
        },
        CapacitorRatings {
            max_hotspot_k: None,
            ..spec
        },
        CapacitorRatings {
            esr_ohm: None,
            ..spec
        },
        CapacitorRatings {
            ripple_rms_a: None,
            ..spec
        },
        CapacitorRatings {
            rated_voltage_v: Some(&estimate),
            ..spec
        },
        CapacitorRatings {
            ripple_rms_a: Some(curve(&estimated_cells)),
            ..spec
        },
        CapacitorRatings {
            esr_ohm: Some(CapacitorCurve {
                frequencies_hz: None,
                ..curve(&ratings.esr)
            }),
            ..spec
        },
    ] {
        let result = assess_capacitor_stress(&input, &incomplete).unwrap();
        assert_eq!(result.status, Status::NotAnalyzed, "{}", result.reason);
    }
    // Ошибка 1 В оставляет переход через 9 В возможным при отсчётах до 8,236 В.
    let hidden_peak = exact(1.0);
    let unresolved_peak = CapacitorStressInput {
        voltage_error_v: Some(&hidden_peak),
        ..input
    };
    let result = assess_capacitor_stress(&unresolved_peak, &spec).unwrap();
    assert_eq!(result.status, Status::NotAnalyzed);
    contains(analyzed(&result.evidence.voltage_v).margin, 0.0);

    let unsafe_bench = Bench::run(2.0);
    let mut sparse = unsafe_bench.series.clone();
    sparse.time = vec![WINDOW.start, WINDOW.end];
    for row in sparse.values.values_mut() {
        *row = vec![row[0], row[row.len() - 1]];
    }
    // 9 В между напряжением на краях периода 6 − √2 В и внутренним пиком 6 + 2√5 В.
    let peak = 6.0 + 2.0 * 5.0_f64.sqrt();
    let voltage_probe = Probe::Voltage(
        unsafe_bench.terminals[0].clone(),
        unsafe_bench.terminals[1].clone(),
    );
    assert!(sparse.values[&voltage_probe].iter().all(|&v| v < 9.0) && peak > 9.0);
    let sparse_input = CapacitorStressInput {
        series: Some(&sparse),
        voltage_error_v: None,
        current_error_a: None,
        ..unsafe_bench.input()
    };
    let result = assess_capacitor_stress(&sparse_input, &spec).unwrap();
    assert_eq!(result.status, Status::NotAnalyzed);
    assert!(matches!(
        result.evidence.voltage_v,
        Analysis::NotAnalyzed { .. }
    ));

    // Подтверждённое превышение напряжения сохраняется без соседнего канала тока.
    let input = CapacitorStressInput {
        current_error_a: None,
        ..unsafe_bench.input()
    };
    let result = assess_capacitor_stress(&input, &spec).unwrap();
    assert_eq!(result.status, Status::Fail);
    assert!(analyzed(&result.evidence.voltage_v).margin.max < 0.0);
    assert!(matches!(
        result.evidence.ripple_a,
        Analysis::NotAnalyzed { .. }
    ));
}

#[test]
fn tolerances_and_gaps_in_both_rating_dimensions_are_conservative() {
    let bench = Bench::run(1.0);
    let ratings = Ratings::new(12.0, 1.0, 303.0);
    // Два интервала по каждой оси. Диапазон опыта пересекает обе границы.
    let frequency = SourcedRange {
        min: 900.0,
        max: 1100.0,
        ..exact(FREQUENCY)
    };
    let case = SourcedRange {
        min: 295.0,
        max: 305.0,
        ..exact(300.0)
    };
    let hot = SourcedRange {
        min: 299.0,
        max: 301.0,
        ..exact(300.0)
    };
    let peak = exact(301.0);
    let input = CapacitorStressInput {
        frequency_hz: Some(&frequency),
        temperatures: CapacitorTemperatures {
            case_k: Some(&case),
            hotspot_k: Some(&hot),
            ..bench.input().temperatures
        },
        thermal: Some(CapacitorThermal::MeasuredPeak(&peak)),
        ..bench.input()
    };
    let current_cells = [2.0, 1.5, 1.25, 1.0].map(|v| Some(exact(v)));
    let esr_cells = [0.5, 1.0, 1.5, 2.0].map(|v| Some(exact(v)));
    let current = CapacitorCurve {
        frequencies_hz: Some(&[500.0, FREQUENCY, 2000.0]),
        temperatures_k: &[290.0, 300.0, 310.0],
        temperature_basis: CapacitorTemperatureBasis::Case,
        cells: &current_cells,
    };
    let esr = CapacitorCurve {
        temperature_basis: CapacitorTemperatureBasis::Hotspot,
        cells: &esr_cells,
        ..current
    };
    let spec = CapacitorRatings {
        ripple_rms_a: Some(current),
        esr_ohm: Some(esr),
        ..ratings.spec()
    };
    let result = assess_capacitor_stress(&input, &spec).unwrap();
    assert_eq!(result.status, Status::Pass);
    assert_eq!(
        analyzed(&result.evidence.ripple_a).limit,
        CapacitorBounds { min: 1.0, max: 2.0 }
    );
    // Ручная огибающая ESR при Irms² = 0,5 А².
    let loss = *analyzed(&result.evidence.loss_w);
    contains(loss, 0.25);
    contains(loss, 1.0);
    assert!(loss.max - loss.min < 0.75 + 4.0 * I_ERROR);
    let input_on_edge = CapacitorStressInput {
        frequency_hz: Some(&bench.frequency),
        ..input
    };
    let on_edge = assess_capacitor_stress(&input_on_edge, &spec).unwrap();
    assert_eq!(analyzed(&on_edge.evidence.ripple_a).limit.max, 1.5);

    let mut gap = current_cells.clone();
    gap[0] = None;
    let unknown = CapacitorRatings {
        ripple_rms_a: Some(CapacitorCurve {
            cells: &gap,
            ..current
        }),
        ..spec
    };
    assert_eq!(
        assess_capacitor_stress(&input, &unknown).unwrap().status,
        Status::NotAnalyzed
    );
    assert_eq!(
        assess_capacitor_stress(&input_on_edge, &unknown)
            .unwrap()
            .status,
        Status::Pass
    );
    let beyond_axis = exact(2001.0); // На 1 Гц за краем подтверждённой полосы.
    let outside = CapacitorStressInput {
        frequency_hz: Some(&beyond_axis),
        ..input
    };
    assert_eq!(
        assess_capacitor_stress(&outside, &spec).unwrap().status,
        Status::NotAnalyzed
    );

    let changed_peak = exact(302.0); // Противоречит огибающей горячей точки до 301 К.
    let inconsistent = CapacitorStressInput {
        thermal: Some(CapacitorThermal::MeasuredPeak(&changed_peak)),
        ..input
    };
    assert!(matches!(
        assess_capacitor_stress(&inconsistent, &spec),
        Err(ProcessError::InvalidInput(_))
    ));
}

#[test]
fn nonuniform_grid_uses_time_weighting_and_absolute_voltage_when_requested() {
    let bench = Bench::run(1.0);
    let limits = Ratings::new(12.0, 2.0, 310.0);
    let middle = Time(WINDOW.start.0 + PERIOD.0 / 4);
    let series = ElectricalSeries {
        time: vec![WINDOW.start, middle, WINDOW.end],
        values: BTreeMap::from([
            (
                Probe::Voltage(bench.terminals[0].clone(), bench.terminals[1].clone()),
                vec![-7.0, -10.0, 4.0],
            ),
            (Probe::Current(bench.branch.clone()), vec![-2.0, 2.0, 2.0]),
        ]),
    };
    let zero = exact(0.0); // Для отдельной арифметической сверки ряд между точками точно линейный.
    let input = CapacitorStressInput {
        series: Some(&series),
        current_error_a: Some(&zero),
        voltage_error_v: Some(&zero),
        ..bench.input()
    };
    let forward = assess_capacitor_stress(&input, &limits.spec()).unwrap();
    assert_eq!(forward.status, Status::Pass);
    // Средний квадрат рампы ±2 А = 4/3, постоянного участка = 4.
    // Веса 1/4 и 3/4. Средний квадрат за окно 10/3, при ESR=1 Ом потери 10/3 Вт.
    let mean_square = 10.0_f64 / 3.0;
    contains(
        analyzed(&forward.evidence.ripple_a).observed,
        mean_square.sqrt(),
    );
    contains(*analyzed(&forward.evidence.loss_w), mean_square);
    contains(analyzed(&forward.evidence.voltage_v).observed, 4.0);
    assert_eq!(forward.evidence.voltage_peak_sample, Some(WINDOW.end));
    let absolute = assess_capacitor_stress(
        &CapacitorStressInput {
            voltage_sense: CapacitorVoltageSense::Absolute,
            ..input
        },
        &limits.spec(),
    )
    .unwrap();
    assert_eq!(absolute.status, Status::Fail);
    contains(analyzed(&absolute.evidence.voltage_v).margin, -1.0); // 12 × 0,75 − 10 В.
    assert_eq!(absolute.evidence.voltage_peak_sample, Some(middle));
}

#[test]
fn nonfinite_inputs_and_invalid_time_are_errors_even_with_missing_data() {
    let bench = Bench::run(1.0);
    let ratings = Ratings::new(12.0, 1.0, 303.0);
    let input = bench.input();
    let spec = ratings.spec();
    fn error(input: &CapacitorStressInput<'_>, spec: &CapacitorRatings<'_>) {
        assert!(matches!(
            assess_capacitor_stress(input, spec),
            Err(ProcessError::InvalidInput(_))
        ));
    }
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut bad_current = bench.series.clone();
        bad_current
            .values
            .get_mut(&Probe::Current(bench.branch.clone()))
            .unwrap()[0] = value;
        error(
            &CapacitorStressInput {
                series: Some(&bad_current),
                ..input
            },
            &CapacitorRatings::default(),
        );
        let bad_range = SourcedRange {
            nominal: value,
            ..bench.frequency.clone()
        };
        error(
            &CapacitorStressInput {
                series: None,
                frequency_hz: Some(&bad_range),
                ..input
            },
            &spec,
        );
        let bad_axis = [value];
        let esr_ohm = Some(CapacitorCurve {
            temperatures_k: &bad_axis,
            ..curve(&ratings.esr)
        });
        error(&input, &CapacitorRatings { esr_ohm, ..spec });
        let bad_cells = [Some(exact(value))];
        error(
            &input,
            &CapacitorRatings {
                esr_ohm: Some(curve(&bad_cells)),
                ..spec
            },
        );
    }
    let mut repeated_tick = bench.series.clone();
    repeated_tick.time[1] = repeated_tick.time[0];
    error(
        &CapacitorStressInput {
            series: Some(&repeated_tick),
            ..input
        },
        &spec,
    );
}
