use sim_circuit::battery::{Cell, CellParams, RcParams, Table};
use sim_process::schematic::deep_discharge::*;
use sim_process::schematic::{Mode, Scope, Section, Status};
use sim_process::{Analysis, Interval, ProcessError, SourcedRange};
use sim_signal::Time;
use sim_world::Provenance;

// Стенд «блок»: две последовательные ячейки, OCV = 2,5 + 2·z, R0 = 0,1 Ом,
// R1 = 0,2 Ом, C1 = 5 Ф, τ = 1 с. Начальный SOC 0,5 / 0,8, ёмкость 10 / 20 Кл,
// поляризация 0 В. До размыкания нагрузка 1 А, после него ток защиты 0,01 А.
// Саморазряд ячеек 0,02 / 0,01 А влияет на заряд, минуя клеммы и RC.
// Предел 3 В. Срабатывание фиксируется в 1 с. У слабейшей z(t)=0,5−1,02·t/10,
// U(t)=2,5+2·z(t)−0,1−0,2·(1−exp(−t)). При задержках 1 / 0,25 с
// минимумы до размыкания 2,819067056647 / 3,002300959372 В, запас
// −0,180932943353 / +0,002300959372 В. К 3 с исправленная ячейка отдала
// 1,25·1,02 + 1,75·0,03 = 1,3275 Кл; даже при начальной поляризации
// хвоста её U ≥ 2,5 + 2·(0,5−1,3275/10) − 0,001 − 0,142699 = 3,090801 В.
const SEC: u64 = 1_000_000_000; // Наносекунд в секунде.
const COULOMBS_PER_AH: f64 = 3600.0; // Кулонов в А·ч.
const MODEL_ERROR_V: f64 = 1e-10; // Численный допуск синтетического стенда.
const TOL: f64 = 2e-10; // Численный допуск модели плюс округление ручных чисел.

fn window(start: u64, end: u64) -> Interval {
    Interval {
        start: Time(start),
        end: Time(end),
    }
}

fn source() -> Provenance {
    Provenance::Derived("Аналитические условия стенда блока".into())
}

fn bounds(min: f64, max: f64) -> SourcedRange {
    SourcedRange {
        min,
        nominal: min,
        max,
        provenance: source(),
    }
}

fn exact(x: f64) -> SourcedRange {
    bounds(x, x)
}

fn analyzed<T>(analysis: &Analysis<T>) -> &T {
    match analysis {
        Analysis::Analyzed { value, .. } => value,
        Analysis::NotAnalyzed { reason } => panic!("ожидался результат: {reason}"),
    }
}

fn near(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} вместо {expected}"
    );
}

struct Bench {
    branch: Section,
    params: CellParams,
    source: Provenance,
    zero: SourcedRange,
    rest: [SourcedRange; 1],
    temp: SourcedRange,
    capacity: [SourcedRange; 2],
    soc: [SourcedRange; 2],
    leak: [SourcedRange; 2],
    load: SourcedRange,
    idle: SourcedRange,
    limit: SourcedRange,
    error: SourcedRange,
}

impl Bench {
    fn new() -> Self {
        // Из исходной модели оставлены только метаданные и паспортные поля.
        // Используемые электрические таблицы и ёмкости заданы условиями выше.
        let mut params = CellParams::lg_m50();
        params.ocv = Table(vec![(0.0, 2.5), (1.0, 4.5)]);
        params.entropic = Table(vec![(0.0, 0.0), (1.0, 0.0)]);
        params.r0 = Table(vec![(0.0, 0.1), (1.0, 0.1)]);
        params.rc = vec![RcParams {
            r: Table(vec![(0.0, 0.2), (1.0, 0.2)]),
            c: 5.0,
        }];
        Self {
            branch: Section::Element {
                component: "блок".into(),
                element: "цепочка".into(),
            },
            params,
            source: source(),
            zero: exact(0.0),
            rest: [exact(0.0)],
            temp: exact(25.0), // Опорная температура таблиц sim-circuit::battery.
            capacity: [10.0, 20.0].map(|q| exact(q / COULOMBS_PER_AH)),
            soc: [exact(0.5), exact(0.8)],
            leak: [exact(0.02), exact(0.01)],
            load: exact(1.0),
            idle: exact(0.01),
            limit: exact(3.0),
            error: exact(MODEL_ERROR_V),
        }
    }

    fn cells(&self) -> [DeepDischargeCell<'_>; 2] {
        std::array::from_fn(|i| DeepDischargeCell {
            id: ["a", "b"][i],
            model: Some(DeepDischargeModel {
                params: &self.params,
                applicability: &self.source,
                voltage_error_v: Some(&self.error),
            }),
            capacity_ah: Some(&self.capacity[i]),
            initial_soc: Some(&self.soc[i]),
            temperature_c: Some(&self.temp),
            initial_polarization_v: Some(&self.rest),
            self_discharge_a: Some(&self.leak[i]),
            min_voltage_v: Some(&self.limit),
        })
    }

    fn segments(&self, off: u64, end: u64) -> [DeepDischargeSegment<'_>; 3] {
        [window(0, SEC), window(SEC, off), window(off, end)].map(|w| {
            let on = w.start < Time(off);
            DeepDischargeSegment {
                window: w,
                load_a: Some(if on { &self.load } else { &self.zero }),
                protection_a: Some(if on { &self.zero } else { &self.idle }),
            }
        })
    }

    fn input<'a>(
        &'a self,
        cells: &'a [DeepDischargeCell<'a>],
        segments: &'a [DeepDischargeSegment<'a>],
        off: u64,
        end: u64,
        delay: &'a SourcedRange,
    ) -> DeepDischargeInput<'a> {
        DeepDischargeInput {
            scope: Scope::Reference,
            mode: &Mode::Normal,
            branch: &self.branch,
            window: window(0, end),
            cells: Some(cells),
            series_applicability: Some(&self.source),
            segments: Some(segments),
            cutoff: Some(DeepDischargeCutoff {
                detected_at: Time(SEC),
                disconnected_at: Time(off),
                delay_s: Some(delay),
                source: &self.source,
            }),
        }
    }
}

#[test]
fn late_cutoff_fails_the_weakest_cell_and_early_cutoff_passes() {
    let bench = Bench::new();
    let cells = bench.cells();
    let end = 3 * SEC;
    for (off, delay, expected, status) in [
        (2 * SEC, 1.0, -0.180_932_943_353, Status::Fail),
        (SEC + SEC / 4, 0.25, 0.002_300_959_372, Status::Pass),
    ] {
        let segments = bench.segments(off, end);
        let delay = exact(delay);
        let result =
            assess_deep_discharge(&bench.input(&cells, &segments, off, end, &delay)).unwrap();
        assert_eq!(result.status, status, "{result:?}");
        assert_eq!(result.evidence.code, "deep_discharge");
        assert_eq!(result.evidence.branch, bench.branch);
        assert_eq!(result.evidence.mode, Mode::Normal);
        assert_eq!(result.evidence.window, window(0, end));
        assert_eq!(result.evidence.limiting_cell.as_deref(), Some("a"));
        let cutoff = analyzed(&result.evidence.cutoff);
        assert_eq!(cutoff.disconnected_at, Time(off));
        assert_eq!(cutoff.observed_delay, Time(off - SEC));

        let weak = &result.evidence.cells[0];
        let margin = analyzed(&weak.margin_v);
        near(margin.min, expected, TOL);
        near(margin.max, expected, TOL);
        assert!(analyzed(&result.evidence.cells[1].margin_v).min > -expected);
        assert_eq!(
            weak.violation.map(|v| v.time),
            (status == Status::Fail).then_some(Time(off))
        );
        if let Some(violation) = weak.violation {
            near(
                violation.voltage_v.max - violation.limit_v.min,
                expected,
                TOL,
            );
        }
        let t_on = Time(off).as_secs();
        let q_balance = 1.02 * t_on + 0.03 * (3.0 - t_on);
        let charge = analyzed(&weak.removed_charge_ah);
        near(charge.min * COULOMBS_PER_AH, q_balance, TOL);
        near(charge.max * COULOMBS_PER_AH, q_balance, TOL);

        // Отдельный штатный расчёт RC: ток саморазряда учитываем через
        // линейную поправку OCV по независимому зарядовому балансу.
        let mut params = bench.params.clone();
        params.capacity_ah = bench.capacity[0].nominal;
        let mut cell = Cell::with_soc(params, bench.soc[0].nominal);
        let _ = cell.step(1.0, t_on, 25.0);
        near(
            cell.terminal_voltage() - 2.0 * (0.02 * t_on / 10.0) - 3.0,
            expected,
            TOL,
        );
    }
}

#[test]
fn stage6_first_tick_matches_circuit_and_manual_reference() {
    let bench = Bench::new();
    let params = CellParams::lg_m50();
    let limit_source = Provenance::Datasheet("LG Chem LRB-PS-CY18.2Wh-M50, предел разряда".into());
    let limit = SourcedRange {
        provenance: limit_source,
        ..exact(params.v_min)
    };
    let capacity = exact(params.capacity_ah);
    let end = SEC / 1000; // 1 мс, первый такт корпуса этапа 6.
    let segments = [DeepDischargeSegment {
        window: window(0, end),
        load_a: Some(&bench.load),
        protection_a: Some(&bench.zero),
    }];
    // devices/stage6/acceptance.toml: OCV Chen 2020 − I·R0 − I·R1·(1−exp(−t/τ)).
    const STAGE6_TOL_V: f64 = 1e-5; // Допуск первого такта из той же записи.
    for (initial, expected, status) in [
        (0.0, 2.458_363_861_639_222, Status::Fail),
        (0.5, 3.682_962_327_400_720_6, Status::NotAnalyzed),
    ] {
        let soc = exact(initial);
        let cells = [DeepDischargeCell {
            model: Some(DeepDischargeModel {
                params: &params,
                applicability: &bench.source,
                voltage_error_v: Some(&bench.error),
            }),
            capacity_ah: Some(&capacity),
            initial_soc: Some(&soc),
            min_voltage_v: Some(&limit),
            self_discharge_a: Some(&bench.zero),
            ..bench.cells()[0]
        }];
        let delay = exact(0.0);
        let mut input = bench.input(&cells, &segments, end, end, &delay);
        input.cutoff = None;
        let result = assess_deep_discharge(&input).unwrap();
        assert_eq!(result.status, status, "{result:?}");
        let final_v = analyzed(&result.evidence.cells[0].final_model_voltage_v);
        let mut cell = Cell::with_soc(params.clone(), initial);
        let _ = cell.step(1.0, Time(end).as_secs(), 25.0);
        near(cell.terminal_voltage(), expected, STAGE6_TOL_V);
        near(final_v.min, expected, STAGE6_TOL_V);
        near(final_v.max, expected, STAGE6_TOL_V);
        assert_eq!(final_v.max < limit.min, initial == 0.0);
        assert_eq!(limit.min, 2.50);
        if initial == 0.0 {
            assert_eq!(
                result.evidence.cells[0].violation.map(|v| v.time),
                Some(Time(0))
            );
            assert!(matches!(
                result.evidence.cells[0].margin_v,
                Analysis::NotAnalyzed { .. }
            ));
        }
    }
}

#[test]
fn residual_discharge_can_cross_limit_after_a_confirmed_cutoff() {
    let bench = Bench::new();
    let cells = bench.cells();
    let off = SEC + SEC / 4;
    let end = 60 * SEC; // Продлённое окно хранения стенда.
    let segments = bench.segments(off, end);
    let delay = exact(0.25);
    let result = assess_deep_discharge(&bench.input(&cells, &segments, off, end, &delay)).unwrap();
    let weak = &result.evidence.cells[0];
    assert_eq!(result.status, Status::Fail);
    assert_eq!(result.evidence.code, DEEP_DISCHARGE_CODE);
    assert_eq!(weak.violation.map(|v| v.time), Some(Time(end)));
    assert_eq!(analyzed(&result.evidence.cutoff).disconnected_at, Time(off));
    // q = 1,25·1,02 + 58,75·0,03 = 3,0375 Кл. Поляризация за
    // 58,75τ стремится к 0,002 В. U = 2,5 + 2·(0,5−0,30375)−0,001−0,002 = 2,8895 В.
    near(
        analyzed(&weak.removed_charge_ah).max * COULOMBS_PER_AH,
        3.0375,
        TOL,
    );
    near(analyzed(&weak.final_model_voltage_v).max, 2.8895, TOL);
}

#[test]
fn missing_or_estimated_inputs_never_certify_the_assembly() {
    let bench = Bench::new();
    let off = SEC + SEC / 4;
    let end = 3 * SEC;
    let segments = bench.segments(off, end);
    let delay = exact(0.25);
    let cells = bench.cells();
    let base = bench.input(&cells, &segments, off, end, &delay);
    assert_eq!(assess_deep_discharge(&base).unwrap().status, Status::Pass);

    let estimate = Provenance::Estimate("Неподтверждённые данные".into());
    let estimated_limit = SourcedRange {
        provenance: estimate.clone(),
        ..bench.limit.clone()
    };
    let estimated_curve = Some(DeepDischargeModel {
        applicability: &estimate,
        ..cells[0].model.unwrap()
    });
    for missing in [
        DeepDischargeCell {
            model: estimated_curve,
            ..cells[0]
        },
        DeepDischargeCell {
            min_voltage_v: Some(&estimated_limit),
            ..cells[0]
        },
        DeepDischargeCell {
            min_voltage_v: None,
            ..cells[0]
        },
        DeepDischargeCell {
            capacity_ah: None,
            ..cells[0]
        },
        DeepDischargeCell {
            initial_soc: None,
            ..cells[0]
        },
        DeepDischargeCell {
            self_discharge_a: None,
            ..cells[0]
        },
        DeepDischargeCell {
            initial_polarization_v: None,
            ..cells[0]
        },
        DeepDischargeCell {
            model: Some(DeepDischargeModel {
                voltage_error_v: None,
                ..cells[0].model.unwrap()
            }),
            ..cells[0]
        },
    ] {
        let partial = [missing, cells[1]];
        let result = assess_deep_discharge(&DeepDischargeInput {
            cells: Some(&partial),
            ..base
        })
        .unwrap();
        assert_eq!(result.status, Status::NotAnalyzed, "{result:?}");
        assert!(matches!(
            result.evidence.cells[0].margin_v,
            Analysis::NotAnalyzed { .. }
        ));
        assert!(analyzed(&result.evidence.cells[1].margin_v).min > 0.0);
    }

    let mut partial_trace = segments;
    partial_trace[2].protection_a = None;
    let wrong_delay = exact(1.0); // Задержка опасного опыта вместо наблюдаемых 0,25 с.
    let mut load_remains = segments;
    load_remains[2].load_a = Some(&bench.idle);
    for input in [
        DeepDischargeInput {
            cells: None,
            ..base
        },
        DeepDischargeInput {
            series_applicability: None,
            ..base
        },
        DeepDischargeInput {
            scope: Scope::Assembly,
            ..base
        },
        DeepDischargeInput {
            segments: Some(&partial_trace),
            ..base
        },
        DeepDischargeInput {
            segments: Some(&segments[..2]),
            ..base
        },
        DeepDischargeInput {
            segments: Some(&load_remains),
            ..base
        },
        DeepDischargeInput {
            cutoff: None,
            ..base
        },
        DeepDischargeInput {
            cutoff: Some(DeepDischargeCutoff {
                delay_s: Some(&wrong_delay),
                ..base.cutoff.unwrap()
            }),
            ..base
        },
        DeepDischargeInput {
            cutoff: Some(DeepDischargeCutoff {
                delay_s: None,
                ..base.cutoff.unwrap()
            }),
            ..base
        },
    ] {
        assert_eq!(
            assess_deep_discharge(&input).unwrap().status,
            Status::NotAnalyzed
        );
    }
}

#[test]
fn uncertainty_and_unresolved_intersample_dip_stay_open() {
    let mut bench = Bench::new();
    let off = SEC + SEC / 4;
    let end = 3 * SEC;
    // Постоянная OCV 3,5 В. При z=0,45 (t=0,05·10/1,02≈0,4902 с)
    // R0=0,6 Ом и U<2,9 В. В обеих границах 0…1 с R0=0,1 Ом и U>3 В.
    bench.params.ocv = Table(vec![(0.0, 3.5), (1.0, 3.5)]);
    bench.params.r0 = Table(vec![
        (0.0, 0.1),
        (0.4, 0.1),
        (0.45, 0.6),
        (0.5, 0.1),
        (1.0, 0.1),
    ]);
    let cells = bench.cells();
    let segments = bench.segments(off, end);
    let delay = exact(0.25);
    let result = assess_deep_discharge(&bench.input(&cells, &segments, off, end, &delay)).unwrap();
    assert_eq!(result.status, Status::NotAnalyzed);
    let margin = analyzed(&result.evidence.cells[0].margin_v);
    assert!(margin.min < 0.0 && margin.max > 0.0);
    assert_eq!(result.evidence.cells[0].violation.map(|v| v.time), None);

    let close_to_knot = SEC * 49 / 100; // 0,49 с, почти в узле R0.
    let split = [
        DeepDischargeSegment {
            window: window(0, close_to_knot),
            ..segments[0]
        },
        DeepDischargeSegment {
            window: window(close_to_knot, SEC),
            ..segments[0]
        },
        segments[1],
        segments[2],
    ];
    let resolved = assess_deep_discharge(&bench.input(&cells, &split, off, end, &delay)).unwrap();
    assert_eq!(resolved.status, Status::Fail);
    assert_eq!(
        resolved.evidence.cells[0].violation.map(|v| v.time),
        Some(Time(close_to_knot))
    );

    let bench = Bench::new();
    let segments = bench.segments(off, end);
    // Предел 3,000–3,005 В пересекает вычисленный минимум 3,002301 В.
    let limit = bounds(3.0, 3.005);
    let cells = [
        DeepDischargeCell {
            min_voltage_v: Some(&limit),
            ..bench.cells()[0]
        },
        bench.cells()[1],
    ];
    let result = assess_deep_discharge(&bench.input(&cells, &segments, off, end, &delay)).unwrap();
    assert_eq!(result.status, Status::NotAnalyzed);
    let margin = analyzed(&result.evidence.cells[0].margin_v);
    assert!(margin.min < 0.0 && margin.max > 0.0);
}

#[test]
fn confirmed_violation_survives_an_open_tail_and_unavailable_second_cell() {
    let bench = Bench::new();
    let mut cells = bench.cells();
    cells[1].model = None;
    let off = 2 * SEC;
    let end = 3 * SEC;
    let mut segments = bench.segments(off, end);
    segments[2].protection_a = None;
    let delay = exact(1.0);
    let result = assess_deep_discharge(&bench.input(&cells, &segments, off, end, &delay)).unwrap();
    assert_eq!(result.status, Status::Fail);
    assert_eq!(
        result.evidence.cells[0].violation.map(|v| v.time),
        Some(Time(off))
    );
    assert!(matches!(
        result.evidence.cutoff,
        Analysis::NotAnalyzed { .. }
    ));
    assert!(matches!(
        result.evidence.cells[1].margin_v,
        Analysis::NotAnalyzed { .. }
    ));
}

#[test]
fn non_finite_inputs_are_errors_even_if_the_calculation_is_already_open() {
    let bench = Bench::new();
    let cells = bench.cells();
    let off = SEC + SEC / 4;
    let end = 3 * SEC;
    let delay = exact(0.25);
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let bad_value = exact(bad);
        let mut segments = bench.segments(off, end);
        segments[1].load_a = None;
        segments[2].protection_a = Some(&bad_value);
        let input = bench.input(&cells, &segments, off, end, &delay);
        assert!(matches!(
            assess_deep_discharge(&input),
            Err(ProcessError::InvalidInput(_))
        ));

        let missing_trace = [];
        let mut params = bench.params.clone();
        params.rc[0].r.0[0].1 = bad;
        let cells = [DeepDischargeCell {
            model: Some(DeepDischargeModel {
                params: &params,
                ..cells[0].model.unwrap()
            }),
            ..cells[0]
        }];
        let input = bench.input(&cells, &missing_trace, off, end, &delay);
        assert!(matches!(
            assess_deep_discharge(&input),
            Err(ProcessError::InvalidInput(_))
        ));
        let cells = bench.cells();
        let input = bench.input(&cells, &missing_trace, off, end, &bad_value);
        assert!(matches!(
            assess_deep_discharge(&input),
            Err(ProcessError::InvalidInput(_))
        ));
    }
}

#[test]
fn initial_polarization_from_charge_can_cross_zero_during_discharge() {
    let bench = Bench::new();
    let initial = [exact(-0.2)]; // Противоположный разряду знак после предыдущего заряда.
    let cells = [
        DeepDischargeCell {
            initial_polarization_v: Some(&initial),
            ..bench.cells()[0]
        },
        bench.cells()[1],
    ];
    let off = SEC + SEC / 4;
    let end = 3 * SEC;
    let segments = bench.segments(off, end);
    let delay = exact(0.25);
    let result = assess_deep_discharge(&bench.input(&cells, &segments, off, end, &delay)).unwrap();
    assert_eq!(result.status, Status::Pass);
    // Vp(1,25) = 0,2 − 0,4·exp(−1,25) > 0. На хвосте равновесие 0,002 В.
    let vp_off = 0.2 - 0.4 * (-1.25_f64).exp();
    let vp_final = 0.002 + (vp_off - 0.002) * (-1.75_f64).exp();
    let predicted = 2.5 + 2.0 * (0.5 - 1.3275 / 10.0) - 0.001 - vp_final;
    let voltage = analyzed(&result.evidence.cells[0].final_model_voltage_v);
    near(voltage.min, predicted, TOL);
    near(voltage.max, predicted, TOL);
}

#[test]
fn a_curve_that_does_not_cover_the_discharge_cannot_be_clamped_for_proof() {
    let mut bench = Bench::new();
    // Та же прямая OCV, обрезанная при SOC 0,4. Исправленная ячейка
    // к моменту снятия нагрузки имеет SOC 0,3725, вне области подгонки.
    bench.params.ocv = Table(vec![(0.4, 3.3), (1.0, 4.5)]);
    let cells = bench.cells();
    let off = SEC + SEC / 4;
    let end = 3 * SEC;
    let segments = bench.segments(off, end);
    let delay = exact(0.25);
    let result = assess_deep_discharge(&bench.input(&cells, &segments, off, end, &delay)).unwrap();
    assert_eq!(result.status, Status::NotAnalyzed);
    assert!(matches!(
        result.evidence.cells[0].minimum_voltage_v,
        Analysis::NotAnalyzed { .. }
    ));
    assert!(matches!(
        result.evidence.cells[0].final_model_voltage_v,
        Analysis::NotAnalyzed { .. }
    ));
    near(
        analyzed(&result.evidence.cells[0].removed_charge_ah).max * COULOMBS_PER_AH,
        1.3275,
        TOL,
    );
}
