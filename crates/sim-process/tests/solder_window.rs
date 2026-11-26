//! Пределы и ручные запасы синтетического стенда заданы до реализации оценщика.

use std::num::NonZeroU32;

use sim_process::work::{
    heat::solder_network::{SolderContact, SolderNetworkInput, SolderNetworkSpec},
    solder_window::{
        SolderBounds, SolderBridgeLimit, SolderCriterion, SolderWettingTime, SolderWindowInput,
        SolderWindowLimits, SolderWindowStatus, assess_solder_window,
    },
};
use sim_process::{Analysis, ProcessError, SourcedRange};
use sim_world::Provenance;

fn range(min: f64, nominal: f64, max: f64) -> SourcedRange {
    SourcedRange {
        min,
        nominal,
        max,
        provenance: Provenance::Datasheet("математическая постановка стенда в этом тесте".into()),
    }
}

fn point(value: f64) -> Option<SourcedRange> {
    Some(range(value, value, value))
}

fn bench(amplitude: f64, duration: f64, dose_mm3: f64) -> SolderWindowInput {
    // Постановка из devices/electrical/depth_t5/expected.md. Для заданных C и G
    // вектор [3, 2, 3, 1] — тепловая мода с постоянной времени 1 с.
    // При 21,6 Вт баланс даёт T∞ = [708, 528, 528, 408] К.
    let steady = [708.0, 528.0, 528.0, 408.0];
    let mode = [3.0, 2.0, 3.0, 1.0];
    let initial_k = std::array::from_fn(|i| point(steady[i] - amplitude * mode[i]));
    SolderWindowInput {
        network: SolderNetworkSpec {
            capacity_j_k: [0.04, 0.015, 0.01, 0.02].map(point).into(),
            initial_k: initial_k.into(),
            pad_lead_w_k: point(0.03),
            pad_copper_w_k: point(0.18),
            copper_polygon_w_k: point(0.20),
            polygon_k: point(300.0),
            max_power_w: point(21.6),
        },
        thermal: SolderNetworkInput {
            contact: Some(SolderContact::Touching {
                conductance_w_k: point(0.12),
            }),
            requested_power_w: point(27.0), // Запрос 5/4 лимита.
        },
        force_n: point(1.0), // Проводимость калибрована в точке 1 Н.
        duration_s: point(duration),
        pitch_m: point(0.001),
        dose_m3: point(dose_mm3 * 1e-9), // Перевод мм³ в м³.
        limits: SolderWindowLimits {
            wetting_k: point(500.0),
            max_time_to_wet_s: point(1.0),
            liquidus_k: point(493.15), // 220 °C + 273,15 по паспорту сплава стенда.
            min_above_liquidus_s: point(0.5),
            max_above_liquidus_s: point(1.2),
            max_pad_k: point(515.0),
            max_component_k: point(520.0),
            min_volume_m3: point(2.5e-10),
            bridge: Some(SolderBridgeLimit {
                pitch_m: point(0.001).unwrap(),
                max_volume_m3: point(5e-10).unwrap(),
            }),
        },
    }
}

fn steps(n: u32) -> NonZeroU32 {
    NonZeroU32::new(n).unwrap()
}

fn analyzed<T>(result: Analysis<T>) -> T {
    match result {
        Analysis::Analyzed { value, .. } => value,
        Analysis::NotAnalyzed { reason } => panic!("{reason}"),
    }
}

fn includes(bounds: SolderBounds, expected: f64, tolerance: f64) {
    assert!(
        bounds.lower <= expected + tolerance,
        "{bounds:?}, {expected}"
    );
    assert!(
        bounds.upper >= expected - tolerance,
        "{bounds:?}, {expected}"
    );
}

#[test]
fn five_pairs_change_only_the_target_criterion() {
    use SolderCriterion::*;

    // Источник чисел — формулы и пределы синтетического стенда выше.
    // Смачивание ln(2A/28); ликвидус d − ln(70/34,85); площадка 528 − 70 exp(−d).
    // Доза = (0,5 мм)² × длина заготовки, предельные длины — один и два шага по 1 мм.
    let pairs = [
        (
            MaxTimeToWet,
            (42.0, 1.45, 0.375, -0.098612289),
            (35.0, 1.45, 0.375, 0.083709268),
        ),
        (
            MinAboveLiquidus,
            (35.0, 1.15, 0.375, -0.047442105),
            (35.0, 1.25, 0.375, 0.052557895),
        ),
        (
            MaxPadTemperature,
            (35.0, 1.77, 0.375, -1.076690782),
            (35.0, 1.63, 0.375, 0.715070189),
        ),
        (
            Bridge,
            (35.0, 1.45, 0.525, -2.5e-11),
            (35.0, 1.45, 0.475, 2.5e-11),
        ),
        (
            MinVolume,
            (35.0, 1.45, 0.225, -2.5e-11),
            (35.0, 1.45, 0.275, 2.5e-11),
        ),
    ];

    for (criterion, unsafe_case, fixed) in pairs {
        for ((amplitude, duration, dose, margin), dangerous) in
            [(unsafe_case, true), (fixed, false)]
        {
            let input = bench(amplitude, duration, dose);
            // 1024 интервала: шаг меньше 1,8 мс во всех профилях стенда.
            let report = assess_solder_window(&input, steps(1024)).unwrap();
            let target = report.check(criterion);
            let bounds = target.margin.unwrap();
            // Запасы в таблице округлены до 9 знаков, объёмы заданы точно в десятичной записи.
            let tolerance = if matches!(criterion, Bridge | MinVolume) {
                1e-25
            } else {
                5e-10
            };
            includes(bounds, margin, tolerance);
            let expected = if dangerous {
                SolderWindowStatus::Fail
            } else {
                SolderWindowStatus::Pass
            };
            assert_eq!(target.status, expected);
            assert_eq!(report.status(), expected);
            assert!(
                report.checks().iter().all(|check| {
                    check.criterion == criterion || check.status == SolderWindowStatus::Pass
                }),
                "{:?}",
                report.checks()
            );

            // Второй расчёт проходит по четырём тепловым балансам методом RK4.
            let independent = reference(&input, duration);
            let metrics = analyzed(report.thermal);
            let SolderWettingTime::Reached(wetting) = analyzed(metrics.time_to_wet_s) else {
                panic!("порог смачивания должен пересекаться");
            };
            // 2 мкс покрывают интерполяцию пересечения в независимой сетке.
            includes(wetting, independent.0.unwrap(), 2e-6);
            includes(analyzed(metrics.above_liquidus_s), independent.1, 2e-6);
            // 1 мкК покрывает округление при независимом интегрировании.
            includes(metrics.peak_pad_k, independent.2, 1e-6);
            includes(metrics.peak_component_k, independent.3, 1e-6);
            let decay = (-duration).exp();
            for (bounds, mode) in [(metrics.peak_pad_k, 2.0), (metrics.peak_component_k, 3.0)] {
                let peak = 528.0 - amplitude * mode * decay;
                let rise = amplitude * mode * (1.0 - decay);
                // ТЗ В требует не более 5% по приросту. Проверяется вся огибающая.
                assert!(bounds.lower >= peak - 0.05 * rise, "{bounds:?}, {peak}");
                assert!(bounds.upper <= peak + 0.05 * rise, "{bounds:?}, {peak}");
            }
            let geometric_dose = (0.5_f64 * 1e-3).powi(2) * (dose / 0.25 * 1e-3);
            if criterion == Bridge {
                includes(
                    bounds,
                    (0.5_f64 * 1e-3).powi(2) * (2.0 * 1e-3) - geometric_dose,
                    1e-25,
                );
            } else if criterion == MinVolume {
                includes(
                    bounds,
                    geometric_dose - (0.5_f64 * 1e-3).powi(2) * 1e-3,
                    1e-25,
                );
            }
        }
    }
}

// Независимые балансы узлов. RK4 на 4096 равных интервалах; пересечения
// оцениваются секущей между соседними независимыми температурами.
fn reference(input: &SolderWindowInput, duration: f64) -> (Option<f64>, f64, f64, f64) {
    let network = &input.network;
    let number = |x: &Option<SourcedRange>| x.as_ref().unwrap().nominal;
    let nodes = |x: &sim_process::work::heat::solder_network::SolderNodes<_>| {
        [&x.tip, &x.pad, &x.lead, &x.copper].map(number)
    };
    let [ct, cp, cl, cc] = nodes(&network.capacity_j_k);
    let Some(SolderContact::Touching { conductance_w_k }) = &input.thermal.contact else {
        panic!("нет контакта в независимом стенде");
    };
    let gtp = number(conductance_w_k);
    let gpl = number(&network.pad_lead_w_k);
    let gpc = number(&network.pad_copper_w_k);
    let gcg = number(&network.copper_polygon_w_k);
    let polygon = number(&network.polygon_k);
    let power = number(&input.thermal.requested_power_w).min(number(&network.max_power_w));
    let derivative = |[t, p, l, c]: [f64; 4]| {
        let tp = (t - p) * gtp;
        let pl = (p - l) * gpl;
        let pc = (p - c) * gpc;
        [
            (power - tp) / ct,
            (tp - pl - pc) / cp,
            pl / cl,
            (pc - (c - polygon) * gcg) / cc,
        ]
    };
    let advance =
        |x: [f64; 4], slope: [f64; 4], dt: f64| std::array::from_fn(|i| x[i] + slope[i] * dt);
    let mut x = nodes(&network.initial_k);
    let n = 4096;
    let dt = duration / f64::from(n);
    let wet = number(&input.limits.wetting_k);
    let liquidus = number(&input.limits.liquidus_k);
    let mut first_wet = (x[1] >= wet).then_some(0.0);
    let mut above = 0.0;
    let mut peak_pad = x[1];
    let mut peak_component = x[2];
    for step in 0..n {
        let a = derivative(x);
        let b = derivative(advance(x, a, dt / 2.0));
        let c = derivative(advance(x, b, dt / 2.0));
        let d = derivative(advance(x, c, dt));
        let next =
            std::array::from_fn(|i| x[i] + dt / 6.0 * (a[i] + 2.0 * b[i] + 2.0 * c[i] + d[i]));
        if first_wet.is_none() && x[1] < wet && next[1] >= wet {
            first_wet = Some(dt * (f64::from(step) + (wet - x[1]) / (next[1] - x[1])));
        }
        let top = x[1].max(next[1]);
        let bottom = x[1].min(next[1]);
        if top > liquidus {
            above += if bottom >= liquidus {
                dt
            } else {
                dt * (top - liquidus) / (top - bottom)
            };
        }
        peak_pad = peak_pad.max(next[1]);
        peak_component = peak_component.max(next[2]);
        x = next;
    }
    (first_wet, above, peak_pad, peak_component)
}

#[test]
fn upper_time_and_component_temperature_have_independent_limits() {
    use SolderCriterion::*;
    let mut input = bench(35.0, 1.45, 0.375);
    // d − ln(70/34,85) = 0,752557895 с. Пик вывода
    // 528 − 105 exp(−1,45) = 503,370104265 К, отдельно от площадки.
    let independent = reference(&input, 1.45);
    for (time_limit, component_limit, status) in [
        (0.7, 502.0, SolderWindowStatus::Fail),
        (0.8, 505.0, SolderWindowStatus::Pass),
    ] {
        input.limits.max_above_liquidus_s = point(time_limit);
        input.limits.max_component_k = point(component_limit);
        let report = assess_solder_window(&input, steps(1024)).unwrap();
        for (criterion, margin) in [
            (MaxAboveLiquidus, time_limit - independent.1),
            (MaxComponentTemperature, component_limit - independent.3),
        ] {
            let check = report.check(criterion);
            assert_eq!(check.status, status);
            includes(check.margin.unwrap(), margin, 2e-6);
        }
        assert_eq!(
            report.check(MaxPadTemperature).status,
            SolderWindowStatus::Pass
        );
    }
}

#[test]
fn extrema_and_crossings_inside_one_step_cannot_pass_from_endpoints() {
    use SolderCriterion::*;
    let mut input = bench(35.0, 20.0, 0.375);
    // Остывание через полигон: C=1, G=1 вдоль цепи жало–площадка–медь–полигон.
    // Нагрева нет, начальный избыток 60 К только у жала, холодный конец 300 К.
    input.network.capacity_j_k = [1.0; 4].map(point).into();
    input.network.initial_k = [360.0, 300.0, 300.0, 300.0].map(point).into();
    input.network.pad_lead_w_k = point(0.0);
    input.network.pad_copper_w_k = point(1.0);
    input.network.copper_polygon_w_k = point(1.0);
    input.thermal.contact = Some(SolderContact::Touching {
        conductance_w_k: point(1.0),
    });
    input.thermal.requested_power_w = point(0.0);
    input.limits.wetting_k = point(312.0);
    input.limits.liquidus_k = point(310.0);
    input.limits.max_pad_k = point(315.0);
    let independent = reference(&input, 20.0);
    assert!(independent.2 > 315.0);
    assert!(independent.1 > 1.2);
    let coarse = assess_solder_window(&input, steps(1)).unwrap();
    assert_ne!(
        coarse.check(MaxPadTemperature).status,
        SolderWindowStatus::Pass
    );
    assert_ne!(
        coarse.check(MaxAboveLiquidus).status,
        SolderWindowStatus::Pass
    );
    let refined = assess_solder_window(&input, steps(2048)).unwrap();
    assert_eq!(
        refined.check(MaxPadTemperature).status,
        SolderWindowStatus::Fail
    );
    assert_eq!(
        refined.check(MaxAboveLiquidus).status,
        SolderWindowStatus::Fail
    );
    let metrics = analyzed(refined.thermal);
    // У независимого RK4 пик берётся по сетке 4,9 мс, запас сравнения 1 мК.
    includes(metrics.peak_pad_k, independent.2, 1e-3);
    includes(analyzed(metrics.above_liquidus_s), independent.1, 2e-5);
}

#[test]
fn missing_or_unresolved_inputs_keep_checks_open_and_do_not_hide_failure() {
    use SolderCriterion::*;
    let baseline = || bench(35.0, 1.45, 0.375);
    let unknown = |input: &SolderWindowInput, criterion| {
        let report = assess_solder_window(input, steps(1024)).unwrap();
        assert!(matches!(&report.check(criterion).status,
            SolderWindowStatus::NotAnalyzed { reason } if !reason.is_empty()));
        assert!(matches!(
            report.status(),
            SolderWindowStatus::NotAnalyzed { .. }
        ));
    };

    let mut input = baseline();
    input.thermal.contact = None;
    unknown(&input, MaxTimeToWet);
    input.thermal.contact = Some(SolderContact::Separated);
    unknown(&input, MaxTimeToWet);
    input.thermal.contact = Some(SolderContact::Touching {
        conductance_w_k: Some(range(0.11, 0.12, 0.12)),
    });
    unknown(&input, MaxPadTemperature);

    let mut input = baseline();
    input
        .limits
        .min_above_liquidus_s
        .as_mut()
        .unwrap()
        .provenance = Provenance::Estimate("не подтверждено на стенде".into());
    unknown(&input, MinAboveLiquidus);
    input
        .limits
        .min_above_liquidus_s
        .as_mut()
        .unwrap()
        .provenance = Provenance::Datasheet(String::new());
    unknown(&input, MinAboveLiquidus);
    input.limits.liquidus_k = None;
    unknown(&input, MaxAboveLiquidus);
    input.limits.wetting_k = None;
    unknown(&input, MaxTimeToWet);

    let mut input = baseline();
    input.dose_m3 = None;
    unknown(&input, Bridge);
    input.dose_m3 = Some(range(4.5e-10, 5e-10, 5.5e-10));
    unknown(&input, Bridge);
    input.dose_m3 = point(5.5e-10);
    input.pitch_m = point(0.0009);
    unknown(&input, Bridge);
    input.pitch_m = point(0.001);
    input.force_n = None;
    let report = assess_solder_window(&input, steps(1024)).unwrap();
    assert!(matches!(
        report.check(MaxPadTemperature).status,
        SolderWindowStatus::NotAnalyzed { .. }
    ));
    assert_eq!(report.check(Bridge).status, SolderWindowStatus::Fail);
    assert_eq!(report.status(), SolderWindowStatus::Fail);
}

#[test]
fn never_wets_and_dose_boundaries_are_closed() {
    use SolderCriterion::*;
    let mut input = bench(35.0, 1.45, 0.375);
    // Tп(1,45) ≈ 511,58 К, поэтому маркер 530 К остаётся недостижимым в этом окне.
    input.limits.wetting_k = point(530.0);
    input.dose_m3 = point(5e-10);
    let report = assess_solder_window(&input, steps(1024)).unwrap();
    assert_eq!(report.check(MaxTimeToWet).status, SolderWindowStatus::Fail);
    assert_eq!(report.check(Bridge).status, SolderWindowStatus::Pass);
    assert_eq!(
        analyzed(analyzed(report.thermal).time_to_wet_s),
        SolderWettingTime::NotReached
    );
    input.dose_m3 = point(2.5e-10);
    let report = assess_solder_window(&input, steps(1024)).unwrap();
    assert_eq!(report.check(MinVolume).status, SolderWindowStatus::Pass);
}

#[test]
fn non_finite_and_inconsistent_values_are_errors_even_with_missing_evidence() {
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut input = bench(35.0, 1.45, 0.375);
        input.duration_s = None;
        input.thermal.contact = None;
        input.network.initial_k.pad.as_mut().unwrap().max = bad;
        assert!(matches!(
            assess_solder_window(&input, steps(1)),
            Err(ProcessError::InvalidInput(_))
        ));
        input.network.initial_k.pad = point(458.0);
        input.thermal.requested_power_w = point(bad);
        assert!(matches!(
            assess_solder_window(&input, steps(1)),
            Err(ProcessError::InvalidInput(_))
        ));
        input.thermal.requested_power_w = point(27.0);
        input.dose_m3 = point(bad);
        assert!(matches!(
            assess_solder_window(&input, steps(1)),
            Err(ProcessError::InvalidInput(_))
        ));
        input.dose_m3 = point(3.75e-10);
        input.limits.bridge.as_mut().unwrap().pitch_m.nominal = bad;
        assert!(matches!(
            assess_solder_window(&input, steps(1)),
            Err(ProcessError::InvalidInput(_))
        ));
    }
    let mut input = bench(35.0, 1.45, 0.375);
    input.limits.min_above_liquidus_s = point(1.3);
    assert!(matches!(
        assess_solder_window(&input, steps(1)),
        Err(ProcessError::InvalidInput(_))
    ));
}
