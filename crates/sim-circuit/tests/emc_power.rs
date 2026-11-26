//! Синтетический стенд Т6.1. Пределы действуют только для этого опыта.

// Точка подключения библиотеки принадлежит Т6.4. До неё тест собирает те же исходники.
#[path = "../src/emc/loop_model.rs"]
mod loop_model;
#[path = "../src/emc/power.rs"]
mod power;

use std::f64::consts::{PI, TAU};
use std::num::NonZeroU32;

use loop_model::*;
use nalgebra::Complex;
use power::*;
use sim_circuit::SimError;
use sim_circuit::ngspice::{NgAnalysis, find_ngspice, run};

fn sourced<T>(value: T) -> Option<Sourced<'static, T>> {
    Some(Sourced {
        value,
        source: "Синтетический стенд Т6.1, условия заданы в тесте",
    })
}

fn calculated<T: std::fmt::Debug>(result: Result<Analysis<T>, sim_circuit::SimError>) -> T {
    match result.unwrap() {
        Analysis::Calculated(value) => value,
        other => panic!("ожидалась численная модель: {other:?}"),
    }
}

fn close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual:e} ≠ {expected:e}, допуск {tolerance:e}"
    );
}

// Полоса стенда 10 кГц. 0,01 λ даёт фазовый набег до 3,6°.
fn domain() -> LumpedDomain {
    LumpedDomain {
        max_frequency_hz: 10_000.0,
        longest_path_m: 6.0,
        min_wave_speed_m_s: 200e6,
        max_wavelength_fraction: 0.01,
    }
}

fn geometry() -> RectangularLoop {
    // Rosa и Cohen, пример 49. ρ меди из sim_circuit::pcb при 20 °C.
    RectangularLoop {
        side_a_m: 2.0,
        side_b_m: 1.0,
        wire_radius_m: 1e-3,
        resistivity_ohm_m: sim_circuit::pcb::CU_RHO_20C,
        max_radius_to_side: 0.002,
    }
}

#[test]
fn rectangular_wire_matches_published_example_and_resistance_balance() {
    let result = calculated(rectangular_loop(sourced(geometry()), sourced(domain())));
    close(result.area_m2, 2.0, f64::EPSILON);
    let rl = result.path.series_rl();
    // Пример 49 даёт 8,0171 мкГн. 1 нГн покрывает табличное округление
    // примера: прямая подстановка в формулу (86) даёт 8,017862 мкГн.
    close(rl.inductance_h, 8.0171e-6, 1e-9);
    close(rl.inductance_h, 8.017862e-6, 1e-12);
    // Численная сверка интеграла Неймана без первообразной. Удвоение
    // сетки Симпсона с 8192 до 16384 панелей меняет L на 0,504 пГн.
    // На тонкой сетке шаг по длинной стороне меньше r/8.
    let fine = quadrature_loop_inductance(16_384);
    let coarse = quadrature_loop_inductance(8192);
    assert!((fine - coarse).abs() < 1e-12);
    // Для метода порядка 4 при делении шага пополам оценка ошибки ≈ Δ/15.
    close(rl.inductance_h, fine, 0.05e-12);
    // Ток 1 А: 6 м меди, сечение π мм², падение 0,0320856365 В.
    close(rl.resistance_ohm, 0.0320856365, 1e-10);
    let impedance = calculated(result.path.impedance(1000.0));
    close(impedance.re, 0.0320856365, 1e-10);
    // 2π·1000·8,017862 мкГн, допуск учитывает отсечение членов (86).
    close(impedance.im, 0.050377713, 1e-8);
    // δ = r при 4255,49 Гц. Проверяем обе стороны физической границы.
    assert!(matches!(
        result.path.impedance(4255.0),
        Ok(Analysis::Calculated(_))
    ));
    assert!(matches!(
        result.path.impedance(4256.0),
        Ok(Analysis::NotAnalyzed(_))
    ));
}

fn quadrature_loop_inductance(panels: u32) -> f64 {
    let integrate = |len: f64, gap: f64| {
        let step = len / f64::from(panels);
        let integrand = |x: f64| (len - x) / x.hypot(gap);
        let sum = (1..panels).fold(integrand(0.0) + integrand(len), |sum, i| {
            let weight = if i % 2 == 0 { 2.0 } else { 4.0 };
            sum + weight * integrand(step * f64::from(i))
        });
        step * sum / 3.0
    };
    // Ток в противоположных сторонах направлен навстречу. Внутреннюю
    // индуктивность даёт энергия поля в поперечном сечении: μ₀/(8π) на метр.
    // μ₀ по CODATA 2022, размеры по примеру 49 Rosa и Cohen.
    let mu = 1.256_637_061_27e-6;
    let l_inside = mu * 6.0 / (8.0 * PI);
    let l_outside = (mu / PI)
        * (integrate(2.0, 0.001) + integrate(1.0, 0.001)
            - integrate(2.0, 1.0)
            - integrate(1.0, 2.0));
    l_inside + l_outside
}

#[test]
fn geometry_and_electrical_length_boundaries_are_enforced() {
    let mut rectangle = geometry();
    rectangle.wire_radius_m = 0.00201;
    assert!(matches!(
        rectangular_loop(sourced(rectangle), sourced(domain())),
        Ok(Analysis::NotAnalyzed(_))
    ));
    rectangle.wire_radius_m = 0.00199;
    calculated(rectangular_loop(sourced(rectangle), sourced(domain())));

    let mut spec = domain();
    spec.longest_path_m = 5.99;
    assert!(matches!(
        rectangular_loop(sourced(geometry()), sourced(spec)),
        Ok(Analysis::NotAnalyzed(_))
    ));
    // 0,01 × (200 Мм/с) / (400 м) = 5000 Гц.
    spec.longest_path_m = 400.0;
    let path = calculated(fitted_path(
        sourced(SeriesRl {
            resistance_ohm: 0.1,
            inductance_h: 2e-6,
        }),
        sourced(spec),
    ));
    close(path.max_frequency_hz(), 5000.0, 1e-9);
    calculated(path.impedance(4999.0));
    assert!(matches!(
        path.impedance(5001.0),
        Ok(Analysis::NotAnalyzed(_))
    ));
    assert!(matches!(
        path.impedance(f64::NAN),
        Err(sim_circuit::SimError::Invalid(_))
    ));
}

fn path(resistance_ohm: f64, inductance_h: f64) -> LumpedPath {
    calculated(fitted_path(
        sourced(SeriesRl {
            resistance_ohm,
            inductance_h,
        }),
        sourced(domain()),
    ))
}

fn input<'a>(harmonics: &'a [Harmonic], decoupling: &'a [Decoupling]) -> PowerInput<'a> {
    PowerInput {
        domain: sourced(domain()),
        source_v: sourced(5.0),
        feed: Some(path(1.0, 0.0)),
        shared_return: Some(path(0.2, 0.0)),
        decoupling: sourced(decoupling),
        load: sourced(PeriodicLoad {
            dc_a: 1.0,
            fundamental_hz: 1000.0,
            harmonics,
            complete_spectrum: true,
        }),
    }
}

fn harmonic(order: u32, re: f64, im: f64) -> Harmonic {
    Harmonic {
        order,
        peak: Complex::new(re, im),
    }
}

fn samples() -> NonZeroU32 {
    // 256 ячеек разрешают обе вершины косинуса и дают поправку < 19 мкВ.
    NonZeroU32::new(256).unwrap()
}

#[test]
fn geometric_loop_drives_rail_and_limits_spectrum() {
    let rectangle = calculated(rectangular_loop(sourced(geometry()), sourced(domain())));
    let harmonics = [harmonic(1, 0.25, 0.0)];
    let response = calculated(analyze_power(PowerInput {
        feed: Some(rectangle.path),
        shared_return: Some(path(0.0, 0.0)),
        ..input(&harmonics, &[])
    }));
    // Законы Ома и Фарадея в фазах 0 и 1/4. Берём R по сечению,
    // L по подстановке в (86), с её округлением до 8,017862 мкГн.
    close(response.rail.at_phase(0.0).unwrap(), 4.959892954375, 1e-10);
    close(response.rail.at_phase(0.25).unwrap(), 4.98050879175, 5e-9);
    close(
        response.return_shift.at_phase(0.25).unwrap(),
        0.0,
        f64::EPSILON,
    );
    close(response.max_frequency_hz, 4255.489714, 1e-6);
    let harmonics = [harmonic(5, 0.25, 0.0)];
    assert!(matches!(
        analyze_power(PowerInput {
            feed: Some(rectangle.path),
            shared_return: Some(path(0.0, 0.0)),
            ..input(&harmonics, &[])
        }),
        Ok(Analysis::NotAnalyzed(_))
    ));
}

#[test]
fn sourced_stand_limits_distinguish_both_sides_without_positive_verdict() {
    let supply = sourced(VoltageLimit::Min(3.8));
    let ground = sourced(VoltageLimit::Max(0.24));
    // Ручной расчёт до реализации: Vmin = 5 − 1(1 + A), Gmax = 0,2(1 + A).
    // A = 0,25 / 0,15 А: запасы Vmin−3,8 = −0,05 / +0,05 В,
    // 0,24−Gmax = −0,01 / +0,01 В. Vpp = 2A = 0,50 / 0,30 В.
    for (amplitude, rail_margin, ground_margin) in [(0.25, -0.05, -0.01), (0.15, 0.05, 0.01)] {
        let response = calculated(analyze_power(input(&[harmonic(1, amplitude, 0.0)], &[])));
        let rail = response.rail.extrema(samples()).unwrap();
        close(
            rail.max_sample_v - rail.min_sample_v,
            2.0 * amplitude,
            1e-14,
        );
        for (trace, limit, expected) in [
            (&response.rail, supply, rail_margin),
            (&response.return_shift, ground, ground_margin),
        ] {
            let margin = match trace.check(limit, samples()).unwrap() {
                VoltageCheck::Fail(margin) if expected < 0.0 => margin,
                VoltageCheck::NotAnalyzed {
                    margin: Some(margin),
                    ..
                } if expected > 0.0 => margin,
                other => panic!("неожиданный исход стенда: {other:?}"),
            };
            close(margin.sampled_v, expected, 1e-14);
            assert!((margin.lower_v..=margin.sampled_v + 1e-14).contains(&expected));
        }
    }
}

#[test]
fn rc_closed_form_and_between_sample_extremum() {
    // При R=1 Ом, ωRC=1, I(t)=1+0,25 cos(ωt), решение C·V'=(5−V)/R−I:
    // V(t)=4−0,125[cos(ωt)+sin(ωt)], Vmin=4−0,25/√2 при фазе 1/8.
    let cap = [Decoupling {
        capacitance_f: 1.0 / (TAU * 1000.0),
        esr_ohm: 0.0,
        esl_h: 0.0,
    }];
    let result = calculated(analyze_power(input(&[harmonic(1, 0.25, 0.0)], &cap)));
    let exact_min = 4.0 - 0.25 / 2.0_f64.sqrt();
    close(result.rail.at_phase(0.125).unwrap(), exact_min, 1e-14);
    close(result.rail.at_phase(-0.875).unwrap(), exact_min, 1e-14);
    close(result.rail.harmonics()[0].peak.re, -0.125, 1e-14);
    close(result.rail.harmonics()[0].peak.im, 0.125, 1e-14);
    close(
        result.return_shift.at_phase(0.125).unwrap(),
        0.2 + 0.05 / 2.0_f64.sqrt(),
        1e-14,
    );
    let extrema = result.rail.extrema(NonZeroU32::new(4).unwrap()).unwrap();
    assert!(extrema.min_lower_v <= exact_min && exact_min < extrema.min_sample_v);
    // 3,85 В пропущены 4 отсчётами, но огибающая не даёт положительного вывода.
    let check = result
        .rail
        .check(
            sourced(VoltageLimit::Min(3.85)),
            NonZeroU32::new(4).unwrap(),
        )
        .unwrap();
    assert!(
        matches!(check, VoltageCheck::NotAnalyzed { margin: Some(m), .. } if m.lower_v < 0.0 && m.sampled_v > 0.0)
    );
    assert!(matches!(
        result
            .rail
            .check(sourced(VoltageLimit::Min(3.85)), samples())
            .unwrap(),
        VoltageCheck::Fail(_)
    ));
}

#[test]
fn missing_inputs_sources_and_incomplete_or_out_of_band_loads_stay_open() {
    let harmonics = [harmonic(1, 0.25, 0.0)];
    let valid = input(&harmonics, &[]);
    for unknown in [
        PowerInput {
            shared_return: None,
            ..valid
        },
        PowerInput {
            load: None,
            ..valid
        },
        PowerInput {
            decoupling: None,
            ..valid
        },
        PowerInput {
            domain: None,
            ..valid
        },
        PowerInput {
            feed: None,
            ..valid
        },
        PowerInput {
            source_v: Some(Sourced {
                value: 5.0,
                source: " ",
            }),
            ..valid
        },
        input(&[], &[]),
        input(&[harmonic(11, 0.25, 0.0)], &[]),
    ] {
        assert!(matches!(
            analyze_power(unknown),
            Ok(Analysis::NotAnalyzed(_))
        ));
    }
    let mut incomplete = valid;
    incomplete.load.as_mut().unwrap().value.complete_spectrum = false;
    assert!(matches!(
        analyze_power(incomplete),
        Ok(Analysis::NotAnalyzed(_))
    ));
    let response = calculated(analyze_power(valid));
    assert!(matches!(
        response.rail.check(None, samples()).unwrap(),
        VoltageCheck::NotAnalyzed { margin: None, .. }
    ));
    assert!(matches!(
        response
            .rail
            .check(
                Some(Sourced {
                    value: VoltageLimit::Min(4.0),
                    source: ""
                }),
                samples()
            )
            .unwrap(),
        VoltageCheck::NotAnalyzed { margin: None, .. }
    ));
    assert!(matches!(
        rectangular_loop(None, sourced(domain())),
        Ok(Analysis::NotAnalyzed(_))
    ));
}

#[test]
fn invalid_inputs_are_errors_even_alongside_missing_information() {
    let harmonics = [harmonic(1, 0.25, 0.0)];
    let valid = input(&harmonics, &[]);
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(
            analyze_power(PowerInput {
                source_v: sourced(invalid),
                domain: None,
                ..valid
            })
            .is_err()
        );
        assert!(analyze_power(input(&[harmonic(1, 0.0, invalid)], &[])).is_err());
        assert!(
            analyze_power(input(
                &harmonics,
                &[Decoupling {
                    capacitance_f: 1e-6,
                    esr_ohm: invalid,
                    esl_h: 0.0
                }]
            ))
            .is_err()
        );
        assert!(
            rectangular_loop(
                sourced(RectangularLoop {
                    side_a_m: invalid,
                    ..geometry()
                }),
                None
            )
            .is_err()
        );
        assert!(
            fitted_path(
                None,
                sourced(LumpedDomain {
                    max_frequency_hz: invalid,
                    ..domain()
                })
            )
            .is_err()
        );
        assert!(
            fitted_path(
                sourced(SeriesRl {
                    resistance_ohm: 0.1,
                    inductance_h: invalid
                }),
                None
            )
            .is_err()
        );
        let rail = calculated(analyze_power(valid)).rail;
        assert!(
            rail.check(sourced(VoltageLimit::Max(invalid)), samples())
                .is_err()
        );
        assert!(rail.at_phase(invalid).is_err());
    }
    assert!(analyze_power(input(&[harmonic(1, 1.0, 0.0), harmonic(1, 0.0, 1.0)], &[])).is_err());
    assert!(analyze_power(input(&[harmonic(0, 1.0, 0.0)], &[])).is_err());
    assert!(
        analyze_power(PowerInput {
            feed: Some(path(0.1, 0.0)),
            ..valid
        })
        .is_err()
    );
}

#[test]
fn undamped_resonance_has_no_finite_steady_state_envelope() {
    let harmonics = [harmonic(1, 1.0, 0.0)];
    let inductance = 1e-3;
    // ω²LC = 1. Источник и контур без потерь, периодический отклик не определён.
    let cap = [Decoupling {
        capacitance_f: 1.0 / ((TAU * 1000.0).powi(2) * inductance),
        esr_ohm: 0.0,
        esl_h: 0.0,
    }];
    let mut fixture = input(&harmonics, &cap);
    fixture.feed = Some(path(0.0, inductance));
    fixture.shared_return = Some(path(0.0, 0.0));
    assert!(matches!(
        analyze_power(fixture),
        Ok(Analysis::NotAnalyzed(_))
    ));
}

#[test]
fn rlc_rail_and_shared_return_match_ngspice_for_same_ports_and_waveform() {
    let harmonics = [harmonic(1, 0.25, 0.0), harmonic(3, 0.0, -0.10)];
    let caps = [
        Decoupling {
            capacitance_f: 100e-6,
            esr_ohm: 0.05,
            esl_h: 5e-6,
        },
        Decoupling {
            capacitance_f: 47e-6,
            esr_ohm: 0.15,
            esl_h: 2e-6,
        },
    ];
    let mut fixture = input(&harmonics, &caps);
    fixture.feed = Some(path(1.0, 100e-6));
    fixture.shared_return = Some(path(0.2, 10e-6));
    let response = calculated(analyze_power(fixture));
    // У ngspice собственное временное интегрирование. Источники физически
    // включены между VDD и локальным возвратом. 1 мкс даёт 333 шага на период
    // старшей гармоники. Сверяем весь последний период после 19 мс затухания.
    let deck = "Блок с локальной развязкой\n\
        Vsrc src 0 5\nRfeed src a 0.8\nLfeed a vdd 90u\n\
        Rshared g b 0.2\nLshared b 0 10u\n\
        Rcap1 vdd c1 0.05\nLcap1 c1 d1 5u\nCcap1 d1 g 100u\n\
        Rcap2 vdd c2 0.15\nLcap2 c2 d2 2u\nCcap2 d2 g 47u\n\
        Ione vdd g SIN(1 0.25 1k 0 0 90)\nIthree vdd g SIN(0 0.10 3k)\n\
        .options reltol=1e-5 vntol=1e-8 abstol=1e-10\n.end";
    let ng = find_ngspice().expect("для независимой сверки нужен ngspice из комплекта проекта");
    // Рабочая точка среднего тока рассчитывается отдельно от пуска SIN на t=0.
    let dc_deck = deck
        .replace("SIN(1 0.25 1k 0 0 90)", "1")
        .replace("SIN(0 0.10 3k)", "0");
    let op = run(&ng, &dc_deck, NgAnalysis::Op, &["v(vdd)", "v(g)"]).unwrap();
    close(response.rail.dc_v(), op.rows[0][0] - op.rows[0][1], 1e-12);
    close(response.return_shift.dc_v(), op.rows[0][1], 1e-12);
    let t = run(
        &ng,
        deck,
        NgAnalysis::Tran {
            tstep: 1e-6,
            tstart: 0.019,
            tstop: 0.020,
            tmax: 1e-6,
            uic: false,
        },
        &["v(vdd)", "v(g)"],
    )
    .unwrap();
    assert!(t.rows.len() >= 1000);
    let mut err_rail: f64 = 0.0;
    let mut err_shared: f64 = 0.0;
    let mut peak_rail: f64 = 0.0;
    for row in t.rows {
        let phase = row[0] * 1000.0;
        let rail = row[1] - row[2];
        peak_rail = peak_rail.max((rail - 4.0).abs());
        err_rail = err_rail.max((response.rail.at_phase(phase).unwrap() - rail).abs());
        err_shared =
            err_shared.max((response.return_shift.at_phase(phase).unwrap() - row[2]).abs());
    }
    assert!(
        peak_rail > 0.2,
        "воздействие на стенде должно заметно менять питание"
    );
    // 50 мкВ / 200 мВ = 0,025% для сверки независимых численных методов.
    assert!(
        err_rail < 50e-6 && err_shared < 50e-6,
        "невязка {err_rail:e} / {err_shared:e} В"
    );
    println!(
        "ngspice: максимальные невязки питания и возврата {err_rail:.6e} / {err_shared:.6e} В"
    );
    close(response.rail.fundamental_hz(), 1000.0, f64::EPSILON);
    close(
        calculated(fixture.shared_return.unwrap().impedance(1000.0)).im,
        2.0 * PI * 0.01,
        1e-14,
    );
}
