//! Синтетический стенд Т6.2. Амплитуды отнесены к ЭДС источника 1 В.

// Публичную точку подключения emc добавляет Т6.4.
#[path = "../src/emc/coupling.rs"]
mod coupling;
#[allow(dead_code)]
#[path = "../src/emc/loop_model.rs"]
mod loop_model;

use std::fmt::Write as _;
use std::io::Write as _;
use std::process::{Command, Stdio};

use coupling::*;
use loop_model::{Analysis, Sourced};
use nalgebra::Complex;
pub use sim_circuit::{SimError, pcb};

// Kobayashi, Terakado, 1981, табл. 7, метод G, εr = 10, s/h = 0,2.
// https://rose-ibadai.repo.nii.ac.jp/record/12931/files/CSI2011_1281.pdf
const TABLE: [[f64; 5]; 4] = [
    [0.1, 6.2693, 14.165, 6.2775, 5.5127],
    [0.5, 10.924, 23.133, 6.7915, 5.5798],
    [1.0, 16.101, 29.440, 7.2307, 5.7113],
    [2.0, 26.271, 40.148, 7.7984, 6.0309],
];
// CODATA 2022 и определение СИ соответственно.
const EPS_0: f64 = 8.854_187_818_8e-12;
const LIGHT_SPEED: f64 = 299_792_458.0;

fn sourced<T>(value: T) -> Option<Sourced<'static, T>> {
    Some(Sourced {
        value,
        source: "Стенд Т6.2, геометрия, полоса и нагрузки заданы в тесте",
    })
}

fn geometry() -> MicrostripPair {
    // Отношения размеров по табл. 7. Для стенда h = 1 мм и длина 20 мм.
    MicrostripPair {
        length_m: 0.020,
        width_m: 0.001,
        gap_m: 0.0002,
        height_m: 0.001,
        thickness_m: 0.0,
        relative_permittivity: 10.0,
        uniform_over_plane: true,
    }
}

fn band() -> CouplingBand {
    // От DC до 40 МГц, но не выше λ/100 (3,6° по длине участка).
    CouplingBand {
        min_frequency_hz: 0.0,
        max_frequency_hz: 40e6,
        max_wavelength_fraction: 0.01,
    }
}

fn resistive_ports() -> CouplingPorts {
    // Четыре одинаковых порта по 50 Ом. Ёмкости внешних нагрузок равны нулю.
    let port = ParallelRc {
        resistance_ohm: 50.0,
        capacitance_f: 0.0,
    };
    CouplingPorts {
        aggressor_near: port,
        victim_near: port,
        aggressor_far: port,
        victim_far: port,
    }
}

fn calculated<T: std::fmt::Debug>(result: Result<Analysis<T>, SimError>) -> T {
    match result.unwrap() {
        Analysis::Calculated(value) => value,
        other => panic!("ожидался расчёт, получено {other:?}"),
    }
}

fn not_analyzed<T: std::fmt::Debug>(result: Result<Analysis<T>, SimError>) {
    assert!(matches!(result, Ok(Analysis::NotAnalyzed(_))), "{result:?}");
}

fn model() -> CoupledMicrostrip {
    calculated(coupled_microstrip(sourced(geometry()), sourced(band())))
}

fn close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual:e} ≠ {expected:e}, допуск {tolerance:e}"
    );
}

#[test]
fn capacitance_and_inductance_match_independent_green_function_table() {
    let mut max_error: f64 = 0.0;
    for [width_ratio, ce, co, er_even, er_odd] in TABLE {
        let g = MicrostripPair {
            width_m: width_ratio * geometry().height_m,
            ..geometry()
        };
        let lc = calculated(coupled_microstrip(sourced(g), sourced(band()))).parameters();
        // По табличным C и εeff, с независимо вычисленной v, L' = 1/(v²C').
        let (ce, co) = (ce * EPS_0, co * EPS_0);
        let le = er_even / (LIGHT_SPEED.powi(2) * ce);
        let lo = er_odd / (LIGHT_SPEED.powi(2) * co);
        for (actual, per_m) in [
            (lc.self_c_f, (co + ce) / 2.0),
            (lc.mutual_c_f, (co - ce) / 2.0),
            (lc.self_l_h, (le + lo) / 2.0),
            (lc.mutual_l_h, (le - lo) / 2.0),
        ] {
            let expected = per_m * g.length_m;
            // 2% на разность модальных величин с остаточными ошибками
            // аппроксимаций εeff и Z0 из работы Киршнинга и Янсена.
            let relative_error = (actual / expected - 1.0).abs();
            close(actual, expected, 0.02 * expected);
            max_error = max_error.max(relative_error);
        }
    }
    let lc = model().parameters();
    println!(
        "Cвз = {:.6} пФ, Lвз = {:.6} нГн, наибольшее относительное расхождение C/L = {:.4}%",
        lc.mutual_c_f * 1e12,
        lc.mutual_l_h * 1e9,
        max_error * 100.0
    );
}

#[test]
fn near_and_far_have_expected_polarity_and_straddle_bench_limit() {
    // Ручной ориентир до реализации. Для w/h = 1 из табл. 7 получаем
    // Cm = 1,181060 пФ, M = 3,205505 нГн на 20 мм. Разложение по ω:
    // NEXT = jω(R Cm + M/R)/4, FEXT = jω(R Cm − M/R)/4, R = 50 Ом.
    // При 10/20 МГц NEXT = j1,93464/j3,86928 мВ, FEXT = −j0,07944/−j0,15887 мВ.
    // Синтетический предел 3 мВ даёт запасы +1,06536 и −0,86928 мВ.
    for (frequency, next_mv, fext_mv, margin_mv) in [
        (10e6, 1.93464, -0.07944, 1.06536),
        (20e6, 3.86928, -0.15887, -0.86928),
    ] {
        let result = calculated(model().transfer(sourced(resistive_ports()), frequency));
        // 0,03 мВ покрывает остаток линейного разложения и аппроксимацию C/L.
        close(result.victim_near.im * 1e3, next_mv, 0.03);
        close(result.victim_far.im * 1e3, fext_mv, 0.03);
        let margin = 3.0 - result.victim_near.norm() * 1e3;
        close(margin, margin_mv, 0.03);
        assert_eq!(margin.is_sign_positive(), margin_mv.is_sign_positive());
        println!("{frequency:e} Гц, запас на ближнем порту {margin:.6} мВ");
    }
    let dc = calculated(model().transfer(sourced(resistive_ports()), 0.0));
    // В DC остаётся делитель R/R и два независимых проводника.
    for on_excited_trace in [dc.aggressor_near, dc.aggressor_far] {
        close(on_excited_trace.re, 0.5, f64::EPSILON);
        assert_eq!(on_excited_trace.im, 0.0);
    }
    assert_eq!(dc.victim_near, Complex::new(0.0, 0.0));
    assert_eq!(dc.victim_far, Complex::new(0.0, 0.0));
}

#[test]
fn frequency_electrical_length_and_geometry_boundaries_are_enforced() {
    let p = sourced(resistive_ports());
    calculated(model().transfer(p, 40e6));
    not_analyzed(model().transfer(p, 40e6 + 1.0));
    let bandpass = CouplingBand {
        min_frequency_hz: 10e6,
        ..band()
    };
    let restricted = calculated(coupled_microstrip(sourced(geometry()), sourced(bandpass)));
    not_analyzed(restricted.transfer(p, 10e6 - 1.0));
    calculated(restricted.transfer(p, 10e6));

    // c/(√10·0,04 м·100) = 23,700674 МГц.
    let long = MicrostripPair {
        length_m: 0.040,
        ..geometry()
    };
    let limited = calculated(coupled_microstrip(sourced(long), sourced(band())));
    close(limited.frequency_range_hz()[1], 23_700_674.8155, 1.0);
    calculated(limited.transfer(p, 23.70e6));
    not_analyzed(limited.transfer(p, 23.71e6));
    let non_overlapping = CouplingBand {
        min_frequency_hz: 24e6,
        ..band()
    };
    not_analyzed(coupled_microstrip(sourced(long), sourced(non_overlapping)));

    // Поперечный размер 2w+s = 110 мм превосходит длину участка.
    // Синтетическая геометрия сохраняет w/h = 1 и s/h = 0,2.
    // Предел поперечной квазистатики c/(√10·0,11 м·100) ≈ 8,618 МГц.
    let wide = MicrostripPair {
        width_m: 0.050,
        height_m: 0.050,
        gap_m: 0.010,
        ..geometry()
    };
    let limited = calculated(coupled_microstrip(sourced(wide), sourced(band())));
    calculated(limited.transfer(p, 8.61e6));
    not_analyzed(limited.transfer(p, 8.62e6));

    // При h = 1 мм нижняя граница s/h = 0,1 из формулы (1) составляет 0,1 мм.
    for (gap, in_domain) in [(0.0000999, false), (0.0001, true)] {
        let g = MicrostripPair {
            gap_m: gap,
            ..geometry()
        };
        let outcome = coupled_microstrip(sourced(g), sourced(band()));
        if in_domain {
            calculated(outcome);
        } else {
            not_analyzed(outcome);
        }
    }
    for unsupported in [
        MicrostripPair {
            thickness_m: 35e-6,
            ..geometry()
        }, // Фольга 1 oz, см. pcb.
        MicrostripPair {
            uniform_over_plane: false,
            ..geometry()
        },
    ] {
        not_analyzed(coupled_microstrip(sourced(unsupported), sourced(band())));
    }
}

#[test]
fn missing_or_unsourced_inputs_leave_the_result_open() {
    not_analyzed(coupled_microstrip(None, sourced(band())));
    not_analyzed(coupled_microstrip(sourced(geometry()), None));
    let unsourced = Some(Sourced {
        value: geometry(),
        source: " ",
    });
    not_analyzed(coupled_microstrip(unsourced, sourced(band())));
    not_analyzed(model().transfer(None, 10e6));
    let unsourced = Some(Sourced {
        value: resistive_ports(),
        source: " ",
    });
    not_analyzed(model().transfer(unsourced, 10e6));
}

#[test]
fn invalid_numbers_are_errors_even_if_other_data_are_missing_or_outside_band() {
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for g in [
            MicrostripPair {
                width_m: bad,
                ..geometry()
            },
            MicrostripPair {
                height_m: bad,
                ..geometry()
            },
            MicrostripPair {
                gap_m: bad,
                ..geometry()
            },
            MicrostripPair {
                length_m: bad,
                ..geometry()
            },
            MicrostripPair {
                thickness_m: bad,
                ..geometry()
            },
            MicrostripPair {
                relative_permittivity: bad,
                ..geometry()
            },
        ] {
            assert!(coupled_microstrip(sourced(g), None).is_err());
        }
        for b in [
            CouplingBand {
                max_wavelength_fraction: bad,
                ..band()
            },
            CouplingBand {
                max_frequency_hz: bad,
                ..band()
            },
            CouplingBand {
                min_frequency_hz: bad,
                ..band()
            },
        ] {
            assert!(coupled_microstrip(None, sourced(b)).is_err());
        }
        assert!(model().transfer(None, bad).is_err());
        let mut p = resistive_ports();
        p.victim_far.resistance_ohm = bad;
        assert!(model().transfer(sourced(p), 50e6).is_err());
        p = resistive_ports();
        p.aggressor_near.capacitance_f = bad;
        assert!(model().transfer(sourced(p), 50e6).is_err());
    }

    let mut p = resistive_ports();
    p.victim_far.resistance_ohm = 0.0;
    assert!(model().transfer(sourced(p), 10e6).is_err());
    p.victim_far.resistance_ohm = 50.0;
    p.victim_far.capacitance_f = f64::MAX;
    assert!(model().transfer(sourced(p), 10e6).is_err());
}

#[test]
fn four_port_transfer_matches_converged_ngspice_ladder() {
    // Независимый AC-расчёт с несимметричными нагрузками, в т.ч. реактивными.
    let ports = CouplingPorts {
        aggressor_near: ParallelRc {
            resistance_ohm: 30.0,
            capacitance_f: 2e-12,
        },
        victim_near: ParallelRc {
            resistance_ohm: 100.0,
            capacitance_f: 8e-12,
        },
        aggressor_far: ParallelRc {
            resistance_ohm: 75.0,
            capacitance_f: 3e-12,
        },
        victim_far: ParallelRc {
            resistance_ohm: 1000.0,
            capacitance_f: 12e-12,
        },
    };
    let model = model();
    let lc = model.parameters();
    let coarse = solve_ac_ladder(lc, ports, 16);
    let fine = solve_ac_ladder(lc, ports, 32);
    let mut max_grid_error: f64 = 0.0;
    let mut max_transfer_error: f64 = 0.0;
    for (index, (coarse, fine)) in coarse.iter().zip(&fine).enumerate() {
        let frequency = (5.0 + 5.0 * index as f64) * 1e6;
        close(fine[0], frequency, 1e-6);
        close(coarse[0], frequency, 1e-6);
        let response = calculated(model.transfer(sourced(ports), frequency));
        for (i, actual) in [
            response.aggressor_near,
            response.victim_near,
            response.aggressor_far,
            response.victim_far,
        ]
        .into_iter()
        .enumerate()
        {
            let at = |row: &[f64]| Complex::new(row[1 + 2 * i], row[2 + 2 * i]);
            max_grid_error = max_grid_error.max((at(fine) - at(coarse)).norm());
            max_transfer_error = max_transfer_error.max((actual - at(fine)).norm());
        }
    }
    // 2·10⁻⁷ В допускает дискретизацию π-лестницы, контроль сгущения отдельно.
    assert!(max_grid_error < 2e-7, "невязка сеток {max_grid_error:e} В");
    assert!(
        max_transfer_error < 2e-7,
        "невязка передачи {max_transfer_error:e} В"
    );
    println!(
        "ngspice AC, 5–40 МГц: Δ сеток {max_grid_error:.6e} В, Δ передачи {max_transfer_error:.6e} В"
    );
}

fn solve_ac_ladder(lc: CoupledLc, p: CouplingPorts, sections: u32) -> Vec<Vec<f64>> {
    let mut deck = String::from("Пара трасс с общей опорой\nVsrc src 0 DC 0 AC 1\n");
    for (label, node, rail, port) in [
        ("an", "a0".into(), "src", p.aggressor_near),
        ("bn", "b0".into(), "0", p.victim_near),
        ("af", format!("a{sections}"), "0", p.aggressor_far),
        ("bf", format!("b{sections}"), "0", p.victim_far),
    ] {
        writeln!(deck, "R{label} {node} {rail} {:.16e}", port.resistance_ohm).unwrap();
        writeln!(deck, "C{label} {node} 0 {:.16e}", port.capacitance_f).unwrap();
    }
    let sections_f = f64::from(sections);
    for k in 0..sections {
        let next = k + 1;
        let inductance = lc.self_l_h / sections_f;
        writeln!(deck, "LA{k} a{k} a{next} {inductance:.16e}").unwrap();
        writeln!(deck, "LB{k} b{k} b{next} {inductance:.16e}").unwrap();
        writeln!(
            deck,
            "K{k} LA{k} LB{k} {:.16e}",
            lc.mutual_l_h / lc.self_l_h
        )
        .unwrap();
    }
    for k in 0..=sections {
        // У π-секции половина C на каждом конце. Внутренние узлы общие для соседей.
        let fraction = if k == 0 || k == sections { 0.5 } else { 1.0 } / sections_f;
        let ground_c = (lc.self_c_f - lc.mutual_c_f) * fraction;
        writeln!(
            deck,
            "CA{k} a{k} 0 {ground_c:.16e}\nCB{k} b{k} 0 {ground_c:.16e}"
        )
        .unwrap();
        writeln!(deck, "CM{k} a{k} b{k} {:.16e}", lc.mutual_c_f * fraction).unwrap();
    }
    deck.push_str(".control\nset numdgt=16\nset width=300\nac lin 8 5e6 40e6\n");
    for (label, node) in [
        ("an", "a0".into()),
        ("bn", "b0".into()),
        ("af", format!("a{sections}")),
        ("bf", format!("b{sections}")),
    ] {
        writeln!(
            deck,
            "let r{label}=real(v({node}))\nlet i{label}=imag(v({node}))"
        )
        .unwrap();
    }
    deck.push_str("print ran ian rbn ibn raf iaf rbf ibf\nquit\n.endc\n.end\n");
    let executable = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../assets/third_party/ngspice/bin/ngspice"
    );
    let mut child = Command::new(executable)
        .args(["-n", "-b"])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("для независимой сверки нужен ngspice из комплекта проекта");
    if let Err(error) = child.stdin.take().unwrap().write_all(deck.as_bytes()) {
        let _ = child.kill();
        let _ = child.wait();
        panic!("не удалось передать схему в ngspice: {error}");
    }
    let output = child.wait_with_output().unwrap();
    assert!(
        output.status.success() && output.stderr.is_empty(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let stdout = String::from_utf8(output.stdout).unwrap();
    let rows: Vec<_> = stdout
        .lines()
        .filter_map(|line| {
            let (index, values) = line.split_once('\t')?;
            index.parse::<usize>().ok()?;
            let values: Vec<f64> = values
                .split_whitespace()
                .map(|v| v.parse().unwrap())
                .collect();
            assert_eq!(values.len(), 9);
            assert!(values.iter().all(|x| x.is_finite()));
            Some(values)
        })
        .collect();
    assert_eq!(rows.len(), 8, "{stdout}");
    rows
}
