//! Синтетический стенд Т6.3, «стержень» над заявленной плоскостью возврата.

// Публичную точку подключения emc добавляет Т6.4.
#[path = "../src/emc/cable.rs"]
mod cable;
#[allow(dead_code)]
#[path = "../src/emc/loop_model.rs"]
mod loop_model;

use std::fmt::Write as _;
use std::io::Write as _;
use std::process::{Command, Stdio};

use cable::*;
use loop_model::{Analysis, Sourced};
use nalgebra::Complex;
pub use sim_circuit::SimError;

// Геометрия стенда. Радиус 4 мм, высота оси 5 мм, длина 0,2 м, однородный воздух
// в идеализации εr = 1. Отношение h/r = 5/4 выбрано для ручной сверки.
const LENGTH_M: f64 = 0.2;
const PLANE: &str = "стенд/плоскость";
const FIXTURE: &str = "Исходная спецификация синтетического стенда Т6.3";
// CODATA 2022, таблица XXXIII.
const C0: f64 = 299_792_458.0;
const EPS0: f64 = 8.854_187_818_8e-12;
// Для точных ручных формул допуск 64 ε f64 на округление нескольких операций.
const ROUNDING: f64 = 64.0 * f64::EPSILON;

fn sourced<T>(value: T) -> Option<Sourced<'static, T>> {
    Some(Sourced {
        value,
        source: FIXTURE,
    })
}

fn fixture() -> CableInput<'static> {
    CableInput {
        geometry: sourced(CableGeometry {
            length_m: LENGTH_M,
            radius_m: 0.004,
            axis_height_m: 0.005,
            relative_permittivity: 1.0,
            reference: PLANE,
            uniform_over_plane: true,
        }),
        // Параметры синтетического стенда. R кабеля 1 Ом, утечки нет.
        losses: sourced(CableLosses {
            series_resistance_ohm_per_m: 5.0,
            shunt_conductance_s_per_m: 0.0,
        }),
        // Область стенда до 200 МГц с дополнительным ограничением λ/10.
        band: sourced(CableBand {
            min_frequency_hz: 0.0,
            max_frequency_hz: 200e6,
            max_wavelength_fraction: 0.1,
        }),
        source: sourced(CableSource {
            reference: PLANE,
            peak_v: Complex::new(1.0, 0.0),
            impedance_ohm: Complex::new(10.0, 0.0),
        }),
        termination: sourced(CableTermination {
            reference: PLANE,
            load: CableLoad::ImpedanceOhm(Complex::new(8.0, 0.0)),
        }),
        limit: sourced(CableLimit {
            reference: PLANE,
            end: CableEnd::Far,
            frequency_hz: 0.0,
            max_peak_a: 0.05,
        }),
    }
}

fn calculated<'a>(input: &CableInput<'a>, frequency_hz: f64) -> CableResponse<'a> {
    match common_mode_current(input, frequency_hz).unwrap() {
        Analysis::Calculated(response) => response,
        other => panic!("ожидался расчёт, получено {other:?}"),
    }
}

fn unanalysed(input: &CableInput<'_>, frequency_hz: f64) {
    let result = common_mode_current(input, frequency_hz);
    assert!(matches!(result, Ok(Analysis::NotAnalyzed(_))), "{result:?}");
}

fn close(actual: Complex<f64>, expected: Complex<f64>, tolerance: f64) {
    assert!(
        (actual - expected).norm() <= tolerance,
        "{actual:e} ≠ {expected:e}, допуск {tolerance:e}"
    );
}

#[test]
fn dc_straddles_the_declared_current_limit() {
    // Ручной ориентир до реализации. При DC и G=0 по закону Ома
    // I = 1/(10+1+Rн). Для Rн=8/9/10 Ом это 52,631579/50/47,619048 мА,
    // запас до 50 мА равен −2,631579/0/+2,380952 мА. Напряжения на
    // портах: ближнее (1+Rн)I, дальнее Rн I.
    for (load, total_ohm) in [(8.0, 19.0), (9.0, 20.0), (10.0, 21.0)] {
        let mut input = fixture();
        input.termination.as_mut().unwrap().value.load =
            CableLoad::ImpedanceOhm(Complex::new(load, 0.0));
        let expected_a = 1.0 / total_ohm;
        let expected_margin = 0.05 - expected_a;
        let response = calculated(&input, 0.0);
        for (port, to_reference_ohm) in [(response.near, load + 1.0), (response.far, load)] {
            close(port.current_peak_a, expected_a.into(), ROUNDING);
            close(
                port.voltage_peak_v,
                (expected_a * to_reference_ohm).into(),
                ROUNDING,
            );
        }
        let margin = match response.check {
            CableCheck::Fail(margin) if expected_margin < 0.0 => margin,
            CableCheck::NotAnalyzed {
                margin: Some(margin),
                ..
            } if expected_margin >= 0.0 => margin,
            other => panic!("нагрузка {load} Ом, получено {other:?}"),
        };
        assert!((margin.margin_a - expected_margin).abs() <= ROUNDING);
        assert_eq!(margin.end, CableEnd::Far);
        println!(
            "Rн={load} Ом, I={:.9} мА, запас={:.9} мА",
            margin.peak_a * 1e3,
            margin.margin_a * 1e3
        );
    }
}

fn lossless(input: &mut CableInput<'_>) {
    // Стенд бегущей волны и предельных нагрузок с явными нулевыми потерями.
    input
        .losses
        .as_mut()
        .unwrap()
        .value
        .series_resistance_ohm_per_m = 0.0;
}

fn independent_lc_per_m() -> (f64, f64) {
    // MIT 6.013, §4.7. В поперечном сечении две линейные плотности
    // заряда стоят на ±sqrt(h²−r²) = ±3 мм. В ближайшей к плоскости
    // точке стержня z = 1 мм, расстояния до них 2 и 4 мм.
    // По потенциалу между этой точкой и плоскостью C' = 2πε₀/ln(4/2).
    // Из скорости бегущей волны c₀ следует L' = 1/(c₀² C').
    let c = std::f64::consts::TAU * EPS0 / 2.0_f64.ln();
    (1.0 / (C0 * C0 * c), c)
}

#[test]
fn matched_line_preserves_current_magnitude_and_delays_phase() {
    let mut input = fixture();
    lossless(&mut input);
    let (l, c) = independent_lc_per_m();
    let impedance = (l / c).sqrt();
    let drive = Complex::new(1.0, -0.25);
    input.source.as_mut().unwrap().value.peak_v = drive;
    input.termination.as_mut().unwrap().value.load =
        CableLoad::ImpedanceOhm(Complex::new(impedance, 0.0));
    // Стенд в двух точках полосы. Согласованная нагрузка не отражает волну.
    for f in [10e6, 140e6] {
        let response = calculated(&input, f);
        let expected_near = drive / (10.0 + impedance);
        let phase = -std::f64::consts::TAU * f * LENGTH_M / C0;
        let expected_far = expected_near * Complex::from_polar(1.0, phase);
        // 3e-11 А учитывает округление независимых C', L' через ε₀ и c₀.
        for (port, expected_i) in [(response.near, expected_near), (response.far, expected_far)] {
            close(port.current_peak_a, expected_i, 3e-11);
            close(
                port.voltage_peak_v,
                expected_i * impedance,
                3e-11 * impedance,
            );
        }
        assert_eq!(response.reference, PLANE);
        assert_eq!(response.frequency_hz, f);
        assert_eq!(response, calculated(&input, f));
    }
}

#[test]
fn open_and_short_have_distinct_port_conditions() {
    let mut input = fixture();
    lossless(&mut input);
    let (l, c) = independent_lc_per_m();
    let z0 = (l / c).sqrt();
    // Стенд на 100 МГц. MIT 6.013 §14.5: открытая линия Zin = −jZ₀ cot θ,
    // короткозамкнутая Zin = jZ₀ tan θ. Входной ток = E/(Zs + Zin).
    let f = 100e6;
    let tan = (std::f64::consts::TAU * f * LENGTH_M / C0).tan();
    input.termination.as_mut().unwrap().value.load = CableLoad::Open;
    input.limit.as_mut().unwrap().value.frequency_hz = f;
    // Синтетический порог 5 мА разделяет Iд=0 и Iб≈10,7 мА при разомкнутой нагрузке.
    input.limit.as_mut().unwrap().value.max_peak_a = 0.005;
    let open = calculated(&input, f);
    close(open.far.current_peak_a, 0.0.into(), 0.0);
    close(
        open.near.current_peak_a,
        Complex::new(1.0, 0.0) / Complex::new(10.0, -z0 / tan),
        3e-11,
    );
    assert!(matches!(
        open.check,
        CableCheck::NotAnalyzed {
            margin: Some(_),
            ..
        }
    ));
    input.limit.as_mut().unwrap().value.end = CableEnd::Near;
    assert!(matches!(calculated(&input, f).check, CableCheck::Fail(_)));

    input.termination.as_mut().unwrap().value.load = CableLoad::ImpedanceOhm(0.0.into());
    let short = calculated(&input, f);
    close(short.far.voltage_peak_v, 0.0.into(), 0.0);
    close(
        short.near.current_peak_a,
        Complex::new(1.0, 0.0) / Complex::new(10.0, z0 * tan),
        3e-11,
    );
    // Идеальная ЭДС непосредственно на закоротке при DC не даёт определённого тока.
    input.source.as_mut().unwrap().value.impedance_ohm = 0.0.into();
    unanalysed(&input, 0.0);
}

#[test]
fn incomplete_data_and_out_of_domain_points_stay_open() {
    let input = fixture();
    for variant in [
        CableInput {
            geometry: None,
            ..input
        },
        CableInput {
            band: None,
            ..input
        },
        CableInput {
            source: None,
            ..input
        },
        CableInput {
            termination: None,
            ..input
        },
        CableInput {
            losses: None,
            ..input
        },
    ] {
        unanalysed(&variant, 0.0);
    }
    let mut variant = input;
    variant.termination.as_mut().unwrap().source = " ";
    unanalysed(&variant, 0.0);
    variant = input;
    variant.termination.as_mut().unwrap().value.reference = "другая/плоскость";
    unanalysed(&variant, 0.0);
    variant = input;
    variant.geometry.as_mut().unwrap().value.reference = "";
    unanalysed(&variant, 0.0);
    variant = input;
    variant.geometry.as_mut().unwrap().value.uniform_over_plane = false;
    unanalysed(&variant, 0.0);
    variant = input;
    variant.band.as_mut().unwrap().value.min_frequency_hz = 1e6;
    variant.band.as_mut().unwrap().value.max_frequency_hz = 10e6;
    unanalysed(&variant, 999_999.0);
    calculated(&variant, 1e6);
    calculated(&variant, 10e6);
    unanalysed(&variant, 10_000_001.0);

    // Частота λ/10 для L=0,2 м в воздухе равна 149,896229 МГц.
    variant = input;
    // С сопротивлением стенда фактический фазовый набег растёт и на
    // частоте чуть ниже предела линии без потерь уже превышает 36°.
    unanalysed(&variant, 149_896_228.0);
    lossless(&mut variant);
    calculated(&variant, 149_896_228.0);
    unanalysed(&variant, 149_896_230.0);
    // Такой же электрический предел по поперечному размеру при коротком участке.
    variant.geometry.as_mut().unwrap().value.length_m = 0.01;
    variant.geometry.as_mut().unwrap().value.axis_height_m = 0.196;
    calculated(&variant, 149_896_228.0);
    unanalysed(&variant, 149_896_230.0);
    for limit in [
        None,
        Some(Sourced {
            source: "",
            ..input.limit.unwrap()
        }),
        sourced(CableLimit {
            frequency_hz: 1e6,
            ..input.limit.unwrap().value
        }),
        sourced(CableLimit {
            reference: "другая/плоскость",
            ..input.limit.unwrap().value
        }),
    ] {
        let variant = CableInput { limit, ..input };
        assert!(matches!(
            calculated(&variant, 0.0).check,
            CableCheck::NotAnalyzed { margin: None, .. }
        ));
    }
}

#[test]
fn source_scaling_keeps_representable_port_voltage() {
    // Крайний численный стенд. Закон Ома даёт I = 10⁻⁴⁰⁰ А за пределами
    // представления f64, а напряжение нагрузки остаётся 10⁻³⁰⁰ В.
    let mut input = fixture();
    input.source.as_mut().unwrap().value.peak_v = 1e-300.into();
    input.termination.as_mut().unwrap().value.load = CableLoad::ImpedanceOhm(1e100.into());
    for port in {
        let result = calculated(&input, 0.0);
        [result.near, result.far]
    } {
        close(port.voltage_peak_v * 1e300, 1.0.into(), ROUNDING);
    }
}

#[test]
fn every_numeric_input_is_validated_even_if_other_data_are_missing() {
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let input = fixture();
        assert!(common_mode_current(&input, bad).is_err());
        // Макрос меняет ровно одно поле за опыт. Пропущенный соседний вход
        // или пустой источник проверяемого поля не должны скрывать ошибку.
        macro_rules! rejects {
            ($field:ident, $($part:ident).+) => {{
                let mut variant = input;
                variant.$field.as_mut().unwrap().value.$($part).+ = bad;
                variant.$field.as_mut().unwrap().source = "";
                assert!(common_mode_current(&variant, 0.0).is_err(), stringify!($field.$($part).+));
            }};
        }
        rejects!(geometry, length_m);
        rejects!(geometry, radius_m);
        rejects!(geometry, axis_height_m);
        rejects!(geometry, relative_permittivity);
        rejects!(losses, series_resistance_ohm_per_m);
        rejects!(losses, shunt_conductance_s_per_m);
        rejects!(band, min_frequency_hz);
        rejects!(band, max_frequency_hz);
        rejects!(band, max_wavelength_fraction);
        rejects!(source, peak_v.re);
        rejects!(source, peak_v.im);
        rejects!(source, impedance_ohm.re);
        rejects!(source, impedance_ohm.im);
        rejects!(limit, max_peak_a);
        rejects!(limit, frequency_hz);

        let mut variant = CableInput {
            geometry: None,
            ..input
        };
        for impedance in [Complex::new(bad, 0.0), Complex::new(0.0, bad)] {
            variant.termination.as_mut().unwrap().value.load = CableLoad::ImpedanceOhm(impedance);
            assert!(common_mode_current(&variant, 0.0).is_err());
        }
    }
    let mut input = fixture();
    input.termination.as_mut().unwrap().value.load = CableLoad::ImpedanceOhm((-1.0).into());
    assert!(common_mode_current(&input, 0.0).is_err());
    input = fixture();
    input.geometry.as_mut().unwrap().value.axis_height_m = 0.003;
    assert!(common_mode_current(&input, 0.0).is_err());
}

#[test]
fn lossy_line_matches_ngspice_on_two_ladder_grids() {
    let mut input = fixture();
    // Синтетическая паразитная проводимость, внешние последовательные Ls и Lн.
    const G: f64 = 0.002;
    const SOURCE_L: f64 = 7e-9;
    const LOAD_L: f64 = 31e-9;
    const LOAD_R: f64 = 19.0;
    // Семь частот с шагом 20 МГц на 20–140 МГц, внутри области стенда.
    const POINTS: usize = 7;
    const STEP_HZ: f64 = 20e6;
    // Два вложенных разбиения для контроля дискретизации независимой модели.
    const COARSE: usize = 32;
    let coarse = spice_ladder(COARSE, POINTS, STEP_HZ, G, SOURCE_L, LOAD_L, LOAD_R);
    let fine = spice_ladder(2 * COARSE, POINTS, STEP_HZ, G, SOURCE_L, LOAD_L, LOAD_R);
    input
        .losses
        .as_mut()
        .unwrap()
        .value
        .shunt_conductance_s_per_m = G;
    let mut max_grid_error: f64 = 0.0;
    let mut max_model_error: f64 = 0.0;
    for (index, (coarse, fine)) in coarse.iter().zip(&fine).enumerate() {
        let f = (index + 1) as f64 * STEP_HZ;
        assert_eq!(coarse[0], f);
        assert_eq!(fine[0], f);
        let omega = std::f64::consts::TAU * f;
        input.source.as_mut().unwrap().value.impedance_ohm = Complex::new(10.0, omega * SOURCE_L);
        input.termination.as_mut().unwrap().value.load =
            CableLoad::ImpedanceOhm(Complex::new(LOAD_R, omega * LOAD_L));
        let line = calculated(&input, f);
        for (i, actual) in [line.near.current_peak_a, line.far.current_peak_a]
            .into_iter()
            .enumerate()
        {
            let j = 1 + 2 * i;
            let coarse = Complex::new(coarse[j], coarse[j + 1]);
            let fine = Complex::new(fine[j], fine[j + 1]);
            max_grid_error = max_grid_error.max((coarse - fine).norm());
            max_model_error = max_model_error.max((actual - fine).norm());
        }
    }
    // Бюджет расчётного стенда: 1 мкА на изменение сетки и 1 мкА на
    // комплексную невязку анализа, то есть 0,002% от DC-порога 50 мА.
    assert!(max_grid_error < 1e-6, "невязка сеток {max_grid_error:e} А");
    assert!(
        max_model_error < 1e-6,
        "невязка модели {max_model_error:e} А"
    );
    println!(
        "ngspice AC, 20–140 МГц: Δ сеток {max_grid_error:.6e} А, Δ модели {max_model_error:.6e} А"
    );
}

fn spice_ladder(
    sections: usize,
    points: usize,
    step_hz: f64,
    g: f64,
    source_l: f64,
    load_l: f64,
    load_r: f64,
) -> Vec<[f64; 5]> {
    let (l, c) = independent_lc_per_m();
    let dx = LENGTH_M / sections as f64;
    // Нулевые источники измеряют ток, ориентированный вдоль стержня.
    let mut deck = format!(
        "Стержень над плоскостью\nVdrive drive 0 DC 0 AC 1\nRs drive ls 10\nLs ls near {source_l:.16e}\nVnear near n0 0\nVfar n{sections} far 0\nLl far load {load_l:.16e}\nRl load 0 {load_r:.16e}\n"
    );
    for k in 0..sections {
        writeln!(
            deck,
            "R{k} n{k} r{k} {:.16e}\nL{k} r{k} n{} {:.16e}",
            5.0 * dx,
            k + 1,
            l * dx
        )
        .unwrap();
    }
    for k in 0..=sections {
        // На концах симметричной π-секции по половине шунта.
        let cell = if k == 0 || k == sections {
            dx / 2.0
        } else {
            dx
        };
        writeln!(
            deck,
            "C{k} n{k} 0 {:.16e}\nG{k} n{k} 0 n{k} 0 {:.16e}",
            c * cell,
            g * cell
        )
        .unwrap();
    }
    writeln!(
        deck,
        ".control\nset numdgt=16\nset width=200\nac lin {points} {step_hz:.16e} {:.16e}",
        points as f64 * step_hz
    )
    .unwrap();
    deck.push_str("let rn=real(i(vnear))\nlet inear=imag(i(vnear))\nlet rf=real(i(vfar))\nlet ifar=imag(i(vfar))\nprint rn inear rf ifar\nquit\n.endc\n.end\n");
    let executable = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../assets/third_party/ngspice/bin/ngspice"
    );
    let mut child = Command::new(executable)
        .args(["-n", "-b"])
        .stdin(Stdio::piped())
        .stderr(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .expect("для независимой сверки нужен ngspice из комплекта проекта");
    if let Err(error) = child.stdin.take().unwrap().write_all(deck.as_bytes()) {
        let _ = child.kill();
        let _ = child.wait();
        panic!("не удалось передать схему в ngspice: {error}");
    }
    let result = child.wait_with_output().unwrap();
    assert!(
        result.status.success() && result.stderr.is_empty(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let stdout = String::from_utf8(result.stdout).unwrap();
    let rows: Vec<_> = stdout
        .lines()
        .filter_map(|line| {
            let (index, values) = line.split_once('\t')?;
            index.parse::<usize>().ok()?;
            let values: Vec<f64> = values
                .split_whitespace()
                .map(|v| v.parse().unwrap())
                .collect();
            assert!(values.iter().all(|v| v.is_finite()));
            Some(
                values
                    .try_into()
                    .expect("ожидались частота и два комплексных тока"),
            )
        })
        .collect();
    assert_eq!(rows.len(), points, "{stdout}");
    rows
}
