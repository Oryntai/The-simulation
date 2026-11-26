//! Изолированная прямоугольная петля с общей резистивной ветвью возврата.

use super::*;
use sim_circuit::emc::loop_model::{LumpedDomain, SeriesRl, fitted_path};
use sim_circuit::emc::power::{Harmonic, PeriodicLoad, PowerInput, analyze_power};
use std::f64::consts::PI;

// Геометрия синтетического стенда Т6.5. Оси провода 24 × 16 мм, радиус
// 1 мм. Два последовательных зазора 2 мм в серединах длинных сторон.
const LENGTH_MM: f64 = 24.0;
const HEIGHT_MM: f64 = 16.0;
const RADIUS_MM: f64 = 1.0;
const SOURCE_R: f64 = 25.0;
const RETURN_R: f64 = 25.0;
const FREQUENCIES: [f64; 5] = [100e6, 150e6, 200e6, 250e6, 300e6];
// Периметр 80 мм на 300 МГц ≈ 0,080 λ. r/min(a,b) = 1/16.
// Внешняя L круглого провода в вакууме по Rosa и Cohen (73), (77), (86).
const GEOMETRY_SOURCE: &str = "стенд Т6.5; Rosa–Cohen, NBS Bulletin 5(1), (73), (77), (86)";
const CIRCUIT_SOURCE: &str =
    "топология Т6.5: PEC, два зазора по 25 Ом, без отвода тока от общего возврата";

pub const FIXTURE: Fixture = Fixture {
    field,
    analytic,
    signals: &[1, 2],
};

pub fn field(grid: Grid) -> Model<'static> {
    // Геометрические якоря и границы области, мм. Узлы зондов общие для обеих сеток.
    let x = [
        -12., -6., -1., 0., 1., 5., 9., 11., 12., 13., 15., 19., 23., 24., 25., 30., 36.,
    ];
    let y = [-16., -8., -4., -2., -1., 0., 1., 2., 4., 8., 16.];
    let z = [
        -12., -8., -4., -2., -1., 0., 1., 2., 4., 8., 12., 14., 15., 16., 17., 18., 20., 24., 28.,
    ];
    let mut model = baseline(grid, [&x, &y, &z], Boundary::Mur, band(&FREQUENCIES));
    let wire = |start: [f64; 3], stop: [f64; 3]| {
        metal(Shape::Cylinder {
            start_m: start.map(mm),
            stop_m: stop.map(mm),
            radius_m: mm(RADIUS_MM),
        })
    };
    let lo = LENGTH_MM / 2.0 - RADIUS_MM;
    let hi = LENGTH_MM / 2.0 + RADIUS_MM;
    for z in [0.0, HEIGHT_MM] {
        model.solids.push(wire([0.0, 0.0, z], [lo, 0.0, z]));
        model.solids.push(wire([hi, 0.0, z], [LENGTH_MM, 0.0, z]));
    }
    for x in [0.0, LENGTH_MM] {
        model.solids.push(wire([x, 0.0, 0.0], [x, 0.0, HEIGHT_MM]));
        for z in [0.0, HEIGHT_MM] {
            // Куб со стороной 2r соединяет торцы перпендикулярных PEC-цилиндров.
            let r = RADIUS_MM;
            model.solids.push(metal(Shape::Box(region(
                [x - r, -r, z - r],
                [x + r, r, z + r],
            ))));
        }
    }
    for (z, names, resistance, drive) in [
        (HEIGHT_MM, ["u_src", "i_src"], SOURCE_R, true),
        (0.0, ["u_return", "i_return"], RETURN_R, false),
    ] {
        let mid = LENGTH_MM / 2.0;
        let contour = 2.0 * RADIUS_MM;
        port(
            &mut model,
            names,
            Axis::X,
            region([lo, 0.0, z], [hi, 0.0, z]),
            region([mid, -contour, z - contour], [mid, contour, z + contour]),
            resistance,
            drive,
        );
    }
    model
}

fn external_loop_h() -> f64 {
    // Rosa–Cohen. В PEC нет объёмной внутренней L = μ₀P/(8π).
    // Здесь только внешний интеграл Неймана. μ₀ по CODATA 2022.
    const MU0: f64 = 1.256_637_061_27e-6;
    let a = mm(LENGTH_MM);
    let b = mm(HEIGHT_MM);
    let r = mm(RADIUS_MM);
    let kernel =
        |len: f64, distance: f64| len * (len / distance).asinh() - len.hypot(distance) + distance;
    MU0 / PI * (kernel(a, r) + kernel(b, r) - kernel(a, b) - kernel(b, a))
}

pub fn analytic(frequency_hz: f64) -> Result<Analysis<Vec<Complex<f64>>>, SimError> {
    let domain = sourced(
        LumpedDomain {
            max_frequency_hz: FREQUENCIES[4],
            longest_path_m: 2.0 * mm(LENGTH_MM + HEIGHT_MM),
            min_wave_speed_m_s: C0,
            max_wavelength_fraction: 0.1,
        },
        GEOMETRY_SOURCE,
    );
    let feed = calculated!(fitted_path(
        sourced(
            SeriesRl {
                resistance_ohm: SOURCE_R + RETURN_R,
                inductance_h: external_loop_h(),
            },
            CIRCUIT_SOURCE
        ),
        domain
    )?);
    let shared = calculated!(fitted_path(
        sourced(
            SeriesRl {
                resistance_ohm: RETURN_R,
                inductance_h: 0.0,
            },
            CIRCUIT_SOURCE
        ),
        domain
    )?);
    let z_total = calculated!(feed.impedance(frequency_hz)?);
    if frequency_hz < FREQUENCIES[0] {
        return Ok(Analysis::NotAnalyzed(
            "частота ниже полосы канонической петли",
        ));
    }
    let i = z_total.inv();
    let harmonics = [Harmonic { order: 1, peak: i }];
    let response = calculated!(analyze_power(PowerInput {
        domain,
        source_v: sourced(0.0, CIRCUIT_SOURCE),
        feed: Some(feed),
        shared_return: Some(shared),
        decoupling: sourced(&[], CIRCUIT_SOURCE),
        load: sourced(
            PeriodicLoad {
                dc_a: 0.0,
                fundamental_hz: frequency_hz,
                harmonics: &harmonics,
                complete_spectrum: true,
            },
            CIRCUIT_SOURCE
        ),
    })?);
    // Зазор возврата расположен на противоположной стороне. Падение напряжения
    // измеряем от x=11 к x=13 мм, а ток контура в этом зазоре течёт в −X.
    Ok(Analysis::Calculated(vec![
        i,
        response.return_shift.harmonics()[0].peak,
    ]))
}
