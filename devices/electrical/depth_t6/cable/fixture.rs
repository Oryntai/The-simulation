//! Одномодовый круглый PEC-провод над общей PEC-опорой.

use super::*;
use sim_circuit::emc::cable::{
    CableBand, CableGeometry, CableInput, CableLoad, CableLosses, CableSource, CableTermination,
    common_mode_current,
};

// Геометрия синтетического стенда Т6.5. Длина 40 мм, радиус 1 мм,
// ось на высоте 4 мм. На 600 МГц длина ≈ 0,080 λ, поперечник ≤ 0,010 λ.
// Нагрузки по 100 Ом. По методу изображений Z₀ ≈ 123,7 Ом (MIT 6.013, 4.7).
const LENGTH_MM: f64 = 40.0;
const RADIUS_MM: f64 = 1.0;
const HEIGHT_MM: f64 = 4.0;
const R: f64 = 100.0;
const FREQUENCIES: [f64; 5] = [200e6, 300e6, 400e6, 500e6, 600e6];
const REFERENCE: &str = "pec_z0";
const SOURCE: &str =
    "стенд Т6.5; круглый PEC над плоской PEC-опорой, вакуум, метод изображений MIT 6.013 §4.7";

pub const FIXTURE: Fixture = Fixture {
    field,
    analytic,
    signals: &[1, 3],
};

pub fn field(grid: Grid) -> Model<'static> {
    let x = [
        -12., -6., -2., 0., 2., 6., 12., 20., 28., 34., 38., 40., 42., 46., 52.,
    ];
    let y = [-20., -12., -6., -3., -2., -1., 0., 1., 2., 3., 6., 12., 20.];
    let z = [0., 1., 2., 3., 4., 5., 6., 9., 14., 22.];
    let mut model = baseline(grid, [&x, &y, &z], Boundary::Pec, band(&FREQUENCIES));
    model.solids.push(metal(Shape::Cylinder {
        start_m: [0.0, 0.0, mm(HEIGHT_MM)],
        stop_m: [mm(LENGTH_MM), 0.0, mm(HEIGHT_MM)],
        radius_m: mm(RADIUS_MM),
    }));
    // Вертикальный порт приходит к нижней образующей цилиндра. Плоскости
    // сдвига вдоль продольной оси нет. Нормаль токового контура +Z.
    let contact = HEIGHT_MM - RADIUS_MM;
    for (x, names, drive) in [
        (0.0, ["u_near", "i_near"], true),
        (LENGTH_MM, ["u_far", "i_far"], false),
    ] {
        port(
            &mut model,
            names,
            Axis::Z,
            region([x, 0.0, 0.0], [x, 0.0, contact]),
            region([x - 2.0, -2.0, RADIUS_MM], [x + 2.0, 2.0, RADIUS_MM]),
            R,
            drive,
        );
    }
    model
}

pub fn analytic(frequency_hz: f64) -> Result<Analysis<Vec<Complex<f64>>>, SimError> {
    let response = calculated!(common_mode_current(
        &CableInput {
            geometry: sourced(
                CableGeometry {
                    length_m: mm(LENGTH_MM),
                    radius_m: mm(RADIUS_MM),
                    axis_height_m: mm(HEIGHT_MM),
                    relative_permittivity: 1.0,
                    reference: REFERENCE,
                    uniform_over_plane: true,
                },
                SOURCE
            ),
            losses: sourced(
                CableLosses {
                    series_resistance_ohm_per_m: 0.0,
                    shunt_conductance_s_per_m: 0.0,
                },
                SOURCE
            ),
            band: sourced(
                CableBand {
                    min_frequency_hz: FREQUENCIES[0],
                    max_frequency_hz: FREQUENCIES[4],
                    max_wavelength_fraction: 0.1,
                },
                SOURCE
            ),
            source: sourced(
                CableSource {
                    reference: REFERENCE,
                    peak_v: 1.0.into(),
                    impedance_ohm: R.into(),
                },
                SOURCE
            ),
            termination: sourced(
                CableTermination {
                    reference: REFERENCE,
                    load: CableLoad::ImpedanceOhm(R.into()),
                },
                SOURCE
            ),
            limit: None,
        },
        frequency_hz
    )?);
    // Ток от ближнего к дальнему порту на дальнем окончании направлен в −Z.
    Ok(Analysis::Calculated(vec![
        response.near.current_peak_a,
        -response.far.current_peak_a,
    ]))
}
