//! Две одинаковые PEC-полоски над общей PEC-опорой в вакууме.

use super::*;
use sim_circuit::emc::coupling::{
    CouplingBand, CouplingPorts, MicrostripPair, ParallelRc, coupled_microstrip,
};

// Синтетический стенд Т6.5: 36 × 3 мм, зазор 2 мм, высота 3 мм.
// w/h = 1, g/h = 2/3. На 600 МГц длина ≈ 0,072 λ.
// Окончания 35 Ом, с фактическим рассогласованием мод: дальняя наводка наблюдаема.
const LENGTH_MM: f64 = 36.0;
const WIDTH_MM: f64 = 3.0;
const GAP_MM: f64 = 2.0;
const HEIGHT_MM: f64 = 3.0;
const R: f64 = 35.0;
const FREQUENCIES: [f64; 5] = [200e6, 300e6, 400e6, 500e6, 600e6];
const SOURCE: &str =
    "стенд Т6.5, вакуумные равномерные полоски с общей опорой; Kirschning–Jansen (1984)";

pub const FIXTURE: Fixture = Fixture {
    field,
    analytic,
    signals: &[0, 2, 4, 6],
};

pub fn field(grid: Grid) -> Model<'static> {
    let x = [
        -12., -6., -2., 0., 2., 6., 12., 18., 24., 30., 34., 36., 38., 42., 48.,
    ];
    let y = [
        -18., -12., -8., -5., -4., -3.5, -2.5, -1.5, -1., 0., 1., 1.5, 2.5, 3.5, 4., 5., 8., 12.,
        18.,
    ];
    let z = [0., 1.5, 3., 4.5, 6., 9., 14., 22.];
    // Первый расчёт на исходной сетке показал дрейф 1,505 дБ. Для обеих
    // рабочих сеток удваиваем разрешение, а на второй дополнительно
    // уточняем поперечное сечение полосок и зазоров около опоры.
    let mut model = baseline(Grid::Fine, [&x, &y, &z], Boundary::Pec, band(&FREQUENCIES));
    // На уточнённой сетке Δt ≈ 0,239 пс. При fmax=2,4 ГГц число шагов
    // Найквиста ≈ 873, ниже порога 1000 из openEMS SetupFDTD.
    model.pulse.cutoff_hz = 2.0e9;
    if grid == Grid::Fine {
        // Границы локального уточнения из якорей: боковой ореол до |y|=5 мм,
        // по высоте до 2h. Поглощающий слой и плоскости портов совпадают.
        bisect_near(&mut model.grid_m[1], -5.0, 5.0);
        bisect_near(&mut model.grid_m[2], 0.0, 2.0 * HEIGHT_MM);
    }
    let center = (WIDTH_MM + GAP_MM) / 2.0;
    for y in [-center, center] {
        let half = WIDTH_MM / 2.0;
        model.solids.push(metal(Shape::Box(region(
            [0.0, y - half, HEIGHT_MM],
            [LENGTH_MM, y + half, HEIGHT_MM],
        ))));
    }
    for (x, y, names, driven) in [
        (0.0, -center, ["u_aggressor_near", "i_aggressor_near"], true),
        (0.0, center, ["u_victim_near", "i_victim_near"], false),
        (
            LENGTH_MM,
            -center,
            ["u_aggressor_far", "i_aggressor_far"],
            false,
        ),
        (LENGTH_MM, center, ["u_victim_far", "i_victim_far"], false),
    ] {
        // Ток измеряется в плоскости середины вертикального зазора.
        // Контур охватывает окончание одной полоски и исключает соседнее.
        let z = HEIGHT_MM / 2.0;
        port(
            &mut model,
            names,
            Axis::Z,
            region([x, y, 0.0], [x, y, HEIGHT_MM]),
            region([x - 2.0, y - 1.0, z], [x + 2.0, y + 1.0, z]),
            R,
            driven,
        );
    }
    model
}

fn bisect_near(axis: &mut Vec<f64>, lo_mm: f64, hi_mm: f64) {
    let mut refined = Vec::new();
    for pair in axis.windows(2) {
        refined.push(pair[0]);
        if pair[0] >= mm(lo_mm) && pair[1] <= mm(hi_mm) {
            refined.push(pair[0].midpoint(pair[1]));
        }
    }
    refined.push(*axis.last().unwrap());
    *axis = refined;
}

pub fn analytic(frequency_hz: f64) -> Result<Analysis<Vec<Complex<f64>>>, SimError> {
    let pair = calculated!(coupled_microstrip(
        sourced(
            MicrostripPair {
                length_m: mm(LENGTH_MM),
                width_m: mm(WIDTH_MM),
                gap_m: mm(GAP_MM),
                height_m: mm(HEIGHT_MM),
                thickness_m: 0.0,
                relative_permittivity: 1.0,
                uniform_over_plane: true,
            },
            SOURCE
        ),
        sourced(
            CouplingBand {
                min_frequency_hz: FREQUENCIES[0],
                max_frequency_hz: FREQUENCIES[4],
                max_wavelength_fraction: 0.1,
            },
            SOURCE
        )
    )?);
    let load = ParallelRc {
        resistance_ohm: R,
        capacitance_f: 0.0,
    };
    let transfer = calculated!(pair.transfer(
        sourced(
            CouplingPorts {
                aggressor_near: load,
                victim_near: load,
                aggressor_far: load,
                victim_far: load,
            },
            SOURCE
        ),
        frequency_hz
    )?);
    Ok(Analysis::Calculated(vec![
        transfer.aggressor_near,
        transfer.victim_near,
        transfer.aggressor_far,
        transfer.victim_far,
    ]))
}
