//! Три локальные постановки численной сверки из Т6.5.

use std::path::Path;
use std::time::Duration;

use nalgebra::Complex;
use sim_circuit::SimError;
use sim_circuit::emc::openems::{
    Axis, Band, Boundary, GaussianPulse, LumpedPort, Material, Model, OpenEmsResult, Probe,
    ProbeKind, Region, Shape, Solid, run_openems,
};
use sim_circuit::emc::{Analysis, Sourced};

#[path = "../cable/fixture.rs"]
pub mod cable;
#[path = "../coupled_lines/fixture.rs"]
pub mod coupled_lines;
#[path = "fixture.rs"]
pub mod current_loop;

// Скорость света по определению СИ. Для всех стендов среда вакуум.
const C0: f64 = 299_792_458.0;
// Инженерная граница Т6 из DEPTH_PLAN.md.
const AGREEMENT_DB: f64 = 3.0;
// Разрешение бюджета Т6.5: отдельно отсеиваем изменения сетки от 1 дБ,
// чтобы сама дискретизация не расходовала более трети допуска Т6.
const GRID_DB: f64 = AGREEMENT_DB / 3.0;
// Отдельный бюджет стенда для невязки комплексного закона Ома на окончаниях.
const PORT_RELATIVE: f64 = 0.10;
// Нижняя граница возбуждения относительно максимума спектра ЭДС в той же постановке.
// 0,1 по амплитуде соответствует принятому уровню −20 дБ полосы импульса SetGaussExcite.
const DRIVE_FRACTION: f64 = 0.1;
// Принимаем спектр только с амплитудным отступом 20 дБ от наблюдаемой ошибки хвоста.
const TAIL_FACTOR: f64 = 10.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Grid {
    Coarse,
    Fine,
}

#[derive(Clone, Copy)]
pub struct Fixture {
    field: fn(Grid) -> Model<'static>,
    analytic: fn(f64) -> Result<Analysis<Vec<Complex<f64>>>, SimError>,
    /// Индексы зондов для сравнения в порядке выхода аналитики.
    signals: &'static [usize],
}

pub const FIXTURES: [(&str, Fixture); 3] = [
    ("loop", current_loop::FIXTURE),
    ("coupled_lines", coupled_lines::FIXTURE),
    ("cable", cable::FIXTURE),
];

#[derive(Debug)]
pub struct Comparison {
    pub coarse: OpenEmsResult,
    pub fine: OpenEmsResult,
    pub extended: OpenEmsResult,
    pub points: Vec<Point>,
    pub max_grid_db: f64,
    pub max_difference_db: f64,
    pub max_port_residual: f64,
    pub min_tail_clearance_db: f64,
    /// Диагностика сопоставления численных моделей. Статус ЭМС всегда открыт.
    pub applicability: Analysis<()>,
    pub emc: Analysis<()>,
}

#[derive(Debug)]
pub struct Point {
    pub frequency_hz: f64,
    pub signal: String,
    pub analytic: Complex<f64>,
    pub coarse: Complex<f64>,
    pub fine: Complex<f64>,
    pub difference_db: f64,
}

/// Сопоставить обе сетки и отдельный расчёт с более длинным хвостом.
pub fn compare(
    fixture: Fixture,
    executable: &Path,
    timeout: Duration,
) -> Result<Analysis<Comparison>, SimError> {
    let coarse_model = (fixture.field)(Grid::Coarse);
    let fine_model = (fixture.field)(Grid::Fine);
    let mut extended_model = fine_model.clone();
    // Два порядка запаса энергии для независимой оценки хвоста.
    extended_model.end_energy_ratio /= 100.0;
    let coarse = calculated!(run_openems(executable, &coarse_model, timeout)?);
    let fine = calculated!(run_openems(executable, &fine_model, timeout)?);
    let extended = calculated!(run_openems(executable, &extended_model, timeout)?);
    compare_spectra(fixture, &coarse_model, &fine_model, coarse, fine, extended)
}

// Расчёт без частичного результата, если исходный опыт остался открыт.
macro_rules! calculated {
    ($value:expr) => {
        match $value {
            Analysis::Calculated(value) => value,
            Analysis::NotAnalyzed(reason) => return Ok(Analysis::NotAnalyzed(reason)),
        }
    };
}
use calculated;

fn compare_spectra(
    fixture: Fixture,
    coarse_model: &Model<'_>,
    fine_model: &Model<'_>,
    coarse: OpenEmsResult,
    fine: OpenEmsResult,
    extended: OpenEmsResult,
) -> Result<Analysis<Comparison>, SimError> {
    if coarse_model.ports != fine_model.ports
        || coarse_model.solids != fine_model.solids
        || coarse_model.probes != fine_model.probes
        || coarse_model.boundaries != fine_model.boundaries
        || coarse_model.pulse != fine_model.pulse
        || coarse_model.band != fine_model.band
        || fine.frequencies_hz != coarse.frequencies_hz
        || fine.frequencies_hz != extended.frequencies_hz
        || fine.openems_version != coarse.openems_version
        || fine.openems_version != extended.openems_version
        || fine.csxcad_version != coarse.csxcad_version
        || fine.csxcad_version != extended.csxcad_version
    {
        return Ok(Analysis::NotAnalyzed(
            "постановки сеток или версии решателя различаются",
        ));
    }
    if fine.timestep_s >= coarse.timestep_s || extended.iterations <= fine.iterations {
        return Ok(Analysis::NotAnalyzed(
            "уточнение сетки или временного окна не подтверждено",
        ));
    }
    for (model, result) in [
        (coarse_model, &coarse),
        (fine_model, &fine),
        (fine_model, &extended),
    ] {
        if !model
            .probes
            .iter()
            .zip(&result.probes)
            .all(|(probe, actual)| {
                (0..3).all(|axis| {
                    let grid = &model.grid_m[axis];
                    let lo = probe.region.lower_m[axis];
                    let hi = probe.region.upper_m[axis];
                    let start = actual.snapped_start[axis];
                    let stop = actual.snapped_stop[axis];
                    match probe.kind {
                        ProbeKind::Voltage => grid[start] == lo && grid[stop] == hi,
                        ProbeKind::Current(normal) => {
                            // ProcessCurrent использует двойственную сетку, а заголовок
                            // ProcessIntegral печатает координаты основной сетки.
                            let Some(start_next) = grid.get(start + 1) else {
                                return false;
                            };
                            let Some(stop_next) = grid.get(stop + 1) else {
                                return false;
                            };
                            if axis == normal as usize {
                                start == stop && grid[start] < lo && lo <= *start_next
                            } else {
                                grid[start].midpoint(*start_next) <= lo
                                    && grid[stop].midpoint(*stop_next) >= hi
                            }
                        }
                    }
                })
            })
        {
            return Ok(Analysis::NotAnalyzed(
                "зонд смещён относительно заявленных координат",
            ));
        }
    }
    let mut max_port_residual: f64 = 0.0;
    let mut normalized = Vec::new();
    for run in [&coarse, &fine, &extended] {
        let (transfer, residual) = calculated!(normalize(fine_model, run)?);
        max_port_residual = max_port_residual.max(residual);
        normalized.push(transfer);
    }
    let mut max_grid_db: f64 = 0.0;
    let mut max_difference_db: f64 = 0.0;
    let mut min_tail_clearance_db = f64::INFINITY;
    let mut points = Vec::new();
    for (i, &frequency_hz) in fine.frequencies_hz.iter().enumerate() {
        let analytic = calculated!((fixture.analytic)(frequency_hz)?);
        if analytic.len() != fixture.signals.len() {
            return Err(invalid(
                "нет взаимно однозначного соответствия аналитики и зондов",
            ));
        }
        for (&signal, analytic) in fixture.signals.iter().zip(analytic) {
            let a = normalized[0][signal][i];
            let b = normalized[2][signal][i];
            if analytic.norm() == 0.0 || a.norm() == 0.0 || b.norm() == 0.0 {
                return Ok(Analysis::NotAnalyzed(
                    "нулевой отсчёт не разрешает сравнение в дБ",
                ));
            }
            let tail = (b - normalized[1][signal][i]).norm();
            // Две комплексные амплитуды из FD напечатаны шестью значащими цифрами.
            // Для U + RI и отношения это лишь нижняя оценка пола, добавляем также хвост.
            let clearance = db(b.norm(), tail.max(b.norm() * 1e-5))?;
            min_tail_clearance_db = min_tail_clearance_db.min(clearance);
            max_grid_db = max_grid_db.max(db(b.norm(), a.norm())?.abs());
            let difference_db = db(b.norm(), analytic.norm())?;
            max_difference_db = max_difference_db.max(difference_db.abs());
            points.push(Point {
                frequency_hz,
                signal: fine.probes[signal].name.clone(),
                analytic,
                coarse: a,
                fine: b,
                difference_db,
            });
        }
    }
    let applicability = assess(
        max_port_residual,
        min_tail_clearance_db,
        max_grid_db,
        max_difference_db,
    );
    Ok(Analysis::Calculated(Comparison {
        coarse,
        fine,
        extended,
        points,
        max_grid_db,
        max_difference_db,
        max_port_residual,
        min_tail_clearance_db,
        applicability,
        emc: Analysis::NotAnalyzed("нужен стендовый замер на тех же портах и в той же полосе"),
    }))
}

fn assess(port_residual: f64, tail_db: f64, grid_db: f64, difference_db: f64) -> Analysis<()> {
    if port_residual > PORT_RELATIVE {
        Analysis::NotAnalyzed("на окончаниях не подтверждены направление или нормировка U/I")
    } else if tail_db < 20.0 * TAIL_FACTOR.log10() {
        Analysis::NotAnalyzed("полезный сигнал не отделён от ошибки хвоста")
    } else if grid_db > GRID_DB {
        Analysis::NotAnalyzed("изменение сетки превышает бюджет 1 дБ")
    } else if difference_db > AGREEMENT_DB {
        Analysis::NotAnalyzed("разница аналитики и поля превышает 3 дБ")
    } else {
        Analysis::Calculated(())
    }
}

type Spectra = Vec<Vec<Complex<f64>>>;

fn normalize(model: &Model<'_>, run: &OpenEmsResult) -> Result<Analysis<(Spectra, f64)>, SimError> {
    let mut result = vec![Vec::new(); run.probes.len()];
    let mut residual: f64 = 0.0;
    let mut drive_min = f64::INFINITY;
    let mut drive_max: f64 = 0.0;
    let source_r = model.ports[0].resistance_ohm.unwrap();
    let source_floor =
        model.probes[0].floor.unwrap().value + source_r * model.probes[1].floor.unwrap().value;
    for i in 0..run.frequencies_hz.len() {
        let v = |port: usize| run.probes[2 * port].values[i];
        let current = |port: usize| run.probes[2 * port + 1].values[i];
        // I положителен от минусовой к плюсовой клемме внутри зазора.
        // На источнике это отдаваемый ток, на пассивной ветви U = −RI.
        let source_drop = source_r * current(0);
        let drive = v(0) + source_drop;
        let magnitude = drive.norm();
        // Шесть значащих цифр ProcessIntegral::Dump_FD_Data. В сумме U + RI
        // абсолютные погрешности слагаемых могут сложиться даже при их компенсации.
        let drive_resolution = source_floor + 5e-6 * (v(0).norm() + source_drop.norm());
        if !magnitude.is_finite() || !drive_resolution.is_finite() {
            return Err(invalid("переполнение при восстановлении ЭДС"));
        }
        if magnitude <= drive_resolution {
            return Ok(Analysis::NotAnalyzed(
                "восстановленная ЭДС на уровне спектрального пола",
            ));
        }
        drive_min = drive_min.min(magnitude);
        drive_max = drive_max.max(magnitude);
        for (p, port) in model.ports.iter().enumerate().skip(1) {
            let drop = port.resistance_ohm.unwrap() * current(p);
            residual = residual.max((v(p) + drop).norm() / v(p).norm().max(drop.norm()));
        }
        for (target, probe) in result.iter_mut().zip(&run.probes) {
            let value = probe.values[i] / drive;
            if !value.norm().is_finite() {
                return Err(invalid("переполнение нормированной передачи"));
            }
            target.push(value);
        }
    }
    if drive_min < DRIVE_FRACTION * drive_max {
        return Ok(Analysis::NotAnalyzed(
            "спектр восстановленной ЭДС не покрывает полосу",
        ));
    }
    Ok(Analysis::Calculated((result, residual)))
}

fn db(a: f64, b: f64) -> Result<f64, SimError> {
    if !a.is_finite() || !b.is_finite() || a <= 0.0 || b <= 0.0 {
        return Err(invalid(
            "для разности в дБ нужны конечные ненулевые амплитуды",
        ));
    }
    Ok(20.0 * (a.log10() - b.log10()))
}

fn invalid(reason: &str) -> SimError {
    SimError::Invalid(reason.to_owned())
}

fn sourced<T>(value: T, source: &'static str) -> Option<Sourced<'static, T>> {
    Some(Sourced { value, source })
}

fn mm(x: f64) -> f64 {
    x / 1000.0
}

fn region(lower: [f64; 3], upper: [f64; 3]) -> Region {
    Region {
        lower_m: lower.map(mm),
        upper_m: upper.map(mm),
    }
}

fn metal(shape: Shape) -> Solid {
    Solid {
        shape,
        material: Material::Pec,
        // Контакт порта имеет приоритет над краями PEC на тех же координатах.
        priority: 10,
    }
}

fn baseline(grid: Grid, axes: [&[f64]; 3], ground: Boundary, band: Band) -> Model<'static> {
    let [lo, hi] = band.range_hz;
    // Четыре слоя PML по минимуму openEMS SetBoundaryCond. Шаг внешней
    // ячейки продолжается с последнего интервала внутренней сетки.
    const PML_CELLS: u8 = 4;
    const ABSORB: Boundary = Boundary::Pml(PML_CELLS);
    let bottom = if ground == Boundary::Pec {
        ground
    } else {
        ABSORB
    };
    let boundaries = [ABSORB, ABSORB, ABSORB, ABSORB, bottom, ABSORB];
    let grid_m = std::array::from_fn(|axis| {
        let anchors = axes[axis];
        let layers = |boundary| {
            if boundary == ABSORB {
                i32::from(PML_CELLS)
            } else {
                0
            }
        };
        let left = anchors[1] - anchors[0];
        let mut result = (1..=layers(boundaries[2 * axis]))
            .rev()
            .map(|i| mm(anchors[0] - f64::from(i) * left))
            .collect::<Vec<_>>();
        result.push(mm(anchors[0]));
        for pair in anchors.windows(2) {
            if grid == Grid::Fine {
                result.push(mm((pair[0] + pair[1]) / 2.0));
            }
            result.push(mm(pair[1]));
        }
        let last = anchors.len() - 1;
        let right = anchors[last] - anchors[last - 1];
        result.extend(
            (1..=layers(boundaries[2 * axis + 1]))
                .map(|i| mm(anchors[last] + f64::from(i) * right)),
        );
        result
    });
    Model {
        grid_m,
        boundaries,
        solids: Vec::new(),
        ports: Vec::new(),
        probes: Vec::new(),
        pulse: GaussianPulse {
            center_hz: (hi + lo) / 2.0,
            // Более широкий импульс удерживает отношение периода Найквиста
            // к шагу Yee ниже порога 1000 из openEMS SetupFDTD.
            cutoff_hz: 3.0 * (hi - lo),
        },
        band,
        // Лимит стенда. Затухание проверяет адаптер, отдельно от лимита шагов.
        max_steps: 100_000,
        end_energy_ratio: 1e-5,
    }
}

fn band(frequencies_hz: &[f64]) -> Band {
    Band {
        range_hz: [frequencies_hz[0], *frequencies_hz.last().unwrap()],
        frequencies_hz: frequencies_hz.to_vec(),
        // Минимальное окно 1/B для полосы импульса шириной B. Отсчёты
        // внутри полосы вычисляются непосредственно на общей сетке частот.
        max_spacing_hz: frequencies_hz[frequencies_hz.len() - 1] - frequencies_hz[0],
    }
}

fn port(
    model: &mut Model<'static>,
    names: [&'static str; 2],
    axis: Axis,
    gap: Region,
    contour: Region,
    r: f64,
    drive: bool,
) {
    let direction = axis as usize;
    model.ports.push(LumpedPort {
        region: gap,
        axis,
        priority: 20,
        resistance_ohm: Some(r),
        // Отрицательное поле источника и линия U = −∫E·dl по AddLumpedPort.m.
        // Номинальный интеграл поля 1 В. Фактическая ЭДС восстанавливается по U и I.
        excitation_v_per_m: drive.then(|| -1.0 / (gap.upper_m[direction] - gap.lower_m[direction])),
    });
    // Численный порог разбора FD для этого синтетического стенда: в 10⁶ раз ниже
    // масштаба В/ГГц и в 10⁶ раз ниже масштаба (В/100 Ом)/ГГц. Фактический пол
    // нормированной передачи дополнительно проверяется продлённым расчётом.
    model.probes.extend([
        Probe {
            name: names[0],
            kind: ProbeKind::Voltage,
            region: gap,
            weight: -1.0,
            floor: sourced(1e-15, "порог чтения стенда Т6.5, В·с"),
        },
        Probe {
            name: names[1],
            kind: ProbeKind::Current(axis),
            region: contour,
            weight: 1.0,
            floor: sourced(1e-17, "порог чтения стенда Т6.5, А·с"),
        },
    ]);
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_circuit::emc::openems::prepare_openems;

    #[test]
    fn geometries_ports_loads_and_band_are_shared() {
        for (name, fixture) in FIXTURES {
            let coarse = (fixture.field)(Grid::Coarse);
            let fine = (fixture.field)(Grid::Fine);
            assert_eq!(coarse.ports, fine.ports, "{name}");
            assert_eq!(coarse.solids, fine.solids, "{name}");
            assert_eq!(coarse.probes, fine.probes, "{name}");
            assert_eq!(coarse.band, fine.band, "{name}");
            assert_eq!(
                coarse
                    .ports
                    .iter()
                    .filter(|p| p.excitation_v_per_m.is_some())
                    .count(),
                1
            );
            assert_eq!(coarse.probes.len(), 2 * coarse.ports.len());
            assert_ne!(coarse.grid_m, fine.grid_m, "{name}");
            for (axis, (a, b)) in coarse.grid_m.iter().zip(&fine.grid_m).enumerate() {
                let layers = |side| match coarse.boundaries[2 * axis + side] {
                    Boundary::Pml(n) => usize::from(n),
                    _ => 0,
                };
                let (left, right) = (layers(0), layers(1));
                assert_eq!(a[..left], b[..left]);
                assert_eq!(a[a.len() - right..], b[b.len() - right..]);
                assert!(a.iter().all(|coordinate| b.contains(coordinate)));
            }
            for model in [&coarse, &fine] {
                assert!(matches!(
                    prepare_openems(model).unwrap(),
                    Analysis::Calculated(_)
                ));
            }
            for f in coarse.band.frequencies_hz {
                let Analysis::Calculated(values) = (fixture.analytic)(f).unwrap() else {
                    panic!("{name}: аналитика открыта на {f} Гц");
                };
                assert_eq!(values.len(), fixture.signals.len());
                assert!(
                    values
                        .iter()
                        .all(|v| v.norm().is_finite() && v.norm() > 0.0)
                );
            }
            for f in [coarse.band.range_hz[0] - 1.0, coarse.band.range_hz[1] + 1.0] {
                assert!(
                    matches!((fixture.analytic)(f), Ok(Analysis::NotAnalyzed(_))),
                    "{name} {f}"
                );
            }
            for f in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                assert!((fixture.analytic)(f).is_err(), "{name}");
            }
        }
    }

    #[test]
    fn unresolved_cases_do_not_count_as_agreement() {
        // Приёмка Т6 по амплитуде: 3 дБ включительно. По обе стороны
        // используем 2,9 и 3,1 дБ, то есть запас +0,1 и −0,1 дБ.
        assert_eq!(assess(0.0, 40.0, 0.0, 2.9), Analysis::Calculated(()));
        assert_eq!(assess(0.0, 40.0, 0.0, 3.0), Analysis::Calculated(()));
        assert!(matches!(
            assess(0.0, 40.0, 0.0, 3.1),
            Analysis::NotAnalyzed(_)
        ));
        assert!(matches!(
            assess(0.0, 19.9, 0.0, 2.9),
            Analysis::NotAnalyzed(_)
        ));
        assert!(matches!(
            assess(0.0, 40.0, 1.1, 2.9),
            Analysis::NotAnalyzed(_)
        ));
        assert!(matches!(
            assess(0.11, 40.0, 0.0, 2.9),
            Analysis::NotAnalyzed(_)
        ));
        let missing = compare(
            current_loop::FIXTURE,
            Path::new("/dev/null/openEMS"),
            Duration::from_secs(1),
        );
        assert!(
            matches!(missing, Ok(Analysis::NotAnalyzed(reason)) if reason.contains("недоступен"))
        );
    }

    #[test]
    fn audit_t6_drive_below_floor_after_cancellation() {
        let model = current_loop::field(Grid::Fine);
        let frequencies_hz = model.band.frequencies_hz.clone();
        // Синтетические отсчёты: 1 пА·с на токовом зонде и 25 пВ·с
        // на его 25-омном порту. Оба зонда выше своих полов, но разность
        // 10⁻¹⁷ В·с меньше 10⁻¹⁵ В·с даже по одному зонду напряжения.
        let current_a_s = 1e-12;
        let volt_v_s = model.ports[0].resistance_ohm.unwrap() * current_a_s;
        let mut probes: Vec<_> = model
            .probes
            .iter()
            .map(|probe| {
                let value = match probe.kind {
                    ProbeKind::Voltage => -volt_v_s,
                    ProbeKind::Current(_) => current_a_s,
                };
                assert!(value.abs() > probe.floor.unwrap().value);
                sim_circuit::emc::openems::ProbeSpectrum {
                    name: probe.name.into(),
                    snapped_start: [0; 3],
                    snapped_stop: [0; 3],
                    values: vec![Complex::new(value, 0.0); frequencies_hz.len()],
                }
            })
            .collect();
        probes[0].values.fill(Complex::new(volt_v_s, 0.0));
        let mut run = OpenEmsResult {
            input_xml: String::new(),
            openems_version: String::new(),
            csxcad_version: String::new(),
            iterations: 0,
            timestep_s: 0.0,
            frequencies_hz,
            probes,
        };
        let Analysis::Calculated((transfer, residual)) = normalize(&model, &run).unwrap() else {
            panic!("контрольная ЭДС выше пола");
        };
        assert_eq!(residual, 0.0);
        assert!((transfer[0][0].re - 0.5).abs() <= f64::EPSILON);
        run.probes[0]
            .values
            .fill(Complex::new(-volt_v_s + 1e-17, 0.0));
        assert!(
            matches!(normalize(&model, &run), Ok(Analysis::NotAnalyzed(reason)) if reason.contains("пола"))
        );
    }

    #[test]
    fn loop_and_cable_match_independent_calculation() {
        // Петля: квадратура Неймана по 10000 серединам интервалов даёт
        // 35,73869916 нГн. На 200 МГц баланс цепи даёт
        // 25/(50 + j·2π·200 МГц·L) ≈ 0,2767348167 − j·0,2485663887.
        // Допуск 2e−9 В/В включает ошибку квадратуры 0,13 фГн и печать результата.
        let expected = Complex::new(0.2767348167, -0.2485663887);
        let Analysis::Calculated(loop_values) = current_loop::analytic(200e6).unwrap() else {
            panic!("нет аналитики петли на 200 МГц");
        };
        assert!((loop_values[1] - expected).norm() < 2e-9);
        assert!((25.0 * loop_values[0] - expected).norm() < 2e-9);

        // Кабель: складываем падающую и отражённую волны по MIT 6.013 §14.5.
        // Из ln(4+√15) и μ₀c/(2π) получается Z₀ = 123,72057415 Ом.
        // Граница 1e−11 А/В допускает округление μ₀ и Z₀ в независимой формуле.
        let z = 123.72057415;
        let r = 100.0;
        let reflection = (r - z) / (r + z);
        for frequency in cable::field(Grid::Coarse).band.frequencies_hz {
            let propagation =
                Complex::from_polar(1.0, -std::f64::consts::TAU * frequency * mm(40.0) / C0);
            let reflected = reflection * propagation * propagation;
            let forward = (1.0 + reflected + (r / z) * (1.0 - reflected)).inv();
            let near = forward * (1.0 - reflected) / z;
            let far = forward * propagation * ((1.0 - reflection) / z);
            let Analysis::Calculated(values) = cable::analytic(frequency).unwrap() else {
                panic!("нет аналитики кабеля на {frequency} Гц");
            };
            assert!((values[0] - near).norm() < 1e-11);
            assert!((values[1] + far).norm() < 1e-11);
        }
    }

    #[test]
    #[ignore = "нужен установленный openEMS 0.37"]
    fn loop_openems() {
        run_fixture("loop", current_loop::FIXTURE);
    }

    #[test]
    #[ignore = "нужен установленный openEMS 0.37"]
    fn coupled_lines_openems() {
        run_fixture("coupled_lines", coupled_lines::FIXTURE);
    }

    #[test]
    #[ignore = "нужен установленный openEMS 0.37"]
    fn cable_openems() {
        run_fixture("cable", cable::FIXTURE);
    }

    #[test]
    #[ignore = "нужен установленный openEMS 0.37"]
    fn audit_t6_intermediate_frequencies() {
        fn densify(mut model: Model<'static>) -> Model<'static> {
            // Восемь частей каждого исходного интервала для проверки
            // внутри него. ДПФ считается в дополнительных точках того же
            // окна, поэтому новых независимых полос разрешения это не даёт.
            const DIVISIONS: u32 = 8;
            let originals = &model.band.frequencies_hz;
            model.band.frequencies_hz = originals
                .windows(2)
                .flat_map(|w| {
                    (0..DIVISIONS)
                        .map(move |i| w[0] + (w[1] - w[0]) * (f64::from(i) / f64::from(DIVISIONS)))
                })
                .chain(originals.last().copied())
                .collect();
            model
        }

        let exe = executable();
        for ((name, fixture), field) in FIXTURES.into_iter().zip([
            (|grid| densify(current_loop::field(grid))) as fn(Grid) -> Model<'static>,
            |grid| densify(coupled_lines::field(grid)),
            |grid| densify(cable::field(grid)),
        ]) {
            // Тот же лимит отдельного процесса, что у исходного стенда.
            let dense = Fixture { field, ..fixture };
            let Analysis::Calculated(comparison) =
                compare(dense, &exe, Duration::from_secs(60)).unwrap()
            else {
                panic!("{name}: не получено сравнение на уплотнённой сетке частот");
            };
            assert_eq!(comparison.applicability, Analysis::Calculated(()), "{name}");
            assert!(matches!(comparison.emc, Analysis::NotAnalyzed(_)));
            let requested = field(Grid::Fine).band.frequencies_hz;
            assert_eq!(requested.len(), (5 - 1) * 8 + 1);
            assert_eq!(comparison.extended.frequencies_hz, requested);
            assert_eq!(
                comparison.points.len(),
                requested.len() * fixture.signals.len()
            );
            let mut maximum = 0.0_f64;
            for (point, frequency) in comparison.points.iter().zip(
                requested
                    .iter()
                    .flat_map(|f| std::iter::repeat_n(*f, fixture.signals.len())),
            ) {
                assert_eq!(point.frequency_hz, frequency);
                // Независимый пересчёт из квадратов модулей в каждой точке.
                let db = 10.0 * (point.fine.norm_sqr() / point.analytic.norm_sqr()).log10();
                assert!(
                    db.is_finite() && db.abs() <= AGREEMENT_DB,
                    "{name}: {point:?}"
                );
                maximum = maximum.max(db.abs());
            }
            println!(
                "{name}: {} частот, {} сравнений, Δ={maximum:.6} дБ, сетка={:.6} дБ, порт={:.6}%, пол={:.6} дБ",
                requested.len(),
                comparison.points.len(),
                comparison.max_grid_db,
                100.0 * comparison.max_port_residual,
                comparison.min_tail_clearance_db
            );
        }
    }

    fn executable() -> std::path::PathBuf {
        std::env::var_os("OPENEMS")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|| {
                std::path::PathBuf::from(std::env::var_os("HOME").unwrap())
                    .join("opt/openEMS/bin/openEMS")
            })
    }

    fn run_fixture(name: &str, fixture: Fixture) {
        let exe = executable();
        // 60 с — бюджет одного процесса стенда.
        let answer = compare(fixture, &exe, Duration::from_secs(60));
        let c = match answer {
            Ok(Analysis::Calculated(c)) => c,
            Ok(Analysis::NotAnalyzed(reason)) => panic!("{name}: NotAnalyzed, {reason}"),
            Err(error) => panic!("{name}: {error}"),
        };
        println!(
            "{name}: Δ={:.6} дБ, сетка={:.6} дБ, порт={:.6}%, пол={:.6} дБ; {} / {}, шаги {} / {} / {}; {:?}; ЭМС {:?}",
            c.max_difference_db,
            c.max_grid_db,
            100.0 * c.max_port_residual,
            c.min_tail_clearance_db,
            c.fine.openems_version,
            c.fine.csxcad_version,
            c.coarse.iterations,
            c.fine.iterations,
            c.extended.iterations,
            c.applicability,
            c.emc,
        );
        for p in &c.points {
            println!(
                "{} {:e} Гц: модель {:e}, поле {:e}, Δ {:.6} дБ",
                p.signal,
                p.frequency_hz,
                p.analytic.norm(),
                p.fine.norm(),
                p.difference_db
            );
        }
        assert_eq!(c.applicability, Analysis::Calculated(()));
        assert!(matches!(c.emc, Analysis::NotAnalyzed(_)));
    }
}
