#![cfg(unix)]

use std::f64::consts::TAU;
use std::fs;
use std::os::unix::fs::{DirBuilderExt, PermissionsExt};
use std::path::{Path, PathBuf};
use std::time::Duration;

use nalgebra::Complex;
use sim_circuit::emc::openems::*;
use sim_circuit::emc::{Analysis, Sourced};

fn model() -> Model<'static> {
    // Синтетический куб 10 мм с шагом 1 мм. Зазор порта 2 мм, нагрузка 50 Ом.
    // Импульс 1,5 ± 0,6 ГГц, расчётная полоса 1–2 ГГц с шагом 0,5 ГГц.
    // Ручная проверка окна: 1/(0,5 ГГц) = 2 нс. Строки ровно на краях и внутри полосы.
    // Число шагов и затухание заданы для этого синтетического опыта.
    let grid = (0..=10).map(|i| f64::from(i) / 1000.0).collect::<Vec<_>>();
    let gap = Region {
        lower_m: [0.005, 0.005, 0.004],
        upper_m: [0.005, 0.005, 0.006],
    };
    Model {
        grid_m: [grid.clone(), grid.clone(), grid],
        boundaries: [Boundary::Mur; 6],
        solids: vec![],
        ports: vec![LumpedPort {
            region: gap,
            axis: Axis::Z,
            priority: 5,
            resistance_ohm: Some(50.0),
            excitation_v_per_m: Some(-1.0),
        }],
        probes: vec![Probe {
            name: "gap",
            kind: ProbeKind::Voltage,
            region: gap,
            weight: -1.0,
            // Порог этого теста, в В·с. В фиктивном выходе минимум ровно 5e-14 В·с.
            floor: Some(Sourced {
                value: 4.9e-14,
                source: "исходные данные синтетического стенда",
            }),
        }],
        pulse: GaussianPulse {
            center_hz: 1.5e9,
            cutoff_hz: 0.6e9,
        },
        band: Band {
            range_hz: [1e9, 2e9],
            frequencies_hz: vec![1e9, 1.5e9, 2e9],
            max_spacing_hz: 0.5e9,
        },
        max_steps: 20_000,
        end_energy_ratio: 1e-8,
    }
}

// Время выделено тестом на старт локального процесса, отдельный тест ниже укорачивает лимит.
const LIMIT: Duration = Duration::from_secs(15);

struct Fixture(PathBuf);
impl Fixture {
    fn new() -> Self {
        for slot in 0.. {
            let path =
                Path::new("/tmp").join(format!("sim-t64-fixture-{}-{slot}", std::process::id()));
            match fs::DirBuilder::new().mode(0o700).create(&path) {
                Ok(()) => return Self(path),
                Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {}
                Err(e) => panic!("{e}"),
            }
        }
        unreachable!()
    }

    fn script(&self, text: &str) -> PathBuf {
        let path = self.0.join("solver");
        fs::write(&path, format!("#!/bin/sh\nset -eu\n{text}\n")).unwrap();
        fs::set_permissions(&path, fs::Permissions::from_mode(0o700)).unwrap();
        path
    }
}

impl Drop for Fixture {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

fn mock(fixture: &Fixture, td: &str, fd: &str, extra: &str) -> PathBuf {
    fixture.script(&format!(
        r#"
test "$#" = 5
test "$1" = model.xml
test "$2" = --engine=basic
test "$3" = --numThreads=1
test "$4" = --exact-endcriteria
test "$5" = --dump-statistics
test "$LC_ALL" = C
test -f model.xml
pwd > "{tmp}/cwd"
: > "{token}"
echo ' | openEMS 64bit -- version 305df56'
echo '        CSXCAD -- Version: c46e320'
echo 'openEMS - enabled basic engine'
echo 'openEMS - fixed number of threads: 1'
echo 'Exact-endcriteria: evaluating the end criteria every 80 timestep(s)'
echo 'Excitation signal length is: 744 timesteps (1.43282e-09s)'
echo 'RunFDTD: end-criteria of -80dB reached after 5000 timesteps (-80.10dB)'
cat > openEMS_stats.txt <<'STATS'
1331 % number of cells
1e-12 % timestep (s)
5000 % number of iterations
5e-9 % total numerical time (s)
0.07 % simulation time (s)
111 % speed (cells/s)
STATS
cat > probe_0 <<'TD'
% time-domain voltage integration by openEMS 305df56 @синтетический вывод
% start-coordinates: (0.005,0.005,0.004) m -> [5,5,4]
% stop-coordinates: (0.005,0.005,0.006) m -> [5,5,6]
% t/s voltage
{td}
TD
cat > probe_0_FD <<'FD'
% frequency-domain voltage integration by openEMS 305df56 @синтетический вывод
% start-coordinates: (0.005,0.005,0.004) m -> [5,5,4]
% stop-coordinates: (0.005,0.005,0.006) m -> [5,5,6]
% f/Hz voltage
% real imag
{fd}
FD
{extra}
"#,
        tmp = fixture.0.display(),
        token = fixture.0.file_name().unwrap().to_str().unwrap()
    ))
}

// Спектр фиктивной программы. Модули по теореме Пифагора 5, 13 и 17 · 10⁻¹⁴ В·с.
// Отступ первого значения от пола равен 1e-15 В·с. Увеличиваем пол симметрично.
const FULL: &str = "1e9 3e-14 4e-14\n1.5e9 5e-14 12e-14\n2e9 8e-14 15e-14";

fn time_series(last: usize) -> String {
    // 20 пс между пробами, 250 интервалов дают 5 нс.
    (0..=last)
        .map(|i| format!("{} {}", i as f64 * 2e-11, if i == 1 { 1.0 } else { 0.0 }))
        .collect::<Vec<_>>()
        .join("\n")
}

fn assert_not_analyzed<T: std::fmt::Debug>(
    value: Result<Analysis<T>, sim_circuit::SimError>,
    reason: &str,
) {
    match value.unwrap() {
        Analysis::NotAnalyzed(message) => assert!(message.contains(reason), "{message}"),
        other => panic!("ожидалась открытая оценка: {other:?}"),
    }
}

fn calculated<T: std::fmt::Debug>(value: Result<Analysis<T>, sim_circuit::SimError>) -> T {
    match value.unwrap() {
        Analysis::Calculated(value) => value,
        other => panic!("ожидался спектр: {other:?}"),
    }
}

#[test]
fn strict_input_does_not_invoke_solver() {
    let fixture = Fixture::new();
    let executable = fixture.0.join("отсутствует");
    let mut mesh = model();
    mesh.grid_m[0][1] = f64::NAN;
    assert!(run_openems(&executable, &mesh, LIMIT).is_err());
    let mut geometry = model();
    geometry.ports[0].resistance_ohm = Some(f64::INFINITY);
    assert!(run_openems(&executable, &geometry, LIMIT).is_err());
    let mut invalid = model();
    invalid.band.frequencies_hz[1] = 1e9;
    assert!(prepare_openems(&invalid).is_err());
    invalid.band.frequencies_hz[1] = 1_000_000_001.0;
    assert!(prepare_openems(&invalid).is_err());

    let mut no_probe = model();
    no_probe.probes[0].floor.as_mut().unwrap().source = " ";
    assert_not_analyzed(prepare_openems(&no_probe), "источник");
    let mut no_center = model();
    no_center.band.frequencies_hz.remove(1);
    assert_not_analyzed(prepare_openems(&no_center), "полос");
    let mut no_edge = model();
    no_edge.band.frequencies_hz.pop();
    assert_not_analyzed(prepare_openems(&no_edge), "полос");
    let mut outside = model();
    outside.pulse.cutoff_hz = 0.4e9;
    assert_not_analyzed(prepare_openems(&outside), "возбуждени");
    assert_not_analyzed(run_openems(&executable, &model(), LIMIT), "исполняем");
}

#[test]
fn whole_band_is_bound_to_input_and_threshold_is_strict() {
    let model = model();
    let fixture = Fixture::new();
    let binary = mock(&fixture, &time_series(250), FULL, "");
    let result = calculated(run_openems(&binary, &model, LIMIT));
    let cwd = fs::read_to_string(fixture.0.join("cwd")).unwrap();
    assert!(
        Path::new(cwd.trim()).starts_with("/private/tmp")
            || Path::new(cwd.trim()).starts_with("/tmp")
    );
    // Другой тест может занять освободившийся путь. Своего файла в нём уже нет.
    assert!(
        !Path::new(cwd.trim())
            .join(fixture.0.file_name().unwrap())
            .exists()
    );
    assert_eq!(result.input_xml, calculated(prepare_openems(&model)));
    assert_eq!(result.openems_version, "305df56");
    assert_eq!(result.csxcad_version, "c46e320");
    assert_eq!(result.iterations, 5000);
    assert!((result.timestep_s * f64::from(result.iterations) - 5e-9).abs() < 5e-9 * f64::EPSILON);
    assert_eq!(result.frequencies_hz, [1e9, 1.5e9, 2e9]);
    let probe = &result.probes[0];
    assert_eq!(probe.name, "gap");
    assert_eq!(probe.snapped_start, [5, 5, 4]);
    assert_eq!(probe.snapped_stop, [5, 5, 6]);
    for (measured, expected) in probe.values.iter().zip([5e-14, 13e-14, 17e-14]) {
        assert!((measured.norm() - expected).abs() <= expected * f64::EPSILON);
    }
    let mut guarded = model.clone();
    guarded.probes[0].floor.as_mut().unwrap().value = 5.1e-14;
    assert_not_analyzed(run_openems(&binary, &guarded, LIMIT), "пола");
    let upper_trace = time_series(100);
    let binary = mock(&fixture, &upper_trace, FULL, "");
    assert_not_analyzed(run_openems(&binary, &model, LIMIT), "окн");
}

#[test]
fn rejects_incomplete_or_invalid_native_output() {
    let fixture = Fixture::new();
    let model = model();
    let td = time_series(250);
    for fd in [
        "1e9 3e-14 4e-14\n2e9 8e-14 15e-14",
        "1e9 3e-14 4e-14\n1.5e9 5e-14 12e-14",
        "1e9 3e-14 4e-14\n1.6e9 5e-14 12e-14\n2e9 8e-14 15e-14",
    ] {
        let binary = mock(&fixture, &td, fd, "");
        assert_not_analyzed(run_openems(&binary, &model, LIMIT), "полос");
    }
    for fd in [
        "1e9 NaN 4e-14",
        "1e9 3e-14 inf",
        "1e9 3e-14 4e-14 extra",
        "1e9 3e-14 4e-14\n1e9 3e-14 4e-14",
    ] {
        let binary = mock(&fixture, &td, fd, "");
        assert!(run_openems(&binary, &model, LIMIT).is_err(), "{fd}");
    }
    let gapped = td
        .lines()
        .enumerate()
        .filter_map(|(i, line)| (i != 2).then_some(line))
        .collect::<Vec<_>>()
        .join("\n");
    for (trace, extra, reason) in [
        (td.as_str(), "rm probe_0_FD", "отсутствует"),
        (gapped.as_str(), "", "окн"),
        (
            td.as_str(),
            "echo 'RunFDTD: Warning: no convergence' >&2",
            "предупреждение",
        ),
        (td.as_str(), "exit 7", "кодом"),
    ] {
        let binary = mock(&fixture, trace, FULL, extra);
        assert_not_analyzed(run_openems(&binary, &model, LIMIT), reason);
    }
}

#[test]
fn timeout_kills_process_and_removes_its_workdir() {
    let fixture = Fixture::new();
    let binary = fixture.script(&format!(
        r#"
pwd > "{0}/cwd"
echo "$$" > "{0}/pid"
: > expired-run
exec /bin/sleep 10
"#,
        fixture.0.display()
    ));
    // Лимит 0,5 с значительно меньше 10 с ожидания тестового дочернего процесса.
    assert_not_analyzed(
        run_openems(&binary, &model(), Duration::from_millis(500)),
        "лимит",
    );
    let cwd = fs::read_to_string(fixture.0.join("cwd")).unwrap();
    assert!(!Path::new(cwd.trim()).join("expired-run").exists());
    let pid = fs::read_to_string(fixture.0.join("pid")).unwrap();
    assert!(
        !std::process::Command::new("/bin/kill")
            .args(["-0", pid.trim()])
            .output()
            .unwrap()
            .status
            .success()
    );
}

#[test]
#[ignore = "требуется установленный openEMS 0.37; запускать с --ignored"]
fn real_openems_is_reproducible_and_matches_time_domain_quadrature() {
    let executable = std::env::var_os("OPENEMS")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            PathBuf::from(std::env::var_os("HOME").unwrap()).join("opt/openEMS/bin/openEMS")
        })
        .canonicalize()
        .expect("нужен openEMS 0.37");
    let fixture = Fixture::new();
    // Сохраняем временной ряд для независимой квадратуры, пока адаптер держит рабочий каталог.
    let executable_arg = executable.to_str().unwrap().replace('\'', "'\\''");
    let binary = fixture.script(&format!(
        r#"
'{executable_arg}' "$@"
cp probe_0 "{tmp}/voltage"
cp probe_1 "{tmp}/current"
cp stdout stderr "{tmp}"
"#,
        tmp = fixture.0.display()
    ));
    let mut scene = model();
    scene.probes.push(Probe {
        name: "contour",
        kind: ProbeKind::Current(Axis::Z),
        region: Region {
            lower_m: [0.004, 0.004, 0.005],
            upper_m: [0.006, 0.006, 0.005],
        },
        weight: 1.0,
        floor: Some(Sourced {
            value: 1e-18,
            source: "пол тестового стенда, А·с",
        }),
    });
    let observed = run_openems(&binary, &scene, LIMIT);
    assert!(
        matches!(observed, Ok(Analysis::Calculated(_))),
        "{observed:?}\n{}\n{}",
        fs::read_to_string(fixture.0.join("stdout")).unwrap_or_default(),
        fs::read_to_string(fixture.0.join("stderr")).unwrap_or_default(),
    );
    let first = calculated(observed);
    let second = calculated(run_openems(&executable, &scene, LIMIT));
    assert_eq!(first, second);
    // Для вакуума на равномерной кубической сетке предел CFL равен Δx/(c·√3).
    // 1 мм из постановки, c = 299792458 м/с по определению СИ.
    // В openEMS Operator::CalcTimestep_Var3 берёт эквиваленты C и L как FDTD_FLOAT.
    let cfl_s = 0.001 / (299_792_458.0 * 3.0_f64.sqrt());
    assert!((first.timestep_s - cfl_s).abs() <= cfl_s * (2.0 * f64::from(f32::EPSILON)));
    let mut max_relative: f64 = 0.0;
    for ((a, b), file) in first
        .probes
        .iter()
        .zip(&second.probes)
        .zip(["voltage", "current"])
    {
        let trace = fs::read_to_string(fixture.0.join(file)).unwrap();
        let samples: Vec<(f64, f64)> = trace
            .lines()
            .filter(|l| !l.starts_with('%'))
            .map(|l| {
                let mut values = l.split_whitespace().map(|v| v.parse::<f64>().unwrap());
                (values.next().unwrap(), values.next().unwrap())
            })
            .collect();
        let dt = samples[1].0 - samples[0].0;
        for ((first, second), hz) in a.values.iter().zip(&b.values).zip(&first.frequencies_hz) {
            assert_eq!(
                [first.re.to_bits(), first.im.to_bits()],
                [second.re.to_bits(), second.im.to_bits()]
            );
            // Независимо интегрируем TD методом трапеций по записанному шагу. В отличие от FD
            // openEMS, берём все TD-пробы. 2% — допуск тестовой сетки, включая разницу квадратур.
            let direct = samples
                .windows(2)
                .map(|w| {
                    let term = |(t, x): (f64, f64)| Complex::from_polar(x, -TAU * hz * t);
                    (term(w[0]) + term(w[1])) * dt
                })
                .sum::<Complex<f64>>();
            let relative = (direct - first).norm() / first.norm();
            max_relative = max_relative.max(relative);
            assert!(
                relative <= 0.02,
                "{file} {hz}: {relative}; {direct:?} {first:?}"
            );
        }
    }
    println!(
        "Сверка напряжения и тока в {} точках на зонд, {} шагов по {:.9e} с, максимальное отклонение {:.6}%",
        first.frequencies_hz.len(),
        first.iterations,
        first.timestep_s,
        100.0 * max_relative
    );
}
