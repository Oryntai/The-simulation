use std::{num::NonZeroU32, time::Duration};

use nalgebra::Complex;
use sim_circuit::{
    SimError,
    emc::{Analysis, Sourced, cable::*, coupling::*, loop_model::*, openems::*, power::*},
};

fn sourced<T>(value: T) -> Option<Sourced<'static, T>> {
    Some(Sourced {
        value,
        source: "Параметр синтетического стенда блока Т6",
    })
}

fn calculated<T: std::fmt::Debug>(result: Result<Analysis<T>, SimError>) -> T {
    match result.unwrap() {
        Analysis::Calculated(value) => value,
        other => panic!("ожидался расчёт в области модели: {other:?}"),
    }
}

fn open<T: std::fmt::Debug>(result: Result<Analysis<T>, SimError>, fragment: &str) {
    assert!(
        matches!(&result, Ok(Analysis::NotAnalyzed(reason)) if reason.contains(fragment)),
        "{result:?}"
    );
}

#[test]
fn петля_и_питание_за_границей_модели_открыты() {
    // Стенд: 0,1 м при v=2e8 м/с и λ/10 даёт 200 МГц, хотя отдельно задан 1 ГГц.
    let domain = LumpedDomain {
        max_frequency_hz: 1e9,
        longest_path_m: 0.1,
        min_wave_speed_m_s: 2e8,
        max_wavelength_fraction: 0.1,
    };
    let path = |resistance_ohm| {
        calculated(fitted_path(
            sourced(SeriesRl {
                resistance_ohm,
                inductance_h: 0.0,
            }),
            sourced(domain),
        ))
    };
    let feed = path(1.0);
    let shared = path(0.2);
    assert_eq!(feed.max_frequency_hz(), 200e6);
    assert_eq!(calculated(feed.impedance(200e6)), Complex::ONE);
    open(feed.impedance(200e6_f64.next_up()), "полос");
    assert!(feed.impedance(f64::NAN).is_err());

    // Периметр прямоугольника 0,1 м, r/min(a,b)=1/2 выходит за границу 0,1.
    let geometry = RectangularLoop {
        side_a_m: 0.03,
        side_b_m: 0.02,
        wire_radius_m: 0.01,
        resistivity_ohm_m: 1.0, // Синтетический однородный провод.
        max_radius_to_side: 0.1,
    };
    open(
        rectangular_loop(sourced(geometry), sourced(domain)),
        "геометри",
    );
    let geometry = RectangularLoop {
        wire_radius_m: 0.001,
        ..geometry
    };
    assert!(calculated(rectangular_loop(sourced(geometry), sourced(domain))).area_m2 > 0.0);
    let too_short = LumpedDomain {
        longest_path_m: 0.09,
        ..domain
    };
    open(
        rectangular_loop(sourced(geometry), sourced(too_short)),
        "геометри",
    );

    // 5 В и I=1+0,25 cos(ωt) А. По закону Ома минимум 3,75 В.
    let harmonics = [Harmonic {
        order: 1,
        peak: Complex::new(0.25, 0.0),
    }];
    let load = PeriodicLoad {
        dc_a: 1.0,
        fundamental_hz: 200e6,
        harmonics: &harmonics,
        complete_spectrum: true,
    };
    let fixed = PowerInput {
        domain: sourced(domain),
        source_v: sourced(5.0),
        feed: Some(feed),
        shared_return: Some(shared),
        decoupling: sourced(&[]),
        load: sourced(load),
    };
    let rail = calculated(analyze_power(fixed)).rail;
    assert_eq!(rail.at_phase(0.0).unwrap(), 3.75);
    let samples = NonZeroU32::MIN; // В фазе ноль находится минимум одной гармоники.
    for (limit, violates) in [(3.7, false), (3.8, true)] {
        let result = rail
            .check(sourced(VoltageLimit::Min(limit)), samples)
            .unwrap();
        if violates {
            assert!(matches!(result, VoltageCheck::Fail(_)), "{result:?}");
        } else {
            assert!(
                matches!(
                    result,
                    VoltageCheck::NotAnalyzed {
                        margin: Some(_),
                        ..
                    }
                ),
                "{result:?}"
            );
        }
    }
    for load in [
        PeriodicLoad {
            fundamental_hz: load.fundamental_hz.next_up(),
            ..load
        },
        PeriodicLoad {
            complete_spectrum: false,
            ..load
        },
    ] {
        open(
            analyze_power(PowerInput {
                load: sourced(load),
                ..fixed
            }),
            "спектр",
        );
    }
}

#[test]
fn пара_трасс_и_кабель_ограничены_геометрией_и_частотой() {
    // Синтетическая пара в вакууме: длина 0,1 м, w/h = gap/h = 1,
    // с λ/10 верхняя граница по длине равна c Гц (около 300 МГц).
    let geometry = MicrostripPair {
        length_m: 0.1,
        width_m: 0.001,
        gap_m: 0.001,
        height_m: 0.001,
        thickness_m: 0.0,
        relative_permittivity: 1.0,
        uniform_over_plane: true,
    };
    let band = CouplingBand {
        min_frequency_hz: 0.0,
        max_frequency_hz: 1e9,
        max_wavelength_fraction: 0.1,
    };
    let pair = calculated(coupled_microstrip(sourced(geometry), sourced(band)));
    // Нагрузки стенда, на DC прямая передача — делитель пополам, перекрёстная — 0.
    let load = ParallelRc {
        resistance_ohm: 50.0,
        capacitance_f: 0.0,
    };
    let ports = sourced(CouplingPorts {
        aggressor_near: load,
        victim_near: load,
        aggressor_far: load,
        victim_far: load,
    });
    let dc = calculated(pair.transfer(ports, 0.0));
    assert_eq!(dc.aggressor_far, Complex::new(0.5, 0.0));
    assert_eq!(dc.victim_far, Complex::ZERO);
    let [_, high] = pair.frequency_range_hz();
    assert!((high - 299_792_458.0).abs() < 299_792_458.0 * f64::EPSILON * 2.0);
    open(pair.transfer(ports, high.next_up()), "частота");
    assert!(pair.transfer(ports, f64::INFINITY).is_err());
    for geometry in [
        MicrostripPair {
            thickness_m: geometry.width_m,
            ..geometry
        },
        MicrostripPair {
            uniform_over_plane: false,
            ..geometry
        },
    ] {
        open(
            coupled_microstrip(sourced(geometry), sourced(band)),
            "геометри",
        );
    }

    // Стержень той же длины, без потерь, источник и нагрузка по 50 Ом.
    // Закон Ома даёт 10 мА на обоих портах при DC; предел стенда 11 мА.
    let cable = CableInput {
        geometry: sourced(CableGeometry {
            length_m: 0.1,
            radius_m: 0.001,
            axis_height_m: 0.005,
            relative_permittivity: 1.0,
            reference: "опора",
            uniform_over_plane: true,
        }),
        losses: sourced(CableLosses {
            series_resistance_ohm_per_m: 0.0,
            shunt_conductance_s_per_m: 0.0,
        }),
        band: sourced(CableBand {
            min_frequency_hz: 0.0,
            max_frequency_hz: band.max_frequency_hz,
            max_wavelength_fraction: band.max_wavelength_fraction,
        }),
        source: sourced(CableSource {
            reference: "опора",
            peak_v: Complex::ONE,
            impedance_ohm: Complex::new(50.0, 0.0),
        }),
        termination: sourced(CableTermination {
            reference: "опора",
            load: CableLoad::ImpedanceOhm(Complex::new(50.0, 0.0)),
        }),
        limit: sourced(CableLimit {
            reference: "опора",
            end: CableEnd::Far,
            frequency_hz: 0.0,
            max_peak_a: 0.011,
        }),
    };
    let response = calculated(common_mode_current(&cable, 0.0));
    assert!((response.far.current_peak_a.re - 0.01).abs() <= f64::EPSILON * 0.01);
    assert!(matches!(
        response.check,
        CableCheck::NotAnalyzed {
            margin: Some(_),
            ..
        }
    ));
    open(common_mode_current(&cable, high.next_up()), "размера");
    let mut incorrect = cable;
    incorrect.termination.as_mut().unwrap().value.reference = "другая опора";
    open(common_mode_current(&incorrect, 0.0), "опора");
    incorrect = cable;
    incorrect
        .geometry
        .as_mut()
        .unwrap()
        .value
        .uniform_over_plane = false;
    open(common_mode_current(&incorrect, 0.0), "геометри");
}

#[cfg(unix)]
#[test]
fn openems_полоса_пол_и_фактическое_затухание() {
    use std::{fs, os::unix::fs::PermissionsExt};

    struct Directory(std::path::PathBuf);
    impl Drop for Directory {
        fn drop(&mut self) {
            fs::remove_dir_all(&self.0).unwrap();
        }
    }

    // Тот же синтетический блок 10 мм и спектр, что в tests/openems.rs.
    // При шаге 1 пс и 5000 шагах окно 5 нс, а 20 пс между пробами
    // разрешают 2 ГГц. Минимальный модуль (3+4j)·10⁻¹⁴ равен 5·10⁻¹⁴.
    let axis = (0..=10).map(|i| f64::from(i) / 1000.0).collect::<Vec<_>>();
    let gap = Region {
        lower_m: [0.005, 0.005, 0.004],
        upper_m: [0.005, 0.005, 0.006],
    };
    let model = Model {
        grid_m: [axis.clone(), axis.clone(), axis],
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
            name: "зазор",
            kind: ProbeKind::Voltage,
            region: gap,
            weight: -1.0,
            floor: sourced(4.9e-14),
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
        end_energy_ratio: 1e-8, // 10 log₁₀(10⁻⁸) = −80 дБ.
    };
    let scratch = std::env::temp_dir().join(format!("ta6-openems-{}", std::process::id()));
    fs::create_dir(&scratch).unwrap();
    let dir = Directory(scratch);
    let binary = dir.0.join("стенд");
    let limit = Duration::from_secs(15); // Бюджет синтетического опыта tests/openems.rs.
    let run = |energy: &str, tail: &str, model: &Model<'_>| {
        fs::write(
            &binary,
            format!(
                r#"#!/bin/sh
set -eu
cat <<'LOG'
 | openEMS 64bit -- version 305df56
        CSXCAD -- Version: c46e320
openEMS - enabled basic engine
openEMS - fixed number of threads: 1
Exact-endcriteria: evaluating the end criteria every 80 timestep(s)
Excitation signal length is: 744 timesteps (7.44e-10s)
{energy}
LOG
cat > openEMS_stats.txt <<'STATS'
1331 % number of cells
1e-12 % timestep (s)
5000 % number of iterations
5e-9 % total numerical time (s)
STATS
cat > probe_0_FD <<'FD'
% frequency-domain voltage integration by openEMS 305df56 @блок
% start-coordinates: (0.005,0.005,0.004) m -> [5,5,4]
% stop-coordinates: (0.005,0.005,0.006) m -> [5,5,6]
% f/Hz voltage
% real imag
1e9 3e-14 4e-14
1.5e9 5e-14 12e-14
{tail}
FD
cat > probe_0 <<'TD'
% time-domain voltage integration by openEMS 305df56 @блок
% start-coordinates: (0.005,0.005,0.004) m -> [5,5,4]
% stop-coordinates: (0.005,0.005,0.006) m -> [5,5,6]
% t/s voltage
TD
awk 'BEGIN {{for (i=0;i<=250;++i) printf "%.12g %d\n",i*2e-11,(i==1)}}' >> probe_0
"#
            ),
        )
        .unwrap();
        fs::set_permissions(&binary, fs::Permissions::from_mode(0o700)).unwrap();
        run_openems(&binary, model, limit)
    };

    let valid = "RunFDTD: end-criteria of -80dB reached after 5000 timesteps (-80.10dB)";
    let tail = "2e9 8e-14 15e-14";
    let result = calculated(run(valid, tail, &model));
    assert_eq!(result.frequencies_hz, model.band.frequencies_hz);
    assert!((result.probes[0].values[0].norm() - 5e-14).abs() <= 5e-14 * f64::EPSILON);
    open(run(valid, "", &model), "полос");
    let mut sparse = model.clone();
    sparse.band.frequencies_hz.remove(1);
    open(prepare_openems(&sparse), "полос");
    let mut floor = model.clone();
    floor.probes[0].floor = sourced(5e-14);
    open(run(valid, tail, &floor), "пола");
    open(run("", tail, &model), "затухани");
    for incomplete in [
        "RunFDTD: end-criteria of -80dB reached after 5000 timesteps (-20dB)",
        "RunFDTD: end-criteria of -20dB reached after 5000 timesteps (-21dB)",
    ] {
        open(run(incomplete, tail, &model), "затухани");
    }
    for malformed in [
        "RunFDTD: end-criteria of -80dB reached after 5000 timesteps (-nandB)",
        "RunFDTD: end-criteria of -80dB reached after 5000 timesteps (-infdB)",
    ] {
        assert!(run(malformed, tail, &model).is_err());
    }
}
