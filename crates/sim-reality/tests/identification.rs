use std::{collections::BTreeMap, fmt::Write};

use sim_reality::*;
use sim_signal::Rng;

struct Log {
    rows: Vec<(u64, &'static str, &'static str, f64)>,
    schema: Vec<Channel>,
}

impl Log {
    fn new() -> Self {
        Self {
            rows: Vec::new(),
            schema: Vec::new(),
        }
    }

    fn add(&mut self, time: u64, name: &'static str, unit: &'static str, value: f64) {
        if self.schema.iter().all(|c| c.name != name) {
            self.schema.push(Channel {
                name: name.into(),
                unit: unit.into(),
                role: Role::Observation,
                sampling: Sampling::Event,
                allow_invalid: false,
            });
        }
        self.rows.push((time, name, unit, value));
    }

    fn csv(&mut self, origin: &str) -> String {
        self.rows.sort_by_key(|r| r.0);
        let end = self.rows.last().unwrap().0;
        let mut csv = format!(
            "sim-reality,1,{origin}\nbuild,стенд,ревизия-а,{end}\ntime_ns,signal,unit,seq,value,valid\n"
        );
        let mut indexes = BTreeMap::new();
        for &(at, name, unit, v) in &self.rows {
            let index = indexes.entry(name).or_insert(0);
            writeln!(csv, "{at},{name},{unit},{index},{v},1").unwrap();
            *index += 1;
        }
        writeln!(csv, "end,{}", self.rows.len()).unwrap();
        csv
    }
}

fn criteria(values: &[(Parameter, f64)]) -> Criteria {
    Criteria {
        // В синтетических опытах отсекаем почти совпадающие регрессоры и остаток
        // по СКО более двух известных шумовых СКО (χ²/dof > 4).
        max_condition: 1000.0,
        max_reduced_chi_square: 4.0,
        max_half_width: values.iter().copied().collect(),
    }
}

fn dynamics(sigma: f64) -> Experiment {
    Experiment {
        model: "стержень".into(),
        input: ExperimentKind::Dynamics {
            axis: Axis::Angular,
            acceleration: "уск".into(),
            velocity: "скор".into(),
            effort: "момент".into(),
            sigma_effort: sigma,
            min_abs_velocity: 0.1, // Стенд задан вне стартовой полосы ±0,1 рад/с.
        },
        criteria: criteria(&[
            (Parameter::Coulomb, 0.01),
            (Parameter::Viscous, 0.01),
            (Parameter::Inertia, 0.01),
        ]),
    }
}

fn coupling(sigma: f64) -> Experiment {
    Experiment {
        model: "стержень".into(),
        input: ExperimentKind::Coupling {
            axis: Axis::Angular,
            relative_position: "сдвиг".into(),
            effort: "нагрузка".into(),
            sigma_position: sigma,
            min_abs_effort: 0.05,
        },
        criteria: criteria(&[(Parameter::Backlash, 0.001), (Parameter::Stiffness, 1.0)]),
    }
}

fn timing(sigma: f64, offset: u64) -> Experiment {
    Experiment {
        model: "канал".into(),
        input: ExperimentKind::Delay {
            sent: "выход".into(),
            received: "вход".into(),
            sigma_ns: sigma,
            clock_offset_bound_ns: offset,
        },
        criteria: criteria(&[(Parameter::Delay, 0.000_4)]),
    }
}

fn camera(sigma: f64) -> Experiment {
    Experiment {
        model: "оптика".into(),
        input: ExperimentKind::Camera {
            x: "x".into(),
            y: "y".into(),
            z: "z".into(),
            u: "u".into(),
            v: "v".into(),
            width: 1920,
            height: 1440,
            sigma_px: sigma,
            max_iterations: 40,
        },
        criteria: criteria(&[
            (Parameter::Fx, 2.0),
            (Parameter::Fy, 2.0),
            (Parameter::Cx, 1.0),
            (Parameter::Cy, 1.0),
            (Parameter::K1, 0.002),
            (Parameter::K2, 0.003),
            (Parameter::K3, 0.002),
            (Parameter::P1, 0.0005),
            (Parameter::P2, 0.0005),
        ]),
    }
}

/// Независимая проекция напрямую по коэффициентам степени и тангенциальному сдвигу.
fn project(p: [f64; 9], point: [f64; 3]) -> [f64; 2] {
    let [x, y, z] = point;
    let [x, y] = [x / z, y / z];
    let [fx, fy, cx, cy, k1, k2, k3, p1, p2] = p;
    let r = x.hypot(y);
    let radial = 1.0 + k1 * r.powi(2) + k2 * r.powi(4) + k3 * r.powi(6);
    [
        cx + fx * (x * radial + p1 * 2.0 * x * y + p2 * (3.0 * x * x + y * y)),
        cy + fy * (y * radial + p1 * (x * x + 3.0 * y * y) + p2 * 2.0 * x * y),
    ]
}

fn value(report: &Identification, p: Parameter) -> f64 {
    let estimate = report
        .estimates()
        .iter()
        .find(|e| e.parameter == p)
        .unwrap();
    let Outcome::Identified { value } = estimate.outcome else {
        panic!("{estimate:?}");
    };
    let [lo, hi] = estimate.interval95.unwrap();
    assert!(lo <= value && value <= hi && lo < hi, "{estimate:?}");
    assert!(estimate.condition.unwrap() >= 1.0 - f64::EPSILON.sqrt());
    assert!(estimate.sensitivity.unwrap() > 0.0);
    value
}

#[test]
fn recovers_all_parameters_across_seeds_noise_and_delays() {
    let mut max_error = BTreeMap::<Parameter, f64>::new();
    // Четыре постановки: меняются и заданные параметры, и СКО в физических единицах.
    // Истина остаётся в генераторе и проверках. В identify попадают журнал и настройки опыта.
    for (case, seed) in [11, 71, 243, 1051].into_iter().enumerate() {
        let c = case as f64;
        let [dry, viscous, inertia, gap, stiffness] = [
            0.24 + 0.03 * c,
            0.14 - 0.02 * c,
            0.51 + 0.1 * c,
            0.025 + 0.003 * c,
            18.0 - 2.0 * c,
        ];
        let optics = [
            510.0 + 8.0 * c,
            550.0 - 10.0 * c,
            954.0,
            717.0,
            -0.16 + 0.02 * c,
            0.085 + 0.01 * c,
            -0.046,
            0.011,
            -0.014 - 0.004 * c,
        ];
        let delay_ns = 12_500_000 + case as u64 * 7_130_000;
        let [effort_noise, position_noise, time_noise, pixel_noise] =
            [0.008 * c, 0.00012 * c, 22_000.0 * c, 0.026 * c];
        let mut log = Log::new();
        let mut input = Rng::stream(seed, "возбуждение");
        let mut noise = Rng::stream(seed, "приборы");
        // 400 независимых точек, один отсчёт на 0,1 с. Опыты сведены в один файл.
        for i in 0..400 {
            let t = i * 100_000_000;
            let side = if i % 2 == 0 { 1.0 } else { -1.0 };
            let velocity = side * (0.4 + 4.0 * input.uniform());
            let total = 8.0 * (input.uniform() - 0.5);
            // Прямой расчёт ускорения из приложенного момента и постоянного смещения.
            let acceleration = (total - dry * side - viscous * velocity - 0.11) / inertia;
            let static_effort = side * (0.08 + 1.1 * input.uniform());
            let position = side * gap / 2.0 + static_effort / stiffness + 0.006;
            log.add(t, "скор", "rad/s", velocity);
            log.add(t, "уск", "rad/s^2", acceleration);
            log.add(t, "момент", "N*m", total + effort_noise * noise.normal());
            log.add(t, "нагрузка", "N*m", static_effort);
            log.add(
                t,
                "сдвиг",
                "rad",
                position + position_noise * noise.normal(),
            );
            let received = (delay_ns as f64 + time_noise * noise.normal()).round() as u64;
            log.add(t, "выход", "1", i as f64);
            log.add(t + received, "вход", "1", i as f64);
            let z = 0.4 + input.uniform();
            let xyz = [
                (2.0 * input.uniform() - 1.0) * z,
                (2.0 * input.uniform() - 1.0) * z,
                z,
            ];
            let [u, v] = project(optics, xyz);
            for (key, unit, val) in [
                ("x", "m", xyz[0]),
                ("y", "m", xyz[1]),
                ("z", "m", xyz[2]),
                ("u", "px", u + pixel_noise * noise.normal()),
                ("v", "px", v + pixel_noise * noise.normal()),
            ] {
                log.add(t, key, unit, val);
            }
        }
        let csv = log.csv("synthetic");
        let capture = Capture::parse(&csv, &log.schema).unwrap();
        let sigma = |actual: f64, floor: f64| actual.max(floor);
        let reports = [
            identify(&capture, &dynamics(sigma(effort_noise, 1e-3))).unwrap(),
            identify(&capture, &coupling(sigma(position_noise, 1e-5))).unwrap(),
            identify(&capture, &timing(sigma(time_noise, 1.0), 0)).unwrap(),
            identify(&capture, &camera(sigma(pixel_noise, 1e-3))).unwrap(),
        ];
        let targets = [
            (0, Parameter::Coulomb, dry),
            (0, Parameter::Viscous, viscous),
            (0, Parameter::Inertia, inertia),
            (1, Parameter::Backlash, gap),
            (1, Parameter::Stiffness, stiffness),
            (2, Parameter::Delay, delay_ns as f64 * 1e-9),
            (3, Parameter::Fx, optics[0]),
            (3, Parameter::Fy, optics[1]),
            (3, Parameter::Cx, optics[2]),
            (3, Parameter::Cy, optics[3]),
            (3, Parameter::K1, optics[4]),
            (3, Parameter::K2, optics[5]),
            (3, Parameter::K3, optics[6]),
            (3, Parameter::P1, optics[7]),
            (3, Parameter::P2, optics[8]),
        ];
        for (r, parameter, truth) in targets {
            let error = (value(&reports[r], parameter) / truth - 1.0).abs();
            assert!(error <= 0.05, "зерно {seed} {parameter:?} {error:.4}");
            let worst = max_error.entry(parameter).or_default();
            *worst = worst.max(error);
        }
        let duplicate = identify(&capture, &camera(sigma(pixel_noise, 1e-3))).unwrap();
        assert_eq!(
            serde_json::to_vec(&duplicate).unwrap(),
            serde_json::to_vec(&reports[3]).unwrap()
        );
    }
    for (p, e) in max_error {
        println!("{p:?} {:.6}%", 100.0 * e);
    }
}

#[test]
fn audit_recovers_blind_parameters_with_factorial_contrasts_and_holdout() {
    use Parameter::*;

    // Новые стендовые истины и СКО в единицах CSV. Критерии опыта общие для всех трёх серий.
    let cases = [
        (
            893,
            [0.39, 0.19, 0.36, 0.033, 23.0],
            9_300_000,
            [0.013, 0.00017, 19_000.0, 0.045],
            [
                562.0, 488.0, 928.0, 690.0, -0.12, 0.16, -0.068, 0.018, -0.021,
            ],
        ),
        (
            8191,
            [0.52, 0.08, 1.13, 0.012, 31.0],
            27_400_000,
            [0.020, 0.00022, 43_000.0, 0.083],
            [
                476.0, 536.0, 984.0, 751.0, 0.09, -0.135, 0.057, -0.009, 0.013,
            ],
        ),
        (
            60139,
            [0.18, 0.26, 0.71, 0.058, 8.5],
            38_200_000,
            [0.011, 0.00015, 36_000.0, 0.062],
            [
                519.0, 603.0, 891.0, 679.0, -0.19, 0.19, -0.082, 0.014, 0.009,
            ],
        ),
    ];
    let optics_keys = [Fx, Fy, Cx, Cy, K1, K2, K3, P1, P2];
    let mut worst = BTreeMap::<Parameter, f64>::new();
    let mut max_contrast_error = 0.0_f64;
    let mut max_holdout_error = 0.0_f64;
    for (seed, [dry, viscous, inertia, backlash, stiffness], delay_ns, sigmas, optics) in cases {
        let [sigma_torque, sigma_position, sigma_time, sigma_px] = sigmas;
        let mut noise = Rng::stream(seed, "шум независимого стенда");
        let mut targets = Rng::stream(seed, "координаты мишени независимого стенда");
        let mut log = Log::new();
        let mut torque_sums = [[0.0; 2]; 4];
        let mut displacement_sums = [[0.0; 2]; 2];
        let mut elapsed_ns = 0_i64;
        // Полный факторный план 4 скорости × 2 ускорения, 48 повторений.
        // Нагрузки обеих сторон передачи также сбалансированы.
        let velocities: [f64; 4] = [-3.2, -0.75, 0.75, 3.2];
        let accelerations = [-2.4, 2.4];
        let loads = [0.17, 1.4];
        const REPEATS: u64 = 48;
        for i in 0..8 * REPEATS {
            let t = i * 60_000_000; // 60 мс вмещают все заданные задержки.
            let v_index = (i % 4) as usize;
            let a_index = ((i / 4) % 2) as usize;
            let s_index = v_index / 2;
            let k_index = v_index % 2;
            let v = velocities[v_index];
            let a = accelerations[a_index];
            let sign = v.signum();
            // Заданные нулевые смещения относятся к синтетическим приборам.
            let torque =
                dry * sign + viscous * v + inertia * a - 0.17 + sigma_torque * noise.normal();
            let load = sign * loads[k_index];
            let displacement =
                sign * backlash / 2.0 + load / stiffness - 0.007 + sigma_position * noise.normal();
            torque_sums[v_index][a_index] += torque;
            displacement_sums[s_index][k_index] += displacement;
            let lag = (delay_ns as f64 + sigma_time * noise.normal()).round() as i64;
            elapsed_ns += lag;
            for (name, unit, response) in [
                ("скор", "rad/s", v),
                ("уск", "rad/s^2", a),
                ("момент", "N*m", torque),
                ("нагрузка", "N*m", load),
                ("сдвиг", "rad", displacement),
                ("выход", "1", i as f64),
            ] {
                log.add(t, name, unit, response);
            }
            log.add(t + lag as u64, "вход", "1", i as f64);
            // Расставляем мишени в разных глубинах и по обеим осям кадра.
            let z = 0.5 + 2.0 * targets.uniform();
            let xyz = [
                (1.84 * targets.uniform() - 0.92) * z,
                (1.76 * targets.uniform() - 0.88) * z,
                z,
            ];
            let [u, v] = project(optics, xyz);
            for (name, unit, response) in [
                ("x", "m", xyz[0]),
                ("y", "m", xyz[1]),
                ("z", "m", xyz[2]),
                ("u", "px", u + sigma_px * noise.normal()),
                ("v", "px", v + sigma_px * noise.normal()),
            ] {
                log.add(t, name, unit, response);
            }
        }
        let capture = Capture::parse(&log.csv("synthetic"), &log.schema).unwrap();
        let experiments = [
            dynamics(sigma_torque),
            coupling(sigma_position),
            timing(sigma_time, 0),
            camera(sigma_px),
        ];
        let [motion, gear, timing, camera] = experiments.map(|e| identify(&capture, &e).unwrap());

        // Независимое решение по контрастам ячеек плана, без регрессии и SVD.
        let torque = torque_sums.map(|v| v.map(|sum| sum / REPEATS as f64));
        let mean = torque.map(|[lo, hi]| (lo + hi) / 2.0);
        let viscous_check =
            ((mean[3] - mean[2]) + (mean[1] - mean[0])) / (2.0 * (velocities[3] - velocities[2]));
        let dry_check = ((mean[3] + mean[2] - mean[1] - mean[0]) / 2.0
            - viscous_check * (velocities[3] + velocities[2]))
            / 2.0;
        let inertia_check = torque.iter().map(|[lo, hi]| hi - lo).sum::<f64>()
            / (4.0 * (accelerations[1] - accelerations[0]));
        let [[neg_lo, neg_hi], [pos_lo, pos_hi]] =
            displacement_sums.map(|p| p.map(|sum| sum / (2 * REPEATS) as f64));
        let compliance = (pos_hi - pos_lo - (neg_hi - neg_lo)) / (2.0 * (loads[1] - loads[0]));
        let backlash_check = pos_lo - neg_lo - 2.0 * loads[0] * compliance;
        let lag_check = elapsed_ns as f64 / (8 * REPEATS) as f64 * 1e-9;
        let fitted_optics = optics_keys.map(|p| value(&camera, p));
        let mut record = |p: Parameter, fitted: f64, truth: f64| {
            let error = (fitted / truth - 1.0_f64).abs();
            assert!(error <= 0.05, "зерно {seed}, {p:?}: {error:.6}");
            let previous = worst.entry(p).or_default();
            *previous = previous.max(error);
        };
        for (p, result, truth, check) in [
            (Coulomb, &motion, dry, dry_check),
            (Viscous, &motion, viscous, viscous_check),
            (Inertia, &motion, inertia, inertia_check),
            (Backlash, &gear, backlash, backlash_check),
            (Stiffness, &gear, stiffness, compliance.recip()),
            (Delay, &timing, delay_ns as f64 * 1e-9, lag_check),
        ] {
            let fitted = value(result, p);
            record(p, fitted, truth);
            let discrepancy = (fitted / check - 1.0).abs();
            max_contrast_error = max_contrast_error.max(discrepancy);
            // Допуск округления независимых формул на малой матрице.
            assert!(discrepancy < 1e-10, "{p:?}: {fitted} != {check}");
        }
        for ((p, fitted), truth) in optics_keys.into_iter().zip(fitted_optics).zip(optics) {
            record(p, fitted, truth);
        }
        // Контрольные точки с другими координатами не входят в журнал. Допуск 0,1 px из §4.4.
        for xyz in [
            [-0.73, 0.79, 1.0],
            [0.33, -0.51, 0.8],
            [0.81, 0.12, 1.3],
            [0.0, 0.0, 0.7],
        ] {
            let prediction = project(fitted_optics, xyz);
            let expected = project(optics, xyz);
            let error = (prediction[0] - expected[0]).hypot(prediction[1] - expected[1]);
            max_holdout_error = max_holdout_error.max(error);
            assert!(error < 0.1, "зерно {seed}, {xyz:?}: {error} px");
        }
    }
    for (p, error) in worst {
        println!("Независимая серия, {p:?}: {:.6}%", 100.0 * error);
    }
    println!(
        "Контрасты: {max_contrast_error:.3e}, контрольная проекция: {max_holdout_error:.6} px"
    );
}

#[test]
fn rejects_unexcited_branches_and_keeps_independent_stiffness() {
    let mut log = Log::new();
    for i in 1..20 {
        log.add(i, "скор", "rad/s", 2.0);
        log.add(i, "уск", "rad/s^2", 0.0);
        log.add(i, "момент", "N*m", 1.0);
        log.add(i, "нагрузка", "N*m", i as f64 * 0.1);
        log.add(i, "сдвиг", "rad", 0.02 + i as f64 * 0.01);
        for (name, unit, v) in [
            ("x", "m", 0.0),
            ("y", "m", 0.0),
            ("z", "m", 1.0),
            ("u", "px", 960.0),
            ("v", "px", 720.0),
        ] {
            log.add(i, name, unit, v);
        }
        log.add(i * 100, "выход", "1", i as f64);
        // Маркеры приходят, но соответствие потеряно.
        log.add(i * 100 + 5, "вход", "1", (i + 1) as f64);
    }
    let csv = log.csv("synthetic");
    let cap = Capture::parse(&csv, &log.schema).unwrap();
    for experiment in [dynamics(0.001), camera(0.01), timing(1.0, 0)] {
        let result = identify(&cap, &experiment).unwrap();
        assert!(!result.estimates().is_empty());
        assert!(result.estimates().iter().all(|e| matches!(e.outcome, Outcome::NotAnalyzed { .. }) && e.interval95.is_none()));
    }
    let result = identify(&cap, &coupling(0.00001)).unwrap();
    assert!(matches!(
        result.estimates()[0].outcome,
        Outcome::NotAnalyzed { .. }
    ));
    // На одной стороне остаётся независимый наклон ΔT/Δq = 0,1/0,01 Н·м/рад.
    assert!((value(&result, Parameter::Stiffness) - 10.0).abs() < 1e-12);
}

#[test]
fn audit_refuses_confounded_coefficients_even_with_perfect_fit() {
    use Parameter::*;
    let mut log = Log::new();
    // На одном модуле скорости сухое и вязкое трение неразделимы.
    // На одном модуле момента люфт и податливость неразделимы.
    // На одном радиусе коэффициенты r², r⁴ и r⁶ наблюдаются только в сумме.
    let optics = [481.0, 537.0, 918.0, 693.0, -0.15, 0.12, -0.035, 0.01, -0.01];
    const POINTS: u64 = 32; // Полный оборот равномерных углов на фиксированном радиусе.
    for i in 0..POINTS {
        let t = i + 1;
        let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
        let acceleration = (i % 5) as f64 - 2.0; // Ускорение меняется независимо от знака.
        let velocity = sign * 1.7;
        let torque = 0.24 * sign + 0.08 * velocity + 0.72 * acceleration;
        let load = sign * 0.75;
        let displacement = sign * 0.018 + load / 26.0;
        let angle = std::f64::consts::TAU * i as f64 / POINTS as f64;
        let xyz = [0.64 * angle.cos(), 0.64 * angle.sin(), 1.0];
        let [u, v] = project(optics, xyz);
        for (name, unit, response) in [
            ("скор", "rad/s", velocity),
            ("уск", "rad/s^2", acceleration),
            ("момент", "N*m", torque),
            ("нагрузка", "N*m", load),
            ("сдвиг", "rad", displacement),
            ("x", "m", xyz[0]),
            ("y", "m", xyz[1]),
            ("z", "m", xyz[2]),
            ("u", "px", u),
            ("v", "px", v),
        ] {
            log.add(t, name, unit, response);
        }
    }
    let cap = Capture::parse(&log.csv("synthetic"), &log.schema).unwrap();
    let motion = identify(&cap, &dynamics(0.001)).unwrap();
    let gear = identify(&cap, &coupling(0.00001)).unwrap();
    let camera = identify(&cap, &camera(0.01)).unwrap();
    for (report, unidentifiable) in [
        (&motion, &[Coulomb, Viscous][..]),
        (&gear, &[Backlash, Stiffness][..]),
        (&camera, &[K1, K2, K3][..]),
    ] {
        for p in unidentifiable {
            let e = report
                .estimates()
                .iter()
                .find(|e| e.parameter == *p)
                .unwrap();
            assert!(matches!(e.outcome, Outcome::NotAnalyzed { .. }), "{e:?}");
            assert!(e.interval95.is_none(), "{e:?}");
        }
    }
    // Инерция по-прежнему определяется независимым наклоном момента по ускорению.
    assert!((value(&motion, Inertia) - 0.72).abs() < 1e-12);
}

#[test]
fn delay_interval_matches_closed_form_and_rejects_clock_uncertainty() {
    let mut log = Log::new();
    for i in 0..4 {
        log.add(i * 30_000_000, "выход", "1", i as f64);
        log.add(i * 30_000_000 + 13_000_000, "вход", "1", i as f64);
    }
    let csv = log.csv("synthetic");
    let cap = Capture::parse(&csv, &log.schema).unwrap();
    let fit = identify(&cap, &timing(100_000.0, 50_000)).unwrap();
    let center = value(&fit, Parameter::Delay);
    let [lo, hi] = fit.estimates()[0].interval95.unwrap();
    // Независимый расчёт: σ среднего четырёх серий = 0,1 мс / 2.
    // 95%: 1,959964·0,05 мс + 0,05 мс предельной ошибки синхронизации.
    assert!((center - 0.013).abs() < 1e-15);
    assert!(((hi - lo) / 2.0 - 0.0001479982).abs() < 1e-11);
    let unknown = identify(&cap, &timing(100_000.0, 2_000_000)).unwrap();
    assert!(matches!(
        unknown.estimates()[0].outcome,
        Outcome::NotAnalyzed { .. }
    ));
}

#[test]
fn synthetic_and_other_revision_cannot_install_measurements() {
    let mut log = Log::new();
    for i in 0..10 {
        log.add(i * 20_000_000, "выход", "1", i as f64);
        log.add(i * 20_000_000 + 8_000_000, "вход", "1", i as f64);
    }
    let synthetic = Capture::parse(&log.csv("synthetic"), &log.schema).unwrap();
    let mut receipt = Receipt {
        build: synthetic.build().clone(),
        log_fingerprint: format!("{:016x}", synthetic.fingerprint()),
        capture_id: "фикстура".into(),
        calibration: "фикстура".into(),
        scope: "фикстура".into(),
    };
    assert!(MeasuredCapture::verify(&synthetic, synthetic.build(), &receipt).is_err());
    // Здесь имитация записи владельца только проверяет программную границу.
    let recorded = Capture::parse(&log.csv("recorded"), &log.schema).unwrap();
    assert!(MeasuredCapture::verify(&recorded, recorded.build(), &receipt).is_err());
    receipt.log_fingerprint = format!("{:016x}", recorded.fingerprint());
    let other = BuildId {
        revision: "ревизия-б".into(),
        ..receipt.build.clone()
    };
    assert!(MeasuredCapture::verify(&recorded, &other, &receipt).is_err());
    let measured = MeasuredCapture::verify(&recorded, &receipt.build, &receipt).unwrap();
    let result = identify(&recorded, &timing(1000.0, 0)).unwrap();
    let update = Calibration::from_identification(&result, &measured).unwrap();
    let wrong = identify(&synthetic, &timing(1000.0, 0)).unwrap();
    assert!(Calibration::from_identification(&wrong, &measured).is_err());
    assert!(update.parameters(&other, "канал").is_err());
    assert!(
        update
            .parameters(&receipt.build, "отличающийся канал")
            .is_err()
    );
    assert!(
        (update.parameters(&receipt.build, "канал").unwrap()[&Parameter::Delay].value - 0.008)
            .abs()
            < 1e-15
    );
    assert_eq!(serde_json::to_value(&update).unwrap()["source"], "measured");
}
