use std::{collections::BTreeMap, path::Path};

use sim_process::contact::*;
use sim_process::{Analysis, Interval, SourcedRange};
use sim_signal::{Rate, Time};
use sim_world::*;

const FLOOR: &str = "environment/collision/0";
const BLOCK: &str = "блок/блок/collision/0";
const MASS: f64 = 2.0; // Известная масса синтетического стенда, кг.

fn datum(x: f64, source: &str) -> SourcedRange {
    SourcedRange {
        min: x,
        nominal: x,
        max: x,
        provenance: Provenance::Measured(source.into()),
    }
}

fn source() -> Provenance {
    Provenance::Measured(
        "Синтетический контроль с фиксированными входами, без притязания на свойства материала"
            .into(),
    )
}

fn plane_fixture(shape: Shape, z: f64, rate: u64, gravity: [f64; 3]) -> World {
    // Плоская опора 40×40 м сохраняет контакт даже в двухсекундном опыте срыва.
    let mut floor = Geometry::new(Shape::Box {
        size_m: [40.0, 40.0, 1.0],
    });
    floor.pose.position_m[2] = -0.5;
    let [ix, iy, iz] = match shape {
        Shape::Box { size_m } => std::array::from_fn(|axis| {
            MASS * (size_m[(axis + 1) % 3].powi(2) + size_m[(axis + 2) % 3].powi(2)) / 12.0
        }),
        Shape::Sphere { radius_m } => [2.0 / 5.0 * MASS * radius_m.powi(2); 3],
        _ => unreachable!(),
    };
    World {
        format: FORMAT,
        name: "Блок на опоре".into(),
        seed: 0,
        physics_hz: rate,
        gravity_m_s2: gravity,
        environment: Environment {
            collision: vec![floor],
            ..Environment::default()
        },
        objects: vec![Object {
            name: "блок".into(),
            pose: Pose {
                position_m: [0.0, 0.0, z],
                ..Pose::default()
            },
            mass: Mass::Explicit {
                mass_kg: MASS,
                com_m: [0.0; 3],
                inertia_kg_m2: [ix, iy, iz, 0.0, 0.0, 0.0],
                provenance: source(),
            },
            collision: vec![Geometry::new(shape)],
            visual: vec![],
            mounts: vec![],
        }],
        devices: vec![],
        attachments: vec![],
        sensors: vec![],
    }
}

fn build(world: &World, opts: &ContactBuildOptions) -> CompiledWorld {
    compile_world_with_contact_options(Path::new("опыт.toml"), world, &BTreeMap::new(), opts)
        .unwrap()
}

fn solver() -> ContactSolver {
    // Настройки повторяемого численного опыта по MuJoCo 4.0.1, таблица option.
    ContactSolver {
        cone: FrictionCone::Elliptic,
        solver: ConstraintSolver::Newton,
        impratio: 4.0,
        tolerance: 1e-10,
        iterations: 40,
        line_search_tolerance: 1e-10,
        line_search_iterations: 30,
        noslip_iterations: 0,
        noslip_tolerance: 1e-9,
        ccd_iterations: 50,
        ccd_tolerance: 1e-8,
    }
}

fn calibration() -> PairCalibration {
    // Постоянный импеданс 0.8 отличим от жёсткого предела в опыте пружины.
    PairCalibration {
        geometries: [BLOCK.into(), FLOOR.into()],
        solimp: [0.8, 0.8, 0.001, 0.5, 2.0],
        impedance_source: source(),
        effective_mass_kg: Some(datum(MASS, "масса свободной сферы стенда, известная точно")),
        friction_reference: FrictionReference::UseNormal,
    }
}

fn spring() -> PairParameters {
    PairParameters {
        friction: ContactFriction::NormalOnly,
        // Контрольные значения к=4000 Н/м, b=40 Н·с/м.
        normal: NormalResponse::SpringDamper {
            stiffness_n_m: datum(4000.0, "жёсткость контрольной пружины"),
            damping_ns_m: datum(40.0, "демпфирование контрольного стенда"),
        },
    }
}

fn table(parameters: PairParameters) -> ContactMaterialTable {
    ContactMaterialTable::new([(["матрица".into(), "вкладыш".into()], parameters)]).unwrap()
}

fn resolved(
    scene: &CompiledWorld,
    table: &ContactMaterialTable,
    calibrations: &[PairCalibration],
) -> Vec<ResolvedContactPair> {
    table
        .resolve(
            scene,
            &[
                GeometryMaterial {
                    geometry: FLOOR.into(),
                    material: "матрица".into(),
                },
                GeometryMaterial {
                    geometry: BLOCK.into(),
                    material: "вкладыш".into(),
                },
            ],
            calibrations,
            FrictionCone::Elliptic,
        )
        .unwrap()
}

#[test]
fn symmetric_table_and_invalid_material_parameters() {
    let m = ["матрица", "вкладыш"];
    let table = table(spring());
    assert_eq!(
        table.lookup(m).unwrap(),
        table.lookup([m[1], m[0]]).unwrap()
    );
    assert!(matches!(
        table.lookup([m[0], "чужой"]).unwrap(),
        Analysis::NotAnalyzed { .. }
    ));
    assert!(table.lookup([m[0], ""]).is_err());
    assert!(
        ContactMaterialTable::new([
            (m.map(str::to_owned), spring()),
            ([m[1].into(), m[0].into()], spring()),
        ])
        .is_err()
    );

    let make = |e, source| PairParameters {
        friction: ContactFriction::NormalOnly,
        normal: NormalResponse::Restitution {
            coefficient: datum(e, source),
            duration_s: datum(0.02, "длительность опыта"),
        },
    };
    // e=0 не представим конечной линейной системой, e>1 означал бы активный удар.
    for e in [f64::NAN, f64::INFINITY, 0.0, 1.1] {
        assert!(ContactMaterialTable::new([(m.map(str::to_owned), make(e, "стенд"))]).is_err());
    }
    assert!(ContactMaterialTable::new([(m.map(str::to_owned), make(0.5, ""))]).is_err());
    let mut shape = calibration();
    let rate = Rate::hz(4000).unwrap();
    shape.solimp[0] = 0.7;
    assert!(
        spring()
            .contact_settings(&shape, rate, FrictionCone::Elliptic)
            .is_err()
    );
    shape = calibration();
    shape.effective_mass_kg = None;
    assert!(
        spring()
            .contact_settings(&shape, rate, FrictionCone::Elliptic)
            .is_err()
    );
    shape = calibration();
    shape.friction_reference =
        FrictionReference::DirectDamping(datum(30.0, "частота касательной моды стенда"));
    assert!(
        spring()
            .contact_settings(&shape, rate, FrictionCone::Elliptic)
            .is_err()
    );
}

#[test]
fn incomplete_or_estimated_catalog_never_supplies_proof() {
    let world = plane_fixture(Shape::Sphere { radius_m: 0.05 }, 0.049, 4000, [0.0; 3]);
    let scene = build(&world, &ContactBuildOptions::default());
    let mut assignments = [
        GeometryMaterial {
            geometry: FLOOR.into(),
            material: "матрица".into(),
        },
        GeometryMaterial {
            geometry: BLOCK.into(),
            material: "вкладыш".into(),
        },
    ];
    let empty = ContactMaterialTable::default();
    assert!(
        empty
            .resolve(
                &scene,
                &assignments,
                &[calibration()],
                FrictionCone::Elliptic
            )
            .is_err()
    );
    assert!(
        table(spring())
            .resolve(
                &scene,
                &assignments[..1],
                &[calibration()],
                FrictionCone::Elliptic
            )
            .is_err()
    );
    assert!(
        table(spring())
            .resolve(&scene, &assignments, &[], FrictionCone::Elliptic)
            .is_err()
    );
    assignments[1].geometry = FLOOR.into();
    assert!(
        table(spring())
            .resolve(
                &scene,
                &assignments,
                &[calibration()],
                FrictionCone::Elliptic
            )
            .is_err()
    );

    for provenance in [
        Provenance::Estimate("оценённая масса".into()),
        Provenance::Derived("промежуточный расчёт".into()),
    ] {
        let mut c = calibration();
        c.effective_mass_kg.as_mut().unwrap().provenance = provenance;
        assert!(matches!(
            resolved(&scene, &table(spring()), &[c])[0].nominal_coverage,
            Analysis::NotAnalyzed { .. }
        ));
    }
    let mut c = calibration();
    c.effective_mass_kg.as_mut().unwrap().min = 1.9;
    assert!(matches!(
        resolved(&scene, &table(spring()), &[c])[0].nominal_coverage,
        Analysis::NotAnalyzed { .. }
    ));
}

#[test]
fn spring_matches_single_normal_mode_on_known_mass() {
    // Сфера 2 кг, вдавливание 1 мм, скорость сближения 0.2 м/с.
    // F = 4000*0.001 + 40*0.2 = 12 Н при нулевом свободном ускорении.
    let world = plane_fixture(Shape::Sphere { radius_m: 0.05 }, 0.049, 4000, [0.0; 3]);
    let base = build(&world, &ContactBuildOptions::default());
    let pairs = resolved(&base, &table(spring()), &[calibration()]);
    assert!(matches!(
        pairs[0].nominal_coverage,
        Analysis::Analyzed { .. }
    ));
    assert_eq!(pairs[0].pair.settings.solref, [-2000.0, -20.0]);
    let scene = build(
        &world,
        &ContactBuildOptions {
            solver: Some(solver()),
            pairs: vec![pairs[0].pair.clone()],
            ..Default::default()
        },
    );
    let mut sim = scene.open_contact_sim().unwrap();
    sim.qvel_mut()[2] = -0.2;
    let step = sim.step_with_contacts().unwrap();
    let interval = Interval {
        start: Time::ZERO,
        end: scene.physics.tick(1),
    };
    let frame = ContactFrame::from_engine_step(interval, &step).unwrap();
    assert_eq!(frame.at, scene.physics.tick(1));
    assert_eq!(frame.contacts.len(), 1);
    let actual = frame.contacts[0].force_n[2].abs();
    println!("одиночная мода: F={actual:.9} Н, пружина+демпфер=12 Н");
    assert!((actual - 12.0).abs() < 1e-9);
    assert!((MASS * (sim.qvel()[2] + 0.2) - 12.0 * sim.timestep()).abs() < 1e-12);
    sim.forward();
    let forwarded = sim.contact_wrenches().unwrap()[0].force_n[2].abs();
    assert!((forwarded - actual).abs() > 0.1); // Уже новая поза и новая реакция.
    assert!(
        ContactFrame::from_engine_step(
            Interval {
                end: Time(1),
                ..interval
            },
            &step
        )
        .is_err()
    );
    assert!(
        ContactFrame::from_engine_step(
            Interval {
                start: interval.end,
                end: scene.physics.tick(2)
            },
            &step
        )
        .is_err()
    );
    let mut bad = step;
    bad.contacts[0].friction[0] = f64::NAN;
    assert!(ContactFrame::from_engine_step(interval, &bad).is_err());
}

#[test]
fn impact_matches_impulse_and_separation_time() {
    // Невесомость, масса 2 кг, скорость сближения 1 м/с, e=0.5, удар 20 мс.
    // Импульс жёсткого решения m*(v_out-v_in)=3 Н·с.
    let world = plane_fixture(Shape::Sphere { radius_m: 0.05 }, 0.10, 20000, [0.0; 3]);
    let base = build(&world, &ContactBuildOptions::default());
    let law = PairParameters {
        friction: ContactFriction::NormalOnly,
        normal: NormalResponse::Restitution {
            coefficient: datum(0.5, "восстановление контрольного удара"),
            duration_s: datum(0.02, "длительность контрольного удара"),
        },
    };
    let pair = resolved(&base, &table(law), &[calibration()])
        .remove(0)
        .pair;
    let scene = build(
        &world,
        &ContactBuildOptions {
            solver: Some(solver()),
            pairs: vec![pair],
            ..Default::default()
        },
    );
    let mut sim = scene.open_contact_sim().unwrap();
    sim.qvel_mut()[2] = -1.0;
    let mut impulse = 0.0;
    let mut compressed = 0;
    // Окно 0.12 с содержит вход, силу и свободный отскок.
    for _ in 0..2400 {
        for c in sim.step_with_contacts().unwrap().contacts {
            let f = c.force_n[2].abs();
            impulse += f * sim.timestep();
            compressed += usize::from(f > 0.0);
        }
    }
    let duration = compressed as f64 * sim.timestep();
    println!(
        "удар: J={impulse:.9} против 3 Н·с, v={:.9} против 0.5 м/с, T={duration:.9} против 0.02 с",
        sim.qvel()[2]
    );
    assert!((impulse - MASS * (sim.qvel()[2] + 1.0)).abs() < 1e-10);
    assert!((impulse - 3.0).abs() < 0.015); // 0.5% для Euler при h/T=1/400.
    assert!((duration - 0.02).abs() < 0.0002); // Четыре шага на границах участка.
}

#[test]
fn restitution_spans_overdamped_critical_and_elastic_cases() {
    // Те же 2 кг и 20 мс. Шаг 5 мкс разрешает быструю затухающую моду
    // и даёт 4000 делений времени удара для проверки его границ.
    let world = plane_fixture(Shape::Sphere { radius_m: 0.05 }, 0.06, 200_000, [0.0; 3]);
    let rate = Rate::hz(world.physics_hz).unwrap();
    for e in [0.05, (-2.0_f64).exp(), 1.0] {
        let parameters = PairParameters {
            friction: ContactFriction::NormalOnly,
            normal: NormalResponse::Restitution {
                coefficient: datum(e, "выбранная ветвь аналитического удара"),
                duration_s: datum(0.02, "20 мс в синтетическом опыте"),
            },
        };
        let calibration = calibration();
        let pair = ContactPair {
            geometries: calibration.geometries.clone(),
            settings: parameters
                .contact_settings(&calibration, rate, FrictionCone::Elliptic)
                .unwrap(),
        };
        let scene = build(
            &world,
            &ContactBuildOptions {
                solver: Some(solver()),
                pairs: vec![pair],
                ..Default::default()
            },
        );
        let mut s = scene.open_contact_sim().unwrap();
        s.qvel_mut()[2] = -1.0;
        let mut active = 0;
        // Ход 10 мм до контакта и минимум 20 мс свободного выбега после.
        for _ in 0..10_000 {
            active += s
                .step_with_contacts()
                .unwrap()
                .contacts
                .iter()
                .filter(|p| p.force_n[2] != 0.0)
                .count();
        }
        let duration = active as f64 * s.timestep();
        let restored = s.qvel()[2];
        println!("граничный удар: e={e:.9} → {restored:.9}, T={duration:.9} с");
        assert!((restored - e).abs() < 0.005 * e); // 0.5% от аналитического выбега.
        assert!((duration - 0.02).abs() <= 0.00005); // Десять шагов для двух границ силы.
    }
}

fn support(outside: bool) -> RigidPlaneSupport {
    // Блок прижат весом 20 Н. 6 Н или 11 Н действуют через центр масс
    // на высоте 0.025 м. Конус с μ=0.5 допускает до 10 Н.
    let tangent = if outside { 11.0 } else { 6.0 };
    RigidPlaneSupport {
        fixture: "Блок 2 кг, 0.2×0.2×0.05 м, опора 40 м, сила через ЦМ, отсчёт 1–3 с".into(),
        provenance: source(),
        half_extent_m: [0.1; 2],
        required_force_n: [-tangent, 0.0, 20.0],
        required_moment_nm: [0.0, -0.025 * tangent, 0.0],
        friction: datum(0.5, "сухое изотропное трение синтетического опыта"),
        moment_residual_nm: datum(1e-12, "округление аналитического баланса"),
    }
}

fn norms(x: [f64; 3]) -> f64 {
    x[0].hypot(x[1]).hypot(x[2])
}

fn run_creep(variant: CreepVariant) -> CreepTrial {
    creep_with_friction(variant, None)
}

fn creep_with_friction(variant: CreepVariant, coefficient: Option<f64>) -> CreepTrial {
    use CreepVariant::*;
    // h=1 мс, h/2, h/4. На мелкой сетке меняется по одному фактору.
    let hz = match variant {
        Baseline => 1000,
        HalfStep => 2000,
        _ => 4000,
    };
    let rate = Rate::hz(hz).unwrap();
    let mut model = if variant == OutsideCone {
        support(true)
    } else {
        support(false)
    };
    if let Some(coefficient) = coefficient {
        model.friction = datum(coefficient, "один изотропный коэффициент в опыте на срыв");
    }
    let world = plane_fixture(
        Shape::Box {
            size_m: [0.2, 0.2, 0.05],
        },
        0.025,
        hz,
        [
            -model.required_force_n[0] / MASS,
            0.0,
            -model.required_force_n[2] / MASS,
        ],
    );
    let mut s = solver();
    s.impratio = 1.0; // Исходное отношение равно единице; отдельно проверяется 10.
    let mut calibration = calibration();
    calibration.effective_mass_kg = None;
    calibration.solimp = if variant == Impedance {
        [0.95, 0.95, 0.001, 0.5, 2.0]
    } else {
        [0.99, 0.99, 0.001, 0.5, 2.0]
    };
    match variant {
        Cone => s.cone = FrictionCone::Pyramidal,
        ImpedanceRatio => s.impratio = 10.0,
        Tolerance => s.tolerance = 1e-7,
        Iterations => s.iterations = 5,
        NoSlip => s.noslip_iterations = 3,
        _ => {}
    }
    let law = PairParameters {
        friction: ContactFriction::Sliding {
            sliding: [model.friction.clone(), model.friction.clone()],
        },
        // Прямой закон искусственной опоры, с⁻² и с⁻¹.
        normal: NormalResponse::Direct {
            stiffness_per_s2: datum(
                if variant == Reference {
                    80000.0
                } else {
                    40000.0
                },
                "численная жёсткость стенда",
            ),
            damping_per_s: datum(400.0, "численное демпфирование стенда"),
        },
    };
    let pair = ContactPair {
        geometries: calibration.geometries.clone(),
        settings: law.contact_settings(&calibration, rate, s.cone).unwrap(),
    };
    let scene = build(
        &world,
        &ContactBuildOptions {
            solver: Some(s),
            pairs: vec![pair.clone()],
            ..Default::default()
        },
    );
    let mut sim = scene.open_contact_sim().unwrap();
    let config = CreepConfiguration {
        engine_version: "4.0.1".into(),
        rate,
        solver: s,
        pair,
        source: source(),
    };
    let [fx, fy, _] = model.required_force_n;
    let mut sample = |tick| {
        let step = sim.step_with_contacts().unwrap();
        let center = sim.center_of_mass("блок/блок").unwrap();
        let mut n = 0.0;
        let mut tangent = [-fx, -fy];
        let mut depth = 0.0_f64;
        let mut moment = [0.0; 3];
        for c in &step.contacts {
            let sign = if c.geometries[1] == BLOCK { 1.0 } else { -1.0 };
            let [fx, fy, fz] = c.force_n.map(|x| sign * x);
            let [x, y, z] = std::array::from_fn(|i| c.point_m[i] - center[i]);
            n += fz;
            tangent[0] += fx;
            tangent[1] += fy;
            depth = depth.max(-c.separation_m);
            for (i, v) in [y * fz - z * fy, z * fx - x * fz, x * fy - y * fx]
                .into_iter()
                .enumerate()
            {
                moment[i] += v + sign * c.torque_nm[i];
            }
        }
        HoldSample {
            at: rate.tick(tick),
            contacts: step.contacts.len(),
            tangent_position_m: sim.qpos()[..2].try_into().unwrap(),
            tangent_velocity_m_s: sim.qvel()[..2].try_into().unwrap(),
            normal_reaction_n: n,
            max_penetration_m: depth,
            net_moment_nm: norms(moment),
            net_tangent_force_n: tangent,
        }
    };
    for tick in 1..hz {
        sample(tick);
    }
    let first = sample(hz);
    let mut trace = CreepTrace::new(variant, config, model, hz, first).unwrap();
    for tick in hz + 1..=3 * hz {
        trace.record(sample(tick)).unwrap();
    }
    trace.finish().unwrap()
}

fn tolerances() -> CreepTolerances {
    // Критерии синтетического опыта: различают 20 Н опоры и срыв под
    // избытком 1 Н, допускают наблюдаемое мягкое смещение в режиме удержания.
    CreepTolerances {
        holding_speed_m_s: datum(
            0.02,
            "различение удержания и контрольного срыва после 1 с при ускорении 0.5 м/с²",
        ),
        normal_balance_n: datum(0.05, "баланс силы до 0.25% веса блока"),
        tangent_balance_n: datum(0.01, "менее 0.2% касательной нагрузки 6 Н"),
        moment_balance_nm: datum(0.001, "менее 1% момента касательной нагрузки 0.15 Н·м"),
        step_displacement_m: datum(5e-5, "50 мкм при сравнении вложенных сеток"),
        step_speed_m_s: datum(1e-4, "0.1 мм/с при сравнении вложенных сеток"),
        step_force_n: datum(0.01, "0.05% веса при сравнении вложенных сеток"),
        step_penetration_m: datum(2e-6, "2 мкм при сравнении вложенных сеток"),
    }
}

#[test]
fn creep_series_bounds_numerics_and_rejects_unsupported_inference() {
    use CreepVariant::*;
    let trials: Vec<_> = [
        Baseline,
        HalfStep,
        QuarterStep,
        Reference,
        Impedance,
        Cone,
        ImpedanceRatio,
        Tolerance,
        Iterations,
        NoSlip,
        OutsideCone,
    ]
    .into_iter()
    .map(run_creep)
    .collect();
    let rigid = support(false).balance().unwrap();
    let Analysis::Analyzed { value: rigid, .. } = rigid else {
        panic!("нет независимого баланса")
    };
    assert_eq!(rigid.mode, RigidMode::WithinCone);
    assert_eq!(rigid.reserve_n, [4.0; 2]);
    assert!((rigid.center_of_pressure_m[0] - 0.0075).abs() < 1e-14);
    assert!(
        matches!(support(true).balance().unwrap(), Analysis::Analyzed { value, .. }
        if value.mode == RigidMode::OutsideCone && value.reserve_n == [-1.0; 2])
    );
    let mut shifted = support(false);
    shifted.required_moment_nm[2] = 1.0;
    assert!(matches!(
        shifted.balance().unwrap(),
        Analysis::NotAnalyzed { .. }
    ));

    let report = CreepReport::from_trials(trials.clone(), &tolerances()).unwrap();
    assert_eq!(
        report.hold,
        Interval {
            start: Time(1_000_000_000),
            end: Time(3_000_000_000)
        }
    );
    for t in report.trials() {
        let m = t.metrics();
        println!(
            "{:?}: dx={:.9} max={:.9} m; v={:.9} vmax={:.9} m/s; N={:.9} [{:.9},{:.9}] N; depth={:.9} m; reserve={:.9} N; Mmax={:.9} N m",
            t.variant(),
            m.final_drift_m,
            m.peak_drift_m,
            m.final_speed_m_s,
            m.peak_speed_m_s,
            m.final_normal_n,
            m.normal_range_n[0],
            m.normal_range_n[1],
            m.peak_penetration_m,
            m.min_friction_reserve_n,
            m.peak_net_moment_nm
        );
        println!(
            "  Jn={:.9} Н·с, контакт {:.9} с, касательная невязка {:.9} Н",
            m.normal_impulse_n_s, m.contact_time_s, m.peak_tangent_imbalance_n
        );
    }
    println!("численная граница: {:?}", report.numerical_uncertainty_m());
    let Analysis::Analyzed { value: bound, .. } = report.numerical_uncertainty_m() else {
        panic!(
            "полный опыт должен оценить границу: {:?}",
            report.numerical_uncertainty_m()
        );
    };
    let fine = &trials[2];
    let no_slip = &trials[9];
    let outside = &trials[10];
    assert!(fine.metrics().final_drift_m > no_slip.metrics().final_drift_m + 1e-6);
    assert!((outside.metrics().final_speed_m_s - 1.5).abs() < 0.1); // 3 с при (11−10)/2 м/с².
    for t in &trials[..10] {
        assert!(*bound >= t.metrics().peak_drift_m);
    }
    let limit = datum(0.05, "допуск отдельного вычислительного испытания 5 см");
    assert!(
        matches!(report.compare_displacement_limit(fine.configuration(), fine.support(),
        [0.0, fine.metrics().peak_drift_m], &limit).unwrap(), Analysis::Analyzed { value, .. }
        if value.1 == LimitComparison::BelowLimit && value.0[1] == fine.metrics().peak_drift_m + bound)
    );
    assert!(matches!(
        report
            .compare_displacement_limit(
                fine.configuration(),
                fine.support(),
                [limit.nominal; 2],
                &limit
            )
            .unwrap(),
        Analysis::NotAnalyzed { .. }
    ));
    assert!(matches!(
        report
            .compare_displacement_limit(fine.configuration(), outside.support(), [0.0; 2], &limit)
            .unwrap(),
        Analysis::NotAnalyzed { .. }
    ));
    assert!(
        report
            .compare_displacement_limit(fine.configuration(), fine.support(), [f64::NAN; 2], &limit)
            .is_err()
    );

    let mut tight = tolerances();
    tight.step_displacement_m = datum(0.0, "проверка ошибочного требования точного совпадения");
    assert!(matches!(
        CreepReport::from_trials(trials.clone(), &tight)
            .unwrap()
            .numerical_uncertainty_m(),
        Analysis::NotAnalyzed { .. }
    ));
    tight = tolerances();
    tight.holding_speed_m_s = datum(
        0.0,
        "проверка исчезновения удержания на мягких ограничениях",
    );
    assert!(matches!(
        CreepReport::from_trials(trials[..10].to_vec(), &tolerances())
            .unwrap()
            .numerical_uncertainty_m(),
        Analysis::NotAnalyzed { .. }
    ));
    assert!(matches!(
        CreepReport::from_trials(trials, &tight)
            .unwrap()
            .numerical_uncertainty_m(),
        Analysis::NotAnalyzed { .. }
    ));
}

#[test]
fn empty_or_nonfinite_trace_is_refused() {
    let trial = run_creep(CreepVariant::Baseline);
    let config = trial.configuration().clone();
    let zero = HoldSample {
        at: config.rate.tick(1000),
        tangent_position_m: [0.0; 2],
        tangent_velocity_m_s: [0.0; 2],
        contacts: 1,
        normal_reaction_n: 20.0,
        max_penetration_m: 0.001,
        net_moment_nm: 0.0,
        net_tangent_force_n: [0.0; 2],
    };
    let mut trace = CreepTrace::new(
        CreepVariant::Baseline,
        config.clone(),
        support(false),
        1000,
        zero,
    )
    .unwrap();
    assert!(trace.record(zero).is_err());
    assert!(
        trace
            .record(HoldSample {
                at: config.rate.tick(1001),
                normal_reaction_n: f64::INFINITY,
                ..zero
            })
            .is_err()
    );
    assert!(trace.finish().is_err());
}

#[test]
fn коэффициент_трения_меняет_срыв_при_той_же_нагрузке() {
    // При 20 Н нормальной и 6 Н тяги снижение μ с 0.5 до 0.25
    // меняет запас с +4 до −1 Н. Ожидаем касательное ускорение 1 Н / 2 кг.
    let held = run_creep(CreepVariant::QuarterStep);
    let slip = creep_with_friction(CreepVariant::QuarterStep, Some(0.25));
    eprintln!("μ=0.5: {:?}; μ=0.25: {:?}", held.metrics(), slip.metrics());
    assert_eq!(
        held.support().required_force_n,
        slip.support().required_force_n
    );
    assert!(
        matches!(held.support().balance().unwrap(), Analysis::Analyzed { value, .. }
        if value.reserve_n == [4.0; 2])
    );
    assert!(
        matches!(slip.support().balance().unwrap(), Analysis::Analyzed { value, .. }
        if value.reserve_n == [-1.0; 2])
    );
    let dv = slip.metrics().final_velocity_m_s[0];
    assert!(held.metrics().final_speed_m_s < tolerances().holding_speed_m_s.min);
    // 3 с от старта, ошибка разгона начального контакта отнесена к 1 мс
    // базовой сетки плюс зарегистрированная погрешность скорости удержания.
    assert!(
        (dv - 3.0 / MASS).abs() < tolerances().holding_speed_m_s.min,
        "{dv}"
    );
}
