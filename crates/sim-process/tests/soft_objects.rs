use std::f64::consts::{FRAC_PI_2, PI};

use sim_process::attachments::*;
use sim_process::work::HazardKind;
use sim_process::{Analysis, Interval, SourcedRange, StepModel};
use sim_signal::{Rate, Rng, Time};
use sim_world::{CompiledWorld, Provenance};

fn band(min: f64, nominal: f64, max: f64) -> SourcedRange {
    SourcedRange {
        min,
        nominal,
        max,
        provenance: Provenance::Measured("синтетический стенд с заданной характеристикой".into()),
    }
}
fn sourced(value: f64) -> SourcedRange {
    band(value, value, value)
}
fn known<T>(value: T) -> Analysis<T> {
    Analysis::Analyzed {
        value,
        scope: "синтетический стенд".into(),
    }
}
fn value<T>(analysis: &Analysis<T>) -> &T {
    match analysis {
        Analysis::Analyzed { value, .. } => value,
        Analysis::NotAnalyzed { reason } => panic!("требовался расчёт: {reason}"),
    }
}
fn unavailable<T>(analysis: &Analysis<T>) -> bool {
    matches!(analysis, Analysis::NotAnalyzed { .. })
}
fn near(actual: f64, expected: f64) {
    // Допуск ошибок округления при подстановке параметров синтетического стенда.
    assert!((actual - expected).abs() < 1e-12, "{actual} != {expected}");
}

fn model(law: SoftLaw, elements: usize) -> SoftModel {
    SoftModel::new(
        SoftBinding {
            name: "образец".into(),
            bodies: (0..=elements).map(|i| format!("узел/{i}")).collect(),
            geometries: (0..elements).map(|i| format!("ветвь/{i}")).collect(),
        },
        law,
    )
    .unwrap()
}

fn tick(model: &mut SoftModel, k: u64, input: &SoftInput) -> SoftStep {
    model
        .advance(
            Interval {
                start: Time(k),
                end: Time(k + 1),
            },
            input,
            &mut Rng::stream(7, "образец"),
        )
        .unwrap()
}

fn input(points: &[[f64; 3]]) -> SoftInput {
    SoftInput {
        nodes: points
            .iter()
            .map(|p| SoftNode {
                point_m: *p,
                velocity_m_s: [0.0; 3],
                com_m: *p,
            })
            .collect(),
        contacts: known(
            (0..points.len() - 1)
                .map(|i| SoftContactReaction {
                    geometry: format!("ветвь/{i}"),
                    reaction_n: 0.0,
                })
                .collect(),
        ),
    }
}

fn cable(elements: usize, rest: f64) -> SoftModel {
    let span = CableSpan {
        rest_length_m: sourced(rest),
        tensile_rigidity_n: sourced(100.0),
        axial_viscosity_ns: sourced(2.0),
        strain: band(-0.5, 0.0, 0.2),
    };
    let bend = CableBend {
        rigidity_nm2: sourced(2.0),
        maximum_curvature_inv_m: sourced(10.0),
        maximum_angle_rad: sourced(2.0),
    };
    model(
        SoftLaw::Cable(CableLaw {
            spans: vec![span; elements],
            bends: vec![bend; elements - 1],
        }),
        elements,
    )
}

fn force(step: &SoftStep, i: usize) -> [f64; 3] {
    step.loads
        .iter()
        .find(|l| l.body == format!("узел/{i}"))
        .unwrap()
        .force_n
}

fn curve(points: &[(f64, f64)]) -> ForceCurve {
    ForceCurve::new(
        points
            .iter()
            .map(|&(x, f)| CurvePoint {
                displacement_m: sourced(x),
                force_n: sourced(f),
            })
            .collect(),
    )
    .unwrap()
}

#[test]
fn cable_tension_energy_contact_and_slack_match_axial_bar() {
    let mut cable = cable(2, 1.0);
    let mut input = input(&[[0.0, 0.0, 0.0], [1.1, 0.0, 0.0], [2.2, 0.0, 0.0]]);
    input.contacts = known(vec![
        SoftContactReaction {
            geometry: "ветвь/1".into(),
            reaction_n: 4.0,
        },
        SoftContactReaction {
            geometry: "ветвь/0".into(),
            reaction_n: 3.0,
        },
    ]);
    let step = tick(&mut cable, 0, &input);
    let response = value(&step.response);
    // EA=100 Н, по 0,1 м удлинения на ветвь 1 м. В каждой 10 Н и 0,5 Дж.
    near(response.reaction_n, 10.0);
    near(response.deformation_m, 0.2);
    near(response.stored_energy_j, 1.0);
    near(response.contact_reaction_n, 7.0);
    near(force(&step, 0)[0], 10.0);
    near(force(&step, 1)[0], 0.0);
    near(force(&step, 2)[0], -10.0);
    let tension = value(&step.tensions_n);
    assert_eq!(tension.len(), 2);
    near(tension[0], 10.0);
    near(tension[1], 10.0);
    near(*value(&step.measure().stored_energy_j), 1.0);

    input.nodes[2].velocity_m_s[0] = 2.0;
    let damped = tick(&mut cable, 1, &input);
    near(value(&damped.tensions_n)[1], 14.0);
    near(*value(&damped.dissipated_power_w), 8.0);
    input.nodes[1].point_m[0] = 0.9;
    input.nodes[2].point_m[0] = 1.8;
    let slack = tick(&mut cable, 2, &input);
    near(value(&slack.response).deformation_m, -0.2);
    near(value(&slack.response).reaction_n, 0.0);
    near(value(&slack.response).stored_energy_j, 0.0);
}

#[test]
fn cable_bending_gradient_matches_virtual_work_and_balances_torque() {
    let mut cable = cable(2, 1.0);
    let input = input(&[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]]);
    let step = tick(&mut cable, 0, &input);
    // При θ=π/2, EI=2 Н·м², L=1 м: запас EI θ²/(2 L), реакция EI θ/L на плече 1 м.
    near(value(&step.response).stored_energy_j, FRAC_PI_2.powi(2));
    near(force(&step, 2)[0], PI);
    near(force(&step, 0)[1], -PI);
    let origin_moment: f64 = input
        .nodes
        .iter()
        .enumerate()
        .map(|(i, node)| {
            let [fx, fy, _] = force(&step, i);
            node.point_m[0] * fy - node.point_m[1] * fx
        })
        .sum();
    near(origin_moment, 0.0);
    for axis in 0..3 {
        near((0..3).map(|i| force(&step, i)[axis]).sum(), 0.0);
    }
    // Независимая вариация угла через acos, без использования сил модели.
    let energy = |dx: f64| (dx / 1.0f64.hypot(dx)).acos().powi(2);
    let dx = 1e-5; // Для центральной разности: малое возмущение относительно плеча 1 м.
    let numeric_force = (energy(-dx) - energy(dx)) / (2.0 * dx);
    assert!((numeric_force - force(&step, 2)[0]).abs() < 1e-8);
}

#[test]
fn cable_spatial_refinement_compares_both_differences_against_circular_arc() {
    let samples = [4, 8, 16].map(|n| {
        let pts: Vec<_> = (0..=n)
            .map(|i| {
                let angle = i as f64 / n as f64;
                [angle.cos(), angle.sin(), 0.0]
            })
            .collect();
        let output = tick(&mut cable(n, 1.0 / n as f64), 0, &input(&pts));
        // Дуга с κ=1 м⁻¹, единичной длиной и EI=2 Н·м² даёт 1 Дж.
        // Дискретная модель теряет по половине двойственного отрезка с каждого конца.
        near(
            value(&output.response).stored_energy_j,
            1.0 - 1.0 / n as f64,
        );
        SpatialSample {
            elements: n,
            at: output.at,
            response: output.response,
        }
    });
    let mut tolerance = SpatialTolerance {
        deformation_m: sourced(0.01),
        reaction_n: sourced(0.0),
        stored_energy_j: sourced(0.13),
        contact_reaction_n: sourced(0.0),
    };
    let result = compare_spatial(&samples, &tolerance).unwrap();
    let comparison = value(&result);
    assert!(comparison.within_tolerance);
    near(comparison.differences[0].stored_energy_j, 0.125);
    near(comparison.differences[1].stored_energy_j, 0.0625);
    tolerance.stored_energy_j = sourced(0.1);
    assert!(unavailable(&compare_spatial(&samples, &tolerance).unwrap()));
    let mut wrong = samples.clone();
    wrong[2].elements = 12;
    assert!(compare_spatial(&wrong, &tolerance).is_err());
    wrong = samples.clone();
    wrong[1].response = Analysis::NotAnalyzed {
        reason: "нет покрытия материалов".into(),
    };
    assert!(unavailable(&compare_spatial(&wrong, &tolerance).unwrap()));
}

#[test]
fn rubber_reversible_energy_and_viscous_work_have_correct_sign() {
    let mut rubber = model(
        SoftLaw::Rubber(RubberLaw {
            free_length_m: sourced(0.04),
            compression: curve(&[(0.0, 0.0), (0.02, 20.0)]),
            damping_ns_m: sourced(20.0),
        }),
        1,
    );
    let mut input = input(&[[0.0; 3], [0.03, 0.0, 0.0]]);
    input.nodes[1].velocity_m_s[0] = -0.2;
    let load = tick(&mut rubber, 0, &input);
    // Независимо от интерполяции: k=1000 Н/м, x=0,01 м, U=k x²/2,
    // F=kx+c ẋ, D=c ẋ². На разгрузке меняет знак только вязкая сила.
    near(value(&load.response).stored_energy_j, 0.05);
    near(value(&load.response).reaction_n, 14.0);
    near(*value(&load.dissipated_power_w), 0.8);
    near(force(&load, 0)[0], -14.0);
    near(force(&load, 1)[0], 14.0);
    input.nodes[1].velocity_m_s[0] = 0.2;
    let unload = tick(&mut rubber, 1, &input);
    near(value(&unload.response).reaction_n, 6.0);
    near(value(&unload.response).stored_energy_j, 0.05);
    near(*value(&unload.dissipated_power_w), 0.8);
    input.nodes[1].velocity_m_s[0] = 1.0;
    near(
        value(&tick(&mut rubber, 2, &input).response).reaction_n,
        0.0,
    );
    input.nodes[1].point_m[0] = 0.05;
    near(
        value(&tick(&mut rubber, 3, &input).response).stored_energy_j,
        0.0,
    );
}

fn foam() -> SoftModel {
    model(
        SoftLaw::Foam(FoamLaw {
            free_length_m: sourced(0.04),
            loading: curve(&[(0.0, 0.0), (0.02, 20.0)]),
            unloading: curve(&[(0.0, 0.0), (0.01, 4.0), (0.02, 20.0)]),
        }),
        1,
    )
}

#[test]
fn foam_hysteresis_matches_integral_and_keeps_history_after_reversal() {
    let mut foam = foam();
    let mut input = input(&[[0.0; 3], [0.04, 0.0, 0.0]]);
    near(value(&tick(&mut foam, 0, &input).response).reaction_n, 0.0);
    input.nodes[1].point_m[0] = 0.02;
    let peak = tick(&mut foam, 1, &input);
    // Работа нагружения (20·0,02)/2 = 0,20 Дж.
    // Две трапеции разгрузки (4+0)·0,01/2 и (20+4)·0,01/2 возвращают 0,14 Дж.
    near(value(&peak.response).stored_energy_j, 0.14);
    near(value(&peak.response).reaction_n, 20.0);
    near(*value(&peak.foam_loss_j), 0.06);
    input.nodes[1].point_m[0] = 0.03;
    let half = tick(&mut foam, 2, &input);
    near(value(&half.response).reaction_n, 4.0);
    near(value(&half.response).stored_energy_j, 0.02);
    input.nodes[1].point_m[0] = 0.02;
    let reloaded = tick(&mut foam, 3, &input);
    near(*value(&reloaded.foam_loss_j), 0.06);
    input.nodes[1].point_m[0] = 0.04;
    let unloaded = tick(&mut foam, 4, &input);
    near(value(&unloaded.response).stored_energy_j, 0.0);
    near(*value(&unloaded.foam_loss_j), 0.06);
}

#[test]
fn missing_pair_curve_and_calibration_do_not_erase_hazards() {
    let mut cable = cable(2, 1.0);
    let mut cable_input = input(&[[0.0; 3], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]);
    cable_input.contacts = known(vec![SoftContactReaction {
        geometry: "ветвь/0".into(),
        reaction_n: 0.0,
    }]);
    let absent = tick(&mut cable, 0, &cable_input);
    assert!(unavailable(&absent.response) && absent.loads.is_empty());
    assert!(
        absent
            .hazards
            .iter()
            .any(|h| h.kind == HazardKind::StoredEnergy && unavailable(&h.coverage))
    );
    cable_input = input(&[[0.0; 3], [1.4, 0.0, 0.0], [2.4, 0.0, 0.0]]);
    assert!(unavailable(&tick(&mut cable, 1, &cable_input).response));
    cable_input.nodes[1].velocity_m_s[1] = f64::NAN;
    let interval = Interval {
        start: Time(2),
        end: Time(3),
    };
    assert!(
        cable
            .advance(interval, &cable_input, &mut Rng::stream(7, "образец"))
            .is_err()
    );
    cable_input.nodes[1].velocity_m_s[1] = 0.0;
    cable_input.contacts = known(vec![SoftContactReaction {
        geometry: "ветвь/0".into(),
        reaction_n: f64::INFINITY,
    }]);
    assert!(
        cable
            .advance(interval, &cable_input, &mut Rng::stream(7, "образец"))
            .is_err()
    );

    let mut foam = foam();
    let mut specimen = input(&[[0.0; 3], [0.04, 0.0, 0.0]]);
    tick(&mut foam, 0, &specimen);
    specimen.nodes[1].point_m[0] = 0.01; // Сжатие 30 мм при калибровке только до 20 мм.
    let outside = tick(&mut foam, 1, &specimen);
    assert!(unavailable(&outside.measure().stored_energy_j));
    assert!(outside.hazards.iter().all(|h| unavailable(&h.coverage)));
    specimen.nodes[1].point_m[0] = 0.03;
    assert!(unavailable(&tick(&mut foam, 2, &specimen).response));
}

#[test]
fn incomplete_or_active_foam_curve_is_refused() {
    let config = |unloading| {
        SoftLaw::Foam(FoamLaw {
            free_length_m: sourced(0.04),
            loading: curve(&[(0.0, 0.0), (0.02, 20.0)]),
            unloading,
        })
    };
    let binding = foam().binding().clone();
    assert!(SoftModel::new(binding.clone(), config(curve(&[(0.0, 0.0), (0.01, 4.0)]))).is_err());
    assert!(
        SoftModel::new(
            binding,
            config(curve(&[(0.0, 0.0), (0.01, 15.0), (0.02, 20.0)]))
        )
        .is_err()
    );

    let loaded = input(&[[0.0; 3], [0.03, 0.0, 0.0]]);
    assert!(unavailable(&tick(&mut foam(), 0, &loaded).response));
}

#[test]
fn cable_load_in_mujoco_changes_momentum_by_calculated_impulse() {
    // Два блока по 2 кг, начальная длина 1,1 м, одна ветвь длиной покоя 1 м.
    let world = CompiledWorld { name: "два блока".into(), seed: 7, physics: Rate::hz(1000).unwrap(),
        mjcf: r#"<mujoco><option gravity="0 0 0" timestep="0.001"/>
          <worldbody>
            <body name="a"><freejoint/><inertial pos="0 0 0" mass="2" diaginertia="1 1 1"/></body>
            <body name="b" pos="1.1 0 0"><freejoint/><inertial pos="0 0 0" mass="2" diaginertia="1 1 1"/></body>
          </worldbody></mujoco>"#.into(), bodies: vec![], joints: vec![], efforts: vec![], drives: vec![],
        sensors: vec![], simplifications: vec![] };
    let mut sim = world.open_contact_sim().unwrap();
    let mut model = cable(1, 1.0);
    let step = tick(
        &mut model,
        0,
        &input(&[[0.0; 3], [sim.qpos()[7], 0.0, 0.0]]),
    );
    // Для свободных тел на оси мировые силы равны этим двум обобщённым силам.
    sim.qfrc_applied_mut()[0] = force(&step, 0)[0];
    sim.qfrc_applied_mut()[6] = force(&step, 1)[0];
    sim.step();
    near(sim.qvel()[0], 10.0 / 2.0 * 0.001);
    near(sim.qvel()[6], -10.0 / 2.0 * 0.001);
    near(2.0 * (sim.qvel()[0] + sim.qvel()[6]), 0.0);
}

#[test]
fn cable_contact_uses_engine_reaction_and_refuses_unresolved_pair() {
    use sim_process::contact::ContactFrame;

    // Ветвь представлена контактным прокси на блоке 2 кг. Его опора должна
    // восстановить вес 2·9,81 Н после установления, без повторного приложения этой силы.
    let world = CompiledWorld {
        name: "блок на плоскости".into(),
        seed: 7,
        physics: Rate::hz(1000).unwrap(),
        mjcf: r#"<mujoco><option timestep="0.001" gravity="0 0 -9.81" integrator="Euler"/>
          <worldbody>
            <geom name="plane" type="plane" size="2 2 0.1"/>
            <body name="block" pos="1 0 0.1"><freejoint/>
              <geom name="branch" type="box" size="0.1 0.1 0.1" mass="2"/>
            </body>
          </worldbody></mujoco>"#
            .into(),
        bodies: vec![],
        joints: vec![],
        efforts: vec![],
        drives: vec![],
        sensors: vec![],
        simplifications: vec![],
    };
    let mut sim = world.open_contact_sim().unwrap();
    for _ in 0..1000 {
        // Выдержка 1 с для установления контакта на шаге 1 мс.
        sim.step();
    }
    let before = sim.qvel()[2];
    let sample = sim.step_with_contacts().unwrap();
    assert!(!sample.contacts.is_empty());
    let interval = Interval {
        start: Time::from_secs(sample.start_s).unwrap(),
        end: Time::from_secs(sample.end_s).unwrap(),
    };
    let frame = ContactFrame::from_engine_step(interval, &sample).unwrap();
    let mut model = SoftModel::new(
        SoftBinding {
            name: "ветвь".into(),
            bodies: vec!["опора".into(), "block".into()],
            geometries: vec!["branch".into()],
        },
        SoftLaw::Cable(CableLaw {
            spans: vec![CableSpan {
                rest_length_m: sourced(1.0),
                tensile_rigidity_n: sourced(100.0),
                axial_viscosity_ns: sourced(0.0),
                strain: band(-0.1, 0.0, 0.1),
            }],
            bends: vec![],
        }),
    )
    .unwrap();
    let mut specimen = input(&[[0.0; 3], [1.0, 0.0, 0.0]]);
    specimen.contacts = model
        .binding()
        .observe_contacts(interval, &frame, &[])
        .unwrap();
    assert!(unavailable(&specimen.contacts));
    let mut incomplete = model.clone();
    let mut rng = Rng::stream(7, "контакт");
    let result = incomplete.advance(interval, &specimen, &mut rng).unwrap();
    assert!(unavailable(&result.response) && result.loads.is_empty());

    specimen.contacts = model
        .binding()
        .observe_contacts(interval, &frame, &[["branch".into(), "plane".into()]])
        .unwrap();
    let result = model.advance(interval, &specimen, &mut rng).unwrap();
    let reaction = value(&result.response).contact_reaction_n;
    let acceleration = (sim.qvel()[2] - before) / 0.001;
    // Допуск 0,1 Н на мягкость решателя и остаточный переходный процесс.
    assert!((reaction - 19.62).abs() < 0.1, "{reaction}");
    assert!(
        (reaction - 2.0 * (9.81 + acceleration)).abs() < 0.1,
        "{reaction}"
    );
    assert!(result.loads.iter().all(|load| load.force_n == [0.0; 3]));
}
