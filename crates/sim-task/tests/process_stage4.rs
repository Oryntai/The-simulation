use std::{
    collections::{BTreeMap, BTreeSet},
    path::Path,
};

use sim_process::{
    Interval, ProcessError, SourcedRange,
    contact::*,
    work::{HazardCheck, HazardKind, ProcessKind, WorkSubject},
};
use sim_runtime::{
    Runtime,
    process::{ProcessEvent, ProcessFrame, ProcessHazard, ProcessModel, ProcessOut},
};
use sim_signal::{Rate, Rng, Time};
use sim_task::{Outcome, process::*};
use sim_world::*;

const FLOOR: &str = "environment/collision/0";
const BLOCK: &str = "блок/блок/collision/0";
const MASS: f64 = 2.0; // Масса и ускорение контрольного стенда в СИ.
const GRAVITY: f64 = 10.0;
const K: f64 = 4000.0;
const MS: u64 = 1_000_000;

fn datum(value: f64, reason: &str) -> SourcedRange {
    SourcedRange {
        min: value,
        nominal: value,
        max: value,
        provenance: Provenance::Measured(reason.into()),
    }
}

fn elastic(value: f64) -> SourcedRange {
    datum(value, "параметр аналитической пружины с демпфером")
}

fn window(start_ms: u64, end_ms: u64) -> Interval {
    Interval {
        start: Time(start_ms * MS),
        end: Time(end_ms * MS),
    }
}

fn build(
    rate: Rate,
    variant: &TrialVariant,
    impact: bool,
    unknown: bool,
) -> Result<Runtime, String> {
    build_with_xml(rate, variant, impact, unknown, |_| {})
}

fn build_with_xml(
    rate: Rate,
    variant: &TrialVariant,
    impact: bool,
    unknown: bool,
    change_xml: impl FnOnce(&mut String),
) -> Result<Runtime, String> {
    let mut floor = Geometry::new(Shape::Box {
        size_m: [2.0, 2.0, 0.2],
    });
    floor.pose.position_m[2] = -0.1;
    // Для d=0,8 и g=10 опорная реакция равна k·δ + d·m·g.
    // Статическое сжатие 1 мм. Для удара начинаем на 0,5 мм над плоскостью.
    let radius: f64 = 0.1;
    let z = if impact {
        radius + 0.0005
    } else {
        radius - (1.0 - 0.8) * MASS * GRAVITY / K
    };
    let inertia = 2.0 * MASS * radius.powi(2) / 5.0;
    let world = World {
        format: FORMAT,
        name: "блок на плоскости".into(),
        seed: 11,
        physics_hz: rate.as_hz() as u64,
        gravity_m_s2: [0.0, 0.0, -GRAVITY],
        environment: Environment {
            collision: vec![floor],
            ..Environment::default()
        },
        objects: vec![],
        devices: vec![Placement {
            name: "блок".into(),
            file: "блок.device".into(),
            pose: Pose {
                position_m: [0.0, 0.0, z],
                ..Pose::default()
            },
        }],
        attachments: vec![],
        sensors: vec![],
    };
    let mut joint = Joint::new(JointType::Free);
    // В аналитическом опыте у свободного блока нет направляющей,
    // поэтому вклад дополнительного трения и инерции оси точно нулевой.
    joint.damping = Some(0.0);
    joint.friction_loss = Some(0.0);
    joint.armature = Some(0.0);
    let device = Device {
        format: FORMAT,
        name: "блок".into(),
        schematic: None,
        links: vec![Link {
            name: "блок".into(),
            parent: None,
            pose: Pose::default(),
            joint,
            mass: Mass::Explicit {
                mass_kg: MASS,
                com_m: [0.0; 3],
                inertia_kg_m2: [inertia, inertia, inertia, 0.0, 0.0, 0.0],
                provenance: elastic(MASS).provenance,
            },
            collision: vec![Geometry::new(Shape::Sphere { radius_m: radius })],
            visual: vec![],
            mounts: vec![],
        }],
        constraints: vec![],
        transmissions: vec![],
        drives: vec![],
        sensors: vec![],
        legacy_v2: None,
    };
    let devices = BTreeMap::from([(
        "блок".into(),
        DeviceInput {
            path: "блок.device".into(),
            device,
        },
    )]);
    let compiled = |options: &ContactBuildOptions| {
        compile_world_with_contact_options(Path::new("блок.toml"), &world, &devices, options)
    };
    // Newton с независимым изменением допуска и числа итераций. Параметры
    // теста из таблицы option MuJoCo 4.0.1 и задания на численное сравнение.
    let finer = variant.kind == VariantKind::Solver;
    let solver = ContactSolver {
        cone: FrictionCone::Elliptic,
        solver: ConstraintSolver::Newton,
        impratio: 1.0,
        tolerance: if finer { 1e-12 } else { 1e-9 },
        iterations: if finer { 80 } else { 40 },
        line_search_tolerance: 1e-12,
        line_search_iterations: 30,
        noslip_tolerance: 1e-9,
        noslip_iterations: 0,
        ccd_tolerance: 1e-8,
        ccd_iterations: 50,
    };
    let mut options = ContactBuildOptions {
        solver: Some(solver),
        ..ContactBuildOptions::default()
    };
    let preview = compiled(&ContactBuildOptions::default())?;
    let k = variant.materials.get("жёсткость").copied().unwrap_or(K);
    let parameters = PairParameters {
        friction: ContactFriction::NormalOnly,
        normal: NormalResponse::SpringDamper {
            stiffness_n_m: elastic(k),
            damping_ns_m: elastic(40.0),
        },
    };
    let table = ContactMaterialTable::new([(["матрица".into(), "опора".into()], parameters)])
        .map_err(|e| e.to_string())?;
    let pairs = table
        .resolve(
            &preview,
            &[
                GeometryMaterial {
                    geometry: BLOCK.into(),
                    material: "матрица".into(),
                },
                GeometryMaterial {
                    geometry: FLOOR.into(),
                    material: "опора".into(),
                },
            ],
            &[PairCalibration {
                geometries: [BLOCK.into(), FLOOR.into()],
                solimp: [0.8, 0.8, 0.001, 0.5, 2.0],
                impedance_source: Provenance::Measured(
                    "постоянный импеданс контрольной опоры".into(),
                ),
                effective_mass_kg: Some(elastic(MASS)),
                friction_reference: FrictionReference::UseNormal,
            }],
            FrictionCone::Elliptic,
        )
        .map_err(|e| e.to_string())?;
    options.pairs = pairs.iter().map(|pair| pair.pair.clone()).collect();
    let mut compiled = compiled(&options)?;
    change_xml(&mut compiled.mjcf);
    let mut rt = Runtime::load(compiled)?;
    rt.record_process_contacts(if unknown { vec![] } else { pairs })?;
    rt.set_process_inventory(&[], &[])?;
    Ok(rt)
}

fn metric(
    name: &str,
    unit: &str,
    series: Series,
    statistic: Statistic,
    tolerance: f64,
) -> Criterion {
    // Для стенда известны аналитическое равновесие и константный импульс.
    let known_error = datum(
        tolerance,
        "ошибка синтетического стенда около аналитического равновесия",
    );
    Criterion {
        name: name.into(),
        unit: unit.into(),
        series,
        statistic,
        tolerance: known_error.clone(),
        remainder: ErrorBounds {
            discretization: known_error.clone(),
            solver: known_error.clone(),
            materials: known_error,
        },
        lower: None,
        upper: None,
    }
}

fn variants() -> Vec<TrialVariant> {
    [
        ("номинал", VariantKind::Baseline),
        ("решатель", VariantKind::Solver),
    ]
    .map(|(name, kind)| TrialVariant {
        name: name.into(),
        kind,
        materials: BTreeMap::new(),
    })
    .into()
}

fn plan(impact: bool) -> ConvergencePlan {
    let pair = [BLOCK.into(), FLOOR.into()];
    let mut peak = metric(
        "пик",
        "N",
        Series::NormalForce(pair.clone()),
        Statistic::Maximum,
        1e-8,
    );
    // Оба предела отдельно от допуска сходимости: в опыте равновесия
    // реакция равна 20 Н, верхняя граница контрольного стенда 21 Н.
    peak.upper = Some(datum(
        if impact { 1000.0 } else { 21.0 },
        "предел опыта с опорой",
    ));
    let mut continuous = metric(
        "кривая",
        "N",
        Series::NormalForce(pair.clone()),
        Statistic::Continuous,
        1e-8,
    );
    if !impact {
        continuous.lower = Some(datum(19.0, "предел длительного удержания блока"));
    }
    ConvergencePlan {
        base_hz: 1000,
        window: window(0, if impact { 40 } else { 10 }),
        window_source: "известный интервал контрольного баланса импульса".into(),
        variants: variants(),
        material_ranges: BTreeMap::new(),
        criteria: vec![
            peak,
            continuous,
            metric(
                "импульс",
                "N*s",
                Series::NormalForce(pair.clone()),
                Statistic::Integral,
                1e-9,
            ),
            metric(
                "вдавливание",
                "m",
                Series::Penetration(pair),
                Statistic::Maximum,
                1e-10,
            ),
            metric(
                "скорость",
                "m/s",
                Series::Velocity(2),
                Statistic::End,
                1e-10,
            ),
        ],
        events: vec![],
        states: vec![],
        creep: vec![],
    }
}

#[test]
fn удержание_сходится_и_реакция_сверена_с_импульсом() {
    let report = assess_processes(&plan(false), |rate, variant| {
        build(rate, variant, false, false)
    })
    .unwrap();
    assert_eq!(report.outcome, Outcome::Pass, "{report:#?}");
    assert_eq!(
        report.trials.iter().map(|r| r.steps).collect::<Vec<_>>(),
        [10, 20, 40, 10, 20, 40]
    );
    let force = &report.metrics[0];
    let impulse = &report.metrics[2];
    let penetration = &report.metrics[3];
    for (metric, expected, tolerance) in [
        (force, MASS * GRAVITY, 1e-9),
        (impulse, MASS * GRAVITY * Time(10 * MS).as_secs(), 1e-10),
        (penetration, (1.0 - 0.8) * MASS * GRAVITY / K, 1e-12),
    ] {
        assert!(
            metric
                .values
                .iter()
                .flatten()
                .all(|values| (values[0] - expected).abs() < tolerance)
        );
    }
    eprintln!(
        "Удержание: пик {:?}, импульс {:?}, вдавливание {:?}",
        force.values, impulse.values, penetration.values
    );
}

#[test]
fn удар_несходится_по_пику_и_проверяется_балансом_импульса() {
    let spec = plan(true);
    let report =
        assess_processes(&spec, |rate, variant| build(rate, variant, true, false)).unwrap();
    assert_eq!(report.outcome, Outcome::NotAnalyzed, "{report:#?}");
    let peak = &report.metrics[0];
    assert!(peak.detail.contains("OutsideTolerance"), "{peak:?}");
    assert!(
        peak.adjacent_differences[0]
            .unwrap()
            .iter()
            .all(|d| *d > spec.criteria[0].tolerance.max)
    );
    for (impulse, velocity) in report.metrics[2]
        .values
        .iter()
        .zip(&report.metrics[4].values)
    {
        let integrated = impulse.unwrap()[0];
        let momentum = MASS * velocity.unwrap()[0];
        let weight_impulse =
            MASS * GRAVITY * (spec.window.end.as_secs() - spec.window.start.as_secs());
        // Независимый импульсный баланс всей сферы, включая свободное падение.
        assert!((integrated - (momentum + weight_impulse)).abs() < 1e-10);
    }
    eprintln!(
        "Удар: пики {:?}, соседние разности {:?}, импульсы {:?}",
        peak.values, peak.adjacent_differences, report.metrics[2].values
    );
}

#[derive(Clone)]
struct StateProbe {
    trigger: bool,
    claim: Option<HazardCheck>,
    estimate: bool,
}
impl ProcessModel for StateProbe {
    fn update(
        &mut self,
        frame: &ProcessFrame<'_>,
        _: &mut Rng,
    ) -> Result<ProcessOut, ProcessError> {
        let mut out = ProcessOut::default();
        let at = frame.next.start;
        // Синтетическая дискретная цепь. Задание появляется в 0,501 мс и
        // принимается только начиная с ближайшей границы шага физики.
        let request = Time(501_000);
        let active = at >= request && self.trigger;
        let mode = if active {
            "включён"
        } else {
            "свободен"
        };
        out.states.insert("состояние".into(), mode.into());
        if active
            && frame
                .completed
                .is_some_and(|last| last.interval.start < request)
        {
            out.events.push(ProcessEvent {
                name: "состояние".into(),
                state: mode.into(),
                requested: request,
                applied: at,
            });
        }
        if let Some(check) = &self.claim {
            let mut bound = datum(1.0, "контрольный предел стенда");
            if self.estimate {
                bound.provenance = Provenance::Estimate("эскиз".into());
            }
            out.hazards.push(ProcessHazard {
                subject: "сменный блок".into(),
                kind: HazardKind::SharpEdge,
                check: check.clone(),
                sources: vec![bound],
            });
        }
        Ok(out)
    }
    fn state_fingerprint(&self) -> u64 {
        u64::from(self.trigger)
    }
}

#[test]
fn окно_проверяет_наличие_события_порядок_и_два_момента() {
    let probe = |trigger| StateProbe {
        trigger,
        claim: None,
        estimate: false,
    };
    let mut spec = plan(false);
    spec.events.push(EventExpectation {
        name: "состояние".into(),
        state: "включён".into(),
        requested: Some(Time(501_000)),
        applied_window: window(0, 1),
        phase_tolerance_ns: datum(MS as f64, "фазовый бюджет до первого грубого такта"),
    });
    spec.states.push(StateExpectation {
        name: "состояние".into(),
        sequence: vec!["свободен".into(), "включён".into()],
    });
    for (present, expected) in [(true, Outcome::Pass), (false, Outcome::NotAnalyzed)] {
        let report = assess_processes(&spec, |rate, variant| {
            let mut rt = build(rate, variant, false, false)?;
            rt.register_process("статус", probe(present || rate.as_hz() != 2000.0))?;
            Ok(rt)
        })
        .unwrap();
        assert_eq!(report.outcome, expected, "{report:#?}");
        assert_eq!(report.trials[0].events[0].requested_ns, 501_000);
        assert_eq!(report.trials[0].events[0].applied_ns, MS);
        assert_eq!(report.trials[2].events[0].applied_ns, 750_000);
        if !present {
            assert!(report.missing.iter().any(|m| m.contains("исход режимов")));
        }
    }
    spec.events.clear();
    let report = assess_processes(&spec, |rate, variant| {
        let mut rt = build(rate, variant, false, false)?;
        rt.register_process("статус", probe(true))?;
        Ok(rt)
    })
    .unwrap();
    assert_eq!(report.outcome, Outcome::NotAnalyzed);
    assert!(
        report.trials[0]
            .missing
            .iter()
            .any(|m| m.contains("добавлено событие"))
    );
}

#[test]
fn пустой_лог_неизвестная_пара_и_неподтверждённый_параметр_блокируют() {
    let mut spec = plan(false);
    let empty = assess_processes(&spec, |rate, _| {
        let mut world = support::world(support::PART);
        world.physics = rate;
        world.mjcf = world.mjcf.replace(
            "timestep=\"0.001\"",
            &format!("timestep=\"{}\"", rate.as_hz().recip()),
        );
        Runtime::load(world)
    })
    .unwrap();
    assert_eq!(empty.outcome, Outcome::NotAnalyzed);
    assert!(empty.trials[0].missing[0].contains("нет сплошного журнала"));
    let unknown =
        assess_processes(&spec, |rate, variant| build(rate, variant, false, true)).unwrap();
    assert_eq!(unknown.outcome, Outcome::NotAnalyzed);
    assert!(
        unknown.trials[0]
            .missing
            .iter()
            .any(|m| m.contains("неизвестная пара"))
    );
    spec.criteria[0].upper.as_mut().unwrap().provenance = Provenance::Estimate("эскиз".into());
    let estimated =
        assess_processes(&spec, |rate, variant| build(rate, variant, false, false)).unwrap();
    assert_eq!(estimated.outcome, Outcome::NotAnalyzed);
    assert!(estimated.metrics[0].detail.contains("источник"));
    spec.criteria[0].tolerance.nominal = f64::NAN;
    assert!(assess_processes(&spec, |_, _| panic!("неконечный ввод до физики")).is_err());
}

#[test]
fn диапазон_материала_нельзя_закрыть_одним_номиналом() {
    let mut spec = plan(false);
    spec.material_ranges.insert(
        "жёсткость".into(),
        SourcedRange {
            min: K - 1.0,
            nominal: K,
            max: K + 1.0,
            provenance: elastic(K).provenance,
        },
    );
    for trial in &mut spec.variants {
        trial.materials.insert("жёсткость".into(), K);
    }
    let incomplete =
        assess_processes(&spec, |rate, trial| build(rate, trial, false, false)).unwrap();
    assert_eq!(incomplete.outcome, Outcome::NotAnalyzed);
    assert!(
        incomplete
            .missing
            .iter()
            .any(|m| m.contains("не исследована граница"))
    );
    for endpoint in [K - 1.0, K + 1.0] {
        spec.variants.push(TrialVariant {
            name: format!("жёсткость {endpoint}"),
            kind: VariantKind::Material,
            materials: BTreeMap::from([("жёсткость".into(), endpoint)]),
        });
    }
    let complete = assess_processes(&spec, |rate, trial| build(rate, trial, false, false)).unwrap();
    assert!(
        !complete
            .missing
            .iter()
            .any(|m| m.contains("не исследована граница"))
    );
    assert!(complete.metrics[0].materials.unwrap() > spec.criteria[0].remainder.materials.max);
}

#[test]
fn опасность_следует_из_состава_и_переживает_удаление_декларации() {
    let spec = plan(false);
    for (claim, estimate, expect) in [
        (None, false, Outcome::NotAnalyzed),
        (
            Some(HazardCheck::WithinLimit {
                scope: "проверено по заданному пределу".into(),
            }),
            true,
            Outcome::NotAnalyzed,
        ),
        (
            Some(HazardCheck::WithinLimit {
                scope: "проверено по заданному пределу".into(),
            }),
            false,
            Outcome::Pass,
        ),
        (
            Some(HazardCheck::Violation {
                reason: "нарушение измерено".into(),
            }),
            false,
            Outcome::Fail,
        ),
    ] {
        let report = assess_processes(&spec, |rate, variant| {
            // В этой сцене состав устанавливается сборщиком до регистрации моделей.
            let mut rt = build(rate, variant, false, false)?;
            let required = sim_process::work::required_hazards(
                &[WorkSubject {
                    name: "сменный блок".into(),
                    properties: BTreeSet::from([ProcessKind::SharpEdge]),
                }],
                &[],
            )
            .map_err(|e| e.to_string())?;
            rt.require_process_hazards(&required)?;
            rt.register_process(
                "проверка",
                StateProbe {
                    trigger: false,
                    claim: claim.clone(),
                    estimate,
                },
            )?;
            Ok(rt)
        })
        .unwrap();
        assert_eq!(report.outcome, expect, "{report:#?}");
        if claim.is_none() {
            assert!(
                report.trials[0]
                    .missing
                    .iter()
                    .any(|m| m.contains("SharpEdge"))
            );
        }
    }
}

mod support;

#[test]
fn оценщик_оставляет_покрытие_записанного_радио_неподтверждённым() {
    use sim_runtime::control::{self, ControlAssets, ControlConfig, RunSpec};
    use sim_task::{Assessor, Fingerprint, Task};
    use std::path::PathBuf;

    struct Assets;
    impl ControlAssets for Assets {
        fn read(&self, name: &str) -> Result<Vec<u8>, String> {
            match name {
                "образ" => {
                    Ok(include_bytes!("../../sim-boards/testdata/vmcu_ports.elf").to_vec())
                }
                "параметры" => Ok([b'S', 0, 0, 0]
                    .into_iter()
                    .chain(1000_u32.to_le_bytes())
                    .collect()),
                "слоты" => {
                    Ok(b"accepts_through_ns,delivers_at_ns\n1000000,2000000\n".to_vec())
                }
                _ => Err(format!("неизвестный файл {name}")),
            }
        }
        fn executable(&self, _: &str) -> Result<PathBuf, String> {
            Err("нет внешней программы".into())
        }
    }

    let config = ControlConfig::parse(
        r#"version=1
[[board]]
name='плата'
catalog='vmcu-c3'
image='образ'
config='параметры'
rate_hz=4000
source='контрольный стенд'
[[board_input]]
name='вход'
board='плата'
signal='измерение'
unit='m'
kind='adc'
port=0
max_age_ns=2000000
scale=1
offset=0
fallback=0
source='контрольный стенд'
[[packet]]
name='связь'
kind='scheduled_radio'
to_board='плата'
to_port=0
capacity=2
max_age_ns=3000000
rssi_dbm=-60
max_bytes=20
slots_asset='слоты'
source='запись контрольного стенда'
[[transmission]]
route='связь'
at_ns=250000
bytes=[1]
source='контрольный стенд'
"#,
    )
    .unwrap();
    let document = support::document(1.0, 6 * MS, true);
    let world = support::world(support::PART);
    let mut inputs = support::inputs();
    let install = |rt: &mut Runtime| -> Result<(), String> {
        rt.register_observation("измерение", "m")?;
        control::install(
            rt,
            &config,
            &Assets,
            RunSpec {
                world: b"fixture",
                catalog: b"fixture",
                task: document.as_bytes(),
                criteria: document.as_bytes(),
                mode: "рабочий",
                horizon: Time(support::HORIZON),
                model_ports: &[],
            },
        )
    };
    let mut probe = Runtime::load(world.clone()).unwrap();
    install(&mut probe).unwrap();
    let run = control::run(&probe).unwrap();
    inputs.build = run.identity().build.clone();
    inputs.catalog = Fingerprint::from_fnv1a(run.identity().catalog);
    let assessor = Assessor::new(
        Task::parse(&document).unwrap(),
        support::topology(true),
        world,
        inputs.clone(),
    )
    .unwrap();
    let case = assessor
        .required_cases()
        .iter()
        .find(|case| case.fault.is_none())
        .unwrap();
    let result = assessor
        .execute(case, |rt, _, _| {
            install(rt)?;
            Ok(inputs.clone())
        })
        .unwrap();
    let report = result.report();
    assert!(report.completed, "{report:#?}");
    assert_eq!(
        report.control.as_ref().unwrap().packet_deliveries["связь"],
        1
    );
    assert_eq!(report.radio_coverage["связь"], Outcome::NotAnalyzed);
    assert_ne!(report.outcome, Outcome::Pass);
    assert!(
        report
            .missing
            .iter()
            .any(|m| m.contains("доставка взята из записи"))
    );
}

#[derive(Clone)]
struct IdealHold;

impl ProcessModel for IdealHold {
    fn update(&mut self, _: &ProcessFrame<'_>, _: &mut Rng) -> Result<ProcessOut, ProcessError> {
        let mut out = ProcessOut::default();
        out.hold_scalar_axis("ось".into());
        Ok(out)
    }

    fn state_fingerprint(&self) -> u64 {
        0 // У закона нет собственного состояния.
    }
}

#[test]
fn удержание_без_решателя_и_исключение_контакта_не_дают_доказательства() {
    let mut p = plan(false);
    p.variants.truncate(1);
    p.criteria.truncate(1);
    p.criteria[0].series = Series::Displacement(0);
    p.criteria[0].unit = "m".into();
    // Неподвижный стержень обязан оставаться в пределах 1 мм на всех сетках.
    p.criteria[0].upper = Some(datum(0.001, "граница стендового опыта"));
    let mut observed = Vec::new();
    for mode in ["контроль", "идеализация", "исключение"] {
        let report = assess_processes(&p, |rate, _| {
            // Стержень единичной массы на горизонтальной оси, нулевая нагрузка.
            // В третьем опыте геометрии перекрываются на радиус сферы.
            let geometry = if mode == "исключение" {
                r#"<geom name="стержень" type="sphere" size="0.1"/>"#
            } else {
                ""
            };
            let fixture = if mode == "исключение" {
                r#"<body name="опора"><geom name="опора" type="sphere" pos="0.1 0 0" size="0.1"/></body>"#
            } else {
                ""
            };
            let exclusion = if mode == "исключение" {
                r#"<contact><exclude body1="подвижное" body2="опора"/></contact>"#
            } else {
                ""
            };
            let compiled = CompiledWorld {
                name: "стержень".into(),
                seed: 11,
                physics: rate,
                mjcf: format!(r#"<mujoco><option gravity="0 0 0" timestep="{}"/>
                    <worldbody>{fixture}<body name="подвижное">
                    <joint name="ось" type="slide" axis="1 0 0"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                    {geometry}</body></worldbody>{exclusion}</mujoco>"#, 1.0 / rate.as_hz()),
                bodies: vec![],
                joints: vec![CompiledJoint {
                    name: "ось".into(),
                    owner: "стержень".into(),
                    kind: CompiledJointKind::Slide,
                }],
                efforts: vec![],
                drives: vec![],
                sensors: vec![],
                simplifications: vec![],
            };
            let mut rt = Runtime::load(compiled)?;
            rt.set_process_inventory(&[], &[])?;
            if mode == "идеализация" {
                rt.register_process("проба", IdealHold)?;
            }
            Ok(rt)
        }).unwrap();
        assert!(report.trials.iter().all(|t| t.completed));
        eprintln!(
            "{mode}: {:?} на {} сетках",
            report.outcome,
            report.trials.len()
        );
        observed.push(report.outcome);
    }
    assert_eq!(
        observed,
        [Outcome::Pass, Outcome::NotAnalyzed, Outcome::NotAnalyzed]
    );
}

#[test]
fn неверная_сетка_и_подмена_коэффициента_mjcf_блокируют() {
    let spec = plan(false);
    let single = assess_processes(&spec, |_, variant| {
        build(Rate::hz(spec.base_hz).unwrap(), variant, false, false)
    })
    .unwrap();
    assert_eq!(single.outcome, Outcome::NotAnalyzed);
    assert_eq!(
        single.trials.iter().filter(|trial| trial.completed).count(),
        2
    );
    assert!(
        single
            .trials
            .iter()
            .filter(|trial| !trial.completed)
            .all(|trial| trial.missing[0].contains("чужую сетку"))
    );

    let mismatch = assess_processes(&spec, |rate, variant| {
        build_with_xml(rate, variant, false, false, |xml| {
            // Пара единственная, в явной записи меняем ровно первую
            // тангенциальную компоненту трения на 0.5 (единицы безразмерны).
            let start = xml.find("<pair ").unwrap();
            let start = start + xml[start..].find("friction=\"").unwrap() + "friction=\"".len();
            let end = start + xml[start..].find(' ').unwrap();
            assert_ne!(&xml[start..end], "0.5");
            xml.replace_range(start..end, "0.5");
        })
    })
    .unwrap();
    assert_eq!(mismatch.outcome, Outcome::NotAnalyzed);
    assert!(
        mismatch
            .trials
            .iter()
            .all(|trial| !trial.completed
                && trial.missing[0].contains("расходится с моделью MuJoCo"))
    );
}
