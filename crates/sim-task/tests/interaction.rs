use sim_process::{
    Interval, SourcedRange,
    attachments::{
        CompliantPad, CurvePoint, ForceCurve, PadCalibration, PadLaw, PadReactionOwner, PadSpec,
        PatchAreaPoint,
    },
};
use sim_runtime::{
    Forces, Object, Runtime, Status, Truth, TruthId,
    control::{self, ControlAssets, ControlConfig, ModelPort, RunSpec},
    process::interaction::*,
    register_interaction,
};
use sim_signal::{ChannelId, Command, Rate, Rng, Time};
use sim_task::{
    Fingerprint, Outcome, assess_interaction, assess_interaction_grids, interaction::*,
};
use sim_world::{
    CompiledDrive, CompiledJoint, CompiledJointKind, CompiledWorld, EffortChannel, Provenance,
};

const MS: u64 = 1_000_000;
const WINDOW: Interval = Interval {
    // Площадка B начинает калибровать след после первого завершённого кадра.
    start: Time(10 * MS),
    end: Time(40 * MS),
};
const REGIONS: [&str; 2] = ["зона а", "зона б"];
const PATCHES: [&str; 2] = ["пятно а", "пятно б"];
const AXES: [&str; 2] = ["ось а", "ось б"];
const RESPONSE_AXIS: &str = "ось отклика";

fn source() -> Provenance {
    Provenance::Measured("протокол синтетического стенда, заданные значения".into())
}
fn band(min: f64, max: f64) -> SourcedRange {
    SourcedRange {
        min,
        nominal: min,
        max,
        provenance: source(),
    }
}
fn exact(x: f64) -> SourcedRange {
    band(x, x)
}
fn reach() -> PowerInventory {
    PowerInventory {
        reach: Reach::Reachable,
        topology: Some(source()),
        required_sources: Default::default(),
        sources: Default::default(),
    }
}

#[derive(Clone, Copy)]
enum Area {
    Missing,
    Fixed,
    ByCompression,
}

fn runtime(hz: u64, area: Area, balanced: bool, duration: Time, fault: bool) -> Runtime {
    runtime_with_response(hz, area, balanced, duration, fault, 1.0)
}

fn runtime_with_response(
    hz: u64,
    area: Area,
    balanced: bool,
    duration: Time,
    fault: bool,
    response_cap: f64,
) -> Runtime {
    // Две независимые каретки по 2 кг, Euler, без тяжести. Сжатие 10/5 мм
    // при 1000 Н/м даёт 10/5 Н. В статическом опыте силы компенсированы.
    let mjcf = format!(
        r#"
<mujoco>
  <option timestep="{}" gravity="0 0 0" integrator="Euler"/>
  <default><geom type="sphere" size=".02" mass="2" contype="0" conaffinity="0"/></default>
  <worldbody>
    <body name="опора а"><geom name="опора-га"/></body>
    <body name="блок а"><joint name="ось а" type="slide" axis="0 0 1"/><geom name="блок-га"/></body>
    <body name="опора б"><geom name="опора-гб"/></body>
    <body name="блок б"><joint name="ось б" type="slide" axis="0 0 1"/><geom name="блок-гб"/></body>
    <body name="блок отклика"><joint name="ось отклика" type="slide" axis="1 0 0"/><geom/></body>
  </worldbody>
</mujoco>"#,
        1.0 / hz as f64
    );
    let positions = [0.03, 0.035];
    let pads = ["а", "б"]
        .into_iter()
        .enumerate()
        .map(|(i, label)| {
            let bodies = [format!("опора {label}"), format!("блок {label}")];
            let mounts = bodies.clone().map(|body| LocalMount {
                point: LocalPoint {
                    body,
                    position_m: [0.0; 3],
                },
                normal: [0.0, 0.0, 1.0],
                tangent: [1.0, 0.0, 0.0],
            });
            let mut model = CompliantPad::new(PadSpec {
                name: PATCHES[i].into(),
                bodies,
                geometries: [format!("опора-г{label}"), format!("блок-г{label}")],
                owner: PadReactionOwner::Model(Box::new(PadLaw {
                    free_gap_m: exact(0.04),
                    compression: ForceCurve::new(
                        [0.0, 0.04]
                            .map(|x| CurvePoint {
                                displacement_m: exact(x),
                                force_n: exact(1000.0 * x),
                            })
                            .into(),
                    )
                    .unwrap(),
                    damping_ns_m: exact(0.0),
                    shear_stiffness_n_m: exact(100.0),
                    friction: exact(0.4),
                    capacity_n: exact(100.0),
                })),
                area_m2: matches!(area, Area::Fixed).then(|| band(0.002, 0.004)),
                calibration: PadCalibration {
                    compression_m: band(0.0, 0.04),
                    offset_m: band(0.0, 0.01),
                    axis_error_rad: band(0.0, 0.01),
                    relative_speed_m_s: band(0.0, 2.0),
                },
            })
            .unwrap();
            if matches!(area, Area::ByCompression) {
                // Калиброванный плоский образец: на рабочем ходе перекрытие
                // одинаково, границы измерения площади 20–40 см².
                model = model
                    .with_area_curve(
                        [0.0, 0.04]
                            .map(|compression_m| PatchAreaPoint {
                                compression_m,
                                area_m2: band(0.002, 0.004),
                            })
                            .into(),
                    )
                    .unwrap();
            }
            PadBinding {
                mounts,
                events: vec![],
                model,
            }
        })
        .collect();
    let world = CompiledWorld {
        name: "две каретки".into(),
        seed: 3,
        physics: Rate::hz(hz).unwrap(),
        mjcf,
        bodies: vec![],
        drives: fault
            .then(|| CompiledDrive {
                name: "привод стенда".into(),
                owner: "стенд".into(),
                part: "стендовый исполнитель".into(),
                effort: AXES[0].into(),
                joint: AXES[0].into(),
                params: Default::default(),
            })
            .into_iter()
            .collect(),
        sensors: vec![],
        simplifications: vec![],
        joints: AXES
            .into_iter()
            .chain([RESPONSE_AXIS])
            .map(|name| CompiledJoint {
                name: name.into(),
                owner: "стенд".into(),
                kind: CompiledJointKind::Slide,
            })
            .collect(),
        efforts: AXES
            .into_iter()
            .chain([RESPONSE_AXIS])
            .map(|name| EffortChannel {
                name: name.into(),
                joint: name.into(),
                dof: 0,
            })
            .chain(fault.then(|| EffortChannel {
                name: "питание".into(),
                joint: AXES[0].into(),
                dof: 0,
            }))
            .collect(),
    };
    let mut rt = Runtime::load(world).unwrap();
    register_interaction(
        &mut rt,
        ResolvedInteraction {
            name: "опыт".into(),
            resources: 5,
            variant: 7,
            regions: REGIONS
                .into_iter()
                .zip(["блок-га", "блок-гб"])
                .map(|(r, g)| (r.into(), vec![g.into()]))
                .collect(),
            pads,
            routes: vec![],
            motion: MotionProfile::Dynamic {
                axes: positions
                    .into_iter()
                    .enumerate()
                    .map(|(i, q)| MotionAxis {
                        channel: ChannelId(i as u32),
                        initial_position: q,
                        initial_velocity: 0.0,
                        reference: Trajectory {
                            source: source(),
                            knots: vec![(Time::ZERO, q)],
                        },
                        stiffness: exact(0.0),
                        damping: exact(0.0),
                        effort_limit: exact(0.0),
                    })
                    .chain([MotionAxis {
                        channel: ChannelId(2),
                        initial_position: 0.0,
                        initial_velocity: 0.0,
                        reference: Trajectory {
                            source: source(),
                            knots: vec![(Time::ZERO, 0.0)],
                        },
                        // Независимая каретка отклика, k=100 Н/м, предел 1 Н.
                        stiffness: exact(100.0),
                        damping: exact(0.0),
                        effort_limit: exact(response_cap),
                    }])
                    .collect(),
                maneuver: None,
                response: Some(Box::new(Response {
                    enabled: true,
                    initiator: "опыт отклика каретки".into(),
                    patch: PATCHES[0].into(),
                    // Реакция 10 Н на несущей опоре, контрольный порог 5 Н.
                    threshold_n: exact(5.0),
                    latency_ns: 10 * MS,
                    latency_source: source(),
                    offsets: [(
                        ChannelId(2),
                        Trajectory {
                            source: source(),
                            // Испытательное перемещение на 5 мм за 10 мс.
                            knots: vec![(Time::ZERO, 0.0), (Time(10 * MS), 0.005)],
                        },
                    )]
                    .into(),
                })),
            },
        },
    )
    .unwrap();
    if fault {
        install_power(&mut rt, positions);
    } else if balanced {
        rt.act(&Command {
            targets: positions
                .into_iter()
                .enumerate()
                .map(|(i, q)| (ChannelId(i as u32), -1000.0 * (0.04 - q)))
                .collect(),
        })
        .unwrap();
    }
    assert_eq!(
        rt.run_until(duration).unwrap(),
        Status::Running,
        "{:?}",
        rt.failure()
    );
    rt
}

fn setup(area: bool, balanced: bool, stop: Time) -> ([Runtime; 3], InteractionPlan) {
    let area = if area {
        Area::ByCompression
    } else {
        Area::Missing
    };
    let runs = [100, 200, 400].map(|hz| runtime(hz, area, balanced, stop, false));
    let limits = PATCHES
        .into_iter()
        .zip(REGIONS)
        .flat_map(|(patch, region)| {
            // Проверяемые пороги фикстуры в Н и Па, отдельно от медицинских данных.
            [
                (ContactMetric::NormalForce, 12.0),
                (ContactMetric::MeanPressure, 6000.0),
                (ContactMetric::MeanShear, 40.0),
            ]
            .map(|(metric, max)| LimitCriterion {
                patch: patch.into(),
                region: region.into(),
                metric,
                kind: CriterionKind::Exposure,
                limit: exact(max),
                validation: Some(source()),
                geometry: "плоская проба".into(),
                mode: "статический".into(),
                population: "условный образец".into(),
                duration_s: band(0.02, 0.05),
                max_repetitions: 1,
                area_m2: matches!(
                    metric,
                    ContactMetric::MeanPressure | ContactMetric::MeanShear
                )
                .then(|| band(0.002, 0.004)),
            })
        })
        .collect();
    let plan = InteractionPlan {
        build: Fingerprint::of(b"fixtures"),
        legacy_design: Some(8),
        construction_complete: true,
        grids: runs.each_ref().map(|rt| rt.interaction_identity().unwrap()),
        variant: "покой".into(),
        window: WINDOW,
        regions: REGIONS.map(str::to_owned).into(),
        patches: PATCHES
            .map(|patch| {
                (
                    patch.into(),
                    PatchPlan {
                        geometry: "плоская проба".into(),
                        mode: "статический".into(),
                        population: "условный образец".into(),
                        repetitions: 1,
                        absence_basis: None,
                    },
                )
            })
            .into(),
        // Начало контакта известно до целого такта. Для остальных величин
        // допуск покрывает только округление статического ручного расчёта.
        tolerances: ContactTolerances {
            onset_s: exact(0.006),
            force_n: exact(1e-10),
            impulse_ns: exact(1e-10),
            duration_s: exact(1e-10),
            drift_m: exact(1e-10),
            stress_pa: exact(1e-8),
        },
        limits,
        power: reach(),
        power_loss: None,
        discrepancies: vec![],
    };
    (runs, plan)
}

fn near(actual: f64, expected: f64) {
    // Для проверяемых точек статического решения.
    assert!((actual - expected).abs() < 1e-9, "{actual} != {expected}");
}

#[test]
fn fixed_area_cannot_certify_a_loaded_assembly() {
    let runs = [100, 200, 400].map(|hz| runtime(hz, Area::Fixed, true, WINDOW.end, false));
    let (_, mut plan) = setup(true, true, WINDOW.end);
    plan.grids = runs.each_ref().map(|rt| rt.interaction_identity().unwrap());
    let evidence = assess_interaction_grids(runs.each_ref(), &plan).unwrap();
    assert!(evidence.blockers().iter().any(|b| b.contains("площади")));
    assert_eq!(evidence.decision(), SafetyDecision::BenchOnly);
    assert!(evidence.findings()[1].measurement.is_none());
}

#[test]
fn uncovered_reference_cannot_certify_a_loaded_assembly() {
    let (runs, mut plan) = setup(true, true, WINDOW.end);
    // Эталонная пара для проверки свёртки, в единицах стенда.
    plan.discrepancies.push(ReferenceDiscrepancy {
        id: "контроль".into(),
        mode: "нагруженный блок".into(),
        simulated: 2.0,
        reference: 1.0,
        unit: "Н".into(),
        source: "ручной баланс оснастки".into(),
        tolerance: "0 Н".into(),
        note: "для проверяемого пути требуется отдельная модель".into(),
    });
    let evidence = assess_interaction_grids(runs.each_ref(), &plan).unwrap();
    assert!(evidence.blockers().iter().any(|b| b.contains("контроль")));
    assert!(
        evidence
            .findings()
            .iter()
            .all(|f| f.outcome == Outcome::Pass)
    );
    assert_eq!(evidence.decision(), SafetyDecision::BenchOnly);
}

#[test]
fn response_marker_without_applied_effort_does_not_certify_the_run() {
    // Каретка получает отметку и задание отклика, но её предел усилия снят до нуля.
    let runs = [100, 200, 400]
        .map(|hz| runtime_with_response(hz, Area::ByCompression, true, WINDOW.end, false, 0.0));
    let (_, mut plan) = setup(true, true, WINDOW.end);
    plan.grids = runs.each_ref().map(|rt| rt.interaction_identity().unwrap());
    for rt in &runs {
        assert!(
            rt.process_journal()
                .iter()
                .flat_map(|tick| tick.outputs.values())
                .flat_map(|out| out.interaction.as_ref())
                .flat_map(|out| &out.motion_events)
                .any(|event| event.contact_at.is_some())
        );
    }
    let evidence = assess_interaction_grids(runs.each_ref(), &plan).unwrap();
    assert!(
        evidence
            .findings()
            .iter()
            .all(|row| row.outcome == Outcome::Pass)
    );
    assert!(
        evidence
            .blockers()
            .iter()
            .any(|b| b.contains("реакция на контакт не подтверждена"))
    );
    assert_eq!(evidence.decision(), SafetyDecision::BenchOnly);
}

#[test]
fn a_reachable_assembly_cannot_hide_an_installed_drive_behind_off_participant() {
    let runs = [100, 200, 400].map(|hz| runtime(hz, Area::ByCompression, true, WINDOW.end, true));
    let (_, mut plan) = setup(true, true, WINDOW.end);
    plan.grids = runs.each_ref().map(|rt| rt.interaction_identity().unwrap());
    plan.power = power(Some(2.0), Some(0.1));
    let source = plan.power.sources.remove("накопитель").unwrap();
    plan.power.sources.insert(
        "привод стенда".into(),
        PowerSource {
            reach: Reach::OffParticipant,
            ..source
        },
    );
    plan.power.required_sources = ["привод стенда".into()].into();
    let evidence = assess_interaction_grids(runs.each_ref(), &plan).unwrap();
    assert!(evidence.power().iter().any(|row| row.confirmed_hazard));
}

#[test]
fn all_grids_and_regions_are_measured_and_only_one_excess_rejects() {
    let (runs, mut plan) = setup(true, true, WINDOW.end);
    let result = assess_interaction_grids(runs.each_ref(), &plan).unwrap();
    assert_eq!(result.outcome(), Outcome::Pass, "{:?}", result.blockers());
    assert_eq!(result.decision(), SafetyDecision::Passed);
    assert_eq!(
        result.grids().iter().map(|g| g.rate_hz).collect::<Vec<_>>(),
        [100.0, 200.0, 400.0]
    );
    assert_eq!(result.findings().len(), 6);
    for row in result.grids() {
        for (i, f) in [10.0, 5.0].into_iter().enumerate() {
            let m = &row.patches[PATCHES[i]];
            near(m.peak_normal_n, f);
            near(m.peak_mean_pressure_pa.unwrap()[0], f / 0.004);
            near(m.peak_mean_pressure_pa.unwrap()[1], f / 0.002);
            near(m.impulse_ns, f * 0.03);
            near(m.duration_s, 0.03);
            near(row.regions[REGIONS[i]].peak_normal_n, f);
        }
    }
    let onset = &result.comparisons()["patch/пятно а"][0];
    near(onset.differences[0].unwrap(), 0.005);
    near(onset.differences[1].unwrap(), 0.0025);
    assert!(result.matches(&plan, 8));
    assert!(!result.matches(&plan, 9));
    plan.limits[0].limit = exact(9.0);
    assert!(!result.matches(&plan, 8));
    let result = assess_interaction_grids(runs.each_ref(), &plan).unwrap();
    assert_eq!(result.outcome(), Outcome::Fail);
    assert_eq!(result.decision(), SafetyDecision::Rejected);
    assert_eq!(
        result
            .findings()
            .iter()
            .filter(|r| r.outcome == Outcome::Fail)
            .count(),
        1
    );
    assert_eq!(result.findings()[0].region, REGIONS[0]);
    assert_eq!(result.findings()[3].outcome, Outcome::Pass);
}

#[test]
fn missing_identity_window_region_area_or_applicability_cannot_pass() {
    let (runs, plan) = setup(true, true, WINDOW.end);
    let mut bad = plan.clone();
    bad.grids[1].profile ^= 1;
    assert_eq!(
        assess_interaction_grids(runs.each_ref(), &bad)
            .unwrap()
            .outcome(),
        Outcome::NotAnalyzed
    );
    bad = plan.clone();
    bad.grids[0].version += 1;
    assert!(
        !assess_interaction_grids(runs.each_ref(), &bad)
            .unwrap()
            .matches(&plan, 8)
    );
    assert_eq!(
        assess_interaction(&runs[0], &plan).unwrap().outcome(),
        Outcome::NotAnalyzed
    );
    let (short, p) = setup(true, true, Time(30 * MS));
    let incomplete = assess_interaction_grids(short.each_ref(), &p).unwrap();
    assert!(
        incomplete
            .blockers()
            .iter()
            .any(|s| s.contains("неполное окно"))
    );
    assert_eq!(incomplete.outcome(), Outcome::NotAnalyzed);
    bad = plan.clone();
    bad.regions.insert("пропущенная зона".into());
    bad.limits[0].region = "пропущенная зона".into();
    let uncovered = assess_interaction_grids(runs.each_ref(), &bad).unwrap();
    assert_eq!(uncovered.outcome(), Outcome::NotAnalyzed);
    assert_eq!(uncovered.findings()[0].outcome, Outcome::NotAnalyzed);
    let (no_area, p) = setup(false, true, WINDOW.end);
    let m = assess_interaction_grids(no_area.each_ref(), &p).unwrap();
    assert_eq!(m.outcome(), Outcome::NotAnalyzed);
    assert!(m.findings()[1].measurement.is_none());
    assert_eq!(m.findings()[1].outcome, Outcome::NotAnalyzed);
    for invalid in [f64::NAN, f64::INFINITY] {
        bad = plan.clone();
        bad.limits[0].limit = exact(invalid);
        assert!(assess_interaction_grids(runs.each_ref(), &bad).is_err());
    }
    for edit in [
        (|c: &mut LimitCriterion| c.validation = None) as fn(&mut LimitCriterion),
        |c| c.mode = "за пределами протокола".into(),
        |c| c.duration_s = exact(0.0),
        |c| c.limit.provenance = Provenance::Estimate("ориентир".into()),
    ] {
        bad = plan.clone();
        edit(&mut bad.limits[0]);
        let result = assess_interaction_grids(runs.each_ref(), &bad).unwrap();
        assert_eq!(result.findings()[0].outcome, Outcome::NotAnalyzed);
        assert!(result.findings()[0].measurement.is_some());
    }
}

#[test]
fn unequal_peaks_block_even_if_integral_is_accepted() {
    let (runs, mut plan) = setup(true, false, WINDOW.end);
    // Окно начинается после первого такта. Пик убывающего усилия теперь
    // зависит от шага Euler, а допустимая разница импульса шире.
    plan.window.start = Time(10 * MS);
    plan.tolerances.impulse_ns = exact(1.0);
    let result = assess_interaction_grids(runs.each_ref(), &plan).unwrap();
    let rows = &result.comparisons()["patch/пятно а"];
    assert!(
        rows.iter()
            .any(|r| r.quantity == "impulse" && r.outcome == Outcome::Pass)
    );
    assert!(
        rows.iter()
            .any(|r| r.quantity == "normal" && r.outcome == Outcome::NotAnalyzed)
    );
    assert_eq!(result.findings()[0].outcome, Outcome::NotAnalyzed);
    assert_eq!(result.outcome(), Outcome::NotAnalyzed);
}

fn power(stored: Option<f64>, delivered: Option<f64>) -> PowerInventory {
    let mut inventory = reach();
    let name = "накопитель".to_owned();
    inventory.required_sources.insert(name.clone());
    inventory.sources.insert(
        name,
        PowerSource {
            reach: Reach::Reachable,
            paths: [
                EnergyPath::Thermal,
                EnergyPath::Electrical,
                EnergyPath::Mechanical,
            ]
            .map(|path| {
                (
                    path,
                    EnergyBounds {
                        stored_j: stored.map(exact),
                        deliverable_j: delivered.map(exact),
                        hazardous_above_j: Some(exact(1.0)),
                        violation: None,
                    },
                )
            })
            .into(),
        },
    );
    inventory
}

#[test]
fn hazardous_stored_or_delivered_energy_and_unknown_power_cap_all_other_passes() {
    let (runs, mut plan) = setup(true, true, WINDOW.end);
    for (stored, delivered, hazard) in [
        (Some(2.0), Some(0.1), true),
        (Some(0.1), Some(2.0), true),
        (None, Some(0.1), false),
    ] {
        plan.power = power(stored, delivered);
        let r = assess_interaction_grids(runs.each_ref(), &plan).unwrap();
        assert_eq!(r.decision(), SafetyDecision::BenchOnly);
        assert_eq!(r.outcome(), Outcome::NotAnalyzed);
        assert!(r.findings().iter().all(|f| f.outcome == Outcome::Pass));
        assert_eq!(r.power().iter().any(|f| f.confirmed_hazard), hazard);
    }
    plan.power = power(Some(f64::NAN), Some(0.1));
    assert!(assess_interaction_grids(runs.each_ref(), &plan).is_err());
    plan.power = power(Some(0.1), Some(0.2));
    assert_eq!(
        assess_interaction_grids(runs.each_ref(), &plan)
            .unwrap()
            .decision(),
        SafetyDecision::Passed
    );
    plan.power
        .required_sources
        .insert("пропущенный источник".into());
    assert_eq!(
        assess_interaction_grids(runs.each_ref(), &plan)
            .unwrap()
            .decision(),
        SafetyDecision::BenchOnly
    );
    plan.construction_complete = false;
    assert_eq!(
        assess_interaction_grids(runs.each_ref(), &plan)
            .unwrap()
            .decision(),
        SafetyDecision::NeedsWork
    );
    plan.limits[0].limit = exact(9.0);
    assert_eq!(
        assess_interaction_grids(runs.each_ref(), &plan)
            .unwrap()
            .decision(),
        SafetyDecision::Rejected
    );
    plan.limits[0].limit = exact(12.0);
    plan.power
        .sources
        .get_mut("накопитель")
        .unwrap()
        .paths
        .get_mut(&EnergyPath::Electrical)
        .unwrap()
        .violation = Some((exact(2.0), exact(1.0)));
    assert_eq!(
        assess_interaction_grids(runs.each_ref(), &plan)
            .unwrap()
            .outcome(),
        Outcome::Fail
    );
}

#[test]
fn loaded_patch_is_not_evidence_that_power_was_cut() {
    let (runs, mut plan) = setup(true, true, WINDOW.end);
    plan.power_loss = Some(PowerLossPlan {
        id: "отказ".into(),
        target: "power:вход".into(),
        at: Time(20 * MS),
        loaded_patch: PATCHES[0].into(),
        witness: None,
    });
    let r = assess_interaction_grids(runs.each_ref(), &plan).unwrap();
    assert_eq!(r.outcome(), Outcome::NotAnalyzed);
    assert!(r.grids().iter().all(|g| g.power_loss_applied_ns.is_none() && g.patches[PATCHES[0]].peak_normal_n > 0.0));
    assert!(
        r.blockers()
            .iter()
            .any(|s| s.contains("отключение питания под нагрузкой"))
    );
}

#[derive(Clone)]
struct Supplied {
    output: TruthId,
    channel: ChannelId,
    loads: [f64; 2],
}
impl Object for Supplied {
    fn update(
        &mut self,
        _: &Truth<'_>,
        cmd: &Command,
        _: &mut Rng,
        out: &mut Forces,
    ) -> Result<(), String> {
        if let Some((_, supply)) = cmd.targets.iter().find(|(id, _)| *id == self.channel) {
            out.publish(self.output, *supply);
        }
        for (i, load) in self.loads.into_iter().enumerate() {
            out.set(ChannelId(i as u32), load);
        }
        Ok(())
    }
}

struct PowerAssets;
impl ControlAssets for PowerAssets {
    fn read(&self, _: &str) -> Result<Vec<u8>, String> {
        Ok("version=1
[[block]]
kind='constant'
value=1.0
source='стенд'
[[bind]]
node=0
output='выход'
missing='omit'
source='стенд'"
            .as_bytes()
            .to_vec())
    }
    fn executable(&self, _: &str) -> Result<std::path::PathBuf, String> {
        Err("только встроенный контроллер".into())
    }
}

fn install_power(rt: &mut Runtime, positions: [f64; 2]) {
    let supply_channel = rt.channel("питание").unwrap();
    let output = rt.register_truth("отклик питания", "bool").unwrap();
    rt.register_object(
        "источник",
        rt.physics_rate(),
        &[supply_channel],
        Supplied {
            output,
            channel: supply_channel,
            loads: positions.map(|q| -1000.0 * (0.04 - q)),
        },
    )
    .unwrap();
    let config = ControlConfig::parse(
        r#"
version=1
[[controller]]
name='подача'
program='источник'
rate_hz=100
kind='builtin'
source='синтетический стенд'
[[output]]
name='выход'
controller='подача'
target='питание'
target_kind='power'
unit='bool'
max_age_ns=20000000
fallback=0.0
source='синтетический стенд'
[[fault]]
id='отказ'
target='power:питание'
at_ns=20000000
until_ns=40000000
kind='power_off'
source='синтетический стенд'
"#,
    )
    .unwrap();
    control::install(
        rt,
        &config,
        &PowerAssets,
        RunSpec {
            world: b"block",
            catalog: b"fixture",
            task: b"power",
            criteria: b"force",
            mode: "отказ",
            horizon: WINDOW.end,
            model_ports: &[ModelPort::Power {
                channel: supply_channel,
            }],
        },
    )
    .unwrap();
}

#[test]
fn power_delivery_witness_is_checked_on_each_grid() {
    let (_, mut plan) = setup(true, true, WINDOW.end);
    let live = [100, 200, 400].map(|hz| runtime(hz, Area::ByCompression, true, WINDOW.end, true));
    plan.grids = live.each_ref().map(|rt| rt.interaction_identity().unwrap());
    plan.power_loss = Some(PowerLossPlan {
        id: "отказ".into(),
        target: "power:питание".into(),
        at: Time(20 * MS),
        loaded_patch: PATCHES[0].into(),
        witness: Some(PowerWitness {
            signal: "отклик питания".into(),
            unit: "bool".into(),
            before: exact(1.0),
            after: exact(0.0),
        }),
    });
    let missing = assess_interaction_grids(live.each_ref(), &plan).unwrap();
    assert_eq!(missing.outcome(), Outcome::NotAnalyzed);
    assert!(
        missing
            .power()
            .iter()
            .any(|row| row.source.as_deref() == Some("привод стенда")
                && row.outcome == Outcome::NotAnalyzed)
    );
    let mut bounded = power(Some(0.1), Some(0.2));
    let source = bounded.sources.remove("накопитель").unwrap();
    plan.power.required_sources.insert("привод стенда".into());
    plan.power.sources.insert("привод стенда".into(), source);
    let result = assess_interaction_grids(live.each_ref(), &plan).unwrap();
    assert_eq!(result.outcome(), Outcome::NotAnalyzed);
    assert!(
        result
            .power()
            .iter()
            .all(|row| row.outcome == Outcome::Pass)
    );
    assert!(
        result
            .blockers()
            .iter()
            .any(|row| row.contains("приводного механизма"))
    );
    assert!(
        result
            .grids()
            .iter()
            .all(|grid| grid.power_loss_applied_ns == Some(20 * MS))
    );
    let witness = plan.power_loss.as_mut().unwrap().witness.as_mut().unwrap();
    witness.after = exact(0.5);
    let result = assess_interaction_grids(live.each_ref(), &plan).unwrap();
    assert_eq!(result.outcome(), Outcome::NotAnalyzed);
    assert!(
        result
            .grids()
            .iter()
            .all(|grid| grid.power_loss_applied_ns.is_none())
    );
}

#[test]
fn reachable_sources_can_exceed_the_budget_together() {
    // Два независимых запаса по 0,6 Дж дают 1,2 Дж против границы стенда 1 Дж.
    let mut inventory = power(Some(0.6), Some(0.6));
    inventory.required_sources.insert("второй источник".into());
    inventory.sources.insert(
        "второй источник".into(),
        inventory.sources["накопитель"].clone(),
    );
    let findings = assess_power_inventory(&inventory);
    assert!(
        findings
            .iter()
            .filter(|row| row.source.is_some())
            .all(|row| row.outcome == Outcome::Pass)
    );
    assert_eq!(
        findings.iter().filter(|row| row.confirmed_hazard).count(),
        3
    );
    assert_eq!(
        safety_decision(findings.into_iter().map(|row| row.outcome), true),
        SafetyDecision::BenchOnly
    );
}
