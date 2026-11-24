use std::{collections::BTreeMap, fs, path::PathBuf};

use sim_process::work::{HazardKind, ProcessKind, WorkSubject};
use sim_runtime::Runtime;
use sim_signal::Command;
use sim_task::{
    Fault, FaultKind, Fingerprint, Limit, Outcome, Signal, Source, SourceKind,
    campaign::{
        Binding, Campaign, CampaignPlan, FaultSupport, Grid, ModelParameter, RequiredPath,
        ResolvedInputs, ResolvedReference, SampledInputs, Target, Window, WorldInputs,
    },
    metrics::Installation,
    verdict::{
        CampaignEvidence, Check, Component, ComponentKind, Coverage, Domain, HazardClass,
        Installed, MeasurementSample, REGISTRY, Readiness, ResolvedInventory, RunSelector,
        SimulationVerdict, Verification, assess_campaign, derive_requirements,
    },
};
use sim_world::SectionAxis;

// Стержень имеет b=0,02 м, h=0,04 м, длину 1 м.
// На каретку массой 2 кг действует 2 Н ровно 4 шага по 1 мс.
const SOURCE: &str = "{ kind = \"measured\", reference = \"протокол синтетического стенда\" }";
const ROW: Window = Window {
    start_ns: 0,
    end_ns: 4_000_000,
    max_gap_ns: 1_000_000,
};

fn source(kind: SourceKind, reference: &str) -> Source {
    Source {
        kind,
        reference: reference.into(),
    }
}

fn measured() -> Source {
    source(SourceKind::Measured, "протокол синтетического стенда")
}
fn build() -> Fingerprint {
    Fingerprint::of("версия каретки".as_bytes())
}
fn position() -> Signal {
    Signal {
        name: "qpos[0]".into(),
        unit: "m".into(),
    }
}
fn limit(signal: Signal, hi: f64) -> Limit {
    Limit::Range {
        signal,
        min: None,
        max: Some(hi),
        source: measured(),
    }
}

struct Fixture(PathBuf);

impl Drop for Fixture {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

impl Fixture {
    fn new(id: &str) -> Self {
        let this = Self(
            std::env::temp_dir().join(format!("campaign-verdict-{}-{id}", std::process::id())),
        );
        fs::create_dir(&this.0).unwrap();
        this.write("catalog.toml", "version = 1");
        this.write(
            "world.toml",
            r#"
format = 3
name = "Каретка"
seed = 1
physics_hz = 1000
gravity_m_s2 = [0.0, 0.0, 0.0]
[environment]
density_kg_m3 = 0.0
viscosity_pa_s = 0.0
wind_m_s = [0.0, 0.0, 0.0]
[[devices]]
name = "каретка"
file = "body.toml"
"#,
        );
        this.write(
            "body.toml",
            r#"
format = 3
name = "Стержень"
[[links]]
name = "стержень"
[links.joint]
kind = "slide"
axis = [1.0, 0.0, 0.0]
damping = 0.0
friction_loss = 0.0
armature = 0.0
efforts = [{ name = "сила", dof = 0 }]
[links.mass]
kind = "explicit"
mass_kg = 2.0
com_m = [0.0, 0.0, 0.0]
inertia_kg_m2 = [0.01, 0.02, 0.025, 0.0, 0.0, 0.0]
provenance = { measured = "синтетическая масса с инерцией стержня" }
[[links.collision]]
shape = { kind = "box", size_m = [1.0, 0.02, 0.04] }
# Контакт отключён у единственной геометрии, трение равно нулю.
contact_mask = [0, 0]
friction = [0.0, 0.0, 0.0]
"#,
        );
        this
    }

    fn write(&self, file: &str, text: &str) {
        fs::write(self.0.join(file), text).unwrap();
    }

    fn plan(&self, part: &str, threshold: f64, fault: bool) -> CampaignPlan {
        self.plan_with_sampling(part, threshold, fault, "")
    }

    fn plan_with_sampling(
        &self,
        part: &str,
        threshold: f64,
        fault: bool,
        sampling: &str,
    ) -> CampaignPlan {
        let fault_description = if fault {
            format!(
                r#"faults = [{{ fault = {{ node = "провод", kind = "wire_open" }}, at_ns = 2000000, max_lag_ns = 0, window = "whole", source = {SOURCE} }}]"#
            )
        } else {
            String::new()
        };
        self.write("campaign.toml", &format!(r#"
version = 1
world = "world.toml"
catalog = "catalog.toml"
seed = 7
{sampling}
[stations.stand]
experiment = {{ kind = "scenario" }}
[stations.stand.modes.motion]
horizon_ns = 4000000
source = {SOURCE}
steps = [{{ id = "push", target = {{ kind = "effort", name = "каретка/сила" }}, at_ns = 0, value = 2.0, unit = "N", source = {SOURCE} }}]
criteria = [{{ id = "ход", window = "whole", limit = {{ kind = "range", signal = {{ name = "qpos[0]", unit = "m" }}, max = {threshold:?}, source = {SOURCE} }} }}]
{fault_description}
{part}
[stations.stand.modes.motion.windows.whole]
start_ns = 0
end_ns = 4000000
max_gap_ns = 1000000
"#));
        let campaign = Campaign::load(self.0.join("campaign.toml")).unwrap();
        let faults = fault
            .then(|| {
                (
                    Fault {
                        node: "провод".into(),
                        kind: FaultKind::WireOpen,
                    },
                    FaultSupport::Available {
                        channel: "провод/разрыв".into(),
                        resource: "линия".into(),
                    },
                )
            })
            .into_iter()
            .collect();
        campaign
            .plan(&ResolvedInputs {
                world: WorldInputs::load(campaign.world_path()).unwrap(),
                references: campaign
                    .references()
                    .map(|r| {
                        (
                            r.path.clone(),
                            ResolvedReference {
                                kind: r.kind,
                                file_fingerprint: r.file_fingerprint,
                                graph_fingerprint: Fingerprint::of(
                                    "разрешённый граф стенда".as_bytes(),
                                ),
                            },
                        )
                    })
                    .collect(),
                paths: vec![RequiredPath {
                    station: "stand".into(),
                    mode: "motion".into(),
                    path: "сила".into(),
                    source: measured(),
                    grids: vec![Grid {
                        id: "1000".into(),
                        hz: 1000,
                        source: measured(),
                    }],
                    faults,
                    combinations: vec![],
                    unavailable: None,
                }],
                targets: BTreeMap::from([(
                    Target::Effort {
                        name: "каретка/сила".into(),
                    },
                    Binding {
                        unit: "N".into(),
                        resource: "сила".into(),
                        unavailable: None,
                    },
                )]),
                signals: [position()].into(),
                model_parameters: BTreeMap::from([(
                    sim_task::sampling::Parameter::ChannelDelay {
                        channel: "observation:координата".into(),
                    },
                    ModelParameter {
                        unit: "s".into(),
                        // Два положения настройки шины стенда: 0 и 1,5 мс.
                        nominal: 0.0,
                        min: 0.0,
                        max: 0.0015,
                    },
                )]),
                initial_velocity: true,
            })
            .unwrap()
    }
}

fn inventory() -> ResolvedInventory {
    let coverage = [
        Domain::Parts,
        Domain::Geometry,
        Domain::Attachments,
        Domain::Catalog,
        Domain::Power,
        Domain::Control,
        Domain::Environment,
        Domain::Participant,
        Domain::Measurements,
    ]
    .map(|domain| (domain, Coverage::Resolved(measured())))
    .into();
    ResolvedInventory {
        build: Some(build()),
        participant: Some(sim_task::interaction::Reach::OffParticipant),
        coverage,
        checks: vec![
            Check {
                class: HazardClass::StaticStrength,
                object: "каретка/стержень".into(),
                selector: RunSelector::default(),
                window: ROW,
                applicability: measured(),
                scope: "однородный прямоугольный стержень, изгиб в одной плоскости".into(),
                inputs: vec![],
                verification: Verification::Section {
                    geometry: 0,
                    axis: SectionAxis::X,
                    // 20 Н на плече 1 м и предельные 5 МПа из постановки стенда.
                    moment_nm: [20.0; 2],
                    allowable_pa: [5e6; 2],
                    load: measured(),
                    limit: measured(),
                },
            },
            Check {
                class: HazardClass::TravelJam,
                object: "каретка/стержень".into(),
                selector: RunSelector::default(),
                window: ROW,
                applicability: measured(),
                scope: "граница измерительного хода стенда".into(),
                inputs: vec![],
                // Измерительный зазор 10 мм из нейтральной постановки.
                verification: Verification::Truth(limit(position(), 0.01)),
            },
        ],
        ..ResolvedInventory::default()
    }
}

fn activate(inv: &mut ResolvedInventory, kind: ComponentKind, object: &str) {
    inv.components.push(Component {
        object: object.into(),
        kind,
        source: measured(),
    });
}

fn receipt(sample: &SampledInputs) -> Installed {
    Installed {
        build: build(),
        settings: sample.settings.clone(),
        operations: vec![],
        installation: Installation {
            fingerprint: sample.fingerprint,
            run_seed: sample.sample.seeds.run,
            physics_hz: sample.world.world.physics_hz,
            values: sample
                .values()
                .iter()
                .map(|a| (a.id.clone(), a.value))
                .collect(),
        },
    }
}

fn drive(rt: &mut Runtime, force: f64, record: bool) -> Result<(), String> {
    if record {
        rt.record_joint_state("кинематика")?;
    }
    rt.act(&Command {
        targets: vec![(rt.channel("каретка/сила").unwrap(), force)],
    })
}

fn execute(rt: &mut Runtime, sample: &SampledInputs) -> Result<Installed, String> {
    drive(rt, 2.0, true)?;
    Ok(receipt(sample))
}

fn discrete_sampling() -> String {
    // Два положения упора испытательной каретки: исходное и с проставкой 1 мкм.
    format!(
        r#"
[sampling]
discrete = {SOURCE}
axes = [{{ id = "координата", parameter = {{ kind = "joint_position", joint = "каретка/стержень" }}, scope = "run", unit = "m", nominal = 0.0, min = 0.0, max = 0.000001, source = {SOURCE} }}]
boundaries = [{{ id = "проставка", values = {{ "координата" = 0.000001 }}, source = {SOURCE} }}]
"#
    )
}

#[test]
fn отсутствующий_пустой_и_номинальный_разброс_закрывают_допуск() {
    let fixture = Fixture::new("sampling-required");
    let discrete = discrete_sampling();
    let same_state = discrete.replace(
        "values = { \"координата\" = 0.000001 }",
        "values = { \"координата\" = 0.0 }",
    );
    for (sampling, reason) in [
        ("", "отсутствует обязательный раздел"),
        ("[sampling]", "разброс пуст"),
        (&same_state, "меньше двух различимых выборок"),
    ] {
        let plan = fixture.plan_with_sampling("", 0.01, false, sampling);
        let req = derive_requirements(&inventory(), &plan).unwrap();
        let mut evidence = CampaignEvidence::new(&req);
        for row in plan.rows() {
            evidence.execute(&req, &plan, &row.key, execute).unwrap();
        }
        let report = assess_campaign(&req, &evidence).unwrap();
        assert_eq!(report.simulation, SimulationVerdict::NotProven);
        assert_eq!(report.readiness, Readiness::NeedsWork);
        assert!(
            report.findings.iter().any(|finding| {
                finding.key.object == "разброс"
                    && finding.outcome == Outcome::NotAnalyzed
                    && finding.reason.contains(reason)
            }),
            "{:#?}",
            report.findings
        );
    }
}

#[test]
fn квитанция_без_установленной_скорости_или_задержки_не_подтверждает_опыт() {
    let fixture = Fixture::new("velocity-receipt");
    // Стержень стартует из покоя или с 0,2 м/с вдоль поступательной оси.
    let sampling = format!(
        r#"
[sampling]
discrete = {SOURCE}
axes = [
    {{ id = "скорость", parameter = {{ kind = "joint_velocity", joint = "каретка/стержень", dof = 0 }}, scope = "run", unit = "m/s", nominal = 0.0, min = 0.0, max = 0.2, source = {SOURCE} }},
    {{ id = "доставка", parameter = {{ kind = "channel_delay", channel = "observation:координата" }}, scope = "run", unit = "s", nominal = 0.0, min = 0.0, max = 0.0015, source = {SOURCE} }},
]
boundaries = [{{ id = "движение", values = {{ "скорость" = 0.2, "доставка" = 0.0015 }}, source = {SOURCE} }}]
"#
    );
    let plan = fixture.plan_with_sampling("", 0.01, false, &sampling);
    let req = derive_requirements(&inventory(), &plan).unwrap();
    #[derive(Clone)]
    struct Reader(sim_signal::SignalId);
    impl sim_runtime::Sensor for Reader {
        fn sample(
            &mut self,
            truth: &sim_runtime::Truth<'_>,
            _: &mut sim_signal::Rng,
            out: &mut sim_runtime::ObservationOut,
        ) -> Result<(), String> {
            out.write(self.0, truth.qpos()[0], true);
            Ok(())
        }
    }
    for (velocity, delay) in [(false, true), (true, false), (true, true)] {
        let mut evidence = CampaignEvidence::new(&req);
        for row in plan.rows() {
            evidence
                .execute(&req, &plan, &row.key, |rt, sampled| {
                    let id = rt.register_observation("координата", "m")?;
                    rt.register_sensor("датчик", rt.physics_rate(), Reader(id))?;
                    if velocity {
                        rt.set_initial_velocities(&sampled.settings.initial_velocity)?;
                    }
                    if delay {
                        rt.set_channel_delays(&sampled.settings.channel_delay_ns)?;
                    }
                    execute(rt, sampled)
                })
                .unwrap();
        }
        let report = assess_campaign(&req, &evidence).unwrap();
        if velocity && delay {
            assert_eq!(report.simulation, SimulationVerdict::PassedSimulation);
            let moved = report
                .findings
                .iter()
                .find(|finding| {
                    finding.key.class == HazardClass::TaskCriterion
                        && finding.key.run.as_ref() == Some(&plan.rows()[1].key)
                })
                .unwrap();
            // За 4 мс v₀ добавляет 800 мкм к 10 мкм от ускорения 1 м/с².
            assert!((moved.measurement.unwrap()[1] - 0.000810).abs() < 1e-14);
        } else {
            assert_eq!(report.simulation, SimulationVerdict::NotProven);
            assert!(
                report.findings.iter().any(|finding| {
                    finding.outcome == Outcome::NotAnalyzed
                        && finding
                            .coverage_gaps
                            .iter()
                            .chain(&finding.missing_data)
                            .any(|reason| reason.contains("фактические установки"))
                }),
                "{:#?}",
                report.findings
            );
        }
    }
}

#[test]
fn live_section_and_motion_match_independent_calculations_and_readiness() {
    let fixture = Fixture::new("ready");
    let plan = fixture.plan_with_sampling("", 0.01, false, &discrete_sampling());
    let key = &plan.rows()[0].key;
    let mut inv = inventory();
    let requirements = derive_requirements(&inv, &plan).unwrap();
    let empty = CampaignEvidence::new(&requirements);
    assert_eq!(
        assess_campaign(&requirements, &empty).unwrap().readiness,
        Readiness::NeedsWork
    );
    let mut evidence = empty;
    for row in plan.rows() {
        evidence
            .execute(&requirements, &plan, &row.key, execute)
            .unwrap();
    }
    let report = assess_campaign(&requirements, &evidence).unwrap();
    assert_eq!(
        report.simulation,
        SimulationVerdict::PassedSimulation,
        "{:#?}",
        report.findings
    );
    assert_eq!(report.readiness, Readiness::BenchOnly);

    let section = report
        .findings
        .iter()
        .find(|f| f.key.class == HazardClass::StaticStrength)
        .unwrap();
    let [lo, hi] = section.measurement.unwrap();
    // Независимая формула σ = 6FL/(bh²), запас = 5 МПа − σ.
    let expected = 6.0 * 20.0 * 1.0 / (0.02 * 0.04_f64.powi(2));
    assert_eq!(lo, hi);
    assert!((lo - expected).abs() < 1e-8);
    assert!((section.limit.as_ref().unwrap().upper.unwrap()[1] - lo - 1_250_000.0).abs() < 1e-8);
    let travel = report
        .findings
        .iter()
        .find(|f| f.key.class == HazardClass::TaskCriterion)
        .unwrap();
    // Сумма скоростей полунеявного Euler при F/m = 1 м/с².
    assert!((travel.measurement.unwrap()[1] - 10e-6).abs() < 1e-14);
    assert!(travel.journal.is_some());

    activate(&mut inv, ComponentKind::CriticalMeasurement, "ход");
    inv.checks.push(Check {
        class: HazardClass::Measurement,
        object: "ход".into(),
        window: ROW,
        selector: RunSelector::default(),
        applicability: measured(),
        scope: "калиброванная линейка".into(),
        inputs: vec![],
        verification: Verification::Measurement(limit(position(), 0.01)),
    });
    let req = derive_requirements(&inv, &plan).unwrap();
    let mut evidence = CampaignEvidence::new(&req);
    for row in plan.rows() {
        evidence.execute(&req, &plan, &row.key, execute).unwrap();
    }
    assert_eq!(
        assess_campaign(&req, &evidence).unwrap().readiness,
        Readiness::BenchOnly
    );
    let samples: Vec<_> = (0..=4)
        .map(|tick| MeasurementSample {
            at_ns: tick * ROW.max_gap_ns,
            value: (tick * (tick + 1) / 2) as f64 * 1e-6,
        })
        .collect();
    assert!(
        evidence
            .record_measurement(
                &req,
                key,
                Fingerprint::of(b"other"),
                position(),
                measured(),
                Some(0.0), // Точный ряд из рекурсии Euler.
                &samples
            )
            .is_err()
    );
    for row in plan.rows() {
        let offset = plan.materialize(&row.key).unwrap().values()[0].value;
        let shifted: Vec<_> = samples
            .iter()
            .map(|s| MeasurementSample {
                at_ns: s.at_ns,
                value: s.value + offset,
            })
            .collect();
        evidence
            .record_measurement(&req, &row.key, build(), position(), measured(), Some(0.0), &shifted)
            .unwrap();
    }

    assert_eq!(
        assess_campaign(&req, &evidence).unwrap().readiness,
        Readiness::Passed
    );
}

#[test]
fn unmodeled_subject_unknown_extension_and_nested_estimate_stay_open() {
    let fixture = Fixture::new("open");
    let plan = fixture.plan(&format!(r#"hazards = [{{ class = "новый_класс", object = "насадка", window = "whole", source = {SOURCE} }}]"#), 0.01, false);
    let key = &plan.rows()[0].key;
    let mut inv = inventory();
    inv.work_subjects.push(WorkSubject {
        name: "насадка".into(),
        properties: [ProcessKind::HeatTransfer].into(),
    });
    inv.checks[0].applicability = source(SourceKind::Derived, "расчёт материала");
    inv.derivations.insert(
        "расчёт материала".into(),
        vec![source(SourceKind::Derived, "нижний уровень")],
    );
    inv.derivations.insert(
        "нижний уровень".into(),
        vec![source(SourceKind::Estimate, "свойства неизвестны")],
    );
    let req = derive_requirements(&inv, &plan).unwrap();
    let mut evidence = CampaignEvidence::new(&req);
    evidence.execute(&req, &plan, key, execute).unwrap();
    let report = assess_campaign(&req, &evidence).unwrap();
    assert_eq!(report.simulation, SimulationVerdict::NotProven);
    assert_eq!(report.readiness, Readiness::NeedsWork);
    assert!(
        report
            .findings
            .iter()
            .any(|f| f.code == "новый_класс" && f.key.extension.is_some())
    );
    assert!(
        report
            .findings
            .iter()
            .any(|f| f.key.class == HazardClass::Burns
                && f.key.object == "насадка"
                && f.outcome == Outcome::NotAnalyzed)
    );
    let section = report
        .findings
        .iter()
        .find(|f| f.key.class == HazardClass::StaticStrength)
        .unwrap();
    assert!(section.measurement.is_some());
    assert_eq!(section.outcome, Outcome::NotAnalyzed);
    assert!(
        section
            .measurement_sources
            .iter()
            .any(|s| s.contains("свойства неизвестны"))
    );
    assert!(
        !report
            .inapplicable
            .iter()
            .any(|r| r.class == HazardClass::Burns)
    );
}

#[test]
fn короткое_окно_не_закрывает_опасность_на_остатке_режима() {
    let fixture = Fixture::new("partial-window");
    let plan = fixture.plan("", 0.01, false);
    let mut inv = inventory();
    inv.checks
        .retain(|check| check.class != HazardClass::TravelJam);
    inv.checks.push(Check {
        class: HazardClass::TravelJam,
        object: "каретка/стержень".into(),
        selector: RunSelector::default(),
        window: Window {
            end_ns: 2_000_000,
            ..ROW
        },
        applicability: measured(),
        scope: "первые два такта".into(),
        inputs: vec![],
        // При ускорении 1 м/с² на втором такте 3 мкм, на четвёртом 10 мкм.
        verification: Verification::Truth(limit(position(), 8e-6)),
    });
    let assess = |inv: &ResolvedInventory| {
        let req = derive_requirements(inv, &plan).unwrap();
        let mut evidence = CampaignEvidence::new(&req);
        evidence
            .execute(&req, &plan, &plan.rows()[0].key, execute)
            .unwrap();
        assess_campaign(&req, &evidence).unwrap()
    };
    let short = assess(&inv);
    assert_eq!(short.simulation, SimulationVerdict::NotProven);
    assert!(
        short
            .findings
            .iter()
            .any(|f| f.key.class == HazardClass::TravelJam && f.outcome == Outcome::NotAnalyzed)
    );

    let mut rest = inv.checks.last().unwrap().clone();
    rest.window = Window {
        start_ns: 2_000_000,
        ..ROW
    };
    inv.checks.push(rest);
    let full = assess(&inv);
    assert_eq!(full.simulation, SimulationVerdict::Rejected);
    assert!(
        full.findings
            .iter()
            .any(|f| f.key.class == HazardClass::TravelJam
                && f.outcome == Outcome::Fail
                && f.event_ns == Some(4_000_000))
    );

    inv.checks.pop();
    inv.checks.last_mut().unwrap().class = HazardClass::Fatigue;
    for removed in [false, true] {
        if removed {
            inv.checks.pop();
        }
        let result = assess(&inv);
        assert_eq!(result.simulation, SimulationVerdict::NotProven);
        assert!(result.findings.iter().any(|f| f.key.class == HazardClass::TravelJam
            && f.outcome == Outcome::NotAnalyzed));
    }
}

#[test]
fn one_violation_survives_pass_and_open_neighbor() {
    let fixture = Fixture::new("fail");
    // Предел соответствует ходу после второго шага. На третьем шаге ход 6 мкм.
    let plan = fixture.plan("", 3.1e-6, false);
    let mut inv = inventory();
    inv.participant = Some(sim_task::interaction::Reach::Reachable);
    activate(&mut inv, ComponentKind::ParticipantContact, "пятно");
    activate(&mut inv, ComponentKind::EnergyPath, "пружина");
    let req = derive_requirements(&inv, &plan).unwrap();
    let mut evidence = CampaignEvidence::new(&req);
    evidence
        .execute(&req, &plan, &plan.rows()[0].key, execute)
        .unwrap();
    let report = assess_campaign(&req, &evidence).unwrap();
    assert_eq!(report.simulation, SimulationVerdict::Rejected);
    assert_eq!(report.readiness, Readiness::Rejected);
    let fail = report
        .findings
        .iter()
        .find(|f| f.key.class == HazardClass::TaskCriterion)
        .unwrap();
    assert_eq!(fail.outcome, Outcome::Fail);
    assert_eq!(fail.event_ns, Some(3_000_000));
    assert!(!fail.confirmed_hazard);
    for class in [
        HazardClass::TissueLoad,
        HazardClass::ContactStructure,
        HazardClass::StoredEnergy,
    ] {
        assert!(
            report
                .findings
                .iter()
                .any(|f| f.key.class == class && f.outcome == Outcome::NotAnalyzed)
        );
    }
    assert!(
        report
            .findings
            .iter()
            .any(|f| f.key.class == HazardClass::StaticStrength && f.outcome == Outcome::Pass)
    );
}

#[test]
fn wrong_stamps_and_missing_signal_never_grant_credit() {
    let fixture = Fixture::new("binding");
    let plan = fixture.plan("", 0.01, false);
    let key = &plan.rows()[0].key;
    let req = derive_requirements(&inventory(), &plan).unwrap();
    let mut evidence = CampaignEvidence::new(&req);
    evidence
        .execute(&req, &plan, key, |rt, sample| {
            let mut data = execute(rt, sample)?;
            data.installation.fingerprint = Fingerprint::of(b"other sample");
            Ok(data)
        })
        .unwrap();
    let report = assess_campaign(&req, &evidence).unwrap();
    assert!(
        report
            .findings
            .iter()
            .all(|f| f.outcome == Outcome::NotAnalyzed)
    );
    let mut foreign = inventory();
    foreign.build = Some(Fingerprint::of(b"other build"));
    assert_eq!(
        assess_campaign(&derive_requirements(&foreign, &plan).unwrap(), &evidence)
            .unwrap()
            .simulation,
        SimulationVerdict::NotProven
    );

    let mut evidence = CampaignEvidence::new(&req);
    evidence
        .execute(&req, &plan, key, |rt, sample| {
            drive(rt, 2.0, false)?;
            Ok(receipt(sample))
        })
        .unwrap();
    let report = assess_campaign(&req, &evidence).unwrap();
    assert!(
        report
            .findings
            .iter()
            .any(|f| f.key.class == HazardClass::TaskCriterion
                && f.measurement.is_none()
                && f.outcome == Outcome::NotAnalyzed)
    );
}

#[derive(Clone)]
struct Stop;
impl sim_runtime::Evaluator for Stop {
    fn evaluate(
        &mut self,
        truth: &sim_runtime::Truth<'_>,
        _: &mut sim_signal::Rng,
        _: &mut sim_runtime::TruthOut,
    ) -> Result<(), String> {
        // Стенд останавливает прогон через два шага, раньше горизонта в четыре.
        if truth.time().0 >= 2 * ROW.max_gap_ns {
            Err("остановка контрольного прогона".into())
        } else {
            Ok(())
        }
    }
}

#[test]
fn truncated_repeated_and_uninjected_rows_preserve_reasons() {
    let fixture = Fixture::new("coverage");
    let plan = fixture.plan("", 0.01, true);
    let req = derive_requirements(&inventory(), &plan).unwrap();
    let normal = &plan.rows()[0].key;
    let fault = &plan.rows()[1].key;
    let mut evidence = CampaignEvidence::new(&req);
    let mut repeat = false;
    evidence
        .execute(&req, &plan, normal, |rt, sample| {
            rt.register_evaluator("досрочная остановка", rt.physics_rate(), Stop)?;
            drive(rt, if repeat { 1.0 } else { 2.0 }, true)?;
            repeat = true;
            Ok(receipt(sample))
        })
        .unwrap();
    evidence.execute(&req, &plan, fault, execute).unwrap();
    let report = assess_campaign(&req, &evidence).unwrap();
    assert_eq!(report.simulation, SimulationVerdict::NotProven);
    let gaps = |key, text: &str| {
        report.findings.iter().any(|f| {
            f.key.run.as_ref() == Some(key)
                && f.coverage_gaps.iter().any(|reason| reason.contains(text))
        })
    };
    assert!(gaps(normal, "горизонт"));
    assert!(gaps(normal, "повтор"));
    assert!(gaps(fault, "отказов"));
    assert!(
        report
            .findings
            .iter()
            .any(|f| f.key.class == HazardClass::WireFault && f.outcome == Outcome::NotAnalyzed)
    );
}

#[test]
fn nonfinite_limits_and_replacement_runtime_are_rejected() {
    let fixture = Fixture::new("invalid");
    let plan = fixture.plan("", 0.01, false);
    let mut inv = inventory();
    for invalid in [f64::NAN, f64::INFINITY] {
        let Verification::Section {
            ref mut moment_nm, ..
        } = inv.checks[0].verification
        else {
            unreachable!()
        };
        moment_nm[1] = invalid;
        assert!(derive_requirements(&inv, &plan).is_err());
    }
    let req = derive_requirements(&inventory(), &plan).unwrap();
    let mut evidence = CampaignEvidence::new(&req);
    assert!(
        evidence
            .execute(&req, &plan, &plan.rows()[0].key, |rt, sample| {
                *rt = Runtime::load(sample.world.compile().map_err(|e| e.to_string())?)?;
                execute(rt, sample)
            })
            .is_err()
    );
}

#[test]
fn registry_has_distinct_methods_and_all_process_classes() {
    let classes = [
        HazardKind::Burns,
        HazardKind::Impact,
        HazardKind::Pinch,
        HazardKind::ThreadFailure,
        HazardKind::ElectricShock,
        HazardKind::Overcurrent,
        HazardKind::StoredEnergy,
        HazardKind::SharpEdge,
        HazardKind::Ingress,
    ];
    let keys: std::collections::BTreeSet<_> = classes
        .into_iter()
        .map(|kind| HazardClass::from(kind).spec().id)
        .collect();
    assert_eq!(keys.len(), classes.len());
    let ids: std::collections::BTreeSet<_> = REGISTRY.iter().map(|r| r.id).collect();
    assert_eq!(ids.len(), REGISTRY.len());
    assert!(
        REGISTRY
            .iter()
            .all(|r| !r.independent_check.is_empty() && !r.limit_origin.is_empty())
    );
}

#[test]
fn process_adapter_checks_convergence_profile_and_report_integrity() {
    use sim_process::{Interval, SourcedRange};
    use sim_signal::{Rate, Time};
    use sim_task::process::{
        ConvergencePlan, Criterion, ErrorBounds, Series, Statistic, TrialVariant, VariantKind,
        assess_processes,
    };
    use sim_world::Provenance;

    let fixture = Fixture::new("process");
    let plan = fixture.plan("", 0.01, false);
    let key = &plan.rows()[0].key;
    let sampled = plan.materialize(key).unwrap();
    let datum = |value, source: &str| SourcedRange {
        min: value,
        nominal: value,
        max: value,
        provenance: Provenance::Measured(source.into()),
    };
    let convergence = |tolerance| ConvergencePlan {
        base_hz: 1000,
        window: Interval {
            start: Time(ROW.start_ns),
            end: Time(ROW.end_ns),
        },
        window_source: "четыре миллисекунды наблюдения линейного перемещения".into(),
        variants: vec![TrialVariant {
            name: "основной".into(),
            kind: VariantKind::Baseline,
            materials: BTreeMap::new(),
        }],
        material_ranges: BTreeMap::new(),
        criteria: vec![Criterion {
            name: "ход каретки".into(),
            unit: "m".into(),
            series: Series::Position(0),
            statistic: Statistic::Maximum,
            tolerance: datum(tolerance, "предел различия сеток в синтетическом опыте"),
            remainder: ErrorBounds {
                // Остаток линейной квадратичной траектории после 4/8/16 шагов
                // входит в сумму межсеточных различий. Дополнительных членов здесь нет.
                discretization: datum(0.0, "сумма разностей покрывает остаток Euler"),
                solver: datum(0.0, "свободное поступательное движение"),
                materials: datum(0.0, "в идеальном опыте нет контакта и трения"),
            },
            upper: Some(datum(0.01, "граница держателя синтетической каретки")),
            lower: None,
        }],
        events: vec![],
        states: vec![],
        creep: vec![],
    };
    let install = |rt: &mut Runtime| {
        rt.set_process_inventory(&[], &[])?;
        drive(rt, 2.0, true)
    };
    let run_series = |criteria: &ConvergencePlan| {
        assess_processes(criteria, |rate, _| {
            let mut compiled = sampled.world.clone();
            compiled.world.physics_hz = rate.as_hz() as u64;
            compiled.contact.physics = Some(Rate::hz(compiled.world.physics_hz).unwrap());
            let mut rt = Runtime::load(compiled.compile().map_err(|e| e.to_string())?)?;
            install(&mut rt)?;
            Ok(rt)
        })
        .unwrap()
    };
    // x_h(4 мс) = (T²+Th)/2. Для h=1/0,5/0,25 мс значения 10/9/8,5 мкм.
    // Допуск 2 мкм покрывает обе разности, 0,25 мкм не покрывает ни одну.
    for (tolerance, expected) in [(2e-6, Outcome::Pass), (0.25e-6, Outcome::NotAnalyzed)] {
        let criteria = convergence(tolerance);
        let mut inv = inventory();
        activate(&mut inv, ComponentKind::Retention, "держатель");
        inv.checks.push(Check {
            class: HazardClass::Retention,
            object: "держатель".into(),
            window: ROW,
            selector: RunSelector::default(),
            applicability: measured(),
            scope: "смещение линейной каретки".into(),
            inputs: vec![measured()],
            verification: Verification::Process {
                profile: criteria.fingerprint(),
                criterion: "ход каретки".into(),
            },
        });
        let req = derive_requirements(&inv, &plan).unwrap();
        let mut evidence = CampaignEvidence::new(&req);
        evidence
            .execute(&req, &plan, key, |rt, sample| {
                install(rt)?;
                Ok(receipt(sample))
            })
            .unwrap();
        let mut series = run_series(&criteria);
        for (observed, analytic) in series.metrics[0].values.iter().zip([10e-6, 9e-6, 8.5e-6]) {
            assert!((observed.unwrap()[1] - analytic).abs() < 1e-14);
        }
        let original = series.metrics[0].outcome;
        series.metrics[0].outcome = Outcome::Fail;
        assert!(evidence.record_process(key, &series).is_err());
        series.metrics[0].outcome = original;
        evidence.record_process(key, &series).unwrap();
        let report = assess_campaign(&req, &evidence).unwrap();
        let finding = report
            .findings
            .iter()
            .find(|f| f.key.class == HazardClass::Retention)
            .unwrap();
        assert_eq!(finding.outcome, expected, "{:#?}", report.findings);
        assert!(finding.journal.is_some());
        assert!(
            finding
                .limit_sources
                .iter()
                .any(|s| s.contains("держателя"))
        );
        if expected == Outcome::NotAnalyzed {
            assert!(finding.reason.contains("соседние сетки"));
            assert_eq!(report.simulation, SimulationVerdict::NotProven);
        }
    }
}

#[test]
fn executor_honors_timed_commands_and_rejects_foreign_expanded_world() {
    let fixture = Fixture::new("scheduler");
    let plan = fixture.plan_with_sampling("", 0.01, false, &discrete_sampling());
    let key = &plan.rows()[0].key;
    let mut inv = inventory();
    let expanded = plan.materialize(key).unwrap().world.compile().unwrap();
    inv.compiled_rows.insert(
        key.clone(),
        sim_task::verdict::compiled_world_fingerprint(&expanded),
    );
    let req = derive_requirements(&inv, &plan).unwrap();
    let mut evidence = CampaignEvidence::new(&req);
    let mut foreign = expanded.clone();
    foreign.seed ^= 1;
    assert!(
        evidence
            .execute_linked(&req, &plan, key, foreign, execute)
            .is_err()
    );
    evidence
        .execute_linked(&req, &plan, key, expanded, execute)
        .unwrap();
    for row in &plan.rows()[1..] {
        evidence.execute(&req, &plan, &row.key, execute).unwrap();
    }
    assert_eq!(
        assess_campaign(&req, &evidence).unwrap().simulation,
        SimulationVerdict::PassedSimulation
    );

    let req = derive_requirements(&inventory(), &plan).unwrap();
    let mut evidence = CampaignEvidence::new(&req);
    for row in plan.rows() {
        evidence
            .execute_timed(
                &req,
                &plan,
                &row.key,
                |rt, sample| {
                    rt.record_joint_state("кинематика")?;
                    Ok(receipt(sample))
                },
                |_, _| panic!("в постановке только усилия"),
            )
            .unwrap();
    }
    assert_eq!(
        assess_campaign(&req, &evidence).unwrap().simulation,
        SimulationVerdict::PassedSimulation
    );
}

#[test]
fn violation_in_divergent_repeat_keeps_its_own_witness() {
    let fixture = Fixture::new("repeat-violation");
    // Для 2 Н последний ход 10 мкм, а для 4 Н он удваивается.
    let plan = fixture.plan("", 15e-6, false);
    let req = derive_requirements(&inventory(), &plan).unwrap();
    let mut evidence = CampaignEvidence::new(&req);
    let mut force = 2.0;
    evidence
        .execute(&req, &plan, &plan.rows()[0].key, |rt, sample| {
            drive(rt, force, true)?;
            force *= 2.0;
            Ok(receipt(sample))
        })
        .unwrap();
    let report = assess_campaign(&req, &evidence).unwrap();
    assert_eq!(report.simulation, SimulationVerdict::Rejected);
    let findings: Vec<_> = report
        .findings
        .iter()
        .filter(|f| f.key.class == HazardClass::TaskCriterion)
        .collect();
    assert_eq!(findings.len(), 2);
    assert_eq!(findings[0].outcome, Outcome::NotAnalyzed);
    assert_eq!(findings[1].outcome, Outcome::Fail);
    assert_ne!(findings[0].journal, findings[1].journal);
    assert_eq!(findings[1].event_ns, Some(ROW.end_ns));
    assert!((findings[1].measurement.unwrap()[1] - 20e-6).abs() < 1e-14);
}
