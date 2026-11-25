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
        CampaignEvidence, Check, Component, ComponentKind, ContactDuration, Coverage, Domain, HazardClass,
        Installed, MeasurementSample, REGISTRY, Readiness, ResolvedInventory, RunSelector,
        SectionLoad, SimulationVerdict, Verification, assess_campaign, derive_requirements,
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

fn strength_bench_check(inv: &mut ResolvedInventory) {
    let mut bench = inv.checks[0].clone();
    bench.scope = "Протокол стендового замера напряжения".into();
    bench.verification = Verification::Measurement(limit(strength_signal(), 5e6));
    inv.checks.push(bench);
}

fn strength_signal() -> Signal {
    Signal { name: "напряжение изгиба".into(), unit: "Pa".into() }
}

fn strength_samples() -> Vec<MeasurementSample> {
    // Независимое ожидаемое напряжение 6·2 Н·м / (0,04 м·(0,02 м)²).
    (0..=4).map(|tick| MeasurementSample {
        at_ns: tick * ROW.max_gap_ns,
        value: 750_000.0,
    }).collect()
}

fn record_strength(
    evidence: &mut CampaignEvidence,
    req: &sim_task::verdict::RequirementSet,
    key: &sim_task::campaign::RunKey,
) {
    evidence.record_measurement(req, key, build(), strength_signal(), measured(), Some(0.0),
        &strength_samples()).unwrap();
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
        self.custom_plan(part, threshold, fault, sampling, r#"{ kind = "scenario" }"#, (2.0, ""))
    }

    fn custom_plan(
        &self,
        part: &str,
        threshold: f64,
        fault: bool,
        sampling: &str,
        experiment: &str,
        (effort, extra_steps): (f64, &str),
    ) -> CampaignPlan {
        let world = WorldInputs::load(self.0.join("world.toml")).unwrap();
        let hinge = world.devices["каретка"].device.links[0].joint.kind == sim_world::JointType::Hinge;
        let (effort_unit, position_unit) = if hinge { ("N*m", "rad") } else { ("N", "m") };
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
experiment = {experiment}
[stations.stand.modes.motion]
horizon_ns = 4000000
source = {SOURCE}
steps = [{{ id = "push", target = {{ kind = "effort", name = "каретка/сила" }}, at_ns = 0, value = {effort:?}, unit = "{effort_unit}", source = {SOURCE} }}{extra_steps}]
criteria = [{{ id = "ход", window = "whole", limit = {{ kind = "range", signal = {{ name = "qpos[0]", unit = "{position_unit}" }}, max = {threshold:?}, source = {SOURCE} }} }}]
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
                world,
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
                        unit: effort_unit.into(),
                        resource: "сила".into(),
                        unavailable: None,
                    },
                )]),
                signals: [Signal { unit: position_unit.into(), ..position() }].into(),
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
                    // Верхняя нагрузка штатной оснастки 2 Н·м, плечо 1 м и предел 5 МПа из постановки стенда.
                    moment_nm: [2.0; 2],
                    live_load: Some(SectionLoad { effort: "каретка/сила".into(), moment_per_effort: 1.0 }),
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

fn reaction_at(runtime: &Runtime, component: &str, at_ns: u64) -> f64 {
    let log = runtime.log();
    let name = format!("section_reaction/каретка/стержень/{component}");
    let id = log.header().truth.iter().position(|signal| signal.name == name).unwrap();
    log.entries().iter().find(|entry| entry.kind == sim_runtime::LogKind::Truth
        && entry.id as usize == id && entry.at.0 == at_ns && entry.valid).unwrap().value
}

fn discrete_sampling() -> String {
    // Два положения упора испытательной каретки: исходное и с проставкой 1 мкм.
    format!(
        r#"
[sampling]
discrete = {{ source = {SOURCE}, boundary_ids = ["проставка"] }}
axes = [{{ id = "координата", parameter = {{ kind = "joint_position", joint = "каретка/стержень" }}, scope = "run", unit = "m", nominal = 0.0, values = [0.0, 0.000001], source = {SOURCE} }}]
boundaries = [{{ id = "проставка", values = {{ "координата" = 0.000001 }}, source = {SOURCE} }}]
"#
    )
}

#[test]
fn дискретный_допуск_требует_границ_источника_и_исполнения() {
    let fixture = Fixture::new("discrete-edges");
    let sampling = discrete_sampling();
    let mut inv = inventory();
    strength_bench_check(&mut inv);
    let assess = |sampling: &str, inv: &ResolvedInventory, all_states: bool| {
        let plan = fixture.plan_with_sampling("", 0.02, false, sampling);
        assert_eq!(plan.rows().len(), 2);
        let req = derive_requirements(inv, &plan).unwrap();
        let mut evidence = CampaignEvidence::new(&req);
        for row in plan.rows() {
            if all_states || row.key.sample == sim_task::sampling::SampleKey::Nominal {
                evidence.execute(&req, &plan, &row.key, execute).unwrap();
                record_strength(&mut evidence, &req, &row.key);
            }
        }
        assess_campaign(&req, &evidence).unwrap()
    };
    assert_eq!(
        assess(&sampling, &inv, true).simulation,
        SimulationVerdict::PassedSimulation
    );
    let missing = assess(&sampling, &inv, false);
    assert_eq!(missing.simulation, SimulationVerdict::NotProven);
    assert!(
        missing
            .findings
            .iter()
            .any(|f| f.key.class == HazardClass::Execution
                && f.key.level == sim_task::verdict::Level::Simulation
                && f.outcome == Outcome::NotAnalyzed)
    );

    // Граница испытания 10 мм, а состояние с проставкой по-прежнему даёт 1 мкм.
    let wider = sampling.replace("values = [0.0, 0.000001]", "min = 0.0, max = 0.01").replacen(
        &format!("source = {SOURCE}"),
        r#"source = { kind = "datasheet", reference = "x" }"#,
        1,
    );
    let derived = sampling.replacen(
        &format!("source = {SOURCE}"),
        r#"source = { kind = "derived", reference = "матрица допуска" }"#,
        1,
    );
    let mut derived_inv = inv.clone();
    derived_inv
        .derivations
        .insert("матрица допуска".into(), vec![measured()]);
    assert_eq!(
        assess(&derived, &derived_inv, true).simulation,
        SimulationVerdict::PassedSimulation
    );
    derived_inv.derivations.insert(
        "матрица допуска".into(),
        vec![source(SourceKind::Estimate, "не измеренная проставка")],
    );
    for (sampling, inventory) in [(&wider, &inv), (&derived, &derived_inv)] {
        let report = assess(sampling, inventory, true);
        assert_eq!(report.simulation, SimulationVerdict::NotProven);
        assert_eq!(report.readiness, Readiness::NeedsWork);
        assert!(
            report
                .findings
                .iter()
                .any(|f| f.key.object == "допуск/координата"
                    && f.key.level == sim_task::verdict::Level::Simulation
                    && f.outcome == Outcome::NotAnalyzed)
        );
    }

    // Даже обе крайние точки 0 и 10 мм с источником из одного символа
    // не описывают непрерывное множество между ними.
    let endpoints = wider.replace("values = { \"координата\" = 0.000001 }",
        "values = { \"координата\" = 0.01 }");
    let mut loose = inv.clone();
    loose.checks[1].verification = Verification::Truth(limit(position(), 0.02));
    let report = assess(&endpoints, &loose, true);
    assert_eq!(report.simulation, SimulationVerdict::NotProven);
    assert!(report.findings.iter().any(|f| f.key.object == "допуск/координата"));
    assert_eq!(report.findings.iter().filter(|f| f.key.class == HazardClass::Execution).count(), 2);
    assert!(report.findings.iter().filter(|f| f.key.run.is_some()).all(|f| f.outcome == Outcome::Pass));
}

#[test]
fn неисследованный_диапазон_ограничивает_вердикт_симуляции() {
    let fixture = Fixture::new("unresolved-range");
    let plan = fixture.plan_with_sampling("", 0.01, false, &discrete_sampling());
    let mut inv = inventory();
    inv.unresolved_ranges
        .push("температурная огибающая не рассчитана".into());
    let req = derive_requirements(&inv, &plan).unwrap();
    let mut evidence = CampaignEvidence::new(&req);
    for row in plan.rows() {
        evidence.execute(&req, &plan, &row.key, execute).unwrap();
    }
    let report = assess_campaign(&req, &evidence).unwrap();
    assert_eq!(report.simulation, SimulationVerdict::NotProven);
    assert!(report.findings.iter().any(|f| f.key.object == "диапазон/0"
        && f.key.level == sim_task::verdict::Level::Simulation
        && f.outcome == Outcome::NotAnalyzed));
    assert!(
        report
            .findings
            .iter()
            .filter(|f| f.key.class == HazardClass::Execution)
            .all(|f| f.outcome == Outcome::Pass)
    );
}

#[test]
fn живые_сто_ньютонов_нельзя_уменьшить_заявленным_плечом() {
    let fixture = Fixture::new("section-arm");
    let plan = fixture.plan_with_sampling("", 0.01, false, &discrete_sampling());
    let mut inv = inventory();
    let Verification::Section {
        live_load: Some(live),
        ..
    } = &mut inv.checks[0].verification
    else {
        panic!("нет проверки стержня")
    };
    // Атакуемая запись заявляет плечо 1 мкм для метрового стержня.
    live.moment_per_effort = 1e-6;
    let req = derive_requirements(&inv, &plan).unwrap();
    let mut evidence = CampaignEvidence::new(&req);
    for row in plan.rows() {
        evidence
            .execute(&req, &plan, &row.key, |rt, sample| {
                drive(rt, 100.0, true)?;
                Ok(receipt(sample))
            })
            .unwrap();
    }
    let report = assess_campaign(&req, &evidence).unwrap();
    assert_eq!(report.simulation, SimulationVerdict::Rejected);
    let results: Vec<_> = report
        .findings
        .iter()
        .filter(|f| f.key.class == HazardClass::StaticStrength)
        .collect();
    assert_eq!(results.len(), 2);
    // Независимый расчёт прямоугольника 20×40 мм, M = 100 Н · 1 м.
    let stress = 6.0 * 100.0 / (0.04 * 0.02_f64.powi(2));
    for f in results {
        assert_eq!(f.outcome, Outcome::Fail);
        assert!((f.measurement.unwrap()[0] - stress).abs() <= stress * f64::EPSILON);
        assert!(
            f.coverage_gaps
                .iter()
                .any(|reason| reason.contains("меньше геометрической границы"))
        );
    }
}

fn check_rod_strength(fixture: &Fixture, selections: &[(usize, SectionAxis)]) {
    let plan = fixture.plan_with_sampling("", 0.01, false, &discrete_sampling());
    for &(geometry, axis) in selections {
        let mut inv = inventory();
        let Verification::Section {
            geometry: index, axis: direction, allowable_pa, ..
        } = &mut inv.checks[0].verification else {
            panic!("нет проверки стержня");
        };
        *index = geometry;
        *direction = axis;
        // Порог 100 кПа из отрицательной постановки независимой приёмки.
        *allowable_pa = [1e5; 2];
        let req = derive_requirements(&inv, &plan).unwrap();
        let mut evidence = CampaignEvidence::new(&req);
        for row in plan.rows() {
            evidence.execute(&req, &plan, &row.key, execute).unwrap();
        }
        let report = assess_campaign(&req, &evidence).unwrap();
        assert_eq!(report.simulation, SimulationVerdict::Rejected, "{geometry}/{axis:?}");
        // Независимая формула для метрового стержня: σ = 6FL / bh² = 750 кПа.
        let stress = 6.0 * 2.0 / (0.04 * 0.02_f64.powi(2));
        let strength: Vec<_> = report.findings.iter()
            .filter(|f| f.key.class == HazardClass::StaticStrength).collect();
        assert_eq!(strength.len(), 2);
        for finding in strength {
            assert!(finding.confirmed_hazard);
            assert!((finding.measurement.unwrap()[0] - stress).abs() <= stress * f64::EPSILON);
        }
    }
}

#[test]
fn выбор_оси_не_скрывает_изгиб_стержня() {
    let fixture = Fixture::new("section-direction");
    check_rod_strength(&fixture, &[(0, SectionAxis::X), (0, SectionAxis::Y), (0, SectionAxis::Z)]);
}

#[test]
fn выбор_коллизии_не_скрывает_тонкое_сечение() {
    let fixture = Fixture::new("section-geometry");
    let mut rod = fs::read_to_string(fixture.0.join("body.toml")).unwrap();
    // Второй объём из независимой приёмки имеет то же плечо,
    // а модуль сечения больше в тысячу раз.
    rod.push_str(r#"
[[links.collision]]
shape = { kind = "box", size_m = [1.0, 0.2, 0.4] }
contact_mask = [0, 0]
friction = [0.0, 0.0, 0.0]
"#);
    fixture.write("body.toml", &rod);
    check_rod_strength(&fixture, &[(0, SectionAxis::X), (1, SectionAxis::X)]);
}

fn run_strength(fixture: &Fixture, bound: f64, declared: [f64; 2]) -> sim_task::verdict::VerdictReport {
    let world = WorldInputs::load(fixture.0.join("world.toml")).unwrap();
    let hinge = world.devices["каретка"].device.links[0].joint.kind == sim_world::JointType::Hinge;
    let mut sampling = discrete_sampling();
    if hinge {
        // Угловой аналог проставки в синтетической постановке, 1 мкрад.
        sampling = sampling.replace(r#"unit = "m""#, r#"unit = "rad""#);
    }
    let plan = fixture.plan_with_sampling("", 0.01, false, &sampling);
    let mut inv = inventory();
    let Verification::Section { allowable_pa, moment_nm, .. } = &mut inv.checks[0].verification else {
        unreachable!()
    };
    *allowable_pa = [bound; 2];
    *moment_nm = declared;
    if hinge {
        inv.checks[1].verification = Verification::Truth(limit(Signal { unit: "rad".into(), ..position() }, 0.01));
    }
    for body in world.compile().unwrap().bodies {
        if let sim_world::Provenance::Derived(reference) = body.provenance {
            // Для теста с объёмом оба входа заданы протоколом: геометрия и плотность.
            inv.derivations.insert(reference, vec![measured()]);
        }
    }
    let req = derive_requirements(&inv, &plan).unwrap();
    let mut evidence = CampaignEvidence::new(&req);
    for row in plan.rows() {
        evidence.execute(&req, &plan, &row.key, execute).unwrap();
    }
    assess_campaign(&req, &evidence).unwrap()
}

#[test]
fn перестановка_размеров_сохраняет_худшее_направление_изгиба() {
    let fixture = Fixture::new("rotated-section");
    let rod = fs::read_to_string(fixture.0.join("body.toml")).unwrap();
    for body in [rod.clone(), rod.replace("[1.0, 0.02, 0.04]", "[1.0, 0.04, 0.02]")] {
        fixture.write("body.toml", &body);
        // Порог 500 кПа из отрицательного опыта. W_min = 40·20²/6 мм³.
        let report = run_strength(&fixture, 5e5, [2.0; 2]);
        assert_eq!(report.simulation, SimulationVerdict::Rejected);
        assert!(report.findings.iter().any(|f| f.key.class == HazardClass::StaticStrength
            && f.measurement == Some([750_000.0; 2]) && f.confirmed_hazard));
    }
}

#[test]
fn ось_и_поза_сечения_отличают_кручение_от_изгиба() {
    let fixture = Fixture::new("torsion-section");
    let rod = fs::read_to_string(fixture.0.join("body.toml")).unwrap()
        .replace("kind = \"slide\"", "kind = \"hinge\"")
        .replace(
            r#"shape = { kind = "box", size_m = [1.0, 0.02, 0.04] }"#,
            "shape = { kind = \"cylinder\", length_m = 1.0, radius_m = 0.02 }\npose = { rpy_deg = [0.0, 90.0, 0.0] }",
        );
    // Однородный цилиндр с прежним осевым I = 0,01 кг·м².
    // Из I = mr²/2 следует масса 50 кг при радиусе 20 мм.
    let mass = 2.0 * 0.01 / 0.02_f64.powi(2);
    let rod = uniform_mass(&rod, mass / (std::f64::consts::PI * 0.02_f64.powi(2) * 1.0));
    // В стенде 2 Н·м и диаметр 40 мм. Для круга J = 2I,
    // I = π d⁴/64. Порог между чистым кручением и чистым изгибом.
    let bending = 32.0 * 2.0 / (std::f64::consts::PI * 0.04_f64.powi(3));
    let torsion = bending * 3.0_f64.sqrt() / 2.0;
    let bound = (torsion + bending) / 2.0;
    for (axis, stress, verdict) in [
        ("[1.0, 0.0, 0.0]", torsion, SimulationVerdict::NotProven),
        ("[0.0, 1.0, 0.0]", bending, SimulationVerdict::Rejected),
    ] {
        fixture.write("body.toml", &rod.replace("axis = [1.0, 0.0, 0.0]", &format!("axis = {axis}")));
        let report = run_strength(&fixture, bound, [2.0; 2]);
        assert_eq!(report.simulation, verdict, "{:#?}", report.findings);
        let findings: Vec<_> = report.findings.iter().filter(|f| f.key.class == HazardClass::StaticStrength).collect();
        assert_eq!(findings.len(), 2);
        for f in findings {
            assert!((f.measurement.unwrap()[0] - stress).abs() < stress * 4.0 * f64::EPSILON);
            assert_eq!(f.outcome, if stress > bound { Outcome::Fail } else { Outcome::NotAnalyzed });
        }
    }
}

#[test]
fn маскированный_толстый_коллайдер_не_заменяет_тонкий_объём_массы() {
    let fixture = Fixture::new("mass-section");
    let src = fs::read_to_string(fixture.0.join("body.toml")).unwrap();
    let (head, rest) = src.split_once("[links.mass]").unwrap();
    let (_, collision) = rest.split_once("[[links.collision]]").unwrap();
    // Масса 2 кг при 1×0,02×0,04 м и плотности синтетического материала 2500 кг/м³.
    let vol = r#"[links.mass]
kind = "volumes"
material = { name = "стендовый материал", density_kg_m3 = 2500.0, provenance = { measured = "синтетическая плотность" } }
volumes = [{ shape = { kind = "box", size_m = [1.0, 0.02, 0.04] } }]
[[links.collision]]"#;
    let world = || WorldInputs::load(fixture.0.join("world.toml")).unwrap().compile().unwrap();
    let gap = "Объёмы массы и коллизии описывают разную физическую геометрию";
    fixture.write("body.toml", &format!("{head}{vol}{collision}"));
    assert!(!world().simplifications.iter().any(|s| s.text == gap));
    let nominal = run_strength(&fixture, 5e6, [2.0; 2]);
    assert_eq!(nominal.simulation, SimulationVerdict::NotProven);
    assert!(nominal.findings.iter().any(|f| f.key.class == HazardClass::StaticStrength
        && f.measurement == Some([750_000.0; 2]) && f.outcome == Outcome::NotAnalyzed));
    fixture.write("body.toml", &format!("{head}{vol}{}", collision.replace("[1.0, 0.02, 0.04]", "[1.0, 0.2, 0.4]")));
    assert!(world().simplifications.iter().any(|s| s.text == gap && s.blocks_proof));
    for (bound, verdict, outcome) in [
        (5e5, SimulationVerdict::Rejected, Outcome::Fail),
        (5e6, SimulationVerdict::NotProven, Outcome::NotAnalyzed),
    ] {
        let report = run_strength(&fixture, bound, [2.0; 2]);
        assert_eq!(report.simulation, verdict);
        assert!(report.findings.iter().any(|f| f.key.class == HazardClass::StaticStrength
            && f.measurement == Some([750_000.0; 2]) && f.outcome == outcome
            && f.coverage_gaps.iter().any(|reason| reason.contains(gap))));
    }
}

#[test]
fn заявленный_момент_может_быть_опаснее_живого() {
    let fixture = Fixture::new("declared-moment");
    // Для 2 Н·м запас положительный, для 20 Н·м того же знака или
    // обратного знак не меняет модуль напряжения. Предел стенда 5 МПа.
    let nominal = run_strength(&fixture, 5e6, [2.0; 2]);
    assert_eq!(nominal.simulation, SimulationVerdict::NotProven);
    assert!(nominal.findings.iter().any(|f| f.key.class == HazardClass::StaticStrength
        && f.measurement == Some([750_000.0; 2]) && f.outcome == Outcome::NotAnalyzed));
    for declared in [[2.0, 20.0], [-20.0, 2.0]] {
        let report = run_strength(&fixture, 5e6, declared);
        assert_eq!(report.simulation, SimulationVerdict::Rejected);
        let stress = 6.0 * 20.0 / (0.04 * 0.02_f64.powi(2));
        assert!(report.findings.iter().any(|f| f.key.class == HazardClass::StaticStrength
            && f.measurement.is_some_and(|[lo, hi]| lo == hi && (lo - stress).abs() <= stress * f64::EPSILON)
            && f.confirmed_hazard
            && f.event_ns.is_none() && f.journal.is_some()));
    }
}

#[test]
fn вращение_с_начальной_скоростью_перегружает_корень_стержня() {
    let fixture = Fixture::new("centrifugal-section");
    let rod = fs::read_to_string(fixture.0.join("body.toml")).unwrap()
        .replace("kind = \"slide\"", "kind = \"hinge\"")
        .replace("axis = [1.0, 0.0, 0.0]", "axis = [0.0, 0.0, 1.0]")
        .replace("com_m = [0.0, 0.0, 0.0]", "com_m = [0.5, 0.0, 0.0]")
        .replace("[[links.collision]]", "[[links.collision]]\npose = { position_m = [0.5, 0.0, 0.0] }");
    fixture.write("body.toml", &rod);
    let mut inv = inventory();
    inv.checks[1].verification = Verification::Truth(limit(Signal { unit: "rad".into(), ..position() }, 1.0));
    // Скорость 100 рад/с и геометрия из примера приёмки. Окно 4 мс,
    // поэтому критерий угла 1 рад оставляет запас и не подменяет проверку сечения.
    let plan = fixture.plan_with_sampling(
        &format!(r#"initial = [{{ parameter = {{ kind = "joint_velocity", joint = "каретка/стержень", dof = 0 }}, value = 100.0, unit = "rad/s", source = {SOURCE} }}]"#),
        1.0,
        false,
        &discrete_sampling().replace(r#"unit = "m""#, r#"unit = "rad""#),
    );
    let req = derive_requirements(&inv, &plan).unwrap();
    let mut evidence = CampaignEvidence::new(&req);
    // Баланс при ω₀=100 рад/с: Fₓ = −mω₀²r, Fᵧ = mrα,
    // α = M/(I_z + mr²). Момент реакции в корне равен приложенному.
    let axial_force = 2.0 * 100.0_f64.powi(2) * 0.5;
    let angular_acceleration = 2.0 / (0.025 + 2.0 * 0.5_f64.powi(2));
    let tangential_force = 2.0 * 0.5 * angular_acceleration;
    for row in plan.rows() {
        evidence.execute_scenario_observed(&req, &plan, &row.key, Some(sim_signal::Time(2 * ROW.max_gap_ns)),
            |rt, sample| {
                rt.set_initial_velocities(&sample.settings.initial_velocity)?;
                rt.record_joint_state("кинематика")?;
                Ok(receipt(sample))
            },
            |rt| {
                for (component, expected) in [("fx", -axial_force), ("fy", tangential_force), ("mz", 2.0)] {
                    let observed = reaction_at(rt, component, ROW.max_gap_ns);
                    // Сверяем малый касательный вклад на фоне центробежной реакции.
                    let tolerance = axial_force * f64::EPSILON;
                    assert!((observed - expected).abs() <= tolerance, "{component}: {observed} / {expected}");
                }
                Ok(())
            },
        ).unwrap();
        assert!(evidence.scenario_report(&row.key).unwrap().3.unwrap().matched);
    }
    let report = assess_campaign(&req, &evidence).unwrap();
    assert_eq!(report.simulation, SimulationVerdict::Rejected, "{:#?}", report.findings);
    assert_eq!(report.readiness, Readiness::Rejected);
    let findings: Vec<_> = report.findings.iter()
        .filter(|f| f.key.class == HazardClass::StaticStrength).collect();
    assert_eq!(findings.len(), plan.rows().len());
    // m·ω²·r / A даёт нижнюю границу по одной продольной составляющей.
    let area = 0.02 * 0.04;
    let axial = axial_force / area;
    // Последний интервал начинается после трёх шагов Euler.
    // Изгиб 6M/(bh²), максимальный сдвиг прямоугольника 3V/(2A).
    let last_left = (ROW.end_ns - ROW.max_gap_ns) as f64 * 1e-9;
    let speed = 100.0 + angular_acceleration * last_left;
    let normal = 2.0 * 0.5 * speed.powi(2) / area + 6.0 * 2.0 / (0.04 * 0.02_f64.powi(2));
    let expected_peak = normal.hypot(3.0_f64.sqrt() * 1.5 * tangential_force / area);
    for finding in findings {
        assert_eq!(finding.outcome, Outcome::Fail);
        assert!(finding.confirmed_hazard);
        let actual = finding.measurement.unwrap()[0];
        assert!((actual - expected_peak).abs() <= expected_peak * f64::EPSILON.sqrt());
        assert_eq!(finding.event_ns, Some(ROW.end_ns - ROW.max_gap_ns));
    }
    let peak = report.findings.iter().filter(|f| f.key.class == HazardClass::StaticStrength)
        .map(|f| f.measurement.unwrap()[0]).fold(0.0_f64, f64::max);
    println!("Ручной расчёт F/A = {axial:.3} Па. Пик эквивалентного напряжения {peak:.3} Па.");
}

fn uniform_mass(rod: &str, density: f64) -> String {
    let (head, rest) = rod.split_once("[links.mass]").unwrap();
    let (_, collision) = rest.split_once("[[links.collision]]").unwrap();
    format!(
        r#"{head}[links.mass]
kind = "geometry"
material = {{ name = "материал стенда", density_kg_m3 = {density:?}, provenance = {{ measured = "плотность однородного образца" }} }}
[[links.collision]]{collision}"#
    )
}

#[test]
fn вращение_диска_отклоняется_включая_последний_отсчёт() {
    let fixture = Fixture::new("centrifugal-disk");
    // Геометрия и скорость из независимой приёмки. ρ = 2500 кг/м³
    // и ν = 0,3 дают указанный масштаб напряжения.
    let (radius, length, density, speed) = (0.3_f64, 0.02, 2500.0, 3000.0_f64);
    let rod = fs::read_to_string(fixture.0.join("body.toml")).unwrap()
        .replace("kind = \"slide\"", "kind = \"hinge\"")
        .replace("axis = [1.0, 0.0, 0.0]", "axis = [0.0, 0.0, 1.0]")
        .replace("kind = \"box\", size_m = [1.0, 0.02, 0.04]",
            &format!("kind = \"cylinder\", radius_m = {radius}, length_m = {length}"));
    fixture.write("body.toml", &uniform_mass(&rod, density));
    let world = WorldInputs::load(fixture.0.join("world.toml")).unwrap();
    let mut inv = inventory();
    for body in world.compile().unwrap().bodies {
        if let sim_world::Provenance::Derived(reference) = body.provenance {
            inv.derivations.insert(reference, vec![measured()]);
        }
    }
    let Verification::Section { allowable_pa, live_load, .. } = &mut inv.checks[0].verification else {
        unreachable!()
    };
    // Предел пробы ниже 835 МПа. Отключаем объявленную нагрузку сечения:
    // вращение должно проверяться даже без канала измерения реакции.
    *allowable_pa = [800e6; 2];
    *live_load = None;
    inv.checks[1].verification = Verification::Truth(limit(
        Signal { unit: "rad".into(), ..position() },
        speed * ROW.end_ns as f64 * 1e-9 + 1.0, // Радиан запаса над ходом свободного диска.
    ));
    let inertia = density * std::f64::consts::PI * radius.powi(4) * length / 2.0;
    let torque = inertia * speed / (ROW.max_gap_ns as f64 * 1e-9);
    let initial = format!(r#"initial = [{{ parameter = {{ kind = "joint_velocity", joint = "каретка/стержень", dof = 0 }}, value = {speed:?}, unit = "rad/s", source = {SOURCE} }}]"#);
    let spin_final = format!(r#", {{ id = "разгон", target = {{ kind = "effort", name = "каретка/сила" }}, at_ns = {}, value = {torque:?}, unit = "N*m", source = {SOURCE} }}"#,
        ROW.end_ns - ROW.max_gap_ns);
    for (part, step, at) in [(&initial[..], "", None), ("", &spin_final[..], Some(ROW.end_ns))] {
        let plan = fixture.custom_plan(part, 13.0, false,
            &discrete_sampling().replace(r#"unit = "m""#, r#"unit = "rad""#),
            r#"{ kind = "scenario" }"#, (0.0, step));
        let req = derive_requirements(&inv, &plan).unwrap();
        let mut evidence = CampaignEvidence::new(&req);
        for row in plan.rows() {
            evidence.execute_scenario_observed(&req, &plan, &row.key,
                Some(sim_signal::Time(2 * ROW.max_gap_ns)),
                |rt, sample| {
                    rt.set_initial_velocities(&sample.settings.initial_velocity)?;
                    rt.record_joint_state("кинематика")?;
                    Ok(receipt(sample))
                },
                |rt| {
                    // Независимая проверка скорости из JΔω = MΔt.
                    let actual = rt.joint_velocity("каретка/стержень", 0).unwrap();
                    assert!((actual - speed).abs() <= speed * 8.0 * f64::EPSILON);
                    Ok(())
                }).unwrap();
            assert!(evidence.scenario_report(&row.key).unwrap().3.unwrap().matched);
        }
        let report = assess_campaign(&req, &evidence).unwrap();
        assert_eq!(report.simulation, SimulationVerdict::Rejected, "{:#?}", report.findings);
        for finding in report.findings.iter().filter(|f| f.key.class == HazardClass::StaticStrength) {
            assert_eq!(finding.outcome, Outcome::Fail);
            let observed_at = finding.event_ns.unwrap();
            if let Some(at) = at {
                assert_eq!(observed_at, at);
            }
            let [reference, envelope] = finding.measurement.unwrap();
            // Независимая подстановка ρ·(ωR)² = 2025 МПа.
            assert!((reference - 835.3125e6).abs() <= reference * 16.0 * f64::EPSILON);
            assert!((envelope - 1012.5e6).abs() <= envelope * 16.0 * f64::EPSILON);
            println!("Диск, {observed_at} нс: {reference:.1} / {envelope:.1} Па, {:?}", finding.outcome);
        }
    }
}

fn центральный_стержень(
    id: &str, mass: f64, gravity: f64, speed: f64, allowable: f64, uniform: bool,
) -> sim_task::verdict::VerdictReport {
    let fixture = Fixture::new(id);
    let axis = if gravity == 0.0 { "[0.0, 0.0, 1.0]" } else { "[0.0, 1.0, 0.0]" };
    let rod = fs::read_to_string(fixture.0.join("body.toml")).unwrap()
        .replace("kind = \"slide\"", "kind = \"hinge\"")
        .replace("axis = [1.0, 0.0, 0.0]", &format!("axis = {axis}"));
    let rod = if uniform {
        // Плотность тестового образца определяется его заданной массой и размерами.
        uniform_mass(&rod, mass / (1.0 * 0.02 * 0.04))
    } else {
        rod
    };
    fixture.write("body.toml", &rod);
    run_centered_rod(&fixture, mass, gravity, speed, allowable)
}

fn run_centered_rod(
    fixture: &Fixture, mass: f64, gravity: f64, speed: f64, allowable: f64,
) -> sim_task::verdict::VerdictReport {
    let world = fs::read_to_string(fixture.0.join("world.toml")).unwrap()
        .replace("gravity_m_s2 = [0.0, 0.0, 0.0]", &format!("gravity_m_s2 = [0.0, 0.0, {gravity:?}]"));
    fixture.write("world.toml", &world);
    // Два угла исходной установки из дискретной пары, без приложенного момента.
    // Предел хода 2 рад больше 300 рад/с × 4 мс.
    let plan = fixture.custom_plan(
        &format!(r#"initial = [{{ parameter = {{ kind = "joint_velocity", joint = "каретка/стержень", dof = 0 }}, value = {speed:?}, unit = "rad/s", source = {SOURCE} }}]"#),
        2.0, false, &discrete_sampling().replace(r#"unit = "m""#, r#"unit = "rad""#),
        r#"{ kind = "scenario" }"#, (0.0, ""),
    );
    let mut inv = inventory();
    inv.checks[1].verification = Verification::Truth(limit(Signal { unit: "rad".into(), ..position() }, 2.0));
    let Verification::Section { allowable_pa, .. } = &mut inv.checks[0].verification else {
        unreachable!()
    };
    *allowable_pa = [allowable; 2];
    let world = WorldInputs::load(fixture.0.join("world.toml")).unwrap();
    for body in world.compile().unwrap().bodies {
        if let sim_world::Provenance::Derived(reference) = body.provenance {
            inv.derivations.insert(reference, vec![measured()]);
        }
    }
    let req = derive_requirements(&inv, &plan).unwrap();
    let mut evidence = CampaignEvidence::new(&req);
    for row in plan.rows() {
        evidence.execute_scenario_observed(&req, &plan, &row.key, Some(sim_signal::Time(2 * ROW.max_gap_ns)),
            |rt, sample| {
                rt.set_initial_velocities(&sample.settings.initial_velocity)?;
                rt.record_joint_state("кинематика")?;
                Ok(receipt(sample))
            },
            |rt| {
                // Симметрия тела: приводного момента нет, в корне остаётся только вес.
                for (component, expected) in [("fx", 0.0), ("fy", 0.0), ("fz", mass * -gravity), ("my", 0.0), ("mz", 0.0)] {
                    let actual = reaction_at(rt, component, ROW.max_gap_ns);
                    // Ориентация второй установки отличается на 1 мкрад.
                    let tolerance = 1e-6 * expected.abs().max(mass * gravity.abs()).max(1.0);
                    assert!((actual - expected).abs() <= tolerance, "{component}: {actual} / {expected}");
                }
                Ok(())
            },
        ).unwrap();
        assert!(evidence.scenario_report(&row.key).unwrap().3.unwrap().matched);
    }
    assess_campaign(&req, &evidence).unwrap()
}

#[test]
fn центральное_сечение_несёт_центробежное_растяжение_обеих_половин() {
    let report = центральный_стержень("inner-centrifugal", 2.0, 0.0, 300.0, 5e6, true);
    // Каждая половина весит 1 кг. Её центр масс на четверти метровой длины.
    let expected = (2.0 / 2.0) * 300.0_f64.powi(2) * (1.0 / 4.0) / (0.02 * 0.04);
    check_inner_strength(&report, expected, 0.0);
    let unknown = центральный_стержень("inner-explicit-mass", 2.0, 0.0, 300.0, 5e6, false);
    assert_eq!(unknown.simulation, SimulationVerdict::NotProven, "{:#?}", unknown.findings);
    assert!(unknown.findings.iter().any(|f| f.key.class == HazardClass::StaticStrength
        && f.coverage_gaps.iter().any(|reason| reason.contains("распределение массы"))));
}

#[test]
fn центральное_сечение_несёт_изгиб_от_веса_обеих_половин() {
    let report = центральный_стержень("inner-gravity", 200.0, -9.81, 0.0, 10e6, true);
    let force = (200.0 / 2.0) * 9.81;
    let moment = force * (1.0 / 4.0);
    // Изгиб вокруг y, высота по z. Максимальный поперечный сдвиг прямоугольника 3V/(2A).
    let normal = moment / (0.02 * 0.04_f64.powi(2) / 6.0);
    let shear = 1.5 * force / (0.02 * 0.04);
    let angle_error = force * 1e-6 / (0.02 * 0.04);
    check_inner_strength(&report, normal.hypot(3.0_f64.sqrt() * shear), angle_error);
}

#[test]
fn тонкая_сторона_ступени_несёт_нагрузку_у_стыка() {
    let fixture = Fixture::new("stepped-rod");
    let rod = fs::read_to_string(fixture.0.join("body.toml")).unwrap()
        .replace("kind = \"slide\"", "kind = \"hinge\"")
        .replace("axis = [1.0, 0.0, 0.0]", "axis = [0.0, 0.0, 1.0]")
        .replace("[1.0, 0.02, 0.04]", "[0.5, 0.04, 0.08]");
    // Центральные 0,5 м: 4 кг. По краям два отрезка по 0,25 м и 0,5 кг.
    // При 300 рад/с толстая сторона у опоры несёт 12,305 МПа,
    // а в каждой тонкой стороне у стыка сила равна m·ω²·0,375 м.
    let mut rod = uniform_mass(&rod, 2500.0);
    for x in [-0.375, 0.375] {
        rod.push_str(&format!(r#"
[[links.collision]]
shape = {{ kind = "box", size_m = [0.25, 0.02, 0.04] }}
pose = {{ position_m = [{x:?}, 0.0, 0.0] }}
contact_mask = [0, 0]
"#));
    }
    fixture.write("body.toml", &rod);
    let expected = 0.5 * 300.0_f64.powi(2) * 0.375 / (0.02 * 0.04);
    let report = run_centered_rod(&fixture, 5.0, 0.0, 300.0, 15e6);
    check_inner_strength(&report, expected, 0.0);
    let open = run_centered_rod(&fixture, 5.0, 0.0, 300.0, 25e6);
    assert_eq!(open.simulation, SimulationVerdict::NotProven);
    assert!(open.findings.iter().any(|f| f.key.class == HazardClass::StaticStrength
        && f.coverage_gaps.iter().any(|reason| reason.contains("передача нагрузки между объёмами"))));
}

fn check_inner_strength(report: &sim_task::verdict::VerdictReport, expected: f64, pose_error: f64) {
    assert_eq!(report.simulation, SimulationVerdict::Rejected, "{:#?}", report.findings);
    let findings: Vec<_> = report.findings.iter().filter(|f| f.key.class == HazardClass::StaticStrength).collect();
    assert_eq!(findings.len(), 2);
    for finding in findings {
        assert_eq!(finding.outcome, Outcome::Fail);
        assert!(finding.confirmed_hazard);
        assert_eq!(finding.event_ns, Some(0));
        let [lo, hi] = finding.measurement.unwrap();
        // Поворот проставки 1 мкрад меняет осевую силу не более чем на вес × угол.
        let tolerance = expected * f64::EPSILON.sqrt() + pose_error;
        assert!((lo - expected).abs() <= tolerance && (hi - expected).abs() <= tolerance,
            "огибающая {lo}…{hi}, ручной расчёт {expected}");
        println!("Напряжение {lo:.3}…{hi:.3} Па, ручной расчёт {expected:.3} Па.");
    }
}

#[test]
fn реакция_корня_сходится_с_балансом_тяжести_и_контакта() {
    let fixture = Fixture::new("contact-section");
    let world = fs::read_to_string(fixture.0.join("world.toml")).unwrap()
        .replace("gravity_m_s2 = [0.0, 0.0, 0.0]", "gravity_m_s2 = [0.0, 0.0, -10.0]");
    let rod = fs::read_to_string(fixture.0.join("body.toml")).unwrap()
        .replace("contact_mask = [0, 0]", "contact_mask = [1, 1]");
    fixture.write("body.toml", &rod);
    for with_contact in [false, true] {
        // Нижняя грань стержня на −20 мм. Площадка толщиной 100 мм
        // с центром на −69 мм даёт 1 мм начального вдавливания.
        let support = if with_contact { r#"
[[environment.collision]]
shape = { kind = "box", size_m = [2.0, 1.0, 0.1] }
pose = { position_m = [0.0, 0.0, -0.069] }
contact_mask = [1, 1]
"# } else { "" };
        fixture.write("world.toml", &world.replace("[[devices]]", &format!("{support}\n[[devices]]")));
        let plan = fixture.plan("", 0.01, false);
        let req = derive_requirements(&inventory(), &plan).unwrap();
        let mut evidence = CampaignEvidence::new(&req);
        let row = &plan.rows()[0];
        let compiled = plan.materialize(&row.key).unwrap().world.compile().unwrap();
        let mut engine = compiled.open_contact_sim().unwrap();
        let rod = engine.joints()[0].body;
        let geoms = engine.geoms();
        engine.qfrc_applied_mut()[0] = 2.0;
        engine.step();
        let contact_force: f64 = engine.contacts().iter().map(|contact| {
            let second = geoms[contact.geom[1] as usize].body == rod;
            contact.normal_force * contact.normal[2] * if second { 1.0 } else { -1.0 }
        }).sum();
        assert_eq!(contact_force > 0.0, with_contact);
        // Каретка вдоль x. Ускорение по z равно нулю, поэтому
        // реакция опоры по z равна весу за вычетом нормальных реакций контакта.
        let weight = 2.0 * 10.0;
        evidence.execute_scenario_observed(&req, &plan, &row.key, None,
            |rt, sample| {
                rt.record_joint_state("кинематика")?;
                Ok(receipt(sample))
            },
            |rt| {
                let actual = reaction_at(rt, "fz", ROW.max_gap_ns);
                let expected = weight - contact_force;
                // Бюджет округления для суммы до четырёх точек плоского контакта.
                let tolerance = 4.0 * f64::EPSILON * (weight + contact_force);
                assert!((actual - expected).abs() <= tolerance, "{actual} Н / {expected} Н");
                println!("Тяжесть и контакт. Реакция по z {actual:.6} Н, по балансу {expected:.6} Н.");
                Ok(())
            },
        ).unwrap();
    }
}

#[test]
fn произвольный_ряд_не_доказывает_отсутствие_клина() {
    let fixture = Fixture::new("travel-signal");
    let plan = fixture.plan_with_sampling("", 0.01, false, &discrete_sampling());
    let mut inv = inventory();
    // Скорость 4 мм/с на четвёртом такте не заменяет координату при проверке хода.
    let unrelated = Signal { name: "qvel[0]".into(), unit: "m/s".into() };
    inv.checks[1].verification = Verification::Truth(limit(unrelated, 0.01));
    let req = derive_requirements(&inv, &plan).unwrap();
    let mut evidence = CampaignEvidence::new(&req);
    for row in plan.rows() {
        evidence.execute(&req, &plan, &row.key, execute).unwrap();
    }
    let report = assess_campaign(&req, &evidence).unwrap();
    assert_eq!(report.simulation, SimulationVerdict::NotProven);
    let findings: Vec<_> = report.findings.iter().filter(|f| f.key.class == HazardClass::TravelJam).collect();
    assert_eq!(findings.len(), 2);
    assert!(findings.iter().all(|f| f.outcome == Outcome::NotAnalyzed
        && f.measurement == Some([0.0, 0.004])
        && f.coverage_gaps.iter().any(|gap| gap.contains("привязки ряда"))));
}

#[test]
fn открытое_механическое_крепление_открывает_участника() {
    let fixture = Fixture::new("open-mount");
    let rod = fs::read_to_string(fixture.0.join("body.toml")).unwrap();
    for (kind, verdict) in [
        ("signal", SimulationVerdict::PassedSimulation),
        ("mechanical", SimulationVerdict::NotProven),
    ] {
        fixture.write("body.toml", &format!(r#"{rod}
[[links.mounts]]
name = "крепление"
kind = "{kind}"
interface = "ремень посадки"
"#));
        let plan = fixture.plan_with_sampling("", 0.01, false, &discrete_sampling());
        let mut inv = inventory();
        strength_bench_check(&mut inv);
        let req = derive_requirements(&inv, &plan).unwrap();
        let mut evidence = CampaignEvidence::new(&req);
        for row in plan.rows() {
            evidence.execute(&req, &plan, &row.key, execute).unwrap();
            record_strength(&mut evidence, &req, &row.key);
        }
        let report = assess_campaign(&req, &evidence).unwrap();
        assert_eq!(report.simulation, verdict);
        assert_eq!(report.findings.iter().any(|f| f.key.class == HazardClass::TissueLoad
            && f.key.object == "каретка/стержень/крепление" && f.outcome == Outcome::NotAnalyzed),
            kind == "mechanical");
    }
}

#[test]
fn связанный_тип_станции_не_скрывает_ход_и_открытое_крепление() {
    let fixture = Fixture::new("linked-reach");
    let rod = fs::read_to_string(fixture.0.join("body.toml")).unwrap();
    let world = fs::read_to_string(fixture.0.join("world.toml")).unwrap();
    // Грань блока на 0,7 м от центра стержня, его торец на 0,5 м.
    // Разрешённый ход 0,3 м перекрывает начальный зазор 0,2 м.
    fixture.write("body.toml", &format!(r#"{}
[[links.mounts]]
name = "крепление"
kind = "mechanical"
interface = "держатель"
"#, rod.replace("damping = 0.0", "limits_m = [0.0, 0.3]\ndamping = 0.0")));
    fixture.write("block.toml", r#"
format = 3
name = "Блок"
[[links]]
name = "объём"
joint = { kind = "fixed" }
mass = { kind = "massless" }
[[links.collision]]
shape = { kind = "box", size_m = [0.1, 0.1, 0.1] }
contact_mask = [0, 0]
"#);
    fixture.write("world.toml", &format!(r#"{world}
[[devices]]
name = "блок"
file = "block.toml"
pose = {{ position_m = [0.75, 0.0, 0.0] }}
"#));
    // Разрешитель планировщика в этой проверке подтверждает файл ссылки.
    // Досягаемость считается на физическом мире, без запуска адаптера.
    fixture.write("profile.toml", "version = 1");
    for kind in ["process", "interaction"] {
        fixture.write("linked.toml", &format!("version = 1\n{kind}_profile = \"profile.toml\""));
        let experiment = format!(r#"{{ kind = "{kind}", task = "linked.toml" }}"#);
        let plan = fixture.custom_plan("", 0.01, false, &discrete_sampling(), &experiment, (2.0, ""));
        let req = derive_requirements(&inventory(), &plan).unwrap();
        for object in ["блок/объём", "каретка/стержень/крепление"] {
            assert!(req.keys().any(|key| key.class == HazardClass::TissueLoad
                && key.object == object && key.run.is_some()), "{kind}: {object}");
        }
    }
}

#[test]
fn опасности_модели_открываются_без_объявления_в_инвентаре() {
    let fixture = Fixture::new("modeled-obligations");
    let rod = fs::read_to_string(fixture.0.join("body.toml")).unwrap();
    let world = fs::read_to_string(fixture.0.join("world.toml")).unwrap();
    fixture.write("body.toml", &format!(r#"{rod}
[[links.mounts]]
name = "вывод"
kind = "power"
interface = "разъём"
[[links.mounts]]
name = "крепление"
kind = "mechanical"
interface = "держатель"
"#));
    fixture.write("world.toml", &format!(r#"{world}
[[sensors]]
name = "координата"
part = "датчик положения"
joint = "каретка/стержень"
"#));
    // Усилие 2 Н в нуле, затем разгрузка на втором такте и повторное нагружение.
    let steps = format!(r#", {{ id = "release", target = {{ kind = "effort", name = "каретка/сила" }}, at_ns = 1000000, value = 0.0, unit = "N", source = {SOURCE} }},
{{ id = "return", target = {{ kind = "effort", name = "каретка/сила" }}, at_ns = 2000000, value = 2.0, unit = "N", source = {SOURCE} }}"#);
    let plan = fixture.custom_plan("", 0.01, false, &discrete_sampling(), r#"{ kind = "scenario" }"#, (2.0, &steps));
    let inv = inventory();
    assert!(inv.components.is_empty());
    let req = derive_requirements(&inv, &plan).unwrap();
    let mut evidence = CampaignEvidence::new(&req);
    for row in plan.rows() {
        evidence.execute_timed(&req, &plan, &row.key, |rt, sample| {
            rt.record_joint_state("кинематика")?;
            Ok(receipt(sample))
        }, |_, _| unreachable!()).unwrap();
    }
    let report = assess_campaign(&req, &evidence).unwrap();
    assert_eq!(report.simulation, SimulationVerdict::NotProven);
    for (class, object) in [
        (HazardClass::Fatigue, "каретка/стержень"),
        (HazardClass::Battery, "каретка/источник питания"),
        (HazardClass::SharpEdge, "каретка/стержень"),
        (HazardClass::ElectricShock, "каретка/стержень/вывод"),
        (HazardClass::EmergencyStop, "каретка/стержень/вывод"),
        (HazardClass::Measurement, "координата"),
    ] {
        assert!(report.findings.iter().any(|f| f.key.class == class
            && f.key.object == object && f.outcome == Outcome::NotAnalyzed), "{class:?}: {object}");
        assert!(!report.inapplicable.iter().any(|f| f.class == class));
    }
    assert!(report.findings.iter().filter(|f| f.key.class == HazardClass::Execution)
        .all(|f| f.outcome == Outcome::Pass));
}

#[test]
fn правила_новой_приёмки_отмечены_в_таблице_доверия() {
    let document = include_str!("../../../docs/STAGE6_PLAN.md");
    let (_, table) = document.split_once("## Доверие к объявлениям").unwrap();
    for number in [1, 2, 6, 12, 35] {
        let row = table.lines().find(|line| line.starts_with(&format!("| {number} |"))).unwrap();
        assert!(row.contains('★'), "{row}");
    }
}

#[test]
fn ход_каретки_открывает_достижимость_без_объявленного_контакта() {
    let fixture = Fixture::new("powered-reach");
    let body = fs::read_to_string(fixture.0.join("body.toml")).unwrap();
    // Половина стержня 0,5 м. Грань неподвижного блока начинается в 0,7 м.
    // Начальный зазор 0,2 м, ход 0,3 м. При поперечном зазоре 1 м контакта нет.
    fixture.write(
        "body.toml",
        &body.replace("damping = 0.0", "limits_m = [0.0, 0.3]\ndamping = 0.0"),
    );
    fixture.write(
        "block.toml",
        r#"
format = 3
name = "Блок"
[[links]]
name = "объём"
joint = { kind = "fixed" }
mass = { kind = "massless" }
[[links.collision]]
shape = { kind = "box", size_m = [0.1, 0.1, 0.1] }
contact_mask = [0, 0]
"#,
    );
    let world = fs::read_to_string(fixture.0.join("world.toml")).unwrap();
    // На 1,0 м вдоль оси зазор лежит за номинальным упором. Мягкое
    // ограничение не доказывает изоляцию, поперечный зазор остаётся доказуемым.
    for (along, offset, reachable) in [(0.75, 0.0, true), (1.0, 0.0, true), (0.75, 1.0, false)] {
        fixture.write(
            "world.toml",
            &format!(
                r#"{world}
[[devices]]
name = "блок"
file = "block.toml"
pose = {{ position_m = [{along:?}, {offset:?}, 0.0] }}
"#
            ),
        );
        let plan = fixture.plan_with_sampling("", 0.01, false, &discrete_sampling());
        let req = derive_requirements(&inventory(), &plan).unwrap();
        let mut evidence = CampaignEvidence::new(&req);
        for row in plan.rows() {
            evidence.execute(&req, &plan, &row.key, execute).unwrap();
        }
        let report = assess_campaign(&req, &evidence).unwrap();
        assert_eq!(
            report
                .findings
                .iter()
                .any(|f| f.key.object == "состав/достижимость"
                    && f.key.level == sim_task::verdict::Level::Simulation),
            reachable
        );
        assert_eq!(
            report
                .findings
                .iter()
                .any(|f| f.key.class == HazardClass::TissueLoad && f.key.object == "блок/объём"),
            reachable
        );
        assert_eq!(
            report
                .findings
                .iter()
                .any(|f| f.key.class == HazardClass::TravelJam
                    && f.key.extension.as_deref() == Some("замер")),
            reachable
        );
    }
}

#[test]
fn односторонний_предел_хода_ограничивает_обратное_смещение() {
    let fixture = Fixture::new("reverse-travel");
    let plan = fixture.plan_with_sampling(
        &format!(r#"initial = [{{ parameter = {{ kind = "joint_velocity", joint = "каретка/стержень", dof = 0 }}, value = -5.0, unit = "m/s", source = {SOURCE} }}]"#),
        0.01,
        false,
        &discrete_sampling(),
    );
    for (min, outcome) in [(None, Outcome::Fail), (Some(-0.03), Outcome::Pass)] {
        let mut inv = inventory();
        inv.checks.iter_mut().find(|c| c.class == HazardClass::TravelJam).unwrap()
            .verification = Verification::Truth(Limit::Range {
                signal: position(), min, max: Some(0.01), source: measured(),
            });
        let req = derive_requirements(&inv, &plan).unwrap();
        let mut evidence = CampaignEvidence::new(&req);
        for row in plan.rows() {
            evidence.execute(&req, &plan, &row.key, |rt, sample| {
                rt.set_initial_velocities(&sample.settings.initial_velocity)?;
                execute(rt, sample)
            }).unwrap();
        }
        let report = assess_campaign(&req, &evidence).unwrap();
        let findings: Vec<_> = report.findings.iter()
            .filter(|f| f.key.class == HazardClass::TravelJam).collect();
        assert_eq!(findings.len(), plan.rows().len());
        for finding in findings {
            assert_eq!(finding.outcome, outcome, "{finding:#?}");
            // Четыре шага по 1 мс, v₀ = −5 м/с, a = 2 Н / 2 кг.
            let displacement = -5.0 * 4e-3 + 10.0 * 1e-6;
            let [observed, _] = finding.measurement.unwrap();
            let initial = observed - displacement;
            assert!([0.0, 1e-6].iter().any(|q0| (initial - q0).abs() < 1e-14));
            if min.is_none() {
                assert!(finding.confirmed_hazard);
                assert_eq!(finding.event_ns, Some(3_000_000));
            }
        }
    }
}

#[test]
fn отсутствующий_пустой_и_номинальный_разброс_закрывают_допуск() {
    let fixture = Fixture::new("sampling-required");
    let discrete = discrete_sampling();
    let same_state = discrete
        .replace("values = [0.0, 0.000001]", "min = 0.0, max = 0.000001")
        .replace(
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
discrete = {{ source = {SOURCE}, boundary_ids = ["движение"] }}
axes = [
    {{ id = "скорость", parameter = {{ kind = "joint_velocity", joint = "каретка/стержень", dof = 0 }}, scope = "run", unit = "m/s", nominal = 0.0, values = [0.0, 0.2], source = {SOURCE} }},
    {{ id = "доставка", parameter = {{ kind = "channel_delay", channel = "observation:координата" }}, scope = "run", unit = "s", nominal = 0.0, values = [0.0, 0.0015], source = {SOURCE} }},
]
boundaries = [{{ id = "движение", values = {{ "скорость" = 0.2, "доставка" = 0.0015 }}, source = {SOURCE} }}]
"#
    );
    let plan = fixture.plan_with_sampling("", 0.01, false, &sampling);
    // Два двоичных переключателя дают четыре сочетания; в файле описан
    // только угол (max,max), а движок дополняет два смешанных угла.
    let all_settings = plan.rows().iter().map(|row| {
        let sample = plan.materialize(&row.key).unwrap();
        (sample.settings.initial_velocity["каретка/стержень"][&0] > 0.0,
         sample.settings.channel_delay_ns["observation:координата"] > 0)
    }).collect::<std::collections::BTreeSet<_>>();
    assert_eq!(all_settings, [(false, false), (false, true), (true, false), (true, true)].into());
    assert_eq!(plan.rows().len(), 4);
    let mut inv = inventory();
    strength_bench_check(&mut inv);
    let req = derive_requirements(&inv, &plan).unwrap();
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
    let install = |rt: &mut Runtime, sampled: &SampledInputs, velocity: bool, delay: bool| {
        let id = rt.register_observation("координата", "m")?;
        rt.register_sensor("датчик", rt.physics_rate(), Reader(id))?;
        if velocity {
            rt.set_initial_velocities(&sampled.settings.initial_velocity)?;
        }
        if delay {
            rt.set_channel_delays(&sampled.settings.channel_delay_ns)?;
        }
        execute(rt, sampled)
    };
    let continuous = sampling
        .replace("values = [0.0, 0.2]", "min = 0.0, max = 0.2")
        .replace("values = [0.0, 0.0015]", "min = 0.0, max = 0.0015");
    let ranges = fixture.plan_with_sampling("", 0.01, false, &continuous);
    assert_eq!(ranges.rows().len(), 2);
    let ranges_req = derive_requirements(&inventory(), &ranges).unwrap();
    let mut ranges_evidence = CampaignEvidence::new(&ranges_req);
    for row in ranges.rows() {
        ranges_evidence.execute(&ranges_req, &ranges, &row.key, |rt, sampled|
            install(rt, sampled, true, true)).unwrap();
    }
    let report = assess_campaign(&ranges_req, &ranges_evidence).unwrap();
    assert_eq!(report.simulation, SimulationVerdict::NotProven);
    for axis in ["допуск/доставка", "допуск/скорость"] {
        assert!(report.findings.iter().any(|f| f.key.object == axis && f.outcome == Outcome::NotAnalyzed));
    }

    for (velocity, delay, all_states) in [
        (false, true, true), (true, false, true), (true, true, false), (true, true, true),
    ] {
        let mut evidence = CampaignEvidence::new(&req);
        for row in plan.rows() {
            if !all_states && row.key != plan.rows()[0].key
                && !matches!(&row.key.sample, sim_task::sampling::SampleKey::Boundary { id } if id == "движение")
            {
                continue;
            }
            evidence
                .execute(&req, &plan, &row.key, |rt, sampled| install(rt, sampled, velocity, delay))
                .unwrap();
            if velocity && delay {
                record_strength(&mut evidence, &req, &row.key);
            }
        }
        let report = assess_campaign(&req, &evidence).unwrap();
        if !all_states {
            assert_eq!(report.simulation, SimulationVerdict::NotProven);
            assert_eq!(report.findings.iter().filter(|f| f.key.class == HazardClass::Execution
                && f.outcome == Outcome::NotAnalyzed).count(), 2);
        } else if velocity && delay {
            assert_eq!(report.simulation, SimulationVerdict::PassedSimulation);
            let moved = report
                .findings
                .iter()
                .find(|finding| {
                    finding.key.class == HazardClass::TaskCriterion
                        && finding.key.run.as_ref().is_some_and(|key|
                            matches!(&key.sample, sim_task::sampling::SampleKey::Boundary { id } if id == "движение"))
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
        SimulationVerdict::NotProven,
        "{:#?}",
        report.findings
    );
    assert_eq!(report.readiness, Readiness::NeedsWork);

    let section = report
        .findings
        .iter()
        .find(|f| f.key.class == HazardClass::StaticStrength)
        .unwrap();
    let [lo, hi] = section.measurement.unwrap();
    assert_eq!(section.outcome, Outcome::NotAnalyzed);
    assert!(section.reason.contains(sim_world::STRENGTH_REQUIRES_BENCH));
    // Независимая формула σ = 6FL/(bh²), запас = 5 МПа − σ.
    let expected = 6.0 * 2.0 * 1.0 / (0.04 * 0.02_f64.powi(2));
    assert_eq!(lo, hi);
    assert!((lo - expected).abs() < 1e-8);
    assert!((section.limit.as_ref().unwrap().upper.unwrap()[1] - lo - 4_250_000.0).abs() < 1e-8);
    let travel = report
        .findings
        .iter()
        .find(|f| f.key.class == HazardClass::TaskCriterion)
        .unwrap();
    // Сумма скоростей полунеявного Euler при F/m = 1 м/с².
    assert!((travel.measurement.unwrap()[1] - 10e-6).abs() < 1e-14);
    assert!(travel.journal.is_some());

    activate(&mut inv, ComponentKind::CriticalMeasurement, "ход");
    strength_bench_check(&mut inv);
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
    let unmeasured = assess_campaign(&req, &evidence).unwrap();
    assert_eq!(unmeasured.simulation, SimulationVerdict::NotProven);
    assert_eq!(unmeasured.readiness, Readiness::NeedsWork);
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
    assert!(evidence.record_measurement(&req, key, build(), strength_signal(),
        source(SourceKind::Estimate, "синтетический ряд"), Some(0.0), &strength_samples()).is_err());
    assert!(evidence.record_measurement(&req, key, Fingerprint::of(b"other"), strength_signal(),
        measured(), Some(0.0), &strength_samples()).is_err());
    assert_eq!(assess_campaign(&req, &evidence).unwrap().simulation, SimulationVerdict::NotProven);
    for row in plan.rows() {
        record_strength(&mut evidence, &req, &row.key);
    }

    let report = assess_campaign(&req, &evidence).unwrap();
    assert_eq!(report.readiness, Readiness::Passed);
    assert!(report.findings.iter().any(|f| f.key.class == HazardClass::StaticStrength
        && f.outcome == Outcome::Pass));
    assert!(report.diagnostics.iter().any(|f| f.key.class == HazardClass::StaticStrength
        && f.outcome == Outcome::NotAnalyzed));
}

#[test]
fn стендовая_прочность_самостоятельна_и_не_отменяет_расчётный_перегруз() {
    let fixture = Fixture::new("strength-bench");
    let plan = fixture.plan_with_sampling("", 0.01, false, &discrete_sampling());
    let mut inventory = inventory();
    strength_bench_check(&mut inventory);
    let Verification::Section { moment_nm, .. } = &mut inventory.checks[0].verification else {
        unreachable!()
    };
    // 20 Н·м при прежнем сечении дают 7,5 МПа. Оба метода
    // сравниваются с одним пределом стенда 5 МПа.
    *moment_nm = [20.0; 2];
    for with_model in [true, false] {
        if !with_model {
            inventory.checks.remove(0);
        }
        let req = derive_requirements(&inventory, &plan).unwrap();
        let mut evidence = CampaignEvidence::new(&req);
        for row in plan.rows() {
            evidence.execute(&req, &plan, &row.key, execute).unwrap();
            record_strength(&mut evidence, &req, &row.key);
        }
        let report = assess_campaign(&req, &evidence).unwrap();
        assert_eq!(report.simulation, if with_model {
            SimulationVerdict::Rejected
        } else {
            SimulationVerdict::PassedSimulation
        });
        let strength: Vec<_> = report.findings.iter()
            .filter(|f| f.key.class == HazardClass::StaticStrength).collect();
        assert_eq!(strength.iter().filter(|f| f.outcome == Outcome::Pass
            && f.measurement == Some([750_000.0; 2])).count(), plan.rows().len());
        assert_eq!(strength.iter().filter(|f| f.outcome == Outcome::Fail
            && f.measurement == Some([7_500_000.0; 2])).count(),
            if with_model { plan.rows().len() } else { 0 });
    }
}

#[test]
fn короткий_замер_не_скрывает_превышение_на_последнем_такте() {
    let fixture = Fixture::new("bench-window");
    let plan = fixture.plan_with_sampling("", 0.01, false, &discrete_sampling());
    let mut inv = inventory();
    activate(&mut inv, ComponentKind::CriticalMeasurement, "ход");
    for (end_ns, expected) in [
        (ROW.end_ns, SimulationVerdict::Rejected),
        (ROW.end_ns - ROW.max_gap_ns, SimulationVerdict::NotProven),
    ] {
        let mut inputs = inv.clone();
        inputs.checks.push(Check {
            class: HazardClass::Measurement,
            object: "ход".into(),
            window: Window { end_ns, ..ROW },
            selector: RunSelector::default(),
            applicability: measured(),
            scope: "Калиброванная линейка в оснастке".into(),
            inputs: vec![],
            // Зазор оснастки 12 мкм: ход 10 мкм из Euler и запас 2 мкм.
            verification: Verification::Measurement(limit(position(), 12e-6)),
        });
        let requirements = derive_requirements(&inputs, &plan).unwrap();
        let mut evidence = CampaignEvidence::new(&requirements);
        for row in plan.rows() {
            evidence.execute(&requirements, &plan, &row.key, execute).unwrap();
            let offset = plan.materialize(&row.key).unwrap().values()[0].value;
            let samples: Vec<_> = (0..=4)
                .map(|tick| MeasurementSample {
                    at_ns: tick * ROW.max_gap_ns,
                    // Сумма Euler плюс контрольное возмущение 5 мкм в последнем отсчёте.
                    value: offset + (tick * (tick + 1) / 2) as f64 * 1e-6
                        + if tick == 4 { 5e-6 } else { 0.0 },
                })
                .collect();
            evidence.record_measurement(
                &requirements, &row.key, build(), position(), measured(), Some(0.0), &samples,
            ).unwrap();
        }
        let report = assess_campaign(&requirements, &evidence).unwrap();
        assert_eq!(report.simulation, expected, "окно 0…{end_ns} нс");
        if expected == SimulationVerdict::Rejected {
            assert!(report.findings.iter().any(|finding| {
                finding.key.class == HazardClass::Measurement && finding.confirmed_hazard
                    && finding.event_ns == Some(ROW.end_ns)
            }));
        } else {
            assert!(report.findings.iter().any(|finding| {
                finding.key.class == HazardClass::Measurement
                    && finding.outcome == Outcome::NotAnalyzed
                    && finding.key.window_ns == Some([end_ns, ROW.end_ns])
            }));
        }
    }
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
    assert!(report.findings.iter().any(|f| f.key.class == HazardClass::TravelJam
        && f.outcome == Outcome::Pass));
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
            .any(|f| f.key.class == HazardClass::StaticStrength && f.outcome == Outcome::NotAnalyzed)
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

#[test]
fn объединение_отклоняет_чужие_требования_и_повтор_строки() {
    let fixture = Fixture::new("merge");
    let plan = fixture.plan("", 3.1e-6, false); // Предел между вторым и третьим тактом.
    let key = &plan.rows()[0].key;
    let req = derive_requirements(&inventory(), &plan).unwrap();
    let mut foreign = inventory();
    foreign.build = Some(Fingerprint::of(b"other build"));
    let foreign = derive_requirements(&foreign, &plan).unwrap();

    let mut combined = CampaignEvidence::new(&req);
    assert!(combined.merge(CampaignEvidence::new(&foreign)).is_err());
    let mut first = CampaignEvidence::new(&req);
    first.execute(&req, &plan, key, execute).unwrap();
    let expected = first.scenario_report(key).unwrap().1;
    combined.merge(first).unwrap();
    let mut duplicate = CampaignEvidence::new(&req);
    duplicate
        .record_unavailable(&req, key, "проверка повтора".into())
        .unwrap();
    assert!(combined.merge(duplicate).is_err());
    assert_eq!(combined.scenario_report(key).unwrap().1, expected);
    assert_eq!(
        assess_campaign(&req, &combined).unwrap().simulation,
        SimulationVerdict::Rejected
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
fn ткань_и_проникновение_закрывает_только_ряд_своей_сборки() {
    let fixture = Fixture::new("measured-screen");
    let plan = fixture.plan("", 0.01, false);
    let key = &plan.rows()[0].key;
    let mut inv = inventory();
    inv.participant = Some(sim_task::interaction::Reach::Reachable);
    // Нулевые показания контрольной сухой и разгруженной оснастки.
    let samples = (0..=4).map(|tick| MeasurementSample {
        at_ns: tick * ROW.max_gap_ns,
        value: 0.0,
    }).collect::<Vec<_>>();
    let checks = [
        (HazardClass::TissueLoad, Signal { name: "давление".into(), unit: "Pa".into() }, 80000.0),
        (HazardClass::Ingress, Signal { name: "вода в защищаемой зоне".into(), unit: "m³".into() }, 0.0),
    ];
    for (class, signal, upper) in &checks {
        inv.checks.push(Check {
            class: *class,
            object: signal.name.clone(),
            window: ROW,
            selector: RunSelector::default(),
            applicability: measured(),
            scope: "сухая и разгруженная контрольная оснастка".into(),
            inputs: vec![],
            // 80 кПа для плоской пробы и нулевой допустимый объём из корпуса этапа 6.
            verification: if *class == HazardClass::TissueLoad {
                Verification::TissueMeasurement {
                    limit: limit(signal.clone(), *upper),
                    // Оснастка допускает весь контакт в записи от 0 до 4 мс.
                    duration: ContactDuration { ns: [0, ROW.end_ns], source: measured(), contact: None },
                }
            } else {
                Verification::Measurement(limit(signal.clone(), *upper))
            },
        });
    }
    for index in inv.checks.len() - checks.len()..inv.checks.len() {
        let mut model = inv.clone();
        let bound = model.checks[index].verification.measurement_limit().unwrap();
        model.checks[index].verification = Verification::Truth(bound.clone());
        assert!(derive_requirements(&model, &plan).is_err());
    }
    let requirements = derive_requirements(&inv, &plan).unwrap();
    let mut evidence = CampaignEvidence::new(&requirements);
    #[derive(Clone)]
    struct Unloaded(Vec<sim_runtime::TruthId>);
    impl sim_runtime::Evaluator for Unloaded {
        fn evaluate(
            &mut self,
            _: &sim_runtime::Truth<'_>,
            _: &mut sim_signal::Rng,
            out: &mut sim_runtime::TruthOut,
        ) -> Result<(), String> {
            for &channel in &self.0 {
                out.write(channel, 0.0); // Модель сухой оснастки без нагрузки на пробу.
            }
            Ok(())
        }
    }
    evidence.execute(&requirements, &plan, key, |rt, sample| {
        let ids = checks.iter().map(|(_, signal, _)| {
            rt.register_truth(&signal.name, &signal.unit)
        }).collect::<Result<Vec<_>, _>>()?;
        rt.register_evaluator("сухая ненагруженная оснастка", rt.physics_rate(), Unloaded(ids))?;
        execute(rt, sample)
    }).unwrap();
    for (_, signal, _) in &checks {
        for (fingerprint, origin) in [
            (Fingerprint::of(b"other"), measured()),
            (build(), source(SourceKind::Derived, "Расчётный ряд")),
        ] {
            assert!(evidence.record_measurement(
                &requirements, key, fingerprint, signal.clone(), origin, Some(0.0), &samples,
            ).is_err());
        }
    }
    let outcome = |evidence: &CampaignEvidence, class| {
        assess_campaign(&requirements, evidence).unwrap().findings.into_iter()
            .filter(|entry| entry.key.class == class && entry.key.level != sim_task::verdict::Level::Bench)
            .map(|entry| entry.outcome)
            .collect::<Vec<_>>()
    };
    for (class, signal, _) in checks {
        assert_eq!(outcome(&evidence, class), [Outcome::NotAnalyzed]);
        evidence.record_measurement(
            &requirements, key, build(), signal, measured(), Some(0.0), &samples,
        ).unwrap();
        assert_eq!(outcome(&evidence, class), [Outcome::Pass]);
    }
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
        inv.checks.retain(|check| check.class != HazardClass::StaticStrength);
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
    strength_bench_check(&mut inv);
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
    record_strength(&mut evidence, &req, key);
    for row in &plan.rows()[1..] {
        evidence.execute(&req, &plan, &row.key, execute).unwrap();
        record_strength(&mut evidence, &req, &row.key);
    }
    assert_eq!(
        assess_campaign(&req, &evidence).unwrap().simulation,
        SimulationVerdict::PassedSimulation
    );

    inv.compiled_rows.clear();
    let req = derive_requirements(&inv, &plan).unwrap();
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
        record_strength(&mut evidence, &req, &row.key);
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

#[test]
fn для_приводной_сборки_каждая_опасность_требует_натурного_ряда() {
    use sim_task::interaction::Reach;
    let fixture = Fixture::new("powered-bench");
    let plan = fixture.plan_with_sampling("", 0.01, false, &discrete_sampling());
    let mut inv = inventory();
    inv.participant = Some(Reach::Reachable);
    activate(&mut inv, ComponentKind::MotionAtParticipant, "опора");
    activate(&mut inv, ComponentKind::CriticalMeasurement, "ход");
    let check = |class, object: &str, verification| Check {
        class,
        object: object.into(),
        selector: RunSelector::default(),
        window: ROW,
        applicability: measured(),
        scope: "Пределы синтетической оснастки в протоколе измерений.".into(),
        inputs: Vec::new(),
        verification,
    };
    let upper = |name: &str, unit: &str, max| {
        limit(Signal { name: name.into(), unit: unit.into() }, max)
    };
    // Численный стенд: граница хода 10 мм, скорость у опоры до 0,5 м/с.
    let speed = upper("qvel[0]", "m/s", 0.5);
    inv.checks.push(check(HazardClass::DangerousMotion, "опора", Verification::Truth(speed.clone())));
    for (class, object, bound) in [
        (HazardClass::Measurement, "ход", limit(position(), 0.01)),
        (HazardClass::TravelJam, "каретка/стержень", limit(position(), 0.01)),
        (HazardClass::DangerousMotion, "опора", speed),
    ] {
        inv.checks.push(check(class, object, Verification::Measurement(bound)));
    }
    // Предел синтетической плоской пробы из корпуса.
    let tissue_limit = upper("давление опоры", "Pa", 80_000.0);
    let tissue = |ns, source| Verification::TissueMeasurement {
        limit: tissue_limit.clone(),
        duration: ContactDuration { ns, source, contact: None },
    };
    let run = |inventory: &ResolvedInventory| {
        let requirements = derive_requirements(inventory, &plan).unwrap();
        let mut evidence = CampaignEvidence::new(&requirements);
        for row in plan.rows() {
            evidence.execute(&requirements, &plan, &row.key, execute).unwrap();
            let signals = inventory.checks.iter().filter_map(|check| {
                check.verification.measurement_limit().map(|limit| limit.signal().clone())
            }).collect::<std::collections::BTreeSet<_>>();
            for signal in signals {
                let samples: Vec<_> = (0..=4).map(|tick| MeasurementSample {
                    at_ns: tick * ROW.max_gap_ns,
                    // Точная стендовая рекурсия хода при F/m=1 м/с².
                    // Остальные каналы — разгруженный стенд.
                    value: match signal.name.as_str() {
                        "qpos[0]" => (tick * (tick + 1) / 2) as f64 * 1e-6,
                        "qvel[0]" => tick as f64 * 0.001,
                        "напряжение изгиба" => 750_000.0, // 6·2 Н·м / (0,04 м·(0,02 м)²).
                        _ => 0.0,
                    },
                }).collect();
                evidence.record_measurement(&requirements, &row.key, build(), signal, measured(), Some(0.0), &samples).unwrap();
            }
        }
        assess_campaign(&requirements, &evidence).unwrap()
    };
    for bad in [
        Verification::Measurement(tissue_limit.clone()),
        tissue([0, ROW.end_ns - ROW.max_gap_ns], measured()),
        tissue([0, ROW.end_ns], source(SourceKind::Estimate, "длительность оценена")),
    ] {
        let mut unqualified = inv.clone();
        unqualified.checks.push(check(HazardClass::TissueLoad, "опора", bad));
        let report = run(&unqualified);
        assert_eq!(report.simulation, SimulationVerdict::NotProven);
        assert!(report.findings.iter().any(|f| f.key.class == HazardClass::TissueLoad
            && f.missing_data.iter().any(|reason| reason.contains("длительности контакта"))));
    }
    inv.checks.push(check(HazardClass::TissueLoad, "опора", tissue([0, ROW.end_ns], measured())));
    let report = run(&inv);
    assert_eq!(report.simulation, SimulationVerdict::NotProven, "{:#?}", report.findings);
    assert_eq!(report.readiness, Readiness::NeedsWork);
    assert!(report.findings.iter().any(|f| f.key.class == HazardClass::StaticStrength
        && f.key.extension.as_deref() == Some("замер") && f.outcome == Outcome::NotAnalyzed));
    inv.checks.push(check(HazardClass::StaticStrength, "каретка/стержень",
        Verification::Measurement(upper("напряжение изгиба", "Pa", 5e6))));
    assert_eq!(run(&inv).readiness, Readiness::Passed);
}

#[test]
fn недозаявленная_нагрузка_не_проходит_даже_ниже_предела_материала() {
    let fixture = Fixture::new("loaded-section");
    let plan = fixture.plan_with_sampling("", 0.01, false, &discrete_sampling());
    let mut inv = inventory();
    let Verification::Section { moment_nm, .. } = &mut inv.checks[0].verification else {
        unreachable!()
    };
    // Заявлен 1 Н·м, в живой оси действуют 2 Н с плечом 1 м.
    *moment_nm = [1.0; 2];
    let requirements = derive_requirements(&inv, &plan).unwrap();
    let mut evidence = CampaignEvidence::new(&requirements);
    for row in plan.rows() {
        evidence.execute(&requirements, &plan, &row.key, execute).unwrap();
    }
    let report = assess_campaign(&requirements, &evidence).unwrap();
    assert_eq!(report.simulation, SimulationVerdict::NotProven);
    for finding in report.findings.iter().filter(|f| f.key.class == HazardClass::StaticStrength) {
        assert_eq!(finding.outcome, Outcome::NotAnalyzed);
        assert_eq!(finding.measurement, Some([750_000.0; 2]));
        assert!(finding.coverage_gaps.iter().any(|reason| reason.contains("превышает заявленную")));
    }
}
