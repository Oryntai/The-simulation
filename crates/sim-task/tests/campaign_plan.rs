use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    path::PathBuf,
};

use sim_runtime::Runtime;
use sim_signal::{Command, Time};
use sim_task::{
    Fault, FaultKind, Fingerprint, Outcome, Signal, Source, SourceKind,
    campaign::{
        Binding, Campaign, CampaignPlan, Disposition, FaultSupport, Grid, RequiredPath,
        ResolvedInputs, ResolvedReference, Target, Window, WorldInputs,
    },
    metrics::{
        FaultObservation, Installation, Interpolation, MetricSpec, MetricValue, Operation, Point,
        Population, Reference, RunSummary, Series, SeriesKind, Worse, clopper_pearson,
        evaluate_metric, evaluate_scenario, summarize,
    },
    sampling::{Parameter, SampleKey, Scope},
};

const SOURCE: &str = "{ kind = \"derived\", reference = \"синтетический стенд\" }";

fn source() -> Source {
    Source {
        kind: SourceKind::Derived,
        reference: "синтетический стенд".into(),
    }
}

struct Files(PathBuf);
impl Files {
    fn new(id: &str) -> Self {
        // У каждого теста свой каталог; PID нужен только для изоляции параллельных Cargo.
        let path =
            std::env::temp_dir().join(format!("sim-task-campaign-{}-{id}", std::process::id()));
        fs::create_dir(&path).unwrap();
        let fixture = Self(path);
        fixture.write("catalog.toml", "version = 1");
        fixture.write(
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
name = "carriage"
file = "body.toml"
"#,
        );
        fixture.write(
            "body.toml",
            r#"
format = 3
name = "Каретка"
[[links]]
name = "block"
[links.joint]
kind = "slide"
axis = [1.0, 0.0, 0.0]
limits_m = [-1.0, 1.0]
damping = 0.0
friction_loss = 0.0
armature = 0.0
efforts = [{ name = "push", dof = 0 }]
[links.mass]
kind = "explicit"
mass_kg = 2.0
com_m = [0.0, 0.0, 0.0]
inertia_kg_m2 = [0.01, 0.02, 0.025, 0.0, 0.0, 0.0]
provenance = { measured = "синтетические масса и инерция блока" }
"#,
        );
        fixture.write("scenarios.toml", &scenario(""));
        fixture
    }

    fn write(&self, file: &str, content: &str) {
        fs::write(self.0.join(file), content).unwrap();
    }

    fn manifest(&self, seed: u64, sampling: &str) {
        self.write(
            "campaign.toml",
            &format!(
                r#"
version = 1
seed = {seed}
world = "world.toml"
catalog = "catalog.toml"
include = ["scenarios.toml"]
{sampling}
"#
            ),
        );
    }

    fn load(&self) -> Result<Campaign, sim_task::Error> {
        Campaign::load(self.0.join("campaign.toml"))
    }
}

impl Drop for Files {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

fn scenario(extra: &str) -> String {
    // Сила 2 Н, масса 2 кг, опыт 4 мс, начало возмущения через 2 мс.
    format!(
        r#"
version = 1
[stations.stand]
experiment = {{ kind = "scenario" }}
[stations.stand.modes.motion]
horizon_ns = 4000000
source = {SOURCE}
steps = [{{ id = "push", target = {{ kind = "effort", name = "carriage/push" }}, at_ns = 0, value = 2.0, unit = "N", source = {SOURCE} }}]
criteria = [{{ id = "travel", window = "whole", limit = {{ kind = "range", signal = {{ name = "qpos[0]", unit = "m" }}, min = 0.0, max = 0.1, source = {SOURCE} }} }}]
faults = [
    {{ fault = {{ node = "a", kind = "wire_open" }}, at_ns = 2000000, max_lag_ns = 0, window = "whole", source = {SOURCE} }},
    {{ fault = {{ node = "b", kind = "wire_open" }}, at_ns = 2000000, max_lag_ns = 0, window = "whole", source = {SOURCE} }}
]
combinations = [{{ path = "line", faults = [{{node = "b", kind = "wire_open"}}, {{node = "a", kind = "wire_open"}}], source = {SOURCE} }}]
metrics = [{{ id = "end", signal = {{ name = "qpos[0]", unit = "m" }}, series = "truth", reference = {{ kind = "constant", value = 0.0, unit = "m" }}, window = "whole", interpolation = "linear", operation = {{ kind = "absolute", at_ns = 4000000 }}, worse = "higher", source = {SOURCE} }}]
{extra}
[stations.stand.modes.motion.windows.whole]
start_ns = 0
end_ns = 4000000
max_gap_ns = 1000000
"#
    )
}

fn axis(
    id: &str,
    parameter: &str,
    scope: &str,
    unit: &str,
    nominal: f64,
    min: f64,
    max: f64,
) -> String {
    format!(
        "{{ id = \"{id}\", parameter = {parameter}, scope = \"{scope}\", unit = \"{unit}\", nominal = {nominal:?}, min = {min:?}, max = {max:?}, source = {SOURCE} }}"
    )
}

fn sampling(independence: &str, specimens: u32, repeats: u32) -> String {
    let mass = axis(
        "mass",
        "{ kind = \"mass\", body = \"carriage/block\" }",
        "batch",
        "kg",
        2.0,
        1.0,
        3.0,
    );
    let offset = axis(
        "offset",
        "{ kind = \"placement_offset\", placement = \"carriage\", axis = 0 }",
        "specimen",
        "m",
        0.0,
        0.0,
        0.1,
    );
    let wind = axis(
        "wind",
        "{ kind = \"wind\", axis = 0 }",
        "run",
        "m/s",
        0.0,
        0.0,
        2.0,
    );
    format!(
        r#"
[sampling]
axes = [
    {mass},
    {offset},
    {wind}
]
boundaries = [{{ id = "edge", values = {{ mass = 3.0, offset = 0.1 }}, source = {SOURCE} }}]
[sampling.probability]
batches = 2
specimens = {specimens}
repeats = {repeats}
confidence = 0.95
independence = "{independence}"
source = {SOURCE}
batch = {{ source = {SOURCE}, alternatives = [
    {{ id = "all", weight = 1.0, values = {{ mass = {{ kind = "uniform", min = 1.0, max = 3.0, factor = "batch-q" }} }} }}
] }}
specimen = [
    {{ batch = "all", law = {{ source = {SOURCE}, alternatives = [
        {{ id = "a", weight = 1.0, values = {{ offset = {{ kind = "uniform", min = 0.0, max = 0.05, factor = "specimen-q" }} }} }},
        {{ id = "b", weight = 1.0, values = {{ offset = {{ kind = "uniform", min = 0.05, max = 0.1, factor = "specimen-q" }} }} }}
    ] }} }}
]
run = [
    {{ batch = "all", specimen = "a", law = {{ source = {SOURCE}, alternatives = [
        {{ id = "r", weight = 1.0, values = {{ wind = {{ kind = "uniform", min = 0.0, max = 1.0, factor = "run-q" }} }} }}
    ] }} }},
    {{ batch = "all", specimen = "b", law = {{ source = {SOURCE}, alternatives = [
        {{ id = "r", weight = 1.0, values = {{ wind = {{ kind = "uniform", min = 1.0, max = 2.0, factor = "batch-q", from = "batch" }} }} }}
    ] }} }}
]
"#
    )
}

fn inventory(campaign: &Campaign, extra_modes: bool) -> ResolvedInputs {
    let grids = [("coarse", 1000), ("fine", 2000)]
        .map(|(id, hz)| Grid {
            id: id.into(),
            hz,
            source: source(),
        })
        .to_vec();
    let available = |name: &str| FaultSupport::Available {
        channel: name.into(),
        resource: "line-state".into(),
    };
    let wire = |node: &str| Fault {
        node: node.into(),
        kind: FaultKind::WireOpen,
    };
    let mut paths = vec![RequiredPath {
        station: "stand".into(),
        mode: "motion".into(),
        path: "line".into(),
        source: source(),
        grids,
        unavailable: None,
        combinations: vec![],
        faults: BTreeMap::from([
            (wire("a"), available("a/state")),
            (wire("b"), available("b/state")),
            (
                wire("no_schedule"),
                FaultSupport::Available {
                    channel: "c/state".into(),
                    resource: "other".into(),
                },
            ),
            (
                wire("absent"),
                FaultSupport::Inapplicable {
                    reason: "инвентарь стенда".into(),
                    basis: source(),
                },
            ),
            (
                wire("unknown"),
                FaultSupport::Unsupported {
                    reason: "нет адаптера".into(),
                },
            ),
        ]),
    }];
    if extra_modes {
        paths.push(RequiredPath {
            mode: "missing".into(),
            ..paths[0].clone()
        });
    }
    let world = WorldInputs::load(campaign.world_path()).unwrap();
    ResolvedInputs {
        world,
        paths,
        references: campaign
            .references()
            .map(|r| {
                (
                    r.path.clone(),
                    ResolvedReference {
                        kind: r.kind,
                        file_fingerprint: r.file_fingerprint,
                        graph_fingerprint: Fingerprint::of(
                            "синтетический каталог и профили".as_bytes(),
                        ),
                    },
                )
            })
            .collect(),
        targets: BTreeMap::from([(
            Target::Effort {
                name: "carriage/push".into(),
            },
            Binding {
                unit: "N".into(),
                resource: "slide-force".into(),
                unavailable: None,
            },
        )]),
        signals: BTreeSet::from([Signal {
            name: "qpos[0]".into(),
            unit: "m".into(),
        }]),
        model_parameters: BTreeMap::new(),
        initial_velocity: false,
    }
}

fn near(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "получено {actual:.17}, ожидалось {expected:.17}"
    );
}

#[test]
fn full_matrix_and_samples_are_order_invariant() {
    let files = Files::new("matrix");
    let sampling = sampling("unspecified", 2, 2);
    files.manifest(7, &sampling);
    let campaign = files.load().unwrap();
    let mut inventory = inventory(&campaign, true);
    let plan = campaign.plan(&inventory).unwrap();
    // (7 штатных/отказных сочетаний с режимом + 6 без) × 2 сетки × (2 детерминированных + 8 случайных).
    assert_eq!(plan.rows().len(), 260);
    let mut expected = Vec::new();
    let mut samples = vec![
        SampleKey::Nominal,
        SampleKey::Boundary { id: "edge".into() },
    ];
    for batch in 0..2 {
        for specimen in 0..2 {
            for repeat in 0..2 {
                samples.push(SampleKey::Random {
                    batch,
                    specimen,
                    repeat,
                });
            }
        }
    }
    for mode in ["missing", "motion"] {
        let mut faults = vec![
            vec![],
            vec!["a"],
            vec!["absent"],
            vec!["b"],
            vec!["no_schedule"],
            vec!["unknown"],
        ];
        if mode == "motion" {
            faults.push(vec!["a", "b"]);
        }
        for set in &faults {
            for sample in &samples {
                for grid in ["coarse", "fine"] {
                    expected.push((mode, set.clone(), sample, grid));
                }
            }
        }
    }
    expected.sort();
    let actual: Vec<_> = plan
        .rows()
        .iter()
        .map(|r| {
            assert_eq!((&*r.key.station, &*r.key.path), ("stand", "line"));
            (
                &*r.key.mode,
                r.key
                    .faults
                    .iter()
                    .map(|f| f.node.as_str())
                    .collect::<Vec<_>>(),
                &r.key.sample,
                &*r.key.grid,
            )
        })
        .collect();
    assert_eq!(actual, expected);
    assert_eq!(
        plan.rows()
            .iter()
            .filter(|r| matches!(r.disposition, Disposition::Ready))
            .count(),
        60
    );
    assert_eq!(
        plan.rows()
            .iter()
            .filter(|r| matches!(r.disposition, Disposition::Inapplicable { .. }))
            .count(),
        40
    );
    let find = |sample: &SampleKey, grid: &str| {
        plan.rows()
            .iter()
            .find(|r| {
                r.key.mode == "motion"
                    && r.key.faults.is_empty()
                    && r.key.sample == *sample
                    && r.key.grid == grid
            })
            .unwrap()
    };
    let materialize = |key: SampleKey, grid: &str| plan.materialize(&find(&key, grid).key).unwrap();
    let nominal = materialize(SampleKey::Nominal, "coarse");
    let edge = materialize(SampleKey::Boundary { id: "edge".into() }, "coarse");
    assert_eq!(
        nominal.values().iter().map(|v| v.value).collect::<Vec<_>>(),
        [2.0, 0.0, 0.0]
    );
    assert_eq!(
        edge.values().iter().map(|v| v.value).collect::<Vec<_>>(),
        [3.0, 0.1, 0.0]
    );
    let compiled = edge.world.compile().unwrap();
    let sim = sim_mujoco::Sim::from_xml(&compiled.mjcf).unwrap();
    // Умножение массы на 3/2 при фиксированной геометрии так же масштабирует тензор.
    near(sim.total_mass(), 3.0, 1e-14);
    let mut inertia = sim.body_inertia(1);
    inertia.sort_by(f64::total_cmp);
    for (actual, manual) in inertia.into_iter().zip([0.015, 0.03, 0.0375]) {
        near(actual, manual, 1e-14);
    }
    near(sim.body_pose(1).0[0], 0.1, 1e-14);
    let random = |batch, specimen, repeat| SampleKey::Random {
        batch,
        specimen,
        repeat,
    };
    let a = materialize(random(0, 0, 0), "coarse");
    let b = materialize(random(0, 1, 0), "coarse");
    let c = materialize(random(1, 0, 0), "coarse");
    let repeat = materialize(random(0, 0, 1), "coarse");
    let fine = materialize(random(0, 0, 0), "fine");
    let bits = |s: &sim_task::campaign::SampledInputs| {
        s.values()
            .iter()
            .map(|a| a.value.to_bits())
            .collect::<Vec<_>>()
    };
    assert_eq!(a.sample.seeds, fine.sample.seeds);
    assert_eq!(bits(&a), bits(&fine));
    assert_eq!(bits(&a)[0], bits(&b)[0]);
    assert_ne!(bits(&a)[0], bits(&c)[0]);
    assert_ne!(bits(&a)[1], bits(&b)[1]);
    assert_eq!(bits(&a)[..2], bits(&repeat)[..2]);
    let mut conditional = 0;
    for key in samples {
        let s = materialize(key, "coarse");
        match s
            .sample
            .alternatives
            .get(&Scope::Specimen)
            .map(String::as_str)
        {
            Some("a") => {
                assert!((0.0..0.05).contains(&s.values()[1].value));
                assert!((0.0..1.0).contains(&s.values()[2].value));
                conditional += 1;
            }
            Some("b") => {
                assert!((0.05..0.1).contains(&s.values()[1].value));
                near(
                    s.values()[2].value,
                    1.0 + (s.values()[0].value - 1.0) / 2.0,
                    1e-15,
                );
                conditional += 1;
            }
            _ => (),
        }
    }
    assert_eq!(conditional, 8);
    inventory.paths.reverse();
    for path in &mut inventory.paths {
        path.grids.reverse();
    }
    let reordered = campaign.plan(&inventory).unwrap();
    assert_eq!(plan.fingerprint(), reordered.fingerprint());
    for row in plan.rows().iter().rev() {
        let a = plan.materialize(&row.key).unwrap();
        let b = reordered.materialize(&row.key).unwrap();
        assert_eq!(
            (a.fingerprint, a.sample.seeds.clone(), bits(&a)),
            (b.fingerprint, b.sample.seeds.clone(), bits(&b))
        );
    }
    files.manifest(8, &sampling);
    let other = files.load().unwrap().plan(&inventory).unwrap();
    assert_ne!(bits(&a)[0], bits(&other.materialize(&a.key).unwrap())[0]);
}

#[test]
fn malformed_graph_and_unselected_overlays_are_rejected() {
    let files = Files::new("errors");
    files.manifest(7, "");
    let campaign = files.load().unwrap();
    let mut resolved = inventory(&campaign, false);
    resolved.references.clear();
    assert!(
        campaign
            .plan(&resolved)
            .unwrap_err()
            .to_string()
            .contains("графа")
    );
    files.write("scenarios.toml", "version = 1\ninclude = ['campaign.toml']");
    assert!(files.load().unwrap_err().to_string().contains("цикл"));
    files.write("scenarios.toml", &scenario("typo = 1"));
    assert!(files.load().unwrap_err().to_string().contains("typo"));
    files.write(
        "scenarios.toml",
        &scenario("").replace("value = 2.0", "value = inf"),
    );
    assert!(
        files
            .load()
            .unwrap_err()
            .to_string()
            .contains("steps[0].value")
    );
    files.write("scenarios.toml", &scenario(""));
    files.manifest(
        7,
        &sampling("unspecified", 2, 2).replace("min = 1.0, max = 2.0", "min = nan, max = 2.0"),
    );
    let error = files.load().unwrap_err().to_string();
    assert!(
        error.contains("probability.run[1]") && error.contains("wind.min"),
        "{error}"
    );
    files.manifest(7, "");
    files.write(
        "scenarios.toml",
        &scenario("").replace("unit = \"N\"", "unit = \"V\""),
    );
    let campaign = files.load().unwrap();
    assert!(
        campaign
            .plan(&inventory(&campaign, false))
            .unwrap_err()
            .to_string()
            .contains("steps[0]")
    );
    files.write("scenarios.toml", &scenario("").replace("steps = [", &format!(
        "steps = [{{ id = \"alias\", target = {{ kind = \"port\", name = \"alias\" }}, at_ns = 0, value = 2.0, unit = \"N\", source = {SOURCE} }},"
    )));
    let campaign = files.load().unwrap();
    let mut resolved = inventory(&campaign, false);
    resolved.targets.insert(
        Target::Port {
            name: "alias".into(),
        },
        Binding {
            unit: "N".into(),
            resource: "slide-force".into(),
            unavailable: None,
        },
    );
    assert!(
        campaign
            .plan(&resolved)
            .unwrap_err()
            .to_string()
            .contains("интервалы действий на одном ресурсе")
    );
    resolved.paths.clear();
    assert!(
        campaign
            .plan(&resolved)
            .unwrap_err()
            .to_string()
            .contains("пустой список")
    );
    files.write("scenarios.toml", &scenario(&format!(
        "initial = [{{ parameter = {{kind = \"joint_velocity\", joint = \"carriage/block\", dof = 0}}, unit = \"m/s\", value = 0.5, source = {SOURCE} }}]"
    )));
    let campaign = files.load().unwrap();
    let plan = campaign.plan(&inventory(&campaign, false)).unwrap();
    assert!(matches!(
        plan.rows()[0].disposition,
        Disposition::NotAnalyzed { .. }
    ));
    assert_eq!(
        plan.materialize(&plan.rows()[0].key)
            .unwrap()
            .settings
            .initial_velocity["carriage/block"][&0],
        0.5
    );
    let position = axis(
        "start",
        "{ kind = \"joint_position\", joint = \"carriage/block\" }",
        "run",
        "m",
        0.0,
        0.0,
        2.0,
    );
    files.manifest(7, &format!("[sampling]\naxes = [{position}]\nboundaries = [{{id=\"edge\",values={{start=2.0}},source={SOURCE}}}]"));
    let campaign = files.load().unwrap();
    assert!(
        campaign
            .plan(&inventory(&campaign, false))
            .unwrap_err()
            .to_string()
            .contains("начальная координата")
    );
}

fn install(inputs: &sim_task::campaign::SampledInputs) -> Installation {
    Installation {
        fingerprint: inputs.fingerprint,
        run_seed: inputs.sample.seeds.run,
        physics_hz: inputs.world.world.physics_hz,
        values: inputs
            .values()
            .iter()
            .map(|v| (v.id.clone(), v.value))
            .collect(),
    }
}

fn run_carriage(inputs: &sim_task::campaign::SampledInputs, force: f64, until: u64) -> Runtime {
    let mut rt = Runtime::load(inputs.world.compile().unwrap()).unwrap();
    rt.record_joint_state("состояние").unwrap();
    let channel = rt.channel("carriage/push").unwrap();
    rt.act(&Command {
        targets: vec![(channel, force)],
    })
    .unwrap();
    rt.run_until(Time(until)).unwrap();
    rt
}

#[test]
fn live_carriage_uses_actual_commands_and_keeps_filtered_and_unconfirmed_rows_open() {
    let fixture = Files::new("live");
    fixture.manifest(7, "");
    let campaign = fixture.load().unwrap();
    let plan = campaign.plan(&inventory(&campaign, false)).unwrap();
    let nominal = &plan.rows()[0];
    let inputs = plan.materialize(&nominal.key).unwrap();
    let rt = run_carriage(&inputs, 2.0, 4_000_000);
    let report = evaluate_scenario(nominal, &inputs, rt.log(), install(&inputs), &[], &[]).unwrap();
    assert_eq!(report.outcome, Outcome::Pass);
    let MetricValue::Value { value: travel } = report.metrics["end"] else {
        panic!("нет хода каретки");
    };
    // Полунеявный Euler: при a = F/m = 1 ход за 4 шага равен a·h²·(1+2+3+4).
    near(travel, 10e-6, 1e-14);
    let short = run_carriage(&inputs, 2.0, 3_000_000);
    let short_report =
        evaluate_scenario(nominal, &inputs, short.log(), install(&inputs), &[], &[]).unwrap();
    assert!(!short_report.completed);
    assert!(matches!(
        short_report.metrics["end"],
        MetricValue::NotAnalyzed { .. }
    ));
    assert_eq!(short_report.outcome, Outcome::NotAnalyzed);
    let wrong_command = run_carriage(&inputs, 1.0, 4_000_000);
    assert_eq!(
        evaluate_scenario(
            nominal,
            &inputs,
            wrong_command.log(),
            install(&inputs),
            &[],
            &[]
        )
        .unwrap()
        .outcome,
        Outcome::NotAnalyzed
    );

    let selected = plan
        .rows()
        .iter()
        .find(|r| r.key.faults.len() == 1 && r.key.faults[0].node == "a")
        .unwrap();
    let fault_input = plan.materialize(&selected.key).unwrap();
    let base = run_carriage(&fault_input, 2.0, 4_000_000);
    let planned_fault = &selected.faults[0];
    assert!(
        FaultObservation::from_state(planned_fault, base.log())
            .unwrap()
            .is_none()
    );
    let no_fault = evaluate_scenario(
        selected,
        &fault_input,
        base.log(),
        install(&fault_input),
        &[],
        &[],
    )
    .unwrap();
    let first = summarize(&plan, &[report.clone(), no_fault.clone()]).unwrap();
    assert_eq!(first.rows.len(), 14);
    assert_eq!(first.rows[0].outcome, Some(Outcome::Pass));
    assert!(first.rows.iter().any(|r| r.key == selected.key
        && r.completed
        && r.outcome == Some(Outcome::NotAnalyzed)
        && r.reasons.iter().any(|reason| reason.contains("инъекция"))));
    assert!(
        first
            .rows
            .iter()
            .any(|r| r.reasons.iter().any(|reason| reason.contains("фильтрации")))
    );
    assert!(first.rows.iter().any(|r| {
        r.reasons
            .iter()
            .any(|reason| reason.contains("конфликтуют"))
    }));

    #[derive(Clone)]
    struct State(sim_runtime::TruthId, u64);
    impl sim_runtime::Evaluator for State {
        fn evaluate(
            &mut self,
            truth: &sim_runtime::Truth<'_>,
            _: &mut sim_signal::Rng,
            out: &mut sim_runtime::TruthOut,
        ) -> Result<(), String> {
            out.write(self.0, u8::from(truth.time().0 >= self.1).into());
            Ok(())
        }
    }
    let fault_report = |time, channel: &str| {
        let mut rt = Runtime::load(fault_input.world.compile().unwrap()).unwrap();
        rt.record_joint_state("состояние").unwrap();
        let signal = rt.register_truth(channel, "1").unwrap();
        rt.register_evaluator("состояние разрыва", rt.physics_rate(), State(signal, time))
            .unwrap();
        rt.act(&Command {
            targets: vec![(rt.channel("carriage/push").unwrap(), 2.0)],
        })
        .unwrap();
        rt.run_until(Time(4_000_000)).unwrap();
        let facts: Vec<_> = FaultObservation::from_state(planned_fault, rt.log())
            .unwrap()
            .into_iter()
            .collect();
        evaluate_scenario(
            selected,
            &fault_input,
            rt.log(),
            install(&fault_input),
            &[],
            &facts,
        )
        .unwrap()
    };
    let at_requested = fault_report(2_000_000, "a/state");
    assert_eq!(at_requested.faults[0].applied_ns, 2_000_000);
    let stats = summarize(&plan, &[at_requested.clone()]).unwrap();
    assert_eq!(
        stats
            .rows
            .iter()
            .find(|row| row.key == selected.key)
            .unwrap()
            .outcome,
        Some(Outcome::Pass)
    );
    for misplaced in [
        fault_report(3_000_000, "a/state"),
        fault_report(2_000_000, "b/state"),
    ] {
        let stats = summarize(&plan, &[misplaced]).unwrap();
        assert_eq!(
            stats
                .rows
                .iter()
                .find(|row| row.key == selected.key)
                .unwrap()
                .outcome,
            Some(Outcome::NotAnalyzed)
        );
    }
    let mut wrong_installation = report.clone();
    wrong_installation.installation.as_mut().unwrap().run_seed ^= 1;
    assert_eq!(
        summarize(&plan, &[wrong_installation]).unwrap().rows[0].outcome,
        Some(Outcome::NotAnalyzed)
    );
    assert!(summarize(&plan, &[report.clone(), report]).is_err());

    let delayed = scenario("")
        .replace("at_ns = 0, value", "at_ns = 0, max_lag_ns = 1000000, value")
        .replace(
            "kind = \"constant\", value = 0.0",
            "kind = \"step\", stimulus = \"push\", before = 0.0, after = 1.0",
        )
        .replace(
            "kind = \"absolute\", at_ns = 4000000",
            "kind = \"absolute\", at_ns = 0",
        );
    fixture.write("scenarios.toml", &delayed);
    let campaign = fixture.load().unwrap();
    let plan_with_lag = campaign.plan(&inventory(&campaign, false)).unwrap();
    let row = &plan_with_lag.rows()[0];
    let inputs = plan_with_lag.materialize(&row.key).unwrap();
    let mut rt = Runtime::load(inputs.world.compile().unwrap()).unwrap();
    rt.record_joint_state("состояние").unwrap();
    rt.run_until(Time(1_000_000)).unwrap();
    rt.act(&Command {
        targets: vec![(rt.channel("carriage/push").unwrap(), 2.0)],
    })
    .unwrap();
    rt.run_until(Time(4_000_000)).unwrap();
    let delayed_report =
        evaluate_scenario(row, &inputs, rt.log(), install(&inputs), &[], &[]).unwrap();
    assert_eq!(delayed_report.outcome, Outcome::Pass);
    // При фактическом воздействии на 1 мс опора в нуле ещё равна 0 м.
    // Подстановка запланированного времени дала бы ошибку 1 м.
    assert_eq!(delayed_report.metrics["end"], value(0.0));

    fixture.write(
        "scenarios.toml",
        &scenario("").replace("max = 0.1, source", "max = 0.000005, source"),
    );
    let campaign = fixture.load().unwrap();
    let failing = campaign.plan(&inventory(&campaign, false)).unwrap();
    let inputs = failing.materialize(&failing.rows()[0].key).unwrap();
    let rt = run_carriage(&inputs, 2.0, 4_000_000);
    let result = evaluate_scenario(
        &failing.rows()[0],
        &inputs,
        rt.log(),
        install(&inputs),
        &[],
        &[],
    )
    .unwrap();
    assert_eq!(result.outcome, Outcome::Fail);
    assert_eq!(
        summarize(&failing, &[result]).unwrap().rows[0].outcome,
        Some(Outcome::Fail)
    );
}

fn value(value: f64) -> MetricValue {
    MetricValue::Value { value }
}

#[test]
fn metrics_match_piecewise_analytic_step_and_distinguish_missing_data() {
    let signal = Signal {
        name: "ход".into(),
        unit: "m".into(),
    };
    // Моменты 0,1,2,3,4,5 нс, приращение цели 2 м в t=1, отклик по отрезкам.
    let series = Series {
        signal: signal.clone(),
        points: [0.0, 0.0, 1.0, 3.0, 2.0, 2.0]
            .into_iter()
            .enumerate()
            .map(|(at_ns, value)| Point {
                at_ns: at_ns as u64,
                value,
                valid: true,
            })
            .collect(),
    };
    let window = Window {
        start_ns: 0,
        end_ns: 5,
        max_gap_ns: 1,
    };
    let spec = |operation| MetricSpec {
        id: "метрика".into(),
        signal: signal.clone(),
        series: SeriesKind::Truth,
        reference: Reference::Step {
            stimulus: "ступень".into(),
            before: 0.0,
            after: 2.0,
            unit: "m".into(),
        },
        window: "весь прогон".into(),
        interpolation: Interpolation::Linear,
        operation,
        worse: Worse::Higher,
        source: source(),
        limit: None,
    };
    let measure = |op| evaluate_metric(&spec(op), window, &series, None, Some(1)).unwrap();
    assert_eq!(measure(Operation::Absolute { at_ns: 2 }).value, value(1.0));
    let MetricValue::Value { value: rms } = measure(Operation::TrackingRms).value else {
        panic!("нет СКО");
    };
    // Первообразная на трёх ненулевых участках: 7/3, 1/3 и 1/3. Среднее за 5 нс равно 3/5.
    near(rms, (3.0_f64 / 5.0).sqrt(), 1e-15);
    assert_eq!(measure(Operation::Overshoot).value, value(1.0));
    assert_eq!(measure(Operation::OvershootFraction).value, value(0.5));
    let settling = measure(Operation::Settling {
        band: 0.5,
        deadline_ns: Some(3),
    });
    // Последний вход в полосу на t=3.5, округление вверх даёт t=4, задержка от t=1 равна 3 нс.
    assert_eq!(settling.value, value(3.0));
    assert_eq!(settling.outcome, Some(Outcome::Pass));

    let mut unsettled = series.clone();
    unsettled
        .points
        .iter_mut()
        .skip(3)
        .for_each(|p| p.value = 3.0);
    let policy = spec(Operation::Settling {
        band: 0.5,
        deadline_ns: Some(4),
    });
    let censored = evaluate_metric(&policy, window, &unsettled, None, Some(1)).unwrap();
    assert_eq!(censored.value, MetricValue::Censored { lower_bound: 4.0 });
    assert_eq!(censored.outcome, Some(Outcome::Fail));
    let mut missing = series.clone();
    missing.points.remove(3);
    let incomplete = evaluate_metric(&policy, window, &missing, None, Some(1)).unwrap();
    assert!(matches!(incomplete.value, MetricValue::NotAnalyzed { .. }));
    assert_eq!(incomplete.outcome, Some(Outcome::NotAnalyzed));
    missing.points[0].valid = false;
    missing.points[0].value = f64::NAN;
    assert!(evaluate_metric(&policy, window, &missing, None, Some(1)).is_err());
    assert!(matches!(
        evaluate_metric(&policy, window, &series, None, None)
            .unwrap()
            .value,
        MetricValue::NotAnalyzed { .. }
    ));
    let mut zero = spec(Operation::OvershootFraction);
    zero.reference = Reference::Step {
        stimulus: "ступень".into(),
        before: 2.0,
        after: 2.0,
        unit: "m".into(),
    };
    assert!(matches!(
        evaluate_metric(&zero, window, &series, None, Some(1))
            .unwrap()
            .value,
        MetricValue::NotAnalyzed { .. }
    ));
    let mut negative = spec(Operation::OvershootFraction);
    negative.reference = Reference::Step {
        stimulus: "ступень".into(),
        before: 0.0,
        after: -2.0,
        unit: "m".into(),
    };
    let mut descending = series.clone();
    descending
        .points
        .iter_mut()
        .for_each(|p| p.value = -p.value);
    assert_eq!(
        evaluate_metric(&negative, window, &descending, None, Some(1))
            .unwrap()
            .value,
        value(0.5)
    );
}

fn fixtures_for_statistics(plan: &CampaignPlan, count: usize) -> Vec<RunSummary> {
    plan.rows()
        .iter()
        .filter(|r| {
            r.key.faults.is_empty()
                && r.key.grid == "coarse"
                && matches!(r.key.sample, SampleKey::Random { .. })
        })
        .take(count)
        .enumerate()
        .map(|(i, row)| RunSummary {
            key: row.key.clone(),
            installation: Some(install(&plan.materialize(&row.key).unwrap())),
            observed_through_ns: 4_000_000,
            completed: true,
            criteria_complete: true,
            faults: vec![],
            metrics: BTreeMap::from([("end".into(), value((i + 1) as f64))]),
            outcome: if i % 2 == 0 {
                Outcome::Pass
            } else {
                Outcome::Fail
            },
        })
        .collect()
}

fn fixed_probability(batches: u32) -> String {
    let law = format!(
        "{{ source = {SOURCE}, alternatives = [{{ id = 'only', weight = 1.0, values = {{}} }}] }}"
    );
    format!(
        r#"
[sampling.probability]
batches = {batches}
specimens = 1
repeats = 1
confidence = 0.95
independence = "batches"
source = {SOURCE}
batch = {law}
specimen = [{{ batch = "only", law = {law} }}]
run = [{{ batch = "only", specimen = "only", law = {law} }}]
"#
    )
}

#[test]
fn confidence_matches_direct_tails_and_statistics_preserve_independence_and_denominators() {
    let confidence = 0.95;
    let all = clopper_pearson(10, 10, confidence).unwrap();
    let none = clopper_pearson(0, 10, confidence).unwrap();
    near(all.low, 0.025_f64.powf(0.1), 1e-15);
    near(none.high, 1.0 - 0.025_f64.powf(0.1), 1e-15);
    assert_eq!((none.low, all.high), (0.0, 1.0));
    let inner = clopper_pearson(2, 5, confidence).unwrap();
    // Прямые члены для n=5, k=2. Коэффициенты — строка треугольника Паскаля.
    let p = inner.low;
    let q = 1.0 - p;
    near(
        10.0 * p.powi(2) * q.powi(3)
            + 10.0 * p.powi(3) * q.powi(2)
            + 5.0 * p.powi(4) * q
            + p.powi(5),
        0.025,
        1e-15,
    );
    let p = inner.high;
    let q = 1.0 - p;
    near(
        q.powi(5) + 5.0 * p * q.powi(4) + 10.0 * p.powi(2) * q.powi(3),
        0.025,
        1e-15,
    );
    assert!(clopper_pearson(1, 0, confidence).is_err());
    assert!(clopper_pearson(0, 1, f64::INFINITY).is_err());

    let files = Files::new("statistics");
    files.manifest(7, &fixed_probability(20));
    let campaign = files.load().unwrap();
    let plan = campaign.plan(&inventory(&campaign, false)).unwrap();
    let mut runs = fixtures_for_statistics(&plan, 20);
    let select = |stats: sim_task::metrics::CampaignStats| {
        stats
            .series
            .into_iter()
            .find(|series| {
                series.key.faults.is_empty()
                    && series.key.grid == "coarse"
                    && series.key.population == Population::Random
            })
            .unwrap()
    };
    let stats = select(summarize(&plan, &runs).unwrap());
    assert_eq!(
        (
            stats.planned,
            stats.completed,
            stats.passed,
            stats.failed,
            stats.open
        ),
        (20, 20, 10, 10, 0)
    );
    assert_eq!(stats.classified_success_rate, Some(0.5));
    assert_eq!(
        stats.confidence.interval,
        Some(clopper_pearson(10, 20, 0.95).unwrap())
    );
    let metric = &stats.metrics["end"];
    // Отсортированный ряд 1..20: средняя пара 10,11, ранг P95 равен 19.
    assert_eq!(
        (metric.median, metric.p95, metric.measured),
        (Some(10.5), Some(19.0), 20)
    );
    let worst = metric.worst.as_ref().unwrap();
    assert_eq!(worst.key, runs[19].key);
    assert_eq!(
        worst.seeds.run,
        runs[19].installation.as_ref().unwrap().run_seed
    );
    let MetricValue::Value {
        value: repeatability,
    } = metric.repeatability
    else {
        panic!("нет повторяемости");
    };
    // Дисперсия 1..n с делителем n−1 равна n(n+1)/12.
    near(
        repeatability,
        10.5 + 3.0 * (20.0_f64 * 21.0 / 12.0).sqrt(),
        1e-13,
    );
    let previous = runs.pop().unwrap();
    let partial = select(summarize(&plan, &runs).unwrap());
    assert_eq!(
        (partial.planned, partial.classified, partial.open),
        (20, 19, 1)
    );
    assert_eq!(partial.classified_success_rate, Some(10.0 / 19.0));
    assert_eq!(partial.applicable_success_bounds.unwrap().high, 11.0 / 20.0);
    assert!(partial.confidence.interval.is_none());
    let mut tied = previous;
    tied.metrics.insert("end".into(), value(19.0));
    runs.push(tied);
    assert_eq!(
        select(summarize(&plan, &runs).unwrap()).metrics["end"]
            .worst
            .as_ref()
            .unwrap()
            .key,
        runs[18].key
    );

    files.manifest(7, &sampling("unspecified", 2, 2));
    let campaign = files.load().unwrap();
    let plan = campaign.plan(&inventory(&campaign, false)).unwrap();
    let complete = fixtures_for_statistics(&plan, 8);
    let dependent = select(summarize(&plan, &complete).unwrap());
    assert_eq!(dependent.completed, 8);
    assert!(dependent.confidence.interval.is_none());
    assert!(matches!(
        dependent.metrics["end"].repeatability,
        MetricValue::NotAnalyzed { .. }
    ));

    files.manifest(7, &sampling("within_batch", 2, 1));
    let campaign = files.load().unwrap();
    let plan = campaign.plan(&inventory(&campaign, false)).unwrap();
    let rows = fixtures_for_statistics(&plan, 4);
    let report = summarize(&plan, &rows).unwrap();
    assert!(select(report.clone()).confidence.interval.is_none());
    let batches: Vec<_> = report
        .series
        .iter()
        .filter(|s| {
            s.key.grid == "coarse"
                && s.key.faults.is_empty()
                && matches!(s.key.population, Population::Batch { .. })
        })
        .collect();
    assert_eq!(batches.len(), 2);
    for s in batches {
        assert_eq!(
            s.confidence.interval,
            Some(clopper_pearson(1, 2, 0.95).unwrap())
        );
        assert!(matches!(
            s.metrics["end"].repeatability,
            MetricValue::NotAnalyzed { .. }
        ));
    }
}

#[test]
fn overlays_reach_mujoco_and_installable_model_settings() {
    use sim_task::campaign::ModelParameter;
    use sim_world::{ContactPair, ContactSettings};
    let files = Files::new("overlay");
    // Три параметра каталога датчика и один параметр проводки подаются разрешителем.
    let data = fs::read_to_string(files.0.join("body.toml")).unwrap();
    files.write(
        "body.toml",
        &(data
            + r#"
[[sensors]]
name = "probe"
part = "measurement"
joint = "block"
[ sensors.params ]
noise_std = 0.0
bias = 0.0
latency_s = 0.0
[[links.collision]]
shape = { kind = "box", size_m = [0.2, 0.2, 0.2] }
"#),
    );
    let floor = "environment/collision/0";
    let block = "carriage/block/collision/0";
    let world = fs::read_to_string(files.0.join("world.toml")).unwrap();
    files.write(
        "world.toml",
        &(world
            + r#"
[[environment.collision]]
shape = { kind = "box", size_m = [1.0, 1.0, 0.2] }
pose = { position_m = [0.0, 0.0, -1.0] }
"#),
    );
    let sensor = "carriage/probe";
    let params = [
        (
            Parameter::SensorNoise {
                sensor: sensor.into(),
            },
            "m",
            0.01,
        ),
        (
            Parameter::SensorBias {
                sensor: sensor.into(),
            },
            "m",
            0.02,
        ),
        (
            Parameter::SensorLatency {
                sensor: sensor.into(),
            },
            "s",
            0.003,
        ),
        (
            Parameter::ChannelDelay {
                channel: "line".into(),
            },
            "s",
            0.0000000015,
        ),
    ];
    let axes: Vec<_> = params
        .iter()
        .enumerate()
        .map(|(i, (param, unit, max))| {
            let table = toml::to_string(param).unwrap();
            let inline = table.lines().collect::<Vec<_>>().join(", ");
            axis(
                &format!("p{i}"),
                &format!("{{ {inline} }}"),
                "run",
                unit,
                0.0,
                0.0,
                *max,
            )
        })
        .chain(std::iter::once(axis(
            "friction",
            &format!("{{ kind = \"friction\", pair = [\"{block}\", \"{floor}\"], component = 0 }}"),
            "batch",
            "1",
            0.2,
            0.2,
            0.4,
        )))
        .collect();
    let bounds = params
        .iter()
        .enumerate()
        .map(|(i, (_, _, value))| format!("p{i} = {value:?}"))
        .chain(std::iter::once("friction = 0.4".into()))
        .collect::<Vec<_>>()
        .join(", ");
    files.manifest(7, &format!("[sampling]\naxes = [{}]\nboundaries = [{{ id = 'edge', values = {{ {bounds} }}, source = {SOURCE} }}]", axes.join(", ")));
    let campaign = files.load().unwrap();
    let mut inv = inventory(&campaign, false);
    // Коэффициенты синтетического контакта. Нули в зазоре и адгезии задают сухую пару.
    // Настройки численного стенда из sim-world/tests/contact_options.rs.
    inv.world.contact.solver = Some(sim_world::ContactSolver {
        cone: sim_world::FrictionCone::Elliptic,
        solver: sim_world::ConstraintSolver::Newton,
        impratio: 2.0,
        tolerance: 1e-10,
        iterations: 40,
        line_search_tolerance: 1e-10,
        line_search_iterations: 30,
        noslip_tolerance: 1e-9,
        noslip_iterations: 0,
        ccd_tolerance: 1e-8,
        ccd_iterations: 60,
    });
    inv.world.contact.pairs.push(ContactPair {
        geometries: [floor.into(), block.into()],
        settings: ContactSettings {
            dim: 3,
            friction: [0.2, 0.2, 0.01, 0.001, 0.001],
            solref: [-4000.0, -80.0],
            solreffriction: [0.0, 0.0],
            solimp: [0.9, 0.9, 0.001, 0.5, 2.0],
            margin_m: 0.0,
            gap_m: 0.0,
            adhesion_n: 0.0,
        },
    });
    inv.model_parameters
        .extend(params.iter().map(|(key, unit, max)| {
            (
                key.clone(),
                ModelParameter {
                    unit: (*unit).into(),
                    nominal: 0.0,
                    min: 0.0,
                    max: *max,
                },
            )
        }));
    let plan = campaign.plan(&inv).unwrap();
    let edge = plan
        .rows()
        .iter()
        .find(|r| matches!(r.key.sample, SampleKey::Boundary { .. }))
        .unwrap();
    let inputs = plan.materialize(&edge.key).unwrap();
    let compiled = inputs.world.compile().unwrap();
    let sim = compiled.open_contact_sim().unwrap();
    assert_eq!(
        sim.explicit_contact_settings([floor, block])
            .unwrap()
            .unwrap()
            .friction[0],
        0.4
    );
    assert_eq!(
        compiled.sensors[0].params,
        BTreeMap::from([
            ("noise_std".into(), 0.01),
            ("bias".into(), 0.02),
            ("latency_s".into(), 0.003),
        ])
    );
    assert_eq!(inputs.settings.channel_delay_ns["line"], 2);
    assert_eq!(
        inv.world.devices["carriage"].device.sensors[0].params["noise_std"],
        0.0
    );
    inv.model_parameters.get_mut(&params[0].0).unwrap().max = 0.009;
    assert!(
        campaign
            .plan(&inv)
            .unwrap_err()
            .to_string()
            .contains("область модели")
    );
}

#[test]
fn combined_unavailable_fault_stays_open_and_sampling_keeps_parent_sources() {
    use sim_task::campaign::FaultCombination;

    let fixture = Files::new("parents");
    let probability = sampling("unspecified", 2, 2).replace(
        &format!("source = {SOURCE}\nbatch"),
        "source = { kind = \"estimate\", reference = \"непроверенный закон партии\" }\nbatch",
    );
    fixture.manifest(7, &probability);
    let task = fixture.load().unwrap();
    let mut resolved = inventory(&task, false);
    resolved.paths[0].combinations.push(FaultCombination {
        path: "line".into(),
        faults: ["absent", "no_schedule"]
            .map(|node| Fault {
                node: node.into(),
                kind: FaultKind::WireOpen,
            })
            .to_vec(),
        source: source(),
    });
    let plan = task.plan(&resolved).unwrap();
    let combined = plan
        .rows()
        .iter()
        .find(|row| row.key.faults.len() == 2 && row.key.faults[0].node == "absent")
        .unwrap();
    let Disposition::NotAnalyzed { reasons } = &combined.disposition else {
        panic!("неполный набор исчез из плана");
    };
    assert!(reasons.iter().any(|r| r.contains("отсутствует расписание")));
    assert!(reasons.iter().any(|r| r.contains("инвентарь стенда")));

    let random = plan
        .rows()
        .iter()
        .find(|row| row.key.faults.is_empty() && matches!(row.key.sample, SampleKey::Random { .. }))
        .unwrap();
    let inputs = plan.materialize(&random.key).unwrap();
    assert_eq!(inputs.values().len(), 3);
    assert!(inputs.values().iter().all(|input| {
        input.source.kind == SourceKind::Estimate
            && input
                .sources
                .iter()
                .any(|source| source.reference == "непроверенный закон партии")
    }));
}
