use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    path::PathBuf,
};

use sim_process::{
    Interval, SourcedRange,
    schematic::{
        capacitor_stress::*, deep_discharge::*, fuse_coordination::*, sleep_drain::*,
        switch_thermal::*, *,
    },
    work::cable_flex::*,
};
use sim_signal::Time;
use sim_task::{
    Fingerprint, Outcome, Signal, Source, SourceKind,
    campaign::{
        Campaign, CampaignPlan, Grid, RequiredPath, ResolvedInputs, ResolvedReference, WorldInputs,
    },
    verdict::{
        CampaignEvidence, DEPTH_REGISTRY, ElectricalEvidence, HazardClass, REGISTRY_VERSION,
        ResolvedInventory, SimulationVerdict, VerdictReport, assess_campaign,
        derive_requirements_with_electrical,
    },
};
use sim_world::Provenance;

const FILE: &[u8] = "описание расчётного блока".as_bytes();
const CONDITIONS: &str = "протокол нагрузки блока при температуре стенда";
// Окно опыта — 1 мс, два шага для проверки на половинной сетке.
const END: Time = Time(1_000_000);
const STEP: Time = Time(END.0 / 2);
const WINDOW: Interval = Interval {
    start: Time::ZERO,
    end: END,
};

fn source() -> Provenance {
    Provenance::Measured(CONDITIONS.into())
}

fn exact(value: f64) -> SourcedRange {
    SourcedRange {
        min: value,
        nominal: value,
        max: value,
        provenance: source(),
    }
}

fn parameter(value: f64) -> Parameter {
    Parameter {
        range: Some(exact(value)),
        measurement: CONDITIONS.into(),
    }
}

fn endpoint(component: &str, pin: &str) -> Endpoint {
    Endpoint {
        component: component.into(),
        pin: pin.into(),
    }
}

fn branch(component: &str) -> Section {
    Section::Element {
        component: component.into(),
        element: "ветвь".into(),
    }
}

fn circuit(scope: Scope, limit: f64) -> CompiledSchematic {
    let part = |kind, primitive, nodes: &[&str], parameters: &[(&str, f64)]| Part {
        kind,
        source: CONDITIONS.into(),
        pins: nodes
            .iter()
            .map(|&pin| {
                let role = match (kind, pin) {
                    (PartKind::BenchSupply, "p") => PinRole::Power,
                    (PartKind::BenchSupply, "g") => PinRole::Return,
                    _ => PinRole::Passive,
                };
                (
                    pin.into(),
                    Pin {
                        contact: Contact {
                            kind: ContactKind::Bare,
                            family: None,
                            key: None,
                        },
                        role,
                    },
                )
            })
            .collect(),
        internal_nodes: BTreeSet::new(),
        elements: BTreeMap::from([(
            "ветвь".into(),
            Primitive {
                kind: primitive,
                nodes: nodes.iter().map(|&pin| pin.into()).collect(),
                parameters: parameters
                    .iter()
                    .map(|&(name, value)| (name.into(), parameter(value)))
                    .collect(),
                continuity: false,
            },
        )]),
        ratings: BTreeMap::new(),
        model_gap: None,
        reference_only: false,
    };
    // Источник задаёт 5 В, конденсатор 1 мкФ подключён через два участка по 1 Ом.
    // Ключ нужен для режима отключения. Его затвор связан с истоком.
    let supply = part(
        PartKind::BenchSupply,
        PrimitiveKind::Voltage,
        &["p", "g"],
        &[("volt", 5.0)],
    );
    let mut capacitor = part(
        PartKind::Capacitor,
        PrimitiveKind::Capacitor,
        &["p", "g"],
        &[("farad", 1e-6)],
    );
    capacitor.ratings.insert(
        "пик".into(),
        Rating {
            quantity: Quantity::VoltageV,
            nodes: ["p".into(), "g".into()],
            limit: parameter(limit),
            conditions: CONDITIONS.into(),
        },
    );
    let switch = part(
        PartKind::SwitchModule,
        PrimitiveKind::Switch,
        &["p", "g", "gate", "g"],
        &[
            ("on_ohm", 1.0),
            ("off_ohm", 1e6),
            ("threshold_v", 2.0),
            ("hysteresis_v", 0.0),
        ],
    );
    // Параллельная индуктивная ветвь 1 мГн делает применимым прежний поиск перенапряжения.
    let inductance = part(
        PartKind::Other,
        PrimitiveKind::Inductor,
        &["p", "g"],
        &[("henry", 1e-3)],
    );
    let catalog = Catalog::new(BTreeMap::from([
        ("питание".into(), supply),
        ("накопитель".into(), capacitor),
        ("ключ".into(), switch),
        ("катушка".into(), inductance),
    ]))
    .unwrap();
    let components = ["питание", "накопитель", "ключ", "катушка"]
        .into_iter()
        .map(|name| {
            let mut nets =
                BTreeMap::from([("p".into(), "плюс".into()), ("g".into(), "возврат".into())]);
            if name == "ключ" {
                nets.insert("gate".into(), "возврат".into());
            }
            (
                name.into(),
                Component {
                    part: name.into(),
                    nets,
                },
            )
        })
        .collect();
    let connections = [
        ("провод", "p", "накопитель", "p"),
        ("возврат", "g", "накопитель", "g"),
        ("плюс ключа", "p", "ключ", "p"),
        ("возврат ключа", "g", "ключ", "g"),
        ("затвор", "g", "ключ", "gate"),
        ("плюс катушки", "p", "катушка", "p"),
        ("возврат катушки", "g", "катушка", "g"),
    ]
    .into_iter()
    .map(|(name, pin, part, port)| {
        // Участки по 1 Ом, в индуктивной ветви по 10 Ом для ограничения тока до 0,25 А.
        // Предел 1 А относится к проводам этого стенда.
        (
            name.into(),
            Connection {
                kind: JointKind::Solder,
                ends: [endpoint("питание", pin), endpoint(part, port)],
                resistance_ohm: parameter(if part == "катушка" { 10.0 } else { 1.0 }),
                current_limit_a: parameter(1.0),
                after: BTreeSet::new(),
            },
        )
    })
    .collect();
    compile(
        Schematic {
            name: "Блок".into(),
            scope,
            nets: BTreeSet::from(["плюс".into(), "возврат".into()]),
            components,
            connections,
            rails: BTreeMap::new(),
            bindings: Vec::new(),
        },
        &catalog,
    )
    .unwrap()
}

fn mode_plan(mode: Mode) -> ModePlan {
    let case = ModeCase {
        source: source(),
        window: Some(Window {
            step: STEP,
            end: END,
            initial: InitialState::OperatingPoint,
        }),
        // Плавное уменьшение 5 → 4 В. Монотонный отклик RC начинается в 5 В.
        sources: BTreeMap::from([(
            branch("питание"),
            SourceTrace {
                points: vec![(Time::ZERO, 5.0), (END, 4.0)],
                source: source(),
            },
        )]),
        pins: BTreeMap::new(),
        probes: BTreeSet::from([Probe::Voltage(
            endpoint("накопитель", "p"),
            endpoint("накопитель", "g"),
        )]),
        sampling: None,
    };
    ModePlan {
        cases: BTreeMap::from([(mode, case)]),
        ..ModePlan::default()
    }
}

fn build() -> Fingerprint {
    Fingerprint::of("сборка блока".as_bytes())
}

fn evidence(circuit: &CompiledSchematic, plan: &ModePlan) -> ElectricalEvidence {
    ElectricalEvidence::evaluate("блок", FILE, circuit, plan, build()).unwrap()
}

fn capacitor(
    electrical: &mut ElectricalEvidence,
    circuit: &CompiledSchematic,
    plan: &ModePlan,
    (id, window, sense): (&str, Interval, CapacitorVoltageSense),
    limit: &SourcedRange,
    mark: DepthModeMark<'_>,
) {
    let terminals = [endpoint("накопитель", "p"), endpoint("накопитель", "g")];
    let branch = branch("накопитель");
    let series = ElectricalSeries {
        time: vec![window.start, window.end],
        values: BTreeMap::from([
            (
                Probe::Voltage(terminals[0].clone(), terminals[1].clone()),
                vec![5.0; 2],
            ),
            (Probe::Current(branch.clone()), vec![0.0; 2]),
        ]),
    };
    // Напряжение удерживается на 5 В на эталонном входном ряду, ток C·dV/dt равен нулю.
    // Единица derating означает неизменный предел. Пульсации ограничены 0,5 А,
    // ESR 0,2 Ом, температура горячей точки 300 К при пределе 320 К.
    let error = exact(0.0);
    let voltage_error = exact(1e-10); // Допуск округления в сравнении с решателем цепи.
    let frequency = exact(1000.0); // Полоса условного стенда с постоянным паспортом.
    let temperature = exact(300.0);
    let curve = |cells| CapacitorCurve {
        frequencies_hz: Some(&[1000.0]),
        temperatures_k: &[300.0],
        temperature_basis: CapacitorTemperatureBasis::Hotspot,
        cells,
    };
    let derating = [Some(exact(1.0))];
    let rms = [Some(exact(0.5))];
    let esr = [Some(exact(0.2))];
    let max_hotspot = exact(320.0);
    let ratings = CapacitorRatings {
        rated_voltage_v: Some(limit),
        voltage_derating: Some(curve(&derating)),
        ripple_rms_a: Some(curve(&rms)),
        esr_ohm: Some(curve(&esr)),
        max_hotspot_k: Some(&max_hotspot),
    };
    let input = CapacitorStressInput {
        scope: circuit.depth_scope(),
        mode: plan.cases.keys().next().unwrap(),
        branch: &branch,
        terminals: &terminals,
        voltage_sense: sense,
        window,
        series: Some(&series),
        voltage_error_v: Some(&voltage_error),
        current_error_a: Some(&error),
        frequency_hz: Some(&frequency),
        temperatures: CapacitorTemperatures {
            hotspot_k: Some(&temperature),
            ..Default::default()
        },
        thermal: Some(CapacitorThermal::MeasuredPeak(&temperature)),
    };
    let calculated = electrical
        .record_depth(
            circuit,
            plan,
            id,
            mark,
            DepthCheck::CapacitorStress(input, ratings),
        )
        .unwrap();
    assert_eq!(calculated.window, Some(window));
}

fn by_id<'a>(evidence: &'a ElectricalEvidence, id: &str) -> &'a sim_task::verdict::Finding {
    evidence
        .findings()
        .iter()
        .find(|f| f.depth.as_ref().is_some_and(|depth| depth.id == id))
        .unwrap()
}

#[test]
fn предел_и_область_конденсатора_определяют_исход() {
    let plan = mode_plan(Mode::LoadOff);
    let application = source();
    let mark = DepthModeMark {
        applicability: Some(&application),
        ..Default::default()
    };
    // Независимое сравнение: постоянные 5 В больше 4 В и меньше 6 В.
    for (scope, limit, reference_only, expected) in [
        (Scope::Reference, exact(4.0), true, Outcome::Fail),
        (Scope::Reference, exact(6.0), true, Outcome::Pass),
        (Scope::Assembly, exact(6.0), true, Outcome::NotAnalyzed),
        (
            Scope::Assembly,
            SourcedRange {
                provenance: Provenance::Estimate("оценка стенда".into()),
                ..exact(6.0)
            },
            false,
            Outcome::NotAnalyzed,
        ),
    ] {
        let circuit = circuit(scope, limit.nominal);
        let mut electrical = evidence(&circuit, &plan);
        capacitor(
            &mut electrical,
            &circuit,
            &plan,
            ("конденсатор", WINDOW, CapacitorVoltageSense::Absolute),
            &limit,
            DepthModeMark {
                reference_only,
                ..mark
            },
        );
        let finding = by_id(&electrical, "конденсатор");
        assert_eq!(finding.key.class, HazardClass::CapacitorStress);
        assert_eq!(finding.code, "capacitor_stress");
        assert_eq!(finding.outcome, expected);
        assert_eq!(finding.confirmed_hazard, expected == Outcome::Fail);
        assert_eq!(finding.key.window_ns, Some([0, END.0]));
        if expected == Outcome::Fail {
            let [lo, hi] = finding.measurement.unwrap();
            assert!(lo > 4.0 && lo <= 5.0 && hi >= 5.0);
            // Произведение с derating = 1 округлено наружу по соседним f64.
            assert_eq!(
                finding.limit.as_ref().unwrap().upper,
                Some([4.0_f64.next_down(), 4.0_f64.next_up()])
            );
        }
        if scope == Scope::Assembly {
            assert!(finding.depth.as_ref().unwrap().legacy_voltage.is_none());
        }
    }
}

#[test]
fn старый_свидетель_требует_совпадения_предела_окна_и_метрики() {
    let circuit = circuit(Scope::Reference, 6.0);
    let plan = mode_plan(Mode::LoadOff);
    let mut electrical = evidence(&circuit, &plan);
    let application = source();
    let mark = DepthModeMark {
        applicability: Some(&application),
        ..Default::default()
    };
    for (id, window, sense, limit, reused) in [
        (
            "точное совпадение",
            WINDOW,
            CapacitorVoltageSense::Absolute,
            exact(6.0),
            true,
        ),
        (
            "другой предел",
            WINDOW,
            CapacitorVoltageSense::Absolute,
            exact(7.0),
            false,
        ),
        (
            "сдвинутое окно",
            Interval {
                start: STEP,
                end: END,
            },
            CapacitorVoltageSense::Absolute,
            exact(6.0),
            false,
        ),
        (
            "другая величина",
            WINDOW,
            CapacitorVoltageSense::Forward,
            exact(6.0),
            false,
        ),
        (
            "другой источник",
            WINDOW,
            CapacitorVoltageSense::Absolute,
            SourcedRange {
                provenance: Provenance::Datasheet("другой паспорт".into()),
                ..exact(6.0)
            },
            false,
        ),
    ] {
        capacitor(
            &mut electrical,
            &circuit,
            &plan,
            (id, window, sense),
            &limit,
            mark,
        );
        let finding = by_id(&electrical, id);
        assert_eq!(finding.outcome, Outcome::Pass);
        let legacy = finding.depth.as_ref().unwrap().legacy_voltage.as_ref();
        assert_eq!(
            legacy.is_some(),
            reused,
            "{id}. Исходное свидетельство: {:?}",
            electrical
                .assessment()
                .findings
                .iter()
                .filter(|old| {
                    old.class == ElectricalClass::InductiveSpike
                        && old.mode.as_ref() == Some(&Mode::LoadOff)
                })
                .collect::<Vec<_>>(),
        );
        if let Some(legacy) = legacy {
            let old = electrical
                .findings()
                .iter()
                .find(|f| f.key == legacy.key)
                .unwrap();
            assert_eq!(old.code, "inductive_spike");
            assert_eq!(old.outcome, Outcome::Pass);
            assert_eq!(old.journal, Some(legacy.journal));
            assert!(finding.journal.is_none());
        }
    }
}

#[test]
fn открытые_причины_имеют_свои_ключи_и_сохраняют_старые_обязательства() {
    let circuit = circuit(Scope::Reference, 6.0);
    let mode = Mode::Normal;
    let plan = mode_plan(mode.clone());
    let mut electrical = evidence(&circuit, &plan);
    let before = electrical
        .findings()
        .iter()
        .map(|f| (f.key.clone(), f.outcome))
        .collect::<Vec<_>>();
    let power = branch("питание");
    let switch = branch("ключ");
    let wire = Section::Connection("провод".into());
    let terminals = [
        endpoint("ключ", "p"),
        endpoint("ключ", "g"),
        endpoint("ключ", "gate"),
    ];
    let scope = Scope::Reference;
    let mark_source = source();
    let mark = DepthModeMark {
        applicability: Some(&mark_source),
        ..Default::default()
    };
    let fault = DepthEvent {
        kind: DepthEventKind::FaultApplied,
        at: Time::ZERO,
        branch: wire.clone(),
        source: source(),
    };
    let fuse_input = FuseCoordinationInput {
        scope,
        mode: &mode,
        fuse: &power,
        wire: &wire,
        switch: &switch,
        window: WINDOW,
        trace: &[],
        protection_path: Some(&mark_source),
        prospective_current_a: None,
        recovery_voltage_v: None,
        dc_time_constant_s: None,
        fuse_temperature_k: None,
        wire_start_temperature_k: None,
        switch_start_temperature_k: None,
        prospective_waveform: None,
        switch_waveform: None,
    };
    let fuse = DepthCheck::FuseCoordination(fuse_input, FuseCoordinationRatings::default());
    let thermal = DepthCheck::SwitchThermal(
        SwitchThermalInput {
            scope,
            mode: &mode,
            branch: &switch,
            terminals: &terminals,
            window: WINDOW,
            segments: None,
            ambient_k: None,
            case_k: None,
            initial_rises_k: None,
            thermal_applicability: None,
            exposure: SwitchExposure::Window,
        },
        SwitchThermalRatings::default(),
    );
    let discharge = DepthCheck::DeepDischarge(DeepDischargeInput {
        scope,
        mode: &mode,
        branch: &power,
        window: WINDOW,
        cells: None,
        series_applicability: None,
        segments: None,
        cutoff: None,
    });
    let sleep = DepthCheck::SleepDrain(SleepDrainInput {
        scope,
        mode: &mode,
        branch: &power,
        required: None,
        profile: None,
        battery: None,
    });
    let cable = DepthCheck::CableFlex(
        CableFlexInput {
            scope,
            mode: &mode,
            branch: &wire,
            window: WINDOW,
            mount: None,
            motion: None,
            duty: None,
        },
        CableFlexRatings::default(),
    );
    for (code, class, check, mark) in [
        (
            "fuse_coordination",
            HazardClass::Overcurrent,
            fuse,
            DepthModeMark {
                event: Some(&fault),
                ..mark
            },
        ),
        ("switch_thermal", HazardClass::SwitchThermal, thermal, mark),
        ("deep_discharge", HazardClass::Battery, discharge, mark),
        ("sleep_drain", HazardClass::Battery, sleep, mark),
        ("cable_flex", HazardClass::ConnectionFailure, cable, mark),
    ] {
        electrical
            .record_depth(&circuit, &plan, code, mark, check)
            .unwrap();
        let finding = by_id(&electrical, code);
        assert_eq!(finding.code, code);
        assert_eq!(finding.key.class, class);
        assert_eq!(finding.outcome, Outcome::NotAnalyzed);
        assert_eq!(
            finding.key.window_ns,
            (code != "sleep_drain").then_some([0, END.0])
        );
        assert_eq!(
            finding.electrical.as_ref().unwrap().mode.as_deref(),
            Some("Normal")
        );
        assert!(finding.depth.as_ref().unwrap().legacy_voltage.is_none());
    }
    let event = by_id(&electrical, "fuse_coordination")
        .depth
        .as_ref()
        .unwrap()
        .event
        .as_ref()
        .unwrap();
    assert_eq!(event.kind, "fault_applied");
    assert_eq!(event.at_ns, 0);
    assert_eq!(event.branch, format!("{wire:?}"));
    assert_eq!(
        before,
        electrical.findings()[..before.len()]
            .iter()
            .map(|f| (f.key.clone(), f.outcome))
            .collect::<Vec<_>>()
    );
    assert!(
        electrical
            .record_depth(&circuit, &plan, "deep_discharge", mark, discharge)
            .is_err()
    );
    assert!(
        electrical
            .record_depth(
                &circuit,
                &ModePlan::default(),
                "другая установка",
                mark,
                discharge
            )
            .is_err()
    );
    let wrong_event = DepthEvent { at: END, ..fault };
    assert!(
        electrical
            .record_depth(
                &circuit,
                &plan,
                "неприменимый отказ",
                DepthModeMark {
                    event: Some(&wrong_event),
                    ..mark
                },
                fuse
            )
            .is_err()
    );
    let nan = exact(f64::NAN);
    let invalid = DepthCheck::FuseCoordination(
        FuseCoordinationInput {
            prospective_current_a: Some(&nan),
            ..fuse_input
        },
        FuseCoordinationRatings::default(),
    );
    assert!(
        electrical
            .record_depth(&circuit, &plan, "неконечный ток", mark, invalid)
            .is_err()
    );
    assert_eq!(electrical.findings().len(), before.len() + 5);
}

struct Fixture(PathBuf);

impl Drop for Fixture {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

impl Fixture {
    fn plan() -> (Self, CampaignPlan) {
        let fixture =
            Self(std::env::temp_dir().join(format!("sim-task-depth-{}", std::process::id())));
        fs::create_dir(&fixture.0).unwrap();
        let path = |name: &str| fixture.0.join(name);
        fs::write(path("circuit.toml"), FILE).unwrap();
        fs::write(path("catalog.toml"), "version = 1").unwrap();
        fs::write(
            path("device.toml"),
            r#"
format = 3
name = "Блок"
schematic = "circuit.toml"
[[links]]
name = "блок"
[links.joint]
kind = "fixed"
[links.mass]
kind = "massless"
"#,
        )
        .unwrap();
        fs::write(
            path("world.toml"),
            r#"
format = 3
name = "Блок"
# Частота стенда 1 кГц, нулевая гравитация и отсутствие среды.
seed = 1
physics_hz = 1000
gravity_m_s2 = [0.0, 0.0, 0.0]
[environment]
density_kg_m3 = 0.0
viscosity_pa_s = 0.0
wind_m_s = [0.0, 0.0, 0.0]
[[devices]]
name = "блок"
file = "device.toml"
"#,
        )
        .unwrap();
        fs::write(
            path("campaign.toml"),
            format!(
                r#"
version = 1
world = "world.toml"
catalog = "catalog.toml"
seed = 1
[stations.block]
experiment = {{ kind = "scenario" }}
[stations.block.modes.normal]
horizon_ns = {}
source = {{ kind = "measured", reference = "окно стенда" }}
steps = []
criteria = [{{ id = "граница времени", window = "whole", limit = {{ kind = "range", signal = {{ name = "time", unit = "ns" }}, max = {} , source = {{ kind = "measured", reference = "горизонт стенда" }} }} }}]
[stations.block.modes.normal.windows.whole]
start_ns = 0
end_ns = {}
max_gap_ns = {}
"#,
                END.0, END.0, END.0, END.0
            ),
        )
        .unwrap();
        let campaign = Campaign::load(path("campaign.toml")).unwrap();
        let plan = campaign
            .plan(&ResolvedInputs {
                world: WorldInputs::load(path("world.toml")).unwrap(),
                references: campaign
                    .references()
                    .map(|r| {
                        (
                            r.path.clone(),
                            ResolvedReference {
                                kind: r.kind,
                                file_fingerprint: r.file_fingerprint,
                                graph_fingerprint: build(),
                            },
                        )
                    })
                    .collect(),
                paths: vec![RequiredPath {
                    station: "block".into(),
                    mode: "normal".into(),
                    path: "блок".into(),
                    source: Source {
                        kind: SourceKind::Measured,
                        reference: CONDITIONS.into(),
                    },
                    grids: vec![Grid {
                        id: "номинал".into(),
                        hz: 1000,
                        source: Source {
                            kind: SourceKind::Measured,
                            reference: CONDITIONS.into(),
                        },
                    }],
                    faults: BTreeMap::new(),
                    combinations: vec![],
                    unavailable: None,
                }],
                targets: BTreeMap::new(),
                signals: BTreeSet::from([Signal {
                    name: "time".into(),
                    unit: "ns".into(),
                }]),
                model_parameters: BTreeMap::new(),
                initial_velocity: false,
            })
            .unwrap();
        (fixture, plan)
    }
}

#[test]
fn подтверждённый_отказ_доходит_до_штатного_вердикта() {
    let (_fixture, campaign) = Fixture::plan();
    let plan = mode_plan(Mode::Normal);
    // У прежнего импульсного предела запас: 5 < 6 В. Новый предел 4 В нарушен.
    let circuit = circuit(Scope::Reference, 6.0);
    let inventory = ResolvedInventory {
        build: Some(build()),
        ..Default::default()
    };
    let assess = |result| {
        let set = derive_requirements_with_electrical(
            &inventory,
            &campaign,
            &BTreeMap::from([("блок".into(), result)]),
        )
        .unwrap();
        assess_campaign(&set, &CampaignEvidence::new(&set)).unwrap()
    };
    let mut result = evidence(&circuit, &plan);
    let baseline = assess(evidence(&circuit, &plan));
    assert_eq!(baseline.simulation, SimulationVerdict::NotProven);
    assert!(
        baseline
            .findings
            .iter()
            .all(|finding| finding.depth.is_none())
    );
    capacitor(
        &mut result,
        &circuit,
        &plan,
        ("опасный блок", WINDOW, CapacitorVoltageSense::Absolute),
        &exact(4.0),
        DepthModeMark {
            applicability: Some(&source()),
            ..Default::default()
        },
    );
    let verdict = assess(result);
    assert_eq!(verdict.simulation, SimulationVerdict::Rejected);
    let failed = verdict
        .findings
        .iter()
        .filter(|finding| finding.outcome == Outcome::Fail)
        .collect::<Vec<_>>();
    assert_eq!(failed.len(), 1);
    assert_eq!(failed[0].code, "capacitor_stress");
    assert_eq!(failed[0].key.window_ns, Some([0, END.0]));
}

#[test]
fn отчёт_реестра_версии_2_читается_со_старыми_идентификаторами() {
    // Снимок структуры отчёта версии 2 до появления поля depth.
    let header = r#"
registry_version = 2
plan = 0
simulation = "NotProven"
readiness = "NeedsWork"
diagnostics = []
inapplicable = []
inapplicable_runs = []
"#;
    let old = [
        ("overcurrent", "Power", HazardClass::Overcurrent),
        ("battery", "Battery", HazardClass::Battery),
        ("wire_fault", "Wiring", HazardClass::WireFault),
        ("switch_underdrive", "Power", HazardClass::SwitchUnderdrive),
        ("fatigue", "Parts", HazardClass::Fatigue),
        (
            "connection_failure",
            "Parts",
            HazardClass::ConnectionFailure,
        ),
    ];
    let mut text = header.to_string();
    for (code, group, _) in old {
        text.push_str(&format!(
            r#"
[[findings]]
key = {{ class = "{code}", object = "блок", level = "simulation" }}
code = "{code}"
group = "{group}"
outcome = "not_analyzed"
reason = "обязательство открыто"
measurement_sources = []
limit_sources = []
coverage_gaps = []
missing_data = ["обязательство открыто"]
verification = "стенд"
confirmed_hazard = false
"#
        ));
    }
    let report: VerdictReport = toml::from_str(&text).unwrap();
    assert_eq!(report.registry_version, 2);
    assert_eq!(REGISTRY_VERSION, 3);
    assert_eq!(report.findings.len(), old.len());
    for (finding, (code, _, class)) in report.findings.iter().zip(old) {
        assert_eq!(finding.key.class, class);
        assert_eq!(finding.key.class.spec().id, code);
        assert_eq!(finding.code, code);
        assert!(finding.depth.is_none());
    }
    assert!(toml::from_str::<VerdictReport>(&format!("{text}\nmeasurement = [nan, inf]")).is_err());
    assert_eq!(
        DEPTH_REGISTRY
            .iter()
            .map(|cause| cause.code)
            .collect::<BTreeSet<_>>()
            .len(),
        6
    );
}
