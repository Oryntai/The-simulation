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

struct Fixture(PathBuf);

impl Drop for Fixture {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

impl Fixture {
    fn plan(label: &str) -> (Self, CampaignPlan) {
        let fixture =
            Self(std::env::temp_dir().join(format!("sim-task-depth-{}-{label}", std::process::id())));
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
