use std::collections::{BTreeMap, BTreeSet};

use sim_process::{SourcedRange, schematic::*};
use sim_task::{
    Fingerprint, Outcome,
    verdict::{ElectricalEvidence, HazardClass},
};
use sim_world::Provenance;

// Контрольный резистивный контур: 5 В, нагрузка 8 Ом, по 1 Ом на двух стыках.
// Предел 0,25 А — порог численного стенда. Независимый расчёт даёт 0,5 А и 0,25 Вт.
fn measured(value: f64) -> Parameter {
    Parameter {
        range: Some(SourcedRange {
            min: value,
            nominal: value,
            max: value,
            provenance: Provenance::Measured("исходная постановка резистивного стенда".into()),
        }),
        measurement: "измерить на контрольном резистивном контуре".into(),
    }
}

fn end(component: &str, pin: &str) -> Endpoint {
    Endpoint {
        component: component.into(),
        pin: pin.into(),
    }
}

fn circuit(
    limit: Parameter,
    incompatible: bool,
) -> Result<CompiledSchematic, sim_process::ProcessError> {
    let part = |kind, primitive, parameter, value| Part {
        kind,
        source: "постановка резистивного стенда".into(),
        pins: ["p", "g"]
            .into_iter()
            .map(|pin| {
                (
                    pin.into(),
                    Pin {
                        contact: Contact {
                            kind: ContactKind::Bare,
                            family: None,
                            key: None,
                        },
                        role: match (kind, pin) {
                            (PartKind::BenchSupply, "p") => PinRole::Power,
                            (PartKind::BenchSupply, _) => PinRole::Return,
                            _ => PinRole::Passive,
                        },
                    },
                )
            })
            .collect(),
        internal_nodes: BTreeSet::new(),
        elements: BTreeMap::from([(
            "branch".into(),
            Primitive {
                kind: primitive,
                nodes: vec!["p".into(), "g".into()],
                parameters: BTreeMap::from([(String::from(parameter), measured(value))]),
                continuity: false,
            },
        )]),
        ratings: BTreeMap::new(),
        model_gap: None,
        reference_only: false,
    };
    let catalog = Catalog::new(BTreeMap::from([
        (
            "source".into(),
            part(PartKind::BenchSupply, PrimitiveKind::Voltage, "volt", 5.0),
        ),
        (
            "load".into(),
            part(PartKind::Resistor, PrimitiveKind::Resistor, "ohm", 8.0),
        ),
    ]))?;
    let components = ["source", "load"]
        .into_iter()
        .map(|id| {
            (
                id.into(),
                Component {
                    part: id.into(),
                    nets: BTreeMap::from([
                        ("p".into(), "плюс".into()),
                        ("g".into(), "возврат".into()),
                    ]),
                },
            )
        })
        .collect();
    let joint = |pin, kind, current_limit_a, after| Connection {
        kind,
        ends: [end("source", pin), end("load", pin)],
        resistance_ohm: measured(1.0),
        current_limit_a,
        after,
    };
    compile(
        Schematic {
            name: "Блок".into(),
            scope: Scope::Assembly,
            nets: BTreeSet::from(["плюс".into(), "возврат".into()]),
            components,
            connections: BTreeMap::from([
                (
                    "питание".into(),
                    joint(
                        "p",
                        if incompatible {
                            JointKind::Plug
                        } else {
                            JointKind::Solder
                        },
                        limit,
                        BTreeSet::from(["возврат".into()]),
                    ),
                ),
                (
                    "возврат".into(),
                    joint("g", JointKind::Solder, measured(1.0), BTreeSet::new()),
                ),
            ]),
            rails: BTreeMap::new(),
            bindings: Vec::new(),
        },
        &catalog,
    )
}

fn main_only() -> ModePlan {
    ModePlan {
        cases: BTreeMap::from([(
            Mode::MainOnly,
            ModeCase {
                source: Provenance::Measured("исходная постановка резистивного стенда".into()),
                window: None,
                sources: BTreeMap::new(),
                pins: BTreeMap::new(),
                // Ток обеих физических ветвей должен попасть в отчёт и без объявления пробы.
                probes: BTreeSet::from([Probe::Voltage(end("load", "p"), end("load", "g"))]),
                sampling: None,
            },
        )]),
        ..ModePlan::default()
    }
}

fn evidence(compiled: &CompiledSchematic, plan: &ModePlan) -> ElectricalEvidence {
    ElectricalEvidence::evaluate(
        "блок",
        b"fixture",
        compiled,
        plan,
        Fingerprint::of(b"build"),
    )
    .unwrap()
}

#[test]
fn сверхток_и_потери_сохраняются_при_неполных_режимах() {
    let result = evidence(&circuit(measured(0.25), false).unwrap(), &main_only());
    let fail = result
        .findings()
        .iter()
        .find(|f| f.outcome == Outcome::Fail)
        .unwrap();
    let expected = 5.0 / (8.0 + 1.0 + 1.0);
    let tolerance = 1e-9; // Запас против округления решения линейной системы стенда.
    assert_eq!(fail.code, "connection_overcurrent");
    assert_eq!(fail.key.class, HazardClass::Overcurrent);
    assert_eq!(fail.key.object, "блок/питание");
    assert!((fail.measurement.unwrap()[0] - expected).abs() < tolerance);
    let mode = result
        .assessment()
        .modes
        .iter()
        .find(|m| m.mode == Mode::MainOnly)
        .unwrap();
    assert_eq!(fail.journal, result.mode_journal(mode));
    assert!(fail.journal.is_some());
    let heat = result
        .findings()
        .iter()
        .find(|f| {
            f.key.class == HazardClass::WiringHeat
                && f.key.object == fail.key.object
                && f.journal == fail.journal
        })
        .unwrap();
    assert_eq!(heat.outcome, Outcome::NotAnalyzed);
    assert_eq!(heat.unit.as_deref(), Some("W"));
    assert!((heat.measurement.unwrap()[0] - expected * expected * 1.0).abs() < tolerance);
    assert!(
        result
            .assessment()
            .modes
            .iter()
            .any(|m| m.mode == Mode::OpenWire("возврат".into()) && !m.nominal)
    );
    assert!(
        result
            .findings()
            .iter()
            .any(|f| f.code == "connection_overcurrent" && f.outcome == Outcome::NotAnalyzed)
    );
}

#[test]
fn оценка_отсутствие_и_нечисло_не_закрывают_ток() {
    let mut estimated = measured(0.25);
    estimated.range.as_mut().unwrap().provenance =
        Provenance::Estimate("оценка владельца профиля".into());
    for limit in [
        estimated,
        Parameter {
            range: None,
            measurement: "снять предельный ток".into(),
        },
    ] {
        let result = evidence(&circuit(limit, false).unwrap(), &main_only());
        let measured = result
            .findings()
            .iter()
            .find(|f| {
                f.key.object == "блок/питание"
                    && f.code == "connection_overcurrent"
                    && f.measurement.is_some()
            })
            .unwrap();
        assert_eq!(measured.outcome, Outcome::NotAnalyzed);
        assert!(measured.measurement.unwrap()[1] > 0.25);
    }
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(circuit(measured(invalid), false).is_err());
    }
    let mut bad_plan = main_only();
    bad_plan
        .cases
        .get_mut(&Mode::MainOnly)
        .unwrap()
        .probes
        .clear();
    assert!(
        circuit(measured(1.0), false)
            .unwrap()
            .assess_electrical(&bad_plan)
            .is_err()
    );
}

#[test]
fn несовместимость_сохраняется_без_профиля_и_видна_в_листе() {
    let compiled = circuit(measured(1.0), true).unwrap();
    let result = evidence(&compiled, &ModePlan::default());
    assert!(
        result
            .findings()
            .iter()
            .any(|f| f.code == "assembly_mismatch"
                && f.outcome == Outcome::Fail
                && f.confirmed_hazard
                && f.journal.is_some())
    );
    let sheet = compiled.assembly_sheet();
    assert_eq!(sheet.operations[0].id, "возврат");
    assert_eq!(
        sheet.operations[1].ends,
        [end("source", "p"), end("load", "p")]
    );
    assert!(!sheet.operations[1].compatible);
    assert!(sheet.inspections.iter().all(|i| i.subject != "питание"));
    let fixed_compiled = circuit(measured(1.0), false).unwrap();
    let fixed = fixed_compiled.assembly_sheet();
    assert_eq!(fixed, fixed_compiled.assembly_sheet());
    assert!(
        fixed
            .inspections
            .iter()
            .any(|i| i.subject == "питание" && i.ends == sheet.operations[1].ends)
    );
}

#[test]
fn отсутствие_схемы_открывает_все_причины_и_сохраняет_общий_токовый_класс() {
    let missing = ElectricalEvidence::missing("блок", "нет схемы");
    assert_eq!(missing.len(), ElectricalClass::ALL.len());
    assert_eq!(
        missing
            .iter()
            .map(|f| &f.code)
            .collect::<BTreeSet<_>>()
            .len(),
        missing.len()
    );
    assert!(
        missing
            .iter()
            .all(|f| f.outcome == Outcome::NotAnalyzed && f.journal.is_none())
    );
    let current: BTreeSet<_> = missing
        .iter()
        .filter(|f| f.key.class == HazardClass::Overcurrent)
        .map(|f| f.code.as_str())
        .collect();
    assert_eq!(
        current,
        BTreeSet::from(["connection_overcurrent", "regulator_overload"])
    );
}

#[test]
fn свидетельство_от_другой_схемы_версии_или_экземпляра_отклоняется() {
    struct Directory(std::path::PathBuf);
    impl Drop for Directory {
        fn drop(&mut self) {
            std::fs::remove_dir_all(&self.0).unwrap();
        }
    }

    let dir = Directory(std::env::temp_dir().join(format!("sim-e4-{}", std::process::id())));
    std::fs::create_dir(&dir.0).unwrap();
    let device_path = dir.0.join("body.toml");
    let device = sim_world::parse_device(
        &device_path,
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
    let scheme = dir.0.join("circuit.toml");
    std::fs::write(&scheme, b"fixture").unwrap();
    let result = evidence(&circuit(measured(1.0), false).unwrap(), &main_only());
    let build = Some(Fingerprint::of(b"build"));
    result.verify("блок", build, &device_path, &device).unwrap();
    assert!(
        result
            .verify("каретка", build, &device_path, &device)
            .is_err()
    );
    assert!(
        result
            .verify(
                "блок",
                Some(Fingerprint::of(b"another")),
                &device_path,
                &device
            )
            .is_err()
    );
    std::fs::write(&scheme, b"edited").unwrap();
    assert!(result.verify("блок", build, &device_path, &device).is_err());
    let mut detached = device;
    detached.schematic = None;
    assert!(
        result
            .verify("блок", build, &device_path, &detached)
            .is_err()
    );
}
