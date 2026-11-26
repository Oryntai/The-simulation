//! Ручные границы синтетического стенда записаны до реализации переноса серии.

use std::{collections::BTreeMap, num::NonZeroU32};

use sim_process::schematic::{
    self, Catalog, CompiledSchematic, Component, Connection, Contact, ContactKind, Endpoint,
    JointKind, Parameter, Part, PartKind, Pin, PinRole, Schematic, Scope, Section,
    sim_circuit::netlist::ElementKind,
    solder_joint::{
        SolderControlSeries, SolderJointAddress, SolderJointProfile, SolderSeriesApplicability,
        SolderSeriesMeasurement,
    },
};
use sim_process::work::{
    heat::solder_network::{SolderContact, SolderNetworkInput, SolderNetworkSpec},
    solder_window::{
        SolderBounds, SolderBridgeLimit, SolderCriterion, SolderWindowInput, SolderWindowLimits,
        SolderWindowStatus,
    },
};
use sim_process::{Analysis, ProcessError, SourcedRange};
use sim_world::Provenance;

const JOINT: &str = "стык";
const REVISION: &str = "стенд-1";
const LEGACY_R: f64 = 0.02; // Номинал прежней схемы синтетического стенда.
const LEGACY_I: f64 = 0.5; // Токовый предел прежней схемы стенда.

fn bounds(lower: f64, upper: f64) -> SolderBounds {
    SolderBounds { lower, upper }
}

fn range(min: f64, nominal: f64, max: f64, provenance: Provenance) -> SourcedRange {
    SourcedRange {
        min,
        nominal,
        max,
        provenance,
    }
}

fn point(value: f64) -> Option<SourcedRange> {
    Some(range(
        value,
        value,
        value,
        Provenance::Datasheet("постановка синтетического стенда".into()),
    ))
}

fn end(component: &str) -> Endpoint {
    Endpoint {
        component: component.into(),
        pin: "p".into(),
    }
}

fn schematic(scope: Scope, joint: JointKind, pad: ContactKind) -> CompiledSchematic {
    let catalog = Catalog::new(BTreeMap::from(
        [("площадка", pad), ("вывод", ContactKind::Bare)].map(|(part, kind)| {
            (
                part.into(),
                Part {
                    kind: PartKind::Other,
                    source: "монтажная постановка стенда".into(),
                    pins: BTreeMap::from([(
                        "p".into(),
                        Pin {
                            contact: Contact {
                                kind,
                                family: None,
                                key: None,
                            },
                            role: PinRole::Passive,
                        },
                    )]),
                    internal_nodes: Default::default(),
                    elements: Default::default(),
                    ratings: Default::default(),
                    model_gap: Some("стенд задаёт только контакты".into()),
                    reference_only: false,
                },
            )
        }),
    ))
    .unwrap();
    let parameter = |value| Parameter {
        range: point(value),
        measurement: "контрольный замер".into(),
    };
    schematic::compile(
        Schematic {
            name: "блок".into(),
            scope,
            nets: ["сеть".into()].into(),
            components: ["площадка", "вывод"]
                .map(|part| {
                    (
                        part.into(),
                        Component {
                            part: part.into(),
                            nets: BTreeMap::from([("p".into(), "сеть".into())]),
                        },
                    )
                })
                .into(),
            connections: BTreeMap::from([(
                JOINT.into(),
                Connection {
                    kind: joint,
                    ends: [end("площадка"), end("вывод")],
                    resistance_ohm: parameter(LEGACY_R),
                    current_limit_a: parameter(LEGACY_I),
                    after: Default::default(),
                },
            )]),
            rails: Default::default(),
            bindings: vec![],
        },
        &catalog,
    )
    .unwrap()
}

fn profile() -> SolderJointProfile {
    // Точный стационарный режим: все узлы и полигон имеют 500 К, приток мощности нулевой.
    // По балансу каждого из четырёх узлов dT/dt = 0, маркер 490 К достигнут в нуле,
    // всё время контакта в 1 с выше ликвидуса 495 К, запас по перегреву 10 К.
    SolderJointProfile {
        connection: JOINT.into(),
        revision: Some(REVISION.into()),
        alloy: Some("сплав".into()),
        tool: Some("жало".into()),
        input: SolderWindowInput {
            network: SolderNetworkSpec {
                capacity_j_k: [1.0; 4].map(point).into(),
                initial_k: [500.0; 4].map(point).into(),
                pad_lead_w_k: point(1.0),
                pad_copper_w_k: point(1.0),
                copper_polygon_w_k: point(1.0),
                polygon_k: point(500.0),
                max_power_w: point(1.0),
            },
            thermal: SolderNetworkInput {
                contact: Some(SolderContact::Touching {
                    conductance_w_k: point(1.0),
                }),
                requested_power_w: point(0.0),
            },
            force_n: point(1.0),
            duration_s: point(1.0),
            pitch_m: point(0.001),
            dose_m3: point(3e-10),
            limits: SolderWindowLimits {
                wetting_k: point(490.0),
                max_time_to_wet_s: point(1.0),
                liquidus_k: point(495.0),
                min_above_liquidus_s: point(0.5),
                max_above_liquidus_s: point(1.5),
                max_pad_k: point(510.0),
                max_component_k: point(510.0),
                min_volume_m3: point(2.5e-10),
                bridge: Some(SolderBridgeLimit {
                    pitch_m: point(0.001).unwrap(),
                    max_volume_m3: point(5e-10).unwrap(),
                }),
            },
        },
    }
}

fn series(profile: &SolderJointProfile) -> SolderControlSeries {
    let measurement = |min, nominal, max, conditions: &str| SolderSeriesMeasurement {
        range: range(
            min,
            nominal,
            max,
            Provenance::Measured("синтетический журнал контрольной серии".into()),
        ),
        conditions: Some(conditions.into()),
    };
    SolderControlSeries {
        id: "серия".into(),
        address: SolderJointAddress {
            schematic: "блок".into(),
            scope: Scope::Reference,
            connection: JOINT.into(),
            ends: [end("площадка"), end("вывод")],
        },
        revision: REVISION.into(),
        applicability: Some(SolderSeriesApplicability {
            alloy: "сплав".into(),
            tool: "жало".into(),
            network: profile.input.network.clone(),
            thermal: profile.input.thermal.clone(),
            force_n: Some(bounds(1.0, 1.0)),
            duration_s: Some(bounds(1.0, 1.0)),
            pitch_m: Some(bounds(0.001, 0.001)),
            dose_m3: Some(bounds(3e-10, 4e-10)),
            source: Some(Provenance::Measured("совместная огибающая стенда".into())),
        }),
        four_wire_resistance_ohm: Some(measurement(0.001, 0.0015, 0.002, "Kelvin, 2 А")),
        destructive_strength_n: Some(measurement(10.0, 15.0, 20.0, "осевое растяжение")),
    }
}

fn steps() -> NonZeroU32 {
    // Четыре четверти секунды стационарного решения.
    NonZeroU32::new(4).unwrap()
}

fn value<T>(analysis: &Analysis<T>) -> &T {
    match analysis {
        Analysis::Analyzed { value, .. } => value,
        Analysis::NotAnalyzed { reason } => panic!("{reason}"),
    }
}

fn open<T: std::fmt::Debug>(analysis: &Analysis<T>, fragment: &str) {
    assert!(
        matches!(analysis, Analysis::NotAnalyzed { reason } if reason.contains(fragment)),
        "ожидалась причина про «{fragment}»: {analysis:?}"
    );
}

fn margin_contains(margin: SolderBounds, expected: f64) {
    assert!(
        margin.lower <= expected && expected <= margin.upper,
        "{margin:?}, {expected}"
    );
}

#[test]
fn window_and_series_are_separate_and_legacy_branch_keeps_its_values() {
    let scheme = schematic(Scope::Reference, JointKind::Solder, ContactKind::Pad);
    let profile = profile();
    let series = series(&profile);
    let legacy = scheme
        .assess_solder_joint(JOINT, None, None, steps())
        .unwrap();
    assert_eq!(scheme.solder_joint_ids().collect::<Vec<_>>(), [JOINT]);
    open(&legacy.window, "профиля пайки");
    open(&legacy.resistance_ohm, "контрольной серии");
    open(&legacy.strength_n, "контрольной серии");

    let report = scheme
        .assess_solder_joint(JOINT, Some(&profile), Some(&series), steps())
        .unwrap();
    assert_eq!(
        report,
        scheme
            .assess_solder_joint(JOINT, Some(&profile), Some(&series), steps())
            .unwrap()
    );
    assert_eq!(report.address, series.address);
    assert_eq!(report.window_status(), SolderWindowStatus::Pass);
    let thermal = value(&value(&report.window).thermal);
    margin_contains(thermal.peak_pad_k, 500.0);
    margin_contains(thermal.peak_component_k, 500.0);
    margin_contains(*value(&thermal.above_liquidus_s), 1.0);
    let margin = value(&report.window)
        .check(SolderCriterion::MinVolume)
        .margin
        .unwrap();
    margin_contains(margin, 3e-10 - 2.5e-10);
    // 0,30 − 0,25 = 0,05 мм³. Допуск покрывает округление входов в СИ.
    assert!((margin.lower - 5e-11).abs() <= 1e-25);
    assert_eq!(
        value(&report.applicability),
        series.applicability.as_ref().unwrap()
    );

    let resistance = value(&report.resistance_ohm);
    let strength = value(&report.strength_n);
    // Независимые первичные записи стенда: U = 2/3/4 мВ при I = 2 А.
    assert_eq!(
        [
            resistance.range.min,
            resistance.range.nominal,
            resistance.range.max
        ],
        [2e-3, 3e-3, 4e-3].map(|voltage| voltage / 2.0)
    );
    // Разрушающие нагрузки от масс 1/1,5/2 кг при учебном g = 10 Н/кг.
    assert_eq!(
        [
            strength.range.min,
            strength.range.nominal,
            strength.range.max
        ],
        [1.0, 1.5, 2.0].map(|mass| mass * 10.0)
    );
    assert_eq!(
        resistance,
        series.four_wire_resistance_ohm.as_ref().unwrap()
    );
    assert_eq!(strength, series.destructive_strength_n.as_ref().unwrap());

    let expanded = scheme.netlist();
    let expanded = value(&expanded);
    let element = expanded
        .netlist
        .elements
        .iter()
        .find(|element| expanded.origins[&element.name] == Section::Connection(JOINT.into()))
        .unwrap();
    assert!(matches!(element.kind, ElementKind::Resistor { r } if r == LEGACY_R));
    let sheet = scheme.assembly_sheet();
    assert!(sheet.inspections.iter().any(
        |check| matches!(&check.expected, schematic::Expected::CurrentLimitA(parameter)
            if parameter.range.as_ref().unwrap().nominal == LEGACY_I)
    ));
}

#[test]
fn failing_and_indeterminate_windows_never_transfer_measurements() {
    let scheme = schematic(Scope::Reference, JointKind::Solder, ContactKind::Pad);
    let mut profile = profile();
    let mut series = series(&profile);
    // Ниже минимума 0,25 мм³ на 0,025 мм³. Серия включает этот режим намеренно.
    series.applicability.as_mut().unwrap().dose_m3 = Some(bounds(2e-10, 4e-10));
    profile.input.dose_m3 = point(2.25e-10);
    let report = scheme
        .assess_solder_joint(JOINT, Some(&profile), Some(&series), steps())
        .unwrap();
    let window = value(&report.window);
    assert_eq!(report.window_status(), SolderWindowStatus::Fail);
    let criterion = window.check(SolderCriterion::MinVolume);
    let expected_margin = (0.225_f64 - 0.25) * 1e-9;
    let margin = criterion.margin.unwrap();
    // Верхняя и нижняя границы включают вычисление разности двух чисел в СИ.
    assert!((margin.upper - expected_margin).abs() <= 1e-25);
    assert!(margin.upper < 0.0);
    assert!(
        window
            .checks()
            .iter()
            .all(|check| check.criterion == SolderCriterion::MinVolume
                || check.status == SolderWindowStatus::Pass)
    );
    open(&report.resistance_ohm, "выполнение окна");
    open(&report.strength_n, "выполнение окна");

    // 0,275 мм³ даёт +0,025 мм³ по тому же пределу.
    profile.input.dose_m3 = point(2.75e-10);
    let report = scheme
        .assess_solder_joint(JOINT, Some(&profile), Some(&series), steps())
        .unwrap();
    assert_eq!(report.window_status(), SolderWindowStatus::Pass);
    let margin = value(&report.window)
        .check(SolderCriterion::MinVolume)
        .margin
        .unwrap();
    assert!((margin.lower - (0.275_f64 - 0.25) * 1e-9).abs() <= 1e-25);
    assert!(margin.lower > 0.0);
    value(&report.resistance_ohm);
    value(&report.strength_n);

    profile
        .input
        .limits
        .min_volume_m3
        .as_mut()
        .unwrap()
        .provenance = Provenance::Estimate("предел без замера".into());
    let report = scheme
        .assess_solder_joint(JOINT, Some(&profile), Some(&series), steps())
        .unwrap();
    assert!(matches!(
        report.window_status(),
        SolderWindowStatus::NotAnalyzed { .. }
    ));
    open(&report.resistance_ohm, "выполнение окна");
    profile.input.dose_m3 = point(5.25e-10); // +0,025 мм³ сверх предела перемычки.
    let report = scheme
        .assess_solder_joint(JOINT, Some(&profile), Some(&series), steps())
        .unwrap();
    assert_eq!(report.window_status(), SolderWindowStatus::Fail);
    assert_eq!(
        value(&report.window).check(SolderCriterion::Bridge).status,
        SolderWindowStatus::Fail
    );
    open(&report.strength_n, "выполнение окна");
}

#[test]
fn only_full_envelope_with_identical_revision_and_setup_is_transferable() {
    let scheme = schematic(Scope::Reference, JointKind::Solder, ContactKind::Pad);
    let mut profile = profile();
    let series = series(&profile);
    let provenance = point(0.0).unwrap().provenance;
    let dose = |min, max| Some(range(min, 3.5e-10, max, provenance.clone()));
    profile.input.dose_m3 = dose(3e-10, 4e-10);
    let evaluate = |profile: &SolderJointProfile, series: &SolderControlSeries| {
        let result = scheme
            .assess_solder_joint(JOINT, Some(profile), Some(series), steps())
            .unwrap();
        assert_eq!(result.window_status(), SolderWindowStatus::Pass);
        result
    };
    value(&evaluate(&profile, &series).resistance_ohm);
    // Выход на один представимый шаг при том же номинале с обеих сторон.
    for (lo, hi) in [(3e-10_f64.next_down(), 4e-10), (3e-10, 4e-10_f64.next_up())] {
        profile.input.dose_m3 = dose(lo, hi);
        open(&evaluate(&profile, &series).resistance_ohm, "доза");
    }
    profile.input.dose_m3 = dose(3e-10, 4e-10);
    let edits: [fn(&mut SolderControlSeries); 11] = [
        |s| s.revision = "другая".into(),
        |s| s.address.scope = Scope::Assembly,
        |s| s.address.connection = "другой".into(),
        |s| s.address.ends = [end("другая площадка"), end("вывод")],
        |s| s.address.ends.reverse(),
        |s| s.applicability.as_mut().unwrap().tool = "другое жало".into(),
        |s| s.applicability.as_mut().unwrap().network.max_power_w = point(2.0),
        |s| s.applicability.as_mut().unwrap().duration_s = None,
        |s| s.applicability.as_mut().unwrap().pitch_m = Some(bounds(0.002, 0.002)),
        |s| s.applicability.as_mut().unwrap().force_n = Some(bounds(2.0, 2.0)),
        |s| s.applicability.as_mut().unwrap().source = None,
    ];
    for edit in edits {
        let mut changed = series.clone();
        edit(&mut changed);
        let report = evaluate(&profile, &changed);
        assert!(matches!(report.applicability, Analysis::NotAnalyzed { .. }));
        assert!(matches!(
            report.resistance_ohm,
            Analysis::NotAnalyzed { .. }
        ));
        assert!(matches!(report.strength_n, Analysis::NotAnalyzed { .. }));
    }
    let assembly = schematic(Scope::Assembly, JointKind::Solder, ContactKind::Pad);
    let report = assembly
        .assess_solder_joint(JOINT, Some(&profile), Some(&series), steps())
        .unwrap();
    open(&report.resistance_ohm, "области схемы");
    open(&report.strength_n, "области схемы");
    profile.revision = None;
    open(&evaluate(&profile, &series).strength_n, "ревизия");
}

#[test]
fn each_measured_range_requires_its_own_source_and_method_conditions() {
    let scheme = schematic(Scope::Reference, JointKind::Solder, ContactKind::Pad);
    let profile = profile();
    let mut series = series(&profile);
    let report = scheme
        .assess_solder_joint(JOINT, Some(&profile), None, steps())
        .unwrap();
    assert_eq!(report.window_status(), SolderWindowStatus::Pass);
    open(&report.strength_n, "контрольной серии");
    series.destructive_strength_n = None;
    let report = scheme
        .assess_solder_joint(JOINT, Some(&profile), Some(&series), steps())
        .unwrap();
    value(&report.resistance_ohm);
    open(&report.strength_n, "разруша");
    series
        .four_wire_resistance_ohm
        .as_mut()
        .unwrap()
        .range
        .provenance = Provenance::Datasheet("предел из паспорта".into());
    let report = scheme
        .assess_solder_joint(JOINT, Some(&profile), Some(&series), steps())
        .unwrap();
    open(&report.resistance_ohm, "источник");
    series.four_wire_resistance_ohm = self::series(&profile).four_wire_resistance_ohm;
    series.four_wire_resistance_ohm.as_mut().unwrap().conditions = None;
    let report = scheme
        .assess_solder_joint(JOINT, Some(&profile), Some(&series), steps())
        .unwrap();
    open(&report.resistance_ohm, "условия");
}

#[test]
fn invalid_input_is_rejected_even_if_window_or_scope_is_missing() {
    let scheme = schematic(Scope::Reference, JointKind::Solder, ContactKind::Pad);
    let mut profile = profile();
    let mut series = series(&profile);
    for number in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut bad = series.clone();
        bad.destructive_strength_n.as_mut().unwrap().range.min = number;
        assert!(matches!(
            scheme.assess_solder_joint(JOINT, None, Some(&bad), steps()),
            Err(ProcessError::InvalidInput(_))
        ));
        let mut bad = series.clone();
        bad.applicability
            .as_mut()
            .unwrap()
            .thermal
            .requested_power_w = point(number);
        assert!(matches!(
            scheme.assess_solder_joint(JOINT, None, Some(&bad), steps()),
            Err(ProcessError::InvalidInput(_))
        ));
        let mut bad = series.clone();
        bad.applicability.as_mut().unwrap().duration_s = Some(bounds(number, 1.0));
        assert!(matches!(
            scheme.assess_solder_joint(JOINT, None, Some(&bad), steps()),
            Err(ProcessError::InvalidInput(_))
        ));
    }
    series.applicability.as_mut().unwrap().dose_m3 = Some(bounds(4e-10, 3e-10));
    assert!(
        scheme
            .assess_solder_joint(JOINT, None, Some(&series), steps())
            .is_err()
    );
    profile.input.pitch_m = point(f64::NAN);
    assert!(
        scheme
            .assess_solder_joint(JOINT, Some(&profile), None, steps())
            .is_err()
    );
    profile.connection = "иной стык".into();
    assert!(
        scheme
            .assess_solder_joint(JOINT, Some(&profile), None, steps())
            .is_err()
    );
    assert!(
        scheme
            .assess_solder_joint("нет такого стыка", None, None, steps())
            .is_err()
    );

    let splice = schematic(Scope::Reference, JointKind::Splice, ContactKind::Bare);
    assert_eq!(splice.solder_joint_ids().count(), 0);
    assert!(
        splice
            .assess_solder_joint(JOINT, None, None, steps())
            .is_err()
    );

    let mismatch = schematic(Scope::Reference, JointKind::Solder, ContactKind::Terminal);
    let profile = self::profile();
    let series = self::series(&profile);
    let report = mismatch
        .assess_solder_joint(JOINT, Some(&profile), Some(&series), steps())
        .unwrap();
    open(&report.window, "несовместим");
    open(&report.strength_n, "выполнение окна");
}
