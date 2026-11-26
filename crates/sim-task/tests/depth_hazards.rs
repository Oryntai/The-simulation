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

include!("fixtures/depth_hazards.rs");

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

#[test]
fn подтверждённый_отказ_доходит_до_штатного_вердикта() {
    let (_fixture, campaign) = Fixture::plan("depth-hazards");
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
