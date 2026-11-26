use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    path::PathBuf,
};

use sim_process::{
    Interval, SourcedRange,
    schematic::{capacitor_stress::*, *},
};
use sim_signal::Time;
use sim_task::{
    Fingerprint, Outcome, Signal, Source, SourceKind,
    campaign::{
        Campaign, CampaignPlan, Grid, RequiredPath, ResolvedInputs, ResolvedReference, WorldInputs,
    },
    verdict::{
        CampaignEvidence, ElectricalEvidence, HazardClass, ResolvedInventory, SimulationVerdict,
        assess_campaign, derive_requirements_with_electrical,
    },
};
use sim_world::Provenance;

include!("fixtures/depth_hazards.rs");

#[test]
fn missing_depth_is_open_in_campaign_without_board_bindings() {
    let (_fixture, campaign) = Fixture::plan("undeclared");
    let circuit = circuit(Scope::Assembly, 6.0);
    assert!(circuit.bindings().is_empty());
    let plan = mode_plan(Mode::Normal);
    let set = derive_requirements_with_electrical(
        &ResolvedInventory {
            build: Some(build()),
            ..Default::default()
        },
        &campaign,
        &BTreeMap::from([("блок".into(), evidence(&circuit, &plan))]),
    )
    .unwrap();
    let result = assess_campaign(&set, &CampaignEvidence::new(&set)).unwrap();
    for class in [HazardClass::CapacitorStress, HazardClass::SwitchThermal] {
        assert!(
            result.findings.iter().any(|finding| {
                finding.key.class == class
                    && finding.outcome == Outcome::NotAnalyzed
                    && finding.reason.contains("не заявлено")
            }),
            "{class:?}: {:?}",
            result.findings
        );
    }
    assert_eq!(result.simulation, SimulationVerdict::NotProven);
}

#[test]
fn applicability_gap_preserves_confirmed_failure() {
    let circuit = circuit(Scope::Assembly, 6.0);
    let plan = mode_plan(Mode::LoadOff);
    let mut electrical = evidence(&circuit, &plan);
    // Источник удерживает 5 В, верхняя граница паспорта 4 В,
    // погрешность 10⁻¹⁰ В не меняет знак запаса.
    capacitor(
        &mut electrical,
        &circuit,
        &plan,
        ("конденсатор", WINDOW, CapacitorVoltageSense::Absolute),
        &exact(4.0),
        DepthModeMark::default(),
    );
    let finding = by_id(&electrical, "конденсатор");
    assert_eq!(finding.outcome, Outcome::Fail);
    assert!(finding.confirmed_hazard);
    assert!(!finding.coverage_gaps.is_empty());
    assert!(finding.measurement.unwrap()[0] > finding.limit.as_ref().unwrap().upper.unwrap()[1]);
    let (_fixture, campaign) = Fixture::plan("confirmed-fail");
    let set = derive_requirements_with_electrical(
        &ResolvedInventory {
            build: Some(build()),
            ..Default::default()
        },
        &campaign,
        &BTreeMap::from([("блок".into(), electrical)]),
    )
    .unwrap();
    let result = assess_campaign(&set, &CampaignEvidence::new(&set)).unwrap();
    assert_eq!(result.simulation, SimulationVerdict::Rejected);
}
