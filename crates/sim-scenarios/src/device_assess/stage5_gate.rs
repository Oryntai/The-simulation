//! Общая блокировка готовности по достижимым источникам питания.

use std::collections::BTreeMap;

use sim_task::interaction::{
    PowerInventory, PowerSource, Reach, ReferenceDiscrepancy, assess_power_inventory,
};
use sim_world::Provenance;

use super::{BodyRegion, Device, DeviceReport, Group, Outcome};

pub struct InteractionInput<'a> {
    pub expected: &'a sim_task::InteractionPlan,
    pub evidence: &'a sim_task::InteractionEvidence,
}

pub fn interaction_discrepancies() -> Vec<ReferenceDiscrepancy> {
    sim_human::known_discrepancies()
        .into_iter()
        .map(|check| {
            let tolerance = check.tolerance_text();
            ReferenceDiscrepancy {
                id: check.id.into(),
                mode: check.title,
                simulated: check.simulated,
                reference: check.reference,
                unit: check.unit.into(),
                source: check.reference_method,
                tolerance,
                note: check.note,
            }
        })
        .collect()
}

pub(super) fn outcome(outcome: sim_task::Outcome) -> Outcome {
    match outcome {
        sim_task::Outcome::Pass => Outcome::Pass,
        sim_task::Outcome::NotAnalyzed => Outcome::NotAnalyzed,
        sim_task::Outcome::Fail => Outcome::Fail,
    }
}

fn legacy_power(device: &Device) -> PowerInventory {
    let reachable = device.mount != BodyRegion::None
        || device
            .links
            .iter()
            .any(|link| link.near_body != BodyRegion::None);
    let reach = if reachable {
        Reach::Reachable
    } else {
        Reach::OffParticipant
    };
    let electrical = &device.electronics;
    let mut sources = BTreeMap::new();
    if !electrical.cell.trim().is_empty() {
        sources.insert(
            format!("накопитель {}", electrical.cell),
            PowerSource {
                reach,
                paths: BTreeMap::new(),
            },
        );
    }
    if [&electrical.board, &electrical.driver, &electrical.regulator]
        .into_iter()
        .any(|name| !name.trim().is_empty())
        || !electrical.connections.is_empty()
    {
        sources.insert(
            "цепь питания".into(),
            PowerSource {
                reach,
                paths: BTreeMap::new(),
            },
        );
    }
    for (index, _) in device
        .links
        .iter()
        .enumerate()
        .filter(|(_, link)| !link.joint.actuator.trim().is_empty())
    {
        sources.insert(
            format!("активная ось {index}"),
            PowerSource {
                reach,
                paths: BTreeMap::new(),
            },
        );
    }
    PowerInventory {
        reach,
        // Старый формат задаёт область крепления и близость к участку.
        // Для достижимого воздействия в нём нет инвентаря путей отдачи при отказе.
        topology: device
            .settings
            .contact_source
            .as_ref()
            .filter(|_| !reachable)
            .map(|source| Provenance::Measured(source.clone())),
        required_sources: sources.keys().cloned().collect(),
        sources,
    }
}

pub(super) fn append(
    device: &Device,
    stamp: u64,
    input: Option<InteractionInput<'_>>,
    report: &mut DeviceReport,
) {
    let inventory = legacy_power(device);
    if inventory.reach != Reach::OffParticipant {
        report.interaction_discrepancies = interaction_discrepancies();
    }
    for finding in assess_power_inventory(&inventory) {
        report.finding(
            Group::Human,
            "participant_power",
            None,
            "Достижимая энергия питания",
            outcome(finding.outcome),
            format!(
                "{:?} {:?}. {}",
                finding.source, finding.path, finding.explanation
            ),
        );
    }
    let Some(input) = input else {
        if inventory.reach != Reach::OffParticipant {
            report.finding(
                Group::Contact,
                "interaction_coverage",
                None,
                "Контакт с участником",
                Outcome::NotAnalyzed,
                "нет связанного прогона с измерением реакций и подтверждённым покрытием",
            );
        }
        return;
    };
    let evidence = input.evidence;
    if !evidence.matches(input.expected, stamp) {
        report.finding(
            Group::Contact,
            "interaction_version",
            None,
            "Связанный прогон",
            Outcome::NotAnalyzed,
            "доказательство относится к другой версии, сборке или заданию",
        );
        return;
    }
    for finding in evidence.power() {
        report.finding(
            Group::Human,
            "participant_power",
            None,
            "Достижимая энергия питания",
            outcome(finding.outcome),
            format!(
                "{:?} {:?}. {}",
                finding.source, finding.path, finding.explanation
            ),
        );
    }
    for finding in evidence.findings() {
        let group = match finding.kind {
            sim_task::interaction::CriterionKind::Exposure => Group::Human,
            sim_task::interaction::CriterionKind::Design => Group::Contact,
        };
        let detail = format!(
            "Вариант «{}», участок «{}», пятно «{}», окно {:?} нс. {:?} против {:?} {}. {}. {}",
            finding.variant,
            finding.region,
            finding.patch,
            finding.window_ns,
            finding.measurement,
            finding.limit,
            finding.unit,
            finding.applicability,
            finding.source,
        );
        if let Some(([lower, upper], [allow_low, allow_high])) =
            finding.measurement.zip(finding.limit)
        {
            let (value, limit) = if finding.outcome == sim_task::Outcome::Fail {
                (lower, allow_high)
            } else {
                (upper, allow_low)
            };
            report.numeric(
                group,
                "interaction_exposure",
                None,
                "Воздействие связанного прогона",
                outcome(finding.outcome),
                value,
                limit,
                finding.unit,
                detail,
            );
        } else {
            report.finding(
                group,
                "interaction_exposure",
                None,
                "Воздействие связанного прогона",
                outcome(finding.outcome),
                detail,
            );
        }
    }
    for blocker in evidence.blockers() {
        report.finding(
            Group::Contact,
            "interaction_coverage",
            None,
            "Покрытие связанного прогона",
            Outcome::NotAnalyzed,
            blocker,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        blueprint::BuildVerdict,
        device_assess::{Finding, Readiness, readiness, verdict},
    };
    use sim_task::interaction::{EnergyBounds, EnergyPath};

    fn green() -> DeviceReport {
        DeviceReport {
            findings: Group::ALL
                .into_iter()
                .map(|group| Finding {
                    group,
                    code: "test",
                    link: None,
                    title: "Блок".into(),
                    outcome: Outcome::Pass,
                    value: 0.0,
                    limit: 0.0,
                    unit: "",
                    detail: String::new(),
                })
                .collect(),
            ..DeviceReport::default()
        }
    }

    #[test]
    fn inventory_without_actuator_is_mandatory_without_a_live_run() {
        let mut d = crate::device_templates::empty_device();
        d.electronics.cell = "испытательный накопитель".into();
        // Имя области приходит из перечисления конструктора.
        d.mount = BodyRegion::ALL
            .into_iter()
            .find(|&region| region != BodyRegion::None)
            .unwrap();
        assert!(d.links.iter().all(|l| l.joint.actuator.is_empty()));
        let mut report = green();
        append(&d, 0, None, &mut report);
        assert!(
            report
                .findings
                .iter()
                .any(|row| row.code == "participant_power"
                    && row.detail.contains("накопитель")
                    && row.outcome == Outcome::NotAnalyzed)
        );
        assert_eq!(report.interaction_discrepancies.len(), 5);
        assert!(report.interaction_discrepancies.iter().all(|row| {
            row.simulated != row.reference && !row.source.is_empty() && !row.mode.is_empty()
        }));
        assert_eq!(verdict(&report.findings), BuildVerdict::NotProven);
        assert_eq!(readiness(&report.findings), Readiness::BenchOnly);
        super::super::human::reconcile_unpowered(&d, &mut report);
        assert_eq!(readiness(&report.findings), Readiness::BenchOnly);
        report
            .findings
            .retain(|finding| finding.group != Group::Mechanics);
        assert_eq!(readiness(&report.findings), Readiness::NeedsWork);
        let actual = super::super::assess_device(&d);
        assert!(
            actual
                .findings
                .iter()
                .any(|row| row.code == "participant_power")
        );
    }

    #[test]
    fn both_legacy_scales_preserve_confirmed_violation() {
        let reference = Provenance::Measured("синтетический опыт, протокол энергии".into());
        // Границы синтетической постановки. Числа не используются в оценке реальных сборок.
        let energy = |value| sim_task::interaction::SourcedRange {
            min: value,
            nominal: value,
            max: value,
            provenance: reference.clone(),
        };
        let mut inventory = PowerInventory {
            reach: Reach::Reachable,
            topology: Some(reference.clone()),
            required_sources: ["накопитель".into()].into(),
            sources: [(
                "накопитель".into(),
                PowerSource {
                    reach: Reach::Reachable,
                    paths: [
                        EnergyPath::Electrical,
                        EnergyPath::Thermal,
                        EnergyPath::Mechanical,
                    ]
                    .into_iter()
                    .map(|path| {
                        (
                            path,
                            EnergyBounds {
                                stored_j: Some(energy(2.0)),
                                deliverable_j: Some(energy(2.0)),
                                hazardous_above_j: Some(energy(1.0)),
                                violation: None,
                            },
                        )
                    })
                    .collect(),
                },
            )]
            .into(),
        };
        let evaluate = |inventory: &PowerInventory| {
            let mut report = green();
            for f in assess_power_inventory(inventory) {
                report.finding(
                    Group::Human,
                    "participant_power",
                    None,
                    "Энергия",
                    outcome(f.outcome),
                    f.explanation,
                );
            }
            (report.verdict(), report.readiness())
        };
        assert_eq!(
            evaluate(&inventory),
            (BuildVerdict::NotProven, Readiness::BenchOnly)
        );
        inventory
            .sources
            .get_mut("накопитель")
            .unwrap()
            .paths
            .get_mut(&EnergyPath::Thermal)
            .unwrap()
            .violation = Some((energy(2.0), energy(1.0)));
        assert_eq!(
            evaluate(&inventory),
            (BuildVerdict::Rejected, Readiness::Rejected)
        );
    }
}
