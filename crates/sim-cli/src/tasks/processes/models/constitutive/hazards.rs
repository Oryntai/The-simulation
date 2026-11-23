use serde::Deserialize;
use sim_process::{
    Analysis, SourcedRange,
    work::{
        HazardKind,
        mechanics::{EdgeApplicability, EdgeInput, EnergyStore, assess_edge, assess_energy},
    },
};
use sim_runtime::{
    Runtime,
    process::{ProcessFrame, ProcessHazard, ProcessOut},
};
use sim_world::{CompiledWorld, Provenance};

use super::{Scalar, Stateless, norm, same_pair, unavailable, validated_range};
use crate::tasks::processes::models::{measured, validate_pair};

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(in crate::tasks::processes) struct Energy {
    store: Option<Store>,
    limit_j: Option<Scalar>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
enum Store {
    Spring {
        stiffness_n_m: Option<Scalar>,
        deflection_m: Option<Scalar>,
    },
    Capacitor {
        capacitance_f: Option<Scalar>,
        voltage_v: Option<Scalar>,
    },
}

impl Store {
    fn build(self) -> Result<Option<(EnergyStore, [SourcedRange; 2])>, sim_process::ProcessError> {
        let (spring, parameter, coordinate) = match self {
            Self::Spring {
                stiffness_n_m,
                deflection_m,
            } => (true, stiffness_n_m, deflection_m),
            Self::Capacitor {
                capacitance_f,
                voltage_v,
            } => (false, capacitance_f, voltage_v),
        };
        let parameter = parameter.map(validated_range).transpose()?;
        let coordinate = coordinate.map(validated_range).transpose()?;
        Ok(parameter.zip(coordinate).map(|(parameter, coordinate)| {
            let store = if spring {
                EnergyStore::Spring {
                    stiffness_n_m: parameter.clone(),
                    deflection_m: coordinate.clone(),
                }
            } else {
                EnergyStore::Capacitor {
                    capacitance_f: parameter.clone(),
                    voltage_v: coordinate.clone(),
                }
            };
            (store, [parameter, coordinate])
        }))
    }
}

impl Energy {
    pub(in crate::tasks::processes::models) fn install(
        self,
        name: &str,
        subject: &str,
        rt: &mut Runtime,
    ) -> Result<(), String> {
        // Вход описывает гарантированный диапазон на всём окне, включая исходную границу.
        let store = self
            .store
            .map(Store::build)
            .transpose()
            .map_err(|e| e.to_string())?
            .flatten();
        let limit = self.limit_j.map(SourcedRange::from);
        let result = assess_energy(store.as_ref().map(|(store, _)| store), limit.as_ref())
            .map_err(|e| e.to_string())?;
        let sources: Vec<_> = store
            .into_iter()
            .flat_map(|(_, sources)| sources)
            .chain(limit)
            .collect();
        let name_owned = name.to_owned();
        let subject = subject.to_owned();
        rt.register_process(
            name,
            Stateless(move |frame: &ProcessFrame<'_>| {
                let mut out = ProcessOut::default();
                for (field, reduce) in [
                    (
                        "energy_min",
                        (|b: &sim_process::work::mechanics::EnergyBounds| b.min_j) as fn(_) -> _,
                    ),
                    ("energy", |b| b.nominal_j),
                    ("energy_max", |b| b.max_j),
                ] {
                    let value = match &result.energy {
                        Analysis::Analyzed { value, scope } => measured(reduce(value), scope),
                        Analysis::NotAnalyzed { reason } => unavailable(reason),
                    };
                    out.measure(
                        frame.next.start,
                        format!("{name_owned}/{field}"),
                        "J",
                        value,
                    )?;
                }
                out.hazards.push(ProcessHazard {
                    subject: subject.clone(),
                    kind: HazardKind::StoredEnergy,
                    check: result.check.clone(),
                    sources: sources.clone(),
                });
                Ok(out)
            }),
        )
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(in crate::tasks::processes) struct Edge {
    pair: Option<[String; 2]>,
    measured_radius_m: Option<Scalar>,
    minimum_radius_m: Option<Scalar>,
    force_envelope_n: Option<Scalar>,
    calibration: Option<Calibration>,
    scope: Option<String>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Calibration {
    contact_force_n: Scalar,
    #[serde(with = "super::super::Origin")]
    source: Provenance,
}

impl Edge {
    pub(in crate::tasks::processes::models) fn install(
        self,
        name: &str,
        subject: &str,
        world: &CompiledWorld,
        rt: &mut Runtime,
    ) -> Result<(), String> {
        if let Some(pair) = &self.pair {
            validate_pair(world, pair)?;
        }
        let covered = self.pair.as_ref().is_some_and(|expected| {
            rt.process_contact_pairs().any(|row| {
                same_pair(expected, &row.pair.geometries)
                    && matches!(row.nominal_coverage, Analysis::Analyzed { .. })
            })
        });
        let force = self
            .force_envelope_n
            .map(super::nonnegative)
            .transpose()
            .map_err(|e| e.to_string())?;
        let scope_missing = self.scope.is_none();
        let input = EdgeInput {
            measured_radius_m: self.measured_radius_m.map(Into::into),
            minimum_radius_m: self.minimum_radius_m.map(Into::into),
            contact_force_n: None,
            calibrated_application: self.calibration.map(|c| EdgeApplicability {
                source: c.source,
                contact_force_n: c.contact_force_n.into(),
            }),
            scope: self
                .scope
                .unwrap_or_else(|| "область применения не задана".into()),
        };
        assess_edge(&input).map_err(|e| e.to_string())?;
        let sources: Vec<_> = input
            .measured_radius_m
            .iter()
            .chain(&input.minimum_radius_m)
            .chain(&force)
            .chain(
                input
                    .calibrated_application
                    .iter()
                    .map(|c| &c.contact_force_n),
            )
            .cloned()
            .collect();
        let name_owned = name.to_owned();
        let subject = subject.to_owned();
        rt.register_process(
            name,
            Stateless(move |frame: &ProcessFrame<'_>| {
                let mut out = ProcessOut::default();
                let Some(previous) = frame.completed else {
                    return Ok(out);
                };
                let mut total = None;
                if let Some(pair) = &self.pair
                    && covered
                    && previous.contact_gaps.is_empty()
                {
                    for c in &previous.reactions.contacts {
                        if same_pair(&c.geometries, pair)
                            && c.separation_m <= 0.0
                            && c.force_n != [0.0; 3]
                        {
                            *total.get_or_insert(0.0) += norm(c.force_n);
                        }
                    }
                }
                let mut input = input.clone();
                if let Some(total) = total {
                    out.measure(
                        frame.next.start,
                        format!("{name_owned}/contact_force"),
                        "N",
                        measured(total, "сумма модулей фактических реакций пары кромки"),
                    )?;
                    if !scope_missing {
                        input.contact_force_n = force
                            .as_ref()
                            .filter(|f| (f.min..=f.max).contains(&total))
                            .cloned();
                    }
                }
                if let Some((radius, limit)) = input
                    .measured_radius_m
                    .as_ref()
                    .zip(input.minimum_radius_m.as_ref())
                {
                    out.measure(
                        frame.next.start,
                        format!("{name_owned}/radius_margin"),
                        "m",
                        measured(
                            radius.min - limit.max,
                            "минимальный радиус минус верхний порог",
                        ),
                    )?;
                }
                out.hazards.push(ProcessHazard {
                    subject: subject.clone(),
                    kind: HazardKind::SharpEdge,
                    check: assess_edge(&input)?,
                    sources: sources.clone(),
                });
                Ok(out)
            }),
        )
    }
}
