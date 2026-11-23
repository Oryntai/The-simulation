use serde::Deserialize;
use sim_process::{
    Analysis,
    attachments::{
        CableBend, CableLaw, CableSpan, FoamLaw, RubberLaw, SoftBinding, SoftInput, SoftLaw,
        SoftModel, SoftNode,
    },
};
use sim_runtime::{
    Runtime,
    process::{ProcessFrame, SoftAdapter},
};
use sim_world::CompiledWorld;

use super::{Knot, Scalar, Supported, curve, missing, same_pair, sourced_config, unavailable};
use crate::tasks::processes::models::{Point, invalid, position};

sourced_config!(Span => CableSpan {
    rest_length_m, tensile_rigidity_n, axial_viscosity_ns, strain,
});
sourced_config!(Bend => CableBend {
    rigidity_nm2, maximum_curvature_inv_m, maximum_angle_rad,
});

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(in crate::tasks::processes) struct SoftObject {
    nodes: Option<Vec<Point>>,
    geometries: Option<Vec<String>>,
    law: Option<Law>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
enum Law {
    Cable {
        spans: Option<Vec<Span>>,
        bends: Option<Vec<Bend>>,
    },
    Rubber {
        free_length_m: Option<Scalar>,
        compression: Option<Vec<Knot>>,
        damping_ns_m: Option<Scalar>,
    },
    Foam {
        free_length_m: Option<Scalar>,
        loading: Option<Vec<Knot>>,
        unloading: Option<Vec<Knot>>,
    },
}

impl Law {
    fn build(self) -> Result<Option<SoftLaw>, sim_process::ProcessError> {
        Ok(match self {
            Self::Cable { spans, bends } => {
                let spans = spans.and_then(|v| v.into_iter().map(Span::build).collect());
                let bends = bends.and_then(|v| v.into_iter().map(Bend::build).collect());
                spans
                    .zip(bends)
                    .map(|(spans, bends)| SoftLaw::Cable(CableLaw { spans, bends }))
            }
            Self::Rubber {
                free_length_m,
                compression,
                damping_ns_m,
            } => match (free_length_m, curve(compression)?, damping_ns_m) {
                (Some(length), Some(compression), Some(damping)) => {
                    Some(SoftLaw::Rubber(RubberLaw {
                        free_length_m: length.into(),
                        compression,
                        damping_ns_m: damping.into(),
                    }))
                }
                _ => None,
            },
            Self::Foam {
                free_length_m,
                loading,
                unloading,
            } => match (free_length_m, curve(loading)?, curve(unloading)?) {
                (Some(length), Some(loading), Some(unloading)) => Some(SoftLaw::Foam(FoamLaw {
                    free_length_m: length.into(),
                    loading,
                    unloading,
                })),
                _ => None,
            },
        })
    }
}

impl SoftObject {
    pub(in crate::tasks::processes::models) fn install(
        self,
        name: &str,
        world: &CompiledWorld,
        rt: &mut Runtime,
    ) -> Result<(), String> {
        let law = self
            .law
            .map(Law::build)
            .transpose()
            .map_err(|e| e.to_string())?
            .flatten();
        let (Some(nodes), Some(geometries), Some(law)) = (self.nodes, self.geometries, law) else {
            return missing(rt, name, "нет узлов, геометрии или параметров деформации");
        };
        for node in &nodes {
            position(world, node)?;
        }
        let known = world.contact_geometry_names()?;
        if let Some(unknown) = geometries
            .iter()
            .find(|geometry| !known.contains(*geometry))
        {
            return Err(format!(
                "неизвестная геометрия мягкого предмета «{unknown}»"
            ));
        }
        let binding = SoftBinding {
            name: name.into(),
            bodies: nodes.iter().map(|node| node.body.clone()).collect(),
            geometries,
        };
        let model = SoftModel::new(binding.clone(), law).map_err(|e| e.to_string())?;
        let pairs: Vec<_> = rt
            .process_contact_pairs()
            .filter(|pair| matches!(pair.nominal_coverage, Analysis::Analyzed { .. }))
            .map(|pair| pair.pair.geometries.clone())
            .collect();
        // Учитываются и пары, которые пока не вошли в контакт.
        let covered = world.contact_candidates()?.iter().all(|candidate| {
            !candidate.iter().any(|id| binding.geometries.contains(id))
                || pairs.iter().any(|pair| same_pair(pair, candidate))
        });
        rt.require_process_hazards(&model.hazards())?;
        let adapter = SoftAdapter {
            model,
            input: move |frame: &ProcessFrame<'_>| {
                let previous = frame
                    .completed
                    .ok_or_else(|| invalid("нет завершённого шага"))?;
                let nodes = nodes
                    .iter()
                    .map(|point| {
                        let point = frame.body_point(&point.body, point.local_m)?;
                        Ok(SoftNode {
                            point_m: point.position_m,
                            velocity_m_s: point.velocity_m_s,
                            com_m: point.com_m,
                        })
                    })
                    .collect::<Result<_, _>>()?;
                let contacts = if covered {
                    binding.observe_contacts(previous.interval, &previous.reactions, &pairs)?
                } else {
                    unavailable("не заданы материалы всех возможных пар мягкого предмета")
                };
                Ok(SoftInput { nodes, contacts })
            },
        };
        Supported::install(adapter, name, world, rt)
    }
}
