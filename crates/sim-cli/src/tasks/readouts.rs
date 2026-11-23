use serde::Deserialize;
use sim_runtime::{Evaluator, Runtime, Truth, TruthOut};
use sim_signal::{Rate, Rng};
use sim_task::{Signal, Source, SourceKind};

#[derive(Clone, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Formula {
    Position {
        index: usize,
        clamp: Option<[f64; 2]>,
    },
    Velocity {
        index: usize,
    },
    BearingError {
        target_index: usize,
        offset_m: f64,
        distance_m: f64,
        axis_index: usize,
    },
    PlanarAngle {
        scalar_index: usize,
        vector_index: usize,
    },
    ContactNormal {
        geoms: Option<[usize; 2]>,
    },
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Readout {
    signal: Signal,
    rate_hz: u64,
    source: Source,
    formula: Formula,
}

#[derive(Clone)]
struct Channel(Formula);

impl Channel {
    fn sample(&self, truth: &Truth<'_>) -> Result<f64, String> {
        let q = |index: usize| {
            truth
                .qpos()
                .get(index)
                .copied()
                .ok_or("нет индекса координаты")
        };
        let value = match &self.0 {
            Formula::Position { index, clamp } => {
                let value = q(*index)?;
                clamp.map_or(value, |[min, max]| value.clamp(min, max))
            }
            Formula::Velocity { index } => {
                *truth.qvel().get(*index).ok_or("нет индекса скорости")?
            }
            Formula::BearingError {
                target_index,
                offset_m,
                distance_m,
                axis_index,
            } => (q(*target_index)? + offset_m).atan2(*distance_m) - q(*axis_index)?,
            Formula::PlanarAngle {
                scalar_index,
                vector_index,
            } => {
                // Плоская ориентация из кватерниона wxyz. Удвоение полуугла.
                2.0 * q(*vector_index)?.atan2(q(*scalar_index)?)
            }
            Formula::ContactNormal { geoms } => {
                if geoms.is_some_and(|pair| {
                    pair[0] == pair[1] || pair.iter().any(|&id| id >= truth.engine().ngeom())
                }) {
                    return Err("для контакта нужны два разных существующих геома".into());
                }
                let pair = geoms.map(|pair| pair.map(|id| id as i32));
                truth
                    .engine()
                    .contacts()
                    .into_iter()
                    .filter(|contact| {
                        pair.is_none_or(|pair| {
                            contact.geom == pair || contact.geom == [pair[1], pair[0]]
                        })
                    })
                    .map(|contact| contact.normal_force)
                    .sum()
            }
        };
        Ok(value)
    }
}

#[derive(Clone)]
struct Evaluation(Channel, sim_runtime::TruthId);

impl Evaluator for Evaluation {
    fn evaluate(
        &mut self,
        truth: &Truth<'_>,
        _: &mut Rng,
        out: &mut TruthOut,
    ) -> Result<(), String> {
        out.write(self.1, self.0.sample(truth)?);
        Ok(())
    }
}

impl Readout {
    pub(super) fn install(&self, runtime: &mut Runtime) -> Result<(), String> {
        let rate = Rate::hz(self.rate_hz).ok_or("некорректная частота расчётного ряда")?;
        if rate.as_hz() > runtime.physics_rate().as_hz() || self.source.reference.trim().is_empty()
        {
            return Err("проверьте частоту расчётного ряда и источник".into());
        }
        match self.formula {
            Formula::BearingError {
                offset_m,
                distance_m,
                ..
            } if !offset_m.is_finite() || !distance_m.is_finite() || distance_m <= 0.0 => {
                return Err("неверная геометрия расчётной линии визирования".into());
            }
            Formula::Position {
                clamp: Some([min, max]),
                ..
            } if !min.is_finite() || !max.is_finite() || min > max => {
                return Err("неверный диапазон координаты".into());
            }
            _ => {}
        }
        let name = &self.signal.name;
        let id = runtime.register_truth(name, &self.signal.unit)?;
        runtime.register_evaluator(name, rate, Evaluation(Channel(self.formula.clone()), id))?;
        if self.source.kind == SourceKind::Estimate {
            runtime.add_simplification(sim_world::Simplification {
                subject: name.clone(),
                text: self.source.reference.clone(),
                blocks_proof: true,
            })?;
        }
        Ok(())
    }
}
