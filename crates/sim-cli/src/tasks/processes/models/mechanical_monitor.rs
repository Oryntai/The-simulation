use sim_process::{
    SourcedRange,
    work::{
        HazardCheck, HazardEvidence,
        mechanical_limits::{
            NormalImpulse, closing_speed, normal_load, opposed_load, reversed_bending_stress,
            upper_check,
        },
    },
};
use sim_runtime::process::ProcessHazard;

use super::*;

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(in crate::tasks::processes) struct Spec {
    scope: String,
    upper: Scalar,
    observation: Observation,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Observation {
    Impact {
        pair: [String; 2],
    },
    Pinch {
        pairs: [[String; 2]; 2],
        axis: [f64; 3],
    },
    Support {
        pair: [String; 2],
    },
    Closing {
        points: [Point; 2],
        reach_m: Scalar,
    },
    Fatigue(Box<Bending>),
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Bending {
    weld: String,
    section_body: String,
    world_axis: [f64; 3],
    samples_ns: [u64; 3],
    section_modulus_m3: Scalar,
    reversal_error_nm: Scalar,
    maximum_force_n: Scalar,
    maximum_cross_moment_nm: Scalar,
    elastic_limit_pa: Scalar,
}

impl Bending {
    fn parameters(&self) -> [SourcedRange; 5] {
        [
            &self.section_modulus_m3,
            &self.reversal_error_nm,
            &self.maximum_force_n,
            &self.maximum_cross_moment_nm,
            &self.elastic_limit_pa,
        ]
        .map(|s| s.clone().into())
    }

    fn direction(&self) -> Result<[f64; 3], ProcessError> {
        let length = vector_norm(self.world_axis);
        if !length.is_finite() || length <= 0.0 {
            return Err(invalid("невалидная ось изгиба"));
        }
        Ok(self.world_axis.map(|v| v / length))
    }
}

fn vector_norm([x, y, z]: [f64; 3]) -> f64 {
    x.hypot(y).hypot(z)
}

impl Observation {
    fn channel(&self) -> (HazardKind, &'static str, &'static str) {
        match self {
            Self::Impact { .. } => (HazardKind::Impact, "normal_impulse", "N*s"),
            Self::Pinch { .. } => (HazardKind::Pinch, "opposed_load", "N"),
            Self::Support { .. } => (HazardKind::ContactStructure, "normal_load", "N"),
            Self::Closing { .. } => (HazardKind::DangerousMotion, "closing_speed", "m/s"),
            Self::Fatigue(_) => (HazardKind::Fatigue, "reversed_stress", "Pa"),
        }
    }

    fn validate(&self, world: &CompiledWorld, runtime: &mut Runtime) -> Result<(), String> {
        let pairs = match self {
            Self::Impact { pair } | Self::Support { pair } => std::slice::from_ref(pair),
            Self::Pinch { pairs, axis } => {
                let sim = world.open_contact_sim()?;
                let geometries = sim.geoms();
                let body = |name: &str| geometries.iter().find(|g| g.name == name).map(|g| g.body);
                let [[a, first], [b, second]] = pairs
                    .each_ref()
                    .map(|pair| pair.each_ref().map(|g| body(g)));
                if first.is_none() || first != second || [a, b].contains(&first) || a == b {
                    return Err("для сжатия нужны две разные стороны общего тела".into());
                }
                if !axis.iter().all(|x| x.is_finite()) || *axis == [0.0; 3] {
                    return Err("невалидная ось сжатия".into());
                }
                pairs.as_slice()
            }
            Self::Closing { points, reach_m } => {
                for point in points {
                    position(world, point)?;
                }
                if points[0].body == points[1].body {
                    return Err("точки сближения должны лежать на разных телах".into());
                }
                let reach = SourcedRange::from(reach_m.clone());
                reach.validate().map_err(as_error)?;
                if reach.min <= 0.0 {
                    return Err("граница зоны сближения должна быть положительной".into());
                }
                &[]
            }
            Self::Fatigue(spec) => {
                let [a, b, c] = spec.samples_ns;
                if !(0 < a && a < b && b < c) {
                    return Err("нужны три упорядоченные фазы изгиба после начала прогона".into());
                }
                spec.direction().map_err(as_error)?;
                let parameters = spec.parameters();
                for parameter in &parameters {
                    parameter.validate().map_err(as_error)?;
                }
                if parameters.iter().any(|p| p.min < 0.0)
                    || parameters[0].min == 0.0
                    || parameters[4].min == 0.0
                {
                    return Err("невалидные пределы поверки изгиба".into());
                }
                let [_, loaded] = runtime.record_process_weld(&spec.weld)?;
                if loaded != spec.section_body {
                    return Err("сечение должно лежать на нагруженной стороне связи".into());
                }
                &[]
            }
        };
        for pair in pairs {
            validate_pair(world, pair)?;
            if !runtime.process_contact_pairs().any(|entry| {
                let candidate = &entry.pair.geometries;
                (candidate == pair || candidate.iter().eq(pair.iter().rev()))
                    && matches!(entry.nominal_coverage, Analysis::Analyzed { .. })
            }) {
                return Err(format!(
                    "для пары {pair:?} нет параметров численного контакта"
                ));
            }
        }
        Ok(())
    }
}

#[derive(Clone)]
struct Monitor {
    name: String,
    spec: Spec,
    limit: SourcedRange,
    impulse: Option<NormalImpulse>,
    moments: [Option<f64>; 3],
}

impl ProcessModel for Monitor {
    fn update(
        &mut self,
        frame: &ProcessFrame<'_>,
        _: &mut Rng,
    ) -> Result<ProcessOut, ProcessError> {
        let mut out = ProcessOut::default();
        let at = frame.next.start;
        let (kind, field, unit) = self.spec.observation.channel();
        let mut sources = vec![self.limit.clone()];
        let value = match (&self.spec.observation, frame.completed) {
            (Observation::Impact { .. }, None) => {
                self.impulse = Some(NormalImpulse::new(at));
                Some(0.0) // Начальная граница, завершённых интервалов ещё нет.
            }
            (Observation::Impact { pair }, Some(step)) if step.contact_gaps.is_empty() => {
                let force = normal_load(step.interval, &step.reactions, pair)?;
                // После пропуска кадра весь накопленный импульс остаётся неизвестным.
                self.impulse
                    .as_mut()
                    .map(|sum| sum.advance(step.interval, force))
                    .transpose()?
            }
            (Observation::Impact { .. }, Some(_)) => {
                self.impulse = None;
                None
            }
            (Observation::Pinch { pairs, axis }, Some(step)) if step.contact_gaps.is_empty() => {
                Some(opposed_load(step.interval, &step.reactions, pairs, *axis)?)
            }
            (Observation::Support { pair }, Some(step)) if step.contact_gaps.is_empty() => {
                Some(normal_load(step.interval, &step.reactions, pair)?)
            }
            (Observation::Closing { points, reach_m }, _) => {
                let [a, b] = points
                    .each_ref()
                    .map(|p| frame.body_point(&p.body, p.local_m));
                let [a, b] = [a?, b?];
                let reach = SourcedRange::from(reach_m.clone());
                let value = closing_speed(
                    [a.position_m, b.position_m],
                    [a.velocity_m_s, b.velocity_m_s],
                )?
                .and_then(|(distance, speed)| {
                    if distance <= reach.min {
                        Some(speed)
                    } else if distance > reach.max {
                        Some(0.0) // Проверяемая пара расположена вне заданной зоны.
                    } else {
                        None
                    }
                });
                sources.push(reach);
                value
            }
            (Observation::Fatigue(spec), _) => {
                let params = spec.parameters();
                if let Ok(index) = spec.samples_ns.binary_search(&at.0)
                    && let Some(reaction) = frame
                        .completed
                        .and_then(|step| step.weld_reactions.get(&spec.weld))
                        .and_then(|reaction| *reaction)
                {
                    let wrench = reaction.on_second;
                    let direction = spec.direction()?;
                    let moment = wrench
                        .torque_nm
                        .into_iter()
                        .zip(direction)
                        .map(|(torque, axis)| torque * axis)
                        .sum::<f64>();
                    let cross_moment = vector_norm(std::array::from_fn(|i| {
                        wrench.torque_nm[i] - moment * direction[i]
                    }));
                    let force = vector_norm(wrench.force_n);
                    if ![moment, cross_moment, force]
                        .into_iter()
                        .all(f64::is_finite)
                    {
                        return Err(invalid("неконечная реакция в сечении"));
                    }
                    if force <= params[2].min && cross_moment <= params[3].min {
                        self.moments[index] = Some(moment);
                    }
                }
                let value = if let [Some(a), Some(b), Some(c)] = self.moments {
                    reversed_bending_stress([a, b, c], &params[0], &params[1])?.and_then(
                        |[lo, hi]| {
                            (hi <= params[4].min && (lo > self.limit.max || hi <= self.limit.min))
                                .then_some(lo)
                        },
                    )
                } else {
                    None
                };
                sources.extend(params);
                value
            }
            _ => None,
        };
        let reason = "нет подтверждённого наблюдения в области модели";
        let signal = format!("{}/{field}", self.name);
        let outcome = if let Some(value) = value {
            out.measure(at, signal, unit, measured(value, &self.spec.scope))?;
            if sources.iter().all(SourcedRange::can_prove) {
                upper_check(value, &self.limit, &self.spec.scope)?
            } else {
                HazardCheck::NotAnalyzed {
                    reason: "не подтверждены границы области модели".into(),
                }
            }
        } else {
            out.measure(
                at,
                signal,
                unit,
                Analysis::NotAnalyzed {
                    reason: reason.into(),
                },
            )?;
            HazardCheck::NotAnalyzed {
                reason: reason.into(),
            }
        };
        out.hazards.push(ProcessHazard {
            subject: self.name.clone(),
            kind,
            check: outcome,
            sources,
        });
        Ok(out)
    }

    fn state_fingerprint(&self) -> u64 {
        sim_signal::fnv1a(format!("{:?}{:?}", self.impulse, self.moments).bytes())
    }
}

impl Spec {
    pub(super) fn install(
        self,
        name: &str,
        world: &CompiledWorld,
        runtime: &mut Runtime,
    ) -> Result<(), String> {
        self.observation.validate(world, runtime)?;
        let limit = SourcedRange::from(self.upper.clone());
        upper_check(0.0, &limit, &self.scope).map_err(as_error)?;
        runtime.require_process_hazards(&[HazardEvidence {
            subject: name.into(),
            kind: self.observation.channel().0,
            coverage: Analysis::NotAnalyzed {
                reason: "ожидается численное наблюдение".into(),
            },
        }])?;
        runtime.register_process(
            name,
            Monitor {
                name: name.into(),
                spec: self,
                limit,
                impulse: None,
                moments: [None; 3],
            },
        )
    }
}
