mod attachments;
mod hazards;
mod soft;

use std::collections::BTreeSet;

pub(in crate::tasks::processes) use attachments::Attachment;
pub(in crate::tasks::processes) use hazards::{Edge, Energy};
pub(in crate::tasks::processes) use soft::SoftObject;

use sim_process::{
    Analysis, ProcessError, SourcedRange,
    attachments::{CurvePoint, ForceCurve},
};
use sim_runtime::{
    Runtime,
    process::{ProcessFrame, ProcessModel, ProcessOut},
};
use sim_signal::Rng;
use sim_world::CompiledWorld;

use super::{Scalar, Stateless, invalid};
use serde::Deserialize;

// Формат параметров совпадает с моделью, но отсутствие числа сохраняется явно.
macro_rules! sourced_config {
    ($config:ident => $model:ident { $($field:ident),+ $(,)? }) => {
        #[derive(Clone, Debug, Deserialize)]
        #[serde(deny_unknown_fields)]
        struct $config {
            $($field: Option<Scalar>,)+
        }

        impl $config {
            fn build(self) -> Option<$model> {
                Some($model { $($field: self.$field?.into(),)+ })
            }
        }
    };
}
use sourced_config;

sourced_config!(Knot => CurvePoint { displacement_m, force_n });

fn curve(points: Option<Vec<Knot>>) -> Result<Option<ForceCurve>, ProcessError> {
    let Some(points) = points.and_then(|points| points.into_iter().map(Knot::build).collect())
    else {
        return Ok(None);
    };
    ForceCurve::new(points).map(Some)
}

fn unavailable<T>(reason: &str) -> Analysis<T> {
    Analysis::NotAnalyzed {
        reason: reason.into(),
    }
}

fn missing(rt: &mut Runtime, name: &str, reason: &'static str) -> Result<(), String> {
    rt.register_process(
        name,
        Stateless(move |_: &ProcessFrame<'_>| {
            let mut out = ProcessOut::default();
            out.blockers.push(reason.into());
            Ok(out)
        }),
    )
}

fn validated_range(data: Scalar) -> Result<SourcedRange, ProcessError> {
    let range = SourcedRange::from(data);
    range.validate()?;
    Ok(range)
}

fn nonnegative(data: Scalar) -> Result<SourcedRange, ProcessError> {
    let range = validated_range(data)?;
    if range.min < 0.0 {
        return Err(invalid("ожидался неотрицательный диапазон"));
    }
    Ok(range)
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a.into_iter().zip(b).map(|(x, y)| x * y).sum()
}

fn cross([x, y, z]: [f64; 3], [a, b, c]: [f64; 3]) -> [f64; 3] {
    [y * c - z * b, z * a - x * c, x * b - y * a]
}

fn norm([x, y, z]: [f64; 3]) -> f64 {
    x.hypot(y).hypot(z)
}

fn same_pair(a: &[String; 2], b: &[String; 2]) -> bool {
    a == b || a[0] == b[1] && a[1] == b[0]
}

#[derive(Clone)]
struct Supported<M> {
    model: M,
    fixed_bodies: BTreeSet<String>,
}

impl<M: ProcessModel + Clone + 'static> Supported<M> {
    fn install(
        self_model: M,
        name: &str,
        world: &CompiledWorld,
        rt: &mut Runtime,
    ) -> Result<(), String> {
        let sim = world.open_contact_sim()?;
        let joint_bodies: BTreeSet<_> = sim.joints().into_iter().map(|j| j.body).collect();
        let fixed_bodies = sim
            .bodies()
            .into_iter()
            .enumerate()
            .filter_map(|(mut id, (name, _))| {
                while id != 0 && !joint_bodies.contains(&id) {
                    id = sim.body_parent(id);
                }
                (id == 0).then_some(name)
            })
            .collect();
        rt.register_process(
            name,
            Self {
                model: self_model,
                fixed_bodies,
            },
        )
    }
}

impl<M: ProcessModel> ProcessModel for Supported<M> {
    fn update(
        &mut self,
        frame: &ProcessFrame<'_>,
        rng: &mut Rng,
    ) -> Result<ProcessOut, ProcessError> {
        let mut out = self.model.update(frame, rng)?;
        // Реакция на закреплённую в мире опору остаётся в расчёте модели.
        // Движок принимает внешние усилия только на тела с подвижной кинематикой.
        out.body_loads
            .retain(|load| !self.fixed_bodies.contains(&load.body));
        Ok(out)
    }

    fn state_fingerprint(&self) -> u64 {
        self.model.state_fingerprint()
    }
}
