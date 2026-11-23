use super::*;
use sim_process::work::rotary::{AngularStop, RotaryMedium};

pub(super) fn value(x: Scalar) -> Result<f64, String> {
    let x = sim_process::SourcedRange::from(x);
    x.validate().map_err(as_error)?;
    Ok(x.nominal)
}

fn rotary_effort(
    world: &CompiledWorld,
    runtime: &Runtime,
    axis_name: &str,
    channel: &str,
) -> Result<ChannelId, String> {
    if joint(world, axis_name)? != CompiledJointKind::Hinge
        || !world
            .efforts
            .iter()
            .any(|e| e.name == channel && e.joint == axis_name && e.dof == 0)
    {
        return Err("момент должен быть приложен к указанной вращательной оси".into());
    }
    bound(runtime, channel)
}

pub(super) fn stop(
    name: &str,
    world: &CompiledWorld,
    runtime: &mut Runtime,
    joint: String,
    channel: &str,
    model: AngularStop,
) -> Result<(), String> {
    let channel = rotary_effort(world, runtime, &joint, channel)?;
    let signal = name.to_owned();
    runtime.publish_process_measurement(&format!("{name}/force"), "N")?;
    runtime.register_process(
        name,
        Stateless(move |frame: &ProcessFrame<'_>| {
            let (q, v) = axis(frame, &joint)?;
            let (force, moment) = model.evaluate(q, v)?;
            let mut out = ProcessOut::default();
            out.efforts.push((channel, moment));
            emit(
                &mut out,
                frame.next.start,
                &signal,
                &[("force", "N", force), ("moment", "N*m", moment)],
                "Односторонний угловой упор с постоянным плечом",
            )?;
            Ok(out)
        }),
    )
}

pub(super) fn medium(
    name: &str,
    world: &CompiledWorld,
    runtime: &mut Runtime,
    joint: String,
    channel: &str,
    model: RotaryMedium,
) -> Result<(), String> {
    let channel = rotary_effort(world, runtime, &joint, channel)?;
    let signal = name.to_owned();
    runtime.register_process(
        name,
        Stateless(move |frame: &ProcessFrame<'_>| {
            let (q, v) = axis(frame, &joint)?;
            let moment = model.moment(q, v)?;
            let mut out = ProcessOut::default();
            out.efforts.push((channel, moment));
            emit(
                &mut out,
                frame.next.start,
                &signal,
                &[("moment", "N*m", moment)],
                "Внешняя гармоническая и квадратичная нагрузка",
            )?;
            Ok(out)
        }),
    )
}

#[derive(Clone)]
struct Hold {
    name: String,
    joint: String,
    at: Time,
    engaged: bool,
}

impl ProcessModel for Hold {
    fn state_fingerprint(&self) -> u64 {
        u64::from(self.engaged)
    }

    fn update(
        &mut self,
        frame: &ProcessFrame<'_>,
        _: &mut Rng,
    ) -> Result<ProcessOut, ProcessError> {
        let mut out = ProcessOut::default();
        let at = frame.next.start;
        if at >= self.at {
            out.hold_scalar_axis(self.joint.clone());
            out.states.insert(self.name.clone(), "удержание".into());
            if !self.engaged {
                out.events.push(ProcessEvent {
                    name: self.name.clone(),
                    state: "удержание".into(),
                    requested: self.at,
                    applied: at,
                });
                self.engaged = true;
            }
        } else {
            out.states.insert(self.name.clone(), "свободно".into());
        }
        Ok(out)
    }
}

pub(super) fn hold(
    name: &str,
    runtime: &mut Runtime,
    joint: String,
    at: Time,
) -> Result<(), String> {
    runtime.validate_scalar_hold(&joint)?;
    runtime.register_process(
        name,
        Hold {
            name: name.into(),
            joint,
            at,
            engaged: false,
        },
    )
}
