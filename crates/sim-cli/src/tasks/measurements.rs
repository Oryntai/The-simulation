use std::collections::BTreeMap;

use serde::Deserialize;
use sim_drive::sampled::{Measurement as DriveMeasurement, Spec};
use sim_runtime::{ObservationOut, Runtime, Sensor, Truth, TruthId};
use sim_signal::{Rate, Rng, SignalId};
use sim_task::{Signal, Source};
use sim_world::{CompiledJointKind, CompiledWorld, Simplification};

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Transducer {
    RotaryPosition {
        joint: String,
        bounds_rad: Option<[f64; 2]>,
    },
    RotaryVelocity {
        joint: String,
    },
    SampledDrive {
        drive: String,
        port: DriveMeasurement,
    },
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Measurement {
    signal: Signal,
    rate_hz: u64,
    source: Source,
    transducer: Transducer,
}

#[derive(Clone)]
enum Input {
    RotaryPosition(String, Option<[f64; 2]>),
    RotaryVelocity(String),
    SampledDrive(TruthId, f64),
}

#[derive(Clone)]
struct IdealSensor(SignalId, Input);

impl Sensor for IdealSensor {
    fn sample(
        &mut self,
        truth: &Truth<'_>,
        _: &mut Rng,
        out: &mut ObservationOut,
    ) -> Result<(), String> {
        let joint = |name: &str| truth.joint(name).ok_or("ось измерителя отсутствует");
        let value = match &self.1 {
            Input::RotaryPosition(name, bounds) => {
                let angle = joint(name)?.0[0];
                bounds.map_or(angle, |[min, max]| angle.clamp(min, max))
            }
            Input::RotaryVelocity(name) => joint(name)?.1[0],
            Input::SampledDrive(id, initial) => truth.plant(*id).unwrap_or(*initial),
        };
        out.write(self.0, value, true);
        Ok(())
    }
}

impl Measurement {
    pub(super) fn install(
        &self,
        runtime: &mut Runtime,
        world: &CompiledWorld,
        sampled: &BTreeMap<String, Spec>,
    ) -> Result<(), String> {
        let rate = Rate::hz(self.rate_hz).ok_or("некорректная частота измерения")?;
        if rate.as_hz() > runtime.physics_rate().as_hz() || self.source.reference.trim().is_empty()
        {
            return Err("проверьте частоту измерения и источник".into());
        }
        let rotary = |name: &str| -> Result<String, String> {
            if world
                .joints
                .iter()
                .any(|joint| joint.name == name && joint.kind == CompiledJointKind::Hinge)
            {
                Ok(name.into())
            } else {
                Err(format!(
                    "измерителю нужна именованная вращательная ось «{name}»"
                ))
            }
        };
        let (input, unit) = match &self.transducer {
            Transducer::RotaryPosition { joint, bounds_rad } => {
                if bounds_rad
                    .is_some_and(|[min, max]| !min.is_finite() || !max.is_finite() || min >= max)
                {
                    return Err("неверный диапазон датчика угла".into());
                }
                (Input::RotaryPosition(rotary(joint)?, *bounds_rad), "rad")
            }
            Transducer::RotaryVelocity { joint } => {
                (Input::RotaryVelocity(rotary(joint)?), "rad/s")
            }
            Transducer::SampledDrive { drive, port } => {
                let spec = sampled
                    .get(drive)
                    .ok_or("у измерителя нет дискретной модели привода")?;
                let (name, unit) = port.output();
                let id = runtime
                    .truth_id(&format!("{drive}/{name}"))
                    .ok_or("нет измеряемого вывода модели")?;
                (
                    Input::SampledDrive(id, spec.initial_measurement(*port)?),
                    unit,
                )
            }
        };
        if self.signal.unit != unit {
            return Err(format!(
                "измерителю «{}» нужна единица «{unit}»",
                self.signal.name
            ));
        }
        let name = &self.signal.name;
        let signal = runtime.register_observation(name, unit)?;
        runtime.register_sensor(name, rate, IdealSensor(signal, input))?;
        runtime.add_simplification(Simplification {
            subject: name.clone(),
            text: format!("Идеальный преобразователь. {}", self.source.reference),
            blocks_proof: true,
        })
    }
}
