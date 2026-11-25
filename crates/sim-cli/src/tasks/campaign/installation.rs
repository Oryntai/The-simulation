use std::collections::{BTreeMap, BTreeSet};

use serde::Serialize;
use sim_process::{
    Analysis, SourcedRange,
    contact::{ContactFriction, ResolvedContactPair},
};
use sim_runtime::Runtime;
use sim_task::{
    campaign::{ModelParameter, WorldInputs},
    sampling::{Axis, ModelSettings, Parameter, Sample, Sampling},
    verdict::{ComponentKind, HazardClass, REGISTRY, world_components},
};
use sim_world::{CompiledJointKind, CompiledWorld, Provenance};

use super::super::Models;

#[derive(Clone, Copy)]
pub(in crate::tasks) struct LinkedSample<'a> {
    pub sampling: &'a Sampling,
    pub sample: &'a Sample,
}

#[derive(Serialize)]
pub(in crate::tasks) struct Receipt {
    pub seed: u64,
    pub settings: ModelSettings,
    pub values: BTreeMap<String, f64>,
}

impl LinkedSample<'_> {
    pub fn overlay(
        self,
        nominal: &CompiledWorld,
        world: &mut WorldInputs,
        models: Option<&Models>,
        wires: &BTreeMap<String, u64>,
    ) -> Result<ModelSettings, String> {
        let parameters = model_parameters(self.sampling.axes(), nominal, models, wires)?;
        self.sampling
            .apply_to_world(self.sample, nominal, world, &parameters)
            .map_err(|e| e.to_string())
    }

    pub fn install(
        self,
        runtime: &mut Runtime,
        compiled: &CompiledWorld,
        world: &WorldInputs,
        settings: &ModelSettings,
    ) -> Result<Receipt, String> {
        for axis in self.sampling.axes() {
            if let Parameter::JointVelocity { joint, dof } = &axis.parameter
                && runtime.joint_velocity(joint, *dof) != Some(axis.nominal)
            {
                return Err(format!(
                    "номинальная скорость оси «{}» расходится с исходным режимом",
                    axis.id
                ));
            }
        }
        let actual = apply_settings(runtime, settings)?;
        let values = super::native::installed_values(
            &self.sample.values,
            world,
            compiled,
            &actual,
            runtime,
        )?;
        for value in &self.sample.values {
            if values
                .get(&value.id)
                .is_none_or(|actual| actual.to_bits() != value.value.to_bits())
            {
                return Err(format!(
                    "не подтверждена фактическая установка «{}»",
                    value.id
                ));
            }
            if value.sources.iter().any(|source| {
                matches!(
                    source.kind,
                    sim_task::SourceKind::Estimate | sim_task::SourceKind::Derived
                )
            }) {
                runtime.add_simplification(sim_world::Simplification {
                    subject: value.id.clone(),
                    text: "для выборки нет полной цепочки проверяемых источников".into(),
                    blocks_proof: true,
                })?;
            }
        }
        Ok(Receipt {
            seed: runtime.seed(),
            settings: actual,
            values,
        })
    }

    pub fn update_contact_pairs(self, pairs: &mut [ResolvedContactPair]) -> Result<(), String> {
        for (axis, value) in self.sampling.axes().iter().zip(&self.sample.values) {
            let Parameter::Friction { pair, component } = &axis.parameter else {
                continue;
            };
            let resolved = pairs
                .iter_mut()
                .find(|entry| {
                    let names = &entry.pair.geometries;
                    names == pair || names[0] == pair[1] && names[1] == pair[0]
                })
                .ok_or_else(|| format!("контакт {pair:?} не разрешён в постановке"))?;
            let range = friction_component(&mut resolved.parameters.friction, *component)
                .ok_or("компонента трения не участвует в контакте")?;
            if axis.min < range.min || axis.max > range.max {
                return Err(format!(
                    "допуск «{}» выходит за калибровку пары {pair:?}",
                    axis.id
                ));
            }
            let source = format!("{:?} | {}", range.provenance, value.source.reference);
            if range.nominal != value.value {
                range.nominal = value.value;
                range.provenance = Provenance::Derived(source);
                resolved.nominal_coverage = Analysis::NotAnalyzed {
                    reason: "применимость калибровки для выбранного трения проверяется отдельно"
                        .into(),
                };
            }
            resolved.pair.settings.friction[usize::from(*component)] = value.value;
        }
        Ok(())
    }
}

fn friction_component(friction: &mut ContactFriction, index: u8) -> Option<&mut SourcedRange> {
    use ContactFriction::*;
    match (friction, index) {
        (
            Sliding { sliding }
            | SlidingAndTorsion { sliding, .. }
            | SlidingTorsionAndRolling { sliding, .. },
            0..=1,
        ) => sliding.get_mut(usize::from(index)),
        (SlidingAndTorsion { torsion_m, .. } | SlidingTorsionAndRolling { torsion_m, .. }, 2) => {
            Some(torsion_m)
        }
        (SlidingTorsionAndRolling { rolling_m, .. }, 3..=4) => {
            rolling_m.get_mut(usize::from(index - 3))
        }
        _ => None,
    }
}

pub(super) fn apply_settings(
    runtime: &mut Runtime,
    settings: &ModelSettings,
) -> Result<ModelSettings, String> {
    runtime.set_initial_velocities(&settings.initial_velocity)?;
    runtime.set_channel_delays(&settings.channel_delay_ns)?;
    let mut actual = ModelSettings::default();
    for (joint, dofs) in &settings.initial_velocity {
        for &dof in dofs.keys() {
            let velocity = runtime
                .joint_velocity(joint, dof)
                .ok_or("нет скорости в движке")?;
            actual
                .initial_velocity
                .entry(joint.clone())
                .or_default()
                .insert(dof, velocity);
        }
    }
    for channel in settings.channel_delay_ns.keys() {
        let delay = runtime
            .channel_delay_ns(channel)
            .ok_or("нет задержки в проводке")?;
        actual.channel_delay_ns.insert(channel.clone(), delay);
    }
    Ok(actual)
}

pub(super) fn sensor_field(parameter: &Parameter) -> Option<(&str, &'static str)> {
    match parameter {
        Parameter::SensorNoise { sensor } => Some((sensor, "noise_std")),
        Parameter::SensorBias { sensor } => Some((sensor, "bias")),
        Parameter::SensorLatency { sensor } => Some((sensor, "latency_s")),
        _ => None,
    }
}

pub(super) fn model_parameters(
    axes: &[Axis],
    world: &CompiledWorld,
    models: Option<&Models>,
    wires: &BTreeMap<String, u64>,
) -> Result<BTreeMap<Parameter, ModelParameter>, String> {
    let mut parameters = BTreeMap::new();
    for axis in axes {
        let resolve = || -> Result<Option<(&str, f64)>, String> {
            if let Parameter::ChannelDelay { channel } = &axis.parameter {
                let ns = if channel.strip_prefix("observation:").is_some() {
                    // Добавочная задержка шины наблюдений без наложения равна нулю.
                    0
                } else {
                    *wires
                        .get(channel)
                        .ok_or("у канала нет постоянной задержки провода")?
                };
                return Ok(Some(("s", sim_signal::Time(ns).as_secs())));
            }
            let Some((name, field)) = sensor_field(&axis.parameter) else {
                return Ok(None);
            };
            let sensor = world
                .sensors
                .iter()
                .find(|sensor| sensor.name == name)
                .ok_or("датчик отсутствует в мире")?;
            let spec = models
                .and_then(|models| models.sensors.get(&sensor.part))
                .ok_or("модель датчика отсутствует в каталоге установки")?;
            let value = sensor
                .params
                .get(field)
                .copied()
                .or_else(|| spec.model.get(field).and_then(|value| value.value()))
                .ok_or("в модели нет номинального значения параметра датчика")?;
            use sim_sense::SensorKind::*;
            // Единицы основной сигнальной цепочки заданы моделью sim-sense.
            let unit = if field == "latency_s" {
                "s"
            } else {
                match spec.kind {
                    Encoder => match world
                        .joints
                        .iter()
                        .find(|joint| Some(&joint.name) == sensor.joint.as_ref())
                        .map(|j| j.kind)
                    {
                        Some(CompiledJointKind::Hinge) => "rad",
                        Some(CompiledJointKind::Slide) => "m",
                        _ => return Err("неразрешённая ось датчика".into()),
                    },
                    Current => "A",
                    Temperature => "degC",
                    BridgeAdc => "mV",
                    Imu => "m/s^2",
                    Force => "N",
                    Torque => "N*m",
                    Pressure => "Pa",
                    Touch => "1",
                    Rangefinder | DepthCamera => "m",
                    ColorCamera => "1",
                }
            };
            Ok(Some((unit, value)))
        };
        if let Some((unit, nominal)) =
            resolve().map_err(|e| format!("sampling.axes.{}: {e}", axis.id))?
        {
            parameters.insert(
                axis.parameter.clone(),
                ModelParameter {
                    unit: unit.into(),
                    nominal,
                    min: axis.min,
                    max: axis.max,
                },
            );
        }
    }
    Ok(parameters)
}

#[derive(Clone)]
pub(in crate::tasks) struct LinkedInventory {
    pub report: serde_json::Value,
    pub hazards: BTreeSet<(HazardClass, String)>,
}

pub(in crate::tasks) fn inventory(
    compiled: &CompiledWorld,
    inputs: &WorldInputs,
    runtime: &Runtime,
    scope: serde_json::Value,
    extra: impl IntoIterator<Item = (ComponentKind, String)>,
) -> LinkedInventory {
    let mut components: BTreeSet<_> = world_components(inputs).into_iter().chain(extra).collect();
    let control = sim_runtime::control::run(runtime).map(|run| {
        for path in run.paths() {
            components.insert((ComponentKind::Controller, path.producer.clone()));
            components.insert((ComponentKind::Program, path.producer.clone()));
            components.insert((ComponentKind::Output, path.target.clone()));
            for target in &path.fault_targets {
                if target.mechanisms.contains(&sim_runtime::control::FaultMechanism::PowerOff) {
                    components.insert((ComponentKind::Protection, target.name.clone()));
                }
                let Ok(node) = super::super::node_kind(&target.name, path) else {
                    continue;
                };
                let kind = match node {
                    sim_task::NodeKind::Sensor => ComponentKind::Sensor,
                    sim_task::NodeKind::Wire => ComponentKind::Wire,
                    sim_task::NodeKind::Controller => ComponentKind::Controller,
                    sim_task::NodeKind::Drive => ComponentKind::Drive,
                    sim_task::NodeKind::Driver => ComponentKind::Output,
                    sim_task::NodeKind::Power => ComponentKind::Supply,
                    _ => ComponentKind::Communication,
                };
                components.insert((kind, target.name.clone()));
            }
        }
        serde_json::json!({
            "origin": format!("{:?}", run.origin()),
            "external_process": run.has_external_processes(),
            "wiring": super::super::resolved_wiring(&run),
        })
    });
    let mut hazards: BTreeSet<_> = components
        .iter()
        .flat_map(|(kind, object)| {
            REGISTRY
                .iter()
                .filter(move |spec| spec.applies_to.contains(kind))
                .map(|spec| (spec.class, object.clone()))
        })
        .collect();
    hazards.extend(
        runtime
            .process_requirements()
            .unwrap_or_default()
            .iter()
            .map(|evidence| (evidence.kind.into(), evidence.subject.clone())),
    );
    let report = serde_json::json!({
        "world": sim_task::verdict::compiled_world_fingerprint(compiled),
        "seed": compiled.seed,
        "bodies": compiled.bodies.iter().map(|body| (&body.name, (&body.owner, body.mass_kg, body.inertia_kg_m2, format!("{:?}", body.provenance)))).collect::<BTreeMap<_, _>>(),
        "joints": compiled.joints.iter().map(|joint| (&joint.name, format!("{:?}", joint.kind))).collect::<BTreeMap<_, _>>(),
        "drives": compiled.drives.iter().map(|drive| (&drive.name, &drive.part)).collect::<BTreeMap<_, _>>(),
        "sensors": compiled.sensors.iter().map(|sensor| (&sensor.name, &sensor.part)).collect::<BTreeMap<_, _>>(),
        "attachments": inputs.world.attachments.iter().chain(&inputs.contact.added_attachments).map(|a| (&a.name, format!("{:?}", a.mode))).collect::<BTreeMap<_, _>>(),
        "geometries": compiled.contact_geometry_names().ok(),
        "contact_pairs": runtime.process_contact_pairs().map(|p| &p.pair.geometries).collect::<Vec<_>>(),
        "limitations": runtime.simplifications().iter().filter(|s| s.blocks_proof).map(|s| (&s.subject, &s.text)).collect::<Vec<_>>(),
        "control": control,
        "components": components,
        "applicable": hazards,
        "scope": scope,
    });
    LinkedInventory { report, hazards }
}
