use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use sim_world::{CompiledJointKind, CompiledWorld, Simplification};

use crate::{BuildId, Capture, Identification, Origin, Outcome, Parameter};

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Receipt {
    pub build: BuildId,
    pub log_fingerprint: String,
    pub capture_id: String,
    pub calibration: String,
    pub scope: String,
}

/// Создаётся только для журнала с зарегистрированным стендовым источником.
/// Запись владельца удостоверяет происхождение. Отпечаток связывает её с байтами журнала.
pub struct MeasuredCapture<'a> {
    capture: &'a Capture,
    receipt: &'a Receipt,
}

impl<'a> MeasuredCapture<'a> {
    /// Измеренные отсчёты с ожидаемой единицей. Командные каналы исключены.
    pub fn observation(&self, name: &str, unit: &str) -> Result<&'a [sim_signal::Sample], String> {
        let samples = self.capture.series(name, unit)?;
        if samples.is_empty() {
            return Err(format!("нет измеренных отсчётов канала «{name}»"));
        }
        Ok(samples)
    }

    pub fn verify(
        capture: &'a Capture,
        build: &BuildId,
        receipt: &'a Receipt,
    ) -> Result<Self, String> {
        if capture.origin() != Origin::Recorded {
            return Err("синтетический журнал не может получить источник «замер»".into());
        }
        if capture.build() != build || &receipt.build != build {
            return Err(
                "журнал замера и запись о его происхождении должны относиться к текущей ревизии"
                    .into(),
            );
        }
        if capture.has_missing_observations() {
            return Err("журнал с пропусками пригоден только для диагностики".into());
        }
        if receipt.log_fingerprint != format!("{:016x}", capture.fingerprint())
            || [&receipt.capture_id, &receipt.calibration, &receipt.scope]
                .iter()
                .any(|s| s.trim().is_empty())
        {
            return Err(
                "запись о происхождении не связывает лог, калибровку и область замера".into(),
            );
        }
        Ok(Self { capture, receipt })
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct MeasuredValue {
    pub value: f64,
    pub unit: &'static str,
    pub interval95: [f64; 2],
}

#[derive(Clone, Copy, Debug, Serialize)]
#[serde(rename_all = "snake_case")]
enum Source {
    Measured,
}

#[derive(Debug, Serialize)]
pub struct Calibration {
    source: Source,
    build: BuildId,
    model: String,
    log_fingerprint: String,
    capture_id: String,
    calibration: String,
    scope: String,
    parameters: BTreeMap<Parameter, MeasuredValue>,
}

impl Calibration {
    pub fn from_identification(
        result: &Identification,
        measured: &MeasuredCapture<'_>,
    ) -> Result<Self, String> {
        Self::from_identification_if_resolved(result, measured)?
            .ok_or_else(|| "в опыте нет определённых параметров для замера".into())
    }

    pub(crate) fn from_identification_if_resolved(
        result: &Identification,
        measured: &MeasuredCapture<'_>,
    ) -> Result<Option<Self>, String> {
        let capture = measured.capture;
        if result.origin() != Origin::Recorded
            || result.build() != capture.build()
            || result.fingerprint() != capture.fingerprint()
        {
            return Err("результат получен из другого журнала".into());
        }
        let parameters: BTreeMap<_, _> = result
            .estimates()
            .iter()
            .filter_map(|estimate| match estimate.outcome {
                Outcome::Identified { value } => Some((
                    estimate.parameter,
                    MeasuredValue {
                        value,
                        unit: estimate.unit,
                        interval95: estimate
                            .interval95
                            .expect("у определённого параметра есть интервал"),
                    },
                )),
                Outcome::NotAnalyzed { .. } => None,
            })
            .collect();
        if parameters.is_empty() {
            return Ok(None);
        }
        let receipt = measured.receipt;
        Ok(Some(Self {
            source: Source::Measured,
            build: capture.build().clone(),
            model: result.model().into(),
            log_fingerprint: receipt.log_fingerprint.clone(),
            capture_id: receipt.capture_id.clone(),
            calibration: receipt.calibration.clone(),
            scope: receipt.scope.clone(),
            parameters,
        }))
    }

    pub fn parameters(
        &self,
        build: &BuildId,
        model: &str,
    ) -> Result<&BTreeMap<Parameter, MeasuredValue>, String> {
        if &self.build != build || self.model != model {
            return Err("замер предназначен для другой ревизии или модели".into());
        }
        Ok(&self.parameters)
    }

    /// Устанавливает только полную определённую группу в одноимённую модель.
    /// Каталожные ограничения затем проверяет ReplayRuntime::load.
    pub fn apply(
        &self,
        build: &BuildId,
        world: &mut CompiledWorld,
        target: Target,
    ) -> Result<(), String> {
        let measured = self.parameters(build, &self.model)?;
        let lookup = |parameter, unit| {
            measured
                .get(&parameter)
                .filter(|m| m.unit == unit)
                .map(|m| m.value)
                .ok_or_else(|| {
                    format!("для установки нужен определённый параметр {parameter:?} в «{unit}»")
                })
        };
        use Parameter::*;
        let mut patch = BTreeMap::new();
        match target {
            Target::DriveMotor => {
                patch.insert("coulomb_nm", lookup(Coulomb, "N*m")?);
                patch.insert("viscous_nm_per_rad_s", lookup(Viscous, "N*m*s/rad")?);
                patch.insert("rotor_inertia_kg_m2", lookup(Inertia, "kg*m^2")?);
                // Стенд снимает суммарную входную инерцию. В sim-drive модель
                // использует сумму rotor_inertia_kg_m2 и gear_input_inertia_kg_m2.
                patch.insert("gear_input_inertia_kg_m2", 0.0);
            }
            Target::DriveCoupling => {
                let drive = world
                    .drives
                    .iter()
                    .find(|d| d.name == self.model)
                    .ok_or("нет целевого привода")?;
                let joint = world
                    .joints
                    .iter()
                    .find(|j| j.name == drive.joint)
                    .ok_or("нет оси целевого привода")?;
                let effort = world
                    .efforts
                    .iter()
                    .find(|e| e.name == drive.effort && e.joint == joint.name)
                    .ok_or("нет входа целевого привода")?;
                let linear = joint.kind == CompiledJointKind::Slide
                    || joint.kind == CompiledJointKind::Free && effort.dof < 3;
                let (displacement, stiffness) = if linear {
                    ("m", "N/m")
                } else {
                    ("rad", "N*m/rad")
                };
                patch.insert("backlash", lookup(Backlash, displacement)?);
                patch.insert("coupling_stiffness", lookup(Stiffness, stiffness)?);
            }
            Target::DriveDelay => {
                patch.insert("bus_delay_s", lookup(Delay, "s")?);
            }
            Target::SensorDelay => {
                patch.insert("latency_s", lookup(Delay, "s")?);
            }
            Target::CameraOptics => {
                for (parameter, field, unit) in [
                    (Fx, "fx", "px"),
                    (Fy, "fy", "px"),
                    (Cx, "cx", "px"),
                    (Cy, "cy", "px"),
                    (K1, "k1", "1"),
                    (K2, "k2", "1"),
                    (K3, "k3", "1"),
                    (P1, "p1", "1"),
                    (P2, "p2", "1"),
                ] {
                    patch.insert(field, lookup(parameter, unit)?);
                }
            }
        }
        let params = match target {
            Target::DriveMotor | Target::DriveCoupling | Target::DriveDelay => world
                .drives
                .iter_mut()
                .find(|d| d.name == self.model)
                .map(|d| &mut d.params),
            Target::CameraOptics | Target::SensorDelay => world
                .sensors
                .iter_mut()
                .find(|s| s.name == self.model)
                .map(|s| &mut s.params),
        }
        .ok_or("в мире нет целевой модели замера")?;
        params.extend(
            patch
                .into_iter()
                .map(|(key, value)| (key.to_string(), value)),
        );
        world.simplifications.push(Simplification {
            subject: self.model.clone(),
            text: format!("Источник «замер» для {target:?}: {} / {}, журнал {}, калибровка {}, область {}. Требуется независимая проверка применимости.",
                self.build.design, self.build.revision, self.log_fingerprint, self.calibration, self.scope),
            blocks_proof: true,
        });
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Target {
    DriveMotor,
    DriveCoupling,
    DriveDelay,
    SensorDelay,
    CameraOptics,
}

impl Target {
    pub(crate) fn accepts(self, experiment: &crate::ExperimentKind) -> bool {
        matches!(
            (self, experiment),
            (
                Self::DriveMotor,
                crate::ExperimentKind::Dynamics {
                    axis: crate::Axis::Angular,
                    ..
                }
            ) | (Self::DriveCoupling, crate::ExperimentKind::Coupling { .. })
                | (
                    Self::DriveDelay | Self::SensorDelay,
                    crate::ExperimentKind::Delay { .. }
                )
                | (Self::CameraOptics, crate::ExperimentKind::Camera { .. })
        )
    }
}
