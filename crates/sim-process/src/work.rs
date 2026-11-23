//! Связи насадок с рабочими процессами и их данными для оценщика.

use std::collections::{BTreeMap, BTreeSet};

use sim_signal::{SignalSpec, Time};

use crate::{Analysis, ProcessError};

pub mod electrical;
pub mod heat;
pub mod immersion;
pub mod mechanical_limits;
pub mod mechanics;
pub mod rotary;
pub mod trace;

mod math;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum ProcessKind {
    SurfaceMarking,
    HeatTransfer,
    ImpactTransmission,
    ThreadEngagement,
    ElectricalContact,
    StoredEnergy,
    SharpEdge,
    Immersion,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum HazardKind {
    Burns,
    Impact,
    Pinch,
    ThreadFailure,
    ElectricShock,
    Overcurrent,
    StoredEnergy,
    SharpEdge,
    Ingress,
    Retention,
    ConnectionFailure,
    ContactStructure,
    DangerousMotion,
    Fatigue,
}

/// Имена портов проверяет выбранная общая модель. Значения портов ссылаются
/// на объекты или каналы сцены, которые сборщик разрешает до первого шага.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProcessBinding {
    pub name: String,
    pub kind: ProcessKind,
    pub attachment: Option<String>,
    pub ports: BTreeMap<String, String>,
}

/// Обязательное покрытие опасности выводится из состава и свойств сцены,
/// включая сменные насадки. Статус относится к конкретной опасности и сценарию.
#[derive(Clone, Debug, PartialEq)]
pub struct HazardEvidence {
    pub subject: String,
    pub kind: HazardKind,
    pub coverage: Analysis<()>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ProcessMeasurement {
    pub at: Time,
    pub signal: SignalSpec,
    pub result: Analysis<f64>,
}

impl ProcessMeasurement {
    pub fn new(at: Time, signal: SignalSpec, result: Analysis<f64>) -> Result<Self, ProcessError> {
        math::name(&signal.name)?;
        math::name(&signal.unit)?;
        match &result {
            Analysis::Analyzed { value, scope } => {
                math::finite(&[*value])?;
                math::name(scope)?;
            }
            Analysis::NotAnalyzed { reason } => math::name(reason)?,
        }
        Ok(Self { at, signal, result })
    }
}

impl ProcessKind {
    pub fn hazards(self) -> &'static [HazardKind] {
        use HazardKind::*;
        match self {
            Self::SurfaceMarking => &[Pinch, SharpEdge],
            Self::HeatTransfer => &[Burns],
            Self::ImpactTransmission => &[Impact, Pinch],
            Self::ThreadEngagement => &[ThreadFailure, Pinch, StoredEnergy],
            Self::ElectricalContact => &[ElectricShock, Overcurrent],
            Self::StoredEnergy => &[StoredEnergy],
            Self::SharpEdge => &[SharpEdge],
            Self::Immersion => &[Ingress],
        }
    }
}

/// Исходные свойства сборки из каталога и геометрии. В список входят также
/// доступные сменные насадки, даже если для них отсутствует привязка процесса.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WorkSubject {
    pub name: String,
    pub properties: BTreeSet<ProcessKind>,
}

pub fn required_hazards(
    subjects: &[WorkSubject],
    bindings: &[ProcessBinding],
) -> Result<Vec<HazardEvidence>, ProcessError> {
    let mut names = BTreeSet::new();
    let mut requirements = BTreeSet::new();
    for subject in subjects {
        math::name(&subject.name)?;
        if !names.insert(&subject.name) {
            return Err(math::invalid("повторное имя элемента сборки"));
        }
        for kind in &subject.properties {
            requirements.extend(kind.hazards().iter().map(|h| (&subject.name, *h)));
        }
    }
    let mut process_names = BTreeSet::new();
    for binding in bindings {
        math::name(&binding.name)?;
        if !process_names.insert(&binding.name) {
            return Err(math::invalid("повторное имя процесса"));
        }
        for (port, target) in &binding.ports {
            math::name(port)?;
            math::name(target)?;
        }
        let subject = if let Some(attachment) = &binding.attachment {
            if !names.contains(attachment) {
                return Err(math::invalid("насадка процесса отсутствует в составе"));
            }
            attachment
        } else {
            if names.contains(&binding.name) {
                return Err(math::invalid("имя процесса совпало с элементом сборки"));
            }
            &binding.name
        };
        requirements.extend(binding.kind.hazards().iter().map(|h| (subject, *h)));
    }
    Ok(requirements
        .into_iter()
        .map(|(subject, kind)| HazardEvidence {
            subject: subject.clone(),
            kind,
            coverage: math::unknown("нет подтверждённого покрытия опасности в прогоне"),
        })
        .collect())
}

/// Сравнение с одним применимым ограничением. Для итогового вердикта
/// требуется покрытие всех обязательных опасностей и численная сходимость.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum HazardCheck {
    Violation { reason: String },
    WithinLimit { scope: String },
    NotAnalyzed { reason: String },
}

pub use immersion::ImmersionRegion;
