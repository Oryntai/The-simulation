//! Контракты моделей, работающих на шкале времени симуляции.
//! Рантайм связывает модели с физикой и журналом. Контроллер работает через измерения.

pub mod attachments;
pub mod contact;
pub mod schematic;
pub mod work;

use sim_signal::{Rng, Time};
use sim_world::Provenance;

/// Параметр в единицах поля. При загрузке требуется конечный непустой
/// диапазон с `min <= nominal <= max` и источник для всего диапазона.
#[derive(Clone, Debug, PartialEq)]
pub struct SourcedRange {
    pub min: f64,
    pub nominal: f64,
    pub max: f64,
    pub provenance: Provenance,
}

impl SourcedRange {
    pub fn validate(&self) -> Result<(), ProcessError> {
        let description = match &self.provenance {
            Provenance::Datasheet(s)
            | Provenance::Paper(s)
            | Provenance::Measured(s)
            | Provenance::Derived(s)
            | Provenance::Estimate(s) => s,
        };
        if ![self.min, self.nominal, self.max]
            .into_iter()
            .all(f64::is_finite)
            || self.min > self.nominal
            || self.nominal > self.max
            || description.trim().is_empty()
            || description.chars().any(char::is_control)
        {
            return Err(ProcessError::InvalidInput(
                "недопустимый диапазон или источник".into(),
            ));
        }
        Ok(())
    }

    /// У производного значения в существующем контракте нет цепочки исходных данных.
    pub fn can_prove(&self) -> bool {
        self.validate().is_ok() && source_can_prove(&self.provenance)
    }
}

fn source_can_prove(source: &Provenance) -> bool {
    matches!(source, Provenance::Datasheet(s) | Provenance::Paper(s) | Provenance::Measured(s)
        if !s.trim().is_empty() && !s.chars().any(char::is_control))
}

impl contact::ResolvedContactPair {
    /// Повторная проверка источников при приёме описания из сборщика.
    pub fn supports_nominal_proof(&self) -> bool {
        use contact::{ContactFriction, FrictionReference, NormalResponse};

        let exact = |value: &SourcedRange| value.can_prove() && value.min == value.max;
        let normal = match &self.parameters.normal {
            NormalResponse::Restitution {
                coefficient,
                duration_s,
            } => exact(coefficient) && exact(duration_s),
            NormalResponse::SpringDamper {
                stiffness_n_m,
                damping_ns_m,
            } => {
                exact(stiffness_n_m)
                    && exact(damping_ns_m)
                    && self
                        .calibration
                        .effective_mass_kg
                        .as_ref()
                        .is_some_and(exact)
            }
            NormalResponse::Direct { .. } => false,
        };
        let friction = match &self.parameters.friction {
            ContactFriction::NormalOnly => true,
            ContactFriction::Sliding { sliding } => sliding.iter().all(exact),
            ContactFriction::SlidingAndTorsion { sliding, torsion_m } => {
                sliding.iter().all(exact) && exact(torsion_m)
            }
            ContactFriction::SlidingTorsionAndRolling {
                sliding,
                torsion_m,
                rolling_m,
            } => sliding.iter().chain(rolling_m).all(exact) && exact(torsion_m),
        };
        let friction_reference = match &self.calibration.friction_reference {
            FrictionReference::UseNormal => true,
            FrictionReference::DirectDamping(value) => exact(value),
        };
        normal
            && friction
            && friction_reference
            && source_can_prove(&self.calibration.impedance_source)
            && self
                .calibration
                .effective_mass_kg
                .as_ref()
                .is_none_or(exact)
    }
}

/// Область применимости указывается вместе с числом или причиной его отсутствия.
/// Сам факт получения числа не является вердиктом о безопасности.
#[derive(Clone, Debug, PartialEq)]
pub enum Analysis<T> {
    Analyzed { value: T, scope: String },
    NotAnalyzed { reason: String },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProcessError {
    InvalidInput(String),
    Execution(String),
}

impl std::fmt::Display for ProcessError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let message = match self {
            Self::InvalidInput(message) | Self::Execution(message) => message,
        };
        f.write_str(message)
    }
}

impl std::error::Error for ProcessError {}

/// Границы одного шага. Требуется `start < end`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Interval {
    pub start: Time,
    pub end: Time,
}

/// Сила и момент относительно центра масс тела, компоненты в системе мира.
#[derive(Clone, Debug, PartialEq)]
pub struct BodyLoad {
    pub body: String,
    pub force_n: [f64; 3],
    pub torque_nm: [f64; 3],
}

/// Клон должен владеть независимым изменяемым состоянием. Вход содержит
/// зафиксированные данные за шаг, выход передаётся рантайму в фиксированном порядке.
/// Ошибка останавливает прогон без применения частичного выхода.
pub trait StepModel: Clone {
    type Input;
    type Output;

    fn advance(
        &mut self,
        interval: Interval,
        input: &Self::Input,
        rng: &mut Rng,
    ) -> Result<Self::Output, ProcessError>;
}
