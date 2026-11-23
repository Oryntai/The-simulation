//! Интерфейс управления. Наблюдения содержат только показания датчиков и связи.
//!
//! Тип истинного состояния недоступен через зависимости крейта.
//!
//! ```compile_fail,E0432
//! use sim_runtime::Truth;
//! ```

pub mod builtin;
mod channels;
mod codec;
pub mod kinematics;
pub mod transport;

pub use channels::{CheckedObservation, ControlContract, InputChannel, InputKind, OutputChannel};

use sim_signal::{Command, Observation, Rng};

/// Ошибка обмена или исполнения, которую рантайм должен отразить в результате прогона.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ControlError {
    InvalidObservation(String),
    InvalidCommand(String),
    Transport(String),
    Execution(String),
}

impl std::fmt::Display for ControlError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let (kind, message) = match self {
            Self::InvalidObservation(message) => ("наблюдение", message),
            Self::InvalidCommand(message) => ("команда", message),
            Self::Transport(message) => ("транспорт", message),
            Self::Execution(message) => ("исполнение", message),
        };
        write!(f, "{kind}: {message}")
    }
}

impl std::error::Error for ControlError {}

/// Один вызов на такт управления. Время берётся из наблюдения.
/// Рантайм выдаёт отдельный воспроизводимый поток случайности.
pub trait Controller {
    fn step(&mut self, observation: &Observation, rng: &mut Rng) -> Result<Command, ControlError>;
}

/// Номер запроса в пределах сеанса. Номер связывает ответ с исходным наблюдением.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct RequestId(pub u64);

#[derive(Clone, Debug, PartialEq)]
pub struct ControlRequest {
    pub id: RequestId,
    pub observation: Observation,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ControlResponse {
    pub id: RequestId,
    pub command: Command,
}

/// Обмен сообщениями с внешним контроллером. Адаптер симуляции и адаптер железа
/// используют один протокол. Время доставки задаёт принимающая сторона.
pub trait ControlTransport {
    fn exchange(&mut self, request: &ControlRequest) -> Result<ControlResponse, ControlError>;
}

/// Настройки моделируемой задержки в одном направлении.
/// Отклонение равномерно в целых наносекундах от `-jitter_ns` до `+jitter_ns`.
/// При установке проверяются `jitter_ns <= latency_ns` и отсутствие переполнения суммы.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Delay {
    pub latency_ns: u64,
    pub jitter_ns: u64,
}
