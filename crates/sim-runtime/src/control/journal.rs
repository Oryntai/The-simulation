use sim_boards::{BoardInput, InputOutcome, OutputEvent};
use sim_control::{ControlRequest, ControlResponse};
use sim_radio::timed::PacketEvent;
use sim_signal::{ChannelId, SignalId, Time, fnv1a};
use sim_world::Simplification;

use crate::{Log, LogEnd, Runtime};

use super::FaultAction;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RunOrigin {
    Live,
    Replay,
    ExternalCommands,
    ProgrammaticController,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RunIdentity {
    pub build: String,
    pub world: u64,
    pub catalog: u64,
    pub control: u64,
    /// Схема парного опыта с неизменными параметрами, кроме амплитуд стимулов.
    pub paired_setup: u64,
    pub assets: Vec<(String, u64)>,
    pub task: u64,
    pub criteria: u64,
    pub seed: u64,
    pub mode: String,
    pub phase_horizon: Time,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MeasurementTime {
    pub captured_at: Time,
    pub delivered_at: Time,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandOrigin {
    Controller,
    Board,
    Stimulus,
    External,
    Fallback,
    Fault,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TimedCommand {
    pub sequence: u64,
    pub value: f64,
    pub origin: CommandOrigin,
    pub observed_at: Option<Time>,
    pub sent_at: Time,
    pub delivered_at: Time,
    pub expires_at: Option<Time>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ApplicationTiming {
    /// Левая граница уже успешно проинтегрированного интервала MuJoCo.
    PhysicsInterval,
    /// Такт публикации подтверждения. Внутренний такт модели может быть раньше.
    ModelReport,
    /// Модель сообщила сквозной номер и собственное время принятия.
    ModelAcceptance,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rejection {
    Missing,
    Expired,
    Overflow,
    Superseded,
    Disconnected,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BoardAction {
    Gpio { pin: u32, high: bool },
    Reset { config_hash: Option<u64> },
    Restart { config_hash: Option<u64> },
}

#[derive(Clone, Debug, PartialEq)]
pub enum EventKind {
    Measurement {
        signal: SignalId,
        time: MeasurementTime,
        valid: bool,
        value: f64,
    },
    ControllerInput {
        controller: String,
        signal: SignalId,
        time: MeasurementTime,
        valid: bool,
        value: f64,
    },
    RequestSent {
        controller: String,
        id: u64,
        due: Time,
    },
    RequestDelivered {
        controller: String,
        id: u64,
        protocol_id: u64,
        sent_at: Time,
    },
    RequestInput {
        controller: String,
        id: u64,
        signal: SignalId,
        captured_at: Time,
        value: f64,
    },
    ResponseSent {
        controller: String,
        id: u64,
        response: ControlResponse,
        due: Time,
    },
    ResponseDelivered {
        controller: String,
        id: u64,
        sent_at: Time,
    },
    Rejected {
        path: String,
        id: Option<u64>,
        reason: Rejection,
    },
    Failure {
        component: String,
        reason: String,
    },
    CommandDelivered {
        path: String,
        command: TimedCommand,
    },
    GoalDelivered {
        path: String,
        signal: SignalId,
        command: Option<TimedCommand>,
    },
    OutputInvalidated {
        path: String,
        reason: Rejection,
    },
    CommandPresented {
        path: String,
        channel: ChannelId,
        command: TimedCommand,
    },
    CommandApplied {
        path: String,
        channel: ChannelId,
        command: TimedCommand,
        effective_at: Time,
        timing: ApplicationTiming,
    },
    ModelAcceptanceUnattributed {
        path: String,
        channel: ChannelId,
        value: f64,
    },
    BoardActionRequested {
        id: String,
        board: String,
        action: BoardAction,
    },
    BoardActionApplied {
        id: String,
        board: String,
        cycle: u64,
        action: BoardAction,
    },
    BoardInput {
        board: String,
        cycle: u64,
        input: BoardInput,
        outcome: InputOutcome,
    },
    BoardOutput {
        board: String,
        event: OutputEvent,
        produced_at: Time,
    },
    PacketSent {
        route: String,
        id: u64,
        due: Time,
        bytes: Vec<u8>,
    },
    Packet {
        route: String,
        event: PacketEvent,
    },
    PacketDelivered {
        route: String,
        id: u64,
        sent_at: Time,
        bytes: Vec<u8>,
    },
    FaultRequested {
        id: String,
        target: String,
        action: FaultAction,
        active: bool,
    },
    FaultApplied {
        id: String,
        target: String,
        active: bool,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct Event {
    /// Время, когда событие стало доступно рантайму. Поля события отдельно
    /// содержат время захвата, отправки или интегрируемой границы физики.
    pub at: Time,
    pub kind: EventKind,
}

/// Доступ к результату экземпляра Runtime. Конструктора из файла журнала нет.
pub struct ControlRun<'a> {
    pub(super) runtime: &'a Runtime,
}

impl ControlRun<'_> {
    pub(super) fn state(&self) -> &super::Engine {
        self.runtime
            .state
            .control
            .as_ref()
            .expect("контур установлен")
    }

    pub fn identity(&self) -> &RunIdentity {
        &self.state().identity
    }
    pub fn events(&self) -> &[Event] {
        &self.state().events
    }
    pub fn log(&self) -> &Log {
        self.runtime.log()
    }
    pub fn truth_spec(&self, id: crate::TruthId) -> Option<&sim_signal::SignalSpec> {
        self.runtime.truth_spec(id)
    }
    pub fn end(&self) -> LogEnd {
        self.runtime.log().end()
    }
    pub fn origin(&self) -> RunOrigin {
        self.state().origin
    }
    /// Работник запускается как локальный процесс с правами пользователя.
    pub fn has_external_processes(&self) -> bool {
        !self.state().executables.is_empty()
    }
    pub fn reached_horizon(&self) -> bool {
        self.end().started
            && self.end().time == self.identity().phase_horizon
            && self.end().status == crate::Status::Running
    }
    pub fn simplifications(&self) -> &[Simplification] {
        self.runtime.simplifications()
    }
    pub fn paths(&self) -> &[ControlPath] {
        &self.state().paths
    }

    /// Маршруты с записанными временами доставки для численного воспроизведения.
    pub fn recorded_radio_routes(&self) -> impl Iterator<Item = &str> {
        self.state()
            .routes
            .iter()
            .filter(|route| route.slots.is_some())
            .map(|route| route.name.as_str())
    }

    pub fn fingerprint(&self) -> u64 {
        let header = format!("{:?}{:?}", self.identity(), self.simplifications());
        // Смешиваем фиксированные по длине отпечатки записей, не создавая
        // общую копию многомегабайтного журнала.
        fnv1a(
            self.log()
                .fingerprint()
                .to_le_bytes()
                .into_iter()
                .chain(header.bytes())
                .chain(
                    self.events()
                        .iter()
                        .flat_map(|event| fnv1a(format!("{event:?}").bytes()).to_le_bytes()),
                ),
        )
    }

    pub fn recording(&self) -> ControlRecording {
        ControlRecording {
            identity: self.identity().clone(),
            origin: self.origin(),
            log: self.log().clone(),
            exchanges: self.state().transcript.clone(),
            events: self.state().events.clone(),
            fingerprint: self.fingerprint(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ControlPath {
    pub name: String,
    pub producer: String,
    pub producer_index: u32,
    pub program_hash: u64,
    pub board_output_pin: Option<u32>,
    pub target: String,
    pub unit: String,
    pub channel: Option<ChannelId>,
    pub accepted_truth: Option<crate::TruthId>,
    pub feedback: Vec<SignalId>,
    pub fault_targets: Vec<FaultTarget>,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct FaultTarget {
    pub name: String,
    pub mechanisms: Vec<FaultMechanism>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum FaultMechanism {
    Drop,
    Stuck,
    PowerOff,
    Pause,
    Delay,
}

/// Запись предназначена для диагностического воспроизведения.
#[derive(Clone, Debug)]
pub struct ControlRecording {
    pub(super) identity: RunIdentity,
    pub(super) origin: RunOrigin,
    pub(super) log: Log,
    pub(crate) exchanges: std::rc::Rc<Vec<Exchange>>,
    pub(crate) events: std::rc::Rc<Vec<Event>>,
    pub(super) fingerprint: u64,
}

#[derive(Clone, Debug)]
pub(crate) struct Exchange {
    pub controller: String,
    pub request: ControlRequest,
    pub response: Result<ControlResponse, String>,
}
