//! Сборка электроники и управление в общей шкале времени.
mod board;
mod bus;
pub mod external_board;
mod install;
mod journal;
mod program;
mod schema;

pub use install::{
    ControlAssets, ControllerBindings, FileAssets, ModelPort, RunSpec, install, install_replay,
    install_with_controllers,
};
pub use journal::*;
pub use schema::{ControlConfig, FaultAction};
pub use sim_boards::{BoardOutput as BoardEvent, ResetCause};
pub use sim_radio::timed::Outcome as PacketOutcome;

use std::cmp::Reverse;
use std::collections::{BTreeMap, BTreeSet, BinaryHeap, VecDeque};
use std::path::PathBuf;
use std::rc::Rc;

use sim_control::{
    ControlContract, ControlRequest, ControlResponse, ControlTransport, Controller, Delay,
    InputKind, RequestId,
};
use sim_signal::{ChannelId, Clock, Command, Observation, Phase, Rng, Sample, SignalId, Time};

use crate::{ObservationBus, Runtime, checked_due};
use board::{Board, Route, ScheduledBoardAction};
use schema::{ControllerSpec, DriverSpec, FaultSpec, TargetKind};

pub const EXTERNAL_ISOLATION_GAP: &str = "изоляция внешнего процесса средствами ОС не подтверждена: файлы и ресурсы ОС доступны за пределами протокола";

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Schedule {
    Fault,
    Stimulus,
    PacketSend,
    Packet,
    BoardAction,
    Board,
    Response,
    Exchange,
    Controller,
    Goal,
}

impl Schedule {
    fn phase(self) -> Phase {
        match self {
            Self::Fault | Self::Stimulus => Phase::Physics,
            Self::PacketSend | Self::Packet | Self::BoardAction | Self::Board => Phase::Links,
            Self::Response | Self::Exchange | Self::Controller => Phase::Controllers,
            Self::Goal => Phase::Actuators,
        }
    }
}

#[derive(Clone)]
struct Node {
    spec: ControllerSpec,
    clock: Clock,
    contract: ControlContract,
    outputs: Vec<usize>,
    builtin: Option<Box<dyn LocalController>>,
    rng: Rng,
    delay_in: Rng,
    delay_out: Rng,
    requests: BTreeMap<(Time, u64), Request>,
    answers: BTreeMap<(Time, u64), Answer>,
    next_request: u64,
    last_sent_due: Time,
    last_answer: Option<Time>,
    protocol_sequence: u64,
}

trait LocalController: Controller {
    fn snapshot(&self) -> Box<dyn LocalController>;
}

impl<T: Controller + Clone + 'static> LocalController for T {
    fn snapshot(&self) -> Box<dyn LocalController> {
        Box::new(self.clone())
    }
}

impl Clone for Box<dyn LocalController> {
    fn clone(&self) -> Self {
        self.snapshot()
    }
}

#[derive(Clone)]
struct Request {
    observation: Observation,
    sent_at: Time,
}

#[derive(Clone)]
struct Answer {
    observed_at: Time,
    sent_at: Time,
    response: ControlResponse,
}

#[derive(Clone, Copy)]
enum Target {
    Goal(SignalId),
    Channel(ChannelId, Option<crate::TruthId>),
}

#[derive(Clone)]
struct Output {
    id: ChannelId,
    name: String,
    target: Target,
    target_name: String,
    kind: TargetKind,
    max_age: u64,
    fallback: Option<f64>,
    driver: Option<DriverSpec>,
    latest: Option<TimedCommand>,
    fault_value: Option<TimedCommand>,
    fallback_value: Option<TimedCommand>,
    presented: Option<TimedCommand>,
    waiting_for_acceptance: VecDeque<(Time, TimedCommand)>,
    applied: Option<TimedCommand>,
    ack_seen: Option<f64>,
}

#[derive(Clone)]
struct FaultEdge {
    index: usize,
    active: bool,
}

#[derive(Clone)]
struct Stimulus {
    at: Time,
    target: ChannelId,
    value: f64,
}

#[derive(Clone)]
struct Transmission {
    at: Time,
    route: usize,
    bytes: Vec<u8>,
}

#[derive(Clone)]
pub(crate) struct Engine {
    identity: RunIdentity,
    origin: RunOrigin,
    paths: Vec<ControlPath>,
    events: Rc<Vec<Event>>,
    transcript: Rc<Vec<Exchange>>,
    playback: Option<(ControlRecording, usize)>,
    executables: BTreeMap<usize, (PathBuf, u64)>,
    queue: BinaryHeap<Reverse<(Time, Schedule, usize)>>,
    nodes: Vec<Node>,
    outputs: Vec<Output>,
    channels: BTreeMap<ChannelId, usize>,
    stimuli_channels: BTreeSet<ChannelId>,
    stimulus_values: BTreeMap<ChannelId, (String, TimedCommand)>,
    stimulus_integrated: BTreeMap<ChannelId, u64>,
    stimulus_pending: Vec<(ChannelId, String, TimedCommand)>,
    boards: Vec<Board>,
    board_actions: Vec<ScheduledBoardAction>,
    routes: Vec<Route>,
    faults: Vec<FaultSpec>,
    fault_edges: Vec<FaultEdge>,
    active: BTreeMap<String, usize>,
    applied_fault: BTreeMap<String, (usize, bool)>,
    recovering: BTreeMap<String, usize>,
    sensor_barriers: BTreeMap<SignalId, Time>,
    observation_writers: BTreeMap<SignalId, u32>,
    input_shapes: BTreeMap<SignalId, Option<sim_signal::PixelFormat>>,
    goals: BTreeSet<SignalId>,
    stimuli: Vec<Stimulus>,
    transmissions: Vec<Transmission>,
    sequence: u64,
    /// Сила фиксируется на левой границе очередного интервала физики.
    direct: Vec<(ChannelId, f64)>,
    latched_forces: Vec<(ChannelId, f64)>,
    latched_start: Time,
    direct_pending: Vec<(usize, TimedCommand)>,
    numeric_commands: Vec<crate::LogEntry>,
}

impl Engine {
    pub(crate) fn wired_delay(&self, name: &str) -> Option<u64> {
        (self.origin == RunOrigin::Live).then_some(())?;
        self.routes
            .iter()
            .find(|route| route.name == name && !route.is_radio() && route.slots.is_none())
            .map(|route| route.delay_ns)
    }

    pub(crate) fn set_wired_delay(&mut self, name: &str, delay_ns: u64) {
        let route = self
            .routes
            .iter_mut()
            .find(|route| route.name == name)
            .expect("адрес задержки уже проверен");
        route.delay_ns = delay_ns;
        self.identity.control = sim_signal::fnv1a(
            self.identity
                .control
                .to_le_bytes()
                .into_iter()
                .chain(name.bytes())
                .chain(delay_ns.to_le_bytes()),
        );
    }

    pub(crate) fn require_modeled_radio(&self, name: &str) -> Result<(), String> {
        if self
            .routes
            .iter()
            .any(|route| route.name == name && route.radio.is_some())
        {
            Ok(())
        } else {
            Err(format!(
                "для среды маршрута «{name}» нужна модель sim-radio"
            ))
        }
    }

    pub(crate) fn verify_executables(&self) -> Result<(), String> {
        for (path, expected) in self.executables.values() {
            let image = std::fs::read(path)
                .map_err(|error| format!("исполняемый файл {}: {error}", path.display()))?;
            if sim_signal::fnv1a(image) != *expected {
                return Err(format!("исполняемый файл {} изменился", path.display()));
            }
        }
        Ok(())
    }

    pub(crate) fn horizon(&self) -> Time {
        self.playback
            .as_ref()
            .map_or(self.identity.phase_horizon, |(r, _)| r.log.end().time)
    }

    pub(crate) fn next_event(&self) -> Option<Time> {
        self.queue.peek().map(|event| event.0.0)
    }

    fn push(&mut self, at: Time, kind: EventKind) {
        Rc::make_mut(&mut self.events).push(Event { at, kind });
    }

    fn schedule(&mut self, at: Time, role: Schedule, id: usize) {
        self.queue.push(Reverse((at, role, id)));
    }

    pub(crate) fn phase(
        &mut self,
        runtime: &mut Runtime,
        at: Time,
        phase: Phase,
    ) -> Result<(), String> {
        while let Some(&Reverse((due, role, id))) = self.queue.peek() {
            if due != at || role.phase() != phase {
                break;
            }
            self.queue.pop();
            match role {
                Schedule::Fault => self.change_fault(runtime, at, id)?,
                Schedule::Stimulus => {
                    let stimulus = self.stimuli[id].clone();
                    runtime.apply_commands(
                        &[(stimulus.target, stimulus.value)],
                        crate::CommandSource::External,
                    );
                    let command =
                        self.command(stimulus.value, CommandOrigin::Stimulus, at, at, None)?;
                    let path = runtime.log().header().commands[stimulus.target.0 as usize]
                        .name
                        .clone();
                    self.stimulus_values
                        .insert(stimulus.target, (path.clone(), command));
                    self.push(at, EventKind::CommandDelivered { path, command });
                }
                Schedule::PacketSend => {
                    let tx = self.transmissions[id].clone();
                    self.send_packet(at, tx.route, tx.bytes)?;
                }
                Schedule::Packet => self.advance_route(runtime, at, id)?,
                Schedule::BoardAction => self.request_board_action(at, id),
                Schedule::Board => self.tick_board(&runtime.state.bus, at, id)?,
                Schedule::Response => self.deliver_response(at, id)?,
                Schedule::Exchange => self.exchange(runtime, at, id)?,
                Schedule::Controller => self.tick_controller(&runtime.state.bus, at, id)?,
                Schedule::Goal => self.deliver_goal(runtime, at, id)?,
            }
        }
        for entry in std::mem::take(&mut self.numeric_commands) {
            runtime.state.log.push(entry);
        }
        Ok(())
    }

    pub(crate) fn observation_source(
        &mut self,
        writer: u32,
        signal: SignalId,
        format: Option<sim_signal::PixelFormat>,
    ) -> Result<(), String> {
        if self.goals.contains(&signal)
            || self
                .observation_writers
                .get(&signal)
                .is_some_and(|&old| old != writer)
        {
            return Err(format!("конфликт писателей наблюдения {}", signal.0));
        }
        if self
            .input_shapes
            .get(&signal)
            .is_some_and(|declared| *declared != format)
        {
            return Err(format!(
                "формат наблюдения {} расходится с проводкой",
                signal.0
            ));
        }
        self.observation_writers.insert(signal, writer);
        Ok(())
    }

    pub(crate) fn measurement(
        &mut self,
        id: SignalId,
        captured: Time,
        at: Time,
        valid: bool,
        value: f64,
    ) {
        self.push(
            at,
            EventKind::Measurement {
                signal: id,
                time: MeasurementTime {
                    captured_at: captured,
                    delivered_at: at,
                },
                valid,
                value,
            },
        );
    }

    fn output_fault(&mut self, at: Time, id: usize) -> Result<Option<FaultAction>, String> {
        let out = &self.outputs[id];
        let wire = format!("wire:{}", out.name);
        let target = match out.kind {
            TargetKind::Power => format!("power:{}", out.target_name),
            TargetKind::Drive | TargetKind::Effort => format!("actuator:{}", out.target_name),
            TargetKind::Goal => format!("channel:{}", out.target_name),
        };
        let wire_fault = self.use_fault(at, &wire);
        let target_fault = self.use_fault(at, &target);
        if wire_fault.is_some() && target_fault.is_some() {
            return Err(format!("одновременные отказы в «{wire}» и «{target}»"));
        }
        Ok(wire_fault.or(target_fault))
    }

    fn use_fault(&mut self, at: Time, target: &str) -> Option<FaultAction> {
        let (index, active) = if let Some(&index) = self.active.get(target) {
            (index, true)
        } else {
            (self.recovering.remove(target)?, false)
        };
        if self
            .applied_fault
            .insert(target.to_owned(), (index, active))
            != Some((index, active))
        {
            self.push(
                at,
                EventKind::FaultApplied {
                    id: self.faults[index].id.clone(),
                    target: target.into(),
                    active,
                },
            );
        }
        active.then(|| self.faults[index].action)
    }

    fn change_fault(&mut self, runtime: &Runtime, at: Time, index: usize) -> Result<(), String> {
        let edge = &self.fault_edges[index];
        let fault = self.faults[edge.index].clone();
        let active = edge.active;
        if active {
            self.active.insert(fault.target.clone(), edge.index);
            self.recovering.remove(&fault.target);
        } else {
            self.active.remove(&fault.target);
            self.recovering.insert(fault.target.clone(), edge.index);
        }
        self.push(
            at,
            EventKind::FaultRequested {
                id: fault.id.clone(),
                target: fault.target.clone(),
                action: fault.action,
                active,
            },
        );
        if active
            && let Some(name) = fault.target.strip_prefix("controller:")
            && let Some(index) = self.nodes.iter().position(|node| node.spec.name == name)
        {
            let requests = std::mem::take(&mut self.nodes[index].requests);
            let answers = std::mem::take(&mut self.nodes[index].answers);
            self.queue.retain(|entry| !matches!(entry.0, (_, Schedule::Exchange | Schedule::Response, id) if id == index));
            for (_, id) in requests.keys().chain(answers.keys()).copied() {
                self.rejection(at, name.into(), Some(id), Rejection::Disconnected);
            }
        }
        if let Some(signal) = fault
            .target
            .strip_prefix("sensor:")
            .and_then(|name| runtime.observation_id(name))
        {
            self.sensor_barriers.insert(signal, at);
        }
        if fault.action == FaultAction::Drop
            && let Some(name) = fault.target.strip_prefix("channel:")
            && let Some(id) = self.routes.iter().position(|route| route.name == name)
        {
            self.route_connection(at, id, active)?;
        }
        // У тех же целей инвалидируем уже доставленную, но ещё не принятую команду.
        for id in 0..self.outputs.len() {
            let out = &self.outputs[id];
            let targeted = fault.target.strip_prefix("wire:") == Some(&out.name)
                || (fault.target.split_once(':').is_some_and(|(kind, name)| {
                    name == out.target_name && matches!(kind, "actuator" | "power" | "channel")
                }));
            let owner = &self.paths[id].producer;
            let paused = fault.target.strip_prefix("controller:") == Some(owner)
                || fault.target.strip_prefix("board:") == Some(owner);
            if targeted || paused {
                self.outputs[id].latest = None;
                self.outputs[id].fault_value = None;
                self.outputs[id].fallback_value = None;
                if matches!(self.outputs[id].target, Target::Goal(_)) {
                    self.schedule(at, Schedule::Goal, id);
                }
            }
        }
        Ok(())
    }

    fn command(
        &mut self,
        value: f64,
        origin: CommandOrigin,
        sent_at: Time,
        at: Time,
        expires_at: Option<Time>,
    ) -> Result<TimedCommand, String> {
        if !value.is_finite() {
            return Err("неконечное значение команды".into());
        }
        let sequence = self.sequence;
        self.sequence = sequence.checked_add(1).ok_or("исчерпаны номера команд")?;
        Ok(TimedCommand {
            sequence,
            value,
            origin,
            observed_at: None,
            sent_at,
            delivered_at: at,
            expires_at,
        })
    }

    fn rejection(&mut self, at: Time, path: String, id: Option<u64>, reason: Rejection) {
        self.push(at, EventKind::Rejected { path, id, reason });
    }

    fn invalidate(&mut self, at: Time, index: usize, reason: Rejection) {
        self.outputs[index].latest = None;
        self.push(
            at,
            EventKind::OutputInvalidated {
                path: self.outputs[index].name.clone(),
                reason,
            },
        );
        if matches!(self.outputs[index].target, Target::Goal(_)) {
            self.schedule(at, Schedule::Goal, index);
        }
    }

    fn deliver_output(
        &mut self,
        at: Time,
        index: usize,
        observed_at: Time,
        sent_at: Time,
        value: Option<f64>,
        origin: CommandOrigin,
    ) -> Result<(), String> {
        let deadline = add(observed_at, self.outputs[index].max_age)?;
        let Some(value) = value else {
            self.invalidate(at, index, Rejection::Missing);
            return Ok(());
        };
        if deadline < at {
            self.invalidate(at, index, Rejection::Expired);
            return Ok(());
        }
        if self.outputs[index].kind == TargetKind::Power && value != 0.0 && value != 1.0 {
            return Err("команда питания должна равняться 0 или 1".into());
        }
        let mut command = self.command(value, origin, sent_at, at, Some(deadline))?;
        command.observed_at = Some(observed_at);
        let path = self.outputs[index].name.clone();
        self.outputs[index].latest = Some(command);
        if let Some(channel) = self.paths[index].channel {
            self.numeric_commands.push(crate::LogEntry {
                at,
                kind: crate::LogKind::Command(crate::CommandSource::Controller(
                    self.paths[index].producer_index,
                )),
                id: channel.0,
                value,
                valid: true,
            });
        }
        self.push(at, EventKind::CommandDelivered { path, command });
        if matches!(self.outputs[index].target, Target::Goal(_)) {
            self.schedule(at, Schedule::Goal, index);
            if let Some(expiry) = deadline.0.checked_add(1).map(Time) {
                self.schedule(expiry, Schedule::Goal, index);
            }
        }
        Ok(())
    }

    fn choice(&mut self, at: Time, index: usize) -> Result<Option<TimedCommand>, String> {
        let action = self.output_fault(at, index)?;
        let (value, fault) = match action {
            Some(FaultAction::Stuck(value)) => (Some(value), true),
            Some(FaultAction::PowerOff) => (Some(0.0), true),
            Some(FaultAction::Drop) => (self.outputs[index].fallback, false),
            Some(FaultAction::Pause | FaultAction::Delay(_)) => {
                unreachable!("проверено при установке")
            }
            None => {
                if let Some(command) = self.outputs[index].latest {
                    if command.expires_at.is_none_or(|expires| at <= expires) {
                        self.outputs[index].fallback_value = None;
                        return Ok(Some(command));
                    }
                    // При проверке с такта исполнительного объекта отдельный такт цели не нужен.
                    self.outputs[index].latest = None;
                    self.push(
                        at,
                        EventKind::OutputInvalidated {
                            path: self.outputs[index].name.clone(),
                            reason: Rejection::Expired,
                        },
                    );
                }
                (self.outputs[index].fallback, false)
            }
        };
        let Some(value) = value else {
            return Ok(None);
        };
        let previous = if fault {
            self.outputs[index].fault_value
        } else {
            self.outputs[index].fallback_value
        };
        if let Some(command) = previous
            && command.value.to_bits() == value.to_bits()
        {
            return Ok(Some(command));
        }
        let command = self.command(
            value,
            if fault {
                CommandOrigin::Fault
            } else {
                CommandOrigin::Fallback
            },
            at,
            at,
            None,
        )?;
        if fault {
            self.outputs[index].fault_value = Some(command);
        } else {
            self.outputs[index].fallback_value = Some(command);
        }
        Ok(Some(command))
    }

    fn deliver_goal(&mut self, runtime: &mut Runtime, at: Time, id: usize) -> Result<(), String> {
        let Target::Goal(signal) = self.outputs[id].target else {
            unreachable!()
        };
        let choice = self.choice(at, id)?;
        if self.outputs[id].applied == choice
            && (choice.is_some() || runtime.state.bus.sample(signal).is_some())
        {
            return Ok(());
        }
        self.outputs[id].applied = choice;
        // Для цели временем происхождения является отправка команды. Старый
        // результат не получает новый срок из-за доставки в следующий уровень.
        let captured_at = choice.map_or(at, |command| command.sent_at);
        let sample = Sample {
            signal,
            value: choice.map_or(0.0, |command| command.value),
            at,
            valid: choice.is_some(),
        };
        let ordered = runtime.publish_sample(sample, captured_at);
        self.measurement(
            signal,
            captured_at,
            at,
            sample.valid && ordered,
            sample.value,
        );
        if !ordered {
            self.outputs[id].latest = None;
            self.outputs[id].applied = None;
            self.rejection(
                at,
                self.paths[id].name.clone(),
                choice.map(|c| c.sequence),
                Rejection::Superseded,
            );
            return Ok(());
        }
        self.push(
            at,
            EventKind::GoalDelivered {
                path: self.paths[id].name.clone(),
                signal,
                command: choice,
            },
        );
        Ok(())
    }

    fn read_input(
        &mut self,
        bus: &ObservationBus,
        at: Time,
        signal: SignalId,
        name: &str,
        age: u64,
        consumer: &str,
    ) -> Option<Sample> {
        let action = self.use_fault(at, &format!("sensor:{name}"));
        let (mut sample, captured) = bus.sample(signal)?;
        let time = MeasurementTime {
            captured_at: captured,
            delivered_at: sample.at,
        };
        if let Some(FaultAction::Stuck(value)) = action {
            sample.value = value;
        }
        let barrier = self.sensor_barriers.get(&signal).copied();
        sample.valid &= match action {
            Some(FaultAction::Drop) => false,
            Some(FaultAction::Stuck(_)) => at.0 - captured.0 <= age,
            _ => barrier.is_none_or(|edge| captured >= edge) && at.0 - captured.0 <= age,
        };
        self.push(
            at,
            EventKind::ControllerInput {
                controller: consumer.into(),
                signal,
                time,
                valid: sample.valid,
                value: sample.value,
            },
        );
        // Внешняя программа и sim-control проверяют возраст по Sample.at.
        // Исходный момент доставки остаётся в MeasurementTime и основном журнале.
        sample.at = captured;
        Some(sample)
    }

    fn observations(&mut self, bus: &ObservationBus, at: Time, id: usize) -> Observation {
        let specs: Vec<_> = self.nodes[id].contract.inputs().cloned().collect();
        let name = self.nodes[id].spec.name.clone();
        let mut observation = Observation {
            time: at,
            ..Observation::default()
        };
        for input in specs {
            match input.kind {
                InputKind::Scalar { max_age_ns } => {
                    if let Some(sample) =
                        self.read_input(bus, at, input.id, &input.name, max_age_ns, &name)
                    {
                        observation.samples.push(sample);
                    }
                }
                InputKind::Image {
                    format: _,
                    max_capture_age_ns,
                    max_delivery_age_ns,
                } => {
                    let action = self.use_fault(at, &format!("sensor:{}", input.name));
                    if let Some(mut frame) = bus.frames[input.id.0 as usize].clone() {
                        frame.valid &= action.is_none()
                            && self
                                .sensor_barriers
                                .get(&input.id)
                                .is_none_or(|edge| frame.captured >= *edge)
                            && at.0 - frame.captured.0 <= max_capture_age_ns
                            && at.0 - frame.at.0 <= max_delivery_age_ns;
                        self.push(
                            at,
                            EventKind::ControllerInput {
                                controller: name.clone(),
                                signal: input.id,
                                time: MeasurementTime {
                                    captured_at: frame.captured,
                                    delivered_at: frame.at,
                                },
                                valid: frame.valid,
                                value: frame.fingerprint(),
                            },
                        );
                        observation.frames.push(frame);
                    }
                }
            }
        }
        observation
    }

    fn tick_controller(&mut self, bus: &ObservationBus, at: Time, id: usize) -> Result<(), String> {
        self.nodes[id].clock.advance();
        if let Some(due) = checked_due(&self.nodes[id].clock)? {
            self.schedule(due, Schedule::Controller, id);
        }
        let name = self.nodes[id].spec.name.clone();
        if self.use_fault(at, &format!("controller:{name}")).is_some() {
            for output in self.nodes[id].outputs.clone() {
                self.invalidate(at, output, Rejection::Disconnected);
            }
            return Ok(());
        }
        let observation = self.observations(bus, at, id);
        let request_link = if self.nodes[id].spec.external.is_some() {
            self.use_fault(at, &format!("transport:{name}/request"))
        } else {
            None
        };
        if request_link == Some(FaultAction::Drop) {
            self.rejection(
                at,
                format!("transport:{name}/request"),
                None,
                Rejection::Disconnected,
            );
            for output in self.nodes[id].outputs.clone() {
                self.invalidate(at, output, Rejection::Disconnected);
            }
            return Ok(());
        }
        let node = &mut self.nodes[id];
        node.contract
            .observation(&observation)
            .map_err(|e| e.to_string())?;
        if let Some(builtin) = &mut node.builtin {
            let command = builtin
                .step(&observation, &mut node.rng)
                .map_err(|e| format!("{name}: {e}"))?;
            return self.deliver_command(at, id, at, at, command);
        }
        let spec = node.spec.external.as_ref().expect("внешний контроллер");
        if node.requests.len() + node.answers.len() >= spec.queue_capacity {
            self.rejection(at, name, None, Rejection::Overflow);
            for output in self.nodes[id].outputs.clone() {
                self.invalidate(at, output, Rejection::Overflow);
            }
            return Ok(());
        }
        let serial = node.next_request;
        node.next_request = serial.checked_add(1).ok_or("исчерпаны номера запросов")?;
        let ready = delayed(at, spec.incoming, &mut node.delay_in)?;
        let ready = if let Some(FaultAction::Delay(ns)) = request_link {
            add(ready, ns)?
        } else {
            ready
        };
        let due = first_tick(spec.request_rate, ready.max(node.last_sent_due), true)?;
        node.last_sent_due = due;
        node.requests.insert(
            (due, serial),
            Request {
                observation,
                sent_at: at,
            },
        );
        self.schedule(due, Schedule::Exchange, id);
        self.push(
            at,
            EventKind::RequestSent {
                controller: name,
                id: serial,
                due,
            },
        );
        Ok(())
    }

    fn exchange(&mut self, runtime: &mut Runtime, at: Time, id: usize) -> Result<(), String> {
        let Some(((due, number), request)) = self.nodes[id].requests.pop_first() else {
            return Ok(());
        };
        debug_assert_eq!(due, at);
        let name = self.nodes[id].spec.name.clone();
        let spec = self.nodes[id]
            .spec
            .external
            .as_ref()
            .expect("внешний контроллер");
        let expired = at > add(request.sent_at, spec.request_max_age_ns)?;
        let disconnected = self.use_fault(at, &format!("controller:{name}")).is_some()
            || self.use_fault(at, &format!("transport:{name}/request")) == Some(FaultAction::Drop);
        if expired || disconnected {
            let reason = if expired {
                Rejection::Expired
            } else {
                Rejection::Disconnected
            };
            self.rejection(at, name, Some(number), reason);
            for output in self.nodes[id].outputs.clone() {
                self.invalidate(at, output, reason);
            }
            return Ok(());
        }
        // Номера протокола выдаются только для доставленных запросов: потерянные
        // сообщения очереди не создают дыру в последовательности работника.
        let protocol_id = self.nodes[id].protocol_sequence;
        let contract = &self.nodes[id].contract;
        let observation = Observation {
            time: at,
            ..request.observation
        };
        let request_msg = ControlRequest {
            id: RequestId(protocol_id),
            observation: contract
                .observation(&observation)
                .map_err(|e| e.to_string())?
                .usable(),
        };
        self.nodes[id].protocol_sequence = protocol_id
            .checked_add(1)
            .ok_or("исчерпаны номера протокола")?;
        self.push(
            at,
            EventKind::RequestDelivered {
                controller: name.clone(),
                id: number,
                protocol_id,
                sent_at: request.sent_at,
            },
        );
        for (signal, captured_at, value) in request_msg
            .observation
            .samples
            .iter()
            .map(|sample| (sample.signal, sample.at, sample.value))
            .chain(
                request_msg
                    .observation
                    .frames
                    .iter()
                    .map(|frame| (frame.signal, frame.captured, frame.fingerprint())),
            )
        {
            self.push(
                at,
                EventKind::RequestInput {
                    controller: name.clone(),
                    id: number,
                    signal,
                    captured_at,
                    value,
                },
            );
        }
        let result = if let Some((recording, cursor)) = &mut self.playback {
            let entry = recording
                .exchanges
                .get(*cursor)
                .ok_or("в записи нет внешнего ответа")?;
            if entry.controller != name
                || format!("{:?}", entry.request) != format!("{request_msg:?}")
            {
                return Err("запрос во время воспроизведения отличается от записи".into());
            }
            *cursor += 1;
            entry.response.clone()
        } else {
            runtime
                .processes
                .get_mut(&id)
                .ok_or("внешний процесс не открыт")?
                .exchange(&request_msg)
                .map_err(|e| e.to_string())
        };
        Rc::make_mut(&mut self.transcript).push(Exchange {
            controller: name.clone(),
            request: request_msg,
            response: result.clone(),
        });
        let response = result.map_err(|e| format!("{name}: {e}"))?;
        if response.id != RequestId(protocol_id) {
            return Err(format!("{name}: ответ не соответствует номеру запроса"));
        }
        self.nodes[id]
            .contract
            .validate_command(&response.command, request.sent_at, request.sent_at)
            .map_err(|e| e.to_string())?;
        let response_link = self.use_fault(at, &format!("transport:{name}/response"));
        if response_link == Some(FaultAction::Drop) {
            self.rejection(
                at,
                format!("transport:{name}/response"),
                Some(number),
                Rejection::Disconnected,
            );
            for output in self.nodes[id].outputs.clone() {
                self.invalidate(at, output, Rejection::Disconnected);
            }
            return Ok(());
        }
        let node = &mut self.nodes[id];
        let delay = node
            .spec
            .external
            .as_ref()
            .expect("внешний контроллер")
            .outgoing;
        let ready = delayed(at, delay, &mut node.delay_out)?;
        let ready = if let Some(FaultAction::Delay(ns)) = response_link {
            add(ready, ns)?
        } else {
            ready
        };
        let due = first_tick(node.spec.rate, ready, true)?;
        node.answers.insert(
            (due, number),
            Answer {
                observed_at: request.sent_at,
                sent_at: at,
                response: response.clone(),
            },
        );
        self.schedule(due, Schedule::Response, id);
        self.push(
            at,
            EventKind::ResponseSent {
                controller: name,
                id: number,
                response,
                due,
            },
        );
        Ok(())
    }

    fn deliver_response(&mut self, at: Time, id: usize) -> Result<(), String> {
        let Some(((due, number), answer)) = self.nodes[id].answers.pop_first() else {
            return Ok(());
        };
        debug_assert_eq!(due, at);
        let name = self.nodes[id].spec.name.clone();
        if self.use_fault(at, &format!("controller:{name}")).is_some()
            || self.use_fault(at, &format!("transport:{name}/response")) == Some(FaultAction::Drop)
        {
            self.rejection(at, name, Some(number), Rejection::Disconnected);
            for output in self.nodes[id].outputs.clone() {
                self.invalidate(at, output, Rejection::Disconnected);
            }
            return Ok(());
        }
        let node = &mut self.nodes[id];
        if node
            .last_answer
            .is_some_and(|previous| answer.observed_at <= previous)
        {
            self.rejection(at, name, Some(number), Rejection::Superseded);
            return Ok(());
        }
        node.last_answer = Some(answer.observed_at);
        self.push(
            at,
            EventKind::ResponseDelivered {
                controller: name,
                id: number,
                sent_at: answer.sent_at,
            },
        );
        self.deliver_command(
            at,
            id,
            answer.observed_at,
            answer.sent_at,
            answer.response.command,
        )
    }

    fn deliver_command(
        &mut self,
        at: Time,
        id: usize,
        observed_at: Time,
        sent_at: Time,
        command: Command,
    ) -> Result<(), String> {
        let node = &self.nodes[id];
        node.contract
            .validate_command(&command, observed_at, observed_at)
            .map_err(|e| e.to_string())?;
        // Срок любой составляющей делает весь ответ непригодным.
        let mut expired = false;
        for &(channel, _) in &command.targets {
            let output = node
                .outputs
                .iter()
                .find(|&&output| self.outputs[output].id == channel)
                .expect("проверено контрактом");
            expired |= at > add(observed_at, self.outputs[*output].max_age)?;
        }
        for output in node.outputs.clone() {
            if expired {
                self.invalidate(at, output, Rejection::Expired);
                continue;
            }
            let channel = self.outputs[output].id;
            let value = command
                .targets
                .iter()
                .find(|&&(id, _)| id == channel)
                .map(|&(_, value)| value);
            self.deliver_output(
                at,
                output,
                observed_at,
                sent_at,
                value,
                CommandOrigin::Controller,
            )?;
        }
        Ok(())
    }

    pub(crate) fn owns(&self, channel: ChannelId) -> bool {
        self.channels.contains_key(&channel) || self.stimuli_channels.contains(&channel)
    }

    pub(crate) fn record_external(
        &mut self,
        at: Time,
        channels: &[(ChannelId, f64)],
        names: &[sim_signal::SignalSpec],
    ) -> Result<(), String> {
        if self.origin == RunOrigin::Replay {
            return Err("команда во время воспроизведения недоступна".into());
        }
        self.origin = RunOrigin::ExternalCommands;
        for &(channel, value) in channels {
            let path = names[channel.0 as usize].name.clone();
            let command = self.command(value, CommandOrigin::External, at, at, None)?;
            self.stimulus_values
                .insert(channel, (path.clone(), command));
            self.push(at, EventKind::CommandDelivered { path, command });
        }
        Ok(())
    }

    pub(crate) fn latches_input(&self, id: ChannelId) -> bool {
        self.stimulus_values.contains_key(&id)
    }

    pub(crate) fn prepare_object(
        &mut self,
        at: Time,
        ids: &[ChannelId],
        command: &Command,
        sequences: &mut Vec<(ChannelId, u64)>,
    ) -> Result<Option<Command>, String> {
        if !ids.iter().any(|id| self.channels.contains_key(id)) {
            return Ok(None);
        }
        let mut targets = command.targets.clone();
        for &channel in ids {
            let Some(&index) = self.channels.get(&channel) else {
                continue;
            };
            let choice = self.choice(at, index)?;
            let old = targets.binary_search_by_key(&channel, |&(id, _)| id);
            match (choice, old) {
                (Some(c), Ok(position)) => targets[position].1 = c.value,
                (Some(c), Err(position)) => targets.insert(position, (channel, c.value)),
                (None, Ok(position)) => {
                    targets.remove(position);
                }
                (None, Err(_)) => (),
            }
            let output = &mut self.outputs[index];
            if let Some(selected) = choice
                && matches!(output.target, Target::Channel(_, Some(_)))
            {
                sequences.push((channel, selected.sequence));
            }
            if output.presented.map(|value| value.sequence) != choice.map(|value| value.sequence) {
                output.presented = choice;
                output
                    .waiting_for_acceptance
                    .retain(|(_, pending)| pending.expires_at.is_none_or(|due| at <= due));
                if let Some(command) = choice {
                    if matches!(output.target, Target::Channel(_, Some(_))) {
                        output.waiting_for_acceptance.push_back((at, command));
                    }
                    self.push(
                        at,
                        EventKind::CommandPresented {
                            path: self.paths[index].name.clone(),
                            channel,
                            command,
                        },
                    );
                }
            }
        }
        Ok(Some(Command { targets }))
    }

    pub(crate) fn object_result(
        &mut self,
        at: Time,
        ids: &[ChannelId],
        plant: &[(crate::TruthId, f64)],
        accepted: &[(ChannelId, u64, Time, f64)],
    ) -> Result<(), String> {
        for &(channel, sequence, effective_at, value) in accepted {
            let index = *self
                .channels
                .get(&channel)
                .ok_or("принятие команды на неизвестном входе")?;
            let out = &mut self.outputs[index];
            let Target::Channel(_, Some(ack)) = out.target else {
                return Err("подтверждение получено от входа без приёмника".into());
            };
            if effective_at > at
                || !value.is_finite()
                || !ids.contains(&channel)
                || !plant
                    .iter()
                    .any(|&(id, actual)| id == ack && actual.to_bits() == value.to_bits())
            {
                return Err("некорректное подтверждение исполнительной модели".into());
            }
            out.ack_seen = Some(value);
            let path = self.paths[index].name.clone();
            let candidate = out
                .waiting_for_acceptance
                .iter()
                .position(|(_, cmd)| cmd.sequence == sequence);
            if let Some(position) = candidate {
                let (presented, command) = out.waiting_for_acceptance[position];
                out.waiting_for_acceptance.drain(..=position);
                if command.value.to_bits() == value.to_bits()
                    && presented <= effective_at
                    && command.expires_at.is_none_or(|due| effective_at <= due)
                {
                    out.applied = Some(command);
                    self.push(
                        at,
                        EventKind::CommandApplied {
                            path,
                            channel,
                            command,
                            effective_at,
                            timing: ApplicationTiming::ModelAcceptance,
                        },
                    );
                    continue;
                }
            }
            self.push(
                at,
                EventKind::ModelAcceptanceUnattributed {
                    path,
                    channel,
                    value,
                },
            );
        }
        for &channel in ids {
            if accepted.iter().any(|item| item.0 == channel) {
                continue;
            }
            let Some(&index) = self.channels.get(&channel) else {
                continue;
            };
            let out = &mut self.outputs[index];
            let Target::Channel(_, Some(ack)) = out.target else {
                continue;
            };
            let Some(value) = plant
                .iter()
                .find(|(key, _)| *key == ack)
                .map(|&(_, value)| value)
            else {
                continue;
            };
            let changed = out
                .ack_seen
                .replace(value)
                .is_none_or(|prior| prior.to_bits() != value.to_bits());
            if !changed {
                continue;
            }
            let path = self.paths[index].name.clone();
            let mut candidates = out
                .waiting_for_acceptance
                .iter()
                .enumerate()
                .filter(|(_, (_, cmd))| cmd.value.to_bits() == value.to_bits())
                .map(|(index, &(_, cmd))| (index, cmd));
            let first = candidates.next();
            let other = candidates.next_back();
            if let Some((position, command)) = other.or(first) {
                out.waiting_for_acceptance.drain(..=position);
                if other.is_none() && command.expires_at.is_none_or(|due| at <= due) {
                    out.applied = Some(command);
                    self.push(
                        at,
                        EventKind::CommandApplied {
                            path,
                            channel,
                            command,
                            effective_at: at,
                            timing: ApplicationTiming::ModelReport,
                        },
                    );
                    continue;
                }
            }
            self.push(
                at,
                EventKind::ModelAcceptanceUnattributed {
                    path,
                    channel,
                    value,
                },
            );
        }
        Ok(())
    }

    pub(crate) fn latch(&mut self, at: Time, runtime: &Runtime) -> Result<(), String> {
        self.latched_start = at;
        self.direct.clear();
        self.latched_forces.clear();
        self.direct_pending.clear();
        self.stimulus_pending.clear();
        for (&channel, (name, command)) in &self.stimulus_values {
            if !runtime.state.claimed.contains(&channel) {
                self.direct.push((channel, command.value));
                if self.stimulus_integrated.get(&channel) != Some(&command.sequence) {
                    self.stimulus_pending
                        .push((channel, name.clone(), *command));
                }
            }
        }
        self.latched_forces.extend(
            runtime
                .state
                .components
                .iter()
                .flat_map(|component| component.forces.iter().copied()),
        );
        for index in 0..self.outputs.len() {
            let out = &self.outputs[index];
            let Target::Channel(channel, None) = out.target else {
                continue;
            };
            if out.kind != TargetKind::Effort {
                continue;
            }
            let choice = self.choice(at, index)?;
            if let Some(command) = choice {
                self.direct.push((channel, command.value));
                if self.outputs[index].applied != Some(command) {
                    self.direct_pending.push((index, command));
                }
            } else {
                self.outputs[index].applied = None;
            }
        }
        Ok(())
    }

    pub(crate) fn forces(&self, target: &mut [f64], dofs: &[usize]) {
        for &(channel, value) in self.direct.iter().chain(&self.latched_forces) {
            target[dofs[channel.0 as usize]] += value;
        }
    }

    pub(crate) fn integrated(&mut self, at: Time) {
        for (channel, path, command) in std::mem::take(&mut self.stimulus_pending) {
            self.stimulus_integrated.insert(channel, command.sequence);
            self.push(
                at,
                EventKind::CommandApplied {
                    path,
                    channel,
                    command,
                    effective_at: self.latched_start,
                    timing: ApplicationTiming::PhysicsInterval,
                },
            );
        }
        for (index, command) in std::mem::take(&mut self.direct_pending) {
            let out = &mut self.outputs[index];
            let Target::Channel(channel, _) = out.target else {
                unreachable!()
            };
            out.applied = Some(command);
            self.push(
                at,
                EventKind::CommandApplied {
                    path: self.paths[index].name.clone(),
                    channel,
                    command,
                    effective_at: self.latched_start,
                    timing: ApplicationTiming::PhysicsInterval,
                },
            );
        }
    }

    pub(crate) fn finish_failure(&mut self, at: Time, component: &str, reason: &str) {
        self.push(
            at,
            EventKind::Failure {
                component: component.into(),
                reason: reason.into(),
            },
        );
    }
}

fn add(at: Time, delay: u64) -> Result<Time, String> {
    at.0.checked_add(delay)
        .map(Time)
        .ok_or_else(|| "момент события вне шкалы времени".into())
}

fn delayed(at: Time, delay: Delay, rng: &mut Rng) -> Result<Time, String> {
    let span = u128::from(delay.jitter_ns) * 2 + 1;
    let low = delay.latency_ns - delay.jitter_ns;
    // Отбрасывание неполного остатка обеспечивает равномерность целых наносекунд.
    let zone = ((1u128 << 64) / span) * span;
    let offset = loop {
        let number = u128::from(rng.next_u64());
        if number < zone {
            break (number % span) as u64;
        }
    };
    add(at, low + offset)
}

fn first_tick(rate: sim_signal::Rate, at: Time, inclusive: bool) -> Result<Time, String> {
    let at = if inclusive { at } else { add(at, 1)? };
    // Бинарный поиск ограничен числом наносекунд: частота не выше 1 ГГц.
    // Clock проверяет переполнение, а Rate::tick может паниковать за шкалой.
    let mut lo = 0;
    let mut hi = 0;
    if at != Time::ZERO {
        hi = 1;
        while tick(rate, hi)?.is_some_and(|due| due < at) {
            lo = hi + 1;
            hi = hi.checked_mul(2).ok_or("не найден такт получателя")?;
        }
    }
    while lo < hi {
        let mid = lo + (hi - lo) / 2;
        if tick(rate, mid)?.is_none_or(|due| due >= at) {
            hi = mid;
        } else {
            lo = mid + 1;
        }
    }
    tick(rate, lo)?.ok_or_else(|| "такт получателя вне шкалы времени".into())
}

fn tick(rate: sim_signal::Rate, index: u64) -> Result<Option<Time>, String> {
    crate::time_at(rate, index)
}

/// Метки времени последнего скалярного отсчёта, в том числе недействительного.
pub fn measurement_time(runtime: &Runtime, signal: SignalId) -> Option<MeasurementTime> {
    runtime
        .state
        .bus
        .sample(signal)
        .map(|(sample, captured_at)| MeasurementTime {
            captured_at,
            delivered_at: sample.at,
        })
}

pub fn run(runtime: &Runtime) -> Option<ControlRun<'_>> {
    runtime.state.control.as_ref()?;
    Some(ControlRun { runtime })
}

pub fn verify_replay(runtime: &Runtime) -> Result<(), String> {
    let run = run(runtime).ok_or("контур не установлен")?;
    let (recording, cursor) = run
        .state()
        .playback
        .as_ref()
        .ok_or("воспроизведение не включено")?;
    if *cursor != recording.exchanges.len()
        || !run.log().matches(&recording.log)
        || run.events().len() != recording.events.len()
        || run
            .events()
            .iter()
            .zip(recording.events.iter())
            .any(|(a, b)| format!("{a:?}") != format!("{b:?}"))
        || run.fingerprint() != recording.fingerprint
    {
        return Err("воспроизведение контура разошлось с записью".into());
    }
    Ok(())
}

pub fn worker_ids(runtime: &Runtime) -> impl Iterator<Item = u32> + '_ {
    runtime
        .processes
        .values()
        .map(sim_control::transport::ProcessTransport::worker_id)
}
