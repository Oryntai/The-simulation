use std::collections::BTreeMap;
use std::rc::Rc;

use sim_boards::{
    BoardInput, BoardOutput, BoardSim, InputEvent, InputOutcome, OutputEvent, OutputPort, Packet,
    PortValue, TransferStatus,
};
use sim_radio::timed::{Outcome, PacketEvent, TimedChannel};
use sim_signal::{Clock, SignalId, Time};

use crate::{ObservationBus, Runtime, checked_due};

use super::bus::{BusModel, Profile};
use super::schema::{BoardWire, DeliverySlot};
use super::{CommandOrigin, Engine, EventKind, FaultAction, Rejection, Schedule, add};

#[derive(Clone)]
pub(super) struct Board {
    pub name: String,
    pub program_hash: u64,
    pub clock: Clock,
    pub sim: Box<dyn BoardSim>,
    pub wires: Vec<Wire>,
    pub buses: Vec<Bus>,
    pub outputs: Vec<usize>,
    pub pending: Vec<Delivery>,
    pub actions: Vec<usize>,
    pub input_at: Time,
    pub end_sample: bool,
    pub boot: Option<Box<dyn BoardSim>>,
    pub epoch: Time,
    pub epoch_cycle: u64,
    pub electrical: Option<usize>,
    pub electrical_restart: Option<Time>,
    pub driven_gpio: BTreeMap<u32, bool>,
}

impl Board {
    fn cycle_at(&self, at: Time) -> Result<u64, String> {
        self.epoch_cycle
            .checked_add(time_cycle(
                Time(at.0 - self.epoch.0),
                self.sim.capabilities().cpu_hz,
            )?)
            .ok_or_else(|| "такт платы вне диапазона".into())
    }
}

#[derive(Clone)]
pub(super) struct ScheduledBoardAction {
    pub id: String,
    pub at: Time,
    pub board: usize,
    pub action: super::BoardAction,
    pub config: Option<Vec<u8>>,
}

#[derive(Clone)]
pub(super) struct Wire {
    pub spec: std::rc::Rc<BoardWire>,
    pub signal: SignalId,
    pub last: Option<BoardInput>,
}

#[derive(Clone)]
pub(super) struct Bus {
    pub name: String,
    pub bus: u32,
    pub chip_select: Option<u8>,
    pub model: BusModel,
    pub signal: SignalId,
    pub signal_name: String,
    pub max_age_ns: u64,
    pub sample: Option<(f64, Time)>,
    pub connected: bool,
}

#[derive(Clone)]
pub(super) struct Route {
    pub name: String,
    pub from: Option<(usize, u32)>,
    pub to: Option<(usize, u32)>,
    pub radio: Option<TimedChannel>,
    pub scheduled_rssi: Option<i8>,
    pub packet_bytes: usize,
    pub slots: Option<Rc<[DeliverySlot]>>,
    pub delay_ns: u64,
    pub capacity: usize,
    pub max_age_ns: u64,
    pub sequence: u64,
    pub pending: BTreeMap<(Time, u64), PacketEvent>,
}

impl Route {
    pub(super) fn is_radio(&self) -> bool {
        self.radio.is_some() || self.scheduled_rssi.is_some()
    }

    fn delivery_at(&self, ready: Time) -> Result<Time, String> {
        let Some(slots) = &self.slots else {
            return add(ready, self.delay_ns);
        };
        let index = slots.partition_point(|slot| slot.accepts_through < ready);
        slots
            .get(index)
            .map(|slot| slot.delivers_at)
            .ok_or_else(|| {
                format!(
                    "календарь канала «{}» не содержит слота для готовности в {} нс",
                    self.name, ready.0
                )
            })
    }
}

#[derive(Clone)]
pub(super) struct Delivery {
    route: usize,
    packet: PacketEvent,
}

impl Engine {
    pub(crate) fn electrical_board_due(&self, at: Time) -> bool {
        self.boards
            .iter()
            .any(|board| board.electrical.is_some() && checked_due(&board.clock) == Ok(Some(at)))
    }

    pub(crate) fn electrical_advance(
        &mut self,
        bus: &ObservationBus,
        at: Time,
        circuit: &mut crate::electrical::Electrical,
    ) -> Result<(), String> {
        for index in 0..self.boards.len() {
            let Some(slot) = self.boards[index].electrical else {
                continue;
            };
            let target = self.boards[index].cycle_at(at)?;
            self.advance_board(at, index, target)?;
            if let Some(boot) = self.boards[index].electrical_restart.take() {
                circuit.boards[slot].restart(boot);
            }
            if at == Time::ZERO && circuit.boards[slot].initial_reset {
                self.electrical_reset(at, index, true)?;
            }
            let (voltage_v, current_a, edge) = circuit.read_board(at, slot)?;
            self.push(
                at,
                EventKind::ElectricalRail {
                    board: self.boards[index].name.clone(),
                    voltage_v,
                    current_a,
                    phase: circuit.boards[slot].phase,
                },
            );
            if let Some(asserted) = edge {
                self.electrical_reset(at, index, asserted)?;
                if !asserted {
                    self.board_inputs(bus, at, index)?;
                }
            }
        }
        Ok(())
    }

    fn electrical_reset(&mut self, at: Time, index: usize, asserted: bool) -> Result<(), String> {
        let board = &mut self.boards[index];
        // После асинхронного фронта первое целое деление такта отсчитывается заново.
        board.epoch = at;
        board.epoch_cycle = board.sim.cycles();
        for wire in &mut board.wires {
            wire.last = None;
        }
        self.apply_board(at, index, BoardInput::Brownout { asserted })?;
        self.board_events(at, index)?;
        self.push(
            at,
            EventKind::ElectricalReset {
                board: self.boards[index].name.clone(),
                asserted,
            },
        );
        if asserted {
            for output in self.boards[index].outputs.clone() {
                self.invalidate(at, output, Rejection::Disconnected);
            }
        }
        Ok(())
    }

    pub(crate) fn electrical_latch(
        &mut self,
        at: Time,
        circuit: &mut crate::electrical::Electrical,
    ) -> Result<(), String> {
        circuit.latch_sources(at)?;
        for board in &mut self.boards {
            let Some(slot) = board.electrical else {
                continue;
            };
            if let Some(boot) = board.electrical_restart.take() {
                circuit.boards[slot].restart(boot);
            }
            let changed = circuit.latch(at, slot, |pin| board.sim.gpio_mode(pin))?;
            for (pin, state) in changed {
                Rc::make_mut(&mut self.events).push(super::Event {
                    at,
                    kind: EventKind::ElectricalPin {
                        board: board.name.clone(),
                        pin,
                        state,
                    },
                });
            }
        }
        circuit.schedule(at, self.identity.phase_horizon)
    }

    pub(super) fn request_board_action(&mut self, at: Time, id: usize) {
        let action = &self.board_actions[id];
        let board = &mut self.boards[action.board];
        board.actions.push(id);
        let event = EventKind::BoardActionRequested {
            id: action.id.clone(),
            board: board.name.clone(),
            action: action.action.clone(),
        };
        self.push(at, event);
    }

    fn apply_board_actions(&mut self, at: Time, index: usize) -> Result<bool, String> {
        let mut replaced = false;
        for id in std::mem::take(&mut self.boards[index].actions) {
            let item = self.board_actions[id].clone();
            let board = self.boards[index].name.clone();
            let cycle = self.boards[index].sim.cycles();
            if !self.boards[index].sim.powered() {
                self.rejection(at, item.id, None, Rejection::Disconnected);
                continue;
            }
            match item.action {
                super::BoardAction::Gpio { pin, high } => {
                    self.apply_board(at, index, BoardInput::Gpio { pin, high })?;
                    self.boards[index].driven_gpio.insert(pin, high);
                }
                super::BoardAction::Reset { .. } | super::BoardAction::Restart { .. } => {
                    let device = &mut self.boards[index];
                    if matches!(item.action, super::BoardAction::Restart { .. }) {
                        device.sim = device
                            .boot
                            .as_ref()
                            .expect("начальный образ сохранён")
                            .clone();
                        device.epoch = at;
                        device.epoch_cycle = 0;
                        for wire in &mut device.wires {
                            wire.last = None;
                        }
                        replaced = true;
                    }
                    if let Some(config) = item.config {
                        device
                            .sim
                            .write_config(&config)
                            .map_err(|e| format!("плата «{board}»: {e}"))?;
                    }
                    device.sim.reset();
                    if device.sim.reset_cause() != Some(sim_boards::ResetCause::Host) {
                        return Err(format!("плата «{board}» не подтвердила сброс"));
                    }
                    self.board_events(at, index)?;
                    if replaced {
                        for (pin, high) in self.boards[index].driven_gpio.clone() {
                            self.apply_board(at, index, BoardInput::Gpio { pin, high })?;
                        }
                    }
                }
            }
            self.push(
                at,
                EventKind::BoardActionApplied {
                    id: item.id,
                    board,
                    cycle,
                    action: item.action,
                },
            );
        }
        Ok(replaced)
    }

    pub(super) fn tick_board(
        &mut self,
        bus: &ObservationBus,
        at: Time,
        index: usize,
    ) -> Result<(), String> {
        let name = self.boards[index].name.clone();
        let end_sample = self.boards[index].end_sample;
        let observed_at = if end_sample {
            at
        } else {
            self.boards[index].input_at
        };
        let target_cycles = self.boards[index].cycle_at(at)?;
        self.advance_board(at, index, target_cycles)?;
        let fault = self.use_fault(at, &format!("board:{name}"));
        if self.boards[index].electrical.is_none()
            && fault != Some(FaultAction::PowerOff)
            && !self.boards[index].sim.powered()
        {
            self.apply_board(at, index, BoardInput::Power { on: true })?;
            for wire in &mut self.boards[index].wires {
                wire.last = None;
            }
        }
        if fault == Some(FaultAction::PowerOff) && self.boards[index].sim.powered() {
            self.apply_board(at, index, BoardInput::Power { on: false })?;
            self.board_events(at, index)?;
        }
        if self.boards[index].sim.powered() {
            self.apply_board(
                at,
                index,
                BoardInput::Execution {
                    enabled: fault != Some(FaultAction::Pause),
                },
            )?;
            self.board_inputs(bus, at, index)?;
        }
        if self.apply_board_actions(at, index)? {
            if fault == Some(FaultAction::Pause) {
                self.apply_board(at, index, BoardInput::Execution { enabled: false })?;
            }
            self.board_inputs(bus, at, index)?;
        }
        self.boards[index].clock.advance();
        let next = checked_due(&self.boards[index].clock)?;
        if end_sample {
            // Выход вычисляется по выборке конца того же численного интервала.
            if let Some(next) = next.filter(|next| *next <= self.horizon()) {
                let target = self.boards[index].cycle_at(next)?;
                self.advance_board(at, index, target)?;
            }
        } else {
            self.receive_packets(at, index)?;
        }
        for output_index in self.boards[index].outputs.clone() {
            let driver = self.outputs[output_index].driver.as_ref().expect("драйвер");
            let value = if fault.is_none() {
                driver_value(self.boards[index].sim.as_ref(), driver)?
            } else {
                None
            };
            self.deliver_output(
                at,
                output_index,
                observed_at,
                at,
                value,
                CommandOrigin::Board,
            )?;
        }
        if end_sample {
            self.receive_packets(at, index)?;
        }
        self.boards[index].input_at = at;
        if let Some(due) = next {
            self.schedule(due, Schedule::Board, index);
        }
        Ok(())
    }

    fn advance_board(&mut self, at: Time, index: usize, target: u64) -> Result<(), String> {
        // Входы текущего момента подаются после исполнения предыдущего интервала.
        self.board_events(at, index)?;
        while self.boards[index].sim.cycles() < target {
            let previous = self.boards[index].sim.cycles();
            let result = self.boards[index].sim.run_cycles(target - previous);
            self.board_events(at, index)?;
            let progress =
                result.map_err(|e| format!("плата «{}»: {e}", self.boards[index].name))?;
            if progress.cycle <= previous || progress.cycle > target {
                return Err("плата не продвинулась до допустимого такта".into());
            }
        }
        Ok(())
    }

    fn apply_board(
        &mut self,
        at: Time,
        index: usize,
        input: BoardInput,
    ) -> Result<InputOutcome, String> {
        let board = &mut self.boards[index];
        let cycle = board.sim.cycles();
        let outcome = board
            .sim
            .apply(InputEvent {
                cycle,
                input: input.clone(),
            })
            .map_err(|e| format!("плата «{}»: {e}", board.name))?;
        if outcome != InputOutcome::Unchanged {
            let name = board.name.clone();
            self.push(
                at,
                EventKind::BoardInput {
                    board: name,
                    cycle,
                    input,
                    outcome,
                },
            );
        }
        Ok(outcome)
    }

    fn board_inputs(&mut self, bus: &ObservationBus, at: Time, index: usize) -> Result<(), String> {
        let name = self.boards[index].name.clone();
        for id in 0..self.boards[index].wires.len() {
            let wire = self.boards[index].wires[id].clone();
            let action = self.use_fault(at, &format!("wire:{}", wire.spec.name));
            let sample = self.read_input(
                bus,
                at,
                wire.signal,
                &wire.spec.signal,
                wire.spec.max_age_ns,
                &name,
            );
            let value = match action {
                Some(FaultAction::Drop) => wire.spec.fallback,
                Some(FaultAction::Stuck(value)) => Some(value),
                _ => sample
                    .filter(|s| s.valid)
                    .map(|s| s.value.mul_add(wire.spec.scale, wire.spec.offset))
                    .or(wire.spec.fallback),
            };
            if value.is_some_and(|value| !value.is_finite()) {
                return Err("неконечное преобразованное значение провода".into());
            }
            let port = wire.spec.port;
            let input = match (wire.spec.kind.as_str(), value) {
                ("adc", Some(volts)) => {
                    let max = sim_boards::board_spec(self.boards[index].sim.board()).adc_max_v;
                    if !(0.0..=max).contains(&volts) {
                        return Err(format!(
                            "вход АЦП «{}»: {volts} В вне диапазона 0…{max}",
                            wire.spec.name
                        ));
                    }
                    BoardInput::Adc {
                        channel: port,
                        volts,
                    }
                }
                ("adc", None) => BoardInput::DisconnectAdc { channel: port },
                ("gpio", Some(0.0)) => BoardInput::Gpio {
                    pin: port,
                    high: false,
                },
                ("gpio", Some(1.0)) => BoardInput::Gpio {
                    pin: port,
                    high: true,
                },
                _ => return Err("логический вход принимает только 0 или 1".into()),
            };
            if wire.last.as_ref() != Some(&input) {
                self.apply_board(at, index, input.clone())?;
                self.boards[index].wires[id].last = Some(input);
            }
        }
        for id in 0..self.boards[index].buses.len() {
            let registered = &self.boards[index].buses[id];
            let (wire, signal, signal_name, age) = (
                registered.name.clone(),
                registered.signal,
                registered.signal_name.clone(),
                registered.max_age_ns,
            );
            let connected = self.use_fault(at, &format!("bus:{wire}")).is_none();
            let sample = self.read_input(bus, at, signal, &signal_name, age, &name);
            let registered = &mut self.boards[index].buses[id];
            registered.connected = connected;
            registered.sample = if connected {
                sample.filter(|s| s.valid).map(|s| (s.value, s.at))
            } else {
                None
            };
        }
        Ok(())
    }

    fn board_events(&mut self, at: Time, index: usize) -> Result<(), String> {
        let hz = self.boards[index].sim.capabilities().cpu_hz;
        let name = self.boards[index].name.clone();
        for event in self.boards[index].sim.take_events() {
            let cycle = event
                .cycle
                .checked_sub(self.boards[index].epoch_cycle)
                .ok_or("событие платы относится к прошлой эпохе")?;
            let produced_at = add(self.boards[index].epoch, cycle_time(cycle, hz)?.0)?;
            if self.boards[index].electrical.is_some()
                && matches!(
                    event.output,
                    BoardOutput::Reset(
                        sim_boards::ResetCause::Watchdog
                            | sim_boards::ResetCause::Software
                            | sim_boards::ResetCause::Host
                    )
                )
            {
                self.boards[index].electrical_restart = Some(produced_at);
            }
            self.push(
                at,
                EventKind::BoardOutput {
                    board: name.clone(),
                    event: event.clone(),
                    produced_at,
                },
            );
            self.handle_board_event(at, produced_at, index, event)?;
        }
        Ok(())
    }

    fn handle_board_event(
        &mut self,
        at: Time,
        produced: Time,
        index: usize,
        event: OutputEvent,
    ) -> Result<(), String> {
        match event.output {
            BoardOutput::I2c(req) => {
                let connection = self.boards[index].buses.iter_mut().find(|bus| bus.bus == req.bus
                    && matches!(bus.model.profile, Profile::I2c { address, .. } if address == req.address));
                let (status, bytes) =
                    connection.map_or((TransferStatus::Nack, Vec::new()), |bus| {
                        let sample = bus
                            .sample
                            .filter(|&(_, t)| t <= produced && produced.0 - t.0 <= bus.max_age_ns)
                            .map(|(v, _)| v);
                        bus.model.i2c(&req, sample)
                    });
                self.apply_board(
                    at,
                    index,
                    BoardInput::I2cReply {
                        bus: req.bus,
                        id: req.id,
                        status,
                        bytes,
                    },
                )?;
            }
            BoardOutput::Spi(req) => {
                let connection = self.boards[index]
                    .buses
                    .iter_mut()
                    .find(|bus| bus.bus == req.bus && bus.chip_select == Some(req.chip_select));
                let (status, bytes) =
                    connection.map_or((TransferStatus::NoResponse, Vec::new()), |bus| {
                        let sample = bus
                            .sample
                            .filter(|&(_, t)| t <= produced && produced.0 - t.0 <= bus.max_age_ns)
                            .map(|(v, _)| v);
                        bus.model.spi(&req, sample)
                    });
                self.apply_board(
                    at,
                    index,
                    BoardInput::SpiReply {
                        bus: req.bus,
                        id: req.id,
                        status,
                        bytes,
                    },
                )?;
            }
            BoardOutput::Uart { port, byte } => {
                self.route_output(at, index, port, false, &[byte])?
            }
            BoardOutput::Radio { port, packet } => {
                self.route_output(at, index, port, true, &packet.bytes)?
            }
            _ => (),
        }
        Ok(())
    }

    fn route_output(
        &mut self,
        at: Time,
        board: usize,
        port: u32,
        radio: bool,
        bytes: &[u8],
    ) -> Result<(), String> {
        for id in 0..self.routes.len() {
            if self.routes[id].from == Some((board, port)) && self.routes[id].is_radio() == radio {
                self.send_packet(at, id, bytes.to_vec())?;
            }
        }
        Ok(())
    }

    pub(super) fn send_packet(
        &mut self,
        at: Time,
        index: usize,
        bytes: Vec<u8>,
    ) -> Result<(), String> {
        let path = self.routes[index].name.clone();
        let fault = self.use_fault(at, &format!("channel:{path}"));
        let disconnected = fault == Some(FaultAction::Drop);
        let hold_ns = if let Some(FaultAction::Delay(ns)) = fault {
            ns
        } else {
            0
        };
        let route = &mut self.routes[index];
        if route.is_radio() && bytes.len() > route.packet_bytes {
            return Err(format!("пакет превышает размер канала «{path}»"));
        }
        let event = if let Some(channel) = &mut route.radio {
            channel.send_after(at, bytes, hold_ns)?
        } else {
            let id = route.sequence;
            route.sequence = id.checked_add(1).ok_or("исчерпаны номера UART")?;
            let queued = route.pending.len();
            let (due, outcome) = if disconnected {
                (at, Outcome::Disconnected)
            } else if queued >= route.capacity {
                (at, Outcome::Overflow)
            } else {
                (route.delivery_at(add(at, hold_ns)?)?, Outcome::Queued)
            };
            let event = PacketEvent {
                id,
                sent_at: at,
                at: due,
                attempt: 0,
                bytes,
                outcome,
            };
            if event.outcome == Outcome::Queued {
                route.pending.insert((event.at, event.id), event.clone());
            }
            event
        };
        if event.outcome == Outcome::Queued {
            self.schedule(event.at, Schedule::Packet, index);
            self.push(
                at,
                EventKind::PacketSent {
                    route: path,
                    id: event.id,
                    due: event.at,
                    bytes: event.bytes,
                },
            );
        } else {
            self.push(at, EventKind::Packet { route: path, event });
        }
        Ok(())
    }

    pub(super) fn route_connection(
        &mut self,
        at: Time,
        index: usize,
        active: bool,
    ) -> Result<(), String> {
        let target = format!("channel:{}", self.routes[index].name);
        self.use_fault(at, &target);
        let route = &mut self.routes[index];
        let name = route.name.clone();
        let packets = if let Some(channel) = &mut route.radio {
            channel.set_connected(at, !active)?
        } else if active {
            std::mem::take(&mut route.pending)
                .into_values()
                .map(|mut p| {
                    p.at = at;
                    p.outcome = Outcome::Disconnected;
                    p
                })
                .collect()
        } else {
            Vec::new()
        };
        for packet in packets {
            self.push(
                at,
                EventKind::Packet {
                    route: name.clone(),
                    event: packet,
                },
            );
        }
        for board in 0..self.boards.len() {
            let pending = std::mem::take(&mut self.boards[board].pending);
            for item in pending {
                if active && item.route == index {
                    self.rejection(
                        at,
                        name.clone(),
                        Some(item.packet.id),
                        Rejection::Disconnected,
                    );
                } else {
                    self.boards[board].pending.push(item);
                }
            }
        }
        Ok(())
    }

    pub(super) fn advance_route(
        &mut self,
        runtime: &Runtime,
        at: Time,
        index: usize,
    ) -> Result<(), String> {
        let route = &mut self.routes[index];
        let name = route.name.clone();
        let events = if let Some(channel) = &mut route.radio {
            if let Some(path) = runtime.state.work.radio_paths.get(&name) {
                channel.advance_with_link(at, |_, _, link| {
                    use sim_process::Analysis;
                    // Среда удерживается от последней зафиксированной границы физики.
                    let env = match runtime.state.work.radio.get(&name) {
                        Some(Analysis::Analyzed { value, .. }) => value,
                        Some(Analysis::NotAnalyzed { reason }) => {
                            return Err(format!("радиоканал «{name}»: {reason}"));
                        }
                        None => return Err(format!("нет замера среды радиоканала «{name}»")),
                    };
                    link.through_layer(
                        env.medium,
                        env.depth_m,
                        path.frequency_hz.nominal,
                        path.sensitivity_dbm.nominal,
                    )
                })?
            } else {
                channel.advance(at)?
            }
        } else {
            let mut events = Vec::new();
            while let Some(entry) = route.pending.first_entry() {
                if entry.key().0 > at {
                    break;
                }
                let mut packet = entry.remove();
                packet.outcome = Outcome::Delivered {
                    rssi_dbm: route.scheduled_rssi.unwrap_or(0),
                };
                packet.attempt = 1;
                events.push(packet);
            }
            events
        };
        let to = route.to;
        let next = route.radio.as_ref().and_then(TimedChannel::next_event);
        if let Some(due) = next
            && !self
                .queue
                .iter()
                .any(|entry| entry.0 == (due, Schedule::Packet, index))
        {
            self.schedule(due, Schedule::Packet, index);
        }
        for packet in events {
            let delivered = matches!(packet.outcome, Outcome::Delivered { .. });
            self.push(
                at,
                EventKind::Packet {
                    route: name.clone(),
                    event: packet.clone(),
                },
            );
            if !delivered {
                continue;
            }
            if let Some((board, _)) = to {
                let waiting = self.boards[board]
                    .pending
                    .iter()
                    .filter(|p| p.route == index)
                    .count();
                if waiting >= self.routes[index].capacity {
                    self.rejection(at, name.clone(), Some(packet.id), Rejection::Overflow);
                } else {
                    self.boards[board].pending.push(Delivery {
                        route: index,
                        packet,
                    });
                }
            } else if at <= add(packet.sent_at, self.routes[index].max_age_ns)? {
                self.push(
                    at,
                    EventKind::PacketDelivered {
                        route: name.clone(),
                        id: packet.id,
                        sent_at: packet.sent_at,
                        bytes: packet.bytes,
                    },
                );
            } else {
                self.rejection(at, name.clone(), Some(packet.id), Rejection::Expired);
            }
        }
        Ok(())
    }

    fn receive_packets(&mut self, at: Time, board: usize) -> Result<(), String> {
        for delivery in std::mem::take(&mut self.boards[board].pending) {
            let route = &self.routes[delivery.route];
            let path = route.name.clone();
            let packet = delivery.packet;
            let (_, port) = route.to.expect("порт приёмника");
            if at > add(packet.sent_at, route.max_age_ns)? {
                self.rejection(at, path, Some(packet.id), Rejection::Expired);
                continue;
            }
            if !self.boards[board].sim.powered() {
                self.rejection(at, path, Some(packet.id), Rejection::Disconnected);
                continue;
            }
            let input = if route.is_radio() {
                let Outcome::Delivered { rssi_dbm } = packet.outcome else {
                    unreachable!()
                };
                BoardInput::Radio {
                    port,
                    packet: Packet {
                        bytes: packet.bytes.clone(),
                        rssi_dbm: Some(rssi_dbm),
                    },
                }
            } else {
                BoardInput::Uart {
                    port,
                    bytes: packet.bytes.clone(),
                }
            };
            let outcome = self.apply_board(at, board, input)?;
            let n = match outcome {
                InputOutcome::Uart { accepted, .. } => accepted,
                InputOutcome::PacketDropped => 0,
                _ => packet.bytes.len(),
            };
            if n != 0 {
                self.push(
                    at,
                    EventKind::PacketDelivered {
                        route: path.clone(),
                        id: packet.id,
                        sent_at: packet.sent_at,
                        bytes: packet.bytes[..n].to_vec(),
                    },
                );
            }
            if n < packet.bytes.len() {
                self.rejection(at, path, Some(packet.id), Rejection::Overflow);
            }
        }
        Ok(())
    }
}

// 10⁹ нс/с. Коэффициенты считаются на целых, без накопления ошибки.
const NS_PER_S: u128 = 1_000_000_000;
fn time_cycle(time: Time, hz: u64) -> Result<u64, String> {
    u64::try_from(u128::from(time.0) * u128::from(hz) / NS_PER_S)
        .map_err(|_| "такт платы вне диапазона".into())
}
fn cycle_time(cycle: u64, hz: u64) -> Result<Time, String> {
    u64::try_from(u128::from(cycle) * NS_PER_S / u128::from(hz))
        .map(Time)
        .map_err(|_| "время платы вне диапазона".into())
}

fn driver_value(sim: &dyn BoardSim, driver: &super::DriverSpec) -> Result<Option<f64>, String> {
    let logic = |pin| match sim.sample(OutputPort::Gpio(pin)).value {
        Some(PortValue::Logic(high)) => Some(high),
        _ => None,
    };
    if let Some(pin) = driver.enable_pin
        && logic(pin) != driver.active_high
    {
        return Ok(None);
    }
    let raw = if driver.quantity == "logic" {
        logic(driver.pin).map(|high| f64::from(u8::from(high)))
    } else {
        let Some(PortValue::Pwm(pwm)) = sim.sample(OutputPort::Pwm(driver.pin)).value else {
            return Ok(None);
        };
        if ![pwm.period_s, pwm.duty, pwm.high_s]
            .into_iter()
            .all(f64::is_finite)
        {
            return Err("плата выдала неконечный ШИМ".into());
        }
        if let Some((min, max)) = driver.period_min_s.zip(driver.period_max_s)
            && !(min..=max).contains(&pwm.period_s)
        {
            return Ok(None);
        }
        Some(match driver.quantity.as_str() {
            "duty" => pwm.duty,
            "high_s" => pwm.high_s,
            "period_s" => pwm.period_s,
            _ => unreachable!("проверено схемой"),
        })
    };
    let sign = if let Some(pin) = driver.direction_pin {
        match logic(pin) {
            Some(true) => 1.0,
            Some(false) => -1.0,
            None => return Ok(None),
        }
    } else {
        1.0
    };
    Ok(raw.map(|raw| (sign * raw).mul_add(driver.scale, driver.offset)))
}
