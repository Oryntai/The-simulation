use std::{
    collections::{BTreeMap, BTreeSet},
    rc::Rc,
};

use sim_boards::GpioMode;
use sim_process::{
    Analysis, SourcedRange,
    schematic::{
        PinPhase, PinProfile, PinState, Pull, ResetProfile, Scope,
        emi_reset::{
            EmiEvidence, EmiPacketProbe, EmiPacketWitness, EmiResetObservation, EmiTrial,
            assess_emi_reset,
        },
    },
};
use sim_signal::{ChannelId, Command, Phase, SignalId, Time, fnv1a};

use crate::{LogEntry, LogKind, Output, Runtime, Status, TruthId, control};

use super::rail::{Rail, after, duration, exact};

/// Граница с единственным решателем цепи. Клон обязан сохранять реактивное
/// состояние, нелинейные элементы и команды, отложенные на следующий интервал.
pub trait CircuitModel: CloneCircuit + 'static {
    /// Проверить, что ветвь тока направлена от plus к minus, а контакты
    /// принадлежат той же плате и раскрыты с переданными профилями.
    fn validate_board(&self, connection: &BoardConnection) -> Result<(), String>;
    /// Закрыть интервал удержанными входами. На нуле доступны начальные условия.
    fn advance_to(&mut self, at: Time) -> Result<(), String>;
    fn voltage(&self, plus: &str, minus: &str) -> Result<f64, String>;
    /// Принятое значение вдоль ориентации ветви, с возможным обратным знаком.
    fn current(&self, branch: &str) -> Result<f64, String>;
    /// Скачок на правой стороне границы. Уже принятая точка остаётся неизменной.
    fn set_source_step(&mut self, name: &str, value: f64) -> Result<(), String>;
    /// Сменить проводящие пути вывода для следующего интервала. Силовой
    /// ключ получает напряжение с физического узла в решателе.
    fn set_pin(&mut self, contact: &str, state: PinState) -> Result<(), String>;
    fn validate_current_source(
        &self,
        _plus: &str,
        _minus: &str,
        _branch: &str,
    ) -> Result<(), String> {
        Err("решатель не подтверждает ориентацию внешней токовой ветви".into())
    }
}

#[doc(hidden)]
pub trait CloneCircuit {
    fn clone_circuit(&self) -> Box<dyn CircuitModel>;
}

impl<T: CircuitModel + Clone> CloneCircuit for T {
    fn clone_circuit(&self) -> Box<dyn CircuitModel> {
        Box::new(self.clone())
    }
}

impl Clone for Box<dyn CircuitModel> {
    fn clone(&self) -> Self {
        self.as_ref().clone_circuit()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PowerPhase {
    Unpowered,
    Reset,
    Boot,
    Working,
}

#[derive(Clone, Debug)]
pub struct BoardCurrents {
    pub unpowered_a: SourcedRange,
    pub reset_a: SourcedRange,
    pub boot_a: SourcedRange,
    pub working_a: SourcedRange,
}

#[derive(Clone, Debug)]
pub struct GpioConnection {
    pub pin: u32,
    /// Эквивалент контакта в MNA, включая защитные ветви и подтяжки.
    pub contact: String,
    pub profile: PinProfile,
}

#[derive(Clone, Debug)]
pub struct BoardConnection {
    pub board: String,
    pub plus: String,
    pub minus: String,
    /// Ориентированный EXT-источник тока между plus и minus.
    pub current_source: String,
    pub currents: BoardCurrents,
    pub reset: ResetProfile,
    pub initially_in_reset: bool,
    /// Граница работы загрузчика того же образа, в том числе платы без GPIO.
    pub boot_duration_s: sim_process::schematic::Parameter,
    pub pins: Vec<GpioConnection>,
}

#[derive(Clone, Debug)]
pub struct SourceStep {
    pub at: Time,
    pub source: String,
    pub value: f64,
}

#[derive(Clone, Debug)]
pub struct LoadConnection {
    pub plus: String,
    pub minus: String,
    pub current_source: String,
    pub voltage_input: ChannelId,
    pub next_current: TruthId,
}

pub struct ElectricalSetup {
    pub max_step: Time,
    pub scope: Scope,
    pub boards: Vec<BoardConnection>,
    pub sources: Vec<SourceStep>,
    /// Дополнительные потактовые величины для оценщика и двоичного журнала.
    pub probes: BTreeMap<String, ElectricalProbe>,
}

#[derive(Clone, Debug)]
pub enum ElectricalProbe {
    Voltage { plus: String, minus: String },
    Current { branch: String },
}

impl ElectricalProbe {
    pub(crate) fn unit(&self) -> &'static str {
        match self {
            Self::Voltage { .. } => "V",
            Self::Current { .. } => "A",
        }
    }
}

#[derive(Clone)]
struct Pin {
    connection: GpioConnection,
    boot: Vec<(u64, PinState)>,
    reset: PinState,
    unpowered: PinState,
}

#[derive(Clone)]
struct EmiPortState {
    trial: Rc<EmiTrial>,
    coupled: bool,
    voltage: TruthId,
    detector: Rail,
    caused_reset: Option<Time>,
}

#[derive(Clone)]
pub(crate) struct BoardPower {
    pub name: String,
    plus: String,
    minus: String,
    branch: String,
    currents: [f64; 4],
    pub initial_reset: bool,
    rail: Rail,
    emi: Option<EmiPortState>,
    reset: bool,
    boot: Time,
    boot_ns: u64,
    pins: Vec<Pin>,
    pub phase: PowerPhase,
    last_current: Option<f64>,
    states: BTreeMap<u32, PinState>,
}

impl BoardPower {
    fn new(input: BoardConnection, scope: Scope) -> Result<Self, String> {
        fn state(profile: &PinProfile, phase: PinPhase, scope: Scope) -> Result<PinState, String> {
            match profile.state(&phase, scope).map_err(|e| e.to_string())? {
                Analysis::Analyzed { value, .. } => Ok(value),
                Analysis::NotAnalyzed { reason } => Err(reason),
            }
        }
        let boot_ns = duration(&input.boot_duration_s, scope)?;
        let mut pins = Vec::new();
        let mut ids = BTreeSet::new();
        for connection in input.pins {
            if !ids.insert(connection.pin) || !label(&connection.contact) {
                return Err(format!(
                    "повтор или пустая привязка GPIO {}/{}",
                    input.board, connection.pin
                ));
            }
            let profile = &connection.profile;
            let reset = state(profile, PinPhase::Reset, scope)?;
            let unpowered = state(profile, PinPhase::Unpowered, scope)?;
            let mut boot = Vec::new();
            let mut elapsed = Time::ZERO;
            for stage in &profile.boot {
                let before = elapsed;
                elapsed = after(elapsed, duration(&stage.duration_s, scope)?)?;
                boot.push((
                    elapsed.0,
                    state(profile, PinPhase::Boot { elapsed: before }, scope)?,
                ));
            }
            if elapsed.0 != boot_ns {
                return Err(format!(
                    "профиль {}/{} не покрывает длительность загрузки",
                    input.board, connection.pin
                ));
            }
            pins.push(Pin {
                connection,
                boot,
                reset,
                unpowered,
            });
        }
        pins.sort_by_key(|pin| pin.connection.pin);
        Ok(Self {
            name: input.board,
            plus: input.plus,
            minus: input.minus,
            branch: input.current_source,
            currents: [
                exact(&input.currents.unpowered_a, "ток без питания", scope)?,
                exact(&input.currents.reset_a, "ток при сбросе", scope)?,
                exact(&input.currents.boot_a, "ток загрузки", scope)?,
                exact(&input.currents.working_a, "ток платы", scope)?,
            ],
            initial_reset: input.initially_in_reset,
            rail: Rail::new(&input.reset, input.initially_in_reset, scope)?,
            emi: None,
            reset: input.initially_in_reset,
            boot: Time::ZERO,
            boot_ns,
            pins,
            phase: PowerPhase::Reset,
            last_current: None,
            states: BTreeMap::new(),
        })
    }

    pub fn contacts(&self) -> impl Iterator<Item = u32> + '_ {
        self.pins.iter().map(|pin| pin.connection.pin)
    }

    fn observe(&mut self, at: Time, voltage: f64) -> Result<Option<bool>, String> {
        self.rail.observe(at, voltage, true)?;
        let was_reset = self.reset;
        self.reset = self.rail.reset();
        if let Some(emi) = &mut self.emi {
            // Контроль повторяет сетку детектора. Дополнительная граница не
            // должна менять выборки шины только в опыте с воздействием.
            let edge = emi
                .detector
                .observe(at, emi.trial.voltage_at(at, true), false)?;
            if emi.coupled {
                if !was_reset && !self.reset && edge == Some(true) {
                    emi.caused_reset.get_or_insert(at);
                }
                self.reset |= emi.detector.reset();
            }
        }
        if was_reset && !self.reset {
            self.boot = at;
        }
        self.phase = if self.reset {
            // Нулевая или обратная разность клемм не питает цифровую логику.
            if voltage <= 0.0 {
                PowerPhase::Unpowered
            } else {
                PowerPhase::Reset
            }
        } else if at.0 - self.boot.0 < self.boot_ns {
            PowerPhase::Boot
        } else {
            PowerPhase::Working
        };
        Ok((was_reset != self.reset).then_some(self.reset))
    }

    pub fn restart(&mut self, at: Time) {
        if !self.reset {
            self.boot = self.boot.max(at);
            if self.boot_ns > 0 {
                self.phase = PowerPhase::Boot;
            }
        }
    }

    fn next(&self, at: Time) -> Result<Option<Time>, String> {
        let mut next = self.rail.deadline();
        if let Some(emi) = &self.emi {
            next = next
                .into_iter()
                .chain(emi.detector.deadline())
                .chain(emi.trial.next_boundary(at))
                .min();
        }
        if self.phase == PowerPhase::Boot {
            let boot = self.boot;
            let elapsed = at.0 - boot.0;
            let boundary = self
                .pins
                .iter()
                .flat_map(|pin| pin.boot.iter().map(|&(end, _)| end))
                .filter(|end| *end > elapsed)
                .chain([self.boot_ns])
                .min()
                .expect("граница загрузки");
            next = next.into_iter().chain([after(boot, boundary)?]).min();
        }
        Ok(next)
    }

    fn selected_current(&self) -> f64 {
        self.currents[match self.phase {
            PowerPhase::Unpowered => 0,
            PowerPhase::Reset => 1,
            PowerPhase::Boot => 2,
            PowerPhase::Working => 3,
        }]
    }
}

#[derive(Clone)]
pub(crate) struct Electrical {
    pub boards: Vec<BoardPower>,
    circuit: Box<dyn CircuitModel>,
    steps: Rc<[SourceStep]>,
    cursor: usize,
    last: Option<Time>,
    step: Time,
    scope: Scope,
    pub identity: u64,
    pub due: Option<Time>,
    pub probes: Vec<(crate::TruthId, ElectricalProbe)>,
    loads: Vec<LoadConnection>,
}

impl Electrical {
    pub fn new(
        setup: ElectricalSetup,
        identity: &[u8],
        circuit: impl CircuitModel + Clone,
    ) -> Result<Self, String> {
        let ElectricalSetup {
            max_step,
            scope,
            mut boards,
            mut sources,
            probes,
        } = setup;
        if max_step == Time::ZERO
            || max_step.0 > (1u64 << (f64::MANTISSA_DIGITS - 1))
            || identity.is_empty()
        {
            return Err("электрическая установка требует шаг и идентичность схемы".into());
        }
        boards.sort_by(|a, b| a.board.cmp(&b.board));
        sources.sort_by(|a, b| (a.at, &a.source).cmp(&(b.at, &b.source)));
        let mut branches = BTreeSet::new();
        let mut contacts = BTreeSet::new();
        let mut checked = circuit.clone();
        for board in &boards {
            if ![
                &board.board,
                &board.plus,
                &board.minus,
                &board.current_source,
            ]
            .into_iter()
            .all(|s| label(s))
                || board.plus == board.minus
                || !branches.insert(board.current_source.as_str())
                || board
                    .pins
                    .iter()
                    .any(|pin| !contacts.insert(pin.contact.as_str()))
            {
                return Err(format!(
                    "конфликт электрической привязки платы «{}»",
                    board.board
                ));
            }
            circuit.validate_board(board)?;
            finite(circuit.voltage(&board.plus, &board.minus)?)?;
            let current = finite(circuit.current(&board.current_source)?)?;
            checked.set_source_step(&board.current_source, current)?;
        }
        if boards.windows(2).any(|pair| pair[0].board == pair[1].board)
            || sources
                .windows(2)
                .any(|pair| pair[0].at == pair[1].at && pair[0].source == pair[1].source)
            || sources.iter().any(|step| {
                !label(&step.source)
                    || branches.contains(step.source.as_str())
                    || !step.value.is_finite()
            })
        {
            return Err("повтор владельца источника, платы или неконечная команда цепи".into());
        }
        for step in &sources {
            checked.set_source_step(&step.source, step.value)?;
        }
        for (name, probe) in &probes {
            if !label(name) {
                return Err("пустое или некорректное имя электрического наблюдения".into());
            }
            finite(match probe {
                ElectricalProbe::Voltage { plus, minus } => circuit.voltage(plus, minus)?,
                ElectricalProbe::Current { branch } => circuit.current(branch)?,
            })?;
        }
        let description = format!("{max_step:?}{scope:?}{boards:?}{sources:?}{probes:?}");
        let identity = fnv1a(
            fnv1a(identity.iter().copied())
                .to_le_bytes()
                .into_iter()
                .chain(description.bytes()),
        );
        let boards = boards
            .into_iter()
            .map(|b| BoardPower::new(b, scope))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self {
            boards,
            circuit: Box::new(circuit),
            steps: sources.into(),
            cursor: 0,
            last: None,
            step: max_step,
            scope,
            identity,
            due: Some(Time::ZERO),
            probes: Vec::new(),
            loads: Vec::new(),
        })
    }

    fn next(&self) -> Result<Time, String> {
        let Some(at) = self.last else {
            return Ok(Time::ZERO);
        };
        let mut next = after(at, self.step.0 - at.0 % self.step.0)?;
        if let Some(step) = self.steps.get(self.cursor) {
            next = next.min(step.at);
        }
        for board in &self.boards {
            if let Some(time) = board.next(at)? {
                next = next.min(time);
            }
        }
        Ok(next)
    }

    pub fn schedule(&mut self, at: Time, horizon: Time) -> Result<(), String> {
        self.due = if at < horizon {
            Some(self.next()?.min(horizon))
        } else {
            None
        };
        Ok(())
    }

    pub fn check_horizon(&self, end: Time) -> Result<(), String> {
        // До 2^52 нс после пересчёта в секунды сохраняется разрешение в 1 нс (§8, опыт Э2).
        if end.0 > (1u64 << (f64::MANTISSA_DIGITS - 1))
            || self.steps.last().is_some_and(|step| step.at > end)
            || self.emi().is_some_and(|emi| emi.trial.input().end > end)
        {
            return Err("электрические события или горизонт вне сетки прогона".into());
        }
        Ok(())
    }

    pub fn close(&mut self, at: Time) -> Result<(), String> {
        self.circuit.advance_to(at)?;
        self.last = Some(at);
        Ok(())
    }

    pub fn samples(&self) -> Result<Vec<(crate::TruthId, f64)>, String> {
        let mut samples = self
            .probes
            .iter()
            .map(|(id, probe)| {
                let value = match probe {
                    ElectricalProbe::Voltage { plus, minus } => {
                        self.circuit.voltage(plus, minus)?
                    }
                    ElectricalProbe::Current { branch } => self.circuit.current(branch)?,
                };
                Ok((*id, finite(value)?))
            })
            .collect::<Result<Vec<_>, String>>()?;
        if let (Some(emi), Some(at)) = (self.emi(), self.last) {
            samples.push((emi.voltage, emi.trial.voltage_at(at, emi.coupled)));
        }
        Ok(samples)
    }

    fn emi(&self) -> Option<&EmiPortState> {
        self.boards.iter().find_map(|board| board.emi.as_ref())
    }

    pub fn read_board(
        &mut self,
        at: Time,
        index: usize,
    ) -> Result<(f64, f64, Option<bool>), String> {
        let board = &mut self.boards[index];
        let voltage = finite(self.circuit.voltage(&board.plus, &board.minus)?)?;
        let current = finite(self.circuit.current(&board.branch)?)?;
        let edge = board.observe(at, voltage)?;
        Ok((voltage, current, edge))
    }

    pub fn latch(
        &mut self,
        at: Time,
        index: usize,
        mode: impl Fn(u32) -> Option<GpioMode>,
    ) -> Result<Vec<(u32, PinState)>, String> {
        let board = &mut self.boards[index];
        let current = board.selected_current();
        if board.last_current != Some(current) {
            self.circuit.set_source_step(&board.branch, current)?;
            board.last_current = Some(current);
        }
        let mut changed = Vec::new();
        for pin in &board.pins {
            let id = pin.connection.pin;
            let state = match board.phase {
                PowerPhase::Unpowered => pin.unpowered,
                PowerPhase::Reset => pin.reset,
                PowerPhase::Boot => {
                    let elapsed = at.0 - board.boot.0;
                    pin.boot
                        .iter()
                        .find(|&&(end, _)| elapsed < end)
                        .ok_or("нет стадии загрузки вывода")?
                        .1
                }
                PowerPhase::Working => match mode(id)
                    .ok_or_else(|| format!("режим GPIO {}/{id} не установлен", board.name))?
                {
                    GpioMode::Output(true) => PinState::High,
                    GpioMode::Output(false) => PinState::Low,
                    GpioMode::Input => PinState::Input,
                    GpioMode::PullUp => PinState::InputPull(Pull::Up),
                    GpioMode::PullDown => PinState::InputPull(Pull::Down),
                    GpioMode::HighImpedance => PinState::HighZ,
                },
            };
            if board.states.get(&id) != Some(&state) {
                self.circuit.set_pin(&pin.connection.contact, state)?;
                board.states.insert(id, state);
                changed.push((id, state));
            }
        }
        Ok(changed)
    }

    pub fn latch_sources(&mut self, at: Time) -> Result<(), String> {
        while let Some(step) = self.steps.get(self.cursor).filter(|step| step.at == at) {
            self.circuit.set_source_step(&step.source, step.value)?;
            self.cursor += 1;
        }
        Ok(())
    }

    pub fn has_loads(&self) -> bool {
        !self.loads.is_empty()
    }

    pub fn check_load_inputs(&self, control: &control::Engine) -> Result<(), String> {
        if self
            .loads
            .iter()
            .any(|load| control.owns(load.voltage_input))
        {
            return Err("вход напряжения нагрузки уже занят контуром".into());
        }
        Ok(())
    }

    pub fn latch_loads(&mut self, at: Time, entries: &[LogEntry]) -> Result<(), String> {
        for load in &self.loads {
            let current = entries
                .iter()
                .rev()
                .find(|entry| {
                    entry.at == at
                        && entry.kind == LogKind::Truth
                        && entry.id == load.next_current.0
                })
                .filter(|entry| entry.valid)
                .map(|entry| entry.value)
                .ok_or_else(|| {
                    format!(
                        "«{}»: нагрузка не опубликовала ток шины на такте {} нс",
                        load.current_source, at.0
                    )
                })?;
            self.circuit
                .set_source_step(&load.current_source, finite(current)?)?;
        }
        Ok(())
    }
}

impl Runtime {
    /// Связать входы шины и знаковые токи приводов до установки контура.
    pub fn bind_electrical_loads(&mut self, loads: Vec<LoadConnection>) -> Result<(), String> {
        self.ensure_configurable()?;
        if loads.is_empty() {
            return Ok(());
        }
        if self.state.control.is_some() {
            return Err("нагрузки цепи подключаются до установки управления".into());
        }
        let electrical = self
            .state
            .electrical
            .as_ref()
            .ok_or("цепь не установлена")?;
        if electrical.has_loads() {
            return Err("нагрузки цепи уже подключены".into());
        }
        let mut branches: BTreeSet<_> = electrical
            .boards
            .iter()
            .map(|b| b.branch.as_str())
            .chain(electrical.steps.iter().map(|s| s.source.as_str()))
            .collect();
        let mut inputs = BTreeSet::new();
        let mut currents = BTreeSet::new();
        for load in &loads {
            self.check_channel(load.voltage_input)?;
            if !self.state.claimed.contains(&load.voltage_input)
                || self
                    .state
                    .truth
                    .spec(SignalId(load.next_current.0))
                    .is_none_or(|s| s.unit != "A")
                || !branches.insert(&load.current_source)
                || !inputs.insert(load.voltage_input)
                || !currents.insert(load.next_current)
            {
                return Err("неизвестный порт или повтор владельца электрической нагрузки".into());
            }
            electrical.circuit.validate_current_source(
                &load.plus,
                &load.minus,
                &load.current_source,
            )?;
        }
        let previous = self.state.clone();
        let rate = self.physics_rate();
        let result = (|| {
            let mut ports = Vec::new();
            for load in &loads {
                let name = &self.state.log.header.commands[load.voltage_input.0 as usize].name;
                let voltage = self.register_truth(&format!("{name}/напряжение цепи"), "V")?;
                self.state.electrical.as_mut().unwrap().probes.push((
                    voltage,
                    ElectricalProbe::Voltage {
                        plus: load.plus.clone(),
                        minus: load.minus.clone(),
                    },
                ));
                ports.push((load.voltage_input, voltage));
            }
            self.register(
                "напряжения нагрузок цепи",
                rate,
                Phase::Links,
                ports,
                |ports, input, _| {
                    let targets = ports
                        .iter()
                        .map(|&(channel, voltage)| {
                            let value = input
                                .truth
                                .plant(voltage)
                                .ok_or("нет рассчитанного напряжения нагрузки")?;
                            Ok((channel, finite(value)?))
                        })
                        .collect::<Result<Vec<_>, String>>()?;
                    Ok(Output::Commands(Command { targets }))
                },
            )?;
            let electrical = self.state.electrical.as_mut().unwrap();
            electrical.identity = fnv1a(
                electrical
                    .identity
                    .to_le_bytes()
                    .into_iter()
                    .chain(format!("{loads:?}").bytes()),
            );
            electrical.loads = loads;
            Ok(())
        })();
        if result.is_err() {
            self.state = previous;
        }
        result
    }

    /// Дойти до абсолютного момента по общему расписанию, включая обмен со схемой.
    pub fn run_electrical_until(&mut self, until: Time) -> Result<Status, String> {
        self.run_until(until)
    }
}

fn label(text: &str) -> bool {
    !text.trim().is_empty() && text == text.trim() && !text.chars().any(char::is_control)
}

fn finite(value: f64) -> Result<f64, String> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err("решатель цепи вернул неконечное значение".into())
    }
}

impl Runtime {
    /// Подключить адресованную низкоактивную линию сброса до установки контура.
    /// В обоих опытах одинаковое расписание. Меняется только возбуждение.
    pub fn bind_emi_reset(&mut self, trial: EmiTrial, coupled: bool) -> Result<(), String> {
        self.ensure_configurable()?;
        if self.state.control.is_some() {
            return Err("вход сброса подключается до установки управления".into());
        }
        let electrical = self
            .state
            .electrical
            .as_ref()
            .ok_or("цепь не установлена")?;
        if electrical.emi().is_some() || electrical.scope != trial.input().scope {
            return Err("повтор входа сброса или несовпадающая область цепи и опыта".into());
        }
        let slot = electrical
            .boards
            .iter()
            .position(|board| board.name == trial.input().board)
            .ok_or("плата адресованного входа сброса не установлена в схеме")?;
        let detector = Rail::from_detector(trial.detector(), false);
        let mode = if coupled {
            "с воздействием"
        } else {
            "контроль"
        };
        let input = trial.input();
        let voltage = self.register_truth(
            &format!(
                "ЭМС {mode} {:?}: {:?} относительно {:?}",
                input.source, input.input, input.reference
            ),
            "V",
        )?;
        let electrical = self.state.electrical.as_mut().expect("цепь проверена");
        // Обе стороны пары имеют одну идентичность установки. Режим различается
        // в таблице каналов SIMLOG, которую также проверяет загрузка повтора.
        electrical.identity = fnv1a(
            electrical
                .identity
                .to_le_bytes()
                .into_iter()
                .chain(format!("{input:?}").bytes()),
        );
        electrical.boards[slot].emi = Some(EmiPortState {
            trial: Rc::new(trial),
            coupled,
            voltage,
            detector,
            caused_reset: None,
        });
        Ok(())
    }

    /// Сопоставить живой опыт и его контроль по свидетельству штатного контура.
    /// Наблюдение пакета относится к доставке конкретного сообщения на стенде.
    pub fn compare_emi_reset(
        &self,
        baseline: &Runtime,
        packet: Option<&EmiPacketProbe>,
    ) -> Result<EmiEvidence, String> {
        let attached = |rt: &Runtime| -> Result<_, String> {
            let Some(state) = rt.state.electrical.as_ref().and_then(Electrical::emi) else {
                return Err("нет установленного входа помехи".into());
            };
            Ok((state.trial.clone(), state.coupled, state.caused_reset))
        };
        let (trial, coupled, caused_reset) = attached(self)?;
        let (quiet_trial, quiet_coupled, _) = attached(baseline)?;
        let active = control::run(self).ok_or("нет журнала опыта с помехой")?;
        let quiet = control::run(baseline).ok_or("нет журнала контроля")?;
        let valid_origin = |run: &control::ControlRun<'_>| {
            matches!(
                run.origin(),
                control::RunOrigin::Live | control::RunOrigin::Replay
            ) && !run.has_external_processes()
        };
        if !coupled
            || quiet_coupled
            || trial != quiet_trial
            || active.identity() != quiet.identity()
            || !valid_origin(&active)
            || !valid_origin(&quiet)
        {
            return Err(
                "для сравнения нужны связанный опыт и контроль той же установки и конфигурации"
                    .into(),
            );
        }
        let board = &trial.input().board;
        let reset_at = |run: &control::ControlRun<'_>| {
            run.events().iter().find_map(|event| match &event.kind {
                control::EventKind::ElectricalReset {
                    board: name,
                    asserted: true,
                } if name == board => Some(event.at),
                _ => None,
            })
        };
        let quiet_end = quiet.end();
        let active_end = active.end();
        let common = active_end.time.min(quiet_end.time);
        let before_effect = |event: &&control::Event| {
            event.at <= common && caused_reset.is_none_or(|effect| event.at < effect)
        };
        if !active
            .events()
            .iter()
            .take_while(before_effect)
            .eq(quiet.events().iter().take_while(before_effect))
        {
            return Err("журналы расходятся до действия связанного сброса".into());
        }
        let confirmed_reset = caused_reset.filter(|at| active.events().iter().any(|event| {
            matches!(&event.kind, control::EventKind::BoardOutput { board: name, event: output, produced_at }
                if name == board && produced_at == at
                    && output.output == sim_boards::BoardOutput::Reset(sim_boards::ResetCause::Brownout))
        }));
        let observed = (active_end.started && quiet_end.started).then(|| EmiResetObservation {
            observed_until: common,
            complete: active.reached_horizon() && quiet.reached_horizon(),
            reset_at: reset_at(&active),
            external_asserted_at: confirmed_reset,
            baseline_reset_at: reset_at(&quiet),
        });
        let mut evidence = assess_emi_reset(&trial, observed);
        if let Some(packet) = packet {
            let sourced = packet
                .validate(trial.input().scope)
                .map_err(|e| e.to_string())?;
            if sourced
                && active_end.started
                && quiet_end.started
                && common >= packet.deadline
                && (active_end.status == Status::Running || active_end.time > packet.deadline)
                && (quiet_end.status == Status::Running || quiet_end.time > packet.deadline)
            {
                evidence.packet = packet_witness(&active, &quiet, packet);
            }
            if evidence.packet.is_none() {
                evidence.packet_open = Some("нужны подтверждённый срок и полные журналы отправки и доставки того же пакета в паре опытов".into());
            }
        }
        Ok(evidence)
    }
}

fn packet_witness(
    active: &control::ControlRun<'_>,
    baseline: &control::ControlRun<'_>,
    probe: &EmiPacketProbe,
) -> Option<EmiPacketWitness> {
    use control::EventKind::{PacketDelivered, PacketSent};

    let sent = |run: &control::ControlRun<'_>| {
        let mut rows = run.events().iter().filter_map(|event| match &event.kind {
            PacketSent {
                route, id, bytes, ..
            } if route == &probe.route && event.at == probe.sent_at => Some((*id, bytes.clone())),
            _ => None,
        });
        let row = rows.next()?;
        rows.next().is_none().then_some(row)
    };
    let (id, bytes) = sent(baseline)?;
    if sent(active)? != (id, bytes.clone()) || bytes.is_empty() {
        return None;
    }
    let received = |run: &control::ControlRun<'_>| {
        run.events().iter().find_map(|event| match &event.kind {
            PacketDelivered {
                route,
                id: got_id,
                sent_at,
                bytes: got_bytes,
            } if route == &probe.route
                && *got_id == id
                && *sent_at == probe.sent_at
                && got_bytes == &bytes
                && event.at <= probe.deadline =>
            {
                Some(event.at)
            }
            _ => None,
        })
    };
    Some(EmiPacketWitness {
        route: probe.route.clone(),
        id,
        sent_at: probe.sent_at,
        deadline: probe.deadline,
        baseline_received_at: received(baseline)?,
        disturbed_received_at: received(active),
    })
}
