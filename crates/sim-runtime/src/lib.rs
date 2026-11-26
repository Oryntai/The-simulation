//! Рантайм без окна. На нулевом такте компоненты получают начальную позу, а первый
//! шаг физики завершается в момент первого ненулевого такта. В одном моменте
//! физика выполняется первой, другие роли следуют порядку `sim_signal::Phase`.

mod components;
pub mod control;
pub mod electrical;
mod log;
pub mod process;
mod view;

use std::cmp::Reverse;
use std::collections::{BTreeMap, BTreeSet, BinaryHeap};
use std::io;
use std::rc::Rc;

use sim_mujoco::{JointInfo, JointKind, Sim, state::SimSnapshot};
use sim_signal::{
    ChannelId, Clock, Command, ImageFrame, Observation, Phase, Rate, Rng, Sample, SignalId,
    SignalSpec, SignalTable, Time, fnv1a,
};
use sim_world::{CompiledJointKind, CompiledWorld, Simplification};

pub use components::{
    Evaluator, Forces, Link, Object, ObservationOut, Sensor, Truth, TruthId, TruthOut,
};
pub use log::{CommandSource, Log, LogEnd, LogEntry, LogHeader, LogKind};
pub use process::interaction::{ResolvedInteraction, register_interaction};
pub use view::{ViewContacts, ViewEquality, ViewRecording, ViewScene};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Running,
    NumericalFailure,
    ComponentFailure,
}

impl Status {
    fn code(self) -> u8 {
        // Коды формата журнала версии 1.
        match self {
            Self::Running => 0,
            Self::NumericalFailure => 1,
            Self::ComponentFailure => 2,
        }
    }

    fn from_code(code: u8) -> io::Result<Self> {
        match code {
            0 => Ok(Self::Running),
            1 => Ok(Self::NumericalFailure),
            2 => Ok(Self::ComponentFailure),
            _ => Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "неизвестный статус прогона",
            )),
        }
    }
}

#[derive(Clone)]
struct Replay {
    expected: Log,
    commands: Rc<Vec<LogEntry>>,
    cursor: usize,
}

#[derive(Clone)]
enum Delayed {
    Sample(Sample, Time, Time),
    Frame(ImageFrame, Time),
}

#[derive(Clone, Default)]
struct ObservationBus {
    slots: Vec<ObservationSlot>,
    frames: Vec<Option<ImageFrame>>,
    /// Показания в пути: момент доступности и порядок записи.
    pending: BTreeMap<(Time, u64), Delayed>,
    written: u64,
    transport_delays: BTreeMap<SignalId, u64>,
    confirmed_delays: BTreeSet<SignalId>,
}

#[derive(Clone, Default)]
struct ObservationSlot {
    current: Option<(Sample, Time)>,
    last_valid: Option<Sample>,
}

impl ObservationBus {
    fn confirm_delay(&mut self, signal: SignalId, available: Time, delivered: Time) {
        if self
            .transport_delays
            .get(&signal)
            .is_some_and(|&delay| delivered.0.checked_sub(available.0) == Some(delay))
        {
            self.confirmed_delays.insert(signal);
        }
    }

    fn delivery_time(&self, signal: SignalId, available: Time) -> Result<Time, String> {
        let delay = self.transport_delays.get(&signal).copied().unwrap_or(0);
        available.0.checked_add(delay).map(Time).ok_or_else(|| {
            format!(
                "задержка доставки сигнала {} вышла за шкалу времени",
                signal.0
            )
        })
    }

    fn read(&self, time: Time) -> Observation {
        self.view(time, false)
    }

    fn view(&self, time: Time, keep_last_valid: bool) -> Observation {
        let mut samples: Vec<_> = self
            .slots
            .iter()
            .filter_map(|slot| slot.current.map(|(sample, _)| sample))
            .collect();
        if keep_last_valid {
            samples.extend(self.slots.iter().filter_map(|slot| {
                slot.current.filter(|(sample, _)| !sample.valid)?;
                slot.last_valid
            }));
        }
        samples.sort_by_key(|sample| sample.at);
        let frames = self.frames.iter().flatten().cloned().collect();
        Observation {
            time,
            samples,
            frames,
        }
    }

    fn delay(&mut self, available: Time, item: Delayed) {
        self.pending.insert((available, self.written), item);
        self.written += 1;
    }

    fn next_delivery(&self) -> Option<Time> {
        self.pending.keys().next().map(|key| key.0)
    }

    fn sample(&self, id: SignalId) -> Option<(Sample, Time)> {
        self.slots.get(id.0 as usize)?.current
    }

    fn write(&mut self, sample: Sample, captured: Time) -> bool {
        let slot = &mut self.slots[sample.signal.0 as usize];
        if slot
            .current
            .is_some_and(|(_, previous)| captured < previous)
        {
            return false;
        }
        slot.current = Some((sample, captured));
        if sample.valid {
            slot.last_valid = Some(sample);
        }
        true
    }
}

struct Inputs<'a> {
    truth: Truth<'a>,
    observations: &'a ObservationBus,
    command: &'a Command,
}

enum Output {
    Observations(ObservationOut),
    Commands(Command),
    Forces(Forces),
    Truth(TruthOut),
}

trait Task {
    fn run(&mut self, input: Inputs<'_>, rng: &mut Rng) -> Result<Output, String>;
    fn copy(&self) -> Box<dyn Task>;
}

type Update<T> = fn(&mut T, Inputs<'_>, &mut Rng) -> Result<Output, String>;

struct Model<T> {
    component: T,
    update: Update<T>,
}

impl<T: Clone + 'static> Task for Model<T> {
    fn run(&mut self, input: Inputs<'_>, rng: &mut Rng) -> Result<Output, String> {
        (self.update)(&mut self.component, input, rng)
    }

    fn copy(&self) -> Box<dyn Task> {
        Box::new(Self {
            component: self.component.clone(),
            update: self.update,
        })
    }
}

impl Clone for Box<dyn Task> {
    fn clone(&self) -> Self {
        self.copy()
    }
}

#[derive(Clone)]
struct Component {
    name: String,
    phase: Phase,
    claimed: Vec<ChannelId>,
    clock: Clock,
    rng: Rng,
    task: Box<dyn Task>,
    forces: Vec<(ChannelId, f64)>,
}

#[derive(Clone)]
struct State {
    physics: Clock,
    next_physics: Option<Time>,
    queue: BinaryHeap<Reverse<(Time, Phase, u32)>>,
    components: Vec<Component>,
    observations: SignalTable,
    truth: SignalTable,
    bus: ObservationBus,
    command: Command,
    claimed: BTreeSet<ChannelId>,
    /// Последние значения, опубликованные моделями мира, по номеру канала истины.
    plant: Vec<Option<f64>>,
    /// Кинематика движка соответствует текущему qpos.
    fresh: bool,
    log: Log,
    view: Option<ViewRecording>,
    replay: Option<Replay>,
    control: Option<control::Engine>,
    electrical: Option<electrical::Electrical>,
    work: process::Processes,
    failure: Option<String>,
    sealed: bool,
}

pub struct Snapshot {
    identity: Rc<()>,
    physics: SimSnapshot,
    state: State,
    processes: Result<ProcessSnapshots, String>,
    simplifications: Vec<Simplification>,
}

type ProcessSnapshots = BTreeMap<usize, sim_control::transport::ProcessSnapshot>;

impl Snapshot {
    pub fn end(&self) -> LogEnd {
        self.state.log.end()
    }

    pub fn log_len(&self) -> usize {
        self.state.log.entries().len()
    }
}

pub struct Runtime {
    sim: Sim,
    view_scene: Rc<ViewScene>,
    world_model_hash: u64,
    process_scene: process::SceneIdentity,
    compiled_drive_names: Rc<BTreeSet<String>>,
    identity: Rc<()>,
    simplifications: Vec<Simplification>,
    effort_dofs: Vec<usize>,
    joints: Rc<Vec<JointInfo>>,
    state: State,
    processes: BTreeMap<usize, sim_control::transport::ProcessTransport>,
}

impl Runtime {
    /// Загрузить мир. Частота физики должна совпадать с шагом MJCF.
    pub fn load(world: CompiledWorld) -> Result<Self, String> {
        if world
            .drives
            .iter()
            .any(|model| model.params.values().any(|v| !v.is_finite()))
            || world.sensors.iter().any(|model| {
                model
                    .params
                    .values()
                    .chain(&model.position_m)
                    .chain(&model.quat_wxyz)
                    .any(|v| !v.is_finite())
            })
        {
            return Err("неконечные параметры модели мира".into());
        }
        let world_model_hash = fnv1a(format!("{world:?}").bytes());
        for body in &world.bodies {
            if !std::iter::once(&body.mass_kg)
                .chain(&body.com_m)
                .chain(&body.inertia_kg_m2)
                .all(|value| value.is_finite())
            {
                return Err(format!("нечисловые параметры тела «{}»", body.name));
            }
        }
        let sim = Sim::from_xml(&world.mjcf)?;
        let options = &sim.model().opt;
        let scalars = [
            options.impratio,
            options.tolerance,
            options.ls_tolerance,
            options.noslip_tolerance,
            options.ccd_tolerance,
            options.sleep_tolerance,
            options.density,
            options.viscosity,
            options.o_margin,
        ];
        if !scalars
            .iter()
            .chain(&options.gravity)
            .chain(&options.wind)
            .chain(&options.magnetic)
            .chain(&options.o_solref)
            .chain(&options.o_solimp)
            .chain(&options.o_friction)
            .all(|value| value.is_finite())
        {
            return Err("нечисловые параметры физики в MJCF".into());
        }
        let timestep = sim.timestep();
        let exact_step = 1.0 / world.physics.as_hz();
        // Запас 4 epsilon покрывает округления f64 при переводе двух целых,
        // делении, обращении частоты и чтении десятичного периода из MJCF.
        if !timestep.is_finite()
            || timestep <= 0.0
            || (timestep - exact_step).abs() > exact_step * (4.0 * f64::EPSILON)
        {
            return Err(format!(
                "шаг MJCF {timestep} с расходится с частотой мира: период {exact_step} с"
            ));
        }
        if !physics_ok(&sim) {
            return Err("начальное состояние MuJoCo численно некорректно".into());
        }

        let joints = sim.joints();
        let mut names = BTreeSet::new();
        let mut commands = Vec::new();
        let mut effort_dofs = Vec::new();
        for channel in &world.efforts {
            check_spec(&channel.name, "")?;
            if channel.name.trim().is_empty() || !names.insert(&channel.name) {
                return Err(format!("пустое или повторное имя входа «{}»", channel.name));
            }
            let declared = world
                .joints
                .iter()
                .find(|joint| joint.name == channel.joint)
                .ok_or_else(|| {
                    format!(
                        "вход «{}» ссылается на необъявленное соединение «{}»",
                        channel.name, channel.joint
                    )
                })?;
            let joint = joints
                .iter()
                .find(|joint| joint.name == channel.joint)
                .ok_or_else(|| format!("нет соединения MJCF «{}»", channel.joint))?;
            // Размерности обобщённых скоростей MuJoCo и сопряжённых с ними сил.
            let (dofs, unit) = match (declared.kind, joint.kind) {
                (CompiledJointKind::Hinge, JointKind::Hinge) => (1, "N*m"),
                (CompiledJointKind::Slide, JointKind::Slide) => (1, "N"),
                (CompiledJointKind::Ball, JointKind::Ball) => (3, "N*m"),
                (CompiledJointKind::Free, JointKind::Free) if channel.dof < 3 => (6, "N"),
                (CompiledJointKind::Free, JointKind::Free) => (6, "N*m"),
                _ => {
                    return Err(format!(
                        "тип соединения «{}» расходится с MJCF",
                        channel.joint
                    ));
                }
            };
            if channel.dof >= dofs || joint.dof_adr + channel.dof >= sim.nv() {
                return Err(format!(
                    "несуществующая степень свободы входа «{}»",
                    channel.name
                ));
            }
            effort_dofs.push(joint.dof_adr + channel.dof);
            commands.push(SignalSpec {
                name: channel.name.clone(),
                unit: unit.into(),
            });
        }

        let version = sim_mujoco::version();
        let process_scene = process::SceneIdentity::new(&world, &sim);
        let world_hash = ViewScene::world_hash(&world.name, &world.mjcf, &version, &effort_dofs);
        let header = LogHeader {
            world_hash,
            seed: world.seed,
            observations: Vec::new(),
            commands,
            truth: Vec::new(),
        };
        let end = LogEnd {
            time: Time::ZERO,
            steps: 0,
            status: Status::Running,
            started: false,
            failed_controller: None,
            state_hash: sim.state_hash(),
        };
        let mut physics = Clock::new(world.physics);
        physics.advance();
        let state = State {
            next_physics: checked_due(&physics)?,
            physics,
            queue: BinaryHeap::new(),
            components: Vec::new(),
            observations: SignalTable::default(),
            truth: SignalTable::default(),
            bus: ObservationBus::default(),
            command: Command::default(),
            claimed: BTreeSet::new(),
            plant: Vec::new(),
            fresh: true,
            log: Log::new(header, end),
            view: None,
            replay: None,
            control: None,
            electrical: None,
            work: process::Processes::default(),
            failure: None,
            sealed: false,
        };
        Ok(Self {
            sim,
            view_scene: Rc::new(ViewScene {
                name: world.name,
                mjcf: world.mjcf,
                sensors: world.sensors,
                engine: version,
                effort_dofs: effort_dofs.clone(),
            }),
            world_model_hash,
            process_scene,
            compiled_drive_names: Rc::new(
                world.drives.into_iter().map(|drive| drive.name).collect(),
            ),
            identity: Rc::new(()),
            simplifications: world.simplifications,
            effort_dofs,
            joints: Rc::new(joints),
            state,
            processes: BTreeMap::new(),
        })
    }

    pub fn time(&self) -> Time {
        self.state.log.end.time
    }
    pub fn steps(&self) -> u64 {
        self.state.log.end.steps
    }
    pub fn status(&self) -> Status {
        self.state.log.end.status
    }
    pub fn failure(&self) -> Option<&str> {
        self.state.failure.as_deref()
    }
    pub fn seed(&self) -> u64 {
        self.state.log.header.seed
    }
    pub fn physics_rate(&self) -> Rate {
        self.state.physics.rate()
    }
    pub fn simplifications(&self) -> &[Simplification] {
        &self.simplifications
    }
    pub fn log(&self) -> &Log {
        &self.state.log
    }
    pub fn observe(&self) -> Observation {
        // Прежний интерфейс разомкнутого наблюдения сохраняет последний
        // пригодный отсчёт рядом с сообщением об обрыве. Входы компонентов
        // и замкнутого контура получают только текущие показания.
        self.state
            .bus
            .view(self.time(), self.state.control.is_none())
    }

    pub fn channel(&self, name: &str) -> Option<ChannelId> {
        let index = self
            .state
            .log
            .header
            .commands
            .iter()
            .position(|spec| spec.name == name)?;
        u32::try_from(index).ok().map(ChannelId)
    }

    pub fn truth_spec(&self, id: TruthId) -> Option<&SignalSpec> {
        self.state.truth.spec(SignalId(id.0))
    }

    /// Канал истины по имени: так датчик находит опубликованное состояние привода.
    pub fn truth_id(&self, name: &str) -> Option<TruthId> {
        self.state.truth.id(name).map(|id| TruthId(id.0))
    }

    /// Приведённая инерция степени свободы соединения в MJCF.
    pub fn joint_armature(&self, joint: &str, dof: usize) -> Option<f64> {
        let info = self.joints.iter().find(|j| j.name == joint)?;
        let armature = self.sim.model().dof_armature;
        let index = info.dof_adr + dof;
        (index < self.sim.nv()).then(|| unsafe { *armature.add(index) })
    }

    fn joint_velocity_index(&self, joint: &str, dof: u8) -> Option<usize> {
        let info = self.joints.iter().find(|info| info.name == joint)?;
        let count = match info.kind {
            JointKind::Free => 6,
            JointKind::Ball => 3,
            JointKind::Slide | JointKind::Hinge => 1,
        };
        (usize::from(dof) < count).then_some(info.dof_adr + usize::from(dof))
    }

    /// Скорость степени свободы в действительном состоянии движка.
    pub fn joint_velocity(&self, joint: &str, dof: u8) -> Option<f64> {
        self.sim
            .qvel()
            .get(self.joint_velocity_index(joint, dof)?)
            .copied()
    }

    /// Установить начальные скорости в единицах MuJoCo до первого такта.
    pub fn set_initial_velocities(
        &mut self,
        values: &BTreeMap<String, BTreeMap<u8, f64>>,
    ) -> Result<(), String> {
        self.ensure_configurable()?;
        let mut resolved = Vec::new();
        for (joint, dofs) in values {
            for (&dof, &value) in dofs {
                let index = self
                    .joint_velocity_index(joint, dof)
                    .filter(|_| value.is_finite())
                    .ok_or_else(|| format!("неверная начальная скорость «{joint}»[{dof}]"))?;
                resolved.push((index, value));
            }
        }
        for (index, value) in resolved {
            self.sim.qvel_mut()[index] = value;
        }
        if !values.is_empty() {
            self.state.fresh = false;
        }
        Ok(())
    }

    /// Задержка доставки наблюдения поверх задержки захвата модели
    /// (`observation:<имя сигнала>`) либо базовая задержка провода (`channel:<имя>`).
    pub fn channel_delay_ns(&self, channel: &str) -> Option<u64> {
        if let Some(name) = channel.strip_prefix("observation:") {
            let id = self.observation_id(name)?;
            return Some(
                self.state
                    .bus
                    .transport_delays
                    .get(&id)
                    .copied()
                    .unwrap_or(0),
            );
        }
        self.state
            .control
            .as_ref()?
            .wired_delay(channel.strip_prefix("channel:")?)
    }

    /// Установить задержки только на зарегистрированных каналах до первого такта.
    pub fn set_channel_delays(&mut self, delays: &BTreeMap<String, u64>) -> Result<(), String> {
        self.ensure_configurable()?;
        for channel in delays.keys() {
            if self.channel_delay_ns(channel).is_none() {
                return Err(format!(
                    "у канала «{channel}» нет настраиваемой задержки провода"
                ));
            }
        }
        for (channel, &delay) in delays {
            if let Some(name) = channel.strip_prefix("observation:") {
                let id = self.observation_id(name).expect("адрес проверен");
                self.state.bus.transport_delays.insert(id, delay);
            } else {
                let name = channel.strip_prefix("channel:").expect("адрес проверен");
                self.state
                    .control
                    .as_mut()
                    .expect("проводка проверена")
                    .set_wired_delay(name, delay);
            }
        }
        Ok(())
    }

    /// Есть живая доставка по установленному пути с момента начала прогона.
    pub fn channel_delay_observed(&self, channel: &str) -> bool {
        if let Some(name) = channel.strip_prefix("observation:") {
            return self
                .observation_id(name)
                .is_some_and(|id| self.state.bus.confirmed_delays.contains(&id));
        }
        let Some(name) = channel.strip_prefix("channel:") else {
            return false;
        };
        let (Some(delay), Some(run)) = (self.channel_delay_ns(channel), control::run(self)) else {
            return false;
        };
        let mut arrivals = BTreeSet::new();
        for event in run.events() {
            match &event.kind {
                control::EventKind::Packet {
                    route,
                    event: packet,
                } if route == name
                    && matches!(packet.outcome, control::PacketOutcome::Delivered { .. })
                    && event.at.0.checked_sub(packet.sent_at.0) == Some(delay) =>
                {
                    arrivals.insert(packet.id);
                }
                control::EventKind::PacketDelivered { route, id, .. }
                    if route == name && arrivals.contains(id) =>
                {
                    return true;
                }
                _ => (),
            }
        }
        false
    }

    /// Записать упрощение модели, подключённой после компиляции мира.
    pub fn add_simplification(&mut self, item: Simplification) -> Result<(), String> {
        self.ensure_configurable()?;
        if !self.simplifications.contains(&item) {
            self.simplifications.push(item);
        }
        Ok(())
    }

    pub fn observation_id(&self, name: &str) -> Option<SignalId> {
        self.state.observations.id(name)
    }

    pub fn register_observation(&mut self, name: &str, unit: &str) -> Result<SignalId, String> {
        self.ensure_configurable()?;
        check_spec(name, unit)?;
        let id = self.state.observations.register(name, unit)?;
        self.state.log.header.observations.push(SignalSpec {
            name: name.into(),
            unit: unit.into(),
        });
        self.state.bus.slots.push(ObservationSlot::default());
        self.state.bus.frames.push(None);
        Ok(id)
    }

    pub fn register_truth(&mut self, name: &str, unit: &str) -> Result<TruthId, String> {
        self.ensure_configurable()?;
        check_spec(name, unit)?;
        let id = self.state.truth.register(name, unit)?;
        self.state.plant.push(None);
        self.state.log.header.truth.push(SignalSpec {
            name: name.into(),
            unit: unit.into(),
        });
        Ok(TruthId(id.0))
    }

    /// Прореживать ряды истины в журнале: из каждого окна по каналу остаются первое
    /// значение, минимум и максимум. Проверки диапазонов видят те же экстремумы.
    pub fn set_truth_log_rate(&mut self, rate: Rate) -> Result<(), String> {
        self.ensure_configurable()?;
        if rate.as_hz() > self.physics_rate().as_hz() {
            return Err("частота журнала истины выше частоты физики".into());
        }
        let period = rate.tick(1).0;
        if period == 0 || rate.tick(2).0 != 2 * period {
            return Err("период журнала истины должен быть целым числом наносекунд".into());
        }
        self.state.log.set_truth_period(period);
        Ok(())
    }

    /// Записывать координаты и скорости всех соединений на тактах физики,
    /// включая начальное состояние. Имена каналов содержат индексы MJCF.
    pub fn record_joint_state(&mut self, name: &str) -> Result<(), String> {
        self.record_joint_state_at(name, self.physics_rate())
    }

    /// То же с заданной частотой записи. На высокой частоте физики запись на
    /// каждом такте занимает сотни мегабайт на секунду модели.
    pub fn record_joint_state_at(&mut self, name: &str, rate: Rate) -> Result<(), String> {
        self.ensure_configurable()?;
        if rate.as_hz() > self.physics_rate().as_hz() {
            return Err("частота записи состояния выше частоты физики".into());
        }
        self.check_component_name(name)?;
        let mut coordinates = vec![""; self.sim.nq()];
        let mut velocities = vec![""; self.sim.nv()];
        for joint in self.joints.iter() {
            // Порядок координат и скоростей соединений в MuJoCo.
            let q = joint.qpos_adr;
            let v = joint.dof_adr;
            let (pos, vel): (&[&str], &[&str]) = match joint.kind {
                JointKind::Hinge => (&["rad"], &["rad/s"]),
                JointKind::Slide => (&["m"], &["m/s"]),
                JointKind::Ball => (&["1"; 4], &["rad/s"; 3]),
                JointKind::Free => (
                    &["m", "m", "m", "1", "1", "1", "1"],
                    &["m/s", "m/s", "m/s", "rad/s", "rad/s", "rad/s"],
                ),
            };
            coordinates[q..q + pos.len()].copy_from_slice(pos);
            velocities[v..v + vel.len()].copy_from_slice(vel);
        }
        let specs = |prefix, units: Vec<&'static str>| {
            units
                .into_iter()
                .enumerate()
                .map(|(i, unit)| (format!("{prefix}[{i}]"), unit))
                .collect::<Vec<_>>()
        };
        let qpos = specs("qpos", coordinates);
        let qvel = specs("qvel", velocities);
        for (channel, _) in qpos.iter().chain(&qvel) {
            if self.state.truth.id(channel).is_some() {
                return Err(format!("истинный сигнал «{channel}» уже зарегистрирован"));
            }
        }
        let mut register = |specs: Vec<(String, &'static str)>| {
            specs
                .into_iter()
                .map(|(channel, unit)| self.register_truth(&channel, unit))
                .collect::<Result<Vec<_>, _>>()
        };
        let recorder = components::RecordState {
            qpos: register(qpos)?,
            qvel: register(qvel)?,
        };
        self.register_evaluator(name, rate, recorder)
    }

    pub fn register_sensor(
        &mut self,
        name: &str,
        rate: Rate,
        sensor: impl Sensor + Clone + 'static,
    ) -> Result<(), String> {
        self.register(name, rate, Phase::Sensors, sensor, |sensor, input, rng| {
            let mut out = ObservationOut::default();
            sensor.sample(&input.truth, rng, &mut out)?;
            Ok(Output::Observations(out))
        })
    }

    pub fn register_link(
        &mut self,
        name: &str,
        rate: Rate,
        link: impl Link + Clone + 'static,
    ) -> Result<(), String> {
        self.register(name, rate, Phase::Links, link, |link, input, rng| {
            let mut out = ObservationOut::default();
            link.transfer(&input.observations.read(input.truth.time()), rng, &mut out)?;
            Ok(Output::Observations(out))
        })
    }

    /// Контроллер — клонируемый вызываемый объект. Его код может зависеть только от
    /// sim-signal: функция `fn(&Observation) -> Command` подходит напрямую. Генератор
    /// для замыкания можно создать как `Rng::stream(runtime.seed(), name)` при регистрации.
    ///
    /// ```
    /// use sim_signal::{Command, Observation};
    ///
    /// fn управление(input: &Observation) -> Command {
    ///     assert!(input.samples.is_empty());
    ///     Command::default()
    /// }
    /// assert!(управление(&Observation::default()).targets.is_empty());
    /// ```
    ///
    /// ```compile_fail,E0593
    /// use sim_runtime::{Runtime, Truth};
    /// use sim_signal::{Command, Observation, Rate};
    /// fn установить(r: &mut Runtime, rate: Rate) {
    ///     r.register_controller("управление", rate, |_: &Observation, _: &Truth<'_>| Command::default());
    /// }
    /// ```
    pub fn register_controller(
        &mut self,
        name: &str,
        rate: Rate,
        controller: impl FnMut(&Observation) -> Command + Clone + 'static,
    ) -> Result<(), String> {
        if self.state.control.is_some() {
            return Err("встроенный контур уже владеет расписанием контроллеров".into());
        }
        self.register(
            name,
            rate,
            Phase::Controllers,
            controller,
            |controller, input, _| {
                Ok(Output::Commands(controller(
                    &input.observations.read(input.truth.time()),
                )))
            },
        )
    }

    /// Каналы из `claimed` подаются модели через Command. Для них отключается
    /// прямая передача команды в физику. Возвращённые силы всех моделей суммируются.
    pub fn register_object(
        &mut self,
        name: &str,
        rate: Rate,
        claimed: &[ChannelId],
        object: impl Object + Clone + 'static,
    ) -> Result<(), String> {
        let mut unique = BTreeSet::new();
        for &channel in claimed {
            self.check_channel(channel)?;
            if !unique.insert(channel) || self.state.claimed.contains(&channel) {
                return Err(format!("для канала {} уже назначена модель", channel.0));
            }
        }
        self.register(
            name,
            rate,
            Phase::Actuators,
            object,
            |object, input, rng| {
                let mut out = Forces::default();
                object.update(&input.truth, input.command, rng, &mut out)?;
                Ok(Output::Forces(out))
            },
        )?;
        self.state
            .components
            .last_mut()
            .expect("объект зарегистрирован")
            .claimed = claimed.to_vec();
        self.state.claimed.extend(unique);
        Ok(())
    }

    pub fn register_evaluator(
        &mut self,
        name: &str,
        rate: Rate,
        evaluator: impl Evaluator + Clone + 'static,
    ) -> Result<(), String> {
        self.register(
            name,
            rate,
            Phase::Evaluators,
            evaluator,
            |evaluator, input, rng| {
                let mut out = TruthOut::default();
                evaluator.evaluate(&input.truth, rng, &mut out)?;
                Ok(Output::Truth(out))
            },
        )
    }

    fn register<T: Clone + 'static>(
        &mut self,
        name: &str,
        rate: Rate,
        phase: Phase,
        component: T,
        update: Update<T>,
    ) -> Result<(), String> {
        self.ensure_configurable()?;
        self.check_component_name(name)?;
        let index =
            u32::try_from(self.state.components.len()).map_err(|_| "слишком много компонентов")?;
        let clock = Clock::new(rate);
        let rng = Rng::stream(self.seed(), name);
        self.state.components.push(Component {
            name: name.into(),
            phase,
            claimed: Vec::new(),
            clock,
            rng,
            task: Box::new(Model { component, update }),
            forces: Vec::new(),
        });
        self.state.queue.push(Reverse((Time::ZERO, phase, index)));
        Ok(())
    }

    fn check_component_name(&self, name: &str) -> Result<(), String> {
        if name.trim().is_empty() || self.state.components.iter().any(|e| e.name == name) {
            return Err(format!("пустое или повторное имя компонента «{name}»"));
        }
        Ok(())
    }

    fn ensure_configurable(&self) -> Result<(), String> {
        if self.state.sealed {
            Err("конфигурация прогона уже зафиксирована".into())
        } else {
            Ok(())
        }
    }

    fn check_channel(&self, channel: ChannelId) -> Result<(), String> {
        if self.effort_dofs.get(channel.0 as usize).is_none() {
            return Err(format!("неизвестный канал {}", channel.0));
        }
        Ok(())
    }

    fn check_targets(&self, targets: &[(ChannelId, f64)]) -> Result<(), String> {
        let mut channels = BTreeSet::new();
        for &(channel, value) in targets {
            self.check_channel(channel)?;
            if !value.is_finite() || !channels.insert(channel) {
                return Err(format!(
                    "нечисловое значение или повтор канала {}",
                    channel.0
                ));
            }
        }
        Ok(())
    }

    /// Удерживать обновлённые команды до следующей записи в те же каналы.
    /// В момент вызова физика уже выполнена, если рантайм дошёл до этого момента.
    pub fn act(&mut self, command: &Command) -> Result<(), String> {
        if self.status() != Status::Running || self.state.replay.is_some() {
            return Err(
                "внешняя команда недоступна после остановки или во время воспроизведения".into(),
            );
        }
        self.check_targets(&command.targets)?;
        let at = self.time();
        if let Some(control) = &mut self.state.control
            && !command.targets.is_empty()
        {
            if command
                .targets
                .iter()
                .any(|(channel, _)| control.owns(*channel))
            {
                return Err("командный канал занят контуром или объявленным стимулом".into());
            }
            control.record_external(at, &command.targets, &self.state.log.header.commands)?;
        }
        self.state.sealed = true;
        let source = if self.state.log.end.started {
            CommandSource::External
        } else {
            CommandSource::BeforeStart
        };
        self.apply_commands(&command.targets, source);
        Ok(())
    }

    fn apply_commands(&mut self, targets: &[(ChannelId, f64)], source: CommandSource) {
        let at = self.time();
        for &(channel, value) in targets {
            let held = &mut self.state.command.targets;
            match held.binary_search_by_key(&channel, |&(id, _)| id) {
                Ok(index) => held[index].1 = value,
                Err(index) => held.insert(index, (channel, value)),
            }
            self.state.log.push(LogEntry {
                at,
                kind: LogKind::Command(source),
                id: channel.0,
                value,
                valid: true,
            });
        }
    }

    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            identity: self.identity.clone(),
            physics: self.sim.snapshot(),
            state: self.state.clone(),
            processes: self.snapshot_processes(),
            simplifications: self.simplifications.clone(),
        }
    }

    pub fn try_snapshot(&self) -> Result<Snapshot, String> {
        let snapshot = self.snapshot();
        snapshot.processes.as_ref().map_err(Clone::clone)?;
        Ok(snapshot)
    }

    fn snapshot_processes(&self) -> Result<ProcessSnapshots, String> {
        if let Some(board) = self
            .state
            .control
            .as_ref()
            .and_then(|control| control.external_boards.first())
        {
            return Err(format!(
                "внешняя плата «{board}»: нет снимка QEMU для восстановления контура"
            ));
        }
        self.processes
            .iter()
            .map(|(&id, process)| {
                process
                    .snapshot()
                    .map(|snap| (id, snap))
                    .map_err(|e| e.to_string())
            })
            .collect()
    }

    pub fn restore(&mut self, snapshot: &Snapshot) -> Result<(), String> {
        self.verify_snapshot_owner(snapshot)?;
        if let Some(control) = &snapshot.state.control {
            control.verify_executables()?;
        }
        let processes = restore_processes(snapshot.processes.as_ref().map_err(Clone::clone)?)?;
        self.sim.restore(&snapshot.physics)?;
        self.state = snapshot.state.clone();
        self.simplifications = snapshot.simplifications.clone();
        self.processes = processes;
        Ok(())
    }

    /// Проверка идентичности экземпляра без восстановления состояния или процессов.
    pub fn verify_snapshot_owner(&self, snapshot: &Snapshot) -> Result<(), String> {
        if !Rc::ptr_eq(&self.identity, &snapshot.identity) {
            return Err("снимок снят с другого экземпляра мира".into());
        }
        Ok(())
    }

    /// Сверяет непрерывный прогон и восстановление в том же экземпляре.
    /// При расхождении сохраняет завершённое непрерывное исполнение.
    pub fn audit_snapshot(&mut self, at: Time, horizon: Time) -> Result<(), String> {
        if at <= self.time() || at >= horizon {
            return Err("такт снимка должен быть внутри оставшегося интервала".into());
        }
        if control::run(self).is_some_and(|run| run.has_external_processes()) {
            return Err(control::EXTERNAL_ISOLATION_GAP.into());
        }
        let initial = self.try_snapshot()?;
        if self.run_until(horizon)? != Status::Running || self.time() != horizon {
            return Err("не достигнут горизонт непрерывного прогона".into());
        }
        let completed = self.try_snapshot()?;
        let evidence = |rt: &Self| {
            control::run(rt).map(|run| {
                let recording = run.recording();
                (recording.events, recording.exchanges)
            })
        };
        let uninterrupted = evidence(self);
        let result = (|| {
            self.restore(&initial)?;
            if self.run_until(at)? != Status::Running || self.time() != at {
                return Err("не достигнут заявленный такт снимка".into());
            }
            let checkpoint = self.try_snapshot()?;
            if self.fork()?.verify_snapshot_owner(&checkpoint).is_ok() {
                return Err("принят снимок другого экземпляра".into());
            }
            self.restore(&checkpoint)?;
            self.run_until(horizon)?;
            let same_control = match (&uninterrupted, evidence(self)) {
                (None, None) => true,
                (Some((events, exchanges)), Some((other_events, other_exchanges))) => {
                    // Представление события сохраняет также знак нуля.
                    events.len() == other_events.len()
                        && events
                            .iter()
                            .zip(other_events.iter())
                            .all(|(a, b)| format!("{a:?}") == format!("{b:?}"))
                        && exchanges.len() == other_exchanges.len()
                        && exchanges
                            .iter()
                            .zip(other_exchanges.iter())
                            .all(|(a, b)| format!("{a:?}") == format!("{b:?}"))
                }
                _ => false,
            };
            if !same_control || !self.log().matches(&completed.state.log) {
                return Err("после снимка изменились журнал, обмен или итог".into());
            }
            Ok(())
        })();
        if result.is_err() {
            self.restore(&completed)?;
        }
        result
    }

    /// Версия и отпечаток файла загруженной библиотеки физики.
    #[cfg(target_vendor = "apple")]
    pub fn physics_engine_identity() -> Result<(String, u64), String> {
        use std::ffi::{CStr, OsStr, c_char, c_int, c_void};
        use std::os::unix::ffi::OsStrExt;
        use std::path::Path;

        // Раскладка Dl_info из <dlfcn.h> SDK macOS.
        #[repr(C)]
        struct ImageInfo {
            file: *const c_char,
            base: *mut c_void,
            symbol: *const c_char,
            address: *mut c_void,
        }
        unsafe extern "C" {
            fn dladdr(address: *const c_void, info: *mut ImageInfo) -> c_int;
        }
        let mut info = std::mem::MaybeUninit::uninit();
        let address = sim_mujoco::ffi::mj_versionString as *const c_void;
        // При успехе dyld заполняет Dl_info и владеет строкой пути загруженного образа.
        if unsafe { dladdr(address, info.as_mut_ptr()) } == 0 {
            return Err("dyld не разрешил образ MuJoCo".into());
        }
        let info = unsafe { info.assume_init() };
        if info.file.is_null() {
            return Err("dyld не вернул путь загруженного MuJoCo".into());
        }
        let path = Path::new(OsStr::from_bytes(
            unsafe { CStr::from_ptr(info.file) }.to_bytes(),
        ));
        let bytes = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
        Ok((sim_mujoco::version(), fnv1a(bytes)))
    }

    #[cfg(not(target_vendor = "apple"))]
    pub fn physics_engine_identity() -> Result<(String, u64), String> {
        Err("определение двоичного образа MuJoCo на этой платформе не поддержано".into())
    }

    pub fn fork(&self) -> Result<Self, String> {
        if let Some(control) = &self.state.control {
            control.verify_executables()?;
        }
        let processes = restore_processes(&self.snapshot_processes()?)?;
        Ok(Self {
            sim: self.sim.fork()?,
            view_scene: self.view_scene.clone(),
            world_model_hash: self.world_model_hash,
            process_scene: self.process_scene,
            compiled_drive_names: self.compiled_drive_names.clone(),
            identity: Rc::new(()),
            simplifications: self.simplifications.clone(),
            effort_dofs: self.effort_dofs.clone(),
            joints: self.joints.clone(),
            state: self.state.clone(),
            processes,
        })
    }

    pub fn next_physics_time(&self) -> Option<Time> {
        self.state
            .next_physics
            .filter(|_| self.status() == Status::Running)
    }

    /// Установить цепь до сборки живого контура или его воспроизведения.
    /// `identity` связывает запись с байтами схемы, каталога и настройками решателя.
    pub fn install_electrical(
        &mut self,
        setup: electrical::ElectricalSetup,
        identity: &[u8],
        circuit: impl electrical::CircuitModel + Clone,
    ) -> Result<(), String> {
        self.ensure_configurable()?;
        if self.state.electrical.is_some() || self.state.control.is_some() {
            return Err("цепь устанавливается один раз до сборки управления".into());
        }
        let probes = setup.probes.clone();
        let mut electrical = electrical::Electrical::new(setup, identity, circuit)?;
        let previous = self.state.clone();
        for (name, probe) in probes {
            match self.register_truth(&name, probe.unit()) {
                Ok(id) => electrical.probes.push((id, probe)),
                Err(reason) => {
                    self.state = previous;
                    return Err(reason);
                }
            }
        }
        self.add_simplification(Simplification {
            subject: "электрическая связь платы".into(),
            text: "Сброс и GPIO рассчитаны на сетке обмена; непрерывная огибающая и поведение физической сборки требуют сверки".into(),
            blocks_proof: true,
        })?;
        self.state.electrical = Some(electrical);
        Ok(())
    }

    /// Ближайший момент, когда есть работа, включая внешние команды воспроизведения.
    pub fn next_event_time(&self) -> Option<Time> {
        if self.status() != Status::Running {
            return None;
        }
        let next_command = self
            .state
            .replay
            .as_ref()
            .and_then(|r| r.commands.get(r.cursor))
            .map(|e| e.at);
        self.state
            .next_physics
            .into_iter()
            .chain(self.state.queue.peek().map(|e| e.0.0))
            .chain(self.state.bus.next_delivery())
            .chain(
                self.state
                    .electrical
                    .as_ref()
                    .and_then(|circuit| circuit.due),
            )
            .chain(
                (self.process_recording_enabled() && self.process_journal().is_empty())
                    .then_some(Time::ZERO),
            )
            .chain(
                self.state
                    .control
                    .as_ref()
                    .and_then(control::Engine::next_event),
            )
            .chain(next_command)
            .min()
    }

    /// Дойти до следующего шага физики включительно.
    pub fn step(&mut self) -> Result<Status, String> {
        if self.status() != Status::Running {
            return Ok(self.status());
        }
        let at = self
            .next_physics_time()
            .ok_or("следующий шаг физики за пределами шкалы времени")?;
        self.run_until(at)
    }

    /// Дойти до абсолютного момента, выполнив все события вплоть до него.
    pub fn run_until(&mut self, until: Time) -> Result<Status, String> {
        if let Some(circuit) = &self.state.electrical {
            let control = self
                .state
                .control
                .as_ref()
                .ok_or("к электрической цепи не установлен контур с платами")?;
            circuit.check_load_inputs(control)?;
        }
        if until < self.time() {
            return Err("задан момент в прошлом: используйте restore".into());
        }
        if self
            .state
            .replay
            .as_ref()
            .is_some_and(|r| until > r.expected.end().time)
        {
            return Err("заданный момент выходит за конец воспроизведения".into());
        }
        if self
            .state
            .control
            .as_ref()
            .is_some_and(|control| until > control.horizon())
        {
            return Err("заданный момент выходит за горизонт контура".into());
        }
        self.state.sealed = true;
        if self
            .state
            .replay
            .as_ref()
            .is_some_and(|r| !r.expected.end().started)
        {
            return Ok(self.status());
        }
        self.state.log.end.started = true;
        // До следующего выхода из рантайма журнал дописывается одним
        // владельцем. Снимки сохраняют прежние строки по copy-on-write.
        self.state.log.set_process_ticks(None);
        while let Some(at) = self.next_event_time().filter(|&at| at <= until) {
            self.state.log.end.time = at;
            let physics_boundary = self.state.next_physics == Some(at);
            let load_boundary = (at == Time::ZERO || physics_boundary)
                && self
                    .state
                    .electrical
                    .as_ref()
                    .is_some_and(electrical::Electrical::has_loads);
            let electrical_boundary = self.state.electrical.as_ref().is_some_and(|circuit| {
                load_boundary
                    || circuit.due == Some(at)
                    || self
                        .state
                        .control
                        .as_ref()
                        .is_some_and(|control| control.electrical_board_due(at))
            });
            if electrical_boundary && let Err(reason) = self.close_electrical(at) {
                self.stop(Status::ComponentFailure, reason);
                break;
            }
            // Отсчёты цепи относятся к завершённому интервалу. Свежий ток
            // нагрузки для следующего интервала появится позже на этом такте.
            let load_log_start = load_boundary.then(|| self.state.log.entries().len());
            if physics_boundary {
                if !self.step_physics() {
                    break;
                }
                if let Some(control) = &mut self.state.control {
                    control.integrated(at);
                }
                self.state.physics.advance();
                match checked_due(&self.state.physics) {
                    Ok(next) => self.state.next_physics = next,
                    Err(reason) => {
                        self.stop(Status::ComponentFailure, reason);
                        break;
                    }
                }
            }
            if let Err(error) = self.control_phase(at, Phase::Physics) {
                self.stop(Status::ComponentFailure, error);
                break;
            }
            if electrical_boundary
                && let (Some(control), Some(circuit)) =
                    (&mut self.state.control, &mut self.state.electrical)
                && let Err(reason) = control.electrical_advance(&self.state.bus, at, circuit)
            {
                self.stop(Status::ComponentFailure, reason);
                break;
            }
            self.deliver(at);
            for phase in [
                Phase::Sensors,
                Phase::Links,
                Phase::Controllers,
                Phase::Actuators,
                Phase::Evaluators,
            ] {
                if self.status() != Status::Running {
                    break;
                }
                if let Err(error) = self.control_phase(at, phase) {
                    self.stop(Status::ComponentFailure, error);
                    break;
                }
                if electrical_boundary
                    && phase == Phase::Links
                    && let (Some(control), Some(circuit)) =
                        (&mut self.state.control, &mut self.state.electrical)
                    && let Err(reason) = control.electrical_latch(at, circuit)
                {
                    self.stop(Status::ComponentFailure, reason);
                    break;
                }
                let boundary = phase == Phase::Actuators && (at == Time::ZERO || physics_boundary);
                let prepared = if boundary && self.process_recording_enabled() {
                    match self.prepare_process_boundary(at) {
                        Ok(value) => Some(value),
                        Err(error) => {
                            self.stop_process(error);
                            break;
                        }
                    }
                } else {
                    None
                };
                while let Some(&Reverse((time, next_phase, index))) = self.state.queue.peek() {
                    if time != at || next_phase != phase || self.status() != Status::Running {
                        break;
                    }
                    self.state.queue.pop();
                    // После шага MuJoCo позы тел отстают от qpos на шаг.
                    if matches!(phase, Phase::Sensors | Phase::Evaluators) && !self.state.fresh {
                        self.sim.forward();
                        self.state.fresh = true;
                    }
                    if let Err(reason) = self.run_component(phase, index, prepared.as_ref()) {
                        if phase == Phase::Controllers {
                            self.state.log.end.failed_controller = Some(index);
                        }
                        let name = &self.state.components[index as usize].name;
                        self.stop(
                            Status::ComponentFailure,
                            format!("компонент «{name}»: {reason}"),
                        );
                    }
                }
                if boundary && self.status() == Status::Running {
                    if let Some(mut control) = self.state.control.take() {
                        let result = control.latch(at, self);
                        self.state.control = Some(control);
                        if let Err(reason) = result {
                            self.stop(Status::ComponentFailure, reason);
                        }
                    }
                    if self.status() == Status::Running
                        && let Some(prepared) = prepared
                        && let Err(reason) = self.commit_process_boundary(at, prepared)
                    {
                        self.stop_process(reason);
                    }
                }
            }
            if self.status() == Status::Running {
                if let Some(start) = load_log_start
                    && let Some(circuit) = &mut self.state.electrical
                    && let Err(reason) = circuit.latch_loads(at, &self.state.log.entries()[start..])
                {
                    self.stop(Status::ComponentFailure, reason);
                    break;
                }
                self.inject(CommandSource::External);
                if self
                    .state
                    .replay
                    .as_ref()
                    .and_then(|r| r.commands.get(r.cursor))
                    .is_some_and(|entry| entry.at == at)
                {
                    self.stop(
                        Status::ComponentFailure,
                        "в журнале команда без соответствующего такта контроллера".into(),
                    );
                }
            }
        }
        if self.status() == Status::Running {
            self.state.log.end.time = until;
        }
        self.state.log.end.state_hash = self.process_state_hash(self.sim.state_hash());
        self.state.log.flush();
        self.state
            .log
            .set_process_ticks(Some(self.state.work.journal.clone()));
        Ok(self.status())
    }

    fn control_phase(&mut self, at: Time, phase: Phase) -> Result<(), String> {
        let Some(mut control) = self.state.control.take() else {
            return Ok(());
        };
        let result = control.phase(self, at, phase);
        self.state.control = Some(control);
        result
    }

    fn close_electrical(&mut self, at: Time) -> Result<(), String> {
        let Some(circuit) = &mut self.state.electrical else {
            return Ok(());
        };
        circuit.close(at)?;
        for (id, value) in circuit.samples()? {
            self.state.plant[id.0 as usize] = Some(value);
            self.state.log.push(LogEntry {
                at,
                kind: LogKind::Truth,
                id: id.0,
                value,
                valid: true,
            });
        }
        Ok(())
    }

    fn stop(&mut self, status: Status, reason: String) {
        self.state.log.end.status = status;
        let at = self.time();
        if let Some(control) = &mut self.state.control {
            control.finish_failure(at, "рантайм", &reason);
        }
        self.state.failure = Some(reason);
    }

    fn stop_process(&mut self, error: String) {
        self.state
            .log
            .record_process(self.time(), self.state.work.failure_record(&error));
        self.stop(Status::ComponentFailure, error);
    }

    fn step_physics(&mut self) -> bool {
        if !physics_ok(&self.sim) {
            self.stop(
                Status::NumericalFailure,
                "MuJoCo: некорректное состояние перед шагом".into(),
            );
            return false;
        }
        if self.sim.nv() != 0
            && let Err(reason) = sum_forces(
                &self.state,
                &self.effort_dofs,
                &self.state.work.efforts,
                self.sim.qfrc_applied_mut(),
            )
        {
            self.stop(Status::NumericalFailure, reason);
            return false;
        }
        let previous = self.sim.time();
        let expected = previous + self.sim.timestep();
        let held = self.state.work.scalar_hold.then(|| self.sim.qpos()[0]);
        if self.process_recording_enabled() {
            if let Err(reason) = self.capture_process_physics() {
                self.state.log.record_process(self.time(), reason.clone());
                self.stop(Status::NumericalFailure, reason);
                return false;
            }
        } else {
            self.sim.step();
            self.capture_view_contacts(self.time());
        }
        if let Some(q) = held {
            self.sim.qpos_mut()[0] = q;
            self.sim.qvel_mut()[0] = 0.0;
        }
        self.state.fresh = false;
        if !physics_ok(&self.sim) || self.sim.time() <= previous || self.sim.time() != expected {
            self.stop(
                Status::NumericalFailure,
                "MuJoCo: расхождение или сброс при шаге физики".into(),
            );
            return false;
        }
        self.state.log.end.steps += 1;
        true
    }

    /// Выдать показания, чей момент доступности наступил.
    fn deliver(&mut self, at: Time) {
        while let Some(entry) = self.state.bus.pending.first_entry() {
            if entry.key().0 > at {
                break;
            }
            match entry.remove() {
                Delayed::Sample(sample, captured, available) => {
                    if self.publish_sample(sample, captured) && sample.valid {
                        self.state.bus.confirm_delay(sample.signal, available, at);
                    }
                }
                Delayed::Frame(frame, available) => {
                    let signal = frame.signal;
                    if self.publish_frame(frame) {
                        self.state.bus.confirm_delay(signal, available, at);
                    }
                }
            }
        }
    }

    fn publish_sample(&mut self, mut sample: Sample, captured: Time) -> bool {
        let ordered = self.state.bus.write(sample, captured);
        sample.valid &= ordered;
        if let Some(control) = &mut self.state.control {
            control.measurement(
                sample.signal,
                captured,
                sample.at,
                sample.valid,
                sample.value,
            );
        }
        self.state.log.push(LogEntry {
            at: sample.at,
            kind: LogKind::Observation,
            id: sample.signal.0,
            value: sample.value,
            valid: sample.valid,
        });
        ordered
    }

    /// В лог идёт отпечаток кадра: так изображения участвуют в побитовой сверке прогонов.
    fn publish_frame(&mut self, mut frame: ImageFrame) -> bool {
        let index = frame.signal.0 as usize;
        let current = self.state.bus.frames[index].as_ref();
        let ordered = current.is_none_or(|last| frame.captured >= last.captured);
        frame.valid &= ordered;
        if let Some(control) = &mut self.state.control {
            control.measurement(
                frame.signal,
                frame.captured,
                frame.at,
                frame.valid,
                frame.fingerprint(),
            );
        }
        self.state.log.push(LogEntry {
            at: frame.at,
            kind: LogKind::Observation,
            id: frame.signal.0,
            value: frame.fingerprint(),
            valid: frame.valid,
        });
        let valid = frame.valid;
        if let Some(view) = &mut self.state.view {
            Rc::make_mut(&mut view.frames).push(frame.clone());
        }
        if ordered {
            self.state.bus.frames[index] = Some(frame);
        }
        valid
    }

    fn run_component(
        &mut self,
        phase: Phase,
        index: u32,
        prepared: Option<&process::Prepared>,
    ) -> Result<(), String> {
        let at = self.time();
        if phase == Phase::Controllers && self.state.replay.is_some() {
            self.inject(CommandSource::Controller(index));
            if self.state.replay.as_ref().is_some_and(|replay| {
                replay.expected.end().time == at
                    && replay.expected.end().failed_controller == Some(index)
            }) {
                return Err("воспроизведён записанный отказ контроллера".into());
            }
        } else {
            let c = &mut self.state.components[index as usize];
            let mut command_sequences = Vec::new();
            let command = match &mut self.state.control {
                Some(control) if phase == Phase::Actuators => control.prepare_object(
                    at,
                    &c.claimed,
                    &self.state.command,
                    &mut command_sequences,
                )?,
                _ => None,
            };
            let input = Inputs {
                truth: Truth::new(
                    at,
                    &self.sim,
                    &self.joints,
                    &self.state.plant,
                    &command_sequences,
                    prepared.map_or(&self.state.work.thermal, |pending| &pending.thermal),
                    self.state.work.last_tick(),
                ),
                observations: &self.state.bus,
                command: command.as_ref().unwrap_or(&self.state.command),
            };
            match c.task.run(input, &mut c.rng)? {
                Output::Observations(out) => {
                    for &components::PendingSample {
                        signal: id,
                        value,
                        captured,
                        available,
                        ..
                    } in &out.samples
                    {
                        if self.state.observations.spec(id).is_none() || !value.is_finite() {
                            return Err(format!("неизвестное или нечисловое показание {}", id.0));
                        }
                        if available.is_some_and(|t| t < at) || captured.is_some_and(|t| t > at) {
                            return Err(format!("показание {} доступно в прошлом", id.0));
                        }
                    }
                    for frame in &out.frames {
                        if self.state.observations.spec(frame.signal).is_none()
                            || !frame.is_consistent()
                            || frame.at < at
                            || frame.captured > frame.at
                        {
                            return Err(format!("неверный кадр сигнала {}", frame.signal.0));
                        }
                    }
                    if let Some(control) = &mut self.state.control {
                        for sample in &out.samples {
                            control.observation_source(index, sample.signal, None)?;
                        }
                        for frame in &out.frames {
                            control.observation_source(index, frame.signal, Some(frame.format))?;
                        }
                    }
                    for components::PendingSample {
                        signal,
                        value,
                        valid,
                        captured,
                        available,
                    } in out.samples
                    {
                        let captured = captured.unwrap_or(at);
                        let available = available.unwrap_or(at);
                        let when = self.state.bus.delivery_time(signal, available)?;
                        let sample = Sample {
                            signal,
                            value,
                            at: when,
                            valid,
                        };
                        if when == at {
                            if self.publish_sample(sample, captured) && sample.valid {
                                self.state.bus.confirm_delay(signal, available, at);
                            }
                        } else {
                            self.state
                                .bus
                                .delay(when, Delayed::Sample(sample, captured, available));
                        }
                    }
                    for mut frame in out.frames {
                        let available = frame.at;
                        let signal = frame.signal;
                        frame.at = self.state.bus.delivery_time(signal, available)?;
                        if frame.at == at {
                            if self.publish_frame(frame) {
                                self.state.bus.confirm_delay(signal, available, at);
                            }
                        } else {
                            self.state
                                .bus
                                .delay(frame.at, Delayed::Frame(frame, available));
                        }
                    }
                }
                Output::Commands(command) => {
                    self.check_targets(&command.targets)?;
                    self.apply_commands(&command.targets, CommandSource::Controller(index));
                }
                Output::Forces(out) => {
                    self.check_targets(&out.efforts)?;
                    for &(id, value) in &out.plant {
                        if self.state.truth.spec(SignalId(id.0)).is_none()
                            || !value.is_finite()
                            || self.state.work.owns_publication(id)
                        {
                            return Err(format!(
                                "неизвестное или нечисловое состояние модели {}",
                                id.0
                            ));
                        }
                    }
                    if let Some(control) = &mut self.state.control {
                        control.object_result(
                            at,
                            &self.state.components[index as usize].claimed,
                            &out.plant,
                            &out.accepted,
                        )?;
                    }
                    for (id, value) in out.plant {
                        self.state.plant[id.0 as usize] = Some(value);
                        self.state.log.push(LogEntry {
                            at,
                            kind: LogKind::Truth,
                            id: id.0,
                            value,
                            valid: true,
                        });
                    }
                    self.state.components[index as usize].forces = out.efforts;
                }
                Output::Truth(out) => {
                    for &(id, value) in &out.0 {
                        if self.state.truth.spec(SignalId(id.0)).is_none() || !value.is_finite() {
                            return Err(format!(
                                "неизвестное или нечисловое значение истины {}",
                                id.0
                            ));
                        }
                    }
                    for (id, value) in out.0 {
                        self.state.log.push(LogEntry {
                            at,
                            kind: LogKind::Truth,
                            id: id.0,
                            value,
                            valid: true,
                        });
                    }
                }
            }
        }
        let clock = &mut self.state.components[index as usize].clock;
        clock.advance();
        if let Some(next) = checked_due(clock)? {
            self.state.queue.push(Reverse((next, phase, index)));
        }
        Ok(())
    }

    /// Внешние команды и команды контроллеров берутся из записи. Модели мира,
    /// связь, датчики и оценщики продолжают вычисляться из текущего состояния.
    pub fn replay(&mut self, log: Log) -> Result<(), String> {
        self.ensure_configurable()?;
        if self.state.control.is_some() {
            return Err("для контура требуется типизированная запись управления".into());
        }
        if self.log().header() != log.header() {
            return Err("мир, зерно или таблицы сигналов отличаются от журнала".into());
        }
        let commands = log
            .entries()
            .iter()
            .copied()
            .filter(|e| matches!(e.kind, LogKind::Command(_)))
            .collect();
        self.state.replay = Some(Replay {
            expected: log,
            commands: Rc::new(commands),
            cursor: 0,
        });
        self.inject(CommandSource::BeforeStart);
        self.state.sealed = true;
        Ok(())
    }

    fn inject(&mut self, source: CommandSource) {
        let at = self.time();
        while let Some(replay) = &mut self.state.replay {
            let Some(entry) = replay
                .commands
                .get(replay.cursor)
                .copied()
                .filter(|e| e.at == at && e.kind == LogKind::Command(source))
            else {
                break;
            };
            replay.cursor += 1;
            self.apply_commands(&[(ChannelId(entry.id), entry.value)], source);
        }
    }

    /// Сверить время, статус, состояние физики и весь журнал побитово.
    pub fn verify_replay(&self) -> Result<(), String> {
        let replay = self
            .state
            .replay
            .as_ref()
            .ok_or("воспроизведение не включено")?;
        if replay.cursor != replay.commands.len() || !self.log().matches(&replay.expected) {
            return Err("воспроизведение разошлось с журналом или не дошло до его конца".into());
        }
        Ok(())
    }
}

fn sum_forces(
    state: &State,
    dofs: &[usize],
    process_efforts: &[(ChannelId, f64)],
    forces: &mut [f64],
) -> Result<(), String> {
    forces.fill(0.0);
    for &(id, value) in &state.command.targets {
        if !state.claimed.contains(&id)
            && !state
                .control
                .as_ref()
                .is_some_and(|control| control.latches_input(id))
        {
            forces[dofs[id.0 as usize]] += value;
        }
    }
    if let Some(control) = &state.control {
        control.forces(forces, dofs);
    } else {
        for &(id, value) in state.components.iter().flat_map(|c| &c.forces) {
            forces[dofs[id.0 as usize]] += value;
        }
    }
    for &(id, value) in process_efforts {
        forces[dofs[id.0 as usize]] += value;
    }
    if forces.iter().any(|value| !value.is_finite()) {
        return Err("сумма обобщённых сил вышла за диапазон f64".into());
    }
    Ok(())
}

fn check_spec(name: &str, unit: &str) -> Result<(), String> {
    if u32::try_from(name.len()).is_err() || u32::try_from(unit.len()).is_err() {
        return Err("имя или единица не помещаются в журнал".into());
    }
    Ok(())
}

fn physics_ok(sim: &Sim) -> bool {
    sim.numerically_ok()
        && sim.time().is_finite()
        && sim.qpos().iter().chain(sim.qvel()).all(|v| v.is_finite())
}

fn restore_processes(
    snapshots: &ProcessSnapshots,
) -> Result<BTreeMap<usize, sim_control::transport::ProcessTransport>, String> {
    snapshots
        .iter()
        .map(|(&id, snapshot)| {
            sim_control::transport::ProcessTransport::restore(snapshot)
                .map(|session| (id, session))
                .map_err(|e| e.to_string())
        })
        .collect()
}

fn checked_due(clock: &Clock) -> Result<Option<Time>, String> {
    time_at(clock.rate(), clock.index())
}

fn time_at(rate: Rate, index: u64) -> Result<Option<Time>, String> {
    const NS_PER_SECOND: u64 = 1_000_000_000;
    // as_hz служит только ограждением от переполнения. Момент события
    // всегда считает Clock целочисленно. Четыре epsilon покрывают округления:
    // индекс, числитель и знаменатель частоты, деления и масштаб наносекунд.
    let estimate = index as f64 / rate.as_hz() * NS_PER_SECOND as f64;
    let margin = 4.0 * f64::EPSILON;
    let limit = u64::MAX as f64;
    if estimate > limit * (1.0 + margin) {
        return Ok(None);
    }
    // Эти границы гарантируют, что промежуточное произведение Clock::due
    // помещается в u128 при любом скрытом знаменателе u64.
    if index > u64::MAX / NS_PER_SECOND || estimate >= limit * (1.0 - margin) {
        return Err("момент такта нельзя безопасно вычислить через Clock".into());
    }
    Ok(Some(rate.tick(index)))
}
