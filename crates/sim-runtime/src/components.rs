use sim_mujoco::{JointInfo, JointKind, Sim};
use sim_process::{Analysis, work::immersion::ThermalBoundary};
use sim_signal::{ChannelId, Command, ImageFrame, Observation, Rng, SignalId, Time};
use std::collections::BTreeMap;

/// Истина доступна на время вызова модели мира, датчика или оценщика.
///
/// ```compile_fail,E0451
/// use sim_runtime::Truth;
/// let _ = Truth { time: sim_signal::Time::ZERO, sim: todo!(), joints: &[], plant: &[] };
/// ```
///
/// ```compile_fail,E0599
/// use sim_runtime::{Runtime, Truth};
/// fn получить_истину(runtime: &Runtime) -> &Truth<'_> {
///     runtime.truth()
/// }
/// ```
pub struct Truth<'a> {
    time: Time,
    sim: &'a Sim,
    joints: &'a [JointInfo],
    plant: &'a [Option<f64>],
    commands: &'a [(ChannelId, u64)],
    thermal: &'a BTreeMap<ChannelId, Analysis<ThermalBoundary>>,
    processes: Option<&'a crate::process::ProcessTick>,
}

impl<'a> Truth<'a> {
    pub(crate) fn new(
        time: Time,
        sim: &'a Sim,
        joints: &'a [JointInfo],
        plant: &'a [Option<f64>],
        commands: &'a [(ChannelId, u64)],
        thermal: &'a BTreeMap<ChannelId, Analysis<ThermalBoundary>>,
        processes: Option<&'a crate::process::ProcessTick>,
    ) -> Self {
        Self {
            time,
            sim,
            joints,
            plant,
            commands,
            thermal,
            processes,
        }
    }

    pub fn time(&self) -> Time {
        self.time
    }

    /// Сквозной номер команды, переданной в модель на этом вызове.
    pub fn command_sequence(&self, channel: ChannelId) -> Option<u64> {
        self.commands
            .iter()
            .find(|&&(id, _)| id == channel)
            .map(|&(_, sequence)| sequence)
    }

    /// Координаты в порядке MJCF, в единицах соответствующих соединений.
    pub fn qpos(&self) -> &'a [f64] {
        self.sim.qpos()
    }

    /// Скорости в порядке степеней свободы MJCF.
    pub fn qvel(&self) -> &'a [f64] {
        self.sim.qvel()
    }

    /// Движок целиком, только для чтения: позы тел и геометрий, контакты, лучи.
    /// Для датчиков и оценщиков кинематика уже посчитана для текущего момента.
    pub fn engine(&self) -> &'a Sim {
        self.sim
    }

    /// Последнее значение, которое модель мира опубликовала через [`Forces::publish`].
    pub fn plant(&self, id: TruthId) -> Option<f64> {
        self.plant.get(id.0 as usize).copied().flatten()
    }

    /// Граница теплообмена для текущего интервала, если она подключена к приводу.
    pub fn thermal_boundary(&self, drive: ChannelId) -> Option<&Analysis<ThermalBoundary>> {
        self.thermal.get(&drive)
    }

    pub fn process_state(&self, name: &str) -> Option<&str> {
        self.processes?
            .outputs
            .values()
            .find_map(|out| out.states.get(name))
            .map(String::as_str)
    }

    /// Значение процесса с предыдущей зафиксированной границы физики.
    pub fn process_value(&self, name: &str) -> Option<f64> {
        self.processes?
            .outputs
            .values()
            .flat_map(|out| &out.measurements)
            .find_map(|sample| match &sample.result {
                Analysis::Analyzed { value, .. } if sample.signal.name == name => Some(*value),
                _ => None,
            })
    }

    pub fn attachment_state(
        &self,
        name: &str,
    ) -> Option<sim_process::attachments::AttachmentState> {
        self.processes?
            .outputs
            .values()
            .flat_map(|out| out.attachment_records())
            .find(|record| record.name == name)
            .map(|record| record.state)
    }

    /// Координаты и скорости именованного соединения в порядке MuJoCo.
    /// У свободного соединения это 3 координаты, кватернион wxyz и 6 скоростей.
    pub fn joint(&self, name: &str) -> Option<(&[f64], &[f64])> {
        let joint = self
            .joints
            .iter()
            .find(|joint| joint.name == name && !joint.name.is_empty())?;
        let (coordinates, velocities) = match joint.kind {
            JointKind::Hinge | JointKind::Slide => (1, 1),
            JointKind::Ball => (4, 3),
            JointKind::Free => (7, 6),
        };
        Some((
            &self.qpos()[joint.qpos_adr..joint.qpos_adr + coordinates],
            &self.qvel()[joint.dof_adr..joint.dof_adr + velocities],
        ))
    }
}

pub(crate) struct PendingSample {
    pub signal: SignalId,
    pub value: f64,
    pub valid: bool,
    pub captured: Option<Time>,
    pub available: Option<Time>,
}

#[derive(Default)]
pub struct ObservationOut {
    pub(crate) samples: Vec<PendingSample>,
    pub(crate) frames: Vec<ImageFrame>,
}

impl ObservationOut {
    /// Показание доступно сразу. Для отказа датчика передать valid=false.
    pub fn write(&mut self, signal: SignalId, value: f64, valid: bool) {
        self.samples.push(PendingSample {
            signal,
            value,
            valid,
            captured: None,
            available: None,
        });
    }

    /// Показание станет доступно контроллеру в момент `available`, не раньше текущего.
    pub fn write_at(&mut self, signal: SignalId, value: f64, valid: bool, available: Time) {
        self.samples.push(PendingSample {
            signal,
            value,
            valid,
            captured: None,
            available: Some(available),
        });
    }

    /// Скаляр измерен в исходный момент, доставлен позднее. Оба указаны по шкале прогона.
    pub fn write_measured(
        &mut self,
        signal: SignalId,
        value: f64,
        valid: bool,
        captured: Time,
        available: Time,
    ) {
        self.samples.push(PendingSample {
            signal,
            value,
            valid,
            captured: Some(captured),
            available: Some(available),
        });
    }

    /// Кадр станет доступен в момент `frame.at`.
    pub fn write_frame(&mut self, frame: ImageFrame) {
        self.frames.push(frame);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct TruthId(pub(crate) u32);

#[derive(Default)]
pub struct TruthOut(pub(crate) Vec<(TruthId, f64)>);

impl TruthOut {
    pub fn write(&mut self, signal: TruthId, value: f64) {
        self.0.push((signal, value));
    }
}

#[derive(Default)]
pub struct Forces {
    pub(crate) efforts: Vec<(ChannelId, f64)>,
    pub(crate) plant: Vec<(TruthId, f64)>,
    pub(crate) accepted: Vec<(ChannelId, u64, Time, f64)>,
}

impl Forces {
    /// Подтверждение принятия команды из очереди модели.
    pub fn accept(&mut self, channel: ChannelId, sequence: u64, at: Time, value: f64) {
        self.accepted.push((channel, sequence, at, value));
    }

    /// Обобщённая сила в СИ. Держится до следующего вызова этой модели.
    /// За такт модель задаёт каждый канал максимум один раз.
    pub fn set(&mut self, channel: ChannelId, value: f64) {
        self.efforts.push((channel, value));
    }

    /// Внутреннее состояние модели (ток, температура): его видят датчики и оценщики
    /// через [`Truth::plant`]. Контроллеру оно недоступно.
    pub fn publish(&mut self, signal: TruthId, value: f64) {
        self.plant.push((signal, value));
    }
}

/// Клоны зарегистрированных моделей должны владеть независимым изменяемым состоянием.
pub trait Sensor {
    fn sample(
        &mut self,
        truth: &Truth<'_>,
        rng: &mut Rng,
        out: &mut ObservationOut,
    ) -> Result<(), String>;
}

pub trait Link {
    fn transfer(
        &mut self,
        observation: &Observation,
        rng: &mut Rng,
        out: &mut ObservationOut,
    ) -> Result<(), String>;
}

pub trait Object {
    fn update(
        &mut self,
        truth: &Truth<'_>,
        command: &Command,
        rng: &mut Rng,
        out: &mut Forces,
    ) -> Result<(), String>;
}

pub trait Evaluator {
    fn evaluate(
        &mut self,
        truth: &Truth<'_>,
        rng: &mut Rng,
        out: &mut TruthOut,
    ) -> Result<(), String>;
}

#[derive(Clone)]
pub(crate) struct RecordState {
    pub qpos: Vec<TruthId>,
    pub qvel: Vec<TruthId>,
}

impl Evaluator for RecordState {
    fn evaluate(
        &mut self,
        truth: &Truth<'_>,
        _: &mut Rng,
        out: &mut TruthOut,
    ) -> Result<(), String> {
        for (&id, &value) in self
            .qpos
            .iter()
            .zip(truth.qpos())
            .chain(self.qvel.iter().zip(truth.qvel()))
        {
            out.write(id, value);
        }
        Ok(())
    }
}
