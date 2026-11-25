use std::collections::BTreeSet;

use sim_runtime::{Forces, Object, Runtime, Truth, TruthId};
use sim_signal::{ChannelId, Clock, Command, Rng, Time};
use sim_world::{CompiledDrive, CompiledJointKind, CompiledWorld, Simplification};

use crate::catalog::{DriveCatalog, DriveKind};
use crate::config::{Config, Features, Mode, resolve};
use crate::electric::{Bridge, bus_interval};
use crate::input::{BridgeCommand, PulseInputs, PulseReceiver};
use crate::mass::MassProbe;
use crate::model::{Model, Motion};
use crate::solve::DriveGroup;
use crate::step_size::DriveLimit;

/// Каналы привода. Значения публикуются в истину рантайма.
#[derive(Clone, Debug)]
pub struct DriveHandle {
    pub name: String,
    /// Команда в единицах режима каталога: доля ШИМ, А, рад/с или м/с,
    /// рад или м, либо микрошаг/с соответственно.
    pub command: ChannelId,
    /// При наличии подаётся 0 (питание снято) или 1 (питание подано).
    pub power: Option<ChannelId>,
    /// Вход вольт. Для привода со схемой обязателен с нулевого такта.
    pub bus_voltage: Option<ChannelId>,
    pub bridge_mode: Option<ChannelId>,
    pub pulse: Option<PulseInputs>,
    pub current_a: TruthId,
    pub voltage_v: TruthId,
    pub winding_c: Option<TruthId>,
    pub housing_c: Option<TruthId>,
    pub motor_rad: TruthId,
    pub motor_rad_s: TruthId,
    pub joint_torque_nm: Option<TruthId>,
    pub joint_force_n: Option<TruthId>,
    pub backlash_rad: Option<TruthId>,
    pub backlash_m: Option<TruthId>,
    /// Суммарно пропущенные микрошаги, в том числе при обратном ходе.
    pub lost_steps: Option<TruthId>,
    /// Токи двух фаз для шагового двигателя. Общий ток — норма вектора.
    pub phase_current_a: Option<[TruthId; 2]>,
    pub phase_voltage_v: Option<[TruthId; 2]>,
    pub current_limit_exceeded: TruthId,
    pub winding_overheated: Option<TruthId>,
    /// Последнее доставленное сообщение, в единицах команды. До первого
    /// сообщения значение отсутствует. При импульсном входе учитывайте
    /// command_present: рантайм хранит последнее опубликованное число.
    pub accepted_command: TruthId,
    pub motor_energy_residual_j: TruthId,
    pub world_energy_residual_j: TruthId,
    pub telemetry: Option<DriveTelemetry>,
}

/// Диагностика при подключении хотя бы одного из новых входов.
#[derive(Clone, Debug)]
pub struct DriveTelemetry {
    /// Напряжение шины, состояние моста и средняя мощность за завершённый
    /// шаг. Отрицательная мощность означает возврат энергии в шину.
    pub bus_voltage_v: TruthId,
    pub bridge_mode: TruthId,
    pub bus_power_w: TruthId,
    /// Знаковый заряд с начала прогона, положительный при потреблении.
    pub bus_charge_c: Option<TruthId>,
    /// Средний знаковый ток со стороны шины на предстоящем интервале физики.
    /// Зарегистрирован при подключении схемы, включая нулевой такт.
    pub next_bus_current_a: Option<TruthId>,
    /// Для жёсткой передачи: усилие от тока в конце последнего подшага
    /// при расчётной скорости этого подшага. Основной канал усилия хранит
    /// удерживаемое среднее, которое фактически получает мир.
    pub end_effort: Option<TruthId>,
    /// Удержанная знаковая скважность за завершённый шаг в режиме PWM.
    /// Для отключённого моста равна нулю.
    pub duty: Option<TruthId>,
    pub command_present: TruthId,
    pub resistance_ohm: TruthId,
    /// Накопленные с начала прогона энергии.
    pub bus_energy_j: TruthId,
    pub copper_loss_j: TruthId,
    pub friction_loss_j: TruthId,
    pub gear_loss_j: TruthId,
    pub damping_loss_j: TruthId,
    /// Остаточное поле, рассеянное при прежнем идеальном отключении.
    pub bridge_dump_j: TruthId,
    pub output_work_j: TruthId,
    /// Текущее магнитное, кинетическое и упругое накопление внутри привода.
    pub stored_energy_j: TruthId,
}

#[derive(Clone, Copy)]
enum Axis {
    Position(usize),
    AngularTravel(usize),
}

#[derive(Clone)]
pub(crate) struct DriveObject {
    pub joint: String,
    axis: Axis,
    angular_motion: Option<Motion>,
    pub model: Model,
    pub id: DriveHandle,
    pub limit: DriveLimit,
    pulse: Option<PulseReceiver>,
}

impl DriveObject {
    pub(crate) fn motion(&self, truth: &Truth<'_>) -> Result<Motion, String> {
        let (coordinates, speeds) = truth
            .joint(&self.joint)
            .ok_or_else(|| format!("нет оси «{}»", self.joint))?;
        let dt = truth.engine().timestep();
        let motion = match self.axis {
            Axis::Position(dof) => Motion {
                q: coordinates[dof],
                v: speeds[dof],
            },
            Axis::AngularTravel(dof) => {
                let v = speeds[dof];
                // Та же правая квадратура, что в Euler для скалярной оси.
                let q = self.angular_motion.map_or(0.0, |old| v.mul_add(dt, old.q));
                Motion { q, v }
            }
        };
        Ok(motion)
    }

    pub(crate) fn prepare_step(
        &mut self,
        time: Time,
        dt: f64,
        motion: Motion,
        command: &Command,
        sequence: Option<u64>,
    ) -> Result<(), String> {
        if matches!(self.axis, Axis::AngularTravel(_)) {
            self.angular_motion = Some(motion);
        }
        let input = |id| {
            command
                .targets
                .binary_search_by_key(&id, |&(channel, _)| channel)
                .ok()
                .map(|index| command.targets[index].1)
        };
        self.model.finish(time, motion, Some(dt))?;
        if let Some(channel) = self.id.power {
            self.model.power = match input(channel) {
                None | Some(0.0) => Some(false),
                Some(1.0) => Some(true),
                _ => return Err("на входе питания привода требуется 0 или 1".into()),
            };
        }
        self.model.bus_voltage = self.id.bus_voltage.and_then(input);
        if self.model.circuit && self.model.bus_voltage.is_none() {
            return Err("к такту привода не поступило напряжение из схемы".into());
        }
        if let Some(value) = self.model.bus_voltage {
            let [mut min, max] = self.model.config.bus_range.expect("границы шины проверены");
            if self.model.circuit {
                // Шина может просесть ниже рабочего диапазона вплоть до нуля.
                min = 0.0;
            }
            if !value.is_finite() || !(min..=max).contains(&value) {
                return Err(format!(
                    "напряжение шины {value} В вне диапазона [{min}, {max}] В"
                ));
            }
        }
        self.model.bridge_request = self
            .id
            .bridge_mode
            .map(|id| input(id).map_or(Ok(BridgeCommand::Coast), BridgeCommand::try_from))
            .transpose()?;
        let mut target = input(self.id.command);
        if let Some((ids, pulse)) = self.id.pulse.zip(self.pulse.as_mut()) {
            if target.is_some() {
                return Err("прямая команда положения несовместима с импульсным входом".into());
            }
            let (position, reset) = pulse.sample(
                time,
                input(ids.period_s),
                input(ids.width_s),
                input(ids.sequence),
            )?;
            if reset {
                self.model.control.clear_command();
            }
            target = position;
        }
        self.model.command(time, motion, target, sequence)
    }

    pub(crate) fn publish(&self, motion: Motion, out: &mut Forces) -> Result<(), String> {
        let h = &self.id;
        let m = &self.model;
        let stepped = h.phase_current_a.is_some();
        let value = |[x, y]: [f64; 2]| if stepped { x.hypot(y) } else { x };
        out.set(h.command, m.effort);
        for (id, v) in [
            (Some(h.current_a), value(m.current)),
            (Some(h.voltage_v), value(m.voltage)),
            (Some(h.motor_rad), m.motor.q),
            (Some(h.motor_rad_s), m.motor.v),
            (h.joint_torque_nm.or(h.joint_force_n), m.effort),
            (h.backlash_rad.or(h.backlash_m), m.backlash),
            (h.lost_steps, m.lost_steps),
            (Some(h.current_limit_exceeded), f64::from(m.limit_exceeded)),
            (
                Some(h.motor_energy_residual_j),
                m.work.residual(m.stored(motion)),
            ),
        ] {
            if let Some(id) = id {
                out.publish(id, v);
            }
        }
        out.publish(h.world_energy_residual_j, m.world_energy_residual(motion));
        if let Some(ids) = h.phase_current_a.zip(h.phase_voltage_v) {
            for (id, v) in ids
                .0
                .into_iter()
                .zip(m.current)
                .chain(ids.1.into_iter().zip(m.voltage))
            {
                out.publish(id, v);
            }
        }
        if let Some([winding, housing]) = m.temperatures {
            out.publish(h.winding_c.expect("зарегистрирован"), winding);
            out.publish(h.housing_c.expect("зарегистрирован"), housing);
            if let Some(id) = h.winding_overheated {
                let maximum = m
                    .config
                    .thermal
                    .and_then(|t| t.maximum)
                    .expect("порог задан");
                out.publish(id, f64::from(winding > maximum));
            }
        }
        if let Some(value) = m.control.accepted {
            out.publish(h.accepted_command, value);
        }
        if let Some((sequence, at, value)) = m.control.receipt {
            out.accept(h.command, sequence, at, value);
        }
        if let Some(t) = &h.telemetry {
            if let Some(interval) = m.interval {
                out.publish(t.bus_voltage_v, interval.supply);
                out.publish(t.bridge_mode, interval.bridge.mode().value());
                out.publish(t.bus_power_w, interval.bus_power_w);
                if let Some((id, value)) = t.end_effort.zip(interval.end_effort) {
                    out.publish(id, value);
                }
                if let Some(id) = t.duty {
                    let duty = match interval.bridge {
                        Bridge::Duty(value) => value,
                        _ => 0.0,
                    };
                    out.publish(id, duty);
                }
            }
            if let Some(id) = t.bus_charge_c {
                out.publish(id, m.work.bus_charge_c);
            }
            for (id, value) in [
                (t.command_present, f64::from(m.control.accepted.is_some())),
                (t.resistance_ohm, m.resistance()?),
                (t.bus_energy_j, m.work.electrical),
                (t.copper_loss_j, m.work.copper),
                (t.friction_loss_j, m.work.friction),
                (t.gear_loss_j, m.work.gear),
                (t.damping_loss_j, m.work.damping),
                (t.bridge_dump_j, m.work.bridge_dump),
                (t.output_work_j, m.work.output),
                (t.stored_energy_j, m.stored(motion)),
            ] {
                out.publish(id, value);
            }
        }
        Ok(())
    }
}

#[derive(Clone)]
struct CircuitDriveGroup {
    group: DriveGroup,
    probe: MassProbe,
    clock: Clock,
    held_charge: Vec<f64>,
}

impl Object for CircuitDriveGroup {
    fn update(
        &mut self,
        truth: &Truth<'_>,
        command: &Command,
        rng: &mut Rng,
        out: &mut Forces,
    ) -> Result<(), String> {
        let seconds = bus_interval(&mut self.clock, truth.time())?;
        self.group.update(truth, command, rng, out)?;

        // Повторяем только пробный шаг с уже найденными усилиями группы.
        // Нужны те же скорости, что в неявном расчёте ЭДС общего привода.
        let sim = truth.engine();
        let joints = sim.joints();
        let mut dofs = vec![0; self.group.inertia.len()];
        let mut efforts = vec![0.0; dofs.len()];
        for drive in &self.group.drives {
            let joint = joints
                .iter()
                .find(|joint| joint.name == drive.joint)
                .ok_or("нет оси электрической нагрузки")?;
            dofs[drive.limit.axis] = joint.dof_adr + drive.limit.dof;
            efforts[drive.limit.axis] += drive.model.effort;
        }
        self.probe.measure(sim, &dofs, &mut self.group.inertia)?;
        let free = self.probe.free_velocity(sim, &dofs)?;
        let end = self.probe.respond(sim, &dofs, &free, &efforts)?;
        for (drive, held) in self.group.drives.iter().zip(&mut self.held_charge) {
            let Some((_, current)) = drive.id.circuit_port() else {
                continue;
            };
            let dt = sim.timestep();
            if let Some(interval) = drive.model.interval {
                let actual = interval.bus_charge_c;
                let scale = held
                    .abs()
                    .max(actual.abs())
                    .max(dt * drive.model.config.electrical.limit);
                // Повторное вычисление заряда из принятого кадра допускает
                // округление в пробе MuJoCo, R–L и сумме фаз/подшагов.
                if (actual - *held).abs() > 256.0 * f64::EPSILON * scale {
                    return Err(format!(
                        "привод «{}»: заряд на шине за завершённый интервал {actual:e} Кл отличается от переданного в цепь {held:e} Кл",
                        drive.id.name
                    ));
                }
            }
            *held = drive.model.forecast_charge(dt, end[drive.limit.axis])?;
            out.publish(current, *held / seconds);
        }
        Ok(())
    }
}

#[derive(Clone, Copy)]
struct Inputs {
    command: ChannelId,
    power: Option<ChannelId>,
    bus_voltage: Option<ChannelId>,
    bridge_mode: Option<ChannelId>,
    pulse: Option<PulseInputs>,
}

impl Inputs {
    fn channels(self) -> impl Iterator<Item = ChannelId> {
        [
            Some(self.command),
            self.power,
            self.bus_voltage,
            self.bridge_mode,
        ]
        .into_iter()
        .flatten()
        .chain(
            self.pulse
                .into_iter()
                .flat_map(|p| [p.period_s, p.width_s, p.sequence]),
        )
    }

    fn extended(self) -> bool {
        self.bus_voltage.is_some() || self.bridge_mode.is_some() || self.pulse.is_some()
    }
}

struct Prepared<'a> {
    drive: &'a CompiledDrive,
    inputs: Inputs,
    axis: Axis,
    dof: usize,
    linear: bool,
    config: Config,
    circuit: bool,
    notes: Vec<String>,
}

/// Установить модели всех приводов до начала прогона. Runtime должен быть
/// загружен из переданного CompiledWorld. При ошибке регистрация откатывается.
/// При недостаточной частоте физики прогон завершается с ComponentFailure.
pub fn install(
    runtime: &mut Runtime,
    world: &CompiledWorld,
    catalog: &DriveCatalog,
) -> Result<Vec<DriveHandle>, String> {
    DriveHandle::install_with_circuit(runtime, world, catalog, &[])
}

impl DriveHandle {
    /// Канал напряжения и канал знакового тока для следующего интервала.
    /// Компоновщик должен владеть входом напряжения и направить ток между
    /// теми же клеммами схемы от плюса к минусу.
    pub fn circuit_port(&self) -> Option<(ChannelId, TruthId)> {
        self.bus_voltage
            .zip(self.telemetry.as_ref()?.next_bus_current_a)
    }

    /// Установить все общие приводы мира. У перечисленных приводов шина
    /// должна поступать из схемы через объявленный вход `<effort>-bus-v`.
    pub fn install_with_circuit(
        runtime: &mut Runtime,
        world: &CompiledWorld,
        catalog: &DriveCatalog,
        circuit_drives: &[&str],
    ) -> Result<Vec<Self>, String> {
        install_drives(runtime, world, catalog, circuit_drives)
    }
}

fn install_drives(
    runtime: &mut Runtime,
    world: &CompiledWorld,
    catalog: &DriveCatalog,
    circuit_drives: &[&str],
) -> Result<Vec<DriveHandle>, String> {
    if runtime.physics_rate() != world.physics
        || runtime.seed() != world.seed
        || runtime.log().header().commands.len() != world.efforts.len()
        || runtime
            .log()
            .header()
            .commands
            .iter()
            .zip(&world.efforts)
            .any(|(actual, declared)| actual.name != declared.name)
    {
        return Err("Runtime загружен с другой конфигурацией мира".into());
    }
    let mut circuit_names = circuit_drives.iter().copied().collect::<BTreeSet<_>>();
    if circuit_names.len() != circuit_drives.len() {
        return Err("привод повторно связан со схемой".into());
    }
    let mut names = BTreeSet::new();
    let mut commands = BTreeSet::new();
    let mut prepared = Vec::with_capacity(world.drives.len());
    for drive in &world.drives {
        let circuit = circuit_names.remove(drive.name.as_str());
        let item = prepare(runtime, world, catalog, drive, circuit)
            .map_err(|err| format!("привод «{}», позиция «{}»: {err}", drive.name, drive.part))?;
        if !names.insert(&drive.name) || item.inputs.channels().any(|id| !commands.insert(id)) {
            return Err(format!(
                "привод «{}»: повтор имени или канала команды",
                drive.name
            ));
        }
        prepared.push(item);
    }
    if let Some(name) = circuit_names.first() {
        return Err(format!("в мире нет общего привода «{name}» для схемы"));
    }
    if prepared.is_empty() {
        return Ok(Vec::new());
    }

    // На случай коллизии с компонентом, который был установлен другим модулем.
    // Упрощения добавляются только после регистрации всех объектов и каналов.
    let before = runtime.snapshot();
    let group_name = format!("{}/проверка шага", prepared[0].drive.name);
    let result = (|| {
        // Регистрируем сигналы и владение каналами одной транзакцией.
        let mut objects = Vec::with_capacity(prepared.len());
        let mut claimed = Vec::new();
        for item in &prepared {
            let object = register(runtime, item)
                .map_err(|err| format!("привод «{}»: {err}", item.drive.name))?;
            claimed.extend(item.inputs.channels());
            objects.push(object);
        }
        let handles = objects.iter().map(|object| object.id.clone()).collect();
        let group = DriveGroup::new(objects);
        if circuit_drives.is_empty() {
            runtime.register_object(&group_name, world.physics, &claimed, group)?;
        } else {
            runtime.register_object(
                &group_name,
                world.physics,
                &claimed,
                CircuitDriveGroup {
                    held_charge: vec![0.0; group.drives.len()],
                    group,
                    probe: MassProbe::default(),
                    clock: Clock::new(world.physics),
                },
            )?;
        }
        Ok::<_, String>(handles)
    })();
    let handles = match result {
        Ok(handles) => handles,
        Err(err) => {
            runtime
                .restore(&before)
                .map_err(|restore| format!("{err}; восстановление Runtime: {restore}"))?;
            return Err(err);
        }
    };
    for item in prepared {
        for text in item.notes {
            runtime.add_simplification(Simplification {
                subject: item.drive.name.clone(),
                text,
                blocks_proof: true,
            })?;
        }
    }
    Ok(handles)
}

fn prepare<'a>(
    runtime: &Runtime,
    world: &CompiledWorld,
    catalog: &DriveCatalog,
    drive: &'a CompiledDrive,
    circuit: bool,
) -> Result<Prepared<'a>, String> {
    let entry = catalog
        .get(&drive.part)
        .ok_or("неизвестная позиция каталога")?;
    let command = runtime
        .channel(&drive.effort)
        .ok_or_else(|| format!("нет входа «{}»", drive.effort))?;
    let channel = world
        .efforts
        .iter()
        .find(|ch| ch.name == drive.effort && ch.joint == drive.joint)
        .ok_or("вход усилия не относится к оси привода")?;
    let auxiliary = |suffix: &str| -> Result<Option<ChannelId>, String> {
        let name = format!("{}{suffix}", drive.effort);
        let Some(id) = runtime.channel(&name) else {
            return Ok(None);
        };
        if !world.efforts.iter().any(|input| {
            input.name == name && input.joint == channel.joint && input.dof == channel.dof
        }) {
            return Err(format!(
                "«{name}» должен быть назначен той же степени свободы"
            ));
        }
        Ok(Some(id))
    };
    let power = if entry.power_input {
        Some(
            auxiliary("-power")?
                .ok_or_else(|| format!("power_input: нет входа «{}-power»", drive.effort))?,
        )
    } else {
        None
    };
    let inputs =
        Inputs {
            command,
            power,
            bus_voltage: auxiliary("-bus-v")?,
            bridge_mode: auxiliary("-bridge")?,
            pulse: match (
                auxiliary("-pulse-period")?,
                auxiliary("-pulse-width")?,
                auxiliary("-pulse-seq")?,
            ) {
                (None, None, None) => None,
                (Some(period_s), Some(width_s), Some(sequence)) => Some(PulseInputs {
                    period_s,
                    width_s,
                    sequence,
                }),
                _ => return Err(
                    "импульсный вход требует одновременно -pulse-period, -pulse-width и -pulse-seq"
                        .into(),
                ),
            },
        };
    if circuit && inputs.bus_voltage.is_none() {
        return Err(format!("для схемы нужен вход «{}-bus-v»", drive.effort));
    }
    let joint = world
        .joints
        .iter()
        .find(|j| j.name == drive.joint)
        .ok_or("нет соединения привода")?;
    let dof = channel.dof;
    let (axis, linear) = match (joint.kind, dof) {
        (CompiledJointKind::Hinge, 0) => (Axis::Position(0), false),
        (CompiledJointKind::Slide, 0) => (Axis::Position(0), true),
        (CompiledJointKind::Free, 0..=2) => (Axis::Position(dof), true),
        (CompiledJointKind::Free, 3..=5) | (CompiledJointKind::Ball, 0..=2) => {
            (Axis::AngularTravel(dof), false)
        }
        _ => return Err("неверная степень свободы привода".into()),
    };
    if (entry.kind == DriveKind::Linear) != linear {
        return Err("тип привода и размерность степени свободы не совпадают".into());
    }
    let armature = runtime
        .joint_armature(&joint.name, dof)
        .ok_or("не найдена armature оси в MJCF")?;
    let features = Features {
        bus: inputs.bus_voltage.is_some(),
        bridge: inputs.bridge_mode.is_some(),
        pulse: inputs.pulse.is_some(),
    };
    let (config, mut notes) = resolve(entry, &drive.params, world.physics, armature, features)?;
    if circuit {
        notes.push("схема и привод обмениваются удержанными значениями на границах физики. Ниже нижнего рабочего напряжения или при снятии разрешения моделируется выбег через идеальные диоды. Порог выключения реального моста и выбросы при коммутации требуют сверки".into());
    }
    if features.pulse {
        notes.push("импульсная команда декодируется по двум точкам и принимается на сетке физики; нелинейность, разброс и реакция после потери сигнала требуют стендовой сверки".into());
    }
    if power.is_some() {
        notes.push(format!(
            "снятие питания действует на такте привода (период {:.9e} с); команда CSV на границе такта видна только на следующем. Для проверки перехода задайте физический момент снятия питания явно и сверяйте при сгущении сетки",
            1.0 / world.physics.as_hz()
        ));
    }
    if matches!(axis, Axis::AngularTravel(_)) {
        notes.push("для многокоординатного вращения используется интеграл локальной скорости; абсолютный угол и связи осей не доказаны".into());
    }
    Ok(Prepared {
        drive,
        inputs,
        axis,
        dof,
        linear,
        config,
        circuit,
        notes,
    })
}

fn register(runtime: &mut Runtime, item: &Prepared<'_>) -> Result<DriveObject, String> {
    let Prepared {
        drive,
        inputs,
        axis,
        linear,
        config,
        circuit,
        ..
    } = *item;
    let mut signal =
        |name: &str, unit: &str| runtime.register_truth(&format!("{}/{name}", drive.name), unit);
    let thermal = config.thermal.is_some();
    let stepper = matches!(config.mode, Mode::StepRate { .. });
    let coupling = config.mechanical.coupling.is_some();
    let optional = |enabled: bool,
                    id: &mut dyn FnMut(&str, &str) -> Result<TruthId, String>,
                    name,
                    unit| { enabled.then(|| id(name, unit)).transpose() };
    let mut handle = DriveHandle {
        name: drive.name.clone(),
        command: inputs.command,
        power: inputs.power,
        bus_voltage: inputs.bus_voltage,
        bridge_mode: inputs.bridge_mode,
        pulse: inputs.pulse,
        current_a: signal("current_a", "A")?,
        voltage_v: signal("voltage_v", "V")?,
        winding_c: optional(thermal, &mut signal, "winding_c", "degC")?,
        housing_c: optional(thermal, &mut signal, "housing_c", "degC")?,
        motor_rad: signal("motor_rad", "rad")?,
        motor_rad_s: signal("motor_rad_s", "rad/s")?,
        joint_torque_nm: optional(!linear, &mut signal, "joint_torque_nm", "N*m")?,
        joint_force_n: optional(linear, &mut signal, "joint_force_n", "N")?,
        backlash_rad: optional(coupling && !linear, &mut signal, "backlash_rad", "rad")?,
        backlash_m: optional(coupling && linear, &mut signal, "backlash_m", "m")?,
        lost_steps: optional(stepper, &mut signal, "lost_steps", "step")?,
        phase_current_a: if stepper {
            Some([
                signal("phase_a_current_a", "A")?,
                signal("phase_b_current_a", "A")?,
            ])
        } else {
            None
        },
        phase_voltage_v: if stepper {
            Some([
                signal("phase_a_voltage_v", "V")?,
                signal("phase_b_voltage_v", "V")?,
            ])
        } else {
            None
        },
        current_limit_exceeded: signal("current_limit_exceeded", "bool")?,
        winding_overheated: optional(
            config.thermal.is_some_and(|t| t.maximum.is_some()),
            &mut signal,
            "winding_overheated",
            "bool",
        )?,
        accepted_command: signal(
            "accepted_command",
            match config.mode {
                Mode::Pwm => "1",
                Mode::Current => "A",
                Mode::Velocity(_) if linear => "m/s",
                Mode::Velocity(_) => "rad/s",
                Mode::Position { .. } if linear => "m",
                Mode::Position { .. } => "rad",
                Mode::StepRate { .. } => "step/s",
            },
        )?,
        motor_energy_residual_j: signal("motor_energy_residual_j", "J")?,
        world_energy_residual_j: signal("world_energy_residual_j", "J")?,
        telemetry: None,
    };
    if inputs.extended() {
        handle.telemetry = Some(DriveTelemetry {
            bus_voltage_v: signal("bus_voltage_v", "V")?,
            bridge_mode: signal("bridge_mode", "1")?,
            bus_power_w: signal("bus_power_w", "W")?,
            bus_charge_c: optional(circuit, &mut signal, "bus_charge_c", "C")?,
            next_bus_current_a: optional(circuit, &mut signal, "next_bus_current_a", "A")?,
            end_effort: if coupling {
                None
            } else if linear {
                Some(signal("end_joint_force_n", "N")?)
            } else {
                Some(signal("end_joint_torque_nm", "N*m")?)
            },
            duty: optional(matches!(config.mode, Mode::Pwm), &mut signal, "duty", "1")?,
            command_present: signal("command_present", "bool")?,
            resistance_ohm: signal("resistance_ohm", "ohm")?,
            bus_energy_j: signal("bus_energy_j", "J")?,
            copper_loss_j: signal("copper_loss_j", "J")?,
            friction_loss_j: signal("friction_loss_j", "J")?,
            gear_loss_j: signal("gear_loss_j", "J")?,
            damping_loss_j: signal("damping_loss_j", "J")?,
            bridge_dump_j: signal("bridge_dump_j", "J")?,
            output_work_j: signal("output_work_j", "J")?,
            stored_energy_j: signal("stored_energy_j", "J")?,
        });
    }
    let mut model = Model::new(config);
    model.circuit = circuit;
    Ok(DriveObject {
        joint: drive.joint.clone(),
        axis,
        angular_motion: None,
        model,
        id: handle,
        limit: DriveLimit::new(item.dof, config)?,
        pulse: config.pulse.map(PulseReceiver::new),
    })
}
