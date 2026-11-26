use std::collections::{BTreeMap, BTreeSet};
use std::ffi::OsString;
use std::path::{Path, PathBuf};
use std::rc::Rc;

use sim_boards::{BoardCatalog, BoardSim, pin_spec};
use sim_control::transport::{ProcessSpec, ProcessTransport, SessionOptions};
use sim_control::{ControlContract, Controller, InputChannel, InputKind, OutputChannel, RequestId};
use sim_process::Analysis;
use sim_process::schematic::mcu_energy::{McuEnergyReport, McuPowerProfile, assess_mcu_energy};
use sim_process::schematic::sleep_drain::{
    SleepDrainInput, SleepDrainProfile, SleepDrainRequirement, SleepDrainState,
};
use sim_process::schematic::{Mode, Scope, Section};
use sim_radio::timed::TimedChannel;
use sim_signal::{ChannelId, Clock, Rng, SignalId, Time, fnv1a};
use sim_world::{Provenance, Simplification};
use std::collections::VecDeque;

use crate::{Runtime, TruthId};

use super::board::{Board, Bus, Route, ScheduledBoardAction, Wire};
use super::bus::{BusModel, Profile};
use super::external_board::{self, Boot, Report, UartProbe};
use super::journal::{ControlPath, FaultMechanism, FaultTarget, RunIdentity, RunOrigin};
use super::schema::{self, TargetKind};
use super::{
    ControlConfig, ControlRecording, Engine, FaultAction, FaultEdge, LocalController, Node, Output,
    Schedule, Stimulus, Target, Transmission,
};

/// Подтверждённые точки модели, установленные сборщиком из результата install привода.
#[derive(Clone, Copy, Debug)]
pub enum ModelPort {
    Drive {
        channel: ChannelId,
        accepted: TruthId,
    },
    Power {
        channel: ChannelId,
    },
    DrivePower {
        channel: ChannelId,
        drive: ChannelId,
    },
}

pub struct RunSpec<'a> {
    pub world: &'a [u8],
    pub catalog: &'a [u8],
    pub task: &'a [u8],
    pub criteria: &'a [u8],
    pub mode: &'a str,
    pub horizon: Time,
    pub model_ports: &'a [ModelPort],
}

pub trait ControlAssets {
    fn read(&self, name: &str) -> Result<Vec<u8>, String>;

    /// Путь запускается напрямую через ProcessTransport. Он должен соответствовать
    /// байтам read и оставаться неизменным до завершения сеансов и ветвей.
    fn executable(&self, name: &str) -> Result<PathBuf, String>;

    /// Супервизор получает PID до блокирующего рукопожатия.
    fn worker_started(&self, _name: &str, _pid: u32) {}
}

pub struct FileAssets<'a> {
    pub base: &'a Path,
}

impl ControlAssets for FileAssets<'_> {
    fn read(&self, name: &str) -> Result<Vec<u8>, String> {
        std::fs::read(self.base.join(name)).map_err(|e| format!("ресурс «{name}»: {e}"))
    }

    fn executable(&self, name: &str) -> Result<PathBuf, String> {
        self.base
            .join(name)
            .canonicalize()
            .map_err(|e| format!("программа «{name}»: {e}"))
    }
}

impl ControlConfig {
    /// Отдельная диагностика UART0, вне шкалы времени симуляции. Использует
    /// те же проверки профиля и образов, что и установщик контура.
    pub fn probe_external_board(
        &self,
        assets: &FileAssets<'_>,
        name: &str,
        probe: UartProbe<'_>,
    ) -> Result<(Report, Simplification), String> {
        let mut matches = self
            .get("external_board")
            .map(schema::external_board)
            .collect::<Result<Vec<_>, _>>()?
            .into_iter()
            .filter(|item| item.name == name);
        let item = matches
            .next()
            .ok_or_else(|| format!("нет внешней платы «{name}»"))?;
        if matches.next().is_some() {
            return Err(format!("повтор внешней платы «{name}»"));
        }
        let profile = load_external(&item, &mut |path| assets.read(path))?;
        if !profile.ports.iter().any(|port| port == probe.port) {
            return Err(format!(
                "порт «{}» отсутствует в профиле платы «{name}»",
                probe.port
            ));
        }
        let flash = item.image.map(|path| assets.base.join(path));
        let config = external_board::Config {
            qemu: assets.base.join(profile.qemu),
            boot: if profile.flash_boot {
                Boot::Flash(flash.ok_or("для загрузки QEMU нужен образ flash")?)
            } else {
                Boot::UartDownload { flash }
            },
            efuse: item.efuse.map(|path| assets.base.join(path)),
            psram: profile.psram,
            clock: profile.clock,
            seed: profile.seed,
            limits: profile.limits,
        };
        let report = external_board::probe_esp32s3(&config, probe)?;
        let Analysis::NotAnalyzed { reason } = report.repeatability() else {
            return Err(
                "адаптер QEMU вернул результат без сверенного восстановления контура".into(),
            );
        };
        Ok((report, open_board_proof(name, &reason)))
    }

    /// Оценить заряд МК для явно заданного графика на том же интервале,
    /// который передаётся установщику. Профиль не даёт огибающую тока батареи.
    pub fn mcu_energy(
        &self,
        assets: &impl ControlAssets,
        name: &str,
        horizon: Time,
    ) -> Result<McuEnergyReport, String> {
        let mut matches = self
            .get("board")
            .map(schema::board)
            .collect::<Result<Vec<_>, _>>()?
            .into_iter()
            .filter(|item| item.name == name);
        let item = matches
            .next()
            .ok_or_else(|| format!("нет платы «{name}»"))?;
        if matches.next().is_some() {
            return Err(format!("повтор платы «{name}»"));
        }
        let mut read = |path: &str| assets.read(path);
        let catalog = load_board_catalog(item.catalog_asset.as_deref(), &mut read)?;
        let board = catalog.create(&item.catalog).map_err(|e| e.to_string())?;
        load_mcu_energy(&item, board.board().key(), horizon, &mut read)
    }
}

/// Ввод пользовательских реализаций Controller через тот же контракт.
/// Clone копирует всё состояние прогона. Эта форма маркирует происхождение
/// отдельно, поскольку двоичный код встраиваемого объекта не связан с файлом графа.
#[derive(Clone, Default)]
pub struct ControllerBindings(BTreeMap<String, Box<dyn LocalController>>);

impl ControllerBindings {
    pub fn bind(
        &mut self,
        name: &str,
        controller: impl Controller + Clone + 'static,
    ) -> Result<(), String> {
        if name.trim().is_empty() || self.0.contains_key(name) {
            return Err("пустое или дублированное имя контроллера".into());
        }
        self.0.insert(name.into(), Box::new(controller));
        Ok(())
    }
}

pub fn install_with_controllers(
    runtime: &mut Runtime,
    config: &ControlConfig,
    assets: &impl ControlAssets,
    spec: RunSpec<'_>,
    bindings: ControllerBindings,
) -> Result<(), String> {
    assemble(runtime, config, assets, spec, None, bindings)
}

pub fn install(
    runtime: &mut Runtime,
    config: &ControlConfig,
    assets: &impl ControlAssets,
    spec: RunSpec<'_>,
) -> Result<(), String> {
    assemble(
        runtime,
        config,
        assets,
        spec,
        None,
        ControllerBindings::default(),
    )
}

pub fn install_replay(
    runtime: &mut Runtime,
    config: &ControlConfig,
    assets: &impl ControlAssets,
    spec: RunSpec<'_>,
    recording: ControlRecording,
) -> Result<(), String> {
    assemble(
        runtime,
        config,
        assets,
        spec,
        Some(recording),
        ControllerBindings::default(),
    )
}

fn assemble(
    runtime: &mut Runtime,
    config: &ControlConfig,
    source: &impl ControlAssets,
    spec: RunSpec<'_>,
    recording: Option<ControlRecording>,
    bindings: ControllerBindings,
) -> Result<(), String> {
    runtime.ensure_configurable()?;
    if runtime.state.control.is_some()
        || runtime.state.replay.is_some()
        || runtime
            .state
            .components
            .iter()
            .any(|c| c.phase == sim_signal::Phase::Controllers)
    {
        return Err("конфликт писателей управления".into());
    }
    if spec.mode.trim().is_empty()
        || spec.horizon == Time::ZERO
        || spec.world.is_empty()
        || spec.catalog.is_empty()
        || spec.task.is_empty()
        || spec.criteria.is_empty()
    {
        return Err("неполные входные данные прогона".into());
    }
    let saved = runtime.state.clone();
    let result = build(runtime, config, source, spec, recording, bindings);
    match result {
        Ok((engine, processes, limitations)) => {
            runtime.state.control = Some(engine);
            runtime.processes = processes;
            for item in limitations {
                runtime.add_simplification(item)?;
            }
            Ok(())
        }
        Err(error) => {
            runtime.state = saved;
            Err(error)
        }
    }
}

type Processes = BTreeMap<usize, ProcessTransport>;

fn build(
    runtime: &mut Runtime,
    config: &ControlConfig,
    source: &impl ControlAssets,
    spec: RunSpec<'_>,
    recording: Option<ControlRecording>,
    mut bindings: ControllerBindings,
) -> Result<(Engine, Processes, Vec<Simplification>), String> {
    let has_programmatic_controller = !bindings.0.is_empty();
    let mut executables = BTreeMap::new();
    let mut by_path = BTreeMap::new();
    let mut read = |name: &str| -> Result<Vec<u8>, String> {
        let bytes = source.read(name)?;
        let hash = fnv1a(bytes.iter().copied());
        if by_path
            .insert(name.to_owned(), hash)
            .is_some_and(|old| old != hash)
        {
            return Err(format!("ресурс «{name}» изменился во время сборки"));
        }
        Ok(bytes)
    };
    let mut board_catalog = None;
    let mut boards = Vec::new();
    let mut names = BTreeSet::new();
    let mut limitations = Vec::new();
    let mut external_boards = BTreeSet::new();
    for item in config.get("external_board").map(schema::external_board) {
        let item = item?;
        if !names.insert(item.name.clone()) {
            return Err(format!("повтор платы «{}»", item.name));
        }
        load_external(&item, &mut read)?;
        if recording.is_some() {
            return Err(format!(
                "внешняя плата «{}» не поддерживает повтор опыта без снимка QEMU",
                item.name
            ));
        }
        limitations.push(open_board_proof(
            &item.name,
            "внешняя плата доступна для диагностики UART0; нет подтверждённого снимка QEMU и привязки UART к виртуальному времени",
        ));
        external_boards.insert(item.name);
    }
    let mut board_specs = config
        .get("board")
        .map(schema::board)
        .collect::<Result<Vec<_>, _>>()?;
    board_specs.sort_by(|a, b| a.name.cmp(&b.name));
    for item in board_specs {
        if !names.insert(item.name.clone()) {
            return Err(format!("повтор платы «{}»", item.name));
        }
        let local_catalog;
        let catalog = if let Some(asset) = &item.catalog_asset {
            local_catalog = load_board_catalog(Some(asset), &mut read)?;
            &local_catalog
        } else {
            if board_catalog.is_none() {
                board_catalog = Some(load_board_catalog(None, &mut read)?);
            }
            board_catalog.as_ref().expect("каталог загружен")
        };
        let mut board = catalog.create(&item.catalog).map_err(|e| e.to_string())?;
        let image = read(&item.image)?;
        let program_hash = fnv1a(image.iter().copied());
        load_board_image(&item, board.as_mut(), &image, &mut read)?;
        if let Some(name) = &item.config {
            board
                .write_config(&read(name)?)
                .map_err(|e| e.to_string())?;
        }
        if item.power_profile.is_some() {
            let report = load_mcu_energy(&item, board.board().key(), spec.horizon, &mut read)?;
            let chip = match report.chip_charge {
                Analysis::Analyzed { value, scope } => format!(
                    "{scope}: {} Кл за {} нс; удержан измеренный пик: {}",
                    value.charge_c,
                    value.window.end.0,
                    if value.includes_peak_held {
                        "да"
                    } else {
                        "нет"
                    },
                ),
                Analysis::NotAnalyzed { reason } => reason,
            };
            limitations.push(open_board_proof(
                &item.name,
                &format!("{chip}. Автономность: {}", report.sleep_drain.reason),
            ));
        }
        if item.rate.as_hz() > board.capabilities().cpu_hz as f64 {
            return Err(format!("частота обмена выше частоты платы «{}»", item.name));
        }
        limitations.push(Simplification {
            subject: format!("board:{}", item.name),
            text: "Профиль CPU/MMIO и электрические параметры входов имеют область применимости из sim-boards/SUPPORT.md".into(),
            blocks_proof: board.capabilities().blocks_hardware_proof,
        });
        if item.end_sample {
            // Частоты платы и физики поступают из целых полей конфигурации.
            // При вложенной сетке граница каждого интервала платы совпадает
            // с границей физики, а выход удерживается на всех его подшагах.
            let subdivisions = runtime.physics_rate().as_hz() / item.rate.as_hz();
            if item.rate != runtime.physics_rate()
                && (subdivisions < 1.0 || subdivisions.fract() != 0.0)
            {
                return Err("выборка с конца шага требует вложенной сетки физики".into());
            }
            limitations.push(Simplification {
                subject: format!("board:{}", item.name),
                text: "Выход со шкалы конца шага применяется ко всему интервалу платы. Режим сверки численного эксперимента.".into(),
                blocks_proof: true,
            });
        }
        boards.push(Board {
            name: item.name,
            program_hash,
            clock: Clock::new(item.rate),
            sim: board,
            wires: Vec::new(),
            buses: Vec::new(),
            outputs: Vec::new(),
            pending: Vec::new(),
            actions: Vec::new(),
            input_at: Time::ZERO,
            end_sample: item.end_sample,
            boot: None,
            epoch: Time::ZERO,
            epoch_cycle: 0,
            electrical: None,
            electrical_restart: None,
            driven_gpio: BTreeMap::new(),
        });
    }
    let board_id = |boards: &[Board], name: &str| {
        if external_boards.contains(name) {
            return Err(format!(
                "внешняя плата «{name}»: подключение к контуру требует порта с виртуальными метками времени и снимка QEMU"
            ));
        }
        boards
            .iter()
            .position(|b| b.name == name)
            .ok_or_else(|| format!("неизвестная плата «{name}»"))
    };
    let mut node_specs = config
        .get("controller")
        .map(schema::controller)
        .collect::<Result<Vec<_>, _>>()?;
    node_specs.sort_by(|a, b| a.name.cmp(&b.name));
    for node in &node_specs {
        if !names.insert(node.name.clone()) {
            return Err(format!("конфликт имени «{}»", node.name));
        }
    }
    let mut out_specs = config
        .get("output")
        .chain(config.get("driver"))
        .map(schema::output)
        .collect::<Result<Vec<_>, _>>()?;
    out_specs.sort_by(|a, b| a.name.cmp(&b.name));
    let mut outputs = Vec::new();
    let mut paths = Vec::new();
    let mut targets = BTreeMap::new();
    let mut destinations = BTreeSet::new();
    let mut channels = BTreeMap::new();
    let base = runtime.log().header().commands.len();
    let mut model_ports = BTreeMap::new();
    let mut power_links = Vec::new();
    for port in spec.model_ports {
        let (channel, ack) = match *port {
            ModelPort::Drive { channel, accepted } => (channel, Some(accepted)),
            ModelPort::Power { channel } => (channel, None),
            ModelPort::DrivePower { channel, drive } => {
                power_links.push((drive, channel));
                (channel, None)
            }
        };
        if model_ports.insert(channel, ack).is_some() || !runtime.state.claimed.contains(&channel) {
            return Err("повтор или неподключённый порт модели".into());
        }
    }
    for &(drive, _) in &power_links {
        if !model_ports.get(&drive).is_some_and(Option::is_some) {
            return Err("порт питания связан с неизвестным приводом".into());
        }
    }
    for item in &out_specs {
        if !destinations.insert((item.target_kind == TargetKind::Goal, item.target.clone())) {
            return Err(format!("конфликт писателей на «{}»", item.target));
        }
        let wire = format!("wire:{}", item.name);
        let physical = match item.target_kind {
            TargetKind::Goal => format!("channel:{}", item.target),
            TargetKind::Power => format!("power:{}", item.target),
            _ => format!("actuator:{}", item.target),
        };
        let mut allowed = vec![FaultMechanism::Drop];
        if item.target_kind == TargetKind::Power {
            allowed.push(FaultMechanism::PowerOff);
        } else {
            allowed.push(FaultMechanism::Stuck);
        }
        let faults = vec![
            fault_target(&mut targets, wire, allowed.clone())?,
            fault_target(&mut targets, physical, allowed)?,
        ];
        let id = ChannelId(
            u32::try_from(
                base.checked_add(outputs.len())
                    .ok_or("слишком много портов")?,
            )
            .map_err(|_| "слишком много портов")?,
        );
        let target = match item.target_kind {
            TargetKind::Goal => {
                Target::Goal(runtime.register_observation(&item.target, &item.unit)?)
            }
            _ => {
                let channel = runtime
                    .channel(&item.target)
                    .ok_or_else(|| format!("нет входа «{}»", item.target))?;
                let (unit, ack) = match item.target_kind {
                    TargetKind::Drive => {
                        let ack = runtime
                            .truth_id(item.ack.as_ref().expect("проверено схемой"))
                            .ok_or("нет канала подтверждения привода")?;
                        if model_ports.get(&channel) != Some(&Some(ack)) {
                            return Err(format!(
                                "подтверждение не связано с приводом «{}»",
                                item.target
                            ));
                        }
                        (
                            runtime.log().header().truth[ack.0 as usize].unit.as_str(),
                            Some(ack),
                        )
                    }
                    TargetKind::Power if model_ports.get(&channel) == Some(&None) => ("bool", None),
                    TargetKind::Effort if !runtime.state.claimed.contains(&channel) => (
                        runtime.log().header().commands[channel.0 as usize]
                            .unit
                            .as_str(),
                        None,
                    ),
                    _ => return Err(format!("неверный профиль получателя «{}»", item.target)),
                };
                require_unit(&item.target, unit, &item.unit)?;
                channels.insert(channel, outputs.len());
                Target::Channel(channel, ack)
            }
        };
        if let Some(driver) = &item.driver {
            let board = board_id(&boards, &item.owner)?;
            check_pin(
                boards[board].sim.as_ref(),
                driver.pin,
                driver.quantity != "logic",
            )?;
            for pin in [driver.direction_pin, driver.enable_pin]
                .into_iter()
                .flatten()
            {
                check_pin(boards[board].sim.as_ref(), pin, false)?;
            }
            boards[board].outputs.push(outputs.len());
        } else if !node_specs.iter().any(|node| node.name == item.owner) {
            return Err(format!("неизвестный контроллер «{}»", item.owner));
        }
        paths.push(ControlPath {
            name: item.name.clone(),
            producer: item.owner.clone(),
            target: item.target.clone(),
            producer_index: 0,
            program_hash: 0,
            board_output_pin: item.driver.as_ref().map(|driver| u32::from(driver.pin)),
            unit: item.unit.clone(),
            channel: match target {
                Target::Goal(_) => None,
                Target::Channel(channel, _) => Some(channel),
            },
            accepted_truth: match target {
                Target::Channel(_, ack) => ack,
                Target::Goal(_) => None,
            },
            feedback: Vec::new(),
            fault_targets: faults,
        });
        outputs.push(Output {
            id,
            name: item.name.clone(),
            target,
            target_name: item.target.clone(),
            kind: item.target_kind,
            max_age: item.max_age_ns,
            fallback: item.fallback,
            hold_last: item.hold_last,
            last_value: None,
            driver: item.driver.clone(),
            latest: None,
            fallback_value: None,
            fault_value: None,
            presented: None,
            waiting_for_acceptance: VecDeque::new(),
            applied: None,
            ack_seen: None,
        });
    }

    let mut wires = config
        .get("board_input")
        .map(schema::board_input)
        .collect::<Result<Vec<_>, _>>()?;
    wires.sort_by(|a, b| a.name.cmp(&b.name));
    let mut assigned_pins = BTreeSet::new();
    for wire in wires {
        let id = board_id(&boards, &wire.board)?;
        let board = &mut boards[id];
        match wire.kind.as_str() {
            "adc" if wire.port < board.sim.capabilities().adc_channels => {
                let max = sim_boards::board_spec(board.sim.board()).adc_max_v;
                if wire.fallback.is_some_and(|v| !(0.0..=max).contains(&v)) {
                    return Err(format!("напряжение обрыва «{}» вне диапазона", wire.name));
                }
            }
            "gpio" => check_pin(board.sim.as_ref(), wire.port, false)?,
            _ => return Err(format!("неподдержанный вход «{}»", wire.name)),
        }
        if !assigned_pins.insert((id, wire.kind.clone(), wire.port)) {
            return Err("конфликт писателей входа платы".into());
        }
        let signal = observation(runtime, &wire.signal, &wire.unit)?;
        fault_target(
            &mut targets,
            format!("wire:{}", wire.name),
            vec![FaultMechanism::Drop, FaultMechanism::Stuck],
        )?;
        sensor_target(&mut targets, &wire.signal, true)?;
        board.wires.push(Wire {
            spec: Rc::new(wire),
            signal,
            last: None,
        });
    }

    let mut board_actions = Vec::new();
    let mut action_ids = BTreeSet::new();
    let mut action_pins = BTreeSet::new();
    for action in config.get("board_action").map(schema::board_action) {
        let (id, at, board, action) = action?;
        let board = board_id(&boards, &board)?;
        if at > spec.horizon || !action_ids.insert(id.clone()) {
            return Err(format!("повтор или время вне режима у действия «{id}»"));
        }
        let (action, config) = match action {
            schema::BoardActionSpec::Gpio { pin, high } => {
                check_pin(boards[board].sim.as_ref(), pin, false)?;
                if assigned_pins.contains(&(board, "gpio".into(), pin))
                    || !action_pins.insert((board, at, Some(pin)))
                {
                    return Err(format!("конфликт писателей у действия «{id}»"));
                }
                (super::BoardAction::Gpio { pin, high }, None)
            }
            schema::BoardActionSpec::Reset {
                config,
                restart_clock,
            } => {
                if !action_pins.insert((board, at, None)) {
                    return Err(format!("двойной сброс у действия «{id}»"));
                }
                let config = config.as_deref().map(&mut read).transpose()?;
                if config.as_ref().is_some_and(|bytes| {
                    bytes.is_empty() || bytes.len() > boards[board].sim.capabilities().config_bytes
                }) {
                    return Err(format!("блок конфигурации не подходит действию «{id}»"));
                }
                let config_hash = config.as_ref().map(|bytes| fnv1a(bytes.iter().copied()));
                let action = if restart_clock {
                    boards[board].boot = Some(boards[board].sim.clone());
                    super::BoardAction::Restart { config_hash }
                } else {
                    super::BoardAction::Reset { config_hash }
                };
                (action, config)
            }
        };
        board_actions.push(ScheduledBoardAction {
            id,
            at,
            board,
            action,
            config,
        });
    }
    board_actions.sort_by(|a, b| (a.at, a.board, &a.id).cmp(&(b.at, b.board, &b.id)));

    let mut buses = config
        .get("bus")
        .map(schema::bus)
        .collect::<Result<Vec<_>, _>>()?;
    buses.sort_by(|a, b| a.name.cmp(&b.name));
    let mut bus_ports = BTreeSet::new();
    for bus in buses {
        let id = board_id(&boards, &bus.board)?;
        let model = BusModel::parse(utf8(&read(&bus.model)?)?)?;
        let board = &mut boards[id];
        let caps = board.sim.capabilities();
        let address = match model.profile {
            // BoardSim объявляет доступность периферии, текущий профиль
            // предоставляет именно I2C0 и SPI2 согласно sim-boards/SUPPORT.md.
            Profile::I2c { address, .. }
                if caps.i2c_fifo.is_some() && bus.bus == 0 && bus.chip_select.is_none() =>
            {
                (false, address)
            }
            Profile::Spi { .. }
                if caps.spi_bytes.is_some()
                    && bus.bus == 2
                    && bus.chip_select.is_some_and(|cs| cs <= 5) =>
            {
                (true, bus.chip_select.unwrap())
            }
            _ => return Err(format!("неподдержанный профиль шины «{}»", bus.name)),
        };
        if !bus_ports.insert((id, bus.bus, address)) {
            return Err("конфликт адресов шины".into());
        }
        let signal = observation(runtime, &bus.signal, &model.unit)?;
        fault_target(
            &mut targets,
            format!("bus:{}", bus.name),
            vec![FaultMechanism::Drop],
        )?;
        sensor_target(&mut targets, &bus.signal, true)?;
        limitations.push(Simplification {
            subject: format!("bus:{}", bus.name),
            text: format!(
                "Подмножество регистров и идеальная передача. {}",
                model.source
            ),
            blocks_proof: true,
        });
        board.buses.push(Bus {
            name: bus.name,
            bus: bus.bus,
            chip_select: bus.chip_select,
            model,
            signal,
            signal_name: bus.signal,
            max_age_ns: bus.max_age_ns,
            sample: None,
            connected: true,
        });
    }

    let mut routes = Vec::new();
    let mut packets = config
        .get("packet")
        .map(schema::packet)
        .collect::<Result<Vec<_>, _>>()?;
    packets.sort_by(|a, b| a.name.cmp(&b.name));
    let mut receivers = BTreeSet::new();
    for route in packets {
        let packet_bytes = route
            .profile
            .map(|profile| profile.max_bytes)
            .or(route.scheduled_max_bytes);
        let radio = packet_bytes.is_some();
        let from = route
            .from
            .as_ref()
            .map(|name| board_id(&boards, name).map(|id| (id, route.from_port)))
            .transpose()?;
        let to = route
            .to
            .as_ref()
            .map(|name| board_id(&boards, name).map(|id| (id, route.to_port)))
            .transpose()?;
        for (endpoint, received) in [(from, false), (to, true)] {
            let Some((id, port)) = endpoint else {
                continue;
            };
            let limits = boards[id].sim.capabilities();
            let supported = if radio {
                limits.radio.is_some_and(|limit| {
                    port < limit.ports
                        && packet_bytes.is_some_and(|bytes| bytes <= limit.packet_bytes)
                })
            } else if received {
                limits
                    .uart_rx_fifo
                    .get(port as usize)
                    .is_some_and(|&n| n > 0)
            } else {
                port < limits.uart_tx_ports
            };
            if !supported {
                return Err(format!("неподдержанный порт «{}»", route.name));
            }
            if received && !receivers.insert((id, radio, port)) {
                return Err("конфликт писателей пакетного входа".into());
            }
        }
        let channel = route
            .profile
            .map(|p| {
                TimedChannel::new(
                    p,
                    Rng::stream(runtime.seed(), &format!("packet:{}", route.name)),
                )
            })
            .transpose()?;
        let slots = if let Some(name) = &route.slots_asset {
            let slots = schema::delivery_slots(&read(name)?)
                .map_err(|error| format!("календарь «{name}»: {error}"))?;
            Some(Rc::from(slots))
        } else {
            None
        };
        fault_target(
            &mut targets,
            format!("channel:{}", route.name),
            vec![FaultMechanism::Drop, FaultMechanism::Delay],
        )?;
        if radio {
            limitations.push(Simplification {
                subject: format!("channel:{}", route.name),
                text: if slots.is_some() {
                    "Записанные слоты доставки. Покрытие радиоканала: NotAnalyzed".into()
                } else {
                    "Пакетное приближение, без физического слоя и моделирования стека".into()
                },
                blocks_proof: true,
            });
        }
        routes.push(Route {
            name: route.name,
            from,
            to,
            radio: channel,
            scheduled_rssi: route.scheduled_rssi,
            packet_bytes: packet_bytes.unwrap_or_default(),
            slots,
            delay_ns: route.delay_ns,
            capacity: route.capacity,
            max_age_ns: route.max_age_ns,
            sequence: 0,
            pending: BTreeMap::new(),
        });
    }

    let mut all_inputs = BTreeMap::<String, Vec<_>>::new();
    for item in config.get("input").map(schema::input) {
        let item = item?;
        let id = observation(runtime, &item.signal, &item.unit)?;
        if !node_specs.iter().any(|n| n.name == item.controller) {
            return Err(format!("неизвестный контроллер «{}»", item.controller));
        }
        if !outputs
            .iter()
            .any(|out| matches!(out.target, Target::Goal(signal) if signal == id))
        {
            sensor_target(
                &mut targets,
                &item.signal,
                matches!(item.kind, InputKind::Scalar { .. }),
            )?;
        } else if !matches!(item.kind, InputKind::Scalar { .. }) {
            return Err("цель верхнего уровня должна быть скаляром".into());
        }
        all_inputs
            .entry(item.controller)
            .or_default()
            .push(InputChannel {
                id,
                name: item.signal,
                unit: item.unit,
                kind: item.kind,
            });
    }

    let mut nodes = Vec::new();
    let mut processes = BTreeMap::new();
    let mut process_specs = Vec::new();
    for item in node_specs {
        let name = item.name.clone();
        let inputs = all_inputs.remove(&name).unwrap_or_default();
        let owned: Vec<_> = out_specs
            .iter()
            .enumerate()
            .filter(|(_, spec)| spec.driver.is_none() && spec.owner == name)
            .map(|(i, _)| i)
            .collect();
        let contract = ControlContract::new(
            inputs,
            owned
                .iter()
                .map(|&i| OutputChannel {
                    id: outputs[i].id,
                    name: outputs[i].name.clone(),
                    unit: out_specs[i].unit.clone(),
                    max_age_ns: outputs[i].max_age,
                })
                .collect(),
        )
        .map_err(|e| e.to_string())?;
        let target = fault_target(
            &mut targets,
            format!("controller:{name}"),
            vec![FaultMechanism::Pause],
        )?;
        let mut transport_faults = Vec::new();
        if item.external.is_some() {
            for direction in ["request", "response"] {
                transport_faults.push(fault_target(
                    &mut targets,
                    format!("transport:{name}/{direction}"),
                    vec![FaultMechanism::Drop, FaultMechanism::Delay],
                )?);
            }
        }
        for &i in &owned {
            paths[i]
                .fault_targets
                .extend(transport_faults.iter().cloned());
            paths[i]
                .feedback
                .extend(contract.inputs().map(|input| input.id));
            paths[i].fault_targets.push(target.clone());
            paths[i].fault_targets.extend(
                contract
                    .inputs()
                    .filter_map(|input| targets.get(&format!("sensor:{}", input.name)).cloned()),
            );
        }
        let bytes = read(&item.program)?;
        let program_hash = fnv1a(bytes.iter().copied());
        let producer_index = u32::try_from(runtime.state.components.len() + nodes.len())
            .map_err(|_| "слишком много контроллеров")?;
        for &i in &owned {
            paths[i].program_hash = program_hash;
            paths[i].producer_index = producer_index;
        }
        let bound = bindings.0.remove(&name);
        let builtin: Option<Box<dyn LocalController>> = if let Some(external) = &item.external {
            limitations.push(Simplification {
                subject: format!("controller:{name}"),
                text: super::EXTERNAL_ISOLATION_GAP.into(),
                blocks_proof: true,
            });
            if bound.is_some() {
                return Err(format!(
                    "внешний контроллер «{name}» нельзя заменить локальным"
                ));
            }
            let args = process_args(&external.args, &contract)?;
            if recording.is_none() {
                let program = source.executable(&item.program)?;
                let executable_bytes = std::fs::read(&program)
                    .map_err(|e| format!("не читается внешний исполняемый файл: {e}"))?;
                if executable_bytes != bytes {
                    return Err(format!(
                        "исполняемый файл «{}» отличается от ресурса",
                        item.program
                    ));
                }
                executables.insert(nodes.len(), (program.clone(), program_hash));
                process_specs.push((
                    nodes.len(),
                    ProcessSpec { program, args },
                    SessionOptions {
                        first_request: RequestId(0),
                        // Зерно протокола доступно работнику. Оно зависит только от
                        // публичной программы и имени, отдельно от генераторов мира.
                        seed: fnv1a(program_hash.to_le_bytes().into_iter().chain(name.bytes())),
                        max_message_bytes: external.max_message_bytes,
                    },
                    external.max_history_bytes,
                ));
            }
            None
        } else if let Some(controller) = bound {
            limitations.push(Simplification { subject: format!("controller:{name}"), text: "Встроенный программный объект передан через API. Его код не связан с файлом графа.".into(), blocks_proof: true });
            Some(controller)
        } else {
            Some(Box::new(super::program::builtin(
                utf8(&bytes)?,
                contract.clone(),
            )?))
        };
        nodes.push(Node {
            clock: Clock::new(item.rate),
            spec: item,
            contract,
            outputs: owned,
            builtin,
            rng: Rng::stream(runtime.seed(), &format!("controller:{name}")),
            delay_in: Rng::stream(runtime.seed(), &format!("controller:{name}/request")),
            delay_out: Rng::stream(runtime.seed(), &format!("controller:{name}/response")),
            requests: BTreeMap::new(),
            answers: BTreeMap::new(),
            next_request: 0,
            last_sent_due: Time::ZERO,
            last_answer: None,
            protocol_sequence: 0,
        });
    }
    if !bindings.0.is_empty() {
        return Err("в документе нет части указанных локальных контроллеров".into());
    }
    for (id, board) in boards.iter().enumerate() {
        let target = fault_target(
            &mut targets,
            format!("board:{}", board.name),
            vec![FaultMechanism::Pause, FaultMechanism::PowerOff],
        )?;
        let producer_index = u32::try_from(runtime.state.components.len() + nodes.len() + id)
            .map_err(|_| "слишком много контроллеров")?;
        for &i in &board.outputs {
            paths[i].program_hash = board.program_hash;
            paths[i].producer_index = producer_index;
            paths[i].feedback.extend(
                board
                    .wires
                    .iter()
                    .map(|wire| wire.signal)
                    .chain(board.buses.iter().map(|bus| bus.signal)),
            );
            paths[i].fault_targets.push(target.clone());
            paths[i]
                .fault_targets
                .extend(board.wires.iter().flat_map(|wire| {
                    [
                        targets[&format!("wire:{}", wire.spec.name)].clone(),
                        targets[&format!("sensor:{}", wire.spec.signal)].clone(),
                    ]
                }));
            paths[i]
                .fault_targets
                .extend(board.buses.iter().flat_map(|bus| {
                    [
                        targets[&format!("bus:{}", bus.name)].clone(),
                        targets[&format!("sensor:{}", bus.signal_name)].clone(),
                    ]
                }));
            paths[i].fault_targets.extend(
                routes
                    .iter()
                    .filter(|route| {
                        route.to.is_some_and(|(owner, _)| owner == id)
                            || route.from.is_some_and(|(owner, _)| owner == id)
                    })
                    .map(|route| targets[&format!("channel:{}", route.name)].clone()),
            );
        }
    }
    link_coverage(&outputs, &mut paths);
    for (drive, supply) in power_links {
        if let (Some(&drive), Some(&supply)) = (channels.get(&drive), channels.get(&supply)) {
            let faults = paths[supply].fault_targets.clone();
            paths[drive].fault_targets.extend(faults);
            paths[drive]
                .fault_targets
                .sort_by(|a, b| a.name.cmp(&b.name));
            paths[drive].fault_targets.dedup_by(|a, b| a.name == b.name);
        }
    }
    let mut stimuli = Vec::new();
    let mut stimuli_channels = BTreeSet::new();
    let mut stimulus_keys = BTreeSet::new();
    for item in config.get("stimulus").map(schema::stimulus) {
        let (at, target, unit, value) = item?;
        let channel = runtime
            .channel(&target)
            .ok_or_else(|| format!("неизвестный стимул «{target}»"))?;
        // Порт привода принимает управляющую величину, а одноимённая физическая
        // степень свободы хранит единицу силы или момента.
        let expected = match model_ports.get(&channel) {
            Some(Some(ack)) => runtime
                .truth_spec(*ack)
                .ok_or("нет единицы порта привода")?
                .unit
                .as_str(),
            Some(None) => "bool",
            None => &runtime.log().header().commands[channel.0 as usize].unit,
        };
        require_unit(&target, expected, &unit)?;
        if at > spec.horizon
            || channels.contains_key(&channel)
            || !stimulus_keys.insert((channel, at))
        {
            return Err(format!(
                "конфликт или время вне режима у стимула «{target}»"
            ));
        }
        stimuli_channels.insert(channel);
        stimuli.push(Stimulus {
            at,
            target: channel,
            value,
        });
    }
    stimuli.sort_by_key(|s| (s.at, s.target));
    let mut transmissions = Vec::new();
    for item in config.get("transmission").map(schema::transmission) {
        let (at, name, bytes) = item?;
        let route = routes
            .iter()
            .position(|route| route.name == name && route.from.is_none())
            .ok_or_else(|| format!("неизвестный канал стимула или конфликт писателей «{name}»"))?;
        if at > spec.horizon || bytes.is_empty() {
            return Err("неверный пакет стимула".into());
        }
        transmissions.push(Transmission { at, route, bytes });
    }
    transmissions.sort_by_key(|t| (t.at, t.route));
    let faults = schedule_faults(config, &targets, spec.horizon)?;
    if let Some(electrical) = &runtime.state.electrical {
        electrical.check_horizon(spec.horizon)?;
        for (slot, connection) in electrical.boards.iter().enumerate() {
            let index = board_id(&boards, &connection.name)?;
            let board = &mut boards[index];
            if board.end_sample {
                return Err(format!(
                    "плата «{}»: цепь несовместима с выборкой конца шага",
                    board.name
                ));
            }
            if faults.iter().any(|fault| {
                fault.target == format!("board:{}", board.name)
                    && fault.action == FaultAction::PowerOff
            }) {
                return Err(format!(
                    "плата «{}»: отключение должно задаваться через цепь",
                    board.name
                ));
            }
            for pin in connection.contacts() {
                if !pin_spec(board.sim.board(), pin).is_some_and(|p| p.gpio && !p.internal_only) {
                    return Err(format!(
                        "плата «{}»: GPIO {pin} недоступен для схемы",
                        board.name
                    ));
                }
                if board.outputs.iter().any(|&id| {
                    let output = &outputs[id];
                    output.kind == TargetKind::Power
                        && output.driver.as_ref().is_some_and(|driver| {
                            [Some(driver.pin), driver.enable_pin, driver.direction_pin]
                                .contains(&Some(pin))
                        })
                }) {
                    return Err(format!(
                        "плата «{}», GPIO {pin}: ключом уже владеет цепь",
                        board.name
                    ));
                }
            }
            board.electrical = Some(slot);
        }
    }
    let mut fault_edges = Vec::new();
    for (index, fault) in faults.iter().enumerate() {
        fault_edges.push(FaultEdge {
            index,
            active: true,
        });
        if fault.end.is_some() {
            fault_edges.push(FaultEdge {
                index,
                active: false,
            });
        }
    }
    // При совпадении границ сначала завершается предыдущий отказ.
    fault_edges.sort_by_key(|edge| {
        (
            edge.active,
            &faults[edge.index].target,
            &faults[edge.index].id,
        )
    });

    let catalog_hash = fnv1a(
        spec.catalog.iter().copied().chain(
            include_bytes!("../../../../catalog/boards/catalog.toml")
                .iter()
                .copied(),
        ),
    );
    let control_hash = fnv1a(config.text.bytes());
    let identity = RunIdentity {
        build: format!(
            "sim-runtime {} ({}) / MuJoCo {}",
            env!("CARGO_PKG_VERSION"),
            env!("SIM_RUNTIME_BUILD"),
            sim_mujoco::version()
        ),
        world: fnv1a(
            runtime
                .world_model_hash
                .to_le_bytes()
                .into_iter()
                .chain(spec.world.iter().copied()),
        ),
        catalog: catalog_hash,
        control: runtime
            .state
            .electrical
            .as_ref()
            .map_or(control_hash, |circuit| {
                fnv1a(
                    control_hash
                        .to_le_bytes()
                        .into_iter()
                        .chain(circuit.identity.to_le_bytes()),
                )
            }),
        paired_setup: config.paired_setup(),
        assets: by_path.into_iter().collect(),
        task: fnv1a(spec.task.iter().copied()),
        criteria: fnv1a(spec.criteria.iter().copied()),
        seed: runtime.seed(),
        mode: spec.mode.into(),
        phase_horizon: spec.horizon,
    };
    if let Some(recording) = &recording
        && (!matches!(recording.origin, RunOrigin::Live | RunOrigin::Replay)
            || recording.identity != identity
            || recording.log.header() != runtime.log().header())
    {
        return Err("входы контура отличаются от записи".into());
    }
    let goals: BTreeSet<_> = outputs
        .iter()
        .filter_map(|output| match output.target {
            Target::Goal(id) => Some(id),
            Target::Channel(..) => None,
        })
        .collect();
    let mut input_shapes = BTreeMap::new();
    for board in &boards {
        input_shapes.extend(board.wires.iter().map(|wire| (wire.signal, None)));
        input_shapes.extend(board.buses.iter().map(|bus| (bus.signal, None)));
    }
    for node in &nodes {
        for input in node.contract.inputs() {
            let format = match input.kind {
                InputKind::Scalar { .. } => None,
                InputKind::Image { format, .. } => Some(format),
            };
            if input_shapes
                .insert(input.id, format)
                .is_some_and(|old| old != format)
            {
                return Err(format!("несогласованный формат входа «{}»", input.name));
            }
        }
    }
    for (id, program, options, history) in process_specs {
        let node = &nodes[id];
        let process = ProcessTransport::spawn_supervised(
            program,
            node.contract.clone(),
            options,
            history,
            |pid| source.worker_started(&node.spec.name, pid),
        )
        .map_err(|e| e.to_string())?;
        processes.insert(id, process);
    }
    let mut engine = Engine {
        identity,
        origin: if recording.is_some() {
            RunOrigin::Replay
        } else if has_programmatic_controller {
            RunOrigin::ProgrammaticController
        } else {
            RunOrigin::Live
        },
        paths,
        events: Rc::default(),
        transcript: Rc::default(),
        playback: recording.map(|r| (r, 0)),
        executables,
        queue: Default::default(),
        nodes,
        outputs,
        channels,
        stimuli_channels,
        stimulus_values: BTreeMap::new(),
        stimulus_integrated: BTreeMap::new(),
        stimulus_pending: Vec::new(),
        boards,
        external_boards,
        board_actions,
        routes,
        faults,
        fault_edges,
        active: BTreeMap::new(),
        applied_fault: BTreeMap::new(),
        recovering: BTreeMap::new(),
        sensor_barriers: BTreeMap::new(),
        observation_writers: BTreeMap::new(),
        input_shapes,
        goals,
        stimuli,
        transmissions,
        sequence: 0,
        direct: Vec::new(),
        latched_forces: Vec::new(),
        latched_start: Time::ZERO,
        direct_pending: Vec::new(),
        numeric_commands: Vec::new(),
    };
    for id in 0..engine.nodes.len() {
        engine.schedule(Time::ZERO, Schedule::Controller, id);
    }
    for id in 0..engine.boards.len() {
        engine.schedule(Time::ZERO, Schedule::Board, id);
    }
    for id in 0..engine.board_actions.len() {
        engine.schedule(engine.board_actions[id].at, Schedule::BoardAction, id);
    }
    for id in 0..engine.stimuli.len() {
        engine.schedule(engine.stimuli[id].at, Schedule::Stimulus, id);
    }
    for id in 0..engine.transmissions.len() {
        engine.schedule(engine.transmissions[id].at, Schedule::PacketSend, id);
    }
    for id in 0..engine.fault_edges.len() {
        let edge = &engine.fault_edges[id];
        let fault = &engine.faults[edge.index];
        engine.schedule(
            if edge.active {
                fault.start
            } else {
                fault.end.expect("конец отказа")
            },
            Schedule::Fault,
            id,
        );
    }
    Ok((engine, processes, limitations))
}

fn open_board_proof(name: &str, reason: &str) -> Simplification {
    Simplification {
        subject: format!("board:{name}"),
        text: format!("NotAnalyzed: {reason}"),
        blocks_proof: true,
    }
}

fn load_profile<T>(
    path: &str,
    read: &mut impl FnMut(&str) -> Result<Vec<u8>, String>,
    parse: impl FnOnce(&str) -> Result<T, String>,
) -> Result<T, String> {
    let bytes = read(path)?;
    utf8(&bytes)
        .and_then(parse)
        .map_err(|e| format!("ресурс «{path}»: {e}"))
}

fn load_board_catalog(
    path: Option<&str>,
    read: &mut impl FnMut(&str) -> Result<Vec<u8>, String>,
) -> Result<BoardCatalog, String> {
    match path {
        Some(path) => load_profile(path, read, |text| {
            BoardCatalog::parse(text).map_err(|e| e.to_string())
        }),
        None => BoardCatalog::builtin().map_err(|e| e.to_string()),
    }
}

fn require_profile_board(name: &str, actual: &str, expected: &str) -> Result<(), String> {
    if actual != expected {
        return Err(format!(
            "плата «{name}»: профиль «{actual}» несовместим с «{expected}»"
        ));
    }
    Ok(())
}

fn load_board_image(
    item: &schema::BoardSpec,
    board: &mut dyn BoardSim,
    image: &[u8],
    read: &mut impl FnMut(&str) -> Result<Vec<u8>, String>,
) -> Result<(), String> {
    let (Some(profile), Some(rom), Some(efuse)) = (&item.flash_profile, &item.rom, &item.efuse)
    else {
        return board.load(image).map_err(|e| e.to_string());
    };
    let profile = load_profile(profile, read, schema::flash_profile)?;
    require_profile_board(&item.name, &profile.board, board.board().key())?;
    let rom = read(rom)?;
    // ESP-IDF soc/esp32c3/register/soc/efuse_reg.h: чтение от EFUSE_RD_WR_DIS_REG
    // до EFUSE_RD_SYS_PART2_DATA7_REG включительно. API BoardSim проверяет размер.
    let bytes = read(efuse)?;
    let (words, remainder) = bytes.as_chunks::<{ size_of::<u32>() }>();
    if !remainder.is_empty() {
        return Err(format!("ресурс «{efuse}»: неполное 32-битное слово eFuse"));
    }
    let words = words
        .iter()
        .copied()
        .map(u32::from_le_bytes)
        .collect::<Vec<_>>();
    board
        .load_esp_flash_with_options(
            image,
            profile.partition_table_offset,
            &rom,
            &words,
            profile.straps,
            profile.options,
        )
        .map_err(|e| e.to_string())
}

fn load_mcu_energy(
    item: &schema::BoardSpec,
    board: &str,
    horizon: Time,
    read: &mut impl FnMut(&str) -> Result<Vec<u8>, String>,
) -> Result<McuEnergyReport, String> {
    let power = item
        .power_profile
        .as_deref()
        .map(|path| load_profile(path, read, ControlConfig::parse_mcu_power_profile))
        .transpose()?;
    if let Some(power) = &power {
        require_profile_board(&item.name, power.board(), board)?;
    }
    let schedule = item
        .power_schedule
        .as_deref()
        .map(|path| load_profile(path, read, schema::power_schedule))
        .transpose()?;
    let mut states = Vec::new();
    if let Some(schedule) = &schedule {
        let power: &McuPowerProfile = power.as_ref().ok_or("нет профиля точек тока МК")?;
        for (id, duration) in &schedule.states {
            if let Analysis::NotAnalyzed { reason } = power.current(id) {
                return Err(format!("плата «{}»: {reason}", item.name));
            }
            states.push(SleepDrainState {
                id,
                duration: *duration,
                controller_a: None,
                loads: None,
            });
        }
    }
    let horizon_source = Provenance::Estimate("горизонт расчёта контура".into());
    // Адрес результата расчёта МК. Электрическая ветвь батареи здесь не установлена.
    let branch = Section::Element {
        component: item.name.clone(),
        element: "mcu".into(),
    };
    assess_mcu_energy(
        power.as_ref(),
        &SleepDrainInput {
            scope: Scope::Assembly,
            mode: &Mode::Normal,
            branch: &branch,
            required: Some(SleepDrainRequirement {
                duration: horizon,
                source: &horizon_source,
            }),
            profile: schedule.as_ref().map(|s| SleepDrainProfile {
                states: &states,
                repeat: s.repeat,
                source: &s.source,
            }),
            battery: None,
        },
    )
    .map_err(|e| e.to_string())
}

fn load_external(
    item: &schema::ExternalBoardSpec,
    read: &mut impl FnMut(&str) -> Result<Vec<u8>, String>,
) -> Result<schema::ExternalProfile, String> {
    let profile = load_profile(&item.profile, read, schema::external_profile)?;
    require_profile_board(&item.name, &profile.board, &item.catalog)?;
    if profile.flash_boot && item.image.is_none() {
        return Err(format!(
            "внешняя плата «{}»: для загрузки QEMU нужен образ flash",
            item.name
        ));
    }
    // Руководство QEMU Espressif для S3: «Compiling the ESP-IDF program to emulate»
    // и «Specifying eFuse storage». Эти размеры относятся к образам QEMU.
    if let Some(path) = &item.image
        && ![2 << 20, 4 << 20, 8 << 20, 16 << 20].contains(&read(path)?.len())
    {
        return Err(format!(
            "ресурс «{path}»: неподдержанный размер образа flash QEMU"
        ));
    }
    if let Some(path) = &item.efuse
        && read(path)?.len() != 1 << 10
    {
        return Err(format!(
            "ресурс «{path}»: неподдержанный размер образа eFuse QEMU"
        ));
    }
    Ok(profile)
}

fn process_args(args: &[String], contract: &ControlContract) -> Result<Vec<OsString>, String> {
    args.iter()
        .map(|arg| {
            let index = if let Some(name) = arg.strip_prefix("$input:") {
                contract
                    .inputs()
                    .find(|input| input.name == name)
                    .map(|input| input.id.0)
            } else if let Some(name) = arg.strip_prefix("$output:") {
                contract
                    .outputs()
                    .find(|output| output.name == name)
                    .map(|output| output.id.0)
            } else {
                return Ok(arg.into());
            };
            index
                .map(|i| i.to_string().into())
                .ok_or_else(|| format!("аргумент «{arg}» не связан с каналом"))
        })
        .collect()
}

fn link_coverage(outputs: &[Output], paths: &mut [ControlPath]) {
    let by_signal: BTreeMap<_, _> = outputs
        .iter()
        .enumerate()
        .filter_map(|(id, output)| {
            if let Target::Goal(signal) = output.target {
                Some((signal, id))
            } else {
                None
            }
        })
        .collect();
    for i in 0..paths.len() {
        let mut signals = BTreeSet::new();
        let mut faults = BTreeSet::new();
        let mut walked = BTreeSet::new();
        let mut visit = vec![i];
        while let Some(id) = visit.pop() {
            if !walked.insert(id) {
                continue;
            }
            signals.extend(paths[id].feedback.iter().copied());
            faults.extend(paths[id].fault_targets.iter().cloned());
            visit.extend(
                paths[id]
                    .feedback
                    .iter()
                    .filter_map(|signal| by_signal.get(signal).copied()),
            );
        }
        paths[i].feedback = signals.into_iter().collect();
        paths[i].fault_targets = faults.into_iter().collect();
    }
}

fn schedule_faults(
    config: &ControlConfig,
    targets: &BTreeMap<String, FaultTarget>,
    horizon: Time,
) -> Result<Vec<schema::FaultSpec>, String> {
    let mut faults = config
        .get("fault")
        .map(schema::fault)
        .collect::<Result<Vec<_>, _>>()?;
    faults.sort_by(|a, b| (&a.target, a.start, &a.id).cmp(&(&b.target, b.start, &b.id)));
    let mut ids = BTreeSet::new();
    for (i, fault) in faults.iter().enumerate() {
        let mechanism = match fault.action {
            FaultAction::Drop => FaultMechanism::Drop,
            FaultAction::Stuck(_) => FaultMechanism::Stuck,
            FaultAction::PowerOff => FaultMechanism::PowerOff,
            FaultAction::Pause => FaultMechanism::Pause,
            FaultAction::Delay(_) => FaultMechanism::Delay,
        };
        if !targets
            .get(&fault.target)
            .is_some_and(|t| t.mechanisms.contains(&mechanism))
        {
            return Err(format!(
                "неподдержанный отказ «{}» для «{}»",
                fault.id, fault.target
            ));
        }
        if !ids.insert(&fault.id)
            || fault.start > horizon
            || fault.end.is_some_and(|end| end > horizon)
        {
            return Err(format!(
                "повтор или время вне режима у отказа «{}»",
                fault.id
            ));
        }
        if i > 0
            && faults[i - 1].target == fault.target
            && faults[i - 1].end.is_none_or(|end| end > fault.start)
        {
            return Err(format!("пересечение отказов на «{}»", fault.target));
        }
    }
    Ok(faults)
}

fn observation(runtime: &Runtime, name: &str, unit: &str) -> Result<SignalId, String> {
    let id = runtime
        .observation_id(name)
        .ok_or_else(|| format!("нет сигнала «{name}»"))?;
    require_unit(
        name,
        &runtime.log().header().observations[id.0 as usize].unit,
        unit,
    )?;
    Ok(id)
}

fn require_unit(name: &str, actual: &str, expected: &str) -> Result<(), String> {
    if actual == expected {
        Ok(())
    } else {
        Err(format!("«{name}»: единица «{actual}» вместо «{expected}»"))
    }
}

fn check_pin(board: &dyn BoardSim, pin: u32, pwm: bool) -> Result<(), String> {
    if pin_spec(board.board(), pin).is_some_and(|p| if pwm { p.pwm } else { p.gpio }) {
        Ok(())
    } else {
        Err(format!("неподдержанный вывод платы {pin}"))
    }
}

fn fault_target(
    targets: &mut BTreeMap<String, FaultTarget>,
    name: String,
    mechanisms: Vec<FaultMechanism>,
) -> Result<FaultTarget, String> {
    let target = FaultTarget {
        name: name.clone(),
        mechanisms,
    };
    if targets.insert(name.clone(), target.clone()).is_some() {
        return Err(format!("конфликт имени «{name}»"));
    }
    Ok(target)
}

fn sensor_target(
    targets: &mut BTreeMap<String, FaultTarget>,
    signal: &str,
    scalar: bool,
) -> Result<(), String> {
    let name = format!("sensor:{signal}");
    let mechanisms = if scalar {
        vec![FaultMechanism::Drop, FaultMechanism::Stuck]
    } else {
        vec![FaultMechanism::Drop]
    };
    if let Some(target) = targets.get(&name) {
        if target.mechanisms != mechanisms {
            return Err(format!("двойной тип сигнала «{signal}»"));
        }
        return Ok(());
    }
    fault_target(targets, name, mechanisms)?;
    Ok(())
}

fn utf8(bytes: &[u8]) -> Result<&str, String> {
    std::str::from_utf8(bytes).map_err(|e| format!("документ должен быть UTF-8: {e}"))
}
