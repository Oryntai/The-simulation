use std::collections::{BTreeMap, BTreeSet};

use serde::Serialize;
use sim_world::CompiledWorld;

use crate::task::{check_name, finite};
use crate::{Case, Error, Fault, FaultKind, Fingerprint, Signal, Source, invalid};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub enum NodeKind {
    Sensor,
    Wire,
    Controller,
    Driver,
    Power,
    Drive,
    Bus,
    Radio,
}

impl NodeKind {
    fn faults(self) -> &'static [FaultKind] {
        use FaultKind::*;
        match self {
            Self::Sensor => &[SensorUnavailable],
            Self::Wire => &[WireOpen],
            Self::Controller => &[ControllerHalted],
            Self::Driver => &[DriverDisabled],
            Self::Power => &[PowerCut],
            Self::Drive => &[DriveDisabled],
            Self::Bus | Self::Radio => &[PacketLoss, PacketDelay],
        }
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct Node {
    pub id: String,
    pub kind: NodeKind,
    /// Двоичная истина модели (0 штатно, 1 пока отказ действует).
    /// Для шин и радио вместо неё нужны исходы конкретных сообщений.
    pub fault_state: Option<Signal>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum Direction {
    Input,
    Output,
}

#[derive(Clone, Debug, Serialize)]
pub struct Hop {
    pub id: String,
    pub node: String,
    pub direction: Direction,
    pub delay_ns: u64,
    pub jitter_ns: u64,
    pub receiver_tick_ns: u64,
    pub source: Source,
}

/// Необязательное соответствие для старого формата конструктора.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct LegacyDrive {
    pub pin: u32,
    pub link: usize,
}

#[derive(Clone, Debug, Serialize)]
pub struct Path {
    pub id: String,
    pub drive: String,
    pub observation: Signal,
    pub command: Signal,
    /// Сигнал истины, записываемый моделью при фактическом принятии команды.
    pub accepted: Signal,
    /// Измеряемые физические состояния, связанные с выходом этого пути.
    pub physical_response: Vec<Signal>,
    pub controller: String,
    /// Отпечаток кода, который реально исполняет этот контроллер.
    pub program: Fingerprint,
    pub command_delta: f64,
    pub max_observation_age_ns: u64,
    pub source: Source,
    pub nodes: Vec<Node>,
    pub hops: Vec<Hop>,
    pub legacy: Option<LegacyDrive>,
}

/// Топология берётся из разрешённой проводки и установленных моделей.
/// Обязательные режимы приходят из профиля эксплуатации независимо от файла задачи.
#[derive(Clone, Debug, Serialize)]
pub struct Topology {
    pub required_modes: Vec<String>,
    pub paths: Vec<Path>,
    /// Известная область действия ограничений по subject. Неизвестная тема
    /// считается существенной для всех путей.
    pub limitation_paths: BTreeMap<String, BTreeSet<String>>,
}

impl Topology {
    pub(crate) fn validate(&self, world: &CompiledWorld) -> Result<(), Error> {
        let mut modes = BTreeSet::new();
        let mut ids = BTreeSet::new();
        for mode in &self.required_modes {
            check_name(mode)?;
            if !modes.insert(mode) {
                return Err(invalid(
                    "повтор обязательного режима в профиле эксплуатации",
                ));
            }
        }
        for path in &self.paths {
            path.validate(world)?;
            if !ids.insert(&path.id) {
                return Err(invalid("повтор имени пути"));
            }
        }
        for (subject, paths) in &self.limitation_paths {
            check_name(subject)?;
            if paths.is_empty() || paths.iter().any(|path| !ids.contains(path)) {
                return Err(invalid("неверная область применимости ограничения модели"));
            }
        }
        Ok(())
    }

    pub(crate) fn cases(&self) -> Vec<Case> {
        let mut cases = BTreeSet::new();
        for path in &self.paths {
            for mode in &self.required_modes {
                let mut add = |fault| {
                    cases.insert(Case {
                        path: path.id.clone(),
                        mode: mode.clone(),
                        fault,
                    });
                };
                add(None);
                for node in &path.nodes {
                    for &kind in node.kind.faults() {
                        add(Some(Fault {
                            node: node.id.clone(),
                            kind,
                        }));
                    }
                }
            }
        }
        cases.into_iter().collect()
    }
}

impl Path {
    fn validate(&self, world: &CompiledWorld) -> Result<(), Error> {
        check_name(&self.id)?;
        self.source.validate()?;
        check_name(&self.controller)?;
        self.observation.validate()?;
        self.command.validate()?;
        self.accepted.validate()?;
        let mut outcomes = BTreeSet::new();
        for signal in &self.physical_response {
            signal.validate()?;
            if signal == &self.accepted || !outcomes.insert(signal) {
                return Err(invalid(
                    "повтор физической реакции или ссылка на саму команду",
                ));
            }
        }
        finite(self.command_delta)?;
        if self.command_delta <= 0.0
            || self.max_observation_age_ns == 0
            || self.accepted.unit != self.command.unit
        {
            return Err(invalid(
                "неверные единицы, срок свежести или порог реакции контроллера",
            ));
        }
        let Some(drive) = world.drives.iter().find(|drive| drive.name == self.drive) else {
            return Err(invalid(format!("не найден привод пути «{}»", self.drive)));
        };
        if self.command.name != drive.effort
            || !self.accepted.name.starts_with(&format!("{}/", drive.name))
        {
            return Err(invalid(
                "команда или подтверждение привязаны к другому приводу",
            ));
        }
        let mut nodes = BTreeSet::new();
        for node in &self.nodes {
            check_name(&node.id)?;
            if let Some(state) = &node.fault_state {
                state.validate()?;
                if !matches!(state.unit.as_str(), "1" | "bool") {
                    return Err(invalid(
                        "состояние отказа должно быть двоичным сигналом истины",
                    ));
                }
            }
            if !nodes.insert(&node.id) {
                return Err(invalid("повтор узла пути"));
            }
        }
        if !self.nodes.iter().any(|n| {
            n.kind == NodeKind::Drive
                && (n.id == drive.name || n.id.strip_prefix("actuator:") == Some(&drive.effort))
        }) || !self.nodes.iter().any(|n| {
            n.kind == NodeKind::Controller
                && (n.id == self.controller
                    || n.id.split_once(':').is_some_and(|(kind, name)| {
                        matches!(kind, "board" | "controller") && name == self.controller
                    }))
        }) || [NodeKind::Sensor, NodeKind::Power, NodeKind::Driver]
            .iter()
            .any(|kind| !self.nodes.iter().any(|node| node.kind == *kind))
        {
            return Err(invalid(
                "в пути отсутствуют обязательные узлы замкнутого контура",
            ));
        }
        let mut hops = BTreeSet::new();
        for hop in &self.hops {
            check_name(&hop.id)?;
            hop.source.validate()?;
            if !hops.insert(&hop.id)
                || !nodes.contains(&hop.node)
                || hop.jitter_ns > hop.delay_ns
                || hop.receiver_tick_ns == 0
                || hop
                    .delay_ns
                    .checked_add(hop.jitter_ns)
                    .and_then(|ns| ns.checked_add(hop.receiver_tick_ns))
                    .is_none()
            {
                return Err(invalid("некорректный участок доставки"));
            }
        }
        if self.nodes.iter().any(|n| {
            matches!(n.kind, NodeKind::Bus | NodeKind::Radio)
                && !self.hops.iter().any(|hop| hop.node == n.id)
        }) {
            return Err(invalid("для шины или радио отсутствует профиль задержки"));
        }
        Ok(())
    }
}
