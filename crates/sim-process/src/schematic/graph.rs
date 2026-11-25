use std::collections::{BTreeMap, BTreeSet, VecDeque};

use crate::{ProcessError, SourcedRange};

use super::{Parameter, identifier, invalid};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Scope {
    Assembly,
    Reference,
}

impl Scope {
    pub(super) fn exact(self, range: &SourcedRange) -> bool {
        range.min == range.max
            && (range.can_prove()
                || (self == Self::Reference
                    && matches!(range.provenance, sim_world::Provenance::Derived(_))))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContactKind {
    Plug,
    Socket,
    Terminal,
    Pad,
    Bare,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Contact {
    pub kind: ContactKind,
    pub family: Option<String>,
    /// Идентификатор позиции ключованного соединителя.
    pub key: Option<String>,
}

impl Contact {
    pub(super) fn validate(&self) -> Result<(), ProcessError> {
        let keyed = matches!(self.kind, ContactKind::Plug | ContactKind::Socket);
        if keyed != self.family.is_some()
            || (!keyed && self.key.is_some())
            || self.family.iter().chain(&self.key).any(|s| !identifier(s))
        {
            return Err(invalid(
                "штекеру и гнезду требуется семейство; позиция допустима только у разъёма",
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PinRole {
    Passive,
    Power,
    Return,
    Control,
    SignalOut(String),
    SignalIn(String),
    /// Линия с попеременной передачей, арбитраж остаётся задачей модели вывода.
    SignalIo(String),
}

impl PinRole {
    pub(super) fn reference(&self) -> Option<&str> {
        match self {
            Self::SignalOut(r) | Self::SignalIn(r) | Self::SignalIo(r) => Some(r),
            _ => None,
        }
    }

    pub(super) fn sends(&self) -> bool {
        matches!(self, Self::SignalOut(_) | Self::SignalIo(_))
    }

    pub(super) fn receives(&self) -> bool {
        matches!(self, Self::SignalIn(_) | Self::SignalIo(_))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Endpoint {
    pub component: String,
    pub pin: String,
}

impl std::fmt::Display for Endpoint {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}/{}", self.component, self.pin)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointKind {
    Plug,
    Terminal,
    Solder,
    Splice,
}

impl JointKind {
    pub fn mates(self, a: &Contact, b: &Contact) -> bool {
        use ContactKind::*;
        match (self, a.kind, b.kind) {
            (Self::Plug, Plug, Socket) | (Self::Plug, Socket, Plug) => {
                a.family.is_some() && a.family == b.family && a.key == b.key
            }
            (Self::Terminal, Bare, Terminal)
            | (Self::Terminal, Terminal, Bare)
            | (Self::Solder, Bare, Pad)
            | (Self::Solder, Pad, Bare)
            | (Self::Solder, Bare, Bare)
            | (Self::Splice, Bare, Bare) => true,
            _ => false,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Connection {
    pub kind: JointKind,
    pub ends: [Endpoint; 2],
    pub resistance_ohm: Parameter,
    pub current_limit_a: Parameter,
    /// Предшествующие операции, учитывающие доступ к местам сборки.
    pub after: BTreeSet<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Component {
    pub part: String,
    /// Имена цепей используются для сверки монтажа и поиска сигнальных пар.
    /// Электрическая связь задаётся явными соединениями и элементами.
    pub nets: BTreeMap<String, String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum PinFunction {
    GpioInput,
    GpioOutput,
    Adc,
    Pwm,
    Uart,
    I2c,
    Spi,
}

/// Канал уже разрешён в физический контакт установленной платы. Ключ контакта хранится без номера канала.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum BindingTarget {
    BoardPin {
        board: String,
        pin: String,
        function: PinFunction,
    },
    BoardPower {
        board: String,
        rail: String,
    },
    DrivePower {
        drive: String,
    },
    SensorPower {
        sensor: String,
    },
}

impl BindingTarget {
    pub(super) fn valid(&self) -> bool {
        match self {
            Self::BoardPin { board, pin, .. } => identifier(board) && identifier(pin),
            Self::BoardPower { board, rail } => identifier(board) && identifier(rail),
            Self::DrivePower { drive } => identifier(drive),
            Self::SensorPower { sensor } => identifier(sensor),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Binding {
    pub target: BindingTarget,
    /// Для контакта платы один вывод. Для питания порядок «плюс, возврат».
    pub ends: Vec<Endpoint>,
}

/// Заданная полярность шины с измерением между физическими точками.
#[derive(Clone, Debug, PartialEq)]
pub struct Rail {
    pub ends: [Endpoint; 2],
    pub reverse_limit_v: Parameter,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Schematic {
    pub name: String,
    pub scope: Scope,
    pub nets: BTreeSet<String>,
    pub components: BTreeMap<String, Component>,
    pub connections: BTreeMap<String, Connection>,
    pub rails: BTreeMap<String, Rail>,
    pub bindings: Vec<Binding>,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Section {
    Connection(String),
    Element { component: String, element: String },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PathStep {
    pub section: Section,
    pub from: Endpoint,
    pub to: Endpoint,
}

#[derive(Clone, Debug)]
pub(super) struct Edge {
    pub section: Section,
    pub to: usize,
    pub assured: bool,
}

pub(super) fn path<'a>(
    edges: &'a [Vec<Edge>],
    start: usize,
    end: usize,
    assured: bool,
    excluded: &BTreeSet<Section>,
) -> Option<Vec<(usize, &'a Edge)>> {
    let mut seen = BTreeSet::from([start]);
    let mut prior = BTreeMap::new();
    let mut pending = VecDeque::from([start]);
    while let Some(at) = pending.pop_front() {
        if at == end {
            let mut next = end;
            let mut steps = Vec::new();
            while let Some(&(from, edge)) = prior.get(&next) {
                steps.push((from, edge));
                next = from;
            }
            steps.reverse();
            return Some(steps);
        }
        for edge in &edges[at] {
            if (!assured || edge.assured)
                && !excluded.contains(&edge.section)
                && seen.insert(edge.to)
            {
                prior.insert(edge.to, (at, edge));
                pending.push_back(edge.to);
            }
        }
    }
    None
}
