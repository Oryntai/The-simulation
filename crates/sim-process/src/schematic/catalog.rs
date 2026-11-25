use std::collections::{BTreeMap, BTreeSet};

use crate::{ProcessError, SourcedRange};

use super::{Contact, PinRole, identifier, invalid};

/// При пробеле в данных указывается способ получить применимое значение.
#[derive(Clone, Debug, PartialEq)]
pub struct Parameter {
    pub range: Option<SourcedRange>,
    pub measurement: String,
}

impl Parameter {
    pub(super) fn validate(&self, valid: impl Fn(f64) -> bool) -> Result<(), ProcessError> {
        if !identifier(&self.measurement) {
            return Err(invalid("требуется способ замера"));
        }
        if let Some(range) = &self.range {
            range.validate()?;
            if !valid(range.min) || !valid(range.max) || !valid(range.nominal) {
                return Err(invalid("диапазон выходит за допустимые границы"));
            }
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum PartKind {
    Resistor,
    PolarCapacitor,
    Capacitor,
    Diode,
    Nmos,
    Pmos,
    SwitchModule,
    LinearRegulator,
    SwitchingRegulator,
    ChargePowerModule,
    UsbSource,
    Battery,
    BenchSupply,
    Wire,
    ScrewTerminal,
    Header,
    SolderJoint,
    Other,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Pin {
    pub contact: Contact,
    pub role: PinRole,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PrimitiveKind {
    Resistor,
    Capacitor,
    Inductor,
    Voltage,
    Current,
    Diode,
    Nmos,
    Pmos,
    Switch,
    Vcvs,
    Vccs,
}

impl PrimitiveKind {
    pub(super) fn parameters(self) -> &'static [&'static str] {
        match self {
            Self::Resistor => &["ohm"],
            Self::Capacitor => &["farad"],
            Self::Inductor => &["henry"],
            Self::Voltage => &["volt"],
            Self::Current => &["ampere"],
            Self::Diode => &["is_a", "ideality", "series_ohm"],
            Self::Nmos | Self::Pmos => &[
                "threshold_v",
                "kp_a_per_v2",
                "lambda_per_v",
                "is_a",
                "width_m",
                "length_m",
            ],
            Self::Switch => &["on_ohm", "off_ohm", "threshold_v", "hysteresis_v"],
            Self::Vcvs => &["gain"],
            Self::Vccs => &["siemens"],
        }
    }

    fn node_count(self) -> usize {
        match self {
            Self::Nmos | Self::Pmos | Self::Switch | Self::Vcvs | Self::Vccs => 4,
            _ => 2,
        }
    }

    fn accepts(name: &str, value: f64) -> bool {
        match name {
            "volt" | "ampere" | "gain" | "siemens" | "threshold_v" => true,
            "hysteresis_v" | "lambda_per_v" => value >= 0.0,
            "series_ohm" => value == 0.0 || (value > 0.0 && value.recip().is_finite()),
            "ohm" | "on_ohm" | "off_ohm" => value > 0.0 && value.recip().is_finite(),
            _ => value > 0.0,
        }
    }
}

/// Узлы элемента перечисляются в порядке выводов SPICE. Внутренние узлы локальны детали.
#[derive(Clone, Debug, PartialEq)]
pub struct Primitive {
    pub kind: PrimitiveKind,
    pub nodes: Vec<String>,
    pub parameters: BTreeMap<String, Parameter>,
    /// Паспортный проводящий участок (например жила), пригодный как штатный путь.
    pub continuity: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Rating {
    pub quantity: Quantity,
    pub nodes: [String; 2],
    pub limit: Parameter,
    pub conditions: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Quantity {
    CurrentA,
    VoltageV,
    ReverseVoltageV,
    PowerW,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Part {
    pub kind: PartKind,
    pub source: String,
    pub pins: BTreeMap<String, Pin>,
    pub internal_nodes: BTreeSet<String>,
    pub elements: BTreeMap<String, Primitive>,
    pub ratings: BTreeMap<String, Rating>,
    /// Существенный пробел эквивалента, например отсутствие обратной ветви модуля.
    pub model_gap: Option<String>,
    /// Синтетическая позиция с областью применения только в расчётном корпусе.
    pub reference_only: bool,
}

#[derive(Clone, Debug)]
pub struct Catalog(BTreeMap<String, Part>);

impl Catalog {
    pub fn new(parts: BTreeMap<String, Part>) -> Result<Self, ProcessError> {
        if parts.is_empty() {
            return Err(invalid("электрический каталог пуст"));
        }
        for (id, part) in &parts {
            part.validate(id)
                .map_err(|e| invalid(format!("позиция «{id}»: {e}")))?;
        }
        Ok(Self(parts))
    }

    pub fn part(&self, id: &str) -> Option<&Part> {
        self.0.get(id)
    }

    pub fn parts(&self) -> &BTreeMap<String, Part> {
        &self.0
    }
}

impl Part {
    fn validate(&self, id: &str) -> Result<(), ProcessError> {
        if !identifier(id)
            || !identifier(&self.source)
            || self.pins.is_empty()
            || self.model_gap.as_ref().is_some_and(|gap| !identifier(gap))
        {
            return Err(invalid(
                "требуются имя, источник и выводы; описание пробела должно быть непустым",
            ));
        }
        if self.elements.is_empty() && self.model_gap.is_none() {
            return Err(invalid(
                "требуется эквивалентная схема или описание пробела модели",
            ));
        }
        for (name, pin) in &self.pins {
            if !identifier(name) || name.contains('/') || self.internal_nodes.contains(name) {
                return Err(invalid(format!("неоднозначное имя вывода «{name}»")));
            }
            pin.contact.validate()?;
            if let Some(reference) = pin.role.reference()
                && !self
                    .pins
                    .get(reference)
                    .is_some_and(|r| r.role == PinRole::Return)
            {
                return Err(invalid(format!(
                    "«{name}»: опора «{reference}» должна быть выводом возврата"
                )));
            }
        }
        if self.internal_nodes.iter().any(|node| !identifier(node)) {
            return Err(invalid("недопустимое имя внутреннего узла"));
        }
        for (name, element) in &self.elements {
            self.validate_element(name, element)
                .map_err(|e| invalid(format!("элемент «{name}»: {e}")))?;
        }
        for (name, rating) in &self.ratings {
            if !identifier(name)
                || !identifier(&rating.conditions)
                || rating.nodes[0] == rating.nodes[1]
                || rating
                    .nodes
                    .iter()
                    .any(|node| !self.pins.contains_key(node))
            {
                return Err(invalid(format!(
                    "предел «{name}»: требуются два разных вывода и условия"
                )));
            }
            rating
                .limit
                .validate(|v| v >= 0.0)
                .map_err(|e| invalid(format!("предел «{name}»: {e}")))?;
        }
        if self.kind == PartKind::PolarCapacitor
            && !self
                .elements
                .values()
                .filter(|e| e.kind == PrimitiveKind::Capacitor)
                .any(|cap| {
                    self.ratings.values().any(|r| {
                        r.quantity == Quantity::ReverseVoltageV
                            && r.nodes.as_slice() == cap.nodes.as_slice()
                    })
                })
        {
            return Err(invalid(
                "полярный конденсатор требует ориентированной ёмкости и обратного предела",
            ));
        }
        Ok(())
    }

    fn validate_element(&self, name: &str, element: &Primitive) -> Result<(), ProcessError> {
        let known =
            |node: &String| self.pins.contains_key(node) || self.internal_nodes.contains(node);
        if !identifier(name)
            || element.nodes.len() != element.kind.node_count()
            || element.nodes.iter().any(|node| !known(node))
            || element.nodes[0] == element.nodes[1]
            || (element.continuity && element.kind != PrimitiveKind::Resistor)
        {
            return Err(invalid(
                "недопустимые выводы или модель проводящего участка",
            ));
        }
        let keys = element.kind.parameters();
        if keys.len() != element.parameters.len()
            || keys
                .iter()
                .any(|key| !element.parameters.contains_key(*key))
        {
            return Err(invalid(format!("ожидались параметры {}", keys.join(", "))));
        }
        for (key, param) in &element.parameters {
            param
                .validate(|value| PrimitiveKind::accepts(key, value))
                .map_err(|e| invalid(format!("{key}: {e}")))?;
        }
        if element.kind == PrimitiveKind::Switch
            && let (Some(on), Some(off)) = (
                &element.parameters["on_ohm"].range,
                &element.parameters["off_ohm"].range,
            )
            && on.max > off.min
        {
            return Err(invalid(
                "сопротивление включённого ключа выше сопротивления выключенного",
            ));
        }
        Ok(())
    }
}
