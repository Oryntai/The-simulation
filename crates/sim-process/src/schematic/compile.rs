use std::collections::{BTreeMap, BTreeSet};

use sim_circuit::netlist::{
    Analysis as CircuitAnalysis, Element, ElementKind, ModelCard, ModelKind, Netlist, Source,
    SwitchControl,
};
use sim_circuit::{Circuit, SimOptions, operating_point_with};

use crate::{Analysis, ProcessError};

use super::{
    BindingTarget, Catalog, ContactKind, Edge, Endpoint, Parameter, PinRole, Primitive,
    PrimitiveKind, Schematic, Section, identifier, invalid, path,
};

#[derive(Clone, Debug)]
pub(super) struct Branch {
    pub section: Section,
    pub nodes: Vec<usize>,
    pub element: Option<ElementKind>,
    pub gap: Option<String>,
    pub capacitor: bool,
}

#[derive(Clone, Debug)]
pub struct CompiledSchematic {
    pub(super) schematic: Schematic,
    pub(super) catalog: Catalog,
    pub(super) nodes: Vec<Endpoint>,
    pub(super) index: BTreeMap<Endpoint, usize>,
    pub(super) graph: Vec<Vec<Edge>>,
    pub(super) order: Vec<String>,
    pub(super) mismatches: BTreeSet<String>,
    branches: Vec<Branch>,
    models: Vec<ModelCard>,
    gaps: Vec<(Vec<usize>, String)>,
}

impl CompiledSchematic {
    pub fn pins(&self) -> impl Iterator<Item = &Endpoint> {
        self.index.keys().filter(|end| {
            let component = &self.schematic.components[&end.component];
            self.catalog
                .part(&component.part)
                .unwrap()
                .pins
                .contains_key(&end.pin)
        })
    }

    pub(super) fn pin(&self, end: &Endpoint) -> &super::Pin {
        let component = &self.schematic.components[&end.component];
        &self.catalog.part(&component.part).unwrap().pins[&end.pin]
    }

    pub(super) fn index_of_pin(&self, end: &Endpoint) -> Result<usize, ProcessError> {
        if let Some(&index) = self.index.get(end)
            && self
                .catalog
                .part(&self.schematic.components[&end.component].part)
                .unwrap()
                .pins
                .contains_key(&end.pin)
        {
            return Ok(index);
        }
        Err(invalid(format!("неизвестный вывод «{end}»")))
    }

    pub(super) fn route(&self, a: usize, b: usize, assured: bool) -> Option<Vec<super::PathStep>> {
        self.route_avoiding(a, b, assured, &BTreeSet::new())
    }

    pub(super) fn route_avoiding(
        &self,
        a: usize,
        b: usize,
        assured: bool,
        excluded: &BTreeSet<Section>,
    ) -> Option<Vec<super::PathStep>> {
        path(&self.graph, a, b, assured, excluded).map(|steps| {
            steps
                .into_iter()
                .map(|(from, edge)| super::PathStep {
                    section: edge.section.clone(),
                    from: self.nodes[from].clone(),
                    to: self.nodes[edge.to].clone(),
                })
                .collect()
        })
    }

    /// Сверка с контактами и функциями, разрешёнными установщиком по каталогу и конфигурации.
    pub fn validate_bindings(
        &self,
        installed: &BTreeSet<BindingTarget>,
    ) -> Result<(), ProcessError> {
        let declared: BTreeSet<_> = self
            .schematic
            .bindings
            .iter()
            .map(|b| b.target.clone())
            .collect();
        if declared != *installed {
            return Err(invalid(format!(
                "привязки не совпали с установкой: лишние {:?}, отсутствуют {:?}",
                declared.difference(installed).collect::<Vec<_>>(),
                installed.difference(&declared).collect::<Vec<_>>()
            )));
        }
        Ok(())
    }

    /// Номинальный нетлист, с обратной картой ветвей и выводов. Пробелы блокируют экспорт.
    /// Для проверки DC на развязанных островах предназначен `dc`.
    pub fn netlist(&self) -> Analysis<ExpandedNetlist> {
        if let Some(branch) = self.branches.iter().find(|branch| branch.element.is_none()) {
            return Analysis::NotAnalyzed {
                reason: branch.gap.clone().unwrap(),
            };
        }
        let branches = self.branches.iter().collect::<Vec<_>>();
        let Some(anchor) = branches.first().map(|branch| branch.nodes[0]) else {
            return Analysis::NotAnalyzed {
                reason: "нет проводящих или реактивных ветвей".into(),
            };
        };
        Analysis::Analyzed {
            value: {
                let mut expanded = self.expand(&branches, anchor);
                expanded.netlist.warnings = self
                    .gaps
                    .iter()
                    .map(|(_, gap)| gap.clone())
                    .chain(branches.iter().filter_map(|branch| branch.gap.clone()))
                    .collect();
                expanded
            },
            scope: "номинальный эквивалент схемы".into(),
        }
    }

    fn expand(&self, branches: &[&Branch], anchor: usize) -> ExpandedNetlist {
        let mut origins = BTreeMap::new();
        let mut nodes = BTreeMap::new();
        let elements = branches
            .iter()
            .enumerate()
            .filter_map(|(i, branch)| {
                let kind = branch.element.clone()?;
                let name = format!("{}{}", prefix(&kind), i);
                origins.insert(name.clone(), branch.section.clone());
                Some(Element {
                    name,
                    nodes: branch
                        .nodes
                        .iter()
                        .map(|&node| {
                            let name = node_name(node, anchor);
                            nodes.insert(self.nodes[node].clone(), name.clone());
                            name
                        })
                        .collect(),
                    kind,
                    // Элементы получены из типизированных данных, строки SPICE нет.
                    line: 0,
                })
            })
            .collect();
        ExpandedNetlist {
            netlist: Netlist {
                title: self.schematic.name.clone(),
                elements,
                models: self.models.clone(),
                analyses: vec![CircuitAnalysis::Op],
                warnings: Vec::new(),
            },
            nodes,
            origins,
        }
    }

    /// Разности напряжений доступны только между точками одного DC-острова.
    pub fn dc(&self) -> DcReport {
        let mut groups = Islands::new(self.nodes.len());
        for branch in &self.branches {
            if !branch.capacitor && branch.element.is_some() {
                for &node in &branch.nodes[1..] {
                    groups.join(branch.nodes[0], node);
                }
            }
        }

        let island = (0..self.nodes.len())
            .map(|node| groups.root(node))
            .collect::<Vec<_>>();
        let mut gaps = BTreeMap::new();
        let mut candidates: BTreeMap<usize, Vec<&Branch>> = BTreeMap::new();
        for branch in &self.branches {
            if let Some(gap) = &branch.gap {
                for &node in &branch.nodes {
                    gaps.entry(island[node]).or_insert_with(|| gap.clone());
                }
            }
            if !branch.capacitor && branch.element.is_some() {
                candidates
                    .entry(island[branch.nodes[0]])
                    .or_default()
                    .push(branch);
            }
        }
        for (nodes, gap) in &self.gaps {
            for &node in nodes {
                gaps.entry(island[node]).or_insert_with(|| gap.clone());
            }
        }

        let mut potentials = BTreeMap::new();
        let mut branches = Vec::new();
        for (root, entries) in candidates {
            let expanded = self.expand(&entries, root);
            let options = SimOptions {
                // Выбор опоры только задаёт координату. Численные шунты не замыкают цепь.
                gshunt: 0.0,
                gmin: 0.0,
                ..SimOptions::default()
            };
            let point = Circuit::from_netlist(&expanded.netlist)
                .and_then(|circuit| operating_point_with(&circuit, options));
            let calculated = point.map_err(|e| e.to_string()).and_then(|point| {
                let values = expanded
                    .nodes
                    .iter()
                    .map(|(end, name)| {
                        let value = point.voltage(name).map_err(|e| e.to_string())?;
                        finite(value).map(|value| (end.clone(), (root, value)))
                    })
                    .collect::<Result<Vec<_>, _>>()?;
                let readings = expanded
                    .origins
                    .into_iter()
                    .map(|(name, section)| {
                        Ok(BranchReading {
                            section,
                            current_a: finite(point.current(&name).map_err(|e| e.to_string())?)?,
                            power_w: finite(point.power(&name).map_err(|e| e.to_string())?)?,
                        })
                    })
                    .collect::<Result<Vec<_>, String>>()?;
                Ok((values, readings))
            });
            match calculated {
                Ok((values, readings)) => {
                    potentials.extend(values);
                    branches.extend(readings);
                }
                Err(error) => {
                    gaps.insert(root, format!("рабочая точка: {error}"));
                }
            }
        }
        // Идеальный конденсатор разомкнут в установившемся DC. Его напряжение
        // выдаётся только там, где разность потенциалов уже определена цепью.
        for branch in self
            .branches
            .iter()
            .filter(|branch| branch.capacitor && branch.element.is_some())
        {
            branches.push(BranchReading {
                section: branch.section.clone(),
                current_a: 0.0,
                power_w: 0.0,
            });
        }
        DcReport {
            potentials,
            gaps,
            islands: self
                .index
                .iter()
                .map(|(end, &i)| (end.clone(), island[i]))
                .collect(),
            branches,
        }
    }
}

#[derive(Clone, Debug)]
pub struct ExpandedNetlist {
    pub netlist: Netlist,
    pub nodes: BTreeMap<Endpoint, String>,
    pub origins: BTreeMap<String, Section>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BranchReading {
    pub section: Section,
    pub current_a: f64,
    pub power_w: f64,
}

#[derive(Clone, Debug)]
pub struct DcReport {
    potentials: BTreeMap<Endpoint, (usize, f64)>,
    gaps: BTreeMap<usize, String>,
    islands: BTreeMap<Endpoint, usize>,
    branches: Vec<BranchReading>,
}

impl DcReport {
    /// Номинальные токи и мощности элементов, положительное направление по порядку выводов.
    pub fn branches(&self) -> &[BranchReading] {
        &self.branches
    }

    /// Возвращает номинальное измерение даже при открытой огибающей параметров.
    pub fn voltage(&self, plus: &Endpoint, minus: &Endpoint) -> Analysis<f64> {
        if let (Some(&(a, v)), Some(&(b, w))) =
            (self.potentials.get(plus), self.potentials.get(minus))
            && a == b
            && let Ok(value) = finite(v - w)
        {
            return Analysis::Analyzed {
                value,
                scope: "номинальное установившееся DC".into(),
            };
        }
        Analysis::NotAnalyzed {
            reason: [plus, minus]
                .into_iter()
                .find_map(|end| self.islands.get(end).and_then(|i| self.gaps.get(i)))
                .cloned()
                .unwrap_or_else(|| {
                    "разность потенциалов не определена: разные DC-острова или плавающий вывод"
                        .into()
                }),
        }
    }

    pub(super) fn proof_gap(&self, end: &Endpoint) -> Option<&str> {
        self.islands
            .get(end)
            .and_then(|i| self.gaps.get(i))
            .map(String::as_str)
    }
}

fn finite(value: f64) -> Result<f64, String> {
    value
        .is_finite()
        .then_some(value)
        .ok_or_else(|| "решатель вернул неконечное число".into())
}

fn node_name(index: usize, anchor: usize) -> String {
    if index == anchor {
        "0".into()
    } else {
        format!("n{index}")
    }
}

struct Islands(Vec<usize>);

impl Islands {
    fn new(count: usize) -> Self {
        Self((0..count).collect())
    }

    fn root(&mut self, mut x: usize) -> usize {
        while self.0[x] != x {
            self.0[x] = self.0[self.0[x]];
            x = self.0[x];
        }
        x
    }

    fn join(&mut self, a: usize, b: usize) {
        let a = self.root(a);
        let b = self.root(b);
        self.0[a.max(b)] = a.min(b);
    }
}

/// Проверяет ссылки и сохраняет только физически совместимые участки.
pub fn compile(schematic: Schematic, catalog: &Catalog) -> Result<CompiledSchematic, ProcessError> {
    if !identifier(&schematic.name)
        || schematic.components.is_empty()
        || schematic.nets.iter().any(|net| !identifier(net))
    {
        return Err(invalid(
            "требуются имя схемы, компоненты и непустые имена цепей",
        ));
    }
    let order = super::assembly::order(&schematic.connections)?;
    let mut result = CompiledSchematic {
        schematic,
        catalog: catalog.clone(),
        nodes: Vec::new(),
        index: BTreeMap::new(),
        graph: Vec::new(),
        order,
        mismatches: BTreeSet::new(),
        branches: Vec::new(),
        models: Vec::new(),
        gaps: Vec::new(),
    };
    result.read_components()?;
    result.read_connections()?;
    result.validate_rails_and_bindings()?;
    Ok(result)
}

impl CompiledSchematic {
    fn read_components(&mut self) -> Result<(), ProcessError> {
        for (id, component) in &self.schematic.components {
            let part = self.catalog.part(&component.part).ok_or_else(|| {
                invalid(format!(
                    "компонент «{id}»: неизвестная позиция «{}»",
                    component.part
                ))
            })?;
            if !identifier(id)
                || id.contains('/')
                || (part.reference_only && self.schematic.scope != super::Scope::Reference)
            {
                return Err(invalid(format!(
                    "компонент «{id}»: недопустимое имя или эталон вне корпуса"
                )));
            }
            if component.nets.len() != part.pins.len()
                || part.pins.keys().any(|pin| {
                    component
                        .nets
                        .get(pin)
                        .is_none_or(|net| !self.schematic.nets.contains(net))
                })
            {
                return Err(invalid(format!(
                    "компонент «{id}»: каждый каталожный вывод должен иметь объявленную цепь"
                )));
            }
            let first = self.nodes.len();
            for pin in part.pins.keys().chain(&part.internal_nodes) {
                let end = Endpoint {
                    component: id.clone(),
                    pin: pin.clone(),
                };
                self.index.insert(end.clone(), self.nodes.len());
                self.nodes.push(end);
                self.graph.push(Vec::new());
            }
            if let Some(gap) = &part.model_gap {
                let returns = part
                    .pins
                    .iter()
                    .filter(|(_, pin)| pin.role == PinRole::Return)
                    .map(|(name, _)| {
                        self.index[&Endpoint {
                            component: id.clone(),
                            pin: name.clone(),
                        }]
                    })
                    .collect::<Vec<_>>();
                // С неполной моделью общий возврат внутри детали возможен, но не доказан.
                for pair in returns.windows(2) {
                    let section = Section::Element {
                        component: id.clone(),
                        element: "неописанный возврат".into(),
                    };
                    add_edge(&mut self.graph, pair[0], pair[1], &section, false);
                }
                self.gaps
                    .push(((first..self.nodes.len()).collect(), format!("{id}: {gap}")));
            }
            for (name, model) in &part.elements {
                let section = Section::Element {
                    component: id.clone(),
                    element: name.clone(),
                };
                let nodes = model
                    .nodes
                    .iter()
                    .map(|pin| {
                        self.index[&Endpoint {
                            component: id.clone(),
                            pin: pin.clone(),
                        }]
                    })
                    .collect::<Vec<_>>();
                let gap = parameter_gap(self.schematic.scope, &model.parameters)
                    .or_else(|| {
                        // В эталонном ключе без гистерезиса состояние в DC
                        // определяет решённое управляющее напряжение.
                        (model.kind == PrimitiveKind::Switch
                            && !(self.schematic.scope == super::Scope::Reference
                                && model.parameters["hysteresis_v"].range.as_ref().is_some_and(
                                    |range| self.schematic.scope.exact(range) && range.nominal == 0.0,
                                )))
                        .then(|| {
                            "для ключа требуется проверка исходного состояния и гистерезиса".into()
                        })
                    })
                    .map(|gap| format!("«{id}/{name}»: {gap}"));
                let element = if model.parameters.values().all(|p| p.range.is_some()) {
                    Some(reveal(model, &mut self.models))
                } else {
                    None
                };
                let conditional = matches!(
                    model.kind,
                    PrimitiveKind::Switch | PrimitiveKind::Nmos | PrimitiveKind::Pmos
                );
                if model.continuity || conditional {
                    let other = if matches!(model.kind, PrimitiveKind::Nmos | PrimitiveKind::Pmos) {
                        2
                    } else {
                        1
                    };
                    add_edge(
                        &mut self.graph,
                        nodes[0],
                        nodes[other],
                        &section,
                        model.continuity,
                    );
                }
                self.branches.push(Branch {
                    section,
                    nodes,
                    element,
                    gap,
                    capacitor: model.kind == PrimitiveKind::Capacitor,
                });
            }
        }
        Ok(())
    }

    fn read_connections(&mut self) -> Result<(), ProcessError> {
        let mut pairs = BTreeSet::new();
        let mut plugged = BTreeSet::new();
        for (id, joint) in &self.schematic.connections {
            let [a, b] = joint.ends.each_ref().map(|end| self.index_of_pin(end));
            let (a, b) = (a?, b?);
            if a == b || !pairs.insert((a.min(b), a.max(b))) {
                return Err(invalid(format!(
                    "«{id}»: повтор или соединение вывода с собой"
                )));
            }
            if joint.ends.iter().any(|end| {
                let kind = self.pin(end).contact.kind;
                matches!(kind, ContactKind::Plug | ContactKind::Socket) && !plugged.insert(end)
            }) {
                return Err(invalid(format!(
                    "«{id}»: один контакт разъёма занят дважды"
                )));
            }
            let net = |end: &Endpoint| &self.schematic.components[&end.component].nets[&end.pin];
            if net(&joint.ends[0]) != net(&joint.ends[1]) {
                return Err(invalid(format!(
                    "«{id}»: концы должны принадлежать одной объявленной цепи"
                )));
            }
            joint
                .resistance_ohm
                .validate(|r| r > 0.0 && r.recip().is_finite())
                .map_err(|e| invalid(format!("«{id}».resistance_ohm: {e}")))?;
            joint
                .current_limit_a
                .validate(|v| v > 0.0)
                .map_err(|e| invalid(format!("«{id}».current_limit_a: {e}")))?;
            if !joint.kind.mates(
                &self.pin(&joint.ends[0]).contact,
                &self.pin(&joint.ends[1]).contact,
            ) {
                self.mismatches.insert(id.clone());
                continue;
            }
            let section = Section::Connection(id.clone());
            let gap = scalar_gap(self.schematic.scope, &joint.resistance_ohm)
                .map(|gap| format!("«{id}»: {gap}"));
            add_edge(&mut self.graph, a, b, &section, true);
            self.branches.push(Branch {
                section,
                nodes: vec![a, b],
                element: joint
                    .resistance_ohm
                    .range
                    .as_ref()
                    .map(|r| ElementKind::Resistor { r: r.nominal }),
                gap,
                capacitor: false,
            });
        }
        Ok(())
    }

    fn validate_rails_and_bindings(&self) -> Result<(), ProcessError> {
        for (name, rail) in &self.schematic.rails {
            if !identifier(name) {
                return Err(invalid("пустое имя шины"));
            }
            self.supply_ends(&rail.ends)?;
            rail.reverse_limit_v
                .validate(|v| v >= 0.0)
                .map_err(|e| invalid(format!("шина «{name}»: {e}")))?;
        }
        let mut targets = BTreeSet::new();
        let mut contacts = BTreeSet::new();
        let mut endpoints = BTreeSet::new();
        for binding in &self.schematic.bindings {
            if !binding.target.valid() || !targets.insert(&binding.target) {
                return Err(invalid("некорректная или повторная привязка"));
            }
            match (&binding.target, binding.ends.as_slice()) {
                (
                    BindingTarget::BoardPin {
                        board,
                        pin,
                        function,
                    },
                    [end],
                ) => {
                    self.index_of_pin(end)?;
                    let role = &self.pin(end).role;
                    let compatible = match function {
                        super::PinFunction::GpioInput | super::PinFunction::Adc => role.receives(),
                        super::PinFunction::GpioOutput | super::PinFunction::Pwm => role.sends(),
                        _ => role.reference().is_some(),
                    };
                    if !compatible || !contacts.insert((board, pin)) || !endpoints.insert(end) {
                        return Err(invalid(format!(
                            "контакт «{board}/{pin}»: роль вывода не совпала или контакт занят дважды"
                        )));
                    }
                }
                (BindingTarget::BoardPin { .. }, _) => {
                    return Err(invalid("контакт платы требует один вывод"));
                }
                (_, [plus, minus]) => self.supply_ends(&[plus.clone(), minus.clone()])?,
                _ => return Err(invalid("питание требует пару «плюс, возврат»")),
            }
        }
        Ok(())
    }

    fn supply_ends(&self, ends: &[Endpoint; 2]) -> Result<(), ProcessError> {
        self.index_of_pin(&ends[0])?;
        self.index_of_pin(&ends[1])?;
        if self.pin(&ends[0]).role != PinRole::Power || self.pin(&ends[1]).role != PinRole::Return {
            return Err(invalid(
                "ожидались ориентированные выводы питания «плюс, возврат»",
            ));
        }
        Ok(())
    }
}

fn add_edge(graph: &mut [Vec<Edge>], a: usize, b: usize, section: &Section, assured: bool) {
    graph[a].push(Edge {
        section: section.clone(),
        to: b,
        assured,
    });
    graph[b].push(Edge {
        section: section.clone(),
        to: a,
        assured,
    });
}

fn scalar_gap(scope: super::Scope, parameter: &Parameter) -> Option<String> {
    if parameter
        .range
        .as_ref()
        .is_some_and(|range| scope.exact(range))
    {
        None
    } else {
        Some(format!(
            "нет точного обоснованного значения, замер: {}",
            parameter.measurement
        ))
    }
}

fn parameter_gap(scope: super::Scope, parameters: &BTreeMap<String, Parameter>) -> Option<String> {
    let gaps = parameters
        .iter()
        .filter_map(|(name, p)| scalar_gap(scope, p).map(|gap| format!("{name}: {gap}")))
        .collect::<Vec<_>>();
    (!gaps.is_empty()).then(|| gaps.join("; "))
}

fn reveal(model: &Primitive, cards: &mut Vec<ModelCard>) -> ElementKind {
    use PrimitiveKind::*;
    let value = |key: &str| model.parameters[key].range.as_ref().unwrap().nominal;
    let mut card = |kind, params: &[(&str, f64)]| {
        let name = format!("model{}", cards.len());
        cards.push(ModelCard {
            name: name.clone(),
            kind,
            params: params.iter().map(|(k, v)| ((*k).into(), *v)).collect(),
            line: 0,
        });
        name
    };
    match model.kind {
        Resistor => ElementKind::Resistor { r: value("ohm") },
        Capacitor => ElementKind::Capacitor {
            c: value("farad"),
            ic: None,
        },
        Inductor => ElementKind::Inductor {
            l: value("henry"),
            ic: None,
        },
        Voltage | Current => {
            let src = Source {
                dc: Some(value(if model.kind == Voltage {
                    "volt"
                } else {
                    "ampere"
                })),
                wave: None,
            };
            if model.kind == Voltage {
                ElementKind::VoltageSource(src)
            } else {
                ElementKind::CurrentSource(src)
            }
        }
        Diode => ElementKind::Diode {
            model: card(
                ModelKind::Diode,
                &[
                    ("is", value("is_a")),
                    ("n", value("ideality")),
                    ("rs", value("series_ohm")),
                ],
            ),
            // Модель описывает один экземпляр с паспортной площадью.
            area: 1.0,
        },
        Nmos | Pmos => ElementKind::Mosfet {
            model: card(
                if model.kind == Nmos {
                    ModelKind::Nmos
                } else {
                    ModelKind::Pmos
                },
                &[
                    ("vto", value("threshold_v")),
                    ("kp", value("kp_a_per_v2")),
                    ("lambda", value("lambda_per_v")),
                    ("is", value("is_a")),
                ],
            ),
            w: Some(value("width_m")),
            l: Some(value("length_m")),
        },
        Switch => ElementKind::Switch {
            control: SwitchControl::Voltage,
            model: Some(card(
                ModelKind::Switch,
                &[("vt", value("threshold_v")), ("vh", value("hysteresis_v"))],
            )),
            // Статическое раскрытие не доказывает состояние в полосе гистерезиса.
            initially_on: false,
            ron: Some(value("on_ohm")),
            roff: Some(value("off_ohm")),
        },
        Vcvs => ElementKind::Vcvs {
            gain: value("gain"),
        },
        Vccs => ElementKind::Vccs {
            gm: value("siemens"),
        },
    }
}

fn prefix(kind: &ElementKind) -> char {
    match kind {
        ElementKind::Resistor { .. } => 'r',
        ElementKind::Capacitor { .. } => 'c',
        ElementKind::Inductor { .. } => 'l',
        ElementKind::VoltageSource(_) => 'v',
        ElementKind::CurrentSource(_) => 'i',
        ElementKind::Diode { .. } => 'd',
        ElementKind::Mosfet { .. } => 'm',
        ElementKind::Switch { .. } => 's',
        ElementKind::Vcvs { .. } => 'e',
        ElementKind::Vccs { .. } => 'g',
    }
}
