//! Сверка выводов схемы по фактической меди файла KiCad 8/9. Независимые
//! надписи цепей используются после построения геометрических контактов.

use std::collections::{BTreeMap, BTreeSet};

use sim_circuit::pcb::kicad::{self, KiCadBoard, PadKind};

use super::{CompiledSchematic, Endpoint, identifier, invalid};
use crate::{Analysis, ProcessError};

// До публичного экспорта Layout в 4.9 используется тот же нормализатор,
// что и в тестах 4.2. Его типы не выходят через этот интерфейс.
#[allow(dead_code)]
#[path = "../../../sim-circuit/src/pcb/layout.rs"]
mod layout;

use layout::{Centerline, Layout, Polygon, RoundedRect};

/// Адрес логической клеммы в файле платы. Один адрес может иметь несколько
/// физических площадок, например контактные поверхности одного вывода.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct PadAddress {
    pub reference: String,
    pub pad: String,
}

impl std::fmt::Display for PadAddress {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}/{}", self.reference, self.pad)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PadBinding {
    pub address: PadAddress,
    pub endpoint: Endpoint,
}

/// Индексы с нуля в исходной плате. Для зоны region — порядковый номер
/// сохранённого полигона на слое, указанном у медной области.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CopperSource {
    Track(usize),
    Via(usize),
    Pad { footprint: usize, pad: usize },
    Fill { zone: usize, region: usize },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CopperPiece {
    pub source: CopperSource,
    pub layer: String,
    /// Номер надписи цепи KiCad. Медь без надписи тоже участвует в графе.
    pub net: Option<u32>,
}

impl std::fmt::Display for CopperPiece {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.source {
            CopperSource::Track(i) => write!(f, "дорожка {}", i + 1)?,
            CopperSource::Via(i) => write!(f, "переход {}", i + 1)?,
            CopperSource::Pad { footprint, pad } => {
                write!(
                    f,
                    "площадка {} посадочного места {}",
                    pad + 1,
                    footprint + 1
                )?;
            }
            CopperSource::Fill { zone, region } => {
                write!(f, "полигон {} зоны {}", region + 1, zone + 1)?
            }
        }
        write!(f, " на {}", self.layer)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CopperTerminal {
    pub endpoint: Endpoint,
    pub pieces: Vec<usize>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CopperGraph {
    pub pieces: Vec<CopperPiece>,
    /// Неориентированные контакты. Пары индексов отсортированы.
    /// Межслойный контакт означает заявленную металлизацию отверстия.
    pub contacts: Vec<[usize; 2]>,
    /// Связные группы, заданные индексами областей меди.
    pub groups: Vec<Vec<usize>>,
    pub terminals: BTreeMap<PadAddress, CopperTerminal>,
}

/// Требуется явная привязка всех площадок платы с назначенной цепью.
/// Выводы схемы вне этой платы могут отсутствовать в списке привязок.
/// Для каждой цепи все привязанные площадки должны прозваниваться через
/// медь платы. Несовпадение имён, короткое или доказанный обрыв — ошибка
/// входа. Связность относится к геометрии файла и заявленным проходам.
pub fn check_pcb_map(
    board: &KiCadBoard,
    schematic: &CompiledSchematic,
    bindings: &[PadBinding],
) -> Result<Analysis<CopperGraph>, ProcessError> {
    if bindings.is_empty() {
        return Err(invalid(
            "требуется соответствие хотя бы одной клеммы платы выводу схемы",
        ));
    }
    let layout = Layout::from_kicad(board).map_err(|e| invalid(format!("плата: {e}")))?;
    let mut net_names = BTreeSet::new();
    if board.nets.get(&0).is_some_and(|name| !name.is_empty())
        || layout
            .nets
            .values()
            .any(|name| !identifier(name) || !net_names.insert(name))
    {
        return Err(invalid("плата: недопустимое или повторное имя цепи"));
    }
    // Проверка клемм ниже не видит назначенных цепей у отдельной меди.
    for id in layout
        .tracks
        .iter()
        .filter_map(|track| track.net)
        .chain(layout.vias.iter().filter_map(|via| via.net))
        .chain(layout.zones.iter().filter_map(|zone| zone.net))
    {
        let name = &layout.nets[&id];
        if !schematic.schematic.nets.contains(name) {
            return Err(invalid(format!("цепь меди «{name}» отсутствует в схеме")));
        }
    }
    let mut copper = Copper::new(&layout);
    let terminals = copper.read_pads(&layout, schematic, bindings)?;
    copper.read_routes(&layout);
    if let Err(reason) = copper.check_geometry(&layout) {
        return Ok(Analysis::NotAnalyzed { reason });
    }
    copper.find_contacts();
    let (roots, groups) = components(copper.pieces.len(), &copper.contacts);
    check_circuits(&layout, &copper, &terminals, &roots, &groups)?;
    if !copper.gaps.is_empty() {
        return Ok(Analysis::NotAnalyzed {
            reason: copper.gaps.into_iter().collect::<Vec<_>>().join("; "),
        });
    }
    Ok(Analysis::Analyzed {
        value: CopperGraph {
            pieces: copper.pieces,
            contacts: copper.contacts.into_iter().collect(),
            groups,
            terminals,
        },
        scope: "геометрия записанной меди и заявленных металлизированных проходов".into(),
    })
}

#[derive(Default)]
struct Copper<'a> {
    pieces: Vec<CopperPiece>,
    shapes: Vec<Shape>,
    layers: BTreeMap<&'a str, Vec<usize>>,
    drills: Vec<Drill>,
    contacts: BTreeSet<[usize; 2]>,
    gaps: BTreeSet<String>,
}

struct Drill {
    address: String,
    shape: Shape,
    layers: Vec<String>,
    lands: Vec<usize>,
    plated: bool,
}

impl<'a> Copper<'a> {
    fn new(layout: &'a Layout) -> Self {
        Self {
            layers: layout
                .copper_layers
                .iter()
                .map(|s| (s.as_str(), Vec::new()))
                .collect(),
            ..Self::default()
        }
    }

    fn push(&mut self, source: CopperSource, layer: &'a str, net: Option<u32>, shape: Shape) {
        self.layers.get_mut(layer).unwrap().push(self.pieces.len());
        self.pieces.push(CopperPiece {
            source,
            layer: layer.into(),
            net,
        });
        self.shapes.push(shape);
    }

    fn read_pads(
        &mut self,
        layout: &'a Layout,
        schematic: &CompiledSchematic,
        bindings: &[PadBinding],
    ) -> Result<BTreeMap<PadAddress, CopperTerminal>, ProcessError> {
        let mut terminals = BTreeMap::new();
        let mut endpoints = BTreeSet::new();
        for binding in bindings {
            let address = &binding.address;
            let endpoint = &binding.endpoint;
            schematic.index_of_pin(endpoint)?;
            if !identifier(&address.reference) || !identifier(&address.pad) {
                return Err(invalid(format!("неполный адрес площадки «{address}»")));
            }
            if terminals.contains_key(address) || !endpoints.insert(endpoint) {
                return Err(invalid(format!("повтор привязки {address} к {endpoint}")));
            }
            terminals.insert(
                address.clone(),
                CopperTerminal {
                    endpoint: endpoint.clone(),
                    pieces: Vec::new(),
                },
            );
        }
        let mut references = BTreeSet::new();
        for (footprint, fp) in layout.footprints.iter().enumerate() {
            if let Some(reference) = &fp.reference
                && (!identifier(reference) || !references.insert(reference))
            {
                return Err(invalid(format!(
                    "некорректный или повторный ref «{reference}»"
                )));
            }
            for (pad, item) in fp.pads.iter().enumerate() {
                let address = fp.reference.as_ref().map(|reference| PadAddress {
                    reference: reference.clone(),
                    pad: item.number.clone(),
                });
                let label = address.as_ref().map_or_else(
                    || format!("посадочное место {}, площадка {}", footprint + 1, pad + 1),
                    ToString::to_string,
                );
                let first = self.pieces.len();
                if let Some(land) = &item.land
                    && !(item.kind == PadKind::NonPlatedHole && item.hole == Some(land.outline))
                {
                    // Типичное крепёжное сверление KiCad имеет одинаковые size и drill.
                    // Нарисованный круг целиком вычтен отверстием и меди не оставляет.
                    for layer in &land.layers {
                        self.push(
                            CopperSource::Pad { footprint, pad },
                            layer,
                            item.net,
                            Shape::rounded(land.outline),
                        );
                    }
                }
                let lands = first..self.pieces.len();
                let terminal = address.as_ref().and_then(|a| terminals.get_mut(a));
                if let Some(terminal) = terminal {
                    let end = &terminal.endpoint;
                    let expected = &schematic.schematic.components[&end.component].nets[&end.pin];
                    if lands.is_empty()
                        || item.net.and_then(|net| layout.nets.get(&net)) != Some(expected)
                    {
                        let actual = item.net.and_then(|net| layout.nets.get(&net));
                        return Err(invalid(format!(
                            "{label} ↔ {end}: ожидается цепь «{expected}», в плате {}",
                            actual.map_or_else(
                                || "нет назначенной медной клеммы".into(),
                                |net| format!("цепь «{net}»")
                            ),
                        )));
                    }
                    terminal.pieces.extend(lands.clone());
                } else if item.net.is_some() {
                    return Err(invalid(format!(
                        "{label}: отсутствует привязка к выводу схемы"
                    )));
                }

                if let Some(hole) = item.hole {
                    self.drills.push(Drill {
                        address: label,
                        shape: Shape::rounded(hole),
                        layers: layout.copper_layers.clone(),
                        lands: lands.collect(),
                        plated: item.kind == PadKind::PlatedHole,
                    });
                }
            }
        }
        if let Some((address, terminal)) = terminals.iter().find(|(_, t)| t.pieces.is_empty()) {
            return Err(invalid(format!(
                "{address} ↔ {}: площадка отсутствует",
                terminal.endpoint
            )));
        }
        Ok(terminals)
    }

    fn read_routes(&mut self, layout: &'a Layout) {
        for (index, track) in layout.tracks.iter().enumerate() {
            self.push(
                CopperSource::Track(index),
                &track.layer,
                track.net,
                Shape::track(track.path, track.width_m / 2.0),
            );
        }
        for (index, via) in layout.vias.iter().enumerate() {
            let first = self.pieces.len();
            let center = via.at.metres();
            for layer in &via.layers {
                self.push(
                    CopperSource::Via(index),
                    layer,
                    via.net,
                    Shape::circle(center, via.diameter_m / 2.0),
                );
            }
            self.drills.push(Drill {
                address: format!("переход {}", index + 1),
                shape: Shape::circle(center, via.drill_m / 2.0),
                layers: via.layers.clone(),
                lands: (first..self.pieces.len()).collect(),
                plated: true,
            });
        }
        for (zone, item) in layout.zones.iter().enumerate() {
            for layer in item.layers() {
                match item.fill(layer) {
                    Ok(fill) => {
                        for (region, area) in fill.iter().enumerate() {
                            self.push(
                                CopperSource::Fill { zone, region },
                                layer,
                                item.net,
                                Shape::polygon(&area.polygon),
                            );
                        }
                    }
                    Err(reason) => {
                        self.gaps.insert(format!("зона {}: {reason}", zone + 1));
                    }
                }
            }
        }
    }

    fn check_geometry(&self, layout: &Layout) -> Result<(), String> {
        let substrate = layout.polygonal_substrate().map_err(|e| e.to_string())?;
        let substrate = substrate.iter().map(Shape::polygon).collect::<Vec<_>>();
        if let Some((i, _)) = self.shapes.iter().enumerate().find(|(_, shape)| {
            // Центр окружности очень пологой дуги может выйти за точность
            // исходной сетки. В этом случае округление не подтверждает контакт.
            shape.curves.iter().any(|curve| {
                matches!(curve, Curve::Circle(c, r)
                    if (c[0].abs().max(c[1].abs()) + r) * f64::EPSILON >= GRID)
            })
        }) {
            return Err(format!(
                "{}: координаты дуги вне точности сетки",
                self.pieces[i]
            ));
        }
        for (i, shape) in self.shapes.iter().enumerate() {
            if relation(shape, None, substrate.iter()) != Relation::Apart {
                return Err(format!("{} у границы Edge.Cuts или за ней", self.pieces[i]));
            }
        }

        for (i, drill) in self.drills.iter().enumerate() {
            if relation(&drill.shape, None, substrate.iter()) != Relation::Apart {
                return Err(format!(
                    "{}: сверление у границы Edge.Cuts или за ней",
                    drill.address
                ));
            }
            for other in &self.drills[..i] {
                if drill
                    .layers
                    .iter()
                    .any(|layer| other.layers.contains(layer))
                    && relation(&drill.shape, Some(&other.shape), []) != Relation::Apart
                {
                    return Err(format!(
                        "пересечение отверстий «{}» и «{}»",
                        drill.address, other.address
                    ));
                }
            }
            if drill.plated {
                // Полное кольцо позволяет обойти отверстие в любой меди,
                // вышедшей к нему; граф сохраняет эту связь через саму площадку.
                for &land in &drill.lands {
                    if relation(&drill.shape, None, [&self.shapes[land]]) != Relation::Apart {
                        return Err(format!(
                            "{}: не подтверждено сплошное кольцо на {}",
                            drill.address, self.pieces[land].layer
                        ));
                    }
                }
            }
            for layer in &drill.layers {
                let landed = drill
                    .lands
                    .iter()
                    .any(|&land| &self.pieces[land].layer == layer);
                for &id in &self.layers[layer.as_str()] {
                    let shape = &self.shapes[id];
                    if relation(shape, Some(&drill.shape), []) == Relation::Apart {
                        continue;
                    }
                    if relation(shape, None, [&drill.shape]) != Relation::Overlap {
                        return Err(format!(
                            "{}: не подтверждена медь вне сверления ({})",
                            drill.address, self.pieces[id]
                        ));
                    }
                    if !(drill.plated && landed) {
                        // Без замкнутого кольца разрез может разделить медь.
                        // Отверстие целиком внутри связной области её не разделяет.
                        if drill.plated || relation(&drill.shape, None, [shape]) != Relation::Apart
                        {
                            return Err(format!(
                                "{}: сверление пересекает медь без подтверждённого обхода ({})",
                                drill.address, self.pieces[id]
                            ));
                        }
                    }
                }
            }
        }
        Ok(())
    }

    fn find_contacts(&mut self) {
        for drill in &self.drills {
            if drill.plated {
                self.contacts
                    .extend(drill.lands.windows(2).map(|pair| [pair[0], pair[1]]));
            }
        }
        for (&layer, indices) in &self.layers {
            let drills = self
                .drills
                .iter()
                .filter(|d| d.layers.iter().any(|l| l == layer))
                .map(|d| &d.shape)
                .collect::<Vec<_>>();
            let mut order = indices.clone();
            order.sort_by(|&a, &b| {
                self.shapes[a].bounds.min[0].total_cmp(&self.shapes[b].bounds.min[0])
            });
            for (i, &a) in order.iter().enumerate() {
                let first = &self.shapes[a];
                for &b in &order[i + 1..] {
                    let second = &self.shapes[b];
                    if second.bounds.min[0] > first.bounds.max[0] + GRID {
                        break;
                    }
                    match relation(first, Some(second), drills.iter().copied()) {
                        Relation::Overlap => {
                            self.contacts.insert([a.min(b), a.max(b)]);
                        }
                        Relation::Touch => {
                            self.gaps.insert(format!(
                                "касание в пределах сетки KiCad: {} и {}",
                                self.pieces[a], self.pieces[b]
                            ));
                        }
                        Relation::Apart => {}
                    }
                }
            }
        }
    }
}

fn components(count: usize, contacts: &BTreeSet<[usize; 2]>) -> (Vec<usize>, Vec<Vec<usize>>) {
    // Итеративный обход сохраняет результат для произвольной длины цепи.
    let mut neighbours = vec![Vec::new(); count];
    for &[a, b] in contacts {
        neighbours[a].push(b);
        neighbours[b].push(a);
    }
    let mut roots = vec![usize::MAX; count];
    let mut groups = Vec::new();
    for start in 0..count {
        if roots[start] != usize::MAX {
            continue;
        }
        let group = groups.len();
        roots[start] = group;
        let mut members = vec![start];
        let mut cursor = 0;
        while let Some(&id) = members.get(cursor) {
            for &next in &neighbours[id] {
                if roots[next] == usize::MAX {
                    roots[next] = group;
                    members.push(next);
                }
            }
            cursor += 1;
        }
        members.sort_unstable();
        groups.push(members);
    }
    (roots, groups)
}

fn check_circuits(
    layout: &Layout,
    copper: &Copper<'_>,
    terminals: &BTreeMap<PadAddress, CopperTerminal>,
    roots: &[usize],
    groups: &[Vec<usize>],
) -> Result<(), ProcessError> {
    let mut issues = Vec::new();
    for group in groups {
        let nets: BTreeSet<_> = group
            .iter()
            .filter_map(|&id| copper.pieces[id].net)
            .collect();
        if nets.len() > 1 {
            let names = nets
                .into_iter()
                .map(|id| layout.nets[&id].as_str())
                .collect::<Vec<_>>();
            let pads = terminals
                .iter()
                .filter(|(_, t)| t.pieces.iter().any(|&id| roots[id] == roots[group[0]]))
                .map(|(address, _)| address.to_string())
                .collect::<Vec<_>>();
            let context = if pads.is_empty() {
                format!("группа меди с участком «{}»", copper.pieces[group[0]])
            } else {
                format!("медная группа с клеммами {}", pads.join(", "))
            };
            issues.push(format!(
                "короткое замыкание цепей {}: {context}",
                names.join(" ↔ ")
            ));
        }
    }

    // Дополнительная медь может закрыть обрыв, но не устранит найденное короткое.
    if copper.gaps.is_empty() {
        let mut by_net: BTreeMap<u32, BTreeMap<usize, BTreeSet<String>>> = BTreeMap::new();
        for (address, terminal) in terminals {
            for &id in &terminal.pieces {
                let net = copper.pieces[id].net.unwrap(); // Привязке уже сопоставлено точное имя.
                by_net
                    .entry(net)
                    .or_default()
                    .entry(roots[id])
                    .or_default()
                    .insert(format!("{address} ↔ {}", terminal.endpoint));
            }
        }
        for (net, islands) in by_net {
            if islands.len() > 1 {
                let ends = islands
                    .into_values()
                    .map(|group| group.into_iter().collect::<Vec<_>>().join(", "))
                    .collect::<Vec<_>>();
                issues.push(format!(
                    "обрыв цепи «{}»: {}",
                    layout.nets[&net],
                    ends.join(" | ")
                ));
            }
        }
    }
    if issues.is_empty() {
        Ok(())
    } else {
        Err(invalid(issues.join("; ")))
    }
}

// Проверка контакта выполняется по сечениям аналитических прямых и окружностей.
// Граница может менять порядок внутри сечения только в вершине, экстремуме
// окружности или при пересечении кривых. Между такими абсциссами достаточно
// проверить интервал между каждой парой соседних ординат. Дуги и скругления
// задаются несущими окружностями и предикатом материала, без замены хордами.
// Запас для положительного контакта — одна единица координат формата KiCad.
const GRID: f64 = kicad::METRES_PER_NM;

type XY = [f64; 2];

fn add(a: XY, b: XY) -> XY {
    [a[0] + b[0], a[1] + b[1]]
}
fn sub(a: XY, b: XY) -> XY {
    [a[0] - b[0], a[1] - b[1]]
}
fn scale(v: XY, k: f64) -> XY {
    [v[0] * k, v[1] * k]
}
fn dot(a: XY, b: XY) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn det(a: XY, b: XY) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
fn norm(v: XY) -> f64 {
    v[0].hypot(v[1])
}

fn distance_to_line(p: XY, a: XY, b: XY) -> f64 {
    let d = sub(b, a);
    let length2 = dot(d, d);
    let t = if length2 > 0.0 {
        (dot(sub(p, a), d) / length2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    norm(sub(sub(p, a), scale(d, t)))
}

#[derive(Clone, Copy)]
struct Bounds {
    min: XY,
    max: XY,
}

impl Bounds {
    fn between(a: XY, b: XY) -> Self {
        Self {
            min: [a[0].min(b[0]), a[1].min(b[1])],
            max: [a[0].max(b[0]), a[1].max(b[1])],
        }
    }

    fn union(self, other: Self) -> Self {
        Self {
            min: Self::between(self.min, other.min).min,
            max: Self::between(self.max, other.max).max,
        }
    }

    fn expand(self, width: f64) -> Self {
        Self {
            min: sub(self.min, [width; 2]),
            max: add(self.max, [width; 2]),
        }
    }

    fn intersection(self, other: Self) -> Option<Self> {
        let min = Self::between(self.min, other.min).max;
        let max = Self::between(self.max, other.max).min;
        (min[0] <= max[0] && min[1] <= max[1]).then_some(Self { min, max })
    }
}

#[derive(Clone, Copy)]
enum Curve {
    Segment(XY, XY),
    Circle(XY, f64),
}

impl Curve {
    fn bounds(self) -> Bounds {
        match self {
            Self::Segment(a, b) => Bounds::between(a, b),
            Self::Circle(c, r) => Bounds::between(c, c).expand(r),
        }
    }

    fn ordinates(self, x: f64, values: &mut Vec<f64>) {
        match self {
            Self::Segment(a, b) => {
                let d = sub(b, a);
                if d[0] == 0.0 {
                    if x == a[0] {
                        values.extend([a[1], b[1]]);
                    }
                } else {
                    let t = (x - a[0]) / d[0];
                    if (0.0..=1.0).contains(&t) {
                        values.push(a[1] + t * d[1]);
                    }
                }
            }
            Self::Circle(c, r) => {
                let dx = (x - c[0]).abs();
                if dx <= r {
                    let dy = ((r - dx) * (r + dx)).sqrt();
                    values.extend([c[1] - dy, c[1] + dy]);
                }
            }
        }
    }

    fn crossing_x(self, other: Self, values: &mut Vec<f64>) {
        match (self, other) {
            (Self::Segment(a, b), Self::Segment(c, d)) => {
                let u = sub(b, a);
                let v = sub(d, c);
                let cross = det(u, v);
                if cross != 0.0 {
                    let offset = sub(c, a);
                    let t = det(offset, v) / cross;
                    let s = det(offset, u) / cross;
                    if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&s) {
                        values.push(a[0] + t * u[0]);
                    }
                }
            }
            (Self::Segment(a, b), Self::Circle(c, r))
            | (Self::Circle(c, r), Self::Segment(a, b)) => {
                let u = sub(b, a);
                let length = norm(u);
                if length == 0.0 {
                    return;
                }
                let n = scale(u, length.recip());
                let shift = sub(c, a);
                let d = det(n, shift).abs();
                if d <= r + GRID {
                    let half = ((r - d).max(0.0) * (r + d)).sqrt();
                    let t = dot(n, shift);
                    for s in [t - half, t + half] {
                        if (-GRID..=length + GRID).contains(&s) {
                            values.push(a[0] + s.clamp(0.0, length) * n[0]);
                        }
                    }
                }
            }
            (Self::Circle(a, r), Self::Circle(b, s)) => {
                let u = sub(b, a);
                let d = norm(u);
                if d > 0.0 && d <= r + s + GRID && d + GRID >= (r - s).abs() {
                    let n = scale(u, d.recip());
                    let axis = (d + ((r - s) / d) * (r + s)) / 2.0;
                    // Перестановка множителей формулы Герона удерживает точность
                    // у внешнего и внутреннего касания окружностей.
                    let half = ((r + s - d).max(0.0) * (r + s + d)).sqrt()
                        * ((d - (r - s).abs()).max(0.0) * (d + (r - s).abs())).sqrt()
                        / (2.0 * d);
                    let x = a[0] + axis * n[0];
                    values.extend([x - half * n[1], x + half * n[1]]);
                }
            }
        }
    }
}

enum Area {
    RoundBox {
        center: XY,
        cos: f64,
        sin: f64,
        core: XY,
        radius: f64,
    },
    Capsule {
        start: XY,
        end: XY,
        radius: f64,
    },
    Arc {
        start: XY,
        end: XY,
        center: XY,
        radius: f64,
        sweep: f64,
        half_width: f64,
    },
    Polygon,
}

struct Shape {
    area: Area,
    bounds: Bounds,
    curves: Vec<Curve>,
}

impl Shape {
    fn new(area: Area, curves: Vec<Curve>) -> Self {
        let bounds = curves
            .iter()
            .map(|c| c.bounds())
            .reduce(Bounds::union)
            .unwrap();
        Self {
            area,
            bounds,
            curves,
        }
    }

    fn rounded(rect: RoundedRect) -> Self {
        let center = rect.center.metres();
        let (sin, cos) = rect.angle_rad.sin_cos();
        let radius = rect.radius_m;
        let core = [rect.width_m / 2.0 - radius, rect.height_m / 2.0 - radius];
        let transform = |[x, y]: XY| add(center, [x * cos + y * sin, -x * sin + y * cos]);
        let [u, v] = core;
        let mut curves = Vec::new();
        for sign in [-1.0, 1.0] {
            curves.extend([
                Curve::Segment(
                    transform([-u, sign * (v + radius)]),
                    transform([u, sign * (v + radius)]),
                ),
                Curve::Segment(
                    transform([sign * (u + radius), -v]),
                    transform([sign * (u + radius), v]),
                ),
            ]);
            if radius > 0.0 {
                for x in [-u, u] {
                    curves.push(Curve::Circle(transform([x, sign * v]), radius));
                }
            }
        }
        Self::new(
            Area::RoundBox {
                center,
                cos,
                sin,
                core,
                radius,
            },
            curves,
        )
    }

    fn circle(center: XY, radius: f64) -> Self {
        Self::new(
            Area::RoundBox {
                center,
                cos: 1.0,
                sin: 0.0,
                core: [0.0; 2],
                radius,
            },
            vec![Curve::Circle(center, radius)],
        )
    }

    fn track(path: Centerline, half_width: f64) -> Self {
        match path {
            Centerline::Segment { start, end } => {
                let (start, end) = (start.metres(), end.metres());
                let [dx, dy] = sub(end, start);
                let offset = scale([-dy, dx], half_width / dx.hypot(dy));
                Self::new(
                    Area::Capsule {
                        start,
                        end,
                        radius: half_width,
                    },
                    vec![
                        Curve::Circle(start, half_width),
                        Curve::Circle(end, half_width),
                        Curve::Segment(add(start, offset), add(end, offset)),
                        Curve::Segment(sub(start, offset), sub(end, offset)),
                    ],
                )
            }
            Centerline::Arc {
                start,
                end,
                center_m: center,
                radius_m: radius,
                sweep_rad: sweep,
                ..
            } => {
                let (start, end) = (start.metres(), end.metres());
                let mut curves = vec![
                    Curve::Circle(start, half_width),
                    Curve::Circle(end, half_width),
                    Curve::Circle(center, radius + half_width),
                ];
                if radius > half_width {
                    curves.push(Curve::Circle(center, radius - half_width));
                }
                let mut result = Self::new(
                    Area::Arc {
                        start,
                        end,
                        center,
                        radius,
                        sweep,
                        half_width,
                    },
                    curves,
                );
                // Внешняя окружность может сильно превосходить малую дугу.
                // Ограничивающий прямоугольник строится по экстремумам самой дуги.
                let mut bounds = Bounds::between(start, end);
                for dir in [[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0]] {
                    if on_sweep(sub(start, center), dir, sweep) {
                        let p = add(center, scale(dir, radius));
                        bounds = bounds.union(Bounds::between(p, p));
                    }
                }
                result.bounds = bounds.expand(half_width);
                result
            }
        }
    }

    fn polygon(polygon: &Polygon) -> Self {
        let curves = std::iter::once(polygon.outline())
            .chain(polygon.holes())
            .flat_map(|ring| {
                let points = ring.points();
                points
                    .iter()
                    .zip(points.iter().cycle().skip(1))
                    .map(|(a, b)| Curve::Segment(a.metres(), b.metres()))
            })
            .collect();
        Self::new(Area::Polygon, curves)
    }

    fn margin(&self, point: XY) -> f64 {
        match self.area {
            Area::RoundBox {
                center,
                cos,
                sin,
                core: [u, v],
                radius,
            } => {
                let [x, y] = sub(point, center);
                let a = (x * cos - y * sin).abs() - u;
                let b = (x * sin + y * cos).abs() - v;
                a.max(0.0).hypot(b.max(0.0)) + a.max(b).min(0.0) - radius
            }
            Area::Capsule { start, end, radius } => distance_to_line(point, start, end) - radius,
            Area::Arc {
                start,
                end,
                center,
                radius,
                sweep,
                half_width,
            } => {
                let radial = sub(point, center);
                let distance = if on_sweep(sub(start, center), radial, sweep) {
                    (norm(radial) - radius).abs()
                } else {
                    norm(sub(point, start)).min(norm(sub(point, end)))
                };
                distance - half_width
            }
            Area::Polygon => {
                // Чётность пересечений внешнего кольца и вырезов.
                let mut inside = false;
                let mut distance = f64::INFINITY;
                for &edge in &self.curves {
                    let Curve::Segment(a, b) = edge else {
                        unreachable!()
                    };
                    distance = distance.min(distance_to_line(point, a, b));
                    if (a[1] > point[1]) != (b[1] > point[1]) {
                        let side = det(sub(b, a), sub(point, a));
                        inside ^= (side > 0.0) == (b[1] > a[1]);
                    }
                }
                if inside { -distance } else { distance }
            }
        }
    }
}

fn on_sweep(start: XY, radial: XY, sweep: f64) -> bool {
    let angle = (sweep.signum() * det(start, radial)).atan2(dot(start, radial));
    angle.rem_euclid(std::f64::consts::TAU) <= sweep.abs()
}

#[derive(Debug, PartialEq, Eq)]
enum Relation {
    Apart,
    Touch,
    Overlap,
}

/// Ищет область, принадлежащую положительным фигурам и лежащую вне вычитаний.
fn relation<'a>(
    a: &'a Shape,
    b: Option<&'a Shape>,
    excluded: impl IntoIterator<Item = &'a Shape>,
) -> Relation {
    let mut window = a.bounds.expand(GRID);
    if let Some(b) = b {
        let Some(common) = window.intersection(b.bounds.expand(GRID)) else {
            return Relation::Apart;
        };
        window = common;
    }
    let excluded = excluded
        .into_iter()
        .filter(|shape| shape.bounds.expand(GRID).intersection(window).is_some())
        .collect::<Vec<_>>();
    let mut curves = std::iter::once(a)
        .chain(b)
        .chain(excluded.iter().copied())
        .flat_map(|shape| shape.curves.iter().copied())
        .filter_map(|curve| {
            let bounds = curve.bounds();
            bounds.intersection(window).map(|_| (curve, bounds))
        })
        .collect::<Vec<_>>();
    curves.sort_by(|(_, a), (_, b)| a.min[0].total_cmp(&b.min[0]));

    let mut abscissas = vec![window.min[0], window.max[0]];
    for (i, &(first, bounds)) in curves.iter().enumerate() {
        abscissas.extend([bounds.min[0], bounds.max[0]]);
        for &(second, other_bounds) in &curves[i + 1..] {
            if other_bounds.min[0] > bounds.max[0] + GRID {
                break;
            }
            if bounds.expand(GRID).intersection(other_bounds).is_some() {
                first.crossing_x(second, &mut abscissas);
            }
        }
    }
    let mut found = Relation::Apart;
    let mut ordinates = Vec::new();
    for x in samples(&mut abscissas, window.min[0], window.max[0]) {
        ordinates.clear();
        ordinates.extend([window.min[1], window.max[1]]);
        for &(curve, bounds) in &curves {
            if bounds.min[0] <= x && x <= bounds.max[0] {
                curve.ordinates(x, &mut ordinates);
            }
        }
        for y in samples(&mut ordinates, window.min[1], window.max[1]) {
            let p = [x, y];
            let mut margin = a.margin(p);
            if let Some(b) = b {
                margin = margin.max(b.margin(p));
            }
            if margin > GRID {
                continue;
            }
            for shape in &excluded {
                margin = margin.max(-shape.margin(p));
            }
            if margin < -GRID {
                return Relation::Overlap;
            }
            if margin <= GRID {
                found = Relation::Touch;
            }
        }
    }
    found
}

fn samples(values: &mut Vec<f64>, min: f64, max: f64) -> impl Iterator<Item = f64> + '_ {
    values.retain(|&value| min <= value && value <= max);
    values.sort_by(f64::total_cmp);
    values.dedup();
    values.iter().enumerate().flat_map(|(i, &value)| {
        let mid = values.get(i + 1).map(|&next| value.midpoint(next));
        std::iter::once(value).chain(mid)
    })
}
