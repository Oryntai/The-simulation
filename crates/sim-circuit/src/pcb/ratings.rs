//! Установившийся ток и пределы участков разводки. Геометрия — из
//! KiCad 8/9, разделы [Stack Up Settings](https://dev-docs.kicad.org/en/file-formats/sexpr-pcb/#stack-up-settings)
//! и [Tracks](https://dev-docs.kicad.org/en/file-formats/sexpr-pcb/#tracks-section).
//! Дорожки считаются по оси, стенки переходов — между серединами соседних
//! слоёв меди. Сопротивление растекания в контактах и проводимость полигонов
//! требуют отдельной модели. Электрический расчёт ведётся при заданной
//! температуре меди. IPC-2221 даёт отдельную оценку перегрева.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

use nalgebra::{DMatrix, DVector};

use super::layout::{Centerline, Layout, NotAnalyzed, Point};
use super::{
    CU_RHO_20C, FOIL_M_PER_OZ, IpcLayer, MIL, ipc2221_current, ipc2221_delta_t, trace_resistance,
    via_resistance,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Scope {
    Reference,
    Assembly,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source<'a> {
    /// Исходная постановка синтетического опыта.
    Reference(&'a str),
    /// Паспорт или замер, применимый к заданной ревизии и режиму.
    Verified(&'a str),
    Estimate(&'a str),
}

impl<'a> Source<'a> {
    fn label(self) -> &'a str {
        match self {
            Self::Reference(text) | Self::Verified(text) | Self::Estimate(text) => text,
        }
    }

    fn applies(self, scope: Scope) -> bool {
        match self {
            Self::Reference(_) => scope == Scope::Reference,
            Self::Verified(_) => true,
            Self::Estimate(_) => false,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Datum<'a> {
    pub value: f64,
    pub source: Source<'a>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Copper<'a> {
    pub rho_20_ohm_m: Datum<'a>,
    pub alpha_per_k: Datum<'a>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Node<'a> {
    pub at: Point,
    pub layer: &'a str,
}

impl<'a> Node<'a> {
    fn key(self) -> (&'a str, i32, i32) {
        (self.layer, self.at.x_nm, self.at.y_nm)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Load<'a> {
    pub node: Node<'a>,
    /// Знаковый ток. Положительный отбирается из меди, отрицательный вводится.
    pub current_a: Option<Datum<'a>>,
    /// Допустимый модуль разности потенциалов с опорной клеммой.
    pub max_drop_v: Option<Datum<'a>>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViaLimit<'a> {
    /// Индекс перехода в Layout. Предел тока относится к любому сечению стенки.
    pub via: usize,
    pub plating_m: Option<Datum<'a>>,
    pub max_current_a: Option<Datum<'a>>,
    pub min_ring_m: Option<Datum<'a>>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Input<'a> {
    pub scope: Scope,
    pub net: u32,
    pub reference: Node<'a>,
    /// Все точки с внешним током, кроме опорной. Остальные узлы пассивны.
    pub loads: &'a [Load<'a>],
    /// Источник проверки центролинейной схемы и полноты токовых вводов:
    /// например, сверка с графом меди, клеммами и монтажом.
    pub topology: Option<Source<'a>>,
    /// Параметры, применимые к фольге и металлизации выбранной цепи.
    pub copper: Option<Copper<'a>>,
    /// Заданная температура проводников для расчёта сопротивления.
    pub copper_temp_c: Option<Datum<'a>>,
    pub ambient_c: Option<Datum<'a>>,
    pub max_rise_k: Option<Datum<'a>>,
    pub max_laminate_c: Option<Datum<'a>>,
    /// Для сборки здесь нужна тепловая сверка для реального материала,
    /// геометрии и условий охлаждения. Для эталона — постановка опыта IPC.
    pub thermal_model: Option<Source<'a>>,
    pub vias: &'a [ViaLimit<'a>],
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Open {
    Layout(NotAnalyzed),
    MissingOrUnverified(&'static str),
    UnmodeledCopper,
    OverlappingCopper,
    ArcTap,
    Disconnected,
    Network,
    OutsideIpc2221,
}

impl fmt::Display for Open {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Layout(reason) => write!(f, "{reason}"),
            Self::MissingOrUnverified(name) => write!(f, "нет применимых данных: {name}"),
            Self::UnmodeledCopper => f.write_str("нужна модель проводимости заливки или площадки"),
            Self::OverlappingCopper => f.write_str("нужна модель совмещённых участков меди"),
            Self::ArcTap => f.write_str("нужна модель контакта внутри дуги"),
            Self::Disconnected => f.write_str("модель дорожек и переходов не связывает все узлы"),
            Self::Network => f.write_str("электрический расчёт сети открыт"),
            Self::OutsideIpc2221 => f.write_str("режим вне диапазона графиков IPC-2221"),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Status {
    Pass,
    Fail,
    NotAnalyzed(Open),
}

#[derive(Clone, Debug, PartialEq)]
pub struct Check {
    pub value: Option<f64>,
    pub limit: Option<f64>,
    pub status: Status,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Section {
    /// Индекс дорожки. Одна дорожка может иметь несколько участков.
    Track(usize),
    /// Индекс перехода. Слои данного участка указаны в концах ветви.
    Via(usize),
}

#[derive(Clone, Debug, PartialEq)]
pub struct Branch<'a> {
    pub section: Section,
    pub ends: [Node<'a>; 2],
    pub length_m: Option<f64>,
    pub resistance_ohm: Option<f64>,
    pub open: Option<Open>,
    /// Знак вдоль ends[0] → ends[1].
    pub current_a: Option<f64>,
    pub drop_v: Option<f64>,
    pub power_w: Option<f64>,
    /// Только дорожки. Вне диапазона IPC число остаётся диагностическим.
    pub ipc_rise_k: Option<f64>,
    /// Температура дорожки относительно паспортного предела ламината.
    /// Для стенки перехода применимость по температуре задаёт паспорт тока.
    pub temperature_c: Option<Check>,
    pub current_limit: Check,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ViaReport {
    pub via: usize,
    pub ring: Check,
}

#[derive(Clone, Debug, PartialEq)]
pub struct LoadReport<'a> {
    pub node: Node<'a>,
    /// Знак V(опора) − V(нагрузка). Предел проверяется по модулю.
    pub drop: Check,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Report<'a> {
    pub scope: Scope,
    pub net: u32,
    pub network_open: Vec<Open>,
    pub branches: Vec<Branch<'a>>,
    pub vias: Vec<ViaReport>,
    pub loads: Vec<LoadReport<'a>>,
}

impl Report<'_> {
    /// Статус в пределах режима и модели. Нарушение сохраняется при
    /// открытых соседних ограничениях.
    pub fn status(&self) -> Status {
        let mut status = self
            .network_open
            .first()
            .map_or(Status::Pass, |reason| Status::NotAnalyzed(reason.clone()));
        let checks = self
            .branches
            .iter()
            .flat_map(|branch| std::iter::once(&branch.current_limit).chain(&branch.temperature_c))
            .chain(self.vias.iter().map(|via| &via.ring))
            .chain(self.loads.iter().map(|load| &load.drop));
        for check in checks {
            match &check.status {
                Status::Fail => return Status::Fail,
                Status::NotAnalyzed(_) if status == Status::Pass => status = check.status.clone(),
                _ => {}
            }
        }
        status
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Error(pub &'static str);

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.0)
    }
}

impl std::error::Error for Error {}

type Available<T> = Result<T, Open>;

fn missing(name: &'static str) -> Open {
    Open::MissingOrUnverified(name)
}

fn data(datum: Option<Datum<'_>>, scope: Scope, name: &'static str) -> Available<f64> {
    datum
        .filter(|datum| datum.source.applies(scope))
        .map(|datum| datum.value)
        .ok_or_else(|| missing(name))
}

fn evidence(source: Option<Source<'_>>, scope: Scope, name: &'static str) -> Available<()> {
    source
        .filter(|source| source.applies(scope))
        .map(|_| ())
        .ok_or_else(|| missing(name))
}

fn validate_datum(datum: Option<Datum<'_>>, valid: impl FnOnce(f64) -> bool) -> Result<(), Error> {
    if let Some(datum) = datum {
        ensure(
            datum.value.is_finite() && valid(datum.value),
            "недопустимое число в исходных данных",
        )?;
        validate_source(Some(datum.source))?;
    }
    Ok(())
}

fn validate_source(source: Option<Source<'_>>) -> Result<(), Error> {
    ensure(
        source.is_none_or(|source| !source.label().trim().is_empty()),
        "пустой источник данных",
    )
}

fn ensure(ok: bool, message: &'static str) -> Result<(), Error> {
    if ok { Ok(()) } else { Err(Error(message)) }
}

fn positive(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

fn finite(value: f64) -> Result<f64, Error> {
    ensure(value.is_finite(), "переполнение в расчёте платы")?;
    Ok(value)
}

fn validate(layout: &Layout, input: &Input<'_>) -> Result<(), Error> {
    validate_geometry(layout)?;
    ensure(
        layout.nets.contains_key(&input.net) && input.net != 0,
        "неизвестная цепь платы",
    )?;
    ensure(!input.loads.is_empty(), "не заданы клеммы нагрузки")?;
    let mut locations = BTreeSet::new();
    for node in std::iter::once(input.reference).chain(input.loads.iter().map(|load| load.node)) {
        ensure(
            layout.copper_layers.iter().any(|layer| layer == node.layer),
            "неизвестный слой клеммы",
        )?;
        ensure(locations.insert(node.key()), "повтор места ввода тока")?;
    }
    for load in input.loads {
        validate_datum(load.current_a, |_| true)?;
        validate_datum(load.max_drop_v, |v| v >= 0.0)?;
    }
    validate_source(input.topology)?;
    validate_source(input.thermal_model)?;
    if let Some(copper) = input.copper {
        validate_datum(Some(copper.rho_20_ohm_m), positive)?;
        validate_datum(Some(copper.alpha_per_k), |v| v >= 0.0)?;
    }
    for datum in [input.copper_temp_c, input.ambient_c, input.max_laminate_c] {
        // Ноль абсолютной шкалы по определению шкалы Цельсия.
        validate_datum(datum, |v| v >= -273.15)?;
    }
    validate_datum(input.max_rise_k, positive)?;
    let mut vias = BTreeSet::new();
    for rule in input.vias {
        let via = layout
            .vias
            .get(rule.via)
            .ok_or(Error("неизвестный переход в правилах"))?;
        ensure(
            via.net == Some(input.net) && vias.insert(rule.via),
            "предел повторён или задан для другой цепи",
        )?;
        validate_datum(rule.plating_m, |v| v > 0.0 && v < via.drill_m / 2.0)?;
        validate_datum(rule.max_current_a, |v| v >= 0.0)?;
        validate_datum(rule.min_ring_m, positive)?;
    }
    Ok(())
}

fn copper_factor(input: &Input<'_>) -> Result<Available<f64>, Error> {
    let values = (|| {
        let copper = input.copper.ok_or_else(|| missing("материал проводника"))?;
        Ok((
            data(
                Some(copper.rho_20_ohm_m),
                input.scope,
                "удельное сопротивление меди",
            )?,
            data(
                Some(copper.alpha_per_k),
                input.scope,
                "температурный коэффициент меди",
            )?,
            data(
                input.copper_temp_c,
                input.scope,
                "температура меди для расчёта R",
            )?,
        ))
    })();
    let (rho, alpha, temp) = match values {
        Ok(values) => values,
        Err(reason) => return Ok(Err(reason)),
    };
    // 20 °C — опорная температура исходных функций сопротивления.
    let factor = rho / CU_RHO_20C * (1.0 + alpha * (temp - 20.0));
    ensure(
        positive(factor),
        "модель меди дала неположительное или неконечное ρ(T)",
    )?;
    Ok(Ok(factor))
}

struct Edge<'a> {
    ends: [Node<'a>; 2],
    section: Section,
    length: Available<f64>,
    ohms: Available<f64>,
    /// Сечение в mil², ширина, толщина и тип слоя для формул IPC.
    trace: Option<Available<TraceGeometry>>,
}

#[derive(Clone, Copy)]
struct TraceGeometry {
    area_mil2: f64,
    width_m: f64,
    thickness_m: f64,
    layer: IpcLayer,
}

fn ends(path: Centerline) -> [Point; 2] {
    match path {
        Centerline::Segment { start, end } | Centerline::Arc { start, end, .. } => [start, end],
    }
}

fn validate_geometry(layout: &Layout) -> Result<(), Error> {
    ensure(
        layout.nominal_thickness_m.is_none_or(positive),
        "некорректная номинальная толщина платы",
    )?;
    for track in &layout.tracks {
        ensure(
            positive(track.width_m) && layout.copper_layers.contains(&track.layer),
            "некорректная ширина или слой дорожки",
        )?;
        let [start, end] = ends(track.path);
        ensure(
            start != end && positive(track.path.length_m()),
            "некорректная длина дорожки",
        )?;
        if let Centerline::Arc {
            center_m,
            radius_m,
            sweep_rad,
            ..
        } = track.path
        {
            ensure(
                center_m.iter().all(|v| v.is_finite())
                    && positive(radius_m)
                    && sweep_rad.abs() < std::f64::consts::TAU,
                "некорректная дуга дорожки",
            )?;
        }
    }
    for via in &layout.vias {
        ensure(
            positive(via.drill_m) && via.diameter_m.is_finite() && via.diameter_m > via.drill_m,
            "некорректный диаметр перехода",
        )?;
        ensure(
            via.layers.len() >= 2
                && layout
                    .copper_layers
                    .windows(via.layers.len())
                    .any(|w| w == via.layers),
            "некорректный набор слоёв перехода",
        )?;
    }
    for footprint in &layout.footprints {
        ensure(
            footprint.angle_rad.is_finite(),
            "неконечный угол посадочного места",
        )?;
        for pad in &footprint.pads {
            for shape in pad.land.iter().map(|land| land.outline).chain(pad.hole) {
                ensure(
                    [
                        shape.angle_rad,
                        shape.width_m,
                        shape.height_m,
                        shape.radius_m,
                    ]
                    .into_iter()
                    .all(f64::is_finite),
                    "неконечная геометрия площадки или сверления",
                )?;
            }
        }
    }
    Ok(())
}

fn segment_split(start: Point, end: Point, taps: &BTreeSet<(i32, i32)>) -> Vec<Point> {
    let xy = |p: Point| (p.x_nm, p.y_nm);
    let (first, last) = (xy(start).min(xy(end)), xy(start).max(xy(end)));
    let dx = i128::from(end.x_nm) - i128::from(start.x_nm);
    let dy = i128::from(end.y_nm) - i128::from(start.y_nm);
    let mut points = vec![Point {
        x_nm: first.0,
        y_nm: first.1,
    }];
    for &(x_nm, y_nm) in taps.range(first..last) {
        if (x_nm, y_nm) != first
            && dx * (i128::from(y_nm) - i128::from(start.y_nm))
                == dy * (i128::from(x_nm) - i128::from(start.x_nm))
        {
            points.push(Point { x_nm, y_nm });
        }
    }
    points.push(Point {
        x_nm: last.0,
        y_nm: last.1,
    });
    if xy(start) != first {
        points.reverse();
    }
    points
}

fn length_m(start: Point, end: Point) -> f64 {
    Centerline::Segment { start, end }.length_m()
}

fn track_geometry(layout: &Layout, width_m: f64, layer: &str) -> Available<TraceGeometry> {
    let thickness_m = layout
        .stackup()
        .and_then(|stack| stack.copper_layer(layer))
        .and_then(|copper| copper.thickness_m())
        .map_err(Open::Layout)?;
    let layer = if layout.copper_layers.first().is_some_and(|s| s == layer)
        || layout.copper_layers.last().is_some_and(|s| s == layer)
    {
        IpcLayer::External
    } else {
        IpcLayer::Internal
    };
    Ok(TraceGeometry {
        area_mil2: (width_m / MIL) * (thickness_m / MIL),
        width_m,
        thickness_m,
        layer,
    })
}

fn resistance(ohms: Available<f64>) -> Result<Available<f64>, Error> {
    if let Ok(value) = ohms {
        ensure(
            positive(value) && positive(value.recip()),
            "сопротивление вне численного диапазона",
        )?;
    }
    Ok(ohms)
}

fn find_via<'a>(input: &Input<'a>, index: usize) -> Option<ViaLimit<'a>> {
    input.vias.iter().copied().find(|v| v.via == index)
}

fn edges<'a>(
    layout: &'a Layout,
    input: &Input<'a>,
    factor: &Available<f64>,
) -> Result<Vec<Edge<'a>>, Error> {
    let tracks = layout
        .tracks
        .iter()
        .enumerate()
        .filter(|(_, t)| t.net == Some(input.net));
    let vias = layout
        .vias
        .iter()
        .enumerate()
        .filter(|(_, v)| v.net == Some(input.net));
    let mut taps = BTreeMap::<&'a str, BTreeSet<_>>::new();
    let mut insert = |node: Node<'a>| {
        let (layer, x, y) = node.key();
        taps.entry(layer).or_default().insert((x, y));
    };
    insert(input.reference);
    for load in input.loads {
        insert(load.node);
    }
    for (_, track) in tracks.clone() {
        let [start, end] = ends(track.path);
        insert(Node {
            at: start,
            layer: &track.layer,
        });
        insert(Node {
            at: end,
            layer: &track.layer,
        });
    }
    for (_, via) in vias.clone() {
        for layer in &via.layers {
            insert(Node { at: via.at, layer });
        }
    }

    let mut edges = Vec::new();
    let mut segments = BTreeSet::new();
    for (index, track) in tracks {
        let geom = track_geometry(layout, track.width_m, &track.layer);
        if let Ok(geom) = geom {
            ensure(positive(geom.area_mil2), "некорректное сечение дорожки")?;
        }
        let points = match track.path {
            Centerline::Segment { start, end } => {
                segment_split(start, end, &taps[track.layer.as_str()])
            }
            Centerline::Arc { .. } => ends(track.path).to_vec(),
        };
        for pair in points.windows(2) {
            let length = match track.path {
                Centerline::Segment { .. } => length_m(pair[0], pair[1]),
                Centerline::Arc { .. } => track.path.length_m(),
            };
            let ends = [pair[0], pair[1]].map(|at| Node {
                at,
                layer: &track.layer,
            });
            let [a, b] = ends.map(Node::key);
            let unsupported = match track.path {
                Centerline::Segment { .. } => {
                    (!segments.insert([a.min(b), a.max(b)])).then_some(Open::OverlappingCopper)
                }
                Centerline::Arc { mid, .. } => taps[track.layer.as_str()]
                    .contains(&(mid.x_nm, mid.y_nm))
                    .then_some(Open::ArcTap),
            };
            let ohms = if let Some(reason) = unsupported {
                Err(reason)
            } else {
                resistance(geom.clone().and_then(|g| {
                    factor.clone().map(|factor| {
                        factor * trace_resistance(length, g.width_m, g.thickness_m, 20.0)
                    })
                }))?
            };
            edges.push(Edge {
                ends,
                section: Section::Track(index),
                length: Ok(length),
                ohms,
                trace: Some(geom.clone()),
            });
        }
    }
    for (index, via) in vias {
        let plating = data(
            find_via(input, index).and_then(|v| v.plating_m),
            input.scope,
            "толщина металлизации перехода",
        );
        for pair in via.layers.windows(2) {
            let [a, b] = [&pair[0], &pair[1]];
            let length = layout
                .stackup()
                .and_then(|s| s.copper_center_distance_m(a, b))
                .map_err(Open::Layout);
            let ends = [a, b].map(|layer| Node { at: via.at, layer });
            let ohms = if segments.insert(ends.map(Node::key)) {
                resistance(length.clone().and_then(|length| {
                    Ok(factor.clone()?
                        * via_resistance(via.drill_m, plating.clone()?, length, 20.0))
                }))?
            } else {
                Err(Open::OverlappingCopper)
            };
            edges.push(Edge {
                ends,
                section: Section::Via(index),
                length,
                ohms,
                trace: None,
            });
        }
    }
    Ok(edges)
}

struct Network<'a> {
    ids: BTreeMap<(&'a str, i32, i32), usize>,
    links: Vec<[usize; 2]>,
    injection: Vec<f64>,
}

impl<'a> Network<'a> {
    fn new(edges: &[Edge<'a>], input: &Input<'a>) -> (Self, Vec<Open>) {
        let mut ids = BTreeMap::new();
        // Опорная клемма получает индекс ноль.
        let mut index = |node: Node<'a>| {
            let next = ids.len();
            *ids.entry(node.key()).or_insert(next)
        };
        index(input.reference);
        let links: Vec<_> = edges.iter().map(|e| e.ends.map(&mut index)).collect();
        for load in input.loads {
            index(load.node);
        }
        let mut adjacent = vec![Vec::new(); ids.len()];
        for &[a, b] in &links {
            adjacent[a].push(b);
            adjacent[b].push(a);
        }
        let mut component = vec![false; ids.len()];
        let mut queue = vec![0];
        component[0] = true;
        while let Some(a) = queue.pop() {
            for &b in &adjacent[a] {
                if !component[b] {
                    component[b] = true;
                    queue.push(b);
                }
            }
        }
        let mut open = Vec::new();
        if component.contains(&false) {
            open.push(Open::Disconnected);
        }
        let mut injection = vec![0.0; ids.len()];
        for load in input.loads {
            match data(load.current_a, input.scope, "ток клеммы в заданном режиме")
            {
                Ok(current) => injection[ids[&load.node.key()]] = -current,
                Err(reason) => add_open(&mut open, reason),
            }
        }
        (
            Self {
                ids,
                links,
                injection,
            },
            open,
        )
    }

    fn solve(&self, edges: &[Edge<'_>]) -> Result<Vec<f64>, Error> {
        let n = self.ids.len() - 1;
        let mut conductance = DMatrix::<f64>::zeros(n, n);
        for (&[a, b], edge) in self.links.iter().zip(edges) {
            let g = edge
                .ohms
                .as_ref()
                .map_err(|_| Error("нет сопротивления ветви"))?
                .recip();
            for (row, col, value) in [(a, a, g), (b, b, g), (a, b, -g), (b, a, -g)] {
                if row != 0 && col != 0 {
                    let entry = &mut conductance[(row - 1, col - 1)];
                    *entry = finite(*entry + value)?;
                }
            }
        }
        // Для резистивной связной сети приведённый лапласиан положительно
        // определён. Искусственных проводимостей в системе нет.
        let rhs = DVector::from_column_slice(&self.injection[1..]);
        let solved = conductance
            .cholesky()
            .ok_or(Error("сеть численно вырождена"))?
            .solve(&rhs);
        std::iter::once(Ok(0.0))
            .chain(solved.iter().copied().map(finite))
            .collect()
    }

    fn check_balance(&self, currents: &[f64]) -> Result<(), Error> {
        let mut sums = vec![0.0; self.ids.len()];
        let mut scales = vec![0.0; self.ids.len()];
        for (&[a, b], &current) in self.links.iter().zip(currents) {
            for (node, sign) in [(a, 1.0), (b, -1.0)] {
                sums[node] = finite(sums[node] + sign * current)?;
                scales[node] = finite(scales[node] + current.abs())?;
            }
        }
        let mut injection = self.injection.clone();
        injection[0] = finite(-injection[1..].iter().sum::<f64>())?;
        let network_scale = finite(injection.iter().map(|i| i.abs()).sum::<f64>())?;
        for ((sum, scale), expected) in sums.into_iter().zip(scales).zip(injection) {
            // Корень из машинной точности — допуск относительной невязки,
            // в пассивном узле также учитывается округление общего падения.
            let passive_roundoff = if expected == 0.0 {
                f64::EPSILON * network_scale
            } else {
                0.0
            };
            let tolerance = f64::EPSILON.sqrt() * scale.max(expected.abs()) + passive_roundoff;
            ensure(
                (sum - expected).abs() <= tolerance,
                "не сошёлся баланс токов Кирхгофа",
            )?;
        }
        Ok(())
    }
}

fn add_open(open: &mut Vec<Open>, reason: Open) {
    if !open.contains(&reason) {
        open.push(reason);
    }
}

fn maximum(value: f64, limit: f64) -> bool {
    value <= limit
}

fn check(
    value: Option<f64>,
    limit: Available<f64>,
    comparable: Available<()>,
    accepts: impl FnOnce(f64, f64) -> bool,
) -> Check {
    let status = match (&comparable, &limit, value) {
        (_, Err(reason), _) | (Err(reason), _, _) => Status::NotAnalyzed(reason.clone()),
        (Ok(()), Ok(limit), Some(value)) if accepts(value, *limit) => Status::Pass,
        (Ok(()), Ok(_), Some(_)) => Status::Fail,
        _ => Status::NotAnalyzed(Open::Network),
    };
    Check {
        value,
        limit: limit.ok(),
        status,
    }
}

fn laminate_material(layout: &Layout) -> Available<()> {
    if let [first, .., last] = layout.copper_layers.as_slice() {
        for dielectric in layout
            .stackup()
            .and_then(|s| s.dielectrics_between(first, last))
            .map_err(Open::Layout)?
        {
            dielectric.material().map_err(Open::Layout)?;
        }
    }
    Ok(())
}

fn thermal_context(layout: &Layout, input: &Input<'_>, material: Available<()>) -> Available<()> {
    material?;
    for zone in &layout.zones {
        for layer in zone.layers() {
            zone.fill(layer).map_err(Open::Layout)?;
        }
    }
    evidence(
        input.thermal_model,
        input.scope,
        "условия применимости тепловой модели платы",
    )
}

fn ipc_domain(trace: TraceGeometry, rise: f64, current: Option<f64>) -> Available<()> {
    // Границы графиков из описания pcb::ipc2221_current.
    if (10.0..=100.0).contains(&rise)
        && trace.width_m <= 400.0 * MIL
        && (0.5 * FOIL_M_PER_OZ..=3.0 * FOIL_M_PER_OZ).contains(&trace.thickness_m)
        && current.is_none_or(|i| i.abs() <= 35.0)
    {
        Ok(())
    } else {
        Err(Open::OutsideIpc2221)
    }
}

fn rate_trace(
    trace: &Available<TraceGeometry>,
    input: &Input<'_>,
    material: &Available<()>,
    thermal: &Available<()>,
    current: Option<f64>,
) -> Result<(Option<f64>, Check, Check), Error> {
    let scope = input.scope;
    let rise = trace
        .as_ref()
        .ok()
        .zip(current)
        .map(|(g, i)| finite(ipc2221_delta_t(i.abs(), g.area_mil2, g.layer)))
        .transpose()?;
    let ambient = data(input.ambient_c, scope, "температура среды");
    let temperature = ambient
        .as_ref()
        .ok()
        .zip(rise)
        .map(|(t, r)| finite(t + r))
        .transpose()?;
    let max_temp = data(
        input.max_laminate_c,
        scope,
        "рабочая температура материала платы",
    );
    let allowed_rise = data(input.max_rise_k, scope, "расчётный предел перегрева");
    let allowed_current = match (trace, &allowed_rise) {
        (Ok(g), Ok(rise)) => Ok(finite(ipc2221_current(g.area_mil2, *rise, g.layer))?),
        (Err(reason), _) | (_, Err(reason)) => Err(reason.clone()),
    };
    let comparable = |rise: Available<f64>| {
        ipc_domain(trace.clone()?, rise?, current)?;
        thermal.clone()
    };
    let temp_rise = match (&max_temp, &ambient) {
        (Ok(limit), Ok(ambient)) => Ok(finite(limit - ambient)?),
        (Err(reason), _) | (_, Err(reason)) => Err(reason.clone()),
    };
    // Если среда сама горячее паспортного предела, IPC не нужен.
    let ambient_violation = ambient
        .as_ref()
        .ok()
        .zip(max_temp.as_ref().ok())
        .is_some_and(|(actual, limit)| actual > limit);
    let temperature_check = if ambient_violation {
        check(
            temperature.or_else(|| ambient.ok()),
            max_temp,
            material.clone(),
            maximum,
        )
    } else {
        check(temperature, max_temp, comparable(temp_rise), maximum)
    };
    Ok((
        rise,
        check(
            current.map(f64::abs),
            allowed_current,
            comparable(allowed_rise),
            maximum,
        ),
        temperature_check,
    ))
}

/// Считает одну цепь и один установившийся режим. Сохраняет геометрию,
/// сопротивления и независимые ограничения при открытом сетевом расчёте.
/// Клеммы задаются на осях дорожек либо в центрах переходов. Замыкания
/// по краям меди, посадочные места и полноту монтажного маршрута сверяет
/// источник `Input::topology`. Без этой сверки сравнения по току и падению
/// остаются открытыми.
pub fn evaluate<'a>(layout: &'a Layout, input: &Input<'a>) -> Result<Report<'a>, Error> {
    validate(layout, input)?;
    let factor = copper_factor(input)?;
    let edges = edges(layout, input, &factor)?;
    let (network, mut network_open) = Network::new(&edges, input);
    if layout.has_unmodeled_contacts(input.net) {
        add_open(&mut network_open, Open::OverlappingCopper);
    }
    if let Err(reason) = evidence(
        input.topology,
        input.scope,
        "сверка связности и вводов тока",
    ) {
        add_open(&mut network_open, reason);
    }
    for zone in layout
        .zones
        .iter()
        .filter(|zone| zone.net == Some(input.net))
    {
        for layer in zone.layers() {
            let reason = zone
                .fill(layer)
                .err()
                .map_or(Open::UnmodeledCopper, Open::Layout);
            add_open(&mut network_open, reason);
        }
    }
    // Металлизированная площадка может замкнуть ветви на разных слоях.
    if layout
        .footprints
        .iter()
        .flat_map(|fp| &fp.pads)
        .any(|pad| pad.net == Some(input.net) && pad.kind == super::kicad::PadKind::PlatedHole)
    {
        add_open(&mut network_open, Open::UnmodeledCopper);
    }
    for edge in &edges {
        if let Err(reason) = &edge.ohms {
            add_open(&mut network_open, reason.clone());
        }
    }
    let material = laminate_material(layout);
    let thermal = thermal_context(layout, input, material.clone());
    let mut result = Report {
        scope: input.scope,
        net: input.net,
        network_open,
        branches: Vec::new(),
        vias: Vec::new(),
        loads: Vec::new(),
    };
    let voltage = result
        .network_open
        .is_empty()
        .then(|| network.solve(&edges))
        .transpose()?;
    let mut currents = Vec::new();
    for (edge, &[a, b]) in edges.into_iter().zip(&network.links) {
        let drop = voltage.as_ref().map(|v| finite(v[a] - v[b])).transpose()?;
        let current = drop
            .zip(edge.ohms.as_ref().ok())
            .map(|(v, r)| finite(v / r))
            .transpose()?;
        currents.extend(current);
        let power_w = drop.zip(current).map(|(v, i)| finite(v * i)).transpose()?;
        let (ipc_rise_k, current_limit, temperature_c) = if let Some(g) = &edge.trace {
            let (rise, ampacity, temperature) = rate_trace(g, input, &material, &thermal, current)?;
            (rise, ampacity, Some(temperature))
        } else {
            let Section::Via(index) = edge.section else {
                unreachable!()
            };
            let limit = data(
                find_via(input, index).and_then(|v| v.max_current_a),
                input.scope,
                "паспортный ток перехода",
            );
            (
                None,
                check(current.map(f64::abs), limit, Ok(()), maximum),
                None,
            )
        };
        result.branches.push(Branch {
            section: edge.section,
            ends: edge.ends,
            length_m: edge.length.ok(),
            open: edge.ohms.as_ref().err().cloned(),
            resistance_ohm: edge.ohms.ok(),
            current_a: current,
            drop_v: drop,
            power_w,
            ipc_rise_k,
            temperature_c,
            current_limit,
        });
    }
    if voltage.is_some() {
        network.check_balance(&currents)?;
    }
    for (index, via) in layout
        .vias
        .iter()
        .enumerate()
        .filter(|(_, v)| v.net == Some(input.net))
    {
        let limit = data(
            find_via(input, index).and_then(|v| v.min_ring_m),
            input.scope,
            "минимальное кольцо перехода",
        );
        result.vias.push(ViaReport {
            via: index,
            ring: check(
                Some((via.diameter_m - via.drill_m) / 2.0),
                limit,
                Ok(()),
                |value, limit| value >= limit,
            ),
        });
    }
    for load in input.loads {
        let drop = voltage.as_ref().map(|v| -v[network.ids[&load.node.key()]]);
        let limit = data(load.max_drop_v, input.scope, "допустимое падение на клемме");
        result.loads.push(LoadReport {
            node: load.node,
            drop: check(drop, limit, Ok(()), |value, limit| value.abs() <= limit),
        });
    }
    Ok(result)
}
