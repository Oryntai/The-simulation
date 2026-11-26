//! Геометрия возврата и сосредоточенная модель общего участка.
//! Источник геометрии — KiCad 8/9, разделы [Tracks](https://dev-docs.kicad.org/en/file-formats/sexpr-pcb/#tracks-section)
//! и [Zone Fill Polygons](https://dev-docs.kicad.org/en/file-formats/sexpr-intro/#zone-fill-polygons).
//! Геометрическая проверка относится к указанной плоскости и полосе проекции трассы.
//! Для физического вывода об импедансе, переходном процессе и ЭМС нужны отдельные данные.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::TAU;
use std::fmt;

use nalgebra::Vector2;

use super::layout::{
    Centerline, Layout, Location, NotAnalyzed, Point, Polygon, RoundedRect, Track,
};
use super::ratings::{Datum, Node, Scope, Source};
use super::{CU_RHO_20C, trace_resistance, via_resistance};

pub const HAZARD_CODE: &str = "ground_bounce";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Error(pub &'static str);

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.0)
    }
}

impl std::error::Error for Error {}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Open {
    Layout(NotAnalyzed),
    Evidence(&'static str),
    MultipleRegions,
    ClippedPlane,
    InterveningLayers,
    Transfer,
    DetourGeometry,
    SharedTopology,
    Uncertainty,
}

impl fmt::Display for Open {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Layout(e) => e.fmt(f),
            Self::Evidence(name) => write!(f, "нет применимых данных: {name}"),
            Self::MultipleRegions => f.write_str("нужна сверка сшивки отдельных областей заливки"),
            Self::ClippedPlane => {
                f.write_str("нужна связность заливки после обрезки контуром платы")
            }
            Self::InterveningLayers => {
                f.write_str("роль удалённой плоскости требует модели промежуточной меди")
            }
            Self::Transfer => f.write_str("не задан локальный путь смены опоры"),
            Self::DetourGeometry => f.write_str("нужна геометрия обхода разрыва у перехода"),
            Self::SharedTopology => f.write_str("нужна модель распределения тока между путями"),
            Self::Uncertainty => f.write_str("интервал результата пересекает предел"),
        }
    }
}

impl From<NotAnalyzed> for Open {
    fn from(value: NotAnalyzed) -> Self {
        Self::Layout(value)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Status {
    Pass,
    Fail,
    NotAnalyzed(Open),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane<'a> {
    pub zone: usize,
    pub layer: &'a str,
    /// Точка электрической привязки внутри выбранной сохранённой области меди.
    pub tie: Point,
    /// Сверка привязки к штатному возврату, ревизии заливки и её контактов.
    pub connection: Option<Source<'a>>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReferenceInput<'a> {
    pub scope: Scope,
    pub track: usize,
    pub plane: Plane<'a>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ReferenceReport {
    pub status: Status,
    pub height_m: Option<f64>,
    /// Границы пропусков от начала оси трассы в порядке файла.
    pub center_gaps_m: Option<Vec<[f64; 2]>>,
    /// Пропуски с учётом радиуса меди, включая круглые торцы.
    pub copper_gaps_m: Option<Vec<[f64; 2]>>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Step<'a> {
    /// Участок прямой дорожки или дуга целиком. Направление задают текущий узел и конец.
    Track {
        index: usize,
        to: Point,
    },
    Via {
        index: usize,
        to_layer: &'a str,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Route<'a> {
    pub net: u32,
    pub start: Node<'a>,
    pub steps: &'a [Step<'a>],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Terminal<'a> {
    pub reference: &'a str,
    pub pad: &'a str,
    pub layer: &'a str,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DecouplingInput<'a> {
    pub scope: Scope,
    /// Пути от выводов нагрузки к площадкам конденсатора.
    pub supply: Route<'a>,
    pub return_arm: Route<'a>,
    pub load: [Terminal<'a>; 2],
    pub capacitor: [Terminal<'a>; 2],
    /// Сверка схемы, клемм, применимости конденсатора и фактической меди.
    pub topology: Option<Source<'a>>,
    /// Допустимый периметр геометрического контура в данном режиме.
    pub max_perimeter_m: Option<Datum<'a>>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DecouplingReport {
    pub status: Status,
    pub arm_lengths_m: Option<[f64; 2]>,
    /// Монтажный периметр. Контур замыкается между центрами площадок каждой детали.
    pub perimeter_m: Option<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Transfer<'a> {
    StitchVia(usize),
    /// Конденсатор между плоскостями. Начала ветвей лежат в их заливках.
    Capacitor {
        arms: [Route<'a>; 2],
        pads: [Terminal<'a>; 2],
        /// Сверка цепи конденсатора и его применимости для данного фронта.
        coupling: Option<Source<'a>>,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TransitionInput<'a> {
    pub scope: Scope,
    pub tracks: [usize; 2],
    pub planes: [Plane<'a>; 2],
    pub signal_via: usize,
    pub transfer: Option<Transfer<'a>>,
    /// Допустимая длина подхода от сигнального перехода.
    /// Для конденсатора включает ход обеих ветвей и расстояние между его площадками.
    pub max_detour_m: Option<Datum<'a>>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct TransitionReport {
    pub status: Status,
    pub detour_m: Option<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GroundBounceInput<'a> {
    pub scope: Scope,
    /// Оба маршрута идут от одной звезды к своим клеммам.
    pub power: Route<'a>,
    pub signal: Route<'a>,
    /// Сверка единственности путей, включая растекание в меди, зоны, площадки и вводы тока.
    pub topology: Option<Source<'a>>,
    pub resistivity_ohm_m: Option<Datum<'a>>,
    /// Металлизация по индексу перехода в Layout.
    pub plating_m: &'a [(usize, Datum<'a>)],
    /// Индуктивность всей общей части, включая взаимную связь в применимой модели.
    pub shared_inductance_h: Option<Datum<'a>>,
    /// Знаковые ток и производная в одной расчётной точке режима.
    /// Для Pass их модули должны одновременно ограничивать огибающие всего режима.
    pub current_a: Option<Datum<'a>>,
    pub slew_a_per_s: Option<Datum<'a>>,
    pub max_bounce_v: Option<Datum<'a>>,
    /// Верхняя граница модуля оставшихся вкладов, включая площадку звезды
    /// и ошибку модели. Для Pass исходные ток и производная должны покрывать режим.
    pub omitted_bounce_v: Option<Datum<'a>>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct GroundBounceReport {
    pub status: Status,
    pub shared_length_m: Option<f64>,
    pub resistance_ohm: Option<f64>,
    pub inductance_h: Option<f64>,
    pub bounce_v: Option<f64>,
    pub bounds_v: Option<[f64; 2]>,
}

type V = Vector2<f64>;
type Available<T> = Result<T, Open>;

fn require(condition: bool, message: &'static str) -> Result<(), Error> {
    condition.then_some(()).ok_or(Error(message))
}

fn point(p: Point) -> V {
    V::from(p.metres())
}

fn valid_source(source: Option<Source<'_>>) -> Result<(), Error> {
    require(
        source.is_none_or(|s| match s {
            Source::Reference(s) | Source::Verified(s) | Source::Estimate(s) => {
                !s.trim().is_empty()
            }
        }),
        "пустое название источника",
    )
}

fn evidence(source: Option<Source<'_>>, scope: Scope, name: &'static str) -> Available<()> {
    match source {
        Some(Source::Verified(_)) => Ok(()),
        Some(Source::Reference(_)) if scope == Scope::Reference => Ok(()),
        _ => Err(Open::Evidence(name)),
    }
}

fn validate_datum(
    datum: Option<Datum<'_>>,
    predicate: impl FnOnce(f64) -> bool,
) -> Result<(), Error> {
    if let Some(datum) = datum {
        valid_source(Some(datum.source))?;
        require(
            datum.value.is_finite() && predicate(datum.value),
            "некорректное численное данное возврата",
        )?;
    }
    Ok(())
}

fn data(datum: Option<Datum<'_>>, scope: Scope, name: &'static str) -> Available<f64> {
    let datum = datum.ok_or(Open::Evidence(name))?;
    evidence(Some(datum.source), scope, name)?;
    Ok(datum.value)
}

fn positive(number: f64) -> bool {
    number.is_finite() && number > 0.0
}

fn finite(number: f64) -> Result<f64, Error> {
    require(number.is_finite(), "переполнение в расчёте возврата")?;
    Ok(number)
}

fn nonnegative(number: f64) -> bool {
    number >= 0.0
}

fn ends(path: Centerline) -> [Point; 2] {
    match path {
        Centerline::Segment { start, end } | Centerline::Arc { start, end, .. } => [start, end],
    }
}

fn validate_layout(layout: &Layout) -> Result<(), Error> {
    require(
        layout.nominal_thickness_m.is_none_or(positive),
        "некорректная толщина платы",
    )?;
    for t in &layout.tracks {
        require(
            positive(t.width_m)
                && positive(t.path.length_m())
                && ends(t.path)[0] != ends(t.path)[1]
                && layout.copper_layers.contains(&t.layer),
            "некорректная дорожка",
        )?;
        if let Centerline::Arc {
            center_m,
            radius_m,
            sweep_rad,
            ..
        } = t.path
        {
            require(
                center_m.iter().all(|v| v.is_finite())
                    && positive(radius_m)
                    && sweep_rad.abs() < TAU,
                "некорректная дуга",
            )?;
        }
    }
    for via in &layout.vias {
        require(
            positive(via.drill_m)
                && positive(via.diameter_m)
                && via.diameter_m > via.drill_m
                && via.layers.len() >= 2
                && layout
                    .copper_layers
                    .windows(via.layers.len())
                    .any(|w| w == via.layers),
            "некорректный переход",
        )?;
    }
    for fp in &layout.footprints {
        require(
            fp.angle_rad.is_finite(),
            "некорректный угол посадочного места",
        )?;
        for pad in &fp.pads {
            for r in pad.land.iter().map(|land| land.outline).chain(pad.hole) {
                require(
                    r.angle_rad.is_finite()
                        && positive(r.width_m)
                        && positive(r.height_m)
                        && r.radius_m.is_finite()
                        && (0.0..=r.width_m.min(r.height_m) / 2.0).contains(&r.radius_m),
                    "некорректная геометрия площадки или сверления",
                )?;
            }
        }
    }
    Ok(())
}

fn track(layout: &Layout, index: usize) -> Result<&Track, Error> {
    layout.tracks.get(index).ok_or(Error("неизвестная дорожка"))
}

fn known_layer(layout: &Layout, layer: &str) -> bool {
    layout.copper_layers.iter().any(|name| name == layer)
}

fn validate_plane(layout: &Layout, plane: Plane<'_>) -> Result<u32, Error> {
    valid_source(plane.connection)?;
    let zone = layout
        .zones
        .get(plane.zone)
        .ok_or(Error("неизвестная опорная зона"))?;
    require(
        zone.layers().iter().any(|name| name == plane.layer),
        "слой отсутствует в опорной зоне",
    )?;
    zone.net
        .filter(|net| layout.nets.contains_key(net))
        .ok_or(Error("опорная зона не имеет известной цепи"))
}

fn active_plane<'a>(layout: &'a Layout, plane: Plane<'_>) -> Available<Option<&'a Polygon>> {
    let regions = layout.zones[plane.zone].fill(plane.layer)?;
    let mut at_tie = regions
        .iter()
        .filter(|r| r.polygon.locate(plane.tie) != Location::Outside);
    let Some(region) = at_tie.next() else {
        return Ok(None);
    };
    if at_tie.next().is_some() {
        return Err(Open::MultipleRegions);
    }
    if region.island || region.polygon.locate(plane.tie) != Location::Inside {
        return Err(Open::Evidence("контакт с опорной заливкой"));
    }
    Ok(Some(&region.polygon))
}

fn stack_between(layout: &Layout, a: &str, b: &str) -> Available<f64> {
    let stack = layout.stackup()?;
    stack.copper_layer(a)?.thickness_m()?;
    for layer in stack.dielectrics_between(a, b)? {
        layer.material()?;
    }
    Ok(stack.copper_center_distance_m(a, b)?)
}

fn adjacent_plane(layout: &Layout, trace: &str, plane: &str) -> Available<()> {
    layout
        .copper_layers
        .windows(2)
        .any(|pair| {
            (pair[0] == trace && pair[1] == plane) || (pair[0] == plane && pair[1] == trace)
        })
        .then_some(())
        .ok_or(Open::InterveningLayers)
}

fn status(open: Available<()>) -> Status {
    open.map_or_else(Status::NotAnalyzed, |()| Status::Pass)
}

fn ring_edges(points: &[Point]) -> impl Iterator<Item = (V, V)> + '_ {
    points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .map(|(&a, &b)| (point(a), point(b)))
}

fn boundaries(poly: &Polygon) -> impl Iterator<Item = (V, V)> + '_ {
    std::iter::once(poly.outline())
        .chain(poly.holes())
        .flat_map(|ring| ring_edges(ring.points()))
}

fn cross(a: V, b: V) -> f64 {
    a.x * b.y - a.y * b.x
}

fn distance_edge(p: V, a: V, b: V) -> f64 {
    let d = b - a;
    let t = ((p - a).dot(&d) / d.norm_squared()).clamp(0.0, 1.0);
    (p - (a + t * d)).norm()
}

fn on_same_substrate(copper: &Polygon, substrate: &[Polygon]) -> bool {
    substrate.iter().any(|board| {
        boundaries(copper).all(|(a, b)| {
            if !inside(board, a, 0.0) {
                return false;
            }
            // Концы могут лежать в материале по разные стороны выреза.
            !boundaries(board).any(|(c, d)| {
                let same_side = |x, y| (x > 0.0 && y > 0.0) || (x < 0.0 && y < 0.0);
                let overlaps = (0..2)
                    .all(|i| a[i].min(b[i]) <= c[i].max(d[i]) && c[i].min(d[i]) <= a[i].max(b[i]));
                overlaps
                    && !same_side(cross(b - a, c - a), cross(b - a, d - a))
                    && !same_side(cross(d - c, a - c), cross(d - c, b - c))
            })
        })
    })
}

struct Motion {
    start: V,
    end: V,
    arc: Option<(V, f64, f64)>,
}

impl Motion {
    fn new(path: Centerline) -> Self {
        let [start, end] = ends(path).map(point);
        let arc = match path {
            Centerline::Arc {
                center_m,
                radius_m,
                sweep_rad,
                ..
            } => Some((V::from(center_m), radius_m, sweep_rad)),
            _ => None,
        };
        Self { start, end, arc }
    }

    fn at(&self, t: f64) -> V {
        if let Some((c, r, sweep)) = self.arc {
            let d = self.start - c;
            let angle = d.y.atan2(d.x) + t * sweep;
            return c + r * V::new(angle.cos(), angle.sin());
        }
        self.start + t * (self.end - self.start)
    }

    fn arc_event(&self, events: &mut Vec<f64>, at: V) {
        let Some((c, _, sweep)) = self.arc else {
            unreachable!()
        };
        let from = self.start - c;
        let to = at - c;
        let angle = cross(from, to).atan2(from.dot(&to));
        let along = (angle * sweep.signum()).rem_euclid(TAU);
        push_event(events, along / sweep.abs());
    }

    fn line_events(&self, events: &mut Vec<f64>, origin: V, normal: V, offset: f64) {
        if let Some((c, r, _)) = self.arc {
            let distance = offset - (c - origin).dot(&normal);
            if distance.abs() <= r {
                let tangent = V::new(-normal.y, normal.x);
                let leg = ((r - distance.abs()) * (r + distance.abs())).sqrt();
                for sign in [-1.0, 1.0] {
                    self.arc_event(events, c + distance * normal + sign * leg * tangent);
                }
            }
        } else {
            let slope = (self.end - self.start).dot(&normal);
            if slope != 0.0 {
                push_event(
                    events,
                    (offset - (self.start - origin).dot(&normal)) / slope,
                );
            }
        }
    }

    fn circle_events(&self, events: &mut Vec<f64>, center: V, radius: f64) {
        if let Some((c, r, _)) = self.arc {
            let d = center - c;
            let distance = d.norm();
            if distance > 0.0 && distance <= r + radius && distance >= (r - radius).abs() {
                // Радикальная ось двух окружностей, относительно центра c.
                let offset = (distance * distance + (r - radius) * (r + radius)) / (2.0 * distance);
                self.line_events(events, c, d / distance, offset);
            }
        } else {
            let d = self.end - self.start;
            let len = d.norm();
            let unit = d / len;
            let offset = center - self.start;
            let height = cross(unit, offset).abs();
            if height <= radius {
                let projection = offset.dot(&unit);
                let leg = ((radius - height) * (radius + height)).sqrt();
                push_event(events, (projection - leg) / len);
                push_event(events, (projection + leg) / len);
            }
        }
    }
}

fn push_event(events: &mut Vec<f64>, t: f64) {
    if t > 0.0 && t < 1.0 {
        events.push(t);
    }
}

fn inside(poly: &Polygon, p: V, margin: f64) -> bool {
    let mut parity = false;
    for (a, b) in boundaries(poly) {
        if distance_edge(p, a, b) <= margin {
            return false;
        }
        if (a.y > p.y) != (b.y > p.y) {
            parity ^= (cross(b - a, p - a) > 0.0) == (b.y > a.y);
        }
    }
    parity
}

struct Opening {
    center: V,
    axes: [V; 2],
    core: V,
    radius: f64,
}

impl Opening {
    fn new(shape: RoundedRect) -> Self {
        let (sin, cos) = shape.angle_rad.sin_cos();
        Self {
            center: point(shape.center),
            axes: [V::new(cos, sin), V::new(-sin, cos)],
            core: V::new(shape.width_m / 2.0, shape.height_m / 2.0) - V::repeat(shape.radius_m),
            radius: shape.radius_m,
        }
    }

    fn reaches(&self, p: V, margin: f64) -> bool {
        let offset = p - self.center;
        let local = V::from(self.axes.map(|axis| offset.dot(&axis).abs()));
        (local - self.core).map(|x| x.max(0.0)).norm() <= self.radius + margin
    }

    fn events(&self, motion: &Motion, events: &mut Vec<f64>, margin: f64) {
        let radius = self.radius + margin;
        for (axis, half) in self.axes.into_iter().zip(self.core.iter().copied()) {
            for sign in [-1.0, 1.0] {
                motion.line_events(events, self.center, axis, sign * (half + radius));
            }
        }
        for x in [-self.core.x, self.core.x] {
            for y in [-self.core.y, self.core.y] {
                let corner = self.center + x * self.axes[0] + y * self.axes[1];
                motion.circle_events(events, corner, radius);
            }
        }
    }
}

fn openings(layout: &Layout, layer: &str) -> Vec<Opening> {
    let bores = layout
        .vias
        .iter()
        .filter(|via| via.layers.iter().any(|name| name == layer))
        .map(|via| RoundedRect {
            center: via.at,
            angle_rad: 0.0,
            width_m: via.drill_m,
            height_m: via.drill_m,
            radius_m: via.drill_m / 2.0,
        });
    let holes = layout
        .footprints
        .iter()
        .flat_map(|fp| &fp.pads)
        .filter_map(|pad| pad.hole);
    bores.chain(holes).map(Opening::new).collect()
}

fn gap_intervals(
    motion: &Motion,
    length: f64,
    radius: f64,
    copper: &Polygon,
    substrate: &[Polygon],
    openings: &[Opening],
) -> Vec<[f64; 2]> {
    let mut events = vec![0.0, 1.0];
    for (a, b) in std::iter::once(copper)
        .chain(substrate)
        .flat_map(boundaries)
    {
        // Граница капсулы ребра состоит из параллельных прямых и торцевых дуг.
        // Лишние пересечения лишь дробят интервал с неизменной классификацией.
        let d = (b - a).normalize();
        let normal = V::new(-d.y, d.x);
        motion.line_events(&mut events, a, normal, radius);
        motion.line_events(&mut events, a, normal, -radius);
        motion.circle_events(&mut events, a, radius);
        motion.circle_events(&mut events, b, radius);
    }
    for opening in openings {
        opening.events(motion, &mut events, radius);
    }
    events.sort_by(f64::total_cmp);
    events.dedup();
    let supported = |t| {
        let p = motion.at(t);
        inside(copper, p, radius)
            && substrate.iter().any(|poly| inside(poly, p, radius))
            && !openings.iter().any(|hole| hole.reaches(p, radius))
    };
    let mut gaps: Vec<[f64; 2]> = Vec::new();
    let mut add = |a, b| {
        let a = a * length;
        let b = b * length;
        if let Some(last) = gaps.last_mut().filter(|v| v[1] == a) {
            last[1] = b;
        } else {
            gaps.push([a, b]);
        }
    };
    for w in events.windows(2) {
        let [a, b] = [w[0], w[1]];
        if !supported(a) {
            add(a, a);
        }
        if !supported(f64::midpoint(a, b)) {
            add(a, b);
        }
    }
    if !supported(1.0) {
        add(1.0, 1.0);
    }
    gaps
}

/// Проверка проекции всей ширины дорожки на привязанную заливку.
/// При нескольких областях результат остаётся открытым, если нужно объединить их геометрию.
pub fn check_reference(
    layout: &Layout,
    input: &ReferenceInput<'_>,
) -> Result<ReferenceReport, Error> {
    validate_layout(layout)?;
    let t = track(layout, input.track)?;
    let net = validate_plane(layout, input.plane)?;
    require(
        t.layer != input.plane.layer && t.net.is_some() && t.net != Some(net),
        "некорректная пара трассы и опоры",
    )?;
    let mut report = ReferenceReport {
        status: Status::Pass,
        height_m: None,
        center_gaps_m: None,
        copper_gaps_m: None,
    };
    let height = stack_between(layout, &t.layer, input.plane.layer);
    report.height_m = height.clone().ok();
    let geometric = (|| {
        let Some(selected) = active_plane(layout, input.plane)? else {
            return Ok(false);
        };
        let substrate = layout.polygonal_substrate()?;
        let holes = openings(layout, input.plane.layer);
        let tie = point(input.plane.tie);
        if !substrate.iter().any(|poly| inside(poly, tie, 0.0))
            || holes.iter().any(|hole| hole.reaches(tie, 0.0))
        {
            return Ok(false);
        }
        let m = Motion::new(t.path);
        let length = t.path.length_m();
        let center = gap_intervals(&m, length, 0.0, selected, &substrate, &holes);
        let full = gap_intervals(&m, length, t.width_m / 2.0, selected, &substrate, &holes);
        let clear = full.is_empty();
        report.center_gaps_m = Some(center);
        report.copper_gaps_m = Some(full);
        if !clear
            && layout.zones[input.plane.zone]
                .fill(input.plane.layer)?
                .len()
                > 1
        {
            return Err(Open::MultipleRegions);
        }
        if clear && !on_same_substrate(selected, &substrate) {
            return Err(Open::ClippedPlane);
        }
        Ok(clear)
    })();
    report.status = match geometric {
        Ok(false) => Status::Fail,
        Ok(true) => status(
            height
                .and_then(|_| adjacent_plane(layout, &t.layer, input.plane.layer))
                .and_then(|_| {
                    evidence(
                        input.plane.connection,
                        input.scope,
                        "привязка опорной плоскости",
                    )
                }),
        ),
        Err(open) => Status::NotAnalyzed(open),
    };
    Ok(report)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Element {
    Track(usize),
    Via(usize),
}

struct Part<'a> {
    element: Element,
    from: Node<'a>,
    to: Node<'a>,
    length: Available<f64>,
}

struct Routed<'a> {
    end: Node<'a>,
    parts: Vec<Part<'a>>,
}

impl Routed<'_> {
    fn length(&self) -> Available<f64> {
        self.parts.iter().map(|part| part.length.clone()).sum()
    }
}

fn key(node: Node<'_>) -> (&str, i32, i32) {
    (node.layer, node.at.x_nm, node.at.y_nm)
}

fn on_segment(p: Point, start: Point, end: Point) -> bool {
    let offset = |p: Point| {
        [
            i128::from(p.x_nm) - i128::from(start.x_nm),
            i128::from(p.y_nm) - i128::from(start.y_nm),
        ]
    };
    let [x, y] = offset(p);
    let [dx, dy] = offset(end);
    let dot = x * dx + y * dy;
    x * dy == y * dx && (0..=dx * dx + dy * dy).contains(&dot)
}

fn route<'a>(layout: &Layout, route: Route<'a>) -> Result<Routed<'a>, Error> {
    require(
        layout.nets.contains_key(&route.net) && known_layer(layout, route.start.layer),
        "неизвестная цепь или слой маршрута",
    )?;
    let mut at = route.start;
    let mut visited = BTreeSet::from([key(at)]);
    let mut parts = Vec::new();
    let mut push = |part: Part<'a>| -> Result<(), Error> {
        require(
            visited.insert(key(part.to)),
            "маршрут содержит замкнутый обход",
        )?;
        parts.push(part);
        Ok(())
    };
    for step in route.steps {
        match *step {
            Step::Track { index, to } => {
                let t = track(layout, index)?;
                require(
                    t.net == Some(route.net) && t.layer == at.layer && at.at != to,
                    "дорожка маршрута имеет другую цепь, слой или нулевую длину",
                )?;
                let [start, end] = ends(t.path);
                let length = match t.path {
                    Centerline::Segment { .. } => {
                        require(
                            on_segment(at.at, start, end) && on_segment(to, start, end),
                            "разрыв маршрута на дорожке",
                        )?;
                        (point(to) - point(at.at)).norm()
                    }
                    Centerline::Arc { .. } => {
                        require(
                            [at.at, to] == [start, end] || [to, at.at] == [start, end],
                            "для дуги маршрута нужны оба конца",
                        )?;
                        t.path.length_m()
                    }
                };
                let next = Node { at: to, ..at };
                let thickness = layout
                    .stackup()
                    .and_then(|s| s.copper_layer(at.layer))
                    .and_then(|l| l.thickness_m());
                push(Part {
                    element: Element::Track(index),
                    from: at,
                    to: next,
                    length: thickness.map(|_| length).map_err(Open::from),
                })?;
                at = next;
            }
            Step::Via { index, to_layer } => {
                let via = layout.vias.get(index).ok_or(Error("неизвестный переход"))?;
                require(
                    via.net == Some(route.net) && via.at == at.at && at.layer != to_layer,
                    "разрыв маршрута на переходе",
                )?;
                let idx = |name| {
                    via.layers
                        .iter()
                        .position(|l| l == name)
                        .ok_or(Error("переход не проходит через слой маршрута"))
                };
                let a = idx(at.layer)?;
                let b = idx(to_layer)?;
                let next = Node {
                    layer: to_layer,
                    ..at
                };
                // Внутренние узлы перехода для пересечения ветвей нужны с тем же сроком жизни,
                // что и начало маршрута. Здесь сохраняется целый пролёт; неполное перекрытие
                // пролётов на общей стенке отдельно блокирует результат R–L.
                let length = stack_between(layout, &via.layers[a], &via.layers[b]);
                push(Part {
                    element: Element::Via(index),
                    from: at,
                    to: next,
                    length,
                })?;
                at = next;
            }
        }
    }
    Ok(Routed { end: at, parts })
}

fn terminal<'a>(layout: &Layout, addr: Terminal<'a>, net: u32) -> Result<Node<'a>, Error> {
    let mut rows = layout
        .footprints
        .iter()
        .filter(|f| f.reference.as_deref() == Some(addr.reference))
        .flat_map(|f| f.pads.iter())
        .filter(|p| {
            p.number == addr.pad
                && p.land
                    .as_ref()
                    .is_some_and(|land| land.layers.iter().any(|l| l == addr.layer))
        });
    let pad = rows
        .next()
        .ok_or(Error("не найдена медная площадка клеммы"))?;
    require(
        rows.next().is_none() && pad.net == Some(net),
        "неоднозначная площадка или другая цепь клеммы",
    )?;
    Ok(Node {
        at: pad.anchor,
        layer: addr.layer,
    })
}

fn validate_pair(pads: [Terminal<'_>; 2]) -> Result<(), Error> {
    require(
        pads[0].reference == pads[1].reference && pads[0].pad != pads[1].pad,
        "нужны две разные площадки одной детали",
    )
}

fn span(layout: &Layout, nodes: [Node<'_>; 2]) -> Available<f64> {
    let [a, b] = nodes;
    let dz = stack_between(layout, a.layer, b.layer)?;
    Ok((point(a.at) - point(b.at)).norm().hypot(dz))
}

fn route_materials(layout: &Layout, routed: &Routed<'_>) -> Available<()> {
    let stack = layout.stackup()?;
    for part in &routed.parts {
        for layer in stack.dielectrics_between(part.from.layer, part.to.layer)? {
            layer.material()?;
        }
    }
    Ok(())
}

fn length_limit(value: Available<f64>, limit: Available<f64>, proven: Available<()>) -> Status {
    match (value, limit, proven) {
        (Ok(v), Ok(max), Ok(())) if v > max => Status::Fail,
        (Ok(_), Ok(_), Ok(())) => Status::Pass,
        (Err(open), _, _) | (_, Err(open), _) | (_, _, Err(open)) => Status::NotAnalyzed(open),
    }
}

/// Сверяет действительный монтажный контур развязки с заданным пределом.
pub fn check_decoupling(
    layout: &Layout,
    input: &DecouplingInput<'_>,
) -> Result<DecouplingReport, Error> {
    validate_layout(layout)?;
    valid_source(input.topology)?;
    validate_datum(input.max_perimeter_m, nonnegative)?;
    validate_pair(input.load)?;
    validate_pair(input.capacitor)?;
    require(
        input.load[0].reference != input.capacitor[0].reference
            && input.supply.net != input.return_arm.net,
        "контур развязки требует разных деталей и цепей",
    )?;
    let [supply, return_arm] = [input.supply, input.return_arm];
    let starts = [
        terminal(layout, input.load[0], supply.net)?,
        terminal(layout, input.load[1], return_arm.net)?,
    ];
    let ends = [
        terminal(layout, input.capacitor[0], supply.net)?,
        terminal(layout, input.capacitor[1], return_arm.net)?,
    ];
    let paths = [route(layout, supply)?, route(layout, return_arm)?];
    require(
        [supply.start, return_arm.start] == starts && [paths[0].end, paths[1].end] == ends,
        "маршрут развязки разомкнут у клеммы",
    )?;
    let dimensions = (|| {
        let arms = [paths[0].length()?, paths[1].length()?];
        Ok((
            arms,
            arms.iter().sum::<f64>() + span(layout, starts)? + span(layout, ends)?,
        ))
    })();
    let perimeter = dimensions
        .as_ref()
        .map(|(_, perimeter)| *perimeter)
        .map_err(Clone::clone);
    if let Ok(value) = perimeter {
        finite(value)?;
    }
    let proven = evidence(input.topology, input.scope, "схема и контур развязки")
        .and_then(|_| route_materials(layout, &paths[0]))
        .and_then(|_| route_materials(layout, &paths[1]));
    Ok(DecouplingReport {
        status: length_limit(
            perimeter.clone(),
            data(
                input.max_perimeter_m,
                input.scope,
                "предел контура развязки",
            ),
            proven,
        ),
        arm_lengths_m: dimensions.ok().map(|(arms, _)| arms),
        perimeter_m: perimeter.ok(),
    })
}

fn via_hits_plane(via: &super::layout::Via, poly: &Polygon) -> bool {
    let p = point(via.at);
    let nearest = if poly.locate(via.at) == Location::Outside {
        boundaries(poly)
            .map(|(a, b)| distance_edge(p, a, b))
            .fold(f64::INFINITY, f64::min)
    } else {
        0.0
    };
    let farthest = poly
        .outline()
        .points()
        .iter()
        .map(|&at| (point(at) - p).norm())
        .fold(0.0, f64::max);
    // Перекрытие с кольцом должно иметь положительную площадь.
    nearest < via.diameter_m / 2.0 && farthest > via.drill_m / 2.0
}

fn clear_approach(
    layout: &Layout,
    layer: &str,
    start: V,
    end: V,
    plane: &Polygon,
    substrate: &[Polygon],
) -> bool {
    let holes = openings(layout, layer);
    let length = (end - start).norm();
    let motion = Motion {
        start,
        end,
        arc: None,
    };
    // Начальный разрыв допустим в пределах проверяемого подхода от сигнального
    // перехода до меди. После входа в медь щели и прорези уже меняют путь возврата.
    inside(plane, end, 0.0)
        && substrate.iter().any(|board| inside(board, end, 0.0))
        && !holes.iter().any(|hole| hole.reaches(end, 0.0))
        && (length == 0.0
            || gap_intervals(&motion, length, 0.0, plane, substrate, &holes)
                .iter()
                .all(|gap| gap[0] == 0.0 && gap[1] < length))
}

/// Проверяет локальный возврат в месте перехода сигнала на другой слой.
/// Покрытие вдоль каждой дорожки проверяется отдельно через `check_reference`.
pub fn check_transition(
    layout: &Layout,
    input: &TransitionInput<'_>,
) -> Result<TransitionReport, Error> {
    validate_layout(layout)?;
    validate_datum(input.max_detour_m, nonnegative)?;
    let [a, b] = [
        track(layout, input.tracks[0])?,
        track(layout, input.tracks[1])?,
    ];
    let [first, second] = input.planes;
    let nets = [
        validate_plane(layout, first)?,
        validate_plane(layout, second)?,
    ];
    let via = layout
        .vias
        .get(input.signal_via)
        .ok_or(Error("неизвестный сигнальный переход"))?;
    require(
        a.layer != b.layer
            && a.net.is_some()
            && a.net == b.net
            && via.net == a.net
            && via.layers.contains(&a.layer)
            && via.layers.contains(&b.layer)
            && ends(a.path).contains(&via.at)
            && ends(b.path).contains(&via.at)
            && a.layer != first.layer
            && b.layer != second.layer
            && nets.into_iter().all(|net| Some(net) != via.net),
        "разрыв сигнального пути в месте смены слоя",
    )?;
    let mut copper_arms = None;
    let mut stitch = None;
    let mut capacitor_source = Ok(());
    match input.transfer {
        Some(Transfer::StitchVia(index)) => {
            stitch = Some(
                layout
                    .vias
                    .get(index)
                    .ok_or(Error("неизвестный опорный переход"))?,
            );
        }
        Some(Transfer::Capacitor {
            arms,
            pads,
            coupling,
        }) => {
            valid_source(coupling)?;
            validate_pair(pads)?;
            require(
                [arms[0].net, arms[1].net] == nets
                    && nets[0] != nets[1]
                    && arms[0].start.layer == first.layer
                    && arms[1].start.layer == second.layer,
                "ветви конденсатора не привязаны к плоскостям",
            )?;
            let paths = [route(layout, arms[0])?, route(layout, arms[1])?];
            let terminals = [
                terminal(layout, pads[0], nets[0])?,
                terminal(layout, pads[1], nets[1])?,
            ];
            require(
                [paths[0].end, paths[1].end] == terminals,
                "разрыв ветви у конденсатора",
            )?;
            copper_arms = Some((arms, paths, terminals));
            capacitor_source = evidence(
                coupling,
                input.scope,
                "применимость конденсатора смены опоры",
            );
        }
        None => {}
    }
    let mut report = TransitionReport {
        status: Status::Pass,
        detour_m: None,
    };
    let good = || -> Available<bool> {
        let selected = [active_plane(layout, first)?, active_plane(layout, second)?];
        let [Some(p), Some(q)] = selected else {
            return Ok(false);
        };
        let substrate = layout.polygonal_substrate()?;
        let on_board = |at, margin| substrate.iter().any(|poly| inside(poly, point(at), margin));
        if !on_board(via.at, via.diameter_m / 2.0) {
            return Ok(false);
        }
        if [p, q]
            .into_iter()
            .any(|poly| !on_same_substrate(poly, &substrate))
        {
            return Err(Open::ClippedPlane);
        }
        if [(first, p), (second, q)].into_iter().any(|(plane, poly)| {
            via.layers.iter().any(|layer| layer == plane.layer) && via_hits_plane(via, poly)
        }) {
            return Ok(false);
        }
        for (track, plane) in [(a, first), (b, second)] {
            stack_between(layout, &track.layer, plane.layer)?;
            adjacent_plane(layout, &track.layer, plane.layer)?;
            evidence(plane.connection, input.scope, "привязка опорной плоскости")?;
            if !on_board(plane.tie, 0.0)
                || openings(layout, plane.layer)
                    .iter()
                    .any(|hole| hole.reaches(point(plane.tie), 0.0))
            {
                return Ok(false);
            }
        }
        if first.zone == second.zone && first.layer == second.layer && std::ptr::eq(p, q) {
            // Даже непрерывная опора должна обходить антипад сигнального
            // перехода. Его локальный обход здесь не рассчитан.
            data(input.max_detour_m, input.scope, "предел обхода смены опоры")?;
            return Err(Open::DetourGeometry);
        }
        let (length, direct) = if let Some(stitch) = stitch {
            if nets[0] != nets[1]
                || stitch.net != Some(nets[0])
                || !stitch.layers.iter().any(|l| l == first.layer)
                || !stitch.layers.iter().any(|l| l == second.layer)
                || first.layer == second.layer
                || !on_board(stitch.at, stitch.diameter_m / 2.0)
                || !via_hits_plane(stitch, p)
                || !via_hits_plane(stitch, q)
            {
                return Ok(false);
            }
            stack_between(layout, first.layer, second.layer)?;
            let start = point(via.at);
            let axis = point(stitch.at) - start;
            let length = axis.norm();
            // Радиус середины кольца проверяет подход по меди снаружи сверла.
            let annulus = (stitch.diameter_m + stitch.drill_m) / 4.0;
            if length <= annulus {
                return Ok(false);
            }
            let end = start + (1.0 - annulus / length) * axis;
            (
                length,
                [(first.layer, p), (second.layer, q)]
                    .into_iter()
                    .all(|(layer, plane)| {
                        clear_approach(layout, layer, start, end, plane, &substrate)
                    }),
            )
        } else if let Some((arms, paths, terminals)) = copper_arms {
            let mut approaches = 0.0;
            let mut direct = true;
            for (arm, poly) in arms.into_iter().zip([p, q]) {
                if poly.locate(arm.start.at) != Location::Inside || !on_board(arm.start.at, 0.0) {
                    return Ok(false);
                }
                approaches += (point(arm.start.at) - point(via.at)).norm();
                direct &= clear_approach(
                    layout,
                    arm.start.layer,
                    point(via.at),
                    point(arm.start.at),
                    poly,
                    &substrate,
                );
            }
            capacitor_source?;
            (
                approaches + paths[0].length()? + paths[1].length()? + span(layout, terminals)?,
                direct,
            )
        } else {
            return Err(Open::Transfer);
        };
        report.detour_m = Some(length);
        if length > data(input.max_detour_m, input.scope, "предел обхода смены опоры")?
        {
            return Ok(false);
        }
        if !direct {
            return Err(Open::DetourGeometry);
        }
        Ok(true)
    };
    report.status = match good() {
        Ok(true) => Status::Pass,
        Ok(false) => Status::Fail,
        Err(open) => Status::NotAnalyzed(open),
    };
    if let Some(value) = report.detour_m {
        finite(value)?;
    }
    Ok(report)
}

fn track_overlap(a: &Part<'_>, b: &Part<'_>, layout: &Layout) -> bool {
    if a.element != b.element {
        return false;
    }
    match a.element {
        Element::Via(index) => {
            let layers = &layout.vias[index].layers;
            let range = |part: &Part<'_>| {
                let i = layers
                    .iter()
                    .position(|layer| layer == part.from.layer)
                    .unwrap();
                let j = layers
                    .iter()
                    .position(|layer| layer == part.to.layer)
                    .unwrap();
                i.min(j)..i.max(j)
            };
            let (a, b) = (range(a), range(b));
            a.start.max(b.start) < a.end.min(b.end)
        }
        Element::Track(index) => {
            if matches!(layout.tracks[index].path, Centerline::Arc { .. }) {
                return true;
            }
            let order = |p: Point| (p.x_nm, p.y_nm);
            let range = |part: &Part<'_>| {
                let (a, b) = (order(part.from.at), order(part.to.at));
                a.min(b)..a.max(b)
            };
            let (a, b) = (range(a), range(b));
            a.start.max(b.start) < a.end.min(b.end)
        }
    }
}

fn check_return_tree<'a>(
    layout: &'a Layout,
    net: u32,
    star: Node<'a>,
    power: &Routed<'a>,
    signal: &Routed<'a>,
) -> Available<()> {
    let tracks = layout.tracks.iter().filter(|t| t.net == Some(net));
    let vias = layout.vias.iter().filter(|v| v.net == Some(net));
    let mut taps = BTreeMap::<&'a str, BTreeSet<(i32, i32)>>::new();
    let mut add_tap = |n: Node<'a>| {
        taps.entry(n.layer)
            .or_default()
            .insert((n.at.x_nm, n.at.y_nm));
    };
    for t in tracks.clone() {
        for at in ends(t.path) {
            add_tap(Node {
                at,
                layer: &t.layer,
            });
        }
    }
    for v in vias.clone() {
        for layer in &v.layers {
            add_tap(Node { at: v.at, layer });
        }
    }
    for p in power.parts.iter().chain(&signal.parts) {
        add_tap(p.from);
        add_tap(p.to);
    }

    let mut adjacent = BTreeMap::<_, Vec<_>>::new();
    let mut connect = |a, b| {
        adjacent.entry(a).or_default().push(b);
        adjacent.entry(b).or_default().push(a);
    };
    for t in tracks {
        let layer = t.layer.as_str();
        let [a, b] = ends(t.path);
        match t.path {
            Centerline::Segment { .. } => {
                // Лексикографический порядок монотонен вдоль прямолинейного участка.
                let mut previous = None;
                for &(x, y) in &taps[layer] {
                    if on_segment(Point { x_nm: x, y_nm: y }, a, b) {
                        let current = (layer, x, y);
                        if let Some(last) = previous {
                            connect(last, current);
                        }
                        previous = Some(current);
                    }
                }
            }
            Centerline::Arc { mid, .. } => {
                if taps[layer].contains(&(mid.x_nm, mid.y_nm)) {
                    return Err(Open::SharedTopology);
                }
                connect((layer, a.x_nm, a.y_nm), (layer, b.x_nm, b.y_nm));
            }
        }
    }
    for v in vias {
        for pair in v.layers.windows(2) {
            connect(
                (pair[0].as_str(), v.at.x_nm, v.at.y_nm),
                (pair[1].as_str(), v.at.x_nm, v.at.y_nm),
            );
        }
    }
    let mut visited = BTreeSet::new();
    let mut pending = vec![(key(star), None)];
    while let Some((n, parent)) = pending.pop() {
        if !visited.insert(n) {
            return Err(Open::SharedTopology);
        }
        // Второе совпадение с родителем означает параллельную ветвь.
        let mut parent_seen = false;
        for &neighbor in adjacent.get(&n).into_iter().flatten() {
            if Some(neighbor) == parent && !std::mem::replace(&mut parent_seen, true) {
                continue;
            }
            pending.push((neighbor, Some(n)));
        }
    }
    if !visited.contains(&key(power.end)) || !visited.contains(&key(signal.end)) {
        return Err(Open::SharedTopology);
    }
    Ok(())
}

fn shared_prefix<'a, 'b>(
    layout: &Layout,
    power: &'b Routed<'a>,
    signal: &'b Routed<'a>,
) -> Available<Vec<&'b Part<'a>>> {
    let mut shared = Vec::new();
    for (a, b) in power.parts.iter().zip(&signal.parts) {
        if a.from != b.from || a.element != b.element {
            break;
        }
        if a.to == b.to {
            shared.push(a);
        } else {
            if !matches!(a.element, Element::Track(_)) || !track_overlap(a, b, layout) {
                return Err(Open::SharedTopology);
            }
            shared.push(if a.length.clone()? < b.length.clone()? {
                a
            } else {
                b
            });
            break;
        }
    }
    let overlaps = |parts: &[Part<'_>]| {
        parts
            .iter()
            .enumerate()
            .any(|(i, a)| parts[i + 1..].iter().any(|b| track_overlap(a, b, layout)))
    };
    if overlaps(&power.parts) || overlaps(&signal.parts) {
        return Err(Open::SharedTopology);
    }
    for (i, a) in power.parts.iter().enumerate() {
        for (j, b) in signal.parts.iter().enumerate() {
            if (i != j || i >= shared.len()) && track_overlap(a, b, layout) {
                return Err(Open::SharedTopology);
            }
        }
    }
    Ok(shared)
}

fn section_r(
    layout: &Layout,
    input: &GroundBounceInput<'_>,
    part: &Part<'_>,
    rho: f64,
) -> Available<f64> {
    let length = part.length.clone()?;
    let r_at_reference = match part.element {
        Element::Track(index) => {
            let layer = layout.stackup()?.copper_layer(part.from.layer)?;
            trace_resistance(
                length,
                layout.tracks[index].width_m,
                layer.thickness_m()?,
                20.0,
            )
        }
        Element::Via(index) => {
            let parameter = input
                .plating_m
                .iter()
                .find(|(i, _)| *i == index)
                .map(|(_, d)| *d);
            let plating = data(
                parameter,
                input.scope,
                "толщина металлизации общего перехода",
            )?;
            via_resistance(layout.vias[index].drill_m, plating, length, 20.0)
        }
    };
    // Обе формулы pcb.rs здесь вызваны при опорных 20 °C. Подставляем ρ
    // конкретного проводника в рабочем режиме без температурного умолчания.
    Ok((rho / CU_RHO_20C) * r_at_reference)
}

/// Проверяет смещение сигнальной земли в заданном режиме по общей части двух путей.
/// Значение знаковое. Статус использует интервал модуля с границей неучтённых вкладов.
pub fn check_ground_bounce(
    layout: &Layout,
    input: &GroundBounceInput<'_>,
) -> Result<GroundBounceReport, Error> {
    validate_layout(layout)?;
    valid_source(input.topology)?;
    validate_datum(input.resistivity_ohm_m, positive)?;
    for datum in [
        input.shared_inductance_h,
        input.max_bounce_v,
        input.omitted_bounce_v,
    ] {
        validate_datum(datum, nonnegative)?;
    }
    for datum in [input.current_a, input.slew_a_per_s] {
        validate_datum(datum, |_| true)?;
    }
    let mut plating = BTreeSet::new();
    for &(i, datum) in input.plating_m {
        let via = layout
            .vias
            .get(i)
            .ok_or(Error("неизвестный переход в данных металлизации"))?;
        require(
            plating.insert(i) && via.net == Some(input.power.net),
            "повтор металлизации или другая цепь перехода",
        )?;
        validate_datum(Some(datum), |p| p > 0.0 && p < via.drill_m / 2.0)?;
    }
    require(
        input.power.net == input.signal.net && input.power.start == input.signal.start,
        "два возврата должны начинаться в одной звезде и лежать в одной цепи",
    )?;
    let power = route(layout, input.power)?;
    let signal = route(layout, input.signal)?;
    require(
        power.end != signal.end && !power.parts.is_empty() && !signal.parts.is_empty(),
        "нужны два отдельных маршрута возврата",
    )?;
    let mut report = GroundBounceReport {
        status: Status::Pass,
        shared_length_m: None,
        resistance_ohm: None,
        inductance_h: None,
        bounce_v: None,
        bounds_v: None,
    };
    let mut calculate = || -> Result<Available<bool>, Error> {
        let geometry = (|| {
            evidence(
                input.topology,
                input.scope,
                "единственность путей и вводов тока",
            )?;
            if layout.has_unmodeled_contacts(input.power.net) {
                return Err(Open::SharedTopology);
            }
            let route_layers: BTreeSet<_> = power
                .parts
                .iter()
                .chain(&signal.parts)
                .flat_map(|part| [part.from.layer, part.to.layer])
                .collect();
            for zone in layout
                .zones
                .iter()
                .filter(|z| z.net == Some(input.power.net))
            {
                for layer in zone.layers() {
                    zone.fill(layer)?;
                    // Модель R–L не распределяет ток между дорожками и заливкой
                    // на слоях заданных возвратов.
                    if route_layers.contains(layer.as_str())
                        || layout
                            .vias
                            .iter()
                            .filter(|via| {
                                via.net == Some(input.power.net) && via.layers.contains(layer)
                            })
                            .take(2)
                            .count()
                            > 1
                    {
                        return Err(Open::SharedTopology);
                    }
                }
            }
            if layout.footprints.iter().flat_map(|fp| &fp.pads).any(|pad| {
                pad.net == Some(input.power.net) && pad.kind == super::kicad::PadKind::PlatedHole
            }) {
                return Err(Open::SharedTopology);
            }
            route_materials(layout, &power)?;
            route_materials(layout, &signal)?;
            power.length()?;
            signal.length()?;
            check_return_tree(layout, input.power.net, input.power.start, &power, &signal)?;
            shared_prefix(layout, &power, &signal)
        })();
        let shared = match geometry {
            Ok(shared) => shared,
            Err(open) => return Ok(Err(open)),
        };
        let length = finite(
            shared
                .iter()
                .map(|part| part.length.clone())
                .sum::<Available<f64>>()
                .unwrap(),
        )?;
        report.shared_length_m = Some(length);
        let values = (|| {
            let rho = data(
                input.resistivity_ohm_m,
                input.scope,
                "удельное сопротивление материала в режиме",
            )?;
            let resistance = shared
                .iter()
                .map(|part| section_r(layout, input, part, rho))
                .sum::<Available<f64>>()?;
            let inductance = if shared.is_empty() {
                0.0 // Пустой общий путь в данной модели.
            } else {
                data(
                    input.shared_inductance_h,
                    input.scope,
                    "индуктивность общей части",
                )?
            };
            let current = data(input.current_a, input.scope, "ток силового возврата")?;
            let slew = data(
                input.slew_a_per_s,
                input.scope,
                "производная тока силового возврата",
            )?;
            Ok((resistance, inductance, current, slew))
        })();
        let (r, l, i, di_dt) = match values {
            Ok(values) => values,
            Err(open) => return Ok(Err(open)),
        };
        require(
            shared.is_empty() || positive(r),
            "некорректное сопротивление общего участка",
        )?;
        let resistive = finite(r * i)?;
        let inductive = finite(l * di_dt)?;
        let v = finite(resistive + inductive)?;
        report.resistance_ohm = Some(r);
        report.inductance_h = Some(l);
        report.bounce_v = Some(v);
        let omitted = match data(
            input.omitted_bounce_v,
            input.scope,
            "граница неучтённых вкладов",
        ) {
            Ok(value) => value,
            Err(open) => return Ok(Err(open)),
        };
        // Нижняя граница следует из конкретной точки. Для верхней берём
        // неравенство треугольника: знаки вкладов могут меняться в окне режима.
        let bounds = [
            (v.abs() - omitted).max(0.0),
            finite(resistive.abs() + inductive.abs() + omitted)?,
        ];
        report.bounds_v = Some(bounds);
        Ok(data(
            input.max_bounce_v,
            input.scope,
            "предел смещения сигнальной опоры",
        )
        .and_then(|max| {
            if bounds[0] > max {
                Ok(false)
            } else if bounds[1] <= max {
                Ok(true)
            } else {
                Err(Open::Uncertainty)
            }
        }))
    };
    report.status = match calculate()? {
        Ok(true) => Status::Pass,
        Ok(false) => Status::Fail,
        Err(open) => Status::NotAnalyzed(open),
    };
    Ok(report)
}
