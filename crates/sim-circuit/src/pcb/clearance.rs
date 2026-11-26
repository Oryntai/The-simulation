//! Зазор и путь утечки между заданными цепями на одной наружной стороне платы.
//! Геометрия меди из KiCad 8.0/9.0: [Tracks](https://dev-docs.kicad.org/en/file-formats/sexpr-pcb/#tracks-section),
//! [Footprint и Zone](https://dev-docs.kicad.org/en/file-formats/sexpr-intro/).
//! Пределы и их область задаются входом. См. [IEC 60664-1:2020, область применения](https://webstore.iec.ch/en/publication/59671).
//! Модель описывает открытую плоскую поверхность и медь без учёта защитных свойств маски.
//! Торцы, противоположная сторона, покрытия, детали, паяные выступы и объёмная
//! изоляция требуют собственных проверок. Результат относится только к заданной паре и стороне.

use super::kicad::METRES_PER_NM;
use super::layout::{Centerline, Layout, NotAnalyzed, Point, Polygon, RoundedRect};
use nalgebra::Vector2;
use std::f64::consts::TAU;
use std::fmt;

pub const HAZARD_CODE: &str = "creepage_clearance";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Face {
    Front,
    Back,
}

impl Face {
    fn layer(self) -> &'static str {
        match self {
            Self::Front => "F.Cu",
            Self::Back => "B.Cu",
        }
    }
}

/// Синтетический предел допустим только для своей расчётной постановки.
/// Паспортные или измеренные входы описывают указанную наружную поверхность.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Scope {
    Reference,
    Sourced,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Conditions<'a> {
    pub scope: Scope,
    /// Источник огибающей напряжений и условий эксплуатации этой платы.
    pub source: &'a str,
    /// Материал открытой поверхности, идентифицированный источником.
    pub material: &'a str,
    /// Одинаковый идентификатор применимой среды в условиях и правиле, с
    /// загрязнением, влажностью, температурой и состоянием поверхности.
    pub environment: &'a str,
    pub working_v: f64,
    pub peak_v: f64,
    pub frequency_hz: f64,
    pub altitude_m: f64,
    /// Граница изменения обоих расстояний из-за геометрии и изготовления
    /// в пределах указанной области. Ноль допустим для точного эталона.
    /// При сравнении с огибающей правила используется только значение установки.
    pub distance_error_m: Option<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SlotCredit<'a> {
    /// Замкнутые прямоугольные окна. Порог номинальной ширины из источника
    /// правила уже включает допуск фрезеровки и метод измерения прорези.
    Rectangular { minimum_width_m: f64 },
    /// Источник подтверждает применимость всех показанных фрезерованных
    /// прорезей на этой стороне, включая ширину, допуск и условия среды.
    Verified { source: &'a str },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Limits<'a> {
    /// Верхние границы применимости. Источник должен покрывать всю указанную огибающую.
    pub envelope: Conditions<'a>,
    pub clearance_m: Option<f64>,
    pub creepage_m: Option<f64>,
    /// При отсутствии подтверждения прорези не увеличивают нижнюю оценку для допуска.
    pub slot_credit: Option<SlotCredit<'a>>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ClearanceError(pub &'static str);

impl fmt::Display for ClearanceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.0)
    }
}

impl std::error::Error for ClearanceError {}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Distance {
    pub lower_m: f64,
    /// Есть только при найденном маршруте, допустимом в данной геометрии.
    pub upper_m: Option<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Pass,
    Fail,
    NotAnalyzed,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Open {
    Conditions,
    Limits,
    OutsideEnvelope,
    MissingGeometryError,
    SurfaceMaterialMismatch,
    UnassignedCopper,
    NoCopper(u32),
    Layout(NotAnalyzed),
    CopperAtEdge,
    SurfaceDisconnected,
    SurfaceWitnessBlocked,
    SlotQualification,
    DistanceUncertainty,
}

impl fmt::Display for Open {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Conditions => {
                f.write_str("нужны подтверждённые материал, среда и режим напряжения")
            }
            Self::Limits => f.write_str("нет применимого предела расстояния"),
            Self::OutsideEnvelope => f.write_str("условия установки выходят за область правила"),
            Self::MissingGeometryError => f.write_str("нет границы погрешности расстояния"),
            Self::SurfaceMaterialMismatch => {
                f.write_str("материал поверхности расходится со стеком")
            }
            Self::UnassignedCopper => f.write_str("на стороне есть медь без назначения цепи"),
            Self::NoCopper(net) => write!(f, "на стороне не найдена медь цепи {net}"),
            Self::Layout(reason) => reason.fmt(f),
            Self::CopperAtEdge => {
                f.write_str("медь доходит до границы материала или выходит за неё")
            }
            Self::SurfaceDisconnected => {
                f.write_str("между участками не прослеживается плоская поверхность")
            }
            Self::SurfaceWitnessBlocked => {
                f.write_str("допустимый маршрут для верхней оценки не подтверждён")
            }
            Self::SlotQualification => f.write_str(
                "для зачёта прорези нужен применимый геометрический предел или источник",
            ),
            Self::DistanceUncertainty => {
                f.write_str("неопределённость расстояния захватывает предел")
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Check {
    pub distance: Option<Distance>,
    pub minimum_m: Option<f64>,
    pub status: Status,
    pub open: Vec<Open>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ClearanceReport {
    pub nets: [u32; 2],
    pub face: Face,
    pub scope: Option<Scope>,
    pub clearance: Check,
    pub creepage: Check,
    /// По фактическому полигональному контуру, до применения правила зачёта прорезей.
    /// Сверления могут мешать получить верхнюю границу.
    pub geometric_creepage: Option<Distance>,
}

impl ClearanceReport {
    pub fn status(&self) -> Status {
        match (self.clearance.status, self.creepage.status) {
            (Status::Fail, _) | (_, Status::Fail) => Status::Fail,
            (Status::Pass, Status::Pass) => Status::Pass,
            _ => Status::NotAnalyzed,
        }
    }
}

/// Проверяет именованную пару на одной стороне. Метры, вольты и герцы.
/// Известное нарушение сохраняется при независимом незакрытом входе.
/// В вызывающем коде требуется перечислить все пары и стороны своей установки.
pub fn evaluate(
    layout: &Layout,
    face: Face,
    nets: [u32; 2],
    conditions: Option<&Conditions<'_>>,
    limits: Option<&Limits<'_>>,
) -> Result<ClearanceReport, ClearanceError> {
    validate(layout, nets, conditions, limits)?;
    let mut applicability = Vec::new();
    match (conditions, limits) {
        (None, _) => applicability.push(Open::Conditions),
        (Some(actual), Some(rule)) if !applies(actual, &rule.envelope) => {
            applicability.push(Open::OutsideEnvelope);
        }
        _ => {}
    }
    let applicable = applicability.is_empty() && limits.is_some();
    let margin = conditions
        .and_then(|c| c.distance_error_m)
        .map(|error| error + METRES_PER_NM);
    if conditions.is_some() && margin.is_none() {
        applicability.push(Open::MissingGeometryError);
    }
    let decision_margin = margin.filter(|_| applicable);
    let layer = face.layer();
    let mut common = Vec::new();
    let material = layout.stackup().and_then(|stack| {
        stack.copper_layer(layer)?.thickness_m()?;
        let dielectrics = stack.dielectrics_between("F.Cu", "B.Cu")?;
        match face {
            Face::Front => dielectrics.first(),
            Face::Back => dielectrics.last(),
        }
        .ok_or_else(|| NotAnalyzed::MissingMaterial(layer.into()))?
        .material()
    });
    let surface_matches = material
        .as_ref()
        .is_ok_and(|&material| conditions.is_some_and(|c| c.material == material));
    match material {
        Ok(material) if conditions.is_some_and(|c| c.material != material) => {
            common.push(Open::SurfaceMaterialMismatch);
        }
        Err(reason) => common.push(Open::Layout(reason)),
        _ => {}
    }
    let Copper {
        groups,
        drills,
        foreign,
    } = collect_copper(layout, layer, nets, &mut common)?;
    for (index, group) in groups.iter().enumerate() {
        if group.is_empty() {
            common.push(Open::NoCopper(nets[index]));
        }
    }
    let mut air = DistanceAccumulator::default();
    let mut surface = DistanceAccumulator::default();
    let mut credited = DistanceAccumulator::default();
    let mut air_open = common.clone();
    let mut surface_open = common;
    let substrate = match layout.polygonal_substrate() {
        Ok(polygons) => Some(polygons),
        Err(reason) => {
            surface_open.push(Open::Layout(reason));
            None
        }
    };
    let mut actual_domain = substrate
        .as_deref()
        .map(|regions| Domain::new(regions, |_| true));
    let credit = limits
        .filter(|_| applicable)
        .and_then(|rule| rule.slot_credit);
    let mut credit_domain = match (substrate.as_deref(), credit) {
        (Some(regions), Some(SlotCredit::Rectangular { minimum_width_m }))
            if regions.iter().all(|poly| convex(poly.outline().points())) =>
        {
            Some(Domain::new(regions, |points| {
                rectangle_width(points)
                    .is_some_and(|width| width - METRES_PER_NM >= minimum_width_m)
            }))
        }
        _ => None,
    };

    if let Some(domain) = &actual_domain {
        if groups.iter().flatten().any(|item| !domain.supports(item)) {
            air_open.push(Open::CopperAtEdge);
            surface_open.push(Open::CopperAtEdge);
        }
    }
    let mut disconnected = false;
    for a in &groups[0] {
        for b in &groups[1] {
            let nearest = a.core.nearest(&b.core);
            let projected = a.distance_to(b, &[nearest.a, nearest.b], &drills, None);
            air.add(projected);
            if let Some(domain) = &mut actual_domain {
                if let Some(path) = domain.shortest_path(&a.core, &b.core, nearest) {
                    let distance = a.distance_to(b, &path, &drills, Some(&foreign));
                    surface.add(distance);
                } else {
                    disconnected = true;
                    // Tras la separación del sustrato queda al menos la proyección.
                    surface.add(Distance {
                        lower_m: projected.lower_m,
                        upper_m: None,
                    });
                }
            }
            let mut rule_lower = projected.lower_m;
            if let Some(domain) = &mut credit_domain
                && let Some(path) = domain.shortest_path(&a.core, &b.core, nearest)
            {
                rule_lower = a.lower_to(b, path_length(&path));
            }
            credited.add(Distance {
                lower_m: rule_lower,
                upper_m: None,
            });
        }
    }

    if disconnected {
        surface_open.push(Open::SurfaceDisconnected);
    }
    let geometric_creepage = surface.finish();
    if geometric_creepage.is_some_and(|d| d.upper_m.is_none()) && !disconnected {
        surface_open.push(Open::SurfaceWitnessBlocked);
    }
    let creepage_distance = match (geometric_creepage, credited.finish()) {
        (Some(geometry), Some(credit_bound)) => Some(Distance {
            lower_m: if matches!(credit, Some(SlotCredit::Verified { .. })) {
                geometry.lower_m
            } else {
                credit_bound.lower_m.min(geometry.lower_m)
            },
            upper_m: geometry.upper_m,
        }),
        _ => None,
    };
    if let (Some(measured), Some(for_rule), Some(minimum), Some(margin)) = (
        geometric_creepage,
        creepage_distance,
        limits.and_then(|rule| rule.creepage_m),
        decision_margin,
    ) && measured.lower_m - margin >= minimum
        && for_rule.lower_m - margin < minimum
    {
        surface_open.push(Open::SlotQualification);
    }

    air_open.extend(applicability.iter().cloned());
    surface_open.extend(applicability);
    Ok(ClearanceReport {
        nets,
        face,
        scope: conditions.map(|c| c.scope),
        clearance: decide(
            air.finish(),
            limits.and_then(|l| l.clearance_m),
            decision_margin,
            air_open,
        ),
        creepage: decide(
            creepage_distance,
            limits.and_then(|l| l.creepage_m),
            decision_margin.filter(|_| surface_matches),
            surface_open,
        ),
        geometric_creepage,
    })
}

fn decide(
    distance: Option<Distance>,
    minimum_m: Option<f64>,
    margin: Option<f64>,
    mut open: Vec<Open>,
) -> Check {
    if minimum_m.is_none() {
        open.push(Open::Limits);
    }
    let status = match (distance, minimum_m, margin) {
        // Один шаг координат KiCad защищает решение у границы сравнения.
        (Some(d), Some(limit), Some(margin))
            if d.upper_m.is_some_and(|value| value < limit - margin) =>
        {
            Status::Fail
        }
        (Some(d), Some(limit), Some(margin)) if open.is_empty() && d.lower_m - margin >= limit => {
            Status::Pass
        }
        (Some(_), Some(_), _) if open.is_empty() => {
            open.push(Open::DistanceUncertainty);
            Status::NotAnalyzed
        }
        _ => Status::NotAnalyzed,
    };
    Check {
        distance,
        minimum_m,
        status,
        open,
    }
}

fn applies(actual: &Conditions<'_>, rule: &Conditions<'_>) -> bool {
    actual.scope == rule.scope
        && actual.material == rule.material
        && actual.environment == rule.environment
        && actual.working_v <= rule.working_v
        && actual.peak_v <= rule.peak_v
        && actual.frequency_hz <= rule.frequency_hz
        && actual.altitude_m <= rule.altitude_m
}

fn validate(
    layout: &Layout,
    nets: [u32; 2],
    conditions: Option<&Conditions<'_>>,
    limits: Option<&Limits<'_>>,
) -> Result<(), ClearanceError> {
    if nets[0] == nets[1] || nets.iter().any(|n| *n == 0 || !layout.nets.contains_key(n)) {
        return Err(ClearanceError("нужны две разные объявленные цепи"));
    }
    for c in conditions.into_iter().chain(limits.map(|l| &l.envelope)) {
        if [c.source, c.material, c.environment]
            .iter()
            .any(|s| s.trim().is_empty())
            || ![c.working_v, c.peak_v, c.frequency_hz, c.altitude_m]
                .iter()
                .all(|v| v.is_finite())
            || c.working_v < 0.0
            || c.peak_v < c.working_v
            || c.frequency_hz < 0.0
            || c.distance_error_m
                .is_some_and(|error| !error.is_finite() || error < 0.0)
        {
            return Err(ClearanceError(
                "некорректные условия изоляции или их источник",
            ));
        }
    }
    if let Some(rule) = limits {
        let mut values = [rule.clearance_m, rule.creepage_m, None];
        match rule.slot_credit {
            Some(SlotCredit::Rectangular { minimum_width_m }) => values[2] = Some(minimum_width_m),
            Some(SlotCredit::Verified { source }) if source.trim().is_empty() => {
                return Err(ClearanceError("нет источника применимости прорезей"));
            }
            _ => {}
        }
        if values
            .into_iter()
            .flatten()
            .any(|v| !v.is_finite() || v <= 0.0)
        {
            return Err(ClearanceError(
                "предел должен быть конечным и положительным",
            ));
        }
    }
    if layout.nominal_thickness_m.is_some_and(|v| !dimension(v))
        || layout
            .tracks
            .iter()
            .any(|t| !dimension(t.width_m) || !valid_path(t.path))
        || layout
            .vias
            .iter()
            .any(|v| !dimension(v.diameter_m) || !dimension(v.drill_m) || v.drill_m >= v.diameter_m)
        || layout.footprints.iter().any(|f| {
            !f.angle_rad.is_finite()
                || f.pads.iter().any(|p| {
                    p.land.as_ref().is_some_and(|l| !valid_rect(l.outline))
                        || p.hole.is_some_and(|h| !valid_rect(h))
                })
        })
    {
        return Err(ClearanceError("некорректное число в геометрии платы"));
    }
    let valid_net =
        |net: Option<u32>| net.is_none_or(|id| id != 0 && layout.nets.contains_key(&id));
    let valid_layer = |layer: &String| layout.copper_layers.contains(layer);
    if layout
        .tracks
        .iter()
        .any(|t| !valid_net(t.net) || !valid_layer(&t.layer))
        || layout
            .vias
            .iter()
            .any(|v| !valid_net(v.net) || v.layers.is_empty() || !v.layers.iter().all(valid_layer))
        || layout.footprints.iter().flat_map(|f| &f.pads).any(|p| {
            !valid_net(p.net)
                || p.land
                    .as_ref()
                    .is_some_and(|l| l.layers.is_empty() || !l.layers.iter().all(valid_layer))
        })
        || layout
            .zones
            .iter()
            .any(|z| !valid_net(z.net) || !z.layers().iter().all(valid_layer))
    {
        return Err(ClearanceError("неверная ссылка на слой или цепь меди"));
    }
    Ok(())
}

fn dimension(v: f64) -> bool {
    // Размер имеет тот же диапазон, что и положительная координата в импортёре.
    v.is_finite() && v > 0.0 && v <= f64::from(i32::MAX) * METRES_PER_NM
}

fn valid_rect(r: RoundedRect) -> bool {
    dimension(r.width_m)
        && dimension(r.height_m)
        && r.angle_rad.is_finite()
        && (0.0..=r.width_m.min(r.height_m) / 2.0).contains(&r.radius_m)
}

fn valid_path(path: Centerline) -> bool {
    match path {
        Centerline::Segment { start, end } => start != end,
        Centerline::Arc {
            radius_m,
            center_m,
            sweep_rad,
            ..
        } => {
            radius_m.is_finite()
                && radius_m > 0.0
                && center_m.iter().all(|v| v.is_finite())
                && sweep_rad.abs() > 0.0
                && sweep_rad.abs() <= TAU
                && (radius_m * sweep_rad.abs()).is_finite()
        }
    }
}

#[derive(Default)]
struct DistanceAccumulator(Option<Distance>);

impl DistanceAccumulator {
    fn add(&mut self, other: Distance) {
        self.0 = Some(match self.0 {
            Some(d) => Distance {
                lower_m: d.lower_m.min(other.lower_m),
                upper_m: match (d.upper_m, other.upper_m) {
                    (Some(a), Some(b)) => Some(a.min(b)),
                    (a, b) => a.or(b),
                },
            },
            None => other,
        });
    }

    fn finish(self) -> Option<Distance> {
        self.0
    }
}

type V = Vector2<f64>;

fn xy(p: Point) -> V {
    V::from(p.metres())
}

struct Copper {
    groups: [Vec<Conductor>; 2],
    drills: Vec<Conductor>,
    foreign: Vec<Conductor>,
}

fn collect_copper(
    layout: &Layout,
    layer: &str,
    nets: [u32; 2],
    open: &mut Vec<Open>,
) -> Result<Copper, ClearanceError> {
    // Первые два списка принадлежат паре, третий — остальной меди стороны.
    let mut groups = [Vec::new(), Vec::new(), Vec::new()];
    let mut drills = Vec::new();
    let mut unassigned = false;
    let mut index = |net: Option<u32>| {
        unassigned |= net.is_none();
        nets.iter()
            .position(|&id| Some(id) == net)
            .unwrap_or(nets.len())
    };
    // Все сверления учитываются при проверке маршрута. Видимость металлизации
    // на разных высотах выходит за пределы плоской модели.
    for track in &layout.tracks {
        if track.layer == layer {
            groups[index(track.net)].extend(Conductor::track(track.path, track.width_m / 2.0)?);
        }
    }
    for via in &layout.vias {
        if via.layers.iter().any(|s| s == layer) {
            drills.push(Conductor::disk(via.at, via.drill_m / 2.0));
            groups[index(via.net)].push(Conductor::disk(via.at, via.diameter_m / 2.0));
        }
    }
    for pad in layout.footprints.iter().flat_map(|f| &f.pads) {
        if let Some(hole) = pad.hole {
            drills.push(Conductor::rounded(hole));
        }
        if let Some(land) = &pad.land
            && land.layers.iter().any(|s| s == layer)
        {
            groups[index(pad.net)].push(Conductor::rounded(land.outline));
        }
    }
    for zone in &layout.zones {
        if !zone.layers().iter().any(|s| s == layer) {
            continue;
        }
        match zone.fill(layer) {
            Ok(regions) => groups[index(zone.net)].extend(regions.iter().map(|region| Conductor {
                core: Shape::polygon(&region.polygon, |_| true),
                radius: 0.0,
                error: 0.0,
            })),
            Err(reason) => {
                let missing = Open::Layout(reason);
                if !open.contains(&missing) {
                    open.push(missing);
                }
            }
        }
    }
    if unassigned {
        open.push(Open::UnassignedCopper);
    }
    let [a, b, foreign] = groups;
    Ok(Copper {
        groups: [a, b],
        drills,
        foreign,
    })
}

struct Conductor {
    core: Shape,
    radius: f64,
    error: f64,
}

impl Conductor {
    fn disk(center: Point, radius: f64) -> Self {
        Self {
            core: Shape::segment(xy(center), xy(center)),
            radius,
            error: 0.0,
        }
    }

    fn rounded(r: RoundedRect) -> Self {
        // KiCad RotatePoint: при +90° ось X направляется против оси Y платы.
        let (sin, cos) = (-r.angle_rad % TAU).sin_cos();
        let x = V::new(cos, sin) * (r.width_m / 2.0 - r.radius_m);
        let y = V::new(-sin, cos) * (r.height_m / 2.0 - r.radius_m);
        let c = xy(r.center);
        let core = if x == V::zeros() || y == V::zeros() {
            Shape::segment(c - x - y, c + x + y)
        } else {
            Shape::closed(vec![c - x - y, c + x - y, c + x + y, c - x + y], Vec::new())
        };
        Self {
            core,
            radius: r.radius_m,
            error: 0.0,
        }
    }

    fn track(path: Centerline, radius: f64) -> Result<Vec<Self>, ClearanceError> {
        let pieces = match path {
            Centerline::Segment { start, end } => vec![(xy(start), xy(end), 0.0)],
            Centerline::Arc {
                start,
                end,
                center_m,
                radius_m,
                sweep_rad,
                ..
            } => {
                let s = xy(start);
                let e = xy(end);
                let center = V::from(center_m);
                // Стрела хорды R(1−cos(θ/2)) ≤ Rθ²/8. Цель — шаг сетки KiCad.
                let span = radius_m * sweep_rad.powi(2) / 8.0;
                let count = (span / METRES_PER_NM).sqrt().ceil().max(1.0);
                // Угол не превышает полного оборота. При большей дискретизации,
                // чем допускает указанный импортером диапазон геометрии, вход ошибочен.
                let max_count = (f64::from(i32::MAX) * TAU.powi(2) / 8.0).sqrt().ceil();
                if !count.is_finite() || count > max_count {
                    return Err(ClearanceError(
                        "дуга выходит за диапазон дискретизации платы",
                    ));
                }
                let unit = (s - center)
                    .try_normalize(0.0)
                    .ok_or(ClearanceError("центр дуги совпадает с началом"))?;
                let tangent = V::new(-unit.y, unit.x);
                let at = |angle: f64| {
                    // При малом угле 1−cos теряет значащие цифры, sin² сохраняет.
                    s + (tangent * angle.sin() - unit * (2.0 * (angle / 2.0).sin().powi(2)))
                        * radius_m
                };
                let end_error = (at(sweep_rad) - e).norm();
                let start_error = ((s - center).norm() - radius_m).abs();
                if !end_error.is_finite()
                    || !start_error.is_finite()
                    || end_error > METRES_PER_NM
                    || start_error > METRES_PER_NM
                {
                    return Err(ClearanceError("параметры дуги не согласованы с её концами"));
                }
                let chord_error = span / count.powi(2) + end_error + start_error;
                let mut p = s;
                (1..=count as usize)
                    .map(|i| {
                        let q = if i == count as usize {
                            e
                        } else {
                            at(sweep_rad * i as f64 / count)
                        };
                        let piece = (p, q, chord_error);
                        p = q;
                        piece
                    })
                    .collect()
            }
        };
        Ok(pieces
            .into_iter()
            .map(|(a, b, error)| Self {
                core: Shape::segment(a, b),
                radius,
                error,
            })
            .collect())
    }

    fn outer_radius(&self) -> f64 {
        self.radius + self.error
    }

    fn lower_to(&self, other: &Self, length: f64) -> f64 {
        // Суммы Минковского заполненных ядер с дисками. В проекции вычитаем
        // оба радиуса. На поверхности это та же нижняя граница: от точки
        // расширения до ядра есть отрезок внутри расширенной меди.
        (length - self.outer_radius() - other.outer_radius()).max(0.0)
    }

    fn distance_to(
        &self,
        other: &Self,
        path: &[V],
        drills: &[Self],
        foreign: Option<&[Self]>,
    ) -> Distance {
        let length = path_length(path);
        let lower_m = self.lower_to(other, length);
        let mut upper_m = None;
        if self.radius >= self.error && other.radius >= other.error {
            let from = (self.radius - self.error).min(length);
            let to = (length - (other.radius - other.error)).max(from);
            let route = trim_path(path, from, to);
            let endpoints_exist = drills
                .iter()
                .all(|hole| route.iter().all(|&p| !hole.covers(p)));
            let surface_exists = foreign.is_none_or(|metal| {
                route.windows(2).all(|pair| {
                    let segment = Shape::segment(pair[0], pair[1]);
                    drills
                        .iter()
                        .all(|hole| hole.core.nearest(&segment).distance() >= hole.radius)
                        && metal.iter().all(|part| {
                            part.core.nearest(&segment).distance() > part.outer_radius()
                        })
                })
            });
            if endpoints_exist && surface_exists {
                upper_m = Some(to - from);
            }
        }
        Distance { lower_m, upper_m }
    }

    fn covers(&self, point: V) -> bool {
        self.core.nearest_point(point).distance() < self.radius
    }
}

fn path_length(path: &[V]) -> f64 {
    path.windows(2).map(|p| (p[1] - p[0]).norm()).sum()
}

fn trim_path(path: &[V], from: f64, to: f64) -> Vec<V> {
    let mut result = Vec::new();
    let mut traversed = 0.0;
    for pair in path.windows(2) {
        let delta = pair[1] - pair[0];
        let length = delta.norm();
        if length > 0.0 && traversed + length >= from && traversed <= to {
            for offset in [(from - traversed).max(0.0), (to - traversed).min(length)] {
                result.push(pair[0] + delta * (offset / length));
            }
        }
        traversed += length;
    }
    if result.is_empty() {
        result.push(path[0]);
    }
    result
}

struct Shape {
    // Контур замкнут повтором первой точки. Стержни и точки представлены одной гранью.
    rings: Vec<Vec<V>>,
    filled: bool,
}

impl Shape {
    fn closed(outline: Vec<V>, holes: Vec<Vec<V>>) -> Self {
        let rings = std::iter::once(outline)
            .chain(holes)
            .map(|mut ring| {
                ring.push(ring[0]);
                ring
            })
            .collect();
        Self {
            rings,
            filled: true,
        }
    }

    fn segment(a: V, b: V) -> Self {
        Self {
            rings: vec![vec![a, b]],
            filled: false,
        }
    }

    fn polygon(p: &Polygon, keep_hole: impl Fn(&[Point]) -> bool) -> Self {
        let outline = p.outline().points().iter().copied().map(xy).collect();
        let holes = p
            .holes()
            .iter()
            .map(|h| h.points())
            .filter(|points| keep_hole(points))
            .map(|points| points.iter().copied().map(xy).collect())
            .collect();
        Self::closed(outline, holes)
    }

    fn edges(&self) -> impl Iterator<Item = (V, V)> + '_ {
        self.rings
            .iter()
            .flat_map(|ring| ring.windows(2).map(|pair| (pair[0], pair[1])))
    }

    fn contains(&self, p: V) -> bool {
        self.filled && inside(&self.rings[0], p) && !self.rings[1..].iter().any(|h| inside(h, p))
    }

    fn nearest_point(&self, p: V) -> Leg {
        if self.contains(p) {
            return Leg { a: p, b: p };
        }
        self.edges()
            .map(|(a, b)| Leg {
                a: p,
                b: project(p, a, b),
            })
            .min_by(|a, b| a.distance().total_cmp(&b.distance()))
            .unwrap()
    }

    fn nearest(&self, other: &Self) -> Leg {
        let a = self.rings[0][0];
        let b = other.rings[0][0];
        if self.contains(b) {
            return Leg { a: b, b };
        }
        if other.contains(a) {
            return Leg { a, b: a };
        }
        self.legs(other)
            .min_by(|a, b| a.distance().total_cmp(&b.distance()))
            .unwrap()
    }

    fn legs<'a>(&'a self, other: &'a Self) -> impl Iterator<Item = Leg> + 'a {
        self.edges().flat_map(|(a, b)| {
            other
                .edges()
                .flat_map(move |(c, d)| edge_legs(a, b, c, d).into_iter().flatten())
        })
    }
}

#[derive(Clone, Copy)]
struct Leg {
    a: V,
    b: V,
}

impl Leg {
    fn distance(self) -> f64 {
        (self.b - self.a).norm()
    }
}

fn cross(a: V, b: V) -> f64 {
    a.x * b.y - a.y * b.x
}

fn project(p: V, a: V, b: V) -> V {
    let axis = b - a;
    let square = axis.norm_squared();
    if square == 0.0 {
        a
    } else {
        a + axis * ((p - a).dot(&axis) / square).clamp(0.0, 1.0)
    }
}

fn intersection(a: V, b: V, c: V, d: V) -> Option<f64> {
    let ab = b - a;
    let cd = d - c;
    let determinant = cross(ab, cd);
    if determinant == 0.0 {
        return None;
    }
    let t = cross(c - a, cd) / determinant;
    let s = cross(c - a, ab) / determinant;
    ((0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&s)).then_some(t)
}

fn edge_legs(a: V, b: V, c: V, d: V) -> [Option<Leg>; 5] {
    [
        intersection(a, b, c, d).map(|t| {
            let p = a + (b - a) * t;
            Leg { a: p, b: p }
        }),
        Some(Leg {
            a,
            b: project(a, c, d),
        }),
        Some(Leg {
            a: b,
            b: project(b, c, d),
        }),
        Some(Leg {
            a: project(c, a, b),
            b: c,
        }),
        Some(Leg {
            a: project(d, a, b),
            b: d,
        }),
    ]
}

fn inside(ring: &[V], p: V) -> bool {
    // Луч вдоль X, с полуоткрытыми рёбрами для однократного учёта вершины.
    ring.windows(2).fold(false, |odd, edge| {
        let [a, b] = [edge[0], edge[1]];
        let crosses =
            (a.y > p.y) != (b.y > p.y) && p.x < a.x + (b.x - a.x) * ((p.y - a.y) / (b.y - a.y));
        odd != crosses
    })
}

struct Domain {
    regions: Vec<Shape>,
    vertices: Vec<V>,
    edges: Vec<(V, V)>,
    visible_vertices: Vec<Vec<(usize, f64)>>,
}

impl Domain {
    fn new(regions: &[Polygon], keep_hole: impl Fn(&[Point]) -> bool) -> Self {
        let regions: Vec<_> = regions
            .iter()
            .map(|p| Shape::polygon(p, &keep_hole))
            .collect();
        let vertices: Vec<_> = regions
            .iter()
            .flat_map(|region| {
                region
                    .rings
                    .iter()
                    .flat_map(|ring| ring[..ring.len() - 1].iter().copied())
            })
            .collect();
        let edges = regions.iter().flat_map(Shape::edges).collect();
        // Граф видимости строится лениво, когда прямой путь закрыт.
        Self {
            regions,
            edges,
            vertices,
            visible_vertices: Vec::new(),
        }
    }

    fn contains(&self, p: V) -> bool {
        self.regions.iter().any(|region| region.contains(p))
            || self.edges.iter().any(|&(a, b)| {
                // Ошибка сравнения пропорциональна координатам, а не ширине прорези.
                (p - project(p, a, b)).norm() <= f64::EPSILON * (p.norm() + a.norm() + b.norm())
            })
    }

    fn supports(&self, conductor: &Conductor) -> bool {
        conductor
            .core
            .rings
            .iter()
            .flatten()
            .all(|&p| self.contains(p))
            && self.edges.iter().all(|&(a, b)| {
                // Касание монтажа с торцом требует пространственной модели.
                conductor.core.nearest(&Shape::segment(a, b)).distance() > conductor.outer_radius()
            })
    }

    fn visible(&self, a: V, b: V) -> bool {
        if !self.contains(a) || !self.contains(b) {
            return false;
        }
        let ab = b - a;
        let square = ab.norm_squared();
        if square == 0.0 {
            return true;
        }
        let mut knots = vec![0.0, 1.0];
        for &(c, d) in &self.edges {
            knots.extend(intersection(a, b, c, d));
            for p in [c, d] {
                let offset = p - a;
                // Коллинеарность нужна и при ходе вдоль границы с выемкой.
                if cross(ab, offset).abs() <= f64::EPSILON * ab.norm() * (p.norm() + a.norm()) {
                    knots.push((offset.dot(&ab) / square).clamp(0.0, 1.0));
                }
            }
        }
        knots.sort_by(f64::total_cmp);
        knots
            .windows(2)
            .all(|ab_t| ab_t[0] == ab_t[1] || self.contains(a + ab * ((ab_t[0] + ab_t[1]) / 2.0)))
    }

    fn connection(&self, point: V, shape: &Shape) -> Option<Leg> {
        if shape.contains(point) {
            return Some(Leg { a: point, b: point });
        }
        shape
            .edges()
            .map(|(a, b)| Leg {
                a: point,
                b: project(point, a, b),
            })
            .filter(|leg| self.visible(leg.a, leg.b))
            .min_by(|a, b| a.distance().total_cmp(&b.distance()))
    }

    fn shortest_path(&mut self, a: &Shape, b: &Shape, nearest: Leg) -> Option<Vec<V>> {
        if self.visible(nearest.a, nearest.b) {
            return Some(vec![nearest.a, nearest.b]);
        }
        // В полигональной области кратчайшая ломаная поворачивает только в вершинах границы.
        // Если ортогональный выход к меди заслонён, граница видимости проходит через
        // такую вершину. Этот выход уже представлен путём через неё.
        if self.visible_vertices.is_empty() {
            self.visible_vertices = vec![Vec::new(); self.vertices.len()];
            for (i, &p) in self.vertices.iter().enumerate() {
                for (j, &q) in self.vertices[..i].iter().enumerate() {
                    if self.visible(p, q) {
                        let length = (q - p).norm();
                        self.visible_vertices[i].push((j, length));
                        self.visible_vertices[j].push((i, length));
                    }
                }
            }
        }
        let direct = a
            .legs(b)
            .filter(|leg| self.visible(leg.a, leg.b))
            .min_by(|a, b| a.distance().total_cmp(&b.distance()));
        let mut best = direct.map(|leg| leg.distance()).unwrap_or(f64::INFINITY);
        let mut path = direct.map(|leg| vec![leg.a, leg.b]);
        let mut distances = Vec::new();
        let mut starts = Vec::new();
        let mut ends = Vec::new();
        for &p in &self.vertices {
            let from = self.connection(p, a);
            distances.push(from.map_or(f64::INFINITY, Leg::distance));
            starts.push(from.map(|leg| leg.b));
            ends.push(self.connection(p, b));
        }
        let mut previous = vec![None; distances.len()];
        let mut visited = vec![false; distances.len()];
        while let Some(i) = (0..distances.len())
            .filter(|&i| !visited[i] && distances[i] < best)
            .min_by(|&i, &j| distances[i].total_cmp(&distances[j]))
        {
            visited[i] = true;
            if let Some(leg) = ends[i]
                && distances[i] + leg.distance() < best
            {
                best = distances[i] + leg.distance();
                let mut route = vec![leg.b, self.vertices[i]];
                let mut node = i;
                while let Some(j) = previous[node] {
                    route.push(self.vertices[j]);
                    node = j;
                }
                route.push(starts[node]?);
                route.reverse();
                path = Some(route);
            }
            for &(j, length) in &self.visible_vertices[i] {
                let new_distance = distances[i] + length;
                if new_distance < distances[j] {
                    distances[j] = new_distance;
                    previous[j] = Some(i);
                }
            }
        }
        path
    }
}

fn direction(a: Point, b: Point) -> [i128; 2] {
    [
        i128::from(b.x_nm) - i128::from(a.x_nm),
        i128::from(b.y_nm) - i128::from(a.y_nm),
    ]
}

fn convex(points: &[Point]) -> bool {
    let mut sign = 0;
    for i in 0..points.len() {
        let a = direction(points[i], points[(i + 1) % points.len()]);
        let b = direction(
            points[(i + 1) % points.len()],
            points[(i + 2) % points.len()],
        );
        let turn = (a[0] * b[1] - a[1] * b[0]).signum();
        if turn != 0 {
            if sign != 0 && sign != turn {
                return false;
            }
            sign = turn;
        }
    }
    true
}

fn rectangle_width(points: &[Point]) -> Option<f64> {
    let &[a, b, c, d] = points else {
        return None;
    };
    let u = direction(a, b);
    let v = direction(b, c);
    (direction(d, c) == u && direction(a, d) == v && u[0] * v[0] + u[1] * v[1] == 0).then(|| {
        (u[0] as f64)
            .hypot(u[1] as f64)
            .min((v[0] as f64).hypot(v[1] as f64))
            * METRES_PER_NM
    })
}
