//! Геометрия из разобранной платы KiCad 8.0/9.0.
//! [Board File Format](https://dev-docs.kicad.org/en/file-formats/sexpr-pcb/)
//! (Layers, Stack Up Settings, Tracks) и
//! [Board Common Syntax](https://dev-docs.kicad.org/en/file-formats/sexpr-intro/#board-common-syntax)
//! (Coordinates and Sizes, Position Identifier, Footprint, Zone).
//!
//! Оси платы совпадают с KiCad: X вправо, Y вниз. Точки остаются на сетке
//! файла и переводятся в СИ через `Point::metres`. Размеры расчётной геометрии
//! даны в метрах, углы в радианах. Заливка и стек описывают запись в файле.
//! Для доказательства физической сборки нужны их актуальность и данные изготовителя.

use super::kicad::{self, Drill, Edge, KiCadBoard, PadKind, PadShape, Path, Position, ViaKind};
use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::TAU;
use std::fmt;

pub use super::kicad::Point;

/// Причина, по которой из геометрии платы нельзя получить запрошенные данные.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NotAnalyzed {
    MissingStackup,
    MissingStackLayer(String),
    MissingThickness(String),
    MissingMaterial(String),
    MissingPermittivity(String),
    MissingLossTangent(String),
    MissingFill(String),
    MissingEdgeCuts,
    CurvedEdgeTopology,
}

impl fmt::Display for NotAnalyzed {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingStackup => f.write_str("нет явно записанного стека"),
            Self::MissingStackLayer(layer) => write!(f, "в стеке нет слоя {layer}"),
            Self::MissingThickness(layer) => write!(f, "нет толщины слоя {layer}"),
            Self::MissingMaterial(layer) => write!(f, "нет материала слоя {layer}"),
            Self::MissingPermittivity(layer) => write!(f, "нет εr слоя {layer}"),
            Self::MissingLossTangent(layer) => write!(f, "нет тангенса потерь слоя {layer}"),
            Self::MissingFill(layer) => write!(f, "нет сохранённой заливки слоя {layer}"),
            Self::MissingEdgeCuts => f.write_str("нет геометрии Edge.Cuts"),
            Self::CurvedEdgeTopology => {
                f.write_str("вложенность криволинейных границ Edge.Cuts не проверена")
            }
        }
    }
}

impl std::error::Error for NotAnalyzed {}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LayoutError {
    pub item: String,
    pub message: &'static str,
}

impl fmt::Display for LayoutError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.item, self.message)
    }
}

impl std::error::Error for LayoutError {}

type Geometry<T> = Result<T, &'static str>;

fn checked<T>(item: impl Into<String>, result: Geometry<T>) -> Result<T, LayoutError> {
    result.map_err(|message| LayoutError {
        item: item.into(),
        message,
    })
}

#[derive(Clone, Debug, PartialEq)]
pub struct Layout {
    pub format_version: u32,
    pub generator_version: Option<String>,
    pub copper_layers: Vec<String>,
    /// Общий номинал из `general`. Для восстановления стека не используется.
    pub nominal_thickness_m: Option<f64>,
    pub nets: BTreeMap<u32, String>,
    pub tracks: Vec<Track>,
    pub vias: Vec<Via>,
    pub footprints: Vec<Footprint>,
    pub zones: Vec<Zone>,
    stackup: Option<Stackup>,
    edge_cuts: Vec<EdgeContour>,
}

impl Layout {
    /// Нормализует поддерживаемую геометрию из импортёра 4.1.
    /// Противоречия в слоях, размерах и контурах возвращает с адресом объекта.
    pub fn from_kicad(board: &KiCadBoard) -> Result<Self, LayoutError> {
        let layers = checked("layers", CopperLayers::new(&board.layers))?;
        checked(
            "version",
            require(
                kicad::SUPPORTED_VERSIONS.contains(&board.version),
                "неподдерживаемая версия формата",
            ),
        )?;
        let nominal_thickness_m = checked(
            "general/thickness",
            board.thickness_nm.map(positive_metres).transpose(),
        )?;
        let stackup = checked(
            "setup/stackup",
            board
                .stackup
                .as_deref()
                .map(|stack| Stackup::new(stack, &layers))
                .transpose(),
        )?;
        let net = |id| match id {
            0 => Ok(None),
            _ if board.nets.get(&id).is_some_and(|name| !name.is_empty()) => Ok(Some(id)),
            _ => Err("ссылка на необъявленную цепь"),
        };
        let mut tracks = Vec::new();
        for (i, track) in board.tracks.iter().enumerate() {
            tracks.push(checked(
                format!("дорожка {}", i + 1),
                (|| {
                    Ok(Track {
                        path: centerline(&track.path)?,
                        width_m: positive_metres(track.width_nm)?,
                        layer: layers.name(&track.layer)?.into(),
                        net: net(track.net)?,
                    })
                })(),
            )?);
        }
        let mut vias = Vec::new();
        for (i, via) in board.vias.iter().enumerate() {
            vias.push(checked(
                format!("переход {}", i + 1),
                (|| {
                    let first = layers.index(&via.layers[0])?;
                    let last = layers.index(&via.layers[1])?;
                    require(first < last, "перевёрнутая или пустая пара слоёв")?;
                    let full = first == 0 && last == layers.names.len() - 1;
                    require(
                        match via.kind {
                            ViaKind::Through => full,
                            ViaKind::BlindBuried => !full,
                            ViaKind::Micro => last == first + 1,
                        },
                        "пара слоёв не соответствует типу перехода",
                    )?;
                    let diameter_m = positive_metres(via.diameter_nm)?;
                    let drill_m = positive_metres(via.drill_nm)?;
                    require(drill_m < diameter_m, "у перехода нет кольца меди")?;
                    Ok(Via {
                        kind: via.kind,
                        at: via.at,
                        diameter_m,
                        drill_m,
                        layers: layers.names[first..=last].to_vec(),
                        net: net(via.net)?,
                    })
                })(),
            )?);
        }
        let mut edges = CutPaths::default();
        checked("Edge.Cuts", edges.extend(&board.edges, None))?;
        let mut footprints = Vec::new();
        for (i, fp) in board.footprints.iter().enumerate() {
            let label = format!(
                "посадочное место {} ({})",
                i + 1,
                fp.reference.as_deref().unwrap_or(&fp.library_id)
            );
            let transform = checked(&label, Placement::new(fp.at))?;
            checked(
                &label,
                require(
                    matches!(fp.layer.as_str(), "F.Cu" | "B.Cu"),
                    "посадочное место должно быть на наружном слое",
                ),
            )?;
            checked(
                format!("{label}/Edge.Cuts"),
                edges.extend(&fp.edges, Some(transform)),
            )?;
            let mut pads = Vec::new();
            for (j, pad) in fp.pads.iter().enumerate() {
                pads.push(checked(
                    format!("{label}, площадка {} ({})", j + 1, pad.number),
                    (|| place_pad(pad, transform, &layers, net(pad.net)?))(),
                )?);
            }
            footprints.push(Footprint {
                library_id: fp.library_id.clone(),
                reference: fp.reference.clone(),
                layer: fp.layer.clone(),
                origin: fp.at.point,
                angle_rad: fp.at.angle_rad,
                pads,
            });
        }
        let mut zones = Vec::new();
        for (i, zone) in board.zones.iter().enumerate() {
            zones.push(checked(
                format!("зона {}", i + 1),
                normalize_zone(zone, &layers, net(zone.net)),
            )?);
        }
        Ok(Self {
            format_version: board.version,
            generator_version: board.generator_version.clone(),
            copper_layers: layers.names,
            nominal_thickness_m,
            nets: board
                .nets
                .iter()
                .filter(|&(&id, _)| id != 0)
                .map(|(&id, name)| (id, name.clone()))
                .collect(),
            tracks,
            vias,
            footprints,
            zones,
            stackup,
            edge_cuts: checked("Edge.Cuts", edges.finish())?,
        })
    }

    pub fn stackup(&self) -> Result<&Stackup, NotAnalyzed> {
        self.stackup.as_ref().ok_or(NotAnalyzed::MissingStackup)
    }

    /// Контакты, которые сеть дорожек и переходов не содержит в своих узлах.
    pub(super) fn has_unmodeled_contacts(&self, net: u32) -> bool {
        let tracks: Vec<_> = self.tracks.iter().filter(|t| t.net == Some(net)).collect();
        let vias: Vec<_> = self.vias.iter().filter(|v| v.net == Some(net)).collect();
        let pads: Vec<_> = self
            .footprints
            .iter()
            .flat_map(|fp| &fp.pads)
            .filter(|pad| pad.net == Some(net))
            .filter_map(|pad| Some((pad.anchor, pad.land.as_ref()?)))
            .collect();
        for (i, t) in tracks.iter().enumerate() {
            let (a, b) = t.path.ends();
            let (chord, arc_error) = t.path.chord_bound();
            let near = |other: Segment, radius: f64| {
                chord.separation_m(other)
                    <= t.width_m / 2.0 + arc_error + radius + kicad::METRES_PER_NM
            };
            for u in &tracks[i + 1..] {
                if t.layer != u.layer {
                    continue;
                }
                let (c, d) = u.path.ends();
                let (other, other_error) = u.path.chord_bound();
                let joint = t.path.has_network_node(c)
                    || t.path.has_network_node(d)
                    || u.path.has_network_node(a)
                    || u.path.has_network_node(b);
                // Совпавший узел прямых уже есть в графе. Для дуги
                // он не исключает второго пересечения вне этого узла.
                if (joint && (t.path.is_arc() || u.path.is_arc()))
                    || (!joint && near(other, u.width_m / 2.0 + other_error))
                {
                    return true;
                }
            }
            if vias.iter().any(|v| {
                v.layers.contains(&t.layer)
                    && !t.path.has_network_node(v.at)
                    && near(Segment::new(v.at, v.at), v.diameter_m / 2.0)
            }) || pads.iter().any(|(anchor, pad)| {
                let land = pad.outline;
                pad.layers.contains(&t.layer)
                    && !t.path.has_network_node(*anchor)
                    && near(
                        Segment::new(land.center, land.center),
                        land.width_m.hypot(land.height_m) / 2.0,
                    )
            }) {
                return true;
            }
        }
        vias.iter().enumerate().any(|(i, v)| {
            vias[i + 1..].iter().any(|u| {
                v.layers.iter().any(|layer| u.layers.contains(layer))
                    && v.at != u.at
                    && distance_m(v.at, u.at)
                        <= (v.diameter_m + u.diameter_m) / 2.0 + kicad::METRES_PER_NM
            })
        })
    }

    /// Замкнутые фрезеруемые границы, в том числе внутренние.
    /// Для криволинейных контуров здесь проверяется стыковка концов. Вложенность
    /// и пересечения кривых следует проверить при анализе поверхности платы.
    pub fn edge_cuts(&self) -> Result<&[EdgeContour], NotAnalyzed> {
        if self.edge_cuts.is_empty() {
            return Err(NotAnalyzed::MissingEdgeCuts);
        }
        Ok(&self.edge_cuts)
    }

    /// Поверхность до сверления для прямолинейных Edge.Cuts. Чётная глубина
    /// вложенности означает материал, нечётная — вырез.
    pub fn polygonal_substrate(&self) -> Result<Vec<Polygon>, NotAnalyzed> {
        let rings = self
            .edge_cuts()?
            .iter()
            .map(|edge| match edge {
                EdgeContour::Polygon(ring) => Ok(ring),
                _ => Err(NotAnalyzed::CurvedEdgeTopology),
            })
            .collect::<Result<Vec<_>, _>>()?;

        // Прямолинейные границы уже проверены на пересечения. Родитель —
        // наименьший объемлющий контур, независимо от направления обхода.
        let parents: Vec<_> = rings
            .iter()
            .enumerate()
            .map(|(i, ring)| {
                rings
                    .iter()
                    .enumerate()
                    .filter(|&(j, other)| {
                        j != i && other.locate(ring.points[0]) == Location::Inside
                    })
                    .min_by_key(|(_, other)| other.area2_nm2.abs())
                    .map(|(j, _)| j)
            })
            .collect();
        let mut regions = Vec::new();
        for (i, outline) in rings.iter().enumerate() {
            let depth = std::iter::successors(parents[i], |&j| parents[j]).count();
            if depth % 2 == 0 {
                let holes = rings
                    .iter()
                    .zip(&parents)
                    .filter(|&(_, parent)| *parent == Some(i))
                    .map(|(&hole, _)| hole.clone())
                    .collect();
                regions.push(Polygon {
                    outline: (*outline).clone(),
                    holes,
                });
            }
        }
        Ok(regions)
    }
}

fn require(condition: bool, message: &'static str) -> Geometry<()> {
    if condition { Ok(()) } else { Err(message) }
}

fn metres(nm: i32) -> f64 {
    f64::from(nm) * kicad::METRES_PER_NM
}

fn positive_metres(nm: i32) -> Geometry<f64> {
    require(nm > 0, "размер должен быть положительным")?;
    Ok(metres(nm))
}

struct CopperLayers {
    names: Vec<String>,
    declared: BTreeSet<String>,
}

fn copper_name(name: &str) -> bool {
    matches!(name, "F.Cu" | "B.Cu") || name.starts_with("In") && name.ends_with(".Cu")
}

impl CopperLayers {
    fn new(layers: &[kicad::Layer]) -> Geometry<Self> {
        let declared: BTreeSet<_> = layers.iter().map(|layer| layer.name.clone()).collect();
        let ordinals: BTreeSet<_> = layers.iter().map(|layer| layer.ordinal).collect();
        require(
            declared.len() == layers.len() && ordinals.len() == layers.len(),
            "повтор слоя или его номера",
        )?;
        let names: Vec<_> = layers
            .iter()
            .filter(|layer| copper_name(&layer.name))
            .map(|layer| layer.name.clone())
            .collect();
        require(
            matches!(names.first(), Some(front) if front == "F.Cu")
                && matches!(names.last(), Some(back) if back == "B.Cu"),
            "слои меди должны идти от F.Cu к B.Cu",
        )?;
        for (i, name) in names[1..names.len() - 1].iter().enumerate() {
            require(
                *name == format!("In{}.Cu", i + 1),
                "нарушен порядок внутренних слоёв меди",
            )?;
        }
        Ok(Self { names, declared })
    }

    fn index(&self, name: &str) -> Geometry<usize> {
        self.names
            .iter()
            .position(|layer| layer == name)
            .ok_or("необъявленный слой меди")
    }

    fn name(&self, name: &str) -> Geometry<&str> {
        Ok(&self.names[self.index(name)?])
    }

    fn pad_layers(&self, masks: &[String]) -> Geometry<Vec<String>> {
        let mut selected = BTreeSet::new();
        require(!masks.is_empty(), "пустой набор слоёв площадки")?;
        for name in masks {
            match name.as_str() {
                "*.Cu" => selected.extend(self.names.iter().map(String::as_str)),
                "F&B.Cu" => selected.extend(["F.Cu", "B.Cu"]),
                "*.Mask" | "*.Paste" | "*.SilkS" | "*.Adhes" | "*.Fab" | "*.CrtYd" => {}
                _ => {
                    require(self.declared.contains(name), "необъявленный слой площадки")?;
                    if copper_name(name) {
                        selected.insert(name);
                    }
                }
            }
        }
        Ok(self
            .names
            .iter()
            .filter(|name| selected.contains(name.as_str()))
            .cloned()
            .collect())
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Stackup {
    /// Явно записанные слои, включая технологические, в порядке файла.
    layers: Vec<StackLayer>,
    copper: Vec<(String, Option<usize>)>,
    dielectric: Vec<Option<usize>>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StackLayer {
    pub name: String,
    pub kind: String,
    thickness_m: Option<f64>,
    material: Option<String>,
    epsilon_r: Option<f64>,
    loss_tangent: Option<f64>,
}

impl StackLayer {
    pub fn thickness_m(&self) -> Result<f64, NotAnalyzed> {
        self.thickness_m
            .ok_or_else(|| NotAnalyzed::MissingThickness(self.name.clone()))
    }

    pub fn material(&self) -> Result<&str, NotAnalyzed> {
        self.material
            .as_deref()
            .ok_or_else(|| NotAnalyzed::MissingMaterial(self.name.clone()))
    }

    /// Номинал из файла. Зависимость от частоты и разброс остаются за пределами этих данных.
    pub fn epsilon_r(&self) -> Result<f64, NotAnalyzed> {
        self.epsilon_r
            .ok_or_else(|| NotAnalyzed::MissingPermittivity(self.name.clone()))
    }

    pub fn loss_tangent(&self) -> Result<f64, NotAnalyzed> {
        self.loss_tangent
            .ok_or_else(|| NotAnalyzed::MissingLossTangent(self.name.clone()))
    }
}

impl Stackup {
    fn new(input: &[kicad::StackLayer], layers: &CopperLayers) -> Geometry<Self> {
        require(!input.is_empty(), "пустой стек")?;
        let mut stack = Self {
            layers: Vec::new(),
            copper: layers
                .names
                .iter()
                .cloned()
                .map(|name| (name, None))
                .collect(),
            dielectric: vec![None; layers.names.len() - 1],
        };
        let mut last_rank = None;
        // Порядок KiCad: перед лицевой медью идут маркировка, паста, маска.
        // На обратной стороне технологические слои идут в обратном порядке.
        let front = [
            ("F.SilkS", ["Top Silk Screen", "silkscreen"]),
            ("F.Paste", ["Top Solder Paste", "solderpaste"]),
            ("F.Mask", ["Top Solder Mask", "soldermask"]),
        ];
        let back = [
            ("B.Mask", ["Bottom Solder Mask", "soldermask"]),
            ("B.Paste", ["Bottom Solder Paste", "solderpaste"]),
            ("B.SilkS", ["Bottom Silk Screen", "silkscreen"]),
        ];
        let back_start = front.len() + 2 * (layers.names.len() - 1) + 1;
        let mut names = BTreeSet::new();
        for raw in input {
            require(names.insert(raw.name.as_str()), "повтор слоя в стеке")?;
            let index = stack.layers.len();
            // Диэлектрик N лежит перед медью с индексом N.
            let rank = if let Ok(i) = layers.index(&raw.name) {
                require(raw.kind == "copper", "тип слоя меди в стеке не copper")?;
                stack.copper[i].1 = Some(index);
                front.len() + 2 * i
            } else if let Some(number) = raw.name.strip_prefix("dielectric ") {
                let n = number
                    .parse::<usize>()
                    .ok()
                    .filter(|n| (1..layers.names.len()).contains(n))
                    .ok_or("номер диэлектрика вне набора слоёв")?;
                require(
                    raw.name == format!("dielectric {n}"),
                    "неоднозначное имя диэлектрика",
                )?;
                require(
                    matches!(raw.kind.as_str(), "core" | "prepreg"),
                    "неизвестный тип диэлектрика",
                )?;
                stack.dielectric[n - 1] = Some(index);
                front.len() + 2 * n - 1
            } else {
                require(
                    layers.declared.contains(&raw.name),
                    "необъявленный технологический слой стека",
                )?;
                let matches_slot = |&(name, kinds): &(&str, [&str; 2])| {
                    raw.name == name && kinds.contains(&raw.kind.as_str())
                };
                front
                    .iter()
                    .position(matches_slot)
                    .or_else(|| back.iter().position(matches_slot).map(|i| back_start + i))
                    .ok_or("неизвестный технологический слой стека или его тип")?
            };
            require(
                last_rank.is_none_or(|previous| previous < rank),
                "слои стека записаны не в физическом порядке",
            )?;
            last_rank = Some(rank);
            require(
                raw.epsilon_r.is_none_or(|r| r.is_finite() && r > 0.0)
                    && raw.loss_tangent.is_none_or(|r| r.is_finite() && r >= 0.0),
                "некорректный параметр материала стека",
            )?;
            stack.layers.push(StackLayer {
                name: raw.name.clone(),
                kind: raw.kind.clone(),
                thickness_m: raw.thickness_nm.map(positive_metres).transpose()?,
                // KiCad 9.0, stackup_predefined_prms: «Not specified» обозначает пропуск.
                material: raw
                    .material
                    .as_deref()
                    .map(str::trim)
                    .filter(|value| {
                        !value.is_empty() && !value.eq_ignore_ascii_case("Not specified")
                    })
                    .map(str::to_owned),
                epsilon_r: raw.epsilon_r,
                loss_tangent: raw.loss_tangent,
            });
        }
        Ok(stack)
    }

    pub fn layers(&self) -> &[StackLayer] {
        &self.layers
    }

    fn copper_index(&self, name: &str) -> Result<usize, NotAnalyzed> {
        self.copper
            .iter()
            .position(|(layer, _)| layer == name)
            .ok_or_else(|| NotAnalyzed::MissingStackLayer(name.into()))
    }

    pub fn copper_layer(&self, name: &str) -> Result<&StackLayer, NotAnalyzed> {
        self.copper[self.copper_index(name)?]
            .1
            .map(|i| &self.layers[i])
            .ok_or_else(|| NotAnalyzed::MissingStackLayer(name.into()))
    }

    /// Диэлектрики между указанными слоями от лицевой стороны к обратной.
    pub fn dielectrics_between(&self, a: &str, b: &str) -> Result<Vec<&StackLayer>, NotAnalyzed> {
        let a = self.copper_index(a)?;
        let b = self.copper_index(b)?;
        (a.min(b)..a.max(b))
            .map(|i| {
                self.dielectric[i]
                    .map(|j| &self.layers[j])
                    .ok_or_else(|| NotAnalyzed::MissingStackLayer(format!("dielectric {}", i + 1)))
            })
            .collect()
    }

    /// Номинальное расстояние между срединными плоскостями меди. Включает
    /// полутолщины крайних и полные толщины внутренних слоёв данной пары.
    pub fn copper_center_distance_m(&self, a: &str, b: &str) -> Result<f64, NotAnalyzed> {
        let ia = self.copper_index(a)?;
        let ib = self.copper_index(b)?;
        if ia == ib {
            return Ok(0.0); // Совпадающие плоскости.
        }
        let mut distance =
            (self.copper_layer(a)?.thickness_m()? + self.copper_layer(b)?.thickness_m()?) / 2.0;
        for (name, _) in &self.copper[ia.min(ib) + 1..ia.max(ib)] {
            distance += self.copper_layer(name)?.thickness_m()?;
        }
        for dielectric in self.dielectrics_between(a, b)? {
            distance += dielectric.thickness_m()?;
        }
        Ok(distance)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Track {
    pub path: Centerline,
    pub width_m: f64,
    pub layer: String,
    /// Цепь 0 в файле означает отсутствие назначения у отдельного объекта.
    pub net: Option<u32>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Centerline {
    Segment {
        start: Point,
        end: Point,
    },
    Arc {
        start: Point,
        mid: Point,
        end: Point,
        center_m: [f64; 2],
        radius_m: f64,
        /// Положительный обход направлен по часовой стрелке в осях платы.
        sweep_rad: f64,
    },
}

impl Centerline {
    pub fn length_m(&self) -> f64 {
        match *self {
            Self::Segment { start, end } => distance_m(start, end),
            Self::Arc {
                radius_m,
                sweep_rad,
                ..
            } => radius_m * sweep_rad.abs(),
        }
    }

    fn ends(&self) -> (Point, Point) {
        match *self {
            Self::Segment { start, end } | Self::Arc { start, end, .. } => (start, end),
        }
    }

    fn has_network_node(self, at: Point) -> bool {
        match self {
            Self::Segment { start, end } => Segment::new(start, end).contains(at),
            Self::Arc { start, end, .. } => at == start || at == end,
        }
    }

    fn is_arc(self) -> bool {
        matches!(self, Self::Arc { .. })
    }

    fn chord_bound(self) -> (Segment, f64) {
        let (a, b) = self.ends();
        let error = match self {
            Self::Segment { .. } => 0.0,
            // Стрела R(1−cos(θ/2)) ограничивает расстояние до хорды.
            Self::Arc {
                radius_m,
                sweep_rad,
                ..
            } => 2.0 * radius_m * (sweep_rad / 4.0).sin().powi(2),
        };
        (Segment::new(a, b), error)
    }

    fn reversed(self) -> Self {
        match self {
            Self::Segment { start, end } => Self::Segment {
                start: end,
                end: start,
            },
            Self::Arc {
                start,
                mid,
                end,
                center_m,
                radius_m,
                sweep_rad,
            } => Self::Arc {
                start: end,
                mid,
                end: start,
                center_m,
                radius_m,
                sweep_rad: -sweep_rad,
            },
        }
    }
}

fn delta(a: Point, b: Point) -> [i128; 2] {
    [
        i128::from(b.x_nm) - i128::from(a.x_nm),
        i128::from(b.y_nm) - i128::from(a.y_nm),
    ]
}

fn det(u: [i128; 2], v: [i128; 2]) -> i128 {
    u[0] * v[1] - u[1] * v[0]
}

fn dot(u: [i128; 2], v: [i128; 2]) -> i128 {
    u[0] * v[0] + u[1] * v[1]
}

fn distance_m(a: Point, b: Point) -> f64 {
    let [x, y] = delta(a, b);
    (x as f64).hypot(y as f64) * kicad::METRES_PER_NM
}

fn centerline(path: &Path) -> Geometry<Centerline> {
    match *path {
        Path::Segment { start, end } => {
            require(start != end, "отрезок нулевой длины")?;
            Ok(Centerline::Segment { start, end })
        }
        Path::Arc { start, mid, end } => {
            let u = delta(start, mid);
            let v = delta(start, end);
            let cross = det(u, v);
            require(cross != 0, "вырожденная дуга")?;
            // Решение двух уравнений серединных перпендикуляров относительно start.
            let denominator = (2 * cross) as f64;
            let uu = dot(u, u);
            let vv = dot(v, v);
            let dx = (uu * v[1] - vv * u[1]) as f64 / denominator;
            let dy = (u[0] * vv - v[0] * uu) as f64 / denominator;
            let scale = kicad::METRES_PER_NM;
            let origin = start.metres();
            // Вписанный угол при mid стягивает дополнительную дугу. atan2 с
            // инверсией скалярного произведения не теряет малый угол при вычитании из π.
            let cosine = dot(delta(mid, start), delta(mid, end));
            let sweep_rad =
                2.0 * (cross.abs() as f64).atan2(-cosine as f64) * cross.signum() as f64;
            Ok(Centerline::Arc {
                start,
                mid,
                end,
                center_m: [origin[0] + dx * scale, origin[1] + dy * scale],
                radius_m: dx.hypot(dy) * scale,
                sweep_rad,
            })
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Via {
    pub kind: ViaKind,
    pub at: Point,
    /// Внешний диаметр кольца на каждом слое прохода.
    pub diameter_m: f64,
    pub drill_m: f64,
    /// Слои прохода включительно. Первый и последний элементы задают пару.
    /// Толщина металлизации этим набором не задаётся.
    pub layers: Vec<String>,
    pub net: Option<u32>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Footprint {
    pub library_id: String,
    pub reference: Option<String>,
    pub layer: String,
    pub origin: Point,
    pub angle_rad: f64,
    pub pads: Vec<Pad>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Pad {
    pub number: String,
    pub kind: PadKind,
    /// Центр сверления либо якорь площадки.
    pub anchor: Point,
    /// Область меди задана внешней фигурой за вычетом отверстия.
    /// Для неметаллизированного отверстия кольца не соединяются по вертикали.
    pub land: Option<Land>,
    pub hole: Option<RoundedRect>,
    pub net: Option<u32>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Land {
    pub outline: RoundedRect,
    pub layers: Vec<String>,
}

/// Ориентированный прямоугольник со скруглениями. Радиус 0 даёт прямоугольник,
/// радиус в половину меньшей стороны даёт овал, при равных сторонах круг.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RoundedRect {
    pub center: Point,
    pub angle_rad: f64,
    pub width_m: f64,
    pub height_m: f64,
    pub radius_m: f64,
}

#[derive(Clone, Copy)]
struct Placement {
    origin: Point,
    sin: f64,
    cos: f64,
}

impl Placement {
    fn new(at: Position) -> Geometry<Self> {
        require(at.angle_rad.is_finite(), "угол должен быть конечным")?;
        let (sin, cos) = (at.angle_rad % TAU).sin_cos();
        Ok(Self {
            origin: at.point,
            sin,
            cos,
        })
    }

    fn point(self, local: Point) -> Geometry<Point> {
        // KiCad 8/9, RotatePoint: +90° преобразует (x, y) в (y, −x).
        // Локальный вектор округляется до внутренней единицы KiCad (1 нм)
        // перед переносом. Отражение обратной стороны уже записано в файле.
        let x = f64::from(local.x_nm);
        let y = f64::from(local.y_nm);
        let absolute = |delta: f64, origin: i32| {
            let nm = delta.round() as i64 + i64::from(origin);
            i32::try_from(nm).map_err(|_| "координата вышла за диапазон сетки KiCad")
        };
        Ok(Point {
            x_nm: absolute(x * self.cos + y * self.sin, self.origin.x_nm)?,
            y_nm: absolute(y * self.cos - x * self.sin, self.origin.y_nm)?,
        })
    }
}

fn place_pad(
    raw: &kicad::Pad,
    footprint: Placement,
    layers: &CopperLayers,
    net: Option<u32>,
) -> Geometry<Pad> {
    let anchor = footprint.point(raw.at.point)?;
    let angle_rad = raw.at.angle_rad; // KiCad 8/9, parsePAD: угол уже в осях платы.
    let pad = Placement::new(Position {
        point: anchor,
        angle_rad,
    })?;
    let width_m = positive_metres(raw.width_nm)?;
    let height_m = positive_metres(raw.height_nm)?;
    let minor = width_m.min(height_m);
    let radius_m = match raw.shape {
        PadShape::Circle => {
            require(
                raw.width_nm == raw.height_nm,
                "размеры круглой площадки различаются",
            )?;
            minor / 2.0
        }
        PadShape::Rect => 0.0,
        PadShape::Oval => minor / 2.0,
        PadShape::RoundRect { radius_ratio } => {
            require(
                (0.0..=0.5).contains(&radius_ratio),
                "некорректная доля скругления",
            )?;
            minor * radius_ratio
        }
    };
    let needs_hole = matches!(raw.kind, PadKind::PlatedHole | PadKind::NonPlatedHole);
    require(
        raw.drill.is_some() == needs_hole && (needs_hole || raw.copper_offset.is_none()),
        "сверление или смещение не соответствует типу площадки",
    )?;
    let hole = raw
        .drill
        .map(|drill| -> Geometry<_> {
            let (w, h) = match drill {
                Drill::Round { diameter_nm } => (diameter_nm, diameter_nm),
                Drill::Oval {
                    width_nm,
                    height_nm,
                } => (width_nm, height_nm),
            };
            let (width_m, height_m) = (positive_metres(w)?, positive_metres(h)?);
            Ok(RoundedRect {
                center: anchor,
                angle_rad,
                width_m,
                height_m,
                radius_m: width_m.min(height_m) / 2.0,
            })
        })
        .transpose()?;
    let layers = layers.pad_layers(&raw.layers)?;
    require(
        needs_hole
            || (layers.len() <= 1
                && layers
                    .iter()
                    .all(|name| matches!(name.as_str(), "F.Cu" | "B.Cu"))),
        "поверхностная площадка должна иметь медь только на одном наружном слое",
    )?;
    require(
        raw.kind != PadKind::PlatedHole || !layers.is_empty(),
        "металлизированная площадка без слоя меди",
    )?;
    require(
        net.is_none() || (!layers.is_empty() && raw.kind != PadKind::NonPlatedHole),
        "назначена цепь площадке без электрического вывода",
    )?;
    let center = raw
        .copper_offset
        .map(|p| pad.point(p))
        .transpose()?
        .unwrap_or(anchor);
    let land = (!layers.is_empty()).then_some(Land {
        outline: RoundedRect {
            center,
            angle_rad,
            width_m,
            height_m,
            radius_m,
        },
        layers,
    });
    Ok(Pad {
        number: raw.number.clone(),
        kind: raw.kind,
        anchor,
        land,
        hole,
        net,
    })
}

/// Положение точки относительно поверхности либо полигона меди.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Location {
    Outside,
    Boundary,
    Inside,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Ring {
    points: Vec<Point>,
    area2_nm2: i128,
}

impl Ring {
    fn new(mut points: Vec<Point>) -> Geometry<Self> {
        if points.first() == points.last() {
            points.pop(); // Последнее ребро всегда замыкается на первую вершину.
        }
        require(
            points.len() >= 3,
            "контур должен иметь не менее трёх вершин",
        )?;
        let edges: Vec<_> = ring_edges(&points)
            .map(|(a, b)| Segment::new(a, b))
            .collect();
        let mut area2_nm2 = 0;
        for (i, edge) in edges.iter().enumerate() {
            let next = &edges[(i + 1) % edges.len()];
            require(edge.a != edge.b, "нулевое ребро контура")?;
            require(
                !edge.contains(next.b) && !next.contains(edge.a),
                "соседние рёбра контура перекрываются",
            )?;
            area2_nm2 += det(delta(points[0], edge.a), delta(points[0], edge.b));
        }
        require(area2_nm2 != 0, "нулевая площадь контура")?;

        // Отсев по ограничивающим прямоугольникам. Точный тест на целых нм
        // выполняется только для рёбер, чьи прямоугольники пересекаются.
        let mut order: Vec<_> = (0..edges.len()).collect();
        order.sort_by_key(|&i| edges[i].min_x);
        for (position, &i) in order.iter().enumerate() {
            let a = edges[i];
            for &j in &order[position + 1..] {
                let b = edges[j];
                if b.min_x > a.max_x {
                    break;
                }
                let adjacent = i.abs_diff(j) == 1 || i.abs_diff(j) == edges.len() - 1;
                require(adjacent || !a.intersects(b), "самопересечение контура")?;
            }
        }
        Ok(Self { points, area2_nm2 })
    }

    pub fn points(&self) -> &[Point] {
        &self.points
    }

    pub fn area_m2(&self) -> f64 {
        self.area2_nm2.abs() as f64 * kicad::METRES_PER_NM.powi(2) / 2.0
    }

    pub fn locate(&self, point: Point) -> Location {
        let mut inside = false;
        for (a, b) in ring_edges(&self.points) {
            let edge = Segment::new(a, b);
            if edge.contains(point) {
                return Location::Boundary;
            }
            if (a.y_nm > point.y_nm) != (b.y_nm > point.y_nm) {
                // Луч вправо. Знак сравнивается в целых числах без деления.
                let side = det(delta(a, b), delta(a, point));
                inside ^= (side > 0) == (b.y_nm > a.y_nm);
            }
        }
        if inside {
            Location::Inside
        } else {
            Location::Outside
        }
    }

    fn intersects(&self, other: &Self) -> bool {
        ring_edges(&self.points).any(|(a, b)| {
            let edge = Segment::new(a, b);
            ring_edges(&other.points).any(|(c, d)| edge.intersects(Segment::new(c, d)))
        })
    }
}

fn ring_edges(points: &[Point]) -> impl Iterator<Item = (Point, Point)> + '_ {
    points
        .iter()
        .copied()
        .zip(points.iter().copied().cycle().skip(1))
}

#[derive(Clone, Copy)]
struct Segment {
    a: Point,
    b: Point,
    min_x: i32,
    max_x: i32,
    min_y: i32,
    max_y: i32,
}

impl Segment {
    fn new(a: Point, b: Point) -> Self {
        Self {
            a,
            b,
            min_x: a.x_nm.min(b.x_nm),
            max_x: a.x_nm.max(b.x_nm),
            min_y: a.y_nm.min(b.y_nm),
            max_y: a.y_nm.max(b.y_nm),
        }
    }

    fn side(self, point: Point) -> i128 {
        det(delta(self.a, self.b), delta(self.a, point))
    }

    fn contains(self, point: Point) -> bool {
        (self.min_x..=self.max_x).contains(&point.x_nm)
            && (self.min_y..=self.max_y).contains(&point.y_nm)
            && self.side(point) == 0
    }

    fn intersects(self, other: Self) -> bool {
        if self.max_x < other.min_x
            || other.max_x < self.min_x
            || self.max_y < other.min_y
            || other.max_y < self.min_y
        {
            return false;
        }
        let crosses = |a: Self, b: Self| {
            let x = a.side(b.a);
            let y = a.side(b.b);
            x == 0 || y == 0 || x.signum() != y.signum()
        };
        crosses(self, other) && crosses(other, self)
    }

    fn distance_to(self, p: Point) -> f64 {
        let axis = delta(self.a, self.b);
        let projection = dot(axis, delta(self.a, p));
        if projection <= 0 {
            distance_m(self.a, p)
        } else if projection >= dot(axis, axis) {
            distance_m(self.b, p)
        } else {
            self.side(p).abs() as f64 * kicad::METRES_PER_NM.powi(2) / distance_m(self.a, self.b)
        }
    }

    fn separation_m(self, other: Self) -> f64 {
        if self.intersects(other) {
            return 0.0;
        }
        self.distance_to(other.a)
            .min(self.distance_to(other.b))
            .min(other.distance_to(self.a))
            .min(other.distance_to(self.b))
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Polygon {
    outline: Ring,
    holes: Vec<Ring>,
}

impl Polygon {
    fn new(outline: Ring, holes: Vec<Ring>) -> Geometry<Self> {
        for (i, hole) in holes.iter().enumerate() {
            require(
                outline.locate(hole.points[0]) == Location::Inside && !outline.intersects(hole),
                "вырез пересекает внешний контур или лежит снаружи",
            )?;
            for other in &holes[..i] {
                require(
                    !hole.intersects(other)
                        && hole.locate(other.points[0]) == Location::Outside
                        && other.locate(hole.points[0]) == Location::Outside,
                    "вырезы пересекаются или вложены друг в друга",
                )?;
            }
        }
        Ok(Self { outline, holes })
    }

    pub fn outline(&self) -> &Ring {
        &self.outline
    }

    pub fn holes(&self) -> &[Ring] {
        &self.holes
    }

    pub fn area_m2(&self) -> f64 {
        let holes: i128 = self.holes.iter().map(|hole| hole.area2_nm2.abs()).sum();
        (self.outline.area2_nm2.abs() - holes) as f64 * kicad::METRES_PER_NM.powi(2) / 2.0
    }

    pub fn locate(&self, point: Point) -> Location {
        let outer = self.outline.locate(point);
        if outer != Location::Inside {
            return outer;
        }
        for hole in &self.holes {
            match hole.locate(point) {
                Location::Inside => return Location::Outside,
                Location::Boundary => return Location::Boundary,
                Location::Outside => {}
            }
        }
        Location::Inside
    }
}

fn key(point: Point) -> [i32; 2] {
    [point.x_nm, point.y_nm]
}

fn point([x_nm, y_nm]: [i32; 2]) -> Point {
    Point { x_nm, y_nm }
}

fn filled_polygon(points: &[Point]) -> Geometry<Polygon> {
    // KiCad 9.0, SHAPE_POLY_SET::Unfracture: отверстия могут храниться в одном
    // обходе с внешней границей, а мосты между ними проходят в обоих направлениях.
    // https://github.com/KiCad/kicad-source-mirror/blob/9.0.0/libs/kimath/src/geometry/shape_poly_set.cpp#L1559
    let points = if points.len() > 1 && points.first() == points.last() {
        &points[..points.len() - 1]
    } else {
        points
    };
    let mut edges = BTreeSet::new();
    for (a, b) in ring_edges(points) {
        require(a != b, "нулевое ребро заливки")?;
        require(
            edges.insert((key(a), key(b))),
            "повтор ребра заливки в том же направлении",
        )?;
    }
    let mut bridge_ends = BTreeMap::new();
    let mut bridge_count = 0;
    let mut successor = BTreeMap::new();
    for &(a, b) in &edges {
        if edges.contains(&(b, a)) {
            // Учитываем один раз каждое ребро, которое прошли туда и обратно.
            if a < b {
                bridge_count += 1;
                *bridge_ends.entry(a).or_insert(0) += 1;
                *bridge_ends.entry(b).or_insert(0) += 1;
            }
        } else {
            require(
                successor.insert(a, b).is_none(),
                "разветвлённый контур заливки",
            )?;
        }
    }
    let mut rings = Vec::new();
    let mut boundary_vertices = BTreeSet::new();
    while let Some((&start, _)) = successor.first_key_value() {
        let mut current = start;
        let mut ring = Vec::new();
        loop {
            let next = successor
                .remove(&current)
                .ok_or("оборванный контур заливки")?;
            boundary_vertices.insert(current);
            ring.push(point(current));
            current = next;
            if current == start {
                break;
            }
        }
        rings.push(Ring::new(ring)?);
    }
    let intermediates = bridge_ends
        .iter()
        .filter(|(v, _)| !boundary_vertices.contains(*v));
    require(
        intermediates.clone().all(|(_, &degree)| degree >= 2),
        "мост заливки не доходит до контура",
    )?;
    require(
        !rings.is_empty() && bridge_count + 1 == rings.len() + intermediates.count(),
        "мосты заливки не образуют дерево между контурами",
    )?;
    let outer_index = rings
        .iter()
        .enumerate()
        .max_by_key(|(_, r)| r.area2_nm2.abs())
        .ok_or("нет границы заливки")?
        .0;
    let outline = rings.remove(outer_index);
    require(
        rings
            .iter()
            .all(|r| r.area2_nm2.signum() != outline.area2_nm2.signum()),
        "вырез и внешний контур заливки имеют одинаковое направление",
    )?;
    Polygon::new(outline, rings)
}

#[derive(Clone, Debug, PartialEq)]
pub struct FilledRegion {
    pub polygon: Polygon,
    /// Флаг из файла KiCad. Связность определяет отдельный анализ.
    pub island: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Zone {
    pub net: Option<u32>,
    pub design_outline: Polygon,
    layers: Vec<String>,
    filled: BTreeMap<String, Vec<FilledRegion>>,
}

impl Zone {
    pub fn layers(&self) -> &[String] {
        &self.layers
    }

    /// Сохранённая заливка выбранного слоя. Контур зоны не подменяет заливку.
    pub fn fill(&self, layer: &str) -> Result<&[FilledRegion], NotAnalyzed> {
        self.filled
            .get(layer)
            .map(Vec::as_slice)
            .ok_or_else(|| NotAnalyzed::MissingFill(layer.into()))
    }
}

fn normalize_zone(
    raw: &kicad::Zone,
    layers: &CopperLayers,
    net: Geometry<Option<u32>>,
) -> Geometry<Zone> {
    let net = net?;
    let mut ordered_layers = raw
        .layers
        .iter()
        .map(|name| layers.index(name))
        .collect::<Geometry<Vec<_>>>()?;
    require(!ordered_layers.is_empty(), "зона без слоёв меди")?;
    ordered_layers.sort_unstable();
    require(
        ordered_layers.windows(2).all(|pair| pair[0] != pair[1]),
        "повтор слоя зоны",
    )?;
    let [outline, holes @ ..] = raw.outlines.as_slice() else {
        return Err("зона без внешнего контура");
    };
    let outline = Ring::new(outline.clone())?;
    let holes = holes
        .iter()
        .cloned()
        .map(Ring::new)
        .collect::<Geometry<_>>()?;
    let design_outline = Polygon::new(outline, holes)?;
    let mut filled: BTreeMap<String, Vec<FilledRegion>> = BTreeMap::new();
    for item in &raw.filled {
        let layer = layers.index(&item.layer)?;
        require(
            ordered_layers.binary_search(&layer).is_ok(),
            "заливка на слое вне зоны",
        )?;
        filled
            .entry(item.layer.clone())
            .or_default()
            .push(FilledRegion {
                polygon: filled_polygon(&item.points)?,
                island: item.island,
            });
    }
    Ok(Zone {
        net,
        design_outline,
        layers: ordered_layers
            .into_iter()
            .map(|i| layers.names[i].clone())
            .collect(),
        filled,
    })
}

/// Геометрическая граница на Edge.Cuts. Внешние и внутренние границы
/// представлены в одинаковых осях. Порядок записи контуров роли не задаёт.
#[derive(Clone, Debug, PartialEq)]
pub enum EdgeContour {
    Polygon(Ring),
    Circle {
        center: Point,
        end: Point,
        radius_m: f64,
    },
    Path(Vec<Centerline>),
}

#[derive(Default)]
struct CutPaths {
    closed: Vec<EdgeContour>,
    paths: Vec<Centerline>,
}

impl CutPaths {
    fn extend(&mut self, edges: &[Edge], placement: Option<Placement>) -> Geometry<()> {
        let at = |p| placement.map_or(Ok(p), |transform| transform.point(p));
        for edge in edges {
            match edge {
                Edge::Path(path) => {
                    let path = match *path {
                        Path::Segment { start, end } => Path::Segment {
                            start: at(start)?,
                            end: at(end)?,
                        },
                        Path::Arc { start, mid, end } => Path::Arc {
                            start: at(start)?,
                            mid: at(mid)?,
                            end: at(end)?,
                        },
                    };
                    self.paths.push(centerline(&path)?);
                }
                Edge::Rect { start, end } => {
                    let other_a = Point {
                        x_nm: start.x_nm,
                        y_nm: end.y_nm,
                    };
                    let other_b = Point {
                        x_nm: end.x_nm,
                        y_nm: start.y_nm,
                    };
                    self.polygon([*start, other_a, *end, other_b].into_iter().map(at))?;
                }
                Edge::Circle { center, end } => {
                    let (center, end) = (at(*center)?, at(*end)?);
                    require(center != end, "окружность нулевого радиуса")?;
                    self.closed.push(EdgeContour::Circle {
                        center,
                        end,
                        radius_m: distance_m(center, end),
                    });
                }
                Edge::Polygon(points) => self.polygon(points.iter().copied().map(at))?,
            }
        }
        Ok(())
    }

    fn polygon(&mut self, points: impl Iterator<Item = Geometry<Point>>) -> Geometry<()> {
        self.closed.push(EdgeContour::Polygon(Ring::new(
            points.collect::<Geometry<_>>()?,
        )?));
        Ok(())
    }

    fn finish(mut self) -> Geometry<Vec<EdgeContour>> {
        let mut incident: BTreeMap<_, Vec<usize>> = BTreeMap::new();
        for (i, path) in self.paths.iter().enumerate() {
            let (a, b) = path.ends();
            incident.entry(key(a)).or_default().push(i);
            incident.entry(key(b)).or_default().push(i);
        }
        require(
            incident.values().all(|paths| paths.len() == 2),
            "граница платы разомкнута или разветвляется",
        )?;
        let mut used = vec![false; self.paths.len()];
        for index in 0..self.paths.len() {
            let path = self.paths[index];
            if used[index] {
                continue;
            }
            used[index] = true;
            let (start, mut end) = path.ends();
            let mut chain = vec![path];
            while start != end {
                let next = incident[&key(end)]
                    .iter()
                    .copied()
                    .find(|&i| !used[i])
                    .ok_or("граница платы не образует простой цикл")?;
                used[next] = true;
                let path = self.paths[next];
                let path = if path.ends().0 == end {
                    path
                } else {
                    path.reversed()
                };
                end = path.ends().1;
                chain.push(path);
            }
            if chain
                .iter()
                .all(|p| matches!(p, Centerline::Segment { .. }))
            {
                self.polygon(chain.iter().map(|p| Ok(p.ends().0)))?;
            } else {
                self.closed.push(EdgeContour::Path(chain));
            }
        }
        let mut polygons = Vec::<&Ring>::new();
        for contour in &self.closed {
            if let EdgeContour::Polygon(ring) = contour {
                require(
                    !polygons.iter().any(|p| ring.intersects(p)),
                    "прямолинейные границы платы пересекаются",
                )?;
                polygons.push(ring);
            }
        }
        Ok(self.closed)
    }
}
