//! Строгий импорт подмножества KiCad 8.0 и 9.0. Источники синтаксиса:
//! [Board File Format](https://dev-docs.kicad.org/en/file-formats/sexpr-pcb/#tracks-section),
//! разделы Header, Layers, Setup/Stack Up, Nets и Tracks, и
//! [Board Common Syntax](https://dev-docs.kicad.org/en/file-formats/sexpr-intro/#board-common-syntax),
//! разделы Coordinates and Sizes, Footprint, Graphic Items и Zone.
//!
//! Подмножество включает канонические слои, цепи, однослойные сегменты и дуги,
//! переходы с постоянными кольцами, посадочные места, круглые, прямоугольные,
//! овальные и скруглённые площадки, круглое и овальное сверление, медные зоны
//! и геометрию `Edge.Cuts` из отрезков, дуг, окружностей и многоугольников.
//! Составной диэлектрик, переменная геометрия колец, сложные площадки,
//! объекты на физических слоях вне этого списка и запретные области дают ошибку.
//!
//! Координаты и размеры в нанометрах, углы в радианах. Сохраняются системы
//! координат файла. Контуры зон и сохранённая заливка хранятся раздельно.
//! Пропуски представлены `None` или отсутствием сохранённого полигона на слое.
//! Зависящие от них проверки остаются `NotAnalyzed`. Подтверждение заливки,
//! применимость материалов и технологические пределы требуют отдельных источников.

use std::borrow::Cow;
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

/// Точные номера форматов на тегах [8.0.0](https://github.com/KiCad/kicad-source-mirror/blob/8.0.0/pcbnew/pcb_io/kicad_sexpr/pcb_io_kicad_sexpr.h)
/// и [9.0.0](https://github.com/KiCad/kicad-source-mirror/blob/9.0.0/pcbnew/pcb_io/kicad_sexpr/pcb_io_kicad_sexpr.h), `SEXPR_BOARD_FILE_VERSION`.
pub const SUPPORTED_VERSIONS: &[u32] = &[20240108, 20241229];

/// СИ: один нанометр равен 10⁻⁹ м; в миллиметре 10⁶ нм.
pub const METRES_PER_NM: f64 = 1e-9;
const NM_PER_MM: i64 = 1_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Point {
    pub x_nm: i32,
    pub y_nm: i32,
}

impl Point {
    pub fn metres(self) -> [f64; 2] {
        [
            f64::from(self.x_nm) * METRES_PER_NM,
            f64::from(self.y_nm) * METRES_PER_NM,
        ]
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Position {
    pub point: Point,
    pub angle_rad: f64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Layer {
    pub ordinal: u32,
    pub name: String,
    pub kind: String,
    pub user_name: Option<String>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StackLayer {
    /// Каноническое имя или `dielectric N`.
    pub name: String,
    pub kind: String,
    pub thickness_nm: Option<i32>,
    pub material: Option<String>,
    pub epsilon_r: Option<f64>,
    pub loss_tangent: Option<f64>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Path {
    Segment {
        start: Point,
        end: Point,
    },
    Arc {
        start: Point,
        mid: Point,
        end: Point,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Track {
    pub path: Path,
    pub width_nm: i32,
    pub layer: String,
    /// `0` означает отсутствие назначения цепи в KiCad.
    pub net: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ViaKind {
    Through,
    BlindBuried,
    Micro,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Via {
    pub kind: ViaKind,
    pub at: Point,
    pub diameter_nm: i32,
    pub drill_nm: i32,
    /// Начальный и конечный слои в порядке от лицевой стороны к обратной.
    pub layers: [String; 2],
    pub net: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PadKind {
    Smd,
    Connect,
    PlatedHole,
    NonPlatedHole,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PadShape {
    Circle,
    Rect,
    Oval,
    RoundRect { radius_ratio: f64 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Drill {
    Round { diameter_nm: i32 },
    Oval { width_nm: i32, height_nm: i32 },
}

#[derive(Clone, Debug, PartialEq)]
pub struct Pad {
    pub number: String,
    pub kind: PadKind,
    pub shape: PadShape,
    /// Точка относительно посадочного места. Угол площадки записан в осях платы.
    pub at: Position,
    pub width_nm: i32,
    pub height_nm: i32,
    pub drill: Option<Drill>,
    /// Смещение фигуры меди относительно центра сверления в осях площадки.
    pub copper_offset: Option<Point>,
    /// Маска слоёв файла, включая `*.Cu` и технические слои.
    pub layers: Vec<String>,
    pub net: u32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FilledPolygon {
    pub layer: String,
    pub points: Vec<Point>,
    pub island: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Zone {
    pub net: u32,
    pub layers: Vec<String>,
    /// Первый контур внешний, последующие контуры — вырезы в зоне.
    pub outlines: Vec<Vec<Point>>,
    /// Только сохранённые полигоны. Отсутствие полигона на слое не подтверждает состояние заливки.
    pub filled: Vec<FilledPolygon>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Edge {
    Path(Path),
    Rect { start: Point, end: Point },
    Circle { center: Point, end: Point },
    Polygon(Vec<Point>),
}

#[derive(Clone, Debug, PartialEq)]
pub struct Footprint {
    pub library_id: String,
    pub reference: Option<String>,
    pub layer: String,
    pub at: Position,
    pub pads: Vec<Pad>,
    /// Геометрия `Edge.Cuts` в локальных координатах посадочного места.
    pub edges: Vec<Edge>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct KiCadBoard {
    pub version: u32,
    pub generator_version: Option<String>,
    pub layers: Vec<Layer>,
    pub nets: BTreeMap<u32, String>,
    pub thickness_nm: Option<i32>,
    /// Только явно записанные данные `setup/stackup`.
    pub stackup: Option<Vec<StackLayer>>,
    pub tracks: Vec<Track>,
    pub vias: Vec<Via>,
    pub footprints: Vec<Footprint>,
    pub zones: Vec<Zone>,
    pub edges: Vec<Edge>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ErrorKind {
    Syntax,
    UnsupportedVersion(u32),
    Unsupported,
    InvalidGeometry,
    InvalidReference,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParseError {
    pub kind: ErrorKind,
    /// Строка и столбец начинаются с единицы. Столбцы считаются по символам UTF-8.
    pub line: usize,
    pub column: usize,
    pub message: String,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "KiCad, строка {}, столбец {}: {}",
            self.line, self.column, self.message
        )
    }
}

impl std::error::Error for ParseError {}

/// Разобрать целый UTF-8 файл `.kicad_pcb`. Неизвестная форма, которая может
/// изменить топологию, и противоречивые ссылки возвращают ошибку с позицией.
pub fn parse(source: &str) -> Result<KiCadBoard, ParseError> {
    let mut reader = Reader { source, offset: 0 };
    let result = (|| {
        let root = reader.node(0)?;
        reader.space();
        if reader.offset != source.len() {
            return Err(error(
                reader.offset,
                ErrorKind::Syntax,
                "данные после конца платы",
            ));
        }
        let (tag, _) = root.form()?;
        if tag != "kicad_pcb" {
            return Err(root.error(ErrorKind::Syntax, "ожидалась форма kicad_pcb"));
        }
        board(&root)
    })();
    result.map_err(|err| {
        let before = &source[..err.offset];
        ParseError {
            kind: err.kind,
            line: before.bytes().filter(|&byte| byte == b'\n').count() + 1,
            column: before.rsplit('\n').next().unwrap_or("").chars().count() + 1,
            message: err.message,
        }
    })
}

struct Issue {
    offset: usize,
    kind: ErrorKind,
    message: String,
}

type ResultAt<T> = Result<T, Issue>;

fn error(offset: usize, kind: ErrorKind, message: impl Into<String>) -> Issue {
    Issue {
        offset,
        kind,
        message: message.into(),
    }
}

enum Value<'a> {
    Atom(&'a str),
    Text(Cow<'a, str>),
    List(Vec<Node<'a>>),
}

struct Node<'a> {
    offset: usize,
    value: Value<'a>,
}

impl<'a> Node<'a> {
    fn error(&self, kind: ErrorKind, message: impl Into<String>) -> Issue {
        error(self.offset, kind, message)
    }

    fn list(&self) -> ResultAt<&[Node<'a>]> {
        match &self.value {
            Value::List(nodes) => Ok(nodes),
            _ => Err(self.error(ErrorKind::Syntax, "ожидался список")),
        }
    }

    fn atom(&self) -> ResultAt<&str> {
        match &self.value {
            Value::Atom(value) => Ok(value),
            _ => Err(self.error(ErrorKind::Syntax, "ожидался атом без кавычек")),
        }
    }

    fn text(&self) -> ResultAt<&str> {
        match &self.value {
            Value::Text(value) => Ok(value),
            _ => self.atom(),
        }
    }

    fn form(&self) -> ResultAt<(&str, &[Node<'a>])> {
        match self.list()? {
            [tag, rest @ ..] => Ok((tag.atom()?, rest)),
            [] => Err(self.error(ErrorKind::Syntax, "пустая форма")),
        }
    }

    fn args<const N: usize>(&self) -> ResultAt<&[Node<'a>; N]> {
        let (tag, args) = self.form()?;
        args.try_into().map_err(|_| {
            self.error(
                ErrorKind::Syntax,
                format!("`{tag}`: ожидалось значений: {N}"),
            )
        })
    }

    fn number(&self) -> ResultAt<f64> {
        let value = self.atom()?;
        let invalid = || {
            self.error(
                ErrorKind::Syntax,
                format!("ожидалось конечное число, получено `{value}`"),
            )
        };
        let number = value.parse::<f64>().map_err(|_| invalid())?;
        if !number.is_finite() {
            return Err(invalid());
        }
        Ok(number)
    }

    fn unsigned(&self) -> ResultAt<u32> {
        let value = self.atom()?;
        value.parse().map_err(|_| {
            self.error(
                ErrorKind::Syntax,
                format!("ожидалось целое без знака, получено `{value}`"),
            )
        })
    }
}

struct Reader<'a> {
    source: &'a str,
    offset: usize,
}

impl<'a> Reader<'a> {
    fn peek(&self) -> Option<u8> {
        self.source.as_bytes().get(self.offset).copied()
    }

    fn space(&mut self) {
        while matches!(self.peek(), Some(b' ' | b'\t' | b'\r' | b'\n')) {
            self.offset += 1;
        }
    }

    fn node(&mut self, depth: usize) -> ResultAt<Node<'a>> {
        // Ограничение глубины парсера для защиты его стека при ошибочном входе.
        const MAX_DEPTH: usize = 64;
        self.space();
        let offset = self.offset;
        let value = match self.peek() {
            Some(b'(') if depth < MAX_DEPTH => {
                self.offset += 1;
                let mut children = Vec::new();
                loop {
                    self.space();
                    match self.peek() {
                        Some(b')') => {
                            self.offset += 1;
                            break;
                        }
                        None => return Err(error(offset, ErrorKind::Syntax, "незакрытая скобка")),
                        _ => children.push(self.node(depth + 1)?),
                    }
                }
                Value::List(children)
            }
            Some(b'(') => {
                return Err(error(
                    offset,
                    ErrorKind::Syntax,
                    "превышена глубина вложенности",
                ));
            }
            Some(b'"') => Value::Text(self.quoted()?),
            Some(b')') | None => {
                return Err(error(offset, ErrorKind::Syntax, "ожидалось выражение"));
            }
            _ => {
                while let Some(byte) = self.peek() {
                    if matches!(byte, b'(' | b')' | b'"' | b' ' | b'\t' | b'\r' | b'\n') {
                        break;
                    }
                    if byte.is_ascii_control() {
                        return Err(error(self.offset, ErrorKind::Syntax, "управляющий символ"));
                    }
                    self.offset += 1;
                }
                let atom = &self.source[offset..self.offset];
                // Проверяем числовые атомы и внутри пропускаемых настроек и метаданных.
                if atom.parse::<f64>().is_ok_and(|value| !value.is_finite()) {
                    return Err(error(offset, ErrorKind::Syntax, "неконечное число"));
                }
                Value::Atom(atom)
            }
        };
        Ok(Node { offset, value })
    }

    fn quoted(&mut self) -> ResultAt<Cow<'a, str>> {
        let opening = self.offset;
        self.offset += 1;
        let mut start = self.offset;
        let mut decoded = String::new();
        while let Some(byte) = self.peek() {
            match byte {
                b'"' => {
                    let tail = &self.source[start..self.offset];
                    self.offset += 1;
                    if decoded.is_empty() {
                        return Ok(Cow::Borrowed(tail));
                    }
                    decoded.push_str(tail);
                    return Ok(Cow::Owned(decoded));
                }
                b'\\' => {
                    decoded.push_str(&self.source[start..self.offset]);
                    self.offset += 1;
                    let ch = match self.peek() {
                        Some(b'"') => '"',
                        Some(b'\\') => '\\',
                        Some(b'n') => '\n',
                        Some(b'r') => '\r',
                        Some(b't') => '\t',
                        _ => {
                            return Err(error(
                                self.offset - 1,
                                ErrorKind::Syntax,
                                "неизвестное экранирование",
                            ));
                        }
                    };
                    decoded.push(ch);
                    self.offset += 1;
                    start = self.offset;
                }
                byte if byte.is_ascii_control() => {
                    return Err(error(
                        self.offset,
                        ErrorKind::Syntax,
                        "управляющий символ в строке",
                    ));
                }
                _ => self.offset += 1,
            }
        }
        Err(error(opening, ErrorKind::Syntax, "незакрытая строка"))
    }
}

struct Fields<'n, 's> {
    owner: &'n Node<'s>,
    index: BTreeMap<&'n str, Vec<&'n Node<'s>>>,
}

impl<'n, 's> Fields<'n, 's> {
    fn new(owner: &'n Node<'s>, nodes: &'n [Node<'s>], single: &str, many: &str) -> ResultAt<Self> {
        let mut index: BTreeMap<_, Vec<_>> = BTreeMap::new();
        for node in nodes {
            let (tag, _) = node.form()?;
            let multiple = many.split_ascii_whitespace().any(|name| name == tag);
            if !multiple && !single.split_ascii_whitespace().any(|name| name == tag) {
                return Err(node.error(
                    ErrorKind::Unsupported,
                    format!("неподдерживаемая форма `{tag}`"),
                ));
            }
            let entries = index.entry(tag).or_default();
            if !multiple && !entries.is_empty() {
                return Err(node.error(ErrorKind::Syntax, format!("повтор поля `{tag}`")));
            }
            entries.push(node);
        }
        Ok(Self { owner, index })
    }

    fn all(&self, key: &str) -> impl Iterator<Item = &'n Node<'s>> + '_ {
        self.index.get(key).into_iter().flatten().copied()
    }

    fn get(&self, key: &str) -> Option<&'n Node<'s>> {
        self.all(key).next()
    }

    fn need(&self, key: &str) -> ResultAt<&'n Node<'s>> {
        self.get(key).ok_or_else(|| {
            self.owner
                .error(ErrorKind::Syntax, format!("нет обязательного поля `{key}`"))
        })
    }
}

fn length(node: &Node<'_>) -> ResultAt<i32> {
    // KiCad хранит координаты как знаковые целые в нанометрах. Десятичную запись
    // разбираем точно, чтобы проверка совпадения и коллинеарности не зависела от f64.
    let raw = node.atom()?;
    let invalid = || {
        node.error(
            ErrorKind::InvalidGeometry,
            format!("`{raw}` вне сетки координат KiCad (мм с шагом 1 нм)"),
        )
    };
    let magnitude = raw.strip_prefix(['-', '+']).unwrap_or(raw);
    let (whole, fraction) = magnitude.split_once('.').unwrap_or((magnitude, ""));
    if (whole.is_empty() && fraction.is_empty())
        || !whole
            .bytes()
            .chain(fraction.bytes())
            .all(|byte| byte.is_ascii_digit())
    {
        return Err(invalid());
    }
    let fraction = fraction.trim_end_matches('0');
    let digits = NM_PER_MM.ilog10() as usize;
    if fraction.len() > digits {
        return Err(invalid());
    }
    let whole = if whole.is_empty() {
        0
    } else {
        whole.parse::<i64>().map_err(|_| invalid())?
    };
    let fraction_nm = if fraction.is_empty() {
        0
    } else {
        fraction.parse::<i64>().map_err(|_| invalid())?
            * 10_i64.pow((digits - fraction.len()) as u32)
    };
    let nm = whole
        .checked_mul(NM_PER_MM)
        .and_then(|value| value.checked_add(fraction_nm))
        .ok_or_else(invalid)?;
    let signed = if raw.starts_with('-') { -nm } else { nm };
    i32::try_from(signed).map_err(|_| invalid())
}

fn positive(node: &Node<'_>) -> ResultAt<i32> {
    let nm = length(node)?;
    if nm <= 0 {
        return Err(node.error(
            ErrorKind::InvalidGeometry,
            "размер должен быть положительным",
        ));
    }
    Ok(nm)
}

fn point(x: &Node<'_>, y: &Node<'_>) -> ResultAt<Point> {
    Ok(Point {
        x_nm: length(x)?,
        y_nm: length(y)?,
    })
}

fn xy(node: &Node<'_>) -> ResultAt<Point> {
    let [x, y] = node.args()?;
    point(x, y)
}

fn position(node: &Node<'_>) -> ResultAt<Position> {
    let (_, args) = node.form()?;
    match args {
        [x, y, angle @ ..] if angle.len() <= 1 => Ok(Position {
            point: point(x, y)?,
            // Отсутствующий угол в `at` означает отсутствие поворота в формате KiCad.
            angle_rad: angle
                .first()
                .map(Node::number)
                .transpose()?
                .unwrap_or(0.0)
                .to_radians(),
        }),
        _ => Err(node.error(ErrorKind::Syntax, "`at`: ожидались X Y [ANGLE]")),
    }
}

fn scalar<'n>(node: &'n Node<'_>) -> ResultAt<&'n str> {
    let [value] = node.args()?;
    value.text()
}

fn optional_text(fields: &Fields<'_, '_>, tag: &str) -> ResultAt<Option<String>> {
    fields
        .get(tag)
        .map(|node| scalar(node).map(str::to_owned))
        .transpose()
}

fn flag(node: &Node<'_>) -> ResultAt<bool> {
    let (_, args) = node.form()?;
    match args {
        [] => Ok(true),
        [value] if value.atom()? == "yes" => Ok(true),
        [value] if value.atom()? == "no" => Ok(false),
        _ => Err(node.error(ErrorKind::Syntax, "ожидался флаг или значение yes/no")),
    }
}

fn numeric_metadata(fields: &Fields<'_, '_>, tags: &str) -> ResultAt<()> {
    for tag in tags.split_ascii_whitespace() {
        if let Some(node) = fields.get(tag) {
            let [value] = node.args()?;
            value.number()?;
        }
    }
    Ok(())
}

fn board(root: &Node<'_>) -> ResultAt<KiCadBoard> {
    let (_, nodes) = root.form()?;
    // Версию проверяем раньше содержимого, чтобы диагностика будущего формата
    // не зависела от того, какие новые формы в нём встретились.
    let version_node = nodes
        .iter()
        .find(|node| matches!(node.form(), Ok(("version", _))))
        .ok_or_else(|| root.error(ErrorKind::Syntax, "нет обязательного поля `version`"))?;
    let [value] = version_node.args()?;
    let version = value.unsigned()?;
    if !SUPPORTED_VERSIONS.contains(&version) {
        return Err(value.error(
            ErrorKind::UnsupportedVersion(version),
            format!("версия формата {version} не поддерживается"),
        ));
    }
    let fields = Fields::new(
        root,
        nodes,
        "version generator generator_version general paper title_block layers setup embedded_fonts embedded_files",
        "property net segment arc via footprint zone gr_line gr_rect gr_circle gr_arc gr_poly gr_curve gr_text gr_text_box dimension image target table group",
    )?;
    if scalar(fields.need("generator")?)?.is_empty() {
        return Err(fields
            .need("generator")?
            .error(ErrorKind::Syntax, "пустой generator"));
    }
    let layers = layers(fields.need("layers")?)?;
    let mut nets = BTreeMap::new();
    let mut names = BTreeSet::new();
    for node in fields.all("net") {
        let [id, name] = node.args()?;
        let (id, name) = (id.unsigned()?, name.text()?);
        if (id == 0) != name.is_empty() || nets.contains_key(&id) || !names.insert(name) {
            return Err(node.error(
                ErrorKind::InvalidReference,
                "повтор или противоречивое объявление цепи",
            ));
        }
        nets.insert(id, name.to_owned());
    }
    let refs = References {
        layers: layers.iter().map(|layer| layer.name.as_str()).collect(),
        copper: layers
            .iter()
            .filter(|layer| is_copper(&layer.name))
            .map(|layer| layer.name.as_str())
            .collect(),
        nets: &nets,
    };
    let general = fields.need("general")?;
    let general = Fields::new(general, general.form()?.1, "thickness legacy_teardrops", "")?;
    if let Some(option) = general.get("legacy_teardrops") {
        flag(option)?;
    }
    let thickness_nm = general
        .get("thickness")
        .map(|node| positive(&node.args::<1>()?[0]))
        .transpose()?;
    let stackup = stackup(fields.need("setup")?, &refs)?;
    let mut tracks = Vec::new();
    let mut vias = Vec::new();
    let mut footprints = Vec::new();
    let mut zones = Vec::new();
    let mut edges = Vec::new();
    for node in nodes {
        match node.form()?.0 {
            "segment" | "arc" => tracks.push(track(node, &refs)?),
            "via" => vias.push(via(node, &refs)?),
            "footprint" => footprints.push(footprint(node, &refs)?),
            "zone" => zones.push(zone(node, &refs)?),
            "gr_line" | "gr_rect" | "gr_circle" | "gr_arc" | "gr_poly" | "gr_curve" | "gr_text"
            | "gr_text_box" | "dimension" | "image" | "target" | "table" => {
                edges.extend(graphic(node, &refs)?);
            }
            _ => {} // Остальные формы заголовка и метаданных допущены выше.
        }
    }
    Ok(KiCadBoard {
        version,
        generator_version: optional_text(&fields, "generator_version")?,
        layers,
        nets,
        thickness_nm,
        stackup,
        tracks,
        vias,
        footprints,
        zones,
        edges,
    })
}

fn is_copper(name: &str) -> bool {
    matches!(name, "F.Cu" | "B.Cu")
        || name
            .strip_prefix("In")
            .and_then(|value| value.strip_suffix(".Cu"))
            .and_then(|value| value.parse::<u32>().ok())
            .is_some_and(|ordinal| ordinal > 0)
}

fn is_decoration(name: &str) -> bool {
    matches!(
        name,
        "F.SilkS"
            | "B.SilkS"
            | "F.Fab"
            | "B.Fab"
            | "F.CrtYd"
            | "B.CrtYd"
            | "Dwgs.User"
            | "Cmts.User"
            | "Eco1.User"
            | "Eco2.User"
    ) || name
        .strip_prefix("User.")
        .is_some_and(|value| value.parse::<u32>().is_ok())
}

fn layers(node: &Node<'_>) -> ResultAt<Vec<Layer>> {
    let mut ordinals = BTreeSet::new();
    let mut names = BTreeSet::new();
    let mut layers = Vec::new();
    for entry in node.form()?.1 {
        let (ordinal, name, kind, user_name) = match entry.list()? {
            [id, name, kind, label @ ..] if label.len() <= 1 => (
                id.unsigned()?,
                name.text()?,
                kind.atom()?,
                label.first().map(Node::text).transpose()?,
            ),
            _ => {
                return Err(entry.error(
                    ErrorKind::Syntax,
                    "слой: ожидались номер, имя, тип и необязательное имя пользователя",
                ));
            }
        };
        if !ordinals.insert(ordinal) || !names.insert(name) {
            return Err(entry.error(
                ErrorKind::InvalidReference,
                "повтор номера или канонического имени слоя",
            ));
        }
        let known = is_copper(name)
            || is_decoration(name)
            || matches!(
                name,
                "F.Adhes"
                    | "B.Adhes"
                    | "F.Paste"
                    | "B.Paste"
                    | "F.Mask"
                    | "B.Mask"
                    | "Edge.Cuts"
                    | "Margin"
            );
        if !known || !matches!(kind, "jumper" | "mixed" | "power" | "signal" | "user") {
            return Err(entry.error(
                ErrorKind::Unsupported,
                format!("неподдерживаемый слой `{name}` типа `{kind}`"),
            ));
        }
        layers.push(Layer {
            ordinal,
            name: name.into(),
            kind: kind.into(),
            user_name: user_name.map(str::to_owned),
        });
    }
    let copper: Vec<_> = layers
        .iter()
        .filter(|layer| is_copper(&layer.name))
        .map(|layer| layer.name.as_str())
        .collect();
    if !matches!(copper.as_slice(), ["F.Cu", .., "B.Cu"]) {
        return Err(node.error(
            ErrorKind::InvalidGeometry,
            "список меди должен идти от F.Cu к B.Cu",
        ));
    }
    let inner = &copper[1..copper.len() - 1];
    if inner
        .iter()
        .enumerate()
        .any(|(idx, name)| *name != format!("In{}.Cu", idx + 1))
    {
        return Err(node.error(
            ErrorKind::InvalidGeometry,
            "внутренние слои меди должны идти по порядку от In1.Cu",
        ));
    }
    Ok(layers)
}

struct References<'a> {
    layers: BTreeSet<&'a str>,
    copper: Vec<&'a str>,
    nets: &'a BTreeMap<u32, String>,
}

impl References<'_> {
    fn layer(&self, node: &Node<'_>, copper_only: bool) -> ResultAt<String> {
        let name = node.text()?;
        if self.layers.contains(name) && (!copper_only || is_copper(name)) {
            Ok(name.to_owned())
        } else {
            Err(node.error(
                ErrorKind::InvalidReference,
                format!("необъявленный или неприменимый слой `{name}`"),
            ))
        }
    }

    fn net(&self, id: &Node<'_>, name: Option<&Node<'_>>) -> ResultAt<u32> {
        let net = id.unsigned()?;
        let expected = if net == 0 {
            Some("")
        } else {
            self.nets.get(&net).map(String::as_str)
        };
        let expected = expected.ok_or_else(|| {
            id.error(
                ErrorKind::InvalidReference,
                format!("необъявленный номер цепи {net}"),
            )
        })?;
        if let Some(name) = name {
            if name.text()? != expected {
                return Err(name.error(
                    ErrorKind::InvalidReference,
                    format!("имя не совпадает с объявлением цепи {net}"),
                ));
            }
        }
        Ok(net)
    }

    fn copper_index(&self, layer: &str) -> Option<usize> {
        self.copper.iter().position(|name| *name == layer)
    }
}

fn stackup(setup: &Node<'_>, refs: &References<'_>) -> ResultAt<Option<Vec<StackLayer>>> {
    // Настройки маски, пасты, печати и координат редактора допускаются, но
    // здесь не являются моделью толщины или электроизоляции.
    let fields = Fields::new(
        setup,
        setup.form()?.1,
        "stackup pad_to_mask_clearance solder_mask_min_width pad_to_paste_clearance pad_to_paste_clearance_ratio allow_soldermask_bridges_in_footprints tenting aux_axis_origin grid_origin pcbplotparams",
        "",
    )?;
    numeric_metadata(
        &fields,
        "pad_to_mask_clearance solder_mask_min_width pad_to_paste_clearance pad_to_paste_clearance_ratio",
    )?;
    for tag in ["aux_axis_origin", "grid_origin"] {
        if let Some(origin) = fields.get(tag) {
            xy(origin)?;
        }
    }
    let Some(node) = fields.get("stackup") else {
        return Ok(None);
    };
    let fields = Fields::new(
        node,
        node.form()?.1,
        "copper_finish dielectric_constraints edge_connector castellated_pads edge_plating",
        "layer",
    )?;
    for tag in ["edge_plating", "castellated_pads"] {
        if let Some(option) = fields.get(tag) {
            if flag(option)? {
                return Err(option.error(
                    ErrorKind::Unsupported,
                    format!("неподдерживаемая геометрия `{tag}`"),
                ));
            }
        }
    }
    if let Some(option) = fields.get("edge_connector") {
        if scalar(option)? != "no" {
            return Err(option.error(
                ErrorKind::Unsupported,
                "неподдерживаемая геометрия края разъёма",
            ));
        }
    }
    let mut names = BTreeSet::new();
    let mut stack = Vec::new();
    for layer in fields.all("layer") {
        let [name, attrs @ ..] = layer.form()?.1 else {
            return Err(layer.error(ErrorKind::Syntax, "слой stackup без имени"));
        };
        let name = name.text()?;
        let dielectric = name
            .strip_prefix("dielectric ")
            .and_then(|value| value.parse::<u32>().ok())
            .is_some_and(|id| id > 0);
        if !dielectric && !refs.layers.contains(name) || !names.insert(name) {
            return Err(layer.error(
                ErrorKind::InvalidReference,
                format!("необъявленный или повторный слой stackup `{name}`"),
            ));
        }
        if let Some(marker) = attrs
            .iter()
            .find(|node| matches!(node.value, Value::Atom("addsublayer")))
        {
            return Err(marker.error(
                ErrorKind::Unsupported,
                "составной диэлектрик stackup не поддерживается",
            ));
        }
        let attrs = Fields::new(
            layer,
            attrs,
            "type color thickness material epsilon_r loss_tangent",
            "",
        )?;
        let thickness_nm = attrs
            .get("thickness")
            .map(|field| {
                // `locked` означает неизменность номинала в редакторе stackup.
                match field.form()?.1 {
                    [value] => positive(value),
                    [value, locked] if locked.atom()? == "locked" => positive(value),
                    _ => Err(field.error(ErrorKind::Syntax, "ожидалась толщина [locked]")),
                }
            })
            .transpose()?;
        let epsilon_r = attrs
            .get("epsilon_r")
            .map(|field| field.args::<1>()?[0].number())
            .transpose()?;
        let loss_tangent = attrs
            .get("loss_tangent")
            .map(|field| field.args::<1>()?[0].number())
            .transpose()?;
        if epsilon_r.is_some_and(|value| value <= 0.0)
            || loss_tangent.is_some_and(|value| value < 0.0)
        {
            return Err(layer.error(
                ErrorKind::InvalidGeometry,
                "неположительная εr или отрицательный тангенс потерь",
            ));
        }
        stack.push(StackLayer {
            name: name.to_owned(),
            kind: scalar(attrs.need("type")?)?.to_owned(),
            thickness_nm,
            material: optional_text(&attrs, "material")?.filter(|value| !value.trim().is_empty()),
            epsilon_r,
            loss_tangent,
        });
    }
    if stack.is_empty() {
        return Err(node.error(ErrorKind::Syntax, "пустой stackup"));
    }
    Ok(Some(stack))
}

fn cross(a: Point, b: Point, c: Point) -> i128 {
    let dx = |point: Point| i128::from(point.x_nm) - i128::from(a.x_nm);
    let dy = |point: Point| i128::from(point.y_nm) - i128::from(a.y_nm);
    dx(b) * dy(c) - dy(b) * dx(c)
}

fn path(fields: &Fields<'_, '_>) -> ResultAt<Path> {
    let start = xy(fields.need("start")?)?;
    let end = xy(fields.need("end")?)?;
    if let Some(mid) = fields.get("mid") {
        let mid = xy(mid)?;
        if cross(start, mid, end) == 0 {
            return Err(fields.owner.error(
                ErrorKind::InvalidGeometry,
                "вырожденная дуга: точки совпали или лежат на одной прямой",
            ));
        }
        Ok(Path::Arc { start, mid, end })
    } else if start == end {
        Err(fields
            .owner
            .error(ErrorKind::InvalidGeometry, "отрезок нулевой длины"))
    } else {
        Ok(Path::Segment { start, end })
    }
}

fn track(node: &Node<'_>, refs: &References<'_>) -> ResultAt<Track> {
    let (tag, nodes) = node.form()?;
    let allowed = if tag == "arc" {
        "start mid end width layer net locked uuid tstamp"
    } else {
        "start end width layer net locked uuid tstamp"
    };
    let fields = Fields::new(node, nodes, allowed, "")?;
    if tag == "arc" {
        fields.need("mid")?;
    }
    Ok(Track {
        path: path(&fields)?,
        width_nm: positive(&fields.need("width")?.args::<1>()?[0])?,
        layer: refs.layer(&fields.need("layer")?.args::<1>()?[0], true)?,
        net: refs.net(&fields.need("net")?.args::<1>()?[0], None)?,
    })
}

fn fixed_anchors(fields: &Fields<'_, '_>) -> ResultAt<()> {
    for tag in ["remove_unused_layers", "keep_end_layers"] {
        if let Some(field) = fields.get(tag) {
            if flag(field)? {
                return Err(field.error(
                    ErrorKind::Unsupported,
                    "геометрия колец зависит от подключённых слоёв",
                ));
            }
        }
    }
    Ok(())
}

fn via(node: &Node<'_>, refs: &References<'_>) -> ResultAt<Via> {
    let (_, mut nodes) = node.form()?;
    let mut kind = ViaKind::Through;
    if let [first, rest @ ..] = nodes {
        if let Value::Atom(tag) = first.value {
            kind = match tag {
                "blind" => ViaKind::BlindBuried,
                "micro" => ViaKind::Micro,
                _ => {
                    return Err(first.error(
                        ErrorKind::Unsupported,
                        format!("неподдерживаемый тип перехода `{tag}`"),
                    ));
                }
            };
            nodes = rest;
        }
    }
    let fields = Fields::new(
        node,
        nodes,
        "at size drill layers net locked free tenting uuid tstamp remove_unused_layers keep_end_layers",
        "",
    )?;
    fixed_anchors(&fields)?;
    let [front, back] = fields.need("layers")?.args()?;
    let layers = [refs.layer(front, true)?, refs.layer(back, true)?];
    let from = refs
        .copper_index(&layers[0])
        .expect("медный слой уже проверен");
    let to = refs
        .copper_index(&layers[1])
        .expect("медный слой уже проверен");
    let full_span = from == 0 && to == refs.copper.len() - 1;
    if from >= to
        || match kind {
            ViaKind::Through => !full_span,
            ViaKind::BlindBuried => full_span,
            ViaKind::Micro => to - from != 1,
        }
    {
        return Err(front.error(
            ErrorKind::InvalidGeometry,
            "пара слоёв не соответствует типу перехода",
        ));
    }
    let diameter_nm = positive(&fields.need("size")?.args::<1>()?[0])?;
    let drill_node = &fields.need("drill")?.args::<1>()?[0];
    let drill_nm = positive(drill_node)?;
    if drill_nm >= diameter_nm {
        return Err(drill_node.error(
            ErrorKind::InvalidGeometry,
            "диаметр сверления должен быть меньше диаметра перехода",
        ));
    }
    Ok(Via {
        kind,
        at: xy(fields.need("at")?)?,
        diameter_nm,
        drill_nm,
        layers,
        net: refs.net(&fields.need("net")?.args::<1>()?[0], None)?,
    })
}

fn footprint(node: &Node<'_>, refs: &References<'_>) -> ResultAt<Footprint> {
    let (_, args) = node.form()?;
    let [library_id, nodes @ ..] = args else {
        return Err(node.error(
            ErrorKind::Syntax,
            "нет библиотечного имени посадочного места",
        ));
    };
    let fields = Fields::new(
        node,
        nodes,
        "layer at locked placed uuid tstamp descr tags path sheetname sheetfile autoplace_cost90 autoplace_cost180 attr clearance solder_mask_margin solder_paste_margin solder_paste_ratio solder_paste_margin_ratio zone_connect thermal_width thermal_bridge_width thermal_gap embedded_fonts embedded_files component_classes",
        "property model pad fp_text fp_text_box fp_line fp_rect fp_circle fp_arc fp_poly fp_curve group",
    )?;
    numeric_metadata(
        &fields,
        "autoplace_cost90 autoplace_cost180 clearance solder_mask_margin solder_paste_margin solder_paste_ratio solder_paste_margin_ratio zone_connect thermal_width thermal_bridge_width thermal_gap",
    )?;
    let layer_node = &fields.need("layer")?.args::<1>()?[0];
    let layer = refs.layer(layer_node, true)?;
    if !matches!(layer.as_str(), "F.Cu" | "B.Cu") {
        return Err(layer_node.error(
            ErrorKind::InvalidGeometry,
            "посадочное место должно быть на наружном слое",
        ));
    }
    let mut reference = None;
    let mut property_names = BTreeSet::new();
    let mut pads = Vec::new();
    let mut edges = Vec::new();
    for child in nodes {
        let (tag, args) = child.form()?;
        match tag {
            "pad" => pads.push(pad(child, refs)?),
            "property" => {
                let [key, value, attributes @ ..] = args else {
                    return Err(
                        child.error(ErrorKind::Syntax, "свойство: ожидались имя и значение")
                    );
                };
                let key = key.text()?;
                if !property_names.insert(key) {
                    return Err(child.error(ErrorKind::Syntax, format!("повтор свойства `{key}`")));
                }
                let layers = graphic_layer(child, attributes, refs)?;
                if layers.is_some_and(|layer| !is_decoration(&layer)) {
                    return Err(child.error(ErrorKind::Unsupported, "свойство на физическом слое"));
                }
                if key == "Reference" {
                    set_reference(&mut reference, value)?;
                }
            }
            "fp_text" => {
                graphic(child, refs)?;
                if let [kind, value, ..] = args {
                    if kind.atom()? == "reference" {
                        set_reference(&mut reference, value)?;
                    }
                }
            }
            "fp_line" | "fp_rect" | "fp_circle" | "fp_arc" | "fp_poly" | "fp_curve"
            | "fp_text_box" => {
                edges.extend(graphic(child, refs)?);
            }
            _ => {} // Прочие допущенные поля не задают физические фигуры платы.
        }
    }
    Ok(Footprint {
        library_id: library_id.text()?.to_owned(),
        reference,
        layer,
        at: position(fields.need("at")?)?,
        pads,
        edges,
    })
}

fn set_reference(reference: &mut Option<String>, value: &Node<'_>) -> ResultAt<()> {
    let value_text = value.text()?;
    if value_text.trim().is_empty() || reference.as_deref().is_some_and(|name| name != value_text) {
        return Err(value.error(
            ErrorKind::InvalidReference,
            "пустое или противоречивое обозначение компонента",
        ));
    }
    *reference = Some(value_text.to_owned());
    Ok(())
}

fn pad(node: &Node<'_>, refs: &References<'_>) -> ResultAt<Pad> {
    let (_, args) = node.form()?;
    let [number, kind, shape, nodes @ ..] = args else {
        return Err(node.error(ErrorKind::Syntax, "площадка: ожидались номер, тип и форма"));
    };
    let fields = Fields::new(
        node,
        nodes,
        "at size drill layers net roundrect_rratio property locked uuid tstamp remove_unused_layers keep_end_layers pinfunction pintype die_length solder_mask_margin solder_paste_margin solder_paste_margin_ratio clearance zone_connect thermal_width thermal_bridge_width thermal_gap thermal_bridge_angle tenting teardrops",
        "",
    )?;
    numeric_metadata(
        &fields,
        "die_length solder_mask_margin solder_paste_margin solder_paste_margin_ratio clearance zone_connect thermal_width thermal_bridge_width thermal_gap thermal_bridge_angle",
    )?;
    fixed_anchors(&fields)?;
    if let Some(field) = fields.get("property") {
        if scalar(field)? == "pad_prop_castellated" {
            return Err(field.error(
                ErrorKind::Unsupported,
                "металлизация края площадки не поддерживается",
            ));
        }
    }
    let kind = match kind.atom()? {
        "smd" => PadKind::Smd,
        "connect" => PadKind::Connect,
        "thru_hole" => PadKind::PlatedHole,
        "np_thru_hole" => PadKind::NonPlatedHole,
        text => {
            return Err(kind.error(
                ErrorKind::Unsupported,
                format!("неподдерживаемый тип площадки `{text}`"),
            ));
        }
    };
    let [width, height] = fields.need("size")?.args()?;
    let (width_nm, height_nm) = (positive(width)?, positive(height)?);
    let shape = match shape.atom()? {
        "circle" if width_nm == height_nm => PadShape::Circle,
        "circle" => {
            return Err(shape.error(
                ErrorKind::InvalidGeometry,
                "у круглой площадки размеры по осям должны совпадать",
            ));
        }
        "rect" => PadShape::Rect,
        "oval" => PadShape::Oval,
        "roundrect" => {
            let [ratio] = fields.need("roundrect_rratio")?.args()?;
            let radius_ratio = ratio.number()?;
            // Радиус считается от меньшей стороны; больше половины сторона не вмещает.
            if !(0.0..=0.5).contains(&radius_ratio) {
                return Err(ratio.error(
                    ErrorKind::InvalidGeometry,
                    "доля радиуса скругления вне диапазона 0–0,5",
                ));
            }
            PadShape::RoundRect { radius_ratio }
        }
        text => {
            return Err(shape.error(
                ErrorKind::Unsupported,
                format!("неподдерживаемая форма площадки `{text}`"),
            ));
        }
    };
    if let Some(field) = fields.get("roundrect_rratio") {
        if !matches!(shape, PadShape::RoundRect { .. }) {
            return Err(field.error(
                ErrorKind::InvalidGeometry,
                "скругление указано для другой формы площадки",
            ));
        }
    }
    let (drill, copper_offset) = match fields.get("drill") {
        Some(field) if matches!(kind, PadKind::PlatedHole | PadKind::NonPlatedHole) => {
            let (drill, offset) = pad_drill(field)?;
            (Some(drill), offset)
        }
        None if matches!(kind, PadKind::Smd | PadKind::Connect) => (None, None),
        _ => {
            return Err(node.error(
                ErrorKind::InvalidGeometry,
                "сверление не соответствует типу площадки",
            ));
        }
    };
    let layer_node = fields.need("layers")?;
    let mut copper_layers = BTreeSet::new();
    let mut layers = Vec::new();
    for token in layer_node.form()?.1 {
        let name = token.text()?;
        match name {
            "*.Cu" => copper_layers.extend(refs.copper.iter().copied()),
            "F&B.Cu" => copper_layers.extend(["F.Cu", "B.Cu"]),
            "*.Mask" | "*.Paste" | "*.SilkS" | "*.Adhes" | "*.Fab" | "*.CrtYd" => {}
            _ => {
                refs.layer(token, false)?;
                if is_copper(name) {
                    copper_layers.insert(name);
                }
            }
        }
        if layers.iter().any(|layer| layer == name) {
            return Err(token.error(ErrorKind::InvalidReference, "повтор слоя площадки"));
        }
        layers.push(name.to_owned());
    }
    if layers.is_empty()
        || matches!(kind, PadKind::Smd | PadKind::Connect) && copper_layers.len() > 1
        || kind == PadKind::PlatedHole && copper_layers.is_empty()
    {
        return Err(layer_node.error(
            ErrorKind::InvalidGeometry,
            "набор слоёв не соответствует типу площадки",
        ));
    }
    let net = match fields.get("net") {
        Some(field) => {
            let [id, name] = field.args()?;
            refs.net(id, Some(name))?
        }
        // KiCad не записывает `net` на неназначенной площадке.
        None => 0,
    };
    if net != 0 && (copper_layers.is_empty() || kind == PadKind::NonPlatedHole) {
        return Err(node.error(
            ErrorKind::InvalidReference,
            "цепь указана у площадки без проводящего вывода",
        ));
    }
    Ok(Pad {
        number: number.text()?.to_owned(),
        kind,
        shape,
        at: position(fields.need("at")?)?,
        width_nm,
        height_nm,
        drill,
        copper_offset,
        layers,
        net,
    })
}

fn pad_drill(node: &Node<'_>) -> ResultAt<(Drill, Option<Point>)> {
    let (_, nodes) = node.form()?;
    let (nodes, offset) = match nodes.split_last() {
        Some((last, rest)) if matches!(last.form(), Ok(("offset", _))) => (rest, Some(xy(last)?)),
        _ => (nodes, None),
    };
    let drill = match nodes {
        [diameter] => Drill::Round {
            diameter_nm: positive(diameter)?,
        },
        [oval, width] if oval.atom()? == "oval" => {
            let width_nm = positive(width)?;
            Drill::Oval {
                width_nm,
                height_nm: width_nm,
            }
        }
        [oval, width, height] if oval.atom()? == "oval" => Drill::Oval {
            width_nm: positive(width)?,
            height_nm: positive(height)?,
        },
        _ => {
            return Err(node.error(
                ErrorKind::Syntax,
                "неподдерживаемая запись сверления площадки",
            ));
        }
    };
    Ok((drill, offset))
}

fn polygon(node: &Node<'_>) -> ResultAt<Vec<Point>> {
    let (_, nodes) = node.form()?;
    let mut points = Vec::with_capacity(nodes.len());
    for vertex in nodes {
        if vertex.form()?.0 != "xy" {
            return Err(vertex.error(ErrorKind::Unsupported, "поддерживаются только вершины `xy`"));
        }
        points.push(xy(vertex)?);
    }
    if let [first, ..] = points.as_slice() {
        // KiCad хранит контуры замкнутыми неявно. Граница может содержать
        // обратные проходы, которыми в сохранённую заливку вшиты отверстия.
        let area_twice: i128 = points
            .windows(2)
            .map(|pair| cross(*first, pair[0], pair[1]))
            .sum();
        if points.len() >= 3 && area_twice != 0 {
            return Ok(points);
        }
    }
    Err(node.error(
        ErrorKind::InvalidGeometry,
        "полигон вырожден: меньше трёх вершин или нулевая площадь",
    ))
}

fn zone(node: &Node<'_>, refs: &References<'_>) -> ResultAt<Zone> {
    let fields = Fields::new(
        node,
        node.form()?.1,
        "net net_name layer layers name uuid tstamp locked hatch priority connect_pads min_thickness filled_areas_thickness fill",
        "polygon filled_polygon",
    )?;
    numeric_metadata(&fields, "priority min_thickness")?;
    let layer_tokens = match (fields.get("layer"), fields.get("layers")) {
        (Some(layer), None) => layer.args::<1>()?.as_slice(),
        (None, Some(layers)) if !layers.form()?.1.is_empty() => layers.form()?.1,
        _ => {
            return Err(node.error(
                ErrorKind::Syntax,
                "зона: нужен один непустой список `layer` или `layers`",
            ));
        }
    };
    let mut layers = Vec::new();
    for token in layer_tokens {
        let layer = refs.layer(token, true)?;
        if layers.contains(&layer) {
            return Err(token.error(ErrorKind::InvalidReference, "повтор слоя зоны"));
        }
        layers.push(layer);
    }
    let [net_id] = fields.need("net")?.args()?;
    let net_name = fields
        .get("net_name")
        .map(|name| name.args::<1>().map(|args| &args[0]))
        .transpose()?;
    let net = refs.net(net_id, net_name)?;
    let outlines: Vec<_> = fields
        .all("polygon")
        .map(|poly| {
            let fields = Fields::new(poly, poly.form()?.1, "pts", "")?;
            polygon(fields.need("pts")?)
        })
        .collect::<ResultAt<_>>()?;
    if outlines.is_empty() {
        return Err(node.error(ErrorKind::InvalidGeometry, "в зоне нет внешнего контура"));
    }
    // По формату KiCad признак yes предшествует параметрам пересчёта.
    // Сохранённые полигоны без этого признака противоречат состоянию зоны.
    let mut marked_filled = false;
    if let Some(fill) = fields.get("fill") {
        let (_, args) = fill.form()?;
        let parameters = match args {
            [first, rest @ ..] if matches!(first.value, Value::Atom("yes" | "no")) => {
                marked_filled = first.atom()? == "yes";
                rest
            }
            _ => args,
        };
        let recipe = Fields::new(
            fill,
            parameters,
            "mode thermal_gap thermal_bridge_width smoothing radius island_removal_mode island_area_min hatch_thickness hatch_gap hatch_orientation hatch_smoothing_level hatch_smoothing_value hatch_border_algorithm hatch_min_hole_area",
            "",
        )?;
        numeric_metadata(
            &recipe,
            "thermal_gap thermal_bridge_width radius island_removal_mode island_area_min hatch_thickness hatch_gap hatch_orientation hatch_smoothing_level hatch_smoothing_value hatch_min_hole_area",
        )?;
    }
    if let Some(poly) = fields.get("filled_polygon") {
        if !marked_filled {
            return Err(poly.error(
                ErrorKind::InvalidGeometry,
                "сохранённый полигон в зоне без признака заливки",
            ));
        }
    }
    let filled = fields
        .all("filled_polygon")
        .map(|poly| {
            let fields = Fields::new(poly, poly.form()?.1, "layer pts island", "")?;
            let [layer] = fields.need("layer")?.args()?;
            let name = refs.layer(layer, true)?;
            if !layers.contains(&name) {
                return Err(layer.error(
                    ErrorKind::InvalidReference,
                    "слой заливки не входит в слои зоны",
                ));
            }
            Ok(FilledPolygon {
                layer: name,
                points: polygon(fields.need("pts")?)?,
                island: fields.get("island").map(flag).transpose()?.unwrap_or(false),
            })
        })
        .collect::<ResultAt<_>>()?;
    Ok(Zone {
        net,
        layers,
        outlines,
        filled,
    })
}

fn graphic_layer(
    owner: &Node<'_>,
    nodes: &[Node<'_>],
    refs: &References<'_>,
) -> ResultAt<Option<String>> {
    let mut layer = None;
    for node in nodes {
        match node.form() {
            Ok(("layer", [name, attrs @ ..]))
                if attrs.is_empty()
                    || matches!(attrs, [item] if item.atom().is_ok_and(|value| value == "knockout")) =>
            {
                if layer.is_some() {
                    return Err(node.error(ErrorKind::Syntax, "повтор поля `layer`"));
                }
                layer = Some(refs.layer(name, false)?);
            }
            Ok(("layer", _)) => return Err(node.error(ErrorKind::Syntax, "неверное поле `layer`")),
            Ok(("layers", _)) => {
                return Err(node.error(
                    ErrorKind::Unsupported,
                    "многослойная графика не поддерживается",
                ));
            }
            _ => {}
        }
    }
    if layer.is_none() && owner.form()?.0 != "property" {
        return Err(owner.error(ErrorKind::Syntax, "графический объект без слоя"));
    }
    Ok(layer)
}

fn graphic(node: &Node<'_>, refs: &References<'_>) -> ResultAt<Option<Edge>> {
    let (tag, args) = node.form()?;
    let Some(layer) = graphic_layer(node, args, refs)? else {
        return Ok(None);
    };
    if is_decoration(&layer) {
        return Ok(None);
    }
    if layer != "Edge.Cuts" {
        return Err(node.error(
            ErrorKind::Unsupported,
            format!("графика `{tag}` на физическом слое `{layer}` не поддерживается"),
        ));
    }
    let (shape, extra) = match tag {
        "gr_line" | "fp_line" => ("line", "start end"),
        "gr_arc" | "fp_arc" => ("arc", "start mid end"),
        "gr_rect" | "fp_rect" => ("rect", "start end fill"),
        "gr_circle" | "fp_circle" => ("circle", "center end fill"),
        "gr_poly" | "fp_poly" => ("poly", "pts fill"),
        _ => {
            return Err(node.error(
                ErrorKind::Unsupported,
                format!("неподдерживаемая форма контура платы `{tag}`"),
            ));
        }
    };
    let fields = Fields::new(
        node,
        args,
        &format!("{extra} layer stroke locked uuid tstamp"),
        "",
    )?;
    fields.need("layer")?.args::<1>()?;
    if let Some(fill) = fields.get("fill") {
        if !matches!(scalar(fill)?, "no" | "none") {
            return Err(fill.error(
                ErrorKind::Unsupported,
                "заливка контура платы не поддерживается",
            ));
        }
    }
    if let Some(stroke) = fields.get("stroke") {
        let fields = Fields::new(stroke, stroke.form()?.1, "width type color", "")?;
        let [width] = fields.need("width")?.args()?;
        if length(width)? < 0 {
            return Err(width.error(
                ErrorKind::InvalidGeometry,
                "толщина линии контура отрицательная",
            ));
        }
        if let Some(kind) = fields.get("type") {
            if !matches!(scalar(kind)?, "default" | "solid") {
                return Err(kind.error(
                    ErrorKind::Unsupported,
                    "штриховой контур платы не поддерживается",
                ));
            }
        }
    }
    let edge = match shape {
        "arc" => {
            fields.need("mid")?;
            Edge::Path(path(&fields)?)
        }
        "line" => Edge::Path(path(&fields)?),
        "rect" => {
            let start = xy(fields.need("start")?)?;
            let end = xy(fields.need("end")?)?;
            if start.x_nm == end.x_nm || start.y_nm == end.y_nm {
                return Err(node.error(
                    ErrorKind::InvalidGeometry,
                    "прямоугольник контура нулевой площади",
                ));
            }
            Edge::Rect { start, end }
        }
        "circle" => {
            let center = xy(fields.need("center")?)?;
            let end = xy(fields.need("end")?)?;
            if center == end {
                return Err(node.error(
                    ErrorKind::InvalidGeometry,
                    "окружность контура нулевого радиуса",
                ));
            }
            Edge::Circle { center, end }
        }
        _ => Edge::Polygon(polygon(fields.need("pts")?)?),
    };
    Ok(Some(edge))
}
