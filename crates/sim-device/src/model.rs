//! Описание устройства. Всё задаётся простыми полями, чтобы редактироваться и из формы в окне,
//! и руками в файле проекта.

use sim_core::Vec3;

use crate::measure::Measurement;
use crate::settings::*;
use crate::wiring::Wiring;

/// Биомеханический предел участка тела при контакте. Значения ISO/TS 15066 по табл. 1 в
/// arXiv:2311.13814. Стандарт написан для роботов, пороги близки к началу боли: для ортеза
/// это ориентир, а не гарантия отсутствия травмы.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactLimit {
    pub name: &'static str,
    /// Допустимая сила при зажиме, Н. В первые 0.5 с удара допускается вдвое больше.
    pub quasi_static_n: f64,
    pub stiffness_n_per_m: f64,
    pub mass_kg: f64,
}

/// Во сколько раз сила удара (первые 0.5 с) может превышать силу зажима: arXiv:2009.01036.
pub const TRANSIENT_FACTOR: f64 = 2.0;

/// Участок тела, рядом с которым работает звено.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyRegion {
    None,
    Head,
    Face,
    Neck,
    Torso,
    UpperArm,
    Forearm,
    Hand,
    Thigh,
    Shank,
}

impl BodyRegion {
    pub const ALL: [BodyRegion; 10] = [
        BodyRegion::None,
        BodyRegion::Head,
        BodyRegion::Face,
        BodyRegion::Neck,
        BodyRegion::Torso,
        BodyRegion::UpperArm,
        BodyRegion::Forearm,
        BodyRegion::Hand,
        BodyRegion::Thigh,
        BodyRegion::Shank,
    ];

    pub fn key(self) -> &'static str {
        match self {
            BodyRegion::None => "none",
            BodyRegion::Head => "head",
            BodyRegion::Face => "face",
            BodyRegion::Neck => "neck",
            BodyRegion::Torso => "torso",
            BodyRegion::UpperArm => "upper_arm",
            BodyRegion::Forearm => "forearm",
            BodyRegion::Hand => "hand",
            BodyRegion::Thigh => "thigh",
            BodyRegion::Shank => "shank",
        }
    }

    pub fn from_key(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|r| r.key() == s)
    }

    pub fn title(self) -> &'static str {
        match self {
            BodyRegion::None => "не рядом с телом",
            BodyRegion::Head => "голова",
            BodyRegion::Face => "лицо",
            BodyRegion::Neck => "шея",
            BodyRegion::Torso => "туловище",
            BodyRegion::UpperArm => "плечо и локоть",
            BodyRegion::Forearm => "предплечье и запястье",
            BodyRegion::Hand => "кисть и пальцы",
            BodyRegion::Thigh => "бедро и колено",
            BodyRegion::Shank => "голень",
        }
    }

    /// Предел контакта. Для туловища берётся худшее из спины, груди, живота и таза:
    /// наименьшая допустимая сила (живот) и наибольшая жёсткость (спина).
    pub fn contact(self) -> Option<ContactLimit> {
        let c = |name, f, k_n_per_mm: f64, m| ContactLimit {
            name,
            quasi_static_n: f,
            stiffness_n_per_m: k_n_per_mm * 1000.0,
            mass_kg: m,
        };
        match self {
            BodyRegion::None => None,
            BodyRegion::Head => Some(c("голова", 130.0, 150.0, 4.4)),
            BodyRegion::Face => Some(c("лицо", 65.0, 75.0, 4.4)),
            BodyRegion::Neck => Some(c("шея", 150.0, 50.0, 1.2)),
            BodyRegion::Torso => Some(c("туловище", 110.0, 35.0, 40.0)),
            BodyRegion::UpperArm => Some(c("плечо и локоть", 150.0, 30.0, 3.0)),
            BodyRegion::Forearm => Some(c("предплечье и запястье", 160.0, 40.0, 2.0)),
            BodyRegion::Hand => Some(c("кисть и пальцы", 140.0, 75.0, 0.6)),
            BodyRegion::Thigh => Some(c("бедро и колено", 220.0, 50.0, 75.0)),
            BodyRegion::Shank => Some(c("голень", 130.0, 60.0, 75.0)),
        }
    }

    /// Минимальный зазор от раздавливания по ISO 13854, мм: берётся самая крупная часть
    /// тела, которая может попасть в этот зазор.
    pub fn crush_gap_mm(self) -> Option<f64> {
        match self {
            BodyRegion::None => None,
            BodyRegion::Head | BodyRegion::Face | BodyRegion::Neck => Some(300.0),
            BodyRegion::Torso => Some(500.0),
            BodyRegion::UpperArm | BodyRegion::Forearm => Some(120.0),
            BodyRegion::Hand => Some(100.0),
            BodyRegion::Thigh | BodyRegion::Shank => Some(180.0),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    pub const ALL: [Axis; 3] = [Axis::X, Axis::Y, Axis::Z];

    pub fn unit(self) -> Vec3 {
        match self {
            Axis::X => Vec3::x(),
            Axis::Y => Vec3::y(),
            Axis::Z => Vec3::z(),
        }
    }

    pub fn key(self) -> &'static str {
        match self {
            Axis::X => "x",
            Axis::Y => "y",
            Axis::Z => "z",
        }
    }

    pub fn from_key(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|a| a.key() == s)
    }

    pub fn title(self) -> &'static str {
        match self {
            Axis::X => "X (вперёд)",
            Axis::Y => "Y (вбок)",
            Axis::Z => "Z (вверх)",
        }
    }
}

/// Куда направлено звено от своего шарнира, в нулевой позе.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Back,
    Left,
    Right,
    Up,
    Down,
}

impl Direction {
    pub const ALL: [Direction; 6] = [
        Direction::Forward,
        Direction::Back,
        Direction::Left,
        Direction::Right,
        Direction::Up,
        Direction::Down,
    ];

    pub fn unit(self) -> Vec3 {
        match self {
            Direction::Forward => Vec3::x(),
            Direction::Back => -Vec3::x(),
            Direction::Left => Vec3::y(),
            Direction::Right => -Vec3::y(),
            Direction::Up => Vec3::z(),
            Direction::Down => -Vec3::z(),
        }
    }

    pub fn key(self) -> &'static str {
        match self {
            Direction::Forward => "forward",
            Direction::Back => "back",
            Direction::Left => "left",
            Direction::Right => "right",
            Direction::Up => "up",
            Direction::Down => "down",
        }
    }

    pub fn from_key(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|d| d.key() == s)
    }

    pub fn title(self) -> &'static str {
        match self {
            Direction::Forward => "вперёд",
            Direction::Back => "назад",
            Direction::Left => "влево",
            Direction::Right => "вправо",
            Direction::Up => "вверх",
            Direction::Down => "вниз",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Section {
    Rect { b_mm: f64, h_mm: f64 },
    Round { d_mm: f64 },
    Tube { od_mm: f64, id_mm: f64 },
}

impl Section {
    pub fn area_m2(self) -> f64 {
        let pi = std::f64::consts::PI;
        match self {
            Section::Rect { b_mm, h_mm } => b_mm * h_mm * 1e-6,
            Section::Round { d_mm } => pi * d_mm * d_mm / 4.0 * 1e-6,
            Section::Tube { od_mm, id_mm } => pi * (od_mm * od_mm - id_mm * id_mm) / 4.0 * 1e-6,
        }
    }

    /// Радиус описанной окружности сечения, м (для инерции и показа).
    pub fn outer_radius_m(self) -> f64 {
        match self {
            Section::Rect { b_mm, h_mm } => 0.5e-3 * b_mm.hypot(h_mm),
            Section::Round { d_mm } => 0.5e-3 * d_mm,
            Section::Tube { od_mm, .. } => 0.5e-3 * od_mm,
        }
    }

    fn is_valid(self) -> bool {
        let ok = |x: f64| x.is_finite() && x > 0.0;
        match self {
            Section::Rect { b_mm, h_mm } => ok(b_mm) && ok(h_mm),
            Section::Round { d_mm } => ok(d_mm),
            Section::Tube { od_mm, id_mm } => ok(od_mm) && ok(id_mm) && id_mm < od_mm,
        }
    }
}

/// Настройки печати для детали из пластика.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PrintSpec {
    pub layer_mm: f64,
    pub walls: u32,
    pub infill_pct: f64,
    /// Основная нагрузка идёт поперёк слоёв (самое слабое направление).
    pub across_layers: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointKind {
    Fixed,
    Hinge,
}

#[derive(Clone, Debug, PartialEq)]
pub struct JointSpec {
    pub kind: JointKind,
    pub axis: Axis,
    pub min_deg: f64,
    pub max_deg: f64,
    /// Идентификатор привода из каталога; пусто — шарнир без привода.
    pub actuator: String,
    /// Предел скорости в прошивке, °/с.
    pub speed_limit_dps: f64,
    /// Предел момента в прошивке, Н·м (0 — только предел самого привода).
    pub torque_limit_nm: f64,
    /// Наименьший зазор между этим звеном и соседней деталью или телом при схождении, мм.
    pub pinch_gap_mm: f64,
    /// Зона схождения закрыта кожухом. Это заявление конструктора, его проверяют на стенде.
    pub guarded: bool,
}

impl JointSpec {
    pub fn fixed() -> Self {
        Self {
            kind: JointKind::Fixed,
            axis: Axis::Y,
            min_deg: 0.0,
            max_deg: 0.0,
            actuator: String::new(),
            speed_limit_dps: 0.0,
            torque_limit_nm: 0.0,
            pinch_gap_mm: 0.0,
            guarded: false,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct LinkSpec {
    pub name: String,
    /// Родительское звено; `None` — крепится прямо к телу.
    pub parent: Option<usize>,
    /// Где шарнир этого звена в кадре родителя, мм.
    pub offset_mm: [f64; 3],
    pub direction: Direction,
    pub length_mm: f64,
    pub section: Section,
    /// Идентификатор материала из каталога.
    pub material: String,
    /// Настройки печати, если деталь печатная.
    pub print: Option<PrintSpec>,
    /// Диаметр отверстия под ось или болт у основания звена, мм (0 — нет).
    pub hole_mm: f64,
    /// Масса на конце звена (груз, датчик, кисть), кг.
    pub end_mass_kg: f64,
    /// Какой участок тела может оказаться на пути звена.
    pub near_body: BodyRegion,
    /// Ближайшая к шарниру точка, где звено может давить на тело, мм.
    pub contact_lever_mm: f64,
    pub joint: JointSpec,
    pub settings: LinkSettings,
}

/// Электроника устройства. Идентификаторы — из каталогов; пусто — детали нет.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct Electronics {
    pub board: String,
    pub driver: String,
    pub regulator: String,
    pub cell: String,
    pub series: u32,
    pub parallel: u32,
    /// Номинал предохранителя, А (0 — нет).
    pub fuse_a: f64,
    /// Сечение силового провода, мм².
    pub wire_mm2: f64,
    pub wire_m: f64,
    /// Есть ли плата защиты батареи (BMS).
    pub bms: bool,
    /// Путь к файлу прошивки (ELF или HEX), пусто — прошивка не задана.
    pub firmware: String,
    pub settings: ElectricalSettings,
    pub connections: Vec<Wiring>,
    pub sensors: Vec<SensorSettings>,
    pub firmware_cases: Vec<FirmwareSettings>,
}

/// Как будут пользоваться устройством.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UseProfile {
    pub ambient_c: f64,
    /// Сколько циклов нагрузки за весь срок службы.
    pub cycles: f64,
    /// Носится долго (строже предел давления и нагрева у кожи).
    pub long_wear: bool,
    /// Рабочих часов в день.
    pub hours_per_day: f64,
}

impl Default for UseProfile {
    fn default() -> Self {
        Self {
            ambient_c: 25.0,
            cycles: 100_000.0,
            long_wear: false,
            hours_per_day: 2.0,
        }
    }
}

/// Меньше минуты в день — не режим работы, а способ обнулить нагрев и разряд в расчёте.
const MIN_WORK_H: f64 = 1.0 / 60.0;

#[derive(Clone, Debug, PartialEq)]
pub struct Device {
    pub name: String,
    /// Из какого шаблона создано (для справки).
    pub template: String,
    /// Где устройство крепится к телу.
    pub mount: BodyRegion,
    pub links: Vec<LinkSpec>,
    pub electronics: Electronics,
    pub usage: UseProfile,
    pub settings: DeviceSettings,
    /// Замеры на настоящем устройстве.
    pub measurements: Vec<Measurement>,
}

impl Device {
    /// Проверка, что описание вообще имеет смысл. Пустой список — можно считать.
    pub fn validate(&self) -> Vec<String> {
        let mut e = Vec::new();
        if self.name.trim().is_empty() {
            e.push("у устройства нет названия".to_string());
        }
        if self.links.is_empty() {
            e.push("в устройстве нет ни одного звена".to_string());
        }
        e.extend(self.settings.validate_fields());
        let el = &self.electronics;
        e.extend(el.settings.validate_fields());
        for sensor in &el.sensors {
            e.extend(sensor.validate_fields());
        }
        for case in &el.firmware_cases {
            e.extend(case.validate_fields());
        }
        if ![el.fuse_a, el.wire_mm2, el.wire_m]
            .into_iter()
            .all(|x| x.is_finite() && x >= 0.0)
        {
            e.push("неверные параметры предохранителя или провода".into());
        }
        // Сечение и длина: 0 — не задано, остальное должно быть реальным проводом.
        if el.wire_mm2 > 240.0 || (el.wire_mm2 > 0.0 && el.wire_mm2 < 0.005) {
            e.push("сечение провода вне диапазона реальных проводов".into());
        }
        if el.wire_m > 100.0 || (el.wire_m > 0.0 && el.wire_m < 0.01) {
            e.push("длина провода вне реального диапазона".into());
        }
        if el.connections.iter().any(|c| !c.is_finite()) {
            e.push("неконечное число в соединении платы".into());
        }
        let fin = |x: f64| x.is_finite();
        for (i, l) in self.links.iter().enumerate() {
            let who = format!("звено {} «{}»", i + 1, l.name);
            if let Some(p) = l.parent
                && p >= i
            {
                e.push(format!("{who}: родитель должен стоять раньше в списке"));
            }
            if !(fin(l.length_mm) && l.length_mm > 0.0) {
                e.push(format!("{who}: длина должна быть больше нуля"));
            }
            if !l.section.is_valid() {
                e.push(format!("{who}: неверное сечение"));
            }
            if l.material.trim().is_empty() {
                e.push(format!("{who}: не выбран материал"));
            }
            if !(fin(l.end_mass_kg) && l.end_mass_kg >= 0.0) {
                e.push(format!("{who}: масса на конце не может быть отрицательной"));
            }
            if !(fin(l.hole_mm) && l.hole_mm >= 0.0) {
                e.push(format!("{who}: неверный диаметр отверстия"));
            }
            if !l.offset_mm.iter().all(|x| fin(*x)) {
                e.push(format!("{who}: неверное смещение шарнира"));
            }
            if let Some(p) = l.print
                && !(fin(p.layer_mm)
                    && p.layer_mm > 0.0
                    && p.walls > 0
                    && fin(p.infill_pct)
                    && (0.0..=100.0).contains(&p.infill_pct))
            {
                e.push(format!("{who}: неверные настройки печати"));
            }
            if l.near_body != BodyRegion::None
                && !(fin(l.contact_lever_mm) && l.contact_lever_mm > 0.0)
            {
                e.push(format!("{who}: укажите, где звено может давить на тело"));
            }
            if l.near_body != BodyRegion::None && l.contact_lever_mm > l.length_mm {
                e.push(format!(
                    "{who}: точка давления на тело не может быть дальше конца звена"
                ));
            }
            if l.settings.bolt_lever_mm.is_some_and(|x| x > l.length_mm) {
                e.push(format!("{who}: плечо болтовой группы длиннее самого звена"));
            }
            e.extend(
                l.settings
                    .validate_fields()
                    .into_iter()
                    .map(|error| format!("{who}: {error}")),
            );
            let j = &l.joint;
            if ![
                j.min_deg,
                j.max_deg,
                j.speed_limit_dps,
                j.torque_limit_nm,
                j.pinch_gap_mm,
                l.contact_lever_mm,
            ]
            .into_iter()
            .all(fin)
            {
                e.push(format!("{who}: неконечное значение шарнира или контакта"));
            }
            if j.kind == JointKind::Fixed && !j.actuator.is_empty() {
                e.push(format!("{who}: выбран привод без подвижного шарнира"));
            }
            if j.kind == JointKind::Hinge {
                if !(fin(j.min_deg) && fin(j.max_deg) && j.min_deg < j.max_deg) {
                    e.push(format!("{who}: неверный диапазон шарнира"));
                }
                if !j.actuator.is_empty() && !(fin(j.speed_limit_dps) && j.speed_limit_dps > 0.0) {
                    e.push(format!("{who}: у привода должен быть предел скорости"));
                }
                if !(fin(j.torque_limit_nm) && j.torque_limit_nm >= 0.0) {
                    e.push(format!("{who}: неверный предел момента"));
                }
                if !(fin(j.pinch_gap_mm) && j.pinch_gap_mm >= 0.0) {
                    e.push(format!("{who}: неверный зазор"));
                }
            }
        }
        let u = &self.usage;
        if !(fin(u.ambient_c)
            && fin(u.cycles)
            && u.cycles >= 0.0
            && fin(u.hours_per_day)
            && (0.0..=24.0).contains(&u.hours_per_day))
        {
            e.push("неверные условия использования".to_string());
        }
        if (u.cycles > 0.0 && u.cycles < 1.0) || (u.hours_per_day > 0.0 && u.hours_per_day < MIN_WORK_H)
        {
            e.push("ресурс меньше одного цикла или работа короче минуты не описывают реальное использование".into());
        }
        if !(-60.0..=150.0).contains(&u.ambient_c) {
            e.push("температура воздуха вне диапазона от −60 до 150 °C".to_string());
        }
        for (i, m) in self.measurements.iter().enumerate() {
            let who = format!("замер {} ({})", i + 1, m.quantity.title());
            let min = if m.quantity.unit() == "°C" {
                -273.15
            } else {
                0.0
            }; // Абсолютный нуль.
            if !fin(m.value) || m.value < min {
                e.push(format!(
                    "{who}: значение должно быть конечным и не ниже {min}"
                ));
            }
            match m.link {
                Some(l) if l >= self.links.len() => e.push(format!("{who}: нет такого звена")),
                None if m.quantity.per_link() => e.push(format!("{who}: укажите звено")),
                Some(_) if !m.quantity.per_link() => {
                    e.push(format!("{who}: величина относится ко всему устройству"))
                }
                _ => {}
            }
        }
        e
    }

    /// Звенья поддерева, начиная с `link` (включая его самого).
    pub fn subtree(&self, link: usize) -> Vec<usize> {
        let mut inside = vec![false; self.links.len()];
        if link >= inside.len() {
            return Vec::new();
        }
        inside[link] = true;
        for i in link + 1..self.links.len() {
            if let Some(p) = self.links[i].parent
                && p < i
                && inside[p]
            {
                inside[i] = true;
            }
        }
        (0..self.links.len()).filter(|i| inside[*i]).collect()
    }
}
