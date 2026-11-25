//! Формат 3. Длины, массы и время в СИ, углы с суффиксом `_deg`.
//! STEP требует отдельного преобразования в геометрию формата 3 и здесь не импортируется.

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::path::PathBuf;

use crate::Provenance;

/// Номер формата дисковых документов.
pub const FORMAT: u32 = 3;

#[derive(Serialize, Deserialize)]
#[serde(remote = "Provenance", rename_all = "snake_case")]
enum Source {
    Datasheet(String),
    Paper(String),
    Measured(String),
    Derived(String),
    Estimate(String),
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Pose {
    #[serde(default)]
    pub position_m: [f64; 3],
    /// Кватернион в порядке w, x, y, z.
    pub quat_wxyz: Option<[f64; 4]>,
    /// Повороты вокруг неподвижных осей X, Y, Z в этом порядке.
    pub rpy_deg: Option<[f64; 3]>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct World {
    pub format: u32,
    pub name: String,
    pub seed: u64,
    pub physics_hz: u64,
    pub gravity_m_s2: [f64; 3],
    pub environment: Environment,
    #[serde(default)]
    pub devices: Vec<Placement>,
    #[serde(default)]
    pub objects: Vec<Object>,
    #[serde(default)]
    pub attachments: Vec<Attachment>,
    /// Датчики вне устройств: неподвижная камера, контрольный дальномер.
    #[serde(default)]
    pub sensors: Vec<SensorUse>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Environment {
    /// Нулевые плотность, вязкость и скорость задают вакуум.
    pub density_kg_m3: f64,
    pub viscosity_pa_s: f64,
    pub wind_m_s: [f64; 3],
    #[serde(default)]
    pub collision: Vec<Geometry>,
    #[serde(default)]
    pub visual: Vec<Geometry>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Placement {
    pub name: String,
    pub file: PathBuf,
    #[serde(default)]
    pub pose: Pose,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Object {
    pub name: String,
    #[serde(default)]
    pub pose: Pose,
    pub mass: Mass,
    #[serde(default)]
    pub collision: Vec<Geometry>,
    #[serde(default)]
    pub visual: Vec<Geometry>,
    #[serde(default)]
    pub mounts: Vec<Mount>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Device {
    pub format: u32,
    pub name: String,
    /// Отдельный TOML электрической схемы относительно файла устройства.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub schematic: Option<PathBuf>,
    pub links: Vec<Link>,
    #[serde(default)]
    pub constraints: Vec<Constraint>,
    #[serde(default)]
    pub transmissions: Vec<Transmission>,
    /// Приводы: деталь каталога на входе усилия соединения.
    #[serde(default)]
    pub drives: Vec<DriveUse>,
    #[serde(default)]
    pub sensors: Vec<SensorUse>,
    pub legacy_v2: Option<LegacyV2>,
}

/// Электрические ссылки хранятся по именам экземпляров, вне физического контракта.
#[derive(Debug)]
pub struct WorldWithSchematics {
    pub world: crate::CompiledWorld,
    pub schematics: BTreeMap<String, PathBuf>,
}

pub use crate::loader::{
    load_and_compile_with_schematics, load_and_compile_with_schematics_and_options,
};

/// Привод из каталога на входе усилия. Его физику считает модель привода.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DriveUse {
    pub name: String,
    /// Идентификатор позиции каталога.
    pub part: String,
    /// Имя входа усилия в этом устройстве.
    pub effort: String,
    /// Параметры этого экземпляра поверх каталога, например замеренный люфт.
    #[serde(default)]
    pub params: BTreeMap<String, f64>,
}

/// Датчик из каталога. Без `link` закреплён в мире.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorUse {
    pub name: String,
    pub part: String,
    /// Звено устройства. В файле мира — `экземпляр/звено`.
    pub link: Option<String>,
    /// Поза датчика в кадре звена или мира.
    #[serde(default)]
    pub pose: Pose,
    /// Соединение, которое измеряет датчик (энкодер).
    pub joint: Option<String>,
    /// Привод, чьё внутреннее состояние измеряет датчик (ток, температура).
    pub drive: Option<String>,
    #[serde(default)]
    pub params: BTreeMap<String, f64>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LegacyV2 {
    pub original: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Link {
    pub name: String,
    pub parent: Option<String>,
    #[serde(default)]
    pub pose: Pose,
    pub joint: Joint,
    pub mass: Mass,
    #[serde(default)]
    pub collision: Vec<Geometry>,
    #[serde(default)]
    pub visual: Vec<Geometry>,
    #[serde(default)]
    pub mounts: Vec<Mount>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JointType {
    Hinge,
    Slide,
    Ball,
    Fixed,
    Free,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Joint {
    pub kind: JointType,
    pub name: Option<String>,
    pub axis: Option<[f64; 3]>,
    /// Якорь в системе звена.
    #[serde(default)]
    pub anchor_m: [f64; 3],
    pub limits_deg: Option<[f64; 2]>,
    pub limits_m: Option<[f64; 2]>,
    /// Радиус углового конуса вокруг начальной ориентации.
    pub ball_limit_deg: Option<f64>,
    pub initial_deg: Option<f64>,
    pub initial_m: Option<f64>,
    pub initial_quat_wxyz: Option<[f64; 4]>,
    pub initial_rpy_deg: Option<[f64; 3]>,
    /// На степень свободы в единицах СИ. Отсутствие задаёт идеализацию.
    pub damping: Option<f64>,
    pub friction_loss: Option<f64>,
    pub armature: Option<f64>,
    /// Паспортные ограничения URDF. Потребитель канала должен применить их сам.
    pub effort_limit_si: Option<f64>,
    pub velocity_limit_si: Option<f64>,
    #[serde(default)]
    pub efforts: Vec<Effort>,
}

impl Joint {
    pub fn new(kind: JointType) -> Self {
        Self {
            kind,
            name: None,
            axis: None,
            anchor_m: [0.0; 3],
            limits_deg: None,
            limits_m: None,
            ball_limit_deg: None,
            initial_deg: None,
            initial_m: None,
            initial_quat_wxyz: None,
            initial_rpy_deg: None,
            damping: None,
            friction_loss: None,
            armature: None,
            effort_limit_si: None,
            velocity_limit_si: None,
            efforts: Vec::new(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Effort {
    pub name: String,
    pub dof: usize,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Mass {
    /// Однородные объёмы всего списка collision суммируются.
    Geometry { material: Material },
    /// Объёмы материала, независимые от контакта и показа.
    Volumes {
        material: Material,
        volumes: Vec<Geometry>,
    },
    Explicit {
        mass_kg: f64,
        com_m: [f64; 3],
        /// [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] в кадре звена, относительно центра масс.
        inertia_kg_m2: [f64; 6],
        #[serde(with = "Source")]
        provenance: Provenance,
    },
    /// Разрешено только без собственной степени свободы. Звено может нести дочерние тела.
    Massless,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Material {
    pub name: String,
    pub density_kg_m3: f64,
    #[serde(with = "Source")]
    pub provenance: Provenance,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Geometry {
    pub shape: Shape,
    #[serde(default)]
    pub pose: Pose,
    /// Переопределяет материал тела при выводе массы из геометрии.
    pub material: Option<Material>,
    pub friction: Option<[f64; 3]>,
    pub contact_dim: Option<u8>,
    pub contact_mask: Option<[u32; 2]>,
    /// Масса данного объёма переопределяет плотность при выводе инерции.
    pub explicit_mass: Option<VolumeMass>,
    pub rgba: Option<[f64; 4]>,
}

impl Geometry {
    pub fn new(shape: Shape) -> Self {
        Self {
            shape,
            pose: Pose::default(),
            material: None,
            friction: None,
            contact_dim: None,
            contact_mask: None,
            explicit_mass: None,
            rgba: None,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VolumeMass {
    pub mass_kg: f64,
    #[serde(with = "Source")]
    pub provenance: Provenance,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Shape {
    /// Полные размеры, в порядке X, Y, Z.
    Box {
        size_m: [f64; 3],
    },
    Sphere {
        radius_m: f64,
    },
    /// Длина цилиндрической части вдоль Z.
    Capsule {
        radius_m: f64,
        length_m: f64,
    },
    Cylinder {
        radius_m: f64,
        length_m: f64,
    },
    Ellipsoid {
        radii_m: [f64; 3],
    },
    /// Относительный путь от описания. Координаты файла умножаются на scale.
    Mesh {
        file: PathBuf,
        #[serde(default = "unit_scale")]
        scale: [f64; 3],
    },
    /// Встроенная треугольная сетка.
    Triangles {
        vertices_m: Vec<[f64; 3]>,
        faces: Vec<[usize; 3]>,
    },
}

fn unit_scale() -> [f64; 3] {
    [1.0; 3]
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MountKind {
    Mechanical,
    Power,
    Signal,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Mount {
    pub name: String,
    pub kind: MountKind,
    /// Например стандарт фланца, разъёма питания или сигнала.
    pub interface: String,
    #[serde(default)]
    pub pose: Pose,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AttachmentMode {
    Design,
    Runtime,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Attachment {
    pub name: String,
    /// Имя точки в виде `экземпляр/звено/точка`. У предмета звено называется как экземпляр.
    pub first: String,
    pub second: String,
    pub mode: AttachmentMode,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Anchor {
    /// Имя звена внутри устройства. Без звена координаты в системе мира.
    pub link: Option<String>,
    #[serde(default)]
    pub pose: Pose,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Constraint {
    Connect {
        name: String,
        first: Anchor,
        second: Anchor,
        #[serde(default = "enabled")]
        active: bool,
    },
    Weld {
        name: String,
        first: Anchor,
        second: Anchor,
        #[serde(default = "enabled")]
        active: bool,
    },
    /// Якорь относительно первого звена. Второй MuJoCo выводит из начальной позы.
    ConnectAtReference {
        name: String,
        first: String,
        second: Option<String>,
        anchor_m: [f64; 3],
        #[serde(default = "enabled")]
        active: bool,
    },
    /// Относительная поза берётся из начального положения звеньев.
    WeldAtReference {
        name: String,
        first: String,
        second: Option<String>,
        #[serde(default = "enabled")]
        active: bool,
    },
}

fn enabled() -> bool {
    true
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Transmission {
    /// Δвыход = ratio · Δвход. Для шестерни обе оси вращательные.
    Gear {
        name: String,
        input: String,
        output: String,
        ratio: f64,
    },
    /// Δвыход = ratio · Δвход. Оси одного типа.
    Belt {
        name: String,
        input: String,
        output: String,
        ratio: f64,
    },
    /// Знак задаёт направление перемещения за один полный оборот входной оси.
    Screw {
        name: String,
        input: String,
        output: String,
        lead_m_per_turn: f64,
    },
    /// Нерастяжимый трос тянет после выбора провисания.
    Cable {
        name: String,
        max_length_m: f64,
        branches: Vec<CableBranch>,
    },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CableBranch {
    /// Вклад ветви в общую длину равен длине ветви / divisor.
    pub divisor: f64,
    pub points: Vec<Anchor>,
}
