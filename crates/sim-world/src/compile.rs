use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::TAU;
use std::fmt::{Display, Write};
use std::path::{Path, PathBuf};

use nalgebra::{Point3, Translation3, Unit, UnitQuaternion};
use sim_mujoco::composition::compose_mjcf;
use sim_mujoco::{Sim, ffi::mjtObj as Obj};
use sim_signal::Rate;

use crate::contact_options::ordered;
use crate::fragments::{self, PreparedFragment};
use crate::physics::{self, Frame, Geom, Vec3, finite, frame, nonnegative, positive, quat};
use crate::*;

#[derive(Clone, Debug, PartialEq)]
pub struct DeviceInput {
    /// Файл описания. По нему разрешаются пути сеток и строятся сообщения об ошибках.
    pub path: PathBuf,
    pub device: Device,
}

/// Компилирует мир и уже загруженные экземпляры. Ключ словаря — имя размещения в мире.
pub fn compile_world(
    path: &Path,
    world: &World,
    devices: &BTreeMap<String, DeviceInput>,
) -> Result<CompiledWorld, String> {
    compile_world_with_contact_options(path, world, devices, &ContactBuildOptions::default())
}

pub fn compile_world_with_contact_options(
    path: &Path,
    world: &World,
    devices: &BTreeMap<String, DeviceInput>,
    options: &ContactBuildOptions,
) -> Result<CompiledWorld, String> {
    let devices = devices
        .iter()
        .map(|(name, input)| (name.as_str(), input))
        .collect();
    compile(path, world, &devices, &[], options, false)
        .map(|(world, _)| world)
        .map_err(|e| format!("{}: {e}", path.display()))
}

pub(crate) fn compile_fragments(
    path: &Path,
    world: &World,
    devices: &BTreeMap<&str, &DeviceInput>,
    fragments: &[ModelFragment],
    options: &ContactBuildOptions,
) -> Result<(CompiledWorld, FragmentBindings), String> {
    compile(path, world, devices, fragments, options, true)
        .map_err(|e| format!("{}: {e}", path.display()))
}

pub(crate) fn base(path: &Path) -> &Path {
    path.parent().unwrap_or(Path::new(""))
}

fn field<T>(result: Result<T, String>, path: impl Display) -> Result<T, String> {
    result.map_err(|e| format!("{path}.{e}"))
}

pub(crate) fn version(version: u32) -> Result<(), String> {
    if version != FORMAT {
        Err(format!("format: ожидался {FORMAT}, получен {version}"))
    } else {
        Ok(())
    }
}

pub(crate) fn id(name: &str) -> Result<(), String> {
    if name.is_empty()
        || name.contains('/')
        || name.chars().any(|c| c.is_control() || c.is_whitespace())
    {
        Err(format!(
            "name: «{name}» должно быть непустым сегментом без пробелов и '/'"
        ))
    } else {
        Ok(())
    }
}

pub(crate) fn label(name: &str) -> Result<(), String> {
    if name.trim().is_empty() || name.chars().any(char::is_control) {
        Err("name: требуется непустое имя без управляющих символов".into())
    } else {
        Ok(())
    }
}

fn unique(set: &mut BTreeSet<String>, name: &str) -> Result<(), String> {
    if set.insert(name.into()) {
        Ok(())
    } else {
        Err(format!("name: имя «{name}» задано дважды"))
    }
}

#[derive(Clone, Default)]
struct Xml {
    text: String,
}

impl Xml {
    fn tag(&mut self, tag: &str, attrs: &[(&str, &str)], leaf: bool) {
        write!(self.text, "<{tag}").unwrap();
        for &(key, value) in attrs {
            write!(self.text, " {key}=\"{}\"", quick_xml::escape::escape(value)).unwrap();
        }
        self.text.push_str(if leaf { "/>\n" } else { ">\n" });
    }

    fn end(&mut self, tag: &str) {
        writeln!(self.text, "</{tag}>").unwrap();
    }

    fn body(&mut self, name: &str, p: &Frame) {
        self.tag(
            "body",
            &[
                ("name", name),
                ("pos", &numbers(p.translation.vector.as_slice())),
                ("quat", &numbers(&quat(&p.rotation))),
            ],
            false,
        );
    }

    fn site(&mut self, name: &str, p: &Frame) {
        self.tag(
            "site",
            &[
                ("name", name),
                ("pos", &numbers(p.translation.vector.as_slice())),
                ("quat", &numbers(&quat(&p.rotation))),
                ("rgba", "0 0 0 0"),
            ],
            true,
        );
    }
}

fn numbers<T: Display>(values: &[T]) -> String {
    let mut out = String::new();
    for (j, value) in values.iter().enumerate() {
        if j > 0 {
            out.push(' ');
        }
        write!(out, "{value}").unwrap();
    }
    out
}

#[derive(Clone)]
struct MountSite {
    site: String,
    body: String,
    kind: MountKind,
    interface: String,
    world: Frame,
    mobile: bool,
}

struct JointData {
    kind: JointType,
    axis: Option<Unit<Vec3>>,
    range: Option<[f64; 2]>,
    initial: f64,
    motion: Frame,
    dynamics: [Option<f64>; 3],
}

fn check_joint(j: &Joint) -> Result<JointData, String> {
    finite(&j.anchor_m, "anchor_m")?;
    if let Some(value) = j.effort_limit_si {
        nonnegative(&[value], "effort_limit_si")?;
    }
    if let Some(value) = j.velocity_limit_si {
        nonnegative(&[value], "velocity_limit_si")?;
    }
    let dynamics = [j.damping, j.friction_loss, j.armature];
    for (value, key) in dynamics
        .into_iter()
        .zip(["damping", "friction_loss", "armature"])
    {
        if let Some(value) = value {
            nonnegative(&[value], key)?;
        }
    }
    for (applies, given, key) in [
        (
            matches!(j.kind, JointType::Hinge | JointType::Slide),
            j.axis.is_some(),
            "axis",
        ),
        (
            j.kind == JointType::Hinge,
            j.limits_deg.is_some(),
            "limits_deg",
        ),
        (j.kind == JointType::Slide, j.limits_m.is_some(), "limits_m"),
        (
            j.kind == JointType::Ball,
            j.ball_limit_deg.is_some(),
            "ball_limit_deg",
        ),
        (
            j.kind == JointType::Hinge,
            j.initial_deg.is_some(),
            "initial_deg",
        ),
        (
            j.kind == JointType::Slide,
            j.initial_m.is_some(),
            "initial_m",
        ),
        (
            j.kind == JointType::Ball,
            j.initial_quat_wxyz.is_some(),
            "initial_quat_wxyz",
        ),
        (
            j.kind == JointType::Ball,
            j.initial_rpy_deg.is_some(),
            "initial_rpy_deg",
        ),
        (
            matches!(j.kind, JointType::Hinge | JointType::Ball),
            j.anchor_m != [0.0; 3],
            "anchor_m",
        ),
        (j.kind != JointType::Fixed, j.name.is_some(), "name"),
        (
            j.kind != JointType::Fixed,
            j.effort_limit_si.is_some(),
            "effort_limit_si",
        ),
        (
            j.kind != JointType::Fixed,
            j.velocity_limit_si.is_some(),
            "velocity_limit_si",
        ),
        (j.kind != JointType::Fixed, !j.efforts.is_empty(), "efforts"),
    ] {
        if given && !applies {
            return Err(format!("{key}: неприменимо к соединению {:?}", j.kind));
        }
    }
    if j.kind == JointType::Fixed && dynamics.iter().any(Option::is_some) {
        return Err("damping/friction_loss/armature: неприменимо к fixed".into());
    }
    if let Some(name) = &j.name {
        id(name)?;
    }
    let mut range = None;
    let mut initial = 0.0;
    let mut a = None;
    let mut motion = Frame::identity();
    let anchor = Vec3::from(j.anchor_m);
    match j.kind {
        JointType::Hinge => {
            a = Some(physics::axis(j.axis.ok_or("axis: требуется ось hinge")?)?);
            initial = j.initial_deg.unwrap_or(0.0).to_radians();
            finite(&[initial], "initial_deg")?;
            range = scalar_range(
                j.limits_deg.map(|r| r.map(f64::to_radians)),
                initial,
                "limits_deg",
            )?;
            let rotation = UnitQuaternion::from_axis_angle(a.as_ref().unwrap(), initial);
            motion = Frame::from_parts(Translation3::from(anchor - rotation * anchor), rotation);
        }
        JointType::Slide => {
            a = Some(physics::axis(j.axis.ok_or("axis: требуется ось slide")?)?);
            initial = j.initial_m.unwrap_or(0.0);
            finite(&[initial], "initial_m")?;
            range = scalar_range(j.limits_m, initial, "limits_m")?;
            motion.translation = Translation3::from(a.as_ref().unwrap().as_ref() * initial);
        }
        JointType::Ball => {
            let initial = Pose {
                position_m: [0.0; 3],
                quat_wxyz: j.initial_quat_wxyz,
                rpy_deg: j.initial_rpy_deg,
            };
            let rotation = field(frame(&initial), "initial")?.rotation;
            motion = Frame::from_parts(Translation3::from(anchor - rotation * anchor), rotation);
            if let Some(limit) = j.ball_limit_deg {
                positive(&[limit], "ball_limit_deg")?;
                // Максимальный угол кратчайшего поворота равен π.
                if limit > 180.0 {
                    return Err("ball_limit_deg: угол конуса больше 180°".into());
                }
                range = Some([0.0, limit.to_radians()]);
            }
        }
        JointType::Fixed | JointType::Free => {}
    }
    Ok(JointData {
        kind: j.kind,
        axis: a,
        range,
        initial,
        motion,
        dynamics,
    })
}

fn check_part(part: &str, params: &BTreeMap<String, f64>) -> Result<(), String> {
    if part.trim().is_empty() {
        return Err("part: не указана позиция каталога".into());
    }
    for (key, value) in params {
        finite(&[*value], &format!("params.{key}"))?;
    }
    Ok(())
}

fn scalar_range(range: Option<[f64; 2]>, q: f64, path: &str) -> Result<Option<[f64; 2]>, String> {
    if let Some([lo, hi]) = range {
        finite(&[lo, hi], path)?;
        if !(lo < hi && q >= lo && q <= hi) {
            return Err(format!(
                "{path}: нужны возрастающие пределы, содержащие начальную координату"
            ));
        }
    }
    Ok(range)
}

#[derive(Default)]
struct Compiler {
    transmission_reference: Option<String>,
    asset: Xml,
    custom: Xml,
    has_colliders: bool,
    space: Xml,
    equality: Xml,
    tendon: Xml,
    bodies: Vec<CompiledBody>,
    joints: Vec<CompiledJoint>,
    efforts: Vec<EffortChannel>,
    drives: Vec<CompiledDrive>,
    sensors: Vec<CompiledSensor>,
    simplifications: Vec<Simplification>,
    mounts: BTreeMap<String, MountSite>,
    equality_names: BTreeSet<String>,
    mesh_count: usize,
    site_count: usize,
    explicit_contact: bool,
}

struct Tree<'a> {
    links: &'a [Link],
    index: BTreeMap<&'a str, usize>,
    children: Vec<Vec<usize>>,
    roots: Vec<usize>,
    joints: Vec<JointData>,
    relative: Vec<Frame>,
    world: Vec<Frame>,
    mobile: Vec<bool>,
}

impl<'a> Tree<'a> {
    fn new(links: &'a [Link], root: &Frame) -> Result<Self, String> {
        if links.is_empty() {
            return Err("links: требуется хотя бы одно звено".into());
        }
        let mut index = BTreeMap::new();
        for (i, link) in links.iter().enumerate() {
            field(id(&link.name), format_args!("links[{i}]"))?;
            if index.insert(link.name.as_str(), i).is_some() {
                return Err(format!(
                    "links[{i}].name: имя «{}» задано дважды",
                    link.name
                ));
            }
        }
        let mut out = Self {
            links,
            index,
            children: vec![Vec::new(); links.len()],
            roots: Vec::new(),
            joints: Vec::new(),
            relative: Vec::new(),
            world: vec![Frame::identity(); links.len()],
            mobile: vec![false; links.len()],
        };
        for (i, link) in links.iter().enumerate() {
            if let Some(parent) = &link.parent {
                let p = out
                    .index
                    .get(parent.as_str())
                    .ok_or_else(|| format!("links[{i}].parent: неизвестное звено «{parent}»"))?;
                out.children[*p].push(i);
                if link.joint.kind == JointType::Free {
                    return Err(format!(
                        "links[{i}].joint.kind: free допускается только в корне"
                    ));
                }
            } else {
                out.roots.push(i);
            }
            let j = field(check_joint(&link.joint), format_args!("links[{i}].joint"))?;
            let r = field(frame(&link.pose), format_args!("links[{i}].pose"))?;
            out.relative.push(r * j.motion);
            out.joints.push(j);
        }

        let mut visited = vec![false; links.len()];
        let mut pending: Vec<_> = out.roots.iter().rev().map(|&i| (i, *root, false)).collect();
        while let Some((i, parent, moving)) = pending.pop() {
            visited[i] = true;
            if links[i].parent.is_none() {
                out.relative[i] = parent * out.relative[i];
            }
            let world = if links[i].parent.is_none() {
                out.relative[i]
            } else {
                parent * out.relative[i]
            };
            finite(
                world.translation.vector.as_slice(),
                &format!("links[{i}].pose.position_m"),
            )?;
            let moving = moving || out.joints[i].kind != JointType::Fixed;
            out.world[i] = world;
            out.mobile[i] = moving;
            pending.extend(out.children[i].iter().rev().map(|&j| (j, world, moving)));
        }
        if let Some(i) = visited.iter().position(|v| !v) {
            return Err(format!(
                "links[{i}].parent: в цепочке родителей обнаружен цикл"
            ));
        }
        Ok(out)
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum GeomMode {
    Collision,
    Visual,
    Inertial,
}

impl Compiler {
    fn note(&mut self, subject: &str, text: &str) {
        self.simplifications.push(Simplification {
            subject: subject.into(),
            text: text.into(),
            blocks_proof: true,
        });
    }

    fn write_geom(&mut self, g: &Geom<'_>, name: &str, mode: GeomMode, density: Option<f64>) {
        let mut size = String::new();
        let mut mesh_name = String::new();
        let kind = match &g.data.shape {
            Shape::Box { size_m } => {
                size = numbers(&size_m.map(|x| x / 2.0));
                "box"
            }
            Shape::Sphere { radius_m } => {
                size = radius_m.to_string();
                "sphere"
            }
            Shape::Cylinder { radius_m, length_m } | Shape::Capsule { radius_m, length_m } => {
                size = numbers(&[*radius_m, length_m / 2.0]);
                if matches!(g.data.shape, Shape::Cylinder { .. }) {
                    "cylinder"
                } else {
                    "capsule"
                }
            }
            Shape::Ellipsoid { radii_m } => {
                size = numbers(radii_m);
                "ellipsoid"
            }
            Shape::Mesh { .. } | Shape::Triangles { .. } => {
                let mesh = g.mesh.as_ref().expect("сетка загружена");
                mesh_name = format!("mesh/{}", self.mesh_count);
                self.mesh_count += 1;
                let vertices: Vec<_> = mesh.vertices_m.iter().flatten().copied().collect();
                let faces: Vec<_> = mesh.faces.iter().flatten().copied().collect();
                let mut attrs = vec![
                    ("name", mesh_name.clone()),
                    ("vertex", numbers(&vertices)),
                    ("face", numbers(&faces)),
                ];
                if density.is_some()
                    && g.data
                        .explicit_mass
                        .as_ref()
                        .is_none_or(|m| m.mass_kg > 0.0)
                {
                    attrs.push(("inertia", "exact".into()));
                }
                let refs: Vec<_> = attrs
                    .iter()
                    .map(|(name, value)| (*name, value.as_str()))
                    .collect();
                self.asset.tag("mesh", &refs, true);
                "mesh"
            }
        };
        let mut attrs = vec![
            ("name", name.into()),
            ("type", kind.into()),
            ("pos", numbers(g.pose.translation.vector.as_slice())),
            ("quat", numbers(&quat(&g.pose.rotation))),
        ];
        if kind == "mesh" {
            attrs.push(("mesh", mesh_name));
        } else {
            attrs.push(("size", size));
        }
        if mode != GeomMode::Collision {
            let group = if mode == GeomMode::Visual { "1" } else { "2" };
            attrs.extend([
                ("contype", "0".into()),
                ("conaffinity", "0".into()),
                ("group", group.into()),
            ]);
            if mode == GeomMode::Inertial {
                attrs.push(("rgba", "0 0 0 0".into()));
            } else if let Some(rgba) = g.data.rgba {
                attrs.push(("rgba", numbers(&rgba)));
            }
        } else {
            self.has_colliders |= g.data.contact_mask != Some([0, 0]);
            attrs.push(("rgba", "0 0 0 0".into()));
            if let Some(friction) = g.data.friction {
                attrs.push(("friction", numbers(&friction)));
            } else if !self.explicit_contact {
                self.note(name, "Контакт использует параметры MuJoCo по умолчанию: коэффициенты геометрии не заданы");
            }
            if let Some(n) = g.data.contact_dim {
                attrs.push(("condim", n.to_string()));
            } else if g.data.friction.is_some() {
                // Все три введённых коэффициента участвуют в контакте, если параметр не уточнён.
                attrs.push(("condim", "6".into()));
            }
            if let Some([kind, affinity]) = g.data.contact_mask {
                attrs.push(("contype", kind.to_string()));
                attrs.push(("conaffinity", affinity.to_string()));
            } else if self.explicit_contact {
                // Обычная группа столкновений по правилам contype/conaffinity
                // MuJoCo 4.0.1. Проверка полного покрытия пар обязательна и здесь.
                attrs.extend([("contype", "1".into()), ("conaffinity", "1".into())]);
            }
            if kind == "mesh" {
                self.note(name, "Контакт использует выпуклую оболочку сетки");
            }
        }
        match (density, g.data.explicit_mass.as_ref().map(|m| m.mass_kg)) {
            (Some(_), Some(kg)) => attrs.push(("mass", kg.to_string())),
            (Some(density), None) => attrs.push(("density", density.to_string())),
            (None, _) => attrs.push(("mass", "0".into())),
        }
        let refs: Vec<_> = attrs
            .iter()
            .map(|(name, value)| (*name, value.as_str()))
            .collect();
        self.space.tag("geom", &refs, true);
    }

    fn geoms<'a>(list: &'a [Geometry], base: &Path, path: &str) -> Result<Vec<Geom<'a>>, String> {
        list.iter()
            .enumerate()
            .map(|(i, g)| field(Geom::new(g, base), format_args!("{path}[{i}]")))
            .collect()
    }

    fn write_body(
        &mut self,
        name: &str,
        mass: &Mass,
        colliders: &[Geometry],
        visual: &[Geometry],
        base: &Path,
        moving: bool,
    ) -> Result<(), String> {
        let colliders = Self::geoms(colliders, base, "collision")?;
        let visual = Self::geoms(visual, base, "visual")?;
        if moving && matches!(mass, Mass::Massless) {
            return Err("mass: подвижному звену нужны масса и инерция".into());
        }
        let volumes = match mass {
            Mass::Volumes { volumes, .. } => Self::geoms(volumes, base, "mass.volumes")?,
            _ => Vec::new(),
        };
        let inertial_geoms = if matches!(mass, Mass::Volumes { .. }) {
            &volumes
        } else {
            &colliders
        };
        let (inertia, inertia_kg_m2, provenance) =
            field(physics::mass(mass, inertial_geoms), "mass")?;
        if !provenance.can_prove() && !matches!(mass, Mass::Massless) {
            self.note(
                name,
                "Источника массы или инерции недостаточно для доказательства",
            );
        }
        if matches!(mass, Mass::Massless) && (!colliders.is_empty() || !visual.is_empty()) {
            self.note(name, "Звено с геометрией объявлено безмассовым");
        }
        if matches!(mass, Mass::Geometry { .. } | Mass::Volumes { .. }) && inertial_geoms.len() > 1
        {
            self.note(
                name,
                "Массы объёмов суммированы. Перекрытие составных объёмов не проверено",
            );
        }
        let tensor = numbers(&physics::tensor_values(&inertia.tensor));
        self.custom.tag(
            "numeric",
            &[("name", &format!("inertia/{name}")), ("data", &tensor)],
            true,
        );
        if matches!(mass, Mass::Explicit { .. }) {
            self.space.tag(
                "inertial",
                &[
                    ("mass", &inertia.kg.to_string()),
                    ("pos", &numbers(inertia.com.as_slice())),
                    ("fullinertia", &tensor),
                ],
                true,
            );
        }
        let (owner, _) = name.split_once('/').expect("составное имя тела");
        self.bodies.push(CompiledBody {
            name: name.into(),
            owner: owner.into(),
            mass_kg: inertia.kg,
            com_m: inertia.com.into(),
            inertia_kg_m2,
            provenance,
        });
        for (i, g) in colliders.iter().enumerate() {
            let density = match mass {
                Mass::Geometry { material } => {
                    Some(g.data.material.as_ref().unwrap_or(material).density_kg_m3)
                }
                _ => None,
            };
            let name = format!("{name}/collision/{i}");
            if density.is_some() && g.mesh.is_some() {
                self.note(
                    &name,
                    "Интеграл объёма сетки предполагает отсутствие самопересечений",
                );
            }
            self.write_geom(g, &name, GeomMode::Collision, density);
        }
        for (i, g) in visual.iter().enumerate() {
            self.write_geom(g, &format!("{name}/visual/{i}"), GeomMode::Visual, None);
        }
        if let Mass::Volumes { material, .. } = mass {
            for (i, g) in volumes.iter().enumerate() {
                let name = format!("{name}/mass/{i}");
                if g.mesh.is_some() {
                    self.note(
                        &name,
                        "Интеграл объёма сетки предполагает отсутствие самопересечений",
                    );
                }
                let density = g.data.material.as_ref().unwrap_or(material).density_kg_m3;
                self.write_geom(g, &name, GeomMode::Inertial, Some(density));
            }
        }
        Ok(())
    }

    fn mount_sites(
        &mut self,
        body: &str,
        world: &Frame,
        mounts: &[Mount],
        mobile: bool,
    ) -> Result<(), String> {
        let mut names = BTreeSet::new();
        for (i, m) in mounts.iter().enumerate() {
            field(
                id(&m.name).and_then(|()| unique(&mut names, &m.name)),
                format_args!("mounts[{i}]"),
            )?;
            field(label(&m.interface), format_args!("mounts[{i}].interface"))?;
            let p = field(frame(&m.pose), format_args!("mounts[{i}].pose"))?;
            let name = format!("{body}/{}", m.name);
            self.space.site(&name, &p);
            self.mounts.insert(
                name.clone(),
                MountSite {
                    site: name,
                    body: body.into(),
                    kind: m.kind,
                    interface: m.interface.clone(),
                    world: world * p,
                    mobile,
                },
            );
        }
        Ok(())
    }

    fn write_joint(
        &mut self,
        full: &str,
        owner: &str,
        j: &Joint,
        data: &JointData,
        efforts: &mut BTreeSet<String>,
    ) -> Result<(), String> {
        let (kind, xml_kind, n) = match data.kind {
            JointType::Hinge => (CompiledJointKind::Hinge, "hinge", 1),
            JointType::Slide => (CompiledJointKind::Slide, "slide", 1),
            JointType::Ball => (CompiledJointKind::Ball, "ball", 3),
            JointType::Free => (CompiledJointKind::Free, "free", 6),
            JointType::Fixed => return Ok(()),
        };
        self.joints.push(CompiledJoint {
            name: full.into(),
            owner: owner.into(),
            kind,
        });
        let mut attrs = vec![("name", full.into()), ("type", xml_kind.into())];
        if let Some(axis) = &data.axis {
            attrs.push(("axis", numbers(axis.as_slice())));
        }
        if j.anchor_m != [0.0; 3] {
            attrs.push(("pos", numbers(&j.anchor_m)));
        }
        if let Some(range) = data.range {
            attrs.extend([("limited", "true".into()), ("range", numbers(&range))]);
        } else {
            attrs.push(("limited", "false".into()));
        }
        if j.initial_deg.is_some() || j.initial_m.is_some() {
            attrs.push(("ref", data.initial.to_string()));
        }
        for (value, key) in data
            .dynamics
            .into_iter()
            .zip(["damping", "frictionloss", "armature"])
        {
            if let Some(value) = value {
                attrs.push((key, value.to_string()));
            } else {
                self.note(
                    full,
                    &format!("{key} не задан: в модели вклад этого эффекта равен нулю"),
                );
            }
        }
        if j.effort_limit_si.is_some() || j.velocity_limit_si.is_some() {
            self.note(full, "Переданные паспортные ограничения усилия и скорости требуется обеспечить в контроллере");
        }
        for (i, channel) in j.efforts.iter().enumerate() {
            field(
                id(&channel.name).and_then(|()| unique(efforts, &channel.name)),
                format_args!("efforts[{i}]"),
            )?;
            if channel.dof >= n {
                return Err(format!(
                    "efforts[{i}].dof: нет степени свободы {}",
                    channel.dof
                ));
            }
            self.efforts.push(EffortChannel {
                name: format!("{owner}/{}", channel.name),
                joint: full.into(),
                dof: channel.dof,
            });
        }
        let refs: Vec<_> = attrs
            .iter()
            .map(|(name, value)| (*name, value.as_str()))
            .collect();
        self.space.tag("joint", &refs, true);
        Ok(())
    }

    fn device(&mut self, name: &str, pose: &Frame, input: &DeviceInput) -> Result<(), String> {
        let d = &input.device;
        version(d.format)?;
        label(&d.name)?;
        if let Some(legacy) = &d.legacy_v2 {
            sim_device::file::from_text(&legacy.original)
                .map_err(|e| format!("legacy_v2.original: {e}"))?;
            self.note(name, "Поля прежнего формата сохранены в legacy_v2; прочность и свойства вне механики требуют отдельных моделей");
        }
        let tree = Tree::new(&d.links, pose)?;
        let mut joints = BTreeMap::new();
        for (i, l) in d.links.iter().enumerate() {
            if l.joint.kind == JointType::Fixed {
                continue;
            }
            let joint = l.joint.name.as_deref().unwrap_or(&l.name);
            let joint = joint.to_owned();
            if joints.insert(joint.clone(), l.joint.kind).is_some() {
                return Err(format!(
                    "links[{i}].joint.name: имя «{joint}» задано дважды"
                ));
            }
        }

        let mut sites = vec![Xml::default(); d.links.len() + 1];
        self.constraints(name, &tree, &d.constraints, &mut sites)?;
        self.transmissions(name, &tree, &joints, &d.transmissions, &mut sites)?;
        self.space.text.push_str(&sites[d.links.len()].text);
        let mut effort_names = BTreeSet::new();
        let mut pending: Vec<_> = tree.roots.iter().rev().map(|&i| (i, false)).collect();
        while let Some((i, close)) = pending.pop() {
            if close {
                self.space.end("body");
                continue;
            }
            let l = &d.links[i];
            let body = format!("{name}/{}", l.name);
            self.space.body(&body, &tree.relative[i]);
            let joint = format!("{name}/{}", l.joint.name.as_deref().unwrap_or(&l.name));
            field(
                self.write_joint(&joint, name, &l.joint, &tree.joints[i], &mut effort_names),
                format_args!("links[{i}].joint"),
            )?;
            field(
                self.write_body(
                    &body,
                    &l.mass,
                    &l.collision,
                    &l.visual,
                    base(&input.path),
                    l.joint.kind != JointType::Fixed,
                ),
                format_args!("links[{i}]"),
            )?;
            field(
                self.mount_sites(&body, &tree.world[i], &l.mounts, tree.mobile[i]),
                format_args!("links[{i}]"),
            )?;
            self.space.text.push_str(&sites[i].text);
            pending.push((i, true));
            pending.extend(tree.children[i].iter().rev().map(|&j| (j, false)));
        }
        self.parts(name, d, &joints)
    }

    /// Приводы и датчики устройства: все ссылки разрешаются при компиляции.
    fn parts(
        &mut self,
        owner: &str,
        d: &Device,
        joints: &BTreeMap<String, JointType>,
    ) -> Result<(), String> {
        let mut drive_names = BTreeSet::new();
        for (i, u) in d.drives.iter().enumerate() {
            let path = format!("drives[{i}]");
            field(
                id(&u.name).and_then(|()| unique(&mut drive_names, &u.name)),
                &path,
            )?;
            field(check_part(&u.part, &u.params), &path)?;
            let joint = d
                .links
                .iter()
                .find(|l| l.joint.efforts.iter().any(|e| e.name == u.effort))
                .map(|l| l.joint.name.as_deref().unwrap_or(&l.name))
                .ok_or_else(|| format!("{path}.effort: нет входа усилия «{}»", u.effort))?;
            let effort = format!("{owner}/{}", u.effort);
            if self.drives.iter().any(|x| x.effort == effort) {
                return Err(format!("{path}.effort: вход «{}» уже занят", u.effort));
            }
            self.drives.push(CompiledDrive {
                name: format!("{owner}/{}", u.name),
                owner: owner.into(),
                part: u.part.clone(),
                effort,
                joint: format!("{owner}/{joint}"),
                params: u.params.clone(),
            });
        }
        let mut sensor_names = BTreeSet::new();
        for (i, u) in d.sensors.iter().enumerate() {
            let path = format!("sensors[{i}]");
            field(
                id(&u.name).and_then(|()| unique(&mut sensor_names, &u.name)),
                &path,
            )?;
            field(check_part(&u.part, &u.params), &path)?;
            let body = match &u.link {
                Some(link) if d.links.iter().any(|l| l.name == *link) => {
                    Some(format!("{owner}/{link}"))
                }
                Some(link) => return Err(format!("{path}.link: нет звена «{link}»")),
                None => None,
            };
            if let Some(joint) = &u.joint
                && !joints.contains_key(joint)
            {
                return Err(format!("{path}.joint: нет соединения «{joint}»"));
            }
            if let Some(drive) = &u.drive
                && !drive_names.contains(drive)
            {
                return Err(format!("{path}.drive: нет привода «{drive}»"));
            }
            let p = field(frame(&u.pose), format_args!("{path}.pose"))?;
            self.sensors.push(CompiledSensor {
                name: format!("{owner}/{}", u.name),
                part: u.part.clone(),
                body,
                position_m: p.translation.vector.into(),
                quat_wxyz: quat(&p.rotation),
                joint: u.joint.as_ref().map(|j| format!("{owner}/{j}")),
                drive: u.drive.as_ref().map(|x| format!("{owner}/{x}")),
                params: u.params.clone(),
            });
        }
        Ok(())
    }

    /// Датчики мира ссылаются на полные имена уже собранных тел и соединений.
    fn world_sensors(&mut self, list: &[SensorUse]) -> Result<(), String> {
        for (i, u) in list.iter().enumerate() {
            let path = format!("sensors[{i}]");
            field(id(&u.name), &path)?;
            field(check_part(&u.part, &u.params), &path)?;
            if self.sensors.iter().any(|s| s.name == u.name) {
                return Err(format!("{path}.name: имя «{}» задано дважды", u.name));
            }
            if let Some(link) = &u.link
                && !self.bodies.iter().any(|b| b.name == *link)
            {
                return Err(format!("{path}.link: нет тела «{link}»"));
            }
            if let Some(joint) = &u.joint
                && !self.joints.iter().any(|j| j.name == *joint)
            {
                return Err(format!("{path}.joint: нет соединения «{joint}»"));
            }
            if let Some(drive) = &u.drive
                && !self.drives.iter().any(|d| d.name == *drive)
            {
                return Err(format!("{path}.drive: нет привода «{drive}»"));
            }
            let p = field(frame(&u.pose), format_args!("{path}.pose"))?;
            self.sensors.push(CompiledSensor {
                name: u.name.clone(),
                part: u.part.clone(),
                body: u.link.clone(),
                position_m: p.translation.vector.into(),
                quat_wxyz: quat(&p.rotation),
                joint: u.joint.clone(),
                drive: u.drive.clone(),
                params: u.params.clone(),
            });
        }
        Ok(())
    }

    fn constraint_name(&mut self, owner: &str, short: &str) -> Result<String, String> {
        id(short)?;
        let name = format!("{owner}/{short}");
        unique(&mut self.equality_names, &name)?;
        self.note(
            &name,
            "Идеальная геометрическая связь в решателе мягких ограничений MuJoCo",
        );
        Ok(name)
    }

    fn point(
        &mut self,
        tree: &Tree<'_>,
        anchor: &Anchor,
        sites: &mut [Xml],
    ) -> Result<(String, usize, Vec3), String> {
        let i = tree.index_of(anchor.link.as_deref())?;
        let p = field(frame(&anchor.pose), "pose")?;
        let name = format!("constraint/site/{}", self.site_count);
        self.site_count += 1;
        sites[i].site(&name, &p);
        let p = tree.world.get(i).copied().unwrap_or_else(Frame::identity)
            * Point3::from(p.translation.vector);
        finite(p.coords.as_slice(), "pose.position_m в мире")?;
        Ok((name, i, p.coords))
    }

    fn constraints(
        &mut self,
        owner: &str,
        tree: &Tree<'_>,
        constraints: &[Constraint],
        sites: &mut [Xml],
    ) -> Result<(), String> {
        for (i, c) in constraints.iter().enumerate() {
            let mut write = || -> Result<(), String> {
                match c {
                    Constraint::Connect {
                        name,
                        first,
                        second,
                        active,
                    }
                    | Constraint::Weld {
                        name,
                        first,
                        second,
                        active,
                    } => {
                        let name = self.constraint_name(owner, name)?;
                        let (a, ia, _) = field(self.point(tree, first, sites), "first")?;
                        let (b, ib, _) = field(self.point(tree, second, sites), "second")?;
                        tree.check_pair(ia, ib)?;
                        let tag = if matches!(c, Constraint::Connect { .. }) {
                            "connect"
                        } else {
                            "weld"
                        };
                        self.equality.tag(
                            tag,
                            &[
                                ("name", &name),
                                ("site1", &a),
                                ("site2", &b),
                                ("active", &active.to_string()),
                            ],
                            true,
                        );
                    }
                    Constraint::ConnectAtReference {
                        name,
                        first,
                        second,
                        active,
                        ..
                    }
                    | Constraint::WeldAtReference {
                        name,
                        first,
                        second,
                        active,
                    } => {
                        let name = self.constraint_name(owner, name)?;
                        let a = field(tree.index_of(Some(first)), "first")?;
                        let b = field(tree.index_of(second.as_deref()), "second")?;
                        tree.check_pair(a, b)?;
                        let first = format!("{owner}/{first}");
                        let second = second.as_ref().map(|l| format!("{owner}/{l}"));
                        let mut entries = vec![
                            ("name", name),
                            ("body1", first),
                            ("active", active.to_string()),
                        ];
                        if let Some(second) = second {
                            entries.push(("body2", second));
                        }
                        let tag = if let Constraint::ConnectAtReference { anchor_m, .. } = c {
                            finite(anchor_m, "anchor_m")?;
                            entries.push(("anchor", numbers(anchor_m)));
                            "connect"
                        } else {
                            "weld"
                        };
                        let refs: Vec<_> = entries.iter().map(|(n, v)| (*n, v.as_str())).collect();
                        self.equality.tag(tag, &refs, true);
                    }
                }
                Ok(())
            };
            field(write(), format_args!("constraints[{i}]"))?;
        }
        Ok(())
    }

    fn transmissions(
        &mut self,
        owner: &str,
        tree: &Tree<'_>,
        joints: &BTreeMap<String, JointType>,
        transmissions: &[Transmission],
        sites: &mut [Xml],
    ) -> Result<(), String> {
        let mut cables = BTreeSet::new();
        for (i, t) in transmissions.iter().enumerate() {
            let mut write = || -> Result<(), String> {
                let (name, input, output, ratio, unit) = match t {
                    Transmission::Gear {
                        name,
                        input,
                        output,
                        ratio,
                    } => (name, input, output, *ratio, "ratio"),
                    Transmission::Belt {
                        name,
                        input,
                        output,
                        ratio,
                    } => (name, input, output, *ratio, "ratio"),
                    Transmission::Screw {
                        name,
                        input,
                        output,
                        lead_m_per_turn,
                    } => (
                        name,
                        input,
                        output,
                        *lead_m_per_turn / TAU,
                        "lead_m_per_turn",
                    ),
                    Transmission::Cable {
                        name,
                        max_length_m,
                        branches,
                    } => {
                        id(name)?;
                        unique(&mut cables, name)?;
                        return self.cable(owner, tree, name, *max_length_m, branches, sites);
                    }
                };
                let name = self.constraint_name(owner, name)?;
                finite(&[ratio], unit)?;
                if ratio == 0.0 {
                    return Err(format!("{unit}: нужен ненулевой коэффициент"));
                }
                let lookup = |j: &str, path| {
                    joints
                        .get(j)
                        .copied()
                        .ok_or_else(|| format!("{path}: неизвестная ось «{j}»"))
                };
                let a = lookup(input, "input")?;
                let b = lookup(output, "output")?;
                let valid = match t {
                    Transmission::Gear { .. } => a == JointType::Hinge && b == JointType::Hinge,
                    Transmission::Belt { .. } => {
                        a == b && matches!(a, JointType::Hinge | JointType::Slide)
                    }
                    Transmission::Screw { .. } => a == JointType::Hinge && b == JointType::Slide,
                    Transmission::Cable { .. } => unreachable!(),
                };
                if !valid || input == output {
                    return Err("input/output: несовместимые оси передачи".into());
                }
                let output = format!("{owner}/{output}");
                let input = format!("{owner}/{input}");
                let polynomial = numbers(&[0.0, ratio, 0.0, 0.0, 0.0]);
                let mut attrs = vec![
                    ("name", name.as_str()),
                    ("joint1", output.as_str()),
                    ("joint2", input.as_str()),
                    ("polycoef", polynomial.as_str()),
                ];
                if let Some(reference) = &self.transmission_reference {
                    attrs.push(("solref", reference));
                }
                self.equality.tag("joint", &attrs, true);
                Ok(())
            };
            field(write(), format_args!("transmissions[{i}]"))?;
        }
        Ok(())
    }

    fn cable(
        &mut self,
        owner: &str,
        tree: &Tree<'_>,
        short: &str,
        max: f64,
        branches: &[CableBranch],
        sites: &mut [Xml],
    ) -> Result<(), String> {
        positive(&[max], "max_length_m")?;
        if branches.is_empty() {
            return Err("branches: нужен путь троса".into());
        }
        let name = format!("{owner}/{short}");
        self.tendon.tag(
            "spatial",
            &[
                ("name", &name),
                ("limited", "true"),
                ("range", &numbers(&[0.0, max])),
            ],
            false,
        );
        let mut has_motion = false;
        for (i, branch) in branches.iter().enumerate() {
            positive(&[branch.divisor], &format!("branches[{i}].divisor"))?;
            if branch.points.len() < 2 {
                return Err(format!("branches[{i}].points: нужны хотя бы две точки"));
            }
            if i > 0 || branch.divisor != 1.0 {
                self.tendon
                    .tag("pulley", &[("divisor", &branch.divisor.to_string())], true);
            }
            let mut prev: Option<(usize, Vec3)> = None;
            for (j, anchor) in branch.points.iter().enumerate() {
                let (site, body, pos) = field(
                    self.point(tree, anchor, sites),
                    format_args!("branches[{i}].points[{j}]"),
                )?;
                if let Some((prev_body, prev_pos)) = prev {
                    positive(
                        &[(pos - prev_pos).norm()],
                        &format!("branches[{i}].points[{j}]: длина участка"),
                    )?;
                    has_motion |=
                        body != prev_body && (tree.is_mobile(body) || tree.is_mobile(prev_body));
                }
                self.tendon.tag("site", &[("site", &site)], true);
                prev = Some((body, pos));
            }
        }
        if !has_motion {
            return Err("branches: ни один участок не связан с подвижной осью".into());
        }
        self.tendon.end("spatial");
        self.note(
            &name,
            "Нерастяжимый трос с идеальными точками прохода и мягким пределом длины",
        );
        Ok(())
    }

    fn bind_fragment(
        &mut self,
        sim: &Sim,
        fragment: &ModelFragment,
        bindings: &mut FragmentBindings,
    ) -> Result<(), String> {
        let qualify = |local: &str| format!("{}/{local}", fragment.name);
        for fragments::FragmentBody {
            metadata: body,
            tensor,
        } in fragments::read_bodies(sim, fragment)?
        {
            if let Some(tensor) = tensor {
                self.custom.tag(
                    "numeric",
                    &[
                        ("name", &format!("inertia/{}", body.name)),
                        ("data", &numbers(&tensor)),
                    ],
                    true,
                );
            }
            if !body.provenance.can_prove() {
                self.note(
                    &body.name,
                    "Масса и инерция требуют подтверждённого источника",
                );
            }
            self.bodies.push(body);
        }
        let joints = fragments::read_joints(sim, fragment);
        self.efforts
            .extend(fragments::read_efforts(&joints, fragment)?);
        self.joints.extend(joints);
        let mut drives = BTreeSet::new();
        for drive in &fragment.drives {
            id(&drive.name)?;
            unique(&mut drives, &drive.name)?;
            check_part(&drive.part, &drive.params)?;
            let effort = qualify(&drive.effort);
            let channel = self
                .efforts
                .iter()
                .find(|p| p.name == effort)
                .ok_or_else(|| format!("нет входа усилия «{effort}»"))?;
            if self.drives.iter().any(|d| d.effort == effort) {
                return Err(format!("вход «{effort}» уже занят"));
            }
            self.drives.push(CompiledDrive {
                name: qualify(&drive.name),
                owner: fragment.name.clone(),
                part: drive.part.clone(),
                effort,
                joint: channel.joint.clone(),
                params: drive.params.clone(),
            });
        }
        for (alias, geom) in &fragment.surfaces {
            id(alias)?;
            let name = qualify(geom);
            fragments::lookup(sim, Obj::mjOBJ_GEOM, &name)?;
            let alias = qualify(alias);
            if alias != name && sim.id(Obj::mjOBJ_GEOM, &alias).is_some() {
                return Err(format!("алиас поверхности «{alias}» совпал с другим geom"));
            }
            bindings.surfaces.insert(alias, name);
        }
        for (alias, port) in &fragment.ports {
            id(alias)?;
            let name = qualify(alias);
            let port =
                fragments::resolve_port(sim, &qualify(&port.site), port.kind, &port.interface)?;
            let body = fragments::lookup(sim, Obj::mjOBJ_BODY, &port.body)?;
            let (pos, rot) = sim.body_pose(body);
            let world = frame(&Pose {
                position_m: pos,
                quat_wxyz: Some(rot),
                rpy_deg: None,
            })? * frame(&port.local_pose)?;
            finite(
                world.translation.vector.as_slice(),
                "положение порта в мире",
            )?;
            // SAFETY: id тела проверен, body_weldid указывает на первое подвижное тело
            // в жёсткой группе. Для неподвижной группы это id мира.
            let mobile = unsafe { *sim.model().body_weldid.add(body) != 0 };
            self.mounts.insert(
                name.clone(),
                MountSite {
                    site: port.site.clone(),
                    body: port.body.clone(),
                    kind: port.kind,
                    interface: port.interface.clone(),
                    world,
                    mobile,
                },
            );
            bindings.ports.insert(name, port);
        }
        Ok(())
    }

    fn attachments<'a>(
        &mut self,
        attachments: impl IntoIterator<Item = &'a Attachment>,
    ) -> Result<(), String> {
        for (i, a) in attachments.into_iter().enumerate() {
            let mut write = || -> Result<(), String> {
                let name = self.constraint_name("attachment", &a.name)?;
                let lookup = |key: &str, value: &str| {
                    self.mounts
                        .get(value)
                        .ok_or_else(|| format!("{key}: неизвестная точка «{value}»"))
                };
                let first = lookup("first", &a.first)?;
                let second = lookup("second", &a.second)?;
                if first.body == second.body {
                    return Err("second: обе точки принадлежат одному телу".into());
                }
                if first.kind != second.kind || first.interface != second.interface {
                    return Err("second: несовместимые типы или интерфейсы точек".into());
                }
                if first.kind != MountKind::Mechanical {
                    return Err("mode: питание и сигнал не образуют механический weld".into());
                }
                if !first.mobile && !second.mobile {
                    return Err("second: оба тела неподвижны".into());
                }
                if a.mode == AttachmentMode::Design {
                    // Сравнение поз допускает только погрешность представления чисел.
                    let scale = first
                        .world
                        .translation
                        .vector
                        .abs()
                        .max()
                        .max(second.world.translation.vector.abs().max())
                        .max(1.0);
                    let tolerance = 64.0 * f64::EPSILON;
                    if (first.world.translation.vector - second.world.translation.vector).norm()
                        > tolerance * scale
                        || first.world.rotation.angle_to(&second.world.rotation) > tolerance
                    {
                        return Err(
                            "second: для design совместите позы точек в начальной конфигурации"
                                .into(),
                        );
                    }
                }
                self.equality.tag(
                    "weld",
                    &[
                        ("name", &name),
                        ("site1", &first.site),
                        ("site2", &second.site),
                        ("active", &(a.mode == AttachmentMode::Design).to_string()),
                    ],
                    true,
                );
                Ok(())
            };
            field(write(), format_args!("attachments[{i}]"))?;
        }
        Ok(())
    }
}

impl Tree<'_> {
    fn index_of(&self, name: Option<&str>) -> Result<usize, String> {
        match name {
            Some(l) => self
                .index
                .get(l)
                .copied()
                .ok_or_else(|| format!("link: неизвестное звено «{l}»")),
            None => Ok(self.links.len()),
        }
    }

    fn is_mobile(&self, i: usize) -> bool {
        self.mobile.get(i).copied().unwrap_or(false)
    }

    fn check_pair(&self, a: usize, b: usize) -> Result<(), String> {
        if a == b {
            return Err("second: связь с тем же телом".into());
        }
        if !self.is_mobile(a) && !self.is_mobile(b) {
            return Err("second: оба тела неподвижны".into());
        }
        Ok(())
    }
}

fn compile(
    path: &Path,
    world: &World,
    devices: &BTreeMap<&str, &DeviceInput>,
    fragments: &[ModelFragment],
    options: &ContactBuildOptions,
    collect_bindings: bool,
) -> Result<(CompiledWorld, FragmentBindings), String> {
    version(world.format)?;
    label(&world.name)?;
    let world_rate =
        Rate::hz(world.physics_hz).ok_or("physics_hz: частота вне шкалы sim-signal")?;
    let physics = options.physics.unwrap_or(world_rate);
    options.validate()?;
    if options
        .transmission_time_constant_s
        .is_some_and(|time| time < 2.0 / physics.as_hz())
    {
        return Err("постоянная времени передачи меньше двух шагов".into());
    }
    if options.solver.is_some() && sim_mujoco::version() != "4.0.1" {
        return Err("contact: настройки проверены для MuJoCo 4.0.1".into());
    }
    finite(&world.gravity_m_s2, "gravity_m_s2")?;
    let env = &world.environment;
    nonnegative(&[env.density_kg_m3], "environment.density_kg_m3")?;
    nonnegative(&[env.viscosity_pa_s], "environment.viscosity_pa_s")?;
    finite(&env.wind_m_s, "environment.wind_m_s")?;

    let mut c = Compiler {
        explicit_contact: options.solver.is_some(),
        // Относительное демпфирование 1 соответствует критической нормальной моде.
        transmission_reference: options
            .transmission_time_constant_s
            .map(|time| numbers(&[time, 1.0])),
        ..Compiler::default()
    };
    if c.transmission_reference.is_some() {
        c.simplifications.push(Simplification {
            subject: "transmission/solref".into(),
            text: "Численная податливость передач требует измерения люфта и жёсткости.".into(),
            blocks_proof: true,
        });
    }
    let mut names = BTreeSet::new();
    let mut used = BTreeSet::new();
    for (i, d) in world.devices.iter().enumerate() {
        let path = format!("devices[{i}]");
        field(
            id(&d.name).and_then(|()| unique(&mut names, &d.name)),
            &path,
        )?;
        // Префикс служебных равенств крепления зарезервирован.
        if d.name == "attachment" {
            return Err(format!("{path}.name: зарезервированное имя"));
        }
        let pose = field(frame(&d.pose), format_args!("{path}.pose"))?;
        let input = devices
            .get(d.name.as_str())
            .ok_or_else(|| format!("{path}.file: отсутствует загруженное описание"))?;
        c.device(&d.name, &pose, input)
            .map_err(|e| format!("{path}.file ({}): {e}", input.path.display()))?;
        used.insert(d.name.as_str());
    }
    if let Some(unused) = devices.keys().find(|key| !used.contains(**key)) {
        return Err(format!(
            "devices: загруженное описание «{unused}» не размещено"
        ));
    }
    let asset_base = base(path);
    for (i, o) in world
        .objects
        .iter()
        .chain(&options.added_objects)
        .enumerate()
    {
        let path = format!("objects[{i}]");
        field(
            id(&o.name).and_then(|()| unique(&mut names, &o.name)),
            &path,
        )?;
        let p = field(frame(&o.pose), format_args!("{path}.pose"))?;
        let name = format!("{0}/{0}", o.name);
        c.space.body(&name, &p);
        field(
            c.write_joint(
                &name,
                &o.name,
                &Joint::new(JointType::Free),
                &check_joint(&Joint::new(JointType::Free))?,
                &mut BTreeSet::new(),
            ),
            format_args!("{path}.joint"),
        )?;
        field(
            c.write_body(&name, &o.mass, &o.collision, &o.visual, asset_base, true),
            &path,
        )?;
        field(c.mount_sites(&name, &p, &o.mounts, true), &path)?;
        c.space.end("body");
    }
    for (mode, geoms, group) in [
        (GeomMode::Collision, &env.collision, "collision"),
        (GeomMode::Visual, &env.visual, "visual"),
    ] {
        let geoms = Compiler::geoms(geoms, base(path), &format!("environment.{group}"))?;
        for (i, g) in geoms.iter().enumerate() {
            c.write_geom(g, &format!("environment/{group}/{i}"), mode, None);
        }
    }
    if env.density_kg_m3 > 0.0 || env.viscosity_pa_s > 0.0 {
        c.note(
            "environment",
            "Силы среды считаются приближённой моделью MuJoCo",
        );
    }
    if (c.has_colliders || !fragments.is_empty()) && !c.explicit_contact {
        c.note("contact", "Мягкий контакт MuJoCo: таблица пар материалов и сходимость по шагу здесь не проверяются");
    }
    if (c.has_colliders || !fragments.is_empty()) && c.explicit_contact {
        c.note("contact", "Явные пары MuJoCo: применимость, сходимость и погрешность ползучести требуют численного доказательства");
    }
    if options.solver.is_some_and(|s| s.noslip_iterations != 0) {
        c.note(
            "contact/solver/noslip",
            "NoSlip используется для диагностики чувствительности решателя",
        );
    }
    if fragments.is_empty() {
        c.attachments(world.attachments.iter().chain(&options.added_attachments))?;
        c.world_sensors(&world.sensors)?;
    }

    let mut xml = Xml::default();
    xml.tag("mujoco", &[("model", &world.name)], false);
    xml.tag(
        "compiler",
        &[
            ("angle", "radian"),
            ("fusestatic", "false"),
            ("inertiafromgeom", "auto"),
            ("balanceinertia", "false"),
            ("usethread", "false"),
            ("alignfree", "false"),
        ],
        true,
    );
    write_options(&mut xml, world, physics, options.solver);
    if options.joint_limit_response.is_some() || options.joint_limit_impedance.is_some() {
        // MuJoCo ограничивает положительный timeconst снизу двумя шагами.
        if options.joint_limit_response.is_some_and(|r| r[0] < 2.0 / physics.as_hz()) {
            return Err("постоянная времени ограничения хода меньше двух шагов".into());
        }
        let response = options.joint_limit_response.map(|r| numbers(&r));
        let impedance = options.joint_limit_impedance.map(|r| numbers(&r));
        let attrs: Vec<_> = [("solreflimit", response.as_deref()), ("solimplimit", impedance.as_deref())]
            .into_iter().filter_map(|(name, value)| value.map(|v| (name, v))).collect();
        xml.tag("default", &[], false);
        xml.tag("joint", &attrs, true);
        xml.end("default");
        c.simplifications.push(Simplification {
            subject: "joint/limit".into(),
            text: "Податливость ограничения хода задана численно. Для физического вывода нужна характеристика упора.".into(),
            blocks_proof: true,
        });
    }
    for (section, body) in [
        ("asset", std::mem::take(&mut c.asset)),
        ("custom", std::mem::take(&mut c.custom)),
        ("worldbody", std::mem::take(&mut c.space)),
        ("tendon", std::mem::take(&mut c.tendon)),
        ("equality", std::mem::take(&mut c.equality)),
    ] {
        if !body.text.is_empty() || section == "worldbody" {
            xml.tag(section, &[], false);
            xml.text.push_str(&body.text);
            xml.end(section);
        }
    }
    let mut bindings = FragmentBindings::default();
    let mut resolved_options;
    let options = if fragments.is_empty() {
        options
    } else {
        let mut prepared = Vec::new();
        for fragment in fragments {
            id(&fragment.name)?;
            unique(&mut names, &fragment.name)?;
            if fragment.name == "attachment" {
                return Err("фрагмент использует зарезервированный префикс attachment".into());
            }
            prepared.push(
                PreparedFragment::prepare(path, fragment, &mut bindings.resources)
                    .map_err(|e| format!("фрагмент «{}»: {e}", fragment.name))?,
            );
        }
        let models: Vec<_> = prepared
            .iter()
            .zip(fragments)
            .map(|(p, f)| p.model(&f.name))
            .collect();
        let base = finish_contact_xml(xml, &[], &[]);
        let joined = compose_mjcf(&base, &models)?;
        for (path, stamp) in joined.resources {
            if bindings
                .resources
                .insert(path, stamp)
                .is_some_and(|old| old != stamp)
            {
                return Err("ресурс изменился во время композиции".into());
            }
        }
        let sim = Sim::from_xml(&joined.xml)?;
        for (name, mount) in &c.mounts {
            bindings.ports.insert(
                name.clone(),
                fragments::resolve_port(&sim, &mount.site, mount.kind, &mount.interface)?,
            );
        }
        for fragment in fragments {
            c.bind_fragment(&sim, fragment, &mut bindings)
                .map_err(|e| format!("фрагмент «{}»: {e}", fragment.name))?;
        }
        c.attachments(world.attachments.iter().chain(&options.added_attachments))?;
        c.world_sensors(&world.sensors)?;
        let body = joined
            .xml
            .trim_end()
            .strip_suffix("</mujoco>")
            .ok_or("нет конца общего MJCF")?;
        xml = Xml { text: body.into() };
        for (section, part) in [("custom", &c.custom), ("equality", &c.equality)] {
            if !part.text.is_empty() {
                xml.tag(section, &[], false);
                xml.text.push_str(&part.text);
                xml.end(section);
            }
        }
        resolved_options = options.clone();
        resolved_options.resolve_geometries(&sim, &bindings.surfaces)?;
        resolved_options.validate()?;
        &resolved_options
    };
    let (mjcf, sim) = compile_contact_scene(xml, options, physics, &mut c.simplifications)?;
    let out = CompiledWorld {
        name: world.name.clone(),
        seed: world.seed,
        physics,
        mjcf,
        bodies: c.bodies,
        joints: c.joints,
        efforts: c.efforts,
        drives: c.drives,
        sensors: c.sensors,
        simplifications: c.simplifications,
    };
    let complete = crate::verify::check_bodies(&sim, &out).map_err(|e| format!("mjcf: {e}"))?;
    if !complete
        && out.bodies.iter().any(|body| {
            sim.id(Obj::mjOBJ_NUMERIC, &format!("inertia/{}", body.name))
                .is_none()
                && fragments
                    .iter()
                    .find(|f| f.name == body.owner)
                    .is_none_or(|f| {
                        body.name
                            .split_once('/')
                            .is_none_or(|(_, local)| f.inertia.contains_key(local))
                    })
        })
    {
        return Err("mjcf: потерян контрольный тензор при сборке модели".into());
    }
    if collect_bindings {
        if fragments.is_empty() {
            for (name, mount) in &c.mounts {
                let port =
                    fragments::resolve_port(&sim, &mount.site, mount.kind, &mount.interface)?;
                bindings.ports.insert(name.clone(), port);
            }
        }
        bindings.verify_resources()?;
        bindings.fingerprint = fragments::stamp(&bindings, &out);
    }
    Ok((out, bindings))
}

fn write_options(xml: &mut Xml, world: &World, physics: Rate, solver: Option<ContactSolver>) {
    let env = &world.environment;
    let mut attrs = vec![
        ("timestep", (1.0 / physics.as_hz()).to_string()),
        ("gravity", numbers(&world.gravity_m_s2)),
        ("density", env.density_kg_m3.to_string()),
        ("viscosity", env.viscosity_pa_s.to_string()),
        ("wind", numbers(&env.wind_m_s)),
    ];
    if let Some(s) = solver {
        let (cone, solver) = s.names();
        attrs.extend([
            ("integrator", "Euler".into()),
            ("cone", cone.into()),
            ("solver", solver.into()),
            ("impratio", s.impratio.to_string()),
            ("tolerance", s.tolerance.to_string()),
            ("iterations", s.iterations.to_string()),
            ("ls_tolerance", s.line_search_tolerance.to_string()),
            ("ls_iterations", s.line_search_iterations.to_string()),
            ("noslip_tolerance", s.noslip_tolerance.to_string()),
            ("noslip_iterations", s.noslip_iterations.to_string()),
            ("ccd_tolerance", s.ccd_tolerance.to_string()),
            ("ccd_iterations", s.ccd_iterations.to_string()),
        ]);
    }
    let refs: Vec<_> = attrs.iter().map(|(k, v)| (*k, v.as_str())).collect();
    xml.tag("option", &refs, solver.is_none());
    if solver.is_some() {
        xml.tag(
            "flag",
            &[
                ("override", "disable"),
                ("autoreset", "disable"),
                ("filterparent", "enable"),
                ("diagexact", "enable"),
            ],
            true,
        );
        xml.end("option");
    }
}

fn finish_contact_xml(mut xml: Xml, exclusions: &[[String; 2]], pairs: &[&ContactPair]) -> String {
    if !exclusions.is_empty() || !pairs.is_empty() {
        xml.tag("contact", &[], false);
        for [a, b] in exclusions {
            xml.tag("exclude", &[("body1", a), ("body2", b)], true);
        }
        for pair in pairs {
            let [a, b] = ordered(pair.geometries.each_ref());
            let p = pair.settings;
            xml.tag(
                "pair",
                &[
                    ("geom1", a),
                    ("geom2", b),
                    ("condim", &p.dim.to_string()),
                    ("friction", &numbers(&p.friction)),
                    ("solref", &numbers(&p.solref)),
                    ("solreffriction", &numbers(&p.solreffriction)),
                    ("solimp", &numbers(&p.solimp)),
                    ("margin", &p.margin_m.to_string()),
                    ("gap", &p.gap_m.to_string()),
                    ("adhesion", &p.adhesion_n.to_string()),
                ],
                true,
            );
        }
        xml.end("contact");
    }
    xml.end("mujoco");
    xml.text
}

fn compile_contact_scene(
    xml: Xml,
    options: &ContactBuildOptions,
    physics: Rate,
    notes: &mut Vec<Simplification>,
) -> Result<(String, Sim), String> {
    let exclusions: Vec<_> = options
        .excluded_bodies
        .iter()
        .cloned()
        .map(ordered)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    let open = |mjcf: &str| Sim::from_xml(mjcf).map_err(|e| format!("mjcf: {e}"));
    let Some(solver) = options.solver else {
        let mjcf = finish_contact_xml(xml, &exclusions, &[]);
        let sim = open(&mjcf)?;
        return Ok((mjcf, sim));
    };
    let baseline = finish_contact_xml(xml.clone(), &exclusions, &[]);
    let sim = open(&baseline)?;

    let mut ordered_pairs: Vec<_> = options.pairs.iter().collect();
    ordered_pairs.sort_by_key(|p| ordered(p.geometries.each_ref()));
    let mut assigned = BTreeMap::new();
    for pair in &ordered_pairs {
        let names = pair.geometries.each_ref().map(String::as_str);
        pair.validate_step(physics, solver.cone)?;
        if !sim.contact_pair_allowed(names)? {
            return Err(format!(
                "contact: маски, исключения или геометрия запрещают пару {names:?}"
            ));
        }
        assigned.insert(ordered(names), pair.settings);
    }
    for candidate in sim.contact_candidates()? {
        if !assigned.contains_key(&ordered(candidate.each_ref().map(String::as_str))) {
            return Err(format!(
                "contact: для пары {candidate:?} отсутствует явная модель"
            ));
        }
    }
    if ordered_pairs.is_empty() {
        solver.verify(&sim, physics)?;
        return Ok((baseline, sim));
    }

    let mjcf = finish_contact_xml(xml, &exclusions, &ordered_pairs);
    let sim = open(&mjcf)?;
    solver.verify(&sim, physics)?;
    if !sim.numerically_ok() {
        return Err("contact: численная авария при сборке".into());
    }
    for pair in &ordered_pairs {
        let names = pair.geometries.each_ref().map(String::as_str);
        if sim.explicit_contact_settings(names)? != Some(pair.settings) {
            return Err(format!("contact: движок изменил параметры пары {names:?}"));
        }
    }
    for active in sim.contact_wrenches()? {
        let names = ordered(active.geometries.each_ref().map(String::as_str));
        if !assigned
            .get(&names)
            .is_some_and(|&expected| active.matches_settings(expected))
        {
            return Err(format!(
                "contact: действующие параметры пары {names:?} не совпадают с заданными"
            ));
        }
    }
    if solver.cone == FrictionCone::Pyramidal && ordered_pairs.iter().any(|p| p.settings.dim != 1) {
        notes.push(Simplification {
            subject: "contact/solver/cone".into(),
            text: "Многогранный конус является приближением трения".into(),
            blocks_proof: true,
        });
    }
    Ok((mjcf, sim))
}
