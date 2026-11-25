use std::{collections::BTreeMap, path::Path};

use crate::physics::{frame, tensor, tensor_values};
use crate::{Device, FORMAT, Geometry, Joint, JointType, Link, Mass, Pose, Provenance, Shape};

use super::{Node, input_path, parse_xml};

/// Переводит кинематическую модель URDF. Единицы URDF: СИ и радианы.
pub fn import_urdf(path: &Path, text: &str) -> Result<Device, String> {
    parse_xml(text, "robot")
        .and_then(|root| convert(path, &root))
        .map_err(|e| format!("{}: {e}", path.display()))
}

fn convert(path: &Path, root: &Node) -> Result<Device, String> {
    root.check(&["name", "version"], &["link", "joint", "material"])?;
    if root.get("version").is_some_and(|v| v != "1.0") {
        return Err(root.err("поддерживается URDF 1.0"));
    }
    let name = root.need("name")?.into();
    let mut materials = BTreeMap::new();
    for m in root.children.iter().filter(|n| n.tag == "material") {
        let name = m.need("name")?;
        if materials
            .insert(name, color(m, &BTreeMap::new())?)
            .is_some()
        {
            return Err(m.err(format!("материал «{name}» задан дважды")));
        }
    }
    let mut links = Vec::new();
    let mut link_index = BTreeMap::new();
    for n in root.children.iter().filter(|n| n.tag == "link") {
        n.check(&["name"], &["inertial", "visual", "collision"])?;
        let name = n.need("name")?;
        if link_index.insert(name, links.len()).is_some() {
            return Err(n.err(format!("звено «{name}» задано дважды")));
        }
        let mass = n
            .opt("inertial")?
            .map(|i| inertia(path, i))
            .transpose()?
            .unwrap_or(Mass::Massless);
        let mut collision = Vec::new();
        let mut visual = Vec::new();
        for g in &n.children {
            match g.tag.as_str() {
                "collision" => collision.push(geometry(g, &materials, false)?),
                "visual" => visual.push(geometry(g, &materials, true)?),
                _ => {}
            }
        }
        links.push(Link {
            name: name.into(),
            parent: None,
            pose: Pose::default(),
            joint: Joint::new(JointType::Fixed),
            mass,
            collision,
            visual,
            mounts: Vec::new(),
        });
    }
    if links.is_empty() {
        return Err(root.err("нет звеньев <link>"));
    }
    let mut joint_names = std::collections::BTreeSet::new();
    for n in root.children.iter().filter(|n| n.tag == "joint") {
        n.check(
            &["name", "type"],
            &["origin", "parent", "child", "axis", "limit", "dynamics"],
        )?;
        let name = n.need("name")?;
        if !joint_names.insert(name) {
            return Err(n.err(format!("соединение «{name}» задано дважды")));
        }
        let parent = endpoint(n.one("parent")?)?;
        let child = endpoint(n.one("child")?)?;
        if !link_index.contains_key(parent) {
            return Err(n.err(format!("parent: неизвестное звено «{parent}»")));
        }
        let i = *link_index
            .get(child)
            .ok_or_else(|| n.err(format!("child: неизвестное звено «{child}»")))?;
        if links[i].parent.is_some() {
            return Err(n.err(format!("звено «{child}» имеет два родителя")));
        }
        links[i].parent = Some(parent.into());
        links[i].pose = origin(n.opt("origin")?)?;
        links[i].joint = joint(n)?;
    }
    Ok(Device {
        format: FORMAT,
        name,
        schematic: None,
        links,
        constraints: Vec::new(),
        transmissions: Vec::new(),
        drives: Vec::new(),
        sensors: Vec::new(),
        legacy_v2: None,
    })
}

fn endpoint(n: &Node) -> Result<&str, String> {
    n.check(&["link"], &[])?;
    n.need("link")
}

fn origin(n: Option<&Node>) -> Result<Pose, String> {
    let Some(n) = n else {
        return Ok(Pose::default());
    };
    n.check(&["xyz", "rpy"], &[])?;
    Ok(Pose {
        position_m: n.array_or("xyz", [0.0; 3])?,
        rpy_deg: n
            .get("rpy")
            .map(|_| n.array::<3>("rpy").map(|a| a.map(f64::to_degrees)))
            .transpose()?,
        quat_wxyz: None,
    })
}

fn inertia(path: &Path, n: &Node) -> Result<Mass, String> {
    n.check(&[], &["origin", "mass", "inertia"])?;
    let pose = origin(n.opt("origin")?)?;
    let mass = n.one("mass")?;
    mass.check(&["value"], &[])?;
    let i = n.one("inertia")?;
    let keys = ["ixx", "iyy", "izz", "ixy", "ixz", "iyz"];
    i.check(&keys, &[])?;
    let mut v = [0.0; 6];
    for (k, value) in keys.into_iter().zip(&mut v) {
        *value = i.scalar(k)?;
    }
    let orientation = frame(&pose)?.rotation.to_rotation_matrix();
    let matrix = orientation.matrix() * tensor(v) * orientation.matrix().transpose();
    Ok(Mass::Explicit {
        mass_kg: mass.scalar("value")?,
        com_m: pose.position_m,
        inertia_kg_m2: tensor_values(&matrix),
        provenance: Provenance::Estimate(format!(
            "URDF {} {}, первичный источник не указан",
            path.display(),
            n.path
        )),
    })
}

fn color(
    n: &Node,
    materials: &BTreeMap<&str, Option<[f64; 4]>>,
) -> Result<Option<[f64; 4]>, String> {
    n.check(&["name"], &["color"])?;
    let name = n.need("name")?;
    if let Some(c) = n.opt("color")? {
        c.check(&["rgba"], &[])?;
        Ok(Some(c.array("rgba")?))
    } else {
        materials
            .get(name)
            .copied()
            .ok_or_else(|| n.err(format!("материал «{name}» без определения")))
    }
}

fn geometry(
    n: &Node,
    materials: &BTreeMap<&str, Option<[f64; 4]>>,
    visual: bool,
) -> Result<Geometry, String> {
    let allowed: &[_] = if visual {
        &["origin", "geometry", "material"]
    } else {
        &["origin", "geometry"]
    };
    n.check(&["name"], allowed)?;
    let g = n.one("geometry")?;
    g.check(&[], &["box", "sphere", "cylinder", "mesh"])?;
    let [s] = g.children.as_slice() else {
        return Err(g.err("нужна ровно одна форма"));
    };
    let shape = match s.tag.as_str() {
        "box" => {
            s.check(&["size"], &[])?;
            Shape::Box {
                size_m: s.array("size")?,
            }
        }
        "sphere" => {
            s.check(&["radius"], &[])?;
            Shape::Sphere {
                radius_m: s.scalar("radius")?,
            }
        }
        "cylinder" => {
            s.check(&["radius", "length"], &[])?;
            Shape::Cylinder {
                radius_m: s.scalar("radius")?,
                length_m: s.scalar("length")?,
            }
        }
        "mesh" => {
            s.check(&["filename", "scale"], &[])?;
            Shape::Mesh {
                file: input_path(s, "filename")?,
                scale: s.array_or("scale", [1.0; 3])?,
            }
        }
        _ => unreachable!(),
    };
    let mut geometry = Geometry::new(shape);
    geometry.pose = origin(n.opt("origin")?)?;
    geometry.rgba = n
        .opt("material")?
        .map(|m| color(m, materials))
        .transpose()?
        .flatten();
    Ok(geometry)
}

fn joint(n: &Node) -> Result<Joint, String> {
    let kind = n.need("type")?;
    let mut j = Joint::new(match kind {
        "fixed" => JointType::Fixed,
        "revolute" | "continuous" => JointType::Hinge,
        "prismatic" => JointType::Slide,
        "floating" => JointType::Free,
        other => return Err(n.err(format!("неподдерживаемый тип <joint> «{other}»"))),
    });
    if j.kind != JointType::Fixed {
        j.name = Some(n.need("name")?.into());
    }
    if matches!(j.kind, JointType::Hinge | JointType::Slide) {
        j.axis = Some(if let Some(ax) = n.opt("axis")? {
            ax.check(&["xyz"], &[])?;
            ax.array("xyz")?
        } else {
            [1.0, 0.0, 0.0]
        }); // Ось URDF по умолчанию X.
    } else if let Some(ax) = n.opt("axis")? {
        return Err(ax.err("неприменимо к типу соединения"));
    }
    if let Some(lim) = n.opt("limit")? {
        if !matches!(j.kind, JointType::Hinge | JointType::Slide) {
            return Err(lim.err("неприменимо к типу соединения"));
        }
        lim.check(&["lower", "upper", "effort", "velocity"], &[])?;
        if kind != "continuous" {
            let lo = lim.scalar("lower")?;
            let hi = lim.scalar("upper")?;
            // Нейтраль URDF — ноль. Если он за упором, берём ближайшую
            // допустимую координату и сохраняем связь с исходным нулём.
            if lo > hi {
                return Err(lim.err("нижняя граница больше верхней"));
            }
            let q = 0.0_f64.clamp(lo, hi);
            if kind == "revolute" {
                j.limits_deg = Some([lo.to_degrees(), hi.to_degrees()]);
                j.initial_deg = Some(q.to_degrees());
            } else {
                j.limits_m = Some([lo, hi]);
                j.initial_m = Some(q);
            }
        } else if lim.get("lower").is_some() || lim.get("upper").is_some() {
            return Err(lim.err("неподдерживаемые lower/upper для continuous"));
        }
        j.effort_limit_si = Some(lim.scalar("effort")?);
        j.velocity_limit_si = Some(lim.scalar("velocity")?);
    } else if matches!(kind, "revolute" | "prismatic") {
        return Err(n.err("нет обязательного <limit>"));
    }
    if let Some(d) = n.opt("dynamics")? {
        if !matches!(j.kind, JointType::Hinge | JointType::Slide) {
            return Err(d.err("неприменимо к типу соединения"));
        }
        d.check(&["damping", "friction"], &[])?;
        j.damping = d.optional("damping")?;
        j.friction_loss = d.optional("friction")?;
    }
    Ok(j)
}
