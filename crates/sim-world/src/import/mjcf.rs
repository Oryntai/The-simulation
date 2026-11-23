use std::{
    collections::{BTreeMap, BTreeSet},
    f64::consts::TAU,
    path::{Path, PathBuf},
};

use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

use super::{Node, input_path, parse_xml};
use crate::physics::{axis, frame, pose, quat, tensor_values};
use crate::*;

/// Переводит дерево тел и механические ограничения MJCF в описание устройства.
/// Настройки мира, группы default и плагины требуют преобразования вызывающей стороной.
pub fn import_mjcf(path: &Path, text: &str) -> Result<Device, String> {
    parse_xml(text, "mujoco")
        .and_then(|root| convert(path, &root, false))
        .map_err(|e| format!("{}: {e}", path.display()))
}

/// Переводит также настройки мира MJCF. Зерно задаёт вызывающая сторона.
/// Возвращает мир и описание его единственного размещения для `compile_world`.
pub fn import_mjcf_world(
    path: &Path,
    text: &str,
    seed: u64,
) -> Result<(World, DeviceInput), String> {
    let read = || {
        let root = parse_xml(text, "mujoco")?;
        let device = convert(path, &root, true)?;
        // MuJoCo XML Reference, option: 2 мс, земная гравитация, вакуум.
        let mut timestep = 0.002;
        let mut gravity = [0.0, 0.0, -9.81];
        let mut environment = Environment::default();
        if let Some(option) = root.opt("option")? {
            option.check(
                &["timestep", "gravity", "density", "viscosity", "wind"],
                &[],
            )?;
            timestep = option.optional("timestep")?.unwrap_or(timestep);
            gravity = option.array_or("gravity", gravity)?;
            environment.density_kg_m3 = option.optional("density")?.unwrap_or(0.0);
            environment.viscosity_pa_s = option.optional("viscosity")?.unwrap_or(0.0);
            environment.wind_m_s = option.array_or("wind", [0.0; 3])?;
        }
        let hz = timestep.recip().round();
        if sim_signal::Rate::hz(hz as u64).is_none() || hz.recip() != timestep {
            return Err(
                "/mujoco/option.timestep: шаг должен точно соответствовать целой частоте в Гц"
                    .into(),
            );
        }
        let world = World {
            format: FORMAT,
            name: device.name.clone(),
            seed,
            physics_hz: hz as u64,
            gravity_m_s2: gravity,
            environment,
            devices: vec![Placement {
                name: "mjcf".into(),
                file: path.into(),
                pose: Pose::default(),
            }],
            objects: Vec::new(),
            attachments: Vec::new(),
        sensors: Vec::new(),
        };
        Ok((
            world,
            DeviceInput {
                path: path.into(),
                device,
            },
        ))
    };
    read().map_err(|e: String| format!("{}: {e}", path.display()))
}

const WORLD_LINK: &str = "mjcf-world";

#[derive(Default)]
struct Geometries {
    collision: Vec<Geometry>,
    visual: Vec<Geometry>,
    volumes: Vec<Geometry>,
}

struct Asset {
    shape: Shape,
    exact: bool,
}

struct Model<'a> {
    path: &'a Path,
    radians: bool,
    autolimits: bool,
    inertiafromgeom: Option<bool>,
    inertia_groups: [i32; 2],
    meshes: BTreeMap<String, Asset>,
    links: Vec<Link>,
    link_names: BTreeSet<String>,
    joints: BTreeMap<String, JointType>,
    sites: BTreeMap<String, Anchor>,
    used_sites: BTreeSet<String>,
}

fn convert(path: &Path, root: &Node, include_world: bool) -> Result<Device, String> {
    let mut supported = vec!["compiler", "asset", "worldbody", "equality", "tendon"];
    if include_world {
        supported.push("option");
    }
    root.check(&["model"], &supported)?;
    let mut model = Model {
        path,
        radians: false,
        autolimits: true,
        inertiafromgeom: None,
        // Значение inertiagrouprange MJCF по умолчанию.
        inertia_groups: [0, 5],
        meshes: BTreeMap::new(),
        links: Vec::new(),
        link_names: BTreeSet::new(),
        joints: BTreeMap::new(),
        sites: BTreeMap::new(),
        used_sites: BTreeSet::new(),
    };
    let mut mesh_dir = PathBuf::new();
    if let Some(c) = root.opt("compiler")? {
        c.check(
            &[
                "angle",
                "meshdir",
                "assetdir",
                "inertiafromgeom",
                "inertiagrouprange",
                "autolimits",
                "fusestatic",
                "balanceinertia",
                "usethread",
                "alignfree",
            ],
            &[],
        )?;
        model.radians = match c.get("angle").unwrap_or("degree") {
            "radian" => true,
            "degree" => false,
            value => return Err(c.err(format!("неподдерживаемый angle «{value}»"))),
        };
        model.autolimits = c.flag("autolimits", true)?;
        for key in ["fusestatic", "balanceinertia", "alignfree"] {
            if c.flag(key, false)? {
                return Err(c.err(format!("неподдерживаемое значение {key}=true")));
            }
        }
        c.flag("usethread", false)?;
        model.inertiafromgeom = match c.get("inertiafromgeom").unwrap_or("auto") {
            "auto" => None,
            "true" => Some(true),
            "false" => Some(false),
            value => return Err(c.err(format!("неподдерживаемый inertiafromgeom «{value}»"))),
        };
        let key = if c.get("meshdir").is_some() {
            "meshdir"
        } else {
            "assetdir"
        };
        if c.get(key).is_some() {
            mesh_dir = input_path(c, key)?;
        }
        if c.get("inertiagrouprange").is_some() {
            let range = c
                .need("inertiagrouprange")?
                .split_whitespace()
                .map(|v| c.parse("inertiagrouprange", v))
                .collect::<Result<Vec<i32>, _>>()?;
            model.inertia_groups = range
                .try_into()
                .map_err(|_| c.err("inertiagrouprange: нужны две группы"))?;
            if model.inertia_groups[0] > model.inertia_groups[1] {
                return Err(c.err("inertiagrouprange: обратные границы"));
            }
        }
    }
    if let Some(a) = root.opt("asset")? {
        a.check(&[], &["mesh"])?;
        for mesh in &a.children {
            mesh.check(&["name", "file", "scale", "vertex", "face", "inertia"], &[])?;
            let shape = if mesh.get("file").is_some() {
                if mesh.get("vertex").is_some() || mesh.get("face").is_some() {
                    return Err(mesh.err("смешаны file и vertex/face"));
                }
                Shape::Mesh {
                    file: mesh_dir.join(input_path(mesh, "file")?),
                    scale: mesh.array_or("scale", [1.0; 3])?,
                }
            } else {
                let scale: [f64; 3] = mesh.array_or("scale", [1.0; 3])?;
                crate::physics::positive(&scale, "scale").map_err(|e| mesh.err(e))?;
                let vertices = mesh.floats("vertex")?;
                let faces = mesh
                    .need("face")?
                    .split_whitespace()
                    .map(|v| mesh.parse("face", v))
                    .collect::<Result<Vec<usize>, _>>()?;
                if vertices.len() % 3 != 0 || faces.len() % 3 != 0 {
                    return Err(mesh.err("vertex и face должны содержать тройки"));
                }
                let vertices_m = vertices
                    .as_chunks::<3>()
                    .0
                    .iter()
                    .map(|v| [v[0] * scale[0], v[1] * scale[1], v[2] * scale[2]])
                    .collect();
                let faces = faces.as_chunks::<3>().0.to_vec();
                Shape::Triangles { vertices_m, faces }
            };
            let exact = match mesh.get("inertia").unwrap_or("convex") {
                "exact" => true,
                "convex" => false,
                other => return Err(mesh.err(format!("неподдерживаемый inertia «{other}»"))),
            };
            let name = match mesh.get("name") {
                Some(name) => name.to_owned(),
                None => Path::new(mesh.need("file")?)
                    .file_stem()
                    .and_then(|s| s.to_str())
                    .ok_or_else(|| mesh.err("не удалось вывести имя mesh"))?
                    .into(),
            };
            if model
                .meshes
                .insert(name.clone(), Asset { shape, exact })
                .is_some()
            {
                return Err(mesh.err(format!("сетка «{name}» задана дважды")));
            }
        }
    }
    let space = root.one("worldbody")?;
    space.check(&[], &["body", "geom", "site"])?;
    let equality = root.opt("equality")?;
    let uses_world = space
        .children
        .iter()
        .any(|n| matches!(n.tag.as_str(), "geom" | "site"))
        || equality.is_some_and(|e| {
            e.children
                .iter()
                .any(|n| n.get("body1").is_some() && n.get("body2").is_none())
        });
    if uses_world {
        let Geometries {
            collision, visual, ..
        } = model.geometries(space, false)?;
        model.link_names.insert(WORLD_LINK.into());
        model.links.push(Link {
            name: WORLD_LINK.into(),
            parent: None,
            pose: Pose::default(),
            joint: Joint::new(JointType::Fixed),
            mass: Mass::Massless,
            collision,
            visual,
            mounts: Vec::new(),
        });
    }
    model.read_sites(space, Some(WORLD_LINK))?;
    for n in space.children.iter().filter(|n| n.tag == "body") {
        model.body(n, None)?;
    }
    if model.links.is_empty() {
        return Err(space.err("нет ни одного тела или геометрии"));
    }
    let (constraints, mut transmissions) = model.equalities(equality)?;
    if let Some(t) = root.opt("tendon")? {
        model.tendons(t, &mut transmissions)?;
    }
    if let Some(site) = model.sites.keys().find(|s| !model.used_sites.contains(*s)) {
        return Err(format!(
            "/mujoco/worldbody: <site> «{site}» без механической связи не выражен в схеме"
        ));
    }
    Ok(Device {
        format: FORMAT,
        name: root.get("model").unwrap_or("модель MJCF").into(),
        links: model.links,
        constraints,
        transmissions,
        drives: Vec::new(),
        sensors: Vec::new(),
        legacy_v2: None,
    })
}

impl Model<'_> {
    fn degrees(&self, value: f64) -> f64 {
        if self.radians {
            value.to_degrees()
        } else {
            value
        }
    }

    fn radians(&self, value: f64) -> f64 {
        if self.radians {
            value
        } else {
            value.to_radians()
        }
    }

    fn pose(&self, n: &Node) -> Result<Pose, String> {
        let mut p = Pose {
            position_m: n.array_or("pos", [0.0; 3])?,
            ..Pose::default()
        };
        let keys = ["quat", "euler", "axisangle"];
        if keys.iter().filter(|k| n.get(k).is_some()).count() > 1 {
            return Err(n.err("у позы несколько способов задания ориентации"));
        }
        if n.get("quat").is_some() {
            p.quat_wxyz = Some(n.array("quat")?);
        }
        if n.get("euler").is_some() {
            let [x, y, z] = n.array("euler")?.map(|v| self.radians(v));
            // Последовательность xyz MJCF по умолчанию вращает локальные оси.
            let q = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), x)
                * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), y)
                * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), z);
            p.quat_wxyz = Some(quat(&q));
        }
        if n.get("axisangle").is_some() {
            let [x, y, z, a] = n.array("axisangle")?;
            let axis = axis([x, y, z]).map_err(|e| n.err(e))?;
            p.quat_wxyz = Some(quat(&UnitQuaternion::from_axis_angle(
                &axis,
                self.radians(a),
            )));
        }
        frame(&p).map_err(|e| n.err(e))?;
        Ok(p)
    }

    fn read_sites(&mut self, n: &Node, link: Option<&str>) -> Result<(), String> {
        for s in n.children.iter().filter(|n| n.tag == "site") {
            s.check(&["name", "pos", "quat", "euler", "axisangle"], &[])?;
            let name = s.need("name")?;
            let anchor = Anchor {
                link: link.map(str::to_owned),
                pose: self.pose(s)?,
            };
            if self.sites.insert(name.into(), anchor).is_some() {
                return Err(s.err(format!("точка «{name}» задана дважды")));
            }
        }
        Ok(())
    }

    fn body(&mut self, n: &Node, parent: Option<&str>) -> Result<(), String> {
        n.check(
            &["name", "pos", "quat", "euler", "axisangle"],
            &["body", "joint", "freejoint", "geom", "inertial", "site"],
        )?;
        let name = n.need("name")?;
        if !self.link_names.insert(name.into()) {
            return Err(n.err(format!("тело «{name}» задано дважды")));
        }
        let mut p = self.pose(n)?;
        let joint_node = n.opt("joint")?;
        let freejoint_node = n.opt("freejoint")?;
        if joint_node.is_some() && freejoint_node.is_some() {
            return Err(n.err("сочетание <joint> и <freejoint> на одном теле"));
        }
        let joint = if let Some(j) = joint_node {
            self.joint(j, name, &mut p)?
        } else if let Some(j) = freejoint_node {
            j.check(&["name"], &[])?;
            let mut out = Joint::new(JointType::Free);
            out.name = Some(j.get("name").unwrap_or(name).into());
            out.damping = Some(0.0);
            out.friction_loss = Some(0.0);
            out.armature = Some(0.0);
            out
        } else {
            Joint::new(JointType::Fixed)
        };
        if let Some(j) = &joint.name
            && self.joints.insert(j.clone(), joint.kind).is_some()
        {
            return Err(n.err(format!("соединение «{j}» задано дважды")));
        }
        let explicit = n.opt("inertial")?.map(|i| self.inertia(i)).transpose()?;
        let infer = self.inertiafromgeom != Some(false) && explicit.is_none();
        if self.inertiafromgeom == Some(true) && explicit.is_some() {
            return Err(n.err("<inertial> переопределяется inertiafromgeom=true"));
        }
        let Geometries {
            collision,
            visual,
            volumes,
        } = self.geometries(n, infer)?;
        let mass = if let Some(i) = explicit {
            i
        } else if volumes.is_empty() {
            Mass::Massless
        } else {
            Mass::Volumes {
                material: self.material(),
                volumes,
            }
        };
        self.links.push(Link {
            name: name.into(),
            parent: parent.map(str::to_owned),
            pose: p,
            joint,
            mass,
            collision,
            visual,
            mounts: Vec::new(),
        });
        self.read_sites(n, Some(name))?;
        for child in n.children.iter().filter(|n| n.tag == "body") {
            self.body(child, Some(name))?;
        }
        Ok(())
    }

    fn material(&self) -> Material {
        Material {
            name: "плотность MJCF по умолчанию".into(),
            // XML Reference, body/geom/density: стандартное значение MJCF.
            density_kg_m3: 1000.0,
            provenance: Provenance::Estimate(
                "MuJoCo XML Reference, body/geom/density; материал не идентифицирован".into(),
            ),
        }
    }

    fn inertia(&self, n: &Node) -> Result<Mass, String> {
        n.check(
            &[
                "mass",
                "pos",
                "quat",
                "euler",
                "axisangle",
                "diaginertia",
                "fullinertia",
            ],
            &[],
        )?;
        let p = self.pose(n)?;
        let inertia_kg_m2 = match (n.get("fullinertia"), n.get("diaginertia")) {
            (Some(_), None) if p.quat_wxyz.is_none() => n.array("fullinertia")?,
            (None, Some(_)) => {
                let diag =
                    nalgebra::Matrix3::from_diagonal(&Vector3::from(n.array::<3>("diaginertia")?));
                let r = frame(&p)?.rotation.to_rotation_matrix();
                tensor_values(&(r.matrix() * diag * r.matrix().transpose()))
            }
            _ => {
                return Err(
                    n.err("нужен либо fullinertia, либо diaginertia с опциональной ориентацией")
                );
            }
        };
        Ok(Mass::Explicit {
            mass_kg: n.scalar("mass")?,
            com_m: p.position_m,
            inertia_kg_m2,
            provenance: Provenance::Estimate(format!(
                "MJCF {} {}, первичный источник не указан",
                self.path.display(),
                n.path
            )),
        })
    }

    fn joint(&self, n: &Node, body: &str, p: &mut Pose) -> Result<Joint, String> {
        n.check(
            &[
                "name",
                "type",
                "pos",
                "axis",
                "range",
                "limited",
                "ref",
                "damping",
                "frictionloss",
                "armature",
            ],
            &[],
        )?;
        let kind = match n.get("type").unwrap_or("hinge") {
            "hinge" => JointType::Hinge,
            "slide" => JointType::Slide,
            "ball" => JointType::Ball,
            "free" => JointType::Free,
            other => return Err(n.err(format!("неподдерживаемый тип <joint> «{other}»"))),
        };
        let mut j = Joint::new(kind);
        j.name = Some(n.get("name").unwrap_or(body).into());
        // XML Reference, body/joint: значения вязкости, сухого трения и инерции по умолчанию нулевые.
        j.damping = Some(n.optional("damping")?.unwrap_or(0.0));
        j.friction_loss = Some(n.optional("frictionloss")?.unwrap_or(0.0));
        j.armature = Some(n.optional("armature")?.unwrap_or(0.0));
        if matches!(kind, JointType::Hinge | JointType::Slide) {
            j.axis = Some(n.array_or("axis", [0.0, 0.0, 1.0])?);
        } else if n.get("axis").is_some() {
            return Err(n.err("axis неприменим к ball/free"));
        }
        if n.get("pos").is_some() {
            let pos = n.array("pos")?;
            if matches!(kind, JointType::Hinge | JointType::Ball) {
                j.anchor_m = pos;
            } else if pos != [0.0; 3] {
                return Err(n.err("ненулевой pos неприменим к slide/free"));
            }
        }
        if self.limited(n)? {
            let range = n.array::<2>("range")?;
            match kind {
                JointType::Hinge => j.limits_deg = Some(range.map(|v| self.degrees(v))),
                JointType::Slide => j.limits_m = Some(range),
                JointType::Ball => {
                    if range[0] != 0.0 {
                        return Err(n.err("нижняя граница ball должна быть нулевой"));
                    }
                    j.ball_limit_deg = Some(self.degrees(range[1]));
                }
                _ => return Err(n.err("range неприменим к free")),
            }
        } else if n.get("range").is_some() {
            n.array::<2>("range")?;
        }
        if let Some(reference) = n.optional("ref")? {
            // MJCF хранит ref как координату текущей позы. Формат 3 хранит
            // нулевую позу, поэтому вычитаем движение, которое добавит компилятор.
            let a = axis(j.axis.ok_or_else(|| n.err("ref применим к hinge/slide"))?)
                .map_err(|e| n.err(e))?;
            let inv = if kind == JointType::Hinge {
                j.initial_deg = Some(self.degrees(reference));
                let r = UnitQuaternion::from_axis_angle(&a, -self.radians(reference));
                let anchor = Vector3::from(j.anchor_m);
                Isometry3::from_parts(Translation3::from(anchor - r * anchor), r)
            } else {
                j.initial_m = Some(reference);
                Isometry3::from_parts(
                    Translation3::from(-a.into_inner() * reference),
                    UnitQuaternion::identity(),
                )
            };
            *p = pose(&(frame(p)? * inv));
        }
        Ok(j)
    }

    fn limited(&self, n: &Node) -> Result<bool, String> {
        match n.get("limited").unwrap_or("auto") {
            "true" => Ok(true),
            "false" => Ok(false),
            "auto" if n.get("range").is_none() => Ok(false),
            "auto" if self.autolimits => Ok(true),
            "auto" => Err(n.err("range требует явного limited при autolimits=false")),
            other => Err(n.err(format!("неподдерживаемое значение limited «{other}»"))),
        }
    }

    fn geometries(&self, n: &Node, infer: bool) -> Result<Geometries, String> {
        let mut out = Geometries::default();
        for g in n.children.iter().filter(|n| n.tag == "geom") {
            let (geom, group, exact) = self.geom(g)?;
            if infer
                && (self.inertia_groups[0]..=self.inertia_groups[1]).contains(&group)
                && geom.explicit_mass.as_ref().is_none_or(|m| m.mass_kg > 0.0)
            {
                if !exact {
                    return Err(g.err("для вывода массы по сетке задайте mesh inertia=\"exact\" или явный <inertial>"));
                }
                out.volumes.push(geom.clone());
            }
            if geom.contact_mask != Some([0, 0]) {
                out.collision.push(geom.clone());
            }
            out.visual.push(geom);
        }
        Ok(out)
    }

    fn geom(&self, n: &Node) -> Result<(Geometry, i32, bool), String> {
        n.check(
            &[
                "name",
                "type",
                "size",
                "pos",
                "quat",
                "euler",
                "axisangle",
                "fromto",
                "mesh",
                "mass",
                "density",
                "friction",
                "rgba",
                "group",
                "contype",
                "conaffinity",
                "condim",
            ],
            &[],
        )?;
        let kind = n.get("type").unwrap_or(if n.get("mesh").is_some() {
            "mesh"
        } else {
            "sphere"
        });
        let mut p = self.pose(n)?;
        let size = n
            .get("size")
            .map(|_| n.floats("size"))
            .transpose()?
            .unwrap_or_default();
        let endpoints = n
            .get("fromto")
            .map(|_| n.array::<6>("fromto"))
            .transpose()?;
        if endpoints.is_some() && !matches!(kind, "capsule" | "cylinder") {
            return Err(n.err("неподдерживаемый fromto для этой формы"));
        }
        let length = if let Some([ax, ay, az, bx, by, bz]) = endpoints {
            if n.get("pos").is_some() || p.quat_wxyz.is_some() {
                return Err(n.err("fromto и поза заданы одновременно"));
            }
            let a = Vector3::new(ax, ay, az);
            let b = Vector3::new(bx, by, bz);
            let vector = b - a;
            let dir = axis(vector.into()).map_err(|e| n.err(e))?;
            let rotation = UnitQuaternion::rotation_between(&Vector3::z(), dir.as_ref())
                .unwrap_or_else(|| {
                    UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI)
                });
            p = Pose {
                position_m: (a / 2.0 + b / 2.0).into(),
                quat_wxyz: Some(quat(&rotation)),
                rpy_deg: None,
            };
            Some(vector.norm())
        } else {
            None
        };
        let mut exact = true;
        if n.get("mesh").is_some() && kind != "mesh" {
            return Err(n.err("mesh неприменим к указанному типу geom"));
        }
        let shape = match (kind, size.as_slice()) {
            ("sphere", &[radius_m]) => Shape::Sphere { radius_m },
            ("box", &[x, y, z]) => Shape::Box {
                size_m: [2.0 * x, 2.0 * y, 2.0 * z],
            },
            ("ellipsoid", &[a, b, c]) => Shape::Ellipsoid { radii_m: [a, b, c] },
            ("capsule" | "cylinder", s) => {
                let (radius_m, length_m) = match (s, length) {
                    (&[r, half], None) => (r, 2.0 * half),
                    (&[r], Some(l)) => (r, l),
                    _ => return Err(n.err("неверное число компонентов size для capsule/cylinder")),
                };
                if kind == "capsule" {
                    Shape::Capsule { radius_m, length_m }
                } else {
                    Shape::Cylinder { radius_m, length_m }
                }
            }
            ("mesh", []) => {
                let name = n.need("mesh")?;
                let mesh = self
                    .meshes
                    .get(name)
                    .ok_or_else(|| n.err(format!("неизвестная сетка «{name}»")))?;
                exact = mesh.exact;
                mesh.shape.clone()
            }
            _ => {
                return Err(n.err(format!(
                    "неподдерживаемый <geom> type=«{kind}» или неверный size"
                )));
            }
        };
        let mut g = Geometry::new(shape);
        g.pose = p;
        g.rgba = n.get("rgba").map(|_| n.array("rgba")).transpose()?;
        let mass = n.optional("mass")?;
        let mut zero_density = false;
        if let Some(density) = n.optional("density")? {
            if density < 0.0 {
                return Err(n.err("density должна быть неотрицательной"));
            }
            if density == 0.0 {
                zero_density = true;
            } else {
                g.material = Some(Material {
                    name: "плотность из MJCF".into(),
                    density_kg_m3: density,
                    provenance: Provenance::Estimate(format!(
                        "MJCF {} {}.density, первичный источник не указан",
                        self.path.display(),
                        n.path
                    )),
                });
            }
        }
        g.explicit_mass = mass
            .or_else(|| zero_density.then_some(0.0))
            .map(|mass_kg| VolumeMass {
                mass_kg,
                provenance: Provenance::Estimate(format!(
                    "MJCF {} {}.mass/density, первичный источник не указан",
                    self.path.display(),
                    n.path
                )),
            });
        g.friction = n.get("friction").map(|_| n.array("friction")).transpose()?;
        // MuJoCo XML Reference, body/geom: condim=3, обе маски=1 по умолчанию.
        let mask = |key| n.parse(key, n.get(key).unwrap_or("1"));
        g.contact_mask = Some([mask("contype")?, mask("conaffinity")?]);
        g.contact_dim = Some(n.parse("condim", n.get("condim").unwrap_or("3"))?);
        Ok((g, n.parse("group", n.get("group").unwrap_or("0"))?, exact))
    }

    fn site(&mut self, n: &Node, key: &str) -> Result<Anchor, String> {
        let name = n.need(key)?;
        let a = self
            .sites
            .get(name)
            .ok_or_else(|| n.err(format!("{key}: неизвестная точка «{name}»")))?;
        self.used_sites.insert(name.into());
        Ok(a.clone())
    }

    fn equalities(
        &mut self,
        equality: Option<&Node>,
    ) -> Result<(Vec<Constraint>, Vec<Transmission>), String> {
        let mut constraints = Vec::new();
        let mut transmissions = Vec::new();
        let Some(eq) = equality else {
            return Ok((constraints, transmissions));
        };
        eq.check(&[], &["connect", "weld", "joint"])?;
        for (i, n) in eq.children.iter().enumerate() {
            let name = n
                .get("name")
                .map(str::to_owned)
                .unwrap_or_else(|| format!("связь-{}", i + 1));
            let active = n.flag("active", true)?;
            if n.tag == "joint" {
                n.check(&["name", "active", "joint1", "joint2", "polycoef"], &[])?;
                let input = n.need("joint2")?;
                let output = n.need("joint1")?;
                let [constant, ratio, b, c, d] =
                    n.array_or("polycoef", [0.0, 1.0, 0.0, 0.0, 0.0])?;
                if !active || [constant, b, c, d] != [0.0; 4] {
                    return Err(n.err("поддерживается активная линейная связь без смещения"));
                }
                let a = self
                    .joints
                    .get(input)
                    .ok_or_else(|| n.err(format!("неизвестное соединение «{input}»")))?;
                let b = self
                    .joints
                    .get(output)
                    .ok_or_else(|| n.err(format!("неизвестное соединение «{output}»")))?;
                let (input, output) = (input.into(), output.into());
                transmissions.push(match (a, b) {
                    (JointType::Hinge, JointType::Hinge) => Transmission::Gear {
                        name,
                        input,
                        output,
                        ratio,
                    },
                    (JointType::Slide, JointType::Slide) => Transmission::Belt {
                        name,
                        input,
                        output,
                        ratio,
                    },
                    (JointType::Hinge, JointType::Slide) => Transmission::Screw {
                        name,
                        input,
                        output,
                        lead_m_per_turn: ratio * TAU,
                    },
                    _ => return Err(n.err("неподдерживаемая пара осей")),
                });
                continue;
            }
            if n.get("site1").is_some() || n.get("site2").is_some() {
                n.check(&["name", "active", "site1", "site2"], &[])?;
                let first = self.site(n, "site1")?;
                let second = self.site(n, "site2")?;
                constraints.push(if n.tag == "weld" {
                    Constraint::Weld {
                        name,
                        first,
                        second,
                        active,
                    }
                } else {
                    Constraint::Connect {
                        name,
                        first,
                        second,
                        active,
                    }
                });
            } else {
                let first = n.need("body1")?.into();
                let second = Some(n.get("body2").unwrap_or(WORLD_LINK).to_owned());
                constraints.push(if n.tag == "weld" {
                    n.check(&["name", "active", "body1", "body2"], &[])?;
                    Constraint::WeldAtReference {
                        name,
                        first,
                        second,
                        active,
                    }
                } else {
                    n.check(&["name", "active", "body1", "body2", "anchor"], &[])?;
                    Constraint::ConnectAtReference {
                        name,
                        first,
                        second,
                        anchor_m: n.array("anchor")?,
                        active,
                    }
                });
            }
        }
        Ok((constraints, transmissions))
    }

    fn tendons(&mut self, t: &Node, transmissions: &mut Vec<Transmission>) -> Result<(), String> {
        t.check(&[], &["spatial"])?;
        for (i, n) in t.children.iter().enumerate() {
            n.check(&["name", "limited", "range"], &["site", "pulley"])?;
            if !self.limited(n)? {
                return Err(n.err("<spatial> требует верхнего предела длины"));
            }
            let [min, max_length_m] = n.array("range")?;
            if min != 0.0 {
                return Err(n.err("нижний предел длины должен быть нулевым"));
            }
            let name = n
                .get("name")
                .map(str::to_owned)
                .unwrap_or_else(|| format!("трос-{}", i + 1));
            let mut branches = Vec::new();
            let mut current = CableBranch {
                divisor: 1.0,
                points: Vec::new(),
            };
            for entry in &n.children {
                if entry.tag == "pulley" {
                    entry.check(&["divisor"], &[])?;
                    if !current.points.is_empty() {
                        branches.push(current);
                    } else if current.divisor != 1.0 {
                        return Err(entry.err("последовательные pulley без точек"));
                    }
                    current = CableBranch {
                        divisor: entry.scalar("divisor")?,
                        points: Vec::new(),
                    };
                } else {
                    entry.check(&["site"], &[])?;
                    current.points.push(self.site(entry, "site")?);
                }
            }
            branches.push(current);
            transmissions.push(Transmission::Cable {
                name,
                max_length_m,
                branches,
            });
        }
        Ok(())
    }
}
