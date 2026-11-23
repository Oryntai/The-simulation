//! Паспорта локальных механизмов и разрешённые адреса общей сцены.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write;
use std::path::{Path, PathBuf};

use quick_xml::{XmlVersion, events::Event};
use sim_mujoco::composition::{MjcfFragment, content_fingerprint};
use sim_mujoco::ffi::mjtObj as Obj;
use sim_mujoco::{JointKind, Sim};

use crate::compile::{self, base, id, label};
use crate::physics::{self, frame, quat};
use crate::{
    CompiledBody, CompiledJoint, CompiledJointKind, CompiledWorld, ContactBuildOptions,
    DeviceInput, DriveUse, EffortChannel, MountKind, Pose, Provenance, World,
};

#[derive(Clone, Debug, PartialEq)]
pub struct FragmentInertia {
    pub mass_kg: f64,
    pub com_m: [f64; 3],
    /// Ixx, Iyy, Izz, Ixy, Ixz, Iyz в кадре звена относительно центра масс.
    pub tensor_kg_m2: [f64; 6],
    pub provenance: Provenance,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FragmentPort {
    /// Имя site в исходной модели.
    pub site: String,
    pub kind: MountKind,
    pub interface: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FragmentEffort {
    pub name: String,
    pub joint: String,
    pub dof: usize,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ModelFragment {
    pub name: String,
    /// Исходный файл. От него разрешаются ресурсы и включения. Относительный путь считается от мира.
    pub source: PathBuf,
    /// Подготовленный MJCF. При None читается исходный файл.
    pub mjcf: Option<String>,
    pub pose: Pose,
    pub inertia: BTreeMap<String, FragmentInertia>,
    pub ports: BTreeMap<String, FragmentPort>,
    /// Публичное имя поверхности → локальное имя geom.
    pub surfaces: BTreeMap<String, String>,
    pub efforts: Vec<FragmentEffort>,
    pub drives: Vec<DriveUse>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ResolvedPort {
    pub body: String,
    pub site: String,
    pub local_pose: Pose,
    pub kind: MountKind,
    pub interface: String,
}

/// Адреса в именах движка. Внешний профиль хранит параметры материалов, областей и моделей.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FragmentBindings {
    pub ports: BTreeMap<String, ResolvedPort>,
    pub surfaces: BTreeMap<String, String>,
    pub resources: BTreeMap<PathBuf, u64>,
    /// Отпечаток MJCF, разрешённых привязок, паспортов, экспортов и байтов ресурсов.
    pub fingerprint: u64,
}

impl FragmentBindings {
    /// Проверяет, что файлы для последующей загрузки совпадают со сборкой.
    pub fn verify_resources(&self) -> Result<(), String> {
        for (path, &expected) in &self.resources {
            let bytes = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
            if content_fingerprint(&bytes) != expected {
                return Err(format!("{}: ресурс изменился после сборки", path.display()));
            }
        }
        Ok(())
    }

    /// Обычная точка тела для площадки или узла натяжной связи.
    pub fn mechanical_port(&self, name: &str) -> Result<&ResolvedPort, String> {
        let port = self
            .ports
            .get(name)
            .ok_or_else(|| format!("неизвестный порт «{name}»"))?;
        if port.kind != MountKind::Mechanical {
            return Err(format!("порт «{name}» не механический"));
        }
        Ok(port)
    }

    /// Узлы в порядке от начала к концу, пригодные для `ProcessFrame::body_point`.
    pub fn tension_path(&self, names: &[String]) -> Result<Vec<&ResolvedPort>, String> {
        if names.len() < 2 {
            return Err("трассе нужны хотя бы две точки".into());
        }
        let points = names
            .iter()
            .map(|name| self.mechanical_port(name))
            .collect::<Result<Vec<_>, _>>()?;
        if points.windows(2).any(|p| {
            p[0].body == p[1].body && p[0].local_pose.position_m == p[1].local_pose.position_m
        }) {
            return Err("соседние узлы трассы совпадают".into());
        }
        Ok(points)
    }
}

/// Описания устройств идут в порядке `world.devices`. Имена задаёт мир.
pub fn compile_with_fragments(
    path: &Path,
    world: &World,
    devices: &[DeviceInput],
    fragments: &[ModelFragment],
    contacts: &ContactBuildOptions,
) -> Result<CompiledWorld, String> {
    compile_with_fragment_bindings(path, world, devices, fragments, contacts)
        .map(|(world, _)| world)
}

pub fn compile_with_fragment_bindings(
    path: &Path,
    world: &World,
    devices: &[DeviceInput],
    fragments: &[ModelFragment],
    contacts: &ContactBuildOptions,
) -> Result<(CompiledWorld, FragmentBindings), String> {
    if devices.len() != world.devices.len() {
        return Err(format!(
            "{}: число загруженных описаний отличается от числа размещений",
            path.display()
        ));
    }
    let devices = world
        .devices
        .iter()
        .map(|d| d.name.as_str())
        .zip(devices)
        .collect();
    compile::compile_fragments(path, world, &devices, fragments, contacts)
}

pub(crate) struct PreparedFragment {
    source: PathBuf,
    xml: String,
    pose: physics::Frame,
}

impl PreparedFragment {
    pub(crate) fn prepare(
        world_path: &Path,
        fragment: &ModelFragment,
        resources: &mut BTreeMap<PathBuf, u64>,
    ) -> Result<Self, String> {
        let source = base(world_path).join(&fragment.source);
        let pose = frame(&fragment.pose)?;
        let xml = match &fragment.mjcf {
            Some(xml) => xml.clone(),
            None => read_xml(&source, resources)?,
        };
        let entry = source.canonicalize().unwrap_or_else(|_| source.clone());
        let mut active = BTreeSet::from([entry]);
        let xml = expand(&source, &xml, false, &mut active, resources)?;
        Ok(Self { source, xml, pose })
    }

    pub(crate) fn model<'a>(&'a self, prefix: &'a str) -> MjcfFragment<'a> {
        MjcfFragment {
            prefix,
            source: &self.source,
            xml: &self.xml,
            position_m: self.pose.translation.vector.into(),
            quat_wxyz: quat(&self.pose.rotation),
        }
    }
}

fn read_xml(path: &Path, resources: &mut BTreeMap<PathBuf, u64>) -> Result<String, String> {
    let path = path
        .canonicalize()
        .map_err(|e| format!("{}: {e}", path.display()))?;
    let bytes = std::fs::read(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    let stamp = content_fingerprint(&bytes);
    if resources
        .insert(path.clone(), stamp)
        .is_some_and(|old| old != stamp)
    {
        return Err(format!(
            "{}: файл изменился во время сборки",
            path.display()
        ));
    }
    String::from_utf8(bytes).map_err(|e| format!("{}: {e}", path.display()))
}

fn expand(
    path: &Path,
    xml: &str,
    included: bool,
    active: &mut BTreeSet<PathBuf>,
    resources: &mut BTreeMap<PathBuf, u64>,
) -> Result<String, String> {
    let mut reader = quick_xml::Reader::from_str(xml);
    let mut writer = quick_xml::Writer::new(Vec::new());
    let mut depth = 0_usize;
    let mut root_seen = false;
    loop {
        let event = reader.read_event().map_err(|e| e.to_string())?;
        match &event {
            Event::Start(tag) | Event::Empty(tag) => {
                let name = tag.name();
                if depth == 0 {
                    if root_seen || (!included && name.as_ref() != b"mujoco") {
                        return Err(format!("{}: ожидался один корень MJCF", path.display()));
                    }
                    root_seen = true;
                }
                if name.as_ref() == b"include" {
                    if !matches!(event, Event::Empty(_)) || depth == 0 {
                        return Err("include: ожидается пустой элемент внутри модели".into());
                    }
                    let mut attrs = tag.attributes();
                    let attr = attrs
                        .next()
                        .ok_or("include: не задан file")?
                        .map_err(|e| e.to_string())?;
                    if attr.key.as_ref() != b"file" || attrs.next().is_some() {
                        return Err("include: ожидается один атрибут file".into());
                    }
                    let file = attr
                        .decoded_and_normalized_value(XmlVersion::Implicit1_0, reader.decoder())
                        .map_err(|e| e.to_string())?;
                    if file.is_empty() {
                        return Err("include: не задан file".into());
                    }
                    let file = base(path)
                        .join(file.as_ref())
                        .canonicalize()
                        .map_err(|e| format!("{file}: {e}"))?;
                    if !active.insert(file.clone()) {
                        return Err(format!("цикл включений: {}", file.display()));
                    }
                    let text = read_xml(&file, resources)?;
                    writer
                        .get_mut()
                        .extend(expand(&file, &text, true, active, resources)?.as_bytes());
                    active.remove(&file);
                    continue;
                }
                let inner = depth > 0 || !included;
                depth += usize::from(matches!(event, Event::Start(_)));
                if !inner {
                    continue;
                }
            }
            Event::End(_) => {
                depth = depth
                    .checked_sub(1)
                    .ok_or("закрывающий тег вне корня MJCF")?;
                if included && depth == 0 {
                    continue;
                }
            }
            Event::Decl(_) if included => continue,
            Event::DocType(_) | Event::PI(_) => {
                return Err("неподдерживаемое объявление XML".into());
            }
            Event::Text(text) if depth == 0 && !text.iter().all(u8::is_ascii_whitespace) => {
                return Err("текст вне корня MJCF".into());
            }
            Event::Eof if depth == 0 && root_seen => break,
            Event::Eof => return Err("незавершённый MJCF".into()),
            _ => {}
        }
        writer.write_event(event).map_err(|e| e.to_string())?;
    }
    String::from_utf8(writer.into_inner()).map_err(|e| e.to_string())
}

pub(crate) fn lookup(sim: &Sim, kind: Obj::Type, name: &str) -> Result<usize, String> {
    sim.id(kind, name)
        .ok_or_else(|| format!("в собранной модели нет «{name}»"))
}

pub(crate) fn resolve_port(
    sim: &Sim,
    site: &str,
    kind: MountKind,
    interface: &str,
) -> Result<ResolvedPort, String> {
    label(interface)?;
    let i = lookup(sim, Obj::mjOBJ_SITE, site)?;
    // SAFETY: идентификатор взят из живой модели. Формы массивов описаны в mjModel.
    let (body, position, orientation) = unsafe {
        let model = sim.model();
        (
            *model.site_bodyid.add(i) as usize,
            *model.site_pos.add(3 * i).cast::<[f64; 3]>(),
            *model.site_quat.add(4 * i).cast::<[f64; 4]>(),
        )
    };
    if body == 0 {
        return Err(format!(
            "порт «{site}» должен находиться на именованном теле"
        ));
    }
    let local_pose = Pose {
        position_m: position,
        quat_wxyz: Some(orientation),
        rpy_deg: None,
    };
    frame(&local_pose)?;
    Ok(ResolvedPort {
        body: sim.name(Obj::mjOBJ_BODY, body),
        site: site.into(),
        local_pose,
        kind,
        interface: interface.into(),
    })
}

pub(crate) struct FragmentBody {
    pub metadata: CompiledBody,
    pub tensor: Option<[f64; 6]>,
}

pub(crate) fn read_bodies(
    sim: &Sim,
    fragment: &ModelFragment,
) -> Result<Vec<FragmentBody>, String> {
    let prefix = format!("{}/", fragment.name);
    for local in fragment.inertia.keys() {
        lookup(sim, Obj::mjOBJ_BODY, &format!("{prefix}{local}"))?;
    }
    sim.bodies()
        .into_iter()
        .enumerate()
        .filter(|(_, (name, _))| name.starts_with(&prefix))
        .map(|(i, (name, mass))| {
            let local = &name[prefix.len()..];
            let passport = fragment.inertia.get(local);
            let result = if let Some(p) = passport {
                physics::nonnegative(&[p.mass_kg], "mass_kg")?;
                physics::finite(&p.com_m, "com_m")?;
                physics::finite(&p.tensor_kg_m2, "tensor_kg_m2")?;
                let source = match &p.provenance {
                    Provenance::Derived(s)
                    | Provenance::Measured(s)
                    | Provenance::Paper(s)
                    | Provenance::Datasheet(s)
                    | Provenance::Estimate(s) => s,
                };
                label(source)?;
                let diag: [f64; 3] = physics::tensor(p.tensor_kg_m2)
                    .symmetric_eigen()
                    .eigenvalues
                    .into();
                physics::nonnegative(&diag, "главные моменты инерции")?;
                (
                    p.mass_kg,
                    p.com_m,
                    diag,
                    p.provenance.clone(),
                    Some(p.tensor_kg_m2),
                )
            } else {
                // SAFETY: i получен перечислением тел этой модели.
                let com = unsafe { *sim.model().body_ipos.add(3 * i).cast::<[f64; 3]>() };
                (
                    mass,
                    com,
                    sim.body_inertia(i),
                    Provenance::Estimate("MJCF фрагмента без независимого паспорта".into()),
                    None,
                )
            };
            let (mass_kg, com_m, inertia_kg_m2, provenance, tensor) = result;
            Ok(FragmentBody {
                metadata: CompiledBody {
                    name,
                    owner: fragment.name.clone(),
                    mass_kg,
                    com_m,
                    inertia_kg_m2,
                    provenance,
                },
                tensor,
            })
        })
        .collect()
}

pub(crate) fn read_joints(sim: &Sim, fragment: &ModelFragment) -> Vec<CompiledJoint> {
    let prefix = format!("{}/", fragment.name);
    sim.joints()
        .into_iter()
        .filter(|j| j.name.starts_with(&prefix))
        .map(|j| CompiledJoint {
            name: j.name,
            owner: fragment.name.clone(),
            kind: match j.kind {
                JointKind::Hinge => CompiledJointKind::Hinge,
                JointKind::Slide => CompiledJointKind::Slide,
                JointKind::Ball => CompiledJointKind::Ball,
                JointKind::Free => CompiledJointKind::Free,
            },
        })
        .collect()
}

pub(crate) fn read_efforts(
    joints: &[CompiledJoint],
    fragment: &ModelFragment,
) -> Result<Vec<EffortChannel>, String> {
    let mut names = BTreeSet::new();
    fragment
        .efforts
        .iter()
        .map(|e| {
            id(&e.name)?;
            if !names.insert(&e.name) {
                return Err(format!("повтор входа «{}»", e.name));
            }
            let joint = format!("{}/{}", fragment.name, e.joint);
            let j = joints
                .iter()
                .find(|j| j.name == joint)
                .ok_or_else(|| format!("нет оси «{joint}»"))?;
            // Размерности скоростей шарового, свободного и одноосного соединений MuJoCo.
            let count = match j.kind {
                CompiledJointKind::Hinge | CompiledJointKind::Slide => 1,
                CompiledJointKind::Ball => 3,
                CompiledJointKind::Free => 6,
            };
            if e.dof >= count {
                return Err(format!("нет степени свободы {} у оси «{joint}»", e.dof));
            }
            Ok(EffortChannel {
                name: format!("{}/{}", fragment.name, e.name),
                joint,
                dof: e.dof,
            })
        })
        .collect()
}

pub(crate) fn stamp(bindings: &FragmentBindings, world: &CompiledWorld) -> u64 {
    // Формат привязан к версии сборки, как отпечаток журнала. BTreeMap фиксирует порядок.
    let mut data = world.mjcf.clone();
    write!(
        data,
        "\0{:?}\0{:?}\0{:?}\0{:?}\0{:?}\0{:?}\0{:?}\0{:?}\0{}",
        bindings.ports,
        bindings.surfaces,
        bindings.resources,
        world.bodies,
        world.efforts,
        world.drives,
        world.sensors,
        world.simplifications,
        world.seed
    )
    .unwrap();
    content_fingerprint(data.as_bytes())
}
