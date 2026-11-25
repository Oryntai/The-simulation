use std::{f64::consts::TAU, path::Path};

use nalgebra::{UnitQuaternion, Vector3};
use sim_device::{ActuatorData, Catalog};
use sim_materials::MaterialId;

use crate::physics::{quat, tensor_values};
use crate::{
    Device, FORMAT, Geometry, Joint, JointType, LegacyV2, Link, Mass, Pose, Provenance, Shape,
};

/// Каталог материалов прежнего формата. Для моделей с каталогом приводов
/// вызывающая сторона передаёт свою реализацию Catalog.
pub struct LegacyCatalog;

impl Catalog for LegacyCatalog {
    fn density_kg_m3(&self, material: &str) -> Option<f64> {
        let aliases = [
            (MaterialId::Pla, "pla"),
            (MaterialId::Petg, "petg"),
            (MaterialId::Abs, "abs"),
            (MaterialId::Asa, "asa"),
            (MaterialId::Pa12, "pa12"),
            (MaterialId::PaCf, "pa-cf"),
            (MaterialId::Tpu95a, "tpu95a"),
            (MaterialId::Al6061T6, "al6061-t6"),
            (MaterialId::Al7075T6, "al7075-t6"),
            (MaterialId::SteelS235, "s235"),
            (MaterialId::Stainless304, "inox304"),
            (MaterialId::Ti6Al4V, "ti6al4v"),
            (MaterialId::CfrpTube, "cfrp-tube"),
        ];
        aliases.into_iter().find_map(|(id, key)| {
            let m = id.material();
            (material == key || material == m.name).then_some(m.density)
        })
    }

    fn actuator(&self, _: &str) -> Option<ActuatorData> {
        None
    }
}

/// Переводит разбор прежнего проекта через его же каталог и расчёт инерций.
/// Исходный текст сохраняется побитово, включая поля вне динамической схемы.
pub fn migrate_v2(path: &Path, text: &str, catalog: &dyn Catalog) -> Result<Device, String> {
    let original = sim_device::file::from_text(text)
        .map_err(|e| format!("{}: legacy_v2: {e}", path.display()))?;
    migrate_v2_device(path, &original, text, catalog)
}

/// Переводит уже разобранный проект. Текст должен описывать то же устройство.
pub fn migrate_v2_device(
    path: &Path,
    original: &sim_device::Device,
    text: &str,
    catalog: &dyn Catalog,
) -> Result<Device, String> {
    let read = sim_device::file::from_text(text)
        .map_err(|e| format!("{}: legacy_v2: {e}", path.display()))?;
    if &read != original {
        return Err(format!(
            "{}: legacy_v2.original расходится с переданным устройством",
            path.display()
        ));
    }
    let (built, _) = sim_device::mechanics::build(original, catalog)
        .map_err(|e| format!("{}: legacy_v2.links: {}", path.display(), e.join(", ")))?;
    // Миллиметры прежнего формата переводятся в СИ.
    const MM: f64 = 1e-3;
    let mut used = std::collections::BTreeSet::new();
    let names: Vec<_> = original
        .links
        .iter()
        .enumerate()
        .map(|(i, l)| {
            // Номер звена — его устойчивый идентификатор в старом файле. Если имя
            // можно взять без неоднозначности, сохраняем его как есть.
            let valid = !l.name.is_empty()
                && !l.name.contains('/')
                && !l.name.starts_with("звено-")
                && !l.name.chars().any(|c| c.is_control() || c.is_whitespace());
            if valid && used.insert(l.name.as_str()) {
                l.name.clone()
            } else {
                format!("звено-{}", i + 1)
            }
        })
        .collect();
    let links = original
        .links
        .iter()
        .enumerate()
        .map(|(i, l)| {
            let body = &built.model.bodies[i];
            let inertia = &body.inertia;
            let z = l.direction.unit();
            // Ориентация геометрии меняется отдельно. Система координат звена и ось
            // соединения остаются в исходном кадре родителя.
            let rotation =
                UnitQuaternion::rotation_between(&Vector3::z(), &z).unwrap_or_else(|| {
                    UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI)
                });
            let length_m = l.length_mm * MM;
            let shape = match l.section {
                sim_device::Section::Rect { b_mm, h_mm } => Shape::Box {
                    size_m: [b_mm * MM, h_mm * MM, length_m],
                },
                sim_device::Section::Round { d_mm } => Shape::Cylinder {
                    radius_m: d_mm * MM / 2.0,
                    length_m,
                },
                sim_device::Section::Tube { od_mm, id_mm } => {
                    tube(od_mm * MM / 2.0, id_mm * MM / 2.0, length_m)
                }
            };
            let mut geometry = Geometry::new(shape);
            geometry.pose = Pose {
                position_m: (z * (length_m / 2.0)).into(),
                quat_wxyz: Some(quat(&rotation)),
                rpy_deg: None,
            };
            let mut joint = Joint::new(if l.joint.kind == sim_device::JointKind::Hinge {
                JointType::Hinge
            } else {
                JointType::Fixed
            });
            if joint.kind == JointType::Hinge {
                joint.axis = Some(l.joint.axis.unit().into());
                joint.limits_deg = Some([l.joint.min_deg, l.joint.max_deg]);
                if l.joint.speed_limit_dps > 0.0 {
                    joint.velocity_limit_si = Some(l.joint.speed_limit_dps.to_radians());
                }
                if l.joint.torque_limit_nm > 0.0 {
                    joint.effort_limit_si = Some(l.joint.torque_limit_nm);
                }
                if l.joint.actuator.is_empty() || body.armature > 0.0 {
                    joint.armature = Some(body.armature);
                }
                // У прежнего формата нет отдельной конфигурации запуска. Если ноль
                // за упором, начинаем от ближайшей допустимой координаты.
                if !(l.joint.min_deg..=l.joint.max_deg).contains(&0.0) {
                    joint.initial_deg = Some(0.0_f64.clamp(l.joint.min_deg, l.joint.max_deg));
                }
            }
            Link {
                name: names[i].clone(),
                parent: l.parent.map(|p| names[p].clone()),
                pose: Pose {
                    position_m: l.offset_mm.map(|x| x * MM),
                    ..Pose::default()
                },
                joint,
                mass: Mass::Explicit {
                    mass_kg: inertia.mass,
                    com_m: inertia.com.into(),
                    inertia_kg_m2: tensor_values(&inertia.i_com),
                    provenance: Provenance::Estimate(format!(
                        "legacy_v2.links[{i}], расчёт прежнего формата, материал «{}»",
                        l.material
                    )),
                },
                collision: Vec::new(),
                visual: vec![geometry],
                mounts: Vec::new(),
            }
        })
        .collect();
    Ok(Device {
        format: FORMAT,
        name: original.name.clone(),
        schematic: None,
        links,
        constraints: Vec::new(),
        transmissions: Vec::new(),
        drives: Vec::new(),
        sensors: Vec::new(),
        legacy_v2: Some(LegacyV2 {
            original: text.into(),
        }),
    })
}

pub fn extract_v2(device: &Device) -> Option<&str> {
    device
        .legacy_v2
        .as_ref()
        .map(|section| section.original.as_str())
}

fn tube(outside: f64, inside: f64, length: f64) -> Shape {
    // Только показ. 32 сегмента дают относительную стрелу дуги 1−cos(π/32) < 0,5%.
    const N: usize = 32;
    let mut vertices_m = Vec::new();
    let mut faces = Vec::new();
    for z in [-length / 2.0, length / 2.0] {
        for radius in [outside, inside] {
            for i in 0..N {
                let (sin, cos) = (TAU * i as f64 / N as f64).sin_cos();
                vertices_m.push([radius * cos, radius * sin, z]);
            }
        }
    }
    for i in 0..N {
        let j = (i + 1) % N;
        for [a, b, c, d] in [
            [i, j, 2 * N + j, 2 * N + i],
            [N + j, N + i, 3 * N + i, 3 * N + j],
            [j, i, N + i, N + j],
            [2 * N + i, 2 * N + j, 3 * N + j, 3 * N + i],
        ] {
            faces.extend([[a, b, c], [a, c, d]]);
        }
    }
    Shape::Triangles { vertices_m, faces }
}
