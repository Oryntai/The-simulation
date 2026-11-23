//! Механика устройства: масса, худшие моменты в шарнирах и изгиб звеньев.
//!
//! Человек с устройством может наклониться как угодно, поэтому сила тяжести берётся в
//! худшем направлении. Для момента вокруг оси a в заданной позе это точная формула:
//! max по направлению g |a·(r × m·g)| = g·|a × Σ mᵢrᵢ|.

use nalgebra::DVector;
use sim_core::dynamics::{body_coms_world, body_poses, mass_matrix};
use sim_core::spatial::Mat3;
use sim_core::{Body, G_STANDARD, Inertia, Joint, Model, Vec3, Xform};

use crate::model::*;

/// За какое время привод разгоняется до предела скорости, с (оценка для запаса по моменту).
pub const ACCEL_TIME_S: f64 = 0.25;
/// Что конструктору нужно знать о приводе.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActuatorData {
    pub mass_kg: f64,
    /// Инерция ротора через редуктор на выходе, кг·м².
    pub reflected_inertia_kg_m2: Option<f64>,
    pub peak_torque_nm: Option<f64>,
}

/// Откуда брать данные деталей.
pub trait Catalog {
    fn density_kg_m3(&self, material: &str) -> Option<f64>;
    fn actuator(&self, id: &str) -> Option<ActuatorData>;
}

/// Нагрузка на шарнир в худшей позе.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointLoad {
    pub link: usize,
    /// Момент от веса в худшей позе и при худшем наклоне тела, Н·м.
    pub gravity_nm: f64,
    /// Момент на разгон до предела скорости, Н·м.
    pub inertial_nm: f64,
    /// Сколько должен дать привод, Н·м.
    pub required_nm: f64,
    /// Наибольшая инерция на оси, включая привод, кг·м².
    pub inertia_kg_m2: f64,
    /// Известна верхняя граница инерции привода.
    pub inertia_complete: bool,
    pub speed_rad_s: f64,
}

/// Нагрузка у основания звена.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinkLoad {
    pub link: usize,
    pub bending_nm: f64,
    pub shear_n: f64,
    pub mass_kg: f64,
    /// Наибольший момент приводов этого звена и звеньев за ним, если выход заблокирован, Н·м.
    pub stall_nm: f64,
    /// Паспортный пик всех этих приводов известен.
    pub stall_known: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Analysis {
    pub total_mass_kg: f64,
    pub joints: Vec<JointLoad>,
    pub links: Vec<LinkLoad>,
}

/// Собранная модель и соответствие «звено → степень свободы».
pub struct Built {
    pub model: Model,
    pub dof: Vec<Option<usize>>,
}

fn link_inertia(l: &LinkSpec, rho: f64, actuator_mass: f64) -> (Inertia, f64) {
    let len = l.length_mm * 1e-3;
    let u = l.direction.unit();
    let m = rho * l.section.area_m2() * len;
    let r = l.section.outer_radius_m();
    let uut = u * u.transpose();
    let i_com = (Mat3::identity() - uut) * (m * len * len / 12.0 + m * r * r / 4.0)
        + uut * (0.5 * m * r * r);
    let mut total = Inertia::new(m, u * (len / 2.0), i_com);
    if l.end_mass_kg > 0.0 {
        total = total.combine(&Inertia::point(l.end_mass_kg, u * len));
    }
    if actuator_mass > 0.0 {
        total = total.combine(&Inertia::point(actuator_mass, Vec3::zeros()));
    }
    (total, m + l.end_mass_kg + actuator_mass)
}

/// Модель для `sim-core`. Ошибки — понятные пользователю.
pub fn build(d: &Device, cat: &dyn Catalog) -> Result<(Built, Vec<f64>), Vec<String>> {
    let mut errors = d.validate();
    if !errors.is_empty() {
        return Err(errors);
    }
    let mut model = Model::earth();
    let mut dof = Vec::with_capacity(d.links.len());
    let mut masses = Vec::with_capacity(d.links.len());
    let mut nv = 0;
    for (i, l) in d.links.iter().enumerate() {
        let who = format!("звено {} «{}»", i + 1, l.name);
        let Some(rho) = cat
            .density_kg_m3(&l.material)
            .filter(|x| x.is_finite() && *x > 0.0)
        else {
            errors.push(format!("{who}: материала «{}» нет в каталоге", l.material));
            continue;
        };
        let actuator = if l.joint.actuator.is_empty() {
            None
        } else {
            match cat.actuator(&l.joint.actuator) {
                Some(a) => Some(a),
                None => {
                    errors.push(format!(
                        "{who}: привода «{}» нет в каталоге",
                        l.joint.actuator
                    ));
                    continue;
                }
            }
        };
        if !errors.is_empty() {
            continue;
        }
        let (inertia, mass) = link_inertia(l, rho, actuator.map_or(0.0, |a| a.mass_kg));
        if ![mass, inertia.mass]
            .into_iter()
            .chain(inertia.com.iter().copied())
            .chain(inertia.i_com.iter().copied())
            .all(f64::is_finite)
            || actuator.is_some_and(|a| {
                !a.mass_kg.is_finite()
                    || a.mass_kg < 0.0
                    || a.reflected_inertia_kg_m2
                        .is_some_and(|v| !v.is_finite() || v < 0.0)
                    || a.peak_torque_nm.is_some_and(|v| !v.is_finite() || v < 0.0)
            })
        {
            errors.push(format!(
                "{who}: неконечная или неверная масса, инерция или момент"
            ));
            continue;
        }
        let joint = match l.joint.kind {
            JointKind::Fixed => Joint::Fixed,
            JointKind::Hinge => Joint::revolute(l.joint.axis.unit()),
        };
        let o = l.offset_mm;
        let placement = Xform::translation(Vec3::new(o[0], o[1], o[2]) * 1e-3);
        let mut body = Body::new(&l.name, l.parent, joint, placement, inertia);
        if l.joint.kind == JointKind::Hinge {
            body.armature = reflected(l, actuator.as_ref()).unwrap_or(0.0);
            dof.push(Some(nv));
            nv += 1;
        } else {
            dof.push(None);
        }
        model.add_body(body);
        masses.push(mass);
    }
    if errors.is_empty() {
        Ok((Built { model, dof }, masses))
    } else {
        Err(errors)
    }
}

fn reflected(link: &LinkSpec, actuator: Option<&ActuatorData>) -> Option<f64> {
    if link.joint.actuator.is_empty() {
        return Some(0.0);
    }
    let measured = link.settings.inertia_kg_m2.filter(|_| {
        link.settings
            .inertia_source
            .as_ref()
            .is_some_and(|s| !s.trim().is_empty())
    });
    match (measured, actuator?.reflected_inertia_kg_m2) {
        (Some(measured), Some(catalog)) => Some(measured.max(catalog)),
        (measured, catalog) => measured.or(catalog),
    }
}

/// Масса, верхние границы момента по всем позам и нагрузки на детали.
pub fn analyze(d: &Device, cat: &dyn Catalog) -> Result<Analysis, Vec<String>> {
    let (built, masses) = build(d, cat)?;
    let q = DVector::zeros(built.model.nv());
    let frames = body_poses(&built.model, &q);
    let coms = body_coms_world(&built.model, &q);
    let matrix = mass_matrix(&built.model, &q);
    let mut joints = Vec::new();
    let mut links = Vec::new();
    for i in 0..d.links.len() {
        let tree = d.subtree(i);
        let subtree_mass: f64 = tree.iter().map(|&k| masses[k]).sum();
        let hinge = built.dof[i];
        let axis = frames[i].rot * d.links[i].joint.axis.unit();
        let origin = frames[i].pos;
        let mut rigid_moment = Vec3::zeros();
        let (mut flexible_moment, mut flexible_inertia, mut rigid_inertia) = (0.0, 0.0, 0.0);
        let mut variable = false;
        let mut bounds = vec![None; d.links.len()];
        for k in tree {
            let inertia = &built.model.bodies[k].inertia;
            let mass = inertia.mass;
            if k != i {
                let parent = d.links[k].parent.expect("звено поддерева имеет родителя");
                bounds[k] =
                    bounds[parent].map(|r: f64| r + (frames[k].pos - frames[parent].pos).norm());
                if bounds[k].is_none() && built.dof[k].is_some() {
                    bounds[k] = Some((frames[k].pos - origin).norm());
                }
            }
            if let Some(r) = bounds[k] {
                // Неравенство треугольника для длины пути до центра масс и граница
                // I_ось ≤ tr(I_c) + m·r² по теореме Штейнера. Охватывает и позы между отсчётами.
                variable = true;
                let reach = r + inertia.com.norm();
                flexible_moment += mass * reach;
                flexible_inertia += inertia.i_com.trace() + mass * reach * reach;
            } else {
                let relative = coms[k] - origin;
                rigid_moment += relative * mass;
                let local_axis = frames[k].rot.inverse() * axis;
                rigid_inertia += local_axis.dot(&(inertia.i_com * local_axis))
                    + mass * axis.cross(&relative).norm_squared();
            }
        }
        let gravity = (axis.cross(&rigid_moment).norm() + flexible_moment) * G_STANDARD;
        let bending = (rigid_moment.norm() + flexible_moment) * G_STANDARD;
        let j = &d.links[i].joint;
        let reflected = reflected(&d.links[i], cat.actuator(&j.actuator).as_ref());
        let dynamic = if let Some(v) = hinge {
            let inertia = if variable {
                rigid_inertia + flexible_inertia + reflected.unwrap_or(0.0)
            } else {
                matrix[(v, v)]
            };
            let speed_rad_s = j.speed_limit_dps.to_radians();
            let accel = d.links[i].settings.acceleration_s.unwrap_or(ACCEL_TIME_S);
            let inertial_nm = inertia * speed_rad_s / accel;
            let required_nm = gravity + inertial_nm;
            if ![inertia, speed_rad_s, inertial_nm, required_nm]
                .into_iter()
                .all(f64::is_finite)
                || accel <= 0.0
            {
                return Err(vec![format!(
                    "Звено {}: неверная динамическая нагрузка или время разгона",
                    i + 1
                )]);
            }
            joints.push(JointLoad {
                link: i,
                gravity_nm: gravity,
                inertial_nm,
                required_nm,
                inertia_kg_m2: inertia,
                inertia_complete: reflected.is_some(),
                speed_rad_s,
            });
            inertial_nm
        } else {
            0.0
        };
        // При заклинивании или столкновении привод выдаёт свой паспортный пик, какой бы предел
        // ни стоял в прошивке: сбой прошивки не должен ломать несущую деталь. Скорость при этом
        // нулевая, поэтому разгон и стопорный момент не складываются.
        let (mut stall_nm, mut stall_known) = (0.0_f64, true);
        for k in d.subtree(i) {
            let spec = &d.links[k].joint;
            if spec.actuator.is_empty() {
                continue;
            }
            match cat.actuator(&spec.actuator).and_then(|a| a.peak_torque_nm) {
                Some(peak) => stall_nm = stall_nm.max(peak.max(spec.torque_limit_nm)),
                None => {
                    stall_nm = stall_nm.max(spec.torque_limit_nm);
                    stall_known = false;
                }
            }
        }
        let bending_nm = bending + dynamic.max(stall_nm);
        let shear_n = subtree_mass * G_STANDARD;
        if ![bending_nm, shear_n, masses[i]]
            .into_iter()
            .all(f64::is_finite)
        {
            return Err(vec![format!(
                "Звено {}: нечисловой результат механики",
                i + 1
            )]);
        }
        links.push(LinkLoad {
            link: i,
            bending_nm,
            shear_n,
            mass_kg: masses[i],
            stall_nm,
            stall_known,
        });
    }
    let total_mass_kg: f64 = masses.iter().sum();
    if !total_mass_kg.is_finite() {
        return Err(vec!["Неконечная масса устройства".into()]);
    }
    Ok(Analysis {
        total_mass_kg,
        joints,
        links,
    })
}
