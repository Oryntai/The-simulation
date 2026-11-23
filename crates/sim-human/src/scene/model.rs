use std::collections::{BTreeMap, BTreeSet};
use std::ffi::{CStr, CString};
use std::path::PathBuf;
use std::ptr::{self, NonNull};

use nalgebra::{Matrix3, Quaternion, Rotation3, Unit, UnitQuaternion, Vector3};
use sim_mujoco::{Sim, ffi, models};

use super::{
    Anthropometry, Axis, AxisCoupling, AxisKind, BaseVariant, LimitStatus, MassProperties,
    ParticipantModel, ParticipantSpec, Port, ProportionMeasure, RegionId, Segment, Side, Source,
    Surface, SurfaceShape, marginals,
};

const BASE: &str = include_str!("../../data/stage5/participant.xml");
const NAMES: &str = include_str!("../../data/stage5/names.tsv");
const SURFACES: &str = include_str!("../../data/stage5/surfaces.tsv");
const PORTS: &str = include_str!("../../data/stage5/ports.tsv");
const RESOURCES: &str = include_str!("../../data/stage5/resources.tsv");
const SOURCE: &str = "assets/third_party/ms_human_700";
const ROOT: &str = "b0000";
const ROOT_JOINT: &str = "jroot";
// Запись MuJoCo использует шесть значащих разрядов. Запас на сумму,
// диагонализацию и повторную нормализацию кватерниона.
const XML_REL_ERROR: f64 = 2.0e-5;
const MATERIAL: &str = "tissue-proxy";
const DOF: ffi::mjtObj::Type = ffi::mjtObj::mjOBJ_JOINT;
const BODY: ffi::mjtObj::Type = ffi::mjtObj::mjOBJ_BODY;
const GEOM: ffi::mjtObj::Type = ffi::mjtObj::mjOBJ_GEOM;
const SITE: ffi::mjtObj::Type = ffi::mjtObj::mjOBJ_SITE;
const MESH: ffi::mjtObj::Type = ffi::mjtObj::mjOBJ_MESH;
const EQ: ffi::mjtObj::Type = ffi::mjtObj::mjOBJ_EQUALITY;

type V = Vector3<f64>;
type Q = UnitQuaternion<f64>;

pub(super) struct SurfaceRow {
    pub id: &'static str,
    geometry: &'static str,
    body: &'static str,
    side: &'static str,
    pub part: &'static str,
    label: &'static str,
    locator: &'static str,
}

pub(super) fn surface_rows() -> impl Iterator<Item = SurfaceRow> {
    SURFACES.lines().map(|line| {
        let [id, geometry, body, side, part, label, locator] = fields(line);
        SurfaceRow {
            id,
            geometry,
            body,
            side,
            part,
            label,
            locator,
        }
    })
}

fn fields<const N: usize>(line: &str) -> [&str; N] {
    // Таблицы сгенерированы вместе с MJCF, сторонние данные здесь не разбираются.
    let mut parts = line.split('\t');
    let row = std::array::from_fn(|_| parts.next().expect("столбец паспорта модели"));
    assert!(parts.next().is_none(), "лишний столбец паспорта модели");
    row
}

fn quat([w, x, y, z]: [f64; 4]) -> Q {
    Q::new_normalize(Quaternion::new(w, x, y, z))
}

fn xyzw(q: Q) -> [f64; 4] {
    let q = q.quaternion();
    [q.w, q.i, q.j, q.k]
}

fn xml_escape(path: &str) -> String {
    path.replace('&', "&amp;")
        .replace('"', "&quot;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}

struct Spec(NonNull<ffi::mjSpec>);

impl Drop for Spec {
    fn drop(&mut self) {
        // Указатель принадлежит этому экземпляру и освобождается один раз.
        unsafe { ffi::mj_deleteSpec(self.0.as_ptr()) };
    }
}

impl Spec {
    fn parse(xml: &str) -> Result<Self, String> {
        let text = CString::new(xml).map_err(|err| err.to_string())?;
        // Размер буфера принят из обёртки sim-mujoco; MuJoCo пишет C-строку.
        let mut error = [0; 2048];
        let parsed = unsafe {
            ffi::mj_parseXMLString(
                text.as_ptr(),
                ptr::null(),
                error.as_mut_ptr(),
                error.len() as i32,
            )
        };
        NonNull::new(parsed).map(Self).ok_or_else(|| {
            format!(
                "не удалось разобрать участника: {}",
                unsafe { CStr::from_ptr(error.as_ptr()) }.to_string_lossy()
            )
        })
    }

    fn elements(&self, kind: ffi::mjtObj::Type) -> Vec<*mut ffi::mjsElement> {
        let spec = self.0.as_ptr();
        let mut elements = Vec::new();
        // Список действителен до уничтожения спецификации. Во время обхода структура стабильна.
        let mut next = unsafe { ffi::mjs_firstElement(spec, kind) };
        while !next.is_null() {
            elements.push(next);
            next = unsafe { ffi::mjs_nextElement(spec, next) };
        }
        elements
    }

    fn xml(&self) -> Result<String, String> {
        let spec = self.0.as_ptr();
        let model = unsafe { ffi::mj_compile(spec, ptr::null()) };
        if model.is_null() {
            let message = unsafe { CStr::from_ptr(ffi::mjs_getError(spec)) };
            return Err(format!(
                "не удалось скомпилировать участника: {}",
                message.to_string_lossy()
            ));
        }
        // Каноническая запись сохраняет значащие разряды и явные инерции.
        unsafe { (*spec).compiler.savecanonical = true };
        let xml = self.save_compiled();
        unsafe { ffi::mj_deleteModel(model) };
        xml
    }

    fn save_compiled(&self) -> Result<String, String> {
        let mut error = [0; 2048];
        let mut output = vec![0];
        let save = |output: &mut [std::ffi::c_char], error: &mut [std::ffi::c_char]| unsafe {
            ffi::mj_saveXMLString(
                self.0.as_ptr(),
                output.as_mut_ptr(),
                output.len() as i32,
                error.as_mut_ptr(),
                error.len() as i32,
            )
        };
        let needed = save(&mut output, &mut error);
        if needed > 0 {
            // Завершающий ноль C-строки.
            output.resize(needed as usize + 1, 0);
        }
        if needed < 0 || save(&mut output, &mut error) != 0 {
            return Err(format!(
                "не удалось сохранить участника: {}",
                unsafe { CStr::from_ptr(error.as_ptr()) }.to_string_lossy()
            ));
        }
        let xml = unsafe { CStr::from_ptr(output.as_ptr()) }
            .to_str()
            .map_err(|err| err.to_string())?;
        Ok(xml.to_owned())
    }
}

fn name(element: *mut ffi::mjsElement) -> String {
    // Имя копируется до изменения модуля. Элемент получен из активной спецификации.
    unsafe { string(ffi::mjs_getName(element)) }
}

unsafe fn string(value: *const ffi::mjString) -> String {
    // Строка принадлежит MjSpec и не изменяется в ходе копирования.
    unsafe {
        CStr::from_ptr(ffi::mjs_getString(value))
            .to_string_lossy()
            .into_owned()
    }
}

fn ffi_array<T>(_: &Sim, values: *const T, count: usize) -> &[T] {
    // Вызовы используют список из ещё живой Sim и сразу копируют данные.
    // Для нулевого числа элементов указатель MuJoCo может быть пустым.
    if count == 0 {
        &[]
    } else {
        unsafe { std::slice::from_raw_parts(values, count) }
    }
}

#[derive(Clone)]
struct RestBody {
    name: String,
    label: &'static str,
    parent: usize,
    position: V,
    rotation: Q,
    mass: f64,
    com: V,
    tensor: Matrix3<f64>,
}

#[derive(Clone)]
struct Proxy<'a> {
    row: &'a SurfaceRow,
    body: usize,
    center: V,
    rotation: Q,
    size: V,
    kind: ffi::mjtGeom::Type,
}

impl Proxy<'_> {
    fn support(&self, axis: V) -> f64 {
        let local = self.rotation.inverse() * axis;
        if self.kind == ffi::mjtGeom::mjGEOM_CAPSULE {
            self.size.x + self.size.y * local.z.abs()
        } else {
            self.size.component_mul(&local).norm()
        }
    }

    fn scaled(&self, scale: V) -> Self {
        let mut out = self.clone();
        out.center.component_mul_assign(&scale);
        if self.kind == ffi::mjtGeom::mjGEOM_CAPSULE {
            // Участки с измеренными поправками — выровненные эллипсоиды.
            out.size *= scale.x;
        } else {
            out.size.component_mul_assign(&scale);
        }
        out
    }

    fn port(&self, point: V, scale: V) -> (V, Q) {
        let new = self.scaled(scale);
        let local = self.rotation.inverse() * (point - self.center);
        let p = if self.kind == ffi::mjtGeom::mjGEOM_CAPSULE {
            local * scale.x
        } else {
            local.component_mul(&scale)
        };
        let normal = if self.kind == ffi::mjtGeom::mjGEOM_CAPSULE {
            let segment = p.z.clamp(-new.size.y, new.size.y);
            V::new(p.x, p.y, p.z - segment)
        } else {
            p.component_div(&new.size.component_mul(&new.size))
        };
        let point = new.center + new.rotation * p;
        let normal = Unit::new_normalize(new.rotation * normal);
        let tangent_axis = if normal.x.abs() <= normal.y.abs() && normal.x.abs() <= normal.z.abs() {
            V::x()
        } else if normal.y.abs() <= normal.z.abs() {
            V::y()
        } else {
            V::z()
        };
        let tangent = tangent_axis.cross(&normal).normalize();
        let frame = Matrix3::from_columns(&[tangent, normal.cross(&tangent), *normal]);
        (
            point,
            Q::from_rotation_matrix(&Rotation3::from_matrix_unchecked(frame)),
        )
    }
}

fn body_data(sim: &Sim, labels: &BTreeMap<&str, &'static str>) -> Vec<RestBody> {
    let model = sim.model();
    // Длины соответствуют массивам mjModel, позиция и кватернион в рамке родителя.
    let positions = ffi_array(sim, model.body_pos.cast::<[f64; 3]>(), sim.nbody());
    let rotations = ffi_array(sim, model.body_quat.cast::<[f64; 4]>(), sim.nbody());
    let coms = ffi_array(sim, model.body_ipos.cast::<[f64; 3]>(), sim.nbody());
    let irot = ffi_array(sim, model.body_iquat.cast::<[f64; 4]>(), sim.nbody());
    let parents = ffi_array(sim, model.body_parentid, sim.nbody());
    (0..sim.nbody())
        .map(|i| {
            let name = sim.name(BODY, i);
            let label = labels
                .get(name.as_str())
                .copied()
                .unwrap_or("мировая рамка");
            let r = quat(irot[i]).to_rotation_matrix();
            RestBody {
                name,
                label,
                parent: parents[i] as usize,
                position: positions[i].into(),
                rotation: quat(rotations[i]),
                mass: sim.body_mass(i),
                com: coms[i].into(),
                tensor: r.matrix()
                    * Matrix3::from_diagonal(&sim.body_inertia(i).into())
                    * r.matrix().transpose(),
            }
        })
        .collect()
}

fn scale_inertia(inertia: Matrix3<f64>, scale: V, k: f64) -> Matrix3<f64> {
    // Q = ∫rrᵀ dm, I = tr(Q)E − Q. Для S: Q' = k·SQSᵀ.
    let second_moment = Matrix3::identity() * (inertia.trace() / 2.0) - inertia;
    let s = Matrix3::from_diagonal(&scale);
    let transformed = s * second_moment * s * k;
    Matrix3::identity() * transformed.trace() - transformed
}

fn neutral_poses(bodies: &[RestBody], scales: &[V], root_z: f64) -> Vec<(V, Q)> {
    let mut poses = vec![(V::zeros(), Q::identity())];
    for body in &bodies[1..] {
        let (origin, rotation) = poses[body.parent];
        let shift = if body.parent == 0 {
            V::z() * root_z
        } else {
            V::zeros()
        };
        let point = origin + rotation * body.position.component_mul(&scales[body.parent]) + shift;
        poses.push((point, rotation * body.rotation));
    }
    poses
}

fn vertical_bounds(proxies: &[Proxy<'_>], poses: &[(V, Q)], scales: &[V]) -> [f64; 2] {
    proxies
        .iter()
        .fold([f64::INFINITY, f64::NEG_INFINITY], |[lo, hi], proxy| {
            let (origin, rotation) = poses[proxy.body];
            let p = proxy.scaled(scales[proxy.body]);
            let z = (origin + rotation * p.center).z;
            let reach = p.support(rotation.inverse() * V::z());
            [lo.min(z - reach), hi.max(z + reach)]
        })
}

fn model_inertia(
    bodies: &[RestBody],
    poses: &[(V, Q)],
    scales: &[V],
    k: f64,
) -> Result<MassProperties, String> {
    let mut com = V::zeros();
    let mut mass_kg = 0.0;
    let mut segments = Vec::new();
    let mut world_segments = Vec::new();
    for (i, body) in bodies.iter().enumerate().skip(1) {
        let mass = body.mass * k;
        let local_com = body.com.component_mul(&scales[i]);
        let tensor = scale_inertia(body.tensor, scales[i], k);
        let (position, rotation) = poses[i];
        let r = rotation.to_rotation_matrix();
        let world_com = position + rotation * local_com;
        com += mass * world_com;
        mass_kg += mass;
        world_segments.push((
            mass,
            world_com,
            r.matrix() * tensor * r.matrix().transpose(),
        ));
        segments.push(Segment {
            name: body.name.clone(),
            label: body.label,
            mass_kg: mass,
            com_m: local_com.into(),
            inertia_kg_m2: tensor.into(),
            source: Source {
                reference: SOURCE,
                locator: body.label,
            },
        });
    }
    com /= mass_kg;
    let tensor = world_segments
        .into_iter()
        .fold(Matrix3::zeros(), |acc, (mass, point, tensor)| {
            let r = point - com;
            acc + tensor + mass * (Matrix3::identity() * r.norm_squared() - r * r.transpose())
        });
    if !mass_kg.is_finite() || !com.iter().chain(tensor.iter()).all(|x| x.is_finite()) {
        return Err("масса и тензор участника вышли за численный диапазон".into());
    }
    Ok(MassProperties {
        mass_kg,
        com_m: com.into(),
        inertia_kg_m2: tensor.into(),
        segments,
    })
}

/// Строит локальный MJCF без изменения исходных ресурсов или мира.
/// Все внешние ссылки на ресурсы абсолютные и перечислены в паспорте.
pub fn build(spec: &ParticipantSpec) -> Result<ParticipantModel, String> {
    if !spec.stature_m.is_finite()
        || spec.stature_m <= 0.0
        || !spec.mass_kg.is_finite()
        || spec.mass_kg <= 0.0
    {
        return Err("рост и масса должны быть конечными положительными числами".into());
    }
    let rows: Vec<_> = surface_rows().collect();
    let asset_root = match spec.base {
        BaseVariant::MsHuman700 | BaseVariant::MsHuman700Free => models::ms_human_700()
            .parent()
            .ok_or("не найден каталог исходной модели")?
            .canonicalize()
            .map_err(|e| e.to_string())?,
    };
    let source_xml = BASE.replace(
        "@assets@",
        &xml_escape(asset_root.to_str().ok_or("путь к модели вне UTF-8")?),
    );
    let original = Sim::from_xml(&source_xml)?;
    let parsed = Spec::parse(&source_xml)?;
    let labels: BTreeMap<_, _> = NAMES
        .lines()
        .map(fields::<3>)
        .map(|[_, local, source]| (local, source))
        .collect();
    let bodies = body_data(&original, &labels);
    let proxies = read_proxies(&original, &rows)?;
    let reference_mass = original.total_mass();
    let k = spec.mass_kg / reference_mass;
    let baseline = vec![V::repeat(1.0); bodies.len()];
    let [lo, hi] = vertical_bounds(&proxies, &neutral_poses(&bodies, &baseline, 0.0), &baseline);
    let reference_stature = hi - lo;

    let overrides = local_scales(spec, &proxies)?;
    let (s, scales, root_z, height) = fit_height(
        spec.stature_m,
        reference_stature,
        &bodies,
        &proxies,
        &overrides,
    )?;
    let poses = neutral_poses(&bodies, &scales, root_z);
    let mass = model_inertia(&bodies, &poses, &scales, k)?;
    let scale_model = ModelScale {
        bodies: &bodies,
        original: &original,
        scales: &scales,
        s,
        k,
        root_z,
    };
    let (axes, couplings) = scale_model.apply(&parsed, &labels)?;
    let root_joint = if spec.base == BaseVariant::MsHuman700Free {
        add_free_joint(&parsed)?;
        Some(ROOT_JOINT.into())
    } else {
        None
    };
    let ports = scale_ports(&parsed, &proxies, &scales)?;
    let regions = proxies.iter().map(|proxy| {
        let p = proxy.scaled(scales[proxy.body]);
        let r = p.row;
        let shape = if p.kind == ffi::mjtGeom::mjGEOM_CAPSULE {
            SurfaceShape::Capsule { radius_m: p.size.x, half_segment_m: p.size.y }
        } else {
            SurfaceShape::Ellipsoid { semi_axes_m: p.size.into() }
        };
        let surface = Surface {
            geometry: r.geometry.into(), body: r.body.into(), label: r.label,
            side: match r.side { "l" => Side::Left, "r" => Side::Right, _ => Side::Midline },
            digit: r.part.strip_prefix("digit").and_then(|digit| digit.parse().ok()),
            center_m: p.center.into(), orientation: xyzw(p.rotation), shape,
            material: MATERIAL, contype: 0, conaffinity: 1,
            source: Source { reference: SOURCE, locator: r.locator },
            status: LimitStatus::Orientation,
            applicability: "Отдельный внешний контакт на прокси. Посадка и параметры тканей требуют замера.",
            effective_area_m2: None,
        };
        (RegionId(r.id.into()), surface)
    }).collect();
    let excluded_body_pairs = parsed
        .elements(ffi::mjtObj::mjOBJ_EXCLUDE)
        .into_iter()
        .map(|element| unsafe {
            let exclude = &*ffi::mjs_asExclude(element);
            [string(exclude.bodyname1), string(exclude.bodyname2)]
        })
        .collect();
    let table = marginals(spec.population);
    let anthropometry = Anthropometry {
        population: spec.population,
        stature_m: spec.stature_m,
        neutral_proxy_height_m: height,
        mass_kg: spec.mass_kg,
        reference_stature_m: reference_stature,
        reference_mass_kg: reference_mass,
        uniform_geometric_scale: overrides.is_empty().then_some(s),
        mass_scale: k,
        stature_in_marginal_range: (table.stature_m[0]..=table.stature_m[2])
            .contains(&spec.stature_m),
        mass_in_marginal_range: (table.mass_kg[0]..=table.mass_kg[2]).contains(&spec.mass_kg),
        joint_distribution_known: false,
        measured_proportions: spec.proportions.clone(),
        strength_coverage: None,
        soft_tissue_coverage: None,
        limitations: [
            "Перцентили — отдельные маргинальные диапазоны для взрослых США 20+, без совместных весов.",
            "Сочетаемость пропорций и положение вне границ перцентилей не подтверждены выборкой.",
            "Разброс силы и податливости не выводится из массы, роста или габаритов.",
        ],
    };
    let mjcf = parsed.xml()?;
    let verification = Sim::from_xml(&mjcf)?;
    if !verification.numerically_ok() {
        return Err("MuJoCo сообщил о численной ошибке участника в нейтральной позе".into());
    }
    let exported_bodies = body_data(&verification, &labels);
    let exported = vec![V::repeat(1.0); exported_bodies.len()];
    let exported_mass = model_inertia(
        &exported_bodies,
        &neutral_poses(&exported_bodies, &exported, 0.0),
        &exported,
        1.0,
    )?;
    check_mass_export(&mass, &exported_mass)?;
    Ok(ParticipantModel {
        mjcf,
        source: asset_root.join("MS-Human-700.xml"),
        resources: RESOURCES
            .lines()
            .map(|path| asset_root.join(path))
            .collect::<Vec<PathBuf>>(),
        root_body: ROOT.into(),
        frame: "Z вверх, X вперёд, Y влево; Z=0 у нижнего края прокси",
        mass,
        exported_mass,
        root_joint,
        axes,
        couplings,
        ports,
        regions,
        excluded_body_pairs,
        anthropometry,
        limitations: vec![
            "Ограничения модели относятся к отдельным координатам и сопряжениям. Достижимость совместных поз требует проверки.",
            "При свободном основании опоры и усилия задаются в общем мире.",
            "Прокси основаны на исходных капсулах и габаритах костных сеток. Промежутки, лицо и мягкие ткани требуют измерений.",
            "Маски выключают самоконтакт. Покрытие внешних пар и достижимых поз проверяется в собранном мире.",
            "Площади пятен, распределённое давление, внутренние реакции и параметры кожи требуют независимой калибровки.",
            "Сухожилия описывают пути без мышечных приводов. При непропорциональных размерах обёртки аппроксимируются.",
        ],
    })
}

fn add_free_joint(spec: &Spec) -> Result<(), String> {
    let root = CString::new(ROOT).map_err(|err| err.to_string())?;
    let joint_name = CString::new(ROOT_JOINT).map_err(|err| err.to_string())?;
    // Тело и добавленный сустав принадлежат активной спецификации.
    unsafe {
        let body = ffi::mjs_findBody(spec.0.as_ptr(), root.as_ptr());
        if body.is_null() {
            return Err("не найдено основание участника".into());
        }
        let joint = ffi::mjs_addFreeJoint(body);
        if joint.is_null() || ffi::mjs_setName((*joint).element, joint_name.as_ptr()) != 0 {
            return Err("не удалось добавить пространственный сустав".into());
        }
    }
    Ok(())
}

fn check_mass_export(analytic: &MassProperties, exported: &MassProperties) -> Result<(), String> {
    if analytic.segments.len() != exported.segments.len() {
        return Err("при записи изменилось число сегментов".into());
    }
    for (before, after) in analytic.segments.iter().zip(&exported.segments) {
        let mass_error = (after.mass_kg - before.mass_kg).abs();
        let inertia = Matrix3::from(before.inertia_kg_m2);
        let inertia_error = (Matrix3::from(after.inertia_kg_m2) - inertia).norm();
        let center = V::from(before.com_m);
        let center_error = (V::from(after.com_m) - center).norm();
        if before.name != after.name
            || mass_error > XML_REL_ERROR * before.mass_kg
            || inertia_error > XML_REL_ERROR * inertia.norm()
            || (before.mass_kg > 0.0 && center_error > XML_REL_ERROR * center.norm())
        {
            return Err(format!(
                "при записи искажены инерционные параметры {}",
                before.name
            ));
        }
    }
    Ok(())
}

fn read_proxies<'a>(sim: &Sim, rows: &'a [SurfaceRow]) -> Result<Vec<Proxy<'a>>, String> {
    let m = sim.model();
    let n = sim.ngeom();
    let kind = ffi_array(sim, m.geom_type, n);
    let pos = ffi_array(sim, m.geom_pos.cast::<[f64; 3]>(), n);
    let rotation = ffi_array(sim, m.geom_quat.cast::<[f64; 4]>(), n);
    let size = ffi_array(sim, m.geom_size.cast::<[f64; 3]>(), n);
    let owners = ffi_array(sim, m.geom_bodyid, n);
    rows.iter()
        .map(|row| {
            let geom = sim
                .id(GEOM, row.geometry)
                .ok_or_else(|| format!("нет геометрии {}", row.geometry))?;
            let body = owners[geom] as usize;
            if sim.name(BODY, body) != row.body {
                return Err(format!(
                    "ошибка связи геометрии {} с телом {}",
                    row.geometry, row.body
                ));
            }
            Ok(Proxy {
                row,
                body,
                center: pos[geom].into(),
                rotation: quat(rotation[geom]),
                size: size[geom].into(),
                kind: kind[geom] as ffi::mjtGeom::Type,
            })
        })
        .collect()
}

fn local_scales(
    spec: &ParticipantSpec,
    proxies: &[Proxy<'_>],
) -> Result<BTreeMap<usize, V>, String> {
    let mut out = BTreeMap::new();
    for correction in &spec.proportions {
        let p = proxies
            .iter()
            .find(|p| p.row.id == correction.region.0)
            .ok_or_else(|| format!("участок {} отсутствует", correction.region.0))?;
        if correction.source.trim().is_empty()
            || p.kind != ffi::mjtGeom::mjGEOM_ELLIPSOID
            || p.rotation != Q::identity()
            || out.contains_key(&p.body)
        {
            return Err(format!("неприменимое измерение для участка {}", p.row.id));
        }
        let factors = match correction.measure {
            ProportionMeasure::LocalDimensions(size) => {
                V::from(size).component_div(&(p.size * 2.0))
            }
            ProportionMeasure::LengthAndCircumference {
                length_m,
                circumference_m,
            } => {
                let radius = circumference_m / ellipse_perimeter(p.size.x, p.size.z);
                V::new(radius, length_m / (2.0 * p.size.y), radius)
            }
        };
        if !factors.iter().all(|v| v.is_finite() && *v > 0.0) {
            return Err(format!("некорректные размеры участка {}", p.row.id));
        }
        out.insert(p.body, factors);
    }
    Ok(out)
}

fn ellipse_perimeter(a: f64, b: f64) -> f64 {
    // Правило Симпсона для интеграла на четверти эллипса. 256 отрезков
    // сдваивают сетку до погрешности менее 10⁻⁸ м на используемых пропорциях.
    const N: usize = 256;
    let step = std::f64::consts::FRAC_PI_2 / N as f64;
    let speed = |i| {
        let (sin, cos) = (step * i as f64).sin_cos();
        (a * sin).hypot(b * cos)
    };
    let interior = (1..N)
        .map(|i| if i % 2 == 0 { 2.0 } else { 4.0 } * speed(i))
        .sum::<f64>();
    4.0 * step / 3.0 * (speed(0) + interior + speed(N))
}

fn fit_height(
    target: f64,
    reference: f64,
    bodies: &[RestBody],
    proxies: &[Proxy<'_>],
    overrides: &BTreeMap<usize, V>,
) -> Result<(f64, Vec<V>, f64, f64), String> {
    let get = |s| {
        let scales: Vec<_> = (0..bodies.len())
            .map(|i| overrides.get(&i).copied().unwrap_or(V::repeat(s)))
            .collect();
        let [lo, hi] = vertical_bounds(proxies, &neutral_poses(bodies, &scales, 0.0), &scales);
        (scales, lo, hi - lo)
    };
    let nominal = target / reference;
    let mut s = nominal;
    if !overrides.is_empty() {
        // В нейтральной позе высота — разность крайних опорных функций. По
        // остальному масштабу это выпуклая кусочно-линейная функция.
        let mut lo = 0.0;
        let mut hi = nominal;
        if get(lo).2 >= target {
            return Err("измеренные размеры уже превышают требуемый рост".into());
        }
        for _ in 0..f64::MANTISSA_DIGITS {
            if get(hi).2 >= target {
                break;
            }
            hi *= 2.0;
        }
        for _ in 0..f64::MANTISSA_DIGITS {
            s = lo + (hi - lo) / 2.0;
            if get(s).2 < target {
                lo = s;
            } else {
                hi = s;
            }
        }
    }
    let (scales, floor_z, height) = get(s);
    // Допуск — округления операций по дереву, не антропометрическая погрешность.
    let tolerance = f64::EPSILON * bodies.len() as f64 * target;
    if !s.is_finite() || s <= 0.0 || !height.is_finite() || (height - target).abs() > tolerance {
        return Err("требуемый рост не согласуется с измеренными размерами".into());
    }
    Ok((s, scales, -floor_z, height))
}

struct ModelScale<'a> {
    bodies: &'a [RestBody],
    original: &'a Sim,
    scales: &'a [V],
    s: f64,
    k: f64,
    root_z: f64,
}

impl ModelScale<'_> {
    fn owner(&self, element: *mut ffi::mjsElement) -> Result<usize, String> {
        // Элементы в дереве спецификации имеют живого родителя.
        let parent = unsafe { ffi::mjs_getParent(element) };
        let body = NonNull::new(parent).ok_or("объект без тела")?;
        let body_name = name(unsafe { body.as_ref() }.element);
        self.original
            .id(BODY, &body_name)
            .ok_or_else(|| format!("нет тела {body_name}"))
    }

    fn apply(
        &self,
        spec: &Spec,
        labels: &BTreeMap<&str, &'static str>,
    ) -> Result<(Vec<Axis>, Vec<AxisCoupling>), String> {
        // В исходном варианте boundmass применяется при выводе инерции из геометрии.
        // При явных масштабированных инерциях повторная отсечка исказила бы массу.
        unsafe { (*spec.0.as_ptr()).compiler.boundmass = 0.0 };
        for element in spec.elements(BODY) {
            let name = name(element);
            let Some(i) = self.original.id(BODY, &name) else {
                // Мировая рамка не является частью участника.
                continue;
            };
            if i == 0 {
                continue;
            }
            let body = &self.bodies[i];
            let target = unsafe { &mut *ffi::mjs_asBody(element) };
            let position = body.position.component_mul(&self.scales[body.parent]);
            target.pos = (position
                + if body.parent == 0 {
                    V::z() * self.root_z
                } else {
                    V::zeros()
                })
            .into();
            if body.mass > 0.0 {
                target.mass = self.k * body.mass;
                target.ipos = body.com.component_mul(&self.scales[i]).into();
                let inertia = scale_inertia(body.tensor, self.scales[i], self.k);
                let mut eigen = inertia.symmetric_eigen();
                if eigen.eigenvectors.determinant() < 0.0 {
                    eigen.eigenvectors.column_mut(0).neg_mut();
                }
                if !eigen.eigenvalues.iter().all(|v| v.is_finite() && *v > 0.0) {
                    return Err(format!("некорректный тензор сегмента {name}"));
                }
                target.iquat = xyzw(Q::from_rotation_matrix(&Rotation3::from_matrix_unchecked(
                    eigen.eigenvectors,
                )));
                target.inertia = eigen.eigenvalues.into();
                target.explicitinertial = true;
            }
        }

        let mut axes = Vec::new();
        let mut factors = BTreeMap::new();
        for element in spec.elements(DOF) {
            let joint = name(element);
            let owner = self.owner(element)?;
            let scale = self.scales[owner];
            let target = unsafe { &mut *ffi::mjs_asJoint(element) };
            target.pos = V::from(target.pos).component_mul(&scale).into();
            let (kind, coordinate, inertia) = match target.type_ {
                ffi::mjtJoint::mjJNT_HINGE => {
                    let direction = V::from(target.axis).normalize();
                    // Проекция S² на плоскость, перпендикулярную оси. Численные
                    // коэффициенты упругости и гашения остаются модельными.
                    let variance =
                        scale.norm_squared() - direction.component_mul(&scale).norm_squared();
                    (AxisKind::Rotation, 1.0, self.k * variance / 2.0)
                }
                ffi::mjtJoint::mjJNT_SLIDE => {
                    let direction = V::from(target.axis).normalize().component_mul(&scale);
                    let ratio = direction.norm();
                    target.axis = (direction / ratio).into();
                    (AxisKind::Translation, ratio, self.k)
                }
                _ => return Err(format!("неподдерживаемая координата {joint}")),
            };
            target.range = target.range.map(|v| v * coordinate);
            target.ref_ *= coordinate;
            target.springref *= coordinate;
            target.margin *= coordinate;
            target.armature *= inertia;
            target.damping = target.damping.map(|v| v * inertia);
            for (power, v) in target.stiffness.iter_mut().enumerate() {
                *v *= inertia / coordinate.powi(power as i32);
            }
            factors.insert(joint.clone(), coordinate);
            let direction = V::from(target.axis).normalize();
            axes.push(Axis {
                label: labels[joint.as_str()],
                joint,
                body: self.bodies[owner].name.clone(),
                kind,
                range: target.range,
                direction: direction.into(),
                anchor_m: target.pos,
                neutral: target.ref_,
                dependent: false,
                source: Source {
                    reference: SOURCE,
                    locator: "суставы Body_Primary и Body_Manipulation",
                },
            });
        }

        let mut coupled = BTreeSet::new();
        let mut couplings = Vec::new();
        for element in spec.elements(EQ) {
            let target = unsafe { &mut *ffi::mjs_asEquality(element) };
            if target.type_ != ffi::mjtEq::mjEQ_JOINT {
                return Err("неподдерживаемый закон зависимой оси".into());
            }
            let left = unsafe { string(target.name1) };
            let right = unsafe { string(target.name2) };
            if !coupled.insert(left.clone()) {
                return Err(format!("несколько зависимостей для оси {left}"));
            }
            let a = factors[&left];
            let b = factors[&right];
            // Источник использует полином четвёртой степени от q2 − ref2.
            for (power, v) in target.data[..5].iter_mut().enumerate() {
                *v *= a / b.powi(power as i32);
            }
            couplings.push(AxisCoupling {
                dependent: left,
                driver: right,
                coefficients: std::array::from_fn(|index| target.data[index]),
            });
        }
        for axis in &mut axes {
            axis.dependent = coupled.contains(&axis.joint);
        }
        self.scale_geometries(spec)?;
        for element in spec.elements(SITE) {
            let scale = self.scales[self.owner(element)?];
            let target = unsafe { &mut *ffi::mjs_asSite(element) };
            target.pos = V::from(target.pos).component_mul(&scale).into();
            // Служебный размер точки относится только к показу.
            target.size = target.size.map(|v| v * self.s);
        }
        for element in spec.elements(ffi::mjtObj::mjOBJ_TENDON) {
            let target = unsafe { &mut *ffi::mjs_asTendon(element) };
            target.width *= self.s;
        }
        Ok((axes, couplings))
    }

    fn scale_geometries(&self, spec: &Spec) -> Result<(), String> {
        let meshes: BTreeMap<_, _> = spec
            .elements(MESH)
            .into_iter()
            .map(|mesh| (name(mesh), mesh))
            .collect();
        for element in spec.elements(GEOM) {
            let scale = self.scales[self.owner(element)?];
            let target = unsafe { &mut *ffi::mjs_asGeom(element) };
            let uniform = scale.x == scale.y && scale.y == scale.z;
            target.pos = V::from(target.pos).component_mul(&scale).into();
            target.margin *= self.s;
            target.gap *= self.s;
            target.mass = 0.0;
            match target.type_ {
                ffi::mjtGeom::mjGEOM_MESH => {
                    let mesh_name = unsafe { string(target.meshname) };
                    let mesh = unsafe { &mut *ffi::mjs_asMesh(meshes[&mesh_name]) };
                    if uniform || quat(target.quat) == Q::identity() {
                        mesh.scale = V::from(mesh.scale).component_mul(&scale).into();
                    } else {
                        self.inline_transformed_mesh(&name(element), target, mesh, scale)?;
                    }
                }
                ffi::mjtGeom::mjGEOM_ELLIPSOID => {
                    target.size = V::from(target.size).component_mul(&scale).into();
                }
                ffi::mjtGeom::mjGEOM_CAPSULE | ffi::mjtGeom::mjGEOM_CYLINDER => {
                    let rotation = quat(target.quat);
                    let axial = (rotation * V::z()).component_mul(&scale);
                    let length_factor = axial.norm();
                    // Обёртка круглого сечения: сохраняем площадь сечения после
                    // аппроксимации. Для внешних капсул здесь масштаб подобия.
                    let radius_factor = (scale.x * scale.y * scale.z / length_factor).sqrt();
                    target.size[0] *= radius_factor;
                    target.size[1] *= length_factor;
                    if !uniform {
                        let aligned = Q::rotation_between(&(rotation * V::z()), &axial)
                            .ok_or("не удалось совместить ось обёртки")?;
                        target.quat = xyzw(aligned * rotation);
                    }
                }
                ffi::mjtGeom::mjGEOM_SPHERE => {
                    target.size[0] *= (scale.x * scale.y * scale.z).cbrt();
                }
                _ => return Err(format!("неподдерживаемая геометрия {}", name(element))),
            }
        }
        Ok(())
    }

    fn inline_transformed_mesh(
        &self,
        geom_name: &str,
        target: &mut ffi::mjsGeom,
        mesh: &mut ffi::mjsMesh,
        scale: V,
    ) -> Result<(), String> {
        let original = self.original;
        let m = original.model();
        let geom = original
            .id(GEOM, geom_name)
            .ok_or("нет исходной геометрии сетки")?;
        let mesh_id = ffi_array(original, m.geom_dataid, original.ngeom())[geom] as usize;
        let data = original.mesh(mesh_id);
        let origin =
            V::from(ffi_array(original, m.geom_pos.cast::<[f64; 3]>(), original.ngeom())[geom]);
        let rotation =
            quat(ffi_array(original, m.geom_quat.cast::<[f64; 4]>(), original.ngeom())[geom]);
        let vertices: Vec<f32> = data
            .positions
            .into_iter()
            .flat_map(|xyz| {
                let point = (origin + rotation * V::from(xyz.map(f64::from))).component_mul(&scale);
                <[f64; 3]>::from(point).map(|part| part as f32)
            })
            .collect();
        let faces: Vec<i32> = data.indices.into_iter().map(|i| i as i32).collect();
        if !vertices.iter().all(|v| v.is_finite()) {
            return Err(format!("сетка {geom_name} вышла за численный диапазон"));
        }
        // Вершины уже перенесены в рамку тела. Индексы сохранены, S имеет положительный det.
        target.pos = [0.0; 3];
        target.quat = [1.0, 0.0, 0.0, 0.0];
        mesh.refpos = [0.0; 3];
        mesh.refquat = [1.0, 0.0, 0.0, 0.0];
        mesh.scale = [1.0; 3];
        unsafe {
            ffi::mjs_setString(mesh.file, c"".as_ptr());
            ffi::mjs_setString(mesh.content_type, c"".as_ptr());
            ffi::mjs_setFloat(mesh.uservert, vertices.as_ptr(), vertices.len() as i32);
            ffi::mjs_setInt(mesh.userface, faces.as_ptr(), faces.len() as i32);
        }
        Ok(())
    }
}

fn scale_ports(spec: &Spec, proxies: &[Proxy<'_>], scales: &[V]) -> Result<Vec<Port>, String> {
    let sites: BTreeMap<_, _> = spec
        .elements(SITE)
        .into_iter()
        .map(|site| (name(site), site))
        .collect();
    PORTS
        .lines()
        .map(fields::<3>)
        .map(|[site, region, direction]| {
            let proxy = proxies
                .iter()
                .find(|p| p.row.id == region)
                .ok_or("нет участка точки крепления")?;
            let scale = scales[proxy.body];
            let target = unsafe { &mut *ffi::mjs_asSite(sites[site]) };
            let local = V::from(target.pos).component_div(&scale);
            let (point, orientation) = proxy.port(local, scale);
            target.pos = point.into();
            target.quat = xyzw(orientation);
            Ok(Port {
                site: site.into(),
                body: proxy.row.body.into(),
                region: RegionId(region.into()),
                point_m: target.pos,
                orientation: target.quat,
                direction,
            })
        })
        .collect()
}
