use std::f64::consts::PI;
use std::path::Path;

use nalgebra::{Isometry3, Matrix3, Quaternion, Translation3, Unit, UnitQuaternion, Vector3};

use crate::mesh::{Mesh, load_mesh};
use crate::{Geometry, Mass, Material, Pose, Provenance, Shape};

pub(crate) type Vec3 = Vector3<f64>;
pub(crate) type Frame = Isometry3<f64>;

pub(crate) fn finite(values: &[f64], field: &str) -> Result<(), String> {
    if values.iter().all(|x| x.is_finite()) {
        Ok(())
    } else {
        Err(format!("{field}: допустимы только конечные числа"))
    }
}

pub(crate) fn positive(values: &[f64], field: &str) -> Result<(), String> {
    finite(values, field)?;
    if values.iter().all(|x| *x > 0.0) {
        Ok(())
    } else {
        Err(format!("{field}: нужно положительное значение"))
    }
}

pub(crate) fn nonnegative(values: &[f64], field: &str) -> Result<(), String> {
    finite(values, field)?;
    if values.iter().all(|x| *x >= 0.0) {
        Ok(())
    } else {
        Err(format!("{field}: нужно неотрицательное значение"))
    }
}

pub(crate) fn axis(a: [f64; 3]) -> Result<Unit<Vec3>, String> {
    finite(&a, "axis")?;
    let scale = a.iter().map(|x| x.abs()).fold(0.0, f64::max);
    positive(&[scale], "axis")?;
    Ok(Unit::new_normalize(Vec3::from(a) / scale))
}

pub(crate) fn frame(p: &Pose) -> Result<Frame, String> {
    finite(&p.position_m, "position_m")?;
    let rotation = match (p.quat_wxyz, p.rpy_deg) {
        (Some(_), Some(_)) => return Err("quat_wxyz и rpy_deg заданы одновременно".into()),
        (Some(q), None) => {
            finite(&q, "quat_wxyz")?;
            let scale = q.iter().map(|x| x.abs()).fold(0.0, f64::max);
            positive(&[scale], "quat_wxyz")?;
            let [w, x, y, z] = q.map(|x| x / scale);
            UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z))
        }
        (None, Some(r)) => {
            finite(&r, "rpy_deg")?;
            let [x, y, z] = r.map(f64::to_radians);
            UnitQuaternion::from_euler_angles(x, y, z)
        }
        (None, None) => UnitQuaternion::identity(),
    };
    Ok(Frame::from_parts(
        Translation3::from(p.position_m),
        rotation,
    ))
}

pub(crate) fn quat(rotation: &UnitQuaternion<f64>) -> [f64; 4] {
    [rotation.w, rotation.i, rotation.j, rotation.k]
}

pub(crate) fn pose(transform: &Frame) -> Pose {
    Pose {
        position_m: transform.translation.vector.into(),
        quat_wxyz: Some(quat(&transform.rotation)),
        rpy_deg: None,
    }
}

pub(crate) fn tensor([a, b, c, d, e, f]: [f64; 6]) -> Matrix3<f64> {
    Matrix3::new(a, d, e, d, b, f, e, f, c)
}

pub(crate) fn tensor_values(i: &Matrix3<f64>) -> [f64; 6] {
    [
        i[(0, 0)],
        i[(1, 1)],
        i[(2, 2)],
        i[(0, 1)],
        i[(0, 2)],
        i[(1, 2)],
    ]
}

#[derive(Clone, Debug)]
pub(crate) struct Inertial {
    pub kg: f64,
    pub com: Vec3,
    pub tensor: Matrix3<f64>,
}

pub(crate) struct Geom<'a> {
    pub data: &'a Geometry,
    pub pose: Frame,
    pub mesh: Option<Mesh>,
}

impl<'a> Geom<'a> {
    pub fn new(data: &'a Geometry, base: &Path) -> Result<Self, String> {
        let pose = frame(&data.pose).map_err(|e| format!("pose.{e}"))?;
        if let Some(m) = &data.explicit_mass {
            nonnegative(&[m.mass_kg], "explicit_mass.mass_kg")?;
            provenance(&m.provenance).map_err(|e| format!("explicit_mass.{e}"))?;
        }
        if data
            .contact_dim
            .is_some_and(|n| !matches!(n, 1 | 3 | 4 | 6))
        {
            return Err("contact_dim: MuJoCo поддерживает 1, 3, 4 и 6".into());
        }
        if data
            .contact_mask
            .is_some_and(|m| m.iter().any(|&n| n > i32::MAX as u32))
        {
            return Err("contact_mask: маска вне диапазона MuJoCo".into());
        }
        if let Some(f) = data.friction {
            nonnegative(&f, "friction")?;
        }
        if let Some(rgba) = data.rgba {
            nonnegative(&rgba, "rgba")?;
            if rgba.iter().any(|x| *x > 1.0) {
                return Err("rgba: компоненты должны быть долями".into());
            }
        }
        if let Some(m) = &data.material {
            material(m).map_err(|e| format!("material.{e}"))?;
        }
        let mesh = match &data.shape {
            Shape::Box { size_m } => {
                positive(size_m, "shape.size_m")?;
                None
            }
            Shape::Sphere { radius_m } => {
                positive(&[*radius_m], "shape.radius_m")?;
                None
            }
            Shape::Capsule { radius_m, length_m } | Shape::Cylinder { radius_m, length_m } => {
                positive(&[*radius_m], "shape.radius_m")?;
                positive(&[*length_m], "shape.length_m")?;
                None
            }
            Shape::Ellipsoid { radii_m } => {
                positive(radii_m, "shape.radii_m")?;
                None
            }
            Shape::Mesh { file, scale } => {
                Some(load_mesh(&base.join(file), *scale).map_err(|e| format!("shape.file: {e}"))?)
            }
            Shape::Triangles { vertices_m, faces } => {
                let mesh = Mesh {
                    vertices_m: vertices_m.clone(),
                    faces: faces.clone(),
                };
                mesh.validate().map_err(|e| format!("shape.{e}"))?;
                Some(mesh)
            }
        };
        Ok(Self { data, pose, mesh })
    }

    fn inertia(&self, density: f64) -> Result<Inertial, String> {
        let (kg, local) = match self.data.shape {
            // Интегралы вторых моментов плотности по однородному объёму.
            Shape::Box { size_m: [x, y, z] } => {
                let kg = density * x * y * z;
                (
                    kg,
                    [y * y + z * z, x * x + z * z, x * x + y * y].map(|v| kg * v / 12.0),
                )
            }
            Shape::Sphere { radius_m: r } => {
                let kg = density * (4.0 / 3.0) * PI * r.powi(3);
                (kg, [kg * (2.0 / 5.0) * r * r; 3])
            }
            Shape::Ellipsoid { radii_m: [x, y, z] } => {
                let kg = density * (4.0 / 3.0) * PI * x * y * z;
                (
                    kg,
                    [y * y + z * z, x * x + z * z, x * x + y * y].map(|v| kg * v / 5.0),
                )
            }
            Shape::Cylinder {
                radius_m: r,
                length_m: l,
            }
            | Shape::Capsule {
                radius_m: r,
                length_m: l,
            } => {
                let kg = density * PI * r * r * l;
                let axial = kg * r * r / 2.0;
                let transverse = kg * (r * r / 4.0 + l * l / 12.0);
                if matches!(self.data.shape, Shape::Capsule { .. }) {
                    // Две полусферы со средним |z|=3r/8, сдвинутые от центра на ±l/2.
                    let sphere = density * (4.0 / 3.0) * PI * r.powi(3);
                    let sphere_axis = sphere * (2.0 / 5.0) * r * r;
                    let sphere_transverse =
                        sphere_axis + sphere * (l * l / 4.0 + 3.0 * r * l / 8.0);
                    (
                        kg + sphere,
                        [
                            transverse + sphere_transverse,
                            transverse + sphere_transverse,
                            axial + sphere_axis,
                        ],
                    )
                } else {
                    (kg, [transverse, transverse, axial])
                }
            }
            Shape::Mesh { .. } | Shape::Triangles { .. } => {
                let mut i = mesh_inertia(self.mesh.as_ref().expect("сетка загружена"), density)?;
                self.transform_inertia(&mut i);
                return Ok(i);
            }
        };
        let mut i = Inertial {
            kg,
            com: Vec3::zeros(),
            tensor: Matrix3::from_diagonal(&local.into()),
        };
        self.transform_inertia(&mut i);
        Ok(i)
    }

    fn transform_inertia(&self, i: &mut Inertial) {
        let r = self.pose.rotation.to_rotation_matrix();
        i.com = self.pose.translation.vector + r * i.com;
        i.tensor = r.matrix() * i.tensor * r.matrix().transpose();
    }
}

pub(crate) fn mesh_inertia(mesh: &Mesh, density: f64) -> Result<Inertial, String> {
    mesh.require_closed()?;
    // Интегрирование по ориентированным тетраэдрам (0,a,b,c). Перенос начала
    // к вершине уменьшает потерю точности на многоугольниках со смещёнными координатами.
    let origin = Vec3::from(mesh.vertices_m[0]);
    let points: Vec<_> = mesh
        .vertices_m
        .iter()
        .map(|p| Vec3::from(*p) - origin)
        .collect();
    let mut volume = 0.0;
    let mut first = Vec3::zeros();
    let mut second = Matrix3::zeros();
    for f in &mesh.faces {
        let [a, b, c] = f.map(|j| points[j]);
        let v = a.dot(&b.cross(&c)) / 6.0;
        let sum = a + b + c;
        volume += v;
        first += (v / 4.0) * sum;
        second += (v / 20.0)
            * (sum * sum.transpose() + a * a.transpose() + b * b.transpose() + c * c.transpose());
    }
    positive(&[volume], "ориентированный объём сетки")?;
    let com = first / volume;
    let central = second - volume * com * com.transpose();
    let tensor = density * (central.trace() * Matrix3::identity() - central);
    Ok(Inertial {
        kg: density * volume,
        com: com + origin,
        tensor,
    })
}

fn source_text(source: &Provenance) -> &str {
    match source {
        Provenance::Datasheet(s)
        | Provenance::Paper(s)
        | Provenance::Measured(s)
        | Provenance::Derived(s)
        | Provenance::Estimate(s) => s,
    }
}

fn provenance(source: &Provenance) -> Result<(), String> {
    if source_text(source).trim().is_empty() {
        Err("provenance: требуется непустой источник".into())
    } else {
        Ok(())
    }
}

fn material(m: &Material) -> Result<(), String> {
    if m.name.trim().is_empty() {
        return Err("name: требуется название материала".into());
    }
    positive(&[m.density_kg_m3], "density_kg_m3")?;
    provenance(&m.provenance)
}

pub(crate) fn mass(
    m: &Mass,
    geoms: &[Geom<'_>],
) -> Result<(Inertial, [f64; 3], Provenance), String> {
    let (mut i, source) = match m {
        Mass::Explicit {
            mass_kg,
            com_m,
            inertia_kg_m2,
            provenance: source,
        } => {
            provenance(source)?;
            (
                Inertial {
                    kg: *mass_kg,
                    com: (*com_m).into(),
                    tensor: tensor(*inertia_kg_m2),
                },
                source.clone(),
            )
        }
        Mass::Geometry { material: default }
        | Mass::Volumes {
            material: default, ..
        } => {
            material(default).map_err(|e| format!("material.{e}"))?;
            if geoms.is_empty() {
                return Err("для вывода массы нужны collision-объёмы".into());
            }
            let mut sources = Vec::new();
            let mut proven = true;
            let parts = geoms
                .iter()
                .enumerate()
                .filter(|(_, g)| {
                    g.data
                        .explicit_mass
                        .as_ref()
                        .is_none_or(|m| m.mass_kg > 0.0)
                })
                .map(|(j, g)| {
                    let m = g.data.material.as_ref().unwrap_or(default);
                    sources.push(format!(
                        "{}: {:?} {}",
                        m.name, m.provenance, m.density_kg_m3
                    ));
                    let mut i = g
                        .inertia(m.density_kg_m3)
                        .map_err(|e| format!("объём[{j}]: {e}"))?;
                    if let Some(m) = &g.data.explicit_mass {
                        i.tensor *= m.mass_kg / i.kg;
                        i.kg = m.mass_kg;
                        proven &= m.provenance.can_prove();
                        sources.push(format!(
                            "явная масса объёма {} кг: {:?}",
                            m.mass_kg, m.provenance
                        ));
                    } else {
                        proven &= m.provenance.can_prove();
                    }
                    positive(&[i.kg], &format!("collision[{j}].mass_kg"))?;
                    Ok(i)
                })
                .collect::<Result<Vec<_>, String>>()?;
            let kg: f64 = parts.iter().map(|i| i.kg).sum();
            let com = parts
                .iter()
                .fold(Vec3::zeros(), |c, i| c + i.com * (i.kg / kg));
            // Теорема Штейнера: I(C)=I(Cᵢ)+mᵢ(|d|²E−ddᵀ).
            let tensor = parts.iter().fold(Matrix3::zeros(), |total, i| {
                let d = i.com - com;
                total
                    + i.tensor
                    + i.kg * (d.norm_squared() * Matrix3::identity() - d * d.transpose())
            });
            let desc = format!("геометрия и плотность: {}", sources.join(", "));
            (
                Inertial { kg, com, tensor },
                if proven {
                    Provenance::Derived(desc)
                } else {
                    Provenance::Estimate(desc)
                },
            )
        }
        Mass::Massless => {
            return Ok((
                Inertial {
                    kg: 0.0,
                    com: Vec3::zeros(),
                    tensor: Matrix3::zeros(),
                },
                [0.0; 3],
                Provenance::Estimate("безмассовая система координат".into()),
            ));
        }
    };
    positive(&[i.kg], "mass_kg")?;
    finite(i.com.as_slice(), "com_m")?;
    finite(i.tensor.as_slice(), "inertia_kg_m2")?;
    // Симметрия после поворотов и переносов с конечной точностью.
    i.tensor = i.tensor / 2.0 + i.tensor.transpose() / 2.0;
    let scale = i.tensor.abs().max();
    positive(&[scale], "inertia_kg_m2")?;
    let eigen = (i.tensor / scale)
        .try_symmetric_eigen(f64::EPSILON, 64)
        .ok_or("inertia_kg_m2: не удалось вычислить главные моменты")?;
    let mut principal: [f64; 3] = (eigen.eigenvalues * scale).into();
    positive(&principal, "inertia_kg_m2: главные моменты")?;
    principal.sort_by(f64::total_cmp);
    // Треугольные неравенства моментов следуют из неотрицательности вторых моментов массы.
    if principal[2] - principal[1] > principal[0] {
        return Err("inertia_kg_m2: нарушено неравенство главных моментов".into());
    }
    Ok((i, principal, source))
}
