//! Программная текстура для окна и приёмки. Здесь нет шагов физики.
use bevy::mesh::{Indices, VertexAttributeValues};
use bevy::prelude::{Capsule3d, Cuboid, Cylinder, Mesh, Meshable, Sphere};
use nalgebra::{Matrix3, Point3, Vector3};
use sim_mujoco::{GeomKind, Sim};
use sim_sense::Intrinsics;

pub type Point = Point3<f64>;
pub type Vec3 = Vector3<f64>;
pub const BACKGROUND: [u8; 3] = [18, 22, 31];
pub const WRENCH_COLOR: [u8; 3] = [104, 234, 222];
pub(super) const KINEMATICS_COLOR: [u8; 3] = [147, 183, 255];
pub const FORCE_COLOR: [u8; 3] = [255, 183, 56];
// Три строки позволяют различать полосу даже при масштабировании текстуры.
pub(super) const FORCE_ROWS: usize = 3;

pub(crate) struct Geometry {
    id: usize,
    body: usize,
    vertices: Vec<Point>,
    triangles: Vec<[u32; 3]>,
    color: [f32; 4],
}

impl Geometry {
    pub fn build(sim: &Sim) -> Result<Vec<Self>, String> {
        let geoms = sim.geoms();
        let visible: std::collections::BTreeSet<_> = geoms
            .iter()
            .filter(|g| g.rgba[3] > 0.0)
            .map(|g| g.body)
            .collect();
        geoms
            .into_iter()
            .filter(|g| {
                g.rgba[3] > 0.0 || (g.name.contains("/collision/") && !visible.contains(&g.body))
            })
            .map(|g| {
                let [x, y, z] = g.size;
                if !g.size.iter().all(|x| x.is_finite()) || !g.rgba.iter().all(|x| x.is_finite()) {
                    return Err(format!("неконечная геометрия «{}»", g.name));
                }
                let (vertices, indices) = match g.kind {
                    // Размер бесконечной плоскости подбирается при подготовке кадра.
                    GeomKind::Plane => (Vec::new(), Vec::new()),
                    GeomKind::Mesh => {
                        let mesh = sim.mesh(g.mesh.ok_or("отсутствует индекс сетки")?);
                        (mesh.positions, mesh.indices)
                    }
                    GeomKind::Sphere | GeomKind::Ellipsoid => {
                        // Число граней влияет только на качество обзорного изображения.
                        let (mut vertices, indices) = mesh(Sphere::new(1.0).mesh().uv(24, 16))?;
                        let scale = if g.kind == GeomKind::Sphere {
                            [x; 3]
                        } else {
                            [x, y, z]
                        };
                        for p in &mut vertices {
                            for (p, scale) in p.iter_mut().zip(scale) {
                                *p *= scale as f32;
                            }
                        }
                        (vertices, indices)
                    }
                    GeomKind::Box => mesh(
                        Cuboid::new((2.0 * x) as f32, (2.0 * y) as f32, (2.0 * z) as f32).into(),
                    )?,
                    GeomKind::Capsule | GeomKind::Cylinder => {
                        let primitive = if g.kind == GeomKind::Capsule {
                            Capsule3d::new(x as f32, (2.0 * y) as f32).into()
                        } else {
                            Cylinder::new(x as f32, (2.0 * y) as f32).into()
                        };
                        let (mut vertices, indices) = mesh(primitive)?;
                        // Поворот оси примитива Bevy из Y в Z MuJoCo.
                        for p in &mut vertices {
                            *p = [p[0], -p[2], p[1]];
                        }
                        (vertices, indices)
                    }
                    GeomKind::Other => {
                        return Err(format!("неподдерживаемая видимая геометрия «{}»", g.name));
                    }
                };
                let (triangles, tail) = indices.as_chunks::<3>();
                if !tail.is_empty()
                    || indices.iter().any(|&i| i as usize >= vertices.len())
                    || !vertices.iter().flatten().all(|v| v.is_finite())
                {
                    return Err(format!("неверная сетка «{}»", g.name));
                }
                Ok(Self {
                    id: g.id,
                    body: g.body,
                    vertices: vertices
                        .into_iter()
                        .map(|p| Point::from(p.map(f64::from)))
                        .collect(),
                    triangles: triangles.to_vec(),
                    // Цвет обозначает геометрию столкновений в отсутствие отдельной сетки показа.
                    color: if g.rgba[3] > 0.0 {
                        g.rgba
                    } else {
                        [0.27, 0.67, 0.77, 1.0]
                    },
                })
            })
            .collect()
    }
}

fn mesh(mesh: Mesh) -> Result<(Vec<[f32; 3]>, Vec<u32>), String> {
    let Some(VertexAttributeValues::Float32x3(vertices)) = mesh.attribute(Mesh::ATTRIBUTE_POSITION)
    else {
        return Err("у примитива нет координат вершин".into());
    };
    let indices = match mesh.indices() {
        Some(Indices::U32(indices)) => indices.clone(),
        Some(Indices::U16(indices)) => indices.iter().copied().map(u32::from).collect(),
        None => (0..vertices.len() as u32).collect(),
    };
    Ok((vertices.clone(), indices))
}

/// Положение обзорной камеры. Положительный Z камеры направлен вперёд.
#[derive(Clone, Debug)]
pub struct Camera {
    pub eye: Point,
    pub axes: Matrix3<f64>,
    pub lens: Intrinsics,
    near: f64,
}

impl Camera {
    pub fn new(
        eye: Point,
        axes: Matrix3<f64>,
        lens: Intrinsics,
        near: f64,
    ) -> Result<Self, String> {
        lens.validate()?;
        if !eye.coords.iter().chain(axes.iter()).all(|v| v.is_finite())
            || !near.is_finite()
            || near <= 0.0
        {
            return Err("неверная поза обзорной камеры".into());
        }
        Ok(Self {
            eye,
            axes,
            lens,
            near,
        })
    }

    pub fn project(&self, p: Point) -> Option<[f64; 2]> {
        let p = self.local(p);
        if p.z < self.near {
            return None;
        }
        self.lens.project(p.into()).ok().flatten()
    }

    fn local(&self, p: Point) -> Vec3 {
        self.axes.transpose() * (p - self.eye)
    }
}

/// Записанный скаляр в ньютонах. Полоса кодирует модуль, подпись показывает знак.
#[derive(Clone, Debug)]
pub struct Force {
    pub name: String,
    pub value_n: f64,
    pub scale_n: f64,
}

impl Force {
    pub(super) fn pixels(&self, width: usize) -> Result<usize, String> {
        if !self.value_n.is_finite()
            || !self.scale_n.is_finite()
            || self.scale_n < 0.0
            || self.value_n.abs() > self.scale_n
        {
            return Err("неверные данные наложения силы".into());
        }
        Ok(if self.scale_n == 0.0 {
            0
        } else {
            (self.value_n.abs() / self.scale_n * width as f64).round() as usize
        })
    }
}

pub struct Texture {
    pub size: [u32; 2],
    pub rgb: Vec<[u8; 3]>,
    pub ids: Vec<Option<usize>>,
    pub depth: Vec<f64>,
    pub force_pixels: usize,
    pub projected_reactions: Vec<[[f64; 2]; 2]>,
    pub projected_kinematics: Vec<[[f64; 2]; 2]>,
}

impl Texture {
    fn new(size: [u32; 2]) -> Result<Self, String> {
        // Верхняя граница текстуры защищает память при открытии пользовательского отчёта.
        if size.contains(&0) || size.iter().any(|&n| n > 4096) {
            return Err("неверный размер текстуры".into());
        }
        let pixels = size[0] as usize * size[1] as usize;
        Ok(Self {
            size,
            rgb: vec![BACKGROUND; pixels],
            ids: vec![None; pixels],
            depth: vec![f64::INFINITY; pixels],
            force_pixels: 0,
            projected_reactions: Vec::new(),
            projected_kinematics: Vec::new(),
        })
    }

    pub fn save_ppm(&self, path: &std::path::Path) -> std::io::Result<()> {
        use std::io::{BufWriter, Write};
        let mut file = BufWriter::new(std::fs::File::create(path)?);
        writeln!(file, "P6\n{} {}\n255", self.size[0], self.size[1])?;
        file.write_all(self.rgb.as_flattened())?;
        file.flush()
    }

    fn triangle(&mut self, [a, b, c]: [[f64; 3]; 3], id: usize, color: [u8; 3]) {
        let edge = |a: [f64; 3], b: [f64; 3], x: f64, y: f64| {
            (x - a[0]) * (b[1] - a[1]) - (y - a[1]) * (b[0] - a[0])
        };
        let area = edge(a, b, c[0], c[1]);
        if area == 0.0 || !area.is_finite() {
            return;
        }
        let limits = |i: usize| {
            let low = a[i].min(b[i]).min(c[i]).floor().max(0.0) as u32;
            let high = a[i]
                .max(b[i])
                .max(c[i])
                .ceil()
                .clamp(0.0, f64::from(self.size[i])) as u32;
            low..high
        };
        let xs = limits(0);
        let ys = limits(1);
        for y in ys {
            for x in xs.clone() {
                let (px, py) = (f64::from(x) + 0.5, f64::from(y) + 0.5);
                let weights = [
                    edge(b, c, px, py) / area,
                    edge(c, a, px, py) / area,
                    edge(a, b, px, py) / area,
                ];
                if weights.iter().any(|&w| w < 0.0) {
                    continue;
                }
                // Интерполяция обратной глубины в экранных координатах.
                let z = 1.0 / (weights[0] / a[2] + weights[1] / b[2] + weights[2] / c[2]);
                let i = (y * self.size[0] + x) as usize;
                if z < self.depth[i] {
                    self.depth[i] = z;
                    self.ids[i] = Some(id);
                    self.rgb[i] = color;
                }
            }
        }
    }

    fn line(&mut self, from: [f64; 2], to: [f64; 2], color: [u8; 3]) {
        let (dx, dy) = (to[0] - from[0], to[1] - from[1]);
        // Шаг по ведущей оси не превышает один пиксель. Границы пересечения
        // обрезают отрезки даже при камере рядом с точкой наложения.
        let mut range = [0.0_f64, 1.0_f64];
        for (axis, delta) in [dx, dy].into_iter().enumerate() {
            if delta == 0.0 {
                if from[axis] < 0.0 || from[axis] >= f64::from(self.size[axis]) {
                    return;
                }
            } else {
                let a = -from[axis] / delta;
                let b = (f64::from(self.size[axis]) - from[axis]) / delta;
                range[0] = range[0].max(a.min(b));
                range[1] = range[1].min(a.max(b));
            }
        }
        if range[0] > range[1] {
            return;
        }
        let len = (dx.abs().max(dy.abs()) * (range[1] - range[0])).ceil() as usize;
        for i in 0..=len {
            let t = range[0] + (range[1] - range[0]) * i as f64 / len.max(1) as f64;
            let x = (from[0] + dx * t).floor() as u32;
            let y = (from[1] + dy * t).floor() as u32;
            if x < self.size[0] && y < self.size[1] {
                self.rgb[(y * self.size[0] + x) as usize] = color;
            }
        }
    }

    fn reactions(
        &mut self,
        camera: &Camera,
        reactions: &[super::Wrench],
        length_m: f64,
        scale_n: f64,
    ) -> Result<(), String> {
        if !scale_n.is_finite() || scale_n < 0.0 {
            return Err("неверный масштаб векторов силы".into());
        }
        for w in reactions {
            let force = Vec3::from(w.force_n);
            let point = Point::from(w.point_m);
            if !force
                .iter()
                .chain(point.coords.iter())
                .all(|v| v.is_finite())
                || force.norm() > scale_n
            {
                return Err("неверная запись вектора силы".into());
            }
            if force.norm() == 0.0 {
                continue;
            }
            let tip = point + force * (length_m / scale_n);
            if let [Some(a), Some(b)] = [camera.project(point), camera.project(tip)] {
                self.projected_reactions.push([a, b]);
                self.line(a, b, WRENCH_COLOR);
                // Наконечник занимает 1/5 проекции вектора и сохраняет его направление.
                let d = [(b[0] - a[0]) / 5.0, (b[1] - a[1]) / 5.0];
                self.line(b, [b[0] - d[0] + d[1] / 2.0, b[1] - d[1] - d[0] / 2.0], WRENCH_COLOR);
                self.line(b, [b[0] - d[0] - d[1] / 2.0, b[1] - d[1] + d[0] / 2.0], WRENCH_COLOR);
            }
        }
        Ok(())
    }

    fn force_overlay(&mut self, force: &Force) -> Result<(), String> {
        let width = self.size[0] as usize;
        self.force_pixels = force.pixels(width)?;
        for row in self.rgb.chunks_exact_mut(width).rev().take(FORCE_ROWS) {
            row[..self.force_pixels].fill(FORCE_COLOR);
        }
        Ok(())
    }
}

fn clip([a, b, c]: [Vec3; 3], near: f64) -> ([Vec3; 4], usize) {
    let mut out = [Vec3::zeros(); 4];
    let mut len = 0;
    for (from, to) in [(a, b), (b, c), (c, a)] {
        if (from.z >= near) != (to.z >= near) {
            let mut p = from + (to - from) * ((near - from.z) / (to.z - from.z));
            p.z = near;
            out[len] = p;
            len += 1;
        }
        if to.z >= near {
            out[len] = to;
            len += 1;
        }
    }
    (out, len)
}

pub(crate) fn render(
    sim: &Sim,
    geometry: &[Geometry],
    camera: &Camera,
    floor_reach: f64,
    force: Option<&Force>,
    reactions: &[super::Wrench],
    length_m: f64,
    scale_n: f64,
) -> Result<Texture, String> {
    let mut texture = Texture::new([camera.lens.width, camera.lens.height])?;
    for geometry in geometry {
        let (p, r) = sim.geom_pose(geometry.id);
        let world_position = Point::from(p);
        let rotation = Matrix3::from_row_slice(&r);
        // Плоскость ограничивает только изображение. Размер берётся из обзорной области.
        let floor = [
            Point::new(-floor_reach, -floor_reach, 0.0),
            Point::new(floor_reach, -floor_reach, 0.0),
            Point::new(floor_reach, floor_reach, 0.0),
            Point::new(-floor_reach, floor_reach, 0.0),
        ];
        let (vertices, triangles) = if geometry.vertices.is_empty() {
            (&floor[..], &[[0, 1, 2], [0, 2, 3]][..])
        } else {
            (&geometry.vertices[..], &geometry.triangles[..])
        };
        let camera_points: Vec<_> = vertices
            .iter()
            .map(|v| camera.local(world_position + rotation * v.coords))
            .collect();
        for triangle in triangles {
            let [a, b, c] = triangle.map(|i| camera_points[i as usize]);
            let (vertices, count) = clip([a, b, c], camera.near);
            if count < 3 {
                continue;
            }
            // Диффузная подсветка граней. Это только контраст для интерфейса.
            let intensity = (b - a)
                .cross(&(c - a))
                .try_normalize(0.0)
                .map_or(0.5, |n| 0.55 + 0.45 * n.z.abs());
            let color = std::array::from_fn(|i| {
                (f64::from(geometry.color[i]).clamp(0.0, 1.0) * intensity * f64::from(u8::MAX))
                    .round() as u8
            });
            let project = |p: Vec3| -> Result<[f64; 3], String> {
                let [x, y] = camera
                    .lens
                    .project(p.into())?
                    .ok_or("не удалось спроецировать отсечённую грань")?;
                Ok([x, y, p.z])
            };
            let first = project(vertices[0])?;
            for i in 1..count - 1 {
                texture.triangle(
                    [first, project(vertices[i])?, project(vertices[i + 1])?],
                    geometry.id,
                    color,
                );
            }
        }
    }
    if needs_kinematics(sim, geometry) {
        for [a, b] in kinematics(sim) {
            if let Some([from, to]) = camera.project(a).zip(camera.project(b)).map(|(a, b)| [a, b]) {
                texture.line(from, to, KINEMATICS_COLOR);
                texture.projected_kinematics.push([from, to]);
            }
        }
    }
    texture.reactions(camera, reactions, length_m, scale_n)?;
    if let Some(force) = force {
        texture.force_overlay(force)?;
    }
    Ok(texture)
}

pub(super) fn needs_kinematics(sim: &Sim, geometry: &[Geometry]) -> bool {
    sim.nbody() > 1 && !geometry.iter().any(|g| g.body != 0 && !g.vertices.is_empty())
}

fn kinematics(sim: &Sim) -> impl Iterator<Item = [Point; 2]> + '_ {
    (1..sim.nbody()).flat_map(|id| {
        let origin = Point::from(sim.body_pose(id).0);
        // MuJoCo хранит одного родителя и три координаты центра масс на тело.
        // Индекс берётся из диапазона nbody того же экземпляра.
        let (parent, center) = unsafe {
            let parent = *sim.model().body_parentid.add(id) as usize;
            let center = *sim.data().xipos.cast::<[f64; 3]>().add(id);
            (parent, Point::from(center))
        };
        [[origin, center]].into_iter().chain(
            (parent != 0).then(|| [Point::from(sim.body_pose(parent).0), origin])
        )
    })
}

pub(super) fn kinematic_bounds(sim: &Sim) -> Option<(Point, f64)> {
    let mut points = kinematics(sim).flatten();
    let first = points.next()?.coords;
    let (low, high) = points.fold((first, first), |(lo, hi), p| {
        (lo.inf(&p.coords), hi.sup(&p.coords))
    });
    let radius = (high - low).norm() / 2.0;
    let center = Point::from((high + low) / 2.0);
    // Для совпавших точек используется штатный метровый масштаб обзора.
    (radius.is_finite() && center.coords.iter().all(|v| v.is_finite()))
        .then(|| (center, if radius > 0.0 { radius } else { 1.0 }))
}

pub(crate) fn bounds(
    sim: &Sim,
    geometry: &[Geometry],
    body: Option<usize>,
) -> Option<(Point, f64)> {
    let mut low = Vec3::repeat(f64::INFINITY);
    let mut high = Vec3::repeat(f64::NEG_INFINITY);
    let use_environment = !geometry
        .iter()
        .any(|g| g.body != 0 && !g.vertices.is_empty());
    for g in geometry
        .iter()
        .filter(|g| body.map_or(use_environment || g.body != 0, |id| id == g.body))
    {
        let (p, r) = sim.geom_pose(g.id);
        let rotation = Matrix3::from_row_slice(&r);
        for vertex in &g.vertices {
            let v = rotation * vertex.coords + Vec3::from(p);
            low = low.inf(&v);
            high = high.sup(&v);
        }
    }
    let radius = (high - low).norm() / 2.0;
    (radius.is_finite() && radius > 0.0).then(|| (Point::from((high + low) / 2.0), radius))
}
