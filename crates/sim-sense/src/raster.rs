//! Растеризация в нормализованном кадре: экранные границы ограничивают покрытие,
//! обратная оптика определяет точку теста, z-буфер хранит ближайшую поверхность.

use std::{collections::BTreeMap, ops::Range};

use nalgebra::{Matrix3, Point3};
use sim_mujoco::{GeomKind, Sim, ffi};

use crate::optics::{Frame, Intrinsics, Pose, Vec3, checked, pose};
use crate::vision::ObjectPose;

#[derive(Clone, Copy)]
enum Surface {
    Plane,
    Ellipsoid(Vec3),
    Capsule { radius: f64, half: f64 },
    Cylinder { radius: f64, half: f64 },
    Box(Vec3),
    Triangle([Vec3; 3]),
}

struct Texture {
    width: usize,
    height: usize,
    channels: usize,
    colors: Vec<u8>,
    repeat: [f64; 2],
    uniform: bool,
}

impl Texture {
    fn read(sim: &Sim, geom: usize) -> Result<Option<Self>, String> {
        let m = sim.model();
        // Индексы и размеры ниже взяты из одной скомпилированной модели MuJoCo.
        unsafe {
            let material = *m.geom_matid.add(geom);
            if material < 0 {
                return Ok(None);
            }
            let material = material as usize;
            let base = material * ffi::mjtTextureRole::mjNTEXROLE as usize;
            let texture = [
                ffi::mjtTextureRole::mjTEXROLE_RGB,
                ffi::mjtTextureRole::mjTEXROLE_RGBA,
                ffi::mjtTextureRole::mjTEXROLE_USER,
            ]
            .into_iter()
            .map(|role| *m.mat_texid.add(base + role as usize))
            .find(|&id| id >= 0);
            let Some(texture) = texture else {
                return Ok(None);
            };
            let i = texture as usize;
            if *m.tex_type.add(i) != ffi::mjtTexture::mjTEXTURE_2D as i32 {
                // Цветовые карты куба не являются частью упрощённой оптики.
                return Ok(None);
            }
            let width = *m.tex_width.add(i) as usize;
            let height = *m.tex_height.add(i) as usize;
            let channels = *m.tex_nchannel.add(i) as usize;
            let start = *m.tex_adr.add(i) as usize;
            let end = width
                .checked_mul(height)
                .and_then(|n| n.checked_mul(channels))
                .and_then(|n| n.checked_add(start))
                .filter(|&end| end <= m.ntexdata as usize)
                .ok_or("размер текстуры MuJoCo вне диапазона")?;
            let repeat = [
                f64::from(*m.mat_texrepeat.add(2 * material)),
                f64::from(*m.mat_texrepeat.add(2 * material + 1)),
            ];
            checked(&repeat, "повторы текстуры")?;
            if width == 0 || height == 0 || !matches!(channels, 1 | 3 | 4) {
                return Err("неверный размер или формат цветовой текстуры".into());
            }
            Ok(Some(Self {
                width,
                height,
                channels,
                repeat,
                uniform: *m.mat_texuniform.add(material),
                colors: std::slice::from_raw_parts(m.tex_data.add(start), end - start).to_vec(),
            }))
        }
    }

    fn rgb(&self, point: Vec3, size: Vec3) -> [f64; 3] {
        // Планарная развёртка XY. texuniform задаёт повторы на метр; иначе
        // прямоугольник [-size,+size] занимает по одному периоду при texrepeat=1.
        let uv = |coord: f64, half: f64, repeat: f64, count: usize| {
            let u = if self.uniform || half == 0.0 {
                coord
            } else {
                coord / (2.0 * half) + 0.5
            };
            ((u * repeat).rem_euclid(1.0) * count as f64) as usize % count
        };
        let x = uv(point.x, size.x, self.repeat[0], self.width);
        let y = uv(point.y, size.y, self.repeat[1], self.height);
        let base = (y * self.width + x) * self.channels;
        std::array::from_fn(|c| {
            f64::from(self.colors[base + if self.channels == 1 { 0 } else { c }])
                / f64::from(u8::MAX)
        })
    }
}

struct Geometry {
    id: u32,
    name: String,
    body: u32,
    body_name: String,
    anchor: usize,
    local: Frame,
    size: Vec3,
    color: [f64; 3],
    texture: Option<Texture>,
    surfaces: Vec<Surface>,
}

pub(crate) struct Scene {
    geometries: Vec<Geometry>,
    bodies: Vec<usize>,
}

impl Scene {
    pub fn new(sim: &Sim, color: bool) -> Result<Self, String> {
        let mut geometries = Vec::new();
        let mut bodies = Vec::new();
        let mut anchors = BTreeMap::new();
        for g in sim.geoms() {
            checked(&g.size, "размеры геометрии")?;
            checked(&g.rgba.map(f64::from), "цвет геометрии")?;
            // Компилятор мира задаёт alpha=0 для коллизий и инерционных объёмов.
            if g.rgba[3] <= 0.0 {
                continue;
            }
            let size = Vec3::from(g.size);
            let one = |s| vec![s];
            let surfaces = match g.kind {
                GeomKind::Plane => one(Surface::Plane),
                GeomKind::Sphere => one(Surface::Ellipsoid(Vec3::repeat(size.x))),
                GeomKind::Ellipsoid => one(Surface::Ellipsoid(size)),
                GeomKind::Capsule => one(Surface::Capsule {
                    radius: size.x,
                    half: size.y,
                }),
                GeomKind::Cylinder => one(Surface::Cylinder {
                    radius: size.x,
                    half: size.y,
                }),
                GeomKind::Box => one(Surface::Box(size)),
                GeomKind::Mesh => {
                    let mesh = sim.mesh(g.mesh.ok_or("геометрия сетки не ссылается на сетку")?);
                    let (faces, remainder) = mesh.indices.as_chunks::<3>();
                    if !remainder.is_empty() {
                        return Err("неполная грань сетки".into());
                    }
                    faces
                        .iter()
                        .map(|face| {
                            let vertex = |i: u32| -> Result<Vec3, String> {
                                let vertex = mesh
                                    .positions
                                    .get(i as usize)
                                    .ok_or("индекс грани вне сетки")?;
                                let p = vertex.map(f64::from);
                                checked(&p, "вершина сетки")?;
                                Ok(Vec3::from(p))
                            };
                            Ok(Surface::Triangle([
                                vertex(face[0])?,
                                vertex(face[1])?,
                                vertex(face[2])?,
                            ]))
                        })
                        .collect::<Result<_, String>>()?
                }
                GeomKind::Other => {
                    return Err(format!("видимая геометрия «{}» не поддержана", g.name));
                }
            };
            let anchor = *anchors.entry(g.body).or_insert_with(|| {
                let index = bodies.len();
                bodies.push(g.body);
                index
            });
            // Локальная поза из скомпилированной модели, включая преобразование сетки.
            let local = unsafe {
                let m = sim.model();
                Pose {
                    position_m: *m.geom_pos.cast::<[f64; 3]>().add(g.id),
                    quat_wxyz: *m.geom_quat.cast::<[f64; 4]>().add(g.id),
                }
            }
            .frame()?;
            geometries.push(Geometry {
                id: u32::try_from(g.id).map_err(|_| "номер геометрии превышает u32")?,
                name: g.name,
                body: u32::try_from(g.body).map_err(|_| "номер тела превышает u32")?,
                body_name: sim.name(ffi::mjtObj::mjOBJ_BODY, g.body),
                anchor,
                local,
                size,
                color: [g.rgba[0], g.rgba[1], g.rgba[2]].map(f64::from),
                texture: if color {
                    Texture::read(sim, g.id)?
                } else {
                    None
                },
                surfaces,
            });
        }
        Ok(Self { geometries, bodies })
    }

    pub fn frames(&self, sim: &Sim) -> Result<Vec<Frame>, String> {
        self.bodies
            .iter()
            .map(|&id| {
                if id >= sim.nbody() {
                    return Err("тело геометрии камеры отсутствует в движке".into());
                }
                let (position_m, quat_wxyz) = sim.body_pose(id);
                Pose {
                    position_m,
                    quat_wxyz,
                }
                .frame()
            })
            .collect()
    }

    pub fn objects(&self, frames: &[Frame]) -> Result<Vec<ObjectPose>, String> {
        self.geometries
            .iter()
            .map(|g| {
                let frame = frames[g.anchor] * g.local;
                checked(frame.translation.vector.as_slice(), "позиция геометрии")?;
                Ok(ObjectPose {
                    id: g.id,
                    name: g.name.clone(),
                    body: g.body,
                    body_name: g.body_name.clone(),
                    pose: pose(&frame),
                })
            })
            .collect()
    }

    pub fn prepare<'a>(
        &'a self,
        camera: &Frame,
        frames: &[Frame],
        near: f64,
        far: f64,
    ) -> Result<Vec<Primitive<'a>>, String> {
        let view = camera.inverse();
        let mut geometry = Vec::new();
        for g in &self.geometries {
            let local_to_camera = view * (frames[g.anchor] * g.local);
            let camera_to_local = local_to_camera.inverse();
            checked(
                camera_to_local.translation.vector.as_slice(),
                "относительная поза геометрии",
            )?;
            let rotation = camera_to_local.rotation.to_rotation_matrix().into_inner();
            let normal = rotation.column(2).into_owned();
            for &surface in &g.surfaces {
                if let Some(bounds) = bounds(surface, &local_to_camera, near, far)? {
                    geometry.push(Primitive {
                        surface,
                        bounds,
                        material: g,
                        rotation,
                        normal,
                        eye: camera_to_local.translation.vector,
                    });
                }
            }
        }
        Ok(geometry)
    }
}

#[derive(Clone, Copy)]
struct Bounds {
    min: [f64; 2],
    max: [f64; 2],
}

impl Bounds {
    fn empty() -> Self {
        Self {
            min: [f64::INFINITY; 2],
            max: [f64::NEG_INFINITY; 2],
        }
    }

    fn unbounded() -> Self {
        Self {
            min: [f64::NEG_INFINITY; 2],
            max: [f64::INFINITY; 2],
        }
    }

    fn include(&mut self, [x, y]: [f64; 2]) {
        self.min[0] = self.min[0].min(x);
        self.min[1] = self.min[1].min(y);
        self.max[0] = self.max[0].max(x);
        self.max[1] = self.max[1].max(y);
    }

    fn overlaps(self, b: Self) -> bool {
        (0..2).all(|i| self.min[i] <= b.max[i] && self.max[i] >= b.min[i])
    }

    fn contains(self, p: [f64; 2]) -> bool {
        (0..2).all(|i| self.min[i] <= p[i] && p[i] <= self.max[i])
    }
}

fn bounds(surface: Surface, pose: &Frame, near: f64, far: f64) -> Result<Option<Bounds>, String> {
    let mut result = Bounds::empty();
    let mut min_z = f64::INFINITY;
    let mut max_z = f64::NEG_INFINITY;
    let mut include = |p| -> Result<(), String> {
        let p = pose.transform_point(&Point3::from(p));
        checked(p.coords.as_slice(), "граница растеризации")?;
        min_z = min_z.min(p.z);
        max_z = max_z.max(p.z);
        if p.z >= near {
            let xy = [p.x / p.z, p.y / p.z];
            checked(&xy, "экранная граница растеризации")?;
            result.include(xy);
        }
        Ok(())
    };
    match surface {
        Surface::Plane => return Ok(Some(Bounds::unbounded())),
        Surface::Triangle(vertices) => {
            for v in vertices {
                include(v)?;
            }
        }
        _ => {
            let half = match surface {
                Surface::Box(half) | Surface::Ellipsoid(half) => half,
                Surface::Capsule { radius, half } => Vec3::new(radius, radius, half + radius),
                Surface::Cylinder { radius, half } => Vec3::new(radius, radius, half),
                _ => unreachable!("плоскость и треугольник обработаны"),
            };
            for x in [-half.x, half.x] {
                for y in [-half.y, half.y] {
                    for z in [-half.z, half.z] {
                        include(Vec3::new(x, y, z))?;
                    }
                }
            }
        }
    }
    if max_z < near || min_z > far {
        return Ok(None);
    }
    if min_z < near {
        return Ok(Some(Bounds::unbounded()));
    }
    // Защита границы от округления преобразования 3×3, деления и сравнения луча.
    const ROUNDING_STEPS: f64 = 16.0;
    for i in 0..2 {
        let guard =
            ROUNDING_STEPS * f64::EPSILON * result.min[i].abs().max(result.max[i].abs()).max(1.0);
        result.min[i] -= guard;
        result.max[i] += guard;
    }
    Ok(Some(result))
}

pub(crate) struct Primitive<'a> {
    surface: Surface,
    bounds: Bounds,
    material: &'a Geometry,
    rotation: Matrix3<f64>,
    normal: Vec3,
    eye: Vec3,
}

#[derive(Clone, Copy, Default)]
pub(crate) struct Pixel {
    pub id: Option<u32>,
    pub depth: f64,
    pub color: [f64; 3],
}

struct Tile {
    x: usize,
    rays: Vec<[f64; 2]>,
    bounds: Bounds,
}

pub(crate) struct Grid {
    lens: Intrinsics,
    rows: Vec<Vec<Tile>>,
}

impl Grid {
    pub fn new(lens: Intrinsics) -> Result<Self, String> {
        // Верхний предел одного z-буфера — 512 МиБ, включая растр DCI 4K.
        // Проверка до выделения памяти сохраняет явную ошибку для непригодной настройки.
        const MAX_BUFFER_BYTES: usize = 512 * 1024 * 1024;
        let _ = (lens.width as usize)
            .checked_mul(lens.height as usize)
            .and_then(|n| n.checked_mul(std::mem::size_of::<Pixel>()))
            .filter(|&n| n <= MAX_BUFFER_BYTES.min(isize::MAX as usize))
            .ok_or("буфер кадра превышает лимит памяти растеризатора")?;
        let mut rows = Vec::new();
        rows.try_reserve_exact(lens.height as usize)
            .map_err(|_| "не удалось выделить таблицу лучей")?;
        for y in 0..lens.height as usize {
            rows.push(Self::row(lens, y, [0.5; 2])?);
        }
        // Края тоже входят в область калибровки.
        for y in [0.0, f64::from(lens.height)] {
            for x in [0.0, f64::from(lens.width)] {
                lens.ray(x, y)?;
            }
        }
        Ok(Self { lens, rows })
    }

    fn row(lens: Intrinsics, y: usize, offset: [f64; 2]) -> Result<Vec<Tile>, String> {
        // Плитка 256 байт лучей: мелкие экранные границы отсекают соседние плитки целиком.
        const TILE: usize = 256 / std::mem::size_of::<[f64; 2]>();
        let width = lens.width as usize;
        let mut row = Vec::new();
        row.try_reserve_exact(width.div_ceil(TILE))
            .map_err(|_| "не удалось выделить строку лучей")?;
        for x in (0..width).step_by(TILE) {
            let mut bounds = Bounds::empty();
            let rays = (x..width.min(x + TILE))
                .map(|i| {
                    let ray = lens.ray(i as f64 + offset[0], y as f64 + offset[1])?;
                    bounds.include(ray);
                    Ok(ray)
                })
                .collect::<Result<_, String>>()?;
            row.push(Tile { x, rays, bounds });
        }
        Ok(row)
    }

    pub fn draw(
        &self,
        rows: Range<usize>,
        geometry: &[Primitive<'_>],
        band: (f64, f64),
        ambient: Option<f64>,
        offset: [f64; 2],
    ) -> Result<Vec<Pixel>, String> {
        let width = self.lens.width as usize;
        let mut buffer = Vec::new();
        buffer
            .try_reserve_exact(width * rows.len())
            .map_err(|_| "не удалось выделить буфер растеризации")?;
        buffer.resize(width * rows.len(), Pixel::default());
        for (y, scan) in rows.zip(buffer.chunks_exact_mut(width)) {
            let custom;
            let tiles = if offset == [0.5; 2] {
                &self.rows[y]
            } else {
                custom = Self::row(self.lens, y, offset)?;
                &custom
            };
            for g in geometry {
                for tile in tiles.iter().filter(|tile| g.bounds.overlaps(tile.bounds)) {
                    for (i, &ray) in tile
                        .rays
                        .iter()
                        .enumerate()
                        .filter(|(_, p)| g.bounds.contains(**p))
                    {
                        let pixel = &mut scan[tile.x + i];
                        let d = g.rotation * Vec3::new(ray[0], ray[1], 1.0);
                        let limit = if pixel.id.is_some() {
                            pixel.depth
                        } else {
                            band.1
                        };
                        if let Some(depth) = g.surface.intersect(g.eye, d, band.0, limit)
                            && (pixel.id.is_none() || depth < pixel.depth)
                        {
                            pixel.depth = depth;
                            pixel.id = Some(g.material.id);
                            if let Some(ambient) = ambient {
                                let point = g.eye + depth * d;
                                let n = g.surface.normal(point);
                                // Двустороннее ламбертово освещение от направления оптической оси.
                                let light =
                                    ambient + (1.0 - ambient) * n.dot(&g.normal).abs().min(1.0);
                                let texture = g
                                    .material
                                    .texture
                                    .as_ref()
                                    .map_or([1.0; 3], |t| t.rgb(point, g.material.size));
                                pixel.color = std::array::from_fn(|c| {
                                    g.material.color[c] * texture[c] * light
                                });
                            }
                        }
                    }
                }
            }
        }
        Ok(buffer)
    }
}

impl Surface {
    fn intersect(self, p: Vec3, d: Vec3, near: f64, far: f64) -> Option<f64> {
        let accept = |z: f64| (near..=far).contains(&z);
        match self {
            Self::Plane => {
                let z = -p.z / d.z;
                accept(z).then_some(z)
            }
            Self::Ellipsoid(r) => {
                let p = p.component_div(&r);
                let d = d.component_div(&r);
                roots(d.dot(&d), p.dot(&d), p.dot(&p) - 1.0)
                    .into_iter()
                    .flatten()
                    .find(|&z| accept(z))
            }
            Self::Capsule { radius, half } | Self::Cylinder { radius, half } => {
                let mut best: Option<f64> = None;
                let mut check = |candidate, valid| {
                    if valid
                        && accept(candidate)
                        && best.is_none_or(|previous| candidate < previous)
                    {
                        best = Some(candidate);
                    }
                };
                let r2 = radius * radius;
                for z in roots(
                    d.x * d.x + d.y * d.y,
                    p.x * d.x + p.y * d.y,
                    p.x * p.x + p.y * p.y - r2,
                )
                .into_iter()
                .flatten()
                {
                    check(z, (p.z + z * d.z).abs() <= half);
                }
                for sign in [-1.0, 1.0] {
                    let end = sign * half;
                    if matches!(self, Self::Capsule { .. }) {
                        let offset = p - Vec3::new(0.0, 0.0, end);
                        for z in roots(d.dot(&d), offset.dot(&d), offset.dot(&offset) - r2)
                            .into_iter()
                            .flatten()
                        {
                            check(z, sign * (p.z + z * d.z - end) >= 0.0);
                        }
                    } else {
                        let z = (end - p.z) / d.z;
                        let q = p + z * d;
                        check(z, q.x * q.x + q.y * q.y <= r2);
                    }
                }
                best
            }
            Self::Box(h) => {
                let mut entry = f64::NEG_INFINITY;
                let mut exit = f64::INFINITY;
                for i in 0..3 {
                    if d[i] == 0.0 {
                        if p[i].abs() > h[i] {
                            return None;
                        }
                    } else {
                        let a = (-h[i] - p[i]) / d[i];
                        let b = (h[i] - p[i]) / d[i];
                        entry = entry.max(a.min(b));
                        exit = exit.min(a.max(b));
                    }
                }
                (entry <= exit)
                    .then(|| [entry, exit].into_iter().find(|&z| accept(z)))
                    .flatten()
            }
            Self::Triangle([a, b, c]) => {
                // Möller — Trumbore, двусторонний тест покрытия треугольника.
                let ab = b - a;
                let ac = c - a;
                let h = d.cross(&ac);
                let det = ab.dot(&h);
                if det == 0.0 {
                    return None;
                }
                let s = p - a;
                let u = s.dot(&h) / det;
                let q = s.cross(&ab);
                let v = d.dot(&q) / det;
                let z = ac.dot(&q) / det;
                (u >= 0.0 && v >= 0.0 && u + v <= 1.0 && accept(z)).then_some(z)
            }
        }
    }

    fn normal(self, p: Vec3) -> Vec3 {
        let n = match self {
            Self::Plane => Vec3::z(),
            Self::Ellipsoid(r) => p.component_div(&r).component_div(&r),
            Self::Capsule { half, .. } => Vec3::new(p.x, p.y, p.z - p.z.clamp(-half, half)),
            Self::Cylinder { radius, half } => {
                if (p.z.abs() - half).abs() <= (p.x.hypot(p.y) - radius).abs() {
                    Vec3::new(0.0, 0.0, p.z.signum())
                } else {
                    Vec3::new(p.x, p.y, 0.0)
                }
            }
            Self::Box(h) => {
                let gap = p.abs() - h;
                let (axis, _) = gap
                    .iter()
                    .enumerate()
                    .min_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
                    .unwrap();
                let mut n = Vec3::zeros();
                n[axis] = p[axis].signum();
                n
            }
            Self::Triangle([a, b, c]) => (b - a).cross(&(c - a)),
        };
        n.normalize()
    }
}

fn roots(a: f64, b: f64, c: f64) -> Option<[f64; 2]> {
    // Уравнение ax² + 2bx + c = 0. Вторая форма убирает вычитание близких чисел.
    let disc = b * b - a * c;
    if a <= 0.0 || disc < 0.0 {
        return None;
    }
    let q = -b - disc.sqrt().copysign(b);
    let x = q / a;
    let y = if q == 0.0 { x } else { c / q };
    Some([x.min(y), x.max(y)])
}
