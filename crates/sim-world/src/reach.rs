//! Внешние огибающие геометрий для аудита подавленного контакта.

use nalgebra::Vector3;
use sim_mujoco::{GeomKind, JointKind, Sim, ffi::{mjtDisableBit, mjtObj as Obj}};

use crate::CompiledWorld;

type Vec3 = Vector3<f64>;

#[derive(Clone, Copy)]
struct Bounds {
    lower: Vec3,
    upper: Vec3,
}

impl Bounds {
    fn sphere(center: Vec3, radius: f64) -> Self {
        if !radius.is_finite() {
            return Self {
                lower: Vec3::repeat(f64::NEG_INFINITY),
                upper: Vec3::repeat(f64::INFINITY),
            };
        }
        Self {
            lower: center.add_scalar(-radius),
            upper: center.add_scalar(radius),
        }
    }

    fn overlaps(&self, other: &Self) -> bool {
        (0..3).all(|i| {
            // Запас округления пропорционален масштабу координат сравниваемых граней.
            let scale = self.lower[i]
                .abs()
                .max(self.upper[i].abs())
                .max(other.lower[i].abs())
                .max(other.upper[i].abs());
            let padding = f64::EPSILON * scale;
            self.lower[i] - other.upper[i] <= padding && other.lower[i] - self.upper[i] <= padding
        })
    }

    fn slide(&mut self, axis: Vec3) {
        for i in 0..3 {
            if axis[i] == 0.0 {
                continue;
            }
            self.lower[i] = f64::NEG_INFINITY;
            self.upper[i] = f64::INFINITY;
        }
    }

    fn rotate(&mut self, anchor: Vec3) {
        let extent = (self.lower - anchor)
            .abs()
            .sup(&(self.upper - anchor).abs());
        let radius = extent.x.hypot(extent.y).hypot(extent.z);
        // Для произвольного угла расстояние между двумя позами точки
        // ограничено диаметром окружности её движения.
        let travel = 2.0 * radius;
        *self = if travel.is_finite() {
            Self {
                lower: self.lower.add_scalar(-travel),
                upper: self.upper.add_scalar(travel),
            }
        } else {
            Self::sphere(anchor, f64::INFINITY)
        };
    }
}

enum Motion {
    Slide { axis: Vec3 },
    Rotate { anchor: Vec3 },
    Free,
}

struct Surface {
    name: String,
    body: usize,
    bounds: Bounds,
}

pub(super) struct ReachScene {
    parents: Vec<usize>,
    weld: Vec<usize>,
    motions: Vec<Vec<Motion>>,
    surfaces: Vec<Vec<Surface>>,
}

impl ReachScene {
    pub(super) fn new(sim: &Sim, is_collision: impl Fn(&str) -> bool) -> Result<Self, String> {
        let model = sim.model();
        let data = sim.data();
        // SAFETY: индексы получены из той же модели и ограничены числом её
        // тел, геометрий и сочленений. Движок хранит xaxis и xanchor тройками.
        let (parents, weld) = unsafe {
            (0..sim.nbody())
                .map(|body| {
                    (
                        *model.body_parentid.add(body) as usize,
                        *model.body_weldid.add(body) as usize,
                    )
                })
                .unzip()
        };
        let mut motions: Vec<Vec<_>> = (0..sim.nbody()).map(|_| Vec::new()).collect();
        for joint in sim.joints() {
            let read = |ptr: *const f64| {
                Vec3::from(std::array::from_fn(|i| unsafe {
                    *ptr.add(3 * joint.id + i)
                }))
            };
            // Мягкий предел MuJoCo способен пропустить ось за номинальный
            // ход. Без огибающей перебега считаем всю степень свободы.
            let motion = match joint.kind {
                JointKind::Free => Motion::Free,
                JointKind::Slide => Motion::Slide {
                    axis: read(data.xaxis),
                },
                JointKind::Hinge | JointKind::Ball => Motion::Rotate {
                    anchor: read(data.xanchor),
                },
            };
            motions[joint.body].push(motion);
        }
        let mut surfaces: Vec<Vec<_>> = (0..sim.nbody()).map(|_| Vec::new()).collect();
        for geom in sim.geoms() {
            // У фрагментов не все контактные геометрии имеют внешний псевдоним.
            let (mask, radius, margin) = unsafe {
                (
                    *model.geom_contype.add(geom.id) | *model.geom_conaffinity.add(geom.id),
                    *model.geom_rbound.add(geom.id),
                    *model.geom_margin.add(geom.id),
                )
            };
            if !is_collision(&geom.name) && mask == 0 {
                continue;
            }
            let (center, _) = sim.geom_pose(geom.id);
            if center.iter().any(|v| !v.is_finite()) || !radius.is_finite() || !margin.is_finite() {
                return Err("неконечная геометрия огибающей контакта".into());
            }
            let radius = if radius > 0.0 && !matches!(geom.kind, GeomKind::Plane | GeomKind::Other)
            {
                radius + margin.max(0.0)
            } else {
                // Бесконечная плоскость или форма без доступной внешней сферы.
                f64::INFINITY
            };
            surfaces[geom.body].push(Surface {
                name: if geom.name.is_empty() {
                    format!("геометрия[{}]", geom.id)
                } else {
                    geom.name
                },
                body: geom.body,
                bounds: Bounds::sphere(center.into(), radius),
            });
        }
        Ok(Self {
            parents,
            weld,
            motions,
            surfaces,
        })
    }

    fn ancestors(&self, body: usize) -> impl Iterator<Item = usize> {
        std::iter::successors(Some(body), |&body| (body != 0).then(|| self.parents[body]))
    }

    fn can_meet(&self, a: &Surface, b: &Surface) -> bool {
        // Выше общего предка движение переносит обе геометрии одинаково.
        let common = self
            .ancestors(a.body)
            .find(|ancestor| self.ancestors(b.body).any(|body| body == *ancestor))
            .expect("у всех тел есть общий мировой корень");
        let sweep = |surface: &Surface| {
            let mut bounds = surface.bounds;
            for body in self
                .ancestors(surface.body)
                .take_while(|&body| body != common)
            {
                for motion in self.motions[body].iter().rev() {
                    match *motion {
                        Motion::Slide { axis } => bounds.slide(axis),
                        Motion::Rotate { anchor } => bounds.rotate(anchor),
                        Motion::Free => return Bounds::sphere(Vec3::zeros(), f64::INFINITY),
                    }
                }
            }
            bounds
        };
        sweep(a).overlaps(&sweep(b))
    }

    pub(super) fn suppressed_pairs(&self, sim: &Sim) -> Vec<[String; 2]> {
        let model = sim.model();
        let filter_parent = model.opt.disableflags & mjtDisableBit::mjDSBL_FILTERPARENT as i32 == 0;
        // SAFETY: массив принадлежит модели и содержит ровно nexclude сигнатур.
        let excluded: std::collections::BTreeSet<_> = (0..model.nexclude as usize)
            .map(|i| {
                let signature = unsafe { *model.exclude_signature.add(i) as u32 };
                [
                    (signature >> u16::BITS) as usize,
                    (signature & u32::from(u16::MAX)) as usize,
                ]
            })
            .collect();
        let mut pairs = Vec::new();
        for (a, first) in self
            .surfaces
            .iter()
            .enumerate()
            .filter(|(_, geoms)| !geoms.is_empty())
        {
            for (b, second) in self.surfaces.iter().enumerate().skip(a + 1) {
                let (wa, wb) = (self.weld[a], self.weld[b]);
                let same_weld = wa == wb;
                let excluded = excluded.contains(&[a, b]);
                if same_weld && !excluded && (self.parents[b] == a || self.parents[a] == b) {
                    continue;
                }
                // MuJoCo фильтрует по родителям подвижных групп, включая
                // геометрии на их жёстко присоединённых дочерних телах.
                let adjacent = filter_parent && wa != 0 && wb != 0 && !same_weld
                    && (self.weld[self.parents[wa]] == wb || self.weld[self.parents[wb]] == wa);
                if same_weld || adjacent || excluded {
                    for x in first {
                        for y in second {
                            if self.can_meet(x, y) {
                                pairs.push([x.name.clone(), y.name.clone()]);
                            }
                        }
                    }
                }
            }
        }
        pairs
    }
}

impl CompiledWorld {
    /// Тела, которые по геометрии и ходу осей могут оказаться в контакте
    /// с приводной сборкой. У тел формата мира роль участника не типизирована,
    /// поэтому отдельно смоделированное тело рассматривается как возможный участник.
    pub fn powered_contact_candidates(&self) -> Result<Vec<String>, String> {
        let powered: std::collections::BTreeSet<_> = self
            .joints
            .iter()
            .filter(|joint| self.efforts.iter().any(|effort| effort.joint == joint.name))
            .map(|joint| &joint.owner)
            .collect();
        if powered.is_empty() || self.bodies.len() < 2 {
            return Ok(Vec::new());
        }
        let sim = Sim::from_xml(&self.mjcf)?;
        let scene = ReachScene::new(&sim, |name| {
            name.rsplit_once('/')
                .is_some_and(|(prefix, _)| prefix.ends_with("/collision"))
        })?;
        let bodies = self
            .bodies
            .iter()
            .map(|body| {
                let id = sim
                    .id(Obj::mjOBJ_BODY, &body.name)
                    .ok_or_else(|| format!("нет тела «{}» при аудите досягаемости", body.name))?;
                Ok((body, &scene.surfaces[id]))
            })
            .collect::<Result<Vec<_>, String>>()?;
        let mut candidates = std::collections::BTreeSet::new();
        for (source, first) in &bodies {
            if !powered.contains(&source.owner) {
                continue;
            }
            for (target, second) in &bodies {
                if source.name == target.name {
                    continue;
                }
                // У тела без поверхности нельзя подтвердить геометрическую изоляцию.
                if first.is_empty()
                    || second.is_empty()
                    || first
                        .iter()
                        .any(|a| second.iter().any(|b| scene.can_meet(a, b)))
                {
                    candidates.insert(target.name.clone());
                }
            }
        }
        Ok(candidates.into_iter().collect())
    }
}
