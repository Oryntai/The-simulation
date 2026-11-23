//! Кинематика дерева по описанию и переданным координатам, на единицах СИ.

mod math;
mod solve;
mod trajectory;

use std::collections::{BTreeSet, VecDeque};

pub use math::Pose;
use math::{AXES, add, cross, finite, rotation_vector_axes, scale, sub, unit};
pub use solve::*;
pub use trajectory::*;

pub type Vec3 = [f64; 3];
pub type Result<T> = std::result::Result<T, KinematicError>;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum KinematicError {
    Invalid(String),
    JointLimit { coordinate: usize },
    Numerical(String),
}

impl std::fmt::Display for KinematicError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Invalid(message) | Self::Numerical(message) => f.write_str(message),
            Self::JointLimit { coordinate } => write!(f, "координата {coordinate} за пределами"),
        }
    }
}

impl std::error::Error for KinematicError {}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LinkId(pub usize);

/// Замкнутый интервал независимой координаты. `None` означает явно неограниченную сторону.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CoordinateLimit {
    pub min: Option<f64>,
    pub max: Option<f64>,
}

impl CoordinateLimit {
    fn valid(self) -> bool {
        self.min.is_none_or(f64::is_finite)
            && self.max.is_none_or(f64::is_finite)
            && !matches!((self.min, self.max), (Some(lo), Some(hi)) if lo > hi)
    }

    pub fn contains(self, value: f64) -> bool {
        value.is_finite()
            && self.min.is_none_or(|lo| value >= lo)
            && self.max.is_none_or(|hi| value <= hi)
    }

    fn project(self, mut value: f64) -> f64 {
        if let Some(lo) = self.min {
            value = value.max(lo)
        }
        if let Some(hi) = self.max {
            value = value.min(hi)
        }
        value
    }

    fn locked(self) -> bool {
        matches!((self.min, self.max), (Some(lo), Some(hi)) if lo == hi)
    }
}

#[derive(Clone, Debug)]
pub enum Joint {
    Fixed,
    Hinge {
        axis: Vec3,
        limit: CoordinateLimit,
    },
    Slide {
        axis: Vec3,
        limit: CoordinateLimit,
    },
    /// Вектор поворота exp(r), рад. Пределы действуют на компоненты координат.
    Ball {
        rotation: [CoordinateLimit; 3],
    },
    /// Смещение в неподвижном кадре соединения, м, затем вектор поворота, рад.
    Free {
        translation: [CoordinateLimit; 3],
        rotation: [CoordinateLimit; 3],
    },
}

impl Joint {
    fn limits(&self) -> impl Iterator<Item = CoordinateLimit> + '_ {
        let (first, second): (&[CoordinateLimit], &[CoordinateLimit]) = match self {
            Self::Fixed => (&[], &[]),
            Self::Hinge { limit, .. } | Self::Slide { limit, .. } => {
                (std::slice::from_ref(limit), &[])
            }
            Self::Ball { rotation } => (rotation, &[]),
            Self::Free {
                translation,
                rotation,
            } => (translation, rotation),
        };
        first.iter().chain(second).copied()
    }

    fn motion(&self, q: &[f64]) -> Result<Pose> {
        match *self {
            Self::Fixed => Ok(Pose::IDENTITY),
            Self::Hinge { axis, .. } => Pose::rotation(axis, q[0]),
            Self::Slide { axis, .. } => Pose::new(scale(axis, q[0]), Pose::IDENTITY.quaternion()),
            Self::Ball { .. } => Pose::from_rotation_vector([0.0; 3], [q[0], q[1], q[2]]),
            Self::Free { .. } => Pose::from_rotation_vector([q[0], q[1], q[2]], [q[3], q[4], q[5]]),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Link {
    pub name: String,
    pub parent: Option<LinkId>,
    pub parent_to_joint: Pose,
    pub joint: Joint,
    pub joint_to_link: Pose,
}

#[derive(Clone, Copy, Debug)]
pub enum Frame {
    World,
    Fixed(Pose),
    Link(LinkId),
    Site(Site),
}

#[derive(Clone, Copy, Debug)]
pub struct Site {
    pub link: LinkId,
    pub offset: Pose,
}

#[derive(Clone, Copy, Debug)]
pub struct TaskFrame {
    pub site: Site,
    pub reference: Frame,
}

#[derive(Clone, Debug)]
pub struct Tree {
    links: Vec<Link>,
    order: Vec<usize>,
    starts: Vec<usize>,
    limits: Vec<CoordinateLimit>,
}

struct TaskPaths {
    /// Номера от общего предка к концам ветвей. Предок в пути не входит.
    site: Vec<LinkId>,
    reference: Vec<LinkId>,
    reference_offset: Pose,
}

impl TaskPaths {
    fn links(&self) -> impl Iterator<Item = LinkId> + '_ {
        self.site.iter().chain(&self.reference).copied()
    }
}

struct BranchPose {
    end: Pose,
    joints: Vec<(LinkId, Pose)>,
}

#[derive(Clone, Debug)]
pub struct Jacobian {
    /// Столбец: линейная скорость сайта и угловая скорость сайта относительно системы отсчёта.
    /// Скорости выражены в осях системы отсчёта. Столбцы соответствуют порядку координат.
    pub columns: Vec<[f64; 6]>,
}

impl Tree {
    pub fn new(mut links: Vec<Link>) -> Result<Self> {
        let mut children = vec![Vec::new(); links.len()];
        let mut roots = VecDeque::new();
        let mut seen = BTreeSet::new();
        let mut starts = Vec::new();
        let mut limits = Vec::new();
        let count = links.len();
        for (index, link) in links.iter_mut().enumerate() {
            if link.name.trim().is_empty() || !seen.insert(link.name.clone()) {
                return Err(KinematicError::Invalid(format!(
                    "пустое или повторное имя звена {}",
                    link.name
                )));
            }
            if let Some(LinkId(parent)) = link.parent {
                if parent >= count || parent == index {
                    return Err(KinematicError::Invalid(format!(
                        "неверный родитель звена {}",
                        link.name
                    )));
                }
                children[parent].push(index);
            } else {
                roots.push_back(index);
            }
            if let Joint::Hinge { axis, .. } | Joint::Slide { axis, .. } = &mut link.joint {
                *axis = unit(*axis)?;
            }
            starts.push(limits.len());
            for limit in link.joint.limits() {
                if !limit.valid() {
                    return Err(KinematicError::Invalid(format!(
                        "неверный предел звена {}",
                        link.name
                    )));
                }
                limits.push(limit);
            }
        }
        if roots.len() != 1 {
            return Err(KinematicError::Invalid(
                "дерево должно иметь один корень".into(),
            ));
        }
        let mut order = Vec::new();
        while let Some(index) = roots.pop_front() {
            order.push(index);
            roots.extend(&children[index]);
        }
        if order.len() != count {
            return Err(KinematicError::Invalid("цикл в дереве".into()));
        }
        starts.push(limits.len());
        Ok(Self {
            links,
            order,
            starts,
            limits,
        })
    }

    pub fn coordinate_limits(&self) -> &[CoordinateLimit] {
        &self.limits
    }

    /// Координаты уложены в порядке переданных звеньев.
    pub fn coordinates_for(&self, id: LinkId) -> Result<std::ops::Range<usize>> {
        self.link(id)?;
        Ok(self.link_range(id))
    }

    pub fn validate_coordinates(&self, q: &[f64]) -> Result<()> {
        if q.len() != self.limits.len() || !finite(q) {
            return Err(KinematicError::Invalid(
                "неверное число или неконечные координаты".into(),
            ));
        }
        for (coordinate, (&value, limit)) in q.iter().zip(&self.limits).enumerate() {
            if !limit.contains(value) {
                return Err(KinematicError::JointLimit { coordinate });
            }
        }
        Ok(())
    }

    fn link(&self, id: LinkId) -> Result<&Link> {
        self.links
            .get(id.0)
            .ok_or_else(|| KinematicError::Invalid(format!("нет звена {}", id.0)))
    }

    fn link_range(&self, id: LinkId) -> std::ops::Range<usize> {
        self.starts[id.0]..self.starts[id.0 + 1]
    }

    fn link_poses(&self, q: &[f64], id: LinkId, parent: Pose) -> Result<(Pose, Pose)> {
        let link = &self.links[id.0];
        let joint = parent.compose(link.parent_to_joint)?;
        let motion = link.joint.motion(&q[self.link_range(id)])?;
        Ok((joint.compose(motion)?.compose(link.joint_to_link)?, joint))
    }

    /// Положение всех звеньев в мировой системе в порядке переданного описания.
    pub fn forward(&self, q: &[f64]) -> Result<Vec<Pose>> {
        self.validate_coordinates(q)?;
        let mut poses = vec![Pose::IDENTITY; self.links.len()];
        for &index in &self.order {
            let parent = self.links[index]
                .parent
                .map_or(Pose::IDENTITY, |id| poses[id.0]);
            poses[index] = self.link_poses(q, LinkId(index), parent)?.0;
        }
        Ok(poses)
    }

    fn ancestors(&self, id: LinkId) -> Result<Vec<LinkId>> {
        self.link(id)?;
        let mut path = Vec::new();
        let mut node = Some(id);
        while let Some(id) = node {
            path.push(id);
            node = self.links[id.0].parent;
        }
        Ok(path)
    }

    fn task_paths(&self, task: TaskFrame) -> Result<TaskPaths> {
        let mut site = self.ancestors(task.site.link)?;
        let (reference_id, reference_offset) = match task.reference {
            Frame::World => (None, Pose::IDENTITY),
            Frame::Fixed(pose) => (None, pose),
            Frame::Link(id) => (Some(id), Pose::IDENTITY),
            Frame::Site(site) => (Some(site.link), site.offset),
        };
        let mut reference = reference_id
            .map(|id| self.ancestors(id))
            .transpose()?
            .unwrap_or_default();
        // Общая часть дерева в относительной позе сокращается до вычисления в f64.
        while !reference.is_empty() && site.last() == reference.last() {
            site.pop();
            reference.pop();
        }
        site.reverse();
        reference.reverse();
        Ok(TaskPaths {
            site,
            reference,
            reference_offset,
        })
    }

    fn branch_pose(
        &self,
        q: &[f64],
        path: &[LinkId],
        offset: Pose,
        with_joints: bool,
    ) -> Result<BranchPose> {
        let mut end = Pose::IDENTITY;
        let mut joints = Vec::new();
        for &id in path {
            let (pose, joint) = self.link_poses(q, id, end)?;
            end = pose;
            if with_joints {
                joints.push((id, joint));
            }
        }
        Ok(BranchPose {
            end: end.compose(offset)?,
            joints,
        })
    }

    pub fn pose(&self, q: &[f64], task: TaskFrame) -> Result<Pose> {
        self.validate_coordinates(q)?;
        let paths = self.task_paths(task)?;
        let site = self
            .branch_pose(q, &paths.site, task.site.offset, false)?
            .end;
        let reference = self
            .branch_pose(q, &paths.reference, paths.reference_offset, false)?
            .end;
        relative_pose(site, reference)
    }

    pub fn jacobian(&self, q: &[f64], task: TaskFrame) -> Result<Jacobian> {
        Ok(self.pose_and_jacobian(q, task)?.1)
    }

    fn pose_and_jacobian(&self, q: &[f64], task: TaskFrame) -> Result<(Pose, Jacobian)> {
        self.validate_coordinates(q)?;
        let paths = self.task_paths(task)?;
        let site = self.branch_pose(q, &paths.site, task.site.offset, true)?;
        let reference = self.branch_pose(q, &paths.reference, paths.reference_offset, true)?;
        let mut columns = vec![[0.0; 6]; q.len()];
        self.branch_columns(q, &site.joints, site.end.translation(), &mut columns)?;
        // Столбцы родителей системы отсчёта берём в точке сайта. Это учитывает перенос её начала
        // и вращение осей. Пути раздельны, поэтому запись не затрагивает столбцы ветви сайта.
        self.branch_columns(q, &reference.joints, site.end.translation(), &mut columns)?;
        let inverse = orientation_inverse(reference.end)?;
        for &id in &paths.reference {
            for column in &mut columns[self.link_range(id)] {
                *column = column.map(|v| -v);
            }
        }
        for &id in paths.site.iter().chain(&paths.reference) {
            for column in &mut columns[self.link_range(id)] {
                let (v, w) = split(*column);
                *column = twist(inverse.rotate(v), inverse.rotate(w));
            }
        }
        if !columns.iter().all(|column| finite(column)) {
            return Err(KinematicError::Numerical("переполнение якобиана".into()));
        }
        Ok((
            relative_pose(site.end, reference.end)?,
            Jacobian { columns },
        ))
    }

    fn branch_columns(
        &self,
        q: &[f64],
        joints: &[(LinkId, Pose)],
        point: Vec3,
        columns: &mut [[f64; 6]],
    ) -> Result<()> {
        for &(LinkId(index), joint) in joints {
            let link = &self.links[index];
            let start = self.starts[index];
            let linear = |axis| twist(joint.rotate(axis), [0.0; 3]);
            let angular = |axis, center| {
                let w = joint.rotate(axis);
                twist(cross(w, sub(point, center)), w)
            };
            match link.joint {
                Joint::Fixed => {}
                Joint::Hinge { axis, .. } => columns[start] = angular(axis, joint.translation()),
                Joint::Slide { axis, .. } => columns[start] = linear(axis),
                Joint::Ball { .. } => {
                    let axes = rotation_vector_axes([q[start], q[start + 1], q[start + 2]])?;
                    for (column, axis) in columns[start..start + 3].iter_mut().zip(axes) {
                        *column = angular(axis, joint.translation());
                    }
                }
                Joint::Free { .. } => {
                    let [x, y, z, rx, ry, rz] = q[start..start + 6] else {
                        unreachable!("раскладка проверена")
                    };
                    let center = add(joint.translation(), joint.rotate([x, y, z]));
                    let axes = rotation_vector_axes([rx, ry, rz])?;
                    for (column, axis) in columns[start..start + 3].iter_mut().zip(AXES) {
                        *column = linear(axis);
                    }
                    for (column, axis) in columns[start + 3..start + 6].iter_mut().zip(axes) {
                        *column = angular(axis, center);
                    }
                }
            }
        }
        Ok(())
    }
}

fn orientation_inverse(pose: Pose) -> Result<Pose> {
    Pose::new([0.0; 3], pose.quaternion())?.inverse()
}

fn relative_pose(site: Pose, reference: Pose) -> Result<Pose> {
    let inverse = orientation_inverse(reference)?;
    let displacement = sub(site.translation(), reference.translation());
    Pose::new(
        inverse.rotate(displacement),
        inverse
            .compose(Pose::new([0.0; 3], site.quaternion())?)?
            .quaternion(),
    )
}

fn split([x, y, z, a, b, c]: [f64; 6]) -> (Vec3, Vec3) {
    ([x, y, z], [a, b, c])
}
fn twist([x, y, z]: Vec3, [a, b, c]: Vec3) -> [f64; 6] {
    [x, y, z, a, b, c]
}
