//! Модель механизма: дерево тел, соединённых шарнирами (как `<body>`/`<joint>` в MJCF MuJoCo
//! или `link`/`joint` в URDF). Мир — неподвижный корень, у тела без родителя родитель — мир.
//!
//! Система координат движка: Z вверх, X вперёд, Y влево (как в MuJoCo и Drake).

use std::ops::Range;

use nalgebra::{DVector, Matrix6xX, Rotation3, Unit, UnitQuaternion};

use crate::spatial::{Inertia, Mat3, SVec, Vec3, Xform, assert_finite};

/// Стандартное ускорение свободного падения, м/с² (CGPM, 1901).
pub const G_STANDARD: f64 = 9.80665;

/// Тип шарнира.
#[derive(Clone, Debug, PartialEq)]
pub enum Joint {
    /// Жёсткая заделка: 0 степеней свободы.
    Fixed,
    /// Вращательный вокруг единичной оси (в кадре шарнира). q — угол, рад.
    Revolute { axis: Vec3 },
    /// Поступательный вдоль единичной оси. q — смещение, м.
    Prismatic { axis: Vec3 },
    /// Шаровое соединение. q = [w, x, y, z], v = [ωx, ωy, ωz] в осях ребёнка, рад/с.
    Ball,
    /// Свободное тело (6 степеней свободы). q = [x, y, z, qw, qx, qy, qz],
    /// v = [ω; v] в осях самого тела.
    Free,
}

impl Joint {
    pub fn revolute(axis: Vec3) -> Self {
        Joint::Revolute {
            axis: joint_axis(axis),
        }
    }

    pub fn prismatic(axis: Vec3) -> Self {
        Joint::Prismatic {
            axis: joint_axis(axis),
        }
    }

    pub fn nq(&self) -> usize {
        match self {
            Joint::Fixed => 0,
            Joint::Revolute { .. } | Joint::Prismatic { .. } => 1,
            Joint::Ball => 4,
            Joint::Free => 7,
        }
    }

    pub fn nv(&self) -> usize {
        match self {
            Joint::Fixed => 0,
            Joint::Revolute { .. } | Joint::Prismatic { .. } => 1,
            Joint::Ball => 3,
            Joint::Free => 6,
        }
    }

    pub(crate) fn quaternion_offset(&self) -> Option<usize> {
        match self {
            Joint::Ball => Some(0),
            Joint::Free => Some(3),
            _ => None,
        }
    }

    /// Подпространство движения S (6 × nv) в кадре ребёнка. Для всех поддерживаемых
    /// шарниров оно постоянно, поэтому c_J = 0 (Featherstone, табл. 4.1).
    pub fn motion_subspace(&self) -> Matrix6xX<f64> {
        match self {
            Joint::Fixed => Matrix6xX::zeros(0),
            Joint::Revolute { axis } => {
                assert_finite(axis.as_slice(), "ось шарнира");
                let mut s = Matrix6xX::zeros(1);
                s[(0, 0)] = axis.x;
                s[(1, 0)] = axis.y;
                s[(2, 0)] = axis.z;
                s
            }
            Joint::Prismatic { axis } => {
                assert_finite(axis.as_slice(), "ось шарнира");
                let mut s = Matrix6xX::zeros(1);
                s[(3, 0)] = axis.x;
                s[(4, 0)] = axis.y;
                s[(5, 0)] = axis.z;
                s
            }
            Joint::Ball => Matrix6xX::identity(3),
            Joint::Free => Matrix6xX::identity(6),
        }
    }

    /// Преобразование из кадра шарнира в кадр ребёнка при координатах `q`.
    /// Неверная размерность, неконечные координаты и нулевой кватернион приводят к панике.
    pub fn transform(&self, q: &[f64]) -> Xform {
        assert_eq!(q.len(), self.nq(), "размерность координат шарнира");
        assert_finite(q, "координаты шарнира");
        match self {
            Joint::Fixed => Xform::IDENTITY,
            Joint::Revolute { axis } => {
                assert_finite(axis.as_slice(), "ось шарнира");
                let r = Rotation3::from_axis_angle(&Unit::new_unchecked(*axis), q[0]);
                Xform::from_pose(r.matrix(), &Vec3::zeros())
            }
            Joint::Prismatic { axis } => {
                assert_finite(axis.as_slice(), "ось шарнира");
                Xform::translation(axis * q[0])
            }
            Joint::Ball => {
                Xform::from_pose(joint_quat(q).to_rotation_matrix().matrix(), &Vec3::zeros())
            }
            Joint::Free => {
                let quat = free_quat(q);
                Xform::from_pose(
                    quat.to_rotation_matrix().matrix(),
                    &Vec3::new(q[0], q[1], q[2]),
                )
            }
        }
    }
}

fn joint_axis(axis: Vec3) -> Vec3 {
    let (unit, norm) = Unit::new_and_get(axis);
    assert!(norm.is_finite() && norm > 0.0, "недопустимая ось шарнира");
    unit.into_inner()
}

/// Кватернион ориентации свободного тела из его q (нормируется).
pub fn free_quat(q: &[f64]) -> UnitQuaternion<f64> {
    assert_eq!(
        q.len(),
        Joint::Free.nq(),
        "размерность координат свободного тела"
    );
    assert_finite(q, "координаты свободного тела");
    joint_quat(&q[3..7])
}

fn joint_quat(q: &[f64]) -> UnitQuaternion<f64> {
    let quat = nalgebra::Quaternion::new(q[0], q[1], q[2], q[3]);
    let (unit, n) = UnitQuaternion::new_and_get(quat);
    assert!(n.is_finite() && n > 0.0, "недопустимая норма кватерниона");
    unit
}

pub(crate) fn quaternion_norm(q: &[f64]) -> f64 {
    let n = (q[0].powi(2) + q[1].powi(2) + q[2].powi(2) + q[3].powi(2)).sqrt();
    assert!(n.is_finite() && n > 0.0, "недопустимая норма кватерниона");
    n
}

/// Поза: ориентация + положение (в родительском кадре).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub rot: UnitQuaternion<f64>,
    pub pos: Vec3,
}

impl Default for Pose {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Pose {
    pub const IDENTITY: Pose = Pose {
        rot: UnitQuaternion::new_unchecked(nalgebra::Quaternion::new(1.0, 0.0, 0.0, 0.0)),
        pos: Vec3::new(0.0, 0.0, 0.0),
    };

    pub fn at(pos: Vec3) -> Self {
        Self {
            rot: UnitQuaternion::identity(),
            pos,
        }
    }

    pub fn new(rot: UnitQuaternion<f64>, pos: Vec3) -> Self {
        Self { rot, pos }
    }

    /// Поза, у которой локальная ось z смотрит вдоль `dir`.
    pub fn z_along(pos: Vec3, dir: Vec3) -> Self {
        let rot = UnitQuaternion::rotation_between(&Vec3::z(), &dir).unwrap_or_else(|| {
            UnitQuaternion::from_axis_angle(&Vec3::x_axis(), std::f64::consts::PI)
        });
        Self { rot, pos }
    }

    pub fn compose(&self, child: &Pose) -> Pose {
        Pose {
            rot: self.rot * child.rot,
            pos: self.pos + self.rot * child.pos,
        }
    }

    pub fn xform(&self) -> Xform {
        Xform::from_pose(self.rot.to_rotation_matrix().matrix(), &self.pos)
    }

    pub fn from_xform(x: &Xform) -> Pose {
        let r = Rotation3::from_matrix_unchecked(x.orientation());
        Pose {
            rot: UnitQuaternion::from_rotation_matrix(&r),
            pos: x.pos,
        }
    }

    pub fn transform_point(&self, p: &Vec3) -> Vec3 {
        self.pos + self.rot * p
    }
}

/// Геометрия для показа (на физику не влияет — масса задаётся `Inertia`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    Sphere {
        radius: f64,
    },
    /// Капсула вдоль локальной z: `length` — длина цилиндрической части.
    Capsule {
        radius: f64,
        length: f64,
    },
    /// Цилиндр вдоль локальной z.
    Cylinder {
        radius: f64,
        length: f64,
    },
    Box {
        half: Vec3,
    },
    /// Треугольная сетка из библиотеки сценария (`Scenario::mesh_data`), в своих координатах.
    Mesh {
        id: u32,
    },
    /// Эллипсоид с полуосями по локальным x, y, z.
    Ellipsoid {
        radii: Vec3,
    },
    /// Трубка вдоль локальной z, центр — в начале координат (линия действия мышцы).
    /// Размеры меняются каждый кадр без пересоздания сетки.
    Tube {
        radius: f64,
        length: f64,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Visual {
    pub shape: Shape,
    pub pose: Pose,
    pub color: [f32; 4],
}

impl Visual {
    pub fn new(shape: Shape, pose: Pose, color: [f32; 4]) -> Self {
        Self { shape, pose, color }
    }
}

#[derive(Clone, Debug)]
pub struct Body {
    pub name: String,
    pub parent: Option<usize>,
    pub joint: Joint,
    /// Кадр шарнира относительно кадра родителя (неподвижная часть).
    pub placement: Xform,
    pub inertia: Inertia,
    /// Вязкое трение в шарнире, Н·м·с/рад (или Н·с/м).
    pub damping: f64,
    /// Жёсткость пружины в шарнире, Н·м/рад (или Н/м), и её нейтраль.
    pub stiffness: f64,
    pub spring_rest: f64,
    /// Инерция ротора в шарнире (как `armature` в MuJoCo): добавка к диагонали матрицы масс,
    /// кг·м² (или кг для поступательного). Для мотора с редуктором: I_ротора·i².
    pub armature: f64,
    pub visuals: Vec<Visual>,
}

impl Body {
    pub fn new(
        name: &str,
        parent: Option<usize>,
        joint: Joint,
        placement: Xform,
        inertia: Inertia,
    ) -> Self {
        Self {
            name: name.to_string(),
            parent,
            joint,
            placement,
            inertia,
            damping: 0.0,
            stiffness: 0.0,
            spring_rest: 0.0,
            armature: 0.0,
            visuals: Vec::new(),
        }
    }

    pub fn with_visual(mut self, v: Visual) -> Self {
        self.visuals.push(v);
        self
    }

    pub fn with_spring(mut self, stiffness: f64, damping: f64) -> Self {
        self.stiffness = stiffness;
        self.damping = damping;
        self
    }

    pub(crate) fn validate_passive_parameters(&self) {
        assert_finite(
            &[self.damping, self.stiffness, self.spring_rest],
            "пассивные параметры соединения",
        );
        if matches!(self.joint, Joint::Ball) {
            assert!(
                self.stiffness == 0.0 && self.spring_rest == 0.0,
                "скалярная пружина неприменима к шаровому соединению «{}»",
                self.name
            );
        }
    }
}

#[derive(Clone, Debug)]
pub struct Model {
    pub bodies: Vec<Body>,
    pub gravity: Vec3,
    /// Неподвижная обстановка (стойки, манекен) — только для показа.
    pub world_visuals: Vec<Visual>,
    q_start: Vec<usize>,
    v_start: Vec<usize>,
    subspace: Vec<Matrix6xX<f64>>,
    nq: usize,
    nv: usize,
}

impl Model {
    pub fn new(gravity: Vec3) -> Self {
        Self {
            bodies: Vec::new(),
            gravity,
            world_visuals: Vec::new(),
            q_start: Vec::new(),
            v_start: Vec::new(),
            subspace: Vec::new(),
            nq: 0,
            nv: 0,
        }
    }

    /// Земная гравитация вдоль −Z.
    pub fn earth() -> Self {
        Self::new(Vec3::new(0.0, 0.0, -G_STANDARD))
    }

    /// Добавить тело; родитель должен быть добавлен раньше. Возвращает индекс тела.
    pub fn add_body(&mut self, body: Body) -> usize {
        let idx = self.bodies.len();
        if let Some(p) = body.parent {
            assert!(
                p < idx,
                "родитель тела «{}» должен быть добавлен раньше",
                body.name
            );
        }
        self.q_start.push(self.nq);
        self.v_start.push(self.nv);
        self.nq += body.joint.nq();
        self.nv += body.joint.nv();
        self.subspace.push(body.joint.motion_subspace());
        self.bodies.push(body);
        idx
    }

    pub fn nq(&self) -> usize {
        self.nq
    }

    pub fn nv(&self) -> usize {
        self.nv
    }

    pub fn q_range(&self, body: usize) -> Range<usize> {
        let s = self.q_start[body];
        s..s + self.bodies[body].joint.nq()
    }

    pub fn v_range(&self, body: usize) -> Range<usize> {
        let s = self.v_start[body];
        s..s + self.bodies[body].joint.nv()
    }

    pub fn subspace(&self, body: usize) -> &Matrix6xX<f64> {
        &self.subspace[body]
    }

    pub fn body_index(&self, name: &str) -> Option<usize> {
        self.bodies.iter().position(|b| b.name == name)
    }

    /// Нулевая конфигурация (единичные кватернионы у шаровых и свободных соединений).
    pub fn neutral_q(&self) -> DVector<f64> {
        let mut q = DVector::zeros(self.nq);
        for (i, b) in self.bodies.iter().enumerate() {
            if let Some(offset) = b.joint.quaternion_offset() {
                q[self.q_start[i] + offset] = 1.0;
            }
        }
        q
    }

    pub fn total_mass(&self) -> f64 {
        self.bodies.iter().map(|b| b.inertia.mass).sum()
    }

    /// Нулевая пространственная сила для каждого тела (удобно для `f_ext`).
    pub fn zero_forces(&self) -> Vec<SVec> {
        vec![SVec::zeros(); self.bodies.len()]
    }

    pub fn identity_rot() -> Mat3 {
        Mat3::identity()
    }
}
