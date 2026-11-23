//! Геометрия и инерция тела на модели MS-Human-700 (всё тело, 700 мышц).
//!
//! Обобщённые координаты цепочки: три степени плеча, локоть, пронация и три степени L5/S1.
//! Лопаточно-плечевой ритм задан связями модели, поэтому плечо мышцы считается как −dℓ/dθ при
//! соблюдённых связях, а обобщённая сила — из виртуальной работы:
//!   τₖ = Σ_b [ m_b (a_b − g)·∂c_b/∂θₖ + Ḣ_b·∂ψ_b/∂θₖ ].
//! Так момент согласован с ритмом лопатки и с плечами мышц.
//!
//! Допущения модели:
//! * таз в MuJoCo неподвижен, наклон таза учитывается поворотом мира вокруг оси тазобедренных
//!   суставов; подвижный отдел позвоночника в модели один — L5/S1, выше корпус жёсткий;
//! * масса и длины масштабируются единым коэффициентом (без отдельных пропорций сегментов);
//! * груз — точечная масса на продолжении предплечья.

use nalgebra::{Matrix3, Vector3};
use sim_mujoco::muscle::MuscleModel;
use sim_mujoco::{JointCoupling, MuscleInfo, Sim, models};

pub const NC: usize = 8;
pub const C_PLANE: usize = 0;
pub const C_ELEV: usize = 1;
pub const C_ROT: usize = 2;
pub const C_ELBOW: usize = 3;
pub const C_PROSUP: usize = 4;
pub const C_L5_FE: usize = 5;
pub const C_L5_LB: usize = 6;
pub const C_L5_AR: usize = 7;

/// Участки, для которых считается сила в суставе.
pub const LINK_ELBOW: usize = 0;
pub const LINK_SHOULDER: usize = 1;
pub const LINK_SPINE: usize = 2;

const COORD_JOINTS: [&str; NC] = [
    "elv_angle_r",
    "shoulder_elv_r",
    "shoulder_rot_r",
    "elbow_flexion_r",
    "pro_sup_r",
    "L5_S1_FE",
    "L5_S1_LB",
    "L5_S1_AR",
];
const LEFT_ARM_JOINTS: [&str; 5] = [
    "elv_angle_l",
    "shoulder_elv_l",
    "shoulder_rot_l",
    "elbow_flexion_l",
    "pro_sup_l",
];

/// Численный шаг по углу, рад. Проверяется сравнением с якобианом MuJoCo.
const FD_ANGLE: f64 = 1e-5;

/// Поза человека, радианы.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct BodyPose {
    /// Плоскость подъёма плеча: 0 — отведение, 90° — вперёд.
    pub shoulder_plane: f64,
    pub shoulder_elevation: f64,
    pub shoulder_rotation: f64,
    pub elbow_flexion: f64,
    pub forearm_rotation: f64,
    /// Наклон корпуса от вертикали вперёд (таз + поясница).
    pub trunk_inclination: f64,
    /// Часть наклона, приходящаяся на сгибание в L5/S1.
    pub lumbar_flexion: f64,
    /// Боковой наклон и осевое вращение по положительным осям L5/S1 в MS-Human-700.
    pub lumbar_lateral_bend: f64,
    pub lumbar_rotation: f64,
}

impl BodyPose {
    pub fn pelvis_tilt(&self) -> f64 {
        self.trunk_inclination - self.lumbar_flexion
    }

    pub(crate) fn coords(&self) -> [f64; NC] {
        let mut q = [0.0; NC];
        q[C_PLANE] = self.shoulder_plane;
        q[C_ELEV] = self.shoulder_elevation;
        q[C_ROT] = self.shoulder_rotation;
        q[C_ELBOW] = self.elbow_flexion;
        q[C_PROSUP] = self.forearm_rotation;
        // Положительный L5_S1_FE в модели — разгибание, поэтому сгибание идёт со знаком минус.
        q[C_L5_FE] = -self.lumbar_flexion;
        q[C_L5_LB] = self.lumbar_lateral_bend;
        q[C_L5_AR] = self.lumbar_rotation;
        q
    }
}

/// Кинематика кисти: как точка груза зависит от координат.
#[derive(Clone, Copy, Debug)]
pub struct HandKin {
    /// Доля груза на этой руке.
    pub share: f64,
    pub point: [f64; 3],
    pub dir: [f64; 3],
    pub dpoint: [[f64; 3]; NC],
    pub ddir: [[f64; 3]; NC],
    pub accel: [f64; 3],
    pub dir_accel: [f64; 3],
}

/// Мышца в данной позе.
#[derive(Clone, Debug)]
pub struct MuscleRow {
    pub actuator: usize,
    pub name: String,
    /// Плечо по каждой координате, м (знак — по положительному направлению координаты).
    pub r: [f64; NC],
    /// Активная сила при a = 1 с учётом длины и скорости, Н.
    pub active: f64,
    /// То же без учёта скорости, Н.
    pub active_iso: f64,
    /// Пассивная сила, Н.
    pub passive: f64,
    /// Единичная тяга на участок дистальнее сустава (локоть, плечо, поясница).
    pub pull: [[f64; 3]; 3],
}

/// Внешние и инерционные силы на участок (без мышц).
#[derive(Clone, Copy, Debug, Default)]
pub struct LinkForces {
    /// Σ m·(−g), Н.
    pub body_gravity: [f64; 3],
    /// Σ m·a, Н.
    pub body_inertia: [f64; 3],
    /// Какая доля груза лежит на этом участке.
    pub load_share: f64,
}

/// Всё, что нужно для расчёта нагрузки в одной точке движения.
#[derive(Clone, Debug)]
pub struct GeometrySample {
    pub pose: BodyPose,
    pub tau_gravity: [f64; NC],
    pub tau_inertia: [f64; NC],
    pub hands: [HandKin; 2],
    /// Ускорение свободного падения в системе модели, м/с².
    pub gravity: [f64; 3],
    pub muscles: Vec<MuscleRow>,
    pub links: [LinkForces; 3],
    /// Ось сжатия L5/S1 (вдоль позвоночника), единичная.
    pub spine_axis: [f64; 3],
}

impl GeometrySample {
    pub(crate) fn is_finite(&self) -> bool {
        self.tau_gravity
            .iter()
            .chain(&self.tau_inertia)
            .chain(&self.gravity)
            .chain(&self.spine_axis)
            .all(|x| x.is_finite())
            && self.links.iter().all(|l| {
                l.body_gravity
                    .iter()
                    .chain(&l.body_inertia)
                    .all(|x| x.is_finite())
            })
            && self.hands.iter().all(|h| {
                h.point
                    .iter()
                    .chain(&h.dir)
                    .chain(&h.accel)
                    .chain(&h.dir_accel)
                    .chain(h.dpoint.iter().flatten())
                    .chain(h.ddir.iter().flatten())
                    .all(|x| x.is_finite())
            })
            && self.muscles.iter().all(|m| {
                m.r.iter()
                    .chain(m.pull.iter().flatten())
                    .all(|x| x.is_finite())
                    && [m.active, m.active_iso, m.passive]
                        .iter()
                        .all(|x| x.is_finite() && *x >= 0.0)
            })
    }
}

/// Срез цепочки для интегрирования при отказе.
#[derive(Clone, Debug)]
pub struct ChainProbe {
    pub gravity: [f64; 3],
    /// Момент, нужный для удержания тела против тяжести, в единицах модели.
    pub tau_gravity: Vec<f64>,
    pub hand: [[f64; 3]; 2],
    pub dir: [[f64; 3]; 2],
    pub dpoint: [Vec<[f64; 3]>; 2],
    pub ddir: [Vec<[f64; 3]>; 2],
    /// Σ m·(−g)·c по телам, в единицах модели.
    pub potential_body: f64,
}

#[derive(Clone)]
struct Snapshot {
    com: Vec<[f64; 3]>,
    rot: Vec<Matrix3<f64>>,
    hand: [[f64; 3]; 2],
    dir: [[f64; 3]; 2],
    lengths: Vec<f64>,
}

/// Подготовленная модель тела.
pub struct BodyRig {
    sim: Sim,
    couplings: Vec<JointCoupling>,
    coord_qadr: [usize; NC],
    coord_dof: [usize; NC],
    left_qadr: [usize; 5],
    elbow_joints: [usize; 2],
    spine_axis_joint: usize,
    pelvis_range: [f64; 2],
    hand_bodies: [usize; 2],
    masses: Vec<f64>,
    inertias: Vec<[f64; 3]>,
    subtrees: [Vec<bool>; 3],
    hip_center: [f64; 3],
    muscles: Vec<MuscleInfo>,
    models: Vec<MuscleModel>,
    nbody: usize,
}

fn v(a: [f64; 3]) -> Vector3<f64> {
    Vector3::new(a[0], a[1], a[2])
}
fn a3(x: Vector3<f64>) -> [f64; 3] {
    [x[0], x[1], x[2]]
}

/// Поворот вокруг оси Y: положительный угол наклоняет +Z к +X (корпус вперёд).
fn rot_y(phi: f64) -> Matrix3<f64> {
    let (s, c) = phi.sin_cos();
    Matrix3::new(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c)
}

fn rotation_vector(rel: &Matrix3<f64>) -> Vector3<f64> {
    // Для малых поворотов след матрицы может округлиться выше 3.
    // Вектор кососимметричной части равен sin(θ)·ось, atan2 не требует округления acos.
    let skew = 0.5
        * Vector3::new(
            rel[(2, 1)] - rel[(1, 2)],
            rel[(0, 2)] - rel[(2, 0)],
            rel[(1, 0)] - rel[(0, 1)],
        );
    let sin = skew.norm();
    let cos = 0.5 * (rel.trace() - 1.0);
    if sin == 0.0 {
        if cos > 0.0 {
            skew
        } else {
            Vector3::repeat(f64::NAN)
        }
    } else {
        skew * (sin.atan2(cos) / sin)
    }
}

impl BodyRig {
    /// Загрузить MS-Human-700 и найти суставы цепочки.
    pub fn load() -> Result<Self, String> {
        let sim = Sim::load(&models::ms_human_700())?;
        Self::new(sim)
    }

    fn new(sim: Sim) -> Result<Self, String> {
        let joints = sim.joints();
        let find = |n: &str| {
            joints
                .iter()
                .find(|j| j.name == n)
                .ok_or_else(|| format!("в модели нет сустава {n}"))
        };
        let mut coord_qadr = [0; NC];
        let mut coord_dof = [0; NC];
        for (k, name) in COORD_JOINTS.iter().enumerate() {
            let j = find(name)?;
            coord_qadr[k] = j.qpos_adr;
            coord_dof[k] = j.dof_adr;
        }
        let mut left_qadr = [0; 5];
        for (k, name) in LEFT_ARM_JOINTS.iter().enumerate() {
            left_qadr[k] = find(name)?.qpos_adr;
        }
        let elbow_joints = [find("elbow_flexion_r")?.id, find("elbow_flexion_l")?.id];
        let spine_axis_joint = find("L5_S1_AR")?.id;
        let pelvis_range = find("pelvis_tilt")?.range;

        let bodies = sim.bodies();
        let body = |n: &str| {
            bodies
                .iter()
                .position(|(b, _)| b == n)
                .ok_or_else(|| format!("в модели нет тела {n}"))
        };
        let hand_bodies = [body("hand_r")?, body("hand_l")?];
        let subtrees = [
            sim.subtree_mask(body("ulna_r")?),
            sim.subtree_mask(body("humerus_r")?),
            sim.subtree_mask(body("lumbar5")?),
        ];
        let (fr, _) = sim.body_pose(body("femur_r")?);
        let (fl, _) = sim.body_pose(body("femur_l")?);
        let hip_center = [
            0.5 * (fr[0] + fl[0]),
            0.5 * (fr[1] + fl[1]),
            0.5 * (fr[2] + fl[2]),
        ];

        // Связи: только те, что ведутся суставами руки (лопатка, ключица, плечо).
        let driving: Vec<usize> = [
            "elv_angle_r",
            "shoulder_elv_r",
            "elv_angle_l",
            "shoulder_elv_l",
        ]
        .iter()
        .map(|n| find(n).map(|j| j.qpos_adr))
        .collect::<Result<_, _>>()?;
        let couplings = sim
            .joint_couplings()
            .into_iter()
            .filter(|c| c.q2.is_some_and(|q| driving.contains(&q)))
            .collect();

        let masses: Vec<f64> = bodies.iter().map(|(_, m)| *m).collect();
        let nbody = sim.nbody();
        let inertias = (0..nbody).map(|b| sim.body_inertia(b)).collect();
        let muscles = sim.muscles();
        let models = muscles
            .iter()
            .map(|m| sim.muscle_model(m.actuator))
            .collect();
        Ok(Self {
            sim,
            couplings,
            coord_qadr,
            coord_dof,
            left_qadr,
            elbow_joints,
            spine_axis_joint,
            pelvis_range,
            hand_bodies,
            masses,
            inertias,
            subtrees,
            hip_center,
            muscles,
            models,
            nbody,
        })
    }

    pub fn sim(&self) -> &Sim {
        &self.sim
    }

    /// Масса тела модели, кг.
    pub fn body_mass(&self) -> f64 {
        self.sim.subtree_mass(self.sim.human_root())
    }

    /// Диапазон координаты в модели, рад.
    pub fn coord_range(&self, k: usize) -> [f64; 2] {
        let dof = self.coord_dof[k];
        self.sim
            .joints()
            .iter()
            .find(|j| j.dof_adr == dof)
            .map(|j| j.range)
            .unwrap_or([f64::NEG_INFINITY, f64::INFINITY])
    }

    pub fn pelvis_range(&self) -> [f64; 2] {
        self.pelvis_range
    }

    /// Поставить позу с приращениями по координатам и пересчитать модель.
    pub fn apply(&mut self, pose: &BodyPose, delta: &[f64; NC], two_handed: bool) {
        let base = pose.coords();
        let q0: Vec<f64> = self.sim.qpos0().to_vec();
        {
            let q = self.sim.qpos_mut();
            q.copy_from_slice(&q0);
        }
        for k in 0..NC {
            self.sim.qpos_mut()[self.coord_qadr[k]] = base[k] + delta[k];
        }
        if two_handed {
            for k in 0..5 {
                self.sim.qpos_mut()[self.left_qadr[k]] = base[k] + delta[k];
            }
        }
        let couplings = std::mem::take(&mut self.couplings);
        self.sim.apply_joint_couplings(&couplings);
        self.couplings = couplings;
        self.sim.qvel_mut().fill(0.0);
        self.sim.forward();
    }

    fn ximat(&self) -> Vec<Matrix3<f64>> {
        // SAFETY: ximat — массив 9·nbody чисел в живых данных MuJoCo, которыми владеет Sim.
        let raw = unsafe { std::slice::from_raw_parts(self.sim.data().ximat, 9 * self.nbody) };
        (0..self.nbody)
            .map(|b| {
                let m = &raw[9 * b..9 * b + 9];
                Matrix3::new(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8])
            })
            .collect()
    }

    fn snapshot(&self) -> Snapshot {
        let com = (0..self.nbody).map(|b| self.sim.body_com(b)).collect();
        let mut hand = [[0.0; 3]; 2];
        let mut dir = [[0.0; 3]; 2];
        for side in 0..2 {
            let p = self.sim.body_com(self.hand_bodies[side]);
            let e = self.sim.joint_anchor(self.elbow_joints[side]);
            let u = (v(p) - v(e)).normalize();
            hand[side] = p;
            dir[side] = a3(u);
        }
        let lengths = self
            .muscles
            .iter()
            .map(|m| self.sim.actuator_length(m.actuator))
            .collect();
        Snapshot {
            com,
            rot: self.ximat(),
            hand,
            dir,
            lengths,
        }
    }

    /// Геометрия в покое.
    pub fn sample_static(&mut self, pose: &BodyPose, two_handed: bool) -> GeometrySample {
        self.sample(None, pose, None, 0.0, two_handed)
    }

    /// Геометрия и инерция в точке движения: позы в моменты t − dt, t, t + dt.
    pub fn sample_moving(
        &mut self,
        before: &BodyPose,
        pose: &BodyPose,
        after: &BodyPose,
        dt: f64,
        two_handed: bool,
    ) -> GeometrySample {
        self.sample(Some(before), pose, Some(after), dt, two_handed)
    }

    fn sample(
        &mut self,
        before: Option<&BodyPose>,
        pose: &BodyPose,
        after: Option<&BodyPose>,
        dt: f64,
        two_handed: bool,
    ) -> GeometrySample {
        // Частные производные по координатам.
        let mut dcom = vec![[[0.0; 3]; NC]; self.nbody];
        let mut dpsi = vec![[[0.0; 3]; NC]; self.nbody];
        let mut dhand = [[[0.0; 3]; NC]; 2];
        let mut ddir = [[[0.0; 3]; NC]; 2];
        let nm = self.muscles.len();
        let mut arms = vec![[0.0; NC]; nm];
        for k in 0..NC {
            let mut d = [0.0; NC];
            d[k] = FD_ANGLE;
            self.apply(pose, &d, two_handed);
            let plus = self.snapshot();
            d[k] = -FD_ANGLE;
            self.apply(pose, &d, two_handed);
            let minus = self.snapshot();
            let h2 = 2.0 * FD_ANGLE;
            for b in 0..self.nbody {
                dcom[b][k] = a3((v(plus.com[b]) - v(minus.com[b])) / h2);
                let rel = plus.rot[b] * minus.rot[b].transpose();
                dpsi[b][k] = a3(rotation_vector(&rel) / h2);
            }
            for side in 0..2 {
                dhand[side][k] = a3((v(plus.hand[side]) - v(minus.hand[side])) / h2);
                ddir[side][k] = a3((v(plus.dir[side]) - v(minus.dir[side])) / h2);
            }
            for j in 0..nm {
                arms[j][k] = -(plus.lengths[j] - minus.lengths[j]) / h2;
            }
        }

        // Движение: ускорения в мире по трём положениям.
        let tilt0 = pose.pelvis_tilt();
        let r0 = rot_y(tilt0);
        let world = |rig: &Self, p: &[f64; 3], rot: &Matrix3<f64>| -> Vector3<f64> {
            v(rig.hip_center) + rot * (v(*p) - v(rig.hip_center))
        };
        let g_world = v(self.sim.gravity());
        let g_model = r0.transpose() * g_world;

        let mut acc = vec![Vector3::zeros(); self.nbody];
        let mut hdot = vec![Vector3::zeros(); self.nbody];
        let mut hand_acc = [Vector3::zeros(); 2];
        let mut dir_acc = [Vector3::zeros(); 2];
        let mut velocity = vec![0.0; nm];
        let moving = before.is_some() && after.is_some() && dt > 0.0;
        if moving {
            let (pa, pb) = (before.unwrap(), after.unwrap());
            self.apply(pa, &[0.0; NC], two_handed);
            let s_m = self.snapshot();
            self.apply(pb, &[0.0; NC], two_handed);
            let s_p = self.snapshot();
            self.apply(pose, &[0.0; NC], two_handed);
            let s_0 = self.snapshot();
            let (rm, rp) = (rot_y(pa.pelvis_tilt()), rot_y(pb.pelvis_tilt()));
            for b in 0..self.nbody {
                let cw_m = world(self, &s_m.com[b], &rm);
                let cw_0 = world(self, &s_0.com[b], &r0);
                let cw_p = world(self, &s_p.com[b], &rp);
                let a_w = (cw_p - 2.0 * cw_0 + cw_m) / (dt * dt);
                acc[b] = r0.transpose() * a_w;
                // Угловая скорость на полушагах, момент импульса в мире.
                let (rb_m, rb_0, rb_p) = (rm * s_m.rot[b], r0 * s_0.rot[b], rp * s_p.rot[b]);
                let w_plus = rotation_vector(&(rb_p * rb_0.transpose())) / dt;
                let w_minus = rotation_vector(&(rb_0 * rb_m.transpose())) / dt;
                let omega = 0.5 * (w_plus + w_minus);
                let alpha = (w_plus - w_minus) / dt;
                let i = self.inertias[b];
                let iw = rb_0
                    * Matrix3::from_diagonal(&Vector3::new(i[0], i[1], i[2]))
                    * rb_0.transpose();
                let h = iw * alpha + omega.cross(&(iw * omega));
                hdot[b] = r0.transpose() * h;
            }
            for side in 0..2 {
                let (hm, h0, hp) = (
                    world(self, &s_m.hand[side], &rm),
                    world(self, &s_0.hand[side], &r0),
                    world(self, &s_p.hand[side], &rp),
                );
                hand_acc[side] = r0.transpose() * ((hp - 2.0 * h0 + hm) / (dt * dt));
                let (um, u0, up) = (
                    rm * v(s_m.dir[side]),
                    r0 * v(s_0.dir[side]),
                    rp * v(s_p.dir[side]),
                );
                dir_acc[side] = r0.transpose() * ((up - 2.0 * u0 + um) / (dt * dt));
            }
            for j in 0..nm {
                velocity[j] = (s_p.lengths[j] - s_m.lengths[j]) / (2.0 * dt);
            }
        } else {
            self.apply(pose, &[0.0; NC], two_handed);
        }

        // Состояние модели — базовая поза.
        let base = self.snapshot();
        let mut tau_gravity = [0.0; NC];
        let mut tau_inertia = [0.0; NC];
        for b in 0..self.nbody {
            let m = self.masses[b];
            if m <= 0.0 {
                continue;
            }
            for k in 0..NC {
                let dc = v(dcom[b][k]);
                tau_gravity[k] += m * (-g_model).dot(&dc);
                if moving {
                    tau_inertia[k] += m * acc[b].dot(&dc) + hdot[b].dot(&v(dpsi[b][k]));
                }
            }
        }

        let share = if two_handed { [0.5, 0.5] } else { [1.0, 0.0] };
        let hands = [0, 1].map(|side| HandKin {
            share: share[side],
            point: base.hand[side],
            dir: base.dir[side],
            dpoint: dhand[side],
            ddir: ddir[side],
            accel: a3(hand_acc[side]),
            dir_accel: a3(dir_acc[side]),
        });

        let mut links = [LinkForces::default(); 3];
        for (s, link) in links.iter_mut().enumerate() {
            let mut grav = Vector3::zeros();
            let mut iner = Vector3::zeros();
            for b in 0..self.nbody {
                if self.subtrees[s][b] {
                    grav += self.masses[b] * (-g_model);
                    iner += self.masses[b] * acc[b];
                }
            }
            link.body_gravity = a3(grav);
            link.body_inertia = a3(iner);
            link.load_share = (0..2)
                .filter(|&side| self.subtrees[s][self.hand_bodies[side]])
                .map(|side| share[side])
                .sum();
        }

        let mut muscles = Vec::new();
        for j in 0..nm {
            let info = &self.muscles[j];
            let model = &self.models[j];
            let len = base.lengths[j];
            let passive = -model.bias(len);
            let pull = self.muscle_pull(info.tendon);
            if arms[j].iter().all(|&r| r == 0.0)
                && (passive == 0.0 || pull.iter().flatten().all(|&x| x == 0.0))
            {
                continue;
            }
            muscles.push(MuscleRow {
                actuator: info.actuator,
                name: info.name.clone(),
                r: arms[j],
                active: -model.gain(len, velocity[j]),
                active_iso: -model.gain(len, 0.0),
                passive,
                pull,
            });
        }

        let spine_axis = self.sim.joint_axis(self.spine_axis_joint);
        GeometrySample {
            pose: *pose,
            tau_gravity,
            tau_inertia,
            hands,
            gravity: a3(g_model),
            muscles,
            links,
            spine_axis,
        }
    }

    /// Быстрый срез для интегрирования: тяжесть и якобианы кистей только по нужным координатам.
    pub fn probe(&mut self, pose: &BodyPose, coords: &[usize], two_handed: bool) -> ChainProbe {
        let g_model = rot_y(pose.pelvis_tilt()).transpose() * v(self.sim.gravity());
        let mut tau_gravity = vec![0.0; coords.len()];
        let mut dpoint = [vec![[0.0; 3]; coords.len()], vec![[0.0; 3]; coords.len()]];
        let mut ddir = [vec![[0.0; 3]; coords.len()], vec![[0.0; 3]; coords.len()]];
        for (p, &k) in coords.iter().enumerate() {
            let mut d = [0.0; NC];
            d[k] = FD_ANGLE;
            self.apply(pose, &d, two_handed);
            let plus = self.snapshot_light();
            d[k] = -FD_ANGLE;
            self.apply(pose, &d, two_handed);
            let minus = self.snapshot_light();
            let h2 = 2.0 * FD_ANGLE;
            for b in 0..self.nbody {
                let dc = (v(plus.0[b]) - v(minus.0[b])) / h2;
                tau_gravity[p] += self.masses[b] * (-g_model).dot(&dc);
            }
            for side in 0..2 {
                dpoint[side][p] = a3((v(plus.1[side]) - v(minus.1[side])) / h2);
                ddir[side][p] = a3((v(plus.2[side]) - v(minus.2[side])) / h2);
            }
        }
        self.apply(pose, &[0.0; NC], two_handed);
        let base = self.snapshot_light();
        let potential_body = (0..self.nbody)
            .map(|b| self.masses[b] * (-g_model).dot(&v(base.0[b])))
            .sum();
        ChainProbe {
            gravity: a3(g_model),
            tau_gravity,
            hand: base.1,
            dir: base.2,
            dpoint,
            ddir,
            potential_body,
        }
    }

    /// Положения кистей и направления предплечий в позе.
    pub fn hand_points(
        &mut self,
        pose: &BodyPose,
        two_handed: bool,
    ) -> ([[f64; 3]; 2], [[f64; 3]; 2]) {
        self.apply(pose, &[0.0; NC], two_handed);
        let s = self.snapshot_light();
        (s.1, s.2)
    }

    fn snapshot_light(&self) -> (Vec<[f64; 3]>, [[f64; 3]; 2], [[f64; 3]; 2]) {
        let com = (0..self.nbody).map(|b| self.sim.body_com(b)).collect();
        let mut hand = [[0.0; 3]; 2];
        let mut dir = [[0.0; 3]; 2];
        for side in 0..2 {
            let p = self.sim.body_com(self.hand_bodies[side]);
            let e = self.sim.joint_anchor(self.elbow_joints[side]);
            hand[side] = p;
            dir[side] = a3((v(p) - v(e)).normalize());
        }
        (com, hand, dir)
    }

    /// Единичная тяга мышцы на дистальный участок: отрезки пути, пересекающие границу поддерева.
    fn muscle_pull(&self, tendon: usize) -> [[f64; 3]; 3] {
        let path = self.sim.tendon_path_bodies(tendon);
        let mut out = [[0.0; 3]; 3];
        for (s, slot) in out.iter_mut().enumerate() {
            let mut f = Vector3::zeros();
            for w in path.windows(2) {
                if let (Some((p0, b0)), Some((p1, b1))) = (w[0], w[1]) {
                    let (in0, in1) = (self.subtrees[s][b0], self.subtrees[s][b1]);
                    if in0 != in1 {
                        let (pin, pout) = if in0 { (p0, p1) } else { (p1, p0) };
                        let d = v(pout) - v(pin);
                        let n = d.norm();
                        if n > 0.0 {
                            f += d / n;
                        }
                    }
                }
            }
            *slot = a3(f);
        }
        out
    }

    /// Полная матрица масс, приведённая к выбранным координатам с учётом связей: BᵀMB.
    /// `coords` — индексы координат цепочки; поза уже поставлена.
    pub fn reduced_mass(&mut self, pose: &BodyPose, b: &[Vec<f64>], two_handed: bool) -> Vec<f64> {
        let nv = self.sim.nv();
        self.apply(pose, &[0.0; NC], two_handed);
        let m = self.sim.full_mass_matrix();
        let n = b.len();
        let mut out = vec![0.0; n * n];
        for p in 0..n {
            for q in 0..n {
                let mut sum = 0.0;
                for (i, bi) in b[p].iter().enumerate() {
                    if *bi == 0.0 {
                        continue;
                    }
                    for (j, bj) in b[q].iter().enumerate() {
                        if *bj != 0.0 {
                            sum += bi * m[i * nv + j] * bj;
                        }
                    }
                }
                out[p * n + q] = sum;
            }
        }
        out
    }

    /// Столбцы B = ∂q/∂θ: как полные суставы модели меняются при движении координаты.
    pub fn coupling_columns(
        &mut self,
        pose: &BodyPose,
        coords: &[usize],
        two_handed: bool,
    ) -> Vec<Vec<f64>> {
        let nv = self.sim.nv();
        coords
            .iter()
            .map(|&k| {
                let mut d = [0.0; NC];
                d[k] = FD_ANGLE;
                self.apply(pose, &d, two_handed);
                let plus: Vec<f64> = self.sim.qpos().to_vec();
                d[k] = -FD_ANGLE;
                self.apply(pose, &d, two_handed);
                let minus: Vec<f64> = self.sim.qpos().to_vec();
                // В этой модели все суставы шарнирные: qpos и qvel совпадают по индексам.
                (0..nv)
                    .map(|i| {
                        let x = (plus[i] - minus[i]) / (2.0 * FD_ANGLE);
                        if x.abs() < 1e-9 { 0.0 } else { x }
                    })
                    .collect()
            })
            .collect()
    }

    /// Обобщённые силы смещения (сила тяжести мира модели + кориолисовы) MuJoCo: Bᵀ·qfrc_bias.
    pub fn reduced_bias(
        &mut self,
        pose: &BodyPose,
        coords: &[usize],
        rates: &[f64],
        columns: &[Vec<f64>],
        two_handed: bool,
    ) -> Vec<f64> {
        self.apply(pose, &[0.0; NC], two_handed);
        let nv = self.sim.nv();
        let mut qvel = vec![0.0; nv];
        for (p, _) in coords.iter().enumerate() {
            for i in 0..nv {
                qvel[i] += columns[p][i] * rates[p];
            }
        }
        self.sim.qvel_mut().copy_from_slice(&qvel);
        self.sim.forward();
        let bias = self.sim.bias_forces();
        let out = columns
            .iter()
            .map(|c| c.iter().zip(bias).map(|(a, b)| a * b).sum())
            .collect();
        self.sim.qvel_mut().fill(0.0);
        out
    }
}
