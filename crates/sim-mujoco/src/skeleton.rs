//! Перенос твёрдотельной части модели MuJoCo в `sim-core` и сверка двух движков.
//!
//! Как в `mj_kinematics`, каждое преобразование соединения Xⱼ применяется в порядке MJCF:
//! кадр = кадр родителя · (pos, quat) · Π [T(aⱼ) · Xⱼ · T(−aⱼ)].
//! aⱼ — точка крепления. Для шарового Xⱼ берётся из кватерниона, для одноосного — из qⱼ − q⁰ⱼ.
//! Тело с k соединениями становится цепочкой из k тел ядра. Промежуточные тела не имеют массы.
//! T(−aₖ) задаёт смещение до кадра исходного тела.
//!
//! Свободное соединение. MuJoCo: qvel = [v в осях мира; ω в осях тела]. Ядро: v = [ω; v] в осях
//! тела. Поэтому v_ядра = T·qvel, M_mj = Tᵀ·M·T, а смещение равно Tᵀ·RNEA(q, v, q̈ₒ), где
//! q̈ₒ = [0; −ω × v_тела]: это производная T при нулевом ускорении в координатах MuJoCo.

use std::ops::Range;

use nalgebra::{DMatrix, DVector, Matrix3, Quaternion, UnitQuaternion, Vector3};
use sim_core::dynamics::{mass_matrix, rnea, world_to_body};
use sim_core::{Body, Inertia, Joint, Model, Rng, Vec3, Xform};

use crate::ffi::mjtJoint;
use crate::{JointKind, Sim};

/// Диапазоны одного соединения в MuJoCo и ядре.
#[derive(Clone, Debug, PartialEq)]
pub struct JointMapping {
    pub qpos: Range<usize>,
    pub dofs: Range<usize>,
    pub q: Range<usize>,
    pub v: Range<usize>,
    /// Опорная координата одноосного соединения. У шарового кватернион передаётся целиком.
    pub scalar_reference: Option<f64>,
    /// Свободное соединение: скорости MuJoCo и ядра заданы в разных кадрах.
    pub free: bool,
}

/// Наше дерево тел, построенное по модели MuJoCo, и соответствие индексов.
pub struct Skeleton {
    pub model: Model,
    /// Для каждого тела MuJoCo: наше тело и смещение от его кадра до кадра тела MuJoCo.
    /// `None` — мир (тело 0).
    pub body_frame: Vec<Option<(usize, Xform)>>,
    /// Диапазоны координат и степеней свободы в порядке соединений MuJoCo.
    pub joints: Vec<JointMapping>,
}

fn unit_quat(q: &[f64]) -> Option<UnitQuaternion<f64>> {
    let (quat, norm) = UnitQuaternion::new_and_get(Quaternion::new(q[0], q[1], q[2], q[3]));
    (norm.is_finite() && norm > 0.0).then_some(quat)
}

fn rot(q: &[f64], field: &str) -> Result<Matrix3<f64>, String> {
    unit_quat(q)
        .map(|r| r.to_rotation_matrix().into_inner())
        .ok_or_else(|| format!("{field}: недопустимый кватернион"))
}

fn assert_finite(values: &[f64], field: &str) {
    assert!(
        values.iter().all(|x| x.is_finite()),
        "{field}: требуется конечное значение"
    );
}

impl Skeleton {
    /// Перенести твёрдотельную часть модели с вращательными, поступательными и шаровыми соединениями.
    pub fn from_mujoco(sim: &Sim) -> Result<Self, String> {
        let m = sim.model();
        let nb = sim.nbody();
        let nj = sim.njnt();
        let parent = Sim::arr(m.body_parentid, nb);
        let pos = Sim::arr(m.body_pos, 3 * nb);
        let quat = Sim::arr(m.body_quat, 4 * nb);
        let mass = Sim::arr(m.body_mass, nb);
        let ipos = Sim::arr(m.body_ipos, 3 * nb);
        let iquat = Sim::arr(m.body_iquat, 4 * nb);
        let inertia = Sim::arr(m.body_inertia, 3 * nb);
        let jntnum = Sim::arr(m.body_jntnum, nb);
        let jntadr = Sim::arr(m.body_jntadr, nb);
        let jtype = Sim::arr(m.jnt_type, nj);
        let jpos = Sim::arr(m.jnt_pos, 3 * nj);
        let jaxis = Sim::arr(m.jnt_axis, 3 * nj);
        let qadr = Sim::arr(m.jnt_qposadr, nj);
        let dadr = Sim::arr(m.jnt_dofadr, nj);
        let qpos0 = sim.qpos0();
        let armature = Sim::arr(m.dof_armature, sim.nv());
        let g = sim.gravity();

        for (field, values) in [
            ("гравитация", g.as_slice()),
            ("положение тела", pos),
            ("масса тела", mass),
            ("центр масс", ipos),
            ("тензор инерции", inertia),
            ("точка крепления", jpos),
            ("ось соединения", jaxis),
            ("опорная конфигурация", qpos0),
            ("приведённая инерция", armature),
        ] {
            if !values.iter().all(|x| x.is_finite()) {
                return Err(format!("{field}: требуется конечное значение"));
            }
        }

        let mut model = Model::new(Vec3::new(g[0], g[1], g[2]));
        let mut body_frame: Vec<Option<(usize, Xform)>> = vec![None; nb];
        let mut joints = Vec::with_capacity(nj);
        let v3 = |a: &[f64], i: usize| Vec3::new(a[3 * i], a[3 * i + 1], a[3 * i + 2]);

        for b in 1..nb {
            let p = parent[b] as usize;
            let (our_parent, off) = match body_frame[p] {
                Some((i, x)) => (Some(i), x),
                None => (None, Xform::IDENTITY),
            };
            let body_rot = rot(&quat[4 * b..4 * b + 4], "ориентация тела")?;
            let body_x = Xform::from_pose(&body_rot, &v3(pos, b));
            let mut prefix = off.then(&body_x);
            let mut parent_idx = our_parent;
            let name = sim.name(crate::ffi::mjtObj::mjOBJ_BODY, b);
            // Инерция тела MuJoCo в его собственном кадре.
            let ri = rot(&iquat[4 * b..4 * b + 4], "ориентация тензора инерции")?;
            let i_com =
                ri * Matrix3::from_diagonal(&Vector3::new(
                    inertia[3 * b],
                    inertia[3 * b + 1],
                    inertia[3 * b + 2],
                )) * ri.transpose();
            let com = v3(ipos, b);

            let n = jntnum[b] as usize;
            if n == 0 {
                let idx = model.add_body(Body::new(
                    &name,
                    parent_idx,
                    Joint::Fixed,
                    prefix,
                    Inertia::new(mass[b], com, i_com),
                ));
                body_frame[b] = Some((idx, Xform::IDENTITY));
                continue;
            }
            let first = jntadr[b] as usize;
            let mut last_anchor = Vec3::zeros();
            for k in 0..n {
                let j = first + k;
                let a = v3(jpos, j);
                let axis = v3(jaxis, j);
                let joint = match jtype[j] as u32 {
                    mjtJoint::mjJNT_HINGE => Joint::revolute(axis),
                    mjtJoint::mjJNT_SLIDE => Joint::prismatic(axis),
                    mjtJoint::mjJNT_BALL => Joint::Ball,
                    mjtJoint::mjJNT_FREE => Joint::Free,
                    other => {
                        return Err(format!(
                            "соединение «{}»: тип {other} не поддерживается при переносе",
                            sim.name(crate::ffi::mjtObj::mjOBJ_JOINT, j)
                        ));
                    }
                };
                let qa = qadr[j] as usize;
                let da = dadr[j] as usize;
                let qpos = qa..qa + joint.nq();
                let dofs = da..da + joint.nv();
                let scalar_reference = (joint.nq() == 1).then(|| qpos0[qa]);
                let free = matches!(joint, Joint::Free);
                let arm = armature[da];
                // У атрибута armature в MJCF одно значение на все степени свободы соединения.
                if armature[dofs.clone()].iter().any(|&x| x != arm) {
                    return Err(format!("разная приведённая инерция по осям соединения {j}"));
                }
                // В ядре приведённая инерция применяется к скоростям в осях тела, у MuJoCo
                // поступательная часть свободного соединения в осях мира: это разные модели.
                if free && armature[dofs.clone()].iter().any(|&x| x != 0.0) {
                    return Err(format!(
                        "соединение «{}»: приведённая инерция свободного тела не переносится",
                        sim.name(crate::ffi::mjtObj::mjOBJ_JOINT, j)
                    ));
                }
                // Поза свободного тела целиком лежит в его координатах.
                let placement = if free {
                    Xform::IDENTITY
                } else {
                    prefix.then(&Xform::translation(a))
                };
                let last = k + 1 == n;
                let inert = if last {
                    Inertia::new(mass[b], com - a, i_com)
                } else {
                    Inertia::zero()
                };
                let mut body =
                    Body::new(&format!("{name}#{k}"), parent_idx, joint, placement, inert);
                body.armature = arm;
                let idx = model.add_body(body);
                // MuJoCo нумерует соединения в порядке тел и хранит их непрерывными группами.
                assert_eq!(j, joints.len(), "порядок соединений MuJoCo");
                joints.push(JointMapping {
                    qpos,
                    dofs,
                    q: model.q_range(idx),
                    v: model.v_range(idx),
                    scalar_reference,
                    free,
                });
                parent_idx = Some(idx);
                prefix = Xform::translation(-a);
                last_anchor = a;
            }
            body_frame[b] = Some((parent_idx.unwrap(), Xform::translation(-last_anchor)));
        }
        Ok(Self {
            model,
            body_frame,
            joints,
        })
    }

    /// Координаты ядра из qpos MuJoCo. Одноосные координаты отсчитываются от q⁰.
    /// Неверная размерность, неконечные координаты и нулевой кватернион приводят к панике.
    pub fn q_from(&self, qpos: &[f64]) -> DVector<f64> {
        assert_eq!(qpos.len(), self.model.nq(), "размерность qpos MuJoCo");
        assert_finite(qpos, "qpos MuJoCo");
        let mut q = self.model.neutral_q();
        for joint in &self.joints {
            if let Some(q0) = joint.scalar_reference {
                q[joint.q.start] = qpos[joint.qpos.start] - q0;
            } else {
                let values = &qpos[joint.qpos.clone()];
                let quat = if joint.free { &values[3..] } else { values };
                assert!(
                    unit_quat(quat).is_some(),
                    "недопустимый кватернион qpos MuJoCo"
                );
                q.as_mut_slice()[joint.q.clone()].copy_from_slice(values);
            }
        }
        assert_finite(q.as_slice(), "координаты ядра");
        q
    }

    /// Переход от скоростей MuJoCo к скоростям ядра: v = T·qvel (строки — ядро, столбцы — MuJoCo).
    pub fn velocity_map(&self, qpos: &[f64]) -> DMatrix<f64> {
        assert_eq!(qpos.len(), self.model.nq(), "размерность qpos MuJoCo");
        assert_finite(qpos, "qpos MuJoCo");
        let nv = self.model.nv();
        let mut t = DMatrix::zeros(nv, nv);
        for joint in &self.joints {
            if joint.free {
                let (dof, v) = (joint.dofs.start, joint.v.start);
                let quat = unit_quat(&qpos[joint.qpos.start + 3..joint.qpos.end])
                    .expect("недопустимый кватернион qpos MuJoCo");
                let world_to_body = quat.to_rotation_matrix().into_inner().transpose();
                for k in 0..3 {
                    t[(v + k, dof + 3 + k)] = 1.0;
                }
                t.view_mut((v + 3, dof), (3, 3)).copy_from(&world_to_body);
            } else {
                for (i, d) in joint.v.clone().zip(joint.dofs.clone()) {
                    t[(i, d)] = 1.0;
                }
            }
        }
        t
    }

    /// Ускорение ядра, которому в координатах MuJoCo соответствует нулевое q̈.
    fn zero_mj_acceleration(&self, v: &DVector<f64>) -> DVector<f64> {
        let mut qdd = DVector::zeros(self.model.nv());
        for joint in self.joints.iter().filter(|j| j.free) {
            let i = joint.v.start;
            let omega = Vec3::new(v[i], v[i + 1], v[i + 2]);
            let linear = Vec3::new(v[i + 3], v[i + 4], v[i + 5]);
            let a = -omega.cross(&linear);
            qdd.as_mut_slice()[i + 3..i + 6].copy_from_slice(a.as_slice());
        }
        qdd
    }

    /// Скорости ядра для сцены без свободных соединений. Со свободным нужен `velocity_map`.
    pub fn v_from(&self, qvel: &[f64]) -> DVector<f64> {
        assert!(
            self.joints.iter().all(|j| !j.free),
            "для свободного соединения нужен velocity_map"
        );
        assert_eq!(qvel.len(), self.model.nv(), "размерность qvel MuJoCo");
        assert_finite(qvel, "qvel MuJoCo");
        let mut v = DVector::zeros(self.model.nv());
        for joint in &self.joints {
            v.as_mut_slice()[joint.v.clone()].copy_from_slice(&qvel[joint.dofs.clone()]);
        }
        v
    }

    /// Обратно: наш вектор обобщённых сил → порядок степеней свободы MuJoCo.
    pub fn to_mj_dofs(&self, ours: &DVector<f64>, nv: usize) -> Vec<f64> {
        assert!(
            self.joints.iter().all(|j| !j.free),
            "для свободного соединения нужен velocity_map"
        );
        assert_eq!(nv, self.model.nv(), "размерность степеней свободы MuJoCo");
        assert_eq!(
            ours.len(),
            self.model.nv(),
            "размерность обобщённых сил ядра"
        );
        assert_finite(ours.as_slice(), "обобщённые силы ядра");
        let mut out = vec![0.0; nv];
        for joint in &self.joints {
            out[joint.dofs.clone()].copy_from_slice(&ours.as_slice()[joint.v.clone()]);
        }
        out
    }

    /// Положение и ориентация каждого тела MuJoCo по нашему движку.
    pub fn body_poses(&self, q: &DVector<f64>) -> Vec<Option<(Vec3, UnitQuaternion<f64>)>> {
        let xw = world_to_body(&self.model, q);
        self.body_frame
            .iter()
            .map(|f| {
                f.map(|(i, off)| {
                    let x = xw[i].then(&off);
                    let r = nalgebra::Rotation3::from_matrix_unchecked(x.orientation());
                    (x.pos, UnitQuaternion::from_rotation_matrix(&r))
                })
            })
            .collect()
    }

    fn dof_indices(&self) -> impl Iterator<Item = (usize, usize)> + '_ {
        self.joints
            .iter()
            .flat_map(|joint| joint.dofs.clone().zip(joint.v.clone()))
    }

    /// Наша матрица (nv × nv) → порядок степеней свободы MuJoCo, по строкам.
    pub fn to_mj_matrix(&self, ours: &DMatrix<f64>, nv: usize) -> Vec<f64> {
        assert!(
            self.joints.iter().all(|j| !j.free),
            "для свободного соединения нужен velocity_map"
        );
        assert_eq!(nv, self.model.nv(), "размерность степеней свободы MuJoCo");
        assert_eq!(ours.shape(), (nv, nv), "размерность матрицы масс ядра");
        assert_finite(ours.as_slice(), "матрица масс ядра");
        let mut out = vec![0.0; nv * nv];
        for (da, i) in self.dof_indices() {
            for (db, k) in self.dof_indices() {
                out[da * nv + db] = ours[(i, k)];
            }
        }
        out
    }
}

/// Поворот с равномерным направлением оси на сфере и углом до `max_angle`.
/// Элемент площади сферы равен dφ dz, где z — проекция оси на вертикаль.
fn random_rotation(rng: &mut Rng, max_angle: f64) -> UnitQuaternion<f64> {
    let z = 2.0 * rng.uniform() - 1.0;
    let (sin_az, cos_az) = (std::f64::consts::TAU * rng.uniform()).sin_cos();
    let r = (1.0 - z * z).sqrt();
    let axis = nalgebra::Unit::new_normalize(Vec3::new(r * cos_az, r * sin_az, z));
    UnitQuaternion::from_axis_angle(&axis, max_angle * rng.uniform())
}

/// Итог сверки «наш движок ↔ MuJoCo» на одной модели.
#[derive(Clone, Copy, Debug, Default)]
pub struct EngineCheck {
    pub samples: usize,
    /// Наибольшее расхождение положения тела, м.
    pub max_pos_err: f64,
    /// Наибольшее расхождение ориентации тела, рад.
    pub max_rot_err: f64,
    /// Наибольшее расхождение матрицы масс, доля от max|M|.
    pub max_mass_rel: f64,
    /// Наибольшее расхождение сил тяжести и инерции (C + G), Н·м или Н.
    pub max_bias_abs: f64,
    /// Масштаб сил C + G (наибольшее |значение|) — чтобы понимать относительную величину.
    pub bias_scale: f64,
}

/// Случайные позы в пределах суставов и случайные скорости; сравниваем кинематику,
/// матрицу масс и силы тяжести/инерции двух движков.
/// Пустая выборка, несовместимые модели и неконечные данные приводят к панике.
pub fn cross_check(sim: &mut Sim, sk: &Skeleton, samples: usize, seed: u64) -> EngineCheck {
    assert!(samples > 0, "сверка требует хотя бы одной позы");
    assert_eq!(
        sim.njnt(),
        sk.joints.len(),
        "число соединений MuJoCo и ядра"
    );
    assert_eq!(
        sim.nbody(),
        sk.body_frame.len(),
        "число тел MuJoCo и отображений"
    );
    assert_eq!(sim.nq(), sk.model.nq(), "размерность qpos MuJoCo и ядра");
    assert_eq!(sim.nv(), sk.model.nv(), "размерность qvel MuJoCo и ядра");
    assert_finite(sim.qpos(), "начальные qpos MuJoCo");
    assert_finite(sim.qvel(), "начальные qvel MuJoCo");
    let mut rng = Rng::stream(seed, "сверка многотельной динамики");
    let joints = sim.joints();
    let q0 = sim.qpos0().to_vec();
    let nv = sim.nv();
    let mut out = EngineCheck {
        samples,
        ..Default::default()
    };
    for _ in 0..samples {
        sim.qpos_mut().copy_from_slice(&q0);
        for j in &joints {
            assert_finite(&j.range, "границы соединения MuJoCo");
            if j.kind == JointKind::Ball {
                // MuJoCo ограничивает угол поворота только верхним значением range.
                // Углы до π покрывают все ориентации.
                let max_angle = if j.limited {
                    assert!(
                        j.range[1] >= 0.0,
                        "отрицательная граница шарового соединения"
                    );
                    j.range[1].min(std::f64::consts::PI)
                } else {
                    std::f64::consts::PI
                };
                let q = random_rotation(&mut rng, max_angle);
                sim.qpos_mut()[j.qpos_adr..j.qpos_adr + 4].copy_from_slice(&[q.w, q.i, q.j, q.k]);
                continue;
            }
            if j.kind == JointKind::Free {
                for k in 0..3 {
                    // Диапазон выборки: ±0,5 м вокруг начальной позы.
                    sim.qpos_mut()[j.qpos_adr + k] = q0[j.qpos_adr + k] + rng.uniform() - 0.5;
                }
                let q = random_rotation(&mut rng, std::f64::consts::PI);
                sim.qpos_mut()[j.qpos_adr + 3..j.qpos_adr + 7]
                    .copy_from_slice(&[q.w, q.i, q.j, q.k]);
                continue;
            }
            // Унаследованные диапазоны сверки: 5 см для каретки, 0,6 рад для шарнира.
            let (lo, hi) = if j.limited {
                (j.range[0], j.range[1])
            } else if j.kind == JointKind::Slide {
                (-0.05, 0.05)
            } else {
                (-0.6, 0.6)
            };
            assert!(lo <= hi, "обратные границы соединения MuJoCo");
            sim.qpos_mut()[j.qpos_adr] = lo + (hi - lo) * rng.uniform();
        }
        // Унаследованный диапазон скоростей −1…1 м/с или рад/с.
        for speed in sim.qvel_mut() {
            *speed = 2.0 * rng.uniform() - 1.0;
        }
        let q = sk.q_from(sim.qpos());
        let t = sk.velocity_map(sim.qpos());
        let v = &t * DVector::from_column_slice(sim.qvel());
        sim.forward();
        for (b, ours) in sk.body_poses(&q).iter().enumerate() {
            if let Some((p, r)) = ours {
                let (mp, mq) = sim.body_pose(b);
                let dp = (p - Vec3::new(mp[0], mp[1], mp[2])).norm();
                let mr = unit_quat(&mq).expect("недопустимый кватернион позы MuJoCo");
                let dr = r.angle_to(&mr);
                assert_finite(&[dp, dr], "расхождение поз");
                out.max_pos_err = out.max_pos_err.max(dp);
                out.max_rot_err = out.max_rot_err.max(dr);
            }
        }
        let m_in_mj = t.transpose() * mass_matrix(&sk.model, &q) * &t;
        assert_finite(m_in_mj.as_slice(), "матрица масс ядра");
        let m_ours: Vec<f64> = (0..nv)
            .flat_map(|r| (0..nv).map(move |c| (r, c)))
            .map(|ij| m_in_mj[ij])
            .collect();
        let m_mj = sim.full_mass_matrix();
        assert_finite(&m_mj, "матрица масс MuJoCo");
        let scale = m_mj.iter().fold(0.0_f64, |a, x| a.max(x.abs()));
        let dm = m_ours
            .iter()
            .zip(&m_mj)
            .fold(0.0_f64, |a, (x, y)| a.max((x - y).abs()));
        // У пустой системы матрица пуста. У подвижной системы нулевая матрица вырождена.
        if nv > 0 {
            assert!(scale > 0.0, "нулевая матрица масс MuJoCo");
            let rel = dm / scale;
            assert_finite(&[rel], "расхождение матриц масс");
            out.max_mass_rel = out.max_mass_rel.max(rel);
        }
        let bias_ours = t.transpose() * rnea(&sk.model, &q, &v, &sk.zero_mj_acceleration(&v), &[]);
        assert_finite(bias_ours.as_slice(), "обобщённые силы ядра");
        let bias_mj = sim.bias_forces();
        assert_finite(bias_mj, "силы тяжести и инерции MuJoCo");
        for (x, y) in bias_ours.iter().zip(bias_mj) {
            let error = (x - y).abs();
            assert_finite(&[error], "расхождение сил тяжести и инерции");
            out.max_bias_abs = out.max_bias_abs.max(error);
            out.bias_scale = out.bias_scale.max(y.abs());
        }
    }
    sim.qpos_mut().copy_from_slice(&q0);
    sim.qvel_mut().fill(0.0);
    sim.forward();
    out
}
