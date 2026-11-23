//! Алгоритмы динамики дерева тел (Featherstone 2008):
//! * RNEA (гл. 5) — обратная динамика: какие моменты нужны в шарнирах для заданного движения;
//! * ABA (гл. 7) — прямая динамика: какое ускорение даст заданный момент;
//! * CRBA через RNEA — матрица масс (для настройки регуляторов и проверок).
//!
//! Внешние силы `f_ext[i]` — пространственная сила, действующая НА тело i,
//! в осях тела i, относительно начала его кадра.

use nalgebra::{DMatrix, DVector, UnitQuaternion};

use crate::model::{Joint, Model, Pose, free_quat, quaternion_norm};
use crate::spatial::{Mat3, SVec, Vec3, Xform, assert_finite, cross_force, cross_motion, svec};

/// Преобразования «родитель → тело» для всех тел при координатах `q`.
pub fn parent_to_body(model: &Model, q: &DVector<f64>) -> Vec<Xform> {
    assert_eq!(q.len(), model.nq(), "размерность координат модели");
    model
        .bodies
        .iter()
        .enumerate()
        .map(|(i, b)| {
            assert_finite(b.placement.rot.as_slice(), "ориентация кадра шарнира");
            assert_finite(b.placement.pos.as_slice(), "смещение кадра шарнира");
            let qr = model.q_range(i);
            b.placement.then(&b.joint.transform(&q.as_slice()[qr]))
        })
        .collect()
}

/// Преобразования «мир → тело».
pub fn world_to_body(model: &Model, q: &DVector<f64>) -> Vec<Xform> {
    let up = parent_to_body(model, q);
    let mut out: Vec<Xform> = Vec::with_capacity(up.len());
    for (i, b) in model.bodies.iter().enumerate() {
        let x = match b.parent {
            Some(p) => out[p].then(&up[i]),
            None => up[i],
        };
        out.push(x);
    }
    out
}

/// Поза каждого тела в мире.
pub fn body_poses(model: &Model, q: &DVector<f64>) -> Vec<Pose> {
    world_to_body(model, q)
        .iter()
        .map(Pose::from_xform)
        .collect()
}

/// Пространственные скорости тел (в осях тел).
pub fn body_velocities(model: &Model, q: &DVector<f64>, v: &DVector<f64>) -> Vec<SVec> {
    assert_eq!(v.len(), model.nv(), "размерность скоростей модели");
    assert_finite(v.as_slice(), "скорости");
    let up = parent_to_body(model, q);
    let mut vel = vec![SVec::zeros(); model.bodies.len()];
    for (i, b) in model.bodies.iter().enumerate() {
        let vr = model.v_range(i);
        let vj: SVec = model.subspace(i) * v.rows(vr.start, vr.len());
        let vp = b
            .parent
            .map(|p| up[i].apply_motion(&vel[p]))
            .unwrap_or_else(SVec::zeros);
        vel[i] = vp + vj;
    }
    vel
}

fn gravity_accel(model: &Model) -> SVec {
    assert_finite(model.gravity.as_slice(), "гравитация");
    // Гравитация как фиктивное ускорение основания: a₀ = −g.
    svec(Vec3::zeros(), -model.gravity)
}

fn ext(f_ext: &[SVec], i: usize) -> SVec {
    f_ext.get(i).copied().unwrap_or_else(SVec::zeros)
}

fn validate_external_forces(f_ext: &[SVec]) {
    for force in f_ext {
        assert_finite(force.as_slice(), "внешняя сила");
    }
}

/// Обратная динамика (RNEA): обобщённые силы τ, при которых система с (q, v) получит ускорение `qdd`.
pub fn rnea(
    model: &Model,
    q: &DVector<f64>,
    v: &DVector<f64>,
    qdd: &DVector<f64>,
    f_ext: &[SVec],
) -> DVector<f64> {
    assert_eq!(v.len(), model.nv(), "размерность скоростей модели");
    assert_eq!(qdd.len(), model.nv(), "размерность ускорений модели");
    assert_finite(v.as_slice(), "скорости");
    assert_finite(qdd.as_slice(), "ускорения");
    validate_external_forces(f_ext);
    rnea_with_gravity(model, q, v, qdd, f_ext, gravity_accel(model))
}

fn rnea_with_gravity(
    model: &Model,
    q: &DVector<f64>,
    v: &DVector<f64>,
    qdd: &DVector<f64>,
    f_ext: &[SVec],
    a0: SVec,
) -> DVector<f64> {
    let n = model.bodies.len();
    let up = parent_to_body(model, q);
    let mut vel = vec![SVec::zeros(); n];
    let mut acc = vec![SVec::zeros(); n];
    let mut force = vec![SVec::zeros(); n];
    for i in 0..n {
        let b = &model.bodies[i];
        let s = model.subspace(i);
        let vr = model.v_range(i);
        let vj: SVec = s * v.rows(vr.start, vr.len());
        let aj: SVec = s * qdd.rows(vr.start, vr.len());
        let (vp, ap) = match b.parent {
            Some(p) => (up[i].apply_motion(&vel[p]), up[i].apply_motion(&acc[p])),
            None => (SVec::zeros(), up[i].apply_motion(&a0)),
        };
        vel[i] = vp + vj;
        acc[i] = ap + aj + cross_motion(&vel[i], &vj);
        let im = b.inertia.matrix();
        force[i] = im * acc[i] + cross_force(&vel[i], &(im * vel[i])) - ext(f_ext, i);
    }
    let mut tau = DVector::zeros(model.nv());
    for i in (0..n).rev() {
        let vr = model.v_range(i);
        if !vr.is_empty() {
            let mut t = model.subspace(i).transpose() * force[i];
            let arm = model.bodies[i].armature;
            assert_finite(&[arm], "приведённая инерция");
            if arm != 0.0 {
                t += qdd.rows(vr.start, vr.len()) * arm;
            }
            tau.rows_mut(vr.start, vr.len()).copy_from(&t);
        }
        if let Some(p) = model.bodies[i].parent {
            let fp = up[i].inv_apply_force(&force[i]);
            force[p] += fp;
        }
    }
    assert_finite(tau.as_slice(), "результат обратной динамики");
    tau
}

/// Прямая динамика (ABA): ускорения `qdd` при моментах `tau` и внешних силах.
pub fn aba(
    model: &Model,
    q: &DVector<f64>,
    v: &DVector<f64>,
    tau: &DVector<f64>,
    f_ext: &[SVec],
) -> DVector<f64> {
    assert_eq!(v.len(), model.nv(), "размерность скоростей модели");
    assert_eq!(tau.len(), model.nv(), "размерность обобщённых сил модели");
    assert_finite(v.as_slice(), "скорости");
    assert_finite(tau.as_slice(), "обобщённые силы");
    validate_external_forces(f_ext);
    let n = model.bodies.len();
    let up = parent_to_body(model, q);
    let mut vel = vec![SVec::zeros(); n];
    let mut c = vec![SVec::zeros(); n];
    let mut ia: Vec<crate::spatial::SMat> = Vec::with_capacity(n);
    let mut pa = vec![SVec::zeros(); n];

    for i in 0..n {
        let b = &model.bodies[i];
        let vr = model.v_range(i);
        let vj: SVec = model.subspace(i) * v.rows(vr.start, vr.len());
        let vp = b
            .parent
            .map(|p| up[i].apply_motion(&vel[p]))
            .unwrap_or_else(SVec::zeros);
        vel[i] = vp + vj;
        c[i] = cross_motion(&vel[i], &vj);
        let im = b.inertia.matrix();
        pa[i] = cross_force(&vel[i], &(im * vel[i])) - ext(f_ext, i);
        ia.push(im);
    }

    struct JointSolve {
        u: nalgebra::Matrix6xX<f64>,
        d_inv: DMatrix<f64>,
        uu: DVector<f64>,
    }
    let mut solves: Vec<Option<JointSolve>> = (0..n).map(|_| None).collect();

    for i in (0..n).rev() {
        let s = model.subspace(i);
        let vr = model.v_range(i);
        let (ia_art, pa_art) = if !vr.is_empty() {
            let u = ia[i] * s;
            let mut d = s.transpose() * &u;
            let arm = model.bodies[i].armature;
            assert_finite(&[arm], "приведённая инерция");
            if arm != 0.0 {
                for k in 0..d.nrows() {
                    d[(k, k)] += arm;
                }
            }
            let d_inv = d.clone().try_inverse().unwrap_or_else(|| {
                panic!(
                    "вырожденная инерция в шарнире тела «{}»",
                    model.bodies[i].name
                )
            });
            let tau_i = tau.rows(vr.start, vr.len()).clone_owned();
            let uu = tau_i - s.transpose() * pa[i];
            let ia_a = ia[i] - &u * &d_inv * u.transpose();
            let pa_a = pa[i] + ia_a * c[i] + &u * (&d_inv * &uu);
            solves[i] = Some(JointSolve { u, d_inv, uu });
            (ia_a, pa_a)
        } else {
            (ia[i], pa[i] + ia[i] * c[i])
        };
        if let Some(p) = model.bodies[i].parent {
            let xm = up[i].motion_matrix();
            ia[p] += xm.transpose() * ia_art * xm;
            pa[p] += up[i].inv_apply_force(&pa_art);
        }
    }

    let a0 = gravity_accel(model);
    let mut acc = vec![SVec::zeros(); n];
    let mut qdd = DVector::zeros(model.nv());
    for i in 0..n {
        let ap = match model.bodies[i].parent {
            Some(p) => up[i].apply_motion(&acc[p]),
            None => up[i].apply_motion(&a0),
        };
        let a_prime = ap + c[i];
        match &solves[i] {
            Some(js) => {
                let qdd_i = &js.d_inv * (&js.uu - js.u.transpose() * a_prime);
                let vr = model.v_range(i);
                qdd.rows_mut(vr.start, vr.len()).copy_from(&qdd_i);
                acc[i] = a_prime + model.subspace(i) * qdd_i;
            }
            None => acc[i] = a_prime,
        }
    }
    assert_finite(qdd.as_slice(), "результат прямой динамики");
    qdd
}

/// Матрица масс M(q) (столбцы через RNEA без гравитации и скоростей).
pub fn mass_matrix(model: &Model, q: &DVector<f64>) -> DMatrix<f64> {
    assert_eq!(q.len(), model.nq(), "размерность координат модели");
    let nv = model.nv();
    let zero = DVector::zeros(nv);
    let mut m = DMatrix::zeros(nv, nv);
    for j in 0..nv {
        let mut e = DVector::zeros(nv);
        e[j] = 1.0;
        let col = rnea_with_gravity(model, q, &zero, &e, &[], SVec::zeros());
        m.set_column(j, &col);
    }
    m
}

/// Моменты, которые нужны, чтобы держать систему неподвижно в позе `q` (гравитация + внешние силы).
pub fn static_holding_torques(model: &Model, q: &DVector<f64>, f_ext: &[SVec]) -> DVector<f64> {
    let z = DVector::zeros(model.nv());
    rnea(model, q, &z, &z, f_ext)
}

/// Пружины и вязкое трение для одноосных шарниров, изотропное вязкое трение для шаровых.
/// Скалярная пружина в шаровом соединении приводит к панике.
pub fn passive_forces(model: &Model, q: &DVector<f64>, v: &DVector<f64>) -> DVector<f64> {
    assert_eq!(q.len(), model.nq(), "размерность координат модели");
    assert_eq!(v.len(), model.nv(), "размерность скоростей модели");
    assert_finite(q.as_slice(), "координаты");
    assert_finite(v.as_slice(), "скорости");
    let mut tau = DVector::zeros(model.nv());
    for (i, b) in model.bodies.iter().enumerate() {
        b.validate_passive_parameters();
        if b.joint.nv() == 1 && (b.stiffness != 0.0 || b.damping != 0.0) {
            let qi = q[model.q_range(i).start];
            let vi = v[model.v_range(i).start];
            tau[model.v_range(i).start] = -b.stiffness * (qi - b.spring_rest) - b.damping * vi;
        } else if matches!(b.joint, Joint::Ball) && b.damping != 0.0 {
            for k in model.v_range(i) {
                tau[k] = -b.damping * v[k];
            }
        }
    }
    tau
}

/// Производная координат по скоростям, включая производную кватерниона шарового
/// или свободного соединения. При недопустимой конфигурации или скорости паникует.
pub fn q_derivative(model: &Model, q: &DVector<f64>, v: &DVector<f64>) -> DVector<f64> {
    assert_eq!(q.len(), model.nq(), "размерность координат модели");
    assert_eq!(v.len(), model.nv(), "размерность скоростей модели");
    assert_finite(q.as_slice(), "координаты");
    assert_finite(v.as_slice(), "скорости");
    let mut qd = DVector::zeros(model.nq());
    for (i, b) in model.bodies.iter().enumerate() {
        let qr = model.q_range(i);
        let vr = model.v_range(i);
        if matches!(b.joint, Joint::Free) {
            let linear_velocity = Vec3::from_column_slice(&v.as_slice()[vr.start + 3..vr.end]);
            let pd = free_quat(&q.as_slice()[qr.clone()]) * linear_velocity;
            qd.rows_mut(qr.start, 3).copy_from(&pd);
        }
        if let Some(offset) = b.joint.quaternion_offset() {
            let s = qr.start + offset;
            let qs = &q.as_slice()[s..qr.end];
            quaternion_norm(qs);
            let (qw, qx, qy, qz) = (qs[0], qs[1], qs[2], qs[3]);
            let w = Vec3::from_column_slice(&v.as_slice()[vr.start..vr.start + 3]);
            // q̇ = ½ q ⊗ (0, ω_тела). В стадиях RK4 берём q до итоговой проекции.
            qd[s] = -0.5 * (qx * w.x + qy * w.y + qz * w.z);
            qd[s + 1] = 0.5 * (qw * w.x + qy * w.z - qz * w.y);
            qd[s + 2] = 0.5 * (qw * w.y + qz * w.x - qx * w.z);
            qd[s + 3] = 0.5 * (qw * w.z + qx * w.y - qy * w.x);
        } else if !vr.is_empty() {
            qd[qr.start] = v[vr.start];
        }
    }
    qd
}

/// Нормировать кватернионы шаровых и свободных соединений после шага интегратора.
/// При недопустимой конфигурации паникует.
pub fn normalize_q(model: &Model, q: &mut DVector<f64>) {
    assert_eq!(q.len(), model.nq(), "размерность координат модели");
    assert_finite(q.as_slice(), "координаты");
    for (i, b) in model.bodies.iter().enumerate() {
        if let Some(offset) = b.joint.quaternion_offset() {
            let qr = model.q_range(i);
            let quat = &mut q.as_mut_slice()[qr.start + offset..qr.end];
            let n = quaternion_norm(quat);
            for x in quat {
                *x /= n;
            }
        }
    }
}

/// Кинетическая энергия, Дж.
pub fn kinetic_energy(model: &Model, q: &DVector<f64>, v: &DVector<f64>) -> f64 {
    let vel = body_velocities(model, q, v);
    let bodies: f64 = model
        .bodies
        .iter()
        .zip(vel.iter())
        .map(|(b, vi)| 0.5 * vi.dot(&(b.inertia.matrix() * vi)))
        .sum();
    // Роторы в шарнирах.
    let rotors: f64 = model
        .bodies
        .iter()
        .enumerate()
        .filter(|(_, b)| b.armature != 0.0)
        .map(|(i, b)| {
            assert_finite(&[b.armature], "приведённая инерция");
            let vr = model.v_range(i);
            0.5 * b.armature * v.rows(vr.start, vr.len()).norm_squared()
        })
        .sum();
    bodies + rotors
}

/// Потенциальная энергия (гравитация + пружины в шарнирах), Дж.
pub fn potential_energy(model: &Model, q: &DVector<f64>) -> f64 {
    assert_finite(model.gravity.as_slice(), "гравитация");
    let xw = world_to_body(model, q);
    let mut e = 0.0;
    for (i, b) in model.bodies.iter().enumerate() {
        b.validate_passive_parameters();
        assert_finite(&[b.inertia.mass], "масса");
        assert_finite(b.inertia.com.as_slice(), "центр масс");
        let c = xw[i].point_to_parent(&b.inertia.com);
        e -= b.inertia.mass * model.gravity.dot(&c);
        if b.joint.nq() == 1 && b.stiffness != 0.0 {
            let d = q[model.q_range(i).start] - b.spring_rest;
            e += 0.5 * b.stiffness * d * d;
        }
    }
    e
}

/// Центры масс тел в мире.
pub fn body_coms_world(model: &Model, q: &DVector<f64>) -> Vec<Vec3> {
    let xw = world_to_body(model, q);
    model
        .bodies
        .iter()
        .enumerate()
        .map(|(i, b)| xw[i].point_to_parent(&b.inertia.com))
        .collect()
}

/// Суммарный импульс системы `[L; p]` в мировых осях относительно начала мира.
pub fn spatial_momentum_world(model: &Model, q: &DVector<f64>, v: &DVector<f64>) -> SVec {
    let xw = world_to_body(model, q);
    let vel = body_velocities(model, q, v);
    let mut h = SVec::zeros();
    for (i, b) in model.bodies.iter().enumerate() {
        h += xw[i].inv_apply_force(&(b.inertia.matrix() * vel[i]));
    }
    h
}

/// Точечная сила, заданная в мире, приложенная к точке тела (в осях тела) →
/// пространственная сила в осях тела для `f_ext`.
pub fn point_force_on_body(world_to_body: &Xform, point_body: &Vec3, force_world: &Vec3) -> SVec {
    let f_body = world_to_body.rot * force_world;
    svec(point_body.cross(&f_body), f_body)
}

/// Ориентация как кватернион из матрицы осей.
pub fn quat_from_mat(m: &Mat3) -> UnitQuaternion<f64> {
    UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix_unchecked(*m))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{Body, Joint, Model};
    use crate::spatial::Inertia;

    /// Ветвящееся дерево со всеми типами шарниров — для сверки RNEA и ABA между собой.
    fn messy_model() -> Model {
        let mut m = Model::earth();
        let b0 = m.add_body(Body::new(
            "base",
            None,
            Joint::Free,
            Xform::translation(Vec3::new(0.0, 0.0, 1.0)),
            Inertia::solid_box(3.0, Vec3::new(0.2, 0.1, 0.05), Vec3::new(0.01, 0.0, 0.0)),
        ));
        let b1 = m.add_body(Body::new(
            "a",
            Some(b0),
            Joint::revolute(Vec3::new(0.0, 1.0, 0.3)),
            Xform::translation(Vec3::new(0.2, 0.0, 0.0)),
            Inertia::solid_cylinder(1.0, 0.03, 0.4, 2, Vec3::new(0.0, 0.0, -0.2)),
        ));
        m.add_body(Body::new(
            "b",
            Some(b1),
            Joint::prismatic(Vec3::new(0.0, 0.0, -1.0)),
            Xform::translation(Vec3::new(0.0, 0.0, -0.4)),
            Inertia::solid_sphere(0.5, 0.05, Vec3::new(0.02, 0.0, 0.0)),
        ));
        let b3 = m.add_body(Body::new(
            "c",
            Some(b0),
            Joint::revolute(Vec3::new(1.0, 0.0, 0.0)),
            Xform::translation(Vec3::new(-0.2, 0.1, 0.0)),
            Inertia::solid_box(0.7, Vec3::new(0.02, 0.15, 0.02), Vec3::new(0.0, 0.15, 0.0)),
        ));
        m.add_body(Body::new(
            "d",
            Some(b3),
            Joint::Fixed,
            Xform::translation(Vec3::new(0.0, 0.3, 0.0)),
            Inertia::solid_sphere(0.3, 0.04, Vec3::zeros()),
        ));
        m
    }

    fn sample_state(m: &Model) -> (DVector<f64>, DVector<f64>) {
        let mut q = m.neutral_q();
        q[0] = 0.1;
        q[1] = -0.2;
        q[2] = 1.3;
        let quat = UnitQuaternion::from_euler_angles(0.3, -0.4, 0.8);
        q[3] = quat.w;
        q[4] = quat.i;
        q[5] = quat.j;
        q[6] = quat.k;
        q[7] = 0.7;
        q[8] = 0.05;
        q[9] = -1.1;
        let v = DVector::from_vec(vec![0.3, -0.5, 0.9, 0.2, 0.1, -0.4, 1.5, -0.3, 2.0]);
        (q, v)
    }

    #[test]
    fn rnea_inverts_aba() {
        let m = messy_model();
        let (q, v) = sample_state(&m);
        let tau = DVector::from_vec(vec![0.5, -1.0, 2.0, 0.1, 0.0, -0.3, 1.2, 4.0, -0.7]);
        let mut fext = m.zero_forces();
        fext[2] = SVec::new(0.1, 0.2, -0.1, 3.0, -2.0, 1.0);
        let qdd = aba(&m, &q, &v, &tau, &fext);
        let tau_back = rnea(&m, &q, &v, &qdd, &fext);
        let err = (&tau - &tau_back).norm();
        assert!(err < 1e-9, "RNEA(ABA(τ)) ≠ τ: {err}");
    }

    #[test]
    fn mass_matrix_is_symmetric_positive_and_consistent() {
        let m = messy_model();
        let (q, v) = sample_state(&m);
        let mm = mass_matrix(&m, &q);
        assert!((&mm - mm.transpose()).norm() < 1e-9);
        assert!(mm.clone().cholesky().is_some());
        // ABA == M⁻¹ (τ − C(q, v))
        let tau = DVector::from_element(m.nv(), 0.3);
        let bias = rnea(&m, &q, &v, &DVector::zeros(m.nv()), &[]);
        let qdd_crba = mm.cholesky().unwrap().solve(&(&tau - bias));
        let qdd_aba = aba(&m, &q, &v, &tau, &[]);
        assert!((qdd_crba - qdd_aba).norm() < 1e-9);
        // Кинетическая энергия == ½ vᵀ M v
        let ke = kinetic_energy(&m, &q, &v);
        let ke_m = 0.5 * v.dot(&(mass_matrix(&m, &q) * &v));
        assert!((ke - ke_m).abs() < 1e-9);
    }

    #[test]
    fn armature_is_consistent_between_rnea_aba_and_energy() {
        let mut m = messy_model();
        for b in &mut m.bodies {
            if b.joint.nv() == 1 {
                b.armature = 0.03;
            }
        }
        let (q, v) = sample_state(&m);
        let tau = DVector::from_element(m.nv(), 0.7);
        let qdd = aba(&m, &q, &v, &tau, &[]);
        let back = rnea(&m, &q, &v, &qdd, &[]);
        assert!((&back - &tau).norm() < 1e-9);
        let mm = mass_matrix(&m, &q);
        let ke = 0.5 * v.dot(&(mm * &v));
        assert!((ke - kinetic_energy(&m, &q, &v)).abs() < 1e-9);
    }

    #[test]
    fn free_body_in_gravity_accelerates_at_g() {
        let mut m = Model::earth();
        m.add_body(Body::new(
            "ball",
            None,
            Joint::Free,
            Xform::IDENTITY,
            Inertia::solid_sphere(2.0, 0.1, Vec3::zeros()),
        ));
        let q = m.neutral_q();
        let v = DVector::zeros(6);
        let qdd = aba(&m, &q, &v, &DVector::zeros(6), &[]);
        assert!((qdd[5] + crate::model::G_STANDARD).abs() < 1e-12);
        assert!(qdd.rows(0, 5).norm() < 1e-12);
    }
}
