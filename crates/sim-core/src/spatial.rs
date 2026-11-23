//! Пространственная (6D) алгебра по Featherstone, «Rigid Body Dynamics Algorithms» (2008), гл. 2.
//!
//! Соглашения (те же, что у RBDL и Pinocchio, поэтому результаты сверяемы напрямую):
//! * вектор движения `[ω; v]` — сначала угловая часть, потом линейная;
//! * вектор силы `[n; f]` — сначала момент, потом сила;
//! * `v` в векторе движения — скорость точки тела, совпадающей с началом координат кадра.

use nalgebra::{Matrix3, Matrix6, Vector3, Vector6};

pub type Vec3 = Vector3<f64>;
pub type Mat3 = Matrix3<f64>;
/// Пространственный вектор (движения или силы — по смыслу).
pub type SVec = Vector6<f64>;
pub type SMat = Matrix6<f64>;

pub(crate) fn assert_finite(values: &[f64], name: &str) {
    assert!(
        values.iter().all(|x| x.is_finite()),
        "{name}: требуется конечное значение"
    );
}

/// Кососимметричная матрица: `skew(a) * b == a × b`.
pub fn skew(v: &Vec3) -> Mat3 {
    Mat3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

#[inline]
pub fn ang(s: &SVec) -> Vec3 {
    Vec3::new(s[0], s[1], s[2])
}

#[inline]
pub fn lin(s: &SVec) -> Vec3 {
    Vec3::new(s[3], s[4], s[5])
}

#[inline]
pub fn svec(angular: Vec3, linear: Vec3) -> SVec {
    SVec::new(
        angular.x, angular.y, angular.z, linear.x, linear.y, linear.z,
    )
}

/// `v ×  m` — пространственное векторное произведение для векторов движения (crm).
pub fn cross_motion(v: &SVec, m: &SVec) -> SVec {
    let (w, vo) = (ang(v), lin(v));
    let (mw, mv) = (ang(m), lin(m));
    svec(w.cross(&mw), w.cross(&mv) + vo.cross(&mw))
}

/// `v ×* f` — пространственное векторное произведение для векторов силы (crf).
pub fn cross_force(v: &SVec, f: &SVec) -> SVec {
    let (w, vo) = (ang(v), lin(v));
    let (n, fl) = (ang(f), lin(f));
    svec(w.cross(&n) + vo.cross(&fl), w.cross(&fl))
}

/// Преобразование Плюккера из кадра A в кадр B.
///
/// * `rot` = E — переводит координаты из осей A в оси B (E = Rᵀ, где R — ориентация B в A);
/// * `pos` = r — положение начала B, выраженное в A.
///
/// Матрица для векторов движения: `X = [E 0; -E·r× E]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Xform {
    pub rot: Mat3,
    pub pos: Vec3,
}

impl Default for Xform {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Xform {
    pub const IDENTITY: Xform = Xform {
        rot: Mat3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        pos: Vec3::new(0.0, 0.0, 0.0),
    };

    /// Из позы: `orientation` — оси B в координатах A, `origin` — начало B в A.
    pub fn from_pose(orientation: &Mat3, origin: &Vec3) -> Self {
        Self {
            rot: orientation.transpose(),
            pos: *origin,
        }
    }

    pub fn translation(p: Vec3) -> Self {
        Self {
            rot: Mat3::identity(),
            pos: p,
        }
    }

    /// Ориентация B в A (матрица, столбцы которой — оси B в координатах A).
    pub fn orientation(&self) -> Mat3 {
        self.rot.transpose()
    }

    pub fn apply_motion(&self, m: &SVec) -> SVec {
        let w = ang(m);
        let v = lin(m);
        svec(self.rot * w, self.rot * (v - self.pos.cross(&w)))
    }

    pub fn apply_force(&self, f: &SVec) -> SVec {
        let n = ang(f);
        let fl = lin(f);
        svec(self.rot * (n - self.pos.cross(&fl)), self.rot * fl)
    }

    /// `X⁻¹ m`: вектор движения из B обратно в A.
    pub fn inv_apply_motion(&self, m: &SVec) -> SVec {
        let w = self.rot.transpose() * ang(m);
        let v = self.rot.transpose() * lin(m) + self.pos.cross(&w);
        svec(w, v)
    }

    /// `Xᵀ f`: сила из B в A (так сила ребёнка передаётся родителю).
    pub fn inv_apply_force(&self, f: &SVec) -> SVec {
        let fl = self.rot.transpose() * lin(f);
        let n = self.rot.transpose() * ang(f) + self.pos.cross(&fl);
        svec(n, fl)
    }

    /// Композиция: `self` (A→B), затем `next` (B→C) ⇒ A→C.
    pub fn then(&self, next: &Xform) -> Xform {
        Xform {
            rot: next.rot * self.rot,
            pos: self.pos + self.rot.transpose() * next.pos,
        }
    }

    pub fn inverse(&self) -> Xform {
        Xform {
            rot: self.rot.transpose(),
            pos: -(self.rot * self.pos),
        }
    }

    pub fn motion_matrix(&self) -> SMat {
        let mut x = SMat::zeros();
        let e = self.rot;
        let erx = -(e * skew(&self.pos));
        x.fixed_view_mut::<3, 3>(0, 0).copy_from(&e);
        x.fixed_view_mut::<3, 3>(3, 3).copy_from(&e);
        x.fixed_view_mut::<3, 3>(3, 0).copy_from(&erx);
        x
    }

    /// Точка из координат A в координаты B.
    pub fn point_to_child(&self, p: &Vec3) -> Vec3 {
        self.rot * (p - self.pos)
    }

    /// Точка из координат B в координаты A.
    pub fn point_to_parent(&self, p: &Vec3) -> Vec3 {
        self.rot.transpose() * p + self.pos
    }
}

/// Инерция твёрдого тела: масса, центр масс (в кадре тела) и тензор инерции относительно ЦМ.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Inertia {
    pub mass: f64,
    pub com: Vec3,
    pub i_com: Mat3,
}

impl Inertia {
    pub fn zero() -> Self {
        Self {
            mass: 0.0,
            com: Vec3::zeros(),
            i_com: Mat3::zeros(),
        }
    }

    pub fn new(mass: f64, com: Vec3, i_com: Mat3) -> Self {
        Self { mass, com, i_com }
    }

    pub fn point(mass: f64, com: Vec3) -> Self {
        Self {
            mass,
            com,
            i_com: Mat3::zeros(),
        }
    }

    pub fn solid_sphere(mass: f64, radius: f64, com: Vec3) -> Self {
        let i = 0.4 * mass * radius * radius;
        Self {
            mass,
            com,
            i_com: Mat3::from_diagonal_element(i),
        }
    }

    /// Сплошной цилиндр, ось — по локальной оси `axis` (0 = x, 1 = y, 2 = z).
    pub fn solid_cylinder(mass: f64, radius: f64, length: f64, axis: usize, com: Vec3) -> Self {
        let i_axial = 0.5 * mass * radius * radius;
        let i_trans = mass * (3.0 * radius * radius + length * length) / 12.0;
        let mut d = Vec3::from_element(i_trans);
        d[axis] = i_axial;
        Self {
            mass,
            com,
            i_com: Mat3::from_diagonal(&d),
        }
    }

    /// Сплошной брус с полуразмерами `half`.
    pub fn solid_box(mass: f64, half: Vec3, com: Vec3) -> Self {
        let (a, b, c) = (2.0 * half.x, 2.0 * half.y, 2.0 * half.z);
        let d = Vec3::new(b * b + c * c, a * a + c * c, a * a + b * b) * (mass / 12.0);
        Self {
            mass,
            com,
            i_com: Mat3::from_diagonal(&d),
        }
    }

    /// Сегмент тела по антропометрическим таблицам: масса, ЦМ и радиус гирации
    /// относительно ЦМ (поперёк сегмента), ось сегмента — локальная z.
    pub fn segment_z(mass: f64, com: Vec3, radius_of_gyration: f64, axial_radius: f64) -> Self {
        let it = mass * radius_of_gyration * radius_of_gyration;
        let ia = 0.5 * mass * axial_radius * axial_radius;
        Self {
            mass,
            com,
            i_com: Mat3::from_diagonal(&Vec3::new(it, it, ia)),
        }
    }

    /// Сложение двух тел, заданных в одном кадре (теорема Штейнера).
    pub fn combine(&self, other: &Inertia) -> Inertia {
        let m = self.mass + other.mass;
        if m <= 0.0 {
            return Inertia::zero();
        }
        let com = (self.com * self.mass + other.com * other.mass) / m;
        let shift = |i: &Inertia| {
            let d = i.com - com;
            i.i_com + (Mat3::identity() * d.norm_squared() - d * d.transpose()) * i.mass
        };
        Inertia {
            mass: m,
            com,
            i_com: shift(self) + shift(other),
        }
    }

    /// Пространственная инерция относительно начала кадра тела:
    /// `[Ic + m·c×·c×ᵀ, m·c×; m·c×ᵀ, m·1]`.
    pub fn matrix(&self) -> SMat {
        assert_finite(&[self.mass], "масса");
        assert_finite(self.com.as_slice(), "центр масс");
        assert_finite(self.i_com.as_slice(), "тензор инерции");
        let c = skew(&self.com);
        let m = self.mass;
        let mut out = SMat::zeros();
        out.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(self.i_com + c * c.transpose() * m));
        out.fixed_view_mut::<3, 3>(0, 3).copy_from(&(c * m));
        out.fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&(c.transpose() * m));
        out.fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(Mat3::identity() * m));
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Rotation3;

    fn sample_xform() -> Xform {
        let r = Rotation3::from_euler_angles(0.3, -0.7, 1.1).into_inner();
        Xform::from_pose(&r, &Vec3::new(0.4, -1.2, 2.0))
    }

    #[test]
    fn xform_matches_matrix_form() {
        let x = sample_xform();
        let m = SVec::new(0.1, -0.2, 0.3, 1.0, 2.0, -3.0);
        let via_fn = x.apply_motion(&m);
        let via_mat = x.motion_matrix() * m;
        assert!((via_fn - via_mat).norm() < 1e-12);
        // Силы: X* = X⁻ᵀ
        let f = SVec::new(0.5, 0.1, -0.4, 3.0, -1.0, 2.0);
        let via_mat_f = x.motion_matrix().try_inverse().unwrap().transpose() * f;
        assert!((x.apply_force(&f) - via_mat_f).norm() < 1e-12);
        // Xᵀ f
        assert!((x.inv_apply_force(&f) - x.motion_matrix().transpose() * f).norm() < 1e-12);
        assert!((x.inv_apply_motion(&x.apply_motion(&m)) - m).norm() < 1e-12);
    }

    #[test]
    fn power_is_frame_invariant() {
        // m·f не зависит от кадра — базовое свойство пары «движение/сила».
        let x = sample_xform();
        let m = SVec::new(0.1, -0.2, 0.3, 1.0, 2.0, -3.0);
        let f = SVec::new(0.5, 0.1, -0.4, 3.0, -1.0, 2.0);
        let p_a = m.dot(&f);
        let p_b = x.apply_motion(&m).dot(&x.apply_force(&f));
        assert!((p_a - p_b).abs() < 1e-12);
    }

    #[test]
    fn composition_and_inverse() {
        let a = sample_xform();
        let b = Xform::from_pose(
            &Rotation3::from_euler_angles(-0.2, 0.5, 0.1).into_inner(),
            &Vec3::new(1.0, 0.0, -0.5),
        );
        let ab = a.then(&b);
        assert!((ab.motion_matrix() - b.motion_matrix() * a.motion_matrix()).norm() < 1e-12);
        let id = a.then(&a.inverse());
        assert!((id.motion_matrix() - SMat::identity()).norm() < 1e-12);
        let p = Vec3::new(0.3, 0.2, -0.1);
        assert!((a.point_to_parent(&a.point_to_child(&p)) - p).norm() < 1e-12);
    }

    #[test]
    fn inertia_combine_matches_spatial_sum() {
        let a = Inertia::solid_box(2.0, Vec3::new(0.1, 0.2, 0.3), Vec3::new(0.1, 0.0, -0.2));
        let b = Inertia::solid_sphere(1.5, 0.05, Vec3::new(-0.3, 0.4, 0.0));
        let c = a.combine(&b);
        assert!((c.matrix() - (a.matrix() + b.matrix())).norm() < 1e-12);
    }
}
