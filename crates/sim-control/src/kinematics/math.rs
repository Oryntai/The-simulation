use super::{KinematicError, Result, Vec3};

pub(super) fn add(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|i| a[i] + b[i])
}

pub(super) fn sub(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|i| a[i] - b[i])
}

pub(super) fn scale(a: Vec3, k: f64) -> Vec3 {
    a.map(|v| v * k)
}

pub(super) fn dot(a: Vec3, b: Vec3) -> f64 {
    a.into_iter().zip(b).map(|(x, y)| x * y).sum()
}

pub(super) fn cross([x, y, z]: Vec3, [a, b, c]: Vec3) -> Vec3 {
    [y * c - z * b, z * a - x * c, x * b - y * a]
}

pub(super) fn norm([x, y, z]: Vec3) -> f64 {
    x.hypot(y).hypot(z)
}

pub(super) fn finite(values: &[f64]) -> bool {
    values.iter().all(|x| x.is_finite())
}

pub(super) fn unit(v: Vec3) -> Result<Vec3> {
    let [x, y, z] = normalize(v)?;
    Ok([x, y, z])
}

fn normalize<const N: usize>(mut v: [f64; N]) -> Result<[f64; N]> {
    let largest = v.iter().map(|x| x.abs()).fold(0.0, f64::max);
    if !finite(&v) || largest == 0.0 {
        return Err(KinematicError::Invalid(
            "нулевая или неконечная ось либо ориентация".into(),
        ));
    }
    v.iter_mut().for_each(|x| *x /= largest);
    let norm = v.iter().map(|x| x * x).sum::<f64>().sqrt();
    Ok(v.map(|x| x / norm))
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    translation: Vec3,
    quaternion: [f64; 4],
}

impl Pose {
    pub const IDENTITY: Self = Self {
        translation: [0.0; 3],
        quaternion: [1.0, 0.0, 0.0, 0.0],
    };

    /// Положение в метрах, кватернион движения из локальной системы в родительскую: [w,x,y,z].
    /// Ненулевой кватернион нормируется.
    pub fn new(translation: Vec3, quaternion: [f64; 4]) -> Result<Self> {
        if !finite(&translation) {
            return Err(KinematicError::Invalid("неконечное положение".into()));
        }
        Ok(Self {
            translation,
            quaternion: normalize(quaternion)?,
        })
    }

    pub fn translation(self) -> Vec3 {
        self.translation
    }
    pub fn quaternion(self) -> [f64; 4] {
        self.quaternion
    }

    pub fn compose(self, child: Self) -> Result<Self> {
        let ([a, b, c, d], [e, f, g, h]) = (self.quaternion, child.quaternion);
        let quaternion = [
            a * e - b * f - c * g - d * h,
            a * f + b * e + c * h - d * g,
            a * g - b * h + c * e + d * f,
            a * h + b * g - c * f + d * e,
        ];
        Self::new(
            add(self.translation, self.rotate(child.translation)),
            quaternion,
        )
    }

    pub fn inverse(self) -> Result<Self> {
        let [w, x, y, z] = self.quaternion;
        let mut inverse = Self {
            translation: [0.0; 3],
            quaternion: [w, -x, -y, -z],
        };
        inverse.translation = inverse.rotate(scale(self.translation, -1.0));
        if !finite(&inverse.translation) {
            return Err(KinematicError::Numerical(
                "переполнение при обращении позы".into(),
            ));
        }
        Ok(inverse)
    }

    pub(super) fn rotate(self, v: Vec3) -> Vec3 {
        let [w, x, y, z] = self.quaternion;
        // Матрица кватерниона. Умножение сначала строит коэффициенты порядка единицы.
        let [u, v_y, t] = v;
        [
            (1.0 - 2.0 * (y * y + z * z)) * u
                + (2.0 * (x * y - w * z)) * v_y
                + (2.0 * (x * z + w * y)) * t,
            (2.0 * (x * y + w * z)) * u
                + (1.0 - 2.0 * (x * x + z * z)) * v_y
                + (2.0 * (y * z - w * x)) * t,
            (2.0 * (x * z - w * y)) * u
                + (2.0 * (y * z + w * x)) * v_y
                + (1.0 - 2.0 * (x * x + y * y)) * t,
        ]
    }

    pub(super) fn from_rotation_vector(translation: Vec3, rotation: Vec3) -> Result<Self> {
        let angle = norm(rotation);
        if !angle.is_finite() {
            return Err(KinematicError::Numerical(
                "переполнение модуля поворота".into(),
            ));
        }
        if angle == 0.0 {
            return Self::new(translation, Self::IDENTITY.quaternion);
        }
        let (sin, cos) = (angle * 0.5).sin_cos();
        let [x, y, z] = unit(rotation)?.map(|x| x * sin);
        Self::new(translation, [cos, x, y, z])
    }

    pub(super) fn rotation(axis: Vec3, angle: f64) -> Result<Self> {
        let (sin, cos) = (0.5 * angle).sin_cos();
        let [x, y, z] = scale(axis, sin);
        Self::new([0.0; 3], [cos, x, y, z])
    }

    pub(super) fn rotation_vector(self) -> Vec3 {
        // Из двух знаков кватерниона выбираем кратчайший поворот.
        let [w, x, y, z] = self.quaternion;
        let v = if w < 0.0 { [-x, -y, -z] } else { [x, y, z] };
        let length = norm(v);
        if length == 0.0 {
            [0.0; 3]
        } else {
            let angle = 2.0 * length.atan2(w.abs());
            scale(v, angle / length)
        }
    }
}

pub(super) const AXES: [Vec3; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

pub(super) fn rotation_vector_axes(rotation: Vec3) -> Result<[Vec3; 3]> {
    let angle = norm(rotation);
    if !angle.is_finite() {
        return Err(KinematicError::Numerical(
            "переполнение модуля поворота".into(),
        ));
    }
    if angle == 0.0 {
        return Ok(AXES);
    }
    let axis = unit(rotation)?;
    // Левый якобиан SO(3): sinc(θ)I + (1−sinc(θ))aaᵀ + ((1−cos θ)/θ)[a]×.
    let sinc = angle.sin() / angle;
    let skew = angle.mul_add(0.5, 0.0).sin();
    let skew = if skew == 0.0 {
        0.0
    } else {
        (2.0 * skew / angle) * skew
    };
    Ok(AXES.map(|e| {
        add(
            add(scale(e, sinc), scale(axis, (1.0 - sinc) * dot(axis, e))),
            scale(cross(axis, e), skew),
        )
    }))
}
