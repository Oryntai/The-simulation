use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};

pub(crate) type Frame = Isometry3<f64>;
pub(crate) type Vec3 = Vector3<f64>;

/// Пинхол с дисторсией Брауна — Конради. В кадре датчика X вправо, Y вниз, Z вперёд.
/// Пиксель (x, y) занимает [x, x+1] × [y, y+1], его центр имеет координаты (x+.5, y+.5).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Intrinsics {
    pub width: u32,
    pub height: u32,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    /// k1, k2, k3 по возрастающим степеням r².
    pub radial: [f64; 3],
    /// p1, p2.
    pub tangential: [f64; 2],
}

impl Intrinsics {
    pub fn validate(&self) -> Result<(), String> {
        if self.width == 0
            || self.height == 0
            || ![self.fx, self.fy, self.cx, self.cy]
                .iter()
                .chain(&self.radial)
                .chain(&self.tangential)
                .all(|v| v.is_finite())
            || self.fx <= 0.0
            || self.fy <= 0.0
        {
            return Err("неверные внутренние параметры камеры".into());
        }
        Ok(())
    }

    /// Проекция конечной точки перед камерой. Координаты за краем кадра допустимы.
    pub fn project(&self, point_in_camera_m: [f64; 3]) -> Result<Option<[f64; 2]>, String> {
        self.validate()?;
        let [x, y, z] = point_in_camera_m;
        if ![x, y, z].iter().all(|v| v.is_finite()) {
            return Err("нечисловая точка проекции".into());
        }
        if z <= 0.0 {
            return Ok(None);
        }
        let (xy, _) = self.distort(x / z, y / z);
        let pixel = [self.fx * xy[0] + self.cx, self.fy * xy[1] + self.cy];
        if pixel.iter().any(|v| !v.is_finite()) {
            return Err("проекция вышла за диапазон f64".into());
        }
        Ok(Some(pixel))
    }

    pub(crate) fn ray(&self, px: f64, py: f64) -> Result<[f64; 2], String> {
        let target = [(px - self.cx) / self.fx, (py - self.cy) / self.fy];
        if target.iter().any(|v| !v.is_finite()) {
            return Err("луч пикселя вышел за диапазон f64".into());
        }
        if self.radial == [0.0; 3] && self.tangential == [0.0; 2] {
            return Ok(target);
        }
        let [mut x, mut y] = target;
        // Порог решения в пикселях на четыре порядка строже допуска проекции 0.1 px.
        // Ограничение итераций прекращает поиск при вырожденной калибровке.
        const PIXEL_RESIDUAL: f64 = 0.000_01;
        const NEWTON_STEPS: usize = 32;
        for _ in 0..NEWTON_STEPS {
            let (value, [a, b, c, d]) = self.distort(x, y);
            let det = a * d - b * c;
            if !det.is_finite() || det <= 0.0 {
                break;
            }
            let dx = value[0] - target[0];
            let dy = value[1] - target[1];
            if (dx * self.fx).abs().max((dy * self.fy).abs()) <= PIXEL_RESIDUAL {
                return Ok([x, y]);
            }
            x -= (d * dx - b * dy) / det;
            y -= (a * dy - c * dx) / det;
        }
        Err(format!("дисторсия необратима в пикселе ({px}, {py})"))
    }

    fn distort(&self, x: f64, y: f64) -> ([f64; 2], [f64; 4]) {
        // Полином радиального смещения и его аналитический якобиан по Брауну — Конради.
        let [k1, k2, k3] = self.radial;
        let [p1, p2] = self.tangential;
        let r2 = x * x + y * y;
        let r4 = r2 * r2;
        let scale = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
        let slope = 2.0 * (k1 + 2.0 * k2 * r2 + 3.0 * k3 * r4);
        let cross = x * y * slope + 2.0 * (p1 * x + p2 * y);
        (
            [
                x * scale + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x),
                y * scale + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y,
            ],
            [
                scale + x * x * slope + 2.0 * p1 * y + 6.0 * p2 * x,
                cross,
                cross,
                scale + y * y * slope + 6.0 * p1 * y + 2.0 * p2 * x,
            ],
        )
    }
}

/// Поза в метрах и кватернион в порядке wxyz.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub position_m: [f64; 3],
    pub quat_wxyz: [f64; 4],
}

impl Pose {
    pub(crate) fn frame(self) -> Result<Frame, String> {
        let scale = self.quat_wxyz.iter().map(|v| v.abs()).fold(0.0, f64::max);
        if !self
            .position_m
            .iter()
            .chain(&self.quat_wxyz)
            .all(|v| v.is_finite())
            || scale == 0.0
        {
            return Err("нечисловая поза или нулевой кватернион".into());
        }
        let [w, x, y, z] = self.quat_wxyz.map(|v| v / scale);
        Ok(Frame::from_parts(
            Translation3::from(self.position_m),
            UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z)),
        ))
    }
}

pub(crate) fn pose(frame: &Frame) -> Pose {
    Pose {
        position_m: frame.translation.vector.into(),
        quat_wxyz: [
            frame.rotation.w,
            frame.rotation.i,
            frame.rotation.j,
            frame.rotation.k,
        ],
    }
}

pub(crate) fn checked(values: &[f64], subject: &str) -> Result<(), String> {
    if values.iter().any(|v| !v.is_finite()) {
        return Err(format!("{subject}: нечисловое значение"));
    }
    Ok(())
}
