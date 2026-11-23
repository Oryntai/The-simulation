//! Одномерные законы нагрузок на вращательную ось с заданными параметрами.

use crate::ProcessError;

#[derive(Clone, Copy, Debug)]
pub struct AngularStop {
    limit_rad: f64,
    lever_m: f64,
    stiffness_n_m: f64,
    damping_ns_m: f64,
}

impl AngularStop {
    pub fn new(
        limit_rad: f64,
        lever_m: f64,
        stiffness_n_m: f64,
        damping_ns_m: f64,
    ) -> Result<Self, ProcessError> {
        if ![limit_rad, lever_m, stiffness_n_m, damping_ns_m]
            .into_iter()
            .all(f64::is_finite)
            || lever_m <= 0.0
            || stiffness_n_m <= 0.0
            || damping_ns_m < 0.0
        {
            return Err(invalid("неверные параметры одностороннего углового упора"));
        }
        Ok(Self {
            limit_rad,
            lever_m,
            stiffness_n_m,
            damping_ns_m,
        })
    }

    /// Возвращает (нормальная сила, момент). Упор срабатывает со стороны роста угла.
    pub fn evaluate(self, angle_rad: f64, speed_rad_s: f64) -> Result<(f64, f64), ProcessError> {
        if !angle_rad.is_finite() || !speed_rad_s.is_finite() {
            return Err(invalid("неверное состояние оси у упора"));
        }
        // Виртуальная работа: x = r(q − q₀), dx/dq = r и M = −rF.
        let depth = self.lever_m * (angle_rad - self.limit_rad);
        if !depth.is_finite() {
            return Err(invalid("переполнение хода углового упора"));
        }
        if depth <= 0.0 {
            return Ok((0.0, 0.0));
        }
        let load = self.stiffness_n_m * depth + self.damping_ns_m * self.lever_m * speed_rad_s;
        let force = load.max(0.0);
        let moment = -self.lever_m * force;
        if !load.is_finite() || !moment.is_finite() {
            return Err(invalid("переполнение нагрузки углового упора"));
        }
        Ok((force, moment))
    }
}

#[derive(Clone, Copy, Debug)]
pub struct RotaryMedium {
    sine_moment_nm: f64,
    quadratic_drag_nm_s2: f64,
}

impl RotaryMedium {
    pub fn new(sine_moment_nm: f64, quadratic_drag_nm_s2: f64) -> Result<Self, ProcessError> {
        if !sine_moment_nm.is_finite()
            || !quadratic_drag_nm_s2.is_finite()
            || quadratic_drag_nm_s2 < 0.0
        {
            return Err(invalid("неверные параметры вращательной нагрузки среды"));
        }
        Ok(Self {
            sine_moment_nm,
            quadratic_drag_nm_s2,
        })
    }

    /// Гармоническая поправка к внешнему моменту и сопротивление против движения.
    pub fn moment(self, angle_rad: f64, speed_rad_s: f64) -> Result<f64, ProcessError> {
        if !angle_rad.is_finite() || !speed_rad_s.is_finite() {
            return Err(invalid("неверное состояние оси в среде"));
        }
        let moment = self.sine_moment_nm * angle_rad.sin()
            - self.quadratic_drag_nm_s2 * speed_rad_s * speed_rad_s.abs();
        if !moment.is_finite() {
            return Err(invalid("переполнение вращательной нагрузки среды"));
        }
        Ok(moment)
    }
}

fn invalid(message: &str) -> ProcessError {
    ProcessError::InvalidInput(message.into())
}
