use crate::config::{Coupling, Friction, Hold, Mechanics, Stop};

impl Friction {
    pub(crate) fn max_slope(self) -> f64 {
        // max |d(exp(−x²))/dx| = √(2/e), при |x|=1/√2.
        self.viscous
            + self.stiction / self.band
            + ((self.stiction - self.coulomb) * (2.0 / std::f64::consts::E).sqrt()) / self.stribeck
    }

    pub(crate) fn torque(self, speed: f64) -> f64 {
        // Кривая Страйбека с гауссовым спадом и линейной регуляризацией знака
        // в измеряемой полосе скоростей, как для Кулонова трения в sim-em.
        let s = speed / self.stribeck;
        let level = self.coulomb + (self.stiction - self.coulomb) * (-s * s).exp();
        level * (speed / self.band).clamp(-1.0, 1.0) + self.viscous * speed
    }
}

impl Mechanics {
    pub(crate) fn rigid_effort(self, shaft: f64, speed_motor: f64) -> f64 {
        if shaft * speed_motor >= 0.0 {
            shaft * self.ratio * self.forward
        } else {
            shaft * self.ratio / self.backward
        }
    }

    pub(crate) fn load_on_motor(self, effort: f64, speed_motor: f64) -> f64 {
        let load = effort / self.ratio;
        if load * speed_motor >= 0.0 {
            load / self.forward
        } else {
            load * self.backward
        }
    }
}

impl Coupling {
    pub(crate) fn deflection(self, relative: f64) -> f64 {
        relative - relative.clamp(-self.half_gap, self.half_gap)
    }

    pub(crate) fn mean_force(self, from: f64, to: f64, speed: f64) -> (f64, f64) {
        let viscous = self.damping * speed;
        if self.half_gap == 0.0 {
            return (
                self.stiffness * (0.5 * from + 0.5 * to) + viscous,
                viscous * speed,
            );
        }
        let force = |damping| {
            unilateral(
                from - self.half_gap,
                to - self.half_gap,
                self.stiffness,
                damping,
            ) - unilateral(
                -from - self.half_gap,
                -to - self.half_gap,
                self.stiffness,
                -damping,
            )
        };
        let total = force(viscous);
        (total, (total - force(0.0)) * speed)
    }

    #[cfg(test)]
    pub(crate) fn force(self, relative: f64, relative_speed: f64) -> (f64, f64) {
        self.mean_force(relative, relative, relative_speed)
    }
}

impl Stop {
    pub(crate) fn mean_force(self, from: f64, to: f64, speed: f64) -> f64 {
        unilateral(
            self.min - from,
            self.min - to,
            self.stiffness,
            -self.damping * speed,
        ) - unilateral(
            from - self.max,
            to - self.max,
            self.stiffness,
            self.damping * speed,
        )
    }

    #[cfg(test)]
    pub(crate) fn force(self, position: f64, speed: f64) -> f64 {
        self.mean_force(position, position, speed)
    }
}

impl Hold {
    /// Упругое удержание до предела, затем скольжение с переносом точки.
    pub(crate) fn mean_force(self, from: f64, to: f64, midpoint: f64, center: f64) -> (f64, f64) {
        let first = self.stiffness * (center - from);
        let last = self.stiffness * (center - to);
        if first.abs().max(last.abs()) <= self.effort {
            return (self.stiffness * (center - midpoint), center);
        }
        let mean = if first == last {
            first.clamp(-self.effort, self.effort)
        } else {
            -self.effort + positive_mean(first + self.effort, last + self.effort)
                - positive_mean(first - self.effort, last - self.effort)
        };
        let next_center = if last.abs() > self.effort {
            to + self.effort.copysign(last) / self.stiffness
        } else {
            center
        };
        (mean, next_center)
    }
}

#[cfg(test)]
pub(crate) fn hold(
    model: Option<Hold>,
    anchor: &mut Option<f64>,
    position: f64,
    enabled: bool,
) -> f64 {
    let Some(model) = model.filter(|_| enabled) else {
        *anchor = None;
        return 0.0;
    };
    let (force, center) =
        model.mean_force(position, position, position, anchor.unwrap_or(position));
    *anchor = Some(center);
    force
}

// Интеграл положительной части линейной функции по интервалу [0,1].
fn positive_mean(first: f64, last: f64) -> f64 {
    let low = first.min(last);
    let high = first.max(last);
    if high <= 0.0 {
        0.0
    } else if low >= 0.0 {
        0.5 * low + 0.5 * high
    } else {
        0.5 * high * (high / (high - low))
    }
}

// Интегрируем по части шага в контакте и исключаем растягивающее
// усилие при разгрузке. Положение меняется линейно между концами шага.
fn unilateral(first: f64, last: f64, stiffness: f64, damping: f64) -> f64 {
    let low = first.min(last);
    let high = first.max(last);
    if high < 0.0 {
        return 0.0;
    }
    let fraction = if low >= 0.0 { 1.0 } else { high / (high - low) };
    fraction
        * positive_mean(
            stiffness.mul_add(low.max(0.0), damping),
            stiffness.mul_add(high, damping),
        )
}
