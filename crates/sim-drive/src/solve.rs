use sim_runtime::{Forces, Object, Truth};
use sim_signal::{Command, Rng};

use crate::install::DriveObject;
use crate::mass::MassProbe;
use crate::model::{Model, Motion};

#[derive(Clone)]
pub(crate) struct DriveGroup {
    pub(crate) drives: Vec<DriveObject>,
    pub(crate) inertia: Vec<(f64, f64)>,
    dofs: Vec<usize>,
    probe: MassProbe,
    forecasts: Vec<Forecast>,
}

impl DriveGroup {
    pub(crate) fn new(drives: Vec<DriveObject>) -> Self {
        Self {
            drives,
            inertia: Vec::new(),
            dofs: Vec::new(),
            probe: MassProbe::default(),
            forecasts: Vec::new(),
        }
    }

    fn bind(&mut self, truth: &Truth<'_>) -> Result<(), String> {
        let joints = truth.engine().joints();
        for drive in &mut self.drives {
            let joint = joints
                .iter()
                .find(|joint| joint.name == drive.joint)
                .ok_or_else(|| format!("нет оси «{}»", drive.joint))?;
            let dof = joint.dof_adr + drive.limit.dof;
            drive.limit.axis = self
                .dofs
                .iter()
                .position(|&index| index == dof)
                .unwrap_or_else(|| {
                    let index = self.dofs.len();
                    self.dofs.push(dof);
                    index
                });
        }
        self.inertia.resize(self.dofs.len(), (0.0, 0.0));
        Ok(())
    }
}

#[derive(Clone, Copy, Default)]
struct Effort {
    transmission: f64,
    support: f64,
    loss_w: f64,
    lock: Option<f64>,
    brake: Option<f64>,
    scale: f64,
}

impl Effort {
    fn total(self) -> f64 {
        self.transmission + self.support
    }
}

#[derive(Clone, Copy)]
struct Forecast {
    from: Motion,
    to: Motion,
    anchors: [Option<f64>; 2],
    effort: Effort,
}

fn verify_response(
    drive: &DriveObject,
    forecast: Forecast,
    actual: Motion,
    h: f64,
    inverse: f64,
) -> Result<(), String> {
    let velocity_scale = forecast
        .from
        .v
        .abs()
        .max(forecast.to.v.abs())
        .max(actual.v.abs());
    let speed_error = (actual.v - forecast.to.v).abs();
    // Скорости из пробного и основного Euler могут отличаться порядком
    // операций. 32 epsilon — запас округления для этой сверки.
    if speed_error <= 32.0 * f64::EPSILON * velocity_scale {
        return Ok(());
    }
    let checked = evaluate(
        &drive.model,
        h,
        forecast.from,
        actual,
        forecast.effort,
        forecast.anchors,
    )?;
    let error = (checked.transmission - forecast.effort.transmission).abs()
        + (checked.support - forecast.effort.support).abs();
    let impulse_scale = (velocity_scale / inverse)
        .max(h * checked.total().abs())
        .max(h * forecast.effort.total().abs());
    let budget = crate::step_size::ACCURACY * impulse_scale;
    let impulse_error = h * error;
    if !budget.is_finite() || !impulse_error.is_finite() {
        return Err(format!(
            "привод «{}»: оценка невязки импульса вне численного диапазона",
            drive.id.name
        ));
    }
    if impulse_error <= budget {
        return Ok(());
    }
    Err(format!(
        "привод «{}»: фактический отклик оси отличается от пробного, невязка импульса {:.6e} при допуске {:.6e}. Нужна частота физики ≥ {:.0} Гц для повторной проверки",
        drive.id.name,
        h * error,
        budget,
        (2.0 / h).ceil()
    ))
}

impl Object for DriveGroup {
    fn update(
        &mut self,
        truth: &Truth<'_>,
        command: &Command,
        _: &mut Rng,
        out: &mut Forces,
    ) -> Result<(), String> {
        if self.dofs.is_empty() {
            self.bind(truth)?;
        }
        let sim = truth.engine();
        let dt = sim.timestep();
        let start = self
            .drives
            .iter()
            .map(|drive| drive.motion(truth))
            .collect::<Result<Vec<_>, _>>()?;
        for ((drive, &actual), &forecast) in self.drives.iter().zip(&start).zip(&self.forecasts) {
            verify_response(
                drive,
                forecast,
                actual,
                dt,
                self.inertia[drive.limit.axis].1,
            )?;
        }
        for (drive, &motion) in self.drives.iter_mut().zip(&start) {
            drive
                .prepare_step(
                    truth.time(),
                    dt,
                    motion,
                    command,
                    truth.command_sequence(drive.id.command),
                )
                .map_err(|reason| format!("привод «{}»: {reason}", drive.id.name))?;
        }
        self.probe.measure(sim, &self.dofs, &mut self.inertia)?;
        self.check_accuracy(dt, sim.model().opt.integrator as u32)?;
        let free = self.probe.free_velocity(sim, &self.dofs)?;
        let mut efforts = if self.forecasts.is_empty() {
            vec![Effort::default(); self.drives.len()]
        } else {
            self.forecasts
                .iter()
                .map(|forecast| forecast.effort)
                .collect()
        };
        let mut sum = vec![0.0; self.dofs.len()];
        let mut next = Vec::with_capacity(efforts.len());
        // Остаток уравнения F = F(q₁(F), v₁(F)). Округление
        // ограничиваем намного жёстче 1% бюджета физического шага.
        // Пробный шаг использует итерационный решатель контактов MuJoCo:
        // при перестройке активных ограничений допуск sqrt(epsilon).
        // Для прямого умножения блока M⁻¹ достаточно запаса округления.
        // Оба допуска существенно меньше локального бюджета 1%.
        let tolerance = if self.probe.uses_trial_step() {
            f64::EPSILON.sqrt()
        } else {
            16.0 * f64::EPSILON
        };
        // При коэффициенте сжатия 1/2 за 64 итерации остаётся 2⁻⁶⁴
        // начальной невязки. Для переключений без сходимости — отказ.
        let mut residuals = [0.0_f64; 3];
        for _ in 0..64 {
            residuals = [0.0; 3];
            sum.fill(0.0);
            for (drive, effort) in self.drives.iter().zip(&efforts) {
                sum[drive.limit.axis] += effort.total();
            }
            let velocity = self.probe.respond(sim, &self.dofs, &free, &sum)?;
            next.clear();
            let mut converged = true;
            for ((drive, &initial), &trial) in self.drives.iter().zip(&start).zip(&efforts) {
                let v = velocity[drive.limit.axis];
                let end = Motion {
                    q: v.mul_add(dt, initial.q),
                    v,
                };
                let result = evaluate(
                    &drive.model, dt, initial, end, trial,
                    [drive.model.lock_anchor, drive.model.brake_anchor],
                ).map_err(|reason| format!(
                    "привод «{}»: {reason}. Нужна частота физики ≥ {:.0} Гц для повторной проверки",
                    drive.id.name, (2.0 / dt).ceil()
                ))?;
                let scale = result.scale;
                // Около нулевой скорости разность упругого и полного усилий
                // может терять значащие разряды. Бюджет потерь в Вт.
                let speed_scale = drive.model.motor.v.abs() / drive.model.config.mechanical.ratio
                    + initial.v.abs()
                    + end.v.abs();
                let loss_scale = (scale * speed_scale)
                    .max(result.loss_w.abs())
                    .max(trial.loss_w.abs());
                if !loss_scale.is_finite() {
                    return Err("масштаб невязки потерь вне численного диапазона".into());
                }
                residuals[0] = residuals[0].max((result.transmission - trial.transmission).abs());
                residuals[1] = residuals[1].max((result.support - trial.support).abs());
                residuals[2] = residuals[2].max((result.loss_w - trial.loss_w).abs());
                converged &=
                    smaller_error(result.transmission, trial.transmission, scale, tolerance)
                        && smaller_error(result.support, trial.support, scale, tolerance)
                        && (result.loss_w - trial.loss_w).abs() <= f64::EPSILON.sqrt() * loss_scale;
                next.push(result);
            }
            if converged {
                self.forecasts.clear();
                // В дискретном балансе следующий шаг ротора получит
                // в точности ту же переданную силу, что и канал MuJoCo.
                for (((drive, &joint), &effort), &solved) in
                    self.drives.iter_mut().zip(&start).zip(&efforts).zip(&next)
                {
                    let model = &mut drive.model;
                    let v = velocity[drive.limit.axis];
                    self.forecasts.push(Forecast {
                        from: joint,
                        to: Motion {
                            q: v.mul_add(dt, joint.q),
                            v,
                        },
                        anchors: [model.lock_anchor, model.brake_anchor],
                        effort,
                    });
                    model.driving_effort = effort.transmission;
                    model.effort = effort.total();
                    model.coupling_loss_w = effort.loss_w;
                    model.lock_anchor = solved.lock;
                    model.brake_anchor = solved.brake;
                    if let Some(coupling) = model.config.mechanical.coupling {
                        model.backlash = (model.motor.q / model.config.mechanical.ratio - joint.q)
                            .clamp(-coupling.half_gap, coupling.half_gap);
                    }
                    drive.publish(joint, out)?;
                }
                return Ok(());
            }
            std::mem::swap(&mut efforts, &mut next);
        }
        Err(format!(
            "не сошлось уравнение привода за 64 итерации, невязки передачи {:.3e}, опоры {:.3e}, потерь {:.3e} Вт. Нужна частота физики ≥ {:.0} Гц для повторной проверки",
            residuals[0],
            residuals[1],
            residuals[2],
            (2.0 / dt).ceil()
        ))
    }
}

fn smaller_error(value: f64, trial: f64, scale: f64, tolerance: f64) -> bool {
    if value == 0.0 {
        return trial == 0.0;
    }
    (value - trial).abs() <= tolerance * scale.max(value.abs()).max(trial.abs())
}

fn evaluate(
    model: &Model,
    h: f64,
    from: Motion,
    to: Motion,
    trial: Effort,
    anchors: [Option<f64>; 2],
) -> Result<Effort, String> {
    let m = model.config.mechanical;
    // У Euler q₁=q₀+h v₁. В координатах x=q−hv/2 получаем
    // x₁=x₀+h(v₀+v₁)/2. Сила средней точки обновляет энергию
    // линейной упругости в этих координатах без систематического дрейфа.
    let x0 = (-0.5 * h).mul_add(from.v, from.q);
    let x1 = (-0.5 * h).mul_add(to.v, to.q);
    let v = 0.5 * (from.v + to.v);
    let (transmission, loss_w, transmission_scale) = if let Some(c) = m.coupling {
        let mut rotor = model.clone();
        rotor.driving_effort = trial.transmission;
        rotor.coupling_loss_w = trial.loss_w;
        rotor.integrate(h, from, to)?;
        // Координаты ротора сдвинуты на полшага тем же способом.
        let v0 = model.motor.v / m.ratio;
        let v1 = rotor.motor.v / m.ratio;
        let r0 = (-0.5 * h).mul_add(v0, model.motor.q / m.ratio) - x0;
        let r1 = (-0.5 * h).mul_add(v1, rotor.motor.q / m.ratio) - x1;
        let speed = 0.5 * (v0 + v1) - v;
        let scale = c.stiffness
            * (x0.abs()
                + x1.abs()
                + model.motor.q.abs() / m.ratio
                + rotor.motor.q.abs() / m.ratio
                + c.half_gap)
            + c.damping * (v0.abs() + v1.abs() + v.abs());
        let (force, loss) = c.mean_force(r0, r1, speed);
        (force, loss, scale)
    } else {
        let mid = Motion {
            q: 0.5 * (x0 + x1) * m.ratio,
            v: v * m.ratio,
        };
        let phases = model.windings(h, mid, model.resistance()?)?;
        let torque = model.torque(phases.map(|p| p.mean), mid.q);
        let friction = m.friction.torque(mid.v);
        (
            m.rigid_effort(torque - friction, mid.v),
            0.0,
            (torque.abs() + friction.abs()) * m.ratio / m.backward,
        )
    };
    let stop = m.stop.map_or(0.0, |s| s.mean_force(x0, x1, v));
    let hold = |law: Option<crate::config::Hold>, anchor: Option<f64>| {
        let Some(law) = law.filter(|_| model.unpowered()) else {
            return (0.0, None);
        };
        // Алгебраически (x0+x1)/2, без потери точного нуля при постоянной скорости.
        let midpoint = (0.25 * h).mul_add(to.v - from.v, from.q);
        let (effort, center) = law.mean_force(x0, x1, midpoint, anchor.unwrap_or(from.q));
        (effort, Some(center))
    };
    let (lock_force, lock) = hold(m.lock, anchors[0]);
    let (brake_force, brake) = hold(m.brake, anchors[1]);
    let support = stop + lock_force + brake_force;
    let scale = transmission_scale
        + stop.abs()
        + lock_force.abs()
        + brake_force.abs()
        + (x0.abs() + x1.abs())
            * (m.stop.map_or(0.0, |s| s.stiffness)
                + m.brake.map_or(0.0, |s| s.stiffness)
                + m.lock.map_or(0.0, |s| s.stiffness));
    if ![
        transmission,
        support,
        transmission + support,
        loss_w,
        scale,
        x0,
        x1,
    ]
    .into_iter()
    .chain(lock)
    .chain(brake)
    .all(f64::is_finite)
    {
        return Err("неконечные усилие, потери или положение при неявном шаге".into());
    }
    Ok(Effort {
        transmission,
        support,
        loss_w,
        lock,
        brake,
        scale,
    })
}
