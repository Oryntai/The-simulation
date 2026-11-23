use std::f64::consts::TAU;

use sim_signal::Time;

use crate::config::{Config, Electrical, Mode};
use crate::control::Control;
use crate::electric::{Bridge, Winding, winding};
use crate::input::BridgeCommand;
#[cfg(test)]
use crate::mechanical::hold;

#[derive(Clone, Copy, Default)]
pub(crate) struct Motion {
    pub q: f64,
    pub v: f64,
}

#[derive(Clone, Copy, Default)]
pub(crate) struct Work {
    pub electrical: f64,
    pub copper: f64,
    pub friction: f64,
    pub gear: f64,
    pub damping: f64,
    pub bridge_dump: f64,
    pub output: f64,
    initial_storage: f64,
}

impl Work {
    pub fn residual(self, stored: f64) -> f64 {
        self.residual_with_output(stored, self.output)
    }

    fn residual_with_output(self, stored: f64, output: f64) -> f64 {
        self.initial_storage + self.electrical
            - self.copper
            - self.friction
            - self.gear
            - self.damping
            - self.bridge_dump
            - output
            - stored
    }
}

#[derive(Clone, Copy)]
pub(crate) struct Interval {
    pub supply: f64,
    pub bridge: Bridge,
    pub bus_power_w: f64,
    pub end_effort: Option<f64>,
}

#[derive(Clone)]
pub(crate) struct Model {
    pub config: Config,
    pub current: [f64; 2],
    pub voltage: [f64; 2],
    pub temperatures: Option<[f64; 2]>,
    pub motor: Motion,
    pub work: Work,
    pub effort: f64,
    pub(crate) driving_effort: f64,
    pub(crate) coupling_loss_w: f64,
    actual_output: f64,
    pub backlash: f64,
    pub limit_exceeded: bool,
    pub lost_steps: f64,
    pub power: Option<bool>,
    pub bus_voltage: Option<f64>,
    pub bridge_request: Option<BridgeCommand>,
    pub interval: Option<Interval>,
    bridge: Bridge,
    pub control: Control,
    previous: Option<(Time, Motion)>,
    phase_origin: f64,
    basin: f64,
    pub(crate) lock_anchor: Option<f64>,
    pub(crate) brake_anchor: Option<f64>,
}

impl Model {
    pub(crate) fn new(config: Config) -> Self {
        Self {
            config,
            current: [0.0; 2],
            voltage: [0.0; 2],
            temperatures: config.thermal.map(|t| t.initial),
            motor: Motion::default(),
            work: Work::default(),
            effort: 0.0,
            driving_effort: 0.0,
            coupling_loss_w: 0.0,
            actual_output: 0.0,
            backlash: 0.0,
            limit_exceeded: false,
            lost_steps: 0.0,
            power: None,
            bus_voltage: None,
            bridge_request: None,
            interval: None,
            bridge: Bridge::Open,
            control: Control::new(),
            previous: None,
            phase_origin: 0.0,
            basin: 0.0,
            lock_anchor: None,
            brake_anchor: None,
        }
    }

    #[cfg(test)]
    pub(crate) fn update(
        &mut self,
        time: Time,
        joint: Motion,
        target: Option<f64>,
    ) -> Result<(), String> {
        self.update_at(time, joint, target, None)
    }

    #[cfg(test)]
    fn update_at(
        &mut self,
        time: Time,
        joint: Motion,
        target: Option<f64>,
        physics_step: Option<f64>,
    ) -> Result<(), String> {
        if ![joint.q, joint.v].into_iter().all(f64::is_finite) {
            return Err("координата и скорость соединения должны быть конечными".into());
        }
        let m = self.config.mechanical;
        self.finish(time, joint, physics_step)?;
        self.command(time, joint, target, None)?;

        let driving_effort = if let Some(coupling) = m.coupling {
            let relative = self.motor.q / m.ratio - joint.q;
            self.backlash = relative.clamp(-coupling.half_gap, coupling.half_gap);
            let (effort, loss) = coupling.force(relative, self.motor.v / m.ratio - joint.v);
            self.coupling_loss_w = loss;
            effort
        } else {
            m.rigid_effort(
                self.torque(self.current, self.motor.q) - m.friction.torque(self.motor.v),
                self.motor.v,
            )
        };
        let stopped = m
            .stop
            .map(|stop| stop.force(joint.q, joint.v))
            .unwrap_or(0.0);
        let unpowered = self.unpowered();
        let locked = hold(m.lock, &mut self.lock_anchor, joint.q, unpowered);
        let braked = hold(m.brake, &mut self.brake_anchor, joint.q, unpowered);
        self.driving_effort = driving_effort;
        self.effort = driving_effort + stopped + locked + braked;
        let residual = self.world_energy_residual(joint);
        if ![
            self.effort,
            self.motor.q,
            self.motor.v,
            self.coupling_loss_w,
            residual,
            self.work.residual(self.stored(joint)),
        ]
        .into_iter()
        .all(f64::is_finite)
        {
            return Err(
                "усилие, состояние ротора или энергобаланс вне численного диапазона".into(),
            );
        }
        Ok(())
    }

    pub(crate) fn finish(
        &mut self,
        time: Time,
        joint: Motion,
        physics_step: Option<f64>,
    ) -> Result<(), String> {
        if ![joint.q, joint.v].into_iter().all(f64::is_finite) {
            return Err("координата и скорость соединения должны быть конечными".into());
        }
        let m = self.config.mechanical;
        if let Some((past, old)) = self.previous {
            let ns = time
                .0
                .checked_sub(past.0)
                .filter(|&ns| ns > 0)
                .ok_or("не возрастающее время привода")?;
            // MuJoCo уже прошёл интервал с прошлым усилием. Его работа
            // равна F·Δq, даже когда расчёт двигателя внутри шага дал
            // другую силу. Упор и удержание здесь за границей баланса.
            self.actual_output = self
                .driving_effort
                .mul_add(joint.q - old.q, self.actual_output);
            // Шкала событий квантуется до наносекунды. Обмен импульсом
            // обязан использовать тот же вещественный шаг, что и MuJoCo.
            let dt = physics_step.unwrap_or_else(|| Time(ns).as_secs());
            if !dt.is_finite() || dt <= 0.0 {
                return Err("период физики должен быть конечным и положительным".into());
            }
            self.integrate(dt, old, joint)?;
        } else {
            self.motor = Motion {
                q: joint.q * m.ratio,
                v: joint.v * m.ratio,
            };
            self.phase_origin = self.motor.q;
            if m.coupling.is_some() && self.motor.v != 0.0 {
                // Из начального движения берётся энергия ротора на нулевом такте.
                self.work.initial_storage = self.stored(joint);
            }
        }
        self.previous = Some((time, joint));
        Ok(())
    }

    pub(crate) fn command(
        &mut self,
        time: Time,
        joint: Motion,
        target: Option<f64>,
        sequence: Option<u64>,
    ) -> Result<(), String> {
        let regulator_power = self.bridge_request.map_or(self.power, |request| {
            Some(self.power != Some(false) && request == BridgeCommand::Drive)
        });
        self.control.update(
            self.config,
            time,
            target.map(|value| (value, sequence)),
            joint.q,
            joint.v,
            regulator_power,
        )?;
        self.bridge = match self.bridge_request {
            Some(_) if self.power == Some(false) => Bridge::Coast,
            Some(BridgeCommand::Coast) => Bridge::Coast,
            Some(BridgeCommand::Short) => Bridge::Short,
            Some(BridgeCommand::Drive) if matches!(self.control.bridge, Bridge::Open) => {
                Bridge::Coast
            }
            _ => self.control.bridge,
        };
        if matches!(self.bridge, Bridge::Open) {
            self.work.bridge_dump += self.magnetic_j();
            self.current = [0.0; 2];
            self.voltage = [0.0; 2];
        }
        let electrical = self.electrical();
        if electrical.inductance == 0.0 {
            for index in 0..self.phases() {
                let end = winding(
                    electrical,
                    self.resistance()?,
                    self.current[index],
                    self.config.electrical.k * self.phase(self.motor.q)[index] * self.motor.v,
                    self.bridge,
                    index,
                    0.0,
                )?;
                self.current[index] = end.current;
                self.voltage[index] = end.voltage;
                self.limit_exceeded |= end.limit_exceeded;
            }
        }
        self.count_steps()
    }

    pub(crate) fn unpowered(&self) -> bool {
        self.power.map_or_else(
            || matches!(self.bridge, Bridge::Open | Bridge::Coast),
            |on| !on,
        )
    }

    pub(crate) fn electrical_feedback(&self) -> bool {
        // При снятом разрешении мост с диодами ещё передаёт ток и момент.
        self.bridge_request.is_some() || !self.unpowered()
    }

    fn electrical(&self) -> Electrical {
        let mut electrical = self.config.electrical;
        if let Some(voltage) = self.bus_voltage {
            electrical.supply = voltage;
        }
        electrical
    }

    pub(crate) fn world_energy_residual(&self, joint: Motion) -> f64 {
        self.work
            .residual_with_output(self.stored(joint), self.actual_output)
    }

    fn magnetic_j(&self) -> f64 {
        0.5 * self.config.electrical.inductance * self.current.iter().map(|i| i * i).sum::<f64>()
    }

    pub(crate) fn stored(&self, joint: Motion) -> f64 {
        let m = self.config.mechanical;
        self.magnetic_j()
            + m.coupling
                .map(|coupling| {
                    let d = coupling.deflection(self.motor.q / m.ratio - joint.q);
                    0.5 * (coupling.inertia_motor * self.motor.v * self.motor.v
                        + coupling.stiffness * d * d)
                })
                .unwrap_or(0.0)
    }

    pub(crate) fn phases(&self) -> usize {
        if matches!(self.config.mode, Mode::StepRate { .. }) {
            2
        } else {
            1
        }
    }

    pub(crate) fn resistance(&self) -> Result<f64, String> {
        let base = self.config.electrical.resistance;
        let resistance = self
            .config
            .thermal
            .zip(self.temperatures)
            .map_or(base, |(t, temps)| {
                base * (1.0 + t.alpha * (temps[0] - t.reference))
            });
        if !resistance.is_finite() || resistance <= 0.0 {
            return Err("сопротивление обмотки R(T) должно быть конечным и положительным".into());
        }
        Ok(resistance)
    }

    pub(crate) fn phase(&self, q: f64) -> [f64; 2] {
        if let Mode::StepRate { poles, .. } = self.config.mode {
            let angle = (q - self.phase_origin) * poles;
            [-angle.sin(), angle.cos()]
        } else {
            [1.0, 0.0]
        }
    }

    pub(crate) fn torque(&self, [a, b]: [f64; 2], q: f64) -> f64 {
        let [x, y] = self.phase(q);
        self.config.electrical.k * a.mul_add(x, b * y)
    }

    fn count_steps(&mut self) -> Result<(), String> {
        if let Mode::StepRate {
            poles,
            steps_per_period,
            ..
        } = self.config.mode
            && self.bridge.mode() == BridgeCommand::Drive
        {
            // Необратимой потерей считаем полный проскок электрического периода.
            // При колебаниях на границе ещё один период в ту же сторону не теряется.
            let error = self.control.steps / steps_per_period
                - (self.motor.q - self.phase_origin) * poles / TAU;
            let slipped = (error - self.basin).trunc();
            self.lost_steps += slipped.abs() * steps_per_period;
            self.basin += slipped;
            if !self.lost_steps.is_finite()
                || self.lost_steps >= (1_u64 << f64::MANTISSA_DIGITS) as f64
            {
                return Err("счётчик пропущенных шагов исчерпал точность".into());
            }
        }
        Ok(())
    }

    pub(crate) fn integrate(&mut self, dt: f64, from: Motion, to: Motion) -> Result<(), String> {
        let m = self.config.mechanical;
        let mut frequency = 0.0_f64;
        if let Some(c) = m.coupling {
            let reflected = c.inertia_motor * m.ratio * m.ratio;
            let e = self.config.electrical;
            let electrical = if e.inductance == 0.0 {
                e.k * e.k / (self.resistance()? * c.inertia_motor)
            } else {
                e.k / (e.inductance * c.inertia_motor).sqrt()
            };
            frequency = frequency
                .max((c.stiffness / (reflected * m.forward)).sqrt())
                .max(c.damping / (reflected * m.forward))
                .max(m.friction.max_slope() / c.inertia_motor)
                .max(electrical);
        }
        if let Some(t) = self.config.thermal {
            frequency = frequency
                .max(1.0 / (t.r_wh * t.c_w))
                .max((1.0 / t.r_wh + 1.0 / t.r_ha) / t.c_h);
        }
        // Восемь подшагов на самый короткий характерный интервал для схемы
        // средней точки. Если нужен больший бюджет, шаг физики следует уменьшить.
        let steps = (8.0 * dt * frequency).ceil().max(1.0);
        if !steps.is_finite() || steps > 4096.0 {
            return Err(
                "требуется меньший шаг физики: интегрирование привода просит больше 4096 подшагов"
                    .into(),
            );
        }
        let h = dt / steps;
        self.limit_exceeded = false;
        if m.coupling.is_some() {
            self.motor.q = (-0.5 * dt).mul_add(self.motor.v, self.motor.q);
        }
        let mut bus_energy = 0.0;
        let mut last_midpoint = from;
        for index in 0..steps as u32 {
            let alpha = (f64::from(index) + 0.5) / steps;
            let delta = to.q - from.q;
            let midpoint = Motion {
                q: delta.mul_add(alpha, from.q),
                v: if m.coupling.is_some() {
                    delta / dt
                } else {
                    (to.v - from.v).mul_add(alpha, from.v)
                },
            };
            bus_energy += self.substep(h, midpoint)?;
            last_midpoint = midpoint;
        }
        if m.coupling.is_some() {
            self.motor.q = (0.5 * dt).mul_add(self.motor.v, self.motor.q);
        } else {
            self.motor = Motion {
                q: to.q * m.ratio,
                v: to.v * m.ratio,
            };
        }
        self.interval = Some(Interval {
            supply: self.electrical().supply,
            bridge: self.bridge,
            bus_power_w: bus_energy / dt,
            end_effort: m.coupling.is_none().then(|| {
                let speed = last_midpoint.v * m.ratio;
                let shaft =
                    self.torque(self.current, last_midpoint.q * m.ratio) - m.friction.torque(speed);
                m.rigid_effort(shaft, speed)
            }),
        });
        Ok(())
    }

    fn substep(&mut self, h: f64, joint: Motion) -> Result<f64, String> {
        let m = self.config.mechanical;
        let start = self.motor;
        let resistance = self.resistance()?;
        let (mid, phases, shaft, gear, damping, output) = if let Some(c) = m.coupling {
            let effort = self.driving_effort;
            let loads = [m.load_on_motor(effort, -1.0), m.load_on_motor(effort, 1.0)];
            let mut speed = start.v;
            // Шаг по собственной моде ротора ограничен в integrate.
            // 32 итерации достаточно до округления при сжатии 1/4.
            let mut solved = None;
            for _ in 0..32 {
                let mid = Motion {
                    q: (0.5 * h * speed) + start.q,
                    v: speed,
                };
                let phases = self.windings(h, mid, resistance)?;
                let shaft = self.torque(phases.map(|p| p.mean), mid.q) - m.friction.torque(mid.v);
                let spin = spin(h, start.v, shaft, loads, c.inertia_motor, effort / m.ratio);
                let mean_speed = spin.travel / h;
                let scale = start
                    .v
                    .abs()
                    .max(speed.abs())
                    .max(mean_speed.abs())
                    .max(h * (shaft.abs() + loads[0].abs() + loads[1].abs()) / c.inertia_motor);
                if (mean_speed - speed).abs() <= f64::EPSILON.sqrt() * scale {
                    solved = Some((mid, phases, shaft, spin));
                    break;
                }
                speed = mean_speed;
            }
            let Some((mid, phases, shaft, spin)) = solved else {
                return Err("не сошлись ЭДС, трение и реакция ротора на внутреннем подшаге".into());
            };
            self.motor.q += spin.travel;
            self.motor.v = spin.speed;
            (
                mid,
                phases,
                shaft,
                spin.gear_j,
                h * self.coupling_loss_w,
                h * effort * joint.v,
            )
        } else {
            let mid = Motion {
                q: joint.q * m.ratio,
                v: joint.v * m.ratio,
            };
            let phases = self.windings(h, mid, resistance)?;
            let shaft = self.torque(phases.map(|p| p.mean), mid.q) - m.friction.torque(mid.v);
            let output = h * m.rigid_effort(shaft, mid.v) * joint.v;
            (mid, phases, shaft, h * shaft * mid.v - output, 0.0, output)
        };
        let friction = h * m.friction.torque(mid.v) * mid.v;
        let copper = phases.iter().map(|p| p.copper_j).sum::<f64>();
        self.current = phases.map(|p| p.current);
        self.voltage = phases.map(|p| p.voltage);
        self.limit_exceeded |= phases.iter().any(|p| p.limit_exceeded);
        let electrical = phases.iter().map(|p| p.electrical_j).sum::<f64>();
        self.work.electrical += electrical;
        self.work.copper += copper;
        self.work.gear += gear;
        self.work.output += output;
        self.work.friction += friction;
        self.work.damping += damping;
        if let Some((t, temperatures)) = self.config.thermal.zip(self.temperatures) {
            self.temperatures = Some(t.advance(
                temperatures,
                h,
                copper + t.winding_fraction * friction,
                (1.0 - t.winding_fraction) * friction + t.gear_fraction * (gear + damping),
            )?);
        }
        if [shaft, self.motor.q, self.motor.v, gear, damping, output]
            .into_iter()
            .all(f64::is_finite)
        {
            Ok(electrical)
        } else {
            Err("состояние ротора или работа вне численного диапазона".into())
        }
    }

    pub(crate) fn windings(
        &self,
        h: f64,
        at: Motion,
        resistance: f64,
    ) -> Result<[Winding; 2], String> {
        let p = self.electrical();
        let mut result = [Winding::default(); 2];
        for (index, winding_out) in result.iter_mut().enumerate().take(self.phases()) {
            *winding_out = winding(
                p,
                resistance,
                self.current[index],
                p.k * self.phase(at.q)[index] * at.v,
                self.bridge,
                index,
                h,
            )?;
        }
        Ok(result)
    }
}

struct Spin {
    speed: f64,
    travel: f64,
    gear_j: f64,
}

// На подшаге момент обмотки и переданное усилие постоянны. До смены
// знака v ускорение постоянно; в нуле реакция может держать ротор,
// если ускорения с обеих сторон направлены к нулю. В покое работа нулевая.
fn spin(h: f64, v: f64, shaft: f64, loads: [f64; 2], inertia: f64, ideal: f64) -> Spin {
    let acceleration = loads.map(|load| (shaft - load) / inertia);
    let from_rest = |duration| {
        let side = usize::from(acceleration[1] > 0.0);
        let a = if side == 0 {
            acceleration[0].min(0.0)
        } else {
            acceleration[1]
        };
        let speed = a * duration;
        let travel = 0.5 * duration * speed;
        Spin {
            speed,
            travel,
            gear_j: (loads[side] - ideal) * travel,
        }
    };
    if v == 0.0 {
        return from_rest(h);
    }
    let side = usize::from(v > 0.0);
    let a = acceleration[side];
    let speed = a.mul_add(h, v);
    if speed.is_sign_positive() == v.is_sign_positive() || speed == 0.0 {
        let travel = (0.5 * v + 0.5 * speed) * h;
        return Spin {
            speed,
            travel,
            gear_j: (loads[side] - ideal) * travel,
        };
    }
    let duration = -v / a;
    let travel = 0.5 * duration * v;
    let mut spin = from_rest((h - duration).max(0.0));
    spin.travel += travel;
    spin.gear_j += (loads[side] - ideal) * travel;
    spin
}
