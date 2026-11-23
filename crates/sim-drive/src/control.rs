use std::collections::VecDeque;
use std::f64::consts::TAU;

use sim_signal::Time;

use crate::config::{Config, Mode, VelocityControl};
use crate::electric::Bridge;

#[derive(Clone)]
pub(crate) struct Control {
    next: u64,
    last: Time,
    seen: Option<(f64, Option<u64>)>,
    enabled: bool,
    enabled_since: Time,
    pending: VecDeque<(Time, f64, Option<u64>)>,
    pub accepted: Option<f64>,
    pub receipt: Option<(u64, Time, f64)>,
    integral: f64,
    fractional_steps: f64,
    pub steps: f64,
    held_step_rate: f64,
    pub bridge: Bridge,
}

impl Control {
    pub(crate) fn new() -> Self {
        Self {
            next: 0,
            last: Time::ZERO,
            seen: None,
            enabled: true,
            enabled_since: Time::ZERO,
            pending: VecDeque::new(),
            accepted: None,
            receipt: None,
            integral: 0.0,
            fractional_steps: 0.0,
            steps: 0.0,
            held_step_rate: 0.0,
            bridge: Bridge::Open,
        }
    }

    pub(crate) fn clear_command(&mut self) {
        self.seen = None;
        self.accepted = None;
        self.receipt = None;
        self.pending.clear();
        self.integral = 0.0;
        self.bridge = Bridge::Open;
    }

    pub(crate) fn update(
        &mut self,
        config: Config,
        now: Time,
        incoming: Option<(f64, Option<u64>)>,
        q: f64,
        v: f64,
        powered: Option<bool>,
    ) -> Result<(), String> {
        self.receipt = None;
        if let Some((value, sequence)) = incoming {
            if !value.is_finite() {
                return Err("команда привода должна быть конечной".into());
            }
            if self.seen != incoming {
                let due = now
                    .0
                    .checked_add(config.delay.0)
                    .ok_or("задержка выходит за шкалу времени")?;
                self.pending.push_back((Time(due), value, sequence));
                self.seen = incoming;
            }
        }
        let enabled = powered != Some(false);
        if enabled && !self.enabled {
            self.enabled_since = now;
        }
        self.enabled = enabled;
        if !enabled {
            self.bridge = Bridge::Open;
            self.integral = 0.0;
            self.held_step_rate = 0.0;
        }
        if matches!(config.mode, Mode::Pwm) {
            // Усреднённый мост не имеет состояния регулятора. Если тактов
            // несколько, только последний определяет удержанное напряжение.
            let last = config.control.last_due(now);
            if last >= self.next {
                let due = config
                    .control
                    .tick(last)
                    .ok_or("такт моста вне шкалы времени")?;
                self.deliver(due);
                if enabled && due >= self.enabled_since {
                    self.bridge = self.regulate(config, q, v, 0.0)?;
                }
                self.last = due;
                self.next = last.checked_add(1).ok_or("исчерпаны такты моста")?;
            }
            self.deliver(now);
            return Ok(());
        }
        let mut tick = config.control.tick(self.next);
        while let Some(due) = tick.filter(|&due| due <= now) {
            // При несовпадающих сетках в одном шаге физики могут
            // оказаться и тик, и доставка более поздней команды.
            self.deliver(due);
            // Снятие питания действует сразу. Восстановление питания
            // даёт регулятору право исполнять только последующие тики.
            if enabled && due >= self.enabled_since {
                let dt = Time(due.0 - self.last.0.max(self.enabled_since.0)).as_secs();
                self.bridge = self.regulate(config, q, v, dt)?;
            }
            self.last = due;
            self.next += 1;
            tick = config.control.tick(self.next);
        }
        self.deliver(now);
        Ok(())
    }

    fn deliver(&mut self, now: Time) {
        while let Some(&(due, value, sequence)) = self.pending.front() {
            if due > now {
                break;
            }
            self.pending.pop_front();
            self.accepted = Some(value);
            self.receipt = sequence.map(|sequence| (sequence, now, value));
        }
    }

    fn regulate(&mut self, config: Config, q: f64, v: f64, dt: f64) -> Result<Bridge, String> {
        let Some(command) = self.accepted else {
            return Ok(Bridge::Open);
        };
        let limit = config.electrical.limit;
        let request = match config.mode {
            Mode::Pwm => Bridge::Duty(command.clamp(-1.0, 1.0)),
            Mode::Current => Bridge::Current([command.clamp(-limit, limit), 0.0]),
            Mode::Velocity(velocity) => {
                Bridge::Current([self.speed(velocity, command, v, limit, dt)?, 0.0])
            }
            Mode::Position {
                min,
                max,
                deadband,
                gain,
                velocity,
            } => {
                let error = command.clamp(min, max) - q;
                let desired = (error.abs() - deadband).max(0.0).copysign(error) * gain;
                if !desired.is_finite() {
                    return Err(
                        "задание скорости из ошибки положения вне численного диапазона".into(),
                    );
                }
                if error.abs() <= deadband {
                    self.integral = 0.0;
                    Bridge::Current([0.0; 2])
                } else {
                    Bridge::Current([self.speed(velocity, desired, v, limit, dt)?, 0.0])
                }
            }
            Mode::StepRate {
                limit,
                current,
                steps_per_period,
                ..
            } => {
                self.fractional_steps += self.held_step_rate * dt;
                self.held_step_rate = command.clamp(-limit, limit);
                let whole = self.fractional_steps.trunc();
                self.fractional_steps -= whole;
                self.steps += whole;
                // После 2^53 f64 уже не различает соседние целые шаги.
                if self.steps.abs() >= (1_u64 << f64::MANTISSA_DIGITS) as f64 {
                    return Err("счётчик команд шагового двигателя исчерпал точность".into());
                }
                let phase = self.steps.rem_euclid(steps_per_period) * (TAU / steps_per_period);
                Bridge::Current([current * phase.cos(), current * phase.sin()])
            }
        };
        Ok(request)
    }

    fn speed(
        &mut self,
        control: VelocityControl,
        target: f64,
        speed: f64,
        limit: f64,
        dt: f64,
    ) -> Result<f64, String> {
        let error = target.clamp(-control.limit, control.limit) - speed;
        let p = control.kp * error;
        let candidate = self.integral + control.ki * error * dt;
        let requested = p + candidate;
        if !requested.is_finite() {
            return Err("выход регулятора скорости вне численного диапазона".into());
        }
        // Условное интегрирование: на упоре оставляем только шаги к рабочему
        // диапазону; за один тик можно дойти до границы допустимого тока.
        let capped = requested.clamp(-limit, limit) - p;
        self.integral = capped.clamp(self.integral.min(candidate), self.integral.max(candidate));
        Ok((p + self.integral).clamp(-limit, limit))
    }
}
