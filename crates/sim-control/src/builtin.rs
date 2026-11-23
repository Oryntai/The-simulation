//! Граф встроенного контроллера. Источники, калибровка и режимы задаются конфигурацией.

use std::collections::BTreeSet;

use sim_signal::{ChannelId, Command, ImageFrame, Observation, PixelFormat, Rng, SignalId, Time};

use crate::{CheckedObservation, ControlContract, ControlError, Controller, InputKind};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NodeId(pub usize);

#[derive(Clone, Copy, Debug)]
pub struct PidConfig {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub integral_limits: [f64; 2],
    pub output_limits: [f64; 2],
}

#[derive(Clone, Copy, Debug)]
pub struct TiltConfig {
    /// Полоса модуля ускорения в измеряемой плоскости, в единицах входов.
    pub acceleration_range: [f64; 2],
    pub time_constant_s: f64,
}

#[derive(Clone, Copy, Debug)]
pub enum ImageAxis {
    X,
    Y,
}

#[derive(Clone, Copy, Debug)]
pub enum ImageProjection {
    Pixel,
    /// Угол в радианах по модели камеры-обскуры. Центр и фокусное расстояние в пикселях.
    Bearing {
        center_px: f64,
        focal_px: f64,
    },
}

#[derive(Clone, Copy, Debug)]
pub struct RgbTarget {
    pub min: [u8; 3],
    pub max: [u8; 3],
    pub min_pixels: u32,
    pub axis: ImageAxis,
    pub projection: ImageProjection,
}

#[derive(Clone, Debug)]
pub enum Block {
    Scalar(SignalId),
    Constant(f64),
    /// Калибровка, взвешенная сумма или разность в зависимости от коэффициентов.
    Affine {
        terms: Vec<(NodeId, f64)>,
        offset: f64,
    },
    /// Предикат возвращает 1 внутри полосы и 0 снаружи. Нет измерения — нет предиката.
    InRange {
        input: NodeId,
        bounds: [f64; 2],
    },
    /// Пропуск при истинном предикате. При ложном или неизвестном результат отсутствует.
    Gate {
        input: NodeId,
        condition: NodeId,
    },
    Pid {
        error: NodeId,
        enable: Option<NodeId>,
        config: PidConfig,
    },
    /// Калиброванные компоненты ускорения дают atan2(sin, cos). Гироскоп в рад/с.
    PlanarTilt {
        sin: NodeId,
        cos: NodeId,
        rate: NodeId,
        config: TiltConfig,
    },
    Rgb {
        image: SignalId,
        target: RgbTarget,
    },
}

#[derive(Clone, Copy, Debug)]
pub enum MissingOutput {
    /// Принимающая сторона инвалидирует предыдущее значение канала.
    Omit,
    /// Явная реакция по конфигурации. Допустимое безопасное значение определяется задачей.
    Value(f64),
    Error,
}

#[derive(Clone, Debug)]
pub struct OutputBinding {
    pub channel: ChannelId,
    pub node: NodeId,
    pub missing: MissingOutput,
}

#[derive(Clone, Debug)]
pub struct BuiltinConfig {
    /// Порядок вычисления. Ссылки допустимы только на предшествующие блоки.
    pub blocks: Vec<Block>,
    pub outputs: Vec<OutputBinding>,
}

#[derive(Clone, Copy, Debug)]
enum State {
    Empty,
    Pid { error: f64, integral: f64 },
    Tilt(f64),
}

#[derive(Clone, Debug)]
pub struct BuiltinController {
    contract: ControlContract,
    config: BuiltinConfig,
    state: Vec<State>,
    last: Option<Time>,
}

impl BuiltinController {
    pub fn new(contract: ControlContract, config: BuiltinConfig) -> Result<Self, ControlError> {
        for (index, block) in config.blocks.iter().enumerate() {
            if !block_valid(block, index, &contract) {
                return Err(ControlError::Execution(format!(
                    "неверная конфигурация блока {index}"
                )));
            }
        }
        let mut channels = BTreeSet::new();
        for binding in &config.outputs {
            if !contract.outputs.contains_key(&binding.channel)
                || !channels.insert(binding.channel)
                || binding.node.0 >= config.blocks.len()
                || matches!(binding.missing, MissingOutput::Value(value) if !value.is_finite())
            {
                return Err(ControlError::InvalidCommand(format!(
                    "неверная привязка выхода {}",
                    binding.channel.0
                )));
            }
        }
        if channels.len() != contract.outputs.len() {
            return Err(ControlError::InvalidCommand(
                "не для всех выходов задана привязка".into(),
            ));
        }
        Ok(Self {
            state: vec![State::Empty; config.blocks.len()],
            contract,
            config,
            last: None,
        })
    }

    pub fn contract(&self) -> &ControlContract {
        &self.contract
    }
}

impl Controller for BuiltinController {
    fn step(&mut self, observation: &Observation, _rng: &mut Rng) -> Result<Command, ControlError> {
        let observation = self.contract.observation(observation)?;
        if self.last.is_some_and(|last| observation.time <= last) {
            return Err(ControlError::InvalidObservation(
                "время контроллера не возросло".into(),
            ));
        }
        const NS_PER_SECOND: f64 = 1_000_000_000.0;
        let dt = self
            .last
            .map(|last| (observation.time.0 - last.0) as f64 / NS_PER_SECOND);
        let mut values = Vec::with_capacity(self.config.blocks.len());
        let mut state = self.state.clone();
        for (index, block) in self.config.blocks.iter().enumerate() {
            values.push(evaluate(
                block,
                &observation,
                &values,
                &mut state[index],
                dt,
            )?);
        }
        let mut command = Command::default();
        for binding in &self.config.outputs {
            let value = match (values[binding.node.0], binding.missing) {
                (Some(value), _) | (None, MissingOutput::Value(value)) => value,
                (None, MissingOutput::Omit) => continue,
                (None, MissingOutput::Error) => {
                    return Err(ControlError::Execution(format!(
                        "нет данных для выхода {}",
                        binding.channel.0
                    )));
                }
            };
            command.targets.push((binding.channel, value));
        }
        self.contract
            .validate_command(&command, observation.time, observation.time)?;
        self.state = state;
        self.last = Some(observation.time);
        Ok(command)
    }
}

fn band(bounds: [f64; 2]) -> bool {
    bounds.iter().all(|v| v.is_finite()) && bounds[0] <= bounds[1]
}

fn block_valid(block: &Block, index: usize, contract: &ControlContract) -> bool {
    let prior = |node: NodeId| node.0 < index;
    match block {
        Block::Scalar(id) => matches!(
            contract.inputs.get(id).map(|c| c.kind),
            Some(InputKind::Scalar { .. })
        ),
        Block::Constant(value) => value.is_finite(),
        Block::Affine { terms, offset } => {
            !terms.is_empty()
                && offset.is_finite()
                && terms
                    .iter()
                    .all(|&(id, gain)| prior(id) && gain.is_finite())
        }
        Block::InRange { input, bounds } => prior(*input) && band(*bounds),
        Block::Gate { input, condition } => prior(*input) && prior(*condition),
        Block::Pid {
            error,
            enable,
            config,
        } => {
            prior(*error)
                && enable.is_none_or(prior)
                && [config.kp, config.ki, config.kd]
                    .iter()
                    .all(|v| v.is_finite())
                && band(config.integral_limits)
                && band(config.output_limits)
                && config.integral_limits[0] <= 0.0
                && config.integral_limits[1] >= 0.0
        }
        Block::PlanarTilt {
            sin,
            cos,
            rate,
            config,
        } => {
            [*sin, *cos, *rate].into_iter().all(prior)
                && band(config.acceleration_range)
                && config.acceleration_range[0] > 0.0
                && config.time_constant_s.is_finite()
                && config.time_constant_s > 0.0
        }
        Block::Rgb { image, target } => {
            matches!(
                contract.inputs.get(image).map(|c| c.kind),
                Some(InputKind::Image {
                    format: PixelFormat::Rgb8,
                    ..
                })
            ) && target.min_pixels > 0
                && target.min.iter().zip(target.max).all(|(&lo, hi)| lo <= hi)
                && match target.projection {
                    ImageProjection::Pixel => true,
                    ImageProjection::Bearing {
                        center_px,
                        focal_px,
                    } => center_px.is_finite() && focal_px.is_finite() && focal_px > 0.0,
                }
        }
    }
}

fn evaluate(
    block: &Block,
    observation: &CheckedObservation<'_>,
    values: &[Option<f64>],
    state: &mut State,
    dt: Option<f64>,
) -> Result<Option<f64>, ControlError> {
    let get = |node: NodeId| values[node.0];
    let value = match *block {
        Block::Scalar(id) => observation.sample(id).map(|s| s.value),
        Block::Constant(value) => Some(value),
        Block::Affine { ref terms, offset } => {
            let mut sum = offset;
            for &(id, gain) in terms {
                let Some(value) = get(id) else {
                    return Ok(None);
                };
                sum = finite(sum + gain * value)?;
            }
            Some(sum)
        }
        Block::InRange { input, bounds } => {
            get(input).map(|x| f64::from(x >= bounds[0] && x <= bounds[1]))
        }
        Block::Gate { input, condition } => get(condition)
            .filter(|&x| x == 1.0)
            .and_then(|_| get(input)),
        Block::Pid {
            error,
            enable,
            config,
        } => {
            if let Some(error) = get(error).filter(|_| enable.is_none_or(|id| get(id) == Some(1.0)))
            {
                Some(pid(config, state, error, dt)?)
            } else {
                *state = State::Empty;
                None
            }
        }
        Block::PlanarTilt {
            sin,
            cos,
            rate,
            config,
        } => tilt(config, state, dt, get(sin), get(cos), get(rate))?,
        Block::Rgb { image, target } => {
            if let Some(frame) = observation.frame(image) {
                rgb(frame, target)?
            } else {
                None
            }
        }
    };
    value.map(finite).transpose()
}

fn finite(value: f64) -> Result<f64, ControlError> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(ControlError::Execution(
            "переполнение в вычислении блока".into(),
        ))
    }
}

fn pid(
    config: PidConfig,
    state: &mut State,
    error: f64,
    dt: Option<f64>,
) -> Result<f64, ControlError> {
    let [lo, hi] = config.output_limits;
    let [i_lo, i_hi] = config.integral_limits;
    let (mut integral, mut slope) = (0.0, 0.0);
    let proportional = finite(config.kp * error)?;
    if let (
        State::Pid {
            error: previous,
            integral: old,
        },
        Some(dt),
    ) = (*state, dt)
    {
        // Прямоугольник по новому отсчёту. При насыщении приращение разрешено к рабочему диапазону.
        if config.kd != 0.0 {
            slope = finite(config.kd * ((error - previous) / dt))?;
        }
        let candidate = finite(old + finite(config.ki * error)? * dt)?.clamp(i_lo, i_hi);
        let trial = finite(proportional + candidate + slope)?;
        integral = if (trial > hi && candidate > old) || (trial < lo && candidate < old) {
            old
        } else {
            candidate
        };
    }
    *state = State::Pid { error, integral };
    Ok(finite(proportional + integral + slope)?.clamp(lo, hi))
}

fn tilt(
    config: TiltConfig,
    state: &mut State,
    dt: Option<f64>,
    sin: Option<f64>,
    cos: Option<f64>,
    rate: Option<f64>,
) -> Result<Option<f64>, ControlError> {
    if let (Some(sin), Some(cos), Some(rate)) = (sin, cos, rate) {
        let magnitude = finite(sin.hypot(cos))?;
        let [lo, hi] = config.acceleration_range;
        if (lo..=hi).contains(&magnitude) {
            let measured = sin.atan2(cos);
            let angle = if let (State::Tilt(previous), Some(dt)) = (*state, dt) {
                // Точное затухание фильтра первого порядка; поправка по кратчайшей дуге.
                let correction = -(-dt / config.time_constant_s).exp_m1();
                let predicted = wrap(finite(previous + rate * dt)?);
                wrap(predicted + correction * wrap(measured - predicted))
            } else {
                measured
            };
            *state = State::Tilt(angle);
            return Ok(Some(angle));
        }
    }
    *state = State::Empty;
    Ok(None)
}

fn wrap(angle: f64) -> f64 {
    use std::f64::consts::{PI, TAU};
    (angle + PI).rem_euclid(TAU) - PI
}

fn rgb(frame: &ImageFrame, target: RgbTarget) -> Result<Option<f64>, ControlError> {
    let width = frame.width as usize;
    let mut count = 0_u64;
    let mut sum = 0_u128;
    for (index, pixel) in frame
        .data
        .chunks_exact(PixelFormat::Rgb8.bytes_per_pixel())
        .enumerate()
    {
        if pixel
            .iter()
            .zip(target.min)
            .zip(target.max)
            .all(|((&v, lo), hi)| (lo..=hi).contains(&v))
        {
            count += 1;
            sum += match target.axis {
                ImageAxis::X => index % width,
                ImageAxis::Y => index / width,
            } as u128;
        }
    }
    if count < u64::from(target.min_pixels) {
        return Ok(None);
    }
    let center = sum as f64 / count as f64;
    Ok(Some(match target.projection {
        ImageProjection::Pixel => center,
        ImageProjection::Bearing {
            center_px,
            focal_px,
        } => finite(center - center_px)?.atan2(focal_px),
    }))
}
