//! Согласование каналов и проверки на границе контроллера.

use std::collections::{BTreeMap, BTreeSet};

use sim_signal::{
    ChannelId, Command, ImageFrame, Observation, PixelFormat, Sample, SignalId, Time,
};

use crate::{ControlError, ControlRequest, ControlResponse};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InputKind {
    Scalar {
        max_age_ns: u64,
    },
    Image {
        format: PixelFormat,
        max_capture_age_ns: u64,
        max_delivery_age_ns: u64,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InputChannel {
    pub id: SignalId,
    pub name: String,
    pub unit: String,
    pub kind: InputKind,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OutputChannel {
    pub id: ChannelId,
    pub name: String,
    pub unit: String,
    /// Срок от времени наблюдения, на основании которого сделана команда, включительно.
    pub max_age_ns: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ControlContract {
    pub(crate) inputs: BTreeMap<SignalId, InputChannel>,
    pub(crate) outputs: BTreeMap<ChannelId, OutputChannel>,
}

impl ControlContract {
    pub fn new(
        inputs: Vec<InputChannel>,
        outputs: Vec<OutputChannel>,
    ) -> Result<Self, ControlError> {
        let mut contract = Self {
            inputs: BTreeMap::new(),
            outputs: BTreeMap::new(),
        };
        let mut names = BTreeSet::new();
        for input in inputs {
            if input.name.trim().is_empty()
                || input.unit.trim().is_empty()
                || !names.insert(input.name.clone())
                || contract.inputs.contains_key(&input.id)
            {
                return Err(ControlError::InvalidObservation(format!(
                    "пустой или повторный вход: {}",
                    input.name
                )));
            }
            contract.inputs.insert(input.id, input);
        }
        names.clear();
        for output in outputs {
            if output.name.trim().is_empty()
                || output.unit.trim().is_empty()
                || !names.insert(output.name.clone())
                || contract.outputs.contains_key(&output.id)
            {
                return Err(ControlError::InvalidCommand(format!(
                    "пустой или повторный выход: {}",
                    output.name
                )));
            }
            contract.outputs.insert(output.id, output);
        }
        Ok(contract)
    }

    pub fn inputs(&self) -> impl Iterator<Item = &InputChannel> {
        self.inputs.values()
    }

    pub fn outputs(&self) -> impl Iterator<Item = &OutputChannel> {
        self.outputs.values()
    }

    /// Проверяет весь вход. Возвращает представление с доступом только к свежим данным.
    pub fn observation<'a>(
        &'a self,
        observation: &'a Observation,
    ) -> Result<CheckedObservation<'a>, ControlError> {
        let mut samples = BTreeMap::new();
        let mut frames = BTreeMap::new();
        let mut sample_times = BTreeSet::new();
        let mut frame_times = BTreeSet::new();
        for sample in &observation.samples {
            if !sample.value.is_finite()
                || sample.at > observation.time
                || !matches!(
                    self.inputs.get(&sample.signal).map(|c| c.kind),
                    Some(InputKind::Scalar { .. })
                )
                || !sample_times.insert((sample.signal, sample.at))
            {
                return Err(ControlError::InvalidObservation(format!(
                    "некорректный, неизвестный или неоднозначный скаляр {}",
                    sample.signal.0
                )));
            }
            keep_latest(&mut samples, sample.signal, sample, |s| s.at);
        }
        for frame in &observation.frames {
            if frame.captured > frame.at
                || frame.at > observation.time
                || frame.width == 0
                || frame.height == 0
                || !frame.is_consistent()
                || !matches!(self.inputs.get(&frame.signal).map(|c| c.kind),
                    Some(InputKind::Image { format, .. }) if format == frame.format)
                || !frame_times.insert((frame.signal, frame.at))
            {
                return Err(ControlError::InvalidObservation(format!(
                    "некорректный, неизвестный или неоднозначный кадр {}",
                    frame.signal.0
                )));
            }
            keep_latest(&mut frames, frame.signal, frame, |f| f.at);
        }
        Ok(CheckedObservation {
            contract: self,
            time: observation.time,
            samples,
            frames,
        })
    }

    /// Проверка в момент фактической доставки. При ошибке непригодна вся команда.
    pub fn validate_command(
        &self,
        command: &Command,
        observed_at: Time,
        now: Time,
    ) -> Result<(), ControlError> {
        let age = now.0.checked_sub(observed_at.0).ok_or_else(|| {
            ControlError::InvalidCommand("выход получен раньше исходного наблюдения".into())
        })?;
        let mut seen = BTreeSet::new();
        for &(id, value) in &command.targets {
            let output = self.outputs.get(&id).ok_or_else(|| {
                ControlError::InvalidCommand(format!("неизвестный выход {}", id.0))
            })?;
            if !value.is_finite() || !seen.insert(id) {
                return Err(ControlError::InvalidCommand(format!(
                    "неконечный или повторный выход {}",
                    output.name
                )));
            }
            if age > output.max_age_ns {
                return Err(ControlError::InvalidCommand(format!(
                    "просроченный выход {}",
                    output.name
                )));
            }
        }
        Ok(())
    }

    pub fn validate_response(
        &self,
        request: &ControlRequest,
        response: &ControlResponse,
        now: Time,
    ) -> Result<(), ControlError> {
        if request.id != response.id {
            return Err(ControlError::Transport(
                "номер ответа не совпадает с запросом".into(),
            ));
        }
        self.validate_command(&response.command, request.observation.time, now)
    }
}

fn keep_latest<'a, T>(
    entries: &mut BTreeMap<SignalId, &'a T>,
    id: SignalId,
    incoming: &'a T,
    moment: impl Fn(&T) -> Time,
) {
    if entries
        .get(&id)
        .is_none_or(|current| moment(incoming) > moment(current))
    {
        entries.insert(id, incoming);
    }
}

pub struct CheckedObservation<'a> {
    contract: &'a ControlContract,
    pub time: Time,
    samples: BTreeMap<SignalId, &'a Sample>,
    frames: BTreeMap<SignalId, &'a ImageFrame>,
}

impl<'a> CheckedObservation<'a> {
    pub fn sample(&self, id: SignalId) -> Option<&'a Sample> {
        let InputKind::Scalar { max_age_ns } = self.contract.inputs.get(&id)?.kind else {
            return None;
        };
        let sample = *self.samples.get(&id)?;
        (sample.valid && self.time.0 - sample.at.0 <= max_age_ns).then_some(sample)
    }

    pub fn frame(&self, id: SignalId) -> Option<&'a ImageFrame> {
        let InputKind::Image {
            max_capture_age_ns,
            max_delivery_age_ns,
            ..
        } = self.contract.inputs.get(&id)?.kind
        else {
            return None;
        };
        let frame = *self.frames.get(&id)?;
        (frame.valid
            && self.time.0 - frame.captured.0 <= max_capture_age_ns
            && self.time.0 - frame.at.0 <= max_delivery_age_ns)
            .then_some(frame)
    }
}

impl CheckedObservation<'_> {
    /// Только пригодные отсчёты, по одному на канал, в порядке номеров.
    pub fn usable(&self) -> Observation {
        Observation {
            time: self.time,
            samples: self
                .samples
                .keys()
                .filter_map(|&id| self.sample(id).copied())
                .collect(),
            frames: self
                .frames
                .keys()
                .filter_map(|&id| self.frame(id).cloned())
                .collect(),
        }
    }
}
