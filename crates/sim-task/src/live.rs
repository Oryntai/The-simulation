use std::cell::RefCell;
use std::rc::Rc;

use sim_runtime::{Evaluator, Truth, TruthOut};
use sim_signal::{Rng, Time};

use crate::{Error, Fault, Fingerprint, Signal, invalid, task::finite};

#[derive(Clone, Debug)]
pub struct StimulusReceipt {
    pub id: String,
    pub signal: Signal,
    pub requested: Time,
    pub applied: Time,
    pub value: f64,
}

#[derive(Clone, Debug)]
pub struct FaultReceipt {
    pub fault: Fault,
    pub requested: Time,
    /// Момент подтверждения отказа моделью, которая его применила.
    pub applied: Time,
}

#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub at: Time,
    pub value: f64,
}

#[derive(Clone, Debug)]
pub struct Cycle {
    pub path: String,
    /// Номер обмена выделяет сборщик. Все звенья одной реакции сохраняют этот номер.
    pub sequence: u64,
    /// Именно использованное доставленное измерение.
    pub observation: Sample,
    pub captured_at: Time,
    pub read_at: Time,
    pub controller_name: String,
    pub program: Fingerprint,
    /// Индекс компонента в CommandSource::Controller.
    pub controller: u32,
    pub command: Sample,
    pub accepted: Sample,
    pub feedback: Sample,
}

#[derive(Clone, Debug)]
pub struct Transfer {
    pub path: String,
    pub hop: String,
    pub sequence: u64,
    pub sent: Time,
    /// Явный итог доставки, в том числе время, когда зафиксирована потеря.
    pub outcome: Delivery,
}

#[derive(Clone, Copy, Debug)]
pub enum Delivery {
    Received(Time),
    Lost(Time),
}

impl Delivery {
    pub(crate) fn at(self) -> Time {
        match self {
            Self::Received(at) | Self::Lost(at) => at,
        }
    }
}

#[derive(Clone, Debug)]
pub enum Event {
    Stimulus(StimulusReceipt),
    Fault(FaultReceipt),
    Cycle(Cycle),
    Transfer(Transfer),
}

/// Приёмник для доверенных компонентов установленного рантайма.
/// Открыт только внутри execute, загрузки или десериализации событий у него нет.
#[derive(Clone, Default)]
pub struct EventSink(Rc<RefCell<Option<Journal>>>);

#[derive(Default)]
struct Journal {
    events: Vec<Event>,
    error: Option<Error>,
}

impl EventSink {
    pub fn record(&self, event: Event) -> Result<(), Error> {
        let mut state = self.0.borrow_mut();
        let journal = state
            .as_mut()
            .ok_or_else(|| invalid("запись доступна только во время живого прогона"))?;
        let validity = match &event {
            Event::Stimulus(receipt) => finite(receipt.value),
            Event::Cycle(cycle) => [
                cycle.observation,
                cycle.command,
                cycle.accepted,
                cycle.feedback,
            ]
            .into_iter()
            .try_for_each(|sample| finite(sample.value)),
            Event::Fault(_) | Event::Transfer(_) => Ok(()),
        };
        if let Err(error) = validity {
            journal.error = Some(error.clone());
            return Err(error);
        }
        journal.events.push(event);
        Ok(())
    }

    pub(crate) fn open(&self) {
        *self.0.borrow_mut() = Some(Journal::default());
    }

    pub(crate) fn close(&self) -> Result<Vec<Event>, Error> {
        let journal = self.0.borrow_mut().take().expect("живой журнал открыт");
        match journal.error {
            Some(error) => Err(error),
            None => Ok(journal.events),
        }
    }
}

/// Установка после сборки одновременно опечатывает вход в оценщик:
/// рантайм с replay, внешними командами или хотя бы нулевым шагом запрещает регистрацию.
#[derive(Clone)]
pub(crate) struct FreshRun;

impl Evaluator for FreshRun {
    fn evaluate(&mut self, _: &Truth<'_>, _: &mut Rng, _: &mut TruthOut) -> Result<(), String> {
        Ok(())
    }
}
