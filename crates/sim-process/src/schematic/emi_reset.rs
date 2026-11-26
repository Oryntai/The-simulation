//! Дифференциальное напряжение на низкоактивном входе сброса и живое свидетельство платы.

use sim_circuit::emc::{
    Analysis as CircuitAnalysis, Sourced,
    loop_model::{LumpedDomain, SeriesRl, fitted_path},
};
use sim_signal::Time;
use sim_world::Provenance;

use crate::{Analysis, ProcessError, SourcedRange};

use super::{
    Endpoint, Parameter, ResetProfile, Scope, identifier, invalid, ns_bound, supports_source,
    validate_source,
};

pub const EMI_RESET_CODE: &str = "emi_reset";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EmiSourceKind {
    Motor,
    Switch,
}

/// Огибающая на всём интервале, включая неразрешённые фронты.
/// Номинал поступает в дискретную модель приёмника и удерживается до границы.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EmiBand {
    pub lower: f64,
    pub nominal: f64,
    pub upper: f64,
}

impl EmiBand {
    pub fn point(value: f64) -> Self {
        Self {
            lower: value,
            nominal: value,
            upper: value,
        }
    }

    fn validate(self) -> Result<Self, ProcessError> {
        if [self.lower, self.nominal, self.upper]
            .into_iter()
            .all(f64::is_finite)
            && self.lower <= self.nominal
            && self.nominal <= self.upper
        {
            Ok(self)
        } else {
            Err(invalid("неконечная или перевёрнутая огибающая помехи"))
        }
    }

    fn plus(self, rhs: Self) -> Result<Self, ProcessError> {
        Self {
            lower: (self.lower + rhs.lower).next_down(),
            nominal: self.nominal + rhs.nominal,
            upper: (self.upper + rhs.upper).next_up(),
        }
        .validate()
    }

    fn minus(self, rhs: Self) -> Result<Self, ProcessError> {
        self.plus(Self {
            lower: -rhs.upper,
            nominal: -rhs.nominal,
            upper: -rhs.lower,
        })
    }
}

impl From<&SourcedRange> for EmiBand {
    fn from(value: &SourcedRange) -> Self {
        Self {
            lower: value.min,
            nominal: value.nominal,
            upper: value.max,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EmiInterval<T> {
    /// Правая граница исключена. Левая — предыдущая граница или начало опыта.
    pub end: Time,
    pub value: T,
}

#[derive(Clone, Debug, PartialEq)]
pub struct EmiReturn {
    /// Привязка общего участка к цепям источника и приёмника, с подтверждённой
    /// линейностью, высоким импедансом входа и отсутствием иных значимых путей.
    pub geometry: String,
    pub domain: LumpedDomain,
    pub rl: SeriesRl,
    /// Верхняя частота обоснования этой формы, включая спектр фронтов.
    pub highest_frequency_hz: f64,
    pub model_source: Option<Provenance>,
    /// Верхний модуль остатка, включая хвост спектра, погрешность R/L,
    /// отклонение топологии и численную погрешность.
    pub remainder_v: Parameter,
    /// Огибающие одного и того же тока (А) и его производной (А/с).
    /// Ток направлен так, что положительный R·I + L·dI/dt поднимает опору входа.
    pub intervals: Vec<EmiInterval<(EmiBand, EmiBand)>>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum EmiTransfer {
    /// Синхронный замер изменения V(вход) − V(опора) при включении источника.
    /// Источник ряда подтверждает полосу, погрешность и огибающую между точками.
    Measured {
        intervals: Vec<EmiInterval<EmiBand>>,
        source: Option<Provenance>,
    },
    SharedReturn(EmiReturn),
}

#[derive(Clone, Debug, PartialEq)]
pub struct EmiInput {
    pub scope: Scope,
    pub source: String,
    pub source_kind: EmiSourceKind,
    /// Имя экземпляра в контуре. Клеммы адресуются в схеме установки.
    pub board: String,
    pub input: Endpoint,
    pub reference: Endpoint,
    /// Подтверждение разводки клемм, полярности и назначения входа платы.
    pub binding: Option<Provenance>,
    pub start: Time,
    pub end: Time,
    /// Напряжение на том же входе в покое, также до и после окна помехи.
    pub bias_v: Parameter,
    pub detector: Option<ResetProfile>,
    pub transfer: Option<EmiTransfer>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EmiDetector {
    pub falling_v: f64,
    pub rising_v: f64,
    pub assert_ns: u64,
    pub release_ns: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct EmiTrial {
    input: EmiInput,
    bias: EmiBand,
    intervals: Vec<EmiInterval<EmiBand>>,
    detector: EmiDetector,
    falling: EmiBand,
    hold: [u64; 3],
}

impl EmiTrial {
    pub fn input(&self) -> &EmiInput {
        &self.input
    }

    pub fn detector(&self) -> EmiDetector {
        self.detector
    }

    pub fn voltage_at(&self, at: Time, coupled: bool) -> f64 {
        if coupled && at >= self.input.start {
            let index = self
                .intervals
                .partition_point(|interval| interval.end <= at);
            if let Some(interval) = self.intervals.get(index) {
                return interval.value.nominal;
            }
        }
        self.bias.nominal
    }

    pub fn next_boundary(&self, after: Time) -> Option<Time> {
        if after < self.input.start {
            return Some(self.input.start);
        }
        self.intervals
            .get(
                self.intervals
                    .partition_point(|interval| interval.end <= after),
            )
            .map(|interval| interval.end)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EmiCriterion {
    Violated,
    WithinBench,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EmiOutcome {
    Fail { at: Time },
    NotAnalyzed { reason: String },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EmiMargins {
    /// Нижняя, номинальная и верхняя границы запаса минимума напряжения.
    pub voltage_v: EmiBand,
    /// Минимальная выдержка минус возможное время ниже порога, затем
    /// номинальная пара и максимальная выдержка минус гарантированное время.
    pub dwell_ns: [i128; 3],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EmiResetObservation {
    /// Последний общий момент пары, сравниваемой с установкой и журналом.
    pub observed_until: Time,
    pub complete: bool,
    pub reset_at: Option<Time>,
    /// Сброс подтверждён платой и совпал с фронтом внешнего детектора.
    pub external_asserted_at: Option<Time>,
    pub baseline_reset_at: Option<Time>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EmiPacketProbe {
    pub route: String,
    pub sent_at: Time,
    pub deadline: Time,
    /// Требование к сроку доставки запроса в этой установке.
    pub source: Option<Provenance>,
}

impl EmiPacketProbe {
    pub fn validate(&self, scope: Scope) -> Result<bool, ProcessError> {
        if !identifier(&self.route) || self.deadline <= self.sent_at {
            return Err(invalid("некорректный маршрут или срок наблюдения пакета"));
        }
        confirmed(scope, self.source.as_ref())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EmiPacketWitness {
    pub route: String,
    pub id: u64,
    pub sent_at: Time,
    pub deadline: Time,
    pub baseline_received_at: Time,
    /// Отсутствие означает, что адресат не получил тот же пакет до срока.
    pub disturbed_received_at: Option<Time>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct EmiEvidence {
    pub code: &'static str,
    pub scope: Scope,
    pub source: String,
    pub source_kind: EmiSourceKind,
    pub board: String,
    pub input: Endpoint,
    pub reference: Endpoint,
    pub observation: Option<EmiResetObservation>,
    pub margins: EmiMargins,
    pub criterion: Option<EmiCriterion>,
    pub outcome: EmiOutcome,
    pub packet: Option<EmiPacketWitness>,
    pub packet_open: Option<String>,
}

/// Подготовить огибающую и отдельный детектор входа сброса.
/// Все заданные числа проверяются, в том числе при пропуске другого входа.
pub fn prepare_emi_reset(input: EmiInput) -> Result<Analysis<EmiTrial>, ProcessError> {
    if ![
        input.source.as_str(),
        &input.board,
        &input.input.component,
        &input.input.pin,
        &input.reference.component,
        &input.reference.pin,
    ]
    .into_iter()
    .all(identifier)
        || input.start >= input.end
        || input.input == input.reference
    {
        return Err(invalid(
            "некорректный адрес, источник или окно опыта сброса",
        ));
    }
    let scope = input.scope;
    let bias = parameter(scope, &input.bias_v, |_| true)?;
    let bound = confirmed(scope, input.binding.as_ref())?;
    let intervals = transfer(&input)?;
    let detector = input
        .detector
        .as_ref()
        .map(|profile| resolve_detector(scope, profile))
        .transpose()?
        .flatten();
    let Some(bias) = bias else {
        return Ok(open(
            "нужен замер дифференциального напряжения входа в покое",
        ));
    };
    let Some(intervals) = intervals else {
        return Ok(open(
            "нужна полная подтверждённая огибающая связи в рабочей полосе на заданных портах",
        ));
    };
    let Some((detector, falling, rising, hold)) = detector else {
        return Ok(open(
            "нужны подтверждённые пороги и выдержки этого входа с разрешением в наносекунду",
        ));
    };
    if !bound || bias.lower <= rising.upper || falling.upper >= rising.lower {
        return Ok(open(
            "нужна подтверждённая разводка и разделённые пороги с гарантированным неактивным уровнем покоя",
        ));
    }
    let intervals = intervals
        .into_iter()
        .map(|interval| {
            let value = bias.plus(interval.value)?;
            value.minus(falling)?; // Проверяем представимость всех вычисляемых запасов.
            Ok(EmiInterval {
                end: interval.end,
                value,
            })
        })
        .collect::<Result<Vec<_>, ProcessError>>()?;
    Ok(Analysis::Analyzed {
        value: EmiTrial {
            input,
            bias,
            intervals,
            detector,
            falling,
            hold,
        },
        scope: "заданный стенд, дифференциальный низкоактивный вход сброса".into(),
    })
}

fn resolve_detector(
    scope: Scope,
    profile: &ResetProfile,
) -> Result<Option<(EmiDetector, EmiBand, EmiBand, [u64; 3])>, ProcessError> {
    profile.validate()?;
    for span in [&profile.assert_delay_s, &profile.release_delay_s] {
        if let Some(range) = &span.range {
            ns_bound(range.max, true)?;
        }
    }
    let (Some(falling), Some(rising), Some(assert), Some(release)) = (
        parameter(scope, &profile.falling_v, |v| v >= 0.0)?,
        parameter(scope, &profile.rising_v, |v| v >= 0.0)?,
        parameter(scope, &profile.assert_delay_s, |v| v >= 0.0)?,
        parameter(scope, &profile.release_delay_s, |v| v >= 0.0)?,
    ) else {
        return Ok(None);
    };
    let assert_ns = ns_bound(assert.nominal, true)?;
    let release_ns = ns_bound(release.nominal, true)?;
    if assert_ns != ns_bound(assert.nominal, false)?
        || release_ns != ns_bound(release.nominal, false)?
    {
        return Ok(None);
    }
    let detector = EmiDetector {
        falling_v: falling.nominal,
        rising_v: rising.nominal,
        assert_ns,
        release_ns,
    };
    let hold = [
        ns_bound(assert.lower, false)?,
        assert_ns,
        ns_bound(assert.upper, true)?,
    ];
    Ok(Some((detector, falling, rising, hold)))
}

fn transfer(input: &EmiInput) -> Result<Option<Vec<EmiInterval<EmiBand>>>, ProcessError> {
    let (intervals, sourced) = match &input.transfer {
        None => return Ok(None),
        Some(EmiTransfer::Measured { intervals, source }) => {
            for row in intervals {
                row.value.validate()?;
            }
            (intervals.clone(), confirmed(input.scope, source.as_ref())?)
        }
        Some(EmiTransfer::SharedReturn(path)) => {
            if !identifier(&path.geometry)
                || !path.highest_frequency_hz.is_finite()
                || path.highest_frequency_hz < 0.0
            {
                return Err(invalid("некорректное описание или полоса общего возврата"));
            }
            let residual = parameter(input.scope, &path.remainder_v, |v| v >= 0.0)?;
            let sourced = confirmed(input.scope, path.model_source.as_ref())?;
            // fitted_path проверяет геометрию, R/L и электрическую длину.
            let source = if sourced { path.geometry.as_str() } else { "" };
            let fit = fitted_path(
                Some(Sourced {
                    value: path.rl,
                    source,
                }),
                Some(Sourced {
                    value: path.domain,
                    source,
                }),
            )
            .map_err(|e| invalid(e.to_string()))?;
            let in_band = matches!(fit, CircuitAnalysis::Calculated(model)
                if path.highest_frequency_hz <= model.max_frequency_hz());
            let rows = path
                .intervals
                .iter()
                .map(|row| {
                    let (current, slew) = row.value;
                    current.validate()?;
                    slew.validate()?;
                    let scale = |coefficient: f64, band: EmiBand| EmiBand {
                        lower: (coefficient * band.lower).next_down(),
                        nominal: coefficient * band.nominal,
                        upper: (coefficient * band.upper).next_up(),
                    };
                    let drop = scale(path.rl.resistance_ohm, current)
                        .plus(scale(path.rl.inductance_h, slew))?;
                    Ok(EmiInterval {
                        end: row.end,
                        value: EmiBand::point(0.0).minus(drop)?,
                    })
                })
                .collect::<Result<Vec<_>, ProcessError>>()?;
            let usable = sourced && in_band && residual.is_some();
            let mut rows = rows;
            if let Some(residual) = residual {
                let error = EmiBand {
                    lower: -residual.upper,
                    nominal: 0.0,
                    upper: residual.upper,
                };
                for row in &mut rows {
                    row.value = row.value.plus(error)?;
                }
            }
            (rows, usable)
        }
    };
    let mut end = input.start;
    for row in &intervals {
        if row.end <= end || row.end > input.end {
            return Err(invalid(
                "границы огибающей должны строго возрастать в окне опыта",
            ));
        }
        end = row.end;
    }
    Ok((sourced && end == input.end).then_some(intervals))
}

/// Оценить только целевой критерий и оставить запрос стендового замера.
pub fn assess_emi_reset(trial: &EmiTrial, observed: Option<EmiResetObservation>) -> EmiEvidence {
    // Округляем наружу, чтобы сравнение не зависело от потери младшего бита.
    let mut voltage = EmiBand {
        lower: (trial.bias.lower - trial.falling.upper).next_down(),
        nominal: trial.bias.nominal - trial.falling.nominal,
        upper: (trial.bias.upper - trial.falling.lower).next_up(),
    };
    let mut longest = [0; 3];
    let mut since = [None; 3];
    let mut guaranteed_at_event = false;
    let mut start = trial.input.start;
    let at = observed.and_then(|item| item.reset_at);
    for interval in &trial.intervals {
        let v = interval.value;
        voltage.lower = voltage
            .lower
            .min((v.lower - trial.falling.upper).next_down());
        voltage.nominal = voltage.nominal.min(v.nominal - trial.falling.nominal);
        voltage.upper = voltage.upper.min((v.upper - trial.falling.lower).next_up());
        let below = [
            v.lower < trial.falling.upper,
            v.nominal < trial.falling.nominal,
            v.upper < trial.falling.lower,
        ];
        for (i, low) in below.into_iter().enumerate() {
            if low {
                let from = *since[i].get_or_insert(start);
                longest[i] = longest[i].max(interval.end.0 - from.0);
            } else {
                since[i] = None;
            }
        }
        if let (Some(at), Some(from)) = (at, since[2])
            && start <= at
            && at < interval.end
            && at.0 - from.0 >= trial.hold[2]
        {
            guaranteed_at_event = true;
        }
        start = interval.end;
    }
    let dwell_ns = std::array::from_fn(|i| i128::from(trial.hold[i]) - i128::from(longest[i]));
    let mut evidence = EmiEvidence {
        code: EMI_RESET_CODE, scope: trial.input.scope, source: trial.input.source.clone(),
        source_kind: trial.input.source_kind, board: trial.input.board.clone(), input: trial.input.input.clone(),
        reference: trial.input.reference.clone(), observation: observed,
        margins: EmiMargins { voltage_v: voltage, dwell_ns },
        criterion: None, packet: None, packet_open: None,
        outcome: EmiOutcome::NotAnalyzed {
            reason: "нужен синхронный стендовый замер напряжения входа, сброса и работы платы при коммутации".into(),
        },
    };
    let Some(observed) = observed else {
        return evidence;
    };
    if let Some(at) = at
        && guaranteed_at_event
        && observed.external_asserted_at == Some(at)
        && observed.observed_until >= at
        && observed
            .baseline_reset_at
            .is_none_or(|quiet_at| quiet_at > at)
    {
        evidence.outcome = EmiOutcome::Fail { at };
        evidence.criterion = Some(EmiCriterion::Violated);
    } else if observed.complete
        && observed.observed_until >= trial.input.end
        && at.is_none()
        && observed.external_asserted_at.is_none()
        && observed.baseline_reset_at.is_none()
        && (longest[0] == 0 || dwell_ns[0] > 0)
    {
        evidence.criterion = Some(EmiCriterion::WithinBench);
    } else {
        let reason = if observed.baseline_reset_at.is_some() {
            "в контрольном опыте есть собственный сброс. Нужен синхронный замер питания, линии и сброса"
        } else if !observed.complete {
            "журнал пары неполон. Нужен синхронный ряд напряжения и сброса на всём испытательном окне"
        } else {
            "нет подтверждённой причинной связи между огибающей помехи и сбросом. Нужен синхронный замер фронтов и фактической выдержки"
        };
        evidence.outcome = EmiOutcome::NotAnalyzed {
            reason: reason.into(),
        };
    }
    evidence
}

fn parameter(
    scope: Scope,
    parameter: &Parameter,
    valid: impl Fn(f64) -> bool,
) -> Result<Option<EmiBand>, ProcessError> {
    parameter.validate(valid)?;
    let Some(range) = &parameter.range else {
        return Ok(None);
    };
    Ok(confirmed(scope, Some(&range.provenance))?.then(|| range.into()))
}

fn confirmed(scope: Scope, source: Option<&Provenance>) -> Result<bool, ProcessError> {
    let Some(source) = source else {
        return Ok(false);
    };
    validate_source(source)?;
    Ok(supports_source(scope, source))
}

fn open<T>(reason: &str) -> Analysis<T> {
    Analysis::NotAnalyzed {
        reason: reason.into(),
    }
}
