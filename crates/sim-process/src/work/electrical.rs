//! Механический пороговый контакт, дребезг и измерения резистивной цепи.

use sim_circuit::{Circuit, SimOptions, operating_point_with};
use sim_signal::{Rng, Time};
use sim_world::Provenance;

use crate::{Analysis, Interval, ProcessError, SourcedRange, StepModel};

use super::{HazardCheck, math::*};

#[derive(Clone, Debug, PartialEq)]
pub struct BounceSegment {
    /// Длительность участка записи дребезга. Сетка Time использует наносекунды.
    pub duration_s: SourcedRange,
    pub follows_target: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub enum BounceModel {
    VerifiedAbsent {
        source: Provenance,
    },
    Profile {
        closing: Vec<BounceSegment>,
        opening: Vec<BounceSegment>,
    },
}

#[derive(Clone, Debug)]
pub struct ElectricalSpec {
    pub release_below_m: SourcedRange,
    pub close_above_m: SourcedRange,
    pub initially_closed: bool,
    pub initial_state_source: Provenance,
    pub bounce: Option<BounceModel>,
    pub contact_closed_ohm: SourcedRange,
    pub contact_open_ohm: SourcedRange,
    pub source_resistance_ohm: SourcedRange,
    pub load_ohm: SourcedRange,
    pub calibrated_supply_v: SourcedRange,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContactFault {
    None,
    /// Принудительное размыкание с заданным сопротивлением разомкнутой ветви.
    OpenCircuit,
    StuckClosed,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ElectricalInput {
    /// Реальный ход механизма в начале интервала, удерживаемый на этом шаге.
    pub travel_m: Option<f64>,
    /// Напряжение холостого хода источника на интервале.
    pub supply_v: f64,
    pub fault: ContactFault,
}

/// Направление тока от источника через контакт к нагрузке. Напряжения — падения
/// в том же направлении. Энергия источника положительна при отдаче в цепь.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ElectricalSegment {
    pub interval: Interval,
    pub closed: bool,
    pub current_a: f64,
    pub load_v: f64,
    pub contact_v: f64,
    pub load_j: f64,
    pub contact_j: f64,
    pub source_loss_j: f64,
    pub source_j: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ElectricalStep {
    pub requested_closed: Option<bool>,
    pub supply_v: f64,
    pub fault: ContactFault,
    pub segments: Analysis<Vec<ElectricalSegment>>,
    /// Интеграл по нагрузке с начала модели. Пропущенный интервал разрывает интеграл.
    pub total_load_j: Analysis<f64>,
}

#[derive(Clone, Copy, Debug)]
struct Response {
    current_a: f64,
    load_v: f64,
    contact_v: f64,
    source_loss_v: f64,
}

#[derive(Clone, Debug)]
pub struct ElectricalContact {
    spec: ElectricalSpec,
    response: [Response; 2],
    requested: Option<bool>,
    changed: Option<Time>,
    previous_fault: ContactFault,
    total_load_j: Option<f64>,
    last: Option<Time>,
}

impl ElectricalContact {
    pub fn new(spec: ElectricalSpec) -> Result<Self, ProcessError> {
        range(&spec.release_below_m, "порог размыкания", |x| x >= 0.0)?;
        range(&spec.close_above_m, "порог замыкания", |x| {
            x >= 0.0
        })?;
        if spec.release_below_m.max >= spec.close_above_m.min {
            return Err(invalid("диапазоны порогов гистерезиса пересекаются"));
        }
        source(&spec.initial_state_source)?;
        range(&spec.calibrated_supply_v, "диапазон питания", |_| true)?;
        let closed = range(
            &spec.contact_closed_ohm,
            "сопротивление замкнутого контакта",
            |x| x > 0.0,
        )?;
        let open = range(
            &spec.contact_open_ohm,
            "сопротивление разомкнутого контакта",
            |x| x > 0.0,
        )?;
        if spec.contact_closed_ohm.max >= spec.contact_open_ohm.min {
            return Err(invalid(
                "сопротивления открытого и закрытого контакта пересекаются",
            ));
        }
        let series = range(
            &spec.source_resistance_ohm,
            "сопротивление источника",
            |x| x >= 0.0,
        )?;
        let load = range(&spec.load_ohm, "сопротивление нагрузки", |x| x > 0.0)?;
        match &spec.bounce {
            Some(BounceModel::VerifiedAbsent { source: src }) => source(src)?,
            Some(BounceModel::Profile { closing, opening }) => {
                validate_profile(closing)?;
                validate_profile(opening)?;
            }
            None => {}
        }
        let response = [
            response(false, closed, open, series, load)?,
            response(true, closed, open, series, load)?,
        ];
        for item in response {
            if item.current_a <= 0.0 || item.load_v <= 0.0 || item.contact_v <= 0.0 {
                return Err(execution(
                    "рабочая точка цепи потеряла ток или падение напряжения",
                ));
            }
        }
        let requested = Some(spec.initially_closed);
        Ok(Self {
            spec,
            response,
            requested,
            changed: None,
            previous_fault: ContactFault::None,
            total_load_j: Some(0.0),
            last: None,
        })
    }

    /// Сравнение пика по всем разрешённым импульсам с пределом этой цепи.
    /// Для оценки перегрева или поражения током требуются отдельные модели.
    pub fn assess_overcurrent(
        &self,
        step: &ElectricalStep,
        limit_a: Option<&SourcedRange>,
    ) -> Result<HazardCheck, ProcessError> {
        let missing = |reason: &str| HazardCheck::NotAnalyzed {
            reason: reason.into(),
        };
        finite(&[step.supply_v])?;
        let Some(limit) = limit_a else {
            return Ok(missing("нет применимого предела тока"));
        };
        range(limit, "предел тока ветви", |x| x >= 0.0)?;
        let Analysis::Analyzed {
            value: segments, ..
        } = &step.segments
        else {
            return Ok(missing("электрическая ветвь не покрыта измерениями"));
        };
        let mut last = None;
        for segment in segments {
            continuous(last, segment.interval)?;
            last = Some(segment.interval.end);
            finite(&[
                segment.current_a,
                segment.load_v,
                segment.contact_v,
                segment.load_j,
                segment.contact_j,
                segment.source_loss_j,
                segment.source_j,
            ])?;
        }
        let spec = &self.spec;
        let timing_confirmed = step.fault != ContactFault::None
            || (proven_point(&spec.release_below_m)
                && proven_point(&spec.close_above_m)
                && direct(&spec.initial_state_source)
                && match &spec.bounce {
                    Some(BounceModel::VerifiedAbsent { source }) => direct(source),
                    Some(BounceModel::Profile { closing, opening }) => closing
                        .iter()
                        .chain(opening)
                        .all(|segment| proven_point(&segment.duration_s)),
                    None => false,
                });
        if segments.is_empty()
            || !timing_confirmed
            || ![
                &limit.provenance,
                &spec.contact_closed_ohm.provenance,
                &spec.contact_open_ohm.provenance,
                &spec.source_resistance_ohm.provenance,
                &spec.load_ohm.provenance,
                &spec.calibrated_supply_v.provenance,
            ]
            .into_iter()
            .all(direct)
        {
            return Ok(missing("неполное покрытие или неподтверждённые параметры"));
        }
        let mut min_peak: f64 = 0.0;
        let mut max_peak: f64 = 0.0;
        for segment in segments {
            let contact = if segment.closed {
                &spec.contact_closed_ohm
            } else {
                &spec.contact_open_ohm
            };
            let high = contact.max + spec.source_resistance_ohm.max + spec.load_ohm.max;
            let low = contact.min + spec.source_resistance_ohm.min + spec.load_ohm.min;
            computed(&[high, low])?;
            min_peak = min_peak.max(step.supply_v.abs() / high);
            max_peak = max_peak.max(step.supply_v.abs() / low);
        }
        computed(&[min_peak, max_peak])?;
        Ok(if min_peak > limit.max {
            HazardCheck::Violation {
                reason: format!("пиковый ток ≥ {min_peak} А при пределе {} А", limit.max),
            }
        } else if max_peak < limit.min {
            HazardCheck::WithinLimit {
                scope: format!("пик тока ≤ {max_peak} А для измеренных состояний цепи"),
            }
        } else {
            missing("диапазон тока перекрывается с пределом")
        })
    }

    fn segment(
        &self,
        interval: Interval,
        closed: bool,
        supply_v: f64,
    ) -> Result<ElectricalSegment, ProcessError> {
        let dt = duration(interval)?;
        let r = self.response[usize::from(closed)];
        let current_a = r.current_a * supply_v;
        let load_v = r.load_v * supply_v;
        let contact_v = r.contact_v * supply_v;
        let load_j = load_v * current_a * dt;
        let contact_j = contact_v * current_a * dt;
        let source_loss_j = r.source_loss_v * supply_v * current_a * dt;
        let source_j = supply_v * current_a * dt;
        computed(&[
            current_a,
            load_v,
            contact_v,
            load_j,
            contact_j,
            source_loss_j,
            source_j,
        ])?;
        Ok(ElectricalSegment {
            interval,
            closed,
            current_a,
            load_v,
            contact_v,
            load_j,
            contact_j,
            source_loss_j,
            source_j,
        })
    }
}

impl StepModel for ElectricalContact {
    type Input = ElectricalInput;
    type Output = ElectricalStep;

    fn advance(
        &mut self,
        interval: Interval,
        input: &Self::Input,
        _: &mut Rng,
    ) -> Result<Self::Output, ProcessError> {
        continuous(self.last, interval)?;
        finite(&[input.supply_v])?;
        let requested = if let Some(travel) = input.travel_m {
            finite(&[travel])?;
            if travel >= self.spec.close_above_m.nominal {
                Some(true)
            } else if travel <= self.spec.release_below_m.nominal {
                Some(false)
            } else {
                self.requested
            }
        } else {
            None
        };
        let transition = requested != self.requested
            || (input.fault == ContactFault::None && self.previous_fault != ContactFault::None);
        let changed = if transition {
            Some(interval.start)
        } else {
            self.changed
        };
        let single = |closed| {
            self.segment(interval, closed, input.supply_v)
                .map(|s| vec![s])
        };
        let segments = if !(self.spec.calibrated_supply_v.min..=self.spec.calibrated_supply_v.max)
            .contains(&input.supply_v)
        {
            unknown("напряжение питания вне калибровки контакта")
        } else {
            let result = match (input.fault, requested, changed, &self.spec.bounce) {
                (ContactFault::OpenCircuit, ..) => Some(single(false)?),
                (ContactFault::StuckClosed, ..) => Some(single(true)?),
                (ContactFault::None, Some(target), None, _) => Some(single(target)?),
                (
                    ContactFault::None,
                    Some(target),
                    Some(_),
                    Some(BounceModel::VerifiedAbsent { source }),
                ) if direct(source) => Some(single(target)?),
                (
                    ContactFault::None,
                    Some(target),
                    Some(start),
                    Some(BounceModel::Profile { closing, opening }),
                ) => {
                    let profile = if target { closing } else { opening };
                    let mut result = Vec::new();
                    let mut from = start;
                    for segment in profile {
                        let ns = segment_time(segment.duration_s.nominal)?.0;
                        let to = Time(
                            from.0
                                .checked_add(ns)
                                .ok_or_else(|| execution("дребезг вне шкалы времени"))?,
                        );
                        let begin = from.max(interval.start);
                        let end = to.min(interval.end);
                        if begin < end {
                            result.push(self.segment(
                                Interval { start: begin, end },
                                segment.follows_target == target,
                                input.supply_v,
                            )?);
                        }
                        from = to;
                        if from >= interval.end {
                            break;
                        }
                    }
                    if from < interval.end {
                        result.push(self.segment(
                            Interval {
                                start: from.max(interval.start),
                                end: interval.end,
                            },
                            target,
                            input.supply_v,
                        )?);
                    }
                    Some(result)
                }
                _ => None,
            };
            result.map_or_else(
                || unknown("неизвестны механическое положение или дребезг перехода"),
                |value| {
                    calculated(
                        value,
                        "резистивная цепь, кусочно-постоянные состояния, номиналы параметров",
                    )
                },
            )
        };
        let total = match (&segments, self.total_load_j) {
            (Analysis::Analyzed { value, .. }, Some(previous)) => {
                let total = previous + value.iter().map(|sample| sample.load_j).sum::<f64>();
                computed(&[total])?;
                Some(total)
            }
            _ => None,
        };
        self.requested = requested;
        self.changed = changed;
        self.previous_fault = input.fault;
        self.total_load_j = total;
        self.last = Some(interval.end);
        Ok(ElectricalStep {
            requested_closed: requested,
            supply_v: input.supply_v,
            fault: input.fault,
            segments,
            total_load_j: total.map_or_else(
                || unknown("неполное покрытие энергетической истории"),
                |value| calculated(value, "интеграл мощности нагрузки по рассчитанным участкам"),
            ),
        })
    }
}

fn segment_time(seconds: f64) -> Result<Time, ProcessError> {
    let time = Time::from_secs(seconds).filter(|time| *time > Time::ZERO);
    time.ok_or_else(|| invalid("длительность дребезга вне разрешения шкалы времени"))
}

fn validate_profile(profile: &[BounceSegment]) -> Result<(), ProcessError> {
    if profile.is_empty() {
        return Err(invalid("пустой профиль дребезга"));
    }
    let mut total = 0_u64;
    for segment in profile {
        range(
            &segment.duration_s,
            "длительность участка дребезга",
            |x| x > 0.0,
        )?;
        segment_time(segment.duration_s.min)?;
        total = total
            .checked_add(segment_time(segment.duration_s.max)?.0)
            .ok_or_else(|| invalid("профиль дребезга вне шкалы времени"))?;
    }
    Ok(())
}

fn response(
    closed: bool,
    ron: f64,
    roff: f64,
    series: f64,
    load: f64,
) -> Result<Response, ProcessError> {
    let state = if closed { "ON" } else { "OFF" };
    let terminal = if series > 0.0 { "terminal" } else { "feed" };
    // Линейный базис 1 В для точного масштабирования по напряжению источника.
    let mut circuit = format!(
        "Контакт\nVfeed feed 0 1\nScontact {terminal} load EXT {state} RON={ron:e} ROFF={roff:e}\nRload load 0 {load:e}\n"
    );
    if series > 0.0 {
        circuit.push_str(&format!("Rsource feed terminal {series:e}\n"));
    }
    let failed = |error| execution(format!("рабочая точка электрического контакта: {error}"));
    let circuit = Circuit::parse(&circuit).map_err(failed)?;
    // Цепь имеет ненулевой физический путь утечки. Дополнительные проводимости
    // решателя здесь меняли бы баланс тока и энергию при разомкнутом контакте.
    let opts = SimOptions {
        gshunt: 0.0,
        gmin: 0.0,
        ..SimOptions::default()
    };
    let op = operating_point_with(&circuit, opts).map_err(failed)?;
    let current_a = op.current("rload").map_err(failed)?;
    let load_v = op.voltage("load").map_err(failed)?;
    let terminal_v = op.voltage(terminal).map_err(failed)?;
    // Закон Ома для падения на малом сопротивлении устойчивее вычитания потенциалов.
    let contact_v = current_a * if closed { ron } else { roff };
    let source_loss_v = current_a * series;
    computed(&[current_a, load_v, terminal_v, contact_v, source_loss_v])?;
    Ok(Response {
        current_a,
        load_v,
        contact_v,
        source_loss_v,
    })
}
