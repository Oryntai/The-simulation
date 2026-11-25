use std::collections::BTreeSet;

use sim_circuit::SimOptions;

use crate::Analysis;

use super::{
    BindingTarget, CompiledSchematic, Contact, Endpoint, JointKind, Parameter, PathStep, Quantity,
    Scope,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Pass,
    Fail,
    NotAnalyzed,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StaticClass {
    AssemblyMismatch,
    MissingGround,
    ReversePolarity,
}

impl StaticClass {
    pub fn code(self) -> &'static str {
        match self {
            Self::AssemblyMismatch => "assembly_mismatch",
            Self::MissingGround => "missing_ground",
            Self::ReversePolarity => "reverse_polarity",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Finding<E> {
    pub subject: String,
    pub status: Status,
    pub reason: String,
    pub evidence: E,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Mating {
    pub ends: [Endpoint; 2],
    pub contacts: [Contact; 2],
    pub kind: JointKind,
    pub branch_connected: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ReturnPath {
    pub signal: Vec<PathStep>,
    pub signal_ends: [Endpoint; 2],
    pub references: [Endpoint; 2],
    pub return_path: Option<Vec<PathStep>>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Polarity {
    pub ends: [Endpoint; 2],
    pub voltage_v: Option<f64>,
    pub reverse_limit_v: Parameter,
    pub conditions: String,
}

#[derive(Clone, Debug)]
pub struct StaticReport {
    pub scope: Scope,
    pub assembly_mismatch: Vec<Finding<Mating>>,
    pub missing_ground: Vec<Finding<ReturnPath>>,
    pub unmatched_signals: Vec<Endpoint>,
    pub reverse_polarity: Vec<Finding<Polarity>>,
}

impl StaticReport {
    pub fn status(&self, class: StaticClass) -> Status {
        match class {
            StaticClass::AssemblyMismatch => summarize(&self.assembly_mismatch),
            StaticClass::MissingGround => {
                let status = summarize(&self.missing_ground);
                if status == Status::Pass && !self.unmatched_signals.is_empty() {
                    Status::NotAnalyzed
                } else {
                    status
                }
            }
            StaticClass::ReversePolarity => summarize(&self.reverse_polarity),
        }
    }
}

fn summarize<E>(findings: &[Finding<E>]) -> Status {
    if findings.iter().any(|item| item.status == Status::Fail) {
        Status::Fail
    } else if !findings.is_empty() && findings.iter().all(|item| item.status == Status::Pass) {
        Status::Pass
    } else {
        Status::NotAnalyzed
    }
}

impl CompiledSchematic {
    /// Три статических обязательства. Вывод относится к указанной схеме и номинальному DC.
    pub fn check_static(&self) -> StaticReport {
        let (missing_ground, unmatched_signals) = self.check_return();
        StaticReport {
            scope: self.schematic.scope,
            assembly_mismatch: self.check_mating(),
            missing_ground,
            unmatched_signals,
            reverse_polarity: self.check_polarity(),
        }
    }

    fn check_mating(&self) -> Vec<Finding<Mating>> {
        self.schematic
            .connections
            .iter()
            .map(|(id, connection)| {
                let mates = !self.mismatches.contains(id);
                Finding {
                    subject: id.clone(),
                    status: if mates { Status::Pass } else { Status::Fail },
                    reason: if mates {
                        "типы, семейство и позиция сопрягаются".into()
                    } else {
                        "несовместимые контакты, физическая ветвь разомкнута".into()
                    },
                    evidence: Mating {
                        ends: connection.ends.clone(),
                        contacts: connection
                            .ends
                            .each_ref()
                            .map(|end| self.pin(end).contact.clone()),
                        kind: connection.kind,
                        branch_connected: mates,
                    },
                }
            })
            .collect()
    }

    fn check_return(&self) -> (Vec<Finding<ReturnPath>>, Vec<Endpoint>) {
        let signals = self
            .pins()
            .filter(|end| self.pin(end).role.reference().is_some())
            .collect::<Vec<_>>();
        let net =
            |end: &Endpoint| self.schematic.components[&end.component].nets[&end.pin].as_str();
        let mut findings = Vec::new();
        let mut matched = BTreeSet::new();

        for &send in signals.iter().filter(|end| self.pin(end).role.sends()) {
            for &recv in signals
                .iter()
                .filter(|end| self.pin(end).role.receives() && **end != send)
            {
                let a = self.index[send];
                let b = self.index[recv];
                if net(send) == net(recv) || self.route(a, b, false).is_some() {
                    matched.extend([send, recv]);
                    findings.push(self.check_pair(send, recv));
                }
            }
        }
        let unmatched = signals
            .into_iter()
            .filter(|end| !matched.contains(end))
            .cloned()
            .collect();
        (findings, unmatched)
    }

    fn check_pair(&self, send: &Endpoint, recv: &Endpoint) -> Finding<ReturnPath> {
        let references = [send, recv].map(|end| Endpoint {
            component: end.component.clone(),
            pin: self.pin(end).role.reference().unwrap().into(),
        });
        let [a, b] = references.each_ref().map(|end| self.index[end]);
        let signal = self.route(self.index[send], self.index[recv], true);
        // Общим минусом не может быть та же ветвь, по которой прошёл сигнал.
        let used = signal
            .iter()
            .flatten()
            .map(|step| step.section.clone())
            .collect();
        let proven = self.route_avoiding(a, b, true, &used);
        let (status, reason, return_path) = match (signal.as_ref(), proven) {
            (None, proven) => (
                Status::NotAnalyzed,
                "нет подтверждённого пути сигнала",
                proven,
            ),
            (Some(_), Some(return_path)) => (
                Status::Pass,
                "найден штатный физический путь возврата",
                Some(return_path),
            ),
            (Some(_), None) => match self.route_avoiding(a, b, false, &used) {
                Some(path) => (
                    Status::NotAnalyzed,
                    "возврат зависит от неподтверждённого соединения или состояния ключа",
                    Some(path),
                ),
                None => (
                    Status::Fail,
                    "сигнал соединён, между локальными опорами нет штатного пути возврата",
                    None,
                ),
            },
        };
        Finding {
            subject: format!("{send} → {recv}"),
            status,
            reason: reason.into(),
            evidence: ReturnPath {
                signal: signal.unwrap_or_default(),
                signal_ends: [send.clone(), recv.clone()],
                references,
                return_path,
            },
        }
    }

    fn check_polarity(&self) -> Vec<Finding<Polarity>> {
        let targets = self.polarity_targets();
        if targets.is_empty() {
            return Vec::new();
        }
        let dc = self.dc();
        targets
            .into_iter()
            .map(|target| {
                let measurement = dc.voltage(&target.ends[0], &target.ends[1]);
                let (voltage_v, gap) = match measurement {
                    Analysis::Analyzed { value, .. } => (
                        Some(value),
                        dc.proof_gap(&target.ends[0]).map(str::to_owned),
                    ),
                    Analysis::NotAnalyzed { reason } => (None, Some(reason)),
                };
                let (status, reason) =
                    assess_polarity(self.schematic.scope, voltage_v, gap, &target.limit);
                Finding {
                    subject: target.subject,
                    status,
                    reason,
                    evidence: Polarity {
                        ends: target.ends,
                        voltage_v,
                        reverse_limit_v: target.limit,
                        conditions: target.conditions,
                    },
                }
            })
            .collect()
    }

    pub(super) fn polarity_targets(&self) -> Vec<PolarityTarget> {
        let mut targets = Vec::new();
        let mut covered = BTreeSet::new();
        for (id, component) in &self.schematic.components {
            let part = self.catalog.part(&component.part).unwrap();
            for (name, rating) in &part.ratings {
                if rating.quantity == Quantity::ReverseVoltageV {
                    let ends = rating.nodes.each_ref().map(|pin| Endpoint {
                        component: id.clone(),
                        pin: pin.clone(),
                    });
                    covered.insert(ends.clone());
                    targets.push(PolarityTarget {
                        subject: format!("{id}/{name}"),
                        ends,
                        limit: rating.limit.clone(),
                        conditions: rating.conditions.clone(),
                    });
                }
            }
        }
        for (id, rail) in &self.schematic.rails {
            covered.insert(rail.ends.clone());
            targets.push(PolarityTarget {
                subject: format!("шина «{id}»"),
                ends: rail.ends.clone(),
                limit: rail.reverse_limit_v.clone(),
                conditions: "установившееся DC шины".into(),
            });
        }
        for binding in &self.schematic.bindings {
            if let [plus, minus] = binding.ends.as_slice()
                && !matches!(&binding.target, BindingTarget::BoardPin { .. })
                && covered.insert([plus.clone(), minus.clone()])
            {
                targets.push(PolarityTarget {
                    subject: format!("питание {:?}", binding.target),
                    ends: [plus.clone(), minus.clone()],
                    limit: Parameter {
                        range: None,
                        measurement:
                            "уточнить допустимое обратное напряжение питания по паспорту нагрузки"
                                .into(),
                    },
                    conditions: "установившееся DC подключённой нагрузки".into(),
                });
            }
        }
        targets
    }
}

pub(super) struct PolarityTarget {
    pub subject: String,
    pub ends: [Endpoint; 2],
    pub limit: Parameter,
    pub conditions: String,
}

fn assess_polarity(
    scope: Scope,
    voltage: Option<f64>,
    gap: Option<String>,
    limit: &Parameter,
) -> (Status, String) {
    if let Some(gap) = gap {
        return (Status::NotAnalyzed, gap);
    }
    let Some((v, limit_range)) = voltage
        .zip(limit.range.as_ref())
        .filter(|(_, range)| range.can_prove() || scope.exact(range))
    else {
        return (
            Status::NotAnalyzed,
            format!("нет обоснованного предела, замер: {}", limit.measurement),
        );
    };
    // Защитный зазор сравнения взят из настроек напряжения MNA.
    let tolerance = SimOptions::default().vntol;
    if v < -limit_range.max - tolerance {
        (
            Status::Fail,
            "обратное напряжение превышает допустимое".into(),
        )
    } else if v > -limit_range.min + tolerance {
        (
            Status::Pass,
            "напряжение внутри предела обратной полярности в этом DC-режиме".into(),
        )
    } else {
        (
            Status::NotAnalyzed,
            "напряжение у границы численного допуска или внутри неопределённости предела".into(),
        )
    }
}

use super::{
    BoardContact, ClampPath, PinLimits, PinState, PreparedMode, Probe, Section, identifier, invalid,
};
use crate::{
    ProcessError, SourcedRange,
    work::{HazardCheck, electrical::assess_current_envelope},
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CurrentDirection {
    Absolute,
    Forward,
    Reverse,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PinChecks {
    pub absolute_voltage: HazardCheck,
    pub output_current: HazardCheck,
    pub absolute_pin_current: HazardCheck,
    /// Ток по направлению каждого защитного диода. Верхняя ветвь ориентирована к VDD.
    pub upper_injection: HazardCheck,
    pub lower_injection: HazardCheck,
    pub output_level: HazardCheck,
}

fn unknown(reason: impl Into<String>) -> HazardCheck {
    HazardCheck::NotAnalyzed {
        reason: reason.into(),
    }
}

impl PreparedMode {
    /// Общая проверка Overcurrent. Применимость предела по температуре и длительности
    /// устанавливается по данным испытания, передающим этот предел.
    pub fn check_current(
        &self,
        section: &Section,
        direction: CurrentDirection,
        limit: Option<&SourcedRange>,
    ) -> Result<HazardCheck, ProcessError> {
        if let Some(limit) = limit {
            limit.validate()?;
            if limit.min < 0.0 {
                return Err(invalid("отрицательный предел тока"));
            }
        }
        let Analysis::Analyzed { value, .. } = self.solve() else {
            return Ok(unknown("нет рассчитанной токовой огибающей"));
        };
        self.check_trace_current(&value, section, direction, limit)
    }

    fn check_trace_current(
        &self,
        value: &super::ElectricalSeries,
        section: &Section,
        direction: CurrentDirection,
        limit: Option<&SourcedRange>,
    ) -> Result<HazardCheck, ProcessError> {
        if let Some(bound) = self
            .dc_bounds
            .as_ref()
            .and_then(|bounds| bounds.get(&Probe::Current(section.clone())))
        {
            let (lo, hi) = (bound.min.value, bound.max.value);
            let peaks = match direction {
                CurrentDirection::Absolute => [
                    if lo <= 0.0 && hi >= 0.0 {
                        0.0
                    } else {
                        lo.abs().min(hi.abs())
                    },
                    lo.abs().max(hi.abs()),
                ],
                CurrentDirection::Forward => [lo.max(0.0), hi.max(0.0)],
                CurrentDirection::Reverse => [(-hi).max(0.0), (-lo).max(0.0)],
            };
            return assess_current_envelope(peaks, limit, true, true);
        }
        let Some(trace) = value
            .values
            .get(&Probe::Current(section.clone()))
            .filter(|trace| !trace.is_empty())
        else {
            return Ok(unknown("не наблюдался ток требуемой ветви"));
        };
        let peak = trace
            .iter()
            .map(|&i| match direction {
                CurrentDirection::Absolute => i.abs(),
                CurrentDirection::Forward => i.max(0.0),
                CurrentDirection::Reverse => (-i).max(0.0),
            })
            .fold(0.0, f64::max);
        assess_current_envelope(
            [peak, peak],
            limit,
            self.model_confirmed && (self.window.is_none() || self.linear_window),
            self.gaps.is_empty(),
        )
    }

    pub fn check_pin(
        &self,
        contact: &BoardContact,
        limits: &PinLimits,
    ) -> Result<PinChecks, ProcessError> {
        validate_pin_limits(limits)?;
        let missing = |reason: &str| PinChecks {
            absolute_voltage: unknown(reason),
            output_current: unknown(reason),
            absolute_pin_current: unknown(reason),
            upper_injection: unknown(reason),
            lower_injection: unknown(reason),
            output_level: unknown(reason),
        };
        let Some(stamp) = self.pins.get(contact) else {
            return Ok(missing("нет раскрытой модели контакта"));
        };
        let Some(model) = self.pin_models.get(contact) else {
            return Ok(missing("нет защитного профиля контакта"));
        };
        let Analysis::Analyzed { value, .. } = self.solve() else {
            return Ok(missing("нет наблюдений вывода"));
        };
        let [pad, vdd, vss] = &stamp.ends;
        let rail = value.values.get(&Probe::Voltage(vdd.clone(), vss.clone()));
        let applicable = rail.is_some_and(|values| {
            !values.is_empty() && values.iter().all(|&v| v == limits.nominal_supply_v)
        });
        let volts = value.values.get(&Probe::Voltage(pad.clone(), vss.clone()));
        let check_voltage = |limit: Option<[f64; 2]>| {
            let Some((volts, [min, max])) = volts.zip(limit).filter(|_| applicable) else {
                return unknown("нужен предел напряжения при фактическом VDD и локальной опоре");
            };
            if self.model_confirmed
                && (self.window.is_none() || self.linear_window)
                && volts.iter().any(|&v| v < min || v > max)
            {
                HazardCheck::Violation {
                    reason: format!(
                        "контакт выходит за диапазон {min}…{max} В ({})",
                        limits.source
                    ),
                }
            } else if self.gaps.is_empty()
                && !volts.is_empty()
                && volts.iter().all(|&v| v >= min && v <= max)
            {
                HazardCheck::WithinLimit {
                    scope: format!(
                        "диапазон {min}…{max} В при номинальном VDD ({})",
                        limits.source
                    ),
                }
            } else {
                unknown("напряжение контакта покрыто неполностью")
            }
        };
        let current_limit = |v| SourcedRange {
            min: v,
            nominal: v,
            max: v,
            provenance: sim_world::Provenance::Datasheet(limits.source.clone()),
        };
        let output_current = match (&stamp.output, limits.abs_pin_a) {
            (Some(section), Some(limit)) => self.check_trace_current(
                &value,
                section,
                CurrentDirection::Absolute,
                Some(&current_limit(limit)),
            )?,
            _ => unknown("нет активного драйвера или применимого абсолютного предела тока"),
        };
        let absolute_pin_current = if let Some(limit) = limits.abs_pin_a {
            let mut pad_out = vec![0.0_f64; value.time.len()];
            let mut complete = !pad_out.is_empty();
            for (branch, reverse) in stamp
                .output
                .iter()
                .chain(&stamp.upper_clamp)
                .chain(&stamp.passive)
                .map(|section| (section, false))
                .chain(stamp.lower_clamp.iter().map(|section| (section, true)))
            {
                if let Some(currents) = value.values.get(&Probe::Current(branch.clone())) {
                    // Нижняя защита ориентирована к площадке. Остальные ветви идут от неё.
                    for (total, &current) in pad_out.iter_mut().zip(currents) {
                        *total += if reverse { -current } else { current };
                    }
                } else {
                    complete = false;
                }
            }
            if pad_out.iter().any(|current| !current.is_finite()) {
                unknown("неконечная сумма токов площадки")
            } else {
                let peak = pad_out.into_iter().map(f64::abs).fold(0.0, f64::max);
                assess_current_envelope(
                    [peak, peak],
                    Some(&current_limit(limit)),
                    complete
                        && self.model_confirmed
                        && (self.window.is_none() || self.linear_window),
                    complete && self.gaps.is_empty(),
                )?
            }
        } else {
            unknown("нет абсолютного предела тока физического контакта")
        };
        let clamp =
            |branch: &Option<Section>, path: &ClampPath| -> Result<HazardCheck, ProcessError> {
                match (branch, path) {
                    (Some(section), ClampPath::Diode(d)) => self.check_trace_current(
                        &value,
                        section,
                        CurrentDirection::Forward,
                        d.injection_limit_a.range.as_ref(),
                    ),
                    _ => Ok(unknown(
                        "нет наблюдаемой защитной ветви или предела инжекции",
                    )),
                }
            };
        let output_level = match stamp.state {
            PinState::High => {
                threshold_level(volts.filter(|_| applicable), limits.voh_min_v, true, self)
            }
            PinState::Low => {
                threshold_level(volts.filter(|_| applicable), limits.vol_max_v, false, self)
            }
            _ => unknown("для входа требуется ожидаемый цифровой уровень и применимые пороги"),
        };
        Ok(PinChecks {
            absolute_voltage: check_voltage(limits.abs_input_v),
            output_current,
            absolute_pin_current,
            upper_injection: clamp(&stamp.upper_clamp, &model.upper_clamp)?,
            lower_injection: clamp(&stamp.lower_clamp, &model.lower_clamp)?,
            output_level,
        })
    }
}

impl PreparedMode {
    /// Сверяет ожидаемый цифровой уровень с VIL/VIH при номинальном локальном питании.
    pub fn check_input_level(
        &self,
        contact: &BoardContact,
        expected_high: bool,
        limits: &PinLimits,
    ) -> Result<HazardCheck, ProcessError> {
        validate_pin_limits(limits)?;
        let Some(pin) = self
            .pins
            .get(contact)
            .filter(|pin| pin.state.input_enabled())
        else {
            return Ok(unknown("входной буфер не подтверждён или отключён"));
        };
        let Analysis::Analyzed { value, .. } = self.solve() else {
            return Ok(unknown("нет наблюдений уровня входа"));
        };
        let [pad, vdd, vss] = &pin.ends;
        if !value
            .values
            .get(&Probe::Voltage(vdd.clone(), vss.clone()))
            .is_some_and(|rail| {
                !rail.is_empty() && rail.iter().all(|&v| v == limits.nominal_supply_v)
            })
        {
            return Ok(unknown("нужны VIL/VIH при этом питании"));
        }
        let threshold = if expected_high {
            limits.vih_min_v
        } else {
            limits.vil_max_v
        };
        Ok(threshold_level(
            value.values.get(&Probe::Voltage(pad.clone(), vss.clone())),
            threshold,
            expected_high,
            self,
        ))
    }
}

fn threshold_level(
    trace: Option<&Vec<f64>>,
    limit: f64,
    high: bool,
    mode: &PreparedMode,
) -> HazardCheck {
    let within = |v: f64| if high { v >= limit } else { v <= limit };
    match trace {
        Some(trace)
            if mode.model_confirmed
                && (mode.window.is_none() || mode.linear_window)
                && trace.iter().any(|&v| !within(v)) =>
        {
            HazardCheck::Violation {
                reason: format!("нарушен требуемый логический уровень {limit} В"),
            }
        }
        Some(trace)
            if mode.gaps.is_empty() && !trace.is_empty() && trace.iter().all(|&v| within(v)) =>
        {
            HazardCheck::WithinLimit {
                scope: format!("логический уровень {limit} В в области огибающей"),
            }
        }
        _ => unknown("логический уровень покрыт неполностью"),
    }
}

fn validate_pin_limits(limits: &PinLimits) -> Result<(), ProcessError> {
    let voltages = [
        limits.nominal_supply_v,
        limits.vil_max_v,
        limits.vih_min_v,
        limits.vol_max_v,
        limits.voh_min_v,
    ];
    let bad_absolute = limits
        .abs_input_v
        .is_some_and(|[lo, hi]| !lo.is_finite() || !hi.is_finite() || lo >= hi);
    let bad_currents = [
        limits.specified_source_a,
        limits.specified_sink_a,
        limits.abs_pin_a,
    ]
    .into_iter()
    .flatten()
    .any(|i| !i.is_finite() || i < 0.0);
    if voltages.into_iter().any(|v| !v.is_finite())
        || limits.nominal_supply_v <= 0.0
        || limits.vil_max_v >= limits.vih_min_v
        || limits.vol_max_v >= limits.voh_min_v
        || bad_absolute
        || bad_currents
        || !identifier(&limits.source)
    {
        return Err(invalid("некорректные пределы физического контакта"));
    }
    Ok(())
}
