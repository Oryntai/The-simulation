use std::collections::{BTreeMap, BTreeSet};

use crate::{
    Analysis, Interval, ProcessError, SourcedRange,
    work::{HazardCheck, cable_flex::*, electrical::assess_current_envelope},
};
use sim_circuit::SimOptions;
use sim_signal::Time;
use sim_world::Provenance;

use super::{
    Binding, BindingTarget, CompiledSchematic, Connection, ElectricalSeries, Endpoint, Envelope,
    Finding, JointKind, Mode, ModeCoverage, ModePlan, Parameter, Part, PartKind, Primitive,
    PrimitiveKind, Probe, Quantity, Scope, Section, Status, capacitor_stress::*, deep_discharge::*,
    fuse_coordination::*, identifier, invalid, sleep_drain::*, supports_source, switch_thermal::*,
    validate_source,
};

#[derive(Clone, Debug, PartialEq)]
pub struct AssemblyPart {
    pub component: String,
    pub part: String,
    pub kind: PartKind,
    pub source: String,
    pub pins: BTreeMap<String, String>,
    pub missing_parameters: BTreeMap<String, String>,
    pub model_gap: Option<String>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AssemblyOperation {
    pub id: String,
    pub kind: JointKind,
    pub ends: [Endpoint; 2],
    pub after: BTreeSet<String>,
    pub compatible: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    BeforePower,
    CurrentLimitedStart,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Expected {
    /// Переходное сопротивление или конкретный требуемый замер.
    ResistanceOhm(Parameter),
    /// Между этими цепями не предусмотрено прямого монтажного соединения.
    NoDirectBridge,
    /// Ориентация для визуальной сверки маркировки в указанных точках.
    PositiveMark,
    /// Знаковое напряжение и применимый обратный предел.
    ReverseVoltageV(Parameter),
    CurrentLimitA(Parameter),
    Continuity,
    SignalVoltage,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Inspection {
    pub subject: String,
    pub stage: Stage,
    pub ends: [Endpoint; 2],
    pub expected: Expected,
    pub conditions: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AssemblySheet {
    pub parts: Vec<AssemblyPart>,
    pub operations: Vec<AssemblyOperation>,
    pub bindings: Vec<Binding>,
    pub inspections: Vec<Inspection>,
}

impl CompiledSchematic {
    pub fn assembly_sheet(&self) -> AssemblySheet {
        let parts = self
            .schematic
            .components
            .iter()
            .map(|(id, component)| {
                let part = self.catalog.part(&component.part).unwrap();
                let mut missing_parameters = BTreeMap::new();
                for (branch, element) in &part.elements {
                    for (name, parameter) in &element.parameters {
                        if parameter.range.is_none() {
                            missing_parameters.insert(
                                format!("elements/{branch}/{name}"),
                                parameter.measurement.clone(),
                            );
                        }
                    }
                }
                for (name, rating) in &part.ratings {
                    if rating.limit.range.is_none() {
                        missing_parameters
                            .insert(format!("ratings/{name}"), rating.limit.measurement.clone());
                    }
                }
                AssemblyPart {
                    component: id.clone(),
                    part: component.part.clone(),
                    kind: part.kind,
                    source: part.source.clone(),
                    pins: component.nets.clone(),
                    missing_parameters,
                    model_gap: part.model_gap.clone(),
                }
            })
            .collect();

        let mut inspections = Vec::new();
        let operations = self.order.iter().map(|id| {
            let joint = &self.schematic.connections[id];
            let compatible = !self.mismatches.contains(id);
            if compatible {
                inspections.push(Inspection {
                    subject: id.clone(), stage: Stage::BeforePower, ends: joint.ends.clone(),
                    expected: Expected::ResistanceOhm(joint.resistance_ohm.clone()),
                    conditions: "обесточить и изолировать участок от параллельных цепей".into(),
                });
                inspections.push(Inspection {
                    subject: id.clone(), stage: Stage::CurrentLimitedStart, ends: joint.ends.clone(),
                    expected: Expected::CurrentLimitA(joint.current_limit_a.clone()),
                    conditions: "на первом включении с ограничением измерить ток последовательно этому участку".into(),
                });
            }
            AssemblyOperation {
                id: id.clone(), kind: joint.kind, ends: joint.ends.clone(),
                after: joint.after.clone(), compatible,
            }
        }).collect();

        for target in self.polarity_targets() {
            inspections.push(Inspection {
                subject: target.subject.clone(),
                stage: Stage::BeforePower,
                ends: target.ends.clone(),
                expected: Expected::PositiveMark,
                conditions: "обесточить, сверить маркировку плюса и возврата с монтажом".into(),
            });
            if self
                .route(
                    self.index[&target.ends[0]],
                    self.index[&target.ends[1]],
                    false,
                )
                .is_none()
            {
                inspections.push(Inspection {
                    subject: target.subject.clone(),
                    stage: Stage::BeforePower,
                    ends: target.ends.clone(),
                    expected: Expected::NoDirectBridge,
                    conditions: "обесточить, отключить детали и проверить изоляцию монтажных цепей"
                        .into(),
                });
            }
            inspections.push(Inspection {
                subject: target.subject,
                stage: Stage::CurrentLimitedStart,
                ends: target.ends,
                expected: Expected::ReverseVoltageV(target.limit),
                conditions: format!(
                    "измерить плюс относительно возврата при ограничении тока; {}",
                    target.conditions
                ),
            });
        }
        for item in self.check_static().missing_ground {
            if item.evidence.return_path.is_some() {
                inspections.push(Inspection {
                    subject: item.subject,
                    stage: Stage::BeforePower,
                    ends: item.evidence.references,
                    expected: Expected::Continuity,
                    conditions:
                        "прозвонить штатный возврат при снятом питании, без обхода через сигнал"
                            .into(),
                });
            }
        }
        for binding in self.bindings() {
            let (BindingTarget::BoardPin { board, .. }, [pad]) =
                (&binding.target, &binding.ends[..])
            else {
                continue;
            };
            for supply in self.bindings().iter().filter(|supply| {
                matches!(&supply.target,
                BindingTarget::BoardPower { board: owner, .. } if owner == board)
            }) {
                let [_, minus] = &supply.ends[..] else {
                    continue;
                };
                inspections.push(Inspection {
                    subject: format!("{:?}", binding.target), stage: Stage::CurrentLimitedStart,
                    ends: [pad.clone(), minus.clone()], expected: Expected::SignalVoltage,
                    conditions: "замерить уровень относительно локальной опоры при выключении, сбросе и загрузке".into(),
                });
            }
        }
        // Контроль без питания идёт перед измерениями первого включения.
        inspections.sort_by_key(|item| item.stage == Stage::CurrentLimitedStart);
        AssemblySheet {
            parts,
            operations,
            bindings: self.bindings().to_vec(),
            inspections,
        }
    }
}

/// Причины §4. Две токовые причины сводятся к одному классу вердикта.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum ElectricalClass {
    AssemblyMismatch,
    MissingGround,
    ReversePolarity,
    PinOvervoltage,
    GroundSwitchBackfeed,
    SourceConflict,
    RailSag,
    StartupActuation,
    InductiveSpike,
    SwitchUnderdrive,
    ConnectionOvercurrent,
    RegulatorOverload,
}

impl ElectricalClass {
    pub const ALL: [Self; 12] = [
        Self::AssemblyMismatch,
        Self::MissingGround,
        Self::ReversePolarity,
        Self::PinOvervoltage,
        Self::GroundSwitchBackfeed,
        Self::SourceConflict,
        Self::RailSag,
        Self::StartupActuation,
        Self::InductiveSpike,
        Self::SwitchUnderdrive,
        Self::ConnectionOvercurrent,
        Self::RegulatorOverload,
    ];

    pub fn code(self) -> &'static str {
        match self {
            Self::AssemblyMismatch => "assembly_mismatch",
            Self::MissingGround => "missing_ground",
            Self::ReversePolarity => "reverse_polarity",
            Self::PinOvervoltage => "pin_overvoltage",
            Self::GroundSwitchBackfeed => "ground_switch_backfeed",
            Self::SourceConflict => "source_conflict",
            Self::RailSag => "rail_sag",
            Self::StartupActuation => "startup_actuation",
            Self::InductiveSpike => "inductive_spike",
            Self::SwitchUnderdrive => "switch_underdrive",
            Self::ConnectionOvercurrent => "connection_overcurrent",
            Self::RegulatorOverload => "regulator_overload",
        }
    }

    fn observes(self, mode: &Mode) -> bool {
        match self {
            Self::AssemblyMismatch => false,
            Self::SourceConflict => matches!(
                mode,
                Mode::UsbOnly | Mode::MainOnly | Mode::Both | Mode::Normal
            ),
            Self::StartupActuation => matches!(mode, Mode::AllOff | Mode::Reset | Mode::Boot(_)),
            Self::InductiveSpike => matches!(mode, Mode::Stop | Mode::LoadOff | Mode::OpenWire(_)),
            Self::GroundSwitchBackfeed => matches!(
                mode,
                Mode::AllOff | Mode::Reset | Mode::LoadOff | Mode::OpenWire(_)
            ),
            _ => true,
        }
    }
}

#[derive(Clone, Debug)]
pub struct ElectricalFinding {
    pub class: ElectricalClass,
    pub subject: String,
    pub status: Status,
    pub reason: String,
    pub mode: Option<Mode>,
    pub probe: Option<Probe>,
    pub measurement: Option<[f64; 2]>,
    pub unit: Option<&'static str>,
    pub limit: Option<SourcedRange>,
    pub event: Option<Time>,
    pub witness: Option<String>,
    pub dissipation_w: Option<f64>,
    pub conditions: Option<String>,
}

impl ElectricalFinding {
    fn new(
        class: ElectricalClass,
        subject: impl Into<String>,
        status: Status,
        reason: impl Into<String>,
    ) -> Self {
        Self {
            class,
            subject: subject.into(),
            status,
            reason: reason.into(),
            mode: None,
            probe: None,
            measurement: None,
            unit: None,
            limit: None,
            event: None,
            witness: None,
            dissipation_w: None,
            conditions: None,
        }
    }

    fn open(class: ElectricalClass, subject: impl Into<String>, reason: impl Into<String>) -> Self {
        Self::new(class, subject, Status::NotAnalyzed, reason)
    }
}

#[derive(Clone, Debug)]
pub struct ElectricalMode {
    pub mode: Mode,
    pub horizon: Option<Time>,
    pub nominal: bool,
    pub journal: Option<u64>,
    pub envelope: bool,
    pub reason: Option<String>,
}

#[derive(Clone, Debug)]
pub struct ElectricalAssessment {
    pub scope: Scope,
    pub findings: Vec<ElectricalFinding>,
    pub inapplicable: BTreeMap<ElectricalClass, String>,
    pub modes: Vec<ElectricalMode>,
    pub absent_modes: BTreeMap<u8, String>,
}

#[derive(Clone, Copy)]
enum Sense {
    PeakCurrent,
    ForwardCurrent,
    ReverseVoltage,
    MinimumVoltage,
    AbsoluteVoltage,
    PeakPower,
    Observe(&'static str),
}

struct Witness {
    class: ElectricalClass,
    subject: String,
    probe: Probe,
    sense: Sense,
    limit: Option<Parameter>,
    conditions: String,
}

// Именованные паспортные пределы имеют фиксированные единицы и привязываются к
// физической паре выводов. Обычный токовый или обратный предел их не заменяет.
const OFF_SIGNAL_CURRENT: &str = "off_signal_current";
const MINIMUM_SUPPLY_VOLTAGE: &str = "minimum_supply_voltage";
const UNCOMMANDED_LOAD_CURRENT: &str = "uncommanded_load_current";
const REVERSE_OUTPUT_CURRENT: &str = "reverse_output_current";

impl CompiledSchematic {
    pub fn bindings(&self) -> &[Binding] {
        &self.schematic.bindings
    }

    /// Опыт берёт обязательные ветви из раскрытой схемы. Заявленные пробы могут
    /// расширить журнал. Закрытие класса требует подтверждённой огибающей.
    pub fn assess_electrical(&self, plan: &ModePlan) -> Result<ElectricalAssessment, ProcessError> {
        plan.cases.values().try_for_each(|case| {
            // Не даём автоматическим пробам скрыть некорректный исходный опыт.
            if case.probes.is_empty() {
                return Err(invalid("опыт требует наблюдаемые величины"));
            }
            Ok(())
        })?;
        let (applicable, inapplicable) = self.electrical_classes();
        let static_report = self.check_static();
        let mut findings = Vec::new();
        for item in static_report.assembly_mismatch {
            let mut finding = ElectricalFinding::new(
                ElectricalClass::AssemblyMismatch,
                item.subject,
                item.status,
                item.reason,
            );
            finding.witness = Some(format!("{:?}", item.evidence));
            findings.push(finding);
        }
        let return_paths = static_report
            .missing_ground
            .iter()
            .map(|item| (&item.subject, &item.evidence))
            .collect::<Vec<_>>();
        for item in &static_report.missing_ground {
            // Отдельный номинал не покрывает выключение и все обрывы.
            let status = self.static_electrical_status(item.status);
            let mut finding = ElectricalFinding::new(
                ElectricalClass::MissingGround,
                &item.subject,
                status,
                &item.reason,
            );
            finding.witness = Some(format!("{:?}", item.evidence));
            findings.push(finding);
        }
        for endpoint in static_report.unmatched_signals {
            findings.push(ElectricalFinding::open(
                ElectricalClass::MissingGround,
                endpoint.to_string(),
                "нет подтверждённой пары сигнальных выводов и её возврата",
            ));
        }
        for item in static_report.reverse_polarity {
            let mut finding = ElectricalFinding::new(
                ElectricalClass::ReversePolarity,
                item.subject,
                self.static_electrical_status(item.status),
                item.reason,
            );
            finding.probe = Some(Probe::Voltage(
                item.evidence.ends[0].clone(),
                item.evidence.ends[1].clone(),
            ));
            finding.measurement = item.evidence.voltage_v.map(|v| [v, v]);
            finding.unit = Some("V");
            finding.limit = item.evidence.reverse_limit_v.range;
            finding.conditions = Some(item.evidence.conditions);
            findings.push(finding);
        }
        let witnesses = self.electrical_witnesses();
        let mut resolved = plan.clone();
        for (mode, case) in &mut resolved.cases {
            case.probes.extend(
                witnesses
                    .iter()
                    .filter(|w| {
                        w.class.observes(mode)
                            && !matches!(mode, Mode::OpenWire(id)
                    if w.probe == Probe::Current(Section::Connection(id.clone())))
                    })
                    .map(|w| w.probe.clone()),
            );
        }
        let mut modes = Vec::new();
        let mut absent_modes = BTreeMap::new();
        for coverage in self.run_modes(&resolved)? {
            let report = match coverage {
                ModeCoverage::Inapplicable { number, reason } => {
                    absent_modes.insert(number, reason);
                    continue;
                }
                ModeCoverage::Case(report) => report,
            };
            let envelope = match &report.envelope {
                Analysis::Analyzed { value, .. } => Some(value),
                Analysis::NotAnalyzed { .. } => None,
            };
            let nominal = match &report.nominal {
                Analysis::Analyzed { value, .. } => Some(value.envelopes()),
                Analysis::NotAnalyzed { .. } => None,
            };
            let prepared = if nominal.is_some() {
                match self.prepare_mode(&resolved, &report.mode)? {
                    Analysis::Analyzed { value, .. } => Some(value),
                    Analysis::NotAnalyzed { .. } => None,
                }
            } else {
                None
            };
            let point_confirmed = prepared.as_ref().is_some_and(|mode| {
                mode.model_confirmed && (mode.window.is_none() || mode.linear_window)
            });
            let reference_point = self.schematic.scope == Scope::Reference
                && prepared
                    .as_ref()
                    .is_some_and(|mode| mode.model_confirmed && mode.window.is_none());
            let reference_repeat = if self.schematic.scope == Scope::Reference
                && witnesses
                    .iter()
                    .any(|w| w.limit.is_some() && w.needs_reference_repeat(&report.mode))
                && report
                    .source_fronts
                    .values()
                    .any(|front| front.points.windows(2).any(|pair| pair[0].1 != pair[1].1))
                && let Some(mode) = &prepared
            {
                reference_repeat(mode)
            } else {
                None
            };
            let reason = match &report.envelope {
                Analysis::NotAnalyzed { reason } => Some(reason.clone()),
                _ => None,
            };
            for &class in &applicable {
                if !class.observes(&report.mode) {
                    continue;
                }
                let start = findings.len();
                if class == ElectricalClass::MissingGround && prepared.is_some() {
                    findings.extend(return_paths.iter().map(|(label, path)| {
                        self.check_mode_return(label, path, &report.mode, &report.opened)
                    }));
                }
                if class == ElectricalClass::SourceConflict
                    && point_confirmed
                    && let Analysis::Analyzed { value, .. } = &report.nominal
                {
                    findings.extend(self.check_parallel_sources(
                        value,
                        &report.mode,
                        &report.opened,
                    ));
                }
                for witness in witnesses.iter().filter(|w| w.class == class) {
                    let disconnected = match &witness.probe {
                        Probe::Current(section) | Probe::Power(section) => {
                            report.opened.contains(section)
                        }
                        _ => false,
                    };
                    if disconnected {
                        continue;
                    }
                    let complete = envelope.and_then(|v| v.get(&witness.probe));
                    let observed = nominal.as_ref().and_then(|v| v.get(&witness.probe));
                    let mut finding = witness.finding(
                        complete,
                        observed,
                        reference_point,
                        reference_repeat
                            .as_ref()
                            .filter(|_| witness.needs_reference_repeat(&report.mode))
                            .and_then(|rows| rows.get(&witness.probe)),
                        point_confirmed,
                        self.schematic.scope,
                    )?;
                    if let Probe::Current(Section::Connection(id)) = &witness.probe
                        && class == ElectricalClass::ConnectionOvercurrent
                        && let Some([_, peak]) = finding.measurement
                        && let Some(resistance) =
                            &self.schematic.connections[id].resistance_ohm.range
                    {
                        // Геометрическая модель участка — резистор. Для температуры нужны Cth и отвод.
                        let dissipation = peak * peak * resistance.max;
                        if !dissipation.is_finite() {
                            return Err(invalid("диссипация участка вышла за конечную область"));
                        }
                        finding.dissipation_w = Some(dissipation);
                    }
                    if finding.measurement.is_none() {
                        finding.reason = reason
                            .clone()
                            .unwrap_or_else(|| "нет ряда нужной ветви".into());
                    }
                    finding.mode = Some(report.mode.clone());
                    findings.push(finding);
                }
                if class == ElectricalClass::PinOvervoltage
                    && let Some(prepared) = &prepared
                    && let Some(nominal) = &nominal
                {
                    findings.extend(pin_injection(
                        prepared,
                        nominal,
                        &report.mode,
                        reference_point,
                    )?);
                }
                if class == ElectricalClass::RegulatorOverload {
                    for (id, _) in self
                        .part_instances()
                        .filter(|(_, part)| is_regulator(part.kind))
                    {
                        if !witnesses.iter().any(|w| {
                            w.class == class
                                && matches!(&w.probe,
                            Probe::Current(Section::Element { component, .. }) if component == id)
                        }) {
                            let mut finding =
                                ElectricalFinding::open(class, id, class.missing_witness());
                            finding.mode = Some(report.mode.clone());
                            findings.push(finding);
                        }
                    }
                }
                if findings.len() == start {
                    let mut finding = ElectricalFinding::open(
                        class,
                        "схема",
                        reason.as_deref().unwrap_or_else(|| class.missing_witness()),
                    );
                    finding.mode = Some(report.mode.clone());
                    findings.push(finding);
                }
            }
            modes.push(ElectricalMode {
                mode: report.mode.clone(),
                horizon: report.window.map(|w| w.end),
                nominal: nominal.is_some(),
                journal: match &report.nominal {
                    Analysis::Analyzed { value, .. } => {
                        let mut material =
                            format!("{:?}{:?}{:?}", report.mode, report.source_fronts, value);
                        if let Some(repeat) = &reference_repeat {
                            material.push_str(&format!("{repeat:?}"));
                        }
                        Some(sim_signal::fnv1a(material.bytes()))
                    }
                    Analysis::NotAnalyzed { .. } => None,
                },
                envelope: envelope.is_some(),
                reason,
            });
        }
        for class in applicable {
            if !findings.iter().any(|finding| finding.class == class) {
                findings.push(ElectricalFinding::open(
                    class,
                    "схема",
                    class.missing_witness(),
                ));
            }
        }
        Ok(ElectricalAssessment {
            scope: self.schematic.scope,
            findings,
            inapplicable,
            modes,
            absent_modes,
        })
    }

    fn static_electrical_status(&self, status: Status) -> Status {
        if self.schematic.scope == Scope::Reference || status == Status::Fail {
            status
        } else {
            Status::NotAnalyzed
        }
    }

    fn check_mode_return(
        &self,
        subject: &str,
        path: &super::ReturnPath,
        mode: &Mode,
        opened: &BTreeSet<Section>,
    ) -> ElectricalFinding {
        let route = |ends: &[Endpoint; 2], assured, exclude: &BTreeSet<Section>| {
            self.route_avoiding(self.index[&ends[0]], self.index[&ends[1]], assured, exclude)
        };
        let mut finding = ElectricalFinding::open(
            ElectricalClass::MissingGround,
            subject,
            "требуется проверить состояние сигнала и его штатный возврат",
        );
        finding.mode = Some(mode.clone());
        if let Some(signal) = route(&path.signal_ends, true, opened) {
            let excluded = opened
                .iter()
                .cloned()
                .chain(signal.iter().map(|step| step.section.clone()))
                .collect();
            if let Some(return_path) = route(&path.references, true, &excluded) {
                finding.status = Status::Pass;
                finding.reason = "штатный возврат доступен в топологии режима".into();
                finding.witness = Some(format!("{:?} / {:?}", signal, return_path));
            } else if matches!(
                mode,
                Mode::MainOnly | Mode::UsbOnly | Mode::Both | Mode::Normal
            ) && route(&path.references, false, &excluded).is_none()
            {
                finding.status = Status::Fail;
                finding.reason = "сигнальный путь соединён без возврата между опорами".into();
                finding.witness = Some(format!("{:?} / {:?}", signal, path.references));
            }
        }
        finding
    }

    fn check_parallel_sources(
        &self,
        series: &ElectricalSeries,
        mode: &Mode,
        opened: &BTreeSet<Section>,
    ) -> Vec<ElectricalFinding> {
        let sources: Vec<_> = self
            .part_instances()
            .filter_map(|(id, part)| {
                if !is_source(part.kind) {
                    return None;
                }
                let pin = |role: super::PinRole| {
                    let mut pins = part.pins.iter().filter(|(_, pin)| pin.role == role);
                    let (name, _) = pins.next()?;
                    pins.next().is_none().then(|| Endpoint {
                        component: id.into(),
                        pin: name.clone(),
                    })
                };
                let mut elements = part
                    .elements
                    .iter()
                    .filter(|(_, element)| element.kind == PrimitiveKind::Voltage);
                let (name, _) = elements.next()?;
                if elements.next().is_some() {
                    return None;
                }
                let probe = Probe::Current(Section::Element {
                    component: id.into(),
                    element: name.clone(),
                });
                Some((
                    id,
                    [pin(super::PinRole::Power)?, pin(super::PinRole::Return)?],
                    series.values.get(&probe)?,
                    probe,
                ))
            })
            .collect();
        let mut findings = Vec::new();
        for (receiver, recv_ends, recv, probe) in &sources {
            for (sender, send_ends, send, _) in &sources {
                if receiver == sender {
                    continue;
                }
                let paths = recv_ends
                    .each_ref()
                    .into_iter()
                    .zip(send_ends)
                    .map(|(a, b)| self.route_avoiding(self.index[a], self.index[b], true, opened))
                    .collect::<Option<Vec<_>>>();
                let Some(paths) = paths else { continue };
                // Это численный допуск MNA для обнаружения направления. Физический
                // допуск обратного тока определяется паспортом, если параллель разрешён.
                let tolerance = SimOptions::default().abstol;
                let peak = recv
                    .iter()
                    .zip(*send)
                    .zip(&series.time)
                    .filter(|((into, out), _)| **into > tolerance && **out < -tolerance)
                    .max_by(|((a, _), _), ((b, _), _)| a.total_cmp(b));
                if let Some(((current, _), at)) = peak {
                    let mut finding = ElectricalFinding::new(
                        ElectricalClass::SourceConflict,
                        format!("{sender} → {receiver}"),
                        Status::Fail,
                        "прямое параллельное включение подаёт ток в выход другого источника",
                    );
                    finding.mode = Some(mode.clone());
                    finding.probe = Some(probe.clone());
                    finding.measurement = Some([*current; 2]);
                    finding.unit = Some("A");
                    finding.event = Some(*at);
                    finding.witness = Some(format!("{paths:?}"));
                    findings.push(finding);
                }
            }
        }
        findings
    }

    fn part_instances(&self) -> impl Iterator<Item = (&str, &Part)> {
        self.schematic
            .components
            .iter()
            .map(|(id, component)| (id.as_str(), self.catalog.part(&component.part).unwrap()))
    }

    fn electrical_classes(&self) -> (BTreeSet<ElectricalClass>, BTreeMap<ElectricalClass, String>) {
        use ElectricalClass::*;
        let has =
            |predicate: fn(&Part) -> bool| self.part_instances().any(|(_, part)| predicate(part));
        let primitive = |kind| {
            self.part_instances()
                .any(|(_, part)| part.elements.values().any(|e| e.kind == kind))
        };
        let signal = self
            .pins()
            .any(|endpoint| self.pin(endpoint).role.reference().is_some());
        let board_pin = self
            .bindings()
            .iter()
            .any(|b| matches!(b.target, BindingTarget::BoardPin { .. }));
        let board_power = self
            .bindings()
            .iter()
            .any(|b| matches!(b.target, BindingTarget::BoardPower { .. }));
        let drive = self
            .bindings()
            .iter()
            .any(|b| matches!(b.target, BindingTarget::DrivePower { .. }));
        let powered_load = drive
            || self
                .bindings()
                .iter()
                .any(|b| matches!(b.target, BindingTarget::SensorPower { .. }));
        let switch = has(|part| {
            matches!(
                part.kind,
                PartKind::SwitchModule | PartKind::Nmos | PartKind::Pmos
            )
        }) || [
            PrimitiveKind::Switch,
            PrimitiveKind::Nmos,
            PrimitiveKind::Pmos,
        ]
        .into_iter()
        .any(primitive);
        let sources = self
            .part_instances()
            .filter(|(_, part)| {
                is_source(part.kind)
                    || part
                        .elements
                        .values()
                        .any(|e| e.kind == PrimitiveKind::Voltage)
            })
            .count();
        let unknown = self.part_instances().any(|(_, p)| p.model_gap.is_some());
        let mut applicable = BTreeSet::new();
        let mut inapplicable = BTreeMap::new();
        for (class, present, absent) in [
            (
                AssemblyMismatch,
                !self.schematic.connections.is_empty(),
                "нет физических монтажных соединений",
            ),
            (
                MissingGround,
                signal || board_pin,
                "нет сигнальных выводов и связей платы",
            ),
            (
                ReversePolarity,
                !self.polarity_targets().is_empty()
                    || has(|p| matches!(p.kind, PartKind::PolarCapacitor) || is_source(p.kind)),
                "нет полярных элементов и питаемых клемм",
            ),
            (
                PinOvervoltage,
                signal || board_pin,
                "нет сигнальных выводов и связей платы",
            ),
            (
                GroundSwitchBackfeed,
                switch || drive || (signal && board_pin),
                "нет ключей, силовой нагрузки и путей сигнала платы",
            ),
            (
                SourceConflict,
                sources > 1,
                "в разрешённой схеме меньше двух отдельных источников",
            ),
            (
                RailSag,
                board_power
                    || board_pin
                    || powered_load
                    || switch
                    || primitive(PrimitiveKind::Current)
                    || self
                        .part_instances()
                        .any(|(_, part)| supply_rating(part).is_some()),
                "в разрешённой схеме нет питаемой платы, управляемой нагрузки или вывода с порогом питания",
            ),
            (
                StartupActuation,
                switch || drive || board_pin,
                "в разрешённой схеме нет ключа, привода или управляющего вывода платы",
            ),
            (
                InductiveSpike,
                primitive(PrimitiveKind::Inductor) || drive,
                "нет индуктивной ветви и модели привода",
            ),
            (
                SwitchUnderdrive,
                switch || drive,
                "нет силового ключа и модели привода",
            ),
            (
                ConnectionOvercurrent,
                !self.schematic.connections.is_empty() || has(|p| is_conductor(p.kind)),
                "нет проводных участков и физических соединений",
            ),
            (
                RegulatorOverload,
                has(|p| is_regulator(p.kind)),
                "нет стабилизатора или модуля питания",
            ),
        ] {
            if present || unknown {
                applicable.insert(class);
            } else {
                inapplicable.insert(class, absent.into());
            }
        }
        (applicable, inapplicable)
    }

    fn electrical_witnesses(&self) -> Vec<Witness> {
        use ElectricalClass::*;
        let mut witnesses = Vec::new();
        for target in self.polarity_targets() {
            // Предел, написанный возле шины, не удостоверяет обратный предел нагрузки.
            let catalog_limit = self.part_instances().any(|(id, part)| {
                part.ratings.values().any(|rating| {
                    rating.quantity == Quantity::ReverseVoltageV
                        && rating.nodes.each_ref().map(|pin| Endpoint {
                            component: id.into(),
                            pin: pin.clone(),
                        }) == target.ends
                        && rating.limit == target.limit
                })
            });
            let [plus, minus] = target.ends;
            witnesses.push(Witness {
                class: ReversePolarity,
                subject: target.subject,
                probe: Probe::Voltage(plus, minus),
                sense: Sense::ReverseVoltage,
                limit: catalog_limit.then_some(target.limit),
                conditions: target.conditions,
            });
        }
        for (id, joint) in &self.schematic.connections {
            // Провод и его монтажное соединение проверяются отдельно.
            let net_role = joint
                .ends
                .iter()
                .any(|end| self.pin(end).role.reference().is_some());
            let probe = Probe::Current(Section::Connection(id.clone()));
            witnesses.push(Witness {
                class: ConnectionOvercurrent,
                subject: id.clone(),
                probe: probe.clone(),
                sense: Sense::PeakCurrent,
                limit: Some(joint.current_limit_a.clone()),
                conditions: "предел монтажного участка".into(),
            });
            if net_role {
                witnesses.push(Witness {
                    class: GroundSwitchBackfeed,
                    subject: id.clone(),
                    probe,
                    sense: Sense::Observe("A"),
                    limit: None,
                    conditions: "снять обратный ток и остаточное питание при разомкнутом возврате"
                        .into(),
                });
            }
        }
        for binding in self.bindings() {
            match (&binding.target, binding.ends.as_slice()) {
                (BindingTarget::BoardPower { .. }, [plus, minus]) => witnesses.push(Witness {
                    class: RailSag,
                    subject: format!("{:?}", binding.target),
                    probe: Probe::Voltage(plus.clone(), minus.clone()),
                    sense: Sense::Observe("V"),
                    limit: None,
                    conditions: "снять порог и выдержку сброса относительно локального возврата"
                        .into(),
                }),
                (
                    BindingTarget::DrivePower { .. } | BindingTarget::SensorPower { .. },
                    [plus, minus],
                ) => {
                    witnesses.push(Witness {
                        class: GroundSwitchBackfeed,
                        subject: format!("{:?}", binding.target),
                        probe: Probe::Voltage(plus.clone(), minus.clone()),
                        sense: Sense::Observe("V"),
                        limit: None,
                        conditions: "снять остаточное питание на нагрузке при разомкнутом возврате"
                            .into(),
                    });
                }
                (BindingTarget::BoardPin { board, .. }, [pad]) => {
                    for supply in self.bindings().iter().filter(|supply| {
                        matches!(&supply.target,
                        BindingTarget::BoardPower { board: owner, .. } if owner == board)
                    }) {
                        let [_, minus] = &supply.ends[..] else {
                            continue;
                        };
                        if pad == minus {
                            continue;
                        }
                        witnesses.push(Witness {
                            class: PinOvervoltage, subject: format!("{:?}", binding.target),
                            probe: Probe::Voltage(pad.clone(), minus.clone()), sense: Sense::Observe("V"), limit: None,
                            conditions: "снять предел и ток защитной ветви для фактического состояния питания".into(),
                        });
                    }
                }
                _ => {}
            }
        }
        for (id, part) in self.part_instances() {
            if let Some(rating) = supply_rating(part) {
                let [plus, minus] = rating.nodes.each_ref().map(|pin| Endpoint {
                    component: id.into(),
                    pin: pin.clone(),
                });
                witnesses.push(Witness {
                    class: RailSag,
                    subject: format!("{id}/{MINIMUM_SUPPLY_VOLTAGE}"),
                    probe: Probe::Voltage(plus, minus),
                    sense: Sense::MinimumVoltage,
                    limit: Some(rating.limit.clone()),
                    conditions: rating.conditions.clone(),
                });
            }
            let fixed_off = part.elements.values().any(|element| {
                element.kind == PrimitiveKind::Switch
                    && element.nodes[2] == element.nodes[3]
                    && element.parameters["threshold_v"]
                        .range
                        .as_ref()
                        .is_some_and(|range| self.schematic.scope.exact(range) && range.min > 0.0)
                    && element.parameters["hysteresis_v"]
                        .range
                        .as_ref()
                        .is_some_and(|range| self.schematic.scope.exact(range) && range.max == 0.0)
            });
            for (name, element) in &part.elements {
                let section = Section::Element {
                    component: id.into(),
                    element: name.clone(),
                };
                let subject = format!("{id}/{name}");
                if !is_regulator(part.kind) && element.kind == PrimitiveKind::Voltage {
                    let rating = source_output_rating(part, element);
                    witnesses.push(Witness {
                        class: SourceConflict,
                        subject: subject.clone(),
                        probe: Probe::Current(section.clone()),
                        sense: Sense::ForwardCurrent,
                        limit: rating.map(|r| r.limit.clone()),
                        conditions: rating.map_or_else(
                            || "нужен паспортный предел тока, входящего в плюс источника".into(),
                            |r| r.conditions.clone(),
                        ),
                    });
                }
                let branch_nodes = branch_ends(element);
                for (class, key, suitable) in [
                    (GroundSwitchBackfeed, OFF_SIGNAL_CURRENT, fixed_off),
                    (StartupActuation, UNCOMMANDED_LOAD_CURRENT, true),
                ] {
                    if let Some(rating) = named_rating(part, key, Quantity::CurrentA)
                        && suitable
                        && element.kind == PrimitiveKind::Resistor
                        && matches_pair(rating, branch_nodes)
                    {
                        witnesses.push(Witness {
                            class,
                            subject: subject.clone(),
                            probe: Probe::Current(section.clone()),
                            sense: Sense::PeakCurrent,
                            limit: Some(rating.limit.clone()),
                            conditions: rating.conditions.clone(),
                        });
                    }
                }
                let pair = |a: usize, b: usize| {
                    let [plus, minus] = [a, b].map(|n| Endpoint {
                        component: id.into(),
                        pin: element.nodes[n].clone(),
                    });
                    (plus != minus
                        && part.pins.contains_key(&plus.pin)
                        && part.pins.contains_key(&minus.pin))
                    .then_some(Probe::Voltage(plus, minus))
                };
                if element.kind == PrimitiveKind::Inductor
                    && let Some(probe) = pair(0, 1)
                {
                    witnesses.push(Witness {
                        class: InductiveSpike,
                        subject: subject.clone(),
                        probe,
                        sense: Sense::Observe("V"),
                        limit: None,
                        conditions: "измерить пик на выводах индуктивности при снятии питания"
                            .into(),
                    });
                }
                if matches!(
                    element.kind,
                    PrimitiveKind::Nmos | PrimitiveKind::Pmos | PrimitiveKind::Switch
                ) {
                    let control = if element.kind == PrimitiveKind::Switch {
                        pair(2, 3)
                    } else {
                        pair(1, 2)
                    };
                    if let Some(probe) = control {
                        witnesses.push(Witness {
                            class: SwitchUnderdrive, subject: subject.clone(), probe,
                            sense: Sense::Observe("V"), limit: None,
                            conditions: "измерить напряжение управления относительно опоры и сверить с огибающей потерь".into(),
                        });
                    }

                    let nodes = branch_ends(element);
                    witnesses.push(rated(
                        part,
                        id,
                        &subject,
                        nodes,
                        SwitchUnderdrive,
                        Probe::Power(section.clone()),
                        Sense::PeakPower,
                        Quantity::PowerW,
                    ));
                    witnesses.push(Witness {
                        class: StartupActuation, subject: subject.clone(), probe: Probe::Current(section.clone()),
                        sense: Sense::Observe("A"), limit: None,
                        conditions: "снять реально разрешённый ток и движение нагрузки на сбросе и при загрузке".into(),
                    });
                }
                if is_conductor(part.kind) || is_regulator(part.kind) {
                    let class = if is_conductor(part.kind) {
                        ConnectionOvercurrent
                    } else {
                        RegulatorOverload
                    };
                    let nodes = branch_ends(element);
                    let witness = rated(
                        part,
                        id,
                        &subject,
                        nodes,
                        class,
                        Probe::Current(section),
                        Sense::PeakCurrent,
                        Quantity::CurrentA,
                    );
                    if is_conductor(part.kind) || witness.limit.is_some() {
                        witnesses.push(witness);
                    }
                }
            }
            for (name, rating) in &part.ratings {
                if rating.quantity == Quantity::VoltageV && name != MINIMUM_SUPPLY_VOLTAGE {
                    let [plus, minus] = rating.nodes.each_ref().map(|pin| Endpoint {
                        component: id.into(),
                        pin: pin.clone(),
                    });
                    witnesses.push(Witness {
                        class: InductiveSpike,
                        subject: format!("{id}/{name}"),
                        probe: Probe::Voltage(plus, minus),
                        sense: Sense::AbsoluteVoltage,
                        limit: Some(rating.limit.clone()),
                        conditions: rating.conditions.clone(),
                    });
                }
            }
        }
        witnesses
    }
}

fn is_source(kind: PartKind) -> bool {
    matches!(
        kind,
        PartKind::UsbSource | PartKind::Battery | PartKind::BenchSupply
    )
}

fn is_regulator(kind: PartKind) -> bool {
    matches!(
        kind,
        PartKind::LinearRegulator | PartKind::SwitchingRegulator | PartKind::ChargePowerModule
    )
}

fn is_conductor(kind: PartKind) -> bool {
    matches!(
        kind,
        PartKind::Wire | PartKind::ScrewTerminal | PartKind::Header | PartKind::SolderJoint
    )
}

fn branch_ends(element: &Primitive) -> [&str; 2] {
    // Порядок D,G,S,B у MOSFET и порядок клемм двухполюсника из каталога.
    let second = if matches!(element.kind, PrimitiveKind::Nmos | PrimitiveKind::Pmos) {
        2
    } else {
        1
    };
    [&element.nodes[0], &element.nodes[second]]
}

fn named_rating<'a>(part: &'a Part, name: &str, quantity: Quantity) -> Option<&'a super::Rating> {
    part.ratings
        .get(name)
        .filter(|rating| rating.quantity == quantity)
}

fn power_port(part: &Part, rating: &super::Rating) -> bool {
    part.pins[&rating.nodes[0]].role == super::PinRole::Power
        && part.pins[&rating.nodes[1]].role == super::PinRole::Return
}

fn supply_rating(part: &Part) -> Option<&super::Rating> {
    named_rating(part, MINIMUM_SUPPLY_VOLTAGE, Quantity::VoltageV)
        .filter(|rating| power_port(part, rating))
}

fn source_output_rating<'a>(part: &'a Part, emf: &Primitive) -> Option<&'a super::Rating> {
    let rating = named_rating(part, REVERSE_OUTPUT_CURRENT, Quantity::CurrentA)?;
    if !is_source(part.kind) || !power_port(part, rating) || part.pins.len() != 2 {
        return None;
    }
    // Ток ЭДС равен току входа в плюс только для двухполюсного
    // источника без ответвлений и с указанной ориентацией.
    let [positive, negative] = rating.nodes.each_ref().map(String::as_str);
    let [emf_plus, emf_minus] = branch_ends(emf);
    if emf_minus != negative {
        return None;
    }
    let resolved = match part.elements.len() {
        1 => emf_plus == positive,
        2 => {
            !part.pins.contains_key(emf_plus)
                && part.elements.values().any(|element| {
                    element.kind == PrimitiveKind::Resistor
                        && (branch_ends(element) == [emf_plus, positive]
                            || branch_ends(element) == [positive, emf_plus])
                })
        }
        _ => false,
    };
    resolved.then_some(rating)
}

fn matches_pair(rating: &super::Rating, nodes: [&str; 2]) -> bool {
    let rated = rating.nodes.each_ref().map(String::as_str);
    rated == nodes || rated == [nodes[1], nodes[0]]
}

fn rated(
    part: &Part,
    id: &str,
    subject: &str,
    nodes: [&str; 2],
    class: ElectricalClass,
    probe: Probe,
    sense: Sense,
    quantity: Quantity,
) -> Witness {
    // Если применимость нескольких пределов двусмысленна, требуется их подтверждение.
    let mut ratings = part
        .ratings
        .values()
        .filter(|r| r.quantity == quantity && matches_pair(r, nodes));
    let rating = ratings.next().filter(|_| ratings.next().is_none());
    Witness {
        class,
        subject: subject.into(),
        probe,
        sense,
        limit: rating.map(|r| r.limit.clone()),
        conditions: rating.map_or_else(
            || format!("{id}: требуется предел для конкретной ветви"),
            |r| r.conditions.clone(),
        ),
    }
}

impl ElectricalClass {
    fn missing_witness(self) -> &'static str {
        match self {
            Self::MissingGround => {
                "проследить штатный возврат и состояние при каждом обрыве питания или земли"
            }
            Self::PinOvervoltage => {
                "проверить пределы вывода и ток инжекции для каждого состояния платы"
            }
            Self::SourceConflict => {
                "измерить обратный ток каждого источника при совместном и раздельном подключении"
            }
            Self::RailSag => "нужны порог, выдержка и живой отклик платы на просадку",
            Self::StartupActuation => {
                "нужны реальные состояния выводов и отклик нагрузки при сбросе и загрузке"
            }
            Self::InductiveSpike => "нужны разрешённый фронт, пик на клеммах и импульсный предел",
            Self::SwitchUnderdrive => "нужны Vgs, потери и область допустимых режимов ключа",
            Self::GroundSwitchBackfeed => {
                "нужны путь обратного тока и остаточное питание при выключении"
            }
            Self::RegulatorOverload => {
                "нужны запрос нагрузки, разрешение выходной ветви и условия регулирования"
            }
            _ => "нет разрешённого участка или применимого предела",
        }
    }
}

fn reference_repeat(prepared: &super::PreparedMode) -> Option<BTreeMap<Probe, Envelope>> {
    let window = prepared.window.as_ref()?;
    // Без известного фронта две одинаковые рабочие точки не подтверждают переход.
    if !prepared.model_confirmed || prepared.events.is_empty() || window.step.0 % 2 != 0 {
        return None;
    }
    let mut fine = prepared.clone();
    fine.window.as_mut()?.step.0 /= 2;
    match fine.solve() {
        Analysis::Analyzed { value, .. } => Some(value.envelopes()),
        Analysis::NotAnalyzed { .. } => None,
    }
}

impl Witness {
    fn needs_reference_repeat(&self, mode: &Mode) -> bool {
        match self.sense {
            Sense::MinimumVoltage => matches!(mode, Mode::Start | Mode::Stall),
            Sense::AbsoluteVoltage => matches!(mode, Mode::Stop | Mode::LoadOff),
            _ => false,
        }
    }

    fn failure_reason(&self) -> &'static str {
        match self.class {
            ElectricalClass::GroundSwitchBackfeed => {
                "ток возврата через сигнальную ветвь при выключенном ключе превышает предел"
            }
            ElectricalClass::RailSag => "минимум напряжения шины ниже порога сброса",
            ElectricalClass::StartupActuation => {
                "ток нагрузки при сбросе или загрузке превышает предел непреднамеренного включения"
            }
            ElectricalClass::InductiveSpike => {
                "импульс напряжения при отключении индуктивной цепи превышает предел"
            }
            ElectricalClass::SwitchUnderdrive => {
                "потери канала при фактическом напряжении управления превышают предел"
            }
            _ => "подтверждённое наблюдение превышает применимый предел",
        }
    }

    fn finding(
        &self,
        complete: Option<&Envelope>,
        nominal: Option<&Envelope>,
        reference_point: bool,
        reference_repeat: Option<&Envelope>,
        point_confirmed: bool,
        scope: Scope,
    ) -> Result<ElectricalFinding, ProcessError> {
        let mut finding = ElectricalFinding::open(
            self.class,
            &self.subject,
            self.limit
                .as_ref()
                .map_or(self.conditions.as_str(), |limit| limit.measurement.as_str()),
        );
        finding.probe = Some(self.probe.clone());
        finding.conditions = Some(self.conditions.clone());
        let limit = self.limit.as_ref().and_then(|p| p.range.as_ref());
        finding.limit = limit.cloned();
        let Some(row) = complete.or(nominal) else {
            return Ok(finding);
        };
        let absolute_max = if row.min.value.abs() > row.max.value.abs() {
            &row.min
        } else {
            &row.max
        };
        let (bounds, event, unit) = match self.sense {
            Sense::PeakCurrent | Sense::AbsoluteVoltage => (
                [
                    if row.min.value > 0.0 || row.max.value < 0.0 {
                        row.min.value.abs().min(row.max.value.abs())
                    } else {
                        0.0
                    },
                    absolute_max.value.abs(),
                ],
                absolute_max.time,
                if matches!(self.sense, Sense::PeakCurrent) {
                    "A"
                } else {
                    "V"
                },
            ),
            Sense::ReverseVoltage => (
                [(-row.max.value).max(0.0), (-row.min.value).max(0.0)],
                row.min.time,
                "V",
            ),
            Sense::PeakPower => (
                [row.min.value.max(0.0), row.max.value.max(0.0)],
                row.max.time,
                "W",
            ),
            Sense::ForwardCurrent => (
                [row.min.value.max(0.0), row.max.value.max(0.0)],
                row.max.time,
                "A",
            ),
            Sense::MinimumVoltage => ([row.min.value; 2], row.min.time, "V"),
            Sense::Observe(unit) => ([row.min.value, row.max.value], row.min.time, unit),
        };
        finding.measurement = Some(bounds);
        finding.unit = Some(unit);
        finding.event = Some(event);

        if let Some((coarse, fine)) = nominal.zip(reference_repeat)
            && let Some(limit) = limit.filter(|r| r.can_prove())
        {
            let lower = matches!(self.sense, Sense::MinimumVoltage);
            let extreme = |row: &Envelope| {
                if lower {
                    (row.min.value, row.min.time)
                } else if row.min.value.abs() > row.max.value.abs() {
                    (row.min.value.abs(), row.min.time)
                } else {
                    (row.max.value.abs(), row.max.time)
                }
            };
            let (a, _) = extreme(coarse);
            let (b, at) = extreme(fine);
            let delta = (a - b).abs();
            // §5 электрического ТЗ: допуск пика 5%. Разность сеток
            // дополнительно отнимает запас до порога с обеих сторон.
            if delta <= 0.05 * b.abs() {
                let low = a.min(b) - delta;
                let high = a.max(b) + delta;
                if !low.is_finite() || !high.is_finite() {
                    return Err(invalid("границы пика вышли за конечную область"));
                }
                finding.measurement = Some([low, high]);
                finding.event = Some(at);
                if (lower && high < limit.min) || (!lower && low > limit.max) {
                    finding.status = Status::Fail;
                    finding.reason = self.failure_reason().into();
                } else if (lower && low > limit.max) || (!lower && high < limit.min) {
                    finding.status = Status::Pass;
                    finding.reason =
                        "на двух сетках экстремум стенда внутри применимого предела".into();
                }
                finding.witness =
                    Some("экстремумы на исходной и половинной сетках эталонного перехода".into());
            } else {
                finding.reason = "экстремум эталонного перехода расходится на двух сетках".into();
            }
            return Ok(finding);
        }
        if matches!(self.sense, Sense::MinimumVoltage) {
            // Просадка проверяется в переходе питания. Другие состояния шины
            // остаются измерениями, в том числе штатно отключённое питание.
            return Ok(finding);
        }
        // Текст условий каталога не доказывает температуру, режим стабилизатора,
        // доступную область ключа или импульсный рейтинг реальной сборки.
        let conditions_resolved = scope == Scope::Reference
            || matches!(self.probe, Probe::Current(Section::Connection(_)));
        let confirmed = (complete.is_some() || reference_point) && conditions_resolved;
        let witness = nominal.is_some() && point_confirmed && conditions_resolved;
        let tested = if witness && let Some(row) = nominal {
            match self.sense {
                Sense::PeakCurrent | Sense::AbsoluteVoltage => {
                    row.min.value.abs().max(row.max.value.abs())
                }
                Sense::ReverseVoltage => (-row.min.value).max(0.0),
                Sense::PeakPower | Sense::ForwardCurrent => row.max.value.max(0.0),
                Sense::Observe(_) | Sense::MinimumVoltage => return Ok(finding),
            }
        } else {
            bounds[0]
        };
        let test_bounds = [tested, tested.max(bounds[1])];
        let check = match self.sense {
            Sense::PeakCurrent => {
                assess_current_envelope(test_bounds, limit, witness || confirmed, confirmed)?
            }
            Sense::Observe(_) => return Ok(finding),
            _ => match limit.filter(|r| r.can_prove()) {
                Some(r) if (confirmed || witness) && tested > r.max => HazardCheck::Violation {
                    reason: "подтверждённое наблюдение превышает применимый предел".into(),
                },
                Some(r) if confirmed && bounds[1] < r.min => HazardCheck::WithinLimit {
                    scope: "вся огибающая внутри применимого предела".into(),
                },
                _ => HazardCheck::NotAnalyzed {
                    reason: finding.reason.clone(),
                },
            },
        };
        (finding.status, finding.reason) = match check {
            HazardCheck::Violation { reason } => (
                Status::Fail,
                if matches!(
                    self.class,
                    ElectricalClass::GroundSwitchBackfeed
                        | ElectricalClass::StartupActuation
                        | ElectricalClass::SwitchUnderdrive
                        | ElectricalClass::InductiveSpike
                ) {
                    self.failure_reason().into()
                } else {
                    reason
                },
            ),
            HazardCheck::WithinLimit { scope } => (Status::Pass, scope),
            HazardCheck::NotAnalyzed { reason } => (Status::NotAnalyzed, reason),
        };
        if finding.status == Status::Fail
            && witness
            && let Some(row) = nominal
        {
            finding.measurement = Some(test_bounds);
            finding.event = Some(match self.sense {
                Sense::ReverseVoltage => row.min.time,
                Sense::PeakCurrent | Sense::AbsoluteVoltage
                    if row.min.value.abs() > row.max.value.abs() =>
                {
                    row.min.time
                }
                _ => row.max.time,
            });
        }
        Ok(finding)
    }
}

fn pin_injection(
    prepared: &super::PreparedMode,
    nominal: &BTreeMap<Probe, Envelope>,
    mode: &Mode,
    reference_point: bool,
) -> Result<Vec<ElectricalFinding>, ProcessError> {
    let mut findings = Vec::new();
    for (contact, stamp) in &prepared.pins {
        let model = &prepared.pin_models[contact];
        for (section, clamp) in [
            (&stamp.upper_clamp, &model.upper_clamp),
            (&stamp.lower_clamp, &model.lower_clamp),
        ] {
            let (Some(section), super::ClampPath::Diode(diode)) = (section, clamp) else {
                continue;
            };
            let probe = Probe::Current(section.clone());
            let mut finding = ElectricalFinding::open(
                ElectricalClass::PinOvervoltage,
                format!("{}/{}", contact.board, contact.pin),
                &diode.injection_limit_a.measurement,
            );
            finding.mode = Some(mode.clone());
            finding.probe = Some(probe.clone());
            if let Some(row) = nominal.get(&probe) {
                // Защитный диод ориентирован по прямому току в раскрытой цепи.
                let peak = row.max.value.max(0.0);
                let limit = diode.injection_limit_a.range.as_ref();
                finding.measurement = Some([peak; 2]);
                finding.limit = limit.cloned();
                finding.unit = Some("A");
                finding.event = Some(row.max.time);
                match assess_current_envelope(
                    [peak; 2],
                    limit,
                    prepared.model_confirmed && prepared.window.is_none(),
                    reference_point,
                )? {
                    HazardCheck::Violation { reason } => {
                        finding.status = Status::Fail;
                        finding.reason = format!("ток защитного диода: {reason}");
                    }
                    HazardCheck::WithinLimit { scope } => {
                        finding.status = Status::Pass;
                        finding.reason = format!("ток защитного диода: {scope}");
                    }
                    HazardCheck::NotAnalyzed { .. } => {}
                }
            }
            findings.push(finding);
        }
    }
    Ok(findings)
}

/// Топологический порядок, среди свободных узлов первым идёт меньший идентификатор.
pub(super) fn order(
    connections: &BTreeMap<String, Connection>,
) -> Result<Vec<String>, ProcessError> {
    let mut pending = BTreeMap::new();
    let mut children: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
    let mut available = BTreeSet::new();
    for (id, joint) in connections {
        if !identifier(id) {
            return Err(invalid("пустое или недопустимое имя операции сборки"));
        }
        for parent in &joint.after {
            if !connections.contains_key(parent) {
                return Err(invalid(format!(
                    "«{id}»: неизвестная предшествующая операция «{parent}»"
                )));
            }
            children.entry(parent).or_default().push(id);
        }
        pending.insert(id.as_str(), joint.after.len());
        if joint.after.is_empty() {
            available.insert(id.as_str());
        }
    }

    let mut ordered = Vec::with_capacity(connections.len());
    while let Some(id) = available.pop_first() {
        ordered.push(id.into());
        for &child in children.get(id).into_iter().flatten() {
            let left = pending.get_mut(child).unwrap();
            *left -= 1;
            if *left == 0 {
                available.insert(child);
            }
        }
    }
    if ordered.len() != connections.len() {
        let blocked = pending
            .into_iter()
            .filter_map(|(id, count)| (count > 0).then_some(id))
            .collect::<Vec<_>>();
        return Err(invalid(format!(
            "цикл в порядке сборки: {}",
            blocked.join(", ")
        )));
    }
    Ok(ordered)
}

/// Дополнительные наблюдения сохраняются отдельно от прежних кодов электрического вердикта.
#[derive(Clone, Copy, Debug)]
pub enum DepthCheck<'a> {
    CapacitorStress(CapacitorStressInput<'a>, CapacitorRatings<'a>),
    FuseCoordination(FuseCoordinationInput<'a>, FuseCoordinationRatings<'a>),
    SwitchThermal(SwitchThermalInput<'a>, SwitchThermalRatings<'a>),
    DeepDischarge(DeepDischargeInput<'a>),
    SleepDrain(SleepDrainInput<'a>),
    CableFlex(CableFlexInput<'a>, CableFlexRatings<'a>),
}

#[derive(Clone, Debug)]
pub enum DepthEvidence {
    CapacitorStress(CapacitorStressEvidence),
    FuseCoordination(FuseCoordinationEvidence),
    SwitchThermal(SwitchThermalEvidence),
    DeepDischarge(DeepDischargeEvidence),
    SleepDrain(SleepDrainEvidence),
    CableFlex(CableFlexEvidence),
}

#[derive(Clone, Debug)]
pub struct DepthEvent {
    pub kind: DepthEventKind,
    pub at: Time,
    pub branch: Section,
    /// Источник подтверждает, что событие произошло на этой ветви в журнале опыта.
    pub source: Provenance,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DepthEventKind {
    WireOpened,
    FaultApplied,
    ModeEntered,
}

impl DepthEventKind {
    pub fn code(self) -> &'static str {
        match self {
            Self::WireOpened => "wire_opened",
            Self::FaultApplied => "fault_applied",
            Self::ModeEntered => "mode_entered",
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct DepthModeMark<'a> {
    /// Подтверждение привязки всех входов к этой установке, ветви, режиму и окну.
    pub applicability: Option<&'a Provenance>,
    pub event: Option<&'a DepthEvent>,
    pub passport: Option<&'a str>,
    /// Данные стенда ограничены стендовой областью независимо от вида их источника.
    pub reference_only: bool,
}

#[derive(Clone, Debug)]
pub struct DepthFinding {
    pub code: &'static str,
    pub scope: Scope,
    pub mode: Mode,
    pub branch: Section,
    pub window: Option<Interval>,
    pub event: Option<DepthEvent>,
    pub applicability: Option<Provenance>,
    pub passport: Option<String>,
    pub applicability_gap: Option<&'static str>,
    pub status: Status,
    pub reason: String,
    pub evidence: Option<DepthEvidence>,
}

impl CompiledSchematic {
    /// Разрешает физическую ветвь и возвращает каталожную позицию для ветви элемента.
    pub fn depth_part(&self, branch: &Section) -> Result<Option<&str>, ProcessError> {
        match branch {
            Section::Connection(id) if self.schematic.connections.contains_key(id) => Ok(None),
            Section::Element { component, element } => {
                if let Some(item) = self.schematic.components.get(component)
                    && self
                        .catalog
                        .part(&item.part)
                        .unwrap()
                        .elements
                        .contains_key(element)
                {
                    return Ok(Some(&item.part));
                }
                Err(invalid(format!("неизвестная ветвь {branch:?}")))
            }
            _ => Err(invalid(format!("неизвестная ветвь {branch:?}"))),
        }
    }

    /// Проверяет тип и адреса модели до использования её результатов в отчёте.
    pub fn assess_depth(
        &self,
        plan: &ModePlan,
        mark: DepthModeMark<'_>,
        check: DepthCheck<'_>,
    ) -> Result<DepthFinding, ProcessError> {
        self.validate_mode_plan(plan)?;
        let (scope, mode, branch, window) = match check {
            DepthCheck::CapacitorStress(i, _) => (i.scope, i.mode, i.branch, Some(i.window)),
            DepthCheck::FuseCoordination(i, _) => (i.scope, i.mode, i.fuse, Some(i.window)),
            DepthCheck::SwitchThermal(i, _) => (i.scope, i.mode, i.branch, Some(i.window)),
            DepthCheck::DeepDischarge(i) => (i.scope, i.mode, i.branch, Some(i.window)),
            DepthCheck::SleepDrain(i) => (
                i.scope,
                i.mode,
                i.branch,
                i.required.map(|r| Interval {
                    start: Time::ZERO,
                    end: r.duration,
                }),
            ),
            DepthCheck::CableFlex(i, _) => (i.scope, i.mode, i.branch, Some(i.window)),
        };
        if scope != self.schematic.scope {
            return Err(invalid("область расчёта должна совпадать с областью схемы"));
        }
        let mut reference_only = self.depth_reference_branch(branch)? || mark.reference_only;
        if mark.passport.is_some_and(|id| !identifier(id)) {
            return Err(invalid("нужно имя углублённого паспорта"));
        }
        if let Some(source) = mark.applicability {
            validate_source(source)?;
        }
        if let Mode::OpenWire(id) = mode {
            self.depth_part(&Section::Connection(id.clone()))?;
        }
        if let Some(event) = mark.event {
            reference_only |= self.depth_reference_branch(&event.branch)?;
            validate_source(&event.source)?;
            if !window.is_some_and(|window| (window.start..window.end).contains(&event.at)) {
                return Err(invalid("отметка события вне окна проверки"));
            }
            if let Mode::OpenWire(id) = mode
                && (event.kind != DepthEventKind::WireOpened
                    || event.branch != Section::Connection(id.clone()))
            {
                return Err(invalid("событие не подтверждает обрыв ветви режима"));
            }
        }

        let (code, mut status, mut reason, evidence) = match check {
            DepthCheck::CapacitorStress(input, ratings) => {
                let (kind, terminals) = self.depth_element::<2>(branch)?;
                if kind != PrimitiveKind::Capacitor || terminals != *input.terminals {
                    return Err(invalid(
                        "ветвь или полярность выводов конденсатора не совпадают со схемой",
                    ));
                }
                depth_evidence(
                    CAPACITOR_STRESS_CODE,
                    assess_capacitor_stress(&input, &ratings)?,
                    DepthEvidence::CapacitorStress,
                )
            }
            DepthCheck::FuseCoordination(input, ratings) => {
                reference_only |= self.depth_reference_branch(input.wire)?;
                reference_only |= self.depth_reference_branch(input.switch)?;
                depth_evidence(
                    FUSE_COORDINATION_CODE,
                    assess_fuse_coordination(&input, &ratings)?,
                    DepthEvidence::FuseCoordination,
                )
            }
            DepthCheck::SwitchThermal(input, ratings) => {
                let (kind, terminals) = self.depth_element::<3>(branch)?;
                if !matches!(
                    kind,
                    PrimitiveKind::Nmos | PrimitiveKind::Pmos | PrimitiveKind::Switch
                ) || terminals != *input.terminals
                {
                    return Err(invalid(
                        "ветвь или выводы силового ключа не совпадают со схемой",
                    ));
                }
                depth_evidence(
                    SWITCH_THERMAL_CODE,
                    assess_switch_thermal(&input, &ratings)?,
                    DepthEvidence::SwitchThermal,
                )
            }
            DepthCheck::DeepDischarge(input) => depth_evidence(
                DEEP_DISCHARGE_CODE,
                assess_deep_discharge(&input)?,
                DepthEvidence::DeepDischarge,
            ),
            DepthCheck::SleepDrain(input) => {
                if let Some(profile) = input.profile {
                    for load in profile
                        .states
                        .iter()
                        .filter_map(|state| state.loads)
                        .flatten()
                    {
                        reference_only |= self.depth_reference_branch(load.branch)?;
                    }
                }
                depth_evidence(
                    SLEEP_DRAIN_CODE,
                    assess_sleep_drain(&input)?,
                    DepthEvidence::SleepDrain,
                )
            }
            DepthCheck::CableFlex(input, ratings) => {
                let mut finding = assess_cable_flex(&input, &ratings)?;
                if finding.status == Status::Pass
                    && (input.duty.is_some() || scope == Scope::Assembly)
                    && finding.evidence.fatigue.status == Status::NotAnalyzed
                {
                    finding.status = Status::NotAnalyzed;
                    finding.reason = "не подтверждён ресурс для заданного цикла изгиба".into();
                }
                depth_evidence(CABLE_FLEX_CODE, finding, DepthEvidence::CableFlex)
            }
        };
        let reference_gap = reference_only && scope == Scope::Assembly;
        let event_required =
            matches!(mode, Mode::OpenWire(_)) || matches!(check, DepthCheck::FuseCoordination(..));
        let case = plan.cases.get(mode);
        let applicability_gap = if reference_gap {
            Some("данные эталонного стенда требуют подтверждения для физической сборки")
        } else if !mark
            .applicability
            .is_some_and(|source| supports_source(scope, source))
        {
            Some("не подтверждена применимость данных к ветви и режиму")
        } else if !case.is_some_and(|case| supports_source(scope, &case.source)) {
            Some("нет подтверждённой постановки для указанного режима")
        } else if window
            .zip(case.and_then(|case| case.window.as_ref()))
            .is_some_and(|(window, mode_window)| window.end > mode_window.end)
        {
            Some("окно проверки выходит за горизонт указанного режима")
        } else if event_required
            && !mark.event.is_some_and(|event| {
                supports_source(scope, &event.source)
                    && match check {
                        DepthCheck::FuseCoordination(input, _) => {
                            event.at == input.window.start
                                && matches!(
                                    event.kind,
                                    DepthEventKind::WireOpened | DepthEventKind::FaultApplied
                                )
                                && (matches!(mode, Mode::OpenWire(_))
                                    || [input.fuse, input.wire, input.switch].contains(&&event.branch))
                        }
                        _ => true,
                    }
            })
        {
            Some("не подтверждена инъекция отказа на требуемой ветви и отметке")
        } else if status == Status::Pass
            && let (DepthCheck::CapacitorStress(input, _), DepthEvidence::CapacitorStress(result)) =
                (check, &evidence)
            && self.capacitor_mode_conflicts(plan, &input, result)?
        {
            Some("ряд напряжения не согласуется с запасом до предела в указанном режиме")
        } else {
            None
        };
        if let Some(gap) = applicability_gap {
            status = Status::NotAnalyzed;
            if reference_gap {
                reason = gap.into();
            } else {
                reason = format!("{gap}. {reason}");
            }
        }
        Ok(DepthFinding {
            code,
            scope,
            mode: mode.clone(),
            branch: branch.clone(),
            window,
            event: mark.event.cloned(),
            applicability: mark.applicability.cloned(),
            passport: mark.passport.map(str::to_owned),
            applicability_gap,
            status,
            reason,
            evidence: (!reference_gap).then_some(evidence),
        })
    }

    pub fn depth_scope(&self) -> Scope {
        self.schematic.scope
    }

    fn capacitor_mode_conflicts(
        &self,
        plan: &ModePlan,
        input: &CapacitorStressInput<'_>,
        result: &CapacitorStressEvidence,
    ) -> Result<bool, ProcessError> {
        let Analysis::Analyzed { value: voltage, .. } = &result.voltage_v else {
            return Ok(false);
        };
        let [positive, negative] = input.terminals;
        let probe = Probe::Voltage(positive.clone(), negative.clone());
        let mut case = plan.cases[input.mode].clone();
        case.probes.insert(probe.clone());
        let mode_plan = ModePlan {
            source_disconnects: plan.source_disconnects.clone(),
            cases: BTreeMap::from([(input.mode.clone(), case)]),
        };
        let Analysis::Analyzed { value: prepared, .. } = self.prepare_mode(&mode_plan, input.mode)?
        else {
            return Ok(false);
        };
        let Analysis::Analyzed { value: series, .. } = prepared.solve() else {
            return Ok(false);
        };
        // Сверяем только фактические отсчёты того же режима. Противоречие
        // с отдельным расчётом блокирует допуск и требует выяснить источник.
        Ok(series.values.get(&probe).is_some_and(|values| {
            series.time.iter().zip(values).any(|(&at, &value)| {
                let observed = match input.voltage_sense {
                    CapacitorVoltageSense::Forward => value,
                    CapacitorVoltageSense::Absolute => value.abs(),
                };
                (input.window.start..=input.window.end).contains(&at)
                    && observed >= voltage.limit.min
            })
        }))
    }

    fn depth_reference_branch(&self, section: &Section) -> Result<bool, ProcessError> {
        let part = self.depth_part(section)?;
        let reference = match section {
            Section::Element { .. } => self.catalog.part(part.unwrap()).unwrap().reference_only,
            Section::Connection(id) => self.schematic.connections[id].ends.iter().any(|end| {
                let component = &self.schematic.components[&end.component];
                self.catalog.part(&component.part).unwrap().reference_only
            }),
        };
        Ok(reference)
    }

    fn depth_element<const N: usize>(
        &self,
        branch: &Section,
    ) -> Result<(PrimitiveKind, [Endpoint; N]), ProcessError> {
        let Section::Element { component, element } = branch else {
            return Err(invalid("проверка требует ветвь элемента"));
        };
        let part = self.depth_part(branch)?.unwrap();
        let model = &self.catalog.part(part).unwrap().elements[element];
        let indices: &[usize] = match (N, model.kind) {
            (2, PrimitiveKind::Capacitor) => &[0, 1],
            (3, PrimitiveKind::Nmos | PrimitiveKind::Pmos) => &[0, 2, 1],
            (3, PrimitiveKind::Switch) if model.nodes[1] == model.nodes[3] => &[0, 1, 2],
            _ => return Err(invalid("для проверки нужны совместимые выводы элемента")),
        };
        Ok((
            model.kind,
            std::array::from_fn(|i| Endpoint {
                component: component.clone(),
                pin: model.nodes[indices[i]].clone(),
            }),
        ))
    }
}

fn depth_evidence<E>(
    code: &'static str,
    finding: Finding<E>,
    wrap: impl FnOnce(E) -> DepthEvidence,
) -> (&'static str, Status, String, DepthEvidence) {
    (code, finding.status, finding.reason, wrap(finding.evidence))
}
