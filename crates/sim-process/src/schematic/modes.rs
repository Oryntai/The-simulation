//! Режимы §3.5. Опыт, дискретные события и случайная серия имеют разные свидетельства.

use std::collections::{BTreeMap, BTreeSet};

use sim_circuit::netlist::{ElementKind, Waveform};
use sim_circuit::{Circuit, SimOptions, TranOptions, Transient, operating_point_with};
use sim_signal::{Rng, Time};
use sim_world::Provenance;

use crate::{Analysis, ProcessError, SourcedRange};

use super::{
    BindingTarget, BoardContact, CompiledSchematic, Endpoint, ExpandedNetlist, Parameter, PartKind,
    PinPhase, PinSetup, PinStamp, Section, identifier, invalid, open, supports_source,
    validate_source,
};

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Mode {
    AllOff,
    UsbOnly,
    MainOnly,
    Both,
    Reset,
    Boot(Time),
    Normal,
    Start,
    Stop,
    Stall,
    LoadOff,
    OpenWire(String),
}

impl Mode {
    pub fn number(&self) -> u8 {
        // Нумерация задана §3.5. Подопыты сохраняют общий номер режима.
        match self {
            Self::AllOff => 1,
            Self::UsbOnly => 2,
            Self::MainOnly => 3,
            Self::Both => 4,
            Self::Reset | Self::Boot(_) => 5,
            Self::Normal => 6,
            Self::Start | Self::Stop | Self::Stall => 7,
            Self::LoadOff => 8,
            Self::OpenWire(_) => 9,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Probe {
    Voltage(Endpoint, Endpoint),
    Current(Section),
    Power(Section),
}

#[derive(Clone, Debug, PartialEq)]
pub struct SourceTrace {
    /// Первый отсчёт в нуле задаёт также исходную рабочую точку.
    pub points: Vec<(Time, f64)>,
    pub source: Provenance,
}

#[derive(Clone, Debug, PartialEq)]
pub enum InitialState {
    OperatingPoint,
    /// Начальные V(C) и I(L) по ориентированным ветвям. Нули задаются явно.
    Reactive(BTreeMap<Section, f64>),
}

#[derive(Clone, Debug, PartialEq)]
pub struct Window {
    pub step: Time,
    pub end: Time,
    pub initial: InitialState,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Sampling {
    pub seed: u64,
    pub count: usize,
    /// Независимые равномерные параметры сопротивлений соединений из указанной модели популяции.
    pub connection_ohm: BTreeMap<String, SourcedRange>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ModeCase {
    pub source: Provenance,
    pub window: Option<Window>,
    pub sources: BTreeMap<Section, SourceTrace>,
    pub pins: BTreeMap<BoardContact, PinSetup>,
    pub probes: BTreeSet<Probe>,
    pub sampling: Option<Sampling>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ModePlan {
    /// Размыкаемый физический контакт входа. Оставшиеся ветви источника сохраняются.
    pub source_disconnects: BTreeMap<String, String>,
    pub cases: BTreeMap<Mode, ModeCase>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ElectricalSeries {
    pub time: Vec<Time>,
    pub values: BTreeMap<Probe, Vec<f64>>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Extremum {
    pub time: Time,
    pub value: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Envelope {
    pub min: Extremum,
    pub max: Extremum,
}

impl ElectricalSeries {
    pub fn envelopes(&self) -> BTreeMap<Probe, Envelope> {
        self.values
            .iter()
            .filter_map(|(probe, trace)| {
                let points = || {
                    self.time
                        .iter()
                        .copied()
                        .zip(trace.iter().copied())
                        .map(|(time, value)| Extremum { time, value })
                };
                Some((
                    probe.clone(),
                    Envelope {
                        min: points().min_by(|a, b| a.value.total_cmp(&b.value))?,
                        max: points().max_by(|a, b| a.value.total_cmp(&b.value))?,
                    },
                ))
            })
            .collect()
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct SampleRun {
    pub connection_ohm: BTreeMap<String, f64>,
    pub trace: Analysis<ElectricalSeries>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ModeReport {
    pub mode: Mode,
    pub nominal: Analysis<ElectricalSeries>,
    pub envelope: Analysis<BTreeMap<Probe, Envelope>>,
    pub window: Option<Window>,
    pub source_fronts: BTreeMap<Section, SourceTrace>,
    pub opened: BTreeSet<Section>,
    pub seed: Option<u64>,
    pub samples: Vec<SampleRun>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum ModeCoverage {
    Inapplicable { number: u8, reason: String },
    Case(ModeReport),
}

#[derive(Clone, Debug)]
pub struct PreparedMode {
    pub(super) net: ExpandedNetlist,
    pub(super) window: Option<Window>,
    source_fronts: BTreeMap<Section, SourceTrace>,
    pub(super) probes: BTreeSet<Probe>,
    pub(super) opened: BTreeSet<Section>,
    pub(super) events: BTreeSet<Time>,
    pub(super) gaps: Vec<String>,
    pub(super) model_confirmed: bool,
    pub(super) linear_window: bool,
    pub(super) dc_bounds: Option<BTreeMap<Probe, Envelope>>,
    pub pins: BTreeMap<BoardContact, PinStamp>,
    pub(super) pin_models: BTreeMap<BoardContact, super::PinModel>,
}

fn main_source(kind: PartKind) -> bool {
    matches!(kind, PartKind::Battery | PartKind::BenchSupply)
}

impl CompiledSchematic {
    pub fn run_modes(&self, plan: &ModePlan) -> Result<Vec<ModeCoverage>, ProcessError> {
        self.validate_mode_plan(plan)?;
        let mut reports = Vec::new();
        for (number, modes, reason) in self.applicable_modes(plan) {
            if modes.is_empty() {
                reports.push(ModeCoverage::Inapplicable {
                    number,
                    reason: reason.into(),
                });
            }
            for mode in modes {
                let requested = plan.cases.get(&mode);
                let prepared = self.prepare_validated_mode(plan, &mode)?;
                let mut report = match prepared {
                    Analysis::NotAnalyzed { reason } => ModeReport {
                        mode: mode.clone(),
                        nominal: open(reason.clone()),
                        envelope: open(reason),
                        window: requested.and_then(|case| case.window.clone()),
                        source_fronts: requested
                            .map_or_else(BTreeMap::new, |case| case.sources.clone()),
                        opened: BTreeSet::new(),
                        seed: None,
                        samples: Vec::new(),
                    },
                    Analysis::Analyzed { value, .. } => {
                        let mut report = value.report(mode.clone());
                        if let Some(sampling) = requested.and_then(|case| case.sampling.as_ref()) {
                            report.samples = value.sample(&mode, sampling);
                        }
                        report
                    }
                };
                report.seed = requested
                    .and_then(|case| case.sampling.as_ref())
                    .map(|sampling| sampling.seed);
                reports.push(ModeCoverage::Case(report));
            }
        }

        Ok(reports)
    }

    fn applicable_modes(&self, plan: &ModePlan) -> Vec<(u8, Vec<Mode>, &'static str)> {
        let has = |select: fn(PartKind) -> bool| {
            self.schematic
                .components
                .values()
                .any(|c| select(self.catalog.part(&c.part).unwrap().kind))
        };
        let usb = has(|kind| kind == PartKind::UsbSource);
        let main = has(main_source);
        let board = self.schematic.bindings.iter().any(|b| {
            matches!(
                b.target,
                BindingTarget::BoardPower { .. } | BindingTarget::BoardPin { .. }
            )
        });
        let drive = self
            .schematic
            .bindings
            .iter()
            .any(|b| matches!(b.target, BindingTarget::DrivePower { .. }))
            || has(|kind| {
                matches!(
                    kind,
                    PartKind::SwitchModule | PartKind::Nmos | PartKind::Pmos
                )
            })
            || self.schematic.components.values().any(|c| {
                self.catalog
                    .part(&c.part)
                    .unwrap()
                    .elements
                    .values()
                        .any(|e| matches!(e.kind,
                            super::PrimitiveKind::Inductor | super::PrimitiveKind::Switch
                            | super::PrimitiveKind::Nmos | super::PrimitiveKind::Pmos))
            });
        let load = drive
            || self
                .schematic
                .bindings
                .iter()
                .any(|b| matches!(b.target, BindingTarget::SensorPower { .. }));
        let power_nets: BTreeSet<_> = self
            .pins()
            .filter(|end| {
                matches!(
                    self.pin(end).role,
                    super::PinRole::Power | super::PinRole::Return
                )
            })
            .map(|end| &self.schematic.components[&end.component].nets[&end.pin])
            .collect();
        let wires = self
            .schematic
            .connections
            .iter()
            .filter(|(id, joint)| {
                !self.mismatches.contains(*id)
                    && power_nets.contains(
                        &self.schematic.components[&joint.ends[0].component].nets
                            [&joint.ends[0].pin],
                    )
            })
            .map(|(id, _)| Mode::OpenWire(id.clone()))
            .collect();
        let mut boot: BTreeSet<_> = plan
            .cases
            .keys()
            .filter_map(|m| match m {
                Mode::Boot(t) => Some(*t),
                _ => None,
            })
            .collect();
        boot.insert(Time::ZERO);
        // Включены отдельные начала каждой фазы на всех заданных выводах.
        for setup in plan.cases.values().flat_map(|case| case.pins.values()) {
            let mut at = Time::ZERO;
            for stage in &setup.profile.boot {
                boot.insert(at);
                let Some(next) = stage
                    .duration_s
                    .range
                    .as_ref()
                    .and_then(|r| Time::from_secs(r.nominal))
                    .and_then(|d| at.0.checked_add(d.0))
                else {
                    break;
                };
                at = Time(next);
            }
        }
        vec![
            (1, vec![Mode::AllOff], ""),
            (
                2,
                usb.then_some(Mode::UsbOnly).into_iter().collect(),
                "нет источника USB",
            ),
            (
                3,
                main.then_some(Mode::MainOnly).into_iter().collect(),
                "нет основного источника из каталога",
            ),
            (
                4,
                (usb && main).then_some(Mode::Both).into_iter().collect(),
                "нет пары источников разных ролей",
            ),
            (
                5,
                if board {
                    [Mode::Reset]
                        .into_iter()
                        .chain(boot.into_iter().map(Mode::Boot))
                        .collect()
                } else {
                    Vec::new()
                },
                "нет привязок платы",
            ),
            (6, vec![Mode::Normal], ""),
            (
                7,
                if drive {
                    vec![Mode::Start, Mode::Stop, Mode::Stall]
                } else {
                    Vec::new()
                },
                "нет привода или управляемой силовой ветви",
            ),
            (
                8,
                load.then_some(Mode::LoadOff).into_iter().collect(),
                "нет адресуемой отключаемой нагрузки",
            ),
            (
                9,
                wires,
                "нет подключённых физических соединений цепи питания или возврата",
            ),
        ]
    }

    pub fn validate_mode_plan(&self, plan: &ModePlan) -> Result<(), ProcessError> {
        let valid: BTreeSet<_> = self
            .applicable_modes(plan)
            .into_iter()
            .flat_map(|(_, m, _)| m)
            .collect();
        for (id, wire) in &plan.source_disconnects {
            let Some(component) = self.schematic.components.get(id) else {
                return Err(invalid(format!("неизвестный источник «{id}»")));
            };
            let kind = self.catalog.part(&component.part).unwrap().kind;
            if !(main_source(kind) || kind == PartKind::UsbSource)
                || self.mismatches.contains(wire)
                || !self.schematic.connections.get(wire).is_some_and(|joint| {
                    joint.ends.iter().any(|end| {
                        &end.component == id && self.pin(end).role == super::PinRole::Power
                    })
                })
            {
                return Err(invalid(format!(
                    "«{wire}» не является подключённым контактом питания источника «{id}»"
                )));
            }
        }
        for (mode, case) in &plan.cases {
            if !valid.contains(mode) || !identifier(case_source_name(&case.source)) {
                return Err(invalid(format!(
                    "недопустимый режим или источник опыта {mode:?}"
                )));
            }
            if case.probes.is_empty() {
                return Err(invalid("опыт требует наблюдаемые величины"));
            }
            for probe in &case.probes {
                match probe {
                    Probe::Voltage(a, b) if a != b => {
                        self.index_of_pin(a)?;
                        self.index_of_pin(b)?;
                    }
                    Probe::Current(section) | Probe::Power(section)
                        if self.has_section(section) => {}
                    _ => return Err(invalid(format!("неизвестная ветвь наблюдения {probe:?}"))),
                }
            }
            if let Some(window) = &case.window {
                // До 2^52 нс шаг f64 после перевода в секунды остаётся меньше наносекунды.
                if window.step == Time::ZERO
                    || window.end < window.step
                    || window.end.0 > (1_u64 << (f64::MANTISSA_DIGITS - 1))
                {
                    return Err(invalid("недопустимая сетка электрического опыта"));
                }
                if let InitialState::Reactive(values) = &window.initial {
                    for (section, value) in values {
                        if !value.is_finite()
                            || !self.primitive(section).is_some_and(|p| {
                                matches!(
                                    p.kind,
                                    super::PrimitiveKind::Capacitor
                                        | super::PrimitiveKind::Inductor
                                )
                            })
                        {
                            return Err(invalid(format!(
                                "некорректное начальное условие {section:?}"
                            )));
                        }
                    }
                }
            }
            for (section, trace) in &case.sources {
                validate_source(&trace.source)?;
                if !self.primitive(section).is_some_and(|p| {
                    matches!(
                        p.kind,
                        super::PrimitiveKind::Voltage | super::PrimitiveKind::Current
                    )
                }) || trace.points.first().is_none_or(|(t, _)| *t != Time::ZERO)
                    || trace.points.iter().any(|(_, v)| !v.is_finite())
                    || trace
                        .points
                        .windows(2)
                        .any(|p| p[0].0.as_secs() >= p[1].0.as_secs())
                    || case
                        .window
                        .as_ref()
                        .is_none_or(|w| trace.points.last().is_none_or(|(t, _)| *t > w.end))
                {
                    return Err(invalid(format!("некорректная PWL-команда для {section:?}")));
                }
            }
            for (contact, setup) in &case.pins {
                self.validate_pin_setup(contact, setup)?;
            }
            if let Some(sampling) = &case.sampling {
                if !matches!(mode.number(), 6 | 7)
                    || sampling.count == 0
                    || sampling.connection_ohm.is_empty()
                {
                    return Err(invalid(
                        "выборка разрешена только в режимах 6 и 7 и требует популяцию",
                    ));
                }
                for (wire, range) in &sampling.connection_ohm {
                    range.validate()?;
                    let declared = self
                        .schematic
                        .connections
                        .get(wire)
                        .and_then(|joint| joint.resistance_ohm.range.as_ref());
                    if declared.is_none_or(|r| range.min < r.min || range.max > r.max)
                        || range.min <= 0.0
                        || !range.min.recip().is_finite()
                        || self.mismatches.contains(wire)
                    {
                        return Err(invalid(format!(
                            "«{wire}»: выборка вне каталожного диапазона сопротивления"
                        )));
                    }
                }
            }
        }
        Ok(())
    }

    fn primitive(&self, section: &Section) -> Option<&super::Primitive> {
        let Section::Element { component, element } = section else {
            return None;
        };
        self.schematic
            .components
            .get(component)
            .and_then(|c| self.catalog.part(&c.part))
            .and_then(|p| p.elements.get(element))
    }

    fn has_section(&self, section: &Section) -> bool {
        match section {
            Section::Connection(id) => self.schematic.connections.contains_key(id),
            _ => self.primitive(section).is_some(),
        }
    }

    pub fn prepare_mode(
        &self,
        plan: &ModePlan,
        mode: &Mode,
    ) -> Result<Analysis<PreparedMode>, ProcessError> {
        self.validate_mode_plan(plan)?;
        self.prepare_validated_mode(plan, mode)
    }

    fn prepare_validated_mode(
        &self,
        plan: &ModePlan,
        mode: &Mode,
    ) -> Result<Analysis<PreparedMode>, ProcessError> {
        let from_reset;
        let case = if let Some(case) = plan.cases.get(mode) {
            case
        } else if matches!(mode, Mode::Boot(_))
            && let Some(reset) = plan.cases.get(&Mode::Reset)
        {
            // Точки загрузки используют ту же статическую схему питания и те же выводы.
            // Динамический опыт задаётся отдельно, поскольку его начальные условия и фронты свои.
            from_reset = ModeCase {
                window: None,
                sources: BTreeMap::new(),
                sampling: None,
                ..reset.clone()
            };
            &from_reset
        } else {
            return Ok(open(
                "нужен профиль опыта с исходным состоянием, сеткой и наблюдениями",
            ));
        };
        let mut opened: BTreeSet<_> = self
            .mismatches
            .iter()
            .cloned()
            .map(Section::Connection)
            .collect();
        for (id, comp) in &self.schematic.components {
            let kind = self.catalog.part(&comp.part).unwrap().kind;
            let off = (kind == PartKind::UsbSource
                && matches!(mode, Mode::AllOff | Mode::MainOnly))
                || (main_source(kind) && matches!(mode, Mode::AllOff | Mode::UsbOnly));
            if off {
                let Some(wire) = plan.source_disconnects.get(id) else {
                    return Ok(open(format!(
                        "нужно описать размыкание источника «{id}» и оставшиеся пути обратного тока"
                    )));
                };
                let joint = &self.schematic.connections[wire];
                let terminal = joint
                    .ends
                    .iter()
                    .find(|end| &end.component == id && self.pin(end).role == super::PinRole::Power)
                    .unwrap();
                let paths = self
                    .schematic
                    .connections
                    .iter()
                    .filter(|(other, link)| {
                        !self.mismatches.contains(*other) && link.ends.contains(terminal)
                    })
                    .count();
                let ports = self
                    .pins()
                    .filter(|end| {
                        &end.component == id && self.pin(end).role == super::PinRole::Power
                    })
                    .count();
                if paths != 1 || ports != 1 {
                    return Ok(open(format!(
                        "«{id}»: отключение одного контакта не изолирует все выходы источника"
                    )));
                }
                opened.insert(Section::Connection(wire.clone()));
            }
        }
        if let Mode::OpenWire(wire) = mode {
            opened.insert(Section::Connection(wire.clone()));
        }
        let mut net = match self.netlist() {
            Analysis::Analyzed { value, .. } => value,
            Analysis::NotAnalyzed { reason } => return Ok(open(reason)),
        };
        net.netlist
            .elements
            .retain(|el| !opened.contains(&net.origins[&el.name]));
        net.origins.retain(|_, section| !opened.contains(section));
        let mut gaps = std::mem::take(&mut net.netlist.warnings);
        let mut pins = BTreeMap::new();
        for binding in &self.schematic.bindings {
            let BindingTarget::BoardPin { board, pin, .. } = &binding.target else {
                continue;
            };
            let contact = BoardContact {
                board: board.clone(),
                pin: pin.clone(),
            };
            let Some(mut setup) = case.pins.get(&contact).cloned() else {
                gaps.push(format!(
                    "нет электрического состояния контакта {board}/{pin}"
                ));
                continue;
            };
            setup.phase = match mode {
                Mode::AllOff => PinPhase::Unpowered,
                Mode::Reset => PinPhase::Reset,
                Mode::Boot(elapsed) => PinPhase::Boot { elapsed: *elapsed },
                _ => setup.phase,
            };
            match self.stamp_pin(&mut net, &contact, &setup)? {
                Analysis::Analyzed { value, .. } => {
                    pins.insert(contact, value);
                }
                Analysis::NotAnalyzed { reason } => gaps.push(format!("{board}/{pin}: {reason}")),
            }
        }
        gaps.append(&mut net.netlist.warnings);
        if !supports_source(self.schematic.scope, &case.source) {
            gaps.push("постановка не подтверждена для этой сборки".into());
        }
        let mut events = BTreeSet::new();
        for el in &mut net.netlist.elements {
            let section = &net.origins[&el.name];
            if let Some(trace) = case.sources.get(section) {
                if let ElementKind::VoltageSource(source) | ElementKind::CurrentSource(source) =
                    &mut el.kind
                {
                    source.dc = Some(trace.points[0].1);
                    source.wave = Some(Waveform::Pwl(
                        trace
                            .points
                            .iter()
                            .map(|(at, v)| (at.as_secs(), *v))
                            .collect(),
                    ));
                    events.extend(trace.points.iter().skip(1).map(|(at, _)| *at));
                    if !supports_source(self.schematic.scope, &trace.source) {
                        gaps.push(format!("не подтверждён фронт {section:?}"));
                    }
                }
            }
            if let Some(Window {
                initial: InitialState::Reactive(initial),
                ..
            }) = &case.window
            {
                match &mut el.kind {
                    ElementKind::Capacitor { ic, .. } | ElementKind::Inductor { ic, .. } => {
                        *ic = initial.get(section).copied();
                        if ic.is_none() {
                            return Ok(open(format!("не задано исходное состояние {section:?}")));
                        }
                    }
                    _ => {}
                }
            }
        }

        let reactive = net.netlist.elements.iter().any(|e| {
            matches!(
                e.kind,
                ElementKind::Capacitor { .. } | ElementKind::Inductor { .. }
            )
        });
        if matches!(mode, Mode::AllOff)
            && reactive
            && !matches!(
                &case.window,
                Some(Window {
                    initial: InitialState::Reactive(_),
                    ..
                })
            )
        {
            gaps.push(
                "для остаточного заряда или тока нужно предсостояние C/L перед отключением".into(),
            );
        }
        let linear_window = net.netlist.elements.iter().all(|e| {
            matches!(
                e.kind,
                ElementKind::Resistor { .. }
                    | ElementKind::VoltageSource(_)
                    | ElementKind::CurrentSource(_)
                    | ElementKind::Vcvs { .. }
                    | ElementKind::Vccs { .. }
            )
        });
        let model_confirmed = gaps.is_empty();
        match mode {
            Mode::Reset | Mode::Boot(_) => {
                gaps.push("для непрерывного перехода нужен фронт смены состояния контактов".into())
            }
            Mode::Normal if case.window.is_none() => {
                gaps.push("нет объявленного штатного интервала".into())
            }
            Mode::OpenWire(_) if !linear_window => gaps.push(
                "физическое соединение разомкнуто до старта; фронт обрыва не наблюдался".into(),
            ),
            Mode::Start | Mode::Stop | Mode::Stall | Mode::LoadOff
                if !case
                    .sources
                    .values()
                    .any(|trace| trace.points.windows(2).any(|p| p[0].1 != p[1].1)) =>
            {
                gaps.push("нет подтверждённого изменения нагрузки или управляющего фронта".into())
            }
            Mode::Start | Mode::Stop | Mode::Stall | Mode::LoadOff => gaps.push(
                "форма команды известна; требуется подтвердить её связь с физическим событием нагрузки".into(),
            ),
            _ => {}
        }
        if case.window.is_some()
            && (!linear_window
                || case
                    .probes
                    .iter()
                    .any(|probe| matches!(probe, Probe::Power(_))))
        {
            gaps.push("для непрерывной огибающей нужен контроль пиков между точками и сходимость на более мелкой сетке".into());
        }
        let mut probes: BTreeSet<_> = case
            .probes
            .iter()
            .cloned()
            .chain(pins.values().flat_map(|pin| {
                let [pad, vdd, vss] = &pin.ends;
                [
                    Probe::Voltage(pad.clone(), vss.clone()),
                    Probe::Voltage(vdd.clone(), vss.clone()),
                ]
                .into_iter()
                .chain(
                    pin.output
                        .iter()
                        .chain(&pin.upper_clamp)
                        .chain(&pin.lower_clamp)
                        .chain(&pin.passive)
                        .cloned()
                        .map(Probe::Current),
                )
            }))
            .collect();
        let islands = match ground_islands(&mut net, case.window.is_some()) {
            Ok(value) => value,
            Err(reason) => return Ok(open(reason)),
        };
        probes.retain(|probe| {
            if let Probe::Voltage(a, b) = probe
                && islands
                    .get(a)
                    .zip(islands.get(b))
                    .is_none_or(|(a, b)| a != b)
            {
                gaps.push(format!(
                    "не определена разность между {a} и {b}: разные острова или плавающий вывод"
                ));
                return false;
            }
            true
        });
        if probes.is_empty() {
            return Ok(open(gaps.join("; ")));
        }
        let dc_bounds = if case.window.is_none()
            && matches!(
                mode,
                Mode::AllOff | Mode::UsbOnly | Mode::MainOnly | Mode::Both | Mode::OpenWire(_)
            )
            && supports_source(self.schematic.scope, &case.source)
            && !self
                .schematic
                .bindings
                .iter()
                .any(|b| matches!(b.target, BindingTarget::BoardPin { .. }))
            && self
                .schematic
                .components
                .values()
                .all(|c| self.catalog.part(&c.part).unwrap().model_gap.is_none())
            && probes.len() == case.probes.len()
        {
            self.series_dc_bounds(&net, &probes, &opened)
        } else {
            None
        };
        Ok(Analysis::Analyzed {
            value: PreparedMode {
                net,
                window: case.window.clone(),
                source_fronts: case.sources.clone(),
                probes,
                opened,
                events,
                gaps,
                model_confirmed,
                linear_window,
                dc_bounds,
                pins,
                pin_models: case
                    .pins
                    .iter()
                    .map(|(key, setup)| (key.clone(), setup.profile.model.clone()))
                    .collect(),
            },
            scope: "один электрический опыт".into(),
        })
    }
}

type Bounds = [f64; 2];

impl CompiledSchematic {
    /// Для одного последовательного контура ток монотонен по сумме R.
    /// Доля напряжения возрастает по сопротивлениям выбранного пути и убывает
    /// по остальным. Другие топологии здесь остаются за пределами доказательства.
    fn series_dc_bounds(
        &self,
        net: &ExpandedNetlist,
        probes: &BTreeSet<Probe>,
        opened: &BTreeSet<Section>,
    ) -> Option<BTreeMap<Probe, Envelope>> {
        let bounds = |parameter: &Parameter| {
            let r = parameter.range.as_ref()?;
            (r.can_prove() || self.schematic.scope.exact(r)).then_some([r.min, r.max])
        };
        let mut supply = None;
        let mut neighbors: BTreeMap<&str, Vec<(&str, &Section, Bounds)>> = BTreeMap::new();
        for branch in &net.netlist.elements {
            let section = &net.origins[&branch.name];
            let (a, b) = (branch.nodes[0].as_str(), branch.nodes[1].as_str());
            match &branch.kind {
                ElementKind::Resistor { .. } => {
                    let p = match section {
                        Section::Connection(id) => {
                            &self.schematic.connections.get(id)?.resistance_ohm
                        }
                        _ => self.primitive(section)?.parameters.get("ohm")?,
                    };
                    let r = bounds(p)?;
                    neighbors.entry(a).or_default().push((b, section, r));
                    neighbors.entry(b).or_default().push((a, section, r));
                }
                ElementKind::VoltageSource(source) if source.wave.is_none() && supply.is_none() => {
                    supply = Some((
                        a,
                        b,
                        section,
                        bounds(self.primitive(section)?.parameters.get("volt")?)?,
                    ));
                }
                _ => return None,
            }
        }
        let (plus, minus, supply, emf) = supply?;
        if neighbors.get(plus)?.len() != 1
            || neighbors.get(minus)?.len() != 1
            || neighbors.values().map(Vec::len).sum::<usize>() != 2 * (neighbors.len() - 1)
        {
            return None;
        }
        let mut path = Vec::new();
        let mut order = BTreeMap::from([(plus, 0)]);
        let mut from = None;
        let mut at = plus;
        while at != minus {
            let mut candidates = neighbors
                .get(at)?
                .iter()
                .filter(|(next, _, _)| Some(*next) != from);
            let &(next, section, r) = candidates.next()?;
            if candidates.next().is_some() || order.insert(next, path.len() + 1).is_some() {
                return None;
            }
            let branch = net
                .netlist
                .elements
                .iter()
                .find(|e| &net.origins[&e.name] == section)?;
            path.push((section, r, branch.nodes[0] == at));
            from = Some(at);
            at = next;
        }
        if order.len() != neighbors.len() {
            return None;
        }
        let sum = |slice: &[(&Section, Bounds, bool)]| {
            slice
                .iter()
                .fold([0.0, 0.0], |sum, (_, r, _)| sum_bounds(sum, *r))
        };
        let total = sum(&path);
        if total[0] <= 0.0 || !total[1].is_finite() {
            return None;
        }
        let amps = product(
            emf,
            [total[1].recip().next_down(), total[0].recip().next_up()],
        );
        let signed = |[lo, hi]: Bounds, forward| if forward { [lo, hi] } else { [-hi, -lo] };
        probes
            .iter()
            .map(|probe| {
                let [lo, hi] = match probe {
                    Probe::Voltage(a, b) => {
                        let a = *order.get(net.nodes.get(a)?.as_str())?;
                        let b = *order.get(net.nodes.get(b)?.as_str())?;
                        if a == b {
                            [0.0, 0.0]
                        } else {
                            let (i, j) = (a.min(b), a.max(b));
                            let covered = sum(&path[i..j]);
                            let other = sum_bounds(sum(&path[..i]), sum(&path[j..]));
                            let denominator_min = sum_bounds([covered[1]; 2], [other[0]; 2])[0];
                            let denominator_max = sum_bounds([covered[0]; 2], [other[1]; 2])[1];
                            let ratio = [
                                (covered[0] / denominator_max).next_down(),
                                (covered[1] / denominator_min).next_up(),
                            ];
                            signed(product(emf, ratio), a < b)
                        }
                    }
                    Probe::Current(section) | Probe::Power(section) if opened.contains(section) => {
                        [0.0, 0.0]
                    }
                    Probe::Current(section) if section == supply => signed(amps, false),
                    Probe::Power(section) if section == supply => signed(
                        product(
                            square(emf),
                            [total[1].recip().next_down(), total[0].recip().next_up()],
                        ),
                        false,
                    ),
                    Probe::Current(section) | Probe::Power(section) => {
                        let (_, r, forward) = path
                            .iter()
                            .find(|(candidate, _, _)| *candidate == section)?;
                        if matches!(probe, Probe::Current(_)) {
                            signed(amps, *forward)
                        } else {
                            product(square(amps), *r)
                        }
                    }
                };
                (lo.is_finite() && hi.is_finite() && lo <= hi).then_some((
                    probe.clone(),
                    Envelope {
                        min: Extremum {
                            time: Time::ZERO,
                            value: lo,
                        },
                        max: Extremum {
                            time: Time::ZERO,
                            value: hi,
                        },
                    },
                ))
            })
            .collect()
    }
}

fn sum_bounds(a: Bounds, b: Bounds) -> Bounds {
    // Сопротивления неотрицательны. Округляем промежуточные границы наружу.
    [(a[0] + b[0]).next_down().max(0.0), (a[1] + b[1]).next_up()]
}

fn product(a: Bounds, b: Bounds) -> Bounds {
    let products = [a[0] * b[0], a[0] * b[1], a[1] * b[0], a[1] * b[1]];
    if products.iter().any(|v| !v.is_finite()) {
        return [f64::NEG_INFINITY, f64::INFINITY];
    }
    [
        products
            .into_iter()
            .fold(f64::INFINITY, f64::min)
            .next_down(),
        products
            .into_iter()
            .fold(f64::NEG_INFINITY, f64::max)
            .next_up(),
    ]
}

fn square([lo, hi]: Bounds) -> Bounds {
    let bounds = [lo.abs().min(hi.abs()), lo.abs().max(hi.abs())];
    let min = if lo <= 0.0 && hi >= 0.0 {
        0.0
    } else {
        (bounds[0] * bounds[0]).next_down().max(0.0)
    };
    [min, (bounds[1] * bounds[1]).next_up()]
}

/// Каждому физическому острову назначается собственный ноль координат.
/// Перед переносом проверяются пары управления и токовые ветви.
fn ground_islands(
    net: &mut ExpandedNetlist,
    transient: bool,
) -> Result<BTreeMap<Endpoint, usize>, String> {
    let raw: BTreeSet<_> = net
        .netlist
        .elements
        .iter()
        .flat_map(|e| e.nodes.iter().cloned())
        .collect();
    if raw.is_empty() {
        return Err("в цепи нет ветвей".into());
    }
    let indexes: BTreeMap<_, _> = raw.into_iter().enumerate().map(|(i, n)| (n, i)).collect();
    let mut groups: Vec<_> = (0..indexes.len()).collect();
    fn root(groups: &mut [usize], mut node: usize) -> usize {
        while node != groups[node] {
            groups[node] = groups[groups[node]];
            node = groups[node];
        }
        node
    }
    for e in &net.netlist.elements {
        let pairs: &[(usize, usize)] = match e.kind {
            ElementKind::CurrentSource(_) | ElementKind::Vccs { .. } => &[],
            ElementKind::Capacitor { .. } if !transient => &[],
            ElementKind::Mosfet { .. } => &[(0, 2), (2, 3)],
            _ => &[(0, 1)],
        };
        for &(a, b) in pairs {
            let a = root(&mut groups, indexes[&e.nodes[a]]);
            let b = root(&mut groups, indexes[&e.nodes[b]]);
            groups[a.max(b)] = a.min(b);
        }
    }
    let groups: BTreeMap<_, _> = indexes
        .iter()
        .map(|(name, &i)| (name.clone(), root(&mut groups, i)))
        .collect();
    for e in &net.netlist.elements {
        let required: &[(usize, usize)] = match e.kind {
            ElementKind::CurrentSource(_) => &[(0, 1)],
            ElementKind::Vccs { .. } => &[(0, 1), (2, 3)],
            ElementKind::Mosfet { .. } => &[(1, 2)],
            ElementKind::Switch {
                control: sim_circuit::netlist::SwitchControl::Voltage,
                ..
            }
            | ElementKind::Vcvs { .. } => &[(2, 3)],
            _ => &[],
        };
        if required
            .iter()
            .any(|&(a, b)| groups[&e.nodes[a]] != groups[&e.nodes[b]])
        {
            return Err(format!(
                "{}: управляющая разность или возврат тока зависит от плавающего острова",
                e.name
            ));
        }
    }
    let mut anchors = BTreeSet::new();
    let aliases: BTreeMap<_, _> = groups
        .iter()
        .map(|(name, &group)| {
            let alias = if anchors.insert(group) {
                "0".to_owned()
            } else {
                name.clone()
            };
            (name.clone(), alias)
        })
        .collect();
    // Исходный узел 0 лексикографически первый, поэтому он остаётся опорой своего острова.
    let islands = net
        .nodes
        .iter()
        .filter_map(|(endpoint, name)| Some((endpoint.clone(), *groups.get(name)?)))
        .collect();
    for el in &mut net.netlist.elements {
        for node in &mut el.nodes {
            *node = aliases[node].clone();
        }
    }
    for name in net.nodes.values_mut() {
        if let Some(alias) = aliases.get(name) {
            *name = alias.clone();
        }
    }
    Ok(islands)
}

fn case_source_name(source: &Provenance) -> &str {
    match source {
        Provenance::Datasheet(s)
        | Provenance::Measured(s)
        | Provenance::Paper(s)
        | Provenance::Derived(s)
        | Provenance::Estimate(s) => s,
    }
}

impl PreparedMode {
    pub fn netlist(&self) -> &ExpandedNetlist {
        &self.net
    }

    pub fn solve(&self) -> Analysis<ElectricalSeries> {
        let solve = || -> Result<ElectricalSeries, String> {
            let ckt = Circuit::from_netlist(&self.net.netlist).map_err(|e| e.to_string())?;
            let options = SimOptions {
                gshunt: 0.0,
                gmin: 0.0,
                ..SimOptions::default()
            };
            let mut result = ElectricalSeries {
                time: Vec::new(),
                values: self
                    .probes
                    .iter()
                    .cloned()
                    .map(|probe| (probe, Vec::new()))
                    .collect(),
            };
            if let Some(window) = &self.window {
                let uic = matches!(window.initial, InitialState::Reactive(_));
                let mut tran = Transient::with_options(
                    &ckt,
                    window.step.as_secs(),
                    TranOptions {
                        uic,
                        max_step: Some(window.step.as_secs()),
                        sim: options,
                    },
                )
                .map_err(|e| e.to_string())?;
                if !tran.warnings.is_empty() {
                    return Err(tran.warnings.join("; "));
                }
                let mut at = Time::ZERO;
                let mut events = self.events.iter().copied().peekable();
                loop {
                    tran.advance_to(at.as_secs()).map_err(|e| e.to_string())?;
                    if tran.time() != at.as_secs() {
                        return Err("решатель не достиг заданной отметки".into());
                    }
                    self.record(
                        &mut result,
                        at,
                        |n| tran.voltage(n),
                        |n| tran.current(n),
                        |n| tran.power(n),
                    )?;
                    if at == window.end {
                        break;
                    }
                    while events.peek().is_some_and(|&event| event <= at) {
                        events.next();
                    }
                    let next_grid =
                        at.0.checked_div(window.step.0)
                            .and_then(|k| k.checked_add(1))
                            .and_then(|k| k.checked_mul(window.step.0))
                            .map_or(window.end, Time);
                    at = next_grid.min(window.end);
                    if let Some(&event) = events.peek() {
                        at = at.min(event);
                    }
                }
            } else {
                let op = operating_point_with(&ckt, options).map_err(|e| e.to_string())?;
                self.record(
                    &mut result,
                    Time::ZERO,
                    |n| op.voltage(n),
                    |n| op.current(n),
                    |n| op.power(n),
                )?;
            }
            Ok(result)
        };
        match solve() {
            Ok(value) => Analysis::Analyzed {
                value,
                scope: "расчёт номинального опыта на объявленной сетке".into(),
            },
            Err(reason) => open(format!("опыт не разрешён MNA: {reason}")),
        }
    }

    fn record(
        &self,
        row: &mut ElectricalSeries,
        at: Time,
        volt: impl Fn(&str) -> Result<f64, sim_circuit::SimError>,
        current: impl Fn(&str) -> Result<f64, sim_circuit::SimError>,
        power: impl Fn(&str) -> Result<f64, sim_circuit::SimError>,
    ) -> Result<(), String> {
        let vertex = |end: &Endpoint| {
            self.net
                .nodes
                .get(end)
                .ok_or_else(|| format!("плавающий вывод {end}"))
        };
        let read = |f: &dyn Fn(&str) -> Result<f64, sim_circuit::SimError>, name: &str| {
            f(name).map_err(|e| e.to_string())
        };
        for (probe, values) in &mut row.values {
            let value = match probe {
                Probe::Voltage(a, b) => read(&volt, vertex(a)?)? - read(&volt, vertex(b)?)?,
                Probe::Current(section) | Probe::Power(section) => {
                    if self.opened.contains(section) {
                        0.0
                    } else {
                        let name = self
                            .net
                            .origins
                            .iter()
                            .find_map(|(n, s)| (s == section).then_some(n))
                            .ok_or_else(|| format!("нет ветви {section:?}"))?;
                        read(
                            if matches!(probe, Probe::Current(_)) {
                                &current
                            } else {
                                &power
                            },
                            name,
                        )?
                    }
                }
            };
            if !value.is_finite() {
                return Err(format!("неконечное наблюдение {probe:?}"));
            }
            values.push(value);
        }
        row.time.push(at);
        Ok(())
    }

    fn report(&self, mode: Mode) -> ModeReport {
        let nominal = self.solve();
        let envelope = match &nominal {
            Analysis::Analyzed { value, .. } if self.gaps.is_empty() => Analysis::Analyzed {
                value: value.envelopes(),
                scope:
                    "точные подтверждённые параметры; DC или кусочно-линейный ряд без накопителей"
                        .into(),
            },
            Analysis::Analyzed { .. } => self.dc_bounds.clone().map_or_else(
                || open(self.gaps.join("; ")),
                |value| Analysis::Analyzed {
                    value,
                    scope: "интервальная огибающая одиночного резистивного контура".into(),
                },
            ),
            Analysis::NotAnalyzed { reason } => open(reason.clone()),
        };
        ModeReport {
            mode,
            nominal,
            envelope,
            window: self.window.clone(),
            source_fronts: self.source_fronts.clone(),
            opened: self.opened.clone(),
            seed: None,
            samples: Vec::new(),
        }
    }

    fn sample(&self, mode: &Mode, sampling: &Sampling) -> Vec<SampleRun> {
        let mut rng = Rng::stream(sampling.seed, &format!("электрический режим {mode:?}"));
        (0..sampling.count)
            .map(|_| {
                let mut prepared = self.clone();
                let connection_ohm: BTreeMap<_, _> = sampling
                    .connection_ohm
                    .iter()
                    .map(|(id, r)| (id.clone(), r.min + (r.max - r.min) * rng.uniform()))
                    .collect();
                for el in &mut prepared.net.netlist.elements {
                    if let Section::Connection(id) = &prepared.net.origins[&el.name]
                        && let Some(&r) = connection_ohm.get(id)
                    {
                        el.kind = ElementKind::Resistor { r };
                    }
                }
                SampleRun {
                    connection_ohm,
                    trace: prepared.solve(),
                }
            })
            .collect()
    }
}
