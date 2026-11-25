//! Эквивалент контакта относительно его локальных шин и состояния входного буфера.

use sim_circuit::netlist::{Element, ElementKind, ModelCard, ModelKind};
use sim_signal::Time;
use sim_world::Provenance;

use crate::{Analysis, ProcessError};

use super::{
    BindingTarget, CompiledSchematic, Endpoint, ExpandedNetlist, Parameter, PinFunction, Scope,
    Section, identifier, invalid,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pull {
    Up,
    Down,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PinState {
    High,
    Low,
    Input,
    InputPull(Pull),
    HighZ,
}

impl PinState {
    pub fn input_enabled(self) -> bool {
        matches!(self, Self::Input | Self::InputPull(_))
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct SourcedPinState {
    pub state: PinState,
    pub source: Provenance,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ClampDiode {
    pub saturation_a: Parameter,
    pub ideality: Parameter,
    pub series_ohm: Parameter,
    pub injection_limit_a: Parameter,
}

#[derive(Clone, Debug, PartialEq)]
pub enum ClampPath {
    Unknown,
    Absent(Provenance),
    Diode(ClampDiode),
}

#[derive(Clone, Debug, PartialEq)]
pub struct PinModel {
    pub high_ohm: Parameter,
    pub low_ohm: Parameter,
    pub pull_up_ohm: Parameter,
    pub pull_down_ohm: Parameter,
    /// Эквивалент утечки включённого приёмника к VSS в заданной области модели.
    pub input_ohm: Parameter,
    /// Остаточная утечка отключённого буфера к VSS; драйверы и подтяжки разомкнуты.
    pub high_z_ohm: Parameter,
    pub upper_clamp: ClampPath,
    pub lower_clamp: ClampPath,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BootStage {
    pub duration_s: Parameter,
    pub pin: SourcedPinState,
}

#[derive(Clone, Debug, PartialEq)]
pub enum PinPhase {
    Unpowered,
    Reset,
    Boot { elapsed: Time },
    Working(SourcedPinState),
}

#[derive(Clone, Debug, PartialEq)]
pub struct PinProfile {
    pub model: PinModel,
    pub unpowered: Option<SourcedPinState>,
    pub reset: Option<SourcedPinState>,
    /// Последовательность для конкретного образа, включая загрузчик.
    pub boot: Vec<BootStage>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ResetProfile {
    pub falling_v: Parameter,
    pub rising_v: Parameter,
    pub assert_delay_s: Parameter,
    pub release_delay_s: Parameter,
}

impl ResetProfile {
    pub fn validate(&self) -> Result<(), ProcessError> {
        for parameter in [
            &self.falling_v,
            &self.rising_v,
            &self.assert_delay_s,
            &self.release_delay_s,
        ] {
            parameter.validate(|v| v >= 0.0)?;
        }
        if let (Some(fall), Some(rise)) = (&self.falling_v.range, &self.rising_v.range)
            && fall.nominal >= rise.nominal
        {
            return Err(invalid(
                "номинальный порог восстановления должен быть выше порога сброса",
            ));
        }
        Ok(())
    }
}

impl PinProfile {
    pub fn validate(&self) -> Result<(), ProcessError> {
        self.model.validate()?;
        for state in self
            .unpowered
            .iter()
            .chain(&self.reset)
            .chain(self.boot.iter().map(|stage| &stage.pin))
        {
            validate_source(&state.source)?;
        }
        for stage in &self.boot {
            stage
                .duration_s
                .validate(|v| Time::from_secs(v).is_some_and(|t| t > Time::ZERO))?;
        }
        Ok(())
    }

    pub fn state(
        &self,
        phase: &PinPhase,
        scope: Scope,
    ) -> Result<Analysis<PinState>, ProcessError> {
        self.validate()?;
        let selected = match phase {
            PinPhase::Unpowered => self.unpowered.as_ref(),
            PinPhase::Reset => self.reset.as_ref(),
            PinPhase::Working(state) => Some(state),
            PinPhase::Boot { elapsed } => {
                let mut lower = 0_u64;
                let mut upper = 0_u64;
                for stage in &self.boot {
                    let Some(range) = stage
                        .duration_s
                        .range
                        .as_ref()
                        .filter(|r| r.can_prove() || scope.exact(r))
                    else {
                        return Ok(open(format!(
                            "выдержка загрузки: {}",
                            stage.duration_s.measurement
                        )));
                    };
                    let min = ns_bound(range.min, false)?;
                    let max = ns_bound(range.max, true)?;
                    let end_lower = lower
                        .checked_add(min)
                        .ok_or_else(|| invalid("загрузка вне шкалы времени"))?;
                    let end_upper = upper
                        .checked_add(max)
                        .ok_or_else(|| invalid("загрузка вне шкалы времени"))?;
                    if upper <= elapsed.0 && elapsed.0 < end_lower {
                        return sourced_state(&stage.pin, scope);
                    }
                    lower = end_lower;
                    upper = end_upper;
                }
                return Ok(open(
                    "состояние на этом такте загрузки не установлено профилем",
                ));
            }
        };
        selected.map_or_else(
            || Ok(open("состояние вывода для фазы не задано")),
            |state| sourced_state(state, scope),
        )
    }
}

fn sourced_state(
    state: &SourcedPinState,
    scope: Scope,
) -> Result<Analysis<PinState>, ProcessError> {
    validate_source(&state.source)?;
    Ok(if supports_source(scope, &state.source) {
        Analysis::Analyzed {
            value: state.state,
            scope: "электрический профиль контакта".into(),
        }
    } else {
        open("состояние контакта не подтверждено для этой сборки")
    })
}

pub(super) fn open<T>(reason: impl Into<String>) -> Analysis<T> {
    Analysis::NotAnalyzed {
        reason: reason.into(),
    }
}

pub(super) fn supports_source(scope: Scope, source: &Provenance) -> bool {
    crate::source_can_prove(source)
        || matches!((scope, source), (Scope::Reference, Provenance::Derived(_)))
}

pub(super) fn validate_source(source: &Provenance) -> Result<(), ProcessError> {
    let text = match source {
        Provenance::Datasheet(s)
        | Provenance::Paper(s)
        | Provenance::Measured(s)
        | Provenance::Derived(s)
        | Provenance::Estimate(s) => s,
    };
    if identifier(text) {
        Ok(())
    } else {
        Err(invalid("требуется источник состояния или события"))
    }
}

/// Границы длительности округляются наружу при переводе секунд в наносекунды.
pub fn ns_bound(seconds: f64, upper: bool) -> Result<u64, ProcessError> {
    let nearest = Time::from_secs(seconds).ok_or_else(|| invalid("выдержка вне шкалы времени"))?;
    let scaled = seconds * 1_000_000_000.0;
    // Целые наносекунды, прошедшие обратный перевод, описывают точный такт.
    if nearest.as_secs() == seconds {
        return Ok(nearest.0);
    }
    Ok(if upper { scaled.ceil() } else { scaled.floor() } as u64)
}

impl PinModel {
    pub fn validate(&self) -> Result<(), ProcessError> {
        for r in [
            &self.high_ohm,
            &self.low_ohm,
            &self.pull_up_ohm,
            &self.pull_down_ohm,
            &self.input_ohm,
            &self.high_z_ohm,
        ] {
            r.validate(|v| v > 0.0 && v.recip().is_finite())?;
        }
        for clamp in [&self.upper_clamp, &self.lower_clamp] {
            match clamp {
                ClampPath::Diode(d) => {
                    d.saturation_a.validate(|v| v > 0.0)?;
                    d.ideality.validate(|v| v > 0.0)?;
                    d.series_ohm
                        .validate(|v| v == 0.0 || (v > 0.0 && v.recip().is_finite()))?;
                    d.injection_limit_a.validate(|v| v >= 0.0)?;
                }
                ClampPath::Absent(source) => validate_source(source)?,
                ClampPath::Unknown => {}
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct BoardContact {
    pub board: String,
    pub pin: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PinSetup {
    pub rail: String,
    pub profile: PinProfile,
    pub phase: PinPhase,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PinStamp {
    pub ends: [Endpoint; 3],
    pub state: PinState,
    pub output: Option<Section>,
    pub passive: Vec<Section>,
    pub upper_clamp: Option<Section>,
    pub lower_clamp: Option<Section>,
}

impl CompiledSchematic {
    pub fn board_contact_ends(
        &self,
        contact: &BoardContact,
        rail: &str,
    ) -> Result<[Endpoint; 3], ProcessError> {
        let find = |predicate: &dyn Fn(&BindingTarget) -> bool| {
            self.schematic
                .bindings
                .iter()
                .find(|b| predicate(&b.target))
        };
        let binding = find(&|t| matches!(t, BindingTarget::BoardPin { board, pin, .. } if board == &contact.board && pin == &contact.pin))
            .ok_or_else(|| invalid(format!("неизвестный контакт платы {}/{}", contact.board, contact.pin)))?;
        let supply = find(&|t| matches!(t, BindingTarget::BoardPower { board, rail: name } if board == &contact.board && name == rail))
            .ok_or_else(|| invalid(format!("плата «{}»: неизвестная локальная шина «{rail}»", contact.board)))?;
        let ends = [
            binding.ends[0].clone(),
            supply.ends[0].clone(),
            supply.ends[1].clone(),
        ];
        let pin = self.pin(&ends[0]);
        let reference = Endpoint {
            component: ends[0].component.clone(),
            pin: pin.role.reference().unwrap().into(),
        };
        if reference != ends[2]
            && self
                .route(self.index[&reference], self.index[&ends[2]], true)
                .is_none()
        {
            return Err(invalid(format!(
                "опора контакта «{}» не совпала с возвратом шины «{rail}»",
                ends[0]
            )));
        }
        Ok(ends)
    }

    pub(super) fn validate_pin_setup(
        &self,
        contact: &BoardContact,
        setup: &PinSetup,
    ) -> Result<(), ProcessError> {
        self.board_contact_ends(contact, &setup.rail)?;
        setup.profile.validate()?;
        if let PinPhase::Working(state) = &setup.phase {
            validate_source(&state.source)?;
            let Some(function) = self
                .schematic
                .bindings
                .iter()
                .find_map(|binding| match &binding.target {
                    BindingTarget::BoardPin {
                        board,
                        pin,
                        function,
                    } if board == &contact.board && pin == &contact.pin => Some(*function),
                    _ => None,
                })
            else {
                return Err(invalid("неизвестный контакт в штатном режиме"));
            };
            let compatible = match function {
                PinFunction::GpioInput => state.state.input_enabled(),
                PinFunction::Adc => matches!(state.state, PinState::Input | PinState::HighZ),
                PinFunction::GpioOutput | PinFunction::Pwm => matches!(
                    state.state,
                    PinState::High | PinState::Low | PinState::HighZ
                ),
                PinFunction::Uart | PinFunction::I2c | PinFunction::Spi => true,
            };
            if !compatible {
                return Err(invalid(format!(
                    "{}/{}: штатное электрическое состояние {:?} противоречит функции {function:?}",
                    contact.board, contact.pin, state.state
                )));
            }
        }
        Ok(())
    }

    /// Раскрывает вывод в физических узлах схемы и сохраняет адреса MNA-ветвей.
    pub fn stamp_pin(
        &self,
        net: &mut ExpandedNetlist,
        contact: &BoardContact,
        setup: &PinSetup,
    ) -> Result<Analysis<PinStamp>, ProcessError> {
        self.validate_pin_setup(contact, setup)?;
        let ends = self.board_contact_ends(contact, &setup.rail)?;
        let state = match setup.profile.state(&setup.phase, self.schematic.scope)? {
            Analysis::Analyzed { value, .. } => value,
            Analysis::NotAnalyzed { reason } => return Ok(open(reason)),
        };
        let [pad, vdd, vss] = ends.each_ref().map(|end| {
            net.nodes
                .entry(end.clone())
                .or_insert_with(|| format!("pin{}", self.index[end]))
                .clone()
        });
        let model = &setup.profile.model;
        let owner = format!("{}/{}", contact.board, contact.pin);
        let mut stamp = PinStamp {
            ends,
            state,
            output: None,
            passive: Vec::new(),
            upper_clamp: None,
            lower_clamp: None,
        };
        let mut resist = |label: &str, rail: &str, parameter: &Parameter| {
            let value = nominal(net, self.schematic.scope, &owner, parameter)?;
            Some(add(
                net,
                &owner,
                label,
                'r',
                [pad.clone(), rail.into()],
                ElementKind::Resistor { r: value },
            ))
        };
        match state {
            PinState::High => stamp.output = resist("high", &vdd, &model.high_ohm),
            PinState::Low => stamp.output = resist("low", &vss, &model.low_ohm),
            PinState::Input | PinState::InputPull(_) => {
                stamp
                    .passive
                    .extend(resist("input", &vss, &model.input_ohm));
                match state {
                    PinState::InputPull(Pull::Up) => {
                        stamp
                            .passive
                            .extend(resist("pull_up", &vdd, &model.pull_up_ohm));
                    }
                    PinState::InputPull(Pull::Down) => {
                        stamp
                            .passive
                            .extend(resist("pull_down", &vss, &model.pull_down_ohm));
                    }
                    _ => {}
                }
            }
            PinState::HighZ => {
                stamp
                    .passive
                    .extend(resist("leakage", &vss, &model.high_z_ohm));
            }
        }
        stamp.upper_clamp = clamp(
            net,
            self.schematic.scope,
            &owner,
            "upper",
            [pad.clone(), vdd],
            &model.upper_clamp,
        );
        stamp.lower_clamp = clamp(
            net,
            self.schematic.scope,
            &owner,
            "lower",
            [vss, pad],
            &model.lower_clamp,
        );
        Ok(Analysis::Analyzed {
            value: stamp,
            scope: "номинальный эквивалент контакта; пробелы записаны в нетлисте".into(),
        })
    }
}

fn nominal(
    net: &mut ExpandedNetlist,
    scope: Scope,
    owner: &str,
    parameter: &Parameter,
) -> Option<f64> {
    if !parameter.range.as_ref().is_some_and(|r| scope.exact(r)) {
        net.netlist.warnings.push(format!(
            "{owner}: нужна огибающая или замер ({})",
            parameter.measurement
        ));
    }
    parameter.range.as_ref().map(|r| r.nominal)
}

fn add(
    net: &mut ExpandedNetlist,
    owner: &str,
    label: &str,
    prefix: char,
    nodes: [String; 2],
    kind: ElementKind,
) -> Section {
    let name = format!("{prefix}pin{}", net.netlist.elements.len());
    let section = Section::Element {
        component: owner.into(),
        element: label.into(),
    };
    net.origins.insert(name.clone(), section.clone());
    net.netlist.elements.push(Element {
        name,
        nodes: nodes.into(),
        kind,
        line: 0,
    });
    section
}

fn clamp(
    net: &mut ExpandedNetlist,
    scope: Scope,
    owner: &str,
    label: &str,
    nodes: [String; 2],
    path: &ClampPath,
) -> Option<Section> {
    let diode = match path {
        ClampPath::Diode(diode) => diode,
        ClampPath::Absent(source) if supports_source(scope, source) => return None,
        _ => {
            net.netlist.warnings.push(format!("{owner}: не установлен путь защиты {label}; нужна ВАХ контакта при включённой и отключённой шине"));
            return None;
        }
    };
    let values = [&diode.saturation_a, &diode.ideality, &diode.series_ohm]
        .map(|p| nominal(net, scope, owner, p));
    let [Some(is), Some(n), Some(rs)] = values else {
        return None;
    };
    let model = format!("pinmodel{}", net.netlist.models.len());
    net.netlist.models.push(ModelCard {
        name: model.clone(),
        kind: ModelKind::Diode,
        params: vec![("is".into(), is), ("n".into(), n), ("rs".into(), rs)],
        line: 0,
    });
    // Карточка описывает одну защитную ветвь, поэтому множитель площади равен единице.
    Some(add(
        net,
        owner,
        label,
        'd',
        nodes,
        ElementKind::Diode { model, area: 1.0 },
    ))
}

/// Пределы PinSpec после разрешения физического контакта. Логические уровни
/// относятся к номинальному питанию; номинальные токи не заменяют абсолютный предел.
#[derive(Clone, Debug, PartialEq)]
pub struct PinLimits {
    pub nominal_supply_v: f64,
    pub abs_input_v: Option<[f64; 2]>,
    pub specified_source_a: Option<f64>,
    pub specified_sink_a: Option<f64>,
    pub abs_pin_a: Option<f64>,
    pub vil_max_v: f64,
    pub vih_min_v: f64,
    pub vol_max_v: f64,
    pub voh_min_v: f64,
    pub source: String,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::SourcedRange;
    use sim_circuit::{Circuit, SimOptions, operating_point_with};
    use std::collections::BTreeMap;

    #[test]
    fn clamp_orientation_and_kirchhoff_current() {
        let point = |value| Parameter {
            range: Some(SourcedRange {
                min: value,
                nominal: value,
                max: value,
                provenance: Provenance::Derived("параметры синтетической ветви".into()),
            }),
            measurement: "снять ВАХ защитного диода".into(),
        };
        // 5 В через 1 кОм в обе стороны. Отключённая шина VDD удерживается на VSS
        // нулевым источником стенда для отдельного измерения тока верхнего диода.
        let diode = ClampPath::Diode(ClampDiode {
            saturation_a: point(1e-12),
            ideality: point(1.0),
            series_ohm: point(0.0),
            injection_limit_a: point(0.001),
        });
        for supply in [5.0, -5.0] {
            let mut net = ExpandedNetlist {
                netlist: sim_circuit::parse(&format!(
                    "Блок\nVtest drive 0 {supply:e}\nVrail vdd 0 0\nRfeed drive pad 1000\n"
                ))
                .unwrap(),
                nodes: BTreeMap::new(),
                origins: BTreeMap::new(),
            };
            clamp(
                &mut net,
                Scope::Reference,
                "board/0",
                "upper",
                ["pad".into(), "vdd".into()],
                &diode,
            )
            .unwrap();
            clamp(
                &mut net,
                Scope::Reference,
                "board/0",
                "lower",
                ["0".into(), "pad".into()],
                &diode,
            )
            .unwrap();
            let op = operating_point_with(
                &Circuit::from_netlist(&net.netlist).unwrap(),
                SimOptions {
                    gmin: 0.0,
                    gshunt: 0.0,
                    ..SimOptions::default()
                },
            )
            .unwrap();
            let upper = op.current("dpin3").unwrap();
            let lower = op.current("dpin4").unwrap();
            let pad = op.voltage("pad").unwrap();
            // KCL: (Vист−Vконтакта)/R = Iверх−Iниз, 1 нА покрывает численное округление.
            assert!(((supply - pad) / 1000.0 - (upper - lower)).abs() < 1e-9);
            let [conducting, blocking] = if supply > 0.0 {
                [upper, lower]
            } else {
                [lower, upper]
            };
            // Ток должен пересечь заданный предел стенда только в прямой ветви.
            assert!(conducting > 0.001 && blocking < 0.0 && blocking.abs() < 0.001);
        }
    }
}
