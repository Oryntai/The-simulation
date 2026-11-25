use std::collections::{BTreeMap, BTreeSet};

use sim_process::schematic::{
    ClampPath, Parameter, PinProfile, PinState, Pull,
    sim_circuit::{
        Circuit, SimOptions, TranOptions, Transient,
        netlist::{
            Analysis, Element, ElementKind, ModelCard, ModelKind, Netlist, SwitchControl, Waveform,
        },
    },
};
use sim_signal::Time;

use super::{BoardConnection, CircuitModel};

#[derive(Clone, Debug)]
pub struct LivePin {
    pub contact: String,
    /// Порядок узлов: контакт, локальные плюс и возврат питания.
    pub nodes: [String; 3],
    pub profile: PinProfile,
    pub initial: PinState,
}

#[derive(Clone)]
pub struct TransientCircuit {
    solver: Transient,
    sources: BTreeMap<String, [String; 2]>,
    pins: BTreeMap<String, PinSwitches>,
}

#[derive(Clone)]
struct PinSwitches {
    nodes: [String; 3],
    profile: PinProfile,
    switches: Vec<(String, PinState)>,
}

fn nominal(parameter: &Parameter) -> Result<f64, String> {
    let range = parameter
        .range
        .as_ref()
        .ok_or_else(|| parameter.measurement.clone())?;
    range.validate().map_err(|e| e.to_string())?;
    Ok(range.nominal)
}

// Подтянутый вход использует одновременно сопротивление входа и подтяжку.
fn active(path: PinState, state: PinState) -> bool {
    path == state || path == PinState::Input && matches!(state, PinState::InputPull(_))
}

fn validate_netlist(net: &Netlist) -> Result<(), String> {
    let positive = |x: f64| x.is_finite() && x > 0.0;
    let mut names = BTreeSet::new();
    for element in &net.elements {
        let (terminals, valid) = match &element.kind {
            ElementKind::Resistor { r } => (2, positive(*r) && r.recip().is_finite()),
            ElementKind::Capacitor { c, ic } => (2, positive(*c) && ic.is_none_or(f64::is_finite)),
            ElementKind::Inductor { l, ic } => (2, positive(*l) && ic.is_none_or(f64::is_finite)),
            ElementKind::VoltageSource(source) | ElementKind::CurrentSource(source) => {
                let wave = match &source.wave {
                    None => true,
                    Some(Waveform::Ext { initial }) => initial.is_finite(),
                    Some(Waveform::Pwl(points)) => {
                        !points.is_empty()
                            && points
                                .iter()
                                .all(|&(t, v)| t.is_finite() && t >= 0.0 && v.is_finite())
                            && points.windows(2).all(|pair| pair[0].0 < pair[1].0)
                    }
                    Some(Waveform::Sin {
                        vo,
                        va,
                        freq,
                        td,
                        theta,
                        phase_deg,
                    }) => [vo, va, freq, td, theta, phase_deg]
                        .into_iter()
                        .all(|v| v.is_finite()),
                    Some(Waveform::Pulse {
                        v1,
                        v2,
                        td,
                        tr,
                        tf,
                        pw,
                        per,
                    }) => [v1, v2, td, tr, tf, pw, per]
                        .into_iter()
                        .all(|v| v.is_finite()),
                };
                (2, wave && source.dc.is_none_or(f64::is_finite))
            }
            ElementKind::Diode { area, .. } => (2, positive(*area)),
            ElementKind::Switch {
                control, ron, roff, ..
            } => (
                if *control == SwitchControl::External {
                    2
                } else {
                    4
                },
                ron.is_none_or(positive) && roff.is_none_or(positive),
            ),
            ElementKind::Mosfet { w, l, .. } => {
                (4, w.is_none_or(positive) && l.is_none_or(positive))
            }
            ElementKind::Vcvs { gain } => (4, gain.is_finite()),
            ElementKind::Vccs { gm } => (4, gm.is_finite()),
        };
        if !valid
            || element.nodes.len() != terminals
            || element.nodes.iter().any(|node| node.trim().is_empty())
            || element.name.is_empty()
            || !names.insert(element.name.to_ascii_lowercase())
        {
            return Err(format!(
                "«{}»: недопустимые параметры, клеммы или повтор ветви MNA",
                element.name
            ));
        }
    }
    names.clear();
    for model in &net.models {
        let mut keys = BTreeSet::new();
        if !names.insert(model.name.to_ascii_lowercase())
            || model.name.is_empty()
            || model
                .params
                .iter()
                .any(|(key, value)| !value.is_finite() || !keys.insert(key.to_ascii_lowercase()))
        {
            return Err(format!(
                "«{}»: недопустимая карточка модели MNA",
                model.name
            ));
        }
    }
    for analysis in &net.analyses {
        if let Analysis::Tran(spec) = analysis
            && !(positive(spec.tstep)
                && positive(spec.tstop)
                && spec.tstart.is_finite()
                && (0.0..=spec.tstop).contains(&spec.tstart)
                && spec.tmax.is_none_or(positive))
        {
            return Err("недопустимые границы переходного анализа".into());
        }
    }
    Ok(())
}

impl TransientCircuit {
    /// Начальная точка берётся из DC или явных начальных условий UIC. Шаг интегрирования ограничен сеткой
    /// обмена, а все переключения сохраняют принятую левую границу.
    pub fn new(mut net: Netlist, step: Time, pins: Vec<LivePin>) -> Result<Self, String> {
        let mut contacts = BTreeMap::new();
        for pin in pins {
            pin.profile.validate().map_err(|e| e.to_string())?;
            let [pad, plus, minus] = &pin.nodes;
            let model = &pin.profile.model;
            let mut switches = Vec::new();
            for (path, rail, resistance) in [
                (PinState::High, plus, &model.high_ohm),
                (PinState::Low, minus, &model.low_ohm),
                (PinState::Input, minus, &model.input_ohm),
                (PinState::HighZ, minus, &model.high_z_ohm),
                (PinState::InputPull(Pull::Up), plus, &model.pull_up_ohm),
                (PinState::InputPull(Pull::Down), minus, &model.pull_down_ohm),
            ] {
                let name = format!("slive{}", net.elements.len());
                net.elements.push(Element {
                    name: name.clone(),
                    nodes: vec![pad.clone(), rail.clone()],
                    kind: ElementKind::Switch {
                        control: SwitchControl::External,
                        model: None,
                        initially_on: active(path, pin.initial),
                        ron: Some(nominal(resistance)?),
                        // Машинное представление разомкнутой ветви MNA. Сама
                        // физическая утечка задаётся отдельным high_z_ohm.
                        roff: Some(f64::MAX),
                    },
                    line: 0,
                });
                switches.push((name, path));
            }
            for (path, terminals) in [
                (&model.upper_clamp, [pad, plus]),
                (&model.lower_clamp, [minus, pad]),
            ] {
                match path {
                    ClampPath::Diode(diode) => {
                        let name = format!("live_diode{}", net.models.len());
                        net.models.push(ModelCard {
                            name: name.clone(),
                            kind: ModelKind::Diode,
                            params: vec![
                                ("is".into(), nominal(&diode.saturation_a)?),
                                ("n".into(), nominal(&diode.ideality)?),
                                ("rs".into(), nominal(&diode.series_ohm)?),
                            ],
                            line: 0,
                        });
                        net.elements.push(Element {
                            name: format!("dlive{}", net.elements.len()),
                            nodes: terminals.into_iter().cloned().collect(),
                            // Нормированная площадь соответствует модели самого вывода.
                            kind: ElementKind::Diode {
                                model: name,
                                area: 1.0,
                            },
                            line: 0,
                        });
                    }
                    ClampPath::Absent(_) => {}
                    ClampPath::Unknown => return Err("неизвестен защитный путь контакта".into()),
                }
            }
            if contacts
                .insert(
                    pin.contact,
                    PinSwitches {
                        nodes: pin.nodes,
                        profile: pin.profile,
                        switches,
                    },
                )
                .is_some()
            {
                return Err("контакт MNA подключён дважды".into());
            }
        }
        validate_netlist(&net)?;
        let mut sources = BTreeMap::new();
        for element in &net.elements {
            if let ElementKind::CurrentSource(source) = &element.kind
                && matches!(source.wave, Some(Waveform::Ext { .. }))
            {
                let nodes: [String; 2] = element.nodes.clone().try_into().map_err(|_| {
                    format!("{}: ожидаются две клеммы источника тока", element.name)
                })?;
                sources.insert(element.name.to_ascii_lowercase(), nodes);
            }
        }
        let circuit = Circuit::from_netlist(&net).map_err(|e| e.to_string())?;
        let transient = circuit.tran_spec();
        let solver = Transient::with_options(
            &circuit,
            step.as_secs(),
            TranOptions {
                uic: transient.is_some_and(|spec| spec.uic),
                max_step: transient.and_then(|spec| spec.tmax),
                // Численные шунты не должны замыкать отсутствующий путь возврата.
                sim: SimOptions {
                    gshunt: 0.0,
                    gmin: 0.0,
                    ..Default::default()
                },
            },
        )
        .map_err(|e| e.to_string())?;
        Ok(Self {
            solver,
            sources,
            pins: contacts,
        })
    }
}

impl CircuitModel for TransientCircuit {
    fn validate_board(&self, connection: &BoardConnection) -> Result<(), String> {
        self.validate_current_source(
            &connection.plus,
            &connection.minus,
            &connection.current_source,
        )?;
        for pin in &connection.pins {
            let wired = self
                .pins
                .get(&pin.contact)
                .ok_or_else(|| format!("контакт «{}» отсутствует в MNA", pin.contact))?;
            if wired.nodes[1..] != [connection.plus.as_str(), connection.minus.as_str()]
                || wired.profile != pin.profile
            {
                return Err(format!(
                    "контакт «{}»: шина или электрическая модель не совпала",
                    pin.contact
                ));
            }
        }
        Ok(())
    }

    fn validate_current_source(&self, plus: &str, minus: &str, branch: &str) -> Result<(), String> {
        if plus.eq_ignore_ascii_case(minus)
            || !self
                .sources
                .get(&branch.to_ascii_lowercase())
                .is_some_and(|[p, n]| p.eq_ignore_ascii_case(plus) && n.eq_ignore_ascii_case(minus))
        {
            return Err(format!(
                "«{branch}»: нужна внешняя ветвь тока от «{plus}» к «{minus}»"
            ));
        }
        Ok(())
    }

    fn advance_to(&mut self, at: Time) -> Result<(), String> {
        self.solver
            .advance_to(at.as_secs())
            .map_err(|e| e.to_string())
    }

    fn voltage(&self, plus: &str, minus: &str) -> Result<f64, String> {
        let value = |node| self.solver.voltage(node).map_err(|e| e.to_string());
        Ok(value(plus)? - value(minus)?)
    }

    fn current(&self, branch: &str) -> Result<f64, String> {
        self.solver.current(branch).map_err(|e| e.to_string())
    }

    fn set_source_step(&mut self, name: &str, value: f64) -> Result<(), String> {
        self.solver
            .set_source_step(name, value)
            .map_err(|e| e.to_string())
    }

    fn set_pin(&mut self, contact: &str, state: PinState) -> Result<(), String> {
        let pin = self
            .pins
            .get(contact)
            .ok_or_else(|| format!("контакт «{contact}» отсутствует в MNA"))?;
        for (name, path) in &pin.switches {
            self.solver
                .set_switch(name, active(*path, state))
                .map_err(|e| e.to_string())?;
        }
        Ok(())
    }
}
