#![allow(dead_code)]

use std::cell::RefCell;
use std::rc::Rc;

use sim_runtime::{
    Evaluator, Forces, Object, ObservationOut, Runtime, Sensor, Truth, TruthId, TruthOut,
};
use sim_signal::{ChannelId, Command, Observation, Rate, Rng, SignalId, Time};
use sim_task::*;
use sim_world::{
    CompiledDrive, CompiledJoint, CompiledJointKind, CompiledWorld, EffortChannel, Simplification,
};

/// Параметры аналитического стенда: 1 кг, 1 кГц, без тяжести и трения.
/// Контроллер подаёт 2 Н/м, а единичная ступень цели включается на 4-м такте.
/// Отказ активируется на 8-м такте, горизонт 12 тактов.
pub const TICK: u64 = 1_000_000;
const DT: f64 = 0.001;
const EXCITE: u64 = 4 * TICK;
const FAULT: u64 = 8 * TICK;
pub const HORIZON: u64 = 12 * TICK;
pub const DRIVE: &str = "стенд/модель";
pub const PATH: &str = "цепь";
pub const PART: &str = "нейтральная модель";
const JOINT: &str = "стенд/ось";
const OBS: &str = "стенд/измерение";
const COMMAND: &str = "стенд/сила";
const ACCEPTED: &str = "стенд/модель/принято";
const POSITION: &str = "стенд/положение";
const ACCEL: &str = "стенд/ускорение";
const EVENT: &str = "стенд/граница";
const CONTROLLER: &str = "регулятор";
const RADIO: &str = "связь";
const SOURCE: &str = "{kind='derived',reference='параметры и уравнения аналитического стенда'}";

pub fn source() -> Source {
    Source {
        kind: SourceKind::Derived,
        reference: "параметры и уравнения аналитического стенда".into(),
    }
}

pub fn signal(name: &str, unit: &str) -> Signal {
    Signal {
        name: name.into(),
        unit: unit.into(),
    }
}

const FAULTS: [(&str, NodeKind, &str); 6] = [
    ("датчик", NodeKind::Sensor, "sensor_unavailable"),
    ("провод", NodeKind::Wire, "wire_open"),
    (CONTROLLER, NodeKind::Controller, "controller_halted"),
    ("драйвер", NodeKind::Driver, "driver_disabled"),
    ("питание", NodeKind::Power, "power_cut"),
    (DRIVE, NodeKind::Drive, "drive_disabled"),
];

pub fn world(part: &str) -> CompiledWorld {
    CompiledWorld {
        name: "Каретка".into(),
        // Зерно тестового вектора. Источников шума в этом мире нет.
        seed: 13,
        physics: Rate::hz(1000).unwrap(),
        mjcf: format!(
            r#"<mujoco><option timestep="0.001" gravity="0 0 0" integrator="Euler"/>
          <worldbody><body name="блок"><joint name="{JOINT}" type="slide" axis="1 0 0"/>
          <geom size="0.1" mass="1" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#
        ),
        bodies: vec![],
        joints: vec![CompiledJoint {
            name: JOINT.into(),
            owner: "стенд".into(),
            kind: CompiledJointKind::Slide,
        }],
        drives: vec![CompiledDrive {
            name: DRIVE.into(),
            owner: "стенд".into(),
            part: part.into(),
            effort: COMMAND.into(),
            joint: JOINT.into(),
            params: Default::default(),
        }],
        efforts: vec![EffortChannel {
            name: COMMAND.into(),
            joint: JOINT.into(),
            dof: 0,
        }],
        sensors: vec![],
        simplifications: vec![],
    }
}

pub fn rotary_world(part: &str) -> CompiledWorld {
    let mut world = world(part);
    world.joints[0].kind = CompiledJointKind::Hinge;
    world.mjcf = world
        .mjcf
        .replace("type=\"slide\"", "type=\"hinge\"")
        .replace(
            "<geom",
            "<inertial pos=\"0 0 0\" mass=\"1\" diaginertia=\"1 1 1\"/><geom",
        );
    world
}

pub fn rotary_topology() -> Topology {
    let mut topology = topology(false);
    let path = &mut topology.paths[0];
    path.observation.unit = "rad".into();
    path.command.unit = "N*m".into();
    path.accepted.unit = "N*m".into();
    path.physical_response[0].unit = "rad".into();
    path.physical_response[1].unit = "rad/s^2".into();
    topology
}

pub fn rotary_document(max: f64, deadline: u64) -> String {
    document(max, deadline, false)
        .replace("unit='m'", "unit='rad'")
        .replace("unit='m/s^2'", "unit='rad/s^2'")
}

pub fn topology(radio: bool) -> Topology {
    let mut nodes: Vec<_> = FAULTS
        .iter()
        .map(|&(id, kind, _)| Node {
            id: id.into(),
            kind,
            fault_state: Some(signal(&fault_name(id), "1")),
        })
        .collect();
    let mut hops = Vec::new();
    if radio {
        nodes.push(Node {
            id: RADIO.into(),
            kind: NodeKind::Radio,
            fault_state: None,
        });
        hops.push(Hop {
            id: RADIO.into(),
            node: RADIO.into(),
            direction: Direction::Input,
            delay_ns: 0,
            jitter_ns: 0,
            receiver_tick_ns: TICK,
            source: source(),
        });
    }
    Topology {
        required_modes: vec!["рабочий".into()],
        paths: vec![Path {
            id: PATH.into(),
            drive: DRIVE.into(),
            observation: signal(OBS, "m"),
            command: signal(COMMAND, "N"),
            accepted: signal(ACCEPTED, "N"),
            physical_response: vec![signal(POSITION, "m"), signal(ACCEL, "m/s^2")],
            controller: CONTROLLER.into(),
            program: inputs().program,
            // Для ступени 1 м и усиления 2 Н/м разница не менее 1 Н,
            // возраст отсчёта допускает такт планировщика.
            command_delta: 1.0,
            max_observation_age_ns: TICK,
            source: source(),
            nodes,
            hops,
            legacy: None,
        }],
        limitation_paths: Default::default(),
    }
}

pub fn inputs() -> Inputs {
    Inputs {
        build: env!("CARGO_PKG_VERSION").into(),
        catalog: Fingerprint::of(b"analytical model"),
        wiring: Fingerprint::of(b"feedback and actuator"),
        program: Fingerprint::of(b"F = 2 * (target - position)"),
        configuration: Fingerprint::of(b"1 kg, dt = 1 ms"),
        firmware_image: None,
        legacy_design: None,
    }
}

pub fn document(max: f64, reaction_ns: u64, radio: bool) -> String {
    let mut faults: Vec<_> = FAULTS.iter().map(|&(node, _, kind)| (node, kind)).collect();
    if radio {
        faults.extend([(RADIO, "packet_loss"), (RADIO, "packet_delay")]);
    }
    let mut text = format!(
        "version=1\ncontrol='control.toml'\n[[modes]]\nid='рабочий'\nsource={SOURCE}\n\
         horizon_ns={HORIZON}\nbefore_ns={}\nafter_ns={}\nmax_gap_ns={TICK}\nexcitation='ступень'\n\
         stimuli=[{{id='ступень',signal={{name='цель',unit='m'}},at_ns={EXCITE},value=1.0,comparison_value=0.0,source={SOURCE}}}]\n",
        2 * TICK,
        4 * TICK
    );
    for &(node, kind) in &faults {
        let delay = if kind == "packet_delay" {
            format!("extra_delay_ns={}\n", 2 * TICK)
        } else {
            String::new()
        };
        text.push_str(&format!("[[modes.faults]]\nfault={{node='{node}',kind='{kind}'}}\nat_ns={FAULT}\n{delay}source={SOURCE}\n"));
    }
    for fault in std::iter::once(None).chain(faults.into_iter().map(Some)) {
        let (reaction, unit, change, injection) = match fault {
            // Ускорение меняется с 2 м/с² до нуля при отключении тракта.
            Some((node, kind)) => (
                ACCEL,
                "m/s^2",
                1.0,
                format!(",fault={{node='{node}',kind='{kind}'}}"),
            ),
            // 2 Н дают 2 мкм на первом шаге, порог обнаружения 1 мкм.
            None => (POSITION, "m", 0.000_001, String::new()),
        };
        text.push_str(&format!(
            "[[criteria]]\ncase={{path='{PATH}',mode='рабочий'{injection}}}\n\
             limits=[{{kind='range',signal={{name='{POSITION}',unit='m'}},min=0.0,max={max:?},source={SOURCE}}},\
             {{kind='forbidden',signal={{name='{EVENT}',unit='1'}},source={SOURCE}}}]\n\
             response={{signal={{name='{reaction}',unit='{unit}'}},min_change={change:?},within_ns={reaction_ns},source={SOURCE}}}\n"
        ));
    }
    text
}

pub fn assessor(max: f64, reaction_ns: u64, radio: bool) -> Assessor {
    Assessor::new(
        Task::parse(&document(max, reaction_ns, radio)).unwrap(),
        topology(radio),
        world(PART),
        inputs(),
    )
    .unwrap()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Trial {
    Normal,
    Constant,
    Scripted,
    ScriptedUntilExcitation,
    MissingAck,
    NoFaultEffect,
    Blocked,
    Incomplete,
    Forbidden,
    NoCommandAcceptance,
    InvalidSample,
    WrongController,
    WrongFeedback,
    StaleCapture,
    External,
    ClaimedRadioWithoutTraffic,
    LateRadio,
    NonFinite,
}

fn fault_name(node: &str) -> String {
    format!("{node}/отказ")
}

fn fault_at(plan: &RunPlan<'_>) -> Option<(Fault, Time)> {
    plan.fault
        .map(|injection| (injection.fault.clone(), Time(injection.at_ns)))
}

pub fn execute(assessor: &Assessor, case: &Case, trial: Trial) -> RunResult {
    assessor
        .execute(case, |runtime, plan, events| {
            install(runtime, plan, events, trial).map(|()| inputs())
        })
        .unwrap()
}

pub fn install(
    runtime: &mut Runtime,
    plan: &RunPlan<'_>,
    events: &EventSink,
    trial: Trial,
) -> Result<(), String> {
    install_with(runtime, plan, events, trial, inputs().program, |sample| {
        2.0 * sample
    })
}

pub fn install_with(
    runtime: &mut Runtime,
    plan: &RunPlan<'_>,
    events: &EventSink,
    trial: Trial,
    program: Fingerprint,
    mut controller: impl FnMut(f64) -> f64 + Clone + 'static,
) -> Result<(), String> {
    if trial == Trial::Blocked {
        runtime.add_simplification(Simplification {
            subject: "контакт".into(),
            text: "неопределённые условия на границе".into(),
            blocks_proof: true,
        })?;
    }
    let rate = runtime.physics_rate();
    let observation = runtime.register_observation(OBS, &plan.path.observation.unit)?;
    let accepted = runtime.register_truth(ACCEPTED, &plan.path.accepted.unit)?;
    let position = runtime.register_truth(POSITION, &plan.path.physical_response[0].unit)?;
    let accel = runtime.register_truth(ACCEL, &plan.path.physical_response[1].unit)?;
    let forbidden = runtime.register_truth(EVENT, "1")?;
    let channel = runtime.channel(COMMAND).unwrap();
    let shared = Rc::new(RefCell::new(Local {
        trial,
        target: plan.stimulus_value(&plan.mode.stimuli[0]),
        stimulus_signal: plan.mode.stimuli[0].signal.clone(),
        fault: fault_at(plan),
        active: None,
        last_capture: None,
        pending: None,
        command: None,
        sink: events.clone(),
        path: plan.path.id.clone(),
        program,
        radio: !plan.path.hops.is_empty(),
        held: None,
    }));
    let states = FAULTS
        .iter()
        .map(|&(node, _, _)| {
            runtime
                .register_truth(&fault_name(node), "1")
                .map(|id| (node, id))
        })
        .collect::<Result<Vec<_>, _>>()?;
    runtime.register_sensor(
        "измеритель",
        rate,
        Meter {
            shared: shared.clone(),
            observation,
        },
    )?;
    let controller_shares = shared.clone();
    runtime.register_controller(CONTROLLER, rate, move |input: &Observation| {
        regulate(
            &mut controller_shares.borrow_mut(),
            input,
            observation,
            channel,
            &mut controller,
        )
    })?;
    runtime.register_object(
        "источник силы",
        rate,
        &[channel],
        Motor {
            shared: shared.clone(),
            channel,
            accepted,
        },
    )?;
    runtime.register_evaluator(
        "физические метрики",
        rate,
        Record {
            position,
            accel,
            forbidden,
            prev_v: 0.0,
            states,
            shared,
            trial,
        },
    )?;
    if trial == Trial::External {
        runtime.act(&Command {
            targets: vec![(channel, 1.0)],
        })?;
    }
    Ok(())
}

struct Local {
    trial: Trial,
    target: f64,
    stimulus_signal: Signal,
    fault: Option<(Fault, Time)>,
    active: Option<FaultKind>,
    pending: Option<Cycle>,
    command: Option<Pending>,
    last_capture: Option<Sample>,
    sink: EventSink,
    path: String,
    program: Fingerprint,
    radio: bool,
    held: Option<(Time, Sample)>,
}

struct Pending {
    observed: Sample,
    captured: Time,
    output: Sample,
}

fn write(sink: &EventSink, event: Event) -> Result<(), String> {
    sink.record(event).map_err(|error| error.to_string())
}

#[derive(Clone)]
struct Meter {
    shared: Rc<RefCell<Local>>,
    observation: SignalId,
}

impl Sensor for Meter {
    fn sample(
        &mut self,
        truth: &Truth<'_>,
        _: &mut Rng,
        out: &mut ObservationOut,
    ) -> Result<(), String> {
        let mut local = self.shared.borrow_mut();
        let at = truth.time();
        let (q, _) = truth.joint(JOINT).unwrap();
        let target = if at.0 >= EXCITE { local.target } else { 0.0 };
        if at.0 == EXCITE {
            write(
                &local.sink,
                Event::Stimulus(StimulusReceipt {
                    id: "ступень".into(),
                    signal: local.stimulus_signal.clone(),
                    requested: at,
                    applied: at,
                    value: target,
                }),
            )?;
        }
        if let Some((fault, requested)) = local.fault.clone()
            && at == requested
        {
            if local.trial != Trial::NoFaultEffect {
                local.active = Some(fault.kind);
            }
            if local.trial != Trial::MissingAck {
                write(
                    &local.sink,
                    Event::Fault(FaultReceipt {
                        fault,
                        requested,
                        applied: at,
                    }),
                )?;
            }
        }
        let mut captured = Sample {
            at,
            value: target - q[0],
        };
        let mut delivery = Some(at);
        let active = local.active;
        if local.radio && local.trial != Trial::ClaimedRadioWithoutTraffic {
            let mut transfer_at = at;
            let mut outcome = Delivery::Received(at);
            match active {
                Some(FaultKind::PacketLoss) => {
                    outcome = Delivery::Lost(at);
                    delivery = None;
                }
                Some(FaultKind::PacketDelay) => {
                    let deadline = Time(at.0 + 2 * TICK);
                    if let Some((due, sample)) = local.held.filter(|(due, _)| *due == at) {
                        local.held = None;
                        transfer_at = sample.at;
                        captured = sample;
                        outcome = Delivery::Received(due);
                    } else if local.held.is_none() && deadline.0 <= HORIZON {
                        local.held = Some((deadline, captured));
                        delivery = None;
                    } else {
                        delivery = None;
                    }
                }
                _ if local.trial == Trial::LateRadio && at.0 == EXCITE => {
                    // Этот тестовый пакет отмечен просроченным в тесте расчёта границы.
                    outcome = Delivery::Received(Time(at.0 + 2 * TICK));
                    delivery = None;
                }
                _ => {}
            }
            if delivery.is_some()
                || matches!(outcome, Delivery::Lost(_))
                || local.trial == Trial::LateRadio
            {
                write(
                    &local.sink,
                    Event::Transfer(Transfer {
                        path: local.path.clone(),
                        hop: RADIO.into(),
                        sequence: transfer_at.0 / TICK,
                        sent: transfer_at,
                        outcome,
                    }),
                )?;
            }
        }
        let valid = delivery.is_some()
            && !matches!(
                active,
                Some(FaultKind::SensorUnavailable | FaultKind::WireOpen)
            )
            && !(local.trial == Trial::InvalidSample && at.0 >= EXCITE);
        if local.trial == Trial::NonFinite && at.0 == EXCITE {
            captured.value = f64::NAN;
        }
        out.write(self.observation, captured.value, valid);
        local.last_capture = Some(captured);
        if let Some(mut cycle) = local
            .pending
            .take()
            .filter(|_| valid && captured.value.is_finite())
        {
            cycle.feedback = Sample {
                at,
                value: captured.value,
            };
            if local.trial == Trial::WrongFeedback {
                cycle.feedback.value += 1.0;
            }
            write(&local.sink, Event::Cycle(cycle))?;
        }
        Ok(())
    }
}

fn regulate(
    local: &mut Local,
    input: &Observation,
    signal: SignalId,
    channel: ChannelId,
    controller: &mut impl FnMut(f64) -> f64,
) -> Command {
    let Some(sample) = input
        .samples
        .iter()
        .find(|sample| sample.signal == signal && sample.at == input.time && sample.valid)
        .filter(|_| local.active != Some(FaultKind::ControllerHalted))
    else {
        local.command = None;
        return Command::default();
    };
    let value = match local.trial {
        Trial::Constant => 2.0,
        Trial::Scripted | Trial::ScriptedUntilExcitation
            if local.trial == Trial::Scripted || input.time.0 <= EXCITE =>
        {
            2.0 * f64::from(input.time.0 >= EXCITE)
        }
        _ => controller(sample.value),
    };
    local.command = Some(Pending {
        observed: Sample {
            at: sample.at,
            value: sample.value,
        },
        captured: local.last_capture.unwrap().at,
        output: Sample {
            at: input.time,
            value,
        },
    });
    Command {
        targets: vec![(channel, value)],
    }
}

#[derive(Clone)]
struct Motor {
    shared: Rc<RefCell<Local>>,
    channel: ChannelId,
    accepted: TruthId,
}

impl Object for Motor {
    fn update(
        &mut self,
        truth: &Truth<'_>,
        command: &Command,
        _: &mut Rng,
        out: &mut Forces,
    ) -> Result<(), String> {
        let mut local = self.shared.borrow_mut();
        let pending = local.command.take();
        let at = truth.time();
        let enabled = !matches!(
            local.active,
            Some(FaultKind::DriverDisabled | FaultKind::PowerCut | FaultKind::DriveDisabled)
        );
        if let Some(pending) = pending.filter(|_| enabled) {
            let value = command
                .targets
                .iter()
                .find(|(id, _)| *id == self.channel)
                .unwrap()
                .1;
            out.set(self.channel, value);
            if local.trial != Trial::NoCommandAcceptance {
                out.publish(self.accepted, value);
            }
            local.pending = Some(Cycle {
                path: local.path.clone(),
                sequence: pending.captured.0 / TICK,
                observation: pending.observed,
                captured_at: if local.trial == Trial::StaleCapture {
                    Time::ZERO
                } else {
                    pending.captured
                },
                read_at: at,
                controller_name: CONTROLLER.into(),
                program: local.program,
                // Измеритель зарегистрирован первым, контроллер вторым.
                controller: if local.trial == Trial::WrongController {
                    0
                } else {
                    1
                },
                command: pending.output,
                accepted: Sample { at, value },
                feedback: Sample { at, value },
            });
        }
        Ok(())
    }
}

#[derive(Clone)]
struct Record {
    position: TruthId,
    accel: TruthId,
    forbidden: TruthId,
    prev_v: f64,
    states: Vec<(&'static str, TruthId)>,
    shared: Rc<RefCell<Local>>,
    trial: Trial,
}

impl Evaluator for Record {
    fn evaluate(
        &mut self,
        truth: &Truth<'_>,
        _: &mut Rng,
        out: &mut TruthOut,
    ) -> Result<(), String> {
        let at = truth.time();
        if self.trial == Trial::Incomplete && at.0 == 6 * TICK {
            return Ok(());
        }
        let (q, v) = truth.joint(JOINT).unwrap();
        out.write(self.position, q[0]);
        out.write(self.accel, (v[0] - self.prev_v) / DT);
        out.write(
            self.forbidden,
            f64::from(self.trial == Trial::Forbidden && at.0 == 5 * TICK),
        );
        let local = self.shared.borrow();
        for &(node, state) in &self.states {
            let active = local.active.is_some()
                && local
                    .fault
                    .as_ref()
                    .is_some_and(|(fault, _)| fault.node == node);
            out.write(state, f64::from(active));
        }
        self.prev_v = v[0];
        Ok(())
    }
}

/// Независимое решение по явным уравнениям для шага Euler MuJoCo:
/// v[k+1] = v[k] + F[k] dt/m, q[k+1] = q[k] + v[k+1] dt.
pub fn reference_nominal() -> Vec<f64> {
    let mut positions = vec![0.0];
    let (mut q, mut v) = (0.0, 0.0);
    for k in 0..HORIZON / TICK {
        let target = f64::from(k * TICK >= EXCITE);
        v += 2.0 * (target - q) * DT;
        q += v * DT;
        positions.push(q);
    }
    positions
}
