//! Перевод типизированного журнала установленного контура в события оценщика.

use std::collections::BTreeMap;

use sim_runtime::control::{
    ApplicationTiming, CommandOrigin, ControlRun, Event as RuntimeEvent, EventKind, FaultAction,
    RunOrigin,
};
use sim_signal::Time;

use crate::{
    Cycle, Delivery, Event, FaultKind, FaultReceipt, Fingerprint, RunPlan, Sample, StimulusReceipt,
    Transfer,
};

fn planned_fault(event: &RuntimeEvent, plan: &RunPlan<'_>) -> bool {
    let (
        EventKind::FaultRequested {
            id,
            target,
            action,
            active: true,
        },
        Some(injection),
    ) = (&event.kind, plan.fault)
    else {
        return false;
    };
    let expected = match injection.fault.kind {
        FaultKind::SensorUnavailable
        | FaultKind::WireOpen
        | FaultKind::DriverDisabled
        | FaultKind::DriveDisabled
        | FaultKind::PacketLoss => Some(FaultAction::Drop),
        FaultKind::PacketDelay => injection.extra_delay_ns.map(FaultAction::Delay),
        FaultKind::ControllerHalted => Some(FaultAction::Pause),
        FaultKind::PowerCut => Some(FaultAction::PowerOff),
    };
    id == "task"
        && target == &injection.fault.node
        && event.at == Time(injection.at_ns)
        && Some(*action) == expected
}

pub(crate) fn unplanned_fault(event: &RuntimeEvent, plan: &RunPlan<'_>) -> bool {
    matches!(event.kind, EventKind::FaultRequested { .. }) && !planned_fault(event, plan)
}

pub(crate) fn events(run: &ControlRun<'_>, plan: &RunPlan<'_>) -> Vec<Event> {
    let Some(path) = run.paths().iter().find(|path| {
        path.name == plan.path.id
            && path.producer == plan.path.controller
            && Fingerprint::from_fnv1a(path.program_hash) == plan.path.program
            && path.target == plan.path.command.name
            && path.unit == plan.path.command.unit
    }) else {
        return Vec::new();
    };
    if run.origin() != RunOrigin::Live {
        return Vec::new();
    }
    let Some(&signal) = path.feedback.iter().find(|id| {
        let spec = &run.log().header().observations[id.0 as usize];
        spec.name == plan.path.observation.name && spec.unit == plan.path.observation.unit
    }) else {
        return Vec::new();
    };
    let mut result = Vec::new();
    let mut reads = BTreeMap::new();
    let mut requests = BTreeMap::new();
    let mut payloads = BTreeMap::new();
    let mut measurements = Vec::new();
    let mut transfers = BTreeMap::new();
    let mut requested_fault = None;
    for event in run.events() {
        let (id, sent, hop) = match &event.kind {
            EventKind::Measurement {
                signal: key,
                time,
                valid: true,
                value,
            } if *key == signal => {
                measurements.push((
                    time.captured_at,
                    Sample {
                        at: time.delivered_at,
                        value: *value,
                    },
                ));
                continue;
            }
            EventKind::ControllerInput {
                controller,
                signal: key,
                time,
                valid: true,
                value,
            } if controller == &path.producer && *key == signal => {
                reads.insert(event.at, (*time, *value));
                continue;
            }
            EventKind::RequestInput {
                controller,
                id,
                signal: key,
                captured_at,
                value,
            } if controller == &path.producer && *key == signal => {
                payloads.insert(*id, (*captured_at, *value));
                continue;
            }
            EventKind::CommandApplied {
                path: name,
                channel,
                command,
                effective_at,
                timing: ApplicationTiming::PhysicsInterval,
                ..
            } if command.origin == CommandOrigin::Stimulus => {
                let spec = &run.log().header().commands[channel.0 as usize];
                let signal = crate::Signal {
                    name: spec.name.clone(),
                    unit: spec.unit.clone(),
                };
                let id = plan
                    .mode
                    .stimuli
                    .iter()
                    .find(|stimulus| {
                        stimulus.signal == signal && command.sent_at == Time(stimulus.at_ns)
                    })
                    .map_or_else(
                        || format!("{} @ {} нс", name, command.sent_at.0),
                        |stimulus| stimulus.id.clone(),
                    );
                result.push(Event::Stimulus(StimulusReceipt {
                    id,
                    signal,
                    requested: command.sent_at,
                    applied: *effective_at,
                    value: command.value,
                }));
                continue;
            }
            EventKind::FaultRequested { .. } if planned_fault(event, plan) => {
                requested_fault = Some(event.at);
                continue;
            }
            EventKind::FaultApplied {
                id,
                target,
                active: true,
            } if id == "task" => {
                if let (Some(requested), Some(injection)) = (
                    requested_fault,
                    plan.fault.filter(|item| item.fault.node == *target),
                ) {
                    result.push(Event::Fault(FaultReceipt {
                        fault: injection.fault.clone(),
                        requested,
                        applied: event.at,
                    }));
                }
                continue;
            }
            EventKind::RequestDelivered {
                controller,
                id,
                sent_at,
                ..
            } if controller == &path.producer => {
                requests.insert(*sent_at, (*id, event.at));
                (*id, *sent_at, format!("transport:{controller}/request"))
            }
            EventKind::ResponseDelivered {
                controller,
                id,
                sent_at,
            } if controller == &path.producer => {
                (*id, *sent_at, format!("transport:{controller}/response"))
            }
            EventKind::PacketDelivered {
                route, id, sent_at, ..
            } => (*id, *sent_at, format!("channel:{route}")),
            EventKind::Packet {
                route,
                event: packet,
            } if matches!(
                packet.outcome,
                sim_runtime::control::PacketOutcome::Lost
                    | sim_runtime::control::PacketOutcome::Disconnected
                    | sim_runtime::control::PacketOutcome::Overflow
            ) =>
            {
                let hop = format!("channel:{route}");
                if plan.path.hops.iter().any(|candidate| candidate.id == hop) {
                    result.push(Event::Transfer(Transfer {
                        path: plan.path.id.clone(),
                        hop,
                        sequence: packet.id,
                        sent: packet.sent_at,
                        outcome: Delivery::Lost(event.at),
                    }));
                }
                continue;
            }
            _ => continue,
        };
        if plan.path.hops.iter().any(|candidate| candidate.id == hop) {
            transfers.insert((hop.clone(), id), (sent, event.at));
            result.push(Event::Transfer(Transfer {
                path: plan.path.id.clone(),
                hop,
                sequence: id,
                sent,
                outcome: Delivery::Received(event.at),
            }));
        }
    }
    let uses_process = plan
        .path
        .hops
        .iter()
        .any(|hop| hop.id.starts_with("transport:"));
    for event in run.events() {
        let EventKind::CommandApplied {
            path: name,
            command,
            effective_at,
            timing: ApplicationTiming::ModelAcceptance,
            ..
        } = &event.kind
        else {
            continue;
        };
        if *name != path.name
            || !matches!(
                command.origin,
                CommandOrigin::Board | CommandOrigin::Controller
            )
        {
            continue;
        }
        let Some(observed_at) = command.observed_at else {
            continue;
        };
        let Some(&(time, value)) = reads.get(&observed_at) else {
            continue;
        };
        let mut read_at = observed_at;
        if uses_process {
            let Some(&(id, delivered_at)) = requests.get(&observed_at) else {
                continue;
            };
            if !payloads.get(&id).is_some_and(|&(captured, payload)| {
                captured == time.captured_at && payload.to_bits() == value.to_bits()
            }) || !transfers
                .get(&(format!("transport:{}/response", path.producer), id))
                .is_some_and(|&(_, at)| at == command.delivered_at)
            {
                continue;
            }
            read_at = delivered_at;
        }
        let first = measurements.partition_point(|(_, sample)| sample.at <= *effective_at);
        let Some((_, feedback)) = measurements[first..]
            .iter()
            .find(|(captured, _)| *captured > *effective_at)
        else {
            continue;
        };
        result.push(Event::Cycle(Cycle {
            path: path.name.clone(),
            sequence: command.sequence,
            observation: Sample {
                at: time.delivered_at,
                value,
            },
            captured_at: time.captured_at,
            read_at,
            controller_name: path.producer.clone(),
            program: plan.path.program,
            controller: path.producer_index,
            command: Sample {
                at: command.delivered_at,
                value: command.value,
            },
            accepted: Sample {
                at: *effective_at,
                value: command.value,
            },
            feedback: *feedback,
        }));
    }
    result
}

pub(crate) fn publications(run: &ControlRun<'_>, name: &str) -> BTreeMap<u64, Time> {
    run.events()
        .iter()
        .filter_map(|event| match &event.kind {
            EventKind::CommandApplied {
                path,
                command,
                timing: ApplicationTiming::ModelAcceptance,
                ..
            } if path == name
                && matches!(
                    command.origin,
                    CommandOrigin::Board | CommandOrigin::Controller
                ) =>
            {
                Some((command.sequence, event.at))
            }
            _ => None,
        })
        .collect()
}

pub(crate) fn activity(run: &ControlRun<'_>, plan: &RunPlan<'_>) -> crate::ControlActivity {
    let mut result = crate::ControlActivity::default();
    let mut previous = None;
    for event in run.events() {
        match &event.kind {
            EventKind::ControllerInput {
                controller,
                valid: true,
                ..
            } if controller == &plan.path.controller => {
                result.valid_reads += 1;
            }
            EventKind::CommandApplied {
                path,
                command,
                timing: ApplicationTiming::ModelAcceptance,
                ..
            } if path == &plan.path.id
                && matches!(
                    command.origin,
                    CommandOrigin::Board | CommandOrigin::Controller
                ) =>
            {
                result.model_acceptances += 1;
                let old = previous.replace(command.value.to_bits());
                result.command_changes +=
                    usize::from(old.is_some_and(|old| old != command.value.to_bits()));
            }
            EventKind::OutputInvalidated {
                path,
                reason: sim_runtime::control::Rejection::Expired,
            } if path == &plan.path.id => result.expired_outputs += 1,
            EventKind::FaultApplied {
                target,
                active: true,
                ..
            } => {
                result
                    .applied_faults
                    .entry(target.clone())
                    .or_insert(event.at.0);
            }
            EventKind::PacketDelivered { route, .. } => {
                *result.packet_deliveries.entry(route.clone()).or_default() += 1;
                result.last_packet_ns.insert(route.clone(), event.at.0);
            }
            EventKind::BoardOutput {
                board,
                event: output,
                produced_at,
            } if board == &plan.path.controller => {
                if let sim_runtime::control::BoardEvent::Reset(cause) = output.output {
                    result.resets.push((produced_at.0, format!("{cause:?}")));
                }
            }
            EventKind::BoardActionApplied { id, board, .. } if board == &plan.path.controller => {
                result.actions.insert(id.clone(), event.at.0);
            }
            _ => {}
        }
    }
    result
}
