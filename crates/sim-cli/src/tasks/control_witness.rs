//! Предметные проверки живого контура на заданных окнах истины.

use serde::{Deserialize, Serialize};
use sim_runtime::{
    LogKind, Runtime,
    control::{self, CommandOrigin, EventKind, RunOrigin},
};
use sim_task::{
    Case, CaseReport, FaultKind, Outcome, RunPlan, Signal, Source, SourceKind,
    verdict::{HazardClass, REGISTRY},
};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Spec {
    pub case: Case,
    #[serde(deserialize_with = "class")]
    pub class: HazardClass,
    pub object: String,
    pub window_ns: [u64; 2],
    pub max_gap_ns: u64,
    pub source: Source,
    pub scope: String,
    pub check: Check,
}

fn class<'de, D: serde::Deserializer<'de>>(d: D) -> Result<HazardClass, D::Error> {
    let id = String::deserialize(d)?;
    REGISTRY.iter().find(|entry| entry.id == id).map(|entry| entry.class)
        .ok_or_else(|| serde::de::Error::custom(format!("неизвестный класс «{id}»")))
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub(super) enum Check {
    Truth {
        signal: Signal,
        min: Option<f64>,
        max: Option<f64>,
    },
    Program {
        approved_digest: String,
    },
}

#[derive(Deserialize, Serialize)]
pub(super) struct Observation {
    pub spec: usize,
    pub measured: [f64; 2],
    pub at_ns: u64,
    pub outcome: Outcome,
}

fn fault_class(kind: FaultKind) -> HazardClass {
    match kind {
        FaultKind::SensorUnavailable => HazardClass::SensorFault,
        FaultKind::WireOpen => HazardClass::WireFault,
        FaultKind::PacketDelay | FaultKind::PacketLoss => HazardClass::LinkFault,
        FaultKind::ControllerHalted => HazardClass::ControllerHalt,
        FaultKind::DriverDisabled | FaultKind::DriveDisabled | FaultKind::PowerCut => {
            HazardClass::OutputFault
        }
    }
}

impl Spec {
    pub fn validate(&self, assessor: &sim_task::Assessor) -> Result<(), String> {
        let Some(plan) = assessor.plan(&self.case) else {
            return Err("свидетель контура ссылается на неизвестный опыт".into());
        };
        let [start, end] = self.window_ns;
        if self.object.trim().is_empty()
            || self.scope.trim().is_empty()
            || self.source.reference.trim().is_empty()
            || matches!(self.source.kind, SourceKind::Estimate | SourceKind::Derived)
            || start >= end
            || end > plan.mode.horizon_ns
            || self.max_gap_ns == 0
            || plan.fault.is_some_and(|f| start <= f.at_ns)
        {
            return Err("неприменимый источник или окно свидетеля контура".into());
        }
        let valid = match &self.check {
            Check::Truth { signal, min, max } => {
                let (drive, has_port) = plan
                    .path
                    .accepted
                    .name
                    .rsplit_once('/')
                    .map_or(("", false), |(drive, _)| {
                        (drive, signal.name.starts_with(&format!("{drive}/")))
                    });
                let quantity = signal.name.rsplit_once('/').map(|(_, name)| name);
                let electrical = matches!(
                    (quantity, signal.unit.as_str()),
                    (Some("current_a"), "A")
                        | (Some("bridge_enabled"), "bool")
                        | (Some("voltage_v"), "V")
                        | (Some("accepted_command"), "1")
                );
                let class_matches = match self.class {
                    HazardClass::DriveOverload => {
                        plan.fault.is_none()
                            && self.object == drive
                            && matches!((quantity, signal.unit.as_str()),
                                (Some("current_a"), "A") |
                                (Some("winding_c"), "degC") |
                                (Some("joint_torque_nm"), "N*m"))
                    }
                    HazardClass::Battery => {
                        plan.fault.is_none()
                            && self.object == drive
                            && quantity == Some("bus_next_v")
                            && signal.unit == "V"
                    }
                    HazardClass::EmergencyStop => plan.fault.is_some_and(|f| {
                        f.fault.kind == FaultKind::PowerCut && self.object == f.fault.node
                            && matches!((quantity, signal.unit.as_str()),
                                (Some("current_a"), "A") | (Some("bridge_enabled"), "bool"))
                    }),
                    _ => plan.fault.is_some_and(|f| {
                        fault_class(f.fault.kind) == self.class
                            && self.object == f.fault.node
                            && electrical
                    }),
                };
                has_port
                    && class_matches
                    && (min.is_some() || max.is_some())
                    && min.iter().chain(max).all(|value| value.is_finite())
                    && min.zip(*max).is_none_or(|(lo, hi)| lo <= hi)
            }
            Check::Program { approved_digest } => {
                self.class == HazardClass::ProgramIdentity
                    && plan.fault.is_none()
                    && self.object == plan.path.controller
                    && approved_digest.len() == 16
                    && u64::from_str_radix(approved_digest, 16).is_ok()
            }
        };
        if !valid {
            return Err(format!("несогласованные класс, объект или ряд «{}»", self.class.spec().id));
        }
        Ok(())
    }

    pub fn bounds(&self) -> (Option<f64>, Option<f64>) {
        match self.check {
            Check::Truth { min, max, .. } => (min, max),
            // Несовпадение запрещено для любого утверждённого образа.
            Check::Program { .. } => (None, Some(0.0)),
        }
    }

    pub fn unit(&self) -> &str {
        match &self.check {
            Check::Truth { signal, .. } => &signal.unit,
            Check::Program { .. } => "1",
        }
    }

    pub fn reason(&self) -> &'static str {
        match self.check {
            Check::Program { .. } => "отпечаток действующего образа на исполнительном пути",
            Check::Truth { .. } if self.case.fault.is_some() =>
                "предел исполнительной модели после подтверждённой инъекции",
            Check::Truth { .. } => "предел исполнительной модели в рабочем режиме",
        }
    }
}

pub(super) fn collect(
    specs: &[Spec],
    plan: &RunPlan<'_>,
    rt: &Runtime,
) -> Result<Vec<Observation>, String> {
    if !specs.iter().any(|spec| &spec.case == plan.case) {
        return Ok(Vec::new());
    }
    let run = control::run(rt).ok_or("нет живого контура для предметной проверки")?;
    if run.origin() != RunOrigin::Live || run.end().time.0 != plan.mode.horizon_ns {
        return Ok(Vec::new());
    }
    let path = run.paths().iter().find(|path| path.name == plan.path.id)
        .ok_or("нет исполнительного пути свидетеля")?;
    let mut results = Vec::new();
    for (index, spec) in specs.iter().enumerate().filter(|(_, s)| &s.case == plan.case) {
        let [start, end] = spec.window_ns;
        if spec.class == HazardClass::LinkFault {
            let fault = plan.fault.expect("проверенный отказ связи");
            let disconnected = run.events().iter().any(|event| {
                (fault.at_ns..=end).contains(&event.at.0)
                    && matches!(&event.kind, EventKind::Rejected {
                        path, reason: control::Rejection::Disconnected, ..
                    } if path == &fault.fault.node)
            });
            if fault.fault.kind != FaultKind::PacketLoss || !disconnected {
                continue;
            }
        }
        let applied = run.events().iter().filter_map(|event| match &event.kind {
            EventKind::CommandApplied { path: name, command, effective_at, .. }
                if name == &path.name && command.observed_at.is_some()
                    && matches!(command.origin, CommandOrigin::Controller | CommandOrigin::Board) =>
                    Some(effective_at.0),
            _ => None,
        });
        let Some((measured, min_at, max_at)) = (match &spec.check {
            Check::Truth { signal, .. } => {
                if let Some(fault) = plan.fault {
                    if !applied.clone().any(|at| at < fault.at_ns) {
                        continue;
                    }
                }
                if spec.class == HazardClass::EmergencyStop {
                    let Some((drive, _)) = signal.name.rsplit_once('/') else { continue };
                    let id = rt.log().header().truth.iter().position(|s| {
                        s.name == format!("{drive}/bridge_enabled") && s.unit == "bool"
                    });
                    let Some(id) = id else { continue };
                    let bridge = rt.log().entries().iter().filter(|e| {
                        e.kind == LogKind::Truth && e.id as usize == id
                            && (start..=end).contains(&e.at.0)
                    });
                    if bridge.clone().next().is_none()
                        || !bridge.into_iter().all(|e| e.valid && e.value == 0.0)
                    {
                        continue;
                    }
                }
                let Some(id) = rt.log().header().truth.iter().position(|item| {
                    item.name == signal.name && item.unit == signal.unit
                }) else { continue };
                let mut entries = rt.log().entries().iter().filter(|e| {
                    e.kind == LogKind::Truth && e.id as usize == id
                        && (start..=end).contains(&e.at.0)
                });
                let Some(first) = entries.next() else { continue };
                let (mut low, mut high, mut last) = (first, first, first);
                let mut complete = first.valid && first.at.0 - start <= spec.max_gap_ns;
                for entry in entries {
                    if !entry.value.is_finite() {
                        return Err("неконечный исполнительный отсчёт".into());
                    }
                    if entry.at.0.saturating_sub(last.at.0) > spec.max_gap_ns || !entry.valid {
                        complete = false;
                    }
                    if entry.value < low.value { low = entry; }
                    if entry.value > high.value { high = entry; }
                    last = entry;
                }
                if !low.value.is_finite() || !high.value.is_finite() {
                    return Err("неконечный исполнительный отсчёт".into());
                }
                (complete && first.at < last.at && end - last.at.0 <= spec.max_gap_ns)
                    .then_some(([low.value, high.value], low.at.0, high.at.0))
            }
            Check::Program { approved_digest } => {
                let approved = u64::from_str_radix(approved_digest, 16)
                    .map_err(|e| e.to_string())?;
                applied.into_iter().find(|at| (start..=end).contains(at)).map(|at| {
                    ([(u8::from(path.program_hash != approved)) as f64; 2], at, at)
                })
            }
        }) else { continue };
        let (min, max) = spec.bounds();
        let (outcome, at_ns) = if min.is_some_and(|v| measured[0] < v) {
            (Outcome::Fail, min_at)
        } else if max.is_some_and(|v| measured[1] > v) {
            (Outcome::Fail, max_at)
        } else {
            (Outcome::Pass, max_at)
        };
        results.push(Observation { spec: index, measured, at_ns, outcome });
    }
    Ok(results)
}

pub(super) fn valid_report(spec: &Spec, report: &CaseReport) -> bool {
    report.was_executed() && report.journal.is_some() && (spec.case.fault.is_none()
        || report.applied_ns.is_some_and(|at| at < spec.window_ns[0]))
}
