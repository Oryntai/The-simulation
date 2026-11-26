use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use sim_runtime::{CommandSource, Log, LogEntry, LogKind, Runtime, Status};
use sim_signal::{SignalSpec, Time};
use sim_world::{CompiledWorld, Provenance};

use crate::live::{Cycle, Delivery, Event, Sample, Transfer};
use crate::task::finite;
use crate::{
    Case, Criteria, Direction, Error, FaultKind, Fingerprint, Hop, Limit, LiveRun, Mode, Response,
    RunPlan, Signal, Topology, invalid,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Outcome {
    Pass,
    NotAnalyzed,
    Fail,
}

impl Outcome {
    pub fn aggregate(iter: impl IntoIterator<Item = Self>) -> Self {
        iter.into_iter().max().unwrap_or(Self::NotAnalyzed)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Metric {
    pub signal: Signal,
    pub limit: Limit,
    pub min: f64,
    pub max: f64,
    #[serde(default)]
    pub min_at_ns: Option<u64>,
    #[serde(default)]
    pub max_at_ns: Option<u64>,
    pub last: f64,
    pub rms: f64,
    pub first_ns: u64,
    pub last_ns: u64,
    pub samples: usize,
    /// Окно с подтверждённой непрерывностью дискретных наблюдений.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub covered_window: Option<crate::campaign::Window>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LimitViolation {
    pub signal: Signal,
    pub value: f64,
    pub at_ns: u64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Reaction {
    pub criterion: Response,
    pub requested_ns: u64,
    pub applied_ns: u64,
    pub reaction_ns: u64,
    pub change: f64,
}

/// Фактически подтверждённая последовательность на одном пути.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LoopTiming {
    pub sequence: u64,
    pub captured_ns: u64,
    pub delivered_ns: u64,
    pub read_ns: u64,
    pub command_ns: u64,
    pub accepted_ns: u64,
    pub feedback_ns: u64,
}

impl From<&Cycle> for LoopTiming {
    fn from(cycle: &Cycle) -> Self {
        Self {
            sequence: cycle.sequence,
            captured_ns: cycle.captured_at.0,
            delivered_ns: cycle.observation.at.0,
            read_ns: cycle.read_at.0,
            command_ns: cycle.command.at.0,
            accepted_ns: cycle.accepted.at.0,
            feedback_ns: cycle.feedback.at.0,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Comparison {
    pub journal: u64,
    pub events: Fingerprint,
    pub at_ns: u64,
    pub original_command: f64,
    pub comparison_command: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ComparisonAudit {
    pub journal: u64,
    pub changed_observations: usize,
    pub command_stream_equal: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ComparisonExecution {
    pub journal: u64,
    pub seed: u64,
    pub reached_ns: u64,
    pub complete: bool,
}

/// Статистика действительных доставок с одинаковыми границами задержки.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TransferMetric {
    pub hop: String,
    pub expected_min_ns: u64,
    pub expected_max_ns: u64,
    pub measured_min_ns: u64,
    pub measured_max_ns: u64,
    pub delivered: usize,
    pub source: crate::Source,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CaseReport {
    pub case: Case,
    pub outcome: Outcome,
    pub journal: Option<u64>,
    pub events: Option<Fingerprint>,
    pub reached_ns: Option<u64>,
    pub completed: bool,
    pub requested_ns: Option<u64>,
    pub applied_ns: Option<u64>,
    pub excitation_ns: Option<u64>,
    pub metrics: Vec<Metric>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub process_metrics: BTreeMap<String, [f64; 2]>,
    pub reaction: Option<Reaction>,
    pub before_loop: Option<LoopTiming>,
    pub after_loop: Option<LoopTiming>,
    pub deliveries: Vec<TransferMetric>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub radio_coverage: BTreeMap<String, Outcome>,
    pub comparison: Option<Comparison>,
    pub comparison_audit: Option<ComparisonAudit>,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub comparison_started: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub comparison_execution: Option<ComparisonExecution>,
    pub losses: BTreeMap<String, usize>,
    pub runtime_error: Option<String>,
    pub missing: Vec<String>,
    pub violations: Vec<String>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub limit_violations: Vec<LimitViolation>,
    pub control: Option<ControlActivity>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct ControlActivity {
    pub valid_reads: usize,
    pub model_acceptances: usize,
    pub command_changes: usize,
    pub expired_outputs: usize,
    pub verified_cycles: usize,
    pub packet_deliveries: BTreeMap<String, usize>,
    pub last_packet_ns: BTreeMap<String, u64>,
    pub resets: Vec<(u64, String)>,
    pub actions: BTreeMap<String, u64>,
    pub applied_faults: BTreeMap<String, u64>,
}

impl CaseReport {
    fn new(case: Case) -> Self {
        Self {
            case,
            outcome: Outcome::NotAnalyzed,
            journal: None,
            events: None,
            reached_ns: None,
            completed: false,
            requested_ns: None,
            applied_ns: None,
            excitation_ns: None,
            metrics: Vec::new(),
            process_metrics: BTreeMap::new(),
            reaction: None,
            before_loop: None,
            after_loop: None,
            deliveries: Vec::new(),
            radio_coverage: BTreeMap::new(),
            comparison: None,
            comparison_audit: None,
            comparison_started: false,
            comparison_execution: None,
            losses: BTreeMap::new(),
            runtime_error: None,
            missing: Vec::new(),
            violations: Vec::new(),
            limit_violations: Vec::new(),
            control: None,
        }
    }

    /// Строка без живого прогона остаётся в матрице с указанной причиной.
    pub fn not_analyzed(case: Case, detail: &str) -> Self {
        let mut report = Self::new(case);
        report.missing.push(detail.into());
        report
    }

    /// Горизонт достигнут, а для отказного опыта модель подтвердила его применение.
    pub fn was_executed(&self) -> bool {
        self.completed && (self.case.fault.is_none() || self.applied_ns.is_some())
    }

    pub(crate) fn finish(mut self) -> Self {
        self.outcome = if !self.violations.is_empty() {
            Outcome::Fail
        } else if self.completed && self.missing.is_empty() {
            Outcome::Pass
        } else {
            Outcome::NotAnalyzed
        };
        self
    }
}

struct Trace<'a> {
    log: &'a Log,
    observation: Vec<LogEntry>,
    command: Vec<LogEntry>,
    accepted: Vec<LogEntry>,
    truth: BTreeMap<&'a str, Vec<LogEntry>>,
}

fn signal_id(table: &[SignalSpec], signal: &Signal) -> Option<u32> {
    table
        .iter()
        .position(|spec| spec.name == signal.name && spec.unit == signal.unit)
        .and_then(|id| u32::try_from(id).ok())
}

impl<'a> Trace<'a> {
    fn read(log: &'a Log, plan: &RunPlan<'_>, criteria: Option<&Criteria>) -> Result<Self, Error> {
        let tables = log.header();
        let observation = signal_id(&tables.observations, &plan.path.observation);
        // В рантайме канал усилия помечен единицей механической оси даже
        // при передаче команды модели. Единицу команды удостоверяет принятое
        // значение модели. Topology уже сверила имя канала с CompiledDrive.
        let command = tables
            .commands
            .iter()
            .position(|spec| spec.name == plan.path.command.name)
            .and_then(|id| u32::try_from(id).ok());
        let accepted = signal_id(&tables.truth, &plan.path.accepted);
        let mut names = BTreeMap::new();
        if let Some(criteria) = criteria {
            for signal in criteria
                .limits
                .iter()
                .map(Limit::signal)
                .chain(criteria.response.iter().map(|r| &r.signal))
            {
                if let Some(id) = signal_id(&tables.truth, signal) {
                    names.insert(id, tables.truth[id as usize].name.as_str());
                }
            }
        }
        for state in plan
            .path
            .nodes
            .iter()
            .filter_map(|node| node.fault_state.as_ref())
        {
            if let Some(id) = signal_id(&tables.truth, state) {
                names.insert(id, tables.truth[id as usize].name.as_str());
            }
        }
        let mut data = Self {
            log,
            observation: Vec::new(),
            command: Vec::new(),
            accepted: Vec::new(),
            truth: BTreeMap::new(),
        };
        for entry in log.entries() {
            finite(entry.value)?;
            if entry.at > log.end().time {
                return Err(invalid("запись находится после конца прогона"));
            }
            match entry.kind {
                LogKind::Observation if Some(entry.id) == observation => {
                    data.observation.push(*entry)
                }
                LogKind::Command(_) if Some(entry.id) == command => data.command.push(*entry),
                LogKind::Truth => {
                    if Some(entry.id) == accepted {
                        data.accepted.push(*entry);
                    }
                    if let Some(&name) = names.get(&entry.id) {
                        data.truth.entry(name).or_default().push(*entry);
                    }
                }
                _ => {}
            }
        }
        Ok(data)
    }

    fn series(&self, signal: &Signal) -> Option<&[LogEntry]> {
        signal_id(&self.log.header().truth, signal)?;
        self.truth.get(signal.name.as_str()).map(Vec::as_slice)
    }

    fn cycle(
        &self,
        cycle: &Cycle,
        plan: &RunPlan<'_>,
        transfers: &Transfers<'_>,
        publications: Option<&BTreeMap<u64, Time>>,
    ) -> bool {
        if cycle.controller_name != plan.path.controller
            || cycle.program != plan.path.program
            || cycle.captured_at > cycle.observation.at
            || cycle.observation.at > cycle.read_at
            || cycle.read_at > cycle.command.at
            || cycle.command.at > cycle.accepted.at
            || cycle.accepted.at >= cycle.feedback.at
            || cycle.read_at.0 - cycle.captured_at.0 > plan.path.max_observation_age_ns
            || !same_value(cycle.command.value, cycle.accepted.value)
        {
            return false;
        }
        let last = self
            .observation
            .partition_point(|entry| entry.at <= cycle.read_at);
        let observed = if publications.is_some() {
            unique_at(&self.observation, cycle.observation.at)
        } else {
            last.checked_sub(1).and_then(|i| self.observation.get(i))
        };
        let published_at = match publications {
            Some(times) => match times.get(&cycle.sequence) {
                Some(&at) => at,
                None => return false,
            },
            None => cycle.accepted.at,
        };
        let command = unique_at(&self.command, cycle.command.at);
        let latest_sent = self
            .command
            .partition_point(|entry| entry.at <= cycle.accepted.at);
        if !observed.is_some_and(|entry| matches_sample(entry, cycle.observation))
            || unique_at(&self.observation, cycle.observation.at).is_none()
            || !command.is_some_and(|entry| {
                matches_sample(entry, cycle.command)
                    && entry.kind == LogKind::Command(CommandSource::Controller(cycle.controller))
            })
            || (publications.is_none()
                && !latest_sent
                    .checked_sub(1)
                    .is_some_and(|i| self.command[i].at == cycle.command.at))
            || !unique_at(&self.accepted, published_at)
                .is_some_and(|entry| entry.valid && same_value(entry.value, cycle.accepted.value))
            || !unique_at(&self.observation, cycle.feedback.at)
                .is_some_and(|entry| matches_sample(entry, cycle.feedback))
        {
            return false;
        }
        // Для типизированного журнала связь сообщения с циклом проверена
        // по номеру запроса и фактическому содержимому внешнего обмена.
        if publications.is_some() {
            return true;
        }
        let mut input_at = cycle.captured_at;
        let mut output_at = cycle.command.at;
        for hop in &plan.path.hops {
            let Some([transfer]) = transfers
                .get(&(hop.id.as_str(), cycle.sequence))
                .map(Vec::as_slice)
            else {
                return false;
            };
            let Delivery::Received(delivered) = transfer.outcome else {
                return false;
            };
            let (previous, latest) = match hop.direction {
                Direction::Input => (&mut input_at, cycle.observation.at),
                Direction::Output => (&mut output_at, cycle.accepted.at),
            };
            if transfer.sent < *previous || delivered < transfer.sent || delivered > latest {
                return false;
            }
            *previous = delivered;
        }
        true
    }
}

type Transfers<'a> = BTreeMap<(&'a str, u64), Vec<&'a Transfer>>;

fn unique_at(entries: &[LogEntry], at: Time) -> Option<&LogEntry> {
    let first = entries.partition_point(|entry| entry.at < at);
    let entry = entries.get(first)?;
    (entry.at == at && entries.get(first + 1).is_none_or(|next| next.at != at)).then_some(entry)
}

fn same_value(a: f64, b: f64) -> bool {
    a.to_bits() == b.to_bits()
}

fn matches_sample(entry: &LogEntry, sample: Sample) -> bool {
    entry.valid && entry.at == sample.at && same_value(entry.value, sample.value)
}

fn check_stimuli(
    report: &mut CaseReport,
    plan: &RunPlan<'_>,
    events: &[Event],
    end: Time,
) -> Option<(Time, Time)> {
    let mut anchor = None;
    for stimulus in &plan.mode.stimuli {
        let mut receipts = events.iter().filter_map(|event| match event {
            Event::Stimulus(receipt) if receipt.id == stimulus.id => Some(receipt),
            _ => None,
        });
        let planned = Time(stimulus.at_ns);
        if let Some(receipt) = receipts.next().filter(|receipt| {
            receipt.signal == stimulus.signal
                && receipt.requested == planned
                && receipt.applied >= planned
                && receipt.applied <= end
                && same_value(receipt.value, plan.stimulus_value(stimulus))
                && receipts.next().is_none()
        }) {
            if stimulus.id == plan.mode.excitation {
                report.excitation_ns = Some(receipt.applied.0);
                if plan.fault.is_none() {
                    anchor = Some((planned, receipt.applied));
                }
            }
        } else {
            report
                .missing
                .push(format!("нет подтверждения стимула «{}»", stimulus.id));
        }
        if !stimulus.source.can_prove() {
            report
                .missing
                .push(format!("стимул «{}» задан оценкой", stimulus.id));
        }
    }
    for event in events {
        if let Event::Stimulus(receipt) = event
            && !plan
                .mode
                .stimuli
                .iter()
                .any(|stimulus| stimulus.id == receipt.id)
        {
            report
                .missing
                .push(format!("незапланированный стимул «{}»", receipt.id));
        }
    }
    let mut faults = events.iter().filter_map(|event| match event {
        Event::Fault(receipt) => Some(receipt),
        _ => None,
    });
    if let Some(injection) = plan.fault {
        let planned = Time(injection.at_ns);
        anchor = faults
            .next()
            .filter(|receipt| {
                receipt.fault == injection.fault
                    && receipt.requested == planned
                    && receipt.applied >= planned
                    && receipt.applied <= end
                    && faults.next().is_none()
            })
            .map(|receipt| (planned, receipt.applied));
        if anchor.is_none() {
            report.missing.push(format!(
                "нет однозначного подтверждения применения {:?} на узле «{}» по пути «{}» в окне после {} нс",
                injection.fault.kind, injection.fault.node, plan.path.id, injection.at_ns,
            ));
        }
        if !injection.source.can_prove() {
            report
                .missing
                .push("параметры отказа заданы оценкой".into());
        }
    } else if faults.next().is_some() {
        report
            .missing
            .push("в штатном режиме зарегистрирован отказ".into());
    }
    anchor
}

fn check_transfers(
    report: &mut CaseReport,
    plan: &RunPlan<'_>,
    transfers: &Transfers<'_>,
    anchor: Option<(Time, Time)>,
    end: Time,
) -> Result<(), Error> {
    let mut fault_effect = false;
    let mut deliveries = BTreeMap::new();
    for (&(hop_name, _), events) in transfers {
        let Some(hop) = plan.path.hops.iter().find(|hop| hop.id == hop_name) else {
            report
                .missing
                .push(format!("неизвестный участок доставки «{hop_name}»"));
            continue;
        };
        let [transfer] = events.as_slice() else {
            report.missing.push(format!(
                "неоднозначный исход обмена на участке «{hop_name}»"
            ));
            continue;
        };
        let at = transfer.outcome.at();
        let Some(elapsed) = at.0.checked_sub(transfer.sent.0).filter(|_| at <= end) else {
            report
                .missing
                .push(format!("некорректное время доставки «{hop_name}»"));
            continue;
        };
        let active_fault = plan
            .fault
            .zip(anchor)
            .filter(|(fault, (_, applied))| {
                fault.fault.node == hop.node && transfer.sent >= *applied
            })
            .map(|(fault, _)| fault);
        let within_window = anchor.is_some_and(|(_, applied)| {
            applied
                .0
                .checked_add(plan.mode.after_ns)
                .is_some_and(|window_end| at.0 <= window_end)
        });
        if matches!(transfer.outcome, Delivery::Lost(_)) {
            *report.losses.entry(hop.id.clone()).or_default() += 1;
        }
        match (transfer.outcome, active_fault.map(|fault| fault.fault.kind)) {
            (Delivery::Lost(_), Some(FaultKind::PacketLoss)) => fault_effect |= within_window,
            (Delivery::Lost(_), _) => {
                report.missing.push(format!(
                    "пакет на участке «{hop_name}» потерян вне проверки потери"
                ));
            }
            (Delivery::Received(_), _) => {
                let extra = active_fault
                    .and_then(|fault| fault.extra_delay_ns)
                    .unwrap_or(0);
                let (min, max) = delay_limits(hop, extra)?;
                let metric =
                    deliveries
                        .entry((hop_name, min, max))
                        .or_insert_with(|| TransferMetric {
                            hop: hop_name.into(),
                            expected_min_ns: min,
                            expected_max_ns: max,
                            measured_min_ns: elapsed,
                            measured_max_ns: elapsed,
                            delivered: 0,
                            source: hop.source.clone(),
                        });
                metric.measured_min_ns = metric.measured_min_ns.min(elapsed);
                metric.measured_max_ns = metric.measured_max_ns.max(elapsed);
                metric.delivered += 1;
                if elapsed < min || elapsed > max {
                    report.violations.push(format!(
                        "доставка «{hop_name}»: {elapsed} нс, допустимый диапазон [{min}, {max}] нс"
                    ));
                }
                // Отдельный эффект задержки виден за пределами допустимого штатного разброса.
                if within_window
                    && extra > 0
                    && (min..=max).contains(&elapsed)
                    && elapsed > delay_limits(hop, 0)?.1
                {
                    fault_effect = true;
                }
            }
        }
    }
    report.deliveries = deliveries.into_values().collect();
    if plan.fault.is_some_and(|fault| {
        matches!(
            fault.fault.kind,
            FaultKind::PacketLoss | FaultKind::PacketDelay
        )
    }) && !fault_effect
    {
        report.missing.push(
            "не наблюдалась потеря или различимая добавленная задержка на нужном участке".into(),
        );
    }
    Ok(())
}

fn delay_limits(hop: &Hop, extra: u64) -> Result<(u64, u64), Error> {
    // Планировщик доставляет на первом такте после заданного срока. Ровно один такт
    // покрывает случай, когда отправка и обработка в одной отметке разминулись по фазе.
    let min = hop.delay_ns - hop.jitter_ns;
    let max = hop.delay_ns + hop.jitter_ns + hop.receiver_tick_ns;
    min.checked_add(extra)
        .zip(max.checked_add(extra))
        .ok_or_else(|| invalid("переполнение границы задержки"))
}

pub(crate) fn run(
    runtime: &Runtime,
    status: Result<Status, String>,
    plan: &RunPlan<'_>,
    criteria: &[Criteria],
    events: &[Event],
    topology: &Topology,
    world: &CompiledWorld,
) -> Result<CaseReport, Error> {
    let mut report = CaseReport::new(plan.case.clone());
    let log = runtime.log();
    if runtime.process_recording_enabled() {
        report
            .missing
            .push("для процессов требуется отдельная серия сходимости".into());
        for row in runtime.process_journal() {
            for item in row.outputs.values().flat_map(|out| &out.hazards) {
                if let sim_process::work::HazardCheck::Violation { reason } = &item.check {
                    let message = format!("{} / {:?}: {reason}", item.subject, item.kind);
                    if !report.violations.contains(&message) {
                        report.violations.push(message);
                    }
                }
            }
        }
    }
    let control = sim_runtime::control::run(runtime);
    if let Some(run) = &control {
        report.control = Some(crate::live_control::activity(run, plan));
        for route in run.recorded_radio_routes() {
            report
                .radio_coverage
                .insert(route.into(), Outcome::NotAnalyzed);
            report
                .missing
                .push(format!("радиоканал «{route}»: доставка взята из записи"));
        }
        if run.has_external_processes() {
            report.missing.push(
                "локальный процесс имеет доступ к окружению ОС за пределами протокола".into(),
            );
        }
        if run
            .events()
            .iter()
            .any(|event| crate::live_control::unplanned_fault(event, plan))
        {
            report
                .missing
                .push("в проводке запланированы отдельные диагностические отказы".into());
        }
    }
    if control
        .as_ref()
        .is_some_and(|run| run.origin() != sim_runtime::control::RunOrigin::Live)
    {
        report
            .missing
            .push("происхождение контура не подтверждает живое управление".into());
    }
    let publications = control
        .as_ref()
        .map(|run| crate::live_control::publications(run, &plan.path.id));
    let end = log.end();
    report.journal = Some(log.fingerprint());
    report.events = Some(Fingerprint::of(format!("{events:?}").as_bytes()));
    report.reached_ns = Some(end.time.0);
    report.completed = matches!(status, Ok(Status::Running))
        && end.started
        && end.steps > 0
        && end.time == Time(plan.mode.horizon_ns);
    if !report.completed {
        report
            .missing
            .push("прогон не выполнил расчёт до горизонта".into());
        report.runtime_error = status
            .err()
            .or_else(|| runtime.failure().map(str::to_owned));
    }
    if world
        .bodies
        .iter()
        .any(|body| matches!(body.provenance, Provenance::Estimate(_)))
        || !plan.mode.source.can_prove()
        || !plan.path.source.can_prove()
        || plan.path.hops.iter().any(|hop| !hop.source.can_prove())
    {
        report
            .missing
            .push("существенные параметры физики, режима или доставки заданы оценкой".into());
    }
    for limitation in runtime.simplifications().iter().filter(|limitation| {
        limitation.blocks_proof
            && topology
                .limitation_paths
                .get(&limitation.subject)
                .is_none_or(|paths| paths.contains(&plan.path.id))
    }) {
        report.missing.push(format!(
            "ограничение модели «{}»: {}",
            limitation.subject, limitation.text
        ));
    }
    let criteria = criteria
        .iter()
        .find(|candidate| &candidate.case == plan.case);
    if criteria.is_none_or(|criteria| criteria.limits.is_empty()) {
        report.missing.push("пустой набор ограничений".into());
    }
    let anchor = check_stimuli(&mut report, plan, events, end.time);
    if let Some((requested, applied)) = anchor {
        report.requested_ns = Some(requested.0);
        report.applied_ns = Some(applied.0);
        if applied.0 < plan.mode.before_ns
            || applied
                .0
                .checked_add(plan.mode.after_ns)
                .is_none_or(|t| t > end.time.0)
        {
            report
                .missing
                .push("фактическое воздействие оставило неполное окно наблюдения".into());
        }
    }
    let trace = Trace::read(log, plan, criteria)?;
    check_fault_state(&mut report, &trace, plan, anchor)?;
    let mut transfers = Transfers::new();
    let mut cycles = BTreeMap::<u64, Vec<&Cycle>>::new();
    for event in events {
        match event {
            Event::Transfer(transfer) if transfer.path == plan.path.id => {
                transfers
                    .entry((transfer.hop.as_str(), transfer.sequence))
                    .or_default()
                    .push(transfer);
            }
            Event::Cycle(cycle) if cycle.path == plan.path.id => {
                cycles.entry(cycle.sequence).or_default().push(cycle);
            }
            _ => {}
        }
    }
    check_transfers(&mut report, plan, &transfers, anchor, end.time)?;
    let mut live_cycles = Vec::new();
    for cycles in cycles.values() {
        if let [cycle] = cycles.as_slice()
            && trace.cycle(cycle, plan, &transfers, publications.as_ref())
        {
            live_cycles.push(*cycle);
        } else {
            report.missing.push(
                "неоднозначная цепочка или отметки обмена не совпали с живым журналом".into(),
            );
        }
    }
    if trace
        .command
        .iter()
        .any(|entry| !matches!(entry.kind, LogKind::Command(CommandSource::Controller(_))))
    {
        report
            .missing
            .push("в исполнительный канал писала внешняя команда".into());
    }
    if let Some(activity) = &mut report.control {
        activity.verified_cycles = live_cycles.len();
    }
    if let Some(criteria) = criteria {
        for limit in &criteria.limits {
            check_limit(&mut report, &trace, plan.mode, anchor, limit)?;
        }
    }
    check_reaction(&mut report, &trace, plan, criteria, anchor, &live_cycles)?;
    Ok(report.finish())
}

pub(crate) fn compare(
    report: &mut CaseReport,
    plan: &RunPlan<'_>,
    runtime: &Runtime,
    reference: &LiveRun,
) -> Result<(), Error> {
    let original = runtime.log();
    let controls_match = match (
        sim_runtime::control::run(runtime),
        sim_runtime::control::run(&reference.runtime),
    ) {
        (Some(actual), Some(alternate)) => {
            let mut actual_identity = actual.identity().clone();
            actual_identity.control = alternate.identity().control;
            actual_identity == *alternate.identity()
                && actual.paths() == alternate.paths()
                && actual.origin() == alternate.origin()
        }
        (None, None) => true,
        _ => false,
    };
    if !controls_match {
        if let Some(execution) = &mut report.comparison_execution {
            execution.complete = false;
        }
        report
            .missing
            .push("в парном опыте изменились ресурсы, проводка или расписание контура".into());
        return Ok(());
    }
    let log = reference.runtime.log();
    let end = log.end();
    if !matches!(reference.status, Ok(Status::Running))
        || !end.started
        || end.steps == 0
        || end.time != Time(plan.mode.horizon_ns)
        || log.header() != original.header()
    {
        if let Some(execution) = &mut report.comparison_execution {
            execution.complete = false;
        }
        report
            .missing
            .push("парный прогон не завершён или собран с другим набором сигналов".into());
        if let Some(error) = reference.runtime.failure() {
            report.runtime_error = Some(error.to_owned());
        }
        return Ok(());
    }
    let mut comparison = CaseReport::new(plan.case.clone());
    let applied = check_stimuli(&mut comparison, plan, &reference.journal, end.time)
        .map(|(_, applied)| applied.0);
    report.missing.extend(comparison.missing);
    if comparison.excitation_ns != report.excitation_ns || applied != report.applied_ns {
        report
            .missing
            .push("в парном опыте изменилось время воздействия".into());
        return Ok(());
    }
    let a = Trace::read(original, plan, None)?;
    let b = Trace::read(log, plan, None)?;
    // До воздействия наблюдения и исполнительные команды должны совпадать.
    // Иначе менялся не один вход либо парный сеанс невоспроизводим.
    let Some(anchor) = report.excitation_ns.map(Time) else {
        return Ok(());
    };
    let same_before = |x: &[LogEntry], y: &[LogEntry]| {
        x.iter()
            .take_while(|item| item.at < anchor)
            .eq(y.iter().take_while(|item| item.at < anchor))
    };
    if same_before(&a.command, &b.command) && same_before(&a.observation, &b.observation) {
        let changed_observations = a
            .observation
            .iter()
            .filter(|entry| entry.at >= anchor && entry.valid)
            .filter(|entry| {
                unique_at(&b.observation, entry.at)
                    .is_some_and(|other| other.valid && !same_value(other.value, entry.value))
            })
            .count();
        report.comparison_audit = Some(ComparisonAudit {
            journal: log.fingerprint(),
            changed_observations,
            command_stream_equal: a.command == b.command,
        });
        if changed_observations > 0 && a.command == b.command {
            report
                .missing
                .push("изменение доставленных измерений не изменило команды".into());
        }
    }
    let Some(witness) = &report.after_loop else {
        report
            .missing
            .push("нет подтверждённого обмена для парной сверки".into());
        return Ok(());
    };
    let at = Time(witness.command_ns);
    let delivered_at = Time(witness.delivered_ns);
    let commands = unique_at(&a.command, at).zip(unique_at(&b.command, at));
    let observations =
        unique_at(&a.observation, delivered_at).zip(unique_at(&b.observation, delivered_at));
    if let Some((actual, alternate)) = commands
        && same_before(&a.command, &b.command)
        && same_before(&a.observation, &b.observation)
        && actual.kind == alternate.kind
        && observations.is_some_and(|(actual, alternate)| {
            actual.valid && alternate.valid && actual.value != alternate.value
        })
    {
        let difference = (actual.value - alternate.value).abs();
        finite(difference)?;
        report.comparison = Some(Comparison {
            journal: log.fingerprint(),
            events: Fingerprint::of(format!("{:?}", reference.journal).as_bytes()),
            at_ns: at.0,
            original_command: actual.value,
            comparison_command: alternate.value,
        });
        if difference < plan.path.command_delta {
            report
                .missing
                .push("команда совпала с парным опытом при другом измерении".into());
        }
    } else {
        report
            .missing
            .push("парный опыт не подтвердил сравнимую смену измерения и команды".into());
    }
    Ok(())
}

fn covered(entries: &[LogEntry], mode: &Mode, anchor: Option<(Time, Time)>) -> bool {
    let Some((_, applied)) = anchor else {
        return false;
    };
    let full = entries
        .first()
        .zip(entries.last())
        .is_some_and(|(first, last)| {
            first.at == Time::ZERO
                && last.at.0 <= mode.horizon_ns
                && mode.horizon_ns - last.at.0 <= mode.max_gap_ns
        });
    full && entries.iter().all(|entry| entry.valid)
        && entries.windows(2).all(|pair| {
            let gap = pair[1].at.0.saturating_sub(pair[0].at.0);
            gap > 0 && gap <= mode.max_gap_ns
        })
        && applied
            .0
            .checked_sub(mode.before_ns)
            .is_some_and(|time| entries.iter().any(|e| e.at.0 <= time))
        && applied
            .0
            .checked_add(mode.after_ns)
            .is_some_and(|time| entries.iter().any(|e| e.at.0 >= time))
}

fn check_fault_state(
    report: &mut CaseReport,
    trace: &Trace<'_>,
    plan: &RunPlan<'_>,
    anchor: Option<(Time, Time)>,
) -> Result<(), Error> {
    for node in &plan.path.nodes {
        if matches!(node.kind, crate::NodeKind::Bus | crate::NodeKind::Radio) {
            continue;
        }
        let active = plan
            .fault
            .is_some_and(|injection| injection.fault.node == node.id);
        let Some(series) = node
            .fault_state
            .as_ref()
            .and_then(|state| trace.series(state))
        else {
            if active {
                report.missing.push(format!(
                    "для узла «{}» отсутствует журнал состояния отказа",
                    node.id
                ));
            }
            continue;
        };
        if series
            .iter()
            .any(|entry| entry.value != 0.0 && entry.value != 1.0)
        {
            return Err(invalid(format!(
                "состояние отказа «{}» должно быть 0 или 1",
                node.id
            )));
        }
        if let Some((_, applied)) = anchor {
            let before = series.partition_point(|entry| entry.at < applied);
            let window_end = applied
                .0
                .checked_add(plan.mode.after_ns)
                .ok_or_else(|| invalid("переполнение окна отказа"))?;
            let conflicting = if active {
                !covered(series, plan.mode, anchor)
                    || before == 0
                    || before == series.len()
                    || series[..before].iter().any(|entry| entry.value != 0.0)
                    || series[before].at != applied
                    || series[before..]
                        .iter()
                        .take_while(|entry| entry.at.0 <= window_end)
                        .any(|entry| entry.value != 1.0)
            } else {
                series.iter().any(|entry| entry.value != 0.0)
            };
            if conflicting {
                report.missing.push(format!(
                    "истина не подтверждает заданное состояние отказа «{}»",
                    node.id
                ));
            }
        }
    }
    Ok(())
}

fn check_limit(
    report: &mut CaseReport,
    trace: &Trace<'_>,
    mode: &Mode,
    anchor: Option<(Time, Time)>,
    limit: &Limit,
) -> Result<(), Error> {
    let signal = limit.signal();
    let Some(entries) = trace.series(signal) else {
        report.missing.push(format!(
            "нет истины «{}» в единицах «{}»",
            signal.name, signal.unit
        ));
        return Ok(());
    };
    let first = entries.first().expect("ряд создаётся при первой записи");
    let last = entries.last().expect("ряд создаётся при первой записи");
    let (min, max) = extrema(entries).expect("ряд создаётся при первой записи");
    let scale = min.abs().max(max.abs());
    let rms = if scale > 0.0 {
        let squares = entries
            .iter()
            .map(|entry| (entry.value / scale).powi(2))
            .sum::<f64>();
        (squares / entries.len() as f64).sqrt() * scale
    } else {
        0.0
    };
    finite(rms)?;
    let covered = covered(entries, mode, anchor);
    report.metrics.push(Metric {
        signal: signal.clone(),
        limit: limit.clone(),
        min,
        max,
        min_at_ns: entries.iter().find(|e| e.value == min).map(|e| e.at.0),
        max_at_ns: entries.iter().find(|e| e.value == max).map(|e| e.at.0),
        last: last.value,
        rms,
        first_ns: first.at.0,
        last_ns: last.at.0,
        samples: entries.len(),
        covered_window: (covered && first.at < last.at).then_some(crate::campaign::Window {
            start_ns: first.at.0,
            end_ns: last.at.0,
            max_gap_ns: mode.max_gap_ns,
        }),
    });
    if !covered {
        report
            .missing
            .push(format!("неполное окно истины «{}»", signal.name));
    }
    let source = match limit {
        Limit::Range {
            min: lower,
            max: upper,
            source,
            ..
        } => {
            if let Some(event) = entries.iter().find(|entry| {
                lower.is_some_and(|lo| entry.value < lo) || upper.is_some_and(|hi| entry.value > hi)
            }) {
                report.limit_violations.push(LimitViolation {
                    signal: signal.clone(),
                    value: event.value,
                    at_ns: event.at.0,
                });
                report.violations.push(format!(
                    "«{}»: диапазон [{min}, {max}] {} нарушил границы {lower:?}…{upper:?}",
                    signal.name, signal.unit
                ));
            }
            source
        }
        Limit::Forbidden { source, .. } => {
            if entries
                .iter()
                .any(|entry| entry.value != 0.0 && entry.value != 1.0)
            {
                return Err(invalid(format!(
                    "«{}» не является двоичным событием",
                    signal.name
                )));
            }
            if let Some(event) = entries.iter().find(|entry| entry.value == 1.0) {
                report.limit_violations.push(LimitViolation {
                    signal: signal.clone(),
                    value: event.value,
                    at_ns: event.at.0,
                });
                report.violations.push(format!(
                    "запретное событие «{}» на {} нс",
                    signal.name, event.at.0
                ));
            }
            source
        }
    };
    if !source.can_prove() {
        report
            .missing
            .push(format!("порог «{}» задан оценкой", signal.name));
    }
    Ok(())
}

fn check_reaction(
    report: &mut CaseReport,
    trace: &Trace<'_>,
    plan: &RunPlan<'_>,
    criteria: Option<&Criteria>,
    anchor: Option<(Time, Time)>,
    cycles: &[&Cycle],
) -> Result<(), Error> {
    let Some(response) = criteria.and_then(|criteria| criteria.response.as_ref()) else {
        report
            .missing
            .push("не задан критерий физической реакции".into());
        return Ok(());
    };
    if !plan.path.physical_response.contains(&response.signal) {
        report
            .missing
            .push("физическая реакция не привязана к выходу этого пути".into());
        return Ok(());
    }
    if !response.source.can_prove() {
        report
            .missing
            .push("порог физической реакции задан оценкой".into());
    }
    let Some((requested, applied)) = anchor else {
        return Ok(());
    };
    let Some(series) = trace.series(&response.signal) else {
        report.missing.push(format!(
            "нет ряда физической реакции «{}»",
            response.signal.name
        ));
        return Ok(());
    };
    if !covered(series, plan.mode, anchor) {
        report.missing.push("ряд физической реакции неполон".into());
    }
    let before = series.partition_point(|entry| entry.at < applied);
    let Some(baseline) = before.checked_sub(1).and_then(|i| series.get(i)) else {
        report
            .missing
            .push("нет исходного значения истины до воздействия".into());
        return Ok(());
    };
    let Some(window_end) = applied.0.checked_add(plan.mode.after_ns) else {
        return Err(invalid("переполнение окна реакции"));
    };
    let base_cycle = cycles
        .iter()
        .copied()
        .filter(|cycle| cycle.feedback.at < applied)
        .max_by_key(|cycle| cycle.command.at);
    report.before_loop = base_cycle.map(LoopTiming::from);
    if base_cycle.is_none() {
        report
            .missing
            .push("до воздействия отсутствует завершённый обмен".into());
    }
    let Some((reaction, change)) = crossing(
        &series[before..],
        baseline.value,
        applied,
        Time(window_end),
        response.min_change,
    )?
    else {
        report
            .missing
            .push("не наблюдалась физическая реакция после воздействия".into());
        return Ok(());
    };
    let delay = reaction.0 - applied.0;
    report.reaction = Some(Reaction {
        criterion: response.clone(),
        requested_ns: requested.0,
        applied_ns: applied.0,
        reaction_ns: delay,
        change,
    });
    if delay > response.within_ns {
        report.violations.push(format!(
            "физическая реакция через {delay} нс, предел {} нс",
            response.within_ns
        ));
    }
    if let Some(excitation) = report.excitation_ns.map(Time) {
        let baseline = cycles
            .iter()
            .copied()
            .filter(|cycle| cycle.feedback.at < excitation)
            .max_by_key(|cycle| cycle.command.at);
        if let Some(baseline) = baseline {
            let candidate = cycles.iter().copied().filter(|cycle| {
                cycle.captured_at >= excitation
                    && cycle.observation.value != baseline.observation.value
                    && cycle.feedback.value != baseline.feedback.value
                    && if plan.fault.is_none() {
                        cycle.accepted.at < reaction
                            && cycle.feedback.at >= reaction
                            && cycle.feedback.at.0 <= window_end
                    } else {
                        // В отказном опыте рабочая обратная связь нужна до отказа.
                        // После отказа измеряется физическая защитная реакция.
                        cycle.feedback.at < applied
                    }
            });
            for cycle in candidate {
                let difference = (cycle.command.value - baseline.command.value).abs();
                finite(difference)?;
                if difference >= plan.path.command_delta {
                    report.after_loop = Some(LoopTiming::from(cycle));
                    break;
                }
            }
        }
    }
    if report.after_loop.is_none() {
        report.missing.push(
            "не прослежена реакция на новое измерение до принятия команды и обратного измерения"
                .into(),
        );
    }
    Ok(())
}

fn extrema(entries: &[LogEntry]) -> Option<(f64, f64)> {
    let (first, rest) = entries.split_first()?;
    Some(
        rest.iter()
            .fold((first.value, first.value), |(min, max), entry| {
                (min.min(entry.value), max.max(entry.value))
            }),
    )
}

fn crossing(
    entries: &[LogEntry],
    baseline: f64,
    applied: Time,
    end: Time,
    threshold: f64,
) -> Result<Option<(Time, f64)>, Error> {
    for entry in entries.iter().take_while(|entry| entry.at <= end) {
        if entry.at <= applied {
            continue;
        }
        let change = (entry.value - baseline).abs();
        finite(change)?;
        if change >= threshold {
            return Ok(Some((entry.at, change)));
        }
    }
    Ok(None)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(at: u64, value: f64) -> LogEntry {
        LogEntry {
            at: Time(at),
            kind: LogKind::Truth,
            id: 0,
            value,
            valid: true,
        }
    }

    #[test]
    fn ручной_журнал_стержня_проверяет_границы_и_время() {
        // Ручной ряд в м: пределы −3 и 5. Воздействие в 2 мс, исходное
        // значение −1 м. Порог 2 м достигается в 4 мс при −3 м, через 2 мс.
        let tape = [
            point(0, -1.0),
            point(1_000_000, -1.0),
            point(2_000_000, 5.0),
            point(3_000_000, 0.0),
            point(4_000_000, -3.0),
            point(5_000_000, 2.0),
        ];
        assert_eq!(extrema(&tape), Some((-3.0, 5.0)));
        assert_eq!(
            crossing(&tape, -1.0, Time(2_000_000), Time(5_000_000), 2.0).unwrap(),
            Some((Time(4_000_000), 2.0))
        );
        assert_eq!(
            crossing(&tape, -1.0, Time(2_000_000), Time(3_000_000), 2.0).unwrap(),
            None
        );
        assert_eq!(extrema(&[]), None);
    }

    #[test]
    fn переполнение_даже_конечных_замеров_даёт_ошибку() {
        assert!(crossing(&[point(1, f64::MAX)], -f64::MAX, Time::ZERO, Time(1), 1.0).is_err());
    }
}
