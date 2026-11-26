//! Живые наблюдения и адаптеры предметных оценщиков.

use std::collections::{BTreeMap, BTreeSet};
use std::num::NonZeroU32;

use sim_mujoco::{JointInfo, JointKind};
use sim_process::{
    Analysis, SourcedRange,
    schematic::{
        CompiledSchematic, DepthCheck, DepthEvidence, DepthFinding, DepthModeMark, ElectricalClass,
        ModePlan, Probe, Scope, Status as ElectricalStatus,
        capacitor_stress::CapacitorVoltageSense,
        emi_reset::{EmiEvidence, EmiInput, EmiOutcome, EmiPacketProbe, assess_emi_reset, prepare_emi_reset},
        sim_circuit::pcb::{clearance, layout::Layout, ratings, return_path},
        solder_joint::{SolderControlSeries, SolderJointProfile, SolderJointReport},
        switch_thermal::SwitchSoaTime,
    },
    work::solder_window::SolderWindowStatus,
};
use sim_runtime::{Evaluator, Log, LogEnd, LogEntry, LogKind, Runtime, Status, Truth, TruthId, TruthOut};
use sim_signal::{Command, Rng, SignalSpec, Time};
use sim_world::{
    JointType, Mass, STRENGTH_REQUIRES_BENCH, SectionAxis, Shape, StrengthSection, strength_section,
};

use crate::{
    Error, Fingerprint, Limit, Outcome, RunResult, Source, SourceKind,
    campaign::{
        CampaignPlan, Disposition, PlannedFault, ReferenceKind, RunKey, SampledInputs, Step,
        Target, Window,
    },
    interaction::InteractionEvidence,
    invalid,
    metrics::{
        self, FaultObservation, Installation, Point, RunSummary, Series, SeriesKind,
        StepObservation,
    },
    process::ConvergenceReport,
    sampling::{ModelSettings, Parameter, delay_ns},
};

use super::{
    Check, ControlCriterion, DEPTH_REGISTRY, DepthEventIdentity, DepthFindingIdentity, Finding,
    HazardClass, LegacyVoltageWitness, LimitValue, Requirement, RequirementKey, SectionLoad,
    RequirementSet, Verification, checked_pair, compiled_world_fingerprint, fault_class,
    limit_source, provenance, requires_fault, section_reaction::{self, Wrench, cross, sub}, trace,
    section_span::{self, Distribution, Motion},
};

/// Возвращает разрешитель установки в той же границе доверия, что и сборщик контура.
/// Параметры должны быть считаны из установленных моделей.
#[derive(Clone, Debug)]
pub struct Installed {
    pub installation: Installation,
    pub build: Fingerprint,
    pub settings: ModelSettings,
    pub operations: Vec<StepObservation>,
}

/// Линия стендового журнала, уже разрешённого владельцем формата.
#[derive(Clone, Debug)]
pub struct MeasurementSample {
    pub at_ns: u64,
    pub value: f64,
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
pub struct SnapshotAudit {
    pub at_ns: u64,
    pub matched: bool,
    pub reason: Option<String>,
}

#[derive(Debug)]
struct Measurement {
    signal: crate::Signal,
    source: Source,
    journal: Fingerprint,
    samples: Vec<Point>,
    uncertainty: Option<f64>,
}

#[derive(Clone)]
struct AppliedEffort {
    joint: String,
    dof: usize,
    index: Option<usize>,
    signal: TruthId,
}

impl Evaluator for AppliedEffort {
    fn evaluate(&mut self, truth: &Truth<'_>, _: &mut Rng, out: &mut TruthOut) -> Result<(), String> {
        if truth.time() == Time::ZERO {
            return Ok(());
        }
        let engine = truth.engine();
        let index = if let Some(index) = self.index {
            index
        } else {
            let joint = engine.joints().into_iter().find(|joint| joint.name == self.joint)
                .ok_or("нет физической оси приложения нагрузки")?;
            let index = joint.dof_adr + self.dof;
            if index >= engine.nv() {
                return Err("ось нагрузки за пределами массива MuJoCo".into());
            }
            self.index = Some(index);
            index
        };
        // MuJoCo хранит nv приложенных усилий. Читаем сумму, применённую
        // на завершённом шаге, после интегрирования и до следующего шага.
        out.write(self.signal, unsafe { *engine.data().qfrc_applied.add(index) });
        Ok(())
    }
}

/// Часть живого журнала, которая нужна итоговому оценщику. Остальные
/// записи уже учтены при оценке сценария и в отпечатке полного SIMLOG.
#[derive(Debug)]
struct TruthLog {
    end: LogEnd,
    specs: Vec<SignalSpec>,
    entries: Vec<LogEntry>,
    fingerprint: u64,
}

impl TruthLog {
    fn capture(log: &Log, fingerprint: u64) -> Self {
        Self {
            end: log.end(),
            specs: log.header().truth.clone(),
            entries: log
                .entries()
                .iter()
                .copied()
                .filter(|entry| entry.kind == LogKind::Truth)
                .collect(),
            fingerprint,
        }
    }

    fn series(&self, signal: &crate::Signal) -> Result<Series, Error> {
        Series::from_entries(&self.entries, &self.specs, SeriesKind::Truth, signal)
    }
}

#[derive(Debug)]
struct Run {
    sampled: SampledInputs,
    build: Option<Fingerprint>,
    log: TruthLog,
    divergent_repeat: Option<TruthLog>,
    sources: Vec<Source>,
    installation_confirmed: bool,
    missing: Vec<String>,
    scenario: Outcome,
    summary: Option<RunSummary>,
    snapshot: Option<SnapshotAudit>,
    faults_confirmed: bool,
    planned_faults: Vec<PlannedFault>,
    joints: Vec<JointInfo>,
    section_world_matches: bool,
    task: Option<Fingerprint>,
    process: Option<(Fingerprint, Outcome, Vec<Finding>)>,
    interaction: Option<(Fingerprint, Outcome, Vec<Finding>)>,
    control: Option<(Fingerprint, bool, crate::CaseReport)>,
    linked_profile: Option<Fingerprint>,
    profiles: BTreeMap<ReferenceKind, BTreeSet<Fingerprint>>,
    bench: Vec<Measurement>,
}

#[derive(Clone, Copy)]
struct SectionSample {
    effort: f64,
    reaction: Wrench,
    motion: Option<Motion>,
    at: u64,
    journal: Fingerprint,
}

#[derive(Clone, Copy)]
enum SectionWitness {
    Fixture,
    Root,
    Interior,
}

struct SectionWindow<'a> {
    link: &'a sim_world::Link,
    samples: Vec<SectionSample>,
    complete: bool,
    complex_path: bool,
}

/// Расчёт схемы исполняется здесь. Импорта исходов из JSON или TOML нет.
#[derive(Debug)]
pub struct ElectricalEvidence {
    instance: String,
    file: Fingerprint,
    build: Fingerprint,
    inputs: Fingerprint,
    boards: BTreeSet<String>,
    live: Option<Fingerprint>,
    assessment: sim_process::schematic::ElectricalAssessment,
    findings: Vec<Finding>,
    solder_joints: BTreeMap<String, SolderJointReport>,
}

impl ElectricalEvidence {
    pub fn evaluate(
        instance: &str,
        schematic_bytes: &[u8],
        compiled: &sim_process::schematic::CompiledSchematic,
        plan: &sim_process::schematic::ModePlan,
        build: Fingerprint,
    ) -> Result<Self, Error> {
        Self::evaluate_with_solder(instance, schematic_bytes, compiled, plan, build, &BTreeMap::new())
    }

    /// Считает окно каждого паяного стыка и переносит применимые замеры серии.
    /// Профиль, серия и число интервалов сетки адресуются ID стыка схемы.
    pub fn evaluate_with_solder(
        instance: &str,
        schematic_bytes: &[u8],
        compiled: &CompiledSchematic,
        plan: &ModePlan,
        build: Fingerprint,
        solder: &BTreeMap<
            String,
            (
                Option<SolderJointProfile>,
                Option<SolderControlSeries>,
                NonZeroU32,
            ),
        >,
    ) -> Result<Self, Error> {
        use sim_process::schematic::{ElectricalClass, Scope, Status as ElectricalStatus};

        crate::task::check_name(instance)?;
        let file = Fingerprint::of(schematic_bytes);
        let inputs = electrical_inputs(file, compiled, plan);
        let assessment = compiled.assess_electrical(plan).map_err(|err| invalid(err.to_string()))?;
        let modes: BTreeMap<_, _> = assessment.modes.iter().map(|mode| (&mode.mode, mode)).collect();
        let mut findings = Vec::new();
        for (index, item) in assessment.findings.iter().enumerate() {
            let code = item.class.code();
            let mode = item.mode.as_ref().and_then(|mode| modes.get(mode));
            let key = RequirementKey {
                class: item.class.into(),
                object: format!("{instance}/{}", item.subject),
                extension: Some(format!("{code}/{index}")),
                run: None,
                window_ns: mode.and_then(|mode| mode.horizon).map(|end| [0, end.0]),
                level: super::Level::Simulation,
            };
            let mut finding = Finding::open(&key, &item.reason);
            finding.code = code.into();
            finding.outcome = match item.status {
                ElectricalStatus::Fail => Outcome::Fail,
                ElectricalStatus::Pass if assessment.scope == Scope::Reference
                    || compiled.bindings().is_empty()
                    || item.class == ElectricalClass::AssemblyMismatch => Outcome::Pass,
                _ => Outcome::NotAnalyzed,
            };
            finding.measurement = item.measurement;
            finding.unit = item.unit.map(str::to_owned);
            finding.limit = item.limit.as_ref().map(|r| {
                match item.class {
                    ElectricalClass::RailSag => LimitValue { lower: Some([r.min, r.max]), upper: None },
                    // Статическая проверка хранит напряжение со знаком.
                    ElectricalClass::ReversePolarity if item.mode.is_none() => {
                        LimitValue { lower: Some([-r.max, -r.min]), upper: None }
                    }
                    _ => LimitValue::upper([r.min, r.max]),
                }
            });
            finding.limit_sources.extend(item.limit.as_ref().map(|r| {
                let source = provenance(&r.provenance);
                format!("{:?}: {}", source.kind, source.reference)
            }));
            finding.measurement_sources.push(format!("схема {:016x}, входы {:016x}", file.as_u64(), inputs.as_u64()));
            finding.event_ns = item.event.map(|at| at.0);
            finding.journal = mode.and_then(|mode| mode.journal)
                .map(|journal| electrical_journal(build, inputs, journal))
                .or_else(|| item.mode.is_none().then(|| {
                    Fingerprint::of(format!("{build:?}{inputs:?}{item:?}").as_bytes())
                }));
            finding.model_scope = item.conditions.clone();
            if assessment.scope == Scope::Reference && finding.outcome == Outcome::Pass {
                finding.model_scope = Some(item.conditions.as_ref().map_or_else(
                    || "эталонный стенд, только проверенное состояние".into(),
                    |conditions| format!("эталонный стенд, только проверенное состояние. {conditions}"),
                ));
            }
            if finding.outcome == Outcome::NotAnalyzed {
                if item.status == ElectricalStatus::Pass {
                    finding.reason = "расчёт нуждается в подтверждении на связанном живом режиме".into();
                    finding.missing_data = vec![finding.reason.clone()];
                }
                if let Some(gap) = mode.and_then(|mode| mode.reason.as_ref()) {
                    finding.coverage_gaps.push(gap.clone());
                }
            } else {
                finding.missing_data.clear();
            }
            finding.confirmed_hazard = finding.outcome == Outcome::Fail;
            finding.electrical = Some(super::ElectricalFindingIdentity {
                schematic: file, inputs, build,
                mode: item.mode.as_ref().map(|mode| format!("{mode:?}")),
                probe: item.probe.as_ref().map(|probe| format!("{probe:?}")),
                witness: item.witness.clone(),
            });
            if let Some(power) = item.dissipation_w {
                let key = RequirementKey { class: HazardClass::WiringHeat, ..finding.key.clone() };
                let mut heat = Finding::open(&key, "по диссипации вычисляется нагрев при известных теплоёмкости, теплоотводе и пределе температуры");
                heat.measurement = Some([power, power]);
                heat.unit = Some("W".into());
                heat.event_ns = finding.event_ns;
                heat.journal = finding.journal;
                heat.electrical = finding.electrical.clone();
                heat.measurement_sources.clone_from(&finding.measurement_sources);
                findings.push(heat);
            }
            findings.push(finding);
        }
        if !assessment.inapplicable.contains_key(&ElectricalClass::ConnectionOvercurrent) {
            let key = RequirementKey {
                class: HazardClass::WiringHeat,
                object: format!("{instance}/проводка"),
                extension: Some("электрическая схема".into()),
                run: None, window_ns: None, level: super::Level::Simulation,
            };
            let mut finding = Finding::open(&key,
                "для нагрева нужны теплоёмкость, отвод тепла и предел температуры каждого участка");
            finding.electrical = Some(super::ElectricalFindingIdentity {
                schematic: file, inputs, build, mode: None, probe: None, witness: None,
            });
            findings.push(finding);
        }
        let outstanding = if assessment.scope == Scope::Reference {
            Some("эталонная схема требует подтверждения фактического состава и монтажа")
        } else if !compiled.bindings().is_empty() {
            Some("положительный вывод требует связанного живого свидетельства электрических режимов")
        } else {
            None
        };
        if let Some(reason) = outstanding {
            let key = RequirementKey {
                class: HazardClass::Execution, object: format!("{instance}/электрический опыт"),
                extension: Some("электрическая схема".into()), run: None, window_ns: None,
                level: super::Level::Simulation,
            };
            let mut finding = Finding::open(&key, reason);
            finding.electrical = Some(super::ElectricalFindingIdentity {
                schematic: file, inputs, build, mode: None, probe: None, witness: None,
            });
            findings.push(finding);
        }
        let boards = compiled.bindings().iter().filter_map(|binding| match &binding.target {
            sim_process::schematic::BindingTarget::BoardPower { board, .. } => Some(board.clone()),
            _ => None,
        }).collect();
        let mut result = Self {
            instance: instance.into(),
            file,
            build,
            inputs,
            boards,
            live: None,
            assessment,
            findings,
            solder_joints: BTreeMap::new(),
        };
        result.assess_solder(compiled, solder)?;
        Ok(result)
    }

    fn assess_solder(
        &mut self,
        compiled: &CompiledSchematic,
        supplied: &BTreeMap<
            String,
            (
                Option<SolderJointProfile>,
                Option<SolderControlSeries>,
                NonZeroU32,
            ),
        >,
    ) -> Result<(), Error> {
        let joints: BTreeSet<_> = compiled.solder_joint_ids().collect();
        if let Some(id) = supplied.keys().find(|id| !joints.contains(id.as_str())) {
            return Err(invalid(format!("«{id}»: нет паяного стыка в схеме")));
        }
        for id in joints {
            // Без профиля оценщик возвращает причину до построения сетки.
            let (profile, series, steps) = supplied
                .get(id)
                .map_or((None, None, NonZeroU32::MIN), |(profile, series, steps)| {
                    (profile.as_ref(), series.as_ref(), *steps)
                });
            let report = compiled
                .assess_solder_joint(id, profile, series, steps)
                .map_err(|err| invalid(format!("стык «{id}»: {err}")))?;
            let key = RequirementKey {
                class: HazardClass::SolderDefect,
                object: format!("{}/{id}", self.instance),
                extension: Some(format!("solder_defect/{id}")),
                run: None,
                window_ns: None,
                level: super::Level::Simulation,
            };
            let (outcome, reason) = match report.window_status() {
                SolderWindowStatus::Fail => {
                    (Outcome::Fail, "параметры пайки вне окна процесса".into())
                }
                SolderWindowStatus::Pass if report.address.scope == Scope::Reference => {
                    (Outcome::Pass, "окно процесса выполнено".into())
                }
                SolderWindowStatus::Pass => {
                    let reason = [&report.resistance_ohm, &report.strength_n]
                        .into_iter()
                        .find_map(|measurement| match measurement {
                            Analysis::NotAnalyzed { reason } => Some(reason.as_str()),
                            Analysis::Analyzed { .. } => None,
                        })
                        .unwrap_or("нужны приёмочные пределы сопротивления и прочности для сборки");
                    (
                        Outcome::NotAnalyzed,
                        format!("окно процесса выполнено. {reason}"),
                    )
                }
                SolderWindowStatus::NotAnalyzed { reason } => (Outcome::NotAnalyzed, reason),
            };
            let mut finding = Finding::open(&key, reason);
            finding.outcome = outcome;
            finding.confirmed_hazard = outcome == Outcome::Fail;
            if outcome != Outcome::NotAnalyzed {
                finding.missing_data.clear();
            }
            finding.model_scope = Some(
                match report.address.scope {
                    Scope::Reference => "окно процесса адресованного стыка эталонного стенда",
                    Scope::Assembly => "окно процесса адресованного стыка сборки",
                }
                .into(),
            );
            let inputs = Fingerprint::of(
                format!("{:?}{id:?}{profile:?}{series:?}{steps}", self.inputs).as_bytes(),
            );
            finding.electrical = Some(super::ElectricalFindingIdentity {
                schematic: self.file,
                inputs,
                build: self.build,
                mode: None,
                probe: Some(id.into()),
                witness: None,
            });
            self.findings.push(finding);
            self.solder_joints.insert(id.into(), report);
        }
        Ok(())
    }

    pub fn solder_joints(&self) -> &BTreeMap<String, SolderJointReport> {
        &self.solder_joints
    }

    /// Исполняет профильную проверку и добавляет её обязательство к этой установке.
    /// Возвращает полный предметный результат для отчёта. Готовые исходы не принимаются.
    pub fn record_depth(
        &mut self,
        compiled: &CompiledSchematic,
        plan: &ModePlan,
        id: &str,
        mark: DepthModeMark<'_>,
        check: DepthCheck<'_>,
    ) -> Result<DepthFinding, Error> {
        crate::task::check_name(id)?;
        if id.chars().any(char::is_control)
            || self
                .findings
                .iter()
                .any(|f| f.depth.as_ref().is_some_and(|d| d.id == id))
        {
            return Err(invalid(
                "некорректное или повторное имя углублённой проверки",
            ));
        }
        if self.inputs != electrical_inputs(self.file, compiled, plan) {
            return Err(invalid(
                "углублённая проверка относится к другой схеме, каталогу или режимам",
            ));
        }
        let depth = compiled
            .assess_depth(plan, mark, check)
            .map_err(|err| invalid(err.to_string()))?;
        let spec = DEPTH_REGISTRY
            .iter()
            .find(|spec| spec.code == depth.code)
            .ok_or_else(|| invalid("неизвестная углублённая причина"))?;
        let branch = format!("{:?}", depth.branch);
        let window_ns = depth.window.map(|window| [window.start.0, window.end.0]);
        let key = RequirementKey {
            class: spec.class,
            object: format!("{}/{branch}", self.instance),
            extension: Some(format!("depth/{}/{}", spec.code, id)),
            run: None,
            window_ns,
            level: super::Level::Simulation,
        };
        let mut finding = Finding::open(&key, &depth.reason);
        let scope = match depth.scope {
            Scope::Assembly => "assembly",
            Scope::Reference => "reference",
        };
        finding.code = spec.code.into();
        finding.verification = spec.independent_check.into();
        finding.model_scope = Some(
            match depth.scope {
                Scope::Assembly => "ветвь, режим и окно сборки",
                Scope::Reference => "эталонный стенд, только указанная ветвь, режим и окно",
            }
            .into(),
        );
        if depth.applicability_gap.is_none() {
            finding.outcome = match depth.status {
                ElectricalStatus::Pass => Outcome::Pass,
                ElectricalStatus::Fail => Outcome::Fail,
                ElectricalStatus::NotAnalyzed => Outcome::NotAnalyzed,
            };
        }
        finding.confirmed_hazard = finding.outcome == Outcome::Fail;
        if finding.outcome != Outcome::NotAnalyzed {
            finding.missing_data.clear();
        }
        finding
            .coverage_gaps
            .extend(depth.applicability_gap.map(str::to_owned));
        let observation = Fingerprint::of(
            format!("{:?}{:?}{id:?}{mark:?}{check:?}", self.build, self.inputs).as_bytes(),
        );
        finding.measurement_sources.push(format!(
            "углублённое наблюдение {:016x}",
            observation.as_u64()
        ));

        let metric = depth_metric(&depth, check);
        if let Some(metric) = &metric {
            checked_pair(metric.observed)?;
            if let Some(limit) = &metric.limit {
                for bounds in limit.lower.into_iter().chain(limit.upper) {
                    checked_pair(bounds)?;
                }
            }
            finding.measurement = Some(metric.observed);
            finding.limit.clone_from(&metric.limit);
            finding.unit = Some(metric.unit.into());
            finding.event_ns = metric.at.map(|at| at.0);
            if let Some(source) = metric.source {
                let source = provenance(&source.provenance);
                finding
                    .limit_sources
                    .push(format!("{:?}: {}", source.kind, source.reference));
            }
        }
        finding.electrical = Some(super::ElectricalFindingIdentity {
            schematic: self.file,
            inputs: self.inputs,
            build: self.build,
            mode: Some(format!("{:?}", depth.mode)),
            probe: Some(branch),
            witness: None,
        });
        finding.depth = Some(DepthFindingIdentity {
            id: id.into(),
            scope: scope.into(),
            passport: depth.passport.clone(),
            applicability: depth.applicability.as_ref().map(provenance),
            observation,
            event: depth.event.as_ref().map(|event| DepthEventIdentity {
                kind: event.kind.code().into(),
                at_ns: event.at.0,
                branch: format!("{:?}", event.branch),
                source: provenance(&event.source),
            }),
            metric: metric.map(|item| item.name),
            related: spec.related.to_vec(),
            legacy_voltage: self.legacy_voltage(&depth, check),
        });
        self.findings.push(finding);
        Ok(depth)
    }

    fn legacy_voltage(
        &self,
        depth: &DepthFinding,
        check: DepthCheck<'_>,
    ) -> Option<LegacyVoltageWitness> {
        // Прежний переход подтверждает только абсолютный пик напряжения на стенде.
        // Пульсации, нагрев, ток отсечки и прочность требуют собственных данных.
        let DepthCheck::CapacitorStress(input, ratings) = check else {
            return None;
        };
        let Some(DepthEvidence::CapacitorStress(evidence)) = &depth.evidence else {
            return None;
        };
        if depth.scope != Scope::Reference
            || depth.applicability_gap.is_some()
            || input.voltage_sense != CapacitorVoltageSense::Absolute
        {
            return None;
        }
        let Analysis::Analyzed { value, .. } = &evidence.voltage_v else {
            return None;
        };
        let rating = ratings.rated_voltage_v?;
        // Единичный derating сохраняет паспортный предел. Модель округляет его наружу.
        if !rating.can_prove()
            || !ratings.voltage_derating.is_some_and(|curve| {
                curve.cells.iter().flatten().all(|range| range.min == 1.0 && range.max == 1.0)
            })
        {
            return None;
        }
        let status = if value.margin.max < 0.0 {
            ElectricalStatus::Fail
        } else if value.margin.min > 0.0 {
            ElectricalStatus::Pass
        } else {
            return None;
        };
        let window = depth.window?;
        let mode = self
            .assessment
            .modes
            .iter()
            .find(|m| m.mode == depth.mode)?;
        if window.start != Time::ZERO || mode.horizon != Some(window.end) {
            return None;
        }
        let journal = self.mode_journal(mode)?;
        let [plus, minus] = &evidence.terminals;
        let conditions = provenance(depth.applicability.as_ref()?).reference;
        self.assessment
            .findings
            .iter()
            .enumerate()
            .find_map(|(index, old)| {
                if old.class != ElectricalClass::InductiveSpike
                    || old.status != status
                    || old.witness.is_none()
                    || old.mode.as_ref() != Some(&depth.mode)
                    || old.conditions.as_deref() != Some(conditions.as_str())
                    || old.limit.as_ref() != Some(rating)
                    || old.unit != Some("V")
                    || !matches!(&old.probe, Some(Probe::Voltage(a, b))
                    if (a == plus && b == minus) || (a == minus && b == plus))
                    || !old.measurement.is_some_and(|[low, high]| {
                        low <= value.observed.max && high >= value.observed.min
                    })
                {
                    return None;
                }
                let extension = format!("{}/{index}", old.class.code());
                let old_finding = self.findings.iter().find(|f| {
                    f.key.class == HazardClass::InductiveSpike
                        && f.key.extension.as_deref() == Some(extension.as_str())
                        && f.journal == Some(journal)
                })?;
                Some(LegacyVoltageWitness {
                    key: old_finding.key.clone(),
                    journal,
                })
            })
    }

    /// Добавляет фактическую шину и фронт сброса из установленного контура.
    /// Один опыт не закрывает огибающую режимов.
    pub fn observe_runtime(
        &mut self, runtime: &Runtime, profile: Fingerprint, horizon: Time,
        resets: &BTreeMap<String, sim_process::schematic::ResetProfile>,
    ) -> Result<(), Error> {
        use sim_runtime::{control::EventKind, electrical::PowerPhase};
        use sim_process::schematic::{Scope, ns_bound};

        let mut thresholds = BTreeMap::new();
        if self.assessment.scope == Scope::Assembly {
            for (board, spec) in resets.iter().filter(|(board, _)| self.boards.contains(*board)) {
                spec.validate().map_err(|err| invalid(err.to_string()))?;
                if let (Some(falling), Some(hold)) = (&spec.falling_v.range, &spec.assert_delay_s.range)
                    && falling.can_prove() && hold.can_prove()
                {
                    let delay = ns_bound(hold.max, true).map_err(|err| invalid(err.to_string()))?;
                    thresholds.insert(board, (falling, hold, delay));
                }
            }
        }

        #[derive(Clone, Copy)]
        struct RailObservation {
            range: [f64; 2],
            min_time: Time,
            worked: bool,
            below: Option<Time>,
            reset: Option<Time>,
            qualified: bool,
        }

        let run = sim_runtime::control::run(runtime).ok_or_else(|| invalid("нет живого контура"))?;
        let identity = run.identity();
        if self.live.is_some() || horizon == Time::ZERO
            || identity.task != profile.as_u64() || identity.criteria != profile.as_u64()
            || identity.phase_horizon != horizon || identity.seed != run.log().header().seed
        {
            return Err(invalid("другой электрический профиль, горизонт, зерно или повтор свидетельства"));
        }
        let end = run.end();
        let journal = Fingerprint::from_fnv1a(run.log().fingerprint());
        let mut observations = BTreeMap::new();
        for event in run.events() {
            match &event.kind {
                EventKind::ElectricalRail { board, voltage_v, phase, .. } if self.boards.contains(board) => {
                    if !voltage_v.is_finite() || event.at > end.time {
                        return Err(invalid("некорректное напряжение или метка электрического журнала"));
                    }
                    let observation = observations.entry(board).or_insert(RailObservation {
                        range: [*voltage_v; 2], min_time: event.at, worked: false,
                        below: None, reset: None, qualified: false,
                    });
                    if *voltage_v < observation.range[0] {
                        observation.range[0] = *voltage_v;
                        observation.min_time = event.at;
                    }
                    observation.range[1] = observation.range[1].max(*voltage_v);
                    observation.worked |= *phase == PowerPhase::Working;
                    if thresholds.get(board).is_some_and(|(falling, _, _)| *voltage_v < falling.min) {
                        observation.below.get_or_insert(event.at);
                    } else {
                        observation.below = None;
                    }
                }
                EventKind::ElectricalReset { board, asserted: true }
                    if let Some(observation) = observations.get_mut(board)
                        && observation.worked && observation.reset.is_none() =>
                {
                    observation.reset = Some(event.at);
                    observation.qualified = observation.below.zip(thresholds.get(board))
                        .is_some_and(|(start, (_, _, delay))| event.at.0.checked_sub(start.0)
                            .is_some_and(|held| held >= *delay));
                }
                _ => {}
            }
        }
        for board in &self.boards {
            let key = RequirementKey {
                class: HazardClass::RailSag,
                object: format!("{}/{board}", self.instance),
                extension: Some("rail_sag/живой опыт".into()),
                run: None,
                window_ns: Some([0, horizon.0]),
                level: super::Level::Simulation,
            };
            let mut finding = Finding::open(&key,
                "нужны применимые порог и выдержка сброса и огибающая режимов питания");
            if let Some(observation) = observations.get(board) {
                finding.measurement = Some(observation.range);
                finding.unit = Some("V".into());
                finding.event_ns = Some(observation.reset.unwrap_or(observation.min_time).0);
                if observation.qualified && let Some((falling, hold, _)) = thresholds.get(board) {
                    finding.outcome = Outcome::Fail;
                    finding.confirmed_hazard = true;
                    finding.reason = "после рабочей фазы шина удерживалась ниже порога и плата сбросилась".into();
                    finding.missing_data.clear();
                    finding.limit = Some(LimitValue { lower: Some([falling.min, falling.max]), upper: None });
                    finding.limit_sources.extend([falling, hold].into_iter().map(|range| {
                        let source = provenance(&range.provenance);
                        format!("{:?}: {}", source.kind, source.reference)
                    }));
                }
            } else {
                finding.coverage_gaps.push("нет отсчётов локальной шины платы".into());
            }
            if !run.reached_horizon() || end.steps == 0 {
                finding.coverage_gaps.push(format!("журнал достиг {} из {} нс", end.time.0, horizon.0));
            }
            finding.journal = Some(journal);
            finding.electrical = Some(super::ElectricalFindingIdentity {
                schematic: self.file, inputs: self.inputs, build: self.build,
                mode: Some("живой опыт".into()), probe: Some(format!("шина {board}")),
                witness: observations.get(board).and_then(|observation| {
                    observation.reset.map(|at| format!("сброс после рабочей фазы на {} нс", at.0))
                }),
            });
            self.findings.push(finding);
        }
        if !run.reached_horizon() || end.steps == 0 {
            let key = RequirementKey {
                class: HazardClass::Execution,
                object: format!("{}/электрический журнал", self.instance),
                extension: Some("электрическая схема/горизонт".into()),
                run: None, window_ns: Some([0, horizon.0]), level: super::Level::Simulation,
            };
            let mut finding = Finding::open(&key, format!("журнал достиг {} из {} нс", end.time.0, horizon.0));
            finding.journal = Some(journal);
            finding.electrical = Some(super::ElectricalFindingIdentity {
                schematic: self.file, inputs: self.inputs, build: self.build,
                mode: Some("живой опыт".into()), probe: None, witness: None,
            });
            self.findings.push(finding);
        }
        self.live = Some(journal);
        Ok(())
    }

    pub fn assessment(&self) -> &sim_process::schematic::ElectricalAssessment {
        &self.assessment
    }

    pub fn mode_journal(&self, mode: &sim_process::schematic::ElectricalMode) -> Option<Fingerprint> {
        self.assessment.modes.iter().find(|own| own.mode == mode.mode)
            .and_then(|own| own.journal).map(|journal| electrical_journal(self.build, self.inputs, journal))
    }

    pub fn findings(&self) -> &[Finding] {
        &self.findings
    }

    /// Публикует незакрытое обязательство заявленной платы.
    pub fn record_pcb_open(
        &mut self,
        class: HazardClass,
        id: &str,
        pcb: Fingerprint,
        reason: &str,
    ) -> Result<(), Error> {
        let finding = self.pcb_finding(class, id, pcb, reason)?;
        self.findings.push(finding);
        Ok(())
    }

    /// Считает и публикует воздушный зазор и путь утечки указанной пары.
    /// Возвращает расчёт сравнителя. Уровень доказательства хранится в `findings()`.
    pub fn record_pcb_clearance(
        &mut self,
        id: &str,
        pcb: Fingerprint,
        layout: &Layout,
        face: clearance::Face,
        nets: [u32; 2],
        conditions: Option<&clearance::Conditions<'_>>,
        limits: Option<&clearance::Limits<'_>>,
        topology_gap: Option<&str>,
    ) -> Result<clearance::ClearanceReport, Error> {
        let scope = match self.assessment.scope {
            Scope::Assembly => clearance::Scope::Sourced,
            Scope::Reference => clearance::Scope::Reference,
        };
        if conditions.is_some_and(|c| c.scope != scope) || limits.is_some_and(|l| l.envelope.scope != scope) {
            return Err(invalid("область условий изоляции расходится со схемой"));
        }
        let report = clearance::evaluate(layout, face, nets, conditions, limits)
            .map_err(|e| invalid(e.to_string()))?;
        let (criterion, check) = if report.clearance.status == clearance::Status::Fail
            || report.creepage.status == clearance::Status::Pass
        {
            ("воздушный зазор", &report.clearance)
        } else {
            ("путь утечки", &report.creepage)
        };
        let class = HazardClass::CreepageClearance;
        let reason = match report.status() {
            clearance::Status::Fail => format!("{criterion} меньше предела"),
            clearance::Status::Pass => "оба расстояния в пределах расчётного опыта".into(),
            clearance::Status::NotAnalyzed => report.clearance.open.iter().chain(&report.creepage.open)
                .map(ToString::to_string).collect::<BTreeSet<_>>().into_iter().collect::<Vec<_>>().join(", "),
        };
        let mut finding = self.pcb_finding(class, id, pcb, &reason)?;
        finding.measurement = check.distance.and_then(|d| Some([d.lower_m, d.upper_m?]));
        finding.limit = check.minimum_m.map(|minimum| LimitValue {
            lower: Some([minimum; 2]),
            upper: None,
        });
        finding.unit = Some("m".into());
        finding.measurement_sources.extend(conditions.map(|c| c.source.into()));
        finding.limit_sources.extend(limits.map(|l| l.envelope.source.into()));
        let outcome = match report.status() {
            clearance::Status::Pass => Outcome::Pass,
            clearance::Status::Fail => Outcome::Fail,
            clearance::Status::NotAnalyzed => Outcome::NotAnalyzed,
        };
        self.finish_pcb_finding(finding, outcome, topology_gap);
        Ok(report)
    }

    /// Считает и публикует смещение опоры по связанным маршрутам возврата.
    /// Возвращает расчёт сравнителя. Уровень доказательства хранится в `findings()`.
    pub fn record_pcb_ground_bounce(
        &mut self,
        id: &str,
        pcb: Fingerprint,
        layout: &Layout,
        input: &return_path::GroundBounceInput<'_>,
        topology_gap: Option<&str>,
    ) -> Result<return_path::GroundBounceReport, Error> {
        let scope = match self.assessment.scope {
            Scope::Assembly => ratings::Scope::Assembly,
            Scope::Reference => ratings::Scope::Reference,
        };
        if input.scope != scope {
            return Err(invalid("область расчёта возврата расходится со схемой"));
        }
        let report = return_path::check_ground_bounce(layout, input)
            .map_err(|e| invalid(e.to_string()))?;
        let (outcome, reason) = match &report.status {
            return_path::Status::Pass => (Outcome::Pass, "смещение опоры в пределах расчётного опыта".into()),
            return_path::Status::Fail => (Outcome::Fail, "смещение опоры выше предела приёмника".into()),
            return_path::Status::NotAnalyzed(reason) => (Outcome::NotAnalyzed, reason.to_string()),
        };
        let mut finding = self.pcb_finding(HazardClass::GroundBounce, id, pcb, &reason)?;
        finding.measurement = report.bounds_v;
        if let Some(limit) = input.max_bounce_v {
            let (ratings::Source::Reference(source) | ratings::Source::Verified(source)
                | ratings::Source::Estimate(source)) = limit.source;
            finding.limit = Some(LimitValue::upper([limit.value; 2]));
            finding.limit_sources.push(source.into());
        }
        finding.unit = Some("V".into());
        self.finish_pcb_finding(finding, outcome, topology_gap);
        Ok(report)
    }

    fn pcb_finding(
        &self, class: HazardClass, id: &str, pcb: Fingerprint, reason: &str,
    ) -> Result<Finding, Error> {
        if class.pcb_measurement().is_none() {
            return Err(invalid("неизвестный код проверки платы"));
        }
        crate::task::check_name(id)?;
        let extension = format!("pcb/{}/{id}", class.spec().id);
        if id.contains('/') || id.chars().any(char::is_control) || reason.trim().is_empty()
            || self.findings.iter().any(|finding| finding.key.extension.as_deref() == Some(&extension))
        {
            return Err(invalid("пустая причина или некорректное либо повторное имя проверки платы"));
        }
        let key = RequirementKey {
            class,
            object: self.instance.clone(),
            extension: Some(extension),
            run: None,
            window_ns: None,
            level: super::Level::Simulation,
        };
        let mut finding = Finding::open(&key, reason);
        finding.model_scope = Some(match self.assessment.scope {
            Scope::Reference => "расчётный эталон платы",
            Scope::Assembly => "инженерная оценка платы для указанной ревизии и установки",
        }.into());
        finding.measurement_sources.push(format!("профиль платы {:016x}", pcb.as_u64()));
        finding.electrical = Some(super::ElectricalFindingIdentity {
            schematic: self.file,
            inputs: Fingerprint::of(format!("{:?}{pcb:?}", self.inputs).as_bytes()),
            build: self.build,
            mode: Some("плата".into()),
            probe: Some(id.into()),
            witness: None,
        });
        Ok(finding)
    }

    fn finish_pcb_finding(&mut self, mut finding: Finding, outcome: Outcome, topology_gap: Option<&str>) {
        let gap = topology_gap.or_else(|| {
            (self.assessment.scope == Scope::Assembly).then(|| {
                finding.key.class.pcb_measurement().expect("код проверки платы")
            })
        });
        finding.coverage_gaps.extend(topology_gap.map(str::to_owned));
        if let Some(reason) = gap.filter(|_| outcome == Outcome::Pass) {
            finding.reason = reason.into();
            finding.missing_data = vec![reason.into()];
        } else {
            finding.outcome = outcome;
            finding.confirmed_hazard = outcome == Outcome::Fail;
            if outcome != Outcome::NotAnalyzed {
                finding.missing_data.clear();
            }
        }
        self.findings.push(finding);
    }

    /// Открытое обязательство для явно заявленной проверки ЭМС.
    pub fn record_emc_open(
        &mut self,
        class: HazardClass,
        id: &str,
        profile: Fingerprint,
        reason: &str,
    ) -> Result<(), Error> {
        let finding = self.emc_finding(class, id, profile, class.spec().id, reason)?;
        self.findings.push(finding);
        Ok(())
    }

    /// Публикует результат схемного оценщика контактного разряда.
    /// Численное подтверждение запаса оставляет обязательство измерения на стенде.
    pub fn record_esd_event<E>(
        &mut self,
        compiled: &CompiledSchematic,
        plan: &ModePlan,
        id: &str,
        profile: Fingerprint,
        assess: impl FnOnce(
            &CompiledSchematic,
        )
            -> Result<sim_process::schematic::Finding<E>, sim_process::ProcessError>,
    ) -> Result<sim_process::schematic::Finding<E>, Error> {
        self.check_emc_inputs(compiled, plan)?;
        let mut result = assess(compiled).map_err(|err| invalid(err.to_string()))?;
        if !compiled.pins().any(|end| end.to_string() == result.subject) {
            return Err(invalid("свидетельство ESD не адресует вывод схемы"));
        }
        if result.status == ElectricalStatus::Pass {
            result.status = ElectricalStatus::NotAnalyzed;
            result.reason =
                "нужен стендовый замер остаточных напряжения и тока на защищаемом выводе".into();
        }
        let mut finding = self.emc_finding(
            HazardClass::EsdEvent,
            id,
            profile,
            &result.subject,
            &result.reason,
        )?;
        finding.model_scope = Some(
            "одиночный контактный разряд на адресованных клеммах испытательной установки".into(),
        );
        if result.status == ElectricalStatus::Fail {
            finding.outcome = Outcome::Fail;
            finding.confirmed_hazard = true;
            finding.missing_data.clear();
        } else {
            finding
                .missing_data
                .extend(HazardClass::EsdEvent.emc_measurement().map(str::to_owned));
        }
        self.findings.push(finding);
        Ok(result)
    }

    /// Рассчитывает запас входа и проверяет наблюдение по паре живых запусков.
    /// Отсутствие пары оставляет обязательство открытым даже при численном запасе.
    #[allow(clippy::too_many_arguments)]
    pub fn record_emi_reset(
        &mut self,
        compiled: &CompiledSchematic,
        plan: &ModePlan,
        id: &str,
        profile: Fingerprint,
        input: EmiInput,
        pair: Option<(&Runtime, &Runtime)>,
        packet: Option<&EmiPacketProbe>,
    ) -> Result<Analysis<EmiEvidence>, Error> {
        self.check_emc_inputs(compiled, plan)?;
        if input.scope != compiled.depth_scope()
            || !self.boards.contains(&input.board)
            || !compiled.pins().any(|end| end.component == input.source)
            || ![&input.input, &input.reference]
                .into_iter()
                .all(|target| compiled.pins().any(|end| end == target))
        {
            return Err(invalid(
                "источник, плата или клеммы EMI не соответствуют схеме",
            ));
        }
        if let Some(packet) = packet {
            packet
                .validate(input.scope)
                .map_err(|err| invalid(err.to_string()))?;
        }
        let object = format!("{} относительно {}", input.input, input.reference);
        let window = [input.start.0, input.end.0];
        let mut finding = self.emc_finding(
            HazardClass::EmiReset,
            id,
            profile,
            &object,
            "нужен синхронный стендовый замер дифференциального напряжения, сброса и доставки",
        )?;
        finding.key.window_ns = Some(window);
        let trial = match prepare_emi_reset(input).map_err(|err| invalid(err.to_string()))? {
            Analysis::NotAnalyzed { reason } => {
                finding.reason.clone_from(&reason);
                finding.missing_data.push(reason.clone());
                self.findings.push(finding);
                return Ok(Analysis::NotAnalyzed { reason });
            }
            Analysis::Analyzed { value, .. } => value,
        };
        let evidence = if let Some((active, baseline)) = pair {
            let active_run = sim_runtime::control::run(active)
                .ok_or_else(|| invalid("нет журнала воздействия"))?;
            let quiet_run = sim_runtime::control::run(baseline)
                .ok_or_else(|| invalid("нет контрольного журнала"))?;
            let evidence = active
                .compare_emi_reset(baseline, packet)
                .map_err(invalid)?;
            let expected = assess_emi_reset(&trial, evidence.observation);
            if evidence.source != expected.source
                || evidence.source_kind != expected.source_kind
                || evidence.board != expected.board
                || evidence.input != expected.input
                || evidence.reference != expected.reference
                || evidence.scope != expected.scope
                || evidence.margins != expected.margins
                || evidence.criterion != expected.criterion
                || evidence.outcome != expected.outcome
            {
                return Err(invalid("живая пара EMI относится к другой постановке"));
            }
            let active_journal = active_run.log().fingerprint();
            let quiet_journal = quiet_run.log().fingerprint();
            finding.journal = Some(Fingerprint::from_fnv1a(active_journal));
            finding
                .measurement_sources
                .push(format!("контрольный журнал {quiet_journal:016x}"));
            evidence
        } else {
            let mut evidence = assess_emi_reset(&trial, None);
            evidence.outcome = EmiOutcome::NotAnalyzed {
                reason: "нужны связанный опыт и контроль с той же установкой и синхронный стендовый замер входа и сброса".into(),
            };
            evidence
        };
        let bounds = [
            evidence.margins.voltage_v.lower,
            evidence.margins.voltage_v.upper,
        ];
        checked_pair(bounds)?;
        finding.measurement = Some(bounds);
        finding.limit = Some(LimitValue {
            lower: Some([0.0; 2]),
            upper: None,
        }); // Граница знакового запаса.
        finding.unit = Some("V".into());
        let scope = "дифференциальный вход сброса в указанных монтаже, полосе и окне";
        finding.model_scope = Some(scope.into());
        if let Some(identity) = &mut finding.electrical {
            identity.witness = evidence.criterion.map(|criterion| format!("{criterion:?}"));
        }
        match &evidence.outcome {
            EmiOutcome::Fail { at } => {
                finding.outcome = Outcome::Fail;
                finding.confirmed_hazard = true;
                finding.event_ns = Some(at.0);
                finding.reason =
                    "возмущение вызвало подтверждённый сброс адресованной платы".into();
                finding.missing_data.clear();
            }
            EmiOutcome::NotAnalyzed { reason } => {
                finding.reason.clone_from(reason);
                finding.missing_data.push(reason.clone());
            }
        }
        finding
            .coverage_gaps
            .extend(evidence.packet_open.iter().cloned());
        self.findings.push(finding);
        Ok(Analysis::Analyzed {
            value: evidence,
            scope: scope.into(),
        })
    }

    fn check_emc_inputs(&self, compiled: &CompiledSchematic, plan: &ModePlan) -> Result<(), Error> {
        if self.inputs != electrical_inputs(self.file, compiled, plan) {
            return Err(invalid(
                "свидетельство ЭМС относится к другой схеме, каталогу или режимам",
            ));
        }
        Ok(())
    }

    fn emc_finding(
        &self,
        class: HazardClass,
        id: &str,
        profile: Fingerprint,
        object: &str,
        reason: &str,
    ) -> Result<Finding, Error> {
        if class.emc_measurement().is_none() {
            return Err(invalid("неизвестный код ЭМС"));
        }
        crate::task::check_name(id)?;
        if id.contains('/') || id.chars().any(char::is_control) {
            return Err(invalid("некорректное имя проверки ЭМС"));
        }
        let extension = format!("emc/{}/{id}", class.spec().id);
        if self
            .findings
            .iter()
            .any(|item| item.key.extension.as_deref() == Some(&extension))
        {
            return Err(invalid("повтор имени проверки ЭМС"));
        }
        let key = RequirementKey {
            class,
            object: format!("{}/{object}", self.instance),
            extension: Some(extension),
            run: None,
            window_ns: None,
            level: super::Level::Simulation,
        };
        let mut finding = Finding::open(&key, reason);
        finding
            .measurement_sources
            .push(format!("профиль ЭМС {:016x}", profile.as_u64()));
        finding.electrical = Some(super::ElectricalFindingIdentity {
            schematic: self.file,
            inputs: self.inputs,
            build: self.build,
            mode: Some("ЭМС".into()),
            probe: Some(object.into()),
            witness: None,
        });
        Ok(finding)
    }

    pub(super) fn inapplicable(&self) -> impl Iterator<Item = super::NotApplicable> + '_ {
        self.assessment.inapplicable.iter().map(|(&class, reason)| super::NotApplicable {
            class: class.into(),
            code: Some(class.code().into()),
            object: Some(self.instance.clone()),
            reason: reason.clone(),
            sources: vec![format!("схема {:016x}, входы {:016x}", self.file.as_u64(), self.inputs.as_u64())],
        })
    }

    pub fn missing(instance: &str, reason: &str) -> Vec<Finding> {
        sim_process::schematic::ElectricalClass::ALL.into_iter().map(|class| {
            let key = RequirementKey {
                class: class.into(), object: instance.into(), extension: Some(class.code().into()),
                run: None, window_ns: None, level: super::Level::Simulation,
            };
            let mut finding = Finding::open(&key, reason);
            finding.code = class.code().into();
            finding
        }).collect()
    }

    pub fn verify(
        &self, instance: &str, build: Option<Fingerprint>,
        device_path: &std::path::Path, device: &sim_world::Device,
    ) -> Result<(), Error> {
        let file = device.schematic.as_ref().ok_or_else(|| invalid("у экземпляра нет ссылки на схему"))?;
        let path = device_path.parent().unwrap_or_else(|| std::path::Path::new(".")).join(file);
        let bytes = std::fs::read(&path).map_err(|err| invalid(format!("{}: {err}", path.display())))?;
        if self.instance != instance || Some(self.build) != build || self.file != Fingerprint::of(&bytes) {
            return Err(invalid("электрическое доказательство относится к другому экземпляру, версии или схеме"));
        }
        Ok(())
    }
}

fn electrical_inputs(
    file: Fingerprint,
    compiled: &CompiledSchematic,
    plan: &ModePlan,
) -> Fingerprint {
    Fingerprint::of(format!("{file:?}{compiled:?}{plan:?}").as_bytes())
}

struct DepthMetric<'a> {
    name: String,
    observed: [f64; 2],
    unit: &'static str,
    limit: Option<LimitValue>,
    at: Option<Time>,
    source: Option<&'a SourcedRange>,
}

impl<'a> DepthMetric<'a> {
    fn new(name: impl Into<String>, observed: [f64; 2], unit: &'static str) -> Self {
        Self {
            name: name.into(),
            observed,
            unit,
            limit: None,
            at: None,
            source: None,
        }
    }

    fn upper(mut self, bounds: [f64; 2], source: Option<&'a SourcedRange>) -> Self {
        self.limit = Some(LimitValue::upper(bounds));
        self.source = source;
        self
    }

    fn lower(mut self, bounds: [f64; 2], source: Option<&'a SourcedRange>) -> Self {
        self.limit = Some(LimitValue {
            lower: Some(bounds),
            upper: None,
        });
        self.source = source;
        self
    }
}

fn analyzed<T>(analysis: &Analysis<T>) -> Option<&T> {
    match analysis {
        Analysis::Analyzed { value, .. } => Some(value),
        Analysis::NotAnalyzed { .. } => None,
    }
}

fn depth_metric<'a>(depth: &DepthFinding, check: DepthCheck<'a>) -> Option<DepthMetric<'a>> {
    match (depth.evidence.as_ref()?, check) {
        (DepthEvidence::CapacitorStress(result), DepthCheck::CapacitorStress(_, ratings)) => [
            (
                "пиковое напряжение",
                &result.voltage_v,
                "V",
                result.voltage_peak_sample,
                None,
            ),
            ("RMS пульсаций", &result.ripple_a, "A", None, None),
            (
                "пик температуры",
                &result.hotspot_k,
                "K",
                None,
                ratings.max_hotspot_k,
            ),
        ]
        .into_iter()
        .filter_map(|(label, analysis, unit, at, source)| {
            let value = analyzed(analysis)?;
            let mut metric =
                DepthMetric::new(label, [value.observed.min, value.observed.max], unit)
                    .upper([value.limit.min, value.limit.max], source);
            metric.at = at;
            Some((value.margin.max < 0.0, metric))
        })
        .max_by_key(|(violation, _)| *violation)
        .map(|(_, metric)| metric),
        (DepthEvidence::FuseCoordination(result), DepthCheck::FuseCoordination(input, ratings)) => {
            if let Some(interrupt) = ratings.interrupt {
                for (status, name, observed, unit, limit) in [
                    (
                        result.interrupt_current,
                        "отключающий ток",
                        input.prospective_current_a,
                        "A",
                        interrupt.max_prospective_current_a,
                    ),
                    (
                        result.interrupt_voltage,
                        "восстанавливающееся напряжение",
                        input.recovery_voltage_v,
                        "V",
                        interrupt.max_recovery_voltage_v,
                    ),
                ] {
                    if status == ElectricalStatus::Fail {
                        let observed = observed?;
                        return Some(
                            DepthMetric::new(name, [observed.min, observed.max], unit)
                                .upper([limit.min, limit.max], Some(limit)),
                        );
                    }
                }
            }
            let clear = analyzed(&result.clearing_time_s)?;
            let times = [
                (
                    "время полного отключения до повреждения провода",
                    result.wire_margin_s,
                    analyzed(&result.wire).map(|wire| wire.damage_time_s),
                ),
                (
                    "время полного отключения до повреждения ключа",
                    result.switch_margin_s,
                    analyzed(&result.switch).copied(),
                ),
            ];
            let (label, damage) = times
                .into_iter()
                .filter_map(|(label, margin, damage)| Some((label, margin?, damage?)))
                .min_by(|(_, a, _), (_, b, _)| {
                    let violated =
                        |m: &sim_process::schematic::fuse_coordination::FuseTimeMargin| {
                            m.max.is_some_and(|max| max < 0.0)
                        };
                    violated(b).cmp(&violated(a)).then(a.min.total_cmp(&b.min))
                })
                .map(|(label, _, damage)| (label, Some(damage)))
                .unwrap_or(("время полного отключения", None));
            let mut metric = DepthMetric::new(label, [clear.min, clear.max], "s");
            if let Some(damage) = damage
                && let Some(latest) = damage.latest_s
            {
                metric = metric.upper([damage.earliest_s, latest], None);
            }
            Some(metric)
        }
        (DepthEvidence::SwitchThermal(result), DepthCheck::SwitchThermal(_, ratings)) => {
            if depth.status == ElectricalStatus::Fail
                && let Some(pulse) = analyzed(&result.linear_soa)
                    .into_iter()
                    .flatten()
                    .find(|pulse| pulse.status == ElectricalStatus::Fail)
                && let Some(&[SwitchSoaTime::Seconds(lo), SwitchSoaTime::Seconds(hi)]) =
                    analyzed(&pulse.allowed)
            {
                return Some(
                    DepthMetric::new(
                        "длительность линейного режима",
                        [pulse.duration_s.min, pulse.duration_s.max],
                        "s",
                    )
                    .upper([lo, hi], None),
                );
            }
            let (name, value) = if result.sustained
                && let Some(steady) = analyzed(&result.steady_junction_k)
                && steady.margin.max < 0.0
                && depth.status == ElectricalStatus::Fail
            {
                ("стационарная Tj", steady)
            } else {
                ("пик Tj", analyzed(&result.foster_peak_junction_k)?)
            };
            Some(
                DepthMetric::new(name, [value.observed.min, value.observed.max], "K")
                    .upper([value.limit.min, value.limit.max], ratings.max_junction_k),
            )
        }
        (DepthEvidence::DeepDischarge(result), DepthCheck::DeepDischarge(input)) => {
            let cell = result
                .cells
                .iter()
                .find(|cell| cell.violation.is_some())
                .or_else(|| {
                    result
                        .cells
                        .iter()
                        .find(|cell| Some(&cell.id) == result.limiting_cell.as_ref())
                })?;
            let limit = input
                .cells?
                .iter()
                .find(|input| input.id == cell.id)?
                .min_voltage_v?;
            let (observed, at) = match cell.violation {
                Some(violation) => (violation.voltage_v, Some(violation.time)),
                None => (*analyzed(&cell.minimum_voltage_v)?, None),
            };
            let mut metric = DepthMetric::new(
                format!("минимум напряжения ячейки {}", cell.id),
                [observed.min, observed.max],
                "V",
            )
            .lower([limit.min, limit.max], Some(limit));
            metric.at = at;
            Some(metric)
        }
        (DepthEvidence::SleepDrain(result), DepthCheck::SleepDrain(_)) => {
            // Запас определён как доступная величина минус требуемая. Ноль означает равенство.
            if let Some(violation) = &result.cutoff_violation {
                let mut metric = DepthMetric::new(
                    "запас до порога отсечки",
                    [violation.margin_v.min, violation.margin_v.max],
                    "V",
                )
                .lower([0.0; 2], None);
                metric.at = Some(violation.time);
                return Some(metric);
            }
            let reserve = analyzed(&result.charge_margin_ah)?;
            Some(
                DepthMetric::new("запас заряда", [reserve.min, reserve.max], "Ah")
                    .lower([0.0; 2], None),
            )
        }
        (DepthEvidence::CableFlex(result), DepthCheck::CableFlex(_, ratings)) => {
            if let Some(end) = result
                .relief
                .iter()
                .position(|&s| s == ElectricalStatus::Fail)
            {
                // Двоичная отметка установки. Единица означает подтверждённое крепление.
                return Some(
                    DepthMetric::new(format!("разгрузка конца {}", end + 1), [0.0; 2], "bool")
                        .lower([1.0; 2], None),
                );
            }
            if result.fatigue.status == ElectricalStatus::Fail {
                let margin = analyzed(&result.fatigue.result)?;
                return Some(
                    DepthMetric::new(
                        "запас радиуса по кривой гибкости",
                        [margin.min_m?, margin.max_m?],
                        "m",
                    )
                    .lower([0.0; 2], None),
                );
            }
            if result.slack.status != ElectricalStatus::Fail
                && let Some(radius) = analyzed(&result.minimum_radius_m)
                && let (Some(lo), Some(hi)) = (radius.min_m, radius.max_m)
                && let Some(limit) = ratings.min_bend_radius_m
                && analyzed(&result.bend.result).is_some()
            {
                return Some(
                    DepthMetric::new("минимальный радиус изгиба", [lo, hi], "m")
                        .lower([limit.min, limit.max], Some(limit)),
                );
            }
            let length = analyzed(&result.maximum_length_m)?;
            let limit = ratings
                .usable_length_m
                .filter(|_| analyzed(&result.slack.result).is_some())?;
            let mut metric = DepthMetric::new(
                "максимальная длина трассы",
                [length.bounds.min, length.bounds.max],
                "m",
            )
            .upper([limit.min, limit.max], Some(limit));
            metric.at = Some(length.sample_at);
            Some(metric)
        }
        _ => None,
    }
}

fn electrical_journal(build: Fingerprint, inputs: Fingerprint, journal: u64) -> Fingerprint {
    Fingerprint::of(format!("{build:?}{inputs:?}{journal}").as_bytes())
}

/// Неподменяемые входы оценщика. Дисковый вердикт и статистические строки сюда не загружаются.
#[derive(Debug)]
pub struct CampaignEvidence {
    pub(super) stamp: Fingerprint,
    runs: BTreeMap<RunKey, Run>,
    unavailable: BTreeMap<RunKey, Vec<String>>,
}

impl CampaignEvidence {
    pub fn new(requirements: &RequirementSet) -> Self {
        Self {
            stamp: requirements.stamp,
            runs: BTreeMap::new(),
            unavailable: BTreeMap::new(),
        }
    }

    /// Объединяет независимо исполненные строки одного набора требований.
    /// При несовпадении требований или повторе ключа накопленные данные не меняются.
    pub fn merge(&mut self, mut other: Self) -> Result<(), Error> {
        if self.stamp != other.stamp {
            return Err(invalid("другой набор требований кампании"));
        }
        if other
            .runs
            .keys()
            .chain(other.unavailable.keys())
            .any(|key| self.runs.contains_key(key) || self.unavailable.contains_key(key))
        {
            return Err(invalid("повтор ключа доказательства кампании"));
        }
        self.runs.append(&mut other.runs);
        self.unavailable.append(&mut other.unavailable);
        Ok(())
    }

    /// Регистрирует причину незапущенной строки, например ошибку установки или фильтрацию.
    pub fn record_unavailable(
        &mut self,
        requirements: &RequirementSet,
        key: &RunKey,
        reason: String,
    ) -> Result<(), Error> {
        self.matches(requirements)?;
        crate::task::check_name(&reason)?;
        if self.runs.contains_key(key) || !requirements.keys().any(|k| k.run.as_ref() == Some(key))
        {
            return Err(invalid(
                "строка отсутствует в обязательствах или уже исполнена",
            ));
        }
        self.unavailable
            .entry(key.clone())
            .or_default()
            .push(reason);
        Ok(())
    }

    fn matches(&self, requirements: &RequirementSet) -> Result<(), Error> {
        if self.stamp == requirements.stamp {
            Ok(())
        } else {
            Err(invalid("другой набор требований кампании"))
        }
    }

    /// Дважды исполняет одну строку на свежих экземплярах. Инсталлятор регистрирует
    /// компоненты, подтверждает фактические установки и планирует действия режима.
    /// Сборщик не имеет права заменять экземпляр, загружать лог или шагать сам.
    pub fn execute(
        &mut self,
        requirements: &RequirementSet,
        plan: &CampaignPlan,
        key: &RunKey,
        install: impl FnMut(&mut Runtime, &SampledInputs) -> Result<Installed, String>,
    ) -> Result<(), Error> {
        self.execute_inner(
            requirements,
            plan,
            key,
            None,
            install,
            |_, _| Ok(None),
            false,
            None,
            |_| Ok(()),
        )
    }

    /// Оценщик сам исполняет шаги усилия. Прочие операции передаются
    /// адаптеру в заданное время, а их принятие сверяется с расписанием.
    pub fn execute_timed(
        &mut self,
        requirements: &RequirementSet,
        plan: &CampaignPlan,
        key: &RunKey,
        install: impl FnMut(&mut Runtime, &SampledInputs) -> Result<Installed, String>,
        operation: impl FnMut(&mut Runtime, &Step) -> Result<Option<StepObservation>, String>,
    ) -> Result<(), Error> {
        self.execute_inner(
            requirements,
            plan,
            key,
            None,
            install,
            operation,
            true,
            None,
            |_| Ok(()),
        )
    }

    /// Исполняет сценарий и проверяет продолжение со снимка в том же экземпляре.
    pub fn execute_with_snapshot(
        &mut self,
        requirements: &RequirementSet,
        plan: &CampaignPlan,
        key: &RunKey,
        at: Time,
        install: impl FnMut(&mut Runtime, &SampledInputs) -> Result<Installed, String>,
    ) -> Result<(), Error> {
        self.execute_scenario_observed(requirements, plan, key, Some(at), install, |_| Ok(()))
    }

    /// Передаёт наблюдателю живой экземпляр, журнал которого идёт в оценку.
    pub fn execute_scenario_observed(
        &mut self,
        requirements: &RequirementSet,
        plan: &CampaignPlan,
        key: &RunKey,
        snapshot_at: Option<Time>,
        install: impl FnMut(&mut Runtime, &SampledInputs) -> Result<Installed, String>,
        observe: impl FnOnce(&Runtime) -> Result<(), String>,
    ) -> Result<(), Error> {
        self.execute_inner(
            requirements,
            plan,
            key,
            None,
            install,
            |_, _| Ok(None),
            true,
            snapshot_at,
            observe,
        )
    }

    pub fn scenario_report(
        &self,
        key: &RunKey,
    ) -> Option<(&RunSummary, u64, u64, Option<&SnapshotAudit>)> {
        let run = self.runs.get(key)?;
        Some((
            run.summary.as_ref()?,
            run.log.fingerprint,
            run.log.end.steps,
            run.snapshot.as_ref(),
        ))
    }

    /// Настройки, считанные из живого рантайма и сверенные с постановкой.
    pub fn confirmed_settings(&self, key: &RunKey) -> Option<&ModelSettings> {
        let run = self.runs.get(key)?;
        run.installation_confirmed.then_some(&run.sampled.settings)
    }

    /// Для связанного профиля допустим расширенный мир с точным отпечатком из инвентаря.
    pub fn execute_linked(
        &mut self,
        requirements: &RequirementSet,
        plan: &CampaignPlan,
        key: &RunKey,
        world: sim_world::CompiledWorld,
        install: impl FnMut(&mut Runtime, &SampledInputs) -> Result<Installed, String>,
    ) -> Result<(), Error> {
        self.execute_inner(
            requirements,
            plan,
            key,
            Some(world),
            install,
            |_, _| Ok(None),
            false,
            None,
            |_| Ok(()),
        )
    }

    fn execute_inner(
        &mut self,
        requirements: &RequirementSet,
        plan: &CampaignPlan,
        key: &RunKey,
        expanded: Option<sim_world::CompiledWorld>,
        mut install: impl FnMut(&mut Runtime, &SampledInputs) -> Result<Installed, String>,
        mut operation: impl FnMut(&mut Runtime, &Step) -> Result<Option<StepObservation>, String>,
        timed: bool,
        snapshot_at: Option<Time>,
        observe: impl FnOnce(&Runtime) -> Result<(), String>,
    ) -> Result<(), Error> {
        self.matches(requirements)?;
        if plan.fingerprint() != requirements.plan || self.runs.contains_key(key) {
            return Err(invalid("другой план или повторная регистрация ключа"));
        }
        let row = plan
            .row(key)
            .ok_or_else(|| invalid("ключ отсутствует в плане"))?;
        if !matches!(row.disposition, Disposition::Ready) {
            return Err(invalid("строка плана не готова к исполнению"));
        }
        let sampled = plan.materialize(key)?;
        let mode = sampled
            .mode
            .as_ref()
            .ok_or_else(|| invalid("нет обязательного режима"))?;
        if snapshot_at.is_some_and(|at| at.0 == 0 || at.0 >= mode.horizon_ns) {
            return Err(invalid("снимок должен находиться внутри горизонта"));
        }
        let compiled = match (expanded, requirements.compiled_rows.get(key)) {
            (Some(world), Some(expected))
                if compiled_world_fingerprint(&world) == *expected
                    && world.seed == sampled.sample.seeds.run
                    && world.physics.as_hz() == row.grid.hz as f64 =>
            {
                world
            }
            (None, None) => sampled.world.compile()?,
            _ => {
                return Err(invalid(
                    "расширенный мир не совпал с разрешённой установкой",
                ));
            }
        };
        crate::validate_world_numbers(&compiled)?;
        let mut profiles: BTreeMap<_, BTreeSet<_>> = BTreeMap::new();
        let mut section_channels = BTreeSet::new();
        let mut section_targets = BTreeSet::new();
        let mut strength_bodies = BTreeSet::new();
        let mut kinematic_check = false;
        for (object, check) in requirements
            .requirements
            .iter()
            .filter(|r| r.key.run.as_ref() == Some(key))
            .flat_map(|r| r.checks.iter().map(|check| (r.key.object.as_str(), check)))
        {
            let (kind, fingerprint) = match check.verification {
                Verification::Section { ref live_load, .. } => {
                    strength_bodies.insert(object);
                    if let Some(live) = live_load
                        && compiled.efforts.iter().any(|effort| effort.name == live.effort
                            && effort.joint == object && effort.dof == 0)
                    {
                        section_channels.insert(live.effort.as_str());
                        section_targets.insert(object);
                    }
                    continue;
                }
                Verification::Truth(_) if matches!(check.class,
                    HazardClass::TravelJam | HazardClass::Fall | HazardClass::DangerousMotion) => {
                    kinematic_check = true;
                    continue;
                }
                Verification::Process { profile, .. } => (ReferenceKind::ProcessTask, profile),
                Verification::Interaction { profile, .. } => {
                    (ReferenceKind::InteractionTask, profile)
                }
                Verification::Control { criteria, .. } => (ReferenceKind::ControlTask, criteria),
                _ => continue,
            };
            profiles.entry(kind).or_default().insert(fingerprint);
        }
        let world_matches = !requirements.compiled_rows.contains_key(key) || sampled.world.compile()? == compiled;
        let (engine_joints, strength_bodies) = if kinematic_check || (world_matches && !strength_bodies.is_empty()) {
            let sim = compiled.open_contact_sim().map_err(invalid)?;
            let bodies = strength_bodies.into_iter()
                .filter(|_| world_matches)
                .filter_map(|name| sim.id(sim_mujoco::ffi::mjtObj::mjOBJ_BODY, name)
                    .map(|id| (name.to_string(), id)))
                .collect::<Vec<_>>();
            (sim.joints(), bodies)
        } else {
            (Vec::new(), Vec::new())
        };
        let section_world_matches = !strength_bodies.is_empty() && world_matches;
        let section_joints: Vec<_> = engine_joints.iter()
            .filter(|joint| section_world_matches && section_targets.contains(joint.name.as_str())
                && matches!(joint.kind, JointKind::Slide | JointKind::Hinge))
            .cloned().collect();
        let joints = if kinematic_check { engine_joints } else { Vec::new() };
        let expanded_candidates = if world_matches {
            Vec::new()
        } else {
            compiled.powered_contact_candidates().map_err(invalid)?
        };
        let mut observe = Some(observe);
        let mut run_one = |snapshot_at: Option<Time>| -> Result<_, Error> {
            let mut runtime = Runtime::load(compiled.clone()).map_err(invalid)?;
            for effort in compiled.efforts.iter().filter(|effort| section_channels.contains(effort.name.as_str())) {
                let name = format!("applied_effort/{}", effort.name);
                let channel = runtime.channel(&effort.name).ok_or_else(|| invalid("нет канала нагрузки"))?;
                let unit = runtime.log().header().commands[channel.0 as usize].unit.clone();
                let signal = runtime.register_truth(&name, &unit).map_err(invalid)?;
                runtime.register_evaluator(&name, runtime.physics_rate(), AppliedEffort {
                    joint: effort.joint.clone(), dof: effort.dof, index: None, signal,
                }).map_err(invalid)?;
            }
            section_reaction::register(&mut runtime, section_joints.clone(), &strength_bodies).map_err(invalid)?;
            let owner = runtime.snapshot();
            let mut receipt = install(&mut runtime, &sampled).map_err(invalid)?;
            runtime.verify_snapshot_owner(&owner).map_err(invalid)?;
            if !runtime.process_run_is_fresh() {
                return Err(invalid(
                    "инсталлятор вернул начатый или воспроизведённый прогон",
                ));
            }
            let installation_confirmed = Some(receipt.build) == requirements.build
                && metrics::installation_matches(&sampled, &receipt.installation, row.grid.hz)
                && same_settings(&sampled.settings, &receipt.settings)
                && settings_match_runtime(&sampled.settings, &runtime);
            let mut missing = Vec::new();
            if !world_matches {
                missing.push(
                    "состав расширенной модели требует проверяемой связи с обязательствами".into(),
                );
                if !expanded_candidates.is_empty() {
                    missing.push(format!("участники в пределах хода расширенной модели: {expanded_candidates:?}"));
                }
            }
            if !installation_confirmed {
                missing
                    .push("не совпадают сборка, отпечатки, зерно или фактические установки".into());
            }
            if timed && sampled.experiment.as_ref().and_then(|e| e.kind).is_some() {
                return Err(invalid(
                    "расписание связанного опыта принадлежит его установщику",
                ));
            }
            let mut advance = |runtime: &mut Runtime,
                               steps: &[Step],
                               operations: &mut Vec<StepObservation>,
                               until: Time|
             -> Result<_, Error> {
                if !timed {
                    return Ok(runtime.run_until(until));
                }
                for step in steps {
                    if step.at_ns > runtime.time().0 {
                        let status = runtime.run_until(Time(step.at_ns));
                        if status != Ok(Status::Running) {
                            return Ok(status);
                        }
                    }
                    if let Target::Effort { name } = &step.target {
                        let id = runtime
                            .channel(name)
                            .ok_or_else(|| invalid(format!("нет входа усилия «{name}»")))?;
                        runtime
                            .act(&Command {
                                targets: vec![(id, step.value)],
                            })
                            .map_err(invalid)?;
                    } else if let Some(observed) = operation(runtime, step).map_err(invalid)? {
                        operations.push(observed);
                    }
                }
                Ok(runtime.run_until(until))
            };
            let horizon = Time(mode.horizon_ns);
            let mut snapshot = None;
            let status = if let Some(at) = snapshot_at {
                let (before, after) = mode
                    .steps
                    .split_at(mode.steps.partition_point(|s| s.at_ns < at.0));
                let prefix = advance(&mut runtime, before, &mut receipt.operations, at)?;
                let checkpoint = if prefix == Ok(Status::Running) && runtime.time() == at {
                    runtime.try_snapshot()
                } else {
                    Err("не достигнут заявленный такт снимка".into())
                };
                let prefix_operations = receipt.operations.clone();
                let status = advance(&mut runtime, after, &mut receipt.operations, horizon)?;
                let audit = checkpoint.and_then(|checkpoint| {
                    let foreign = Runtime::load(compiled.clone())?;
                    if foreign.verify_snapshot_owner(&checkpoint).is_ok() {
                        return Err("принят снимок другого экземпляра".into());
                    }
                    let mut uninterrupted = Vec::new();
                    runtime
                        .log()
                        .write_binary(&mut uninterrupted)
                        .map_err(|e| e.to_string())?;
                    runtime.restore(&checkpoint)?;
                    let mut continued_operations = prefix_operations;
                    let resumed_status =
                        advance(&mut runtime, after, &mut continued_operations, horizon)
                            .map_err(|e| e.to_string())?;
                    let mut resumed = Vec::new();
                    runtime
                        .log()
                        .write_binary(&mut resumed)
                        .map_err(|e| e.to_string())?;
                    if uninterrupted != resumed
                        || status != resumed_status
                        || format!("{:?}", receipt.operations)
                            != format!("{continued_operations:?}")
                    {
                        return Err("журнал или завершение изменились после снимка".into());
                    }
                    Ok(())
                });
                snapshot = Some(SnapshotAudit {
                    at_ns: at.0,
                    matched: audit.is_ok(),
                    reason: audit.err(),
                });
                status
            } else {
                advance(&mut runtime, &mode.steps, &mut receipt.operations, horizon)?
            };
            let log = runtime.log();
            if status != Ok(Status::Running)
                || log.end().time.0 != mode.horizon_ns
                || log.end().steps == 0
            {
                missing.push(format!(
                    "не достигнут горизонт: {status:?} {:?}",
                    runtime.failure()
                ));
            }
            for entry in log.entries() {
                crate::task::finite(entry.value)?;
            }
            for value in receipt
                .installation
                .values
                .values()
                .chain(receipt.operations.iter().map(|s| &s.value))
                .chain(
                    receipt
                        .settings
                        .initial_velocity
                        .values()
                        .flat_map(|v| v.values()),
                )
            {
                crate::task::finite(*value)?;
            }
            for limitation in runtime.simplifications().iter().filter(|s| s.blocks_proof) {
                missing.push(format!("{}: {}", limitation.subject, limitation.text));
            }
            for channel in sampled.settings.channel_delay_ns.keys() {
                if !runtime.channel_delay_observed(channel) {
                    missing.push(format!("нет живой доставки с задержкой «{channel}»"));
                }
            }
            for sample in sampled.values() {
                let Parameter::SensorLatency { sensor } = &sample.parameter else {
                    continue;
                };
                let expected = delay_ns(sample.value).expect("задержка проверена");
                let prefix = format!("{sensor}/");
                let observations = runtime.observe();
                let delivered = observations.samples.iter().any(|observation| {
                    if !observation.valid {
                        return false;
                    }
                    let Some(spec) = log.header().observations.get(observation.signal.0 as usize)
                    else {
                        return false;
                    };
                    if !spec.name.starts_with(&prefix) {
                        return false;
                    }
                    let extra = sampled
                        .settings
                        .channel_delay_ns
                        .get(&format!("observation:{}", spec.name))
                        .copied()
                        .unwrap_or(0);
                    sim_runtime::control::measurement_time(&runtime, observation.signal)
                        .is_some_and(|t| {
                            t.delivered_at.0.checked_sub(t.captured_at.0)
                                == expected.checked_add(extra)
                        })
                });
                if !delivered {
                    missing.push(format!(
                        "не подтверждена задержка захвата и доставки датчика «{sensor}»"
                    ));
                }
            }
            if let Some(control) = sim_runtime::control::run(&runtime)
                && (control.origin() != sim_runtime::control::RunOrigin::Live
                    || control.has_external_processes())
            {
                missing.push(
                    "не подтверждены происхождение или изоляция управляющего процесса".into(),
                );
            }
            let mut faults = Vec::new();
            for fault in &row.faults {
                if let Some(observation) = FaultObservation::from_state(fault, log)? {
                    faults.push(observation);
                }
            }
            let complete_faults = row.faults.len() == faults.len()
                && row
                    .faults
                    .iter()
                    .all(|fault| metrics::matching_fault(fault, &faults, log.end().time.0));
            let summary = if sampled.experiment.as_ref().and_then(|e| e.kind).is_none() {
                Some(metrics::evaluate_scenario(
                    row,
                    &sampled,
                    log,
                    receipt.installation,
                    &receipt.operations,
                    &faults,
                )?)
            } else {
                None
            };
            if let Some(observe) = observe.take() {
                observe(&runtime).map_err(invalid)?;
            }
            Ok((
                log.clone(),
                complete_faults,
                missing,
                summary,
                installation_confirmed,
                snapshot,
            ))
        };
        let (log, faults_confirmed, mut missing, summary, installation_confirmed, _) =
            run_one(None)?;
        let (repeat, repeat_faults, repeat_missing, repeat_summary, repeat_installed, mut snapshot) =
            run_one(snapshot_at)?;
        let mut scenario = summary.as_ref().map_or(Outcome::NotAnalyzed, |r| r.outcome);
        let repeat_scenario = repeat_summary
            .as_ref()
            .map_or(Outcome::NotAnalyzed, |r| r.outcome);
        let journal = log.fingerprint();
        let repeated_journal = repeat.fingerprint();
        if journal != repeated_journal || scenario != repeat_scenario || !repeat_installed {
            missing.push("повтор живого прогона не совпал".into());
            if let Some(audit) = &mut snapshot
                && audit.matched
            {
                audit.matched = false;
                audit.reason =
                    Some("результат отличается от отдельного непрерывного прогона".into());
            }
        }
        if let Some(reason) = snapshot.as_ref().and_then(|audit| audit.reason.as_ref()) {
            missing.push(reason.clone());
        }
        // Нарушение во втором живом повторе сохраняется при рассогласовании с первым.
        if repeat_installed {
            scenario = scenario.max(repeat_scenario);
        }
        missing.extend(repeat_missing);
        missing.sort();
        missing.dedup();
        let sources = std::iter::once(&mode.source)
            .chain(mode.initial.iter().map(|i| &i.source))
            .chain(mode.poses.iter().map(|i| &i.source))
            .chain(mode.steps.iter().map(|i| &i.source))
            .chain(std::iter::once(&row.profile_source))
            .chain(std::iter::once(&row.grid.source))
            .chain(&row.combination_source)
            .chain(row.faults.iter().map(|f| &f.schedule.source))
            .chain(mode.metrics.iter().map(|metric| &metric.source))
            .chain(sampled.values().iter().flat_map(|v| &v.sources))
            .cloned()
            .chain(compiled.bodies.iter().map(|b| provenance(&b.provenance)))
            .collect();
        self.unavailable.remove(key);
        self.runs.insert(
            key.clone(),
            Run {
                sampled,
                build: requirements.build,
                log: TruthLog::capture(&log, journal),
                divergent_repeat: (repeat_installed && journal != repeated_journal)
                    .then(|| TruthLog::capture(&repeat, repeated_journal)),
                sources,
                installation_confirmed,
                missing,
                scenario,
                summary,
                snapshot,
                faults_confirmed: faults_confirmed && repeat_faults,
                planned_faults: row.faults.clone(),
                joints,
                section_world_matches,
                task: plan.task_fingerprint(key),
                process: None,
                interaction: None,
                control: None,
                linked_profile: requirements.linked_profiles.get(key).copied(),
                profiles,
                bench: Vec::new(),
            },
        );
        Ok(())
    }

    fn run_mut(&mut self, key: &RunKey) -> Result<&mut Run, Error> {
        self.runs
            .get_mut(key)
            .filter(|r| r.installation_confirmed)
            .ok_or_else(|| invalid("нет связанной живой строки с подтверждённой установкой"))
    }

    /// Принимает результат только собственного живого оценщика с тем же журналом и областью.
    pub fn record_control(&mut self, key: &RunKey, result: &RunResult) -> Result<(), Error> {
        let run = self.run_mut(key)?;
        let report = result.report();
        let scheduled = key.faults.first();
        if run.control.is_some()
            || run.task != Some(result.task)
            || !run.accepts(
                ReferenceKind::ControlTask,
                result.identity.task_and_criteria,
            )
            || report.journal != Some(run.log.fingerprint)
            || report.case.path != key.path
            || report.case.mode != key.mode
            || report.case.fault.as_ref() != scheduled
            || key.faults.len() > 1
            || result.identity.seed != run.sampled.sample.seeds.run
            || run.sampled.experiment.as_ref().and_then(|e| e.kind)
                != Some(crate::campaign::ReferenceKind::ControlTask)
        {
            return Err(invalid("другой журнал, контур, режим или отказ"));
        }
        if let [fault] = run.planned_faults.as_slice()
            && report.outcome == Outcome::Pass
            && report.requested_ns == Some(fault.schedule.at_ns)
            && report.applied_ns.is_some_and(|at| {
                (fault.schedule.at_ns..=fault.schedule.at_ns + fault.schedule.max_lag_ns)
                    .contains(&at)
                    && at > fault.window.start_ns
                    && at < fault.window.end_ns
            })
            && report
                .before_loop
                .as_ref()
                .is_some_and(|before| before.feedback_ns < report.applied_ns.unwrap())
            && report
                .reached_ns
                .is_some_and(|at| at >= fault.window.end_ns)
        {
            run.faults_confirmed = true;
        }
        run.control = Some((
            result.identity.task_and_criteria,
            result.live_board,
            report.clone(),
        ));
        Ok(())
    }

    /// Сохраняет метрики, события и опасности только от живой серии с совпадающим базовым журналом.
    pub fn record_process(
        &mut self,
        key: &RunKey,
        evidence: &ConvergenceReport,
    ) -> Result<(), Error> {
        let run = self.run_mut(key)?;
        if run.process.is_some()
            || !run.accepts(ReferenceKind::ProcessTask, evidence.profile())
            || !evidence.is_live()
            || !evidence
                .trials
                .first()
                .is_some_and(|base| base.journal == Some(run.log.fingerprint))
        {
            return Err(invalid("другой или изменённый отчёт о физических сетках"));
        }
        run.process = Some((
            evidence.profile(),
            evidence.outcome,
            evidence.campaign_findings(key),
        ));
        Ok(())
    }

    /// Переносит раздельные выводы по конструкции, тканям и достижимым путям энергии.
    pub fn record_interaction(
        &mut self,
        key: &RunKey,
        evidence: &InteractionEvidence,
    ) -> Result<(), Error> {
        let build = evidence.campaign_build();
        let run = self.run_mut(key)?;
        if run.interaction.is_some()
            || !run.accepts(ReferenceKind::InteractionTask, evidence.campaign_profile())
            || Some(build) != run.build
            || !evidence
                .grids()
                .iter()
                .any(|g| g.journal == Fingerprint::from_fnv1a(run.log.fingerprint))
        {
            return Err(invalid("другое взаимодействие, сборка или журнал"));
        }
        run.interaction = Some((
            evidence.campaign_profile(),
            evidence.outcome(),
            evidence.campaign_findings(key),
        ));
        Ok(())
    }

    /// Принимает стендовый ряд для этой сборки, полного ключа и канала.
    /// Источник должен идентифицировать замер, его калибровку и область.
    pub fn record_measurement(
        &mut self,
        requirements: &RequirementSet,
        key: &RunKey,
        build: Fingerprint,
        signal: crate::Signal,
        source: Source,
        uncertainty: Option<f64>,
        samples: &[MeasurementSample],
    ) -> Result<(), Error> {
        self.matches(requirements)?;
        if Some(build) != requirements.build || source.kind != SourceKind::Measured {
            return Err(invalid(
                "замер не принадлежит сборке или не имеет измеренного источника",
            ));
        }
        source.validate()?;
        signal.validate()?;
        let run = self.run_mut(key)?;
        if run.bench.iter().any(|m| m.signal == signal) {
            return Err(invalid("повторный стендовый канал"));
        }
        let points: Vec<_> = samples
            .iter()
            .map(|sample| Point {
                at_ns: sample.at_ns,
                value: sample.value,
                valid: true,
            })
            .collect();
        Series {
            signal: signal.clone(),
            points: points.clone(),
        }
        .validate()?;
        if uncertainty.is_some_and(|u| {
            !u.is_finite() || u < 0.0 || points.iter().any(|p| !(p.value.abs() + u).is_finite())
        }) {
            return Err(invalid("некорректная граница погрешности стендового канала"));
        }
        let journal = Fingerprint::of(
            format!("{build:?}{key:?}{signal:?}{source:?}{uncertainty:?}{points:?}").as_bytes(),
        );
        run.bench.push(Measurement {
            signal,
            source,
            journal,
            samples: points,
            uncertainty,
        });
        Ok(())
    }

    pub(super) fn diagnostics(&self, set: &RequirementSet) -> Vec<Finding> {
        self.runs
            .values()
            .filter(|run| run.installation_confirmed)
            .flat_map(|run| {
                run.details().cloned().map(|mut finding| {
                    run.finish(set, &mut finding);
                    finding
                })
            })
            .collect()
    }

    pub(super) fn findings(
        &self,
        set: &RequirementSet,
        requirement: &Requirement,
        output: &mut Vec<Finding>,
        model_strength_open: &mut Vec<Finding>,
    ) -> Result<(), Error> {
        let key = &requirement.key;
        let Some(run) = key
            .run
            .as_ref()
            .and_then(|k| self.runs.get(k))
            .filter(|r| r.installation_confirmed)
        else {
            let mut finding = Finding::open(
                key,
                "нет подходящего живого прогона с подтверждённой установкой",
            );
            if let Some(run) = key.run.as_ref().and_then(|k| self.runs.get(k)) {
                finding.coverage_gaps.extend(run.missing.iter().cloned());
            }
            if let Some(reasons) = key.run.as_ref().and_then(|k| self.unavailable.get(k)) {
                finding.coverage_gaps.extend(reasons.iter().cloned());
            }
            output.push(finding);
            return Ok(());
        };
        if requirement.checks.is_empty() {
            if key.class == HazardClass::Execution {
                output.push(run.execution(set, key));
            } else {
                output.extend(
                    run.details()
                        .filter(|f| {
                            f.key.class == key.class
                                && f.key.object == key.object
                                && f.outcome == Outcome::NotAnalyzed
                        })
                        .cloned(),
                );
                output.push(Finding::open(
                    key,
                    "нет применимого метода, предела или модели",
                ));
            }
            return Ok(());
        }
        if !matches!(
            key.class, HazardClass::Measurement | HazardClass::TissueLoad
                | HazardClass::Ingress | HazardClass::StaticStrength
        )
            && requirement.checks.iter().all(|check| check.verification.measurement_limit().is_some())
        {
            output.push(Finding::open(key, "замер не заменяет профильную проверку опасности"));
        }
        for check in &requirement.checks {
            for finding in run.check(set, requirement, check)? {
                if key.class == HazardClass::StaticStrength
                    && check.verification.measurement_limit().is_none()
                    && finding.outcome == Outcome::NotAnalyzed
                {
                    model_strength_open.push(finding);
                } else {
                    output.push(finding);
                }
            }
        }
        Ok(())
    }

    pub(super) fn has_measured_pass(
        &self, set: &RequirementSet, requirement: &Requirement,
    ) -> Result<bool, Error> {
        let Some(run) = requirement.key.run.as_ref().and_then(|key| self.runs.get(key))
            .filter(|run| run.installation_confirmed) else { return Ok(false) };
        for check in &requirement.checks {
            if check.verification.measurement_limit().is_some()
                && run.check(set, requirement, check)?.iter().any(|f| f.outcome == Outcome::Pass)
            {
                return Ok(true);
            }
        }
        Ok(false)
    }
}

fn same_settings(a: &ModelSettings, b: &ModelSettings) -> bool {
    a.channel_delay_ns == b.channel_delay_ns
        && a.initial_velocity.len() == b.initial_velocity.len()
        && a.initial_velocity.iter().all(|(key, dofs)| {
            b.initial_velocity.get(key).is_some_and(|other| {
                dofs.len() == other.len()
                    && dofs.iter().all(|(dof, value)| {
                        other
                            .get(dof)
                            .is_some_and(|v| v.to_bits() == value.to_bits())
                    })
            })
        })
}

fn settings_match_runtime(settings: &ModelSettings, runtime: &Runtime) -> bool {
    settings.initial_velocity.iter().all(|(joint, dofs)| {
        dofs.iter().all(|(&dof, &expected)| {
            runtime
                .joint_velocity(joint, dof)
                .is_some_and(|actual| actual.to_bits() == expected.to_bits())
        })
    }) && settings
        .channel_delay_ns
        .iter()
        .all(|(channel, &expected)| runtime.channel_delay_ns(channel) == Some(expected))
}

impl Run {
    fn expected_kind(&self) -> Option<ReferenceKind> {
        self.sampled.experiment.as_ref().and_then(|e| e.kind)
    }

    fn accepts(&self, kind: ReferenceKind, fingerprint: Fingerprint) -> bool {
        if self.expected_kind() == Some(kind) {
            self.linked_profile == Some(fingerprint)
        } else {
            self.expected_kind().is_none()
                && self
                    .profiles
                    .get(&kind)
                    .is_some_and(|set| set.contains(&fingerprint))
        }
    }

    fn details(&self) -> impl Iterator<Item = &Finding> {
        self.process
            .iter()
            .chain(&self.interaction)
            .flat_map(|(_, _, findings)| findings)
    }

    fn journal(&self) -> Fingerprint {
        Fingerprint::from_fnv1a(self.log.fingerprint)
    }

    fn execution(&self, set: &RequirementSet, key: &RequirementKey) -> Finding {
        let mut finding = Finding::open(key, "покрытие выполнения режима");
        finding.journal = Some(self.journal());
        finding.outcome = self.scenario;
        if let Some((_, _, control)) = &self.control {
            finding.outcome = control.outcome;
            finding
                .coverage_gaps
                .extend(control.missing.iter().cloned());
            if !control.violations.is_empty() {
                finding.reason = control.violations.join(" / ");
            }
        } else if let Some(kind) = self.expected_kind() {
            let result = match kind {
                ReferenceKind::ProcessTask => self.process.as_ref(),
                ReferenceKind::InteractionTask => self.interaction.as_ref(),
                _ => None,
            };
            finding.outcome = result.map_or(Outcome::NotAnalyzed, |(_, outcome, _)| *outcome);
            finding.coverage_gaps.extend(
                self.details()
                    .filter(|f| f.key.class == HazardClass::Execution)
                    .flat_map(|f| &f.coverage_gaps)
                    .cloned(),
            );
        }
        self.finish(set, &mut finding);
        finding
    }

    fn finish(&self, set: &RequirementSet, finding: &mut Finding) {
        let (proved, sources) = trace(&self.sources, &set.derivations);
        finding.measurement_sources.extend(sources);
        finding.measurement_sources.sort();
        finding.measurement_sources.dedup();
        finding.coverage_gaps.extend(self.missing.iter().cloned());
        if !self.faults_confirmed {
            finding
                .coverage_gaps
                .push("нет подтверждения всех отказов в окне до и после".into());
        }
        if !proved {
            finding
                .coverage_gaps
                .push("есть оценка или неполная цепочка происхождения входов".into());
        }
        if finding.outcome == Outcome::Pass && !finding.coverage_gaps.is_empty() {
            finding.outcome = Outcome::NotAnalyzed;
        }
        if finding.outcome != Outcome::NotAnalyzed {
            finding.missing_data.clear();
        }
        finding.confirmed_hazard &= proved
            && self.faults_confirmed
            && self.installation_confirmed
            && self.log.end.steps > 0
            && self
                .sampled
                .mode
                .as_ref()
                .is_some_and(|m| self.log.end.time.0 == m.horizon_ns);
    }

    fn check(
        &self,
        set: &RequirementSet,
        requirement: &Requirement,
        check: &Check,
    ) -> Result<Vec<Finding>, Error> {
        let key = &requirement.key;
        let mut finding = Finding::open(key, "неполное покрытие проверки");
        finding.model_scope = Some(check.scope.clone());
        finding.journal = Some(self.journal());
        let mut sources = check.inputs.clone();
        sources.extend_from_slice(&requirement.declared);
        sources.push(check.applicability.clone());
        if let Some(reason) = key.class.emc_measurement().or_else(|| key.class.pcb_measurement()) {
            // Универсальный ряд не связывает измерение с адресованной постановкой.
            finding.reason = reason.into();
            finding.missing_data = vec![reason.into()];
            return Ok(self.checked_findings(set, check, key, &sources, vec![finding]));
        }
        let details = match &check.verification {
            Verification::Section {
                geometry: declared_geometry,
                axis: _,
                moment_nm,
                live_load,
                allowable_pa,
                load,
                limit,
            } => {
                finding.limit = Some(LimitValue::upper(*allowable_pa));
                finding.unit = Some("Pa".into());
                sources.push(load.clone());
                finding.limit_sources = trace(std::slice::from_ref(limit), &set.derivations).1;
                if self.section_world_matches
                    && let Some(live) = live_load
                    && let Some(SectionWindow { link, samples, complete, complex_path }) =
                        self.section_samples(&key.object, check.window, live)?
                {
                    if *declared_geometry >= link.collision.len() {
                        finding.coverage_gaps.push("объявленный индекс сечения вне геометрии звена".into());
                    }
                    let mass_volumes = match &link.mass {
                        Mass::Volumes { volumes, .. } => volumes.as_slice(),
                        _ => &[],
                    };
                    let geometries = link.collision.iter().chain(mass_volumes);
                    // Внешние точки нагрузки выводятся из всех креплений, якоря и поз
                    // объёмов. Положение центра масс может выходить за сечение.
                    let points: Vec<_> = std::iter::once(link.joint.anchor_m)
                        .chain(link.mounts.iter()
                            .filter(|mount| mount.kind == sim_world::MountKind::Mechanical)
                            .map(|mount| mount.pose.position_m))
                        .chain(geometries.clone().map(|geometry| geometry.pose.position_m))
                        .chain(match &link.mass {
                            Mass::Explicit { com_m, .. } => Some(*com_m),
                            _ => None,
                        })
                        .collect();
                    let direction = link.joint.axis.ok_or_else(|| invalid("нет оси нагрузки сечения"))?;
                    let sliding = link.joint.kind == JointType::Slide;
                    let declared_moment = moment_nm[0].abs().max(moment_nm[1].abs());
                    let mut worst = None::<f64>;
                    let mut worst_live: Option<(f64, SectionSample, SectionWitness)> = None;
                    let mut maximum_span_stress = 0.0_f64;
                    let mut maximum_arm = 0.0_f64;
                    let mut maximum_fixture_moment = 0.0_f64;
                    let mut maximum_reaction_moment = 0.0_f64;
                    let mut maximum_span_moment = 0.0_f64;
                    let distribution = section_span::distribution(link, sliding);
                    if !complete {
                        finding.coverage_gaps.push("неполное окно записи реакции корневого сечения".into());
                    }
                    if complex_path {
                        finding.coverage_gaps.push("нужна проверка пути нагрузки от реакции корня до сечений звена".into());
                    }
                    if distribution.is_none() {
                        finding.coverage_gaps.push("не доказано распределение массы по отрезанным частям звена".into());
                    }
                    if !complex_path && samples.iter().any(|sample| sample.motion.is_none()) {
                        finding.coverage_gaps.push("нет полного баланса внешних и инерционных нагрузок вдоль звена".into());
                    }
                    for geometry in geometries {
                        use SectionAxis::{X, Y, Z};
                        let local_axis = geometry.local_load_direction(direction).map_err(invalid)?;
                        let basis = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
                            .map(|axis| geometry.local_load_direction(axis));
                        let [bx, by, bz] = basis;
                        let [bx, by, bz] = [bx.map_err(invalid)?, by.map_err(invalid)?, bz.map_err(invalid)?];
                        let local = |[x, y, z]: [f64; 3]| {
                            std::array::from_fn(|i| bx[i] * x + by[i] * y + bz[i] * z)
                        };
                        let center = local(sub(geometry.pose.position_m, link.joint.anchor_m));
                        let outside = |distance: f64, half: f64| distance.abs() - half > half * f64::EPSILON;
                        let outside_root = match geometry.shape {
                            Shape::Box { size_m } => (0..3).any(|i| outside(center[i], size_m[i] / 2.0)),
                            Shape::Cylinder { radius_m, length_m } =>
                                outside(center[0].hypot(center[1]), radius_m) || outside(center[2], length_m / 2.0),
                            _ => false,
                        };
                        if outside_root {
                            finding.coverage_gaps.push("якорь сочленения вне объёма сечения".into());
                        }
                        // Круговое постоянное сечение определено только вдоль оси цилиндра.
                        let axes: &[_] = match geometry.shape {
                            Shape::Box { .. } => &[X, Y, Z],
                            _ => &[Z],
                        };
                        for &axis in axes {
                            let (section, length_m) = match strength_section(&geometry.shape, axis).map_err(invalid)? {
                                StrengthSection::Applicable { section, length_m } => (section, length_m),
                                StrengthSection::NotAnalyzed { reason } => {
                                    finding.missing_data.push(reason.clone());
                                    finding.coverage_gaps.push(reason);
                                    continue;
                                }
                            };
                            let (geometric, eccentricity) = if sliding {
                                let arm = geometry.section_load_arm(axis, &points).map_err(invalid)?
                                    .ok_or_else(|| invalid("нет плеча для применимого сечения"))?;
                                let radial = geometry.section_radial_arm(axis, &points).map_err(invalid)?
                                    .ok_or_else(|| invalid("нет эксцентриситета для применимого сечения"))?;
                                (arm, radial)
                            } else {
                                // Изолированная вращательная ось передаёт момент без редукции.
                                (1.0, 0.0)
                            };
                            let modulus = section.section_modulus();
                            if !modulus.is_finite() || modulus <= 0.0 {
                                return Err(invalid("момент сопротивления сечения вне численного диапазона"));
                            }
                            let i = match axis {
                                X => 0,
                                Y => 1,
                                Z => 2,
                            };
                            let j = (i + 1) % 3;
                            let k = (i + 2) % 3;
                            let dirs = local_axis.map(f64::abs);
                            let (axial, u, v) = (dirs[i], dirs[j], dirs[k]);
                            let transverse = u.hypot(v);
                            // У круга W не зависит от плоскости. Для коробки
                            // суперпозиция двух изгибов ограничена их суммой с W_min.
                            let rectangular = matches!(geometry.shape, Shape::Box { .. });
                            let (bending, eccentric_bending) = if rectangular {
                                // Сумма модулей двух компонент не больше √2 их нормы.
                                (u + v, std::f64::consts::SQRT_2)
                            } else {
                                (transverse, 1.0)
                            };
                            let stress_at = |moment: f64, effort: f64| {
                                // σ_VM² = σ² + 3τ² для одного нормального
                                // напряжения и результирующего сдвига.
                                if sliding {
                                    // Направление силы известно. Линия её передачи на
                                    // неизвестное сечение может отличаться от оси каретки,
                                    // поэтому сохраняем оценку изгиба с полным плечом.
                                    let longitudinal = effort * axial;
                                    let lateral = effort * transverse;
                                    let normal = longitudinal / section.area()
                                        + (moment * bending + longitudinal * eccentricity * eccentric_bending)
                                            / modulus;
                                    let shear = section.shear_stress(lateral)
                                        + section.torsion_stress(lateral * eccentricity);
                                    (moment / modulus).max(normal.hypot(3.0_f64.sqrt() * shear))
                                } else {
                                    let normal = (moment / modulus) * bending;
                                    let shear = section.torsion_stress(moment * axial);
                                    normal.hypot(3.0_f64.sqrt() * shear)
                                }
                            };
                            maximum_arm = maximum_arm.max(geometric);
                            // Перенос к центроиду сечения в плоскости якоря.
                            let mut offset = center;
                            offset[i] = 0.0;
                            let (modulus_j, modulus_k) = match geometry.shape {
                                // Момент каждого направления делится на W именно для
                                // этой плоскости. Заявленная нагрузка и реакция корня
                                // по-прежнему проверяются с прежним W_min.
                                Shape::Box { size_m: d } =>
                                    (d[j] * d[k].powi(2) / 6.0, d[k] * d[j].powi(2) / 6.0),
                                _ => (modulus, modulus),
                            };
                            if ![modulus_j, modulus_k].into_iter().all(|w| w.is_finite() && w > 0.0) {
                                return Err(invalid("момент сопротивления направленного изгиба вне численного диапазона"));
                            }
                            let adjacent = if !complex_path
                                && let Some(Distribution::Stepped { material, volumes }) = distribution
                            {
                                section_span::adjoining_prisms(geometry, axis, i, material, volumes, local).map_err(invalid)?
                            } else {
                                None
                            };
                            let shear_per_n = section.shear_stress(1.0);
                            let torsion_per_nm = section.torsion_stress(1.0);
                            let span_stress = |wrench: Wrench| {
                                let f = wrench.force.map(f64::abs);
                                let m = wrench.moment.map(f64::abs);
                                let bend = if rectangular {
                                    m[j] / modulus_j + m[k] / modulus_k
                                } else {
                                    m[j].hypot(m[k]) / modulus
                                };
                                let normal = f[i] / section.area() + bend;
                                let shear = shear_per_n * f[j].hypot(f[k]) + torsion_per_nm * m[i];
                                normal.hypot(3.0_f64.sqrt() * shear)
                            };
                            for &sample in &samples {
                                let force = local(sample.reaction.force);
                                let moment = local(sample.reaction.moment);
                                let shift = cross(force, offset);
                                let [mx, my, mz]: [f64; 3] = std::array::from_fn(|axis| moment[axis] + shift[axis]);
                                let moment = [mx, my, mz].map(f64::abs);
                                let bend = if rectangular { moment[j] + moment[k] } else { moment[j].hypot(moment[k]) };
                                // Огибающая нормального напряжения при любом знаке
                                // продольной силы и изгиба. Сдвиг и кручение
                                // складываются по модулю перед критерием Мизеса.
                                let normal = force[i].abs() / section.area() + bend / modulus;
                                let shear = section.shear_stress(force[j].hypot(force[k]))
                                    + section.torsion_stress(moment[i]);
                                let reaction_stress = normal.hypot(3.0_f64.sqrt() * shear);
                                crate::task::finite(reaction_stress)?;
                                let live_moment = sample.effort * live.moment_per_effort.max(geometric);
                                crate::task::finite(live_moment)?;
                                // Огибающая оснастки остаётся отдельным ограничением.
                                // Сложный путь оцениваем по реакции корня.
                                let (stress, witness) = if complex_path {
                                    (reaction_stress, SectionWitness::Root)
                                } else {
                                    let fixture_stress = stress_at(live_moment, sample.effort);
                                    if reaction_stress > fixture_stress {
                                        (reaction_stress, SectionWitness::Root)
                                    } else {
                                        (fixture_stress, SectionWitness::Fixture)
                                    }
                                };
                                let declared = stress_at(declared_moment, if complex_path { 0.0 } else { sample.effort });
                                crate::task::finite(stress)?;
                                crate::task::finite(declared)?;
                                if worst_live.is_none_or(|(old, _, _)| stress > old) {
                                    worst_live = Some((stress, sample, witness));
                                }
                                let resultant = moment[0].hypot(moment[1]).hypot(moment[2]);
                                crate::task::finite(resultant)?;
                                maximum_reaction_moment = maximum_reaction_moment.max(resultant);
                                if !complex_path {
                                    maximum_fixture_moment = maximum_fixture_moment.max(live_moment);
                                }
                                let mut observed = stress.max(declared);
                                if !complex_path && let (Some(distribution), Some(motion)) = (distribution, sample.motion) {
                                    let motion = Motion { gravity: local(motion.gravity), ..motion };
                                    let mut bound_at = |wrench: Wrench| -> Result<(), Error> {
                                        for value in wrench.force.into_iter().chain(wrench.moment) {
                                            crate::task::finite(value)?;
                                        }
                                        let stress = span_stress(wrench);
                                        let moment = section_span::norm(wrench.moment);
                                        crate::task::finite(stress)?;
                                        crate::task::finite(moment)?;
                                        maximum_span_stress = maximum_span_stress.max(stress);
                                        maximum_span_moment = maximum_span_moment.max(moment);
                                        Ok(())
                                    };
                                    match distribution {
                                        Distribution::Uniform | Distribution::Stepped { .. } => {
                                            let span = match distribution {
                                                Distribution::Uniform => Some((motion.mass, &[][..])),
                                                _ => adjacent.as_ref().map(|(mass, pieces)| (*mass, pieces.as_slice())),
                                            };
                                            if let Some((mass, adjacent)) = span {
                                                for controls in section_span::overhangs(
                                                    i, length_m, section_span::scale(center, -1.0),
                                                    section_span::variances(geometry), local_axis,
                                                    Motion { mass, ..motion }, sliding, adjacent,
                                                ) {
                                                    // Норма напряжений выпукла по силе и моменту.
                                                    // Максимум контрольных точек ограничивает весь отрезок.
                                                    for control in controls {
                                                        bound_at(control)?;
                                                    }
                                                    for at_cut in [controls[0], Wrench::midpoint(controls), controls[3]] {
                                                        let stress = span_stress(at_cut);
                                                        crate::task::finite(stress)?;
                                                        if worst_live.is_none_or(|(old, _, _)| stress > old) {
                                                            worst_live = Some((stress, sample, SectionWitness::Interior));
                                                        }
                                                        observed = observed.max(stress);
                                                    }
                                                }
                                            }
                                        }
                                        Distribution::Translation { com, inertia_trace } => {
                                            crate::task::finite(inertia_trace)?;
                                            let relative_com = local(sub(com, geometry.pose.position_m));
                                            let acceleration = sub(section_span::scale(local_axis, motion.acceleration), motion.gravity);
                                            bound_at(section_span::translation_bound(
                                                i, length_m, relative_com, inertia_trace, acceleration, motion.mass,
                                            ).map_err(invalid)?)?;
                                        }
                                    }
                                }
                                worst = Some(worst.map_or(observed, |previous| previous.max(observed)));
                            }
                        }
                    }
                    if let (Some(stress), Some((live_stress, sample, witness))) = (worst, worst_live) {
                        let measured = [stress, stress.max(maximum_span_stress)];
                        finding.measurement = Some(measured);
                        finding.outcome = if stress > allowable_pa[1] {
                            Outcome::Fail
                        } else {
                            Outcome::NotAnalyzed
                        };
                        finding.reason = match witness {
                            SectionWitness::Root => "расчёт напряжения по полной реакции корневого сечения",
                            SectionWitness::Interior => "расчёт напряжения по балансу отсечённой части звена",
                            SectionWitness::Fixture if sliding =>
                                "расчёт напряжения изгиба по модулю сечения и проверка продольной силы и сдвига",
                            SectionWitness::Fixture => "расчёт напряжения от изгиба и кручения по оси сочленения",
                        }.into();
                        if measured[1] > measured[0] {
                            finding.reason.push_str(". Верхняя граница охватывает сечения вдоль звена");
                            if stress <= allowable_pa[1] && measured[1] > allowable_pa[0] {
                                finding.coverage_gaps.push("интервал напряжения вдоль звена пересекает допустимую границу".into());
                            }
                        }
                        if stress > live_stress {
                            finding.reason.push_str(". Заявленная огибающая выше оценки живой нагрузки");
                        } else {
                            finding.event_ns = Some(sample.at);
                        }
                        finding.journal = Some(sample.journal);
                        finding.confirmed_hazard = finding.outcome == Outcome::Fail
                            && trace(std::slice::from_ref(limit), &set.derivations).0;
                        if live.moment_per_effort < maximum_arm {
                            finding.coverage_gaps.push(format!(
                                "заявленная передача нагрузки меньше геометрической границы {maximum_arm}"
                            ));
                        }
                        // Граница вычисляется из трёх компонент после поворотов.
                        // Округление на масштабе момента не образует физический перебег.
                        if maximum_fixture_moment > declared_moment
                            || maximum_reaction_moment - declared_moment > f64::EPSILON * maximum_reaction_moment
                        {
                            finding.coverage_gaps.push("максимум живого момента превышает заявленную нагрузку".into());
                        }
                        if maximum_span_moment - declared_moment > f64::EPSILON * maximum_span_moment {
                            finding.coverage_gaps.push("верхняя граница момента вдоль звена выше заявленной нагрузки".into());
                        }
                        // Огибающая отдельных сечений выявляет превышение, но не
                        // восстанавливает физический стык нескольких объёмов.
                        if link.collision.len() > 1 || mass_volumes.len() > 1 {
                            finding.coverage_gaps.push("не доказана передача нагрузки между объёмами звена".into());
                        }
                    } else {
                        finding.missing_data.push("у звена нет применимого сечения".into());
                    }
                } else {
                    finding.missing_data.push("нет полной записи применимой нагрузки сечения".into());
                }
                self.section_rotation(&mut finding, check.window, *allowable_pa,
                    trace(std::slice::from_ref(limit), &set.derivations).0)?;
                if !trace(std::slice::from_ref(limit), &set.derivations).0 {
                    sources.push(limit.clone());
                }
                vec![finding]
            }
            Verification::Truth(limit) => {
                let physical = self.kinematic_limit(key, limit);
                let limit = physical.as_ref().unwrap_or(limit);
                let series = self.log.series(limit.signal())?;
                compare_truth(&mut finding, limit, check.window, &series, 0.0, set)?;
                let unbound = key.class != HazardClass::TaskCriterion && physical.is_none();
                if unbound {
                    // Произвольная метка ряда не подтверждает опасность
                    // назначенного объекта, даже если предел соблюдён.
                    if finding.outcome == Outcome::Pass {
                        finding.outcome = Outcome::NotAnalyzed;
                    }
                    finding.confirmed_hazard = false;
                    finding.coverage_gaps.push("нет выведенной из мира привязки ряда к опасности".into());
                }
                if key.class != HazardClass::TaskCriterion
                    && !trace(std::slice::from_ref(limit_source(limit)), &set.derivations).0
                {
                    sources.push(limit_source(limit).clone());
                }
                if let Some(log) = &self.divergent_repeat {
                    let mut repeated = Finding::open(key, "критерий повторного прогона");
                    let series = log.series(limit.signal())?;
                    compare_truth(&mut repeated, limit, check.window, &series, 0.0, set)?;
                    if repeated.outcome == Outcome::Fail {
                        repeated.confirmed_hazard &= !unbound;
                        repeated.journal = Some(Fingerprint::from_fnv1a(log.fingerprint));
                        vec![finding, repeated]
                    } else {
                        vec![finding]
                    }
                } else {
                    vec![finding]
                }
            }
            Verification::Measurement(limit) | Verification::TissueMeasurement { limit, .. }
                | Verification::EnergyMeasurement { limit, .. } => {
                if requires_fault(key.class)
                    && !self.planned_faults.iter().any(|planned| {
                        planned.schedule.fault.node == key.object
                            && fault_class(planned.schedule.fault.kind) == key.class
                    })
                {
                    finding.missing_data.push("замер не связан с обязательной инъекцией отказа".into());
                    return Ok(self.checked_findings(set, check, key, &sources, vec![finding]));
                }
                if key.class == HazardClass::TissueLoad {
                    let qualified = if let Verification::TissueMeasurement { duration, .. } = &check.verification {
                        sources.push(duration.source.clone());
                        let signal = duration.contact.as_ref().unwrap_or_else(|| limit.signal());
                        let boundary = self.bench.iter().find(|m| &m.signal == signal);
                        if let Some(bench) = boundary {
                            sources.push(bench.source.clone());
                        }
                        // При подтверждённом размыкании на обеих границах длительность
                        // контакта лежит между нулём и длиной окна, включая пробелы ряда.
                        duration.ns[0] == 0 && duration.ns[1] >= check.window.end_ns - check.window.start_ns
                            && trace(std::slice::from_ref(&duration.source), &set.derivations).0
                            && boundary.is_some_and(|bench| bench.uncertainty == Some(0.0)
                                && [check.window.start_ns, check.window.end_ns].iter().all(|at|
                                    bench.samples.iter().any(|point| point.at_ns == *at
                                        && point.valid && point.value == 0.0)))
                    } else {
                        false
                    };
                    if !qualified {
                        finding.missing_data.push("нет подтверждённого тканевого предела для длительности контакта в окне".into());
                        return Ok(self.checked_findings(set, check, key, &sources, vec![finding]));
                    }
                }
                if let Some(bench) = self.bench.iter().find(|m| &m.signal == limit.signal()) {
                    if let Some(uncertainty) = bench.uncertainty {
                        compare_truth(
                            &mut finding,
                            limit,
                            check.window,
                            &Series {
                                signal: bench.signal.clone(),
                                points: bench.samples.clone(),
                            },
                            uncertainty,
                            set,
                        )?;
                    } else {
                        finding.reason = "не задана граница погрешности стендового канала".into();
                        finding.coverage_gaps.push(finding.reason.clone());
                    }
                    finding.journal = Some(bench.journal);
                    sources.push(bench.source.clone());
                    sources.push(limit_source(limit).clone());
                }
                vec![finding]
            }
            Verification::Process { profile, criterion } => {
                let observations: Vec<_> = self
                    .process
                    .iter()
                    .filter(|(actual, _, _)| actual == profile)
                    .flat_map(|(_, _, findings)| findings)
                    .filter(|f| f.key.window_ns == key.window_ns)
                    .collect();
                let modeled = observations.iter().any(|f| {
                    f.key.class == key.class
                        && f.key.object == key.object
                        && f.outcome == Outcome::Pass
                });
                let mut results = Vec::new();
                for source in observations
                    .iter()
                    .filter(|f| f.key.class == HazardClass::TaskCriterion && f.code == *criterion)
                {
                    let mut metric = (*source).clone();
                    metric.group = key.class.spec().group;
                    let observed_failure = observations.iter().any(|f| {
                        f.key.class == key.class
                            && f.key.object == key.object
                            && f.outcome == Outcome::Fail
                            && f.event_ns.is_some()
                    });
                    metric.confirmed_hazard = metric.outcome == Outcome::Fail
                        && metric.measurement.is_some()
                        && metric.limit.is_some()
                        && (key.class.spec().method != super::Method::Process || observed_failure);
                    if !modeled
                        && key.class.spec().method == super::Method::Process
                        && metric.outcome == Outcome::Pass
                    {
                        metric
                            .coverage_gaps
                            .push("нет типизированного покрытия класса моделью".into());
                    }
                    results.push(metric);
                }
                if results.is_empty() {
                    vec![finding]
                } else {
                    results
                }
            }
            Verification::Interaction { profile, quantity } => {
                let matches: Vec<_> = self
                    .interaction
                    .iter()
                    .filter(|(actual, _, _)| actual == profile)
                    .flat_map(|(_, _, findings)| findings)
                    .filter(|f| {
                        f.code == *quantity
                            && f.key.class == key.class
                            && f.key.object == key.object
                            && f.key.window_ns == key.window_ns
                    })
                    .cloned()
                    .collect();
                if matches.is_empty() {
                    vec![finding]
                } else {
                    matches
                }
            }
            Verification::Control {
                criteria,
                check: kind,
            } => {
                if let Some((actual, board, report)) = &self.control
                    && actual == criteria
                    && (!requires_fault(key.class)
                        || report.case.fault.as_ref().is_some_and(|fault| {
                            fault.node == key.object && fault_class(fault.kind) == key.class
                        }))
                {
                    finding.coverage_gaps.extend(report.missing.iter().cloned());
                    match kind {
                        ControlCriterion::ProgramIdentity if *board && report.was_executed() => {
                            finding.reason = "образ и вывод подтверждены в живом контуре".into();
                            finding.outcome = if report.outcome == Outcome::Pass {
                                Outcome::Pass
                            } else {
                                Outcome::NotAnalyzed
                            };
                        }
                        ControlCriterion::Limit(signal) => {
                            let series = self.log.series(signal)?;
                            let mut results = Vec::new();
                            for metric in report
                                .metrics
                                .iter()
                                .filter(|metric| &metric.signal == signal)
                            {
                                let mut item = finding.clone();
                                compare_truth(
                                    &mut item,
                                    &metric.limit,
                                    check.window,
                                    &series,
                                    0.0,
                                    set,
                                )?;
                                // Успех требует подтверждённой причинной цепочки.
                                if item.outcome == Outcome::Pass && report.outcome != Outcome::Pass
                                {
                                    item.outcome = Outcome::NotAnalyzed;
                                }
                                results.push(item);
                            }
                            if !results.is_empty() {
                                finding = results.remove(0);
                            }
                            if !results.is_empty() {
                                // Оценка каждого предела остаётся видна, включая противоречащие.
                                results.insert(0, finding);
                                return Ok(
                                    self.checked_findings(set, check, key, &sources, results)
                                );
                            }
                        }
                        ControlCriterion::Reaction(signal) => {
                            if let Some(reaction) = report
                                .reaction
                                .as_ref()
                                .filter(|r| &r.criterion.signal == signal)
                                && let Some(event) =
                                    reaction.applied_ns.checked_add(reaction.reaction_ns)
                                && (check.window.start_ns..=check.window.end_ns)
                                    .contains(&reaction.applied_ns)
                                && event <= check.window.end_ns
                            {
                                let (proved, names) = trace(
                                    std::slice::from_ref(&reaction.criterion.source),
                                    &set.derivations,
                                );
                                finding.measurement = Some([reaction.reaction_ns as f64; 2]);
                                finding.limit = Some(LimitValue::upper(
                                    [reaction.criterion.within_ns as f64; 2],
                                ));
                                finding.limit_sources = names;
                                finding.event_ns = Some(event);
                                finding.unit = Some("ns".into());
                                finding.reason =
                                    "измеренная задержка физической реакции на воздействие".into();
                                if reaction.reaction_ns > reaction.criterion.within_ns {
                                    finding.outcome = Outcome::Fail;
                                    finding.confirmed_hazard = proved;
                                } else if proved && report.outcome == Outcome::Pass {
                                    finding.outcome = Outcome::Pass;
                                }
                            }
                        }
                        _ => {}
                    }
                }
                vec![finding]
            }
        };
        Ok(self.checked_findings(set, check, key, &sources, details))
    }

    fn kinematic_limit(&self, key: &RequirementKey, limit: &Limit) -> Option<Limit> {
        let Limit::Range { signal, mut min, mut max, source } = limit.clone() else { return None };
        let named = self.joints.iter().find(|joint| joint.name == key.object);
        let scalar_units = |kind| match kind {
            JointKind::Hinge => Some(("rad", "rad/s")),
            JointKind::Slide => Some(("m", "m/s")),
            _ => None,
        };
        let (name, unit) = match key.class {
            HazardClass::TravelJam => {
                let joint = named?;
                if joint.limited {
                    let [low, high] = joint.range;
                    min = Some(min.map_or(low, |given| given.max(low)));
                    max = Some(max.map_or(high, |given| given.min(high)));
                } else {
                    (min, max) = symmetric_limit(min, max)?;
                }
                (format!("qpos[{}]", joint.qpos_adr), scalar_units(joint.kind)?.0)
            }
            HazardClass::Fall => {
                // У свободного тела координата в направлении тяжести
                // проверяет заявленную границу падения по высоте.
                let joint = named.filter(|joint| joint.kind == JointKind::Free)?;
                let gravity = self.sampled.world.world.gravity_m_s2;
                let axis = gravity.iter().position(|component| *component != 0.0)?;
                if gravity.iter().filter(|component| **component != 0.0).count() != 1
                    || if gravity[axis] < 0.0 { min.is_none() } else { max.is_none() }
                {
                    return None;
                }
                (format!("qpos[{}]", joint.qpos_adr + axis), "m")
            }
            HazardClass::DangerousMotion => {
                let joint = named.or_else(|| match self.joints.as_slice() {
                    // На одноосном стенде нет других скоростей, между
                    // которыми мог бы выбирать автор постановки.
                    [joint] => Some(joint),
                    _ => None,
                })?;
                (min, max) = symmetric_limit(min, max)?;
                (format!("qvel[{}]", joint.dof_adr), scalar_units(joint.kind)?.1)
            }
            _ => return None,
        };
        (signal.name == name && signal.unit == unit && min.zip(max).is_none_or(|(a, b)| a <= b))
            .then_some(Limit::Range { signal, min, max, source })
    }

    fn section_rotation(
        &self, finding: &mut Finding, window: Window, allowable_pa: [f64; 2], proved_limit: bool,
    ) -> Result<(), Error> {
        if !self.section_world_matches {
            return Ok(());
        }
        let object = &finding.key.object;
        let Some((placement, body)) = object.split_once('/') else { return Ok(()) };
        let world = &self.sampled.world;
        let part = world.devices.get(placement)
            .and_then(|input| input.device.links.iter().find(|link| link.name == body)
                .map(|link| (&link.mass, link.collision.as_slice(), input.path.as_path())))
            .or_else(|| {
                world.world.objects.iter().chain(&world.contact.added_objects)
                    .find(|item| item.name == placement && item.name == body)
                    .map(|item| (&item.mass, item.collision.as_slice(), world.path.as_path()))
            });
        let Some((mass, collision, path)) = part else { return Ok(()) };
        let (material, geometries) = match mass {
            Mass::Geometry { material } => (Some(material), collision),
            Mass::Volumes { material, volumes } => (Some(material), volumes.as_slice()),
            _ => (None, collision),
        };
        let signals = section_reaction::angular_channels(object);
        let mut peak = None::<([f64; 2], u64, Fingerprint, Option<[f64; 2]>)>;
        let mut models = None;
        let asset_base = path.parent().unwrap_or_else(|| std::path::Path::new(""));
        for log in std::iter::once(&self.log).chain(&self.divergent_repeat) {
            let [x, y, z] = signals.each_ref().map(|signal| log.series(signal));
            let [x, y, z] = [x?, y?, z?];
            let journal = Fingerprint::from_fnv1a(log.fingerprint);
            for ((x, y), z) in x.points.iter().zip(&y.points).zip(&z.points) {
                if !(window.start_ns..=window.end_ns).contains(&x.at_ns)
                    || x.at_ns != y.at_ns || x.at_ns != z.at_ns || !(x.valid && y.valid && z.valid)
                {
                    continue;
                }
                let angular_velocity = [x.value, y.value, z.value];
                if angular_velocity == [0.0; 3] {
                    continue;
                }
                if models.is_none() {
                    models = Some(geometries.iter().map(|geometry| {
                        geometry.centrifugal_model(material, asset_base).map_err(invalid)
                    }).collect::<Result<Vec<_>, _>>()?);
                }
                for model in models.iter().flatten().flatten() {
                    let stress = model.stress(angular_velocity).map_err(invalid)?;
                    let reference = stress.disk_reference_pa
                        .map_or(stress.normal_pa, |[radial, hoop]| stress.normal_pa.max(radial).max(hoop));
                    let envelope = stress.disk_envelope_pa.map_or(reference, |upper| reference.max(upper));
                    if peak.is_none_or(|(old, _, _, _)| envelope > old[1]) {
                        peak = Some(([reference, envelope], x.at_ns, journal, stress.disk_reference_pa));
                    }
                }
            }
        }
        if models.is_some_and(|models| models.is_empty() || models.iter().any(Option::is_none)) {
            finding.coverage_gaps.push("для вращающейся формы нет замкнутого объёма или плотности".into());
        }
        let Some((stress, at, journal, disk_reference)) = peak else { return Ok(()) };
        let fails = stress[1] > allowable_pa[1];
        if let Some(old) = finding.measurement {
            // Подтверждённое превышение сохраняет свой численный свидетель,
            // даже если у другой оценки выше лишь консервативная граница.
            if if finding.outcome == Outcome::Fail { stress[0] <= old[0] }
                else { !fails && stress[1] <= old[1] }
            {
                return Ok(());
            }
        }
        finding.measurement = Some(stress);
        finding.event_ns = Some(at);
        finding.journal = Some(journal);
        finding.outcome = if fails { Outcome::Fail } else { Outcome::NotAnalyzed };
        finding.confirmed_hazard = fails && proved_limit && stress[0] > allowable_pa[1];
        finding.reason = if let Some([radial, hoop]) = disk_reference {
            format!(
                "центробежные напряжения: радиальное {radial:.3} Па и окружное {hoop:.3} Па при справочном ν = 0,3; верхняя огибающая по ν {:.3} Па",
                stress[1],
            )
        } else {
            "центробежное растяжение по балансу нормальной силы в объёме формы".into()
        };
        Ok(())
    }

    fn section_samples(
        &self, object: &str, window: Window, load: &SectionLoad,
    ) -> Result<Option<SectionWindow<'_>>, Error> {
        let world = &self.sampled.world;
        let Some((placement, body)) = object.split_once('/') else {
            return Ok(None);
        };
        let Some(device) = world.devices.get(placement).map(|input| &input.device) else {
            return Ok(None);
        };
        let Some(link) = device.links.iter().find(|link| link.name == body) else {
            return Ok(None);
        };
        if !link.joint.efforts.iter().any(|e| e.dof == 0 && format!("{placement}/{}", e.name) == load.effort) {
            return Ok(None);
        }
        // Прежняя огибающая оснастки применима на изолированной оси.
        // Реакцию корня можно измерить и при дополнительных путях нагрузки,
        // но нужна отдельная проверка, какие сечения они нагружают.
        let complex_path = device.links.len() != 1 || world.devices.len() != 1 || !world.world.objects.is_empty()
            || !world.contact.added_objects.is_empty() || !world.world.attachments.is_empty()
            || !world.contact.added_attachments.is_empty() || !device.constraints.is_empty()
            || !device.transmissions.is_empty() || !world.world.environment.collision.is_empty()
            || world.world.environment.density_kg_m3 != 0.0
            || world.world.environment.viscosity_pa_s != 0.0 || link.joint.limits_deg.is_some()
            || link.joint.limits_m.is_some() || link.joint.damping != Some(0.0)
            || link.joint.friction_loss != Some(0.0);
        let unit = match link.joint.kind {
            sim_world::JointType::Slide => "N",
            sim_world::JointType::Hinge => "N*m",
            _ => return Ok(None),
        };
        let signal = crate::Signal { name: format!("applied_effort/{}", load.effort), unit: unit.into() };
        let rate = sim_signal::Rate::hz(world.world.physics_hz).ok_or_else(|| invalid("частота нагрузки вне шкалы времени"))?;
        let reaction_signals = section_reaction::channels(object);
        let motion_signals = section_reaction::motion_channels(object, link.joint.kind == JointType::Slide);
        let mut samples = Vec::new();
        let mut complete = true;
        for log in std::iter::once(&self.log).chain(&self.divergent_repeat) {
            let series = std::iter::once(&signal).chain(&reaction_signals).chain(&motion_signals).map(|signal| log.series(signal))
                .collect::<Result<Vec<_>, _>>()?;
            let mut channels: Vec<_> = series.iter()
                .map(|series| series.points.iter().peekable()).collect();
            let journal = Fingerprint::from_fnv1a(log.fingerprint);
            let mut left = 0;
            for index in 1..=log.end.steps {
                let right = rate.tick(index).0;
                if right > window.start_ns && left < window.end_ns {
                    let mut values = [None; 13];
                    for (value, points) in values.iter_mut().zip(&mut channels) {
                        while points.peek().is_some_and(|point| point.at_ns < right) {
                            points.next();
                        }
                        if let Some(point) = points.next_if(|point| point.at_ns == right).filter(|point| point.valid) {
                            crate::task::finite(point.value)?;
                            *value = Some(point.value);
                        }
                    }
                    // Разрыв запрещает Pass. Уже записанное превышение
                    // в другом интервале остаётся свидетельством.
                    if let [Some(effort), Some(fx), Some(fy), Some(fz), Some(mx), Some(my), Some(mz), motion @ ..] = values {
                        let motion = match motion {
                            [Some(speed), Some(acceleration), Some(gx), Some(gy), Some(gz), Some(mass)] =>
                                Some(Motion { speed, acceleration, gravity: [gx, gy, gz], mass }),
                            _ => None,
                        };
                        samples.push(SectionSample {
                            effort: effort.abs(), at: left.max(window.start_ns), journal,
                            reaction: Wrench { force: [fx, fy, fz], moment: [mx, my, mz] },
                            motion,
                        });
                        complete &= right - left <= window.max_gap_ns;
                    } else {
                        complete = false;
                    }
                }
                left = right;
                if left >= window.end_ns {
                    break;
                }
            }
            complete &= left >= window.end_ns;
        }
        Ok((!samples.is_empty()).then_some(SectionWindow { link, samples, complete, complex_path }))
    }

    fn checked_findings(
        &self,
        set: &RequirementSet,
        check: &Check,
        key: &RequirementKey,
        sources: &[Source],
        mut details: Vec<Finding>,
    ) -> Vec<Finding> {
        let (proved, source_names) = trace(sources, &set.derivations);
        for item in &mut details {
            item.key = key.clone();
            if item.model_scope.is_none() {
                item.model_scope = Some(check.scope.clone());
            }
            item.measurement_sources
                .extend(source_names.iter().cloned());
            if !proved {
                item.coverage_gaps
                    .push("не подтверждены источник входов, предел или применимость метода".into());
                item.confirmed_hazard = false;
                if item.outcome != Outcome::Fail || key.class != HazardClass::TaskCriterion {
                    item.outcome = Outcome::NotAnalyzed;
                }
            }
            if check.verification.measurement_limit().is_none() {
                if key.class == HazardClass::StaticStrength && item.outcome != Outcome::Fail {
                    item.outcome = Outcome::NotAnalyzed;
                    item.reason.push_str(". ");
                    item.reason.push_str(STRENGTH_REQUIRES_BENCH);
                    item.missing_data.push(STRENGTH_REQUIRES_BENCH.into());
                } else if matches!(key.class, HazardClass::TissueLoad | HazardClass::Ingress)
                    && item.outcome == Outcome::Pass
                {
                    item.outcome = Outcome::NotAnalyzed;
                    item.missing_data.push("требуется заверенный замер этой ревизии".into());
                }
            }
            self.finish(set, item);
        }
        details
    }
}

fn symmetric_limit(min: Option<f64>, max: Option<f64>) -> Option<(Option<f64>, Option<f64>)> {
    match (min, max) {
        (None, Some(upper)) if upper >= 0.0 => Some((Some(-upper), max)),
        (Some(lower), None) if lower <= 0.0 => Some((min, Some(-lower))),
        (Some(_), Some(_)) => Some((min, max)),
        _ => None,
    }
}

fn compare_truth(
    finding: &mut Finding,
    limit: &Limit,
    window: Window,
    series: &Series,
    uncertainty: f64,
    set: &RequirementSet,
) -> Result<(), Error> {
    let points: Vec<_> = series
        .points
        .iter()
        .filter(|p| p.valid && (window.start_ns..=window.end_ns).contains(&p.at_ns))
        .collect();
    let (proved, refs) = trace(std::slice::from_ref(limit_source(limit)), &set.derivations);
    finding.limit_sources = refs;
    finding.unit = Some(limit.signal().unit.clone());
    finding.limit = Some(match limit {
        Limit::Range { min, max, .. } => LimitValue {
            lower: min.map(|v| [v; 2]),
            upper: max.map(|v| [v; 2]),
        },
        // По формату событие равно 1, отсутствие события равно 0.
        Limit::Forbidden { .. } => LimitValue::upper([0.0; 2]),
    });
    if let Some(first) = points.first() {
        let [lo, hi] = points.iter().fold([first.value; 2], |[lo, hi], p| {
            [lo.min(p.value), hi.max(p.value)]
        });
        finding.measurement = Some(if uncertainty == 0.0 {
            [lo, hi]
        } else {
            [lo - uncertainty, hi + uncertainty]
        });
    }
    let violation = points.iter().find(|point| match limit {
        Limit::Range { min, max, .. } => {
            min.is_some_and(|limit| point.value + uncertainty < limit)
                || max.is_some_and(|limit| point.value - uncertainty > limit)
        }
        Limit::Forbidden { .. } => uncertainty == 0.0 && point.value == 1.0,
    });
    let invalid_events = matches!(limit, Limit::Forbidden { .. })
        && (uncertainty != 0.0 || points.iter().any(|p| p.value != 0.0 && p.value != 1.0));
    let possibly_exceeded = match (limit, finding.measurement) {
        (Limit::Range { min, max, .. }, Some([lo, hi])) => {
            min.is_some_and(|limit| lo < limit) || max.is_some_and(|limit| hi > limit)
        }
        _ => false,
    };
    let complete = series.slice(window).is_some_and(|p| {
        p.first().is_some_and(|p| p.at_ns == window.start_ns)
            && p.last().is_some_and(|p| p.at_ns == window.end_ns)
    }) && !invalid_events;
    if !complete {
        finding
            .coverage_gaps
            .push("сигнал, границы или шаг окна не подтверждены".into());
    }
    if !proved {
        finding
            .coverage_gaps
            .push("источник предела не подтверждён".into());
    }
    finding.outcome = if let Some(point) = violation {
        finding.event_ns = Some(point.at_ns);
        finding.reason = "наблюдалось нарушение критерия".into();
        finding.confirmed_hazard = proved && finding.key.class != HazardClass::TaskCriterion;
        Outcome::Fail
    } else if possibly_exceeded {
        finding.reason = "граница погрешности пересекает предел".into();
        finding.coverage_gaps.push(finding.reason.clone());
        Outcome::NotAnalyzed
    } else if complete && proved {
        finding.reason = "в полном окне соблюдён критерий".into();
        Outcome::Pass
    } else {
        Outcome::NotAnalyzed
    };
    Ok(())
}
