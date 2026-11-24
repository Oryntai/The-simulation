use serde::Deserialize;
use sim_process::{Analysis, work::mechanical_limits::normal_load};
use sim_runtime::Runtime;
use sim_task::{
    Fingerprint, Outcome, Source, SourceKind,
    campaign::RunKey,
    process::ConvergenceReport,
    verdict::{Finding, HazardClass, Level, LimitValue, RequirementKey},
};

use super::{Case, models};

fn fixed_inputs<'a>(inputs: impl IntoIterator<Item = &'a super::Scalar>) -> bool {
    inputs.into_iter().all(|input| {
        input.range.is_none_or(|range| range == [input.value; 2]) && direct_input(&input.source)
    })
}

fn direct_input(source: &sim_world::Provenance) -> bool {
    matches!(
        source,
        sim_world::Provenance::Datasheet(_)
            | sim_world::Provenance::Paper(_)
            | sim_world::Provenance::Measured(_)
    )
}

/// Односторонняя проверка: превышение отклоняет постановку, успешная пара
/// сохраняет численный результат и требует отдельного доказательства для железа.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Screen {
    object: String,
    scope: String,
    source: Source,
    limit_source: Source,
    limit: f64,
    condition: Condition,
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Condition {
    DryContact {
        surface: String,
        region: String,
        contact_area_m2: f64,
        body_area_m2: f64,
        max_area_fraction: f64,
        duration_ns: u64,
        limit_surface: String,
        dry_bare_skin: bool,
        head_or_vital_face: bool,
        max_surface_drift_k: f64,
    },
    AccessibleDc {
        live_parts: [String; 2],
        guarding: bool,
        ripple_free: bool,
        voltage_only_protection: bool,
        path_when_positive: [BodyContact; 2],
        limit_path: [BodyContact; 2],
        limit_duration_ns: u64,
        exposure: Exposure,
        dry_limit_v: f64,
        wet_limit_v: f64,
        voltage_source: String,
    },
    Measurement {
        revision: String,
        mode: String,
        unit: String,
        uncertainty: f64,
        synthetic: bool,
    },
}

#[derive(Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
enum BodyContact {
    Hand,
    LeftHand,
    RightHand,
    Feet,
    Torso,
    Other,
}

#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
enum Exposure {
    DrySmallArea,
    DryLargeArea,
    Wet,
    BodyWorn,
}

pub(super) struct Observation {
    bounds: [f64; 2],
    at_ns: u64,
    journal: Fingerprint,
    current_applicable: bool,
    voltage: Option<[f64; 2]>,
}

impl Screen {
    fn class(&self) -> HazardClass {
        match self.condition {
            Condition::DryContact { .. } => HazardClass::Burns,
            Condition::AccessibleDc { .. } => HazardClass::ElectricShock,
            Condition::Measurement { .. } => HazardClass::Measurement,
        }
    }

    fn unit(&self) -> &str {
        match &self.condition {
            Condition::DryContact { .. } => "K",
            Condition::AccessibleDc { .. } => "A",
            Condition::Measurement { unit, .. } => unit,
        }
    }

    pub(super) fn validate(&self, case: &Case) -> Result<(), String> {
        let named = |s: &str| !s.trim().is_empty() && !s.chars().any(char::is_control);
        let valid = match &self.condition {
            Condition::DryContact {
                surface,
                region,
                contact_area_m2,
                body_area_m2,
                max_area_fraction,
                duration_ns,
                limit_surface,
                max_surface_drift_k,
                ..
            } => {
                named(surface)
                    && named(region)
                    && named(limit_surface)
                    && *duration_ns > 0
                    && [*contact_area_m2, *body_area_m2, *max_area_fraction]
                        .into_iter()
                        .all(|x| x.is_finite() && x > 0.0)
                    && *max_area_fraction <= 1.0
                    && max_surface_drift_k.is_finite()
                    && *max_surface_drift_k >= 0.0
            }
            Condition::AccessibleDc {
                live_parts,
                path_when_positive,
                limit_path,
                limit_duration_ns,
                dry_limit_v,
                wet_limit_v,
                voltage_source,
                ..
            } => {
                live_parts.iter().all(|s| named(s))
                    && live_parts[0] != live_parts[1]
                    && path_when_positive[0] != path_when_positive[1]
                    && limit_path[0] != limit_path[1]
                    && !limit_path.contains(&BodyContact::Other)
                    && *limit_duration_ns > 0
                    && named(voltage_source)
                    && dry_limit_v.is_finite()
                    && wet_limit_v.is_finite()
                    && *wet_limit_v > 0.0
                    && wet_limit_v <= dry_limit_v
            }
            Condition::Measurement {
                revision,
                mode,
                unit,
                uncertainty,
                ..
            } => {
                named(revision)
                    && named(mode)
                    && named(unit)
                    && uncertainty.is_finite()
                    && *uncertainty >= 0.0
                    && case.reference.is_some()
            }
        };
        if !valid
            || !named(&self.object)
            || !named(&self.scope)
            || !named(&self.source.reference)
            || !named(&self.limit_source.reference)
            || !self.limit.is_finite()
            || self.limit <= 0.0
        {
            return Err("некорректные данные проверки опасности".into());
        }
        Ok(())
    }

    pub(super) fn measurement(&self) -> bool {
        matches!(self.condition, Condition::Measurement { .. })
    }

    pub(super) fn observe(
        &self,
        case: &Case,
        mode_id: &str,
        runtime: &Runtime,
        reference: Option<&str>,
    ) -> Result<Observation, String> {
        let direct = |source: &Source| {
            matches!(
                source.kind,
                SourceKind::Datasheet | SourceKind::Paper | SourceKind::Measured
            )
        };
        if !direct(&self.source) || !direct(&self.limit_source) {
            return Err("условия и предел требуют прямых источников".into());
        }
        let [start, end] = case.window_ns;
        let mut at_ns = end;
        let mut current_applicable = true;
        let mut voltage = None;
        let bounds = match &self.condition {
            Condition::DryContact {
                surface,
                region,
                contact_area_m2,
                body_area_m2,
                max_area_fraction,
                duration_ns,
                limit_surface,
                dry_bare_skin,
                head_or_vital_face,
                max_surface_drift_k,
            } => {
                let Some(models::Spec::Heat {
                    pair,
                    area_m2,
                    capacities_j_k,
                    initial_k,
                    coefficient_w_m2_k,
                    power_w,
                    nominal_only: false,
                    ..
                }) = case.models.get(&self.object)
                else {
                    return Err("нет применимой тепловой модели пары".into());
                };
                if !dry_bare_skin
                    || *head_or_vital_face
                    || area_m2.value != *contact_area_m2
                    || !fixed_inputs(
                        [area_m2, coefficient_w_m2_k]
                            .into_iter()
                            .chain(capacities_j_k)
                            .chain(initial_k),
                    )
                    // При пассивном остывании двух теплоёмкостей температура
                    // между границами монотонна на каждом интервале касания.
                    || power_w.iter().any(|command| command.value != 0.0)
                    || case.model_mounts.contains_key(&self.object)
                    || surface != limit_surface
                    || region != &pair[1]
                    || end - start != *duration_ns
                    || contact_area_m2 / body_area_m2 >= *max_area_fraction
                    || !runtime.process_contact_pairs().any(|p| {
                        (p.pair.geometries == *pair
                            || p.pair.geometries.iter().eq(pair.iter().rev()))
                            && matches!(p.nominal_coverage, Analysis::Analyzed { .. })
                    })
                {
                    return Err(
                        "сухое касание вне области нормы по материалу, зоне, площади или времени"
                            .into(),
                    );
                }
                // У модели две изотермические ёмкости. До начала окна
                // берётся предыдущий завершённый отсчёт, поэтому окно
                // содержит обе границы даже для остывающей поверхности.
                let bounds = self.process_bounds(
                    runtime,
                    [start, end],
                    "source",
                    "K",
                    |x| x,
                    |frame| {
                        normal_load(frame.interval, &frame.reactions, pair)
                            .is_ok_and(|force| force > 0.0)
                    },
                )?;
                if bounds[1] - bounds[0] > *max_surface_drift_k {
                    return Err(
                        "поверхность не удерживает температуру на протяжении касания".into(),
                    );
                }
                bounds
            }
            Condition::AccessibleDc {
                guarding: false,
                ripple_free: true,
                voltage_only_protection,
                path_when_positive,
                limit_path,
                limit_duration_ns,
                ..
            } => {
                let Some(models::Spec::Electrical {
                    latch: None,
                    initially_closed,
                    fault,
                    source,
                    supply_v,
                    source_ohm,
                    closed_ohm,
                    load_ohm,
                    supply_override_v: None,
                    ..
                }) = case.models.get(&self.object)
                else {
                    return Err("нет живой резистивной цепи доступного контакта".into());
                };
                let closed = match fault {
                    Some(models::ElectricalFault::StuckClosed) => true,
                    Some(models::ElectricalFault::OpenCircuit) => false,
                    None => *initially_closed,
                };
                if !closed
                    || !direct_input(source)
                    || !fixed_inputs([supply_v, source_ohm, closed_ohm, load_ohm])
                    || case.model_mounts.contains_key(&self.object)
                {
                    return Err("цепь прикосновения не замкнута на нагрузку".into());
                }
                let mut path = *path_when_positive;
                if supply_v.value < 0.0 {
                    path.reverse();
                }
                current_applicable = path == *limit_path && end - start == *limit_duration_ns;
                if *voltage_only_protection {
                    // Омическая модель замкнута. Напряжение, доступное до
                    // прикосновения, равно ЭДС этого постоянного источника.
                    voltage = Some([supply_v.value.abs(); 2]);
                }
                self.process_bounds(runtime, [start, end], "current", "A", f64::abs, |_| true)?
            }
            Condition::AccessibleDc { .. } => {
                return Err(
                    "предел относится к открытым полюсам и к постоянному напряжению без пульсаций"
                        .into(),
                );
            }
            Condition::Measurement {
                revision,
                unit,
                mode,
                uncertainty,
                synthetic: true,
                ..
            } => {
                if mode != mode_id || start != 0 {
                    return Err("ряд не привязан к исполняемому режиму и полному горизонту".into());
                }
                let spec = case.reference.as_ref().ok_or("нет описания ряда")?;
                let (error, time) = spec.absolute_difference(
                    reference.ok_or("нет данных ряда")?,
                    runtime,
                    unit,
                    revision,
                )?;
                at_ns = time;
                let bounds = [(error - uncertainty).max(0.0), error + uncertainty];
                if !bounds[1].is_finite() {
                    return Err(format!("режим «{mode}»: переполнение ошибки ряда"));
                }
                bounds
            }
            Condition::Measurement { .. } => {
                return Err("для натурного ряда нужна проверенная привязка версии, калибровки и синхронизации".into());
            }
        };
        Ok(Observation {
            bounds,
            at_ns,
            journal: Fingerprint::from_fnv1a(runtime.log().fingerprint()),
            current_applicable,
            voltage,
        })
    }

    fn process_bounds(
        &self,
        runtime: &Runtime,
        [start, end]: [u64; 2],
        field: &str,
        unit: &str,
        transform: impl Fn(f64) -> f64,
        applicable: impl Fn(&sim_runtime::process::PhysicsFrame) -> bool,
    ) -> Result<[f64; 2], String> {
        let signal = format!("{}/{field}", self.object);
        let mut previous = start;
        let mut bounds: Option<[f64; 2]> = None;
        let mut began = false;
        for tick in runtime.process_journal() {
            let Some(frame) = &tick.completed else {
                continue;
            };
            if frame.interval.end.0 < start || frame.interval.end.0 > end {
                continue;
            }
            let at = frame.interval.end.0;
            began |= at == start;
            if at > start {
                if frame.interval.start.0 != previous
                    || !frame.contact_gaps.is_empty()
                    || !applicable(frame)
                {
                    return Err("нет непрерывного применимого ряда в контрольном окне".into());
                }
                previous = at;
            }
            let mut found = tick
                .outputs
                .get(&self.object)
                .into_iter()
                .flat_map(|output| &output.measurements)
                .filter(|measurement| {
                    measurement.signal.name == signal
                        && measurement.signal.unit == unit
                        && measurement.at.0 == at
                });
            let Some(measurement) = found.next() else {
                return Err(format!("нет отсчёта «{signal}» на {at} нс"));
            };
            let Analysis::Analyzed { value, .. } = &measurement.result else {
                return Err(format!("не доказан отсчёт «{signal}» на {at} нс"));
            };
            let value = transform(*value);
            if found.next().is_some() || !value.is_finite() || value < 0.0 {
                return Err(format!("невалидный отсчёт «{signal}» на {at} нс"));
            }
            bounds = Some(bounds.map_or([value; 2], |[lo, hi]| [lo.min(value), hi.max(value)]));
        }
        bounds
            .filter(|_| began && previous == end)
            .ok_or_else(|| "живой ряд не достиг конца контрольного окна".into())
    }

    pub(super) fn findings(
        &self,
        case: &Case,
        run: &RunKey,
        convergence: &ConvergenceReport,
        observations: Vec<Result<Observation, String>>,
    ) -> Vec<Finding> {
        let key = RequirementKey {
            class: self.class(),
            object: self.object.clone(),
            extension: None,
            run: Some(run.clone()),
            window_ns: Some(case.window_ns),
            level: Level::Simulation,
        };
        let mut finding = Finding::open(&key, "нет полной проверки на всех расчётных сетках");
        finding.unit = Some(self.unit().into());
        finding.limit = Some(LimitValue {
            lower: None,
            upper: Some([self.limit; 2]),
        });
        finding
            .limit_sources
            .push(self.limit_source.reference.clone());
        finding
            .measurement_sources
            .push(self.source.reference.clone());
        finding.model_scope = Some(self.scope.clone());
        if let Some(reason) = observations.iter().find_map(|result| result.as_ref().err()) {
            finding.reason.clone_from(reason);
            finding.missing_data = vec![reason.clone()];
            return vec![finding];
        }
        if observations.is_empty()
            || observations.len() != convergence.trials.len()
            || convergence
                .trials
                .iter()
                .any(|trial| !trial.completed || trial.steps == 0)
        {
            return vec![finding];
        }
        let observations = observations.into_iter().flatten().collect::<Vec<_>>();
        let first = &observations[0];
        let bounds = observations.iter().fold(first.bounds, |[lo, hi], obs| {
            [lo.min(obs.bounds[0]), hi.max(obs.bounds[1])]
        });
        finding.measurement = Some(bounds);
        finding.event_ns = Some(first.at_ns);
        finding.journal = Some(first.journal);
        if observations.iter().any(|o| !o.current_applicable) {
            finding.reason = "для пути, направления или длительности ряда нет применимой границы постоянного тока".into();
            finding.missing_data = vec![finding.reason.clone()];
        } else if bounds[0] > self.limit {
            finding.reason =
                "нижняя граница живого ряда превышает применимый предел на всех сетках".into();
            finding.outcome = Outcome::Fail;
            finding.confirmed_hazard = true;
            finding.missing_data.clear();
        } else {
            finding.reason = "ряд не подтверждает превышение применимого предела".into();
            finding.missing_data = vec!["для вывода о железе нужны стендовые измерения".into()];
        }
        let mut findings = vec![finding.clone()];
        if let Condition::AccessibleDc {
            voltage_only_protection: true,
            exposure,
            dry_limit_v,
            wet_limit_v,
            voltage_source,
            ..
        } = &self.condition
        {
            let limit = match exposure {
                Exposure::DrySmallArea => *dry_limit_v,
                Exposure::DryLargeArea | Exposure::Wet | Exposure::BodyWorn => *wet_limit_v,
            };
            let mut voltage = Finding::open(
                &key,
                "для защиты нужны подтверждённые условия касания и меры защиты",
            );
            voltage.code = "доступное напряжение".into();
            voltage.unit = Some("V".into());
            voltage.limit = Some(LimitValue {
                lower: None,
                upper: Some([limit; 2]),
            });
            voltage.limit_sources.push(voltage_source.clone());
            voltage.model_scope = finding.model_scope;
            voltage.measurement_sources = finding.measurement_sources;
            voltage.event_ns = Some(first.at_ns);
            voltage.journal = Some(first.journal);
            voltage.measurement = first.voltage.and_then(|initial| {
                observations
                    .iter()
                    .skip(1)
                    .try_fold(initial, |[lo, hi], row| {
                        let [a, b] = row.voltage?;
                        Some([lo.min(a), hi.max(b)])
                    })
            });
            if voltage.measurement.is_some_and(|[lo, _]| lo > limit) {
                voltage.outcome = Outcome::Fail;
                voltage.confirmed_hazard = true;
                voltage.reason =
                    "доступное напряжение превышает предел для открытых полюсов".into();
                voltage.missing_data.clear();
            }
            findings.push(voltage);
        }
        findings
    }
}
