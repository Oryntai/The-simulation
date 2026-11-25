//! Вердикт конструктора. У каждой группы должна быть собственная положительная проверка.

mod electrical;
mod firmware;
mod human;
mod stage5_gate;
mod structure;
pub use stage5_gate::{InteractionInput, interaction_discrepancies};
pub use structure::nominal_bending_pa;

use std::path::{Path, PathBuf};

use sim_device::measure::{MeasureResult, MeasureStatus, Quantity, compare};
use sim_device::{Analysis, BodyRegion, Device, JointKind};

use crate::blueprint::BuildVerdict;
use crate::device_catalog::WorkshopCatalog;

/// Погрешность термопары и разброс воздуха у стенда, °C.
const COLD_MARGIN_C: f64 = 5.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    Pass,
    Fail,
    NotAnalyzed,
}

// Старый конструктор сохраняет форму строки и собственные проверки полноты групп.
pub use sim_task::verdict::{Group, Readiness};

#[derive(Clone, Debug, PartialEq)]
pub struct Finding {
    pub group: Group,
    pub code: &'static str,
    pub link: Option<usize>,
    pub title: String,
    pub outcome: Outcome,
    pub value: f64,
    pub limit: f64,
    pub unit: &'static str,
    pub detail: String,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Prediction {
    pub quantity: Quantity,
    pub link: Option<usize>,
    pub value: Option<f64>,
    pub applicable: bool,
    pub limit: Option<f64>,
}

#[derive(Clone, Debug, Default)]
pub struct DeviceReport {
    pub findings: Vec<Finding>,
    pub analysis: Option<Analysis>,
    pub human: Option<sim_human::HumanReport>,
    pub predictions: Vec<Prediction>,
    pub measures: Vec<MeasureResult>,
    pub uart: String,
    pub interaction_discrepancies: Vec<sim_task::interaction::ReferenceDiscrepancy>,
}

impl DeviceReport {
    pub fn verdict(&self) -> BuildVerdict {
        verdict(&self.findings)
    }

    pub fn readiness(&self) -> Readiness {
        readiness(&self.findings)
    }

    fn finding(
        &mut self,
        group: Group,
        code: &'static str,
        link: Option<usize>,
        title: impl Into<String>,
        outcome: Outcome,
        detail: impl Into<String>,
    ) {
        self.numeric(group, code, link, title, outcome, 0.0, 0.0, "", detail);
    }

    fn numeric(
        &mut self,
        group: Group,
        code: &'static str,
        link: Option<usize>,
        title: impl Into<String>,
        outcome: Outcome,
        value: f64,
        limit: f64,
        unit: &'static str,
        detail: impl Into<String>,
    ) {
        let detail = detail.into();
        let (outcome, value, limit, detail) = if value.is_finite() && limit.is_finite() {
            (outcome, value, limit, detail)
        } else {
            (
                Outcome::Fail,
                0.0,
                0.0,
                format!("Неконечный результат расчёта: {detail}"),
            )
        };
        self.findings.push(Finding {
            group,
            code,
            link,
            title: title.into(),
            outcome,
            value,
            limit,
            unit,
            detail,
        });
    }

    fn predict(&mut self, quantity: Quantity, link: Option<usize>, value: f64, limit: Option<f64>) {
        if !value.is_finite() || limit.is_some_and(|v| !v.is_finite()) {
            self.finding(
                Group::Measurements,
                "prediction",
                link,
                quantity.title(),
                Outcome::Fail,
                "Неконечный результат предсказания",
            );
            return;
        }
        if let Some(p) = self
            .predictions
            .iter_mut()
            .find(|p| p.quantity == quantity && p.link == link)
        {
            use sim_device::measure::Danger;
            let worst = |a: f64, b: f64| match quantity.danger() {
                Danger::Higher => a.max(b),
                Danger::Lower => a.min(b),
            };
            p.value = Some(p.value.map_or(value, |old| worst(old, value)));
            p.limit = match (p.limit, limit) {
                (Some(a), Some(b)) => Some(if quantity.danger() == Danger::Higher {
                    a.min(b)
                } else {
                    a.max(b)
                }),
                (a, b) => a.or(b),
            };
            p.applicable = true;
        }
    }
}

macro_rules! outcome_from {
    ($($module:ident),+ $(,)?) => {$(
        impl From<$module::Outcome> for Outcome {
            fn from(o: $module::Outcome) -> Self {
                match o {
                    $module::Outcome::Pass => Self::Pass,
                    $module::Outcome::Fail => Self::Fail,
                    $module::Outcome::NotAnalyzed => Self::NotAnalyzed,
                }
            }
        }
    )+};
}
outcome_from!(
    sim_materials,
    sim_battery,
    sim_actuators,
    sim_boards,
    sim_human
);

fn task_outcomes(findings: &[Finding]) -> impl Iterator<Item = sim_task::Outcome> + '_ {
    findings
        .iter()
        .map(|f| {
            if !f.value.is_finite() || !f.limit.is_finite() {
                return sim_task::Outcome::Fail;
            }
            match f.outcome {
                Outcome::Pass => sim_task::Outcome::Pass,
                Outcome::NotAnalyzed => sim_task::Outcome::NotAnalyzed,
                Outcome::Fail => sim_task::Outcome::Fail,
            }
        })
        .chain(
            (!Group::ALL
                .into_iter()
                .all(|g| group_is_covered(findings, g)))
            .then_some(sim_task::Outcome::NotAnalyzed),
        )
}

fn group_is_covered(findings: &[Finding], group: Group) -> bool {
    findings
        .iter()
        .any(|f| f.group == group && f.outcome == Outcome::Pass && f.code != "participant_power")
}

pub fn verdict(findings: &[Finding]) -> BuildVerdict {
    use sim_task::interaction::{SafetyDecision, safety_decision};
    match safety_decision(task_outcomes(findings), true) {
        SafetyDecision::Rejected => BuildVerdict::Rejected,
        SafetyDecision::Passed => BuildVerdict::PassedSimulation,
        SafetyDecision::NeedsWork | SafetyDecision::BenchOnly => BuildVerdict::NotProven,
    }
}

/// То, что расчётом не закрыть: нужны стенд, замеры или решение по человеку.
/// `motor_data_gap`: производитель не опубликовал тепловые данные и ток привода, поэтому сводка
/// питания остаётся оценкой, пока их не снимут на стенде.
fn needs_real_bench(f: &Finding, motor_data_gap: bool) -> bool {
    f.outcome == Outcome::NotAnalyzed
        && (matches!(f.group, Group::Human | Group::Measurements)
            || matches!(
                f.code,
                "static_strength"
                    | "bending_stress"
                    | "closed_loop"
                    | "coupled_motion"
                    | "motion_limits"
                    | "guard_claim"
                    | "contact_coverage"
                    | "interaction_coverage"
                    | "interaction_exposure"
                    | "interaction_version"
                    | "passive_motion"
                    | "unpowered_person"
                    | "direct_motor"
                    | "motor_speed"
                    | "motor_current"
                    | "driver_temperature"
                    | "winding_temperature"
            )
            || (motor_data_gap && matches!(f.code, "power_budget" | "power_coverage")))
}

/// Незакрытые пункты: что заполнить в чертеже и что можно закрыть только на стенде.
pub fn open_items(findings: &[Finding]) -> (Vec<&Finding>, Vec<&Finding>) {
    let motor_data_gap = findings.iter().any(|f| {
        f.outcome == Outcome::NotAnalyzed
            && matches!(f.code, "motor_current" | "winding_temperature")
    });
    findings
        .iter()
        .filter(|f| f.outcome == Outcome::NotAnalyzed)
        .partition(|f| !needs_real_bench(f, motor_data_gap))
}

pub fn readiness(findings: &[Finding]) -> Readiness {
    use sim_task::interaction::safety_decision;
    let computed = Group::ALL
        .into_iter()
        .filter(|g| !matches!(g, Group::Human | Group::Measurements))
        .all(|g| group_is_covered(findings, g));
    safety_decision(
        task_outcomes(findings),
        computed && open_items(findings).0.is_empty(),
    )
}

/// Отпечаток версии устройства вместе с содержимым образа прошивки: новый замер привязывается к нему.
pub fn measurement_stamp(d: &Device, project_dir: &Path) -> u64 {
    sim_device::measure::design_stamp(
        d,
        &firmware::image_bytes(project_dir, &d.electronics.firmware),
    )
}

pub fn project_directory() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("каталог крейтов")
        .parent()
        .expect("каталог мастерской")
        .join("projects")
}

pub fn assess_device(d: &Device) -> DeviceReport {
    assess_device_in(d, &project_directory())
}

/// Относительный путь к образу прошивки разрешается от каталога проектов.
pub fn assess_device_in(d: &Device, project_dir: &Path) -> DeviceReport {
    assess_device_inner(d, project_dir, None, None)
}

/// Доказательство проверяется относительно версии именно этой сборки и ожидаемого задания.
pub struct ClosedLoopInput<'a> {
    pub expected: &'a sim_task::Assessor,
    pub evidence: &'a sim_task::Evidence,
}

pub fn assess_device_with_closed_loop(
    d: &Device,
    project_dir: &Path,
    closed_loop: ClosedLoopInput<'_>,
) -> DeviceReport {
    assess_device_inner(d, project_dir, Some(closed_loop), None)
}

pub fn assess_device_with_interaction(
    d: &Device,
    project_dir: &Path,
    closed_loop: Option<ClosedLoopInput<'_>>,
    interaction: InteractionInput<'_>,
) -> DeviceReport {
    assess_device_inner(d, project_dir, closed_loop, Some(interaction))
}

fn assess_device_inner(
    d: &Device,
    project_dir: &Path,
    closed_loop: Option<ClosedLoopInput<'_>>,
    interaction: Option<InteractionInput<'_>>,
) -> DeviceReport {
    let mut r = DeviceReport::default();
    let errors = d.validate();
    if !errors.is_empty() {
        for error in errors {
            r.finding(
                Group::Mechanics,
                "input",
                None,
                "Описание устройства",
                Outcome::Fail,
                error,
            );
        }
        stage5_gate::append(d, measurement_stamp(d, project_dir), None, &mut r);
        return r;
    }
    for q in Quantity::ALL {
        let mut add = |link, applicable| {
            r.predictions.push(Prediction {
                quantity: q,
                link,
                applicable,
                value: None,
                limit: None,
            })
        };
        if q.per_link() {
            for (i, l) in d.links.iter().enumerate() {
                let active = !l.joint.actuator.is_empty();
                let applies = match q {
                    Quantity::JointSpeed | Quantity::JointRequiredTorque => {
                        l.joint.kind == JointKind::Hinge
                    }
                    Quantity::ActuatorPeakTorque | Quantity::MotorTemperature => active,
                    Quantity::ImpactForce | Quantity::ClampForce => {
                        active
                            && d.subtree(i).into_iter().any(|j| {
                                d.links[j].near_body != BodyRegion::None
                                    || d.mount != BodyRegion::None
                            })
                    }
                    _ => true,
                };
                add(Some(i), applies);
            }
        } else {
            add(
                None,
                q != Quantity::SkinSurfaceTemperature
                    || d.mount != BodyRegion::None
                    || d.links.iter().any(|l| l.near_body != BodyRegion::None),
            );
        }
    }
    match sim_device::analyze(d, &WorkshopCatalog) {
        Ok(a) => {
            structure::evaluate(d, &a, &mut r);
            electrical::evaluate(d, Some(&a), &mut r);
            human::evaluate(d, &a, &mut r);
            human::reconcile_unpowered(d, &mut r);
            r.analysis = Some(a);
        }
        Err(errors) => {
            for detail in errors {
                let missing = detail.contains("нет в каталоге");
                r.finding(
                    Group::Mechanics,
                    "model",
                    None,
                    "Модель звеньев",
                    if missing {
                        Outcome::NotAnalyzed
                    } else {
                        Outcome::Fail
                    },
                    detail,
                );
            }
            electrical::evaluate(d, None, &mut r);
        }
    }
    firmware::evaluate(d, project_dir, &mut r, closed_loop);
    let stamp = measurement_stamp(d, project_dir);
    for m in &d.measurements {
        let p = r
            .predictions
            .iter()
            .find(|p| p.quantity == m.quantity && p.link == m.link);
        let mut result = compare(m, p.and_then(|p| p.value));
        if let Some(limit) = p.and_then(|p| p.limit) {
            let unsafe_value = match m.quantity.danger() {
                sim_device::measure::Danger::Higher => m.value > limit,
                sim_device::measure::Danger::Lower => m.value < limit,
            };
            if unsafe_value {
                result.status = MeasureStatus::RealIsWorse;
                result.detail = format!(
                    "Замер «{}» превышает безопасную границу: {} {}, граница {limit} {}",
                    m.quantity.title(),
                    m.value,
                    m.quantity.unit(),
                    m.quantity.unit()
                );
            }
        }
        // Прибор не покажет температуру ниже самого холодного воздуха, при котором шёл замер.
        let coldest = d
            .settings
            .cold_c
            .unwrap_or(d.usage.ambient_c)
            .min(d.usage.ambient_c);
        if m.quantity.unit() == "°C"
            && m.value < coldest - COLD_MARGIN_C
            && result.status != MeasureStatus::RealIsWorse
        {
            result.status = MeasureStatus::Suspicious;
            result.detail = format!(
                "{}: замер {} °C ниже самого холодного воздуха в условиях работы ({coldest} °C). Проверьте датчик и режим",
                m.quantity.title(),
                m.value
            );
        }
        let stale = m.design != Some(stamp);
        if stale && result.status != MeasureStatus::RealIsWorse {
            result.detail = format!(
                "{}: замер снят на другой версии устройства. После правки конструкции его нужно повторить",
                m.quantity.title()
            );
        }
        let outcome = match result.status {
            MeasureStatus::RealIsWorse => Outcome::Fail,
            MeasureStatus::NoPrediction | MeasureStatus::Suspicious => Outcome::NotAnalyzed,
            _ if stale || !specified(Some(m.note.as_str())) => Outcome::NotAnalyzed,
            MeasureStatus::Agrees | MeasureStatus::RealIsBetter => Outcome::Pass,
        };
        r.finding(
            Group::Measurements,
            "measurement",
            m.link,
            m.quantity.title(),
            outcome,
            if !stale
                && matches!(
                    result.status,
                    MeasureStatus::Agrees | MeasureStatus::RealIsBetter
                )
                && m.note.trim().is_empty()
            {
                format!("{}. Укажите методику и условия замера.", result.detail)
            } else {
                result.detail.clone()
            },
        );
        r.measures.push(result);
    }
    let missing: Vec<_> = r
        .predictions
        .iter()
        .filter(|p| {
            p.applicable
                && !d
                    .measurements
                    .iter()
                    .any(|m| m.quantity == p.quantity && m.link == p.link)
        })
        .copied()
        .collect();
    for p in missing {
        r.finding(
            Group::Measurements,
            "missing_measurement",
            p.link,
            p.quantity.title(),
            Outcome::NotAnalyzed,
            "Нет стендового замера в расчётном режиме",
        );
    }
    stage5_gate::append(d, stamp, interaction, &mut r);
    r
}

fn specified(value: Option<&str>) -> bool {
    value.is_some_and(|s| !s.trim().is_empty())
}

fn upper(value: f64, limit: f64) -> Outcome {
    if value.is_finite() && limit.is_finite() && value <= limit {
        Outcome::Pass
    } else {
        Outcome::Fail
    }
}

fn lower(value: f64, limit: f64) -> Outcome {
    upper(limit, value)
}
