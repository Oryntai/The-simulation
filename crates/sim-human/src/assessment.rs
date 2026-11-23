//! Расчёт одного случая и выдача вердиктов для формы симулятора.

use nalgebra::Vector3;

use crate::data::*;
use crate::failure::{self, FAILURE_COORDS, FailureInput, FailureOutcome};
use crate::fatigue::{self, EnduranceVerdict, FatigueJoint};
use crate::lift::{MIN_JERK_PEAK_SPEED, min_jerk, pose_at};
use crate::load::{
    BodyScale, LoadInput, MEASURED_COORDS, RESIDUAL_TOL_NM, Strengths, calibrate, required_torque,
    solve_loads,
};
use crate::population::{JointAction, MuscleVariant, Person, Sex, muscle_variants, nhanes_body};
use crate::rig::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    Pass,
    Fail,
    NotAnalyzed,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HumanCheck {
    MuscleCapacity,
    ShoulderLoad,
    ElbowLoad,
    SpineCompression,
    SpineShear,
    DynamicLift,
    DriveFailure,
    Fatigue,
}

impl HumanCheck {
    pub const ALL: [Self; 8] = [
        Self::MuscleCapacity,
        Self::ShoulderLoad,
        Self::ElbowLoad,
        Self::SpineCompression,
        Self::SpineShear,
        Self::DynamicLift,
        Self::DriveFailure,
        Self::Fatigue,
    ];
}

#[derive(Clone, Debug, PartialEq)]
pub struct HumanFinding {
    pub kind: HumanCheck,
    pub outcome: Outcome,
    pub value: f64,
    pub limit: f64,
    pub unit: &'static str,
    pub detail: String,
}

/// Начало подъёма. Конец подъёма задаётся в `HumanLoadCase::pose`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiftInput {
    pub start_pose: BodyPose,
    /// Наибольшая угловая скорость среди задаваемых координат позы, рад/с.
    pub peak_speed_rad_s: f64,
}

/// Случай для правой руки, с точечным грузом и закреплённым тазом.
/// Помощь задаётся обобщёнными моментами. Силы креплений и нагрузки от массы
/// самого устройства из этих моментов не восстанавливаются.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HumanLoadCase {
    pub payload_kg: f64,
    /// Вынос центра масс груза от центра кисти по продолжению предплечья, м.
    pub lever_m: f64,
    pub pose: BodyPose,
    /// Помощь, Н·м, индексы C_*. Положительный L5/S1 FE разгибает поясницу.
    pub assist_nm: [f64; NC],
    pub person: Person,
    /// Длительность непрерывного удержания конечной позы после подъёма, с.
    pub work_time_s: f64,
    pub lift: Option<LiftInput>,
    /// Время линейного спада помощи от отказа до нуля, с. Ноль означает скачок.
    /// Отказ начинается в середине подъёма, либо в позе удержания при отсутствии подъёма.
    pub assist_loss_time_s: Option<f64>,
    /// Число нагружений поясницы за день для критерия Gallagher & Marras.
    pub cycles_per_day: Option<u32>,
}

#[derive(Clone, Debug)]
pub struct MuscleLoad {
    pub actuator: usize,
    pub name: String,
    pub activation: f64,
    pub tension_n: f64,
}

/// Реакции, Н. Плечо и локоть — модуль силы, поясница — компоненты относительно диска L5/S1.
#[derive(Clone, Copy, Debug, Default)]
pub struct ReactionPeaks {
    pub shoulder_n: f64,
    pub elbow_n: f64,
    pub spine_compression_n: f64,
    pub spine_shear_n: f64,
}

impl ReactionPeaks {
    fn include(&mut self, other: Self) {
        self.shoulder_n = self.shoulder_n.max(other.shoulder_n);
        self.elbow_n = self.elbow_n.max(other.elbow_n);
        self.spine_compression_n = self.spine_compression_n.max(other.spine_compression_n);
        self.spine_shear_n = self.spine_shear_n.max(other.spine_shear_n);
    }

    fn values(self) -> [f64; 4] {
        [
            self.shoulder_n,
            self.elbow_n,
            self.spine_compression_n,
            self.spine_shear_n,
        ]
    }
}

#[derive(Clone, Debug)]
pub struct LoadPoint {
    pub pose: BodyPose,
    pub required_nm: [f64; NC],
    pub muscle_nm: [f64; NC],
    /// Моменты фактически найденных мышечных тяг в номинальной геометрии, Н·м.
    pub achieved_muscle_nm: [f64; NC],
    /// Распределение в номинальной анатомии со снижением силы по наблюдениям.
    pub muscles: Vec<MuscleLoad>,
    pub max_activation: f64,
    pub residual_nm: f64,
    pub certified_deficit_nm: f64,
    pub variants_in_equilibrium: usize,
    /// Пики по вариантам, в которых найдено равновесие. При неполном покрытии это
    /// частичная оценка. Устройство здесь передаёт идеальный момент.
    pub reactions: Option<ReactionPeaks>,
    /// Независимая оценка сжатия по схеме Chaffin, Н.
    pub chaffin_compression_n: f64,
    /// Измерительная оценка для выбранного пола и перцентиля.
    pub selected_strength: Strengths,
    /// Консервативная оценка, использованная для решения.
    pub weak_strength: Strengths,
    /// Отношение исходной модели к оценкам слабого человека для этой позы.
    pub raw_model_to_weak_strength: Vec<(JointAction, f64)>,
}

#[derive(Clone, Debug)]
pub struct LiftReport {
    pub duration_s: f64,
    pub peak_required_nm: [f64; NC],
    /// Максимумы изометрических моментов тяжести в тех же позах.
    pub peak_static_nm: [f64; NC],
    pub max_activation: f64,
    pub max_static_activation: f64,
    pub peak_reactions: Option<ReactionPeaks>,
    pub samples: usize,
}

#[derive(Clone, Debug)]
pub struct DriveFailureReport {
    pub reaction_delay_s: f64,
    /// Удержание позы отказа после полного исчезновения помощи.
    pub unassisted_hold: LoadPoint,
    pub motion: Option<FailureOutcome>,
    /// Пиковая линейная скорость кисти вниз по отсчётам аварийной траектории, м/с.
    pub peak_hand_down_m_s: f64,
    /// Пики реакций по всем отсчётам с найденным равновесием после отказа.
    pub post_failure_reactions: Option<ReactionPeaks>,
    /// Пики в отсчётах, в которых после реакции падающий сустав замедляется.
    /// Удар об упор не входит. Если таких кадров с равновесием нет, возвращается None.
    pub braking_reactions: Option<ReactionPeaks>,
    pub braking_samples: usize,
    pub balanced_braking_samples: usize,
}

#[derive(Clone, Debug)]
pub struct HumanReport {
    pub findings: Vec<HumanFinding>,
    pub holding: Option<LoadPoint>,
    pub lift: Option<LiftReport>,
    pub failure: Option<DriveFailureReport>,
    pub endurance: Vec<EnduranceVerdict>,
}

impl HumanReport {
    /// Любой отказ имеет приоритет. Для общего Pass нужны восемь пройденных проверок.
    pub fn outcome(&self) -> Outcome {
        if self
            .findings
            .iter()
            .any(|f| f.outcome == Outcome::Fail || !f.value.is_finite() || !f.limit.is_finite())
        {
            Outcome::Fail
        } else if HumanCheck::ALL.iter().all(|kind| {
            self.findings
                .iter()
                .any(|f| f.kind == *kind && f.outcome == Outcome::Pass)
                && !self
                    .findings
                    .iter()
                    .any(|f| f.kind == *kind && f.outcome == Outcome::NotAnalyzed)
        }) {
            Outcome::Pass
        } else {
            Outcome::NotAnalyzed
        }
    }
}

// 5-й перцентиль — консервативная проектная оценка, не норматив безопасности.
const WEAK_PERCENTILE: f64 = 5.0;
// Численная сетка, включает середину и аналитические экстремумы ускорения.
const LIFT_SEGMENTS: usize = 24;
// Численный шаг центральных разностей, в долях длительности подъёма.
const MOTION_FD_FRACTION: f64 = 1e-4;
// Численный бюджет интегратора; при медленном спаде остаётся проверка полной потери помощи.
const MAX_SIMULATED_LOSS_S: f64 = 5.0;
// Оценка сходимости отказа при удвоении частоты интеграции, в долях пика и работы.
const FAILURE_NUMERICAL_TOL: f64 = 0.05;

fn all_finite(values: impl IntoIterator<Item = f64>) -> bool {
    values.into_iter().all(f64::is_finite)
}

fn finite_pose(p: &BodyPose) -> bool {
    all_finite(
        p.coords()
            .into_iter()
            .chain([p.trunk_inclination, p.pelvis_tilt()]),
    )
}

fn finding(
    kind: HumanCheck,
    outcome: Outcome,
    value: f64,
    limit: f64,
    unit: &'static str,
    detail: impl Into<String>,
) -> HumanFinding {
    let detail = detail.into();
    if !all_finite([value, limit]) {
        return HumanFinding {
            kind,
            outcome: Outcome::Fail,
            value: 0.0,
            limit: 0.0,
            unit,
            detail: format!("Неконечное число в расчёте. {detail}"),
        };
    }
    HumanFinding {
        kind,
        outcome,
        value,
        limit,
        unit,
        detail,
    }
}

fn unavailable(kind: HumanCheck, detail: &str) -> HumanFinding {
    finding(kind, Outcome::NotAnalyzed, 0.0, 0.0, "", detail)
}

fn empty_report(outcome: Outcome, detail: &str) -> HumanReport {
    HumanReport {
        findings: HumanCheck::ALL
            .map(|kind| finding(kind, outcome, 0.0, 0.0, "", detail))
            .to_vec(),
        holding: None,
        lift: None,
        failure: None,
        endurance: Vec::new(),
    }
}

fn validate(case: &HumanLoadCase) -> bool {
    let p = case.person;
    all_finite(
        [
            case.payload_kg,
            case.lever_m,
            case.work_time_s,
            p.age_years,
            p.mass_kg,
            p.stature_m,
            p.strength_percentile,
        ]
        .into_iter()
        .chain(case.assist_nm)
        .chain(case.assist_loss_time_s),
    ) && finite_pose(&case.pose)
        && case.payload_kg >= 0.0
        && case.lever_m >= 0.0
        && case.work_time_s >= 0.0
        && case.assist_loss_time_s.is_none_or(|v| v >= 0.0)
        && p.age_years > 0.0
        && p.mass_kg > 0.0
        && p.stature_m > 0.0
        && (1.0..=99.0).contains(&p.strength_percentile)
        && case.lift.is_none_or(|l| {
            finite_pose(&l.start_pose)
                && l.peak_speed_rad_s.is_finite()
                && l.peak_speed_rad_s > 0.0
                && l.start_pose != case.pose
        })
}

fn weak_strength(pose: &BodyPose, person: Person) -> Strengths {
    let (stature_m, mass_kg) = nhanes_body(Sex::Female, 0);
    let weak = Person {
        sex: Sex::Female,
        strength_percentile: person.strength_percentile.min(WEAK_PERCENTILE),
        stature_m: person.stature_m.min(stature_m),
        mass_kg: person.mass_kg.min(mass_kg),
        ..person
    };
    Strengths::at(pose, &weak).minimum(Strengths::at(pose, &person))
}

#[derive(Clone, Copy)]
struct CapacityTest {
    demand_nm: f64,
    limit_nm: f64,
    action: JointAction,
}

#[derive(Default)]
struct PointEvidence {
    overload: Option<CapacityTest>,
    strength_missing: bool,
    nominal_balanced: bool,
    weak_variant_torque: [f64; NC],
    /// Нужные доли MVC по основным координатам для оценки усталости.
    intensities: [f64; 3],
}

struct PointResult {
    point: LoadPoint,
    evidence: PointEvidence,
}

fn point(
    s: &GeometrySample,
    input: &LoadInput,
    person: Person,
    inertia: bool,
    with_muscles: bool,
) -> Result<PointResult, &'static str> {
    let required = required_torque(s, input, inertia);
    let muscle = std::array::from_fn(|k| required[k] - input.assist[k]);
    let selected = Strengths::at(&s.pose, &person);
    let weak = weak_strength(&s.pose, person);
    if !all_finite(
        required
            .into_iter()
            .chain(muscle)
            .chain(selected.values())
            .chain(weak.values()),
    ) || !s.is_finite()
        || s.muscles.is_empty()
    {
        return Err("Неконечные моменты, параметры мышц или геометрия.");
    }

    let calibration = calibrate(s, &weak, &input.scale);
    if !all_finite(
        calibration
            .factors
            .iter()
            .copied()
            .chain(calibration.overstrength.iter().map(|x| x.1)),
    ) {
        return Err("Неконечный коэффициент калибровки силы.");
    }
    let mut evidence = PointEvidence::default();
    for (i, &k) in MEASURED_COORDS.iter().enumerate() {
        let demand = muscle[k].abs();
        if let Some((action, limit)) = weak.estimate_for_torque(k, muscle[k]) {
            if demand > limit.nm {
                let test = CapacityTest {
                    demand_nm: demand,
                    limit_nm: limit.nm,
                    action,
                };
                if evidence
                    .overload
                    .is_none_or(|prev| demand - limit.nm > prev.demand_nm - prev.limit_nm)
                {
                    evidence.overload = Some(test);
                }
            }
            evidence.intensities[i] = if demand >= limit.nm && demand > 0.0 {
                1.0
            } else if limit.nm > 0.0 {
                demand / limit.nm
            } else {
                0.0
            };
            evidence.strength_missing |= limit.extrapolated && demand > RESIDUAL_TOL_NM;
        } else {
            evidence.strength_missing |= demand > RESIDUAL_TOL_NM;
        }
    }
    evidence.strength_missing |=
        (0..NC).any(|k| !MEASURED_COORDS.contains(&k) && muscle[k].abs() > RESIDUAL_TOL_NM);

    let mut result = LoadPoint {
        pose: s.pose,
        required_nm: required,
        muscle_nm: muscle,
        achieved_muscle_nm: [0.0; NC],
        muscles: Vec::new(),
        max_activation: 0.0,
        residual_nm: 0.0,
        certified_deficit_nm: 0.0,
        variants_in_equilibrium: 0,
        reactions: None,
        chaffin_compression_n: chaffin(s, input, inertia, &muscle),
        selected_strength: selected,
        weak_strength: weak,
        raw_model_to_weak_strength: calibration.overstrength.clone(),
    };
    if !result.chaffin_compression_n.is_finite() {
        return Err("Неконечная оценка сжатия по Chaffin.");
    }
    let variants = muscle_variants();
    for (i, variant) in variants.into_iter().enumerate() {
        let loads = solve_loads(s, input, &calibration, variant, &muscle, inertia);
        let reactions = ReactionPeaks {
            shoulder_n: loads.shoulder_n,
            elbow_n: loads.elbow_n,
            spine_compression_n: loads.spine_compression_n,
            spine_shear_n: loads.spine_shear_n,
        };
        if !all_finite(
            reactions
                .values()
                .into_iter()
                .chain([
                    loads.deficit_nm,
                    loads.certified_deficit_nm,
                    loads.max_activation,
                ])
                .chain(loads.achieved_torque)
                .chain(loads.activation.iter().copied())
                .chain(loads.muscle_tension_n.iter().copied()),
        ) {
            return Err("Неконечный результат мышечной оптимизации или реакции сустава.");
        }
        if loads.converged && loads.deficit_nm <= RESIDUAL_TOL_NM {
            result
                .reactions
                .get_or_insert_with(ReactionPeaks::default)
                .include(reactions);
            result.variants_in_equilibrium += 1;
            evidence.nominal_balanced |= i == 0;
        }
        if variant == variants[1] {
            evidence.weak_variant_torque = loads.achieved_torque;
        }
        if i == 0 {
            result.achieved_muscle_nm = loads.achieved_torque;
        }
        if i == 0 && with_muscles {
            result.muscles = s
                .muscles
                .iter()
                .zip(loads.activation.iter().zip(&loads.muscle_tension_n))
                .map(|(m, (&activation, &tension_n))| MuscleLoad {
                    actuator: m.actuator,
                    name: m.name.clone(),
                    activation,
                    tension_n,
                })
                .collect();
        }
        result.residual_nm = result.residual_nm.max(loads.deficit_nm);
        result.certified_deficit_nm = result.certified_deficit_nm.max(loads.certified_deficit_nm);
        result.max_activation = result.max_activation.max(loads.max_activation);
    }
    Ok(PointResult {
        point: result,
        evidence,
    })
}

fn chaffin(s: &GeometrySample, input: &LoadInput, inertia: bool, muscle: &[f64; NC]) -> f64 {
    let v = Vector3::from;
    let spine = &s.links[LINK_SPINE];
    let axial_gravity = v(spine.body_gravity) * input.scale.mass;
    let axial_inertia = v(spine.body_inertia) * (input.scale.mass * input.scale.length);
    let h = &s.hands[0];
    let payload_gravity = -v(s.gravity) * input.payload_kg;
    let payload_inertia =
        (v(h.accel) * input.scale.length + v(h.dir_accel) * input.lever_m) * input.payload_kg;
    let external = axial_gravity
        + payload_gravity
        + if inertia {
            axial_inertia + payload_inertia
        } else {
            Vector3::zeros()
        };
    let axial = external.dot(&v(s.spine_axis));
    let bending = muscle[C_L5_FE].hypot(muscle[C_L5_LB]);
    axial.max(0.0) + bending / sim_mujoco::loads::ERECTOR_LEVER_M
}

fn capacity_check(result: &PointResult, kind: HumanCheck) -> HumanFinding {
    let p = &result.point;
    if let Some(o) = result.evidence.overload {
        return finding(
            kind,
            Outcome::Fail,
            o.demand_nm,
            o.limit_nm,
            "Н·м",
            format!(
                "Не хватает силы слабого человека: {}. Требуется {:.2}, оценка по измерениям {:.2} Н·м.",
                o.action.label(),
                o.demand_nm,
                o.limit_nm
            ),
        );
    }
    if p.certified_deficit_nm > RESIDUAL_TOL_NM {
        return finding(
            kind,
            Outcome::Fail,
            p.certified_deficit_nm,
            RESIDUAL_TOL_NM,
            "Н·м",
            "Для одного из вариантов мышц в диапазоне ±20% доказана несовместимость моментов с пределами активации.",
        );
    }
    let unknown =
        result.evidence.strength_missing || p.variants_in_equilibrium != muscle_variants().len();
    finding(
        kind,
        if unknown {
            Outcome::NotAnalyzed
        } else {
            Outcome::Pass
        },
        p.max_activation,
        1.0,
        "доля",
        if unknown {
            "Не по всем осям и позам есть измерения силы либо расчёт распределения не сошёлся."
        } else {
            "Равновесие найдено при заданных моментах и измерительных оценках слабого человека."
        },
    )
}

fn add_reaction_checks(
    report: &mut HumanReport,
    case: &HumanLoadCase,
    peaks: Option<ReactionPeaks>,
    chaffin_peak: f64,
    complete: bool,
    gravity: f64,
) {
    let reference = SHOULDER_IN_VIVO_MAX_BW * case.person.mass_kg * gravity;
    let r = peaks.unwrap_or_default();
    let coverage = if peaks.is_none() {
        "Суставная реакция при найденном равновесии недоступна."
    } else if complete {
        "Позы и нагрузка подтверждены в рамках модели."
    } else {
        "Показаны только рассчитанные кадры и варианты, где найдено равновесие."
    };
    report.findings.extend([
        finding(HumanCheck::ShoulderLoad, Outcome::NotAnalyzed, r.shoulder_n, reference, "Н",
            format!("Bergmann и OrthoLoad: максимум нагрузки импланта {:.0}% веса тела. Порог травмы гленогумерального сустава отсутствует. {coverage}", SHOULDER_IN_VIVO_MAX_BW * 100.0)),
        finding(HumanCheck::ElbowLoad, Outcome::NotAnalyzed, r.elbow_n, ELBOW_OCCUPATIONAL_MAX_N, "Н",
            format!("{ELBOW_OCCUPATIONAL_MAX_N:.0} Н у Chadwick & Nicol — расчётная сила плечелоктевого сочленения при производственных задачах. В модели возвращается суммарная реакция предплечья. Порог травмы отсутствует. {coverage}")),
    ]);
    let compression = r.spine_compression_n.max(chaffin_peak);
    let compression_outcome = if compression >= NIOSH_ACTION_LIMIT_N {
        Outcome::Fail
    } else if complete {
        Outcome::Pass
    } else {
        Outcome::NotAnalyzed
    };
    report.findings.push(finding(HumanCheck::SpineCompression, compression_outcome,
        compression, NIOSH_ACTION_LIMIT_N, "Н",
        format!("L5/S1: мышцы {:.0} Н, схема Chaffin {:.0} Н. NIOSH: предел действия {:.0} Н, максимальный предел {:.0} Н. {}",
            r.spine_compression_n, chaffin_peak, NIOSH_ACTION_LIMIT_N, NIOSH_MAX_PERMISSIBLE_N,
            if compression >= NIOSH_MAX_PERMISSIBLE_N { "Превышен максимальный предел." }
            else if !complete { "Доступна частичная оценка. Покрытие поз, равновесие мышц или силы креплений требуют уточнения." }
            else { "Критерий нагрузки для ручного подъёма." })));
    let cycles = case.cycles_per_day.map(f64::from);
    let shear_limit = if cycles.is_some_and(|n| n < SHEAR_OCCASIONAL_MAX_PER_DAY) {
        SHEAR_OCCASIONAL_N
    } else {
        SHEAR_REPETITIVE_N
    };
    let shear_outcome = if peaks.is_some() && r.spine_shear_n >= shear_limit {
        Outcome::Fail
    } else if complete && cycles.is_some_and(|n| n <= SHEAR_REPETITIVE_MAX_PER_DAY) {
        Outcome::Pass
    } else {
        Outcome::NotAnalyzed
    };
    report.findings.push(finding(HumanCheck::SpineShear, shear_outcome, r.spine_shear_n, shear_limit, "Н",
        "Gallagher и Marras: 1000 Н при редкой нагрузке, 700 Н для 100–1000 циклов в день. При неизвестной частоте и выше исследованного диапазона 700 Н используется только для выявления опасности."));
}

fn add_fatigue(
    report: &mut HumanReport,
    case: &HumanLoadCase,
    held: &PointResult,
) -> Result<(), &'static str> {
    if case.work_time_s == 0.0 {
        report.findings.push(unavailable(HumanCheck::Fatigue, "Время непрерывного удержания не задано. Усталость при движении и восстановление между циклами требуют профиля нагрузки."));
        return Ok(());
    }
    let mut most_severe: Option<HumanFinding> = None;
    for (i, joint) in FatigueJoint::ALL.into_iter().enumerate() {
        let intensity = held.evidence.intensities[i];
        if intensity == 0.0 {
            continue;
        }
        let e = fatigue::endurance(joint, intensity);
        if !e.limit_s.is_finite() {
            return Err("Неконечный предел времени удержания.");
        }
        let exceeded = case.work_time_s >= e.limit_s;
        let status = if exceeded {
            Outcome::Fail
        } else {
            Outcome::NotAnalyzed
        };
        let result = finding(
            HumanCheck::Fatigue,
            status,
            case.work_time_s,
            e.limit_s,
            "с",
            format!(
                "{}: удержание при {:.1}% MVC. 3CC: {}, Frey-Law & Avin: {:.1} с, Rohmert: {}. Это время до истощения у исследованных групп. Индивидуальная выносливость и утомление до начала удержания не подтверждены. {}",
                joint.label(),
                intensity * 100.0,
                seconds(e.time_3cc_s),
                e.time_power_s,
                seconds(e.time_rohmert_s),
                if e.extrapolated {
                    "Нагрузка ниже диапазона подбора параметров 3CC."
                } else {
                    ""
                }
            ),
        );
        if most_severe
            .as_ref()
            .is_none_or(|other| result.limit < other.limit)
        {
            most_severe = Some(result);
        }
        report.endurance.push(e);
    }
    report.findings.push(most_severe.unwrap_or_else(|| {
        unavailable(
            HumanCheck::Fatigue,
            "Для загруженных направлений нет данных о выносливости.",
        )
    }));
    Ok(())
}

fn seconds(value: Option<f64>) -> String {
    value.map_or_else(|| "время не определено".into(), |v| format!("{v:.1} с"))
}

fn merge_reactions(dst: &mut Option<ReactionPeaks>, source: Option<ReactionPeaks>) {
    if let Some(r) = source {
        dst.get_or_insert_with(ReactionPeaks::default).include(r);
    }
}

fn pose_in_range(rig: &BodyRig, pose: &BodyPose) -> bool {
    let q = pose.coords();
    let [min_tilt, max_tilt] = rig.pelvis_range();
    (min_tilt..=max_tilt).contains(&pose.pelvis_tilt())
        && (0..NC).all(|k| {
            let [lo, hi] = rig.coord_range(k);
            (lo..=hi).contains(&q[k])
        })
}

// Минимальный рывок задаётся в пространстве суставов. В начале и в конце
// берётся точный предел разностей (нулевые скорость и ускорение).
fn lift_sample(
    rig: &mut BodyRig,
    start: &BodyPose,
    end: &BodyPose,
    u: f64,
    duration: f64,
) -> GeometrySample {
    let pose = pose_at(start, end, u, 1.0);
    if u == 0.0 || u == 1.0 {
        return rig.sample_static(&pose, false);
    }
    let h = MOTION_FD_FRACTION;
    rig.sample_moving(
        &pose_at(start, end, u - h, 1.0),
        &pose,
        &pose_at(start, end, u + h, 1.0),
        h * duration,
        false,
    )
}

fn lift_report(
    rig: &mut BodyRig,
    case: &HumanLoadCase,
    input: &LoadInput,
    lift: LiftInput,
    duration: f64,
) -> Result<(LiftReport, HumanFinding, f64), &'static str> {
    let mut result = LiftReport {
        duration_s: duration,
        peak_required_nm: [0.0; NC],
        peak_static_nm: [0.0; NC],
        max_activation: 0.0,
        max_static_activation: 0.0,
        peak_reactions: None,
        samples: 0,
    };
    let mut worst: Option<HumanFinding> = None;
    let mut chaffin_peak: f64 = 0.0;
    let accel_extreme = 3.0_f64.sqrt() / 6.0;
    let times = (0..=LIFT_SEGMENTS)
        .map(|i| i as f64 / LIFT_SEGMENTS as f64)
        .chain([0.5 - accel_extreme, 0.5 + accel_extreme]);
    for u in times {
        let s = lift_sample(rig, &lift.start_pose, &case.pose, u, duration);
        let moving = point(&s, input, case.person, true, false)?;
        // Убираем инерцию и силу–скорость, сохраняя длины и геометрию.
        let at_rest = point(&s, input, case.person, false, false)?;
        let check = capacity_check(&moving, HumanCheck::DynamicLift);
        let p = &moving.point;
        if worst.as_ref().is_none_or(|w| {
            severity(&check) > severity(w)
                || (check.outcome == w.outcome
                    && check.unit == w.unit
                    && check.value - check.limit > w.value - w.limit)
        }) {
            worst = Some(check);
        }
        for k in 0..NC {
            result.peak_required_nm[k] = result.peak_required_nm[k].max(p.required_nm[k].abs());
            result.peak_static_nm[k] =
                result.peak_static_nm[k].max(at_rest.point.required_nm[k].abs());
        }
        chaffin_peak = chaffin_peak
            .max(p.chaffin_compression_n)
            .max(at_rest.point.chaffin_compression_n);
        merge_reactions(&mut result.peak_reactions, p.reactions);
        merge_reactions(&mut result.peak_reactions, at_rest.point.reactions);
        result.max_activation = result.max_activation.max(p.max_activation);
        result.max_static_activation = result
            .max_static_activation
            .max(at_rest.point.max_activation);
        result.samples += 1;
    }
    let check = worst.ok_or("Нет отсчётов подъёма.")?;
    let detail = format!(
        "{} Пики по {} отсчётам, движение / статика: активация {:.1}% / {:.1}%, момент локтя {:.2} / {:.2} Н·м.",
        check.detail,
        result.samples,
        result.max_activation * 100.0,
        result.max_static_activation * 100.0,
        result.peak_required_nm[C_ELBOW],
        result.peak_static_nm[C_ELBOW]
    );
    // Дискретная траектория годится для выявления перегрузок; межкадровые пики не ограничены.
    let outcome = if check.outcome == Outcome::Fail {
        Outcome::Fail
    } else {
        Outcome::NotAnalyzed
    };
    Ok((
        result,
        finding(
            check.kind,
            outcome,
            check.value,
            check.limit,
            check.unit,
            detail,
        ),
        chaffin_peak,
    ))
}

fn severity(f: &HumanFinding) -> u8 {
    match f.outcome {
        Outcome::Pass => 0,
        Outcome::NotAnalyzed => 1,
        Outcome::Fail => 2,
    }
}

fn loss_report(
    rig: &mut BodyRig,
    case: &HumanLoadCase,
    input: LoadInput,
    duration: f64,
    loss_time_s: f64,
) -> Result<(DriveFailureReport, HumanFinding, f64), &'static str> {
    let delay_s = failure::worst_reaction_delay();
    let (start, rates, initial) = if let Some(lift) = case.lift {
        let s = lift_sample(rig, &lift.start_pose, &case.pose, 0.5, duration);
        let rate = min_jerk(0.5).1 / duration;
        let a = lift.start_pose.coords();
        let b = case.pose.coords();
        let rates = FAILURE_COORDS.map(|k| (b[k] - a[k]) * rate);
        let result = point(&s, &input, case.person, true, false)?;
        (s.pose, rates, result)
    } else {
        let s = rig.sample_static(&case.pose, false);
        let result = point(&s, &input, case.person, false, false)?;
        (s.pose, [0.0; 3], result)
    };
    let hold_input = LoadInput {
        assist: [0.0; NC],
        ..input
    };
    let s = rig.sample_static(&start, false);
    let hold = point(&s, &hold_input, case.person, false, true)?;
    let mut check = capacity_check(&hold, HumanCheck::DriveFailure);
    check.detail = format!("После полной потери помощи. {}", check.detail);
    let mut compression = hold.point.chaffin_compression_n;
    let mut report = DriveFailureReport {
        reaction_delay_s: delay_s,
        unassisted_hold: hold.point,
        motion: None,
        peak_hand_down_m_s: 0.0,
        post_failure_reactions: None,
        braking_reactions: None,
        braking_samples: 0,
        balanced_braking_samples: 0,
    };
    if loss_time_s > MAX_SIMULATED_LOSS_S {
        if check.outcome != Outcome::Fail {
            check = unavailable(
                HumanCheck::DriveFailure,
                "Спад помощи выходит за численный бюджет интегратора. Посчитано только удержание при нулевой помощи.",
            );
        }
        return Ok((report, check, compression));
    }
    // Оси модели отказа: подъём плеча, сгибание локтя и разгибание поясницы.
    let simulated = FailureInput {
        pose: start,
        rates,
        assist: FAILURE_COORDS.map(|k| input.assist[k]),
        muscle: FAILURE_COORDS.map(|k| initial.evidence.weak_variant_torque[k]),
        payload_kg: input.payload_kg,
        lever_m: input.lever_m,
        scale: input.scale,
        two_handed: false,
        loss_time_s,
        delay_s,
        rise_tau_s: TORQUE_RISE_TAU_S,
    };
    let MuscleVariant { f0, moment_arm } = muscle_variants()[1];
    let cap = |pose: &BodyPose, p: usize, need: f64| {
        weak_strength(pose, case.person)
            .for_torque(FAILURE_COORDS[p], need)
            .map_or(0.0, |(_, value)| value * f0 * moment_arm)
    };
    let (coarse, _) = failure::simulate(rig, &simulated, cap, false)?;
    let (motion, trace) = failure::simulate(rig, &simulated, cap, true)?;
    let stable = coarse.hit_limit == motion.hit_limit
        && coarse.stopped == motion.stopped
        && coarse.fell_before_reaction == motion.fell_before_reaction
        && (coarse.peak_speed_deg_s - motion.peak_speed_deg_s).abs()
            <= FAILURE_NUMERICAL_TOL * motion.peak_speed_deg_s.max(1.0)
        && motion.energy_error <= FAILURE_NUMERICAL_TOL;
    for frame in trace {
        // Локальная аппроксимация движения q(t±h) для силы–скорости и реакций при торможении.
        let h = MOTION_FD_FRACTION;
        let offset = |sign: f64| {
            std::array::from_fn(|k| sign * h * frame.rates[k] + 0.5 * h * h * frame.acceleration[k])
        };
        let s = rig.sample_moving(
            &failure::with_angles(&frame.pose, &offset(-1.0)),
            &frame.pose,
            &failure::with_angles(&frame.pose, &offset(1.0)),
            h,
            false,
        );
        let fade = LoadInput {
            assist: input.assist.map(|x| x * frame.assist_fraction),
            ..input
        };
        let step = point(&s, &fade, case.person, true, false)?;
        let hand_velocity: Vector3<f64> = FAILURE_COORDS
            .into_iter()
            .zip(frame.rates)
            .map(|(k, rate)| Vector3::from(s.hands[0].dpoint[k]) * (rate * input.scale.length))
            .sum();
        let down_speed = Vector3::from(s.gravity).normalize().dot(&hand_velocity);
        if !down_speed.is_finite() {
            return Err("Неконечная скорость падения кисти.");
        }
        report.peak_hand_down_m_s = report.peak_hand_down_m_s.max(down_speed);
        merge_reactions(&mut report.post_failure_reactions, step.point.reactions);
        if frame.braking {
            report.braking_samples += 1;
            report.balanced_braking_samples += usize::from(step.point.reactions.is_some());
            merge_reactions(&mut report.braking_reactions, step.point.reactions);
        }
        compression = compression.max(step.point.chaffin_compression_n);
    }
    if check.outcome != Outcome::Fail && motion.hit_limit {
        check = finding(
            HumanCheck::DriveFailure,
            Outcome::Fail,
            motion.max_drop_deg,
            0.0,
            "°",
            "После потери помощи сустав дошёл до границы диапазона MS-Human-700. Сила удара об упор не рассчитана.",
        );
    } else if check.outcome != Outcome::Fail
        && motion.stopped
        && pose_in_range(rig, &motion.end_pose)
    {
        let stopped = point(
            &rig.sample_static(&motion.end_pose, false),
            &hold_input,
            case.person,
            false,
            false,
        )?;
        let end_check = capacity_check(&stopped, HumanCheck::DriveFailure);
        if end_check.outcome == Outcome::Fail {
            check = end_check;
        }
    }
    if check.outcome != Outcome::Fail && motion.fell_before_reaction {
        check = finding(
            HumanCheck::DriveFailure,
            Outcome::Fail,
            motion.peak_speed_deg_s,
            0.0,
            "°/с",
            "Падение началось до нарастания мышечного момента при принятой задержке реакции.",
        );
    } else if check.outcome != Outcome::Fail && !motion.stopped && stable {
        check = finding(
            HumanCheck::DriveFailure,
            Outcome::Fail,
            motion.stop_time_s,
            0.0,
            "с",
            "Модель захвата не остановила движение за расчётное окно.",
        );
    }
    if check.outcome != Outcome::Fail {
        check = unavailable(
            HumanCheck::DriveFailure,
            "Оценка захвата зависит от индивидуальной реакции, геометрии креплений и допустимой траектории после отказа.",
        );
    }
    check.detail.push_str(&format!(
        " Оценка задержки {:.3} с, скорость падения руки до {:.1} °/с, кисти вниз до {:.2} м/с, отклонение до {:.1}°, сбалансированные отсчёты торможения {}/{}. {}",
        delay_s, motion.peak_speed_deg_s, report.peak_hand_down_m_s, motion.max_drop_deg, report.balanced_braking_samples, report.braking_samples,
        if stable { "Повтор с меньшим шагом согласуется с основным расчётом." }
        else { "Повтор с меньшим шагом или баланс энергии выявил численную погрешность." }));
    if !initial.evidence.nominal_balanced {
        check.detail.push_str(
            " Начальная поза движения не сбалансирована мышцами. Аварийная траектория условна.",
        );
    }
    report.motion = Some(motion);
    Ok((report, check, compression))
}

/// Подробный расчёт. Неполные исходные данные остаются видны в вердиктах и отчёте.
pub fn analyze_human(case: &HumanLoadCase) -> HumanReport {
    if !validate(case) {
        return empty_report(
            Outcome::Fail,
            "Вход содержит неконечное число или значение за допустимым диапазоном формы.",
        );
    }
    let mut rig = match BodyRig::load() {
        Ok(rig) => rig,
        Err(e) => {
            return empty_report(
                Outcome::NotAnalyzed,
                &format!("Не удалось загрузить MS-Human-700. {e}"),
            );
        }
    };
    if !pose_in_range(&rig, &case.pose)
        || case
            .lift
            .is_some_and(|l| !pose_in_range(&rig, &l.start_pose))
    {
        return empty_report(
            Outcome::Fail,
            "Начальная или конечная поза выходит за диапазон суставов MS-Human-700.",
        );
    }
    run(&mut rig, case).unwrap_or_else(|error| empty_report(Outcome::Fail, error))
}

fn run(rig: &mut BodyRig, case: &HumanLoadCase) -> Result<HumanReport, &'static str> {
    let scale = BodyScale {
        length: case.person.stature_m / MODEL_STATURE_M,
        mass: case.person.mass_kg / rig.body_mass(),
    };
    if !all_finite([scale.length, scale.mass]) || scale.length <= 0.0 || scale.mass <= 0.0 {
        return Err("Неконечный масштаб тела.");
    }
    let input = LoadInput {
        payload_kg: case.payload_kg,
        lever_m: case.lever_m,
        assist: case.assist_nm,
        scale,
    };
    let g = Vector3::from(rig.sim().gravity()).norm();
    let held = point(
        &rig.sample_static(&case.pose, false),
        &input,
        case.person,
        false,
        true,
    )?;
    let lift_duration;
    let mut report = empty_report(Outcome::NotAnalyzed, "");
    report.findings.clear();
    report
        .findings
        .push(capacity_check(&held, HumanCheck::MuscleCapacity));
    let mut peaks = held.point.reactions;
    let mut max_chaffin = held.point.chaffin_compression_n;
    let mut complete = case.assist_nm.iter().all(|&x| x == 0.0)
        && held.point.variants_in_equilibrium == muscle_variants().len();
    if let Some(l) = case.lift {
        let excursion =
            crate::lift::peak_joint_speed(&l.start_pose, &case.pose, 1.0) / MIN_JERK_PEAK_SPEED;
        let duration = MIN_JERK_PEAK_SPEED * excursion / l.peak_speed_rad_s;
        let dt = duration * MOTION_FD_FRACTION;
        if !duration.is_finite() || !dt.powi(2).is_finite() || dt.powi(2) == 0.0 {
            return Err("Неконечная длительность или шаг подъёма.");
        }
        let (lift, check, compression) = lift_report(rig, case, &input, l, duration)?;
        merge_reactions(&mut peaks, lift.peak_reactions);
        max_chaffin = max_chaffin.max(compression);
        // Межкадровые пики и нагрузки от устройства остаются открыты.
        complete = false;
        lift_duration = duration;
        report.findings.push(check);
        report.lift = Some(lift);
    } else {
        lift_duration = 0.0;
        report.findings.push(unavailable(
            HumanCheck::DynamicLift,
            "Для динамики нужны начальная поза и пиковая скорость подъёма.",
        ));
    }
    if let Some(loss_time) = case.assist_loss_time_s
        && case.assist_nm.iter().any(|&x| x != 0.0)
    {
        let (loss, check, compression) = loss_report(rig, case, input, lift_duration, loss_time)?;
        merge_reactions(&mut peaks, loss.post_failure_reactions);
        merge_reactions(&mut peaks, loss.unassisted_hold.reactions);
        max_chaffin = max_chaffin.max(compression);
        report.findings.push(check);
        report.failure = Some(loss);
    } else {
        report.findings.push(unavailable(
            HumanCheck::DriveFailure,
            "Для расчёта потери помощи нужны момент привода и время его спада.",
        ));
    }
    add_reaction_checks(&mut report, case, peaks, max_chaffin, complete, g);
    add_fatigue(&mut report, case, &held)?;
    report.holding = Some(held.point);
    Ok(report)
}

/// Вердикты для панели симулятора. Отсутствие данных кодируется через NotAnalyzed.
pub fn assess_human(case: &HumanLoadCase) -> Vec<HumanFinding> {
    analyze_human(case).findings
}
