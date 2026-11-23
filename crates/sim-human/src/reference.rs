//! Сверки ручной механикой, мастер-уравнениями и измерениями людей.

use nalgebra::Vector3;
use sim_mujoco::loads::gravity_moment;
use sim_verify::{Check, Tolerance};

use crate::data::*;
use crate::fatigue::{FatigueJoint, endurance_time_3cc, endurance_time_power, endurance_time_rk4};
use crate::lift::{MIN_JERK_PEAK_SPEED, min_jerk, pose_at};
use crate::load::{
    BodyScale, Calibration, LoadInput, Strengths, calibrate, group_capacity, required_torque,
    solve_loads,
};
use crate::population::{Person, Sex, muscle_variants};
use crate::rig::*;

fn check(
    id: &'static str,
    title: &str,
    values: (f64, f64),
    unit: &'static str,
    methods: (&str, &str),
    tolerance: Tolerance,
    note: &str,
) -> Check {
    Check {
        id,
        title: title.into(),
        simulated: values.0,
        reference: values.1,
        unit,
        method: methods.0.into(),
        reference_method: methods.1.into(),
        tolerance,
        note: note.into(),
    }
}

/// Модель расходится с измерениями людей, и подгонять допуск нельзя. Эти числа показываются
/// отдельным списком. На них вердикт не опирается: см. защитные сверки `human_verdict_*`.
const KNOWN_DISCREPANCIES: [&str; 5] = [
    "human_endurance_elbow",
    "human_endurance_spine",
    "human_mvc_men",
    "human_mvc_women",
    "human_shoulder_invivo",
];

/// Сверки, которые обязаны сходиться.
pub fn reference_checks() -> Vec<Check> {
    let mut out: Vec<_> = measured_checks()
        .into_iter()
        .filter(|c| !KNOWN_DISCREPANCIES.contains(&c.id))
        .collect();
    out.extend(verdict_guards());
    out
}

/// Известные расхождения модели с опубликованными измерениями.
pub fn known_discrepancies() -> Vec<Check> {
    measured_checks()
        .into_iter()
        .filter(|c| KNOWN_DISCREPANCIES.contains(&c.id))
        .collect()
}

/// Вердикт обязан быть не оптимистичнее измерений, даже там, где сама модель с ними расходится.
fn verdict_guards() -> Vec<Check> {
    let mut out = Vec::new();
    const INTENSITY: f64 = 0.5;
    for (joint, id) in FatigueJoint::ALL.into_iter().zip([
        "human_verdict_endurance_shoulder",
        "human_verdict_endurance_elbow",
        "human_verdict_endurance_spine",
    ]) {
        let used = crate::fatigue::endurance(joint, INTENSITY).limit_s;
        let measured = endurance_time_power(INTENSITY, joint.endurance_fit());
        out.push(check(
            id,
            &format!("Вердикт не считает выносливость выше кривой: {}", joint.label()),
            ((used - measured).max(0.0), 0.0),
            "с",
            (
                "превышение времени, взятого в вердикт, над опубликованной кривой",
                "ноль: вердикт берёт наименьшую из оценок",
            ),
            Tolerance::Absolute(0.0),
            "Модель 3CC даёт больше кривой для локтя и поясницы, поэтому в вердикт идёт минимум.",
        ));
    }
    let Ok(mut rig) = BodyRig::load() else {
        return out;
    };
    let pose = BodyPose {
        elbow_flexion: 90_f64.to_radians(),
        ..BodyPose::default()
    };
    let sample = rig.sample_static(&pose, false);
    let scale = BodyScale {
        length: 1.0,
        mass: 1.0,
    };
    for (sex, percentile, id, title) in [
        (Sex::Female, 5.0, "human_verdict_strength_weak", "слабая женщина"),
        (Sex::Male, 50.0, "human_verdict_strength_median", "средний мужчина"),
    ] {
        let person = Person {
            sex,
            strength_percentile: percentile,
            age_years: 30.0,
            stature_m: MODEL_STATURE_M,
            mass_kg: 70.0,
        };
        let strengths = Strengths::at(&pose, &person);
        let calibration = calibrate(&sample, &strengths, &scale);
        let usable: f64 = sample
            .muscles
            .iter()
            .zip(&calibration.factors)
            .map(|(m, c)| {
                let active = if m.r[C_ELBOW] > 0.0 { m.active_iso } else { 0.0 };
                m.r[C_ELBOW] * c * (active + m.passive)
            })
            .sum();
        out.push(check(
            id,
            &format!("Сила в вердикте не выше измеренной: {title}"),
            ((usable - strengths.elbow_flexion.nm).max(0.0), 0.0),
            "Н·м",
            (
                "превышение силы сгибания локтя после приведения к измерениям, с пассивной тягой",
                "ноль: Frey-Law 2012 и Harbo 2012 для этого человека",
            ),
            Tolerance::Absolute(CALIBRATION_TOL_NM),
            "Исходная модель сильнее людей, поэтому силы мышц приводятся к измерениям.",
        ));
    }
    out
}

/// Численная точность приведения силы, Н·м.
const CALIBRATION_TOL_NM: f64 = 1e-6;

fn measured_checks() -> Vec<Check> {
    let mut out = Vec::new();
    // Точка сравнения взята посередине диапазона Frey-Law & Avin 2010; это не рабочий предел.
    const INTENSITY: f64 = 0.5;
    for (joint, id) in FatigueJoint::ALL.into_iter().zip([
        "human_3cc_shoulder",
        "human_3cc_elbow",
        "human_3cc_spine",
    ]) {
        let p = joint.params();
        let numeric = endurance_time_rk4(INTENSITY, p).unwrap_or(f64::NAN);
        let analytic = endurance_time_3cc(INTENSITY, p).unwrap_or(f64::NAN);
        out.push(check(
            id,
            &format!("Время удержания, 3CC: {}", joint.label()),
            (numeric, analytic),
            "с",
            (
                "Рунге–Кутта, рост активации",
                "решение линейных уравнений трёх камер до исчерпания резерва",
            ),
            // Допуск равен одному шагу регистрации пересечения в схеме РК4.
            Tolerance::Absolute(0.02),
            "Одинаковые начальные условия. Проверяется момент первого исчерпания резерва.",
        ));
    }
    for (joint, id) in FatigueJoint::ALL.into_iter().zip([
        "human_endurance_shoulder",
        "human_endurance_elbow",
        "human_endurance_spine",
    ]) {
        let p = joint.params();
        out.push(check(id, &format!("Время удержания при 50% MVC: {}", joint.label()),
            (endurance_time_3cc(INTENSITY, p).unwrap_or(f64::NAN), endurance_time_power(INTENSITY, joint.endurance_fit())), "с",
            ("параметры 3CC по суставу, решение уравнений", "Frey-Law & Avin 2010, опубликованная степенная кривая"),
            Tolerance::Absolute(p.rms_test_s), "Допуск равен опубликованной ошибке на проверочных уровнях 3CC. Кривая использовалась для подбора параметров, это сверка реализации."));
    }
    let Ok(mut rig) = BodyRig::load() else {
        out.push(check(
            "human_model",
            "Загрузка MS-Human-700",
            (f64::NAN, 1.0),
            "",
            ("загрузка MuJoCo", "успешная загрузка"),
            Tolerance::Absolute(0.0),
            "Без модели механические сверки недоступны.",
        ));
        return out;
    };
    // Поза динамометрии Frey-Law и контрольная масса Westerhoff.
    let pose = BodyPose {
        elbow_flexion: 90_f64.to_radians(),
        ..BodyPose::default()
    };
    let sample = rig.sample_static(&pose, false);
    let input = LoadInput {
        payload_kg: 2.0,
        lever_m: 0.0,
        assist: [0.0; NC],
        scale: BodyScale {
            length: 1.0,
            mass: 1.0,
        },
    };
    let tau = required_torque(&sample, &input, false);
    let sim = rig.sim();
    let v = Vector3::from;
    let j = sim
        .joints()
        .into_iter()
        .find(|joint| joint.name == "elbow_flexion_r")
        .expect("сустав проверен при загрузке");
    let elbow = sim.joint_anchor(j.id);
    let mask = sim.subtree_mask(j.body);
    let moment = gravity_moment(
        sim,
        elbow,
        &mask,
        Some((sample.hands[0].point, input.payload_kg)),
    );
    let manual_elbow = -v(moment).dot(&v(sim.joint_axis(j.id)));
    // Оценка абсолютной численной ошибки для конечной разности якобиана геометрии.
    let mechanics_tol = Tolerance::Absolute(1e-4);
    out.push(check(
        "human_elbow_gravity",
        "Статический момент локтя",
        (tau[C_ELBOW], manual_elbow),
        "Н·м",
        (
            "виртуальная работа",
            "сумма векторных моментов веса по сегментам и грузу",
        ),
        mechanics_tol,
        "Одна поза и одна точка груза в обеих формулах.",
    ));
    let lumbar = sim
        .joints()
        .into_iter()
        .find(|joint| joint.name == "L5_S1_FE")
        .expect("сустав проверен при загрузке");
    let moment = gravity_moment(
        sim,
        sim.joint_anchor(lumbar.id),
        &sim.subtree_mask(lumbar.body),
        Some((sample.hands[0].point, input.payload_kg)),
    );
    out.push(check(
        "human_lumbar_gravity",
        "Статический момент L5/S1",
        (tau[C_L5_FE], -v(moment).dot(&v(sim.joint_axis(lumbar.id)))),
        "Н·м",
        (
            "виртуальная работа",
            "сумма моментов веса верхней части тела и груза вокруг L5/S1",
        ),
        mechanics_tol,
        "При наклоне таза 0 мировые оси совпадают с осями model.opt.gravity.",
    ));
    let raw = group_capacity(&sample, C_ELBOW, 1.0, 1.0, false)
        + sample
            .muscles
            .iter()
            .map(|m| m.r[C_ELBOW] * m.passive)
            .sum::<f64>();
    let (_, (male, male_sd), (female, female_sd)) = FREY_LAW_ELBOW_FLEXION[3];
    let historical = MODEL_OVERSTRENGTH_ELBOW_90;
    let note = format!(
        "В проектных сверках двух моделей превышение мужского среднего {:.0}–{:.0}%. Здесь другая сборка модели и поза. Критерий вердикта использует силу по измерениям.",
        historical.0 * 100.0,
        historical.1 * 100.0
    );
    for (id, title, measured, tolerance) in [
        (
            "human_mvc_men",
            "Сгибание локтя 90°, мужчины",
            male,
            Tolerance::Absolute(male_sd),
        ),
        (
            "human_mvc_women",
            "Сгибание локтя 90°, женщины",
            female,
            Tolerance::Absolute(female_sd),
        ),
    ] {
        out.push(check(
            id,
            title,
            (raw, measured),
            "Н·м",
            (
                "исходная модель MS-Human-700 до снижения силы",
                "Frey-Law 2012, среднее при 90°",
            ),
            tolerance,
            &note,
        ));
    }
    // Тестовый манёвр: локоть через один радиан за одну секунду, числа задают испытание.
    let start = BodyPose {
        elbow_flexion: pose.elbow_flexion - 1.0,
        ..pose
    };
    let u = 0.5 - 3.0_f64.sqrt() / 6.0;
    let h = 1e-4; // Численный шаг центральной разности, с.
    let reference_pose = pose_at(&start, &pose, u, 1.0);
    let moving = rig.sample_moving(
        &pose_at(&start, &pose, u - h, 1.0),
        &reference_pose,
        &pose_at(&start, &pose, u + h, 1.0),
        h,
        false,
    );
    let unloaded = LoadInput {
        payload_kg: 0.0,
        ..input
    };
    let effect = |inertia| {
        required_torque(&moving, &input, inertia)[C_ELBOW]
            - required_torque(&moving, &unloaded, inertia)[C_ELBOW]
    };
    let sim = rig.sim();
    let radius = v(moving.hands[0].point) - v(sim.joint_anchor(j.id));
    let axis = v(sim.joint_axis(j.id));
    let payload_accel_torque =
        input.payload_kg * radius.cross(&axis).norm_squared() * min_jerk(u).2;
    out.push(check(
        "human_elbow_acceleration",
        "Инерционный момент точечного груза в локте",
        (effect(true) - effect(false), payload_accel_torque),
        "Н·м",
        (
            "обратная динамика с ускорением точки груза",
            "m·r⊥²·α для вращения вокруг одной оси",
        ),
        mechanics_tol,
        "Центростремительное ускорение не даёт момента вокруг оси локтя.",
    ));
    out.push(check(
        "human_lift_speed",
        "Скорость в середине подъёма",
        (min_jerk(0.5).1, MIN_JERK_PEAK_SPEED),
        "",
        (
            "производная полинома",
            "15/8, ручное дифференцирование Flash & Hogan",
        ),
        Tolerance::Absolute(0.0),
        "Время нормировано на длительность подъёма.",
    ));
    out.push(shoulder_check(&mut rig, input));
    out
}

fn shoulder_check(rig: &mut BodyRig, input: LoadInput) -> Check {
    // Westerhoff 2009: 2 кг до уровня головы. Ради сравнения берём близкую геометрию,
    // поскольку индивидуальных поз и скоростей в аннотации нет.
    let pose = BodyPose {
        shoulder_plane: std::f64::consts::FRAC_PI_2,
        shoulder_elevation: std::f64::consts::FRAC_PI_2,
        ..BodyPose::default()
    };
    let sample = rig.sample_static(&pose, false);
    let tau = required_torque(&sample, &input, false);
    let calibration = Calibration {
        factors: vec![1.0; sample.muscles.len()],
        overstrength: Vec::new(),
    };
    let loads = solve_loads(
        &sample,
        &input,
        &calibration,
        muscle_variants()[0],
        &tau,
        false,
    );
    let weight = rig.body_mass() * Vector3::from(sample.gravity).norm();
    let (avg, lo, _) = SHOULDER_LIFT_2KG_BW;
    let result = if loads.converged && loads.deficit_nm <= crate::load::RESIDUAL_TOL_NM {
        loads.shoulder_n / weight
    } else {
        f64::NAN
    };
    check(
        "human_shoulder_invivo",
        "Плечо, подъём 2 кг до уровня головы",
        (result, avg),
        "вес тела",
        (
            "MS-Human-700, статический кадр с грузом, исходная сила мышц",
            "Westerhoff 2009, средний пик контактной силы импланта",
        ),
        Tolerance::Absolute(avg - lo),
        "Допуск равен полуразмаху опубликованных измерений. Сопоставление приблизительное: неизвестны позы и скорости пациентов.",
    )
}
