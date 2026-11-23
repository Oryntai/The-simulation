//! Перенос нагрузок на модель человека с отдельной проверкой снятия помощи.

use sim_device::{Analysis, BodyRegion, Device, DeviceSettings};
use sim_human::{BodyPose, C_ELBOW, C_ELEV, HumanCheck, HumanLoadCase, LiftInput, NC, Person, Sex};

use super::{DeviceReport, Group, Outcome, specified};

fn pose(s: &DeviceSettings) -> Option<BodyPose> {
    Some(BodyPose {
        shoulder_plane: s.shoulder_plane_deg?.to_radians(),
        shoulder_elevation: s.shoulder_elevation_deg?.to_radians(),
        shoulder_rotation: s.shoulder_rotation_deg?.to_radians(),
        elbow_flexion: s.elbow_deg?.to_radians(),
        forearm_rotation: s.forearm_deg?.to_radians(),
        trunk_inclination: s.trunk_deg?.to_radians(),
        lumbar_flexion: s.lumbar_deg?.to_radians(),
        lumbar_lateral_bend: s.lateral_deg?.to_radians(),
        lumbar_rotation: s.lumbar_rotation_deg?.to_radians(),
    })
}

fn person(s: &DeviceSettings) -> Option<Person> {
    Some(Person {
        sex: match s.sex.as_deref()? {
            "female" => Sex::Female,
            "male" => Sex::Male,
            _ => return None,
        },
        strength_percentile: s.strength_pct?,
        age_years: s.age_years?,
        stature_m: s.height_mm? * sim_materials::catalog::MM,
        mass_kg: s.mass_kg?,
    })
}

fn arm(r: BodyRegion) -> bool {
    matches!(
        r,
        BodyRegion::UpperArm | BodyRegion::Forearm | BodyRegion::Hand
    )
}

pub(super) fn evaluate(d: &Device, a: &Analysis, report: &mut DeviceReport) {
    use BodyRegion::None as NoRegion;
    let regions: Vec<_> = BodyRegion::ALL
        .into_iter()
        .filter(|&r| r != NoRegion && (r == d.mount || d.links.iter().any(|l| l.near_body == r)))
        .collect();
    for r in regions.iter().copied().filter(|&r| !arm(r)) {
        report.finding(
            Group::Human,
            "unsupported_region",
            Option::None,
            r.title(),
            Outcome::NotAnalyzed,
            "Модель руки не рассчитывает нагрузку на эту область, её крепление и отказ её привода",
        );
    }
    if regions.is_empty() {
        report.finding(Group::Human, "not_worn", Option::None, "Нагрузка на человека", 
            if specified(d.settings.contact_source.as_deref()) { Outcome::Pass } else { Outcome::NotAnalyzed },
            "Крепление на теле и зона доступа тела в модели отсутствуют. Нужна проверка границ рабочей зоны");
    }
    if !arm(d.mount) {
        if regions.iter().copied().any(arm) {
            report.finding(
                Group::Human,
                "arm_mount",
                Option::None,
                "Нагрузка на руку",
                Outcome::NotAnalyzed,
                "Нет подтверждённой схемы крепления на одной руке для модели правой руки",
            );
        }
        return;
    }
    let missing = |r: &mut DeviceReport, detail| {
        r.finding(
            Group::Human,
            "arm_case",
            Option::None,
            "Работа правой руки",
            Outcome::NotAnalyzed,
            detail,
        );
    };
    let s = &d.settings;
    let mut assist = [0.0; NC];
    let mut has_assistance = false;
    let mut max_speed: f64 = 0.0;
    for j in &a.joints {
        let l = &d.links[j.link];
        if l.joint.actuator.is_empty() {
            continue;
        }
        has_assistance = true;
        max_speed = max_speed.max(j.speed_rad_s);
        let index = match l.settings.arm_action.as_deref() {
            Some("elbow") => C_ELBOW,
            Some("shoulder") => C_ELEV,
            _ => {
                missing(report, "Укажите, какой сустав поддерживает каждый привод");
                return;
            }
        };
        let Some(torque) = l.settings.assist_nm else {
            missing(report, "Нет подтверждённого момента помощи под нагрузкой");
            return;
        };
        if torque < 0.0 || torque > l.joint.torque_limit_nm && l.joint.torque_limit_nm > 0.0 {
            report.finding(
                Group::Human,
                "assist_torque",
                Some(j.link),
                &l.name,
                Outcome::Fail,
                "Момент помощи отрицателен или превышает ограничение шарнира",
            );
            return;
        }
        assist[index] += torque;
    }
    if !specified(s.arm_equivalence_source.as_deref()) || !specified(s.inventory_source.as_deref())
    {
        missing(
            report,
            "Нужно стендовое или расчётное приведение массы устройства и реакций креплений к модели правой руки",
        );
    }
    let (Some(person), Some(pose), Some(lever), Some(hold)) =
        (person(s), pose(s), s.arm_lever_mm, s.hold_s)
    else {
        missing(
            report,
            "Заполните данные человека, рабочую позу, плечо груза и время удержания",
        );
        return;
    };
    let mut static_only = false;
    let lift = if let (Some(shoulder), Some(elbow)) = (s.start_shoulder_deg, s.start_elbow_deg) {
        let start_pose = BodyPose {
            shoulder_elevation: shoulder.to_radians(),
            elbow_flexion: elbow.to_radians(),
            ..pose
        };
        if start_pose.shoulder_elevation == pose.shoulder_elevation
            && start_pose.elbow_flexion == pose.elbow_flexion
        {
            static_only = !has_assistance;
            Option::None
        } else {
            Some(LiftInput {
                start_pose,
                peak_speed_rad_s: max_speed,
            })
        }
    } else {
        missing(report, "Нет начальной позы для проверки подъёма");
        Option::None
    };
    if has_assistance && s.loss_s.is_none() {
        missing(report, "Укажите время снятия помощи для расчёта отказа");
    }
    let case = HumanLoadCase {
        // Оценка в опасную сторону: вся масса переносится в точечный груз у кисти.
        payload_kg: a.total_mass_kg,
        lever_m: lever * sim_materials::catalog::MM,
        pose,
        assist_nm: assist,
        person,
        work_time_s: hold,
        lift,
        assist_loss_time_s: s.loss_s,
        cycles_per_day: s.cycles_day,
    };
    let human = sim_human::analyze_human(&case);
    for f in &human.findings {
        let title = match f.kind {
            HumanCheck::MuscleCapacity => "Сила мышц",
            HumanCheck::ShoulderLoad => "Нагрузка плеча",
            HumanCheck::ElbowLoad => "Нагрузка локтя",
            HumanCheck::SpineCompression => "Сжатие поясницы",
            HumanCheck::SpineShear => "Сдвиг в пояснице",
            HumanCheck::DynamicLift => "Подъём",
            HumanCheck::DriveFailure => "Отказ привода",
            HumanCheck::Fatigue => "Мышечная усталость",
        };
        if f.kind == HumanCheck::DynamicLift
            && static_only
            && f.outcome == sim_human::Outcome::NotAnalyzed
        {
            report.finding(
                Group::Human,
                "static_pose",
                None,
                title,
                Outcome::Pass,
                "Подъёма приводом нет, рабочая поза фиксирована. Динамика подъёма неприменима",
            );
        } else {
            r_finding(report, &case, f, title);
        }
    }
    report.human = Some(human);
    if !has_assistance {
        report.finding(
            Group::Human,
            "no_assistance",
            Option::None,
            "Отказ помощи",
            Outcome::Pass,
            "В устройстве нет приводов, передающих вспомогательный момент человеку",
        );
    }
}

/// Обратимый привод на руке не держит нагрузку без питания, но рука при этом висит на человеке.
/// Опасность решает расчёт отказа: не успевает удержать — провал, не доказано — не «годен».
pub(super) fn reconcile_unpowered(d: &Device, report: &mut DeviceReport) {
    if !arm(d.mount) {
        return;
    }
    let failures: Vec<Outcome> = report
        .findings
        .iter()
        .filter(|f| f.code == "drive_failure")
        .map(|f| f.outcome)
        .collect();
    if failures.contains(&Outcome::Fail) {
        return;
    }
    let proven = !failures.is_empty() && failures.iter().all(|&o| o == Outcome::Pass);
    for f in &mut report.findings {
        let on_arm = f
            .link
            .is_some_and(|i| d.links[i].settings.arm_action.is_some());
        if f.code != "unpowered_hold" || f.outcome != Outcome::Fail || !on_arm {
            continue;
        }
        if proven {
            f.outcome = Outcome::Pass;
            f.detail = "Привод обратим и без питания не держит. Нагрузку принимает человек: расчёт «Отказ привода» показал, что он успевает её удержать".into();
        } else {
            f.code = "unpowered_person";
            f.outcome = Outcome::NotAnalyzed;
            f.detail = "Привод обратим и без питания не держит. Удержит ли человек, расчёт отказа не доказал: на себе не испытывать".into();
        }
    }
}

fn r_finding(r: &mut DeviceReport, case: &HumanLoadCase, f: &sim_human::HumanFinding, title: &str) {
    // Крейту переданы точно эти режимы. Подъём без изменения суставных углов и отказ
    // отсутствующей помощи неприменимы по модели, если исходные позы заданы.
    let code = if f.kind == HumanCheck::DriveFailure {
        "drive_failure"
    } else {
        "human_load"
    };
    if f.kind == HumanCheck::DriveFailure
        && case.assist_nm.iter().all(|&t| t == 0.0)
        && f.outcome == sim_human::Outcome::NotAnalyzed
    {
        return;
    }
    r.numeric(
        Group::Human,
        code,
        None,
        title,
        f.outcome.into(),
        f.value,
        f.limit,
        f.unit,
        &f.detail,
    );
}
