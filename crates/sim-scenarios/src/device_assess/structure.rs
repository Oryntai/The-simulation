//! Силовые детали, зазоры и контакт с телом.

use sim_device::measure::Quantity;
use sim_device::{Analysis, BodyRegion, Device, JointKind, LinkLoad, LinkSpec, Section};
use sim_materials as mat;

use super::{DeviceReport, Group, Outcome, specified};

const MM: f64 = mat::catalog::MM;

fn section(l: &LinkSpec) -> mat::Section {
    // При произвольном наклоне нагрузка может действовать вокруг слабой оси.
    match l.section {
        Section::Rect { b_mm, h_mm } => mat::Section::Rect {
            b: b_mm.max(h_mm) * MM,
            h: b_mm.min(h_mm) * MM,
        },
        Section::Round { d_mm } => mat::Section::Round { d: d_mm * MM },
        Section::Tube { od_mm, id_mm } => mat::Section::Tube {
            od: od_mm * MM,
            wall: (od_mm - id_mm) * MM / 2.0,
        },
    }
}

/// Номинальное напряжение изгиба по слабой оси (M/W), Па, до концентратора.
pub fn nominal_bending_pa(link: &LinkSpec, moment_nm: f64) -> f64 {
    moment_nm.abs() / section(link).section_modulus()
}

fn printed(d: &Device, l: &LinkSpec) -> Option<mat::PrintSpec> {
    let s = &d.settings;
    let g = &l.settings;
    let p = l.print?;
    Some(mat::PrintSpec {
        process: mat::PrintProcess {
            nozzle: s.nozzle_mm? * MM,
            layer: p.layer_mm * MM,
            walls: p.walls,
            infill: p.infill_pct / 100.0,
            load_angle_deg: if p.across_layers { 90.0 } else { 0.0 },
        },
        geometry: mat::PrintGeometry {
            bbox: [g.bbox_x_mm? * MM, g.bbox_y_mm? * MM, g.bbox_z_mm? * MM],
            build_volume: [
                s.volume_x_mm? * MM,
                s.volume_y_mm? * MM,
                s.volume_z_mm? * MM,
            ],
            min_wall: g.wall_mm? * MM,
            max_overhang_deg: g.overhang_deg?,
            max_bridge: g.bridge_mm? * MM,
            min_hole: g.print_hole_mm? * MM,
            supports: s.supports?,
            enclosed: s.enclosed?,
        },
    })
}

fn bolts(l: &LinkSpec) -> Option<mat::FastenerSpec> {
    let s = &l.settings;
    let size = mat::BoltSize::ALL
        .into_iter()
        .find(|b| Some(b.name()) == s.bolt_size.as_deref())?;
    use mat::BoltGrade::*;
    let grade = [G8_8, G10_9, G12_9, A2_70, A4_80]
        .into_iter()
        .find(|b| Some(b.name()) == s.bolt_grade.as_deref())?;
    let engagement = match s.thread.as_deref()? {
        "through" => mat::ThreadEngagement::ThroughBolt,
        "tapped" => mat::ThreadEngagement::Tapped {
            length: s.thread_length_mm? * MM,
        },
        "insert" => mat::ThreadEngagement::Insert {
            outer_d: s.insert_mm? * MM,
            length: s.thread_length_mm? * MM,
        },
        _ => return None,
    };
    Some(mat::FastenerSpec {
        size,
        grade,
        count: s.bolt_count?,
        engagement,
        plate_thickness: s.plate_mm? * MM,
        edge_distance: s.edge_mm? * MM,
    })
}

fn part_code(k: mat::FindingKind) -> &'static str {
    use mat::FindingKind::*;
    match k {
        Input => "part_input",
        Static => "static_strength",
        Fatigue => "fatigue",
        Buckling => "buckling",
        Creep => "creep",
        Temperature => "material_temperature",
        Fasteners => "fastener",
        Printability => "printability",
    }
}

fn check_part(d: &Device, l: &LinkSpec, load: &LinkLoad, r: &mut DeviceReport) {
    let i = Some(load.link);
    let Some(material) = crate::device_catalog::material(&l.material) else {
        r.finding(
            Group::Parts,
            "material",
            i,
            &l.name,
            Outcome::NotAnalyzed,
            "Материал отсутствует в каталоге",
        );
        return;
    };
    let mut p = mat::PartSpec::new(&l.name, material, section(l), l.length_mm * MM);
    // Заделка у основания, свободный конец (AISC 360, C-A-7.1). Горячий прокат
    // принимается для неизвестной обработки стали, чтобы не завысить усталостный запас.
    p.end = mat::EndCondition::FixedFree;
    p.surface = mat::Surface::AsForged;
    p.print = printed(d, l);
    let s = &l.settings;
    let hole_orientation = match s.hole_axis.as_deref() {
        Some("wide") => Some(mat::HoleOrientation::InPlane),
        Some("narrow") => Some(mat::HoleOrientation::OutOfPlane),
        _ => None,
    };
    if l.hole_mm > 0.0 {
        if let (Some(orientation), Some(centered)) = (hole_orientation, s.hole_centered) {
            p.raiser = mat::StressRaiser::Hole {
                d: l.hole_mm * MM,
                orientation,
                centered,
            };
        } else {
            r.finding(
                Group::Parts,
                "hole_data",
                i,
                &l.name,
                Outcome::NotAnalyzed,
                "Для концентратора напряжений нужны ось отверстия и положение относительно центра",
            );
        }
    }
    if material.material().is_printed() {
        p.measured_sn = (|| {
            Some(mat::MeasuredSn {
                a: s.sn_a_mpa? * mat::catalog::MPA,
                b: s.sn_b?,
                max_cycles: s.sn_cycles?,
            })
        })()
        .filter(|_| specified(s.sn_source.as_deref()));
    }
    let lever = s.bolt_lever_mm;
    p.fastening = if s.bolt_size.as_deref() == Some("none") {
        mat::Fastening::NoFasteners
    } else if let Some(b) = bolts(l).filter(|_| lever.is_some_and(|x| x > 0.0)) {
        mat::Fastening::Bolted(b)
    } else {
        mat::Fastening::NotSpecified
    };
    if !load.stall_known {
        r.finding(
            Group::Parts,
            "stall_load",
            i,
            &l.name,
            Outcome::NotAnalyzed,
            "Пиковый момент привода не опубликован. Нагрузка на деталь при заклинивании неизвестна",
        );
    } else if load.stall_nm > 0.0 {
        r.numeric(
            Group::Parts,
            "stall_load",
            i,
            format!("{} — момент при заклинивании", l.name),
            Outcome::Pass,
            load.stall_nm,
            load.bending_nm,
            "Н·м",
            "Паспортный пик привода включён в изгиб: сбой прошивки не должен ломать деталь",
        );
    }
    let m = material.material();
    if m.is_printed()
        && let (Some(a), Some(b)) = (s.sn_a_mpa, s.sn_b)
    {
        // По кривой на 1000 циклов напряжение не может быть выше статической прочности.
        let at_1e3 = a * 1e3_f64.powf(b);
        let strength = m.su_along.max(m.su_across) / mat::catalog::MPA;
        r.numeric(
            Group::Parts,
            "sn_plausibility",
            i,
            format!("{} — достоверность кривой усталости", l.name),
            super::upper(at_1e3, strength),
            at_1e3,
            strength,
            "МПа",
            "Напряжение кривой на 1000 циклах против статической прочности материала",
        );
    }
    let holds_motor = !l.joint.actuator.is_empty()
        || d.links
            .iter()
            .any(|c| c.parent == Some(load.link) && !c.joint.actuator.is_empty());
    let mut part_c = d.usage.ambient_c;
    if m.is_printed() && holds_motor {
        match s.motor_part_c {
            Some(t) if t >= d.usage.ambient_c => part_c = t,
            Some(_) => r.finding(
                Group::Parts,
                "motor_heat",
                i,
                &l.name,
                Outcome::NotAnalyzed,
                "Деталь у мотора не может быть холоднее воздуха вокруг",
            ),
            None => r.finding(
                Group::Parts,
                "motor_heat",
                i,
                &l.name,
                Outcome::NotAnalyzed,
                "Пластик держит мотор. Укажите температуру детали у мотора: от неё зависит размягчение",
            ),
        }
    }
    // Меняющийся наклон тела: верхняя оценка осевого сжатия, среза и реверса нагрузки.
    // Для ползучести весь срок считаем под пиком. Температура здесь — заданная верхняя
    // граница температуры детали; близкие источники тепла проверяются дополнительно.
    let case = mat::LoadCase {
        axial: -load.shear_n,
        shear: load.shear_n,
        bending: load.bending_nm,
        torsion: load.bending_nm,
        temperature_c: part_c,
        cycles: d.usage.cycles,
        load_ratio: -1.0,
        spectrum: Vec::new(),
        sustained_fraction: 1.0,
        sustained_hours: d.settings.lifetime_h.unwrap_or(0.0),
        fastener_shear: load.shear_n,
        fastener_tension: load.shear_n
            + lever
                .filter(|x| *x > 0.0)
                .map_or(0.0, |x| load.bending_nm / (x * MM)),
    };
    let findings = mat::assess_part(&p, &case);
    let total = mat::overall(&findings);
    for f in findings {
        r.numeric(
            Group::Parts,
            part_code(f.kind),
            i,
            format!("{} — {}", l.name, f.kind.title()),
            if l.hole_mm > 0.0
                && (hole_orientation.is_none() || s.hole_centered.is_none())
                && f.outcome == mat::Outcome::Pass
                && matches!(f.kind, mat::FindingKind::Static | mat::FindingKind::Fatigue)
            {
                Outcome::NotAnalyzed
            } else {
                f.outcome.into()
            },
            f.value,
            f.limit,
            f.unit,
            f.detail,
        );
    }
    if total == mat::Outcome::Fail
        && !r
            .findings
            .iter()
            .any(|f| f.group == Group::Parts && f.link == i && f.outcome == Outcome::Fail)
    {
        r.finding(
            Group::Parts,
            "part_summary",
            i,
            &l.name,
            Outcome::Fail,
            "Расчёт детали вернул ошибку",
        );
    }
    if material.material().is_printed() && d.settings.lifetime_h.is_none() {
        r.finding(
            Group::Parts,
            "creep_time",
            i,
            &l.name,
            Outcome::NotAnalyzed,
            "Не задано суммарное время под нагрузкой для оценки ползучести",
        );
    }
    if l.hole_mm > 0.0 && s.bolt_size.as_deref() == Some("none") {
        r.finding(
            Group::Parts,
            "joint_detail",
            i,
            &l.name,
            Outcome::NotAnalyzed,
            "Отверстие есть, но соединение через ось или болт не описано",
        );
    }
    let len = l.length_mm * MM;
    let nominal = nominal_bending_pa(l, load.bending_nm);
    r.numeric(
        Group::Mechanics,
        "bending_stress",
        i,
        format!("{} — номинальное напряжение изгиба", l.name),
        Outcome::Pass,
        nominal / mat::catalog::MPA,
        nominal / mat::catalog::MPA,
        "МПа",
        "M/W по слабой оси, проверка запаса приведена в деталях",
    );
    let e = if m.is_printed() {
        p.print.and_then(|pp| {
            let properties = mat::printed_strength(m, &pp.process, &p.section, case.temperature_c);
            Some(properties.modulus)
        })
    } else {
        Some(m.e_along.min(m.e_across))
    };
    if let Some(e) = e.filter(|&e| e > 0.0) {
        // Интеграл М(x)·(L−x)/(EI); берём постоянную верхнюю границу момента.
        let deflection = (load.bending_nm + load.shear_n * len) * len * len
            / (2.0 * e * p.section.inertia_min());
        r.predict(Quantity::LinkDeflection, i, deflection / MM, None);
    }
}

pub(super) fn evaluate(d: &Device, a: &Analysis, r: &mut DeviceReport) {
    r.finding(
        Group::Mechanics,
        "model",
        None,
        "Дерево звеньев",
        Outcome::Pass,
        "Масса и верхние границы нагрузок при любом направлении силы тяжести",
    );
    if let Some(mass) = d
        .settings
        .accessory_mass_kg
        .filter(|_| specified(d.settings.inventory_source.as_deref()))
    {
        let cell_mass = sim_battery::find_cell(&d.electronics.cell)
            .and_then(|c| c.mass_kg.value)
            .map(|m| m * f64::from(d.electronics.series) * f64::from(d.electronics.parallel));
        if cell_mass.is_some_and(|m| mass < m)
            || mass < 0.0
            || d.links.iter().map(|l| l.end_mass_kg).sum::<f64>() < mass
        {
            r.finding(Group::Mechanics, "inventory", None, "Масса электроники и креплений", Outcome::Fail,
                "Масса дополнительных частей должна быть неотрицательной и распределена по звеньям как концевые массы");
        } else {
            r.predict(Quantity::TotalMass, None, a.total_mass_kg, None);
        }
    } else {
        r.finding(
            Group::Mechanics,
            "inventory",
            None,
            "Масса электроники и креплений",
            Outcome::NotAnalyzed,
            "Нужна ведомость масс батареи, платы, корпуса и креплений с размещением на звеньях",
        );
    }
    for load in &a.links {
        check_part(d, &d.links[load.link], load, r);
    }
    for j in &a.joints {
        let link = &d.links[j.link];
        let i = Some(j.link);
        if j.inertia_complete && link.settings.acceleration_s.is_some() {
            r.predict(Quantity::JointRequiredTorque, i, j.required_nm, None);
        }
        r.predict(
            Quantity::JointSpeed,
            i,
            j.speed_rad_s.to_degrees(),
            Some(link.joint.speed_limit_dps),
        );
        if !j.inertia_complete || link.settings.acceleration_s.is_none() {
            r.finding(
                Group::Mechanics,
                "joint_dynamics",
                i,
                &link.name,
                Outcome::NotAnalyzed,
                "Нет полной приведённой инерции или наименьшего времени разгона",
            );
        }
        if d.subtree(j.link)
            .into_iter()
            .skip(1)
            .any(|k| d.links[k].joint.kind == JointKind::Hinge)
        {
            r.finding(Group::Mechanics, "coupled_motion", i, &link.name, Outcome::NotAnalyzed,
                "Одновременное ускорение вложенных шарниров, реакции и удар при сложении скоростей не подтверждены");
        }
    }
    contact(d, a, r);
}

fn contact(d: &Device, a: &Analysis, r: &mut DeviceReport) {
    use sim_device::safety::{contact_results, crush_gaps};
    let worn =
        d.mount != BodyRegion::None || d.links.iter().any(|l| l.near_body != BodyRegion::None);
    if !specified(d.settings.contact_source.as_deref()) {
        r.finding(Group::Contact, "contact_coverage", None, "Доступные зоны и крепления", Outcome::NotAnalyzed,
            "Нужна проверка по всему ходу, включая пассивные движения, давление креплений и горячие поверхности");
    } else if a.joints.is_empty() {
        r.finding(Group::Contact, "no_moving_contact", None, "Динамический контакт", Outcome::Pass,
            "Звенья неподвижны друг относительно друга. Удар приводом и схождение звеньев неприменимы");
    } else if !worn {
        r.finding(
            Group::Contact,
            "no_body_zone",
            None,
            "Контакт с телом",
            Outcome::Pass,
            "В проверенном рабочем диапазоне нет участков тела в зоне движения",
        );
    }
    if worn {
        for j in &a.joints {
            let l = &d.links[j.link];
            if l.near_body == BodyRegion::None {
                r.finding(
                    Group::Contact,
                    "contact_coverage",
                    Some(j.link),
                    &l.name,
                    Outcome::NotAnalyzed,
                    "Шарнир носимого устройства без заданной зоны доступа тела",
                );
            }
            if l.joint.actuator.is_empty() {
                r.finding(Group::Contact, "passive_motion", Some(j.link), &l.name, Outcome::NotAnalyzed,
                    "Удар и сила схождения свободного шарнира от веса или внешней нагрузки не подтверждены");
            } else if !specified(l.settings.limits_source.as_deref()) {
                r.finding(Group::Contact, "motion_limits", Some(j.link), &l.name, Outcome::NotAnalyzed,
                    "Пределы движения заданы в проекте, нужна независимая проверка под нагрузкой и при отказе");
            }
        }
    }
    for gap in crush_gaps(d) {
        let i = Some(gap.link);
        let l = &d.links[gap.link];
        let code = if gap.guarded { "guard_claim" } else { "crush_gap" };
        let (outcome, detail) = if gap.guarded {
            (
                Outcome::NotAnalyzed,
                "Кожух заявлен. Недоступность опасного зазора не подтверждена",
            )
        } else if gap.gap_mm == 0.0 {
            (
                Outcome::NotAnalyzed,
                "Наименьший зазор не задан (ISO 13854)",
            )
        } else {
            (
                super::lower(gap.gap_mm, gap.required_mm),
                "Наименьший зазор при схождении, ISO 13854",
            )
        };
        r.numeric(
            Group::Contact,
            code,
            i,
            &l.name,
            outcome,
            gap.gap_mm,
            gap.required_mm,
            "мм",
            detail,
        );
    }
    for c in contact_results(d, a, &super::WorkshopCatalog) {
        let i = Some(c.joint_link);
        // OSHA Technical Manual IV-4, RIA TR R15.606: подвижный контакт
        // с чувствительными областями (лицо, виски и горло) исключают.
        if matches!(
            c.region,
            BodyRegion::Head | BodyRegion::Face | BodyRegion::Neck
        ) {
            r.finding(Group::Contact, "sensitive_contact", i, c.region.title(), Outcome::Fail,
                "Описан доступ чувствительной области в зону хода активного звена. Такой контакт нужно исключить");
        }
        for (code, q, value, limit) in [
            (
                "impact",
                Quantity::ImpactForce,
                c.impact_n,
                c.impact_limit_n,
            ),
            ("clamp", Quantity::ClampForce, c.clamp_n, c.clamp_limit_n),
        ] {
            let (outcome, detail) = match value {
                None => (
                    Outcome::NotAnalyzed,
                    "Нет полной инерции или предельного момента привода",
                ),
                Some(v) if !v.is_finite() => (Outcome::Fail, "Неконечная сила контакта"),
                Some(v) if v > limit => (Outcome::Fail, "Сила выше предела ISO/TS 15066"),
                Some(_) if c.joint_link != c.body_link && q == Quantity::ClampForce => (
                    Outcome::NotAnalyzed,
                    "Не задано минимальное плечо контакта от вышестоящего шарнира",
                ),
                Some(_) => (
                    Outcome::Pass,
                    "Предел силы ISO/TS 15066 для заданной пары контакта",
                ),
            };
            r.numeric(
                Group::Contact,
                code,
                i,
                format!("{} — {}", q.title(), c.region.title()),
                outcome,
                value.unwrap_or(0.0),
                limit,
                q.unit(),
                detail,
            );
            if let Some(v) = value {
                r.predict(q, i, v, Some(limit));
            }
        }
    }
}
