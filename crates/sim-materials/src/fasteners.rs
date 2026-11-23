//! Крепёж M2–M8: срез и растяжение болта, вырыв резьбы и вставок из материала детали,
//! смятие и вырыв кромки под отверстием, усталость болта.
//!
//! Модель намеренно простая и в запас:
//! * нагрузка делится между болтами поровну, рычаг и неравномерность задаются в исходной силе;
//! * затяжка не учитывается: болт видит всю переменную силу, а срез идёт по резьбе;
//! * вставка и резьба в печатном материале считаются по прочности поперёк слоёв.

use std::f64::consts::PI;

use crate::catalog::MPA;
use crate::printed::SHEAR_FROM_TENSION;

/// Типоразмер с крупным шагом по ISO 261.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BoltSize {
    M2,
    M2_5,
    M3,
    M4,
    M5,
    M6,
    M8,
}

impl BoltSize {
    /// Все типоразмеры.
    pub const ALL: [BoltSize; 7] = [
        BoltSize::M2,
        BoltSize::M2_5,
        BoltSize::M3,
        BoltSize::M4,
        BoltSize::M5,
        BoltSize::M6,
        BoltSize::M8,
    ];

    /// Обозначение.
    pub fn name(self) -> &'static str {
        match self {
            BoltSize::M2 => "M2",
            BoltSize::M2_5 => "M2.5",
            BoltSize::M3 => "M3",
            BoltSize::M4 => "M4",
            BoltSize::M5 => "M5",
            BoltSize::M6 => "M6",
            BoltSize::M8 => "M8",
        }
    }

    /// Номинальный диаметр, м.
    pub fn d(self) -> f64 {
        match self {
            BoltSize::M2 => 2.0e-3,
            BoltSize::M2_5 => 2.5e-3,
            BoltSize::M3 => 3.0e-3,
            BoltSize::M4 => 4.0e-3,
            BoltSize::M5 => 5.0e-3,
            BoltSize::M6 => 6.0e-3,
            BoltSize::M8 => 8.0e-3,
        }
    }

    /// Крупный шаг, м (ISO 261).
    pub fn pitch(self) -> f64 {
        match self {
            BoltSize::M2 => 0.40e-3,
            BoltSize::M2_5 => 0.45e-3,
            BoltSize::M3 => 0.50e-3,
            BoltSize::M4 => 0.70e-3,
            BoltSize::M5 => 0.80e-3,
            BoltSize::M6 => 1.00e-3,
            BoltSize::M8 => 1.25e-3,
        }
    }

    /// Расчётная площадь резьбы при растяжении, м² (таблица ISO 898-1).
    pub fn stress_area(self) -> f64 {
        1e-6 * match self {
            BoltSize::M2 => 2.07,
            BoltSize::M2_5 => 3.39,
            BoltSize::M3 => 5.03,
            BoltSize::M4 => 8.78,
            BoltSize::M5 => 14.2,
            BoltSize::M6 => 20.1,
            BoltSize::M8 => 36.6,
        }
    }

    /// Внутренний диаметр болта d₃ = d − 1.22687·P, м (ISO 724).
    pub fn minor_diameter(self) -> f64 {
        self.d() - 1.226_869 * self.pitch()
    }

    /// Площадь по внутреннему диаметру, м²: плоскость среза считается проходящей по резьбе.
    pub fn minor_area(self) -> f64 {
        PI * self.minor_diameter().powi(2) / 4.0
    }

    /// Номинальный диаметр головки винта с внутренним шестигранником, м (ISO 4762 / DIN 912).
    pub fn head_diameter(self) -> f64 {
        1e-3 * match self {
            BoltSize::M2 => 3.8,
            BoltSize::M2_5 => 4.5,
            BoltSize::M3 => 5.5,
            BoltSize::M4 => 7.0,
            BoltSize::M5 => 8.5,
            BoltSize::M6 => 10.0,
            BoltSize::M8 => 13.0,
        }
    }
}

/// Класс прочности болта.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoltGrade {
    /// 8.8 по ISO 898-1.
    G8_8,
    /// 10.9 по ISO 898-1.
    G10_9,
    /// 12.9 по ISO 898-1.
    G12_9,
    /// Нержавеющий A2-70 по ISO 3506-1.
    A2_70,
    /// Нержавеющий A4-80 по ISO 3506-1.
    A4_80,
}

impl BoltGrade {
    /// Обозначение.
    pub fn name(self) -> &'static str {
        match self {
            BoltGrade::G8_8 => "8.8",
            BoltGrade::G10_9 => "10.9",
            BoltGrade::G12_9 => "12.9",
            BoltGrade::A2_70 => "A2-70",
            BoltGrade::A4_80 => "A4-80",
        }
    }

    /// Минимальный предел прочности Rm, Па.
    pub fn rm(self) -> f64 {
        MPA * match self {
            BoltGrade::G8_8 => 800.0,   // ISO 898-1, d ≤ 16 мм
            BoltGrade::G10_9 => 1040.0, // ISO 898-1
            BoltGrade::G12_9 => 1220.0, // ISO 898-1
            BoltGrade::A2_70 => 700.0,  // ISO 3506-1
            BoltGrade::A4_80 => 800.0,  // ISO 3506-1
        }
    }

    /// Напряжение пробной нагрузки, Па. У нержавеющих в ISO 3506 его нет: берётся Rp0.2.
    pub fn proof(self) -> f64 {
        MPA * match self {
            BoltGrade::G8_8 => 580.0,  // ISO 898-1, d ≤ 16 мм
            BoltGrade::G10_9 => 830.0, // ISO 898-1
            BoltGrade::G12_9 => 970.0, // ISO 898-1
            BoltGrade::A2_70 => 450.0, // ISO 3506-1, Rp0.2
            BoltGrade::A4_80 => 600.0, // ISO 3506-1, Rp0.2
        }
    }

    /// Полностью скорректированный предел выносливости резьбы, Па (Shigley, табл. 8-17:
    /// катаная резьба, M16–M36; на меньшие размеры перенесено как **оценка**). Для нержавеющих
    /// данных нет.
    pub fn endurance(self) -> Option<f64> {
        match self {
            BoltGrade::G8_8 => Some(129.0 * MPA),
            BoltGrade::G10_9 => Some(162.0 * MPA),
            BoltGrade::G12_9 => Some(190.0 * MPA),
            BoltGrade::A2_70 | BoltGrade::A4_80 => None,
        }
    }
}

/// Как резьба работает в детали.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ThreadEngagement {
    /// Сквозной болт с гайкой: резьбы в материале детали нет.
    ThroughBolt,
    /// Резьба прямо в материале детали, `length` — глубина зацепления, м.
    Tapped { length: f64 },
    /// Резьбовая вставка: `outer_d` — наружный диаметр, `length` — длина, м.
    Insert { outer_d: f64, length: f64 },
}

/// Описание соединения.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FastenerSpec {
    pub size: BoltSize,
    pub grade: BoltGrade,
    /// Число болтов, между которыми делится нагрузка.
    pub count: u32,
    pub engagement: ThreadEngagement,
    /// Толщина детали под болтом, м.
    pub plate_thickness: f64,
    /// Расстояние от центра отверстия до кромки, м.
    pub edge_distance: f64,
}

impl FastenerSpec {
    /// Проверка диапазонов.
    pub fn validate(&self) -> Result<(), String> {
        let ok = |v: f64| v.is_finite() && v > 0.0 && v <= 1.0;
        if self.count == 0 {
            return Err("число болтов равно нулю".into());
        }
        if !ok(self.plate_thickness) {
            return Err(format!(
                "толщина под болтом некорректна: {}",
                self.plate_thickness
            ));
        }
        if !ok(self.edge_distance) {
            return Err(format!(
                "расстояние до кромки некорректно: {}",
                self.edge_distance
            ));
        }
        match self.engagement {
            ThreadEngagement::ThroughBolt => {}
            ThreadEngagement::Tapped { length } => {
                if !ok(length) {
                    return Err(format!("глубина резьбы некорректна: {length}"));
                }
                if length > self.plate_thickness {
                    return Err("глубина резьбы больше толщины детали".into());
                }
            }
            ThreadEngagement::Insert { outer_d, length } => {
                if !ok(outer_d) || !ok(length) {
                    return Err(format!("размеры вставки некорректны: {outer_d} × {length}"));
                }
                if outer_d <= self.size.d() {
                    return Err("наружный диаметр вставки не больше диаметра болта".into());
                }
                if length > self.plate_thickness {
                    return Err("длина вставки больше толщины детали".into());
                }
            }
        }
        Ok(())
    }
}

/// Прочность материала детали для расчёта соединения.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MemberStrength {
    /// Для смятия, Па. У печатных — поперёк слоёв с учётом температуры.
    pub bearing: f64,
    /// Для вырыва резьбы и кромки (сдвиг), Па.
    pub shear: f64,
    /// Снижение вырыва резьбы и вставки в этом материале.
    pub thread_factor: f64,
    /// `false` — модель смятия и вырыва к материалу неприменима (композит).
    pub model_applies: bool,
}

/// Снижение вырыва резьбы и вставки из печатного материала. **Оценка в опасную сторону.**
/// Ориентир: вставки M3 в PLA по CNC Kitchen вырываются при ~1350–1775 Н; формула со снижением
/// даёт заметно меньше (тест `insert_capacity_below_measured`).
pub const PRINTED_THREAD_FACTOR: f64 = 0.5;

/// Наименьшее допустимое расстояние до кромки в диаметрах отверстия (Shigley, §8-12).
pub const MIN_EDGE_RATIO: f64 = 1.5;

/// Площадь среза витков внутренней резьбы: 0.875·π·d·L. Следует из формулы Machinery's Handbook
/// A = π·n·Le·D·(1/(2n) + 0.57735·(D − E)) при E = D − 0.6495·P.
pub fn thread_strip_area(d: f64, length: f64) -> f64 {
    0.875 * PI * d * length
}

/// Сила вырыва резьбы из материала, Н.
pub fn tapped_pullout(d: f64, length: f64, m: &MemberStrength) -> f64 {
    thread_strip_area(d, length) * m.shear * m.thread_factor
}

/// Сила вырыва вставки, Н: срез материала по наружному цилиндру вставки.
pub fn insert_pullout(outer_d: f64, length: f64, m: &MemberStrength) -> f64 {
    PI * outer_d * length * m.shear * m.thread_factor
}

/// Что считалось в соединении.
#[derive(Clone, Debug, PartialEq)]
pub struct FastenerItem {
    pub name: &'static str,
    /// Запас: допустимая нагрузка / действующая.
    pub factor: f64,
}

/// Итог проверки соединения.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct FastenerReport {
    pub items: Vec<FastenerItem>,
    /// Геометрические нарушения: это отказ независимо от запасов.
    pub geometry_faults: Vec<String>,
    /// Что не удалось посчитать.
    pub not_analyzed: Vec<String>,
}

impl FastenerReport {
    /// Наименьший запас и его причина.
    pub fn governing(&self) -> Option<&FastenerItem> {
        self.items
            .iter()
            .min_by(|a, b| a.factor.total_cmp(&b.factor))
    }
}

/// Проверка соединения. Силы — суммарные на группу болтов, Н; знак не важен.
/// `cycles` и `load_ratio` — для усталости болта при растяжении.
pub fn check_fasteners(
    spec: &FastenerSpec,
    member: &MemberStrength,
    shear_force: f64,
    tension_force: f64,
    cycles: f64,
    load_ratio: f64,
) -> FastenerReport {
    let mut r = FastenerReport::default();
    let n = spec.count as f64;
    let d = spec.size.d();
    let v = shear_force.abs() / n;
    let f = tension_force.abs() / n;
    let t = spec.plate_thickness;

    // Под вставку отверстие шире болта: кромка считается от него.
    let hole = match spec.engagement {
        ThreadEngagement::Insert { outer_d, .. } => outer_d,
        _ => d,
    };
    if spec.edge_distance < MIN_EDGE_RATIO * hole {
        r.geometry_faults.push(format!(
            "отверстие слишком близко к кромке: {:.1} мм при минимуме {:.1} мм (1.5d)",
            spec.edge_distance * 1e3,
            MIN_EDGE_RATIO * hole * 1e3
        ));
    }

    if v > 0.0 {
        // Срез по внутреннему диаметру резьбы: τ ≤ 0.577·Sp.
        r.items.push(FastenerItem {
            name: "срез болта",
            factor: SHEAR_FROM_TENSION * spec.grade.proof() * spec.size.minor_area() / v,
        });
        if member.model_applies {
            r.items.push(FastenerItem {
                name: "смятие под болтом",
                factor: member.bearing * d * t / v,
            });
            // Два среза от отверстия до кромки.
            let ligament = spec.edge_distance - hole / 2.0;
            if ligament > 0.0 {
                r.items.push(FastenerItem {
                    name: "вырыв кромки",
                    factor: member.shear * 2.0 * ligament * t / v,
                });
            } else {
                r.geometry_faults.push("отверстие выходит за кромку".into());
            }
        } else {
            r.not_analyzed
                .push("смятие и вырыв кромки: модель к композиту неприменима".into());
        }
    }

    if f > 0.0 {
        r.items.push(FastenerItem {
            name: "растяжение болта",
            factor: spec.grade.proof() * spec.size.stress_area() / f,
        });
        if v > 0.0 {
            // Срез и растяжение в одном сечении: эквивалент Мизеса против пробного напряжения.
            let sigma = f / spec.size.stress_area();
            let tau = v / spec.size.minor_area();
            r.items.push(FastenerItem {
                name: "срез с растяжением",
                factor: spec.grade.proof() / (sigma * sigma + 3.0 * tau * tau).sqrt(),
            });
        }
        match spec.engagement {
            ThreadEngagement::ThroughBolt => {
                // Головка без шайбы: материал сминается под ней и продавливается по её контуру.
                let dk = spec.size.head_diameter();
                if member.model_applies {
                    r.items.push(FastenerItem {
                        name: "смятие под головкой",
                        factor: member.bearing * PI / 4.0 * (dk * dk - d * d) / f,
                    });
                    r.items.push(FastenerItem {
                        name: "продавливание головкой",
                        factor: member.shear * PI * dk * t / f,
                    });
                } else {
                    r.not_analyzed
                        .push("смятие под головкой: модель к композиту неприменима".into());
                }
            }
            ThreadEngagement::Tapped { length } => {
                if member.model_applies {
                    r.items.push(FastenerItem {
                        name: "вырыв резьбы",
                        factor: tapped_pullout(d, length, member) / f,
                    });
                } else {
                    r.not_analyzed
                        .push("вырыв резьбы: модель к композиту неприменима".into());
                }
            }
            ThreadEngagement::Insert { outer_d, length } => {
                if member.model_applies {
                    r.items.push(FastenerItem {
                        name: "вырыв вставки",
                        factor: insert_pullout(outer_d, length, member) / f,
                    });
                } else {
                    r.not_analyzed
                        .push("вырыв вставки: модель к композиту неприменима".into());
                }
            }
        }
        if cycles > 0.0 {
            match spec.grade.endurance() {
                Some(se) => {
                    // Без затяжки болт видит весь размах силы.
                    let s = f / spec.size.stress_area();
                    let sa = s * (1.0 - load_ratio) / 2.0;
                    let sm = s * (1.0 + load_ratio) / 2.0;
                    let denom = sa / se + sm.max(0.0) / spec.grade.rm();
                    if denom > 0.0 {
                        r.items.push(FastenerItem {
                            name: "усталость болта",
                            factor: 1.0 / denom,
                        });
                    }
                }
                None => r.not_analyzed.push(format!(
                    "усталость болта {}: данных о выносливости нет",
                    spec.grade.name()
                )),
            }
        }
    }
    r
}

#[cfg(test)]
mod tests {
    use super::*;

    const SOFT: MemberStrength = MemberStrength {
        bearing: 15e6,
        shear: 8e6,
        thread_factor: PRINTED_THREAD_FACTOR,
        model_applies: true,
    };

    fn spec(engagement: ThreadEngagement) -> FastenerSpec {
        FastenerSpec {
            size: BoltSize::M4,
            grade: BoltGrade::G8_8,
            count: 2,
            engagement,
            plate_thickness: 6e-3,
            edge_distance: 8e-3,
        }
    }

    fn factor(r: &FastenerReport, name: &str) -> f64 {
        r.items.iter().find(|i| i.name == name).unwrap().factor
    }

    #[test]
    fn stress_area_matches_formula() {
        for s in BoltSize::ALL {
            let f = PI / 4.0 * (s.d() - 0.9382 * s.pitch()).powi(2);
            let rel = (f - s.stress_area()).abs() / s.stress_area();
            assert!(rel < 0.01, "{}: {f} против {}", s.name(), s.stress_area());
            assert!(s.minor_area() < s.stress_area());
        }
    }

    #[test]
    fn load_shared_between_bolts() {
        let one = FastenerSpec {
            count: 1,
            ..spec(ThreadEngagement::ThroughBolt)
        };
        let two = spec(ThreadEngagement::ThroughBolt);
        let r1 = check_fasteners(&one, &SOFT, 500.0, 0.0, 0.0, 0.0);
        let r2 = check_fasteners(&two, &SOFT, 500.0, 0.0, 0.0, 0.0);
        let ratio = factor(&r2, "срез болта") / factor(&r1, "срез болта");
        assert!((ratio - 2.0).abs() < 1e-12);
    }

    #[test]
    fn plastic_thread_weaker_than_bolt() {
        let s = spec(ThreadEngagement::Tapped { length: 6e-3 });
        let r = check_fasteners(&s, &SOFT, 0.0, 1000.0, 0.0, 0.0);
        assert!(factor(&r, "вырыв резьбы") < factor(&r, "растяжение болта"));
        assert_eq!(r.governing().unwrap().name, "вырыв резьбы");
    }

    #[test]
    fn head_crushes_soft_material() {
        let s = FastenerSpec {
            size: BoltSize::M3,
            count: 1,
            ..spec(ThreadEngagement::ThroughBolt)
        };
        let r = check_fasteners(&s, &SOFT, 0.0, 400.0, 0.0, 0.0);
        let area = PI / 4.0 * (5.5e-3f64.powi(2) - 3e-3f64.powi(2));
        let expected = 15e6 * area / 400.0;
        assert!((factor(&r, "смятие под головкой") - expected).abs() < 1e-9);
        assert_eq!(r.governing().unwrap().name, "смятие под головкой");
    }

    #[test]
    fn combined_load_is_worse_than_each_alone() {
        let s = FastenerSpec {
            count: 1,
            ..spec(ThreadEngagement::ThroughBolt)
        };
        let r = check_fasteners(&s, &SOFT, 1000.0, 1000.0, 0.0, 0.0);
        let combined = factor(&r, "срез с растяжением");
        assert!(combined < factor(&r, "срез болта") && combined < factor(&r, "растяжение болта"));
    }

    #[test]
    fn close_edge_is_a_fault() {
        let s = FastenerSpec {
            edge_distance: 5e-3,
            ..spec(ThreadEngagement::ThroughBolt)
        };
        let r = check_fasteners(&s, &SOFT, 100.0, 0.0, 0.0, 0.0);
        assert_eq!(r.geometry_faults.len(), 1);
    }

    #[test]
    fn insert_edge_counts_from_insert_diameter() {
        // M4 с кромкой 8 мм годится, а под вставку Ø6.5 нужно уже 9.75 мм.
        let s = spec(ThreadEngagement::Insert {
            outer_d: 6.5e-3,
            length: 5e-3,
        });
        let r = check_fasteners(&s, &SOFT, 100.0, 0.0, 0.0, 0.0);
        assert_eq!(r.geometry_faults.len(), 1, "{:?}", r.geometry_faults);
    }

    #[test]
    fn stainless_fatigue_not_guessed() {
        let s = FastenerSpec {
            grade: BoltGrade::A2_70,
            ..spec(ThreadEngagement::ThroughBolt)
        };
        let r = check_fasteners(&s, &SOFT, 0.0, 100.0, 1e6, 0.0);
        assert_eq!(r.not_analyzed.len(), 1);
        assert!(r.items.iter().all(|i| i.name != "усталость болта"));
    }

    #[test]
    fn composite_bearing_not_guessed() {
        let m = MemberStrength {
            model_applies: false,
            ..SOFT
        };
        let s = spec(ThreadEngagement::Tapped { length: 6e-3 });
        let r = check_fasteners(&s, &m, 100.0, 100.0, 0.0, 0.0);
        assert_eq!(r.not_analyzed.len(), 2);
    }

    #[test]
    fn insert_capacity_below_measured() {
        // Вставка M3 (наружный ~4.6 мм, длина 5.7 мм) в PLA поперёк слоёв при 23 °C.
        // Измерения: 1353.6 Н (MDPI Eng. Proc. 157(1):40) и ~1775 Н (CNC Kitchen).
        let across = 28e6 * crate::printed::INTERLAYER_BOND_FACTOR;
        let m = MemberStrength {
            bearing: across,
            shear: across * SHEAR_FROM_TENSION,
            thread_factor: PRINTED_THREAD_FACTOR,
            model_applies: true,
        };
        let f = insert_pullout(4.6e-3, 5.7e-3, &m);
        assert!(f < 1353.6, "расчёт {f:.0} Н выше измеренного");
        assert!(f > 100.0, "расчёт {f:.0} Н бессмысленно мал");
    }

    #[test]
    fn invalid_spec_rejected() {
        let base = spec(ThreadEngagement::ThroughBolt);
        for s in [
            FastenerSpec { count: 0, ..base },
            FastenerSpec {
                plate_thickness: f64::NAN,
                ..base
            },
            FastenerSpec {
                edge_distance: -1.0,
                ..base
            },
            spec(ThreadEngagement::Tapped {
                length: f64::INFINITY,
            }),
            spec(ThreadEngagement::Insert {
                outer_d: 3e-3,
                length: 5e-3,
            }),
            spec(ThreadEngagement::Tapped { length: 7e-3 }), // глубже толщины детали 6 мм
            spec(ThreadEngagement::Insert {
                outer_d: 6e-3,
                length: 7e-3,
            }),
        ] {
            assert!(s.validate().is_err(), "{s:?}");
        }
        assert!(base.validate().is_ok());
    }
}
