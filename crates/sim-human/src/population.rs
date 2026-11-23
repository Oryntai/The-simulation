//! Разброс людей: сила по полу и перцентилю, рост и масса, разброс параметров мышц.
//!
//! Сила берётся из измерений на людях (Frey-Law 2012, Harbo 2012, Graves 1990), а не из модели:
//! модель мышц сильнее реальных людей на 34–42% при 90° (см. [`crate::data`]). Там, где есть два
//! источника, берём меньшую оценку — ошибка в эту сторону не даёт ложного «безопасно».

use crate::data::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sex {
    Male,
    Female,
}

/// Действие в суставе, для которого есть измерения силы.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointAction {
    ElbowFlexion,
    ElbowExtension,
    ShoulderElevation,
    ShoulderDepression,
    TrunkExtension,
}

impl JointAction {
    pub fn label(self) -> &'static str {
        match self {
            Self::ElbowFlexion => "сгибание локтя",
            Self::ElbowExtension => "разгибание локтя",
            Self::ShoulderElevation => "подъём плеча",
            Self::ShoulderDepression => "опускание плеча",
            Self::TrunkExtension => "разгибание поясницы",
        }
    }
}

/// Человек, для которого считается предел.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Person {
    pub sex: Sex,
    /// Перцентиль силы, 1…99.
    pub strength_percentile: f64,
    pub age_years: f64,
    pub stature_m: f64,
    pub mass_kg: f64,
}

/// Оценка силы человека в данном действии.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StrengthEstimate {
    /// Момент для заданного перцентиля, Н·м.
    pub nm: f64,
    /// Средний момент той же группы (без перцентиля), Н·м.
    pub mean_nm: f64,
    /// Угол вне измеренного диапазона либо нет угловой зависимости — значение приближённое.
    pub extrapolated: bool,
    pub source: &'static str,
}

/// z-оценка нормального распределения по перцентилю (0…100, не включая края).
/// Рациональное приближение Abramowitz & Stegun, 26.2.23: погрешность меньше 4.5e-4.
pub fn z_score(percentile: f64) -> f64 {
    let p = percentile / 100.0;
    if !(p > 0.0 && p < 1.0) {
        return f64::NAN;
    }
    let (q, sign) = if p < 0.5 { (p, -1.0) } else { (1.0 - p, 1.0) };
    let t = (-2.0 * q.ln()).sqrt();
    let num = 2.515517 + t * (0.802853 + t * 0.010328);
    let den = 1.0 + t * (1.432788 + t * (0.189269 + t * 0.001308));
    sign * (t - num / den)
}

/// Линейная интерполяция по таблице (угол, значение); вне таблицы — краевое значение и признак.
fn interp(table: &[(f64, f64)], x: f64) -> (f64, bool) {
    let first = table[0];
    let last = table[table.len() - 1];
    if x < first.0 {
        return (table.iter().map(|p| p.1).fold(first.1, f64::min), true);
    }
    if x > last.0 {
        return (table.iter().map(|p| p.1).fold(last.1, f64::min), true);
    }
    for w in table.windows(2) {
        if x <= w[1].0 {
            let t = (x - w[0].0) / (w[1].0 - w[0].0);
            return (w[0].1 + t * (w[1].1 - w[0].1), false);
        }
    }
    (last.1, false)
}

fn frey_law(table: &[ElbowDatum; 5], sex: Sex, angle_deg: f64, z: f64) -> (f64, bool) {
    let column = table.map(|(angle, male, female)| {
        let (mean, sd) = if sex == Sex::Male { male } else { female };
        (angle, mean + z * sd)
    });
    interp(&column, angle_deg)
}

impl HarboModel {
    pub fn predict(&self, person: &Person) -> f64 {
        self.intercept
            + self.per_year * person.age_years
            + self.per_metre * person.stature_m
            + self.per_kg * person.mass_kg
    }
}

/// Сила человека по измерениям.
///
/// * локоть — меньшее из Frey-Law (по углу) и регрессии Harbo (по возрасту, росту, массе);
/// * плечо — Harbo, одна поза: зависимости от угла в источнике нет, значение помечено приближённым;
/// * поясница — нижний край Graves 1990.
///
/// Перцентиль — нормальное распределение; при 1-м перцентиле и большом разбросе значение может
/// подойти к нулю, отрицательным не становится.
pub fn measured_strength(action: JointAction, angle_deg: f64, person: &Person) -> StrengthEstimate {
    let z = z_score(person.strength_percentile);
    if ![
        z,
        angle_deg,
        person.age_years,
        person.stature_m,
        person.mass_kg,
    ]
    .iter()
    .all(|x| x.is_finite())
    {
        return StrengthEstimate {
            nm: f64::NAN,
            mean_nm: f64::NAN,
            extrapolated: true,
            source: "некорректные входные данные",
        };
    }
    let male = person.sex == Sex::Male;
    let (nm, mean_nm, extrapolated, source) = match action {
        JointAction::ElbowFlexion | JointAction::ElbowExtension => {
            let flexion = action == JointAction::ElbowFlexion;
            let table = if flexion {
                &FREY_LAW_ELBOW_FLEXION
            } else {
                &FREY_LAW_ELBOW_EXTENSION
            };
            let harbo = match (flexion, male) {
                (true, true) => HARBO_ELBOW_FLEX_ISO_M,
                (true, false) => HARBO_ELBOW_FLEX_ISO_F,
                (false, true) => HARBO_ELBOW_EXT_ISOK_M,
                (false, false) => HARBO_ELBOW_EXT_ISOK_F,
            };
            let (by_angle, outside) = frey_law(table, person.sex, angle_deg, z);
            let (frey, _) = frey_law(table, person.sex, angle_deg, 0.0);
            let by_body = harbo.predict(person) + z * harbo.rsd;
            let mean = frey.min(harbo.predict(person));
            (
                by_angle.min(by_body),
                mean,
                outside,
                "Frey-Law 2012 и Harbo 2012, меньшее",
            )
        }
        JointAction::ShoulderElevation => {
            let m = if male {
                HARBO_SHOULDER_ABD_ISO_M
            } else {
                HARBO_SHOULDER_ABD_ISO_F
            };
            (
                m.predict(person) + z * m.rsd,
                m.predict(person),
                true,
                "Harbo 2012, отведение в нейтральной позе (оценка для других углов)",
            )
        }
        JointAction::ShoulderDepression => {
            let m = if male {
                HARBO_SHOULDER_ADD_ISOK_M
            } else {
                HARBO_SHOULDER_ADD_ISOK_F
            };
            (
                m.predict(person) + z * m.rsd,
                m.predict(person),
                true,
                "Harbo 2012, приведение 60 °/с (оценка для других углов)",
            )
        }
        JointAction::TrunkExtension => {
            let (mean, sd) = if male {
                GRAVES_TRUNK_EXTENSION_M
            } else {
                GRAVES_TRUNK_EXTENSION_F
            };
            (
                mean + z * sd,
                mean,
                true,
                "Graves 1990, нижний край диапазона",
            )
        }
    };
    StrengthEstimate {
        nm: if nm.is_finite() {
            nm.max(0.0)
        } else {
            f64::NAN
        },
        mean_nm,
        extrapolated: extrapolated
            || !(HARBO_AGE_RANGE.0..=HARBO_AGE_RANGE.1).contains(&person.age_years),
        source,
    }
}

/// Рост и масса по таблице NHANES: индекс 0, 1, 2 — 5-й, 50-й, 95-й перцентили.
pub fn nhanes_body(sex: Sex, index: usize) -> (f64, f64) {
    let i = index.min(2);
    match sex {
        Sex::Male => (NHANES_STATURE_M_MALE[i], NHANES_MASS_KG_MALE[i]),
        Sex::Female => (NHANES_STATURE_M_FEMALE[i], NHANES_MASS_KG_FEMALE[i]),
    }
}

/// Вариант параметров мышц: множители пиковой силы и плеча для всех мышц разом.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MuscleVariant {
    pub f0: f64,
    pub moment_arm: f64,
}

/// Разброс параметров мышц, доля (±20%): задан владельцем (HUMAN_MODEL.md, разд. 5).
pub const MUSCLE_PARAM_SPREAD: f64 = 0.20;

/// Номинал и четыре угла разброса ±20%: сила и плечо меняются независимо.
pub fn muscle_variants() -> [MuscleVariant; 5] {
    let (lo, hi) = (1.0 - MUSCLE_PARAM_SPREAD, 1.0 + MUSCLE_PARAM_SPREAD);
    [
        MuscleVariant {
            f0: 1.0,
            moment_arm: 1.0,
        },
        MuscleVariant {
            f0: lo,
            moment_arm: lo,
        },
        MuscleVariant {
            f0: lo,
            moment_arm: hi,
        },
        MuscleVariant {
            f0: hi,
            moment_arm: lo,
        },
        MuscleVariant {
            f0: hi,
            moment_arm: hi,
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn person(sex: Sex, p: f64) -> Person {
        Person {
            sex,
            strength_percentile: p,
            age_years: 40.0,
            stature_m: 1.7,
            mass_kg: 70.0,
        }
    }

    #[test]
    fn z_score_matches_normal_table() {
        for (p, z) in [
            (5.0, -1.644854),
            (50.0, 0.0),
            (95.0, 1.644854),
            (1.0, -2.326348),
        ] {
            assert!((z_score(p) - z).abs() < 1e-3, "p={p}: {}", z_score(p));
        }
        assert!(z_score(0.0).is_nan());
        assert!(z_score(f64::NAN).is_nan());
    }

    #[test]
    fn weak_woman_is_weaker_than_median_man() {
        for a in [
            JointAction::ElbowFlexion,
            JointAction::ElbowExtension,
            JointAction::ShoulderElevation,
            JointAction::TrunkExtension,
        ] {
            let weak = measured_strength(a, 90.0, &person(Sex::Female, 5.0)).nm;
            let median = measured_strength(a, 90.0, &person(Sex::Male, 50.0)).nm;
            assert!(
                weak > 0.0 && weak < 0.7 * median,
                "{a:?}: {weak} / {median}"
            );
        }
    }

    #[test]
    fn angle_outside_table_is_flagged() {
        let p = person(Sex::Male, 50.0);
        assert!(measured_strength(JointAction::ElbowFlexion, 5.0, &p).extrapolated);
        assert!(!measured_strength(JointAction::ElbowFlexion, 60.0, &p).extrapolated);
    }
}
