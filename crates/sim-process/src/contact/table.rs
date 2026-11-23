use std::{array, collections::BTreeMap, f64::consts::PI};

use sim_signal::Rate;
use sim_world::{
    CompiledWorld, ContactPair, ContactSettings, FrictionCone, MIN_CONTACT_FRICTION, Provenance,
};

use super::{ContactFriction, ContactTable, NormalResponse, PairParameters};
use crate::{Analysis, ProcessError, SourcedRange};

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ContactMaterialTable {
    pairs: BTreeMap<[String; 2], PairParameters>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct GeometryMaterial {
    pub geometry: String,
    pub material: String,
}

#[derive(Clone, Debug, PartialEq)]
pub enum FrictionReference {
    /// Передать MuJoCo явный [0, 0], означающий общий solref.
    UseNormal,
    /// Прямая запись демпфирования касательных ограничений, с⁻¹.
    DirectDamping(SourcedRange),
}

#[derive(Clone, Debug, PartialEq)]
pub struct PairCalibration {
    pub geometries: [String; 2],
    /// d₀, d_width, ширина в метрах, середина и степень (MuJoCo 4.0.1).
    pub solimp: [f64; 5],
    /// Источник параметров импеданса и область их применимости.
    pub impedance_source: Provenance,
    /// Приведённая масса одиночной нормальной моды, обязательна для пружины.
    pub effective_mass_kg: Option<SourcedRange>,
    pub friction_reference: FrictionReference,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ResolvedContactPair {
    pub pair: ContactPair,
    pub materials: [String; 2],
    pub parameters: PairParameters,
    pub calibration: PairCalibration,
    /// Достаточность источников и диапазонов для использования номинала.
    /// Даже при достаточных данных нужны сходимость и проверка геометрии.
    pub nominal_coverage: Analysis<()>,
}

pub(super) fn invalid(message: impl Into<String>) -> ProcessError {
    ProcessError::InvalidInput(message.into())
}

pub(super) fn name(value: &str) -> bool {
    !value.trim().is_empty() && !value.chars().any(char::is_control)
}

fn pair_key<T: Ord>(mut pair: [T; 2]) -> [T; 2] {
    if pair[0] > pair[1] {
        pair.swap(0, 1);
    }
    pair
}

pub(super) fn source(provenance: &Provenance) -> Result<(), ProcessError> {
    let description = match provenance {
        Provenance::Datasheet(s)
        | Provenance::Paper(s)
        | Provenance::Measured(s)
        | Provenance::Derived(s)
        | Provenance::Estimate(s) => s,
    };
    if !name(description) {
        return Err(invalid("для контактного параметра нужен непустой источник"));
    }
    Ok(())
}

pub(super) fn traceable(provenance: &Provenance) -> bool {
    // В существующем Provenance у Derived хранится строка без исходных данных.
    // Без проверяемой цепочки нельзя решить, была ли среди источников оценка.
    matches!(
        provenance,
        Provenance::Datasheet(_) | Provenance::Paper(_) | Provenance::Measured(_)
    )
}

pub(super) fn range(value: &SourcedRange, valid: impl Fn(f64) -> bool) -> Result<(), ProcessError> {
    source(&value.provenance)?;
    let [min, nominal, max] = [value.min, value.nominal, value.max];
    if !(min <= nominal && nominal <= max)
        || ![min, nominal, max]
            .into_iter()
            .all(|x| x.is_finite() && valid(x))
    {
        return Err(invalid(
            "неконечный или недопустимый диапазон контактного параметра",
        ));
    }
    Ok(())
}

const CRITICAL_DURATION: f64 = 2.0; // В единицах 1/ω₀, нуль силы критически затухающего осциллятора.
const ZETA_LIMIT: f64 = 50.0; // Граница проработанного алгоритма restitution_solref в sim-mujoco.
const BISSECTION_STEPS: usize = f64::MANTISSA_DIGITS as usize;

fn impact(zeta: f64) -> (f64, f64) {
    // Отрыв при нулевой силе, Schwager & Pöschel (2007). При ζ>1
    // r1*r2=1, поэтому медленный корень вычисляется без вычитания.
    let duration = if zeta < 1.0 {
        let w = (1.0 - zeta * zeta).sqrt();
        (PI - 2.0 * zeta.asin()) / w
    } else if zeta == 1.0 {
        CRITICAL_DURATION
    } else {
        let w = (zeta * zeta - 1.0).sqrt();
        2.0 * (zeta + w).ln() / w
    };
    ((-zeta * duration).exp(), duration)
}

fn restitution_reference(e: f64, duration: f64) -> [f64; 2] {
    let zeta = if e == 1.0 {
        0.0
    } else {
        let (mut lo, mut hi) = (0.0, ZETA_LIMIT);
        for _ in 0..BISSECTION_STEPS {
            let mid = (lo + hi) / 2.0;
            if impact(mid).0 > e {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        (lo + hi) / 2.0
    };
    let omega = impact(zeta).1 / duration;
    [-omega * omega, -2.0 * zeta * omega]
}

impl ContactFriction {
    fn components(&self) -> (u8, Vec<&SourcedRange>) {
        match self {
            Self::NormalOnly => (1, vec![]),
            Self::Sliding { sliding: [a, b] } => (3, vec![a, b]),
            Self::SlidingAndTorsion {
                sliding: [a, b],
                torsion_m,
            } => (4, vec![a, b, torsion_m]),
            Self::SlidingTorsionAndRolling {
                sliding: [a, b],
                torsion_m,
                rolling_m: [c, d],
            } => (6, vec![a, b, torsion_m, c, d]),
        }
    }
}

impl PairParameters {
    fn validate(&self) -> Result<(), ProcessError> {
        for component in self.friction.components().1 {
            range(component, |x| x >= MIN_CONTACT_FRICTION)?;
        }
        match &self.normal {
            NormalResponse::Restitution {
                coefficient,
                duration_s,
            } => {
                let min_e = impact(ZETA_LIMIT).0;
                range(coefficient, |x| (min_e..=1.0).contains(&x))?;
                range(duration_s, |x| x > 0.0)?;
            }
            NormalResponse::SpringDamper {
                stiffness_n_m,
                damping_ns_m,
            }
            | NormalResponse::Direct {
                stiffness_per_s2: stiffness_n_m,
                damping_per_s: damping_ns_m,
            } => {
                range(stiffness_n_m, |x| x > 0.0)?;
                range(damping_ns_m, |x| x >= 0.0)?;
            }
        }
        Ok(())
    }

    /// Переводит номиналы в явные параметры. Маппинг физических законов
    /// применим к одиночной нормальной моде с постоянным импедансом.
    pub fn contact_settings(
        &self,
        calibration: &PairCalibration,
        rate: Rate,
        cone: FrictionCone,
    ) -> Result<ContactSettings, ProcessError> {
        self.validate()?;
        source(&calibration.impedance_source)?;
        let [d0, d, ..] = calibration.solimp;
        if !matches!(self.normal, NormalResponse::Direct { .. }) && d0 != d {
            return Err(invalid(
                "перевод нормального закона требует постоянного импеданса",
            ));
        }
        if let Some(mass) = &calibration.effective_mass_kg {
            range(mass, |x| x > 0.0)?;
        }
        let (solref, stiffest) = match &self.normal {
            NormalResponse::SpringDamper {
                stiffness_n_m: k,
                damping_ns_m: b,
            } => {
                let mass = calibration
                    .effective_mass_kg
                    .as_ref()
                    .ok_or_else(|| invalid("для пружины неизвестна приведённая масса"))?;
                // Из A=1/m, R=A(1−d)/d и a_ref=−Bv−K*d*r:
                // F = m*d*(a_ref−a_free). При постоянном d, K=|ref[0]|/d²
                // и B=|ref[1]|/d. Поэтому вклады в закон равны m*|ref|.
                (
                    [-k.nominal / mass.nominal, -b.nominal / mass.nominal],
                    [-k.max / mass.min, -b.max / mass.min],
                )
            }
            NormalResponse::Restitution {
                coefficient,
                duration_s,
            } => {
                let nominal = restitution_reference(coefficient.nominal, duration_s.nominal);
                let elastic = restitution_reference(coefficient.max, duration_s.min);
                let damped = restitution_reference(coefficient.min, duration_s.min);
                // Длительность в единицах 1/ω₀ убывает с ζ, −ln(e) возрастает.
                (nominal, [elastic[0], damped[1]])
            }
            NormalResponse::Direct {
                stiffness_per_s2,
                damping_per_s,
            } => (
                [-stiffness_per_s2.nominal, -damping_per_s.nominal],
                [-stiffness_per_s2.max, -damping_per_s.max],
            ),
        };
        let (dim, components) = self.friction.components();
        let friction = array::from_fn(|i| {
            components
                .get(i)
                .map_or(MIN_CONTACT_FRICTION, |x| x.nominal)
        });
        let solreffriction = match &calibration.friction_reference {
            FrictionReference::UseNormal => [0.0; 2],
            FrictionReference::DirectDamping(value) => {
                range(value, |x| x > 0.0)?;
                [0.0, -value.nominal]
            }
        };
        let mut pair = ContactPair {
            geometries: calibration.geometries.clone(),
            settings: ContactSettings {
                dim,
                friction,
                solref,
                solreffriction,
                solimp: calibration.solimp,
                // Граница физической поверхности, без зазора обнаружения и адгезии.
                margin_m: 0.0,
                gap_m: 0.0,
                adhesion_n: 0.0,
            },
        };
        pair.validate_step(rate, cone).map_err(invalid)?;
        let nominal = pair.settings;
        pair.settings.solref = stiffest;
        pair.validate_step(rate, cone).map_err(invalid)?;
        Ok(nominal)
    }

    fn nominal_coverage(&self, calibration: &PairCalibration) -> Analysis<()> {
        let mut ranges = self.friction.components().1;
        match &self.normal {
            NormalResponse::Restitution {
                coefficient,
                duration_s,
            } => ranges.extend([coefficient, duration_s]),
            NormalResponse::SpringDamper {
                stiffness_n_m,
                damping_ns_m,
            } => ranges.extend([stiffness_n_m, damping_ns_m]),
            NormalResponse::Direct { .. } => {
                return Analysis::NotAnalyzed {
                    reason: "прямой solref требует независимой физической калибровки".into(),
                };
            }
        }
        ranges.extend(calibration.effective_mass_kg.iter());
        if let FrictionReference::DirectDamping(value) = &calibration.friction_reference {
            ranges.push(value);
        }
        if !traceable(&calibration.impedance_source)
            || ranges.iter().any(|p| !traceable(&p.provenance))
        {
            return Analysis::NotAnalyzed {
                reason: "оценка либо неполная цепочка происхождения".into(),
            };
        }
        if ranges.iter().any(|p| p.min != p.max) {
            return Analysis::NotAnalyzed {
                reason: "номинал не покрывает заданные диапазоны".into(),
            };
        }
        Analysis::Analyzed {
            value: (),
            scope: "номиналы с проверяемыми источниками".into(),
        }
    }
}

impl ContactMaterialTable {
    pub fn new(
        rows: impl IntoIterator<Item = ([String; 2], PairParameters)>,
    ) -> Result<Self, ProcessError> {
        let mut pairs = BTreeMap::new();
        for (materials, parameters) in rows {
            if !materials.iter().all(|s| name(s)) {
                return Err(invalid("непустые имена материалов обязательны"));
            }
            parameters.validate()?;
            let key = pair_key(materials);
            if pairs.insert(key.clone(), parameters).is_some() {
                return Err(invalid(format!("повторная пара материалов {key:?}")));
            }
        }
        Ok(Self { pairs })
    }

    /// Требует назначение и калибровку каждой потенциальной пары базовой сцены.
    /// Имена разрешаются после сборки структурных добавлений и фильтров.
    pub fn resolve(
        &self,
        scene: &CompiledWorld,
        assignments: &[GeometryMaterial],
        calibrations: &[PairCalibration],
        cone: FrictionCone,
    ) -> Result<Vec<ResolvedContactPair>, ProcessError> {
        let valid_geometries = scene.contact_geometry_names().map_err(invalid)?;
        let mut by_geom = BTreeMap::new();
        for entry in assignments {
            if !name(&entry.material)
                || !valid_geometries.contains(&entry.geometry)
                || by_geom
                    .insert(entry.geometry.as_str(), entry.material.as_str())
                    .is_some()
            {
                return Err(invalid(format!(
                    "неверное или повторное назначение «{}»",
                    entry.geometry
                )));
            }
        }
        let mut by_pair = BTreeMap::new();
        for calibration in calibrations {
            let key = pair_key(calibration.geometries.clone());
            if by_pair.insert(key.clone(), calibration).is_some() {
                return Err(invalid(format!("повторная калибровка {key:?}")));
            }
        }
        let mut candidates = scene.contact_candidates().map_err(invalid)?;
        for pair in &mut candidates {
            if pair[0] > pair[1] {
                pair.swap(0, 1);
            }
        }
        candidates.sort();
        let mut resolved = Vec::with_capacity(candidates.len());
        for geometries in candidates {
            let [a, b] = geometries.each_ref().map(String::as_str);
            let get_material = |g| {
                by_geom
                    .get(g)
                    .copied()
                    .ok_or_else(|| invalid(format!("для «{g}» материал не задан")))
            };
            let materials = [get_material(a)?, get_material(b)?];
            let Analysis::Analyzed {
                value: parameters, ..
            } = self.lookup(materials)?
            else {
                return Err(invalid(format!(
                    "для {geometries:?} неизвестна пара материалов {materials:?}"
                )));
            };
            let calibration = by_pair
                .remove(&geometries)
                .ok_or_else(|| invalid(format!("для {geometries:?} нет калибровки пары")))?;
            let settings = parameters.contact_settings(calibration, scene.physics, cone)?;
            resolved.push(ResolvedContactPair {
                pair: ContactPair {
                    geometries,
                    settings,
                },
                materials: materials.map(str::to_owned),
                parameters: parameters.clone(),
                calibration: calibration.clone(),
                nominal_coverage: parameters.nominal_coverage(calibration),
            });
        }
        if let Some((key, _)) = by_pair.first_key_value() {
            return Err(invalid(format!(
                "калибровка {key:?} не относится к разрешённой паре"
            )));
        }
        Ok(resolved)
    }
}

impl ContactTable for ContactMaterialTable {
    fn lookup(&self, materials: [&str; 2]) -> Result<Analysis<&PairParameters>, ProcessError> {
        if !materials.iter().all(|s| name(s)) {
            return Err(invalid("непустые имена материалов обязательны"));
        }
        let key = pair_key(materials);
        Ok(match self.pairs.get(&key.map(str::to_owned)) {
            Some(value) => Analysis::Analyzed {
                value,
                scope: format!("пара материалов {key:?}"),
            },
            None => Analysis::NotAnalyzed {
                reason: format!("неизвестная пара материалов {key:?}"),
            },
        })
    }
}
