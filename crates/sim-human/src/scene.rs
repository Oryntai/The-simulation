//! Участник общей сцены. Геометрия и инерция предназначены для расчёта движения
//! и внешних реакций. Применимость пределов проверяется отдельно от столкновений.

mod anthropometry;
mod exposure;
mod model;

use std::collections::BTreeMap;
use std::path::PathBuf;

pub use anthropometry::{
    Anthropometry, BaseVariant, Marginals, ParticipantSpec, Proportion, ProportionMeasure,
    marginal_matrix, marginals,
};
pub use exposure::{
    Applicability, Assessment, AssessmentIssue, Direction, Exposure, ExposureContext,
    ExposureLimit, ExposureOutcome, Interval, LimitStatus, LoadMode, Metric, Patch, Source,
    TissueCriterion, assess_exposure, assess_exposure_with_tissue, limits,
};
pub use model::build;

/// Стабильный локальный идентификатор поверхности. При размещении его дополняет
/// внешний идентификатор участника, принадлежащий компилятору общего мира.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct RegionId(pub String);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    Midline,
    Right,
    Left,
}

#[derive(Clone, Debug)]
pub enum SurfaceShape {
    /// Радиус сферических концов и половина длины прямой части.
    Capsule {
        radius_m: f64,
        half_segment_m: f64,
    },
    Ellipsoid {
        semi_axes_m: [f64; 3],
    },
}

#[derive(Clone, Debug)]
pub struct Surface {
    pub geometry: String,
    pub body: String,
    pub label: &'static str,
    pub side: Side,
    pub digit: Option<u8>,
    /// Центр и ориентация поверхности в рамке тела, метры и кватернион wxyz.
    pub center_m: [f64; 3],
    pub orientation: [f64; 4],
    pub shape: SurfaceShape,
    pub material: &'static str,
    pub contype: u32,
    pub conaffinity: u32,
    pub source: Source<'static>,
    pub status: LimitStatus,
    pub applicability: &'static str,
    /// Площадь физического пятна пока требует отдельного замера интерфейса.
    pub effective_area_m2: Option<Interval>,
}

impl Surface {
    /// Симметричная проверка масок MuJoCo. Явная пара и исключения проверяются
    /// отдельно в собранной сцене.
    pub fn matches_masks(&self, contype: u32, conaffinity: u32) -> bool {
        (contype & self.conaffinity) != 0 || (conaffinity & self.contype) != 0
    }
}

#[derive(Clone, Debug)]
pub struct Port {
    pub site: String,
    pub body: String,
    pub region: RegionId,
    pub point_m: [f64; 3],
    /// Ось Z точки направлена по внешней нормали. Кватернион wxyz.
    pub orientation: [f64; 4],
    pub direction: &'static str,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AxisKind {
    Rotation,
    Translation,
}

#[derive(Clone, Debug)]
pub struct Axis {
    pub joint: String,
    pub body: String,
    pub label: &'static str,
    pub kind: AxisKind,
    pub range: [f64; 2],
    pub direction: [f64; 3],
    pub anchor_m: [f64; 3],
    pub neutral: f64,
    pub dependent: bool,
    pub source: Source<'static>,
}

#[derive(Clone, Debug)]
pub struct AxisCoupling {
    pub dependent: String,
    pub driver: String,
    /// q₁ − ref₁ = p(q₂ − ref₂), коэффициенты от нулевого к четвёртому.
    pub coefficients: [f64; 5],
}

#[derive(Clone, Debug)]
pub struct Segment {
    pub name: String,
    pub label: &'static str,
    pub mass_kg: f64,
    pub com_m: [f64; 3],
    /// Тензор относительно центра масс в рамке сегмента, кг·м².
    pub inertia_kg_m2: [[f64; 3]; 3],
    pub source: Source<'static>,
}

#[derive(Clone, Debug)]
pub struct MassProperties {
    pub mass_kg: f64,
    pub com_m: [f64; 3],
    /// Тензор нейтральной позы относительно общего центра масс, кг·м².
    pub inertia_kg_m2: [[f64; 3]; 3],
    pub segments: Vec<Segment>,
}

#[derive(Clone, Debug)]
pub struct ParticipantModel {
    pub mjcf: String,
    pub source: PathBuf,
    pub resources: Vec<PathBuf>,
    pub root_body: String,
    /// В нейтральной позе Z направлена вверх, X вперёд, Y влево.
    /// Ноль по Z соответствует нижней границе прокси в нейтральной позе.
    pub frame: &'static str,
    /// Аналитический паспорт до округления чисел в MJCF.
    pub mass: MassProperties,
    /// Фактические параметры после загрузки MJCF, для сверки при композиции.
    pub exported_mass: MassProperties,
    /// Имя пространственной координаты для свободного основания, если она задана.
    pub root_joint: Option<String>,
    pub axes: Vec<Axis>,
    pub couplings: Vec<AxisCoupling>,
    pub ports: Vec<Port>,
    pub regions: BTreeMap<RegionId, Surface>,
    pub excluded_body_pairs: Vec<[String; 2]>,
    pub anthropometry: Anthropometry,
    pub limitations: Vec<&'static str>,
}

impl ParticipantModel {
    /// Возвращает координату порта, только если порт принадлежит указанному участку.
    pub fn mount(&self, site: &str, region: &RegionId) -> Result<&Port, String> {
        self.ports
            .iter()
            .find(|port| port.site == site && &port.region == region)
            .ok_or_else(|| format!("точка {site} отсутствует на участке {}", region.0))
    }

    /// Проверяет независимые команды и затронутые зависимые координаты.
    /// Достижимость траектории, её динамика и все ограничения проверяются в общем прогоне.
    pub fn validate_pose(&self, pose: &BTreeMap<String, f64>) -> Result<(), String> {
        for (joint, value) in pose {
            let axis = self
                .axes
                .iter()
                .find(|axis| &axis.joint == joint)
                .ok_or_else(|| format!("неизвестная ось {joint}"))?;
            let [lo, hi] = axis.range;
            if !value.is_finite() || *value < lo || *value > hi || axis.dependent {
                return Err(format!("команда вне области независимой оси {joint}"));
            }
        }
        let axes: BTreeMap<_, _> = self
            .axes
            .iter()
            .map(|axis| (axis.joint.as_str(), axis))
            .collect();
        let mut values: BTreeMap<_, _> = pose
            .iter()
            .map(|(joint, value)| (joint.as_str(), *value))
            .collect();
        let mut remaining: Vec<_> = self.couplings.iter().collect();
        while !remaining.is_empty() {
            let before = remaining.len();
            let mut unresolved = Vec::new();
            for coupling in remaining {
                let driver = axes[coupling.driver.as_str()];
                if let Some(&input) = values.get(coupling.driver.as_str()) {
                    let x = input - driver.neutral;
                    let dependent = axes[coupling.dependent.as_str()];
                    let value = dependent.neutral
                        + coupling
                            .coefficients
                            .iter()
                            .rev()
                            .fold(0.0, |sum, coefficient| sum * x + coefficient);
                    let [lo, hi] = dependent.range;
                    if !value.is_finite() || !(lo..=hi).contains(&value) {
                        return Err(format!(
                            "поза выходит за диапазон зависимой оси {}",
                            coupling.dependent
                        ));
                    }
                    values.insert(coupling.dependent.as_str(), value);
                } else if driver.dependent {
                    unresolved.push(coupling);
                }
            }
            if unresolved.len() == before {
                break;
            }
            remaining = unresolved;
        }
        Ok(())
    }

    pub fn excludes(&self, first: &str, second: &str) -> bool {
        self.excluded_body_pairs.iter().any(|pair| {
            (pair[0] == first && pair[1] == second) || (pair[0] == second && pair[1] == first)
        })
    }

    /// Пересчитывает известную площадь для геометрически подобной пары.
    /// Подобие обеих сторон и условий вдавливания заверяет вызывающий код.
    /// Оценку применимости и источник измерения передают в оценщик отдельно.
    pub fn area_from_similarity(
        &self,
        region: &RegionId,
        reference_area_m2: Interval,
        pair_similarity: Source<'_>,
    ) -> Result<Option<Interval>, String> {
        if !reference_area_m2.min.is_finite()
            || !reference_area_m2.max.is_finite()
            || reference_area_m2.min <= 0.0
            || reference_area_m2.max < reference_area_m2.min
        {
            return Err("площадь пятна должна задавать конечный положительный интервал".into());
        }
        let Some(s) = self.anthropometry.uniform_geometric_scale else {
            return Ok(None);
        };
        if !self.regions.contains_key(region) || !pair_similarity.known() {
            return Ok(None);
        }
        let area = Interval {
            min: reference_area_m2.min * s * s,
            max: reference_area_m2.max * s * s,
        };
        if area.min <= 0.0 || !area.max.is_finite() {
            return Err("масштаб площади вышел за численный диапазон".into());
        }
        Ok(Some(area))
    }

    /// Прежние расхождения с числами, источниками, режимом и допуском.
    pub fn reference_discrepancies(&self) -> Vec<sim_verify::Check> {
        crate::known_discrepancies()
    }
}
