//! Две сосредоточенные теплоёмкости и отдельная проверка горячего касания.

use sim_env::contact::{BURN_TIMES_S, ContactClass, burn_threshold_c};
use sim_signal::{Rng, Time};
use sim_world::Provenance;

use crate::{Analysis, Interval, ProcessError, SourcedRange, StepModel};

use super::{HazardCheck, math::*};

/// Точный переход между двумя изотермическими телами при постоянной мощности
/// источника и контактной проводимости в пределах шага. Других границ здесь нет.
#[derive(Clone, Debug)]
pub struct HeatSpec {
    pub source_capacity_j_k: SourcedRange,
    pub target_capacity_j_k: SourcedRange,
    pub source_initial_k: SourcedRange,
    pub target_initial_k: SourcedRange,
    pub contact_coefficient_w_m2_k: SourcedRange,
    pub calibrated_area_m2: SourcedRange,
}

#[derive(Clone, Debug, PartialEq)]
pub enum ThermalContact {
    Separated,
    Touching { area_m2: Option<SourcedRange> },
}

#[derive(Clone, Debug, PartialEq)]
pub struct HeatInput {
    pub power_into_source_w: f64,
    /// None означает отсутствие наблюдения контакта.
    pub contact: Option<ThermalContact>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeatStep {
    pub source_k: f64,
    pub target_k: f64,
    pub supplied_j: f64,
    /// Положительно при передаче энергии от источника к цели.
    pub transferred_j: f64,
}

#[derive(Clone, Debug)]
pub struct HeatTransfer {
    spec: HeatSpec,
    temperatures: Option<[f64; 2]>,
    last: Option<Time>,
    diagnostic_area: bool,
}

impl HeatTransfer {
    pub fn new(spec: HeatSpec) -> Result<Self, ProcessError> {
        Self::with_area_policy(spec, false)
    }

    /// Тепловая гипотеза для численного опыта с заданной площадью.
    /// Её результаты могут обнаружить нарушение проектного бюджета.
    /// Физическая опасность остаётся открытой до проверки реальной площади.
    pub fn new_diagnostic(spec: HeatSpec) -> Result<Self, ProcessError> {
        Self::with_area_policy(spec, true)
    }

    fn with_area_policy(spec: HeatSpec, diagnostic_area: bool) -> Result<Self, ProcessError> {
        range(
            &spec.source_capacity_j_k,
            "теплоёмкость источника",
            |x| x > 0.0,
        )?;
        range(&spec.target_capacity_j_k, "теплоёмкость цели", |x| x > 0.0)?;
        let source = range(&spec.source_initial_k, "температура источника", |x| x > 0.0)?;
        let target = range(&spec.target_initial_k, "температура цели", |x| x > 0.0)?;
        range(
            &spec.contact_coefficient_w_m2_k,
            "коэффициент теплопередачи",
            |x| x > 0.0,
        )?;
        range(
            &spec.calibrated_area_m2,
            "область калибровки площади контакта",
            |x| x > 0.0,
        )?;
        Ok(Self {
            spec,
            temperatures: Some([source, target]),
            last: None,
            diagnostic_area,
        })
    }
}

impl StepModel for HeatTransfer {
    type Input = HeatInput;
    type Output = Analysis<HeatStep>;

    fn advance(
        &mut self,
        interval: Interval,
        input: &Self::Input,
        _: &mut Rng,
    ) -> Result<Self::Output, ProcessError> {
        let dt = continuous(self.last, interval)?;
        finite(&[input.power_into_source_w])?;
        let conductance = match &input.contact {
            Some(ThermalContact::Separated) => Some(0.0),
            Some(ThermalContact::Touching {
                area_m2: Some(area),
            }) => {
                range(area, "площадь теплового контакта", |x| x > 0.0)?;
                let bounds = &self.spec.calibrated_area_m2;
                if (self.diagnostic_area || direct(&area.provenance))
                    && (bounds.min..=bounds.max).contains(&area.min)
                    && (bounds.min..=bounds.max).contains(&area.max)
                {
                    let conductance = area.nominal * self.spec.contact_coefficient_w_m2_k.nominal;
                    computed(&[conductance])?;
                    if conductance <= 0.0 {
                        return Err(execution("контактная проводимость не представима"));
                    }
                    Some(conductance)
                } else {
                    None
                }
            }
            _ => None,
        };
        let result = match (self.temperatures, conductance) {
            (Some([source, target]), Some(g)) => {
                let cs = self.spec.source_capacity_j_k.nominal;
                let ct = self.spec.target_capacity_j_k.nominal;
                let power = input.power_into_source_w;
                let supplied_j = power * dt;
                let transferred_j = if g == 0.0 {
                    0.0
                } else {
                    let lambda = g / cs + g / ct;
                    let x = lambda * dt;
                    computed(&[x])?;
                    let (phi, psi) = integral_kernels(x);
                    g * dt * ((source - target) * phi + (supplied_j / cs) * psi)
                };
                let source_k = source + (supplied_j - transferred_j) / cs;
                let target_k = target + transferred_j / ct;
                computed(&[source_k, target_k, supplied_j, transferred_j])?;
                if source_k <= 0.0 || target_k <= 0.0 {
                    return Err(execution(
                        "тепловое состояние вышло из физического диапазона",
                    ));
                }
                self.temperatures = Some([source_k, target_k]);
                calculated(
                    HeatStep {
                        source_k,
                        target_k,
                        supplied_j,
                        transferred_j,
                    },
                    if self.diagnostic_area {
                        "численная гипотеза с заданным пятном и двумя теплоёмкостями, применимость к реальному контакту неизвестна"
                    } else {
                        "номиналы, две изотермические ёмкости, постоянные входы шага, без внешнего теплообмена"
                    },
                )
            }
            _ => {
                self.temperatures = None;
                unknown(
                    "неизвестны история температур, состояние контакта или калиброванная площадь",
                )
            }
        };
        self.last = Some(interval.end);
        Ok(result)
    }
}

/// Среднее exp(-λt) и его интеграл для линейного отклика на постоянный нагрев.
/// При x³ ≤ ε ряд до x² избегает потери цифр в (1 − φ)/x.
fn integral_kernels(x: f64) -> (f64, f64) {
    if x <= f64::EPSILON.cbrt() {
        (1.0 - x / 2.0 + x * x / 6.0, 0.5 - x / 6.0 + x * x / 24.0)
    } else {
        let phi = -(-x).exp_m1() / x;
        (phi, (1.0 - phi) / x)
    }
}

/// Подтверждение применимости сухой кожи к области тела и геометрии контакта.
/// Предел доли площади поступает из оценки площади тела и фактической зоны касания.
#[derive(Clone, Debug, PartialEq)]
pub struct SkinApplicability {
    pub body_region: String,
    pub validated_area_m2: SourcedRange,
    pub body_area_fraction: SourcedRange,
    pub dry_skin_source: Provenance,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BurnExposure {
    /// Границы температуры поверхности за непрерывный интервал касания.
    pub surface_k: SourcedRange,
    pub contact_time: Time,
    pub area_m2: Option<SourcedRange>,
    pub surface_class: Option<(ContactClass, Provenance)>,
    pub dry_skin: Option<SkinApplicability>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BurnResult {
    pub threshold_k: Analysis<f64>,
    pub check: HazardCheck,
}

pub fn assess_burn(input: &BurnExposure) -> Result<BurnResult, ProcessError> {
    range(&input.surface_k, "температура поверхности", |x| x > 0.0)?;
    if let Some(area) = &input.area_m2 {
        range(area, "площадь контакта с тканью", |x| x > 0.0)?;
    }
    if let Some((_, src)) = &input.surface_class {
        source(src)?;
    }
    if let Some(skin) = &input.dry_skin {
        name(&skin.body_region)?;
        source(&skin.dry_skin_source)?;
        range(
            &skin.validated_area_m2,
            "область применимости площади",
            |x| x > 0.0,
        )?;
        range(
            &skin.body_area_fraction,
            "доля поверхности тела",
            |x| x > 0.0 && x <= 1.0,
        )?;
    }
    let unavailable = |reason: &str| BurnResult {
        threshold_k: unknown(reason),
        check: HazardCheck::NotAnalyzed {
            reason: reason.into(),
        },
    };
    let (Some((class, class_source)), Some(skin), Some(area)) =
        (&input.surface_class, &input.dry_skin, &input.area_m2)
    else {
        return Ok(unavailable(
            "нет класса материала, площади контакта или модели ткани",
        ));
    };
    let seconds = input.contact_time.as_secs();
    // Граница площади для длительного касания заимствована из sim-env / CENELEC Guide 29.
    if *class == ContactClass::Tissue
        || !(BURN_TIMES_S[0]..=BURN_TIMES_S[BURN_TIMES_S.len() - 1]).contains(&seconds)
        || skin.body_area_fraction.max >= 0.1
        || area.min < skin.validated_area_m2.min
        || area.max > skin.validated_area_m2.max
    {
        return Ok(unavailable(
            "касание вне области таблицы ожогов или калибровки ткани",
        ));
    }
    let threshold = burn_threshold_c(*class, seconds) + sim_env::air::T0;
    let scope = format!(
        "ISO 13732-1 и EN 13202 через sim-env, сухая кожа, область «{}»",
        skin.body_region
    );
    let evidence = [
        class_source,
        &skin.dry_skin_source,
        &skin.body_area_fraction.provenance,
        &skin.validated_area_m2.provenance,
        &area.provenance,
        &input.surface_k.provenance,
    ]
    .into_iter()
    .all(direct);
    let check = if !evidence {
        HazardCheck::NotAnalyzed {
            reason: "параметры касания оценочные или без цепочки происхождения".into(),
        }
    } else if input.surface_k.min >= threshold {
        HazardCheck::Violation {
            reason: format!("поверхность ≥ {threshold} К при касании {seconds} с"),
        }
    } else if input.surface_k.max < threshold {
        HazardCheck::WithinLimit {
            scope: scope.clone(),
        }
    } else {
        HazardCheck::NotAnalyzed {
            reason: "диапазон температуры пересекает порог ожога".into(),
        }
    };
    Ok(BurnResult {
        threshold_k: calculated(threshold, &scope),
        check,
    })
}
