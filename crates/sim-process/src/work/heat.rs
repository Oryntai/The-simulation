//! Сосредоточенные теплоёмкости и отдельная проверка горячего касания.

use sim_env::contact::{BURN_TIMES_S, ContactClass, burn_threshold_c};
use sim_signal::{Rng, Time};
use sim_world::Provenance;

use crate::{Analysis, Interval, ProcessError, SourcedRange, StepModel};

use super::{HazardCheck, math::*};

#[path = "solder_network.rs"]
pub mod solder_network;

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

/// Тепловой переход при постоянных проводимостях и мощности. В столбце
/// перехода хранится распределение единицы энергии узла, в векторе подачи —
/// распределение энергии, равномерно поданной за интервал. Граница с заданной
/// температурой считается накопителем знаковой энергии относительно неё.
struct ThermalResponse<const N: usize> {
    transition: [[f64; N]; N],
    supply: [f64; N],
}

impl<const N: usize> ThermalResponse<N> {
    /// rates[приёмник][донор] = G/C_донора, диагональ нулевая. Для граничного
    /// накопителя исходящих скоростей нет. Решается dE/dt = Q·E + P, где
    /// Q = rates − diag(сумма исходящих скоростей каждого узла).
    fn new(rates: &[[f64; N]; N], source: usize, mut dt: f64) -> Result<Self, ProcessError> {
        let out: [f64; N] = std::array::from_fn(|j| rates.iter().map(|row| row[j]).sum());
        computed(&out)?;
        let rate = out.into_iter().fold(0.0, f64::max);
        let mut doublings = 0;
        while rate * dt > 1.0 {
            dt *= 0.5;
            doublings += 1;
        }
        // Сдвиг Q·dt + xI при x = max(−Qii)·dt делает ряд экспоненты
        // неотрицательным. Множитель exp(−x) общий, его даёт нормировка.
        let x = rate * dt;
        let mut shifted = rates.map(|row| row.map(|r| r * dt));
        for (i, row) in shifted.iter_mut().enumerate() {
            row[i] = x - out[i] * dt;
            if rates[i]
                .iter()
                .zip(row)
                .enumerate()
                .any(|(j, (&r, s))| i != j && r > 0.0 && *s == 0.0)
            {
                return Err(execution("доля теплового перехода не представима"));
            }
        }

        let identity = std::array::from_fn(|i| std::array::from_fn(|j| f64::from(i == j)));
        let mut result = Self {
            transition: identity,
            supply: [0.0; N],
        };
        let mut term = identity;
        let mut feed = [0.0; N];
        let mut constant = 1.0;
        // Последний столбец расширенной матрицы добавляет интеграл экспоненты:
        // его источник постоянен и после сдвига имеет диагональ x. При x ≤ 1
        // хвост после N + 19 членов меньше ε даже с запасом N на путь через сеть.
        // 1/19! ≈ 8,22·10⁻¹⁸, последующие члены убывают быстрее геометрического ряда.
        for k in 1..=N + 19 {
            let k = k as f64;
            term = term.map(|column| mul_thermal(&shifted, column).map(|value| value / k));
            feed = mul_thermal(&shifted, feed);
            feed[source] += constant;
            feed = feed.map(|value| value / k);
            constant *= x / k;
            for (sum, add) in result.transition.iter_mut().zip(term) {
                for (sum, add) in sum.iter_mut().zip(add) {
                    *sum += add;
                }
            }
            for (sum, add) in result.supply.iter_mut().zip(feed) {
                *sum += add;
            }
        }
        result.normalize();
        for _ in 0..doublings {
            // Первая половина поданной энергии проходит второй переход, вторая
            // добавляется напрямую. Нормировка не даёт ошибке суммы расти при удвоении.
            let first_half = result.distribute(result.supply);
            for (weight, from_first) in result.supply.iter_mut().zip(first_half) {
                *weight = (*weight + from_first) * 0.5;
            }
            result.transition = result.transition.map(|column| result.distribute(column));
            result.normalize();
        }
        Ok(result)
    }

    fn normalize(&mut self) {
        for weights in self.transition.iter_mut().chain([&mut self.supply]) {
            let sum: f64 = weights.iter().sum();
            weights.iter_mut().for_each(|value| *value /= sum);
        }
    }

    fn distribute(&self, energy: [f64; N]) -> [f64; N] {
        std::array::from_fn(|i| {
            self.transition
                .iter()
                .zip(energy)
                .map(|(col, e)| col[i] * e)
                .sum()
        })
    }

    fn apply(&self, energy: [f64; N], supplied_j: f64) -> [f64; N] {
        let redistributed = self.distribute(energy);
        std::array::from_fn(|i| redistributed[i] + self.supply[i] * supplied_j)
    }
}

fn mul_thermal<const N: usize>(rates: &[[f64; N]; N], column: [f64; N]) -> [f64; N] {
    rates.map(|row| row.into_iter().zip(column).map(|(a, b)| a * b).sum())
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
