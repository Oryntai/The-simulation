//! Параметры и измерения контакта. Имена геометрий относятся к собранной сцене.

use sim_signal::{Rate, Time};
use sim_world::ContactStep;

use crate::{Analysis, ProcessError, SourcedRange};

mod creep;
mod table;
pub use creep::{
    CreepConfiguration, CreepMetrics, CreepTolerances, CreepTrace, CreepTrial, CreepVariant,
    HoldSample, LimitComparison, RigidBalance, RigidMode, RigidPlaneSupport,
};
pub use table::{
    ContactMaterialTable, FrictionReference, GeometryMaterial, PairCalibration, ResolvedContactPair,
};

/// Для изотропного скольжения оба коэффициента совпадают. Для анизотропного
/// направления задаются контактным базисом, который нужно зафиксировать в отчёте.
#[derive(Clone, Debug, PartialEq)]
pub enum ContactFriction {
    NormalOnly,
    Sliding {
        sliding: [SourcedRange; 2],
    },
    SlidingAndTorsion {
        sliding: [SourcedRange; 2],
        torsion_m: SourcedRange,
    },
    SlidingTorsionAndRolling {
        sliding: [SourcedRange; 2],
        torsion_m: SourcedRange,
        rolling_m: [SourcedRange; 2],
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct PairParameters {
    pub friction: ContactFriction,
    pub normal: NormalResponse,
}

/// Выбор закона и его перевод в параметры движка входят в область применимости.
#[derive(Clone, Debug, PartialEq)]
pub enum NormalResponse {
    Restitution {
        coefficient: SourcedRange,
        duration_s: SourcedRange,
    },
    SpringDamper {
        stiffness_n_m: SourcedRange,
        damping_ns_m: SourcedRange,
    },
    /// Явный прямой solref MuJoCo. Область физической применимости
    /// устанавливается отдельным опытом.
    Direct {
        stiffness_per_s2: SourcedRange,
        damping_per_s: SourcedRange,
    },
}

/// Пара материалов неупорядочена. Перестановка имён сохраняет параметры.
/// Отсутствующую пару следует возвращать с причиной.
pub trait ContactTable {
    fn lookup(&self, materials: [&str; 2]) -> Result<Analysis<&PairParameters>, ProcessError>;
}

/// Сила и момент, действующие на второй геом со стороны первого в точке контакта.
/// Все векторы заданы в системе мира, нормаль направлена от первого геома ко второму.
#[derive(Clone, Debug, PartialEq)]
pub struct ContactSample {
    pub geometries: [String; 2],
    pub point_m: [f64; 3],
    pub normal: [f64; 3],
    pub separation_m: f64,
    pub force_n: [f64; 3],
    pub torque_nm: [f64; 3],
}

#[derive(Clone, Debug, PartialEq)]
pub struct ContactFrame {
    /// Правая граница физического интервала, на котором действовали силы.
    pub at: Time,
    pub contacts: Vec<ContactSample>,
}

impl ContactFrame {
    /// Переносит реакции Euler на целочисленную шкалу рантайма. Базис и
    /// параметры движка доступны адаптеру в исходном ContactStep.
    pub fn from_engine_step(
        interval: crate::Interval,
        step: &ContactStep,
    ) -> Result<Self, ProcessError> {
        let duration = step.end_s - step.start_s;
        let span = interval
            .end
            .0
            .checked_sub(interval.start.0)
            .filter(|&ns| ns > 0);
        // Rate округляет границы к целым нс. Вычитание времён MuJoCo даёт
        // дополнительную ошибку округления до двух машинных единиц масштаба.
        let rounding_s = Time(1).as_secs() + 2.0 * f64::EPSILON * step.end_s.abs();
        if step.start_s < 0.0
            || !duration.is_finite()
            || duration <= 0.0
            || span.is_none_or(|ns| (Time(ns).as_secs() - duration).abs() > rounding_s)
        {
            return Err(table::invalid(
                "реакции не соответствуют физическому интервалу",
            ));
        }
        // Время MuJoCo накапливается суммированием. Предельная погрешность
        // оценивается по числу сложений и машинному масштабу времени.
        let accumulation_s = f64::EPSILON * step.end_s * (step.end_s / duration).ceil();
        let alignment_s = rounding_s + accumulation_s;
        if accumulation_s >= duration / 2.0
            || (step.start_s - interval.start.as_secs()).abs() > alignment_s
            || (step.end_s - interval.end.as_secs()).abs() > alignment_s
        {
            return Err(table::invalid(
                "начало или конец решения не совпадает со шкалой прогона",
            ));
        }
        let mut contacts = Vec::with_capacity(step.contacts.len());
        for c in &step.contacts {
            if !c.geometries.iter().all(|s| table::name(s))
                || !matches!(c.dim, 1 | 3 | 4 | 6)
                || !c
                    .point_m
                    .iter()
                    .chain(c.frame.iter().flatten())
                    .chain(&c.force_n)
                    .chain(&c.torque_nm)
                    .chain(&c.friction)
                    .chain(&c.solref)
                    .chain(&c.solreffriction)
                    .chain(&c.solimp)
                    .chain([&c.separation_m, &c.margin_m, &c.adhesion_n])
                    .all(|x| x.is_finite())
            {
                return Err(table::invalid("недопустимая контактная реакция"));
            }
            contacts.push(ContactSample {
                geometries: c.geometries.clone(),
                point_m: c.point_m,
                normal: c.frame[0],
                separation_m: c.separation_m,
                force_n: c.force_n,
                torque_nm: c.torque_nm,
            });
        }
        Ok(Self {
            at: interval.end,
            contacts,
        })
    }
}

/// Смещение в касательной плоскости относительно начала выдержки.
/// Численная составляющая оценивается отдельно для заданной области применимости.
#[derive(Clone, Debug, PartialEq)]
pub struct CreepReport {
    pub hold: crate::Interval,
    pub drift_m: f64,
    pub peak_drift_m: f64,
    numerical_uncertainty_m: Analysis<f64>,
    tolerances: CreepTolerances,
    trials: std::collections::BTreeMap<CreepVariant, CreepTrial>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Convergence {
    WithinTolerance,
    OutsideTolerance,
    NotAnalyzed { reason: String },
}

/// Значения метрики на сетках в порядке уменьшения шага. Допуск имеет единицы метрики.
#[derive(Clone, Debug, PartialEq)]
pub struct StepComparison {
    pub metric: String,
    pub unit: String,
    pub values: Vec<(Rate, f64)>,
    pub absolute_tolerance: SourcedRange,
    pub outcome: Convergence,
}
