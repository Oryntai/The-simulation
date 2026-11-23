//! Паспортное значение с происхождением: у каждого числа каталога есть источник,
//! а чего производитель не публикует — то `None`, а не догадка.
//!
//! Правило для вердикта: значение с происхождением [`Provenance::Estimate`] может
//! доказать «не годится», но не может доказать «годится» — такая проверка получает
//! `NotAnalyzed`, а не `Pass`.

use std::f64::consts::PI;

/// Перевод об/мин → рад/с.
pub const RPM: f64 = 2.0 * PI / 60.0;
/// 1 кгс·см в Н·м (стандартное ускорение 9.80665 м/с²).
pub const KGF_CM: f64 = 0.098_066_5;
/// Температурный коэффициент сопротивления меди, 1/К (IACS, как в `sim-em`).
pub const ALPHA_CU: f64 = 0.003_93;
/// Температура, к которой приведены паспортные сопротивления, °C.
pub const T_REF_C: f64 = 25.0;
/// Падение K_t от нагрева магнитов NdFeB, 1/К (Arnold, −0.12 %/°C, как в `sim-em`).
pub const NDFEB_KT_COEFF_PER_K: f64 = 0.001_2;
/// Падение K_t для ферритовых или неизвестных магнитов, 1/К — «оценка»
/// (справочное значение для феррита −0.2 %/°C, берём его как худшее).
pub const FERRITE_KT_COEFF_PER_K: f64 = 0.002;
/// Температура обмотки, если тепловых данных нет, °C — «оценка»: считаем привод
/// уже прогретым, чтобы не обещать момент холодной обмотки.
pub const UNKNOWN_HOT_WINDING_C: f64 = 80.0;

/// Откуда взято число.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Provenance {
    /// Документ производителя.
    Datasheet,
    /// Страница или листок продавца: для дешёвых деталей часто завышено.
    Vendor,
    /// Независимый замер.
    Measured,
    /// Вычислено из других паспортных чисел (формула — в `source`).
    Derived,
    /// Оценка без прямого источника, взята в опасную для вывода сторону.
    Estimate,
    /// Не опубликовано.
    Missing,
}

impl Provenance {
    /// Короткая подпись для интерфейса.
    pub fn label(self) -> &'static str {
        match self {
            Provenance::Datasheet => "паспорт",
            Provenance::Vendor => "продавец",
            Provenance::Measured => "замер",
            Provenance::Derived => "расчёт из паспорта",
            Provenance::Estimate => "оценка",
            Provenance::Missing => "нет данных",
        }
    }
}

/// Число каталога с происхождением.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Spec {
    pub value: Option<f64>,
    pub provenance: Provenance,
    pub source: &'static str,
}

impl Spec {
    /// Производитель значение не публикует.
    pub const NONE: Spec = Spec {
        value: None,
        provenance: Provenance::Missing,
        source: "",
    };

    pub const fn ds(value: f64, source: &'static str) -> Self {
        Self {
            value: Some(value),
            provenance: Provenance::Datasheet,
            source,
        }
    }
    pub const fn vendor(value: f64, source: &'static str) -> Self {
        Self {
            value: Some(value),
            provenance: Provenance::Vendor,
            source,
        }
    }
    pub const fn measured(value: f64, source: &'static str) -> Self {
        Self {
            value: Some(value),
            provenance: Provenance::Measured,
            source,
        }
    }
    pub const fn derived(value: f64, source: &'static str) -> Self {
        Self {
            value: Some(value),
            provenance: Provenance::Derived,
            source,
        }
    }
    pub const fn est(value: f64, source: &'static str) -> Self {
        Self {
            value: Some(value),
            provenance: Provenance::Estimate,
            source,
        }
    }

    /// Значение, если оно есть и конечно.
    pub fn get(self) -> Option<f64> {
        self.value.filter(|v| v.is_finite())
    }

    /// Можно ли опираться на это число как на предел для «годится».
    pub fn is_proof(self) -> bool {
        self.get().is_some()
            && !self.source.trim().is_empty()
            && !matches!(
                self.provenance,
                Provenance::Estimate | Provenance::Vendor | Provenance::Missing
            )
    }

    /// Значение задано, но это NaN или бесконечность.
    pub fn is_broken(self) -> bool {
        self.value.is_some_and(|v| !v.is_finite())
    }

    /// Подпись «значение (происхождение: источник)» для пояснений.
    pub fn cite(self) -> String {
        match self.value {
            Some(_) if self.source.is_empty() => self.provenance.label().to_string(),
            Some(_) => format!("{}: {}", self.provenance.label(), self.source),
            None => "не опубликовано".to_string(),
        }
    }
}

impl Default for Spec {
    fn default() -> Self {
        Spec::NONE
    }
}
