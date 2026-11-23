//! Пределы теплового и холодового напряжения организма: температура ядра, обезвоживание.
//! Используются приложением для предупреждений и моделями (двухузловая, погружение).

/// Нормальная температура ядра в покое, °C (ориентир; суточные колебания ±0.5 °C).
pub const CORE_NORMAL_C: f64 = 37.0;

/// Предел температуры ядра для неакклиматизированных работников, °C: 38.0.
/// Источники: ISO 7933:2004 (PHS, предел ректальной температуры 38 °C);
/// ACGIH TLV «Heat Stress and Strain» (38 °C для неакклиматизированных);
/// WHO Technical Report Series No. 412 (1969).
pub const CORE_LIMIT_UNACCLIMATISED_C: f64 = 38.0;

/// Предел температуры ядра для акклиматизированных, медицински отобранных работников, °C:
/// 38.5. Источник: ACGIH TLV «Heat Stress and Strain» (Documentation).
pub const CORE_LIMIT_ACCLIMATISED_C: f64 = 38.5;

/// Тепловой удар: ядро > 40 °C с нарушениями ЦНС, °C.
/// Источник: Bouchama A., Knochel J.P. (2002) N. Engl. J. Med. 346:1978–1988,
/// doi:10.1056/NEJMra011089.
pub const HEAT_STROKE_C: f64 = 40.0;

/// Гипотермия — ядро ниже 35 °C. Стадии: лёгкая 32–35, умеренная 28–32, тяжёлая < 28 °C.
/// Источник: Brown D.J.A., Brugger H., Boyd J., Paal P. (2012) N. Engl. J. Med.
/// 367:1930–1938, doi:10.1056/NEJMra1114208.
pub const HYPOTHERMIA_C: f64 = 35.0;
/// Граница умеренной гипотермии, °C.
pub const HYPOTHERMIA_MODERATE_C: f64 = 32.0;
/// Граница тяжёлой гипотермии, °C.
pub const HYPOTHERMIA_SEVERE_C: f64 = 28.0;
/// Ниже ≈ 24 °C в среднем наступает остановка сердца («ориентир»: Transport Canada
/// TP 13822 «Survival in Cold Waters», 2003, гл. 1).
pub const CARDIAC_ARREST_RISK_C: f64 = 24.0;
/// Ядро около 30 °C — типичный порог потери сознания; до этой температуры считает
/// уравнение Hayward et al. (1975) (см. [`crate::immersion`]).
pub const UNCONSCIOUSNESS_C: f64 = 30.0;

/// Обезвоживание 2 % массы тела ухудшает аэробную работоспособность и терморегуляцию.
/// Источник: Sawka M.N. et al. (2007) ACSM Position Stand, Med. Sci. Sports Exerc.
/// 39:377–390, doi:10.1249/mss.0b013e31802ca597.
pub const DEHYDRATION_PERFORMANCE_FRACTION: f64 = 0.02;
/// Предел потери воды ISO 7933:2004 для защиты 95 % работающих (свободное питьё): 5 % массы.
pub const DEHYDRATION_LIMIT_95_FRACTION: f64 = 0.05;
/// Предел потери воды ISO 7933:2004 для среднего человека: 7.5 % массы.
pub const DEHYDRATION_LIMIT_50_FRACTION: f64 = 0.075;
/// Предел потери воды ISO 7933 без возможности пить: 3 % массы (так же в ISO 7933:2023 —
/// 3 % без питья, 5 % с питьём; по коду PHS pythermalcomfort).
pub const DEHYDRATION_LIMIT_NO_DRINK_FRACTION: f64 = 0.03;

/// Состояние по температуре ядра.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum CoreStatus {
    /// Ниже 28 °C.
    SevereHypothermia,
    /// 28–32 °C.
    ModerateHypothermia,
    /// 32–35 °C.
    MildHypothermia,
    /// 35–38 °C.
    Normal,
    /// 38–38.5 °C: превышен предел для неакклиматизированных.
    AboveUnacclimatisedLimit,
    /// 38.5–40 °C: превышен предел для акклиматизированных — прекращать работу.
    AboveAcclimatisedLimit,
    /// > 40 °C: риск теплового удара.
    HeatStroke,
}

/// Классификация температуры ядра.
pub fn core_status(t_core_c: f64) -> CoreStatus {
    if t_core_c < HYPOTHERMIA_SEVERE_C {
        CoreStatus::SevereHypothermia
    } else if t_core_c < HYPOTHERMIA_MODERATE_C {
        CoreStatus::ModerateHypothermia
    } else if t_core_c < HYPOTHERMIA_C {
        CoreStatus::MildHypothermia
    } else if t_core_c <= CORE_LIMIT_UNACCLIMATISED_C {
        CoreStatus::Normal
    } else if t_core_c <= CORE_LIMIT_ACCLIMATISED_C {
        CoreStatus::AboveUnacclimatisedLimit
    } else if t_core_c <= HEAT_STROKE_C {
        CoreStatus::AboveAcclimatisedLimit
    } else {
        CoreStatus::HeatStroke
    }
}

/// Состояние по обезвоживанию.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum HydrationStatus {
    /// < 2 % массы.
    Ok,
    /// 2–3 %: снижение работоспособности.
    Impaired,
    /// 3–5 %: выше предела ISO 7933 без питья.
    AboveNoDrinkLimit,
    /// > 5 %: выше предела ISO 7933 для 95 % работающих.
    AboveLimit,
}

/// Классификация потери воды `water_loss_kg` при массе тела `body_mass_kg`.
pub fn hydration_status(water_loss_kg: f64, body_mass_kg: f64) -> HydrationStatus {
    let f = water_loss_kg / body_mass_kg;
    if f < DEHYDRATION_PERFORMANCE_FRACTION {
        HydrationStatus::Ok
    } else if f < DEHYDRATION_LIMIT_NO_DRINK_FRACTION {
        HydrationStatus::Impaired
    } else if f < DEHYDRATION_LIMIT_95_FRACTION {
        HydrationStatus::AboveNoDrinkLimit
    } else {
        HydrationStatus::AboveLimit
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classification() {
        assert_eq!(core_status(37.0), CoreStatus::Normal);
        assert_eq!(core_status(34.0), CoreStatus::MildHypothermia);
        assert_eq!(core_status(40.5), CoreStatus::HeatStroke);
        assert_eq!(hydration_status(1.0, 70.0), HydrationStatus::Ok);
        assert_eq!(hydration_status(4.0, 70.0), HydrationStatus::AboveLimit);
    }
}
