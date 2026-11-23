//! Действие холода и жары на тело и на материалы прибора: мышцы, ловкость рук, нагрев
//! поверхностей солнцем, пределы материалов, батарей и электроники, защита IP от воды.
//!
//! Это литературные константы с источником и степенью уверенности, а не модели первого
//! принципа: приложение по ним поднимает предупреждения. «оценка» — наша оценка.

use crate::air::{T0, bisect};
use crate::convection::{self, Medium, STEFAN_BOLTZMANN};

/// Уверенность в литературном значении.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Confidence {
    /// Стандарт или несколько согласованных исследований.
    High,
    /// Одно-два исследования или большой разброс.
    Medium,
    /// Наша оценка / паспортные «типичные» данные.
    Low,
}

/// Число с диапазоном и источником.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Estimate {
    /// Центральное значение.
    pub value: f64,
    /// Нижняя граница разброса.
    pub low: f64,
    /// Верхняя граница разброса.
    pub high: f64,
    /// Уверенность.
    pub confidence: Confidence,
    /// Источник.
    pub source: &'static str,
}

// ---------------------------------------------------------------------------
// Мышцы
// ---------------------------------------------------------------------------

/// Изменение максимальной мощности и динамической силы мышц на 1 °C температуры мышцы,
/// доля/°C: 0.04 (0.02–0.06). Изометрическая сила при 30–39 °C почти не меняется.
pub const MUSCLE_POWER_PER_C: Estimate = Estimate {
    value: 0.04,
    low: 0.02,
    high: 0.06,
    confidence: Confidence::Medium,
    source: "Bergh U., Ekblom B. (1979) Acta Physiol. Scand. 107:33–37, doi:10.1111/j.1748-1716.1979.tb06439.x \
             (4–6 %/°C, T_мышцы 30–39 °C); Sargeant A.J. (1987) Eur. J. Appl. Physiol. 56:693–698, \
             doi:10.1007/BF00424812 (≈3 %/°C); обзор Racinais S., Oksa J. (2010) Scand. J. Med. Sci. Sports \
             20(S3):1–18, doi:10.1111/j.1600-0838.2010.01204.x",
};

/// Температура мышцы, к которой отнесена мощность 1.0, °C (после разминки ≈ 37–38 °C).
pub const MUSCLE_REFERENCE_C: f64 = 37.0;
/// Диапазон температур мышц, в котором измерена зависимость, °C.
pub const MUSCLE_DATA_RANGE_C: (f64, f64) = (30.0, 39.0);

/// Относительная мощность мышц при температуре мышцы `t_muscle_c` (1.0 при 37 °C):
/// (центр, нижняя, верхняя оценка, внутри ли диапазона данных).
pub fn muscle_power_factor(t_muscle_c: f64) -> (f64, f64, f64, bool) {
    let d = t_muscle_c - MUSCLE_REFERENCE_C;
    let f = |s: f64| (1.0 + s * d).max(0.0);
    let (a, b) = (f(MUSCLE_POWER_PER_C.low), f(MUSCLE_POWER_PER_C.high));
    let in_range = (MUSCLE_DATA_RANGE_C.0..=MUSCLE_DATA_RANGE_C.1).contains(&t_muscle_c);
    (f(MUSCLE_POWER_PER_C.value), a.min(b), a.max(b), in_range)
}

// ---------------------------------------------------------------------------
// Руки в холоде
// ---------------------------------------------------------------------------

/// Состояние рук по температуре кожи пальцев.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum HandFunction {
    /// Норма.
    Normal,
    /// Ниже 15 °C: снижение ловкости (мелкая моторика), начало холодовой боли.
    ReducedDexterity,
    /// Ниже 8 °C: резкое падение тактильной чувствительности.
    ReducedTactile,
    /// Ниже 7 °C: онемение (у трети людей уже при 7 °C).
    Numb,
    /// Ниже 0 °C: риск замерзания кожи (сама кожа замерзает около −0.6 °C, с переохлаждением
    /// ниже; риск 5→95 % при −4.8…−7.8 °C).
    FreezingRisk,
}

/// Порог снижения ловкости, °C (кожа пальцев/кисти 12–16 °C → берём 15 °C).
/// Источник: Heus R., Daanen H.A.M., Havenith G. (1995) Appl. Ergon. 26:5–13,
/// doi:10.1016/0003-6870(94)00004-I.
pub const DEXTERITY_LOSS_C: f64 = 15.0;
/// Порог потери тактильной чувствительности, °C. Источник: Morton R., Provins K.A. (1960)
/// J. Appl. Physiol. 15:149–154; цит. по Heus et al. 1995 и Geng et al. 2006.
pub const TACTILE_LOSS_C: f64 = 8.0;
/// Порог онемения, °C (Provins & Morton 1960; критерий ISO 13732-3).
pub const NUMBNESS_C: f64 = 7.0;
/// Порог риска замерзания кожи, °C (ISO 13732-3; Keatinge & Evans 1960 — точка замерзания
/// кожи пальца −0.6 °C; Danielsson 1996 J. Appl. Physiol. 81:2666 — риск 5–95 % при −4.8…−7.8 °C).
pub const SKIN_FREEZING_C: f64 = 0.0;

/// Состояние рук по температуре кожи пальцев, °C.
pub fn hand_function(finger_skin_c: f64) -> HandFunction {
    if finger_skin_c <= SKIN_FREEZING_C {
        HandFunction::FreezingRisk
    } else if finger_skin_c <= NUMBNESS_C {
        HandFunction::Numb
    } else if finger_skin_c <= TACTILE_LOSS_C {
        HandFunction::ReducedTactile
    } else if finger_skin_c <= DEXTERITY_LOSS_C {
        HandFunction::ReducedDexterity
    } else {
        HandFunction::Normal
    }
}

// ---------------------------------------------------------------------------
// Нагрев поверхностей солнцем
// ---------------------------------------------------------------------------

/// Оптические свойства поверхности: поглощательная способность для солнца α_s и
/// излучательная способность в ИК ε.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceOptics {
    /// Название.
    pub name: &'static str,
    /// Поглощение солнечного излучения.
    pub absorptivity: f64,
    /// Излучательная способность (длинноволновая).
    pub emissivity: f64,
}

const GILMORE: &str = "Gilmore D.G. (ed.) Spacecraft Thermal Control Handbook, Vol. 1, 2nd ed. (2002), прил. A; разброс ±0.05";

/// Типичные покрытия (источник: Gilmore 2002, прил. A; кожа — «оценка» по литературе
/// о солнечной нагрузке на человека, 0.6–0.8 в зависимости от пигментации).
pub const SURFACES: [(SurfaceOptics, &str); 7] = [
    (
        SurfaceOptics {
            name: "алюминий, чёрное анодирование",
            absorptivity: 0.88,
            emissivity: 0.88,
        },
        GILMORE,
    ),
    (
        SurfaceOptics {
            name: "алюминий, бесцветное анодирование",
            absorptivity: 0.35,
            emissivity: 0.80,
        },
        GILMORE,
    ),
    (
        SurfaceOptics {
            name: "алюминий полированный",
            absorptivity: 0.15,
            emissivity: 0.05,
        },
        GILMORE,
    ),
    (
        SurfaceOptics {
            name: "белая краска",
            absorptivity: 0.25,
            emissivity: 0.90,
        },
        GILMORE,
    ),
    (
        SurfaceOptics {
            name: "чёрная краска",
            absorptivity: 0.95,
            emissivity: 0.90,
        },
        GILMORE,
    ),
    (
        SurfaceOptics {
            name: "чёрный пластик (PLA/ABS)",
            absorptivity: 0.92,
            emissivity: 0.92,
        },
        "оценка",
    ),
    (
        SurfaceOptics {
            name: "кожа человека",
            absorptivity: 0.70,
            emissivity: 0.98,
        },
        "оценка",
    ),
];

/// Эффективная температура ясного неба для длинноволнового излучения, °C:
/// T_sky = 0.0552·T_a^1.5 (К). Источник: Swinbank W.C. (1963) Q. J. R. Meteorol. Soc.
/// 89:339–348, doi:10.1002/qj.49708938105.
pub fn sky_temperature_swinbank_c(t_air_c: f64) -> f64 {
    0.0552 * (t_air_c + T0).powf(1.5) - T0
}

/// Стационарная температура поверхности на солнце, °C: тыльная сторона теплоизолирована,
/// баланс α_s·G = h_c·(T − T_a) + ε·σ·[T⁴ − F_sky·T_sky⁴ − (1 − F_sky)·T_gnd⁴].
/// `g_w_m2` — облучённость по нормали к поверхности, `view_sky` — доля неба в поле зрения
/// (1 — горизонтальная, 0.5 — вертикальная), `t_ground_c` — температура земли/окружения.
/// Решение — бисекцией (без линеаризации излучения).
pub fn solar_surface_temperature(
    surface: &SurfaceOptics,
    g_w_m2: f64,
    t_air_c: f64,
    h_conv: f64,
    view_sky: f64,
    t_ground_c: f64,
) -> f64 {
    let t_sky = sky_temperature_swinbank_c(t_air_c) + T0;
    let t_gnd = t_ground_c + T0;
    let incoming_lw = view_sky * t_sky.powi(4) + (1.0 - view_sky) * t_gnd.powi(4);
    let f = |t_c: f64| {
        let t = t_c + T0;
        surface.absorptivity * g_w_m2
            - h_conv * (t_c - t_air_c)
            - surface.emissivity * STEFAN_BOLTZMANN * (t.powi(4) - incoming_lw)
    };
    bisect(f, t_air_c - 80.0, t_air_c + 300.0, 1e-9)
}

/// Температура детали на солнце на руке: пластина длиной `length` вдоль ветра `wind`, м/с;
/// h_c — по [`convection::h_flat_plate`] (вынужденная) или свободной конвекции, что больше.
pub fn solar_device_temperature(
    surface: &SurfaceOptics,
    g_w_m2: f64,
    t_air_c: f64,
    wind: f64,
    length: f64,
) -> f64 {
    // h зависит от T поверхности слабо — две итерации
    let air = Medium::air(40.0);
    let mut t = t_air_c + 20.0;
    for _ in 0..4 {
        let hf = convection::h_flat_plate(&air, length, wind.max(0.05), t, t_air_c);
        let hn = convection::h_vertical_natural(&air, length, t, t_air_c);
        t = solar_surface_temperature(surface, g_w_m2, t_air_c, hf.max(hn), 0.5, t_air_c);
    }
    t
}

/// Температура sol-air по ASHRAE (линеаризованная), °C:
/// T_sol = T_a + α·G/h_o − ε·ΔR/h_o; для горизонтальной поверхности ΔR = 63 Вт/м²,
/// для вертикальной — 0; h_o ≈ 17 Вт/(м²·К) (лето). Второй способ для
/// [`solar_surface_temperature`]. Источник: ASHRAE Handbook — Fundamentals (2017), гл. 18,
/// «Sol-air temperature».
pub fn sol_air_temperature(
    surface: &SurfaceOptics,
    g_w_m2: f64,
    t_air_c: f64,
    horizontal: bool,
    h_o: f64,
) -> f64 {
    let dr = if horizontal { 63.0 } else { 0.0 };
    t_air_c + surface.absorptivity * g_w_m2 / h_o - surface.emissivity * dr / h_o
}

// ---------------------------------------------------------------------------
// Пределы материалов, батарей, электроники
// ---------------------------------------------------------------------------

/// Температурные пределы материала конструкции.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MaterialLimits {
    /// Название.
    pub name: &'static str,
    /// Температура стеклования (диапазон), °C; `None` — неприменимо.
    pub glass_transition_c: Option<(f64, f64)>,
    /// Рекомендуемый максимум для нагруженной детали, °C («оценка»: ниже Tg / HDT с запасом).
    pub max_service_c: f64,
    /// Ниже этой температуры материал заметно охрупчивается, °C («оценка»).
    pub min_service_c: f64,
    /// Источник.
    pub source: &'static str,
    /// Уверенность.
    pub confidence: Confidence,
}

/// Пределы материалов мастерской.
pub const MATERIAL_LIMITS: [MaterialLimits; 9] = [
    MaterialLimits {
        name: "PLA",
        glass_transition_c: Some((55.0, 65.0)),
        max_service_c: 50.0,
        min_service_c: -10.0,
        source: "Farah S. et al. (2016) Adv. Drug Deliv. Rev. 107:367, doi:10.1016/j.addr.2016.06.012 (Tg 55–65 °C); пределы службы — оценка",
        confidence: Confidence::Medium,
    },
    MaterialLimits {
        name: "PETG",
        glass_transition_c: Some((78.0, 82.0)),
        max_service_c: 65.0,
        min_service_c: -20.0,
        source: "паспорта Eastman Eastar (Tg ≈ 80 °C); пределы службы — оценка",
        confidence: Confidence::Medium,
    },
    MaterialLimits {
        name: "ABS",
        glass_transition_c: Some((100.0, 110.0)),
        max_service_c: 85.0,
        min_service_c: -20.0,
        source: "типичные паспорта ABS (Tg ≈ 105 °C); пределы службы — оценка",
        confidence: Confidence::Medium,
    },
    MaterialLimits {
        name: "PA12",
        glass_transition_c: Some((40.0, 50.0)),
        max_service_c: 100.0,
        min_service_c: -40.0,
        source: "паспорта PA12 (Tg ≈ 40–50 °C, плавление ≈ 178 °C; выше Tg модуль падает, но деталь держит форму); оценка",
        confidence: Confidence::Low,
    },
    MaterialLimits {
        name: "TPU",
        glass_transition_c: None,
        max_service_c: 80.0,
        min_service_c: -40.0,
        source: "оценка по паспортам TPU",
        confidence: Confidence::Low,
    },
    MaterialLimits {
        name: "силикон",
        glass_transition_c: None,
        max_service_c: 200.0,
        min_service_c: -50.0,
        source: "оценка по паспортам силиконовых резин",
        confidence: Confidence::Low,
    },
    MaterialLimits {
        name: "неопрен",
        glass_transition_c: None,
        max_service_c: 100.0,
        min_service_c: -35.0,
        source: "оценка по паспортам CR-резин",
        confidence: Confidence::Low,
    },
    MaterialLimits {
        name: "углепластик (эпоксидная матрица)",
        glass_transition_c: Some((60.0, 180.0)),
        max_service_c: 60.0,
        min_service_c: -50.0,
        source: "Tg зависит от режима отверждения (холодное ≈ 60–80 °C, автоклав 120–180 °C); оценка",
        confidence: Confidence::Low,
    },
    MaterialLimits {
        name: "алюминий 6061-T6",
        glass_transition_c: None,
        max_service_c: 150.0,
        min_service_c: -200.0,
        source: "ASM: выше ≈150 °C при длительной выдержке перестаривание и потеря прочности; оценка",
        confidence: Confidence::Medium,
    },
];

/// Пределы литий-ионного аккумулятора (типичная ячейка 18650/21700), °C.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BatteryLimits {
    /// Заряд разрешён в диапазоне, °C.
    pub charge_c: (f64, f64),
    /// Разряд разрешён в диапазоне, °C.
    pub discharge_c: (f64, f64),
    /// Начало опасного саморазогрева (разложение SEI), °C.
    pub self_heating_onset_c: f64,
    /// Типичное начало теплового разгона, °C.
    pub thermal_runaway_c: f64,
    /// Источник.
    pub source: &'static str,
}

/// Типичные пределы Li-ion. Заряд ниже 0 °C — осаждение металлического лития на аноде
/// (Waldmann T. et al. (2014) J. Power Sources 262:129–135, doi:10.1016/j.jpowsour.2014.03.112).
pub const LI_ION: BatteryLimits = BatteryLimits {
    charge_c: (0.0, 45.0),
    discharge_c: (-20.0, 60.0),
    self_heating_onset_c: 80.0,
    thermal_runaway_c: 150.0,
    source: "паспорт Panasonic NCR18650B (заряд 0…45, разряд −20…60 °C); саморазогрев/разгон — \
             Feng X. et al. (2018) Energy Storage Mater. 10:246–267, doi:10.1016/j.ensm.2017.05.013 (порядок величин)",
};

/// Класс температурного исполнения электроники, °C.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ElectronicsGrade {
    /// Название.
    pub name: &'static str,
    /// Диапазон окружающей температуры, °C.
    pub range_c: (f64, f64),
    /// Источник.
    pub source: &'static str,
}

/// Классы исполнения микросхем.
pub const ELECTRONICS_GRADES: [ElectronicsGrade; 5] = [
    ElectronicsGrade {
        name: "коммерческий",
        range_c: (0.0, 70.0),
        source: "отраслевое соглашение производителей ИС",
    },
    ElectronicsGrade {
        name: "промышленный",
        range_c: (-40.0, 85.0),
        source: "отраслевое соглашение производителей ИС",
    },
    ElectronicsGrade {
        name: "AEC-Q100 Grade 2",
        range_c: (-40.0, 105.0),
        source: "AEC-Q100 (Automotive Electronics Council)",
    },
    ElectronicsGrade {
        name: "AEC-Q100 Grade 1",
        range_c: (-40.0, 125.0),
        source: "AEC-Q100 (Automotive Electronics Council)",
    },
    ElectronicsGrade {
        name: "AEC-Q100 Grade 0",
        range_c: (-40.0, 150.0),
        source: "AEC-Q100 (Automotive Electronics Council)",
    },
];

/// Проверка: укладывается ли температура в диапазон класса.
pub fn electronics_ok(grade: &ElectronicsGrade, t_c: f64) -> bool {
    (grade.range_c.0..=grade.range_c.1).contains(&t_c)
}

// ---------------------------------------------------------------------------
// IP-код (IEC 60529)
// ---------------------------------------------------------------------------

/// Значение первой цифры IP (твёрдые тела). Источник: IEC 60529:1989+A1:1999+A2:2013.
pub fn ip_solids(digit: u8) -> &'static str {
    match digit {
        0 => "нет защиты",
        1 => "предметы > 50 мм (тыльная сторона руки)",
        2 => "предметы > 12.5 мм (палец)",
        3 => "предметы > 2.5 мм (инструмент)",
        4 => "предметы > 1 мм (проволока)",
        5 => "пылезащищено (пыль проникает, но не мешает работе)",
        6 => "пыленепроницаемо",
        _ => "нет такой цифры",
    }
}

/// Значение второй цифры IP (вода). Источник: IEC 60529:1989+A1:1999+A2:2013 (цифра 9 —
/// из A2:2013, по ISO 20653).
pub fn ip_water(digit: u8) -> &'static str {
    match digit {
        0 => "нет защиты",
        1 => "вертикально падающие капли",
        2 => "капли при наклоне до 15°",
        3 => "дождь (брызги до 60° от вертикали)",
        4 => "брызги с любого направления",
        5 => "струи (сопло 6.3 мм, 12.5 л/мин)",
        6 => "мощные струи (сопло 12.5 мм, 100 л/мин)",
        7 => "кратковременное погружение (до 1 м, 30 мин)",
        8 => "длительное погружение (условия задаёт изготовитель, жёстче чем 7)",
        9 => "струи высокого давления и температуры (80 °C)",
        _ => "нет такой цифры",
    }
}

/// Условия воды для устройства.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WaterExposure {
    /// Дождь.
    Rain,
    /// Брызги, мокрые руки.
    Splash,
    /// Струя (мойка, ливень с ветром).
    Jet,
    /// Случайное падение в воду до 1 м.
    BriefImmersion,
    /// Плавание, длительное нахождение в воде.
    Swimming,
}

/// Минимальная вторая цифра IP для условий. Важно: IPX7/IPX8 испытываются в пресной
/// неподвижной воде — морская вода (коррозия, проводимость ~5 См/м), движение при плавании
/// (динамическое давление) и перепады температуры (подсос воды при остывании) стандартом
/// не покрываются («оценка» и прямое следствие методики испытаний IEC 60529).
pub fn required_ip_water(exposure: WaterExposure) -> u8 {
    match exposure {
        WaterExposure::Rain => 3,
        WaterExposure::Splash => 4,
        WaterExposure::Jet => 6,
        WaterExposure::BriefImmersion => 7,
        WaterExposure::Swimming => 8,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cold_muscle_is_weaker() {
        let (f, lo, hi, ok) = muscle_power_factor(32.0);
        assert!((f - 0.8).abs() < 1e-9 && lo < f && f < hi && ok);
    }

    #[test]
    fn black_anodised_in_sun_is_hot() {
        let black = &SURFACES[0].0;
        let t = solar_surface_temperature(black, 1000.0, 35.0, 10.0, 1.0, 35.0);
        assert!(t > 70.0 && t < 110.0, "{t}");
        let white = &SURFACES[3].0;
        assert!(solar_surface_temperature(white, 1000.0, 35.0, 10.0, 1.0, 35.0) < t - 25.0);
    }

    #[test]
    fn hands() {
        assert_eq!(hand_function(25.0), HandFunction::Normal);
        assert_eq!(hand_function(6.0), HandFunction::Numb);
        assert_eq!(required_ip_water(WaterExposure::Swimming), 8);
    }
}
