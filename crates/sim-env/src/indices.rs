//! Погодные индексы: ветро-холодовой индекс (NWS / Environment Canada, 2001) и время до
//! обморожения, индекс жары NWS (Ротфус + поправки по Стедману), приближённый WBGT,
//! шкала Бофорта с действием ветра на человека, сила ветра на тело.

use crate::air;

// ---------------------------------------------------------------------------
// Ветро-холодовой индекс
// ---------------------------------------------------------------------------

/// Ветро-холодовой индекс NWS (2001), °F; `t_f` — температура воздуха, °F, `v_mph` — ветер
/// на высоте 10 м (33 фута), миль/ч. Определён при T ≤ 50 °F и V ≥ 3 миль/ч; вне области
/// возвращается температура воздуха.
///
/// Источники: Osczevski R., Bluestein M. (2005) BAMS 86:1453–1458,
/// doi:10.1175/BAMS-86-10-1453; NWS Wind Chill Chart (с 01.11.2001),
/// https://www.weather.gov/safety/cold-wind-chill-chart
pub fn wind_chill_f(t_f: f64, v_mph: f64) -> f64 {
    if t_f > 50.0 || v_mph < 3.0 {
        return t_f;
    }
    let v16 = v_mph.powf(0.16);
    35.74 + 0.6215 * t_f - 35.75 * v16 + 0.4275 * t_f * v16
}

/// Ветро-холодовой индекс Environment Canada / NWS в метрических единицах, °C;
/// `v_kmh` — ветер на 10 м, км/ч. Определён при T ≤ 10 °C и V ≥ 4.8 км/ч.
/// Это та же модель (Osczevski & Bluestein 2005), независимо подогнанная в °C и км/ч.
pub fn wind_chill_c(t_c: f64, v_kmh: f64) -> f64 {
    if t_c > 10.0 || v_kmh < 4.8 {
        return t_c;
    }
    let v16 = v_kmh.powf(0.16);
    13.12 + 0.6215 * t_c - 11.37 * v16 + 0.3965 * t_c * v16
}

/// Категории риска по ветро-холодовому индексу Environment and Climate Change Canada.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum WindChillRisk {
    /// 0…−9: низкий риск.
    Low,
    /// −10…−27: умеренный (неприятно, риск переохлаждения при долгом пребывании).
    Moderate,
    /// −28…−39: высокий, открытая кожа может обморозиться за 10–30 мин.
    High,
    /// −40…−47: очень высокий, обморожение за 5–10 мин.
    VeryHigh,
    /// −48…−54: сильный, обморожение за 2–5 мин.
    Severe,
    /// ≤ −55: экстремальный, обморожение менее чем за 2 мин.
    Extreme,
}

impl WindChillRisk {
    /// Категория по индексу, °C.
    /// Источник: ECCC «Wind chill and the Wind Chill Index»,
    /// https://www.canada.ca/en/services/environment/weather/severeweather/wind-chill-index.html
    /// (там же: при устойчивом ветре > 50 км/ч обморожение наступает быстрее указанного).
    pub fn from_index_c(wci: f64) -> Self {
        if wci > -9.5 {
            Self::Low
        } else if wci > -27.5 {
            Self::Moderate
        } else if wci > -39.5 {
            Self::High
        } else if wci > -47.5 {
            Self::VeryHigh
        } else if wci > -54.5 {
            Self::Severe
        } else {
            Self::Extreme
        }
    }

    /// Описание для интерфейса.
    pub fn description(self) -> &'static str {
        match self {
            Self::Low => "низкий риск",
            Self::Moderate => "умеренный риск: при долгом пребывании — переохлаждение",
            Self::High => "высокий риск: открытая кожа обмораживается за 10–30 мин",
            Self::VeryHigh => "очень высокий риск: обморожение за 5–10 мин",
            Self::Severe => "сильный риск: обморожение за 2–5 мин",
            Self::Extreme => "экстремальный риск: обморожение менее чем за 2 мин",
        }
    }
}

/// Таблица Environment Canada «минуты до обморожения» открытой кожи лица (приблизительно,
/// для большинства здоровых взрослых). Строки — ветер на 10 м, км/ч (10…80);
/// столбцы — температура воздуха, °C (−15…−50). `None` — «обморожение маловероятно».
///
/// Источник: Environment Canada, Wind Chill Program, «Minutes to Frostbite» (архив 2006,
/// https://bac-lac.wayback.archive-it.org/web/20060207235844/http:/pda.msc.ec.gc.ca/education/windchill/minutes_freeze_table_e.cfm);
/// основа — модель Tikuisis & Osczevski (2003) BAMS 84:927–933, doi:10.1175/BAMS-84-7-927.
pub const FROSTBITE_TABLE_T_C: [f64; 8] = [-15.0, -20.0, -25.0, -30.0, -35.0, -40.0, -45.0, -50.0];
/// Строки таблицы: ветер, км/ч.
pub const FROSTBITE_TABLE_V_KMH: [f64; 8] = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0];
/// Значения таблицы, мин.
pub const FROSTBITE_TABLE_MIN: [[Option<f64>; 8]; 8] = {
    const N: Option<f64> = None;
    [
        [
            N,
            N,
            Some(22.0),
            Some(15.0),
            Some(10.0),
            Some(8.0),
            Some(7.0),
            Some(2.0),
        ],
        [
            N,
            Some(30.0),
            Some(14.0),
            Some(10.0),
            Some(5.0),
            Some(4.0),
            Some(3.0),
            Some(2.0),
        ],
        [
            N,
            Some(18.0),
            Some(11.0),
            Some(8.0),
            Some(5.0),
            Some(2.0),
            Some(2.0),
            Some(1.0),
        ],
        [
            Some(42.0),
            Some(14.0),
            Some(9.0),
            Some(5.0),
            Some(5.0),
            Some(2.0),
            Some(2.0),
            Some(1.0),
        ],
        [
            Some(27.0),
            Some(12.0),
            Some(8.0),
            Some(5.0),
            Some(2.0),
            Some(2.0),
            Some(2.0),
            Some(1.0),
        ],
        [
            Some(22.0),
            Some(10.0),
            Some(7.0),
            Some(5.0),
            Some(2.0),
            Some(2.0),
            Some(2.0),
            Some(1.0),
        ],
        [
            Some(18.0),
            Some(9.0),
            Some(5.0),
            Some(4.0),
            Some(2.0),
            Some(2.0),
            Some(2.0),
            Some(1.0),
        ],
        [
            Some(16.0),
            Some(8.0),
            Some(5.0),
            Some(4.0),
            Some(2.0),
            Some(2.0),
            Some(2.0),
            Some(1.0),
        ],
    ]
};

/// Минуты до обморожения по таблице Environment Canada с консервативным округлением:
/// температура — к ближайшему более холодному столбцу, ветер — к ближайшей более сильной
/// строке. `None` — обморожение маловероятно (теплее −15 °C или «*» в таблице).
/// Ветер ниже 10 км/ч берётся по строке 10 км/ч, холоднее −50 °C — по столбцу −50 °C.
pub fn frostbite_minutes_table(t_c: f64, v_kmh: f64) -> Option<f64> {
    if t_c > -15.0 {
        return None;
    }
    let col = FROSTBITE_TABLE_T_C
        .iter()
        .position(|&tt| tt <= t_c + 1e-9)
        .unwrap_or(FROSTBITE_TABLE_T_C.len() - 1);
    let row = FROSTBITE_TABLE_V_KMH
        .iter()
        .position(|&vv| vv >= v_kmh - 1e-9)
        .unwrap_or(FROSTBITE_TABLE_V_KMH.len() - 1);
    FROSTBITE_TABLE_MIN[row][col]
}

/// Время до обморожения, мин, по регрессии Environment Canada (2002):
/// t_f = [−24.5·(0.667·V + 4.8) + 2111]·(−4.8 − T)^−1.668, V — км/ч на 10 м, T — °C.
/// Применимость (по источнику): V = 20…80 км/ч, T = −50…−15 °C. Вне её — `None`.
///
/// Источник: формула приводится в US Patent 7,578,615 («Method and system for determining
/// wind chill temperature») со ссылкой на Environment Canada (2002); вторичный источник —
/// сверяется с таблицей [`FROSTBITE_TABLE_MIN`] в `reference_checks()`.
pub fn frostbite_minutes_regression(t_c: f64, v_kmh: f64) -> Option<f64> {
    if !(-50.0..=-15.0).contains(&t_c) || !(20.0..=80.0).contains(&v_kmh) {
        return None;
    }
    Some((-24.5 * (0.667 * v_kmh + 4.8) + 2111.0) * (-4.8 - t_c).powf(-1.668))
}

/// Сводка по ветру и холоду.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WindChill {
    /// Ветро-холодовой индекс, °C.
    pub index_c: f64,
    /// Категория риска ECCC.
    pub risk: WindChillRisk,
    /// Минуты до обморожения открытой кожи (таблица EC, консервативно); `None` — маловероятно.
    pub frostbite_minutes: Option<f64>,
    /// Условия внутри области определения индекса (T ≤ 10 °C, V ≥ 4.8 км/ч).
    pub in_range: bool,
}

/// Ветро-холодовой индекс и время до обморожения; `v_10m` — ветер на 10 м, м/с.
/// Ветер на уровне лица ≈ 2/3 от ветра на 10 м — это уже заложено в индекс.
pub fn wind_chill(t_c: f64, v_10m: f64) -> WindChill {
    let v_kmh = v_10m * 3.6;
    let index_c = wind_chill_c(t_c, v_kmh);
    WindChill {
        index_c,
        risk: WindChillRisk::from_index_c(index_c),
        frostbite_minutes: frostbite_minutes_table(t_c, v_kmh),
        in_range: t_c <= 10.0 && v_kmh >= 4.8,
    }
}

// ---------------------------------------------------------------------------
// Индекс жары
// ---------------------------------------------------------------------------

/// Индекс жары NWS, °F (алгоритм WPC): сначала простая формула Стедмана; если среднее с T
/// ≥ 80 °F — регрессия Ротфуса (SR 90-23, 1990) с поправками на сухой (RH < 13 %) и
/// очень влажный (RH > 85 %) воздух. Ошибка регрессии относительно таблиц Стедмана ±1.3 °F.
///
/// Источники: https://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml;
/// Steadman R.G. (1979) J. Appl. Meteor. 18:861–873,
/// doi:10.1175/1520-0450(1979)018<0861:TAOSPI>2.0.CO;2.
pub fn heat_index_f(t_f: f64, rh: f64) -> f64 {
    let simple = 0.5 * (t_f + 61.0 + (t_f - 68.0) * 1.2 + rh * 0.094);
    if 0.5 * (simple + t_f) < 80.0 {
        return simple;
    }
    let (t, r) = (t_f, rh);
    let mut hi = -42.379 + 2.049_015_23 * t + 10.143_331_27 * r
        - 0.224_755_41 * t * r
        - 0.006_837_83 * t * t
        - 0.054_817_17 * r * r
        + 0.001_228_74 * t * t * r
        + 0.000_852_82 * t * r * r
        - 0.000_001_99 * t * t * r * r;
    if r < 13.0 && (80.0..=112.0).contains(&t) {
        hi -= (13.0 - r) / 4.0 * ((17.0 - (t - 95.0).abs()) / 17.0).sqrt();
    } else if r > 85.0 && (80.0..=87.0).contains(&t) {
        hi += (r - 85.0) / 10.0 * ((87.0 - t) / 5.0);
    }
    hi
}

/// Индекс жары NWS, °C.
pub fn heat_index_c(t_c: f64, rh: f64) -> f64 {
    f_to_c(heat_index_f(c_to_f(t_c), rh))
}

/// Категории индекса жары NWS (для людей из групп риска; тень, слабый ветер — на солнце
/// индекс выше до 15 °F).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum HeatIndexCategory {
    /// Ниже 80 °F (27 °C).
    None,
    /// 80–90 °F (27–32 °C): усталость при долгом пребывании или нагрузке.
    Caution,
    /// 90–105 °F (32–41 °C): возможны тепловой удар, судороги, тепловое истощение.
    ExtremeCaution,
    /// 105–129 °F (41–54 °C): вероятны; тепловой удар возможен.
    Danger,
    /// ≥ 130 °F (54 °C): тепловой удар вероятен.
    ExtremeDanger,
}

impl HeatIndexCategory {
    /// Категория по индексу, °C. Источник: таблица индекса жары NWS
    /// (https://www.weather.gov/safety/heat-index).
    pub fn from_index_c(hi_c: f64) -> Self {
        let f = c_to_f(hi_c);
        if f < 80.0 {
            Self::None
        } else if f < 90.0 {
            Self::Caution
        } else if f < 105.0 {
            Self::ExtremeCaution
        } else if f < 130.0 {
            Self::Danger
        } else {
            Self::ExtremeDanger
        }
    }
}

/// Таблица индекса жары NWS (Rothfusz, NWS Tulsa): строки — температура, °F (80…110 шагом 2);
/// столбцы — RH 40…100 % шагом 5. `None` — клетка вне таблицы (индекс > 137 °F).
/// Источник: таблица NWS «Heat Index» (воспроизведена, напр., в
/// https://pittsburghpa.gov/files/assets/city/v/1/public-safety/documents/heat_index_and_heat_disorders.pdf).
pub const HEAT_INDEX_TABLE_T_F: [f64; 16] = [
    80.0, 82.0, 84.0, 86.0, 88.0, 90.0, 92.0, 94.0, 96.0, 98.0, 100.0, 102.0, 104.0, 106.0, 108.0,
    110.0,
];
/// Столбцы таблицы: RH, %.
pub const HEAT_INDEX_TABLE_RH: [f64; 13] = [
    40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0, 90.0, 95.0, 100.0,
];
/// Значения таблицы, °F (по строкам T; первые n значений, начиная с RH 40 %).
pub const HEAT_INDEX_TABLE: [&[f64]; 16] = [
    &[
        80., 80., 81., 81., 82., 82., 83., 84., 84., 85., 86., 86., 87.,
    ],
    &[
        81., 82., 83., 84., 84., 85., 86., 88., 89., 90., 91., 93., 95.,
    ],
    &[
        83., 84., 85., 86., 88., 89., 90., 92., 94., 96., 98., 100., 103.,
    ],
    &[
        85., 87., 88., 89., 91., 93., 95., 97., 100., 102., 105., 108., 112.,
    ],
    &[
        88., 89., 91., 93., 95., 98., 100., 103., 106., 110., 113., 117., 121.,
    ],
    &[
        91., 93., 95., 97., 100., 103., 106., 109., 113., 117., 122., 127., 132.,
    ],
    &[
        94., 96., 99., 101., 105., 108., 112., 116., 121., 126., 131.,
    ],
    &[97., 100., 103., 106., 110., 114., 119., 124., 129., 135.],
    &[101., 104., 108., 112., 116., 121., 126., 132.],
    &[105., 109., 113., 117., 123., 128., 134.],
    &[109., 114., 118., 124., 129., 136.],
    &[114., 119., 124., 130., 137.],
    &[119., 124., 131., 137.],
    &[124., 130., 137.],
    &[130., 137.],
    &[136.],
];

/// °C → °F.
pub fn c_to_f(t_c: f64) -> f64 {
    t_c * 9.0 / 5.0 + 32.0
}

/// °F → °C.
pub fn f_to_c(t_f: f64) -> f64 {
    (t_f - 32.0) * 5.0 / 9.0
}

/// м/с → миль/ч.
pub fn ms_to_mph(v: f64) -> f64 {
    v / 0.447_04
}

// ---------------------------------------------------------------------------
// WBGT (приближённо)
// ---------------------------------------------------------------------------

/// Приближённый WBGT Бюро метеорологии Австралии для тени и умеренного ветра, °C:
/// WBGT ≈ 0.567·T + 0.393·e + 3.94, e — парциальное давление пара, гПа.
/// «оценка»: формула не учитывает солнце и ветер и на солнце занижает WBGT на несколько
/// градусов; валидированный уличный метод (Liljegren et al. 2008, J. Occup. Environ. Hyg.
/// 5:645, doi:10.1080/15459620802310770) здесь не реализован.
/// Источник: Australian Bureau of Meteorology, «About the WBGT and Apparent Temperature
/// indices», http://www.bom.gov.au/info/thermal_stress/
pub fn wbgt_shade_estimate(t_c: f64, rh_percent: f64) -> f64 {
    let e_hpa = air::vapour_pressure(t_c, rh_percent) / 100.0;
    0.567 * t_c + 0.393 * e_hpa + 3.94
}

// ---------------------------------------------------------------------------
// Шкала Бофорта
// ---------------------------------------------------------------------------

/// Балл шкалы Бофорта с действием ветра на пешехода.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Beaufort {
    /// Балл 0…12.
    pub number: u8,
    /// Нижняя граница скорости на 10 м, м/с.
    pub v_min: f64,
    /// Верхняя граница (не включая), м/с.
    pub v_max: f64,
    /// Название.
    pub name: &'static str,
    /// Действие на человека.
    pub effect_on_people: &'static str,
}

/// Шкала Бофорта: границы скоростей на 10 м — общепринятые эквиваленты ВМО (напр. Met Office,
/// https://www.metoffice.gov.uk/weather/guides/coast-and-sea/beaufort-scale), действие ветра
/// на людей — по Penwarden (1973)
/// «Acceptable wind speeds in towns», Building Science 8:259–267,
/// doi:10.1016/0007-3628(73)90008-X (таблица в пересказе; уверенность средняя).
pub const BEAUFORT: [Beaufort; 13] = [
    Beaufort {
        number: 0,
        v_min: 0.0,
        v_max: 0.3,
        name: "штиль",
        effect_on_people: "ветра нет",
    },
    Beaufort {
        number: 1,
        v_min: 0.3,
        v_max: 1.6,
        name: "тихий",
        effect_on_people: "ветер не заметен",
    },
    Beaufort {
        number: 2,
        v_min: 1.6,
        v_max: 3.4,
        name: "лёгкий",
        effect_on_people: "ветер ощущается лицом",
    },
    Beaufort {
        number: 3,
        v_min: 3.4,
        v_max: 5.5,
        name: "слабый",
        effect_on_people: "шевелит волосы, одежда хлопает",
    },
    Beaufort {
        number: 4,
        v_min: 5.5,
        v_max: 8.0,
        name: "умеренный",
        effect_on_people: "поднимает пыль и бумагу, растрёпывает волосы",
    },
    Beaufort {
        number: 5,
        v_min: 8.0,
        v_max: 10.8,
        name: "свежий",
        effect_on_people: "сила ветра ощущается телом; можно оступиться при входе в ветровую зону",
    },
    Beaufort {
        number: 6,
        v_min: 10.8,
        v_max: 13.9,
        name: "сильный",
        effect_on_people: "зонт держать трудно, идти ровно трудно, шум ветра в ушах неприятен",
    },
    Beaufort {
        number: 7,
        v_min: 13.9,
        v_max: 17.2,
        name: "крепкий",
        effect_on_people: "идти неудобно",
    },
    Beaufort {
        number: 8,
        v_min: 17.2,
        v_max: 20.8,
        name: "очень крепкий",
        effect_on_people: "мешает идти, трудно удержать равновесие в порывах",
    },
    Beaufort {
        number: 9,
        v_min: 20.8,
        v_max: 24.5,
        name: "шторм",
        effect_on_people: "порывы сбивают людей с ног",
    },
    Beaufort {
        number: 10,
        v_min: 24.5,
        v_max: 28.5,
        name: "сильный шторм",
        effect_on_people: "находиться на открытом месте опасно",
    },
    Beaufort {
        number: 11,
        v_min: 28.5,
        v_max: 32.7,
        name: "жестокий шторм",
        effect_on_people: "находиться на открытом месте опасно",
    },
    Beaufort {
        number: 12,
        v_min: 32.7,
        v_max: f64::INFINITY,
        name: "ураган",
        effect_on_people: "находиться на открытом месте опасно",
    },
];

/// Балл Бофорта по скорости ветра на 10 м, м/с.
pub fn beaufort(v_10m: f64) -> &'static Beaufort {
    let v = v_10m.max(0.0);
    BEAUFORT
        .iter()
        .find(|b| v >= b.v_min && v < b.v_max)
        .unwrap_or(&BEAUFORT[12])
}

/// Сила ветра на тело или деталь, Н: F = ½·ρ·v²·C_d·A. Для стоящего человека C_d·A ≈
/// 0.5–0.9 м² («оценка» по аэродинамике тела, напр. Penwarden 1973); для предплечья с
/// экзоскелетом — цилиндр, C_d ≈ 1.0–1.2 при Re 10⁴–10⁵.
pub fn wind_force(rho: f64, v: f64, cd: f64, area: f64) -> f64 {
    0.5 * rho * v * v * cd * area
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nws_example() {
        // пример NWS: 0 °F и 15 миль/ч → −19 °F
        assert_eq!(wind_chill_f(0.0, 15.0).round(), -19.0);
    }

    #[test]
    fn beaufort_lookup() {
        assert_eq!(beaufort(0.0).number, 0);
        assert_eq!(beaufort(9.0).number, 5);
        assert_eq!(beaufort(40.0).number, 12);
    }

    #[test]
    fn frostbite_table_lookup() {
        assert_eq!(frostbite_minutes_table(-30.0, 30.0), Some(8.0));
        assert_eq!(frostbite_minutes_table(-10.0, 50.0), None);
        // −27 °C → столбец −30 °C, 25 км/ч → строка 30 км/ч
        assert_eq!(frostbite_minutes_table(-27.0, 25.0), Some(8.0));
    }
}
