//! Каталог материалов конструктора: печатные пластики, металлы каркаса, углепластиковая труба.
//!
//! Единицы — СИ: Па, кг/м³, °C; удлинение — доля (0.05 = 5%).
//!
//! # Как выбирались числа
//!
//! Паспорта производителей (TDS) дают **типичные** значения на идеальных образцах и прямо
//! предупреждают, что для расчёта они не годятся. Поэтому для печатных материалов берётся
//! правило «худший паспорт минус разброс»: по каждому свойству считается `среднее − 1σ`
//! по каждому прочитанному паспорту и выбирается **минимум**. Рядом с каждым значением указано,
//! какой паспорт его дал. Для металлов берутся **гарантированные минимумы** стандартов
//! (EN 755-2, EN 10025-2, EN 10088-3, ASTM B348), а не типичные значения.
//!
//! Обозначения паспортов (прочитаны 22.11.2026 из PDF; значения, взятые только из выдержки
//! поисковика, потому что сам PDF не открылся, помечены «выдержка»):
//! * **Bambu** — Bambu Lab Technical Data Sheet: PLA Basic V3.0, PETG Basic V3.0, PETG HF V1.0,
//!   ABS V3.0, ASA V2.0, TPU 95A V2.0 (ISO 527; образцы отожжены и высушены — реальная деталь
//!   слабее), <https://wiki.bambulab.com/>, <https://store.bblcdn.com/>;
//! * **Polymaker** — PolyLite PLA / ABS, Polymaker ASA (wiki.polymaker.com), PolyLite PETG V5.4
//!   и V6.0, PolyMide PA12-CF TDS (ISO 527);
//! * **Stratasys** — FDM Nylon 12 MDS 0921a, ABS-M30 MDS 0921a (ASTM D638, F900/F770,
//!   слой 0.254 мм, «как напечатано»), <https://www.stratasys.com/>;
//! * **UltiMaker** — TPU 95A TDS v2.01 (09.2025); **BASF** — Ultrafuse TPU 95A TDS v1.0;
//! * **MatWeb/AA** — данные Aluminum Association через MatWeb: типичные, «not for design»,
//!   поэтому прочность берётся из стандартов, а отсюда — только то, чего в стандартах нет.

/// Паскалей в мегапаскале.
pub const MPA: f64 = 1e6;
/// Паскалей в гигапаскале.
pub const GPA: f64 = 1e9;
/// Метров в миллиметре.
pub const MM: f64 = 1e-3;

/// Комнатная температура, при которой меряются паспортные значения, °C.
pub const ROOM_C: f64 = 23.0;

/// Материал каталога.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MaterialId {
    Pla,
    Petg,
    Abs,
    Asa,
    Pa12,
    PaCf,
    Tpu95a,
    Al6061T6,
    Al7075T6,
    SteelS235,
    Stainless304,
    Ti6Al4V,
    CfrpTube,
}

impl MaterialId {
    /// Все материалы каталога.
    pub const ALL: [MaterialId; 13] = [
        MaterialId::Pla,
        MaterialId::Petg,
        MaterialId::Abs,
        MaterialId::Asa,
        MaterialId::Pa12,
        MaterialId::PaCf,
        MaterialId::Tpu95a,
        MaterialId::Al6061T6,
        MaterialId::Al7075T6,
        MaterialId::SteelS235,
        MaterialId::Stainless304,
        MaterialId::Ti6Al4V,
        MaterialId::CfrpTube,
    ];

    /// Запись каталога.
    pub fn material(self) -> &'static Material {
        match self {
            MaterialId::Pla => &PLA,
            MaterialId::Petg => &PETG,
            MaterialId::Abs => &ABS,
            MaterialId::Asa => &ASA,
            MaterialId::Pa12 => &PA12,
            MaterialId::PaCf => &PA_CF,
            MaterialId::Tpu95a => &TPU_95A,
            MaterialId::Al6061T6 => &AL_6061_T6,
            MaterialId::Al7075T6 => &AL_7075_T6,
            MaterialId::SteelS235 => &STEEL_S235,
            MaterialId::Stainless304 => &STAINLESS_304,
            MaterialId::Ti6Al4V => &TI_6AL_4V,
            MaterialId::CfrpTube => &CFRP_TUBE,
        }
    }
}

/// Класс материала: от него зависят применимые модели.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MaterialClass {
    /// Печатный термопласт: анизотропный, зависит от настроек печати.
    Printed,
    /// Металл: изотропный, пластичный.
    Metal,
    /// Волокнистый композит: хрупкий, ортотропный.
    Composite,
}

/// Что известно об усталости материала.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FatigueData {
    /// Надёжных данных нет — усталость «не посчитана», а не угадана.
    NoReliableData { reason: &'static str },
    /// Сталь: S'e = 0.5·Sut при Sut ≤ 1400 МПа, колено на 10⁶ циклов (Shigley, §6-7).
    SteelEnduranceRule,
    /// Измеренный предел выносливости гладкого образца при R = −1 на базе `n_ref` циклов.
    /// За базой данных нет: расчёт на большее число циклов — «не посчитано».
    Measured { sf: f64, n_ref: f64 },
}

/// Особенности печати материала.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PrintTraits {
    /// Нужна закрытая камера (усадка и коробление).
    pub needs_enclosure: bool,
    /// Коэффициент линейного расширения, 1/К (`None` — в прочитанных паспортах нет).
    pub cte: Option<f64>,
    /// Температура, от которой деталь «садится» при остывании, °C.
    pub set_temp_c: f64,
    /// Предел нависания без поддержек, градусов от вертикали.
    pub max_overhang_deg: f64,
    /// Предел моста без поддержек, м.
    pub max_bridge: f64,
    /// Абразивный наполнитель: нужно закалённое сопло.
    pub abrasive: bool,
    /// Гигроскопичен: перед печатью сушить.
    pub hygroscopic: bool,
    /// Гибкий: нужен прямой экструдер.
    pub flexible: bool,
}

/// Запись каталога. У печатных `*_along` — вдоль слоёв (X-Y), `*_across` — поперёк (Z).
/// У изотропных значения совпадают.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Material {
    pub id: MaterialId,
    pub name: &'static str,
    pub class: MaterialClass,
    /// Плотность, кг/м³.
    pub density: f64,
    /// Модуль упругости вдоль слоёв/волокон, Па.
    pub e_along: f64,
    /// Модуль упругости поперёк, Па.
    pub e_across: f64,
    /// Модуль при сжатии, если он заметно ниже (композит), Па.
    pub e_compression: Option<f64>,
    /// Предел текучести вдоль, Па. У материалов без площадки — расчётная прочность.
    pub sy_along: f64,
    /// Предел текучести поперёк, Па.
    pub sy_across: f64,
    /// Предел прочности вдоль, Па.
    pub su_along: f64,
    /// Предел прочности поперёк, Па.
    pub su_across: f64,
    /// Прочность при сжатии, если она ниже прочности при растяжении (композит), Па.
    pub compressive: Option<f64>,
    /// Прочность на межслойный сдвиг (композит), Па.
    pub interlaminar_shear: Option<f64>,
    /// Удлинение при разрыве вдоль, доля.
    pub elong_along: f64,
    /// Удлинение при разрыве поперёк, доля.
    pub elong_across: f64,
    /// HDT при 1.8 МПа, °C.
    pub hdt_1p8_c: Option<f64>,
    /// HDT при 0.45 МПа, °C.
    pub hdt_0p45_c: Option<f64>,
    /// Температура стеклования, °C.
    pub tg_c: Option<f64>,
    /// Температура размягчения: при ней несущая способность считается нулевой, °C.
    pub softening_c: Option<f64>,
    /// Предельная рабочая температура, °C.
    pub max_service_c: f64,
    /// Данные об усталости.
    pub fatigue: FatigueData,
    /// Особенности печати (только у печатных).
    pub print: Option<PrintTraits>,
}

impl Material {
    /// Печатный ли материал.
    pub fn is_printed(&self) -> bool {
        self.class == MaterialClass::Printed
    }

    /// Пластичный ли материал по критерию Шигли: удлинение при разрыве ≥ 5%.
    /// Для печатных считается по худшему направлению.
    pub fn is_ductile(&self) -> bool {
        self.elong_along.min(self.elong_across) >= 0.05
    }
}

/// Запас до HDT (1.8 МПа), К: предельная рабочая температура печатных = HDT − 10.
/// Оценка: HDT — это прогиб под 1.8 МПа за минуты; под длительной нагрузкой деталь
/// поплывёт раньше, запас взят в безопасную сторону.
pub const HDT_SERVICE_MARGIN_K: f64 = 10.0;

// --- Печатные -----------------------------------------------------------------------------

/// PLA. Паспорта: Bambu PLA Basic V3.0 и Polymaker PolyLite PLA.
pub static PLA: Material = Material {
    id: MaterialId::Pla,
    name: "PLA",
    class: MaterialClass::Printed,
    density: 1240.0, // Bambu: 1.24 г/см³ (Polymaker 1.17 — берём большее, масса в опасную сторону)
    e_along: 2360.0 * MPA, // Bambu: 2580 − 220
    e_across: 1890.0 * MPA, // Bambu: 2060 − 170
    e_compression: None,
    sy_along: 31.0 * MPA,  // площадки нет, = прочность
    sy_across: 28.0 * MPA, // площадки нет, = прочность
    su_along: 31.0 * MPA,  // Bambu: 35 − 4 (Polymaker: 52.3 ± 0.3)
    su_across: 28.0 * MPA, // Bambu: 31 − 3 (Polymaker: 40.5 ± 0.5)
    compressive: None,
    interlaminar_shear: None,
    elong_along: 0.057,      // Polymaker: 6.3 − 0.6 %
    elong_across: 0.017,     // Polymaker: 1.8 − 0.1 %
    hdt_1p8_c: Some(54.0),   // Bambu, ISO 75 (Polymaker: 58)
    hdt_0p45_c: Some(57.0),  // Bambu, ISO 75 (Polymaker: 60)
    tg_c: Some(60.0),        // Bambu, ДСК (Polymaker: 61)
    softening_c: Some(54.0), // = HDT 1.8 МПа
    max_service_c: 44.0,     // оценка: HDT 54 − 10 К
    fatigue: FatigueData::NoReliableData {
        reason: "для печатного PLA S-N сильно зависит от настроек и партии; переносимых данных нет",
    },
    print: Some(PrintTraits {
        needs_enclosure: false,
        cte: None,
        set_temp_c: 60.0,
        max_overhang_deg: 55.0, // Bambu PLA Basic: Max Overhang Angle 55°
        max_bridge: 30.0 * MM,  // Bambu PLA Basic: Max Bridging Length 30 мм
        abrasive: false,
        hygroscopic: false,
        flexible: false,
    }),
};

/// PETG. Паспорта: Bambu PETG Basic V3.0, Bambu PETG HF V1.0, Polymaker PolyLite PETG V5.4/V6.0.
/// Разброс между паспортами большой: Z от 23 до 42.8 МПа. Старые Polymaker V5.0/V5.1 по данным
/// поиска давали Z = 13.4 ± 2.0 МПа (сам PDF не читался, в минимум не входит); итог после
/// коэффициента межслойной связи (0.7·19 = 13.3 МПа) получается на том же уровне.
pub static PETG: Material = Material {
    id: MaterialId::Petg,
    name: "PETG",
    class: MaterialClass::Printed,
    density: 1280.0,        // Bambu PETG HF: 1.28 г/см³
    e_along: 1620.0 * MPA,  // Bambu HF: 1810 − 190
    e_across: 1410.0 * MPA, // Bambu HF: 1540 − 130
    e_compression: None,
    sy_along: 30.0 * MPA,
    sy_across: 19.0 * MPA,
    su_along: 30.0 * MPA,  // Bambu HF: 34 − 4
    su_across: 19.0 * MPA, // Bambu HF: 23 − 4
    compressive: None,
    interlaminar_shear: None,
    elong_along: 0.067,     // Polymaker V5.4: 8.4 − 1.7 %
    elong_across: 0.031,    // Polymaker V5.4: 3.3 − 0.2 %
    hdt_1p8_c: Some(62.0),  // Bambu HF, ISO 75 (Basic 68, Polymaker 75)
    hdt_0p45_c: Some(69.0), // Bambu HF, ISO 75
    tg_c: Some(66.0),       // Bambu HF, ДСК
    softening_c: Some(62.0),
    max_service_c: 52.0, // оценка: HDT 62 − 10 К
    fatigue: FatigueData::NoReliableData {
        reason: "для печатного PETG надёжных S-N кривых, переносимых на другие настройки, нет",
    },
    print: Some(PrintTraits {
        needs_enclosure: false,
        cte: None,
        set_temp_c: 66.0,
        max_overhang_deg: 70.0, // Bambu PETG Basic/HF: ~70°
        max_bridge: 30.0 * MM,  // Bambu PETG Basic/HF: ~30 мм
        abrasive: false,
        hygroscopic: true,
        flexible: false,
    }),
};

/// ABS. Паспорта: Bambu ABS V3.0, Polymaker PolyLite ABS, Stratasys ABS-M30 (F900 и F770).
pub static ABS: Material = Material {
    id: MaterialId::Abs,
    name: "ABS",
    class: MaterialClass::Printed,
    density: 1050.0,        // Bambu, Stratasys: 1.05 г/см³
    e_along: 2000.0 * MPA,  // Stratasys F770 XZ: 2.00 ГПа
    e_across: 1780.0 * MPA, // Stratasys F770 ZX: 1.78 ГПа
    e_compression: None,
    sy_along: 25.2 * MPA,
    sy_across: 21.7 * MPA,
    su_along: 25.2 * MPA,  // Stratasys F770 XZ, разрыв: 27.6 − 2.4
    su_across: 21.7 * MPA, // Stratasys F770 ZX, разрыв: 22.9 − 1.2
    compressive: None,
    interlaminar_shear: None,
    elong_along: 0.033,     // Stratasys F770 XZ: 4.5 − 1.2 %
    elong_across: 0.014,    // Stratasys F770 ZX: 1.6 − 0.2 %
    hdt_1p8_c: Some(84.0),  // Bambu V3.0, ISO 75 (Polymaker 98, Stratasys 99.9)
    hdt_0p45_c: Some(87.0), // Bambu V3.0, ISO 75
    tg_c: Some(101.0),      // Polymaker, ДСК (Stratasys 105.2)
    softening_c: Some(84.0),
    max_service_c: 74.0, // оценка: HDT 84 − 10 К
    fatigue: FatigueData::NoReliableData {
        reason: "для печатного ABS надёжных S-N кривых, переносимых на другие настройки, нет",
    },
    print: Some(PrintTraits {
        needs_enclosure: true, // Bambu ABS: камера 45–60 °C
        cte: Some(60.77e-6),   // Stratasys ABS-M30, ASTM E831, 40–140 °C
        set_temp_c: 101.0,
        max_overhang_deg: 70.0, // Bambu ABS V3.0: ~70°
        max_bridge: 40.0 * MM,  // Bambu ABS V3.0: ~40 мм
        abrasive: false,
        hygroscopic: false,
        flexible: false,
    }),
};

/// ASA. Паспорта: Bambu ASA V2.0 (PDF) и V3.0 (выдержка: X-Y 37 ± 3, Z 31 ± 4 МПа),
/// Polymaker ASA (wiki), PolyLite ASA V5.3 (выдержка: 38.6 ± 0.3 / 30.0 ± 0.5 МПа).
pub static ASA: Material = Material {
    id: MaterialId::Asa,
    name: "ASA",
    class: MaterialClass::Printed,
    density: 1130.0,        // Polymaker: 1.13 г/см³ (Bambu 1.05)
    e_along: 2180.0 * MPA,  // Bambu V2.0: 2450 − 270
    e_across: 1829.0 * MPA, // Polymaker: 1965 − 136
    e_compression: None,
    sy_along: 34.0 * MPA,
    sy_across: 27.0 * MPA,
    su_along: 34.0 * MPA,  // Bambu V3.0 (выдержка): 37 − 3
    su_across: 27.0 * MPA, // Bambu V3.0 (выдержка): 31 − 4
    compressive: None,
    interlaminar_shear: None,
    elong_along: 0.061,      // Polymaker: 6.7 − 0.6 %
    elong_across: 0.0145,    // Polymaker: 1.65 − 0.2 %
    hdt_1p8_c: Some(92.0),   // Bambu V2.0, ISO 75 (Polymaker 100)
    hdt_0p45_c: Some(100.0), // Bambu V2.0, ISO 75
    tg_c: Some(98.0),        // Polymaker, ДСК
    softening_c: Some(92.0),
    max_service_c: 82.0, // оценка: HDT 92 − 10 К
    fatigue: FatigueData::NoReliableData {
        reason: "для печатного ASA надёжных S-N кривых, переносимых на другие настройки, нет",
    },
    print: Some(PrintTraits {
        needs_enclosure: true, // Bambu ASA: камера 45–60 °C
        cte: Some(60.77e-6),   // оценка: своего значения нет, взято как у ABS-M30
        set_temp_c: 98.0,
        max_overhang_deg: 70.0, // Bambu ASA V2.0: ~70°
        max_bridge: 40.0 * MM,  // Bambu ASA V2.0: ~40 мм
        abrasive: false,
        hygroscopic: false,
        flexible: false,
    }),
};

/// PA12 (нейлон). Паспорт: Stratasys FDM Nylon 12 MDS 0921a, F900, «как напечатано».
/// Tg = 34 °C: уже у тела материал выше стеклования — длительная нагрузка не считается.
pub static PA12: Material = Material {
    id: MaterialId::Pa12,
    name: "PA12 (нейлон)",
    class: MaterialClass::Printed,
    density: 1010.0,        // Stratasys: удельный вес 1.01
    e_along: 1423.0 * MPA,  // Stratasys XZ: 1.51 − 0.087 ГПа
    e_across: 1130.0 * MPA, // Stratasys ZX: 1.25 − 0.12 ГПа
    e_compression: None,
    sy_along: 48.8 * MPA,  // Stratasys XZ, текучесть: 49.3 − 0.48
    sy_across: 41.1 * MPA, // Stratasys ZX, текучесть: 41.8 − 0.67
    su_along: 48.8 * MPA,  // пик кривой = текучесть (разрыв 33.4 МПа — уже после шейки)
    su_across: 41.1 * MPA,
    compressive: None,
    interlaminar_shear: None,
    elong_along: 0.07,      // Stratasys XZ: 30 − 23 % (разброс огромный)
    elong_across: 0.0611,   // Stratasys ZX: 6.5 − 0.39 %
    hdt_1p8_c: Some(75.3),  // Stratasys XZ/ZX, ASTM D648, 264 psi
    hdt_0p45_c: Some(91.9), // Stratasys XZ/ZX, ASTM D648, 66 psi
    tg_c: Some(34.03),      // Stratasys, ASTM D7426
    softening_c: Some(75.3),
    max_service_c: 65.3, // оценка: HDT 75.3 − 10 К
    fatigue: FatigueData::NoReliableData {
        reason: "для печатного PA12 надёжных S-N кривых, переносимых на другие настройки, нет",
    },
    print: Some(PrintTraits {
        needs_enclosure: true,
        cte: Some(98.23e-6),    // Stratasys Nylon 12, ASTM E831, 45–70 °C
        set_temp_c: 91.9,       // оценка: HDT 0.45 МПа
        max_overhang_deg: 45.0, // своего значения нет — базовое правило FDM
        max_bridge: 10.0 * MM,  // своего значения нет — базовое правило FDM
        abrasive: false,
        hygroscopic: true,
        flexible: false,
    }),
};

/// PA-CF (нейлон 12 с рубленым углеволокном). Паспорта: Polymaker PolyMide PA12-CF.
/// Вдоль слоёв берётся значение **без отжига** (43.55 ± 0.27 МПа против 71.6 после отжига):
/// отжиг нельзя считать выполненным по умолчанию. Поперёк — Stratasys FDM Nylon 12CF: старая
/// версия паспорта, ZX, текучесть 28 МПа (выдержка); действующая MDS 0824a, табл. 4, ZX,
/// разрыв 32.7 ± 3.5 МПа (PDF), то есть 29.2 — взято меньшее.
pub static PA_CF: Material = Material {
    id: MaterialId::PaCf,
    name: "PA-CF",
    class: MaterialClass::Printed,
    density: 1060.0,        // Polymaker: 1.06 г/см³
    e_along: 1837.0 * MPA,  // Polymaker без отжига: 1971.8 − 135
    e_across: 1436.0 * MPA, // Polymaker, влажный: 1520 − 84
    e_compression: None,
    sy_along: 43.3 * MPA,  // Polymaker без отжига: 43.55 − 0.27
    sy_across: 28.0 * MPA, // Stratasys Nylon 12CF, ZX, текучесть (выдержка)
    su_along: 43.3 * MPA,
    su_across: 28.0 * MPA,
    compressive: None,
    interlaminar_shear: None,
    elong_along: 0.0211,    // Stratasys MDS 0824a, табл. 4, XZ: 2.4 − 0.29 %
    elong_across: 0.0093,   // Stratasys MDS 0824a, табл. 4, ZX: 1.2 − 0.27 %
    hdt_1p8_c: Some(75.3),  // оценка: HDT 105 °C у Polymaker — после отжига; без отжига берём PA12
    hdt_0p45_c: Some(91.9), // оценка: как у PA12 (у Polymaker после отжига 131 °C)
    tg_c: Some(34.03),      // оценка: PA12 по Stratasys (у Nylon 12CF 37.5 °C, у Polymaker 108 °C)
    softening_c: Some(75.3),
    max_service_c: 65.3, // оценка: 75.3 − 10 К
    fatigue: FatigueData::NoReliableData {
        reason: "для печатного PA-CF усталость зависит от ориентации волокна; надёжных данных нет",
    },
    print: Some(PrintTraits {
        needs_enclosure: true,
        cte: None,
        set_temp_c: 91.9,       // оценка: как у PA12
        max_overhang_deg: 45.0, // своего значения нет — базовое правило FDM
        max_bridge: 10.0 * MM,  // своего значения нет — базовое правило FDM
        abrasive: true, // Polymaker: 10% рубленого волокна, латунное сопло быстро изнашивается
        hygroscopic: true,
        flexible: false,
    }),
};

/// TPU 95A. Паспорта: Bambu V2.0, UltiMaker v2.01, BASF Ultrafuse v1.0. Расхождение по Z
/// огромное: Bambu 23.2 МПа, BASF 12.2, UltiMaker 6.4. Площадки текучести нет, поэтому за
/// «текучесть» взято напряжение при 50% деформации (BASF: плоско 8.3, стоя 7.9 МПа),
/// ограниченное прочностью поперёк.
pub static TPU_95A: Material = Material {
    id: MaterialId::Tpu95a,
    name: "TPU 95A",
    class: MaterialClass::Printed,
    density: 1200.0,     // Bambu: 1.20 г/см³ (BASF 1.149)
    e_along: 8.8 * MPA,  // Bambu: 9.2 − 0.4 (UltiMaker и BASF — десятки МПа, модуль нелинеен)
    e_across: 7.3 * MPA, // Bambu: 7.8 − 0.5
    e_compression: None,
    sy_along: 8.3 * MPA,  // BASF, напряжение при 50% деформации, плоско
    sy_across: 6.2 * MPA, // ограничено разрывом поперёк (UltiMaker)
    su_along: 21.6 * MPA, // UltiMaker XY: 23.7 − 2.1
    su_across: 6.2 * MPA, // UltiMaker Z: 6.4 − 0.2
    compressive: None,
    interlaminar_shear: None,
    elong_along: 5.6,    // UltiMaker XY: > 560 %
    elong_across: 0.639, // UltiMaker Z: 82.3 − 18.4 %
    hdt_1p8_c: None,     // Bambu: N/A
    hdt_0p45_c: None,
    tg_c: Some(-25.0),       // BASF, ISO 11357-2
    softening_c: Some(75.0), // BASF: Vicat при 10 Н = 75 °C (при 50 Н — ниже 27 °C)
    max_service_c: 60.0,     // оценка: Vicat 75 − 15 К
    fatigue: FatigueData::NoReliableData {
        reason: "для печатного TPU надёжных данных нет; усталость эластомера считается иначе",
    },
    print: Some(PrintTraits {
        needs_enclosure: false,
        cte: None,
        set_temp_c: 75.0,
        max_overhang_deg: 45.0, // своего значения нет — базовое правило FDM
        max_bridge: 10.0 * MM,  // своего значения нет — базовое правило FDM
        abrasive: false,
        hygroscopic: true,
        flexible: true,
    }),
};

// --- Металлы ------------------------------------------------------------------------------

/// Алюминий 6061-T6. Прочность — минимумы EN 755-2 (прессованный профиль T6, стенка ≤ 5 мм:
/// Rm 260, Rp0.2 240 МПа; выписка Bikar). Типичные 310/276 МПа по данным AA — «not for design».
pub static AL_6061_T6: Material = Material {
    id: MaterialId::Al6061T6,
    name: "Алюминий 6061-T6",
    class: MaterialClass::Metal,
    density: 2700.0,     // MatWeb/AA
    e_along: 68.9 * GPA, // MatWeb/AA
    e_across: 68.9 * GPA,
    e_compression: None,
    sy_along: 240.0 * MPA, // EN 755-2
    sy_across: 240.0 * MPA,
    su_along: 260.0 * MPA, // EN 755-2
    su_across: 260.0 * MPA,
    compressive: None,
    interlaminar_shear: None,
    elong_along: 0.06, // EN 755-2, меньшее из значений A/A50 таблицы
    elong_across: 0.06,
    hdt_1p8_c: None,
    hdt_0p45_c: None,
    tg_c: None,
    softening_c: None,
    max_service_c: 100.0, // MatWeb/AA: Rp0.2 262 МПа при 100 °C, дальше резкий спад (214 при 149)
    // MatWeb/AA: 96.5 МПа на 5·10⁸ циклов, R = −1, машина R.R. Moore.
    fatigue: FatigueData::Measured {
        sf: 96.5 * MPA,
        n_ref: 5e8,
    },
    print: None,
};

/// Алюминий 7075-T6. Прочность — нижняя граница диапазона T6 (510–540 / 430–480 МПа, Wikipedia
/// «7075 aluminium alloy»). Типичные 572/503 МПа (MatWeb/AA) не используются.
pub static AL_7075_T6: Material = Material {
    id: MaterialId::Al7075T6,
    name: "Алюминий 7075-T6",
    class: MaterialClass::Metal,
    density: 2810.0,     // MatWeb/AA
    e_along: 71.7 * GPA, // MatWeb/AA
    e_across: 71.7 * GPA,
    e_compression: None,
    sy_along: 430.0 * MPA,
    sy_across: 430.0 * MPA,
    su_along: 510.0 * MPA,
    su_across: 510.0 * MPA,
    compressive: None,
    interlaminar_shear: None,
    elong_along: 0.05, // Wikipedia: 5–11 %, нижняя граница
    elong_across: 0.05,
    hdt_1p8_c: None,
    hdt_0p45_c: None,
    tg_c: None,
    softening_c: None,
    max_service_c: 100.0, // оценка: T6 перестаривается при нагреве, как и 6061
    // MatWeb/AA: 159 МПа на 5·10⁸ циклов, R = −1, R.R. Moore.
    fatigue: FatigueData::Measured {
        sf: 159.0 * MPA,
        n_ref: 5e8,
    },
    print: None,
};

/// Сталь конструкционная S235JR, толщина ≤ 16 мм. EN 10025-2: Re ≥ 235, Rm ≥ 360 МПа, A ≥ 26%.
/// E и плотность — EN 1993-1-1, §3.2.6.
pub static STEEL_S235: Material = Material {
    id: MaterialId::SteelS235,
    name: "Сталь S235",
    class: MaterialClass::Metal,
    density: 7850.0,      // EN 1993-1-1
    e_along: 210.0 * GPA, // EN 1993-1-1
    e_across: 210.0 * GPA,
    e_compression: None,
    sy_along: 235.0 * MPA, // EN 10025-2
    sy_across: 235.0 * MPA,
    su_along: 360.0 * MPA, // EN 10025-2
    su_across: 360.0 * MPA,
    compressive: None,
    interlaminar_shear: None,
    elong_along: 0.26, // EN 10025-2
    elong_across: 0.26,
    hdt_1p8_c: None,
    hdt_0p45_c: None,
    tg_c: None,
    softening_c: None,
    max_service_c: 300.0, // EN 1993-1-2: k_y = 1 до 400 °C; граница взята с запасом (оценка)
    fatigue: FatigueData::SteelEnduranceRule,
    print: None,
};

/// Нержавеющая сталь 304 (1.4301), отожжённая. EN 10088-3: Rp0.2 ≥ 190, Rm 500–700 МПа.
pub static STAINLESS_304: Material = Material {
    id: MaterialId::Stainless304,
    name: "Нержавеющая сталь 304",
    class: MaterialClass::Metal,
    density: 8000.0,      // MatWeb, AISI 304
    e_along: 193.0 * GPA, // MatWeb, AISI 304 (193–200 ГПа, нижняя граница)
    e_across: 193.0 * GPA,
    e_compression: None,
    sy_along: 190.0 * MPA, // EN 10088-3
    sy_across: 190.0 * MPA,
    su_along: 500.0 * MPA, // EN 10088-3, нижняя граница
    su_across: 500.0 * MPA,
    compressive: None,
    interlaminar_shear: None,
    elong_along: 0.40, // нижняя граница приведённых источников (40–70 %)
    elong_across: 0.40,
    hdt_1p8_c: None,
    hdt_0p45_c: None,
    tg_c: None,
    softening_c: None,
    max_service_c: 300.0, // оценка: для механизмов у тела заведомо выше реальных температур
    // MakeItFrom, Annealed 304: 210 МПа (база 10⁷ по типовым данным). Выше базы — «не посчитано».
    fatigue: FatigueData::Measured {
        sf: 210.0 * MPA,
        n_ref: 1e7,
    },
    print: None,
};

/// Титан Ti-6Al-4V (Grade 5), отожжённый. ASTM B348: Rm ≥ 895, Rp0.2 ≥ 828 МПа, A ≥ 10%.
pub static TI_6AL_4V: Material = Material {
    id: MaterialId::Ti6Al4V,
    name: "Титан Ti-6Al-4V",
    class: MaterialClass::Metal,
    density: 4430.0,      // MatWeb, Grade 5
    e_along: 113.8 * GPA, // MatWeb, Grade 5
    e_across: 113.8 * GPA,
    e_compression: None,
    sy_along: 828.0 * MPA, // ASTM B348
    sy_across: 828.0 * MPA,
    su_along: 895.0 * MPA, // ASTM B348
    su_across: 895.0 * MPA,
    compressive: None,
    interlaminar_shear: None,
    elong_along: 0.10, // ASTM B348
    elong_across: 0.10,
    hdt_1p8_c: None,
    hdt_0p45_c: None,
    tg_c: None,
    softening_c: None,
    max_service_c: 300.0, // оценка: для механизмов у тела заведомо выше реальных температур
    // MatWeb, Grade 5: 510 МПа, гладкий образец, 10⁷ циклов. Надрез Kt = 3.3 — 240 МПа.
    fatigue: FatigueData::Measured {
        sf: 510.0 * MPA,
        n_ref: 1e7,
    },
    print: None,
};

/// Углепластиковая пултрузионная труба. Паспорт Easy Composites «Carbon Fibre Pultrusions»
/// (v1.0, 20.09.2024) — нижние границы диапазонов. Это намеренно самый слабый из найденных
/// паспортов (другие продавцы заявляют до 1650 МПа): партия неизвестна, волокно вдоль оси.
pub static CFRP_TUBE: Material = Material {
    id: MaterialId::CfrpTube,
    name: "Углепластиковая труба",
    class: MaterialClass::Composite,
    density: 1500.0,     // Easy Composites: 1.30–1.50 г/см³, верхняя граница
    e_along: 28.0 * GPA, // Easy Composites: 28–40 ГПа
    e_across: 8.0 * GPA, // Easy Composites: 8–12 ГПа
    e_compression: Some(10.0 * GPA), // Easy Composites: 10–20 ГПа
    sy_along: 400.0 * MPA, // хрупкий, площадки нет: = прочность
    sy_across: 18.0 * MPA,
    su_along: 400.0 * MPA,                // Easy Composites: 400–500 МПа
    su_across: 18.0 * MPA,                // Easy Composites: 18–30 МПа
    compressive: Some(200.0 * MPA),       // Easy Composites: 200–320 МПа
    interlaminar_shear: Some(30.0 * MPA), // Easy Composites: short beam shear 30–40 МПа
    elong_along: 0.0143,                  // оценка: σ/E = 400 МПа / 28 ГПа
    elong_across: 0.00225,                // оценка: σ/E = 18 МПа / 8 ГПа
    hdt_1p8_c: None,
    hdt_0p45_c: None,
    tg_c: None,
    softening_c: None,
    max_service_c: 80.0, // Easy Composites: Maximum Service Temperature 80 °C
    fatigue: FatigueData::NoReliableData {
        reason: "усталость композита зависит от укладки и партии; данных на эту трубу нет",
    },
    print: None,
};

/// Rp0.2 сплава 6061-T6 при температуре, МПа (MatWeb/AA, типичные значения).
/// Используется только отношение к значению при 24 °C.
pub const AL6061_YIELD_VS_T: [(f64, f64); 7] = [
    (24.0, 276.0),
    (100.0, 262.0),
    (149.0, 214.0),
    (204.0, 103.0),
    (260.0, 34.0),
    (316.0, 19.0),
    (371.0, 12.0),
];

/// Доля прочности 7075-T6 при 100 °C. Оценка: своего значения в прочитанных источниках нет,
/// у 6061 отношение 0.95, для более чувствительного к перегреву 7075 взято меньше.
const AL7075_FRACTION_AT_100C: f64 = 0.85;

/// Доля прочности углепластика у предельной температуры смолы. Оценка: данных по этой трубе
/// нет, свойства матрицы падают к Tg, взят двукратный запас.
const CFRP_FRACTION_AT_LIMIT: f64 = 0.5;

fn lerp(x: f64, x0: f64, y0: f64, x1: f64, y1: f64) -> f64 {
    y0 + (y1 - y0) * (x - x0) / (x1 - x0)
}

/// Во сколько раз падает прочность материала при температуре `t_c` (0..1).
///
/// * Печатные и эластомер: линейно от 1 при 23 °C до 0 при температуре размягчения. Это
///   **оценка в опасную сторону**: реальный пластик теряет прочность нелинейно и у комнатной
///   температуры медленнее, зато модель не обещает несущей способности у HDT.
/// * 6061-T6: таблица Rp0.2(T) по MatWeb/AA.
/// * 7075-T6 и углепластик: линейная оценка до предельной температуры.
/// * Сталь, нержавеющая сталь и титан: 1 в пределах рабочей температуры.
///
/// Холод не добавляет прочности: при T < 23 °C множитель равен 1. NaN даёт 0.
pub fn temperature_factor(m: &Material, t_c: f64) -> f64 {
    if !t_c.is_finite() {
        return 0.0;
    }
    let t = t_c.max(ROOM_C);
    let k = match (m.class, m.id) {
        (MaterialClass::Printed, _) => {
            let soft = m.softening_c.unwrap_or(ROOM_C);
            if t >= soft {
                0.0
            } else {
                (soft - t) / (soft - ROOM_C)
            }
        }
        (_, MaterialId::Al6061T6) => {
            let base = AL6061_YIELD_VS_T[0].1;
            let table = &AL6061_YIELD_VS_T;
            if t <= table[0].0 {
                1.0
            } else if t >= table[table.len() - 1].0 {
                table[table.len() - 1].1 / base
            } else {
                let mut v = 1.0;
                for w in table.windows(2) {
                    if t >= w[0].0 && t <= w[1].0 {
                        v = lerp(t, w[0].0, w[0].1, w[1].0, w[1].1) / base;
                        break;
                    }
                }
                v
            }
        }
        (_, MaterialId::Al7075T6) => lerp(t, ROOM_C, 1.0, 100.0, AL7075_FRACTION_AT_100C).max(0.0),
        (MaterialClass::Composite, _) => {
            lerp(t, ROOM_C, 1.0, m.max_service_c, CFRP_FRACTION_AT_LIMIT).max(0.0)
        }
        _ => 1.0,
    };
    k.clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn id_and_record_agree() {
        for id in MaterialId::ALL {
            assert_eq!(id.material().id, id);
        }
    }

    #[test]
    fn all_values_physical() {
        for id in MaterialId::ALL {
            let m = id.material();
            for v in [
                m.density,
                m.e_along,
                m.e_across,
                m.sy_along,
                m.sy_across,
                m.su_along,
                m.su_across,
                m.elong_along,
                m.elong_across,
                m.max_service_c,
            ] {
                assert!(v.is_finite() && v > 0.0, "{}: {v}", m.name);
            }
            assert!(
                m.sy_along <= m.su_along + 1.0,
                "{}: текучесть выше прочности",
                m.name
            );
            assert!(
                m.sy_across <= m.su_across + 1.0,
                "{}: текучесть выше прочности",
                m.name
            );
        }
    }

    #[test]
    fn printed_are_weaker_across_layers() {
        for id in MaterialId::ALL {
            let m = id.material();
            if m.is_printed() {
                assert!(m.su_across < m.su_along, "{}", m.name);
                assert!(m.e_across < m.e_along, "{}", m.name);
                assert!(m.print.is_some(), "{}", m.name);
                assert!(
                    matches!(m.fatigue, FatigueData::NoReliableData { .. }),
                    "{}: у печатных нет каталожной S-N",
                    m.name
                );
            } else {
                assert!(m.print.is_none(), "{}", m.name);
            }
        }
    }

    #[test]
    fn service_limit_below_softening() {
        for id in MaterialId::ALL {
            let m = id.material();
            if let Some(soft) = m.softening_c {
                assert!(m.max_service_c < soft, "{}", m.name);
            }
        }
    }

    #[test]
    fn temperature_factor_shape() {
        // У HDT несущая способность пластика нулевая.
        assert_eq!(temperature_factor(&PLA, 54.0), 0.0);
        assert_eq!(temperature_factor(&PLA, 200.0), 0.0);
        // Холод прочности не добавляет.
        assert_eq!(temperature_factor(&PLA, -20.0), 1.0);
        // NaN — ноль, а не «всё хорошо».
        assert_eq!(temperature_factor(&PLA, f64::NAN), 0.0);
        assert_eq!(temperature_factor(&AL_6061_T6, f64::INFINITY), 0.0);
        // Таблица 6061: при 149 °C — 214/276.
        let k = temperature_factor(&AL_6061_T6, 149.0);
        assert!((k - 214.0 / 276.0).abs() < 1e-12, "{k}");
        // Монотонное падение для всех.
        for id in MaterialId::ALL {
            let m = id.material();
            let mut prev = 1.0;
            for i in 0..60 {
                let k = temperature_factor(m, 20.0 + i as f64 * 5.0);
                assert!(k <= prev + 1e-12, "{} при {} °C", m.name, 20 + i * 5);
                prev = k;
            }
        }
    }
}
