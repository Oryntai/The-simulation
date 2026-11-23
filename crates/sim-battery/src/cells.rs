//! Каталог ячеек: числа из даташитов, у каждого значения свой источник.
//!
//! Правила каталога:
//! * у каждого параметра есть [`Param::source`] и вид источника [`SourceKind`];
//! * значение без прямого источника помечено [`SourceKind::Estimate`] и взято в опасную для
//!   вывода сторону: нижнее сопротивление — для максимального тока КЗ, верхнее — для нагрева
//!   и для минимального тока, на котором должна сработать защита;
//! * нет данных — `value: None`, и зависящая от него проверка получает `NotAnalyzed`;
//! * если редакции даташита расходятся, берётся более строгое число, расхождение записано
//!   в [`CellSpec::notes`].
//!
//! Сопротивление ячейки задано тремя числами, потому что опасная сторона у разных проверок
//! разная: `r_low_ohm` даёт наибольший ток КЗ (отключающая способность, нагрев провода),
//! `r_dc_high_ohm` — наибольший нагрев ячейки и наименьший ток КЗ (сработает ли защита).

/// Вид источника числа.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SourceKind {
    /// Даташит производителя.
    Datasheet,
    /// Рецензируемая публикация или независимое измерение.
    Literature,
    /// Оценка: прямого источника нет, значение взято в опасную сторону.
    Estimate,
    /// Задано пользователем в форме.
    User,
    /// Данных нет.
    Missing,
}

/// Числовой параметр с источником.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Param {
    /// Значение; `None` — данных нет.
    pub value: Option<f64>,
    /// Откуда взято.
    pub source: &'static str,
    /// Вид источника.
    pub kind: SourceKind,
}

impl Param {
    /// Значение из даташита.
    pub const fn datasheet(value: f64, source: &'static str) -> Self {
        Self {
            value: Some(value),
            source,
            kind: SourceKind::Datasheet,
        }
    }

    /// Значение из публикации или независимого измерения.
    pub const fn literature(value: f64, source: &'static str) -> Self {
        Self {
            value: Some(value),
            source,
            kind: SourceKind::Literature,
        }
    }

    /// Оценка в опасную сторону; в `source` — как получена.
    pub const fn estimate(value: f64, source: &'static str) -> Self {
        Self {
            value: Some(value),
            source,
            kind: SourceKind::Estimate,
        }
    }

    /// Данных нет.
    pub const fn missing(source: &'static str) -> Self {
        Self {
            value: None,
            source,
            kind: SourceKind::Missing,
        }
    }

    /// Значение из формы; пустое поле — данных нет.
    pub const fn user(value: Option<f64>) -> Self {
        match value {
            Some(_) => Self {
                value,
                source: SRC_USER,
                kind: SourceKind::User,
            },
            None => Self::missing("поле формы не заполнено"),
        }
    }
}

const SRC_USER: &str = "задано пользователем";

/// Химия ячейки (для порога теплового разгона).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Chemistry {
    /// NMC811 / графит-SiOx.
    Nmc811,
    /// NCA.
    Nca,
    /// Никелевый слоистый катод, тип в даташите не указан: берём худшее из NMC811 и NCA.
    NiRichUnspecified,
    /// LiCoO₂ (типично для LiPo-пачек).
    #[default]
    Lco,
    /// LiFePO₄.
    Lfp,
}

/// Характерные температуры саморазогрева.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RunawayThreshold {
    /// T1 — начало саморазогрева, °C.
    pub t1_self_heating_c: f64,
    /// T2 — запуск разгона, °C (`None` — надёжных данных по этой химии нет).
    pub t2_trigger_c: Option<f64>,
    /// Источник.
    pub source: &'static str,
}

const SRC_TR_811: &str = "Li Y., Liu X., Wang L., Feng X. et al., «Thermal runaway mechanism of lithium-ion \
     battery with LiNi0.8Mn0.1Co0.1O2 cathode materials» (рукопись OSTI 1820565): пакет 4.2 В, \
     T1 = 65 °C, T2 = 218 °C, T3 = 750 °C";
const SRC_TR_NCA: &str = "Abraham et al., ARC ячейки 18650 NCA/графит, 100 % SOC: саморазогрев с 84 °C — \
     по обзорной табл. 1 в Batteries 2017, 3(2), 14; T2 в этих данных нет";
const SRC_TR_NI: &str = "катод в даташите не назван: принято худшее из NMC811 (65 °C, рукопись OSTI \
     1820565) и NCA (84 °C, Batteries 2017, 3(2), 14); T2 не задан";
const SRC_TR_LCO: &str = "Jhu et al., VSP2, Samsung ICR18650-26D при 4.2 В: начало саморазогрева 116.5 °C — \
     по табл. 1 в Batteries 2017, 3(2), 14; разгон LCO/NMC с 150 °C — Golubkov et al., RSC Adv. \
     2014, 4:3633, в пересказе того же обзора";
const SRC_TR_LFP: &str = "T1 = 90 °C — ARC ячейки 18650 LFP, Batteries 2017, 3(2), 14; T2 = 170.3 °C — \
     Tang et al., iScience 2021, 24(10):103088, doi:10.1016/j.isci.2021.103088";

impl Chemistry {
    /// Название для интерфейса.
    pub fn name(self) -> &'static str {
        match self {
            Chemistry::Nmc811 => "NMC811",
            Chemistry::Nca => "NCA",
            Chemistry::NiRichUnspecified => "никелевый катод (тип не указан)",
            Chemistry::Lco => "LiCoO₂",
            Chemistry::Lfp => "LiFePO₄",
        }
    }

    /// Порог теплового разгона. Значения — для свежих ячеек при полном заряде; у ячеек,
    /// состаренных зарядом на холоде, саморазогрев начинается намного раньше (NCM/LMO:
    /// 42.1 °C против 97.9 °C у свежей — Fleischhammer et al., табл. 1 в Batteries 2017, 3(2), 14).
    pub fn runaway(self) -> RunawayThreshold {
        match self {
            Chemistry::Nmc811 => RunawayThreshold {
                t1_self_heating_c: 65.0,
                t2_trigger_c: Some(218.0),
                source: SRC_TR_811,
            },
            Chemistry::Nca => RunawayThreshold {
                t1_self_heating_c: 84.0,
                t2_trigger_c: None,
                source: SRC_TR_NCA,
            },
            Chemistry::NiRichUnspecified => RunawayThreshold {
                t1_self_heating_c: 65.0,
                t2_trigger_c: None,
                source: SRC_TR_NI,
            },
            Chemistry::Lco => RunawayThreshold {
                t1_self_heating_c: 116.5,
                t2_trigger_c: Some(150.0),
                source: SRC_TR_LCO,
            },
            Chemistry::Lfp => RunawayThreshold {
                t1_self_heating_c: 90.0,
                t2_trigger_c: Some(170.3),
                source: SRC_TR_LFP,
            },
        }
    }
}

/// Форма ячейки, мм.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CellFormat {
    /// Цилиндр.
    Cylindrical { diameter_mm: f64, height_mm: f64 },
    /// Пакет (pouch); `None` — размер не задан.
    Pouch {
        length_mm: Option<f64>,
        width_mm: Option<f64>,
        thickness_mm: Option<f64>,
    },
}

/// Предел тока заряда: действует при температуре не выше `up_to_c`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChargeLimit {
    /// Верхняя граница диапазона, °C.
    pub up_to_c: f64,
    /// Предельный ток на ячейку, А.
    pub current_a: f64,
    /// Источник.
    pub source: &'static str,
}

/// Паспорт ячейки.
#[derive(Clone, Debug, PartialEq)]
pub struct CellSpec {
    /// Название.
    pub name: String,
    /// Химия.
    pub chemistry: Chemistry,
    /// Форма и размеры.
    pub format: CellFormat,
    /// Типичная ёмкость, А·ч.
    pub capacity_typ_ah: Param,
    /// Минимальная ёмкость, А·ч.
    pub capacity_min_ah: Param,
    /// Номинальное напряжение, В.
    pub v_nominal: Param,
    /// Предельное напряжение заряда, В.
    pub v_charge_max: Param,
    /// Конечное напряжение разряда, В.
    pub v_cutoff: Param,
    /// Нижняя оценка сопротивления (для наибольшего тока КЗ), Ом.
    pub r_low_ohm: Param,
    /// Типичное DC-сопротивление, Ом.
    pub r_dc_typ_ohm: Param,
    /// Верхняя оценка DC-сопротивления свежей ячейки при 25 °C, Ом.
    pub r_dc_high_ohm: Param,
    /// Длительный ток разряда, А.
    pub i_discharge_cont_a: Param,
    /// Длительный ток разряда на холоде, А.
    pub i_discharge_cold_a: Param,
    /// Ниже этой температуры действует холодный предел, °C.
    pub discharge_cold_below_c: Param,
    /// Импульсный ток разряда, А.
    pub i_pulse_a: Param,
    /// Длительность импульса, с.
    pub pulse_s: Param,
    /// Стандартный ток заряда, А.
    pub i_charge_std_a: Param,
    /// Пределы тока заряда по температуре (по возрастанию `up_to_c`).
    pub charge_limits: Vec<ChargeLimit>,
    /// Заряд: нижняя граница, °C.
    pub t_charge_min_c: Param,
    /// Заряд: верхняя граница, °C.
    pub t_charge_max_c: Param,
    /// Разряд: нижняя граница, °C.
    pub t_discharge_min_c: Param,
    /// Разряд: верхняя граница, °C.
    pub t_discharge_max_c: Param,
    /// Масса, кг.
    pub mass_kg: Param,
    /// Удельная теплоёмкость, Дж/(кг·К).
    pub cp_j_kg_k: Param,
    /// Энергия активации сопротивления, Дж/моль.
    pub ea_r_j_mol: Param,
    /// Расхождения источников и оговорки.
    pub notes: &'static str,
}

/// Удельная теплоёмкость рулона LG M50 (O'Regan et al., Electrochim. Acta 425:140700, 2022) —
/// то же значение, что в `sim-circuit::battery`.
const CP_M50: f64 = 887.0;
/// Энергия активации R0 ячейки LG M50 — то же значение, что в `sim-circuit::battery`.
const EA_M50: f64 = 15.0e3;

const SRC_CP: &str =
    "O'Regan et al., Electrochim. Acta 425:140700 (2022), рулон M50; значение как в sim-circuit";
const SRC_CP_EST: &str =
    "оценка: теплоёмкость рулона LG M50 (O'Regan 2022), для этой ячейки не измерена";
const SRC_EA: &str =
    "Brosa Planella et al., Electrochim. Acta 388:138524 (2021), расчёт в sim-circuit";
const SRC_EA_EST: &str = "оценка: энергия активации R0 ячейки LG M50, для этой ячейки не измерена";

const LG: &str =
    "LG Chem, Product Specification INR21700 M50 18.2Wh, LRB-PS-CY18.2Wh-M50, 22.09.2017";
const LG_CIRCUIT: &str = "даташит LG M50 в записи sim-circuit::battery::CellParams::lg_m50";
const SAM: &str = "Samsung SDI, Specification of product INR18650-30Q, Version 0.0 (§3)";
const BLUE: &str =
    "Blue Robotics, INR18650-30Q Data Sheet: измерено AC 13.13 ± 2 мОм, DC 19.94 ± 2 мОм";
const MOLI: &str = "E-One Moli Energy, Product Data Sheet INR-21700-P42A";
const SONY: &str = "Sony, Technical Information US18650VTC6, Rev. 0.1, 20.01.2015";
const MURATA: &str = "Murata, US18650VTC6 Product Datasheet";
const LW: &str = "Lithium Werks, ANR26650M1B Datasheet, январь 2019";
const A123: &str = "A123 Systems, Nanophosphate High Power Cell ANR26650M1-B, MD100113-01";

impl CellSpec {
    /// LG INR21700-M50 (NMC811).
    pub fn lg_m50() -> CellSpec {
        CellSpec {
            name: "LG INR21700-M50".into(),
            chemistry: Chemistry::Nmc811,
            format: CellFormat::Cylindrical {
                diameter_mm: 21.10,
                height_mm: 70.15,
            },
            capacity_typ_ah: Param::literature(
                4.900,
                "Brosa Planella et al. 2021, C/10 при 25 °C; значение как в sim-circuit",
            ),
            capacity_min_ah: Param::datasheet(
                4.85,
                "LG M50: 0.2C = 970 мА, 0.3C = 1455 мА, то есть 1C = 4.85 А",
            ),
            v_nominal: Param::datasheet(3.63, LG),
            v_charge_max: Param::datasheet(4.20, LG),
            v_cutoff: Param::datasheet(2.50, LG),
            r_low_ohm: Param::datasheet(0.024, "LG M50: DCIR 30 ± 6 мОм, нижняя граница допуска"),
            r_dc_typ_ohm: Param::datasheet(0.030, LG_CIRCUIT),
            r_dc_high_ohm: Param::datasheet(
                0.036,
                "LG M50: DCIR 30 ± 6 мОм, верхняя граница допуска",
            ),
            i_discharge_cont_a: Param::datasheet(7.275, LG_CIRCUIT),
            i_discharge_cold_a: Param::datasheet(2.425, LG_CIRCUIT),
            discharge_cold_below_c: Param::datasheet(10.0, LG_CIRCUIT),
            i_pulse_a: Param::missing("импульсный ток в даташите LG M50 не заявлен"),
            pulse_s: Param::missing("импульсный ток в даташите LG M50 не заявлен"),
            i_charge_std_a: Param::datasheet(1.455, LG),
            charge_limits: vec![
                ChargeLimit {
                    up_to_c: 25.0,
                    current_a: 1.455,
                    source: LG_CIRCUIT,
                },
                ChargeLimit {
                    up_to_c: 45.0,
                    current_a: 3.395,
                    source: LG_CIRCUIT,
                },
            ],
            t_charge_min_c: Param::datasheet(0.0, LG),
            t_charge_max_c: Param::datasheet(45.0, LG),
            t_discharge_min_c: Param::datasheet(-20.0, LG),
            t_discharge_max_c: Param::datasheet(55.0, "LG M50 §2.7 (в §5.1 — 60 °C, взято строже)"),
            mass_kg: Param::datasheet(0.0685, LG_CIRCUIT),
            cp_j_kg_k: Param::literature(CP_M50, SRC_CP),
            ea_r_j_mol: Param::literature(EA_M50, SRC_EA),
            notes: "Таблицы токов в PDF даташита сделаны картинкой: токи и DCIR взяты из записи \
                    в sim-circuit, напряжения, окна температур и размеры сверены с текстом PDF.",
        }
    }

    /// Samsung INR18650-30Q.
    pub fn samsung_30q() -> CellSpec {
        CellSpec {
            name: "Samsung INR18650-30Q".into(),
            chemistry: Chemistry::NiRichUnspecified,
            format: CellFormat::Cylindrical {
                diameter_mm: 18.33,
                height_mm: 64.85,
            },
            capacity_typ_ah: Param::datasheet(3.0, SAM),
            capacity_min_ah: Param::datasheet(
                2.9,
                "Samsung 30Q: ёмкость при разряде 10 А, 2900 мА·ч",
            ),
            v_nominal: Param::datasheet(3.6, SAM),
            v_charge_max: Param::datasheet(4.20, SAM),
            v_cutoff: Param::datasheet(2.5, SAM),
            r_low_ohm: Param::literature(0.01113, BLUE),
            r_dc_typ_ohm: Param::literature(0.01994, BLUE),
            r_dc_high_ohm: Param::estimate(
                0.026 * 19.94 / 13.13,
                "оценка: предел AC ≤ 26 мОм (Samsung) × отношение DC/AC = 19.94/13.13 (Blue Robotics)",
            ),
            i_discharge_cont_a: Param::datasheet(15.0, "Samsung 30Q §3.7: 15 А при 25 °C"),
            i_discharge_cold_a: Param::missing("отдельного холодного предела в даташите нет"),
            discharge_cold_below_c: Param::missing("отдельного холодного предела в даташите нет"),
            i_pulse_a: Param::missing("импульсный ток в даташите 30Q не заявлен"),
            pulse_s: Param::missing("импульсный ток в даташите 30Q не заявлен"),
            i_charge_std_a: Param::datasheet(1.5, SAM),
            charge_limits: vec![ChargeLimit {
                up_to_c: 45.0,
                current_a: 4.0,
                source: "Samsung 30Q §3.4: быстрый заряд 4 А; верх окна — рекомендованные 45 °C",
            }],
            t_charge_min_c: Param::datasheet(0.0, SAM),
            t_charge_max_c: Param::datasheet(
                45.0,
                "Samsung 30Q §3.11: 0…50 °C, рекомендовано < 45 °C",
            ),
            t_discharge_min_c: Param::datasheet(-20.0, SAM),
            t_discharge_max_c: Param::datasheet(
                60.0,
                "Samsung 30Q §3.11: −20…75 °C, рекомендовано < 60 °C",
            ),
            mass_kg: Param::datasheet(0.048, "Samsung 30Q §3.9: не более 48.0 г"),
            cp_j_kg_k: Param::estimate(CP_M50, SRC_CP_EST),
            ea_r_j_mol: Param::estimate(EA_M50, SRC_EA_EST),
            notes: "Редакция V6 (2020) допускает 30 А с отсечкой по 80 °C; в каталоге взято 15 А \
                    без условия. Катод в даташите не назван.",
        }
    }

    /// Molicel INR-21700-P42A.
    pub fn molicel_p42a() -> CellSpec {
        CellSpec {
            name: "Molicel INR21700-P42A".into(),
            chemistry: Chemistry::NiRichUnspecified,
            format: CellFormat::Cylindrical {
                diameter_mm: 21.7,
                height_mm: 70.2,
            },
            capacity_typ_ah: Param::datasheet(4.2, MOLI),
            capacity_min_ah: Param::datasheet(4.0, MOLI),
            v_nominal: Param::datasheet(3.6, MOLI),
            v_charge_max: Param::datasheet(4.2, MOLI),
            v_cutoff: Param::datasheet(2.5, MOLI),
            r_low_ohm: Param::estimate(
                0.008,
                "оценка: типичное AC 10 мОм (Molicel) минус 20 % — допуск как у LG M50 (30 ± 6 мОм)",
            ),
            r_dc_typ_ohm: Param::datasheet(0.016, "Molicel P42A: DC 16 мОм (10 А, 1 с)"),
            r_dc_high_ohm: Param::estimate(
                0.024,
                "оценка: DC 16 мОм × 1.5 (отношение предела AC ≤ 15 мОм к типичному 10 мОм)",
            ),
            i_discharge_cont_a: Param::datasheet(45.0, MOLI),
            i_discharge_cold_a: Param::missing("отдельного холодного предела в даташите нет"),
            discharge_cold_below_c: Param::missing("отдельного холодного предела в даташите нет"),
            i_pulse_a: Param::missing("импульсный ток в даташите P42A не заявлен"),
            pulse_s: Param::missing("импульсный ток в даташите P42A не заявлен"),
            i_charge_std_a: Param::datasheet(4.2, MOLI),
            charge_limits: vec![ChargeLimit {
                up_to_c: 60.0,
                current_a: 4.2,
                source: MOLI,
            }],
            t_charge_min_c: Param::datasheet(0.0, MOLI),
            t_charge_max_c: Param::datasheet(60.0, MOLI),
            t_discharge_min_c: Param::datasheet(-40.0, MOLI),
            t_discharge_max_c: Param::datasheet(60.0, MOLI),
            mass_kg: Param::datasheet(0.070, "Molicel P42A: не более 70 г"),
            cp_j_kg_k: Param::estimate(CP_M50, SRC_CP_EST),
            ea_r_j_mol: Param::estimate(EA_M50, SRC_EA_EST),
            notes: "Независимые измерения дают меньший реальный длительный ток, чем 45 А: \
                    ограничивает нагрев, его считает тепловая проверка. Катод в даташите не назван.",
        }
    }

    /// Sony/Murata US18650VTC6.
    pub fn murata_vtc6() -> CellSpec {
        CellSpec {
            name: "Sony/Murata US18650VTC6".into(),
            chemistry: Chemistry::NiRichUnspecified,
            format: CellFormat::Cylindrical {
                diameter_mm: 18.50,
                height_mm: 65.20,
            },
            capacity_typ_ah: Param::datasheet(3.12, MURATA),
            capacity_min_ah: Param::datasheet(3.0, SONY),
            v_nominal: Param::datasheet(3.6, SONY),
            v_charge_max: Param::datasheet(4.2, SONY),
            v_cutoff: Param::datasheet(
                2.5,
                "Murata: рекомендуемая отсечка 2.5 В (2.0 В — только условие замера ёмкости)",
            ),
            r_low_ohm: Param::datasheet(0.008, "Murata VTC6: AC 1 кГц 8–18 мОм, нижняя граница"),
            r_dc_typ_ohm: Param::missing("DC-сопротивление в даташите не указано"),
            r_dc_high_ohm: Param::estimate(
                0.018 * 19.94 / 13.13,
                "оценка: AC ≤ 18 мОм (Murata) × отношение DC/AC ячейки 30Q = 19.94/13.13",
            ),
            i_discharge_cont_a: Param::datasheet(15.0, "Murata VTC6: 15 А без отсечки по 80 °C"),
            i_discharge_cold_a: Param::missing("отдельного холодного предела в даташите нет"),
            discharge_cold_below_c: Param::missing("отдельного холодного предела в даташите нет"),
            i_pulse_a: Param::missing(
                "импульс в даташите не заявлен; 30 А — только с отсечкой по 80 °C",
            ),
            pulse_s: Param::missing("импульс в даташите не заявлен"),
            i_charge_std_a: Param::datasheet(3.0, SONY),
            charge_limits: vec![
                ChargeLimit {
                    up_to_c: 10.0,
                    current_a: 3.0,
                    source: "оценка: 5 А разрешены только с 10 °C (Murata), ниже — стандартный ток 3 А",
                },
                ChargeLimit {
                    up_to_c: 45.0,
                    current_a: 5.0,
                    source: "Murata VTC6: до 5 А при 10…60 °C; верх окна взят строже, 45 °C",
                },
            ],
            t_charge_min_c: Param::datasheet(0.0, "Enepaq, модуль на VTC6: заряд 0…45 °C"),
            t_charge_max_c: Param::datasheet(
                45.0,
                "Enepaq, модуль на VTC6: заряд 0…45 °C (другие копии до 60 °C)",
            ),
            t_discharge_min_c: Param::datasheet(-20.0, "Enepaq, модуль на VTC6: разряд −20…60 °C"),
            t_discharge_max_c: Param::datasheet(60.0, "Enepaq, модуль на VTC6: разряд −20…60 °C"),
            mass_kg: Param::datasheet(0.0481, "Murata VTC6: 46.6 ± 1.5 г, верхняя граница"),
            cp_j_kg_k: Param::estimate(CP_M50, SRC_CP_EST),
            ea_r_j_mol: Param::estimate(EA_M50, SRC_EA_EST),
            notes: "Копии спецификации расходятся по окну заряда (0…45, 0…60 °C): взято узкое. \
                    PDF Murata зашифрован, его числа получены через поисковый индекс; масса, размеры \
                    и ёмкость сверены с Sony Rev. 0.1.",
        }
    }

    /// A123 / Lithium Werks ANR26650M1B (LiFePO₄).
    pub fn a123_anr26650() -> CellSpec {
        CellSpec {
            name: "A123 ANR26650M1B".into(),
            chemistry: Chemistry::Lfp,
            format: CellFormat::Cylindrical {
                diameter_mm: 26.0,
                height_mm: 65.0,
            },
            capacity_typ_ah: Param::datasheet(2.6, LW),
            capacity_min_ah: Param::datasheet(2.5, LW),
            v_nominal: Param::datasheet(3.3, LW),
            v_charge_max: Param::datasheet(3.6, LW),
            v_cutoff: Param::datasheet(2.0, "Lithium Werks / A123: конечное напряжение 2.0 В"),
            r_low_ohm: Param::estimate(
                0.0048,
                "оценка: типичное AC 6 мОм (A123) минус 20 % — допуск как у LG M50",
            ),
            r_dc_typ_ohm: Param::missing("DC-сопротивление в даташите не указано"),
            r_dc_high_ohm: Param::estimate(
                0.012,
                "оценка: AC 6 мОм × 2 (у 30Q и P42A DC/AC ≈ 1.5–1.6, взято с запасом)",
            ),
            i_discharge_cont_a: Param::datasheet(
                50.0,
                "Lithium Werks 2019: 50 А (ранние листы A123 — 70 А, взято строже)",
            ),
            i_discharge_cold_a: Param::missing("отдельного холодного предела в даташите нет"),
            discharge_cold_below_c: Param::missing("отдельного холодного предела в даташите нет"),
            i_pulse_a: Param::datasheet(120.0, LW),
            pulse_s: Param::datasheet(10.0, LW),
            i_charge_std_a: Param::datasheet(3.0, LW),
            charge_limits: vec![ChargeLimit {
                up_to_c: 55.0,
                current_a: 10.0,
                source: "Lithium Werks 2019: длительный заряд до 10 А",
            }],
            t_charge_min_c: Param::datasheet(
                0.0,
                "Lithium Werks 2019: 0…55 °C; ниже 0 °C допускает 250 мА, модуль это не разрешает",
            ),
            t_charge_max_c: Param::datasheet(55.0, LW),
            t_discharge_min_c: Param::datasheet(-30.0, A123),
            t_discharge_max_c: Param::datasheet(55.0, A123),
            mass_kg: Param::datasheet(0.076, A123),
            cp_j_kg_k: Param::estimate(CP_M50, SRC_CP_EST),
            ea_r_j_mol: Param::estimate(EA_M50, SRC_EA_EST),
            notes: "Таблица PDF Lithium Werks не извлекается как текст: её числа получены через \
                    поисковый индекс. Редакции расходятся по току разряда (50 и 70 А): взято 50 А.",
        }
    }

    /// LiPo-пачка с параметрами из формы. Пустое поле — `NotAnalyzed` в зависящих проверках.
    /// Нижняя и верхняя оценки сопротивления строятся от измеренного DC: AC/DC ≈ 0.6
    /// (30Q: 13.13/19.94, P42A: 10/16), верхняя граница ×1.5 (P42A: предел AC 15 к типичному 10).
    pub fn custom_lipo(input: &LipoInput) -> CellSpec {
        let r_low = input.r_dc_ohm.map(|r| r * 0.6);
        let r_high = input.r_dc_ohm.map(|r| r * 1.5);
        let est = |v: Option<f64>, source: &'static str| match v {
            Some(_) => Param {
                value: v,
                source,
                kind: SourceKind::Estimate,
            },
            None => Param::missing("не задано DC-сопротивление ячейки"),
        };
        let charge_limits = match (input.i_charge_a, input.t_charge_max_c) {
            (Some(current_a), Some(up_to_c)) => vec![ChargeLimit {
                up_to_c,
                current_a,
                source: SRC_USER,
            }],
            _ => Vec::new(),
        };
        CellSpec {
            name: input.name.clone(),
            chemistry: input.chemistry,
            format: CellFormat::Pouch {
                length_mm: input.length_mm,
                width_mm: input.width_mm,
                thickness_mm: input.thickness_mm,
            },
            capacity_typ_ah: Param::user(input.capacity_ah),
            capacity_min_ah: Param::user(input.capacity_ah),
            v_nominal: Param::user(input.v_nominal),
            v_charge_max: Param::user(input.v_charge_max),
            v_cutoff: Param::user(input.v_cutoff),
            r_low_ohm: est(
                r_low,
                "оценка: 0.6 × заданного DC (отношение AC/DC ячеек 30Q и P42A)",
            ),
            r_dc_typ_ohm: Param::user(input.r_dc_ohm),
            r_dc_high_ohm: est(r_high, "оценка: 1.5 × заданного DC (разброс как у P42A)"),
            i_discharge_cont_a: Param::user(input.i_cont_a),
            i_discharge_cold_a: Param::missing("для пачки не задан"),
            discharge_cold_below_c: Param::missing("для пачки не задан"),
            i_pulse_a: Param::user(input.i_pulse_a),
            pulse_s: Param::user(input.pulse_s),
            i_charge_std_a: Param::user(input.i_charge_a),
            charge_limits,
            t_charge_min_c: Param::user(input.t_charge_min_c),
            t_charge_max_c: Param::user(input.t_charge_max_c),
            t_discharge_min_c: Param::user(input.t_discharge_min_c),
            t_discharge_max_c: Param::user(input.t_discharge_max_c),
            mass_kg: Param::user(input.mass_g.map(|g| g / 1000.0)),
            cp_j_kg_k: Param::estimate(CP_M50, SRC_CP_EST),
            ea_r_j_mol: Param::estimate(EA_M50, SRC_EA_EST),
            notes: "Параметры заданы пользователем. Заявленный продавцом C-рейтинг LiPo часто \
                    завышен: вводите ток из даташита или измерения.",
        }
    }

    /// Все числовые параметры с именами (для проверки входа и показа источников).
    pub fn params(&self) -> Vec<(&'static str, Param)> {
        vec![
            ("ёмкость типичная", self.capacity_typ_ah),
            ("ёмкость минимальная", self.capacity_min_ah),
            ("номинальное напряжение", self.v_nominal),
            ("напряжение заряда", self.v_charge_max),
            ("конечное напряжение", self.v_cutoff),
            ("сопротивление нижнее", self.r_low_ohm),
            ("сопротивление типичное", self.r_dc_typ_ohm),
            ("сопротивление верхнее", self.r_dc_high_ohm),
            ("длительный ток", self.i_discharge_cont_a),
            ("ток на холоде", self.i_discharge_cold_a),
            ("граница холода", self.discharge_cold_below_c),
            ("импульсный ток", self.i_pulse_a),
            ("длительность импульса", self.pulse_s),
            ("стандартный ток заряда", self.i_charge_std_a),
            ("заряд, минимум °C", self.t_charge_min_c),
            ("заряд, максимум °C", self.t_charge_max_c),
            ("разряд, минимум °C", self.t_discharge_min_c),
            ("разряд, максимум °C", self.t_discharge_max_c),
            ("масса", self.mass_kg),
            ("теплоёмкость", self.cp_j_kg_k),
            ("энергия активации", self.ea_r_j_mol),
        ]
    }

    /// Предел длительного разряда при температуре, А.
    pub fn discharge_limit_a(&self, temp_c: f64) -> Option<f64> {
        if let (Some(cold), Some(below)) = (
            self.i_discharge_cold_a.value,
            self.discharge_cold_below_c.value,
        ) && temp_c <= below
        {
            return Some(cold);
        }
        self.i_discharge_cont_a.value
    }

    /// Предел тока заряда при температуре, А. Вне таблицы — `None`.
    pub fn charge_limit_a(&self, temp_c: f64) -> Option<f64> {
        self.charge_limits
            .iter()
            .find(|l| temp_c <= l.up_to_c)
            .map(|l| l.current_a)
    }
}

/// Форма LiPo-пачки: простые поля, `None` — не заполнено.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LipoInput {
    /// Название.
    pub name: String,
    /// Химия (по умолчанию LiCoO₂).
    pub chemistry: Chemistry,
    /// Ёмкость, А·ч.
    pub capacity_ah: Option<f64>,
    /// Номинальное напряжение, В.
    pub v_nominal: Option<f64>,
    /// Предельное напряжение заряда, В.
    pub v_charge_max: Option<f64>,
    /// Конечное напряжение, В.
    pub v_cutoff: Option<f64>,
    /// DC-сопротивление, Ом.
    pub r_dc_ohm: Option<f64>,
    /// Длительный ток разряда, А.
    pub i_cont_a: Option<f64>,
    /// Импульсный ток, А.
    pub i_pulse_a: Option<f64>,
    /// Длительность импульса, с.
    pub pulse_s: Option<f64>,
    /// Ток заряда, А.
    pub i_charge_a: Option<f64>,
    /// Заряд: минимум, °C.
    pub t_charge_min_c: Option<f64>,
    /// Заряд: максимум, °C.
    pub t_charge_max_c: Option<f64>,
    /// Разряд: минимум, °C.
    pub t_discharge_min_c: Option<f64>,
    /// Разряд: максимум, °C.
    pub t_discharge_max_c: Option<f64>,
    /// Масса, г.
    pub mass_g: Option<f64>,
    /// Длина, мм.
    pub length_mm: Option<f64>,
    /// Ширина, мм.
    pub width_mm: Option<f64>,
    /// Толщина, мм.
    pub thickness_mm: Option<f64>,
}

/// Весь каталог ячеек с готовыми данными.
pub fn catalog() -> Vec<CellSpec> {
    vec![
        CellSpec::lg_m50(),
        CellSpec::samsung_30q(),
        CellSpec::molicel_p42a(),
        CellSpec::murata_vtc6(),
        CellSpec::a123_anr26650(),
    ]
}

/// Ячейка каталога по точному названию.
pub fn find_cell(name: &str) -> Option<CellSpec> {
    catalog().into_iter().find(|c| c.name == name)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_catalog_value_has_a_source_and_ordered_limits() {
        for c in catalog() {
            for (name, p) in c.params() {
                assert!(!p.source.is_empty(), "{}: {name} без источника", c.name);
                if let Some(v) = p.value {
                    assert!(v.is_finite(), "{}: {name} = {v}", c.name);
                    assert_ne!(p.kind, SourceKind::Missing, "{}: {name}", c.name);
                } else {
                    assert_eq!(p.kind, SourceKind::Missing, "{}: {name}", c.name);
                }
            }
            let v = |p: Param| p.value.expect("обязательный параметр каталога");
            assert!(v(c.v_cutoff) < v(c.v_nominal) && v(c.v_nominal) < v(c.v_charge_max));
            assert!(v(c.r_low_ohm) < v(c.r_dc_high_ohm), "{}", c.name);
            assert!(v(c.capacity_min_ah) <= v(c.capacity_typ_ah), "{}", c.name);
            assert!(v(c.t_charge_min_c) < v(c.t_charge_max_c));
            assert!(v(c.t_discharge_min_c) < v(c.t_discharge_max_c));
            assert!(!c.charge_limits.is_empty(), "{}", c.name);
            assert!(
                c.charge_limits
                    .windows(2)
                    .all(|w| w[0].up_to_c < w[1].up_to_c),
                "{}",
                c.name
            );
            assert_eq!(find_cell(&c.name).as_ref(), Some(&c));
        }
    }

    #[test]
    fn cold_discharge_limit_is_applied() {
        let m50 = CellSpec::lg_m50();
        assert_eq!(m50.discharge_limit_a(5.0), Some(2.425));
        assert_eq!(m50.discharge_limit_a(25.0), Some(7.275));
    }

    #[test]
    fn charge_limit_outside_window_is_none() {
        let m50 = CellSpec::lg_m50();
        assert_eq!(m50.charge_limit_a(20.0), Some(1.455));
        assert_eq!(m50.charge_limit_a(40.0), Some(3.395));
        assert_eq!(m50.charge_limit_a(50.0), None);
    }

    #[test]
    fn empty_lipo_form_has_no_invented_numbers() {
        let c = CellSpec::custom_lipo(&LipoInput::default());
        for (name, p) in c.params() {
            if name == "теплоёмкость" || name == "энергия активации" {
                assert_eq!(p.kind, SourceKind::Estimate);
            } else {
                assert_eq!(p.value, None, "{name}");
            }
        }
    }
}
