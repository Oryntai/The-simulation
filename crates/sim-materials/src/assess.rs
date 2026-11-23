//! Единая оценка детали для вердикта «строить или нет».
//!
//! [`assess_part`] всегда возвращает по одной записи на каждый вид проверки: пропущенная
//! проверка не должна выглядеть как пройденная. Три исхода:
//! * `Pass` — посчитано, запас есть;
//! * `Fail` — посчитано, запаса нет, либо вход некорректен (NaN, бесконечность, вне диапазона);
//! * `NotAnalyzed` — данных или модели нет. Это **не** «годно»: см. [`overall`].
//!
//! Единицы входа — СИ: м, Н, Н·м, °C. Размеры проверяются на правдоподобие, чтобы миллиметры,
//! введённые в поле метров, давали отказ, а не красивый запас.

// Сравнения вида `!(a <= b)` здесь намеренные: при NaN они дают «не годно».
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use crate::catalog::{FatigueData, MPA, Material, MaterialClass, MaterialId, temperature_factor};
use crate::fasteners::{FastenerSpec, MemberStrength, PRINTED_THREAD_FACTOR, check_fasteners};
use crate::fatigue::{
    HCF_START, KC_AXIAL, KTS_HOLE, Reliability, SnCurve, StressBlock, Surface,
    fatigue_notch_factor, goodman_factor, kt_fillet_bending, kt_fillet_tension,
    kt_hole_bending_in_plane, kt_hole_bending_out_of_plane, kt_hole_tension, miner_stress_factor,
    notch_sensitivity_steel, size_factor, surface_factor,
};
use crate::printability::{PrintGeometry, check_printability};
use crate::printed::{PrintProcess, PrintedStrength, SHEAR_FROM_TENSION, printed_strength};
use crate::section::{
    BucklingRegime, EndCondition, Section, critical_load, rect_lateral_buckling,
    tube_flexure_compact_limit, tube_local_buckling_limit, von_mises,
};

/// Исход проверки.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    Pass,
    Fail,
    NotAnalyzed,
}

/// Вид проверки.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum FindingKind {
    /// Корректность входных данных (запись появляется только при ошибке).
    Input,
    Static,
    Fatigue,
    Buckling,
    Creep,
    Temperature,
    Fasteners,
    Printability,
}

impl FindingKind {
    /// Проверки, которые есть в каждом отчёте.
    pub const CHECKS: [FindingKind; 7] = [
        FindingKind::Static,
        FindingKind::Fatigue,
        FindingKind::Buckling,
        FindingKind::Creep,
        FindingKind::Temperature,
        FindingKind::Fasteners,
        FindingKind::Printability,
    ];

    /// Название для интерфейса.
    pub fn title(self) -> &'static str {
        match self {
            FindingKind::Input => "Входные данные",
            FindingKind::Static => "Статическая прочность",
            FindingKind::Fatigue => "Усталость",
            FindingKind::Buckling => "Потеря устойчивости",
            FindingKind::Creep => "Ползучесть",
            FindingKind::Temperature => "Температура",
            FindingKind::Fasteners => "Крепёж",
            FindingKind::Printability => "Печать",
        }
    }
}

/// Результат одной проверки. Смысл `value` и `limit` задан единицей:
/// `×` — запас и требуемый запас (годно при value ≥ limit); `Н`, `Н·м`, `МПа`, `°C` — действующая
/// величина и предел (годно при value ≤ limit); `доля` — суммарный коэффициент использования
/// при сжатии с изгибом (годно при value ≤ 1); `шт` — число блокеров и допустимое число.
#[derive(Clone, Debug, PartialEq)]
pub struct PartFinding {
    pub kind: FindingKind,
    pub outcome: Outcome,
    pub value: f64,
    pub limit: f64,
    pub unit: &'static str,
    pub detail: String,
}

/// Наименьший запас для всего, что носят на теле (ARCHITECTURE.md, требование владельца).
pub const BODY_WORN_MIN_MARGIN: f64 = 2.0;

/// Требуемые запасы.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Margins {
    /// Статический запас по текучести.
    pub static_yield: f64,
    /// Усталостный запас по напряжениям.
    pub fatigue: f64,
    /// Запас по критической силе. По умолчанию 3 — **оценка**: формулы идеального стержня не
    /// учитывают начальную кривизну и эксцентриситет, а потеря устойчивости внезапна.
    pub buckling: f64,
}

impl Default for Margins {
    fn default() -> Self {
        Self {
            static_yield: 2.0,
            fatigue: 2.0,
            buckling: 3.0,
        }
    }
}

/// Ориентация отверстия в прямоугольном сечении.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HoleOrientation {
    /// Ось отверстия вдоль `b`: изгиб идёт в плоскости полосы шириной `h` и толщиной `b`.
    InPlane,
    /// Ось отверстия вдоль `h`: полоса шириной `b` и толщиной `h` гнётся из своей плоскости.
    OutOfPlane,
}

/// Концентратор напряжений в расчётном сечении.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum StressRaiser {
    None,
    /// Поперечное отверстие диаметром `d`, м. Только прямоугольное сечение. При
    /// `centered: false` данных нет — статика и усталость «не посчитаны».
    Hole {
        d: f64,
        orientation: HoleOrientation,
        centered: bool,
    },
    /// Галтель перехода с высоты `big` на высоту `h` сечения, радиус `r`, м.
    Fillet {
        big: f64,
        r: f64,
    },
}

/// Настройки и геометрия печати.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PrintSpec {
    pub process: PrintProcess,
    pub geometry: PrintGeometry,
}

/// Как деталь крепится.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum Fastening {
    /// Не указано: проверка «не посчитана».
    #[default]
    NotSpecified,
    /// Крепежа нет — это указано явно.
    NoFasteners,
    /// Болтовое соединение.
    Bolted(FastenerSpec),
}

/// Собственная кривая S-N с испытаний этой детали или образцов с теми же настройками:
/// σ = a·N^b при R = −1. Единственный способ получить `Pass` по усталости печатной детали.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeasuredSn {
    /// Коэффициент, Па.
    pub a: f64,
    /// Показатель (отрицательный).
    pub b: f64,
    /// Наибольшее число циклов, до которого кривая подтверждена.
    pub max_cycles: f64,
}

/// Описание детали. Простые поля — заполняется из формы.
#[derive(Clone, Debug, PartialEq)]
pub struct PartSpec {
    pub name: String,
    pub material: MaterialId,
    /// Расчётное (наименьшее) сечение.
    pub section: Section,
    /// Длина стержня между опорами, м.
    pub length: f64,
    pub end: EndCondition,
    pub raiser: StressRaiser,
    /// Поверхность (для усталости металлов).
    pub surface: Surface,
    pub reliability: Reliability,
    /// Обязательно для печатного материала.
    pub print: Option<PrintSpec>,
    pub fastening: Fastening,
    /// Носится на теле: запасы не могут быть ниже [`BODY_WORN_MIN_MARGIN`].
    pub body_worn: bool,
    pub margins: Margins,
    pub measured_sn: Option<MeasuredSn>,
}

impl PartSpec {
    /// Деталь с безопасными умолчаниями: носится на теле, шарнирные опоры, надёжность 99%.
    pub fn new(
        name: impl Into<String>,
        material: MaterialId,
        section: Section,
        length: f64,
    ) -> Self {
        Self {
            name: name.into(),
            material,
            section,
            length,
            end: EndCondition::PinnedPinned,
            raiser: StressRaiser::None,
            surface: Surface::Machined,
            reliability: Reliability::R99,
            print: None,
            fastening: Fastening::NotSpecified,
            body_worn: true,
            margins: Margins::default(),
            measured_sn: None,
        }
    }

    /// Запасы, которые реально требуются.
    pub fn required_margins(&self) -> Margins {
        let mut m = self.margins;
        if self.body_worn {
            m.static_yield = m.static_yield.max(BODY_WORN_MIN_MARGIN);
            m.fatigue = m.fatigue.max(BODY_WORN_MIN_MARGIN);
            m.buckling = m.buckling.max(BODY_WORN_MIN_MARGIN);
        }
        m
    }
}

/// Блок спектра: доля пиковой нагрузки и число циклов.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LoadBlock {
    /// Доля пиковой нагрузки, 0..1.
    pub fraction: f64,
    pub cycles: f64,
}

/// Нагрузка на деталь. Силы и моменты — пиковые.
#[derive(Clone, Debug, PartialEq)]
pub struct LoadCase {
    /// Осевая сила, Н: + растяжение, − сжатие.
    pub axial: f64,
    /// Поперечная сила, Н.
    pub shear: f64,
    /// Изгибающий момент, Н·м.
    pub bending: f64,
    /// Крутящий момент, Н·м.
    pub torsion: f64,
    /// Температура детали, °C.
    pub temperature_c: f64,
    /// Требуемое число циклов (если `spectrum` пуст).
    pub cycles: f64,
    /// Коэффициент асимметрии R = σmin/σmax, −1..1. 0 — нагрузка-разгрузка.
    pub load_ratio: f64,
    /// Спектр нагрузки для правила Майнера (если задан, заменяет `cycles`).
    pub spectrum: Vec<LoadBlock>,
    /// Доля пиковой нагрузки, которая держится постоянно, 0..1.
    pub sustained_fraction: f64,
    /// Суммарное время под постоянной нагрузкой, ч.
    pub sustained_hours: f64,
    /// Суммарная сила среза на группу болтов, Н.
    pub fastener_shear: f64,
    /// Суммарная сила растяжения на группу болтов, Н.
    pub fastener_tension: f64,
}

impl Default for LoadCase {
    /// Нагрузки не заданы (деталь получит «не посчитано»), температура — температура тела.
    fn default() -> Self {
        Self {
            axial: 0.0,
            shear: 0.0,
            bending: 0.0,
            torsion: 0.0,
            temperature_c: 37.0,
            cycles: 0.0,
            load_ratio: 0.0,
            spectrum: Vec::new(),
            sustained_fraction: 0.0,
            sustained_hours: 0.0,
            fastener_shear: 0.0,
            fastener_tension: 0.0,
        }
    }
}

/// Рабочий уровень длительного напряжения для пластиков: 25% прочности. Упрощённое правило из
/// руководства Mitsubishi Chemical Advanced Materials «Engineering Thermoplastics Design Guide».
pub const CREEP_WORKING_FRACTION: f64 = 0.25;

/// Дольше этой выдержки данных о длительной прочности нет, ч (диапазон ASTM D2990 в типовых
/// испытаниях; 1000 ч без разрушения не доказывает бесконечную стойкость).
pub const CREEP_MAX_HOURS: f64 = 1000.0;

/// Итог по всем проверкам: любой `Fail` — отказ; иначе любой `NotAnalyzed` — не посчитано.
/// `Pass` только если все проверки посчитаны, числа конечны и список не пуст.
pub fn overall(findings: &[PartFinding]) -> Outcome {
    if findings.is_empty() {
        return Outcome::Fail;
    }
    let broken = |f: &PartFinding| !(f.value.is_finite() && f.limit.is_finite());
    if findings
        .iter()
        .any(|f| f.outcome == Outcome::Fail || broken(f))
    {
        Outcome::Fail
    } else if findings.iter().any(|f| f.outcome == Outcome::NotAnalyzed) {
        Outcome::NotAnalyzed
    } else {
        Outcome::Pass
    }
}

/// Оценка детали. Всегда содержит все виды из [`FindingKind::CHECKS`].
pub fn assess_part(part: &PartSpec, load: &LoadCase) -> Vec<PartFinding> {
    if let Err(reason) = validate(part, load) {
        let mut out = vec![PartFinding {
            kind: FindingKind::Input,
            outcome: Outcome::Fail,
            value: 0.0,
            limit: 0.0,
            unit: "",
            detail: format!("входные данные некорректны: {reason}"),
        }];
        out.extend(FindingKind::CHECKS.iter().map(|&kind| PartFinding {
            kind,
            outcome: Outcome::Fail,
            value: 0.0,
            limit: 0.0,
            unit: "",
            detail: format!("не считалось: {reason}"),
        }));
        return out;
    }

    let m = part.material.material();
    let printed = match (m.is_printed(), &part.print) {
        (true, Some(p)) => Some(printed_strength(
            m,
            &p.process,
            &part.section,
            load.temperature_c,
        )),
        _ => None,
    };
    let ctx = Ctx {
        part,
        load,
        m,
        printed,
        req: part.required_margins(),
    };
    let stress = nominal_stresses(part, load);

    let mut out = vec![
        ctx.static_finding(&stress),
        ctx.fatigue_finding(&stress),
        ctx.buckling_finding(),
        ctx.creep_finding(&stress),
        ctx.temperature_finding(),
        ctx.fastener_finding(),
        ctx.printability_finding(),
    ];
    // Последний рубеж: нечисловой результат никогда не становится «годно».
    for f in &mut out {
        if !(f.value.is_finite() && f.limit.is_finite()) {
            f.outcome = Outcome::Fail;
            f.detail = format!(
                "нечисловой результат ({} / {}): {}",
                f.value, f.limit, f.detail
            );
            f.value = 0.0;
            f.limit = 0.0;
        }
    }
    out
}

// --- Проверка входа ------------------------------------------------------------------------

/// Наибольший правдоподобный размер сечения, м: детали носимых устройств заведомо меньше.
const MAX_SECTION: f64 = 0.5;
/// Наибольшая правдоподобная длина, м.
const MAX_LENGTH: f64 = 5.0;

fn finite(name: &str, v: f64) -> Result<(), String> {
    if v.is_finite() {
        Ok(())
    } else {
        Err(format!("{name} = {v}"))
    }
}

fn validate(part: &PartSpec, load: &LoadCase) -> Result<(), String> {
    part.section.validate()?;
    if part.section.max_dimension() > MAX_SECTION {
        return Err(format!(
            "сечение {} м неправдоподобно велико (размеры вводятся в метрах)",
            part.section.max_dimension()
        ));
    }
    if !(part.length.is_finite() && part.length > 0.0 && part.length <= MAX_LENGTH) {
        return Err(format!("длина вне диапазона: {} м", part.length));
    }
    match part.raiser {
        StressRaiser::None => {}
        StressRaiser::Hole { d, orientation, .. } => {
            if !(d.is_finite() && d > 0.0) {
                return Err(format!("диаметр отверстия некорректен: {d}"));
            }
            if let Section::Rect { b, h } = part.section {
                let width = match orientation {
                    HoleOrientation::InPlane => h,
                    HoleOrientation::OutOfPlane => b,
                };
                if d >= width {
                    return Err(format!("отверстие {d} м не меньше ширины полосы {width} м"));
                }
            }
        }
        StressRaiser::Fillet { big, r } => {
            if !(big.is_finite() && r.is_finite() && r > 0.0) {
                return Err(format!(
                    "галтель некорректна: большая высота {big}, радиус {r}"
                ));
            }
            if let Section::Rect { h, .. } = part.section
                && big <= h
            {
                return Err("большая высота галтели не больше высоты сечения".into());
            }
        }
    }
    let mg = part.margins;
    for (name, v) in [
        ("статический запас", mg.static_yield),
        ("усталостный запас", mg.fatigue),
        ("запас устойчивости", mg.buckling),
    ] {
        if !(v.is_finite() && v >= 1.0) {
            return Err(format!("{name} должен быть не меньше 1: {v}"));
        }
    }
    finite("осевая сила", load.axial)?;
    finite("поперечная сила", load.shear)?;
    finite("изгибающий момент", load.bending)?;
    finite("крутящий момент", load.torsion)?;
    finite("сила среза на крепёж", load.fastener_shear)?;
    finite("сила растяжения на крепёж", load.fastener_tension)?;
    if !(load.temperature_c.is_finite() && load.temperature_c > -273.15) {
        return Err(format!("температура некорректна: {}", load.temperature_c));
    }
    if !(load.cycles.is_finite() && load.cycles >= 0.0) {
        return Err(format!("число циклов некорректно: {}", load.cycles));
    }
    if !(load.load_ratio.is_finite() && (-1.0..=1.0).contains(&load.load_ratio)) {
        return Err(format!(
            "коэффициент асимметрии вне −1..1: {}",
            load.load_ratio
        ));
    }
    for b in &load.spectrum {
        if !(b.fraction.is_finite() && b.fraction > 0.0 && b.fraction <= 1.0) {
            return Err(format!("доля нагрузки в блоке вне 0..1: {}", b.fraction));
        }
        if !(b.cycles.is_finite() && b.cycles >= 0.0) {
            return Err(format!("число циклов в блоке некорректно: {}", b.cycles));
        }
    }
    if !(load.sustained_fraction.is_finite() && (0.0..=1.0).contains(&load.sustained_fraction)) {
        return Err(format!(
            "доля постоянной нагрузки вне 0..1: {}",
            load.sustained_fraction
        ));
    }
    if !(load.sustained_hours.is_finite() && load.sustained_hours >= 0.0) {
        return Err(format!(
            "время под нагрузкой некорректно: {}",
            load.sustained_hours
        ));
    }
    if let Some(p) = &part.print {
        p.process.validate()?;
        p.geometry.validate()?;
    }
    if let Fastening::Bolted(f) = &part.fastening {
        f.validate()?;
    }
    if let Some(sn) = &part.measured_sn {
        if !(sn.a.is_finite() && sn.a > 0.0 && sn.b.is_finite() && sn.b < 0.0) {
            return Err(format!(
                "кривая S-N некорректна: a = {}, b = {}",
                sn.a, sn.b
            ));
        }
        if !(sn.max_cycles.is_finite() && sn.max_cycles >= HCF_START) {
            return Err(format!("база кривой S-N некорректна: {}", sn.max_cycles));
        }
    }
    Ok(())
}

// --- Напряжения ----------------------------------------------------------------------------

/// Номинальные напряжения в расчётном сечении (нетто) и коэффициенты концентрации.
#[derive(Clone, Copy, Debug)]
struct Nominal {
    axial: f64,
    bending: f64,
    shear: f64,
    torsion: f64,
    kt_axial: f64,
    kt_bending: f64,
    kt_shear: f64,
    /// Радиус надреза, м (0 — надреза нет).
    notch_r: f64,
}

/// Все напряжения по модулю: знак учитывается только для устойчивости.
fn nominal_stresses(part: &PartSpec, load: &LoadCase) -> Result<Nominal, String> {
    let s = &part.section;
    let m = part.material.material();
    let plain = Nominal {
        axial: load.axial.abs() / s.area(),
        bending: load.bending.abs() / s.section_modulus(),
        shear: s.shear_stress(load.shear),
        torsion: s.torsion_stress(load.torsion),
        kt_axial: 1.0,
        kt_bending: 1.0,
        kt_shear: 1.0,
        notch_r: 0.0,
    };
    if part.raiser == StressRaiser::None {
        return Ok(plain);
    }
    if m.class == MaterialClass::Composite {
        return Err(
            "коэффициенты Петерсона выведены для изотропного материала, \
                    у волокнистого композита концентрация другая"
                .into(),
        );
    }
    let Section::Rect { b, h } = *s else {
        return Err("концентраторы реализованы только для прямоугольного сечения".into());
    };
    match part.raiser {
        StressRaiser::None => Ok(plain),
        StressRaiser::Hole {
            d,
            orientation,
            centered,
        } => {
            if !centered {
                return Err("отверстие не на оси полосы: данных по такому Kt в модуле нет".into());
            }
            let (width, thick) = match orientation {
                HoleOrientation::InPlane => (h, b),
                HoleOrientation::OutOfPlane => (b, h),
            };
            let net_area = (width - d) * thick;
            let net_modulus = match orientation {
                HoleOrientation::InPlane => (h.powi(3) - d.powi(3)) * b / (6.0 * h),
                HoleOrientation::OutOfPlane => (b - d) * h * h / 6.0,
            };
            let kt_bending = match orientation {
                HoleOrientation::InPlane => kt_hole_bending_in_plane(d / h),
                HoleOrientation::OutOfPlane => kt_hole_bending_out_of_plane(d / b, d / h),
            };
            let net_ratio = width / (width - d);
            Ok(Nominal {
                axial: load.axial.abs() / net_area,
                bending: load.bending.abs() / net_modulus,
                shear: 1.5 * load.shear.abs() / net_area,
                torsion: plain.torsion * net_ratio,
                kt_axial: kt_hole_tension(d / width),
                kt_bending,
                kt_shear: KTS_HOLE,
                notch_r: d / 2.0,
            })
        }
        StressRaiser::Fillet { big, r } => {
            let kt_a = kt_fillet_tension(big, h, r);
            let kt_b = kt_fillet_bending(big, h, r);
            let (Some(kt_axial), Some(kt_bending)) = (kt_a, kt_b) else {
                return Err(format!(
                    "галтель вне данных Петерсона: уступ/радиус = {:.1} (предел 20)",
                    (big - h) / 2.0 / r
                ));
            };
            Ok(Nominal {
                kt_axial,
                kt_bending,
                // Для кручения плоского уступа формулы нет: берём большее из двух, в запас.
                kt_shear: kt_axial.max(kt_bending),
                notch_r: r,
                ..plain
            })
        }
    }
}

// --- Проверки ------------------------------------------------------------------------------

struct Ctx<'a> {
    part: &'a PartSpec,
    load: &'a LoadCase,
    m: &'static Material,
    printed: Option<PrintedStrength>,
    req: Margins,
}

fn finding(
    kind: FindingKind,
    outcome: Outcome,
    value: f64,
    limit: f64,
    unit: &'static str,
    detail: String,
) -> PartFinding {
    PartFinding {
        kind,
        outcome,
        value,
        limit,
        unit,
        detail,
    }
}

fn not_analyzed(
    kind: FindingKind,
    limit: f64,
    unit: &'static str,
    detail: impl Into<String>,
) -> PartFinding {
    finding(kind, Outcome::NotAnalyzed, 0.0, limit, unit, detail.into())
}

/// `Pass`, только если запас — число и не меньше требуемого (NaN даёт `Fail`).
fn margin_outcome(n: f64, required: f64) -> Outcome {
    if n.is_finite() && n >= required {
        Outcome::Pass
    } else {
        Outcome::Fail
    }
}

impl Ctx<'_> {
    fn k_temp(&self) -> f64 {
        temperature_factor(self.m, self.load.temperature_c)
    }

    fn no_load(&self) -> bool {
        let l = self.load;
        l.axial == 0.0 && l.shear == 0.0 && l.bending == 0.0 && l.torsion == 0.0
    }

    fn missing_print(&self) -> bool {
        self.m.is_printed() && self.printed.is_none()
    }

    /// Модуль и «предел» для расчёта стержня.
    fn column_props(&self) -> (f64, f64) {
        let k = self.k_temp();
        match (self.m.class, self.printed) {
            (MaterialClass::Printed, Some(p)) => {
                (p.modulus * p.fill.stiffness, p.yield_strength * p.fill.area)
            }
            (MaterialClass::Composite, _) => (
                self.m.e_compression.unwrap_or(self.m.e_along) * k,
                self.m.compressive.unwrap_or(self.m.su_along) * k,
            ),
            _ => (self.m.e_along * k, self.m.sy_along * k),
        }
    }

    /// Критическая сила; `None` — материал потерял несущую способность.
    fn pcr(&self) -> Option<crate::section::Buckling> {
        let (e, sy) = self.column_props();
        if e > 0.0 && sy > 0.0 {
            Some(critical_load(
                &self.part.section,
                e,
                sy,
                self.part.length,
                self.part.end,
            ))
        } else {
            None
        }
    }

    /// Усиление изгиба сжимающей силой: 1/(1 − P/Pcr) (балка-колонна, AISC 360, прил. 8).
    fn bending_amplification(&self) -> f64 {
        if self.load.axial >= 0.0 || self.load.bending == 0.0 {
            return 1.0;
        }
        match self.pcr() {
            Some(b) if self.load.axial.abs() < b.critical_load => {
                1.0 / (1.0 - self.load.axial.abs() / b.critical_load)
            }
            _ => f64::INFINITY,
        }
    }

    /// Эквивалентное напряжение и запас при статике: `(σэкв, запас, пояснение)`.
    fn static_margin(&self, st: &Nominal) -> (f64, f64, String) {
        let amp = self.bending_amplification();
        let bend = if st.bending == 0.0 {
            0.0
        } else {
            st.bending * amp
        };
        let k = self.k_temp();
        match self.m.class {
            MaterialClass::Metal => {
                // Пластичный металл: местная текучесть перераспределяет напряжение, Kt в
                // статике не применяется (Shigley, §3-13). Для малопластичного — применяется.
                let (sigma, tau) = if self.m.is_ductile() {
                    (st.axial + bend, st.shear + st.torsion)
                } else {
                    (
                        st.kt_axial * st.axial + st.kt_bending * bend,
                        st.kt_shear * (st.shear + st.torsion),
                    )
                };
                let vm = von_mises(sigma, tau);
                let sy = self.m.sy_along * k;
                (
                    vm,
                    sy / vm,
                    format!("σэкв {:.1} МПа при пределе {:.1} МПа", vm / MPA, sy / MPA),
                )
            }
            MaterialClass::Printed => {
                let p = self.printed.expect("проверено выше");
                let sigma =
                    st.kt_axial * st.axial / p.fill.area + st.kt_bending * bend / p.fill.bending;
                let tau = st.kt_shear * (st.shear + st.torsion) / p.fill.area;
                let vm = von_mises(sigma, tau);
                let n_vm = p.yield_strength / vm;
                let n_shear = if tau > 0.0 {
                    p.interlayer_shear / tau
                } else {
                    f64::MAX
                };
                let (n, what) = if n_shear < n_vm {
                    (n_shear, "межслойный сдвиг")
                } else {
                    (n_vm, "напряжение в направлении нагрузки")
                };
                (
                    vm,
                    n,
                    format!(
                        "σэкв {:.1} МПа, прочность {:.1} МПа (поперёк слоёв {:.1}), определяет: {what}",
                        vm / MPA,
                        p.yield_strength / MPA,
                        p.across / MPA
                    ),
                )
            }
            MaterialClass::Composite => {
                let tension_only = self.load.axial >= 0.0 && bend == 0.0;
                let s = if tension_only {
                    self.m.su_along
                } else {
                    self.m.compressive.unwrap_or(self.m.su_along)
                } * k;
                let sigma = st.axial + bend;
                let tau = st.shear + st.torsion;
                let n_sigma = if sigma > 0.0 { s / sigma } else { f64::MAX };
                let ils = self.m.interlaminar_shear.unwrap_or(self.m.su_across) * k;
                let n_tau = if tau > 0.0 { ils / tau } else { f64::MAX };
                let vm = von_mises(sigma, tau);
                (
                    vm,
                    n_sigma.min(n_tau),
                    format!(
                        "σ {:.1} МПа при {:.1} МПа, τ {:.1} МПа при межслойном {:.1} МПа",
                        sigma / MPA,
                        s / MPA,
                        tau / MPA,
                        ils / MPA
                    ),
                )
            }
        }
    }

    fn static_finding(&self, st: &Result<Nominal, String>) -> PartFinding {
        let kind = FindingKind::Static;
        let req = self.req.static_yield;
        if self.no_load() {
            return not_analyzed(kind, req, "×", "нагрузка не задана");
        }
        if self.missing_print() {
            return not_analyzed(kind, req, "×", "печатный материал без настроек печати");
        }
        let st = match st {
            Ok(s) => s,
            Err(e) => return not_analyzed(kind, req, "×", e.clone()),
        };
        let (_, n, text) = self.static_margin(st);
        finding(
            kind,
            margin_outcome(n, req),
            n,
            req,
            "×",
            format!("запас {n:.2} при требуемом {req:.2}: {text}"),
        )
    }

    /// Усталостная кривая материала с поправками или причина, почему её нет.
    fn fatigue_curve(&self, total_cycles: f64) -> Result<(SnCurve, f64), String> {
        let k_t = self.k_temp();
        let m = self.m;
        let sut = match self.printed {
            Some(p) => p.ultimate,
            None => m.su_along * k_t,
        };
        if let Some(sn) = self.part.measured_sn {
            if total_cycles > sn.max_cycles {
                return Err(format!(
                    "{total_cycles:.2e} циклов больше базы испытаний {:.2e}",
                    sn.max_cycles
                ));
            }
            let curve = SnCurve {
                a: sn.a * k_t,
                b: sn.b,
                knee_cycles: sn.max_cycles,
                knee_strength: sn.a * k_t * sn.max_cycles.powf(sn.b),
                endurance_limit: false,
            };
            return Ok((curve, sut));
        }
        let sut_mpa = m.su_along / MPA;
        let marin = surface_factor(self.part.surface, sut_mpa)
            * size_factor(self.part.section.max_dimension())
            * self.part.reliability.factor()
            * k_t;
        match m.fatigue {
            FatigueData::NoReliableData { reason } => Err(reason.to_string()),
            FatigueData::SteelEnduranceRule => {
                let se = 0.5 * m.su_along.min(1400.0 * MPA) * marin;
                Ok((SnCurve::from_sut(sut, se, 1e6, true), sut))
            }
            FatigueData::Measured { sf, n_ref } => {
                if total_cycles > n_ref {
                    return Err(format!(
                        "{total_cycles:.2e} циклов больше базы данных материала {n_ref:.2e}; \
                         предела выносливости у него нет"
                    ));
                }
                Ok((SnCurve::from_sut(sut, sf * marin, n_ref, false), sut))
            }
        }
    }

    fn fatigue_finding(&self, st: &Result<Nominal, String>) -> PartFinding {
        let kind = FindingKind::Fatigue;
        let req = self.req.fatigue;
        if self.no_load() {
            return not_analyzed(kind, req, "×", "нагрузка не задана");
        }
        if self.missing_print() {
            return not_analyzed(kind, req, "×", "печатный материал без настроек печати");
        }
        let st = match st {
            Ok(s) => s,
            Err(e) => return not_analyzed(kind, req, "×", e.clone()),
        };
        let l = self.load;
        let total: f64 = if l.spectrum.is_empty() {
            l.cycles
        } else {
            l.spectrum.iter().map(|b| b.cycles).sum()
        };
        if total <= 0.0 {
            return not_analyzed(kind, req, "×", "число циклов не задано");
        }

        // Эффективные коэффициенты: чувствительность к надрезу считается только для стали
        // по кривой Шигли, для остальных q = 1 (полная концентрация — в запас).
        let steel = self.m.fatigue == FatigueData::SteelEnduranceRule;
        let (q, qs) = if steel && st.notch_r > 0.0 {
            (
                notch_sensitivity_steel(self.m.su_along, st.notch_r, false),
                notch_sensitivity_steel(self.m.su_along, st.notch_r, true),
            )
        } else {
            (1.0, 1.0)
        };
        let kf_a = fatigue_notch_factor(st.kt_axial, q);
        let kf_b = fatigue_notch_factor(st.kt_bending, q);
        let kf_s = fatigue_notch_factor(st.kt_shear, qs);

        // Неполное заполнение печатной детали повышает напряжение в несущем материале.
        let (phi_a, phi_b) = match self.printed {
            Some(p) => (p.fill.area, p.fill.bending),
            None => (1.0, 1.0),
        };
        let amp = self.bending_amplification();
        let bend = if st.bending == 0.0 {
            0.0
        } else {
            kf_b * st.bending * amp / phi_b
        };
        let ax = kf_a * st.axial / phi_a;
        let tau = kf_s * (st.shear + st.torsion) / phi_a;

        // Эквиваленты для амплитуды и среднего (Shigley, §6-16): осевая часть амплитуды
        // делится на kc, потому что кривая S-N построена для изгиба.
        let peak_a = von_mises(bend + ax / KC_AXIAL, tau);
        let peak_m = von_mises(bend + ax, tau);
        let sigma_a = peak_a * (1.0 - l.load_ratio) / 2.0;
        let sigma_m = peak_m * (1.0 + l.load_ratio) / 2.0;

        // Строгая граница, не требующая данных: если пик цикла не меньше статической
        // прочности, деталь не переживёт и первых циклов.
        let ultimate = match self.printed {
            Some(p) => p.ultimate,
            None => self.m.su_along * self.k_temp(),
        };
        let peak = von_mises(bend + ax, tau);
        if !(peak < ultimate) {
            return finding(
                kind,
                Outcome::Fail,
                ultimate / peak,
                req,
                "×",
                format!(
                    "местный пик {:.1} МПа не меньше прочности {:.1} МПа: разрушение в первых циклах",
                    peak / MPA,
                    ultimate / MPA
                ),
            );
        }

        let (curve, sut) = match self.fatigue_curve(total) {
            Ok(c) => c,
            Err(reason) => {
                return not_analyzed(kind, req, "×", format!("усталость не посчитана: {reason}"));
            }
        };
        let (n, how) = if l.spectrum.is_empty() {
            let sf = curve.strength_at(l.cycles);
            (
                goodman_factor(sigma_a, sigma_m, sf, sut),
                format!("Гудман, Sf({:.1e}) = {:.1} МПа", l.cycles, sf / MPA),
            )
        } else {
            let blocks: Vec<StressBlock> = l
                .spectrum
                .iter()
                .map(|b| StressBlock {
                    sigma_a: sigma_a * b.fraction,
                    sigma_m: sigma_m * b.fraction,
                    cycles: b.cycles,
                })
                .collect();
            (
                miner_stress_factor(&curve, sut, &blocks),
                format!("Майнер, {} блоков, {total:.1e} циклов", blocks.len()),
            )
        };
        finding(
            kind,
            margin_outcome(n, req),
            n,
            req,
            "×",
            format!(
                "запас {n:.2} при требуемом {req:.2}: σa {:.1} МПа, σm {:.1} МПа, Kf {:.2}/{:.2}; {how}",
                sigma_a / MPA,
                sigma_m / MPA,
                kf_a,
                kf_b
            ),
        )
    }

    /// Три формы потери устойчивости: стержень при сжатии (Эйлер/Джонсон), боковое выпучивание
    /// высокого бруса при изгибе (AISC 360-16, F11), смятие тонкой стенки трубы. При сжатии с
    /// изгибом коэффициенты использования складываются — линейная интерполяция в запас.
    fn buckling_finding(&self) -> PartFinding {
        let kind = FindingKind::Buckling;
        let l = self.load;
        if self.no_load() {
            return not_analyzed(kind, 0.0, "Н", "нагрузка не задана");
        }
        let compress = l.axial < 0.0;
        let bends = l.bending != 0.0;
        let tall_bar = matches!(self.part.section, Section::Rect { b, h } if h > b);
        let tube = matches!(self.part.section, Section::Tube { .. });
        let wall_loaded = bends || l.shear != 0.0 || l.torsion != 0.0;
        if !(compress || (bends && tall_bar) || (tube && wall_loaded)) {
            return finding(
                kind,
                Outcome::Pass,
                0.0,
                0.0,
                "Н",
                "сжатия нет, форма сечения не склонна к выпучиванию".into(),
            );
        }
        if self.missing_print() {
            return not_analyzed(kind, 0.0, "Н", "печатный материал без настроек печати");
        }
        if self.m.id == MaterialId::Tpu95a {
            return not_analyzed(
                kind,
                0.0,
                "Н",
                "TPU: модуль нелинеен, формулы стержня неприменимы",
            );
        }
        if self.m.class == MaterialClass::Composite && bends && tall_bar {
            return not_analyzed(
                kind,
                0.0,
                "Н·м",
                "боковое выпучивание ортотропного бруса не считается",
            );
        }
        let req = self.req.buckling;
        let (e, sy) = self.column_props();
        if !(e > 0.0 && sy > 0.0) {
            return finding(
                kind,
                Outcome::Fail,
                l.axial.abs(),
                0.0,
                "Н",
                "материал при этой температуре не несёт нагрузку".into(),
            );
        }

        let force = if compress { l.axial.abs() } else { 0.0 };
        let moment = l.bending.abs();
        let column = if compress { self.pcr() } else { None };
        let p_limit = column.map(|b| b.critical_load / req);
        let m_limit = match self.part.section {
            Section::Rect { b, h } if bends => {
                let unbraced = self.part.end.k() * self.part.length;
                rect_lateral_buckling(h, b, e, sy, unbraced).map(|m| m / req)
            }
            _ => None,
        };
        let use_p = p_limit.map_or(0.0, |lim| force / lim);
        let use_m = m_limit.map_or(0.0, |lim| moment / lim);
        let total = use_p + use_m;
        let (value, limit, unit) = match (p_limit, m_limit) {
            (Some(p), None) => (force, p, "Н"),
            (None, Some(m)) => (moment, m, "Н·м"),
            (Some(_), Some(_)) => (total, 1.0, "доля"),
            (None, None) => (0.0, 0.0, "Н"),
        };

        let mut text = Vec::new();
        if let Some(b) = column {
            let regime = match b.regime {
                BucklingRegime::Euler => "Эйлер",
                BucklingRegime::Johnson => "Джонсон",
            };
            text.push(format!(
                "сжатие {force:.0} Н, Pcr {:.0} Н ({regime}, λ = {:.0}, λт = {:.0})",
                b.critical_load, b.slenderness, b.transition
            ));
        }
        if let Some(m) = m_limit {
            text.push(format!(
                "изгиб {moment:.2} Н·м, боковое выпучивание {:.2} Н·м",
                m * req
            ));
        }
        text.push(format!("запас {req:.1}"));
        let text = text.join("; ");

        if !(total <= 1.0) {
            return finding(kind, Outcome::Fail, value, limit, unit, text);
        }
        // Тонкая стенка может смяться раньше: этой формы модуль не считает.
        if let Section::Tube { od, wall } = self.part.section {
            let limit_dt = if wall_loaded {
                tube_flexure_compact_limit(e, sy).min(tube_local_buckling_limit(e, sy))
            } else {
                tube_local_buckling_limit(e, sy)
            };
            if od / wall > limit_dt {
                return finding(
                    kind,
                    Outcome::NotAnalyzed,
                    value,
                    limit,
                    unit,
                    format!(
                        "D/t = {:.1} больше {limit_dt:.1}: местная потеря устойчивости стенки \
                         не считается; {text}",
                        od / wall
                    ),
                );
            }
        }
        finding(kind, Outcome::Pass, value, limit, unit, text)
    }

    fn creep_finding(&self, st: &Result<Nominal, String>) -> PartFinding {
        let kind = FindingKind::Creep;
        let l = self.load;
        if self.no_load() {
            return not_analyzed(kind, 0.0, "МПа", "нагрузка не задана");
        }
        if l.sustained_fraction == 0.0 || l.sustained_hours == 0.0 {
            return finding(
                kind,
                Outcome::Pass,
                0.0,
                0.0,
                "МПа",
                "длительной нагрузки нет".into(),
            );
        }
        if self.missing_print() {
            return not_analyzed(kind, 0.0, "МПа", "печатный материал без настроек печати");
        }
        let st = match st {
            Ok(s) => s,
            Err(e) => return not_analyzed(kind, 0.0, "МПа", e.clone()),
        };
        let (vm, _, _) = self.static_margin(st);
        let sigma = vm * l.sustained_fraction;
        match self.m.class {
            MaterialClass::Composite => not_analyzed(
                kind,
                0.0,
                "МПа",
                "данных о длительной прочности этой трубы нет",
            ),
            MaterialClass::Metal => {
                if l.temperature_c > self.m.max_service_c {
                    return not_analyzed(
                        kind,
                        0.0,
                        "МПа",
                        "выше рабочей температуры ползучесть не считается",
                    );
                }
                let limit = self.m.sy_along * self.k_temp();
                let outcome = if sigma <= limit {
                    Outcome::Pass
                } else {
                    Outcome::Fail
                };
                finding(
                    kind,
                    outcome,
                    sigma / MPA,
                    limit / MPA,
                    "МПа",
                    "металл в пределах рабочей температуры: заметной ползучести нет".into(),
                )
            }
            MaterialClass::Printed => {
                let p = self.printed.expect("проверено выше");
                let limit = CREEP_WORKING_FRACTION * p.yield_strength;
                let (value, lim) = (sigma / MPA, limit / MPA);
                if !(sigma <= limit) {
                    return finding(
                        kind,
                        Outcome::Fail,
                        value,
                        lim,
                        "МПа",
                        format!(
                            "постоянное напряжение {value:.1} МПа выше рабочего уровня {lim:.1} МПа \
                             (25% прочности при этой температуре)"
                        ),
                    );
                }
                if let Some(tg) = self.m.tg_c
                    && l.temperature_c >= tg
                {
                    return finding(
                        kind,
                        Outcome::NotAnalyzed,
                        value,
                        lim,
                        "МПа",
                        format!(
                            "{:.0} °C не ниже стеклования {tg:.0} °C: данных о ползучести нет",
                            l.temperature_c
                        ),
                    );
                }
                if l.sustained_hours > CREEP_MAX_HOURS {
                    return finding(
                        kind,
                        Outcome::NotAnalyzed,
                        value,
                        lim,
                        "МПа",
                        format!(
                            "{:.0} ч под нагрузкой больше {CREEP_MAX_HOURS:.0} ч: данных нет",
                            l.sustained_hours
                        ),
                    );
                }
                finding(
                    kind,
                    Outcome::Pass,
                    value,
                    lim,
                    "МПа",
                    format!("постоянное {value:.1} МПа при рабочем уровне {lim:.1} МПа"),
                )
            }
        }
    }

    fn temperature_finding(&self) -> PartFinding {
        let kind = FindingKind::Temperature;
        let (t, limit) = (self.load.temperature_c, self.m.max_service_c);
        if t > limit {
            let soft = self
                .m
                .softening_c
                .map(|s| format!(", размягчение {s:.0} °C"))
                .unwrap_or_default();
            return finding(
                kind,
                Outcome::Fail,
                t,
                limit,
                "°C",
                format!("{t:.0} °C выше предельной рабочей {limit:.0} °C{soft}"),
            );
        }
        // Печатные и композит — нет данных о хрупкости на холоде. S235JR: ударная вязкость
        // гарантирована только при +20 °C (EN 10025-2), у ОЦК-стали есть порог хладноломкости.
        let cold_unknown =
            self.m.class != MaterialClass::Metal || self.m.id == MaterialId::SteelS235;
        if cold_unknown && t < 0.0 {
            return finding(
                kind,
                Outcome::NotAnalyzed,
                t,
                limit,
                "°C",
                format!(
                    "{t:.0} °C: данных о хрупкости {} на холоде нет",
                    self.m.name
                ),
            );
        }
        finding(
            kind,
            Outcome::Pass,
            t,
            limit,
            "°C",
            format!("{t:.0} °C при пределе {limit:.0} °C"),
        )
    }

    fn fastener_finding(&self) -> PartFinding {
        let kind = FindingKind::Fasteners;
        let req = self.req.static_yield;
        let l = self.load;
        let has_force = l.fastener_shear != 0.0 || l.fastener_tension != 0.0;
        let spec = match &self.part.fastening {
            Fastening::NotSpecified => return not_analyzed(kind, req, "×", "крепёж не указан"),
            Fastening::NoFasteners => {
                return if has_force {
                    finding(
                        kind,
                        Outcome::Fail,
                        0.0,
                        req,
                        "×",
                        "заданы силы на крепёж, но крепежа нет".into(),
                    )
                } else {
                    finding(kind, Outcome::Pass, 0.0, 0.0, "×", "крепежа нет".into())
                };
            }
            Fastening::Bolted(s) => s,
        };
        if !has_force {
            return not_analyzed(kind, req, "×", "силы на крепёж не заданы");
        }
        if self.missing_print() {
            return not_analyzed(kind, req, "×", "печатный материал без настроек печати");
        }
        let k = self.k_temp();
        let member = match (self.m.class, self.printed) {
            (MaterialClass::Printed, Some(p)) => MemberStrength {
                bearing: p.across,
                shear: p.interlayer_shear,
                thread_factor: PRINTED_THREAD_FACTOR,
                model_applies: true,
            },
            (MaterialClass::Composite, _) => MemberStrength {
                bearing: 0.0,
                shear: 0.0,
                thread_factor: 0.0,
                model_applies: false,
            },
            _ => MemberStrength {
                bearing: self.m.sy_along * k,
                shear: SHEAR_FROM_TENSION * self.m.sy_along * k,
                thread_factor: 1.0,
                model_applies: true,
            },
        };
        let cycles: f64 = if l.spectrum.is_empty() {
            l.cycles
        } else {
            l.spectrum.iter().map(|b| b.cycles).sum()
        };
        let report = check_fasteners(
            spec,
            &member,
            l.fastener_shear,
            l.fastener_tension,
            cycles,
            l.load_ratio,
        );
        let label = format!(
            "{} {} × {}",
            spec.size.name(),
            spec.grade.name(),
            spec.count
        );
        let governing = report.governing().cloned();
        let (n, what) = governing
            .map(|g| (g.factor, g.name))
            .unwrap_or((0.0, "не посчитано"));
        if !report.geometry_faults.is_empty() {
            return finding(
                kind,
                Outcome::Fail,
                n,
                req,
                "×",
                format!("{label}: {}", report.geometry_faults.join("; ")),
            );
        }
        if !(n.is_finite() && n >= req) && what != "не посчитано" {
            return finding(
                kind,
                Outcome::Fail,
                n,
                req,
                "×",
                format!("{label}: запас {n:.2} при требуемом {req:.2}, определяет {what}"),
            );
        }
        if !report.not_analyzed.is_empty() || what == "не посчитано" {
            return finding(
                kind,
                Outcome::NotAnalyzed,
                n,
                req,
                "×",
                format!("{label}: {}", report.not_analyzed.join("; ")),
            );
        }
        finding(
            kind,
            Outcome::Pass,
            n,
            req,
            "×",
            format!("{label}: запас {n:.2}, определяет {what}"),
        )
    }

    fn printability_finding(&self) -> PartFinding {
        let kind = FindingKind::Printability;
        if !self.m.is_printed() {
            return finding(
                kind,
                Outcome::Pass,
                0.0,
                0.0,
                "шт",
                "деталь не печатается".into(),
            );
        }
        let Some(spec) = &self.part.print else {
            return not_analyzed(kind, 0.0, "шт", "настройки печати не заданы");
        };
        let r = check_printability(self.m, &spec.process, &spec.geometry);
        let mut text = if r.blockers.is_empty() {
            "можно напечатать".to_string()
        } else {
            format!("блокеры: {}", r.blockers.join("; "))
        };
        if !r.warnings.is_empty() {
            text.push_str(&format!(". Предупреждения: {}", r.warnings.join("; ")));
        }
        let outcome = if r.printable() {
            Outcome::Pass
        } else {
            Outcome::Fail
        };
        finding(kind, outcome, r.blockers.len() as f64, 0.0, "шт", text)
    }
}
