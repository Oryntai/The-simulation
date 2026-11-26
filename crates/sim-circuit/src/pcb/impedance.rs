//! Квазистатический Z₀ прямого наружного микрополоска над соседней плоскостью.
//! Модель Хаммерстада–Йенсена, DOI 10.1109/MWSYM.1980.1124303. Геометрия из
//! KiCad 8/9, разделы [Tracks](https://dev-docs.kicad.org/en/file-formats/sexpr-pcb/#tracks-section)
//! и [Stack Up Settings](https://dev-docs.kicad.org/en/file-formats/sexpr-pcb/#stack-up-settings).
//! Предельное сравнение относится к сечению трассы, а вывод о физической
//! сборке требует подтверждения применимости в полосе и той же ревизии.

use std::fmt;

use super::layout::{Centerline, Layout, NotAnalyzed};
use super::microstrip_hammerstad_jensen;
use super::ratings::{Datum, Scope, Source};
use super::return_path::{self, ReferenceInput};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProcessWindow<'a> {
    pub width_m: [f64; 2],
    /// Толщина диэлектрика между поверхностями меди, без полутолщин меди.
    pub dielectric_height_m: [f64; 2],
    pub trace_copper_m: [f64; 2],
    /// Огибающая εr материала в полной рабочей полосе, включая технологический разброс.
    pub epsilon_r: [f64; 2],
    pub band_hz: [f64; 2],
    pub source: Source<'a>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Limit<'a> {
    /// Допустимый Z₀ в полной рабочей полосе.
    pub ohms: [f64; 2],
    pub source: Source<'a>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mark<'a> {
    /// Индекс сегмента или дуги в нормализованной плате и адрес её опоры.
    pub reference: ReferenceInput<'a>,
    pub process: Option<ProcessWindow<'a>>,
    pub limit: Option<Limit<'a>>,
    /// Верхняя граница абсолютного отклонения от модели, в Ом. Включает
    /// дисперсию, потери, маску, конечную плоскость, соседнюю медь и погрешность
    /// сверки на всём заявленном диапазоне геометрии и частот. Для сборки
    /// применима стендовая сверка той же ревизии, например TDR.
    pub model_error_ohm: Option<Datum<'a>>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Open {
    Layout(NotAnalyzed),
    ReturnPath(return_path::Open),
    InterruptedReturn,
    UnsupportedGeometry,
    OutsideModel,
    MissingOrUnverified(&'static str),
    OverlapsLimit,
}

impl fmt::Display for Open {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Layout(error) => error.fmt(f),
            Self::ReturnPath(error) => error.fmt(f),
            Self::InterruptedReturn => f.write_str("под трассой есть разрыв опоры"),
            Self::UnsupportedGeometry => {
                f.write_str("нужно прямое наружное сечение с соседней опорной плоскостью")
            }
            Self::OutsideModel => f.write_str("сечение вне области модели микрополоска"),
            Self::MissingOrUnverified(item) => write!(f, "нет применимого источника: {item}"),
            Self::OverlapsLimit => f.write_str("диапазон Z₀ пересекает границу допуска"),
        }
    }
}

impl From<NotAnalyzed> for Open {
    fn from(error: NotAnalyzed) -> Self {
        Self::Layout(error)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Status {
    Pass,
    Fail,
    NotAnalyzed(Open),
}

#[derive(Clone, Debug, PartialEq)]
pub struct Report {
    pub status: Status,
    /// Номинал идеализированного сечения без поправки по стенду.
    pub nominal_ohm: Option<f64>,
    /// Огибающая той же формулы по технологическим допускам.
    pub process_range_ohm: Option<[f64; 2]>,
    /// Огибающая с учётом подтверждённой погрешности модели.
    pub comparison_range_ohm: Option<[f64; 2]>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Error {
    Input(&'static str),
    Reference(return_path::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Input(message) => f.write_str(message),
            Self::Reference(error) => error.fmt(f),
        }
    }
}

impl std::error::Error for Error {}

fn require(condition: bool, message: &'static str) -> Result<(), Error> {
    condition.then_some(()).ok_or(Error::Input(message))
}

fn check_range([min, max]: [f64; 2], zero_allowed: bool) -> Result<(), Error> {
    require(
        min.is_finite()
            && max.is_finite()
            && min >= 0.0
            && (zero_allowed || min > 0.0)
            && min <= max,
        "некорректные границы диапазона импеданса, геометрии или частоты",
    )
}

fn validate_source(source: Source<'_>) -> Result<(), Error> {
    let (Source::Verified(label) | Source::Reference(label) | Source::Estimate(label)) = source;
    require(!label.trim().is_empty(), "пустой источник данных импеданса")
}

fn applies(source: Source<'_>, scope: Scope) -> bool {
    matches!(source, Source::Verified(_))
        || scope == Scope::Reference && matches!(source, Source::Reference(_))
}

fn validate_mark(mark: &Mark<'_>) -> Result<(), Error> {
    if let Some(process) = mark.process {
        check_range(process.width_m, false)?;
        check_range(process.dielectric_height_m, false)?;
        check_range(process.trace_copper_m, false)?;
        check_range(process.epsilon_r, false)?;
        check_range(process.band_hz, true)?;
        validate_source(process.source)?;
    }
    if let Some(limit) = mark.limit {
        check_range(limit.ohms, false)?;
        validate_source(limit.source)?;
    }
    if let Some(error) = mark.model_error_ohm {
        check_range([error.value; 2], true)?;
        validate_source(error.source)?;
        require(
            mark.reference.scope == Scope::Reference || error.value > 0.0,
            "для сборки нужна ненулевая верхняя граница погрешности модели",
        )?;
    }
    Ok(())
}

#[derive(Clone, Copy)]
struct Section {
    w: f64,
    h: f64,
    t: f64,
    er: f64,
}

fn section(layout: &Layout, width: f64, trace: &str, plane: &str) -> Result<Section, Open> {
    let stack = layout.stackup()?;
    let t = stack.copper_layer(trace)?.thickness_m()?;
    stack.copper_layer(plane)?.thickness_m()?;
    let dielectric = stack.dielectrics_between(trace, plane)?;
    let [dielectric] = dielectric.as_slice() else {
        return Err(Open::UnsupportedGeometry);
    };
    dielectric.material()?;
    Ok(Section {
        w: width,
        h: dielectric.thickness_m()?,
        t,
        er: dielectric.epsilon_r()?,
    })
}

// Нижняя и верхняя границы w/h и εr из Хаммерстада–Йенсена. εr = 1
// соответствует воздуху. Поправка на толщину предназначена для полоски,
// поэтому исключаем сечения с толщиной меди от ширины полоски или высоты стека.
fn model_domain(u: f64, er: f64) -> bool {
    (0.01..=100.0).contains(&u) && (1.0..128.0).contains(&er)
}

// Qucs, «Single microstrip line», (11.22)–(11.24):
// https://qucs.github.io/tech/node75.html#SECTION00811300000000000000
// Эквивалентная ширина u_r растёт с u и t/h и убывает с εr.
// Z₀ плоской полоски убывает с u_r и εr. Независимые экстремумы u_r
// и εr дают огибающую даже при разнонаправленном влиянии εr на них.
fn effective_width(u: f64, t_over_h: f64, er: f64) -> f64 {
    let field = (6.517 * u).sqrt().tanh();
    let du_air = t_over_h / std::f64::consts::PI
        * (4.0 * std::f64::consts::E * field.powi(2) / t_over_h).ln_1p();
    u + 0.5 * du_air * (1.0 + 1.0 / (er - 1.0).sqrt().cosh())
}

fn nominal(s: Section) -> Option<f64> {
    let u = s.w / s.h;
    if s.t >= s.w.min(s.h)
        || !model_domain(u, s.er)
        || !model_domain(effective_width(u, s.t / s.h, s.er), s.er)
    {
        return None;
    }
    let z0 = microstrip_hammerstad_jensen(s.w, s.h, s.t, s.er).z0;
    (z0.is_finite() && z0 > 0.0).then_some(z0)
}

fn includes_nominal([lo, hi]: [f64; 2], value: f64) -> bool {
    // Один соседний float на преобразование из целых нм в метры.
    value >= lo.next_down() && value <= hi.next_up()
}

fn process_range(p: ProcessWindow<'_>, s: Section, z0: f64) -> Result<Option<[f64; 2]>, Error> {
    for (range, value) in [
        (p.width_m, s.w),
        (p.dielectric_height_m, s.h),
        (p.trace_copper_m, s.t),
        (p.epsilon_r, s.er),
    ] {
        require(
            includes_nominal(range, value),
            "номинал платы вне технологического диапазона",
        )?;
    }
    let [w_min, w_max] = p.width_m;
    let [h_min, h_max] = p.dielectric_height_m;
    let [t_min, t_max] = p.trace_copper_m;
    let [er_min, er_max] = p.epsilon_r;
    let (u_min, u_max) = (w_min / h_max, w_max / h_min);
    if t_max >= w_min.min(h_min) || !model_domain(u_min, er_min) || !model_domain(u_max, er_max) {
        return Ok(None);
    }
    let ur_min = effective_width(u_min, t_min / h_max, er_max);
    let ur_max = effective_width(u_max, t_max / h_min, er_min);
    if !model_domain(ur_min, er_min) || !model_domain(ur_max, er_max) {
        return Ok(None);
    }
    // h = 1 — нормировка на высоту, t = 0 — толщина уже учтена в u_r.
    let min = microstrip_hammerstad_jensen(ur_max, 1.0, 0.0, er_max).z0;
    let max = microstrip_hammerstad_jensen(ur_min, 1.0, 0.0, er_min).z0;
    require(
        min.is_finite() && max.is_finite() && min > 0.0 && max > 0.0,
        "некорректное значение в расчёте диапазона Z₀",
    )?;
    // Номинал входит и при округлении размеров на границе диапазона.
    let enclosure = [min.min(z0).next_down(), max.max(z0).next_up()];
    check_range(enclosure, false)?;
    Ok(Some(enclosure))
}

fn check_reference(layout: &Layout, input: &ReferenceInput<'_>) -> Result<Result<(), Open>, Error> {
    let report = return_path::check_reference(layout, input).map_err(Error::Reference)?;
    Ok(match report.status {
        return_path::Status::Pass => Ok(()),
        return_path::Status::Fail => Err(Open::InterruptedReturn),
        return_path::Status::NotAnalyzed(return_path::Open::Layout(error)) => {
            Err(Open::Layout(error))
        }
        return_path::Status::NotAnalyzed(error) => Err(Open::ReturnPath(error)),
    })
}

fn comparison_range(range: [f64; 2], error: f64) -> Result<[f64; 2], Error> {
    let expanded = [
        (range[0] - error).next_down().max(0.0),
        (range[1] + error).next_up(),
    ];
    check_range(expanded, true)?;
    Ok(expanded)
}

/// Проверить одну отмеченную трассу. Пропуск данных сохраняет уже вычисленные
/// диагностические величины. Несовместимые и неконечные входы дают ошибку.
pub fn evaluate(layout: &Layout, mark: &Mark<'_>) -> Result<Report, Error> {
    validate_mark(mark)?;
    let mut report = Report {
        status: Status::NotAnalyzed(Open::UnsupportedGeometry),
        nominal_ohm: None,
        process_range_ohm: None,
        comparison_range_ohm: None,
    };
    report.status = evaluate_mark(layout, mark, &mut report)?.unwrap_or_else(Status::NotAnalyzed);
    Ok(report)
}

fn evaluate_mark(
    layout: &Layout,
    mark: &Mark<'_>,
    report: &mut Report,
) -> Result<Result<Status, Open>, Error> {
    if let Err(open) = check_reference(layout, &mark.reference)? {
        return Ok(Err(open));
    }
    let trace = &layout.tracks[mark.reference.track];
    let plane = mark.reference.plane.layer;
    if !matches!(trace.path, Centerline::Segment { .. })
        || !matches!(trace.layer.as_str(), "F.Cu" | "B.Cu")
    {
        return Ok(Err(Open::UnsupportedGeometry));
    }
    let s = match section(layout, trace.width_m, &trace.layer, plane) {
        Ok(section) => section,
        Err(open) => return Ok(Err(open)),
    };
    let Some(z0) = nominal(s) else {
        return Ok(Err(Open::OutsideModel));
    };
    report.nominal_ohm = Some(z0);
    let scope = mark.reference.scope;
    let Some(process) = mark.process.filter(|p| applies(p.source, scope)) else {
        return Ok(Err(Open::MissingOrUnverified("допуски стека и εr(f)")));
    };
    let Some(range) = process_range(process, s, z0)? else {
        return Ok(Err(Open::OutsideModel));
    };
    // Проверяем проекцию максимальной ширины, включая края и торцы меди.
    if process.width_m[1] > s.w {
        let mut widest = layout.clone();
        widest.tracks[mark.reference.track].width_m = process.width_m[1];
        if let Err(open) = check_reference(&widest, &mark.reference)? {
            return Ok(Err(open));
        }
    }
    report.process_range_ohm = Some(range);
    let Some(error) = mark.model_error_ohm.filter(|d| applies(d.source, scope)) else {
        return Ok(Err(Open::MissingOrUnverified(
            "погрешность модели в рабочей полосе",
        )));
    };
    let [min, max] = comparison_range(range, error.value)?;
    report.comparison_range_ohm = Some([min, max]);
    let Some(limit) = mark.limit.filter(|limit| applies(limit.source, scope)) else {
        return Ok(Err(Open::MissingOrUnverified("допустимый Z₀")));
    };
    let [lo, hi] = limit.ohms;
    Ok(if min >= lo && max <= hi {
        Ok(Status::Pass)
    } else if max < lo || min > hi {
        Ok(Status::Fail)
    } else {
        Err(Open::OverlapsLimit)
    })
}
