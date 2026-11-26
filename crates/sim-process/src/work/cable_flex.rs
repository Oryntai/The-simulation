//! Проверка монтажа гибкой ветви в заданном окне движения и отдельная оценка ресурса.

use std::collections::{BTreeMap, BTreeSet};

use sim_signal::Time;
use sim_world::Provenance;

use crate::{
    Analysis, Interval, ProcessError, SourcedRange,
    schematic::{Finding, Mode, Scope, Section, Status},
    source_can_prove,
};

use super::{
    math::{calculated, computed, finite, invalid, unknown},
    mechanical_limits::rounded_route,
};

pub const CABLE_FLEX_CODE: &str = "cable_flex";
const WINDOW_SCOPE: &str = "монтаж гибкой ветви в указанном режиме и окне";
const DUTY_SCOPE: &str = "ресурс по применимой кривой гибкости для заданного цикла";

#[derive(Clone, Copy, Debug)]
pub struct CableRelief<'a> {
    /// Подтверждённое наличие либо отсутствие разгрузки. Наличие означает,
    /// что крепление перехватывает нагрузку до клеммы и не проскальзывает
    /// в этом режиме. Сведения об установленной детали сами по себе этого не доказывают.
    pub installed: bool,
    pub source: &'a Provenance,
}

#[derive(Clone, Copy, Debug)]
pub struct CableEnd<'a> {
    /// Метка граничной точки подвижного участка в кадрах траектории.
    pub site: &'a str,
    pub relief: Option<CableRelief<'a>>,
}

#[derive(Clone, Copy, Debug)]
pub struct CableGuide<'a> {
    /// Метка виртуальной вершины пересечения касательных в кадре траектории.
    pub site: &'a str,
    /// Отступ от вершины до точки касания на каждой из двух прямых.
    /// Номинальная геометрия из шаблона монтажа, разброс входит в огибающие.
    pub setback_m: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct CableMount<'a> {
    pub ends: [CableEnd<'a>; 2],
    /// Порядок направляющих от первого конца ко второму. Промежуточная
    /// фиксация, через которую не передаётся запас длины, делит трассу
    /// на участки и требует отдельной проверки каждого участка.
    pub guides: &'a [CableGuide<'a>],
    /// Источник применимости схемы маршрута к ветви в этом окне. Покрывает
    /// свободный ход, контакты, выходы из креплений и валидность геометрии
    /// скруглений. Остаточные отклонения формы (включая провис) входят в
    /// огибающие длины и кривизны. Прямая между концами сама по себе не
    /// подтверждает трассу или отсутствие перегибов у креплений.
    pub applicability: Option<&'a Provenance>,
}

#[derive(Clone, Copy, Debug)]
pub struct CablePose<'a> {
    pub at: Time,
    /// Координаты монтажных меток в системе мира из траектории сочленения.
    pub sites_m: &'a BTreeMap<String, [f64; 3]>,
}

#[derive(Clone, Copy, Debug)]
pub struct CableStep<'a> {
    /// Правая граница. Левая задана предыдущей позой либо начальной позой.
    pub end: CablePose<'a>,
    /// Верхняя граница абсолютной разницы между максимумом требуемой
    /// длины в интервале и большим из двух расчётов на его границах.
    /// Включает погрешности поз, расчёта, монтажа и пики между тактами.
    pub length_error_m: Option<&'a SourcedRange>,
    /// Такая же граница для максимума кривизны осевой линии на всём участке.
    /// Погрешность отдельного замера и шаг журнала такой огибающей не задают.
    pub curvature_error_inv_m: Option<&'a SourcedRange>,
}

#[derive(Clone, Copy, Debug)]
pub struct CableMotion<'a> {
    pub start: CablePose<'a>,
    pub steps: &'a [CableStep<'a>],
}

#[derive(Clone, Copy, Debug)]
pub struct CableFlexDuty<'a> {
    pub cycles: u64,
    /// Идентификаторы ориентации сечения и протокола циклического изгиба.
    pub orientation: &'a str,
    pub profile: &'a str,
    pub temperature_k: &'a SourcedRange,
    /// Подтверждает, что исследуемое окно (либо его огибающая) представляет
    /// весь повторяемый цикл, включая возвращение, амплитуды, осевую нагрузку,
    /// ориентацию, температурную историю и распределение нагрузки по длине.
    pub applicability: &'a Provenance,
}

#[derive(Clone, Copy, Debug)]
pub struct CableFlexBand<'a> {
    pub cycles: [u64; 2],
    pub orientation: &'a str,
    pub profile: &'a str,
    pub temperature_k: [f64; 2],
    /// Границы предельного минимального радиуса для всех циклов и температур
    /// полосы. Пустая полоса оставляет предел открытым. Точка паспорта
    /// покрывает только свою точку. Экстраполяция и интерполяция не выполняются.
    pub min_bend_radius_m: Option<&'a SourcedRange>,
}

#[derive(Clone, Copy, Debug)]
pub struct CableFlexCurve<'a> {
    /// Кабель, его монтаж и протокол циклического опыта должны совпадать.
    /// Разрывная нагрузка, один ход и статический радиус ресурс не задают.
    pub source: &'a Provenance,
    pub bands: &'a [CableFlexBand<'a>],
}

#[derive(Clone, Copy, Debug)]
pub struct CableFlexInput<'a> {
    pub scope: Scope,
    pub mode: &'a Mode,
    pub branch: &'a Section,
    pub window: Interval,
    pub mount: Option<CableMount<'a>>,
    pub motion: Option<CableMotion<'a>>,
    pub duty: Option<CableFlexDuty<'a>>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CableFlexRatings<'a> {
    /// Свободная длина, реально доступная на этом подвижном участке без растяжения.
    /// Вычеты на заделку, обязательный провис и удерживаемый запас уже учтены.
    pub usable_length_m: Option<&'a SourcedRange>,
    /// Нижний предел радиуса осевой линии для кабеля в условиях данного хода.
    pub min_bend_radius_m: Option<&'a SourcedRange>,
    pub curve: Option<CableFlexCurve<'a>>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CableBounds {
    pub min: f64,
    pub max: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CablePeak {
    pub bounds: CableBounds,
    /// Отметка наибольшего расчётного отсчёта. Истинный пик может быть между тактами.
    pub sample_at: Time,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CableRadiusBounds {
    /// `None` означает бесконечную положительную границу радиуса или его
    /// запаса для прямого участка. Пропуск данных отражается в `Analysis`.
    pub min_m: Option<f64>,
    pub max_m: Option<f64>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CableCriterion<T> {
    pub status: Status,
    /// Численный запас (с положительным знаком в допустимой области).
    /// При пересечении порога интервал сохранён, статус остаётся открытым.
    pub result: Analysis<T>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CableFlexEvidence {
    pub code: &'static str,
    pub scope: Scope,
    pub mode: Mode,
    pub branch: Section,
    pub window: Interval,
    pub maximum_length_m: Analysis<CablePeak>,
    pub maximum_curvature_inv_m: Analysis<CablePeak>,
    pub minimum_radius_m: Analysis<CableRadiusBounds>,
    pub slack: CableCriterion<CableBounds>,
    pub bend: CableCriterion<CableRadiusBounds>,
    pub relief: [Status; 2],
    pub fatigue: CableCriterion<CableRadiusBounds>,
}

/// Проверяет радиус, запас длины без растяжения и разгрузку обоих концов.
/// `Pass` верхнего уровня относится к этому ходу. Свидетельство ресурса
/// хранится отдельно, установленное нарушение по ресурсу также даёт `Fail`.
pub fn assess_cable_flex(
    input: &CableFlexInput<'_>,
    ratings: &CableFlexRatings<'_>,
) -> Result<Finding<CableFlexEvidence>, ProcessError> {
    let subject = validate(input, ratings)?;
    let [length, curvature] = motion_peaks(input)?;
    let radius = curvature.map(minimum_radius).transpose()?;
    let slack = slack_check(length, proven_range(input.scope, ratings.usable_length_m))?;
    let bend = radius_check(
        radius,
        proven_range(input.scope, ratings.min_bend_radius_m),
        WINDOW_SCOPE,
        "нет подтверждённой огибающей минимального радиуса на всём ходе",
        "нет применимого предела минимального радиуса",
    )?;
    let relief = input.mount.map_or([Status::NotAnalyzed; 2], |mount| {
        mount.ends.map(
            |end| match end.relief.filter(|r| supports(input.scope, r.source)) {
                Some(CableRelief {
                    installed: true, ..
                }) => Status::Pass,
                Some(CableRelief {
                    installed: false, ..
                }) => Status::Fail,
                None => Status::NotAnalyzed,
            },
        )
    });
    let fatigue = if [slack.status, bend.status, relief[0], relief[1]]
        .into_iter()
        .all(|status| status == Status::Pass)
    {
        radius_check(
            radius,
            fatigue_limit(input, ratings.curve),
            DUTY_SCOPE,
            "нет подтверждённой огибающей радиуса цикла",
            "нет применимой кривой гибкости для числа циклов, ориентации, протокола и всей температуры",
        )?
    } else {
        open_check("применимость кривой требует проверенного монтажа и хода")
    };
    let evidence = CableFlexEvidence {
        code: CABLE_FLEX_CODE,
        scope: input.scope,
        mode: input.mode.clone(),
        branch: input.branch.clone(),
        window: input.window,
        maximum_length_m: peak_analysis(length, "нет огибающей длины на всём ходе"),
        maximum_curvature_inv_m: peak_analysis(curvature, "нет огибающей кривизны на всём ходе"),
        minimum_radius_m: radius.map_or_else(
            || unknown("нет огибающей кривизны на всём ходе"),
            |radius| calculated(radius, WINDOW_SCOPE),
        ),
        slack,
        bend,
        relief,
        fatigue,
    };
    let (status, reason) = summarize(&evidence);
    Ok(Finding {
        subject,
        status,
        reason,
        evidence,
    })
}

fn validate(
    input: &CableFlexInput<'_>,
    ratings: &CableFlexRatings<'_>,
) -> Result<String, ProcessError> {
    if input.window.start >= input.window.end {
        return Err(invalid(
            "окно проверки гибкой ветви должно иметь положительную длительность",
        ));
    }
    let subject = match input.branch {
        Section::Connection(name) => {
            identifier(name)?;
            name.clone()
        }
        Section::Element { component, element } => {
            identifier(component)?;
            identifier(element)?;
            format!("{component}/{element}")
        }
    };
    if let Mode::OpenWire(name) = input.mode {
        identifier(name)?;
    }
    for rating in [ratings.usable_length_m, ratings.min_bend_radius_m]
        .into_iter()
        .flatten()
    {
        quantity(rating, true)?;
    }
    if let Some(mount) = input.mount {
        let mut names = BTreeSet::new();
        for site in mount
            .ends
            .iter()
            .map(|end| end.site)
            .chain(mount.guides.iter().map(|guide| guide.site))
        {
            identifier(site)?;
            if !names.insert(site) {
                return Err(invalid("повторная метка шаблона монтажа"));
            }
        }
        for guide in mount.guides {
            finite(&[guide.setback_m])?;
            if guide.setback_m <= 0.0 {
                return Err(invalid("отступ точки касания должен быть положительным"));
            }
        }
        for source in mount.applicability.into_iter().chain(
            mount
                .ends
                .iter()
                .filter_map(|end| end.relief.map(|r| r.source)),
        ) {
            validate_source(source)?;
        }
    }
    if let Some(motion) = input.motion {
        let mut previous = None;
        for pose in std::iter::once(motion.start).chain(motion.steps.iter().map(|step| step.end)) {
            if previous.is_some_and(|time| time >= pose.at)
                || pose.at < input.window.start
                || pose.at > input.window.end
            {
                return Err(invalid("метки траектории должны возрастать внутри окна"));
            }
            for (site, position) in pose.sites_m {
                identifier(site)?;
                finite(position)?;
            }
            previous = Some(pose.at);
        }
        for step in motion.steps {
            for error in [step.length_error_m, step.curvature_error_inv_m]
                .into_iter()
                .flatten()
            {
                quantity(error, false)?;
            }
        }
    }
    if let Some(duty) = input.duty {
        identifier(duty.orientation)?;
        identifier(duty.profile)?;
        validate_source(duty.applicability)?;
        quantity(duty.temperature_k, true)?;
        if duty.cycles == 0 {
            return Err(invalid("для ресурса нужно положительное число циклов"));
        }
    }
    if let Some(curve) = ratings.curve {
        validate_source(curve.source)?;
        for band in curve.bands {
            identifier(band.orientation)?;
            identifier(band.profile)?;
            let [min_k, max_k] = band.temperature_k;
            finite(&[min_k, max_k])?;
            if band.cycles[0] == 0
                || band.cycles[0] > band.cycles[1]
                || min_k <= 0.0
                || min_k > max_k
            {
                return Err(invalid("некорректная область кривой гибкости"));
            }
            if let Some(radius) = band.min_bend_radius_m {
                quantity(radius, true)?;
            }
        }
    }
    Ok(subject)
}

fn identifier(value: &str) -> Result<(), ProcessError> {
    if value.is_empty() || value.trim() != value || value.chars().any(char::is_control) {
        return Err(invalid(
            "пустая или недопустимая метка, идентификатор или источник",
        ));
    }
    Ok(())
}

fn validate_source(source: &Provenance) -> Result<(), ProcessError> {
    let text = match source {
        Provenance::Datasheet(text)
        | Provenance::Paper(text)
        | Provenance::Measured(text)
        | Provenance::Estimate(text)
        | Provenance::Derived(text) => text,
    };
    identifier(text)
}

fn quantity(value: &SourcedRange, positive: bool) -> Result<(), ProcessError> {
    value.validate()?;
    if value.min < 0.0 || (positive && value.min == 0.0) {
        return Err(invalid(
            "длина, радиус и температура должны быть положительными, погрешность неотрицательной",
        ));
    }
    Ok(())
}

fn supports(scope: Scope, source: &Provenance) -> bool {
    source_can_prove(source)
        || matches!((scope, source), (Scope::Reference, Provenance::Derived(_)))
}

fn proven_range(scope: Scope, value: Option<&SourcedRange>) -> Option<CableBounds> {
    value
        .filter(|value| supports(scope, &value.provenance))
        .map(|value| CableBounds {
            min: value.min,
            max: value.max,
        })
}

fn motion_peaks(input: &CableFlexInput<'_>) -> Result<[Option<CablePeak>; 2], ProcessError> {
    let (Some(mount), Some(motion)) = (input.mount, input.motion) else {
        return Ok([None; 2]);
    };
    if !mount
        .applicability
        .is_some_and(|source| supports(input.scope, source))
        || motion.start.at != input.window.start
        || motion.steps.last().map(|step| step.end.at) != Some(input.window.end)
    {
        return Ok([None; 2]);
    }
    let sites = std::iter::once(mount.ends[0].site)
        .chain(mount.guides.iter().map(|guide| guide.site))
        .chain(std::iter::once(mount.ends[1].site))
        .collect::<Vec<_>>();
    let setbacks = mount
        .guides
        .iter()
        .map(|guide| guide.setback_m)
        .collect::<Vec<_>>();
    let evaluate = |pose: CablePose<'_>| {
        let points = sites
            .iter()
            .map(|site| pose.sites_m.get(*site).copied())
            .collect::<Option<Vec<_>>>();
        match points {
            Some(points) => rounded_route(&points, &setbacks),
            None => Ok(None),
        }
    };
    let Some(mut previous) = evaluate(motion.start)? else {
        return Ok([None; 2]);
    };
    let mut previous_at = motion.start.at;
    let mut peaks = [PeakSweep::default(), PeakSweep::default()];
    for step in motion.steps {
        let Some(next) = evaluate(step.end)? else {
            return Ok([None; 2]);
        };
        let errors = [step.length_error_m, step.curvature_error_inv_m];
        for (index, peak) in peaks.iter_mut().enumerate() {
            let (value, at) = if previous[index] >= next[index] {
                (previous[index], previous_at)
            } else {
                (next[index], step.end.at)
            };
            peak.include(value, at, proven_range(input.scope, errors[index]))?;
        }
        previous = next;
        previous_at = step.end.at;
    }
    Ok(peaks.map(|peak| peak.result()))
}

#[derive(Default)]
struct PeakSweep {
    peak: Option<CablePeak>,
    sample: f64,
    missing: bool,
}

impl PeakSweep {
    fn include(
        &mut self,
        sample: f64,
        at: Time,
        error: Option<CableBounds>,
    ) -> Result<(), ProcessError> {
        let Some(error) = error else {
            self.missing = true;
            return Ok(());
        };
        let bounds = if error.max == 0.0 {
            CableBounds {
                min: sample,
                max: sample,
            }
        } else {
            CableBounds {
                min: (sample - error.max).next_down().max(0.0),
                max: (sample + error.max).next_up(),
            }
        };
        computed(&[bounds.min, bounds.max])?;
        let peak = self.peak.get_or_insert(CablePeak {
            bounds,
            sample_at: at,
        });
        peak.bounds.min = peak.bounds.min.max(bounds.min);
        peak.bounds.max = peak.bounds.max.max(bounds.max);
        if sample > self.sample {
            self.sample = sample;
            peak.sample_at = at;
        }
        Ok(())
    }

    fn result(self) -> Option<CablePeak> {
        self.peak.filter(|_| !self.missing)
    }
}

fn minimum_radius(curvature: CablePeak) -> Result<CableRadiusBounds, ProcessError> {
    // 1/κ убывает. Нулевая кривизна означает прямую без конечного радиуса.
    let mut values = [None; 2];
    for (i, k) in [curvature.bounds.max, curvature.bounds.min]
        .into_iter()
        .enumerate()
    {
        if k != 0.0 {
            let r = k.recip();
            let outward = if i == 0 {
                r.next_down().max(0.0)
            } else {
                r.next_up()
            };
            computed(&[outward, r])?;
            values[i] = Some(outward);
        }
    }
    let [min_m, max_m] = values;
    Ok(CableRadiusBounds { min_m, max_m })
}

fn slack_check(
    length: Option<CablePeak>,
    available: Option<CableBounds>,
) -> Result<CableCriterion<CableBounds>, ProcessError> {
    let Some(length) = length else {
        return Ok(open_check(
            "нет подтверждённой огибающей длины на всём ходе",
        ));
    };
    let Some(available) = available else {
        return Ok(open_check(
            "нет подтверждённой доступной длины без растяжения",
        ));
    };
    let margin = CableBounds {
        min: (available.min - length.bounds.max).next_down(),
        max: (available.max - length.bounds.min).next_up(),
    };
    computed(&[margin.min, margin.max])?;
    // Нулевой запас не подтверждает отсутствие натяжения на границе хода.
    let status = if margin.max < 0.0 {
        Status::Fail
    } else if margin.min > 0.0 {
        Status::Pass
    } else {
        Status::NotAnalyzed
    };
    Ok(CableCriterion {
        status,
        result: calculated(margin, WINDOW_SCOPE),
    })
}

fn radius_check(
    radius: Option<CableRadiusBounds>,
    limit: Option<CableBounds>,
    scope: &str,
    missing_radius: &str,
    missing_limit: &str,
) -> Result<CableCriterion<CableRadiusBounds>, ProcessError> {
    let Some(radius) = radius else {
        return Ok(open_check(missing_radius));
    };
    let Some(limit) = limit else {
        return Ok(open_check(missing_limit));
    };
    let margin = CableRadiusBounds {
        min_m: radius.min_m.map(|min| (min - limit.max).next_down()),
        max_m: radius.max_m.map(|max| (max - limit.min).next_up()),
    };
    for value in [margin.min_m, margin.max_m].into_iter().flatten() {
        computed(&[value])?;
    }
    let status = if margin.max_m.is_some_and(|max| max < 0.0) {
        Status::Fail
    } else if margin.min_m.is_none_or(|min| min >= 0.0) {
        Status::Pass
    } else {
        Status::NotAnalyzed
    };
    Ok(CableCriterion {
        status,
        result: calculated(margin, scope),
    })
}

fn fatigue_limit(
    input: &CableFlexInput<'_>,
    curve: Option<CableFlexCurve<'_>>,
) -> Option<CableBounds> {
    let (duty, curve) = input.duty.zip(curve)?;
    if !supports(input.scope, duty.applicability) || !supports(input.scope, curve.source) {
        return None;
    }
    let temperature = proven_range(input.scope, Some(duty.temperature_k))?;
    let mut bands = curve
        .bands
        .iter()
        .filter(|band| {
            band.orientation == duty.orientation
                && band.profile == duty.profile
                && (band.cycles[0]..=band.cycles[1]).contains(&duty.cycles)
                && band.temperature_k[0] <= temperature.max
                && band.temperature_k[1] >= temperature.min
        })
        .peekable();
    let first = proven_range(input.scope, bands.peek()?.min_bend_radius_m)?;
    let mut limit = first;
    let mut coverage = Vec::new();
    for band in bands {
        let bounds = proven_range(input.scope, band.min_bend_radius_m)?;
        coverage.push(band.temperature_k);
        // Для гарантированного годного или негодного результата по всей
        // неизвестной температурной истории нужны обе крайние границы.
        limit.min = limit.min.min(bounds.min);
        limit.max = limit.max.max(bounds.max);
    }
    coverage.sort_unstable_by(|a, b| a[0].total_cmp(&b[0]));
    let mut covered_to = temperature.min;
    for [start, end] in coverage {
        if start > covered_to {
            return None;
        }
        covered_to = covered_to.max(end);
    }
    (covered_to >= temperature.max).then_some(limit)
}

fn open_check<T>(reason: &str) -> CableCriterion<T> {
    CableCriterion {
        status: Status::NotAnalyzed,
        result: unknown(reason),
    }
}

fn peak_analysis(peak: Option<CablePeak>, reason: &str) -> Analysis<CablePeak> {
    peak.map_or_else(|| unknown(reason), |peak| calculated(peak, WINDOW_SCOPE))
}

fn summarize(evidence: &CableFlexEvidence) -> (Status, String) {
    let checks = [
        (
            evidence.relief[0],
            "разгрузка первого конца отсутствует",
            "нет подтверждённой разгрузки первого конца",
        ),
        (
            evidence.relief[1],
            "разгрузка второго конца отсутствует",
            "нет подтверждённой разгрузки второго конца",
        ),
        (
            evidence.bend.status,
            "радиус меньше применимого предела",
            "предел минимального радиуса не подтверждён на всём ходе",
        ),
        (
            evidence.slack.status,
            "доступной длины не хватает на ход без растяжения",
            "запас длины без растяжения не подтверждён на всём ходе",
        ),
    ];
    if let Some((_, reason, _)) = checks.iter().find(|entry| entry.0 == Status::Fail) {
        return (Status::Fail, (*reason).into());
    }
    if let Some((_, _, reason)) = checks.iter().find(|entry| entry.0 == Status::NotAnalyzed) {
        return (Status::NotAnalyzed, (*reason).into());
    }
    if evidence.fatigue.status == Status::Fail {
        return (
            Status::Fail,
            "радиус меньше предела для заданного ресурса".into(),
        );
    }
    (
        Status::Pass,
        "радиус, запас длины и разгрузка подтверждены в окне хода".into(),
    )
}
