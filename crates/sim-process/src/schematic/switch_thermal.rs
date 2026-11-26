//! Нагрев перехода и линейная область безопасной работы ключа в заданном режиме.

use sim_world::Provenance;

use crate::{Analysis, Interval, ProcessError, SourcedRange};

use super::{
    Endpoint, Finding, Mode, Scope, Section, Status, identifier, invalid, open, supports_source,
    validate_source,
};

pub const SWITCH_THERMAL_CODE: &str = "switch_thermal";
const WINDOW_SCOPE: &str = "ветвь, окно и условия указанного режима";

/// Гарантированные огибающие на всём интервале, включая фронты и пики между
/// отсчётами и неопределённость времени событий. Границы соседних интервалов
/// задают момент смены огибающей. Отдельные отсчёты ряда такой гарантии не дают.
/// Ток ориентирован от стока к истоку.
#[derive(Clone, Copy, Debug)]
pub struct SwitchSegment<'a> {
    pub window: Interval,
    pub vds_v: Option<&'a SourcedRange>,
    pub id_a: Option<&'a SourcedRange>,
    pub vgs_v: Option<&'a SourcedRange>,
}

#[derive(Clone, Copy, Debug)]
pub enum SwitchExposure<'a> {
    Window,
    /// Источник подтверждает неограниченное продолжение последнего режима
    /// с теми же огибающими мощности и условиями охлаждения.
    Sustained(&'a Provenance),
}

#[derive(Clone, Copy, Debug)]
pub struct SwitchThermalInput<'a> {
    pub scope: Scope,
    pub mode: &'a Mode,
    pub branch: &'a Section,
    /// Сток, исток, затвор. Vds и Vgs отсчитываются относительно истока.
    /// Для обратной полярности управления знаки Vds, Id и Vgs меняются вместе,
    /// чтобы прямой режим имел положительные Vds, Id и возрастающее Vgs.
    pub terminals: &'a [Endpoint; 3],
    pub window: Interval,
    pub segments: Option<&'a [SwitchSegment<'a>]>,
    pub ambient_k: Option<&'a SourcedRange>,
    /// Огибающая температуры корпуса для всех импульсов и их начальных условий.
    pub case_k: Option<&'a SourcedRange>,
    /// Превышения над Ta для каждого полюса в начале окна в порядке модели.
    /// Холодный старт задаётся явными подтверждёнными нулями.
    pub initial_rises_k: Option<&'a [SourcedRange]>,
    /// Подтверждает корпус, плату, охлаждение и тепловую историю. Каналы V/I
    /// должны охватывать всё рассеяние, существенное для модели нагрева.
    /// Источник также подтверждает граничные условия модели по Ta.
    pub thermal_applicability: Option<&'a Provenance>,
    pub exposure: SwitchExposure<'a>,
}

#[derive(Clone, Copy, Debug)]
pub struct SwitchFosterPole<'a> {
    pub resistance_k_per_w: &'a SourcedRange,
    pub time_constant_s: &'a SourcedRange,
}

/// Zθ(t) = Σ Rᵢ(1 − exp(−t/τᵢ)), переход → среда для того же монтажа, что θja.
#[derive(Clone, Copy, Debug)]
pub struct SwitchFoster<'a> {
    pub poles: &'a [SwitchFosterPole<'a>],
    /// Верхняя граница ошибки Tj во всём применимом режиме, включая аппроксимацию Zθ(t),
    /// граничные условия и численную погрешность.
    pub fit_error_k: &'a SourcedRange,
}

#[derive(Clone, Copy, Debug)]
pub enum SwitchSoaLimit<'a> {
    Pulse(&'a SourcedRange),
    Continuous(&'a Provenance),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SwitchSoaDuty {
    SinglePulse,
    PulseTrain,
}

/// Линии SOA, представленные обратной зависимостью: допустимая длительность
/// для Vds, Id и Tc. Каждая ячейка гарантирует диапазон допустимой длительности
/// во всей области. Источник должен подтверждать сравнение импульса переменной
/// формы с охватывающим прямоугольником V/I и заявленным типом повторения.
/// Порядок ячеек: Tc, Vds, Id (Id меняется быстрее). На внутренних границах
/// действует правая ячейка, последний край включён. Ось из одной точки задаёт
/// только эту точку, пустая ячейка оставляет предел открытым.
#[derive(Clone, Copy, Debug)]
pub struct SwitchSoa<'a> {
    pub vds_edges_v: &'a [f64],
    pub id_edges_a: &'a [f64],
    pub case_edges_k: &'a [f64],
    pub limits: &'a [Option<SwitchSoaLimit<'a>>],
    /// Применимость линий к корпусу, полярности и всей наблюдаемой работе.
    /// Для одиночного импульса включает отсутствие влияния предыстории.
    /// Для серии включает скважность, паузы и накопление тепла.
    pub applicability: Option<&'a Provenance>,
    pub duty: SwitchSoaDuty,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SwitchThermalRatings<'a> {
    pub max_junction_k: Option<&'a SourcedRange>,
    /// Стационарное θja из независимых данных для указанного монтажа.
    pub theta_ja_k_per_w: Option<&'a SourcedRange>,
    pub foster: Option<SwitchFoster<'a>>,
    /// Допустимое абсолютное расхождение методов на одной мощности в равновесии.
    pub agreement_tolerance_k: Option<&'a SourcedRange>,
    /// Применимый к режиму порог полного включения, не порог начала проводимости.
    /// Полярность Vgs ориентируется так, что рост Vgs включает ключ.
    pub fully_on_vgs_v: Option<&'a SourcedRange>,
    pub soa: Option<SwitchSoa<'a>>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SwitchBounds {
    pub min: f64,
    pub max: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SwitchMargin {
    pub observed: SwitchBounds,
    pub limit: SwitchBounds,
    /// Допустимое значение минус наблюдаемое.
    pub margin: SwitchBounds,
}

/// Допустимая длительность. Неограниченная область задаётся отдельно.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub enum SwitchSoaTime {
    Seconds(f64),
    Continuous,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SwitchPulseEvidence {
    pub window: Interval,
    pub duration_s: SwitchBounds,
    pub closed: bool,
    pub vds_v: SwitchBounds,
    pub id_a: SwitchBounds,
    pub case_k: SwitchBounds,
    pub allowed: Analysis<[SwitchSoaTime; 2]>,
    pub margin_s: Analysis<SwitchBounds>,
    pub status: Status,
    pub reason: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SwitchThermalEvidence {
    pub code: &'static str,
    pub scope: Scope,
    pub mode: Mode,
    pub branch: Section,
    pub terminals: [Endpoint; 3],
    pub window: Interval,
    /// Заявлено продолжение последнего режима за границей окна.
    pub sustained: bool,
    /// Тепловые оценки равновесия считаются на одной мощности. Для конечного
    /// окна это условное равновесие на пике мощности, для длительного режима
    /// это равновесие при продолжении последнего участка.
    pub equilibrium_power_w: Analysis<SwitchBounds>,
    pub steady_junction_k: Analysis<SwitchMargin>,
    pub foster_equilibrium_junction_k: Analysis<SwitchMargin>,
    pub foster_peak_junction_k: Analysis<SwitchMargin>,
    /// Расхождение равновесных оценок на максимальной мощности окна.
    pub agreement_k: Analysis<SwitchMargin>,
    pub linear_soa: Analysis<Vec<SwitchPulseEvidence>>,
}

pub fn assess_switch_thermal(
    input: &SwitchThermalInput<'_>,
    ratings: &SwitchThermalRatings<'_>,
) -> Result<Finding<SwitchThermalEvidence>, ProcessError> {
    let subject = validate(input, ratings)?;
    let proven = |range| bounds(input.scope, range);
    let segments = covered_segments(input);
    let thermal_applicable = confirmed(input.scope, input.thermal_applicability);
    let sustained = matches!(input.exposure, SwitchExposure::Sustained(_));
    let horizon_proven = match input.exposure {
        SwitchExposure::Window => true,
        SwitchExposure::Sustained(source) => confirmed(input.scope, Some(source)),
    };
    let powers = if thermal_applicable {
        segments
            .map(|segments| power_trace(input.scope, segments))
            .transpose()?
            .flatten()
    } else {
        None
    };
    let peak_power = powers.as_deref().map(|powers| {
        powers
            .iter()
            .copied()
            .fold(SwitchBounds::ZERO, SwitchBounds::maximum)
    });
    let equilibrium_power = if sustained {
        powers
            .as_deref()
            .filter(|_| horizon_proven)
            .and_then(|powers| powers.last().copied())
    } else {
        peak_power
    };
    let ambient = proven(input.ambient_k);
    let theta = proven(ratings.theta_ja_k_per_w);
    let foster = ratings
        .foster
        .and_then(|model| foster_bounds(input.scope, model));
    let foster_theta = foster
        .as_ref()
        .map(|(poles, _)| {
            poles
                .iter()
                .try_fold(SwitchBounds::ZERO, |sum, pole| sum.add(pole.resistance))
        })
        .transpose()?;
    let steady = equilibrium(ambient, equilibrium_power, theta)?;
    let foster_equilibrium = equilibrium(ambient, equilibrium_power, foster_theta)?;
    let agreement = peak_power
        .zip(theta.zip(foster_theta))
        .map(|(power, (a, b))| a.difference(b)?.absolute().multiply_nonnegative(power))
        .transpose()?;
    let initial = input.initial_rises_k.and_then(|rises| {
        rises
            .iter()
            .map(|rise| proven(Some(rise)))
            .collect::<Option<Vec<_>>>()
    });
    let transient = match (segments, powers.as_deref(), ambient, foster, initial) {
        (Some(segments), Some(powers), Some(ambient), Some((poles, error)), Some(initial)) => {
            let tail = sustained && horizon_proven;
            Some(foster_peak(
                segments, powers, &poles, initial, ambient, error, tail,
            )?)
        }
        _ => None,
    };
    let thermal_gap = if !thermal_applicable {
        "нет подтверждения тепловой применимости корпуса, платы и рассеяния"
    } else if powers.is_none() {
        "нет подтверждённых Vds и Id между тактами во всём окне"
    } else if !horizon_proven {
        "нет подтверждения продолжения длительного режима"
    } else {
        "нет Ta или параметров тепловой модели и начального состояния"
    };
    let limit = proven(ratings.max_junction_k);
    let evidence = SwitchThermalEvidence {
        code: SWITCH_THERMAL_CODE,
        scope: input.scope,
        mode: input.mode.clone(),
        branch: input.branch.clone(),
        terminals: input.terminals.clone(),
        window: input.window,
        sustained,
        equilibrium_power_w: analyzed(equilibrium_power, thermal_gap),
        steady_junction_k: margin(steady, limit, thermal_gap, "нет применимого предела Tj")?,
        foster_equilibrium_junction_k: margin(
            foster_equilibrium,
            limit,
            thermal_gap,
            "нет применимого предела Tj",
        )?,
        foster_peak_junction_k: margin(
            transient,
            limit,
            thermal_gap,
            "нет применимого предела Tj",
        )?,
        agreement_k: margin(
            agreement,
            proven(ratings.agreement_tolerance_k),
            "нет общей мощности или независимых θja и Zθ(t) для сравнения",
            "нет заявленной погрешности сравнения θja и Zθ(t)",
        )?,
        linear_soa: linear_soa(input, ratings, segments)?,
    };
    let (status, reason) = summarize(&evidence, horizon_proven);
    Ok(Finding {
        subject,
        status,
        reason,
        evidence,
    })
}

fn validate(
    input: &SwitchThermalInput<'_>,
    ratings: &SwitchThermalRatings<'_>,
) -> Result<String, ProcessError> {
    let Section::Element { component, element } = input.branch else {
        return Err(invalid("нужна ветвь элемента ключа"));
    };
    let [d, s, g] = input.terminals;
    if input.window.start >= input.window.end
        || ![component, element]
            .into_iter()
            .all(|name| identifier(name))
        || [d, s, g]
            .iter()
            .any(|end| &end.component != component || !identifier(&end.pin))
        || d == s
        || s == g
        || d == g
    {
        return Err(invalid(
            "недопустимые ветвь, выводы или окно проверки ключа",
        ));
    }
    for range in [
        input.ambient_k,
        input.case_k,
        ratings.max_junction_k,
        ratings.theta_ja_k_per_w,
    ] {
        validate_range(range, |v| v > 0.0)?;
    }
    validate_range(ratings.agreement_tolerance_k, |v| v >= 0.0)?;
    validate_range(ratings.fully_on_vgs_v, |_| true)?;
    if let Some(rises) = input.initial_rises_k {
        for rise in rises {
            validate_range(Some(rise), |v| v >= 0.0)?;
        }
    }
    if let Some(foster) = ratings.foster {
        for pole in foster.poles {
            validate_range(Some(pole.resistance_k_per_w), |v| v > 0.0)?;
            validate_range(Some(pole.time_constant_s), |v| v > 0.0)?;
        }
        validate_range(Some(foster.fit_error_k), |v| v >= 0.0)?;
        if foster.poles.is_empty()
            || input
                .initial_rises_k
                .is_some_and(|rises| rises.len() != foster.poles.len())
        {
            return Err(invalid(
                "число полюсов Фостера и начальных состояний недопустимо",
            ));
        }
    }
    if let Some(source) = input.thermal_applicability {
        validate_source(source)?;
    }
    if let SwitchExposure::Sustained(source) = input.exposure {
        validate_source(source)?;
    }
    if let Some(segments) = input.segments {
        for segment in segments {
            if segment.window.start >= segment.window.end {
                return Err(invalid("недопустимый интервал огибающей ключа"));
            }
            for range in [segment.vds_v, segment.id_a, segment.vgs_v] {
                validate_range(range, |_| true)?;
            }
        }
        if segments
            .windows(2)
            .any(|w| w[0].window.end > w[1].window.start)
        {
            return Err(invalid(
                "интервалы огибающей ключа пересекаются или не упорядочены",
            ));
        }
    }
    if let Some(soa) = ratings.soa {
        soa.validate()?;
    }
    Ok(format!("{component}/{element}"))
}

fn validate_range(
    range: Option<&SourcedRange>,
    accepts: impl Fn(f64) -> bool,
) -> Result<(), ProcessError> {
    if let Some(range) = range {
        range.validate()?;
        if !accepts(range.min) || !accepts(range.max) {
            return Err(invalid("величина ключа вне физических границ"));
        }
    }
    Ok(())
}

fn confirmed(scope: Scope, source: Option<&Provenance>) -> bool {
    source.is_some_and(|source| supports_source(scope, source))
}

fn bounds(scope: Scope, range: Option<&SourcedRange>) -> Option<SwitchBounds> {
    let range = range?;
    confirmed(scope, Some(&range.provenance)).then_some(SwitchBounds {
        min: range.min,
        max: range.max,
    })
}

fn covered_segments<'a>(input: &SwitchThermalInput<'a>) -> Option<&'a [SwitchSegment<'a>]> {
    let segments = input.segments?;
    (segments.first()?.window.start == input.window.start
        && segments.last()?.window.end == input.window.end
        && segments
            .windows(2)
            .all(|w| w[0].window.end == w[1].window.start))
    .then_some(segments)
}

fn power_trace(
    scope: Scope,
    segments: &[SwitchSegment<'_>],
) -> Result<Option<Vec<SwitchBounds>>, ProcessError> {
    let mut powers = Vec::with_capacity(segments.len());
    for segment in segments {
        let Some((v, i)) = bounds(scope, segment.vds_v).zip(bounds(scope, segment.id_a)) else {
            return Ok(None);
        };
        // Положительная часть Vds·Id в пассивном направлении ветви.
        powers.push(v.multiply(i)?.nonnegative());
    }
    Ok(Some(powers))
}

struct FosterPole {
    resistance: SwitchBounds,
    tau: SwitchBounds,
}

fn foster_bounds(scope: Scope, model: SwitchFoster<'_>) -> Option<(Vec<FosterPole>, SwitchBounds)> {
    let error = bounds(scope, Some(model.fit_error_k))?;
    let poles = model
        .poles
        .iter()
        .map(|pole| {
            Some(FosterPole {
                resistance: bounds(scope, Some(pole.resistance_k_per_w))?,
                tau: bounds(scope, Some(pole.time_constant_s))?,
            })
        })
        .collect::<Option<Vec<_>>>()?;
    Some((poles, error))
}

fn equilibrium(
    ambient: Option<SwitchBounds>,
    power: Option<SwitchBounds>,
    theta: Option<SwitchBounds>,
) -> Result<Option<SwitchBounds>, ProcessError> {
    ambient
        .zip(power.zip(theta))
        .map(|(ambient, (power, theta))| ambient.add(power.multiply_nonnegative(theta)?))
        .transpose()
}

fn foster_peak(
    segments: &[SwitchSegment<'_>],
    powers: &[SwitchBounds],
    poles: &[FosterPole],
    mut states: Vec<SwitchBounds>,
    ambient: SwitchBounds,
    error: SwitchBounds,
    tail: bool,
) -> Result<SwitchBounds, ProcessError> {
    let sum = |values: &[SwitchBounds]| {
        values
            .iter()
            .try_fold(SwitchBounds::ZERO, |sum, &value| sum.add(value))
    };
    let mut peak = sum(&states)?;
    for (segment, &power) in segments.iter().zip(powers) {
        let dt = duration(segment.window)?;
        let mut upper = SwitchBounds::ZERO;
        for (pole, state) in poles.iter().zip(&mut states) {
            let target = pole.resistance.multiply_nonnegative(power)?;
            let end = step(*state, target, dt, pole.tau)?;
            upper = upper.add(state.maximum(end))?;
            *state = end;
        }
        // Каждое звено монотонно на ступени. В сумме звенья могут иметь
        // противоположные наклоны, поэтому максимум на концах ограничивает
        // пик снизу, сумма максимумов звеньев ограничивает его сверху.
        peak = peak.maximum(SwitchBounds {
            min: sum(&states)?.min,
            max: upper.max,
        });
    }
    if tail {
        let power = powers[powers.len() - 1];
        let mut lower = SwitchBounds::ZERO;
        let mut upper = SwitchBounds::ZERO;
        for (pole, state) in poles.iter().zip(states) {
            let target = pole.resistance.multiply_nonnegative(power)?;
            lower = lower.add(target)?;
            upper = upper.add(state.maximum(target))?;
        }
        peak = peak.maximum(SwitchBounds {
            min: lower.min,
            max: upper.max,
        });
    }
    let peak = ambient.add(peak)?;
    Ok(SwitchBounds::outward(peak.min - error.max, peak.max + error.max)?.nonnegative())
}

fn step(
    initial: SwitchBounds,
    target: SwitchBounds,
    dt: SwitchBounds,
    tau: SwitchBounds,
) -> Result<SwitchBounds, ProcessError> {
    // Точная RC-ступень x(t+dt) = x(t)(1−a) + RP·a, a = 1−exp(−dt/τ).
    // Для нижней и верхней граничной ступени x монотонен по a.
    let exponent_min = (dt.min / tau.max).next_down().max(0.0);
    let exponent_max = (dt.max / tau.min).next_up();
    let a_min = (-(-exponent_min).exp_m1()).next_down().max(0.0);
    let a_max = (-(-exponent_max).exp_m1()).next_up().min(1.0);
    let interpolate = |from: f64, to: f64, a: f64| {
        let complement = SwitchBounds::outward(1.0 - a, 1.0 - a)?.nonnegative();
        complement
            .multiply_nonnegative(SwitchBounds::point(from))?
            .add(SwitchBounds::point(a).multiply_nonnegative(SwitchBounds::point(to))?)
    };
    let lo = if target.min >= initial.min {
        a_min
    } else {
        a_max
    };
    let hi = if target.max >= initial.max {
        a_max
    } else {
        a_min
    };
    Ok(SwitchBounds {
        min: interpolate(initial.min, target.min, lo)?.min.max(0.0),
        max: interpolate(initial.max, target.max, hi)?.max,
    })
}

fn duration(window: Interval) -> Result<SwitchBounds, ProcessError> {
    const NS_PER_S: f64 = 1_000_000_000.0; // Единица Time — наносекунда.
    let ticks = (window.end.0 - window.start.0) as f64;
    SwitchBounds::outward(ticks.next_down() / NS_PER_S, ticks.next_up() / NS_PER_S)
}

impl SwitchBounds {
    const ZERO: Self = Self::point(0.0);

    const fn point(value: f64) -> Self {
        Self {
            min: value,
            max: value,
        }
    }

    fn outward(min: f64, max: f64) -> Result<Self, ProcessError> {
        let (min, max) = (min.next_down(), max.next_up());
        if !min.is_finite() || !max.is_finite() {
            return Err(ProcessError::Execution(
                "переполнение расчёта нагрева или SOA ключа".into(),
            ));
        }
        Ok(Self { min, max })
    }

    fn nonnegative(self) -> Self {
        Self {
            min: self.min.max(0.0),
            max: self.max.max(0.0),
        }
    }

    fn absolute(self) -> Self {
        Self {
            min: self.min.max(-self.max).max(0.0),
            max: self.min.abs().max(self.max.abs()),
        }
    }

    fn maximum(self, other: Self) -> Self {
        Self {
            min: self.min.max(other.min),
            max: self.max.max(other.max),
        }
    }

    fn hull(self, other: Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    fn add(self, other: Self) -> Result<Self, ProcessError> {
        if self == Self::ZERO {
            return Ok(other);
        }
        if other == Self::ZERO {
            return Ok(self);
        }
        Self::outward(self.min + other.min, self.max + other.max)
    }

    fn difference(self, other: Self) -> Result<Self, ProcessError> {
        if self.min == self.max && self == other {
            return Ok(Self::ZERO);
        }
        Self::outward(self.min - other.max, self.max - other.min)
    }

    fn multiply(self, other: Self) -> Result<Self, ProcessError> {
        if self == Self::ZERO || other == Self::ZERO {
            return Ok(Self::ZERO);
        }
        let values = [
            self.min * other.min,
            self.min * other.max,
            self.max * other.min,
            self.max * other.max,
        ];
        Self::outward(
            values.into_iter().fold(f64::INFINITY, f64::min),
            values.into_iter().fold(f64::NEG_INFINITY, f64::max),
        )
    }

    fn multiply_nonnegative(self, other: Self) -> Result<Self, ProcessError> {
        self.multiply(other).map(Self::nonnegative)
    }
}

impl SwitchSoa<'_> {
    fn validate(self) -> Result<(), ProcessError> {
        let axis = |edges: &[f64], minimum: f64| {
            !edges.is_empty()
                && edges.iter().all(|&x| x.is_finite() && x >= minimum)
                && edges.windows(2).all(|w| w[0] < w[1])
        };
        let cells = cell_count(self.case_edges_k)
            .checked_mul(cell_count(self.vds_edges_v))
            .and_then(|n| n.checked_mul(cell_count(self.id_edges_a)));
        if !axis(self.vds_edges_v, 0.0)
            || !axis(self.id_edges_a, 0.0)
            || !axis(self.case_edges_k, 0.0)
            || self.case_edges_k[0] <= 0.0
            || cells != Some(self.limits.len())
        {
            return Err(invalid("недопустимая сетка SOA ключа"));
        }
        if let Some(source) = self.applicability {
            validate_source(source)?;
        }
        for limit in self.limits.iter().flatten() {
            match limit {
                SwitchSoaLimit::Pulse(range) => validate_range(Some(range), |v| v >= 0.0)?,
                SwitchSoaLimit::Continuous(source) => validate_source(source)?,
            }
        }
        Ok(())
    }

    fn allowance(
        self,
        scope: Scope,
        vds: SwitchBounds,
        id: SwitchBounds,
        case: SwitchBounds,
    ) -> Option<[SwitchSoaTime; 2]> {
        let t = covering_cells(self.case_edges_k, case)?;
        let v = covering_cells(self.vds_edges_v, vds)?;
        let i = covering_cells(self.id_edges_a, id)?;
        let nv = cell_count(self.vds_edges_v);
        let ni = cell_count(self.id_edges_a);
        let limits = t
            .flat_map(|t| v.clone().map(move |v| t * nv + v))
            .flat_map(|row| i.clone().map(move |i| row * ni + i))
            .map(|index| match self.limits[index]? {
                SwitchSoaLimit::Pulse(range) => {
                    let range = bounds(scope, Some(range))?;
                    Some([
                        SwitchSoaTime::Seconds(range.min),
                        SwitchSoaTime::Seconds(range.max),
                    ])
                }
                SwitchSoaLimit::Continuous(source) if confirmed(scope, Some(source)) => {
                    Some([SwitchSoaTime::Continuous; 2])
                }
                SwitchSoaLimit::Continuous(_) => None,
            });
        let mut result: Option<[SwitchSoaTime; 2]> = None;
        for limit in limits {
            let [lo, hi] = limit?;
            result = Some(match result {
                Some([a, b]) => [if a < lo { a } else { lo }, if b > hi { b } else { hi }],
                None => [lo, hi],
            });
        }
        result
    }
}

fn cell_count(edges: &[f64]) -> usize {
    edges.len().saturating_sub(1).max(1)
}

fn covering_cells(edges: &[f64], values: SwitchBounds) -> Option<std::ops::RangeInclusive<usize>> {
    if values.min < edges[0] || values.max > *edges.last()? {
        return None;
    }
    let index = |x| (edges.partition_point(|&edge| edge <= x) - 1).min(cell_count(edges) - 1);
    Some(index(values.min)..=index(values.max))
}

#[derive(Clone, Copy)]
struct Pulse {
    window: Interval,
    vds: SwitchBounds,
    id: SwitchBounds,
    certain: bool,
}

fn linear_soa(
    input: &SwitchThermalInput<'_>,
    ratings: &SwitchThermalRatings<'_>,
    segments: Option<&[SwitchSegment<'_>]>,
) -> Result<Analysis<Vec<SwitchPulseEvidence>>, ProcessError> {
    let proven = |range| bounds(input.scope, range);
    let Some((segments, full_gate)) = segments.zip(proven(ratings.fully_on_vgs_v)) else {
        return Ok(open(
            "нет полного окна огибающих или порога полного включения затвора",
        ));
    };
    let mut pulses = Vec::new();
    let mut active: Option<Pulse> = None;
    let mut reverse = false;
    for segment in segments {
        let (Some(vds), Some(id), Some(vgs)) = (
            proven(segment.vds_v),
            proven(segment.id_a),
            proven(segment.vgs_v),
        ) else {
            return Ok(open(
                "нет подтверждённых Vds, Id или Vgs между тактами во всём окне",
            ));
        };
        reverse |= vds.min < 0.0 && id.min < 0.0;
        if vds.max <= 0.0 || id.max <= 0.0 || vgs.min >= full_gate.max {
            pulses.extend(active.take());
            continue;
        }
        // Ненулевой Vds и Id при недовключении с обеих сторон огибающих.
        // Соседние интервалы сомнительной классификации не разрывают импульс.
        let certain = vds.min > 0.0 && id.min > 0.0 && vgs.max < full_gate.min;
        active = Some(match active {
            Some(pulse) => Pulse {
                window: Interval {
                    end: segment.window.end,
                    ..pulse.window
                },
                vds: pulse.vds.hull(vds),
                id: pulse.id.hull(id),
                certain: pulse.certain && certain,
            },
            None => Pulse {
                window: segment.window,
                vds,
                id,
                certain,
            },
        });
    }
    pulses.extend(active);
    if reverse {
        return Ok(open(
            "огибающая допускает обратную проводимость без применимой линии SOA",
        ));
    }
    if pulses.is_empty() {
        return Ok(analyzed(Some(Vec::new()), ""));
    }
    let Some((soa, case)) = ratings.soa.zip(proven(input.case_k)) else {
        return Ok(open("нужны линия SOA и подтверждённая температура корпуса"));
    };
    if !confirmed(input.scope, soa.applicability) {
        return Ok(open(
            "нет подтверждения применимости SOA к корпусу и условиям импульсов",
        ));
    }
    if soa.duty == SwitchSoaDuty::SinglePulse && pulses.len() > 1 {
        return Ok(open(
            "линия одиночного импульса не покрывает серию импульсов",
        ));
    }
    let result = pulses
        .into_iter()
        .map(|pulse| pulse_evidence(input, soa, case, pulse))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(analyzed(Some(result), ""))
}

fn pulse_evidence(
    input: &SwitchThermalInput<'_>,
    soa: SwitchSoa<'_>,
    case: SwitchBounds,
    pulse: Pulse,
) -> Result<SwitchPulseEvidence, ProcessError> {
    let elapsed = duration(pulse.window)?;
    let closed = pulse.window.start > input.window.start && pulse.window.end < input.window.end;
    let allowed = soa.allowance(input.scope, pulse.vds, pulse.id, case);
    let margin = match allowed {
        Some([SwitchSoaTime::Seconds(lo), SwitchSoaTime::Seconds(hi)])
            if pulse.certain && closed =>
        {
            Some(SwitchBounds { min: lo, max: hi }.difference(elapsed)?)
        }
        _ => None,
    };
    let (status, reason) = match (pulse.certain, allowed) {
        (false, _) => (
            Status::NotAnalyzed,
            "границы V/I или Vgs не разрешают длительность линейного участка",
        ),
        (_, None) => (
            Status::NotAnalyzed,
            "часть огибающей V/I/Tc без применимой линии SOA",
        ),
        (_, Some([_, SwitchSoaTime::Seconds(hi)])) if elapsed.min > hi => (
            Status::Fail,
            "подтверждённая длительность линейного режима превышает SOA",
        ),
        (_, Some([SwitchSoaTime::Continuous, _])) => {
            (Status::Pass, "вся огибающая в непрерывной SOA")
        }
        (_, Some([SwitchSoaTime::Seconds(lo), _])) if closed && elapsed.max < lo => (
            Status::Pass,
            "подтверждённая длительность с запасом укладывается в SOA",
        ),
        _ if !closed => (
            Status::NotAnalyzed,
            "полная длительность импульса выходит за окно",
        ),
        _ => (
            Status::NotAnalyzed,
            "длительность в границах неопределённости предела SOA",
        ),
    };
    Ok(SwitchPulseEvidence {
        window: pulse.window,
        duration_s: elapsed,
        closed,
        vds_v: pulse.vds,
        id_a: pulse.id,
        case_k: case,
        allowed: analyzed(allowed, "часть огибающей V/I/Tc без применимой линии SOA"),
        margin_s: analyzed(
            margin,
            "для конечного запаса нужны две границы длительности и линии SOA",
        ),
        status,
        reason: reason.into(),
    })
}

fn analyzed<T>(value: Option<T>, reason: &str) -> Analysis<T> {
    match value {
        Some(value) => Analysis::Analyzed {
            value,
            scope: WINDOW_SCOPE.into(),
        },
        None => open(reason),
    }
}

fn margin(
    observed: Option<SwitchBounds>,
    limit: Option<SwitchBounds>,
    missing_observed: &str,
    missing_limit: &str,
) -> Result<Analysis<SwitchMargin>, ProcessError> {
    match (observed, limit) {
        (Some(observed), Some(limit)) => Ok(analyzed(
            Some(SwitchMargin {
                observed,
                limit,
                margin: limit.difference(observed)?,
            }),
            "",
        )),
        (None, _) => Ok(open(missing_observed)),
        _ => Ok(open(missing_limit)),
    }
}

fn summarize(e: &SwitchThermalEvidence, horizon_proven: bool) -> (Status, String) {
    let mut gaps = Vec::new();
    let mut violations = Vec::new();
    let mut check = |label, metric: &Analysis<SwitchMargin>| match metric {
        Analysis::Analyzed { value, .. } if value.margin.max < 0.0 => Status::Fail,
        Analysis::Analyzed { value, .. } if value.margin.min > 0.0 => Status::Pass,
        Analysis::Analyzed { .. } => {
            gaps.push(format!("{label} в границах неопределённости предела"));
            Status::NotAnalyzed
        }
        Analysis::NotAnalyzed { reason } => {
            gaps.push(reason.clone());
            Status::NotAnalyzed
        }
    };
    let agreement = match &e.agreement_k {
        Analysis::Analyzed { value, .. } if value.observed.max <= value.limit.min => Status::Pass,
        Analysis::Analyzed { .. } => {
            return (
                Status::NotAnalyzed,
                "согласие θja и Zθ(t) в заявленной погрешности не подтверждено".into(),
            );
        }
        Analysis::NotAnalyzed { .. } => check("расхождение тепловых оценок", &e.agreement_k),
    };
    let peak = check("пик Tj", &e.foster_peak_junction_k);
    if agreement == Status::Pass && peak == Status::Fail {
        violations.push("пик Tj выше предела");
    }
    if e.sustained {
        let steady = check("стационарная Tj", &e.steady_junction_k);
        let equilibrium = check(
            "предел Фостера при длительном нагреве",
            &e.foster_equilibrium_junction_k,
        );
        if !horizon_proven {
            gaps.push("нет подтверждения продолжения длительного режима".into());
        } else if agreement == Status::Pass && steady == Status::Fail && equilibrium == Status::Fail
        {
            violations.push("Tj в длительном режиме выше предела");
        } else if steady == Status::Fail || equilibrium == Status::Fail {
            gaps.push("запас длительного режима не подтверждён обоими методами".into());
        }
    }
    match &e.linear_soa {
        Analysis::Analyzed { value, .. } => {
            for pulse in value {
                match pulse.status {
                    Status::Fail => violations.push(&pulse.reason),
                    Status::NotAnalyzed => gaps.push(pulse.reason.clone()),
                    Status::Pass => {}
                }
            }
        }
        Analysis::NotAnalyzed { reason } => gaps.push(reason.clone()),
    }
    if !violations.is_empty() {
        (Status::Fail, violations.join(". "))
    } else if !gaps.is_empty() {
        (Status::NotAnalyzed, gaps.join(". "))
    } else {
        (
            Status::Pass,
            "Tj и длительность линейной работы в пределах для этой ветви и окна".into(),
        )
    }
}
