use super::{RegionId, model::surface_rows};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Source<'a> {
    pub reference: &'a str,
    pub locator: &'a str,
}

impl Source<'_> {
    pub(super) fn known(self) -> bool {
        !self.reference.trim().is_empty() && !self.locator.trim().is_empty()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Interval {
    pub min: f64,
    pub max: f64,
}

impl Interval {
    pub const fn point(value: f64) -> Self {
        Self {
            min: value,
            max: value,
        }
    }

    fn valid(self) -> bool {
        self.min.is_finite() && self.max.is_finite() && self.min >= 0.0 && self.max >= self.min
    }

    fn contains(self, value: f64) -> bool {
        self.min <= value && value <= self.max
    }

    fn contains_interval(self, other: Self) -> bool {
        self.contains(other.min) && self.contains(other.max)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Metric {
    NormalForce,
    TangentialForce,
    MeanPressure,
    MeanShear,
    EffectiveArea,
    EffectiveMass,
    Stiffness,
    ImpactWindow,
    InternalShear,
}

impl Metric {
    pub const fn unit(self) -> &'static str {
        match self {
            Self::NormalForce | Self::TangentialForce | Self::InternalShear => "Н",
            Self::MeanPressure | Self::MeanShear => "Па",
            Self::EffectiveArea => "м²",
            Self::Stiffness => "Н/м",
            Self::EffectiveMass => "кг",
            Self::ImpactWindow => "с",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    Normal,
    Tangential,
    Axial,
    Unspecified,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadMode {
    Quasistatic,
    Transient,
    Sustained,
    Repeated,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Patch {
    Local,
    Distributed,
    Circumferential,
    Edge,
    Internal,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LimitStatus {
    Measurement,
    Derived,
    Orientation,
    Unknown,
}

/// Неизвестные численные границы хранятся явно. Для обоснования из данных
/// должны совпасть все поля, включая диапазон длительности и геометрию пятна.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Applicability<'a> {
    pub direction: Direction,
    pub mode: LoadMode,
    pub patch: Patch,
    pub area_m2: Option<Interval>,
    pub duration_s: Option<Interval>,
    pub repetitions: Option<[u32; 2]>,
    pub population: &'a str,
    /// Условия геометрии и процедуры, например идентификатор калибровки пятна.
    pub geometry: &'a str,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExposureLimit<'a> {
    pub metric: Metric,
    pub range: Option<Interval>,
    pub source: Source<'a>,
    pub applicability: Applicability<'a>,
    pub status: LimitStatus,
}

const ISO: &str = "https://www.iso.org/standard/62996.html";
const FORCE: Source<'static> = Source {
    reference: "https://arxiv.org/abs/2311.13814",
    locator: "табл. 1, значения по ISO/TS 15066; первичная таблица не сверена",
};
const PRESSURE: Source<'static> = Source {
    reference: "https://doi.org/10.1089/soro.2017.0046",
    locator: "обзор сдавления; прежние проектные ориентиры 16 и 20 кПа",
};
const HEAD: Source<'static> = Source {
    reference: "https://pubmed.ncbi.nlm.nih.gov/39983251/",
    locator: "локальная чувствительность, критерий дискомфорта",
};
const SHEAR: Source<'static> = Source {
    reference: "https://doi.org/10.1016/j.clinbiomech.2012.08.009",
    locator: "аннотация, рекомендации для внутреннего сдвига поясницы",
};
const INDUSTRIAL: Applicability<'static> = Applicability {
    direction: Direction::Normal,
    mode: LoadMode::Quasistatic,
    patch: Patch::Local,
    area_m2: None,
    duration_s: None,
    repetitions: None,
    population: "Выборка из данных о боли при контакте; перенос на индивидуальный профиль не обоснован",
    geometry: "Локальный контакт в области ISO/TS 15066",
};
const COMPRESSION: Applicability<'static> = Applicability {
    direction: Direction::Normal,
    mode: LoadMode::Sustained,
    patch: Patch::Circumferential,
    population: "Зависит от исследованной группы и участка",
    geometry: "Распределённое сдавление; ширина и площадь критичны",
    ..INDUSTRIAL
};
const SLIDE: Applicability<'static> = Applicability {
    direction: Direction::Tangential,
    mode: LoadMode::Quasistatic,
    patch: Patch::Distributed,
    population: "Взрослые участники опыта",
    geometry: "Измеренная продольная жёсткость интерфейса при 0–100 мм рт. ст.",
    ..INDUSTRIAL
};
const INTERNAL: Applicability<'static> = Applicability {
    direction: Direction::Tangential,
    mode: LoadMode::Repeated,
    patch: Patch::Internal,
    population: "Профессиональные нагрузки, данные литературы о пояснице",
    geometry: "Внутренний сдвиг L5/S1, циклы за сутки",
    ..INDUSTRIAL
};

const fn entry(
    metric: Metric,
    min: f64,
    max: f64,
    source: Source<'static>,
    applicability: Applicability<'static>,
) -> ExposureLimit<'static> {
    ExposureLimit {
        metric,
        range: Some(Interval { min, max }),
        source,
        applicability,
        status: LimitStatus::Orientation,
    }
}

const fn unknown(
    metric: Metric,
    source: Source<'static>,
    applicability: Applicability<'static>,
) -> ExposureLimit<'static> {
    ExposureLimit {
        metric,
        range: None,
        source,
        applicability,
        status: LimitStatus::Unknown,
    }
}

// Значения сохранены из прежних таблиц. Численные ориентиры не закрывают опасность.
const fn force(n: f64) -> ExposureLimit<'static> {
    entry(Metric::NormalForce, n, n, FORCE, INDUSTRIAL)
}
const fn stiffness(n_m: f64) -> ExposureLimit<'static> {
    entry(Metric::Stiffness, n_m, n_m, FORCE, INDUSTRIAL)
}
const fn effective_mass(kg: f64) -> ExposureLimit<'static> {
    entry(Metric::EffectiveMass, kg, kg, FORCE, INDUSTRIAL)
}
const WINDOW: ExposureLimit<'static> = entry(
    Metric::ImpactWindow,
    0.5,
    0.5,
    Source {
        reference: "https://arxiv.org/abs/2009.01036",
        locator: "разд. IV; прежнее окно удара",
    },
    Applicability {
        mode: LoadMode::Transient,
        ..INDUSTRIAL
    },
);
const LONG: ExposureLimit<'static> = entry(
    Metric::MeanPressure,
    4000.0,
    4000.0,
    Source {
        reference: "https://www.jci.org/articles/view/100482",
        locator: "прежний проектный ориентир 4 кПа; статья о капиллярном обмене",
    },
    COMPRESSION,
);
const DISCOMFORT: ExposureLimit<'static> = entry(
    Metric::MeanPressure,
    16000.0,
    16000.0,
    PRESSURE,
    COMPRESSION,
);
const PAIN: ExposureLimit<'static> = entry(
    Metric::MeanPressure,
    20000.0,
    20000.0,
    PRESSURE,
    COMPRESSION,
);
const MISALIGNMENT: ExposureLimit<'static> = entry(
    Metric::TangentialForce,
    30.0,
    30.0,
    Source {
        reference: "crates/sim-scenarios/src/human_limits.rs",
        locator: "MISALIGNMENT_FORCE_WARN_N, инженерная оценка",
    },
    SLIDE,
);
const INTERFACE: ExposureLimit<'static> = ExposureLimit {
    status: LimitStatus::Measurement,
    ..entry(
        Metric::Stiffness,
        1139.0,
        2232.0,
        Source {
            reference: "https://cris.vub.be/ws/files/54925842/TMRB2020_Langlois_final.pdf",
            locator: "абстракт, интервалы давления и жёсткости",
        },
        SLIDE,
    )
};
const SCHIEBL: Source<'static> = Source {
    reference: "https://doi.org/10.3390/s25154605",
    locator: "аннотация и результаты, медианы по направлениям",
};
const STIFFNESS_AXIAL: ExposureLimit<'static> = ExposureLimit {
    status: LimitStatus::Measurement,
    ..entry(
        Metric::Stiffness,
        2100.0,
        2100.0,
        SCHIEBL,
        Applicability {
            direction: Direction::Axial,
            population: "Здоровые молодые взрослые",
            geometry: "Крепление на плече, рабочие положения над головой, вдоль оси сегмента",
            ..SLIDE
        },
    )
};
const STIFFNESS_TRANSVERSE: ExposureLimit<'static> = ExposureLimit {
    status: LimitStatus::Measurement,
    ..entry(
        Metric::Stiffness,
        4500.0,
        4500.0,
        SCHIEBL,
        Applicability {
            direction: Direction::Tangential,
            population: "Здоровые молодые взрослые",
            geometry: "Крепление на плече, рабочие положения над головой, поперёк оси сегмента",
            ..SLIDE
        },
    )
};
const HEAD_CLAMP: ExposureLimit<'static> = ExposureLimit {
    status: LimitStatus::Measurement,
    ..entry(
        Metric::NormalForce,
        1.5,
        1.5,
        Source {
            reference: "https://pmc.ncbi.nlm.nih.gov/articles/PMC11365371/",
            locator: "Discussion, измеренное условие опыта с дискомфортом",
        },
        Applicability {
            patch: Patch::Local,
            duration_s: Some(Interval::point(3600.0)),
            repetitions: Some([1, 1]),
            population: "200 взрослых участников, китайская выборка",
            geometry: "Боковой прижим прототипа с регулируемой шириной",
            ..COMPRESSION
        },
    )
};

const MISSING_AREA: ExposureLimit<'static> = unknown(
    Metric::EffectiveArea,
    Source {
        reference: "docs/STAGE5_PLAN.md",
        locator: "площадь пятна требует калибровки",
    },
    INDUSTRIAL,
);
const MISSING_SHEAR: ExposureLimit<'static> = unknown(
    Metric::MeanShear,
    Source {
        reference: "docs/STAGE5_PLAN.md",
        locator: "предел сдвига поверхности требует обоснования",
    },
    SLIDE,
);

const UPPER: &[ExposureLimit<'static>] = &[
    force(150.0),
    stiffness(30000.0),
    effective_mass(3.0),
    WINDOW,
    LONG,
    DISCOMFORT,
    PAIN,
    MISALIGNMENT,
    INTERFACE,
    STIFFNESS_AXIAL,
    STIFFNESS_TRANSVERSE,
    MISSING_AREA,
    MISSING_SHEAR,
];
const LOWER: &[ExposureLimit<'static>] = &[
    force(160.0),
    stiffness(40000.0),
    effective_mass(2.0),
    WINDOW,
    LONG,
    DISCOMFORT,
    PAIN,
    MISALIGNMENT,
    MISSING_AREA,
    MISSING_SHEAR,
];
const HAND: &[ExposureLimit<'static>] = &[
    force(140.0),
    stiffness(75000.0),
    effective_mass(0.6),
    WINDOW,
    MISSING_AREA,
    MISSING_SHEAR,
];
const HEAD_LIMITS: &[ExposureLimit<'static>] = &[
    unknown(Metric::NormalForce, HEAD, INDUSTRIAL),
    unknown(Metric::MeanPressure, HEAD, INDUSTRIAL),
    HEAD_CLAMP,
    MISSING_AREA,
    MISSING_SHEAR,
];
const OTHER: &[ExposureLimit<'static>] = &[
    unknown(
        Metric::NormalForce,
        Source {
            reference: ISO,
            locator: "область стандарта; перенос тканевого предела не обоснован",
        },
        INDUSTRIAL,
    ),
    MISSING_AREA,
    MISSING_SHEAR,
];
const TRUNK: &[ExposureLimit<'static>] = &[
    entry(
        Metric::InternalShear,
        crate::data::SHEAR_OCCASIONAL_N,
        crate::data::SHEAR_OCCASIONAL_N,
        SHEAR,
        Applicability {
            repetitions: Some([1, crate::data::SHEAR_OCCASIONAL_MAX_PER_DAY as u32 - 1]),
            ..INTERNAL
        },
    ),
    entry(
        Metric::InternalShear,
        crate::data::SHEAR_REPETITIVE_N,
        crate::data::SHEAR_REPETITIVE_N,
        SHEAR,
        Applicability {
            repetitions: Some([
                crate::data::SHEAR_OCCASIONAL_MAX_PER_DAY as u32,
                crate::data::SHEAR_REPETITIVE_MAX_PER_DAY as u32,
            ]),
            ..INTERNAL
        },
    ),
    MISSING_AREA,
    MISSING_SHEAR,
];

pub fn limits(id: &RegionId) -> Option<&'static [ExposureLimit<'static>]> {
    let part = surface_rows().find(|row| row.id == id.0)?.part;
    Some(match part {
        "upper" | "shoulder" => UPPER,
        "lower" => LOWER,
        "palm" => HAND,
        "head" => HEAD_LIMITS,
        "lumbar" => TRUNK,
        _ if part.starts_with("digit") => HAND,
        _ => OTHER,
    })
}

#[derive(Clone, Debug)]
pub struct ExposureContext<'a> {
    pub direction: Direction,
    pub mode: LoadMode,
    pub patch: Patch,
    pub duration_s: f64,
    pub repetitions: u32,
    pub population: String,
    pub geometry: String,
    pub area_m2: Option<(Interval, Source<'a>)>,
    pub complete: bool,
    pub converged: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Exposure {
    pub metric: Metric,
    /// Непосредственно измеренная величина (Н и т. д.). Для давления и
    /// касательного напряжения здесь передаётся соответствующая сила в Н.
    pub value: Interval,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AssessmentIssue {
    UnknownRegion,
    InvalidInput,
    Incomplete,
    Unconverged,
    MissingArea,
    MissingLimit,
    OutsideApplicability,
    Indeterminate,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExposureOutcome {
    WithinLimit,
    Fail,
    NotAnalyzed(AssessmentIssue),
}

/// Тканевый критерий для конкретного участка с отдельным подтверждением
/// применимости к используемой модели или замеру. Выбор источника проверяет сборщик.
#[derive(Clone, Debug)]
pub struct TissueCriterion<'a> {
    pub region: RegionId,
    pub limit: ExposureLimit<'a>,
    pub validation: Source<'a>,
}

#[derive(Clone, Debug)]
pub struct Assessment<'a> {
    pub measurement: Option<Interval>,
    pub tissue: ExposureOutcome,
    pub design: ExposureOutcome,
    pub design_limit: Option<ExposureLimit<'a>>,
    pub tissue_limit: Option<ExposureLimit<'a>>,
    /// Ориентиры, пересечённые верхней оценкой. Они сохраняются для диагностики.
    pub exceeded_orientations: Vec<ExposureLimit<'static>>,
}

/// Граница конструкции передаётся отдельно от реестра тканевых данных.
/// Даже обоснованное требование конструкции не даёт положительного тканевого результата.
pub fn assess_exposure<'a>(
    id: &RegionId,
    exposure: Exposure,
    context: &ExposureContext<'_>,
    design_limit: Option<ExposureLimit<'a>>,
) -> Assessment<'a> {
    assess_exposure_with_tissue(id, exposure, context, design_limit, None)
}

pub fn assess_exposure_with_tissue<'a>(
    id: &RegionId,
    exposure: Exposure,
    context: &ExposureContext<'_>,
    design_limit: Option<ExposureLimit<'a>>,
    tissue: Option<&TissueCriterion<'a>>,
) -> Assessment<'a> {
    use AssessmentIssue as Issue;
    use ExposureOutcome as Outcome;
    let tissue_limit = tissue.map(|criterion| criterion.limit);
    let mut result = Assessment {
        measurement: None,
        tissue: Outcome::NotAnalyzed(Issue::MissingLimit),
        design: Outcome::NotAnalyzed(Issue::MissingLimit),
        design_limit,
        tissue_limit,
        exceeded_orientations: Vec::new(),
    };
    let invalid_area = context
        .area_m2
        .is_some_and(|(area, _)| !area.valid() || area.min <= 0.0);
    let invalid_limit = [design_limit, tissue_limit].into_iter().flatten().any(|l| {
        [l.range, l.applicability.area_m2, l.applicability.duration_s]
            .into_iter()
            .flatten()
            .any(|i| !i.valid())
            || l.applicability
                .repetitions
                .is_some_and(|[a, b]| a == 0 || a > b)
    });
    let quotient = matches!(exposure.metric, Metric::MeanPressure | Metric::MeanShear);
    let measurement = match (quotient, context.area_m2) {
        (true, Some((area, source))) if area.min > 0.0 && source.known() => Some(Interval {
            min: exposure.value.min / area.max,
            max: exposure.value.max / area.min,
        }),
        (true, _) => None,
        _ => Some(exposure.value),
    };
    let expected_direction = match exposure.metric {
        Metric::NormalForce | Metric::MeanPressure => Some(Direction::Normal),
        Metric::TangentialForce | Metric::MeanShear | Metric::InternalShear => {
            Some(Direction::Tangential)
        }
        _ => None,
    };
    let wrong_direction = expected_direction.is_some_and(|d| context.direction != d);
    let invalid = wrong_direction
        || !exposure.value.valid()
        || invalid_area
        || invalid_limit
        || !context.duration_s.is_finite()
        || context.duration_s <= 0.0
        || context.repetitions == 0
        || measurement.is_some_and(|i| !i.valid());
    let entries = limits(id);
    if !invalid {
        result.measurement = measurement;
    }
    // Недостоверный ряд сохраняет измеренные числа, но его пик не считается пределом.
    if let (Some(measured), Some(entries)) = (result.measurement, entries) {
        result.exceeded_orientations = entries
            .iter()
            .copied()
            .filter(|entry| {
                entry.status == LimitStatus::Orientation
                    && entry.metric == exposure.metric
                    && entry.applicability.direction == context.direction
                    && entry.range.is_some_and(|i| measured.max > i.max)
            })
            .collect();
    }
    let issue = if invalid {
        Some(Issue::InvalidInput)
    } else if entries.is_none() {
        Some(Issue::UnknownRegion)
    } else if !context.converged {
        Some(Issue::Unconverged)
    } else if measurement.is_none() {
        Some(Issue::MissingArea)
    } else {
        None
    };
    if let Some(issue) = issue {
        result.tissue = Outcome::NotAnalyzed(issue);
        result.design = Outcome::NotAnalyzed(issue);
        return result;
    }

    if !context.complete {
        result.tissue = Outcome::NotAnalyzed(Issue::Incomplete);
    }
    if let (Some(limit), Some(measured)) = (design_limit, measurement) {
        result.design = compare_limit(exposure.metric, measured, context, limit);
    }
    if let (Some(criterion), Some(measured)) = (tissue, measurement) {
        result.tissue = if &criterion.region != id {
            Outcome::NotAnalyzed(Issue::OutsideApplicability)
        } else if !criterion.validation.known() {
            Outcome::NotAnalyzed(Issue::MissingLimit)
        } else {
            compare_limit(exposure.metric, measured, context, criterion.limit)
        };
    }
    result
}

fn compare_limit(
    metric: Metric,
    measured: Interval,
    context: &ExposureContext<'_>,
    limit: ExposureLimit<'_>,
) -> ExposureOutcome {
    use AssessmentIssue as Issue;
    use ExposureOutcome as Outcome;
    let Some(allowed) = limit.range else {
        return Outcome::NotAnalyzed(Issue::MissingLimit);
    };
    if !limit.source.known()
        || !matches!(
            limit.status,
            LimitStatus::Measurement | LimitStatus::Derived
        )
    {
        return Outcome::NotAnalyzed(Issue::MissingLimit);
    }
    let a = limit.applicability;
    let applicable = a.direction != Direction::Unspecified
        && metric == limit.metric
        && a.direction == context.direction
        && a.mode == context.mode
        && a.patch == context.patch
        && !a.geometry.trim().is_empty()
        && a.geometry == context.geometry
        && !a.population.trim().is_empty()
        && a.population == context.population
        && a.duration_s.is_some_and(|d| d.contains(context.duration_s))
        && a.repetitions
            .is_some_and(|[lo, hi]| (lo..=hi).contains(&context.repetitions))
        && a.area_m2
            .zip(context.area_m2)
            .is_some_and(|(scope, (actual, source))| {
                source.known() && scope.contains_interval(actual)
            });
    if !applicable {
        Outcome::NotAnalyzed(Issue::OutsideApplicability)
    } else if measured.min > allowed.max {
        Outcome::Fail
    } else if !context.complete {
        Outcome::NotAnalyzed(Issue::Incomplete)
    } else if measured.max > allowed.min {
        Outcome::NotAnalyzed(Issue::Indeterminate)
    } else {
        Outcome::WithinLimit
    }
}
