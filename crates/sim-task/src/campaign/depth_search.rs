//! Адресное уточнение после покрытия. Один следующий опыт меняет один числовой
//! фактор на соседний уровень из манифеста. Квотой и запуском владеет координатор.

use std::{cmp::Ordering, collections::BTreeMap, fmt};

use serde::{Deserialize, Serialize};

use super::{
    RunKey, Window,
    depth_factors::{DepthFactors, DepthLevels, Selection},
};
use crate::{Error, Fingerprint, Limit, Outcome, Signal, Source, SourceKind, verdict::LimitValue};

#[derive(Debug)]
pub enum SearchError {
    Factors(Error),
    Invalid(String),
}

impl fmt::Display for SearchError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Factors(error) => write!(f, "уровни уточнения: {error}"),
            Self::Invalid(reason) => f.write_str(reason),
        }
    }
}

impl std::error::Error for SearchError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Factors(error) => Some(error),
            Self::Invalid(_) => None,
        }
    }
}

impl From<Error> for SearchError {
    fn from(error: Error) -> Self {
        Self::Factors(error)
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Normalization {
    /// Положительная характерная величина в единицах сигнала, общая для критерия.
    pub scale: f64,
    pub source: Source,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Criterion {
    pub signal: Signal,
    pub window: Window,
    pub normalization: Option<Normalization>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct MarginEvidence {
    /// Огибающая ряда в окне критерия с учётом заявленной погрешности.
    pub observed: Option<[f64; 2]>,
    pub limit: Option<LimitValue>,
    /// Применимый источник всех переданных границ, включая огибающую кривой предела.
    pub limit_source: Option<Source>,
    /// Исход именно этого критерия по штатному оценщику.
    pub outcome: Outcome,
}

/// Переносит порог штатного оценщика в единицы и окно адресного поиска.
pub fn criterion_margin(
    limit: &Limit,
    window: Window,
    observed: Option<[f64; 2]>,
    outcome: Outcome,
) -> (Criterion, MarginEvidence) {
    // Для диапазона масштаб задаёт наибольший модуль предела. У двоичного
    // запрета расстояние между состояниями 0 и 1 равно единице.
    let (lower, upper, scale, source) = match limit {
        Limit::Range { min, max, source, .. } => (
            min.map(|v| [v; 2]),
            max.map(|v| [v; 2]),
            min.iter().chain(max).map(|v| v.abs()).reduce(f64::max),
            source,
        ),
        Limit::Forbidden { source, .. } => (None, Some([0.0; 2]), Some(1.0), source),
    };
    (
        Criterion {
            signal: limit.signal().clone(),
            window,
            normalization: scale.filter(|&scale| scale > 0.0)
                .map(|scale| Normalization { scale, source: source.clone() }),
        },
        MarginEvidence {
            limit: Some(LimitValue { lower, upper }),
            limit_source: Some(source.clone()),
            observed,
            outcome,
        },
    )
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum Margin {
    Scored { normalized: f64, outcome: Outcome },
    NotAnalyzed { reason: &'static str },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub struct RunWitness {
    /// Общая ревизия установки и плана покрытия.
    pub revision: Fingerprint,
    /// Отпечаток фактически установленных входов конкретной строки.
    pub input: Fingerprint,
    pub journal: Fingerprint,
    pub seed: u64,
    pub horizon_ns: u64,
    /// Все требуемые инъекции наблюдались. Для штатного опыта без инъекций — true.
    pub injections_confirmed: bool,
}

#[derive(Clone, Debug)]
pub struct SearchRun {
    pub key: RunKey,
    /// Квитанция только фактически завершённого опыта с проверенным журналом.
    pub witness: Option<RunWitness>,
    /// Оценки из того же журнала, в единицах и окне соответствующего критерия.
    pub evidence: BTreeMap<String, MarginEvidence>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct RankedRun {
    pub key: RunKey,
    pub criterion: String,
    pub window: Window,
    pub witness: RunWitness,
    pub normalized: f64,
    pub outcome: Outcome,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct Boundary {
    pub criterion: String,
    pub factor: String,
    /// Порядок по физическому значению фактора. Стороны имеют разные исходы критерия.
    pub lower: RunKey,
    pub upper: RunKey,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Probe {
    pub key: RunKey,
    pub selection: Selection,
    pub factor: String,
    pub anchor: RankedRun,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "status", content = "probe", rename_all = "snake_case")]
pub enum SearchStep {
    Probe(Probe),
    BudgetExhausted,
    NoUntriedNeighbors,
    NotAnalyzed,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct OpenMargin {
    pub key: RunKey,
    pub criterion: Option<String>,
    pub reason: &'static str,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Refinement {
    pub worst: Option<RankedRun>,
    /// Подтверждённые переходы между соседними дискретными точками в истории.
    pub boundaries: Vec<Boundary>,
    pub not_analyzed: Vec<OpenMargin>,
    pub step: SearchStep,
}

fn valid_bounds([lo, hi]: [f64; 2]) -> bool {
    lo.is_finite() && hi.is_finite() && lo <= hi
}

fn usable_source(source: &Source) -> bool {
    source.kind != SourceKind::Estimate && !source.reference.trim().is_empty()
}

fn validate_scale(normalization: Option<&Normalization>) -> Result<(), SearchError> {
    if normalization.is_some_and(|norm| !norm.scale.is_finite() || norm.scale <= 0.0) {
        return Err(SearchError::Invalid(
            "масштаб нормирования должен быть конечным и положительным".into(),
        ));
    }
    Ok(())
}

fn distance(hi: f64, lo: f64, scale: f64) -> Result<f64, SearchError> {
    let difference = hi - lo;
    let normalized = if difference.is_finite() {
        difference / scale
    } else {
        // Если разность переполнилась, операнды имеют разные знаки.
        hi / scale - lo / scale
    };
    if !normalized.is_finite() || (normalized == 0.0 && hi != lo) {
        return Err(SearchError::Invalid(
            "запас не представим конечным числом с сохранением его знака".into(),
        ));
    }
    Ok(if normalized == 0.0 { 0.0 } else { normalized })
}

/// Запас — наименьшее расстояние до применимых пределов, делённое на масштаб
/// критерия. Для нижнего предела берётся его верхняя оценка, для верхнего —
/// нижняя. Расчёт пригоден для ранжирования в данном окне и сохраняет исход оценщика.
pub fn normalized_margin(
    evidence: &MarginEvidence,
    normalization: Option<&Normalization>,
) -> Result<Margin, SearchError> {
    validate_scale(normalization)?;
    let (lower, upper) = evidence
        .limit
        .as_ref()
        .map_or((None, None), |limit| (limit.lower, limit.upper));
    if evidence
        .observed
        .into_iter()
        .chain(lower)
        .chain(upper)
        .any(|bounds| !valid_bounds(bounds))
        || lower.zip(upper).is_some_and(|(lo, hi)| lo[0] > hi[1])
    {
        return Err(SearchError::Invalid(
            "огибающая ряда или предела неконечна либо имеет обратные границы".into(),
        ));
    }
    let open = |reason| Ok(Margin::NotAnalyzed { reason });
    if evidence.outcome == Outcome::NotAnalyzed {
        return open("исход критерия остаётся открытым");
    }
    let Some([lo, hi]) = evidence.observed else {
        return open("нет огибающей наблюдения в окне критерия");
    };
    if !evidence.limit_source.as_ref().is_some_and(usable_source) {
        return open("нет применимого источника предела");
    }
    let Some(norm) = normalization.filter(|norm| usable_source(&norm.source)) else {
        return open("нет масштаба нормирования с источником");
    };
    let normalized = match (lower, upper) {
        (Some(lower), Some(upper)) => {
            distance(lo, lower[1], norm.scale)?.min(distance(upper[0], hi, norm.scale)?)
        }
        (Some(lower), None) => distance(lo, lower[1], norm.scale)?,
        (None, Some(upper)) => distance(upper[0], hi, norm.scale)?,
        (None, None) => return open("нет границ применимого предела"),
    };
    if (evidence.outcome == Outcome::Pass && normalized < 0.0)
        || (evidence.outcome == Outcome::Fail && normalized > 0.0)
    {
        return open("исход критерия не подтверждается запасом по переданным границам");
    }
    Ok(Margin::Scored {
        normalized,
        outcome: evidence.outcome,
    })
}

fn rank(a: &RankedRun, b: &RankedRun) -> Ordering {
    a.normalized
        .total_cmp(&b.normalized)
        // На точном пороге оценщик может применять строгое неравенство.
        .then_with(|| b.outcome.cmp(&a.outcome))
        .then_with(|| a.key.cmp(&b.key))
        .then_with(|| a.criterion.cmp(&b.criterion))
}

type Scores<'a> = BTreeMap<&'a str, RankedRun>;
type History<'a> = BTreeMap<Selection, Scores<'a>>;
type Axes<'a> = BTreeMap<&'a str, BTreeMap<&'a str, [Option<&'a str>; 2]>>;

fn checked_history<'a>(
    factors: &DepthFactors,
    criteria: &BTreeMap<String, Criterion>,
    runs: &'a [SearchRun],
    not_analyzed: &mut Vec<OpenMargin>,
) -> Result<History<'a>, SearchError> {
    for (id, criterion) in criteria {
        if [id, &criterion.signal.name, &criterion.signal.unit]
            .into_iter()
            .any(|name| name.trim().is_empty())
            || criterion.window.start_ns >= criterion.window.end_ns
            || criterion.window.max_gap_ns == 0
        {
            return Err(SearchError::Invalid(format!("неверный критерий «{id}»")));
        }
        validate_scale(criterion.normalization.as_ref())?;
    }

    let mut history = BTreeMap::new();
    let mut revision = None;
    for run in runs {
        let selection = factors.selection(&run.key)?;
        if factors.exclusion(&selection)?.is_some() {
            return Err(SearchError::Invalid(format!(
                "в истории уточнения есть физически исключённая строка {:?}",
                run.key
            )));
        }
        let mut scored = BTreeMap::new();
        if let Some(witness) = run.witness {
            if revision
                .replace(witness.revision)
                .is_some_and(|seen| seen != witness.revision)
            {
                return Err(SearchError::Invalid(
                    "смешаны ревизии опытов уточнения".into(),
                ));
            }
        }
        if run.evidence.is_empty() {
            not_analyzed.push(OpenMargin {
                key: run.key.clone(),
                criterion: None,
                reason: "для строки нет наблюдений критериев",
            });
        }
        for (id, evidence) in &run.evidence {
            let criterion = criteria.get(id).ok_or_else(|| {
                SearchError::Invalid(format!("неизвестный критерий уточнения «{id}»"))
            })?;
            let margin = normalized_margin(evidence, criterion.normalization.as_ref())?;
            let checked = match (run.witness, margin) {
                (None, _) => Err("нет квитанции завершённого опыта"),
                (Some(witness), _) if !witness.injections_confirmed => {
                    Err("отсутствует подтверждение инъекций")
                }
                (Some(witness), _) if witness.horizon_ns < criterion.window.end_ns => {
                    Err("журнал не достигает конца окна критерия")
                }
                (_, Margin::NotAnalyzed { reason }) => Err(reason),
                (
                    Some(witness),
                    Margin::Scored {
                        normalized,
                        outcome,
                    },
                ) => Ok((witness, normalized, outcome)),
            };
            let (witness, normalized, outcome) = match checked {
                Ok(checked) => checked,
                Err(reason) => {
                    not_analyzed.push(OpenMargin {
                        key: run.key.clone(),
                        criterion: Some(id.clone()),
                        reason,
                    });
                    continue;
                }
            };
            scored.insert(
                id.as_str(),
                RankedRun {
                    key: run.key.clone(),
                    criterion: id.clone(),
                    window: criterion.window,
                    witness,
                    normalized,
                    outcome,
                },
            );
        }
        if history.insert(selection, scored).is_some() {
            return Err(SearchError::Invalid(format!(
                "повтор ключа в истории уточнения {:?}",
                run.key
            )));
        }
    }
    Ok(history)
}

fn neighbors(factors: &DepthFactors) -> Axes<'_> {
    factors
        .factors()
        .iter()
        .filter_map(|(id, factor)| match &factor.levels {
            DepthLevels::Axis { values, .. } if usable_source(&factor.source) => {
                let mut ordered: Vec<_> = values.keys().map(String::as_str).collect();
                ordered.sort_by(|a, b| values[*a].total_cmp(&values[*b]));
                let adjacent = ordered
                    .iter()
                    .enumerate()
                    .map(|(index, &level)| {
                        let lower = index.checked_sub(1).map(|index| ordered[index]);
                        let upper = ordered.get(index + 1).copied();
                        (level, [lower, upper])
                    })
                    .collect();
                Some((id.as_str(), adjacent))
            }
            _ => None,
        })
        .collect()
}

fn at_level(selection: &Selection, factor: &str, level: &str) -> Selection {
    let mut adjacent = selection.clone();
    adjacent.insert(factor.into(), level.into());
    adjacent
}

fn checked_boundaries(axes: &Axes<'_>, history: &History<'_>) -> Vec<Boundary> {
    let mut boundaries = Vec::new();
    for (selection, scores) in history.iter().filter(|(_, scores)| !scores.is_empty()) {
        for (&factor, levels) in axes {
            let Some(level) = levels[selection[factor].as_str()][1] else {
                continue;
            };
            let Some(upper) = history.get(&at_level(selection, factor, level)) else {
                continue;
            };
            for (&id, a) in scores {
                if let Some(b) = upper.get(id)
                    && a.outcome != b.outcome
                    && a.witness.horizon_ns == b.witness.horizon_ns
                {
                    boundaries.push(Boundary {
                        criterion: id.into(),
                        factor: factor.into(),
                        lower: a.key.clone(),
                        upper: b.key.clone(),
                    });
                }
            }
        }
    }
    boundaries
}

fn probe_near(
    factors: &DepthFactors,
    axes: &Axes<'_>,
    history: &History<'_>,
    anchor: &RankedRun,
) -> Result<Option<Probe>, SearchError> {
    let selection = factors.selection(&anchor.key)?;
    let mut best: Option<(Selection, &str, Option<f64>)> = None;
    for (&factor, levels) in axes {
        let choices = levels[selection[factor].as_str()]
            .map(|level| level.map(|level| at_level(&selection, factor, level)));
        let support = |other: Option<&Selection>| {
            history
                .get(other?)?
                .get(anchor.criterion.as_str())
                .filter(|score| score.witness.horizon_ns == anchor.witness.horizon_ns)
                .map(|score| score.normalized)
                .filter(|&margin| margin > anchor.normalized)
        };
        let trends = [support(choices[1].as_ref()), support(choices[0].as_ref())];
        for (choice, trend) in choices.into_iter().zip(trends) {
            let Some(choice) = choice else { continue };
            if !history.contains_key(&choice)
                && best.as_ref().is_none_or(|(_, _, prior)| trend > *prior)
                && factors.exclusion(&choice)?.is_none()
            {
                best = Some((choice, factor, trend));
            }
        }
    }
    best.map(|(selection, factor, _)| {
        Ok(Probe {
            key: factors.run_key(&selection)?,
            selection,
            factor: factor.into(),
            anchor: anchor.clone(),
        })
    })
    .transpose()
}

/// Принимает историю покрытия и предыдущих уточнений, включая строки, которые
/// координатор оставил открытыми. Для каждого критерия рассматривает соседей
/// его худшей подтверждённой строки. Предпочитает сторону, к которой запас уже
/// убывал от проверенного соседа. При равенстве — порядок факторов и их значений.
/// Остаток квоты передаёт координатор после резерва повтора худшего случая.
/// Предложенную строку координатор разрешает в DepthPlan и оплачивает до запуска.
pub fn refine(
    factors: &DepthFactors,
    criteria: &BTreeMap<String, Criterion>,
    runs: &[SearchRun],
    remaining_runs: usize,
) -> Result<Refinement, SearchError> {
    let mut not_analyzed = Vec::new();
    let history = checked_history(factors, criteria, runs, &mut not_analyzed)?;
    not_analyzed.sort_by(|a, b| (&a.key, &a.criterion).cmp(&(&b.key, &b.criterion)));
    let axes = neighbors(factors);
    let boundaries = checked_boundaries(&axes, &history);
    let mut best = BTreeMap::<&str, &RankedRun>::new();
    for scores in history.values() {
        for (&id, score) in scores {
            if best.get(id).is_none_or(|old| rank(score, old).is_lt()) {
                best.insert(id, score);
            }
        }
    }
    let mut anchors: Vec<_> = best.into_values().collect();
    anchors.sort_by(|a, b| rank(a, b));
    let worst = anchors.first().map(|&anchor| anchor.clone());
    let mut next = None;
    for anchor in anchors {
        next = probe_near(factors, &axes, &history, anchor)?;
        if next.is_some() {
            break;
        }
    }

    let step = match next {
        Some(_) if remaining_runs == 0 => SearchStep::BudgetExhausted,
        Some(probe) => SearchStep::Probe(probe),
        None if worst.is_none() => SearchStep::NotAnalyzed,
        None => SearchStep::NoUntriedNeighbors,
    };
    Ok(Refinement {
        worst,
        boundaries,
        not_analyzed,
        step,
    })
}
