//! Покрытие конечных уровней. Допустимы взаимодействия, для которых существует
//! хотя бы одна полная строка, удовлетворяющая физическим ограничениям.

use std::{cmp::Reverse, collections::BTreeSet, fmt, ops::Range};

use serde::Serialize;

use super::{
    RunKey,
    depth_factors::{DepthFactors, Selection},
};
use crate::Error;

// Блок Е третьего ТЗ. Координатор отдельно резервирует бюджет дочерних опытов и повтора.
const MAX_ROWS: usize = 1000;

/// Идентификаторы фактора и его уровня. В кортежах факторы упорядочены по ID.
pub type FactorLevel = (String, String);

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct Coverage<T> {
    required: BTreeSet<T>,
    unplanned: BTreeSet<T>,
}

impl<T> Coverage<T> {
    pub fn required(&self) -> &BTreeSet<T> {
        &self.required
    }

    pub fn unplanned(&self) -> &BTreeSet<T> {
        &self.unplanned
    }

    pub fn planned(&self) -> usize {
        self.required.len() - self.unplanned.len()
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct CoverRow {
    pub selection: Selection,
    pub key: RunKey,
}

/// Дискретный план. Строки ещё должны быть разрешены установщиком и исполнены.
/// Пробел установщика не исключает взаимодействие из реестра требований.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct DepthCover {
    pub rows: Vec<CoverRow>,
    pub pairs: Coverage<[FactorLevel; 2]>,
    pub safety_triples: Coverage<[FactorLevel; 3]>,
}

impl DepthCover {
    pub fn is_complete(&self) -> bool {
        self.pairs.unplanned.is_empty() && self.safety_triples.unplanned.is_empty()
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum CoverError {
    RowLimit { requested: usize },
    NoAdmissibleRows,
    RowKey(Error),
}

impl fmt::Display for CoverError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::RowLimit { requested } => {
                write!(f, "запрошено {requested} строк при пределе {MAX_ROWS}")
            }
            Self::NoAdmissibleRows => f.write_str("нет ни одной допустимой строки покрытия"),
            Self::RowKey(error) => write!(f, "не удалось получить ключ строки покрытия: {error}"),
        }
    }
}

impl std::error::Error for CoverError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::RowKey(error) => Some(error),
            _ => None,
        }
    }
}

type Level = (usize, usize);
type Interactions<const N: usize> = BTreeSet<[Level; N]>;

#[derive(Clone)]
struct Uncovered {
    pairs: Interactions<2>,
    triples: Interactions<3>,
}

struct Factor<'a> {
    id: &'a str,
    levels: Vec<&'a str>,
    safety: bool,
}

struct Search<'a> {
    factors: Vec<Factor<'a>>,
    constraints: Vec<Vec<Level>>,
    by_factor: Vec<Vec<usize>>,
    order: Vec<usize>,
    constrained: usize,
}

impl<'a> Search<'a> {
    fn new(description: &'a DepthFactors) -> Self {
        let factors: Vec<_> = description
            .factors()
            .iter()
            .map(|(id, factor)| Factor {
                id,
                levels: factor.levels.level_ids().collect(),
                safety: factor.safety,
            })
            .collect();
        let mut constraints = Vec::new();
        let mut by_factor = vec![Vec::new(); factors.len()];
        for rule in description.constraints().values() {
            let mut indexed = Vec::new();
            for (id, level) in &rule.when {
                let factor = factors
                    .binary_search_by_key(&id.as_str(), |factor| factor.id)
                    .expect("ID фактора проверен при чтении");
                let level = factors[factor]
                    .levels
                    .binary_search(&level.as_str())
                    .expect("ID уровня проверен при чтении");
                by_factor[factor].push(constraints.len());
                indexed.push((factor, level));
            }
            constraints.push(indexed);
        }
        let mut order: Vec<_> = (0..factors.len()).collect();
        order.sort_unstable_by_key(|&i| (Reverse(by_factor[i].len()), factors[i].levels.len(), i));
        let constrained = order.partition_point(|&i| !by_factor[i].is_empty());
        Self {
            factors,
            constraints,
            by_factor,
            order,
            constrained,
        }
    }

    fn fits(&self, (factor, level): Level, selected: &[Option<usize>]) -> bool {
        !self.by_factor[factor].iter().any(|&rule| {
            self.constraints[rule].iter().all(|&(i, forbidden)| {
                if i == factor {
                    level == forbidden
                } else {
                    selected[i] == Some(forbidden)
                }
            })
        })
    }

    fn start(&self, fixed: &[Level]) -> Option<Vec<Option<usize>>> {
        let mut selected = vec![None; self.factors.len()];
        for &(factor, level) in fixed {
            if !self.fits((factor, level), &selected) {
                return None;
            }
            selected[factor] = Some(level);
        }
        Some(selected)
    }

    fn extend(
        &self,
        order: &[usize],
        selected: &mut [Option<usize>],
        open: Option<&Uncovered>,
    ) -> bool {
        let Some((offset, &factor)) = order
            .iter()
            .enumerate()
            .find(|&(_, &i)| selected[i].is_none())
        else {
            return true;
        };
        let rest = &order[offset + 1..];
        let available = (0..self.factors[factor].levels.len())
            .filter(|&level| self.fits((factor, level), selected));
        // Для реестра достаточно первого продолжения. Для строки выше приоритет
        // у уровня, добавляющего больше ещё непокрытых пар, затем троек.
        let mut levels: Vec<_> = available.collect();
        if let Some(open) = open {
            levels.sort_by_cached_key(|&level| {
                (Reverse(self.gain((factor, level), selected, open)), level)
            });
        }
        for level in levels {
            selected[factor] = Some(level);
            if self.extend(rest, selected, open) {
                return true;
            }
        }
        selected[factor] = None;
        false
    }

    fn gain(&self, level: Level, selected: &[Option<usize>], open: &Uncovered) -> (usize, usize) {
        let mut pairs = 0;
        let mut triples = 0;
        let check_triples = self.factors[level.0].safety && !open.triples.is_empty();
        for (i, &value) in selected.iter().enumerate() {
            let Some(value) = value else { continue };
            let mut pair = [level, (i, value)];
            pair.sort_unstable();
            pairs += usize::from(open.pairs.contains(&pair));
            if check_triples && self.factors[i].safety {
                for (j, &other) in selected[..i].iter().enumerate() {
                    if let Some(other) = other.filter(|_| self.factors[j].safety) {
                        let mut triple = [level, (i, value), (j, other)];
                        triple.sort_unstable();
                        triples += usize::from(open.triples.contains(&triple));
                    }
                }
            }
        }
        (pairs, triples)
    }

    fn required<const N: usize>(
        &self,
        factors: &[usize],
        domains: &[Range<usize>],
    ) -> Interactions<N> {
        let mut required = BTreeSet::new();
        visit(factors, domains, &mut [(0, 0); N], 0, &mut |tuple| {
            // Факторы вне ограничений не влияют на существование продолжения.
            let possible = self.constrained == 0
                || self.start(&tuple).is_some_and(|mut selected| {
                    self.extend(&self.order[..self.constrained], &mut selected, None)
                });
            if possible {
                required.insert(tuple);
            }
        });
        required
    }

    fn name_level(&self, (index, level): Level) -> FactorLevel {
        let factor = &self.factors[index];
        (factor.id.into(), factor.levels[level].into())
    }

    fn coverage<const N: usize>(
        &self,
        required: Interactions<N>,
        unplanned: Interactions<N>,
    ) -> Coverage<[FactorLevel; N]> {
        let named = |set: Interactions<N>| {
            set.into_iter()
                .map(|tuple| tuple.map(|level| self.name_level(level)))
                .collect()
        };
        Coverage {
            required: named(required),
            unplanned: named(unplanned),
        }
    }
}

// Глубина этого перебора равна только размеру пары или тройки безопасности.
// Для снятия покрытия области сужаются до уровней уже выбранной строки.
fn visit<const N: usize>(
    factors: &[usize],
    domains: &[Range<usize>],
    tuple: &mut [Level; N],
    chosen: usize,
    emit: &mut impl FnMut([Level; N]),
) {
    let needed = N - chosen;
    if needed == 0 {
        emit(*tuple);
    } else if factors.len() >= needed {
        for (i, &factor) in factors[..=factors.len() - needed].iter().enumerate() {
            for level in domains[factor].clone() {
                tuple[chosen] = (factor, level);
                visit(&factors[i + 1..], domains, tuple, chosen + 1, emit);
            }
        }
    }
}

/// Выбирает строки по первой непокрытой паре, после пар по первой тройке безопасности.
/// При малой квоте возвращает тот же префикс и точный остаток, в том числе при нуле.
pub fn plan_cover(description: &DepthFactors, row_limit: usize) -> Result<DepthCover, CoverError> {
    if row_limit > MAX_ROWS {
        return Err(CoverError::RowLimit {
            requested: row_limit,
        });
    }
    let search = Search::new(description);
    let all: Vec<_> = (0..search.factors.len()).collect();
    let safety: Vec<_> = all
        .iter()
        .copied()
        .filter(|&i| search.factors[i].safety)
        .collect();
    let domains: Vec<_> = search
        .factors
        .iter()
        .map(|factor| 0..factor.levels.len())
        .collect();
    // В проверенном описании есть как минимум факторы режима, пути и сетки.
    let pairs = search.required(&all, &domains);
    if pairs.is_empty() {
        return Err(CoverError::NoAdmissibleRows);
    }
    let required = Uncovered {
        pairs,
        triples: search.required(&safety, &domains),
    };
    let mut open = required.clone();
    let mut rows = Vec::new();
    for _ in 0..row_limit {
        let Some(anchor) = open
            .pairs
            .first()
            .map(|tuple| tuple.as_slice())
            .or_else(|| open.triples.first().map(|tuple| tuple.as_slice()))
        else {
            break;
        };
        let mut selected = search
            .start(anchor)
            .expect("цель проверена при построении реестра");
        assert!(
            search.extend(&search.order, &mut selected, Some(&open)),
            "цель имеет допустимое продолжение"
        );
        let row: Vec<_> = selected
            .into_iter()
            .map(|level| level.expect("строка достроена"))
            .collect();
        let selection = row
            .iter()
            .copied()
            .enumerate()
            .map(|level| search.name_level(level))
            .collect();
        let key = description
            .run_key(&selection)
            .map_err(CoverError::RowKey)?;
        rows.push(CoverRow { selection, key });

        let single: Vec<_> = row.into_iter().map(|level| level..level + 1).collect();
        visit(&all, &single, &mut [(0, 0); 2], 0, &mut |tuple| {
            open.pairs.remove(&tuple);
        });
        visit(&safety, &single, &mut [(0, 0); 3], 0, &mut |tuple| {
            open.triples.remove(&tuple);
        });
    }
    Ok(DepthCover {
        rows,
        pairs: search.coverage(required.pairs, open.pairs),
        safety_triples: search.coverage(required.triples, open.triples),
    })
}
