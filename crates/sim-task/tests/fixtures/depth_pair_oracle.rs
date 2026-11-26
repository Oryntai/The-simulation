//! Независимый перебор проекций и проверка их допустимых продолжений.

use std::collections::{BTreeMap, BTreeSet};

use sim_task::campaign::depth_factors::{DepthFactor, DepthFactors};

type Level<'a> = (&'a str, &'a str);
pub type Registry<'a, const N: usize> = BTreeSet<[Level<'a>; N]>;

fn has_extension<'a>(
    description: &'a DepthFactors,
    fixed: &mut BTreeMap<&'a str, &'a str>,
) -> bool {
    let active = description.constraints().values().find(|rule| {
        rule.when.iter().all(|(id, forbidden)| {
            fixed
                .get(id.as_str())
                .is_none_or(|&selected| selected == forbidden)
        })
    });
    let Some(rule) = active else {
        // Каждое правило уже опровергнуто. Остальные факторы можно заполнить любыми уровнями.
        return true;
    };
    // Любое допустимое продолжение обязано опровергнуть хотя бы один член этого
    // запрета. Пробуем только такие присваивания. Если все члены уже совпали,
    // исправить выбор оставшимися факторами невозможно.
    for (id, forbidden) in &rule.when {
        if fixed.contains_key(id.as_str()) {
            continue;
        }
        for level in description.factors()[id].levels.level_ids() {
            if level == forbidden {
                continue;
            }
            fixed.insert(id, level);
            let admissible = has_extension(description, fixed);
            fixed.remove(id.as_str());
            if admissible {
                return true;
            }
        }
    }
    false
}

fn collect_interactions<'a, const N: usize>(
    description: &'a DepthFactors,
    remaining: &[(&'a str, &'a DepthFactor)],
    tuple: &mut Vec<Level<'a>>,
    registry: &mut Registry<'a, N>,
) {
    let needed = N - tuple.len();
    if needed == 0 {
        if has_extension(description, &mut tuple.iter().copied().collect()) {
            registry.insert(
                tuple
                    .as_slice()
                    .try_into()
                    .expect("размер кортежа проверен"),
            );
        }
        return;
    }
    if remaining.len() < needed {
        return;
    }
    // Перебираются только уровни выбранных пар или троек.
    for (i, &(id, factor)) in remaining[..=remaining.len() - needed].iter().enumerate() {
        for level in factor.levels.level_ids() {
            tuple.push((id, level));
            collect_interactions(description, &remaining[i + 1..], tuple, registry);
            tuple.pop();
        }
    }
}

pub fn required_interactions<const N: usize>(
    description: &DepthFactors,
    safety_only: bool,
) -> Registry<'_, N> {
    let factors: Vec<_> = description
        .factors()
        .iter()
        .filter(|(_, factor)| !safety_only || factor.safety)
        .map(|(id, factor)| (id.as_str(), factor))
        .collect();
    let mut registry = BTreeSet::new();
    collect_interactions(description, &factors, &mut Vec::new(), &mut registry);
    registry
}
