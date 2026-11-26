//! Независимый реестр для малых постановок Т7. Сверяет точные множества
//! с ручными таблицами. Поиск продолжения использует только условия
//! постановки, без планировщика, установщика и запуска симуляции.

use std::collections::BTreeSet;

use serde::Deserialize;
use sim_task::campaign::depth_factors::{DepthFactors, Selection};

#[path = "fixtures/depth_pair_oracle.rs"]
mod depth_pair_oracle;
use depth_pair_oracle::{Registry, required_interactions};

const BLOCK: &str = include_str!("../../../devices/electrical/depth_t7/fixtures/block.toml");
const ROD: &str = include_str!("../../../devices/electrical/depth_t7/fixtures/rod.toml");

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Group<T> {
    factors: T,
    levels: Vec<T>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Fixture {
    description: DepthFactors,
    pairs: Vec<Group<[String; 2]>>,
    triples: Vec<Group<[String; 3]>>,
}

fn manual_registry<const N: usize>(groups: &[Group<[String; N]>]) -> Registry<'_, N> {
    let mut registry = BTreeSet::new();
    for group in groups {
        assert!(group.factors.windows(2).all(|ids| ids[0] < ids[1]));
        assert!(!group.levels.is_empty());
        for levels in &group.levels {
            let tuple = std::array::from_fn(|i| (group.factors[i].as_str(), levels[i].as_str()));
            assert!(registry.insert(tuple), "повтор в ручном реестре: {tuple:?}");
        }
    }
    registry
}

#[test]
fn реестр_блока_совпадает_с_ручным_перечнем() {
    let fixture: Fixture = toml::from_str(BLOCK).unwrap();
    let pairs = manual_registry(&fixture.pairs);
    let triples = manual_registry(&fixture.triples);
    // Формулы и исключения расписаны отдельно в block.toml перед ручным реестром.
    assert_eq!(pairs.len(), 48);
    assert_eq!(triples.len(), 31);
    assert_eq!(required_interactions(&fixture.description, false), pairs);
    assert_eq!(required_interactions(&fixture.description, true), triples);
}

#[test]
fn частичный_выбор_может_не_иметь_допустимого_продолжения() {
    let Fixture { description, .. } = toml::from_str(BLOCK).unwrap();
    let pairs = required_interactions::<2>(&description, false);
    let triples = required_interactions::<3>(&description, true);
    let pair = [("path", "reserve"), ("wind", "gust")];
    let triple = [("cut", "open"), ("mass", "heavy"), ("mode", "motion")];
    for tuple in [pair.as_slice(), triple.as_slice()] {
        let selection: Selection = tuple
            .iter()
            .map(|&(id, level)| (id.into(), level.into()))
            .collect();
        assert!(description.exclusion(&selection).unwrap().is_none());
    }
    assert!(!pairs.contains(&pair));
    assert!(!triples.contains(&triple));
    let [a, b, c] = triple;
    for projected in [[a, b], [a, c], [b, c]] {
        assert!(pairs.contains(&projected));
    }
    assert!(pairs.contains(&[("path", "reserve"), ("wind", "still")]));
    assert!(triples.contains(&[("cut", "open"), ("mass", "light"), ("mode", "motion")]));
    assert!(!triples.contains(&[("cut", "intact"), ("mass", "light"), ("path", "main")]));
}

#[test]
fn одноуровневые_факторы_входят_в_пары_и_тройки_по_пометкам() {
    let fixture: Fixture = toml::from_str(ROD).unwrap();
    let pairs = manual_registry(&fixture.pairs);
    assert_eq!(pairs.len(), 3); // C(3, 2), один допустимый уровень каждого фактора.
    assert_eq!(required_interactions(&fixture.description, false), pairs);
    let triples = manual_registry(&fixture.triples);
    assert!(triples.is_empty());
    assert_eq!(required_interactions(&fixture.description, true), triples);

    let all_safety: Fixture =
        toml::from_str(&ROD.replace("safety = false", "safety = true")).unwrap();
    assert_eq!(required_interactions(&all_safety.description, false), pairs);
    assert_eq!(
        required_interactions(&all_safety.description, true),
        BTreeSet::from([[("grid", "base"), ("mode", "free"), ("path", "main")]])
    );
}

#[test]
fn несовместимые_условия_оставляют_пустой_реестр() {
    let blocked = format!(
        r#"{ROD}
[description.constraints.free]
when = {{ mode = "free" }}
reason = "вторая позиция также удерживается упором"
source = {{ kind = "derived", reference = "отрицательная постановка стержня" }}
"#
    );
    let fixture: Fixture =
        toml::from_str(&blocked.replace("safety = false", "safety = true")).unwrap();
    // Оба уровня режима запрещены, включая продолжения пары, в которой режима нет.
    assert!(required_interactions::<2>(&fixture.description, false).is_empty());
    assert!(required_interactions::<3>(&fixture.description, true).is_empty());
}

#[test]
fn неверное_описание_отклоняется_до_вычисления_реестра() {
    for (valid, invalid) in [
        ("heavy = 2.0", "heavy = nan"),
        ("heavy = 2.0", "heavy = inf"),
        ("heavy = 2.0", "heavy = -inf"),
        ("mode = \"idle\", path", "mode = \"missing\", path"),
        (
            "kind = \"derived\", reference = \"таблица условий",
            "kind = \"estimate\", reference = \"таблица условий",
        ),
    ] {
        assert!(BLOCK.contains(valid));
        assert!(toml::from_str::<Fixture>(&BLOCK.replacen(valid, invalid, 1)).is_err());
    }
}
