use std::collections::BTreeSet;

use serde::Deserialize;
use sim_task::campaign::depth_factors;

use sim_task::campaign::depth_cover;

use depth_cover::{CoverError, DepthCover, FactorLevel, plan_cover};
use depth_factors::{DepthFactors, Selection};

const BLOCK: &str = include_str!("../../../devices/electrical/depth_t7/fixtures/block.toml");
const ROD: &str = include_str!("../../../devices/electrical/depth_t7/fixtures/rod.toml");

#[derive(Deserialize)]
struct Group<T> {
    factors: T,
    levels: Vec<T>,
}

#[derive(Deserialize)]
struct Fixture {
    description: DepthFactors,
    pairs: Vec<Group<[String; 2]>>,
    triples: Vec<Group<[String; 3]>>,
}

fn manual<const N: usize>(groups: &[Group<[String; N]>]) -> BTreeSet<[FactorLevel; N]> {
    groups
        .iter()
        .flat_map(|g| {
            g.levels
                .iter()
                .map(|levels| std::array::from_fn(|i| (g.factors[i].clone(), levels[i].clone())))
        })
        .collect()
}

// Независимая проекция готовых строк на ручной реестр. В тесте нет поиска продолжений.
fn remove_covered<const N: usize>(
    remaining: &mut BTreeSet<[FactorLevel; N]>,
    selection: &Selection,
) {
    remaining.retain(|tuple| {
        !tuple
            .iter()
            .all(|(id, level)| selection.get(id) == Some(level))
    });
}

fn check_rows(plan: &DepthCover, fixture: &Fixture) {
    let mut pairs = manual(&fixture.pairs);
    let mut triples = manual(&fixture.triples);
    assert_eq!(plan.pairs.required(), &pairs);
    assert_eq!(plan.safety_triples.required(), &triples);
    let mut keys = BTreeSet::new();
    for row in &plan.rows {
        assert!(
            fixture
                .description
                .exclusion(&row.selection)
                .unwrap()
                .is_none()
        );
        assert_eq!(
            fixture.description.selection(&row.key).unwrap(),
            row.selection
        );
        assert!(keys.insert(&row.key));

        let anchor = pairs
            .first()
            .map(|pair| pair.as_slice())
            .or_else(|| triples.first().map(|triple| triple.as_slice()))
            .expect("у строки есть непокрытая цель");
        assert!(
            anchor
                .iter()
                .all(|(id, level)| row.selection.get(id) == Some(level))
        );
        remove_covered(&mut pairs, &row.selection);
        remove_covered(&mut triples, &row.selection);
    }
    assert_eq!(plan.pairs.unplanned(), &pairs);
    assert_eq!(plan.safety_triples.unplanned(), &triples);
}

#[test]
fn план_блока_покрывает_ручной_реестр_и_воспроизводится() {
    let fixture: Fixture = toml::from_str(BLOCK).unwrap();
    let plan = plan_cover(&fixture.description, 1000).unwrap();
    assert!(plan.is_complete());
    check_rows(&plan, &fixture);
    // Таблица блока: C(5,2)·4 + 5·2 − 2 пары, C(4,3)·8 − 1 тройка.
    assert_eq!(plan.pairs.planned(), 48);
    assert_eq!(plan.safety_triples.planned(), 31);
    // Три обязательные тройки по двоичным уровням имеют по восемь комбинаций,
    // а после ограничений из 2^5 полных строк допустимы 32 − 12 − 3 = 17.
    assert!((8..17).contains(&plan.rows.len()));
    let bytes = toml::to_string(&plan).unwrap();
    let repeated = plan_cover(&fixture.description, 1000).unwrap();
    assert_eq!(toml::to_string(&repeated).unwrap(), bytes);
}

#[test]
fn квота_сохраняет_префикс_и_точный_остаток() {
    let fixture: Fixture = toml::from_str(BLOCK).unwrap();
    let plan = plan_cover(&fixture.description, 1000).unwrap();
    for cap in [0, 1, plan.rows.len() - 1, plan.rows.len()] {
        let prefix = plan_cover(&fixture.description, cap).unwrap();
        assert_eq!(prefix.rows, plan.rows[..cap]);
        assert_eq!(prefix.is_complete(), cap == plan.rows.len());
        check_rows(&prefix, &fixture);
    }
    let first = plan_cover(&fixture.description, 1).unwrap();
    // Полная строка блока покрывает C(6,2) = 15 пар и C(4,3) = 4 тройки.
    assert_eq!(first.pairs.planned(), 15);
    assert_eq!(first.safety_triples.planned(), 4);
    assert_eq!(first.pairs.unplanned().len(), 33);
    assert_eq!(first.safety_triples.unplanned().len(), 27);
    assert!(matches!(
        plan_cover(&fixture.description, 1001),
        Err(CoverError::RowLimit { requested: 1001 })
    ));
}

#[test]
fn стержень_с_одной_допустимой_строкой_и_пустой_огибающей_троек() {
    let fixture: Fixture = toml::from_str(ROD).unwrap();
    let plan = plan_cover(&fixture.description, 1).unwrap();
    assert!(plan.is_complete());
    assert_eq!(plan.rows.len(), 1);
    assert_eq!(plan.pairs.planned(), 3); // C(3,2) для единственной строки.
    assert_eq!(plan.safety_triples.planned(), 0); // Помечены только два фактора.
    check_rows(&plan, &fixture);

    let excluded = format!(
        r#"{ROD}
[description.constraints.free]
when = {{ mode = "free" }}
reason = "обе позиции удерживаются упором"
source = {{ kind = "derived", reference = "отрицательная постановка стержня" }}
"#
    );
    let excluded: Fixture = toml::from_str(&excluded).unwrap();
    assert!(matches!(
        plan_cover(&excluded.description, 1),
        Err(CoverError::NoAdmissibleRows)
    ));
}

#[test]
fn числовые_уровни_и_исключения_валидируются_до_планирования() {
    for invalid in ["nan", "inf", "-inf"] {
        assert!(
            toml::from_str::<Fixture>(&BLOCK.replace("heavy = 2.0", &format!("heavy = {invalid}")))
                .is_err()
        );
    }
    let unsupported = BLOCK.replace(
        "kind = \"derived\", reference = \"таблица условий",
        "kind = \"estimate\", reference = \"таблица условий",
    );
    assert!(toml::from_str::<Fixture>(&unsupported).is_err());
}

#[test]
fn множество_уровней_покрывается_без_перебора_всех_строк() {
    // Массы 24 независимых стержней имеют по два уровня. Остальные три фактора
    // имеют ровно по одному допустимому уровню, всего 2^24 полных строк.
    let mut description = ROD.to_owned();
    for i in 0..24 {
        description.push_str(&format!(
            r#"
[description.factors.mass_{i}]
safety = {safety}
source = {{ kind = "derived", reference = "синтетические уровни 1 и 2 кг" }}
levels = {{ kind = "axis", parameter = {{ kind = "mass", body = "block/rod_{i}" }}, unit = "kg", nominal = 1.0, values = {{ light = 1.0, heavy = 2.0 }} }}
"#,
            safety = i == 0,
        ));
    }
    let fixture: Fixture = toml::from_str(&description).unwrap();
    let plan = plan_cover(&fixture.description, 1000).unwrap();
    assert!(plan.is_complete());
    // C(24,2)·4 + 24·3·2 + C(3,2) = 1251. Единственная группа безопасности
    // включает режим, путь и одну массу, поэтому требуется две тройки.
    assert_eq!(plan.pairs.planned(), 1251);
    assert_eq!(plan.safety_triples.planned(), 2);
    let mut pairs = plan.pairs.required().clone();
    let mut triples = plan.safety_triples.required().clone();
    for row in &plan.rows {
        assert_eq!(row.selection["mode"], "free");
        remove_covered(&mut pairs, &row.selection);
        remove_covered(&mut triples, &row.selection);
    }
    assert!(pairs.is_empty());
    assert!(triples.is_empty());
}
