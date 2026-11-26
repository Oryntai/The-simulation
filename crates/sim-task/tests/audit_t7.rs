//! Независимая проверка проекций конечных факторов и уточнения.

use std::collections::{BTreeMap, BTreeSet};

use sim_task::{
    Fingerprint, Outcome, Signal, Source, SourceKind,
    campaign::{RunKey, Window, depth_factors},
    verdict::LimitValue,
};

use sim_task::campaign::depth_cover;
use sim_task::campaign::depth_search;

use depth_factors::{DepthFactors, Selection};
use depth_search::{
    Criterion, MarginEvidence, Normalization, RunWitness, SearchRun, SearchStep, refine,
};

fn fixture(name: &str) -> toml::Value {
    let text = match name {
        "block" => include_str!("../../../devices/electrical/depth_t7/fixtures/block.toml"),
        "rod" => include_str!("../../../devices/electrical/depth_t7/fixtures/rod.toml"),
        _ => panic!("неизвестный стенд"),
    };
    toml::from_str(text).unwrap()
}

type Tuple<const N: usize> = [(String, String); N];

fn table<const N: usize>(value: &toml::Value) -> BTreeSet<Tuple<N>> {
    value
        .as_array()
        .unwrap()
        .iter()
        .flat_map(|group| {
            let names: Vec<String> = group["factors"].clone().try_into().unwrap();
            assert_eq!(names.len(), N);
            group["levels"]
                .as_array()
                .unwrap()
                .iter()
                .map(move |levels| {
                    let levels: Vec<String> = levels.clone().try_into().unwrap();
                    assert_eq!(levels.len(), N);
                    std::array::from_fn(|n| (names[n].clone(), levels[n].clone()))
                })
        })
        .collect()
}

fn projections(
    factors: &DepthFactors,
    cover: &depth_cover::DepthCover,
) -> (BTreeSet<Tuple<2>>, BTreeSet<Tuple<3>>) {
    let mut pairs = BTreeSet::new();
    let mut triples = BTreeSet::new();
    for row in &cover.rows {
        assert!(factors.exclusion(&row.selection).unwrap().is_none());
        assert_eq!(factors.selection(&row.key).unwrap(), row.selection);
        let levels: Vec<_> = row.selection.clone().into_iter().collect();
        for (i, a) in levels.iter().enumerate() {
            for (j, b) in levels.iter().enumerate().skip(i + 1) {
                pairs.insert([a.clone(), b.clone()]);
                for c in levels.iter().skip(j + 1) {
                    if [a, b, c]
                        .iter()
                        .all(|item| factors.factors()[&item.0].safety)
                    {
                        triples.insert([a.clone(), b.clone(), c.clone()]);
                    }
                }
            }
        }
    }
    (pairs, triples)
}

#[test]
fn таблицы_проекций_и_остаток_после_обрезки() {
    // Те же физические условия сверены с явными таблицами из двух стендов.
    // У блока 14 разрешённых строк на основном пути и 3 на резервном,
    // у стержня разрешена только свободная позиция.
    for (name, pair_count, triple_count, admissible) in [("block", 48, 31, 17), ("rod", 3, 0, 1)] {
        let data = fixture(name);
        let factors: DepthFactors = data["description"].clone().try_into().unwrap();
        let required_pairs = table::<2>(&data["pairs"]);
        let required_triples = table::<3>(&data["triples"]);
        assert_eq!(
            (required_pairs.len(), required_triples.len()),
            (pair_count, triple_count)
        );
        let full = depth_cover::plan_cover(&factors, admissible).unwrap();
        assert!(full.is_complete());
        if name == "block" {
            assert!(
                full.rows.len() < admissible,
                "план перебрал все допустимые строки"
            );
        } else {
            assert_eq!(full.rows.len(), admissible);
        }
        assert_eq!(full.pairs.required(), &required_pairs);
        assert_eq!(full.safety_triples.required(), &required_triples);
        assert_eq!(
            projections(&factors, &full),
            (required_pairs.clone(), required_triples.clone())
        );

        // У блока одна строка даёт C(6,2) пар и C(4,3) троек.
        // У стержня есть только одна допустимая строка, обрезаем до нуля.
        let small = depth_cover::plan_cover(&factors, usize::from(name == "block")).unwrap();
        assert!(!small.is_complete());
        let (pairs, triples) = projections(&factors, &small);
        assert_eq!(small.rows, full.rows[..small.rows.len()]);
        assert_eq!(small.pairs.planned(), pairs.len());
        assert_eq!(small.safety_triples.planned(), triples.len());
        assert_eq!(
            small.pairs.unplanned(),
            &required_pairs.difference(&pairs).cloned().collect()
        );
        assert_eq!(
            small.safety_triples.unplanned(),
            &required_triples.difference(&triples).cloned().collect()
        );
    }
}

fn source() -> Source {
    Source {
        kind: SourceKind::Measured,
        reference: "Контрольные интервалы синтетического стержня".into(),
    }
}

fn criterion(name: &str, unit: &str, scale: f64) -> Criterion {
    Criterion {
        signal: Signal {
            name: name.into(),
            unit: unit.into(),
        },
        // Два такта по 1 мс, как в стенде каретки Т7.
        window: Window {
            start_ns: 0,
            end_ns: 2_000_000,
            max_gap_ns: 1_000_000,
        },
        normalization: Some(Normalization {
            scale,
            source: source(),
        }),
    }
}

fn evidence(value: f64, limit: f64) -> MarginEvidence {
    MarginEvidence {
        observed: Some([value; 2]),
        limit: Some(LimitValue {
            lower: None,
            upper: Some([limit; 2]),
        }),
        limit_source: Some(source()),
        outcome: if value > limit {
            Outcome::Fail
        } else {
            Outcome::Pass
        },
    }
}

#[test]
fn запас_и_соседство_сверены_по_физическим_уровням() {
    let data = fixture("block");
    let mut description = data["description"].clone();
    // Средний уровень 1,5 кг намеренно отсутствует в первой истории.
    description["factors"]["mass"]["levels"]["values"]
        .as_table_mut()
        .unwrap()
        .insert("mid".into(), toml::Value::Float(1.5));
    let factors: DepthFactors = description.try_into().unwrap();
    let criteria = BTreeMap::from([
        ("ход".into(), criterion("ход", "m", 1.0)),
        ("сила".into(), criterion("сила", "N", 100.0)),
    ]);
    let row = |mass: &str, travel, force| {
        let selection: Selection = [
            ("cut", "intact"),
            ("grid", "base"),
            ("mass", mass),
            ("mode", "motion"),
            ("path", "main"),
            ("wind", "still"),
        ]
        .into_iter()
        .map(|(id, level)| (id.into(), level.into()))
        .collect();
        let key = factors.run_key(&selection).unwrap();
        let input = Fingerprint::of(&serde_key(&key));
        SearchRun {
            key,
            witness: Some(RunWitness {
                revision: Fingerprint::of(b"block"),
                input,
                journal: Fingerprint::of(&input.as_u64().to_le_bytes()),
                seed: 17, // Зерно нейтрального стенда crates/sim-cli/tests/fixtures/rotary/world.toml.
                horizon_ns: 2_000_000,
                injections_confirmed: true,
            }),
            evidence: BTreeMap::from([
                ("ход".into(), evidence(travel, 2.0)),
                ("сила".into(), evidence(force, 100.0)),
            ]),
        }
    };
    // По x = m + 0,5 м порог при 1,5 кг. Ход тяжёлого уровня даёт −0,5,
    // сила лёгкого уровня даёт (100 − 125)/100 = −0,25 при большем сыром отклонении.
    let light = row("light", 1.5, 125.0);
    let heavy = row("heavy", 2.5, 75.0);
    let mid = row("mid", 2.0, 100.0);
    let mut history = vec![light, heavy.clone()];
    let exhausted = refine(&factors, &criteria, &history, 0).unwrap();
    let worst = exhausted.worst.unwrap();
    assert_eq!(
        (worst.key, worst.criterion.as_str(), worst.normalized),
        (heavy.key.clone(), "ход", -0.5)
    );
    assert!(exhausted.boundaries.is_empty());
    assert_eq!(exhausted.step, SearchStep::BudgetExhausted);
    let pending = refine(&factors, &criteria, &history, 1).unwrap();
    let SearchStep::Probe(probe) = pending.step else {
        panic!("пропущен соседний уровень")
    };
    assert_eq!(probe.key, mid.key);
    let mut interrupted = mid.clone();
    interrupted.witness.as_mut().unwrap().injections_confirmed = false;
    history.push(interrupted);
    let open = refine(&factors, &criteria, &history, 1).unwrap();
    assert!(open.boundaries.is_empty());
    assert!(open.not_analyzed.iter().any(|entry| entry.key == mid.key));

    *history.last_mut().unwrap() = mid.clone();
    let closed = refine(&factors, &criteria, &history, 1).unwrap();
    assert!(
        closed
            .boundaries
            .iter()
            .any(|b| b.lower == mid.key && b.upper == heavy.key && b.criterion == "ход")
    );
    history[2].witness.as_mut().unwrap().revision = Fingerprint::of(b"rod");
    assert!(refine(&factors, &criteria, &history, 1).is_err());
}

fn serde_key(key: &RunKey) -> Vec<u8> {
    toml::to_string(key).unwrap().into_bytes()
}

mod support;

#[test]
fn парная_ветвь_требует_свою_запись_и_ту_же_схему_сигналов() {
    use sim_signal::Time;
    use sim_task::RunKind;
    use support::{HORIZON, TICK, Trial, assessor, inputs, install};

    // Предел 1 мм и срок два шага из синтетической каретки assessment.rs.
    let assessor = assessor(0.001, 2 * TICK, false);
    let case = &assessor.required_cases()[0];
    let trial = |archive_fails, different_header| {
        let mut journals = Vec::new();
        let result = assessor
            .execute_observed_with_comparison(
                case,
                true,
                |runtime, plan, events| {
                    install(runtime, plan, events, Trial::Normal)?;
                    if plan.kind() == RunKind::Comparison && different_header {
                        runtime.register_truth("другой канал", "1")?;
                    }
                    Ok(inputs())
                },
                |kind, runtime, _| {
                    let log = runtime.log();
                    assert_eq!(log.end().time, Time(HORIZON));
                    if kind == RunKind::Comparison && archive_fails {
                        return Err("запись парной ветви недоступна".into());
                    }
                    // Отпечаток сверяем после бинарной выгрузки и чтения.
                    let mut bytes = Vec::new();
                    log.write_binary(&mut bytes).unwrap();
                    let record = sim_runtime::Log::read_binary(&bytes[..]).unwrap();
                    journals.push((kind, record.fingerprint()));
                    Ok(())
                },
            )
            .unwrap();
        (result, journals)
    };
    let (good, journals) = trial(false, false);
    let good = good.report();
    assert_eq!(journals.len(), 2);
    assert_eq!(good.outcome, Outcome::Pass);
    let comparison = good.comparison_execution.as_ref().unwrap();
    assert!(comparison.complete);
    assert_eq!(good.journal, Some(journals[0].1));
    assert_eq!(comparison.journal, journals[1].1);
    assert_ne!(journals[0].1, journals[1].1);

    for (archive_fails, different_header, missing) in [
        (true, false, "журнал парного прогона не сохранён"),
        (false, true, "другим набором сигналов"),
    ] {
        let (result, journals) = trial(archive_fails, different_header);
        let report = result.report();
        assert!(report.completed && report.comparison_started);
        assert_eq!(journals.len(), if archive_fails { 1 } else { 2 });
        assert_eq!(report.outcome, Outcome::NotAnalyzed);
        assert!(report.missing.iter().any(|reason| reason.contains(missing)));
        assert!(
            !report.comparison_execution.as_ref().unwrap().complete,
            "{missing}"
        );
    }
}

#[test]
fn старый_флаг_отказа_сохраняет_тройки_и_запрет_проекций() {
    let data = fixture("block");
    let mut description = data["description"].clone();
    description["factors"]["cut"]["safety"] = toml::Value::Boolean(false);
    let factors: DepthFactors = description.try_into().unwrap();
    // Ручные таблицы блока перечисляют все 48 пар и 31 разрешённую тройку.
    let expected_pairs = table::<2>(&data["pairs"]);
    let expected_triples = table::<3>(&data["triples"]);
    assert_eq!((expected_pairs.len(), expected_triples.len()), (48, 31));
    assert!(
        expected_triples
            .iter()
            .any(|tuple| tuple.iter().any(|(id, _)| id == "cut"))
    );
    let full = depth_cover::plan_cover(&factors, 17).unwrap();
    assert_eq!(
        projections(&factors, &full),
        (expected_pairs.clone(), expected_triples.clone())
    );
    let no_runs = depth_cover::plan_cover(&factors, 0).unwrap();
    assert_eq!(no_runs.pairs.unplanned(), &expected_pairs);
    assert_eq!(no_runs.safety_triples.unplanned(), &expected_triples);
}

#[test]
fn запас_из_журнала_сохраняет_отказ_и_проверяет_границы() {
    use depth_search::{Margin, normalized_margin};
    use sim_signal::{Command, Time};
    use sim_task::{Limit, campaign::OperatingMode, metrics::scenario_margins};
    use support::{PART, TICK};

    let world = support::world(PART);
    let effort = world.efforts[0].name.clone();
    let mut runtime = sim_runtime::Runtime::load(world).unwrap();
    runtime.record_joint_state("положение каретки").unwrap();
    runtime
        .act(&Command {
            targets: vec![(runtime.channel(&effort).unwrap(), 1.0)],
        })
        .unwrap();
    // В опоре support масса 1 кг. При силе 1 Н полунеявный Euler даёт
    // x(2h) = 3h² = 3 мкм. Предел 11/6 мкм даёт запас (11−18)/11.
    let limit = 11e-6 / 6.0;
    let mode: OperatingMode = toml::from_str(&format!(r#"
horizon_ns = {horizon}
source = {{ kind = "derived", reference = "Два шага аналитической каретки" }}
criteria = [{{ id = "ход", window = "whole", limit = {{ kind = "range", signal = {{ name = "qpos[0]", unit = "m" }}, max = {limit}, source = {{ kind = "measured", reference = "Верхняя граница оснастки 11/6 мкм" }} }} }}]
[windows.whole]
start_ns = 0
end_ns = {horizon}
max_gap_ns = {TICK}
"#, horizon = 2 * TICK)).unwrap();
    runtime.run_until(Time(TICK)).unwrap();
    let partial = scenario_margins(&mode, runtime.log()).unwrap();
    let (spec, observed) = &partial["ход"];
    assert_eq!(observed.outcome, Outcome::NotAnalyzed);
    assert!(matches!(
        normalized_margin(observed, spec.normalization.as_ref()).unwrap(),
        Margin::NotAnalyzed { .. }
    ));

    runtime.run_until(Time(2 * TICK)).unwrap();
    let full = scenario_margins(&mode, runtime.log()).unwrap();
    let (spec, observed) = &full["ход"];
    let [low, high] = observed.observed.unwrap();
    // Запас в 16 машинных эпсилонов покрывает округление нескольких операций.
    let tolerance = 16.0 * f64::EPSILON;
    assert_eq!(low, 0.0);
    assert!((high / 3e-6 - 1.0).abs() < tolerance);
    let Margin::Scored {
        normalized,
        outcome,
    } = normalized_margin(observed, spec.normalization.as_ref()).unwrap()
    else {
        panic!("не рассчитан запас из полного ряда");
    };
    assert_eq!(outcome, Outcome::Fail);
    assert!((normalized - (-7.0 / 11.0)).abs() < tolerance);

    for invalid in [
        None,
        Some(f64::NAN),
        Some(f64::INFINITY),
        Some(f64::NEG_INFINITY),
    ] {
        let mut mode = mode.clone();
        let Limit::Range { max, .. } = &mut mode.criteria[0].limit else {
            unreachable!()
        };
        *max = invalid;
        assert!(
            scenario_margins(&mode, runtime.log()).is_err(),
            "{invalid:?}"
        );
    }
}

#[test]
fn без_записи_основная_ветвь_покрытия_остаётся_открытой() {
    use support::{TICK, Trial, assessor, inputs, install};

    // Порог 1 мм и срок два шага из синтетической каретки assessment.rs.
    let assessor = assessor(0.001, 2 * TICK, false);
    let case = &assessor.required_cases()[0];
    let outcome = assessor
        .execute_observed_with_comparison(
            case,
            true,
            |runtime, plan, events| {
                install(runtime, plan, events, Trial::Normal)?;
                Ok(inputs())
            },
            |_, _, _| Err("запись основной ветви недоступна".into()),
        )
        .unwrap();
    let report = outcome.report();
    assert_eq!(report.outcome, Outcome::NotAnalyzed);
    assert!(report.reached_ns.is_some());
    assert!(!report.was_executed());
    assert!(!report.comparison_started);
}
