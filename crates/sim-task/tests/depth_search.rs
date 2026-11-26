use std::collections::BTreeMap;

use sim_task::{
    Fingerprint, Outcome, Signal, Source, SourceKind,
    campaign::{Window, depth_factors},
    verdict::LimitValue,
};

use sim_task::campaign::depth_search;

use depth_factors::{DepthFactors, Selection};
use depth_search::{
    Criterion, Margin, MarginEvidence, Normalization, RunWitness, SearchRun, SearchStep,
    normalized_margin, refine,
};

const FACTORS: &str = r#"
[factors.operation]
safety = true
source = { kind = "derived", reference = "режимы синтетического стенда" }
levels = { kind = "mode", values = { hold = { station = "bench", mode = "hold" }, release = { station = "bench", mode = "release" } } }
[factors.path]
safety = false
source = { kind = "derived", reference = "маршрут блока" }
levels = { kind = "path", values = { primary = "line" } }
[factors.clock]
safety = false
source = { kind = "derived", reference = "сетка стенда" }
levels = { kind = "grid", values = { base = "base" } }
[factors.cut]
safety = true
source = { kind = "derived", reference = "отключение линии блока" }
levels = { kind = "faults", values = { intact = [], open = [{ node = "line", kind = "wire_open" }] } }
[factors.mass]
safety = true
source = { kind = "measured", reference = "синтетические гири от 1 до 5 кг с шагом 1 кг" }
levels = { kind = "axis", parameter = { kind = "mass", body = "block/rod" }, unit = "kg", nominal = 2.0, values = { z_light = 1.0, base = 2.0, mid = 3.0, a_heavy = 4.0, b_top = 5.0 } }
[factors.wind]
safety = false
source = { kind = "derived", reference = "заданные входы 0 и 3 м/с" }
levels = { kind = "axis", parameter = { kind = "wind", axis = 0 }, unit = "m/s", nominal = 0.0, values = { still = 0.0, gust = 3.0 } }
[constraints.hold_stop]
when = { operation = "hold", mass = "b_top" }
reason = "стенд удержания рассчитан на набор гирь до 4 кг"
source = { kind = "measured", reference = "синтетическая ведомость оснастки" }
"#;

fn source() -> Source {
    Source {
        kind: SourceKind::Derived,
        reference: "формулы и пределы синтетического стенда блока".into(),
    }
}

fn criteria() -> BTreeMap<String, Criterion> {
    // Две характеристики одного стенда. Масштабы заданы в единицах критерия.
    [("travel", "m", 4.0), ("force", "N", 40.0)]
        .into_iter()
        .map(|(id, unit, scale)| {
            (
                id.into(),
                Criterion {
                    signal: Signal {
                        name: id.into(),
                        unit: unit.into(),
                    },
                    // Окно стенда [0, 2] мс на сетке 1 кГц.
                    window: Window {
                        start_ns: 0,
                        end_ns: 2_000_000,
                        max_gap_ns: 1_000_000,
                    },
                    normalization: Some(Normalization {
                        scale,
                        source: source(),
                    }),
                },
            )
        })
        .collect()
}

fn evidence(value: f64, upper: f64, outcome: Outcome) -> MarginEvidence {
    MarginEvidence {
        observed: Some([value; 2]),
        limit: Some(LimitValue {
            lower: None,
            upper: Some([upper; 2]),
        }),
        limit_source: Some(source()),
        outcome,
    }
}

fn selection(mass: &str, wind: &str, operation: &str, cut: &str) -> Selection {
    [
        ("mass", mass),
        ("wind", wind),
        ("operation", operation),
        ("cut", cut),
        ("path", "primary"),
        ("clock", "base"),
    ]
    .into_iter()
    .map(|(factor, level)| (factor.into(), level.into()))
    .collect()
}

fn run(factors: &DepthFactors, selection: &Selection, value: f64, outcome: Outcome) -> SearchRun {
    let key = factors.run_key(selection).unwrap();
    let input = Fingerprint::of(toml::to_string(&key).unwrap().as_bytes());
    SearchRun {
        key,
        witness: Some(RunWitness {
            revision: Fingerprint::of(FACTORS.as_bytes()),
            input,
            journal: input,
            // Исходное зерно и горизонт фикстуры конечного поиска.
            seed: 37,
            horizon_ns: 2_000_000,
            injections_confirmed: true,
        }),
        evidence: BTreeMap::from([("travel".into(), evidence(value, 5.0, outcome))]),
    }
}

#[test]
fn запас_по_обе_стороны_порога_и_на_пороге() {
    let criteria = criteria();
    let norm = criteria["travel"].normalization.as_ref();
    // Синтетический ход x = 2(m − 1) м, ограничение x ≤ 5 м, шкала 4 м.
    // Для m = 3 и 4 кг ручные значения (5 − 4)/4 = 1/4 и (5 − 6)/4 = −1/4.
    // Независимо: порог массы 3,5 кг, запас (3,5 − m)/2 кг.
    for (mass, observed, expected, outcome) in [
        (3.0, 4.0, 0.25, Outcome::Pass),
        (3.5, 5.0, 0.0, Outcome::Pass),
        (4.0, 6.0, -0.25, Outcome::Fail),
    ] {
        assert_eq!(expected, (3.5 - mass) / 2.0);
        assert_eq!(
            normalized_margin(&evidence(observed, 5.0, outcome), norm).unwrap(),
            Margin::Scored {
                normalized: expected,
                outcome
            }
        );
    }

    let mut both = evidence(0.0, 8.0, Outcome::Pass);
    both.observed = Some([2.0, 6.0]);
    both.limit = Some(LimitValue {
        lower: Some([0.0, 1.0]),
        upper: Some([8.0, 9.0]),
    });
    // Гарантированный коридор [1, 8]. При расширении ряда [2, 6] на 1 м
    // нижний край касается коридора, верхний оставляет 1 м. Запас 1/4.
    assert_eq!(
        normalized_margin(&both, norm).unwrap(),
        Margin::Scored {
            normalized: 0.25,
            outcome: Outcome::Pass
        }
    );
    both.limit.as_mut().unwrap().upper = None;
    both.observed = Some([0.0, 0.0]);
    both.outcome = Outcome::Fail;
    // Односторонний нижний предел: 0 м − 1 м = −1 м, в единицах шкалы −1/4.
    assert_eq!(
        normalized_margin(&both, norm).unwrap(),
        Margin::Scored {
            normalized: -0.25,
            outcome: Outcome::Fail
        }
    );
}

#[test]
fn открытые_данные_и_неконечные_входы() {
    let criteria = criteria();
    let norm = criteria["travel"].normalization.as_ref();
    let valid = evidence(4.0, 5.0, Outcome::Pass);
    for open in [
        MarginEvidence {
            observed: None,
            ..valid.clone()
        },
        MarginEvidence {
            limit: None,
            ..valid.clone()
        },
        MarginEvidence {
            limit_source: None,
            ..valid.clone()
        },
        MarginEvidence {
            limit_source: Some(Source {
                kind: SourceKind::Estimate,
                ..source()
            }),
            ..valid.clone()
        },
        MarginEvidence {
            limit_source: Some(Source {
                reference: " ".into(),
                ..source()
            }),
            ..valid.clone()
        },
        MarginEvidence {
            outcome: Outcome::NotAnalyzed,
            ..valid.clone()
        },
        evidence(6.0, 5.0, Outcome::Pass),
    ] {
        assert!(matches!(
            normalized_margin(&open, norm).unwrap(),
            Margin::NotAnalyzed { .. }
        ));
    }
    assert!(matches!(
        normalized_margin(&valid, None).unwrap(),
        Margin::NotAnalyzed { .. }
    ));
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(normalized_margin(&evidence(value, 5.0, Outcome::NotAnalyzed), None).is_err());
        assert!(normalized_margin(&evidence(4.0, value, Outcome::NotAnalyzed), None).is_err());
        assert!(
            normalized_margin(
                &valid,
                Some(&Normalization {
                    scale: value,
                    source: source()
                })
            )
            .is_err()
        );
    }
    for scale in [0.0, -1.0] {
        assert!(
            normalized_margin(
                &valid,
                Some(&Normalization {
                    scale,
                    source: source()
                })
            )
            .is_err()
        );
    }
    assert!(
        normalized_margin(
            &MarginEvidence {
                observed: Some([6.0, 4.0]),
                ..valid
            },
            norm
        )
        .is_err()
    );
}

#[test]
fn уточнение_следует_худшему_запасу_и_фиксирует_границу() {
    let factors: DepthFactors = toml::from_str(FACTORS).unwrap();
    let criteria = criteria();
    let selected = |level| selection(level, "still", "release", "intact");
    let mut base = run(&factors, &selected("base"), 2.0, Outcome::Pass);
    // Запас силы (50 − 48)/40 = 0,05, запас хода для mid (5 − 4)/4 = 0,25.
    base.evidence
        .insert("force".into(), evidence(48.0, 50.0, Outcome::Pass));
    let mid = run(&factors, &selected("mid"), 4.0, Outcome::Pass);
    let first = refine(&factors, &criteria, &[base.clone(), mid.clone()], 1).unwrap();
    assert_eq!(first.worst.as_ref().unwrap().criterion, "force");
    assert_eq!(first.worst.as_ref().unwrap().normalized, 0.05);
    let SearchStep::Probe(probe) = first.step else {
        panic!("ожидалось уточнение")
    };
    assert_eq!(probe.anchor.criterion, "force");

    let heavy = run(&factors, &selected("a_heavy"), 6.0, Outcome::Fail);
    let mut runs = vec![base, mid.clone(), heavy.clone()];
    let found = refine(&factors, &criteria, &runs, 1).unwrap();
    let worst = found.worst.as_ref().unwrap();
    assert_eq!(
        (&worst.key, worst.normalized, worst.outcome),
        (&heavy.key, -0.25, Outcome::Fail)
    );
    assert_eq!(worst.witness.input, heavy.witness.unwrap().input);
    assert_eq!(worst.window, criteria["travel"].window);
    assert_eq!(found.boundaries.len(), 1);
    let edge = &found.boundaries[0];
    assert_eq!((&edge.lower, &edge.upper), (&mid.key, &heavy.key));
    assert_eq!((&*edge.criterion, &*edge.factor), ("travel", "mass"));
    let zero = refine(&factors, &criteria, &runs, 0).unwrap();
    assert_eq!(zero.step, SearchStep::BudgetExhausted);
    assert_eq!(zero.worst, found.worst);
    assert_eq!(zero.boundaries, found.boundaries);
    runs.reverse();
    assert_eq!(refine(&factors, &criteria, &runs, 1).unwrap(), found);
    let SearchStep::Probe(probe) = found.step else {
        panic!("ожидалось уточнение")
    };
    assert_eq!(probe.selection, selected("b_top"));
    assert_eq!(probe.key, factors.run_key(&probe.selection).unwrap());
    assert_eq!(factors.selection(&probe.key).unwrap(), probe.selection);

    let top = run(&factors, &probe.selection, 8.0, Outcome::Fail);
    runs.push(top.clone());
    let updated = refine(&factors, &criteria, &runs, 0).unwrap();
    let worst = updated.worst.unwrap();
    // Ход при 5 кг равен 8 м. По порогу массы (3,5 − 5)/2 = −3/4.
    assert_eq!((worst.key, worst.normalized), (top.key, -0.75));
    assert_eq!(updated.boundaries, found.boundaries);
}

#[test]
fn контроль_соседства_и_совместимости() {
    let factors: DepthFactors = toml::from_str(FACTORS).unwrap();
    let criteria = criteria();
    let selected = |level| selection(level, "still", "hold", "intact");
    let heavy = run(&factors, &selected("a_heavy"), 6.0, Outcome::Fail);
    let base = run(&factors, &selected("base"), 2.0, Outcome::Pass);
    let other_mode = run(
        &factors,
        &selection("mid", "still", "release", "intact"),
        4.0,
        Outcome::Pass,
    );
    let other_fault = run(
        &factors,
        &selection("mid", "still", "hold", "open"),
        4.0,
        Outcome::Pass,
    );
    let runs = [heavy.clone(), base, other_mode, other_fault];
    let result = refine(&factors, &criteria, &runs, 1).unwrap();
    assert!(result.boundaries.is_empty());
    let SearchStep::Probe(probe) = result.step else {
        panic!("ожидалось уточнение")
    };
    // Между уровнями 2 и 4 кг лежат 3 кг. Удержание 5 кг исключено.
    assert_eq!(probe.selection, selected("mid"));

    let mut blocked = run(&factors, &selected("mid"), 4.0, Outcome::Pass);
    blocked.witness = None;
    let mut wind = run(
        &factors,
        &selection("a_heavy", "gust", "hold", "intact"),
        4.0,
        Outcome::NotAnalyzed,
    );
    wind.witness.as_mut().unwrap().injections_confirmed = false;
    let result = refine(&factors, &criteria, &[heavy, blocked, wind], 1).unwrap();
    assert!(result.boundaries.is_empty());
    assert_eq!(result.step, SearchStep::NoUntriedNeighbors);
    assert_eq!(result.not_analyzed.len(), 2);
}

#[test]
fn неполные_опыты_не_образуют_границу_и_не_задают_направление() {
    let factors: DepthFactors = toml::from_str(FACTORS).unwrap();
    let criteria = criteria();
    let mut interrupted = run(
        &factors,
        &selection("base", "still", "hold", "open"),
        8.0,
        Outcome::Fail,
    );
    interrupted.witness.as_mut().unwrap().injections_confirmed = false;
    let mut short = run(
        &factors,
        &selection("mid", "still", "hold", "intact"),
        8.0,
        Outcome::Fail,
    );
    // Первый отсчёт есть, правый край окна 2 мс ещё не записан.
    short.witness.as_mut().unwrap().horizon_ns = 1_000_000;
    let result = refine(&factors, &criteria, &[interrupted, short], 1).unwrap();
    assert_eq!(result.step, SearchStep::NotAnalyzed);
    assert_eq!(result.worst, None);
    assert!(result.boundaries.is_empty());
    assert_eq!(result.not_analyzed.len(), 2);
    assert!(
        result
            .not_analyzed
            .iter()
            .any(|open| open.reason.contains("инъекций"))
    );
    assert!(
        result
            .not_analyzed
            .iter()
            .any(|open| open.reason.contains("конца окна"))
    );
}

#[test]
fn чужая_ревизия_дубликат_и_запрещённый_ключ_дают_ошибку() {
    let factors: DepthFactors = toml::from_str(FACTORS).unwrap();
    let criteria = criteria();
    let base = run(
        &factors,
        &selection("base", "still", "hold", "intact"),
        2.0,
        Outcome::Pass,
    );
    assert!(refine(&factors, &criteria, &[base.clone(), base.clone()], 0).is_err());
    let excluded = run(
        &factors,
        &selection("b_top", "still", "hold", "intact"),
        8.0,
        Outcome::Fail,
    );
    assert!(refine(&factors, &criteria, &[excluded], 1).is_err());
    let mut unrelated = run(
        &factors,
        &selection("mid", "still", "hold", "intact"),
        4.0,
        Outcome::Pass,
    );
    unrelated.witness.as_mut().unwrap().revision = Fingerprint::of(b"rod");
    assert!(refine(&factors, &criteria, &[base, unrelated], 1).is_err());
}
