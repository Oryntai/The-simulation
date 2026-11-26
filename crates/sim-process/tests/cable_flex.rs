use std::{collections::BTreeMap, f64::consts::FRAC_PI_2};

use sim_process::{
    Analysis, Interval, ProcessError, SourcedRange,
    schematic::{Mode, Scope, Section, Status},
    work::cable_flex::*,
};
use sim_signal::Time;
use sim_world::Provenance;

// Стенд «каретка»: две прямые 80 мм и 60…120 мм, угол 90°. Круговой
// направляющий участок заменяет r с каждой прямой на четверть окружности.
// Максимум длины 200 − 2r + πr/2 мм. Радиусы 9 и 11 мм при пределе 10 мм
// дают запасы −1 и +1 мм, длины 196,137166941 и 195,278759595 мм.
// При доступных 197 мм запасы длины +0,862833059 и +1,721240405 мм.
// Для независимой сверки ниже суммируются прямые и хорды четверти окружности.
// Один ход за секунду, 298…302 К и 10 000 циклов — условия численного
// стенда. Предельные радиусы взяты из его постановки и не относятся к изделию.
const SOURCE: &str = "Аналитическая постановка стенда каретки";
const START: Time = Time::ZERO;
const MID: Time = Time(500_000_000);
const END: Time = Time(1_000_000_000);
const WINDOW: Interval = Interval {
    start: START,
    end: END,
};
const ERROR: f64 = 1e-12; // Допуск вычислений для размеров стенда в СИ.
const LENGTH: f64 = 0.197;
const LIMIT: f64 = 0.010;
const CYCLES: u64 = 10_000;

fn sourced(min: f64, max: f64) -> SourcedRange {
    SourcedRange {
        min,
        nominal: min,
        max,
        provenance: Provenance::Derived(SOURCE.into()),
    }
}

fn exact(value: f64) -> SourcedRange {
    sourced(value, value)
}

fn pose(y: f64) -> BTreeMap<String, [f64; 3]> {
    [
        ("блок".into(), [0.0, 0.0, 0.0]),
        ("направляющая".into(), [0.080, 0.0, 0.0]),
        ("каретка".into(), [0.080, y, 0.0]),
    ]
    .into()
}

struct Bench {
    branch: Section,
    source: Provenance,
    start: BTreeMap<String, [f64; 3]>,
    mid: BTreeMap<String, [f64; 3]>,
    end: BTreeMap<String, [f64; 3]>,
    error: SourcedRange,
    length: SourcedRange,
    limit: SourcedRange,
}

impl Bench {
    fn new() -> Self {
        Self {
            branch: Section::Connection("гибкая ветвь".into()),
            source: Provenance::Derived(SOURCE.into()),
            start: pose(0.060),
            mid: pose(0.090),
            end: pose(0.120),
            error: exact(ERROR),
            length: exact(LENGTH),
            limit: exact(LIMIT),
        }
    }

    fn steps(&self) -> [CableStep<'_>; 2] {
        [(MID, &self.mid), (END, &self.end)].map(|(at, sites_m)| CableStep {
            end: CablePose { at, sites_m },
            length_error_m: Some(&self.error),
            curvature_error_inv_m: Some(&self.error),
        })
    }

    fn input<'a>(
        &'a self,
        guides: &'a [CableGuide<'a>],
        steps: &'a [CableStep<'a>],
    ) -> CableFlexInput<'a> {
        CableFlexInput {
            scope: Scope::Reference,
            mode: &Mode::Normal,
            branch: &self.branch,
            window: WINDOW,
            mount: Some(CableMount {
                ends: ["блок", "каретка"].map(|site| CableEnd {
                    site,
                    relief: Some(CableRelief {
                        installed: true,
                        source: &self.source,
                    }),
                }),
                guides,
                applicability: Some(&self.source),
            }),
            motion: Some(CableMotion {
                start: CablePose {
                    at: START,
                    sites_m: &self.start,
                },
                steps,
            }),
            duty: None,
        }
    }

    fn ratings(&self) -> CableFlexRatings<'_> {
        CableFlexRatings {
            usable_length_m: Some(&self.length),
            min_bend_radius_m: Some(&self.limit),
            curve: None,
        }
    }
}

fn guide(setback_m: f64) -> [CableGuide<'static>; 1] {
    [CableGuide {
        site: "направляющая",
        setback_m,
    }]
}

fn value<T>(analysis: &Analysis<T>) -> &T {
    match analysis {
        Analysis::Analyzed { value, .. } => value,
        Analysis::NotAnalyzed { reason } => panic!("нет расчёта: {reason}"),
    }
}

fn near(actual: f64, expected: f64, error: f64) {
    assert!(
        (actual - expected).abs() <= error,
        "{actual} вместо {expected}"
    );
}

fn independent_quarter_circle(setback: f64) -> (f64, f64) {
    // Строим обе точки касания по шаблону. Нормаль к первой прямой
    // вертикальна, ко второй горизонтальна. Их пересечение даёт центр.
    let incoming = [0.080 - setback, 0.0];
    let outgoing = [0.080, setback];
    let center = [incoming[0], outgoing[1]];
    let radius = (incoming[0] - center[0]).hypot(incoming[1] - center[1]);
    let arc = |angle: f64| {
        [
            center[0] + radius * angle.sin(),
            center[1] - radius * angle.cos(),
        ]
    };
    let sections = 1024; // Хордовая ошибка меньше 2e-9 м для обоих радиусов стенда.
    let mut length = incoming[0] + (0.120 - outgoing[1]);
    let mut previous = arc(0.0);
    for i in 1..=sections {
        let point = arc(FRAC_PI_2 * f64::from(i) / f64::from(sections));
        length += (point[0] - previous[0]).hypot(point[1] - previous[1]);
        previous = point;
    }
    (length, radius)
}

#[test]
fn опасный_и_исправленный_ход_дают_свой_код_и_ручные_запасы() {
    let bench = Bench::new();
    let steps = bench.steps();
    for (radius, expected, length) in [
        (0.009, Status::Fail, 0.196_137_166_941),
        (0.011, Status::Pass, 0.195_278_759_595),
    ] {
        let guides = guide(radius);
        let input = bench.input(&guides, &steps);
        let result = assess_cable_flex(&input, &bench.ratings()).unwrap();
        assert_eq!(result.status, expected);
        assert_eq!(result.evidence.code, "cable_flex");
        assert_eq!(result.evidence.branch, bench.branch);
        assert_eq!(result.evidence.mode, Mode::Normal);
        assert_eq!(result.evidence.window, WINDOW);
        assert_eq!(result.evidence.scope, Scope::Reference);
        assert_eq!(result.evidence.bend.status, expected);
        assert_eq!(result.evidence.slack.status, Status::Pass);
        assert_eq!(result.evidence.relief, [Status::Pass; 2]);
        assert_eq!(result.evidence.fatigue.status, Status::NotAnalyzed);
        let slack = value(&result.evidence.slack.result);
        let observed = value(&result.evidence.maximum_length_m);
        assert_eq!(observed.sample_at, END);
        near(observed.bounds.max, length, 2.0 * ERROR);
        near(slack.min, LENGTH - length, 2.0 * ERROR);
        let bend = value(&result.evidence.bend.result);
        near(bend.min_m.unwrap(), radius - LIMIT, 2.0 * ERROR);
        let (length_by_chords, radius_by_normal) = independent_quarter_circle(radius);
        near(observed.bounds.max, length_by_chords, 2e-9);
        near(
            value(&result.evidence.minimum_radius_m).min_m.unwrap(),
            radius_by_normal,
            ERROR,
        );
        assert_eq!(result, assess_cable_flex(&input, &bench.ratings()).unwrap());
    }
}

#[test]
fn обе_стороны_порогов_и_перекрытие_диапазонов() {
    let bench = Bench::new();
    let steps = bench.steps();
    for (radius, status) in [
        (0.009_999, Status::Fail),
        (0.010, Status::NotAnalyzed),
        (0.010_001, Status::Pass),
    ] {
        let guides = guide(radius);
        assert_eq!(
            assess_cable_flex(&bench.input(&guides, &steps), &bench.ratings())
                .unwrap()
                .status,
            status
        );
    }
    let guides = guide(0.011);
    let input = bench.input(&guides, &steps);
    let required = 0.195_278_759_595; // Ручная длина из постановки выше.
    for (length, status) in [
        (required - 1e-6, Status::Fail),
        (required, Status::NotAnalyzed),
        (required + 1e-6, Status::Pass),
    ] {
        let length = exact(length);
        let ratings = CableFlexRatings {
            usable_length_m: Some(&length),
            ..bench.ratings()
        };
        let result = assess_cable_flex(&input, &ratings).unwrap();
        assert_eq!(result.status, status);
        assert_eq!(result.evidence.slack.status, status);
        assert_eq!(result.evidence.bend.status, Status::Pass);
    }
}

#[test]
fn отсутствие_фиксации_отличается_от_отсутствия_данных() {
    let bench = Bench::new();
    let guides = guide(0.011);
    let steps = bench.steps();
    for (relief, status) in [
        (None, Status::NotAnalyzed),
        (
            Some(CableRelief {
                installed: false,
                source: &bench.source,
            }),
            Status::Fail,
        ),
        (
            Some(CableRelief {
                installed: true,
                source: &bench.source,
            }),
            Status::Pass,
        ),
    ] {
        let mut input = bench.input(&guides, &steps);
        input.mount.as_mut().unwrap().ends[1].relief = relief;
        let result = assess_cable_flex(&input, &bench.ratings()).unwrap();
        assert_eq!(result.status, status);
        assert_eq!(result.evidence.code, "cable_flex");
        assert_eq!(result.evidence.relief, [Status::Pass, status]);
    }
}

#[test]
fn отсчёты_не_закрывают_пики_между_тактами_и_пропуски() {
    let mut bench = Bench::new();
    let guides = guide(0.011);
    let mut steps = bench.steps();
    steps[0].curvature_error_inv_m = None;
    let mut input = bench.input(&guides, &steps);
    let finding = assess_cable_flex(&input, &bench.ratings()).unwrap();
    assert_eq!(finding.status, Status::NotAnalyzed);
    assert_eq!(finding.evidence.bend.status, Status::NotAnalyzed);
    assert_eq!(finding.evidence.slack.status, Status::Pass);
    input.motion.as_mut().unwrap().steps = &steps[..1];
    assert_eq!(
        assess_cable_flex(&input, &bench.ratings())
            .unwrap()
            .evidence
            .slack
            .status,
        Status::NotAnalyzed
    );
    let broad = exact(20.0); // До 20 м⁻¹ ошибки: верхняя граница пересекает 100 м⁻¹.
    steps[0].curvature_error_inv_m = Some(&broad);
    assert_eq!(
        assess_cable_flex(&bench.input(&guides, &steps), &bench.ratings())
            .unwrap()
            .status,
        Status::NotAnalyzed
    );
    // При вставке реально пропущенного положения поворот на 120° даёт r = 11/√3 мм.
    bench
        .mid
        .insert("каретка".into(), [0.040, 0.040 * 3.0_f64.sqrt(), 0.0]);
    let steps = bench.steps();
    let finding = assess_cable_flex(&bench.input(&guides, &steps), &bench.ratings()).unwrap();
    assert_eq!(finding.status, Status::Fail);
    near(
        value(&finding.evidence.minimum_radius_m).min_m.unwrap(),
        0.011 / 3.0_f64.sqrt(),
        ERROR,
    );
}

#[test]
fn пространственная_трасса_суммирует_дуги_и_выбирает_меньший_радиус() {
    let mut bench = Bench::new();
    // Добавляем прямую 40 мм вдоль Z и скругление 15 мм в плоскости YZ.
    // Получаются две четверти окружности, минимальный радиус 11 мм.
    for pose in [&mut bench.start, &mut bench.mid, &mut bench.end] {
        let [x, y, _] = pose["каретка"];
        pose.insert("стержень".into(), [x, y, 0.040]);
    }
    let guides = [
        guide(0.011)[0],
        CableGuide {
            site: "каретка",
            setback_m: 0.015,
        },
    ];
    let steps = bench.steps();
    let mut input = bench.input(&guides, &steps);
    input.mount.as_mut().unwrap().ends[1].site = "стержень";
    let length = exact(0.240); // Доступная длина равна длине ломаной в конце хода.
    let ratings = CableFlexRatings {
        usable_length_m: Some(&length),
        ..bench.ratings()
    };
    let finding = assess_cable_flex(&input, &ratings).unwrap();
    assert_eq!(finding.status, Status::Pass);
    near(
        value(&finding.evidence.maximum_length_m).bounds.max,
        0.240 - (2.0 - FRAC_PI_2) * 0.026,
        2.0 * ERROR,
    );
    near(
        value(&finding.evidence.minimum_radius_m).min_m.unwrap(),
        0.011,
        ERROR,
    );

    let mut overlapping = guides;
    // Общий отрезок на старте всего 60 мм, отступы 31 + 31 мм перекрываются.
    overlapping[0].setback_m = 0.031;
    overlapping[1].setback_m = 0.031;
    input.mount.as_mut().unwrap().guides = &overlapping;
    let finding = assess_cable_flex(&input, &ratings).unwrap();
    assert_eq!(finding.status, Status::NotAnalyzed);
    assert!(matches!(
        finding.evidence.minimum_radius_m,
        Analysis::NotAnalyzed { .. }
    ));
}

#[test]
fn у_прямой_нулевая_кривизна_а_потеря_метки_открывает_проверку() {
    let mut bench = Bench::new();
    bench.error = exact(0.0); // Прямая вдоль Y, максимальный ход 120 мм, все длины точны.
    let steps = bench.steps();
    let mut input = bench.input(&[], &steps);
    input.mount.as_mut().unwrap().ends[0].site = "направляющая";
    let finding = assess_cable_flex(&input, &bench.ratings()).unwrap();
    assert_eq!(finding.status, Status::Pass);
    assert_eq!(
        value(&finding.evidence.maximum_curvature_inv_m).bounds,
        CableBounds { min: 0.0, max: 0.0 }
    );
    assert_eq!(
        *value(&finding.evidence.minimum_radius_m),
        CableRadiusBounds {
            min_m: None,
            max_m: None
        }
    );
    near(
        value(&finding.evidence.maximum_length_m).bounds.max,
        0.120,
        ERROR,
    );

    bench.mid.remove("направляющая");
    let steps = bench.steps();
    let mut input = bench.input(&[], &steps);
    input.mount.as_mut().unwrap().ends[0].site = "направляющая";
    let finding = assess_cable_flex(&input, &bench.ratings()).unwrap();
    assert_eq!(finding.status, Status::NotAnalyzed);
    assert_eq!(finding.evidence.slack.status, Status::NotAnalyzed);
}

#[test]
fn оценочный_предел_и_синтетика_физической_сборки_оставляют_проверку_открытой() {
    let bench = Bench::new();
    let guides = guide(0.011);
    let steps = bench.steps();
    let mut input = bench.input(&guides, &steps);
    let mut estimate = exact(LIMIT);
    estimate.provenance = Provenance::Estimate("грубая оценка".into());
    for limit in [None, Some(&estimate)] {
        let ratings = CableFlexRatings {
            min_bend_radius_m: limit,
            ..bench.ratings()
        };
        assert_eq!(
            assess_cable_flex(&input, &ratings).unwrap().status,
            Status::NotAnalyzed
        );
    }
    input.scope = Scope::Assembly;
    assert_eq!(
        assess_cable_flex(&input, &bench.ratings()).unwrap().status,
        Status::NotAnalyzed
    );
    input.scope = Scope::Reference;
    input.mount = None;
    assert_eq!(
        assess_cable_flex(&input, &bench.ratings()).unwrap().status,
        Status::NotAnalyzed
    );
}

#[test]
fn ресурс_учитывает_циклы_ориентацию_профиль_и_всю_температуру() {
    let bench = Bench::new();
    let guides = guide(0.011);
    let steps = bench.steps();
    let mut input = bench.input(&guides, &steps);
    let temperature = sourced(298.0, 302.0);
    let limit_ok = exact(0.010_5); // Для 10 000 циклов остаётся +0,5 мм.
    let limit_early = exact(0.013); // При 20 000 циклов не хватает 2 мм.
    let mut duty = CableFlexDuty {
        cycles: CYCLES,
        orientation: "обратный изгиб",
        profile: "ход каретки",
        temperature_k: &temperature,
        // Обратная половина цикла повторяет положения прямого хода в
        // обратном порядке при тех же нагрузке, ориентации и температуре.
        applicability: &bench.source,
    };
    let cool = CableFlexBand {
        cycles: [1, CYCLES],
        orientation: duty.orientation,
        profile: duty.profile,
        temperature_k: [298.0, 300.0],
        min_bend_radius_m: Some(&limit_ok),
    };
    let warm = CableFlexBand {
        temperature_k: [300.0, 302.0],
        ..cool
    };
    let long = CableFlexBand {
        cycles: [CYCLES + 1, 2 * CYCLES],
        temperature_k: [298.0, 302.0],
        min_bend_radius_m: Some(&limit_early),
        ..cool
    };
    input.duty = Some(duty);
    let single_stroke = assess_cable_flex(&input, &bench.ratings()).unwrap();
    assert_eq!(single_stroke.status, Status::Pass);
    assert_eq!(single_stroke.evidence.fatigue.status, Status::NotAnalyzed);
    let check = |input: &CableFlexInput<'_>, bands: &[CableFlexBand<'_>], source| {
        let ratings = CableFlexRatings {
            curve: Some(CableFlexCurve { bands, source }),
            ..bench.ratings()
        };
        assess_cable_flex(input, &ratings).unwrap()
    };
    let both = [warm, cool, long]; // Порядок исходных строк не определяет покрытие.
    let proven = check(&input, &both, &bench.source);
    assert_eq!(proven.evidence.fatigue.status, Status::Pass);
    near(
        value(&proven.evidence.fatigue.result).min_m.unwrap(),
        0.000_5,
        ERROR,
    );
    assert_eq!(
        check(&input, &[cool], &bench.source)
            .evidence
            .fatigue
            .status,
        Status::NotAnalyzed
    );
    let gapped = [
        cool,
        CableFlexBand {
            temperature_k: [301.0, 302.0],
            ..warm
        },
    ];
    assert_eq!(
        check(&input, &gapped, &bench.source)
            .evidence
            .fatigue
            .status,
        Status::NotAnalyzed
    );
    let estimate = Provenance::Estimate("без циклического опыта".into());
    assert_eq!(
        check(&input, &both, &estimate).evidence.fatigue.status,
        Status::NotAnalyzed
    );
    for missing_match in [
        CableFlexDuty {
            orientation: "другая ориентация",
            ..duty
        },
        CableFlexDuty {
            profile: "разрывный опыт",
            ..duty
        },
        CableFlexDuty {
            cycles: 2 * CYCLES + 1,
            ..duty
        },
    ] {
        input.duty = Some(missing_match);
        let result = check(&input, &both, &bench.source);
        assert_eq!(result.status, Status::Pass);
        assert_eq!(result.evidence.fatigue.status, Status::NotAnalyzed);
    }
    duty.cycles = 2 * CYCLES;
    input.duty = Some(duty);
    let result = check(&input, &both, &bench.source);
    assert_eq!(result.status, Status::Fail);
    assert_eq!(result.evidence.bend.status, Status::Pass);
    assert_eq!(result.evidence.fatigue.status, Status::Fail);
    near(
        value(&result.evidence.fatigue.result).max_m.unwrap(),
        -0.002,
        ERROR,
    );
}

#[test]
fn неконечные_значения_ошибка_даже_при_неполном_входе() {
    let mut bench = Bench::new();
    bench
        .mid
        .insert("лишняя_метка".into(), [f64::NAN, 0.0, 0.0]);
    let steps = bench.steps();
    let guides = guide(0.011);
    let mut input = bench.input(&guides, &steps);
    input.mount = None;
    assert!(matches!(
        assess_cable_flex(&input, &bench.ratings()),
        Err(ProcessError::InvalidInput(_))
    ));
    input.motion = None;
    let limit = exact(f64::INFINITY);
    let ratings = CableFlexRatings {
        min_bend_radius_m: Some(&limit),
        ..bench.ratings()
    };
    assert!(matches!(
        assess_cable_flex(&input, &ratings),
        Err(ProcessError::InvalidInput(_))
    ));
}
