//! Разметка трассы, окна stackup и пределы расчёта Z₀.

use sim_circuit::pcb::{
    CU_RHO_20C, FOIL_M_PER_OZ, IpcLayer, MIL, ipc2221_current, ipc2221_delta_t, kicad,
    microstrip_hammerstad_jensen, trace_resistance, via_resistance,
};

// Точка экспорта соседних модулей и оценщика предусмотрена задачей 4.9.
#[path = "../src/pcb/impedance.rs"]
mod impedance;
#[allow(dead_code)]
#[path = "../src/pcb/layout.rs"]
mod layout;
#[allow(dead_code)]
#[path = "../src/pcb/ratings.rs"]
mod ratings;
#[allow(dead_code)]
#[path = "../src/pcb/return_path.rs"]
mod return_path;

use impedance::{Limit, Mark, Open, ProcessWindow, Status, evaluate};
use layout::{Layout, NotAnalyzed, Point};
use ratings::{Datum, Scope, Source};
use return_path::{Plane, ReferenceInput};

const UNSAFE: &str = include_str!("../../../devices/electrical/depth_t4/unsafe.kicad_pcb");
const FIXED: &str = include_str!("../../../devices/electrical/depth_t4/fixed.kicad_pcb");
const TRACK: &str = "bd430000-0000-4000-8000-000000000042";
const OTHER_TRACK: &str = "bd430000-0000-4000-8000-000000000041";
const SOURCE: Source<'static> = Source::Reference("Ручная постановка tests/pcb_impedance.rs");
// depth_t4/expected.md, номиналы SENSE_A и стека.
const W: f64 = 0.400e-3;
const H: f64 = 1.510e-3;
const T: f64 = 0.035e-3;
const ER: f64 = 4.2;

fn point(x_nm: i32, y_nm: i32) -> Point {
    Point { x_nm, y_nm }
}

fn from_text(text: &str) -> (Layout, Mark<'static>) {
    let (board, ids) = kicad::parse_with_track_ids(text).unwrap();
    let mark = Mark {
        reference: ReferenceInput {
            scope: Scope::Reference,
            track: ids.index_of(TRACK).unwrap(),
            plane: Plane {
                zone: 0,
                layer: "B.Cu",
                // depth_t4/expected.md, конец подключённой ветви на B.Cu.
                tie: point(5_000_000, 9_000_000),
                connection: Some(SOURCE),
            },
        },
        process: None,
        limit: None,
        model_error_ohm: None,
    };
    (Layout::from_kicad(&board).unwrap(), mark)
}

// Независимая приближённая сверка: поправки Уилера (11.19, 11.20),
// εэфф Шнайдера (11.14) и узкая полоса по приближению конформного отображения.
// https://qucs.github.io/tech/node75.html
fn wheeler_schneider() -> f64 {
    let dw_air = T / std::f64::consts::PI
        * (4.0 * std::f64::consts::E
            / (T / H).hypot(1.0 / (std::f64::consts::PI * (W / T + 1.10))))
        .ln();
    let u = (W + dw_air * (1.0 + 1.0 / ER) / 2.0) / H;
    let effective = (ER + 1.0) / 2.0 + (ER - 1.0) / (2.0 * (1.0 + 10.0 / u).sqrt());
    60.0 * (8.0 / u + u / 4.0).ln() / effective.sqrt()
}

fn process_window() -> ProcessWindow<'static> {
    // Синтетический разброс для проверки всех направлений изменения.
    // Шаг W и H выбран равным толщине фольги, шаг T — толщине маски
    // из ручного эталона. Второе εr = 4,1 взято из примера Steer §4.4.
    let mask = 0.010e-3;
    ProcessWindow {
        width_m: [W - T, W + T],
        dielectric_height_m: [H - T, H + T],
        trace_copper_m: [T - mask, T + mask],
        epsilon_r: [4.1, ER],
        // Статическая постановка для сверки формул.
        band_hz: [0.0; 2],
        source: SOURCE,
    }
}

fn limit(ohms: [f64; 2]) -> Option<Limit<'static>> {
    Some(Limit {
        ohms,
        source: SOURCE,
    })
}

#[test]
fn marked_pair_checks_the_manually_mapped_return_and_stack() {
    let (bad, mark) = from_text(UNSAFE);
    let report = evaluate(&bad, &mark).unwrap();
    assert_eq!(report.status, Status::NotAnalyzed(Open::InterruptedReturn));
    assert_eq!(report.nominal_ohm, None);

    let (good, mut mark) = from_text(FIXED);
    let trace = &good.tracks[mark.reference.track];
    assert_eq!(
        trace.path,
        layout::Centerline::Segment {
            start: point(22_000_000, 5_000_000),
            end: point(32_000_000, 5_000_000),
        }
    );
    assert_eq!(good.nets[&trace.net.unwrap()], "SENSE_A");
    let stack = good.stackup().unwrap();
    let dielectrics = stack.dielectrics_between("F.Cu", "B.Cu").unwrap();
    assert!((dielectrics[0].thickness_m().unwrap() - H).abs() < kicad::METRES_PER_NM);
    assert!((trace.width_m - W).abs() < kicad::METRES_PER_NM);
    assert_eq!(dielectrics[0].epsilon_r().unwrap(), ER);
    assert!(
        (stack.copper_center_distance_m("F.Cu", "B.Cu").unwrap() - (H + T)).abs()
            < kicad::METRES_PER_NM
    );

    let report = evaluate(&good, &mark).unwrap();
    assert_eq!(
        report.status,
        Status::NotAnalyzed(Open::MissingOrUnverified("допуски стека и εr(f)"))
    );
    let z0 = report.nominal_ohm.unwrap();
    let independent = wheeler_schneider();
    println!("Z₀ эталонной трассы: {z0:.6} Ом, Уилер–Шнайдер: {independent:.6} Ом");
    // 2% — допуск сопоставления двух приближений в постановке, см. §11.2 Qucs.
    assert!((z0 / independent - 1.0).abs() < 0.02);
    assert_eq!(report.process_range_ohm, None);

    mark.process = Some(ProcessWindow {
        width_m: [W; 2],
        dielectric_height_m: [H; 2],
        trace_copper_m: [T; 2],
        epsilon_r: [ER; 2],
        ..process_window()
    });
    let report = evaluate(&good, &mark).unwrap();
    let [lo, hi] = report.process_range_ohm.unwrap();
    assert!(lo <= z0 && z0 <= hi);
    assert!((hi - lo) / z0 < 16.0 * f64::EPSILON);
    assert_eq!(
        report.status,
        Status::NotAnalyzed(Open::MissingOrUnverified(
            "погрешность модели в рабочей полосе"
        ))
    );
}

#[test]
fn process_envelope_covers_corners_and_interior_and_checks_both_limits() {
    let (board, mut mark) = from_text(FIXED);
    let process = process_window();
    mark.process = Some(process);
    // Для расчётного опыта сравнивается сама формула, поэтому ошибка модели здесь 0.
    mark.model_error_ohm = Some(Datum {
        value: 0.0,
        source: SOURCE,
    });
    let oracle = wheeler_schneider();
    // Пределы синтетических опытов по независимому ориентиру, не пределы платы.
    mark.limit = limit([oracle * 0.80, oracle * 1.20]);
    let report = evaluate(&board, &mark).unwrap();
    assert_eq!(report.status, Status::Pass);
    let [lo, hi] = report.comparison_range_ohm.unwrap();
    let grid = |[a, b]: [f64; 2]| [a, a.midpoint(b), b];
    for w in grid(process.width_m) {
        for h in grid(process.dielectric_height_m) {
            for t in grid(process.trace_copper_m) {
                for er in grid(process.epsilon_r) {
                    let value = microstrip_hammerstad_jensen(w, h, t, er).z0;
                    assert!((lo..=hi).contains(&value), "{value} Ом вне [{lo}, {hi}]");
                }
            }
        }
    }
    println!("Z₀ с технологическими допусками: [{lo:.6}, {hi:.6}] Ом");
    assert!(lo < oracle && oracle < hi);
    for outside in [
        [oracle * 0.40, oracle * 0.50],
        [oracle * 1.20, oracle * 1.40],
    ] {
        mark.limit = limit(outside);
        assert_eq!(evaluate(&board, &mark).unwrap().status, Status::Fail);
    }
    mark.limit = limit([oracle * 0.80, oracle]);
    assert_eq!(
        evaluate(&board, &mark).unwrap().status,
        Status::NotAnalyzed(Open::OverlapsLimit)
    );
    mark.limit = None;
    let report = evaluate(&board, &mark).unwrap();
    assert_eq!(
        report.status,
        Status::NotAnalyzed(Open::MissingOrUnverified("допустимый Z₀"))
    );
    assert_eq!(report.comparison_range_ohm, Some([lo, hi]));
}

#[test]
fn missing_stack_copper_material_permittivity_or_fill_stays_open() {
    let (_, mark) = from_text(FIXED);
    let fixture = kicad::parse(FIXED).unwrap();
    let mut board = fixture.clone();
    board.stackup = None;
    let layout = |board: &kicad::KiCadBoard| Layout::from_kicad(board).unwrap();
    assert_eq!(
        evaluate(&layout(&board), &mark).unwrap().status,
        Status::NotAnalyzed(Open::Layout(NotAnalyzed::MissingStackup))
    );
    for layer in ["F.Cu", "B.Cu", "dielectric 1"] {
        let mut board = fixture.clone();
        board
            .stackup
            .as_mut()
            .unwrap()
            .iter_mut()
            .find(|l| l.name == layer)
            .unwrap()
            .thickness_nm = None;
        assert_eq!(
            evaluate(&layout(&board), &mark).unwrap().status,
            Status::NotAnalyzed(Open::Layout(NotAnalyzed::MissingThickness(layer.into())))
        );
    }
    let mut board = fixture.clone();
    board
        .stackup
        .as_mut()
        .unwrap()
        .iter_mut()
        .find(|l| l.name == "dielectric 1")
        .unwrap()
        .epsilon_r = None;
    let report = evaluate(&layout(&board), &mark).unwrap();
    assert_eq!(
        report.status,
        Status::NotAnalyzed(Open::Layout(NotAnalyzed::MissingPermittivity(
            "dielectric 1".into()
        )))
    );
    assert_eq!(report.nominal_ohm, None);
    let mut board = fixture.clone();
    board
        .stackup
        .as_mut()
        .unwrap()
        .iter_mut()
        .find(|l| l.name == "dielectric 1")
        .unwrap()
        .material = None;
    assert_eq!(
        evaluate(&layout(&board), &mark).unwrap().status,
        Status::NotAnalyzed(Open::Layout(NotAnalyzed::MissingMaterial(
            "dielectric 1".into()
        )))
    );
    let mut board = fixture;
    board.zones[0].filled.clear();
    assert_eq!(
        evaluate(&layout(&board), &mark).unwrap().status,
        Status::NotAnalyzed(Open::Layout(NotAnalyzed::MissingFill("B.Cu".into())))
    );
}

#[test]
fn physical_scope_requires_applicable_band_model_and_limit() {
    let (board, mut mark) = from_text(FIXED);
    mark.reference.scope = Scope::Assembly;
    mark.reference.plane.connection = Some(Source::Verified(
        "Привязка в синтетическом тесте области сборки",
    ));
    let mut process = process_window();
    mark.process = Some(process);
    assert_eq!(
        evaluate(&board, &mark).unwrap().status,
        Status::NotAnalyzed(Open::MissingOrUnverified("допуски стека и εr(f)"))
    );

    // Имитация типизированной записи для проверки маршрута результата.
    let verified = Source::Verified("Синтетическая запись верхней границы ошибки в тесте");
    process.source = verified;
    mark.process = Some(process);
    let oracle = wheeler_schneider();
    mark.limit = limit([oracle * 0.80, oracle * 1.20]);
    mark.model_error_ohm = Some(Datum {
        value: oracle * 0.01,
        source: Source::Estimate("Оценка без стенда"),
    });
    let report = evaluate(&board, &mark).unwrap();
    assert_eq!(
        report.status,
        Status::NotAnalyzed(Open::MissingOrUnverified(
            "погрешность модели в рабочей полосе"
        ))
    );
    assert!(report.process_range_ohm.is_some());
    assert!(report.comparison_range_ohm.is_none());

    mark.model_error_ohm.as_mut().unwrap().source = verified;
    let report = evaluate(&board, &mark).unwrap();
    assert_eq!(
        report.status,
        Status::NotAnalyzed(Open::MissingOrUnverified("допустимый Z₀"))
    );
    let nominal = report.process_range_ohm.unwrap();
    let expanded = report.comparison_range_ohm.unwrap();
    let error = mark.model_error_ohm.unwrap().value;
    assert!(expanded[0] <= nominal[0] - error && expanded[1] >= nominal[1] + error);
    mark.limit.as_mut().unwrap().source = verified;
    assert_eq!(evaluate(&board, &mark).unwrap().status, Status::Pass);
    // Даже когда сам диапазон стека помещался, после учёта ошибки вывод открыт.
    mark.limit.as_mut().unwrap().ohms = nominal;
    assert_eq!(
        evaluate(&board, &mark).unwrap().status,
        Status::NotAnalyzed(Open::OverlapsLimit)
    );
}

#[test]
fn nonfinite_and_conflicting_data_are_errors() {
    let (bad, mut mark) = from_text(UNSAFE);
    let mut process = process_window();
    for nonfinite in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        process.band_hz[1] = nonfinite;
        mark.process = Some(process);
        assert!(evaluate(&bad, &mark).is_err());
        mark.process = None;
        mark.limit = limit([1.0, nonfinite]);
        assert!(evaluate(&bad, &mark).is_err());
        mark.limit = None;
        mark.model_error_ohm = Some(Datum {
            value: nonfinite,
            source: SOURCE,
        });
        assert!(evaluate(&bad, &mark).is_err());
        mark.model_error_ohm = None;
    }
    let (good, mut mark) = from_text(FIXED);
    mark.process = Some(ProcessWindow {
        width_m: [W - T; 2],
        ..process_window()
    });
    assert!(evaluate(&good, &mark).is_err());
    mark.process = Some(ProcessWindow {
        trace_copper_m: [T, T / 2.0],
        ..process_window()
    });
    assert!(evaluate(&good, &mark).is_err());
    mark.process = Some(process_window());
    mark.model_error_ohm = Some(Datum {
        value: f64::MAX,
        source: SOURCE,
    });
    assert!(evaluate(&good, &mark).is_err());
    mark.reference.track = good.tracks.len();
    assert!(evaluate(&good, &mark).is_err());
}

#[test]
fn unsupported_curvature_and_gap_at_maximum_width_stay_open() {
    let (_, mut mark) = from_text(FIXED);
    let mut board = kicad::parse(FIXED).unwrap();
    let track = mark.reference.track;
    board.tracks[track].path = kicad::Path::Arc {
        // Середина исходного отрезка (27;5), подъём дуги на толщину маски.
        start: point(22_000_000, 5_000_000),
        mid: point(27_000_000, 5_010_000),
        end: point(32_000_000, 5_000_000),
    };
    let layout = Layout::from_kicad(&board).unwrap();
    assert_eq!(
        evaluate(&layout, &mark).unwrap().status,
        Status::NotAnalyzed(Open::UnsupportedGeometry)
    );

    // Край полигона y = 3 мм. При y = 3 мм + W/2 + T/4 номинал
    // помещается, а максимальная ширина W + T пересекает край.
    let y_nm = 3_000_000 + 400_000 / 2 + 35_000 / 4;
    board.tracks[track].path = kicad::Path::Segment {
        start: point(22_000_000, y_nm),
        end: point(32_000_000, y_nm),
    };
    let layout = Layout::from_kicad(&board).unwrap();
    mark.process = Some(process_window());
    let report = evaluate(&layout, &mark).unwrap();
    assert_eq!(report.status, Status::NotAnalyzed(Open::InterruptedReturn));
    assert!(report.nominal_ohm.is_some());
    assert_eq!(report.process_range_ohm, None);
}

#[test]
fn kicad_8_and_9_identifiers_resolve_without_changing_the_board_api() {
    let (board, ids) = kicad::parse_with_track_ids(FIXED).unwrap();
    assert_eq!(board.version, 20240108);
    assert_eq!(board, kicad::parse(FIXED).unwrap());
    assert_eq!(ids.index_of(&TRACK.to_ascii_uppercase()), Some(1));
    assert_eq!(ids.index_of(OTHER_TRACK), Some(0));
    assert_eq!(ids.index_of("неизвестный"), None);

    // KiCad 9.0.0 SEXPR_BOARD_FILE_VERSION. Упрощённая плата с дугой,
    // старым написанием идентификатора и необязательной меткой сегмента.
    let text = format!(
        r#"(kicad_pcb (version 20241229) (generator pcbnew) (general)
            (layers (0 "F.Cu" signal) (2 "B.Cu" signal)) (setup)
            (segment (start 22 5) (end 32 5) (width 0.4) (layer "F.Cu") (net 0))
            (arc (start 22 5) (mid 27 6) (end 32 5) (width 0.4) (layer "F.Cu") (net 0)
              (tstamp "{TRACK}")))"#
    );
    let (board, ids) = kicad::parse_with_track_ids(&text).unwrap();
    assert_eq!(board.version, 20241229);
    assert_eq!(board.tracks.len(), 2);
    assert_eq!(ids.index_of(TRACK), Some(1));
    assert!(matches!(board.tracks[1].path, kicad::Path::Arc { .. }));

    let ambiguous = FIXED.replace(OTHER_TRACK, &TRACK.to_ascii_uppercase());
    let malformed = FIXED.replace(TRACK, "невалидный");
    let double = FIXED.replace(
        &format!(r#"(uuid "{TRACK}")"#),
        &format!(r#"(uuid "{TRACK}") (tstamp "{TRACK}")"#),
    );
    for text in [ambiguous, malformed, double] {
        let error = kicad::parse_with_track_ids(&text).unwrap_err();
        assert_eq!(error.kind, kicad::ErrorKind::InvalidReference);
        assert!(error.line > 1);
        // Прежний вход сохраняет поддержку файлов, в которых эти метаданные игнорировались.
        assert!(kicad::parse(&text).is_ok());
    }
}
