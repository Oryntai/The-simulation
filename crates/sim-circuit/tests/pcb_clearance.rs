//! Ручные расчёты зазоров и пути утечки в нейтральных постановках.

use sim_circuit::pcb::kicad;

// Экспорт из библиотеки относится к 4.9, как и для модели геометрии 4.2.
#[path = "../src/pcb/clearance.rs"]
mod clearance;
#[allow(dead_code)]
#[path = "../src/pcb/layout.rs"]
mod layout;

use clearance::{Conditions, Face, Limits, Open, Scope, SlotCredit, Status, evaluate};
use layout::{Layout, NotAnalyzed};

const MM: f64 = 1e-3;
const UNSAFE: &str = include_str!("../../../devices/electrical/depth_t4/unsafe.kicad_pcb");
const FIXED: &str = include_str!("../../../devices/electrical/depth_t4/fixed.kicad_pcb");
const EXPECTED: &str = "devices/electrical/depth_t4/expected.md";

fn load(source: &str) -> Layout {
    Layout::from_kicad(&kicad::parse(source).unwrap()).unwrap()
}

fn near(measured: f64, expected_mm: f64) {
    let expected = expected_mm * MM;
    assert!(
        (measured - expected).abs() <= kicad::METRES_PER_NM,
        "{measured:.12e} м вместо {expected:.12e} м"
    );
}

fn distance(distance: clearance::Distance, expected_mm: f64) {
    near(distance.lower_m, expected_mm);
    near(distance.upper_m.unwrap(), expected_mm);
}

// Дополнительная постановка: два круглых электрода и прорезь в воздухе.
// Напряжения и пределы синтетические, взяты для разноса результатов опыта.
fn conditions<'a>(material: &'a str) -> Conditions<'a> {
    Conditions {
        scope: Scope::Reference,
        source: "Постановка теста двух круглых электродов",
        material,
        environment: "сухой лабораторный воздух без загрязнения",
        working_v: 12.0,
        peak_v: 24.0,
        frequency_hz: 0.0,
        altitude_m: 0.0,
        distance_error_m: Some(0.0),
    }
}

fn limits(envelope: Conditions<'_>, air_mm: f64, surface_mm: Option<f64>) -> Limits<'_> {
    Limits {
        envelope,
        clearance_m: Some(air_mm * MM),
        creepage_m: surface_mm.map(|value| value * MM),
        slot_credit: None,
    }
}

const BOARD: &str = r#"(kicad_pcb
    (version 20240108) (generator "reference")
    (general (thickness 1.58))
    (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
    (setup (stackup
        (layer "F.Cu" (type "copper") (thickness 0.035))
        (layer "dielectric 1" (type "core") (thickness 1.51) (material "Блок"))
        (layer "B.Cu" (type "copper") (thickness 0.035))))
    (net 1 "А") (net 2 "Б")
    (footprint "Блок:1" (layer "F.Cu") (at 3 6)
        (pad "1" smd circle (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net 1 "А")))
    (footprint "Блок:2" (layer "F.Cu") (at 9 6)
        (pad "1" smd circle (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net 2 "Б")))
    (gr_rect (start 0 0) (end 12 12) (stroke (width 0.05) (type default)) (fill none)
      (layer "Edge.Cuts"))
    CUTOUT
)"#;

const SLOT: &str = r#"(gr_rect (start 5 3) (end 7 9)
    (stroke (width 0.05) (type default)) (fill none) (layer "Edge.Cuts"))"#;

#[test]
fn hand_reference_clearance_short_and_clean_without_physical_creepage_limit() {
    // У expected.md есть только геометрический зазор 0,25 мм: для сравнения
    // порог испытывается на синтетическом входе сравнителя.
    // Рабочее напряжение и предел утечки в ручном эталоне не заданы.
    let context = Conditions {
        source: "Синтетический режим сравнения расстояний",
        ..conditions("Диэлектрик эталона")
    };
    let rule = limits(
        Conditions {
            source: EXPECTED,
            ..context
        },
        0.25,
        None,
    );
    for (fixture, expected, short, sense_status, probe_status) in [
        (UNSAFE, 0.10, 0.0, Status::Fail, Status::Fail),
        (FIXED, 0.80, 2.20, Status::Pass, Status::Pass),
    ] {
        let board = load(fixture);
        for (pair, gap, expected_status) in [
            ([2, 3], expected, sense_status),
            ([6, 7], short, probe_status),
        ] {
            let report = evaluate(&board, Face::Front, pair, Some(&context), Some(&rule)).unwrap();
            distance(report.clearance.distance.unwrap(), gap);
            assert_eq!(report.clearance.status, expected_status);
            assert_eq!(report.creepage.status, Status::NotAnalyzed);
            assert!(report.creepage.open.contains(&Open::Limits));
            assert_eq!(
                report.status(),
                if expected_status == Status::Fail {
                    Status::Fail
                } else {
                    Status::NotAnalyzed
                }
            );
        }
    }
    assert_eq!(clearance::HAZARD_CODE, "creepage_clearance");
}

#[test]
fn through_slot_changes_geodesic_but_does_not_change_air_gap() {
    let material = conditions("Блок");
    let mut rule = limits(material, 5.0, Some(7.0));
    rule.slot_credit = Some(SlotCredit::Rectangular {
        minimum_width_m: 1.5 * MM,
    });
    let open_board = load(&BOARD.replace("CUTOUT", ""));
    let slot_board = load(&BOARD.replace("CUTOUT", SLOT));
    let open_result = evaluate(
        &open_board,
        Face::Front,
        [1, 2],
        Some(&material),
        Some(&rule),
    )
    .unwrap();
    let slot_result = evaluate(
        &slot_board,
        Face::Front,
        [1, 2],
        Some(&material),
        Some(&rule),
    )
    .unwrap();
    // Независимо от графа: расстояние центров 6 мм минус два радиуса.
    near(open_result.clearance.distance.unwrap().lower_m, 5.4);
    assert_eq!(
        open_result.clearance.distance,
        slot_result.clearance.distance
    );
    distance(open_result.geometric_creepage.unwrap(), 5.4);
    // До двух ближних углов катеты (2; 3) мм, по верхней или нижней кромке ещё 2 мм.
    let around = 2.0 * (2.0_f64.hypot(3.0) - 0.3) + 2.0;
    distance(slot_result.geometric_creepage.unwrap(), around);
    println!("Воздушный зазор 5,4 мм, путь по поверхности без/с прорезью 5,4/{around:.6} мм");
    assert_eq!(open_result.clearance.status, Status::Pass);
    assert_eq!(open_result.creepage.status, Status::Fail);
    assert_eq!(slot_result.creepage.status, Status::Pass);
    assert_eq!(slot_result.status(), Status::Pass);
    assert_eq!(
        slot_result,
        evaluate(
            &slot_board,
            Face::Front,
            [1, 2],
            Some(&material),
            Some(&rule)
        )
        .unwrap()
    );

    let rule_too_narrow = Limits {
        slot_credit: Some(SlotCredit::Rectangular {
            minimum_width_m: 2.5 * MM,
        }),
        ..rule
    };
    for rule in [
        Limits {
            slot_credit: None,
            ..rule
        },
        rule_too_narrow,
    ] {
        let report = evaluate(
            &slot_board,
            Face::Front,
            [1, 2],
            Some(&material),
            Some(&rule),
        )
        .unwrap();
        distance(report.geometric_creepage.unwrap(), around);
        near(report.creepage.distance.unwrap().lower_m, 5.4);
        assert_eq!(report.creepage.status, Status::NotAnalyzed);
        assert!(report.creepage.open.contains(&Open::SlotQualification));
    }
}

#[test]
fn outside_applicability_and_missing_layout_inputs_cannot_pass() {
    let context = conditions("Блок");
    let rule = limits(context, 5.0, Some(5.0));
    let board = load(&BOARD.replace("CUTOUT", ""));
    let mismatches = [
        Conditions {
            scope: Scope::Sourced,
            ..context
        },
        Conditions {
            material: "Другой диэлектрик",
            ..context
        },
        Conditions {
            environment: "конденсация",
            ..context
        },
        Conditions {
            working_v: 13.0,
            ..context
        },
        Conditions {
            peak_v: 25.0,
            ..context
        },
        Conditions {
            altitude_m: 1.0,
            ..context
        },
        Conditions {
            frequency_hz: 1.0,
            ..context
        },
    ];
    for conditions in mismatches {
        let report = evaluate(&board, Face::Front, [1, 2], Some(&conditions), Some(&rule)).unwrap();
        assert_eq!(report.status(), Status::NotAnalyzed);
        assert!(report.clearance.open.contains(&Open::OutsideEnvelope));
    }
    let report = evaluate(&board, Face::Front, [1, 2], None, Some(&rule)).unwrap();
    assert!(report.creepage.open.contains(&Open::Conditions));
    let report = evaluate(&board, Face::Front, [1, 2], Some(&context), None).unwrap();
    assert!(report.clearance.open.contains(&Open::Limits));
    let source = BOARD.replace("CUTOUT", "");
    for (modified, reason) in [
        (source.replace("(material \"Блок\")", ""), NotAnalyzed::MissingMaterial("dielectric 1".into())),
        (source.replace("(thickness 0.035)", ""), NotAnalyzed::MissingThickness("F.Cu".into())),
        (source.replace("(setup (stackup\n        (layer \"F.Cu\" (type \"copper\") (thickness 0.035))\n        (layer \"dielectric 1\" (type \"core\") (thickness 1.51) (material \"Блок\"))\n        (layer \"B.Cu\" (type \"copper\") (thickness 0.035))))", "(setup)"), NotAnalyzed::MissingStackup),
    ] {
        let report = evaluate(&load(&modified), Face::Front, [1, 2], Some(&context), Some(&rule)).unwrap();
        assert_eq!(report.status(), Status::NotAnalyzed);
        assert!(report.clearance.open.contains(&Open::Layout(reason)));
    }
    let without_edges = source.replace(
        r#"(gr_rect (start 0 0) (end 12 12) (stroke (width 0.05) (type default)) (fill none)
      (layer "Edge.Cuts"))"#,
        "",
    );
    let report = evaluate(
        &load(&without_edges),
        Face::Front,
        [1, 2],
        Some(&context),
        Some(&rule),
    )
    .unwrap();
    assert_eq!(report.clearance.status, Status::Pass);
    assert_eq!(report.creepage.status, Status::NotAnalyzed);
    assert!(
        report
            .creepage
            .open
            .contains(&Open::Layout(NotAnalyzed::MissingEdgeCuts))
    );
    let report = evaluate(&board, Face::Back, [1, 2], Some(&context), Some(&rule)).unwrap();
    assert_eq!(report.status(), Status::NotAnalyzed);
    assert!(report.clearance.open.contains(&Open::NoCopper(1)));
}

#[test]
fn nonfinite_and_unknown_references_are_input_errors_even_on_other_face() {
    let context = conditions("Блок");
    let rule = limits(context, 5.0, Some(7.0));
    let mut board = load(&BOARD.replace("CUTOUT", SLOT));
    for v in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let invalid = Conditions {
            frequency_hz: v,
            ..context
        };
        assert!(evaluate(&board, Face::Front, [1, 2], Some(&invalid), Some(&rule)).is_err());
        let invalid = Limits {
            creepage_m: Some(v),
            ..rule
        };
        assert!(evaluate(&board, Face::Front, [1, 2], Some(&context), Some(&invalid)).is_err());
        let invalid = Limits {
            slot_credit: Some(SlotCredit::Rectangular { minimum_width_m: v }),
            ..rule
        };
        assert!(evaluate(&board, Face::Front, [1, 2], Some(&context), Some(&invalid)).is_err());
    }
    assert!(evaluate(&board, Face::Front, [1, 1], None, None).is_err());
    assert!(evaluate(&board, Face::Front, [1, 3], None, None).is_err());
    board.footprints[0].pads[0]
        .land
        .as_mut()
        .unwrap()
        .outline
        .width_m = f64::NAN;
    assert!(evaluate(&board, Face::Back, [1, 2], None, None).is_err());
}

#[test]
fn concave_board_edge_is_counted_only_with_an_applicable_source() {
    let context = conditions("Блок");
    let rule = limits(context, 5.0, Some(7.0));
    // Та же ширина канала 2 мм, но теперь он открыт к внешней кромке.
    let source = BOARD.replace(
        "(gr_rect (start 0 0) (end 12 12)",
        "(gr_poly (pts (xy 0 0) (xy 5 0) (xy 5 9) (xy 7 9) (xy 7 0) (xy 12 0) (xy 12 12) (xy 0 12))",
    );
    let board = load(&source.replace("CUTOUT", ""));
    let report = evaluate(&board, Face::Front, [1, 2], Some(&context), Some(&rule)).unwrap();
    distance(
        report.geometric_creepage.unwrap(),
        2.0 * 2.0_f64.hypot(3.0) + 2.0 - 0.6,
    );
    assert_eq!(report.creepage.status, Status::NotAnalyzed);
    let rule = Limits {
        slot_credit: Some(SlotCredit::Verified {
            source: "Ручная разметка открытого канала",
        }),
        ..rule
    };
    let report = evaluate(&board, Face::Front, [1, 2], Some(&context), Some(&rule)).unwrap();
    assert_eq!(report.status(), Status::Pass);
}

#[test]
fn arc_uses_copper_edge_with_sagitta_bounds() {
    // Дуга радиуса 2 мм, центр (5; 6). Ближайшая к площадке точка оси
    // (5; 4). Зазор: 1 мм между осью и центром минус 0,2 и 0,3 мм.
    let source = BOARD.replace("(at 9 6)", "(at 5 3)").replace(
        "CUTOUT",
        r#"(arc (start 3 6) (mid 5 4) (end 7 6) (width 0.4) (layer "F.Cu") (net 1))"#,
    );
    let context = conditions("Блок");
    let rule = limits(context, 0.6, Some(0.6));
    let report = evaluate(
        &load(&source),
        Face::Front,
        [1, 2],
        Some(&context),
        Some(&rule),
    )
    .unwrap();
    let bound = report.clearance.distance.unwrap();
    assert!(bound.lower_m <= 0.5 * MM && bound.upper_m.unwrap() >= 0.5 * MM);
    // У хорды две огибающие, каждая в пределах одного шага сетки.
    assert!(bound.upper_m.unwrap() - bound.lower_m <= 2.0 * kicad::METRES_PER_NM);
    assert_eq!(report.status(), Status::Fail);
}

#[test]
fn saved_zone_fill_and_via_annulus_are_used_instead_of_design_outline() {
    // Контур заканчивается в x=8, сохранённая медь в x=7. Площадка
    // начинается в x=8,7. Кольцо перехода без заливки заканчивается в x=6,4.
    const ZONE: &str = r#"(zone (net 1) (net_name "А") (layer "F.Cu") (fill yes)
        (polygon (pts (xy 1 4) (xy 8 4) (xy 8 8) (xy 1 8)))
        FILL)"#;
    const FILL: &str =
        r#"(filled_polygon (layer "F.Cu") (pts (xy 1 4) (xy 7 4) (xy 7 8) (xy 1 8)))"#;
    const VIA: &str = r#"(via (at 6 6) (size 0.8) (drill 0.4) (layers "F.Cu" "B.Cu") (net 1))"#;
    let context = conditions("Блок");
    let rule = limits(context, 2.0, Some(2.0));
    let via = BOARD.replace("CUTOUT", VIA);
    let filled = via.replace(VIA, &format!("{VIA} {}", ZONE.replace("FILL", FILL)));
    let unfilled = via.replace(VIA, &format!("{VIA} {}", ZONE.replace("FILL", "")));
    for (source, expected, status) in [
        (&via, 2.3, Status::Pass),
        (&filled, 1.7, Status::Fail),
        (&unfilled, 2.3, Status::NotAnalyzed),
    ] {
        let report = evaluate(
            &load(source),
            Face::Front,
            [1, 2],
            Some(&context),
            Some(&rule),
        )
        .unwrap();
        distance(report.clearance.distance.unwrap(), expected);
        assert_eq!(report.status(), status);
        if status == Status::NotAnalyzed {
            assert!(
                report
                    .clearance
                    .open
                    .contains(&Open::Layout(NotAnalyzed::MissingFill("F.Cu".into())))
            );
        }
    }
}

#[test]
fn drilled_void_and_unknown_curved_topology_do_not_yield_a_false_surface_witness() {
    const ROUND_EDGE: &str = r#"(gr_circle (center 6 6) (end 7 6)
        (stroke (width 0.05) (type default)) (fill none) (layer "Edge.Cuts"))"#;
    const DRILL: &str = r#"(footprint "Блок:Отверстие" (layer "F.Cu") (at 6 6)
        (pad "" np_thru_hole circle (at 0 0) (size 2 2) (drill 2) (layers "*.Mask")))"#;
    let context = conditions("Блок");
    let rule = limits(context, 5.0, Some(6.0));
    let drilled = evaluate(
        &load(&BOARD.replace("CUTOUT", DRILL)),
        Face::Front,
        [1, 2],
        Some(&context),
        Some(&rule),
    )
    .unwrap();
    distance(drilled.clearance.distance.unwrap(), 5.4);
    let geometry = drilled.geometric_creepage.unwrap();
    near(geometry.lower_m, 5.4);
    assert_eq!(geometry.upper_m, None);
    assert_eq!(drilled.creepage.status, Status::NotAnalyzed);
    let curved = evaluate(
        &load(&BOARD.replace("CUTOUT", ROUND_EDGE)),
        Face::Front,
        [1, 2],
        Some(&context),
        Some(&rule),
    )
    .unwrap();
    assert_eq!(curved.clearance.status, Status::Pass);
    assert_eq!(curved.geometric_creepage, None);
    assert!(
        curved
            .creepage
            .open
            .contains(&Open::Layout(NotAnalyzed::CurvedEdgeTopology))
    );
}

#[test]
fn credible_geometry_and_surface_material_are_needed_for_a_positive_comparison() {
    let board = load(&BOARD.replace("CUTOUT", ""));
    let context = conditions("Блок");
    let rule = limits(context, 5.0, Some(5.0));
    for (context, rule, reason) in [
        (
            Conditions {
                distance_error_m: None,
                ..context
            },
            rule,
            Open::MissingGeometryError,
        ),
        // Расстояние 5,4 мм, а погрешность 0,5 мм пересекает предел 5 мм.
        (
            Conditions {
                distance_error_m: Some(0.5 * MM),
                ..context
            },
            rule,
            Open::DistanceUncertainty,
        ),
        (
            conditions("Другая поверхность"),
            limits(conditions("Другая поверхность"), 5.0, Some(5.0)),
            Open::SurfaceMaterialMismatch,
        ),
    ] {
        let report = evaluate(&board, Face::Front, [1, 2], Some(&context), Some(&rule)).unwrap();
        assert_eq!(report.status(), Status::NotAnalyzed);
        assert!(report.creepage.open.contains(&reason), "{report:?}");
    }
}

#[test]
fn oblique_pad_uses_kicad_y_down_angle_and_detects_violation() {
    // От центра (3; 6) к (5; 4) вдоль длинной оси площадки +45°.
    // До круга остаётся sqrt(2² + 2²) − 2 − 0,3 мм.
    let pad = r#"(pad "1" smd circle (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net 1 "А"))"#;
    let rotated = r#"(pad "1" smd rect (at 0 0 45) (size 4 0.6) (layers "F.Cu") (net 1 "А"))"#;
    let source = BOARD
        .replace(pad, rotated)
        .replace("(at 9 6)", "(at 5 4)")
        .replace("CUTOUT", "");
    let context = conditions("Блок");
    let rule = limits(context, 0.6, Some(0.6));
    let report = evaluate(
        &load(&source),
        Face::Front,
        [1, 2],
        Some(&context),
        Some(&rule),
    )
    .unwrap();
    distance(
        report.clearance.distance.unwrap(),
        2.0_f64.hypot(2.0) - 2.0 - 0.3,
    );
    assert_eq!(report.status(), Status::Fail);
}

#[test]
fn route_through_third_net_is_not_a_dielectric_witness() {
    // Стержень меди третьей цепи пересекает прямой отрезок между круглыми
    // площадками. Для нижней границы его можно считать диэлектриком.
    let source = BOARD
        .replacen("(net 1 \"А\")", "(net 1 \"А\") (net 3 \"В\")", 1)
        .replace(
            "CUTOUT",
            r#"(segment (start 6 5) (end 6 7) (width 0.5) (layer "F.Cu") (net 3))"#,
        );
    let context = conditions("Блок");
    let rule = limits(context, 5.0, Some(5.5));
    let report = evaluate(
        &load(&source),
        Face::Front,
        [1, 2],
        Some(&context),
        Some(&rule),
    )
    .unwrap();
    distance(report.clearance.distance.unwrap(), 5.4);
    assert_eq!(report.creepage.status, Status::NotAnalyzed);
    assert_eq!(report.geometric_creepage.unwrap().upper_m, None);
    assert!(report.creepage.open.contains(&Open::SurfaceWitnessBlocked));
}
