//! Ручные эталоны: числа ниже размечены в файле теста и отдельно сверены в СИ.

use sim_circuit::pcb::kicad::{
    Drill, Edge, ErrorKind, KiCadBoard, PadKind, PadShape, Path, Point, ViaKind, parse,
};

const BOARD: &str = r#"(kicad_pcb
  (version 20241229)
  (generator "pcbnew")
  (generator_version "9.0")
  (general (thickness 1.6) (legacy_teardrops no))
  (paper "A4")
  (layers (0 "F.Cu" signal) (4 "In1.Cu" power "Опора")
          (6 "In2.Cu" signal) (2 "B.Cu" signal)
          (5 "F.Paste" user) (1 "F.Mask" user) (7 "F.SilkS" user) (25 "Edge.Cuts" user))
  (setup (stackup
      (layer "F.Cu" (type "copper") (thickness 0.035))
      (layer "dielectric 1" (type "prepreg") (thickness 0.2 locked)
         (material "Материал эталона") (epsilon_r 4.2) (loss_tangent 0.02))
      (layer "In1.Cu" (type "copper"))
      (layer "In2.Cu" (type "copper") (thickness 0.017))
      (layer "B.Cu" (type "copper") (thickness 0.035)))
    (pad_to_mask_clearance 0) (tenting none) (pcbplotparams (useauxorigin false)))
  (net 0 "") (net 5 "Сигнал/\"А\"") (net 9 "Опора")
  (footprint "Эталон:Блок" (layer "F.Cu") (at 10.125 20.25 90)
    (property "Reference" "X1" (at 0 -2 90) (layer "F.SilkS"))
    (fp_circle (center 0 0) (end 1 0) (fill no) (layer "F.SilkS"))
    (pad "1" smd roundrect (at 1.25 -0.875 135) (size 1.2 0.8)
      (layers "F.Cu" "F.Paste" "F.Mask") (roundrect_rratio 0.25) (net 5 "Сигнал/\"А\""))
    (pad "2" thru_hole oval (at 2.5 0) (size 2 1.5)
      (drill oval 1.2 0.6 (offset 0.1 -0.2)) (layers "*.Cu" "*.Mask") (net 9 "Опора"))
    (pad "" np_thru_hole circle (at -1.125 2) (size 0.75 0.75)
      (drill 0.75) (layers "*.Cu" "*.Mask")))
  (footprint "Эталон:Каретка" (layer "B.Cu") (at 22 5 -45)
    (fp_text reference "X2" (at -1 0) (layer "F.SilkS"))
    (pad "1" connect rect (at -0.5 1.25 45) (size 1 2) (layers "B.Cu"))
    (fp_line (start -1 0) (end 1 0) (stroke (width 0.05) (type default)) (layer "Edge.Cuts")))
  (segment (start 10.125 20.25) (end 13.125 24.25) (width 0.25) (layer "F.Cu") (net 5))
  (arc (start 13.125 24.25) (mid 14.125 23.25) (end 15.125 24.25) (width 0.25) (layer "F.Cu") (net 5))
  (via (at 15.125 24.25) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 5))
  (via micro (at 8 9) (size 0.3) (drill 0.1) (layers "F.Cu" "In1.Cu") (net 9))
  (via blind (at 18 19) (size 0.5) (drill 0.2) (layers "In1.Cu" "In2.Cu") (net 9))
  (gr_rect (start 2 3) (end 24 30) (stroke (width 0.05) (type default)) (fill no) (layer "Edge.Cuts"))
  (gr_circle (center 7 8) (end 7.5 8) (stroke (width 0.05) (type default)) (fill none) (layer "Edge.Cuts"))
  (zone (net 9) (net_name "Опора") (layers "In1.Cu" "B.Cu") (hatch edge 0.5)
    (connect_pads (clearance 0.2)) (min_thickness 0.25) (fill yes (thermal_gap 0.3))
    (polygon (pts (xy 4 4) (xy 12 4) (xy 12 8) (xy 4 8)))
    (polygon (pts (xy 6 5) (xy 6 7) (xy 8 7) (xy 8 5)))
    (filled_polygon (layer "In1.Cu") (island)
      (pts (xy 4.25 4.25) (xy 5.75 4.25) (xy 5.75 7.75) (xy 4.25 7.75)))))"#;

const EMPTY_V8: &str = r#"(kicad_pcb (version 20240108) (generator pcbnew) (generator_version "8.0")
  (general (thickness 1.6)) (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
  (setup (pad_to_mask_clearance 0)))"#;

fn nm(x_nm: i32, y_nm: i32) -> Point {
    Point { x_nm, y_nm }
}

fn track_board() -> KiCadBoard {
    parse(BOARD).expect("прочитать плату ручного эталона")
}

#[test]
fn manual_track_and_via_coordinates() {
    let board = track_board();
    assert_eq!(board.version, 20241229);
    assert_eq!(board.generator_version.as_deref(), Some("9.0"));
    assert_eq!(board.layers[1].user_name.as_deref(), Some("Опора"));
    assert_eq!(
        (board.layers[3].ordinal, board.layers[3].name.as_str()),
        (2, "B.Cu")
    );
    assert_eq!(board.nets[&5], "Сигнал/\"А\"");
    assert_eq!(board.tracks.len(), 2);
    let segment = &board.tracks[0];
    assert_eq!(
        segment.path,
        Path::Segment {
            start: nm(10_125_000, 20_250_000),
            end: nm(13_125_000, 24_250_000)
        }
    );
    assert_eq!(
        (segment.width_nm, segment.layer.as_str(), segment.net),
        (250_000, "F.Cu", 5)
    );
    let Path::Segment { start, end } = segment.path else {
        panic!("ожидался отрезок")
    };
    let [x0, y0] = start.metres();
    let [x1, y1] = end.metres();
    // Треугольник 3–4–5 мм. Допуск 1 нм соответствует сетке записи.
    assert!(((x1 - x0).hypot(y1 - y0) - 0.005).abs() < 1e-9);
    let Path::Arc { start, mid, end } = board.tracks[1].path else {
        panic!("ожидалась дуга")
    };
    assert_eq!(mid, nm(14_125_000, 23_250_000));
    for point in [start, mid, end] {
        // Ручная разметка окружности: центр (14,125; 24,25) мм, радиус 1 мм.
        let [x, y] = point.metres();
        assert!(((x - 0.014125).hypot(y - 0.02425) - 0.001).abs() < 1e-9);
    }
    assert_eq!(board.vias.len(), 3);
    let expected = [
        (
            ViaKind::Through,
            nm(15_125_000, 24_250_000),
            600_000,
            300_000,
            ["F.Cu", "B.Cu"],
            5,
        ),
        (
            ViaKind::Micro,
            nm(8_000_000, 9_000_000),
            300_000,
            100_000,
            ["F.Cu", "In1.Cu"],
            9,
        ),
        (
            ViaKind::BlindBuried,
            nm(18_000_000, 19_000_000),
            500_000,
            200_000,
            ["In1.Cu", "In2.Cu"],
            9,
        ),
    ];
    for (via, expected) in board.vias.iter().zip(expected) {
        assert_eq!(
            (
                via.kind,
                via.at,
                via.diameter_nm,
                via.drill_nm,
                via.layers.each_ref().map(String::as_str),
                via.net
            ),
            expected
        );
    }
    assert_eq!(board, track_board());
}

#[test]
fn manual_footprint_and_pad_coordinates() {
    let board = track_board();
    assert_eq!(board.footprints.len(), 2);
    let fp = &board.footprints[0];
    assert_eq!(fp.reference.as_deref(), Some("X1"));
    assert_eq!(fp.at.point, nm(10_125_000, 20_250_000));
    assert_eq!(fp.at.angle_rad, std::f64::consts::FRAC_PI_2);
    assert_eq!(fp.pads.len(), 3);
    let pad = &fp.pads[0];
    assert_eq!(
        (pad.number.as_str(), pad.kind, pad.net),
        ("1", PadKind::Smd, 5)
    );
    assert_eq!(pad.at.point, nm(1_250_000, -875_000));
    assert_eq!(pad.at.angle_rad, 3.0 * std::f64::consts::FRAC_PI_4);
    assert_eq!((pad.width_nm, pad.height_nm), (1_200_000, 800_000));
    assert_eq!(pad.shape, PadShape::RoundRect { radius_ratio: 0.25 });
    assert_eq!(pad.layers, ["F.Cu", "F.Paste", "F.Mask"]);
    assert_eq!(pad.drill, None);
    let pad = &fp.pads[1];
    assert_eq!(
        (pad.number.as_str(), pad.kind, pad.net),
        ("2", PadKind::PlatedHole, 9)
    );
    assert_eq!(pad.at.point, nm(2_500_000, 0));
    assert_eq!(pad.copper_offset, Some(nm(100_000, -200_000)));
    assert_eq!(
        pad.drill,
        Some(Drill::Oval {
            width_nm: 1_200_000,
            height_nm: 600_000
        })
    );
    assert_eq!(pad.layers, ["*.Cu", "*.Mask"]);
    let pad = &fp.pads[2];
    assert_eq!(
        (pad.number.as_str(), pad.kind, pad.net),
        ("", PadKind::NonPlatedHole, 0)
    );
    assert_eq!(
        pad.drill,
        Some(Drill::Round {
            diameter_nm: 750_000
        })
    );
    let fp = &board.footprints[1];
    assert_eq!(fp.reference.as_deref(), Some("X2"));
    assert_eq!(
        (fp.layer.as_str(), fp.at.point, fp.at.angle_rad),
        (
            "B.Cu",
            nm(22_000_000, 5_000_000),
            -std::f64::consts::FRAC_PI_4
        )
    );
    assert_eq!(fp.pads[0].at.point, nm(-500_000, 1_250_000));
    assert_eq!(fp.pads[0].at.angle_rad, std::f64::consts::FRAC_PI_4);
    assert_eq!(fp.pads[0].layers, ["B.Cu"]);
}

#[test]
fn manual_contours_and_stackup() {
    let board = track_board();
    assert_eq!(board.thickness_nm, Some(1_600_000));
    let stack = board.stackup.unwrap();
    assert_eq!(stack.len(), 5);
    assert_eq!(stack[0].thickness_nm, Some(35_000));
    assert_eq!(stack[1].thickness_nm, Some(200_000));
    assert_eq!(stack[1].material.as_deref(), Some("Материал эталона"));
    assert_eq!(stack[1].epsilon_r, Some(4.2));
    assert_eq!(stack[1].loss_tangent, Some(0.02));
    assert_eq!(stack[2].name, "In1.Cu");
    assert_eq!(stack[2].thickness_nm, None);
    assert_eq!(stack[2].material, None);
    assert_eq!(stack[3].thickness_nm, Some(17_000));
    assert_eq!(
        board.edges,
        [
            Edge::Rect {
                start: nm(2_000_000, 3_000_000),
                end: nm(24_000_000, 30_000_000)
            },
            Edge::Circle {
                center: nm(7_000_000, 8_000_000),
                end: nm(7_500_000, 8_000_000)
            },
        ]
    );
    assert!(board.footprints[0].edges.is_empty());
    assert_eq!(
        board.footprints[1].edges,
        [Edge::Path(Path::Segment {
            start: nm(-1_000_000, 0),
            end: nm(1_000_000, 0)
        })]
    );
    let [zone] = &board.zones[..] else {
        panic!("ожидалась одна зона")
    };
    assert_eq!(zone.net, 9);
    assert_eq!(zone.layers, ["In1.Cu", "B.Cu"]);
    assert_eq!(
        zone.outlines,
        [
            vec![
                nm(4_000_000, 4_000_000),
                nm(12_000_000, 4_000_000),
                nm(12_000_000, 8_000_000),
                nm(4_000_000, 8_000_000)
            ],
            vec![
                nm(6_000_000, 5_000_000),
                nm(6_000_000, 7_000_000),
                nm(8_000_000, 7_000_000),
                nm(8_000_000, 5_000_000)
            ],
        ]
    );
    let [fill] = &zone.filled[..] else {
        panic!("ожидалась сохранённая заливка одного слоя")
    };
    assert_eq!(fill.layer, "In1.Cu");
    assert!(fill.island);
    assert_eq!(
        fill.points,
        [
            nm(4_250_000, 4_250_000),
            nm(5_750_000, 4_250_000),
            nm(5_750_000, 7_750_000),
            nm(4_250_000, 7_750_000),
        ]
    );
    // Сверка площади прямоугольника по его независимо размеченным сторонам 1,5 × 3,5 мм.
    let edge = |i: usize, j: usize| {
        let [x, y] = fill.points[i].metres();
        let [u, v] = fill.points[j].metres();
        (x - u).hypot(y - v)
    };
    // Допуск по площади при ошибке не более одного нанометра на каждой стороне.
    let area_tolerance = 1e-9 * (0.0015 + 0.0035 + 1e-9);
    assert!((edge(0, 1) * edge(1, 2) - 5.25e-6).abs() < area_tolerance);
}

#[test]
fn empty_v8_has_no_stackup_or_manufacturing_defaults() {
    let board = parse(EMPTY_V8).unwrap();
    assert_eq!(board.version, 20240108);
    assert_eq!(board.layers[1].ordinal, 31);
    assert_eq!(board.thickness_nm, Some(1_600_000));
    assert_eq!(board.stackup, None);
    assert!(board.nets.is_empty());
    assert!(board.edges.is_empty());
    let board = parse(&EMPTY_V8.replace("(thickness 1.6)", "")).unwrap();
    assert_eq!(board.thickness_nm, None);
    assert_eq!(board.stackup, None);
}

#[test]
fn v8_import_keeps_unfilled_outline_and_nanometre_geometry() {
    let board = parse(&EMPTY_V8.replace(
        "(setup (pad_to_mask_clearance 0))",
        r#"(setup (pad_to_mask_clearance 0))
        (segment (start -0.000001 1) (end 1 1) (width 0.2) (layer "B.Cu") (net 0))
        (via (at 1 1) (size 0.8) (drill 0.4) (layers "F.Cu" "B.Cu") (net 0))
        (zone (net 0) (net_name "") (layer "B.Cu") (fill (thermal_gap 0.3))
            (polygon (pts (xy 0 0) (xy 2 0) (xy 2 2) (xy 0 2))))"#,
    ))
    .unwrap();
    assert_eq!(
        board.tracks[0].path,
        Path::Segment {
            start: nm(-1, 1_000_000),
            end: nm(1_000_000, 1_000_000)
        }
    );
    assert_eq!(board.vias[0].layers, ["F.Cu", "B.Cu"]);
    assert_eq!(board.vias[0].net, 0);
    assert_eq!(board.zones[0].outlines[0].len(), 4);
    assert!(board.zones[0].filled.is_empty());
}

fn rejects(old: &str, new: &str, kind: ErrorKind) {
    assert!(BOARD.contains(old), "нет заменяемого фрагмента: {old}");
    let source = BOARD.replacen(old, new, 1);
    let err = parse(&source).expect_err(new);
    assert_eq!(err.kind, kind, "{new}: {err}");
    assert!(err.line > 0 && err.column > 0, "{err}");
}

#[test]
fn rejects_unknown_versions_and_geometry_instead_of_dropping_them() {
    use ErrorKind::{Unsupported, UnsupportedVersion};
    rejects(
        "(version 20241229)",
        "(version 99999999) (new_shape)",
        UnsupportedVersion(99999999),
    );
    rejects(
        "(version 20241229)",
        "(version 20221018)",
        UnsupportedVersion(20221018),
    );
    rejects("(segment", "(teardrop_new", Unsupported);
    rejects(
        "(size 1.2 0.8)",
        "(size 1.2 0.8) (padstack (mode custom))",
        Unsupported,
    );
    rejects("smd roundrect", "smd custom", Unsupported);
    rejects(
        "(via micro",
        "(via micro (remove_unused_layers yes)",
        Unsupported,
    );
    rejects(
        "(zone (net 9)",
        "(zone (keepout (tracks not_allowed)) (net 9)",
        Unsupported,
    );
    rejects("(gr_rect", "(gr_curve", Unsupported);
    rejects(
        "(fp_circle (center 0 0) (end 1 0) (fill no) (layer \"F.SilkS\"))",
        "(fp_circle (center 0 0) (end 1 0) (fill no) (layer \"F.Cu\"))",
        Unsupported,
    );
    rejects(
        "(gr_rect (start",
        "(gr_rect (unknown_shape yes) (start",
        Unsupported,
    );
    rejects(
        "(setup (stackup",
        "(setup (stackup (edge_plating yes)",
        Unsupported,
    );
    rejects(
        "(loss_tangent 0.02)",
        "(loss_tangent 0.02) addsublayer (thickness 0.1)",
        Unsupported,
    );
}

#[test]
fn rejects_degenerate_geometry_and_inconsistent_references() {
    use ErrorKind::{InvalidGeometry, InvalidReference};
    rejects("(end 13.125 24.25)", "(end 10.125 20.25)", InvalidGeometry);
    rejects("(mid 14.125 23.25)", "(mid 14.125 24.25)", InvalidGeometry);
    rejects("(width 0.25)", "(width -0.25)", InvalidGeometry);
    rejects("(width 0.25)", "(width 0.0000001)", InvalidGeometry);
    rejects(
        "(start 10.125 20.25)",
        "(start -2147.483649 0)",
        InvalidGeometry,
    );
    rejects("(drill 0.3)", "(drill 0.6)", InvalidGeometry);
    rejects(
        "(layers \"F.Cu\" \"B.Cu\") (net 5)",
        "(layers \"F.Cu\" \"In1.Cu\") (net 5)",
        InvalidGeometry,
    );
    rejects(
        "(layers \"F.Cu\" \"In1.Cu\")",
        "(layers \"F.Cu\" \"In2.Cu\")",
        InvalidGeometry,
    );
    rejects(
        "(layers \"In1.Cu\" \"In2.Cu\")",
        "(layers \"In2.Cu\" \"In1.Cu\")",
        InvalidGeometry,
    );
    rejects("(xy 12 8) (xy 4 8)", "(xy 8 4)", InvalidGeometry);
    rejects(
        "(roundrect_rratio 0.25)",
        "(roundrect_rratio 0.51)",
        InvalidGeometry,
    );
    rejects("(size 0.75 0.75)", "(size 0.75 0.5)", InvalidGeometry);
    rejects("(drill 0.75)", "", InvalidGeometry);
    rejects(
        "(fill yes (thermal_gap",
        "(fill no (thermal_gap",
        InvalidGeometry,
    );
    rejects("(epsilon_r 4.2)", "(epsilon_r 0)", InvalidGeometry);
    rejects(
        "(layer \"F.Cu\") (net 5)",
        "(layer \"In3.Cu\") (net 5)",
        InvalidReference,
    );
    rejects("(net 5))", "(net 6))", InvalidReference);
    rejects(
        "(net_name \"Опора\")",
        "(net_name \"Другая цепь\")",
        InvalidReference,
    );
    rejects(
        "(filled_polygon (layer \"In1.Cu\")",
        "(filled_polygon (layer \"F.Cu\")",
        InvalidReference,
    );
    rejects(
        "(net 9 \"Опора\")",
        "(net 9 \"Опора\") (net 12 \"Опора\")",
        InvalidReference,
    );
    rejects("(4 \"In1.Cu\"", "(0 \"In1.Cu\"", InvalidReference);
}

#[test]
fn rejects_malformed_input_and_non_finite_data_with_location() {
    use ErrorKind::Syntax;
    rejects("(width 0.25)", "(width 0.25 0.25)", Syntax);
    rejects("(width 0.25)", "(width 0.25) (width 0.5)", Syntax);
    rejects("(width 0.25)", "(width NaN)", Syntax);
    rejects("(at 10.125 20.25 90)", "(at 10.125 20.25 -inf)", Syntax);
    rejects("(epsilon_r 4.2)", "(epsilon_r 1e999)", Syntax);
    rejects(
        "(pad_to_mask_clearance 0)",
        "(pad_to_mask_clearance \"NaN\")",
        Syntax,
    );
    rejects("(useauxorigin false)", "(useauxorigin Infinity)", Syntax);
    let source = EMPTY_V8.replace(
        "(generator pcbnew)",
        "(generator \"Блок\") (version 20240108)",
    );
    let err = parse(&source).unwrap_err();
    assert_eq!(err.kind, Syntax);
    assert_eq!((err.line, err.column), (1, 50));
    assert!(err.to_string().contains("повтор поля `version`"));
    for source in [
        EMPTY_V8.trim_end_matches(')'),
        r#"(kicad_pcb (generator "без закрытия))"#,
        r#"(kicad_pcb (generator "\z"))"#,
        "(kicad_pcb) (kicad_pcb)",
    ] {
        assert_eq!(parse(source).unwrap_err().kind, Syntax, "{source}");
    }
    let deeply_nested = format!("{}{}", "(".repeat(1000), ")".repeat(1000));
    assert_eq!(parse(&deeply_nested).unwrap_err().kind, Syntax);
}
