//! Ручная разметка в миллиметрах. Проверяются геометрия файла и её перевод в СИ.

use sim_circuit::pcb::kicad::{self, Drill, PadKind, ViaKind};

// Подключение в библиотеке относится к задаче 4.9. Здесь собирается тот же модуль.
#[path = "../src/pcb/layout.rs"]
mod layout;

use layout::{Centerline, EdgeContour, Layout, Location, NotAnalyzed, Point};
use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, PI};

const STACK: &str = r#"(stackup
    (layer "F.SilkS" (type "Top Silk Screen"))
    (layer "F.Cu" (type "copper") (thickness 0.035))
    (layer "dielectric 1" (type "prepreg") (thickness 0.2)
      (material "Диэлектрик эталона") (epsilon_r 4.2) (loss_tangent 0.02))
    (layer "In1.Cu" (type "copper") (thickness 0.017))
    (layer "dielectric 2" (type "core") (thickness 0.8) (material "Not specified"))
    (layer "In2.Cu" (type "copper") (thickness 0.017))
    (layer "dielectric 3" (type "prepreg") (thickness 0.2))
    (layer "B.Cu" (type "copper") (thickness 0.035))
    (layer "B.Mask" (type "Bottom Solder Mask") (thickness 0.013))
    (layer "B.SilkS" (type "Bottom Silk Screen")))"#;

const BOARD: &str = r#"(kicad_pcb (version 20241229) (generator pcbnew) (generator_version "9.0")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (4 "In1.Cu" power) (6 "In2.Cu" signal) (2 "B.Cu" signal)
          (1 "F.Mask" user) (3 "B.Mask" user) (5 "F.Paste" user)
          (7 "F.SilkS" user) (9 "B.SilkS" user) (25 "Edge.Cuts" user))
  (setup STACK)
  (net 0 "") (net 1 "А") (net 3 "Опора")
  (footprint "Эталон:Блок" (layer "F.Cu") (at 10 20 90)
    (property "Reference" "X1" (at 0 0 90) (layer "F.SilkS"))
    (pad "1" smd roundrect (at 2 1 45) (size 1.6 0.8) (roundrect_rratio 0.25)
      (layers "F.Cu" "F.Mask" "F.Paste") (net 1 "А"))
    (pad "2" thru_hole oval (at 0 2 90) (size 2 1.6)
      (drill oval 1.2 0.6 (offset 0.2 0.1)) (layers "*.Cu" "*.Mask") (net 3 "Опора"))
    (pad "" np_thru_hole circle (at -1 0) (size 0.7 0.7) (drill 0.5) (layers "F&B.Cu" "*.Mask"))
    (pad "" smd rect (at 0 4) (size 1 1) (layers "F.Mask")))
  (footprint "Эталон:Каретка" (layer "B.Cu") (at 32 14 -45)
    (property "Reference" "X2" (at 0 0 -45) (layer "B.SilkS"))
    (pad "1" connect rect (at 2 -2) (size 2 1) (layers "B.Cu") (net 1 "А"))
    (fp_poly (pts (xy 0 5) (xy 2 5) (xy 2 7) (xy 0 7))
      (stroke (width 0.05) (type default)) (fill none) (layer "Edge.Cuts")))
  (segment (start 11 18) (end 14 22) (width 0.25) (layer "F.Cu") (net 1))
  (arc (start 14 22) (mid 15 23) (end 16 22) (width 0.25) (layer "F.Cu") (net 1))
  (arc (start 21 20) (mid 20 21) (end 20 19) (width 0.3) (layer "In2.Cu") (net 0))
  (via (at 16 22) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1))
  (via micro (at 11 18) (size 0.3) (drill 0.1) (layers "F.Cu" "In1.Cu") (net 3))
  (via blind (at 12 20) (size 0.5) (drill 0.2) (layers "In1.Cu" "In2.Cu") (net 3))
  (via blind (at 18 20) (size 0.5) (drill 0.2) (layers "In1.Cu" "B.Cu") (net 0))
  (gr_rect (start 1 1) (end 45 35) (stroke (width 0.05) (type default)) (fill none) (layer "Edge.Cuts"))
  (gr_circle (center 8 8) (end 8.5 8) (stroke (width 0.05) (type default)) (fill none) (layer "Edge.Cuts"))
  (gr_line (start 25 10) (end 28 10) (stroke (width 0.05) (type default)) (layer "Edge.Cuts"))
  (gr_arc (start 25 10) (mid 26.5 11.5) (end 28 10) (stroke (width 0.05) (type default)) (layer "Edge.Cuts"))
  (zone (net 3) (net_name "Опора") (layers "In1.Cu" "B.Cu") (fill yes)
    (polygon (pts (xy 3 3) (xy 23 3) (xy 23 16) (xy 3 16)))
    (polygon (pts (xy 5 6) (xy 7 6) (xy 7 8) (xy 5 8)))
    (filled_polygon (layer "In1.Cu")
      (pts (xy 4 4) (xy 20 4) (xy 20 15) (xy 4 15) (xy 4 8)
           (xy 5 8) (xy 7 8) (xy 7 6) (xy 5 6) (xy 5 8) (xy 4 8)))
    (filled_polygon (layer "In1.Cu") (island)
      (pts (xy 21 10) (xy 22 10) (xy 22 11) (xy 21 11)))))"#;

fn source() -> String {
    BOARD.replace("STACK", STACK)
}

fn load(source: &str) -> Layout {
    Layout::from_kicad(&kicad::parse(source).expect("разобрать файл")).expect("построить геометрию")
}

fn nm(x_nm: i32, y_nm: i32) -> Point {
    Point { x_nm, y_nm }
}

fn mm(x: i32, y: i32) -> Point {
    nm(x * 1_000_000, y * 1_000_000) // По СИ в 1 мм содержится 10⁶ нм.
}

fn near(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "получено {actual}, ожидалось {expected}"
    );
}

// Предел сравнения длины соответствует сетке 1 нм. Для угла на радиусе 1 мм
// одному нанометру дуги соответствует 1 мкрад.
const LENGTH_TOL: f64 = 1e-9;
const ANGLE_TOL: f64 = 1e-6;

#[test]
fn layer_stack_and_vertical_spans() {
    let source = source();
    let board = load(&source);
    assert_eq!(board.format_version, 20241229);
    assert_eq!(board.generator_version.as_deref(), Some("9.0"));
    assert_eq!(board, load(&source));
    assert_eq!(board.copper_layers, ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]);
    assert_eq!((board.nets.len(), board.nets[&3].as_str()), (2, "Опора"));
    let stack = board.stackup().unwrap();
    assert_eq!(stack.layers().len(), 10);
    assert_eq!(stack.layers()[0].kind, "Top Silk Screen");
    near(board.nominal_thickness_m.unwrap(), 0.0016, LENGTH_TOL);
    near(
        stack.copper_layer("F.Cu").unwrap().thickness_m().unwrap(),
        35e-6,
        LENGTH_TOL,
    );
    near(
        stack.copper_layer("In1.Cu").unwrap().thickness_m().unwrap(),
        17e-6,
        LENGTH_TOL,
    );
    let dielectric = stack.dielectrics_between("In2.Cu", "F.Cu").unwrap();
    assert_eq!(
        dielectric
            .iter()
            .map(|layer| layer.name.as_str())
            .collect::<Vec<_>>(),
        ["dielectric 1", "dielectric 2"]
    );
    assert_eq!(dielectric[0].material(), Ok("Диэлектрик эталона"));
    assert_eq!(dielectric[0].epsilon_r(), Ok(4.2));
    assert_eq!(dielectric[0].loss_tangent(), Ok(0.02));
    assert_eq!(
        dielectric[1].material(),
        Err(NotAnalyzed::MissingMaterial("dielectric 2".into()))
    );
    assert_eq!(
        dielectric[1].epsilon_r(),
        Err(NotAnalyzed::MissingPermittivity("dielectric 2".into()))
    );
    assert_eq!(
        dielectric[1].loss_tangent(),
        Err(NotAnalyzed::MissingLossTangent("dielectric 2".into()))
    );
    let expected: &[(ViaKind, Point, &[&str], f64, Option<u32>)] = &[
        // 1 200 мкм диэлектрика + 34 мкм внутренних слоёв + 35 мкм двух полутолщин.
        (
            ViaKind::Through,
            mm(16, 22),
            &["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"],
            1_269e-6,
            Some(1),
        ),
        // 200 мкм диэлектрика + 17,5 мкм + 8,5 мкм.
        (
            ViaKind::Micro,
            mm(11, 18),
            &["F.Cu", "In1.Cu"],
            226e-6,
            Some(3),
        ),
        (
            ViaKind::BlindBuried,
            mm(12, 20),
            &["In1.Cu", "In2.Cu"],
            817e-6,
            Some(3),
        ),
        (
            ViaKind::BlindBuried,
            mm(18, 20),
            &["In1.Cu", "In2.Cu", "B.Cu"],
            1_043e-6,
            None,
        ),
    ];
    assert_eq!(board.vias.len(), expected.len());
    for (via, &(kind, at, layers, distance, net)) in board.vias.iter().zip(expected) {
        assert_eq!((via.kind, via.at, via.net), (kind, at, net));
        assert_eq!(via.layers, layers);
        near(
            stack
                .copper_center_distance_m(via.layers.first().unwrap(), via.layers.last().unwrap())
                .unwrap(),
            distance,
            LENGTH_TOL,
        );
    }
    near(board.vias[0].diameter_m, 0.0006, LENGTH_TOL);
    near(board.vias[0].drill_m, 0.0003, LENGTH_TOL);

    let v8 = load(
        &source
            .replace("20241229", "20240108")
            .replace("\"9.0\"", "\"8.0\"")
            .replace("(4 \"In1.Cu\"", "(1 \"In1.Cu\"")
            .replace("(6 \"In2.Cu\"", "(2 \"In2.Cu\"")
            .replace("(2 \"B.Cu\"", "(31 \"B.Cu\"")
            .replace("(1 \"F.Mask\"", "(39 \"F.Mask\""),
    );
    assert_eq!(v8.format_version, 20240108);
    assert_eq!(v8.copper_layers, board.copper_layers);
    assert_eq!(v8.vias, board.vias);
    assert_eq!(v8.footprints, board.footprints);
    assert_eq!(v8.zones, board.zones);
}

#[test]
fn dimensions_and_angles_in_board_axes() {
    let board = load(&source());
    let [front, back] = board.footprints.as_slice() else {
        panic!("два посадочных места")
    };
    assert_eq!(front.reference.as_deref(), Some("X1"));
    assert_eq!(front.library_id, "Эталон:Блок");
    assert_eq!((front.origin, front.angle_rad), (mm(10, 20), FRAC_PI_2));
    // Поворот (2, 1) на +90° даёт (1, −2). Угол площадки остаётся 45°.
    let smd = &front.pads[0];
    assert_eq!(
        (smd.number.as_str(), smd.anchor, smd.net),
        ("1", mm(11, 18), Some(1))
    );
    let land = smd.land.as_ref().unwrap();
    assert_eq!(land.layers, ["F.Cu"]);
    assert_eq!(land.outline.center, smd.anchor);
    near(land.outline.angle_rad, PI / 4.0, ANGLE_TOL);
    near(land.outline.width_m, 0.0016, LENGTH_TOL);
    near(land.outline.height_m, 0.0008, LENGTH_TOL);
    near(land.outline.radius_m, 0.0002, LENGTH_TOL);
    assert_eq!(smd.hole, None);

    let thru = &front.pads[1];
    let hole = thru.hole.unwrap();
    let land = thru.land.as_ref().unwrap();
    assert_eq!(
        (thru.kind, thru.anchor, hole.center),
        (PadKind::PlatedHole, mm(12, 20), mm(12, 20))
    );
    assert_eq!(land.layers, board.copper_layers);
    // Смещение меди (0,2; 0,1) при 90° становится (0,1; −0,2) мм.
    assert_eq!(land.outline.center, nm(12_100_000, 19_800_000));
    assert_eq!(hole.angle_rad, FRAC_PI_2);
    near(hole.width_m, 0.0012, LENGTH_TOL);
    near(hole.height_m, 0.0006, LENGTH_TOL);
    near(land.outline.radius_m, 0.0008, LENGTH_TOL);

    let unplated = &front.pads[2];
    assert_eq!(
        (unplated.kind, unplated.anchor, unplated.net),
        (PadKind::NonPlatedHole, mm(10, 21), None)
    );
    assert_eq!(unplated.land.as_ref().unwrap().layers, ["F.Cu", "B.Cu"]);
    near(unplated.hole.unwrap().radius_m, 0.00025, LENGTH_TOL);
    assert!(front.pads[3].land.is_none());

    assert_eq!(
        (back.reference.as_deref(), back.layer.as_str()),
        (Some("X2"), "B.Cu")
    );
    assert_eq!((back.origin, back.angle_rad), (mm(32, 14), -PI / 4.0));
    let land = back.pads[0].land.as_ref().unwrap();
    // Вектор (2, −2) после −45° уже лежит на оси X. Второго отражения нет.
    near(
        land.outline.center.metres()[0],
        (32.0 + 8.0_f64.sqrt()) * 1e-3,
        LENGTH_TOL,
    );
    assert_eq!(land.outline.center.y_nm, 14_000_000);
    assert_eq!(land.outline.angle_rad, 0.0);
    assert_eq!(land.layers, ["B.Cu"]);
}

#[test]
fn exact_track_endpoints_and_analytic_arc_lengths() {
    let board = load(&source());
    let [segment, semicircle, major] = board.tracks.as_slice() else {
        panic!("три дорожки")
    };
    assert_eq!(
        segment.path,
        Centerline::Segment {
            start: mm(11, 18),
            end: mm(14, 22)
        }
    );
    assert_eq!((segment.layer.as_str(), segment.net), ("F.Cu", Some(1)));
    near(segment.width_m, 0.00025, LENGTH_TOL);
    near(segment.path.length_m(), 0.005, LENGTH_TOL); // Треугольник 3–4–5 мм.
    for (track, expected) in [
        (semicircle, (mm(15, 22), -PI)),
        (major, (mm(20, 20), 3.0 * PI / 2.0)),
    ] {
        let Centerline::Arc {
            start,
            mid,
            end,
            center_m,
            radius_m,
            sweep_rad,
        } = track.path
        else {
            panic!("ожидалась дуга")
        };
        let [x, y] = expected.0.metres();
        near(center_m[0], x, LENGTH_TOL);
        near(center_m[1], y, LENGTH_TOL);
        near(radius_m, 0.001, LENGTH_TOL);
        near(sweep_rad, expected.1, ANGLE_TOL);
        near(track.path.length_m(), expected.1.abs() * 0.001, LENGTH_TOL);
        for point in [start, mid, end] {
            let [a, b] = point.metres();
            near((a - x).hypot(b - y), 0.001, LENGTH_TOL);
        }
    }
    assert_eq!((major.layer.as_str(), major.net), ("In2.Cu", None));
    let mut raw = kicad::parse(&source()).unwrap();
    raw.tracks[0].path = kicad::Path::Arc {
        start: nm(-2_000_000_000, 0),
        mid: nm(0, 1),
        end: nm(2_000_000_000, 0),
    };
    // Контроль почти прямой дуги с прогибом 1 нм на хорде 4 м.
    let arc = Layout::from_kicad(&raw).unwrap().tracks[0].path;
    near(arc.length_m(), 4.0, LENGTH_TOL);
}

#[test]
fn explicit_and_persisted_cutouts() {
    let board = load(&source());
    let [zone] = board.zones.as_slice() else {
        panic!("одна зона")
    };
    assert_eq!(zone.net, Some(3));
    assert_eq!(zone.layers(), ["In1.Cu", "B.Cu"]);
    let [cutout] = zone.design_outline.holes() else {
        panic!("один вырез зоны")
    };
    // Внешний прямоугольник 20 × 13 мм, вырез 2 × 2 мм.
    near(
        zone.design_outline.outline().area_m2(),
        260e-6,
        20e-3 * LENGTH_TOL,
    );
    near(cutout.area_m2(), 4e-6, 2e-3 * LENGTH_TOL);
    near(zone.design_outline.area_m2(), 256e-6, 26e-3 * LENGTH_TOL);
    assert_eq!(zone.design_outline.locate(mm(6, 7)), Location::Outside);
    assert_eq!(zone.design_outline.locate(mm(5, 7)), Location::Boundary);
    assert_eq!(zone.design_outline.locate(mm(4, 7)), Location::Inside);

    let [fill, island] = zone.fill("In1.Cu").unwrap() else {
        panic!("две области заливки")
    };
    assert!(!fill.island && island.island);
    assert_eq!(fill.polygon.holes().len(), 1);
    // Сохранённая заливка 16 × 11 мм с вырезом 2 × 2 мм и отдельно 1 мм² острова.
    near(fill.polygon.area_m2(), 172e-6, 27e-3 * LENGTH_TOL);
    near(island.polygon.area_m2(), 1e-6, 2e-3 * LENGTH_TOL);
    assert_eq!(fill.polygon.locate(mm(6, 7)), Location::Outside);
    assert_eq!(fill.polygon.locate(mm(5, 7)), Location::Boundary);
    // Искусственный мост шёл по y = 8; после восстановления он не является границей.
    assert_eq!(
        fill.polygon.locate(nm(4_500_000, 8_000_000)),
        Location::Inside
    );
    assert_eq!(fill.polygon.locate(mm(3, 7)), Location::Outside);
    assert_eq!(
        zone.fill("B.Cu"),
        Err(NotAnalyzed::MissingFill("B.Cu".into()))
    );
}

#[test]
fn board_contours_keep_polygon_circle_and_reversed_arc() {
    let board = load(&source());
    let [
        EdgeContour::Polygon(outer),
        EdgeContour::Circle {
            center,
            end,
            radius_m,
        },
        EdgeContour::Polygon(local),
        EdgeContour::Path(slot),
    ] = board.edge_cuts().unwrap()
    else {
        panic!("внешняя граница и три отдельных выреза")
    };
    assert_eq!(outer.points().len(), 4);
    near(outer.area_m2(), 44e-3 * 34e-3, 78e-3 * LENGTH_TOL);
    assert_eq!((*center, *end), (mm(8, 8), nm(8_500_000, 8_000_000)));
    near(*radius_m, 0.0005, LENGTH_TOL);
    near(local.area_m2(), 4e-6, 8e-3 * LENGTH_TOL);
    let outer_bounds = [mm(1, 1), mm(45, 35)];
    assert_eq!([outer.points()[0], outer.points()[2]], outer_bounds);
    let [x, y] = local.points()[0].metres();
    // Первая вершина выреза (0, 5) мм уходит на равные части диагонали вверх по числам X и Y.
    near(x, (32.0 - 5.0 * FRAC_1_SQRT_2) * 1e-3, LENGTH_TOL);
    near(y, (14.0 + 5.0 * FRAC_1_SQRT_2) * 1e-3, LENGTH_TOL);
    assert_eq!(outer.locate(local.points()[0]), Location::Inside);
    let [
        Centerline::Segment { start: a, end: b },
        Centerline::Arc {
            start,
            mid,
            end,
            sweep_rad,
            radius_m,
            ..
        },
    ] = slot.as_slice()
    else {
        panic!("вырез замыкается дугой")
    };
    assert_eq!(
        (*a, *b, *start, *end, *mid),
        (
            mm(25, 10),
            mm(28, 10),
            mm(28, 10),
            mm(25, 10),
            nm(26_500_000, 11_500_000)
        )
    );
    near(*radius_m, 0.0015, LENGTH_TOL);
    near(*sweep_rad, PI, ANGLE_TOL);
}

#[test]
fn missing_data_stays_open() {
    let source = source();
    let absent = load(&source.replace(STACK, ""));
    assert_eq!(absent.nominal_thickness_m, Some(1.6e-3));
    assert_eq!(absent.stackup(), Err(NotAnalyzed::MissingStackup));
    let partial = load(
        &source
            .replace(
                r#"(layer "In1.Cu" (type "copper") (thickness 0.017))"#,
                r#"(layer "In1.Cu" (type "copper"))"#,
            )
            .replace(
                r#"(layer "dielectric 3" (type "prepreg") (thickness 0.2))"#,
                "",
            ),
    );
    let stack = partial.stackup().unwrap();
    assert_eq!(
        stack.copper_center_distance_m("F.Cu", "In2.Cu"),
        Err(NotAnalyzed::MissingThickness("In1.Cu".into()))
    );
    assert_eq!(
        stack.dielectrics_between("In2.Cu", "B.Cu"),
        Err(NotAnalyzed::MissingStackLayer("dielectric 3".into()))
    );
    near(
        stack.copper_center_distance_m("In2.Cu", "In2.Cu").unwrap(),
        0.0,
        0.0,
    );

    let empty = load(
        r#"(kicad_pcb (version 20240108) (generator pcbnew) (general)
                     (layers (0 "F.Cu" signal) (31 "B.Cu" signal)) (setup))"#,
    );
    assert_eq!(empty.nominal_thickness_m, None);
    assert_eq!(empty.edge_cuts(), Err(NotAnalyzed::MissingEdgeCuts));
    assert_eq!(
        empty.polygonal_substrate(),
        Err(NotAnalyzed::MissingEdgeCuts)
    );
    assert!(empty.tracks.is_empty() && empty.zones.is_empty());
}

fn rejects(raw: &kicad::KiCadBoard, item: &str, message: &str) {
    let error = Layout::from_kicad(raw).unwrap_err();
    assert!(error.item.contains(item), "{error}");
    assert!(error.message.contains(message), "{error}");
}

#[test]
fn rejects_malformed_and_ambiguous_polygons() {
    let original = kicad::parse(&source()).unwrap();
    let mut raw = original.clone();
    raw.zones[0].outlines[1] = vec![mm(3, 6), mm(7, 6), mm(7, 8), mm(3, 8)];
    rejects(&raw, "зона 1", "вырез пересекает");

    let mut raw = original.clone();
    raw.zones[0].outlines[0] = vec![mm(3, 3), mm(23, 16), mm(3, 16), mm(21, 3)];
    rejects(&raw, "зона 1", "самопересечение");

    let mut raw = original.clone();
    raw.zones[0].filled[0].points = vec![
        mm(4, 4),
        mm(20, 4),
        mm(20, 15),
        mm(4, 15),
        mm(4, 8),
        mm(5, 8),
        mm(4, 8),
    ];
    rejects(&raw, "зона 1", "мост заливки");
    let mut raw = original;
    raw.zones[0].filled[0].points[5..10].reverse();
    rejects(&raw, "зона 1", "одинаковое направление");
}

#[test]
fn rejects_invalid_geometry_and_nonfinite_input() {
    let original = kicad::parse(&source()).unwrap();
    let mut raw = original.clone();
    raw.stackup.as_mut().unwrap().swap(2, 3);
    rejects(&raw, "setup/stackup", "физическом порядке");
    let mut raw = original.clone();
    raw.stackup.as_mut().unwrap()[2].epsilon_r = Some(f64::NAN);
    rejects(&raw, "setup/stackup", "параметр материала");
    let mut raw = original.clone();
    raw.footprints[0].at.angle_rad = f64::INFINITY;
    rejects(&raw, "посадочное место 1", "конечным");
    let mut raw = original.clone();
    raw.footprints[0].at.point.x_nm = i32::MAX;
    rejects(&raw, "площадка 1", "за диапазон");
    let mut raw = original.clone();
    raw.footprints[0].pads[0].shape = kicad::PadShape::RoundRect {
        radius_ratio: f64::NAN,
    };
    rejects(&raw, "площадка 1", "доля скругления");
    let mut raw = original.clone();
    raw.footprints[0].pads[1].drill = Some(Drill::Round { diameter_nm: 0 });
    rejects(&raw, "площадка 2", "положительным");
    let mut raw = original.clone();
    raw.vias[1].layers[1] = "In2.Cu".into();
    rejects(&raw, "переход 2", "пара слоёв");
    let mut raw = original.clone();
    raw.vias[0].net = 2;
    rejects(&raw, "переход 1", "необъявленную цепь");
    let mut raw = original.clone();
    let kicad::Path::Arc { start, end, .. } = raw.tracks[1].path else {
        panic!("дуга")
    };
    raw.tracks[1].path = kicad::Path::Arc {
        start,
        mid: mm(15, 22),
        end,
    };
    rejects(&raw, "дорожка 2", "вырожденная дуга");
    let mut raw = original;
    raw.edges.pop();
    rejects(&raw, "Edge.Cuts", "разомкнута");
}

#[test]
fn polygonal_substrate_preserves_nested_cutouts() {
    assert_eq!(
        load(&source()).polygonal_substrate(),
        Err(NotAnalyzed::CurvedEdgeTopology)
    );
    let mut raw = kicad::parse(&source()).unwrap();
    raw.footprints[1].edges.clear();
    // Ручная карта: две несвязанные области материала. В первой вырез 6 × 6 мм
    // и внутри него отдельный блок 2 × 2 мм. Порядок и направления перемешаны.
    let rect = |start, end| kicad::Edge::Rect { start, end };
    raw.edges = vec![
        rect(mm(2, 2), mm(8, 8)),
        rect(mm(10, 10), mm(0, 0)),
        rect(mm(6, 4), mm(4, 6)),
        rect(mm(12, 1), mm(15, 5)),
    ];
    let board = Layout::from_kicad(&raw).unwrap();
    let regions = board.polygonal_substrate().unwrap();
    let [main, island, separate] = regions.as_slice() else {
        panic!("три области материала")
    };
    assert_eq!(main.holes().len(), 1);
    near(main.area_m2(), 64e-6, 32e-3 * LENGTH_TOL);
    near(island.area_m2(), 4e-6, 4e-3 * LENGTH_TOL);
    near(separate.area_m2(), 12e-6, 7e-3 * LENGTH_TOL);
    assert_eq!(main.locate(mm(1, 1)), Location::Inside);
    assert_eq!(main.locate(mm(2, 3)), Location::Boundary);
    assert_eq!(main.locate(mm(5, 5)), Location::Outside);
    assert_eq!(island.locate(mm(5, 5)), Location::Inside);

    raw.edges.push(rect(mm(9, 8), mm(11, 9)));
    rejects(&raw, "Edge.Cuts", "границы платы пересекаются");
}
