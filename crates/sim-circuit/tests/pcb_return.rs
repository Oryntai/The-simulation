//! Координаты, фронт и пределы эталонной пары взяты из depth_t4/expected.md.

use sim_circuit::pcb::{
    CU_RHO_20C, FOIL_M_PER_OZ, IpcLayer, MIL, ipc2221_current, ipc2221_delta_t, kicad,
    trace_resistance, via_resistance,
};

// Подключение к публичному pcb.rs предусмотрено задачей 4.9.
#[allow(dead_code)]
#[path = "../src/pcb/layout.rs"]
mod layout;
#[allow(dead_code)]
#[path = "../src/pcb/ratings.rs"]
mod ratings;
#[path = "../src/pcb/return_path.rs"]
mod return_path;

use layout::{Layout, NotAnalyzed, Point};
use ratings::{Datum, Node, Scope, Source};
use return_path::*;

const UNSAFE: &str = include_str!("../../../devices/electrical/depth_t4/unsafe.kicad_pcb");
const FIXED: &str = include_str!("../../../devices/electrical/depth_t4/fixed.kicad_pcb");
const REFERENCE: Source<'static> = Source::Reference("devices/electrical/depth_t4/expected.md");
const BLOCK: Source<'static> = Source::Reference("Ручная геометрия нейтральной постановки в тесте");
const FRONT: &str = "F.Cu";
const BACK: &str = "B.Cu";

fn mm(x: i32, y: i32) -> Point {
    // По определению 1 мм = 10⁶ нм.
    Point {
        x_nm: x * 1_000_000,
        y_nm: y * 1_000_000,
    }
}

fn node(at: Point, layer: &str) -> Node<'_> {
    Node { at, layer }
}

fn datum(value: f64, source: Source<'_>) -> Option<Datum<'_>> {
    Some(Datum { value, source })
}

fn segment(start: Point, end: Point, layer: &str, net: u32, width_nm: i32) -> kicad::Track {
    kicad::Track {
        path: kicad::Path::Segment { start, end },
        layer: layer.into(),
        net,
        width_nm,
    }
}

fn load(text: &str) -> Layout {
    Layout::from_kicad(&kicad::parse(text).unwrap()).unwrap()
}

fn near(actual: f64, expected: f64) {
    // Допуск для ручной формулы на той же геометрии и в тех же единицах:
    // округление операций f64. Нулевые результаты проверяются отдельно.
    assert!(
        (actual - expected).abs() <= 1e-12 * expected.abs().max(1e-6),
        "{actual:.14e} вместо {expected:.14e}"
    );
}

fn reference() -> ReferenceInput<'static> {
    ReferenceInput {
        scope: Scope::Reference,
        track: 1,
        plane: Plane {
            zone: 0,
            layer: BACK,
            tie: mm(5, 9),
            connection: Some(REFERENCE),
        },
    }
}

#[test]
fn reference_pair_distinguishes_break_from_connected_plane() {
    let input = reference();
    let bad = check_reference(&load(UNSAFE), &input).unwrap();
    assert_eq!(bad.status, Status::Fail);
    near(bad.height_m.unwrap(), (1.510 + 0.035) * 1e-3);
    let center = bad.center_gaps_m.unwrap();
    assert_eq!(center.len(), 1);
    // Координаты выреза 27…29 мм и начало трассы в 22 мм.
    near(center[0][0], 0.005);
    near(center[0][1], 0.007);
    let copper = bad.copper_gaps_m.unwrap();
    // У самого края выреза радиус 0,2 мм выступает за ось также по X.
    assert_eq!(copper.len(), 1);
    near(copper[0][0], 0.0048);
    near(copper[0][1], 0.0072);

    let good = check_reference(&load(FIXED), &input).unwrap();
    assert_eq!(good.status, Status::Pass);
    assert_eq!(good.center_gaps_m, Some(vec![]));
    assert_eq!(good.copper_gaps_m, Some(vec![]));
}

#[test]
fn partial_width_and_arc_gaps_are_not_hidden_by_centerline_or_chord() {
    let mut board = kicad::parse(UNSAFE).unwrap();
    // Ось на 0,1 мм ниже конца выреза, край меди заходил бы в него ещё на 0,1 мм.
    let at = |x| Point {
        y_nm: 5_350_000,
        ..mm(x, 5)
    };
    board.tracks[1].path = kicad::Path::Segment {
        start: at(22),
        end: at(32),
    };
    let partial = check_reference(&Layout::from_kicad(&board).unwrap(), &reference()).unwrap();
    assert_eq!(partial.status, Status::Fail);
    assert_eq!(partial.center_gaps_m, Some(vec![]));
    let intervals = partial.copper_gaps_m.unwrap();
    assert_eq!(intervals.len(), 1);
    let corner = 1e-3 * (0.2_f64.powi(2) - 0.1_f64.powi(2)).sqrt();
    near(intervals[0][0], 0.005 - corner);
    near(intervals[0][1], 0.007 + corner);

    // Полуокружность радиусом 2 мм, хорда целиком ниже выреза.
    board.tracks[1].path = kicad::Path::Arc {
        start: mm(25, 6),
        mid: mm(27, 4),
        end: mm(29, 6),
    };
    let arc = check_reference(&Layout::from_kicad(&board).unwrap(), &reference()).unwrap();
    assert_eq!(arc.status, Status::Fail);
    let center = arc.center_gaps_m.unwrap();
    assert_eq!(center.len(), 1);
    near(center[0][0], 0.002 * std::f64::consts::FRAC_PI_2);
    near(
        center[0][1],
        0.002 * (std::f64::consts::PI - (0.75_f64 / 2.0).asin()),
    );
}

#[test]
fn board_cutout_overrides_stale_fill() {
    let mut board = kicad::parse(FIXED).unwrap();
    // Прямоугольный вырез 27…29 × 4…6 мм под дорожкой с осью y = 5 мм.
    board.edges.push(kicad::Edge::Rect {
        start: mm(27, 4),
        end: mm(29, 6),
    });
    let report = check_reference(&Layout::from_kicad(&board).unwrap(), &reference()).unwrap();
    assert_eq!(report.status, Status::Fail);
    let center = report.center_gaps_m.unwrap();
    assert_eq!(center.len(), 1);
    near(center[0][1] - center[0][0], 0.002);
}

#[test]
fn absent_stack_material_fill_and_unverified_connection_remain_open() {
    let raw = kicad::parse(FIXED).unwrap();
    for (board, reason) in [
        (
            {
                let mut result = raw.clone();
                result.stackup = None;
                result
            },
            NotAnalyzed::MissingStackup,
        ),
        (
            {
                let mut result = raw.clone();
                result
                    .stackup
                    .as_mut()
                    .unwrap()
                    .iter_mut()
                    .find(|l| l.name == BACK)
                    .unwrap()
                    .thickness_nm = None;
                result
            },
            NotAnalyzed::MissingThickness(BACK.into()),
        ),
        (
            {
                let mut result = raw.clone();
                result
                    .stackup
                    .as_mut()
                    .unwrap()
                    .iter_mut()
                    .find(|l| l.name == "dielectric 1")
                    .unwrap()
                    .material = None;
                result
            },
            NotAnalyzed::MissingMaterial("dielectric 1".into()),
        ),
        (
            {
                let mut result = raw.clone();
                result.zones[0].filled.clear();
                result
            },
            NotAnalyzed::MissingFill(BACK.into()),
        ),
    ] {
        let outcome = check_reference(&Layout::from_kicad(&board).unwrap(), &reference()).unwrap();
        assert_eq!(outcome.status, Status::NotAnalyzed(Open::Layout(reason)));
    }
    let layout = load(FIXED);
    let input = ReferenceInput {
        scope: Scope::Assembly,
        ..reference()
    };
    assert!(matches!(
        check_reference(&layout, &input).unwrap().status,
        Status::NotAnalyzed(Open::Evidence(_))
    ));
    let mut layout = layout;
    layout.tracks[0].width_m = f64::NAN;
    assert!(check_reference(&layout, &input).is_err());
}

#[test]
fn covered_projection_behind_intervening_copper_is_not_proved() {
    let mut board = kicad::parse(FIXED).unwrap();
    let stack = board.stackup.as_mut().unwrap();
    let foil = stack[1].clone();
    let dielectric = stack[2].clone();
    // Два дополнительных внутренних слоя из того же материала и толщины.
    for i in 1..=2 {
        let name = format!("In{i}.Cu");
        board.layers.insert(
            i,
            kicad::Layer {
                ordinal: i as u32,
                name: name.clone(),
                kind: "signal".into(),
                user_name: None,
            },
        );
        stack.insert(
            2 * i + 1,
            kicad::StackLayer {
                name,
                ..foil.clone()
            },
        );
        stack.insert(
            2 * i + 2,
            kicad::StackLayer {
                name: format!("dielectric {}", i + 1),
                ..dielectric.clone()
            },
        );
    }
    let result = check_reference(&Layout::from_kicad(&board).unwrap(), &reference()).unwrap();
    assert_eq!(result.status, Status::NotAnalyzed(Open::InterveningLayers));
    assert_eq!(result.copper_gaps_m, Some(vec![]));
}

fn step(index: usize, to: Point) -> Step<'static> {
    Step::Track { index, to }
}

fn bounce<'a>(power: &'a [Step<'a>], signal: &'a [Step<'a>]) -> GroundBounceInput<'a> {
    let star = node(mm(5, 23), FRONT);
    GroundBounceInput {
        scope: Scope::Reference,
        power: Route {
            net: 5,
            start: star,
            steps: power,
        },
        signal: Route {
            net: 5,
            start: star,
            steps: signal,
        },
        topology: Some(REFERENCE),
        resistivity_ohm_m: datum(1.68e-8, REFERENCE),
        plating_m: &[],
        shared_inductance_h: datum(10e-9, REFERENCE),
        current_a: datum(1.0, REFERENCE),
        slew_a_per_s: datum(1.0 / 100e-9, REFERENCE),
        max_bounce_v: datum(0.050, REFERENCE),
        // В эталоне неучтённые вклады исключены постановкой R–L.
        omitted_bounce_v: datum(0.0, REFERENCE),
    }
}

#[test]
fn ground_bounce_matches_manual_rl_and_separate_star() {
    assert_eq!(HAZARD_CODE, "ground_bounce");
    // Индексы отличаются из-за перемычки PROBE_A в дефектном файле.
    let bad_power = [step(12, mm(10, 23)), step(13, mm(15, 23))];
    let bad_signal = [step(12, mm(10, 23)), step(14, mm(10, 28))];
    let unsafe_report =
        check_ground_bounce(&load(UNSAFE), &bounce(&bad_power, &bad_signal)).unwrap();
    assert_eq!(unsafe_report.status, Status::Fail);
    near(unsafe_report.shared_length_m.unwrap(), 0.005);
    let ohm = 1.68e-8 * 0.005 / (0.0005 * 0.000035);
    near(unsafe_report.resistance_ohm.unwrap(), ohm);
    near(
        unsafe_report.bounce_v.unwrap(),
        ohm * 1.0 + 10e-9 * (1.0 / 100e-9),
    );
    near(unsafe_report.bounds_v.unwrap()[0], 0.1048);

    let power = [step(11, mm(10, 23)), step(12, mm(15, 23))];
    let signal = [step(13, mm(5, 28)), step(14, mm(10, 28))];
    let mut input = bounce(&power, &signal);
    input.shared_inductance_h = None;
    let fixed = load(FIXED);
    let safe = check_ground_bounce(&fixed, &input).unwrap();
    assert_eq!(safe.status, Status::Pass);
    assert_eq!(safe.shared_length_m, Some(0.0));
    assert_eq!(safe.bounce_v, Some(0.0));

    input.omitted_bounce_v = None;
    let open = check_ground_bounce(&fixed, &input).unwrap();
    assert!(matches!(
        open.status,
        Status::NotAnalyzed(Open::Evidence(_))
    ));
    assert_eq!(open.bounce_v, Some(0.0));
    input.scope = Scope::Assembly;
    assert!(matches!(
        check_ground_bounce(&fixed, &input).unwrap().status,
        Status::NotAnalyzed(Open::Evidence(_))
    ));
}

#[test]
fn cutout_between_healthy_projection_and_plane_tie_leaves_connectivity_open() {
    let mut board = kicad::parse(FIXED).unwrap();
    // Вырез x = 19…20 мм пересекает всю ширину заливки по Y. Под дорожкой
    // x = 22…32 мм медь ещё есть, но привязка в (5;9) находится с другой стороны.
    board.edges.push(kicad::Edge::Rect {
        start: mm(19, 2),
        end: mm(20, 11),
    });
    let result = check_reference(&Layout::from_kicad(&board).unwrap(), &reference()).unwrap();
    assert_eq!(result.status, Status::NotAnalyzed(Open::ClippedPlane));
    assert_eq!(result.copper_gaps_m, Some(vec![]));
}

#[test]
fn ground_bounce_limits_uncertainty_and_nonfinite_data() {
    let power = [step(12, mm(10, 23)), step(13, mm(15, 23))];
    let signal = [step(12, mm(10, 23)), step(14, mm(10, 28))];
    let mut input = bounce(&power, &signal);
    let board = load(UNSAFE);
    for replace in [
        None,
        datum(10e-9, Source::Estimate("Оценка вместо параметра опыта")),
    ] {
        input.shared_inductance_h = replace;
        assert!(matches!(
            check_ground_bounce(&board, &input).unwrap().status,
            Status::NotAnalyzed(Open::Evidence(_))
        ));
    }
    input.shared_inductance_h = datum(10e-9, REFERENCE);
    input.omitted_bounce_v = datum(0.100, BLOCK);
    // 104,8 ± 100 мВ пересекает предел 50 мВ.
    assert_eq!(
        check_ground_bounce(&board, &input).unwrap().status,
        Status::NotAnalyzed(Open::Uncertainty)
    );
    input.omitted_bounce_v = datum(0.0, REFERENCE);
    input.slew_a_per_s = datum(-1.0 / 100e-9, BLOCK);
    let negative = check_ground_bounce(&board, &input).unwrap();
    assert_eq!(negative.status, Status::Fail);
    near(negative.bounce_v.unwrap(), -0.0952);
    for invalid in [f64::INFINITY, f64::NAN] {
        input.max_bounce_v = datum(invalid, REFERENCE);
        assert!(check_ground_bounce(&board, &input).is_err());
    }
}

fn transition_board() -> kicad::KiCadBoard {
    let mut board = kicad::parse(FIXED).unwrap();
    board.tracks = board.tracks[7..9].to_vec(); // LINK из ручного эталона.
    board.footprints.clear();
    board.vias.truncate(1);
    // Пара новых плоскостей рядом с сигнальным переходом (10;13), вне его меди.
    let outline = vec![mm(3, 14), mm(17, 14), mm(17, 20), mm(3, 20)];
    board.zones = [BACK, FRONT]
        .into_iter()
        .map(|layer| kicad::Zone {
            net: 5,
            layers: vec![layer.into()],
            outlines: vec![outline.clone()],
            filled: vec![kicad::FilledPolygon {
                layer: layer.into(),
                points: outline.clone(),
                island: false,
            }],
        })
        .collect();
    board.vias.push(kicad::Via {
        at: mm(10, 15),
        net: 5,
        ..board.vias[0].clone()
    });
    board
}

fn transition() -> TransitionInput<'static> {
    TransitionInput {
        scope: Scope::Reference,
        tracks: [0, 1],
        signal_via: 0,
        planes: [BACK, FRONT].map(|layer| Plane {
            zone: usize::from(layer == FRONT),
            layer,
            tie: mm(10, 17),
            connection: Some(BLOCK),
        }),
        transfer: Some(Transfer::StitchVia(1)),
        // Порог между отступами 2 мм (10;15) и 5 мм (10;18) от (10;13).
        max_detour_m: datum(0.003, BLOCK),
    }
}

#[test]
fn transition_requires_local_annular_contact_with_both_planes() {
    let mut board = transition_board();
    let input = transition();
    let correct = check_transition(&Layout::from_kicad(&board).unwrap(), &input).unwrap();
    assert_eq!(correct.status, Status::Pass);
    near(correct.detour_m.unwrap(), 0.002);

    board.vias[1].at = mm(10, 18);
    let distant = check_transition(&Layout::from_kicad(&board).unwrap(), &input).unwrap();
    assert_eq!(distant.status, Status::Fail);
    near(distant.detour_m.unwrap(), 0.005);

    board.vias[1].at = mm(10, 15);
    let separated = vec![mm(3, 16), mm(17, 16), mm(17, 20), mm(3, 20)];
    board.zones[1].filled[0].points = separated.clone();
    board.zones[1].outlines = vec![separated];
    let layout = Layout::from_kicad(&board).unwrap();
    assert_eq!(
        check_transition(&layout, &input).unwrap().status,
        Status::Fail
    );
    let missing = TransitionInput {
        transfer: None,
        ..input
    };
    assert_eq!(
        check_transition(&layout, &missing).unwrap().status,
        Status::NotAnalyzed(Open::Transfer)
    );
}

#[test]
fn transition_with_a_slot_does_not_assume_the_straight_offset_is_the_return() {
    let mut board = transition_board();
    // Узкий вырез заходит слева до x = 11 мм между y = 14,3 и 14,5 мм.
    // Кольцо в (10;15) подключено, но прямой подход от (10;13) пересекает вырез.
    let y = |x, y_nm| Point { y_nm, ..mm(x, 0) };
    let outline = vec![
        mm(3, 14),
        mm(17, 14),
        mm(17, 20),
        mm(3, 20),
        y(3, 14_500_000),
        y(11, 14_500_000),
        y(11, 14_300_000),
        y(3, 14_300_000),
    ];
    board.zones[1].filled[0].points = outline.clone();
    board.zones[1].outlines = vec![outline];
    let result = check_transition(&Layout::from_kicad(&board).unwrap(), &transition()).unwrap();
    assert_eq!(result.status, Status::NotAnalyzed(Open::DetourGeometry));
    near(result.detour_m.unwrap(), 0.002);
}

fn terminals(reference: &str) -> [Terminal<'_>; 2] {
    ["1", "2"].map(|pad| Terminal {
        reference,
        pad,
        layer: FRONT,
    })
}

#[test]
fn decoupling_uses_copper_route_instead_of_component_distance() {
    let mut board = kicad::parse(FIXED).unwrap();
    let short = [step(9, mm(32, 16))];
    let return_steps = [step(10, mm(32, 19))];
    // Для этой постановки две пары PROBE из expected.md назначены клеммами
    // нагрузки и конденсатора. Короткий контур 10 + 10 + 3 + 3 = 26 мм.
    let mut input = DecouplingInput {
        scope: Scope::Reference,
        supply: Route {
            net: 6,
            start: node(mm(22, 16), FRONT),
            steps: &short,
        },
        return_arm: Route {
            net: 7,
            start: node(mm(22, 19), FRONT),
            steps: &return_steps,
        },
        load: terminals("J9"),
        capacitor: terminals("J10"),
        topology: Some(BLOCK),
        max_perimeter_m: datum(0.027, BLOCK), // Между контурами 26 и 28 мм.
    };
    let report = check_decoupling(&Layout::from_kicad(&board).unwrap(), &input).unwrap();
    assert_eq!(report.status, Status::Pass);
    near(report.perimeter_m.unwrap(), 0.026);
    for arm in report.arm_lengths_m.unwrap() {
        near(arm, 0.010);
    }

    // Уход на 1 мм в сторону от прямой и возврат добавляют 2 мм.
    board.tracks[9].path = kicad::Path::Segment {
        start: mm(22, 16),
        end: mm(22, 15),
    };
    let i = board.tracks.len();
    board
        .tracks
        .push(segment(mm(22, 15), mm(32, 15), FRONT, 6, 400_000));
    board
        .tracks
        .push(segment(mm(32, 15), mm(32, 16), FRONT, 6, 400_000));
    let long = [
        step(9, mm(22, 15)),
        step(i, mm(32, 15)),
        step(i + 1, mm(32, 16)),
    ];
    input.supply.steps = &long;
    let layout = Layout::from_kicad(&board).unwrap();
    let report = check_decoupling(&layout, &input).unwrap();
    assert_eq!(report.status, Status::Fail);
    near(report.perimeter_m.unwrap(), 0.028);

    input.max_perimeter_m = None;
    assert!(matches!(
        check_decoupling(&layout, &input).unwrap().status,
        Status::NotAnalyzed(Open::Evidence(_))
    ));
    input.supply.steps = &long[..2];
    assert!(check_decoupling(&layout, &input).is_err());
}

#[test]
fn alternate_return_cannot_be_proved_by_choosing_one_path() {
    let mut board = kicad::parse(FIXED).unwrap();
    // Связь между концами двух ветвей даёт альтернативный путь через звезду.
    board
        .tracks
        .push(segment(mm(15, 23), mm(10, 28), FRONT, 5, 500_000));
    let power = [step(11, mm(10, 23)), step(12, mm(15, 23))];
    let signal = [step(13, mm(5, 28)), step(14, mm(10, 28))];
    let layout = Layout::from_kicad(&board).unwrap();
    let result = check_ground_bounce(&layout, &bounce(&power, &signal)).unwrap();
    assert_eq!(result.status, Status::NotAnalyzed(Open::SharedTopology));
    assert_eq!(result.bounce_v, None);
}

#[test]
fn common_via_uses_plated_wall_section_and_actual_stack() {
    let mut board = kicad::parse(FIXED).unwrap();
    let via = board.vias[1].at;
    let branch_end = Point {
        x_nm: mm(10, 0).x_nm,
        ..via
    };
    // Исходная ветвь от звезды, переход и дорожка на обратном слое из expected.md.
    // Дополнительная ветвь от перехода идёт по X.
    board.tracks = vec![
        board.tracks[15].clone(),
        board.tracks[16].clone(),
        segment(via, branch_end, BACK, 5, 500_000),
    ];
    board.vias.remove(0);
    board.zones.clear();
    let via_step = Step::Via {
        index: 0,
        to_layer: BACK,
    };
    let power = [step(0, via), via_step, step(1, mm(5, 9))];
    let signal = [step(0, via), via_step, step(2, branch_end)];
    let layout = Layout::from_kicad(&board).unwrap();
    let mut input = bounce(&power, &signal);
    let missing = check_ground_bounce(&layout, &input).unwrap();
    assert!(matches!(
        missing.status,
        Status::NotAnalyzed(Open::Evidence(_))
    ));
    assert_eq!(missing.resistance_ohm, None);
    let plating = [(0, datum(25e-6, REFERENCE).unwrap())];
    input.plating_m = &plating;
    let result = check_ground_bounce(&layout, &input).unwrap();
    assert_eq!(result.status, Status::Fail);
    near(result.shared_length_m.unwrap(), (1.500 + 1.545) * 1e-3);
    let section = std::f64::consts::PI * (0.400e-3 - 25e-6) * 25e-6;
    let via_r = 1.68e-8 * 1.545e-3 / section;
    let track_r = 1.68e-8 * 1.5e-3 / (0.5e-3 * 35e-6);
    near(result.resistance_ohm.unwrap(), via_r + track_r);
    near(result.bounce_v.unwrap(), via_r + track_r + 0.100);
}

#[test]
fn capacitive_transfer_between_different_planes_needs_both_wired_arms() {
    let mut board = transition_board();
    board.vias[1].at = mm(11, 15);
    board.zones[1].net = 7;
    for (index, low, high) in [(0, 14, 16), (1, 17, 20)] {
        let outline = vec![mm(3, low), mm(17, low), mm(17, high), mm(3, high)];
        board.zones[index].outlines = vec![outline.clone()];
        board.zones[index].filled[0].points = outline;
    }
    board.tracks.extend([
        segment(mm(9, 15), mm(11, 15), BACK, 5, 400_000),
        segment(mm(11, 15), mm(13, 15), FRONT, 5, 400_000),
        segment(mm(10, 18), mm(13, 18), FRONT, 7, 400_000),
    ]);
    let exemplar = kicad::parse(FIXED).unwrap();
    let mut cap = exemplar
        .footprints
        .iter()
        .find(|fp| fp.reference.as_deref() == Some("J9"))
        .unwrap()
        .clone();
    cap.at.point = mm(13, 15);
    cap.pads[0].net = 5;
    cap.reference = Some("C1".into());
    board.footprints.push(cap);

    let lower = [
        step(2, mm(11, 15)),
        Step::Via {
            index: 1,
            to_layer: FRONT,
        },
        step(3, mm(13, 15)),
    ];
    let upper = [step(4, mm(13, 18))];
    let arms = [
        Route {
            net: 5,
            start: node(mm(9, 15), BACK),
            steps: &lower,
        },
        Route {
            net: 7,
            start: node(mm(10, 18), FRONT),
            steps: &upper,
        },
    ];
    let mut setup = transition();
    setup.planes[0].tie = mm(8, 15);
    setup.planes[1].tie = mm(8, 18);
    setup.transfer = Some(Transfer::Capacitor {
        arms,
        pads: terminals("C1"),
        coupling: Some(BLOCK),
    });
    // Два подхода, 2 + 1,545 + 2 + 3 мм меди и 3 мм между площадками.
    let manual_m = (5.0_f64.sqrt() + 5.0 + 2.0 + 1.545 + 2.0 + 3.0 + 3.0) * 1e-3;
    setup.max_detour_m = datum(0.020, BLOCK);
    let layout = Layout::from_kicad(&board).unwrap();
    let result = check_transition(&layout, &setup).unwrap();
    assert_eq!(result.status, Status::Pass);
    near(result.detour_m.unwrap(), manual_m);

    setup.transfer = Some(Transfer::Capacitor {
        arms,
        pads: terminals("C1"),
        coupling: None,
    });
    assert!(matches!(
        check_transition(&layout, &setup).unwrap().status,
        Status::NotAnalyzed(Open::Evidence(_))
    ));
    setup.transfer = Some(Transfer::StitchVia(1));
    assert_eq!(
        check_transition(&layout, &setup).unwrap().status,
        Status::Fail
    );
}

#[test]
fn stored_plane_cannot_cover_a_bore() {
    let mut board = kicad::parse(FIXED).unwrap();
    // Сверло Ø0,4 мм на оси (27;5) в искусственно оставленной заливке.
    board.vias.push(kicad::Via {
        net: 5,
        at: mm(27, 5),
        ..board.vias[0].clone()
    });
    let result = check_reference(&Layout::from_kicad(&board).unwrap(), &reference()).unwrap();
    assert_eq!(result.status, Status::Fail);
    let center = result.center_gaps_m.unwrap();
    assert_eq!(center.len(), 1);
    near(center[0][0], 0.0048);
    near(center[0][1], 0.0052);
}
