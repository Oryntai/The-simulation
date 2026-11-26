//! Аудит положительных исходов Т4 через публичные точки входа.

use kicad::Point;
use ratings::{Datum, Node, Scope, Source};
use return_path::{GroundBounceInput, Plane, ReferenceInput, Route, Step};
use sim_circuit::pcb::{
    IpcLayer, clearance, impedance, ipc2221_area_mil2, kicad, layout::Layout,
    microstrip_hammerstad_jensen, ratings, return_path,
};

const FIXED: &str = include_str!("../../../devices/electrical/depth_t4/fixed.kicad_pcb");
const UNSAFE: &str = include_str!("../../../devices/electrical/depth_t4/unsafe.kicad_pcb");
const SOURCE: &str = "Ручной эталон devices/electrical/depth_t4/expected.md";
const FRONT: &str = "F.Cu";
const BACK: &str = "B.Cu";

fn point(x_nm: i32, y_nm: i32) -> Point {
    Point { x_nm, y_nm }
}

fn node(x_nm: i32, y_nm: i32, layer: &str) -> Node<'_> {
    Node {
        at: point(x_nm, y_nm),
        layer,
    }
}

fn datum(value: f64) -> Datum<'static> {
    Datum {
        value,
        source: Source::Reference(SOURCE),
    }
}

fn layout(text: &str) -> Layout {
    Layout::from_kicad(&kicad::parse(text).unwrap()).unwrap()
}

fn segment(from: Point, to: Point, net: u32, width_nm: i32) -> kicad::Track {
    kicad::Track {
        path: kicad::Path::Segment {
            start: from,
            end: to,
        },
        width_nm,
        layer: FRONT.into(),
        net,
    }
}

fn near(actual: f64, target: f64, tolerance: f64) {
    let relative = (actual / target - 1.0).abs();
    println!(
        "{actual:.8} ≈ {target} (отклонение {:.5}%)",
        relative * 100.0
    );
    assert!(
        relative <= tolerance,
        "{actual} вместо {target}, допуск {tolerance}"
    );
}

#[test]
fn published_ipc_and_microstrip_numbers() {
    // Опубликованные значения и допуски из DEPTH_PLAN, исходные
    // постановки — sim-circuit/src/checks/pcb.rs (Atlas PCB, Olson, Steer).
    for (current, rise, layer, area, tolerance) in [
        (3.0, 10.0, IpcLayer::External, 74.2, 0.002),
        (10.0, 10.0, IpcLayer::External, 390.3, 0.002),
        (10.0, 20.0, IpcLayer::Internal, 680.0, 0.05),
    ] {
        near(ipc2221_area_mil2(current, rise, layer), area, tolerance);
    }
    let z0 = microstrip_hammerstad_jensen(600e-6, 635e-6, 0.0, 4.1);
    near(z0.z0, 75.4, 0.005);
}

#[test]
fn legacy_stroked_fill_and_unsupported_geometry_are_input_errors() {
    assert!(kicad::parse(FIXED).is_ok());
    for (old, new) in [
        ("(version 20240108)", "(version 20240109)"),
        ("smd rect", "smd custom"),
        // KiCad: прежняя заливка утолщает или пересчитывает сохранённые границы.
        (
            "(filled_areas_thickness no)",
            "(filled_areas_thickness yes)",
        ),
        (
            "(filled_areas_thickness no)",
            "(filled_areas_thickness nan)",
        ),
    ] {
        assert!(FIXED.contains(old));
        let changed = FIXED.replacen(old, new, 1);
        assert!(kicad::parse(&changed).is_err(), "принято {new}");
    }
}

fn isolation() -> clearance::Conditions<'static> {
    // Стенд проверки SENSE_A/B, предел из expected.md. Огибающая
    // напряжения — 5 В постоянного тока для проверки её применимости.
    clearance::Conditions {
        scope: clearance::Scope::Reference,
        source: SOURCE,
        material: "Диэлектрик эталона",
        environment: "Сухая чистая поверхность стенда",
        working_v: 5.0,
        peak_v: 5.0,
        frequency_hz: 0.0,
        altitude_m: 0.0,
        distance_error_m: Some(0.0),
    }
}

#[test]
fn clearance_needs_both_limits_and_a_matching_voltage_environment_and_material() {
    use clearance::{Face, Status};
    let good = layout(FIXED);
    let mut rules = clearance::Limits {
        envelope: isolation(),
        clearance_m: Some(0.25e-3),
        creepage_m: Some(0.25e-3),
        slot_credit: None,
    };
    let run = |b: &Layout,
               actual: Option<clearance::Conditions<'static>>,
               limits: Option<clearance::Limits<'static>>| {
        clearance::evaluate(b, Face::Front, [2, 3], actual.as_ref(), limits.as_ref())
    };
    let safe = run(&good, Some(isolation()), Some(rules)).unwrap();
    assert_eq!(safe.status(), Status::Pass);
    near(safe.clearance.distance.unwrap().lower_m, 0.8e-3, 1e-9);
    let bad = run(&layout(UNSAFE), Some(isolation()), Some(rules)).unwrap();
    assert_eq!(bad.status(), Status::Fail);
    near(
        bad.clearance.distance.unwrap().upper_m.unwrap(),
        0.1e-3,
        1e-9,
    );

    for actual in [
        None,
        Some(clearance::Conditions {
            peak_v: 6.0,
            ..isolation()
        }),
        Some(clearance::Conditions {
            working_v: 6.0,
            peak_v: 6.0,
            ..isolation()
        }),
        Some(clearance::Conditions {
            environment: "Загрязнённая поверхность",
            ..isolation()
        }),
        Some(clearance::Conditions {
            material: "Другой материал",
            ..isolation()
        }),
        Some(clearance::Conditions {
            distance_error_m: None,
            ..isolation()
        }),
    ] {
        assert_eq!(
            run(&good, actual, Some(rules)).unwrap().status(),
            Status::NotAnalyzed
        );
    }
    assert_eq!(
        run(&good, Some(isolation()), None).unwrap().status(),
        Status::NotAnalyzed
    );
    rules.creepage_m = None;
    assert_eq!(
        run(&good, Some(isolation()), Some(rules)).unwrap().status(),
        Status::NotAnalyzed
    );
    for invalid in [f64::NAN, f64::INFINITY] {
        let actual = clearance::Conditions {
            peak_v: invalid,
            ..isolation()
        };
        assert!(run(&good, Some(actual), Some(rules)).is_err());
    }
}

fn reference() -> ReferenceInput<'static> {
    ReferenceInput {
        scope: Scope::Reference,
        track: 1,
        plane: Plane {
            zone: 0,
            layer: BACK,
            tie: point(5_000_000, 9_000_000),
            connection: Some(Source::Reference(SOURCE)),
        },
    }
}

#[test]
fn return_and_impedance_require_stack_fill_full_width_frequency_data_and_limit() {
    let mut raw = kicad::parse(FIXED).unwrap();
    let good = layout(FIXED);
    let mut reference = reference();
    assert_eq!(
        return_path::check_reference(&good, &reference)
            .unwrap()
            .status,
        return_path::Status::Pass
    );
    reference.plane.connection = None;
    assert!(matches!(
        return_path::check_reference(&good, &reference)
            .unwrap()
            .status,
        return_path::Status::NotAnalyzed(_)
    ));
    reference.plane.connection = Some(Source::Reference(SOURCE));

    for raw in [
        kicad::KiCadBoard {
            stackup: None,
            ..raw.clone()
        },
        {
            let mut missing = raw.clone();
            missing.zones[0].filled.clear();
            missing
        },
    ] {
        let board = Layout::from_kicad(&raw).unwrap();
        assert!(matches!(
            return_path::check_reference(&board, &reference)
                .unwrap()
                .status,
            return_path::Status::NotAnalyzed(_)
        ));
    }

    // Выемка эталона заканчивается на y=5,25 мм. Ось на y=5,35
    // проходит, но край полосы y=5,15 заходит в выемку.
    let mut notched = kicad::parse(UNSAFE).unwrap();
    notched.tracks[1] = segment(
        point(22_000_000, 5_350_000),
        point(32_000_000, 5_350_000),
        2,
        400_000,
    );
    let check =
        return_path::check_reference(&Layout::from_kicad(&notched).unwrap(), &reference).unwrap();
    assert_eq!(check.center_gaps_m, Some(vec![]));
    assert_eq!(check.status, return_path::Status::Fail);
    assert!(!check.copper_gaps_m.unwrap().is_empty());

    // Номинальное сечение из expected.md, предел 100–150 Ом выбран
    // для положительного контроля. [0; 2] — квазистатический опыт.
    let process = impedance::ProcessWindow {
        width_m: [0.400e-3; 2],
        dielectric_height_m: [1.510e-3; 2],
        trace_copper_m: [0.035e-3; 2],
        epsilon_r: [4.2; 2],
        band_hz: [0.0; 2],
        source: Source::Reference(SOURCE),
    };
    let mark = impedance::Mark {
        reference,
        process: Some(process),
        limit: Some(impedance::Limit {
            ohms: [100.0, 150.0],
            source: Source::Reference(SOURCE),
        }),
        model_error_ohm: Some(datum(0.0)),
    };
    assert_eq!(
        impedance::evaluate(&good, &mark).unwrap().status,
        impedance::Status::Pass
    );
    for missing in [
        impedance::Mark {
            process: None,
            ..mark
        },
        impedance::Mark {
            limit: None,
            ..mark
        },
        impedance::Mark {
            model_error_ohm: None,
            ..mark
        },
    ] {
        assert!(matches!(
            impedance::evaluate(&good, &missing).unwrap().status,
            impedance::Status::NotAnalyzed(_)
        ));
    }
    let dielectric = raw
        .stackup
        .as_mut()
        .unwrap()
        .iter_mut()
        .find(|layer| layer.name == "dielectric 1")
        .unwrap();
    dielectric.epsilon_r = None;
    assert!(matches!(
        impedance::evaluate(&Layout::from_kicad(&raw).unwrap(), &mark)
            .unwrap()
            .status,
        impedance::Status::NotAnalyzed(_)
    ));
    let invalid = impedance::Mark {
        process: Some(impedance::ProcessWindow {
            band_hz: [0.0, f64::INFINITY],
            ..process
        }),
        ..mark
    };
    assert!(impedance::evaluate(&good, &invalid).is_err());
}

fn bounce<'a>(power: &'a [Step<'a>], signal: &'a [Step<'a>]) -> GroundBounceInput<'a> {
    // Ток 1 А, фронт 100 нс, общий L 10 нГн, предел 50 мВ из expected.md.
    let star = node(5_000_000, 23_000_000, FRONT);
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
        topology: Some(Source::Reference(SOURCE)),
        resistivity_ohm_m: Some(datum(1.68e-8)),
        plating_m: &[],
        shared_inductance_h: Some(datum(10e-9)),
        current_a: Some(datum(1.0)),
        slew_a_per_s: Some(datum(1.0 / 100e-9)),
        max_bounce_v: Some(datum(0.05)),
        omitted_bounce_v: Some(datum(0.0)),
    }
}

fn step(index: usize, x: i32, y: i32) -> Step<'static> {
    Step::Track {
        index,
        to: point(x, y),
    }
}

#[test]
fn ground_bounce_needs_a_unique_physical_return() {
    use return_path::Status;
    let power = [
        step(11, 10_000_000, 23_000_000),
        step(12, 15_000_000, 23_000_000),
    ];
    let signal = [
        step(13, 5_000_000, 28_000_000),
        step(14, 10_000_000, 28_000_000),
    ];
    let input = bounce(&power, &signal);
    let good = layout(FIXED);
    let clean = return_path::check_ground_bounce(&good, &input).unwrap();
    assert_eq!(clean.status, Status::Pass);
    assert_eq!(clean.shared_length_m, Some(0.0));
    let shared_power = [
        step(12, 10_000_000, 23_000_000),
        step(13, 15_000_000, 23_000_000),
    ];
    let shared_signal = [
        step(12, 10_000_000, 23_000_000),
        step(14, 10_000_000, 28_000_000),
    ];
    let bad =
        return_path::check_ground_bounce(&layout(UNSAFE), &bounce(&shared_power, &shared_signal))
            .unwrap();
    assert_eq!(bad.status, Status::Fail);
    near(
        bad.bounce_v.unwrap(),
        1.68e-8 * 5e-3 / (0.5e-3 * 35e-6) + 10e-9 / 100e-9,
        1e-9,
    );

    for missing in [
        GroundBounceInput {
            topology: None,
            ..input
        },
        GroundBounceInput {
            current_a: None,
            ..input
        },
        GroundBounceInput {
            slew_a_per_s: None,
            ..input
        },
        GroundBounceInput {
            omitted_bounce_v: None,
            ..input
        },
        GroundBounceInput {
            max_bounce_v: None,
            ..input
        },
    ] {
        assert!(matches!(
            return_path::check_ground_bounce(&good, &missing)
                .unwrap()
                .status,
            Status::NotAnalyzed(_)
        ));
    }

    let mut raw = kicad::parse(FIXED).unwrap();
    // Перемычка не касается осей. На концах зазор между осями
    // 0,3 мм, сумма радиусов 0,5 мм: медь перекрывается на 0,2 мм.
    raw.tracks.push(segment(
        point(9_000_000, 23_300_000),
        point(9_000_000, 27_700_000),
        5,
        500_000,
    ));
    let result =
        return_path::check_ground_bounce(&Layout::from_kicad(&raw).unwrap(), &input).unwrap();
    assert!(
        matches!(result.status, Status::NotAnalyzed(_)),
        "{result:?}"
    );
}

#[test]
fn ground_bounce_does_not_ignore_copper_pour_between_returns() {
    let power = [
        step(11, 10_000_000, 23_000_000),
        step(12, 15_000_000, 23_000_000),
    ];
    let signal = [
        step(13, 5_000_000, 28_000_000),
        step(14, 10_000_000, 28_000_000),
    ];
    let input = bounce(&power, &signal);
    let mut raw = kicad::parse(FIXED).unwrap();
    let mut fill = raw.zones[0].clone();
    fill.layers = vec![FRONT.into()];
    fill.filled[0].layer = FRONT.into();
    // Прямоугольник x=8…10 мм, y=22…29 мм соединяет обе ветви
    // в стороне от звезды (5;23), заданной в expected.md.
    fill.filled[0].points = vec![
        point(8_000_000, 22_000_000),
        point(10_000_000, 22_000_000),
        point(10_000_000, 29_000_000),
        point(8_000_000, 29_000_000),
    ];
    fill.outlines = vec![fill.filled[0].points.clone()];
    raw.zones.push(fill);
    let result =
        return_path::check_ground_bounce(&Layout::from_kicad(&raw).unwrap(), &input).unwrap();
    assert!(
        matches!(result.status, return_path::Status::NotAnalyzed(_)),
        "{result:?}"
    );

    let mut raw = kicad::parse(FIXED).unwrap();
    // Та же перемычка на обратной стороне соединяет ветви через два
    // дополнительных сквозных перехода в узлах (10;23) и (10;28).
    let mut fill = raw.zones[0].clone();
    fill.filled[0].points = vec![
        point(8_000_000, 22_000_000),
        point(11_000_000, 22_000_000),
        point(11_000_000, 29_000_000),
        point(8_000_000, 29_000_000),
    ];
    fill.outlines = vec![fill.filled[0].points.clone()];
    raw.zones.push(fill);
    for y in [23_000_000, 28_000_000] {
        raw.vias.push(kicad::Via {
            at: point(10_000_000, y),
            ..raw.vias[1].clone()
        });
    }
    let result =
        return_path::check_ground_bounce(&Layout::from_kicad(&raw).unwrap(), &input).unwrap();
    assert_eq!(
        result.status,
        return_path::Status::NotAnalyzed(return_path::Open::SharedTopology)
    );
}

#[test]
fn joined_arc_and_segment_can_reconnect_away_from_the_star() {
    let mut raw = kicad::parse(FIXED).unwrap();
    raw.tracks.clear();
    raw.vias.clear();
    raw.footprints.clear();
    raw.zones.clear();
    let star = point(5_000_000, 23_000_000);
    let end = point(15_000_000, 23_000_000);
    let mut arc = segment(star, end, 5, 500_000);
    arc.path = kicad::Path::Arc {
        start: star,
        mid: point(10_000_000, 18_000_000),
        end,
    };
    raw.tracks.push(arc);
    // Полуокружность (x−10)² + (y−23)² = 25 и прямой отрезок
    // (5+10t; 23−5t) пересекаются при t=0 и 0,8, в (5;23) и (13;19).
    raw.tracks
        .push(segment(star, point(15_000_000, 18_000_000), 5, 500_000));
    let power = [step(0, 15_000_000, 23_000_000)];
    let signal = [step(1, 15_000_000, 18_000_000)];
    let report = return_path::check_ground_bounce(
        &Layout::from_kicad(&raw).unwrap(),
        &bounce(&power, &signal),
    )
    .unwrap();
    assert_eq!(
        report.status,
        return_path::Status::NotAnalyzed(return_path::Open::SharedTopology)
    );
}

#[test]
fn ratings_uses_actual_copper_and_requires_plating_load_and_applicable_thermal_model() {
    // Ток опыта LINK из expected.md. Измерение между центрами площадок.
    let mut raw = kicad::parse(FIXED).unwrap();
    let loads = [ratings::Load {
        node: node(15_000_000, 13_000_000, BACK),
        current_a: Some(datum(1.0)),
        max_drop_v: Some(datum(0.025)), // Порог чуть выше ручных 24,881 мВ.
    }];
    let vias = [ratings::ViaLimit {
        via: 0,
        plating_m: Some(datum(25e-6)),
        max_current_a: Some(datum(2.0)), // Синтетический предел выше испытательного 1 А.
        min_ring_m: Some(datum(0.15e-3)),
    }];
    let input = ratings::Input {
        scope: Scope::Reference,
        net: 4,
        reference: node(5_000_000, 13_000_000, FRONT),
        loads: &loads,
        topology: Some(Source::Reference(SOURCE)),
        copper: Some(ratings::Copper {
            rho_20_ohm_m: datum(1.68e-8),
            alpha_per_k: datum(0.00393),
        }), // NBS Handbook 100, pcb.rs.
        copper_temp_c: Some(datum(20.0)),
        ambient_c: None,
        max_laminate_c: None,
        max_rise_k: Some(datum(10.0)),
        thermal_model: None,
        vias: &vias,
    };
    let good = layout(FIXED);
    let check = ratings::evaluate(&good, &input).unwrap();
    let via_r = 1.68e-8 * 1.545e-3 / (std::f64::consts::PI * (0.4e-3 - 25e-6) * 25e-6);
    near(
        check.loads[0].drop.value.unwrap(),
        2.0 * 1.68e-8 * 5e-3 / (0.2e-3 * 35e-6) + via_r,
        1e-9,
    );
    assert_eq!(check.loads[0].drop.status, ratings::Status::Pass);
    assert!(matches!(
        check.branches[0].current_limit.status,
        ratings::Status::NotAnalyzed(_)
    ));
    let missing_plating = [ratings::ViaLimit {
        plating_m: None,
        ..vias[0]
    }];
    let missing_current = [ratings::Load {
        current_a: None,
        ..loads[0]
    }];
    for missing in [
        ratings::Input {
            vias: &missing_plating,
            ..input.clone()
        },
        ratings::Input {
            loads: &missing_current,
            ..input.clone()
        },
        ratings::Input {
            copper: None,
            ..input.clone()
        },
    ] {
        let report = ratings::evaluate(&good, &missing).unwrap();
        assert!(matches!(
            report.loads[0].drop.status,
            ratings::Status::NotAnalyzed(_)
        ));
        assert_eq!(report.loads[0].drop.value, None);
    }
    raw.stackup
        .as_mut()
        .unwrap()
        .iter_mut()
        .find(|layer| layer.name == FRONT)
        .unwrap()
        .thickness_nm = None;
    let board = Layout::from_kicad(&raw).unwrap();
    let missing = ratings::evaluate(&board, &input).unwrap().loads.remove(0);
    assert!(matches!(
        missing.drop.status,
        ratings::Status::NotAnalyzed(_)
    ));
}

#[test]
fn ratings_does_not_ignore_a_parallel_copper_contact() {
    let mut raw = kicad::parse(FIXED).unwrap();
    // Исходная ширина PWR — 0,8 мм. Боковая ветвь 0,5 мм
    // идёт в 0,6 мм от оси, поэтому медь перекрывается на 0,05 мм.
    // Центролинейная сеть видит только тупик, подключённый к источнику.
    let first = point(5_000_000, 6_000_000);
    let middle = point(5_000_000, 6_600_000);
    let end = point(14_000_000, 6_600_000);
    let loads = [ratings::Load {
        node: node(15_000_000, 6_000_000, FRONT),
        current_a: Some(datum(1.0)),
        max_drop_v: Some(datum(0.020)),
    }];
    let input = ratings::Input {
        scope: Scope::Reference,
        net: 1,
        reference: Node {
            at: first,
            layer: FRONT,
        },
        loads: &loads,
        topology: Some(Source::Reference(SOURCE)),
        copper: Some(ratings::Copper {
            rho_20_ohm_m: datum(1.68e-8),
            alpha_per_k: datum(0.00393),
        }), // NBS Handbook 100, pcb.rs.
        copper_temp_c: Some(datum(20.0)),
        ambient_c: None,
        max_laminate_c: None,
        max_rise_k: Some(datum(10.0)),
        thermal_model: Some(Source::Reference(SOURCE)),
        vias: &[],
    };
    let board = Layout::from_kicad(&raw).unwrap();
    let clean = ratings::evaluate(&board, &input).unwrap();
    assert_eq!(clean.loads[0].drop.status, ratings::Status::Pass);
    near(clean.loads[0].drop.value.unwrap(), 0.006, 1e-9);
    let mut curved = raw.clone();
    curved.tracks[0].path = kicad::Path::Arc {
        start: first,
        mid: point(10_000_000, 11_000_000),
        end: loads[0].node.at,
    };
    // Для полукруга 5 мм падение 6 мВ · π/2. Порог 8 мВ
    // между этим падением и 6 мВ на хорде из expected.md.
    let arc_loads = [ratings::Load {
        max_drop_v: Some(datum(0.008)),
        ..loads[0]
    }];
    let board = Layout::from_kicad(&curved).unwrap();
    let arc_input = ratings::Input {
        loads: &arc_loads,
        ..input.clone()
    };
    let arc = ratings::evaluate(&board, &arc_input).unwrap();
    near(
        arc.loads[0].drop.value.unwrap(),
        0.006 * std::f64::consts::FRAC_PI_2,
        1e-9,
    );
    assert_eq!(arc.loads[0].drop.status, ratings::Status::Fail);
    raw.tracks.push(segment(first, middle, 1, 500_000));
    raw.tracks.push(segment(middle, end, 1, 500_000));
    let board = Layout::from_kicad(&raw).unwrap();
    let overlapped = ratings::evaluate(&board, &input).unwrap();
    assert!(
        matches!(
            overlapped.loads[0].drop.status,
            ratings::Status::NotAnalyzed(_)
        ),
        "{overlapped:?}"
    );
}

#[test]
fn common_reference_plane_cannot_pass_through_signal_via() {
    let mut raw = kicad::parse(FIXED).unwrap();
    // Четырёхслойный вариант LINK из expected.md. У обеих дорожек
    // одна соседняя опора In1.Cu. Заливка под (10;13) сплошная,
    // поэтому медь перехода LINK замыкается на опору GND.
    let inner = ["In1.Cu", "In2.Cu"];
    let mut layers = raw.layers[0].clone();
    for (index, name) in inner.into_iter().enumerate() {
        layers.name = name.into();
        layers.ordinal = (index + 1) as u32;
        raw.layers.insert(index + 1, layers.clone());
    }
    let stack = raw.stackup.as_mut().unwrap();
    let copper = stack[1].clone();
    let dielectric = stack[2].clone();
    for (index, name) in inner.into_iter().enumerate() {
        let mut copper = copper.clone();
        copper.name = name.into();
        let mut dielectric = dielectric.clone();
        dielectric.name = format!("dielectric {}", index + 2);
        stack.splice(3 + 2 * index..3 + 2 * index, [copper, dielectric]);
    }
    raw.tracks = raw.tracks[7..9].to_vec();
    raw.tracks[1].layer = inner[1].into();
    raw.vias.truncate(1);
    raw.vias[0].kind = kicad::ViaKind::BlindBuried;
    raw.vias[0].layers[1] = inner[1].into();
    raw.zones[0].layers = vec![inner[0].into()];
    let fill = &mut raw.zones[0].filled[0];
    fill.layer = inner[0].into();
    // Продлеваем прямоугольник с y=10 до y=14 мм: до кольца Ø0,8 мм
    // остаётся 0,6 мм сплошной меди с южной стороны.
    for p in &mut fill.points {
        if p.y_nm == 10_000_000 {
            p.y_nm = 14_000_000;
        }
    }
    raw.zones[0].outlines = vec![fill.points.clone()];
    let board = Layout::from_kicad(&raw).unwrap();
    let mut plane = reference().plane;
    plane.layer = inner[0];
    let mut input = return_path::TransitionInput {
        scope: Scope::Reference,
        tracks: [0, 1],
        planes: [plane; 2],
        signal_via: 0,
        transfer: None,
        max_detour_m: None,
    };
    for bound in [None, Some(datum(3e-3))] {
        input.max_detour_m = bound;
        let report = return_path::check_transition(&board, &input).unwrap();
        assert_eq!(report.status, return_path::Status::Fail);
    }
    // Отодвигаем край плоскости на 0,5 мм от центра. Кольцо
    // радиусом 0,4 мм свободно, но длина обхода ещё не доказана.
    for p in &mut raw.zones[0].filled[0].points {
        if p.y_nm == 14_000_000 {
            p.y_nm = 12_500_000;
        }
    }
    raw.zones[0].outlines = vec![raw.zones[0].filled[0].points.clone()];
    let board = Layout::from_kicad(&raw).unwrap();
    let result = return_path::check_transition(&board, &input).unwrap();
    assert_eq!(
        result.status,
        return_path::Status::NotAnalyzed(return_path::Open::DetourGeometry)
    );
}
