use std::collections::{BTreeMap, BTreeSet, VecDeque};

use sim_process::schematic::pcb_map::{
    CopperGraph, CopperSource, PadAddress, PadBinding, check_pcb_map,
};
use sim_process::schematic::sim_circuit::pcb::kicad::{self, KiCadBoard, Point, ViaKind};
use sim_process::schematic::{
    self, Catalog, CompiledSchematic, Component, Contact, ContactKind, Endpoint, Part, PartKind,
    Pin, PinRole, Schematic, Scope,
};
use sim_process::{Analysis, ProcessError};

const FIXED: &str = include_str!("../../../devices/electrical/depth_t4/fixed.kicad_pcb");
const UNSAFE: &str = include_str!("../../../devices/electrical/depth_t4/unsafe.kicad_pcb");

// Ручная таблица клемм из devices/electrical/depth_t4/expected.md.
// Имена компонентов и выводов схемы намеренно отличаются от ref/pad.
const EXPECTED: &[(&str, &[&str])] = &[
    ("PWR", &["J1/1", "J2/1"]),
    ("SENSE_A", &["J3/1", "J5/1"]),
    ("SENSE_B", &["J3/2", "J5/2"]),
    ("LINK", &["J1/2", "J4/1"]),
    ("GND", &["J6/1", "J7/1", "J8/1"]),
    ("PROBE_A", &["J9/1", "J10/1"]),
    ("PROBE_B", &["J9/2", "J10/2"]),
];

fn scheme(rows: &[(&str, &[&str])]) -> (CompiledSchematic, Vec<PadBinding>) {
    let part = Part {
        kind: PartKind::Header,
        source: "Синтетический эталон клемм".into(),
        pins: BTreeMap::from([(
            "контакт".into(),
            Pin {
                contact: Contact {
                    kind: ContactKind::Pad,
                    family: None,
                    key: None,
                },
                role: PinRole::Passive,
            },
        )]),
        internal_nodes: BTreeSet::new(),
        elements: BTreeMap::new(),
        ratings: BTreeMap::new(),
        model_gap: Some("В опыте задана топология только внешней меди".into()),
        reference_only: true,
    };
    let catalog = Catalog::new(BTreeMap::from([("клемма".into(), part)])).unwrap();
    let mut components = BTreeMap::new();
    let mut bindings = Vec::new();
    for (net, addresses) in rows {
        for address in *addresses {
            let (reference, pad) = address.split_once('/').unwrap();
            let endpoint = Endpoint {
                component: format!("клемма {reference} №{pad}"),
                pin: "контакт".into(),
            };
            components.insert(
                endpoint.component.clone(),
                Component {
                    part: "клемма".into(),
                    nets: BTreeMap::from([(endpoint.pin.clone(), (*net).into())]),
                },
            );
            bindings.push(PadBinding {
                address: PadAddress {
                    reference: reference.into(),
                    pad: pad.into(),
                },
                endpoint,
            });
        }
    }
    let schematic = Schematic {
        name: "Блок".into(),
        scope: Scope::Reference,
        nets: rows.iter().map(|(net, _)| (*net).into()).collect(),
        components,
        connections: BTreeMap::new(),
        rails: BTreeMap::new(),
        bindings: Vec::new(),
    };
    (schematic::compile(schematic, &catalog).unwrap(), bindings)
}

fn graph(
    board: &KiCadBoard,
    schematic: &CompiledSchematic,
    bindings: &[PadBinding],
) -> CopperGraph {
    match check_pcb_map(board, schematic, bindings) {
        Ok(Analysis::Analyzed { value, .. }) => value,
        other => panic!("ожидался граф меди: {other:?}"),
    }
}

fn rejection(board: &KiCadBoard, schematic: &CompiledSchematic, bindings: &[PadBinding]) -> String {
    match check_pcb_map(board, schematic, bindings) {
        Err(ProcessError::InvalidInput(message)) => message,
        other => panic!("ожидалась ошибка входа: {other:?}"),
    }
}

fn pending(board: &KiCadBoard, schematic: &CompiledSchematic, bindings: &[PadBinding]) -> String {
    match check_pcb_map(board, schematic, bindings) {
        Ok(Analysis::NotAnalyzed { reason }) => reason,
        other => panic!("ожидался незакрытый анализ: {other:?}"),
    }
}

#[test]
fn чистый_блок_совпадает_с_ручной_таблицей_клемм_и_проходов() {
    let (scheme, mut bindings) = scheme(EXPECTED);
    let board = kicad::parse(FIXED).unwrap();
    let result = graph(&board, &scheme, &bindings);
    assert_eq!(result.terminals.len(), 15); // expected.md, таблица клемм.
    assert_eq!(result.groups.len(), 7);
    // 15 площадок, 17 дорожек, два перехода по два слоя и одна заливка.
    assert_eq!(result.pieces.len(), 15 + 17 + 2 * 2 + 1);

    // Отдельный обход рёбер и ручное разбиение клемм. Совпадения одного
    // количества групп недостаточно, это число одинако́во у обеих плат.
    for (net, addresses) in EXPECTED {
        let start = bindings
            .iter()
            .find(|b| b.address.to_string() == addresses[0])
            .unwrap();
        let start = result.terminals[&start.address].pieces[0];
        let mut reached = BTreeSet::from([start]);
        let mut queue = VecDeque::from([start]);
        while let Some(at) = queue.pop_front() {
            for &[a, b] in &result.contacts {
                for (from, to) in [(a, b), (b, a)] {
                    if from == at && reached.insert(to) {
                        queue.push_back(to);
                    }
                }
            }
        }
        let terminals: BTreeSet<_> = result
            .terminals
            .iter()
            .filter(|(_, t)| t.pieces.iter().any(|p| reached.contains(p)))
            .map(|(address, _)| address.to_string())
            .collect();
        let expected: BTreeSet<_> = addresses.iter().map(|s| (*s).to_owned()).collect();
        assert_eq!(terminals, expected, "{net}");
        let group = result.groups.iter().find(|g| g.contains(&start)).unwrap();
        assert_eq!(*group, reached.iter().copied().collect::<Vec<_>>());
    }

    let mut layer_changes = Vec::new();
    let mut fill_taps = Vec::new();
    for &[a, b] in &result.contacts {
        let [a, b] = [&result.pieces[a], &result.pieces[b]];
        if a.layer != b.layer {
            assert_eq!(a.source, b.source);
            assert_eq!([a.layer.as_str(), b.layer.as_str()], ["F.Cu", "B.Cu"]);
            layer_changes.push(a.source.clone());
        }
        if matches!(b.source, CopperSource::Fill { .. }) {
            assert_eq!(a.layer, "B.Cu");
            fill_taps.push(a.source.clone());
        }
    }
    // Центры проходов (10; 13) и (5; 21,5). Заливка начинается у дорожки
    // (5; 21,5)–(5; 9), дорожка входит в её диапазон y = 3…10 мм.
    assert_eq!(layer_changes, [CopperSource::Via(0), CopperSource::Via(1)]);
    assert_eq!(fill_taps, [CopperSource::Track(16)]);
    bindings.reverse();
    assert_eq!(result, graph(&board, &scheme, &bindings));
}

#[test]
fn дефектный_блок_даёт_оба_ручных_расхождения() {
    let (scheme, bindings) = scheme(EXPECTED);
    let board = kicad::parse(UNSAFE).unwrap();
    let error = rejection(&board, &scheme, &bindings);
    // expected.md: мостик пересекает ось PROBE_B в (27; 19).
    // Перед J4/1 остаётся 14,70 − 14,60 = 0,10 мм на B.Cu.
    assert!(
        error.contains("короткое замыкание цепей PROBE_A ↔ PROBE_B"),
        "{error}"
    );
    assert!(error.contains("обрыв цепи «LINK»"), "{error}");
    assert!(error.contains("J1/2") && error.contains("J4/1"), "{error}");
    assert_eq!(error.split("; ").count(), 2, "{error}");
}

#[test]
fn расхождения_ref_pad_endpoint_отклоняются_даже_при_исправной_меди() {
    let (scheme, mut bindings) = scheme(EXPECTED);
    let board = kicad::parse(FIXED).unwrap();
    let mut wrong = bindings.clone();
    let a = wrong[0].endpoint.clone();
    wrong[0].endpoint = wrong[2].endpoint.clone();
    wrong[2].endpoint = a;
    let error = rejection(&board, &scheme, &wrong);
    assert!(
        error.contains("J1/1") && error.contains("SENSE_A") && error.contains("PWR"),
        "{error}"
    );

    let mut wrong = bindings.clone();
    wrong[0].endpoint.pin = "неизвестный".into();
    assert!(rejection(&board, &scheme, &wrong).contains("неизвестный вывод"));

    let mut wrong_board = board.clone();
    wrong_board.footprints[0].pads[0].net = 0;
    assert!(rejection(&wrong_board, &scheme, &bindings).contains("нет назначенной медной клеммы"));
    wrong_board.footprints[0].pads.remove(0);
    assert!(rejection(&wrong_board, &scheme, &bindings).contains("площадка отсутствует"));

    wrong_board = board.clone();
    wrong_board.footprints[1].reference = wrong_board.footprints[0].reference.clone();
    assert!(rejection(&wrong_board, &scheme, &bindings).contains("повторный ref"));

    bindings.push(bindings[0].clone());
    assert!(rejection(&board, &scheme, &bindings).contains("повтор привязки"));
    bindings.remove(0);
    bindings.pop();
    assert!(rejection(&board, &scheme, &bindings).contains("отсутствует привязка"));
    assert!(rejection(&board, &scheme, &[]).contains("требуется соответствие"));
}

#[test]
fn пробел_заливки_не_подменяется_контуром_и_не_скрывает_короткое() {
    let (scheme, bindings) = scheme(EXPECTED);
    let mut board = kicad::parse(FIXED).unwrap();
    board.zones[0].filled.clear();
    assert!(pending(&board, &scheme, &bindings).contains("нет сохранённой заливки слоя B.Cu"));
    let mut bad = kicad::parse(UNSAFE).unwrap();
    bad.zones[0].filled.clear();
    assert!(rejection(&bad, &scheme, &bindings).contains("PROBE_A ↔ PROBE_B"));
    // Топология по XY и именам слоёв определена без расчёта R, L или Z0.
    let mut saved = kicad::parse(FIXED).unwrap();
    let expected = graph(&saved, &scheme, &bindings);
    saved.stackup = None;
    assert_eq!(graph(&saved, &scheme, &bindings), expected);
    saved.footprints[0].at.angle_rad = f64::NAN;
    assert!(rejection(&saved, &scheme, &bindings).contains("угол должен быть конечным"));
}

fn fixture(copper: &str) -> KiCadBoard {
    let source = format!(
        r#"(kicad_pcb (version 20240108) (generator manual)
      (general)
      (layers (0 "F.Cu" signal) (2 "In1.Cu" signal) (4 "In2.Cu" signal)
              (31 "B.Cu" signal) (37 "F.SilkS" user) (44 "Edge.Cuts" user))
      (setup (pad_to_mask_clearance 0))
      (net 1 "А") (net 2 "Б")
      (gr_rect (start 0 0) (end 12 12) (stroke (width 0.05) (type default))
        (fill none) (layer "Edge.Cuts"))
      {copper})"#
    );
    kicad::parse(&source).unwrap()
}

fn pad(reference: &str, layer: &str, x: u32, y: u32, number: u32, name: &str) -> String {
    format!(
        r#"(footprint "Эталон:Блок" (layer "{layer}") (at {x} {y})
      (property "Reference" "{reference}" (at 0 0 0) (layer "F.SilkS"))
      (pad "1" smd circle (at 0 0) (size 0.6 0.6) (layers "{layer}") (net {number} "{name}")))"#
    )
}

#[test]
fn дуга_учитывает_край_меди_и_не_заменяется_хордой() {
    let (scheme, bindings) = scheme(&[("А", &["X1/1", "X2/1"]), ("Б", &["X3/1"])]);
    let mut copper = [
        pad("X1", "F.Cu", 3, 6, 1, "А"),
        pad("X2", "F.Cu", 9, 6, 1, "А"),
        pad("X3", "F.Cu", 6, 6, 2, "Б"),
    ]
    .join("\n");
    copper.push_str(r#"(arc (start 3 6) (mid 6 3) (end 9 6) (width 0.2) (layer "F.Cu") (net 1))"#);
    // Хорда y = 6 проходит через X3; медь дуги на x = 6 лежит у y = 3.
    assert_eq!(graph(&fixture(&copper), &scheme, &bindings).groups.len(), 2);
    // Центр дорожки не достигает дуги, но края пересекаются на 0,05 мм:
    // (3 + 0,1) − (3,15 − 0,1).
    copper.push_str(r#"(segment (start 6 6) (end 6 3.15) (width 0.2) (layer "F.Cu") (net 2))"#);
    assert!(rejection(&fixture(&copper), &scheme, &bindings).contains("А ↔ Б"));
}

#[test]
fn безымянная_медь_может_замкнуть_разные_цепи() {
    let (scheme, bindings) = scheme(&[("А", &["X1/1"]), ("Б", &["X2/1"])]);
    let mut copper = [
        pad("X1", "F.Cu", 3, 6, 1, "А"),
        pad("X2", "F.Cu", 9, 6, 2, "Б"),
    ]
    .join("\n");
    copper.push_str(r#"(segment (start 3 6) (end 9 6) (width 0.2) (layer "F.Cu") (net 0))"#);
    assert!(rejection(&fixture(&copper), &scheme, &bindings).contains("А ↔ Б"));
}

#[test]
fn переход_соединяет_ровно_свой_диапазон_слоёв() {
    let (scheme, bindings) = scheme(&[("А", &["X1/1", "X2/1"])]);
    let copper = format!(
        r#"{}
      {}
      (segment (start 3 6) (end 6 6) (width 0.2) (layer "F.Cu") (net 1))
      (segment (start 6 6) (end 9 6) (width 0.2) (layer "B.Cu") (net 1))
      (via (at 6 6) (size 0.8) (drill 0.4) (layers "F.Cu" "B.Cu") (net 1))"#,
        pad("X1", "F.Cu", 3, 6, 1, "А"),
        pad("X2", "B.Cu", 9, 6, 1, "А")
    );
    let mut board = fixture(&copper);
    assert_eq!(graph(&board, &scheme, &bindings).groups.len(), 1);
    board.vias[0].kind = ViaKind::BlindBuried;
    board.vias[0].layers[1] = "In1.Cu".into();
    assert!(rejection(&board, &scheme, &bindings).contains("обрыв цепи «А»"));
}

#[test]
fn сквозная_площадка_и_неметаллизированное_сверление_различаются() {
    let copper = format!(
        r#"{}
      {}
      (segment (start 3 5) (end 5.5 5) (width 0.2) (layer "F.Cu") (net 1))
      (segment (start 6.5 5) (end 9 5) (width 0.2) (layer "B.Cu") (net 1))
      (footprint "Эталон:Блок" (layer "F.Cu") (at 6 5)
        (property "Reference" "X3" (at 0 0 0) (layer "F.SilkS"))
        (pad "1" thru_hole oval (at 0 0 90) (size 1.4 1.6)
          (drill oval 0.4 0.6) (layers "*.Cu") (net 1 "А")))"#,
        pad("X1", "F.Cu", 3, 5, 1, "А"),
        pad("X2", "B.Cu", 9, 5, 1, "А")
    );
    let (schematic, bindings) = scheme(&[("А", &["X1/1", "X2/1", "X3/1"])]);
    let mut board = fixture(&copper);
    let result = graph(&board, &schematic, &bindings);
    // Овал повёрнут, по x внешний полуразмер 0,8 мм, отверстие 0,3 мм.
    // Концы дорожек на расстоянии 0,5 мм входят в кольцо с обеих сторон.
    assert_eq!(result.groups.len(), 1);
    assert_eq!(result.terminals[&bindings[2].address].pieces.len(), 4);

    let (schematic, bindings) = scheme(&[("А", &["X1/1", "X2/1"])]);
    let hole = &mut board.footprints[2].pads[0];
    hole.kind = kicad::PadKind::NonPlatedHole;
    hole.net = 0;
    assert!(rejection(&board, &schematic, &bindings).contains("обрыв цепи «А»"));
}

#[test]
fn вырез_в_заливке_и_раздельные_острова_сохраняются() {
    let (scheme, bindings) = scheme(&[("А", &["X1/1", "X2/1"]), ("Б", &["X3/1"])]);
    let copper = format!(
        r#"{}
      {}
      {}
      (zone (net 1) (net_name "А") (layer "B.Cu") (hatch edge 0.5) (fill yes)
        (polygon (pts (xy 2 2) (xy 10 2) (xy 10 10) (xy 2 10)))
        (filled_polygon (layer "B.Cu")
          (pts (xy 2 2) (xy 10 2) (xy 10 10) (xy 2 10)
               (xy 2 2) (xy 4 4) (xy 4 8) (xy 8 8) (xy 8 4) (xy 4 4))))"#,
        pad("X1", "B.Cu", 3, 3, 1, "А"),
        pad("X2", "B.Cu", 9, 9, 1, "А"),
        pad("X3", "B.Cu", 6, 6, 2, "Б")
    );
    let mut board = fixture(&copper);
    // Круг X3 радиусом 0,3 мм лежит целиком в отверстии 4…8 мм.
    assert_eq!(graph(&board, &scheme, &bindings).groups.len(), 2);

    // Оба сохранённых острова лежат внутри проектного контура, но не касаются.
    // Масштаб координат из KiCad v8: 1 000 000 нм в мм.
    let rect = |lo, hi| {
        [(lo, lo), (hi, lo), (hi, hi), (lo, hi)]
            .map(|(x, y)| Point {
                x_nm: x * 1_000_000,
                y_nm: y * 1_000_000,
            })
            .to_vec()
    };
    let fill = &mut board.zones[0].filled;
    fill[0].points = rect(2, 4);
    fill.push(kicad::FilledPolygon {
        layer: "B.Cu".into(),
        points: rect(8, 10),
        island: true,
    });
    assert!(rejection(&board, &scheme, &bindings).contains("обрыв цепи «А»"));
}

#[test]
fn вырез_платы_и_точечное_касание_оставляют_анализ_открытым() {
    let (scheme, bindings) = scheme(&[("А", &["X1/1", "X2/1"])]);
    let mut copper = format!(
        r#"{}
      {}
      (segment (start 3 6) (end 9 6) (width 0.2) (layer "F.Cu") (net 1))"#,
        pad("X1", "F.Cu", 3, 6, 1, "А"),
        pad("X2", "F.Cu", 9, 6, 1, "А")
    );
    assert_eq!(graph(&fixture(&copper), &scheme, &bindings).groups.len(), 1);

    let cut = r#"(gr_rect (start 5 5) (end 7 7) (stroke (width 0.05) (type default))
      (fill none) (layer "Edge.Cuts"))"#;
    assert!(pending(&fixture(&format!("{copper}{cut}")), &scheme, &bindings).contains("Edge.Cuts"));

    // x = 8,6 + 0,1 = 9 − 0,3: торец дорожки касается круглой площадки в одной точке.
    copper = copper.replace("(end 9 6)", "(end 8.6 6)");
    assert!(pending(&fixture(&copper), &scheme, &bindings).contains("касание"));
}

#[test]
fn повтор_номера_площадки_не_создаёт_ребро_меди() {
    let (scheme, bindings) = scheme(&[("А", &["X1/1"])]);
    let footprint = |x| {
        format!(
            r#"(footprint "Эталон:Блок" (layer "F.Cu") (at 3 6)
      (property "Reference" "X1" (at 0 0 0) (layer "F.SilkS"))
      (pad "1" smd circle (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net 1 "А"))
      (pad "1" smd circle (at {x} 0) (size 0.6 0.6) (layers "F.Cu") (net 1 "А")))"#
        )
    };
    // У двух площадок одного номера радиусы по 0,3 мм. При разносе
    // центров 0,5 мм есть перекрытие 0,1 мм, при 0,7 мм есть разрыв 0,1 мм.
    let result = graph(&fixture(&footprint(0.5)), &scheme, &bindings);
    assert_eq!(result.terminals[&bindings[0].address].pieces.len(), 2);
    assert_eq!(result.groups.len(), 1);
    assert!(rejection(&fixture(&footprint(0.7)), &scheme, &bindings).contains("обрыв цепи «А»"));
}

#[test]
fn сверление_без_меди_вне_цепи_не_мешает_проверке() {
    let (scheme, bindings) = scheme(&[("А", &["X1/1"])]);
    let copper = format!(
        r#"{}
      (footprint "Эталон:Блок" (layer "F.Cu") (at 9 9)
        (property "Reference" "X2" (at 0 0 0) (layer "F.SilkS"))
        (pad "" np_thru_hole circle (at 0 0) (size 1 1) (drill 1) (layers "*.Cu" "*.Mask")))"#,
        pad("X1", "F.Cu", 3, 3, 1, "А")
    );
    let result = graph(&fixture(&copper), &scheme, &bindings);
    assert_eq!(result.groups, [vec![0]]); // Размер сверления равен диаметру площадки.
    assert!(result.contacts.is_empty());
}
