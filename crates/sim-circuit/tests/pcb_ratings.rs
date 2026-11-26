//! Ручной эталон Т4.3 и независимые проверки узловой сети.

use sim_circuit::pcb::{
    CU_RHO_20C, FOIL_M_PER_OZ, IpcLayer, MIL, ipc2221_current, ipc2221_delta_t, kicad,
    trace_resistance, via_resistance,
};

// Публичное подключение модулей из pcb.rs закреплено за 4.9.
#[allow(dead_code)]
#[path = "../src/pcb/layout.rs"]
mod layout;
#[path = "../src/pcb/ratings.rs"]
mod ratings;

use layout::{Layout, NotAnalyzed, Point};
use ratings::{
    Copper, Datum, Input, Load, Node, Open, Report, Scope, Section, Source, Status, ViaLimit,
    evaluate,
};

const UNSAFE: &str = include_str!("../../../devices/electrical/depth_t4/unsafe.kicad_pcb");
const FIXED: &str = include_str!("../../../devices/electrical/depth_t4/fixed.kicad_pcb");
const REFERENCE: Source<'static> = Source::Reference("devices/electrical/depth_t4/expected.md");
const BLOCK: Source<'static> = Source::Reference("Ручной расчёт нейтральной сети ниже");

fn load(text: &str) -> Layout {
    Layout::from_kicad(&kicad::parse(text).unwrap()).unwrap()
}

fn node(x_nm: i32, y_nm: i32, layer: &str) -> Node<'_> {
    Node {
        at: Point { x_nm, y_nm },
        layer,
    }
}

fn datum(value: f64, source: Source<'_>) -> Datum<'_> {
    Datum { value, source }
}

fn input<'a>(
    net: u32,
    reference: Node<'a>,
    loads: &'a [Load<'a>],
    vias: &'a [ViaLimit<'a>],
) -> Input<'a> {
    Input {
        scope: Scope::Reference,
        net,
        reference,
        loads,
        topology: Some(REFERENCE),
        copper: Some(Copper {
            // expected.md, модель меди при 20 °C. Коэффициент из NBS Handbook 100,
            // приведён в pcb.rs, при этой температуре поправка равна нулю.
            rho_20_ohm_m: datum(1.68e-8, REFERENCE),
            alpha_per_k: datum(0.00393, Source::Reference("NBS Handbook 100, IEC 60028")),
        }),
        copper_temp_c: Some(datum(20.0, REFERENCE)),
        ambient_c: None,
        max_rise_k: Some(datum(10.0, REFERENCE)),
        max_laminate_c: None,
        thermal_model: Some(REFERENCE),
        vias,
    }
}

fn near(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual:.12e} вместо {expected:.12e}, допуск {tolerance:.3e}"
    );
}

fn load_from<'a>(node: Node<'a>, amps: f64, max_drop: f64, source: Source<'a>) -> Load<'a> {
    Load {
        node,
        current_a: Some(datum(amps, source)),
        max_drop_v: Some(datum(max_drop, source)),
    }
}

#[test]
fn reference_power_drop_and_ipc_limit() {
    // Координаты, ток и границы из expected.md. Допуск для IPC — округление
    // ручного результата до сотой ампера. Для Ома — округление float.
    let loads = [load_from(
        node(15_000_000, 6_000_000, "F.Cu"),
        1.0,
        0.020,
        REFERENCE,
    )];
    let setup = input(1, node(5_000_000, 6_000_000, "F.Cu"), &loads, &[]);
    for (text, r, ampacity, status) in [
        (UNSAFE, 0.048, 0.45, Status::Fail),
        (FIXED, 0.006, 2.03, Status::Pass),
    ] {
        let layout = load(text);
        let report = evaluate(&layout, &setup).unwrap();
        assert!(report.network_open.is_empty(), "{report:?}");
        assert_eq!(report.branches.len(), 1);
        let branch = &report.branches[0];
        near(branch.resistance_ohm.unwrap(), r, 1e-15);
        near(branch.current_a.unwrap(), 1.0, 1e-15);
        near(branch.power_w.unwrap(), r, 1e-15);
        near(report.loads[0].drop.value.unwrap(), r, 1e-15);
        near(branch.current_limit.limit.unwrap(), ampacity, 0.005);
        // Из ручного I при 10 К и его округления получаем независимый
        // интервал ΔT(1 А) по степени 0,44 из expected.md.
        let rise_at = |amps: f64| 10.0 * (1.0 / amps).powf(1.0 / 0.44);
        assert!(
            (rise_at(ampacity + 0.005)..=rise_at(ampacity - 0.005))
                .contains(&branch.ipc_rise_k.unwrap())
        );
        assert_eq!(branch.current_limit.status, status);
        assert_eq!(report.loads[0].drop.status, status);
        assert!(matches!(
            branch.temperature_c.as_ref().unwrap().status,
            Status::NotAnalyzed(_)
        ));
        assert_eq!(report.status() == Status::Fail, text == UNSAFE);
    }
}

#[test]
fn reference_link_via_ring_and_open_path() {
    let loads = [Load {
        node: node(15_000_000, 13_000_000, "B.Cu"),
        current_a: Some(datum(1.0, BLOCK)),
        max_drop_v: None,
    }];
    let rules = [ViaLimit {
        via: 0,
        plating_m: Some(datum(25e-6, REFERENCE)),
        max_current_a: None,
        min_ring_m: Some(datum(0.15e-3, REFERENCE)),
    }];
    // Единичный испытательный ток переводит измеренное падение в R.
    let setup = input(4, node(5_000_000, 13_000_000, "F.Cu"), &loads, &rules);
    for (text, ring, status) in [
        (FIXED, 0.20e-3, Status::Pass),
        (UNSAFE, 0.10e-3, Status::Fail),
    ] {
        let layout = load(text);
        let report = evaluate(&layout, &setup).unwrap();
        let via = report
            .branches
            .iter()
            .find(|b| b.section == Section::Via(0))
            .unwrap();
        near(via.length_m.unwrap(), 1.545e-3, 1e-15);
        // Допуск — округление эталона до тысячной миллиома.
        near(via.resistance_ohm.unwrap(), 0.881e-3, 0.5e-6);
        near(report.vias[0].ring.value.unwrap(), ring, 1e-15);
        assert_eq!(report.vias[0].ring.status, status);
        assert!(matches!(via.current_limit.status, Status::NotAnalyzed(_)));
        if text == FIXED {
            assert!(report.network_open.is_empty(), "{report:?}");
            near(report.loads[0].drop.value.unwrap(), 24.881e-3, 0.5e-6);
            let series = report
                .branches
                .iter()
                .map(|branch| branch.drop_v.unwrap())
                .sum::<f64>();
            near(series, report.loads[0].drop.value.unwrap(), 1e-15);
        } else {
            assert!(report.network_open.contains(&Open::Disconnected));
            assert_eq!(report.loads[0].drop.value, None);
            assert_eq!(report.status(), Status::Fail);
        }
    }
}

const HEADER: &str = r#"(kicad_pcb (version 20240108) (generator "reference")
    (general (thickness 1.580))
    (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
    (setup (stackup
        (layer "F.Cu" (type "copper") (thickness 0.035))
        (layer "dielectric 1" (type "core") (thickness 1.510) (material "Блок"))
        (layer "B.Cu" (type "copper") (thickness 0.035))))
    (net 1 "A")
"#;

const PARALLEL: &str = r#"
    (segment (start 0 0) (end 5 0) (width 0.5) (layer "F.Cu") (net 1))
    (segment (start 5 0) (end 15 0) (width 0.5) (layer "F.Cu") (net 1))
    (segment (start 5 0) (end 5 5) (width 0.5) (layer "F.Cu") (net 1))
    (segment (start 5 5) (end 15 5) (width 0.5) (layer "F.Cu") (net 1))
    (segment (start 15 5) (end 15 0) (width 0.5) (layer "F.Cu") (net 1))
"#;

const STUB: &str = r#"(segment (start 10 0) (end 10 -5) (width 0.5) (layer "F.Cu") (net 1))"#;

fn block(body: &str) -> Layout {
    load(&format!("{HEADER}{body})"))
}

fn power_balance(report: &Report<'_>, amps: f64, drop_v: f64) {
    let dissipated = report
        .branches
        .iter()
        .map(|b| b.power_w.unwrap())
        .sum::<f64>();
    near(dissipated, amps * drop_v, 1e-14);
}

#[test]
fn parallel_divider_tap_and_signed_currents() {
    let layout = block(&format!("{PARALLEL}{STUB}"));
    // 0,96 мОм/мм из ρℓ/A для 0,50 × 0,035 мм². Плечи 10 и 20 мм,
    // до развилки 5 мм. R = 4,8 + (9,6 || 19,2) = 11,2 мОм.
    // Боковая ветвь без нагрузки подключена в середине короткого плеча.
    for amps in [1.0, -1.0] {
        let loads = [load_from(node(15_000_000, 0, "F.Cu"), amps, 0.020, BLOCK)];
        let mut setup = input(1, node(0, 0, "F.Cu"), &loads, &[]);
        setup.topology = Some(BLOCK);
        setup.thermal_model = Some(BLOCK);
        setup.ambient_c = Some(datum(20.0, BLOCK));
        setup.max_laminate_c = Some(datum(30.0, BLOCK));
        let report = evaluate(&layout, &setup).unwrap();
        assert_eq!(report.status(), Status::Pass, "{report:?}");
        assert_eq!(report, evaluate(&layout, &setup).unwrap());
        assert_eq!(report.branches.len(), 7);
        let expected = [
            amps,
            amps * 2.0 / 3.0,
            amps * 2.0 / 3.0,
            amps / 3.0,
            amps / 3.0,
            amps / 3.0,
            0.0,
        ];
        for (branch, current) in report.branches.iter().zip(expected) {
            near(branch.current_a.unwrap(), current, 1e-14);
        }
        let drop = report.loads[0].drop.value.unwrap();
        near(drop, amps * 0.0112, 1e-15);
        power_balance(&report, amps, drop);
    }
}

#[test]
fn multiple_loads_match_independent_mna() {
    let layout = block(PARALLEL);
    // У нижней клеммы потребляется 1 А, в верхнюю справа подаётся 0,25 А.
    // Сопротивления в нетлисте размечены вручную по длинам 5/10/5/10/5 мм.
    let loads = [
        load_from(node(15_000_000, 0, "F.Cu"), 1.0, 0.020, BLOCK),
        load_from(node(15_000_000, 5_000_000, "F.Cu"), -0.25, 0.020, BLOCK),
    ];
    let report = evaluate(&layout, &input(1, node(0, 0, "F.Cu"), &loads, &[])).unwrap();
    let circuit = sim_circuit::Circuit::parse(
        "Блок\nR1 0 split 4.8m\nR2 split exit 9.6m\nR3 split bend 4.8m\nR4 bend upper 9.6m\nR5 upper exit 4.8m\nI1 exit 0 1\nI2 0 upper 0.25\n.op\n.end",
    ).unwrap();
    let op = sim_circuit::operating_point_with(
        &circuit,
        sim_circuit::SimOptions {
            gshunt: 0.0,
            ..sim_circuit::SimOptions::default()
        },
    )
    .unwrap();
    for (branch, name) in report.branches.iter().zip(["r1", "r2", "r3", "r4", "r5"]) {
        near(branch.current_a.unwrap(), op.current(name).unwrap(), 1e-14);
    }
    for (load, name) in report.loads.iter().zip(["exit", "upper"]) {
        near(load.drop.value.unwrap(), -op.voltage(name).unwrap(), 1e-15);
    }
}

#[test]
fn missing_material_stack_thickness_fill_and_load_remain_open() {
    let loads = [load_from(
        node(15_000_000, 6_000_000, "F.Cu"),
        1.0,
        0.020,
        REFERENCE,
    )];
    let mut setup = input(1, node(5_000_000, 6_000_000, "F.Cu"), &loads, &[]);
    let fixed = load(FIXED);
    let mut raw = kicad::parse(FIXED).unwrap();
    raw.stackup = None;
    let no_stack = Layout::from_kicad(&raw).unwrap();
    let blocked = evaluate(&no_stack, &setup).unwrap();
    assert!(
        blocked
            .network_open
            .contains(&Open::Layout(NotAnalyzed::MissingStackup))
    );
    assert_eq!(blocked.loads[0].drop.value, None);

    let mut raw = kicad::parse(FIXED).unwrap();
    raw.stackup
        .as_mut()
        .unwrap()
        .iter_mut()
        .find(|s| s.name == "F.Cu")
        .unwrap()
        .thickness_nm = None;
    let no_thickness = Layout::from_kicad(&raw).unwrap();
    assert!(
        evaluate(&no_thickness, &setup)
            .unwrap()
            .network_open
            .contains(&Open::Layout(NotAnalyzed::MissingThickness("F.Cu".into())))
    );

    let mut raw = kicad::parse(FIXED).unwrap();
    raw.stackup
        .as_mut()
        .unwrap()
        .iter_mut()
        .find(|s| s.name == "dielectric 1")
        .unwrap()
        .material = None;
    let no_material = Layout::from_kicad(&raw).unwrap();
    let report = evaluate(&no_material, &setup).unwrap();
    assert!(matches!(
        report.branches[0].current_limit.status,
        Status::NotAnalyzed(Open::Layout(NotAnalyzed::MissingMaterial(_)))
    ));
    assert_eq!(report.loads[0].drop.status, Status::Pass);

    let mut raw = kicad::parse(FIXED).unwrap();
    raw.zones[0].filled.clear();
    let no_fill = Layout::from_kicad(&raw).unwrap();
    assert!(matches!(
        evaluate(&no_fill, &setup).unwrap().branches[0]
            .current_limit
            .status,
        Status::NotAnalyzed(Open::Layout(NotAnalyzed::MissingFill(_)))
    ));
    // Среда на 10 К выше испытательного предела материала. Нарушение
    // следует уже из температуры среды, без расчёта охлаждения заливкой.
    setup.ambient_c = Some(datum(40.0, BLOCK));
    setup.max_laminate_c = Some(datum(30.0, BLOCK));
    assert_eq!(evaluate(&no_fill, &setup).unwrap().status(), Status::Fail);

    setup.copper = None;
    assert!(
        evaluate(&fixed, &setup)
            .unwrap()
            .network_open
            .contains(&Open::MissingOrUnverified("материал проводника"))
    );
    let unresolved = [Load {
        current_a: None,
        ..loads[0]
    }];
    let setup = input(1, node(5_000_000, 6_000_000, "F.Cu"), &unresolved, &[]);
    assert_eq!(
        evaluate(&fixed, &setup).unwrap().branches[0].current_a,
        None
    );
}

#[test]
fn missing_plating_cannot_make_a_zero_resistance_via() {
    let fixed = load(FIXED);
    let loads = [load_from(
        node(15_000_000, 13_000_000, "B.Cu"),
        1.0,
        0.020,
        REFERENCE,
    )];
    let rules = [ViaLimit {
        via: 0,
        plating_m: None,
        max_current_a: None,
        min_ring_m: None,
    }];
    let report = evaluate(
        &fixed,
        &input(4, node(5_000_000, 13_000_000, "F.Cu"), &loads, &rules),
    )
    .unwrap();
    assert_eq!(
        report.network_open,
        [Open::MissingOrUnverified("толщина металлизации перехода")]
    );
    assert_eq!(report.loads[0].drop.value, None);
    assert!(matches!(report.vias[0].ring.status, Status::NotAnalyzed(_)));
}

#[test]
fn passport_via_current_checks_each_barrel_interval() {
    // Сквозной переход с нагрузкой на внутреннем и обратном слое.
    // Два первых интервала проводят 0,9 А, нижний — 0,2 А. Условный
    // паспорт оснастки допускает 0,8 А при этом установившемся режиме.
    let text = r#"(kicad_pcb (version 20240108) (generator "reference")
      (general (thickness 1.58))
      (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) (31 "B.Cu" signal))
      (setup (stackup
          (layer "F.Cu" (type "copper") (thickness 0.035))
          (layer "dielectric 1" (type "core") (thickness 0.480) (material "Блок"))
          (layer "In1.Cu" (type "copper") (thickness 0.035))
          (layer "dielectric 2" (type "core") (thickness 0.480) (material "Блок"))
          (layer "In2.Cu" (type "copper") (thickness 0.035))
          (layer "dielectric 3" (type "core") (thickness 0.480) (material "Блок"))
          (layer "B.Cu" (type "copper") (thickness 0.035))))
      (net 1 "A")
      (via (at 0 0) (size 0.8) (drill 0.4) (layers "F.Cu" "B.Cu") (net 1))
    )"#;
    let layout = load(text);
    let loads = [
        load_from(node(0, 0, "In2.Cu"), 0.7, 0.020, BLOCK),
        load_from(node(0, 0, "B.Cu"), 0.2, 0.020, BLOCK),
    ];
    let rules = [ViaLimit {
        via: 0,
        plating_m: Some(datum(25e-6, BLOCK)),
        max_current_a: Some(datum(0.8, BLOCK)),
        min_ring_m: None,
    }];
    let report = evaluate(&layout, &input(1, node(0, 0, "F.Cu"), &loads, &rules)).unwrap();
    assert!(report.network_open.is_empty(), "{report:?}");
    assert_eq!(report.branches.len(), 3);
    // Независимый пересчёт кольцевого сечения из разности площадей кругов.
    let area = std::f64::consts::PI * (0.200e-3_f64.powi(2) - 0.175e-3_f64.powi(2));
    let ohms = 1.68e-8 * 0.515e-3 / area;
    for (section, (amps, status)) in report.branches.iter().zip([
        (0.9, Status::Fail),
        (0.9, Status::Fail),
        (0.2, Status::Pass),
    ]) {
        near(section.resistance_ohm.unwrap(), ohms, 1e-15);
        near(section.current_a.unwrap(), amps, 1e-14);
        assert_eq!(section.current_limit.status, status);
    }
    near(
        report.loads[1].drop.value.unwrap(),
        ohms * (0.9 + 0.9 + 0.2),
        1e-15,
    );
    assert_eq!(report.status(), Status::Fail);
}

#[test]
fn arcs_use_radius_times_sweep_and_overlap_is_open() {
    let arc = block(r#"(arc (start 0 0) (mid 5 5) (end 10 0) (width 0.5) (layer "F.Cu") (net 1))"#);
    let loads = [load_from(node(10_000_000, 0, "F.Cu"), 1.0, 0.020, BLOCK)];
    let setup = input(1, node(0, 0, "F.Cu"), &loads, &[]);
    let report = evaluate(&arc, &setup).unwrap();
    let length = std::f64::consts::PI * 5e-3;
    near(report.branches[0].length_m.unwrap(), length, 1e-15);
    near(
        report.loads[0].drop.value.unwrap(),
        length * 1.68e-8 / (0.5e-3 * 35e-6),
        1e-15,
    );
    let tapped = [load_from(
        node(5_000_000, 5_000_000, "F.Cu"),
        1.0,
        0.020,
        BLOCK,
    )];
    let report = evaluate(&arc, &input(1, setup.reference, &tapped, &[])).unwrap();
    assert!(report.network_open.contains(&Open::ArcTap));
    assert_eq!(report.loads[0].drop.value, None);

    // Два частично наложенных сегмента — одна полоса, удвоения меди нет.
    let overlap = block(
        r#"
       (segment (start 0 0) (end 10 0) (width 0.5) (layer "F.Cu") (net 1))
       (segment (start 4 0) (end 14 0) (width 0.5) (layer "F.Cu") (net 1))
    "#,
    );
    let report = evaluate(&overlap, &setup).unwrap();
    assert!(report.network_open.contains(&Open::OverlappingCopper));
    assert_eq!(report.loads[0].drop.value, None);
}

#[test]
fn outside_ipc_and_unverified_sources_do_not_pass() {
    let fixed = load(FIXED);
    let loads = [load_from(
        node(15_000_000, 6_000_000, "F.Cu"),
        1.0,
        0.020,
        REFERENCE,
    )];
    let mut setup = input(1, node(5_000_000, 6_000_000, "F.Cu"), &loads, &[]);
    // 1 К ниже нижней границы применимости формулы в pcb.rs.
    setup.max_rise_k = Some(datum(1.0, BLOCK));
    let report = evaluate(&fixed, &setup).unwrap();
    assert_eq!(
        report.branches[0].current_limit.status,
        Status::NotAnalyzed(Open::OutsideIpc2221)
    );
    assert!(report.branches[0].current_limit.limit.is_some());

    setup.topology = None;
    let report = evaluate(&fixed, &setup).unwrap();
    assert!(!report.network_open.is_empty());
    assert_eq!(report.loads[0].drop.value, None);

    setup.scope = Scope::Assembly;
    setup.topology = Some(Source::Estimate("допущение"));
    let report = evaluate(&fixed, &setup).unwrap();
    assert!(matches!(report.status(), Status::NotAnalyzed(_)));
    assert_eq!(report.loads[0].drop.limit, None);
}

#[test]
fn verified_voltage_failure_survives_unverified_thermal_model() {
    let layout = load(FIXED);
    // Синтетические записи имитируют источник, применимый к сборке.
    // Для 6 мОм при 20 °C и 40 К выше опоры: R = 6·(1 + 0,00393·40) мОм.
    // Испытательный предел 6,5 мВ лежит между холодным и горячим значением.
    let source = Source::Verified("Учебная постановка с заданной температурой меди");
    let loads = [load_from(
        node(15_000_000, 6_000_000, "F.Cu"),
        1.0,
        6.5e-3,
        source,
    )];
    let mut setup = input(1, node(5_000_000, 6_000_000, "F.Cu"), &loads, &[]);
    setup.scope = Scope::Assembly;
    setup.topology = Some(source);
    setup.copper = Some(Copper {
        rho_20_ohm_m: datum(1.68e-8, source),
        alpha_per_k: datum(0.00393, source),
    });
    setup.copper_temp_c = Some(datum(60.0, source));
    setup.max_rise_k = Some(datum(10.0, source));
    let report = evaluate(&layout, &setup).unwrap();
    near(report.loads[0].drop.value.unwrap(), 6.9432e-3, 1e-15);
    assert_eq!(report.loads[0].drop.status, Status::Fail);
    assert!(matches!(
        report.branches[0].current_limit.status,
        Status::NotAnalyzed(Open::MissingOrUnverified(
            "условия применимости тепловой модели платы"
        ))
    ));
    assert_eq!(report.status(), Status::Fail);
}

#[test]
fn invalid_and_nonfinite_inputs_are_errors_even_on_an_open_network() {
    let mut fixed = load(FIXED);
    let reference = node(5_000_000, 6_000_000, "F.Cu");
    let load = load_from(node(15_000_000, 6_000_000, "F.Cu"), 1.0, 0.020, REFERENCE);
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let loads = [Load {
            max_drop_v: Some(datum(bad, Source::Estimate("оценка"))),
            ..load
        }];
        let mut setup = input(1, reference, &loads, &[]);
        setup.topology = None;
        assert!(evaluate(&fixed, &setup).is_err());
    }
    let loads = [load, load];
    assert!(evaluate(&fixed, &input(1, reference, &loads, &[])).is_err());
    let loads = [load];
    // Неконечная геометрия запрещена даже у другой цепи той же платы.
    fixed.tracks[1].width_m = f64::NAN;
    assert!(evaluate(&fixed, &input(1, reference, &loads, &[])).is_err());
}
