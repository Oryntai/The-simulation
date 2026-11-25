use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    path::{Path, PathBuf},
};

use sim_cli::electrical::*;
use sim_process::{Analysis, schematic::*};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn catalog() -> Catalog {
    load_catalog(&root().join("catalog/electrical"))
        .unwrap()
        .catalog
}

fn corpus(class: StaticClass, file: &str) -> PathBuf {
    root()
        .join("devices/electrical/corpus")
        .join(class.code())
        .join(file)
}

fn pair(class: StaticClass) -> [CompiledSchematic; 2] {
    let catalog = catalog();
    let pair = ["unsafe.toml", "fixed.toml"].map(|file| {
        load_schematic(&corpus(class, file), &catalog)
            .unwrap()
            .compiled
    });
    for schematic in &pair {
        let report = schematic.check_static();
        for other in [
            StaticClass::AssemblyMismatch,
            StaticClass::MissingGround,
            StaticClass::ReversePolarity,
        ] {
            if other != class {
                assert_ne!(
                    report.status(other),
                    Status::Fail,
                    "лишний отказ {}",
                    other.code()
                );
            }
        }
    }
    pair
}

fn end(component: &str, pin: &str) -> Endpoint {
    Endpoint {
        component: component.into(),
        pin: pin.into(),
    }
}

fn voltage(point: &DcReport, plus: Endpoint, minus: Endpoint) -> f64 {
    let Analysis::Analyzed { value, .. } = point.voltage(&plus, &minus) else {
        panic!("нет напряжения {plus} относительно {minus}");
    };
    value
}

fn current(point: &DcReport, section: Section) -> f64 {
    point
        .branches()
        .iter()
        .find(|b| b.section == section)
        .unwrap()
        .current_a
}

fn near(actual: f64, expected: f64) {
    // 1 нВ / 1 нА отделяют округление линейной цепи от намеренного изменения схемы.
    assert!(
        (actual - expected).abs() < 1e-9,
        "получено {actual}, ожидалось {expected}"
    );
}

fn section(id: &str) -> Section {
    Section::Connection(id.into())
}

#[test]
fn assembly_mismatch() {
    let class = StaticClass::AssemblyMismatch;
    let [bad, good] = pair(class);
    let [bad_report, good_report] = [&bad, &good].map(CompiledSchematic::check_static);
    assert_eq!(bad_report.status(class), Status::Fail);
    assert_eq!(good_report.status(class), Status::Pass);
    let failed = bad_report
        .assembly_mismatch
        .iter()
        .filter(|f| f.status == Status::Fail)
        .collect::<Vec<_>>();
    assert_eq!(failed.len(), 1);
    assert_eq!(failed[0].subject, "feed");
    assert_eq!(
        failed[0].evidence.contacts.each_ref().map(|c| c.kind),
        [ContactKind::Socket; 2]
    );
    assert!(!failed[0].evidence.branch_connected);
    assert!(failed[0].reason.contains("физическая ветвь разомкнута"));

    let fixed = good_report
        .assembly_mismatch
        .iter()
        .filter(|f| f.evidence.kind == JointKind::Plug)
        .collect::<Vec<_>>();
    assert_eq!(fixed.len(), 2);
    assert!(fixed.iter().all(|f| {
        f.evidence
            .contacts
            .each_ref()
            .map(|c| c.kind)
            .contains(&ContactKind::Plug)
            && f.evidence
                .contacts
                .each_ref()
                .map(|c| c.kind)
                .contains(&ContactKind::Socket)
            && f.evidence.contacts[0].family == f.evidence.contacts[1].family
            && f.evidence.branch_connected
    }));
    assert!(
        bad.dc()
            .branches()
            .iter()
            .all(|b| b.section != section("feed"))
    );
    near(current(&bad.dc(), section("return")), 0.0);
    // 5 В, 1 Ом источника, 100 Ом нагрузки, 0,2 Ом жилы и три стыка по 0,1 Ом из TOML.
    let ohm = 5.0 / (1.0 + 100.0 + 0.2 + 3.0 * 0.1);
    near(current(&good.dc(), section("source_end")), ohm);
    near(
        voltage(&good.dc(), end("load", "input"), end("load", "return")),
        ohm * 100.0,
    );

    let sheet = good.assembly_sheet();
    let order = sheet
        .operations
        .iter()
        .map(|op| op.id.as_str())
        .collect::<Vec<_>>();
    assert_eq!(order, ["return", "source_end", "load_end"]);
    assert!(
        sheet
            .parts
            .iter()
            .any(|p| p.component == "adapter" && p.kind == PartKind::Wire)
    );
    assert!(sheet.inspections.iter().any(|i| i.subject == "source_end" && i.stage == Stage::BeforePower
        && matches!(&i.expected, Expected::ResistanceOhm(value) if value.range.as_ref().unwrap().nominal == 0.1)));
    assert!(sheet.inspections.iter().any(|i| i.subject == "source_end" && i.stage == Stage::CurrentLimitedStart
        && matches!(&i.expected, Expected::CurrentLimitA(value) if value.range.as_ref().unwrap().nominal == 1.0)));
    assert_eq!(sheet, good.assembly_sheet());
    println!(
        "assembly_mismatch: 0 из 1 / 2 из 2 сопряжений питания, I = 0 / {:.6} мА",
        ohm * 1000.0
    );
}

#[test]
fn missing_ground() {
    let class = StaticClass::MissingGround;
    let [bad, good] = pair(class);
    let [bad_report, good_report] = [&bad, &good].map(CompiledSchematic::check_static);
    assert_eq!(bad_report.status(class), Status::Fail);
    assert_eq!(good_report.status(class), Status::Pass);
    let [bad_return] = &bad_report.missing_ground[..] else {
        panic!("ожидалась одна сигнальная пара");
    };
    let [good_return] = &good_report.missing_ground[..] else {
        panic!("ожидалась одна сигнальная пара");
    };
    assert_eq!(
        bad_return.evidence.references,
        [end("sender", "minus"), end("receiver", "return")]
    );
    assert_eq!(bad_return.evidence.signal[0].section, section("signal"));
    assert!(bad_return.evidence.return_path.is_none());
    assert!(
        bad_return
            .reason
            .contains("между локальными опорами нет штатного пути")
    );
    let [step] = &good_return.evidence.return_path.as_ref().unwrap()[..] else {
        panic!("ожидалась одна перемычка");
    };
    assert_eq!(step.section, section("return"));
    assert_eq!(
        [step.from.clone(), step.to.clone()],
        good_return.evidence.references
    );

    // Разомкнутый контур и закон Ома для 5 В, 1 Ом источника, 1 кОм входа и двух стыков по 0,1 Ом.
    near(current(&bad.dc(), section("signal")), 0.0);
    let ohm = 5.0 / (1.0 + 1000.0 + 2.0 * 0.1);
    near(current(&good.dc(), section("signal")), ohm);
    near(current(&good.dc(), section("return")), -ohm);
    println!(
        "missing_ground: перемычек 0 / 1, I = 0 / {:.6} мА",
        ohm * 1000.0
    );
}

#[test]
fn reverse_polarity() {
    let class = StaticClass::ReversePolarity;
    let [bad, good] = pair(class);
    let ends = [end("capacitor", "plus"), end("capacitor", "minus")];
    for (schematic, status, voltage_expected) in
        [(&bad, Status::Fail, -5.0), (&good, Status::Pass, 5.0)]
    {
        let report = schematic.check_static();
        assert_eq!(report.status(class), status);
        let [finding] = &report.reverse_polarity[..] else {
            panic!("ожидался один полярный элемент");
        };
        assert_eq!(finding.evidence.ends, ends);
        assert_eq!(
            finding.evidence.reverse_limit_v.range.as_ref().unwrap().max,
            0.0
        );
        // При установившемся DC ток идеальной ёмкости равен нулю, падение на стыках нулевое.
        near(finding.evidence.voltage_v.unwrap(), voltage_expected);
        near(current(&schematic.dc(), section("feed")), 0.0);
        let Analysis::Analyzed { value, .. } = schematic.netlist() else {
            panic!("нет нетлиста");
        };
        assert!(value.origins.values().any(|origin| *origin
            == Section::Element {
                component: "capacitor".into(),
                element: "cap".into(),
            }));
        assert!(value.nodes.contains_key(&ends[0]) && value.nodes.contains_key(&ends[1]));
        let sheet = schematic.assembly_sheet();
        assert!(
            sheet
                .inspections
                .iter()
                .any(|i| i.ends == ends && i.expected == Expected::PositiveMark)
        );
        assert!(sheet.inspections.iter().any(|i| i.ends == ends
            && i.stage == Stage::CurrentLimitedStart
            && matches!(i.expected, Expected::ReverseVoltageV(_))));
    }
    assert!(
        bad.check_static().reverse_polarity[0]
            .reason
            .contains("превышает допустимое")
    );
    println!("reverse_polarity: V(+) − V(−) = −5 / +5 В при обратном пределе 0 В");
}

fn fixture(class: StaticClass, file: &str) -> String {
    fs::read_to_string(corpus(class, file)).unwrap()
}

fn parse(source: &str, catalog: &Catalog) -> Result<CompiledSchematic, String> {
    parse_schematic(Path::new("блок.toml"), source.as_bytes(), catalog)
}

#[test]
fn malformed_schematics_report_the_location() {
    let catalog = catalog();
    let source = fixture(StaticClass::AssemblyMismatch, "fixed.toml");
    for (changed, reason) in [
        (
            source.replacen("nominal = 0.1", "nominal = nan", 1),
            "resistance_ohm",
        ),
        (
            source.replacen("nominal = 0.1", "nominal = inf", 1),
            "resistance_ohm",
        ),
        (
            source.replace("adapter/a", "adapter/unknown"),
            "неизвестный вывод",
        ),
        (source.replace("adapter/b", "adapter/a"), "занят дважды"),
        (
            source.replace("after = [\"return\"]", "after = [\"load_end\"]"),
            "цикл",
        ),
        (
            source.replace("scope = \"reference\"", "scope = \"assembly\""),
            "эталон вне корпуса",
        ),
        (
            source.replace("format = 1", "format = 1\nunknown = 1"),
            "unknown",
        ),
        (source.replace("format = 1", "format = 2"), "format"),
        (
            source.replace(
                "nets = { a = \"supply\", b = \"supply\" }",
                "nets = { a = \"return\", b = \"supply\" }",
            ),
            "одной объявленной цепи",
        ),
    ] {
        assert_ne!(changed, source);
        let error = parse(&changed, &catalog).unwrap_err();
        assert!(
            error.contains("блок.toml") && error.contains(reason),
            "{error}"
        );
    }
}

#[test]
fn catalog_covers_the_required_parts_and_rejects_bad_numbers() {
    use PartKind::*;
    let input = load_catalog(&root().join("catalog/electrical")).unwrap();
    assert_eq!(input.sources.len(), 2);
    let kinds = input
        .catalog
        .parts()
        .values()
        .map(|p| p.kind)
        .collect::<BTreeSet<_>>();
    for kind in [
        Resistor,
        PolarCapacitor,
        Capacitor,
        Diode,
        Nmos,
        Pmos,
        SwitchModule,
        LinearRegulator,
        SwitchingRegulator,
        ChargePowerModule,
        UsbSource,
        Battery,
        BenchSupply,
        Wire,
        ScrewTerminal,
        Header,
        SolderJoint,
    ] {
        assert!(kinds.contains(&kind), "{kind:?}");
    }

    let catalog_file = root().join("catalog/electrical/reference.toml");
    let source = fs::read_to_string(&catalog_file).unwrap();
    for invalid in ["nan", "-inf"] {
        let changed = source.replacen("nominal = 5.0", &format!("nominal = {invalid}"), 1);
        let error = parse_catalog(&catalog_file, changed.as_bytes()).unwrap_err();
        assert!(
            error.contains("reference.toml") && error.contains("volt"),
            "{error}"
        );
    }
}

#[test]
fn floating_capacitor_and_unproved_limits_do_not_pass() {
    let catalog = catalog();
    let class = StaticClass::ReversePolarity;
    let source = fixture(class, "fixed.toml");
    let floating = source.split("[connections.feed]").next().unwrap();
    let report = parse(floating, &catalog).unwrap().check_static();
    assert_eq!(report.status(class), Status::NotAnalyzed);
    assert_eq!(report.reverse_polarity[0].evidence.voltage_v, None);
    assert!(report.reverse_polarity[0].reason.contains("плавающий"));

    let mut parts = catalog.parts().clone();
    let part = parts.get_mut("reference_polar_capacitor").unwrap();
    let limit = &mut part.ratings.get_mut("reverse").unwrap().limit;
    limit.range.as_mut().unwrap().provenance =
        sim_world::Provenance::Estimate("неподтверждённый предел".into());
    let catalog = Catalog::new(parts).unwrap();
    let report = parse(&source, &catalog).unwrap().check_static();
    assert_eq!(report.status(class), Status::NotAnalyzed);
    assert!(
        report.reverse_polarity[0]
            .reason
            .contains("допустимое обратное напряжение")
    );
}

#[test]
fn binding_must_match_resolved_board_pin_and_function() {
    let catalog = catalog();
    let source = fixture(StaticClass::MissingGround, "fixed.toml");
    let source = format!(
        "{source}\n[[bindings]]\nends = [\"receiver/input\"]\ntarget = {{ kind = \"board_pin\", board = \"плата\", pin = \"p3\", function = \"adc\" }}\n"
    );
    let compiled = parse(&source, &catalog).unwrap();
    let target = |pin: &str, function| BindingTarget::BoardPin {
        board: "плата".into(),
        pin: pin.into(),
        function,
    };
    compiled
        .validate_bindings(&BTreeSet::from([target("p3", PinFunction::Adc)]))
        .unwrap();
    for installed in [
        target("p4", PinFunction::Adc),
        target("p3", PinFunction::GpioInput),
    ] {
        assert!(
            compiled
                .validate_bindings(&BTreeSet::from([installed]))
                .is_err()
        );
    }
    assert!(
        parse(
            &source.replace("function = \"adc\"", "function = \"pwm\""),
            &catalog
        )
        .is_err()
    );
    let repeated = source
        .lines()
        .skip_while(|line| *line != "[[bindings]]")
        .collect::<Vec<_>>()
        .join("\n");
    assert!(
        parse(&format!("{source}\n{repeated}"), &catalog)
            .unwrap_err()
            .contains("повторная привязка")
    );
}

#[test]
fn linked_schemes_use_the_same_parser_and_preserve_input_bytes() {
    struct Temp(PathBuf);
    impl Drop for Temp {
        fn drop(&mut self) {
            fs::remove_dir_all(&self.0).unwrap();
        }
    }
    let dir = std::env::temp_dir().join(format!("sim-electrical-{}", std::process::id()));
    fs::create_dir(&dir).unwrap();
    let temp = Temp(dir);
    let world_path = temp.0.join("world.toml");
    let schematic_path = temp.0.join("scheme.toml");
    let bytes = fs::read(corpus(StaticClass::MissingGround, "unsafe.toml")).unwrap();
    fs::write(&schematic_path, &bytes).unwrap();
    fs::write(&world_path, include_str!("fixtures/rotary/world.toml")).unwrap();
    let device = include_str!("fixtures/rotary/device.toml")
        .split_once("[[drives]]")
        .unwrap()
        .0;
    let device = device.replace("format = 3", "format = 3\nschematic = \"scheme.toml\"");
    fs::write(temp.0.join("device.toml"), device).unwrap();

    let catalog = catalog();
    let linked = sim_world::load_and_compile_with_schematics(&world_path).unwrap();
    let loaded = load_world_schematics(&linked, &catalog, &BTreeMap::new()).unwrap();
    assert_eq!(loaded["стенд"].path, schematic_path.canonicalize().unwrap());
    assert_eq!(loaded["стенд"].bytes, bytes);
    assert_eq!(
        loaded["стенд"]
            .compiled
            .check_static()
            .status(StaticClass::MissingGround),
        Status::Fail,
    );

    // Один шаг исходной сцены в 1 мс. Заявленная схема не даёт физическому прогону закрыть доказательство.
    let args = [
        world_path.into_os_string(),
        "--duration".into(),
        "0.001".into(),
        "--json".into(),
    ];
    let mut json = Vec::new();
    let mut diagnostics = Vec::new();
    assert_eq!(
        sim_cli::run(args, std::io::empty(), &mut json, &mut diagnostics),
        0
    );
    let report: serde_json::Value = serde_json::from_slice(&json).unwrap();
    assert_eq!(report["status"], "completed");
    assert_eq!(report["blocks_proof"], true);
    assert_eq!(report["proof_status"], "not_analyzed");
    assert!(
        report["simplifications"]
            .as_array()
            .unwrap()
            .iter()
            .any(|item| item["subject"] == "стенд/электрическая схема"
                && item["blocks_proof"] == true)
    );
}

#[test]
fn protection_diode_does_not_make_a_common_return() {
    let catalog = catalog();
    let class = StaticClass::MissingGround;
    let mut source = fixture(class, "unsafe.toml");
    source += r#"
[components.protection]
part = "diode_1n4148"
nets = { a = "return", k = "return" }

[connections.anode]
kind = "solder"
ends = ["sender/minus", "protection/a"]
resistance_ohm = { measurement = "Измерить стык" }
current_limit_a = { measurement = "Проверить предел" }

[connections.cathode]
kind = "solder"
ends = ["receiver/return", "protection/k"]
resistance_ohm = { measurement = "Измерить стык" }
current_limit_a = { measurement = "Проверить предел" }
"#;
    let report = parse(&source, &catalog).unwrap().check_static();
    assert_eq!(report.status(class), Status::Fail);
    assert!(report.missing_ground[0].evidence.return_path.is_none());
}

#[test]
fn a_conductor_between_differently_named_nets_still_needs_common_return() {
    let catalog = catalog();
    let class = StaticClass::MissingGround;
    let mut source = fixture(class, "unsafe.toml");
    source = source
        .replace(
            "nets = [\"signal\", \"return\"]",
            "nets = [\"signal\", \"return\", \"receiver_side\"]",
        )
        .replace(
            "nets = { input = \"signal\", return = \"return\" }",
            "nets = { input = \"receiver_side\", return = \"return\" }",
        )
        .replace(
            "ends = [\"sender/plus\", \"receiver/input\"]",
            "ends = [\"sender/plus\", \"wire/a\"]",
        );
    source += r#"
[components.wire]
part = "solder_joint"
nets = { a = "signal", b = "receiver_side" }

[connections.signal_receiver]
kind = "solder"
ends = ["wire/b", "receiver/input"]
resistance_ohm = { measurement = "Измерить стык" }
current_limit_a = { measurement = "Проверить предел" }
"#;
    let report = parse(&source, &catalog).unwrap().check_static();
    assert_eq!(report.status(class), Status::Fail);
    assert_eq!(report.missing_ground[0].evidence.signal.len(), 3);
    assert!(report.unmatched_signals.is_empty());
}

#[test]
fn reversed_source_trips_the_load_rail_and_an_unrated_supply_stays_open() {
    let catalog = catalog();
    let class = StaticClass::ReversePolarity;
    let source = fixture(class, "fixed.toml").replace("reference_polar_capacitor", "oriented_load");
    let rail = r#"
[rails.load]
ends = ["capacitor/plus", "capacitor/minus"]
reverse_limit_v = { measurement = "Проверить допустимое обратное напряжение", range = { min = 0.0, nominal = 0.0, max = 0.0, source = { derived = "В этой постановке обратное питание исключено" } } }
"#;
    let binding = r#"
[[bindings]]
ends = ["capacitor/plus", "capacitor/minus"]
target = { kind = "board_power", board = "плата", rail = "питание" }
"#;
    let mut parts = catalog.parts().clone();
    let mut load = parts["reference_polar_capacitor"].clone();
    load.kind = PartKind::Capacitor;
    load.ratings.clear();
    parts.insert("oriented_load".into(), load);
    let catalog = Catalog::new(parts).unwrap();
    let report = parse(&format!("{source}{rail}"), &catalog)
        .unwrap()
        .check_static();
    assert_eq!(report.status(class), Status::Pass);

    // Сохраняем ориентацию нагрузки, переставляем только контакты источника.
    let source = source
        .replacen(
            "nets = { plus = \"feed\", minus = \"return\" }",
            "nets = { plus = \"return\", minus = \"feed\" }",
            1,
        )
        .replace(
            "ends = [\"supply/minus\", \"capacitor/minus\"]",
            "ends = [\"supply/plus\", \"capacitor/minus\"]",
        )
        .replace(
            "ends = [\"supply/plus\", \"capacitor/plus\"]",
            "ends = [\"supply/minus\", \"capacitor/plus\"]",
        );
    let bad = parse(&format!("{source}{rail}"), &catalog)
        .unwrap()
        .check_static();
    assert_eq!(bad.status(class), Status::Fail);
    near(bad.reverse_polarity[0].evidence.voltage_v.unwrap(), -5.0);
    let open = parse(&format!("{source}{binding}"), &catalog)
        .unwrap()
        .check_static();
    assert_eq!(open.status(class), Status::NotAnalyzed);
    assert!(
        open.reverse_polarity[0]
            .reason
            .contains("по паспорту нагрузки")
    );
}
