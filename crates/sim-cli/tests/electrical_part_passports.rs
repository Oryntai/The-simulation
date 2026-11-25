use std::{fmt::Write, path::Path};

use sim_cli::electrical::{SchematicInput, load_catalog, parse_catalog, parse_schematic};
use sim_process::{Analysis, schematic::*};

const MODULE: &str = "esp32_s3_n16r8";
const CAMERA: &str = "ov2640_arducam_u3359";
const CELL: &str = "lipo_803040_1000mah";

// Порядок площадок сверен с рисунком 3-1 и таблицей 3-1 Espressif v1.8.
const MODULE_PINS: &[&str] = &[
    "gnd",
    "vdd",
    "en",
    "io4",
    "io5",
    "io6",
    "io7",
    "io15",
    "io16",
    "io17",
    "io18",
    "io8",
    "io19",
    "io20",
    "io3",
    "io46",
    "io9",
    "io10",
    "io11",
    "io12",
    "io13",
    "io14",
    "io21",
    "io47",
    "io48",
    "io45",
    "io0",
    "reserved_io35",
    "reserved_io36",
    "reserved_io37",
    "io38",
    "io39",
    "io40",
    "io41",
    "io42",
    "rxd0",
    "txd0",
    "io2",
    "io1",
    "gnd_40",
    "epad",
];
// Порядок позиций разъёма по таблице §5 Arducam U3359 Rev. A.
const CAMERA_PINS: &[&str] = &[
    "vcc", "gnd", "scl", "sdata", "vsync", "href", "pclk", "xclk", "dout9", "dout8", "dout7",
    "dout6", "dout5", "dout4", "dout3", "dout2", "dout1", "dout0", "rst", "pwdn",
];
const CELL_PINS: &[&str] = &["plus", "minus"];

fn catalog_dir() -> &'static Path {
    Path::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../catalog/electrical"
    ))
}

fn block(part: &str, pins: &[&str]) -> String {
    let mut schematic = format!(
        "format = 1\nname = 'блок'\nscope = 'assembly'\nnets = {pins:?}\n\
         [components.block]\npart = '{part}'\n[components.block.nets]\n"
    );
    for pin in pins {
        writeln!(schematic, "{pin} = '{pin}'").unwrap();
    }
    schematic
}

#[test]
fn passport_pin_roles_and_limits_match_manufacturer_tables() {
    let input = load_catalog(catalog_dir()).unwrap();
    let module = input.catalog.part(MODULE).unwrap();
    assert_eq!(module.pins.len(), MODULE_PINS.len());
    for &name in MODULE_PINS {
        let pin = &module.pins[name];
        let role = match name {
            "vdd" => PinRole::Power,
            "gnd" | "gnd_40" | "epad" => PinRole::Return,
            "en" => PinRole::SignalIn("gnd".into()),
            "reserved_io35" | "reserved_io36" | "reserved_io37" => PinRole::Passive,
            _ => PinRole::SignalIo("gnd".into()),
        };
        assert_eq!(pin.role, role, "{name}");
        assert_eq!(pin.contact.kind, ContactKind::Pad, "{name}");
    }
    // Нижняя и верхняя границы VDD33 из таблицы 6-2 Espressif v1.8.
    for (name, expected) in [
        ("minimum_supply_voltage", 3.0),
        ("maximum_supply_voltage", 3.6),
    ] {
        let rating = &module.ratings[name];
        assert_eq!(rating.nodes, ["vdd", "gnd"]);
        let range = rating.limit.range.as_ref().unwrap();
        assert_eq!([range.min, range.nominal, range.max], [expected; 3]);
        assert!(range.can_prove());
    }

    let camera = input.catalog.part(CAMERA).unwrap();
    assert_eq!(camera.pins.len(), CAMERA_PINS.len());
    for (index, &name) in CAMERA_PINS.iter().enumerate() {
        let pin = &camera.pins[name];
        let role = match name {
            "vcc" => PinRole::Power,
            "gnd" => PinRole::Return,
            "sdata" => PinRole::SignalIo("gnd".into()),
            "scl" | "xclk" | "rst" | "pwdn" => PinRole::SignalIn("gnd".into()),
            _ => PinRole::SignalOut("gnd".into()),
        };
        assert_eq!(pin.role, role, "{name}");
        assert_eq!(pin.contact.kind, ContactKind::Plug, "{name}");
        assert_eq!(pin.contact.family.as_deref(), Some("arducam_dvp_20"));
        assert_eq!(pin.contact.key, Some((index + 1).to_string()), "{name}");
    }
    // Ключи 1 и 2 из §5 различают питание и возврат даже при одном семействе.
    let vcc = &camera.pins["vcc"].contact;
    let socket = Contact {
        kind: ContactKind::Socket,
        ..vcc.clone()
    };
    assert!(JointKind::Plug.mates(vcc, &socket));
    assert!(!JointKind::Plug.mates(&camera.pins["gnd"].contact, &socket));

    let cell = input.catalog.part(CELL).unwrap();
    assert_eq!(cell.kind, PartKind::Battery);
    assert_eq!(cell.pins["plus"].role, PinRole::Power);
    assert_eq!(cell.pins["minus"].role, PinRole::Return);
    assert!(cell.source.contains("1000 мА·ч — заявленный номинал"));
    assert!(cell.elements["internal"].parameters["ohm"].range.is_none());
    // Длительный ток двух аналогов определён в §3 паспортов Honcell.
    let output = cell.ratings["output"].limit.range.as_ref().unwrap();
    assert_eq!([output.min, output.max], [0.5, 1.0]);
    assert!(!output.can_prove());
}

#[test]
fn assembly_loads_each_part_and_keeps_supply_analysis_open() {
    let input = load_catalog(catalog_dir()).unwrap();
    for (id, file, pins, power, ground, gap) in [
        (
            MODULE,
            "esp32-s3-n16r8.toml",
            MODULE_PINS,
            "vdd",
            "gnd",
            "supply_load/ampere",
        ),
        (
            CAMERA,
            "ov2640.toml",
            CAMERA_PINS,
            "vcc",
            "gnd",
            "supply_load/ampere",
        ),
        (
            CELL,
            "lipo-803040-1000mah.toml",
            CELL_PINS,
            "plus",
            "minus",
            "emf/volt",
        ),
    ] {
        assert!(input.sources.contains_key(&catalog_dir().join(file)));
        let definition = SchematicInput::parse(
            Path::new("блок.toml"),
            block(id, pins).into_bytes(),
            &input.catalog,
        )
        .unwrap();
        let compiled = &definition.compiled;
        assert_eq!(compiled.pins().count(), pins.len());
        let sheet = compiled.assembly_sheet();
        let [part] = sheet.parts.as_slice() else {
            panic!("ожидалась одна позиция в монтажном листе");
        };
        assert_eq!(part.part, id);
        assert_eq!(part.pins[power], power);
        assert_eq!(part.pins[ground], ground);
        let measurement = &part.missing_parameters[&format!("elements/{gap}")];
        assert!(part.model_gap.is_some());
        assert!(
            matches!(compiled.netlist(), Analysis::NotAnalyzed { ref reason }
            if reason.contains(measurement))
        );
        let end = |pin: &str| Endpoint {
            component: "block".into(),
            pin: pin.into(),
        };
        assert!(matches!(
            compiled.dc().voltage(&end(power), &end(ground)),
            Analysis::NotAnalyzed { .. }
        ));
        let assessment = compiled.assess_electrical(&definition.modes).unwrap();
        assert!(!assessment.findings.is_empty());
        assert!(
            assessment
                .findings
                .iter()
                .all(|finding| finding.status == Status::NotAnalyzed),
            "{:?}",
            assessment.findings
        );
    }
}

#[test]
fn unknown_and_reserved_signal_names_and_nonfinite_ranges_are_rejected() {
    let input = load_catalog(catalog_dir()).unwrap();
    let path = Path::new("блок.toml");
    for (id, pins, old, new) in [
        (MODULE, MODULE_PINS, "reserved_io35 =", "io35 ="),
        (CAMERA, CAMERA_PINS, "xclk =", "clk ="),
        (CELL, CELL_PINS, "minus =", "ntc ="),
    ] {
        let text = block(id, pins).replace(old, new);
        let error = parse_schematic(path, text.as_bytes(), &input.catalog).unwrap_err();
        assert!(error.contains("каждый каталожный вывод"), "{error}");
    }

    let passport = catalog_dir().join("esp32-s3-n16r8.toml");
    let data = std::str::from_utf8(&input.sources[&passport]).unwrap();
    for nonfinite in ["nan", "inf", "-inf"] {
        let invalid = data.replacen("min = 3.0", &format!("min = {nonfinite}"), 1);
        assert_ne!(invalid, data);
        let error = parse_catalog(&passport, invalid.as_bytes()).unwrap_err();
        assert!(error.contains("допустимы только конечные числа"), "{error}");
    }
}
