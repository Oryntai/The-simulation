//! Сквозной опыт платы на нейтральном блоке. Ручной оракул — depth_t4/expected.md.

use std::{
    fs,
    path::PathBuf,
    process::{Command, Output},
};

use serde_json::Value;
use sim_task::{
    Fingerprint, Outcome,
    verdict::{Finding, HazardClass},
};

const FIXED: &str = include_str!("../../../devices/electrical/depth_t4/fixed.kicad_pcb");
const UNSAFE: &str = include_str!("../../../devices/electrical/depth_t4/unsafe.kicad_pcb");

// Ручная таблица клемм, независимая от разбора KiCad.
const NETS: &[(&str, &[&str])] = &[
    ("PWR", &["J1/1", "J2/1"]),
    ("SENSE_A", &["J3/1", "J5/1"]),
    ("SENSE_B", &["J3/2", "J5/2"]),
    ("LINK", &["J1/2", "J4/1"]),
    ("GND", &["J6/1", "J7/1", "J8/1"]),
    ("PROBE_A", &["J9/1", "J10/1"]),
    ("PROBE_B", &["J9/2", "J10/2"]),
];

const EXPECTED: &str = "Ручной эталон devices/electrical/depth_t4/expected.md";

struct Block {
    dir: PathBuf,
    rules: String,
    scheme: String,
}

impl Drop for Block {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.dir).expect("удаление временного блока");
    }
}

impl Block {
    fn new(label: &str) -> Self {
        let dir = std::env::temp_dir().join(format!("sim-depth-t4-{}-{label}", std::process::id()));
        fs::create_dir(&dir).unwrap();
        let mut block = Self {
            dir,
            rules: String::new(),
            scheme: String::new(),
        };
        fs::create_dir_all(block.dir.join("catalog/electrical")).unwrap();
        block.write(
            "catalog/electrical/parts.toml",
            r#"
format = 1
[parts.terminal]
kind = "header"
source = "Пассивные клеммы стенда"
model_gap = "Задана только геометрия внешней меди"
[parts.terminal.pins.pin]
contact = "pad"
role = "passive"
"#,
        );
        block.write("device.toml", r#"
format = 3
name = "Блок"
schematic = "scheme.toml"
[[links]]
name = "корпус"
joint = { kind = "slide", axis = [1.0, 0.0, 0.0] }
# Куб 1 м и 6 кг, I = m·a²/6 = 1 кг·м².
mass = { kind = "explicit", mass_kg = 6.0, com_m = [0.0, 0.0, 0.0], inertia_kg_m2 = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0], provenance = { derived = "Инерция однородного куба" } }
"#);
        block.write(
            "world.toml",
            r#"
format = 3
name = "Блок"
seed = 1 # Фиксированное зерно постановки.
physics_hz = 1000 # Сетка носителя из стенда electrical_verdict.
gravity_m_s2 = [0.0, 0.0, 0.0] # Носитель неподвижен.
[environment]
density_kg_m3 = 0.0
viscosity_pa_s = 0.0
wind_m_s = [0.0, 0.0, 0.0]
[[devices]]
name = "блок"
file = "device.toml"
"#,
        );
        block.write(
            "pcb.toml",
            r#"
format = 1
[instances."блок"]
board = "block.kicad_pcb"
rules = "rules.toml"
"#,
        );
        let nets = NETS
            .iter()
            .map(|(name, _)| format!("{name:?}"))
            .collect::<Vec<_>>()
            .join(", ");
        block.scheme =
            format!("format = 1\nname = \"Блок\"\nscope = \"reference\"\nnets = [{nets}]\n");
        block.rules = "format = 1\n".into();
        for (net, terminals) in NETS {
            for address in *terminals {
                let (reference, pad) = address.split_once('/').unwrap();
                let component = format!("клемма-{reference}-{pad}");
                block.scheme.push_str(&format!(
                    "[components.{component:?}]\npart = \"terminal\"\nnets.pin = {net:?}\n"
                ));
                block.rules.push_str(&format!(
                    "[[bindings]]\nreference = {reference:?}\npad = {pad:?}\ncomponent = {component:?}\npin = \"pin\"\n"
                ));
            }
        }
        block.write("scheme.toml", &block.scheme);
        block
    }

    fn write(&self, name: &str, bytes: impl AsRef<[u8]>) {
        fs::write(self.dir.join(name), bytes).unwrap();
    }

    fn profile(&self, checks: &str) -> String {
        let stamp = Fingerprint::of(self.scheme.as_bytes()).as_u64();
        self.rules.replacen(
            "format = 1",
            &format!("format = 1\nschematic_fingerprint = \"{stamp:016x}\""),
            1,
        ) + checks
    }

    fn launch(&self, with_board: bool) -> Output {
        let board = if with_board {
            "pcb = \"pcb.toml\"\n"
        } else {
            ""
        };
        self.write("campaign.toml", format!(
            "electrical_format = 1\ncatalog = \"catalog\"\n[assemblies.block]\nworld = \"world.toml\"\n{board}"
        ));
        Command::new(env!("CARGO_BIN_EXE_sim-run"))
            .arg("--campaign")
            .arg(self.dir.join("campaign.toml"))
            .arg("--json")
            .output()
            .unwrap()
    }

    fn run(&self, board: &str, checks: &str) -> Value {
        self.write("block.kicad_pcb", board);
        self.write("rules.toml", self.profile(checks));
        let output = self.launch(true);
        assert!(output.status.success(), "{output:?}",);
        serde_json::from_slice::<Value>(&output.stdout).unwrap()["assemblies"]["block"].clone()
    }

    fn error(&self, board: &str, rules: &str, expected: &str) {
        self.write("block.kicad_pcb", board);
        self.write("rules.toml", rules);
        let output = self.launch(true);
        assert!(!output.status.success(), "{output:?}");
        let error: Value = serde_json::from_slice(&output.stdout).unwrap();
        assert_eq!(error["status"], "error", "{output:?}");
        assert_eq!(error["proof_status"], "not_analyzed", "{output:?}");
        assert!(
            error["detail"].as_str().unwrap().contains(expected),
            "{error} вместо {expected}"
        );
    }
}

fn datum(value: f64) -> String {
    format!("{{ value = {value:?}, source = {{ reference = {EXPECTED:?} }} }}")
}

fn track(last: &str, end: [i32; 2]) -> String {
    format!(
        "{{ kind = \"track\", track = \"bd430000-0000-4000-8000-0000000000{last}\", to_nm = {end:?} }}"
    )
}

fn inputs(shared: bool) -> String {
    // Прямой пересчёт единиц из ручных координат, источники в expected.md.
    let power1 = track("52", [10_000_000, 23_000_000]);
    let power2 = track("53", [15_000_000, 23_000_000]);
    let signal = if shared {
        format!("{power1}, {}", track("59", [10_000_000, 28_000_000]))
    } else {
        format!(
            "{}, {}",
            track("54", [5_000_000, 28_000_000]),
            track("55", [10_000_000, 28_000_000])
        )
    };
    let plane = format!(
        r#"{{ zone = 0, layer = "B.Cu", tie_nm = [5000000,9000000], connection = {{ reference = {EXPECTED:?} }} }}"#
    );
    let sense_return = [
        track("43", [24_000_000, 8_000_000]),
        track(
            "44",
            [24_000_000, if shared { 5_500_000 } else { 6_200_000 }],
        ),
        track(
            "45",
            [30_000_000, if shared { 5_500_000 } else { 6_200_000 }],
        ),
        track("46", [30_000_000, 8_000_000]),
        track("47", [32_000_000, 8_000_000]),
    ]
    .join(", ");
    let sense_forward = track("42", [32_000_000, 5_000_000]);
    // Нули напряжения и частоты описывают отдельное синтетическое сравнение
    // геометрии. Порог 0,25 мм применяется к обеим метрикам только в этом опыте.
    let context = format!(
        r#"{{ source = {{ reference = {EXPECTED:?} }}, material = "Диэлектрик эталона", environment = "Синтетическая постановка геометрии", working_v = 0.0, peak_v = 0.0, frequency_hz = 0.0, altitude_m = 0.0, distance_error_m = 0.0 }}"#
    );
    let alpha =
        "{ value = 0.00393, source = { reference = \"NBS Handbook 100 и IEC 60028, pcb.rs\" } }";
    let copper = format!(
        "{{ rho_20_ohm_m = {}, alpha_per_k = {alpha} }}",
        datum(1.68e-8)
    );
    format!(
        r#"
[checks.gap]
kind = "clearance"
face = "front"
nets = ["SENSE_A", "SENSE_B"]
conditions = {context}
limits = {{ envelope = {context}, clearance_m = 0.00025, creepage_m = 0.00025 }}
[checks.probe]
kind = "clearance"
face = "front"
nets = ["PROBE_A", "PROBE_B"]
conditions = {context}
limits = {{ envelope = {context}, clearance_m = 0.00025 }}
[checks.bounce]
kind = "ground_bounce"
power = {{ net = "GND", start = {{ at_nm = [5000000,23000000], layer = "F.Cu" }}, steps = [{power1}, {power2}] }}
signal = {{ net = "GND", start = {{ at_nm = [5000000,23000000], layer = "F.Cu" }}, steps = [{signal}] }}
topology = {{ reference = {EXPECTED:?} }}
resistivity_ohm_m = {rho}
shared_inductance_h = {inductance}
current_a = {amp}
slew_a_per_s = {slew}
max_bounce_v = {max_bounce}
# Ноль для вкладов, исключённых моделью звезды из expected.md.
omitted_bounce_v = {omitted}
[checks.power]
kind = "ratings"
net = "PWR"
reference = {{ at_nm = [5000000,6000000], layer = "F.Cu" }}
loads = [{{ node = {{ at_nm = [15000000,6000000], layer = "F.Cu" }}, current_a = {amp}, max_drop_v = {drop} }}]
topology = {{ reference = {EXPECTED:?} }}
copper = {copper}
copper_temp_c = {temp}
max_rise_k = {rise}
thermal_model = {{ reference = {EXPECTED:?} }}
[checks.link]
kind = "ratings"
net = "LINK"
reference = {{ at_nm = [5000000,13000000], layer = "F.Cu" }}
# Единичный испытательный ток для сверки R по падению, как в pcb_ratings.rs.
loads = [{{ node = {{ at_nm = [15000000,13000000], layer = "B.Cu" }}, current_a = {amp} }}]
topology = {{ reference = {EXPECTED:?} }}
copper = {copper}
copper_temp_c = {temp}
vias = [{{ via = 0, plating_m = {plating}, min_ring_m = {ring} }}]
[checks.plane]
kind = "reference"
track = "bd430000-0000-4000-8000-000000000042"
plane = {plane}
[checks.z0]
kind = "impedance"
reference = {{ track = "bd430000-0000-4000-8000-000000000042", plane = {plane} }}
[checks.loop_geometry]
kind = "decoupling"
# Ручная сверка размеров двух путей по таблице SENSE_A/B.
# Данных о конденсаторе и пределе контура в эталоне нет.
supply = {{ net = "SENSE_A", start = {{ at_nm = [22000000,5000000], layer = "F.Cu" }}, steps = [{sense_forward}] }}
return_arm = {{ net = "SENSE_B", start = {{ at_nm = [22000000,8000000], layer = "F.Cu" }}, steps = [{sense_return}] }}
load = [{{ reference = "J3", pad = "1", layer = "F.Cu" }}, {{ reference = "J3", pad = "2", layer = "F.Cu" }}]
capacitor = [{{ reference = "J5", pad = "1", layer = "F.Cu" }}, {{ reference = "J5", pad = "2", layer = "F.Cu" }}]
"#,
        rho = datum(1.68e-8),
        inductance = datum(10e-9),
        amp = datum(1.0),
        slew = datum(1.0 / 100e-9),
        max_bounce = datum(0.050),
        omitted = datum(0.0),
        drop = datum(0.020),
        temp = datum(20.0),
        rise = datum(10.0),
        plating = datum(25e-6),
        ring = datum(0.15e-3)
    )
}

fn replace(source: &str, from: &str, to: &str) -> String {
    assert!(source.contains(from), "нет ручной метки {from:?}");
    source.replace(from, to)
}

fn remove_short(board: &str) -> String {
    let line = board
        .lines()
        .find(|line| line.contains("000000000058"))
        .unwrap();
    replace(board, line, "")
}

fn repair_link(board: &str) -> String {
    replace(board, "(end 14.5 13)", "(end 15 13)")
}

fn pcb(report: &Value) -> &Value {
    &report["assemblies"]["блок"]["pcb"]
}

fn finding(report: &Value, class: HazardClass, id: &str) -> Finding {
    report["findings"]
        .as_array()
        .unwrap()
        .iter()
        .cloned()
        .map(|row| serde_json::from_value::<Finding>(row).unwrap())
        .find(|row| {
            row.key.class == class
                && row
                    .key
                    .extension
                    .as_deref()
                    .is_some_and(|value| value.ends_with(&format!("/{id}")))
        })
        .unwrap()
}

fn near(value: &Value, expected: f64, tolerance: f64) {
    let actual = value
        .as_f64()
        .unwrap_or_else(|| panic!("нет числа у {value:?}"));
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual:.12e} вместо {expected:.12e}"
    );
}

fn contacts_and_input_errors() {
    let block = Block::new("contacts");
    let rules = block.profile("");
    block.error(UNSAFE, &rules, "короткое замыкание");
    block.error(&remove_short(UNSAFE), &rules, "обрыв цепи «LINK»");
    let wrong_pin = replace(&rules, "pin = \"pin\"", "pin = \"unknown\"");
    block.error(FIXED, &wrong_pin, "unknown");
    block.error(
        &replace(FIXED, "(version 20240108)", "(version 19000101)"),
        &rules,
        "версия",
    );
    block.error(
        FIXED,
        &replace(&rules, "schematic_fingerprint", "extra"),
        "unknown field",
    );
    block.write(
        "scheme.toml",
        format!("{}\n# Другая ревизия\n", block.scheme),
    );
    block.error(FIXED, &rules, "отпечаток схемы не совпадает");
}

fn paired_hazards_and_independent_hand_calculation() {
    let block = Block::new("paired");
    let repaired = repair_link(&remove_short(UNSAFE));
    let bad = block.run(&repaired, &inputs(true));
    let good = block.run(FIXED, &inputs(false));
    for (report, status, expected) in [
        (&bad, Outcome::Fail, "Rejected"),
        (&good, Outcome::Pass, "NotProven"),
    ] {
        assert_eq!(report["simulation"], expected, "{report}");
        let pcb = pcb(report);
        assert_eq!(pcb["kicad_version"], 20240108);
        assert_eq!(pcb["copper_layers"], serde_json::json!(["F.Cu", "B.Cu"]));
        assert_eq!(pcb["connectivity"]["groups"], 7);
        assert_eq!(pcb["connectivity"]["terminals"], 15);
        assert_eq!(pcb["connectivity"]["status"], "pass");
        for (class, id) in [
            (HazardClass::CreepageClearance, "gap"),
            (HazardClass::GroundBounce, "bounce"),
        ] {
            let row = finding(report, class, id);
            assert_eq!(row.outcome, status, "{row:?}");
            assert_eq!(row.confirmed_hazard, status == Outcome::Fail);
            let identity = row.electrical.unwrap();
            assert_eq!(identity.build.as_u64(), report["build"].as_u64().unwrap());
            assert_eq!(identity.schematic, Fingerprint::of(block.scheme.as_bytes()));
        }
        assert!(
            !report["findings"]
                .as_array()
                .unwrap()
                .iter()
                .any(|f| f["code"] == "rail_sag" && f["outcome"] == "fail")
        );
    }
    assert!(
        !good["findings"]
            .as_array()
            .unwrap()
            .iter()
            .any(|f| f["outcome"] == "fail")
    );
    for (report, width_m, gap_m, ring_m, bounce_m) in [
        (
            &bad,
            0.10e-3,
            (5.50 - 5.00 - 2.0 * 0.20) * 1e-3,
            (0.60 - 0.40) / 2.0 * 1e-3,
            0.005,
        ),
        (
            &good,
            0.80e-3,
            (6.20 - 5.00 - 2.0 * 0.20) * 1e-3,
            (0.80 - 0.40) / 2.0 * 1e-3,
            0.0,
        ),
    ] {
        // Одна единица координат KiCad для зазоров; для модели Ома — округление f64.
        let metrics = &pcb(report)["checks"];
        near(
            &metrics["gap"]["metrics"]["clearance"]["distance_m"][0],
            gap_m,
            1e-9,
        );
        near(
            &metrics["probe"]["metrics"]["clearance"]["distance_m"][0],
            2.20e-3,
            1e-9,
        );
        let r = 1.68e-8 * 0.010 / (width_m * 0.035e-3);
        let power = &metrics["power"]["metrics"];
        near(&power["branches"][0]["resistance_ohm"], r, 1e-15);
        near(&power["loads"][0]["drop_v"]["value"], 1.0 * r, 1e-15);
        let area_mil2 = width_m * 0.035e-3 / 25.4e-6_f64.powi(2);
        let amps = 0.048 * 10.0_f64.powf(0.44) * area_mil2.powf(0.725);
        near(
            &power["branches"][0]["current_limit_a"]["limit"],
            amps,
            1e-12,
        );
        near(
            &metrics["link"]["metrics"]["vias"][0]["ring_m"]["value"],
            ring_m,
            1e-15,
        );
        let via = metrics["link"]["metrics"]["branches"]
            .as_array()
            .unwrap()
            .iter()
            .find(|b| b["section"]["via"] == 0)
            .unwrap();
        let via_r = 1.68e-8 * 1.545e-3 / (std::f64::consts::PI * (0.40e-3 - 25e-6) * 25e-6);
        near(&via["resistance_ohm"], via_r, 1e-15);
        near(
            &metrics["link"]["metrics"]["loads"][0]["drop_v"]["value"],
            2.0 * 0.012 + via_r,
            1e-15,
        );
        let shared_r = 1.68e-8 * bounce_m / (0.50e-3 * 0.035e-3);
        let v = if bounce_m == 0.0 {
            0.0
        } else {
            shared_r * 1.0 + 10e-9 * (1.0 / 100e-9)
        };
        near(
            &metrics["bounce"]["metrics"]["shared_length_m"],
            bounce_m,
            1e-9,
        );
        near(&metrics["bounce"]["metrics"]["bounce_v"], v, 1e-15);
    }
    let bad_checks = &pcb(&bad)["checks"];
    let gap = &bad_checks["plane"]["metrics"]["center_gaps_m"][0];
    near(&gap[1], (29.0 - 22.0) * 1e-3, 1e-9);
    near(&gap[0], (27.0 - 22.0) * 1e-3, 1e-9);
    assert_eq!(bad_checks["z0"]["status"], "not_analyzed");
    assert_eq!(bad_checks["z0"]["metrics"]["nominal_ohm"], Value::Null);
    let fixed_checks = &pcb(&good)["checks"];
    assert_eq!(
        fixed_checks["plane"]["metrics"]["center_gaps_m"],
        serde_json::json!([])
    );
    assert_eq!(fixed_checks["z0"]["status"], "not_analyzed");
    assert_eq!(fixed_checks["loop_geometry"]["status"], "not_analyzed");
    near(
        &fixed_checks["loop_geometry"]["metrics"]["perimeter_m"],
        (10.0 + 13.6 + 3.0 + 3.0) * 1e-3,
        1e-9,
    );
    assert!(
        fixed_checks["z0"]["metrics"]["nominal_ohm"]
            .as_f64()
            .unwrap()
            > 0.0
    );
}

fn isolated_defects_publish_only_their_own_cause() {
    let block = Block::new("isolated");
    let gap_only = replace(FIXED, " 6.2)", " 5.5)");
    let step =
        |board: &'static str, id: &str| board.lines().find(|line| line.contains(id)).unwrap();
    let bounce_only = replace(
        &replace(
            FIXED,
            step(FIXED, "000000000054"),
            step(UNSAFE, "000000000059"),
        ),
        step(FIXED, "000000000055"),
        "",
    );
    for (board, shared, code) in [
        (&gap_only, false, "creepage_clearance"),
        (&bounce_only, true, "ground_bounce"),
    ] {
        let checks = inputs(shared);
        // Геометрия развязки использует координаты SENSE_B и исключена из
        // изолированного опыта, где меняется один из двух её маршрутов.
        let (checks, _) = checks.split_once("[checks.loop_geometry]").unwrap();
        let report = block.run(board, checks);
        assert_eq!(report["simulation"], "Rejected");
        assert_eq!(
            report["findings"]
                .as_array()
                .unwrap()
                .iter()
                .filter(|f| f["outcome"] == "fail")
                .map(|f| f["code"].as_str().unwrap())
                .collect::<Vec<_>>(),
            [code],
        );
    }
}

fn absent_or_unverified_inputs_cannot_pass() {
    let mut block = Block::new("open");
    let checks = inputs(false);
    let absent = block.run(FIXED, "");
    assert_eq!(pcb(&absent)["status"], "not_analyzed");
    for class in [HazardClass::CreepageClearance, HazardClass::GroundBounce] {
        assert_eq!(
            finding(&absent, class, "@coverage").outcome,
            Outcome::NotAnalyzed
        );
    }
    let unknown_limit = replace(&checks, ", creepage_m = 0.00025", "");
    let unknown_limit = replace(
        &unknown_limit,
        &format!("max_bounce_v = {}\n", datum(0.050)),
        "",
    );
    let open = block.run(FIXED, &unknown_limit);
    assert_eq!(
        finding(&open, HazardClass::CreepageClearance, "gap").outcome,
        Outcome::NotAnalyzed
    );
    assert_eq!(
        finding(&open, HazardClass::GroundBounce, "bounce").outcome,
        Outcome::NotAnalyzed
    );
    assert_eq!(
        pcb(&open)["checks"]["gap"]["metrics"]["clearance"]["status"],
        "pass"
    );
    let missing_cu = block.run(
        &replace(
            FIXED,
            r#"(type "copper") (thickness 0.035)"#,
            r#"(type "copper")"#,
        ),
        &checks,
    );
    assert_eq!(
        pcb(&missing_cu)["checks"]["power"]["status"],
        "not_analyzed"
    );
    assert_eq!(
        pcb(&missing_cu)["checks"]["power"]["metrics"]["branches"][0]["resistance_ohm"],
        Value::Null
    );
    let missing_fill = block.run(
        &replace(
            FIXED,
            r#"    (filled_polygon (layer "B.Cu")
      (pts (xy 3 3) (xy 36 3) (xy 36 10) (xy 3 10)))
"#,
            "",
        ),
        &checks,
    );
    assert_eq!(pcb(&missing_fill)["connectivity"]["status"], "not_analyzed");
    assert_eq!(
        pcb(&missing_fill)["checks"]["plane"]["status"],
        "not_analyzed"
    );
    assert_eq!(
        finding(&missing_fill, HazardClass::CreepageClearance, "gap").outcome,
        Outcome::NotAnalyzed
    );
    let malformed = replace(&checks, "creepage_m = 0.00025", "creepage_m = nan");
    block.error(
        FIXED,
        &block.profile(&malformed),
        "допустимы только конечные числа",
    );
    let extra = format!(
        "{checks}\n[checks.extra]\nkind = \"ratings\"\nnet = \"PWR\"\nreference = {{ at_nm = [5000000,6000000], layer = \"F.Cu\" }}\nextra = true\n"
    );
    block.error(FIXED, &block.profile(&extra), "unknown field");

    block.scheme = replace(
        &block.scheme,
        r#"scope = "reference""#,
        r#"scope = "assembly""#,
    );
    block.write("scheme.toml", &block.scheme);
    let physical = block.run(FIXED, &checks);
    for (class, id) in [
        (HazardClass::CreepageClearance, "gap"),
        (HazardClass::GroundBounce, "bounce"),
    ] {
        assert_eq!(finding(&physical, class, id).outcome, Outcome::NotAnalyzed);
    }
    assert_eq!(physical["simulation"], "NotProven");

    // Тот же численный стенд с типом паспортного входа доходит до
    // положительного сравнения в решателе. Для сборки нужен отдельный замер.
    let sourced = replace(
        &checks,
        "{ reference = \"Ручной эталон",
        "{ verified = \"Ручной эталон",
    );
    let physical = block.run(FIXED, &sourced);
    assert_eq!(
        pcb(&physical)["checks"]["gap"]["metrics"]["clearance"]["status"],
        "pass"
    );
    assert_eq!(
        pcb(&physical)["checks"]["bounce"]["metrics"]["bounce_v"],
        0.0
    );
    assert_eq!(
        finding(&physical, HazardClass::CreepageClearance, "gap").outcome,
        Outcome::NotAnalyzed
    );
    assert_eq!(
        finding(&physical, HazardClass::GroundBounce, "bounce").outcome,
        Outcome::NotAnalyzed
    );
    assert_eq!(physical["simulation"], "NotProven");
}

fn fingerprints_and_legacy_report() {
    let block = Block::new("identity");
    let checks = inputs(false);
    let report = block.run(FIXED, &checks);
    let output = block.launch(true);
    assert!(output.status.success(), "{output:?}");
    let again = block.launch(true);
    assert!(again.status.success(), "{again:?}");
    assert_eq!(output.stdout, again.stdout);
    for (board, checks) in [
        (
            replace(FIXED, "manual_reference", "manual_revision"),
            checks.clone(),
        ),
        (FIXED.into(), format!("{checks}\n# Ревизия правил\n")),
    ] {
        let changed = block.run(&board, &checks);
        assert_ne!(changed["build"], report["build"]);
        assert_ne!(pcb(&changed)["fingerprint"], pcb(&report)["fingerprint"]);
        let key = (HazardClass::GroundBounce, "bounce");
        assert_ne!(
            finding(&changed, key.0, key.1).electrical.unwrap().inputs,
            finding(&report, key.0, key.1).electrical.unwrap().inputs
        );
    }
    let output = block.launch(false);
    assert!(output.status.success(), "{output:?}");
    let legacy: Value = serde_json::from_slice(&output.stdout).unwrap();
    let legacy = &legacy["assemblies"]["block"];
    assert!(legacy["assemblies"]["блок"].get("pcb").is_none());
    assert!(
        !legacy["findings"]
            .as_array()
            .unwrap()
            .iter()
            .any(|f| matches!(
                f["code"].as_str(),
                Some("ground_bounce" | "creepage_clearance")
            ))
    );
}

// Все кампании стенда выполняются последовательно, в том числе при
// стандартном числе потоков тестового раннера.
#[test]
fn board_in_electrical_campaign() {
    contacts_and_input_errors();
    paired_hazards_and_independent_hand_calculation();
    isolated_defects_publish_only_their_own_cause();
    absent_or_unverified_inputs_cannot_pass();
    fingerprints_and_legacy_report();
}
