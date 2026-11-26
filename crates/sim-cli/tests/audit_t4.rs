//! Сквозной аудит доказательств платы на данных нейтрального блока.

use std::{
    fs,
    path::PathBuf,
    process::{Command, Output},
};

use serde_json::Value;
use sim_task::Fingerprint;

const FIXED: &str = include_str!("../../../devices/electrical/depth_t4/fixed.kicad_pcb");
const UNSAFE: &str = include_str!("../../../devices/electrical/depth_t4/unsafe.kicad_pcb");
// Ручная карта клемм из depth_t4/expected.md, независимо от файла платы.
const NETS: &[(&str, &[&str])] = &[
    ("PWR", &["J1/1", "J2/1"]),
    ("SENSE_A", &["J3/1", "J5/1"]),
    ("SENSE_B", &["J3/2", "J5/2"]),
    ("LINK", &["J1/2", "J4/1"]),
    ("GND", &["J6/1", "J7/1", "J8/1"]),
    ("PROBE_A", &["J9/1", "J10/1"]),
    ("PROBE_B", &["J9/2", "J10/2"]),
];

struct Block {
    dir: PathBuf,
    rules: String,
}

impl Drop for Block {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.dir).unwrap();
    }
}

impl Block {
    fn new(label: &str) -> Self {
        let dir = std::env::temp_dir().join(format!("sim-audit-t4-{}-{label}", std::process::id()));
        fs::create_dir(&dir).unwrap();
        let mut block = Self {
            dir,
            rules: String::new(),
        };
        fs::create_dir_all(block.dir.join("catalog/electrical")).unwrap();
        block.write(
            "catalog/electrical/parts.toml",
            r#"
format = 1
[parts.terminal]
kind = "header"
source = "Клеммы стенда"
model_gap = "Только медная геометрия"
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
# Однородный куб 1 м и 6 кг, I = m·a²/6.
mass = { kind = "explicit", mass_kg = 6.0, com_m = [0.0, 0.0, 0.0], inertia_kg_m2 = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0], provenance = { derived = "Инерция однородного куба" } }
"#);
        block.write(
            "world.toml",
            r#"
format = 3
name = "Блок"
seed = 1 # Фиксированное зерно стенда.
physics_hz = 1000 # Сетка неподвижного носителя, как в depth_t4.
gravity_m_s2 = [0.0, 0.0, 0.0]
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
            "campaign.toml",
            r#"
electrical_format = 1
catalog = "catalog"
[assemblies.block]
world = "world.toml"
pcb = "pcb.toml"
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
        let mut scheme =
            format!("format = 1\nname = \"Блок\"\nscope = \"reference\"\nnets = [{nets}]\n");
        let mut bindings = String::new();
        for (net, addresses) in NETS {
            for address in *addresses {
                let (reference, pad) = address.split_once('/').unwrap();
                let component = format!("клемма-{reference}-{pad}");
                scheme.push_str(&format!(
                    "[components.{component:?}]\npart = \"terminal\"\nnets.pin = {net:?}\n"
                ));
                bindings.push_str(&format!(
                    "[[bindings]]\nreference = {reference:?}\npad = {pad:?}\ncomponent = {component:?}\npin = \"pin\"\n"
                ));
            }
        }
        block.write("scheme.toml", &scheme);
        let fingerprint = Fingerprint::of(scheme.as_bytes()).as_u64();
        block.rules =
            format!("format = 1\nschematic_fingerprint = \"{fingerprint:016x}\"\n{bindings}");
        block
    }

    fn write(&self, name: &str, content: &str) {
        fs::write(self.dir.join(name), content).unwrap();
    }

    fn launch(&self, board: &str, checks: &str) -> (Output, Value) {
        self.write("block.kicad_pcb", board);
        self.write("rules.toml", &format!("{}\n{checks}", self.rules));
        let output = Command::new(env!("CARGO_BIN_EXE_sim-run"))
            .arg("--campaign")
            .arg(self.dir.join("campaign.toml"))
            .arg("--json")
            .output()
            .unwrap();
        let json =
            serde_json::from_slice(&output.stdout).unwrap_or_else(|e| panic!("{e}: {output:?}"));
        (output, json)
    }

    fn report(&self, board: &str, checks: &str) -> Value {
        let (output, json) = self.launch(board, checks);
        assert!(output.status.success(), "{json}");
        json["assemblies"]["block"]["assemblies"]["блок"]["pcb"].clone()
    }

    fn error(&self, board: &str, expected: &str) {
        let (output, json) = self.launch(board, "");
        assert!(!output.status.success(), "{json}");
        assert_eq!(json["status"], "error");
        assert_eq!(json["proof_status"], "not_analyzed");
        assert!(
            json["detail"].as_str().unwrap().contains(expected),
            "{json}"
        );
    }
}

#[test]
fn copper_without_schematic_net_and_real_short_are_input_errors() {
    let block = Block::new("net");
    let report = block.report(FIXED, "");
    assert_eq!(report["connectivity"]["status"], "pass");
    // tfix1 №2: импортированная медь уже открывала оба обязательства без проверок платы.
    for code in ["creepage_clearance", "ground_bounce"] {
        assert!(report["open"][code].is_string(), "{report}");
    }
    assert_eq!(report["status"], "not_analyzed");
    block.error(UNSAFE, "короткое замыкание");
    // Отдельный от остальных трасс отрезок между координатами x=33…35,
    // y=26 внутри контура эталона. Цепь есть в плате, отсутствует в схеме.
    let marker = "(net 7 \"PROBE_B\")";
    assert!(FIXED.contains(marker));
    let changed = FIXED.replacen(marker, &format!(
        "{marker}\n(net 8 \"EXTRA\")\n(segment (start 33 26) (end 35 26) (width 0.4) (layer \"F.Cu\") (net 8))"
    ), 1);
    block.error(&changed, "EXTRA");
}

#[test]
fn floating_reference_plane_does_not_pass_despite_positive_projection() {
    let block = Block::new("plane");
    let checks = r#"
[checks.plane]
kind = "reference"
track = "bd430000-0000-4000-8000-000000000042"
plane = { zone = 0, layer = "B.Cu", tie_nm = [5000000,9000000], connection = { reference = "Координаты из depth_t4/expected.md" } }
"#;
    assert_eq!(
        block.report(FIXED, checks)["checks"]["plane"]["status"],
        "pass"
    );
    // Отрезок на B.Cu и переход — единственная связь заливки со звездой согласно expected.md.
    let floating = FIXED
        .lines()
        .filter(|line| {
            !line.contains("bd430000-0000-4000-8000-000000000057")
                && !line.contains("bd430000-0000-4000-8000-000000000062")
        })
        .collect::<Vec<_>>()
        .join("\n");
    assert_eq!(FIXED.lines().count() - floating.lines().count(), 2);
    let report = block.report(&floating, checks);
    assert_eq!(report["connectivity"]["status"], "pass");
    let check = &report["checks"]["plane"];
    assert_eq!(check["metrics"]["copper_gaps_m"], serde_json::json!([]));
    assert_eq!(check["status"], "not_analyzed", "{check}");
}

#[test]
fn all_same_face_pairs_do_not_cover_interlayer_isolation() {
    let block = Block::new("isolation");
    // Сухой геометрический стенд. Порог 1 мкм равен 1000 шагам KiCad
    // и заведомо меньше минимума чистой платы 0,8 мм из expected.md.
    let context = r#"{ source = { reference = "Синтетический опыт геометрии" }, material = "Диэлектрик эталона", environment = "Сухой чистый стенд", working_v = 0.0, peak_v = 0.0, frequency_hz = 0.0, altitude_m = 0.0, distance_error_m = 0.0 }"#;
    let mut checks = String::new();
    for (index, &(a, _)) in NETS.iter().enumerate() {
        for (offset, &(b, _)) in NETS[index + 1..].iter().enumerate() {
            checks.push_str(&format!(
                r#"
[checks.front{index}_{offset}]
kind = "clearance"
face = "front"
nets = [{a:?}, {b:?}]
conditions = {context}
limits = {{ envelope = {context}, clearance_m = 0.000001, creepage_m = 0.000001 }}
"#
            ));
        }
    }
    checks.push_str(&format!(
        r#"
[checks.back]
kind = "clearance"
face = "back"
nets = ["GND", "LINK"]
conditions = {context}
limits = {{ envelope = {context}, clearance_m = 0.000001, creepage_m = 0.000001 }}
"#
    ));
    let report = block.report(FIXED, &checks);
    assert_eq!(report["clearance_uncovered"], serde_json::json!([]));
    assert!(
        report["open"]["creepage_clearance"]
            .as_str()
            .is_some_and(|reason| reason.contains("между слоями")),
        "{report}"
    );
}
