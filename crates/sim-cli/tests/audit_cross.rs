use std::{
    fs,
    path::{Path, PathBuf},
    process::{Command, Output},
};

use serde_json::Value;
use toml::Value as Toml;

include!("fixtures/audit_electrical.rs");

#[test]
fn declared_values_do_not_prove_capacitor_in_assembly() {
    let code = "capacitor_stress";
    let stand = Stand::t3("cross-no-live", code, "fixed");
    let reference = stand.electrical("");
    assert_eq!(
        reference["assemblies"]["block"]["depth_checks"][code]["status"],
        "pass"
    );
    for path in [
        "scheme.toml",
        "catalog/electrical/parts.toml",
        "catalog/electrical/ratings.toml",
    ] {
        stand.write(
            path,
            &fs::read_to_string(stand.0.join(path))
                .unwrap()
                .replace("reference_only = true\n", "")
                .replace("scope = \"reference\"", "scope = \"assembly\"")
                .replace("derived =", "measured ="),
        );
    }
    let report = stand.electrical("");
    assert!(report["live"].is_null());
    let check = &report["assemblies"]["block"]["depth_checks"][code];
    assert_eq!(check["status"], "not_analyzed", "{check}");
    let observed = &check["metrics"]["voltage_v"]["value"];
    // Независимый баланс идеального источника: 8 В < 12 В × 0,75 = 9 В.
    let contains = |field: &str, value: f64| {
        let bounds = &observed[field];
        assert!(bounds[0].as_f64().unwrap() <= value && value <= bounds[1].as_f64().unwrap());
    };
    contains("observed", 8.0);
    contains("margin", 9.0 - 8.0);
    assert!(check["reason"].as_str().unwrap().contains("журнал"));
    assert_eq!(report["simulation"], "NotProven");
}

#[test]
fn missing_check_is_visible_without_board_bindings() {
    let code = "capacitor_stress";
    let stand = Stand::t3("cross-missing-depth", code, "fixed");
    let mut scheme = stand.parse("scheme.toml");
    scheme["scope"] = Toml::String("assembly".into());
    scheme
        .as_table_mut()
        .unwrap()
        .remove("depth_checks")
        .unwrap();
    stand.save("scheme.toml", &scheme);
    let path = "catalog/electrical/parts.toml";
    stand.write(
        path,
        &fs::read_to_string(stand.0.join(path))
            .unwrap()
            .replace("reference_only = true\n", "")
            .replace("derived =", "measured ="),
    );
    let report = stand.electrical("");
    assert_eq!(report["simulation"], "NotProven", "{report}");
    assert!(
        report["findings"]
            .as_array()
            .unwrap()
            .iter()
            .any(|finding| {
                finding["code"] == code
                    && finding["outcome"] == "not_analyzed"
                    && finding["reason"].as_str().unwrap().contains("не заявлено")
            }),
        "{report}"
    );
}

#[test]
fn solder_json_distinguishes_window_and_assembly_outcome() {
    let stand = Stand::t3("cross-solder-scope", "capacitor_stress", "fixed");
    stand.write(
        "scheme.toml",
        &corpus("depth_t5/scheme.toml").replace("scope = \"reference\"", "scope = \"assembly\""),
    );
    stand.write(
        "catalog/electrical/parts.toml",
        &corpus("depth_t5/parts.toml").replace("reference_only = true\n", ""),
    );
    fs::remove_file(stand.0.join("catalog/electrical/ratings.toml")).unwrap();
    stand.write("solder.toml", &corpus("depth_t5/solder.toml"));
    stand.write("profile.toml", &corpus("depth_t5/bridge/fixed.toml"));
    // 1024 интервала на сетке исходного стенда Т5.
    stand.write("setup.toml", "format = 1\ncatalog = 'solder.toml'\nsteps = 1024\n[instances.block.'стык-т5-эталон']\nprofile = 'profile.toml'\n");
    let report = stand.electrical("solder = 'setup.toml'\n");
    let joint = &report["assemblies"]["block"]["solder_checks"]["стык-т5-эталон"];
    assert_eq!(joint["window_status"], "not_analyzed", "{joint}");
    assert_eq!(joint["scope"], "assembly");
    let finding = report["findings"]
        .as_array()
        .unwrap()
        .iter()
        .find(|f| f["code"] == "solder_defect")
        .unwrap();
    assert_eq!(joint["status"], finding["outcome"]);
    assert_eq!(joint["reason"], finding["reason"]);
    assert!(
        joint["window_reason"]
            .as_str()
            .unwrap()
            .contains("нет фазы остывания")
    );
}

#[test]
fn missing_battery_protection_wire_and_emc_follow_typed_parts() {
    for (label, case, part, changes, required, absent) in [
        (
            "battery",
            "deep_discharge",
            "cutoff_stand",
            parse("kind = 'battery'"),
            &["deep_discharge", "sleep_drain"][..],
            &["emi_reset", "fuse_coordination"][..],
        ),
        (
            "fuse",
            "fuse_coordination",
            "series_fault",
            parse("kind = 'other'"),
            &["fuse_coordination"][..],
            &["deep_discharge", "cable_flex"][..],
        ),
        (
            "wire",
            "capacitor_stress",
            "cap_8",
            parse("kind = 'wire'"),
            &["cable_flex"][..],
            &["deep_discharge", "emi_reset"][..],
        ),
        (
            "emc",
            "switch_thermal",
            "thermal_step",
            parse(
                r#"kind = "other"
[pins]
g = { contact = "terminal", role = "control" }
d = { contact = "bare", role = { signal_in = "s" } }
s = { contact = "bare", role = "return" }"#,
            ),
            &["switch_thermal", "esd_event", "emi_reset"][..],
            &["deep_discharge", "cable_flex"][..],
        ),
    ] {
        let stand = Stand::t3(label, case, "fixed");
        let mut scheme = stand.parse("scheme.toml");
        scheme["scope"] = Toml::String("assembly".into());
        scheme
            .as_table_mut()
            .unwrap()
            .remove("depth_checks")
            .unwrap();
        stand.save("scheme.toml", &scheme);
        let catalog = "catalog/electrical/parts.toml";
        let mut parts = stand.parse(catalog);
        let entry = parts["parts"][part].as_table_mut().unwrap();
        entry.remove("reference_only");
        entry.insert("kind".into(), changes["kind"].clone());
        if let Some(pins) = changes.get("pins") {
            entry["pins"]
                .as_table_mut()
                .unwrap()
                .extend(pins.as_table().unwrap().clone());
        }
        stand.save(catalog, &parts);
        let report = stand.electrical("");
        let is_open = |code: &str| {
            report["findings"]
                .as_array()
                .unwrap()
                .iter()
                .any(|finding| {
                    finding["code"] == code
                        && finding["outcome"] == "not_analyzed"
                        && finding["reason"].as_str().unwrap().contains("не заявлено")
                })
        };
        for code in required {
            assert!(is_open(code), "{label}: отсутствует {code}");
        }
        for code in absent {
            assert!(!is_open(code), "{label}: неприменимый {code}");
        }
    }
}
