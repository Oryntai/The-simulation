use std::path::Path;

use serde_json::Value;
use sim_cli::electrical::{load_catalog, load_schematic};
use sim_process::schematic::Status;
use sim_task::{
    Outcome,
    verdict::{HazardClass, SimulationVerdict},
};

mod stand {
    include!("fixtures/electrical_verdict.rs");

    impl ElectricalStand {
        pub(super) fn with_esd_status(
            &self,
            input: &SchematicInput,
            status: sim_process::schematic::Status,
        ) -> VerdictReport {
            fs::write(self.directory.0.join("scheme.toml"), &input.bytes).unwrap();
            let mut evidence = ElectricalEvidence::evaluate(
                "блок",
                &input.bytes,
                &input.compiled,
                &input.modes,
                self.inventory.build.unwrap(),
            )
            .unwrap();
            let recorded = evidence
                .record_esd_event(
                    &input.compiled,
                    &input.modes,
                    "импульс",
                    Fingerprint::of(b"Ta6"),
                    |compiled| {
                        Ok(sim_process::schematic::Finding {
                            subject: compiled.pins().next().unwrap().to_string(),
                            status,
                            reason: "Проверка границы публикации ЭМС".into(),
                            evidence: (),
                        })
                    },
                )
                .unwrap();
            assert_eq!(
                recorded.status,
                match status {
                    sim_process::schematic::Status::Pass =>
                        sim_process::schematic::Status::NotAnalyzed,
                    other => other,
                }
            );
            let requirements = derive_requirements_with_electrical(
                &self.inventory,
                &self.plan,
                &BTreeMap::from([("блок".into(), evidence)]),
            )
            .unwrap();
            assess_campaign(&requirements, &CampaignEvidence::new(&requirements)).unwrap()
        }

        pub(super) fn command(
            &self,
            input: &SchematicInput,
            catalog: &std::path::Path,
            profile: &str,
        ) -> std::process::Command {
            let dir = &self.directory.0;
            fs::write(dir.join("scheme.toml"), &input.bytes).unwrap();
            let parts = dir.join("catalog/electrical");
            fs::create_dir_all(&parts).unwrap();
            fs::copy(catalog.join("reference.toml"), parts.join("reference.toml")).unwrap();
            fs::write(dir.join("emc.toml"), profile).unwrap();
            let manifest = dir.join("electrical.toml");
            fs::write(&manifest, "electrical_format = 1\ncatalog = \"catalog\"\n[assemblies.block]\nworld = \"world.toml\"\nemc = \"emc.toml\"\n").unwrap();
            let mut command = std::process::Command::new(env!("CARGO_BIN_EXE_sim-run"));
            command.arg("--campaign").arg(manifest).arg("--json");
            command
        }
    }
}

fn data() -> (std::path::PathBuf, sim_cli::electrical::SchematicInput) {
    let fixtures = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../devices/electrical/depth_t6");
    let acceptance: toml::Value = toml::from_str(include_str!(
        "../../../devices/electrical/depth_t6/acceptance.toml"
    ))
    .unwrap();
    let reset = &acceptance["reset"];
    let catalog_path = fixtures.join(reset["catalog"].as_str().unwrap());
    let catalog = load_catalog(&catalog_path).unwrap();
    let scheme = load_schematic(
        &fixtures.join(reset["scheme"].as_str().unwrap()),
        &catalog.catalog,
    )
    .unwrap();
    (catalog_path, scheme)
}

#[test]
fn вердикт_не_принимает_pass_от_оценщика_esd() {
    let (_, scheme) = data();
    let stand = stand::ElectricalStand::new("ta6-outcomes");
    let previous = stand.assess(&scheme);
    assert!(
        previous
            .findings
            .iter()
            .all(|item| item.key.class != HazardClass::EsdEvent)
    );
    for (candidate, expected) in [
        (Status::Pass, Outcome::NotAnalyzed),
        (Status::Fail, Outcome::Fail),
    ] {
        let report = stand.with_esd_status(&scheme, candidate);
        assert_eq!(
            report.simulation,
            if expected == Outcome::Fail {
                SimulationVerdict::Rejected
            } else {
                SimulationVerdict::NotProven
            }
        );
        let esd = report
            .findings
            .iter()
            .filter(|item| item.key.class == HazardClass::EsdEvent)
            .collect::<Vec<_>>();
        assert_eq!(esd.len(), 1);
        assert_eq!(esd[0].outcome, expected);
        assert_eq!(esd[0].confirmed_hazard, expected == Outcome::Fail);
        if expected == Outcome::NotAnalyzed {
            assert!(!esd[0].missing_data.is_empty());
        }
    }
}

#[test]
fn cli_без_журналов_открыт_и_отклоняет_неконечные_числа_без_источника() {
    let (catalog, scheme) = data();
    let stand = stand::ElectricalStand::new("ta6-report");
    // Адреса, интервал и напряжение покоя из стенда Т6.7.
    let profile = r#"format = 1
[instances."блок".emi_reset]
source = "sender"
source_kind = "switch"
board = "fixture"
input = "board/io"
reference = "board/g"
start_ns = 11000
end_ns = 22000
bias_v = { min = 3.0, nominal = 3.0, max = 3.0 }
"#;
    let run = |profile: &str| stand.command(&scheme, &catalog, profile).output().unwrap();
    let output = run(profile);
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let report: Value = serde_json::from_slice(&output.stdout).unwrap();
    let block = &report["assemblies"]["block"];
    assert_eq!(block["simulation"], "NotProven");
    let checks = &block["assemblies"]["блок"]["emc_checks"];
    for class in ["esd_event", "emi_reset"] {
        assert_eq!(checks[class]["status"], "not_analyzed");
        assert!(
            !checks[class]["required_measurement"]
                .as_str()
                .unwrap()
                .is_empty()
        );
    }
    for bad in ["nan", "inf", "-inf"] {
        let invalid = profile.replace("nominal = 3.0", &format!("nominal = {bad}"));
        let output = run(&invalid);
        assert_eq!(output.status.code(), Some(2), "принято {bad}");
        let report: Value = serde_json::from_slice(&output.stdout).unwrap();
        assert_eq!(report["status"], "error");
        let detail = report["detail"].as_str().unwrap();
        assert!(
            detail.contains("bias_v.nominal") && detail.contains("конечные числа"),
            "{report:?}"
        );
    }
}
