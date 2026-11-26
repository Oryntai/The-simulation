use std::{fs, path::PathBuf, process::Command};

use serde_json::Value;
use sim_cli::view::{RunView, campaign_bindings};

struct Stand(PathBuf);

impl Stand {
    fn new(id: &str, runs: u32) -> Self {
        let stand =
            Self(std::env::temp_dir().join(format!("depth-repeat-{}-{id}", std::process::id())));
        fs::create_dir(&stand.0).unwrap();
        stand.write(
            "world.toml",
            r#"
format = 3
name = "Блок"
seed = 17
physics_hz = 1000
gravity_m_s2 = [1.0, 0.0, 0.0]
[environment]
density_kg_m3 = 0.0
viscosity_pa_s = 0.0
wind_m_s = [0.0, 0.0, 0.0]
[[devices]]
name = "блок"
file = "block.toml"
"#,
        );
        stand.write("block.toml", r#"
format = 3
name = "Блок"
[[links]]
name = "каретка"
joint = { kind = "slide", axis = [1.0, 0.0, 0.0] }
# Единичный куб, I = ma²/6.
mass = { kind = "explicit", mass_kg = 6.0, com_m = [0.0, 0.0, 0.0], inertia_kg_m2 = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0], provenance = { measured = "куб стенда" } }
"#);
        stand.write("catalog.toml", "version = 1");
        stand.write(
            "manifest.toml",
            r#"
version = 1
[assemblies.block]
scope = "Стенд каретки"
[assemblies.block.experiments.cover]
kind = "scenario"
campaign = "campaign.toml"
installation = "install.toml"
"#,
        );
        stand.write(
            "install.toml",
            &format!(
                r#"
version = 1
[coverage]
runs = {runs}
# Квота времени тестовой оснастки.
seconds = 60
[[paths]]
station = "block"
mode = "motion"
path = "line"
source = {{ kind = "measured", reference = "оснастка каретки" }}
grids = [
    {{ id = "base", hz = 1000, source = {{ kind = "measured", reference = "шаг 1 мс" }} }},
    {{ id = "fine", hz = 2000, source = {{ kind = "measured", reference = "шаг 0,5 мс" }} }},
]
"#
            ),
        );
        stand.write("campaign.toml", r#"
version = 1
world = "world.toml"
catalog = "catalog.toml"
seed = 17
[stations.block]
experiment = { kind = "scenario" }
[stations.block.modes.motion]
horizon_ns = 2000000
source = { kind = "measured", reference = "двухмиллисекундный стенд" }
# x = a·h²·n(n+1)/2 = 3 и 2,5 мкм. Запасы до 2,75 мкм: −0,25 и +0,25 мкм.
criteria = [{ id = "ход", window = "whole", limit = { kind = "range", signal = { name = "qpos[0]", unit = "m" }, min = 0.0, max = 0.00000275, source = { kind = "measured", reference = "порог синтетической оснастки" } } }]
[stations.block.modes.motion.windows.whole]
start_ns = 0
end_ns = 2000000
max_gap_ns = 1000000
[depth_factors.factors.operation]
safety = true
source = { kind = "measured", reference = "режим стенда" }
levels = { kind = "mode", values = { move = { station = "block", mode = "motion" } } }
[depth_factors.factors.route]
safety = true
source = { kind = "measured", reference = "путь стенда" }
levels = { kind = "path", values = { line = "line" } }
[depth_factors.factors.clock]
safety = true
source = { kind = "measured", reference = "сетки стенда" }
levels = { kind = "grid", values = { base = "base", fine = "fine" } }
[depth_factors.factors.mass]
safety = false
source = { kind = "measured", reference = "масса единичного куба стенда" }
levels = { kind = "axis", parameter = { kind = "mass", body = "блок/каретка" }, unit = "kg", nominal = 6.0, values = { base = 6.0 } }
"#);
        stand
    }

    fn write(&self, name: &str, data: &str) {
        fs::write(self.0.join(name), data).unwrap();
    }

    fn run(&self, args: &[&str], code: i32) -> Value {
        let output = Command::new(env!("CARGO_BIN_EXE_sim-run"))
            .arg("--campaign")
            .arg(self.0.join("manifest.toml"))
            .arg("--json")
            .args(args)
            .output()
            .unwrap();
        assert_eq!(
            output.status.code(),
            Some(code),
            "{}",
            String::from_utf8_lossy(&output.stdout)
        );
        serde_json::from_slice(&output.stdout).unwrap()
    }
}

impl Drop for Stand {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

#[test]
fn пары_тройки_и_повтор_худшей_строки_по_исходной_квитанции() {
    let stand = Stand::new("complete", 4);
    let checkpoint = stand.0.join("state");
    let state = checkpoint.to_str().unwrap();
    let report = stand.run(&["--checkpoint", state, "--jobs", "1"], 0);
    assert_eq!(report, stand.run(&["--jobs", "2"], 0));
    let assembly = &report["assemblies"]["block"];
    let depth = &assembly["depth_coverage"];
    // Три пары между постоянными факторами и по две для трёх пар с сеткой.
    // Единственная тройка безопасности содержит одну из двух сеток.
    assert_eq!(
        depth["pairs"],
        serde_json::json!({"required": 9, "planned": 9, "executed": 9})
    );
    assert_eq!(
        depth["safety_triples"],
        serde_json::json!({"required": 2, "planned": 2, "executed": 2})
    );
    assert_eq!(depth["open_rows"], serde_json::json!([]));
    assert_eq!(assembly["verdict"], "Rejected");
    let key = report["worst_run_key"].as_str().unwrap();
    let worst_row = &assembly["runs"][key];
    assert_eq!(worst_row["detail"]["coverage"]["key"]["grid"], "base");
    assert_eq!(depth["worst"]["run_key"], key);
    let bindings = campaign_bindings(&report.to_string()).unwrap();

    for (row_key, expected_reserve) in [
        (key, -0.25e-6),
        (
            assembly["runs"]
                .as_object()
                .unwrap()
                .keys()
                .find(|k| k.as_str() != key)
                .unwrap(),
            0.25e-6,
        ),
    ] {
        let archive = stand.0.join("row.simview");
        let repeat = stand.run(
            &[
                "--checkpoint",
                state,
                "--repeat",
                row_key,
                "--view",
                archive.to_str().unwrap(),
            ],
            0,
        );
        assert_eq!(repeat["depth_repeat"]["status"], "verified");
        let view = RunView::read(&fs::read(&archive).unwrap()).unwrap();
        assert_eq!(view.metadata.campaign.as_ref().unwrap(), &bindings[row_key]);
        assert_eq!(
            view.journal.fingerprint(),
            bindings[row_key].playback.journal
        );
        let observed = view
            .journal
            .entries()
            .iter()
            .rev()
            .find(|entry| {
                entry.kind == sim_runtime::LogKind::Truth
                    && view.journal.header().truth[entry.id as usize].name == "qpos[0]"
            })
            .unwrap();
        assert!(((2.75e-6 - observed.value) - expected_reserve).abs() < 1e-12);
        fs::remove_file(archive).unwrap();
    }
    let archive = stand.0.join("exhausted.simview");
    let repeat = stand.run(
        &[
            "--checkpoint",
            state,
            "--repeat",
            key,
            "--view",
            archive.to_str().unwrap(),
        ],
        0,
    );
    assert_eq!(repeat["depth_repeat"]["status"], "not_analyzed");
    assert!(!archive.exists());
}

#[test]
fn неполное_покрытие_и_физическое_исключение_различимы() {
    let stand = Stand::new("partial", 1);
    let a = stand.run(&["--jobs", "1"], 0);
    let depth = &a["assemblies"]["block"]["depth_coverage"];
    assert_eq!(
        depth["pairs"],
        serde_json::json!({"required": 9, "planned": 9, "executed": 6})
    );
    assert_eq!(
        depth["safety_triples"],
        serde_json::json!({"required": 2, "planned": 2, "executed": 1})
    );
    assert_eq!(depth["open_rows"].as_array().unwrap().len(), 1);
    assert!(
        depth["open_rows"][0]["reason"]
            .as_str()
            .unwrap()
            .contains("квот")
    );

    let input = fs::read_to_string(stand.0.join("campaign.toml")).unwrap();
    stand.write(
        "campaign.toml",
        &format!(
            r#"{input}
[depth_factors.constraints.grid_limit]
when = {{ clock = "base" }}
reason = "оснастка поддерживает только шаг 0,5 мс"
source = {{ kind = "measured", reference = "профиль оснастки" }}
"#
        ),
    );
    let b = stand.run(&[], 0);
    let assembly = &b["assemblies"]["block"];
    let depth = &assembly["depth_coverage"];
    assert_eq!(
        depth["pairs"],
        serde_json::json!({"required": 6, "planned": 6, "executed": 6})
    );
    assert_eq!(
        depth["safety_triples"],
        serde_json::json!({"required": 1, "planned": 1, "executed": 1})
    );
    assert_eq!(assembly["verdict"], "NotProven");
}
