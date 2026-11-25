use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use serde_json::Value;

struct Temp(PathBuf);

impl Drop for Temp {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).expect("удаление временной кампании");
    }
}

fn run(manifest: &Path, args: &[&str], exit: i32) -> Vec<u8> {
    let output = Command::new(env!("CARGO_BIN_EXE_sim-run"))
        .arg("--campaign")
        .arg(manifest)
        .arg("--json")
        .args(args)
        .output()
        .expect("запуск sim-run");
    assert_eq!(
        output.status.code(),
        Some(exit),
        "{}",
        String::from_utf8_lossy(&output.stdout),
    );
    output.stdout
}

#[test]
fn отчёт_и_журналы_совпадают_при_разном_числе_исполнителей() {
    let fixtures = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../devices/stage6/corpus")
        .canonicalize()
        .unwrap();
    let dir = std::env::temp_dir().join(format!("campaign-parallel-{}", std::process::id()));
    fs::create_dir(&dir).unwrap();
    let dir = Temp(dir);
    let manifest = dir.0.join("manifest.toml");
    fs::write(
        &manifest,
        format!(
            "version = 1\n{}",
            ["block", "rod"]
                .map(|id| format!(
                    "[assemblies.{id}]\nscope = 'Стержень на направляющей'\n\
                [assemblies.{id}.experiments.travel]\nkind = 'scenario'\n\
                campaign = '{0}/over-travel.toml'\ninstallation = '{0}/over-travel-install.toml'\n",
                    fixtures.display()
                ))
                .join("")
        ),
    )
    .unwrap();

    // Две сборки по два состояния допуска из оснастки. Четыре места позволяют
    // одновременно занять отдельное место каждой строкой.
    let serial = run(&manifest, &["--jobs", "1"], 0);
    let parallel = run(&manifest, &["--jobs", "4"], 0);
    assert!(serial == parallel, "разные байты свежих JSON-отчётов");
    let report: Value = serde_json::from_slice(&serial).unwrap();
    let checkpoint = dir.0.join("checkpoint");
    let checkpoint = checkpoint.to_str().unwrap();
    let saved = run(&manifest, &["--jobs", "1", "--checkpoint", checkpoint], 0);
    assert!(serial == saved, "запись точек изменила свежий отчёт");
    let mut resumed: Value = serde_json::from_slice(&run(
        &manifest,
        &["--jobs", "4", "--checkpoint", checkpoint],
        0,
    ))
    .unwrap();
    assert!(resumed["resumed"].as_u64().unwrap() > 0);
    resumed["resumed"] = serde_json::json!(0);
    assert_eq!(resumed, report);

    let assemblies = report["assemblies"].as_object().unwrap();
    assert_eq!(assemblies.len(), 2);
    for assembly in assemblies.values() {
        assert_eq!(assembly["verdict"], "Rejected");
        assert_eq!(assembly["coverage"]["executed"], 2);
        let rows = assembly["runs"].as_object().unwrap();
        assert_eq!(rows.len(), 2);
        for (key, row) in rows {
            let log = dir.0.join("row.simlog");
            let log_arg = log.to_str().unwrap();
            let repeat = run(
                &manifest,
                &["--jobs", "1", "--repeat", key, "--log", log_arg],
                0,
            );
            let bytes = fs::read(&log).unwrap();
            fs::remove_file(&log).unwrap();
            assert!(
                repeat
                    == run(
                        &manifest,
                        &["--jobs", "4", "--repeat", key, "--log", log_arg],
                        0
                    )
            );
            assert!(
                bytes == fs::read(&log).unwrap(),
                "{key}: разные байты SIMLOG"
            );
            fs::remove_file(&log).unwrap();
            let decoded = sim_runtime::Log::read_binary(bytes.as_slice()).unwrap();
            assert_eq!(
                decoded.fingerprint(),
                row["detail"]["journal"].as_u64().unwrap()
            );
            assert_eq!(decoded.end().time.0, 4_000_000); // Горизонт over-travel.toml, 4 мс.
            assert!(!decoded.entries().is_empty());
        }
    }
    let error: Value = serde_json::from_slice(&run(&manifest, &["--jobs", "0"], 2)).unwrap();
    assert_eq!(error["status"], "error");
    assert!(error["detail"].as_str().unwrap().contains("--jobs"));
}
