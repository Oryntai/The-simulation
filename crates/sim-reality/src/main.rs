use std::{env, fs, io, path::Path, process::ExitCode};

use sim_reality::{Profile, Receipt, analyze};

fn read(path: &Path) -> Result<String, String> {
    fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))
}

fn run() -> Result<(), String> {
    const USAGE: &str =
        "Использование: sim-reality run <профиль.toml> <лог.csv> [запись-замера.toml]";
    let args: Vec<_> = env::args_os().skip(1).collect();
    if args.len() == 1 && matches!(args[0].to_str(), Some("--help" | "-h")) {
        println!("{USAGE}");
        return Ok(());
    }
    let [verb, profile, csv, extra @ ..] = args.as_slice() else {
        return Err(USAGE.into());
    };
    if verb != "run" || extra.len() > 1 {
        return Err(USAGE.into());
    }
    let profile = Path::new(profile);
    let data = read(profile)?;
    let settings: Profile =
        toml::from_str(&data).map_err(|e| format!("{}: {e}", profile.display()))?;
    let csv = read(Path::new(csv))?;
    let receipt = extra
        .first()
        .map(|p| {
            let path = Path::new(p);
            toml::from_str::<Receipt>(&read(path)?).map_err(|e| format!("{}: {e}", path.display()))
        })
        .transpose()?;
    let root = profile.parent().unwrap_or_else(|| Path::new("."));
    let report = analyze(&settings, root, &csv, receipt.as_ref())?;
    serde_json::to_writer_pretty(io::stdout().lock(), &report).map_err(|e| e.to_string())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}
