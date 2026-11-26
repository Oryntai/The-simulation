use std::borrow::Cow;
use std::collections::BTreeSet;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

use sim_runtime::Runtime;
use sim_signal::{Command, Time};

fn field(text: &str) -> Result<Cow<'_, str>, String> {
    let text = text.trim();
    let Some(quoted) = text.strip_prefix('"') else {
        return if text.contains([',', '"']) {
            Err("поле с запятой или кавычкой требует кавычек CSV".into())
        } else {
            Ok(Cow::Borrowed(text))
        };
    };
    let quoted = quoted.strip_suffix('"').ok_or("незакрытая кавычка CSV")?;
    let mut decoded = String::new();
    let mut chars = quoted.chars();
    while let Some(ch) = chars.next() {
        if ch == '"' && chars.next() != Some('"') {
            return Err("кавычка внутри поля CSV должна быть удвоена".into());
        }
        decoded.push(ch);
    }
    Ok(Cow::Owned(decoded))
}

fn row(line: &str) -> Result<[Cow<'_, str>; 3], String> {
    let (time, tail) = line.split_once(',').ok_or("ожидались три поля")?;
    // Имена каналов могут содержать запятые. В числовых полях запятых нет.
    let (channel, value) = tail.rsplit_once(',').ok_or("ожидались три поля")?;
    Ok([field(time)?, field(channel)?, field(value)?])
}

pub(crate) fn read(
    path: &Path,
    runtime: &Runtime,
    until: Time,
) -> Result<Vec<(Time, Command)>, String> {
    let file = File::open(path).map_err(|err| format!("{}: {err}", path.display()))?;
    let mut commands: Vec<(Time, Command)> = Vec::new();
    let mut channels = BTreeSet::new();
    let mut header = false;
    for (index, line) in BufReader::new(file).lines().enumerate() {
        let report = |err: &str| format!("{}:{}: {err}", path.display(), index + 1);
        let line = line.map_err(|err| report(&err.to_string()))?;
        if line.trim().is_empty() {
            continue;
        }
        let [time, name, value] = row(&line).map_err(|err| report(&err))?;
        if !header {
            if [time.as_ref(), name.as_ref(), value.as_ref()] != ["time_s", "channel", "value"] {
                return Err(report("требуется заголовок time_s,channel,value"));
            }
            header = true;
            continue;
        }
        let at = time
            .parse()
            .ok()
            .and_then(Time::from_secs)
            .ok_or_else(|| report("time_s должно быть конечным неотрицательным временем"))?;
        if at > until {
            return Err(report("time_s выходит за --duration"));
        }
        let id = runtime
            .channel(&name)
            .ok_or_else(|| report(&format!("неизвестный канал «{name}»")))?;
        let value = value
            .parse::<f64>()
            .ok()
            .filter(|value| value.is_finite())
            .ok_or_else(|| report("value должно быть конечным числом"))?;
        if let Some((previous, _)) = commands.last()
            && *previous > at
        {
            return Err(report("время команд должно идти по возрастанию"));
        }
        if commands.last().is_none_or(|(previous, _)| *previous != at) {
            commands.push((at, Command::default()));
            channels.clear();
        }
        if !channels.insert(id) {
            return Err(report(&format!(
                "повтор канала «{name}» на той же наносекунде"
            )));
        }
        commands.last_mut().unwrap().1.targets.push((id, value));
    }
    if !header {
        return Err(format!("{}: отсутствует заголовок CSV", path.display()));
    }
    Ok(commands)
}

/// Вход одного дочернего опыта. Обычный манифест направляет его в тот же
/// установщик, а родитель держит резерв запуска и читает ответ из файла.
#[derive(serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct CoverageRow {
    pub installation: std::path::PathBuf,
    pub selection: sim_task::campaign::depth_factors::Selection,
    pub input: String,
    pub manifest: std::path::PathBuf,
    pub recording: Option<Recording>,
}

#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub(crate) enum Recording {
    Log,
    View,
}

pub(crate) fn coverage_binary() -> Result<std::path::PathBuf, String> {
    let exe = std::env::current_exe().map_err(|error| error.to_string())?;
    if exe.file_stem().is_some_and(|name| name == "sim-run") {
        return Ok(exe);
    }
    let sibling = exe.with_file_name(format!("sim-run{}", std::env::consts::EXE_SUFFIX));
    if !sibling.is_file() {
        return Err("рядом с клиентом отсутствует исполняемый файл sim-run для покрытия".into());
    }
    Ok(sibling)
}

pub(crate) fn coverage_command(
    exe: &Path,
    directory: &Path,
    prefix: &str,
    campaign: &Path,
    row: &CoverageRow,
) -> Result<std::process::Command, String> {
    use serde_json::json;
    use std::{fs, process::Stdio};

    let (assembly, experiment) = prefix.split_once('/').ok_or("неверный адрес опыта")?;
    let installation = directory.join("installation.toml");
    let manifest = directory.join("manifest.toml");
    let content = toml::to_string(&std::collections::BTreeMap::from([("coverage_row", row)]))
        .map_err(|error| error.to_string())?;
    let description = toml::to_string(&json!({
        "version": 1,
        "assemblies": { (assembly): {
            "scope": "Один опыт покрытия",
            "experiments": { (experiment): { "kind": "scenario", "campaign": campaign,
                "installation": installation } }
        } }
    }))
    .map_err(|error| error.to_string())?;
    fs::write(&installation, content).map_err(|error| error.to_string())?;
    fs::write(&manifest, description).map_err(|error| error.to_string())?;
    let output =
        File::create_new(directory.join("report.json")).map_err(|error| error.to_string())?;
    let mut command = std::process::Command::new(exe);
    command
        .args(["--campaign"])
        .arg(manifest)
        .args(["--json", "--jobs", "1"])
        .stdin(Stdio::null())
        .stdout(output);
    Ok(command)
}
