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
