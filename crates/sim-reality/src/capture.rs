use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use sim_signal::{ChannelId, Command, Sample, SignalId, SignalTable, Time, fnv1a};

#[derive(Clone, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct BuildId {
    pub design: String,
    pub revision: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Origin {
    Synthetic,
    Recorded,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Role {
    Command,
    Observation,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Sampling {
    /// События нумеруются источником до записи. Пропущенный номер — ошибка.
    Event,
    /// Пакетный вход до первого пакета пуст. Все поля пакета имеют одну метку.
    Packet { group: String },
    /// Первая метка и период задаются схемой, последняя метка выводится из горизонта.
    Periodic { first_ns: u64, period_ns: u64 },
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Channel {
    pub name: String,
    pub unit: String,
    pub role: Role,
    pub sampling: Sampling,
    #[serde(default)]
    pub allow_invalid: bool,
}

#[derive(Debug)]
pub struct Capture {
    build: BuildId,
    origin: Origin,
    end: Time,
    fingerprint: u64,
    observations: SignalTable,
    inputs: SignalTable,
    packets: Vec<ChannelId>,
    samples: Vec<Vec<Sample>>,
    commands: Vec<(Time, Command)>,
}

fn field(s: &str) -> bool {
    !s.is_empty() && s.trim() == s && !s.contains([',', '"']) && !s.chars().any(char::is_control)
}

fn natural(s: &str) -> Option<u64> {
    (!s.is_empty() && s.bytes().all(|b| b.is_ascii_digit()))
        .then(|| s.parse().ok())
        .flatten()
}

fn columns<const N: usize>(line: &str) -> Result<[&str; N], String> {
    let mut fields = line.split(',');
    let row = std::array::from_fn(|_| fields.next());
    if row.iter().any(Option::is_none) || fields.next().is_some() {
        return Err(format!("ожидалось полей: {N}"));
    }
    Ok(row.map(Option::unwrap))
}

impl Capture {
    pub fn parse(csv: &str, schema: &[Channel]) -> Result<Self, String> {
        let (rows, footer) = csv
            .strip_suffix('\n')
            .unwrap_or(csv)
            .rsplit_once('\n')
            .ok_or("нет завершающей записи лога")?;
        let [tag, total] = columns(footer.strip_suffix('\r').unwrap_or(footer))?;
        let total = natural(total)
            .filter(|_| tag == "end")
            .ok_or("требуется end,число_записей")?;
        let mut lines = rows.lines();
        let origin = match lines.next() {
            Some("sim-reality,1,synthetic") => Origin::Synthetic,
            Some("sim-reality,1,recorded") => Origin::Recorded,
            _ => return Err("строка 1: не распознаны формат, версия или источник лога".into()),
        };
        let [tag, design, revision, end] = columns(lines.next().ok_or("нет строки конструкции")?)?;
        let end = Time(natural(end).ok_or("горизонт должен быть целым числом наносекунд")?);
        if tag != "build" || !field(design) || !field(revision) || end == Time::ZERO {
            return Err("строка 2: требуется build,конструкция,ревизия,горизонт_нс (> 0)".into());
        }
        if lines.next() != Some("time_ns,signal,unit,seq,value,valid") {
            return Err("строка 3: требуется time_ns,signal,unit,seq,value,valid".into());
        }
        if schema.is_empty() {
            return Err("схема каналов пуста".into());
        }
        let mut observations = SignalTable::default();
        let mut inputs = SignalTable::default();
        let mut channels = BTreeMap::new();
        let mut packet_groups = BTreeMap::<&str, Vec<ChannelId>>::new();
        for (index, ch) in schema.iter().enumerate() {
            if !field(&ch.name) || !field(&ch.unit) {
                return Err(format!("некорректное имя или единица канала «{}»", ch.name));
            }
            if let Sampling::Periodic {
                first_ns,
                period_ns,
            } = ch.sampling
                && (period_ns == 0 || first_ns > end.0)
            {
                return Err(format!("некорректная сетка канала «{}»", ch.name));
            }
            if ch.role == Role::Observation && matches!(ch.sampling, Sampling::Packet { .. }) {
                return Err(format!(
                    "пакетная сетка предназначена для команд «{}»",
                    ch.name
                ));
            }
            if ch.role == Role::Command && ch.allow_invalid {
                return Err(format!("пропуски команд «{}» запрещены", ch.name));
            }
            let table = match ch.role {
                Role::Command => &mut inputs,
                Role::Observation => &mut observations,
            };
            let id = table.register(&ch.name, &ch.unit)?;
            if let Sampling::Packet { group } = &ch.sampling {
                if !field(group) {
                    return Err("пустое или некорректное имя группы пакета".into());
                }
                packet_groups
                    .entry(group)
                    .or_default()
                    .push(ChannelId(id.0));
            }
            if channels.insert(ch.name.as_str(), (index, id)).is_some() {
                return Err(format!("повтор канала «{}»", ch.name));
            }
        }
        let mut capture = Self {
            build: BuildId {
                design: design.into(),
                revision: revision.into(),
            },
            origin,
            end,
            fingerprint: fnv1a(csv.bytes()),
            samples: vec![Vec::new(); observations.len()],
            observations,
            inputs,
            packets: packet_groups.values().flatten().copied().collect(),
            commands: Vec::new(),
        };
        let mut progress = vec![(0_u64, None); schema.len()];
        let mut previous = Time::ZERO;
        let mut records = 0;
        for (index, line) in lines.enumerate() {
            let result = (|| {
                let [at, name, unit, seq, value, valid] = columns(line)?;
                let &(index, id) = channels
                    .get(name)
                    .ok_or_else(|| format!("неизвестный канал «{name}»"))?;
                let ch = &schema[index];
                if unit != ch.unit {
                    return Err(format!(
                        "«{name}»: единица «{unit}», ожидалась «{}»",
                        ch.unit
                    ));
                }
                let at = Time(natural(at).ok_or("время должно быть целым числом наносекунд")?);
                let (next, last) = &mut progress[index];
                if at < previous || at > end || last.is_some_and(|t| at <= t) {
                    return Err(format!(
                        "«{name}»: нарушение порядка времени, повтор или выход за горизонт"
                    ));
                }
                if natural(seq) != Some(*next) {
                    return Err(format!("«{name}»: ожидался номер {next}, получен «{seq}»"));
                }
                if let Sampling::Periodic {
                    first_ns,
                    period_ns,
                } = ch.sampling
                    && next
                        .checked_mul(period_ns)
                        .and_then(|n| first_ns.checked_add(n))
                        != Some(at.0)
                {
                    return Err(format!(
                        "«{name}»: пропуск такта или отклонение от объявленной сетки"
                    ));
                }
                let (value, valid) = match valid {
                    "1" => (
                        value
                            .parse::<f64>()
                            .ok()
                            .filter(|v| v.is_finite())
                            .ok_or("действительный отсчёт должен быть конечным числом")?,
                        true,
                    ),
                    // Ноль в Sample — заполнитель контракта, доступен только при valid=false.
                    "0" if ch.allow_invalid && value.is_empty() => (0.0, false),
                    _ => return Err(
                        "пропуск требуется разрешить в схеме и записать пустым значением с valid=0"
                            .into(),
                    ),
                };
                if valid && unit == "bool" && !matches!(value, 0.0 | 1.0) {
                    return Err(format!("«{name}»: значение bool должно быть 0 или 1"));
                }
                if ch.role == Role::Command {
                    if last.is_none()
                        && at != Time::ZERO
                        && !matches!(ch.sampling, Sampling::Packet { .. })
                    {
                        return Err(format!("«{name}»: нужна явная начальная команда в 0 нс"));
                    }
                    if capture.commands.last().is_none_or(|(time, _)| *time != at) {
                        capture.commands.push((at, Command::default()));
                    }
                    capture
                        .commands
                        .last_mut()
                        .unwrap()
                        .1
                        .targets
                        .push((ChannelId(id.0), value));
                } else {
                    capture.samples[id.0 as usize].push(Sample {
                        signal: id,
                        value,
                        at,
                        valid,
                    });
                }
                *next = next.checked_add(1).ok_or("переполнение номера отсчёта")?;
                *last = Some(at);
                previous = at;
                Ok(())
            })();
            result.map_err(|why| format!("строка {}: {why}", index + 4))?;
            records += 1;
        }
        if total != records {
            return Err(format!(
                "в завершающей записи заявлено {total} отсчётов, прочитано {records}"
            ));
        }
        for (ch, (count, last)) in schema.iter().zip(progress) {
            let Some(last) = last else {
                return Err(format!("нет записей канала «{}»", ch.name));
            };
            if let Sampling::Periodic { period_ns, .. } = ch.sampling
                && end.0 - last.0 >= period_ns
            {
                return Err(format!(
                    "«{}»: в конце лога отсутствуют периодические отсчёты после номера {}",
                    ch.name,
                    count - 1
                ));
            }
        }
        for (at, command) in &mut capture.commands {
            command.targets.sort_by_key(|&(id, _)| id);
            for (group, ids) in &packet_groups {
                let count = ids
                    .iter()
                    .filter(|&&id| {
                        command
                            .targets
                            .binary_search_by_key(&id, |&(id, _)| id)
                            .is_ok()
                    })
                    .count();
                if count != 0 && count != ids.len() {
                    return Err(format!("неполный пакет «{group}» в {} нс", at.0));
                }
            }
        }
        Ok(capture)
    }

    pub fn build(&self) -> &BuildId {
        &self.build
    }

    pub fn origin(&self) -> Origin {
        self.origin
    }

    pub fn horizon(&self) -> Time {
        self.end
    }

    pub fn fingerprint(&self) -> u64 {
        self.fingerprint
    }

    pub fn signals(&self) -> &SignalTable {
        &self.observations
    }

    /// Индекс в таблице входов совпадает с ChannelId команд этого лога.
    pub fn inputs(&self) -> &SignalTable {
        &self.inputs
    }

    pub fn samples(&self, signal: SignalId) -> Option<&[Sample]> {
        self.samples.get(signal.0 as usize).map(Vec::as_slice)
    }

    pub fn commands(&self) -> &[(Time, Command)] {
        &self.commands
    }

    pub(crate) fn has_missing_observations(&self) -> bool {
        self.samples.iter().flatten().any(|sample| !sample.valid)
    }

    pub(crate) fn packet_inputs(&self) -> &[ChannelId] {
        &self.packets
    }

    pub(crate) fn series(&self, name: &str, unit: &str) -> Result<&[Sample], String> {
        let id = self
            .signals()
            .id(name)
            .ok_or_else(|| format!("нет наблюдения «{name}»"))?;
        let spec = self.signals().spec(id).unwrap();
        if spec.unit != unit {
            return Err(format!(
                "«{name}»: ожидалась единица «{unit}», есть «{}»",
                spec.unit
            ));
        }
        Ok(self.samples(id).unwrap())
    }
}
