//! Версионированный журнал. Все целые и биты f64 записываются в little endian.

use std::cell::OnceCell;
use std::fmt::Write as _;
use std::io::{self, BufWriter, Read, Write};
use std::rc::Rc;

use sim_signal::{SignalSpec, Time, fnv1a};

use crate::{Status, process::ProcessTick};

const MAGIC: &[u8] = b"SIMLOG\r\n";
const VERSION: u16 = 1;
const VERSION_WITH_PROCESSES: u16 = 2;
// Запись: 3 однобайтовых признака, 2 u32, время и значение по 8 байт.
const ENTRY_BYTES: usize = 27;
// Итог: четыре u64, статус и признак начала расчёта.
const END_BYTES: usize = 4 * size_of::<u64>() + 2 * size_of::<u8>();

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandSource {
    BeforeStart,
    External,
    Controller(u32),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LogKind {
    Observation,
    Command(CommandSource),
    Truth,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LogEntry {
    pub at: Time,
    pub kind: LogKind,
    /// Индекс в таблице своего вида записи.
    pub id: u32,
    pub value: f64,
    pub valid: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LogEnd {
    pub time: Time,
    pub steps: u64,
    pub state_hash: u64,
    pub status: Status,
    pub started: bool,
    /// Отказ обработчика, чьи команды при воспроизведении берутся из журнала.
    pub failed_controller: Option<u32>,
}

/// Таблицы имён и единиц в порядке регистрации.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LogHeader {
    pub world_hash: u64,
    pub seed: u64,
    pub observations: Vec<SignalSpec>,
    pub commands: Vec<SignalSpec>,
    pub truth: Vec<SignalSpec>,
}

/// Прореживание рядов истины: из каждого окна `period_ns` по каждому каналу остаются
/// первое значение, минимум и максимум. Прочие записи журнала не трогаются, порядок по
/// времени сохраняется: окно копится и сбрасывается целиком.
#[derive(Clone, Debug)]
struct Thinning {
    period_ns: u64,
    window: Option<u64>,
    buffer: Vec<LogEntry>,
}

#[derive(Clone, Debug)]
pub struct Log {
    pub(crate) header: LogHeader,
    entries: Rc<Vec<LogEntry>>,
    pub(crate) end: LogEnd,
    // Аудит и ошибки. У прочитанного с диска журнала здесь все записи.
    processes: Rc<Vec<(Time, String)>>,
    process_ticks: Option<Rc<Vec<ProcessTick>>>,
    rendered_processes: OnceCell<Rc<Vec<(Time, String)>>>,
    thinning: Option<Thinning>,
}

enum ProcessRow<'a> {
    Text(&'a (Time, String)),
    Tick(&'a ProcessTick),
}

impl ProcessRow<'_> {
    fn at(&self) -> Time {
        match self {
            Self::Text((at, _)) => *at,
            Self::Tick(tick) => tick.at,
        }
    }

    fn render(&self, text: &mut String) {
        text.clear();
        match self {
            Self::Text((_, line)) => text.push_str(line),
            Self::Tick(tick) => write!(text, "{tick:?}").expect("запись в String"),
        }
    }
}

impl Log {
    pub(crate) fn new(header: LogHeader, end: LogEnd) -> Self {
        Self {
            header,
            entries: Rc::default(),
            end,
            processes: Rc::default(),
            process_ticks: None,
            rendered_processes: OnceCell::new(),
            thinning: None,
        }
    }

    /// Включает прореживание рядов истины окнами `period_ns`.
    pub(crate) fn set_truth_period(&mut self, period_ns: u64) {
        self.thinning = Some(Thinning { period_ns, window: None, buffer: Vec::new() });
    }

    /// Период прореживания рядов истины, если включено.
    pub(crate) fn truth_period(&self) -> Option<u64> {
        self.thinning.as_ref().map(|t| t.period_ns)
    }

    /// Сбрасывает накопленное окно прореживания в журнал.
    pub(crate) fn flush(&mut self) {
        let Some(thinning) = &mut self.thinning else { return };
        let buffer = std::mem::take(&mut thinning.buffer);
        thinning.window = None;
        let mut keep = vec![false; buffer.len()];
        let mut channels: std::collections::BTreeMap<u32, [usize; 3]> = std::collections::BTreeMap::new();
        for (i, entry) in buffer.iter().enumerate() {
            if entry.kind != LogKind::Truth || !entry.valid {
                keep[i] = true;
                continue;
            }
            let slot = channels.entry(entry.id).or_insert([i; 3]);
            if entry.value < buffer[slot[1]].value {
                slot[1] = i;
            }
            if entry.value > buffer[slot[2]].value {
                slot[2] = i;
            }
        }
        for slot in channels.values() {
            for &i in slot {
                keep[i] = true;
            }
        }
        let entries = Rc::make_mut(&mut self.entries);
        entries.extend(buffer.into_iter().zip(keep).filter(|(_, k)| *k).map(|(e, _)| e));
    }

    pub fn header(&self) -> &LogHeader {
        &self.header
    }

    pub fn entries(&self) -> &[LogEntry] {
        &self.entries
    }

    pub fn end(&self) -> LogEnd {
        self.end
    }

    /// Типизированные строки живого журнала. После чтения записи
    /// с диска доступны текстовые строки через process_entries.
    pub fn process_ticks(&self) -> Option<&[ProcessTick]> {
        self.process_ticks.as_deref().map(Vec::as_slice)
    }

    /// Аудит решателя и процессов, включая фактические эффекты и причины отказов.
    pub fn process_entries(&self) -> &[(Time, String)] {
        if self
            .process_ticks
            .as_ref()
            .is_none_or(|rows| rows.is_empty())
        {
            return &self.processes;
        }
        self.rendered_processes.get_or_init(|| {
            Rc::new(
                self.process_rows()
                    .map(|row| {
                        let mut text = String::new();
                        row.render(&mut text);
                        (row.at(), text)
                    })
                    .collect(),
            )
        })
    }

    pub(crate) fn record_process(&mut self, at: Time, description: String) {
        self.rendered_processes.take();
        Rc::make_mut(&mut self.processes).push((at, description));
    }

    pub(crate) fn set_process_ticks(&mut self, ticks: Option<Rc<Vec<ProcessTick>>>) {
        self.rendered_processes.take();
        self.process_ticks = ticks;
    }

    fn process_count(&self) -> usize {
        self.processes.len() + self.process_ticks.as_ref().map_or(0, |ticks| ticks.len())
    }

    fn process_rows(&self) -> impl Iterator<Item = ProcessRow<'_>> {
        let mut text = self.processes.iter().peekable();
        let mut ticks = self
            .process_ticks
            .iter()
            .flat_map(|rows| rows.iter())
            .peekable();
        std::iter::from_fn(move || {
            // Описание состава на нуле предшествует первому такту. При ошибке
            // на границе физики запись ошибки есть, а записи такта нет.
            if let Some((at, _)) = text.peek()
                && ticks.peek().is_none_or(|tick| *at <= tick.at)
            {
                text.next().map(ProcessRow::Text)
            } else {
                ticks.next().map(ProcessRow::Tick)
            }
        })
    }

    pub(crate) fn push(&mut self, entry: LogEntry) {
        if let Some(thinning) = &mut self.thinning {
            let window = entry.at.0 / thinning.period_ns;
            if thinning.window.is_some_and(|current| current != window) {
                self.flush();
            }
            let thinning = self.thinning.as_mut().expect("прореживание включено");
            thinning.window = Some(window);
            thinning.buffer.push(entry);
            return;
        }
        Rc::make_mut(&mut self.entries).push(entry);
    }

    pub(crate) fn matches(&self, other: &Self) -> bool {
        self.header == other.header
            && self.end == other.end
            && self.process_count() == other.process_count()
            && self.entries.len() == other.entries.len()
            && self
                .entries
                .iter()
                .zip(other.entries.iter())
                .all(|(a, b)| a == b && a.value.to_bits() == b.value.to_bits())
            && {
                let mut left = String::new();
                let mut right = String::new();
                self.process_rows().zip(other.process_rows()).all(|(a, b)| {
                    a.render(&mut left);
                    b.render(&mut right);
                    a.at() == b.at() && left == right
                })
            }
    }

    fn write_payload(&self, w: &mut impl Write) -> io::Result<()> {
        w.write_all(MAGIC)?;
        let process_count = self.process_count();
        let version = if process_count == 0 {
            VERSION
        } else {
            VERSION_WITH_PROCESSES
        };
        w.write_all(&version.to_le_bytes())?;
        w.write_all(&self.header.world_hash.to_le_bytes())?;
        w.write_all(&self.header.seed.to_le_bytes())?;
        for table in [
            &self.header.observations,
            &self.header.commands,
            &self.header.truth,
        ] {
            write_len(w, table.len())?;
            for spec in table {
                for field in [&spec.name, &spec.unit] {
                    write_len(w, field.len())?;
                    w.write_all(field.as_bytes())?;
                }
            }
        }
        w.write_all(&(self.entries.len() as u64).to_le_bytes())?;
        for entry in self.entries.iter() {
            // Номера видов и источников фиксированы форматом версии 1.
            let (kind, source, controller) = match entry.kind {
                LogKind::Observation => (0, 0, 0),
                LogKind::Command(CommandSource::BeforeStart) => (1, 0, 0),
                LogKind::Command(CommandSource::External) => (1, 1, 0),
                LogKind::Command(CommandSource::Controller(id)) => (1, 2, id),
                LogKind::Truth => (2, 0, 0),
            };
            w.write_all(&[kind, source, u8::from(entry.valid)])?;
            w.write_all(&controller.to_le_bytes())?;
            w.write_all(&entry.id.to_le_bytes())?;
            w.write_all(&entry.at.0.to_le_bytes())?;
            w.write_all(&entry.value.to_bits().to_le_bytes())?;
        }
        w.write_all(&self.end.time.0.to_le_bytes())?;
        w.write_all(&self.end.steps.to_le_bytes())?;
        w.write_all(&self.end.state_hash.to_le_bytes())?;
        w.write_all(&[self.end.status.code(), u8::from(self.end.started)])?;
        let controller = self.end.failed_controller.map_or(0, |id| u64::from(id) + 1);
        w.write_all(&controller.to_le_bytes())?;
        if version == VERSION_WITH_PROCESSES {
            write_len(w, process_count)?;
            let mut description = String::new();
            for row in self.process_rows() {
                row.render(&mut description);
                w.write_all(&row.at().0.to_le_bytes())?;
                write_len(w, description.len())?;
                w.write_all(description.as_bytes())?;
            }
        }
        Ok(())
    }

    /// Отпечаток всех полей формата, включая биты значений, заголовок и итог.
    pub fn fingerprint(&self) -> u64 {
        let mut w = Hasher::new(io::sink());
        self.write_payload(&mut w)
            .expect("внутренний лог представим в формате");
        w.hash
    }

    pub fn write_binary(&self, writer: impl Write) -> io::Result<()> {
        let mut w = Hasher::new(BufWriter::new(writer));
        self.write_payload(&mut w)?;
        w.inner.write_all(&w.hash.to_le_bytes())?;
        w.inner.flush()
    }

    pub fn read_binary(mut reader: impl Read) -> io::Result<Self> {
        let mut bytes = Vec::new();
        reader.read_to_end(&mut bytes)?;
        let (bytes, checksum) = bytes
            .split_at_checked(bytes.len().saturating_sub(8))
            .filter(|(_, checksum)| checksum.len() == 8)
            .ok_or_else(|| invalid("нет контрольной суммы"))?;
        if fnv1a(bytes.iter().copied()) != u64::from_le_bytes(checksum.try_into().unwrap()) {
            return Err(invalid("не совпадает контрольная сумма журнала"));
        }

        let mut r = Input(bytes);
        if r.take(MAGIC.len())? != MAGIC {
            return Err(invalid("неизвестный формат или версия журнала"));
        }
        let version = u16::from_le_bytes(r.array()?);
        if !matches!(version, VERSION | VERSION_WITH_PROCESSES) {
            return Err(invalid("неизвестная версия журнала"));
        }
        let header = LogHeader {
            world_hash: r.u64()?,
            seed: r.u64()?,
            observations: r.table()?,
            commands: r.table()?,
            truth: r.table()?,
        };
        let count = usize::try_from(r.u64()?).map_err(invalid)?;
        let entries_bytes =
            r.0.len()
                .checked_sub(END_BYTES)
                .ok_or_else(|| invalid("нет итога журнала"))?;
        if count
            .checked_mul(ENTRY_BYTES)
            .is_none_or(|n| n > entries_bytes || (version == VERSION && n != entries_bytes))
        {
            return Err(invalid("неверная длина записей журнала"));
        }
        let mut entries = Vec::new();
        let mut previous = Time::ZERO;
        let mut before_start = true;
        for _ in 0..count {
            let [kind, source, valid] = r.array()?;
            let controller = r.u32()?;
            let id = r.u32()?;
            let at = Time(r.u64()?);
            let value = f64::from_bits(r.u64()?);
            let (kind, table) = match (kind, source, controller, valid) {
                (0, 0, 0, 0 | 1) => (LogKind::Observation, &header.observations),
                (1, 0, 0, 1) if before_start && at == Time::ZERO => (
                    LogKind::Command(CommandSource::BeforeStart),
                    &header.commands,
                ),
                (1, 1, 0, 1) => (LogKind::Command(CommandSource::External), &header.commands),
                (1, 2, controller, 1) => (
                    LogKind::Command(CommandSource::Controller(controller)),
                    &header.commands,
                ),
                (2, 0, 0, 1) => (LogKind::Truth, &header.truth),
                _ => return Err(invalid("неверный вид записи журнала")),
            };
            before_start &= kind == LogKind::Command(CommandSource::BeforeStart);
            if !value.is_finite() || at < previous || table.get(id as usize).is_none() {
                return Err(invalid(
                    "неверное значение, время или индекс записи журнала",
                ));
            }
            entries.push(LogEntry {
                at,
                kind,
                id,
                value,
                valid: valid == 1,
            });
            previous = at;
        }
        let time = Time(r.u64()?);
        let steps = r.u64()?;
        let state_hash = r.u64()?;
        let [status, started] = r.array()?;
        let status = Status::from_code(status)?;
        let started = match started {
            0 => false,
            1 => true,
            _ => return Err(invalid("неверный признак начала расчёта")),
        };
        let failed_controller = r
            .u64()?
            .checked_sub(1)
            .map(u32::try_from)
            .transpose()
            .map_err(invalid)?;
        let end = LogEnd {
            time,
            steps,
            state_hash,
            status,
            started,
            failed_controller,
        };
        if end.time < previous
            || steps > time.0
            || (!started
                && (!before_start || time != Time::ZERO || steps != 0 || status != Status::Running))
            || (failed_controller.is_some() && status != Status::ComponentFailure)
        {
            return Err(invalid("итог журнала противоречит записям"));
        }
        let mut processes = Vec::new();
        if version == VERSION_WITH_PROCESSES {
            let count = r.u32()?;
            if count == 0 || !end.started {
                return Err(invalid("пустая секция процессов или прогон не начат"));
            }
            let mut previous = Time::ZERO;
            for _ in 0..count {
                let at = Time(r.u64()?);
                let len = r.u32()? as usize;
                let description = std::str::from_utf8(r.take(len)?).map_err(invalid)?;
                if at < previous || at > end.time || description.is_empty() {
                    return Err(invalid("неверное время или пустая запись процесса"));
                }
                processes.push((at, description.to_owned()));
                previous = at;
            }
        }
        if !r.0.is_empty() {
            return Err(invalid("лишние данные после журнала"));
        }
        Ok(Self {
            header,
            entries: Rc::new(entries),
            end,
            processes: Rc::new(processes),
            process_ticks: None,
            rendered_processes: OnceCell::new(),
            thinning: None,
        })
    }

    /// Длинный CSV: строка на событие, отдельно вид, источник, имя, единица и достоверность.
    pub fn write_csv(&self, writer: impl Write) -> io::Result<()> {
        let mut w = BufWriter::new(writer);
        writeln!(w, "time_ns,kind,source,name,unit,valid,value")?;
        for e in self.entries.iter() {
            let (kind, source, table) = match e.kind {
                LogKind::Observation => ("observation", String::new(), &self.header.observations),
                LogKind::Truth => ("truth", String::new(), &self.header.truth),
                LogKind::Command(source) => (
                    "command",
                    match source {
                        CommandSource::BeforeStart => "before_start".into(),
                        CommandSource::External => "external".into(),
                        CommandSource::Controller(id) => format!("controller/{id}"),
                    },
                    &self.header.commands,
                ),
            };
            let spec = &table[e.id as usize];
            write!(w, "{},{},{},", e.at.0, kind, source)?;
            csv_field(&mut w, &spec.name)?;
            w.write_all(b",")?;
            csv_field(&mut w, &spec.unit)?;
            writeln!(w, ",{},{}", e.valid, e.value)?;
        }
        w.flush()
    }
}

fn csv_field(w: &mut impl Write, field: &str) -> io::Result<()> {
    w.write_all(b"\"")?;
    w.write_all(field.replace('"', "\"\"").as_bytes())?;
    w.write_all(b"\"")
}

fn invalid(message: impl std::fmt::Display) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message.to_string())
}

fn write_len(w: &mut impl Write, len: usize) -> io::Result<()> {
    w.write_all(&u32::try_from(len).map_err(invalid)?.to_le_bytes())
}

struct Input<'a>(&'a [u8]);

impl<'a> Input<'a> {
    fn take(&mut self, n: usize) -> io::Result<&'a [u8]> {
        let (head, rest) = self
            .0
            .split_at_checked(n)
            .ok_or_else(|| invalid("обрезанный журнал"))?;
        self.0 = rest;
        Ok(head)
    }

    fn array<const N: usize>(&mut self) -> io::Result<[u8; N]> {
        Ok(self.take(N)?.try_into().unwrap())
    }

    fn u32(&mut self) -> io::Result<u32> {
        Ok(u32::from_le_bytes(self.array()?))
    }

    fn u64(&mut self) -> io::Result<u64> {
        Ok(u64::from_le_bytes(self.array()?))
    }

    fn text(&mut self) -> io::Result<String> {
        let len = self.u32()? as usize;
        std::str::from_utf8(self.take(len)?)
            .map(str::to_owned)
            .map_err(invalid)
    }

    fn table(&mut self) -> io::Result<Vec<SignalSpec>> {
        let len = self.u32()? as usize;
        let mut table = Vec::new();
        let mut names = std::collections::BTreeSet::new();
        for _ in 0..len {
            let name = self.text()?;
            let unit = self.text()?;
            if name.trim().is_empty() || !names.insert(name.clone()) {
                return Err(invalid("пустое или повторное имя сигнала в журнале"));
            }
            table.push(SignalSpec { name, unit });
        }
        Ok(table)
    }
}

struct Hasher<W> {
    inner: W,
    hash: u64,
}

impl<W> Hasher<W> {
    fn new(inner: W) -> Self {
        Self {
            inner,
            hash: fnv1a([]),
        }
    }
}

impl<W: Write> Write for Hasher<W> {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        let n = self.inner.write(bytes)?;
        for &byte in &bytes[..n] {
            // Тот же множитель FNV-1a-64, что в sim_signal::fnv1a.
            self.hash = (self.hash ^ u64::from(byte)).wrapping_mul(0x0000_0100_0000_01b3);
        }
        Ok(n)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.inner.flush()
    }
}

#[cfg(test)]
mod thinning_tests {
    use super::*;

    fn entry(at: u64, kind: LogKind, id: u32, value: f64) -> LogEntry {
        LogEntry { at: Time(at), kind, id, value, valid: true }
    }

    #[test]
    fn прореживание_истины_хранит_первое_минимум_максимум_и_порядок() {
        let header = LogHeader { world_hash: 0, seed: 0, observations: vec![], commands: vec![], truth: vec![] };
        let end = LogEnd { time: Time::ZERO, steps: 0, state_hash: 0, status: Status::Running, started: false, failed_controller: None };
        let mut log = Log::new(header, end);
        log.set_truth_period(10);
        // Окно 0..10: истина 0 — 5, 1, 9, 3; истина 1 постоянна; наблюдение посередине.
        for (at, v) in [(0, 5.0), (2, 1.0), (4, 9.0), (6, 3.0)] {
            log.push(entry(at, LogKind::Truth, 0, v));
            log.push(entry(at, LogKind::Truth, 1, 7.0));
        }
        log.push(entry(5, LogKind::Observation, 0, 42.0));
        // Окно 10..20 начинается — прошлое сбрасывается.
        log.push(entry(10, LogKind::Truth, 0, 2.0));
        log.push(entry(14, LogKind::Truth, 0, 2.5));
        log.flush();
        let kept: Vec<_> = log.entries().iter().map(|e| (e.at.0, e.kind == LogKind::Truth, e.id, e.value)).collect();
        assert_eq!(
            kept,
            [
                (0, true, 0, 5.0),
                (0, true, 1, 7.0),
                (2, true, 0, 1.0),
                (4, true, 0, 9.0),
                (5, false, 0, 42.0),
                (10, true, 0, 2.0),
                (14, true, 0, 2.5),
            ]
        );
        assert!(log.entries().windows(2).all(|p| p[0].at <= p[1].at));
    }
}
