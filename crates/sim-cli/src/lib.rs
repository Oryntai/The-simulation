//! Драйвер мира без окна. Часы компьютера используются только для темпа realtime.

mod commands;
mod tasks;
pub mod view;

use std::collections::{BTreeMap, BTreeSet};
use std::ffi::OsString;
use std::fs::File;
use std::io::{BufRead, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use serde::Serialize;
use sim_runtime::{Log, LogEnd, LogEntry, LogKind, Runtime, Status};
use sim_signal::{Command, Time};
use sim_world::CompiledWorld;

const USAGE: &str = "sim-run <world.toml> --duration <с> [--catalog папка] [--commands файл.csv] [--seed N] [--log файл] [--csv файл] [--view файл [--peak канал]] [--mode fast|realtime|step] [--snapshot-at <с>] [--replay файл] [--json]\nsim-run <world.toml> --task <task.toml> [--case режим | --matrix full [--jobs N] [--checkpoint папка]] [--seed N] [--catalog папка] [--view файл [--peak канал]] [--json]\nsim-run --campaign <manifest.toml> [--checkpoint папка] [--repeat ключ [--view файл]] [--measurements файл] [--json]";

#[derive(Clone, Copy, Default)]
enum Mode {
    #[default]
    Fast,
    Realtime,
    Step,
}

#[derive(Clone, Default)]
struct Options {
    world: PathBuf,
    until: Time,
    seed: Option<u64>,
    catalog: Option<PathBuf>,
    commands: Option<PathBuf>,
    log: Option<PathBuf>,
    csv: Option<PathBuf>,
    view: Option<PathBuf>,
    peak: Option<String>,
    measurements: Option<PathBuf>,
    mode: Mode,
    snapshot_at: Option<Time>,
    replay: Option<PathBuf>,
    task: Option<PathBuf>,
    campaign: Option<PathBuf>,
    repeat: Option<String>,
    case: Option<String>,
    matrix_full: bool,
    selected_control_case: Option<sim_task::Case>,
    linked_grid_hz: Option<u64>,
    jobs: Option<std::num::NonZeroUsize>,
    checkpoint: Option<PathBuf>,
    json: bool,
}

impl Options {
    fn exports_run(&self) -> bool {
        self.log.is_some() || self.csv.is_some() || self.view.is_some()
    }

    fn record_view(&self, runtime: &mut Runtime) -> Result<(), String> {
        if self.view.is_some() {
            runtime.record_view()?;
        }
        Ok(())
    }

    fn parse(args: Vec<OsString>) -> Result<Self, String> {
        let mut options = Self::default();
        let mut used = BTreeSet::new();
        let mut args = args.into_iter();
        while let Some(arg) = args.next() {
            if !arg.as_encoded_bytes().starts_with(b"-") {
                if !options.world.as_os_str().is_empty() || arg.is_empty() {
                    return Err("укажите один файл мира".into());
                }
                options.world = arg.into();
                continue;
            }
            let flag = arg.to_str().ok_or("ключ должен быть в UTF-8")?;
            if !used.insert(flag.to_owned()) {
                return Err(format!("повтор ключа {flag}"));
            }
            if flag == "--json" {
                options.json = true;
                continue;
            }
            if !matches!(
                flag,
                "--duration"
                    | "--seed"
                    | "--catalog"
                    | "--commands"
                    | "--log"
                    | "--csv"
                    | "--view"
                    | "--peak"
                    | "--measurements"
                    | "--mode"
                    | "--snapshot-at"
                    | "--replay"
                    | "--task"
                    | "--campaign"
                    | "--repeat"
                    | "--case"
                    | "--matrix"
                    | "--jobs"
                    | "--checkpoint"
            ) {
                return Err(format!("неизвестный ключ {flag}"));
            }
            let value = args
                .next()
                .filter(|v| !v.is_empty() && !v.as_encoded_bytes().starts_with(b"--"))
                .ok_or_else(|| format!("нет значения для {flag}"))?;
            let as_text = || {
                value
                    .to_str()
                    .ok_or_else(|| format!("значение {flag} должно быть в UTF-8"))
            };
            let time = || {
                as_text()?
                    .parse::<f64>()
                    .ok()
                    .and_then(Time::from_secs)
                    .ok_or_else(|| {
                        format!("{flag}: укажите конечное неотрицательное время в секундах")
                    })
            };
            match flag {
                "--duration" => options.until = time()?,
                "--snapshot-at" => options.snapshot_at = Some(time()?),
                "--seed" => {
                    options.seed = Some(
                        as_text()?
                            .parse()
                            .map_err(|_| "--seed: укажите целое u64")?,
                    )
                }
                "--mode" => {
                    options.mode = match as_text()? {
                        "fast" => Mode::Fast,
                        "realtime" => Mode::Realtime,
                        "step" => Mode::Step,
                        _ => return Err("--mode: допустимы fast, realtime и step".into()),
                    }
                }
                "--log" => options.log = Some(value.into()),
                "--csv" => options.csv = Some(value.into()),
                "--view" => options.view = Some(value.into()),
                "--peak" => options.peak = Some(as_text()?.into()),
                "--measurements" => options.measurements = Some(value.into()),
                "--catalog" => options.catalog = Some(value.into()),
                "--commands" => options.commands = Some(value.into()),
                "--replay" => options.replay = Some(value.into()),
                "--task" => options.task = Some(value.into()),
                "--campaign" => options.campaign = Some(value.into()),
                "--repeat" => options.repeat = Some(as_text()?.into()),
                "--case" => options.case = Some(as_text()?.into()),
                "--matrix" => {
                    if as_text()? != "full" {
                        return Err("--matrix: допустимо только full".into());
                    }
                    options.matrix_full = true;
                }
                "--jobs" => {
                    options.jobs = Some(
                        as_text()?
                            .parse()
                            .map_err(|_| "--jobs: укажите положительное целое число потоков")?,
                    )
                }
                "--checkpoint" => options.checkpoint = Some(value.into()),
                _ => unreachable!(),
            }
        }
        if options.campaign.is_some() {
            if !options.world.as_os_str().is_empty()
                || used.iter().any(|flag| {
                    !matches!(
                        flag.as_str(),
                        "--campaign" | "--checkpoint" | "--repeat" | "--view" | "--measurements" | "--json"
                    )
                })
            {
                return Err("для --campaign допустимы --checkpoint, --repeat, --view, --measurements и --json".into());
            }
            if options.view.is_some() && options.repeat.is_none() {
                return Err("просмотр кампании требует ключ --repeat".into());
            }
            return Ok(options);
        }
        if options.repeat.is_some() || options.measurements.is_some() {
            return Err("--repeat и --measurements требуют --campaign".into());
        }
        if options.world.as_os_str().is_empty()
            || options.task.is_some() == used.contains("--duration")
        {
            return Err("нужны файл мира и ровно один из ключей --duration или --task".into());
        }
        if options.task.is_some()
            && ["--commands", "--replay", "--mode", "--snapshot-at"]
                .into_iter()
                .any(|flag| used.contains(flag))
        {
            return Err("задание использует собственные стимулы, расписание и горизонт".into());
        }
        if options.task.is_none() && options.case.is_some() {
            return Err("--case требуется --task".into());
        }
        if options.matrix_full
            && (options.task.is_none()
                || options.case.is_some()
                || options.exports_run())
        {
            return Err("--matrix full требует --task и исключает --case, --log, --csv и --view".into());
        }
        if (!options.matrix_full && options.checkpoint.is_some())
            || (options.task.is_none() && options.jobs.is_some())
        {
            return Err("--jobs требует --task, --checkpoint требует --matrix full".into());
        }
        if options.snapshot_at.is_some_and(|time| time > options.until) {
            return Err("--snapshot-at выходит за --duration".into());
        }
        let mut outputs = BTreeSet::new();
        if [&options.log, &options.csv, &options.view]
            .into_iter().flatten().any(|path| !outputs.insert(path))
        {
            return Err("--log, --csv и --view требуют разных файлов".into());
        }
        if options.peak.is_some() && options.view.is_none() {
            return Err("--peak требует --view".into());
        }
        if options.commands.is_some() && options.replay.is_some() {
            return Err("--commands и --replay нельзя использовать одновременно".into());
        }
        Ok(options)
    }
}

enum Driver {
    Fast,
    Realtime(Instant),
    Step,
}

impl Driver {
    fn new(mode: Mode) -> Self {
        match mode {
            Mode::Fast => Self::Fast,
            Mode::Realtime => Self::Realtime(Instant::now()),
            Mode::Step => Self::Step,
        }
    }

    /// false означает, что stdin закончился до очередного шага.
    fn advance(
        &self,
        runtime: &mut Runtime,
        until: Time,
        stdin: &mut impl BufRead,
        diagnostics: &mut impl Write,
    ) -> Result<bool, String> {
        if let Self::Fast = self {
            runtime.run_until(until)?;
            return Ok(true);
        }
        loop {
            if runtime.status() != Status::Running {
                return Ok(true);
            }
            let due = match self {
                Self::Realtime(_) => runtime.next_event_time(),
                Self::Step => runtime.next_physics_time(),
                Self::Fast => unreachable!(),
            };
            let time = due.map_or(until, |due| due.min(until));
            match self {
                Self::Realtime(start) => {
                    let delay = Duration::from_nanos(time.0).saturating_sub(start.elapsed());
                    if !delay.is_zero() {
                        std::thread::sleep(delay);
                    }
                }
                Self::Step if due == Some(time) => {
                    writeln!(diagnostics, "Enter для шага к {} с", time.as_secs())
                        .and_then(|_| diagnostics.flush())
                        .map_err(|e| e.to_string())?;
                    let mut line = String::new();
                    if stdin.read_line(&mut line).map_err(|e| e.to_string())? == 0 {
                        return Ok(false);
                    }
                }
                _ => {}
            }
            runtime.run_until(time)?;
            if time == until {
                return Ok(true);
            }
        }
    }
}

#[derive(Serialize)]
struct Approximation<'a> {
    subject: &'a str,
    text: &'a str,
    blocks_proof: bool,
}

#[derive(Serialize)]
struct Checkpoint {
    time_ns: u64,
    steps: u64,
    state_hash: String,
}

impl From<LogEnd> for Checkpoint {
    fn from(end: LogEnd) -> Self {
        Self {
            time_ns: end.time.0,
            steps: end.steps,
            state_hash: format!("{:016x}", end.state_hash),
        }
    }
}

#[derive(Serialize)]
struct ChannelValue {
    value: f64,
    time_ns: u64,
    valid: bool,
}

impl From<&LogEntry> for ChannelValue {
    fn from(entry: &LogEntry) -> Self {
        Self {
            value: entry.value,
            time_ns: entry.at.0,
            valid: entry.valid,
        }
    }
}

type ChannelValues<'a> = BTreeMap<&'a str, Option<ChannelValue>>;

#[derive(Default, Serialize)]
struct Channels<'a> {
    commands: ChannelValues<'a>,
    observations: ChannelValues<'a>,
    truth: ChannelValues<'a>,
}

impl<'a> From<&'a Log> for Channels<'a> {
    fn from(log: &'a Log) -> Self {
        let header = log.header();
        let mut commands = vec![None; header.commands.len()];
        let mut observations = vec![None; header.observations.len()];
        let mut truth = vec![None; header.truth.len()];
        for entry in log.entries() {
            let values = match entry.kind {
                LogKind::Command(_) => &mut commands,
                LogKind::Observation => &mut observations,
                LogKind::Truth => &mut truth,
            };
            values[entry.id as usize] = Some(entry);
        }
        let named = |specs: &'a [sim_signal::SignalSpec], values: Vec<Option<&LogEntry>>| {
            specs
                .iter()
                .zip(values)
                .map(|(spec, value)| (spec.name.as_str(), value.map(ChannelValue::from)))
                .collect()
        };
        Self {
            commands: named(&header.commands, commands),
            observations: named(&header.observations, observations),
            truth: named(&header.truth, truth),
        }
    }
}

#[derive(Serialize)]
struct Report<'a> {
    status: &'a str,
    time_s: Option<f64>,
    time_ns: Option<u64>,
    steps: Option<u64>,
    log_fingerprint: Option<String>,
    blocks_proof: bool,
    proof_status: &'static str,
    component_failure: Option<&'a str>,
    channels: Channels<'a>,
    simplifications: Vec<Approximation<'a>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    snapshot: Option<Checkpoint>,
    #[serde(skip_serializing_if = "Option::is_none")]
    detail: Option<&'a str>,
}

fn write_report(report: &Report<'_>, json: bool, stdout: &mut impl Write) -> Result<(), String> {
    if json {
        serde_json::to_writer(&mut *stdout, report).map_err(|e| e.to_string())?;
        return writeln!(stdout).map_err(|e| e.to_string());
    }
    writeln!(
        stdout,
        "Статус {}, время {} с, шагов {}, отпечаток {}",
        report.status,
        report.time_s.unwrap(),
        report.steps.unwrap(),
        report.log_fingerprint.as_deref().unwrap()
    )
    .map_err(|e| e.to_string())?;
    for approximation in &report.simplifications {
        writeln!(
            stdout,
            "Упрощение «{}» — {} (блокирует доказательство: {})",
            approximation.subject, approximation.text, approximation.blocks_proof
        )
        .map_err(|e| e.to_string())?;
    }
    Ok(())
}

fn read_log(path: &Path) -> Result<Log, String> {
    File::open(path)
        .and_then(Log::read_binary)
        .map_err(|e| format!("{}: {e}", path.display()))
}

fn export(path: &Path, write: impl FnOnce(&mut File) -> std::io::Result<()>) -> Result<(), String> {
    let mut file = File::create_new(path).map_err(|e| format!("{}: {e}", path.display()))?;
    if let Err(error) = write(&mut file) {
        drop(file);
        let cleanup = std::fs::remove_file(path);
        return Err(match cleanup {
            Ok(()) => format!("{}: {error}", path.display()),
            Err(cleanup) => format!(
                "{}: {error}; не удалось удалить неполный файл: {cleanup}",
                path.display()
            ),
        });
    }
    Ok(())
}

fn catalog_path(options: &Options) -> Result<PathBuf, String> {
    if let Some(path) = &options.catalog {
        return Ok(path.clone());
    }
    let directory = options
        .world
        .parent()
        .filter(|path| !path.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    let directory = directory
        .canonicalize()
        .map_err(|err| format!("{}: {err}", directory.display()))?;
    for parent in directory.ancestors() {
        let path = parent.join("catalog");
        match path.metadata() {
            Ok(info) if info.is_dir() => return Ok(path),
            Ok(_) => return Err(format!("{}: требуется папка каталога", path.display())),
            Err(err) if err.kind() == std::io::ErrorKind::NotFound => {}
            Err(err) => return Err(format!("{}: {err}", path.display())),
        }
    }
    Err("каталог для мира не найден, укажите --catalog".into())
}

fn install(runtime: &mut Runtime, world: &CompiledWorld, options: &Options) -> Result<(), String> {
    if world.drives.is_empty() && world.sensors.is_empty() {
        return Ok(());
    }
    let catalog = catalog_path(options)?;
    if !world.drives.is_empty() {
        let drives = sim_drive::DriveCatalog::load(catalog.join("drives"))?;
        sim_drive::install(runtime, world, &drives)?;
    }
    if !world.sensors.is_empty() {
        let sensors = sim_sense::SensorCatalog::load(catalog.join("sensors"))?;
        sim_sense::install(runtime, world, &sensors)?;
    }
    Ok(())
}

struct RunDriver<'a, I, W> {
    driver: Driver,
    commands: std::iter::Peekable<std::vec::IntoIter<(Time, Command)>>,
    stdin: &'a mut I,
    diagnostics: &'a mut W,
}

impl<I: BufRead, W: Write> RunDriver<'_, I, W> {
    fn advance(&mut self, runtime: &mut Runtime, until: Time) -> Result<bool, String> {
        while let Some((at, _)) = self.commands.peek().filter(|(at, _)| *at <= until) {
            let at = *at;
            if at != Time::ZERO
                && !self
                    .driver
                    .advance(runtime, at, self.stdin, self.diagnostics)?
            {
                return Ok(false);
            }
            if runtime.status() != Status::Running {
                return Ok(true);
            }
            let (_, command) = self.commands.next().unwrap();
            runtime.act(&command)?;
        }
        self.driver
            .advance(runtime, until, self.stdin, self.diagnostics)
    }
}

fn run_world(
    mut world: CompiledWorld,
    options: &Options,
    stdin: &mut impl BufRead,
    stdout: &mut impl Write,
    diagnostics: &mut impl Write,
) -> Result<u8, String> {
    let replay = options.replay.as_deref().map(read_log).transpose()?;
    if let Some(log) = &replay
        && log.end().time != options.until
    {
        return Err(format!(
            "--duration должен совпадать с концом записи: {} с",
            log.end().time.as_secs()
        ));
    }
    if let Some(seed) = options
        .seed
        .or_else(|| replay.as_ref().map(|log| log.header().seed))
    {
        world.seed = seed;
    }
    let mut runtime = Runtime::load(world.clone())?;
    install(&mut runtime, &world, options)?;
    drop(world);
    runtime.record_joint_state("запись состояния")?;
    options.record_view(&mut runtime)?;
    let commands = options
        .commands
        .as_deref()
        .map(|path| commands::read(path, &runtime, options.until))
        .transpose()?
        .unwrap_or_default();
    let replaying = replay.is_some();
    if let Some(log) = replay {
        runtime.replay(log)?;
    }

    let mut driver = RunDriver {
        driver: Driver::new(options.mode),
        commands: commands.into_iter().peekable(),
        stdin,
        diagnostics,
    };
    let mut snapshot = None;
    let mut complete = true;
    if let Some(at) = options.snapshot_at {
        complete = driver.advance(&mut runtime, at)?;
        if complete && runtime.time() == at {
            snapshot = Some(runtime.snapshot());
        }
    }
    if complete {
        complete = driver.advance(&mut runtime, options.until)?;
    }
    if replaying && complete {
        runtime.verify_replay()?;
    }

    view::save(options, &runtime, None, None)?;
    let log = runtime.log();
    if let Some(path) = &options.log {
        export(path, |out| log.write_binary(out))?;
    }
    if let Some(path) = &options.csv {
        export(path, |out| log.write_csv(out))?;
    }
    let status = match runtime.status() {
        Status::Running if complete => "completed",
        Status::Running => "interrupted",
        Status::NumericalFailure => "numerical_failure",
        Status::ComponentFailure => "component_failure",
    };
    let report = Report {
        status,
        time_s: Some(runtime.time().as_secs()),
        time_ns: Some(runtime.time().0),
        steps: Some(runtime.steps()),
        log_fingerprint: Some(format!("{:016x}", log.fingerprint())),
        blocks_proof: status != "completed"
            || runtime
                .simplifications()
                .iter()
                .any(|item| item.blocks_proof),
        proof_status: "not_analyzed",
        component_failure: if runtime.status() == Status::ComponentFailure {
            runtime.failure()
        } else {
            None
        },
        channels: if options.json {
            log.into()
        } else {
            Channels::default()
        },
        simplifications: runtime
            .simplifications()
            .iter()
            .map(|item| Approximation {
                subject: &item.subject,
                text: &item.text,
                blocks_proof: item.blocks_proof,
            })
            .collect(),
        snapshot: snapshot.as_ref().map(|snapshot| snapshot.end().into()),
        detail: runtime.failure().or(if complete {
            None
        } else {
            Some("ввод завершился до следующего шага")
        }),
    };
    write_report(&report, options.json, stdout)?;
    if !options.json
        && let Some(detail) = report.detail
    {
        writeln!(driver.diagnostics, "{detail}").map_err(|e| e.to_string())?;
    }
    Ok(u8::from(status != "completed"))
}

/// Запустить CLI. Возвращает код завершения процесса.
pub fn run(
    args: impl IntoIterator<Item = OsString>,
    mut stdin: impl BufRead,
    mut stdout: impl Write,
    mut diagnostics: impl Write,
) -> u8 {
    let args: Vec<_> = args.into_iter().collect();
    if args.len() == 1 && matches!(args[0].to_str(), Some("--help" | "-h")) {
        return if writeln!(stdout, "{USAGE}").is_ok() {
            0
        } else {
            2
        };
    }
    let json = args.iter().any(|arg| arg == "--json");
    let result = Options::parse(args).and_then(|options| {
        if let Some(campaign) = &options.campaign {
            return tasks::run_campaign(campaign, &options, &mut stdout);
        }
        let world = sim_world::load_and_compile(&options.world)?;
        if let Some(task) = &options.task {
            return tasks::run(world, task, &options, &mut stdout);
        }
        run_world(world, &options, &mut stdin, &mut stdout, &mut diagnostics)
    });
    match result {
        Ok(code) => code,
        Err(error) => {
            if json {
                let report = Report {
                    status: "error",
                    time_s: None,
                    time_ns: None,
                    steps: None,
                    log_fingerprint: None,
                    blocks_proof: true,
                    proof_status: "not_analyzed",
                    component_failure: None,
                    channels: Channels::default(),
                    simplifications: Vec::new(),
                    snapshot: None,
                    detail: Some(&error),
                };
                if write_report(&report, true, &mut stdout).is_err() {
                    let _ = writeln!(diagnostics, "sim-run: {error}");
                }
            } else {
                let _ = writeln!(diagnostics, "sim-run: {error}\n{USAGE}");
            }
            2
        }
    }
}
