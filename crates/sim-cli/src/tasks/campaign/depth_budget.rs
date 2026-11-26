//! Бюджет координатора для одного устройства и одного плана покрытия.

use std::fmt;
use std::fs::{self, File, TryLockError};
use std::io;
use std::path::{Path, PathBuf};
#[cfg(unix)]
use std::process::Child;
use std::process::{Command, ExitStatus};
use std::sync::{Condvar, Mutex, MutexGuard};
use std::time::{Duration, Instant, SystemTime};

use serde::{Deserialize, Serialize};
use sim_signal::fnv1a;

// Потолки из блока Е третьего ТЗ.
const MAX_RUNS: u32 = 1000;
const MAX_TIME: Duration = Duration::from_secs(30 * 60);

const VERSION: u32 = 1;
const STATE: &str = "depth-budget-v1.json";
const PART: &str = "depth-budget-v1.part";
const LOCK: &str = "depth-budget-v1.lock";

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DepthLimits {
    runs: u32,
    time: Duration,
}

impl DepthLimits {
    pub fn new(runs: u32, time: Duration) -> Result<Self, BudgetError> {
        if runs > MAX_RUNS || time > MAX_TIME {
            return Err(BudgetError::InvalidLimits);
        }
        Ok(Self { runs, time })
    }

    pub(crate) fn time(self) -> Duration {
        self.time
    }
}

#[derive(Debug)]
pub enum BudgetError {
    InvalidLimits,
    QuotaExhausted,
    DeadlineExceeded,
    Cancelled,
    CheckpointBusy,
    Checkpoint(String),
    Process(io::Error),
}

impl fmt::Display for BudgetError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidLimits => {
                write!(f, "бюджет превышает {MAX_RUNS} запусков или {MAX_TIME:?}")
            }
            Self::QuotaExhausted => f.write_str("исчерпана квота запусков"),
            Self::DeadlineExceeded => f.write_str("истёк дедлайн кампании"),
            Self::Cancelled => f.write_str("кампания отменена"),
            Self::CheckpointBusy => f.write_str("точка бюджета уже открыта координатором"),
            Self::Checkpoint(reason) => write!(f, "состояние бюджета: {reason}"),
            Self::Process(error) => write!(f, "дочерний процесс: {error}"),
        }
    }
}

impl std::error::Error for BudgetError {}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DepthUsage {
    pub reserved: u32,
    pub remaining_runs: u32,
    pub elapsed: Duration,
    pub remaining_time: Duration,
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Record {
    version: u32,
    plan: String,
    limits: DepthLimits,
    started_unix: Duration,
    observed_unix: Duration,
    elapsed: Duration,
    reserved: u32,
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Envelope {
    checksum: u64,
    payload: String,
}

struct State {
    record: Record,
    cancelled: bool,
    write_failed: bool,
    cleanup_failed: bool,
    active: u32,
}

pub struct DepthBudget {
    opened: Instant,
    previously_elapsed: Duration,
    checkpoint: Option<Checkpoint>,
    state: Mutex<State>,
    idle: Condvar,
}

impl DepthBudget {
    /// Вызывать в координаторе в начале кампании, до построения плана и установки.
    /// Идентификатор связывает бюджет с данными, версией и порядком плана.
    pub fn start(
        limits: DepthLimits,
        plan: &str,
        directory: Option<&Path>,
    ) -> Result<Self, BudgetError> {
        Self::start_at(limits, plan, directory, Instant::now())
    }

    pub(crate) fn start_at(
        limits: DepthLimits,
        plan: &str,
        directory: Option<&Path>,
        opened: Instant,
    ) -> Result<Self, BudgetError> {
        DepthLimits::new(limits.runs, limits.time)?;
        if plan.trim().is_empty() {
            return Err(checkpoint_error("пустой идентификатор плана"));
        }
        let now = unix_now()?;
        let started_unix = now.saturating_sub(opened.elapsed());
        let checkpoint = directory
            .map(|directory| {
                fs::create_dir_all(directory).map_err(checkpoint_error)?;
                Checkpoint::lock(directory)
            })
            .transpose()?;
        let record = Record {
            version: VERSION,
            plan: plan.to_owned(),
            limits,
            started_unix,
            observed_unix: now,
            elapsed: now - started_unix,
            reserved: 0,
        };
        if let Some(checkpoint) = &checkpoint {
            match checkpoint.directory.join(STATE).symlink_metadata() {
                Ok(_) => return Err(checkpoint_error("бюджет уже создан, требуется продолжение")),
                Err(error) if error.kind() == io::ErrorKind::NotFound => {}
                Err(error) => return Err(checkpoint_error(error)),
            }
            checkpoint.save(&record)?;
        }
        Ok(Self::new(opened, Duration::ZERO, checkpoint, record))
    }

    pub fn resume(limits: DepthLimits, plan: &str, directory: &Path) -> Result<Self, BudgetError> {
        DepthLimits::new(limits.runs, limits.time)?;
        let opened = Instant::now();
        let now = unix_now()?;
        let checkpoint = Checkpoint::lock(directory)?;
        let record = checkpoint.load()?;
        if record.version != VERSION
            || record.plan != plan
            || plan.trim().is_empty()
            || record.limits != limits
            || record.reserved > limits.runs
            || record.observed_unix < record.started_unix
        {
            return Err(checkpoint_error(
                "точка бюджета не соответствует плану или пределам",
            ));
        }
        if now < record.observed_unix {
            return Err(checkpoint_error(
                "часы отстают от сохранённой отметки кампании",
            ));
        }
        // Настенные часы учитывают перерыв между сеансами и сон компьютера.
        // Монотонное время внутри сеанса не даёт увеличить бюджет сдвигом часов.
        let elapsed = record.elapsed.max(now - record.started_unix);
        Ok(Self::new(opened, elapsed, Some(checkpoint), record))
    }

    fn new(
        opened: Instant,
        previously_elapsed: Duration,
        checkpoint: Option<Checkpoint>,
        record: Record,
    ) -> Self {
        Self {
            opened,
            previously_elapsed,
            checkpoint,
            state: Mutex::new(State {
                record,
                cancelled: false,
                write_failed: false,
                cleanup_failed: false,
                active: 0,
            }),
            idle: Condvar::new(),
        }
    }

    fn state(&self) -> Result<MutexGuard<'_, State>, BudgetError> {
        self.state
            .lock()
            .map_err(|_| checkpoint_error("нарушена целостность бюджета"))
    }

    fn elapsed_at(&self, record: &Record, now: Duration) -> Result<Duration, BudgetError> {
        if now < record.observed_unix {
            return Err(checkpoint_error(
                "часы отстают от сохранённой отметки кампании",
            ));
        }
        let monotonic = self
            .previously_elapsed
            .saturating_add(self.opened.elapsed());
        Ok(monotonic.max(now - record.started_unix))
    }

    fn time_left(&self, state: &State) -> Result<Duration, BudgetError> {
        if state.cleanup_failed {
            return Err(checkpoint_error("ошибка завершения дочернего процесса"));
        }
        if state.write_failed {
            return Err(checkpoint_error("сохранение резерва не подтверждено"));
        }
        if state.cancelled {
            return Err(BudgetError::Cancelled);
        }
        let elapsed = self.elapsed_at(&state.record, unix_now()?)?;
        let remaining = state.record.limits.time.saturating_sub(elapsed);
        if remaining.is_zero() {
            return Err(BudgetError::DeadlineExceeded);
        }
        Ok(remaining)
    }

    pub fn usage(&self) -> Result<DepthUsage, BudgetError> {
        let state = self.state()?;
        let elapsed = self.elapsed_at(&state.record, unix_now()?)?;
        Ok(DepthUsage {
            reserved: state.record.reserved,
            remaining_runs: state.record.limits.runs - state.record.reserved,
            elapsed,
            remaining_time: state.record.limits.time.saturating_sub(elapsed),
        })
    }

    /// Резерв списывается до передачи исполнителю и сохраняется даже при ошибке запуска.
    pub fn reserve(&self) -> Result<RunPermit<'_>, BudgetError> {
        self.reserve_for(1, 0, Duration::ZERO)
    }

    pub(crate) fn reserve_for(
        &self,
        runs: u32,
        keep_runs: u32,
        keep_time: Duration,
    ) -> Result<RunPermit<'_>, BudgetError> {
        let mut state = self.state()?;
        if self.time_left(&state)? <= keep_time {
            return Err(BudgetError::DeadlineExceeded);
        }
        if runs == 0 || runs.saturating_add(keep_runs) > state.record.limits.runs - state.record.reserved {
            return Err(BudgetError::QuotaExhausted);
        }
        state.record.reserved += runs;
        if let Some(checkpoint) = &self.checkpoint {
            let saved = (|| {
                let observed = unix_now()?;
                state.record.elapsed = self.elapsed_at(&state.record, observed)?;
                state.record.observed_unix = observed;
                checkpoint.save(&state.record)
            })();
            if saved.is_err() {
                state.write_failed = true;
            }
            saved?;
        }
        if self.time_left(&state)? <= keep_time {
            return Err(BudgetError::DeadlineExceeded);
        }
        Ok(RunPermit(self, keep_time))
    }

    /// Блокирует новые старты и ждёт завершения всех выданных в работу процессов.
    pub fn cancel_and_wait(&self) -> Result<(), BudgetError> {
        let mut state = self.state()?;
        state.cancelled = true;
        let state = self
            .idle
            .wait_while(state, |state| state.active != 0)
            .map_err(|_| checkpoint_error("нарушена целостность бюджета"))?;
        if state.cleanup_failed {
            return Err(checkpoint_error("ошибка завершения дочернего процесса"));
        }
        Ok(())
    }
}

/// Одноразовый допуск к одному физическому опыту. Для повторов нужен новый резерв.
pub struct RunPermit<'a>(&'a DepthBudget, Duration);

impl RunPermit<'_> {
    /// Передавать ввод и вывод файлами или наследуемыми дескрипторами.
    /// Код завершения команды возвращается как есть для проверки координатором.
    #[cfg(unix)]
    pub fn run(self, mut command: Command) -> Result<ExitStatus, BudgetError> {
        use std::os::unix::process::CommandExt;

        command.process_group(0);
        let budget = self.0;
        let time_left = |state: &State| {
            budget.time_left(state)?.checked_sub(self.1)
                .filter(|left| !left.is_zero())
                .ok_or(BudgetError::DeadlineExceeded)
        };
        let mut process = {
            let mut state = budget.state()?;
            time_left(&state)?;
            let child = command.spawn().map_err(BudgetError::Process)?;
            state.active += 1;
            ActiveChild {
                budget,
                child: Some(child),
                cleanup_failed: false,
            }
        };
        loop {
            let observation = (|| {
                let state = budget.state()?;
                let wait = time_left(&state)?;
                // Считываем статус под тем же замком, который сериализует отмену.
                let status = process
                    .child
                    .as_mut()
                    .unwrap()
                    .try_wait()
                    .map_err(BudgetError::Process)?;
                if status.is_some() {
                    process.child = None;
                    // Завершение, прочитанное после дедлайна, уже не принимается.
                    time_left(&state)?;
                }
                Ok((wait, status))
            })();
            match observation {
                Ok((_, Some(status))) => return Ok(status),
                Ok((wait, None)) => {
                    // Каденция опроса 5 мс, как в координаторе внешнего решателя openEMS.
                    std::thread::sleep(wait.min(Duration::from_millis(5)));
                }
                Err(error) => {
                    if let Err(cleanup) = process.stop() {
                        process.cleanup_failed = true;
                        return Err(BudgetError::Process(cleanup));
                    }
                    return Err(error);
                }
            }
        }
    }

    #[cfg(not(unix))]
    pub fn run(self, _command: Command) -> Result<ExitStatus, BudgetError> {
        let state = self.0.state()?;
        self.0.time_left(&state)?;
        Err(BudgetError::Process(io::Error::new(
            io::ErrorKind::Unsupported,
            "остановка группы процессов требует Unix",
        )))
    }
}

#[cfg(unix)]
struct ActiveChild<'a> {
    budget: &'a DepthBudget,
    child: Option<Child>,
    cleanup_failed: bool,
}

#[cfg(unix)]
impl ActiveChild<'_> {
    fn stop(&mut self) -> io::Result<()> {
        let Some(child) = &mut self.child else {
            return Ok(());
        };
        unsafe extern "C" {
            fn kill(pid: std::ffi::c_int, signal: std::ffi::c_int) -> std::ffi::c_int;
        }
        // SIGKILL = 9 и ESRCH = 3 в macOS/Linux. Отрицательный PID
        // адресует группу, созданную в spawn, пока её лидер ещё не забран через wait.
        let group_error =
            (unsafe { kill(-(child.id() as i32), 9) } != 0).then(io::Error::last_os_error);
        if group_error.is_some()
            && let Err(error) = child.kill()
            && child.try_wait()?.is_none()
        {
            return Err(error);
        }
        child.wait()?;
        self.child = None;
        match group_error {
            Some(error) if error.raw_os_error() != Some(3) => Err(error),
            _ => Ok(()),
        }
    }
}

#[cfg(unix)]
impl Drop for ActiveChild<'_> {
    fn drop(&mut self) {
        let result = self.stop();
        let mut state = self
            .budget
            .state
            .lock()
            .unwrap_or_else(|error| error.into_inner());
        state.cleanup_failed |= self.cleanup_failed || result.is_err();
        state.active -= 1;
        self.budget.idle.notify_all();
    }
}

struct Checkpoint {
    directory: PathBuf,
    lock: File,
}

impl Checkpoint {
    fn lock(directory: &Path) -> Result<Self, BudgetError> {
        // Отдельный файл держит блокировку и при атомарной замене записи.
        let lock = File::options()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(directory.join(LOCK))
            .map_err(checkpoint_error)?;
        lock.try_lock().map_err(|error| match error {
            TryLockError::WouldBlock => BudgetError::CheckpointBusy,
            TryLockError::Error(error) => checkpoint_error(error),
        })?;
        Ok(Self {
            directory: directory.to_owned(),
            lock,
        })
    }

    fn load(&self) -> Result<Record, BudgetError> {
        let file = File::open(self.directory.join(STATE)).map_err(checkpoint_error)?;
        let envelope: Envelope = serde_json::from_reader(file).map_err(checkpoint_error)?;
        if fnv1a(envelope.payload.bytes()) != envelope.checksum {
            return Err(checkpoint_error("контрольная сумма бюджета не совпала"));
        }
        serde_json::from_str(&envelope.payload).map_err(checkpoint_error)
    }

    fn save(&self, record: &Record) -> Result<(), BudgetError> {
        let payload = serde_json::to_string(record).map_err(checkpoint_error)?;
        let envelope = Envelope {
            checksum: fnv1a(payload.bytes()),
            payload,
        };
        let path = self.directory.join(PART);
        // После аварийного выхода незавершённая запись может остаться рядом с основной.
        remove_partial(&path).map_err(checkpoint_error)?;
        let result = (|| {
            let file = File::options().write(true).create_new(true).open(&path)?;
            serde_json::to_writer(&file, &envelope).map_err(io::Error::other)?;
            file.sync_all()?;
            fs::rename(&path, self.directory.join(STATE))?;
            #[cfg(unix)]
            File::open(&self.directory)?.sync_all()?;
            Ok::<(), io::Error>(())
        })();
        let cleanup = remove_partial(&path);
        result.and(cleanup).map_err(checkpoint_error)
    }
}

impl Drop for Checkpoint {
    fn drop(&mut self) {
        // Снимаем блокировку явно, в том числе при отказе чтения точки.
        // При наследовании дескриптора одного закрытия может быть недостаточно.
        let _ = self.lock.unlock();
    }
}

fn remove_partial(path: &Path) -> io::Result<()> {
    match fs::remove_file(path) {
        Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(()),
        result => result,
    }
}

fn unix_now() -> Result<Duration, BudgetError> {
    SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .map_err(checkpoint_error)
}

fn checkpoint_error(error: impl fmt::Display) -> BudgetError {
    BudgetError::Checkpoint(error.to_string())
}
