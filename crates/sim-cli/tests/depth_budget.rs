#![cfg(unix)]

#[path = "../src/tasks/workers.rs"]
#[allow(dead_code, reason = "в стенде очереди не выбираются по числу ядер")]
mod workers;

use std::fs;
use std::io::ErrorKind;
use std::num::NonZeroUsize;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::thread;
use std::time::{Duration, Instant};

use workers::Workers;
use workers::depth_budget::{BudgetError, DepthBudget, DepthLimits};

const PLAN: &str = "блок: сетки, разброс, отказ, повтор";

struct Temp(PathBuf);

impl Temp {
    fn new() -> Self {
        for slot in 0.. {
            let path =
                std::env::temp_dir().join(format!("depth-budget-{}-{slot}", std::process::id()));
            match fs::create_dir(&path) {
                Ok(()) => return Self(path),
                Err(error) if error.kind() == ErrorKind::AlreadyExists => {}
                Err(error) => panic!("не удалось создать каталог стенда: {error}"),
            }
        }
        unreachable!()
    }
}

impl Drop for Temp {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

fn shell(script: &str, path: &Path) -> Command {
    let mut command = Command::new("/bin/sh");
    command.args(["-c", script, "стенд"]).arg(path);
    command
}

fn assert_reaped(pid: &str) {
    let status = Command::new("/bin/kill")
        .args(["-0", pid.trim()])
        .stderr(Stdio::null())
        .status()
        .unwrap();
    assert!(!status.success(), "дочерний процесс ещё существует: {pid}");
}

#[test]
fn квота_на_сетки_вариант_отказ_и_повтор_общая() {
    // Две сетки + вариант + отказ + повтор = 5. При квоте 4 запас для
    // префикса из четырёх равен 0, для всего плана −1. При квоте 5 запас 0.
    let names = ["сетка-а", "сетка-б", "вариант", "отказ", "повтор"];
    for (quota, jobs) in [(4, 1), (4, 3), (5, 3)] {
        let temp = Temp::new();
        // Десять секунд дают стенду запас на запуск пяти простых команд.
        let limits = DepthLimits::new(quota, Duration::from_secs(10)).unwrap();
        let budget = DepthBudget::start(limits, PLAN, None).unwrap();
        let workers = Workers::new(NonZeroUsize::new(jobs).unwrap());
        let commands = names
            .iter()
            .map(|name| shell("printf 'опыт' > \"$1\"", &temp.0.join(name)))
            .collect();
        let rows = workers.map_budgeted(&budget, commands, None).unwrap();
        assert_eq!(rows.len(), names.len());
        for (index, row) in rows.into_iter().enumerate() {
            if index < quota as usize {
                assert!(row.unwrap().success());
            } else {
                assert!(matches!(row, Err(BudgetError::QuotaExhausted)));
            }
        }
        let mut files = fs::read_dir(&temp.0)
            .unwrap()
            .map(|entry| entry.unwrap().file_name().into_string().unwrap())
            .collect::<Vec<_>>();
        files.sort();
        let mut expected = names[..quota as usize].to_vec();
        expected.sort();
        assert_eq!(files, expected);
        assert_eq!(budget.usage().unwrap().reserved, quota);
        assert_eq!(budget.usage().unwrap().remaining_runs, 0);
    }
}

#[test]
fn продолжение_учитывает_резерв_до_запуска_и_не_меняет_план() {
    // Квота 2. Ошибка запуска расходует резерв, оставленный до передачи, поэтому запас 1.
    let temp = Temp::new();
    let limits = DepthLimits::new(2, Duration::from_secs(10)).unwrap();
    let first = DepthBudget::start(limits, PLAN, Some(&temp.0)).unwrap();
    assert!(matches!(
        DepthBudget::resume(limits, PLAN, &temp.0),
        Err(BudgetError::CheckpointBusy)
    ));
    let missing = Command::new(temp.0.join("нет-команды"));
    assert!(matches!(
        first.reserve().unwrap().run(missing),
        Err(BudgetError::Process(_))
    ));
    drop(first);
    let Err(BudgetError::Checkpoint(existing)) = DepthBudget::start(limits, PLAN, Some(&temp.0))
    else {
        panic!("создан новый бюджет поверх точки продолжения");
    };
    assert!(existing.contains("бюджет уже создан"));
    let Err(BudgetError::Checkpoint(plan)) = DepthBudget::resume(limits, "другой план", &temp.0)
    else {
        panic!("возобновлён другой план");
    };
    assert!(plan.contains("не соответствует плану или пределам"));
    let higher = DepthLimits::new(3, Duration::from_secs(10)).unwrap();
    let Err(BudgetError::Checkpoint(limits_changed)) = DepthBudget::resume(higher, PLAN, &temp.0)
    else {
        panic!("после возобновления увеличена квота");
    };
    assert!(limits_changed.contains("не соответствует плану или пределам"));
    let next = DepthBudget::resume(limits, PLAN, &temp.0).unwrap();
    assert_eq!(next.usage().unwrap().remaining_runs, 1);
    let marker = temp.0.join("повтор");
    assert!(
        next.reserve()
            .unwrap()
            .run(shell("printf 'повтор' > \"$1\"", &marker))
            .unwrap()
            .success()
    );
    assert_eq!(fs::read_to_string(marker).unwrap(), "повтор");
    assert!(matches!(next.reserve(), Err(BudgetError::QuotaExhausted)));
}

#[test]
fn дедлайн_действует_на_живой_процесс_и_после_возобновления() {
    // Процесс ждёт 30 с. При окне 2 с ручной запас −28 с.
    let temp = Temp::new();
    let pid = temp.0.join("pid");
    let limits = DepthLimits::new(2, Duration::from_secs(2)).unwrap();
    let budget = DepthBudget::start(limits, PLAN, Some(&temp.0)).unwrap();
    let result = budget.reserve().unwrap().run(shell(
        "printf '%s\\n' \"$$\" > \"$1\"; exec /bin/sleep 30",
        &pid,
    ));
    assert!(matches!(result, Err(BudgetError::DeadlineExceeded)));
    assert_reaped(&fs::read_to_string(&pid).unwrap());
    assert_eq!(budget.usage().unwrap().remaining_runs, 1);
    drop(budget);
    let resumed = DepthBudget::resume(limits, PLAN, &temp.0).unwrap();
    assert_eq!(resumed.usage().unwrap().remaining_time, Duration::ZERO);
    assert!(matches!(
        resumed.reserve(),
        Err(BudgetError::DeadlineExceeded)
    ));
}

#[test]
fn время_между_сеансами_списывается_даже_без_запусков() {
    let temp = Temp::new();
    let window = Duration::from_millis(100);
    let limits = DepthLimits::new(1, window).unwrap();
    drop(DepthBudget::start(limits, PLAN, Some(&temp.0)).unwrap());
    // Вне кампании ждём два окна. Израсходовано 0 запусков, запас по времени ≤ −100 мс.
    thread::sleep(window * 2);
    let resumed = DepthBudget::resume(limits, PLAN, &temp.0).unwrap();
    assert_eq!(resumed.usage().unwrap().reserved, 0);
    assert!(matches!(
        resumed.reserve(),
        Err(BudgetError::DeadlineExceeded)
    ));
}

#[test]
fn повреждённая_или_отсутствующая_точка_не_даёт_новую_квоту() {
    let temp = Temp::new();
    let limits = DepthLimits::new(2, Duration::from_secs(10)).unwrap();
    assert!(matches!(
        DepthBudget::resume(limits, PLAN, &temp.0),
        Err(BudgetError::Checkpoint(_))
    ));
    let budget = DepthBudget::start(limits, PLAN, Some(&temp.0)).unwrap();
    let _ = budget.reserve().unwrap();
    drop(budget);
    let path = temp.0.join("depth-budget-v1.json");
    let mut envelope: serde_json::Value =
        serde_json::from_slice(&fs::read(&path).unwrap()).unwrap();
    // Полезная нагрузка остаётся корректным JSON, но вернула уже списанный запуск.
    let payload = envelope["payload"].as_str().unwrap();
    let changed = payload.replace("\"reserved\":1", "\"reserved\":0");
    assert_ne!(changed, payload);
    envelope["payload"] = changed.into();
    fs::write(path, serde_json::to_vec(&envelope).unwrap()).unwrap();
    let Err(BudgetError::Checkpoint(reason)) = DepthBudget::resume(limits, PLAN, &temp.0) else {
        panic!("повреждённая точка принята");
    };
    assert!(reason.contains("контрольная сумма"));
}

#[test]
fn отмена_дожидается_дочернего_процесса_и_запрещает_новый() {
    let temp = Temp::new();
    let pid = temp.0.join("pid");
    let limits = DepthLimits::new(2, Duration::from_secs(10)).unwrap();
    let budget = DepthBudget::start(limits, PLAN, None).unwrap();
    let permit = budget.reserve().unwrap();
    let pending = budget.reserve().unwrap();
    thread::scope(|scope| {
        let handle = scope.spawn(|| {
            permit.run(shell(
                "printf '%s\\n' \"$$\" > \"$1\"; exec /bin/sleep 30",
                &pid,
            ))
        });
        // Отменяем после независимого подтверждения запуска и до дедлайна.
        let started = Instant::now();
        let pid = loop {
            if let Ok(value) = fs::read_to_string(&pid)
                && value.ends_with('\n')
            {
                break value;
            }
            assert!(
                started.elapsed() < Duration::from_secs(5),
                "процесс не отметился"
            );
            thread::yield_now();
        };
        budget.cancel_and_wait().unwrap();
        assert_reaped(&pid);
        assert!(matches!(
            handle.join().unwrap(),
            Err(BudgetError::Cancelled)
        ));
    });
    let marker = temp.0.join("лишний-прогон");
    assert!(matches!(
        pending.run(shell("printf 'лишний' > \"$1\"", &marker)),
        Err(BudgetError::Cancelled)
    ));
    assert!(!marker.exists());
}

#[test]
fn отмена_останавливает_всю_группу_до_отложенной_записи() {
    let temp = Temp::new();
    let ready = temp.0.join("готовность");
    let late = temp.0.join("поздняя-запись");
    let limit = DepthLimits::new(1, Duration::from_secs(10)).unwrap();
    let budget = DepthBudget::start(limit, PLAN, None).unwrap();
    let permit = budget.reserve().unwrap();
    let mut command = shell(
        "(printf 'готов' > \"$1\"; /bin/sleep 1; printf 'поздно' > \"$2\") & wait",
        &ready,
    );
    command.arg(&late);
    thread::scope(|scope| {
        let handle = scope.spawn(|| permit.run(command));
        let started = Instant::now();
        while fs::read_to_string(&ready).as_deref().ok() != Some("готов") {
            assert!(
                started.elapsed() < Duration::from_secs(5),
                "группа не отметилась"
            );
            thread::yield_now();
        }
        budget.cancel_and_wait().unwrap();
        assert!(matches!(
            handle.join().unwrap(),
            Err(BudgetError::Cancelled)
        ));
    });
    // Ждём дольше задержки внутри группы. При остановке только лидера потомок запишет файл.
    thread::sleep(Duration::from_millis(1200));
    assert!(!late.exists());
}

#[test]
fn потолок_тз_и_нулевые_границы() {
    // Пределы взяты из блока Е. На один запуск и на 1 нс выше уже недопустимо.
    let window = Duration::from_secs(30 * 60);
    assert!(DepthLimits::new(1000, window).is_ok());
    assert!(DepthLimits::new(1001, window).is_err());
    assert!(DepthLimits::new(1000, window + Duration::from_nanos(1)).is_err());
    let quota = DepthBudget::start(DepthLimits::new(0, window).unwrap(), PLAN, None).unwrap();
    assert!(matches!(quota.reserve(), Err(BudgetError::QuotaExhausted)));
    let time =
        DepthBudget::start(DepthLimits::new(1, Duration::ZERO).unwrap(), PLAN, None).unwrap();
    assert!(matches!(time.reserve(), Err(BudgetError::DeadlineExceeded)));
}
