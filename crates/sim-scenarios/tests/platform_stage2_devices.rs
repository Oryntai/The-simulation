//! Приёмка открытого контура. Миры, детали и команды загружаются с диска.

use std::ffi::OsString;
use std::fs::{self, File};
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};

use sim_runtime::{CommandSource, Log, LogEntry, LogKind, Runtime};
use sim_signal::{ImageFrame, Observation, PixelFormat, Time};

const REFERENCE_HZ: u64 = 1_000_000;
const OFF: u64 = 240_000_000; // Физический момент снятия питания по сценарию, нс.
const END: u64 = 260_000_000; // Конец сценария, нс.
const IMPACT_WINDOW: u64 = 80_000_000; // Холостой ход; до первого касания в 90–110 мс.
const NS_PER_S: u64 = 1_000_000_000;
const GRID_TOLERANCE: f64 = 0.01;
const WINDOWS: [(u64, u64); 3] = [(0, IMPACT_WINDOW), (IMPACT_WINDOW, OFF), (OFF, END)];

fn exact_tick(hz: u64, at: u64) -> Option<u64> {
    let index = (at * hz).div_ceil(NS_PER_S);
    (sim_signal::Rate::hz(hz).unwrap().tick(index) == Time(at)).then_some(index)
}

fn configure_grid(stand: &Stand, hz: u64) -> PathBuf {
    let world = stand.copy_world();
    let device = stand.scratch.join("device.toml");
    for (path, field, value) in [
        (&world, "physics_hz", hz.to_string()),
        (&device, "control_hz", format!("{hz}.0")),
    ] {
        let data = fs::read_to_string(path).unwrap();
        let line = data.lines().find(|line| line.starts_with(field)).unwrap();
        fs::write(path, data.replace(line, &format!("{field} = {value}"))).unwrap();
    }
    world
}

#[derive(Clone, Copy)]
struct DriveTick {
    at: u64,
    angle: f64,
    speed: f64,
    torque: f64,
    current: f64,
    voltage: f64,
    electrical: f64,
    work: f64,
}

struct DriveTrace {
    ticks: Vec<DriveTick>,
    cutoff: u64,
}

impl DriveTrace {
    fn from_log(log: &Log, hz: u64, cutoff: u64) -> Self {
        let names = [
            "qpos[0]",
            "qvel[0]",
            "нажиматель/мотор/joint_torque_nm",
            "нажиматель/мотор/current_a",
            "нажиматель/мотор/voltage_v",
        ];
        let ids = names.map(|name| signal(log, LogKind::Truth, name));
        let rate = sim_signal::Rate::hz(hz).unwrap();
        let mut ticks: Vec<DriveTick> = Vec::new();
        for entries in log.entries().chunk_by(|a, b| a.at == b.at) {
            let at = entries[0].at.0;
            let mut values = [None; 5];
            for entry in entries.iter().filter(|entry| entry.kind == LogKind::Truth) {
                if let Some(index) = ids.iter().position(|id| *id == entry.id) {
                    assert!(entry.valid && entry.value.is_finite());
                    values[index] = Some(entry.value);
                }
            }
            if values.iter().all(Option::is_none) {
                continue; // Между тактами физики могут быть команды и показания датчиков.
            }
            assert_eq!(at, rate.tick(ticks.len() as u64).0);
            let [angle, speed, torque, current, voltage] = values.map(Option::unwrap);
            let mut tick = DriveTick {
                at,
                angle,
                speed,
                torque,
                current,
                voltage,
                electrical: 0.0,
                work: 0.0,
            };
            if let Some(prev) = ticks.last() {
                let left = prev.voltage * prev.current;
                // Скачок при размыкании имеет правый отсчёт. Последний
                // питавшийся интервал интегрируем до его левой границы.
                let right = if at == cutoff {
                    left
                } else {
                    voltage * current
                };
                tick.electrical =
                    prev.electrical + 0.5 * (left + right) * Time(at - prev.at).as_secs();
                tick.work = prev.work + prev.torque * (angle - prev.angle);
                if at < cutoff {
                    assert!(voltage > 0.0);
                } else {
                    assert_eq!((voltage, current), (0.0, 0.0));
                }
            }
            ticks.push(tick);
        }
        assert_eq!(ticks.len() as u64, exact_tick(hz, END).unwrap() + 1);
        assert!(exact_tick(hz, cutoff).is_some());
        Self { ticks, cutoff }
    }

    fn run(stand: &Stand, hz: u64, delay_one_tick: bool) -> Self {
        // 240 мс кратны вещественному шагу, независимо от округления шкалы.
        assert_eq!(OFF * hz % NS_PER_S, 0);
        let rate = sim_signal::Rate::hz(hz).unwrap();
        let release = exact_tick(hz, OFF).unwrap();
        let input = release - u64::from(!delay_one_tick);
        let world = configure_grid(stand, hz);
        let command = stand.scratch.join("grid.csv");
        let original = fs::read_to_string(stand.data.join("commands.csv")).unwrap();
        let lines: Vec<_> = original.lines().collect();
        let [header, power_on, pwm, power_off] = lines.as_slice() else {
            panic!("ожидалось одно отключение питания");
        };
        let (_, channel_and_value) = power_off.split_once(',').unwrap();
        fs::write(
            &command,
            format!(
                "{header}\n{power_on}\n{pwm}\n0.{:09},{channel_and_value}\n",
                rate.tick(input).0
            ),
        )
        .unwrap();
        let path = stand.scratch.join("grid.bin");
        stand.run(
            &world,
            &[
                ("--catalog", &stand.catalog),
                ("--commands", &command),
                ("--log", &path),
            ],
            0,
        );
        let log = Log::read_binary(File::open(&path).unwrap()).unwrap();
        fs::remove_file(path).unwrap();
        Self::from_log(&log, hz, rate.tick(input + 1).0)
    }

    fn at(&self, at: u64) -> DriveTick {
        let index = self.ticks.partition_point(|tick| tick.at <= at) - 1;
        let old = self.ticks[index];
        if old.at == at {
            return old;
        }
        let next = self.ticks[index + 1];
        let alpha = (at - old.at) as f64 / (next.at - old.at) as f64;
        let lerp = |a: f64, b: f64| (b - a).mul_add(alpha, a);
        DriveTick {
            at,
            angle: lerp(old.angle, next.angle),
            speed: lerp(old.speed, next.speed),
            // Усилие удерживается весь шаг. Ток не интерполируется через
            // размыкание: в журнале на границе уже записан правый предел.
            torque: old.torque,
            current: if next.at == self.cutoff {
                old.current
            } else {
                lerp(old.current, next.current)
            },
            voltage: old.voltage,
            electrical: lerp(old.electrical, next.electrical),
            work: lerp(old.work, next.work),
        }
    }
}

// Максимум ошибки по объединению тактов двух сеток, включая каждую
// границу интервала удержания. Энергия — худшее из двух независимых
// интегралов по журналу: накопленная `UI dt` и работа `τ dθ` от начала окна.
fn grid_error(reference: &DriveTrace, trial: &DriveTrace, start: u64, end: u64) -> ([f64; 5], f64) {
    let values = |tick: DriveTick, origin: DriveTick| {
        [
            tick.angle,
            tick.speed,
            tick.torque,
            tick.current,
            tick.electrical,
            tick.work - origin.work,
        ]
    };
    let mut scale = [0.0_f64; 6];
    let mut max_error = [0.0_f64; 6];
    let mut angle_extremes = [f64::INFINITY, f64::NEG_INFINITY];
    let mut reference_torque = [0.0_f64; 2];
    let mut trial_torque = [0.0_f64; 2];
    let reference_origin = reference.at(start);
    let trial_origin = trial.at(start);
    for at in reference
        .ticks
        .iter()
        .chain(&trial.ticks)
        .map(|tick| tick.at)
        .filter(|at| (start..end).contains(at))
        .chain([end])
    {
        let a = values(reference.at(at), reference_origin);
        let b = values(trial.at(at), trial_origin);
        angle_extremes[0] = angle_extremes[0].min(a[0]);
        angle_extremes[1] = angle_extremes[1].max(a[0]);
        for channel in 0..6 {
            if channel == 2 && at == end {
                continue; // Этот момент относится к следующему интервалу.
            }
            scale[channel] = scale[channel].max(a[channel].abs());
            max_error[channel] = max_error[channel].max((a[channel] - b[channel]).abs());
        }
        if at < end {
            reference_torque = [reference_torque[0].min(a[2]), reference_torque[1].max(a[2])];
            trial_torque = [trial_torque[0].min(b[2]), trial_torque[1].max(b[2])];
        }
    }
    scale[0] = angle_extremes[1] - angle_extremes[0];
    if start >= reference.cutoff {
        // При нулевом эталонном токе используем последний питавшийся отсчёт.
        let off = reference
            .ticks
            .partition_point(|tick| tick.at < reference.cutoff);
        scale[3] = reference.ticks[off - 1].current.abs();
    }
    let ratio = |error: f64, scale: f64| {
        assert!(error.is_finite() && scale.is_finite() && scale > 0.0);
        error / scale
    };
    let [angle, speed, torque, current, input, work] =
        std::array::from_fn(|i| ratio(max_error[i], scale[i]));
    let peak_error = (reference_torque[0] - trial_torque[0])
        .abs()
        .max((reference_torque[1] - trial_torque[1]).abs());
    (
        [angle, speed, torque, current, input.max(work)],
        ratio(peak_error, scale[2]),
    )
}

fn feasible_grids(reference: &DriveTrace) -> Vec<u64> {
    // 240 мс = 6/25 секунды. Только кратные 25 Гц имеют точный такт
    // в момент отключения. Если эталон меняется скачком более двух
    // допусков, сетка обязана иметь границу в этот момент: одним
    // постоянным моментом обе стороны скачка аппроксимировать нельзя.
    let mut feasible: Vec<_> = (25..=REFERENCE_HZ).step_by(25).collect();
    for (start, end) in [(IMPACT_WINDOW, OFF), (OFF, END)] {
        let scale = reference
            .ticks
            .iter()
            .filter(|tick| (start..end).contains(&tick.at))
            .map(|tick| tick.torque.abs())
            .fold(0.0_f64, f64::max);
        for ticks in reference.ticks.windows(2) {
            let [left, right] = [ticks[0], ticks[1]];
            if left.at >= start
                && right.at < end
                && (right.torque - left.torque).abs() > 2.0 * GRID_TOLERANCE * scale
            {
                feasible.retain(|&hz| exact_tick(hz, right.at).is_some());
            }
        }
    }
    feasible
}

fn check_drive_grids(stand: &Stand, log: &Log) {
    let reference = DriveTrace::from_log(log, REFERENCE_HZ, OFF);
    assert_eq!(feasible_grids(&reference), [REFERENCE_HZ]);

    let coarse = DriveTrace::run(stand, 20_000, false);
    let coarse_late = DriveTrace::run(stand, 20_000, true);
    let reference_late = DriveTrace::run(stand, REFERENCE_HZ, true);
    let fine = DriveTrace::run(stand, 2 * REFERENCE_HZ, false);
    let [aligned, unaligned, timing, _, refined] = [
        ("20 кГц, синхронно", &reference, &coarse),
        (
            "20 кГц и 1 МГц с командой в 240 мс",
            &reference_late,
            &coarse_late,
        ),
        (
            "1 МГц, только задержка команды",
            &reference,
            &reference_late,
        ),
        ("20 кГц, только задержка команды", &coarse, &coarse_late),
        ("1 МГц против 2 МГц, синхронно", &fine, &reference),
    ].map(|(label, expected, actual)| {
        WINDOWS.map(|(start, end)| {
            let (error, peak) = grid_error(expected, actual, start, end);
            println!(
                "{label}, {start}–{end} нс: угол, скорость, момент, ток, энергия {:.5?} %, пик момента {:.5} %",
                error.map(|x| 100.0 * x),
                100.0 * peak
            );
            (error, peak)
        })
    });
    let (aligned, _) = aligned[2];
    let (unaligned, _) = unaligned[2];
    assert!(aligned[1] > GRID_TOLERANCE && aligned[1] < unaligned[1]);
    assert!(aligned[2] > GRID_TOLERANCE);
    let (timing, _) = timing[2];
    assert!(timing[3] > GRID_TOLERANCE);
    // Контроль на более мелкой сетке. Пики отдельно, вся кривая момента
    // сообщается выше: у жёсткого выбора люфта она ещё не сходится до 1%.
    for ([angle, speed, _, current, energy], peak) in refined {
        assert!(
            [angle, speed, current, energy, peak]
                .into_iter()
                .all(|e| e <= GRID_TOLERANCE)
        );
    }
}

struct Stand {
    data: PathBuf,
    catalog: PathBuf,
    scratch: PathBuf,
    duration: &'static str,
}

impl Stand {
    fn load(directory: &str, duration: &'static str) -> Self {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let scratch = std::env::temp_dir().join(format!(
            "sim-stage2-test-{}-{directory}-{duration}",
            std::process::id()
        ));
        fs::create_dir(&scratch).unwrap();
        Self {
            data: root.join("devices").join(directory),
            catalog: root.join("catalog"),
            scratch,
            duration,
        }
    }

    fn run(&self, world: &Path, flags: &[(&str, &Path)], code: u8) -> String {
        self.run_for(world, self.duration, flags, code)
    }

    fn run_for(&self, world: &Path, duration: &str, flags: &[(&str, &Path)], code: u8) -> String {
        let mut args = vec![
            world.as_os_str().to_owned(),
            OsString::from("--duration"),
            duration.into(),
            OsString::from("--json"),
        ];
        for &(flag, path) in flags {
            args.extend([flag.into(), path.as_os_str().to_owned()]);
        }
        let mut output = Vec::new();
        let mut errors = Vec::new();
        let actual = sim_cli::run(args, &[][..], &mut output, &mut errors);
        assert_eq!(
            actual,
            code,
            "{} {}",
            String::from_utf8_lossy(&output),
            String::from_utf8_lossy(&errors),
        );
        String::from_utf8(output).unwrap()
    }

    fn deterministic_log(&self) -> (Log, String) {
        let world = self.data.join("world.toml");
        let commands = self.data.join("commands.csv");
        let first = self.scratch.join("first.bin");
        let second = self.scratch.join("second.bin");
        let replay = self.scratch.join("replay.bin");
        let original = self.run(&world, &[("--commands", &commands), ("--log", &first)], 0);
        let again = self.run(
            &world,
            &[
                ("--catalog", &self.catalog),
                ("--commands", &commands),
                ("--log", &second),
            ],
            0,
        );
        let played = self.run(&world, &[("--replay", &first), ("--log", &replay)], 0);
        assert_eq!(original, again);
        assert_eq!(original, played);
        same_bytes(&first, &second);
        same_bytes(&first, &replay);
        assert!(original.contains(r#""status":"completed""#));
        assert!(original.contains(r#""blocks_proof":true,"proof_status":"not_analyzed""#));
        assert!(original.contains("значение экземпляра в params"));
        (
            Log::read_binary(File::open(first).unwrap()).unwrap(),
            original,
        )
    }

    fn copy_world(&self) -> PathBuf {
        for entry in fs::read_dir(&self.data).unwrap() {
            let entry = entry.unwrap();
            if entry.path().extension().is_some_and(|ext| ext == "toml") {
                fs::copy(entry.path(), self.scratch.join(entry.file_name())).unwrap();
            }
        }
        self.scratch.join("world.toml")
    }

    fn invalid_parameters(&self) {
        let world = self.copy_world();
        let device = self.scratch.join("device.toml");
        let original = fs::read_to_string(&device).unwrap();
        let inductance = original
            .lines()
            .find(|line| line.starts_with("inductance_h ="))
            .unwrap();
        for (replacement, detail) in [
            ("inductance_h = nan", "inductance_h"),
            ("inductance_h = inf", "inductance_h"),
            (
                "bad_inductance_h = 1.0",
                "неизвестный параметр «bad_inductance_h»",
            ),
        ] {
            fs::write(&device, original.replace(inductance, replacement)).unwrap();
            let report = self.run_for(&world, "0", &[("--catalog", &self.catalog)], 2);
            assert!(report.contains(r#""status":"error""#), "{report}");
            assert!(report.contains(r#""blocks_proof":true"#), "{report}");
            assert!(report.contains(detail), "{report}");
        }
        fs::write(&device, original).unwrap();
    }
}

impl Drop for Stand {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.scratch);
    }
}

fn same_bytes(first: &Path, second: &Path) {
    let mut a = BufReader::new(File::open(first).unwrap());
    let mut b = BufReader::new(File::open(second).unwrap());
    loop {
        let x = a.fill_buf().unwrap();
        let y = b.fill_buf().unwrap();
        assert!(x == y, "разные байты в журналах");
        if x.is_empty() {
            break;
        }
        let len = x.len();
        a.consume(len);
        b.consume(len);
    }
}

fn time(seconds: f64) -> Time {
    Time::from_secs(seconds).unwrap()
}

fn signal(log: &Log, kind: LogKind, name: &str) -> u32 {
    let table = match kind {
        LogKind::Truth => &log.header().truth,
        LogKind::Observation => &log.header().observations,
        LogKind::Command(_) => &log.header().commands,
    };
    table.iter().position(|spec| spec.name == name).unwrap() as u32
}

fn sample<'a>(log: &'a Log, kind: LogKind, name: &str, at: Time) -> &'a LogEntry {
    let id = signal(log, kind, name);
    let entries = log.entries();
    let start = entries.partition_point(|entry| entry.at < at);
    entries[start..]
        .iter()
        .take_while(|entry| entry.at == at)
        .find(|entry| entry.kind == kind && entry.id == id)
        .unwrap_or_else(|| panic!("нет отсчёта {name} в {at:?}"))
}

fn truth(log: &Log, name: &str, seconds: f64) -> f64 {
    sample(log, LogKind::Truth, name, time(seconds)).value
}

fn near(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "получено {actual:.9}, ожидалось {expected:.9} ± {tolerance:.9}"
    );
}

#[test]
fn stage2_drive_matches_endpoints_and_releases_power() {
    let stand = Stand::load("04-wall-switch", "0.26");
    let (log, report) = stand.deterministic_log();
    let motor = |field: &str, at| truth(&log, &format!("нажиматель/мотор/{field}"), at);
    // 1.8 кгс·см при 4.8 В по TowerPro. 9.80665 м/с² — нормальное g;
    // 0.12 с / 60° — принятая точка catalog/drives/mg90s.toml.
    let nominal_torque = 1.8 * 9.80665 / 100.0;
    let catalog_speed = 60_f64.to_radians() / 0.12;
    let tower_speed = 60_f64.to_radians() / 0.10;
    let nominal_current = 0.7;
    let voltage = 4.8;
    // Допуск сверки реализации 2%. Для расходящихся публикаций по скорости
    // контрольный интервал ±20% от TowerPro включает вариант каталога (−16.7%).
    let tolerance = 0.02;
    let unloaded = truth(&log, "qvel[0]", 0.08);
    near(unloaded, catalog_speed, tolerance * catalog_speed);
    near(unloaded, tower_speed, 0.20 * tower_speed);
    assert_eq!(truth(&log, "qpos[1]", 0.09), 0.0);
    assert_eq!(motor("joint_torque_nm", 0.08), 0.0);

    // Первые микросекунды: люфт ещё не выбран. Аналитический R–L переход без ЭДС,
    // tau = 0.0711 мГн / 20.8 Ом по обмотке-аналогу. При 1 мкс скорость ротора ещё мала.
    let tau = 0.0711e-3 / 20.8;
    let current_early = nominal_current * (1.0 - (-1.0e-6_f64 / tau).exp());
    near(
        motor("current_a", 0.000001),
        current_early,
        current_early * tolerance,
    );
    assert_eq!(truth(&log, "qpos[0]", 0.000011), 0.0);
    assert!(motor("motor_rad", 0.000011) > 0.0);

    // Жёсткость упора в данных соответствует 1° статического прогиба при Tstall.
    // Из координаты независимо находим реакцию, и восстанавливаем момент передачи
    // из реакции и зарегистрированного результирующего момента на оси.
    let spring = nominal_torque / 1_f64.to_radians();
    let limit = 60_f64.to_radians();
    let stopped = truth(&log, "qpos[0]", 0.23);
    let stop_reaction = spring * (stopped - limit);
    let torque = stop_reaction + motor("joint_torque_nm", 0.23);
    near(torque, nominal_torque, tolerance * nominal_torque);
    assert!(stopped > limit);
    near(stopped.to_degrees(), 61.0, 0.02);
    // Порог почти неподвижного выхода: менее 1% скорости без нагрузки.
    assert!(truth(&log, "qvel[0]", 0.23).abs() < 0.01 * catalog_speed);
    near(
        motor("current_a", 0.23),
        nominal_current,
        tolerance * nominal_current,
    );
    assert_eq!(motor("voltage_v", 0.23), voltage);
    // В первые 90 мс без контакта клавиша неподвижна. Положительное движение
    // после контакта проверяет механику. Электрическое переключение на этапе 4.
    assert!(truth(&log, "qpos[1]", 0.11) > 10_f64.to_radians());
    assert!(truth(&log, "qpos[0]", 0.26) < limit);

    let currents = signal(&log, LogKind::Truth, "нажиматель/мотор/current_a");
    let voltages = signal(&log, LogKind::Truth, "нажиматель/мотор/voltage_v");
    let stable = time(0.18)..=time(0.24);
    let entries: Vec<_> = log
        .entries()
        .iter()
        .filter(|entry| entry.kind == LogKind::Truth && stable.contains(&entry.at))
        .filter(|entry| entry.id == currents || entry.id == voltages)
        .collect();
    assert_eq!(entries.len(), 2 * (60_000 + 1)); // 60 мс при шаге 1 мкс, обе границы.
    let (pairs, []) = entries.as_chunks::<2>() else {
        panic!("у отсчёта тока нет пары напряжения");
    };
    let power = |[i, u]: [&LogEntry; 2]| {
        assert!(i.id == currents && u.id == voltages && i.at == u.at);
        assert_eq!(u.value, voltage);
        i.value * u.value
    };
    let mut energy = 0.0;
    for window in pairs.windows(2) {
        let [old, new] = window else { unreachable!() };
        let tick = Time(new[0].at.0 - old[0].at.0);
        // На границе отключения в журнале уже правый отсчёт тока.
        let right = if new[0].at == Time(OFF) {
            power(*old)
        } else {
            power(*new)
        };
        energy += (power(*old) + right) * 0.5 * tick.as_secs();
    }
    let stall_energy = voltage * nominal_current * 0.06;
    near(energy, stall_energy, tolerance * stall_energy);
    let residual = motor("world_energy_residual_j", 0.26);
    assert!(residual.abs() < 0.001 * stall_energy); // До 0.1% от проверенного окна энергии.

    assert_eq!(motor("voltage_v", 0.239999), voltage); // CSV после событий такта.
    assert_eq!(motor("voltage_v", 0.24), 0.0);
    assert_eq!(motor("current_a", 0.24), 0.0);
    let sensed = |at| sample(&log, LogKind::Observation, "нажиматель/ток/value", time(at));
    assert!(sensed(0.241).valid && sensed(0.241).value > 0.0);
    assert!(sensed(0.242).valid);
    assert_eq!(sensed(0.242).value, 0.0);
    assert!(
        report.contains(r#""нажиматель/ток/value":{"value":0.0,"time_ns":260000000,"valid":true}"#)
    );
    assert!(report.contains(
        r#""нажиматель/скважность-power":{"value":0.0,"time_ns":239999000,"valid":true}"#
    ));
    assert!(report.contains(r#""component_failure":null"#));
    assert!(report.contains("команда CSV на границе такта видна только на следующем"));
    println!(
        "04: холостой ход {unloaded:.8} рад/с, stall {torque:.8} Н·м, ток {:.8} А, упор {:.6}°, энергия {energy:.8} Дж, невязка {residual:.8} Дж",
        motor("current_a", 0.23),
        stopped.to_degrees()
    );
    stand.invalid_parameters();
    check_drive_grids(&stand, &log);
}

// Независимая проекция передней грани: R_y(-угол), деление на глубину,
// затем центр площади трапеции с двумя вертикальными рёбрами.
fn projection(yaw: f64, travel: f64) -> (f64, f64) {
    let (s, c) = yaw.sin_cos();
    // Эскиз стенда: X = 0.24 м + ход, Z = 1.20 м, размеры 0.24×0.18×0.001 м,
    // оптика reference-rgb: fx=fy=90 px, cx=48 px.
    let z = 1.20 - 0.001 / 2.0;
    let edge = |x: f64| {
        let depth = s * x + c * z;
        (48.0 + 90.0 * (c * x - s * z) / depth, 90.0 * 0.18 / depth)
    };
    let (left, hl) = edge(0.24 + travel - 0.12);
    let (right, hr) = edge(0.24 + travel + 0.12);
    assert!(0.0 < left && left < right && right < 96.0);
    let center = left + (right - left) * (hl + 2.0 * hr) / (3.0 * (hl + hr));
    (center, (right - left) * (hl + hr) / 2.0)
}

#[test]
fn stage2_optics_projects_moving_target_at_exposure_times() {
    let stand = Stand::load("05-camera-tracker", "0.184");
    let (log, _) = stand.deterministic_log();
    // Перемещение каретки независимо от камеры: a=F/m=0.08/0.02=4 м/с².
    // Для схемы MuJoCo Euler x(n)=a·h² n(n+1)/2, h=4 мкс. После 160 мс v постоянна.
    let step = 1.0 / 250_000.0;
    let travel = |at: f64| {
        let powered = at.min(0.16);
        2.0 * powered * (powered + step) + 4.0 * 0.16 * (at - powered)
    };
    let position = |at| truth(&log, "qpos[0]", at);
    assert_eq!(position(0.025004), 0.0);
    near(
        truth(&log, "подвес/мотор/voltage_v", 0.025008),
        6.0 * 0.18,
        f64::EPSILON,
    );
    assert!(position(0.12) > 20_f64.to_radians());
    assert!(position(0.184) > position(0.12));
    near(truth(&log, "qpos[1]", 0.184), travel(0.184), 1e-12);
    let command = sample(
        &log,
        LogKind::Command(CommandSource::External),
        "подвес/скважность",
        time(0.025001),
    );
    assert_eq!(command.value, 0.18);
    let encoder = sample(&log, LogKind::Observation, "подвес/угол/value", time(0.184));
    assert!(encoder.valid);
    let quantum = std::f64::consts::TAU / 4096.0;
    let measured = encoder.value;
    near(
        measured,
        (position(0.182) / quantum).round() * quantum,
        f64::EPSILON,
    );
    assert_ne!(measured.to_bits(), position(0.184).to_bits());

    let world = sim_world::load_and_compile(&stand.data.join("world.toml")).unwrap();
    let mut runtime = Runtime::load(world.clone()).unwrap();
    let drives = sim_drive::DriveCatalog::load(stand.catalog.join("drives")).unwrap();
    let sensors = sim_sense::SensorCatalog::load(stand.catalog.join("sensors")).unwrap();
    sim_drive::install(&mut runtime, &world, &drives).unwrap();
    sim_sense::install(&mut runtime, &world, &sensors).unwrap();
    runtime.record_joint_state("запись состояния").unwrap();
    runtime.replay(log.clone()).unwrap();
    let image = runtime.observation_id("подвес/кадр/image").unwrap();
    runtime.run_until(Time(time(0.024).0 - 1)).unwrap();
    assert!(runtime.observe().frame(image).is_none());
    runtime.run_until(time(0.024)).unwrap();
    assert_eq!(runtime.observe().frame(image).unwrap().captured, Time::ZERO);
    // Кадр с 120 мс недоступен до окончания 16 мс экспозиции и 8 мс задержки.
    runtime.run_until(Time(time(0.144).0 - 1)).unwrap();
    assert_eq!(runtime.observe().frame(image).unwrap().captured, time(0.08));
    runtime.run_until(time(0.144)).unwrap();
    // Контроллеру доступны только измерения. Без `..` проверяется состав типов,
    // в которых проходят кадр и показания: глубины, маски объектов и поз здесь нет.
    let Observation {
        time: now,
        samples,
        frames,
    } = runtime.observe();
    assert_eq!(now, time(0.144));
    assert!(!samples.is_empty());
    assert_eq!(frames.len(), 1);
    let frame = &frames[0];
    let ImageFrame {
        signal,
        captured,
        at,
        width,
        height,
        format,
        data,
        valid,
    } = frame;
    assert_eq!((*signal, *captured, *at), (image, time(0.12), time(0.144)));
    assert_eq!(
        (*width, *height, *format, *valid),
        (96, 72, PixelFormat::Rgb8, true)
    );
    assert_eq!(
        sample(&log, LogKind::Observation, "подвес/кадр/image", *at).value,
        frame.fingerprint()
    );

    let mut sum = 0.0;
    let mut centroid = [0.0; 2];
    for (index, [r, g, b]) in data.as_chunks::<3>().0.iter().copied().enumerate() {
        let weight = f64::from(r.saturating_sub(g.max(b)));
        sum += weight;
        centroid[0] += weight * ((index % *width as usize) as f64 + 0.5);
        centroid[1] += weight * ((index / *width as usize) as f64 + 0.5);
    }
    assert!(sum > 0.0);
    let [x, y] = centroid.map(|coord| coord / sum);
    let mut area = 0.0;
    let mut moment = 0.0;
    // Середины четырёх окон экспозиции по 4 мс, заданных в каталоге.
    for at in [0.122, 0.126, 0.130, 0.134] {
        near(truth(&log, "qpos[1]", at), travel(at), 1e-12);
        let (center, weight) = projection(position(at), travel(at));
        area += weight;
        moment += center * weight;
    }
    let expected = moment / area;
    near(x, expected, 0.1); // Допуск ТЗ на проекцию, в пикселях.
    near(y, 36.0, 0.1);
    // Стоп-кадр на начале экспозиции обязан существенно разойтись с RGB.
    assert!((x - projection(position(0.12), travel(0.12)).0).abs() > 1.0);
    runtime.run_until(time(0.184)).unwrap();
    runtime.verify_replay().unwrap();
    println!(
        "05: угол {:.6}°, ход {:.8} м, центр ({x:.5}, {y:.5}) px, независимая проекция ({expected:.5}, 36) px, кадр 120 → 144 мс",
        position(0.184).to_degrees(),
        travel(0.184)
    );
    stand.invalid_parameters();
}

#[test]
fn stage2_coarser_physics_reports_component_failure() {
    let stand = Stand::load("04-wall-switch", "0");
    // Независимая подстановка из device.toml: J корпуса не входит в ось.
    // J рычага — параллелепипед, ось проходит на его торце.
    let length = 0.033_f64;
    let width = 0.007_f64;
    let inertia = 1270.0 * length * width * 0.006 * (length.powi(2) / 3.0 + width.powi(2) / 12.0);
    let rotor = 1.79e-9 * 158.976_f64.powi(2) * 0.4943109384284195;
    let inverse = 1.0 / inertia + 1.0 / rotor;
    let stiffness = 20.227667621830875 * inverse + 10.113833810915438 / inertia;
    let damping = 0.0050404381223863775 * inverse;
    // Относительная ошибка фазы и абсолютная ошибка множителя затухания
    // для средней точки ограничены (h√K/J)²/12 и (h·c/J)³/12.
    let budget = 12.0 * GRID_TOLERANCE;
    let required = (stiffness / budget)
        .sqrt()
        .max(damping / budget.cbrt())
        .ceil() as u64;
    assert_eq!(required, 19_972);
    let world = configure_grid(&stand, required - 1);
    let report = stand.run(&world, &[("--catalog", &stand.catalog)], 1);
    assert!(report.contains(r#""status":"component_failure""#));
    assert!(report.contains(r#""steps":0,"#));
    assert!(report.contains(r#""blocks_proof":true"#));
    let reason = report
        .split_once(r#""component_failure":""#)
        .and_then(|(_, text)| text.split_once('"'))
        .map(|(reason, _)| reason)
        .unwrap();
    assert!(
        reason.contains("проверка шага")
            && reason.contains(&format!("Нужна частота физики ≥ {required} Гц")),
        "{reason}"
    );
    println!("{reason}");
    let world = configure_grid(&stand, required);
    stand.run(&world, &[("--catalog", &stand.catalog)], 0);
}
