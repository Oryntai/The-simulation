use sim_signal::SignalId;

use super::*;

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(in crate::tasks::processes) struct DockCommand {
    at_ns: u64,
    attachment: String,
    engage: bool,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(in crate::tasks::processes) struct Dock {
    tool: Point,
    parking_m: [f64; 3],
}

/// Замок, которым командует контур: показание или выход цели на шине,
/// 1 — закрыть, 0 — открыть. Срабатывает по фронту.
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(in crate::tasks::processes) struct LockSignal {
    signal: String,
    max_age_ns: u64,
}

#[derive(Clone, Debug)]
struct Lock {
    signal: SignalId,
    max_age_ns: u64,
    /// Последний принятый уровень команды. До первой команды замок открыт.
    closed: bool,
}

#[derive(Clone)]
struct Coupler {
    name: String,
    carrier: Point,
    stations: BTreeMap<String, Dock>,
    commands: BTreeMap<Time, DockCommand>,
    lock: Option<Lock>,
    active: Option<String>,
    bounds: [f64; 4],
    tool_first: BTreeMap<String, bool>,
}

fn norm(a: [f64; 3], b: [f64; 3]) -> f64 {
    a.into_iter()
        .zip(b)
        .map(|(a, b)| (a - b).powi(2))
        .sum::<f64>()
        .sqrt()
}

impl ProcessModel for Coupler {
    fn state_fingerprint(&self) -> u64 {
        let closed = self.lock.as_ref().map(|lock| lock.closed);
        sim_signal::fnv1a(format!("{:?}{:?}{closed:?}", self.active, self.commands).bytes())
    }

    fn update(
        &mut self,
        frame: &ProcessFrame<'_>,
        _: &mut Rng,
    ) -> Result<ProcessOut, ProcessError> {
        let at = frame.next.start;
        let mut out = ProcessOut::default();
        if let (Some(attachment), Some(previous)) = (&self.active, frame.completed) {
            let key = format!("attachment/{attachment}");
            if let Some(reaction) = previous.weld_reactions.get(&key).and_then(|r| *r) {
                let side = if self.tool_first[attachment] {
                    reaction.on_first
                } else {
                    reaction.on_second
                };
                let [fx, fy, fz] = side.force_n;
                let [mx, my, mz] = side.torque_nm;
                emit(
                    &mut out,
                    at,
                    &key,
                    &[
                        ("force/x", "N", fx),
                        ("force/y", "N", fy),
                        ("force/z", "N", fz),
                        ("moment/x", "N*m", mx),
                        ("moment/y", "N*m", my),
                        ("moment/z", "N*m", mz),
                    ],
                    "Реакция на сменном звене в точке weld, в мировой системе за завершённый интервал",
                )?;
            } else {
                out.blockers
                    .push(format!("нет реакции замкнутого стыка «{attachment}»"));
            }
        }
        if self.lock.is_some() {
            self.follow_lock(frame, at, &mut out)?;
        } else {
            self.follow_schedule(frame, at, &mut out)?;
        }
        out.states.insert(self.name.clone(), self.state().into());
        Ok(out)
    }
}

impl Coupler {
    fn state(&self) -> &str {
        self.active.as_deref().unwrap_or("свободно")
    }

    /// Насадка стыка стоит в своём держателе у носителя, позы совпадают,
    /// оба тела почти неподвижны.
    fn docked(&self, frame: &ProcessFrame<'_>, attachment: &str) -> Result<bool, ProcessError> {
        let dock = &self.stations[attachment];
        let head = frame.body_point(&self.carrier.body, self.carrier.local_m)?;
        let tool = frame.body_point(&dock.tool.body, dock.tool.local_m)?;
        let trace = head
            .rotation
            .iter()
            .flatten()
            .zip(tool.rotation.iter().flatten())
            .map(|(a, b)| a * b)
            .sum::<f64>();
        // Из следа матрицы относительного поворота SO(3) = 1 + 2 cos θ.
        let orientation = ((trace - 1.0) / 2.0).clamp(-1.0, 1.0).acos();
        let [gap, angle, speed, spin] = self.bounds;
        Ok(norm(head.position_m, tool.position_m) <= gap
            && norm(tool.position_m, dock.parking_m) <= gap
            && orientation <= angle
            && norm(head.velocity_m_s, [0.0; 3]) <= speed
            && norm(tool.velocity_m_s, [0.0; 3]) <= speed
            && norm(head.angular_velocity_rad_s, [0.0; 3]) <= spin
            && norm(tool.angular_velocity_rad_s, [0.0; 3]) <= spin)
    }

    fn switch(
        &mut self,
        frame: &ProcessFrame<'_>,
        out: &mut ProcessOut,
        attachment: String,
        engage: bool,
        requested: Time,
        at: Time,
    ) -> Result<(), ProcessError> {
        let equality = format!("attachment/{attachment}");
        let actual = frame
            .truth
            .engine()
            .equality_active(&equality)
            .map_err(invalid)?;
        if actual == engage {
            return Err(invalid(format!(
                "стык «{attachment}» уже в заданном состоянии, учёт сменщика расходится с движком"
            )));
        }
        out.switch_weld_diagnostic(equality, engage);
        self.active = engage.then_some(attachment);
        out.events.push(ProcessEvent {
            name: self.name.clone(),
            state: self.state().into(),
            requested,
            applied: at,
        });
        Ok(())
    }

    fn follow_schedule(
        &mut self,
        frame: &ProcessFrame<'_>,
        at: Time,
        out: &mut ProcessOut,
    ) -> Result<(), ProcessError> {
        let due = self.commands.range(..=at).count();
        if due > 1 {
            return Err(invalid("несколько команд смены на одной границе"));
        }
        if due == 0 {
            return Ok(());
        }
        let (requested, command) = self
            .commands
            .pop_first()
            .expect("проверено наличие запроса");
        let actual = frame
            .truth
            .engine()
            .equality_active(&format!("attachment/{}", command.attachment))
            .map_err(invalid)?;
        let good = actual != command.engage
            && ((self.active.is_none() && command.engage)
                || (!command.engage && self.active.as_ref() == Some(&command.attachment)))
            && self.docked(frame, &command.attachment)?;
        if good {
            self.switch(frame, out, command.attachment, command.engage, requested, at)?;
        } else {
            out.blockers.push(format!(
                "стык «{}»: не выполнены условия обмена",
                command.attachment
            ));
        }
        Ok(())
    }

    /// Замок следует фронтам команды с шины. Устаревшая или невалидная команда
    /// не меняет уровень: механический замок держит последнее положение.
    fn follow_lock(
        &mut self,
        frame: &ProcessFrame<'_>,
        at: Time,
        out: &mut ProcessOut,
    ) -> Result<(), ProcessError> {
        let lock = self.lock.as_ref().expect("режим замка");
        let Some(sample) = frame.observation(lock.signal) else {
            return Ok(());
        };
        if !sample.valid || sample.at > at || at.0 - sample.at.0 > lock.max_age_ns {
            return Ok(());
        }
        let closed = if sample.value == 0.0 {
            false
        } else if sample.value == 1.0 {
            true
        } else {
            return Err(invalid(format!(
                "команда замка «{}» равна {}, допустимы 0 и 1",
                self.name, sample.value
            )));
        };
        if closed == lock.closed {
            return Ok(());
        }
        self.lock.as_mut().expect("режим замка").closed = closed;
        match (closed, self.active.clone()) {
            (true, None) => {
                let mut ready = Vec::new();
                for attachment in self.stations.keys() {
                    if self.docked(frame, attachment)? {
                        ready.push(attachment.clone());
                    }
                }
                match <[String; 1]>::try_from(ready) {
                    Ok([attachment]) => self.switch(frame, out, attachment, true, sample.at, at)?,
                    Err(ready) if ready.is_empty() => out.blockers.push(format!(
                        "замок «{}» закрыт без насадки в держателе у носителя",
                        self.name
                    )),
                    Err(_) => out.blockers.push(format!(
                        "замок «{}»: у носителя несколько насадок",
                        self.name
                    )),
                }
            }
            (false, Some(attachment)) => {
                // Открытый замок отпускает насадку в любом положении.
                // Вне держателя она падает, это отмечается.
                if !self.docked(frame, &attachment)? {
                    out.blockers.push(format!(
                        "насадка стыка «{attachment}» отпущена вне держателя"
                    ));
                }
                self.switch(frame, out, attachment, false, sample.at, at)?;
            }
            // Повторное закрытие при удержанной насадке или открытие пустого замка.
            _ => {}
        }
        Ok(())
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn install(
    name: &str,
    world: &CompiledWorld,
    rt: &mut Runtime,
    carrier: Point,
    stations: BTreeMap<String, Dock>,
    commands: Vec<DockCommand>,
    lock: Option<LockSignal>,
    bounds: [f64; 4],
) -> Result<(), String> {
    position(world, &carrier)?;
    if stations.is_empty() || bounds.iter().any(|&x| x <= 0.0) {
        return Err("нужны стыки и положительные допуски смены".into());
    }
    if commands.is_empty() == lock.is_none() {
        return Err("сменщику нужен ровно один способ команды: расписание или замок с шины".into());
    }
    let lock = match lock {
        Some(lock) => {
            if lock.max_age_ns == 0 {
                return Err("срок команды замка должен быть положительным".into());
            }
            let signal = rt.observation_id(&lock.signal).ok_or_else(|| {
                format!(
                    "нет сигнала «{}» на шине: замок читает выход цели контура или показание",
                    lock.signal
                )
            })?;
            Some(Lock {
                signal,
                max_age_ns: lock.max_age_ns,
                closed: false,
            })
        }
        None => None,
    };
    let mut tool_first = BTreeMap::new();
    for (attachment, station) in &stations {
        position(world, &station.tool)?;
        if carrier.body == station.tool.body {
            return Err("насадка и держатель должны принадлежать разным телам".into());
        }
        let bodies = rt.record_process_weld(&format!("attachment/{attachment}"))?;
        let expected = [&carrier.body, &station.tool.body];
        let first = bodies.each_ref() == [expected[1], expected[0]];
        if !first && bodies.each_ref() != expected {
            return Err("тела weld не совпадают с держателем и сменным звеном".into());
        }
        tool_first.insert(attachment.clone(), first);
    }
    let mut schedule = BTreeMap::new();
    for command in commands {
        if !stations.contains_key(&command.attachment)
            || schedule.insert(Time(command.at_ns), command).is_some()
        {
            return Err("повторная команда или неразрешённый стык".into());
        }
    }
    rt.register_process(
        name,
        Coupler {
            name: name.into(),
            carrier,
            stations,
            commands: schedule,
            lock,
            active: None,
            bounds,
            tool_first,
        },
    )
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use sim_runtime::{ObservationOut, Sensor, Truth};
    use sim_signal::Rate;

    use super::*;

    const HEAD: &str = r#"format = 3
name = "стойка"

[[links]]
name = "голова"
joint = { kind = "fixed" }
mass = { kind = "explicit", mass_kg = 1.0, com_m = [0.0, 0.0, 0.0], inertia_kg_m2 = [0.001, 0.001, 0.001, 0.0, 0.0, 0.0], provenance = { estimate = "тест" } }

[[links.mounts]]
name = "фланец"
kind = "mechanical"
interface = "тест"
pose = { position_m = [0.0, 0.0, 0.1] }
"#;

    const TOOL: &str = r#"format = 3
name = "насадка"

[[links]]
name = "основание"
joint = { kind = "free", name = "основание" }
mass = { kind = "explicit", mass_kg = 0.01, com_m = [0.0, 0.0, 0.0], inertia_kg_m2 = [1e-6, 1e-6, 1e-6, 0.0, 0.0, 0.0], provenance = { estimate = "тест" } }

[[links.mounts]]
name = "фланец"
kind = "mechanical"
interface = "тест"
"#;

    /// Невесомый стенд: неподвижная голова и две насадки. Насадка «А» в `a_x` по оси x от фланца.
    fn world(label: &str, a_x: f64) -> CompiledWorld {
        let dir: PathBuf = std::env::temp_dir().join(format!("sim-coupler-{label}-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("head.toml"), HEAD).unwrap();
        std::fs::write(dir.join("tool.toml"), TOOL).unwrap();
        let mut text = String::from(
            "format = 3\nname = \"стенд замка\"\nseed = 1\nphysics_hz = 1000\ngravity_m_s2 = [0.0, 0.0, 0.0]\n\n\
             [environment]\ndensity_kg_m3 = 0.0\nviscosity_pa_s = 0.0\nwind_m_s = [0.0, 0.0, 0.0]\n\n\
             [[devices]]\nname = \"стойка\"\nfile = \"head.toml\"\n",
        );
        for (name, x) in [("А", a_x), ("Б", 0.2)] {
            text += &format!(
                "\n[[devices]]\nname = \"{name}\"\nfile = \"tool.toml\"\npose = {{ position_m = [{x}, 0.0, 0.1] }}\n\n\
                 [[attachments]]\nname = \"{name}\"\nfirst = \"стойка/голова/фланец\"\nsecond = \"{name}/основание/фланец\"\nmode = \"runtime\"\n"
            );
        }
        std::fs::write(dir.join("world.toml"), text).unwrap();
        let world = sim_world::load_and_compile(&dir.join("world.toml")).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
        world
    }

    /// Команда замка по расписанию: уровень последней точки не позже текущего времени.
    #[derive(Clone)]
    struct Schedule(SignalId, Vec<(u64, f64)>);

    impl Sensor for Schedule {
        fn sample(&mut self, truth: &Truth<'_>, _: &mut Rng, out: &mut ObservationOut) -> Result<(), String> {
            let now = truth.time().0;
            let value = self.1.iter().rev().find(|(at, _)| *at <= now).map_or(0.0, |(_, v)| *v);
            out.write(self.0, value, true);
            Ok(())
        }
    }

    fn point(body: &str, z: f64) -> Point {
        Point { body: body.into(), local_m: [0.0, 0.0, z] }
    }

    fn stations() -> BTreeMap<String, Dock> {
        [("А", 0.0), ("Б", 0.2)]
            .map(|(name, x)| {
                (name.to_string(), Dock { tool: point(&format!("{name}/основание"), 0.0), parking_m: [x, 0.0, 0.1] })
            })
            .into_iter()
            .collect()
    }

    fn lock() -> Option<LockSignal> {
        Some(LockSignal { signal: "замок".into(), max_age_ns: 20_000_000 })
    }

    fn runtime(world: &CompiledWorld, steps: Vec<(u64, f64)>) -> Runtime {
        let mut rt = Runtime::load(world.clone()).unwrap();
        let signal = rt.register_observation("замок", "1").unwrap();
        rt.register_sensor("клапан", Rate::hz(100).unwrap(), Schedule(signal, steps)).unwrap();
        install("сменщик", world, &mut rt, point("стойка/голова", 0.1), stations(), Vec::new(), lock(), [0.002, 0.035, 0.005, 0.05])
            .unwrap();
        rt
    }

    /// Смены состояния сменщика и причины недоступности за прогон. Пометка рантайма
    /// о диагностическом weld сопровождает каждое замыкание и здесь не считается.
    fn history(rt: &Runtime) -> (Vec<(u64, String)>, Vec<String>) {
        let mut states: Vec<(u64, String)> = Vec::new();
        let mut blockers = Vec::new();
        for tick in rt.process_journal() {
            let Some(out) = tick.outputs.get("сменщик") else { continue };
            let state = out.states["сменщик"].clone();
            if states.last().is_none_or(|(_, last)| *last != state) {
                states.push((tick.at.0, state));
            }
            blockers.extend(out.blockers.iter().filter(|b| !b.starts_with("диагностический weld")).cloned());
        }
        (states, blockers)
    }

    #[test]
    fn замок_с_шины_берёт_и_отпускает_насадку_по_фронтам() {
        let world = world("фронты", 0.0);
        let steps = vec![(0, 0.0), (100_000_000, 1.0), (300_000_000, 0.0), (500_000_000, 1.0)];
        let mut rt = runtime(&world, steps);
        rt.run_until(Time(600_000_000)).unwrap();
        let (states, blockers) = history(&rt);
        assert!(blockers.is_empty(), "{blockers:?}");
        let names: Vec<_> = states.iter().map(|(_, s)| s.as_str()).collect();
        assert_eq!(names, ["свободно", "А", "свободно", "А"], "{states:?}");
        // Команда датчика 100 Гц доходит до процесса на ближайшей границе физики.
        for ((at, _), command) in states[1..].iter().zip([100_000_000, 300_000_000, 500_000_000]) {
            assert!((command..=command + 2_000_000).contains(at), "{states:?}");
        }
    }

    #[test]
    fn журнал_с_периодом_сплошной_и_хранит_события() {
        let world = world("журнал", 0.0);
        let steps = vec![(0, 0.0), (100_000_000, 1.0), (300_000_000, 0.0), (500_000_000, 1.0)];
        let mut full = runtime(&world, steps.clone());
        full.run_until(Time(600_000_000)).unwrap();
        let mut thin = runtime(&world, steps);
        thin.set_process_journal_period(50_000_000).unwrap();
        thin.run_until(Time(600_000_000)).unwrap();
        assert_eq!(history(&thin), history(&full));
        let rows = thin.process_journal();
        // 601 граница физики. Остаются начальная строка, последняя строка каждого из 12 окон
        // по 50 мс, строка на 600 мс в своём окне и три строки событий замка.
        assert_eq!(full.process_journal().len(), 601);
        assert_eq!(rows.len(), 17);
        assert_eq!(rows.first().unwrap().at, Time::ZERO);
        assert_eq!(rows.last().unwrap().at, Time(600_000_000));
        for pair in rows.windows(2) {
            let frame = pair[1].completed.as_ref().unwrap();
            assert_eq!((frame.interval.start, frame.interval.end), (pair[0].at, pair[1].at));
        }
        assert_eq!(rows.last().unwrap().qpos, full.process_journal().last().unwrap().qpos);
    }

    #[test]
    fn замок_без_насадки_у_фланца_не_замыкает_стык() {
        let world = world("пусто", 0.05);
        let mut rt = runtime(&world, vec![(0, 0.0), (100_000_000, 1.0)]);
        rt.run_until(Time(200_000_000)).unwrap();
        let (states, blockers) = history(&rt);
        assert_eq!(states.len(), 1, "{states:?}");
        assert_eq!(states[0].1, "свободно");
        assert_eq!(blockers.len(), 1, "{blockers:?}");
        assert!(blockers[0].contains("закрыт без насадки"), "{blockers:?}");
    }

    #[test]
    fn команда_замка_кроме_0_и_1_останавливает_прогон() {
        let world = world("уровень", 0.0);
        let mut rt = runtime(&world, vec![(0, 0.0), (100_000_000, 0.5)]);
        let status = rt.run_until(Time(200_000_000)).unwrap();
        assert_eq!(status, sim_runtime::Status::ComponentFailure);
        let reason = rt.failure().unwrap_or_default();
        assert!(reason.contains("допустимы 0 и 1"), "{reason}");
    }

    #[test]
    fn сменщику_нужен_ровно_один_способ_команды() {
        let world = world("способ", 0.0);
        let command = DockCommand { at_ns: 100_000_000, attachment: "А".into(), engage: true };
        let bounds = [0.002, 0.035, 0.005, 0.05];
        let head = || point("стойка/голова", 0.1);
        let mut rt = Runtime::load(world.clone()).unwrap();
        rt.register_observation("замок", "1").unwrap();
        let both = install("а", &world, &mut rt, head(), stations(), vec![command], lock(), bounds).unwrap_err();
        assert!(both.contains("ровно один способ"), "{both}");
        let none = install("б", &world, &mut rt, head(), stations(), Vec::new(), None, bounds).unwrap_err();
        assert!(none.contains("ровно один способ"), "{none}");
        let mut bare = Runtime::load(world.clone()).unwrap();
        let missing = install("в", &world, &mut bare, head(), stations(), Vec::new(), lock(), bounds).unwrap_err();
        assert!(missing.contains("нет сигнала «замок»"), "{missing}");
    }
}
