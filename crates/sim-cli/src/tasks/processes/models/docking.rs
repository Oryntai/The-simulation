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

#[derive(Clone)]
struct Coupler {
    name: String,
    carrier: Point,
    stations: BTreeMap<String, Dock>,
    commands: BTreeMap<Time, DockCommand>,
    active: Option<String>,
    bounds: [f64; 4],
    tool_first: BTreeMap<String, bool>,
}

impl ProcessModel for Coupler {
    fn state_fingerprint(&self) -> u64 {
        sim_signal::fnv1a(format!("{:?}{:?}", self.active, self.commands).bytes())
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
        let due = self.commands.range(..=at).count();
        if due > 1 {
            return Err(invalid("несколько команд смены на одной границе"));
        }
        if due != 0 {
            let (requested, command) = self
                .commands
                .pop_first()
                .expect("проверено наличие запроса");
            let dock = &self.stations[&command.attachment];
            let head = frame.body_point(&self.carrier.body, self.carrier.local_m)?;
            let tool = frame.body_point(&dock.tool.body, dock.tool.local_m)?;
            let norm = |a: [f64; 3], b: [f64; 3]| {
                a.into_iter()
                    .zip(b)
                    .map(|(a, b)| (a - b).powi(2))
                    .sum::<f64>()
                    .sqrt()
            };
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
            let equality = format!("attachment/{}", command.attachment);
            let actual = frame
                .truth
                .engine()
                .equality_active(&equality)
                .map_err(invalid)?;
            let good = actual != command.engage
                && ((self.active.is_none() && command.engage)
                    || (!command.engage && self.active.as_ref() == Some(&command.attachment)))
                && norm(head.position_m, tool.position_m) <= gap
                && norm(tool.position_m, dock.parking_m) <= gap
                && orientation <= angle
                && norm(head.velocity_m_s, [0.0; 3]) <= speed
                && norm(tool.velocity_m_s, [0.0; 3]) <= speed
                && norm(head.angular_velocity_rad_s, [0.0; 3]) <= spin
                && norm(tool.angular_velocity_rad_s, [0.0; 3]) <= spin;
            if good {
                out.switch_weld_diagnostic(equality, command.engage);
                self.active = command.engage.then_some(command.attachment);
                out.events.push(ProcessEvent {
                    name: self.name.clone(),
                    state: self.state().into(),
                    requested,
                    applied: at,
                });
            } else {
                out.blockers.push(format!(
                    "стык «{}»: не выполнены условия обмена",
                    command.attachment
                ));
            }
        }
        out.states.insert(self.name.clone(), self.state().into());
        Ok(out)
    }
}

impl Coupler {
    fn state(&self) -> &str {
        self.active.as_deref().unwrap_or("свободно")
    }
}

pub(super) fn install(
    name: &str,
    world: &CompiledWorld,
    rt: &mut Runtime,
    carrier: Point,
    stations: BTreeMap<String, Dock>,
    commands: Vec<DockCommand>,
    bounds: [f64; 4],
) -> Result<(), String> {
    position(world, &carrier)?;
    if stations.is_empty() || commands.is_empty() || bounds.iter().any(|&x| x <= 0.0) {
        return Err("нужны стыки, команды и положительные допуски смены".into());
    }
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
            active: None,
            bounds,
            tool_first,
        },
    )
}
