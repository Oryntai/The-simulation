use sim_runtime::TruthId;

use super::*;

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(in crate::tasks::processes) struct Parameters {
    joint: String,
    pivot_m: [f64; 3],
    axis_world: [f64; 3],
    inertia_kg_m2: Scalar,
    gravity_sine_nm: Scalar,
    damping_nms: Scalar,
    drive_torques: Vec<String>,
    reaction: Option<String>,
    contact_on: Option<String>,
}

#[derive(Clone)]
struct Balance {
    name: String,
    p: Parameters,
    drive_torques: Vec<TruthId>,
    previous: Option<(f64, f64)>,
    total_residual: f64,
    support_impulse: f64,
    first_contact_s: Option<f64>,
}

impl ProcessModel for Balance {
    fn state_fingerprint(&self) -> u64 {
        sim_signal::fnv1a(
            format!(
                "{:?}{:?}{:?}{:?}",
                self.previous, self.total_residual, self.support_impulse, self.first_contact_s
            )
            .bytes(),
        )
    }

    fn update(
        &mut self,
        frame: &ProcessFrame<'_>,
        _: &mut Rng,
    ) -> Result<ProcessOut, ProcessError> {
        let current = axis(frame, &self.p.joint)?;
        let mut out = ProcessOut::default();
        let Some(((q, v), completed)) = self.previous.replace(current).zip(frame.completed) else {
            emit(
                &mut out,
                frame.next.start,
                &self.name,
                &[
                    ("step_residual", "N*m*s", 0.0),
                    ("cumulative_residual", "N*m*s", 0.0),
                    ("support_impulse", "N*m*s", 0.0),
                ],
                "начальная граница: интегрированных интервалов нет",
            )?;
            return Ok(out);
        };
        let dt = Time(completed.interval.end.0 - completed.interval.start.0).as_secs();
        let mut torque = -self.p.gravity_sine_nm.value * q.sin() - self.p.damping_nms.value * v;
        for &id in &self.drive_torques {
            torque += frame
                .truth
                .plant(id)
                .ok_or_else(|| invalid("не записан момент на всём интервале"))?;
        }
        let mut reaction = if let Some(name) = &self.p.reaction {
            frame
                .truth
                .process_value(name)
                .ok_or_else(|| invalid("не записана реакция опоры"))?
        } else {
            0.0 // При отсутствии опоры внешнее слагаемое не задано.
        };
        if let Some(geom) = &self.p.contact_on {
            for c in &completed.reactions.contacts {
                let sign = if c.geometries[1] == *geom {
                    1.0
                } else if c.geometries[0] == *geom {
                    -1.0
                } else {
                    continue;
                };
                let [x, y, z] = std::array::from_fn(|i| c.point_m[i] - self.p.pivot_m[i]);
                let [fx, fy, fz] = c.force_n;
                let r_cross_f = [y * fz - z * fy, z * fx - x * fz, x * fy - y * fx];
                reaction += r_cross_f
                    .into_iter()
                    .zip(c.torque_nm)
                    .zip(self.p.axis_world)
                    .map(|((m, t), a)| sign * (m + t) * a)
                    .sum::<f64>();
                if c.force_n != [0.0; 3] && self.first_contact_s.is_none() {
                    self.first_contact_s = Some(completed.interval.start.as_secs());
                }
            }
        }
        let error = self.p.inertia_kg_m2.value * (current.1 - v) - (torque + reaction) * dt;
        self.total_residual += error;
        self.support_impulse += reaction * dt;
        emit(
            &mut out,
            frame.next.start,
            &self.name,
            &[
                ("step_residual", "N*m*s", error),
                ("cumulative_residual", "N*m*s", self.total_residual),
                ("support_impulse", "N*m*s", self.support_impulse),
                ("reaction", "N*m", reaction),
            ],
            "J·Δω и импульс суммы независимых моментов, контакт приведён r×F к оси",
        )?;
        if let Some(at) = self.first_contact_s {
            emit(
                &mut out,
                frame.next.start,
                &self.name,
                &[("first_contact", "s", at)],
                "первый завершённый интервал с реакцией",
            )?;
        }
        Ok(out)
    }
}

pub(super) fn install(
    name: &str,
    world: &CompiledWorld,
    rt: &mut Runtime,
    p: Parameters,
) -> Result<(), String> {
    if joint(world, &p.joint)? != CompiledJointKind::Hinge {
        return Err("баланс импульса требует вращательного соединения".into());
    }
    let axis_len = p.axis_world.iter().map(|x| x * x).sum::<f64>().sqrt();
    if (axis_len - 1.0).abs() > f64::EPSILON.sqrt()
        || p.inertia_kg_m2.value <= 0.0
        || p.damping_nms.value < 0.0
    {
        return Err("недопустимые ось, приведённая инерция или демпфирование".into());
    }
    for value in [&p.inertia_kg_m2, &p.gravity_sine_nm, &p.damping_nms] {
        sim_process::SourcedRange::from(value.clone())
            .validate()
            .map_err(as_error)?;
    }
    if let Some(geom) = &p.contact_on
        && !world.contact_geometry_names()?.contains(geom)
    {
        return Err(format!("нет геометрии «{geom}» для проверки импульса"));
    }
    let drive_torques = p
        .drive_torques
        .iter()
        .map(|name| {
            rt.truth_id(name)
                .ok_or_else(|| format!("не найден ряд момента «{name}»"))
        })
        .collect::<Result<_, _>>()?;
    rt.register_process(
        name,
        Balance {
            name: name.into(),
            p,
            drive_torques,
            previous: None,
            total_residual: 0.0,
            support_impulse: 0.0,
            first_contact_s: None,
        },
    )
}
