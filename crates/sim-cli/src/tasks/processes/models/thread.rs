use sim_process::work::mechanics::{EngagementState, ThreadInput};

use super::*;

#[derive(Clone, Debug)]
struct ThreadRun {
    name: String,
    spec: ThreadSpec,
    pair: [String; 2],
    angular: String,
    axial: String,
    axial_id: usize,
    side: usize,
    torque: String,
    previous: Option<([f64; 2], [f64; 3])>,
    travel: f64,
    expected_travel: f64,
    virtual_force: f64,
    work: f64,
    max_axial: f64,
    max_torque: f64,
    max_error: f64,
}

impl ProcessModel for ThreadRun {
    fn update(
        &mut self,
        frame: &ProcessFrame<'_>,
        _: &mut Rng,
    ) -> Result<ProcessOut, ProcessError> {
        let q = [axis(frame, &self.angular)?.0, axis(frame, &self.axial)?.0];
        let direction = frame.truth.engine().joint_axis(self.axial_id);
        let before = self.previous.replace((q, direction));
        let mut out = ProcessOut::default();
        if let Some(((old, direction), completed)) = before.zip(frame.completed) {
            let drive = if self.spec.handedness == Handedness::Positive {
                1.0
            } else {
                -1.0
            };
            // Сила решателя на подвижном звене против подачи. Ось относится
            // к левой границе интервала, как и измеренные контактные силы.
            let resistance = completed
                .reactions
                .contacts
                .iter()
                .filter(|c| c.separation_m <= 0.0)
                .filter_map(|c| {
                    let sign = if c.geometries == self.pair {
                        if self.side == 1 { 1.0 } else { -1.0 }
                    } else if c.geometries[0] == self.pair[1] && c.geometries[1] == self.pair[0] {
                        if self.side == 0 { 1.0 } else { -1.0 }
                    } else {
                        return None;
                    };
                    Some(
                        c.force_n
                            .into_iter()
                            .zip(direction)
                            .map(|(f, a)| f * a)
                            .sum::<f64>()
                            * sign
                            * -drive,
                    )
                })
                .sum::<f64>();
            if !resistance.is_finite() {
                return Err(invalid("неконечная осевая реакция"));
            }
            let drive_torque_nm = frame
                .truth
                .process_value(&self.torque)
                .ok_or_else(|| invalid("нет измерения приложенного момента"))?;
            let result = self.spec.evaluate(&ThreadInput {
                engagement: Some(if resistance > 0.0 && normal(frame, &self.pair) > 0.0 {
                    EngagementState::Engaged
                } else {
                    EngagementState::Disengaged
                }),
                angle_delta_rad: q[0] - old[0],
                axial_delta_m: q[1] - old[1],
                drive_torque_nm,
                measured_tension_n: None,
            })?;
            if let Analysis::Analyzed { value, .. } = result.motion {
                self.travel += q[1] - old[1];
                self.expected_travel += value.expected_travel_m;
                self.virtual_force = self.virtual_force.max(value.axial_force_n.abs());
                self.work += value.input_work_j;
                self.max_axial = self.max_axial.max(resistance);
                self.max_torque = self.max_torque.max(value.applied_torque_nm);
                self.max_error = self.max_error.max(value.travel_error_m.abs());
            } else if matches!(
                result.alignment,
                sim_process::work::HazardCheck::Violation { .. }
            ) {
                out.blockers
                    .push("осевой ход противоречит фазе резьбы".into());
            }
        }
        emit(
            &mut out,
            frame.next.start,
            &self.name,
            &[
                ("travel", "m", self.travel),
                ("work", "J", self.work),
                (
                    "turns",
                    "1",
                    self.expected_travel / self.spec.lead_m_per_turn.nominal,
                ),
                ("virtual_axial_force", "N", self.virtual_force),
                ("peak_axial_force", "N", self.max_axial),
                ("peak_torque", "N*m", self.max_torque),
                ("maximum_step_error", "m", self.max_error),
            ],
            "фактический осевой ход, измеренная реакция против подачи и работа момента",
        )?;
        Ok(out)
    }

    fn state_fingerprint(&self) -> u64 {
        sim_signal::fnv1a(format!("{self:?}").bytes())
    }
}

pub(super) fn install(
    name: &str,
    world: &CompiledWorld,
    rt: &mut Runtime,
    pair: [String; 2],
    angular: String,
    axial: String,
    spec: ThreadSpec,
    torque: String,
) -> Result<(), String> {
    let sim = world.open_contact_sim()?;
    let axis = sim
        .joints()
        .into_iter()
        .find(|j| j.name == axial)
        .ok_or("поступательная ось отсутствует в движке")?;
    let sides: Vec<_> = sim
        .geoms()
        .into_iter()
        .filter(|g| g.body == axis.body)
        .filter_map(|g| pair.iter().position(|name| *name == g.name))
        .collect();
    let &[side] = sides.as_slice() else {
        return Err("один геом зацепления должен принадлежать телу поступательной оси".into());
    };
    rt.register_process(
        name,
        ThreadRun {
            name: name.into(),
            spec,
            pair,
            angular,
            axial,
            axial_id: axis.id,
            side,
            torque: format!("{torque}/effort"),
            previous: None,
            travel: 0.0,
            expected_travel: 0.0,
            virtual_force: 0.0,
            work: 0.0,
            max_axial: 0.0,
            max_torque: 0.0,
            max_error: 0.0,
        },
    )
}
