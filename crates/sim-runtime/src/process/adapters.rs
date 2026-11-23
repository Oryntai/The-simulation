use std::collections::BTreeMap;

use sim_process::{
    Analysis, ProcessError, StepModel,
    attachments::{
        AttachmentEvent, AttachmentInput, AttachmentModel, AttachmentOutcome, AttachmentRequest,
        LinkEffect, SoftInput, SoftModel,
    },
    work::immersion::{ImmersionInput, ImmersionSpec},
};
use sim_signal::{ChannelId, Rng, Time};

use super::{AttachmentRecord, ProcessEvent, ProcessFrame, ProcessModel, ProcessOut};
use crate::Runtime;

fn invalid(reason: impl Into<String>) -> ProcessError {
    ProcessError::InvalidInput(reason.into())
}

fn unavailable<T>(reason: impl Into<String>) -> Analysis<T> {
    Analysis::NotAnalyzed {
        reason: reason.into(),
    }
}

fn field<T, U>(value: &Analysis<T>, f: impl FnOnce(&T) -> U) -> Analysis<U> {
    match value {
        Analysis::Analyzed { value, scope } => Analysis::Analyzed {
            value: f(value),
            scope: scope.clone(),
        },
        Analysis::NotAnalyzed { reason } => unavailable(reason.clone()),
    }
}

#[derive(Clone)]
pub struct AttachmentAdapter<I> {
    pub model: AttachmentModel,
    pub equality: Option<String>,
    /// Абсолютные моменты команд. Они округляются вверх до границы шага.
    pub requests: BTreeMap<Time, AttachmentRequest>,
    pub input: I,
}

impl Runtime {
    pub fn register_immersion<I>(&mut self, adapter: ImmersionAdapter<I>) -> Result<(), String>
    where
        I: FnMut(&ProcessFrame<'_>) -> Result<ImmersionInput, ProcessError> + Clone + 'static,
    {
        let name = adapter.name.clone();
        let before = self.snapshot();
        let result = (|| {
            if self.process_requirements().is_none() {
                self.set_process_inventory(&[], &[])?;
            }
            self.require_process_hazards(&[sim_process::work::HazardEvidence {
                subject: name.clone(),
                kind: sim_process::work::HazardKind::Ingress,
                coverage: unavailable("для герметичности нужен отдельный предел"),
            }])?;
            self.register_process(&name, adapter)
        })();
        if result.is_err() {
            self.restore(&before)?;
        }
        result
    }

    pub fn register_attachment<I>(&mut self, adapter: AttachmentAdapter<I>) -> Result<(), String>
    where
        I: FnMut(&ProcessFrame<'_>) -> Result<AttachmentInput, ProcessError> + Clone + 'static,
    {
        let name = adapter.model.binding().name.clone();
        if adapter
            .requests
            .iter()
            .any(|(&at, request)| request.at != at || request.name != name)
        {
            return Err("невалидное расписание крепления".into());
        }
        let before = self.snapshot();
        let result = (|| {
            if let Some(id) = &adapter.equality {
                use sim_mujoco::ffi::{mjtEq, mjtObj};
                let index = self
                    .sim
                    .id(mjtObj::mjOBJ_EQUALITY, id)
                    .ok_or("имя ограничения не разрешено")?;
                // eq_type — массив из neq элементов, index получен по имени.
                if unsafe { *self.sim.model().eq_type.add(index) } as u32 != mjtEq::mjEQ_WELD {
                    return Err("креплению требуется ограничение weld".into());
                }
                if self.sim.equality_active(id)? {
                    return Err("для свободного крепления нужна исходно выключенная связь".into());
                }
                self.record_process_equality(id)?;
            }
            if self.process_requirements().is_none() {
                self.set_process_inventory(&[], &[])?;
            }
            self.require_process_hazards(&adapter.model.hazards())?;
            self.register_process(&name, adapter)
        })();
        if result.is_err() {
            self.restore(&before)?;
        }
        result
    }

    pub fn register_soft_object<I>(&mut self, adapter: SoftAdapter<I>) -> Result<(), String>
    where
        I: FnMut(&ProcessFrame<'_>) -> Result<SoftInput, ProcessError> + Clone + 'static,
    {
        let name = adapter.model.binding().name.clone();
        let before = self.snapshot();
        let result = (|| {
            if self.process_requirements().is_none() {
                self.set_process_inventory(&[], &[])?;
            }
            self.require_process_hazards(&adapter.model.hazards())?;
            self.register_process(&name, adapter)
        })();
        if result.is_err() {
            self.restore(&before)?;
        }
        result
    }
}

impl<I> ProcessModel for AttachmentAdapter<I>
where
    I: FnMut(&ProcessFrame<'_>) -> Result<AttachmentInput, ProcessError>,
{
    fn update(
        &mut self,
        frame: &ProcessFrame<'_>,
        rng: &mut Rng,
    ) -> Result<ProcessOut, ProcessError> {
        let mut out = ProcessOut::default();
        let at = frame.next.start;
        let name = self.model.binding().name.clone();
        let Some(previous) = frame.completed else {
            out.states
                .insert(name.clone(), format!("{:?}", self.model.state()));
            out.attachments.push(AttachmentRecord {
                name,
                state: self.model.state(),
                failed: false,
                event: None,
                reaction: unavailable("первый интервал ещё не завершён"),
                applied_limit: None,
                cause: None,
                released_energy_j: None,
            });
            return Ok(out);
        };
        let mut input = (self.input)(frame)?;
        if self.model.binding().kind == sim_process::attachments::AttachmentKind::Latch
            && !previous.contact_gaps.is_empty()
        {
            input.latch_contact_n = unavailable("контактные пары для защёлки не подтверждены");
        }
        if let Some(name) = &self.equality
            && frame
                .truth
                .engine()
                .equality_active(name)
                .map_err(invalid)?
            && previous
                .equality_reactions
                .get(name)
                .is_none_or(Option::is_none)
        {
            input.measured_wrench = unavailable("нет фактической реакции активной связи");
        }
        let due = self.requests.range(..=at).count();
        if due > 1 || (due != 0 && input.command.is_some()) {
            return Err(invalid("конфликт команд крепления на границе физики"));
        }
        if due != 0 {
            input.command = self.requests.pop_first().map(|(_, request)| request);
        }
        let step = self.model.advance(previous.interval, &input, rng)?;
        let mut event = step.event;
        let mut applied_limit = None;
        let mut cause = None;
        let mut released_energy_j = None;
        if let Some(change) = step.change {
            if let LinkEffect::Weld(active) = change.effect {
                let id = self
                    .equality
                    .clone()
                    .ok_or_else(|| invalid("не разрешена ссылка на ограничение"))?;
                let actual = frame.truth.engine().equality_active(&id).map_err(invalid)?;
                if actual == active {
                    return Err(invalid(
                        "состояние движка не соответствует переходу крепления",
                    ));
                }
                applied_limit = Some((id, active));
            }
            let transition = AttachmentEvent {
                request: change.request,
                evaluated_at: at,
                outcome: AttachmentOutcome::Transition {
                    from: change.from,
                    to: change.to,
                },
            };
            // Подтверждается состояние внутренней копии. Копия станет рабочей
            // только после атомарного применения всех эффектов в движке.
            self.model.confirm(&transition)?;
            cause = Some(change.cause);
            released_energy_j = Some(change.released_energy_j);
            out.events.push(ProcessEvent {
                name: name.clone(),
                state: format!("{:?}", change.to),
                requested: transition.request.at,
                applied: at,
            });
            event = Some(transition);
        } else if let Some(request) = input.command
            && event.is_none()
        {
            event = Some(AttachmentEvent {
                request,
                evaluated_at: at,
                outcome: AttachmentOutcome::Rejected {
                    reason: "запрос не вызвал переход".into(),
                },
            });
        }
        out.body_loads = step.loads;
        if let Analysis::NotAnalyzed { reason } = &step.reaction {
            out.blockers.push(reason.clone());
        }
        let state = self.model.state();
        out.states.insert(name.clone(), format!("{state:?}"));
        out.attachments.push(AttachmentRecord {
            name,
            state,
            failed: step.failed
                || matches!(
                    cause,
                    Some(sim_process::attachments::AttachmentCause::Stripped)
                ),
            event,
            reaction: step.reaction,
            applied_limit,
            cause,
            released_energy_j,
        });
        Ok(out)
    }

    fn state_fingerprint(&self) -> u64 {
        sim_signal::fnv1a(format!("{:?}{:?}", self.model, self.requests).bytes())
    }
}

#[derive(Clone)]
pub struct SoftAdapter<I> {
    pub model: SoftModel,
    pub input: I,
}

impl<I> ProcessModel for SoftAdapter<I>
where
    I: FnMut(&ProcessFrame<'_>) -> Result<SoftInput, ProcessError>,
{
    fn update(
        &mut self,
        frame: &ProcessFrame<'_>,
        rng: &mut Rng,
    ) -> Result<ProcessOut, ProcessError> {
        let Some(previous) = frame.completed else {
            return Ok(ProcessOut::default());
        };
        let mut input = (self.input)(frame)?;
        if !previous.contact_gaps.is_empty() {
            input.contacts = unavailable("пары материалов ближайшего шага не подтверждены");
        }
        let step = self.model.advance(previous.interval, &input, rng)?;
        let measured = step.measure();
        let mut out = ProcessOut {
            body_loads: step.loads,
            ..ProcessOut::default()
        };
        if let Analysis::NotAnalyzed { reason } = &step.response {
            out.blockers.push(reason.clone());
        }
        for (field, unit, result) in [
            ("deformation", "m", measured.deformation_m),
            ("reaction", "N", measured.reaction_n),
            ("stored_energy", "J", measured.stored_energy_j),
            ("dissipation", "W", step.dissipated_power_w),
            ("foam_loss", "J", step.foam_loss_j),
        ] {
            out.measure(step.at, format!("{}/{field}", measured.name), unit, result)?;
        }
        if let Analysis::Analyzed { value, scope } = step.tensions_n {
            for (i, force) in value.into_iter().enumerate() {
                out.measure(
                    step.at,
                    format!("{}/tension/{i}", measured.name),
                    "N",
                    Analysis::Analyzed {
                        value: force,
                        scope: scope.clone(),
                    },
                )?;
            }
        }
        Ok(out)
    }

    fn state_fingerprint(&self) -> u64 {
        sim_signal::fnv1a(format!("{:?}", self.model).bytes())
    }
}

#[derive(Clone)]
pub struct ImmersionAdapter<I> {
    pub name: String,
    pub model: ImmersionSpec,
    pub drive: Option<ChannelId>,
    pub radio: Vec<String>,
    pub input: I,
}

impl<I> ProcessModel for ImmersionAdapter<I>
where
    I: FnMut(&ProcessFrame<'_>) -> Result<ImmersionInput, ProcessError>,
{
    fn update(
        &mut self,
        frame: &ProcessFrame<'_>,
        rng: &mut Rng,
    ) -> Result<ProcessOut, ProcessError> {
        let mut input = (self.input)(frame)?;
        let opt = &frame.truth.engine().model().opt;
        input.engine_density_kg_m3 = opt.density;
        input.engine_viscosity_pa_s = opt.viscosity;
        input.gravity_m_s2 = opt.gravity;
        let step = self.model.advance(frame.next, &input, rng)?;
        let at = frame.next.start;
        let mut out = ProcessOut::default();
        let calculated = |value| Analysis::Analyzed {
            value,
            scope: "геометрия прямоугольного объёма и плоскость жидкости".into(),
        };
        out.measure(
            at,
            format!("{}/volume", self.name),
            "m^3",
            calculated(step.volume_m3),
        )?;
        out.measure(
            at,
            format!("{}/wetted_area", self.name),
            "m^2",
            calculated(step.wetted_area_m2),
        )?;
        if input.antenna_m.is_some() {
            out.measure(
                at,
                format!("{}/radio_depth", self.name),
                "m",
                field(&step.radio, |radio| radio.depth_m),
            )?;
        }
        for (i, force) in step.buoyancy.force_n.into_iter().enumerate() {
            out.measure(
                at,
                format!("{}/buoyancy/{i}", self.name),
                "N",
                calculated(force),
            )?;
            out.measure(
                at,
                format!("{}/drag/{i}", self.name),
                "N",
                field(&step.drag, |load| load.force_n[i]),
            )?;
            out.measure(
                at,
                format!("{}/net_force/{i}", self.name),
                "N",
                field(&step.body_load, |load| load.force_n[i]),
            )?;
        }
        // Неполная оценка силы явно записана в net_force. Известная
        // плавучесть действует даже в диагностическом расчёте без сопротивления.
        match step.body_load {
            Analysis::Analyzed { value, .. } => out.body_loads.push(value),
            Analysis::NotAnalyzed { reason } => {
                out.blockers.push(reason);
                out.body_loads.push(step.buoyancy);
            }
        }
        if let Some(drive) = self.drive {
            if let Analysis::NotAnalyzed { reason } = &step.drive_thermal {
                out.blockers.push(reason.clone());
            }
            out.thermal.insert(drive, step.drive_thermal);
        }
        if !self.radio.is_empty() {
            if let Analysis::NotAnalyzed { reason } = &step.radio {
                out.blockers.push(reason.clone());
            }
            for route in &self.radio {
                if out
                    .radio
                    .insert(route.clone(), step.radio.clone())
                    .is_some()
                {
                    return Err(invalid("повтор маршрута радиолинии"));
                }
            }
        }
        Ok(out)
    }

    fn state_fingerprint(&self) -> u64 {
        sim_signal::fnv1a(format!("{:?}", self.model).bytes())
    }
}
