//! Запись для просмотра из того же экземпляра, который пишет SIMLOG.

use std::rc::Rc;

use sim_mujoco::contact::ContactWrench;
use sim_signal::{ImageFrame, Time, fnv1a};
use sim_world::CompiledSensor;

use crate::Runtime;

/// Исходная геометрия движка и монтажные кадры датчиков.
pub struct ViewScene {
    pub name: String,
    pub mjcf: String,
    pub sensors: Vec<CompiledSensor>,
    pub engine: String,
    pub effort_dofs: Vec<usize>,
}

impl ViewScene {
    /// Тот же алгоритм привязки, который используется заголовком SIMLOG.
    pub fn world_hash(name: &str, mjcf: &str, engine: &str, effort_dofs: &[usize]) -> u64 {
        fnv1a(
            [name, engine, mjcf]
                .into_iter()
                .flat_map(|s| (s.len() as u64).to_le_bytes().into_iter().chain(s.bytes()))
                .chain(
                    effort_dofs
                        .iter()
                        .flat_map(|&dof| (dof as u64).to_le_bytes()),
                ),
        )
    }

    pub fn fingerprint(&self) -> u64 {
        Self::world_hash(&self.name, &self.mjcf, &self.engine, &self.effort_dofs)
    }
}

/// Реакции Euler вне профиля процессов. Позы относятся к началу интервала.
#[derive(Clone)]
pub struct ViewContacts {
    pub start: Time,
    pub end: Time,
    pub contacts: Vec<ContactWrench>,
}

/// Связь, которую движок переключил на границе физики.
#[derive(Clone)]
pub struct ViewEquality {
    pub at: Time,
    pub name: String,
    pub active: bool,
}

#[derive(Clone, Default)]
pub struct ViewRecording {
    pub(crate) frames: Rc<Vec<ImageFrame>>,
    pub(crate) contacts: Rc<Vec<ViewContacts>>,
    pub(crate) contact_gap: Option<String>,
    pub(crate) equality: Rc<Vec<ViewEquality>>,
    /// Окно прореживания, в котором последний раз записаны контакты.
    pub(crate) contact_window: Option<u64>,
}

impl ViewRecording {
    pub fn frames(&self) -> &[ImageFrame] {
        &self.frames
    }
    pub fn contacts(&self) -> &[ViewContacts] {
        &self.contacts
    }
    pub fn equality_changes(&self) -> &[ViewEquality] {
        &self.equality
    }
    pub fn contact_gap(&self) -> Option<&str> {
        self.contact_gap.as_deref()
    }
}

impl Runtime {
    /// Включает запись кадров до первого события. Снимки и ветви сохраняют запись.
    pub fn record_view(&mut self) -> Result<(), String> {
        if self.log().end().started || self.state.view.is_some() {
            return Err("включите запись просмотра один раз до запуска".into());
        }
        self.state.view = Some(ViewRecording::default());
        Ok(())
    }

    pub fn view_scene(&self) -> &ViewScene {
        &self.view_scene
    }

    pub fn view_recording(&self) -> Option<&ViewRecording> {
        self.state.view.as_ref()
    }

    pub(crate) fn capture_view_contacts(&mut self, at: Time) {
        if self.state.view.is_none() {
            return;
        }
        let previous_step = self.state.log.end.steps;
        // Шкала Clock и предыдущая граница, включая нулевой интервал.
        let start = self.physics_rate().tick(previous_step);
        let period = self.state.log.truth_period();
        let Some(view) = &mut self.state.view else {
            return;
        };
        // С прореживанием истины (truth_hz) контакты для просмотра пишутся раз в окно:
        // на десятках килогерц запись каждого шага не помещается в память.
        if let Some(period) = period {
            let window = at.0 / period;
            if view.contact_window == Some(window) {
                return;
            }
            view.contact_window = Some(window);
        }
        if view.contact_gap.is_some() {
            return;
        }
        if self.sim.model().opt.integrator != sim_mujoco::ffi::mjtIntegrator::mjINT_EULER as i32 {
            view.contact_gap = Some("для интегратора нет реакции на левой границе шага".into());
            return;
        }
        match self.sim.contact_wrenches() {
            Ok(contacts) if !contacts.is_empty() => {
                Rc::make_mut(&mut view.contacts).push(ViewContacts {
                    start,
                    end: at,
                    contacts,
                });
            }
            Ok(_) => {}
            Err(reason) => view.contact_gap = Some(reason),
        }
    }
}
