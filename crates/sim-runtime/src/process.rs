//! Эффекты процессов фиксируются между завершённым и следующим шагами физики.

use std::{
    collections::{BTreeMap, BTreeSet},
    rc::Rc,
};

use sim_mujoco::{
    contact::ContactStep,
    ffi,
    state::{BodyWrench, EqualitySwitch, WeldReaction},
};
use sim_process::{
    Analysis, BodyLoad, Interval, ProcessError, SourcedRange, StepModel,
    attachments::{AttachmentEvent, AttachmentState},
    contact::{ContactFrame, ResolvedContactPair},
    work::{
        HazardCheck, HazardEvidence, HazardKind, ProcessBinding, ProcessMeasurement, WorkSubject,
        required_hazards,
    },
};
use sim_signal::{ChannelId, Rng, Sample, SignalId, Time};
use sim_world::{CompiledWorld, Provenance, Simplification};

use crate::{Runtime, Truth, TruthId};

mod adapters;
pub mod interaction;

pub use adapters::{AttachmentAdapter, ImmersionAdapter, SoftAdapter};
pub use sim_process::{
    Analysis as ProcessAnalysis,
    work::immersion::{RadioEnvironment, ThermalBoundary},
};

pub struct ProcessFrame<'a> {
    pub truth: Truth<'a>,
    /// Пусто на начальной границе. Реакции принадлежат всему завершённому интервалу.
    pub completed: Option<&'a PhysicsFrame>,
    pub next: Interval,
    observations: &'a crate::ObservationBus,
    contact_calibrations: Option<&'a BTreeMap<[String; 2], ResolvedContactPair>>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyPoint {
    pub position_m: [f64; 3],
    pub com_m: [f64; 3],
    /// Строки матрицы поворота из тела в мир.
    pub rotation: [[f64; 3]; 3],
    pub velocity_m_s: [f64; 3],
    pub angular_velocity_rad_s: [f64; 3],
}

impl ProcessFrame<'_> {
    /// Доставленное показание или выход цели из контура, доступный на границе.
    pub fn observation(&self, signal: SignalId) -> Option<Sample> {
        self.observations.sample(signal).map(|(sample, _)| sample)
    }

    /// Кинематика точки тела. Скорость берётся из якобиана MuJoCo в мировой системе.
    pub fn body_point(&self, name: &str, local_m: [f64; 3]) -> Result<BodyPoint, ProcessError> {
        if !local_m.into_iter().all(f64::is_finite) {
            return Err(ProcessError::InvalidInput(
                "неконечная координата точки".into(),
            ));
        }
        let sim = self.truth.engine();
        let id = sim
            .id(ffi::mjtObj::mjOBJ_BODY, name)
            .ok_or_else(|| ProcessError::InvalidInput(format!("нет тела «{name}»")))?;
        let (origin, _) = sim.body_pose(id);
        // mjData.xmat хранит по девять чисел на каждый body ID.
        let rotation: [[f64; 3]; 3] = unsafe { *sim.data().xmat.add(9 * id).cast() };
        let position_m = std::array::from_fn(|i| {
            origin[i]
                + rotation[i]
                    .iter()
                    .zip(local_m)
                    .map(|(a, b)| a * b)
                    .sum::<f64>()
        });
        if !position_m.into_iter().all(f64::is_finite) {
            return Err(ProcessError::Execution(
                "переполнение координаты точки".into(),
            ));
        }
        let n = sim.nv();
        let mut linear = vec![0.0; 3 * n];
        let mut angular = vec![0.0; 3 * n];
        if n != 0 {
            // mj_jac принимает мировую точку и два выходных массива 3×nv.
            unsafe {
                ffi::mj_jac(
                    sim.model(),
                    sim.data(),
                    linear.as_mut_ptr(),
                    angular.as_mut_ptr(),
                    position_m.as_ptr(),
                    id as i32,
                );
            }
        }
        let velocity = |jacobian: &[f64]| {
            std::array::from_fn(|i| {
                jacobian[i * n..(i + 1) * n]
                    .iter()
                    .zip(sim.qvel())
                    .map(|(j, v)| j * v)
                    .sum()
            })
        };
        let point = BodyPoint {
            position_m,
            com_m: sim.body_com(id),
            rotation,
            velocity_m_s: velocity(&linear),
            angular_velocity_rad_s: velocity(&angular),
        };
        if !point
            .com_m
            .iter()
            .chain(&point.velocity_m_s)
            .chain(&point.angular_velocity_rad_s)
            .all(|x| x.is_finite())
        {
            return Err(ProcessError::Execution("неконечная кинематика тела".into()));
        }
        Ok(point)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct PhysicsFrame {
    pub interval: Interval,
    pub reactions: ContactFrame,
    pub engine_step: ContactStep,
    pub equality_reactions: BTreeMap<String, Option<Vec<f64>>>,
    pub weld_reactions: BTreeMap<String, Option<WeldReaction>>,
    /// Причины недоказанности контактов шага. Общий список, пока набор пар в контакте не меняется.
    pub contact_gaps: Rc<Vec<String>>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ProcessHazard {
    pub subject: String,
    pub kind: HazardKind,
    pub check: HazardCheck,
    /// Входные параметры и предел, от которых зависит это сравнение.
    pub sources: Vec<SourcedRange>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ProcessEvent {
    pub name: String,
    pub state: String,
    pub requested: Time,
    pub applied: Time,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AttachmentRecord {
    pub name: String,
    pub state: AttachmentState,
    pub failed: bool,
    pub event: Option<AttachmentEvent>,
    pub reaction: Analysis<sim_process::attachments::AttachmentReaction>,
    pub applied_limit: Option<(String, bool)>,
    pub cause: Option<sim_process::attachments::AttachmentCause>,
    pub released_energy_j: Option<Analysis<f64>>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ProcessOut {
    pub efforts: Vec<(ChannelId, f64)>,
    pub body_loads: Vec<BodyLoad>,
    pub measurements: Vec<ProcessMeasurement>,
    pub hazards: Vec<ProcessHazard>,
    pub events: Vec<ProcessEvent>,
    pub states: BTreeMap<String, String>,
    pub thermal: BTreeMap<ChannelId, Analysis<ThermalBoundary>>,
    pub radio: BTreeMap<String, Analysis<RadioEnvironment>>,
    pub blockers: Vec<String>,
    pub interaction: Option<interaction::InteractionTick>,
    stop_after_record: Option<String>,
    pub(crate) attachments: Vec<AttachmentRecord>,
    pub(crate) equality_switches: Vec<(String, bool)>,
    diagnostic_welds: Vec<String>,
    scalar_hold: Option<String>,
}

impl ProcessOut {
    /// Удерживает текущую координату на следующем шаге. Поддерживается
    /// только изолированная скалярная ось. Реакция идеального стопора не измеряется.
    pub fn hold_scalar_axis(&mut self, joint: String) {
        self.scalar_hold = Some(joint);
    }

    /// Планирует изменение зарегистрированной связи на границе физики.
    /// Рантайм проверяет конфликт до изменения состояния движка.
    pub fn switch_equality(&mut self, name: String, active: bool) {
        self.equality_switches.push((name, active));
    }

    /// Диагностическая смена связи допускает численную коррекцию невязки.
    /// Работа захвата остаётся непроверенной и блокирует доказательство.
    pub fn switch_weld_diagnostic(&mut self, name: String, active: bool) {
        if active {
            self.diagnostic_welds.push(name.clone());
        }
        self.switch_equality(name, active);
    }

    pub fn attachment_records(&self) -> &[AttachmentRecord] {
        &self.attachments
    }

    pub fn measure(
        &mut self,
        at: Time,
        name: impl Into<String>,
        unit: &str,
        result: Analysis<f64>,
    ) -> Result<(), ProcessError> {
        self.measurements.push(ProcessMeasurement::new(
            at,
            sim_signal::SignalSpec {
                name: name.into(),
                unit: unit.into(),
            },
            result,
        )?);
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ProcessTick {
    pub at: Time,
    pub completed: Option<PhysicsFrame>,
    pub qpos: Vec<f64>,
    pub qvel: Vec<f64>,
    pub outputs: BTreeMap<String, ProcessOut>,
    pub missing_models: Vec<String>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SolverProfile {
    pub solver: i32,
    pub cone: i32,
    pub iterations: i32,
    pub line_search_iterations: i32,
    pub ccd_iterations: i32,
    pub noslip_iterations: i32,
    pub tolerance: f64,
    pub line_search_tolerance: f64,
    pub ccd_tolerance: f64,
    pub noslip_tolerance: f64,
    pub impedance_ratio: f64,
    pub disable_flags: i32,
    pub enable_flags: i32,
}

#[derive(Clone, Copy)]
pub(crate) struct SceneIdentity {
    fingerprint: u64,
    uncertain_mass: bool,
}

impl SceneIdentity {
    pub(crate) fn new(world: &CompiledWorld, sim: &sim_mujoco::Sim) -> Self {
        let data = (
            &world.name,
            &world.bodies,
            &world.joints,
            &world.efforts,
            &world.drives,
            &world.sensors,
        );
        let opt = &sim.model().opt;
        let medium = (opt.gravity, opt.wind, opt.density, opt.viscosity);
        let geoms: Vec<_> = sim
            .geoms()
            .into_iter()
            .map(|geom| {
                let pose = sim.geom_pose(geom.id);
                (geom, pose)
            })
            .collect();
        let joints: Vec<_> = sim
            .joints()
            .into_iter()
            .map(|joint| {
                let axis = sim.joint_axis(joint.id);
                let anchor = sim.joint_anchor(joint.id);
                (joint, axis, anchor)
            })
            .collect();
        let fingerprint =
            sim_signal::fnv1a(format!("{data:?}{medium:?}{geoms:?}{joints:?}").bytes());
        Self {
            fingerprint,
            uncertain_mass: world.bodies.iter().any(|body| {
                matches!(
                    body.provenance,
                    Provenance::Estimate(_) | Provenance::Derived(_)
                )
            }),
        }
    }
}

pub trait ProcessModel {
    /// Истина доступна моделям мира. Клон модели должен владеть независимым состоянием.
    fn update(
        &mut self,
        frame: &ProcessFrame<'_>,
        rng: &mut Rng,
    ) -> Result<ProcessOut, ProcessError>;

    /// Отпечаток состояния, от которого зависит следующий вызов.
    fn state_fingerprint(&self) -> u64;
}

/// Геометрическая модель радиолинии по нормали к границе жидкости.
#[derive(Clone, Debug, PartialEq)]
pub struct RadioPath {
    pub frequency_hz: SourcedRange,
    pub sensitivity_dbm: SourcedRange,
}

struct StepAdapter<M, I, O> {
    model: M,
    input: I,
    output: O,
}

impl<M, I, O> Clone for StepAdapter<M, I, O>
where
    M: Clone,
    I: Clone,
    O: Clone,
{
    fn clone(&self) -> Self {
        Self {
            model: self.model.clone(),
            input: self.input.clone(),
            output: self.output.clone(),
        }
    }
}

impl<M, I, O> ProcessModel for StepAdapter<M, I, O>
where
    M: StepModel + std::fmt::Debug,
    I: FnMut(&ProcessFrame<'_>) -> Result<Option<(Interval, M::Input)>, ProcessError>,
    O: FnMut(Time, M::Output) -> Result<ProcessOut, ProcessError>,
{
    fn update(
        &mut self,
        frame: &ProcessFrame<'_>,
        rng: &mut Rng,
    ) -> Result<ProcessOut, ProcessError> {
        let Some((interval, input)) = (self.input)(frame)? else {
            return Ok(ProcessOut::default());
        };
        if interval != frame.next && frame.completed.is_none_or(|last| last.interval != interval) {
            return Err(ProcessError::InvalidInput(
                "вход модели относится к чужому шагу".into(),
            ));
        }
        (self.output)(frame.next.start, self.model.advance(interval, &input, rng)?)
    }

    fn state_fingerprint(&self) -> u64 {
        sim_signal::fnv1a(format!("{:?}", self.model).bytes())
    }
}

trait SlotModel: ProcessModel {
    fn duplicate(&self) -> Box<dyn SlotModel>;
}

impl<T: ProcessModel + Clone + 'static> SlotModel for T {
    fn duplicate(&self) -> Box<dyn SlotModel> {
        Box::new(self.clone())
    }
}

impl Clone for Box<dyn SlotModel> {
    fn clone(&self) -> Self {
        self.duplicate()
    }
}

#[derive(Clone)]
struct Slot {
    name: String,
    rng: Rng,
    model: Box<dyn SlotModel>,
}

/// Пары геометрий в контакте и их список пробелов.
type GapCache = (Vec<[String; 2]>, Rc<Vec<String>>);

#[derive(Clone, Default)]
pub(crate) struct Processes {
    models: Vec<Slot>,
    requirements: Option<Vec<HazardEvidence>>,
    bindings: BTreeSet<String>,
    pairs: Option<BTreeMap<[String; 2], ResolvedContactPair>>,
    equalities: BTreeSet<String>,
    welds: BTreeSet<String>,
    completed: Option<PhysicsFrame>,
    uncataloged: Vec<String>,
    /// Пары в контакте прошлого шага и их список пробелов: на сотнях килогерц
    /// пересборка и сортировка строк на каждом шаге занимала треть времени прогона.
    gap_cache: Option<GapCache>,
    pub(crate) journal: Rc<Vec<ProcessTick>>,
    pub(crate) efforts: Vec<(ChannelId, f64)>,
    pub(crate) scalar_hold: bool,
    pub(crate) thermal: BTreeMap<ChannelId, Analysis<ThermalBoundary>>,
    pub(crate) radio: BTreeMap<String, Analysis<RadioEnvironment>>,
    pub(crate) radio_paths: BTreeMap<String, RadioPath>,
    last: Time,
    interaction: Option<interaction::RegisteredInteraction>,
    publications: BTreeMap<TruthId, (String, String)>,
    /// Период хранения журнала, нс. Без него журнал хранит каждую границу физики.
    journal_period_ns: Option<u64>,
}

impl Processes {
    pub(crate) fn last_tick(&self) -> Option<&ProcessTick> {
        self.journal.last()
    }

    /// Добавляет строку журнала. С периодом хранения прошлая строка того же окна
    /// сливается с новой: интервал новой строки начинается там, где начиналась
    /// слитая, журнал остаётся сплошным. Строки с событиями, блокерами, опасностями
    /// и пропусками моделей, начальная строка и последняя строка не сливаются.
    fn push_tick(&mut self, mut tick: ProcessTick) {
        let journal = Rc::make_mut(&mut self.journal);
        if let Some(period) = self.journal_period_ns
            && let Some(last) = journal.last()
            && last.at != Time::ZERO
            && last.at.0 / period == tick.at.0 / period
            && last.missing_models.is_empty()
            && last.outputs.values().all(|out| {
                out.events.is_empty() && out.blockers.is_empty() && out.hazards.is_empty()
            })
            && last.completed.is_some()
            && tick.completed.is_some()
        {
            let dropped = journal.pop().and_then(|row| row.completed).expect("проверена строка");
            let frame = tick.completed.as_mut().expect("проверен интервал");
            frame.interval.start = dropped.interval.start;
            frame.engine_step.start_s = dropped.engine_step.start_s;
            // Реакции и контакты одного шага не описывают слитый интервал.
            frame.reactions.contacts.clear();
            frame.engine_step.contacts.clear();
            frame.equality_reactions.clear();
            frame.weld_reactions.clear();
            if !Rc::ptr_eq(&dropped.contact_gaps, &frame.contact_gaps)
                && dropped.contact_gaps != frame.contact_gaps
            {
                let mut gaps: BTreeSet<String> = dropped.contact_gaps.iter().cloned().collect();
                gaps.extend(frame.contact_gaps.iter().cloned());
                frame.contact_gaps = Rc::new(gaps.into_iter().collect());
            }
        }
        journal.push(tick);
    }

    pub(crate) fn owns_publication(&self, id: TruthId) -> bool {
        self.publications.contains_key(&id)
    }

    pub(crate) fn failure_record(&self, error: &str) -> String {
        format!(
            "Ошибка процесса: {error}; завершённый интервал: {:?}",
            self.completed
        )
    }
}

fn key(mut pair: [String; 2]) -> [String; 2] {
    pair.sort();
    pair
}

fn name(text: &str) -> bool {
    !text.trim().is_empty() && !text.chars().any(char::is_control)
}

impl Runtime {
    /// Включает журнал контактов и проверку реально действующих параметров каждой пары.
    pub fn record_process_contacts(
        &mut self,
        pairs: Vec<ResolvedContactPair>,
    ) -> Result<(), String> {
        self.ensure_configurable()?;
        if self.state.work.pairs.is_some() {
            return Err("пары контакта уже заданы".into());
        }
        let mut table = BTreeMap::new();
        let cone = if self.sim.model().opt.cone as u32 == ffi::mjtCone::mjCONE_ELLIPTIC {
            sim_world::FrictionCone::Elliptic
        } else {
            sim_world::FrictionCone::Pyramidal
        };
        for mut pair in pairs {
            let names = pair.pair.geometries.each_ref().map(String::as_str);
            let settings = pair
                .parameters
                .contact_settings(&pair.calibration, self.physics_rate(), cone)
                .map_err(|e| e.to_string())?;
            if !pair.materials.iter().all(|material| name(material))
                || key(pair.calibration.geometries.clone()) != key(pair.pair.geometries.clone())
                || pair.pair.settings != settings
                || !self.sim.contact_pair_allowed(names)?
                || self.sim.explicit_contact_settings(names)? != Some(pair.pair.settings)
            {
                return Err(format!("пара {names:?} расходится с моделью MuJoCo"));
            }
            if matches!(pair.nominal_coverage, Analysis::Analyzed { .. })
                && !pair.supports_nominal_proof()
            {
                pair.nominal_coverage = Analysis::NotAnalyzed {
                    reason: "номинал не покрывает источники и диапазоны калибровки".into(),
                };
            }
            if table
                .insert(key(pair.pair.geometries.clone()), pair)
                .is_some()
            {
                return Err("повтор пары контакта".into());
            }
        }
        self.state.work.pairs = Some(table);
        Ok(())
    }

    /// Состав включает насадки, ещё не установленные в мире.
    pub fn set_process_inventory(
        &mut self,
        subjects: &[WorkSubject],
        bindings: &[ProcessBinding],
    ) -> Result<(), String> {
        self.ensure_configurable()?;
        if self.state.work.requirements.is_some() {
            return Err("состав процессов уже задан".into());
        }
        self.state.work.requirements =
            Some(required_hazards(subjects, bindings).map_err(|e| e.to_string())?);
        self.state.work.bindings = bindings
            .iter()
            .map(|binding| binding.name.clone())
            .collect();
        Ok(())
    }

    pub fn require_process_hazards(&mut self, hazards: &[HazardEvidence]) -> Result<(), String> {
        self.ensure_configurable()?;
        let existing = self
            .state
            .work
            .requirements
            .as_mut()
            .ok_or("не задан состав процессов")?;
        if hazards.iter().any(|h| !name(&h.subject)) {
            return Err("пустое имя опасности".into());
        }
        for hazard in hazards {
            if !existing
                .iter()
                .any(|h| h.subject == hazard.subject && h.kind == hazard.kind)
            {
                existing.push(HazardEvidence {
                    subject: hazard.subject.clone(),
                    kind: hazard.kind,
                    coverage: Analysis::NotAnalyzed {
                        reason: "нет покрытия в прогоне".into(),
                    },
                });
            }
        }
        Ok(())
    }

    /// Регистрирует снимок реакции ограничения после решения, до следующего forward.
    pub fn record_process_equality(&mut self, name: &str) -> Result<(), String> {
        self.ensure_configurable()?;
        self.sim.equality_active(name)?;
        if !self.state.work.equalities.insert(name.into()) {
            return Err(format!("ограничение «{name}» уже зарегистрировано"));
        }
        Ok(())
    }

    /// Сохраняет знаковую пространственную реакцию до обновления кинематики.
    pub fn record_process_weld(&mut self, name: &str) -> Result<[String; 2], String> {
        self.ensure_configurable()?;
        let bodies = self.sim.weld_bodies(name)?;
        self.record_process_equality(name)?;
        self.state.work.welds.insert(name.into());
        Ok(bodies.map(|id| self.sim.name(ffi::mjtObj::mjOBJ_BODY, id)))
    }

    pub fn validate_scalar_hold(&self, joint: &str) -> Result<(), String> {
        if self.sim.nv() == 1
            && self.sim.nq() == 1
            && self.sim.model().neq == 0
            && self.sim.model().npair == 0
            && self.sim.excluded_contact_candidates()?.is_empty()
            && self.sim.contact_candidates()?.is_empty()
            && self.joints.iter().any(|info| info.name == joint)
        {
            Ok(())
        } else {
            Err("удержание требует единственной скалярной оси без контактов и связей".into())
        }
    }

    /// Хранить журнал процессов строками не чаще одной на `period_ns`: строки внутри
    /// окна сливаются в одну со сплошным интервалом, без контактов и реакций шага.
    /// Модели по-прежнему видят каждую границу физики. Нужен длинным прогонам на
    /// высокой частоте физики: на каждой границе строка весит килобайты.
    pub fn set_process_journal_period(&mut self, period_ns: u64) -> Result<(), String> {
        self.ensure_configurable()?;
        if period_ns == 0 {
            return Err("период журнала процессов должен быть положительным".into());
        }
        self.state.work.journal_period_ns = Some(period_ns);
        Ok(())
    }

    pub fn register_process(
        &mut self,
        label: &str,
        model: impl ProcessModel + Clone + 'static,
    ) -> Result<(), String> {
        self.ensure_configurable()?;
        if !name(label) || self.state.work.models.iter().any(|slot| slot.name == label) {
            return Err(format!("пустое или повторное имя процесса «{label}»"));
        }
        let rng = Rng::stream(self.seed(), &format!("process/{label}"));
        self.state.work.models.push(Slot {
            name: label.into(),
            rng,
            model: Box::new(model),
        });
        Ok(())
    }

    /// Мост для всех StepModel, включая модели следа, тепла, удара, резьбы и электрической цепи.
    pub fn register_step_model<M, I, O>(
        &mut self,
        name: &str,
        model: M,
        input: I,
        output: O,
    ) -> Result<(), String>
    where
        M: StepModel + std::fmt::Debug + 'static,
        I: FnMut(&ProcessFrame<'_>) -> Result<Option<(Interval, M::Input)>, ProcessError>
            + Clone
            + 'static,
        O: FnMut(Time, M::Output) -> Result<ProcessOut, ProcessError> + Clone + 'static,
    {
        self.register_process(
            name,
            StepAdapter {
                model,
                input,
                output,
            },
        )
    }

    /// Публикация для датчика задаётся явно. Неполученный отсчёт сбрасывает последнее значение.
    pub fn publish_process_measurement(
        &mut self,
        signal: &str,
        unit: &str,
    ) -> Result<TruthId, String> {
        let id = self.register_truth(signal, unit)?;
        self.state
            .work
            .publications
            .insert(id, (signal.into(), unit.into()));
        Ok(id)
    }

    pub fn process_journal(&self) -> &[ProcessTick] {
        &self.state.work.journal
    }

    pub fn process_requirements(&self) -> Option<&[HazardEvidence]> {
        self.state.work.requirements.as_deref()
    }

    pub fn process_contact_pairs(&self) -> impl Iterator<Item = &ResolvedContactPair> {
        self.state
            .work
            .pairs
            .iter()
            .flat_map(|pairs| pairs.values())
    }

    pub fn process_scene_fingerprint(&self) -> u64 {
        self.process_scene.fingerprint
    }

    pub fn process_mass_is_unverified(&self) -> bool {
        self.process_scene.uncertain_mass
    }

    pub fn process_run_is_fresh(&self) -> bool {
        !self.state.log.end.started
            && self.state.replay.is_none()
            && crate::control::run(self)
                .is_none_or(|run| run.origin() != crate::control::RunOrigin::Replay)
    }

    pub(crate) fn process_state_hash(&self, physics: u64) -> u64 {
        let work = &self.state.work;
        if work.models.is_empty() {
            return physics;
        }
        let states: Vec<_> = work
            .models
            .iter()
            .map(|slot| (&slot.name, &slot.rng, slot.model.state_fingerprint()))
            .collect();
        sim_signal::fnv1a(
            physics.to_le_bytes().into_iter().chain(
                format!(
                    "{states:?}{:?}{:?}{:?}{}",
                    work.thermal, work.radio, work.efforts, work.scalar_hold,
                )
                .bytes(),
            ),
        )
    }

    pub fn process_solver_profile(&self) -> SolverProfile {
        let opt = &self.sim.model().opt;
        SolverProfile {
            solver: opt.solver,
            cone: opt.cone,
            iterations: opt.iterations,
            line_search_iterations: opt.ls_iterations,
            noslip_iterations: opt.noslip_iterations,
            ccd_iterations: opt.ccd_iterations,
            ccd_tolerance: opt.ccd_tolerance,
            tolerance: opt.tolerance,
            line_search_tolerance: opt.ls_tolerance,
            noslip_tolerance: opt.noslip_tolerance,
            impedance_ratio: opt.impratio,
            disable_flags: opt.disableflags,
            enable_flags: opt.enableflags,
        }
    }

    pub fn matches_creep_configuration(
        &self,
        config: &sim_process::contact::CreepConfiguration,
    ) -> bool {
        use sim_world::{ConstraintSolver, FrictionCone};
        let actual = self.process_solver_profile();
        let solver = config.solver;
        self.physics_rate() == config.rate
            && sim_mujoco::version() == config.engine_version
            && self
                .process_contact_pairs()
                .any(|pair| pair.pair == config.pair)
            && actual.solver as u32
                == match solver.solver {
                    ConstraintSolver::Newton => ffi::mjtSolver::mjSOL_NEWTON,
                    ConstraintSolver::ConjugateGradient => ffi::mjtSolver::mjSOL_CG,
                    ConstraintSolver::ProjectedGaussSeidel => ffi::mjtSolver::mjSOL_PGS,
                }
            && actual.cone as u32
                == match solver.cone {
                    FrictionCone::Elliptic => ffi::mjtCone::mjCONE_ELLIPTIC,
                    FrictionCone::Pyramidal => ffi::mjtCone::mjCONE_PYRAMIDAL,
                }
            && actual.iterations as u32 == solver.iterations
            && actual.ccd_iterations as u32 == solver.ccd_iterations
            && actual.noslip_iterations as u32 == solver.noslip_iterations
            && actual.line_search_iterations as u32 == solver.line_search_iterations
            && actual.tolerance == solver.tolerance
            && actual.impedance_ratio == solver.impratio
            && actual.ccd_tolerance == solver.ccd_tolerance
            && actual.line_search_tolerance == solver.line_search_tolerance
            && actual.noslip_tolerance == solver.noslip_tolerance
    }

    /// Проверяет, что смещение относится к поступательной оси тела опоры
    /// в фиксированной мировой плоскости XY. Возвращает имя подвижной геометрии.
    pub fn creep_displacement_geometry(
        &self,
        position_index: usize,
        pair: &[String; 2],
    ) -> Option<String> {
        use sim_mujoco::JointKind;
        let joint = self
            .joints
            .iter()
            .find(|j| j.qpos_adr == position_index && j.kind == JointKind::Slide)?;
        let m = self.sim.model();
        // Индексы тел и геометрий здесь получены из модели MuJoCo.
        let parent = unsafe { *m.body_parentid.add(joint.body) } as usize;
        if unsafe { *m.body_weldid.add(parent) } != 0 || self.sim.joint_axis(joint.id)[2] != 0.0 {
            return None;
        }
        let bodies = pair.each_ref().map(|name| {
            self.sim
                .geom_id(name)
                .ok()
                .map(|geom| unsafe { *m.geom_bodyid.add(geom) } as usize)
        });
        for side in 0..2 {
            if bodies[side] == Some(joint.body)
                && bodies[1 - side].is_some_and(|body| unsafe { *m.body_weldid.add(body) == 0 })
            {
                return Some(pair[side].clone());
            }
        }
        None
    }

    /// Частоты ввода-вывода и расписание контура, которые не меняют при делении шага.
    pub fn process_sampling_fingerprint(&self) -> u64 {
        let schedule: Vec<_> = self
            .state
            .components
            .iter()
            .filter(|c| {
                matches!(
                    c.phase,
                    sim_signal::Phase::Sensors
                        | sim_signal::Phase::Links
                        | sim_signal::Phase::Controllers
                ) || (c.phase == sim_signal::Phase::Actuators
                    && c.clock.rate() != self.physics_rate())
            })
            .map(|c| (&c.name, c.phase, c.clock.rate()))
            .collect();
        let control = crate::control::run(self).map(|r| {
            let id = r.identity();
            format!("{:?}", (id.control, &id.assets, &id.mode, id.phase_horizon))
        });
        sim_signal::fnv1a(format!("{schedule:?}{control:?}").bytes())
    }

    pub fn bind_process_radio(&mut self, route: &str, path: RadioPath) -> Result<(), String> {
        self.ensure_configurable()?;
        self.state
            .control
            .as_ref()
            .ok_or("контур связи не установлен")?
            .require_modeled_radio(route)?;
        path.frequency_hz.validate().map_err(|e| e.to_string())?;
        path.sensitivity_dbm.validate().map_err(|e| e.to_string())?;
        if path.frequency_hz.min <= 0.0 || self.state.work.radio_paths.contains_key(route) {
            return Err("недопустимая частота или повтор пути в жидкости".into());
        }
        self.add_simplification(Simplification {
            subject: format!("channel:{route}"),
            text: "Потери слоя по нормали учитываются в каждой попытке. Модель пороговая, нужны замеры антенн и вероятности ошибок.".into(),
            blocks_proof: true,
        })?;
        self.state.work.radio_paths.insert(route.into(), path);
        Ok(())
    }

    pub fn process_recording_enabled(&self) -> bool {
        let state = &self.state.work;
        state.pairs.is_some()
            || !state.models.is_empty()
            || !state.equalities.is_empty()
            || state.requirements.is_some()
            || !state.radio_paths.is_empty()
            || !state.publications.is_empty()
    }

    pub(crate) fn capture_process_physics(&mut self) -> Result<(), String> {
        let at = self.time();
        let work = &mut self.state.work;
        let interval = Interval {
            start: work.last,
            end: at,
        };
        let engine_step = self.sim.step_with_contacts()?;
        let reactions =
            ContactFrame::from_engine_step(interval, &engine_step).map_err(|e| e.to_string())?;
        let equality_reactions = work
            .equalities
            .iter()
            .map(|id| Ok((id.clone(), self.sim.equality_reaction(id)?)))
            .collect::<Result<_, String>>()?;
        let weld_reactions = work
            .welds
            .iter()
            .map(|id| Ok((id.clone(), self.sim.weld_reaction(id)?)))
            .collect::<Result<_, String>>()?;
        let mut present: Vec<[String; 2]> = engine_step
            .contacts
            .iter()
            .map(|contact| key(contact.geometries.clone()))
            .collect();
        present.sort();
        present.dedup();
        let cached = work
            .gap_cache
            .as_ref()
            .filter(|(pairs, _)| *pairs == present)
            .map(|(_, gaps)| Rc::clone(gaps));
        let contact_gaps = match cached {
            Some(gaps) => gaps,
            None => {
                let mut gaps: BTreeSet<_> = work.uncataloged.iter().cloned().collect();
                for contact in &engine_step.contacts {
                    let names = key(contact.geometries.clone());
                    let Some(pair) = work.pairs.as_ref().and_then(|map| map.get(&names)) else {
                        gaps.insert(format!("неизвестная пара контактных материалов {names:?}"));
                        continue;
                    };
                    if !contact.matches_settings(pair.pair.settings) {
                        gaps.insert(format!(
                            "фактические параметры контакта {names:?} расходятся с калибровкой"
                        ));
                    }
                    if let Analysis::NotAnalyzed { reason } = &pair.nominal_coverage {
                        gaps.insert(format!("{names:?}: {reason}"));
                    }
                }
                let gaps = Rc::new(gaps.into_iter().collect::<Vec<_>>());
                work.gap_cache = Some((present, Rc::clone(&gaps)));
                gaps
            }
        };
        work.completed = Some(PhysicsFrame {
            interval,
            reactions,
            engine_step,
            equality_reactions,
            weld_reactions,
            contact_gaps,
        });
        Ok(())
    }

    pub(crate) fn prepare_process_boundary(&mut self, at: Time) -> Result<Prepared, String> {
        let Some(end) = self.state.next_physics else {
            return Err("не определена правая граница эффектов процессов".into());
        };
        if !self.state.fresh && !self.state.work.models.is_empty() {
            self.sim.forward();
            self.state.fresh = true;
        }
        if at == Time::ZERO {
            let pairs = &self.state.work.pairs;
            let excluded = self.sim.excluded_contact_candidates()?;
            self.state.work.uncataloged = self
                .sim
                .contact_candidates()?
                .into_iter()
                .map(key)
                .filter(|pair| pairs.as_ref().is_none_or(|known| !known.contains_key(pair)))
                .map(|pair| format!("неизвестная пара контактных материалов {pair:?}"))
                .chain(
                    excluded
                        .into_iter()
                        .map(|pair| format!("исключена потенциально сталкивающаяся пара {pair:?}")),
                )
                .collect();
            self.state.work.gap_cache = None;
        }
        let work = &self.state.work;
        let frame = ProcessFrame {
            truth: Truth::new(
                at,
                &self.sim,
                &self.joints,
                &self.state.plant,
                &[],
                &work.thermal,
                work.last_tick(),
            ),
            completed: work.completed.as_ref(),
            next: Interval { start: at, end },
            observations: &self.state.bus,
            contact_calibrations: work.pairs.as_ref(),
        };
        let mut models = work.models.clone();
        let mut outputs = BTreeMap::new();
        for slot in &mut models {
            let mut out = slot
                .model
                .update(&frame, &mut slot.rng)
                .map_err(|e| format!("процесс «{}»: {e}", slot.name))?;
            if !out.diagnostic_welds.is_empty() {
                out.blockers
                    .push("диагностический weld: работа захвата при совмещении не доказана".into());
            }
            if out.scalar_hold.is_some() {
                out.blockers.push(
                    "идеальное удержание координаты: реакция и работа опоры не рассчитаны".into(),
                );
            }
            outputs.insert(slot.name.clone(), out);
        }
        let stop_after_record = outputs
            .values()
            .find_map(|out| out.stop_after_record.clone());
        let mut batch = Batch::default();
        for out in outputs.values() {
            batch.add(self, out, at)?;
        }
        let plant = work
            .publications
            .iter()
            .map(|(&id, (signal, unit))| {
                let value = match batch.signals.get(signal.as_str()) {
                    Some(item) if item.signal.unit == *unit => match item.result {
                        Analysis::Analyzed { value, .. } => Some(value),
                        Analysis::NotAnalyzed { .. } => None,
                    },
                    Some(_) => {
                        return Err(format!(
                            "неверная единица опубликованного сигнала «{signal}»"
                        ));
                    }
                    None => None,
                };
                Ok((id, value))
            })
            .collect::<Result<Vec<_>, String>>()?;
        if self.state.control.is_some() && batch.radio.keys().ne(work.radio_paths.keys()) {
            return Err("среда радиоканала не соответствует объявленным путям".into());
        }
        if work
            .thermal
            .keys()
            .any(|id| !batch.thermal.contains_key(id))
        {
            return Err("пропущено обновление тепловой границы".into());
        }
        let loads = batch
            .loads
            .into_iter()
            .map(|(body, (force_n, torque_nm))| BodyLoad {
                body: body.into(),
                force_n,
                torque_nm,
            })
            .collect();
        let switches = batch
            .switches
            .into_iter()
            .map(|(name, active)| (name.to_owned(), active))
            .collect();
        Ok(Prepared {
            models,
            loads,
            switches,
            diagnostic_welds: batch
                .diagnostic_welds
                .into_iter()
                .map(str::to_owned)
                .collect(),
            efforts: batch.efforts,
            scalar_hold: batch.scalar_hold,
            thermal: batch.thermal,
            radio: batch.radio,
            plant,
            outputs,
            stop_after_record,
        })
    }

    pub(crate) fn commit_process_boundary(
        &mut self,
        at: Time,
        mut prepared: Prepared,
    ) -> Result<(), String> {
        // Тепловая граница уже была доступна приводам. Ни сила на теле,
        // ни переключение равенства не вступают в силу при ошибке общей суммы.
        if prepared.stop_after_record.is_some() {
            for out in prepared.outputs.values_mut() {
                out.efforts.clear();
                out.body_loads.clear();
                if let Some(tick) = &mut out.interaction {
                    tick.axes.clear();
                    tick.motion_events.clear();
                }
            }
        } else {
            let mut combined = vec![0.0; self.sim.nv()];
            super::sum_forces(
                &self.state,
                &self.effort_dofs,
                &prepared.efforts,
                &mut combined,
            )?;
            if !prepared.models.is_empty() {
                let loads: Vec<_> = prepared
                    .loads
                    .iter()
                    .map(|load| BodyWrench {
                        body: &load.body,
                        force_n: load.force_n,
                        torque_nm: load.torque_nm,
                    })
                    .collect();
                let switches: Vec<_> = prepared
                    .switches
                    .iter()
                    .map(|(name, active)| EqualitySwitch {
                        name,
                        active: *active,
                    })
                    .collect();
                let diagnostic: Vec<_> = prepared
                    .diagnostic_welds
                    .iter()
                    .map(String::as_str)
                    .collect();
                self.sim
                    .apply_step_effects_diagnostic(&loads, &switches, &diagnostic)?;
                if let Some(view) = &mut self.state.view
                    && !switches.is_empty()
                {
                    Rc::make_mut(&mut view.equality).extend(switches.iter().map(|switch| {
                        crate::ViewEquality {
                            at,
                            name: switch.name.to_owned(),
                            active: switch.active,
                        }
                    }));
                }
            }
        }
        let work = &mut self.state.work;
        work.models = prepared.models;
        work.efforts = prepared.efforts;
        work.scalar_hold = prepared.scalar_hold;
        work.thermal = prepared.thermal;
        work.radio = prepared.radio;
        work.last = at;
        let tick = ProcessTick {
            at,
            completed: work.completed.take(),
            qpos: self.sim.qpos().to_vec(),
            qvel: self.sim.qvel().to_vec(),
            missing_models: work
                .bindings
                .iter()
                .filter(|name| !prepared.outputs.contains_key(*name))
                .cloned()
                .collect(),
            outputs: prepared.outputs,
        };
        if at == Time::ZERO {
            if let Some(interaction) = &work.interaction {
                self.state.log.record_process(
                    at,
                    format!(
                        "Взаимодействие: {:?}. Движение: {}",
                        interaction.identity, interaction.motion
                    ),
                );
            }
            self.state.log.record_process(
                at,
                format!(
                    "Состав: {:?}; материалы: {:?}; радиолинии: {:?}",
                    work.requirements, work.pairs, work.radio_paths
                ),
            );
        }
        work.push_tick(tick);
        if let Some(error) = prepared.stop_after_record {
            return Err(error);
        }
        for (id, value) in prepared.plant {
            self.state.plant[id.0 as usize] = value;
            if let Some(value) = value {
                self.state.log.push(crate::LogEntry {
                    at,
                    kind: crate::LogKind::Truth,
                    id: id.0,
                    value,
                    valid: true,
                });
            }
        }
        Ok(())
    }
}

pub(crate) struct Prepared {
    models: Vec<Slot>,
    loads: Vec<BodyLoad>,
    switches: Vec<(String, bool)>,
    diagnostic_welds: Vec<String>,
    efforts: Vec<(ChannelId, f64)>,
    scalar_hold: bool,
    pub(crate) thermal: BTreeMap<ChannelId, Analysis<ThermalBoundary>>,
    radio: BTreeMap<String, Analysis<RadioEnvironment>>,
    plant: Vec<(TruthId, Option<f64>)>,
    outputs: BTreeMap<String, ProcessOut>,
    stop_after_record: Option<String>,
}

#[derive(Default)]
struct Batch<'a> {
    loads: BTreeMap<&'a str, ([f64; 3], [f64; 3])>,
    efforts: Vec<(ChannelId, f64)>,
    scalar_hold: bool,
    sum: BTreeMap<usize, f64>,
    switches: BTreeMap<&'a str, bool>,
    diagnostic_welds: BTreeSet<&'a str>,
    signals: BTreeMap<&'a str, &'a ProcessMeasurement>,
    states: BTreeSet<&'a str>,
    attachments: BTreeSet<&'a str>,
    events: BTreeSet<&'a str>,
    thermal: BTreeMap<ChannelId, Analysis<ThermalBoundary>>,
    radio: BTreeMap<String, Analysis<RadioEnvironment>>,
}

impl<'a> Batch<'a> {
    fn add(&mut self, runtime: &Runtime, out: &'a ProcessOut, at: Time) -> Result<(), String> {
        if let Some(joint) = &out.scalar_hold {
            if self.scalar_hold {
                return Err("повторное назначение удержания оси".into());
            }
            runtime.validate_scalar_hold(joint)?;
            self.scalar_hold = true;
        }
        runtime.check_targets(&out.efforts)?;
        for &(id, value) in &out.efforts {
            let sum = self
                .sum
                .entry(runtime.effort_dofs[id.0 as usize])
                .or_default();
            *sum += value;
            if !sum.is_finite() {
                return Err("сумма сил процессов переполнена".into());
            }
            self.efforts.push((id, value));
        }
        let mut bodies = BTreeSet::new();
        for load in &out.body_loads {
            if !name(&load.body) || !bodies.insert(&load.body) {
                return Err(format!(
                    "неоднозначная нагрузка процесса на «{}»",
                    load.body
                ));
            }
            let (force, torque) = self.loads.entry(&load.body).or_default();
            for (sum, delta) in force
                .iter_mut()
                .chain(torque)
                .zip(load.force_n.into_iter().chain(load.torque_nm))
            {
                *sum += delta;
                if !sum.is_finite() {
                    return Err("неконечная нагрузка на тело".into());
                }
            }
        }
        for item in &out.measurements {
            ProcessMeasurement::new(item.at, item.signal.clone(), item.result.clone())
                .map_err(|e| e.to_string())?;
            if item.at != at || self.signals.insert(&item.signal.name, item).is_some() {
                return Err(format!(
                    "неверное время или конфликт записи «{}»",
                    item.signal.name
                ));
            }
        }
        for item in &out.attachments {
            if !self.attachments.insert(&item.name) {
                return Err(format!(
                    "повторный писатель состояния крепления «{}»",
                    item.name
                ));
            }
            if let Some((id, active)) = &item.applied_limit
                && self.switches.insert(id, *active).is_some()
            {
                return Err(format!("повторный писатель ограничения «{id}»"));
            }
        }
        self.diagnostic_welds
            .extend(out.diagnostic_welds.iter().map(String::as_str));
        for (id, active) in &out.equality_switches {
            if !runtime.state.work.equalities.contains(id)
                || self.switches.insert(id, *active).is_some()
            {
                return Err(format!(
                    "незарегистрированное или конфликтующее ограничение «{id}»"
                ));
            }
        }
        for (key, value) in &out.states {
            if !name(key) || !name(value) || !self.states.insert(key) {
                return Err(format!("недопустимое или повторное состояние «{key}»"));
            }
        }
        for event in &out.events {
            if out.states.get(&event.name) != Some(&event.state)
                || !self.events.insert(&event.name)
                || event.requested > event.applied
                || event.applied != at
            {
                return Err(format!("невалидное событие «{}»", event.name));
            }
        }
        if out.blockers.iter().any(|reason| !name(reason)) {
            return Err("пустая причина недоступности модели".into());
        }
        for hazard in &out.hazards {
            let description = match &hazard.check {
                HazardCheck::NotAnalyzed { reason } | HazardCheck::Violation { reason } => reason,
                HazardCheck::WithinLimit { scope } => scope,
            };
            if !name(&hazard.subject) || !name(description) {
                return Err("невалидный вывод об опасности".into());
            }
            for value in &hazard.sources {
                value.validate().map_err(|e| e.to_string())?;
            }
        }
        for (&id, boundary) in &out.thermal {
            runtime.check_channel(id)?;
            if matches!(boundary, Analysis::Analyzed { value: ThermalBoundary::Wet {
                fluid_temperature_k, conductance_w_k, heat_out_w }, .. }
                if !fluid_temperature_k.is_finite() || *fluid_temperature_k <= 0.0
                    || !conductance_w_k.is_finite() || *conductance_w_k <= 0.0 || !heat_out_w.is_finite())
                || !valid_analysis(boundary)
                || self.thermal.insert(id, boundary.clone()).is_some()
            {
                return Err("недопустимая или конфликтующая тепловая граница".into());
            }
        }
        for (id, radio) in &out.radio {
            if !name(id)
                || !valid_analysis(radio)
                || matches!(radio,
                Analysis::Analyzed { value, .. } if !value.depth_m.is_finite() || value.depth_m < 0.0
                    || ((value.medium == sim_radio::dielectric::Medium::Air) != (value.depth_m == 0.0))
    || matches!(value.medium, sim_radio::dielectric::Medium::FreshWater { t_c }
                    | sim_radio::dielectric::Medium::SeaWater { t_c } if !t_c.is_finite()))
                || self.radio.insert(id.clone(), radio.clone()).is_some()
            {
                return Err("недопустимая или конфликтующая среда радиоканала".into());
            }
        }
        Ok(())
    }
}

fn valid_analysis<T>(analysis: &Analysis<T>) -> bool {
    match analysis {
        Analysis::Analyzed { scope, .. } => name(scope),
        Analysis::NotAnalyzed { reason } => name(reason),
    }
}
