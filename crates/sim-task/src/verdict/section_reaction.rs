//! Реакция опоры по завершённому интервалу MuJoCo.

use std::{ptr, rc::Rc, slice};

use sim_mujoco::{JointInfo, Sim, ffi};
use sim_runtime::{Evaluator, Runtime, Truth, TruthId, TruthOut};
use sim_signal::{Rng, Time};

use super::section_span::Motion;

pub(super) fn channels(object: &str) -> [crate::Signal; 6] {
    ["fx", "fy", "fz", "mx", "my", "mz"].map(|component| {
        let unit = if component.starts_with('f') {
            "N"
        } else {
            "N*m"
        };
        crate::Signal {
            name: format!("section_reaction/{object}/{component}"),
            unit: unit.into(),
        }
    })
}

pub(super) fn motion_channels(object: &str, sliding: bool) -> [crate::Signal; 6] {
    let (speed, acceleration) = if sliding {
        ("m/s", "m/s^2")
    } else {
        ("rad/s", "rad/s^2")
    };
    [
        ("speed", speed),
        ("acceleration", acceleration),
        ("gx", "m/s^2"),
        ("gy", "m/s^2"),
        ("gz", "m/s^2"),
        ("mass", "kg"),
    ]
    .map(|(component, unit)| crate::Signal {
        name: format!("section_motion/{object}/{component}"),
        unit: unit.into(),
    })
}

pub(super) fn angular_channels(object: &str) -> [crate::Signal; 3] {
    ["x", "y", "z"].map(|axis| crate::Signal {
        name: format!("section_rotation/{object}/{axis}"),
        unit: "rad/s".into(),
    })
}

pub(super) fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] - b[i])
}

pub(super) fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[(i + 1) % 3] * b[(i + 2) % 3] - a[(i + 2) % 3] * b[(i + 1) % 3])
}

#[derive(Clone, Copy)]
pub(super) struct Wrench {
    pub force: [f64; 3],
    pub moment: [f64; 3],
}

struct StepData(*mut ffi::mjData);

impl Drop for StepData {
    fn drop(&mut self) {
        // Единственный владелец памяти, выделенной mj_copyData.
        unsafe { ffi::mj_deleteData(self.0) }
    }
}

impl StepData {
    fn copy(model: &ffi::mjModel, data: &ffi::mjData) -> Result<Self, String> {
        // MuJoCo выделяет независимый mjData для этой модели.
        let raw = unsafe { ffi::mj_copyData(ptr::null_mut(), model, data) };
        if raw.is_null() {
            return Err("MuJoCo не выделил данные для проверки реакции".into());
        }
        Ok(Self(raw))
    }

    fn restore_from(&mut self, engine: &Sim) {
        // Данные принадлежат тому же формату модели, в том числе в ветви рантайма.
        unsafe { ffi::mj_copyData(self.0, engine.model(), engine.data()) };
    }

    fn solve(&mut self, engine: &Sim) -> Result<(), String> {
        // qfrc_applied заполняется непосредственно перед интегрированием.
        // Его журналируемое значение на правой границе ещё принадлежит этому
        // шагу. Поза, скорость, внешние нагрузки и связи берутся слева.
        let model = engine.model();
        let live = unsafe { slice::from_raw_parts(engine.data().qfrc_applied, engine.nv()) };
        if !live.iter().all(|x| x.is_finite()) {
            return Err("неконечное приложенное усилие за интервал".into());
        }
        // Обе структуры данных соответствуют модели; копия доступна только оценщику.
        unsafe {
            ptr::copy_nonoverlapping(live.as_ptr(), (*self.0).qfrc_applied, live.len());
            ffi::mj_forward(model, self.0);
            ffi::mj_rnePostConstraint(model, self.0);
        }
        Ok(())
    }

    fn wrench(&self, model: &ffi::mjModel, joint: &JointInfo) -> Wrench {
        // Идентификаторы получены из той же модели. cfrc_int — момент и сила
        // со стороны родителя относительно центра масс кинематического дерева.
        // См. MuJoCo API, mj_rnePostConstraint и mjData.
        unsafe {
            let data = &*self.0;
            let vec_at = |ptr: *const f64, index| *ptr.add(index).cast::<[f64; 3]>();
            let root = *model.body_rootid.add(joint.body) as usize;
            let origin = vec_at(data.subtree_com, 3 * root);
            let anchor = vec_at(data.xanchor, 3 * joint.id);
            let force = vec_at(data.cfrc_int, 6 * joint.body + 3);
            let moment = vec_at(data.cfrc_int, 6 * joint.body);
            let shift = cross(sub(origin, anchor), force);
            let rotation = data.xmat.add(9 * joint.body);
            // Перенос момента к якорю и умножение на транспонированную
            // матрицу поворота тела: обе тройки теперь в кадре звена.
            let local = |v: [f64; 3]| {
                std::array::from_fn(|i| (0..3).map(|j| *rotation.add(3 * j + i) * v[j]).sum())
            };
            Wrench {
                force: local(force),
                moment: local(std::array::from_fn(|i| moment[i] + shift[i])),
            }
        }
    }

    fn motion(&self, model: &ffi::mjModel, joint: &JointInfo) -> Result<Option<Motion>, String> {
        // У единственного тела с одним сочленением линейное ускорение
        // ползуна на оси равно qacc. У шарнира якорь неподвижен, ω и α
        // вдоль оси равны qvel и qacc. Контакт или внешняя сила вне
        // якоря требуют известного распределения по отрезанным частям.
        unsafe {
            let data = &*self.0;
            if model.nbody != 2 || model.njnt != 1 || model.neq != 0 || data.ncon != 0 {
                return Ok(None);
            }
            let applied = slice::from_raw_parts(data.xfrc_applied.add(6 * joint.body), 6);
            if !applied.iter().all(|v| v.is_finite()) {
                return Err("неконечная внешняя нагрузка звена".into());
            }
            if applied.iter().any(|&v| v != 0.0) {
                return Ok(None);
            }
            let rotation = data.xmat.add(9 * joint.body);
            let gravity = if model.opt.disableflags as u32 & ffi::mjtDisableBit::mjDSBL_GRAVITY == 0
            {
                std::array::from_fn(|i| {
                    (0..3)
                        .map(|j| *rotation.add(3 * j + i) * model.opt.gravity[j])
                        .sum()
                })
            } else {
                [0.0; 3]
            };
            Ok(Some(Motion {
                speed: *data.qvel.add(joint.dof_adr),
                acceleration: *data.qacc.add(joint.dof_adr),
                gravity,
                mass: *model.body_mass.add(joint.body),
            }))
        }
    }

    fn matches_step(&mut self, engine: &Sim) -> bool {
        // MuJoCo Euler после того же решателя должен воспроизвести переход
        // побитово. Кинематическая фиксация вне решателя даёт разрыв ряда.
        unsafe {
            ffi::mj_Euler(engine.model(), self.0);
            let data = &*self.0;
            let same = |ptr: *const f64, live: &[f64]| {
                slice::from_raw_parts(ptr, live.len())
                    .iter()
                    .zip(live)
                    .all(|(a, b)| a.to_bits() == b.to_bits())
            };
            data.time.to_bits() == engine.time().to_bits()
                && same(data.qpos, engine.qpos())
                && same(data.qvel, engine.qvel())
        }
    }
}

#[derive(Clone)]
struct Recorder {
    targets: Vec<Target>,
    rotating_bodies: Vec<(usize, [TruthId; 3])>,
    previous: Option<Rc<StepData>>,
}

#[derive(Clone)]
struct Target {
    joint: JointInfo,
    reaction: [TruthId; 6],
    motion: [TruthId; 6],
}

fn record<const N: usize>(
    out: &mut TruthOut,
    ids: [TruthId; N],
    values: impl IntoIterator<Item = f64>,
) -> Result<(), String> {
    for (id, value) in ids.into_iter().zip(values) {
        if !value.is_finite() {
            return Err("неконечное значение расчёта сечения".into());
        }
        out.write(id, value);
    }
    Ok(())
}

impl Evaluator for Recorder {
    fn evaluate(
        &mut self,
        truth: &Truth<'_>,
        _: &mut Rng,
        out: &mut TruthOut,
    ) -> Result<(), String> {
        let engine = truth.engine();
        let model = engine.model();
        // Скорость тела относится к текущему состоянию, включая ноль и
        // правую границу последнего шага. Она включает движение предков
        // и доступна независимо от применимости решателя реакции.
        for &(body, ids) in &self.rotating_bodies {
            let mut spatial = [0.0; 6];
            // Рантайм обновил кинематику и скорости перед вызовом оценщика.
            // XBODY задаёт локальный кадр звена, а не главные оси инерции.
            unsafe {
                ffi::mj_objectVelocity(
                    model, engine.data(), ffi::mjtObj::mjOBJ_XBODY as i32,
                    body as i32, spatial.as_mut_ptr(), 1,
                );
            }
            record(out, ids, spatial[..3].iter().copied())?;
        }
        if self.targets.is_empty() {
            return Ok(());
        }
        // MuJoCo не разносит по телам все внешние силы. Для полной реакции
        // принимаем жёсткие тела, вакуум и воздействия через сочленения
        // либо явно заданные силы на телах. См. документацию mj_rnePostConstraint.
        let gravity_compensation =
            (0..engine.nbody()).any(|i| unsafe { *model.body_gravcomp.add(i) != 0.0 });
        let other_actuation = (0..engine.nactuator()).any(|i| {
            let transmission = unsafe { *model.actuator_trntype.add(i) } as u32;
            !matches!(
                transmission,
                ffi::mjtTrn::mjTRN_JOINT | ffi::mjtTrn::mjTRN_JOINTINPARENT
            )
        });
        if model.opt.integrator != ffi::mjtIntegrator::mjINT_EULER as i32
            || model.nflex != 0
            || model.nplugin != 0
            || model.ntendon != 0
            || model.opt.density != 0.0
            || model.opt.viscosity != 0.0
            || gravity_compensation
            || other_actuation
            || !engine.numerically_ok()
        {
            self.previous = None;
            return Ok(());
        }

        // Рантайм обновляет кинематику перед оценщиком. Поэтому на правой
        // границе исходный расчёт ускорений и контактов уже недоступен.
        if let Some(mut previous) = self.previous.take() {
            if Rc::get_mut(&mut previous).is_none() {
                // Снимок может владеть левой границей. Его данные неизменяемы.
                previous = Rc::new(StepData::copy(model, unsafe { &*previous.0 })?);
            }
            let step = Rc::get_mut(&mut previous).expect("частные данные шага");
            step.solve(engine)?;
            let reactions: Vec<_> = self
                .targets
                .iter()
                .map(|target| {
                    Ok((
                        step.wrench(model, &target.joint),
                        step.motion(model, &target.joint)?,
                    ))
                })
                .collect::<Result<_, String>>()?;
            if step.matches_step(engine) {
                for (target, (reaction, motion)) in self.targets.iter().zip(reactions) {
                    record(
                        out,
                        target.reaction,
                        reaction.force.into_iter().chain(reaction.moment),
                    )?;
                    if let Some(Motion {
                        speed,
                        acceleration,
                        gravity: [gx, gy, gz],
                        mass,
                    }) = motion
                    {
                        record(out, target.motion, [speed, acceleration, gx, gy, gz, mass])?;
                    }
                }
            }
            step.restore_from(engine);
            self.previous = Some(previous);
        } else if truth.time() == Time::ZERO {
            self.previous = Some(Rc::new(StepData::copy(model, engine.data())?));
        }
        Ok(())
    }
}

fn register_channels<const N: usize>(
    runtime: &mut Runtime,
    signals: [crate::Signal; N],
) -> Result<[TruthId; N], String> {
    let ids = signals
        .into_iter()
        .map(|signal| runtime.register_truth(&signal.name, &signal.unit))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(ids.try_into().expect("число зарегистрированных каналов"))
}

pub(super) fn register(
    runtime: &mut Runtime,
    joints: Vec<JointInfo>,
    bodies: &[(String, usize)],
) -> Result<(), String> {
    if joints.is_empty() && bodies.is_empty() {
        return Ok(());
    }
    let mut targets = Vec::with_capacity(joints.len());
    for joint in joints {
        targets.push(Target {
            reaction: register_channels(runtime, channels(&joint.name))?,
            motion: register_channels(runtime, motion_channels(
                &joint.name,
                joint.kind == sim_mujoco::JointKind::Slide,
            ))?,
            joint,
        });
    }
    let rotating_bodies = bodies.iter().map(|(name, id)| {
        Ok((*id, register_channels(runtime, angular_channels(name))?))
    }).collect::<Result<_, String>>()?;
    runtime.register_evaluator(
        "section_reaction",
        runtime.physics_rate(),
        Recorder {
            targets,
            rotating_bodies,
            previous: None,
        },
    )
}
