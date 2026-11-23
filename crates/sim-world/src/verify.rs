use std::collections::BTreeSet;
use std::panic::{AssertUnwindSafe, catch_unwind};

use sim_mujoco::ffi::mjtObj;
use sim_mujoco::skeleton::{EngineCheck, Skeleton, cross_check};
use sim_mujoco::{GeomKind, Sim};

use crate::{CompiledWorld, physics};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CheckStatus {
    Verified,
    Mismatch(String),
    NotAnalyzed(String),
}

#[derive(Clone, Debug)]
pub struct CompileCheck {
    /// Масса, центр масс и главные моменты каждого тела.
    pub inertial: CheckStatus,
    /// Сопоставимость модели целиком для динамической сверки.
    pub dynamics: CheckStatus,
    /// M и C+G без реакций ограничений. Может присутствовать при неполной сверке сцены.
    pub engine: Option<EngineCheck>,
}

// Запас округления для суммирования, смены базиса и диагонализации у обоих движков.
const ROUNDING: f64 = 512.0 * f64::EPSILON;
// Координаты сеток хранятся в MuJoCo в float32. Запас для интегралов по тетраэдрам.
const MESH_ROUNDING: f64 = 64.0 * f32::EPSILON as f64;

pub(crate) fn check_bodies(sim: &Sim, world: &CompiledWorld) -> Result<bool, String> {
    if sim.nbody().checked_sub(1) != Some(world.bodies.len()) {
        return Err(format!(
            "число тел: в данных {}, в MuJoCo {}",
            world.bodies.len(),
            sim.nbody().saturating_sub(1)
        ));
    }
    let models_with_mesh: BTreeSet<_> = sim
        .geoms()
        .iter()
        .filter(|g| g.group != 1 && g.kind == GeomKind::Mesh)
        .map(|g| g.body)
        .collect();
    let mut names = BTreeSet::new();
    let mut complete = true;
    for b in &world.bodies {
        if !names.insert(b.name.as_str()) {
            return Err(format!("повтор тела «{}» в данных", b.name));
        }
        let id = sim
            .id(mjtObj::mjOBJ_BODY, &b.name)
            .ok_or_else(|| format!("в MuJoCo нет тела «{}»", b.name))?;
        if id == 0 {
            return Err("служебное тело мира не входит в список звеньев".into());
        }
        let mass = sim.body_mass(id);
        let mut inertia = sim.body_inertia(id);
        // SAFETY: body_ipos имеет 3*nbody элементов. id проверен по индексу модели.
        let com: [f64; 3] =
            unsafe { std::slice::from_raw_parts(sim.model().body_ipos.add(3 * id), 3) }
                .try_into()
                .unwrap();
        physics::nonnegative(&[b.mass_kg, mass], "mass_kg")?;
        physics::nonnegative(&inertia, "MuJoCo inertia_kg_m2")?;
        physics::nonnegative(&b.inertia_kg_m2, "inertia_kg_m2")?;
        physics::finite(&com, "MuJoCo com_m")?;
        physics::finite(&b.com_m, "com_m")?;
        let mut expected = b.inertia_kg_m2;
        inertia.sort_by(f64::total_cmp);
        expected.sort_by(f64::total_cmp);
        let eps = if models_with_mesh.contains(&id) {
            MESH_ROUNDING
        } else {
            ROUNDING
        };
        let diff = |a: f64, b: f64, scale: f64| (a - b).abs() > eps * scale;
        let radius = if mass == 0.0 {
            0.0
        } else {
            inertia[2].sqrt() / mass.sqrt()
        };
        let com_scale = radius.max(
            com.iter()
                .chain(&b.com_m)
                .map(|v| v.abs())
                .fold(0.0, f64::max),
        );
        let report = |field: &str, actual: &dyn std::fmt::Debug, data: &dyn std::fmt::Debug| {
            Err(format!(
                "{}.{}: MuJoCo {actual:?}, данные {data:?}",
                b.name, field
            ))
        };
        if diff(mass, b.mass_kg, mass.max(b.mass_kg)) {
            return report("mass_kg", &mass, &b.mass_kg);
        }
        // Для безмассового кадра центр масс физически не определён.
        if mass > 0.0 && com.iter().zip(b.com_m).any(|(&a, b)| diff(a, b, com_scale)) {
            return report("com_m", &com, &b.com_m);
        }
        if inertia
            .iter()
            .zip(expected)
            .any(|(&a, b)| diff(a, b, a.max(b)))
        {
            return report("inertia_kg_m2", &inertia, &expected);
        }
        let Some(n) = sim.id(mjtObj::mjOBJ_NUMERIC, &format!("inertia/{}", b.name)) else {
            complete = false;
            continue;
        };
        let model = sim.model();
        // SAFETY: n и id получены из текущей модели. MuJoCo задаёт размеры и адреса
        // числовых полей. Срезы заимствуются только на время сверки.
        let (values, q): ([f64; 6], [f64; 4]) = unsafe {
            let len = *model.numeric_size.add(n);
            let offset = *model.numeric_adr.add(n);
            if len != 6 || offset < 0 || offset as usize + 6 > model.nnumericdata as usize {
                return Err(format!("{}.inertia: повреждён контрольный тензор", b.name));
            }
            (
                std::slice::from_raw_parts(model.numeric_data.add(offset as usize), 6)
                    .try_into()
                    .unwrap(),
                std::slice::from_raw_parts(model.body_iquat.add(4 * id), 4)
                    .try_into()
                    .unwrap(),
            )
        };
        physics::finite(&values, "контрольный тензор")?;
        let rotation = physics::frame(&crate::Pose {
            quat_wxyz: Some(q),
            ..crate::Pose::default()
        })?
        .rotation
        .to_rotation_matrix();
        // body_inertia выражена в главных осях, восстанавливаем тензор в кадре звена.
        let diag = nalgebra::Matrix3::from_diagonal(&sim.body_inertia(id).into());
        let actual = rotation.matrix() * diag * rotation.matrix().transpose();
        let actual = physics::tensor_values(&actual);
        let scale = values
            .iter()
            .chain(&actual)
            .map(|x| x.abs())
            .fold(0.0, f64::max);
        if values.iter().zip(actual).any(|(&a, b)| diff(a, b, scale)) {
            return report("inertia_tensor", &actual, &values);
        }
    }
    Ok(complete)
}

/// Сверяет инерции и независимую динамику. Ограничения контакта и троса не входят в M и C+G.
pub fn verify_compiled(world: &CompiledWorld) -> CompileCheck {
    let mut sim = match Sim::from_xml(&world.mjcf) {
        Ok(sim) if sim.numerically_ok() => sim,
        Ok(_) => {
            return CompileCheck {
                inertial: CheckStatus::NotAnalyzed("Численная авария в начальной позе".into()),
                dynamics: CheckStatus::NotAnalyzed("Численная авария в начальной позе".into()),
                engine: None,
            };
        }
        Err(error) => {
            return CompileCheck {
                inertial: CheckStatus::Mismatch(error),
                dynamics: CheckStatus::NotAnalyzed("MuJoCo не создал модель".into()),
                engine: None,
            };
        }
    };
    let inertial = match check_bodies(&sim, world) {
        Ok(true) if world.bodies.is_empty() => {
            CheckStatus::NotAnalyzed("нет тел для сверки".into())
        }
        Ok(true) => CheckStatus::Verified,
        Ok(false) => {
            CheckStatus::NotAnalyzed("В MJCF нет контрольного тензора в кадре звена".into())
        }
        Err(error) => CheckStatus::Mismatch(error),
    };
    let (dynamics, engine) = check_dynamics(&mut sim, world);
    CompileCheck {
        inertial,
        dynamics,
        engine,
    }
}

fn check_dynamics(sim: &mut Sim, world: &CompiledWorld) -> (CheckStatus, Option<EngineCheck>) {
    let missing = |reason| (CheckStatus::NotAnalyzed(reason), None);
    if world.physics.as_hz().recip() != sim.timestep() {
        return (
            CheckStatus::Mismatch("шаг MuJoCo расходится с частотой мира".into()),
            None,
        );
    }
    if sim.nv() == 0 {
        return missing("в сцене нет степеней свободы".into());
    }
    let sk = match Skeleton::from_mujoco(sim) {
        Ok(sk) => sk,
        Err(error) => return missing(error),
    };
    let position_scale = (0..sim.nbody())
        .flat_map(|i| sim.body_pose(i).0)
        .map(f64::abs)
        .fold(1.0, f64::max);
    // 32 детерминированных положения: выборочная проверка, покрытие всего диапазона не утверждается.
    const SAMPLES: usize = 32;
    let check = match catch_unwind(AssertUnwindSafe(|| {
        cross_check(sim, &sk, SAMPLES, world.seed)
    })) {
        Ok(check) => check,
        Err(_) => {
            return (
                CheckStatus::Mismatch("численная сверка движков прервана".into()),
                None,
            );
        }
    };
    // Сравнение M и C+G учитывает накопление округления вдоль дерева. При C+G≈0
    // абсолютный остаток выражен в СИ и изолирован от деления на шум.
    const DYNAMICS_ROUNDING: f64 = 65536.0 * f64::EPSILON;
    let quality = [
        check.max_pos_err,
        check.max_rot_err,
        check.max_mass_rel,
        check.max_bias_abs,
        check.bias_scale,
    ];
    let status = if check.samples != SAMPLES
        || quality.iter().any(|v| !v.is_finite() || *v < 0.0)
        || check.max_pos_err > DYNAMICS_ROUNDING * position_scale
        || check.max_rot_err > DYNAMICS_ROUNDING
        || check.max_mass_rel > DYNAMICS_ROUNDING
        || check.max_bias_abs > DYNAMICS_ROUNDING * check.bias_scale.max(1.0)
    {
        CheckStatus::Mismatch(format!("Расхождение кинематики, M или C+G: {check:?}"))
    } else if sim.model().neq > 0 || sim.ntendon() > 0 {
        CheckStatus::NotAnalyzed(
            "M и C+G совпали; реакции связей и тросов отсутствуют во втором движке".into(),
        )
    } else if sim.geoms().iter().any(|g| g.group == 0)
        || sim
            .contact_candidates()
            .map_or(true, |pairs| !pairs.is_empty())
    {
        CheckStatus::NotAnalyzed("M и C+G совпали; контакт отсутствует во втором движке".into())
    } else if sim.model().opt.density > 0.0 || sim.model().opt.viscosity > 0.0 {
        CheckStatus::NotAnalyzed("M и C+G совпали; силы среды отсутствуют во втором движке".into())
    } else {
        CheckStatus::Verified
    };
    (status, Some(check))
}

/// Включает или выключает по короткому имени weld, созданный из записи world.attachments.
pub fn set_attachment_active(sim: &mut Sim, name: &str, active: bool) -> Result<(), String> {
    let full = format!("attachment/{name}");
    sim.weld_bodies(&full)?;
    sim.set_step_equality(&full, active)?;
    sim.forward();
    Ok(())
}
