//! Явные параметры контакта и добавляемые элементы сцены. Старый путь сборки
//! сохраняет прежние настройки. Пакет параметров входит в MJCF через сборщик.

use std::collections::{BTreeMap, BTreeSet};

use sim_mujoco::{Sim, ffi};
use sim_signal::Rate;

use crate::{Attachment, CompiledWorld, Object};

pub use sim_mujoco::contact::{
    ContactSettings, ContactStep, MAX_CONTACT_IMPEDANCE, MIN_CONTACT_FRICTION,
    MIN_CONTACT_IMPEDANCE,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrictionCone {
    Elliptic,
    Pyramidal,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConstraintSolver {
    Newton,
    ConjugateGradient,
    ProjectedGaussSeidel,
}

/// Настройки численного опыта. Шаг задаётся отдельно через Rate и пересборку.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactSolver {
    pub cone: FrictionCone,
    pub solver: ConstraintSolver,
    pub impratio: f64,
    pub tolerance: f64,
    pub iterations: u32,
    pub line_search_tolerance: f64,
    pub line_search_iterations: u32,
    pub noslip_tolerance: f64,
    pub noslip_iterations: u32,
    pub ccd_tolerance: f64,
    pub ccd_iterations: u32,
}

impl ContactSolver {
    pub fn validate(self) -> Result<(), String> {
        if !self.impratio.is_finite()
            || self.impratio < 1.0
            || ![
                self.tolerance,
                self.line_search_tolerance,
                self.noslip_tolerance,
                self.ccd_tolerance,
            ]
            .into_iter()
            .all(|x| x.is_finite() && x >= 0.0)
            || self.ccd_tolerance == 0.0
            || [
                self.iterations,
                self.line_search_iterations,
                self.ccd_iterations,
            ]
            .contains(&0)
            || [
                self.iterations,
                self.line_search_iterations,
                self.noslip_iterations,
                self.ccd_iterations,
            ]
            .into_iter()
            .any(|x| i32::try_from(x).is_err())
        {
            return Err("contact: недопустимые параметры решателя".into());
        }
        Ok(())
    }

    pub(crate) fn names(self) -> (&'static str, &'static str) {
        let cone = match self.cone {
            FrictionCone::Elliptic => "elliptic",
            FrictionCone::Pyramidal => "pyramidal",
        };
        let solver = match self.solver {
            ConstraintSolver::Newton => "Newton",
            ConstraintSolver::ConjugateGradient => "CG",
            ConstraintSolver::ProjectedGaussSeidel => "PGS",
        };
        (cone, solver)
    }

    pub(crate) fn verify(self, sim: &Sim, rate: Rate) -> Result<(), String> {
        let actual = &sim.model().opt;
        let cone = match self.cone {
            FrictionCone::Elliptic => ffi::mjtCone::mjCONE_ELLIPTIC,
            FrictionCone::Pyramidal => ffi::mjtCone::mjCONE_PYRAMIDAL,
        };
        let solver = match self.solver {
            ConstraintSolver::Newton => ffi::mjtSolver::mjSOL_NEWTON,
            ConstraintSolver::ConjugateGradient => ffi::mjtSolver::mjSOL_CG,
            ConstraintSolver::ProjectedGaussSeidel => ffi::mjtSolver::mjSOL_PGS,
        };
        if actual.cone as u32 != cone
            || actual.solver as u32 != solver
            || actual.integrator as u32 != ffi::mjtIntegrator::mjINT_EULER
            || actual.timestep != 1.0 / rate.as_hz()
            || actual.impratio != self.impratio
            || actual.tolerance != self.tolerance
            || actual.iterations != self.iterations as i32
            || actual.ls_tolerance != self.line_search_tolerance
            || actual.ls_iterations != self.line_search_iterations as i32
            || actual.noslip_tolerance != self.noslip_tolerance
            || actual.noslip_iterations != self.noslip_iterations as i32
            || actual.ccd_tolerance != self.ccd_tolerance
            || actual.ccd_iterations != self.ccd_iterations as i32
        {
            return Err("contact: собранный решатель изменил заданные настройки".into());
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ContactPair {
    pub geometries: [String; 2],
    pub settings: ContactSettings,
}

impl ContactPair {
    pub(crate) fn validate(&self) -> Result<(), String> {
        let p = self.settings;
        let names = &self.geometries;
        if names[0] == names[1]
            || names
                .iter()
                .any(|x| x.trim().is_empty() || x.chars().any(char::is_control))
        {
            return Err(format!("contact: недопустимые имена пары {names:?}"));
        }
        // Нулевая адгезия, зазор обнаружения и смещение поверхности задаются
        // явно для модели сухого механического контакта.
        let [d0, dw, width, mid, power] = p.solimp;
        if !matches!(p.dim, 1 | 3 | 4 | 6)
            || !p
                .friction
                .iter()
                .all(|&x| (x * x).is_finite() && x >= MIN_CONTACT_FRICTION)
            || !p
                .solref
                .into_iter()
                .chain(p.solreffriction)
                .chain(p.solimp)
                .all(f64::is_finite)
            || p.solref[0] >= 0.0
            || p.solref[1] > 0.0
            || p.solreffriction[0] != 0.0
            || p.solreffriction[1] > 0.0
            || ![d0, dw, mid]
                .into_iter()
                .all(|x| (MIN_CONTACT_IMPEDANCE..=MAX_CONTACT_IMPEDANCE).contains(&x))
            || dw < d0
            || width <= 0.0
            || power < 1.0
            || [p.margin_m, p.gap_m, p.adhesion_n] != [0.0; 3]
        {
            return Err(format!(
                "contact: параметры пары {names:?} выходят за поддержанный диапазон"
            ));
        }
        Ok(())
    }

    /// Необходимая проверка разрешения одиночной нормальной моды в Euler.
    /// Устойчивость связанных мод требует отдельной сходимости по шагу.
    pub fn validate_step(&self, rate: Rate, cone: FrictionCone) -> Result<(), String> {
        self.validate()?;
        let p = self.settings;
        if p.solreffriction[1] != 0.0 && (p.dim == 1 || cone == FrictionCone::Pyramidal) {
            return Err(
                "contact: выбранный конус или размерность игнорирует отдельный solreffriction"
                    .into(),
            );
        }
        let h = 1.0 / rate.as_hz();
        // Для полууявного Euler характеристический многочлен одиночной моды
        // имеет корни внутри единичного круга при h²ω² + 2hb < 4 (условие Шура).
        // Множители d/d_width в обоих слагаемых не превосходят единицы.
        let stability = h * h * -p.solref[0] + 2.0 * h * -p.solref[1];
        if !stability.is_finite() || stability >= 4.0 {
            return Err(format!(
                "contact: шаг не разрешает нормальный отклик пары {:?}",
                self.geometries
            ));
        }
        Ok(())
    }
}

/// Дополнительная структура для моделей с дискретными узлами: те же свободные
/// тела и точки крепления, которые проверяет сборщик основного мира.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ContactBuildOptions {
    pub physics: Option<Rate>,
    /// Время и относительное демпфирование ограничений хода MuJoCo.
    pub joint_limit_response: Option<[f64; 2]>,
    pub joint_limit_impedance: Option<[f64; 5]>,
    /// Временная постоянная равенств зубчатых, ременных и винтовых передач.
    pub transmission_time_constant_s: Option<f64>,
    pub solver: Option<ContactSolver>,
    pub pairs: Vec<ContactPair>,
    pub excluded_bodies: Vec<[String; 2]>,
    pub added_objects: Vec<Object>,
    pub added_attachments: Vec<Attachment>,
}

pub(crate) fn ordered<T: Ord>(mut pair: [T; 2]) -> [T; 2] {
    if pair[0] > pair[1] {
        pair.swap(0, 1);
    }
    pair
}

impl ContactBuildOptions {
    pub(crate) fn resolve_geometries(
        &mut self,
        sim: &Sim,
        surfaces: &BTreeMap<String, String>,
    ) -> Result<(), String> {
        for name in self.pairs.iter_mut().flat_map(|pair| &mut pair.geometries) {
            if let Some(engine_name) = surfaces.get(name) {
                name.clone_from(engine_name);
            }
            crate::fragments::lookup(sim, ffi::mjtObj::mjOBJ_GEOM, name)?;
        }
        for name in self.excluded_bodies.iter().flatten() {
            crate::fragments::lookup(sim, ffi::mjtObj::mjOBJ_BODY, name)?;
        }
        Ok(())
    }

    pub(crate) fn validate(&self) -> Result<(), String> {
        if self.joint_limit_response.is_some_and(|[time, damping]| {
            !time.is_finite() || !damping.is_finite() || time <= 0.0 || damping <= 0.0
        }) {
            return Err("contact: недопустимая характеристика ограничения хода".into());
        }
        if self
            .joint_limit_impedance
            .is_some_and(|[start, end, width, midpoint, power]| {
                !(MIN_CONTACT_IMPEDANCE..=MAX_CONTACT_IMPEDANCE).contains(&start)
                    || !(start..=MAX_CONTACT_IMPEDANCE).contains(&end)
                    || !width.is_finite()
                    || width <= 0.0
                    || !(0.0 < midpoint && midpoint < 1.0)
                    || !power.is_finite()
                    || power < 1.0
            })
        {
            return Err("contact: недопустимый импеданс ограничения хода".into());
        }
        if self
            .transmission_time_constant_s
            .is_some_and(|time| !time.is_finite() || time <= 0.0)
        {
            return Err("contact: недопустимая постоянная времени передачи".into());
        }
        if self.solver.is_none() && !self.pairs.is_empty() {
            return Err("contact: явным парам нужны заданные настройки решателя".into());
        }
        if let Some(settings) = self.solver {
            settings.validate()?;
        }
        let mut seen = BTreeSet::new();
        for pair in &self.pairs {
            pair.validate()?;
            if !seen.insert(ordered(pair.geometries.each_ref())) {
                return Err(format!("contact: повторная пара {:?}", pair.geometries));
            }
        }
        seen.clear();
        for exclude in &self.excluded_bodies {
            if exclude[0] == exclude[1]
                || exclude
                    .iter()
                    .any(|b| b.trim().is_empty() || b.chars().any(char::is_control))
                || !seen.insert(ordered(exclude.each_ref()))
            {
                return Err(format!(
                    "contact: недопустимое или повторное исключение {exclude:?}"
                ));
            }
        }
        Ok(())
    }
}

impl CompiledWorld {
    pub fn open_contact_sim(&self) -> Result<Sim, String> {
        Sim::from_xml(&self.mjcf)
    }

    /// Все пары, проходящие маски и исключения текущей сцены, независимо от
    /// начального зазора. Используется перед назначением моделей по материалам.
    pub fn contact_candidates(&self) -> Result<Vec<[String; 2]>, String> {
        self.open_contact_sim()?.contact_candidates()
    }

    pub fn explicit_contact_settings(
        &self,
        pair: [&str; 2],
    ) -> Result<Option<ContactSettings>, String> {
        self.open_contact_sim()?.explicit_contact_settings(pair)
    }

    pub fn contact_geometry_names(&self) -> Result<BTreeSet<String>, String> {
        let sim = self.open_contact_sim()?;
        Ok((0..sim.ngeom())
            .filter_map(|id| {
                let name = sim.name(ffi::mjtObj::mjOBJ_GEOM, id);
                // SAFETY: id получен перечислением geom этой модели.
                let active_mask = unsafe {
                    *sim.model().geom_contype.add(id) | *sim.model().geom_conaffinity.add(id) != 0
                };
                // Старые геометрии столкновения остаются видимы в том числе с нулевой маской.
                let declared = name
                    .rsplit_once('/')
                    .is_some_and(|(prefix, _)| prefix.ends_with("/collision"));
                (!name.is_empty() && (active_mask || declared)).then_some(name)
            })
            .collect())
    }
}
