//! Полное состояние сцены: снимок, откат, копия мира и побитовый отпечаток.

use std::{collections::BTreeSet, sync::Arc};

use crate::Sim;
use crate::ffi::{self, mjData};

/// Полная копия `mjData` и изменяемой среды модели, с которой она снята.
pub struct SimSnapshot {
    d: *mut mjData,
    model: Arc<()>,
    medium: [f64; 5],
}

/// Внешняя нагрузка в системе мира, момент относительно центра масс тела.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyWrench<'a> {
    pub body: &'a str,
    pub force_n: [f64; 3],
    pub torque_nm: [f64; 3],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EqualitySwitch<'a> {
    pub name: &'a str,
    pub active: bool,
}

/// Знаковая реакция на одном теле. Все векторы в мировой системе.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WeldSide {
    pub body: usize,
    pub point_m: [f64; 3],
    pub force_n: [f64; 3],
    pub torque_nm: [f64; 3],
}

impl WeldSide {
    /// Переносит момент в указанную мировую точку по формуле M' = M + (p − p') × F.
    pub fn at_world_point(self, point_m: [f64; 3]) -> Result<Self, String> {
        let [x, y, z] = std::array::from_fn(|i| self.point_m[i] - point_m[i]);
        let [fx, fy, fz] = self.force_n;
        let shift = [y * fz - z * fy, z * fx - x * fz, x * fy - y * fx];
        let torque_nm = std::array::from_fn(|i| self.torque_nm[i] + shift[i]);
        if !point_m.iter().chain(&torque_nm).all(|x| x.is_finite()) {
            return Err("неконечный момент или точка переноса реакции weld".into());
        }
        Ok(Self {
            point_m,
            torque_nm,
            ..self
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WeldReaction {
    pub on_first: WeldSide,
    pub on_second: WeldSide,
}

fn quat_product(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let mut result = [0.0; 4];
    unsafe { ffi::mju_mulQuat(result.as_mut_ptr(), a.as_ptr(), b.as_ptr()) };
    result
}

// Снимок никто не меняет после создания.
unsafe impl Send for SimSnapshot {}
unsafe impl Sync for SimSnapshot {}

impl Drop for SimSnapshot {
    fn drop(&mut self) {
        unsafe { ffi::mj_deleteData(self.d) };
    }
}

impl Sim {
    pub fn snapshot(&self) -> SimSnapshot {
        let d = unsafe { ffi::mj_copyData(std::ptr::null_mut(), self.m, self.d) };
        assert!(!d.is_null(), "MuJoCo не скопировал состояние");
        SimSnapshot {
            d,
            model: Arc::clone(&self.identity),
            medium: self.medium_parameters(),
        }
    }

    /// Вернуть состояние из снимка этой же сцены.
    pub fn restore(&mut self, snapshot: &SimSnapshot) -> Result<(), String> {
        if !Arc::ptr_eq(&snapshot.model, &self.identity) {
            return Err("снимок снят с другой сцены".into());
        }
        unsafe { ffi::mj_copyData(self.d, self.m, snapshot.d) };
        let [density, viscosity, x, y, z] = snapshot.medium;
        self.set_medium(density, viscosity, [x, y, z]);
        Ok(())
    }

    pub(crate) fn medium_parameters(&self) -> [f64; 5] {
        let option = &self.model().opt;
        let [x, y, z] = option.wind;
        [option.density, option.viscosity, x, y, z]
    }

    /// Независимая копия мира вместе с текущим состоянием.
    pub fn fork(&self) -> Result<Sim, String> {
        let m = unsafe { ffi::mj_copyModel(std::ptr::null_mut(), self.m) };
        if m.is_null() {
            return Err("MuJoCo не скопировал модель".into());
        }
        let d = unsafe { ffi::mj_copyData(std::ptr::null_mut(), m, self.d) };
        if d.is_null() {
            unsafe { ffi::mj_deleteModel(m) };
            return Err("MuJoCo не скопировал состояние".into());
        }
        Ok(Sim {
            m,
            d,
            identity: Arc::new(()),
        })
    }

    pub fn equality_active(&self, name: &str) -> Result<bool, String> {
        let id = self.equality_id(name)?;
        Ok(Self::arr(self.data().eq_active, self.model().neq as usize)[id])
    }

    /// Переключает связь, сохраняя все внешние нагрузки.
    pub fn set_step_equality(&mut self, name: &str, active: bool) -> Result<(), String> {
        let id = self.equality_id(name)?;
        self.check_weld_activation(&[(id, active)], &BTreeSet::new())?;
        unsafe { *(*self.d).eq_active.add(id) = active };
        Ok(())
    }

    fn equality_id(&self, name: &str) -> Result<usize, String> {
        self.id(ffi::mjtObj::mjOBJ_EQUALITY, name)
            .ok_or_else(|| format!("неизвестное ограничение «{name}»"))
    }

    pub fn weld_bodies(&self, name: &str) -> Result<[usize; 2], String> {
        let id = self.equality_id(name)?;
        let model = self.model();
        let n = model.neq as usize;
        if Self::arr(model.eq_type, n)[id] as u32 != ffi::mjtEq::mjEQ_WELD {
            return Err(format!("ограничение «{name}» имеет тип, отличный от weld"));
        }
        let objects = [
            Self::arr(model.eq_obj1id, n)[id],
            Self::arr(model.eq_obj2id, n)[id],
        ];
        Ok(match Self::arr(model.eq_objtype, n)[id] as u32 {
            ffi::mjtObj::mjOBJ_BODY => objects.map(|id| id as usize),
            ffi::mjtObj::mjOBJ_SITE => {
                let bodies = Self::arr(model.site_bodyid, model.nsite as usize);
                objects.map(|id| bodies[id as usize] as usize)
            }
            _ => return Err("неизвестная семантика weld".into()),
        })
    }

    /// Реакция последнего решения в двух опорных точках weld. После Euler
    /// точки и силы принадлежат началу интегрированного интервала.
    pub fn weld_reaction(&self, name: &str) -> Result<Option<WeldReaction>, String> {
        let id = self.equality_id(name)?;
        let bodies = self.weld_bodies(name)?;
        if !self.numerically_ok() {
            return Err("численная авария при измерении реакции weld".into());
        }
        let d = self.data();
        let m = self.model();
        let rows = d.nefc as usize;
        let types = Self::arr(d.efc_type, rows);
        let ids = Self::arr(d.efc_id, rows);
        let forces = Self::arr(d.efc_force, rows);
        let mut coefficients = [0.0; 6];
        let mut count = 0;
        for ((&kind, &constraint), &force) in types.iter().zip(ids).zip(forces) {
            if kind == ffi::mjtConstraint::mjCNSTR_EQUALITY as i32 && constraint == id as i32 {
                *coefficients.get_mut(count).ok_or("лишняя строка weld")? = force;
                count += 1;
            }
        }
        if count == 0 {
            return Ok(None);
        }
        if count != coefficients.len() || !coefficients.iter().all(|x| x.is_finite()) {
            return Err("неполная или неконечная реакция weld".into());
        }
        // Порядок строк, якорей и torquescale: MuJoCo 4.0.1,
        // engine_core_constraint.c, mj_equalityAnchors и mj_instantiateEquality.
        let stride = ffi::mjNEQDATA as usize;
        let eq = &Self::arr(m.eq_data, stride * m.neq as usize)[id * stride..][..stride];
        let (points, q0, q1) =
            if Self::arr(m.eq_objtype, m.neq as usize)[id] as u32 == ffi::mjtObj::mjOBJ_BODY {
                let points = std::array::from_fn(|side| {
                    let body = bodies[side];
                    let offset = if side == 0 { 3 } else { 0 };
                    let origin = &Self::arr(d.xpos, 3 * self.nbody())[3 * body..];
                    let rotation = &Self::arr(d.xmat, 9 * self.nbody())[9 * body..];
                    std::array::from_fn(|i| {
                        origin[i]
                            + (0..3)
                                .map(|j| rotation[3 * i + j] * eq[offset + j])
                                .sum::<f64>()
                    })
                });
                let orientation = |body| self.body_pose(body).1;
                (
                    points,
                    quat_product(orientation(bodies[0]), eq[6..10].try_into().unwrap()),
                    orientation(bodies[1]),
                )
            } else {
                let n = m.neq as usize;
                let sites = [Self::arr(m.eq_obj1id, n)[id], Self::arr(m.eq_obj2id, n)[id]];
                let xyz = Self::arr(d.site_xpos, 3 * m.nsite as usize);
                let orientations = Self::arr(d.site_xquat, 4 * m.nsite as usize);
                let points = sites.map(|site| xyz[3 * site as usize..][..3].try_into().unwrap());
                let [q0, q1] =
                    sites.map(|site| orientations[4 * site as usize..][..4].try_into().unwrap());
                (points, q0, q1)
            };
        let [w, x, y, z] = q1;
        let inverse = [w, -x, -y, -z];
        // Виртуальная работа трёх вращательных строк: Mᵢ = λ·∂r/∂ωᵢ.
        let torque: [f64; 3] = std::array::from_fn(|i| {
            let mut basis = [0.0; 4];
            basis[i + 1] = 1.0;
            let derivative = quat_product(quat_product(inverse, basis), q0);
            0.5 * eq[10]
                * derivative[1..]
                    .iter()
                    .zip(&coefficients[3..])
                    .map(|(a, b)| a * b)
                    .sum::<f64>()
        });
        if !points
            .iter()
            .flatten()
            .chain(&torque)
            .all(|x| x.is_finite())
        {
            return Err("неконечные опорные точки или момент weld".into());
        }
        Ok(Some(WeldReaction {
            on_first: WeldSide {
                body: bodies[0],
                point_m: points[0],
                force_n: coefficients[..3].try_into().unwrap(),
                torque_nm: torque,
            },
            on_second: WeldSide {
                body: bodies[1],
                point_m: points[1],
                force_n: std::array::from_fn(|i| -coefficients[i]),
                torque_nm: torque.map(|x| -x),
            },
        }))
    }

    /// Устанавливает полный набор внешних нагрузок и перечисленные состояния равенств.
    /// Вход уже просуммирован по телам. При ошибке ни один эффект не применяется.
    /// Изменения подхватываются на следующем шаге, силы предыдущего шага остаются доступными.
    pub fn apply_step_effects(
        &mut self,
        loads: &[BodyWrench<'_>],
        switches: &[EqualitySwitch<'_>],
    ) -> Result<(), String> {
        self.apply_step_effects_diagnostic(loads, switches, &[])
    }

    /// Разрешает указанным численным опытам активировать сварную связь с
    /// невязкой. Такие активации требуют отдельной оценки работы захвата.
    pub fn apply_step_effects_diagnostic(
        &mut self,
        loads: &[BodyWrench<'_>],
        switches: &[EqualitySwitch<'_>],
        diagnostic_welds: &[&str],
    ) -> Result<(), String> {
        let allowed = diagnostic_welds
            .iter()
            .map(|name| {
                let id = self.equality_id(name)?;
                self.weld_bodies(name)?;
                if !switches.iter().any(|s| s.name == *name && s.active) {
                    return Err(format!("нет команды диагностического включения «{name}»"));
                }
                Ok(id)
            })
            .collect::<Result<BTreeSet<_>, String>>()?;
        let mut body_values = vec![[0.0; 6]; self.nbody()];
        let mut seen = BTreeSet::new();
        for load in loads {
            let id = self
                .id(ffi::mjtObj::mjOBJ_BODY, load.body)
                .filter(|&id| id != 0)
                .ok_or_else(|| format!("нет подвижного тела нагрузки «{}»", load.body))?;
            let weld = Self::arr(self.model().body_weldid, self.nbody())[id];
            if weld == 0
                || Self::arr(self.model().body_dofnum, self.nbody())[weld as usize] == 0
                || !seen.insert(id)
                || !load
                    .force_n
                    .iter()
                    .chain(&load.torque_nm)
                    .all(|x| x.is_finite())
            {
                return Err(format!(
                    "недопустимая или повторная нагрузка «{}»",
                    load.body
                ));
            }
            body_values[id][..3].copy_from_slice(&load.force_n);
            body_values[id][3..].copy_from_slice(&load.torque_nm);
        }
        let mut states = Vec::with_capacity(switches.len());
        seen.clear();
        for switch in switches {
            let id = self.equality_id(switch.name)?;
            if !seen.insert(id)
                || self.model().opt.disableflags
                    & (ffi::mjtDisableBit::mjDSBL_EQUALITY | ffi::mjtDisableBit::mjDSBL_CONSTRAINT)
                        as i32
                    != 0
            {
                return Err(format!(
                    "недопустимое или повторное переключение «{}»",
                    switch.name
                ));
            }
            states.push((id, switch.active));
        }
        self.check_weld_activation(&states, &allowed)?;
        unsafe {
            std::ptr::copy_nonoverlapping(
                body_values.as_ptr().cast::<f64>(),
                (*self.d).xfrc_applied,
                6 * body_values.len(),
            );
            for (id, active) in states {
                *(*self.d).eq_active.add(id) = active;
            }
        }
        Ok(())
    }

    fn check_weld_activation(
        &self,
        switches: &[(usize, bool)],
        diagnostic: &BTreeSet<usize>,
    ) -> Result<(), String> {
        let n = self.model().neq as usize;
        let kinds = Self::arr(self.model().eq_type, n);
        let was_active = Self::arr(self.data().eq_active, n);
        let added: BTreeSet<_> = switches
            .iter()
            .filter(|&&(id, active)| {
                active
                    && !was_active[id]
                    && kinds[id] as u32 == ffi::mjtEq::mjEQ_WELD
                    && !diagnostic.contains(&id)
            })
            .map(|&(id, _)| id)
            .collect();
        if added.is_empty() {
            return Ok(());
        }
        // Проверяем сами строки ограничения в текущей позе на копии mjData.
        // Ненулевая невязка создаёт восстанавливающую силу без работы захвата.
        let scratch = self.snapshot();
        unsafe {
            for &(id, active) in switches {
                *(*scratch.d).eq_active.add(id) = active;
            }
            ffi::mj_fwdPosition(self.m, scratch.d);
        }
        let data = unsafe { &*scratch.d };
        let n = data.nefc as usize;
        let rows = Self::arr(data.efc_type, n)
            .iter()
            .zip(Self::arr(data.efc_id, n))
            .zip(Self::arr(data.efc_pos, n));
        let mut count = 0;
        for ((&kind, &id), &residual) in rows {
            if kind == ffi::mjtConstraint::mjCNSTR_EQUALITY as i32 && added.contains(&(id as usize))
            {
                if residual != 0.0 {
                    return Err(
                        "нельзя включить weld с ненулевой невязкой позы без модели работы захвата"
                            .into(),
                    );
                }
                count += 1;
            }
        }
        if count != 6 * added.len() {
            return Err("неполные строки активируемого weld".into());
        }
        Ok(())
    }

    /// Вклад именованного равенства в обобщённое усилие `Jᵀλ` последнего решения.
    pub fn equality_reaction(&self, name: &str) -> Result<Option<Vec<f64>>, String> {
        let id = self.equality_id(name)? as i32;
        if !self.numerically_ok() {
            return Err("численная авария при измерении реакции ограничения".into());
        }
        let d = self.data();
        let rows = d.nefc as usize;
        let types = Self::arr(d.efc_type, rows);
        let ids = Self::arr(d.efc_id, rows);
        let forces = Self::arr(d.efc_force, rows);
        let mut lambda = vec![0.0; rows];
        let mut found = false;
        for (i, coeff) in lambda.iter_mut().enumerate() {
            if types[i] == ffi::mjtConstraint::mjCNSTR_EQUALITY as i32 && ids[i] == id {
                *coeff = forces[i];
                found = true;
            }
        }
        if !found {
            return Ok(None);
        }
        let mut result = vec![0.0; self.nv()];
        unsafe { ffi::mj_mulJacTVec(self.m, self.d, result.as_mut_ptr(), lambda.as_ptr()) };
        if !result.iter().all(|x| x.is_finite()) {
            return Err("реакция ограничения не конечна".into());
        }
        Ok(Some(result))
    }

    /// Внешние обобщённые силы, действуют на следующих шагах.
    /// Обобщённые силы последнего шага, заданные извне MuJoCo.
    pub fn qfrc_applied(&self) -> &[f64] {
        unsafe { std::slice::from_raw_parts((*self.d).qfrc_applied, self.nv()) }
    }

    pub fn qfrc_applied_mut(&mut self) -> &mut [f64] {
        let n = self.nv();
        unsafe { std::slice::from_raw_parts_mut((*self.d).qfrc_applied, n) }
    }

    pub fn ctrl_mut(&mut self) -> &mut [f64] {
        let n = self.nactuator();
        if n == 0 {
            return &mut [];
        }
        unsafe { std::slice::from_raw_parts_mut((*self.d).ctrl, n) }
    }

    /// FNV-1a по битам всего, от чего зависит следующий шаг.
    pub fn state_hash(&self) -> u64 {
        let d = self.data();
        let m = self.model();
        let (nq, nv, nu, na) = (m.nq as usize, m.nv as usize, m.nu as usize, m.na as usize);
        let medium = self.medium_parameters();
        let parts = [
            Self::arr(d.qpos, nq),
            Self::arr(d.qvel, nv),
            Self::arr(d.act, na),
            Self::arr(d.ctrl, nu),
            Self::arr(d.qfrc_applied, nv),
            Self::arr(d.qacc_warmstart, nv),
            Self::arr(d.xfrc_applied, 6 * self.nbody()),
            &medium,
        ];
        let mut h = 0xcbf2_9ce4_8422_2325_u64;
        for value in std::iter::once(&d.time).chain(parts.into_iter().flatten()) {
            for byte in value.to_bits().to_le_bytes() {
                h = (h ^ u64::from(byte)).wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
        for &active in Self::arr(d.eq_active, m.neq as usize) {
            h = (h ^ u64::from(active)).wrapping_mul(0x0000_0100_0000_01b3);
        }
        h
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SCENE: &str = r#"<mujoco><option timestep="0.001"/><worldbody>
        <geom type="plane" size="1 1 0.1"/>
        <body pos="0 0 0.3"><freejoint/><geom type="box" size="0.05 0.04 0.03" mass="0.3"/></body>
        <body pos="0.4 0 0.5"><joint name="h" axis="0 1 0" damping="0.01"/>
            <geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02" mass="0.2"/></body>
    </worldbody></mujoco>"#;

    fn advance(s: &mut Sim, n: usize) {
        for k in 0..n {
            s.qfrc_applied_mut()[6] = 0.05 * (k as f64 * 0.01).sin();
            s.step();
        }
    }

    #[test]
    fn continuing_after_restore_matches_uninterrupted_run_bit_for_bit() {
        let mut straight = Sim::from_xml(SCENE).unwrap();
        advance(&mut straight, 400);
        let snap = straight.snapshot();
        let at_snapshot = straight.state_hash();
        advance(&mut straight, 900);
        let end = straight.state_hash();
        straight.restore(&snap).unwrap();
        assert_eq!(straight.state_hash(), at_snapshot);
        advance(&mut straight, 900);
        assert_eq!(straight.state_hash(), end);
    }

    #[test]
    fn fork_is_independent_and_identical() {
        let mut main = Sim::from_xml(SCENE).unwrap();
        advance(&mut main, 300);
        let before = main.state_hash();
        let mut fork = main.fork().unwrap();
        assert_eq!(fork.state_hash(), before);
        fork.qfrc_applied_mut()[6] = 1.0;
        for _ in 0..200 {
            fork.step();
        }
        assert_ne!(fork.state_hash(), before);
        assert_eq!(main.state_hash(), before, "ветвь изменила основной мир");
    }

    #[test]
    fn snapshot_of_another_scene_is_refused() {
        let a = Sim::from_xml(SCENE).unwrap();
        let mut b = Sim::from_xml(SCENE).unwrap();
        assert!(b.restore(&a.snapshot()).is_err());
    }
}
