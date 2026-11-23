//! Контакты MuJoCo: сцены из MJCF-текста, силы в точках касания, контроль численной аварии.
//!
//! MuJoCo при расхождении сам сбрасывает симуляцию и считает дальше, как будто ничего не было.
//! Поэтому любой вывод по контактам обязан проверять [`Sim::numerically_ok`].

use std::f64::consts::PI;
use std::ffi::{CStr, CString};

use crate::{Sim, Spec, ffi};

/// Одна точка касания после шага симуляции.
#[derive(Clone, Copy, Debug)]
pub struct ContactInfo {
    pub geom: [i32; 2],
    pub pos: [f64; 3],
    /// Нормаль от `geom[0]` к `geom[1]`.
    pub normal: [f64; 3],
    /// Расстояние между поверхностями, м (меньше нуля — вдавливание).
    pub dist: f64,
    pub normal_force: f64,
    pub friction_force: f64,
}

/// Удар пружины с демпфером, которая умеет только толкать: тела расходятся, когда сила
/// падает до нуля, а не когда вдавливание вернулось к нулю (Schwager & Pöschel, 2007).
/// Возвращает коэффициент восстановления и длительность удара в единицах 1/ω₀.
fn no_tension_impact(zeta: f64) -> (f64, f64) {
    if zeta < 1.0 {
        let s = (1.0 - zeta * zeta).sqrt();
        let phase = PI - (2.0 * zeta * s).atan2(1.0 - 2.0 * zeta * zeta);
        ((-zeta / s * phase).exp(), phase / s)
    } else {
        let w = (zeta * zeta - 1.0).max(1e-12).sqrt();
        let (r1, r2) = (-zeta + w, -zeta - w);
        let t = (r2 * r2 / (r1 * r1)).ln() / (r1 - r2);
        let v = (r1 * (r1 * t).exp() - r2 * (r2 * t).exp()) / (2.0 * w);
        (v.abs(), t)
    }
}

/// `solref` в прямой форме (−жёсткость, −демпфирование) для заданного коэффициента
/// восстановления `e` и длительности удара.
pub fn restitution_solref(e: f64, contact_time_s: f64) -> (f64, f64) {
    let e = e.clamp(1e-3, 0.99);
    let (mut lo, mut hi) = (0.0_f64, 50.0_f64);
    for _ in 0..80 {
        let mid = 0.5 * (lo + hi);
        if no_tension_impact(mid).0 > e {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let zeta = 0.5 * (lo + hi);
    let wn = no_tension_impact(zeta).1 / contact_time_s;
    (-(wn * wn), -(2.0 * zeta * wn))
}

impl Sim {
    /// Собрать модель из MJCF-текста.
    pub fn from_xml(xml: &str) -> Result<Self, String> {
        let c = CString::new(xml).map_err(|e| e.to_string())?;
        let mut err = vec![0 as std::os::raw::c_char; 2048];
        let spec = unsafe {
            ffi::mj_parseXMLString(
                c.as_ptr(),
                std::ptr::null(),
                err.as_mut_ptr(),
                err.len() as i32,
            )
        };
        if spec.is_null() {
            let msg = unsafe { CStr::from_ptr(err.as_ptr()).to_string_lossy().into_owned() };
            return Err(format!("MuJoCo не разобрал сцену: {msg}"));
        }
        let spec = Spec(spec);
        let m = unsafe { ffi::mj_compile(spec.0, std::ptr::null()) };
        if m.is_null() {
            return Err(format!("MuJoCo не скомпилировал сцену: {}", spec.error()));
        }
        Self::from_model(m)
    }

    /// Вернуть сцену в начальное состояние (счётчики предупреждений тоже обнуляются).
    pub fn reset(&mut self) {
        unsafe { ffi::mj_resetData(self.m, self.d) };
        self.forward();
    }

    pub fn time(&self) -> f64 {
        self.data().time
    }

    pub fn timestep(&self) -> f64 {
        self.model().opt.timestep
    }

    /// Активные контакты и силы в них. Вызывать после [`Sim::step`].
    pub fn contacts(&self) -> Vec<ContactInfo> {
        let n = self.data().ncon.max(0) as usize;
        let raw = Self::arr(self.data().contact, n);
        raw.iter()
            .enumerate()
            .filter(|(_, c)| c.efc_address >= 0)
            .map(|(i, c)| {
                let mut f = [0.0; 6];
                unsafe { ffi::mj_contactForce(self.m, self.d, i as i32, f.as_mut_ptr()) };
                ContactInfo {
                    geom: c.geom,
                    pos: c.pos,
                    normal: [c.frame[0], c.frame[1], c.frame[2]],
                    dist: c.dist,
                    normal_force: f[0],
                    friction_force: f[1].hypot(f[2]),
                }
            })
            .collect()
    }

    /// `false`, если MuJoCo сообщил о расхождении или переполнении контактов.
    /// После такого числам сцены верить нельзя.
    pub fn numerically_ok(&self) -> bool {
        use ffi::mjtWarning as w;
        [
            w::mjWARN_BADQPOS,
            w::mjWARN_BADQVEL,
            w::mjWARN_BADQACC,
            w::mjWARN_BADCTRL,
            w::mjWARN_INERTIA,
            w::mjWARN_CONTACTFULL,
            w::mjWARN_CNSTRFULL,
        ]
        .iter()
        .all(|k| self.data().warning[*k as usize].number == 0)
    }
}

/// Нижние границы из mjmodel.h. В подвижных направлениях меньшие значения
/// MuJoCo меняет при сборке ограничения, поэтому они непригодны для явной пары.
pub const MIN_CONTACT_FRICTION: f64 = ffi::mjMINMU;
pub const MIN_CONTACT_IMPEDANCE: f64 = ffi::mjMINIMP;
pub const MAX_CONTACT_IMPEDANCE: f64 = ffi::mjMAXIMP;

/// Данные явной пары в единицах MuJoCo 4.0.1.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactSettings {
    pub dim: u8,
    pub friction: [f64; 5],
    pub solref: [f64; 2],
    pub solreffriction: [f64; 2],
    pub solimp: [f64; 5],
    pub margin_m: f64,
    pub gap_m: f64,
    pub adhesion_n: f64,
}

/// Реакция на второй геом со стороны первого. Сила, момент в точке контакта
/// и строки базиса (нормаль, две касательные) заданы в координатах мира.
#[derive(Clone, Debug, PartialEq)]
pub struct ContactWrench {
    pub geometries: [String; 2],
    pub point_m: [f64; 3],
    pub frame: [[f64; 3]; 3],
    pub separation_m: f64,
    pub force_n: [f64; 3],
    pub torque_nm: [f64; 3],
    pub dim: u8,
    pub friction: [f64; 5],
    pub solref: [f64; 2],
    pub solreffriction: [f64; 2],
    pub solimp: [f64; 5],
    pub margin_m: f64,
    pub adhesion_n: f64,
}

impl ContactWrench {
    /// Проверяет действующие параметры пары при выключенном Contact Override.
    /// Зазор обнаружения в активном контакте отдельно не хранится.
    pub fn matches_settings(&self, expected: ContactSettings) -> bool {
        self.dim == expected.dim
            && self.friction == expected.friction
            && self.solref == expected.solref
            && self.solreffriction == expected.solreffriction
            && self.solimp == expected.solimp
            && self.margin_m == expected.margin_m
            && self.adhesion_n == expected.adhesion_n
    }
}

/// Реакции одного интегрированного шага Euler. Поза, контактный базис и силы
/// относятся к решению перед интегрированием; время задаёт весь интервал.
#[derive(Clone, Debug, PartialEq)]
pub struct ContactStep {
    pub start_s: f64,
    pub end_s: f64,
    pub contacts: Vec<ContactWrench>,
}

impl Sim {
    pub fn geom_id(&self, name: &str) -> Result<usize, String> {
        self.id(ffi::mjtObj::mjOBJ_GEOM, name)
            .ok_or_else(|| format!("неизвестная геометрия «{name}»"))
    }

    /// Положение центра масс после последнего решения кинематики.
    pub fn center_of_mass(&self, name: &str) -> Result<[f64; 3], String> {
        let id = self
            .id(ffi::mjtObj::mjOBJ_BODY, name)
            .ok_or_else(|| format!("неизвестное тело «{name}»"))?;
        let offset = 3 * id;
        let centers = Self::arr(self.data().xipos, 3 * self.nbody());
        let center = &centers[offset..offset + 3];
        if !center.iter().all(|x| x.is_finite()) {
            return Err(format!("неконечный центр масс «{name}»"));
        }
        Ok(center.try_into().unwrap())
    }

    /// Может ли пара геометрий пройти обычную фильтрацию MuJoCo. Явная
    /// пара в MJCF обходит часть этих фильтров, поэтому проверяем их вручную.
    pub fn contact_pair_allowed(&self, names: [&str; 2]) -> Result<bool, String> {
        self.dynamic_pair([self.geom_id(names[0])?, self.geom_id(names[1])?], true)
    }

    /// Потенциальные пары независимо от начальной позы, в порядке ID движка.
    /// Область применения: жёсткие именованные геометрии.
    pub fn contact_candidates(&self) -> Result<Vec<[String; 2]>, String> {
        self.contact_candidates_filtered(false)
    }

    /// Пары, подавленные явными исключениями и иначе разрешённые масками и типами.
    pub fn excluded_contact_candidates(&self) -> Result<Vec<[String; 2]>, String> {
        if self.model().nexclude == 0 {
            return Ok(Vec::new());
        }
        self.contact_candidates_filtered(true)
    }

    fn contact_candidates_filtered(&self, excluded: bool) -> Result<Vec<[String; 2]>, String> {
        if self.model().nflex != 0 {
            return Err("фильтрация деформируемой сетки не проверена".into());
        }
        let mut pairs = Vec::new();
        for a in 0..self.ngeom() {
            for b in a + 1..self.ngeom() {
                if self.dynamic_pair([a, b], !excluded)?
                    && (!excluded || !self.dynamic_pair([a, b], true)?)
                {
                    pairs.push([self.geom_name(a)?, self.geom_name(b)?]);
                }
            }
        }
        Ok(pairs)
    }

    fn geom_name(&self, id: usize) -> Result<String, String> {
        let name = self.name(ffi::mjtObj::mjOBJ_GEOM, id);
        if name.is_empty() {
            return Err(format!("геометрия {id} не именована"));
        }
        Ok(name)
    }

    fn dynamic_pair(&self, [a, b]: [usize; 2], respect_exclusions: bool) -> Result<bool, String> {
        let m = self.model();
        use ffi::{mjtDisableBit as flag, mjtGeom as geom};
        if a == b
            || m.opt.disableflags & (flag::mjDSBL_CONTACT | flag::mjDSBL_CONSTRAINT) as i32 != 0
        {
            return Ok(false);
        }
        let types = Self::arr(m.geom_contype, self.ngeom());
        let affinities = Self::arr(m.geom_conaffinity, self.ngeom());
        if (types[a] & affinities[b]) | (types[b] & affinities[a]) == 0 {
            return Ok(false);
        }
        let bodies = Self::arr(m.geom_bodyid, self.ngeom());
        let [first, second] = [bodies[a] as usize, bodies[b] as usize];
        let weld = Self::arr(m.body_weldid, self.nbody());
        let [wa, wb] = [weld[first] as usize, weld[second] as usize];
        let dofs = Self::arr(m.body_dofnum, self.nbody());
        if wa == wb || (dofs[wa] == 0 && dofs[wb] == 0) {
            return Ok(false);
        }
        let parent = Self::arr(m.body_parentid, self.nbody());
        if m.opt.disableflags & flag::mjDSBL_FILTERPARENT as i32 == 0
            && wa != 0
            && wb != 0
            && (weld[parent[wa] as usize] as usize == wb
                || weld[parent[wb] as usize] as usize == wa)
        {
            return Ok(false);
        }
        if respect_exclusions && m.nexclude != 0 {
            let signature = (u16::try_from(first.min(second))
                .map_err(|_| "индекс тела вне формата исключений")?
                as u32)
                << 16
                | u32::from(
                    u16::try_from(first.max(second))
                        .map_err(|_| "индекс тела вне формата исключений")?,
                );
            if Self::arr(m.exclude_signature, m.nexclude as usize).contains(&(signature as i32)) {
                return Ok(false);
            }
        }

        // Таблица геометрий из engine_collision_driver.c 4.0.1, ограниченная
        // формами сборщика. Другие типы требуют отдельной проверки.
        let kinds = Self::arr(m.geom_type, self.ngeom());
        let [a, b] = [kinds[a] as u32, kinds[b] as u32];
        if ![a, b].into_iter().all(|x| {
            matches!(
                x,
                geom::mjGEOM_PLANE
                    | geom::mjGEOM_SPHERE
                    | geom::mjGEOM_CAPSULE
                    | geom::mjGEOM_ELLIPSOID
                    | geom::mjGEOM_CYLINDER
                    | geom::mjGEOM_BOX
                    | geom::mjGEOM_MESH
            )
        }) {
            return Err("тип геометрии вне проверенной таблицы столкновений".into());
        }
        Ok(a != geom::mjGEOM_PLANE || b != geom::mjGEOM_PLANE)
    }

    /// Параметры пары после компиляции, до возможных преобразований решателем.
    pub fn explicit_contact_settings(
        &self,
        names: [&str; 2],
    ) -> Result<Option<ContactSettings>, String> {
        let [a, b] = [
            self.geom_id(names[0])? as i32,
            self.geom_id(names[1])? as i32,
        ];
        let m = self.model();
        let n = m.npair as usize;
        let a_ids = Self::arr(m.pair_geom1, n);
        let b_ids = Self::arr(m.pair_geom2, n);
        let mut result = None;
        for i in 0..n {
            if [a_ids[i], b_ids[i]] != [a, b] && [a_ids[i], b_ids[i]] != [b, a] {
                continue;
            }
            if result.is_some() {
                return Err("движок содержит дубли явной пары".into());
            }
            result = Some(ContactSettings {
                dim: Self::arr(m.pair_dim, n)[i] as u8,
                friction: Self::arr(m.pair_friction, 5 * n)[5 * i..5 * i + 5]
                    .try_into()
                    .unwrap(),
                solref: Self::arr(m.pair_solref, 2 * n)[2 * i..2 * i + 2]
                    .try_into()
                    .unwrap(),
                solreffriction: Self::arr(m.pair_solreffriction, 2 * n)[2 * i..2 * i + 2]
                    .try_into()
                    .unwrap(),
                solimp: Self::arr(m.pair_solimp, 5 * n)[5 * i..5 * i + 5]
                    .try_into()
                    .unwrap(),
                margin_m: Self::arr(m.pair_margin, n)[i],
                gap_m: Self::arr(m.pair_gap, n)[i],
                adhesion_n: Self::arr(m.pair_adhesion, n)[i],
            });
        }
        Ok(result)
    }

    /// Снимает силы последнего решения. После `forward` это оценка в позе
    /// вызова, для интегрированных реакций используй `step_with_contacts`.
    pub fn contact_wrenches(&self) -> Result<Vec<ContactWrench>, String> {
        if !self.numerically_ok() {
            return Err("численная авария при измерении контакта".into());
        }
        let n = self.data().ncon as usize;
        let mut result = Vec::new();
        for (i, c) in Self::arr(self.data().contact, n).iter().enumerate() {
            if c.efc_address < 0 {
                continue;
            }
            let [a, b] = c.geom;
            if a < 0 || b < 0 || a as usize >= self.ngeom() || b as usize >= self.ngeom() {
                return Err("реакция контакта без двух именованных жёстких геометрий".into());
            }
            let mut local = [0.0; 6];
            unsafe { ffi::mj_contactForce(self.m, self.d, i as i32, local.as_mut_ptr()) };
            let frame = [
                c.frame[..3].try_into().unwrap(),
                c.frame[3..6].try_into().unwrap(),
                c.frame[6..].try_into().unwrap(),
            ];
            let to_world = |f: &[f64]| {
                std::array::from_fn(|j| (0..3).map(|k| f[k] * c.frame[3 * k + j]).sum::<f64>())
            };
            let force_n = to_world(&local[..3]);
            let torque_nm = to_world(&local[3..]);
            if !c
                .pos
                .iter()
                .chain(&c.frame)
                .chain(&force_n)
                .chain(&torque_nm)
                .chain(&c.friction)
                .chain(&c.solref)
                .chain(&c.solreffriction)
                .chain(&c.solimp)
                .chain([&c.dist, &c.includemargin, &c.adhesion])
                .all(|x| x.is_finite())
            {
                return Err("неконечная реакция контакта".into());
            }
            result.push(ContactWrench {
                geometries: [self.geom_name(a as usize)?, self.geom_name(b as usize)?],
                point_m: c.pos,
                frame,
                separation_m: c.dist,
                force_n,
                torque_nm,
                dim: c.dim as u8,
                friction: c.friction,
                solref: c.solref,
                solreffriction: c.solreffriction,
                solimp: c.solimp,
                margin_m: c.includemargin,
                adhesion_n: c.adhesion,
            });
        }
        Ok(result)
    }

    pub fn step_with_contacts(&mut self) -> Result<ContactStep, String> {
        let m = self.model();
        if m.opt.integrator != ffi::mjtIntegrator::mjINT_EULER as i32 {
            return Err("измерение реакции за шаг поддерживает интегратор Euler".into());
        }
        let d = self.data();
        let start_s = d.time;
        let end_expected = start_s + m.opt.timestep;
        let medium = self.medium_parameters();
        if !self.numerically_ok()
            || !end_expected.is_finite()
            || end_expected <= start_s
            || !medium.into_iter().all(f64::is_finite)
            || medium[0] < 0.0
            || medium[1] < 0.0
            || !self
                .qpos()
                .iter()
                .chain(self.qvel())
                .chain(Self::arr(d.act, m.na as usize))
                .chain(Self::arr(d.ctrl, m.nu as usize))
                .chain(Self::arr(d.qfrc_applied, self.nv()))
                .chain(Self::arr(d.xfrc_applied, 6 * self.nbody()))
                .chain(Self::arr(d.qacc_warmstart, self.nv()))
                .all(|x| x.is_finite())
        {
            return Err("неконечное состояние, вход шага или численная авария".into());
        }
        self.step();
        let end_s = self.time();
        if end_s != end_expected || !self.qpos().iter().chain(self.qvel()).all(|x| x.is_finite()) {
            return Err("движок сбросился или выдал неконечное состояние".into());
        }
        Ok(ContactStep {
            start_s,
            end_s,
            contacts: self.contact_wrenches()?,
        })
    }
}
