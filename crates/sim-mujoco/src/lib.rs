//! `sim-mujoco` — тонкая безопасная обёртка над MuJoCo (C API, Apache-2.0).
//!
//! MuJoCo — первый движок для модели человека: он нативно считает мышцы, сухожилия с
//! обёртками вокруг костей, сопряжённые суставы и контакты. Наш движок (`sim-core`) читает
//! те же модели независимо и служит сверкой (см. HUMAN_MODEL.md, шаг 2b).
//!
//! Координаты MuJoCo: Z вверх — как в нашем ядре.

use std::ffi::{CStr, CString};
use std::path::Path;

#[allow(
    non_upper_case_globals,
    non_camel_case_types,
    non_snake_case,
    dead_code,
    clippy::all
)]
pub mod ffi {
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}

use ffi::{mjData, mjModel, mjtGain, mjtGeom, mjtJoint, mjtObj, mjtTrn};

pub mod composition;
pub mod contact;
pub mod elbow;
pub mod fluid;
pub mod loads;
pub mod muscle;
pub mod skeleton;
pub mod state;

/// Версия библиотеки MuJoCo, с которой реально слинковались.
pub fn version() -> String {
    unsafe {
        CStr::from_ptr(ffi::mj_versionString())
            .to_string_lossy()
            .into_owned()
    }
}

/// Модель + данные MuJoCo. Владеет обоими указателями и освобождает их.
pub struct Sim {
    m: *mut mjModel,
    d: *mut mjData,
    identity: std::sync::Arc<()>,
}

// Доступ на запись — только через `&mut self`, поэтому одновременной записи из разных
// потоков не бывает; чтение модели и данных MuJoCo потокобезопасно.
unsafe impl Send for Sim {}
unsafe impl Sync for Sim {}

impl Drop for Sim {
    fn drop(&mut self) {
        unsafe {
            ffi::mj_deleteData(self.d);
            ffi::mj_deleteModel(self.m);
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeomKind {
    Plane,
    Sphere,
    Capsule,
    Ellipsoid,
    Cylinder,
    Box,
    Mesh,
    Other,
}

#[derive(Clone, Debug)]
pub struct GeomInfo {
    pub id: usize,
    pub name: String,
    pub kind: GeomKind,
    /// Размеры по соглашению MuJoCo (радиус, полудлина, полуразмеры …).
    pub size: [f64; 3],
    pub mesh: Option<usize>,
    pub rgba: [f32; 4],
    pub group: i32,
    pub body: usize,
}

#[derive(Clone, Debug)]
pub struct MeshData {
    pub positions: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
}

#[derive(Clone, Debug)]
pub struct JointInfo {
    pub id: usize,
    pub name: String,
    pub kind: JointKind,
    pub qpos_adr: usize,
    pub dof_adr: usize,
    pub limited: bool,
    pub range: [f64; 2],
    pub body: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointKind {
    Free,
    Ball,
    Slide,
    Hinge,
}

/// Связь «ведомый сустав = полином от ведущего» (адреса в qpos).
#[derive(Clone, Debug)]
pub struct JointCoupling {
    pub q1: usize,
    pub q2: Option<usize>,
    pub poly: [f64; 5],
}

/// Мышца — актуатор MuJoCo с мышечным усилением, тянущий за сухожилие.
#[derive(Clone, Debug)]
pub struct MuscleInfo {
    pub actuator: usize,
    pub name: String,
    pub tendon: usize,
    /// Максимальная изометрическая сила F₀, Н (как её скомпилировал MuJoCo).
    pub f_max: f64,
    /// Диапазон длины мышцы-сухожилия, м.
    pub length_range: [f64; 2],
}

fn cstr_or_empty(p: *const std::os::raw::c_char) -> String {
    if p.is_null() {
        String::new()
    } else {
        unsafe { CStr::from_ptr(p).to_string_lossy().into_owned() }
    }
}

impl Sim {
    /// Загрузить MJCF-модель (XML) с диска и посчитать начальную позу.
    pub fn load(path: &Path) -> Result<Self, String> {
        let c_path = CString::new(path.to_string_lossy().as_bytes()).map_err(|e| e.to_string())?;
        let mut err = vec![0 as std::os::raw::c_char; 2048];
        let m = unsafe {
            ffi::mj_loadXML(
                c_path.as_ptr(),
                std::ptr::null(),
                err.as_mut_ptr(),
                err.len() as i32,
            )
        };
        if m.is_null() {
            let msg = unsafe { CStr::from_ptr(err.as_ptr()).to_string_lossy().into_owned() };
            return Err(format!("MuJoCo не загрузил {}: {msg}", path.display()));
        }
        Self::from_model(m)
    }

    /// Взять во владение скомпилированную модель.
    fn from_model(m: *mut mjModel) -> Result<Self, String> {
        let d = unsafe { ffi::mj_makeData(m) };
        if d.is_null() {
            unsafe { ffi::mj_deleteModel(m) };
            return Err("MuJoCo не создал mjData".into());
        }
        let mut s = Self { m, d, identity: std::sync::Arc::new(()) };
        s.forward();
        Ok(s)
    }

    pub fn model(&self) -> &mjModel {
        unsafe { &*self.m }
    }

    pub fn data(&self) -> &mjData {
        unsafe { &*self.d }
    }

    pub(crate) fn arr<'a, T>(ptr: *const T, len: usize) -> &'a [T] {
        if ptr.is_null() || len == 0 {
            &[]
        } else {
            unsafe { std::slice::from_raw_parts(ptr, len) }
        }
    }

    pub fn nq(&self) -> usize {
        self.model().nq as usize
    }
    pub fn nv(&self) -> usize {
        self.model().nv as usize
    }
    pub fn nbody(&self) -> usize {
        self.model().nbody as usize
    }
    pub fn njnt(&self) -> usize {
        self.model().njnt as usize
    }
    pub fn ngeom(&self) -> usize {
        self.model().ngeom as usize
    }
    pub fn nmesh(&self) -> usize {
        self.model().nmesh as usize
    }
    pub fn ntendon(&self) -> usize {
        self.model().ntendon as usize
    }
    pub fn nactuator(&self) -> usize {
        self.model().nactuator as usize
    }
    pub fn nkey(&self) -> usize {
        self.model().nkey as usize
    }

    pub fn qpos(&self) -> &[f64] {
        Self::arr(self.data().qpos, self.nq())
    }

    pub fn qpos_mut(&mut self) -> &mut [f64] {
        let n = self.nq();
        unsafe { std::slice::from_raw_parts_mut((*self.d).qpos, n) }
    }

    pub fn qpos0(&self) -> &[f64] {
        Self::arr(self.model().qpos0, self.nq())
    }

    /// Кинематика + всё, что зависит от позы (сухожилия, мышцы, силы) без шага по времени.
    pub fn forward(&mut self) {
        unsafe { ffi::mj_forward(self.m, self.d) }
    }

    pub fn step(&mut self) {
        unsafe { ffi::mj_step(self.m, self.d) }
    }

    pub fn reset_to_keyframe(&mut self, key: usize) {
        if key < self.nkey() {
            unsafe { ffi::mj_resetDataKeyframe(self.m, self.d, key as i32) };
            self.forward();
        }
    }

    pub fn name(&self, obj: mjtObj::Type, id: usize) -> String {
        cstr_or_empty(unsafe { ffi::mj_id2name(self.m, obj as i32, id as i32) })
    }

    pub fn id(&self, obj: mjtObj::Type, name: &str) -> Option<usize> {
        let c = CString::new(name).ok()?;
        let id = unsafe { ffi::mj_name2id(self.m, obj as i32, c.as_ptr()) };
        (id >= 0).then_some(id as usize)
    }

    /// Масса всего тела по модели, кг.
    pub fn total_mass(&self) -> f64 {
        Self::arr(self.model().body_mass, self.nbody()).iter().sum()
    }

    pub fn geoms(&self) -> Vec<GeomInfo> {
        let m = self.model();
        let n = self.ngeom();
        let gtype = Self::arr(m.geom_type, n);
        let gsize = Self::arr(m.geom_size, 3 * n);
        let dataid = Self::arr(m.geom_dataid, n);
        let matid = Self::arr(m.geom_matid, n);
        let group = Self::arr(m.geom_group, n);
        let body = Self::arr(m.geom_bodyid, n);
        let grgba = Self::arr(m.geom_rgba, 4 * n);
        let mrgba = Self::arr(m.mat_rgba, 4 * m.nmat as usize);
        (0..n)
            .map(|i| {
                let kind = match gtype[i] as u32 {
                    mjtGeom::mjGEOM_PLANE => GeomKind::Plane,
                    mjtGeom::mjGEOM_SPHERE => GeomKind::Sphere,
                    mjtGeom::mjGEOM_CAPSULE => GeomKind::Capsule,
                    mjtGeom::mjGEOM_ELLIPSOID => GeomKind::Ellipsoid,
                    mjtGeom::mjGEOM_CYLINDER => GeomKind::Cylinder,
                    mjtGeom::mjGEOM_BOX => GeomKind::Box,
                    mjtGeom::mjGEOM_MESH => GeomKind::Mesh,
                    _ => GeomKind::Other,
                };
                // Цвет: материал, если задан, иначе rgba геома (так рисует сам MuJoCo).
                let rgba = if matid[i] >= 0 {
                    let k = 4 * matid[i] as usize;
                    [mrgba[k], mrgba[k + 1], mrgba[k + 2], mrgba[k + 3]]
                } else {
                    [
                        grgba[4 * i],
                        grgba[4 * i + 1],
                        grgba[4 * i + 2],
                        grgba[4 * i + 3],
                    ]
                };
                GeomInfo {
                    id: i,
                    name: self.name(mjtObj::mjOBJ_GEOM, i),
                    kind,
                    size: [gsize[3 * i], gsize[3 * i + 1], gsize[3 * i + 2]],
                    mesh: (kind == GeomKind::Mesh && dataid[i] >= 0).then_some(dataid[i] as usize),
                    rgba,
                    group: group[i],
                    body: body[i] as usize,
                }
            })
            .collect()
    }

    /// Положение и ориентация геома в мире: позиция и матрица 3×3 по строкам.
    pub fn geom_pose(&self, i: usize) -> ([f64; 3], [f64; 9]) {
        let n = self.ngeom();
        let p = Self::arr(self.data().geom_xpos, 3 * n);
        let r = Self::arr(self.data().geom_xmat, 9 * n);
        let mut mat = [0.0; 9];
        mat.copy_from_slice(&r[9 * i..9 * i + 9]);
        ([p[3 * i], p[3 * i + 1], p[3 * i + 2]], mat)
    }

    pub fn mesh(&self, id: usize) -> MeshData {
        let m = self.model();
        let nm = self.nmesh();
        let vadr = Self::arr(m.mesh_vertadr, nm)[id] as usize;
        let vnum = Self::arr(m.mesh_vertnum, nm)[id] as usize;
        let fadr = Self::arr(m.mesh_faceadr, nm)[id] as usize;
        let fnum = Self::arr(m.mesh_facenum, nm)[id] as usize;
        let verts = Self::arr(m.mesh_vert, 3 * m.nmeshvert as usize);
        let faces = Self::arr(m.mesh_face, 3 * m.nmeshface as usize);
        let positions = (vadr..vadr + vnum)
            .map(|v| [verts[3 * v], verts[3 * v + 1], verts[3 * v + 2]])
            .collect();
        let indices = faces[3 * fadr..3 * (fadr + fnum)]
            .iter()
            .map(|&k| k as u32)
            .collect();
        MeshData { positions, indices }
    }

    pub fn joints(&self) -> Vec<JointInfo> {
        let m = self.model();
        let n = self.njnt();
        let jt = Self::arr(m.jnt_type, n);
        let qa = Self::arr(m.jnt_qposadr, n);
        let da = Self::arr(m.jnt_dofadr, n);
        let lim = Self::arr(m.jnt_limited, n);
        let rng = Self::arr(m.jnt_range, 2 * n);
        let body = Self::arr(m.jnt_bodyid, n);
        (0..n)
            .map(|i| JointInfo {
                id: i,
                name: self.name(mjtObj::mjOBJ_JOINT, i),
                kind: match jt[i] as u32 {
                    mjtJoint::mjJNT_FREE => JointKind::Free,
                    mjtJoint::mjJNT_BALL => JointKind::Ball,
                    mjtJoint::mjJNT_SLIDE => JointKind::Slide,
                    _ => JointKind::Hinge,
                },
                qpos_adr: qa[i] as usize,
                dof_adr: da[i] as usize,
                limited: lim[i],
                range: [rng[2 * i], rng[2 * i + 1]],
                body: body[i] as usize,
            })
            .collect()
    }

    pub fn muscles(&self) -> Vec<MuscleInfo> {
        let m = self.model();
        let n = self.nactuator();
        let gain = Self::arr(m.actuator_gaintype, n);
        let trn = Self::arr(m.actuator_trntype, n);
        let trnid = Self::arr(m.actuator_trnid, 2 * n);
        let lr = Self::arr(m.actuator_lengthrange, 2 * m.nout as usize);
        let outadr = Self::arr(m.actuator_outadr, n);
        (0..n)
            .filter(|&i| {
                gain[i] as u32 == mjtGain::mjGAIN_MUSCLE && trn[i] as u32 == mjtTrn::mjTRN_TENDON
            })
            .map(|i| MuscleInfo {
                actuator: i,
                name: self.name(mjtObj::mjOBJ_ACTUATOR, i),
                tendon: trnid[2 * i] as usize,
                f_max: self.muscle_model(i).f0(),
                length_range: {
                    let o = outadr[i] as usize;
                    [lr[2 * o], lr[2 * o + 1]]
                },
            })
            .collect()
    }

    /// Путь сухожилия в мире — отрезки между точками (как рисует сам MuJoCo; шкивы разрывают путь).
    pub fn tendon_segments(&self, tendon: usize) -> Vec<([f64; 3], [f64; 3])> {
        let d = self.data();
        let nt = self.ntendon();
        let nwrap = self.model().nwrap as usize;
        let adr = Self::arr(d.ten_wrapadr, nt)[tendon] as usize;
        let num = Self::arr(d.ten_wrapnum, nt)[tendon] as usize;
        let obj = Self::arr(d.wrap_obj, 2 * nwrap);
        let xpos = Self::arr(d.wrap_xpos, 6 * nwrap);
        let mut out = Vec::new();
        for j in adr..(adr + num).saturating_sub(1) {
            if obj[j] == -2 || obj[j + 1] == -2 {
                continue;
            }
            let a = [xpos[3 * j], xpos[3 * j + 1], xpos[3 * j + 2]];
            let b = [xpos[3 * j + 3], xpos[3 * j + 4], xpos[3 * j + 5]];
            out.push((a, b));
        }
        out
    }

    /// Длина сухожилия (мышцы-сухожилия) в текущей позе, м.
    pub fn tendon_length(&self, tendon: usize) -> f64 {
        Self::arr(self.data().ten_length, self.ntendon())[tendon]
    }

    pub fn qvel_mut(&mut self) -> &mut [f64] {
        let n = self.nv();
        unsafe { std::slice::from_raw_parts_mut((*self.d).qvel, n) }
    }

    pub fn qvel(&self) -> &[f64] {
        Self::arr(self.data().qvel, self.nv())
    }

    /// Положение и ориентация (w, x, y, z) тела в мире после `forward`.
    pub fn body_pose(&self, body: usize) -> ([f64; 3], [f64; 4]) {
        let p = Self::arr(self.data().xpos, 3 * self.nbody());
        let q = Self::arr(self.data().xquat, 4 * self.nbody());
        (
            [p[3 * body], p[3 * body + 1], p[3 * body + 2]],
            [
                q[4 * body],
                q[4 * body + 1],
                q[4 * body + 2],
                q[4 * body + 3],
            ],
        )
    }

    /// Полная матрица масс M(q) (с инерцией роторов) после `forward`, по строкам.
    pub fn full_mass_matrix(&self) -> Vec<f64> {
        let nv = self.nv();
        let mut out = vec![0.0; nv * nv];
        unsafe { ffi::mj_fullM(self.m, self.d, out.as_mut_ptr()) };
        out
    }

    /// Силы смещения C(q, q̇) + G(q) после `forward` (то, что MuJoCo называет qfrc_bias).
    pub fn bias_forces(&self) -> &[f64] {
        Self::arr(self.data().qfrc_bias, self.nv())
    }

    pub fn gravity(&self) -> [f64; 3] {
        self.model().opt.gravity
    }

    /// Среда вокруг тела для гидро/аэродинамики MuJoCo: плотность (кг/м³), вязкость (Па·с),
    /// ветер (м/с, мировые оси). 0 плотности и вязкости — вакуум (по умолчанию в моделях).
    pub fn set_medium(&mut self, density: f64, viscosity: f64, wind: [f64; 3]) {
        // SAFETY: m — живая модель, которой владеет Sim; поля opt — простые числа.
        unsafe {
            (*self.m).opt.density = density;
            (*self.m).opt.viscosity = viscosity;
            (*self.m).opt.wind = wind;
        }
    }

    /// Силы среды в обобщённых координатах после `forward` (qfrc_fluid).
    pub fn fluid_forces(&self) -> &[f64] {
        Self::arr(self.data().qfrc_fluid, self.nv())
    }

    /// Все пассивные силы (пружины, демпферы, среда) после `forward`.
    pub fn passive_forces(&self) -> &[f64] {
        Self::arr(self.data().qfrc_passive, self.nv())
    }

    /// Масса тела (без потомков), кг.
    pub fn body_mass(&self, body: usize) -> f64 {
        Self::arr(self.model().body_mass, self.nbody())[body]
    }

    /// Главные моменты инерции тела относительно его центра масс, кг·м².
    pub fn body_inertia(&self, body: usize) -> [f64; 3] {
        let a = Self::arr(self.model().body_inertia, 3 * self.nbody());
        [a[3 * body], a[3 * body + 1], a[3 * body + 2]]
    }

    /// Связи суставов вида q₁ − q₁⁰ = a₀ + a₁x + a₂x² + a₃x³ + a₄x⁴, x = q₂ − q₂⁰
    /// (так в моделях задан, например, лопаточно-плечевой ритм и движение надколенника).
    pub fn joint_couplings(&self) -> Vec<JointCoupling> {
        let m = self.model();
        let n = m.neq as usize;
        let ty = Self::arr(m.eq_type, n);
        let o1 = Self::arr(m.eq_obj1id, n);
        let o2 = Self::arr(m.eq_obj2id, n);
        let act = Self::arr(m.eq_active0, n);
        let data = Self::arr(m.eq_data, ffi::mjNEQDATA as usize * n);
        let qa = Self::arr(m.jnt_qposadr, self.njnt());
        (0..n)
            .filter(|&i| ty[i] as u32 == ffi::mjtEq::mjEQ_JOINT && act[i])
            .map(|i| {
                let k = ffi::mjNEQDATA as usize * i;
                JointCoupling {
                    q1: qa[o1[i] as usize] as usize,
                    q2: (o2[i] >= 0).then(|| qa[o2[i] as usize] as usize),
                    poly: [data[k], data[k + 1], data[k + 2], data[k + 3], data[k + 4]],
                }
            })
            .collect()
    }

    /// Выставить зависимые суставы по связям (несколько проходов — связи бывают цепочкой).
    pub fn apply_joint_couplings(&mut self, couplings: &[JointCoupling]) {
        let q0: Vec<f64> = self.qpos0().to_vec();
        for _ in 0..4 {
            for c in couplings {
                let x = c.q2.map(|j| self.qpos()[j] - q0[j]).unwrap_or(0.0);
                let p = &c.poly;
                let v = q0[c.q1] + p[0] + x * (p[1] + x * (p[2] + x * (p[3] + x * p[4])));
                self.qpos_mut()[c.q1] = v;
            }
        }
    }

    /// Точка сустава в мире (после `forward`).
    pub fn joint_anchor(&self, joint: usize) -> [f64; 3] {
        let a = Self::arr(self.data().xanchor, 3 * self.njnt());
        [a[3 * joint], a[3 * joint + 1], a[3 * joint + 2]]
    }

    /// Направление оси сустава в мире (после `forward`).
    pub fn joint_axis(&self, joint: usize) -> [f64; 3] {
        let a = Self::arr(self.data().xaxis, 3 * self.njnt());
        [a[3 * joint], a[3 * joint + 1], a[3 * joint + 2]]
    }

    /// Имена и массы тел.
    pub fn bodies(&self) -> Vec<(String, f64)> {
        let mass = Self::arr(self.model().body_mass, self.nbody());
        (0..self.nbody())
            .map(|i| (self.name(mjtObj::mjOBJ_BODY, i), mass[i]))
            .collect()
    }

    /// Масса тела вместе со всеми потомками (например, вся рука от ключицы), кг.
    pub fn subtree_mass(&self, body: usize) -> f64 {
        Self::arr(self.model().body_subtreemass, self.nbody())[body]
    }

    /// Корень тела человека: ветка от мира с наибольшей массой (в моделях бывают стол,
    /// бутылка и другие предметы — они человеку не принадлежат).
    pub fn human_root(&self) -> usize {
        let parent = Self::arr(self.model().body_parentid, self.nbody());
        (1..self.nbody())
            .filter(|&b| parent[b] == 0)
            .max_by(|&a, &b| {
                self.subtree_mass(a)
                    .partial_cmp(&self.subtree_mass(b))
                    .unwrap()
            })
            .unwrap_or(0)
    }

    /// Число объектов в пути сухожилия (точки крепления + поверхности обёртки).
    pub fn tendon_path_objects(&self, tendon: usize) -> usize {
        Self::arr(self.model().tendon_num, self.ntendon())[tendon] as usize
    }

    /// Нижняя и верхняя точки видимой геометрии по Z (для роста модели и постановки на пол).
    pub fn z_extent(&self) -> (f64, f64) {
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        let geoms = self.geoms();
        for g in geoms
            .iter()
            .filter(|g| g.kind == GeomKind::Mesh && g.rgba[3] > 0.0)
        {
            let (p, r) = self.geom_pose(g.id);
            let md = self.mesh(g.mesh.unwrap());
            for v in &md.positions {
                let z = p[2] + r[6] * v[0] as f64 + r[7] * v[1] as f64 + r[8] * v[2] as f64;
                lo = lo.min(z);
                hi = hi.max(z);
            }
        }
        (lo, hi)
    }
}

/// Пути к моделям тела в sim/assets (относительно корня workspace).
pub mod models {
    use std::path::PathBuf;

    pub fn assets_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets")
    }

    /// MS-Human-700: всё тело, 700 мышц.
    pub fn ms_human_700() -> PathBuf {
        assets_dir().join("third_party/ms_human_700/MS-Human-700.xml")
    }

    /// MS-Human-700: правая рука с подробной кистью, 81 мышца.
    pub fn ms_human_700_arm() -> PathBuf {
        assets_dir().join("third_party/ms_human_700/MS-Human-700-Manipulation.xml")
    }

    /// Туловище MyoSuite (каркас) и правая рука MyoArm — точки входа для сборки.
    pub fn myotorso_passive() -> PathBuf {
        assets_dir().join("models/myotorso_passive.xml")
    }

    pub fn myoarm_child() -> PathBuf {
        assets_dir().join("models/myoarm_r_child.xml")
    }
}

/// Разобранная, но не скомпилированная модель (MjSpec). Освобождается сама.
struct Spec(*mut ffi::mjSpec);

impl Drop for Spec {
    fn drop(&mut self) {
        unsafe { ffi::mj_deleteSpec(self.0) }
    }
}

impl Spec {
    fn parse(path: &Path) -> Result<Self, String> {
        let c_path = CString::new(path.to_string_lossy().as_bytes()).map_err(|e| e.to_string())?;
        let mut err = vec![0 as std::os::raw::c_char; 2048];
        let s = unsafe {
            ffi::mj_parseXML(
                c_path.as_ptr(),
                std::ptr::null(),
                err.as_mut_ptr(),
                err.len() as i32,
            )
        };
        if s.is_null() {
            let msg = unsafe { CStr::from_ptr(err.as_ptr()).to_string_lossy().into_owned() };
            return Err(format!("MuJoCo не разобрал {}: {msg}", path.display()));
        }
        Ok(Spec(s))
    }

    fn find(&self, obj: mjtObj::Type, name: &str) -> Result<*mut ffi::mjsElement, String> {
        let c = CString::new(name).map_err(|e| e.to_string())?;
        let e = unsafe { ffi::mjs_findElement(self.0, obj, c.as_ptr()) };
        if e.is_null() {
            Err(format!("в модели нет «{name}»"))
        } else {
            Ok(e)
        }
    }

    /// Удалить все элементы данного типа (например, все суставы туловища).
    fn delete_all(&mut self, obj: mjtObj::Type) {
        loop {
            let e = unsafe { ffi::mjs_firstElement(self.0, obj) };
            if e.is_null() {
                break;
            }
            unsafe { ffi::mjs_delete(self.0, e) };
        }
    }

    fn error(&self) -> String {
        cstr_or_empty(unsafe { ffi::mjs_getError(self.0) })
    }
}

/// MyoArm (MyoSuite): правая рука от ключицы до пальцев, 63 мышцы, на неподвижном туловище.
/// Повторяет сборку `myoarm_r` из myo_sim/build/compose.py: туловище без суставов, мышц,
/// сухожилий и связей; рука присоединена в точке `arm_attach_r`.
pub fn compose_myoarm() -> Result<Sim, String> {
    let mut torso = Spec::parse(&models::myotorso_passive())?;
    for obj in [
        mjtObj::mjOBJ_EQUALITY,
        mjtObj::mjOBJ_ACTUATOR,
        mjtObj::mjOBJ_TENDON,
        mjtObj::mjOBJ_JOINT,
    ] {
        torso.delete_all(obj);
    }
    let arm = Spec::parse(&models::myoarm_child())?;
    let site = torso.find(mjtObj::mjOBJ_SITE, "arm_attach_r")?;
    let root = arm.find(mjtObj::mjOBJ_BODY, "myoarm_r_root")?;
    let empty = CString::new("").unwrap();
    let attached = unsafe { ffi::mjs_attach(site, root, empty.as_ptr(), empty.as_ptr()) };
    if attached.is_null() {
        return Err(format!("не удалось присоединить руку: {}", torso.error()));
    }
    let m = unsafe { ffi::mj_compile(torso.0, std::ptr::null()) };
    if m.is_null() {
        return Err(format!("MuJoCo не скомпилировал MyoArm: {}", torso.error()));
    }
    drop(arm);
    Sim::from_model(m)
}
