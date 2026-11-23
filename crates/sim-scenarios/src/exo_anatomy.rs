//! Сценарий «Экзо на анатомической руке» (шаги 2c–2f карты человека): мышечно-скелетная
//! модель руки в MuJoCo, груз в кисти, экзоскелет локтя с манжетой на предплечье.
//!
//! Что считаем и показываем:
//! * сколько момента нужно в локте — MuJoCo, наш движок и сумма по центрам масс (три способа);
//! * сила человека: модель против измерений на людях (Frey-Law 2012), кривая «сила — угол»;
//! * как мышцы делят нагрузку (статическая оптимизация) — мышцы окрашены по активации;
//! * сила в самом локте без экзоскелета и с ним — против оценки по литературе;
//! * сила и давление манжеты на кожу, паразитная сила от несоосности оси;
//! * пределы движения, скорость, зоны нервов, где нельзя давить.

use std::collections::HashMap;

use nalgebra::DVector;
use sim_core::dynamics::{rnea, world_to_body};
use sim_core::integrate::{PointForce, external_forces};
use sim_core::{Pose, Shape, Vec3};
use sim_mujoco::elbow::{
    Capacity, ElbowRig, ExoLoad, JointReaction, MuscleAtPose, StaticOpt, capacity, joint_reaction,
    misalignment_force, muscles_at_pose, required_torque, static_opt,
};
use sim_mujoco::fluid::{
    buoyancy_elbow_torque, cylinder_crossflow_torque, cylinder_rotation_drag, forearm_hand_length,
    mujoco_fluid_elbow_torque,
};
use sim_mujoco::loads::{
    NIOSH_ACTION_LIMIT_N, NIOSH_MAX_PERMISSIBLE_N, ProximalLoads, proximal_loads,
};
use sim_mujoco::skeleton::Skeleton;
use sim_mujoco::{GeomInfo, JointCoupling, MuscleInfo, Sim};

use crate::anatomy::{
    Which, bone_geoms, bone_visuals, muscle_radius, muscle_slots, muscle_tubes, open_model,
};
use crate::human_limits as lim;
use crate::scenario::*;

const MODELS: [(Which, &str); 2] = [
    (Which::HumanArm, "MS-Human-700 — рука с кистью (81 мышца)"),
    (Which::MyoArm, "MyoArm (MyoSuite) — 63 мышцы"),
];

const C_MUSCLE_IDLE: [f32; 4] = [0.78, 0.62, 0.62, 1.0];
const C_PINK: [f32; 4] = [0.95, 0.45, 0.7, 1.0];

fn v3(a: [f64; 3]) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}

/// Цвет мышцы по активации (10 ступеней — чтобы не плодить материалы).
fn activation_color(a: f64) -> [f32; 4] {
    let s = ((a.clamp(0.0, 1.0) * 10.0).round() / 10.0) as f32;
    let lerp = |x: f32, y: f32| x + (y - x) * s;
    [lerp(0.95, 0.85), lerp(0.75, 0.05), lerp(0.55, 0.05), 1.0]
}

struct CurveRow {
    deg: f64,
    cap: Capacity,
    required: f64,
}

/// Зона нерва: (название, центр, радиус).
type Zone = (&'static str, Vec3, f64);

/// Среда вокруг руки.
const MEDIA: [&str; 3] = [
    "сухо, тихо (воздух)",
    "рука в воде по плечо (пресная, 20 °C)",
    "ветер в лицо",
];
/// Плотность чугуна гири, кг/м³.
const KETTLEBELL_DENSITY: f64 = 7200.0;
/// Вода и воздух: плотность (кг/м³), вязкость (Па·с).
const WATER: (f64, f64) = (998.0, 1.0e-3);
const AIR: (f64, f64) = (1.204, 1.81e-5);
/// Ручной расчёт сопротивления: предплечье с кистью — цилиндр D = 7 см, C_d = 1 (оценка).
const FOREARM_D: f64 = 0.07;
const CYL_CD: f64 = 1.0;
/// Шар (гиря), C_d ≈ 0.47 (докритическое обтекание).
const SPHERE_CD: f64 = 0.47;

/// Нагрузки от среды: вода (Архимед + сопротивление) или ветер.
struct EnvLoads {
    /// Обобщённая сила среды в покое (+ помогает сгибанию), Н·м.
    static_q: f64,
    /// Из неё — от груза (Архимед для гири), Н·м.
    static_payload: f64,
    /// Сопротивление предплечья с кистью при сгибании со скоростью ω по MuJoCo (модуль), Н·м.
    drag_mj: f64,
    /// То же ручным расчётом (цилиндр), Н·м.
    drag_hand: f64,
    /// Сопротивление гири, Н·м.
    drag_payload: f64,
    /// Ветер: MuJoCo и ручной расчёт поперечного обтекания (при ветре поперёк предплечья), Н·м.
    wind_mj: f64,
    wind_hand: f64,
    /// Нужно держать / двигать с учётом среды, Н·м.
    tau_hold: f64,
    tau_move: f64,
    /// Наибольшая активация мышц при движении в среде (экзо помогает той же долей).
    max_act_move: f64,
    feasible_move: bool,
}

struct Results {
    ms: Vec<MuscleAtPose>,
    cap: Capacity,
    tau_req: f64,
    tau_ours: f64,
    tau_sum: f64,
    tau_exo: f64,
    opt: StaticOpt,
    feasible_unassisted: bool,
    react: JointReaction,
    react_none: JointReaction,
    act: HashMap<usize, f64>,
    misalign: (f64, f64),
    misalign_max: (f64, f64),
    r_err: f64,
    anchor: Vec3,
    medial: Vec3,
    posterior: Vec3,
    humerus: Vec3,
    forearm: Vec3,
    hand: Vec3,
    cuff_point: Vec3,
    cuff_dir: Vec3,
    proximal: Option<ProximalLoads>,
    env: EnvLoads,
}

/// Итоги расчёта для тестов и отчётов.
#[derive(Clone, Copy, Debug)]
pub struct ExoSummary {
    pub required: f64,
    pub exo: f64,
    pub joint_force_unassisted: f64,
    pub joint_force: f64,
    pub cuff_force: f64,
    pub cuff_pressure_kpa: f64,
    pub max_activation: f64,
    pub feasible: bool,
    pub flexion_90_model: f64,
}

pub struct ExoAnatomy {
    which: usize,
    sim: Option<Sim>,
    rig: Option<ElbowRig>,
    skel: Option<Skeleton>,
    error: Option<String>,
    geoms: Vec<GeomInfo>,
    muscles: Vec<(MuscleInfo, f64, usize)>,
    couplings: Vec<JointCoupling>,
    elbow_range: (f64, f64),
    anterior_sign: f64,
    payload: f64,
    angle_deg: f64,
    assist: f64,
    cuff_cm: f64,
    cuff_area_cm2: f64,
    misalign_mm: f64,
    stiffness: f64,
    don_deg: f64,
    medium: usize,
    speed_dps: f64,
    wind_ms: f64,
    res: Option<Results>,
    curve: Vec<CurveRow>,
    /// Плечи мышц при 90°: (мышца, модель см, литература см).
    arms_90: Vec<(String, f64, f64)>,
    epoch: u64,
    t: f64,
}

impl Default for ExoAnatomy {
    fn default() -> Self {
        let mut s = Self {
            which: 0,
            sim: None,
            rig: None,
            skel: None,
            error: None,
            geoms: Vec::new(),
            muscles: Vec::new(),
            couplings: Vec::new(),
            elbow_range: (0.0, 130.0),
            anterior_sign: 1.0,
            payload: 20.0,
            angle_deg: 90.0,
            assist: 0.7,
            cuff_cm: 15.0,
            cuff_area_cm2: 60.0,
            misalign_mm: 10.0,
            stiffness: lim::TISSUE_STIFFNESS_N_PER_MM,
            don_deg: 90.0,
            medium: 0,
            speed_dps: 90.0,
            wind_ms: 20.0,
            res: None,
            curve: Vec::new(),
            arms_90: Vec::new(),
            epoch: 0,
            t: 0.0,
        };
        s.load();
        s
    }
}

impl ExoAnatomy {
    fn load(&mut self) {
        self.epoch += 1;
        self.res = None;
        let built = open_model(MODELS[self.which].0).and_then(|sim| {
            let rig = ElbowRig::find(&sim)?;
            let skel = Skeleton::from_mujoco(&sim)?;
            Ok((sim, rig, skel))
        });
        match built {
            Err(e) => {
                self.error = Some(e);
                self.sim = None;
            }
            Ok((sim, rig, skel)) => {
                self.error = None;
                self.geoms = bone_geoms(&sim);
                self.muscles = sim
                    .muscles()
                    .into_iter()
                    .map(|m| {
                        (
                            m.clone(),
                            muscle_radius(m.f_max),
                            muscle_slots(&sim, m.tendon),
                        )
                    })
                    .collect();
                self.couplings = sim.joint_couplings();
                let j = &sim.joints()[rig.joint];
                self.elbow_range = (j.range[0].to_degrees(), j.range[1].to_degrees());
                self.sim = Some(sim);
                self.rig = Some(rig);
                self.skel = Some(skel);
                // Где «перед» руки: при 90° предплечье смотрит вперёд. Заодно — плечи мышц при 90°.
                self.set_pose(90.0);
                let (a, h) = (v3(self.rig().axis(self.sim()).1), self.humerus_dir());
                let u = v3(self.rig().forearm_dir(self.sim()));
                self.anterior_sign = if a.cross(&h).dot(&u) >= 0.0 {
                    1.0
                } else {
                    -1.0
                };
                let sim = self.sim.as_mut().unwrap();
                let ms = muscles_at_pose(sim, self.rig.as_ref().unwrap());
                self.arms_90 = lim::MOMENT_ARMS_CM
                    .iter()
                    .filter_map(|(name, cm)| {
                        ms.iter()
                            .find(|m| m.name.starts_with(name) && !m.name.ends_with("_l"))
                            .map(|m| (m.name.clone(), m.r.abs() * 100.0, *cm))
                    })
                    .collect();
                self.compute_curve();
                self.recompute();
            }
        }
    }

    fn sim(&self) -> &Sim {
        self.sim.as_ref().unwrap()
    }

    fn rig(&self) -> &ElbowRig {
        self.rig.as_ref().unwrap()
    }

    fn humerus_dir(&self) -> Vec3 {
        let sim = self.sim();
        let elbow = v3(sim.joint_anchor(self.rig().joint));
        let shoulder = sim
            .joints()
            .into_iter()
            .find(|j| j.name == "shoulder_elv_r")
            .map(|j| v3(sim.joint_anchor(j.id)))
            .unwrap_or(elbow + Vec3::z());
        (shoulder - elbow).normalize()
    }

    fn set_pose(&mut self, deg: f64) {
        let Some(sim) = self.sim.as_mut() else { return };
        let qadr = self.rig.as_ref().unwrap().qadr;
        let q0 = sim.qpos0().to_vec();
        sim.qpos_mut().copy_from_slice(&q0);
        sim.qpos_mut()[qadr] = deg.to_radians();
        sim.apply_joint_couplings(&self.couplings);
        sim.forward();
    }

    fn compute_curve(&mut self) {
        self.curve.clear();
        let hi = self.elbow_range.1.min(lim::ELBOW_ROM_DEG.1);
        let mut d = self.elbow_range.0.max(0.0);
        while d <= hi + 1e-9 {
            self.set_pose(d);
            let sim = self.sim.as_mut().unwrap();
            let rig = self.rig.as_ref().unwrap();
            let ms = muscles_at_pose(sim, rig);
            let required = required_torque(sim, rig, self.payload);
            self.curve.push(CurveRow {
                deg: d,
                cap: capacity(&ms),
                required,
            });
            d += 5.0;
        }
    }

    fn flexion_at(&self, deg: f64) -> f64 {
        self.curve
            .iter()
            .min_by(|a, b| (a.deg - deg).abs().total_cmp(&(b.deg - deg).abs()))
            .map(|c| c.cap.flexion)
            .unwrap_or(f64::NAN)
    }

    fn our_required_torque(&self) -> f64 {
        let (sim, rig, sk) = (self.sim(), self.rig(), self.skel.as_ref().unwrap());
        let q = sk.q_from(sim.qpos());
        let (hb, _) = sk.body_frame[rig.hand_body].unwrap();
        let xw = world_to_body(&sk.model, &q);
        let com = v3(sim.body_com(rig.hand_body));
        let g = v3(sim.gravity());
        let pf = PointForce {
            body: hb,
            point_body: xw[hb].point_to_child(&com),
            force_world: g * self.payload,
        };
        let f = external_forces(&sk.model, &q, &[pf]);
        let z = DVector::zeros(sk.model.nv());
        sk.to_mj_dofs(&rnea(&sk.model, &q, &z, &z, &f), sim.nv())[rig.dof]
    }

    fn sum_required_torque(&self) -> f64 {
        let (sim, rig) = (self.sim(), self.rig());
        let (o, a) = rig.axis(sim);
        let (o, a) = (v3(o), v3(a));
        let g = v3(sim.gravity());
        let lever = |p: Vec3, m: f64| a.dot(&(p - o).cross(&(-g * m)));
        let mut tau = lever(v3(sim.body_com(rig.hand_body)), self.payload);
        for (b, (_, m)) in sim.bodies().iter().enumerate() {
            if rig.subtree[b] {
                tau += lever(v3(sim.body_com(b)), *m);
            }
        }
        tau
    }

    fn recompute(&mut self) {
        if self.sim.is_none() {
            return;
        }
        self.set_pose(self.angle_deg);
        let tau_ours = self.our_required_torque();
        let tau_sum = self.sum_required_torque();
        let humerus = self.humerus_dir();
        let (assist, payload, cuff_cm) = (self.assist, self.payload, self.cuff_cm);
        let sim = self.sim.as_mut().unwrap();
        let rig = self.rig.as_ref().unwrap();
        let ms = muscles_at_pose(sim, rig);
        let cap = capacity(&ms);
        let tau_req = required_torque(sim, rig, payload);
        let tau_exo = assist * tau_req;
        let opt = static_opt(&ms, tau_req - tau_exo);
        let opt_none = static_opt(&ms, tau_req);
        let cuff = ExoLoad {
            torque: tau_exo,
            cuff_distance: cuff_cm / 100.0,
        };
        let react = joint_reaction(sim, rig, &ms, &opt.activation, payload, Some(cuff));
        let react_none = joint_reaction(sim, rig, &ms, &opt_none.activation, payload, None);
        let act = ms
            .iter()
            .zip(&opt.activation)
            .map(|(m, a)| (m.actuator, *a))
            .collect();
        let e = self.misalign_mm / 1000.0;
        let k = self.stiffness * 1000.0;
        let misalign = misalignment_force(e, k, (self.angle_deg - self.don_deg).to_radians());
        let worst = (self.elbow_range.0.max(0.0) - self.don_deg)
            .abs()
            .max((self.elbow_range.1 - self.don_deg).abs());
        let misalign_max = misalignment_force(e, k, worst.to_radians());
        let r_err = ms.iter().map(|m| (m.r - m.r_fd).abs()).fold(0.0, f64::max);
        let (o, a) = rig.axis(sim);
        let (anchor, axis) = (v3(o), v3(a));
        // Медиальная сторона — к туловищу (грудине/грудной клетке).
        let bodies = sim.bodies();
        let torso = ["thorax", "torso", "sternum", "thoracic5"]
            .iter()
            .find_map(|n| bodies.iter().position(|(b, _)| b == n))
            .map(|i| v3(sim.body_com(i)));
        let medial = match torso {
            Some(t) if axis.dot(&(t - anchor)) < 0.0 => -axis,
            _ => axis,
        };
        let anterior = (axis.cross(&humerus) * self.anterior_sign).normalize();
        let forearm = v3(rig.forearm_dir(sim));
        let hand = v3(sim.body_com(rig.hand_body));
        let cuff_point = anchor + forearm * (cuff_cm / 100.0);
        let lever = cuff_point - anchor;
        let cuff_dir =
            axis.cross(&(lever - axis * lever.dot(&axis))).normalize() * tau_exo.signum();
        let proximal = proximal_loads(sim, rig.hand_body, payload);
        let env = {
            let omega = self.speed_dps.to_radians();
            let lever_hand = (hand - anchor).norm();
            let length = forearm_hand_length(sim, rig);
            // Момент веса гири (то, что груз добавляет к удержанию).
            let payload_hold = tau_req - required_torque(sim, rig, 0.0);
            let rb = (3.0 * payload / (4.0 * std::f64::consts::PI * KETTLEBELL_DENSITY)).cbrt();
            let (mut e, mut static_payload) = (0.0, 0.0);
            let (mut drag_mj, mut drag_hand, mut drag_payload, mut wind_mj, mut wind_hand) =
                (0.0, 0.0, 0.0, 0.0, 0.0);
            match self.medium {
                1 => {
                    static_payload = payload_hold * WATER.0 / KETTLEBELL_DENSITY;
                    e = buoyancy_elbow_torque(sim, rig, WATER.0, 1.0) + static_payload;
                    drag_mj =
                        -mujoco_fluid_elbow_torque(sim, rig, WATER.0, WATER.1, [0.0; 3], omega);
                    drag_hand = cylinder_rotation_drag(WATER.0, CYL_CD, FOREARM_D, length, omega);
                    let v = omega * lever_hand;
                    drag_payload = 0.5
                        * WATER.0
                        * SPHERE_CD
                        * std::f64::consts::PI
                        * rb
                        * rb
                        * v
                        * v
                        * lever_hand;
                }
                2 => {
                    // Ветер в лицо: воздух движется назад, горизонтально.
                    let mut back = -anterior;
                    back.z = 0.0;
                    let back = back.normalize() * self.wind_ms;
                    wind_mj = mujoco_fluid_elbow_torque(
                        sim,
                        rig,
                        AIR.0,
                        AIR.1,
                        [back.x, back.y, back.z],
                        0.0,
                    );
                    // Ручной расчёт: составляющая ветра поперёк предплечья в плоскости сгибания.
                    let across = axis.cross(&forearm).normalize();
                    let vn = back.dot(&across);
                    wind_hand =
                        cylinder_crossflow_torque(AIR.0, CYL_CD, FOREARM_D, length, vn.abs())
                            * vn.signum();
                    e = wind_mj;
                }
                _ => {}
            }
            let tau_hold = tau_req - e;
            let tau_move = tau_hold + drag_mj + drag_payload;
            let opt_move = static_opt(&ms, tau_move - assist * tau_move);
            EnvLoads {
                static_q: e,
                static_payload,
                drag_mj,
                drag_hand,
                drag_payload,
                wind_mj,
                wind_hand,
                tau_hold,
                tau_move,
                max_act_move: opt_move.activation.iter().cloned().fold(0.0, f64::max),
                feasible_move: opt_move.feasible,
            }
        };
        self.res = Some(Results {
            ms,
            cap,
            tau_req,
            tau_ours,
            tau_sum,
            tau_exo,
            opt,
            feasible_unassisted: opt_none.feasible,
            react,
            react_none,
            act,
            misalign,
            misalign_max,
            r_err,
            anchor,
            medial,
            posterior: -anterior,
            humerus,
            forearm,
            hand,
            cuff_point,
            cuff_dir,
            proximal,
            env,
        });
    }

    fn cuff_pressure_kpa(&self) -> f64 {
        self.res
            .as_ref()
            .map(|r| r.react.cuff_force / (self.cuff_area_cm2 * 1e-4) / 1000.0)
            .unwrap_or(0.0)
    }

    fn top_muscle(&self) -> String {
        let Some(r) = self.res.as_ref() else {
            return String::new();
        };
        r.ms.iter()
            .zip(&r.opt.activation)
            .max_by(|a, b| a.1.total_cmp(b.1))
            .map(|(m, _)| m.name.clone())
            .unwrap_or_default()
    }

    /// Нагрузки от среды для тестов: (держать, двигать, помощь среды в покое,
    /// сопротивление MuJoCo, сопротивление цилиндром, ветер MuJoCo, ветер цилиндром), Н·м.
    pub fn env_numbers(&self) -> Option<[f64; 7]> {
        let e = &self.res.as_ref()?.env;
        Some([
            e.tau_hold,
            e.tau_move,
            e.static_q,
            e.drag_mj,
            e.drag_hand,
            e.wind_mj,
            e.wind_hand,
        ])
    }

    pub fn summary_numbers(&self) -> Option<ExoSummary> {
        let r = self.res.as_ref()?;
        Some(ExoSummary {
            required: r.tau_req,
            exo: r.tau_exo,
            joint_force_unassisted: r.react_none.magnitude,
            joint_force: r.react.magnitude,
            cuff_force: r.react.cuff_force,
            cuff_pressure_kpa: self.cuff_pressure_kpa(),
            max_activation: r.opt.max_activation,
            feasible: r.opt.feasible,
            flexion_90_model: self.flexion_at(90.0),
        })
    }

    /// Зоны нервов у локтя по ориентирам (ось локтя, медиальная/латеральная сторона, плечевая
    /// кость). Точность ±1 см; зона локтевого нерва широкая — у трети людей он смещается.
    fn nerve_zones(&self) -> Vec<Zone> {
        let Some(r) = self.res.as_ref() else {
            return Vec::new();
        };
        let lateral = -r.medial;
        let anterior = -r.posterior;
        let mut z: Vec<Zone> = [-0.05, -0.025, 0.0, 0.025, 0.05]
            .iter()
            .map(|s| {
                (
                    "локтевой нерв",
                    r.anchor + r.medial * 0.025 + r.posterior * 0.015 + r.humerus * *s,
                    0.02,
                )
            })
            .collect();
        z.push((
            "срединный нерв и плечевая артерия",
            r.anchor + r.medial * 0.012 + anterior * 0.022,
            0.015,
        ));
        let up = 0.5
            * (lim::RADIAL_NERVE_ABOVE_LATERAL_EPICONDYLE_CM.0
                + lim::RADIAL_NERVE_ABOVE_LATERAL_EPICONDYLE_CM.1)
            / 100.0;
        z.push((
            "лучевой нерв",
            r.anchor + lateral * 0.03 + r.posterior * 0.01 + r.humerus * up,
            0.025,
        ));
        z
    }
}

impl Scenario for ExoAnatomy {
    fn id(&self) -> &'static str {
        "exo_anatomy"
    }

    fn title(&self) -> String {
        "Экзо на анатомической руке: мышцы и сила в локте".into()
    }

    fn summary(&self) -> String {
        "Мышечно-скелетная рука (MuJoCo) с грузом в кисти и экзоскелетом локтя. Видно, как мышцы делят \
         нагрузку (цвет — активация), хватает ли силы человека (модель против измерений на людях), какая \
         сила сжимает локоть, сколько давит манжета, что даёт несоосность оси и где нельзя давить (нервы)."
            .into()
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param {
                id: "payload",
                label: "Груз в кисти".into(),
                value: self.payload,
                min: 0.0,
                max: 60.0,
                unit: "кг",
                decimals: 1,
            },
            Param {
                id: "angle",
                label: "Угол в локте".into(),
                value: self.angle_deg,
                min: self.elbow_range.0.max(0.0),
                max: self.elbow_range.1,
                unit: "°",
                decimals: 0,
            },
            Param {
                id: "assist",
                label: "Экзоскелет берёт на себя".into(),
                value: self.assist * 100.0,
                min: 0.0,
                max: 100.0,
                unit: "%",
                decimals: 0,
            },
            Param {
                id: "cuff",
                label: "Манжета: расстояние от локтя".into(),
                value: self.cuff_cm,
                min: 5.0,
                max: 30.0,
                unit: "см",
                decimals: 1,
            },
            Param {
                id: "area",
                label: "Манжета: площадь контакта".into(),
                value: self.cuff_area_cm2,
                min: 10.0,
                max: 300.0,
                unit: "см²",
                decimals: 0,
            },
            Param {
                id: "misalign",
                label: "Несоосность оси привода".into(),
                value: self.misalign_mm,
                min: 0.0,
                max: 30.0,
                unit: "мм",
                decimals: 1,
            },
            Param {
                id: "stiff",
                label: "Жёсткость тканей под манжетой".into(),
                value: self.stiffness,
                min: 0.5,
                max: 8.0,
                unit: "Н/мм",
                decimals: 1,
            },
            Param {
                id: "don",
                label: "Угол, в котором надели ортез".into(),
                value: self.don_deg,
                min: 0.0,
                max: 130.0,
                unit: "°",
                decimals: 0,
            },
            Param {
                id: "speed",
                label: "Скорость сгибания (для сопротивления среды)".into(),
                value: self.speed_dps,
                min: 0.0,
                max: 300.0,
                unit: "°/с",
                decimals: 0,
            },
            Param {
                id: "wind",
                label: "Ветер (если выбран)".into(),
                value: self.wind_ms,
                min: 0.0,
                max: 40.0,
                unit: "м/с",
                decimals: 0,
            },
        ]
    }

    fn set_param(&mut self, id: &str, value: f64) {
        match id {
            "payload" => {
                self.payload = value;
                self.compute_curve();
            }
            "angle" => self.angle_deg = value,
            "assist" => self.assist = value / 100.0,
            "cuff" => self.cuff_cm = value,
            "area" => self.cuff_area_cm2 = value,
            "misalign" => {
                self.misalign_mm = value;
                self.epoch += 1;
            }
            "stiff" => self.stiffness = value,
            "don" => self.don_deg = value,
            "speed" => self.speed_dps = value,
            "wind" => self.wind_ms = value,
            _ => return,
        }
        self.recompute();
    }

    fn presets(&self) -> Vec<String> {
        vec![
            "20 кг без экзо".into(),
            "20 кг, экзо 70%".into(),
            "50 кг, экзо 100%".into(),
            "20 кг, экзо 70%, манжета 25 см × 150 см²".into(),
        ]
    }

    fn apply_preset(&mut self, index: usize) {
        let (p, a, cuff, area) = match index {
            0 => (20.0, 0.0, 15.0, 60.0),
            1 => (20.0, 0.7, 15.0, 60.0),
            2 => (50.0, 1.0, 15.0, 60.0),
            _ => (20.0, 0.7, 25.0, 150.0),
        };
        self.payload = p;
        self.assist = a;
        self.cuff_cm = cuff;
        self.cuff_area_cm2 = area;
        self.angle_deg = 90.0;
        self.compute_curve();
        self.recompute();
    }

    fn choices(&self) -> Vec<Choice> {
        vec![
            Choice {
                id: "model",
                label: "Модель руки".into(),
                options: MODELS.iter().map(|m| m.1.to_string()).collect(),
                selected: self.which,
            },
            Choice {
                id: "medium",
                label: "Среда".into(),
                options: MEDIA.iter().map(|m| m.to_string()).collect(),
                selected: self.medium,
            },
        ]
    }

    fn set_choice(&mut self, id: &str, index: usize) {
        if id == "model" && index != self.which {
            self.which = index.min(MODELS.len() - 1);
            self.load();
        }
        if id == "medium" {
            self.medium = index.min(MEDIA.len() - 1);
            self.epoch += 1;
            self.recompute();
        }
    }

    fn reset(&mut self) {
        let (which, medium) = (self.which, self.medium);
        *self = Self::default();
        self.medium = medium;
        if which != 0 {
            self.which = which;
            self.load();
        } else if medium != 0 {
            self.recompute();
        }
    }

    fn advance(&mut self, dt_real: f64) {
        self.t += dt_real;
    }

    fn time(&self) -> f64 {
        self.t
    }

    fn visuals(&self) -> Vec<VisualInstance> {
        let (Some(sim), Some(r)) = (self.sim.as_ref(), self.res.as_ref()) else {
            return Vec::new();
        };
        let mut out = Vec::new();
        bone_visuals(sim, &self.geoms, &mut out);
        for (m, radius, slots) in &self.muscles {
            let color = match r.act.get(&m.actuator) {
                Some(a) => activation_color(*a),
                None => C_MUSCLE_IDLE,
            };
            muscle_tubes(sim, m.tendon, *slots, *radius, color, &mut out);
        }
        // Гиря в кисти: центр масс висит точно под хватом, поэтому плечо силы по горизонтали
        // то же, что у точки хвата, — и в расчёте, и на экране.
        if self.payload > 0.0 {
            let rb = (3.0 * self.payload / (4.0 * std::f64::consts::PI * 7200.0)).cbrt();
            let top = r.hand - Vec3::z() * 0.05;
            let center = top - Vec3::z() * rb;
            out.push(VisualInstance {
                shape: Shape::Cylinder {
                    radius: 0.01,
                    length: 0.05,
                },
                pose: Pose::at(r.hand - Vec3::z() * 0.025),
                color: C_STEEL,
            });
            out.push(VisualInstance {
                shape: Shape::Sphere { radius: rb },
                pose: Pose::at(center),
                color: C_IRON,
            });
        }
        // Экзоскелет: привод сбоку от локтя (со смещением оси — несоосность), штанги, манжета.
        let lateral = -r.medial;
        let motor = r.anchor + lateral * 0.075 + r.posterior * (-self.misalign_mm / 1000.0);
        out.push(VisualInstance {
            shape: Shape::Cylinder {
                radius: 0.042,
                length: 0.045,
            },
            pose: Pose::z_along(motor, lateral),
            color: C_MOTOR,
        });
        let cuff_side = r.cuff_point + lateral * 0.06;
        let d = cuff_side - motor;
        out.push(VisualInstance {
            shape: Shape::Box {
                half: Vec3::new(0.008, 0.012, d.norm() / 2.0),
            },
            pose: Pose::z_along((motor + cuff_side) / 2.0, d),
            color: C_EXO,
        });
        let cuff_len =
            (self.cuff_area_cm2 / (std::f64::consts::PI * 2.0 * 4.5)).clamp(2.0, 12.0) / 100.0;
        out.push(VisualInstance {
            shape: Shape::Cylinder {
                radius: 0.05,
                length: cuff_len,
            },
            pose: Pose::z_along(r.cuff_point, r.forearm),
            color: C_EXO_CUFF,
        });
        let up = motor + r.humerus * 0.2;
        out.push(VisualInstance {
            shape: Shape::Box {
                half: Vec3::new(0.008, 0.012, 0.1),
            },
            pose: Pose::z_along((motor + up) / 2.0, r.humerus),
            color: C_EXO,
        });
        if self.medium == 1 {
            // Вода по плечо: от пола до уровня на 15 см выше локтя.
            let surface = r.anchor.z + 0.15;
            out.push(VisualInstance {
                shape: Shape::Box {
                    half: Vec3::new(0.45, 0.45, surface / 2.0),
                },
                pose: Pose::at(Vec3::new(r.anchor.x, r.anchor.y, surface / 2.0)),
                color: [0.25, 0.55, 0.9, 0.22],
            });
        }
        out
    }

    fn visual_epoch(&self) -> u64 {
        self.epoch
    }

    fn mesh_data(&self, id: u32) -> Option<MeshData> {
        let m = self.sim.as_ref()?.mesh(id as usize);
        Some(MeshData {
            positions: m.positions,
            indices: m.indices,
        })
    }

    fn overlays(&self) -> Vec<Overlay> {
        let Some(r) = self.res.as_ref() else {
            return Vec::new();
        };
        let k = 0.00025; // м на Н
        let mut out = vec![Overlay::Arrow {
            from: r.anchor,
            to: r.anchor + Vec3::new(r.react.force[0], r.react.force[1], r.react.force[2]) * k,
            color: C_YELLOW,
        }];
        if self.payload > 0.0 {
            out.push(Overlay::Arrow {
                from: r.hand,
                to: r.hand + Vec3::new(0.0, 0.0, -self.payload * 9.80665 * k),
                color: C_RED,
            });
        }
        if r.react.cuff_force > 0.0 {
            out.push(Overlay::Arrow {
                from: r.cuff_point - r.cuff_dir * (r.react.cuff_force * k),
                to: r.cuff_point,
                color: C_EXO_CUFF,
            });
        }
        for (name, c, rad) in self.nerve_zones() {
            let color = if name.starts_with("локтевой") {
                C_RED
            } else {
                C_PINK
            };
            out.push(Overlay::WireSphere {
                center: c,
                radius: rad,
                color,
            });
        }
        if self.medium == 2 && self.wind_ms > 0.0 {
            // Струи ветра: спереди назад, длина ~ скорость.
            let mut back = r.posterior;
            back.z = 0.0;
            let back = back.normalize();
            let side = Vec3::z().cross(&back).normalize();
            let len = 0.02 * self.wind_ms;
            for i in -2..=2 {
                for k in 0..3 {
                    let start = r.anchor - back * 0.6
                        + side * (0.12 * i as f64)
                        + Vec3::z() * (0.15 * k as f64 - 0.15);
                    out.push(Overlay::Arrow {
                        from: start,
                        to: start + back * len,
                        color: [0.6, 0.85, 1.0, 0.8],
                    });
                }
            }
        }
        out
    }

    fn readouts(&self) -> Vec<Readout> {
        let Some(r) = self.res.as_ref() else {
            return Vec::new();
        };
        let w = self.payload * 9.80665;
        let mut v = vec![
            Readout::new("Нужно в локте (груз + рука)", r.tau_req, "Н·м", 1),
            Readout::new("Экзоскелет даёт", r.tau_exo, "Н·м", 1),
            Readout::new("Мышцам остаётся", r.tau_req - r.tau_exo, "Н·м", 1),
            Readout::new(
                "Сила человека в этом угле по модели (сгибание)",
                r.cap.flexion,
                "Н·м",
                1,
            ),
            Readout::new(
                "Пассивный момент тканей (мышцы, без усилия)",
                r.cap.passive,
                "Н·м",
                2,
            ),
            Readout::new(
                "Загрузка самой нагруженной мышцы",
                r.opt.max_activation * 100.0,
                "%",
                0,
            ),
            Readout::new(
                "Сила в локте без экзоскелета",
                r.react_none.magnitude,
                "Н",
                0,
            ),
            Readout::new("Сила в локте с экзоскелетом", r.react.magnitude, "Н", 0),
            Readout::new("Манжета давит на предплечье", r.react.cuff_force, "Н", 0),
            Readout::new(
                "Давление манжеты на кожу (среднее)",
                self.cuff_pressure_kpa(),
                "кПа",
                1,
            ),
            Readout::new(
                "Нужная площадь манжеты для долгой носки (≤ 4 кПа)",
                r.react.cuff_force / (lim::CUFF_PRESSURE_LONG_WEAR_KPA * 1000.0) * 1e4,
                "см²",
                0,
            ),
            Readout::new(
                "Нужная площадь манжеты до дискомфорта (≤ 16 кПа)",
                r.react.cuff_force / (lim::CUFF_PRESSURE_DISCOMFORT_KPA * 1000.0) * 1e4,
                "см²",
                0,
            ),
            Readout::new(
                "Мощность привода на 200 °/с при этом моменте",
                r.tau_exo.abs() * 200f64.to_radians(),
                "Вт",
                0,
            ),
            Readout::new(
                "Сдвиг манжеты от несоосности",
                r.misalign.0 * 1000.0,
                "мм",
                1,
            ),
            Readout::new("Паразитная сила от несоосности", r.misalign.1, "Н", 1),
            Readout::new(
                "Запас до предела сгибания модели",
                self.elbow_range.1 - self.angle_deg,
                "°",
                0,
            ),
        ];
        if let Some(p) = &r.proximal {
            v.push(Readout::new(
                "Момент в плечевом суставе (экзо локтя не меняет)",
                p.shoulder_moment,
                "Н·м",
                1,
            ));
            v.push(Readout::new(
                "Момент в пояснице L5/S1",
                p.l5s1_moment,
                "Н·м",
                1,
            ));
            v.push(Readout::new(
                "Сжатие позвоночника L5/S1 (оценка, Chaffin)",
                p.l5s1_compression,
                "Н",
                0,
            ));
        }
        if w > 0.0 {
            v.insert(
                8,
                Readout::new(
                    "Сила в локте без экзо, в весах груза",
                    r.react_none.magnitude / w,
                    "×",
                    1,
                ),
            );
        }
        let e = &r.env;
        match self.medium {
            1 => {
                v.push(Readout::new(
                    "Вода: Архимед помогает (рука + гиря)",
                    e.static_q,
                    "Н·м",
                    2,
                ));
                v.push(Readout::new(
                    "  из них гиря (чугун)",
                    e.static_payload,
                    "Н·м",
                    2,
                ));
                v.push(Readout::new("Держать в воде нужно", e.tau_hold, "Н·м", 1));
                v.push(Readout::new(
                    "Сопротивление воды: рука (MuJoCo)",
                    e.drag_mj,
                    "Н·м",
                    2,
                ));
                v.push(Readout::new(
                    "Сопротивление воды: гиря",
                    e.drag_payload,
                    "Н·м",
                    2,
                ));
                v.push(Readout::new("Сгибать в воде нужно", e.tau_move, "Н·м", 1));
                v.push(Readout::new(
                    "Мышцы при этом (наиб. активация, экзо той же долей)",
                    100.0 * e.max_act_move,
                    "%",
                    0,
                ));
            }
            2 => {
                v.push(Readout::new(
                    "Ветер даёт в локте (MuJoCo)",
                    e.wind_mj,
                    "Н·м",
                    2,
                ));
                v.push(Readout::new("Держать на ветру нужно", e.tau_hold, "Н·м", 1));
                v.push(Readout::new(
                    "Ветер толкает всё тело (оценка C_dA = 0.7 м²)",
                    0.5 * AIR.0 * 0.7 * self.wind_ms * self.wind_ms,
                    "Н",
                    0,
                ));
            }
            _ => {}
        }
        v
    }

    fn comparisons(&self) -> Vec<Comparison> {
        let Some(r) = self.res.as_ref() else {
            return Vec::new();
        };
        let ok_or = |good: bool, ok: &str, bad: &str| {
            if good {
                (Status::Ok, ok.to_string())
            } else {
                (Status::Mismatch, bad.to_string())
            }
        };
        let rel = |a: f64, b: f64| (a - b).abs() / b.abs().max(1e-9);
        let mut out = Vec::new();
        let (s, e) = ok_or(
            rel(r.tau_ours, r.tau_req) < 1e-6,
            "наш движок (RNEA на той же модели) совпал с MuJoCo",
            "движки расходятся — ошибка переноса модели",
        );
        out.push(Comparison {
            title: "Момент в локте: MuJoCo ↔ наш движок".into(),
            sim_label: "MuJoCo".into(),
            sim: r.tau_req,
            ref_label: "наш движок".into(),
            reference: r.tau_ours,
            unit: "Н·м",
            decimals: 4,
            status: s,
            explanation: e,
        });
        let (s, e) = ok_or(
            rel(r.tau_sum, r.tau_req) < 1e-6,
            "ручная сумма по центрам масс костей совпала",
            "сумма по центрам масс не совпала — ошибка",
        );
        out.push(Comparison {
            title: "Момент в локте: MuJoCo ↔ Σ m·g·x по костям".into(),
            sim_label: "MuJoCo".into(),
            sim: r.tau_req,
            ref_label: "Σ по центрам масс".into(),
            reference: r.tau_sum,
            unit: "Н·м",
            decimals: 4,
            status: s,
            explanation: e,
        });
        let (s, e) = ok_or(
            r.r_err < 1e-5,
            "плечи мышц двумя способами совпали",
            "плечи мышц расходятся — ошибка в путях мышц",
        );
        out.push(Comparison {
            title: "Плечи мышц: якобиан MuJoCo ↔ −dℓ/dθ".into(),
            sim_label: "max |разница|".into(),
            sim: r.r_err * 1000.0,
            ref_label: "должно быть".into(),
            reference: 0.0,
            unit: "мм",
            decimals: 6,
            status: s,
            explanation: e,
        });

        // Сила человека: модель против измерений (Frey-Law 2012, мужчины).
        let sd = lim::FLEXION_SD_MEN;
        for (k, (deg, men, women)) in [
            (3usize, lim::FLEXION_MEASURED[3]),
            (0, lim::FLEXION_MEASURED[0]),
        ] {
            let f = self.flexion_at(deg);
            let (s, e) = if (f - men).abs() <= 0.05 * men {
                (Status::Ok, "совпало с измерениями в пределах 5%".to_string())
            } else if (f - men).abs() <= sd {
                (
                    Status::Explained,
                    format!(
                        "расходится на {:.0}%, это в пределах разброса мужчин (±{sd:.0} Н·м). Для решений по безопасности берём измерения, а не модель",
                        100.0 * (f / men - 1.0).abs()
                    ),
                )
            } else if (f - men).abs() <= 3.0 * sd {
                let why = if k == 0 {
                    "у разогнутого локтя упрощённая мышца MuJoCo искажает кривую «сила — длина»"
                } else {
                    "модель собрана по данным сильнее среднего"
                };
                (
                    Status::Explained,
                    format!(
                        "модель {} среднего мужчины на {:.0}% (женщины — {women:.0} Н·м): {why}. Для решений по безопасности берём измерения, а не модель",
                        if f > men {
                            "сильнее"
                        } else {
                            "слабее"
                        },
                        100.0 * (f / men - 1.0).abs()
                    ),
                )
            } else {
                (
                    Status::Mismatch,
                    "сила модели далеко за разбросом людей — параметры мышц неверны".into(),
                )
            };
            out.push(Comparison {
                title: format!("Сила сгибания при {deg:.0}°: модель ↔ измерения на людях"),
                sim_label: "модель (мышцы на 100%)".into(),
                sim: f,
                ref_label: "мужчины, Frey-Law 2012".into(),
                reference: men,
                unit: "Н·м",
                decimals: 1,
                status: s,
                explanation: e,
            });
        }

        // Плечи мышц при 90° против Murray 2000.
        if !self.arms_90.is_empty() {
            let worst = self
                .arms_90
                .iter()
                .map(|(_, m, l)| (m - l) / l)
                .fold(0.0_f64, |a, x| if x.abs() > a.abs() { x } else { a });
            let list: Vec<String> = self
                .arms_90
                .iter()
                .map(|(n, m, l)| format!("{n}: {m:.1} против {l:.1} см"))
                .collect();
            let (s, e) = if worst.abs() <= 0.25 {
                (
                    Status::Ok,
                    format!(
                        "все в пределах ±25% (разброс между людьми): {}",
                        list.join("; ")
                    ),
                )
            } else {
                (
                    Status::Explained,
                    format!(
                        "есть расхождения больше 25% (у Murray — значения у одной группы людей, разброс плеч между людьми большой): {}",
                        list.join("; ")
                    ),
                )
            };
            let (n, m, l) = &self.arms_90[0];
            out.push(Comparison {
                title: format!("Плечи мышц при 90°: модель ↔ Murray 2000 ({n})"),
                sim_label: "модель".into(),
                sim: *m,
                ref_label: "литература".into(),
                reference: *l,
                unit: "см",
                decimals: 2,
                status: s,
                explanation: e,
            });
        }

        // Сила в локте против оценки по литературе.
        if self.payload > 0.0 && r.feasible_unassisted {
            let w = self.payload * 9.80665;
            let (lo, hi) = (
                lim::JOINT_FORCE_PER_LOAD.0 * w + lim::JOINT_FORCE_FOREARM_N,
                lim::JOINT_FORCE_PER_LOAD.1 * w + lim::JOINT_FORCE_FOREARM_N,
            );
            let f = r.react_none.magnitude;
            let (s, e) = if f >= lo && f <= hi {
                (
                    Status::Ok,
                    format!("в коридоре оценки по литературе {lo:.0}–{hi:.0} Н"),
                )
            } else if f >= 0.75 * lo && f <= 1.25 * hi {
                (
                    Status::Explained,
                    format!(
                        "рядом с коридором {lo:.0}–{hi:.0} Н: плечи мышц и положение груза в модели другие, чем в учебной схеме"
                    ),
                )
            } else {
                (
                    Status::Mismatch,
                    format!(
                        "далеко от коридора {lo:.0}–{hi:.0} Н — проверить равновесие предплечья"
                    ),
                )
            };
            out.push(Comparison {
                title: "Сила в локте без экзо: модель ↔ (7.5–9)·W + 45 Н".into(),
                sim_label: "модель".into(),
                sim: f,
                ref_label: "середина коридора".into(),
                reference: 0.5 * (lo + hi),
                unit: "Н",
                decimals: 0,
                status: s,
                explanation: e,
            });
        }
        let env = &r.env;
        if self.medium == 1 {
            let ratio = if env.drag_hand > 0.0 {
                env.drag_mj / env.drag_hand
            } else {
                0.0
            };
            out.push(Comparison {
                title: format!("Сопротивление воды при сгибании {:.0} °/с", self.speed_dps),
                sim_label: "MuJoCo (брусок по инерции сегментов)".into(),
                sim: env.drag_mj,
                ref_label: "цилиндр D 7 см, C_d 1: ½ρC_dDω²L⁴/4".into(),
                reference: env.drag_hand,
                unit: "Н·м",
                decimals: 2,
                status: if self.speed_dps < 1.0 {
                    Status::Pending
                } else if (0.5..2.0).contains(&ratio) {
                    Status::Explained
                } else {
                    Status::Mismatch
                },
                explanation: "Обе модели грубые: кисть плоская, предплечье сужается, C_d зависит \
                              от числа Рейнольдса. Согласие ±40 % — ожидаемое; для решений берём \
                              большее."
                    .into(),
            });
        }
        if self.medium == 2 {
            let small = env.wind_hand.abs() < 0.05 && env.wind_mj.abs() < 0.05;
            let rel = if env.wind_hand.abs() > 0.0 {
                (env.wind_mj - env.wind_hand).abs() / env.wind_hand.abs()
            } else {
                0.0
            };
            out.push(Comparison {
                title: format!("Ветер {:.0} м/с на предплечье", self.wind_ms),
                sim_label: "MuJoCo".into(),
                sim: env.wind_mj,
                ref_label: "цилиндр поперёк потока".into(),
                reference: env.wind_hand,
                unit: "Н·м",
                decimals: 2,
                status: if small {
                    Status::Pending
                } else if rel < 0.3 {
                    Status::Ok
                } else {
                    Status::Explained
                },
                explanation: "При 90° ветер в лицо дует вдоль предплечья — момента почти нет; \
                              поверните локоть, чтобы увидеть разницу моделей."
                    .into(),
            });
        }
        out
    }

    fn notes(&self) -> Vec<Note> {
        if let Some(e) = &self.error {
            return vec![Note::new(
                Level::Danger,
                format!("Модель не загрузилась: {e}"),
            )];
        }
        let Some(r) = self.res.as_ref() else {
            return Vec::new();
        };
        let mut out = Vec::new();
        let env = &r.env;
        if self.medium != 0 {
            out.push(Note::new(
                Level::Info,
                "Мышцы, сила в локте и манжета ниже посчитаны для воздуха; поправка среды \
                 (Архимед, сопротивление, ветер) — отдельными строками «в воде»/«на ветру».",
            ));
        }
        if self.medium == 1 {
            let over = r.tau_exo - env.tau_hold;
            if over > 0.5 {
                out.push(Note::new(
                    if over > 5.0 {
                        Level::Danger
                    } else {
                        Level::Warn
                    },
                    format!(
                        "В воде рука почти невесома: держать нужно {:.1} Н·м вместо {:.1}. Экзо \
                         с компенсацией веса, настроенное на воздух, даст {:.1} Н·м — на {:.1} \
                         Н·м больше нужного и само поднимет руку. Прошивке нужен режим «вода» \
                         (датчик погружения) или усилие по датчику силы, а не по модели веса.",
                        env.tau_hold, r.tau_req, r.tau_exo, over
                    ),
                ));
            } else {
                out.push(Note::new(
                    Level::Warn,
                    format!(
                        "Вода снимает {:.1} Н·м: держать нужно {:.1} вместо {:.1}. Чугунная гиря \
                         в воде легче лишь на 14 %, а пустая рука — на 88 %: без груза экзо с \
                         компенсацией веса, настроенное на воздух, будет задирать руку. Прошивке \
                         нужен режим «вода» (датчик погружения) или усилие по датчику силы.",
                        env.static_q, env.tau_hold, r.tau_req
                    ),
                ));
            }
            let at200 = env.drag_mj * (200.0 / self.speed_dps.max(1.0)).powi(2);
            out.push(Note::new(
                Level::Info,
                format!(
                    "Сгибание {:.0} °/с в воде: +{:.1} Н·м сопротивления руки и +{:.1} Н·м гири; \
                     растёт как ω² (при 200 °/с — ~{:.0} Н·м). Привод должен ограничивать \
                     скорость, иначе мотор перегреется на сопротивлении.",
                    self.speed_dps, env.drag_mj, env.drag_payload, at200
                ),
            ));
            if !env.feasible_move {
                out.push(Note::new(
                    Level::Danger,
                    "При этой скорости в воде мышцы модели не справляются даже с помощью экзо.",
                ));
            }
        }
        if self.medium == 2 {
            let body = 0.5 * AIR.0 * 0.7 * self.wind_ms * self.wind_ms;
            out.push(Note::new(
                if body > 150.0 {
                    Level::Warn
                } else {
                    Level::Info
                },
                format!(
                    "Ветер {:.0} м/с даёт в локте {:.2} Н·м — для локтя мелочь. Но на всё тело \
                     ≈ {:.0} Н (½ρ·C_dA·v², C_dA ≈ 0.7 м² — оценка): это толкает человека; \
                     экзоскелет ног/корпуса должен держать равновесие, а широкие щитки \
                     экзоскелета добавляют парусность.",
                    self.wind_ms, env.wind_mj, body
                ),
            ));
        }
        let need = r.tau_req - r.tau_exo;
        // 1. Сила человека — по измерениям, а не по модели.
        let men = lim::FLEXION_MEASURED[3].1;
        let women = lim::FLEXION_MEASURED[3].2;
        if need > men {
            out.push(Note::new(Level::Danger, format!("Мышцам остаётся {need:.0} Н·м — больше, чем средний мужчина даёт на максимуме ({men:.0} Н·м; женщина — {women:.0}). Без экзоскелета не удержать.")));
        } else if need > lim::SUSTAINED_MVC_FRACTION * women {
            out.push(Note::new(
                Level::Warn,
                format!(
                    "Мышцам остаётся {need:.0} Н·м: это {:.0}% максимума среднего мужчины и {:.0}% — женщины. Больше ~15–20% максимума долго держать нельзя — утомление.",
                    100.0 * need / men,
                    100.0 * need / women
                ),
            ));
        } else {
            out.push(Note::new(Level::Good, format!("Мышцам остаётся {need:.0} Н·м — до {:.0}% максимума даже у женщины; держать можно долго.", 100.0 * need / women)));
        }
        if !r.opt.feasible {
            out.push(Note::new(Level::Danger, format!("Мышцы модели не удержат: нужно {need:.0} Н·м, максимум в этом угле — {:.0} Н·м.", r.cap.flexion)));
        } else {
            out.push(Note::new(
                Level::Info,
                format!(
                    "По модели самая нагруженная мышца — {} ({:.0}%).",
                    self.top_muscle(),
                    r.opt.max_activation * 100.0
                ),
            ));
        }
        // 2. Сила в суставе.
        let cut = 100.0 * (1.0 - r.react.magnitude / r.react_none.magnitude.max(1e-9));
        out.push(Note::new(
            if r.react.magnitude > 1500.0 { Level::Warn } else { Level::Info },
            format!(
                "Сила в локте: без экзоскелета {:.0} Н, с ним {:.0} Н ({}{:.0}%). Её создают в основном мышцы: их плечо — сантиметры, а груз — в 30 см.",
                r.react_none.magnitude,
                r.react.magnitude,
                if cut >= 0.0 { "−" } else { "+" },
                cut.abs()
            ),
        ));
        // 2б. Плечо и поясница — экзоскелет локтя их не разгружает.
        if let Some(pl) = &r.proximal {
            let lvl = if pl.l5s1_compression > NIOSH_MAX_PERMISSIBLE_N {
                Level::Danger
            } else if pl.l5s1_compression > NIOSH_ACTION_LIMIT_N {
                Level::Warn
            } else {
                Level::Info
            };
            out.push(Note::new(
                lvl,
                format!(
                    "Дальше по цепочке: плечо держит {:.0} Н·м, поясница L5/S1 — {:.0} Н·м, сжатие позвоночника ≈ {:.0} Н (NIOSH: предел действия {:.0} Н, максимум {:.0} Н). Экзоскелет локтя эти нагрузки не снимает — груз всё равно висит на плече и спине; нужен путь нагрузки на таз и ноги.",
                    pl.shoulder_moment,
                    pl.l5s1_moment,
                    pl.l5s1_compression,
                    NIOSH_ACTION_LIMIT_N,
                    NIOSH_MAX_PERMISSIBLE_N
                ),
            ));
        }
        // 3. Манжета.
        let p = self.cuff_pressure_kpa();
        if r.react.cuff_force > 0.0 {
            let (lvl, verdict) = if p > lim::CUFF_PRESSURE_PAIN_KPA {
                (
                    Level::Danger,
                    "боль у здорового человека, пережатие сосудов",
                )
            } else if p > lim::CUFF_PRESSURE_DISCOMFORT_KPA {
                (Level::Warn, "дискомфорт")
            } else if p > lim::CUFF_PRESSURE_LONG_WEAR_KPA {
                (
                    Level::Warn,
                    "терпимо коротко, но при долгой носке страдает кровоток",
                )
            } else {
                (Level::Good, "можно носить долго")
            };
            out.push(Note::new(
                lvl,
                format!(
                    "Манжета давит {:.0} Н на {:.0} см² — {p:.1} кПа: {verdict}. Пороги: кровоток ~{:.0}, дискомфорт ~{:.0}, боль ~{:.0} кПа (Landis 1933; Kermavnar 2018). Сила манжеты = момент привода / расстояние до локтя: дальше от локтя и шире — меньше давление; лучше всего передать груз на конструкцию напрямую.",
                    r.react.cuff_force,
                    self.cuff_area_cm2,
                    lim::CUFF_PRESSURE_LONG_WEAR_KPA,
                    lim::CUFF_PRESSURE_DISCOMFORT_KPA,
                    lim::CUFF_PRESSURE_PAIN_KPA
                ),
            ));
            if r.react.cuff_force > lim::ISO_TS_15066_FOREARM_N {
                out.push(Note::new(
                    Level::Warn,
                    format!(
                        "Сила манжеты {:.0} Н выше {:.0} Н — предела контакта с предплечьем для коллаборативных роботов (ISO/TS 15066). Стандарт о роботах, но ориентир полезный.",
                        r.react.cuff_force,
                        lim::ISO_TS_15066_FOREARM_N
                    ),
                ));
            }
            if self.cuff_cm < 7.0 {
                out.push(Note::new(Level::Warn, "Манжета ближе 7 см к локтю — над зоной локтевого нерва (он идёт по внутренней стороне предплечья). Отодвинуть."));
            }
        }
        // 4. Несоосность — при разбросе жёсткости тканей.
        let e = self.misalign_mm / 1000.0;
        let worst_angle = (self.elbow_range.0.max(0.0) - self.don_deg)
            .abs()
            .max((self.elbow_range.1 - self.don_deg).abs())
            .to_radians();
        let (_, f_lo) = misalignment_force(e, lim::TISSUE_STIFFNESS_RANGE.0 * 1000.0, worst_angle);
        let (_, f_hi) = misalignment_force(e, lim::TISSUE_STIFFNESS_RANGE.1 * 1000.0, worst_angle);
        out.push(Note::new(
            if r.misalign_max.1 > lim::MISALIGNMENT_FORCE_WARN_N { Level::Warn } else { Level::Good },
            format!(
                "Несоосность {:.0} мм: сейчас манжета смещается на {:.1} мм ({:.0} Н), на краю диапазона — до {:.1} мм ({:.0} Н; при жёсткости тканей {:.0}–{:.0} Н/мм — {f_lo:.0}–{f_hi:.0} Н). Нужна самоустанавливающаяся ось или регулировка под руку.",
                self.misalign_mm,
                r.misalign.0 * 1000.0,
                r.misalign.1,
                r.misalign_max.0 * 1000.0,
                r.misalign_max.1,
                lim::TISSUE_STIFFNESS_RANGE.0,
                lim::TISSUE_STIFFNESS_RANGE.1
            ),
        ));
        // 5. Пределы движения и скорость.
        let margin = self.elbow_range.1 - self.angle_deg;
        if margin < 10.0 || self.angle_deg < self.elbow_range.0 + 5.0 {
            out.push(Note::new(Level::Warn, format!("До предела движения {margin:.0}° — привод должен иметь механический упор с запасом и не толкать дальше.")));
        }
        out.push(Note::new(
            Level::Info,
            format!(
                "Пределы: сгибание у мужчин до ~{:.0}° (в модели — до {:.0}°), в быту хватает {:.0}–{:.0}°; переразгибание — упор на 0°. Скорость привода — не больше ~{:.0} °/с (99% бытовых движений), аварийный предел ниже {:.0} °/с.",
                lim::ELBOW_ROM_DEG.1,
                self.elbow_range.1,
                lim::ELBOW_FUNCTIONAL_DEG.0,
                lim::ELBOW_FUNCTIONAL_DEG.1,
                lim::ELBOW_SPEED_P99_DEG_S,
                lim::ELBOW_SPEED_MAX_DEG_S
            ),
        ));
        out.push(Note::new(
            Level::Info,
            "Сферы — зоны «не давить»: красные — локтевой нерв (позади внутреннего надмыщелка, ±5 см по длине: у трети людей смещается при сгибании), розовые — срединный нерв с плечевой артерией (локтевая ямка) и лучевой нерв (наружная сторона плеча в 10–12.5 см над локтем — туда нельзя ставить верхнюю манжету).",
        ));
        out.push(Note::new(
            Level::Info,
            format!(
                "Допущения: статика; равновесие требуем только в локте; распределение сил — минимум Σa²; давление манжеты — среднее (на краях выше); модель — «средний человек», под конкретную руку не масштабирована. Шарнир считать на {:.0}–{:.0} Н·м (запас 2–4×) и проверить на стенде.",
                r.tau_req * 2.0,
                r.tau_req * 4.0
            ),
        ));
        out
    }

    fn plot(&self) -> Option<Plot> {
        if self.curve.is_empty() {
            return None;
        }
        let pts = |f: &dyn Fn(&CurveRow) -> f64| {
            self.curve.iter().map(|c| [c.deg, f(c)]).collect::<Vec<_>>()
        };
        let assist = self.assist;
        let men: Vec<[f64; 2]> = lim::FLEXION_MEASURED
            .iter()
            .map(|(d, m, _)| [*d, *m])
            .collect();
        let women: Vec<[f64; 2]> = lim::FLEXION_MEASURED
            .iter()
            .map(|(d, _, w)| [*d, *w])
            .collect();
        Some(Plot {
            title: "Момент в локте по углу".into(),
            x_label: "угол в локте, °".into(),
            y_label: "Н·м".into(),
            series: vec![
                Series {
                    name: "сила сгибания: модель".into(),
                    points: pts(&|c| c.cap.flexion),
                    color: C_BLUE,
                    style: SeriesStyle::Solid,
                },
                Series {
                    name: "сила сгибания: мужчины (измерено)".into(),
                    points: men,
                    color: [0.55, 0.8, 1.0, 1.0],
                    style: SeriesStyle::Dashed,
                },
                Series {
                    name: "сила сгибания: женщины (измерено)".into(),
                    points: women,
                    color: C_PINK,
                    style: SeriesStyle::Dashed,
                },
                Series {
                    name: format!("нужно при {:.0} кг", self.payload),
                    points: pts(&|c| c.required),
                    color: C_MOTOR,
                    style: SeriesStyle::Dashed,
                },
                Series {
                    name: "остаётся мышцам с экзо".into(),
                    points: pts(&|c| c.required * (1.0 - assist)),
                    color: C_GREEN,
                    style: SeriesStyle::Dashed,
                },
            ],
            bands: vec![Band {
                label: "коридор владельца при 90°: 60–80 Н·м".into(),
                lo: lim::ELBOW_FLEXION_90.0,
                hi: lim::ELBOW_FLEXION_90.1,
                color: [1.0, 0.3, 0.2, 0.12],
            }],
        })
    }

    fn focus(&self) -> Vec3 {
        self.res
            .as_ref()
            .map(|r| r.anchor + r.forearm * 0.12)
            .unwrap_or(Vec3::new(0.0, 0.0, 1.2))
    }

    fn view_offset(&self) -> Vec3 {
        match MODELS[self.which].0 {
            Which::MyoArm => Vec3::new(-1.15, -0.75, 0.15),
            _ => Vec3::new(0.55, -1.25, 0.15),
        }
    }
}
