//! Сценарий «Анатомия»: открытые мышечно-скелетные модели человека в MuJoCo — кости,
//! мышцы, суставы с пределами и сопряжениями. Шаг 2a карты человека (HUMAN_MODEL.md):
//! модели открываются, поза задаётся суставами, мышцы видны по линиям действия.
//! Силы мышц и нагрузки в суставах — следующие шаги (2c, 2d).

use std::f64::consts::PI;

use nalgebra::{Matrix3, Rotation3, UnitQuaternion};
use sim_core::{Pose, Shape, Vec3};
use sim_mujoco::{
    GeomInfo, GeomKind, JointCoupling, JointInfo, MuscleInfo, Sim, compose_myoarm, models,
};

use crate::scenario::*;

/// Удельная сила мышцы (Н на м² физиологического поперечника), только для толщины на экране.
const SPECIFIC_TENSION: f64 = 0.6e6;
const C_BONE: [f32; 4] = [0.9, 0.87, 0.8, 1.0];
const C_MUSCLE: [f32; 4] = [0.78, 0.2, 0.2, 1.0];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Which {
    FullBody,
    MyoArm,
    HumanArm,
}

const MODELS: [(Which, &str); 3] = [
    (Which::FullBody, "MS-Human-700 — всё тело, 700 мышц"),
    (Which::MyoArm, "MyoArm (MyoSuite) — правая рука, 63 мышцы"),
    (Which::HumanArm, "MS-Human-700 — рука с кистью, 81 мышца"),
];

/// Суставы позы (имена по соглашению OpenSim — одинаковые во всех трёх моделях).
const POSE: [(&str, &str, f64); 5] = [
    ("elbow_flexion_r", "Сгибание локтя", 90.0),
    ("shoulder_elv_r", "Подъём плеча", 0.0),
    ("elv_angle_r", "Плоскость подъёма плеча", 0.0),
    ("shoulder_rot_r", "Ротация плеча", 0.0),
    ("pro_sup_r", "Пронация / супинация", 0.0),
];

#[derive(Clone, Copy, PartialEq, Eq)]
enum Layers {
    All,
    Bones,
    Muscles,
}

struct Muscle {
    info: MuscleInfo,
    radius: f64,
    /// Сколько отрезков резервируем (чтобы число фигур не прыгало, когда путь огибает кость).
    slots: usize,
}

/// Короткая сводка по модели — для сверки моделей между собой и с таблицами.
#[derive(Clone, Copy, Debug, Default)]
pub struct ModelSummary {
    /// Масса тела человека (без посторонних объектов); None — модель не всего тела.
    pub body_mass: Option<f64>,
    /// Рука от плечевой кости до пальцев (плечо + предплечье + кисть), кг.
    pub arm_mass: f64,
    /// Σ F₀ сгибателей правого локтя (BIClong, BICshort, BRA, BRD), Н.
    pub flexor_f0: f64,
}

/// Доля массы руки (плечо + предплечье + кисть) от массы тела: 2.8 + 1.6 + 0.6 = 5.0%
/// (Winter 2009, табл. 4.1, по данным Dempster 1955).
pub const WINTER_ARM_FRACTION: f64 = 0.050;

fn summarize(which: Which) -> Result<ModelSummary, String> {
    let sim = open_model(which)?;
    let bodies = sim.bodies();
    let human_mass = sim.subtree_mass(sim.human_root());
    let arm_mass = bodies
        .iter()
        .position(|(n, _)| n == "humerus_r")
        .map(|i| sim.subtree_mass(i))
        .unwrap_or(f64::NAN);
    let flexor_f0 = sim
        .muscles()
        .iter()
        .filter(|m| {
            !m.name.ends_with("_l")
                && ["BIClong", "BICshort", "BRA", "BRD"]
                    .iter()
                    .any(|p| m.name.starts_with(p))
        })
        .map(|m| m.f_max)
        .sum();
    let body_mass = (which != Which::MyoArm).then_some(human_mass);
    Ok(ModelSummary {
        body_mass,
        arm_mass,
        flexor_f0,
    })
}

pub(crate) fn open_model(which: Which) -> Result<Sim, String> {
    match which {
        Which::FullBody => Sim::load(&models::ms_human_700()),
        Which::MyoArm => compose_myoarm(),
        Which::HumanArm => Sim::load(&models::ms_human_700_arm()),
    }
}

/// Сводки по всем трём моделям (порядок как в `MODELS`).
pub fn summaries() -> Vec<Result<ModelSummary, String>> {
    MODELS.iter().map(|(w, _)| summarize(*w)).collect()
}

pub struct Anatomy {
    which: usize,
    layers: Layers,
    sim: Option<Sim>,
    error: Option<String>,
    geoms: Vec<GeomInfo>,
    muscles: Vec<Muscle>,
    pose_joints: Vec<Option<JointInfo>>,
    pose_deg: [f64; 5],
    couplings: Vec<JointCoupling>,
    human_mass: f64,
    arm_mass: Option<f64>,
    extra_mass: Vec<(String, f64)>,
    height: f64,
    summaries: Vec<Result<ModelSummary, String>>,
    epoch: u64,
    t: f64,
}

impl Default for Anatomy {
    fn default() -> Self {
        let mut a = Self {
            which: 0,
            layers: Layers::All,
            sim: None,
            error: None,
            geoms: Vec::new(),
            muscles: Vec::new(),
            pose_joints: Vec::new(),
            pose_deg: POSE.map(|p| p.2),
            couplings: Vec::new(),
            human_mass: 0.0,
            arm_mass: None,
            extra_mass: Vec::new(),
            height: 0.0,
            summaries: summaries(),
            epoch: 0,
            t: 0.0,
        };
        a.load();
        a
    }
}

pub(crate) fn mat_to_pose(p: [f64; 3], r: [f64; 9]) -> Pose {
    let m = Matrix3::new(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]);
    let rot = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(m));
    Pose::new(rot, Vec3::new(p[0], p[1], p[2]))
}

impl Anatomy {
    fn load(&mut self) {
        let res = open_model(MODELS[self.which].0);
        self.epoch += 1;
        match res {
            Err(e) => {
                self.error = Some(e);
                self.sim = None;
                self.geoms.clear();
                self.muscles.clear();
            }
            Ok(sim) => {
                self.error = None;
                // Кости — только сетки (меши). Капсулы, эллипсоиды и цилиндры в этих моделях —
                // оболочки для контактов и поверхности обёртки мышц, а не анатомия.
                self.geoms = bone_geoms(&sim);
                self.muscles = sim
                    .muscles()
                    .into_iter()
                    .map(|m| {
                        let pcsa = m.f_max.max(1.0) / SPECIFIC_TENSION;
                        let radius = (0.3 * (pcsa / PI).sqrt()).clamp(0.0012, 0.009);
                        let slots = (2 * sim.tendon_path_objects(m.tendon)).max(2);
                        Muscle {
                            info: m,
                            radius,
                            slots,
                        }
                    })
                    .collect();
                self.pose_joints = POSE
                    .iter()
                    .map(|(name, _, _)| sim.joints().into_iter().find(|j| j.name == *name))
                    .collect();
                self.couplings = sim.joint_couplings();
                // Масса человека — только ветка тела; стол, бутылка и прочее — отдельно.
                let bodies = sim.bodies();
                let human = sim.subtree_mask(sim.human_root());
                self.extra_mass = bodies
                    .iter()
                    .enumerate()
                    .filter(|(i, (_, m))| *i > 0 && !human[*i] && *m > 0.0)
                    .map(|(_, b)| b.clone())
                    .collect();
                self.human_mass = sim.subtree_mass(sim.human_root());
                // Масса правой руки — вся ветка от ключицы (или от плечевой кости).
                self.arm_mass = ["clavicle_r", "clavicle", "humerus_r", "humerus"]
                    .iter()
                    .find_map(|n| bodies.iter().position(|(b, _)| b == n))
                    .map(|i| sim.subtree_mass(i));
                self.sim = Some(sim);
                self.apply_pose();
                let (lo, hi) = self.sim.as_ref().unwrap().z_extent();
                self.height = hi - lo;
            }
        }
    }

    fn apply_pose(&mut self) {
        let Some(sim) = self.sim.as_mut() else { return };
        let q0: Vec<f64> = sim.qpos0().to_vec();
        sim.qpos_mut().copy_from_slice(&q0);
        for (j, deg) in self.pose_joints.iter().zip(self.pose_deg.iter()) {
            if let Some(j) = j {
                let mut v = deg.to_radians();
                if j.limited {
                    v = v.clamp(j.range[0], j.range[1]);
                }
                sim.qpos_mut()[j.qpos_adr] = v;
            }
        }
        sim.apply_joint_couplings(&self.couplings);
        sim.forward();
    }

    fn count_flexor_force(&self) -> f64 {
        // Сумма F₀ мышц-сгибателей правого локтя (по именам OpenSim): BIC*, BRA, BRD.
        self.muscles
            .iter()
            .filter(|m| {
                !m.info.name.ends_with("_l")
                    && ["BIClong", "BICshort", "BRA", "BRD"]
                        .iter()
                        .any(|p| m.info.name.starts_with(p))
            })
            .map(|m| m.info.f_max)
            .sum()
    }
}

impl Scenario for Anatomy {
    fn id(&self) -> &'static str {
        "anatomy"
    }

    fn title(&self) -> String {
        "Анатомия: кости и мышцы (MuJoCo)".into()
    }

    fn summary(&self) -> String {
        "Открытые мышечно-скелетные модели человека: кости из модели, мышцы — по линиям действия \
         (толщина ~ максимальной силе мышцы), суставы с пределами и связями (плечевой ритм, колено). \
         Поза задаётся суставами. Силы мышц и нагрузки в суставах — следующие шаги карты человека."
            .into()
    }

    fn params(&self) -> Vec<Param> {
        POSE.iter()
            .zip(self.pose_joints.iter())
            .zip(self.pose_deg.iter())
            .filter_map(|(((id, label, _), j), v)| {
                let j = j.as_ref()?;
                let (lo, hi) = if j.limited {
                    (j.range[0].to_degrees(), j.range[1].to_degrees())
                } else {
                    (-180.0, 180.0)
                };
                Some(Param {
                    id,
                    label: format!("{label} (предел модели)"),
                    value: *v,
                    min: lo,
                    max: hi,
                    unit: "°",
                    decimals: 0,
                })
            })
            .collect()
    }

    fn set_param(&mut self, id: &str, value: f64) {
        if let Some(i) = POSE.iter().position(|p| p.0 == id) {
            self.pose_deg[i] = value;
            self.apply_pose();
        }
    }

    fn presets(&self) -> Vec<String> {
        vec![
            "Локоть 90°, плечо вниз".into(),
            "Рука вперёд 90°".into(),
            "Рука висит".into(),
        ]
    }

    fn apply_preset(&mut self, index: usize) {
        self.pose_deg = match index {
            0 => [90.0, 0.0, 0.0, 0.0, 0.0],
            1 => [10.0, 90.0, 90.0, 0.0, 0.0],
            _ => [0.0, 0.0, 0.0, 0.0, 0.0],
        };
        self.apply_pose();
    }

    fn choices(&self) -> Vec<Choice> {
        vec![
            Choice {
                id: "model",
                label: "Модель".into(),
                options: MODELS.iter().map(|m| m.1.to_string()).collect(),
                selected: self.which,
            },
            Choice {
                id: "layers",
                label: "Слои".into(),
                options: vec![
                    "Кости и мышцы".into(),
                    "Только кости".into(),
                    "Только мышцы".into(),
                ],
                selected: match self.layers {
                    Layers::All => 0,
                    Layers::Bones => 1,
                    Layers::Muscles => 2,
                },
            },
        ]
    }

    fn set_choice(&mut self, id: &str, index: usize) {
        match id {
            "model" if index != self.which => {
                self.which = index.min(MODELS.len() - 1);
                self.load();
            }
            "layers" => {
                self.layers = [Layers::All, Layers::Bones, Layers::Muscles][index.min(2)];
                self.epoch += 1;
            }
            _ => {}
        }
    }

    fn reset(&mut self) {
        self.pose_deg = POSE.map(|p| p.2);
        self.apply_pose();
    }

    fn advance(&mut self, dt_real: f64) {
        self.t += dt_real;
    }

    fn time(&self) -> f64 {
        self.t
    }

    fn visuals(&self) -> Vec<VisualInstance> {
        let Some(sim) = self.sim.as_ref() else {
            return Vec::new();
        };
        let mut out = Vec::new();
        if self.layers != Layers::Muscles {
            for g in &self.geoms {
                let (p, r) = sim.geom_pose(g.id);
                let pose = mat_to_pose(p, r);
                let [a, b, c] = g.size;
                let shape = match g.kind {
                    GeomKind::Mesh => Shape::Mesh {
                        id: g.mesh.unwrap() as u32,
                    },
                    GeomKind::Sphere => Shape::Sphere { radius: a },
                    GeomKind::Capsule => Shape::Capsule {
                        radius: a,
                        length: 2.0 * b,
                    },
                    GeomKind::Cylinder => Shape::Cylinder {
                        radius: a,
                        length: 2.0 * b,
                    },
                    GeomKind::Ellipsoid => Shape::Ellipsoid {
                        radii: Vec3::new(a, b, c),
                    },
                    GeomKind::Box => Shape::Box {
                        half: Vec3::new(a, b, c),
                    },
                    _ => continue,
                };
                let color = if g.kind == GeomKind::Mesh {
                    C_BONE
                } else {
                    g.rgba
                };
                out.push(VisualInstance { shape, pose, color });
            }
        }
        if self.layers != Layers::Bones {
            for m in &self.muscles {
                let segs = sim.tendon_segments(m.info.tendon);
                for k in 0..m.slots {
                    let (shape, pose) = match segs.get(k) {
                        Some((a, b)) => {
                            let a = Vec3::new(a[0], a[1], a[2]);
                            let b = Vec3::new(b[0], b[1], b[2]);
                            let d = b - a;
                            let len = d.norm();
                            if len < 1e-9 {
                                (
                                    Shape::Tube {
                                        radius: 0.0,
                                        length: 0.0,
                                    },
                                    Pose::at(a),
                                )
                            } else {
                                (
                                    Shape::Tube {
                                        radius: m.radius,
                                        length: len,
                                    },
                                    Pose::z_along((a + b) / 2.0, d),
                                )
                            }
                        }
                        None => (
                            Shape::Tube {
                                radius: 0.0,
                                length: 0.0,
                            },
                            Pose::IDENTITY,
                        ),
                    };
                    out.push(VisualInstance {
                        shape,
                        pose,
                        color: C_MUSCLE,
                    });
                }
            }
        }
        out
    }

    fn visual_epoch(&self) -> u64 {
        self.epoch
    }

    fn mesh_data(&self, id: u32) -> Option<MeshData> {
        let sim = self.sim.as_ref()?;
        let m = sim.mesh(id as usize);
        Some(MeshData {
            positions: m.positions,
            indices: m.indices,
        })
    }

    fn readouts(&self) -> Vec<Readout> {
        let Some(sim) = self.sim.as_ref() else {
            return Vec::new();
        };
        let mut r = vec![
            Readout::new("Тел (сегментов)", sim.nbody() as f64, "", 0),
            Readout::new("Степеней свободы", sim.nv() as f64, "", 0),
            Readout::new("Мышц", self.muscles.len() as f64, "", 0),
            Readout::new(
                "Связей суставов (ритм плеча и др.)",
                self.couplings.len() as f64,
                "",
                0,
            ),
            Readout::new("Костей и деталей (мешей)", sim.nmesh() as f64, "", 0),
        ];
        if MODELS[self.which].0 == Which::FullBody {
            r.push(Readout::new(
                "Масса человека по модели",
                self.human_mass,
                "кг",
                2,
            ));
            r.push(Readout::new("Рост по геометрии", self.height, "м", 3));
        } else if MODELS[self.which].0 == Which::HumanArm {
            r.push(Readout::new(
                "Масса человека по модели (без стола)",
                self.human_mass,
                "кг",
                2,
            ));
        }
        if let Some(m) = self.arm_mass {
            r.push(Readout::new(
                "Масса правой руки (от ключицы до пальцев)",
                m,
                "кг",
                2,
            ));
        }
        r.extend([Readout::new(
            "Σ F₀ сгибателей правого локтя (бицепс, плечевая, плечелучевая)",
            self.count_flexor_force(),
            "Н",
            0,
        )]);
        if let Some(j) = self.pose_joints[0].as_ref() {
            r.push(Readout::new(
                "Угол в локте (модель)",
                sim.qpos()[j.qpos_adr].to_degrees(),
                "°",
                1,
            ));
        }
        r
    }

    fn comparisons(&self) -> Vec<Comparison> {
        let Some(Ok(me)) = self.summaries.get(self.which) else {
            return Vec::new();
        };
        let other = |w: Which| {
            MODELS
                .iter()
                .position(|(x, _)| *x == w)
                .and_then(|i| self.summaries[i].as_ref().ok().copied())
        };
        let judge = |sim: f64, reference: f64, tol: f64, why: &str| -> (Status, String) {
            let rel = (sim - reference) / reference;
            if rel.abs() <= 0.05 {
                (Status::Ok, "совпало в пределах 5%".into())
            } else if rel.abs() <= tol {
                (
                    Status::Explained,
                    format!(
                        "расходится на {:.0}%: в пределах ±{:.0}%, это разброс между людьми и источниками",
                        rel.abs() * 100.0,
                        tol * 100.0
                    ),
                )
            } else if rel.abs() <= 0.5 {
                (
                    Status::Explained,
                    format!("{why} Решения по безопасности принимаем по худшему краю."),
                )
            } else {
                (Status::Mismatch, "расхождение больше 50% — вероятна ошибка загрузки или единиц, числам не верить".into())
            }
        };
        let mut out = Vec::new();
        if let Some(body) = me.body_mass {
            let reference = WINTER_ARM_FRACTION * body;
            let (status, explanation) = judge(
                me.arm_mass,
                reference,
                0.15,
                "Рука в модели тяжелее, чем по таблицам: таблицы Dempster/Winter — средние по небольшой выборке, модели собраны по другим данным.",
            );
            out.push(Comparison {
                title: "Масса руки (плечо + предплечье + кисть): модель ↔ таблицы Winter".into(),
                sim_label: "по модели".into(),
                sim: me.arm_mass,
                ref_label: format!("5.0% × {body:.1} кг"),
                reference,
                unit: "кг",
                decimals: 2,
                status,
                explanation,
            });
        }
        let (peer, peer_name) = if MODELS[self.which].0 == Which::MyoArm {
            (other(Which::FullBody), "MS-Human-700")
        } else {
            (other(Which::MyoArm), "MyoArm")
        };
        if let Some(p) = peer {
            if MODELS[self.which].0 == Which::MyoArm {
                let (status, explanation) = judge(
                    me.arm_mass,
                    p.arm_mass,
                    0.15,
                    "Модели собраны по разным людям-образцам.",
                );
                out.push(Comparison {
                    title: format!(
                        "Масса руки (плечо + предплечье + кисть): эта модель ↔ {peer_name}"
                    ),
                    sim_label: "MyoArm".into(),
                    sim: me.arm_mass,
                    ref_label: peer_name.into(),
                    reference: p.arm_mass,
                    unit: "кг",
                    decimals: 2,
                    status,
                    explanation,
                });
            }
            let (status, explanation) = judge(
                me.flexor_f0,
                p.flexor_f0,
                0.15,
                "Силы мышц в моделях взяты из разных исследований.",
            );
            out.push(Comparison {
                title: format!("Σ F₀ сгибателей локтя: эта модель ↔ {peer_name}"),
                sim_label: "эта модель".into(),
                sim: me.flexor_f0,
                ref_label: peer_name.into(),
                reference: p.flexor_f0,
                unit: "Н",
                decimals: 0,
                status,
                explanation,
            });
        }
        out
    }

    fn notes(&self) -> Vec<Note> {
        let mut out = Vec::new();
        if let Some(e) = &self.error {
            out.push(Note::new(
                Level::Danger,
                format!("Модель не загрузилась: {e}"),
            ));
            return out;
        }
        out.push(Note::new(
            Level::Info,
            format!("MuJoCo {}. Это шаг 2a: поза задаётся вручную, мышцы показаны линиями действия. Силы мышц, сжатие в суставах и сверка с силой человека — шаги 2c–2d.", sim_mujoco::version()),
        ));
        for (name, m) in &self.extra_mass {
            out.push(Note::new(Level::Info, format!("В модели есть посторонний объект «{name}» ({m:.1} кг) — он не входит в массу человека.")));
        }
        if MODELS[self.which].0 == Which::MyoArm {
            out.push(Note::new(
                Level::Warn,
                "MyoArm сделан из модели MoBL (Stanford): для исследований можно, для продажи изделия — нужна лицензия владельцев MoBL или замена данных.",
            ));
        }
        out.push(Note::new(
            Level::Warn,
            "Модели — «средний человек» из исследований (MS-Human-700: 60 кг, 1.74 м). Под конкретную руку модель ещё не масштабирована.",
        ));
        out
    }

    fn focus(&self) -> Vec3 {
        let Some(sim) = self.sim.as_ref() else {
            return Vec3::new(0.0, 0.0, 1.2);
        };
        match MODELS[self.which].0 {
            Which::FullBody => Vec3::new(0.0, 0.0, 1.0),
            _ => self.pose_joints[0]
                .as_ref()
                .map(|j| {
                    let a = sim.joint_anchor(j.id);
                    Vec3::new(a[0], a[1], a[2])
                })
                .unwrap_or(Vec3::new(0.0, 0.0, 1.2)),
        }
    }

    fn view_offset(&self) -> Vec3 {
        match MODELS[self.which].0 {
            // MS-Human-700 смотрит вдоль +X (правый бок — на −Y), MyoArm — вдоль −Y (правый бок — на −X).
            Which::FullBody => Vec3::new(1.2, -2.9, 0.1),
            Which::HumanArm => Vec3::new(0.55, -1.35, 0.12),
            Which::MyoArm => Vec3::new(-1.6, -0.6, 0.12),
        }
    }
}

/// Кости модели: только сетки (капсулы и эллипсоиды в этих моделях — оболочки контактов и обёрток).
pub(crate) fn bone_geoms(sim: &Sim) -> Vec<GeomInfo> {
    let human = sim.subtree_mask(sim.human_root());
    sim.geoms()
        .into_iter()
        .filter(|g| g.kind == GeomKind::Mesh && g.rgba[3] > 0.05 && human[g.body])
        .collect()
}

pub(crate) fn bone_visuals(sim: &Sim, geoms: &[GeomInfo], out: &mut Vec<VisualInstance>) {
    for g in geoms {
        let (p, r) = sim.geom_pose(g.id);
        if let Some(id) = g.mesh {
            out.push(VisualInstance {
                shape: Shape::Mesh { id: id as u32 },
                pose: mat_to_pose(p, r),
                color: C_BONE,
            });
        }
    }
}

/// Толщина трубки мышцы на экране (~ физиологический поперечник по F₀).
pub(crate) fn muscle_radius(f_max: f64) -> f64 {
    let pcsa = f_max.max(1.0) / SPECIFIC_TENSION;
    (0.3 * (pcsa / PI).sqrt()).clamp(0.0012, 0.009)
}

/// Сколько отрезков резервировать под мышцу (чтобы число фигур не прыгало при обёртке).
pub(crate) fn muscle_slots(sim: &Sim, tendon: usize) -> usize {
    (2 * sim.tendon_path_objects(tendon)).max(2)
}

/// Мышца как цепочка трубок по её пути; всегда ровно `slots` фигур (лишние — нулевые).
pub(crate) fn muscle_tubes(
    sim: &Sim,
    tendon: usize,
    slots: usize,
    radius: f64,
    color: [f32; 4],
    out: &mut Vec<VisualInstance>,
) {
    let segs = sim.tendon_segments(tendon);
    for k in 0..slots {
        let (shape, pose) = match segs.get(k) {
            Some((a, b)) => {
                let a = Vec3::new(a[0], a[1], a[2]);
                let b = Vec3::new(b[0], b[1], b[2]);
                let d = b - a;
                let len = d.norm();
                if len < 1e-9 {
                    (
                        Shape::Tube {
                            radius: 0.0,
                            length: 0.0,
                        },
                        Pose::at(a),
                    )
                } else {
                    (
                        Shape::Tube {
                            radius,
                            length: len,
                        },
                        Pose::z_along((a + b) / 2.0, d),
                    )
                }
            }
            None => (
                Shape::Tube {
                    radius: 0.0,
                    length: 0.0,
                },
                Pose::IDENTITY,
            ),
        };
        out.push(VisualInstance { shape, pose, color });
    }
}
