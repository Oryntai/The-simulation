//! Сценарий «Экзоскелет одной руки»: плечо, предплечье, локтевой шарнир с приводом,
//! гантель в кисти. Привод (ПИД-регулятор, 2 кГц) поднимает предплечье и держит угол.
//!
//! Что сверяем на экране:
//! * момент в локте из прямой динамики (то, что выдал привод) ↔ ручной расчёт m·g·L;
//! * ↔ ручной расчёт со всеми массами Σ mᵢ·g·xᵢ (второй независимый способ);
//! * ↔ обратная динамика RNEA (внутренняя согласованность движка);
//! * ↔ предел обычного человека 60–80 Н·м и запас прочности 2–4×.

use std::f64::consts::PI;

use sim_core::dynamics::{body_coms_world, mass_matrix, static_holding_torques, world_to_body};
use sim_core::{
    Body, G_STANDARD, Inertia, Inputs, Integrator, Joint, Model, Pose, Shape, State, Vec3, Visual,
    Xform, step,
};

use crate::scenario::*;

/// Предел обычного человека в локте (изометрическое сгибание, ~90°), Н·м.
pub const HUMAN_ELBOW_LIMIT: (f64, f64) = (60.0, 80.0);
/// Запас прочности для всего, что касается тела.
pub const SAFETY_FACTOR: (f64, f64) = (2.0, 4.0);

const DT: f64 = 0.0005;
const Y_NEG: Vec3 = Vec3::new(0.0, -1.0, 0.0);

/// Антропометрия по Winter D.A., «Biomechanics and Motor Control of Human Movement», 4-е изд.
/// (2009), табл. 4.1 (данные Dempster, 1955): доли массы тела, положения ЦМ от проксимального
/// сустава, радиусы гирации; длины сегментов — доли роста (Drillis & Contini).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Anthropometry {
    pub body_mass: f64,
    pub height: f64,
}

impl Default for Anthropometry {
    fn default() -> Self {
        Self {
            body_mass: 75.0,
            height: 1.75,
        }
    }
}

impl Anthropometry {
    pub fn upper_arm_len(&self) -> f64 {
        0.186 * self.height
    }
    pub fn forearm_len(&self) -> f64 {
        0.146 * self.height
    }
    pub fn hand_len(&self) -> f64 {
        0.108 * self.height
    }
    pub fn upper_arm_mass(&self) -> f64 {
        0.028 * self.body_mass
    }
    pub fn forearm_mass(&self) -> f64 {
        0.016 * self.body_mass
    }
    pub fn hand_mass(&self) -> f64 {
        0.006 * self.body_mass
    }
    pub fn shoulder_height(&self) -> f64 {
        0.818 * self.height
    }
    pub fn hip_height(&self) -> f64 {
        0.53 * self.height
    }
    pub fn upper_arm_com(&self) -> f64 {
        0.436 * self.upper_arm_len()
    }
    pub fn forearm_com(&self) -> f64 {
        0.430 * self.forearm_len()
    }
    pub fn hand_com_from_wrist(&self) -> f64 {
        0.506 * self.hand_len()
    }
    /// Плечо силы для груза: от оси локтя до хвата (≈ ЦМ кисти).
    pub fn grip_from_elbow(&self) -> f64 {
        self.forearm_len() + self.hand_com_from_wrist()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExoArmConfig {
    pub anthro: Anthropometry,
    /// Масса груза в кисти, кг.
    pub payload: f64,
    /// Целевой угол сгибания локтя, градусы (0 — рука выпрямлена вниз, 90 — предплечье горизонтально).
    pub elbow_target_deg: f64,
    /// Ортез на предплечье (манжета + штанга): масса и расстояние ЦМ от оси локтя — допущение.
    pub exo_forearm_mass: f64,
    pub exo_forearm_com: f64,
    /// Ортез на плече (манжета + штанга) и привод локтя — висят на плече, локоть не грузят.
    pub exo_upper_mass: f64,
    pub motor_mass: f64,
}

impl Default for ExoArmConfig {
    fn default() -> Self {
        Self {
            anthro: Anthropometry::default(),
            payload: 20.0,
            elbow_target_deg: 90.0,
            exo_forearm_mass: 0.5,
            exo_forearm_com: 0.14,
            exo_upper_mass: 0.6,
            motor_mass: 0.7,
        }
    }
}

/// Ручной расчёт статического момента в локте — независимо от движка, «на бумажке».
/// Плечо вертикально, предплечье под углом θ к вертикали: плечо силы = d·sin θ.
#[derive(Clone, Copy, Debug)]
pub struct HandCalc {
    pub lever: f64,
    /// Только груз: m·g·L·sin θ.
    pub payload: f64,
    pub forearm: f64,
    pub hand: f64,
    pub exo: f64,
    /// Σ mᵢ·g·xᵢ — все массы ниже локтя.
    pub full: f64,
}

pub fn hand_calc(cfg: &ExoArmConfig, theta: f64) -> HandCalc {
    let a = &cfg.anthro;
    let g = G_STANDARD;
    let s = theta.sin();
    let lever = a.grip_from_elbow();
    let payload = cfg.payload * g * lever * s;
    let forearm = a.forearm_mass() * g * a.forearm_com() * s;
    let hand = a.hand_mass() * g * a.grip_from_elbow() * s;
    let exo = cfg.exo_forearm_mass * g * cfg.exo_forearm_com * s;
    HandCalc {
        lever,
        payload,
        forearm,
        hand,
        exo,
        full: payload + forearm + hand + exo,
    }
}

/// Радиус диска гантели: два чугунных диска толщиной 4 см.
fn plate_radius(payload: f64) -> f64 {
    let t = 0.04;
    ((payload.max(0.5) / 2.0) / (7200.0 * PI * t)).sqrt()
}

#[derive(Clone, Copy, Debug)]
pub struct ArmIndices {
    pub upper: usize,
    pub forearm: usize,
    pub payload: usize,
}

pub fn build_model(cfg: &ExoArmConfig) -> (Model, ArmIndices) {
    let a = cfg.anthro;
    let (lu, lf, lh) = (a.upper_arm_len(), a.forearm_len(), a.hand_len());
    let mut m = Model::earth();
    let k = a.height / 1.75;
    let shoulder = Vec3::new(0.0, -0.21 * k, a.shoulder_height());
    add_mannequin(&mut m, &a, shoulder);

    // Плечо (сегмент) — вращение вокруг −Y: сгибание вперёд.
    let ru = 0.045 * k;
    let upper = m.add_body(
        Body::new(
            "плечо",
            None,
            Joint::revolute(Y_NEG),
            Xform::translation(shoulder),
            Inertia::segment_z(
                a.upper_arm_mass(),
                Vec3::new(0.0, 0.0, -a.upper_arm_com()),
                0.322 * lu,
                ru,
            ),
        )
        .with_visual(Visual::new(
            Shape::Sphere { radius: ru * 1.15 },
            Pose::IDENTITY,
            C_SKIN,
        ))
        .with_visual(Visual::new(
            Shape::Capsule {
                radius: ru,
                length: lu - ru,
            },
            Pose::at(Vec3::new(0.0, 0.0, -lu / 2.0)),
            C_SKIN,
        )),
    );
    // Ортез на плече: манжета + боковая штанга.
    m.add_body(
        Body::new(
            "ортез: плечо",
            Some(upper),
            Joint::Fixed,
            Xform::translation(Vec3::new(0.0, -0.06 * k, -0.55 * lu)),
            Inertia::solid_box(
                cfg.exo_upper_mass,
                Vec3::new(0.012, 0.008, 0.45 * lu),
                Vec3::zeros(),
            ),
        )
        .with_visual(Visual::new(
            Shape::Box {
                half: Vec3::new(0.012, 0.008, 0.45 * lu),
            },
            Pose::IDENTITY,
            C_EXO,
        ))
        .with_visual(Visual::new(
            Shape::Cylinder {
                radius: ru + 0.012,
                length: 0.06,
            },
            Pose::at(Vec3::new(0.0, 0.06 * k, -0.1 * lu)),
            C_EXO_CUFF,
        )),
    );
    // Привод локтя — на плечевой части ортеза, на оси локтя.
    m.add_body(
        Body::new(
            "ортез: привод локтя",
            Some(upper),
            Joint::Fixed,
            Xform::translation(Vec3::new(0.0, -0.075 * k, -lu)),
            Inertia::solid_cylinder(cfg.motor_mass, 0.045, 0.05, 1, Vec3::zeros()),
        )
        .with_visual(Visual::new(
            Shape::Cylinder {
                radius: 0.045,
                length: 0.05,
            },
            Pose::z_along(Vec3::zeros(), Vec3::y()),
            C_MOTOR,
        )),
    );

    // Предплечье — локтевой шарнир.
    let rf = 0.037 * k;
    let forearm = m.add_body(
        Body::new(
            "предплечье",
            Some(upper),
            Joint::revolute(Y_NEG),
            Xform::translation(Vec3::new(0.0, 0.0, -lu)),
            Inertia::segment_z(
                a.forearm_mass(),
                Vec3::new(0.0, 0.0, -a.forearm_com()),
                0.303 * lf,
                rf,
            ),
        )
        .with_visual(Visual::new(
            Shape::Sphere { radius: ru * 1.02 },
            Pose::IDENTITY,
            C_SKIN,
        ))
        .with_visual(Visual::new(
            Shape::Capsule {
                radius: rf,
                length: lf - rf,
            },
            Pose::at(Vec3::new(0.0, 0.0, -lf / 2.0)),
            C_SKIN,
        )),
    );
    m.add_body(
        Body::new(
            "ортез: предплечье",
            Some(forearm),
            Joint::Fixed,
            Xform::translation(Vec3::new(0.0, -0.06 * k, -cfg.exo_forearm_com)),
            Inertia::solid_box(
                cfg.exo_forearm_mass,
                Vec3::new(0.012, 0.008, 0.12),
                Vec3::zeros(),
            ),
        )
        .with_visual(Visual::new(
            Shape::Box {
                half: Vec3::new(0.012, 0.008, 0.12),
            },
            Pose::at(Vec3::new(0.0, 0.0, 0.0)),
            C_EXO,
        ))
        .with_visual(Visual::new(
            Shape::Cylinder {
                radius: rf + 0.012,
                length: 0.05,
            },
            Pose::at(Vec3::new(0.0, 0.06 * k, -0.06)),
            C_EXO_CUFF,
        )),
    );
    let hand = m.add_body(
        Body::new(
            "кисть",
            Some(forearm),
            Joint::Fixed,
            Xform::translation(Vec3::new(0.0, 0.0, -lf)),
            Inertia::segment_z(
                a.hand_mass(),
                Vec3::new(0.0, 0.0, -a.hand_com_from_wrist()),
                0.297 * lh,
                0.03,
            ),
        )
        .with_visual(Visual::new(
            Shape::Box {
                half: Vec3::new(0.016, 0.042, lh / 2.0),
            },
            Pose::at(Vec3::new(0.0, 0.0, -lh / 2.0)),
            C_SKIN,
        )),
    );
    // Гантель: ЦМ в точке хвата, ручка поперёк ладони (вдоль Y).
    let rp = plate_radius(cfg.payload);
    let t = 0.04;
    let off = 0.075 + t / 2.0;
    let plate =
        |s: f64| Inertia::solid_cylinder(cfg.payload / 2.0, rp, t, 1, Vec3::new(0.0, s * off, 0.0));
    let payload_inertia = if cfg.payload > 0.0 {
        plate(1.0).combine(&plate(-1.0))
    } else {
        Inertia::zero()
    };
    let mut db = Body::new(
        "гантель",
        Some(hand),
        Joint::Fixed,
        Xform::translation(Vec3::new(0.0, 0.0, -a.hand_com_from_wrist())),
        payload_inertia,
    );
    if cfg.payload > 0.0 {
        db = db
            .with_visual(Visual::new(
                Shape::Cylinder {
                    radius: 0.016,
                    length: 2.0 * off + t + 0.04,
                },
                Pose::z_along(Vec3::zeros(), Vec3::y()),
                C_STEEL,
            ))
            .with_visual(Visual::new(
                Shape::Cylinder {
                    radius: rp,
                    length: t,
                },
                Pose::z_along(Vec3::new(0.0, off, 0.0), Vec3::y()),
                C_IRON,
            ))
            .with_visual(Visual::new(
                Shape::Cylinder {
                    radius: rp,
                    length: t,
                },
                Pose::z_along(Vec3::new(0.0, -off, 0.0), Vec3::y()),
                C_IRON,
            ));
    }
    let payload = m.add_body(db);
    (
        m,
        ArmIndices {
            upper,
            forearm,
            payload,
        },
    )
}

/// Манекен — только для показа (на физику руки не влияет).
fn add_mannequin(m: &mut Model, a: &Anthropometry, shoulder: Vec3) {
    let k = a.height / 1.75;
    let hip = a.hip_height();
    let top = a.shoulder_height() + 0.04 * k;
    let c = C_MANNEQUIN;
    let w = &mut m.world_visuals;
    w.push(Visual::new(
        Shape::Box {
            half: Vec3::new(0.1 * k, 0.155 * k, (top - hip) / 2.0),
        },
        Pose::at(Vec3::new(0.0, 0.0, (top + hip) / 2.0)),
        c,
    ));
    w.push(Visual::new(
        Shape::Cylinder {
            radius: 0.05 * k,
            length: 0.1 * k,
        },
        Pose::at(Vec3::new(0.0, 0.0, top + 0.04 * k)),
        c,
    ));
    w.push(Visual::new(
        Shape::Sphere { radius: 0.1 * k },
        Pose::at(Vec3::new(0.0, 0.0, a.height - 0.11 * k)),
        c,
    ));
    for s in [-1.0, 1.0] {
        let y = s * 0.085 * k;
        w.push(Visual::new(
            Shape::Capsule {
                radius: 0.07 * k,
                length: hip - 0.15,
            },
            Pose::at(Vec3::new(0.0, y, hip / 2.0 + 0.04)),
            c,
        ));
        w.push(Visual::new(
            Shape::Box {
                half: Vec3::new(0.12 * k, 0.05 * k, 0.03),
            },
            Pose::at(Vec3::new(0.05 * k, y, 0.03)),
            c,
        ));
    }
    // Левая рука висит свободно.
    let left = Vec3::new(0.0, -shoulder.y, shoulder.z);
    let la = a.upper_arm_len() + a.forearm_len();
    w.push(Visual::new(
        Shape::Capsule {
            radius: 0.042 * k,
            length: la - 0.05,
        },
        Pose::at(left - Vec3::new(0.0, 0.0, la / 2.0)),
        c,
    ));
}

#[derive(Clone, Copy, Debug, Default)]
struct Pid {
    kp: f64,
    ki: f64,
    kd: f64,
    integral: f64,
    limit: f64,
}

impl Pid {
    fn update(&mut self, e: f64, ed: f64, dt: f64) -> f64 {
        self.integral = (self.integral + e * dt).clamp(-self.limit, self.limit);
        self.kp * e + self.kd * ed + self.ki * self.integral
    }
}

/// Траектория минимального рывка (Flash & Hogan, 1985) — так двигается рука человека.
#[derive(Clone, Copy, Debug)]
struct MinJerk {
    t0: f64,
    dur: f64,
    from: f64,
    to: f64,
}

impl MinJerk {
    fn eval(&self, t: f64) -> (f64, f64) {
        let tau = ((t - self.t0) / self.dur).clamp(0.0, 1.0);
        let s = tau.powi(3) * (10.0 - 15.0 * tau + 6.0 * tau * tau);
        let ds = if (0.0..1.0).contains(&tau) {
            30.0 * tau * tau * (1.0 - tau).powi(2) / self.dur
        } else {
            0.0
        };
        (
            self.from + (self.to - self.from) * s,
            (self.to - self.from) * ds,
        )
    }

    fn end(&self) -> f64 {
        self.t0 + self.dur
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Hold,
    Curl,
}

const CURL_RANGE: (f64, f64) = (30.0, 120.0);

pub struct ExoArm {
    cfg: ExoArmConfig,
    model: Model,
    idx: ArmIndices,
    state: State,
    clock: FixedClock,
    elbow: Pid,
    shoulder: Pid,
    traj: MinJerk,
    mode: Mode,
    /// Каналы: 0 τ_локоть, 1 τ_плечо, 2 θ_локоть°, 3 θ_зад°, 4 мощность привода локтя.
    history: History,
    epoch: u64,
    /// Когда последний раз меняли модель (груз, антропометрию) — после этого ждём установления заново.
    changed_at: f64,
}

impl Default for ExoArm {
    fn default() -> Self {
        Self::new(ExoArmConfig::default())
    }
}

impl ExoArm {
    pub fn new(cfg: ExoArmConfig) -> Self {
        let (model, idx) = build_model(&cfg);
        let state = State::rest(&model);
        let mut s = Self {
            cfg,
            model,
            idx,
            state,
            clock: FixedClock::new(DT),
            elbow: Pid::default(),
            shoulder: Pid::default(),
            traj: MinJerk {
                t0: 0.0,
                dur: 1.0,
                from: 0.0,
                to: 0.0,
            },
            mode: Mode::Hold,
            history: History::new(5, 12.0),
            epoch: 0,
            changed_at: 0.0,
        };
        s.reset();
        s
    }

    pub fn config(&self) -> &ExoArmConfig {
        &self.cfg
    }

    pub fn model(&self) -> &Model {
        &self.model
    }

    fn tune_gains(&mut self) {
        let mut q = self.model.neutral_q();
        q[1] = self.cfg.elbow_target_deg.to_radians();
        let mm = mass_matrix(&self.model, &q);
        let wn = 2.0 * PI * 3.0;
        let tune = |pid: &mut Pid, inertia: f64| {
            pid.kp = inertia * wn * wn;
            pid.kd = 2.0 * inertia * wn;
            pid.ki = pid.kp * wn / 10.0;
            pid.limit = 600.0 / pid.ki;
        };
        tune(&mut self.elbow, mm[(1, 1)]);
        tune(&mut self.shoulder, mm[(0, 0)]);
    }

    fn rebuild(&mut self) {
        let (model, idx) = build_model(&self.cfg);
        self.model = model;
        self.idx = idx;
        self.tune_gains();
        self.epoch += 1;
        self.changed_at = self.state.t;
    }

    fn start_move(&mut self, to_deg: f64) {
        let (from, _) = self.traj.eval(self.state.t);
        let to = to_deg.to_radians();
        let dur = 0.6 + (to - from).abs() / 1.5;
        let t0 = if self.state.t < 1e-9 {
            0.4
        } else {
            self.state.t
        };
        self.traj = MinJerk { t0, dur, from, to };
    }

    pub fn elbow_angle(&self) -> f64 {
        self.state.q[1]
    }

    pub fn time_now(&self) -> f64 {
        self.state.t
    }

    /// Установилось ли удержание (можно сравнивать со статикой).
    pub fn settled(&self) -> bool {
        self.mode == Mode::Hold
            && self.state.t > self.traj.end().max(self.changed_at) + 1.5
            && self.state.v.iter().all(|v| v.abs() < 2e-3)
    }

    /// Момент в локте из симуляции (усреднение 0.25 с).
    pub fn sim_elbow_torque(&self) -> f64 {
        self.history.mean(0, 0.25).unwrap_or(0.0)
    }

    pub fn sim_shoulder_torque(&self) -> f64 {
        self.history.mean(1, 0.25).unwrap_or(0.0)
    }

    /// Обратная динамика в текущей позе (статика): [плечо, локоть].
    pub fn rnea_torques(&self) -> (f64, f64) {
        let t = static_holding_torques(&self.model, &self.state.q, &[]);
        (t[0], t[1])
    }

    pub fn target_rad(&self) -> f64 {
        self.cfg.elbow_target_deg.to_radians()
    }

    fn step_once(&mut self) {
        if self.mode == Mode::Curl && self.state.t >= self.traj.end() + 0.3 {
            let next = if self.traj.to.to_degrees() > (CURL_RANGE.0 + CURL_RANGE.1) / 2.0 {
                CURL_RANGE.0
            } else {
                CURL_RANGE.1
            };
            self.start_move(next);
        }
        let (r, rd) = self.traj.eval(self.state.t);
        let tau_e = self
            .elbow
            .update(r - self.state.q[1], rd - self.state.v[1], DT);
        let tau_s = self.shoulder.update(-self.state.q[0], -self.state.v[0], DT);
        let mut inputs = Inputs::zero(&self.model);
        inputs.tau[0] = tau_s;
        inputs.tau[1] = tau_e;
        let qd_e = self.state.v[1];
        step(&self.model, &mut self.state, &inputs, DT, Integrator::Rk4);
        self.history.push(
            self.state.t,
            &[
                tau_e,
                tau_s,
                self.state.q[1].to_degrees(),
                r.to_degrees(),
                tau_e * qd_e,
            ],
        );
    }

    /// Прогнать без окна до установления (для тестов и отчётов).
    pub fn run_until_settled(&mut self, max_time: f64) -> bool {
        while self.state.t < max_time {
            self.step_once();
            if self.settled() {
                return true;
            }
        }
        false
    }

    fn required_torque(&self) -> f64 {
        if self.settled() {
            self.sim_elbow_torque()
        } else {
            hand_calc(&self.cfg, self.target_rad()).full
        }
    }
}

fn limit_color(tau: f64) -> [f32; 4] {
    let t = tau.abs();
    if t <= HUMAN_ELBOW_LIMIT.0 {
        C_GREEN
    } else if t <= HUMAN_ELBOW_LIMIT.1 {
        C_YELLOW
    } else {
        C_RED
    }
}

impl Scenario for ExoArm {
    fn id(&self) -> &'static str {
        "exo_arm"
    }

    fn title(&self) -> String {
        "Экзоскелет руки: момент в локте".into()
    }

    fn summary(&self) -> String {
        "Плечо, предплечье, локтевой шарнир с приводом; гантель в кисти. Привод поднимает предплечье \
         и держит угол. Момент в локте из симуляции сравнивается с ручным расчётом m·g·L и с пределом \
         обычного человека (60–80 Н·м)."
            .into()
    }

    fn params(&self) -> Vec<Param> {
        let c = &self.cfg;
        vec![
            Param {
                id: "payload",
                label: "Груз в кисти".into(),
                value: c.payload,
                min: 0.0,
                max: 60.0,
                unit: "кг",
                decimals: 1,
            },
            Param {
                id: "angle",
                label: "Угол в локте (цель)".into(),
                value: c.elbow_target_deg,
                min: 0.0,
                max: 150.0,
                unit: "°",
                decimals: 0,
            },
            Param {
                id: "exo_fa",
                label: "Ортез на предплечье (допущение)".into(),
                value: c.exo_forearm_mass,
                min: 0.0,
                max: 2.0,
                unit: "кг",
                decimals: 2,
            },
            Param {
                id: "body_mass",
                label: "Масса человека".into(),
                value: c.anthro.body_mass,
                min: 45.0,
                max: 130.0,
                unit: "кг",
                decimals: 0,
            },
            Param {
                id: "height",
                label: "Рост".into(),
                value: c.anthro.height,
                min: 1.5,
                max: 2.05,
                unit: "м",
                decimals: 2,
            },
        ]
    }

    fn set_param(&mut self, id: &str, value: f64) {
        match id {
            "payload" => self.cfg.payload = value,
            "angle" => {
                self.cfg.elbow_target_deg = value;
                if self.mode == Mode::Hold {
                    self.start_move(value);
                }
                self.tune_gains();
                return;
            }
            "exo_fa" => self.cfg.exo_forearm_mass = value,
            "body_mass" => self.cfg.anthro.body_mass = value,
            "height" => self.cfg.anthro.height = value,
            _ => return,
        }
        self.rebuild();
    }

    fn presets(&self) -> Vec<String> {
        vec!["20 кг, локоть 90°".into(), "50 кг, локоть 90°".into()]
    }

    fn apply_preset(&mut self, index: usize) {
        let payload = if index == 0 { 20.0 } else { 50.0 };
        self.cfg = ExoArmConfig {
            payload,
            elbow_target_deg: 90.0,
            ..ExoArmConfig::default()
        };
        self.mode = Mode::Hold;
        self.reset();
    }

    fn choices(&self) -> Vec<Choice> {
        vec![Choice {
            id: "mode",
            label: "Режим".into(),
            options: vec![
                "Удержание угла".into(),
                format!("Подъёмы {:.0}° ↔ {:.0}°", CURL_RANGE.0, CURL_RANGE.1),
            ],
            selected: if self.mode == Mode::Hold { 0 } else { 1 },
        }]
    }

    fn set_choice(&mut self, id: &str, index: usize) {
        if id == "mode" {
            self.mode = if index == 0 { Mode::Hold } else { Mode::Curl };
            if self.mode == Mode::Hold {
                self.start_move(self.cfg.elbow_target_deg);
            } else {
                self.start_move(CURL_RANGE.1);
            }
        }
    }

    fn reset(&mut self) {
        self.rebuild();
        self.state = State::rest(&self.model);
        self.changed_at = 0.0;
        self.elbow.integral = 0.0;
        self.shoulder.integral = 0.0;
        self.clock.reset();
        self.history.clear();
        self.traj = MinJerk {
            t0: 0.0,
            dur: 1.0,
            from: 0.0,
            to: 0.0,
        };
        let target = if self.mode == Mode::Hold {
            self.cfg.elbow_target_deg
        } else {
            CURL_RANGE.1
        };
        self.start_move(target);
    }

    fn advance(&mut self, dt_real: f64) {
        for _ in 0..self.clock.steps(dt_real) {
            self.step_once();
        }
    }

    fn time(&self) -> f64 {
        self.state.t
    }

    fn visuals(&self) -> Vec<VisualInstance> {
        model_visuals(&self.model, &self.state.q)
    }

    fn visual_epoch(&self) -> u64 {
        self.epoch
    }

    fn overlays(&self) -> Vec<Overlay> {
        let xw = world_to_body(&self.model, &self.state.q);
        let coms = body_coms_world(&self.model, &self.state.q);
        let elbow = xw[self.idx.forearm].pos;
        let shoulder = xw[self.idx.upper].pos;
        let mut out = Vec::new();
        if self.cfg.payload > 0.0 {
            let c = coms[self.idx.payload];
            let weight = self.cfg.payload * G_STANDARD;
            out.push(Overlay::Arrow {
                from: c,
                to: c - Vec3::new(0.0, 0.0, weight * 0.0012),
                color: C_RED,
            });
            // Плечо силы — горизонталь от оси локтя до вертикали через груз.
            let foot = Vec3::new(c.x, elbow.y, elbow.z);
            out.push(Overlay::Line {
                from: elbow,
                to: foot,
                color: C_YELLOW,
            });
            out.push(Overlay::Line {
                from: foot,
                to: Vec3::new(c.x, elbow.y, c.z),
                color: C_YELLOW,
            });
        }
        let tau_e = self.history.mean(0, 0.1).unwrap_or(0.0);
        let tau_s = self.history.mean(1, 0.1).unwrap_or(0.0);
        out.push(Overlay::Ring {
            center: elbow + Vec3::new(0.0, -0.11, 0.0),
            normal: Vec3::y(),
            radius: 0.06,
            color: limit_color(tau_e),
        });
        out.push(Overlay::Ring {
            center: shoulder + Vec3::new(0.0, -0.08, 0.0),
            normal: Vec3::y(),
            radius: 0.06,
            color: limit_color(tau_s),
        });
        out
    }

    fn readouts(&self) -> Vec<Readout> {
        let hc = hand_calc(&self.cfg, self.target_rad());
        let peak = self.history.max_abs(0, 10.0).unwrap_or(0.0);
        let power = self.history.max_abs(4, 10.0).unwrap_or(0.0);
        vec![
            Readout::new("Время симуляции", self.state.t, "с", 2),
            Readout::new("Угол в локте (факт)", self.state.q[1].to_degrees(), "°", 2),
            Readout::new(
                "Момент в локте (симуляция, 0.25 с)",
                self.sim_elbow_torque(),
                "Н·м",
                2,
            ),
            Readout::new(
                "Момент в плече (симуляция, 0.25 с)",
                self.sim_shoulder_torque(),
                "Н·м",
                2,
            ),
            Readout::new("Пик момента в локте за 10 с", peak, "Н·м", 1),
            Readout::new("Пик мощности привода локтя за 10 с", power, "Вт", 1),
            Readout::new("Плечо силы L (ось локтя → хват)", hc.lever, "м", 3),
            Readout::new(
                "Масса руки ниже локтя (предпл. + кисть)",
                self.cfg.anthro.forearm_mass() + self.cfg.anthro.hand_mass(),
                "кг",
                2,
            ),
        ]
    }

    fn comparisons(&self) -> Vec<Comparison> {
        let theta = self.target_rad();
        let hc = hand_calc(&self.cfg, theta);
        let sim = self.sim_elbow_torque();
        let sim_s = self.sim_shoulder_torque();
        let (rnea_s, rnea_e) = self.rnea_torques();
        let settled = self.settled();
        let pending = |why: &str| (Status::Pending, why.to_string());
        let wait = if self.mode == Mode::Curl {
            "идёт движение — статическая формула здесь неприменима (разгон груза добавляет момент, см. график)"
        } else {
            "переходный процесс: ждём, пока рука встанет и регулятор успокоится"
        };
        let deg = self.cfg.elbow_target_deg;
        let sin_txt = if (deg - 90.0).abs() < 1e-9 {
            String::new()
        } else {
            format!("·sin {deg:.0}°")
        };

        let simple = {
            let (status, explanation) = if !settled {
                pending(wait)
            } else {
                let d_simple = (sim - hc.payload).abs() / hc.payload.abs().max(1e-9);
                let d_full = (sim - hc.full).abs() / hc.full.abs().max(1e-9);
                if d_simple <= 0.01 {
                    (Status::Ok, "совпадает в пределах 1%".into())
                } else if d_full <= 0.01 {
                    (
                        Status::Explained,
                        format!(
                            "формула учитывает только груз; симуляция ещё держит предплечье ({:.2} Н·м), кисть ({:.2}) и ортез ({:.2}) — итого {:+.2} Н·м",
                            hc.forearm,
                            hc.hand,
                            hc.exo,
                            hc.full - hc.payload
                        ),
                    )
                } else {
                    (Status::Mismatch, "расхождение не объясняется массами руки и ортеза — в модели ошибка, числам не верить".into())
                }
            };
            Comparison {
                title: "Локоть: симуляция ↔ ручной расчёт m·g·L".into(),
                sim_label: "симуляция (привод)".into(),
                sim,
                ref_label: format!(
                    "{:.1}·{:.2}·{:.3}{sin_txt}",
                    self.cfg.payload, G_STANDARD, hc.lever
                ),
                reference: hc.payload,
                unit: "Н·м",
                decimals: 2,
                status,
                explanation,
            }
        };

        let full = {
            let (status, explanation) = if !settled {
                pending(wait)
            } else if (sim - hc.full).abs() <= 0.01 * hc.full.abs().max(1e-9) {
                (
                    Status::Ok,
                    "второй способ (сумма моментов по сегментам) совпал в пределах 1%".into(),
                )
            } else {
                (
                    Status::Mismatch,
                    "симуляция не совпала с ручным расчётом по всем массам — искать ошибку".into(),
                )
            };
            Comparison {
                title:
                    "Локоть: симуляция ↔ Σ m·g·x по всем массам (груз + кисть + предплечье + ортез)"
                        .into(),
                sim_label: "симуляция (привод)".into(),
                sim,
                ref_label: "Σ m·g·x (все массы)".into(),
                reference: hc.full,
                unit: "Н·м",
                decimals: 2,
                status,
                explanation,
            }
        };

        let rnea = {
            let ok = (rnea_e - hc.full).abs() <= 1e-3 * hc.full.abs().max(1e-6);
            let (status, explanation) = if !settled {
                pending(wait)
            } else if ok {
                (
                    Status::Ok,
                    "обратная динамика в фактической позе совпала с ручным расчётом (допуск 0.1%)"
                        .into(),
                )
            } else {
                (Status::Mismatch, "обратная динамика расходится с ручным расчётом — ошибка в модели или в формуле".into())
            };
            Comparison {
                title: "Локоть: обратная динамика (RNEA) ↔ Σ m·g·x".into(),
                sim_label: "RNEA".into(),
                sim: rnea_e,
                ref_label: "Σ m·g·x (все массы)".into(),
                reference: hc.full,
                unit: "Н·м",
                decimals: 3,
                status,
                explanation,
            }
        };

        let shoulder = {
            // Плечо вертикально ⇒ горизонтальные плечи масс от оси плеча те же, что от оси локтя.
            let ref_s = hc.full;
            let (status, explanation) = if !settled {
                pending(wait)
            } else if (sim_s - ref_s).abs() <= 0.01 * ref_s.abs().max(1e-9) {
                (
                    Status::Ok,
                    "плечо вертикально, поэтому плечевой сустав держит тот же момент, что и локоть"
                        .into(),
                )
            } else {
                (
                    Status::Mismatch,
                    "момент в плече не совпал с ручным расчётом".into(),
                )
            };
            Comparison {
                title: "Плечо: симуляция ↔ Σ m·g·x".into(),
                sim_label: "симуляция (регулятор плеча)".into(),
                sim: sim_s,
                ref_label: "Σ m·g·x (все массы)".into(),
                reference: ref_s,
                unit: "Н·м",
                decimals: 2,
                status,
                explanation,
            }
        };
        let _ = rnea_s;
        vec![simple, full, rnea, shoulder]
    }

    fn notes(&self) -> Vec<Note> {
        let tau = self.required_torque().abs();
        let (lo, hi) = HUMAN_ELBOW_LIMIT;
        let mut out = Vec::new();
        let src = if self.settled() {
            "по симуляции"
        } else {
            "по ручному расчёту (симуляция ещё не установилась)"
        };
        let men = crate::human_limits::FLEXION_MEASURED[3].1;
        let women = crate::human_limits::FLEXION_MEASURED[3].2;
        let sustained = crate::human_limits::SUSTAINED_MVC_FRACTION * women;
        if tau <= sustained {
            out.push(Note::new(Level::Good, format!("Нужно {tau:.1} Н·м в локте ({src}): это до {:.0}% максимума средней женщины, держать можно долго.", 100.0 * tau / women)));
        } else if tau <= women {
            out.push(Note::new(
                Level::Warn,
                format!("Нужно {tau:.0} Н·м ({src}): {:.0}% максимума средней женщины и {:.0}% среднего мужчины. Коротко удержат, долго нет: утомление начинается примерно с {sustained:.0} Н·м.", 100.0 * tau / women, 100.0 * tau / men),
            ));
        } else if tau <= lo {
            out.push(Note::new(
                Level::Warn,
                format!("Нужно {tau:.0} Н·м ({src}): больше максимума средней женщины ({women:.0} Н·м) и {:.0}% максимума среднего мужчины ({men:.0} Н·м). Без привода многие не удержат.", 100.0 * tau / men),
            ));
        } else if tau <= hi {
            out.push(Note::new(
                Level::Warn,
                format!("Нужно {tau:.0} Н·м ({src}) — на пределе обычного человека ({lo:.0}–{hi:.0} Н·м): удержит только коротко, на износ."),
            ));
        } else {
            out.push(Note::new(
                Level::Danger,
                format!(
                    "Нужно {tau:.0} Н·м ({src}) — в {:.1}–{:.1} раза больше предела человека ({lo:.0}–{hi:.0} Н·м). Без экзоскелета не удержать: \
                     привод должен дать минимум {:.0}–{:.0} Н·м, а надёжнее — весь момент.",
                    tau / hi,
                    tau / lo,
                    tau - hi,
                    tau - lo
                ),
            ));
        }
        out.push(Note::new(
            Level::Info,
            format!(
                "Измерения на людях (Frey-Law 2012, n = 54): при 90° средний мужчина даёт {:.0} ± {:.0} Н·м, женщина — {:.0} Н·м. Коридор 60–80 Н·м — для сильного мужчины; у женщины предел вдвое ниже.",
                crate::human_limits::FLEXION_MEASURED[3].1,
                crate::human_limits::FLEXION_SD_MEN,
                crate::human_limits::FLEXION_MEASURED[3].2
            ),
        ));
        out.push(Note::new(
            Level::Info,
            format!(
                "Шарнир, редуктор и крепления к телу считать на {:.0}–{:.0} Н·м (запас {:.0}–{:.0}×) и проверить на стенде.",
                tau * SAFETY_FACTOR.0,
                tau * SAFETY_FACTOR.1,
                SAFETY_FACTOR.0,
                SAFETY_FACTOR.1
            ),
        ));
        out.push(Note::new(
            Level::Warn,
            "Плечевой сустав держит тот же момент, что и локоть: экзоскелету одного локтя мало — нужна разгрузка плеча (здесь плечо держит отдельный регулятор).",
        ));
        out.push(Note::new(
            Level::Info,
            format!(
                "Допущения: антропометрия по Winter (2009, табл. 4.1) для {:.0} кг / {:.2} м; масса ортеза — оценка; суставы — идеальные шарниры без трения; мышцы не моделируются (момент — суммарный, человек + привод).",
                self.cfg.anthro.body_mass, self.cfg.anthro.height
            ),
        ));
        out
    }

    fn plot(&self) -> Option<Plot> {
        let sim = self.history.points(0, 10.0, 600);
        let refs = self.history.points(3, 10.0, 600);
        let simple: Vec<[f64; 2]> = refs
            .iter()
            .map(|p| [p[0], hand_calc(&self.cfg, p[1].to_radians()).payload])
            .collect();
        let full: Vec<[f64; 2]> = refs
            .iter()
            .map(|p| [p[0], hand_calc(&self.cfg, p[1].to_radians()).full])
            .collect();
        Some(Plot {
            title: "Момент в локте".into(),
            x_label: "время, с".into(),
            y_label: "Н·м".into(),
            series: vec![
                Series {
                    name: "симуляция (привод)".into(),
                    points: sim,
                    color: C_BLUE,
                    style: SeriesStyle::Solid,
                },
                Series {
                    name: "m·g·L·sinθ (только груз)".into(),
                    points: simple,
                    color: C_GRAY,
                    style: SeriesStyle::Dashed,
                },
                Series {
                    name: "Σ m·g·x (все массы)".into(),
                    points: full,
                    color: C_MOTOR,
                    style: SeriesStyle::Dashed,
                },
            ],
            bands: vec![Band {
                label: "предел обычного человека 60–80 Н·м".into(),
                lo: HUMAN_ELBOW_LIMIT.0,
                hi: HUMAN_ELBOW_LIMIT.1,
                color: [1.0, 0.3, 0.2, 0.18],
            }],
        })
    }

    fn focus(&self) -> Vec3 {
        let xw = world_to_body(&self.model, &self.state.q);
        xw[self.idx.forearm].pos
    }
}

/// Итог установившегося удержания — для тестов и отчётов без окна.
#[derive(Clone, Copy, Debug)]
pub struct HoldResult {
    pub sim_elbow: f64,
    pub sim_shoulder: f64,
    pub rnea_elbow: f64,
    pub elbow_angle_deg: f64,
    pub hand: HandCalc,
    pub settled_at: f64,
}

pub fn simulate_hold(cfg: ExoArmConfig) -> Option<HoldResult> {
    let mut arm = ExoArm::new(cfg);
    if !arm.run_until_settled(20.0) {
        return None;
    }
    // ещё полсекунды для усреднения
    let t_end = arm.state.t + 0.5;
    while arm.state.t < t_end {
        arm.step_once();
    }
    let (_, rnea_e) = arm.rnea_torques();
    Some(HoldResult {
        sim_elbow: arm.sim_elbow_torque(),
        sim_shoulder: arm.sim_shoulder_torque(),
        rnea_elbow: rnea_e,
        elbow_angle_deg: arm.state.q[1].to_degrees(),
        hand: hand_calc(&cfg, cfg.elbow_target_deg.to_radians()),
        settled_at: arm.state.t,
    })
}
