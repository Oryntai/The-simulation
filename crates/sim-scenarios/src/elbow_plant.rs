//! Локоть как объект управления для прошивки в контуре: одна степень свободы (сгибание),
//! плечо вертикально, θ = 0 — рука опущена, θ > 0 — сгибание.
//!
//! Массу, центр масс и момент инерции предплечья с кистью берём из анатомической модели
//! (MS-Human-700, рука с кистью, через MuJoCo: диагональ матрицы масс по оси локтя), груз —
//! точечная масса в кисти. Сверка — антропометрия Winter (табл. 4.1, по Dempster 1955):
//! «предплечье + кисть»: 0.022 массы тела, радиус инерции относительно локтя 0.827 длины
//! предплечья (локоть → шиловидный отросток, 0.146 роста).
//!
//! Упоры: ниже 0° и выше предела сгибания — жёсткая пружина с демпфером (модель контакта),
//! момент в упоре пишется как событие безопасности.

use sim_mujoco::elbow::{ElbowRig, required_torque};
use sim_mujoco::{Sim, models};

pub const G: f64 = 9.80665;

/// Параметры локтя.
#[derive(Clone, Debug, PartialEq)]
pub struct ElbowParams {
    /// Масса предплечья с кистью, кг.
    pub mass: f64,
    /// Расстояние от оси локтя до центра масс предплечья с кистью, м.
    pub com: f64,
    /// Расстояние от оси локтя до хвата (центр кисти), м.
    pub hand: f64,
    /// Момент инерции предплечья с кистью относительно оси локтя, кг·м².
    pub inertia: f64,
    /// Вязкое трение сустава и тканей, Н·м·с/рад (оценка: пассивное демпфирование локтя
    /// ~0.1–0.3 Н·м·с/рад у расслабленного человека).
    pub damping: f64,
    /// Предел сгибания, рад.
    pub max_angle: f64,
}

impl ElbowParams {
    /// Из модели MS-Human-700 (рука с кистью) через MuJoCo.
    pub fn from_anatomy() -> Result<Self, String> {
        let mut sim = Sim::load(&models::ms_human_700_arm())?;
        let rig = ElbowRig::find(&sim)?;
        let q0 = sim.qpos0().to_vec();
        sim.qpos_mut().copy_from_slice(&q0);
        sim.qpos_mut()[rig.qadr] = std::f64::consts::FRAC_PI_2;
        let c = sim.joint_couplings();
        sim.apply_joint_couplings(&c);
        sim.forward();
        let nv = sim.nv();
        let m = sim.full_mass_matrix();
        let inertia = m[rig.dof * nv + rig.dof];
        let mass: f64 = (0..sim.nbody())
            .filter(|&b| rig.subtree[b])
            .map(|b| sim.body_mass(b))
            .sum();
        // При 90° предплечье горизонтально: момент веса = m·g·r_c.
        let hold = required_torque(&sim, &rig, 0.0).abs();
        let com = hold / (mass * G);
        let (p, _) = rig.axis(&sim);
        let h = sim.body_com(rig.hand_body);
        let hand = ((h[0] - p[0]).powi(2) + (h[1] - p[1]).powi(2) + (h[2] - p[2]).powi(2)).sqrt();
        let hi = sim
            .joints()
            .into_iter()
            .find(|j| j.name == "elbow_flexion_r")
            .map(|j| j.range[1])
            .unwrap_or(2.27);
        Ok(Self {
            mass,
            com,
            hand,
            inertia,
            damping: 0.2,
            max_angle: hi,
        })
    }

    /// Запасной вариант без MuJoCo: Winter для человека 75 кг, 1.75 м.
    pub fn winter(body_mass: f64, height: f64) -> Self {
        let l = 0.146 * height;
        let mass = 0.022 * body_mass;
        Self {
            mass,
            com: 0.682 * l,
            hand: l + 0.05,
            inertia: mass * (0.827 * l).powi(2),
            damping: 0.2,
            max_angle: 145f64.to_radians(),
        }
    }
}

/// Внешние условия для локтя.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ElbowLoad {
    /// Груз в кисти, кг.
    pub payload: f64,
    /// Доля веса руки, которую снимает вода (0 в воздухе, ~0.88 в воде).
    pub arm_buoyancy: f64,
    /// Доля веса груза, которую снимает вода (чугун ~0.14).
    pub payload_buoyancy: f64,
    /// Коэффициент квадратичного сопротивления среды: τ = −k·ω·|ω|, Н·м·с²/рад².
    pub drag_k: f64,
}

impl Default for ElbowLoad {
    fn default() -> Self {
        Self {
            payload: 0.0,
            arm_buoyancy: 0.0,
            payload_buoyancy: 0.0,
            drag_k: 0.0,
        }
    }
}

/// Препятствие на пути сгибания: то, во что упирается рука (стол, край, другая часть тела).
/// Контакт — пружина с демпфером, которая умеет только толкать.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Obstacle {
    /// Угол локтя, при котором звено коснулось препятствия, рад.
    pub angle: f64,
    /// Расстояние от оси локтя до точки контакта, м.
    pub lever: f64,
    /// Жёсткость контакта, Н/м (для тела — эффективная жёсткость участка).
    pub stiffness: f64,
    /// Демпфирование контакта, Н·с/м. Ноль даёт наибольшую силу удара.
    pub damping: f64,
}

/// Состояние локтя.
#[derive(Clone, Debug)]
pub struct ElbowPlant {
    pub p: ElbowParams,
    pub load: ElbowLoad,
    pub theta: f64,
    pub omega: f64,
    /// Момент в упоре на последнем шаге, Н·м (0 — не в упоре).
    pub stop_torque: f64,
    /// Мотор заклинило: сустав не движется (отказ для проверки прошивки).
    pub jammed: bool,
    pub obstacle: Option<Obstacle>,
    /// Сила в контакте с препятствием на последнем шаге, Н.
    pub contact_force: f64,
}

/// Жёсткость и демпфирование упора (оценка: связки + мягкие ткани + жёсткий упор ортеза).
const STOP_K: f64 = 400.0;
const STOP_B: f64 = 8.0;

impl ElbowPlant {
    pub fn new(p: ElbowParams, theta: f64) -> Self {
        Self {
            p,
            load: ElbowLoad::default(),
            theta,
            omega: 0.0,
            stop_torque: 0.0,
            jammed: false,
            obstacle: None,
            contact_force: 0.0,
        }
    }

    fn obstacle_force(&self) -> f64 {
        let Some(o) = self.obstacle else {
            return 0.0;
        };
        let depth = o.lever * (self.theta - o.angle);
        if depth <= 0.0 {
            return 0.0;
        }
        (o.stiffness * depth + o.damping * o.lever * self.omega).max(0.0)
    }

    /// Момент инерции с грузом (и отражённой инерцией ротора `extra`), кг·м².
    pub fn inertia(&self, extra: f64) -> f64 {
        self.p.inertia + self.load.payload * self.p.hand * self.p.hand + extra
    }

    /// Момент силы тяжести с поправкой на воду (знак: + сгибает), Н·м.
    pub fn gravity_torque(&self, theta: f64) -> f64 {
        let arm = self.p.mass * self.p.com * (1.0 - self.load.arm_buoyancy);
        let pay = self.load.payload * self.p.hand * (1.0 - self.load.payload_buoyancy);
        -(arm + pay) * G * theta.sin()
    }

    /// Сколько нужно держать в угле θ (статически), Н·м.
    pub fn holding_torque(&self, theta: f64) -> f64 {
        -self.gravity_torque(theta)
    }

    fn stop(&self) -> f64 {
        if self.theta < 0.0 {
            -STOP_K * self.theta - STOP_B * self.omega.min(0.0)
        } else if self.theta > self.p.max_angle {
            -STOP_K * (self.theta - self.p.max_angle) - STOP_B * self.omega.max(0.0)
        } else {
            0.0
        }
    }

    /// Шаг полу-неявным Эйлером: `tau` — сумма моментов привода и человека (+ сгибает),
    /// `extra_inertia` — отражённая инерция ротора J·N².
    pub fn step(&mut self, dt: f64, tau: f64, extra_inertia: f64) {
        if self.jammed {
            self.omega = 0.0;
            self.stop_torque = 0.0;
            self.contact_force = self.obstacle_force();
            return;
        }
        self.stop_torque = self.stop();
        self.contact_force = self.obstacle_force();
        let contact_torque = -self.contact_force * self.obstacle.map_or(0.0, |o| o.lever);
        let drag = -self.load.drag_k * self.omega * self.omega.abs();
        let total = tau + self.gravity_torque(self.theta) - self.p.damping * self.omega
            + drag
            + self.stop_torque
            + contact_torque;
        self.omega += dt * total / self.inertia(extra_inertia);
        self.theta += dt * self.omega;
    }

    /// Энергия (кинетическая + потенциальная), Дж — для проверки интегратора.
    pub fn energy(&self, extra_inertia: f64) -> f64 {
        let arm = self.p.mass * self.p.com * (1.0 - self.load.arm_buoyancy);
        let pay = self.load.payload * self.p.hand * (1.0 - self.load.payload_buoyancy);
        0.5 * self.inertia(extra_inertia) * self.omega * self.omega
            + (arm + pay) * G * (1.0 - self.theta.cos())
    }
}
