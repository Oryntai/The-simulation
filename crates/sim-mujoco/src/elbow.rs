//! Локоть на анатомической руке (шаги 2c–2e карты человека):
//! * плечи мышц в локте — аналитически (якобиан сухожилия MuJoCo) и вторым способом
//!   (конечные разности длины мышцы по углу: r = −dℓ/dθ);
//! * сила человека по углу — максимальный изометрический момент сгибания и разгибания;
//! * распределение сил мышц при удержании груза — статическая оптимизация (min Σaᵢ²);
//! * сила в суставе (сжатие и сдвиг) — из равновесия предплечья с кистью;
//! * вклад экзоскелета — момент привода и сила манжеты на предплечье.
//!
//! Допущения (важно): статика (без разгона); равновесие требуем только в локте — моменты,
//! которые те же мышцы создают в плече, запястье и для пронации, считаются уравновешенными
//! другими мышцами; критерий распределения сил — минимум суммы квадратов активаций.

use crate::Sim;

/// Где на модели локоть, кисть и предплечье.
#[derive(Clone, Debug)]
pub struct ElbowRig {
    pub joint: usize,
    pub qadr: usize,
    pub dof: usize,
    /// Первое тело за локтем (локтевая кость) — корень поддерева «предплечье + кисть».
    pub forearm_root: usize,
    /// Тело, в центр масс которого прикладываем груз (третья пястная кость — середина хвата).
    pub hand_body: usize,
    pub subtree: Vec<bool>,
}

impl ElbowRig {
    pub fn find(sim: &Sim) -> Result<Self, String> {
        let j = sim
            .joints()
            .into_iter()
            .find(|j| j.name == "elbow_flexion_r")
            .ok_or("в модели нет сустава elbow_flexion_r")?;
        let bodies = sim.bodies();
        let hand_body = ["thirdmc_r", "thirdmc", "capitate_r", "capitate"]
            .iter()
            .find_map(|n| bodies.iter().position(|(b, _)| b == n))
            .ok_or("в модели нет тела кисти (thirdmc)")?;
        Ok(Self {
            joint: j.id,
            qadr: j.qpos_adr,
            dof: j.dof_adr,
            forearm_root: j.body,
            hand_body,
            subtree: sim.subtree_mask(j.body),
        })
    }

    /// Точка оси локтя и направление оси (в мире) — после `forward`.
    pub fn axis(&self, sim: &Sim) -> ([f64; 3], [f64; 3]) {
        (sim.joint_anchor(self.joint), sim.joint_axis(self.joint))
    }

    /// Ось предплечья: от оси локтя к центру масс кисти (единичный вектор).
    pub fn forearm_dir(&self, sim: &Sim) -> [f64; 3] {
        let (c, _) = self.axis(sim);
        unit(sub(sim.body_com(self.hand_body), c))
    }

    pub fn angle(&self, sim: &Sim) -> f64 {
        sim.qpos()[self.qadr]
    }
}

/// Мышца, пересекающая локоть, в текущей позе.
#[derive(Clone, Debug)]
pub struct MuscleAtPose {
    pub actuator: usize,
    pub tendon: usize,
    pub name: String,
    /// Плечо в локте, м (+ — сгибатель), из якобиана MuJoCo.
    pub r: f64,
    /// То же, вторым способом: −dℓ/dθ конечными разностями.
    pub r_fd: f64,
    /// Активная сила при полной активации (≥ 0), Н.
    pub active: f64,
    /// Пассивная сила (≥ 0), Н.
    pub passive: f64,
}

pub(crate) fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub(crate) fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
pub(crate) fn scale(a: [f64; 3], k: f64) -> [f64; 3] {
    [a[0] * k, a[1] * k, a[2] * k]
}
pub(crate) fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub(crate) fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
pub(crate) fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}
pub(crate) fn unit(a: [f64; 3]) -> [f64; 3] {
    let n = norm(a);
    if n > 0.0 { scale(a, 1.0 / n) } else { a }
}

/// Плечи и силы всех мышц, пересекающих локоть (|r| > 1 мм), в текущей позе.
/// Меняет позу на ±h для конечных разностей и возвращает её обратно.
pub fn muscles_at_pose(sim: &mut Sim, rig: &ElbowRig) -> Vec<MuscleAtPose> {
    let muscles = sim.muscles();
    let h = 1e-5;
    let q = sim.qpos()[rig.qadr];
    let lengths = |sim: &mut Sim, v: f64| -> Vec<f64> {
        sim.qpos_mut()[rig.qadr] = v;
        sim.forward();
        muscles
            .iter()
            .map(|m| sim.actuator_length(m.actuator))
            .collect()
    };
    let lp = lengths(sim, q + h);
    let lm = lengths(sim, q - h);
    sim.qpos_mut()[rig.qadr] = q;
    sim.forward();
    muscles
        .iter()
        .enumerate()
        .filter_map(|(k, m)| {
            let jac = sim.tendon_jacobian(m.tendon);
            let r = -jac
                .iter()
                .find(|(d, _)| *d == rig.dof)
                .map(|(_, v)| *v)
                .unwrap_or(0.0);
            if r.abs() < 1e-3 {
                return None;
            }
            let mm = sim.muscle_model(m.actuator);
            let len = sim.actuator_length(m.actuator);
            Some(MuscleAtPose {
                actuator: m.actuator,
                tendon: m.tendon,
                name: m.name.clone(),
                r,
                r_fd: -(lp[k] - lm[k]) / (2.0 * h),
                active: -mm.gain(len, 0.0),
                passive: -mm.bias(len),
            })
        })
        .collect()
}

/// Сила человека в этой позе (момент, Н·м): максимум сгибания и разгибания при полной
/// активации одной группы и пассивном натяжении другой, и чисто пассивный момент.
#[derive(Clone, Copy, Debug, Default)]
pub struct Capacity {
    pub flexion: f64,
    pub extension: f64,
    pub passive: f64,
}

pub fn capacity(ms: &[MuscleAtPose]) -> Capacity {
    let passive: f64 = ms.iter().map(|m| m.r * m.passive).sum();
    let flex_active: f64 = ms
        .iter()
        .filter(|m| m.r > 0.0)
        .map(|m| m.r * m.active)
        .sum();
    let ext_active: f64 = ms
        .iter()
        .filter(|m| m.r < 0.0)
        .map(|m| -m.r * m.active)
        .sum();
    Capacity {
        flexion: flex_active + passive,
        extension: ext_active - passive,
        passive,
    }
}

/// Момент в локте, который мышцы (и экзоскелет) должны дать, чтобы держать позу с грузом
/// `payload` кг в кисти. Положительный — сгибающий.
pub fn required_torque(sim: &Sim, rig: &ElbowRig, payload: f64) -> f64 {
    let (c, a) = rig.axis(sim);
    let g = sim.gravity();
    let p = sim.body_com(rig.hand_body);
    let f = scale(g, payload);
    let q_ext = dot(a, cross(sub(p, c), f));
    sim.bias_forces()[rig.dof] - q_ext
}

/// Распределение сил мышц: min Σaᵢ² при Σ rᵢ(aᵢ·Fᵢ + Pᵢ) = τ, 0 ≤ aᵢ ≤ 1.
#[derive(Clone, Debug)]
pub struct StaticOpt {
    pub activation: Vec<f64>,
    /// Хватает ли мышц (иначе все рабочие мышцы на 100% и момента всё равно мало).
    pub feasible: bool,
    /// Максимальный момент рабочей группы в этой позе, Н·м.
    pub capacity: f64,
    pub max_activation: f64,
}

pub fn static_opt(ms: &[MuscleAtPose], tau: f64) -> StaticOpt {
    let passive: f64 = ms.iter().map(|m| m.r * m.passive).sum();
    let need = tau - passive;
    let sign = if need >= 0.0 { 1.0 } else { -1.0 };
    // Вес каждой мышцы рабочей группы: момент при полной активации.
    let w: Vec<f64> = ms
        .iter()
        .map(|m| {
            if m.r * sign > 0.0 {
                (m.r * sign) * m.active
            } else {
                0.0
            }
        })
        .collect();
    let cap: f64 = w.iter().sum();
    let target = need.abs();
    let mut activation = vec![0.0; ms.len()];
    let feasible = target <= cap;
    if target > 0.0 && cap > 0.0 {
        if !feasible {
            for (a, wi) in activation.iter_mut().zip(&w) {
                if *wi > 0.0 {
                    *a = 1.0;
                }
            }
        } else {
            // aᵢ = min(1, λ·wᵢ); Σ wᵢ·aᵢ растёт по λ — ищем λ бисекцией.
            let wmin = w
                .iter()
                .copied()
                .filter(|x| *x > 0.0)
                .fold(f64::INFINITY, f64::min);
            let (mut lo, mut hi) = (0.0, 1.0 / wmin);
            for _ in 0..200 {
                let mid = 0.5 * (lo + hi);
                let got: f64 = w.iter().map(|wi| wi * (mid * wi).min(1.0)).sum();
                if got < target { lo = mid } else { hi = mid }
            }
            let lam = 0.5 * (lo + hi);
            for (a, wi) in activation.iter_mut().zip(&w) {
                *a = (lam * wi).min(1.0);
            }
        }
    }
    let max_activation = activation.iter().copied().fold(0.0, f64::max);
    StaticOpt {
        activation,
        feasible,
        capacity: cap * sign + passive,
        max_activation,
    }
}

/// Экзоскелет: момент привода в локте и манжета на предплечье.
#[derive(Clone, Copy, Debug)]
pub struct ExoLoad {
    /// Момент привода, Н·м (+ — помогает сгибать).
    pub torque: f64,
    /// Расстояние манжеты от оси локтя вдоль предплечья, м.
    pub cuff_distance: f64,
}

/// Сила, которой плечевая кость (хрящ, связки, капсула) давит на предплечье в локте.
#[derive(Clone, Copy, Debug)]
pub struct JointReaction {
    pub force: [f64; 3],
    pub magnitude: f64,
    /// Сжатие вдоль предплечья (+ — кости прижаты друг к другу), Н.
    pub compression: f64,
    /// Сдвиг поперёк предплечья, Н.
    pub shear: f64,
    /// Сумма натяжений мышц, пересекающих локоть, Н.
    pub muscle_tension: f64,
    /// Сила манжеты экзоскелета на предплечье, Н.
    pub cuff_force: f64,
}

/// Равновесие предплечья с кистью: сила в суставе = −(вес сегментов + груз + мышцы + манжета).
pub fn joint_reaction(
    sim: &Sim,
    rig: &ElbowRig,
    ms: &[MuscleAtPose],
    activation: &[f64],
    payload: f64,
    exo: Option<ExoLoad>,
) -> JointReaction {
    let g = sim.gravity();
    let mut f = [0.0; 3];
    for (b, (_, m)) in sim.bodies().iter().enumerate() {
        if rig.subtree[b] {
            f = add(f, scale(g, *m));
        }
    }
    f = add(f, scale(g, payload));
    let mut tension_sum = 0.0;
    for (m, a) in ms.iter().zip(activation) {
        let t = a * m.active + m.passive;
        tension_sum += t;
        let path = sim.tendon_path_bodies(m.tendon);
        for w in path.windows(2) {
            if let (Some((p0, b0)), Some((p1, b1))) = (w[0], w[1]) {
                let (in0, in1) = (rig.subtree[b0], rig.subtree[b1]);
                if in0 != in1 {
                    // Отрезок пересекает границу: на точку внутри — натяжение к точке снаружи.
                    let (pin, pout) = if in0 { (p0, p1) } else { (p1, p0) };
                    f = add(f, scale(unit(sub(pout, pin)), t));
                }
            }
        }
    }
    let mut cuff_force = 0.0;
    if let Some(exo) = exo
        && exo.torque != 0.0
        && exo.cuff_distance > 0.0
    {
        let (c, a) = rig.axis(sim);
        let u = rig.forearm_dir(sim);
        // Точка манжеты и направление, в котором её толкает привод (касательная к дуге).
        let pc = add(c, scale(u, exo.cuff_distance));
        let lever = sub(pc, c);
        let lever_perp = sub(lever, scale(a, dot(lever, a)));
        let t = unit(cross(a, lever_perp));
        cuff_force = exo.torque / norm(lever_perp);
        f = add(f, scale(t, cuff_force));
    }
    let r = scale(f, -1.0);
    let u = rig.forearm_dir(sim);
    let compression = dot(r, u);
    let shear = norm(sub(r, scale(u, compression)));
    JointReaction {
        force: r,
        magnitude: norm(r),
        compression,
        shear,
        muscle_tension: tension_sum,
        cuff_force: cuff_force.abs(),
    }
}

/// Точка кривой «сила человека — угол».
#[derive(Clone, Copy, Debug)]
pub struct CurvePoint {
    pub angle_deg: f64,
    pub capacity: Capacity,
}

/// Кривая силы человека по углу в локте (остальные суставы — как сейчас). Поза восстанавливается.
pub fn torque_angle_curve(
    sim: &mut Sim,
    rig: &ElbowRig,
    from_deg: f64,
    to_deg: f64,
    step_deg: f64,
) -> Vec<CurvePoint> {
    let q = sim.qpos()[rig.qadr];
    let mut out = Vec::new();
    let mut d = from_deg;
    while d <= to_deg + 1e-9 {
        sim.qpos_mut()[rig.qadr] = d.to_radians();
        sim.forward();
        let ms = muscles_at_pose(sim, rig);
        out.push(CurvePoint {
            angle_deg: d,
            capacity: capacity(&ms),
        });
        d += step_deg;
    }
    sim.qpos_mut()[rig.qadr] = q;
    sim.forward();
    out
}

/// Несоосность оси экзоскелета и оси локтя: если ось привода смещена на `offset` м, а привод
/// поворачивается вместе с рукой, манжета на предплечье уходит относительно кожи на
/// δ = 2·|e|·sin(|Δθ|/2), где Δθ — поворот от позы, в которой ортез надели. Ткань под манжетой
/// упругая (жёсткость k Н/м) ⇒ паразитная сила k·δ (Schiele & van der Helm, 2006, плоская модель).
pub fn misalignment_force(offset: f64, stiffness: f64, delta_angle: f64) -> (f64, f64) {
    let delta = 2.0 * offset.abs() * (0.5 * delta_angle.abs()).sin();
    (delta, stiffness * delta)
}
