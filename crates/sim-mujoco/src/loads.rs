//! Куда уходит нагрузка от груза в руке дальше локтя: плечевой сустав и поясница (L5/S1).
//!
//! Момент считаем прямо из равновесия: всё, что выше/дальше сустава (кости по центрам масс +
//! груз), даёт момент Σ (cᵢ − p) × mᵢ·g относительно точки сустава p. Экзоскелет локтя,
//! который держится манжетами на плече и предплечье, этот момент не меняет: его силы — внутренние
//! для руки.
//!
//! Сжатие позвоночника в L5/S1 — статическая схема Chaffin: момент наклона вперёд уравновешивают
//! разгибатели спины с плечом ~5 см, плюс вес всего, что выше. Пределы NIOSH (1981): 3400 Н —
//! предел действия, 6400 Н — максимально допустимое.

use crate::Sim;
use crate::elbow::{add, cross, dot, norm, scale, sub};

/// Плечо разгибателей спины относительно L5/S1 в схеме Chaffin, м.
pub const ERECTOR_LEVER_M: f64 = 0.05;
/// NIOSH 1981: предел действия и максимально допустимое сжатие L5/S1, Н.
pub const NIOSH_ACTION_LIMIT_N: f64 = 3400.0;
pub const NIOSH_MAX_PERMISSIBLE_N: f64 = 6400.0;

/// Момент (вектор, Н·м) от веса поддерева `mask` и груза относительно точки `p`.
pub fn gravity_moment(
    sim: &Sim,
    p: [f64; 3],
    mask: &[bool],
    payload: Option<([f64; 3], f64)>,
) -> [f64; 3] {
    let g = sim.gravity();
    let mut m = [0.0; 3];
    for (b, (_, mass)) in sim.bodies().iter().enumerate() {
        if mask[b] && *mass > 0.0 {
            m = add(m, cross(sub(sim.body_com(b), p), scale(g, *mass)));
        }
    }
    if let Some((at, mass)) = payload {
        m = add(m, cross(sub(at, p), scale(g, mass)));
    }
    m
}

#[derive(Clone, Copy, Debug)]
pub struct ProximalLoads {
    /// Момент в плечевом суставе (модуль), Н·м.
    pub shoulder_moment: f64,
    /// Момент в L5/S1, наклоняющий корпус вперёд/вбок (модуль), Н·м.
    pub l5s1_moment: f64,
    /// Масса всего, что выше L5/S1 (без груза), кг.
    pub upper_body_mass: f64,
    /// Оценка сжатия L5/S1 по схеме Chaffin, Н.
    pub l5s1_compression: f64,
}

/// Нагрузки плеча и поясницы при грузе `payload` кг в кисти (тело кисти `hand_body`).
/// `None`, если в модели нет нужных тел (плечевого сустава или позвонка L5).
pub fn proximal_loads(sim: &Sim, hand_body: usize, payload: f64) -> Option<ProximalLoads> {
    let joints = sim.joints();
    let sh = joints.iter().find(|j| j.name == "shoulder_elv_r")?;
    let bodies = sim.bodies();
    let humerus = bodies.iter().position(|(n, _)| n == "humerus_r")?;
    let l5 = bodies.iter().position(|(n, _)| n == "lumbar5")?;
    let hand = Some((sim.body_com(hand_body), payload));

    let p_sh = sim.joint_anchor(sh.id);
    let shoulder = gravity_moment(sim, p_sh, &sim.subtree_mask(humerus), hand);

    let (p_l5, _) = sim.body_pose(l5);
    let upper = sim.subtree_mask(l5);
    let m_l5 = gravity_moment(sim, p_l5, &upper, hand);
    // Сжатие: момент / плечо разгибателей + вес всего, что выше (вдоль вертикали).
    let g = norm(sim.gravity());
    let upper_mass = sim.subtree_mass(l5);
    let up = scale(sim.gravity(), -1.0 / g);
    let horizontal_moment = norm(sub(m_l5, scale(up, dot(m_l5, up))));
    let compression = horizontal_moment / ERECTOR_LEVER_M + (upper_mass + payload) * g;
    Some(ProximalLoads {
        shoulder_moment: norm(shoulder),
        l5s1_moment: horizontal_moment,
        upper_body_mass: upper_mass,
        l5s1_compression: compression,
    })
}
