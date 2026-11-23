//! Рука в воде и на ветру: выталкивающая сила и сопротивление среды.
//!
//! * Сопротивление считает MuJoCo (модель «эквивалентного бруска» по инерции тела:
//!   вязкое + квадратичное сопротивление, подъёмная сила; `opt.density`, `opt.viscosity`,
//!   `opt.wind`). Выталкивающую силу MuJoCo в этой модели НЕ считает (проверено: в покое
//!   qfrc_fluid = 0) — её добавляем сами по закону Архимеда.
//! * Объём сегмента = масса / плотность сегмента. Плотности — Dempster 1955 по таблице 4.1
//!   у Winter, «Biomechanics and Motor Control of Human Movement», 4-е изд.: кисть 1.16,
//!   предплечье 1.13, плечо 1.07, туловище 1.03 г/см³ (у живого человека с воздухом в лёгких
//!   туловище легче воды — тело плавает).
//! * Второй способ для сопротивления — ручной расчёт: предплечье с кистью как цилиндр
//!   диаметра D и длины L, вращающийся вокруг локтя: τ = ½·ρ·C_d·D·ω²·L⁴/4 (поток
//!   поперёк цилиндра), для ветра скорости v: F = ½·ρ·C_d·D·L·v², τ = F·L/2.

use crate::Sim;
use crate::elbow::{ElbowRig, cross, dot, sub};

/// Плотность сегмента по имени тела модели (OpenSim-имена), кг/м³.
pub fn segment_density(body_name: &str) -> f64 {
    let n = body_name.to_ascii_lowercase();
    if n.contains("ulna") || n.contains("radius") {
        1130.0
    } else if n.contains("humerus") {
        1070.0
    } else if n.contains("thorax") || n.contains("lumbar") || n.contains("pelvis") {
        1030.0
    } else {
        // Кисть и пальцы (кости запястья, пястные, фаланги).
        1160.0
    }
}

/// Объём поддерева тел `mask` по массам и плотностям сегментов, м³.
pub fn subtree_volume(sim: &Sim, mask: &[bool]) -> f64 {
    sim.bodies()
        .iter()
        .enumerate()
        .filter(|(b, _)| mask[*b])
        .map(|(b, (name, _))| sim.body_mass(b) / segment_density(name))
        .sum()
}

/// Момент выталкивающей силы относительно оси локтя для предплечья с кистью, Н·м
/// (знак — как у обобщённой силы сустава: положительный помогает сгибанию).
/// `submerged` — доля объёма под водой (0…1).
pub fn buoyancy_elbow_torque(sim: &Sim, rig: &ElbowRig, rho_fluid: f64, submerged: f64) -> f64 {
    let (p, axis) = rig.axis(sim);
    let g = sim.gravity();
    let mut tau = 0.0;
    for (b, (name, _)) in sim.bodies().iter().enumerate() {
        if !rig.subtree[b] {
            continue;
        }
        let v = sim.body_mass(b) / segment_density(name) * submerged;
        // Сила Архимеда направлена против тяжести: −ρ·V·g.
        let f = [
            -rho_fluid * v * g[0],
            -rho_fluid * v * g[1],
            -rho_fluid * v * g[2],
        ];
        // Точка приложения — центр объёма ≈ центр масс сегмента.
        tau += dot(cross(sub(sim.body_com(b), p), f), axis);
    }
    tau
}

/// Момент сил тяжести предплечья с кистью относительно оси локтя, Н·м (тот же знак).
pub fn gravity_elbow_torque(sim: &Sim, rig: &ElbowRig) -> f64 {
    let (p, axis) = rig.axis(sim);
    let g = sim.gravity();
    let mut tau = 0.0;
    for (b, _) in rig.subtree.iter().enumerate().filter(|(_, on)| **on) {
        let m = sim.body_mass(b);
        tau += dot(
            cross(sub(sim.body_com(b), p), [m * g[0], m * g[1], m * g[2]]),
            axis,
        );
    }
    tau
}

/// Ручной расчёт: момент сопротивления вращающегося цилиндра, Н·м (модуль).
pub fn cylinder_rotation_drag(rho: f64, cd: f64, diameter: f64, length: f64, omega: f64) -> f64 {
    0.5 * rho * cd * diameter * omega * omega * length.powi(4) / 4.0
}

/// Ручной расчёт: момент от равномерного потока поперёк цилиндра, закреплённого одним концом, Н·м.
pub fn cylinder_crossflow_torque(rho: f64, cd: f64, diameter: f64, length: f64, speed: f64) -> f64 {
    0.5 * rho * cd * diameter * length * speed * speed * length / 2.0
}

/// Сопротивление среды в локте по MuJoCo при угловой скорости `omega` (рад/с) и ветре/течении
/// `wind` (м/с): ставит среду, считает `forward`, возвращает qfrc_fluid по оси локтя
/// и восстанавливает скорости.
pub fn mujoco_fluid_elbow_torque(
    sim: &mut Sim,
    rig: &ElbowRig,
    density: f64,
    viscosity: f64,
    wind: [f64; 3],
    omega: f64,
) -> f64 {
    let saved: Vec<f64> = sim.qvel_mut().to_vec();
    sim.qvel_mut().iter_mut().for_each(|v| *v = 0.0);
    sim.qvel_mut()[rig.dof] = omega;
    sim.set_medium(density, viscosity, wind);
    sim.forward();
    let tau = sim.fluid_forces()[rig.dof];
    sim.set_medium(0.0, 0.0, [0.0; 3]);
    sim.qvel_mut().copy_from_slice(&saved);
    sim.forward();
    tau
}

/// Длина «предплечья с кистью» от оси локтя до кончика пальцев (оценка по модели):
/// наибольшее расстояние от оси до центров масс тел поддерева + 4 см (полфаланги).
pub fn forearm_hand_length(sim: &Sim, rig: &ElbowRig) -> f64 {
    let (p, _) = rig.axis(sim);
    (0..sim.nbody())
        .filter(|&b| rig.subtree[b])
        .map(|b| {
            let d = sub(sim.body_com(b), p);
            dot(d, d).sqrt()
        })
        .fold(0.0, f64::max)
        + 0.04
}
