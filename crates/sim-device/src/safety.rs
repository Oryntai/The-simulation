//! Что устройство может сделать с телом: удар, зажим и раздавливание в зазоре.
//!
//! * удар: F = ω·√(k·J) (модель TS 15066 с закреплённым участком тела, m_H⁻¹ ≈ 0);
//! * зажим: F = τ/r, где r — ближайшая к оси точка, в которой звено давит на тело;
//! * зазор: ISO 13854, в узком месте не меньше размера самой крупной части тела.

use crate::mechanics::{Analysis, Catalog};
use crate::model::*;

/// Удар и зажим для пары «шарнир — участок тела».
#[derive(Clone, Debug, PartialEq)]
pub struct ContactResult {
    pub joint_link: usize,
    pub body_link: usize,
    pub region: BodyRegion,
    pub lever_m: f64,
    pub speed_m_s: f64,
    pub impact_n: Option<f64>,
    pub impact_limit_n: f64,
    pub clamp_n: Option<f64>,
    pub clamp_limit_n: f64,
}

/// Сила удара для инерции `j` на оси при угловой скорости `omega` о тело жёсткостью `k`.
pub fn impact_force(omega_rad_s: f64, k_n_per_m: f64, j_kg_m2: f64) -> f64 {
    omega_rad_s.abs() * (k_n_per_m * j_kg_m2).sqrt()
}

/// Наибольшая скорость, при которой удар не превышает `limit_n`, рад/с.
pub fn safe_speed(limit_n: f64, k_n_per_m: f64, j_kg_m2: f64) -> f64 {
    limit_n / (k_n_per_m * j_kg_m2).sqrt()
}

/// Удар и зажим для всех приводных шарниров и всех звеньев за ними, которые рядом с телом.
pub fn contact_results(d: &Device, a: &Analysis, cat: &dyn Catalog) -> Vec<ContactResult> {
    let mut out = Vec::new();
    for j in &a.joints {
        let spec = &d.links[j.link].joint;
        if spec.actuator.is_empty() {
            continue;
        }
        let Some(act) = cat.actuator(&spec.actuator) else {
            continue;
        };
        let torque = if spec.torque_limit_nm > 0.0 {
            Some(spec.torque_limit_nm)
        } else {
            act.peak_torque_nm
        };
        for k in d.subtree(j.link) {
            let l = &d.links[k];
            let Some(limit) = l.near_body.contact() else {
                continue;
            };
            let lever_m = l.contact_lever_mm * 1e-3;
            out.push(ContactResult {
                joint_link: j.link,
                body_link: k,
                region: l.near_body,
                lever_m,
                speed_m_s: j.speed_rad_s * lever_m,
                impact_n: j
                    .inertia_complete
                    .then(|| impact_force(j.speed_rad_s, limit.stiffness_n_per_m, j.inertia_kg_m2)),
                impact_limit_n: TRANSIENT_FACTOR * limit.quasi_static_n,
                clamp_n: torque.map(|torque| (torque + j.gravity_nm) / lever_m),
                clamp_limit_n: limit.quasi_static_n,
            });
        }
    }
    out
}

/// Зазор в подвижном шарнире рядом с телом.
#[derive(Clone, Debug, PartialEq)]
pub struct GapResult {
    pub link: usize,
    pub region: BodyRegion,
    pub gap_mm: f64,
    pub required_mm: f64,
    pub guarded: bool,
}

impl GapResult {
    /// Зазор достаточен, либо зона заявлена закрытой кожухом.
    pub fn ok(&self) -> bool {
        self.gap_mm.is_finite()
            && self.required_mm.is_finite()
            && (self.guarded || self.gap_mm >= self.required_mm)
    }
}

/// Зазоры всех подвижных шарниров, звенья которых работают рядом с телом.
pub fn crush_gaps(d: &Device) -> Vec<GapResult> {
    d.links
        .iter()
        .enumerate()
        .filter(|(_, l)| l.joint.kind == JointKind::Hinge)
        .filter_map(|(i, l)| {
            l.near_body.crush_gap_mm().map(|required_mm| GapResult {
                link: i,
                region: l.near_body,
                gap_mm: l.joint.pinch_gap_mm,
                required_mm,
                guarded: l.joint.guarded,
            })
        })
        .collect()
}
