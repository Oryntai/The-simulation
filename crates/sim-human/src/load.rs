//! Нагрузка в точке движения: нужный момент, распределение сил мышц и силы в суставах.
//!
//! Сила человека берётся из измерений. Модель MS-Human-700 сильнее людей (на 34–42% в локте при
//! 90°), поэтому пиковая сила каждой группы мышц приводится к измеренной силе данного человека
//! ([`Calibration`]). Без этого активации были бы занижены, а запас завышен.

use nalgebra::{DMatrix, DVector, Vector3};

use crate::population::{JointAction, MuscleVariant, Person, StrengthEstimate, measured_strength};
use crate::rig::*;
use crate::solver::solve_activation;

/// Масштаб человека относительно модели.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyScale {
    pub length: f64,
    pub mass: f64,
}

/// Что считаем: груз, его вынос и помощь устройства.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LoadInput {
    pub payload_kg: f64,
    /// Вынос груза от центра кисти вдоль предплечья, м.
    pub lever_m: f64,
    /// Момент помощи по координатам, Н·м (+ — по положительному направлению координаты).
    pub assist: [f64; NC],
    pub scale: BodyScale,
}

fn v(a: [f64; 3]) -> Vector3<f64> {
    Vector3::new(a[0], a[1], a[2])
}

/// Ускорение груза минус g и производная точки груза, в единицах человека.
fn load_terms(s: &GeometrySample, h: &HandKin, input: &LoadInput, inertia: bool) -> Vector3<f64> {
    let (sl, l) = (input.scale.length, input.lever_m);
    let g = v(s.gravity);
    if inertia {
        v(h.accel) * sl + v(h.dir_accel) * l - g
    } else {
        -g
    }
}

/// Обобщённый момент, который должны дать мышцы и устройство вместе, Н·м.
/// `inertia = false` — та же поза без разгона (статика).
pub fn required_torque(s: &GeometrySample, input: &LoadInput, inertia: bool) -> [f64; NC] {
    let (sl, sm) = (input.scale.length, input.scale.mass);
    let mut tau = [0.0; NC];
    for (k, t) in tau.iter_mut().enumerate() {
        *t = sm * sl * s.tau_gravity[k];
        if inertia {
            *t += sm * sl * sl * s.tau_inertia[k];
        }
        for h in &s.hands {
            if h.share == 0.0 {
                continue;
            }
            let e = load_terms(s, h, input, inertia);
            let d = v(h.dpoint[k]) * sl + v(h.ddir[k]) * input.lever_m;
            *t += input.payload_kg * h.share * e.dot(&d);
        }
    }
    tau
}

/// Измеренная сила по действиям в данной позе.
#[derive(Clone, Copy, Debug)]
pub struct Strengths {
    pub elbow_flexion: StrengthEstimate,
    pub elbow_extension: StrengthEstimate,
    pub shoulder_up: StrengthEstimate,
    pub shoulder_down: StrengthEstimate,
    pub trunk_extension: StrengthEstimate,
}

impl Strengths {
    pub fn at(pose: &BodyPose, person: &Person) -> Self {
        let elbow = pose.elbow_flexion.to_degrees();
        let shoulder = pose.shoulder_elevation.to_degrees();
        Self {
            elbow_flexion: measured_strength(JointAction::ElbowFlexion, elbow, person),
            elbow_extension: measured_strength(JointAction::ElbowExtension, elbow, person),
            shoulder_up: measured_strength(JointAction::ShoulderElevation, shoulder, person),
            shoulder_down: measured_strength(JointAction::ShoulderDepression, shoulder, person),
            trunk_extension: measured_strength(JointAction::TrunkExtension, 0.0, person),
        }
    }

    /// Какое действие нужно для момента `tau` по координате и какова его сила.
    pub fn for_torque(&self, coord: usize, tau: f64) -> Option<(JointAction, f64)> {
        self.estimate_for_torque(coord, tau)
            .map(|(action, limit)| (action, limit.nm))
    }

    pub(crate) fn estimate_for_torque(
        &self,
        coord: usize,
        tau: f64,
    ) -> Option<(JointAction, StrengthEstimate)> {
        match coord {
            C_ELBOW if tau >= 0.0 => Some((JointAction::ElbowFlexion, self.elbow_flexion)),
            C_ELBOW => Some((JointAction::ElbowExtension, self.elbow_extension)),
            C_ELEV if tau >= 0.0 => Some((JointAction::ShoulderElevation, self.shoulder_up)),
            C_ELEV => Some((JointAction::ShoulderDepression, self.shoulder_down)),
            C_L5_FE if tau >= 0.0 => Some((JointAction::TrunkExtension, self.trunk_extension)),
            _ => None,
        }
    }

    pub(crate) fn minimum(self, other: Self) -> Self {
        let min = |a: StrengthEstimate, b: StrengthEstimate| {
            if a.nm.is_nan() || a.nm <= b.nm { a } else { b }
        };
        Self {
            elbow_flexion: min(self.elbow_flexion, other.elbow_flexion),
            elbow_extension: min(self.elbow_extension, other.elbow_extension),
            shoulder_up: min(self.shoulder_up, other.shoulder_up),
            shoulder_down: min(self.shoulder_down, other.shoulder_down),
            trunk_extension: min(self.trunk_extension, other.trunk_extension),
        }
    }

    pub(crate) fn values(&self) -> impl Iterator<Item = f64> {
        [
            self.elbow_flexion,
            self.elbow_extension,
            self.shoulder_up,
            self.shoulder_down,
            self.trunk_extension,
        ]
        .into_iter()
        .flat_map(|s| [s.nm, s.mean_nm])
    }
}

/// Координаты, для которых есть измерения силы.
pub const MEASURED_COORDS: [usize; 3] = [C_ELEV, C_ELBOW, C_L5_FE];

/// Сила группы мышц по координате в модели (при полной активации), Н·м.
/// `sign` — направление, `dynamic` — с учётом скорости.
pub fn group_capacity(
    s: &GeometrySample,
    coord: usize,
    sign: f64,
    length_scale: f64,
    dynamic: bool,
) -> f64 {
    s.muscles
        .iter()
        .filter(|m| m.r[coord] * sign > 0.0)
        .map(|m| {
            let f = if dynamic { m.active } else { m.active_iso };
            (m.r[coord] * sign) * length_scale * f
        })
        .sum()
}

/// Приведение силы мышц модели к измеренной силе человека.
#[derive(Clone, Debug)]
pub struct Calibration {
    /// Множитель силы по каждой мышце выборки.
    pub factors: Vec<f64>,
    /// Отношение «модель / измерение» по действиям (для отчёта).
    pub overstrength: Vec<(JointAction, f64)>,
}

/// Для каждой мышцы — наименьший множитель из групп, в которых она работает: ни одна группа не
/// получается сильнее измеренной. Мышцы без измеренной группы берут наименьший из всех множителей.
pub fn calibrate(s: &GeometrySample, strengths: &Strengths, scale: &BodyScale) -> Calibration {
    let groups: [(usize, f64, JointAction, f64); 5] = [
        (
            C_ELBOW,
            1.0,
            JointAction::ElbowFlexion,
            strengths.elbow_flexion.nm,
        ),
        (
            C_ELBOW,
            -1.0,
            JointAction::ElbowExtension,
            strengths.elbow_extension.nm,
        ),
        (
            C_ELEV,
            1.0,
            JointAction::ShoulderElevation,
            strengths.shoulder_up.nm,
        ),
        (
            C_ELEV,
            -1.0,
            JointAction::ShoulderDepression,
            strengths.shoulder_down.nm,
        ),
        (
            C_L5_FE,
            1.0,
            JointAction::TrunkExtension,
            strengths.trunk_extension.nm,
        ),
    ];
    let mut ratios = Vec::with_capacity(5);
    let mut overstrength = Vec::with_capacity(5);
    for &(coord, sign, action, measured) in &groups {
        let model = group_capacity(s, coord, sign, scale.length, false);
        if model > 0.0 && measured > 0.0 {
            ratios.push((coord, sign, (measured / model).min(1.0)));
            overstrength.push((action, model / measured));
        } else if measured <= 0.0 {
            ratios.push((coord, sign, 0.0));
        }
    }
    let default = ratios.iter().map(|x| x.2).fold(1.0_f64, f64::min);
    let factors = s
        .muscles
        .iter()
        .map(|m| {
            let mut c = f64::INFINITY;
            for &(coord, sign, ratio) in &ratios {
                if m.r[coord] * sign > 0.0 {
                    c = c.min(ratio);
                }
            }
            if c.is_finite() { c } else { default }
        })
        .collect();
    Calibration {
        factors,
        overstrength,
    }
}

/// Итог расчёта мышц и суставов в одной точке.
#[derive(Clone, Debug)]
pub struct JointLoads {
    /// Момент фактически найденных активных и пассивных тяг, Н·м.
    pub achieved_torque: [f64; NC],
    pub activation: Vec<f64>,
    pub max_activation: f64,
    /// Наибольший дефицит момента при исходной силе мышц, Н·м.
    pub deficit_nm: f64,
    /// Нижняя оценка неизбежной невязки, полученная опорной плоскостью множества моментов.
    pub certified_deficit_nm: f64,
    pub converged: bool,
    pub elbow_n: f64,
    pub shoulder_n: f64,
    pub spine_compression_n: f64,
    pub spine_shear_n: f64,
    pub muscle_tension_n: Vec<f64>,
}

/// Допустимая невязка момента, Н·м.
pub(crate) const RESIDUAL_TOL_NM: f64 = 0.05;

/// Распределить момент по мышцам и посчитать силы в суставах.
///
/// При нехватке силы возвращается ограниченное решение с невязкой. Реакции такого решения
/// непригодны для оценки равновесной нагрузки.
pub fn solve_loads(
    s: &GeometrySample,
    input: &LoadInput,
    calibration: &Calibration,
    variant: MuscleVariant,
    muscle_torque: &[f64; NC],
    use_velocity: bool,
) -> JointLoads {
    let n = s.muscles.len();
    let r_scale = input.scale.length * variant.moment_arm;
    let mut wmat = DMatrix::zeros(NC, n);
    let mut b = DVector::from_row_slice(muscle_torque);
    let mut active = vec![0.0; n];
    let mut passive = vec![0.0; n];
    for (j, m) in s.muscles.iter().enumerate() {
        let c = calibration.factors[j] * variant.f0;
        active[j] = c * if use_velocity { m.active } else { m.active_iso };
        passive[j] = c * m.passive;
        for k in 0..NC {
            let r = m.r[k] * r_scale;
            wmat[(k, j)] = r * active[j];
            b[k] -= r * passive[j];
        }
    }
    let sol = solve_activation(&wmat, &b);
    let deficit_nm = sol.max_residual();
    // Для любого y: y·Wa ≤ Σ max(y·Wᵢ, 0). Нормировка ‖y‖₁ даёт границу для нормы ∞.
    let normal = -DVector::from_row_slice(&sol.residual);
    let normal_l1 = normal.abs().sum();
    let certified_deficit_nm = if normal_l1 > 0.0 {
        let direction = normal / normal_l1;
        let support: f64 = wmat.tr_mul(&direction).iter().map(|&x| x.max(0.0)).sum();
        (direction.dot(&b) - support).max(0.0)
    } else {
        0.0
    };
    let tension: Vec<f64> = (0..n)
        .map(|j| sol.activation[j] * active[j] + passive[j])
        .collect();

    let reaction = |link: usize, sides: &[usize]| -> Vector3<f64> {
        let l = &s.links[link];
        let (sl, sm) = (input.scale.length, input.scale.mass);
        let mut f = (v(l.body_gravity)
            + if use_velocity {
                v(l.body_inertia) * sl
            } else {
                Vector3::zeros()
            })
            * sm;
        for &side in sides {
            let h = &s.hands[side];
            if h.share > 0.0 {
                f += load_terms(s, h, input, use_velocity) * (input.payload_kg * h.share);
            }
        }
        for (j, m) in s.muscles.iter().enumerate() {
            f -= v(m.pull[link]) * tension[j];
        }
        f
    };
    let elbow = reaction(LINK_ELBOW, &[0]);
    let shoulder = reaction(LINK_SHOULDER, &[0]);
    let spine = reaction(LINK_SPINE, &[0, 1]);
    let axis = v(s.spine_axis);
    let axial = spine.dot(&axis);
    let shear = (spine - axis * axial).norm();

    let max_activation = sol.activation.iter().copied().fold(0.0, f64::max);
    JointLoads {
        achieved_torque: std::array::from_fn(|k| muscle_torque[k] + sol.residual[k]),
        activation: sol.activation,
        max_activation,
        deficit_nm,
        certified_deficit_nm,
        converged: sol.converged,
        elbow_n: elbow.norm(),
        shoulder_n: shoulder.norm(),
        spine_compression_n: axial.max(0.0),
        spine_shear_n: shear,
        muscle_tension_n: tension,
    }
}
