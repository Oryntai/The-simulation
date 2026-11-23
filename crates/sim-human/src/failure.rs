//! Отказ привода посреди подъёма: помощь пропадает за заданное время.
//!
//! Модель движения — три степени свободы (подъём плеча, локоть, L5/S1) с настоящей матрицей масс
//! MuJoCo, приведённой к этим координатам с учётом ритма лопатки, плюс груз. Остальные суставы
//! считаются удержанными.
//!
//! Человек до реакции держит прежний момент: рефлексы и жёсткость мышц не засчитаны, это
//! в опасную сторону. После задержки (реакция + электромеханическая задержка) момент нарастает:
//! в падающем суставе — к пределу силы, в остальных — к моменту удержания.

use nalgebra::{DMatrix, DVector, Vector3};

use crate::data::*;
use crate::load::BodyScale;
use crate::rig::*;

/// Координаты движения при отказе.
pub const FAILURE_COORDS: [usize; 3] = [C_ELEV, C_ELBOW, C_L5_FE];

/// Численный шаг, с. Сходимость проверяется повтором с половинным шагом.
const DT: f64 = 0.002;
/// Модель масс и якобианы обновляем каждые 10 мс (численная оценка).
const REFRESH_EVERY: usize = 5;
/// Оценка горизонта захвата, с, после реакции и окончания спада момента.
const MAX_TIME_S: f64 = 1.5;
/// Численный порог обнаружения разворота, рад/с. Сам по себе не является допуском безопасности.
const REST_SPEED: f64 = 0.02;

/// Постановка отказа.
#[derive(Clone, Copy, Debug)]
pub struct FailureInput {
    pub pose: BodyPose,
    /// Скорости координат FAILURE_COORDS в момент отказа, рад/с.
    pub rates: [f64; 3],
    /// Момент помощи до отказа, Н·м.
    pub assist: [f64; 3],
    /// Момент мышц до отказа, Н·м.
    pub muscle: [f64; 3],
    pub payload_kg: f64,
    pub lever_m: f64,
    pub scale: BodyScale,
    pub two_handed: bool,
    /// За сколько секунд помощь падает до нуля.
    pub loss_time_s: f64,
    /// Задержка до начала роста момента, с.
    pub delay_s: f64,
    /// Постоянная роста момента, с.
    pub rise_tau_s: f64,
}

/// Результат отказа.
#[derive(Clone, Copy, Debug)]
pub struct FailureOutcome {
    /// Наибольшая скорость падения по суставам руки, °/с.
    pub peak_speed_deg_s: f64,
    /// Наибольшее отклонение от позы отказа, °.
    pub max_drop_deg: f64,
    /// Падение остановлено силой человека.
    pub stopped: bool,
    /// Сустав дошёл до края диапазона.
    pub hit_limit: bool,
    pub stop_time_s: f64,
    /// Наибольший по модулю момент мышц при торможении, Н·м.
    pub peak_muscle: [f64; 3],
    /// Поза в момент наибольшей загрузки.
    pub peak_pose: BodyPose,
    /// Момент, нужный только чтобы держать исходную позу без помощи, Н·м.
    pub hold_torque: [f64; 3],
    /// Невязка баланса энергии относительно работы или изменения энергии.
    pub energy_error: f64,
    /// Падение в направлении силы тяжести началось до роста мышечного момента.
    pub fell_before_reaction: bool,
    /// Доля помощи, оставшаяся к началу роста мышечного момента.
    pub assist_fraction_at_reaction: f64,
    pub end_pose: BodyPose,
}

#[derive(Clone, Copy)]
pub(crate) struct FailureFrame {
    pub pose: BodyPose,
    pub rates: [f64; 3],
    pub acceleration: [f64; 3],
    pub assist_fraction: f64,
    pub braking: bool,
}

/// Часть уравнения движения, пересчитываемая по позе.
struct Frame {
    mass: DMatrix<f64>,
    gravity: DVector<f64>,
    /// Кориолисовы и центробежные силы тела и груза.
    velocity_terms: DVector<f64>,
    potential: f64,
}

/// Поза при отклонении координат от базовой. Наклон таза не меняется: разгибается поясница.
pub fn with_angles(base: &BodyPose, x: &[f64; 3]) -> BodyPose {
    let mut p = *base;
    let tilt = base.pelvis_tilt();
    p.shoulder_elevation = base.shoulder_elevation + x[0];
    p.elbow_flexion = base.elbow_flexion + x[1];
    // Положительная координата L5_S1_FE — разгибание.
    p.lumbar_flexion = base.lumbar_flexion - x[2];
    p.trunk_inclination = tilt + p.lumbar_flexion;
    p
}

fn vec3(a: [f64; 3]) -> Vector3<f64> {
    Vector3::new(a[0], a[1], a[2])
}

fn frame(
    rig: &mut BodyRig,
    input: &FailureInput,
    x: &[f64; 3],
    rates: &[f64; 3],
    columns: &[Vec<f64>],
) -> Frame {
    let (sl, sm) = (input.scale.length, input.scale.mass);
    let coords = FAILURE_COORDS;
    let pose = with_angles(&input.pose, x);

    let m = rig.reduced_mass(&pose, columns, input.two_handed);
    let mut mass = DMatrix::from_row_slice(3, 3, &m) * (sm * sl * sl);

    // Кориолисовы силы тела: разность смещающих сил со скоростью и без неё.
    let b0 = rig.reduced_bias(&pose, &coords, &[0.0; 3], columns, input.two_handed);
    let bv = rig.reduced_bias(&pose, &coords, rates, columns, input.two_handed);
    let mut velocity_terms =
        DVector::from_iterator(3, b0.iter().zip(&bv).map(|(a, b)| (b - a) * sm * sl * sl));

    let probe = rig.probe(&pose, &coords, input.two_handed);
    let g = vec3(probe.gravity);
    let mut gravity = DVector::from_iterator(3, probe.tau_gravity.iter().map(|t| t * sm * sl));
    let mut potential = sm * sl * probe.potential_body;

    // Квадратичный по скорости член груза: вторая производная точки вдоль скорости.
    let speed = rates.iter().fold(0.0_f64, |a, b| a.max(b.abs()));
    // Численные шаги для второй разности точки груза вдоль траектории.
    let quad = if speed > 1e-6 && input.payload_kg > 0.0 {
        let h = 1e-3 / speed;
        let plus = [h * rates[0], h * rates[1], h * rates[2]];
        let minus = [-plus[0], -plus[1], -plus[2]];
        let shifted = |a: &[f64; 3]| [x[0] + a[0], x[1] + a[1], x[2] + a[2]];
        let (hp, dp) =
            rig.hand_points(&with_angles(&input.pose, &shifted(&plus)), input.two_handed);
        let (hm, dm) = rig.hand_points(
            &with_angles(&input.pose, &shifted(&minus)),
            input.two_handed,
        );
        let mut out = [Vector3::zeros(); 2];
        for side in 0..2 {
            let point = |p: [f64; 3], u: [f64; 3]| vec3(p) * sl + vec3(u) * input.lever_m;
            let center = point(probe.hand[side], probe.dir[side]);
            out[side] =
                (point(hp[side], dp[side]) - 2.0 * center + point(hm[side], dm[side])) / (h * h);
        }
        out
    } else {
        [Vector3::zeros(); 2]
    };

    let shares = if input.two_handed {
        [0.5, 0.5]
    } else {
        [1.0, 0.0]
    };
    for side in 0..2 {
        if shares[side] == 0.0 {
            continue;
        }
        let kg = input.payload_kg * shares[side];
        let mut jac = [Vector3::zeros(); 3];
        for (p, slot) in jac.iter_mut().enumerate() {
            *slot = vec3(probe.dpoint[side][p]) * sl + vec3(probe.ddir[side][p]) * input.lever_m;
            gravity[p] += kg * (-g).dot(slot);
            velocity_terms[p] += kg * quad[side].dot(slot);
        }
        for p in 0..3 {
            for q in 0..3 {
                mass[(p, q)] += kg * jac[p].dot(&jac[q]);
            }
        }
        let point = vec3(probe.hand[side]) * sl + vec3(probe.dir[side]) * input.lever_m;
        potential += kg * (-g).dot(&point);
    }
    Frame {
        mass,
        gravity,
        velocity_terms,
        potential,
    }
}

fn kinetic(f: &Frame, w: &[f64; 3]) -> f64 {
    let v = DVector::from_row_slice(w);
    0.5 * v.dot(&(&f.mass * &v))
}

/// Интегрирование после отказа.
pub fn simulate(
    rig: &mut BodyRig,
    input: &FailureInput,
    capacity: impl Fn(&BodyPose, usize, f64) -> f64,
    refined: bool,
) -> Result<(FailureOutcome, Vec<FailureFrame>), &'static str> {
    let subdivision = if refined { 2 } else { 1 };
    let dt = DT / subdivision as f64;
    let refresh = REFRESH_EVERY * subdivision;
    let ranges = FAILURE_COORDS.map(|k| rig.coord_range(k));
    let base = input.pose;
    let start_q = [
        base.shoulder_elevation,
        base.elbow_flexion,
        -base.lumbar_flexion,
    ];
    // Связи суставов линейны, поэтому столбцы B считаем один раз.
    let columns = rig.coupling_columns(&base, &FAILURE_COORDS, input.two_handed);

    let mut x = [0.0; 3];
    let mut w = input.rates;
    let mut f = frame(rig, input, &x, &w, &columns);
    let hold_torque = [f.gravity[0], f.gravity[1], f.gravity[2]];
    let initial_energy = kinetic(&f, &w) + f.potential;

    let start_muscle = input.muscle;
    let mut muscle = input.muscle;
    let mut peak_muscle = [0.0_f64; 3];
    let mut peak_ratio = 0.0_f64;
    let mut peak_pose = base;
    let mut peak_speed = 0.0_f64;
    let mut max_drop = 0.0_f64;
    let mut fell = [false; 3];
    let mut fell_before_reaction = false;
    let mut stopped = false;
    let mut hit_limit = false;
    let horizon = input.loss_time_s.max(input.delay_s) + MAX_TIME_S;
    let mut work = 0.0;
    let mut t = 0.0;
    let mut step = 0usize;
    let rise = 1.0 - (-dt / input.rise_tau_s).exp();
    let mut trace = Vec::new();
    let pose_at = |x: &[f64; 3]| with_angles(&base, x);
    let fade_at = |t: f64| {
        if input.loss_time_s > 0.0 {
            (1.0 - t / input.loss_time_s).max(0.0)
        } else {
            0.0
        }
    };

    while t < horizon {
        if step > 0 && step.is_multiple_of(refresh) {
            f = frame(rig, input, &x, &w, &columns);
        }
        let fade = fade_at(t);
        if t >= input.delay_s {
            for p in 0..3 {
                let need = f.gravity[p];
                let falling = w[p] * need < 0.0 && w[p].abs() > REST_SPEED;
                let cap = capacity(&pose_at(&x), p, need);
                if !cap.is_finite() || cap < 0.0 {
                    return Err("Неконечная сила реакции человека.");
                }
                let target = if falling {
                    // Для захвата не засчитываем запас эксцентрической силы сверх изометрии.
                    need.signum() * cap
                } else {
                    need.clamp(-cap, cap)
                };
                muscle[p] += (target - muscle[p]) * rise;
            }
        } else {
            muscle = start_muscle;
        }
        let drive = DVector::from_fn(3, |p, _| muscle[p] + input.assist[p] * fade);
        let rhs = &drive - &f.gravity - &f.velocity_terms;
        if f.mass.iter().chain(rhs.iter()).any(|x| !x.is_finite()) || !f.potential.is_finite() {
            return Err("Неконечная матрица масс, обобщённая сила или потенциальная энергия.");
        }
        let acc = f
            .mass
            .clone()
            .cholesky()
            .ok_or("Вырожденная матрица масс при отказе.")?
            .solve(&rhs);
        if acc.iter().any(|x| !x.is_finite()) {
            return Err("Неконечное ускорение после отказа.");
        }
        if step.is_multiple_of(refresh) {
            let braking = t >= input.delay_s
                && (0..3).any(|p| {
                    w[p].abs() > REST_SPEED && w[p] * f.gravity[p] < 0.0 && w[p] * acc[p] < 0.0
                });
            trace.push(FailureFrame {
                pose: pose_at(&x),
                rates: w,
                acceleration: [acc[0], acc[1], acc[2]],
                assist_fraction: fade,
                braking,
            });
        }
        for p in 0..3 {
            let wn = w[p] + dt * acc[p];
            work += drive[p] * 0.5 * (w[p] + wn) * dt;
            x[p] += 0.5 * (w[p] + wn) * dt;
            w[p] = wn;
            if w[p] * f.gravity[p] < 0.0 && w[p].abs() > REST_SPEED {
                fell[p] = true;
                fell_before_reaction |= t + dt <= input.delay_s;
            }
        }
        if x.iter().chain(w.iter()).any(|x| !x.is_finite()) || !work.is_finite() {
            return Err("Неконечная поза, скорость или работа после отказа.");
        }
        t += dt;
        step += 1;

        let mut ratio = 0.0_f64;
        for p in 0..3 {
            if p < 2 && w[p] * f.gravity[p] < 0.0 {
                peak_speed = peak_speed.max(w[p].abs());
            }
            max_drop = max_drop.max(x[p].abs());
            let q = start_q[p] + x[p];
            if (q <= ranges[p][0] && w[p] < 0.0) || (q >= ranges[p][1] && w[p] > 0.0) {
                hit_limit = true;
            }
            if muscle[p].abs() > peak_muscle[p].abs() {
                peak_muscle[p] = muscle[p];
            }
            let cap = capacity(&pose_at(&x), p, muscle[p]);
            if cap > 0.0 {
                ratio = ratio.max(muscle[p].abs() / cap);
            }
        }
        if ratio > peak_ratio {
            peak_ratio = ratio;
            peak_pose = with_angles(&base, &x);
        }
        if hit_limit {
            break;
        }
        if t > input.delay_s && t > input.loss_time_s {
            // Падавшие суставы развернулись, остальные стоят.
            let done = (0..3).all(|p| {
                if fell[p] {
                    w[p] * f.gravity[p] >= 0.0 || w[p].abs() < REST_SPEED
                } else {
                    w[p].abs() < REST_SPEED
                }
            });
            if done {
                stopped = true;
                break;
            }
        }
    }
    // При упоре интегратор заканчивает движение до расчёта контакта.
    let end = frame(rig, input, &x, &w, &columns);
    let final_energy = kinetic(&end, &w) + end.potential;
    let energy_delta = final_energy - initial_energy;
    // Один джоуль — численный масштаб для окрестности нулевой работы.
    let scale = energy_delta.abs().max(work.abs()).max(1.0);
    let energy_error = (energy_delta - work).abs() / scale;
    if !energy_error.is_finite() {
        return Err("Неконечная погрешность баланса энергии после отказа.");
    }
    Ok((
        FailureOutcome {
            peak_speed_deg_s: peak_speed.to_degrees(),
            max_drop_deg: max_drop.to_degrees(),
            stopped,
            hit_limit,
            stop_time_s: t,
            peak_muscle,
            peak_pose,
            hold_torque,
            energy_error,
            fell_before_reaction,
            assist_fraction_at_reaction: fade_at(input.delay_s),
            end_pose: pose_at(&x),
        },
        trace,
    ))
}

/// Оценка задержки до роста момента: верх окна произвольной реакции + консервативная ЭМЗ, с.
pub fn worst_reaction_delay() -> f64 {
    REACTION_VOLUNTARY_S.1 + EMD_CONSERVATIVE_S
}
