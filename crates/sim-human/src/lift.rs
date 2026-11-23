//! Траектория подъёма минимального рывка (Flash & Hogan, 1985, J Neurosci 5(7):1688):
//! s(τ) = 10τ³ − 15τ⁴ + 6τ⁵, τ = t / T.

use crate::rig::BodyPose;

/// Положение, скорость и ускорение нормированной траектории при τ ∈ [0, 1].
pub fn min_jerk(tau: f64) -> (f64, f64, f64) {
    if !tau.is_finite() {
        return (f64::NAN, f64::NAN, f64::NAN);
    }
    let t = tau.clamp(0.0, 1.0);
    let (t2, t3) = (t * t, t * t * t);
    (
        t3 * (10.0 - 15.0 * t + 6.0 * t2),
        t2 * (30.0 - 60.0 * t + 30.0 * t2),
        t * (60.0 - 180.0 * t + 120.0 * t2),
    )
}

/// Пиковая скорость траектории: 1.875·Δ/T (при τ = 0.5).
pub const MIN_JERK_PEAK_SPEED: f64 = 1.875;
fn mix(a: f64, b: f64, s: f64) -> f64 {
    a + (b - a) * s
}

/// Поза на траектории от `start` до `end` в момент `t` при длительности `duration`.
pub fn pose_at(start: &BodyPose, end: &BodyPose, t: f64, duration: f64) -> BodyPose {
    let (s, _, _) = min_jerk(t / duration);
    BodyPose {
        shoulder_plane: mix(start.shoulder_plane, end.shoulder_plane, s),
        shoulder_elevation: mix(start.shoulder_elevation, end.shoulder_elevation, s),
        shoulder_rotation: mix(start.shoulder_rotation, end.shoulder_rotation, s),
        elbow_flexion: mix(start.elbow_flexion, end.elbow_flexion, s),
        forearm_rotation: mix(start.forearm_rotation, end.forearm_rotation, s),
        trunk_inclination: mix(start.trunk_inclination, end.trunk_inclination, s),
        lumbar_flexion: mix(start.lumbar_flexion, end.lumbar_flexion, s),
        lumbar_lateral_bend: mix(start.lumbar_lateral_bend, end.lumbar_lateral_bend, s),
        lumbar_rotation: mix(start.lumbar_rotation, end.lumbar_rotation, s),
    }
}

/// Наибольшая угловая скорость суставов на траектории, рад/с.
pub fn peak_joint_speed(start: &BodyPose, end: &BodyPose, duration: f64) -> f64 {
    let deltas = [
        end.shoulder_plane - start.shoulder_plane,
        end.shoulder_elevation - start.shoulder_elevation,
        end.shoulder_rotation - start.shoulder_rotation,
        end.elbow_flexion - start.elbow_flexion,
        end.forearm_rotation - start.forearm_rotation,
        end.trunk_inclination - start.trunk_inclination,
        end.lumbar_flexion - start.lumbar_flexion,
        end.lumbar_lateral_bend - start.lumbar_lateral_bend,
        end.lumbar_rotation - start.lumbar_rotation,
    ];
    let max = deltas.iter().fold(0.0_f64, |m, d| m.max(d.abs()));
    MIN_JERK_PEAK_SPEED * max / duration
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn endpoints_rest_to_rest() {
        let (s0, v0, a0) = min_jerk(0.0);
        let (s1, v1, a1) = min_jerk(1.0);
        assert_eq!((s0, v0, a0), (0.0, 0.0, 0.0));
        assert!((s1 - 1.0).abs() < 1e-12 && v1.abs() < 1e-12 && a1.abs() < 1e-12);
    }

    #[test]
    fn derivatives_match_finite_differences() {
        let h = 1e-6;
        for i in 1..20 {
            let t = i as f64 / 20.0;
            let (_, v, a) = min_jerk(t);
            let dv = (min_jerk(t + h).0 - min_jerk(t - h).0) / (2.0 * h);
            let da = (min_jerk(t + h).1 - min_jerk(t - h).1) / (2.0 * h);
            assert!((v - dv).abs() < 1e-6 && (a - da).abs() < 1e-6);
        }
    }
}
