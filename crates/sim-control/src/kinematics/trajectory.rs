use sim_signal::Time;

use super::{KinematicError, Result, Tree};

#[derive(Clone, Copy, Debug)]
pub struct MotionLimit {
    /// Модуль скорости в единицах координаты в секунду.
    pub max_speed: f64,
    /// Модуль ускорения в единицах координаты на секунду в квадрате.
    pub max_acceleration: f64,
}

#[derive(Clone, Debug)]
pub struct Waypoint {
    pub at: Time,
    pub coordinates: Vec<f64>,
}

#[derive(Clone, Debug)]
pub struct TrajectorySample {
    pub coordinates: Vec<f64>,
    pub velocity: Vec<f64>,
    pub acceleration: Vec<f64>,
}

#[derive(Clone, Debug)]
pub struct Trajectory {
    waypoints: Vec<Waypoint>,
}

/// Секунды по точной разности виртуальных времён.
fn seconds(later: Time, earlier: Time) -> f64 {
    Time(later.0 - earlier.0).as_secs()
}

impl Trajectory {
    /// Квинтические участки с нулевыми скоростью и ускорением в узлах.
    /// Пределы проверяются непрерывно в координатах соединений, в том числе между узлами.
    /// Граничные случаи в пределах ошибки округления отклоняются консервативно.
    pub fn new(
        tree: &Tree,
        waypoints: Vec<Waypoint>,
        motion_limits: &[MotionLimit],
    ) -> Result<Self> {
        if waypoints.is_empty() || motion_limits.len() != tree.coordinate_limits().len() {
            return Err(KinematicError::Invalid(
                "нет узлов или пределы заданы не для всех координат".into(),
            ));
        }
        for limit in motion_limits {
            if !limit.max_speed.is_finite()
                || limit.max_speed < 0.0
                || !limit.max_acceleration.is_finite()
                || limit.max_acceleration < 0.0
            {
                return Err(KinematicError::Invalid("неверный предел движения".into()));
            }
        }
        for point in &waypoints {
            tree.validate_coordinates(&point.coordinates)?;
        }
        for pair in waypoints.windows(2) {
            let [from, to] = pair else {
                unreachable!("окно из двух узлов")
            };
            if to.at <= from.at {
                return Err(KinematicError::Invalid(
                    "времена узлов должны возрастать".into(),
                ));
            }
            // Нижняя граница длительности учитывает преобразование целого и деление на 10⁹.
            let dt_lower =
                (((to.at.0 - from.at.0) as f64).next_down() / 1_000_000_000.0).next_down();
            for (coordinate, ((&a, &b), limit)) in from
                .coordinates
                .iter()
                .zip(&to.coordinates)
                .zip(motion_limits)
                .enumerate()
            {
                if a == b {
                    continue;
                }
                let distance = (b - a).abs().next_up();
                // h = 10u³ − 15u⁴ + 6u⁵. max h′ = 15/8 при u=1/2.
                // max |h″| = 10/√3 при u=(3 ± √3)/6.
                // Каждое округление направлено наружу, включая область субнормальных чисел.
                let distance_over_time = (distance / dt_lower).next_up();
                let speed = (distance_over_time * (15.0 / 8.0)).next_up();
                let factor = (10.0 / 3.0_f64.sqrt().next_down()).next_up();
                let acceleration = ((distance_over_time / dt_lower).next_up() * factor).next_up();
                if !speed.is_finite() || !acceleration.is_finite() {
                    return Err(KinematicError::Numerical(
                        "переполнение предела траектории".into(),
                    ));
                }
                if speed > limit.max_speed || acceleration > limit.max_acceleration {
                    return Err(KinematicError::Invalid(format!(
                        "превышен предел движения координаты {coordinate}"
                    )));
                }
            }
        }
        Ok(Self { waypoints })
    }

    pub fn sample(&self, at: Time) -> Result<TrajectorySample> {
        match self.waypoints.binary_search_by_key(&at, |point| point.at) {
            Ok(index) => {
                let coordinates = self.waypoints[index].coordinates.clone();
                let size = coordinates.len();
                Ok(TrajectorySample {
                    coordinates,
                    velocity: vec![0.0; size],
                    acceleration: vec![0.0; size],
                })
            }
            Err(index) if index > 0 && index < self.waypoints.len() => {
                let from = &self.waypoints[index - 1];
                let to = &self.waypoints[index];
                let dt = seconds(to.at, from.at);
                let left = at.0 - from.at.0;
                let right = to.at.0 - at.0;
                let u = left.min(right) as f64 / (to.at.0 - from.at.0) as f64;
                let other = 1.0 - u;
                let blend = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
                let slope = 30.0 * u * u * other * other;
                let curvature = 60.0 * u * other * (1.0 - 2.0 * u);
                let mut result = TrajectorySample {
                    coordinates: Vec::new(),
                    velocity: Vec::new(),
                    acceleration: Vec::new(),
                };
                for (&a, &b) in from.coordinates.iter().zip(&to.coordinates) {
                    let delta = b - a;
                    let (position, sign) = if left <= right {
                        (a + delta * blend, 1.0)
                    } else {
                        (b - delta * blend, -1.0)
                    };
                    result.coordinates.push(position.clamp(a.min(b), a.max(b)));
                    result.velocity.push((delta / dt) * slope);
                    result
                        .acceleration
                        .push(((delta / dt) / dt) * curvature * sign);
                }
                Ok(result)
            }
            Err(_) => Err(KinematicError::Invalid(
                "время за пределами траектории".into(),
            )),
        }
    }
}
