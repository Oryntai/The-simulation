use sim_mujoco::ffi::mjtIntegrator;

use crate::config::{Config, Mode};
use crate::solve::DriveGroup;

// Для схемы средней точки фазовый сдвиг за шаг равен 2 atan(x/2).
// Его относительная ошибка ограничена x²/12 при x=h√(K/J).
// Для экспоненциального затухания абсолютная ошибка множителя
// между exp(−x) и (1−x/2)/(1+x/2) ограничена x³/12 при x≥0.
pub(crate) const ACCURACY: f64 = 0.01;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Applies {
    Always,
    Powered,
    Holding,
}

#[derive(Clone, Copy)]
struct Contribution {
    parameter: &'static str,
    applies: Applies,
    stiffness: f64,
    damping: f64,
    rotor_inverse: f64,
}

#[derive(Clone)]
pub(crate) struct DriveLimit {
    pub(crate) dof: usize,
    pub(crate) axis: usize,
    terms: Vec<Contribution>,
    /// Ось привода — вход передачи (ремня, шестерни, винта) из `[[transmissions]]`.
    pub(crate) transmission_input: bool,
}

impl DriveLimit {
    pub(crate) fn new(dof: usize, config: Config) -> Result<Self, String> {
        let e = config.electrical;
        let m = config.mechanical;
        let mut terms = Vec::new();
        use Applies::{Always, Holding, Powered};
        let mut add = |parameter, applies, stiffness, damping, rotor_inverse| {
            if stiffness != 0.0 || damping != 0.0 {
                terms.push(Contribution {
                    parameter,
                    applies,
                    stiffness,
                    damping,
                    rotor_inverse,
                });
            }
        };
        if let Some(c) = m.coupling {
            // Самая сильная реакция со стороны ротора соответствует
            // нагрузке в прямом потоке мощности: F/(N·η_forward).
            let rotor = c.inertia_motor * m.ratio * m.ratio * m.forward;
            if !rotor.is_finite() || rotor <= 0.0 {
                return Err("приведённая инерция податливой передачи вне диапазона".into());
            }
            add("coupling_stiffness", Always, c.stiffness, 0.0, 1.0 / rotor);
            add("coupling_damping", Always, 0.0, c.damping, 1.0 / rotor);
        } else {
            // На обратном ходе |F| = |τ|·N/η_back. Кривая фрикционного
            // сопротивления может иметь и отрицательный наклон.
            let reflected = m.ratio * m.ratio / m.backward;
            // При alpha ≥ 0 нижняя граница R(T) задаёт наибольшую ЭДС/R.
            let resistance = config.minimum_resistance()?;
            add(
                "motor_constant² / R(T) (противо-ЭДС)",
                Powered,
                0.0,
                e.k * (e.k / resistance) * reflected,
                0.0,
            );
            add(
                "трение (static_friction_nm / friction_band_rad_s, stribeck_rad_s, viscous_nm_per_rad_s)",
                Always,
                0.0,
                m.friction.max_slope() * reflected,
                0.0,
            );
            if let Mode::StepRate { poles, .. } = config.mode {
                // Номинальный предел нормы двух фаз. Сверхток моделируется
                // отдельно и не подтверждает точность модели коммутации.
                add(
                    "motor_constant · current_limit_a · pole_pairs",
                    Powered,
                    e.k * e.limit * 2.0_f64.sqrt() * poles * reflected,
                    0.0,
                    0.0,
                );
            }
        }
        if let Some(s) = m.stop {
            add("stop_stiffness", Always, s.stiffness, 0.0, 0.0);
            add("stop_damping", Always, 0.0, s.damping, 0.0);
        }
        for (hold, parameter) in [(m.lock, "hold_stiffness"), (m.brake, "brake_stiffness")] {
            if let Some(h) = hold {
                add(parameter, Holding, h.stiffness, 0.0, 0.0);
            }
        }
        if let Some(term) = terms.iter().find(|term| {
            ![term.stiffness, term.damping, term.rotor_inverse]
                .into_iter()
                .all(f64::is_finite)
        }) {
            return Err(format!(
                "{}: коэффициент проверки шага вне численного диапазона",
                term.parameter
            ));
        }
        Ok(Self {
            dof,
            axis: 0,
            terms,
            transmission_input: false,
        })
    }
}

impl DriveGroup {
    pub(crate) fn check_accuracy(&self, dt: f64, integrator: u32) -> Result<(), String> {
        let spring_budget = (12.0 * ACCURACY).sqrt();
        let damping_budget = (12.0 * ACCURACY).cbrt();
        let mut stiffness = 0.0;
        let mut damping = 0.0;
        let mut dominant = None;
        let mut largest = 0.0;
        for drive in &self.drives {
            let (diagonal, inverse) = self.inertia[drive.limit.axis];
            for term in &drive.limit.terms {
                if integrator != mjtIntegrator::mjINT_EULER {
                    return Err(format!(
                        "привод «{}», {}: неявный шаг проверен для Euler, выбран интегратор с другой формулой перемещения (например, RK4)",
                        drive.id.name, term.parameter
                    ));
                }
                if (term.applies == Applies::Powered && !drive.model.electrical_feedback())
                    || (term.applies == Applies::Holding && !drive.model.unpowered())
                {
                    continue;
                }
                let inverse = inverse + term.rotor_inverse;
                let k = term.stiffness * inverse;
                let c = term.damping * inverse;
                if !k.is_finite() || !c.is_finite() {
                    return Err(format!(
                        "привод «{}», {}: расчёт частоты вышел за диапазон",
                        drive.id.name, term.parameter
                    ));
                }
                // След ограничивает старшую моду даже при нескольких приводах на одной оси.
                stiffness += k;
                damping += c;
                let score = (k.sqrt() / spring_budget).max(c / damping_budget);
                if score > largest {
                    largest = score;
                    dominant = Some((drive, term, diagonal, inverse));
                }
            }
        }
        let required = (stiffness.sqrt() / spring_budget).max(damping / damping_budget);
        if !required.is_finite() {
            return Err("суммарная необходимая частота физики вышла за диапазон".into());
        }
        if required * dt > 1.0
            && let Some((drive, term, diagonal, inverse)) = dominant
        {
            return Err(format!(
                concat!(
                    "привод «{}», ось «{}»[{}], {}: Mᵢᵢ={:.9e}, ",
                    "обратная инерция с учётом связанных степеней свободы и ротора {:.9e}. ",
                    "Нужна частота физики ≥ {:.0} Гц (сейчас {:.9e} Гц, оценка точности неявного шага)"
                ),
                drive.id.name,
                drive.joint,
                drive.limit.dof,
                term.parameter,
                diagonal,
                inverse,
                required.ceil(),
                1.0 / dt,
            ) + if drive.limit.transmission_input {
                // Лёгкий вал между жёсткостью привода и мягкой связью передачи
                // даёт быструю моду; инерция за передачей в оценку не входит.
                ". Ось — вход передачи из [[transmissions]]: опишите привод на выходной оси, \
                 gear_ratio умножьте на 1/ratio передачи, КПД привода — на КПД передачи, \
                 податливость передачи сложите с coupling_stiffness последовательно, \
                 шаг энкодера на валу привода умножьте на ratio (README, «Привод за передачей»)"
            } else {
                ""
            });
        }
        Ok(())
    }
}
