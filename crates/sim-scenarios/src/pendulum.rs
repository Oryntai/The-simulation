//! Сценарий «Маятник»: эталон в 3D. Видно, где формула малых углов перестаёт работать,
//! и как плохой интегратор (явный Эйлер) «накачивает» энергию.

use sim_core::dynamics::{kinetic_energy, potential_energy, world_to_body};
use sim_core::{G_STANDARD, Inputs, Integrator, Pose, Shape, State, Vec3, Visual, step};
use sim_verify::analytic as an;
use sim_verify::cases::simple_pendulum;
use sim_verify::measure::CrossingDetector;

use crate::scenario::*;

const DT: f64 = 0.001;
const PIVOT_Z: f64 = 2.4;

pub struct Pendulum {
    length: f64,
    amplitude_deg: f64,
    integrator: Integrator,
    model: sim_core::Model,
    state: State,
    clock: FixedClock,
    detector: CrossingDetector,
    e0: f64,
    history: History,
    epoch: u64,
}

impl Default for Pendulum {
    fn default() -> Self {
        let mut p = Self {
            length: 1.5,
            amplitude_deg: 30.0,
            integrator: Integrator::Rk4,
            model: simple_pendulum(1.0, 1.0, Vec3::zeros()),
            state: State {
                q: Default::default(),
                v: Default::default(),
                t: 0.0,
            },
            clock: FixedClock::new(DT),
            detector: CrossingDetector::new(),
            e0: 0.0,
            history: History::new(1, 12.0),
            epoch: 0,
        };
        p.reset();
        p
    }
}

impl Pendulum {
    fn energy(&self) -> f64 {
        kinetic_energy(&self.model, &self.state.q, &self.state.v)
            + potential_energy(&self.model, &self.state.q)
    }

    fn energy_drift(&self) -> f64 {
        (self.energy() - self.e0) / (G_STANDARD * self.length)
    }

    fn measured_period(&self) -> Option<f64> {
        self.detector.recent_period(3)
    }
}

impl Scenario for Pendulum {
    fn id(&self) -> &'static str {
        "pendulum"
    }

    fn title(&self) -> String {
        "Эталон: маятник".into()
    }

    fn summary(&self) -> String {
        "Точечная масса 1 кг на невесомом стержне. Период из симуляции сравнивается с точной формулой \
         (эллиптический интеграл) и с формулой малых углов 2π√(L/g). Смените интегратор на явный Эйлер — \
         увидите, как растёт энергия."
            .into()
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param {
                id: "length",
                label: "Длина L".into(),
                value: self.length,
                min: 0.2,
                max: 2.0,
                unit: "м",
                decimals: 2,
            },
            Param {
                id: "amp",
                label: "Начальный угол θ₀".into(),
                value: self.amplitude_deg,
                min: 1.0,
                max: 175.0,
                unit: "°",
                decimals: 0,
            },
        ]
    }

    fn set_param(&mut self, id: &str, value: f64) {
        match id {
            "length" => self.length = value,
            "amp" => self.amplitude_deg = value,
            _ => return,
        }
        self.reset();
    }

    fn presets(&self) -> Vec<String> {
        vec!["Малые углы 5°".into(), "Большие углы 120°".into()]
    }

    fn apply_preset(&mut self, index: usize) {
        self.amplitude_deg = if index == 0 { 5.0 } else { 120.0 };
        self.reset();
    }

    fn choices(&self) -> Vec<Choice> {
        vec![Choice {
            id: "integrator",
            label: "Интегратор".into(),
            options: Integrator::ALL
                .iter()
                .map(|i| i.label().to_string())
                .collect(),
            selected: Integrator::ALL
                .iter()
                .position(|i| *i == self.integrator)
                .unwrap_or(0),
        }]
    }

    fn set_choice(&mut self, id: &str, index: usize) {
        if id == "integrator" {
            self.integrator = Integrator::ALL[index.min(Integrator::ALL.len() - 1)];
            self.reset();
        }
    }

    fn reset(&mut self) {
        let pivot = Vec3::new(0.0, 0.0, PIVOT_Z);
        self.model = simple_pendulum(self.length, 1.0, pivot);
        // Стойка: одна опора за плоскостью качания и кронштейн до оси подвеса.
        let post = [0.55, 0.57, 0.6, 1.0];
        let back = 0.45;
        self.model.world_visuals.push(Visual::new(
            Shape::Box {
                half: Vec3::new(0.03, 0.03, (PIVOT_Z + 0.05) / 2.0),
            },
            Pose::at(Vec3::new(0.0, back, (PIVOT_Z + 0.05) / 2.0)),
            post,
        ));
        self.model.world_visuals.push(Visual::new(
            Shape::Box {
                half: Vec3::new(0.025, back / 2.0, 0.025),
            },
            Pose::at(Vec3::new(0.0, back / 2.0, PIVOT_Z)),
            post,
        ));
        self.state = State::rest(&self.model);
        self.state.q[0] = self.amplitude_deg.to_radians();
        self.e0 = self.energy();
        self.detector.clear();
        self.history.clear();
        self.clock.reset();
        self.epoch += 1;
    }

    fn advance(&mut self, dt_real: f64) {
        let inputs = Inputs::zero(&self.model);
        for _ in 0..self.clock.steps(dt_real) {
            step(&self.model, &mut self.state, &inputs, DT, self.integrator);
            self.detector
                .push(self.state.t, self.state.q[0], self.state.v[0]);
            self.history
                .push(self.state.t, &[self.state.q[0].to_degrees()]);
            if !self.state.q[0].is_finite() || self.state.q[0].abs() > 1e6 {
                self.reset();
                return;
            }
        }
    }

    fn time(&self) -> f64 {
        self.state.t
    }

    fn visuals(&self) -> Vec<VisualInstance> {
        model_visuals(&self.model, &self.state.q)
    }

    fn visual_epoch(&self) -> u64 {
        self.epoch
    }

    fn overlays(&self) -> Vec<Overlay> {
        let xw = world_to_body(&self.model, &self.state.q);
        let bob = xw[0].point_to_parent(&Vec3::new(0.0, 0.0, -self.length));
        let pivot = Vec3::new(0.0, 0.0, PIVOT_Z);
        let a = self.amplitude_deg.to_radians();
        let edge = |s: f64| pivot + Vec3::new(s * a.sin(), 0.0, -a.cos()) * self.length;
        vec![
            Overlay::Arrow {
                from: bob,
                to: bob - Vec3::new(0.0, 0.0, 0.25),
                color: C_RED,
            },
            Overlay::Line {
                from: pivot,
                to: edge(1.0),
                color: C_GRAY,
            },
            Overlay::Line {
                from: pivot,
                to: edge(-1.0),
                color: C_GRAY,
            },
        ]
    }

    fn readouts(&self) -> Vec<Readout> {
        vec![
            Readout::new("Время симуляции", self.state.t, "с", 2),
            Readout::new("Угол θ", self.state.q[0].to_degrees(), "°", 2),
            Readout::new(
                "Период (симуляция, 3 цикла)",
                self.measured_period().unwrap_or(f64::NAN),
                "с",
                6,
            ),
            Readout::new("Дрейф энергии", 100.0 * self.energy_drift(), "% m·g·L", 6),
            Readout::new("Шаг интегрирования", DT * 1000.0, "мс", 1),
        ]
    }

    fn comparisons(&self) -> Vec<Comparison> {
        let theta0 = self.amplitude_deg.to_radians();
        let exact = an::pendulum_period_exact(self.length, G_STANDARD, theta0);
        let small = an::pendulum_period_small(self.length, G_STANDARD);
        let drift = self.energy_drift();
        let euler_bad = self.integrator == Integrator::ExplicitEuler;
        let measured = self.measured_period();
        let t = measured.unwrap_or(f64::NAN);
        let wait = (
            Status::Pending,
            "ждём хотя бы один полный период".to_string(),
        );

        let c_exact = {
            let (status, explanation) = match measured {
                None => wait.clone(),
                Some(t) if ((t - exact) / exact).abs() <= 1e-4 => {
                    (Status::Ok, "совпало (допуск 1e-4)".into())
                }
                Some(_) if euler_bad => (
                    Status::Explained,
                    "явный Эйлер раскачивает маятник — амплитуда растёт, период уплывает".into(),
                ),
                Some(_) => (
                    Status::Mismatch,
                    "период не совпал с точным решением — ошибка в движке".into(),
                ),
            };
            Comparison {
                title: "Период: симуляция ↔ точная формула".into(),
                sim_label: "симуляция".into(),
                sim: t,
                ref_label: "4√(L/g)·K(sin θ₀/2)".into(),
                reference: exact,
                unit: "с",
                decimals: 5,
                status,
                explanation,
            }
        };
        let c_small = {
            let corr = 100.0 * (exact / small - 1.0);
            let (status, explanation) = match measured {
                None => wait.clone(),
                Some(t) if ((t - small) / small).abs() <= 5e-3 => (
                    Status::Ok,
                    format!("совпало в пределах 0.5% (поправка на амплитуду {corr:.3}%)"),
                ),
                Some(t) if ((t - exact) / exact).abs() <= 1e-4 => (
                    Status::Explained,
                    format!(
                        "формула малых углов не учитывает амплитуду: при θ₀ = {:.0}° период длиннее на {corr:.2}%",
                        self.amplitude_deg
                    ),
                ),
                Some(_) => (Status::Mismatch, "не совпало ни с одной формулой".into()),
            };
            Comparison {
                title: "Период: симуляция ↔ формула малых углов".into(),
                sim_label: "симуляция".into(),
                sim: t,
                ref_label: "2π√(L/g)".into(),
                reference: small,
                unit: "с",
                decimals: 5,
                status,
                explanation,
            }
        };
        let c_energy = {
            let (status, explanation) = if drift.abs() <= 1e-4 {
                (Status::Ok, "энергия сохраняется".into())
            } else if euler_bad {
                (
                    Status::Explained,
                    "явный Эйлер не сохраняет энергию — поэтому его не используют для механики"
                        .into(),
                )
            } else if self.integrator == Integrator::SemiImplicitEuler && drift.abs() <= 1e-2 {
                (
                    Status::Explained,
                    "симплектический Эйлер: энергия колеблется в пределах O(dt), но не уплывает"
                        .into(),
                )
            } else {
                (Status::Mismatch, "энергия уплывает — ошибка".into())
            };
            Comparison {
                title: "Энергия: дрейф ↔ 0".into(),
                sim_label: "(E − E₀)/(m·g·L)".into(),
                sim: drift,
                ref_label: "закон сохранения".into(),
                reference: 0.0,
                unit: "",
                decimals: 7,
                status,
                explanation,
            }
        };
        vec![c_exact, c_small, c_energy]
    }

    fn notes(&self) -> Vec<Note> {
        vec![Note::new(
            Level::Info,
            "Допущения: стержень невесом, подвес без трения, сопротивления воздуха нет.",
        )]
    }

    fn plot(&self) -> Option<Plot> {
        Some(Plot {
            title: "Угол маятника".into(),
            x_label: "время, с".into(),
            y_label: "°".into(),
            series: vec![Series {
                name: "θ(t), симуляция".into(),
                points: self.history.points(0, 10.0, 800),
                color: C_BLUE,
                style: SeriesStyle::Solid,
            }],
            bands: Vec::new(),
        })
    }

    fn focus(&self) -> Vec3 {
        Vec3::new(0.0, 0.0, PIVOT_Z - self.length * 0.5)
    }

    fn view_offset(&self) -> Vec3 {
        // Сбоку, вдоль оси качания: плоскость качания видна целиком, стойка — позади.
        Vec3::new(0.8, -5.4, 0.2)
    }
}
