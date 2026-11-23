//! Сценарий «Свободное падение»: шар падает на пол, рядом «призрак» — положение по формуле.
//! Если призрак и шар разъехались — это видно глазом, а не только в цифрах.

use sim_core::{G_STANDARD, Inputs, Integrator, Pose, Shape, State, Vec3, Visual, step};
use sim_verify::analytic as an;
use sim_verify::cases::free_ball;
use sim_verify::measure::hermite_level_crossing;

use crate::scenario::*;

const DT: f64 = 0.001;
const R: f64 = 0.1;
const X: f64 = 0.0;

pub struct FreeFall {
    height: f64,
    integrator: Integrator,
    model: sim_core::Model,
    state: State,
    clock: FixedClock,
    /// Время с отпускания текущего броска.
    t_drop: f64,
    landed: Option<(f64, f64)>,
    hold: f64,
    history: History,
    epoch: u64,
}

impl Default for FreeFall {
    fn default() -> Self {
        let mut s = Self {
            height: 3.0,
            integrator: Integrator::Rk4,
            model: free_ball(R, 5.0),
            state: State {
                q: Default::default(),
                v: Default::default(),
                t: 0.0,
            },
            clock: FixedClock::new(DT),
            t_drop: 0.0,
            landed: None,
            hold: 0.0,
            history: History::new(2, 6.0),
            epoch: 0,
        };
        s.reset();
        s
    }
}

impl FreeFall {
    fn drop_again(&mut self) {
        self.state = State::rest(&self.model);
        self.state.q[0] = X;
        self.state.q[2] = self.height + R;
        self.t_drop = 0.0;
        self.landed = None;
        self.hold = 0.0;
        self.history.clear();
    }
}

impl Scenario for FreeFall {
    fn id(&self) -> &'static str {
        "free_fall"
    }

    fn title(&self) -> String {
        "Эталон: свободное падение".into()
    }

    fn summary(&self) -> String {
        "Шар 5 кг (свободное тело, 6 степеней свободы) падает с высоты h. Каркасная сфера — положение \
         по формуле z = h − g·t²/2. Время и скорость удара сверяются с √(2h/g) и √(2gh)."
            .into()
    }

    fn params(&self) -> Vec<Param> {
        vec![Param {
            id: "h",
            label: "Высота h".into(),
            value: self.height,
            min: 0.5,
            max: 10.0,
            unit: "м",
            decimals: 2,
        }]
    }

    fn set_param(&mut self, id: &str, value: f64) {
        if id == "h" {
            self.height = value;
            self.reset();
        }
    }

    fn choices(&self) -> Vec<Choice> {
        let opts = [Integrator::Rk4, Integrator::SemiImplicitEuler];
        vec![Choice {
            id: "integrator",
            label: "Интегратор".into(),
            options: opts.iter().map(|i| i.label().to_string()).collect(),
            selected: if self.integrator == Integrator::Rk4 {
                0
            } else {
                1
            },
        }]
    }

    fn set_choice(&mut self, id: &str, index: usize) {
        if id == "integrator" {
            self.integrator = if index == 0 {
                Integrator::Rk4
            } else {
                Integrator::SemiImplicitEuler
            };
            self.reset();
        }
    }

    fn reset(&mut self) {
        self.model = free_ball(R, 5.0);
        // Рейка с отметками высоты.
        let h = self.height.max(1.0) + 0.3;
        self.model.world_visuals.push(Visual::new(
            Shape::Box {
                half: Vec3::new(0.02, 0.02, h / 2.0),
            },
            Pose::at(Vec3::new(X - 0.35, 0.0, h / 2.0)),
            [0.85, 0.85, 0.8, 1.0],
        ));
        self.clock.reset();
        self.drop_again();
        self.epoch += 1;
    }

    fn advance(&mut self, dt_real: f64) {
        let inputs = Inputs::zero(&self.model);
        for _ in 0..self.clock.steps(dt_real) {
            if self.landed.is_some() {
                self.hold += DT;
                if self.hold > 1.2 {
                    self.drop_again();
                }
                continue;
            }
            let (z0, v0, t0) = (self.state.q[2], self.state.v[5], self.t_drop);
            step(&self.model, &mut self.state, &inputs, DT, self.integrator);
            self.t_drop += DT;
            let (z1, v1) = (self.state.q[2], self.state.v[5]);
            self.history.push(
                self.t_drop,
                &[
                    z1 - R,
                    an::free_fall_z(self.height, 0.0, G_STANDARD, self.t_drop),
                ],
            );
            if z1 <= R {
                // Уточняем момент касания пола внутри шага эрмитовым интерполянтом по z и v.
                let ti = hermite_level_crossing(t0, z0, v0, self.t_drop, z1, v1, R);
                let s = (ti - t0) / DT;
                let vi = v0 + (v1 - v0) * s;
                self.landed = Some((ti, vi.abs()));
                self.state.q[2] = R;
                self.state.v.fill(0.0);
            }
        }
    }

    fn time(&self) -> f64 {
        self.t_drop
    }

    fn visuals(&self) -> Vec<VisualInstance> {
        model_visuals(&self.model, &self.state.q)
    }

    fn visual_epoch(&self) -> u64 {
        self.epoch
    }

    fn overlays(&self) -> Vec<Overlay> {
        let mut out = Vec::new();
        let t = self.landed.map(|(ti, _)| ti).unwrap_or(self.t_drop);
        let z = an::free_fall_z(self.height, 0.0, G_STANDARD, t).max(0.0) + R;
        out.push(Overlay::WireSphere {
            center: Vec3::new(X, 0.0, z),
            radius: R * 1.08,
            color: C_YELLOW,
        });
        let mut k = 0.0;
        while k <= self.height + 1e-9 {
            let major = (k * 2.0).round() as i64 % 2 == 0;
            let w = if major { 0.12 } else { 0.07 };
            out.push(Overlay::Line {
                from: Vec3::new(X - 0.35, 0.0, k),
                to: Vec3::new(X - 0.35 + w, 0.0, k),
                color: C_GRAY,
            });
            k += 0.5;
        }
        out.push(Overlay::Arrow {
            from: Vec3::new(X + 0.25, 0.0, self.state.q[2]),
            to: Vec3::new(X + 0.25, 0.0, self.state.q[2] - 0.3),
            color: C_RED,
        });
        out
    }

    fn readouts(&self) -> Vec<Readout> {
        vec![
            Readout::new("Время с отпускания", self.t_drop, "с", 3),
            Readout::new("Высота низа шара", self.state.q[2] - R, "м", 4),
            Readout::new("Скорость", self.state.v[5].abs(), "м/с", 4),
            Readout::new("Шаг интегрирования", DT * 1000.0, "мс", 1),
        ]
    }

    fn comparisons(&self) -> Vec<Comparison> {
        let t_ref = an::free_fall_time(self.height, G_STANDARD);
        let v_ref = an::free_fall_impact_speed(self.height, G_STANDARD);
        let semi = self.integrator == Integrator::SemiImplicitEuler;
        let judge = |sim: f64, reference: f64| -> (Status, String) {
            let rel = ((sim - reference) / reference).abs();
            if rel <= 1e-6 {
                (Status::Ok, format!("совпало, отн. ошибка {rel:.1e}"))
            } else if semi && rel <= 1e-2 {
                (
                    Status::Explained,
                    format!("полунеявный Эйлер: ошибка 1-го порядка O(dt), {rel:.1e} — ожидаемо"),
                )
            } else {
                (
                    Status::Mismatch,
                    format!("отн. ошибка {rel:.1e} — больше ожидаемой"),
                )
            }
        };
        match self.landed {
            None => vec![Comparison {
                title: "Время падения: симуляция ↔ √(2h/g)".into(),
                sim_label: "симуляция".into(),
                sim: f64::NAN,
                ref_label: "√(2h/g)".into(),
                reference: t_ref,
                unit: "с",
                decimals: 6,
                status: Status::Pending,
                explanation: "шар ещё летит".into(),
            }],
            Some((ti, vi)) => {
                let (st, ex) = judge(ti, t_ref);
                let (sv, exv) = judge(vi, v_ref);
                vec![
                    Comparison {
                        title: "Время падения: симуляция ↔ √(2h/g)".into(),
                        sim_label: "симуляция".into(),
                        sim: ti,
                        ref_label: "√(2h/g)".into(),
                        reference: t_ref,
                        unit: "с",
                        decimals: 6,
                        status: st,
                        explanation: ex,
                    },
                    Comparison {
                        title: "Скорость удара: симуляция ↔ √(2gh)".into(),
                        sim_label: "симуляция".into(),
                        sim: vi,
                        ref_label: "√(2gh)".into(),
                        reference: v_ref,
                        unit: "м/с",
                        decimals: 6,
                        status: sv,
                        explanation: exv,
                    },
                ]
            }
        }
    }

    fn notes(&self) -> Vec<Note> {
        vec![Note::new(
            Level::Info,
            "Допущения: сопротивления воздуха нет, удар о пол не моделируется (контакт — следующий этап, после сверки с MuJoCo).",
        )]
    }

    fn plot(&self) -> Option<Plot> {
        Some(Plot {
            title: "Высота шара".into(),
            x_label: "время с отпускания, с".into(),
            y_label: "м".into(),
            series: vec![
                Series {
                    name: "симуляция".into(),
                    points: self.history.points(0, 6.0, 600),
                    color: C_BLUE,
                    style: SeriesStyle::Solid,
                },
                Series {
                    name: "h − g·t²/2".into(),
                    points: self.history.points(1, 6.0, 600),
                    color: C_YELLOW,
                    style: SeriesStyle::Dashed,
                },
            ],
            bands: Vec::new(),
        })
    }

    fn focus(&self) -> Vec3 {
        Vec3::new(X, 0.0, (self.height / 2.0 + 0.1).min(2.0))
    }

    fn view_offset(&self) -> Vec3 {
        let d = (self.height * 1.25).clamp(2.5, 9.0);
        Vec3::new(0.6, -d, 0.0)
    }
}
