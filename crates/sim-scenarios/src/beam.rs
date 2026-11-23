//! Сценарий «Штанга ортеза на изгиб»: боковая штанга предплечья (консоль от привода локтя)
//! под весом груза. Прогиб — МКЭ (балка Эйлера–Бернулли) и, вторым способом, многотельная
//! цепочка; напряжение — σ = M·c/I против предела текучести с запасом 2–4×.

use nalgebra::UnitQuaternion;
use sim_core::{G_STANDARD, Pose, Shape, Vec3};
use sim_fem::section::{AL_6061_T6, Material, STEEL_S235, rect_i};
use sim_fem::{BeamSolution, Cantilever, Loads, root_moment, solve_cantilever};
use sim_verify::analytic as an;
use sim_verify::cases::bar_chain_tip;

use crate::exo_arm::SAFETY_FACTOR;
use crate::scenario::*;

const MATERIALS: [Material; 2] = [AL_6061_T6, STEEL_S235];
const Z0: f64 = 1.1;
const ELEMENTS: usize = 24;
const CHAIN_N: usize = 40;

pub struct BeamStrut {
    load_kg: f64,
    length: f64,
    b_mm: f64,
    h_mm: f64,
    scale: f64,
    material: usize,
    t: f64,
    epoch: u64,
    solution: BeamSolution,
    chain_tip: f64,
}

impl Default for BeamStrut {
    fn default() -> Self {
        let mut s = Self {
            load_kg: 50.0,
            length: 0.35,
            b_mm: 20.0,
            h_mm: 10.0,
            scale: 10.0,
            material: 0,
            t: 0.0,
            epoch: 0,
            solution: BeamSolution {
                x: vec![],
                w: vec![],
                theta: vec![],
            },
            chain_tip: f64::NAN,
        };
        s.recompute();
        s
    }
}

impl BeamStrut {
    fn mat(&self) -> Material {
        MATERIALS[self.material]
    }
    fn i(&self) -> f64 {
        rect_i(self.b_mm / 1000.0, self.h_mm / 1000.0)
    }
    fn ei(&self) -> f64 {
        self.mat().e * self.i()
    }
    fn force(&self) -> f64 {
        self.load_kg * G_STANDARD
    }
    fn beam(&self) -> Cantilever {
        Cantilever {
            length: self.length,
            ei: self.ei(),
            elements: ELEMENTS,
        }
    }
    fn loads(&self) -> Loads {
        Loads {
            tip_force: self.force(),
            distributed: 0.0,
        }
    }
    fn stress(&self) -> f64 {
        an::bending_stress(
            root_moment(&self.beam(), &self.loads()),
            self.h_mm / 2000.0,
            self.i(),
        )
    }
    fn safety(&self) -> f64 {
        self.mat().yield_strength / self.stress().max(1e-9)
    }
    /// Статический запас по текучести σт/σ (усталость и концентраторы не учтены).
    pub fn static_safety(&self) -> f64 {
        self.safety()
    }
    /// Фактический масштаб показа: заданный, но так, чтобы прогиб на экране не превышал 15% длины.
    fn shown_scale(&self) -> f64 {
        let d = self.solution.tip_deflection().abs();
        if d <= 0.0 {
            self.scale
        } else {
            self.scale.min((0.15 * self.length / d).max(1.0))
        }
    }
    fn recompute(&mut self) {
        self.solution = solve_cantilever(&self.beam(), &self.loads());
        self.chain_tip = bar_chain_tip(CHAIN_N, self.force(), self.length, self.ei());
        self.epoch += 1;
    }
}

impl Scenario for BeamStrut {
    fn id(&self) -> &'static str {
        "beam"
    }

    fn title(&self) -> String {
        "Штанга ортеза на изгиб".into()
    }

    fn summary(&self) -> String {
        "Боковая штанга предплечья — консоль от привода локтя, на конце вес груза (худший случай: всю \
         нагрузку держит штанга). Прогиб: МКЭ и многотельная цепочка против формулы P·L³/(3EI); \
         напряжение σ = M·c/I против предела текучести с запасом 2–4×."
            .into()
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param {
                id: "load",
                label: "Груз на конце".into(),
                value: self.load_kg,
                min: 0.0,
                max: 60.0,
                unit: "кг",
                decimals: 1,
            },
            Param {
                id: "length",
                label: "Длина штанги".into(),
                value: self.length,
                min: 0.15,
                max: 0.5,
                unit: "м",
                decimals: 3,
            },
            Param {
                id: "b",
                label: "Ширина сечения".into(),
                value: self.b_mm,
                min: 3.0,
                max: 40.0,
                unit: "мм",
                decimals: 1,
            },
            Param {
                id: "h",
                label: "Высота сечения (в плоскости изгиба)".into(),
                value: self.h_mm,
                min: 3.0,
                max: 40.0,
                unit: "мм",
                decimals: 1,
            },
            Param {
                id: "scale",
                label: "Масштаб показа прогиба".into(),
                value: self.scale,
                min: 1.0,
                max: 50.0,
                unit: "×",
                decimals: 0,
            },
        ]
    }

    fn set_param(&mut self, id: &str, value: f64) {
        match id {
            "load" => self.load_kg = value,
            "length" => self.length = value,
            "b" => self.b_mm = value,
            "h" => self.h_mm = value,
            "scale" => {
                self.scale = value;
                self.epoch += 1;
                return;
            }
            _ => return,
        }
        self.recompute();
    }

    fn presets(&self) -> Vec<String> {
        vec![
            "Al 20×10 мм (тонко)".into(),
            "Al 12×30 мм (на ребро)".into(),
        ]
    }

    fn apply_preset(&mut self, index: usize) {
        self.material = 0;
        (self.b_mm, self.h_mm) = if index == 0 {
            (20.0, 10.0)
        } else {
            (12.0, 30.0)
        };
        self.recompute();
    }

    fn choices(&self) -> Vec<Choice> {
        vec![Choice {
            id: "mat",
            label: "Материал".into(),
            options: MATERIALS.iter().map(|m| m.name.to_string()).collect(),
            selected: self.material,
        }]
    }

    fn set_choice(&mut self, id: &str, index: usize) {
        if id == "mat" {
            self.material = index.min(MATERIALS.len() - 1);
            self.recompute();
        }
    }

    fn reset(&mut self) {
        self.t = 0.0;
        self.recompute();
    }

    fn advance(&mut self, dt_real: f64) {
        self.t += dt_real;
    }

    fn time(&self) -> f64 {
        self.t
    }

    fn visuals(&self) -> Vec<VisualInstance> {
        let s = &self.solution;
        let (b, h) = (self.b_mm / 1000.0, self.h_mm / 1000.0);
        let k = self.shown_scale();
        let node = |i: usize| Vec3::new(s.x[i], 0.0, Z0 - k * s.w[i]);
        let mut out = Vec::new();
        // Привод локтя / заделка.
        out.push(VisualInstance {
            shape: Shape::Cylinder {
                radius: 0.05,
                length: 0.06,
            },
            pose: Pose::z_along(Vec3::new(-0.02, 0.0, Z0), Vec3::y()),
            color: C_MOTOR,
        });
        out.push(VisualInstance {
            shape: Shape::Box {
                half: Vec3::new(0.04, 0.05, 0.4),
            },
            pose: Pose::at(Vec3::new(-0.08, 0.0, Z0 - 0.4)),
            color: C_EXO,
        });
        for i in 0..s.x.len() - 1 {
            let (a, c) = (node(i), node(i + 1));
            let d = c - a;
            let rot = UnitQuaternion::rotation_between(&Vec3::x(), &d)
                .unwrap_or_else(UnitQuaternion::identity);
            out.push(VisualInstance {
                shape: Shape::Box {
                    half: Vec3::new(d.norm() / 2.0 + 0.0005, b / 2.0, h / 2.0),
                },
                pose: Pose::new(rot, (a + c) / 2.0),
                color: [0.78, 0.8, 0.84, 1.0],
            });
        }
        if self.load_kg > 0.0 {
            let tip = node(s.x.len() - 1);
            let side = (self.load_kg / 7200.0).cbrt();
            out.push(VisualInstance {
                shape: Shape::Box {
                    half: Vec3::new(side / 2.0, side / 2.0, side / 2.0),
                },
                pose: Pose::at(tip - Vec3::new(0.0, 0.0, 0.12 + side / 2.0)),
                color: C_IRON,
            });
        }
        out
    }

    fn visual_epoch(&self) -> u64 {
        self.epoch
    }

    fn overlays(&self) -> Vec<Overlay> {
        let s = &self.solution;
        let n = s.x.len() - 1;
        let k = self.shown_scale();
        let tip = Vec3::new(s.x[n], 0.0, Z0 - k * s.w[n]);
        let mut out = vec![
            Overlay::Line {
                from: Vec3::new(0.0, 0.0, Z0),
                to: Vec3::new(self.length, 0.0, Z0),
                color: C_GRAY,
            },
            Overlay::Line {
                from: tip,
                to: tip - Vec3::new(0.0, 0.0, 0.12),
                color: C_STEEL,
            },
        ];
        // Форма по формуле — пунктиром (в том же масштабе).
        let p = self.force();
        let ei = self.ei();
        for j in 0..30 {
            let x0 = self.length * j as f64 / 30.0;
            let x1 = self.length * (j + 1) as f64 / 30.0;
            if j % 2 == 0 {
                out.push(Overlay::Line {
                    from: Vec3::new(
                        x0,
                        0.03,
                        Z0 - k * an::cantilever_deflection_point(x0, p, self.length, ei),
                    ),
                    to: Vec3::new(
                        x1,
                        0.03,
                        Z0 - k * an::cantilever_deflection_point(x1, p, self.length, ei),
                    ),
                    color: C_YELLOW,
                });
            }
        }
        if self.load_kg > 0.0 {
            out.push(Overlay::Arrow {
                from: tip + Vec3::new(0.08, 0.0, 0.0),
                to: tip + Vec3::new(0.08, 0.0, -p * 0.0008),
                color: C_RED,
            });
        }
        out
    }

    fn readouts(&self) -> Vec<Readout> {
        let m = self.mat();
        vec![
            Readout::new("Сила на конце P", self.force(), "Н", 1),
            Readout::new(
                "Момент в заделке M = P·L",
                root_moment(&self.beam(), &self.loads()),
                "Н·м",
                1,
            ),
            Readout::new("Момент инерции сечения I", self.i() * 1e12, "мм⁴", 0),
            Readout::new(
                "Прогиб конца (МКЭ)",
                self.solution.tip_deflection() * 1000.0,
                "мм",
                3,
            ),
            Readout::new("Напряжение σ = M·c/I", self.stress() / 1e6, "МПа", 1),
            Readout::new("Предел текучести σт", m.yield_strength / 1e6, "МПа", 0),
            Readout::new("Запас по текучести σт/σ", self.safety(), "×", 2),
            Readout::new("Прогиб показан с увеличением", self.shown_scale(), "×", 1),
            Readout::new(
                "Масса штанги",
                m.density * self.b_mm * self.h_mm * 1e-6 * self.length * 1000.0,
                "г",
                0,
            ),
        ]
    }

    fn comparisons(&self) -> Vec<Comparison> {
        let p = self.force();
        let ei = self.ei();
        let formula = an::cantilever_tip_point(p, self.length, ei);
        let fem = self.solution.tip_deflection();
        let fem_status = if formula == 0.0 || ((fem - formula) / formula).abs() <= 1e-9 {
            (
                Status::Ok,
                "МКЭ на эрмитовых элементах точен в узлах для этой задачи".to_string(),
            )
        } else {
            (
                Status::Mismatch,
                "МКЭ разошёлся с формулой — ошибка в сборке/решении".to_string(),
            )
        };
        let ratio = formula / self.length;
        let elastica = an::elastica_tip_deflection(p, self.length, ei);
        let expected = 1.0 / (4.0 * (CHAIN_N * CHAIN_N) as f64);
        let chain_status = if self.chain_tip.is_nan() {
            (
                Status::Explained,
                "прогиб слишком большой: итерации равновесия не сошлись — нужна другая схема"
                    .to_string(),
            )
        } else {
            let rel = if formula > 0.0 {
                ((self.chain_tip - formula) / formula).abs()
            } else {
                self.chain_tip.abs()
            };
            let rel_el = if elastica > 0.0 {
                ((self.chain_tip - elastica) / elastica).abs()
            } else {
                self.chain_tip.abs()
            };
            if rel <= expected + 1e-3 {
                (
                    Status::Ok,
                    format!("второй способ совпал (ожидаемая ошибка дискретизации {expected:.1e})"),
                )
            } else if rel_el <= expected + 1e-3 {
                (
                    Status::Explained,
                    format!(
                        "прогиб {:.1}% длины: линейная формула не учитывает большие повороты; с эластикой цепочка совпала",
                        100.0 * ratio
                    ),
                )
            } else {
                (
                    Status::Mismatch,
                    "цепочка разошлась и с формулой, и с эластикой — ошибка".to_string(),
                )
            }
        };
        let el_status = if self.chain_tip.is_nan() {
            (Status::Pending, "нет решения цепочки".to_string())
        } else if elastica == 0.0
            || ((self.chain_tip - elastica) / elastica).abs() <= expected + 1e-3
        {
            (
                Status::Ok,
                "нелинейная теория (большие повороты) совпала с многотельной цепочкой".to_string(),
            )
        } else {
            (
                Status::Mismatch,
                "цепочка разошлась с эластикой — ошибка".to_string(),
            )
        };
        vec![
            Comparison {
                title: "Прогиб конца: МКЭ ↔ P·L³/(3EI)".into(),
                sim_label: "МКЭ, 24 элемента".into(),
                sim: fem * 1000.0,
                ref_label: "P·L³/(3EI)".into(),
                reference: formula * 1000.0,
                unit: "мм",
                decimals: 4,
                status: fem_status.0,
                explanation: fem_status.1,
            },
            Comparison {
                title: "Прогиб конца: многотельная цепочка (40 звеньев) ↔ P·L³/(3EI)".into(),
                sim_label: "цепочка + RNEA".into(),
                sim: self.chain_tip * 1000.0,
                ref_label: "P·L³/(3EI)".into(),
                reference: formula * 1000.0,
                unit: "мм",
                decimals: 4,
                status: chain_status.0,
                explanation: chain_status.1,
            },
            Comparison {
                title: "Прогиб конца: цепочка ↔ эластика Эйлера (большие прогибы)".into(),
                sim_label: "цепочка + RNEA".into(),
                sim: self.chain_tip * 1000.0,
                ref_label: "EI·θ'' = −P·cos θ".into(),
                reference: elastica * 1000.0,
                unit: "мм",
                decimals: 4,
                status: el_status.0,
                explanation: el_status.1,
            },
        ]
    }

    fn notes(&self) -> Vec<Note> {
        let n = self.safety();
        let (lo, _) = SAFETY_FACTOR;
        let mut out = Vec::new();
        if n < 1.0 {
            out.push(Note::new(Level::Danger, format!("σ = {:.0} МПа больше предела текучести: штанга погнётся навсегда (запас {n:.2}×).", self.stress() / 1e6)));
        } else if n < lo {
            out.push(Note::new(
                Level::Warn,
                format!(
                    "Запас {n:.2}× — меньше {lo:.0}×: для изделия на теле мало, увеличить сечение."
                ),
            ));
        } else {
            // Усталость и концентраторы здесь не считаются, поэтому зелёного вывода быть не может.
            out.push(Note::new(
                Level::Warn,
                format!(
                    "Статический запас {n:.2}× (нужно от {lo:.0}×), но это ещё не «годно»: усталость от циклов и концентраторы у отверстий не посчитаны. Облегчать по этому числу нельзя."
                ),
            ));
        }
        let ratio = self.solution.tip_deflection() / self.length;
        if ratio > 0.01 {
            out.push(Note::new(Level::Warn, format!("Конец уходит на {:.1} мм ({:.1}% длины) — человек почувствует «мягкость» ортеза.", self.solution.tip_deflection() * 1000.0, ratio * 100.0)));
        }
        out.push(Note::new(
            Level::Info,
            format!(
                "Допущения: худший случай — весь груз держит штанга; сечение прямоугольное, материал {} линейно-упругий; усталость, концентраторы у отверстий и сварка не учтены (дальше — CalculiX).",
                self.mat().name
            ),
        ));
        out
    }

    fn focus(&self) -> Vec3 {
        Vec3::new(self.length / 2.0, 0.0, Z0)
    }
}
