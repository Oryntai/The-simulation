//! Сценарий «Магниты и электромагнит»: постоянные магниты (сила между ними, к стали),
//! электромагнит с насыщением стали и магнит датчика угла локтя (AS5600). Поля и силы —
//! `sim-em`; каждое число сверено вторым независимым способом (см. панель справа).
//! На экране всё увеличено в 10 раз (лупа): реальные размеры — миллиметры.

use std::f64::consts::PI;

use sim_core::{Pose, Shape, Vec3};
use sim_em::field::{FieldScene, FieldSource, TraceOptions, trace_field_line};
use sim_em::magcircuit::{BhCurve, Coil, CoreMaterial, MagneticCircuit, PoleFace, awg_diameter_m};
use sim_em::magnet::{
    AS5600, AxialCylinderMagnet, CoaxialPair, DiametricDiscMagnet, KJ_D88_N52_PULL_N, NDFEB_GRADES,
    br_at_temperature, coaxial_force_bessel, coaxial_force_loops, contact_pull_upper_bound,
    dipole_dipole_coaxial_force, pull_force_ideal_steel, sensor_bz_amplitude, sensor_gap_window,
    temp_class_of,
};

use crate::scenario::*;

/// Увеличение на экране.
const S: f64 = 10.0;
/// Где стоит «стол» с опытом (координаты движка).
const STAGE: Vec3 = Vec3::new(0.0, 0.0, 0.9);

const MODES: [&str; 4] = [
    "Два магнита: сила между ними",
    "Магнит на стальной плите",
    "Электромагнит с сердечником",
    "Магнит датчика угла локтя (AS5600)",
];

const C_NORTH: [f32; 4] = [0.85, 0.2, 0.2, 1.0];
const C_SOUTH: [f32; 4] = [0.2, 0.35, 0.85, 1.0];
const C_COPPER: [f32; 4] = [0.85, 0.5, 0.2, 1.0];
const C_LINE: [f32; 4] = [0.55, 0.8, 1.0, 0.9];

pub struct Magnets {
    mode: usize,
    grade: usize,
    repel: bool,
    diameter_mm: f64,
    length_mm: f64,
    gap_mm: f64,
    temp_c: f64,
    // электромагнит
    current_a: f64,
    turns: f64,
    pole_mm: f64,
    ideal_core: bool,
    awg: i32,
    // датчик угла
    disc_d_mm: f64,
    disc_t_mm: f64,
    lines: Vec<Vec<Vec3>>,
    plot_cache: Option<Plot>,
    cmp_cache: Vec<Comparison>,
    epoch: u64,
    t: f64,
}

impl Default for Magnets {
    fn default() -> Self {
        let mut s = Self {
            mode: 0,
            grade: 3,
            repel: false,
            diameter_mm: 10.0,
            length_mm: 10.0,
            gap_mm: 2.0,
            temp_c: 20.0,
            current_a: 1.0,
            turns: 400.0,
            pole_mm: 20.0,
            ideal_core: false,
            awg: 22,
            disc_d_mm: 6.0,
            disc_t_mm: 2.5,
            lines: Vec::new(),
            plot_cache: None,
            cmp_cache: Vec::new(),
            epoch: 0,
            t: 0.0,
        };
        s.recompute();
        s
    }
}

/// Перевод точки опыта (м, своя система) в координаты экрана.
fn show(p: Vec3) -> Vec3 {
    STAGE + p * S
}

impl Magnets {
    fn br(&self) -> f64 {
        br_at_temperature(NDFEB_GRADES[self.grade].br_typical_t(), self.temp_c)
    }

    fn r(&self) -> f64 {
        self.diameter_mm / 2000.0
    }

    fn l(&self) -> f64 {
        self.length_mm / 1000.0
    }

    fn g(&self) -> f64 {
        self.gap_mm / 1000.0
    }

    fn pair(&self, gap: f64) -> CoaxialPair {
        let br = self.br();
        CoaxialPair {
            br1: br,
            r1: self.r(),
            l1: self.l(),
            br2: if self.repel { -br } else { br },
            r2: self.r(),
            l2: self.l(),
            gap,
        }
    }

    /// Магниты опыта в своей системе (м): ось z вверх, начало — середина зазора/поверхность.
    fn magnets(&self) -> Vec<AxialCylinderMagnet> {
        let (r, l, g, br) = (self.r(), self.l(), self.g(), self.br());
        match self.mode {
            0 => vec![
                AxialCylinderMagnet::new(Vec3::new(0.0, 0.0, -(g + l) / 2.0), Vec3::z(), r, l, br),
                AxialCylinderMagnet::new(
                    Vec3::new(0.0, 0.0, (g + l) / 2.0),
                    Vec3::z(),
                    r,
                    l,
                    if self.repel { -br } else { br },
                ),
            ],
            1 => vec![AxialCylinderMagnet::new(
                Vec3::new(0.0, 0.0, g + l / 2.0),
                Vec3::z(),
                r,
                l,
                br,
            )],
            _ => vec![],
        }
    }

    /// Источники поля опыта: магниты и (на стали) их зеркальное изображение.
    fn field_sources(&self) -> Vec<AxialCylinderMagnet> {
        let mut v = self.magnets();
        if self.mode == 1 {
            let (r, l, g) = (self.r(), self.l(), self.g());
            v.push(AxialCylinderMagnet::new(
                Vec3::new(0.0, 0.0, -(g + l / 2.0)),
                Vec3::z(),
                r,
                l,
                self.br(),
            ));
        }
        v
    }

    fn disc(&self) -> DiametricDiscMagnet {
        DiametricDiscMagnet::new(
            Vec3::new(0.0, 0.0, self.disc_t_mm / 2000.0),
            Vec3::z(),
            Vec3::x(),
            self.disc_d_mm / 2000.0,
            self.disc_t_mm / 1000.0,
            self.br(),
        )
    }

    /// Электромагнит: U-сердечник из стали с якорем, два зазора.
    fn circuit(&self) -> MagneticCircuit {
        let a = self.pole_mm / 1000.0;
        MagneticCircuit {
            material: if self.ideal_core {
                CoreMaterial::Ideal
            } else {
                CoreMaterial::Nonlinear(BhCurve::aisi_1010())
            },
            // Средний путь: две ноги (2·2a) + ярмо и якорь (по 3a).
            core_path_m: 10.0 * a,
            core_area_m2: a * a,
            pole: PoleFace::Rect { a, b: a },
            gap_m: self.g(),
            n_gaps: 2,
            turns: self.turns,
            fringing: false,
        }
    }

    fn coil(&self) -> Coil {
        let a = self.pole_mm / 1000.0;
        // Обмотка на двух ногах; толщина слоя — по числу витков (заполнение 0.6, оценка).
        let d = awg_diameter_m(self.awg) * 1.08;
        let window_h = 1.6 * a;
        let thickness = (self.turns / 2.0) * d * d / (0.6 * window_h);
        let mean_turn = 4.0 * (a + thickness);
        Coil {
            turns: self.turns,
            awg: self.awg,
            mean_turn_length_m: mean_turn,
            cooling_area_m2: 2.0 * 4.0 * (a + 2.0 * thickness) * window_h,
            h_w_per_m2k: 12.0,
        }
    }

    fn recompute(&mut self) {
        self.epoch += 1;
        self.lines.clear();
        let mut seeds = Vec::new();
        let mut field = FieldScene::new();
        let mut cut_below_zero = false;
        let scene_r = match self.mode {
            0 | 1 => {
                for m in self.magnets() {
                    for k in 0..5 {
                        let x = m.radius * (-0.8 + 0.4 * k as f64);
                        seeds.push(m.center + Vec3::new(x, 0.0, 0.0));
                    }
                    field.push(m);
                }
                if self.mode == 1 {
                    // Идеальная сталь: поле над плитой = магнит + зеркальное изображение.
                    let (r, l, g) = (self.r(), self.l(), self.g());
                    field.push(AxialCylinderMagnet::new(
                        Vec3::new(0.0, 0.0, -(g + l / 2.0)),
                        Vec3::z(),
                        r,
                        l,
                        self.br(),
                    ));
                    cut_below_zero = true;
                }
                4.0 * (self.r().max(self.l())) + self.g()
            }
            3 => {
                let m = self.disc();
                for k in 0..7 {
                    let y = m.radius * (-0.75 + 0.25 * k as f64);
                    seeds.push(m.center + Vec3::new(0.0, y, 0.0));
                }
                field.push(m);
                4.0 * m.radius
            }
            _ => 0.0,
        };
        if !seeds.is_empty() {
            let opt = TraceOptions {
                step: scene_r / 150.0,
                max_steps: 900,
                bounds_center: Vec3::zeros(),
                bounds_radius: scene_r,
                min_field: 1e-6,
                both_directions: true,
            };
            for s in seeds {
                let line = trace_field_line(&field, s, &opt);
                if cut_below_zero {
                    // Разрезаем на куски выше поверхности плиты.
                    let mut cur = Vec::new();
                    for p in line {
                        if p.z >= 0.0 {
                            cur.push(p);
                        } else if cur.len() > 1 {
                            self.lines.push(std::mem::take(&mut cur));
                        } else {
                            cur.clear();
                        }
                    }
                    if cur.len() > 1 {
                        self.lines.push(cur);
                    }
                } else {
                    self.lines.push(line);
                }
            }
        }
        self.refresh_caches();
    }

    fn refresh_caches(&mut self) {
        self.plot_cache = self.compute_plot();
        self.cmp_cache = self.compute_comparisons();
    }

    fn force_now(&self) -> f64 {
        match self.mode {
            0 => -coaxial_force_loops(&self.pair(self.g())),
            1 => pull_force_ideal_steel(self.br(), self.r(), self.l(), self.g()),
            2 => self.circuit().force_maxwell_n(self.current_a),
            _ => 0.0,
        }
    }
}

impl Scenario for Magnets {
    fn id(&self) -> &'static str {
        "magnets"
    }

    fn title(&self) -> String {
        "Магниты и электромагнит".into()
    }

    fn summary(&self) -> String {
        "Сила между магнитами, притяжение к стали, электромагнит с насыщением сердечника и \
         магнит датчика угла локтя. Поля — Био–Савар и замкнутые формулы, силы — двумя \
         независимыми методами. На экране — лупа 10:1."
            .into()
    }

    fn params(&self) -> Vec<Param> {
        let p = |id, label: &str, value, min, max, unit, decimals| Param {
            id,
            label: label.to_string(),
            value,
            min,
            max,
            unit,
            decimals,
        };
        match self.mode {
            0 | 1 => vec![
                p("d", "Диаметр магнита", self.diameter_mm, 2.0, 40.0, "мм", 1),
                p("l", "Длина магнита", self.length_mm, 1.0, 40.0, "мм", 1),
                p("gap", "Зазор", self.gap_mm, 0.05, 40.0, "мм", 2),
                p(
                    "temp",
                    "Температура магнита",
                    self.temp_c,
                    -40.0,
                    150.0,
                    "°C",
                    0,
                ),
            ],
            2 => vec![
                p("i", "Ток катушки", self.current_a, 0.0, 10.0, "А", 2),
                p("turns", "Витков (всего)", self.turns, 50.0, 2000.0, "", 0),
                p(
                    "gap",
                    "Зазор до якоря (каждый)",
                    self.gap_mm,
                    0.05,
                    10.0,
                    "мм",
                    2,
                ),
                p("pole", "Сторона полюса", self.pole_mm, 5.0, 50.0, "мм", 0),
            ],
            _ => vec![
                p("dd", "Диаметр диска", self.disc_d_mm, 3.0, 12.0, "мм", 1),
                p("dt", "Толщина диска", self.disc_t_mm, 1.0, 6.0, "мм", 1),
                p("gap", "Зазор до датчика", self.gap_mm, 0.1, 6.0, "мм", 2),
                p(
                    "temp",
                    "Температура магнита",
                    self.temp_c,
                    -40.0,
                    150.0,
                    "°C",
                    0,
                ),
            ],
        }
    }

    fn set_param(&mut self, id: &str, value: f64) {
        match id {
            "d" => self.diameter_mm = value,
            "l" => self.length_mm = value,
            "gap" => self.gap_mm = value,
            "temp" => self.temp_c = value,
            "i" => self.current_a = value,
            "turns" => self.turns = value,
            "pole" => self.pole_mm = value,
            "dd" => self.disc_d_mm = value,
            "dt" => self.disc_t_mm = value,
            _ => return,
        }
        self.recompute();
    }

    fn choices(&self) -> Vec<Choice> {
        let mut v = vec![Choice {
            id: "mode",
            label: "Опыт".into(),
            options: MODES.iter().map(|m| m.to_string()).collect(),
            selected: self.mode,
        }];
        if self.mode != 2 {
            v.push(Choice {
                id: "grade",
                label: "Марка NdFeB".into(),
                options: NDFEB_GRADES
                    .iter()
                    .map(|g| format!("{} (Br {:.2} Тл)", g.name, g.br_typical_t()))
                    .collect(),
                selected: self.grade,
            });
        }
        if self.mode == 0 {
            v.push(Choice {
                id: "orient",
                label: "Ориентация".into(),
                options: vec![
                    "N к S — притягиваются".into(),
                    "N к N — отталкиваются".into(),
                ],
                selected: self.repel as usize,
            });
        }
        if self.mode == 2 {
            v.push(Choice {
                id: "core",
                label: "Сердечник".into(),
                options: vec![
                    "сталь AISI 1010 (кривая B–H, насыщение)".into(),
                    "идеальная сталь μ → ∞ (учебник)".into(),
                ],
                selected: self.ideal_core as usize,
            });
            v.push(Choice {
                id: "awg",
                label: "Провод".into(),
                options: [18, 20, 22, 24, 26]
                    .iter()
                    .map(|a| format!("AWG {a} (Ø {:.2} мм)", awg_diameter_m(*a) * 1000.0))
                    .collect(),
                selected: [18, 20, 22, 24, 26]
                    .iter()
                    .position(|a| *a == self.awg)
                    .unwrap_or(2),
            });
        }
        v
    }

    fn set_choice(&mut self, id: &str, index: usize) {
        match id {
            "mode" => {
                self.mode = index.min(MODES.len() - 1);
                self.gap_mm = match self.mode {
                    0 => 2.0,
                    1 => 0.5,
                    2 => 0.5,
                    _ => 1.5,
                };
            }
            "grade" => self.grade = index.min(NDFEB_GRADES.len() - 1),
            "orient" => self.repel = index == 1,
            "core" => self.ideal_core = index == 1,
            "awg" => self.awg = [18, 20, 22, 24, 26][index.min(4)],
            _ => return,
        }
        self.recompute();
    }

    fn presets(&self) -> Vec<String> {
        vec![
            "K&J D88-N52 на стали".into(),
            "Защёлка манжеты: Ø10×5 N42, зазор 1 мм".into(),
            "Электромагнит: 1 А → 5 А (насыщение)".into(),
            "AS5600: Ø6×2.5 N35".into(),
        ]
    }

    fn apply_preset(&mut self, index: usize) {
        match index {
            0 => {
                self.mode = 1;
                self.grade = 7;
                self.diameter_mm = 12.7;
                self.length_mm = 12.7;
                self.gap_mm = 0.05;
                self.temp_c = 20.0;
            }
            1 => {
                self.mode = 0;
                self.grade = 3;
                self.diameter_mm = 10.0;
                self.length_mm = 5.0;
                self.gap_mm = 1.0;
                self.repel = false;
            }
            2 => {
                self.mode = 2;
                self.current_a = 5.0;
                self.turns = 400.0;
                self.gap_mm = 0.2;
                self.pole_mm = 20.0;
                self.ideal_core = false;
            }
            _ => {
                self.mode = 3;
                self.grade = 0;
                self.disc_d_mm = 6.0;
                self.disc_t_mm = 2.5;
                self.gap_mm = 1.5;
                self.temp_c = 20.0;
            }
        }
        self.recompute();
    }

    fn reset(&mut self) {
        self.t = 0.0;
    }

    fn advance(&mut self, dt_real: f64) {
        self.t += dt_real;
    }

    fn time(&self) -> f64 {
        self.t
    }

    fn visuals(&self) -> Vec<VisualInstance> {
        let mut out = Vec::new();
        if self.mode != 2 {
            // Подставка под опыт.
            out.push(VisualInstance {
                shape: Shape::Cylinder {
                    radius: 0.04,
                    length: STAGE.z - 0.25,
                },
                pose: Pose::at(Vec3::new(0.0, 0.0, (STAGE.z - 0.25) / 2.0)),
                color: C_GRAY,
            });
        }
        match self.mode {
            0 | 1 => {
                for m in self.magnets() {
                    // Две половинки: верх — по направлению намагниченности (север).
                    let north_up = m.br >= 0.0;
                    for (k, sgn) in [(0, -1.0), (1, 1.0)] {
                        let c = m.center + Vec3::z() * (sgn * m.length / 4.0);
                        let col = if (k == 1) == north_up {
                            C_NORTH
                        } else {
                            C_SOUTH
                        };
                        out.push(VisualInstance {
                            shape: Shape::Cylinder {
                                radius: m.radius * S,
                                length: m.length / 2.0 * S,
                            },
                            pose: Pose::at(show(c)),
                            color: col,
                        });
                    }
                }
                if self.mode == 1 {
                    let w = 4.0 * self.r().max(self.l());
                    out.push(VisualInstance {
                        shape: Shape::Box {
                            half: Vec3::new(w * S, w * S, 0.4 * w * S),
                        },
                        pose: Pose::at(show(Vec3::new(0.0, 0.0, -0.4 * w))),
                        color: C_STEEL,
                    });
                }
            }
            2 => {
                let a = self.pole_mm / 1000.0;
                let g = self.g();
                let leg_h = 2.0 * a;
                let span = 3.0 * a;
                // Ноги, ярмо, якорь.
                for x in [-span / 2.0, span / 2.0] {
                    out.push(VisualInstance {
                        shape: Shape::Box {
                            half: Vec3::new(a / 2.0 * S, a / 2.0 * S, leg_h / 2.0 * S),
                        },
                        pose: Pose::at(show(Vec3::new(x, 0.0, -leg_h / 2.0))),
                        color: C_IRON,
                    });
                    out.push(VisualInstance {
                        shape: Shape::Cylinder {
                            radius: 0.85 * a * S,
                            length: 1.6 * a * S,
                        },
                        pose: Pose::at(show(Vec3::new(x, 0.0, -leg_h / 2.0))),
                        color: C_COPPER,
                    });
                }
                out.push(VisualInstance {
                    shape: Shape::Box {
                        half: Vec3::new((span / 2.0 + a / 2.0) * S, a / 2.0 * S, a / 2.0 * S),
                    },
                    pose: Pose::at(show(Vec3::new(0.0, 0.0, -leg_h - a / 2.0))),
                    color: C_IRON,
                });
                out.push(VisualInstance {
                    shape: Shape::Box {
                        half: Vec3::new((span / 2.0 + a / 2.0) * S, a / 2.0 * S, a / 2.0 * S),
                    },
                    pose: Pose::at(show(Vec3::new(0.0, 0.0, g + a / 2.0))),
                    color: C_STEEL,
                });
            }
            _ => {
                let m = self.disc();
                // Диаметральный диск: половина +x — север.
                for (sgn, col) in [(1.0, C_NORTH), (-1.0, C_SOUTH)] {
                    out.push(VisualInstance {
                        shape: Shape::Box {
                            half: Vec3::new(
                                m.radius / 2.0 * S,
                                m.radius * 0.9 * S,
                                m.thickness / 2.0 * S,
                            ),
                        },
                        pose: Pose::at(show(m.center + Vec3::x() * (sgn * m.radius / 2.0))),
                        color: col,
                    });
                }
                // Датчик (корпус SOIC-8 ~5×4×1.5 мм) под магнитом.
                let g = self.g();
                out.push(VisualInstance {
                    shape: Shape::Box {
                        half: Vec3::new(2.5e-3 * S, 2.0e-3 * S, 0.75e-3 * S),
                    },
                    pose: Pose::at(show(Vec3::new(0.0, 0.0, -g - 0.75e-3))),
                    color: [0.15, 0.15, 0.16, 1.0],
                });
                // Плата.
                out.push(VisualInstance {
                    shape: Shape::Box {
                        half: Vec3::new(12e-3 * S, 12e-3 * S, 0.8e-3 * S),
                    },
                    pose: Pose::at(show(Vec3::new(0.0, 0.0, -g - 1.5e-3 - 0.8e-3))),
                    color: [0.1, 0.45, 0.2, 1.0],
                });
            }
        }
        out
    }

    fn visual_epoch(&self) -> u64 {
        self.epoch
    }

    fn overlays(&self) -> Vec<Overlay> {
        let mut v = Vec::new();
        for line in &self.lines {
            for w in line.windows(2) {
                v.push(Overlay::Line {
                    from: show(w[0]),
                    to: show(w[1]),
                    color: C_LINE,
                });
            }
        }
        let f = self.force_now();
        if self.mode <= 2 && f.is_finite() {
            // Сила на верхнее тело: вниз — притяжение. Масштаб: 1 см экрана на 10 Н (лог).
            let len = 0.05 * (1.0 + f.abs()).ln();
            let top = match self.mode {
                0 => (self.g() + self.l()) / 2.0 + self.l() / 2.0,
                1 => self.g() + self.l(),
                _ => self.g() + self.pole_mm / 1000.0,
            };
            let from = show(Vec3::new(0.0, 0.0, top)) + Vec3::z() * 0.03;
            let dir = if f > 0.0 { -1.0 } else { 1.0 };
            v.push(Overlay::Arrow {
                from: from + Vec3::z() * (if dir < 0.0 { len } else { 0.0 }),
                to: from + Vec3::z() * (if dir < 0.0 { 0.0 } else { len }),
                color: C_YELLOW,
            });
        }
        v
    }

    fn readouts(&self) -> Vec<Readout> {
        let mut r = Vec::new();
        match self.mode {
            0 | 1 => {
                r.push(Readout::new("Br при этой температуре", self.br(), "Тл", 3));
                let m = self.magnets()[0];
                r.push(Readout::new("Магнитный момент", m.moment(), "А·м²", 3));
                r.push(Readout::new(
                    "Поле в середине зазора",
                    1000.0
                        * self
                            .field_sources()
                            .iter()
                            .map(|m| {
                                m.b_at(&Vec3::new(
                                    0.0,
                                    0.0,
                                    if self.mode == 0 { 0.0 } else { self.g() / 2.0 },
                                ))
                            })
                            .sum::<Vec3>()
                            .norm(),
                    "мТл",
                    0,
                ));
                let f = self.force_now();
                r.push(Readout::new(
                    if self.mode == 0 {
                        "Сила (+ притяжение)"
                    } else {
                        "Сила отрыва от стали"
                    },
                    f,
                    "Н",
                    2,
                ));
                r.push(Readout::new("То же в кгс", f / 9.80665, "кгс", 2));
            }
            2 => {
                let c = self.circuit();
                let st = c.solve(self.current_a);
                let coil = self.coil();
                r.push(Readout::new("Индукция в зазоре", st.b_gap_t, "Тл", 3));
                r.push(Readout::new("Индукция в стали", st.b_core_t, "Тл", 3));
                r.push(Readout::new("МДС на зазоры", st.mmf_gaps_a, "А", 0));
                r.push(Readout::new("МДС на сталь", st.mmf_core_a, "А", 0));
                r.push(Readout::new(
                    "Сила притяжения якоря",
                    c.force_maxwell_n(self.current_a),
                    "Н",
                    1,
                ));
                r.push(Readout::new(
                    "Индуктивность",
                    1000.0 * c.inductance_h(self.current_a.max(1e-3)),
                    "мГн",
                    1,
                ));
                r.push(Readout::new(
                    "Сопротивление катушки (20 °C)",
                    coil.resistance_ohm(20.0),
                    "Ом",
                    2,
                ));
                r.push(Readout::new(
                    "Напряжение",
                    self.current_a * coil.resistance_ohm(20.0),
                    "В",
                    1,
                ));
                r.push(Readout::new(
                    "Тепло в катушке",
                    coil.power_w(self.current_a, 20.0),
                    "Вт",
                    1,
                ));
                r.push(Readout::new(
                    "Плотность тока",
                    coil.current_density_a_per_mm2(self.current_a),
                    "А/мм²",
                    1,
                ));
                match coil.steady_temperature_c(self.current_a, 25.0) {
                    Some(t) => r.push(Readout::new(
                        "Установившаяся температура (25 °C вокруг)",
                        t,
                        "°C",
                        0,
                    )),
                    None => r.push(Readout::new(
                        "Установившаяся температура",
                        f64::INFINITY,
                        "°C — разгон",
                        0,
                    )),
                }
            }
            _ => {
                let m = self.disc();
                let bz = sensor_bz_amplitude(
                    m.radius,
                    m.thickness,
                    self.br(),
                    self.g(),
                    AS5600.hall_radius_m,
                );
                r.push(Readout::new("Bz у элементов Холла", 1000.0 * bz, "мТл", 1));
                r.push(Readout::new(
                    "Нужно AS5600",
                    1000.0 * AS5600.bz_min_t,
                    "мТл (мин)",
                    0,
                ));
                r.push(Readout::new(
                    "Нужно AS5600",
                    1000.0 * AS5600.bz_max_t,
                    "мТл (макс)",
                    0,
                ));
                if let Some((lo, hi)) = sensor_gap_window(m.radius, m.thickness, self.br(), &AS5600)
                {
                    r.push(Readout::new(
                        "Рабочее окно зазора: от",
                        1000.0 * lo,
                        "мм",
                        2,
                    ));
                    r.push(Readout::new(
                        "Рабочее окно зазора: до",
                        1000.0 * hi,
                        "мм",
                        2,
                    ));
                }
            }
        }
        r
    }

    fn comparisons(&self) -> Vec<Comparison> {
        self.cmp_cache.clone()
    }

    fn notes(&self) -> Vec<Note> {
        self.compute_notes()
    }

    fn plot(&self) -> Option<Plot> {
        self.plot_cache.clone()
    }

    fn focus(&self) -> Vec3 {
        if self.mode == 2 {
            STAGE - Vec3::z() * (0.3 * self.pole_mm / 20.0)
        } else {
            STAGE
        }
    }

    fn view_offset(&self) -> Vec3 {
        if self.mode == 2 {
            Vec3::new(0.7, -2.0, 0.6) * (self.pole_mm / 20.0).max(0.5)
        } else {
            Vec3::new(0.25, -0.75, 0.25)
        }
    }
}

impl Magnets {
    fn compute_comparisons(&self) -> Vec<Comparison> {
        let mut v = Vec::new();
        match self.mode {
            0 => {
                let p = self.pair(self.g());
                let a = -coaxial_force_loops(&p);
                let b = -coaxial_force_bessel(&p);
                v.push(Comparison {
                    title: "Сила между магнитами: два независимых интеграла".into(),
                    sim_label: "витки с токами (эллиптич. интегралы)".into(),
                    sim: a,
                    ref_label: "интеграл Ганкеля (Vokoun 2009)".into(),
                    reference: b,
                    unit: "Н",
                    decimals: 4,
                    status: if ((a - b) / b).abs() < 1e-6 {
                        Status::Ok
                    } else {
                        Status::Mismatch
                    },
                    explanation: "Одна физика (эквивалентные токи), разные математические пути."
                        .into(),
                });
                let (m1, m2) = p.moments();
                let d = -dipole_dipole_coaxial_force(m1, m2, p.center_distance());
                let ratio = a / d;
                v.push(Comparison {
                    title: "Против приближения «два диполя»".into(),
                    sim_label: "точно".into(),
                    sim: a,
                    ref_label: "3μ₀m₁m₂/(2πz⁴)".into(),
                    reference: d,
                    unit: "Н",
                    decimals: 3,
                    status: if (ratio - 1.0).abs() < 0.05 {
                        Status::Ok
                    } else {
                        Status::Explained
                    },
                    explanation: "Диполи верны далеко (зазор ≫ размера). Вблизи формула из \
                                  учебника ошибается в разы — не считайте ею защёлки."
                        .into(),
                });
            }
            1 => {
                let f = pull_force_ideal_steel(self.br(), self.r(), self.l(), self.g());
                let ub = contact_pull_upper_bound(self.br(), self.r());
                v.push(Comparison {
                    title: "Сила к стали против предела Br²A/2μ₀".into(),
                    sim_label: "метод изображений".into(),
                    sim: f,
                    ref_label: "давление Максвелла на торце".into(),
                    reference: ub,
                    unit: "Н",
                    decimals: 2,
                    status: if f <= ub * 1.001 {
                        Status::Explained
                    } else {
                        Status::Mismatch
                    },
                    explanation: "Предел достигается только вплотную и для длинного магнита; \
                                  с зазором и у короткого магнита сила меньше."
                        .into(),
                });
                if (self.diameter_mm - 12.7).abs() < 0.2
                    && (self.length_mm - 12.7).abs() < 0.2
                    && self.grade == 7
                {
                    v.push(Comparison {
                        title: "K&J D88-N52: модель против замера на плите".into(),
                        sim_label: "идеальная сталь".into(),
                        sim: f,
                        ref_label: "K&J Magnetics, 5 образцов".into(),
                        reference: KJ_D88_N52_PULL_N,
                        unit: "Н",
                        decimals: 1,
                        status: if (f / KJ_D88_N52_PULL_N - 1.0).abs() < 0.15 {
                            Status::Explained
                        } else {
                            Status::Mismatch
                        },
                        explanation: "Модель выше на ~10 %: реальная сталь насыщается и имеет \
                                      конечную μ, покрытие даёт зазор ~20 мкм, Br партии разный. \
                                      Для конструкции — брать замер, модель — верхняя граница."
                            .into(),
                    });
                }
            }
            2 => {
                let c = self.circuit();
                let fm = c.force_maxwell_n(self.current_a);
                let fw = c.force_virtual_work_n(self.current_a);
                v.push(Comparison {
                    title: "Сила якоря двумя способами".into(),
                    sim_label: "давление Максвелла n·B²A/2μ₀".into(),
                    sim: fm,
                    ref_label: "работа поля −∂W'/∂g".into(),
                    reference: fw,
                    unit: "Н",
                    decimals: 2,
                    status: if fw.abs() < 1e-9 || ((fm - fw) / fw).abs() < 1e-4 {
                        Status::Ok
                    } else {
                        Status::Mismatch
                    },
                    explanation: "Независимые пути: поле в зазоре и производная энергии по зазору."
                        .into(),
                });
                let ideal = MagneticCircuit {
                    material: CoreMaterial::Ideal,
                    ..c.clone()
                };
                let bi = ideal.solve(self.current_a).b_gap_t;
                let bn = c.solve(self.current_a).b_gap_t;
                let mu0 = 4e-7 * PI;
                v.push(Comparison {
                    title: "Индукция в зазоре против учебной формулы".into(),
                    sim_label: if self.ideal_core {
                        "идеальная сталь".into()
                    } else {
                        "AISI 1010 (B–H)".into()
                    },
                    sim: bn,
                    ref_label: "μ₀NI/(2g)".into(),
                    reference: mu0 * self.turns * self.current_a / (2.0 * self.g()),
                    unit: "Тл",
                    decimals: 3,
                    status: if ((bn - bi) / bi.max(1e-12)).abs() < 0.03 {
                        Status::Ok
                    } else {
                        Status::Explained
                    },
                    explanation:
                        "Расходится, когда сталь насыщается (>1.5 Тл) или зазор очень мал: \
                                  тогда МДС уходит на сталь, и ток перестаёт добавлять силу."
                            .into(),
                });
            }
            _ => {
                let m = self.disc();
                let p = Vec3::new(AS5600.hall_radius_m, 0.0, -self.g());
                let a = m.b_current_model(&p).z;
                let b = m.b_charge_model(&p, 720).z;
                v.push(Comparison {
                    title: "Поле у датчика: две модели магнита".into(),
                    sim_label: "токи на поверхности".into(),
                    sim: 1000.0 * a.abs(),
                    ref_label: "магнитные заряды".into(),
                    reference: 1000.0 * b.abs(),
                    unit: "мТл",
                    decimals: 2,
                    status: if ((a - b) / b).abs() < 1e-3 {
                        Status::Ok
                    } else {
                        Status::Mismatch
                    },
                    explanation: "Эквивалентные описания одного магнита — должны совпасть.".into(),
                });
                if let Some((lo, hi)) = sensor_gap_window(m.radius, m.thickness, self.br(), &AS5600)
                {
                    v.push(Comparison {
                        title: "Окно зазора против даташита AS5600".into(),
                        sim_label: "верх окна, модель".into(),
                        sim: 1000.0 * hi,
                        ref_label: "«0.5–3 мм» (даташит)".into(),
                        reference: 3.0,
                        unit: "мм",
                        decimals: 2,
                        status: Status::Explained,
                        explanation: format!(
                            "Модель: {:.2}–{:.2} мм от элементов Холла. Даташит считает от корпуса \
                             и для «типового» магнита; кривой B(зазор) производитель не даёт — \
                             сверка качественная.",
                            1000.0 * lo,
                            1000.0 * hi
                        ),
                    });
                }
            }
        }
        v
    }

    fn compute_notes(&self) -> Vec<Note> {
        let mut n = Vec::new();
        if self.mode != 2 {
            let grade = NDFEB_GRADES[self.grade];
            if let Some(cls) = temp_class_of(grade.name) {
                if self.temp_c > cls.max_operating_c {
                    n.push(Note::new(
                        Level::Danger,
                        format!(
                            "{} выше {:.0} °C начинает терять намагниченность необратимо. Возле \
                             мотора/на солнце берите класс H/SH (120/150 °C).",
                            grade.name, cls.max_operating_c
                        ),
                    ));
                } else {
                    n.push(Note::new(
                        Level::Info,
                        format!(
                            "Br падает на 0.12 %/°C (обратимо): при {:.0} °C — {:.3} Тл вместо {:.3}. \
                             Предел класса {} — {:.0} °C.",
                            self.temp_c,
                            self.br(),
                            grade.br_typical_t(),
                            grade.name,
                            cls.max_operating_c
                        ),
                    ));
                }
            }
        }
        match self.mode {
            0 => {
                n.push(Note::new(
                    Level::Info,
                    "Сила падает с зазором очень круто: защёлка, которая держит вплотную, \
                     через 2–3 мм ткани или ткани+чехла почти не держит. Считайте по точной \
                     кривой, а не по «силе с упаковки».",
                ));
                n.push(Note::new(
                    Level::Warn,
                    "Сильные магниты у тела: кардиостимуляторы требуют держать магниты подальше \
                     (обычно ≥ 15 см — см. инструкцию к конкретному устройству); пальцы \
                     защемляет между магнитами.",
                ));
            }
            1 => n.push(Note::new(
                Level::Info,
                "Модель — верхняя граница (идеальная сталь). Тонкий лист, краска и \
                 неровность снижают силу в разы — проверяйте на стенде.",
            )),
            2 => {
                let st = self.circuit().solve(self.current_a);
                if st.b_core_t > 1.5 && !self.ideal_core {
                    n.push(Note::new(
                        Level::Warn,
                        format!(
                            "Сталь насыщается ({:.2} Тл): {:.0} % МДС уходит в сердечник, \
                             добавка тока почти не добавляет силы, только тепла.",
                            st.b_core_t,
                            100.0 * st.mmf_core_a / (st.mmf_core_a + st.mmf_gaps_a)
                        ),
                    ));
                }
                let coil = self.coil();
                match coil.steady_temperature_c(self.current_a, 25.0) {
                    None => n.push(Note::new(
                        Level::Danger,
                        "Катушка не остынет: тепло растёт быстрее отвода (тепловой разгон). \
                         Уменьшите ток или включайте импульсами.",
                    )),
                    Some(t) if t > 105.0 => n.push(Note::new(
                        Level::Danger,
                        format!(
                            "Катушка разогреется до {t:.0} °C — выше класса изоляции A (105 °C) и \
                             опасно для кожи. Держите ток меньше или делайте режим «удержание» \
                             пониженным током."
                        ),
                    )),
                    Some(t) if t > 45.0 => n.push(Note::new(
                        Level::Warn,
                        format!("Катушка нагреется до {t:.0} °C: касаться кожей нельзя (ожог при долгом контакте выше ~43–45 °C)."),
                    )),
                    _ => {}
                }
                n.push(Note::new(
                    Level::Info,
                    "Сила ~ 1/зазор²: держит сильно вплотную, но слабо тянет издалека. Для \
                     фиксатора экзо: зазор ≤ 0.2 мм, удержание — малым током.",
                ));
            }
            _ => n.push(Note::new(
                Level::Info,
                "Угол локтя для экзо: магнит на оси шарнира, AS5600 на плате напротив. Люфт \
                 оси вдоль неё должен оставаться внутри рабочего окна зазора, иначе датчик \
                 выдаст ошибку поля — прошивка должна это ловить как отказ датчика.",
            )),
        }
        n
    }

    fn compute_plot(&self) -> Option<Plot> {
        match self.mode {
            0 | 1 => {
                let gmax = (4.0 * self.diameter_mm).max(10.0);
                let mut exact = Vec::new();
                let mut dip = Vec::new();
                for k in 0..40 {
                    let g = 0.05 * (gmax / 0.05f64).powf(k as f64 / 39.0);
                    let f = if self.mode == 0 {
                        -coaxial_force_loops(&self.pair(g / 1000.0))
                    } else {
                        pull_force_ideal_steel(self.br(), self.r(), self.l(), g / 1000.0)
                    };
                    exact.push([g, f]);
                    if self.mode == 0 {
                        let p = self.pair(g / 1000.0);
                        let (m1, m2) = p.moments();
                        let d = -dipole_dipole_coaxial_force(m1, m2, p.center_distance());
                        if d.abs() < 3.0 * exact[0][1].abs() {
                            dip.push([g, d]);
                        }
                    }
                }
                let mut series = vec![Series {
                    name: "точно (эквивалентные токи)".into(),
                    points: exact,
                    color: C_YELLOW,
                    style: SeriesStyle::Solid,
                }];
                if !dip.is_empty() {
                    series.push(Series {
                        name: "два диполя".into(),
                        points: dip,
                        color: C_GRAY,
                        style: SeriesStyle::Dashed,
                    });
                }
                series.push(Series {
                    name: "сейчас".into(),
                    points: vec![[self.gap_mm, 0.0], [self.gap_mm, self.force_now()]],
                    color: C_RED,
                    style: SeriesStyle::Solid,
                });
                Some(Plot {
                    title: "Сила от зазора".into(),
                    x_label: "зазор, мм".into(),
                    y_label: "Н".into(),
                    series,
                    bands: vec![],
                })
            }
            2 => {
                let c = self.circuit();
                let ideal = MagneticCircuit {
                    material: CoreMaterial::Ideal,
                    ..c.clone()
                };
                let imax = (2.0 * self.current_a).max(2.0);
                let (mut real, mut lin) = (Vec::new(), Vec::new());
                for k in 0..=40 {
                    let i = imax * k as f64 / 40.0;
                    real.push([i, c.solve(i).b_gap_t]);
                    lin.push([i, ideal.solve(i).b_gap_t.min(3.0)]);
                }
                Some(Plot {
                    title: "Индукция в зазоре от тока".into(),
                    x_label: "ток, А".into(),
                    y_label: "Тл".into(),
                    series: vec![
                        Series {
                            name: if self.ideal_core {
                                "идеальная сталь".into()
                            } else {
                                "AISI 1010".into()
                            },
                            points: real,
                            color: C_YELLOW,
                            style: SeriesStyle::Solid,
                        },
                        Series {
                            name: "без насыщения μ₀NI/2g".into(),
                            points: lin,
                            color: C_GRAY,
                            style: SeriesStyle::Dashed,
                        },
                        Series {
                            name: "сейчас".into(),
                            points: vec![
                                [self.current_a, 0.0],
                                [self.current_a, c.solve(self.current_a).b_gap_t],
                            ],
                            color: C_RED,
                            style: SeriesStyle::Solid,
                        },
                    ],
                    bands: vec![],
                })
            }
            _ => {
                let m = self.disc();
                let mut pts = Vec::new();
                for k in 0..=40 {
                    let g = 0.1 + 5.9 * k as f64 / 40.0;
                    pts.push([
                        g,
                        1000.0
                            * sensor_bz_amplitude(
                                m.radius,
                                m.thickness,
                                self.br(),
                                g / 1000.0,
                                AS5600.hall_radius_m,
                            ),
                    ]);
                }
                Some(Plot {
                    title: "Поле у датчика от зазора".into(),
                    x_label: "зазор, мм".into(),
                    y_label: "мТл".into(),
                    series: vec![
                        Series {
                            name: "|Bz| у элементов Холла".into(),
                            points: pts,
                            color: C_YELLOW,
                            style: SeriesStyle::Solid,
                        },
                        Series {
                            name: "сейчас".into(),
                            points: vec![[self.gap_mm, 0.0], [self.gap_mm, 150.0]],
                            color: C_RED,
                            style: SeriesStyle::Solid,
                        },
                    ],
                    bands: vec![Band {
                        label: "окно AS5600 30–90 мТл".into(),
                        lo: 30.0,
                        hi: 90.0,
                        color: [0.2, 0.85, 0.35, 0.18],
                    }],
                })
            }
        }
    }
}
