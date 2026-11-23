//! Сценарий «Плата привода экзо»: H-мост мотора, шунт тока, понижающий стабилизатор,
//! микроконтроллер с линией к антенне 2.4 ГГц на двухслойной плате FR-4. Считаем
//! (`sim-circuit::pcb`): нагрев дорожек по IPC-2221, падение напряжения, температуру
//! переходов деталей по θJA, волновое сопротивление линии к антенне двумя формулами.
//! Ток мотора и температура воздуха — те же величины, что в «Прошивке в контуре» и «Среде».

use sim_circuit::pcb::{
    Board, DesignRules, FR4_HIGH_TG, FR4_STANDARD, IpcLayer, Part, Trace, TraceLayer, evaluate,
    ipc2221_width, microstrip_hammerstad_jensen, microstrip_ipc2141, oz_to_m, package,
};
use sim_core::{Pose, Shape, Vec3};

use crate::scenario::*;

/// Увеличение на экране (плата 70×45 мм → 0.35×0.225 м).
const S: f64 = 5.0;
const Z: f64 = 0.9;
const MM: f64 = 1e-3;
/// Частота ШИМ моста и суммарное время переключения (оценка для MOSFET 40 В класса), Гц, с.
const F_PWM: f64 = 20e3;
const T_SW: f64 = 40e-9;

const FET_PACKAGES: [&str; 3] = ["PowerPAK SO-8 (5×6)", "TO-252 (DPAK)", "TO-263 (D2PAK)"];

pub struct PcbBoard {
    i_motor: f64,
    v_bus: f64,
    trace_mm: f64,
    rf_mm: f64,
    rds_mohm: f64,
    ambient: f64,
    copper2oz: bool,
    fet_pkg: usize,
    high_tg: bool,
    t: f64,
    epoch: u64,
}

impl Default for PcbBoard {
    fn default() -> Self {
        Self {
            i_motor: 8.0,
            v_bus: 22.2,
            trace_mm: 1.0,
            rf_mm: 1.6,
            rds_mohm: 8.0,
            ambient: 25.0,
            copper2oz: false,
            fet_pkg: 0,
            high_tg: false,
            t: 0.0,
            epoch: 0,
        }
    }
}

impl PcbBoard {
    /// Отчёт по дорожкам и деталям при текущих параметрах.
    pub fn report(&self) -> sim_circuit::pcb::BoardReport {
        evaluate(&self.board(), self.ambient, &DesignRules::default())
    }

    fn board(&self) -> Board {
        let mut b = Board::new(70.0 * MM, 45.0 * MM);
        b.copper_oz_outer = if self.copper2oz { 2.0 } else { 1.0 };
        b.laminate = if self.high_tg {
            FR4_HIGH_TG
        } else {
            FR4_STANDARD
        };
        let w = self.trace_mm * MM;
        let i = self.i_motor;
        let tr = |pts: &[[f64; 2]], w: f64, net: &str, cur: f64| Trace {
            points: pts.iter().map(|p| [p[0] * MM, p[1] * MM]).collect(),
            width: w,
            layer: TraceLayer::Top,
            net: net.into(),
            current: cur,
        };
        b.traces = vec![
            tr(
                &[[5.0, 35.0], [25.0, 35.0], [25.0, 28.0]],
                w,
                "+BAT → мост",
                i,
            ),
            tr(
                &[[33.0, 24.0], [52.0, 24.0], [64.0, 24.0]],
                w,
                "мост → мотор A",
                i,
            ),
            tr(
                &[[33.0, 14.0], [52.0, 14.0], [64.0, 14.0]],
                w,
                "мост → мотор B",
                i,
            ),
            tr(
                &[[25.0, 10.0], [25.0, 5.0], [12.0, 5.0]],
                w,
                "мост → шунт → GND",
                i,
            ),
            tr(
                &[[8.0, 30.0], [8.0, 20.0]],
                0.4 * MM,
                "+BAT → стабилизатор",
                0.05,
            ),
            tr(&[[12.0, 18.0], [40.0, 38.0]], 0.3 * MM, "+3V3 → МК", 0.35),
            tr(
                &[[48.0, 40.0], [60.0, 40.0]],
                self.rf_mm * MM,
                "RF → антенна (50 Ом)",
                0.0,
            ),
        ];
        let pkg = package(FET_PACKAGES[self.fet_pkg]).expect("корпус в таблице");
        let rds = self.rds_mohm * 1e-3;
        // В мосту одновременно проводят два ключа; в среднем каждый — половину времени.
        let p_cond = i * i * rds * 0.5;
        let p_sw = 0.5 * self.v_bus * i * T_SW * F_PWM;
        let fet = |name: &str, x: f64, y: f64| Part {
            name: name.into(),
            footprint: [6.0 * MM, 5.0 * MM],
            pos: [x * MM, y * MM],
            power_w: p_cond + p_sw,
            theta_ja: pkg.theta_ja,
            tj_max_c: 150.0,
        };
        let buck_loss = 3.3 * 0.35 * (1.0 / 0.85 - 1.0);
        b.parts = vec![
            fet("Q1 мост, верх A", 25.0, 24.0),
            fet("Q2 мост, низ A", 25.0, 14.0),
            fet("Q3 мост, верх B", 33.0, 24.0),
            fet("Q4 мост, низ B", 33.0, 14.0),
            Part {
                name: "R1 шунт 5 мОм".into(),
                footprint: [6.3 * MM, 3.2 * MM],
                pos: [18.0 * MM, 5.0 * MM],
                power_w: i * i * 0.005,
                theta_ja: 60.0,
                tj_max_c: 155.0,
            },
            Part {
                name: "U2 стабилизатор 3.3 В".into(),
                footprint: [3.0 * MM, 3.0 * MM],
                pos: [10.0 * MM, 18.0 * MM],
                power_w: buck_loss,
                theta_ja: package("SOT-23-6 (силовая ИС)")
                    .map(|p| p.theta_ja)
                    .unwrap_or(92.6),
                tj_max_c: 125.0,
            },
            Part {
                name: "U1 МК (как ESP32-C3), Wi-Fi/BLE".into(),
                footprint: [5.0 * MM, 5.0 * MM],
                pos: [44.0 * MM, 38.0 * MM],
                power_w: 3.3 * 0.1,
                theta_ja: package("VQFN-16 3×3").map(|p| p.theta_ja).unwrap_or(45.0),
                tj_max_c: 105.0,
            },
        ];
        b
    }

    fn copper_t(&self) -> f64 {
        oz_to_m(if self.copper2oz { 2.0 } else { 1.0 })
    }
}

fn heat_color(t: f64) -> [f32; 4] {
    // 25 °C — медь, 60 °C — оранжевый, 100+ — красный.
    let k = ((t - 25.0) / 75.0).clamp(0.0, 1.0) as f32;
    [
        0.85 + 0.15 * k,
        0.55 * (1.0 - k) + 0.15 * k,
        0.25 * (1.0 - k) + 0.1 * k,
        1.0,
    ]
}

impl Scenario for PcbBoard {
    fn id(&self) -> &'static str {
        "pcb_board"
    }

    fn title(&self) -> String {
        "Плата привода экзо: дорожки и нагрев".into()
    }

    fn summary(&self) -> String {
        "Двухслойная плата FR-4: H-мост мотора, шунт тока, стабилизатор 3.3 В, микроконтроллер \
         с линией к антенне. Нагрев дорожек по IPC-2221, температура переходов по θJA, волновое \
         сопротивление линии к антенне двумя формулами. Лупа 5:1."
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
        vec![
            p(
                "i",
                "Ток мотора (длительно)",
                self.i_motor,
                0.5,
                20.0,
                "А",
                1,
            ),
            p("v", "Напряжение шины", self.v_bus, 12.0, 50.0, "В", 1),
            p(
                "w",
                "Ширина силовых дорожек",
                self.trace_mm,
                0.2,
                6.0,
                "мм",
                2,
            ),
            p(
                "rf",
                "Ширина линии к антенне",
                self.rf_mm,
                0.2,
                4.0,
                "мм",
                2,
            ),
            p(
                "rds",
                "Сопротивление ключа R_DS(on)",
                self.rds_mohm,
                1.0,
                50.0,
                "мОм",
                1,
            ),
            p(
                "amb",
                "Воздух вокруг платы",
                self.ambient,
                -40.0,
                85.0,
                "°C",
                0,
            ),
        ]
    }

    fn set_param(&mut self, id: &str, value: f64) {
        match id {
            "i" => self.i_motor = value,
            "v" => self.v_bus = value,
            "w" => self.trace_mm = value,
            "rf" => self.rf_mm = value,
            "rds" => self.rds_mohm = value,
            "amb" => self.ambient = value,
            _ => return,
        }
        self.epoch += 1;
    }

    fn choices(&self) -> Vec<Choice> {
        vec![
            Choice {
                id: "cu",
                label: "Медь".into(),
                options: vec!["медь 1 oz (35 мкм)".into(), "медь 2 oz (70 мкм)".into()],
                selected: self.copper2oz as usize,
            },
            Choice {
                id: "pkg",
                label: "Корпус ключей".into(),
                options: FET_PACKAGES.iter().map(|s| format!("ключи: {s}")).collect(),
                selected: self.fet_pkg,
            },
            Choice {
                id: "lam",
                label: "Материал".into(),
                options: vec![FR4_STANDARD.name.into(), FR4_HIGH_TG.name.into()],
                selected: self.high_tg as usize,
            },
        ]
    }

    fn set_choice(&mut self, id: &str, index: usize) {
        match id {
            "cu" => self.copper2oz = index == 1,
            "pkg" => self.fet_pkg = index.min(FET_PACKAGES.len() - 1),
            "lam" => self.high_tg = index == 1,
            _ => return,
        }
        self.epoch += 1;
    }

    fn presets(&self) -> Vec<String> {
        vec![
            "Тонкие дорожки 1 мм, 8 А".into(),
            "Как надо: 3 мм, 2 oz".into(),
            "Жара в корпусе на солнце 70 °C".into(),
        ]
    }

    fn apply_preset(&mut self, index: usize) {
        *self = Self::default();
        match index {
            1 => {
                self.trace_mm = 3.0;
                self.copper2oz = true;
            }
            2 => {
                self.trace_mm = 3.0;
                self.copper2oz = true;
                self.ambient = 70.0;
            }
            _ => {}
        }
        self.epoch += 1;
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
        let b = self.board();
        let rep = evaluate(&b, self.ambient, &DesignRules::default());
        let (w, h) = (b.width, b.height);
        let at =
            |x: f64, y: f64, z: f64| Vec3::new((x - w / 2.0) * S, (y - h / 2.0) * S, Z + z * S);
        let mut v = vec![
            VisualInstance {
                shape: Shape::Box {
                    half: Vec3::new(w / 2.0 * S, h / 2.0 * S, b.thickness / 2.0 * S),
                },
                pose: Pose::at(at(w / 2.0, h / 2.0, 0.0)),
                color: [0.08, 0.38, 0.17, 1.0],
            },
            VisualInstance {
                shape: Shape::Cylinder {
                    radius: 0.03,
                    length: Z - 0.01,
                },
                pose: Pose::at(Vec3::new(0.0, 0.0, (Z - 0.01) / 2.0)),
                color: C_GRAY,
            },
        ];
        let cu = self.copper_t();
        for (tr, r) in b.traces.iter().zip(rep.traces.iter()) {
            for seg in tr.points.windows(2) {
                let (a, c) = (seg[0], seg[1]);
                let len = ((c[0] - a[0]).powi(2) + (c[1] - a[1]).powi(2)).sqrt();
                let mid = at(
                    (a[0] + c[0]) / 2.0,
                    (a[1] + c[1]) / 2.0,
                    b.thickness / 2.0 + cu / 2.0,
                );
                let dir = Vec3::new(c[0] - a[0], c[1] - a[1], 0.0).normalize();
                // Дорожка — плоский брусок вдоль сегмента: капсула по оси даёт скругления.
                v.push(VisualInstance {
                    shape: Shape::Capsule {
                        radius: (tr.width / 2.0 * S).max(0.0008),
                        length: len * S,
                    },
                    pose: Pose::z_along(mid, dir),
                    color: if tr.current > 0.0 {
                        heat_color(r.temp_c)
                    } else {
                        [0.9, 0.75, 0.3, 1.0]
                    },
                });
            }
        }
        for (p, r) in b.parts.iter().zip(rep.parts.iter()) {
            let hgt = 1.2 * MM;
            let k = (1.0 - r.margin_c / 100.0).clamp(0.0, 1.0) as f32;
            v.push(VisualInstance {
                shape: Shape::Box {
                    half: Vec3::new(
                        p.footprint[0] / 2.0 * S,
                        p.footprint[1] / 2.0 * S,
                        hgt / 2.0 * S,
                    ),
                },
                pose: Pose::at(at(p.pos[0], p.pos[1], b.thickness / 2.0 + hgt / 2.0)),
                color: [0.12 + 0.8 * k, 0.12, 0.13, 1.0],
            });
        }
        v
    }

    fn visual_epoch(&self) -> u64 {
        self.epoch
    }

    fn readouts(&self) -> Vec<Readout> {
        let b = self.board();
        let rep = evaluate(&b, self.ambient, &DesignRules::default());
        let mut r = Vec::new();
        for t in &rep.traces {
            if b.traces[t.index].current > 0.1 {
                r.push(Readout::new(
                    &format!("{}: перегрев", t.net),
                    t.delta_t,
                    "К",
                    1,
                ));
                r.push(Readout::new(
                    &format!("{}: падение", t.net),
                    1000.0 * t.voltage_drop,
                    "мВ",
                    0,
                ));
            }
        }
        for p in &rep.parts {
            r.push(Readout::new(
                &format!("{}: T перехода", p.name),
                p.tj_c,
                "°C",
                0,
            ));
        }
        let loss: f64 = b.parts.iter().map(|p| p.power_w).sum::<f64>()
            + rep.traces.iter().map(|t| t.power_w).sum::<f64>();
        r.push(Readout::new("Потери на плате всего", loss, "Вт", 2));
        r
    }

    fn comparisons(&self) -> Vec<Comparison> {
        let b = self.board();
        let t = self.copper_t();
        let need = ipc2221_width(self.i_motor, 20.0, t, IpcLayer::External);
        let er = b.laminate.er_1ghz;
        let h = b.thickness;
        let w = self.rf_mm * MM;
        let ipc = microstrip_ipc2141(w, h, t, er);
        let hj = microstrip_hammerstad_jensen(w, h, t, er);
        vec![
            Comparison {
                title: "Ширина силовой дорожки против IPC-2221 (перегрев 20 К)".into(),
                sim_label: "у вас".into(),
                sim: self.trace_mm,
                ref_label: "нужно по IPC-2221".into(),
                reference: need / MM,
                unit: "мм",
                decimals: 2,
                status: if self.trace_mm >= need / MM {
                    Status::WithinLimit
                } else {
                    Status::OverLimit
                },
                explanation: "IPC-2221 консервативен для одиночной дорожки на воздухе; полигоны \
                              и вторая сторона охлаждают лучше (IPC-2152). Меньше — греется."
                    .into(),
            },
            Comparison {
                title: "Линия к антенне 2.4 ГГц: волновое сопротивление двумя формулами".into(),
                sim_label: "Хаммерстад–Йенсен".into(),
                sim: hj.z0,
                ref_label: "IPC-2141".into(),
                reference: ipc.z0,
                unit: "Ом",
                decimals: 1,
                status: if !(ipc.valid && hj.valid) {
                    Status::Explained
                } else if ((hj.z0 - ipc.z0) / hj.z0).abs() < 0.05 {
                    Status::Ok
                } else {
                    Status::Explained
                },
                explanation: format!(
                    "Нужно 50 Ом (вход ESP32-C3 и антенны). Сейчас {:.0} Ом: {}. Формула IPC \
                     годится только при 0.1 < w/h < 2.",
                    hj.z0,
                    if (hj.z0 - 50.0).abs() < 5.0 {
                        "согласовано"
                    } else {
                        "рассогласование — часть мощности отразится"
                    }
                ),
            },
        ]
    }

    fn notes(&self) -> Vec<Note> {
        let b = self.board();
        let rep = evaluate(&b, self.ambient, &DesignRules::default());
        let mut n: Vec<Note> = rep
            .warnings
            .iter()
            .map(|w| Note::new(Level::Warn, format!("{}: {}", w.item, w.message)))
            .collect();
        for p in &rep.parts {
            if p.margin_c < 0.0 {
                n.push(Note::new(
                    Level::Danger,
                    format!(
                        "{}: переход {:.0} °C — выше предела детали. Сгорит или уйдёт в защиту.",
                        p.name, p.tj_c
                    ),
                ));
            }
        }
        let hot = rep.traces.iter().map(|t| t.temp_c).fold(f64::MIN, f64::max);
        if hot > b.laminate.max_operating_c {
            n.push(Note::new(
                Level::Danger,
                format!(
                    "Дорожка {:.0} °C — выше рабочего предела материала платы {:.0} °C ({}).",
                    hot, b.laminate.max_operating_c, b.laminate.name
                ),
            ));
        }
        if n.is_empty() {
            // Взаимный нагрев деталей не считается, поэтому зелёный только с запасом 20 °C.
            let min_margin = rep.parts.iter().map(|p| p.margin_c).fold(f64::INFINITY, f64::min);
            if min_margin >= 20.0 {
                n.push(Note::new(
                    Level::Good,
                    "Дорожки и детали в пределах при этих токе и температуре.",
                ));
            } else {
                n.push(Note::new(
                    Level::Warn,
                    format!("Запас по температуре детали {min_margin:.0} °C, нужно от 20 °C: взаимный нагрев через плату не посчитан."),
                ));
            }
        }
        n.push(Note::new(
            Level::Info,
            "Ток мотора берите из «Прошивки в контуре» (пиковый и длительный), температуру воздуха — \
             из «Среды» (корпус на солнце нагревается до 70+ °C). Взаимный нагрев деталей через \
             плату не учтён — запас по температуре держите ≥ 20 °C.",
        ));
        n
    }

    fn focus(&self) -> Vec3 {
        Vec3::new(0.0, 0.0, Z)
    }

    fn view_offset(&self) -> Vec3 {
        Vec3::new(0.1, -0.45, 0.35)
    }
}
