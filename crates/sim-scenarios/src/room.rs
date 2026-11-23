//! Комната: пол, стены, стол, стул, столик и предметы со своей массой, трением и упругостью.
//! Контакты считает MuJoCo (сверка с ручным расчётом — `sim-mujoco/tests/contact.rs`).
//!
//! Шаг 0.1 мс даёт 100 шагов на удар 10 мс: при меньшем числе шагов ошибка упругости
//! больше 2.5%.

use sim_core::{Shape, Vec3};
use sim_mujoco::contact::restitution_solref;
use sim_mujoco::ffi::mjtObj;
use sim_mujoco::{GeomInfo, GeomKind, Sim};

use crate::anatomy::mat_to_pose;
use crate::scenario::*;

const DT: f64 = 1e-4;
const CONTACT_TIME_S: f64 = 0.01;
const G: f64 = 9.81;
const IRON_DENSITY: f64 = 7200.0;
const ROLL_SPEED: f64 = 3.0;

const PRESET_DROP: usize = 1;
const PRESET_ROLL: usize = 2;
const PRESET_TIP: usize = 3;

/// Столик: масса, плечо опоры (внешний край ножки) и точка, куда ставим груз, от центра.
pub const SIDE_TABLE_MASS: f64 = 2.0;
pub const SIDE_TABLE_PIVOT: f64 = 0.112;
pub const SIDE_TABLE_LOAD_X: f64 = 0.22;

/// Масса груза на краю, при которой столик начинает опрокидываться: m·(x − a) = M·a.
pub fn tip_mass() -> f64 {
    SIDE_TABLE_MASS * SIDE_TABLE_PIVOT / (SIDE_TABLE_LOAD_X - SIDE_TABLE_PIVOT)
}

/// Свойства поверхности. Трение и упругость — справочные оценки, не замеры.
#[derive(Clone, Copy)]
struct Surface {
    friction: f64,
    restitution: f64,
    rgba: [f32; 4],
}

const FLOOR: Surface = Surface { friction: 0.6, restitution: 0.2, rgba: [0.45, 0.45, 0.47, 1.0] };
const WALL: Surface = Surface { friction: 0.5, restitution: 0.2, rgba: [0.32, 0.35, 0.40, 1.0] };
const WOOD: Surface = Surface { friction: 0.4, restitution: 0.35, rgba: [0.62, 0.44, 0.27, 1.0] };
const DARK_WOOD: Surface = Surface { friction: 0.4, restitution: 0.35, rgba: [0.38, 0.26, 0.17, 1.0] };
const IRON: Surface = Surface { friction: 0.3, restitution: 0.45, rgba: [0.13, 0.13, 0.15, 1.0] };
const CARDBOARD: Surface = Surface { friction: 0.45, restitution: 0.1, rgba: [0.76, 0.6, 0.42, 1.0] };
const PAPER: Surface = Surface { friction: 0.4, restitution: 0.15, rgba: [0.2, 0.45, 0.75, 1.0] };
const PLASTIC: Surface = Surface { friction: 0.3, restitution: 0.4, rgba: [0.3, 0.75, 0.8, 1.0] };

fn geom(name: &str, kind: &str, size: &str, pos: &str, mass: Option<f64>, s: Surface, group: u8) -> String {
    let (k, b) = restitution_solref(s.restitution, CONTACT_TIME_S);
    let mass = mass.map(|m| format!(r#"mass="{m}""#)).unwrap_or_default();
    let [r, g, bl, a] = s.rgba;
    format!(
        r#"<geom name="{name}" type="{kind}" size="{size}" pos="{pos}" {mass} group="{group}" friction="{} 0.005 0.0001" solref="{k} {b}" solimp="0.99 0.99 0.001 0.5 2" rgba="{r} {g} {bl} {a}"/>"#,
        s.friction
    )
}

fn body(name: &str, pos: [f64; 3], geoms: &str) -> String {
    format!(
        r#"<body name="{name}" pos="{} {} {}"><freejoint name="{name}_j"/>{geoms}</body>"#,
        pos[0], pos[1], pos[2]
    )
}

fn legs(prefix: &str, x: f64, y: f64, half: f64, half_z: f64, mass: f64, s: Surface) -> String {
    [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)]
        .iter()
        .enumerate()
        .map(|(i, (a, b))| {
            geom(
                &format!("{prefix}_leg{i}"),
                "box",
                &format!("{half} {half} {half_z}"),
                &format!("{} {} {half_z}", a * x, b * y),
                Some(mass),
                s,
                0,
            )
        })
        .collect()
}

fn weight_radius(mass: f64) -> f64 {
    (3.0 * mass / (4.0 * std::f64::consts::PI * IRON_DENSITY)).cbrt()
}

fn scene_xml(c: &RoomConfig) -> String {
    let mut w = String::new();
    w += &geom("floor", "plane", "2 1.8 0.1", "0 0 0", None, FLOOR, 3);
    w += &geom("wall_back", "box", "2.1 0.05 1.3", "0 1.85 1.3", None, WALL, 0);
    w += &geom("wall_left", "box", "0.05 1.9 1.3", "-2.05 0 1.3", None, WALL, 0);
    // Эти две стены физические, но не рисуются: через них смотрит камера.
    w += &geom("wall_front", "box", "2.1 0.05 1.3", "0 -1.85 1.3", None, WALL, 3);
    w += &geom("wall_right", "box", "0.05 1.9 1.3", "2.05 0 1.3", None, WALL, 3);

    let table = geom("table_top", "box", "0.6 0.35 0.02", "0 0 0.73", Some(17.0), WOOD, 0)
        + &legs("table", 0.55, 0.30, 0.03, 0.355, 2.0, DARK_WOOD);
    w += &body("table", [0.0, 0.0, 0.001], &table);

    let chair = geom("chair_seat", "box", "0.21 0.21 0.015", "0 0 0.45", Some(2.5), WOOD, 0)
        + &geom("chair_back", "box", "0.21 0.015 0.2", "0 -0.195 0.665", Some(1.5), WOOD, 0)
        + &legs("chair", 0.19, 0.19, 0.015, 0.2175, 0.4, DARK_WOOD);
    w += &body("chair", [0.0, -0.85, 0.001], &chair);

    let side = geom("side_top", "box", "0.25 0.25 0.015", "0 0 0.5", Some(1.4), WOOD, 0)
        + &legs("side", 0.10, 0.10, 0.012, 0.2425, 0.15, DARK_WOOD);
    let side_pos = [1.25, -0.9, 0.001];
    w += &body("side_table", side_pos, &side);

    w += &body("book", [-0.3, 0.1, 0.767], &geom("book_g", "box", "0.1 0.14 0.015", "0 0 0", Some(0.6), PAPER, 0));
    w += &body("bottle", [0.3, -0.1, 0.862], &geom("bottle_g", "cylinder", "0.035 0.11", "0 0 0", Some(0.6), PLASTIC, 0));
    w += &body("box", [0.7, 1.1, 0.151], &geom("box_g", "box", "0.2 0.15 0.15", "0 0 0", Some(0.8), CARDBOARD, 0));

    let r = weight_radius(c.mass_kg);
    let sphere = geom("weight_g", "sphere", &format!("{r}"), "0 0 0", Some(c.mass_kg), IRON, 0);
    w += &match c.preset {
        PRESET_DROP => body("weight", [0.0, 0.0, 0.751 + c.drop_m + r], &sphere),
        PRESET_TIP => {
            let hz = (c.mass_kg / IRON_DENSITY / (0.05 * 0.18) / 2.0).max(0.01);
            let g = geom("weight_g", "box", &format!("0.025 0.09 {hz}"), "0 0 0", Some(c.mass_kg), IRON, 0);
            body("weight", [side_pos[0] + SIDE_TABLE_LOAD_X, side_pos[1], side_pos[2] + 0.515 + hz + 0.001], &g)
        }
        _ => body("weight", [-1.3, 1.1, r + 0.001], &sphere),
    };
    format!(
        r#"<mujoco><option timestep="{DT}" gravity="0 0 -{G}" cone="elliptic" impratio="10"><flag energy="enable"/></option><worldbody>{w}</worldbody></mujoco>"#
    )
}

#[derive(Clone, Copy, Debug)]
pub struct RoomConfig {
    pub preset: usize,
    pub mass_kg: f64,
    pub drop_m: f64,
}

pub struct Room {
    cfg: RoomConfig,
    sim: Option<Sim>,
    error: Option<String>,
    geoms: Vec<GeomInfo>,
    clock: FixedClock,
    epoch: u64,
    energy0: f64,
    impact_speed: Option<f64>,
    peak_force: f64,
    floor_force: f64,
    peak_floor: f64,
    ncon: usize,
    hist: History,
    start: Vec<[f64; 3]>,
    weight_geom: i32,
    floor_geom: i32,
    weight_dof: usize,
    steps: u64,
}

impl Default for Room {
    fn default() -> Self {
        let mut r = Self {
            cfg: RoomConfig { preset: 0, mass_kg: 16.0, drop_m: 0.8 },
            sim: None,
            error: None,
            geoms: Vec::new(),
            clock: FixedClock::new(DT),
            epoch: 0,
            energy0: 0.0,
            impact_speed: None,
            peak_force: 0.0,
            floor_force: 0.0,
            peak_floor: 0.0,
            ncon: 0,
            hist: History::new(1, 8.0),
            start: Vec::new(),
            weight_geom: -1,
            floor_geom: -1,
            weight_dof: 0,
            steps: 0,
        };
        r.rebuild();
        r
    }
}

impl Room {
    fn rebuild(&mut self) {
        self.sim = None;
        self.error = None;
        match Sim::from_xml(&scene_xml(&self.cfg)) {
            Ok(s) => {
                self.geoms = s.geoms();
                self.weight_geom = s.id(mjtObj::mjOBJ_GEOM, "weight_g").map_or(-1, |i| i as i32);
                self.floor_geom = s.id(mjtObj::mjOBJ_GEOM, "floor").map_or(-1, |i| i as i32);
                let joint = s.id(mjtObj::mjOBJ_JOINT, "weight_j");
                self.weight_dof = s.joints().iter().find(|j| Some(j.id) == joint).map_or(0, |j| j.dof_adr);
                self.sim = Some(s);
                self.restart();
            }
            Err(e) => self.error = Some(e),
        }
        self.epoch += 1;
    }

    fn restart(&mut self) {
        let (preset, dof) = (self.cfg.preset, self.weight_dof);
        let Some(s) = self.sim.as_mut() else {
            return;
        };
        s.reset();
        if preset == PRESET_ROLL {
            s.qvel_mut()[dof] = ROLL_SPEED;
            s.forward();
        }
        self.energy0 = s.data().energy[0] + s.data().energy[1];
        self.start = (0..s.nbody()).map(|b| s.body_pose(b).0).collect();
        self.impact_speed = None;
        self.peak_force = 0.0;
        self.floor_force = 0.0;
        self.peak_floor = 0.0;
        self.ncon = 0;
        self.steps = 0;
        self.clock.reset();
        self.hist.clear();
    }

    fn step_once(&mut self) {
        let (weight, floor, dof) = (self.weight_geom, self.floor_geom, self.weight_dof);
        let Some(s) = self.sim.as_mut() else {
            return;
        };
        let speed_before = s.qvel()[dof + 2].abs();
        s.step();
        let contacts = s.contacts();
        self.ncon = contacts.len();
        self.floor_force = contacts
            .iter()
            .filter(|c| c.geom[0] == floor || c.geom[1] == floor)
            .map(|c| c.normal_force * c.normal[2].abs())
            .sum();
        self.peak_floor = self.peak_floor.max(self.floor_force);
        for c in &contacts {
            self.peak_force = self.peak_force.max(c.normal_force);
        }
        if self.impact_speed.is_none() && contacts.iter().any(|c| c.geom[0] == weight || c.geom[1] == weight) {
            self.impact_speed = Some(speed_before);
        }
        self.steps += 1;
        if self.steps % 50 == 0 {
            let t = s.time();
            self.hist.push(t, &[self.floor_force]);
        }
    }

    /// Прокрутить модельное время (для тестов и проверок без окна).
    pub fn run_for(&mut self, seconds: f64) {
        for _ in 0..(seconds / DT).round() as usize {
            self.step_once();
        }
    }

    pub fn error(&self) -> Option<&str> {
        self.error.as_deref()
    }

    pub fn numerically_ok(&self) -> bool {
        self.sim.as_ref().is_some_and(Sim::numerically_ok)
    }

    pub fn floor_force(&self) -> f64 {
        self.floor_force
    }

    /// Наибольшая сила на пол за прогон, Н.
    pub fn peak_floor_force(&self) -> f64 {
        self.peak_floor
    }

    pub fn total_weight(&self) -> f64 {
        self.sim.as_ref().map_or(f64::NAN, |s| s.total_mass() * G)
    }

    pub fn impact_speed(&self) -> Option<f64> {
        self.impact_speed
    }

    pub fn energy(&self) -> f64 {
        self.sim.as_ref().map_or(f64::NAN, |s| s.data().energy[0] + s.data().energy[1])
    }

    pub fn initial_energy(&self) -> f64 {
        self.energy0
    }

    pub fn kinetic(&self) -> f64 {
        self.sim.as_ref().map_or(f64::NAN, |s| s.data().energy[1])
    }

    fn body_id(&self, name: &str) -> Option<usize> {
        self.sim.as_ref()?.id(mjtObj::mjOBJ_BODY, name)
    }

    /// Насколько предмет сместился от начального положения, м.
    pub fn displacement(&self, name: &str) -> f64 {
        let (Some(s), Some(b)) = (self.sim.as_ref(), self.body_id(name)) else {
            return f64::NAN;
        };
        let (p, q) = (s.body_pose(b).0, self.start[b]);
        ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt()
    }

    /// Наклон предмета от вертикали, градусы.
    pub fn tilt_deg(&self, name: &str) -> f64 {
        let (Some(s), Some(b)) = (self.sim.as_ref(), self.body_id(name)) else {
            return f64::NAN;
        };
        let [_, x, y, _] = s.body_pose(b).1;
        (1.0 - 2.0 * (x * x + y * y)).clamp(-1.0, 1.0).acos().to_degrees()
    }

    pub fn weight_speed(&self) -> f64 {
        self.sim.as_ref().map_or(f64::NAN, |s| {
            let v = &s.qvel()[self.weight_dof..self.weight_dof + 3];
            (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
        })
    }

    fn energy_ok(&self) -> bool {
        let e = self.energy();
        e.is_finite() && e <= self.energy0 + (1e-3 * self.energy0.abs()).max(0.05)
    }
}

impl Scenario for Room {
    fn id(&self) -> &'static str {
        "room"
    }

    fn title(&self) -> String {
        "Комната: предметы, вес, удары".into()
    }

    fn summary(&self) -> String {
        "Стол, стул, столик и предметы со своей массой, трением и упругостью. Тяжёлое сдвигает и опрокидывает лёгкое, силы проверяются ручным расчётом.".into()
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param { id: "mass", label: "Масса груза".into(), value: self.cfg.mass_kg, min: 0.5, max: 60.0, unit: "кг", decimals: 1 },
            Param { id: "drop", label: "Высота падения на стол".into(), value: self.cfg.drop_m, min: 0.05, max: 1.5, unit: "м", decimals: 2 },
        ]
    }

    fn set_param(&mut self, id: &str, value: f64) {
        match id {
            "mass" => self.cfg.mass_kg = value,
            "drop" => self.cfg.drop_m = value,
            _ => return,
        }
        self.rebuild();
    }

    fn presets(&self) -> Vec<String> {
        vec![
            "Всё стоит спокойно".into(),
            "Гиря падает на стол".into(),
            "Гиря катится в картонную коробку".into(),
            "Груз на краю лёгкого столика".into(),
        ]
    }

    fn apply_preset(&mut self, index: usize) {
        self.cfg.preset = index.min(PRESET_TIP);
        self.rebuild();
    }

    fn reset(&mut self) {
        self.restart();
    }

    fn advance(&mut self, dt_real: f64) {
        for _ in 0..self.clock.steps(dt_real) {
            self.step_once();
        }
    }

    fn time(&self) -> f64 {
        self.sim.as_ref().map_or(0.0, Sim::time)
    }

    fn visuals(&self) -> Vec<VisualInstance> {
        let Some(s) = &self.sim else {
            return Vec::new();
        };
        self.geoms
            .iter()
            .filter(|g| g.group != 3)
            .filter_map(|g| {
                let [a, b, c] = g.size;
                let shape = match g.kind {
                    GeomKind::Sphere => Shape::Sphere { radius: a },
                    GeomKind::Cylinder => Shape::Cylinder { radius: a, length: 2.0 * b },
                    GeomKind::Box => Shape::Box { half: Vec3::new(a, b, c) },
                    _ => return None,
                };
                let (p, r) = s.geom_pose(g.id);
                Some(VisualInstance { shape, pose: mat_to_pose(p, r), color: g.rgba })
            })
            .collect()
    }

    fn visual_epoch(&self) -> u64 {
        self.epoch
    }

    fn overlays(&self) -> Vec<Overlay> {
        let Some(s) = &self.sim else {
            return Vec::new();
        };
        s.contacts()
            .iter()
            .filter(|c| c.normal_force > 1.0)
            .take(40)
            .map(|c| {
                let from = Vec3::new(c.pos[0], c.pos[1], c.pos[2]);
                let n = Vec3::new(c.normal[0], c.normal[1], c.normal[2]);
                let up = if n.z < 0.0 { -n } else { n };
                Overlay::Arrow { from, to: from + up * (c.normal_force * 0.002).min(0.5), color: C_YELLOW }
            })
            .collect()
    }

    fn readouts(&self) -> Vec<Readout> {
        vec![
            Readout::new("Точек касания", self.ncon as f64, "", 0),
            Readout::new("Сила на пол", self.floor_force, "Н", 1),
            Readout::new("Вес всего в комнате", self.total_weight(), "Н", 1),
            Readout::new("Скорость груза", self.weight_speed(), "м/с", 2),
            Readout::new("Пик силы в точке (оценка)", self.peak_force, "Н", 0),
            Readout::new("Коробка сдвинулась", self.displacement("box") * 100.0, "см", 1),
            Readout::new("Книга сдвинулась", self.displacement("book") * 100.0, "см", 1),
            Readout::new("Наклон столика", self.tilt_deg("side_table"), "°", 1),
        ]
    }

    fn comparisons(&self) -> Vec<Comparison> {
        if self.sim.is_none() {
            return Vec::new();
        }
        let mut v = Vec::new();
        let weight = self.total_weight();
        let resting = self.time() > 0.5 && self.kinetic() < 1e-3;
        v.push(Comparison {
            title: "Пол держит вес всей комнаты".into(),
            sim_label: "сумма сил в контактах с полом".into(),
            sim: self.floor_force,
            ref_label: "Σ m·g".into(),
            reference: weight,
            unit: "Н",
            decimals: 2,
            status: if !self.numerically_ok() {
                Status::Mismatch
            } else if resting {
                Status::agreement(self.floor_force, weight, 0.01)
            } else {
                Status::Pending
            },
            explanation: "Сравниваем, когда всё остановилось: в движении сила на пол другая.".into(),
        });
        v.push(Comparison {
            title: "Энергия не появляется из ниоткуда".into(),
            sim_label: "кинетическая + потенциальная".into(),
            sim: self.energy(),
            ref_label: "в начале".into(),
            reference: self.energy0,
            unit: "Дж",
            decimals: 2,
            status: if self.energy_ok() { Status::WithinLimit } else { Status::OverLimit },
            explanation: "Удары и трение могут только забирать энергию.".into(),
        });
        if self.cfg.preset == PRESET_DROP {
            let reference = (2.0 * G * self.cfg.drop_m).sqrt();
            let sim = self.impact_speed.unwrap_or(f64::NAN);
            v.push(Comparison {
                title: "Скорость груза в момент удара".into(),
                sim_label: "симуляция".into(),
                sim,
                ref_label: "√(2·g·h)".into(),
                reference,
                unit: "м/с",
                decimals: 3,
                status: if self.impact_speed.is_some() { Status::agreement(sim, reference, 0.02) } else { Status::Pending },
                explanation: "До удара груз падает свободно.".into(),
            });
        }
        if self.cfg.preset == PRESET_TIP {
            let crit = tip_mass();
            let near = (self.cfg.mass_kg / crit - 1.0).abs() < 0.15;
            let predicted = (self.cfg.mass_kg > crit) as u8 as f64;
            let tipped = (self.tilt_deg("side_table") > 10.0) as u8 as f64;
            v.push(Comparison {
                title: "Опрокинулся ли столик (1 — да, 0 — нет)".into(),
                sim_label: "симуляция".into(),
                sim: tipped,
                ref_label: format!("расчёт: порог {crit:.2} кг"),
                reference: predicted,
                unit: "",
                decimals: 0,
                status: if self.time() < 1.5 || near { Status::Pending } else { Status::agreement(tipped, predicted, 0.0) },
                explanation: "Порог: m·(x − a) = M·a. В пределах 15% от порога исход зависит от мелочей, поэтому не сравниваем.".into(),
            });
        }
        v
    }

    fn notes(&self) -> Vec<Note> {
        if let Some(e) = &self.error {
            return vec![Note::new(Level::Danger, format!("Сцена не собралась: {e}"))];
        }
        let mut n = Vec::new();
        if !self.numerically_ok() {
            n.push(Note::new(Level::Danger, "Симуляция разошлась. Числам этой сцены верить нельзя, нажмите «Сначала»."));
        }
        if self.cfg.preset == PRESET_TIP && self.tilt_deg("side_table") > 10.0 {
            n.push(Note::new(Level::Warn, format!("Столик опрокинулся: груз {:.1} кг на краю при пороге {:.2} кг.", self.cfg.mass_kg, tip_mass())));
        }
        n.push(Note::new(
            Level::Info,
            "Допущения: предметы жёсткие и не ломаются; трение и упругость материалов — справочные оценки; при контакте двух материалов MuJoCo берёт большее трение. Пик силы зависит от заданной мягкости удара, поэтому это ориентир, а не прочностной расчёт.",
        ));
        n
    }

    fn plot(&self) -> Option<Plot> {
        Some(Plot {
            title: "Сила на пол".into(),
            x_label: "t, с".into(),
            y_label: "Н".into(),
            series: vec![Series {
                name: "сила на пол".into(),
                points: self.hist.points(0, 8.0, 600),
                color: C_BLUE,
                style: SeriesStyle::Solid,
            }],
            bands: Vec::new(),
        })
    }

    fn focus(&self) -> Vec3 {
        Vec3::new(0.35, 0.05, 0.45)
    }

    fn view_offset(&self) -> Vec3 {
        Vec3::new(1.6, -4.3, 2.1)
    }
}
