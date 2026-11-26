//! Контроллер кронштейна со стороны компьютера.
//!
//! Claude пишет программу навыков (JSON в аргументах), контроллер исполняет её 250 раз в секунду:
//! обратная кинематика сферического запястья, плавные траектории, весы в руке (ATI Mini40),
//! зрение (камера глубины D405 на ладони), налив по весу с наводкой струи по модели жидкости.
//! Суставы держат угол сами (драйверы), схват — по току, то есть по силе. Протокол — stdin/stdout
//! платформы.

mod bottle;
mod geometry;
mod vision;

use std::error::Error;
use std::io;
use std::process::ExitCode;
use std::sync::Arc;

use serde::Deserialize;
use sim_control::transport::serve_worker;
use sim_control::{ControlContract, ControlError, Controller, InputChannel, InputKind, OutputChannel};
use sim_signal::{ChannelId, Command, Observation, PixelFormat, Rng, SignalId};

use bottle::Bottle;
use geometry::{Geometry, Mat, V3, add, angle_axis, apply, cross, dot, matmul, min_jerk, rot, scale, sub, transpose, unit};

const G: f64 = 9.80665;
/// Постоянная фильтра весов, с: дрожь руки даёт ±4 г на 0.1 с.
const FILTER_S: f64 = 0.25;

#[derive(Deserialize)]
struct Config {
    arm: geometry::ArmModel,
    geometry: Geometry,
    bottle: bottle::Spec,
    camera: vision::Camera,
    expect: vision::Expect,
    /// Куда записать усреднённый кадр глубины навыка look (PGM) — для отладки.
    #[serde(default)]
    dump: Option<String>,
    program: Vec<Skill>,
}

/// Кружка, как её увидела камера: центр, высота и внутренний радиус края.
#[derive(Clone, Copy)]
struct Mug {
    center: [f64; 2],
    rim_z: f64,
    radius: f64,
}

/// Система отсчёта налива: начало — ось найденной бутылки на столе, `a` — направление подхода
/// (поперёк линии «бутылка — кружка»), `toward` — к кружке, z — вверх.
#[derive(Clone, Copy)]
struct PourFrame {
    origin: [f64; 2],
    a: V3,
    toward: V3,
}

impl PourFrame {
    fn dir(&self, v: V3) -> V3 {
        add(add(scale(self.a, v[0]), scale(self.toward, v[1])), [0.0, 0.0, v[2]])
    }
    fn point(&self, p: V3) -> V3 {
        let h = self.dir([p[0], p[1], 0.0]);
        [self.origin[0] + h[0], self.origin[1] + h[1], p[2]]
    }
}

/// Этап навыка «посмотреть»: переезд в позу, затем накопление свежих кадров.
enum Look {
    Moving(Path),
    Gathering { since: f64, stack: vision::Stack },
}

#[derive(Deserialize, Clone, Debug)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
enum Skill {
    /// Разжать схват током `current` (отрицательный — к упору раскрытия).
    Open { current: f64, t: f64 },
    /// Сжать схват током `current`: сила около 42 Н на ампер.
    Grip { current: f64, t: f64 },
    /// Переезд центра захвата в точку `p` с осью подхода `approach` и осью сжатия `closing`.
    /// `joint` — по суставам, иначе по прямой в пространстве. `of = "бутылка"` — координаты и оси
    /// в системе налива от найденной бутылки (x — подход, y — к кружке, z — высота над столом).
    Move { p: V3, approach: V3, closing: V3, t: f64, #[serde(default)] joint: bool, #[serde(default)] of: Option<String> },
    /// Посмотреть на стол: центр захвата в `p` с осями `approach`/`closing`, после остановки
    /// накопить `frames` свежих кадров глубины и найти бутылку и кружку.
    Look { p: V3, approach: V3, closing: V3, t: f64, frames: usize },
    /// Запомнить показание весов без груза в текущей позе.
    Tare { t: f64 },
    /// Взвесить груз в руке: среднее за `t` секунд.
    Weigh { t: f64 },
    /// Налить `grams` граммов в кружку. Наклон к кружке вокруг оси подхода.
    Pour { grams: f64, head_mm: f64, lead_g: f64, max_s: f64 },
    Wait { t: f64 },
}

/// Траектория суставов на текущий навык.
enum Path {
    Joint { q0: [f64; 6], q1: [f64; 6], t0: f64, dur: f64 },
    Line { p0: V3, r0: Mat, p1: V3, r1: Mat, t0: f64, dur: f64 },
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum Stage {
    Approach,
    Tilt,
    Flow,
    Cut,
    Settle,
    Upright,
}

struct PourState {
    stage: Stage,
    since: f64,
    theta: f64,
    mouth: V3,
    path: Option<Path>,
    start_mass: f64,
    target: f64,
    lead: f64,
    head: f64,
    max_s: f64,
    began: f64,
    /// Скорость наклона, рад/с: наклон ведётся с ограничением ускорения, без рывков.
    omega: f64,
    /// Оценка объёма: только убывает и не быстрее предела расхода.
    volume: f64,
    lifted: bool,
    /// Центр захвата на начало отсечки: отсечка — поворот кистью вокруг него.
    cut_from: Option<V3>,
}

/// Наклон к цели с ограничением скорости `rate` и ускорения `accel`: торможение заранее.
fn approach_angle(theta: f64, omega: f64, target: f64, rate: f64, accel: f64, dt: f64) -> (f64, f64) {
    let error = target - theta;
    // Скорость, с которой ещё можно остановиться на цели.
    let stop = (2.0 * accel * error.abs()).sqrt().min(rate).copysign(error);
    let omega = omega + (stop - omega).clamp(-accel * dt, accel * dt);
    let next = theta + omega * dt;
    if (target - next) * error <= 0.0 { (target, 0.0) } else { (next, omega) }
}

struct Inputs {
    joints: [SignalId; 6],
    jaw: SignalId,
    force: [SignalId; 3],
    torque: [SignalId; 3],
    camera: SignalId,
}

struct Outputs {
    joints: [ChannelId; 6],
    gripper: ChannelId,
    mass: ChannelId,
    poured: ChannelId,
    tilt: ChannelId,
    stage: ChannelId,
    /// Что увидела камера: бутылка x, y и кружка x, y.
    seen: [ChannelId; 4],
}

struct Kent {
    arm: geometry::ArmModel,
    geometry: Geometry,
    bottle: Bottle,
    spec: bottle::Spec,
    camera: vision::Camera,
    expect: vision::Expect,
    /// Кружка и система налива — после навыка «посмотреть».
    mug: Option<Mug>,
    frame: Option<PourFrame>,
    seen: Option<vision::Found>,
    look: Option<Look>,
    /// Последний кадр глубины: момент начала экспозиции и пиксели.
    depth: Option<(u64, Arc<[u8]>)>,
    q_meas: [f64; 6],
    dump: Option<String>,
    program: Vec<Skill>,
    inputs: Inputs,
    outputs: Outputs,
    index: usize,
    since: f64,
    started: bool,
    q_cmd: [f64; 6],
    grip: f64,
    path: Option<Path>,
    /// Поза захвата для наклона и направление к кружке, запомненные в начале налива.
    carry: Option<Mat>,
    toward_mug: V3,
    /// Весы: накопление среднего, ноль без груза (мировая z) и знак.
    sum: (f64, usize),
    tare: Option<f64>,
    mass: Option<f64>,
    filtered: Option<f64>,
    pour: Option<PourState>,
    poured: f64,
    /// Разжатие с удержанием губы у упора: предел тока, А.
    open_limit: Option<f64>,
    /// Губа: прошлый отсчёт энкодера и сглаженная скорость.
    jaw_prev: Option<(f64, f64)>,
    jaw_speed: f64,
}

impl Kent {
    /// Сила датчика в мировых осях по углам энкодеров.
    fn world_force(&self, q: &[f64; 6], f: V3) -> V3 {
        let (_, r) = self.geometry.forward(q);
        apply(&r, f)
    }

    /// Масса в руке по мировой z силы: груз тянет в ту же сторону, что и вес схвата при тарировке.
    fn mass_of(&self, fz: f64) -> Option<f64> {
        let tare = self.tare?;
        Some((fz - tare) * tare.signum() / G)
    }

    /// Журнал этапов в stderr: что контроллер делает и думает. Платформа очищает окружение
    /// процесса, но stderr наследуется.
    fn note(&self, t: f64, text: &str) {
        eprintln!("контроллер {t:8.3} с, навык {}: {text}", self.index);
    }

    fn next_skill(&mut self, t: f64) {
        let what = match self.program.get(self.index) {
            Some(Skill::Weigh { .. }) => format!("взвешено {:.1} г", self.mass.unwrap_or(f64::NAN) * 1000.0),
            Some(Skill::Tare { .. }) => format!("тара {:.3} Н", self.tare.unwrap_or(f64::NAN)),
            Some(Skill::Pour { .. }) => format!("налито по весам {:.1} г", self.poured * 1000.0),
            Some(skill) => format!("готово {skill:?}").chars().take(90).collect(),
            None => String::new(),
        };
        self.note(t, &what);
        self.index += 1;
        self.since = t;
        self.path = None;
        self.sum = (0.0, 0);
    }

    /// Точка и оси навыка в мире: как есть или в системе налива от найденной бутылки.
    fn anchored(&self, of: Option<&str>, p: V3, approach: V3, closing: V3) -> Result<(V3, V3, V3), String> {
        match of {
            None => Ok((p, approach, closing)),
            Some("бутылка") => {
                let f = self.frame.ok_or("движение от бутылки до навыка look")?;
                Ok((f.point(p), f.dir(approach), f.dir(closing)))
            }
            Some(other) => Err(format!("неизвестная опора движения «{other}»")),
        }
    }

    fn follow(&mut self, path: &Path, t: f64) -> Result<bool, String> {
        match *path {
            Path::Joint { q0, q1, t0, dur } => {
                let s = min_jerk((t - t0) / dur);
                self.q_cmd = std::array::from_fn(|i| q0[i] + s * (q1[i] - q0[i]));
                Ok(t >= t0 + dur)
            }
            Path::Line { p0, r0, p1, r1, t0, dur } => {
                let s = min_jerk((t - t0) / dur);
                let p = add(p0, scale(sub(p1, p0), s));
                let (axis, angle) = angle_axis(&matmul(&transpose(&r0), &r1));
                let r = matmul(&r0, &rot(axis, angle * s));
                self.q_cmd = self.geometry.inverse(p, &r, &self.q_cmd)?;
                Ok(t >= t0 + dur)
            }
        }
    }

    /// Ось бутылки в мире при наклоне `theta` к кружке, если захват в позе `carry`.
    fn tool_at(&self, carry: &Mat, theta: f64) -> (Mat, V3) {
        let up = [carry[0][1], carry[1][1], carry[2][1]];
        let toward = self.toward(carry);
        let k = unit(cross(up, toward));
        let r = matmul(&rot(k, theta), carry);
        (r, [r[0][1], r[1][1], r[2][1]])
    }

    /// Направление от бутылки к кружке по горизонтали, запомненное в начале налива.
    fn toward(&self, _carry: &Mat) -> V3 {
        self.toward_mug
    }

    /// Углы для горла в `mouth` при наклоне `theta`.
    fn pour_pose(&self, carry: &Mat, mouth: V3, theta: f64) -> Result<[f64; 6], String> {
        let (r, axis) = self.tool_at(carry, theta);
        let tcp = sub(mouth, scale(axis, self.spec.mouth_above_grasp));
        self.geometry.inverse(tcp, &r, &self.q_cmd)
    }

    /// Горло над кружкой: смещение против хода струи и высота, при которой дно бутылки над столом.
    /// `theta` — текущий наклон (от него высота: дно бутылки над столом), `flow` — наклон, при
    /// котором струя течёт (от него снос). Горло ставится под снос заранее, до начала течения.
    fn aim(&self, carry: &Mat, theta: f64, flow: f64, volume: f64) -> V3 {
        let mug = self.mug.expect("кружка найдена до налива");
        let toward = self.toward(carry);
        let (_, axis) = self.tool_at(carry, flow);
        let lowest = self.spec.height * theta.cos() + self.spec.radius * theta.sin() + 0.015;
        let z = lowest.max(mug.rim_z + 0.03);
        let jet = self.bottle.jet(flow, volume, G);
        let drift = bottle::landing(scale(axis, jet.speed), z - mug.rim_z, G);
        let along = dot(drift, toward);
        let shift = along.clamp(0.0, mug.radius - 0.012);
        let c = [mug.center[0], mug.center[1], z];
        sub(c, scale(toward, shift))
    }

    fn pour_step(&mut self, t: f64, mass: Option<f64>) -> Result<bool, String> {
        let Some(mug) = self.mug else { return Err("налив без кружки: сначала навык look".into()) };
        if self.carry.is_none() {
            let (p, r) = self.geometry.forward(&self.q_cmd);
            let up = [r[0][1], r[1][1], r[2][1]];
            let d = [mug.center[0] - p[0], mug.center[1] - p[1], 0.0];
            self.toward_mug = unit(sub(d, scale(up, dot(d, up))));
            self.carry = Some(r);
        }
        let carry = self.carry.expect("задано");
        let Some(mass) = mass else { return Err("налив без взвешивания".into()) };
        let state = self.pour.as_mut().expect("состояние налива задано");
        let measured = ((mass - self.spec.empty_kg) / self.spec.density).max(0.0);
        // Объём меняется не быстрее 60 мл/с: рывки руки на весах срезаются. Во время переезда к
        // кружке вода не течёт, а весы видят ускорения руки — оценка держит взвешенное значение.
        let dt = 1.0 / 250.0;
        state.volume = if state.volume.is_nan() {
            ((state.start_mass - self.spec.empty_kg) / self.spec.density).max(0.0)
        } else if state.stage == Stage::Approach {
            state.volume
        } else {
            measured.clamp(state.volume - 60e-6 * dt, state.volume + 60e-6 * dt)
        };
        let volume = state.volume;
        state.target = state.target.max(0.0);
        let poured = (state.start_mass - self.spec.empty_kg) - volume * self.spec.density;
        self.poured = poured;
        let stage = state.stage;
        let (theta_now, mouth_now) = (state.theta, state.mouth);
        if (t * 250.0).round() as i64 % 125 == 0 {
            self.note(t, &format!("{stage:?}: наклон {:.1}°, горло ({:.0}, {:.0}, {:.0}) мм, в бутылке {:.0} мл, налито {:.1} г",
                theta_now.to_degrees(), mouth_now[0] * 1e3, mouth_now[1] * 1e3, mouth_now[2] * 1e3, volume * 1e6, poured * 1e3));
        }
        let state = self.pour.as_mut().expect("состояние налива задано");
        match stage {
            Stage::Approach => {
                if state.path.is_none() {
                    // Наклон до начала течения минус 8°: вода ещё не идёт. Горло сразу над точкой,
                    // откуда рассчитанная струя рабочего напора упадёт в центр кружки.
                    let onset = self.bottle.theta_for_head(volume, 0.0);
                    let theta = (onset - 10f64.to_radians()).max(0.0);
                    let flow = self.bottle.theta_for_head(volume, state.head);
                    let mouth = self.aim(&carry, theta, flow, volume);
                    let q1 = self.pour_pose(&carry, mouth, theta)?;
                    let state = self.pour.as_mut().expect("задано");
                    state.theta = theta;
                    state.mouth = mouth;
                    state.path = Some(Path::Joint { q0: self.q_cmd, q1, t0: t, dur: 3.0 });
                }
                let path = self.pour.as_mut().expect("задано").path.take().expect("путь задан");
                let done = self.follow(&path, t)?;
                let state = self.pour.as_mut().expect("задано");
                state.path = Some(path);
                if done {
                    state.stage = Stage::Tilt;
                    state.since = t;
                    state.began = t;
                    state.path = None;
                }
            }
            Stage::Tilt | Stage::Flow => {
                let (theta0, omega0, mouth0, head, target_g, lead, max_s, began) =
                    (state.theta, state.omega, state.mouth, state.head, state.target, state.lead, state.max_s, state.began);
                // Последние 20 г — напором 0.8 от рабочего: расход меньше, упреждение отсечки
                // и ошибка весов в граммах тоже. Запас до полосы медленной струи — около 2.5°.
                let head = if stage == Stage::Flow && poured > target_g - 0.020 { head * 0.8 } else { head };
                let target = self.bottle.theta_for_head(volume, head);
                // Быстро через начало течения, дальше плавно за убылью воды. Ускорение ограничено.
                // Полосу медленной струи (напор 0…3 мм, около 1.5°) проходим за ~10 мс: она бежит по горлу.
                // Пока вода течёт — только плавно: разгон бутылки вокруг горла наклоняет эффективную
                // тяжесть в воде (плеск), 1 м/с² у дна — это 6° и ±10 мм напора.
                let (rate, accel) = if stage == Stage::Tilt { (3.0, 30.0) } else { (0.1, 0.3) };
                let (theta, omega) = approach_angle(theta0, omega0, target, rate, accel, dt);
                let goal = self.aim(&carry, theta, target, volume);
                // Горло к точке наводки — сглаженно (постоянная 0.5 с), без ступенек скорости.
                let step = 0.03 * dt;
                let mouth = std::array::from_fn(|i| mouth0[i] + ((goal[i] - mouth0[i]) * dt / 0.5).clamp(-step, step));
                let reached = (target - theta).abs() < 0.5f64.to_radians();
                self.q_cmd = self.pour_pose(&carry, mouth, theta)?;
                let state = self.pour.as_mut().expect("задано");
                state.theta = theta;
                state.omega = omega;
                state.mouth = mouth;
                if stage == Stage::Tilt && reached {
                    state.stage = Stage::Flow;
                    state.since = t;
                }
                // Упреждение: отставание фильтра и вода, что вытечет за время отсечки.
                let flow = self.bottle.jet(theta, volume, G).rate * self.spec.density;
                let anticipation = lead + flow * (FILTER_S + 0.15);
                if poured >= target_g - anticipation || t - began > max_s {
                    state.stage = Stage::Cut;
                    state.since = t;
                }
            }
            Stage::Cut => {
                // Ось наклона совпадает с осью вращения кисти: отсечка — быстрый поворот кистью вокруг
                // центра захвата (AK60-6 до ~10 рад/с), полоса медленной струи проходится за миллисекунды.
                let target = self.bottle.theta_for_head(volume, -0.008);
                let (theta, omega) = approach_angle(state.theta, state.omega, target, 8.0, 150.0, dt);
                let tcp = match state.cut_from {
                    Some(tcp) => tcp,
                    None => {
                        let (tcp, _) = self.geometry.forward(&self.q_cmd);
                        state.cut_from = Some(tcp);
                        tcp
                    }
                };
                let (r, _) = self.tool_at(&carry, theta);
                self.q_cmd = self.geometry.inverse(tcp, &r, &self.q_cmd)?;
                let state = self.pour.as_mut().expect("задано");
                state.theta = theta;
                state.omega = omega;
                if (theta - target).abs() < 1e-9 {
                    state.stage = Stage::Settle;
                    state.since = t;
                }
            }
            Stage::Settle => {
                if t - state.since > 0.6 {
                    state.stage = Stage::Upright;
                    state.since = t;
                }
            }
            Stage::Upright => {
                if state.path.is_none() {
                    let lifted = state.lifted;
                    let (p, r) = self.geometry.forward(&self.q_cmd);
                    let path = if !lifted {
                        // Сначала поднять бутылку с прежним наклоном: вода не течёт, дно уходит от стола.
                        Path::Line { p0: p, r0: r, p1: add(p, [0.0, 0.0, 0.08]), r1: r, t0: t, dur: 1.2 }
                    } else {
                        // Потом выпрямить, отводя горло от кружки, центр захвата не ниже 20 см.
                        let toward = self.toward(&carry);
                        let mut tcp = sub(p, scale(toward, 0.05));
                        tcp[2] = tcp[2].max(0.20);
                        Path::Joint { q0: self.q_cmd, q1: self.geometry.inverse(tcp, &carry, &self.q_cmd)?, t0: t, dur: 2.5 }
                    };
                    let state = self.pour.as_mut().expect("задано");
                    state.path = Some(path);
                }
                let path = self.pour.as_mut().expect("задано").path.take().expect("путь задан");
                let done = self.follow(&path, t)?;
                let state = self.pour.as_mut().expect("задано");
                if !done {
                    state.path = Some(path);
                } else if !state.lifted {
                    state.lifted = true;
                } else {
                    state.theta = 0.0;
                    self.carry = None;
                    return Ok(true);
                }
            }
        }
        Ok(false)
    }

    fn run_skill(&mut self, t: f64, fz: f64) -> Result<(), String> {
        let Some(skill) = self.program.get(self.index).cloned() else { return Ok(()) };
        let elapsed = t - self.since;
        match skill {
            Skill::Open { current, t: dur } => {
                // Разжать — не в упор: упор рейки держит губу, а не ток. Губа удерживается в 2 мм от
                // упора по своему энкодеру, ток не больше заданного.
                self.open_limit = Some(current.abs());
                if elapsed >= dur {
                    self.next_skill(t);
                }
            }
            Skill::Grip { current, t: dur } => {
                self.open_limit = None;
                // Ток нарастает за 0.2 с: губа не бьёт по предмету.
                let from = self.grip;
                let s = (elapsed / 0.2).min(1.0);
                let target = from + s * (current - from);
                if elapsed >= dur {
                    self.grip = current;
                    self.next_skill(t);
                } else if s >= 1.0 {
                    self.grip = current;
                } else {
                    self.grip = target;
                }
            }
            Skill::Move { p, approach, closing, t: dur, joint, of } => {
                if self.path.is_none() {
                    let (p, approach, closing) = self.anchored(of.as_deref(), p, approach, closing)?;
                    let r1 = geometry::frame(approach, closing);
                    let (p0, r0) = self.geometry.forward(&self.q_cmd);
                    self.path = Some(if joint {
                        Path::Joint { q0: self.q_cmd, q1: self.geometry.inverse(p, &r1, &self.q_cmd)?, t0: t, dur }
                    } else {
                        Path::Line { p0, r0, p1: p, r1, t0: t, dur }
                    });
                }
                let path = self.path.take().expect("путь задан");
                let done = self.follow(&path, t)?;
                self.path = Some(path);
                if done {
                    self.next_skill(t);
                }
            }
            Skill::Tare { t: dur } | Skill::Weigh { t: dur } => {
                // Первые 0.1 с — успокоение после движения.
                if elapsed > 0.1 {
                    self.sum.0 += fz;
                    self.sum.1 += 1;
                }
                if elapsed >= dur && self.sum.1 > 0 {
                    let mean = self.sum.0 / self.sum.1 as f64;
                    if matches!(skill, Skill::Tare { .. }) {
                        self.tare = Some(mean);
                    } else {
                        self.mass = self.mass_of(mean);
                        self.filtered = self.mass;
                    }
                    self.next_skill(t);
                }
            }
            Skill::Pour { grams, head_mm, lead_g, max_s } => {
                if self.pour.is_none() {
                    let start = self.mass.ok_or("налив без взвешивания")?;
                    self.pour = Some(PourState {
                        stage: Stage::Approach,
                        since: t,
                        theta: 0.0,
                        mouth: [0.0; 3],
                        path: None,
                        start_mass: start,
                        target: grams / 1000.0,
                        lead: lead_g / 1000.0,
                        head: head_mm / 1000.0,
                        max_s,
                        began: t,
                        omega: 0.0,
                        volume: f64::NAN,
                        lifted: false,
                        cut_from: None,
                    });
                }
                if self.pour_step(t, self.filtered)? {
                    self.pour = None;
                    self.next_skill(t);
                }
            }
            Skill::Look { p, approach, closing, t: dur, frames } => match self.look.take() {
                None => {
                    let q1 = self.geometry.inverse(p, &geometry::frame(approach, closing), &self.q_cmd)?;
                    self.look = Some(Look::Moving(Path::Joint { q0: self.q_cmd, q1, t0: t, dur }));
                }
                Some(Look::Moving(path)) => {
                    self.look = Some(if self.follow(&path, t)? {
                        Look::Gathering { since: t, stack: vision::Stack::new(&self.camera) }
                    } else {
                        Look::Moving(path)
                    });
                }
                Some(Look::Gathering { since, mut stack }) => {
                    // Только кадры, экспозиция которых началась через 0.4 с после остановки.
                    let settle = ((since + 0.4) * 1e9) as u64;
                    if let Some((captured, data)) = &self.depth
                        && *captured >= settle
                        && *captured > stack.last_captured
                    {
                        stack.add(data, *captured)?;
                    }
                    if stack.frames < frames {
                        if t - since > 3.0 {
                            return Err(format!("за 3 с пришло {} свежих кадров глубины из {frames}", stack.frames));
                        }
                        self.look = Some(Look::Gathering { since, stack });
                        return Ok(());
                    }
                    if let Some(path) = &self.dump {
                        let _ = std::fs::write(path, stack.pgm(&self.camera));
                    }
                    let (tcp, r) = self.geometry.forward(&self.q_meas);
                    let found = vision::find(&stack, &self.camera, &self.expect, tcp, &r)?;
                    let toward = unit([found.mug_center[0] - found.bottle[0], found.mug_center[1] - found.bottle[1], 0.0]);
                    self.frame = Some(PourFrame { origin: found.bottle, a: [toward[1], -toward[0], 0.0], toward });
                    self.mug = Some(Mug { center: found.mug_center, rim_z: found.mug_rim, radius: found.mug_radius });
                    self.seen = Some(found);
                    self.note(t, &format!(
                        "вижу: бутылка ({:.1}, {:.1}) мм, срез горла {:.1} мм; кружка ({:.1}, {:.1}) мм, край {:.1} мм, \
                         радиус {:.1}/{:.1} мм; смещение глубины {:.2} мм по {} точкам стола, кадров {}",
                        found.bottle[0] * 1e3, found.bottle[1] * 1e3, found.bottle_top * 1e3,
                        found.mug_center[0] * 1e3, found.mug_center[1] * 1e3, found.mug_rim * 1e3,
                        found.mug_radius * 1e3, found.mug_outer * 1e3, found.bias * 1e3, found.table_points, stack.frames));
                    self.next_skill(t);
                }
            },
            Skill::Wait { t: dur } => {
                if elapsed >= dur {
                    self.next_skill(t);
                }
            }
        }
        Ok(())
    }
}

impl Controller for Kent {
    fn step(&mut self, observation: &Observation, _rng: &mut Rng) -> Result<Command, ControlError> {
        let t = observation.time.as_secs();
        let read = |id: SignalId| observation.latest(id);
        let mut q = [0.0; 6];
        for (j, id) in self.inputs.joints.iter().enumerate() {
            match read(*id) {
                Some(v) => q[j] = v,
                None => return Ok(Command { targets: vec![] }),
            }
        }
        let force: Option<Vec<f64>> = self.inputs.force.iter().map(|id| read(*id)).collect();
        let jaw = read(self.inputs.jaw).unwrap_or(0.0);
        if !self.started {
            self.started = true;
            self.q_cmd = q;
            self.since = t;
        }
        self.q_meas = q;
        if let Some(frame) = observation.frame(self.inputs.camera)
            && frame.valid
            && frame.format == PixelFormat::DepthMm16
            && self.depth.as_ref().is_none_or(|(c, _)| *c != frame.captured.0)
        {
            self.depth = Some((frame.captured.0, Arc::clone(&frame.data)));
        }
        let fz = force.map_or(0.0, |f| self.world_force(&q, [f[0], f[1], f[2]])[2]);
        // Весы: фильтр 0.1 с поверх мгновенной массы, когда груз взвешен.
        if self.mass.is_some()
            && let Some(m) = self.mass_of(fz)
        {
            let alpha = (1.0 / 250.0) / (FILTER_S + 1.0 / 250.0);
            self.filtered = Some(self.filtered.map_or(m, |f| f + alpha * (m - f)));
        }
        // Скорость губы по энкодеру, сглаживание 10 мс.
        if let Some((prev, then)) = self.jaw_prev
            && t > then
        {
            let raw = (jaw - prev) / (t - then);
            let alpha = (t - then) / (0.01 + t - then);
            self.jaw_speed += alpha * (raw - self.jaw_speed);
        }
        self.jaw_prev = Some((jaw, t));
        if let Err(e) = self.run_skill(t, fz) {
            return Err(ControlError::Execution(format!("навык {}: {e}", self.index)));
        }
        let _ = self.inputs.torque;
        // Упреждение просадки: груз — то, что показали весы в руке.
        let payload = self.filtered.unwrap_or(0.0).max(0.0);
        let command = self.arm.compensate(&self.q_cmd, payload, &self.geometry);
        let mut targets: Vec<_> = self.outputs.joints.iter().zip(command).map(|(c, v)| (*c, v)).collect();
        // Удержание губы в 2 мм от упора: ПД по энкодеру. Приведённая масса губы с ротором около
        // 8.8 кг, 42 Н на ампер: полоса 15 рад/с, затухание 0.7.
        let grip = match self.open_limit {
            Some(limit) => (47.0 * (0.002 - jaw) - 4.4 * self.jaw_speed).clamp(-limit, limit),
            None => self.grip,
        };
        self.grip = grip;
        targets.push((self.outputs.gripper, grip));
        targets.push((self.outputs.mass, self.filtered.unwrap_or(0.0)));
        targets.push((self.outputs.poured, self.poured));
        targets.push((self.outputs.tilt, self.pour.as_ref().map_or(0.0, |p| p.theta)));
        targets.push((self.outputs.stage, self.index as f64));
        let seen = self.seen.map_or([0.0; 4], |f| [f.bottle[0], f.bottle[1], f.mug_center[0], f.mug_center[1]]);
        targets.extend(self.outputs.seen.iter().zip(seen).map(|(c, v)| (*c, v)));
        Ok(Command { targets })
    }
}

struct Args(std::vec::IntoIter<String>);

impl Args {
    fn take(&mut self, what: &str) -> Result<String, String> {
        self.0.next().ok_or_else(|| format!("нет параметра: {what}"))
    }
    fn num<T: std::str::FromStr>(&mut self, what: &str) -> Result<T, String> {
        self.take(what)?.parse().map_err(|_| format!("неверный параметр: {what}"))
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let mut a = Args(std::env::args().skip(1).collect::<Vec<_>>().into_iter());
    // Сверка модели без платформы: --gravity <программа JSON> q1..q6 груз.
    if std::env::args().nth(1).as_deref() == Some("--gravity") {
        a.take("ключ")?;
        let config: Config = serde_json::from_str(&a.take("программа JSON")?)?;
        let mut q = [0.0; 6];
        for v in &mut q {
            *v = a.num("угол")?;
        }
        let payload: f64 = a.num("груз")?;
        println!("{:?}", config.arm.gravity(&q, payload, &config.geometry));
        return Ok(());
    }
    let limit: u32 = a.num("размер сообщения")?;
    let in_age: u64 = a.num("срок входа")?;
    let out_age: u64 = a.num("срок выхода")?;
    let mut channels = Vec::new();
    let mut ids = Vec::new();
    // 6 энкодеров, энкодер схвата, 3 силы, 3 момента.
    for _ in 0..13 {
        let id = SignalId(a.num("номер входа")?);
        ids.push(id);
        channels.push(InputChannel {
            id,
            name: a.take("имя входа")?,
            unit: a.take("единица входа")?,
            kind: InputKind::Scalar { max_age_ns: in_age },
        });
    }
    // Камера глубины: номер, имя, единица, сроки экспозиции и доставки.
    let camera = SignalId(a.num("номер камеры")?);
    channels.push(InputChannel {
        id: camera,
        name: a.take("имя камеры")?,
        unit: a.take("единица камеры")?,
        kind: InputKind::Image {
            format: PixelFormat::DepthMm16,
            max_capture_age_ns: a.num("срок экспозиции")?,
            max_delivery_age_ns: a.num("срок доставки")?,
        },
    });
    let mut outputs = Vec::new();
    let mut out_ids = Vec::new();
    // 6 суставов, схват, 4 телеметрии, 4 увиденных координаты.
    for _ in 0..15 {
        let id = ChannelId(a.num("номер выхода")?);
        out_ids.push(id);
        outputs.push(OutputChannel { id, name: a.take("имя выхода")?, unit: a.take("единица выхода")?, max_age_ns: out_age });
    }
    let config: Config = serde_json::from_str(&a.take("программа JSON")?)?;
    if a.0.next().is_some() {
        return Err("лишние параметры".into());
    }
    let bottle = Bottle::new(&config.bottle);
    let contract = ControlContract::new(channels, outputs)?;
    let kent = Kent {
        arm: config.arm,
        geometry: config.geometry,
        bottle,
        spec: config.bottle,
        camera: config.camera,
        expect: config.expect,
        mug: None,
        frame: None,
        seen: None,
        look: None,
        depth: None,
        q_meas: [0.0; 6],
        dump: config.dump,
        program: config.program,
        inputs: Inputs {
            joints: std::array::from_fn(|i| ids[i]),
            jaw: ids[6],
            force: [ids[7], ids[8], ids[9]],
            torque: [ids[10], ids[11], ids[12]],
            camera,
        },
        outputs: Outputs {
            joints: std::array::from_fn(|i| out_ids[i]),
            gripper: out_ids[6],
            mass: out_ids[7],
            poured: out_ids[8],
            tilt: out_ids[9],
            stage: out_ids[10],
            seen: [out_ids[11], out_ids[12], out_ids[13], out_ids[14]],
        },
        index: 0,
        since: 0.0,
        started: false,
        q_cmd: [0.0; 6],
        grip: 0.0,
        path: None,
        carry: None,
        toward_mug: [0.0; 3],
        sum: (0.0, 0),
        tare: None,
        mass: None,
        filtered: None,
        pour: None,
        poured: 0.0,
        open_limit: None,
        jaw_prev: None,
        jaw_speed: 0.0,
    };
    serve_worker(io::stdin().lock(), io::stdout().lock(), &contract, limit, kent)
        .map_err(|e: ControlError| -> Box<dyn Error> { e.to_string().into() })
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("{e}");
            ExitCode::FAILURE
        }
    }
}
