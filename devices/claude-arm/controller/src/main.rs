//! Контроллер руки Claude со стороны компьютера: обратная кинематика по точкам
//! в координатах стола и плавная траектория суставов. Сервы держат позицию сами,
//! контроллер шлёт им задания углов по шине. Протокол — stdin/stdout платформы.

use std::error::Error;
use std::f64::consts::PI;
use std::io;
use std::process::ExitCode;

use sim_control::transport::serve_worker;
use sim_control::{ControlContract, ControlError, Controller, InputChannel, InputKind, OutputChannel};
use sim_signal::{ChannelId, Command, Observation, Rng, SignalId};

const JOINTS: usize = 5;

/// Геометрия эскиза: ось плеча над началом, длины звеньев, вылет от оси запястья до точки инструмента.
#[derive(Clone, Copy)]
struct Geometry {
    h0: f64,
    a1: f64,
    a2: f64,
    d: f64,
    limits: [[f64; 2]; JOINTS],
}

#[derive(Clone, Copy, Debug)]
struct Waypoint {
    t: f64,
    p: [f64; 3],
    /// Угол оси инструмента от вертикали: 0 — вверх, 90° — вперёд, 180° — вниз.
    tilt: f64,
    roll: f64,
    /// Ход губы схвата в метрах или команда замка насадки 0/1, если шестой выход подключён.
    grip: f64,
}

impl Waypoint {
    /// Точка на отрезке до `next` в доле `s` пути.
    fn towards(&self, next: &Waypoint, s: f64) -> Waypoint {
        let mix = |a: f64, b: f64| a + s * (b - a);
        Waypoint {
            t: mix(self.t, next.t),
            p: std::array::from_fn(|i| mix(self.p[i], next.p[i])),
            tilt: mix(self.tilt, next.tilt),
            roll: mix(self.roll, next.roll),
            grip: self.grip,
        }
    }
}

impl Geometry {
    fn forward(&self, q: &[f64; JOINTS]) -> [f64; 3] {
        let (a1, a2, a3) = (q[1], q[1] + q[2], q[1] + q[2] + q[3]);
        let r = self.a1 * a1.sin() + self.a2 * a2.sin() + self.d * a3.sin();
        let z = self.h0 + self.a1 * a1.cos() + self.a2 * a2.cos() + self.d * a3.cos();
        [r * q[0].cos(), r * q[0].sin(), z]
    }

    fn inverse(&self, w: &Waypoint) -> Result<[f64; JOINTS], String> {
        let [x, y, z] = w.p;
        let q0 = y.atan2(x);
        let r = x.hypot(y);
        let wr = r - self.d * w.tilt.sin();
        let wz = z - self.d * w.tilt.cos() - self.h0;
        let c2 = (wr * wr + wz * wz - self.a1 * self.a1 - self.a2 * self.a2) / (2.0 * self.a1 * self.a2);
        if !(-1.0..=1.0).contains(&c2) {
            return Err(format!("точка {:?} вне досягаемости: cos(локоть)={c2:.4}", w.p));
        }
        let q2 = c2.acos();
        let q1 = wr.atan2(wz) - (self.a2 * q2.sin()).atan2(self.a1 + self.a2 * q2.cos());
        let q3 = w.tilt - q1 - q2;
        let q = [q0, q1, q2, q3, w.roll];
        for (i, (v, [lo, hi])) in q.iter().zip(self.limits).enumerate() {
            if *v < lo - 1e-9 || *v > hi + 1e-9 {
                return Err(format!(
                    "точка {:?}: сустав {i} = {:.2}° вне [{:.1}°, {:.1}°]",
                    w.p,
                    v.to_degrees(),
                    lo.to_degrees(),
                    hi.to_degrees()
                ));
            }
        }
        let back = self.forward(&q);
        let err = (0..3).map(|i| (back[i] - w.p[i]).powi(2)).sum::<f64>().sqrt();
        if err > 1e-9 {
            return Err(format!("невязка прямой задачи {err:e} м"));
        }
        Ok(q)
    }
}

/// Шестой выход: схват ведётся плавно, замок насадки переключается ступенью.
#[derive(Clone, Copy, PartialEq)]
enum Tool {
    Gripper,
    Lock,
}

struct Arm {
    inputs: [SignalId; JOINTS],
    outputs: [ChannelId; JOINTS],
    /// Шестой выход: ход губы или замок. Начально открыто, 0.
    gripper: Option<(ChannelId, Tool)>,
    plan: Vec<(f64, [f64; JOINTS], f64)>,
    start: Option<(f64, [f64; JOINTS])>,
    /// Режим «прямые»: между точками фланец идёт по отрезку в координатах стола.
    /// Первый переезд из стартовой позы всегда идёт по суставам.
    line: Option<(Geometry, Vec<Waypoint>)>,
}

fn min_jerk(s: f64) -> f64 {
    let s = s.clamp(0.0, 1.0);
    s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)
}

impl Arm {
    fn target(&self, t: f64) -> ([f64; JOINTS], f64) {
        let (t0, q0) = self.start.expect("старт задан");
        let step = matches!(self.gripper, Some((_, Tool::Lock)));
        let mut prev = (t0, q0, 0.0);
        for (i, &(ti, qi, gi)) in self.plan.iter().enumerate() {
            if t < ti {
                let s = min_jerk((t - prev.0) / (ti - prev.0));
                let joints = std::array::from_fn(|j| prev.1[j] + s * (qi[j] - prev.1[j]));
                let q = match &self.line {
                    Some((g, points)) if i > 0 => g
                        .inverse(&points[i - 1].towards(&points[i], s))
                        // Отрезки проверены при запуске с шагом 1 мс; между отсчётами — по суставам.
                        .unwrap_or(joints),
                    _ => joints,
                };
                return (
                    q,
                    // Замок держит команду прошлой точки и переключается в момент новой.
                    if step { prev.2 } else { prev.2 + s * (gi - prev.2) },
                );
            }
            prev = (ti, qi, gi);
        }
        (prev.1, prev.2)
    }
}

impl Controller for Arm {
    fn step(&mut self, observation: &Observation, _rng: &mut Rng) -> Result<Command, ControlError> {
        let t = observation.time.as_secs();
        if self.start.is_none() {
            let mut q = [0.0; JOINTS];
            for (j, id) in self.inputs.iter().enumerate() {
                match observation.latest(*id) {
                    Some(v) => q[j] = v,
                    // Без показаний всех энкодеров команды не шлём.
                    None => return Ok(Command { targets: vec![] }),
                }
            }
            self.start = Some((t, q));
        }
        let (q, grip) = self.target(t);
        let mut targets: Vec<_> = self.outputs.iter().zip(q).map(|(c, v)| (*c, v)).collect();
        if let Some((channel, _)) = self.gripper {
            targets.push((channel, grip));
        }
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
    let limit: u32 = a.num("размер сообщения")?;
    let in_age: u64 = a.num("срок входа")?;
    let out_age: u64 = a.num("срок выхода")?;
    let mut inputs = Vec::new();
    let mut in_ids = [SignalId(0); JOINTS];
    for id in &mut in_ids {
        *id = SignalId(a.num("номер входа")?);
        inputs.push(InputChannel {
            id: *id,
            name: a.take("имя входа")?,
            unit: a.take("единица входа")?,
            kind: InputKind::Scalar { max_age_ns: in_age },
        });
    }
    let mut outputs = Vec::new();
    let mut out_ids = [ChannelId(0); JOINTS];
    for id in &mut out_ids {
        *id = ChannelId(a.num("номер выхода")?);
        outputs.push(OutputChannel {
            id: *id,
            name: a.take("имя выхода")?,
            unit: a.take("единица выхода")?,
            max_age_ns: out_age,
        });
    }
    // Необязательный шестой выход: 1 — схват, 2 — замок сменщика насадок.
    let gripper = match a.num::<u8>("шестой выход 0, 1 или 2")? {
        0 => None,
        kind @ (1 | 2) => {
            let id = ChannelId(a.num("номер выхода схвата")?);
            outputs.push(OutputChannel {
                id,
                name: a.take("имя выхода схвата")?,
                unit: a.take("единица выхода схвата")?,
                max_age_ns: out_age,
            });
            Some((id, if kind == 1 { Tool::Gripper } else { Tool::Lock }))
        }
        _ => return Err("шестой выход: 0, 1 или 2".into()),
    };
    let mut g = Geometry {
        h0: a.num("h0")?,
        a1: a.num("a1")?,
        a2: a.num("a2")?,
        d: a.num("d")?,
        limits: [[0.0; 2]; JOINTS],
    };
    for l in &mut g.limits {
        *l = [a.num::<f64>("предел")?.to_radians(), a.num::<f64>("предел")?.to_radians()];
    }
    let n: usize = a.num("число точек")?;
    let mut plan = Vec::new();
    let mut points = Vec::new();
    let mut last = 0.0;
    for _ in 0..n {
        let w = Waypoint {
            t: a.num("t")?,
            p: [a.num("x")?, a.num("y")?, a.num("z")?],
            tilt: a.num::<f64>("наклон")?.to_radians(),
            roll: a.num::<f64>("вращение")?.to_radians(),
            grip: if gripper.is_some() { a.num("ход губы или замок")? } else { 0.0 },
        };
        if matches!(gripper, Some((_, Tool::Lock))) && w.grip != 0.0 && w.grip != 1.0 {
            return Err("команда замка в точке: 0 или 1".into());
        }
        if w.t <= last {
            return Err("времена точек должны возрастать".into());
        }
        last = w.t;
        plan.push((w.t, g.inverse(&w)?, w.grip));
        points.push(w);
    }
    let line = match a.0.next().as_deref() {
        None => None,
        Some("прямые") => {
            for pair in points.windows(2) {
                let steps = ((pair[1].t - pair[0].t) / 0.001).ceil() as usize;
                for k in 1..steps {
                    g.inverse(&pair[0].towards(&pair[1], k as f64 / steps as f64))?;
                }
            }
            Some((g, points))
        }
        Some(_) => return Err("после точек допускается только «прямые»".into()),
    };
    if a.0.next().is_some() {
        return Err("лишние параметры".into());
    }
    let _ = PI;
    let contract = ControlContract::new(inputs, outputs)?;
    let arm = Arm { inputs: in_ids, outputs: out_ids, gripper, plan, start: None, line };
    serve_worker(io::stdin().lock(), io::stdout().lock(), &contract, limit, arm)
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
