//! Жидкость в осесимметричных сосудах: объём, свободная поверхность, слив через горло,
//! струя и пролив.
//!
//! Модель квазистатическая. Свободная поверхность — плоскость, перпендикулярная эффективной
//! тяжести g − a, где a — ускорение сосуда после фильтра первого порядка с постоянной первой
//! моды плеска. Через залитую часть среза горла течёт по Торричелли с коэффициентом расхода.
//! При полностью залитом узком горле воздух входит толчками, расход ограничен паспортом сосуда.
//! Струя летит баллистически от центра залитой части среза вдоль оси сосуда. Если её скорость
//! ниже скорости отрыва от кромки, она обтекает кромку (эффект чайника): у опущенного горла
//! капает с нижней точки кромки, у поднятого бежит по горлу назад и считается пролитой.
//! Струя или капли, пересекающие плоскость среза стоящего сосуда внутри его кромки,
//! попадает в него. Волны, брызги, капли на стенках и инерция перетекающей жидкости не
//! посчитаны: это записано в упрощениях задания.

use std::{collections::BTreeSet, f64::consts::PI, sync::Arc};

use sim_process::BodyLoad;

use super::*;

/// Наклонов в таблице поверхности: шаг 1°.
const TILTS: usize = 180;
/// Ячеек высоты поверхности на один наклон.
const BINS: usize = 1000;
/// Шаг выборки полости, м.
const STEP_M: f64 = 0.0005;
/// Полос по срезу горла при интегрировании расхода.
const MOUTH: usize = 64;
/// Сосуд принимает струю, если его ось отклонена от вертикали не больше чем на 30°.
const UPRIGHT: f64 = 0.866_025_403_784_438_6;

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(in crate::tasks::processes) struct Vessel {
    /// Тело сосуда.
    body: String,
    /// Центр дна полости в системе тела.
    bottom_m: [f64; 3],
    /// Ось полости от дна к срезу горла в системе тела.
    axis: [f64; 3],
    /// Внутренний профиль: пары (высота от дна, радиус) по возрастанию высоты, первая на дне,
    /// последняя — срез горла.
    profile_m: Vec<[f64; 2]>,
    /// Начальный объём, мл.
    initial_ml: f64,
    /// Масса начальной жидкости уже входит в массу тела, неподвижно в центре полного
    /// стоящего сосуда. Процесс тогда прикладывает разницу с фактической жидкостью.
    #[serde(default)]
    rigid_initial: bool,
    /// Предел расхода при полностью залитом срезе горла, мл/с. Без него горло широкое и
    /// воздух входит свободно.
    glug_ml_s: Option<Scalar>,
    /// Сосуд закреплён в мире: вес жидкости на него не прикладывается.
    #[serde(default)]
    fixed: bool,
}

/// Цель налива: к моменту `at_ns` в сосуде `vessel` должно быть `ml ± tolerance_ml`.
/// С этого момента процесс публикует событие `<имя>/цель`: 1 — цель не достигнута.
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(in crate::tasks::processes) struct Goal {
    vessel: String,
    ml: f64,
    tolerance_ml: f64,
    at_ns: u64,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(in crate::tasks::processes) struct Fluid {
    density_kg_m3: Scalar,
    /// Коэффициент расхода через срез горла.
    discharge: Scalar,
    /// Скорость, ниже которой струя не отрывается от кромки, м/с.
    detach_m_s: Scalar,
    /// Постоянная фильтра ускорения сосуда, с: первая мода плеска.
    slosh_s: Scalar,
}

fn radius(profile: &[[f64; 2]], z: f64) -> f64 {
    let i = profile.partition_point(|p| p[0] <= z).clamp(1, profile.len() - 1);
    let ([z0, r0], [z1, r1]) = (profile[i - 1], profile[i]);
    r0 + (r1 - r0) * ((z - z0) / (z1 - z0)).clamp(0.0, 1.0)
}

/// Накопленные объём и моменты жидкости ниже высоты s0 + k·ds для одного наклона.
struct Row {
    s0: f64,
    ds: f64,
    /// [объём, момент по x наклона, момент по оси] ниже границы ячейки; длина BINS + 1.
    cum: Vec<[f64; 3]>,
}

impl Row {
    /// Высота поверхности и центр жидкости (x вдоль направления наклона, z вдоль оси).
    /// Сверх полного объёма поверхность поднимается над срезом на излишек, делённый на `top`.
    fn level(&self, volume: f64, top: f64) -> (f64, [f64; 2]) {
        let full = self.cum[BINS];
        let centroid = |m: [f64; 3]| if m[0] > 0.0 { [m[1] / m[0], m[2] / m[0]] } else { [0.0; 2] };
        if volume >= full[0] {
            return (self.s0 + BINS as f64 * self.ds + (volume - full[0]) / top, centroid(full));
        }
        let volume = volume.max(0.0);
        let i = (self.cum.partition_point(|c| c[0] <= volume) - 1).min(BINS - 1);
        let (lo, hi) = (self.cum[i], self.cum[i + 1]);
        let f = if hi[0] > lo[0] { (volume - lo[0]) / (hi[0] - lo[0]) } else { 0.0 };
        let m = [0, 1, 2].map(|k| lo[k] + f * (hi[k] - lo[k]));
        (self.s0 + (i as f64 + f) * self.ds, centroid(m))
    }
}

/// Таблица поверхности сосуда по наклону оси от вертикали, шаг 1°.
pub(super) struct Table {
    rows: Vec<Row>,
    capacity_m3: f64,
    height_m: f64,
    mouth_m: f64,
}

impl Table {
    fn new(profile: &[[f64; 2]]) -> Self {
        let height_m = profile.last().expect("профиль проверен")[0];
        let nz = (height_m / STEP_M).ceil() as usize;
        let dz = height_m / nz as f64;
        // Слой полости — круг; по y он интегрируется хордой, высота поверхности от y не зависит.
        let mut samples = Vec::new();
        for i in 0..nz {
            let z = (i as f64 + 0.5) * dz;
            let r = radius(profile, z);
            let nx = ((2.0 * r / STEP_M).ceil() as usize).max(16);
            let dx = 2.0 * r / nx as f64;
            for j in 0..nx {
                let x = -r + (j as f64 + 0.5) * dx;
                let chord = 2.0 * (r * r - x * x).max(0.0).sqrt();
                samples.push((x, z, chord * dx * dz, dx));
            }
        }
        let capacity_m3 = samples.iter().map(|s| s.2).sum();
        let dz_half = dz / 2.0;
        let rows = (0..=TILTS)
            .map(|k| {
                let (sin, cos) = (PI * k as f64 / TILTS as f64).sin_cos();
                // Ячейка выборки занимает по высоте середину ± полуразмах; объём ячейки
                // распределяется равномерно по этому отрезку, иначе уровень ступенчатый.
                let span = |&(x, z, _, dx): &(f64, f64, f64, f64)| {
                    let half = 0.5 * dx * sin.abs() + dz_half * cos.abs();
                    (x * sin + z * cos - half, x * sin + z * cos + half)
                };
                let (min, max) = samples.iter().map(span).fold((f64::MAX, f64::MIN), |(a, b), (lo, hi)| (a.min(lo), b.max(hi)));
                let s0 = min;
                let ds = (max - s0) / BINS as f64;
                let mut bins = vec![[0.0; 3]; BINS];
                for sample in &samples {
                    let (x, z, dv, _) = *sample;
                    let (lo, hi) = span(sample);
                    let (a, b) = ((lo - s0) / ds, (hi - s0) / ds);
                    let first = (a.floor().max(0.0) as usize).min(BINS - 1);
                    let last = (b.ceil() as usize).clamp(first + 1, BINS);
                    for (k, bin) in bins.iter_mut().enumerate().take(last).skip(first) {
                        let overlap = (b.min(k as f64 + 1.0) - a.max(k as f64)).max(0.0) / (b - a);
                        let part = dv * overlap;
                        bin[0] += part;
                        bin[1] += part * x;
                        bin[2] += part * z;
                    }
                }
                let mut cum = Vec::with_capacity(BINS + 1);
                let mut acc = [0.0; 3];
                cum.push(acc);
                for bin in bins {
                    acc = [acc[0] + bin[0], acc[1] + bin[1], acc[2] + bin[2]];
                    cum.push(acc);
                }
                Row { s0, ds, cum }
            })
            .collect();
        let mouth_m = profile.last().expect("профиль проверен")[1];
        Self { rows, capacity_m3, height_m, mouth_m }
    }

    /// Высота поверхности вдоль направления «вверх» от центра дна и центр жидкости
    /// (x вдоль наклона, z вдоль оси) для наклона `theta` оси от вертикали.
    fn level(&self, theta: f64, volume: f64) -> (f64, [f64; 2]) {
        let t = (theta / PI * TILTS as f64).clamp(0.0, TILTS as f64);
        let k = (t.floor() as usize).min(TILTS - 1);
        let f = t - k as f64;
        let top = PI * self.mouth_m * self.mouth_m;
        let (sa, ca) = self.rows[k].level(volume, top);
        let (sb, cb) = self.rows[k + 1].level(volume, top);
        (sa + f * (sb - sa), [ca[0] + f * (cb[0] - ca[0]), ca[1] + f * (cb[1] - ca[1])])
    }
}

/// Слив через срез горла при наклоне `theta` и поверхности `surface`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Outflow {
    rate_m3_s: f64,
    area_m2: f64,
    /// Центр залитой части среза вдоль направления наклона, м.
    x_m: f64,
    /// Средняя скорость в срезе, м/с.
    speed_m_s: f64,
}

fn outflow(table: &Table, theta: f64, surface: f64, g: f64, discharge: f64, glug: Option<f64>) -> Outflow {
    let (sin, cos) = theta.sin_cos();
    let r = table.mouth_m;
    let dx = 2.0 * r / MOUTH as f64;
    let (mut rate, mut area, mut moment, mut dry) = (0.0, 0.0, 0.0, false);
    for j in 0..MOUTH {
        let x = -r + (j as f64 + 0.5) * dx;
        let head = surface - (x * sin + table.height_m * cos);
        if head <= 0.0 {
            dry = true;
            continue;
        }
        let da = 2.0 * (r * r - x * x).max(0.0).sqrt() * dx;
        rate += discharge * (2.0 * g * head).sqrt() * da;
        area += da;
        moment += x * da;
    }
    if area == 0.0 {
        return Outflow::default();
    }
    if !dry && let Some(limit) = glug {
        rate = rate.min(limit);
    }
    Outflow { rate_m3_s: rate, area_m2: area, x_m: moment / area, speed_m_s: rate / (discharge * area) }
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn add(a: [f64; 3], b: [f64; 3], k: f64) -> [f64; 3] {
    [a[0] + k * b[0], a[1] + k * b[1], a[2] + k * b[2]]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

/// Попадает ли струя из `start` со скоростью `velocity` в срез радиуса `radius` с центром `rim`.
/// Если струя начинается ниже плоскости среза, она попадает, только если начало внутри
/// цилиндра сосуда над его дном (`depth` — глубина полости).
fn lands(start: [f64; 3], velocity: [f64; 3], gravity: [f64; 3], rim: [f64; 3], radius: f64, depth: f64) -> bool {
    let g = norm(gravity);
    let up = gravity.map(|c| -c / g);
    let offset = add(start, rim, -1.0);
    let h0 = dot(offset, up);
    let horizontal = |v: [f64; 3]| norm(add(v, up, -dot(v, up)));
    if h0 <= 0.0 {
        return h0 > -depth && horizontal(offset) < radius;
    }
    let vu = dot(velocity, up);
    let t = (vu + (vu * vu + 2.0 * g * h0).sqrt()) / g;
    let at = add(add(offset, velocity, t), gravity, 0.5 * t * t);
    horizontal(at) <= radius
}

#[derive(Clone)]
struct Model {
    name: String,
    body: String,
    bottom: [f64; 3],
    axis: [f64; 3],
    table: Arc<Table>,
    glug: Option<f64>,
    /// Масса и точка в системе тела начальной жидкости, уже входящей в тело.
    rigid: Option<(f64, [f64; 3])>,
    fixed: bool,
    volume: f64,
    velocity: Option<([f64; 3], Time)>,
    acceleration: [f64; 3],
}

/// Поза сосуда на границе: поворот тела в мир (строки) и точка дна.
struct Pose {
    rotation: [[f64; 3]; 3],
    bottom: [f64; 3],
    com: [f64; 3],
}

impl Pose {
    fn vector(&self, local: [f64; 3]) -> [f64; 3] {
        self.rotation.map(|row| dot(row, local))
    }

    fn local(&self, world: [f64; 3]) -> [f64; 3] {
        [0, 1, 2].map(|j| (0..3).map(|i| self.rotation[i][j] * world[i]).sum())
    }

    fn point(&self, bottom_local: [f64; 3], offset_local: [f64; 3]) -> [f64; 3] {
        add(self.bottom, self.vector(add(offset_local, bottom_local, -1.0)), 1.0)
    }
}

#[derive(Clone)]
struct Liquid {
    name: String,
    vessels: Vec<Model>,
    density: f64,
    discharge: f64,
    detach: f64,
    slosh: f64,
    gravity: [f64; 3],
    spilled_m3: f64,
    goal: Option<(usize, f64, f64, Time)>,
}

struct Snapshot {
    pose: Pose,
    /// Ось и направление наклона в системе тела.
    tilt: ([f64; 3], [f64; 3]),
    flow: Outflow,
    /// Центр жидкости в мире.
    centroid: [f64; 3],
    mouth_velocity: [f64; 3],
    /// Эффективная тяжесть g − a в сосуде.
    effective: [f64; 3],
}

impl Liquid {
    fn snapshot(&mut self, frame: &ProcessFrame<'_>, index: usize) -> Result<Snapshot, ProcessError> {
        let at = frame.next.start;
        let vessel = &mut self.vessels[index];
        let base = frame.body_point(&vessel.body, vessel.bottom)?;
        let height = vessel.table.height_m;
        let mouth_local = add(vessel.bottom, vessel.axis, height);
        let mouth = frame.body_point(&vessel.body, mouth_local)?;
        if let Some((previous, then)) = vessel.velocity {
            let h = at.as_secs() - then.as_secs();
            if h > 0.0 {
                let raw = add(base.velocity_m_s, previous, -1.0).map(|c| c / h);
                let alpha = h / (self.slosh + h);
                vessel.acceleration = add(vessel.acceleration, add(raw, vessel.acceleration, -1.0), alpha);
            }
        }
        vessel.velocity = Some((base.velocity_m_s, at));
        let pose = Pose { rotation: base.rotation, bottom: base.position_m, com: base.com_m };
        let effective = add(self.gravity, vessel.acceleration, -1.0);
        let g = norm(effective);
        if !g.is_finite() {
            return Err(ProcessError::Execution(format!("неконечное ускорение сосуда «{}»", vessel.name)));
        }
        // В невесомости поверхность не определена: жидкость стоит, центр — как у стоящего сосуда.
        let weightless = g < 1e-6 * norm(self.gravity);
        let up = if weightless { vessel.axis } else { pose.local(effective.map(|c| -c / g)) };
        let cos = dot(up, vessel.axis).clamp(-1.0, 1.0);
        let theta = cos.acos();
        let side = add(up, vessel.axis, -cos);
        let tilt = if norm(side) > 1e-9 {
            side.map(|c| c / norm(side))
        } else {
            // Ось вертикальна: направление наклона любое перпендикулярное.
            let helper = if vessel.axis[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
            let side = add(helper, vessel.axis, -dot(helper, vessel.axis));
            side.map(|c| c / norm(side))
        };
        let (surface, [x, z]) = vessel.table.level(theta, vessel.volume);
        let flow = if weightless { Outflow::default() } else { outflow(&vessel.table, theta, surface, g, self.discharge, vessel.glug) };
        let centroid_local = add(add(vessel.bottom, tilt, x), vessel.axis, z);
        let centroid = pose.point(vessel.bottom, centroid_local);
        Ok(Snapshot { pose, tilt: (vessel.axis, tilt), flow, centroid, mouth_velocity: mouth.velocity_m_s, effective })
    }
}

impl ProcessModel for Liquid {
    fn state_fingerprint(&self) -> u64 {
        let state: Vec<_> = self
            .vessels
            .iter()
            .map(|v| (v.volume.to_bits(), v.velocity.map(|(u, t)| (u.map(f64::to_bits), t)), v.acceleration.map(f64::to_bits)))
            .collect();
        sim_signal::fnv1a(format!("{state:?}{}", self.spilled_m3.to_bits()).bytes())
    }

    fn update(&mut self, frame: &ProcessFrame<'_>, _: &mut Rng) -> Result<ProcessOut, ProcessError> {
        let at = frame.next.start;
        let dt = frame.next.end.as_secs() - at.as_secs();
        let snapshots = (0..self.vessels.len()).map(|i| self.snapshot(frame, i)).collect::<Result<Vec<_>, _>>()?;
        let mut out = ProcessOut::default();
        let mut total_rate = 0.0;
        // Нагрузки по жидкости на начало интервала. Жидкость движется вместе с сосудом и давит
        // на него силой m(g − a); жёсткая начальная жидкость тела вычитается той же силой.
        for (vessel, snap) in self.vessels.iter().zip(&snapshots).filter(|(v, _)| !v.fixed) {
            let weight = snap.effective.map(|c| c * self.density * vessel.volume);
            let mut force = weight;
            let mut torque = cross(add(snap.centroid, snap.pose.com, -1.0), weight);
            if let Some((mass, local)) = vessel.rigid {
                let built_in = snap.effective.map(|c| -c * mass);
                let point = snap.pose.point(vessel.bottom, local);
                force = add(force, built_in, 1.0);
                torque = add(torque, cross(add(point, snap.pose.com, -1.0), built_in), 1.0);
            }
            out.body_loads.push(BodyLoad { body: vessel.body.clone(), force_n: force, torque_nm: torque });
        }
        // Перелив за следующий интервал по состоянию на его начало.
        let mut delta = vec![0.0; self.vessels.len()];
        for (i, snap) in snapshots.iter().enumerate() {
            let flow = snap.flow;
            if flow.rate_m3_s <= 0.0 {
                continue;
            }
            let source = &self.vessels[i];
            let amount = (flow.rate_m3_s * dt).min(source.volume + delta[i]).max(0.0);
            total_rate += flow.rate_m3_s;
            delta[i] -= amount;
            let (axis, tilt) = snap.tilt;
            let up = self.gravity.map(|c| -c / norm(self.gravity));
            let neck = add(source.bottom, axis, source.table.height_m);
            // Быстрая струя отрывается от среза и летит вдоль оси. Медленная обтекает кромку:
            // у опущенного горла капает с нижней точки кромки, у поднятого бежит по горлу назад.
            let fall = if flow.speed_m_s >= self.detach {
                let start = snap.pose.point(source.bottom, add(neck, tilt, flow.x_m));
                let velocity = add(snap.mouth_velocity, snap.pose.vector(axis), flow.speed_m_s);
                Some((start, velocity, (flow.area_m2 / PI).sqrt()))
            } else if dot(snap.pose.vector(axis), up) < 0.0 {
                let lip = add(neck, tilt, -source.table.mouth_m);
                Some((snap.pose.point(source.bottom, lip), snap.mouth_velocity, 0.0))
            } else {
                None
            };
            let target = fall.and_then(|(start, velocity, jet)| {
                snapshots.iter().enumerate().find(|&(j, other)| {
                    if j == i {
                        return false;
                    }
                    let vessel = &self.vessels[j];
                    let axis = other.pose.vector(vessel.axis);
                    let rim = other.pose.point(vessel.bottom, add(vessel.bottom, vessel.axis, vessel.table.height_m));
                    dot(axis, up) >= UPRIGHT
                        && lands(start, velocity, self.gravity, rim, vessel.table.mouth_m - jet, vessel.table.height_m)
                })
            });
            match target {
                Some((j, _)) => delta[j] += amount,
                None => self.spilled_m3 += amount,
            }
        }
        for (vessel, change) in self.vessels.iter_mut().zip(delta) {
            vessel.volume = (vessel.volume + change).max(0.0);
        }
        let mut values: Vec<(String, &str, f64)> =
            self.vessels.iter().map(|v| (v.name.clone(), "ml", v.volume * 1e6)).collect();
        values.push(("мимо".into(), "ml", self.spilled_m3 * 1e6));
        if let Some((index, ml, tolerance, deadline)) = self.goal {
            let missed = at >= deadline && (self.vessels[index].volume * 1e6 - ml).abs() > tolerance;
            values.push(("цель".into(), "1", if missed { 1.0 } else { 0.0 }));
        }
        values.push(("поток".into(), "ml/s", total_rate * 1e6));
        let values: Vec<(&str, &str, f64)> = values.iter().map(|(n, u, v)| (n.as_str(), *u, *v)).collect();
        emit(&mut out, at, &self.name, &values, "Квазистатическая жидкость: объём после перелива за интервал, пролив накопленный")?;
        out.states.insert(self.name.clone(), if total_rate > 0.0 { "льётся" } else { "покой" }.into());
        Ok(out)
    }
}

fn positive(name: &str, value: Scalar) -> Result<f64, String> {
    let value = rotary::value(value)?;
    if !(value.is_finite() && value > 0.0) {
        return Err(format!("{name} должен быть положительным"));
    }
    Ok(value)
}

pub(super) fn install(
    name: &str,
    world: &CompiledWorld,
    rt: &mut Runtime,
    vessels: BTreeMap<String, Vessel>,
    fluid: Fluid,
    gravity_m_s2: [f64; 3],
    goal: Option<Goal>,
) -> Result<(), String> {
    if vessels.is_empty() {
        return Err("жидкости нужен хотя бы один сосуд".into());
    }
    if !(gravity_m_s2.iter().all(|g| g.is_finite()) && norm(gravity_m_s2) > 0.0) {
        return Err("жидкости нужна ненулевая тяжесть".into());
    }
    let density = positive("density_kg_m3", fluid.density_kg_m3)?;
    let discharge = positive("discharge", fluid.discharge)?;
    if discharge > 1.0 {
        return Err("коэффициент расхода больше 1".into());
    }
    let mut models = Vec::new();
    let mut bodies = BTreeSet::new();
    for (vessel_name, vessel) in vessels {
        position(world, &Point { body: vessel.body.clone(), local_m: vessel.bottom_m })?;
        if !bodies.insert(vessel.body.clone()) || ["мимо", "поток", "цель"].contains(&vessel_name.as_str()) {
            return Err(format!("сосуд «{vessel_name}»: одно тело на сосуд, имена «мимо», «поток» и «цель» заняты"));
        }
        let length = norm(vessel.axis);
        if !(length.is_finite() && length > 0.0) || !vessel.bottom_m.iter().all(|c| c.is_finite()) {
            return Err(format!("сосуд «{vessel_name}»: нужна ненулевая ось и конечное дно"));
        }
        let profile = &vessel.profile_m;
        let valid = profile.len() >= 2
            && profile[0][0] == 0.0
            && profile.windows(2).all(|p| p[1][0] > p[0][0])
            && profile.iter().all(|[z, r]| z.is_finite() && r.is_finite() && *r > 0.0);
        if !valid {
            return Err(format!("сосуд «{vessel_name}»: профиль — от дна вверх, высоты растут, радиусы положительны"));
        }
        let table = Arc::new(Table::new(profile));
        let volume = vessel.initial_ml * 1e-6;
        if !(volume.is_finite() && volume >= 0.0 && volume <= table.capacity_m3) {
            return Err(format!(
                "сосуд «{vessel_name}»: начальный объём {} мл вне 0…{:.1} мл",
                vessel.initial_ml,
                table.capacity_m3 * 1e6
            ));
        }
        let axis = vessel.axis.map(|c| c / length);
        if vessel.fixed && vessel.rigid_initial {
            return Err(format!("сосуд «{vessel_name}»: закреплённому сосуду жёсткая жидкость не нужна"));
        }
        let rigid = vessel.rigid_initial.then(|| {
            let (_, [x, z]) = table.level(0.0, volume);
            let helper = if axis[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
            let side = add(helper, axis, -dot(helper, axis));
            let side = side.map(|c| c / norm(side));
            (density * volume, add(add(vessel.bottom_m, side, x), axis, z))
        });
        let glug = vessel.glug_ml_s.map(|g| positive("glug_ml_s", g).map(|v| v * 1e-6)).transpose()?;
        rt.publish_process_measurement(&format!("{name}/{vessel_name}"), "ml")?;
        models.push(Model {
            name: vessel_name,
            body: vessel.body,
            bottom: vessel.bottom_m,
            axis,
            table,
            glug,
            rigid,
            fixed: vessel.fixed,
            volume,
            velocity: None,
            acceleration: [0.0; 3],
        });
    }
    rt.publish_process_measurement(&format!("{name}/мимо"), "ml")?;
    rt.publish_process_measurement(&format!("{name}/поток"), "ml/s")?;
    let goal = match goal {
        Some(goal) => {
            let index = models
                .iter()
                .position(|m| m.name == goal.vessel)
                .ok_or_else(|| format!("цель налива: нет сосуда «{}»", goal.vessel))?;
            if !(goal.ml.is_finite() && goal.ml >= 0.0 && goal.tolerance_ml.is_finite() && goal.tolerance_ml > 0.0) {
                return Err("цель налива: объём и допуск должны быть неотрицательными и конечными".into());
            }
            rt.publish_process_measurement(&format!("{name}/цель"), "1")?;
            Some((index, goal.ml, goal.tolerance_ml, Time(goal.at_ns)))
        }
        None => None,
    };
    rt.register_process(
        name,
        Liquid {
            name: name.into(),
            vessels: models,
            density,
            discharge,
            detach: positive("detach_m_s", fluid.detach_m_s)?,
            slosh: positive("slosh_s", fluid.slosh_s)?,
            gravity: gravity_m_s2,
            spilled_m3: 0.0,
            goal,
        },
    )
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::*;

    const G: f64 = 9.80665;

    fn cylinder(r: f64, h: f64) -> Table {
        Table::new(&[[0.0, r], [h, r]])
    }

    #[test]
    fn стоящий_цилиндр_наливается_слоями() {
        let (r, h) = (0.037, 0.09);
        let table = cylinder(r, h);
        let exact = PI * r * r * h;
        assert!((table.capacity_m3 - exact).abs() / exact < 2e-3, "{} против {exact}", table.capacity_m3);
        for level in [0.01, 0.045, 0.08] {
            let (s, [x, z]) = table.level(0.0, PI * r * r * level);
            assert!((s - level).abs() < 2e-4, "уровень {s} против {level}");
            assert!(x.abs() < 1e-6 && (z - level / 2.0).abs() < 2e-4, "центр ({x}, {z})");
        }
    }

    #[test]
    fn лежащий_цилиндр_наполовину_полон_по_оси() {
        let (r, h) = (0.04, 0.2);
        let table = cylinder(r, h);
        let (s, [x, _]) = table.level(PI / 2.0, table.capacity_m3 / 2.0);
        assert!(s.abs() < 3e-4, "поверхность {s}");
        // Центр полукруга ниже оси на 4r/3π.
        assert!((x + 4.0 * r / (3.0 * PI)).abs() < 3e-4, "центр {x}");
    }

    #[test]
    fn струя_из_залитого_среза_по_торричелли() {
        let table = Table::new(&[[0.0, 0.04], [0.2, 0.04], [0.23, 0.011], [0.27, 0.011]]);
        // Горло вниз: срез на высоте −0.27 от дна, поверхность на 0.1 м выше среза.
        let flow = outflow(&table, PI, -0.17, G, 0.6, None);
        let area = PI * 0.011 * 0.011;
        assert!((flow.area_m2 - area).abs() / area < 2e-3, "{flow:?}");
        let speed = (2.0 * G * 0.1).sqrt();
        assert!((flow.speed_m_s - speed).abs() / speed < 1e-6, "{flow:?}");
        assert!((flow.rate_m3_s - 0.6 * speed * flow.area_m2).abs() < 1e-12, "{flow:?}");
        let limited = outflow(&table, PI, -0.17, G, 0.6, Some(5e-5));
        assert_eq!(limited.rate_m3_s, 5e-5);
        // Стоящий сосуд ниже среза не течёт.
        assert_eq!(outflow(&table, 0.0, 0.2, G, 0.6, Some(5e-5)), Outflow::default());
    }

    #[test]
    fn струя_попадает_только_внутрь_кромки() {
        let gravity = [0.0, 0.0, -G];
        let rim = [0.0, 0.0, 0.1];
        // Над центром кружки, без скорости: падает в центр.
        assert!(lands([0.0, 0.0, 0.14], [0.0; 3], gravity, rim, 0.03, 0.09));
        // Горизонтальная струя 0.5 м/с с 4 см над кромкой улетает на 4.5 см.
        assert!(!lands([0.0, 0.0, 0.14], [0.5, 0.0, 0.0], gravity, rim, 0.03, 0.09));
        assert!(lands([-0.04, 0.0, 0.14], [0.5, 0.0, 0.0], gravity, rim, 0.03, 0.09));
        // Горло опущено в кружку ниже кромки.
        assert!(lands([0.01, 0.0, 0.08], [0.0; 3], gravity, rim, 0.03, 0.09));
        assert!(!lands([0.05, 0.0, 0.08], [0.0; 3], gravity, rim, 0.03, 0.09));
    }

    const VESSEL: &str = r#"format = 3
name = "сосуд"

[[links]]
name = "корпус"
joint = { kind = "fixed" }
mass = { kind = "explicit", mass_kg = 0.05, com_m = [0.0, 0.0, 0.05], inertia_kg_m2 = [1e-4, 1e-4, 1e-4, 0.0, 0.0, 0.0], provenance = { estimate = "тест" } }
"#;

    const BOTTLE: [[f64; 2]; 4] = [[0.0, 0.04], [0.2, 0.04], [0.23, 0.011], [0.27, 0.011]];
    const RIM: [f64; 3] = [0.0, 0.0, 0.095];

    /// Неподвижная бутылка, повёрнутая на `tilt_deg` вокруг x, и кружка с дном в начале координат.
    fn world(label: &str, bottle: [f64; 3], tilt_deg: f64) -> CompiledWorld {
        let dir: PathBuf = std::env::temp_dir().join(format!("sim-liquid-{label}-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        // Бутылка «вес» свободна, остальные закреплены в мире.
        let vessel = if label == "вес" { VESSEL.replace("kind = \"fixed\"", "kind = \"free\", name = \"корпус\"") } else { VESSEL.into() };
        std::fs::write(dir.join("vessel.toml"), vessel).unwrap();
        let (s, c) = (tilt_deg.to_radians() / 2.0).sin_cos();
        let text = format!(
            "format = 3\nname = \"налив\"\nseed = 1\nphysics_hz = 1000\ngravity_m_s2 = [0.0, 0.0, -9.80665]\n\n\
             [environment]\ndensity_kg_m3 = 0.0\nviscosity_pa_s = 0.0\nwind_m_s = [0.0, 0.0, 0.0]\n\n\
             [[devices]]\nname = \"кружка\"\nfile = \"vessel.toml\"\n\n\
             [[devices]]\nname = \"бутылка\"\nfile = \"vessel.toml\"\npose = {{ position_m = {bottle:?}, quat_wxyz = [{c}, {s}, 0.0, 0.0] }}\n"
        );
        std::fs::write(dir.join("world.toml"), text).unwrap();
        let world = sim_world::load_and_compile(&dir.join("world.toml")).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
        world
    }

    fn fluid() -> Fluid {
        toml::from_str(
            r#"density_kg_m3 = { value = 998.0, source = { estimate = "вода 20 °C" } }
discharge = { value = 0.6, source = { estimate = "тест" } }
detach_m_s = { value = 0.1, source = { estimate = "тест" } }
slosh_s = { value = 0.05, source = { estimate = "тест" } }"#,
        )
        .unwrap()
    }

    fn vessels(bottle_ml: f64) -> BTreeMap<String, Vessel> {
        let bottle = Vessel {
            body: "бутылка/корпус".into(),
            bottom_m: [0.0; 3],
            axis: [0.0, 0.0, 1.0],
            profile_m: BOTTLE.to_vec(),
            initial_ml: bottle_ml,
            rigid_initial: false,
            glug_ml_s: None,
            fixed: true,
        };
        let mug = Vessel {
            body: "кружка/корпус".into(),
            bottom_m: [0.0, 0.0, 0.005],
            axis: [0.0, 0.0, 1.0],
            profile_m: vec![[0.0, 0.037], [0.09, 0.037]],
            initial_ml: 0.0,
            rigid_initial: false,
            glug_ml_s: None,
            fixed: true,
        };
        [("бутылка".to_string(), bottle), ("кружка".to_string(), mug)].into_iter().collect()
    }

    /// Цель: в кружку весь объём бутылки ± 1 мл к 100 мс.
    fn goal(ml: f64) -> Option<Goal> {
        Some(Goal { vessel: "кружка".into(), ml, tolerance_ml: 1.0, at_ns: 100_000_000 })
    }

    fn last(rt: &Runtime, signal: &str) -> f64 {
        rt.process_journal()
            .iter()
            .rev()
            .flat_map(|tick| tick.outputs.get("вода"))
            .flat_map(|out| &out.measurements)
            .find(|m| m.signal.name == signal)
            .and_then(|m| match m.result {
                Analysis::Analyzed { value, .. } => Some(value),
                _ => None,
            })
            .unwrap()
    }

    /// Положение начала бутылки, при котором её точка `local` (в системе бутылки) приходится
    /// на `target` при повороте на `tilt_deg` вокруг x.
    fn placed(target: [f64; 3], local: [f64; 3], tilt_deg: f64) -> [f64; 3] {
        let (s, c) = tilt_deg.to_radians().sin_cos();
        let world = [local[0], c * local[1] - s * local[2], s * local[1] + c * local[2]];
        [target[0] - world[0], target[1] - world[1], target[2] - world[2]]
    }

    fn run(label: &str, bottle: [f64; 3], tilt_deg: f64, ml: f64, until_ns: u64) -> [f64; 3] {
        let world = world(label, bottle, tilt_deg);
        let mut rt = Runtime::load(world.clone()).unwrap();
        install("вода", &world, &mut rt, vessels(ml), fluid(), [0.0, 0.0, -G], goal(ml)).unwrap();
        let status = rt.run_until(Time(until_ns)).unwrap();
        assert_eq!(status, sim_runtime::Status::Running, "{:?}", rt.failure());
        let result = ["вода/бутылка", "вода/кружка", "вода/мимо"].map(|s| last(&rt, s));
        // Событие цели: 0 до срока; после срока 1, если в кружке не весь объём ± 1 мл.
        let goal = last(&rt, "вода/цель");
        assert_eq!(goal, if (result[1] - ml).abs() > 1.0 { 1.0 } else { 0.0 });
        result
    }

    /// Объём, при котором поверхность на `head` выше нижней точки кромки при наклоне `tilt_deg`.
    fn barely(tilt_deg: f64, head: f64) -> f64 {
        let table = Table::new(&BOTTLE);
        let theta = tilt_deg.to_radians();
        let lip = 0.27 * theta.cos() - 0.011 * theta.sin() + head;
        let (mut lo, mut hi) = (0.0, table.capacity_m3);
        for _ in 0..60 {
            let mid = (lo + hi) / 2.0;
            if table.level(theta, mid).0 < lip { lo = mid } else { hi = mid }
        }
        lo * 1e6
    }

    #[test]
    fn налив_горлом_вниз_над_кружкой_весь_в_кружке() {
        let bottle = placed(add(RIM, [0.0, 0.0, 0.03], 1.0), [0.0, 0.0, 0.27], 180.0);
        let [b, k, spill] = run("центр", bottle, 180.0, 300.0, 1_500_000_000);
        assert!(k > 299.0, "в кружке {k} мл, в бутылке {b}");
        assert_eq!(spill, 0.0, "мимо {spill} мл");
        assert!((b + k - 300.0).abs() < 1e-6, "{b} + {k}");
    }

    #[test]
    fn налив_мимо_кружки_весь_проливается() {
        let bottle = placed([0.0, 0.12, 0.125], [0.0, 0.0, 0.27], 180.0);
        let [b, k, spill] = run("мимо", bottle, 180.0, 300.0, 300_000_000);
        assert_eq!(k, 0.0);
        assert!(spill > 20.0 && (b + spill - 300.0).abs() < 1e-6, "{b} + {spill}");
    }

    #[test]
    fn медленная_струя_у_поднятого_горла_бежит_мимо() {
        let ml = barely(80.0, 0.0002);
        let bottle = placed([0.0, 0.0, 0.125], [0.0, 0.0, 0.27], 80.0);
        let [b, k, spill] = run("чайник", bottle, 80.0, ml, 200_000_000);
        assert_eq!(k, 0.0, "в кружке {k}");
        assert!(spill > 0.0 && (b + spill - ml).abs() < 1e-6, "{b} + {spill} из {ml}");
    }

    #[test]
    fn медленная_струя_у_опущенного_горла_капает_в_кружку() {
        let ml = barely(100.0, 0.0002);
        let bottle = placed(add(RIM, [0.0, 0.0, 0.03], 1.0), [0.0, -0.011, 0.27], 100.0);
        let [b, k, spill] = run("капли", bottle, 100.0, ml, 200_000_000);
        assert_eq!(spill, 0.0, "мимо {spill}");
        assert!(k > 0.0 && (b + k - ml).abs() < 1e-6, "{b} + {k} из {ml}");
    }

    #[test]
    fn вода_давит_весом_и_падает_вместе_с_сосудом() {
        // Свободная бутылка 50 г с 500 мл воды падает: вода давит весом в начале и не
        // выливается, пока ускорение сосуда догоняет тяжесть.
        let world = world("вес", [0.3, 0.0, 0.0], 0.0);
        let mut rt = Runtime::load(world.clone()).unwrap();
        let mut set = vessels(500.0);
        set.values_mut().for_each(|v| v.fixed = false);
        install("вода", &world, &mut rt, set, fluid(), [0.0, 0.0, -G], None).unwrap();
        let status = rt.run_until(Time(200_000_000)).unwrap();
        assert_eq!(status, sim_runtime::Status::Running, "{:?}", rt.failure());
        let out = &rt.process_journal()[0].outputs["вода"];
        let load = out.body_loads.iter().find(|l| l.body == "бутылка/корпус").unwrap();
        let expected = -998.0 * 500e-6 * G;
        assert!((load.force_n[2] - expected).abs() < 1e-9, "{:?}", load.force_n);
        assert_eq!(last(&rt, "вода/мимо"), 0.0);
        assert_eq!(last(&rt, "вода/бутылка"), 500.0);
        // К концу падения вода почти невесома: сила от неё много меньше веса.
        let tail = &rt.process_journal().last().unwrap().outputs["вода"];
        let load = tail.body_loads.iter().find(|l| l.body == "бутылка/корпус").unwrap();
        assert!(load.force_n[2].abs() < 0.05 * expected.abs(), "{:?}", load.force_n);
    }

    #[test]
    fn неверные_сосуды_отклоняются() {
        let world = world("проверка", [0.3, 0.0, 0.0], 0.0);
        let mut rt = Runtime::load(world.clone()).unwrap();
        let mut bad = vessels(5000.0);
        let error = install("вода", &world, &mut rt, bad.clone(), fluid(), [0.0, 0.0, -G], None).unwrap_err();
        assert!(error.contains("вне 0…"), "{error}");
        bad.get_mut("бутылка").unwrap().initial_ml = 100.0;
        bad.get_mut("бутылка").unwrap().profile_m = vec![[0.0, 0.04], [0.0, 0.04]];
        let error = install("вода", &world, &mut rt, bad, fluid(), [0.0, 0.0, -G], None).unwrap_err();
        assert!(error.contains("профиль"), "{error}");
    }
}
