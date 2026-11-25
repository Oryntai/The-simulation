//! Клиент общего мира и штатного конвейера.
mod audit;
mod input;
mod json;
mod render;
mod report;
mod studio;

use std::collections::BTreeMap;
use std::ffi::OsString;
use std::fs;
use std::io::Cursor;
use std::path::{Path, PathBuf};

use nalgebra::{Matrix3, Quaternion, UnitQuaternion};
use sim_human::scene::{self, ParticipantSpec};
use sim_mujoco::{JointKind, Sim, ffi::mjtObj};
use sim_runtime::{Log, LogKind, Runtime};
use sim_cli::view::{CampaignBinding, MountEvent, Peak, RunView, SceneRecord, SensorMount, ViewMetadata};
use sim_sense::Intrinsics;
use sim_signal::{ImageFrame, Time};
use sim_world::{CompiledWorld, ContactBuildOptions, DeviceInput, ModelFragment, Pose};

pub use audit::Wrench;
pub use render::{Camera, Force, Texture};
use render::{Geometry, Point, Vec3};
pub use report::{Report, Verdict};

#[derive(Clone)]
pub struct Participant {
    pub name: String,
    pub spec: ParticipantSpec,
    pub pose: Pose,
}

/// Геометрия проектного мира и поза последнего запрошенного кадра.
pub struct Scene {
    name: String,
    world_hash: u64,
    sensors: Vec<SensorMount>,
    sim: Sim,
    geometry: Vec<Geometry>,
    center: Point,
    radius: f64,
    units: Vec<&'static str>,
}

#[derive(Clone, Copy)]
pub enum View<'a> {
    Overview {
        azimuth: f64,
        elevation: f64,
        zoom: f64,
    },
    Sensor(&'a str),
}

impl View<'_> {
    /// Начальное направление свободной обзорной камеры.
    pub const OVERVIEW: Self = Self::Overview {
        azimuth: -1.0,
        elevation: 0.5,
        zoom: 1.0,
    };
}

impl Scene {
    pub fn open(
        world_file: &Path,
        participant: Option<&Participant>,
        placements: &BTreeMap<String, Pose>,
    ) -> Result<Self, String> {
        let compiled = if participant.is_none() && placements.is_empty() {
            sim_world::load_and_compile(world_file)?
        } else {
            let mut world = sim_world::parse_world(world_file, &read_text(world_file)?)?;
            for (id, pose) in placements {
                world
                    .devices
                    .iter_mut()
                    .find(|device| &device.name == id)
                    .ok_or_else(|| format!("размещение «{id}» отсутствует в мире"))?
                    .pose = pose.clone();
            }
            let base = world_file.parent().ok_or("нет каталога мира")?;
            let devices = world
                .devices
                .iter()
                .map(|placement| {
                    let path = base.join(&placement.file);
                    let device = sim_world::load_device(&path, &sim_world::LegacyCatalog)?;
                    Ok(DeviceInput { path, device })
                })
                .collect::<Result<Vec<_>, String>>()?;
            // Геометрия того же масштабированного участника. Паспорта нагрузок и проводку
            // устанавливает только конвейер, для показа их регистрировать не нужно.
            let fragments = participant
                .map(|participant| {
                    let model = scene::build(&participant.spec)?;
                    Ok::<_, String>(ModelFragment {
                        name: participant.name.clone(),
                        source: model.source,
                        mjcf: Some(model.mjcf),
                        pose: participant.pose.clone(),
                        inertia: BTreeMap::new(),
                        ports: BTreeMap::new(),
                        surfaces: BTreeMap::new(),
                        efforts: Vec::new(),
                        drives: Vec::new(),
                    })
                })
                .transpose()?
                .into_iter()
                .collect::<Vec<_>>();
            sim_world::compile_with_fragments(
                world_file,
                &world,
                &devices,
                &fragments,
                &ContactBuildOptions::default(),
            )?
        };
        Self::from_compiled(compiled)
    }

    pub fn from_compiled(compiled: CompiledWorld) -> Result<Self, String> {
        let runtime = Runtime::load(compiled)?;
        let recorded = runtime.view_scene();
        let sensors = recorded.sensors.iter().map(|s| SensorMount {
            name: s.name.clone(), body: s.body.clone(), position_m: s.position_m,
            quat_wxyz: s.quat_wxyz,
        }).collect();
        Self::build(recorded.name.clone(), recorded.fingerprint(), sensors, &recorded.mjcf)
    }

    pub fn from_recorded(scene: &SceneRecord) -> Result<Self, String> {
        if scene.engine != sim_mujoco::version() {
            return Err("версия движка сцены отличается от версии записи".into());
        }
        Self::build(scene.name.clone(), scene.world_hash(), scene.sensors.clone(), &scene.mjcf)
    }

    fn build(name: String, world_hash: u64, sensors: Vec<SensorMount>, mjcf: &str) -> Result<Self, String> {
        let sim = Sim::from_xml(mjcf)?;
        let geometry = Geometry::build(&sim)?;
        // Для мира без конечных объектов 1 м задаёт обзорную область сетки интерфейса.
        let (center, radius) = render::bounds(&sim, &geometry, None)
            .or_else(|| render::kinematic_bounds(&sim))
            .unwrap_or((Point::origin(), 1.0));
        let mut units = vec![""; sim.nq()];
        for joint in sim.joints() {
            let fields: &[_] = match joint.kind {
                JointKind::Ball => &["1"; 4],
                JointKind::Hinge => &["rad"],
                JointKind::Slide => &["m"],
                JointKind::Free => &["m", "m", "m", "1", "1", "1", "1"],
            };
            units[joint.qpos_adr..joint.qpos_adr + fields.len()].copy_from_slice(fields);
        }
        Ok(Self {
            name,
            world_hash,
            sensors,
            sim,
            geometry,
            center,
            radius,
            units,
        })
    }

    pub fn name(&self) -> &str {
        &self.name
    }
    pub fn sensors(&self) -> impl Iterator<Item = &str> {
        self.sensors.iter().map(|s| s.name.as_str())
    }
    pub fn uses_kinematics(&self) -> bool {
        render::needs_kinematics(&self.sim, &self.geometry)
    }

    pub fn geoms(&self) -> Vec<sim_mujoco::GeomInfo> {
        self.sim.geoms()
    }
    /// Перекадрировать обзор на видимую геометрию выбранного тела.
    pub fn focus(&mut self, body: &str) -> Result<(), String> {
        let id = self
            .sim
            .id(mjtObj::mjOBJ_BODY, body)
            .ok_or("тело отсутствует в сцене")?;
        (self.center, self.radius) = render::bounds(&self.sim, &self.geometry, Some(id))
            .ok_or("у тела нет геометрии для показа")?;
        Ok(())
    }

    pub fn geometry_pose(&self, id: usize) -> Option<([f64; 3], [f64; 9])> {
        (id < self.sim.ngeom()).then(|| self.sim.geom_pose(id))
    }

    pub fn seek(&mut self, replay: &Replay, at: Time) -> Result<Time, String> {
        if at > replay.end {
            return Err("запрошено время за концом журнала".into());
        }
        if replay.qpos.len() != self.units.len() {
            return Err("состав координат сцены расходится с журналом".into());
        }
        let mut time = None;
        let positions = self
            .units
            .iter()
            .enumerate()
            .map(|(i, &unit)| {
                let series = &replay.series[replay.qpos[i]];
                let sample = series
                    .at(at)
                    .filter(|s| s.valid && s.value.is_finite())
                    .ok_or("отсутствует координата позы в журнале")?;
                if series.unit != unit || time.replace(sample.at).is_some_and(|t| t != sample.at) {
                    return Err("записанные координаты расходятся по времени или единицам".into());
                }
                Ok(sample.value)
            })
            .collect::<Result<Vec<_>, String>>()?;
        for joint in self.sim.joints() {
            let adr = joint.qpos_adr;
            let q = match joint.kind {
                JointKind::Ball => Some(&positions[adr..adr + 4]),
                JointKind::Free => Some(&positions[adr + 3..adr + 7]),
                _ => None,
            };
            if q.is_some_and(|q| {
                let norm = q.iter().copied().fold(0.0_f64, f64::hypot);
                // Допуск покрывает округление четырёх компонент f64 при нормировке MuJoCo.
                (norm - 1.0).abs() > 64.0 * f64::EPSILON
            }) {
                return Err("ненормированный кватернион в журнале".into());
            }
        }
        self.sim.qpos_mut().copy_from_slice(&positions);
        self.sim.forward();
        if !self.sim.numerically_ok()
            || !self
                .sim
                .qpos()
                .iter()
                .zip(&positions)
                .all(|(a, b)| a.to_bits() == b.to_bits())
        {
            return Err("поза из журнала численно некорректна".into());
        }
        Ok(time.unwrap_or(at))
    }

    /// Строгое совпадение мира штатного CLI. Геометрия из отдельного профиля
    /// может иметь другой отпечаток при совпадающей схеме координат.
    pub fn same_world(&self, replay: &Replay) -> Result<bool, String> {
        Ok(self.world_hash == replay.world_hash)
    }

    pub fn camera(&self, view: View<'_>, [width, height]: [u32; 2]) -> Result<Camera, String> {
        // Обзорная оптика 45°, это параметр показа. Дальномер и другие датчики
        // без оптики получают такой же условный вид из точки установки.
        let focal = f64::from(width.min(height)) / (2.0 * (std::f64::consts::FRAC_PI_8).tan());
        let lens = Intrinsics {
            width,
            height,
            fx: focal,
            fy: focal,
            cx: f64::from(width) / 2.0,
            cy: f64::from(height) / 2.0,
            radial: [0.0; 3],
            tangential: [0.0; 2],
        };
        let (center, radius) = if matches!(view, View::Overview { .. }) && self.uses_kinematics() {
            render::kinematic_bounds(&self.sim).ok_or("неконечная кинематическая схема")?
        } else {
            (self.center, self.radius)
        };
        // Плоскость отсечения пропорциональна размеру сцены и служит только рендеру.
        let near = radius / 1024.0;
        let (eye, axes) = match view {
            View::Overview {
                azimuth,
                elevation,
                zoom,
            } => {
                if !azimuth.is_finite()
                    || !elevation.is_finite()
                    || !zoom.is_finite()
                    || zoom <= 0.0
                    || elevation.abs() >= std::f64::consts::FRAC_PI_2
                {
                    return Err("неверные углы или масштаб обзора".into());
                }
                let direction = Vec3::new(
                    azimuth.cos() * elevation.cos(),
                    azimuth.sin() * elevation.cos(),
                    elevation.sin(),
                );
                // Охватывающая сфера на базе угла обзора и масштаба интерфейса.
                let distance = radius / std::f64::consts::FRAC_PI_8.sin() / zoom;
                let z = -direction;
                let x = z.cross(&Vec3::z()).normalize();
                let y = z.cross(&x);
                (
                    center + direction * distance,
                    Matrix3::from_columns(&[x, y, z]),
                )
            }
            View::Sensor(id) => {
                let sensor = self
                    .sensors
                    .iter()
                    .find(|s| s.name == id)
                    .ok_or("датчик не найден в мире")?;
                let (body, rotation) = match sensor.body.as_deref() {
                    Some(body) => {
                        let id = self
                            .sim
                            .id(mjtObj::mjOBJ_BODY, body)
                            .ok_or("тело датчика не найдено")?;
                        let (p, q) = self.sim.body_pose(id);
                        (Point::from(p), quaternion(q)?)
                    }
                    None => (Point::origin(), UnitQuaternion::identity()),
                };
                let mount = rotation * quaternion(sensor.quat_wxyz)?;
                (
                    body + rotation * Vec3::from(sensor.position_m),
                    mount.to_rotation_matrix().into_inner(),
                )
            }
        };
        Camera::new(eye, axes, lens, near)
    }

    pub fn render(
        &self,
        camera: &Camera,
        force: Option<&Force>,
        reactions: &[Wrench],
        scale_n: f64,
    ) -> Result<Texture, String> {
        // Плоскость видна за границей объёма, отступ вычисляется по диагонали кадра.
        render::render(
            &self.sim,
            &self.geometry,
            camera,
            (camera.eye.coords.norm() + self.radius) * 2.0,
            force,
            reactions,
            self.radius / 4.0,
            scale_n,
        )
    }
}

fn quaternion([w, x, y, z]: [f64; 4]) -> Result<UnitQuaternion<f64>, String> {
    let q = Quaternion::new(w, x, y, z);
    if !q.coords.iter().all(|v| v.is_finite()) {
        return Err("неконечная ориентация".into());
    }
    UnitQuaternion::try_new(q, 0.0).ok_or_else(|| "нулевая ориентация".into())
}

#[derive(Clone, Copy, Debug)]
pub struct PointSample {
    pub at: Time,
    pub value: f64,
    pub valid: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SignalKind {
    Observation,
    Truth,
    Command,
    Process,
}

pub struct Series {
    pub kind: SignalKind,
    pub name: String,
    pub unit: String,
    pub points: Vec<PointSample>,
    scale: f64,
}

impl Series {
    pub fn at(&self, at: Time) -> Option<PointSample> {
        self.points
            .partition_point(|p| p.at <= at)
            .checked_sub(1)
            .map(|i| self.points[i])
    }

    pub fn force(&self, at: Time) -> Option<Force> {
        let sample = self.at(at).filter(|sample| sample.valid)?;
        (self.unit == "N").then(|| Force {
            name: self.name.clone(),
            value_n: sample.value,
            scale_n: self.scale,
        })
    }
}

#[derive(Default)]
struct SensorView {
    channels: Vec<usize>,
    image: bool,
}

pub struct Replay {
    pub end: Time,
    pub seed: u64,
    pub journal: u64,
    pub status: sim_runtime::Status,
    pub series: Vec<Series>,
    pub frames: BTreeMap<String, Vec<ImageFrame>>,
    pub wrenches: BTreeMap<Time, Vec<Wrench>>,
    pub wrench_scale: f64,
    pub recorded_scene: Option<SceneRecord>,
    pub peak: Option<Peak>,
    pub campaign: Option<CampaignBinding>,
    sensor_views: BTreeMap<String, SensorView>,
    pub attachments: Vec<MountEvent>,
    pub contact_gap: Option<String>,
    qpos: Vec<usize>,
    world_hash: u64,
}

impl Replay {
    pub fn read(binary: &[u8]) -> Result<Self, String> {
        let log = Log::read_binary(binary).map_err(|error| error.to_string())?;
        Self::from_log(&log, None, BTreeMap::new())
    }

    pub fn from_view(view: RunView) -> Result<Self, String> {
        let frames = group_frames(&view.journal, view.frames);
        Self::from_log(&view.journal, Some(view.metadata), frames)
    }

    fn from_log(log: &Log, mut metadata: Option<ViewMetadata>, frames: BTreeMap<String, Vec<ImageFrame>>) -> Result<Self, String> {
        let header = log.header();
        let mut series = Vec::new();
        let tables = [
            (SignalKind::Observation, &header.observations),
            (SignalKind::Truth, &header.truth),
            (SignalKind::Command, &header.commands),
        ];
        for (kind, table) in tables {
            series.extend(table.iter().map(|spec| Series {
                kind,
                name: spec.name.clone(),
                unit: spec.unit.clone(),
                points: Vec::new(),
                scale: 0.0,
            }));
        }
        for entry in log.entries() {
            let offset = match entry.kind {
                LogKind::Observation => 0,
                LogKind::Truth => header.observations.len(),
                LogKind::Command(_) => header.observations.len() + header.truth.len(),
            };
            let s = &mut series[offset + entry.id as usize];
            s.points.push(PointSample {
                at: entry.at,
                value: entry.value,
                valid: entry.valid,
            });
            if entry.valid {
                s.scale = s.scale.max(entry.value.abs());
            }
        }
        let mut qpos = BTreeMap::new();
        for (index, spec) in series
            .iter()
            .enumerate()
            .filter(|(_, s)| s.kind == SignalKind::Truth)
        {
            if let Some(joint) = spec
                .name
                .strip_prefix("qpos[")
                .and_then(|s| s.strip_suffix(']'))
                .and_then(|s| s.parse::<usize>().ok())
                && qpos.insert(joint, index).is_some()
            {
                return Err("повтор координаты в журнале".into());
            }
        }
        if !qpos.keys().copied().eq(0..qpos.len()) {
            return Err("пропуск координаты в журнале".into());
        }
        let (forces, wrenches) = if let Some(meta) = &mut metadata {
            let mut forces = BTreeMap::new();
            for p in std::mem::take(&mut meta.process) {
                forces.entry((p.name, p.unit)).or_insert_with(Vec::new).push(PointSample {
                    at: Time(p.at_ns), value: p.value.unwrap_or_default(), valid: p.value.is_some(),
                });
            }
            let mut wrenches = BTreeMap::new();
            for r in std::mem::take(&mut meta.reactions) {
                wrenches.entry(Time(r.start_ns)).or_insert_with(Vec::new).push(Wrench {
                    pair: r.pair, point_m: r.point_m, force_n: r.force_n,
                });
            }
            (forces, wrenches)
        } else {
            let audit = audit::Audit::read(log.process_entries())?;
            (audit.measurements, audit.wrenches)
        };
        for ((name, unit), mut points) in forces {
            points.sort_by_key(|p| p.at);
            let scale = points
                .iter()
                .filter(|p| p.valid)
                .map(|p| p.value.abs())
                .fold(0.0, f64::max);
            series.push(Series {
                kind: SignalKind::Process,
                name,
                unit,
                scale,
                points,
            });
        }
        let wrench_scale = wrenches
            .values()
            .flatten()
            .map(|w| Vec3::from(w.force_n).norm())
            .fold(0.0, f64::max);
        if !wrench_scale.is_finite() {
            return Err("модуль силы вышел за диапазон".into());
        }
        let (recorded_scene, peak, attachments, contact_gap, campaign) = metadata.map(|meta| {
            (Some(meta.scene), meta.peak, meta.attachments, meta.contact_gap, meta.campaign)
        }).unwrap_or_default();
        let mounts = recorded_scene.as_ref().into_iter().flat_map(|s| &s.sensors);
        let mounts: Vec<_> = mounts.map(|mount| mount.name.as_str()).collect();
        let mut sensor_views: BTreeMap<String, SensorView> = frames.keys()
            .map(|name| (name.clone(), SensorView { image: true, ..Default::default() }))
            .collect();
        let image_ids: std::collections::BTreeSet<_> = frames.values().flatten()
            .map(|frame| frame.signal.0 as usize).collect();
        for (index, signal) in series.iter().enumerate().filter(|(_, signal)| {
            matches!(signal.kind, SignalKind::Observation | SignalKind::Process)
        }) {
            let fallback = signal.name.rsplit_once('/').map_or(signal.name.as_str(), |(name, _)| name);
            let name = mounts.iter().copied()
                .filter(|mount| signal.name == *mount
                    || signal.name.strip_prefix(*mount).is_some_and(|suffix| suffix.starts_with('/')))
                .max_by_key(|mount| mount.len()).unwrap_or(fallback);
            let view = sensor_views.entry(name.to_owned()).or_default();
            // Форматы и окончания имён совпадают с регистрацией камеры в sim-sense.
            let image = image_ids.contains(&index)
                || signal.name.ends_with("/image") && signal.unit == "RGB8"
                || signal.name.ends_with("/depth") && signal.unit == "mm";
            if image {
                view.image = true;
            } else {
                view.channels.push(index);
            }
        }
        Ok(Self {
            end: log.end().time,
            seed: header.seed,
            journal: log.fingerprint(),
            status: log.end().status,
            series,
            frames,
            wrenches,
            recorded_scene,
            peak,
            campaign,
            sensor_views,
            attachments,
            contact_gap,
            wrench_scale,
            qpos: qpos.into_values().collect(),
            world_hash: header.world_hash,
        })
    }

    pub fn sensor_names(&self) -> impl Iterator<Item = &str> {
        self.sensor_views.keys().map(String::as_str)
    }

    pub fn sensor_series(&self, sensor: &str) -> impl Iterator<Item = &Series> {
        self.sensor_views.get(sensor).into_iter()
            .flat_map(|view| view.channels.iter().map(|&i| &self.series[i]))
    }

    pub fn sensor_has_image(&self, sensor: &str) -> bool {
        self.sensor_views.get(sensor).is_some_and(|view| view.image)
    }

    pub fn image(&self, sensor: &str, at: Time) -> Option<&ImageFrame> {
        if at > self.end {
            return None;
        }
        let images = self.frames.get(sensor)?;
        let i = images.partition_point(|f| f.at <= at).checked_sub(1)?;
        let frame = images
            .get(i)
            .filter(|frame| frame.valid && frame.is_consistent())?;
        // Скалярный канал отмечает и пропуски. Старый кадр после отказа не выдаём.
        let series = self.series.get(frame.signal.0 as usize)?;
        if series.kind != SignalKind::Observation {
            return None;
        }
        let sample = series.at(at)?;
        (sample.at == frame.at && sample.valid && sample.value == frame.fingerprint())
            .then_some(frame)
    }
}

#[derive(Clone, Debug)]
pub enum Input {
    World {
        world: PathBuf,
        duration: Time,
        catalog: Option<PathBuf>,
    },
    Task {
        world: PathBuf,
        task: PathBuf,
        case: Option<String>,
        catalog: Option<PathBuf>,
        matrix: bool,
    },
    Campaign {
        manifest: PathBuf,
        repeat: Option<String>,
        /// Пакет заверенных рядов в формате `sim-run --measurements`.
        measurements: Option<PathBuf>,
    },
}

pub struct Output {
    pub report: Report,
    pub binary: Option<Vec<u8>>,
    pub frames: BTreeMap<String, Vec<ImageFrame>>,
    recording: Option<(Log, ViewMetadata)>,
}

impl Output {
    pub fn into_replay(self) -> Result<(Report, Option<Replay>), String> {
        let replay = self
            .binary
            .map(|binary| -> Result<Replay, String> {
                let replay = match self.recording {
                    Some((log, metadata)) => Replay::from_log(&log, Some(metadata), self.frames)?,
                    None => Replay::read(&binary)?,
                };
                if self
                    .report
                    .journal_fingerprint()
                    .is_some_and(|expected| expected != format!("{:016x}", replay.journal))
                {
                    return Err("отпечатки журнала и отчёта расходятся".into());
                }
                if self.report.status() == "completed"
                    && replay.status != sim_runtime::Status::Running
                {
                    return Err("завершённый отчёт сопровождается аварийным журналом".into());
                }
                Ok(replay)
            })
            .transpose()?;
        Ok((self.report, replay))
    }
}

/// Обычный запуск конвейера; отчёт передаётся без изменений.
pub fn run(input: &Input) -> Result<Output, String> {
    run_with_peak(input, None)
}

pub(crate) fn run_with_peak(input: &Input, peak: Option<&str>) -> Result<Output, String> {
    // Отдельный каталог на вызов CLI, включая конкурирующие вызовы пользователей.
    let temp = Temp::new()?;
    let archive = temp.0.join("run.simview");
    let flag = |f: &str| OsString::from(f);
    let path = |p: &Path| p.as_os_str().to_owned();
    let mut args = vec![flag("--json")];
    match input {
        Input::World {
            world,
            duration,
            catalog,
        } => {
            args.extend([
                path(world),
                flag("--duration"),
                format_seconds(*duration).into(),
            ]);
            if let Some(p) = catalog {
                args.extend([flag("--catalog"), path(p)]);
            }
        }
        Input::Task {
            world,
            task,
            case,
            catalog,
            matrix,
        } => {
            args.extend([path(world), flag("--task"), path(task)]);
            if *matrix {
                args.extend([flag("--matrix"), flag("full")]);
            }
            if let Some(c) = case {
                args.extend([flag("--case"), c.into()]);
            }
            if let Some(p) = catalog {
                args.extend([flag("--catalog"), path(p)]);
            }
        }
        Input::Campaign {
            manifest,
            repeat,
            measurements,
        } => {
            args.extend([flag("--campaign"), path(manifest)]);
            if let Some(p) = measurements {
                args.extend([flag("--measurements"), path(p)]);
            }
            if let Some(key) = repeat {
                args.extend([flag("--repeat"), key.into(), flag("--view"), path(&archive)]);
            }
        }
    }
    if matches!(
        input,
        Input::World { .. } | Input::Task { case: Some(_), .. }
    ) {
        args.extend([flag("--view"), path(&archive)]);
        if let Some(peak) = peak {
            args.extend([flag("--peak"), peak.into()]);
        }
    }
    let mut json = Vec::new();
    let mut errors = Vec::new();
    let code = sim_cli::run(args, Cursor::new([]), &mut json, &mut errors);
    let raw = String::from_utf8(json).map_err(|error| error.to_string())?;
    let report = Report::parse(raw)
        .map_err(|error| format!("{error} · {}", String::from_utf8_lossy(&errors)))?;
    if code > 1 || report.status() == "error" {
        return Err(format!(
            "конвейер завершился с ошибкой · {}",
            report.reasons(None).join(" · ")
        ));
    }
    let (binary, frames, recording) = match fs::read(&archive) {
        Ok(bytes) => {
            let view = RunView::read(&bytes)?;
            let expected_mode = match input {
                Input::Task { case, .. } => case.as_deref(),
                Input::Campaign { repeat: Some(key), .. } => {
                    let binding = report.playback(key)
                        .ok_or("нет завершённой строки отчёта для пакета просмотра")?;
                    if report.repeat_key() != Some(key.as_str())
                        || view.metadata.campaign.as_ref() != Some(binding)
                    {
                        return Err("пакет просмотра и отчёт повтора расходятся".into());
                    }
                    Some(binding.playback.child.mode.as_str())
                }
                _ => None,
            };
            if view.metadata.mode.as_deref() != expected_mode {
                return Err("вариант пакета просмотра отличается от запроса".into());
            }
            (
                Some(view.binary),
                group_frames(&view.journal, view.frames),
                Some((view.journal, view.metadata)),
            )
        }
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
            if report.repeat_key().is_some() && report.view_unavailable().is_none() {
                return Err("в отчёте повтора отсутствуют журнал и причина пропуска".into());
            }
            (None, BTreeMap::new(), None)
        }
        Err(error) => return Err(error.to_string()),
    };
    Ok(Output { report, binary, frames, recording })
}

fn group_frames(log: &Log, frames: Vec<ImageFrame>) -> BTreeMap<String, Vec<ImageFrame>> {
    let mut images = BTreeMap::<String, Vec<ImageFrame>>::new();
    for frame in frames {
        let name = &log.header().observations[frame.signal.0 as usize].name;
        let suffix = match frame.format {
            sim_signal::PixelFormat::Rgb8 => "/image",
            sim_signal::PixelFormat::DepthMm16 => "/depth",
        };
        let name = name.strip_suffix(suffix).unwrap_or(name);
        images.entry(name.into()).or_default().push(frame);
    }
    images
}

fn format_seconds(time: Time) -> String {
    // Основание десятичной шкалы наносекунд из контракта sim-signal.
    let (s, ns) = (time.0 / 1_000_000_000, time.0 % 1_000_000_000);
    format!("{s}.{ns:09}")
}

pub(crate) fn read_text(path: &Path) -> Result<String, String> {
    fs::read_to_string(path).map_err(|error| format!("{} · {error}", path.display()))
}

struct Temp(PathBuf);
impl Temp {
    fn new() -> Result<Self, String> {
        let base = std::env::temp_dir();
        for n in 0..u32::MAX {
            let path = base.join(format!("workshop-{}-{n}", std::process::id()));
            match fs::create_dir(&path) {
                Ok(()) => return Ok(Self(path)),
                Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {}
                Err(e) => return Err(format!("{} · {e}", path.display())),
            }
        }
        Err("исчерпаны имена временных каталогов".into())
    }
}
impl Drop for Temp {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

pub fn start(args: Vec<OsString>) -> bevy::app::AppExit {
    studio::start(args)
}

#[cfg(test)]
mod tests;
