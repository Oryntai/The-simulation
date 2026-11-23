//! Каталог и модели датчиков с раздельными выходами для контроллера и оценщика.

mod camera;
mod catalog;
mod optics;
mod pipeline;
mod raster;
mod scalar;
mod settings;
mod vision;

use std::collections::BTreeSet;
use std::sync::Arc;

use sim_mujoco::Sim;
use sim_runtime::Runtime;
use sim_signal::{Rate, SignalId, fnv1a};
use sim_world::{CompiledWorld, Simplification};

use camera::{CameraPlan, CameraSensor};
use raster::{Grid, Scene};
use scalar::{Mount, NumericSensor, Outputs};
use settings::{CameraMode, Fields};

pub use camera::FrameTruthReader;
pub use catalog::{
    Parameter, Passport, PassportKind, SensorCatalog, SensorKind, SensorSpec, SourceKind,
    SourcedNumber, SupplyRange, Unknown,
};
pub use optics::{Intrinsics, Pose};
pub use vision::{
    Detection, FrameTruth, ObjectPose, PerceptionScore, PoseError, checkerboard, compare_detections,
};

/// Выходы установленного датчика, в порядке осей или форматов.
#[derive(Clone)]
pub struct SensorHandle {
    name: String,
    kind: SensorKind,
    signals: Vec<(&'static str, SignalId)>,
    camera: Option<CameraPlan>,
}

impl SensorHandle {
    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn kind(&self) -> SensorKind {
        self.kind
    }

    pub fn signals(&self) -> &[(&'static str, SignalId)] {
        &self.signals
    }

    pub fn signal(&self, suffix: &str) -> Option<SignalId> {
        self.signals
            .iter()
            .find_map(|&(name, id)| (name == suffix).then_some(id))
    }

    pub fn intrinsics(&self) -> Option<&Intrinsics> {
        self.camera.as_ref().map(|plan| &plan.camera.lens)
    }

    pub fn frame_truth_reader(&self) -> Option<FrameTruthReader> {
        self.camera
            .as_ref()
            .map(|plan| FrameTruthReader::new(plan.clone(), self.signals[0].1))
    }
}

enum Model {
    Numeric(Box<NumericSensor>),
    Camera(Box<CameraPlan>),
}

struct Prepared {
    name: String,
    kind: SensorKind,
    rate: Rate,
    outputs: Outputs,
    model: Model,
}

/// Подключает датчики до запуска рантайма. world — тот же мир, из которого
/// создан runtime. Модели приводов к этому моменту должны зарегистрировать каналы истины.
pub fn install(
    runtime: &mut Runtime,
    world: &CompiledWorld,
    catalog: &SensorCatalog,
) -> Result<Vec<SensorHandle>, String> {
    if world.sensors.is_empty() {
        return Ok(Vec::new());
    }
    let sim = Sim::from_xml(&world.mjcf)?;
    check_world(runtime, world, &sim)?;
    let mut names = BTreeSet::new();
    let mut signals = BTreeSet::new();
    let mut models = Vec::new();
    let mut simplifications = Vec::new();
    let mut scene = None;
    let color = world.sensors.iter().any(|s| {
        catalog
            .get(&s.part)
            .is_some_and(|p| p.kind == SensorKind::ColorCamera)
    });
    for sensor in &world.sensors {
        if sensor.name.trim().is_empty() || !names.insert(&sensor.name) {
            return Err(format!(
                "пустое или повторное имя датчика «{}»",
                sensor.name
            ));
        }
        let prepare = || -> Result<_, String> {
            let spec = catalog
                .get(&sensor.part)
                .ok_or_else(|| format!("неизвестная позиция каталога «{}»", sensor.part))?;
            let kind = spec.kind;
            if sensor.joint.is_some() && kind != SensorKind::Encoder {
                return Err("этот тип датчика не использует joint".into());
            }
            if sensor.drive.is_some()
                && !matches!(
                    kind,
                    SensorKind::Current | SensorKind::Temperature | SensorKind::BridgeAdc
                )
            {
                return Err("этот тип датчика не использует drive".into());
            }
            let fields = Fields::new(spec, &sensor.params)?;
            let settings = fields.settings()?;
            let mount = Mount::new(&sim, sensor, &settings)?;
            Ok((kind, fields, settings, mount))
        };
        let (kind, fields, settings, mount) =
            prepare().map_err(|e| with_sensor(&sensor.name, e))?;
        if settings.unverified {
            simplifications.push(note(&sensor.name, "Параметры экземпляра без происхождения или оценки каталога не дают положительного доказательства."));
        }
        let physics_hz = world.physics.as_hz() as u64;
        // Целые частоты компилятора совпадают по фазе, если период датчика кратен периоду физики.
        // Для произвольной дробной частоты Rate не предоставляет числитель и знаменатель.
        let aligned = world.physics == settings.rate
            || (Rate::hz(physics_hz) == Some(world.physics)
                && physics_hz.is_multiple_of(settings.rate.as_hz() as u64));
        if !kind.camera() && !aligned {
            simplifications.push(note(&sensor.name, "Между тактами физики датчик использует предыдущее состояние. Показания между тактами и связанная с ними погрешность не проверены."));
        }
        if let Some(text) = match kind {
            SensorKind::ColorCamera => Some(
                "Цвет и освещение упрощены. Для положительного вывода по зрению требуется сверка с настоящей камерой.",
            ),
            SensorKind::DepthCamera => Some(
                "Оптическая глубина идеальных поверхностей; не учтены свойства материала, рассеяние и систематика настоящей камеры.",
            ),
            SensorKind::Temperature => Some(
                "Наблюдается температура обмотки из модели привода; теплопередача от обмотки до места датчика не посчитана.",
            ),
            SensorKind::Rangefinder => Some(
                "Дальность одного луча; диаграмма направленности, свойства поверхности и возврат сигнала не посчитаны.",
            ),
            SensorKind::Pressure | SensorKind::Touch => Some(
                "Давление и касание рассчитаны по силам точек MuJoCo в заданной области; распределение напряжений в площадке не посчитано.",
            ),
            SensorKind::Force | SensorKind::Torque => Some(
                "Рассчитана результирующая реакция соединения; деформация чувствительного элемента и его связь с изделием не посчитаны.",
            ),
            _ => None,
        } {
            simplifications.push(note(&sensor.name, text));
        }

        let (rate, model, outputs) = if kind.camera() {
            let camera = fields.camera().map_err(|e| with_sensor(&sensor.name, e))?;
            if settings.rate.as_hz() > world.physics.as_hz() {
                return Err(with_sensor(
                    &sensor.name,
                    "частота камеры превышает частоту опорных состояний физики".into(),
                ));
            }
            if camera.lens.height == 1 && camera.readout.0 != 0 {
                return Err(with_sensor(
                    &sensor.name,
                    "у одной строки readout_s должен быть нулевым".into(),
                ));
            }
            if scene.is_none() {
                scene = Some(Arc::new(
                    Scene::new(&sim, color).map_err(|e| with_sensor(&sensor.name, e))?,
                ));
            }
            let grid = Arc::new(Grid::new(camera.lens).map_err(|e| with_sensor(&sensor.name, e))?);
            let output = match camera.mode {
                CameraMode::Color { .. } => ("image", "RGB8"),
                CameraMode::Depth { .. } => ("depth", "mm"),
            };
            let plan = CameraPlan {
                mount,
                camera,
                settings,
                physics: world.physics,
                scene: Arc::clone(scene.as_ref().expect("сцена создана")),
                grid,
            };
            (world.physics, Model::Camera(Box::new(plan)), vec![output])
        } else {
            let rate = settings.rate;
            let (model, outputs) =
                NumericSensor::prepare(kind, sensor, settings, &fields, mount, runtime, &sim)
                    .map_err(|e| with_sensor(&sensor.name, e))?;
            (rate, Model::Numeric(Box::new(model)), outputs)
        };
        for &(suffix, _) in &outputs {
            let name = format!("{}/{suffix}", sensor.name);
            if runtime.observation_id(&name).is_some() || !signals.insert(name.clone()) {
                return Err(format!("сигнал «{name}» уже зарегистрирован"));
            }
        }
        models.push(Prepared {
            name: sensor.name.clone(),
            kind,
            rate,
            outputs,
            model,
        });
    }

    let snapshot = runtime.snapshot();
    let handles = match register(runtime, models) {
        Ok(handles) => handles,
        Err(reason) => {
            runtime.restore(&snapshot)?;
            return Err(reason);
        }
    };
    for item in simplifications {
        runtime.add_simplification(item)?;
    }
    Ok(handles)
}

fn register(runtime: &mut Runtime, models: Vec<Prepared>) -> Result<Vec<SensorHandle>, String> {
    let mut handles = Vec::with_capacity(models.len());
    for Prepared {
        name,
        kind,
        rate,
        outputs,
        model,
    } in models
    {
        let signals = outputs
            .into_iter()
            .map(|(suffix, unit)| {
                runtime
                    .register_observation(&format!("{name}/{suffix}"), unit)
                    .map(|id| (suffix, id))
            })
            .collect::<Result<Vec<_>, _>>()?;
        let component = format!("датчик/{name}");
        let camera = match model {
            Model::Numeric(mut model) => {
                model.bind(signals.iter().map(|&(_, id)| id).collect());
                runtime.register_sensor(&component, rate, *model)?;
                None
            }
            Model::Camera(plan) => {
                runtime.register_sensor(
                    &component,
                    rate,
                    CameraSensor::new((*plan).clone(), signals[0].1),
                )?;
                Some(*plan)
            }
        };
        handles.push(SensorHandle {
            name,
            kind,
            signals,
            camera,
        });
    }
    Ok(handles)
}

fn note(subject: &str, text: &str) -> Simplification {
    Simplification {
        subject: subject.into(),
        text: text.into(),
        blocks_proof: true,
    }
}

fn with_sensor(name: &str, reason: String) -> String {
    format!("датчик «{name}»: {reason}")
}

fn check_world(runtime: &Runtime, world: &CompiledWorld, sim: &Sim) -> Result<(), String> {
    let joints = sim.joints();
    let mut channels = Vec::new();
    for effort in &world.efforts {
        let joint = joints
            .iter()
            .find(|joint| joint.name == effort.joint)
            .ok_or_else(|| format!("неизвестное соединение «{}»", effort.joint))?;
        let dof = joint
            .dof_adr
            .checked_add(effort.dof)
            .ok_or("индекс степени свободы вне диапазона")?;
        channels.push(dof);
    }
    // Формат world_hash из LogHeader рантайма: позы и номера ниже должны быть
    // прочитаны из той же MJCF-модели, иначе датчик может тихо измерить другое тело.
    let version = sim_mujoco::version();
    let hash = fnv1a(
        (world.name.len() as u64)
            .to_le_bytes()
            .into_iter()
            .chain(world.name.bytes())
            .chain((version.len() as u64).to_le_bytes())
            .chain(version.bytes())
            .chain((world.mjcf.len() as u64).to_le_bytes())
            .chain(world.mjcf.bytes())
            .chain(channels.iter().flat_map(|dof| (*dof as u64).to_le_bytes())),
    );
    if hash != runtime.log().header().world_hash
        || runtime.seed() != world.seed
        || runtime.physics_rate() != world.physics
    {
        return Err("датчики устанавливаются в рантайм другого мира".into());
    }
    Ok(())
}
