use std::ptr;

use nalgebra::{Point3, Translation3, UnitQuaternion};
use sim_mujoco::{JointKind, Sim, ffi};
use sim_runtime::{ObservationOut, Runtime, Sensor, Truth, TruthId};
use sim_signal::{Rng, SignalId, Time};
use sim_world::CompiledSensor;

use crate::SensorKind;
use crate::optics::{Frame, Pose, Vec3, checked};
use crate::pipeline::{self, Drift, SampleMode};
use crate::settings::{Fields, Settings, add_time};

#[derive(Clone)]
pub(crate) struct Mount {
    pub body: Option<usize>,
    local: Frame,
}

impl Mount {
    pub fn new(sim: &Sim, sensor: &CompiledSensor, settings: &Settings) -> Result<Self, String> {
        let body = sensor
            .body
            .as_ref()
            .map(|name| {
                sim.id(ffi::mjtObj::mjOBJ_BODY, name)
                    .ok_or_else(|| format!("неизвестное тело «{name}»"))
            })
            .transpose()?;
        let nominal = Pose {
            position_m: sensor.position_m,
            quat_wxyz: sensor.quat_wxyz,
        }
        .frame()?;
        let local = match settings.mount {
            Some((translation, [roll, pitch, yaw])) => {
                nominal
                    * Frame::from_parts(
                        Translation3::from(translation),
                        UnitQuaternion::from_euler_angles(roll, pitch, yaw),
                    )
            }
            None => nominal,
        };
        checked(local.translation.vector.as_slice(), "место установки")?;
        Ok(Self { body, local })
    }

    pub fn frame(&self, sim: &Sim) -> Result<Frame, String> {
        self.with_base(&self.base_frame(sim)?)
    }

    pub fn base_frame(&self, sim: &Sim) -> Result<Frame, String> {
        let Some(body) = self.body else {
            return Ok(Frame::identity());
        };
        if body >= sim.nbody() {
            return Err("тело датчика отсутствует в движке".into());
        }
        let (position_m, quat_wxyz) = sim.body_pose(body);
        Pose {
            position_m,
            quat_wxyz,
        }
        .frame()
    }

    pub fn with_base(&self, base: &Frame) -> Result<Frame, String> {
        let frame = base * self.local;
        checked(frame.translation.vector.as_slice(), "поза датчика")?;
        Ok(frame)
    }
}

#[derive(Clone)]
enum Encoder {
    Direct,
    Absolute(f64),
    Relative(Option<f64>),
}

#[derive(Clone)]
enum Input {
    Encoder {
        joint: String,
        zero: f64,
        mode: Encoder,
    },
    Plant(TruthId),
    Imu,
    Reaction {
        torque: bool,
    },
    Pressure {
        radius: f64,
        area: f64,
    },
    Touch {
        radius: f64,
        threshold: f64,
    },
    Range {
        min: f64,
        max: f64,
    },
}

#[derive(Clone)]
pub(crate) struct NumericSensor {
    input: Input,
    mount: Mount,
    settings: Settings,
    signals: Vec<SignalId>,
    drift: Vec<Drift>,
    last: Vec<Option<f64>>,
    previous: Time,
}

pub(crate) type Outputs = Vec<(&'static str, &'static str)>;

impl NumericSensor {
    pub fn prepare(
        kind: SensorKind,
        sensor: &CompiledSensor,
        settings: Settings,
        fields: &Fields<'_>,
        mount: Mount,
        runtime: &Runtime,
        sim: &Sim,
    ) -> Result<(Self, Outputs), String> {
        let vector = |unit| vec![("x", unit), ("y", unit), ("z", unit)];
        let scalar = |unit| vec![("value", unit)];
        let (input, outputs) = match kind {
            SensorKind::Encoder => {
                let joint = sensor.joint.as_ref().ok_or("энкодеру нужно соединение")?;
                let model_joint = sim
                    .joints()
                    .into_iter()
                    .find(|j| j.name == *joint)
                    .ok_or_else(|| format!("неизвестное соединение «{joint}»"))?;
                let unit = match model_joint.kind {
                    JointKind::Hinge => "rad",
                    JointKind::Slide => "m",
                    _ => return Err("энкодеру нужно одноосное соединение".into()),
                };
                // Коды формата: 0 — координата; 1 — абсолютный цикл; 2 — от первого отсчёта.
                let mode = match fields.get("encoder_mode")? as u8 {
                    1 => {
                        let period = fields.get("encoder_period")?;
                        if settings
                            .pipe
                            .bounds
                            .is_some_and(|(min, max)| min < 0.0 || max > period)
                        {
                            return Err("границы насыщения лежат вне цикла энкодера".into());
                        }
                        Encoder::Absolute(period)
                    }
                    2 => Encoder::Relative(None),
                    _ => Encoder::Direct,
                };
                (
                    Input::Encoder {
                        joint: joint.clone(),
                        zero: fields.get("encoder_zero")?,
                        mode,
                    },
                    scalar(unit),
                )
            }
            SensorKind::Current | SensorKind::Temperature | SensorKind::BridgeAdc => {
                let (channel, unit) = match kind {
                    SensorKind::Current => ("current_a", "A"),
                    SensorKind::Temperature => ("winding_c", "degC"),
                    _ => ("bridge_mv", "mV"),
                };
                let drive = sensor
                    .drive
                    .as_ref()
                    .ok_or("датчику нужен источник модели привода")?;
                let name = format!("{drive}/{channel}");
                let id = runtime
                    .truth_id(&name)
                    .ok_or_else(|| format!("канал истины «{name}» не зарегистрирован"))?;
                (Input::Plant(id), scalar(unit))
            }
            SensorKind::Imu => {
                let mut axes = vector("m/s^2");
                axes.extend([("gx", "rad/s"), ("gy", "rad/s"), ("gz", "rad/s")]);
                (Input::Imu, axes)
            }
            SensorKind::Force | SensorKind::Torque => {
                if mount.body.is_none_or(|body| body == 0) {
                    return Err("датчику реакции нужно тело с родительским соединением".into());
                }
                let torque = kind == SensorKind::Torque;
                (
                    Input::Reaction { torque },
                    vector(if torque { "N*m" } else { "N" }),
                )
            }
            SensorKind::Pressure => (
                Input::Pressure {
                    radius: fields.get("patch_radius_m")?,
                    area: fields.get("patch_area_m2")?,
                },
                scalar("Pa"),
            ),
            SensorKind::Touch => (
                Input::Touch {
                    radius: fields.get("patch_radius_m")?,
                    threshold: fields.get("touch_threshold_n")?,
                },
                scalar("1"),
            ),
            SensorKind::Rangefinder => {
                let min = fields.get("range_min_m")?;
                let max = fields.get("range_max_m")?;
                if min >= max {
                    return Err("неверный диапазон дальномера".into());
                }
                (Input::Range { min, max }, scalar("m"))
            }
            _ => return Err("камере нужен кадровый конвейер".into()),
        };
        let count = outputs.len();
        Ok((
            Self {
                input,
                mount,
                settings,
                signals: Vec::new(),
                drift: vec![Drift::default(); count],
                last: vec![None; count],
                previous: Time::ZERO,
            },
            outputs,
        ))
    }

    pub fn bind(&mut self, signals: Vec<SignalId>) {
        self.signals = signals;
    }

    fn raw(&mut self, truth: &Truth<'_>) -> Result<Option<Vec<f64>>, String> {
        let sim = truth.engine();
        match &mut self.input {
            Input::Encoder { joint, zero, mode } => {
                let (coordinates, _) = truth
                    .joint(joint)
                    .ok_or("соединение датчика отсутствует в рантайме")?;
                if coordinates.len() != 1 {
                    return Err("размерность координаты энкодера изменилась".into());
                }
                let q = coordinates[0];
                checked(&[q], "координата энкодера")?;
                let raw = match mode {
                    Encoder::Direct | Encoder::Absolute(_) => q - *zero,
                    Encoder::Relative(origin) => q - *origin.get_or_insert(q) - *zero,
                };
                Ok(Some(vec![raw]))
            }
            Input::Plant(id) => Ok(truth.plant(*id).map(|v| vec![v])),
            input => {
                let frame = self.mount.frame(sim)?;
                let body = self.mount.body;
                match *input {
                    Input::Imu => imu(sim, body, &frame).map(Some),
                    Input::Reaction { torque } => {
                        let body = body.ok_or("датчик реакции без тела")?;
                        reaction(sim, body, &frame, torque).map(|v| Some(v.as_slice().to_vec()))
                    }
                    Input::Pressure { radius, area } => {
                        let force =
                            patch_force(sim, body.unwrap_or(0), frame.translation.vector, radius)?;
                        Ok(Some(vec![force / area]))
                    }
                    Input::Touch { radius, threshold } => {
                        let force =
                            patch_force(sim, body.unwrap_or(0), frame.translation.vector, radius)?;
                        Ok(Some(vec![if force >= threshold { 1.0 } else { 0.0 }]))
                    }
                    Input::Range { min, max } => {
                        let ray = frame.rotation * Vec3::z();
                        let origin = frame.translation.vector;
                        // Читающий вызов MuJoCo по живым model/data; указатели на два массива длины 3.
                        let distance = unsafe {
                            ffi::mj_ray(
                                sim.model(),
                                sim.data(),
                                origin.as_ptr(),
                                ray.as_ptr(),
                                ptr::null(),
                                true,
                                body.map_or(-1, |id| id as i32),
                                ptr::null_mut(),
                                ptr::null_mut(),
                            )
                        };
                        checked(&[distance], "дальность луча")?;
                        Ok((min..=max).contains(&distance).then(|| vec![distance]))
                    }
                    _ => Err("неизвестный источник показания".into()),
                }
            }
        }
    }
}

impl Sensor for NumericSensor {
    fn sample(
        &mut self,
        truth: &Truth<'_>,
        rng: &mut Rng,
        out: &mut ObservationOut,
    ) -> Result<(), String> {
        let t = truth.time();
        let delay = add_time(t, self.settings.latency)?;
        let raw = self.raw(truth)?;
        if let Some(raw) = &raw {
            checked(raw, "физическое показание")?;
        }
        let dt = Time(
            t.0.checked_sub(self.previous.0)
                .ok_or("время датчика пошло назад")?,
        )
        .as_secs();
        self.previous = t;
        let mode = pipeline::mode(
            self.settings.fault,
            t..=t,
            self.settings.drop_probability,
            rng,
        );
        let period = match self.input {
            Input::Encoder {
                mode: Encoder::Absolute(period),
                ..
            } => Some(period),
            _ => None,
        };
        for (axis, &signal) in self.signals.iter().enumerate() {
            let p = if axis >= 3 {
                self.settings
                    .gyro
                    .as_ref()
                    .ok_or("нет параметров гироскопа")?
            } else {
                &self.settings.pipe
            };
            let drift = self.drift[axis].advance(p, t.as_secs(), dt, rng)?;
            let value = match mode {
                SampleMode::Invalid => None,
                SampleMode::Stuck => self.last[axis],
                SampleMode::Fresh | SampleMode::Shifted => raw
                    .as_ref()
                    .map(|raw| {
                        pipeline::value(
                            raw[axis],
                            p,
                            drift,
                            mode == SampleMode::Shifted,
                            period,
                            rng,
                        )
                    })
                    .transpose()?,
            };
            let valid = value.is_some() && mode != SampleMode::Stuck;
            if valid {
                self.last[axis] = value;
            }
            if let Some(value) = value {
                out.write_at(signal, value, valid, delay);
            } else {
                // Пустой конечный заполнитель обязателен по контракту Sample, valid=false.
                out.write_at(signal, 0.0, false, delay);
            }
        }
        Ok(())
    }
}

fn imu(sim: &Sim, body: Option<usize>, frame: &Frame) -> Result<Vec<f64>, String> {
    let gravity = Vec3::from(sim.gravity());
    let mut linear = -gravity;
    let mut angular = Vec3::zeros();
    let nv = sim.nv();
    if let Some(body) = body
        && nv != 0
    {
        let mut jp = vec![0.0; 3 * nv];
        let mut jr = vec![0.0; 3 * nv];
        let mut dot = vec![0.0; 3 * nv];
        // Ускорение точки равно J q̈ + J̇ q̇, угловая скорость Jrot q̇.
        // mj_jac и mj_jacDot читают свежую кинематику и заполняют 3×nv элементов.
        let acceleration = unsafe {
            let point = frame.translation.vector.as_ptr();
            ffi::mj_jac(
                sim.model(),
                sim.data(),
                jp.as_mut_ptr(),
                jr.as_mut_ptr(),
                point,
                body as i32,
            );
            ffi::mj_jacDot(
                sim.model(),
                sim.data(),
                dot.as_mut_ptr(),
                ptr::null_mut(),
                point,
                body as i32,
            );
            std::slice::from_raw_parts(sim.data().qacc, nv)
        };
        checked(acceleration, "обобщённое ускорение IMU")?;
        for axis in 0..3 {
            for (k, (&velocity, &acceleration)) in sim.qvel().iter().zip(acceleration).enumerate() {
                let i = axis * nv + k;
                linear[axis] += jp[i] * acceleration + dot[i] * velocity;
                angular[axis] += jr[i] * velocity;
            }
        }
    }
    let a = frame.rotation.inverse_transform_vector(&linear);
    let w = frame.rotation.inverse_transform_vector(&angular);
    Ok(vec![a.x, a.y, a.z, w.x, w.y, w.z])
}

struct Scratch(*mut ffi::mjData);

impl Drop for Scratch {
    fn drop(&mut self) {
        // Выделено mj_copyData, частная копия состояния.
        unsafe { ffi::mj_deleteData(self.0) }
    }
}

fn reaction(sim: &Sim, body: usize, frame: &Frame, torque: bool) -> Result<Vec3, String> {
    let model = sim.model();
    // mj_rnePostConstraint изменяет данные. Частная копия сохраняет read-only контракт Truth.
    let raw = unsafe { ffi::mj_copyData(ptr::null_mut(), model, sim.data()) };
    if raw.is_null() {
        return Err("MuJoCo не выделил копию для расчёта реакции".into());
    }
    let data = Scratch(raw);
    let (force, moment, origin) = unsafe {
        ffi::mj_rnePostConstraint(model, data.0);
        let wrench = std::slice::from_raw_parts((*data.0).cfrc_int.add(6 * body), 6);
        let root = *model.body_rootid.add(body) as usize;
        let com = std::slice::from_raw_parts((*data.0).subtree_com.add(3 * root), 3);
        (
            Vec3::from_row_slice(&wrench[3..]),
            Vec3::from_row_slice(&wrench[..3]),
            Point3::from_slice(com),
        )
    };
    // cfrc_int — (момент, сила) на тело со стороны родителя, в мировых осях
    // с центром в центре масс кинематического дерева (COM-based spatial convention MuJoCo).
    let current = if torque {
        moment + (origin.coords - frame.translation.vector).cross(&force)
    } else {
        force
    };
    Ok(frame.rotation.inverse_transform_vector(&current))
}

fn patch_force(sim: &Sim, body: usize, center: Vec3, radius: f64) -> Result<f64, String> {
    let mut force = 0.0;
    let model = sim.model();
    for contact in sim.contacts() {
        checked(&contact.pos, "место контакта")?;
        checked(&[contact.normal_force], "сила контакта")?;
        if (Vec3::from(contact.pos) - center).norm() > radius {
            continue;
        }
        let participants = contact.geom.map(|id| {
            (id >= 0 && (id as usize) < sim.ngeom()).then(|| {
                // geom_bodyid содержит элемент для каждой геометрии модели.
                unsafe { *model.geom_bodyid.add(id as usize) as usize }
            })
        });
        if participants.contains(&None) {
            return Err("контакт с элементом без жёсткой геометрии не поддержан".into());
        }
        if participants[0] != participants[1] && participants.contains(&Some(body)) {
            force += contact.normal_force.max(0.0);
        }
    }
    Ok(force)
}
