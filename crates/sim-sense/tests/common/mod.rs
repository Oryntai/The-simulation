#![allow(dead_code)]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use sim_runtime::{LogKind, Runtime, Status};
use sim_sense::{Parameter, SensorCatalog, SensorKind, SensorSpec, SourceKind, SourcedNumber};
use sim_signal::{Rate, SignalId, Time};
use sim_world::{CompiledJoint, CompiledJointKind, CompiledSensor, CompiledWorld, EffortChannel};

pub struct CatalogDir(PathBuf);

impl CatalogDir {
    pub fn new(name: &str) -> Self {
        let path = std::env::temp_dir().join(format!("sim-sense-{}-{name}", std::process::id()));
        std::fs::create_dir(&path).unwrap();
        Self(path)
    }

    pub fn write(&self, id: &str, text: &str) {
        std::fs::write(self.0.join(format!("{id}.toml")), text).unwrap();
    }

    pub fn add(&self, id: &str, kind: SensorKind, fields: &[(&str, f64)]) {
        // Явная идеальная калибровка синтетического стенда: нет ни отказов, ни возмущений.
        let mut model = BTreeMap::from([
            ("rate_hz", 100.0),
            ("latency_s", 0.0),
            ("drop_probability", 0.0),
            ("fault", 0.0),
            ("noise_std", 0.0),
            ("bias", 0.0),
            ("drift_per_s", 0.0),
            ("drift_rw_std", 0.0),
            ("quantum", 0.0),
            ("saturation", 0.0),
        ]);
        if !matches!(
            kind,
            SensorKind::Encoder
                | SensorKind::Current
                | SensorKind::Temperature
                | SensorKind::BridgeAdc
        ) {
            model.extend([
                ("mount_dx_m", 0.0),
                ("mount_dy_m", 0.0),
                ("mount_dz_m", 0.0),
                ("mount_roll_rad", 0.0),
                ("mount_pitch_rad", 0.0),
                ("mount_yaw_rad", 0.0),
            ]);
        }
        if kind == SensorKind::Imu {
            model.extend([
                ("gyro_noise_std", 0.0),
                ("gyro_bias", 0.0),
                ("gyro_drift_per_s", 0.0),
                ("gyro_drift_rw_std", 0.0),
                ("gyro_quantum", 0.0),
                ("gyro_saturation", 0.0),
            ]);
        }
        if kind == SensorKind::Encoder {
            model.extend([("encoder_mode", 0.0), ("encoder_zero", 0.0)]);
        }
        if matches!(kind, SensorKind::ColorCamera | SensorKind::DepthCamera) {
            model.extend([
                ("width", 64.0),
                ("height", 48.0),
                ("fx", 80.0),
                ("fy", 80.0),
                ("cx", 32.0),
                ("cy", 24.0),
                ("near_m", 0.01),
                ("far_m", 10.0),
                ("k1", 0.0),
                ("k2", 0.0),
                ("k3", 0.0),
                ("p1", 0.0),
                ("p2", 0.0),
                ("exposure_s", 0.0),
                ("readout_s", 0.0),
                ("exposure_samples", 1.0),
            ]);
        }
        if kind == SensorKind::ColorCamera {
            model.extend([("ambient", 1.0), ("spatial_samples", 1.0)]);
        }
        if kind == SensorKind::DepthCamera {
            model.insert("hole_probability", 0.0);
        }
        model.extend(fields.iter().copied());
        let spec = SensorSpec {
            id: id.into(),
            name: format!("Синтетический датчик {id}"),
            kind,
            passport: None,
            model: model
                .into_iter()
                .map(|(key, value)| {
                    (
                        key.into(),
                        Parameter::Known(SourcedNumber {
                            value,
                            provenance: SourceKind::Measured,
                            source: "Калибровка синтетического стенда из условий теста".into(),
                        }),
                    )
                })
                .collect(),
        };
        self.write(id, &toml::to_string(&spec).unwrap());
    }

    pub fn path(&self) -> &Path {
        &self.0
    }

    pub fn catalog(&self) -> SensorCatalog {
        SensorCatalog::load(self.path()).unwrap()
    }
}

impl Drop for CatalogDir {
    fn drop(&mut self) {
        std::fs::remove_dir_all(&self.0).unwrap();
    }
}

pub fn sensor(name: &str, part: &str) -> CompiledSensor {
    CompiledSensor {
        name: name.into(),
        part: part.into(),
        body: None,
        position_m: [0.0; 3],
        quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        joint: None,
        drive: None,
        params: BTreeMap::new(),
    }
}

pub fn world(geometry: &str, gravity: &str, sensors: Vec<CompiledSensor>) -> CompiledWorld {
    // Шаг стенда 1 мс, Эйлер для ручной сверки.
    CompiledWorld {
        name: "стенд".into(),
        seed: 417,
        physics: Rate::hz(1000).unwrap(),
        mjcf: format!(
            r#"<mujoco><option timestep="0.001" gravity="{gravity}" integrator="Euler"/>
                         <worldbody>{geometry}</worldbody></mujoco>"#
        ),
        bodies: vec![],
        joints: vec![],
        efforts: vec![],
        drives: vec![],
        sensors,
        simplifications: vec![],
    }
}

pub fn effort(world: &mut CompiledWorld, kind: CompiledJointKind) {
    world.joints.push(CompiledJoint {
        name: "ось".into(),
        owner: "стенд".into(),
        kind,
    });
    world.efforts.push(EffortChannel {
        name: "вход".into(),
        joint: "ось".into(),
        dof: 0,
    });
}

pub fn run(runtime: &mut Runtime, time_s: f64) {
    assert_eq!(
        runtime.run_until(Time::from_secs(time_s).unwrap()).unwrap(),
        Status::Running,
        "{:?}",
        runtime.failure()
    );
}

pub fn observations(runtime: &Runtime, id: SignalId) -> Vec<(Time, f64, bool)> {
    runtime
        .log()
        .entries()
        .iter()
        .filter(|e| e.kind == LogKind::Observation && e.id == id.0)
        .map(|e| (e.at, e.value, e.valid))
        .collect()
}

pub fn near(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual:.10} != {expected:.10}, допуск {tolerance}"
    );
}

pub const SLIDE: &str = r#"<body name="каретка">
    <joint name="ось" type="slide" axis="1 0 0"/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    </body>"#;
