use std::collections::BTreeMap;

use sim_signal::{Rate, Time};

use crate::catalog::{SensorKind, SensorSpec};
use crate::optics::Intrinsics;

const PIPE: &[&str] = &[
    "noise_std",
    "bias",
    "drift_per_s",
    "drift_rw_std",
    "quantum",
    "saturation",
    "min",
    "max",
    "fault_offset",
];
const PLACEMENT: &[&str] = &[
    "mount_dx_m",
    "mount_dy_m",
    "mount_dz_m",
    "mount_roll_rad",
    "mount_pitch_rad",
    "mount_yaw_rad",
];
const CAMERA: &[&str] = &[
    "width",
    "height",
    "fx",
    "fy",
    "cx",
    "cy",
    "k1",
    "k2",
    "k3",
    "p1",
    "p2",
    "near_m",
    "far_m",
    "exposure_s",
    "readout_s",
    "exposure_samples",
];

pub(crate) fn check_fields<'a>(
    kind: SensorKind,
    fields: impl IntoIterator<Item = (&'a str, Option<f64>)>,
) -> Result<(), String> {
    let mut numbers = BTreeMap::new();
    for (key, v) in fields {
        let pipe = if kind == SensorKind::Imu {
            key.strip_prefix("gyro_").unwrap_or(key)
        } else {
            key
        };
        let known = PIPE.contains(&pipe)
            || matches!(
                key,
                "rate_hz" | "latency_s" | "drop_probability" | "fault" | "fault_at_s"
            )
            || (kind.spatial() && PLACEMENT.contains(&key))
            || (kind.camera() && CAMERA.contains(&key))
            || match kind {
                SensorKind::ColorCamera => matches!(key, "ambient" | "spatial_samples"),
                SensorKind::DepthCamera => key == "hole_probability",
                SensorKind::Encoder => {
                    matches!(key, "encoder_mode" | "encoder_period" | "encoder_zero")
                }
                SensorKind::Pressure => matches!(key, "patch_radius_m" | "patch_area_m2"),
                SensorKind::Touch => matches!(key, "patch_radius_m" | "touch_threshold_n"),
                SensorKind::Rangefinder => matches!(key, "range_min_m" | "range_max_m"),
                _ => false,
            };
        if !known {
            return Err(format!("неизвестный параметр «{key}»"));
        }
        let Some(v) = v else { continue };
        let valid = v.is_finite()
            && match pipe {
                "width" | "height" | "exposure_samples" | "spatial_samples" => {
                    v.fract() == 0.0 && v >= 1.0 && v <= f64::from(u32::MAX)
                }
                "rate_hz" => v.fract() == 0.0 && v >= 1.0 && Rate::hz(v as u64).is_some(),
                "latency_s" | "fault_at_s" | "readout_s" | "exposure_s" => causal_time(v).is_some(),
                "noise_std" | "drift_rw_std" | "quantum" | "range_min_m" => v >= 0.0,
                "drop_probability" | "hole_probability" | "ambient" => (0.0..=1.0).contains(&v),
                "fx" | "fy" | "near_m" | "far_m" | "encoder_period" | "patch_radius_m"
                | "patch_area_m2" | "touch_threshold_n" | "range_max_m" => v > 0.0,
                "saturation" => v == 0.0 || v == 1.0,
                "encoder_mode" => v.fract() == 0.0 && (0.0..=2.0).contains(&v),
                "fault" => v.fract() == 0.0 && (0.0..=3.0).contains(&v),
                _ => true,
            };
        if !valid {
            return Err(format!("недопустимое значение «{key}»: {v}"));
        }
        numbers.insert(key, v);
    }
    for (min, max) in [
        ("min", "max"),
        ("gyro_min", "gyro_max"),
        ("range_min_m", "range_max_m"),
        ("near_m", "far_m"),
    ] {
        if numbers
            .get(min)
            .zip(numbers.get(max))
            .is_some_and(|(a, b)| a >= b)
        {
            return Err(format!("«{min}» должна быть меньше «{max}»"));
        }
    }
    Ok(())
}

#[derive(Clone, Debug)]
pub(crate) struct PipeSettings {
    pub noise_std: f64,
    pub bias: f64,
    pub drift_per_s: f64,
    pub drift_rw_std: f64,
    pub quantum: f64,
    pub bounds: Option<(f64, f64)>,
    pub shift: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Fault {
    None,
    Stuck(Time),
    Open(Time),
    Shift(Time),
}

#[derive(Clone, Debug)]
pub(crate) struct Settings {
    pub rate: Rate,
    pub latency: Time,
    pub drop_probability: f64,
    pub fault: Fault,
    pub pipe: PipeSettings,
    pub gyro: Option<PipeSettings>,
    pub mount: Option<([f64; 3], [f64; 3])>,
    pub unverified: bool,
}

#[derive(Clone, Debug)]
pub(crate) struct CameraSettings {
    pub lens: Intrinsics,
    pub near: f64,
    pub far: f64,
    pub exposure: Time,
    pub readout: Time,
    pub samples: u32,
    pub mode: CameraMode,
}

#[derive(Clone, Copy, Debug)]
pub(crate) enum CameraMode {
    Color { ambient: f64, spatial_samples: u32 },
    Depth { hole_probability: f64 },
}

pub(crate) struct Fields<'a> {
    spec: &'a SensorSpec,
    overrides: &'a BTreeMap<String, f64>,
}

impl<'a> Fields<'a> {
    pub fn new(spec: &'a SensorSpec, overrides: &'a BTreeMap<String, f64>) -> Result<Self, String> {
        check_fields(
            spec.kind,
            overrides.iter().map(|(k, &v)| (k.as_str(), Some(v))),
        )?;
        Ok(Self { spec, overrides })
    }

    pub fn get(&self, key: &str) -> Result<f64, String> {
        self.overrides
            .get(key)
            .copied()
            .or_else(|| self.spec.model.get(key).and_then(|p| p.value()))
            .ok_or_else(|| format!("параметр «{key}» обязателен, значение неизвестно"))
    }

    fn time(&self, key: &str) -> Result<Time, String> {
        // Размерности и диапазон времени уже проверены при разборе.
        causal_time(self.get(key)?).ok_or_else(|| format!("неверное время «{key}»"))
    }

    pub fn settings(&self) -> Result<Settings, String> {
        // Коды формата: 0 — нет отказа, 1 — залипание последнего показания,
        // 2 — обрыв, 3 — ступенчатое смещение.
        let fault = match self.get("fault")? as u8 {
            1 => Fault::Stuck(self.time("fault_at_s")?),
            2 => Fault::Open(self.time("fault_at_s")?),
            3 => Fault::Shift(self.time("fault_at_s")?),
            _ => Fault::None,
        };
        let shift = matches!(fault, Fault::Shift(_));
        let mount = if self.spec.kind.spatial() {
            Some((
                [
                    self.get("mount_dx_m")?,
                    self.get("mount_dy_m")?,
                    self.get("mount_dz_m")?,
                ],
                [
                    self.get("mount_roll_rad")?,
                    self.get("mount_pitch_rad")?,
                    self.get("mount_yaw_rad")?,
                ],
            ))
        } else {
            None
        };
        Ok(Settings {
            rate: Rate::hz(self.get("rate_hz")? as u64).ok_or("неверная частота датчика")?,
            latency: self.time("latency_s")?,
            drop_probability: self.get("drop_probability")?,
            fault,
            pipe: self.pipe("", shift)?,
            gyro: (self.spec.kind == SensorKind::Imu)
                .then(|| self.pipe("gyro_", shift))
                .transpose()?,
            mount,
            unverified: !self.overrides.is_empty()
                || self
                    .spec
                    .model
                    .values()
                    .any(|p| p.value().is_some() && !p.can_prove()),
        })
    }

    fn pipe(&self, prefix: &str, shift: bool) -> Result<PipeSettings, String> {
        let get = |key| self.get(&format!("{prefix}{key}"));
        let bounds = if get("saturation")? == 1.0 {
            let (min, max) = (get("min")?, get("max")?);
            if min >= max {
                return Err(format!("неверные границы насыщения «{prefix}min/max»"));
            }
            Some((min, max))
        } else {
            None
        };
        Ok(PipeSettings {
            noise_std: get("noise_std")?,
            bias: get("bias")?,
            drift_per_s: get("drift_per_s")?,
            drift_rw_std: get("drift_rw_std")?,
            quantum: get("quantum")?,
            bounds,
            shift: if shift { get("fault_offset")? } else { 0.0 },
        })
    }

    pub fn camera(&self) -> Result<CameraSettings, String> {
        let lens = Intrinsics {
            width: self.get("width")? as u32,
            height: self.get("height")? as u32,
            fx: self.get("fx")?,
            fy: self.get("fy")?,
            cx: self.get("cx")?,
            cy: self.get("cy")?,
            radial: [self.get("k1")?, self.get("k2")?, self.get("k3")?],
            tangential: [self.get("p1")?, self.get("p2")?],
        };
        lens.validate()?;
        let near = self.get("near_m")?;
        let far = self.get("far_m")?;
        if near >= far {
            return Err("near_m должна быть меньше far_m".into());
        }
        let mode = if self.spec.kind == SensorKind::ColorCamera {
            CameraMode::Color {
                ambient: self.get("ambient")?,
                spatial_samples: self.get("spatial_samples")? as u32,
            }
        } else {
            // Предел представления DepthMm16: u16 в миллиметрах, 0 зарезервирован для дыр.
            if far > f64::from(u16::MAX) / 1000.0 {
                return Err("far_m превышает диапазон DepthMm16".into());
            }
            CameraMode::Depth {
                hole_probability: self.get("hole_probability")?,
            }
        };
        let settings = CameraSettings {
            lens,
            near,
            far,
            exposure: self.time("exposure_s")?,
            readout: self.time("readout_s")?,
            samples: self.get("exposure_samples")? as u32,
            mode,
        };
        settings.check_raster_budget()?;
        Ok(settings)
    }
}

impl CameraSettings {
    fn check_raster_budget(&self) -> Result<(), String> {
        let pixels = u128::from(self.lens.width) * u128::from(self.lens.height);
        let spatial = match self.mode {
            CameraMode::Color {
                spatial_samples, ..
            } => u128::from(spatial_samples),
            CameraMode::Depth { .. } => 1,
        };
        let temporal = if self.exposure == Time::ZERO {
            1
        } else {
            if u64::from(self.samples) > self.exposure.0 {
                return Err("число временных проб больше наносекунд экспозиции".into());
            }
            u128::from(self.samples)
        };
        // Защита от настройки, которая блокирует такт на миллиардах растровых проб.
        // Бюджет — 128 Ми проб за кадр: хватает для 4K с пространственной сеткой 4×4
        // или для кадра 320×240 с сеткой 16×16 и четырьмя пробами экспозиции.
        const MAX_SAMPLES: u128 = 128 * 1024 * 1024;
        let budget = pixels
            .checked_mul(spatial * spatial)
            .and_then(|n| n.checked_mul(temporal));
        if budget.is_none_or(|n| n > MAX_SAMPLES) {
            return Err("настройки камеры превышают бюджет растровых проб за кадр".into());
        }
        Ok(())
    }
}

fn causal_time(seconds: f64) -> Option<Time> {
    // Шкала Time в наносекундах. Округление вверх не сокращает причинную задержку.
    let ns = (seconds * 1_000_000_000.0).ceil();
    (seconds >= 0.0 && ns.is_finite() && ns < u64::MAX as f64).then_some(Time(ns as u64))
}

pub(crate) fn add_time(a: Time, b: Time) -> Result<Time, String> {
    a.0.checked_add(b.0)
        .map(Time)
        .ok_or_else(|| "время вышло за предел шкалы".into())
}
