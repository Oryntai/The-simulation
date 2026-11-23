use std::collections::BTreeMap;
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::settings::check_fields;

/// Происхождение числа из паспорта или модели.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SourceKind {
    Datasheet,
    Vendor,
    Paper,
    Measured,
    Derived,
    Estimate,
}

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SourcedNumber {
    pub value: f64,
    pub provenance: SourceKind,
    pub source: String,
}

/// Неопубликованный параметр записывается строкой `"unknown"`.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(untagged)]
pub enum Parameter {
    Known(SourcedNumber),
    Unknown(Unknown),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Unknown {
    Unknown,
}

impl SourcedNumber {
    fn validate(&self) -> Result<(), String> {
        if !self.value.is_finite() || self.source.trim().is_empty() {
            return Err("число должно быть конечным и иметь непустой источник".into());
        }
        Ok(())
    }
}

impl Parameter {
    pub fn value(&self) -> Option<f64> {
        match self {
            Self::Known(value) => Some(value.value),
            Self::Unknown(_) => None,
        }
    }

    pub(crate) fn can_prove(&self) -> bool {
        matches!(self, Self::Known(p) if !matches!(p.provenance, SourceKind::Estimate | SourceKind::Vendor))
    }

    fn validate(&self) -> Result<(), String> {
        match self {
            Self::Known(value) => value.validate(),
            Self::Unknown(_) => Ok(()),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorKind {
    Encoder,
    Current,
    Temperature,
    Imu,
    Force,
    Torque,
    Pressure,
    Touch,
    Rangefinder,
    ColorCamera,
    DepthCamera,
    BridgeAdc,
}

impl SensorKind {
    pub(crate) fn camera(self) -> bool {
        matches!(self, Self::ColorCamera | Self::DepthCamera)
    }

    pub(crate) fn spatial(self) -> bool {
        matches!(
            self,
            Self::Imu
                | Self::Force
                | Self::Torque
                | Self::Pressure
                | Self::Touch
                | Self::Rangefinder
                | Self::ColorCamera
                | Self::DepthCamera
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum PassportKind {
    Angle,
    Encoder,
    Imu,
    Distance,
    ForceAdc,
    LoadCell,
    ForceResistor,
    PowerMonitor,
}

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SupplyRange {
    pub min: SourcedNumber,
    pub max: SourcedNumber,
}

/// Паспорт в исходных единицах. Точность из паспорта может быть пределом, плотностью
/// шума или допустимой погрешностью и сама по себе не задаёт модель случайного процесса.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Passport {
    pub kind: PassportKind,
    pub what_is: String,
    pub supply_ranges: Vec<SupplyRange>,
    pub current_a: Parameter,
    pub current_peak_a: Parameter,
    pub range_min: Parameter,
    pub range_max: Parameter,
    pub range_unit: String,
    pub resolution: Parameter,
    pub resolution_unit: String,
    pub accuracy: Parameter,
    pub accuracy_unit: String,
    pub interface: String,
    pub sources: Vec<String>,
    pub caveats: Vec<String>,
}

impl Passport {
    fn validate(&self) -> Result<(), String> {
        for range in &self.supply_ranges {
            range.min.validate()?;
            range.max.validate()?;
            if range.min.value < 0.0 || range.min.value > range.max.value {
                return Err("неверный диапазон питания".into());
            }
        }
        for (key, value) in [
            ("current_a", &self.current_a),
            ("current_peak_a", &self.current_peak_a),
            ("range_min", &self.range_min),
            ("range_max", &self.range_max),
            ("resolution", &self.resolution),
            ("accuracy", &self.accuracy),
        ] {
            value.validate().map_err(|e| format!("{key}: {e}"))?;
        }
        if self
            .range_min
            .value()
            .zip(self.range_max.value())
            .is_some_and(|(a, b)| a > b)
            || [
                &self.current_a,
                &self.current_peak_a,
                &self.resolution,
                &self.accuracy,
            ]
            .iter()
            .any(|p| p.value().is_some_and(|v| v < 0.0))
        {
            return Err("неверный диапазон или отрицательная паспортная величина".into());
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SensorSpec {
    pub id: String,
    pub name: String,
    pub kind: SensorKind,
    pub passport: Option<Passport>,
    pub model: BTreeMap<String, Parameter>,
}

impl SensorSpec {
    pub fn parse(toml: &str) -> Result<Self, String> {
        let spec: Self = toml::from_str(toml).map_err(|e| format!("разбор датчика: {e}"))?;
        if spec.id.is_empty()
            || !spec
                .id
                .bytes()
                .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == b'-' || c == b'_')
            || spec.name.trim().is_empty()
        {
            return Err("неверный идентификатор или пустое название датчика".into());
        }
        if let Some(passport) = &spec.passport {
            passport.validate().map_err(|e| format!("паспорт: {e}"))?;
        }
        for (key, parameter) in &spec.model {
            parameter.validate().map_err(|e| format!("{key}: {e}"))?;
        }
        check_fields(
            spec.kind,
            spec.model.iter().map(|(k, p)| (k.as_str(), p.value())),
        )?;
        Ok(spec)
    }
}

#[derive(Clone, Debug)]
pub struct SensorCatalog {
    entries: BTreeMap<String, SensorSpec>,
}

impl SensorCatalog {
    pub fn load(directory: impl AsRef<Path>) -> Result<Self, String> {
        let directory = directory.as_ref();
        let read_dir =
            std::fs::read_dir(directory).map_err(|e| format!("{}: {e}", directory.display()))?;
        let mut paths = Vec::new();
        for entry in read_dir {
            let path = entry
                .map_err(|e| format!("{}: {e}", directory.display()))?
                .path();
            if path.extension().is_some_and(|x| x == "toml") {
                paths.push(path);
            }
        }
        paths.sort();
        let mut entries = BTreeMap::new();
        for path in paths {
            let text =
                std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
            let spec = SensorSpec::parse(&text).map_err(|e| format!("{}: {e}", path.display()))?;
            if path.file_stem().and_then(|s| s.to_str()) != Some(&spec.id) {
                return Err(format!(
                    "{}: имя файла должно совпадать с id «{}»",
                    path.display(),
                    spec.id
                ));
            }
            entries.insert(spec.id.clone(), spec);
        }
        Ok(Self { entries })
    }

    pub fn get(&self, id: &str) -> Option<&SensorSpec> {
        self.entries.get(id)
    }

    pub fn iter(&self) -> impl Iterator<Item = &SensorSpec> {
        self.entries.values()
    }
}
