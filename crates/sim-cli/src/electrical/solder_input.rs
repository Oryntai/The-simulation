//! Строгий ввод профиля пайки и каталога, формат 1.

use std::{
    collections::BTreeMap,
    fs,
    path::{Path, PathBuf},
};

use serde::{Deserialize, Deserializer, de::Error};
use sim_process::{
    Analysis, SourcedRange,
    schematic::{JointKind, Schematic},
    work::heat::solder_network::{
        SolderContact, SolderNetworkInput, SolderNetworkSpec, SolderNodes,
    },
};
use sim_world::Provenance;

use super::input::parse;

// Независимая версия дополнительных файлов для схемы формата 1.
const FORMAT: u32 = 1;

/// Исходные байты для отпечатка установки.
#[derive(Debug)]
pub struct SolderFile<T> {
    pub path: PathBuf,
    pub bytes: Vec<u8>,
    pub data: T,
}

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct SolderCatalog {
    format: u32,
    #[serde(default)]
    pub alloys: BTreeMap<String, SolderAlloy>,
    #[serde(default)]
    pub tools: BTreeMap<String, SolderTool>,
}

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct SolderAlloy {
    #[serde(default, deserialize_with = "positive")]
    pub solidus_k: Option<SolderValue>,
    #[serde(default, deserialize_with = "positive")]
    pub liquidus_k: Option<SolderValue>,
}

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct SolderTool {
    #[serde(default, deserialize_with = "positive")]
    pub tip_capacity_j_k: Option<SolderValue>,
    pub max_thermal_power_w: Option<SolderValue>,
}

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct SolderProfile {
    format: u32,
    pub connection: String,
    pub revision: Option<String>,
    pub alloy: Option<String>,
    pub tool: Option<String>,
    #[serde(default)]
    pub geometry: SolderGeometry,
    #[serde(default)]
    pub mode: SolderMode,
    #[serde(default)]
    pub limits: SolderLimits,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct SolderGeometry {
    #[serde(default, deserialize_with = "positive")]
    pub pad_capacity_j_k: Option<SolderValue>,
    #[serde(default, deserialize_with = "positive")]
    pub lead_capacity_j_k: Option<SolderValue>,
    #[serde(default, deserialize_with = "positive")]
    pub copper_capacity_j_k: Option<SolderValue>,
    pub pad_lead_w_k: Option<SolderValue>,
    pub pad_copper_w_k: Option<SolderValue>,
    pub copper_polygon_w_k: Option<SolderValue>,
    #[serde(default, deserialize_with = "positive")]
    pub polygon_k: Option<SolderValue>,
    #[serde(default, deserialize_with = "positive")]
    pub pitch_m: Option<SolderValue>,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct SolderMode {
    #[serde(default, deserialize_with = "positive")]
    pub tip_initial_k: Option<SolderValue>,
    #[serde(default, deserialize_with = "positive")]
    pub pad_initial_k: Option<SolderValue>,
    #[serde(default, deserialize_with = "positive")]
    pub lead_initial_k: Option<SolderValue>,
    #[serde(default, deserialize_with = "positive")]
    pub copper_initial_k: Option<SolderValue>,
    pub requested_thermal_power_w: Option<SolderValue>,
    pub contact: Option<SolderContactData>,
    pub force_n: Option<SolderValue>,
    #[serde(default, deserialize_with = "positive")]
    pub duration_s: Option<SolderValue>,
    pub solder_volume_m3: Option<SolderValue>,
}

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct SolderContactData {
    pub state: ContactState,
    source: Option<Source>,
    #[serde(default, deserialize_with = "positive")]
    pub conductance_w_k: Option<SolderValue>,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ContactState {
    Separated,
    Touching,
}

/// Пределы действуют в геометрии, на инструменте и в ревизии данного профиля.
#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct SolderLimits {
    #[serde(default, deserialize_with = "positive")]
    pub wetting_k: Option<SolderValue>,
    pub max_time_to_wet_s: Option<SolderValue>,
    pub min_above_liquidus_s: Option<SolderValue>,
    pub max_above_liquidus_s: Option<SolderValue>,
    #[serde(default, deserialize_with = "positive")]
    pub max_pad_k: Option<SolderValue>,
    #[serde(default, deserialize_with = "positive")]
    pub max_component_k: Option<SolderValue>,
    pub min_volume_m3: Option<SolderValue>,
    pub bridge_volume_m3: Option<SolderValue>,
}

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(try_from = "SolderValueData")]
pub struct SolderValue {
    min: f64,
    nominal: f64,
    max: f64,
    source: Option<Source>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct SolderValueData {
    min: f64,
    nominal: f64,
    max: f64,
    source: Option<Source>,
}

impl TryFrom<SolderValueData> for SolderValue {
    type Error = &'static str;

    fn try_from(data: SolderValueData) -> Result<Self, Self::Error> {
        if ![data.min, data.nominal, data.max]
            .into_iter()
            .all(f64::is_finite)
            || data.min < 0.0
            || data.min > data.nominal
            || data.nominal > data.max
        {
            return Err("нужен конечный диапазон 0 ≤ min ≤ nominal ≤ max");
        }
        if data.source.as_ref().is_some_and(|source| !source.valid()) {
            return Err("нужен непустой источник без управляющих символов");
        }
        Ok(Self {
            min: data.min,
            nominal: data.nominal,
            max: data.max,
            source: data.source,
        })
    }
}

// Дисковая форма источника совпадает с источниками каталога электрических компонентов.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
enum Source {
    Datasheet(String),
    Measured(String),
    Paper(String),
    Derived(String),
    Estimate(String),
}

impl Source {
    fn provenance(&self) -> Provenance {
        match self {
            Self::Datasheet(value) => Provenance::Datasheet(value.clone()),
            Self::Measured(value) => Provenance::Measured(value.clone()),
            Self::Paper(value) => Provenance::Paper(value.clone()),
            Self::Derived(value) => Provenance::Derived(value.clone()),
            Self::Estimate(value) => Provenance::Estimate(value.clone()),
        }
    }

    fn description(&self) -> &str {
        match self {
            Self::Datasheet(value)
            | Self::Measured(value)
            | Self::Paper(value)
            | Self::Derived(value)
            | Self::Estimate(value) => value,
        }
    }

    fn confirmed(&self) -> bool {
        matches!(
            self,
            Self::Datasheet(_) | Self::Measured(_) | Self::Paper(_)
        ) && self.valid()
    }

    fn valid(&self) -> bool {
        let description = self.description();
        !description.trim().is_empty() && !description.chars().any(char::is_control)
    }
}

impl SolderValue {
    pub fn bounds(&self) -> [f64; 3] {
        [self.min, self.nominal, self.max]
    }

    /// Исходный диапазон сохраняется без выбора номинала и подмены источника.
    pub fn sourced(&self) -> Option<SourcedRange> {
        let range = SourcedRange {
            min: self.min,
            nominal: self.nominal,
            max: self.max,
            provenance: self.source.as_ref()?.provenance(),
        };
        range.validate().ok().map(|()| range)
    }
}

/// Подтверждение происхождения диапазона. Применимость и огибающую проверяет расчёт окна.
pub fn confirmed_parameter(value: Option<&SolderValue>, name: &str) -> Analysis<SourcedRange> {
    let reason = match value {
        None => "нет значения",
        Some(value) => {
            if let Some(range) = value.sourced().filter(SourcedRange::can_prove) {
                return Analysis::Analyzed {
                    value: range,
                    scope: format!("{name}: диапазон с подтверждённым типом источника"),
                };
            }
            "нет подтверждённого источника"
        }
    };
    Analysis::NotAnalyzed {
        reason: format!("{name}: {reason}"),
    }
}

#[derive(Debug)]
pub struct ResolvedSolderProfile<'a> {
    pub profile: &'a SolderProfile,
    pub alloy: Option<&'a SolderAlloy>,
    pub tool: Option<&'a SolderTool>,
}

impl SolderCatalog {
    pub fn parse(path: &Path, bytes: &[u8]) -> Result<Self, String> {
        let data: Self = parse(path, bytes)?;
        validate_version(path, data.format)?;
        for id in data.alloys.keys().chain(data.tools.keys()) {
            validate_identifier(path, "ID позиции каталога", id)?;
        }
        for (id, alloy) in &data.alloys {
            if let (Some(solidus), Some(liquidus)) = (&alloy.solidus_k, &alloy.liquidus_k)
                && solidus.nominal > liquidus.nominal
            {
                return Err(located(
                    path,
                    format!("alloys.{id}: солидус выше ликвидуса"),
                ));
            }
        }
        Ok(data)
    }

    pub fn load(path: &Path) -> Result<SolderFile<Self>, String> {
        load(path, Self::parse)
    }
}

impl SolderProfile {
    pub fn parse(path: &Path, bytes: &[u8]) -> Result<Self, String> {
        let data: Self = parse(path, bytes)?;
        validate_version(path, data.format)?;
        for (field, value) in [
            ("connection", Some(data.connection.as_str())),
            ("revision", data.revision.as_deref()),
            ("alloy", data.alloy.as_deref()),
            ("tool", data.tool.as_deref()),
        ] {
            if let Some(value) = value {
                validate_identifier(path, field, value)?;
            }
        }
        if let Some(contact) = &data.mode.contact {
            if contact.state == ContactState::Separated && contact.conductance_w_k.is_some() {
                return Err(located(
                    path,
                    "mode.contact: проводимость задана без контакта",
                ));
            }
            if contact
                .source
                .as_ref()
                .is_some_and(|source| !source.valid())
            {
                return Err(located(
                    path,
                    "mode.contact.source: нужен непустой источник",
                ));
            }
        }
        Ok(data)
    }

    pub fn load(path: &Path) -> Result<SolderFile<Self>, String> {
        load(path, Self::parse)
    }

    /// Отсутствующая ссылка оставляет вход открытым. Явная неизвестная ссылка является ошибкой.
    pub fn resolve<'a>(
        &'a self,
        schematic: &Schematic,
        catalog: &'a SolderCatalog,
    ) -> Result<ResolvedSolderProfile<'a>, String> {
        let connection = schematic.connections.get(&self.connection).ok_or_else(|| {
            format!(
                "connection: нет стыка «{}» в схеме «{}»",
                self.connection, schematic.name
            )
        })?;
        if connection.kind != JointKind::Solder {
            return Err(format!(
                "connection: стык «{}» имеет тип, отличный от пайки",
                self.connection
            ));
        }
        Ok(ResolvedSolderProfile {
            profile: self,
            alloy: lookup(self.alloy.as_deref(), &catalog.alloys, "alloy")?,
            tool: lookup(self.tool.as_deref(), &catalog.tools, "tool")?,
        })
    }
}

impl ResolvedSolderProfile<'_> {
    /// Готовит входы постоянного режима. Тепловая сеть проверяет точечность параметров.
    pub fn thermal_inputs(&self) -> (SolderNetworkSpec, SolderNetworkInput) {
        let SolderProfile { geometry, mode, .. } = self.profile;
        let sourced = |value: &Option<SolderValue>| value.as_ref().and_then(SolderValue::sourced);
        let spec = SolderNetworkSpec {
            capacity_j_k: SolderNodes {
                tip: self.tool.and_then(|tool| sourced(&tool.tip_capacity_j_k)),
                pad: sourced(&geometry.pad_capacity_j_k),
                lead: sourced(&geometry.lead_capacity_j_k),
                copper: sourced(&geometry.copper_capacity_j_k),
            },
            initial_k: SolderNodes {
                tip: sourced(&mode.tip_initial_k),
                pad: sourced(&mode.pad_initial_k),
                lead: sourced(&mode.lead_initial_k),
                copper: sourced(&mode.copper_initial_k),
            },
            pad_lead_w_k: sourced(&geometry.pad_lead_w_k),
            pad_copper_w_k: sourced(&geometry.pad_copper_w_k),
            copper_polygon_w_k: sourced(&geometry.copper_polygon_w_k),
            polygon_k: sourced(&geometry.polygon_k),
            max_power_w: self
                .tool
                .and_then(|tool| sourced(&tool.max_thermal_power_w)),
        };
        let contact = mode.contact.as_ref().and_then(|contact| {
            contact
                .source
                .as_ref()
                .filter(|source| source.confirmed())?;
            Some(match contact.state {
                ContactState::Separated => SolderContact::Separated,
                ContactState::Touching => SolderContact::Touching {
                    conductance_w_k: sourced(&contact.conductance_w_k),
                },
            })
        });
        let input = SolderNetworkInput {
            contact,
            requested_power_w: sourced(&mode.requested_thermal_power_w),
        };
        (spec, input)
    }
}

fn positive<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Option<SolderValue>, D::Error> {
    let value = Option::<SolderValue>::deserialize(deserializer)?;
    if value.as_ref().is_some_and(|value| value.min <= 0.0) {
        return Err(D::Error::custom(
            "диапазон должен быть строго положительным",
        ));
    }
    Ok(value)
}

fn validate_version(path: &Path, format: u32) -> Result<(), String> {
    if format == FORMAT {
        Ok(())
    } else {
        Err(located(
            path,
            format!("format: версия {format}, ожидается {FORMAT}"),
        ))
    }
}

fn identifier(value: &str) -> bool {
    !value.is_empty() && value.trim() == value && !value.chars().any(char::is_control)
}

fn validate_identifier(path: &Path, field: &str, value: &str) -> Result<(), String> {
    if identifier(value) {
        Ok(())
    } else {
        Err(located(path, format!("{field}: недопустимое имя")))
    }
}

fn located(path: &Path, error: impl std::fmt::Display) -> String {
    format!("{}: {error}", path.display())
}

fn load<T>(
    path: &Path,
    parse: impl FnOnce(&Path, &[u8]) -> Result<T, String>,
) -> Result<SolderFile<T>, String> {
    let bytes = fs::read(path).map_err(|error| located(path, error))?;
    let data = parse(path, &bytes)?;
    Ok(SolderFile {
        path: path.into(),
        bytes,
        data,
    })
}

fn lookup<'a, T>(
    id: Option<&str>,
    entries: &'a BTreeMap<String, T>,
    field: &str,
) -> Result<Option<&'a T>, String> {
    id.map(|id| {
        entries
            .get(id)
            .ok_or_else(|| format!("{field}: неизвестная позиция «{id}»"))
    })
    .transpose()
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_process::{
        Interval, StepModel,
        schematic::{Connection, Endpoint, Parameter, Scope},
        work::heat::solder_network::SolderNetwork,
    };
    use sim_signal::{Rng, Time};

    const ALLOYS: &[u8] = include_bytes!("../../../../catalog/electrical/solder/alloys.toml");
    const PROFILE: &str = r#"
format = 1
connection = "стык"
alloy = "indalloy-106"
tool = "жало"
"#;

    fn catalog() -> SolderCatalog {
        SolderCatalog::parse(Path::new("сплавы.toml"), ALLOYS).unwrap()
    }

    fn schematic(kind: JointKind) -> Schematic {
        let missing = Parameter {
            range: None,
            measurement: "замер стыка".into(),
        };
        Schematic {
            name: "блок".into(),
            scope: Scope::Reference,
            nets: Default::default(),
            components: Default::default(),
            connections: BTreeMap::from([(
                "стык".into(),
                Connection {
                    kind,
                    ends: ["площадка", "вывод"].map(|component| Endpoint {
                        component: component.into(),
                        pin: "p".into(),
                    }),
                    resistance_ohm: missing.clone(),
                    current_limit_a: missing,
                    after: Default::default(),
                },
            )]),
            rails: Default::default(),
            bindings: vec![],
        }
    }

    fn parse_profile(text: &str) -> Result<SolderProfile, String> {
        SolderProfile::parse(Path::new("блок.toml"), text.as_bytes())
    }

    #[test]
    fn каталог_сверен_с_таблицей_температур_kester() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../catalog/electrical/solder/alloys.toml");
        let loaded = SolderCatalog::load(&path).unwrap();
        assert_eq!(loaded.path, path);
        assert_eq!(loaded.bytes, ALLOYS);
        // Строки Sn63Pb37 и Sn96.5Ag3Cu0.5 таблицы Kester, в °C.
        for (id, expected_c) in [
            ("indalloy-106", [183.0, 183.0]),
            ("indalloy-256", [217.0, 220.0]),
        ] {
            let alloy = &loaded.data.alloys[id];
            for (parameter, c) in [&alloy.solidus_k, &alloy.liquidus_k]
                .into_iter()
                .zip(expected_c)
            {
                let range = parameter.as_ref().unwrap().sourced().unwrap();
                assert!(matches!(range.provenance, Provenance::Datasheet(_)));
                assert_eq!([range.min, range.nominal, range.max], [c + 273.15; 3]);
            }
        }
    }

    #[test]
    fn строгий_профиль_отклоняет_ошибочные_поля_и_значения() {
        // 1 — версия формата Т5.2. Остальные числа здесь — контрпримеры разбора.
        let base = "format = 1\nconnection = 'стык'\n";
        for (text, reason) in [
            (base.replace("format = 1", "format = 2"), "format"),
            (format!("{base}connection = 'другой'"), "connection"),
            (format!("{base}alloy_id = 'лишнее'"), "alloy_id"),
            (format!("{base}[geometry]\npitch_mm = 1"), "pitch_mm"),
            (
                format!("{base}[mode]\ncontact = {{state = 'pressing'}}"),
                "pressing",
            ),
            (
                format!(
                    "{base}[mode]\ncontact = {{state = 'separated', conductance_w_k = {{min=1,nominal=1,max=1}}}}"
                ),
                "проводимость",
            ),
            (
                format!("{base}[limits]\nmax_pad_k = {{min=1,nominal=1,max=nan}}"),
                "limits.max_pad_k.max",
            ),
            (
                format!("{base}[mode]\nforce_n = {{min=1,nominal=inf,max=1}}"),
                "mode.force_n.nominal",
            ),
            (
                format!("{base}[mode]\nsolder_volume_m3 = {{min=-inf,nominal=1,max=1}}"),
                "mode.solder_volume_m3.min",
            ),
            (
                format!("{base}[mode]\nforce_n = {{min=-1,nominal=1,max=1}}"),
                "диапазон",
            ),
            (
                format!("{base}[mode]\nforce_n = {{min=1,nominal=3,max=2}}"),
                "диапазон",
            ),
            (
                format!("{base}[mode]\ntip_initial_k = {{min=0,nominal=1,max=2}}"),
                "положительным",
            ),
            (
                format!("{base}[limits]\nmin_volume_m3 = {{min=1,nominal=1,max=1,units='m³'}}"),
                "units",
            ),
            (
                format!(
                    "{base}[limits]\nmin_volume_m3 = {{min=1,nominal=1,max=1,source.measured=' '}}"
                ),
                "источник",
            ),
            (
                format!(
                    "{base}[limits]\nmin_volume_m3 = {{min=1,nominal=1,max=1,source={{measured='замер',estimate='оценка'}}}}"
                ),
                "source",
            ),
        ] {
            let error = parse_profile(&text).unwrap_err();
            assert!(
                error.contains("блок.toml") && error.contains(reason),
                "{error}"
            );
        }
    }

    #[test]
    fn строгий_каталог_проверяет_сплавы_и_инструменты() {
        // Синтетические некорректные значения для проверки структуры и порядка.
        for (text, reason) in [
            ("[tools.tip]\nunknown = 1", "unknown"),
            (
                "[tools.tip]\nmax_thermal_power_w = {min=1,nominal=1,max=+inf}",
                "tools.tip.max_thermal_power_w.max",
            ),
            (
                "[alloys.a]\nsolidus_k={min=3,nominal=3,max=3}\nliquidus_k={min=2,nominal=2,max=2}",
                "солидус выше ликвидуса",
            ),
        ] {
            let bytes = format!("format = 1\n{text}");
            let error =
                SolderCatalog::parse(Path::new("каталог.toml"), bytes.as_bytes()).unwrap_err();
            assert!(
                error.contains("каталог.toml") && error.contains(reason),
                "{error}"
            );
        }
    }

    #[test]
    fn профиль_разрешает_только_паяный_стык_и_существующие_ссылки() {
        let catalog = catalog();
        let joints = schematic(JointKind::Solder);
        let profile = parse_profile("format = 1\nconnection = 'стык'").unwrap();
        let resolved = profile.resolve(&joints, &catalog).unwrap();
        assert!(resolved.tool.is_none() && resolved.alloy.is_none());
        assert!(
            !joints.connections["стык"]
                .resistance_ohm
                .measurement
                .is_empty()
        );
        assert!(
            profile
                .resolve(&schematic(JointKind::Terminal), &catalog)
                .is_err()
        );
        for (text, field) in [
            ("connection = 'нет'", "connection"),
            ("connection = 'стык'\nalloy = 'нет'", "alloy"),
            ("connection = 'стык'\ntool = 'нет'", "tool"),
        ] {
            let profile = parse_profile(&format!("format = 1\n{text}")).unwrap();
            assert!(
                profile
                    .resolve(&joints, &catalog)
                    .unwrap_err()
                    .starts_with(field)
            );
        }
    }

    #[test]
    fn пропуск_источника_контакта_и_огибающей_оставляет_анализ_открытым() {
        let text = r#"
format = 1
connection = "стык"
[limits]
# Синтетические данные для проверки происхождения, не пределы реального процесса.
min_above_liquidus_s = { min = 1, nominal = 1, max = 1 }
max_above_liquidus_s = { min = 2, nominal = 2, max = 2, source.estimate = "оценка" }
"#;
        let profile = parse_profile(text).unwrap();
        for (parameter, reason) in [
            (profile.limits.max_pad_k.as_ref(), "нет значения"),
            (
                profile.limits.min_above_liquidus_s.as_ref(),
                "нет подтверждённого источника",
            ),
            (
                profile.limits.max_above_liquidus_s.as_ref(),
                "нет подтверждённого источника",
            ),
        ] {
            assert_eq!(
                confirmed_parameter(parameter, "предел"),
                Analysis::NotAnalyzed {
                    reason: format!("предел: {reason}")
                }
            );
        }

        let catalog = bench_catalog();
        let schematic = schematic(JointKind::Solder);
        let valid = format!("{PROFILE}{}", bench_mode(1.0));
        for (text, expected) in [
            (
                valid.replace("source.paper = \"стенд\"\nconductance", "conductance"),
                "нет наблюдения контакта",
            ),
            (
                valid.replace(
                    "min = 1, nominal = 1, max = 1, source.paper",
                    "min = 1, nominal = 1, max = 2, source.paper",
                ),
                "огибающая для диапазона не рассчитана",
            ),
        ] {
            let profile = parse_profile(&text).unwrap();
            let (spec, input) = profile
                .resolve(&schematic, &catalog)
                .unwrap()
                .thermal_inputs();
            let result = SolderNetwork::new(spec)
                .unwrap()
                .advance(
                    Interval {
                        start: Time(0),
                        end: Time::from_secs(1.0).unwrap(),
                    },
                    &input,
                    &mut Rng::stream(0, "блок"),
                )
                .unwrap();
            assert!(
                matches!(result, Analysis::NotAnalyzed { reason } if reason.contains(expected))
            );
        }
    }

    fn bench_catalog() -> SolderCatalog {
        let mut bytes = ALLOYS.to_vec();
        bytes.extend_from_slice(
            r#"
[tools."жало"]
tip_capacity_j_k = { min = 2, nominal = 2, max = 2, source.paper = "C_tip, аналитический стенд" }
max_thermal_power_w = { min = 4, nominal = 4, max = 4, source.paper = "P_limit, аналитический стенд" }
"#
            .as_bytes(),
        );
        SolderCatalog::parse(Path::new("стенд.toml"), &bytes).unwrap()
    }

    // Аналитический стенд: Cж = Cп = 2 Дж/К, G = 1 Вт/К, P = 0.
    // Tп(t) = (500 + 440)/2 − (500 − 440)/2 · exp(−2Gt/C).
    // Запас от ликвидуса 456,15 К на 0,5 и 1 с: −4,34591979138 и +2,81361676486 К.
    #[test]
    fn данные_тепловой_сети_сходятся_по_обе_стороны_ликвидуса() {
        let catalog = bench_catalog();
        let schematic = schematic(JointKind::Solder);
        for (seconds, expected_margin) in [(0.5, -4.34591979138), (1.0, 2.81361676486)] {
            let text = format!("{PROFILE}{}", bench_mode(seconds));
            let profile = parse_profile(&text).unwrap();
            let resolved = profile.resolve(&schematic, &catalog).unwrap();
            let Analysis::Analyzed {
                value: liquidus, ..
            } = confirmed_parameter(resolved.alloy.unwrap().liquidus_k.as_ref(), "ликвидус")
            else {
                panic!("в каталоге нужен ликвидус");
            };
            let actual_seconds = resolved.profile.mode.duration_s.as_ref().unwrap().bounds()[1];
            let (spec, input) = resolved.thermal_inputs();
            let mut network = SolderNetwork::new(spec).unwrap();
            let Analysis::Analyzed { value, .. } = network
                .advance(
                    Interval {
                        start: Time(0),
                        end: Time::from_secs(actual_seconds).unwrap(),
                    },
                    &input,
                    &mut Rng::stream(0, "блок"),
                )
                .unwrap()
            else {
                panic!("все параметры аналитического стенда заданы");
            };
            // Точность сравнения следует из 11 знаков после запятой ручного расчёта.
            assert!((value.end_k.pad - liquidus.nominal - expected_margin).abs() < 1e-11);
            assert_eq!(actual_seconds, seconds);
            assert_eq!(value.end_k.lead, 440.0);
            assert_eq!(value.end_k.copper, 440.0);
            assert_eq!(value.supplied_j, 0.0);
            assert_eq!(value.to_polygon_j, 0.0);
        }
    }

    fn bench_mode(seconds: f64) -> String {
        // Теплоизолированные медь и вывод не меняют температуру. Проводимости,
        // температуры и время взяты из постановки аналитической задачи выше.
        format!(
            r#"
[geometry]
pad_capacity_j_k = {{ min = 2, nominal = 2, max = 2, source.paper = "стенд" }}
lead_capacity_j_k = {{ min = 2, nominal = 2, max = 2, source.paper = "стенд" }}
copper_capacity_j_k = {{ min = 2, nominal = 2, max = 2, source.paper = "стенд" }}
pad_lead_w_k = {{ min = 0, nominal = 0, max = 0, source.paper = "стенд" }}
pad_copper_w_k = {{ min = 0, nominal = 0, max = 0, source.paper = "стенд" }}
copper_polygon_w_k = {{ min = 0, nominal = 0, max = 0, source.paper = "стенд" }}
polygon_k = {{ min = 440, nominal = 440, max = 440, source.paper = "стенд" }}

[mode]
tip_initial_k = {{ min = 500, nominal = 500, max = 500, source.paper = "стенд" }}
pad_initial_k = {{ min = 440, nominal = 440, max = 440, source.paper = "стенд" }}
lead_initial_k = {{ min = 440, nominal = 440, max = 440, source.paper = "стенд" }}
copper_initial_k = {{ min = 440, nominal = 440, max = 440, source.paper = "стенд" }}
requested_thermal_power_w = {{ min = 0, nominal = 0, max = 0, source.paper = "стенд" }}
duration_s = {{ min = {seconds}, nominal = {seconds}, max = {seconds}, source.paper = "стенд" }}

[mode.contact]
state = "touching"
source.paper = "стенд"
conductance_w_k = {{ min = 1, nominal = 1, max = 1, source.paper = "стенд" }}
"#
        )
    }
}
