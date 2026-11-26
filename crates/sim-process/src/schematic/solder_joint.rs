//! Окно пайки и перенос измеренных параметров на адресованный стык схемы.

use std::num::NonZeroU32;

use sim_world::Provenance;

use crate::{
    Analysis, ProcessError, SourcedRange, source_can_prove,
    work::{
        heat::solder_network::{
            SolderContact, SolderNetwork, SolderNetworkInput, SolderNetworkSpec,
        },
        solder_window::{
            SolderBounds, SolderWindowInput, SolderWindowReport, SolderWindowStatus,
            assess_solder_window,
        },
    },
};

use super::{CompiledSchematic, Endpoint, JointKind, Scope, identifier, invalid, open};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SolderJointAddress {
    pub schematic: String,
    pub scope: Scope,
    pub connection: String,
    /// Порядок выводов схемы сохраняет ориентацию для протокола испытаний.
    pub ends: [Endpoint; 2],
}

#[derive(Clone, Debug, PartialEq)]
pub struct SolderJointProfile {
    pub connection: String,
    /// Ревизия установки и процесса, включая геометрию стыка, материалы и подготовку.
    pub revision: Option<String>,
    pub alloy: Option<String>,
    pub tool: Option<String>,
    pub input: SolderWindowInput,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SolderSeriesApplicability {
    pub alloy: String,
    pub tool: String,
    /// Фиксированные параметры тепловой установки, включая источники их калибровки.
    pub network: SolderNetworkSpec,
    pub thermal: SolderNetworkInput,
    pub force_n: Option<SolderBounds>,
    pub duration_s: Option<SolderBounds>,
    pub pitch_m: Option<SolderBounds>,
    pub dose_m3: Option<SolderBounds>,
    /// Протокол измерений, подтверждающий совместное покрытие всей области.
    /// Нужна огибающая сочетаний параметров, включая применимость проводимости
    /// контакта во всём диапазоне усилия. Одних минимумов и максимумов проб мало.
    pub source: Option<Provenance>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SolderSeriesMeasurement {
    pub range: SourcedRange,
    /// Условия и протокол замера, включая погрешность. Для прочности
    /// фиксируются схема нагружения, направление, скорость и оснастка.
    pub conditions: Option<String>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SolderControlSeries {
    pub id: String,
    pub address: SolderJointAddress,
    pub revision: String,
    pub applicability: Option<SolderSeriesApplicability>,
    pub four_wire_resistance_ohm: Option<SolderSeriesMeasurement>,
    pub destructive_strength_n: Option<SolderSeriesMeasurement>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SolderJointReport {
    pub address: SolderJointAddress,
    pub revision: Option<String>,
    pub window: Analysis<SolderWindowReport>,
    pub series_id: Option<String>,
    pub applicability: Analysis<SolderSeriesApplicability>,
    /// Параметры стыка для указанных условий измерения. Токовый предел,
    /// номинальный эквивалент цепи и готовность сборки определяются отдельно.
    pub resistance_ohm: Analysis<SolderSeriesMeasurement>,
    pub strength_n: Analysis<SolderSeriesMeasurement>,
}

impl SolderJointReport {
    /// Pass означает выполнение окна процесса для расчётной постановки.
    pub fn window_status(&self) -> SolderWindowStatus {
        match &self.window {
            Analysis::Analyzed { value, .. } => value.status(),
            Analysis::NotAnalyzed { reason } => SolderWindowStatus::NotAnalyzed {
                reason: reason.clone(),
            },
        }
    }
}

impl CompiledSchematic {
    pub fn solder_joint_ids(&self) -> impl Iterator<Item = &str> {
        self.schematic
            .connections
            .iter()
            .filter(|(_, joint)| joint.kind == JointKind::Solder)
            .map(|(id, _)| id.as_str())
    }

    /// Оценивает профиль того же стыка и применимость контрольной серии.
    /// Без профиля окно остаётся открытым. Замеры переносятся только внутри
    /// подтверждённой области серии и при выполненном окне.
    pub fn assess_solder_joint(
        &self,
        connection: &str,
        profile: Option<&SolderJointProfile>,
        series: Option<&SolderControlSeries>,
        steps: NonZeroU32,
    ) -> Result<SolderJointReport, ProcessError> {
        let joint = self
            .schematic
            .connections
            .get(connection)
            .ok_or_else(|| invalid(format!("нет стыка «{connection}» в схеме")))?;
        if joint.kind != JointKind::Solder {
            return Err(invalid(format!("«{connection}»: стык другого типа")));
        }
        let address = SolderJointAddress {
            schematic: self.schematic.name.clone(),
            scope: self.schematic.scope,
            connection: connection.into(),
            ends: joint.ends.clone(),
        };

        if let Some(series) = series {
            series.validate()?;
        }
        let window = if let Some(profile) = profile {
            if profile.connection != connection {
                return Err(invalid("профиль пайки адресован другому стыку"));
            }
            if [&profile.revision, &profile.alloy, &profile.tool]
                .into_iter()
                .flatten()
                .any(|name| !identifier(name))
            {
                return Err(invalid(
                    "некорректная ревизия, сплав или инструмент профиля пайки",
                ));
            }
            let result = assess_solder_window(&profile.input, steps)?;
            if self.mismatches.contains(connection) {
                open("несовместимые контакты паяного стыка")
            } else {
                Analysis::Analyzed {
                    value: result,
                    scope: "расчётное окно процесса адресованного стыка".into(),
                }
            }
        } else {
            open("нет профиля пайки для стыка")
        };

        let applicability = match (series, profile, &window) {
            (None, _, _) => open("нет контрольной серии пайки"),
            (Some(series), Some(profile), Analysis::Analyzed { value, .. })
                if value.status() == SolderWindowStatus::Pass =>
            {
                match series.applicable(&address, profile) {
                    Ok(area) => Analysis::Analyzed {
                        value: area.clone(),
                        scope: format!("серия «{}», ревизия «{}»", series.id, series.revision),
                    },
                    Err(reason) => open(reason),
                }
            }
            _ => open("выполнение окна процесса стыка не подтверждено"),
        };
        let measurement = |value, method| transfer_measurement(&applicability, value, method);
        let resistance_ohm = measurement(
            series.and_then(|s| s.four_wire_resistance_ohm.as_ref()),
            "четырёхпроводный замер сопротивления",
        );
        let strength_n = measurement(
            series.and_then(|s| s.destructive_strength_n.as_ref()),
            "разрушающее испытание прочности",
        );
        Ok(SolderJointReport {
            address,
            revision: profile.and_then(|p| p.revision.clone()),
            window,
            series_id: series.map(|s| s.id.clone()),
            applicability,
            resistance_ohm,
            strength_n,
        })
    }
}

impl SolderControlSeries {
    fn validate(&self) -> Result<(), ProcessError> {
        let address = &self.address;
        if [
            &self.id,
            &self.revision,
            &address.schematic,
            &address.connection,
        ]
        .into_iter()
        .chain(
            address
                .ends
                .iter()
                .flat_map(|end| [&end.component, &end.pin]),
        )
        .any(|name| !identifier(name))
            || address.ends[0] == address.ends[1]
        {
            return Err(invalid(
                "некорректный адрес, идентификатор или ревизия контрольной серии",
            ));
        }
        for result in [&self.four_wire_resistance_ohm, &self.destructive_strength_n]
            .into_iter()
            .flatten()
        {
            let range = &result.range;
            if ![range.min, range.nominal, range.max]
                .into_iter()
                .all(f64::is_finite)
                || range.min < 0.0
                || range.min > range.nominal
                || range.nominal > range.max
            {
                return Err(invalid("некорректный диапазон замеров контрольной серии"));
            }
        }
        if let Some(area) = &self.applicability {
            area.validate()?;
        }
        Ok(())
    }

    fn applicable(
        &self,
        address: &SolderJointAddress,
        profile: &SolderJointProfile,
    ) -> Result<&SolderSeriesApplicability, String> {
        if &self.address != address {
            return Err("контрольная серия относится к другому стыку или области схемы".into());
        }
        if profile.revision.as_deref() != Some(self.revision.as_str()) {
            return Err("ревизия контрольной серии не совпадает с ревизией профиля".into());
        }
        let area = self
            .applicability
            .as_ref()
            .ok_or("нет области применимости контрольной серии")?;
        if !measured(area.source.as_ref()) {
            return Err("нет измеренного источника совместной огибающей контрольной серии".into());
        }
        let input = &profile.input;
        if profile.alloy.as_deref() != Some(area.alloy.as_str())
            || profile.tool.as_deref() != Some(area.tool.as_str())
            || input.network != area.network
            || input.thermal != area.thermal
        {
            return Err(
                "сплав, инструмент или тепловые условия отличаются от контрольной серии".into(),
            );
        }
        let actual = [
            &input.force_n,
            &input.duration_s,
            &input.pitch_m,
            &input.dose_m3,
        ];
        for ((domain, name, _), actual) in area.axes().into_iter().zip(actual) {
            let domain = domain.ok_or_else(|| format!("нет огибающей серии: {name}"))?;
            let actual = actual
                .as_ref()
                .filter(|range| range.can_prove())
                .ok_or_else(|| format!("нет подтверждённой огибающей режима: {name}"))?;
            if actual.min < domain.lower || actual.max > domain.upper {
                return Err(format!("режим вне области контрольной серии: {name}"));
            }
        }
        Ok(area)
    }
}

impl SolderSeriesApplicability {
    fn axes(&self) -> [(Option<SolderBounds>, &'static str, bool); 4] {
        [
            (self.force_n, "усилие", true),
            (self.duration_s, "длительность", true),
            (self.pitch_m, "шаг площадок", true),
            (self.dose_m3, "доза припоя", false),
        ]
    }

    fn validate(&self) -> Result<(), ProcessError> {
        if !identifier(&self.alloy) || !identifier(&self.tool) {
            return Err(invalid(
                "контрольной серии нужны идентификаторы сплава и инструмента",
            ));
        }
        for (domain, name, positive) in self.axes() {
            if let Some(bounds) = domain
                && (!bounds.lower.is_finite()
                    || !bounds.upper.is_finite()
                    || bounds.lower < 0.0
                    || (positive && bounds.lower == 0.0)
                    || bounds.lower > bounds.upper)
            {
                return Err(invalid(format!("некорректная огибающая серии: {name}")));
            }
        }
        SolderNetwork::new(self.network.clone())?;
        for (range, name, positive) in [
            (self.thermal.requested_power_w.as_ref(), "мощность", false),
            (
                match &self.thermal.contact {
                    Some(SolderContact::Touching { conductance_w_k }) => conductance_w_k.as_ref(),
                    _ => None,
                },
                "проводимость контакта",
                true,
            ),
        ] {
            if let Some(range) = range
                && (!range.min.is_finite()
                    || !range.nominal.is_finite()
                    || !range.max.is_finite()
                    || range.min < 0.0
                    || (positive && range.min == 0.0)
                    || range.min > range.nominal
                    || range.nominal > range.max)
            {
                return Err(invalid(format!(
                    "некорректный тепловой параметр серии: {name}"
                )));
            }
        }
        Ok(())
    }
}

fn measured(source: Option<&Provenance>) -> bool {
    source
        .is_some_and(|source| matches!(source, Provenance::Measured(_)) && source_can_prove(source))
}

fn transfer_measurement(
    applicability: &Analysis<SolderSeriesApplicability>,
    measurement: Option<&SolderSeriesMeasurement>,
    method: &str,
) -> Analysis<SolderSeriesMeasurement> {
    let scope = match applicability {
        Analysis::Analyzed { scope, .. } => scope,
        Analysis::NotAnalyzed { reason } => return open(reason.clone()),
    };
    let Some(measurement) = measurement else {
        return open(format!("в серии отсутствует {method}"));
    };
    if !measured(Some(&measurement.range.provenance)) {
        return open(format!(
            "{method}: отсутствует измеренный источник диапазона"
        ));
    }
    let Some(conditions) = measurement
        .conditions
        .as_deref()
        .filter(|text| identifier(text))
    else {
        return open(format!("{method}: не заданы условия испытания"));
    };
    Analysis::Analyzed {
        value: measurement.clone(),
        scope: format!("{scope}; {method}; {conditions}"),
    }
}
