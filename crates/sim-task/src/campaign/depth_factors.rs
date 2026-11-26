//! Конечные факторы покрытия из корневого раздела `[depth_factors]` манифеста версии 1.
//! Уровни разброса задаются в самих факторах. Постановка, путь, сетка и расписание
//! инъекций разрешаются теми же данными, что и в обычной кампании.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use sim_signal::Rng;
use sim_world::CompiledWorld;

use super::{Campaign, Disposition, PlanRow, ResolvedInputs, RunKey, SampledInputs, invalid};
use crate::{
    Error, Fault, Fingerprint, Source,
    sampling::{AppliedValue, Axis, Parameter, Sample, SampleKey, Scope, Seeds, address, name},
};

/// Пустой или частичный выбор допустим для проверки физических ограничений.
pub type Selection = BTreeMap<String, String>;

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct DepthMode {
    pub station: String,
    pub mode: String,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum DepthLevels {
    Mode {
        values: BTreeMap<String, DepthMode>,
    },
    Path {
        values: BTreeMap<String, String>,
    },
    Grid {
        values: BTreeMap<String, String>,
    },
    /// Пустой список задаёт отсутствие дополнительных инъекций на этом факторе.
    Faults {
        values: BTreeMap<String, Vec<Fault>>,
    },
    /// Точки в единице параметра. Конечность списка не утверждает полноту огибающей.
    Axis {
        parameter: Parameter,
        unit: String,
        nominal: f64,
        values: BTreeMap<String, f64>,
    },
}

impl DepthLevels {
    fn ids(&self) -> Box<dyn Iterator<Item = &String> + '_> {
        match self {
            Self::Mode { values } => Box::new(values.keys()),
            Self::Path { values } | Self::Grid { values } => Box::new(values.keys()),
            Self::Faults { values } => Box::new(values.keys()),
            Self::Axis { values, .. } => Box::new(values.keys()),
        }
    }

    /// Идентификаторы уровней в каноническом порядке.
    pub fn level_ids(&self) -> impl Iterator<Item = &str> {
        self.ids().map(String::as_str)
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct DepthFactor {
    /// Тройки строятся только из факторов с этой явной пометкой.
    pub safety: bool,
    pub source: Source,
    pub levels: DepthLevels,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct DepthConstraint {
    /// Сочетание запрещено, если совпали все указанные уровни.
    pub when: Selection,
    pub reason: String,
    pub source: Source,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Description {
    factors: BTreeMap<String, DepthFactor>,
    #[serde(default)]
    constraints: BTreeMap<String, DepthConstraint>,
}

/// Проверенное описание факторов. Для режима, пути и сетки требуется по одному
/// фактору. Независимые оси и группы дополнительных отказов задаются явно.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(try_from = "Description")]
pub struct DepthFactors {
    factors: BTreeMap<String, DepthFactor>,
    constraints: BTreeMap<String, DepthConstraint>,
    #[serde(skip)]
    axes: Vec<Axis>,
}

fn distinct<T: Ord>(values: impl Iterator<Item = T>) -> Result<(), Error> {
    let mut seen = BTreeSet::new();
    for value in values {
        if !seen.insert(value) {
            return Err(invalid("разные уровни ссылаются на одно значение"));
        }
    }
    Ok(())
}

impl TryFrom<Description> for DepthFactors {
    type Error = Error;

    fn try_from(mut description: Description) -> Result<Self, Error> {
        let mut dimensions = BTreeSet::new();
        let mut fault_owners = BTreeMap::new();
        let mut parameters = BTreeSet::new();
        let mut axes = Vec::new();
        for (id, factor) in &mut description.factors {
            let mut validate = || {
                name(id)?;
                factor.source.validate()?;
                if factor.levels.ids().next().is_none() {
                    return Err(invalid("пустой набор уровней"));
                }
                factor.levels.level_ids().try_for_each(name)?;
                let dimension = match &mut factor.levels {
                    DepthLevels::Mode { values } => {
                        for mode in values.values() {
                            name(&mode.station)?;
                            name(&mode.mode)?;
                        }
                        distinct(values.values())?;
                        "mode"
                    }
                    DepthLevels::Path { values } | DepthLevels::Grid { values } => {
                        values.values().try_for_each(|value| name(value))?;
                        distinct(values.values())?;
                        match factor.levels {
                            DepthLevels::Path { .. } => "path",
                            _ => "grid",
                        }
                    }
                    DepthLevels::Faults { values } => {
                        for faults in values.values_mut() {
                            faults.sort();
                            distinct(faults.iter())?;
                            for fault in faults {
                                name(&fault.node)?;
                                if fault_owners
                                    .insert(fault.clone(), id.as_str())
                                    .is_some_and(|owner| owner != id)
                                {
                                    return Err(invalid("отказ принадлежит разным факторам"));
                                }
                            }
                        }
                        distinct(values.values())?;
                        return Ok(());
                    }
                    DepthLevels::Axis {
                        parameter,
                        unit,
                        nominal,
                        values,
                    } => {
                        name(unit)?;
                        if let Parameter::Friction { pair, .. } = parameter {
                            pair.sort();
                        }
                        if !parameters.insert(parameter.clone()) {
                            return Err(invalid("физический параметр принадлежит разным факторам"));
                        }
                        for value in values.values_mut().chain(std::iter::once(&mut *nominal)) {
                            if !value.is_finite() {
                                return Err(invalid("номинал и уровни должны быть конечными"));
                            }
                            // Оба знака нуля обозначают одно физическое значение.
                            if *value == 0.0 {
                                *value = 0.0;
                            }
                        }
                        let mut ordered: Vec<_> = values.values().copied().collect();
                        ordered.sort_by(f64::total_cmp);
                        if ordered.windows(2).any(|pair| pair[0] == pair[1])
                            || !ordered.contains(nominal)
                        {
                            return Err(invalid("повтор уровня или номинал вне набора"));
                        }
                        let axis = Axis {
                            id: id.clone(),
                            parameter: parameter.clone(),
                            unit: unit.clone(),
                            nominal: *nominal,
                            min: ordered[0],
                            max: *ordered.last().expect("уровни проверены"),
                            values: Some(ordered),
                            source: factor.source.clone(),
                            scope: Scope::Run,
                        };
                        parameter.check_domain(&axis)?;
                        axes.push(axis);
                        return Ok(());
                    }
                };
                if !dimensions.insert(dimension) {
                    return Err(invalid(format!("повтор фактора типа {dimension}")));
                }
                Ok(())
            };
            super::field(&format!("depth_factors.factors.{id}"), validate())?;
        }
        for dimension in ["mode", "path", "grid"] {
            if !dimensions.contains(dimension) {
                return Err(invalid(format!(
                    "depth_factors: отсутствует фактор типа {dimension}"
                )));
            }
        }
        let result = Self {
            factors: description.factors,
            constraints: description.constraints,
            axes,
        };
        for (id, rule) in &result.constraints {
            let validate = || {
                name(id)?;
                name(&rule.reason)?;
                rule.source.validate()?;
                if rule.when.is_empty() || !rule.source.can_prove() {
                    return Err(invalid(
                        "физическое исключение требует сочетания и подтверждённого источника",
                    ));
                }
                result.validate_selection(&rule.when)
            };
            super::field(&format!("depth_factors.constraints.{id}"), validate())?;
        }
        Ok(result)
    }
}

impl DepthFactors {
    pub fn factors(&self) -> &BTreeMap<String, DepthFactor> {
        &self.factors
    }

    pub fn constraints(&self) -> &BTreeMap<String, DepthConstraint> {
        &self.constraints
    }

    fn validate_selection(&self, selection: &Selection) -> Result<(), Error> {
        for (id, level) in selection {
            let factor = self
                .factors
                .get(id)
                .ok_or_else(|| invalid(format!("depth_factors: неизвестный фактор {id}")))?;
            if !factor.levels.level_ids().any(|known| known == level) {
                return Err(invalid(format!(
                    "depth_factors.{id}: неизвестный уровень {level}"
                )));
            }
        }
        Ok(())
    }

    /// Возвращает первое по ID физическое исключение, уже доказанное выбранными уровнями.
    /// Для частичного выбора отсутствие совпадения не обещает продолжения.
    pub fn exclusion(
        &self,
        selection: &Selection,
    ) -> Result<Option<(&str, &DepthConstraint)>, Error> {
        self.validate_selection(selection)?;
        Ok(self
            .constraints
            .iter()
            .find(|(_, rule)| {
                rule.when
                    .iter()
                    .all(|(factor, level)| selection.get(factor) == Some(level))
            })
            .map(|(id, rule)| (id.as_str(), rule)))
    }

    /// Идентичность полной строки, в том числе исключённой или открытой.
    /// Уровни осей записываются в существующий ключ граничной выборки в порядке ID.
    pub fn run_key(&self, selection: &Selection) -> Result<RunKey, Error> {
        self.validate_selection(selection)?;
        if selection.len() != self.factors.len() {
            let missing: Vec<_> = self
                .factors
                .keys()
                .filter(|id| !selection.contains_key(*id))
                .collect();
            return Err(invalid(format!(
                "depth_factors: отсутствуют уровни {missing:?}"
            )));
        }
        let mut mode = None;
        let mut path = None;
        let mut grid = None;
        let mut faults = Vec::new();
        let mut axes = Vec::new();
        for (id, factor) in &self.factors {
            let level = &selection[id];
            match &factor.levels {
                DepthLevels::Mode { values } => mode = Some(&values[level]),
                DepthLevels::Path { values } => path = Some(&values[level]),
                DepthLevels::Grid { values } => grid = Some(&values[level]),
                DepthLevels::Faults { values } => faults.extend(values[level].iter().cloned()),
                DepthLevels::Axis { .. } => axes.extend([id.as_str(), level.as_str()]),
            }
        }
        let mode = mode.expect("один фактор режима проверен");
        faults.sort();
        let sample = if axes.is_empty() {
            SampleKey::Nominal
        } else {
            SampleKey::Boundary { id: address(&axes) }
        };
        Ok(RunKey {
            station: mode.station.clone(),
            mode: mode.mode.clone(),
            path: path.expect("один фактор пути проверен").clone(),
            faults,
            sample,
            grid: grid.expect("один фактор сетки проверен").clone(),
        })
    }

    /// Обратное преобразование для повтора. Проверяет принадлежность ключа этому
    /// набору факторов. Отпечаток входов проверяется отдельно.
    pub fn selection(&self, key: &RunKey) -> Result<Selection, Error> {
        let wrong = || invalid(format!("depth_factors: ключ вне описания {key:?}"));
        let mut rest = match &key.sample {
            SampleKey::Boundary { id } => id.as_str(),
            SampleKey::Nominal => "",
            SampleKey::Random { .. } => return Err(wrong()),
        };
        let active: BTreeSet<_> = key.faults.iter().collect();
        let mut result = Selection::new();
        for (id, factor) in &self.factors {
            let level = match &factor.levels {
                DepthLevels::Mode { values } => values
                    .iter()
                    .find(|(_, mode)| mode.station == key.station && mode.mode == key.mode)
                    .map(|(level, _)| level),
                DepthLevels::Path { values } => values
                    .iter()
                    .find(|(_, path)| **path == key.path)
                    .map(|(level, _)| level),
                DepthLevels::Grid { values } => values
                    .iter()
                    .find(|(_, grid)| **grid == key.grid)
                    .map(|(level, _)| level),
                DepthLevels::Faults { values } => {
                    let actual: BTreeSet<_> = values
                        .values()
                        .flatten()
                        .filter(|fault| active.contains(fault))
                        .collect();
                    values
                        .iter()
                        .find(|(_, faults)| faults.iter().eq(actual.iter().copied()))
                        .map(|(level, _)| level)
                }
                DepthLevels::Axis { values, .. } => values.keys().find(|level| {
                    if let Some(suffix) = rest.strip_prefix(&address(&[id, level])) {
                        rest = suffix;
                        true
                    } else {
                        false
                    }
                }),
            }
            .ok_or_else(wrong)?;
            result.insert(id.clone(), level.clone());
        }
        if self.run_key(&result)? != *key {
            return Err(wrong());
        }
        Ok(result)
    }

    pub(super) fn axes(&self) -> &[Axis] {
        &self.axes
    }

    fn axis_values(&self, selection: &Selection) -> impl Iterator<Item = (&Axis, f64)> {
        self.axes.iter().map(|axis| {
            let DepthLevels::Axis { values, .. } = &self.factors[&axis.id].levels else {
                unreachable!("оси построены из факторов разброса");
            };
            (axis, values[&selection[&axis.id]])
        })
    }

    fn sample(&self, root: u64, key: &RunKey, selection: &Selection) -> Sample {
        let stream = |seed, fields: &[&str]| Rng::stream(seed, &address(fields)).next_u64();
        let namespace = "campaign/depth/v1";
        let batch = stream(root, &[namespace, "batch"]);
        let sample_id = match &key.sample {
            SampleKey::Boundary { id } => id.as_str(),
            _ => "",
        };
        let specimen = stream(batch, &[namespace, "specimen", sample_id]);
        let repeat = stream(specimen, &[namespace, "repeat"]);
        let kinds: Vec<_> = key
            .faults
            .iter()
            .map(|fault| format!("{:?}", fault.kind))
            .collect();
        // Сетки одной установки используют одно зерно, как в прежней кампании.
        let mut fields = vec![namespace, "run", &key.station, &key.mode, &key.path];
        for (fault, kind) in key.faults.iter().zip(&kinds) {
            fields.extend([fault.node.as_str(), kind.as_str()]);
        }
        let run = stream(repeat, &fields);
        Sample {
            key: key.sample.clone(),
            seeds: Seeds {
                root,
                batch,
                specimen,
                repeat,
                run,
            },
            alternatives: BTreeMap::new(),
            values: self
                .axis_values(selection)
                .map(|(axis, value)| AppliedValue {
                    id: axis.id.clone(),
                    parameter: axis.parameter.clone(),
                    unit: axis.unit.clone(),
                    value,
                    source: axis.source.clone(),
                    sources: vec![axis.source.clone()],
                })
                .collect(),
        }
    }
}

/// Физическое исключение хранит ID правила и основание. Пробелы установщика
/// сохраняют ключ и причину для отчёта. Готовая строка содержит подтверждаемые
/// расписания инъекций, фактическое наблюдение остаётся обязанностью прогона.
#[derive(Clone, Debug)]
pub enum DepthRow {
    Excluded {
        key: RunKey,
        id: String,
        constraint: DepthConstraint,
    },
    NotAnalyzed {
        key: RunKey,
        reasons: Vec<String>,
    },
    Bound(PlanRow),
}

impl DepthRow {
    pub fn key(&self) -> &RunKey {
        match self {
            Self::Excluded { key, .. } | Self::NotAnalyzed { key, .. } => key,
            Self::Bound(row) => &row.key,
        }
    }
}

/// Разрешённый граф кампании. Строки создаются по запросу, хранимой матрицы нет.
#[derive(Clone, Debug)]
pub struct DepthPlan<'a> {
    campaign: &'a Campaign,
    factors: &'a DepthFactors,
    inputs: ResolvedInputs,
    nominal: CompiledWorld,
    fingerprint: Fingerprint,
}

impl<'a> DepthPlan<'a> {
    pub(super) fn new(
        campaign: &'a Campaign,
        factors: &'a DepthFactors,
        inputs: ResolvedInputs,
        nominal: CompiledWorld,
        fingerprint: Fingerprint,
    ) -> Self {
        Self {
            campaign,
            factors,
            inputs,
            nominal,
            fingerprint,
        }
    }

    pub fn factors(&self) -> &DepthFactors {
        self.factors
    }

    pub fn fingerprint(&self) -> Fingerprint {
        self.fingerprint
    }

    pub fn row(&self, selection: &Selection) -> Result<DepthRow, Error> {
        let key = self.factors.run_key(selection)?;
        if let Some((id, rule)) = self.factors.exclusion(selection)? {
            return Ok(DepthRow::Excluded {
                key,
                id: id.into(),
                constraint: rule.clone(),
            });
        }
        let open = |key, reason: &str| {
            Ok(DepthRow::NotAnalyzed {
                key,
                reasons: vec![reason.into()],
            })
        };
        let Some(path) = self.inputs.paths.iter().find(|path| {
            path.station == key.station && path.mode == key.mode && path.path == key.path
        }) else {
            return open(key, "путь и режим отсутствуют в разрешённом профиле");
        };
        let Some(grid) = path.grids.iter().find(|grid| grid.id == key.grid) else {
            return open(key, "сетка отсутствует у выбранного пути и режима");
        };
        if key
            .faults
            .iter()
            .any(|fault| !path.faults.contains_key(fault))
        {
            return open(key, "отказ отсутствует у выбранного пути и режима");
        }
        let mode = self
            .campaign
            .stations
            .get(&key.station)
            .and_then(|station| station.modes.get(&key.mode));
        let combination_source = if key.faults.len() > 1 {
            let Some(combination) = path
                .combinations
                .iter()
                .chain(
                    mode.into_iter()
                        .flat_map(|mode| &mode.combinations)
                        .filter(|c| c.path == path.path),
                )
                .find(|c| {
                    c.faults.len() == key.faults.len()
                        && c.faults.iter().all(|f| key.faults.binary_search(f).is_ok())
                })
            else {
                return open(
                    key,
                    "отсутствует описание совместной инъекции выбранных отказов",
                );
            };
            Some(&combination.source)
        } else {
            None
        };
        let needs_velocity = !self.inputs.initial_velocity
            && self.factors.axis_values(selection).any(|(axis, value)| {
                matches!(axis.parameter, Parameter::JointVelocity { .. }) && value != 0.0
            });
        Ok(DepthRow::Bound(self.campaign.row(
            key,
            path,
            mode,
            grid,
            combination_source,
            &self.inputs,
            needs_velocity,
        )))
    }

    pub fn materialize(&self, selection: &Selection) -> Result<SampledInputs, Error> {
        let row = match self.row(selection)? {
            DepthRow::Bound(row) if matches!(row.disposition, Disposition::Ready) => row,
            row => {
                return Err(invalid(format!(
                    "depth_factors: строка недоступна для установки {row:?}"
                )));
            }
        };
        let sample = self.factors.sample(self.campaign.seed, &row.key, selection);
        let inputs = self.campaign.materialize_row(
            &row,
            sample,
            self.factors.axes(),
            &self.inputs,
            &self.nominal,
            self.fingerprint,
        )?;
        inputs
            .world
            .compile()
            .map_err(|error| invalid(format!("depth_factors.{:?}: {error}", row.key)))?;
        Ok(inputs)
    }
}
