//! Совместные и условные выборки. Допуск задаёт область, закон задаёт вероятности.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use sim_signal::Rng;

use crate::{Error, Source, campaign::ModelParameter, invalid};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Scope {
    Batch,
    Specimen,
    Run,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Parameter {
    PlacementOffset {
        placement: String,
        axis: u8,
    },
    LinkOffset {
        placement: String,
        link: String,
        axis: u8,
    },
    ObjectOffset {
        object: String,
        axis: u8,
    },
    Mass {
        body: String,
    },
    Friction {
        pair: [String; 2],
        component: u8,
    },
    SensorNoise {
        sensor: String,
    },
    SensorBias {
        sensor: String,
    },
    SensorLatency {
        sensor: String,
    },
    ChannelDelay {
        channel: String,
    },
    JointPosition {
        joint: String,
    },
    JointVelocity {
        joint: String,
        dof: u8,
    },
    Gravity {
        axis: u8,
    },
    Wind {
        axis: u8,
    },
    Density,
    Viscosity,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Axis {
    pub id: String,
    pub parameter: Parameter,
    pub unit: String,
    pub nominal: f64,
    pub min: f64,
    pub max: f64,
    pub source: Source,
    pub scope: Scope,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Boundary {
    pub id: String,
    pub values: BTreeMap<String, f64>,
    pub source: Source,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Draw {
    Fixed {
        value: f64,
    },
    Uniform {
        min: f64,
        max: f64,
        factor: String,
        from: Option<Scope>,
    },
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Alternative {
    id: String,
    weight: f64,
    values: BTreeMap<String, Draw>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Law {
    source: Source,
    alternatives: Vec<Alternative>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Conditional {
    batch: String,
    specimen: Option<String>,
    law: Law,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Independence {
    Unspecified,
    Batches,
    WithinBatch,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Probability {
    pub batches: u32,
    pub specimens: u32,
    pub repeats: u32,
    pub confidence: f64,
    pub independence: Independence,
    pub source: Source,
    batch: Law,
    specimen: Vec<Conditional>,
    run: Vec<Conditional>,
}

#[derive(Clone, Debug, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sampling {
    #[serde(default)]
    pub(crate) axes: Vec<Axis>,
    #[serde(default)]
    boundaries: Vec<Boundary>,
    /// Источник, задающий номинал и граничные векторы как исчерпывающий набор состояний.
    pub(crate) discrete: Option<Source>,
    pub(crate) probability: Option<Probability>,
    #[serde(skip)]
    validated: bool,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum SampleKey {
    Nominal,
    Boundary {
        id: String,
    },
    Random {
        batch: u32,
        specimen: u32,
        repeat: u32,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct Seeds {
    pub root: u64,
    pub batch: u64,
    pub specimen: u64,
    pub repeat: u64,
    pub run: u64,
}

#[derive(Clone, Debug, Serialize)]
pub struct AppliedValue {
    pub id: String,
    pub parameter: Parameter,
    pub unit: String,
    pub value: f64,
    pub source: Source,
    pub sources: Vec<Source>,
}

#[derive(Clone, Debug, Serialize)]
pub struct Sample {
    pub key: SampleKey,
    pub seeds: Seeds,
    pub alternatives: BTreeMap<Scope, String>,
    pub values: Vec<AppliedValue>,
}

pub(crate) fn name(value: &str) -> Result<(), Error> {
    crate::task::check_name(value)
}

/// Длина считается в байтах, поэтому разделители внутри имени не меняют адрес потока.
pub(crate) fn address(fields: &[&str]) -> String {
    fields
        .iter()
        .map(|field| format!("{}:{field}", field.len()))
        .collect()
}

fn stream(seed: u64, fields: &[&str]) -> Rng {
    Rng::stream(seed, &address(fields))
}

fn factor_scope(current: Scope, from: Option<Scope>) -> Scope {
    from.unwrap_or(current)
}

impl Law {
    fn validate(&self, axes: &BTreeMap<&str, &Axis>, scope: Scope) -> Result<(), Error> {
        self.source.validate()?;
        if self.alternatives.is_empty() {
            return Err(invalid("alternatives: пустой закон"));
        }
        let expected: BTreeSet<_> = axes
            .values()
            .filter(|axis| axis.scope == scope)
            .map(|axis| axis.id.as_str())
            .collect();
        let mut ids = BTreeSet::new();
        let mut sum = 0.0;
        for alternative in &self.alternatives {
            name(&alternative.id)?;
            if !ids.insert(&alternative.id)
                || !alternative.weight.is_finite()
                || alternative.weight <= 0.0
            {
                return Err(invalid(format!(
                    "{}.weight: повтор или недопустимый вес",
                    alternative.id
                )));
            }
            let next = sum + alternative.weight;
            if !next.is_finite() || next <= sum {
                return Err(invalid(format!(
                    "{}.weight: вес неразличим в сумме",
                    alternative.id
                )));
            }
            sum = next;
            if alternative
                .values
                .keys()
                .map(String::as_str)
                .collect::<BTreeSet<_>>()
                != expected
            {
                return Err(invalid(format!(
                    "{}.values: нужны все и только оси уровня {scope:?}",
                    alternative.id
                )));
            }
            for (id, draw) in &alternative.values {
                let axis = axes[id.as_str()];
                let (min, max) = match draw {
                    Draw::Fixed { value } => (*value, *value),
                    Draw::Uniform {
                        min,
                        max,
                        factor,
                        from,
                    } => {
                        name(factor)?;
                        if min >= max || from.is_some_and(|parent| parent >= scope) {
                            return Err(invalid(format!(
                                "{}.values.{id}: неверный интервал или уровень фактора",
                                alternative.id
                            )));
                        }
                        (*min, *max)
                    }
                };
                if !(min.is_finite() && max.is_finite() && axis.min <= min && max <= axis.max) {
                    return Err(invalid(format!(
                        "{}.values.{id}: вне диапазона источника",
                        alternative.id
                    )));
                }
            }
        }
        Ok(())
    }

    fn draw<'a>(&'a self, seed: u64) -> &'a Alternative {
        if let [only] = &self.alternatives[..] {
            return only;
        }
        let total: f64 = self.alternatives.iter().map(|row| row.weight).sum();
        let point = stream(seed, &["campaign/v1", "alternative"]).uniform() * total;
        let mut cumulative = 0.0;
        for row in &self.alternatives {
            cumulative += row.weight;
            if point < cumulative {
                return row;
            }
        }
        // Для точки, округлившейся к правой границе суммы, берём последний интервал.
        self.alternatives.last().expect("закон проверен")
    }
}

impl Probability {
    fn validate(&self, axes: &BTreeMap<&str, &Axis>) -> Result<(), Error> {
        self.source.validate()?;
        if self.batches == 0
            || self.specimens == 0
            || self.repeats == 0
            || !(0.0 < self.confidence && self.confidence < 1.0)
        {
            return Err(invalid(
                "probability: нужны положительные размеры и 0 < confidence < 1",
            ));
        }
        match self.independence {
            Independence::Batches if self.specimens != 1 || self.repeats != 1 => {
                return Err(invalid(
                    "probability.independence: для партий нужен один образец и один прогон на партию",
                ));
            }
            Independence::WithinBatch if self.repeats != 1 => {
                return Err(invalid(
                    "probability.independence: для образцов нужен один прогон на образец",
                ));
            }
            _ => {}
        }
        self.count()?;
        self.batch
            .validate(axes, Scope::Batch)
            .map_err(|e| invalid(format!("probability.batch: {e}")))?;
        let batches: BTreeSet<_> = self
            .batch
            .alternatives
            .iter()
            .map(|a| a.id.as_str())
            .collect();
        let mut specimen = BTreeMap::new();
        for (i, branch) in self.specimen.iter().enumerate() {
            if !batches.contains(branch.batch.as_str())
                || branch.specimen.is_some()
                || specimen
                    .insert(branch.batch.as_str(), &branch.law)
                    .is_some()
            {
                return Err(invalid(format!(
                    "probability.specimen[{i}]: лишняя или повторная ветвь"
                )));
            }
            branch
                .law
                .validate(axes, Scope::Specimen)
                .map_err(|e| invalid(format!("probability.specimen[{i}]: {e}")))?;
        }
        if specimen.len() != batches.len() {
            return Err(invalid(
                "probability.specimen: отсутствует условный закон для партии",
            ));
        }
        let mut run = BTreeMap::new();
        for (i, branch) in self.run.iter().enumerate() {
            let key = (
                branch.batch.as_str(),
                branch.specimen.as_deref().unwrap_or(""),
            );
            if !specimen
                .get(key.0)
                .is_some_and(|law| law.alternatives.iter().any(|a| a.id == key.1))
                || run.insert(key, &branch.law).is_some()
            {
                return Err(invalid(format!(
                    "probability.run[{i}]: лишняя или повторная ветвь"
                )));
            }
            branch
                .law
                .validate(axes, Scope::Run)
                .map_err(|e| invalid(format!("probability.run[{i}]: {e}")))?;
        }

        fn factors(row: &Alternative) -> BTreeSet<&str> {
            row.values
                .values()
                .filter_map(|draw| match draw {
                    Draw::Uniform {
                        factor, from: None, ..
                    } => Some(factor.as_str()),
                    _ => None,
                })
                .collect()
        }
        fn parents(
            row: &Alternative,
            batch: &BTreeSet<&str>,
            specimen: &BTreeSet<&str>,
        ) -> Result<(), Error> {
            for draw in row.values.values() {
                if let Draw::Uniform {
                    factor,
                    from: Some(scope),
                    ..
                } = draw
                {
                    let found = match scope {
                        Scope::Batch => batch.contains(factor.as_str()),
                        Scope::Specimen => specimen.contains(factor.as_str()),
                        Scope::Run => false,
                    };
                    if !found {
                        return Err(invalid(format!(
                            "{}.values: не определён родительский фактор {factor}",
                            row.id
                        )));
                    }
                }
            }
            Ok(())
        }
        for b in &self.batch.alternatives {
            let bf = factors(b);
            for s in &specimen[b.id.as_str()].alternatives {
                parents(s, &bf, &BTreeSet::new())
                    .map_err(|e| invalid(format!("probability.specimen.{}: {e}", b.id)))?;
                let sf = factors(s);
                let law = run.get(&(b.id.as_str(), s.id.as_str())).ok_or_else(|| {
                    invalid(format!(
                        "probability.run: отсутствует закон для {}/{}",
                        b.id, s.id
                    ))
                })?;
                for r in &law.alternatives {
                    parents(r, &bf, &sf)
                        .map_err(|e| invalid(format!("probability.run.{}/{}: {e}", b.id, s.id)))?;
                }
            }
        }
        Ok(())
    }

    pub(crate) fn count(&self) -> Result<usize, Error> {
        (self.batches as usize)
            .checked_mul(self.specimens as usize)
            .and_then(|n| n.checked_mul(self.repeats as usize))
            .ok_or_else(|| invalid("probability: переполнение числа выборок"))
    }

    pub(crate) fn sourced(&self) -> bool {
        self.source.can_prove()
            && self.batch.source.can_prove()
            && self
                .specimen
                .iter()
                .chain(&self.run)
                .all(|branch| branch.law.source.can_prove())
    }
}

impl Sampling {
    pub fn coverage_gap(&self, seed: u64) -> Result<Option<&'static str>, Error> {
        if self.axes.is_empty() {
            return Ok(Some("разброс пуст: отсутствуют оси допуска"));
        }
        let mut distinct = BTreeSet::new();
        let mut random = BTreeSet::new();
        for key in self.keys()? {
            let sample = self.sample(seed, &key, &[])?;
            let vector: Vec<_> = sample
                .values
                .iter()
                .map(|value| {
                    if value.value == 0.0 {
                        0
                    } else {
                        value.value.to_bits()
                    }
                })
                .collect();
            if matches!(key, SampleKey::Random { .. }) {
                random.insert(vector.clone());
            }
            distinct.insert(vector);
        }
        // Для сравнения нужны хотя бы два различимых состояния. Номинал и
        // границы не заменяют второй образец заявленной вероятностной серии.
        if distinct.len() < 2 || self.probability.is_some() && random.len() < 2 {
            return Ok(Some("разброс содержит меньше двух различимых выборок"));
        }
        Ok(None)
    }

    pub fn axes(&self) -> &[Axis] {
        &self.axes
    }

    /// Наложить один и тот же образец на разрешённую постановку любой сетки.
    pub fn apply_to_world(
        &self,
        sample: &Sample,
        nominal: &sim_world::CompiledWorld,
        world: &mut WorldInputs,
        parameters: &BTreeMap<Parameter, ModelParameter>,
    ) -> Result<ModelSettings, Error> {
        self.check_validated()?;
        if sample.values.len() != self.axes.len() {
            return Err(invalid("размер вектора не совпадает с осями разброса"));
        }
        let mut copy = world.clone();
        let mut settings = ModelSettings::default();
        for (axis, value) in self.axes.iter().zip(&sample.values) {
            axis.parameter.check_domain(axis)?;
            if axis.id != value.id
                || axis.unit != value.unit
                || axis.parameter != value.parameter
                || !(axis.min <= value.value && value.value <= axis.max)
            {
                return Err(invalid(format!("неверная выборка для оси «{}»", axis.id)));
            }
            value.source.validate()?;
            if let Some(binding) = parameters.get(&axis.parameter)
                && (axis.min < binding.min || axis.max > binding.max)
            {
                return Err(invalid(format!(
                    "допуск «{}» выходит за область модели",
                    axis.id
                )));
            }
            for bound in [axis.min, axis.max] {
                axis.parameter
                    .apply(
                        Some(axis.nominal),
                        &axis.unit,
                        bound,
                        &axis.source,
                        parameters,
                        nominal,
                        &mut world.clone(),
                        &mut ModelSettings::default(),
                    )
                    .map_err(|e| {
                        invalid(format!("sampling.axes.{}: граница допуска: {e}", axis.id))
                    })?;
            }
            axis.parameter
                .apply(
                    Some(axis.nominal),
                    &axis.unit,
                    value.value,
                    &value.source,
                    parameters,
                    nominal,
                    &mut copy,
                    &mut settings,
                )
                .map_err(|e| invalid(format!("sampling.axes.{}: {e}", axis.id)))?;
        }
        copy.world.seed = sample.seeds.run;
        *world = copy;
        Ok(settings)
    }

    pub fn validate(&mut self) -> Result<(), Error> {
        self.validated = false;
        self.axes.sort_by(|a, b| a.id.cmp(&b.id));
        self.boundaries.sort_by(|a, b| a.id.cmp(&b.id));
        let mut axes = BTreeMap::new();
        let mut targets = BTreeSet::new();
        for axis in &self.axes {
            let mut parameter = axis.parameter.clone();
            if let Parameter::Friction { pair, .. } = &mut parameter {
                pair.sort();
            }
            if axes.insert(axis.id.as_str(), axis).is_some() || !targets.insert(parameter) {
                return Err(invalid(format!(
                    "sampling.axes.{}: повтор имени или адреса",
                    axis.id
                )));
            }
            name(&axis.id)?;
            name(&axis.unit)?;
            axis.source.validate()?;
            axis.parameter.check_domain(axis)?;
        }
        let mut boundaries = BTreeSet::new();
        for boundary in &self.boundaries {
            name(&boundary.id)?;
            boundary.source.validate()?;
            if !boundaries.insert(&boundary.id) || boundary.values.is_empty() {
                return Err(invalid(format!(
                    "sampling.boundaries.{}: пустая или повторная комбинация",
                    boundary.id
                )));
            }
            for (id, value) in &boundary.values {
                if !axes
                    .get(id.as_str())
                    .is_some_and(|a| value.is_finite() && a.min <= *value && *value <= a.max)
                {
                    return Err(invalid(format!(
                        "sampling.boundaries.{}.values.{id}: неизвестная ось или значение вне допуска",
                        boundary.id
                    )));
                }
            }
        }
        if self.probability.is_none() && !self.axes.is_empty() && self.boundaries.is_empty() {
            return Err(invalid(
                "sampling.boundaries: для допуска без закона нужно задать граничные сочетания",
            ));
        }
        if let Some(probability) = &mut self.probability {
            probability.validate(&axes)?;
        }
        if let Some(source) = &self.discrete {
            source.validate()?;
            if self.probability.is_some()
                || !source.can_prove()
                || source.kind == crate::SourceKind::Derived
            {
                return Err(invalid(
                    "sampling.discrete: нужен исходный источник дискретной матрицы без вероятностного закона",
                ));
            }
        }
        self.validated = true;
        Ok(())
    }

    fn check_validated(&self) -> Result<(), Error> {
        if self.validated {
            Ok(())
        } else {
            Err(invalid("разброс должен пройти проверку до расчёта выборок"))
        }
    }

    pub fn keys(&self) -> Result<Vec<SampleKey>, Error> {
        self.check_validated()?;
        let count = self
            .probability
            .as_ref()
            .map(Probability::count)
            .transpose()?
            .unwrap_or(0)
            .checked_add(self.boundaries.len())
            .and_then(|n| n.checked_add(1))
            .ok_or_else(|| invalid("sampling: переполнение числа выборок"))?;
        let mut keys = Vec::new();
        keys.try_reserve_exact(count)
            .map_err(|_| invalid("sampling: число выборок не помещается в память"))?;
        keys.push(SampleKey::Nominal);
        keys.extend(
            self.boundaries
                .iter()
                .map(|b| SampleKey::Boundary { id: b.id.clone() }),
        );
        if let Some(p) = &self.probability {
            for batch in 0..p.batches {
                for specimen in 0..p.specimens {
                    for repeat in 0..p.repeats {
                        keys.push(SampleKey::Random {
                            batch,
                            specimen,
                            repeat,
                        });
                    }
                }
            }
        }
        Ok(keys)
    }

    pub fn sample(&self, root: u64, key: &SampleKey, row: &[&str]) -> Result<Sample, Error> {
        self.check_validated()?;
        let mut alternatives = BTreeMap::new();
        let mut values = BTreeMap::new();
        let mut boundary_source = None;
        let (batch_id, specimen_id, repeat_id) = match key {
            SampleKey::Nominal => ("nominal".into(), String::new(), String::new()),
            SampleKey::Boundary { id } => {
                let boundary = self
                    .boundaries
                    .iter()
                    .find(|b| b.id == *id)
                    .ok_or_else(|| invalid("неизвестная граничная комбинация"))?;
                values = boundary.values.clone();
                boundary_source = Some(&boundary.source);
                ("boundary".into(), id.clone(), String::new())
            }
            SampleKey::Random {
                batch,
                specimen,
                repeat,
            } => {
                if !self.probability.as_ref().is_some_and(|law| {
                    *batch < law.batches && *specimen < law.specimens && *repeat < law.repeats
                }) {
                    return Err(invalid("ключ вне заявленной вероятностной серии"));
                }
                (
                    format!("random/{batch}"),
                    specimen.to_string(),
                    repeat.to_string(),
                )
            }
        };
        let batch = stream(root, &["campaign/v1", "batch", &batch_id]).next_u64();
        let specimen = stream(batch, &["campaign/v1", "specimen", &specimen_id]).next_u64();
        let repeat = stream(specimen, &["campaign/v1", "repeat", &repeat_id]).next_u64();
        // Парные опыты и сетки получают одно зерно физического опыта и одинаковый вектор.
        let run = stream(repeat, row).next_u64();
        let mut law_sources = BTreeMap::new();
        let mut law_chain = Vec::new();
        if matches!(key, SampleKey::Random { .. }) {
            let p = self
                .probability
                .as_ref()
                .ok_or_else(|| invalid("закон выборки отсутствует"))?;
            law_chain.push(&p.source);
            let b = p.batch.draw(batch);
            let s_law = &p
                .specimen
                .iter()
                .find(|x| x.batch == b.id)
                .expect("ветвь проверена")
                .law;
            let s = s_law.draw(specimen);
            let r_law = &p
                .run
                .iter()
                .find(|x| x.batch == b.id && x.specimen.as_ref() == Some(&s.id))
                .expect("ветвь проверена")
                .law;
            let r = r_law.draw(repeat);
            for (scope, alt, source) in [
                (Scope::Batch, b, &p.batch.source),
                (Scope::Specimen, s, &s_law.source),
                (Scope::Run, r, &r_law.source),
            ] {
                alternatives.insert(scope, alt.id.clone());
                law_chain.push(source);
                law_sources.insert(scope, law_chain.clone());
                for (id, draw) in &alt.values {
                    let value = match draw {
                        Draw::Fixed { value } => *value,
                        Draw::Uniform {
                            min,
                            max,
                            factor,
                            from,
                        } => {
                            let seed = match factor_scope(scope, *from) {
                                Scope::Batch => batch,
                                Scope::Specimen => specimen,
                                Scope::Run => repeat,
                            };
                            let q = stream(seed, &["campaign/v1", "factor", factor]).uniform();
                            // Выпуклая комбинация работает и при непредставимой разности max - min.
                            (min * (1.0 - q) + max * q).clamp(*min, *max)
                        }
                    };
                    values.insert(id.clone(), value);
                }
            }
        }
        let mut applied = Vec::new();
        for axis in &self.axes {
            let value = values.get(&axis.id).copied().unwrap_or(axis.nominal);
            if !(value.is_finite() && axis.min <= value && value <= axis.max) {
                return Err(invalid(format!(
                    "sampling.axes.{}: выборка вне диапазона источника",
                    axis.id
                )));
            }
            let mut sources = vec![axis.source.clone()];
            if let Some(law) = law_sources.get(&axis.scope) {
                sources.extend(law.iter().map(|source| (*source).clone()));
            }
            if let Some(boundary) = boundary_source.filter(|_| values.contains_key(&axis.id)) {
                sources.push(boundary.clone());
            }
            // Условный закон зависит от всей выбранной ветви и закона её выбора.
            let source = if sources.len() == 1 {
                axis.source.clone()
            } else {
                Source {
                    kind: if sources.iter().all(Source::can_prove) {
                        crate::SourceKind::Derived
                    } else {
                        crate::SourceKind::Estimate
                    },
                    reference: sources
                        .iter()
                        .map(|s| s.reference.as_str())
                        .collect::<Vec<_>>()
                        .join(" | "),
                }
            };
            applied.push(AppliedValue {
                id: axis.id.clone(),
                parameter: axis.parameter.clone(),
                unit: axis.unit.clone(),
                value,
                source,
                sources,
            });
        }
        Ok(Sample {
            key: key.clone(),
            seeds: Seeds {
                root,
                batch,
                specimen,
                repeat,
                run,
            },
            alternatives,
            values: applied,
        })
    }
}

use sim_world::{Geometry, Joint, JointType, Mass, Provenance, SensorUse};

use crate::campaign::WorldInputs;

fn wrong(parameter: &Parameter) -> Error {
    invalid(format!("{parameter:?}: адрес отсутствует или неоднозначен"))
}

pub(crate) fn field(context: &str, result: Result<(), Error>) -> Result<(), Error> {
    result.map_err(|error| invalid(format!("{context}: {error}")))
}

impl Parameter {
    pub(crate) fn check_domain(&self, a: &Axis) -> Result<(), Error> {
        use Parameter::*;
        let (unit, coordinate, lower) = match self {
            PlacementOffset { axis, .. } | LinkOffset { axis, .. } | ObjectOffset { axis, .. } => {
                (Some("m"), Some(*axis), None)
            }
            Gravity { axis } => (Some("m/s^2"), Some(*axis), None),
            Wind { axis } => (Some("m/s"), Some(*axis), None),
            Density => (Some("kg/m^3"), None, Some(0.0)),
            Viscosity => (Some("Pa*s"), None, Some(0.0)),
            Mass { .. } => (Some("kg"), None, None),
            Friction { pair, component } => {
                if pair[0] == pair[1] {
                    return Err(wrong(self));
                }
                // В ContactSettings пять компонент MuJoCo: две касательные, кручение, два качения.
                let unit = match component {
                    0 | 1 => "1",
                    2..=4 => "m",
                    _ => return Err(wrong(self)),
                };
                (Some(unit), None, Some(sim_world::MIN_CONTACT_FRICTION))
            }
            SensorLatency { .. } | ChannelDelay { .. } => (Some("s"), None, Some(0.0)),
            SensorNoise { .. } => (None, None, Some(0.0)),
            SensorBias { .. } | JointPosition { .. } | JointVelocity { .. } => (None, None, None),
        };
        if !(a.min.is_finite()
            && a.max.is_finite()
            && a.nominal.is_finite()
            && a.min <= a.nominal
            && a.nominal <= a.max)
            || unit.is_some_and(|u| u != a.unit)
            || coordinate.is_some_and(|i| i >= 3)
            || lower.is_some_and(|bound| a.min < bound)
            || matches!(self, Mass { .. }) && a.min <= 0.0
            || matches!(self, SensorLatency { .. } | ChannelDelay { .. })
                && delay_ns(a.max).is_none()
            || matches!(self, Friction { .. }) && !(a.max * a.max).is_finite()
        {
            return Err(invalid(format!(
                "sampling.axes.{}: неверная единица, номинал или пределы",
                a.id
            )));
        }
        Ok(())
    }
}

pub fn delay_ns(seconds: f64) -> Option<u64> {
    // Как в настройках датчиков: единицы СИ → наносекунды, без сокращения задержки.
    let ns = (seconds * 1_000_000_000.0).ceil();
    (seconds >= 0.0 && ns.is_finite() && ns < u64::MAX as f64).then_some(ns as u64)
}

/// Применены на копии мира либо предназначены для явной установки сборщиком.
#[derive(Clone, Debug, Default, Serialize)]
pub struct ModelSettings {
    pub initial_velocity: BTreeMap<String, BTreeMap<u8, f64>>,
    /// Наносекунды проводки с причинным округлением выборки вверх.
    pub channel_delay_ns: BTreeMap<String, u64>,
}

fn device_link<'a>(
    world: &'a mut WorldInputs,
    placement: &str,
    link: &str,
) -> Option<&'a mut sim_world::Link> {
    world
        .devices
        .get_mut(placement)?
        .device
        .links
        .iter_mut()
        .find(|l| l.name == link)
}

fn object<'a>(world: &'a mut WorldInputs, name: &str) -> Option<&'a mut sim_world::Object> {
    world
        .world
        .objects
        .iter_mut()
        .chain(&mut world.contact.added_objects)
        .find(|o| o.name == name)
}

fn joint<'a>(world: &'a mut WorldInputs, name: &str) -> Option<&'a mut Joint> {
    let (owner, local) = name.split_once('/')?;
    world
        .devices
        .get_mut(owner)?
        .device
        .links
        .iter_mut()
        .filter(|link| link.joint.kind != JointType::Fixed)
        .find(|link| link.joint.name.as_deref().unwrap_or(&link.name) == local)
        .map(|link| &mut link.joint)
}

fn sensor<'a>(world: &'a mut WorldInputs, name: &str) -> Option<&'a mut SensorUse> {
    if let Some((owner, local)) = name.split_once('/') {
        world
            .devices
            .get_mut(owner)?
            .device
            .sensors
            .iter_mut()
            .find(|s| s.name == local)
    } else {
        world.world.sensors.iter_mut().find(|s| s.name == name)
    }
}

fn scaled(value: &mut f64, factor: f64) -> Result<(), Error> {
    let next = *value * factor;
    if !next.is_finite() || (*value != 0.0 && next == 0.0) {
        return Err(invalid(
            "масса или составляющая инерции не представима после масштабирования",
        ));
    }
    *value = next;
    Ok(())
}

fn mass_provenance(old: &mut Provenance, source: &Source) {
    let reference = format!("{old:?} | {}", source.reference);
    *old = if old.can_prove() && source.can_prove() {
        Provenance::Derived(reference)
    } else {
        Provenance::Estimate(reference)
    };
}

fn scale_density(
    mass: &mut Mass,
    collision: &mut [Geometry],
    factor: f64,
    source: &Source,
) -> Result<(), Error> {
    let (material, volumes) = match mass {
        Mass::Explicit {
            mass_kg,
            inertia_kg_m2,
            provenance,
            ..
        } => {
            scaled(mass_kg, factor)?;
            for component in inertia_kg_m2 {
                scaled(component, factor)?;
            }
            mass_provenance(provenance, source);
            return Ok(());
        }
        Mass::Geometry { material } => (material, collision),
        Mass::Volumes { material, volumes } => (material, volumes.as_mut_slice()),
        Mass::Massless => return Err(invalid("у звена нет массы для масштабирования")),
    };
    scaled(&mut material.density_kg_m3, factor)?;
    mass_provenance(&mut material.provenance, source);
    for geometry in volumes {
        if let Some(material) = &mut geometry.material {
            scaled(&mut material.density_kg_m3, factor)?;
            mass_provenance(&mut material.provenance, source);
        }
        if let Some(mass) = &mut geometry.explicit_mass {
            scaled(&mut mass.mass_kg, factor)?;
            mass_provenance(&mut mass.provenance, source);
        }
    }
    Ok(())
}

impl Parameter {
    /// Проверяет адрес, единицу и номинал разрешённой модели, затем изменяет копию.
    pub(crate) fn apply(
        &self,
        expected: Option<f64>,
        input_unit: &str,
        value: f64,
        source: &Source,
        parameters: &BTreeMap<Parameter, ModelParameter>,
        nominal: &sim_world::CompiledWorld,
        world: &mut WorldInputs,
        settings: &mut ModelSettings,
    ) -> Result<(), Error> {
        let absent = || wrong(self);
        let (base, unit) = match self {
            Self::PlacementOffset { placement, axis } => {
                let t = &mut world
                    .world
                    .devices
                    .iter_mut()
                    .find(|p| p.name == *placement)
                    .ok_or_else(absent)?
                    .pose
                    .position_m[usize::from(*axis)];
                (std::mem::replace(t, value), "m")
            }
            Self::LinkOffset {
                placement,
                link,
                axis,
            } => {
                let t = &mut device_link(world, placement, link)
                    .ok_or_else(absent)?
                    .pose
                    .position_m[usize::from(*axis)];
                (std::mem::replace(t, value), "m")
            }
            Self::ObjectOffset { object: name, axis } => {
                let t = &mut object(world, name).ok_or_else(absent)?.pose.position_m
                    [usize::from(*axis)];
                (std::mem::replace(t, value), "m")
            }
            Self::Gravity { axis } => (
                std::mem::replace(&mut world.world.gravity_m_s2[usize::from(*axis)], value),
                "m/s^2",
            ),
            Self::Wind { axis } => (
                std::mem::replace(
                    &mut world.world.environment.wind_m_s[usize::from(*axis)],
                    value,
                ),
                "m/s",
            ),
            Self::Density => (
                std::mem::replace(&mut world.world.environment.density_kg_m3, value),
                "kg/m^3",
            ),
            Self::Viscosity => (
                std::mem::replace(&mut world.world.environment.viscosity_pa_s, value),
                "Pa*s",
            ),
            Self::Mass { body } => {
                let entry = nominal
                    .bodies
                    .iter()
                    .find(|b| b.name == *body && b.mass_kg > 0.0)
                    .ok_or_else(absent)?;
                let (owner, link) = body.split_once('/').ok_or_else(absent)?;
                let (mass, collision) = if world.devices.contains_key(owner) {
                    let link = device_link(world, owner, link).ok_or_else(absent)?;
                    (&mut link.mass, &mut link.collision)
                } else if owner == link {
                    let o = object(world, owner).ok_or_else(absent)?;
                    (&mut o.mass, &mut o.collision)
                } else {
                    return Err(absent());
                };
                // Геометрия и центр масс фиксированы. Интеграл инерции линеен по плотности.
                if value.to_bits() != entry.mass_kg.to_bits() || !source.can_prove() {
                    scale_density(mass, collision, value / entry.mass_kg, source)?;
                }
                (entry.mass_kg, "kg")
            }
            Self::Friction { pair, component } => {
                let entry = world
                    .contact
                    .pairs
                    .iter_mut()
                    .find(|p| {
                        p.geometries == *pair
                            || p.geometries[0] == pair[1] && p.geometries[1] == pair[0]
                    })
                    .ok_or_else(absent)?;
                let dim = entry.settings.dim;
                let active = match component {
                    0 | 1 => dim >= 3,
                    2 => dim >= 4,
                    3 | 4 => dim == 6,
                    _ => false,
                };
                if !active {
                    return Err(invalid(
                        "компонента трения не действует при заданной размерности контакта",
                    ));
                }
                (
                    std::mem::replace(&mut entry.settings.friction[usize::from(*component)], value),
                    if *component < 2 { "1" } else { "m" },
                )
            }
            Self::SensorNoise { sensor: id }
            | Self::SensorBias { sensor: id }
            | Self::SensorLatency { sensor: id } => {
                let model = sensor(world, id).ok_or_else(absent)?;
                let setting = match self {
                    Self::SensorNoise { .. } => "noise_std",
                    Self::SensorBias { .. } => "bias",
                    _ => "latency_s",
                };
                let effective = parameters.get(self).ok_or_else(absent)?;
                let base = model
                    .params
                    .get(setting)
                    .copied()
                    .unwrap_or(effective.nominal);
                if base != effective.nominal {
                    return Err(invalid(
                        "эффективный параметр датчика расходится с экземпляром мира",
                    ));
                }
                if matches!(self, Self::SensorLatency { .. }) && effective.unit != "s" {
                    return Err(invalid("задержка датчика должна быть в секундах"));
                }
                model.params.insert(setting.into(), value);
                (base, effective.unit.as_str())
            }
            Self::ChannelDelay { channel } => {
                let effective = parameters.get(self).ok_or_else(absent)?;
                if effective.unit != "s" {
                    return Err(invalid("задержка канала должна быть в секундах"));
                }
                let installed = delay_ns(value)
                    .ok_or_else(|| invalid("задержка за пределами шкалы времени"))?;
                settings.channel_delay_ns.insert(channel.clone(), installed);
                (effective.nominal, "s")
            }
            Self::JointPosition { joint: id } => {
                let j = joint(world, id).ok_or_else(absent)?;
                match j.kind {
                    JointType::Hinge => {
                        let degrees = value.to_degrees();
                        if !degrees.is_finite()
                            || j.limits_deg
                                .is_some_and(|[a, b]| degrees < a || degrees > b)
                        {
                            return Err(invalid("начальный угол вне диапазона соединения"));
                        }
                        (
                            j.initial_deg.replace(degrees).unwrap_or(0.0).to_radians(),
                            "rad",
                        )
                    }
                    JointType::Slide => {
                        if j.limits_m.is_some_and(|[a, b]| value < a || value > b) {
                            return Err(invalid("начальная координата вне диапазона соединения"));
                        }
                        (j.initial_m.replace(value).unwrap_or(0.0), "m")
                    }
                    _ => {
                        return Err(invalid(
                            "для начального положения нужна скалярная ось или поза размещения",
                        ));
                    }
                }
            }
            Self::JointVelocity { joint: id, dof } => {
                use sim_world::CompiledJointKind::*;
                let unit = match nominal
                    .joints
                    .iter()
                    .find(|j| j.name == *id)
                    .ok_or_else(absent)?
                    .kind
                {
                    Hinge if *dof == 0 => "rad/s",
                    Slide if *dof == 0 => "m/s",
                    Ball if *dof < 3 => "rad/s",
                    Free if *dof < 3 => "m/s",
                    Free if *dof < 6 => "rad/s",
                    _ => return Err(absent()),
                };
                settings
                    .initial_velocity
                    .entry(id.clone())
                    .or_default()
                    .insert(*dof, value);
                // В формате мира v3 нет начальной скорости; данные MuJoCo создаются с qvel = 0.
                (0.0, unit)
            }
        };
        if expected.is_some_and(|nominal| nominal != base) || input_unit != unit {
            return Err(invalid(format!(
                "номинал или единица расходятся с моделью: {base} {unit}"
            )));
        }
        Ok(())
    }
}
