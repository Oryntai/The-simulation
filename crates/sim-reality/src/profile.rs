use std::{
    collections::{BTreeMap, BTreeSet},
    path::{Path, PathBuf},
};

use serde::{Deserialize, Serialize};

use crate::{
    BuildId, Calibration, Capture, Channel, CurveBinding, Experiment, ExperimentKind,
    Identification, InputBinding, MeasuredCapture, Origin, Receipt, ReplayReport, ReplayRuntime,
    Target, identify, replay,
};

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Profile {
    pub build: BuildId,
    pub channels: Vec<Channel>,
    pub models: Vec<String>,
    #[serde(default)]
    pub experiments: Vec<Experiment>,
    pub simulation: Option<Simulation>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Simulation {
    pub world: PathBuf,
    pub catalog: Option<PathBuf>,
    #[serde(default)]
    pub inputs: Vec<InputBinding>,
    #[serde(default)]
    pub curves: Vec<CurveBinding>,
    #[serde(default)]
    pub apply: Vec<Installation>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Installation {
    pub model: String,
    pub target: Target,
}

#[derive(Debug, Serialize)]
pub struct ModelReport {
    pub comparison: &'static str,
    pub reason: Option<&'static str>,
    pub curves: Vec<usize>,
    pub identifications: Vec<usize>,
}

#[derive(Debug, Serialize)]
pub struct Report {
    pub build: BuildId,
    pub origin: Origin,
    pub log_fingerprint: String,
    pub physical_check: &'static str,
    pub safety_evidence: &'static str,
    pub models: BTreeMap<String, ModelReport>,
    pub identifications: Vec<Identification>,
    pub calibrations: Vec<Calibration>,
    pub replay: Option<ReplayReport>,
}

pub fn analyze(
    profile: &Profile,
    root: &Path,
    csv: &str,
    receipt: Option<&Receipt>,
) -> Result<Report, String> {
    let capture = Capture::parse(csv, &profile.channels)?;
    if capture.build() != &profile.build {
        return Err("ревизия журнала отличается от ревизии профиля".into());
    }
    let declared: BTreeSet<_> = profile.models.iter().map(String::as_str).collect();
    if declared.len() != profile.models.len()
        || declared.iter().any(|m| m.trim().is_empty())
        || declared.is_empty()
    {
        return Err("в профиле нужен непустой список уникальных моделей".into());
    }
    let check_model = |name: &str| {
        if declared.contains(name) {
            Ok(())
        } else {
            Err(format!("не объявлена модель «{name}»"))
        }
    };
    let attested = receipt
        .map(|r| MeasuredCapture::verify(&capture, &profile.build, r))
        .transpose()?;
    let mut identifications = Vec::new();
    let mut calibrations = BTreeMap::new();
    for (index, experiment) in profile.experiments.iter().enumerate() {
        check_model(&experiment.model)?;
        let result = identify(&capture, experiment)?;
        if let Some(source) = &attested {
            // Для задержки исключение означает потерю соответствия маркеров.
            // В других опытах точки возле нулевой нагрузки исключаются по методике.
            if matches!(experiment.input, ExperimentKind::Delay { .. })
                && result.excluded_samples() != 0
            {
                return Err(format!(
                    "для замера «{}» нужно полное соответствие маркеров журнала",
                    experiment.model
                ));
            }
            if let Some(calibration) =
                Calibration::from_identification_if_resolved(&result, source)?
            {
                calibrations.insert(index, calibration);
            }
        }
        identifications.push(result);
    }
    let replay = profile
        .simulation
        .as_ref()
        .map(|sim| {
            let mut world = sim_world::load_and_compile(&root.join(&sim.world))?;
            // Модели установленных приводов и датчиков перечисляются в отчёте даже без кривых.
            let actual: BTreeSet<_> = world
                .drives
                .iter()
                .map(|m| m.name.as_str())
                .chain(world.sensors.iter().map(|m| m.name.as_str()))
                .collect();
            for entry in actual {
                check_model(entry)?;
            }
            let mut installations = BTreeSet::new();
            for item in &sim.apply {
                let matches: Vec<_> = profile
                    .experiments
                    .iter()
                    .enumerate()
                    .filter(|(_, e)| e.model == item.model && item.target.accepts(&e.input))
                    .collect();
                let [(index, _)] = matches.as_slice() else {
                    return Err(format!(
                        "для установки «{}» нужен один подходящий опыт",
                        item.model
                    ));
                };
                let calibration = calibrations.get(index).ok_or_else(|| {
                    format!("для установки «{}» нужен заверенный замер", item.model)
                })?;
                if !installations.insert((&item.model, item.target)) {
                    return Err(format!("повторная установка замера «{}»", item.model));
                }
                calibration.apply(&profile.build, &mut world, item.target)?;
            }
            for curve in &sim.curves {
                check_model(&curve.model)?;
            }
            let catalog = sim.catalog.as_ref().map(|p| root.join(p));
            let runtime = ReplayRuntime::load(world, catalog.as_deref())?;
            replay(&capture, runtime, &sim.inputs, &sim.curves)
        })
        .transpose()?;
    let models = profile
        .models
        .iter()
        .map(|name| {
            let curves = replay
                .iter()
                .flat_map(|r| r.curves.iter().enumerate())
                .filter_map(|(i, c)| (&c.model == name).then_some(i))
                .collect::<Vec<_>>();
            let states = || {
                curves
                    .iter()
                    .map(|&i| replay.as_ref().unwrap().curves[i].state)
            };
            let (comparison, reason) = if curves.is_empty() {
                (
                    "not_analyzed",
                    Some("для модели не задана кривая сравнения"),
                )
            } else if states().all(|s| s == "not_analyzed") {
                ("not_analyzed", Some("нет сопоставимых отсчётов модели"))
            } else if states().any(|s| s != "compared") {
                ("partial", Some("часть кривых модели покрыта неполностью"))
            } else {
                ("compared", None)
            };
            (
                name.clone(),
                ModelReport {
                    comparison,
                    reason,
                    curves,
                    identifications: identifications
                        .iter()
                        .enumerate()
                        .filter_map(|(i, result)| (result.model() == name).then_some(i))
                        .collect(),
                },
            )
        })
        .collect();
    let physical_check = if attested.is_none() {
        "не проверено на реальном устройстве"
    } else if replay
        .as_ref()
        .is_some_and(|r| r.curves.iter().any(|c| c.gap.is_some()))
    {
        if profile
            .simulation
            .as_ref()
            .is_some_and(|s| !s.apply.is_empty())
        {
            "сравнение по журналу калибровки"
        } else {
            "сравнение по стендовому журналу"
        }
    } else {
        "стендовый журнал без пригодного сравнения с симуляцией"
    };
    Ok(Report {
        build: capture.build().clone(),
        origin: capture.origin(),
        log_fingerprint: format!("{:016x}", capture.fingerprint()),
        physical_check,
        safety_evidence: "not_analyzed",
        models,
        identifications,
        calibrations: calibrations.into_values().collect(),
        replay,
    })
}
