//! Допуск происхождения стендового ряда перед передачей штатному оценщику.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use serde::Deserialize;
use sim_reality::{BuildId, Capture, MeasuredCapture, Profile, Receipt};
use sim_task::campaign::RunKey;
use sim_task::verdict::MeasurementSample;
use sim_task::{Fingerprint, Signal, Source, SourceKind};

use crate::tasks::processes::{Inputs, dir};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Document {
    version: u32,
    records: Vec<Attestation>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Attestation {
    experiment: String,
    run: RunKey,
    build_fingerprint: String,
    signal: Signal,
    uncertainty: Option<f64>,
    profile: PathBuf,
    capture: PathBuf,
    receipt: Receipt,
}

pub(super) struct Bench {
    pub experiment: String,
    pub run: RunKey,
    pub build: Fingerprint,
    pub revision: BuildId,
    pub signal: Signal,
    pub source: Source,
    pub uncertainty: Option<f64>,
    pub samples: Vec<MeasurementSample>,
    pub horizon_ns: u64,
}

pub(super) fn load(path: &Path, files: &mut Inputs) -> Result<Vec<Bench>, String> {
    let document: Document = files.toml(path)?;
    if document.version != 1 || document.records.is_empty() {
        return Err("нужен непустой пакет заверенных рядов версии 1".into());
    }
    let mut seen = BTreeSet::new();
    let mut result = Vec::new();
    for entry in document.records {
        if entry.uncertainty.is_some_and(|u| !u.is_finite() || u < 0.0) {
            return Err("некорректная граница погрешности стендового канала".into());
        }
        if !seen.insert((
            entry.experiment.clone(),
            entry.run.clone(),
            entry.signal.name.clone(),
            entry.signal.unit.clone(),
        )) {
            return Err("повтор заверенного канала для одной строки".into());
        }
        let build = u64::from_str_radix(&entry.build_fingerprint, 16)
            .ok()
            .filter(|v| format!("{v:016x}") == entry.build_fingerprint)
            .map(Fingerprint::from_fnv1a)
            .ok_or("некорректный отпечаток сборки в заверении")?;
        let profile: Profile = files.toml(&dir(path).join(&entry.profile))?;
        let path = dir(path).join(&entry.capture);
        let data = std::str::from_utf8(files.capture(&path)?)
            .map_err(|e| format!("{}: {e}", path.display()))?;
        let capture = Capture::parse(data, &profile.channels)?;
        let measured = MeasuredCapture::verify(&capture, &profile.build, &entry.receipt)?;
        let samples = measured.observation(&entry.signal.name, &entry.signal.unit)?;
        let receipt = &entry.receipt;
        result.push(Bench {
            experiment: entry.experiment,
            run: entry.run,
            build,
            revision: profile.build,
            signal: entry.signal,
            uncertainty: entry.uncertainty,
            source: Source {
                kind: SourceKind::Measured,
                reference: format!(
                    "{} / {}, замер {}, лог {}, калибровка {}, область {}",
                    receipt.build.design,
                    receipt.build.revision,
                    receipt.capture_id,
                    receipt.log_fingerprint,
                    receipt.calibration,
                    receipt.scope,
                ),
            },
            samples: samples
                .iter()
                .map(|s| MeasurementSample {
                    at_ns: s.at.0,
                    value: s.value,
                })
                .collect(),
            horizon_ns: capture.horizon().0,
        });
    }
    Ok(result)
}
