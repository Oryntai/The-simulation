//! Переносимый пакет просмотра. В хвосте хранится исходный SIMLOG без изменений.

use std::collections::BTreeMap;
use std::io::{self, Write};
use std::path::Path;

use serde::{Deserialize, Serialize};
use sim_process::{Analysis, attachments::AttachmentOutcome};
use sim_runtime::{Log, LogKind, Runtime, ViewScene};
use sim_signal::{ImageFrame, PixelFormat, SignalId, Time, fnv1a};
use sim_task::{Fingerprint, Outcome, campaign::RunKey, verdict::{Readiness, SimulationVerdict}};

use crate::{Options, export};

const MAGIC: &[u8] = b"SIMVIEW\x01";

#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SceneRecord {
    pub name: String,
    pub mjcf: String,
    pub engine: String,
    pub effort_dofs: Vec<usize>,
    pub sensors: Vec<SensorMount>,
}

impl SceneRecord {
    pub fn world_hash(&self) -> u64 {
        ViewScene::world_hash(&self.name, &self.mjcf, &self.engine, &self.effort_dofs)
    }
}

#[derive(Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SensorMount {
    pub name: String,
    pub body: Option<String>,
    pub position_m: [f64; 3],
    pub quat_wxyz: [f64; 4],
}

/// Метка максимума в одном конкретном журнале. Используется для просмотра.
#[derive(Deserialize, Serialize, PartialEq, Debug)]
#[serde(deny_unknown_fields)]
pub struct Peak {
    pub at_ns: u64,
    pub signal: String,
    pub unit: String,
    pub value: f64,
}

#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Reaction {
    pub kind: ReactionKind,
    pub pair: [String; 2],
    pub start_ns: u64,
    pub end_ns: u64,
    pub point_m: [f64; 3],
    pub force_n: [f64; 3],
    pub torque_nm: [f64; 3],
}

#[derive(Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ReactionKind {
    Contact,
    Weld,
}

#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Scalar {
    pub at_ns: u64,
    pub name: String,
    pub unit: String,
    pub value: Option<f64>,
}

#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct MountEvent {
    pub name: String,
    pub requested_ns: u64,
    pub evaluated_ns: u64,
    pub action: String,
    pub outcome: MountOutcome,
}

#[derive(Deserialize, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum MountOutcome {
    Transition { from: String, to: String },
    Rejected { reason: String },
    NotAnalyzed { reason: String },
    Equality { active: bool },
}

#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TimedProcessEvent {
    pub name: String,
    pub state: String,
    pub requested_ns: u64,
    pub applied_ns: u64,
}

/// Нагрузка на тело в мировой системе. Момент приведён к центру масс.
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct InterfaceLoad {
    pub patch: String,
    pub body: String,
    pub start_ns: u64,
    pub end_ns: u64,
    pub force_n: [f64; 3],
    pub torque_nm: [f64; 3],
    pub from_engine: bool,
}

#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct FrameRecord {
    signal: u32,
    captured_ns: u64,
    at_ns: u64,
    width: u32,
    height: u32,
    #[serde(with = "pixel_format")]
    format: PixelFormat,
    valid: bool,
}

mod pixel_format {
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    use sim_signal::PixelFormat;

    #[derive(Deserialize, Serialize)]
    #[serde(rename_all = "snake_case")]
    enum Format {
        Rgb8,
        DepthMm16,
    }
    pub fn serialize<S: Serializer>(value: &PixelFormat, s: S) -> Result<S::Ok, S::Error> {
        match value {
            PixelFormat::Rgb8 => Format::Rgb8,
            PixelFormat::DepthMm16 => Format::DepthMm16,
        }
        .serialize(s)
    }
    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<PixelFormat, D::Error> {
        Ok(match Format::deserialize(d)? {
            Format::Rgb8 => PixelFormat::Rgb8,
            Format::DepthMm16 => PixelFormat::DepthMm16,
        })
    }
}

impl FrameRecord {
    fn bytes(&self) -> Option<usize> {
        (self.width as usize)
            .checked_mul(self.height as usize)?
            .checked_mul(self.format.bytes_per_pixel())
    }
}

/// Момент из оценщика с указанием роли события в отчёте.
#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct CampaignEvent {
    pub at_ns: u64,
    pub label: String,
    pub violation: bool,
}

/// Дочерний опыт и расчётная сетка, которым принадлежит курсор.
#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct CampaignPlayback {
    pub child: RunKey,
    pub variant: Option<String>,
    pub physics_hz: u64,
    pub seed: u64,
    pub journal: u64,
    pub end_ns: u64,
    pub event: Option<CampaignEvent>,
}

/// Покрытие сборки из того же отчёта, что и вердикт.
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct CampaignCoverage {
    pub required: usize,
    pub executed: usize,
    pub omitted: usize,
    pub open: usize,
}

/// Ссылка на строку отчёта и вердикт конвейера при её повторе.
#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct CampaignBinding {
    pub key: String,
    pub input: Fingerprint,
    pub playback: CampaignPlayback,
    pub verdict: SimulationVerdict,
    pub readiness: Readiness,
    pub coverage: CampaignCoverage,
}

/// Читает привязки из JSON штатного отчёта, сохраняя точность целых отпечатков.
pub fn campaign_bindings(report: &str) -> Result<BTreeMap<String, CampaignBinding>, String> {
    #[derive(Deserialize)]
    struct FindingKey {
        run: Option<RunKey>,
    }
    #[derive(Deserialize)]
    struct FindingEvent {
        key: Option<FindingKey>,
        outcome: Option<Outcome>,
        event_ns: Option<u64>,
        journal: Option<Fingerprint>,
    }
    #[derive(Deserialize)]
    struct Row {
        input: Option<Fingerprint>,
        playback: Option<CampaignPlayback>,
        #[serde(default)]
        findings: Vec<FindingEvent>,
    }
    #[derive(Deserialize)]
    struct Assembly {
        verdict: SimulationVerdict,
        readiness: Option<Readiness>,
        coverage: Option<CampaignCoverage>,
        #[serde(default)]
        runs: BTreeMap<String, Row>,
    }
    #[derive(Deserialize)]
    struct Report {
        #[serde(default)]
        assemblies: BTreeMap<String, Assembly>,
    }
    let report: Report = serde_json::from_str(report).map_err(|e| e.to_string())?;
    let mut result = BTreeMap::new();
    for (id, assembly) in report.assemblies {
        for (key, row) in assembly.runs {
            if let Some(mut playback) = row.playback {
                if key.split_once('/').map(|(id, _)| id) != Some(&id) {
                    return Err("ключ привязки относится к другой сборке".into());
                }
                playback.event = playback.event.filter(|event| event.violation);
                if let Some(event) = &playback.event
                    && (event.at_ns > playback.end_ns || !row.findings.iter().any(|finding| {
                        finding.outcome == Some(Outcome::Fail)
                            && finding.event_ns == Some(event.at_ns)
                            && finding.journal == Some(Fingerprint::from_fnv1a(playback.journal))
                            && finding.key.as_ref().and_then(|key| key.run.as_ref()) == Some(&playback.child)
                    }))
                {
                    return Err("событие просмотра отсутствует в строке отчёта оценщика".into());
                }
                result.insert(key.clone(), CampaignBinding {
                    key,
                    input: row.input.ok_or("в привязке нет отпечатка входов")?,
                    playback,
                    verdict: assembly.verdict,
                    readiness: assembly.readiness.ok_or("в привязке нет ступени готовности")?,
                    coverage: assembly.coverage.ok_or("в привязке нет покрытия сборки")?,
                });
            }
        }
    }
    Ok(result)
}

#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ViewMetadata {
    pub scene: SceneRecord,
    pub journal: u64,
    pub mode: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub campaign: Option<CampaignBinding>,
    pub case: Option<sim_task::Case>,
    pub physics_hz: f64,
    pub peak: Option<Peak>,
    pub reactions: Vec<Reaction>,
    pub contact_gap: Option<String>,
    pub process: Vec<Scalar>,
    pub attachments: Vec<MountEvent>,
    pub process_events: Vec<TimedProcessEvent>,
    pub interface_loads: Vec<InterfaceLoad>,
    peak_signal: Option<String>,
    frames: Vec<FrameRecord>,
}

/// Проверенный пакет. Отдельные байты журнала подходят для штатного воспроизведения.
pub struct RunView {
    pub metadata: ViewMetadata,
    pub frames: Vec<ImageFrame>,
    pub journal: Log,
    pub binary: Vec<u8>,
}

impl RunView {
    pub fn read(bytes: &[u8]) -> Result<Self, String> {
        let mut data = bytes
            .strip_prefix(MAGIC)
            .ok_or("неизвестный формат SIMVIEW")?;
        let header_len = u64::from_le_bytes(take(&mut data, 8)?.try_into().unwrap());
        let checksum = u64::from_le_bytes(take(&mut data, 8)?.try_into().unwrap());
        let header = take(
            &mut data,
            usize::try_from(header_len).map_err(|e| e.to_string())?,
        )?;
        let mut metadata: ViewMetadata = serde_json::from_slice(header).map_err(|e| e.to_string())?;
        let mut frames = Vec::new();
        for spec in &metadata.frames {
            let length = spec.bytes().ok_or("переполнение размера кадра")?;
            frames.push(ImageFrame {
                signal: SignalId(spec.signal),
                captured: Time(spec.captured_ns),
                at: Time(spec.at_ns),
                width: spec.width,
                height: spec.height,
                format: spec.format,
                data: take(&mut data, length)?.into(),
                valid: spec.valid,
            });
        }
        if checksum != view_checksum(header, &frames) {
            return Err("повреждён заголовок или кадры SIMVIEW".into());
        }
        let journal = Log::read_binary(data).map_err(|e| e.to_string())?;
        metadata.validate(&journal, &frames)?;
        if metadata.peak != metadata.select_peak(&journal, metadata.peak_signal.as_deref())? {
            return Err("отметка события не совпадает с экстремумом ряда".into());
        }
        if let Some(binding) = &mut metadata.campaign {
            binding.playback.event = binding.playback.event.take().filter(|event| event.violation);
        }
        Ok(Self {
            metadata,
            frames,
            journal,
            binary: data.to_vec(),
        })
    }
}

fn take<'a>(source: &mut &'a [u8], count: usize) -> Result<&'a [u8], String> {
    let (part, rest) = source
        .split_at_checked(count)
        .ok_or("оборванный пакет SIMVIEW")?;
    *source = rest;
    Ok(part)
}

fn view_checksum(header: &[u8], frames: &[ImageFrame]) -> u64 {
    fnv1a(
        header
            .iter()
            .copied()
            .chain(frames.iter().flat_map(|f| f.data.iter().copied())),
    )
}

impl ViewMetadata {
    fn validate(&self, log: &Log, frames: &[ImageFrame]) -> Result<(), String> {
        let end = log.end().time.0;
        if let Some(binding) = &self.campaign {
            let p = &binding.playback;
            let coverage = binding.coverage;
            let verdict_matches = match binding.verdict {
                SimulationVerdict::Rejected => binding.readiness == Readiness::Rejected,
                SimulationVerdict::NotProven => binding.readiness == Readiness::NeedsWork,
                SimulationVerdict::PassedSimulation => binding.readiness != Readiness::Rejected
                    && coverage.executed == coverage.required,
            };
            if !binding.key.contains('/') || !verdict_matches
                || coverage.executed == 0
                || coverage.executed.checked_add(coverage.omitted)
                    .and_then(|n| n.checked_add(coverage.open)) != Some(coverage.required)
                || self.mode.as_deref() != Some(&p.child.mode)
                || self.physics_hz != p.physics_hz as f64
                || p.seed != log.header().seed || p.journal != self.journal || p.end_ns != end
                || p.event.as_ref().is_some_and(|event| {
                    event.at_ns > end || event.label.trim().is_empty()
                })
                || self.case.as_ref().is_some_and(|case| {
                    case.path != p.child.path
                        || !case.fault.iter().eq(p.child.faults.iter())
                })
            {
                return Err("пакет просмотра не соответствует дочернему опыту кампании".into());
            }
        }
        if self.scene.world_hash() != log.header().world_hash
            || self.journal != log.fingerprint()
            || !self.physics_hz.is_finite()
            || self.physics_hz <= 0.0
            || self
                .case
                .as_ref()
                .is_some_and(|c| Some(&c.mode) != self.mode.as_ref())
            || self.scene.sensors.iter().any(|s| {
                s.name.trim().is_empty()
                    || s.position_m
                        .iter()
                        .chain(&s.quat_wxyz)
                        .any(|x| !x.is_finite())
                    || s.quat_wxyz.iter().all(|&x| x == 0.0)
            })
            || self
                .peak
                .as_ref()
                .is_some_and(|p| p.at_ns > end || !p.value.is_finite())
            || self
                .process
                .iter()
                .any(|p| p.at_ns > end || p.value.is_some_and(|v| !v.is_finite()))
            || self.reactions.iter().any(|r| {
                r.start_ns >= r.end_ns
                    || r.end_ns > end
                    || r.point_m
                        .iter()
                        .chain(&r.force_n)
                        .chain(&r.torque_nm)
                        .any(|v| !v.is_finite())
            })
            || self
                .attachments
                .iter()
                .any(|a| a.requested_ns > a.evaluated_ns || a.evaluated_ns > end)
            || self
                .process_events
                .iter()
                .any(|e| e.requested_ns > e.applied_ns || e.applied_ns > end)
            || self.interface_loads.iter().any(|v| {
                v.start_ns >= v.end_ns
                    || v.force_n.iter().chain(&v.torque_nm).any(|x| !x.is_finite())
            })
        {
            return Err("пакет просмотра расходится с моделью или журналом".into());
        }
        if frames.windows(2).any(|f| f[0].at > f[1].at) {
            return Err("порядок записанных кадров нарушен".into());
        }
        let entries = log.entries();
        for frame in frames {
            let index = entries.partition_point(|e| e.at < frame.at);
            let fingerprint = frame.fingerprint();
            if frame.width == 0
                || frame.height == 0
                || frame.captured > frame.at
                || !frame.is_consistent()
                || !entries[index..]
                    .iter()
                    .take_while(|e| e.at == frame.at)
                    .any(|e| {
                        e.kind == LogKind::Observation
                            && e.id == frame.signal.0
                            && e.valid == frame.valid
                            && e.value == fingerprint
                    })
            {
                return Err("кадр расходится с записью датчика в SIMLOG".into());
            }
        }
        Ok(())
    }

    fn from_runtime(runtime: &Runtime, mode: Option<&str>, case: Option<&sim_task::Case>) -> Self {
        let scene = runtime.view_scene();
        Self {
            scene: SceneRecord {
                name: scene.name.clone(),
                mjcf: scene.mjcf.clone(),
                engine: scene.engine.clone(),
                effort_dofs: scene.effort_dofs.clone(),
                sensors: scene
                    .sensors
                    .iter()
                    .map(|sensor| SensorMount {
                        name: sensor.name.clone(),
                        body: sensor.body.clone(),
                        position_m: sensor.position_m,
                        quat_wxyz: sensor.quat_wxyz,
                    })
                    .collect(),
            },
            journal: runtime.log().fingerprint(),
            mode: mode.map(str::to_owned),
            campaign: None,
            case: case.cloned(),
            physics_hz: runtime.physics_rate().as_hz(),
            peak: None,
            reactions: Vec::new(),
            process: Vec::new(),
            attachments: Vec::new(),
            process_events: Vec::new(),
            interface_loads: Vec::new(),
            peak_signal: None,
            contact_gap: None,
            frames: Vec::new(),
        }
    }

    fn select_peak(&self, log: &Log, signal: Option<&str>) -> Result<Option<Peak>, String> {
        let mut peak: Option<Peak> = None;
        let mut consider = |at_ns, name: &str, unit: &str, value: f64| {
            if peak.as_ref().is_none_or(|old| {
                value.abs() > old.value.abs()
                    || (value.abs() == old.value.abs() && at_ns < old.at_ns)
            }) {
                peak = Some(Peak {
                    at_ns,
                    signal: name.into(),
                    unit: unit.into(),
                    value,
                });
            }
        };
        if let Some(name) = signal {
            let truth = log.header().truth.iter().position(|spec| spec.name == name);
            let process = self.process.iter().any(|p| p.name == name);
            match (truth, process) {
                (Some(id), false) => {
                    for e in log
                        .entries()
                        .iter()
                        .filter(|e| e.kind == LogKind::Truth && e.id as usize == id && e.valid)
                    {
                        consider(e.at.0, name, &log.header().truth[id].unit, e.value);
                    }
                }
                (None, true) => {
                    let unit = &self.process.iter().find(|p| p.name == name).unwrap().unit;
                    for p in self.process.iter().filter(|p| p.name == name) {
                        if &p.unit != unit {
                            return Err(format!("разные единицы ряда «{name}»"));
                        }
                        if let Some(value) = p.value {
                            consider(p.at_ns, name, unit, value);
                        }
                    }
                }
                (None, false) => return Err(format!("в прогоне нет ряда «{name}»")),
                (Some(_), true) => return Err(format!("неоднозначный ряд «{name}»")),
            }
        } else {
            for reaction in self
                .reactions
                .iter()
                .filter(|r| r.kind == ReactionKind::Contact)
            {
                let [x, y, z] = reaction.force_n;
                let norm = x.hypot(y).hypot(z);
                if norm > 0.0 {
                    consider(reaction.start_ns, "Модуль контактной реакции", "N", norm);
                }
            }
        }
        Ok(peak)
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum PlaybackFormat {
    Log,
    View,
}

impl PlaybackFormat {
    pub fn for_options(options: &Options) -> Option<Self> {
        if options.view.is_some() {
            Some(Self::View)
        } else {
            options.log.as_ref().map(|_| Self::Log)
        }
    }

    pub fn collect(self, runtime: &Runtime, mode: &str, case: Option<&sim_task::Case>) -> Result<Vec<u8>, String> {
        let mut bytes = Vec::new();
        match self {
            Self::Log => runtime.log().write_binary(&mut bytes).map_err(|e| e.to_string())?,
            Self::View => write_run_view(runtime, Some(mode), case, None, &mut bytes)?,
        }
        Ok(bytes)
    }
}

/// При повторе составной строки запись включается только в выбранном опыте.
#[derive(Clone)]
pub(crate) struct PlaybackCapture {
    playback: Option<CampaignPlayback>,
    format: PlaybackFormat,
    bytes: std::sync::Arc<std::sync::Mutex<Option<Vec<u8>>>>,
}

impl PlaybackCapture {
    pub fn new(playback: CampaignPlayback, format: PlaybackFormat) -> Self {
        Self { playback: Some(playback), format, bytes: Default::default() }
    }

    pub fn first(format: PlaybackFormat) -> Self {
        Self { playback: None, format, bytes: Default::default() }
    }

    pub fn records_view(&self) -> bool {
        self.format == PlaybackFormat::View
    }

    pub fn grid(&self, variant: Option<&str>, hz: u64) -> bool {
        self.playback.as_ref().is_some_and(|p| p.variant.as_deref() == variant && p.physics_hz == hz)
    }

    pub fn save(&self, runtime: &Runtime, case: Option<&sim_task::Case>) -> Result<(), String> {
        let mode = if let Some(p) = &self.playback {
            if runtime.seed() != p.seed || runtime.log().fingerprint() != p.journal
                || runtime.log().end().time.0 != p.end_ns
            {
                return Err("журнал дочернего прогона изменился при повторе".into());
            }
            &p.child.mode
        } else {
            &case.ok_or("для однопроходной записи нужен ключ опыта")?.mode
        };
        let bytes = self.format.collect(runtime, mode, case)?;
        let mut buffer = self.bytes.lock().map_err(|_| "авария записи пакета")?;
        if buffer.is_some() {
            return Err("расчётная сетка повтора исполнена дважды".into());
        }
        *buffer = Some(bytes);
        Ok(())
    }

    pub fn take(&self) -> Result<Vec<u8>, String> {
        self.bytes.lock().map_err(|_| "авария записи пакета")?
            .take().ok_or_else(|| "выбранный дочерний прогон не записан".into())
    }
}

/// Сохранить геометрию, пиксели, типизированные реакции и исходный SIMLOG.
pub fn write_run_view(
    runtime: &Runtime,
    mode: Option<&str>,
    case: Option<&sim_task::Case>,
    peak_signal: Option<&str>,
    mut out: impl Write,
) -> Result<(), String> {
    let recording = runtime
        .view_recording()
        .ok_or("запись просмотра не включена")?;
    let frames = recording.frames();
    let log = runtime.log();
    let mut meta = ViewMetadata::from_runtime(runtime, mode, case);
    meta.contact_gap = recording.contact_gap().map(str::to_owned);
    meta.frames = frames
        .iter()
        .map(|f| FrameRecord {
            signal: f.signal.0,
            captured_ns: f.captured.0,
            at_ns: f.at.0,
            width: f.width,
            height: f.height,
            format: f.format,
            valid: f.valid,
        })
        .collect();
    for row in recording.contacts() {
        for r in &row.contacts {
            meta.reactions.push(Reaction {
                kind: ReactionKind::Contact,
                pair: r.geometries.clone(),
                start_ns: row.start.0,
                end_ns: row.end.0,
                point_m: r.point_m,
                force_n: r.force_n,
                torque_nm: r.torque_nm,
            });
        }
    }
    for tick in runtime.process_journal() {
        if let Some(f) = &tick.completed {
            for r in &f.engine_step.contacts {
                meta.reactions.push(Reaction {
                    kind: ReactionKind::Contact,
                    pair: r.geometries.clone(),
                    start_ns: f.interval.start.0,
                    end_ns: f.interval.end.0,
                    point_m: r.point_m,
                    force_n: r.force_n,
                    torque_nm: r.torque_nm,
                });
            }
            for (name, weld) in &f.weld_reactions {
                if let Some(weld) = weld {
                    for side in [weld.on_first, weld.on_second] {
                        meta.reactions.push(Reaction {
                            kind: ReactionKind::Weld,
                            pair: [name.clone(), side.body.to_string()],
                            start_ns: f.interval.start.0,
                            end_ns: f.interval.end.0,
                            point_m: side.point_m,
                            force_n: side.force_n,
                            torque_nm: side.torque_nm,
                        });
                    }
                }
            }
        }
        for out in tick.outputs.values() {
            for p in &out.measurements {
                meta.process.push(Scalar {
                    at_ns: p.at.0,
                    name: p.signal.name.clone(),
                    unit: p.signal.unit.clone(),
                    value: match p.result {
                        Analysis::Analyzed { value, .. } => Some(value),
                        _ => None,
                    },
                });
            }
            for attachment in out.attachment_records() {
                if let Some(e) = &attachment.event {
                    let outcome = match &e.outcome {
                        AttachmentOutcome::Transition { from, to } => MountOutcome::Transition {
                            from: format!("{from:?}"),
                            to: format!("{to:?}"),
                        },
                        AttachmentOutcome::Rejected { reason } => MountOutcome::Rejected {
                            reason: reason.clone(),
                        },
                        AttachmentOutcome::NotAnalyzed { reason } => MountOutcome::NotAnalyzed {
                            reason: reason.clone(),
                        },
                    };
                    meta.attachments.push(MountEvent {
                        name: e.request.name.clone(),
                        requested_ns: e.request.at.0,
                        evaluated_ns: e.evaluated_at.0,
                        action: format!("{:?}", e.request.action),
                        outcome,
                    });
                }
            }
            for e in &out.events {
                meta.process_events.push(TimedProcessEvent {
                    name: e.name.clone(),
                    requested_ns: e.requested.0,
                    applied_ns: e.applied.0,
                    state: e.state.clone(),
                });
            }
            if let Some(interaction) = &out.interaction {
                use sim_runtime::process::interaction::ReactionOwner;
                for (patch, sample) in &interaction.patches {
                    if let Analysis::Analyzed { value, .. } = &sample.reaction {
                        for load in &value.loads {
                            meta.interface_loads.push(InterfaceLoad {
                                patch: patch.clone(),
                                body: load.body.clone(),
                                start_ns: sample.interval.start.0,
                                end_ns: sample.interval.end.0,
                                force_n: load.force_n,
                                torque_nm: load.torque_nm,
                                from_engine: sample.owner == ReactionOwner::Engine,
                            });
                        }
                    }
                }
            }
        }
    }
    for switch in recording.equality_changes() {
        meta.attachments.push(MountEvent {
            name: switch.name.clone(),
            requested_ns: switch.at.0,
            evaluated_ns: switch.at.0,
            action: "switch_equality".into(),
            outcome: MountOutcome::Equality {
                active: switch.active,
            },
        });
    }
    meta.attachments.sort_by_key(|e| e.evaluated_ns);
    meta.peak_signal = peak_signal.map(str::to_owned);
    meta.peak = meta.select_peak(log, peak_signal)?;
    meta.validate(log, frames)?;
    let header = serde_json::to_vec(&meta).map_err(|e| e.to_string())?;
    let checksum = view_checksum(&header, frames);
    let write = || -> io::Result<()> {
        out.write_all(MAGIC)?;
        out.write_all(&(header.len() as u64).to_le_bytes())?;
        out.write_all(&checksum.to_le_bytes())?;
        out.write_all(&header)?;
        for f in frames {
            out.write_all(&f.data)?;
        }
        log.write_binary(out)
    };
    write().map_err(|e| e.to_string())
}

pub(crate) fn bind_campaign(
    bytes: &[u8],
    binding: CampaignBinding,
    path: &Path,
) -> Result<(), String> {
    let mut view = RunView::read(bytes)?;
    view.metadata.campaign = Some(binding);
    view.metadata.validate(&view.journal, &view.frames)?;
    let header = serde_json::to_vec(&view.metadata).map_err(|e| e.to_string())?;
    let checksum = view_checksum(&header, &view.frames);
    export(path, |out| {
        out.write_all(MAGIC)?;
        out.write_all(&(header.len() as u64).to_le_bytes())?;
        out.write_all(&checksum.to_le_bytes())?;
        out.write_all(&header)?;
        for frame in &view.frames {
            out.write_all(&frame.data)?;
        }
        out.write_all(&view.binary)
    })
}

pub(crate) fn save(
    options: &Options,
    runtime: &Runtime,
    mode: Option<&str>,
    case: Option<&sim_task::Case>,
) -> Result<(), String> {
    if let Some(path) = &options.view {
        save_at(path, runtime, mode, case, options.peak.as_deref())?;
    }
    Ok(())
}

fn save_at(
    path: &Path,
    runtime: &Runtime,
    mode: Option<&str>,
    case: Option<&sim_task::Case>,
    peak_signal: Option<&str>,
) -> Result<(), String> {
    export(path, |file| {
        write_run_view(runtime, mode, case, peak_signal, file).map_err(io::Error::other)
    })
}
