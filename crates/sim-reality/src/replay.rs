use std::{
    collections::{BTreeMap, BTreeSet},
    path::Path,
};

use serde::{Deserialize, Serialize};
use sim_runtime::{LogEntry, LogKind, Runtime, Status};
use sim_signal::{ChannelId, Command, Time};
use sim_world::CompiledWorld;

use crate::Capture;

pub struct ReplayRuntime {
    runtime: Runtime,
    input_units: Vec<String>,
    frames: BTreeSet<u32>,
    pulse_groups: Vec<[ChannelId; 3]>,
}

impl ReplayRuntime {
    pub fn load(world: CompiledWorld, catalog: Option<&Path>) -> Result<Self, String> {
        let mut runtime = Runtime::load(world.clone())?;
        let mut input_units: Vec<_> = runtime
            .log()
            .header()
            .commands
            .iter()
            .map(|ch| ch.unit.clone())
            .collect();
        let mut pulse_groups = Vec::new();
        if !world.drives.is_empty() {
            let catalog = catalog.ok_or("для приводов нужен каталог")?;
            let drives = sim_drive::DriveCatalog::load(catalog.join("drives"))?;
            for drive in sim_drive::install(&mut runtime, &world, &drives)? {
                let accepted = runtime
                    .truth_spec(drive.accepted_command)
                    .ok_or("нет единицы команды привода")?;
                input_units[drive.command.0 as usize].clone_from(&accepted.unit);
                // Единицы дополнительных портов — контракт DriveHandle и PulseInputs.
                for (id, unit) in [
                    (drive.power, "bool"),
                    (drive.bus_voltage, "V"),
                    (drive.bridge_mode, "1"),
                ] {
                    if let Some(id) = id {
                        input_units[id.0 as usize] = unit.into();
                    }
                }
                if let Some(pulse) = drive.pulse {
                    pulse_groups.push([pulse.period_s, pulse.width_s, pulse.sequence]);
                    for (id, unit) in [
                        (pulse.period_s, "s"),
                        (pulse.width_s, "s"),
                        (pulse.sequence, "1"),
                    ] {
                        input_units[id.0 as usize] = unit.into();
                    }
                }
            }
        }
        let mut frames = BTreeSet::new();
        if !world.sensors.is_empty() {
            let catalog = catalog.ok_or("для датчиков нужен каталог")?;
            let sensors = sim_sense::SensorCatalog::load(catalog.join("sensors"))?;
            for sensor in sim_sense::install(&mut runtime, &world, &sensors)? {
                if sensor.intrinsics().is_some() {
                    frames.extend(sensor.signals().iter().map(|(_, id)| id.0));
                }
            }
        }
        runtime.record_joint_state("состояние для стендовой сверки")?;
        Ok(Self {
            runtime,
            input_units,
            frames,
            pulse_groups,
        })
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InputBinding {
    pub recorded: String,
    pub simulated: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CurveSource {
    Observation,
    Truth,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Alignment {
    Exact,
    Linear { max_gap_ns: u64 },
    Hold { max_age_ns: u64 },
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CurveBinding {
    pub model: String,
    pub recorded: String,
    pub simulated: String,
    pub source: CurveSource,
    pub alignment: Alignment,
    /// Абсолютная граница погрешности отсчёта в его единице. Из калибровки.
    pub uncertainty: Option<f64>,
}

#[derive(Debug, Serialize)]
pub struct AlignedPoint {
    pub at_ns: u64,
    pub recorded: Option<f64>,
    pub simulated: Option<f64>,
}

#[derive(Debug, Serialize)]
pub struct Gap {
    pub compared: usize,
    pub bias: f64,
    pub rms: f64,
    pub max_abs: f64,
    pub upper_max_abs_at_samples: Option<f64>,
}

#[derive(Debug, Serialize)]
pub struct CurveReport {
    pub model: String,
    pub recorded: String,
    pub simulated: String,
    pub unit: String,
    pub state: &'static str,
    pub reason: Option<&'static str>,
    pub gap: Option<Gap>,
    pub points: Vec<AlignedPoint>,
}

#[derive(Debug, Serialize)]
pub struct ReplayReport {
    pub journal: String,
    pub physics_steps: u64,
    pub unmapped_commands: Vec<String>,
    pub limitations: BTreeMap<String, Vec<String>>,
    pub curves: Vec<CurveReport>,
}

fn advance(runtime: &mut Runtime, at: Time) -> Result<(), String> {
    let status = runtime.run_until(at)?;
    if status != Status::Running || runtime.time() != at {
        return Err(format!(
            "прогон остановился на {} нс: {}",
            runtime.time().0,
            runtime.failure().unwrap_or("горизонт не достигнут")
        ));
    }
    Ok(())
}

/// Команда с ненулевой меткой действует после событий её момента, как в рантайме.
/// Команды нулевого момента задаются до первых событий.
pub fn replay(
    capture: &Capture,
    prepared: ReplayRuntime,
    inputs: &[InputBinding],
    curves: &[CurveBinding],
) -> Result<ReplayReport, String> {
    let ReplayRuntime {
        mut runtime,
        input_units,
        frames,
        pulse_groups,
    } = prepared;
    let header = runtime.log().header();
    let mut targets = BTreeSet::new();
    let mut mapping = BTreeMap::new();
    for input in inputs {
        let from = capture
            .inputs()
            .id(&input.recorded)
            .ok_or_else(|| format!("нет записанной команды «{}»", input.recorded))?;
        let to = runtime
            .channel(&input.simulated)
            .ok_or_else(|| format!("нет входа симуляции «{}»", input.simulated))?;
        let source_unit = &capture.inputs().spec(from).unwrap().unit;
        let dest_unit = &input_units[to.0 as usize];
        if source_unit != dest_unit {
            return Err(format!(
                "команда «{}»: единица «{source_unit}», нужна «{dest_unit}»",
                input.recorded
            ));
        }
        if mapping.insert(ChannelId(from.0), to).is_some() || !targets.insert(to.0 as usize) {
            return Err(format!("повтор привязки команды «{}»", input.recorded));
        }
    }
    if mapping.len() != capture.inputs().len() {
        return Err("для части записанных команд нет привязки к симуляции".into());
    }
    let packet_targets: BTreeSet<_> = capture
        .packet_inputs()
        .iter()
        .map(|id| mapping[id])
        .collect();
    let mut recognized_packets = BTreeSet::new();
    for group in &pulse_groups {
        let mapped = group.iter().any(|id| targets.contains(&(id.0 as usize)));
        if mapped && group.iter().any(|id| !packet_targets.contains(id)) {
            return Err("все три поля импульсного входа должны быть объявлены пакетными".into());
        }
        recognized_packets.extend(
            group
                .iter()
                .filter(|id| packet_targets.contains(id))
                .copied(),
        );
    }
    if recognized_packets != packet_targets {
        return Err("пакетные команды требуют импульсного входа симуляции".into());
    }
    let unmapped_commands = header
        .commands
        .iter()
        .enumerate()
        .filter(|(index, _)| !targets.contains(index))
        .map(|(_, spec)| spec.name.clone())
        .collect();

    let mut bindings = Vec::with_capacity(curves.len());
    for curve in curves {
        let specs = match curve.source {
            CurveSource::Observation => &header.observations,
            CurveSource::Truth => &header.truth,
        };
        let (id, spec) = specs
            .iter()
            .enumerate()
            .find(|(_, s)| s.name == curve.simulated)
            .ok_or_else(|| format!("нет сигнала симуляции «{}»", curve.simulated))?;
        if curve.uncertainty.is_some_and(|x| !x.is_finite() || x < 0.0)
            || matches!(
                curve.alignment,
                Alignment::Linear { max_gap_ns: 0 } | Alignment::Hold { max_age_ns: 0 }
            )
        {
            return Err(format!(
                "некорректное выравнивание или погрешность «{}»",
                curve.recorded
            ));
        }
        if curve.source == CurveSource::Observation && frames.contains(&(id as u32)) {
            return Err("отпечаток кадра нельзя сравнивать как числовой отсчёт".into());
        }
        bindings.push((
            curve,
            id as u32,
            spec.unit.clone(),
            capture.series(&curve.recorded, &spec.unit)?,
        ));
    }
    let mut last_pulses = vec![None; pulse_groups.len()];
    for (at, command) in capture.commands() {
        if *at != Time::ZERO {
            advance(&mut runtime, *at)?;
        }
        let targets: Vec<_> = command
            .targets
            .iter()
            .map(|&(id, v)| (mapping[&id], v))
            .collect();
        for (group, previous) in pulse_groups.iter().zip(&mut last_pulses) {
            let packet = group.map(|id| {
                targets
                    .iter()
                    .find_map(|&(target, v)| (target == id).then_some(v))
            });
            if packet == [None; 3] {
                continue;
            }
            let [Some(period), Some(width), Some(seq)] = packet else {
                return Err(format!("неполный импульсный пакет в {} нс", at.0));
            };
            // Целые номера до 2^53 исключая границу, как в приёмнике sim-drive.
            let sequence_limit = (1_u64 << f64::MANTISSA_DIGITS) as f64;
            if width <= 0.0
                || width > period
                || !(0.0..sequence_limit).contains(&seq)
                || seq.fract() != 0.0
                || previous.is_some_and(|(old, _)| seq <= old)
            {
                return Err(format!("некорректный импульсный пакет в {} нс", at.0));
            }
            let tick = runtime.steps();
            if previous.is_some_and(|(_, last_tick)| last_tick == tick) {
                return Err(format!(
                    "сетка физики не разрешает два пакета до одного такта, {} нс",
                    at.0
                ));
            }
            *previous = Some((seq, tick));
        }
        runtime.act(&Command { targets })?;
    }
    advance(&mut runtime, capture.horizon())?;

    let log = runtime.log();
    // Индекс строится один раз. Порядок соответствует журналу рантайма.
    let mut streams = BTreeMap::<(CurveSource, u32), Vec<&LogEntry>>::new();
    for (curve, id, _, _) in &bindings {
        streams.entry((curve.source, *id)).or_default();
    }
    for entry in log.entries() {
        let source = match entry.kind {
            LogKind::Observation => CurveSource::Observation,
            LogKind::Truth => CurveSource::Truth,
            LogKind::Command(_) => continue,
        };
        if let Some(stream) = streams.get_mut(&(source, entry.id)) {
            if !entry.value.is_finite() || stream.last().is_some_and(|p| p.at >= entry.at) {
                return Err(format!(
                    "нечисловой или неоднозначный симуляционный ряд {} в {} нс",
                    entry.id, entry.at.0
                ));
            }
            stream.push(entry);
        }
    }
    let mut reports = Vec::with_capacity(bindings.len());
    for (curve, id, unit, samples) in bindings {
        let stream = &streams[&(curve.source, id)];
        let points = samples
            .iter()
            .map(|p| AlignedPoint {
                at_ns: p.at.0,
                recorded: p.valid.then_some(p.value),
                simulated: aligned(stream, p.at, curve.alignment),
            })
            .collect::<Vec<_>>();
        if points
            .iter()
            .any(|p| p.simulated.is_some_and(|v| !v.is_finite()))
        {
            return Err("выравнивание вышло за диапазон f64".into());
        }
        let gap = metrics(&points, curve.uncertainty)?;
        let (state, reason) = match &gap {
            None => ("not_analyzed", Some("нет пар действительных отсчётов")),
            Some(gap) if gap.compared != points.len() => (
                "partial",
                Some("часть исходных меток не покрыта действительными значениями"),
            ),
            Some(_) => ("compared", None),
        };
        reports.push(CurveReport {
            model: curve.model.clone(),
            recorded: curve.recorded.clone(),
            simulated: curve.simulated.clone(),
            unit,
            state,
            reason,
            gap,
            points,
        });
    }
    let mut limitations = BTreeMap::<String, Vec<String>>::new();
    for item in runtime
        .simplifications()
        .iter()
        .filter(|item| item.blocks_proof)
    {
        limitations
            .entry(item.subject.clone())
            .or_default()
            .push(item.text.clone());
    }
    Ok(ReplayReport {
        journal: format!("{:016x}", log.fingerprint()),
        physics_steps: log.end().steps,
        curves: reports,
        unmapped_commands,
        limitations,
    })
}

fn aligned(stream: &[&LogEntry], at: Time, alignment: Alignment) -> Option<f64> {
    let i = match stream.binary_search_by_key(&at, |p| p.at) {
        Ok(i) => return stream[i].valid.then_some(stream[i].value),
        Err(i) => i,
    };
    let left = stream.get(i.checked_sub(1)?)?;
    let right = stream.get(i)?;
    if !left.valid {
        return None;
    }
    match alignment {
        Alignment::Exact => None,
        Alignment::Hold { max_age_ns } if at.0 - left.at.0 <= max_age_ns => Some(left.value),
        Alignment::Linear { max_gap_ns } if right.valid && right.at.0 - left.at.0 <= max_gap_ns => {
            let fraction = (at.0 - left.at.0) as f64 / (right.at.0 - left.at.0) as f64;
            Some((1.0 - fraction) * left.value + fraction * right.value)
        }
        _ => None,
    }
}

fn metrics(points: &[AlignedPoint], uncertainty: Option<f64>) -> Result<Option<Gap>, String> {
    let residuals = || {
        points
            .iter()
            .filter_map(|p| Some(p.simulated? - p.recorded?))
    };
    let mut count = 0;
    let mut scale: f64 = 0.0;
    for err in residuals() {
        if !err.is_finite() {
            return Err("разница кривых вышла за диапазон f64".into());
        }
        count += 1;
        scale = scale.max(err.abs());
    }
    if count == 0 {
        return Ok(None);
    }
    let mut bias = 0.0;
    let mut rms = 0.0;
    if scale > 0.0 {
        for err in residuals() {
            let normalized = err / scale;
            bias += normalized;
            rms += normalized * normalized;
        }
    }
    let upper = uncertainty.map(|u| scale + u);
    if upper.is_some_and(|v| !v.is_finite()) {
        return Err("граница разрыва вышла за диапазон f64".into());
    }
    Ok(Some(Gap {
        compared: count,
        bias: scale * (bias / count as f64),
        rms: scale * (rms / count as f64).sqrt(),
        max_abs: scale,
        upper_max_abs_at_samples: upper,
    }))
}
