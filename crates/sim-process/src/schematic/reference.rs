//! Сверка того же раскрытого опыта с ngspice. Адаптер работает отдельно от прогона.

use std::collections::{BTreeMap, BTreeSet};

use sim_circuit::netlist::{Element, ElementKind, Source};
use sim_circuit::ngspice::{NgAnalysis, NgTable, find_ngspice, prepare_netlist_with_events};
use sim_signal::Time;

use super::{InitialState, PreparedMode, Probe};
use crate::Analysis;

/// Общее окно с фиксированной целью и полосой установления.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ComparisonWindow {
    pub start: Time,
    pub end: Time,
    pub target: f64,
    pub band: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ReferenceCheck {
    DcVoltage,
    Transient(ComparisonWindow),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReferenceDifference {
    pub measured: f64,
    pub reference: f64,
    pub error: f64,
    pub tolerance: f64,
}

impl ReferenceDifference {
    pub fn matches(self) -> bool {
        self.error <= self.tolerance
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ReferenceResult {
    pub min: ReferenceDifference,
    pub max: ReferenceDifference,
    pub settling_s: Option<ReferenceDifference>,
    pub rows: usize,
}

impl ReferenceResult {
    pub fn matches(&self) -> bool {
        self.min.matches()
            && self.max.matches()
            && self.settling_s.is_none_or(ReferenceDifference::matches)
    }
}

impl PreparedMode {
    pub fn compare_ngspice(
        &self,
        checks: &BTreeMap<Probe, ReferenceCheck>,
    ) -> Result<BTreeMap<Probe, ReferenceResult>, String> {
        if checks.is_empty() {
            return Err("не указаны обязательные сравнения ngspice".into());
        }
        let ngspice = find_ngspice().ok_or("ngspice не найден")?;
        let internal = match self.solve() {
            Analysis::Analyzed { value, .. } => value,
            Analysis::NotAnalyzed { reason } => return Err(reason),
        };
        let mut net = self.net.netlist.clone();
        let mut sensors = BTreeMap::new();
        let mut probes = BTreeMap::new();
        for (probe, check) in checks {
            if !self.probes.contains(probe) {
                return Err(format!("в опыте нет наблюдения {probe:?}"));
            }
            match (check, &self.window) {
                (ReferenceCheck::DcVoltage, None) if matches!(probe, Probe::Voltage(..)) => {}
                (ReferenceCheck::Transient(window), Some(experiment))
                    if window.start < window.end
                        && window.end <= experiment.end
                        && window.target.is_finite()
                        && window.band.is_finite()
                        && window.band > 0.0
                        && (window.target - window.band).is_finite()
                        && (window.target + window.band).is_finite() => {}
                _ => return Err(format!("недопустимое окно или вид сравнения {probe:?}")),
            }
            let expression = match probe {
                Probe::Voltage(a, b) => {
                    Readout::Voltage(self.net.nodes[a].clone(), self.net.nodes[b].clone())
                }
                Probe::Current(section) | Probe::Power(section) => {
                    if self.opened.contains(section) {
                        Readout::Zero
                    } else {
                        let (name, _) = self
                            .net
                            .origins
                            .iter()
                            .find(|(_, origin)| *origin == section)
                            .ok_or_else(|| format!("нет ветви {section:?}"))?;
                        let branch = self
                            .net
                            .netlist
                            .element(name)
                            .ok_or_else(|| format!("нет элемента {name}"))?;
                        let sense = if matches!(branch.kind, ElementKind::VoltageSource(_)) {
                            format!("i({name})")
                        } else {
                            sensors
                                .entry(name.clone())
                                .or_insert_with(|| {
                                    // Нулевой источник сохраняет напряжение и измеряет ток первого вывода.
                                    let index =
                                        net.elements.iter().position(|e| &e.name == name).unwrap();
                                    let sensor_name = format!("vref_sense{index}");
                                    let inner = format!("ref_sense{index}");
                                    let first = std::mem::replace(
                                        &mut net.elements[index].nodes[0],
                                        inner.clone(),
                                    );
                                    net.elements.push(Element {
                                        name: sensor_name.clone(),
                                        nodes: vec![first, inner],
                                        kind: ElementKind::VoltageSource(Source {
                                            dc: Some(0.0),
                                            wave: None,
                                        }),
                                        line: 0,
                                    });
                                    format!("i({sensor_name})")
                                })
                                .clone()
                        };
                        if matches!(probe, Probe::Power(_)) {
                            if matches!(branch.kind, ElementKind::Mosfet { .. }) {
                                return Err("сверка мощности многовыводной модели требует всех токов выводов".into());
                            }
                            Readout::Power {
                                sense,
                                positive: branch.nodes[0].clone(),
                                negative: branch.nodes[1].clone(),
                            }
                        } else {
                            Readout::Current(sense)
                        }
                    }
                }
            };
            probes.insert(probe.clone(), expression);
        }
        // Кривые PWL задаются одинаковыми картами в обоих движках.
        let analysis = self
            .window
            .as_ref()
            .map_or(NgAnalysis::Op, |w| NgAnalysis::Tran {
                tstep: w.step.as_secs(),
                tstop: w.end.as_secs(),
                tstart: Time::ZERO.as_secs(),
                tmax: w.step.as_secs(),
                uic: matches!(w.initial, InitialState::Reactive(_)),
            });
        if matches!(analysis, NgAnalysis::Tran { .. }) {
            // Экспорт использует .tran для выбора PWL вместо DC.
            let NgAnalysis::Tran {
                tstep,
                tstop,
                tstart,
                tmax,
                uic,
            } = analysis
            else {
                unreachable!()
            };
            net.analyses = vec![sim_circuit::netlist::Analysis::Tran(
                sim_circuit::netlist::TranSpec {
                    tstep,
                    tstop,
                    tstart,
                    tmax: Some(tmax),
                    uic,
                },
            )];
        }
        let mut signals: BTreeSet<_> = probes.values().flat_map(Readout::signals).collect();
        if matches!(analysis, NgAnalysis::Tran { .. }) {
            signals.insert("time".into());
        }
        if signals.is_empty() {
            return Err("сверка требует хотя бы один физически наблюдаемый вектор".into());
        }
        let signal_names = signals
            .iter()
            .map(String::as_str)
            .filter(|&name| name != "time")
            .collect::<Vec<_>>();
        if signal_names.is_empty() {
            return Err("требуется хотя бы один наблюдаемый сигнал кроме времени".into());
        }
        let table =
            prepare_netlist_with_events(&net, &[])?.run(&ngspice, analysis, &signal_names)?;
        validate_table(&table, &signals)?;
        let reference_time = if let Some(w) = &self.window {
            let times = column(&table, "time")?;
            if times.len() < 2
                || times.windows(2).any(|two| two[0] >= two[1])
                || !same_time(*times.last().unwrap(), w.end.as_secs())
                || times[0] < 0.0
                || (times[0] > 0.0 && !matches!(w.initial, InitialState::Reactive(_)))
                || self
                    .events
                    .iter()
                    .any(|event| !times.iter().any(|&t| same_time(t, event.as_secs())))
            {
                return Err("ngspice: окно или метки PWL покрыты неполностью".into());
            }
            times
        } else {
            if table.rows.len() != 1 {
                return Err("ngspice: требуется одна рабочая точка".into());
            }
            vec![0.0]
        };
        let internal_time = internal
            .time
            .iter()
            .copied()
            .map(Time::as_secs)
            .collect::<Vec<_>>();
        checks
            .iter()
            .map(|(probe, check)| {
                let calculated = &internal.values[probe];
                let reference = probes[probe].values(&table)?;
                let result = match check {
                    ReferenceCheck::DcVoltage => {
                        // ARCHITECTURE §8.2: max(10 мВ, 1% от модуля эталонного напряжения).
                        let point = difference(
                            calculated[0],
                            reference[0],
                            0.010_f64.max(0.01 * reference[0].abs()),
                        )?;
                        ReferenceResult {
                            min: point,
                            max: point,
                            settling_s: None,
                            rows: table.rows.len(),
                        }
                    }
                    ReferenceCheck::Transient(window) => {
                        let measured = windowed(&internal_time, calculated, *window)?;
                        let expected = windowed(&reference_time, &reference, *window)?;
                        let extremes = |trace: &[(f64, f64)]| {
                            trace
                                .iter()
                                .map(|&(_, v)| v)
                                .fold([f64::INFINITY, f64::NEG_INFINITY], |[lo, hi], v| {
                                    [lo.min(v), hi.max(v)]
                                })
                        };
                        let [a, b] = extremes(&measured);
                        let [c, d] = extremes(&expected);
                        // ARCHITECTURE §8.2: 5% от модуля эталона. Нулевой эталон требует нулевого отклонения.
                        ReferenceResult {
                            min: relative(a, c)?,
                            max: relative(b, d)?,
                            settling_s: Some(relative(
                                settling(&measured, *window)?,
                                settling(&expected, *window)?,
                            )?),
                            rows: table.rows.len(),
                        }
                    }
                };
                Ok((probe.clone(), result))
            })
            .collect()
    }
}

#[derive(Clone, Debug)]
enum Readout {
    Zero,
    Voltage(String, String),
    Current(String),
    Power {
        sense: String,
        positive: String,
        negative: String,
    },
}

impl Readout {
    fn signals(&self) -> Vec<String> {
        let voltage = |name: &String| (name != "0").then(|| format!("v({name})"));
        match self {
            Self::Zero => Vec::new(),
            Self::Voltage(a, b) => [voltage(a), voltage(b)].into_iter().flatten().collect(),
            Self::Current(sense) => vec![sense.clone()],
            Self::Power {
                sense,
                positive,
                negative,
            } => [Some(sense.clone()), voltage(positive), voltage(negative)]
                .into_iter()
                .flatten()
                .collect(),
        }
    }

    fn values(&self, table: &NgTable) -> Result<Vec<f64>, String> {
        let voltage = |name: &str| {
            if name == "0" {
                Ok(vec![0.0; table.rows.len()])
            } else {
                column(table, &format!("v({name})"))
            }
        };
        let subtract = |a, b| {
            Ok::<_, String>(
                voltage(a)?
                    .into_iter()
                    .zip(voltage(b)?)
                    .map(|(a, b)| a - b)
                    .collect::<Vec<_>>(),
            )
        };
        let result = match self {
            Self::Zero => vec![0.0; table.rows.len()],
            Self::Voltage(a, b) => subtract(a, b)?,
            Self::Current(sense) => column(table, sense)?,
            Self::Power {
                sense,
                positive,
                negative,
            } => subtract(positive, negative)?
                .into_iter()
                .zip(column(table, sense)?)
                .map(|(v, i)| v * i)
                .collect(),
        };
        if result.iter().any(|v| !v.is_finite()) {
            return Err("ngspice: неконечная разность или мощность".into());
        }
        Ok(result)
    }
}

fn column(table: &NgTable, name: &str) -> Result<Vec<f64>, String> {
    table
        .column(name)
        .ok_or_else(|| format!("ngspice: нет вектора {name}"))
}

fn validate_table(table: &NgTable, signals: &BTreeSet<String>) -> Result<(), String> {
    let columns = table.columns.iter().cloned().collect::<BTreeSet<_>>();
    if table.rows.is_empty()
        || columns.len() != table.columns.len()
        || &columns != signals
        || table
            .rows
            .iter()
            .any(|row| row.len() != columns.len() || row.iter().any(|v| !v.is_finite()))
    {
        return Err("ngspice: неполная или неконечная таблица".into());
    }
    Ok(())
}

fn relative(measured: f64, reference: f64) -> Result<ReferenceDifference, String> {
    difference(measured, reference, 0.05 * reference.abs())
}

fn difference(
    measured: f64,
    reference: f64,
    tolerance: f64,
) -> Result<ReferenceDifference, String> {
    let error = (measured - reference).abs();
    if !error.is_finite() || !tolerance.is_finite() {
        return Err("неконечное сравнение".into());
    }
    Ok(ReferenceDifference {
        measured,
        reference,
        error,
        tolerance,
    })
}

fn same_time(a: f64, b: f64) -> bool {
    // Погрешность перевода целого такта в f64 и печати времени ngspice.
    a >= b.next_down().next_down() && a <= b.next_up().next_up()
}

fn windowed(
    times: &[f64],
    values: &[f64],
    window: ComparisonWindow,
) -> Result<Vec<(f64, f64)>, String> {
    let [start, end] = [window.start.as_secs(), window.end.as_secs()];
    let at = |time| match times.binary_search_by(|t| t.total_cmp(&time)) {
        Ok(index) => Some(values[index]),
        Err(right) if right > 0 && right < times.len() => {
            let left = right - 1;
            let f = (time - times[left]) / (times[right] - times[left]);
            Some((1.0 - f) * values[left] + f * values[right])
        }
        Err(0) if same_time(times[0], time) => Some(values[0]),
        Err(right) if right == times.len() && same_time(times[right - 1], time) => {
            Some(values[right - 1])
        }
        _ => None,
    };
    let mut points = vec![(start, at(start).ok_or("нет начала сравниваемого окна")?)];
    points.extend(
        times
            .iter()
            .copied()
            .zip(values.iter().copied())
            .filter(|(t, _)| start < *t && *t < end),
    );
    points.push((end, at(end).ok_or("нет конца сравниваемого окна")?));
    if points.iter().any(|&(_, v)| !v.is_finite()) {
        return Err("неконечное значение в окне сравнения".into());
    }
    Ok(points)
}

fn settling(points: &[(f64, f64)], window: ComparisonWindow) -> Result<f64, String> {
    let [lo, hi] = [window.target - window.band, window.target + window.band];
    let Some(last) = points.iter().rposition(|&(_, v)| v < lo || v > hi) else {
        return Ok(0.0);
    };
    let (t0, v0) = points[last];
    let (t1, v1) = *points
        .get(last + 1)
        .ok_or("сигнал не установился к концу окна")?;
    let boundary = if v0 < lo { lo } else { hi };
    let (offset, step) = (boundary - v0, v1 - v0);
    if !offset.is_finite() || !step.is_finite() || step == 0.0 {
        return Err("положение пересечения полосы не определено".into());
    }
    let cross = t0 + (t1 - t0) * (offset / step);
    Ok(cross - window.start.as_secs())
}
